#!/usr/bin/env python3
"""Bütçe kesinti senaryoları: maliyet, kütle, hover süresi ve limit etkisi.

config/budget/scenarios.yaml içindeki yamaları temel donanım profiline uygular ve her senaryo
ile her paket için tools/budget_calc.py hesabını yeniden yapar.

Kullanım:
    python3 tools/scenarios.py              # metin özeti
    python3 tools/scenarios.py --markdown   # senaryo ve paket tabloları
"""
from __future__ import annotations

import argparse
import copy
import sys
from dataclasses import dataclass
from pathlib import Path

import yaml

sys.path.insert(0, str(Path(__file__).resolve().parent))
import budget_calc  # noqa: E402

ROOT = Path(__file__).resolve().parents[1]
SCENARIOS = ROOT / "config" / "budget" / "scenarios.yaml"
LIST_TARGETS = ("components", "ground_equipment")
ADJUSTABLE = ("mass_g", "power_w", "price_usd")


class ScenarioError(ValueError):
    pass


@dataclass
class Result:
    id: str
    title: str
    budget: budget_calc.Budget
    d_airframe_usd: float
    d_ground_usd: float
    d_auw_g: float
    d_hover_min: float

    @property
    def d_total_usd(self) -> float:
        return self.d_airframe_usd + self.d_ground_usd


def load(path: Path = SCENARIOS) -> dict:
    with open(path, encoding="utf-8") as fh:
        return yaml.safe_load(fh)


def base_profile(spec: dict) -> dict:
    return budget_calc.load(budget_calc.HW_DIR / f"{spec['base']}.yaml")


def _resolve(profile: dict, op: dict, sid: str) -> tuple[list | None, int | None, dict]:
    """İşlemin hedefini bulur: (liste, indeks, öğe) — tekil sözlük hedeflerinde liste None'dır."""
    target = op.get("target", "components")
    if target in LIST_TARGETS:
        items = profile.get(target, [])
        hits = [i for i, item in enumerate(items) if op.get("match", "\0") in item.get("name", "")]
        if len(hits) != 1:
            raise ScenarioError(f"{sid}: {target} içinde {op.get('match')!r} {len(hits)} öğeyle eşleşti (1 olmalı)")
        return items, hits[0], items[hits[0]]
    node = profile
    for key in target.split("."):
        if not isinstance(node, dict) or key not in node:
            raise ScenarioError(f"{sid}: hedef {target!r} profilde yok")
        node = node[key]
    if not isinstance(node, dict):
        raise ScenarioError(f"{sid}: hedef {target!r} bir sözlük değil")
    return None, None, node


def apply(profile: dict, scenario: dict) -> dict:
    """Senaryonun işlemlerini profilin bir kopyasına uygular."""
    out = copy.deepcopy(profile)
    sid = scenario["id"]
    for op in scenario.get("ops", []):
        kind = op.get("op")
        items, idx, item = _resolve(out, op, sid)
        if kind == "adjust":
            deltas = {k: op[k] for k in ADJUSTABLE if k in op}
            if not deltas:
                raise ScenarioError(f"{sid}: adjust işleminde fark yok")
            for key, delta in deltas.items():
                if item.get(key) is None and key == "price_usd":
                    raise ScenarioError(f"{sid}: {item.get('name')} fiyatsız, ayarlanamaz")
                item[key] = round(item.get(key, 0.0) + delta, 4)
                if item[key] < 0:
                    raise ScenarioError(f"{sid}: {item.get('name')} {key} negatif oldu")
        elif kind == "replace":
            new = dict(op["with"])
            if items is None:
                item.clear()
                item.update(new)
            else:
                items[idx] = new
        elif kind == "remove":
            if items is None:
                raise ScenarioError(f"{sid}: yalnızca liste öğeleri çıkarılabilir")
            del items[idx]
        else:
            raise ScenarioError(f"{sid}: bilinmeyen işlem {kind!r}")
    return out


def _result(rid: str, title: str, profile: dict, base: budget_calc.Budget) -> Result:
    b = budget_calc.compute(profile)
    return Result(rid, title, b, b.airframe_cost_usd - base.airframe_cost_usd,
                  b.ground_cost_usd - base.ground_cost_usd, b.auw_g - base.auw_g,
                  b.hover_time_min - base.hover_time_min)


def check_package(spec: dict, package: dict) -> None:
    by_id = {s["id"]: s for s in spec["scenarios"]}
    ids = package["scenarios"]
    if len(set(ids)) != len(ids):
        raise ScenarioError(f"paket {package['id']}: tekrar eden senaryo")
    for sid in ids:
        if sid not in by_id:
            raise ScenarioError(f"paket {package['id']}: bilinmeyen senaryo {sid!r}")
        clash = set(by_id[sid].get("excludes", [])) & set(ids)
        if clash:
            raise ScenarioError(f"paket {package['id']}: {sid} ile {', '.join(sorted(clash))} birlikte olamaz")


def package_profile(spec: dict, package: dict, profile: dict | None = None) -> dict:
    """Paketin senaryolarını sırayla uygular; aynı öğeyi değiştiren/çıkaran iki senaryo hata verir."""
    check_package(spec, package)
    by_id = {s["id"]: s for s in spec["scenarios"]}
    patched = profile if profile is not None else base_profile(spec)
    for sid in package["scenarios"]:
        patched = apply(patched, by_id[sid])
    return patched


def run(spec: dict) -> tuple[budget_calc.Budget, list[Result], list[Result]]:
    profile = base_profile(spec)
    base = budget_calc.compute(profile)
    if len({s["id"] for s in spec["scenarios"]}) != len(spec["scenarios"]):
        raise ScenarioError("senaryo kimlikleri benzersiz olmalı")
    singles = [_result(s["id"], s["title"], apply(profile, s), base) for s in spec["scenarios"]]
    packages = [_result(pkg["id"], pkg["title"], package_profile(spec, pkg, profile), base)
                for pkg in spec.get("packages", [])]
    return base, singles, packages


def _usd(v: float) -> str:
    return f"−${-v:,.0f}" if v < -0.5 else (f"+${v:,.0f}" if v > 0.5 else "$0")


def _signed(v: float, unit: str, fmt: str = ".0f") -> str:
    return f"{v:+{fmt}} {unit}".replace("-", "−")


def render_text(spec: dict, base: budget_calc.Budget, singles: list[Result], packages: list[Result]) -> str:
    by_id = {s["id"]: s for s in spec["scenarios"]}
    lines = [f"Temel: {base.name} — hava aracı ${base.airframe_cost_usd:,.0f} + yer ${base.ground_cost_usd:,.0f}"
             f" = ${base.airframe_cost_usd + base.ground_cost_usd:,.0f}; AUW {base.auw_g:.0f} g;"
             f" hover {base.hover_time_min:.1f} dk; T/W {base.tw_ratio_battery_limited:.2f}", "", "Senaryolar:"]
    for r in singles:
        s = by_id[r.id]
        lines.append(f"  {r.id:<20} {_usd(r.d_total_usd):>7}  {_signed(r.d_auw_g, 'g'):>7}"
                     f"  {_signed(r.d_hover_min, 'dk', '.1f'):>8}  risk {s['risk']:<6} {s['scope']:<7} {s['title']}")
    lines += ["", "Paketler:"]
    for r in packages:
        b = r.budget
        lines.append(f"  {r.title}: hava aracı ${b.airframe_cost_usd:,.0f} + yer ${b.ground_cost_usd:,.0f}"
                     f" (tasarruf ${-r.d_total_usd:,.0f}); AUW {b.auw_g:.0f} g; hover {b.hover_time_min:.1f} dk;"
                     f" T/W {b.tw_ratio_battery_limited:.2f}; limitler {'OK' if b.ok else 'HATA'}")
    return "\n".join(lines)


def render_markdown(spec: dict, base: budget_calc.Budget, singles: list[Result], packages: list[Result]) -> str:
    by_id = {s["id"]: s for s in spec["scenarios"]}
    out = [f"Temel: **{base.name}** — hava aracı ${base.airframe_cost_usd:,.0f} + yer ekipmanı "
           f"${base.ground_cost_usd:,.0f}; {base.auw_g:.0f} g; hover {base.hover_time_min:.1f} dk", "",
           "| Senaryo | Maliyet farkı | Δ kütle | Δ hover | Risk | Kapsam | Etki |",
           "|---|---:|---:|---:|---|---|---|"]
    for r in singles:
        s = by_id[r.id]
        out.append(f"| {s['title']} | {_usd(r.d_total_usd)} | {_signed(r.d_auw_g, 'g')} "
                   f"| {_signed(r.d_hover_min, 'dk', '.1f')} | {s['risk']} | {s['scope']} | {s['effect']} |")
    out += ["", "| Paket | Hava aracı | Yer ekipmanı | Toplam | Tasarruf | Kütle | Hover | T/W (batarya) | Limitler |",
            "|---|---:|---:|---:|---:|---:|---:|---:|---|"]
    total0 = base.airframe_cost_usd + base.ground_cost_usd
    out.append(f"| Temel (Seviye A) | ${base.airframe_cost_usd:,.0f} | ${base.ground_cost_usd:,.0f} | ${total0:,.0f} | — "
               f"| {base.auw_g:.0f} g | {base.hover_time_min:.1f} dk | {base.tw_ratio_battery_limited:.2f} | "
               f"{'✅' if base.ok else '⚠️'} |")
    for r in packages:
        b = r.budget
        out.append(f"| {r.title} | ${b.airframe_cost_usd:,.0f} | ${b.ground_cost_usd:,.0f} "
                   f"| ${b.airframe_cost_usd + b.ground_cost_usd:,.0f} | ${-r.d_total_usd:,.0f} "
                   f"(%{-100 * r.d_total_usd / total0:.0f}) | {b.auw_g:.0f} g | {b.hover_time_min:.1f} dk "
                   f"| {b.tw_ratio_battery_limited:.2f} | {'✅' if b.ok else '⚠️'} |")
    return "\n".join(out)


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--file", type=Path, default=SCENARIOS)
    ap.add_argument("--markdown", action="store_true")
    args = ap.parse_args(argv)
    spec = load(args.file)
    try:
        base, singles, packages = run(spec)
    except ScenarioError as exc:
        print("HATA:", exc, file=sys.stderr)
        return 1
    render = render_markdown if args.markdown else render_text
    print(render(spec, base, singles, packages))
    return 0 if all(r.budget.ok for r in packages) else 1


if __name__ == "__main__":
    sys.exit(main())
