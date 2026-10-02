#!/usr/bin/env python3
"""İtki standı ölçümü → donanım profili itki eğrisi, THR_MDL_FAC, verim ve kurulum katsayısı.

CSV sütunları (başlık satırı zorunlu, '#' ile başlayan satırlar yorum):
    throttle_pct, thrust_g, voltage_v, current_a
Güç = gerilim × akım (elektriksel, ESC girişi). THR_MDL_FAC için %100 gaz satırı gerekir.

Kullanım:
    python3 tools/thrust_stand.py serbest.csv                          # YAML thrust_curve bloğu + özet
    python3 tools/thrust_stand.py serbest.csv --guarded korumali.csv   # + installation_factor
    python3 tools/thrust_stand.py serbest.csv --profile config/hardware/tier-a-ekonomik.yaml
"""
from __future__ import annotations

import argparse
import copy
import csv
import math
import sys
from dataclasses import dataclass
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import budget_calc  # noqa: E402

COLUMNS = ("throttle_pct", "thrust_g", "voltage_v", "current_a")


@dataclass
class Row:
    throttle: float     # 0–1
    thrust_g: float
    voltage_v: float
    current_a: float

    @property
    def power_w(self) -> float:
        return self.voltage_v * self.current_a

    @property
    def efficiency(self) -> float:
        return self.thrust_g / self.power_w if self.power_w > 0 else float("nan")


def read_csv(path: Path) -> list[Row]:
    lines = [ln for ln in path.read_text(encoding="utf-8").splitlines() if ln.strip() and not ln.lstrip().startswith("#")]
    reader = csv.DictReader(lines)
    missing = [c for c in COLUMNS if c not in (reader.fieldnames or [])]
    if missing:
        raise ValueError(f"{path}: eksik sütun(lar): {', '.join(missing)}")
    rows = []
    for i, rec in enumerate(reader, 2):
        try:
            u = float(rec["throttle_pct"]) / 100.0
            row = Row(u, float(rec["thrust_g"]), float(rec["voltage_v"]), float(rec["current_a"]))
        except (TypeError, ValueError) as exc:
            raise ValueError(f"{path}: satır {i}: sayı okunamadı ({exc})") from exc
        if not 0.0 < u <= 1.0 or row.thrust_g <= 0 or row.voltage_v <= 0 or row.current_a <= 0:
            raise ValueError(f"{path}: satır {i}: değerler pozitif ve gaz %0–100 olmalı")
        rows.append(row)
    rows.sort(key=lambda r: r.throttle)
    for a, b in zip(rows, rows[1:]):
        if b.thrust_g <= a.thrust_g or b.power_w <= a.power_w:
            raise ValueError(f"{path}: itki ve güç gazla kesin artmalı (%{a.throttle * 100:g} → %{b.throttle * 100:g})")
    if len(rows) < 2:
        raise ValueError(f"{path}: en az 2 ölçüm satırı gerekli")
    return rows


def thrust_curve(rows: list[Row], source: str) -> dict:
    """config/hardware/*.yaml → propulsion.thrust_curve bloğu."""
    curve = {
        "source": source,
        "voltage_v": round(sum(r.voltage_v for r in rows) / len(rows), 2),
        "points": [[round(r.thrust_g, 1), round(r.power_w, 1)] for r in rows],
        "throttle_points": [[round(r.throttle, 3), round(r.thrust_g, 1)] for r in rows],
    }
    budget_calc._curve(curve["points"])            # aynı doğrulama (artan, ≥ 2 nokta)
    return curve


def installation_factor(free: list[Row], guarded: list[Row]) -> float:
    """Aynı elektrik gücünde korumalı / serbest itki oranının ortalaması (ortak güç aralığında)."""
    curve = budget_calc._curve([[r.thrust_g, r.power_w] for r in free])
    lo, hi = curve[0][1], curve[-1][1]
    ratios = [g.thrust_g / budget_calc.thrust_at_power(curve, g.power_w)
              for g in guarded if lo <= g.power_w <= hi]
    if not ratios:
        raise ValueError("korumalı ölçümün güç aralığı serbest ölçümle örtüşmüyor")
    return sum(ratios) / len(ratios)


def render_yaml(curve: dict, k: float | None) -> str:
    lines = ["  installation_factor: %.2f" % k] if k is not None else []
    lines += ["  thrust_curve:",
              f'    source: "{curve["source"]}"',
              f"    voltage_v: {curve['voltage_v']}",
              "    points:"]
    lines += [f"      - [{t:g}, {p:g}]" for t, p in curve["points"]]
    lines.append("    throttle_points:")
    lines += [f"      - [{u:g}, {t:g}]" for u, t in curve["throttle_points"]]
    return "\n".join(lines)


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("csv", type=Path, help="serbest (korumasız) ölçüm")
    ap.add_argument("--guarded", type=Path, help="koruma + ağ takılıyken aynı motor/pervane ölçümü")
    ap.add_argument("--profile", type=Path, help="eğriyi bu donanım profiline uygulayıp bütçeyi yeniden hesapla")
    args = ap.parse_args(argv)

    rows = read_csv(args.csv)
    curve = thrust_curve(rows, f"itki standı: {args.csv.name}")
    print("Ölçüm:")
    print("  gaz  itki (g)  güç (W)  verim (g/W)")
    for r in rows:
        print(f"  {r.throttle * 100:3.0f}%  {r.thrust_g:8.0f}  {r.power_w:7.1f}  {r.efficiency:6.2f}")
    k = None
    if args.guarded:
        k = installation_factor(rows, read_csv(args.guarded))
        print(f"Kurulum katsayısı (installation_factor): {k:.3f}  → koruma + ağ itki kaybı %{100 * (1 - k):.1f}")
    if any(abs(r.throttle - 1.0) < 1e-9 for r in rows):
        print(f"THR_MDL_FAC (PX4 itki modeli): {budget_calc.fit_thr_mdl_fac(curve['throttle_points']):.2f}")
    print("\nconfig/hardware/<profil>.yaml → propulsion altına:")
    print(render_yaml(curve, k))
    if args.profile:
        profile = copy.deepcopy(budget_calc.load(args.profile))
        prop = profile["propulsion"]
        prop["thrust_curve"] = curve
        if k is not None:
            prop["installation_factor"] = round(k, 3)
        b = budget_calc.compute(profile)
        print("\nÖlçülen eğriyle bütçe:")
        print(budget_calc.render_text(b))
        return 0 if b.ok else 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
