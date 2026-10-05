"""Mass properties of the registered aircraft (ARCHITECTURE.md §6.8).

* Manufactured parts: mesh volume x effective density (layup areal mass / thickness, or material density).
* Purchased parts: datasheet ``mass_kg``; CG and inertia from their envelope mesh scaled to that mass.
* Consumables (``process == "consumable"``, e.g. fuel contents): scaled per loading case.
* Loading cases from ``spec.mass.cases``: ``[{name, fuel_fraction, payload: bool, note}]``.
* Budget cross-check against ``spec.mass.budget``: ``{group: {target_kg, tol_kg}}``.

CLI:  python3 -m ucav250.analysis.mass   -> out/mass.md, out/mass.json (exit 1 if a budget/CG limit fails)
"""
from __future__ import annotations

import json
import sys
from collections import defaultdict

import numpy as np

from ..core.parts import Part, Registry

CONSUMABLE = "consumable"


def part_props(reg: Registry, p: Part) -> dict:
    mesh = p.mesh
    vol_props = mesh.mass_props(1.0)                       # unit density: mass == volume
    m = reg.mass(p)
    k = m / max(vol_props["mass"], 1e-15)
    return {"id": p.id, "group": p.group, "mass": m, "cg": vol_props["cg"], "inertia": vol_props["inertia"] * k}


def combine(items: list[dict]) -> dict:
    M = sum(i["mass"] for i in items)
    if M <= 0:
        return {"mass": 0.0, "cg": np.zeros(3), "inertia": np.zeros((3, 3))}
    cg = sum(i["mass"] * i["cg"] for i in items) / M
    I = np.zeros((3, 3))
    for i in items:
        d = i["cg"] - cg
        I += i["inertia"] + i["mass"] * (np.dot(d, d) * np.eye(3) - np.outer(d, d))
    return {"mass": M, "cg": cg, "inertia": I}


def case_items(reg: Registry, props: dict, case: dict) -> list[dict]:
    out = []
    ff = float(case.get("fuel_fraction", 1.0))
    with_payload = bool(case.get("payload", True))
    for pid, pr in props.items():
        p = reg.parts[pid]
        if p.process == CONSUMABLE:
            pr = dict(pr, mass=pr["mass"] * ff, inertia=pr["inertia"] * ff)
        if p.group == "payload" and not with_payload:
            continue
        out.append(pr)
    return out


def mac_fraction(reg: Registry, x_cg: float) -> float | None:
    st = reg.spec.get("stability", {}) or {}
    x_le, mac = st.get("mac_le_x"), st.get("mac")
    if x_le is None or not mac:
        return None
    return (x_cg - float(x_le)) / float(mac)


def run(reg: Registry, write: bool = True) -> dict:
    props = {pid: part_props(reg, p) for pid, p in reg.parts.items()}
    groups = defaultdict(list)
    for pid, pr in props.items():
        if reg.parts[pid].process != CONSUMABLE:
            groups[reg.parts[pid].group].append(pr)
    by_group = {g: combine(v) for g, v in groups.items()}
    empty = combine([pr for pid, pr in props.items() if reg.parts[pid].process != CONSUMABLE
                     and reg.parts[pid].group != "payload"])
    spec_mass = reg.spec.get("mass", {}) or {}
    cases = spec_mass.get("cases") or [{"name": "mtow", "fuel_fraction": 1.0, "payload": True}]
    res_cases = {}
    st = reg.spec.get("stability", {}) or {}
    cg_lo, cg_hi = (st.get("cg_range_x") or [None, None])
    failures = []
    for c in cases:
        tot = combine(case_items(reg, props, c))
        frac = mac_fraction(reg, float(tot["cg"][0]))
        res_cases[c["name"]] = {"mass": tot["mass"], "cg": tot["cg"].tolist(), "inertia": tot["inertia"].tolist(),
                                "cg_mac": frac}
        if cg_lo is not None and not (cg_lo - 1e-9 <= tot["cg"][0] <= cg_hi + 1e-9):
            failures.append(f"{c['name']}: CG x {tot['cg'][0]:.4f} outside [{cg_lo}, {cg_hi}]")
        if abs(tot["cg"][1]) > 0.005:
            failures.append(f"{c['name']}: lateral CG {tot['cg'][1]*1000:.1f} mm off centre")
    budget = spec_mass.get("budget", {}) or {}
    rows = []
    for g, b in budget.items():
        got = by_group.get(g, {"mass": 0.0})["mass"]
        tgt, tol = float(b["target_kg"]), float(b.get("tol_kg", 0.05 * float(b["target_kg"]) + 0.1))
        ok = abs(got - tgt) <= tol
        rows.append({"group": g, "computed_kg": got, "target_kg": tgt, "tol_kg": tol, "ok": ok})
        if not ok:
            failures.append(f"budget {g}: {got:.2f} kg vs {tgt:.2f} ± {tol:.2f}")
    mtow = spec_mass.get("mtow_kg")
    if mtow and "mtow" in res_cases and res_cases["mtow"]["mass"] > float(mtow) + 1e-6:
        failures.append(f"MTOW case {res_cases['mtow']['mass']:.2f} kg exceeds spec mtow {mtow}")
    S = {"parts": len(props), "empty": {"mass": empty["mass"], "cg": empty["cg"].tolist()},
         "by_group": {g: {"mass": v["mass"], "cg": v["cg"].tolist()} for g, v in by_group.items()},
         "cases": res_cases, "budget": rows, "failures": failures, "ok": not failures,
         "top_parts": sorted(({"id": k, "mass": v["mass"]} for k, v in props.items()), key=lambda r: -r["mass"])[:25]}
    if write:
        from ..core.spec import OUT_DIR
        OUT_DIR.mkdir(parents=True, exist_ok=True)
        (OUT_DIR / "mass.json").write_text(json.dumps(S, indent=1, default=float), encoding="utf-8")
        (OUT_DIR / "mass.md").write_text(report_md(S), encoding="utf-8")
    return S


def _f(x, n=2):
    return f"{x:,.{n}f}".replace(",", " ").replace(".", ",")


def report_md(S: dict) -> str:
    L = ["# YK-250 kütle özellikleri", "",
         f"Boş kütle (faydalı yük ve yakıt hariç): **{_f(S['empty']['mass'])} kg**, AG x = "
         f"{_f(S['empty']['cg'][0], 3)} m.", "", "## Yükleme durumları", "",
         "| Durum | Kütle (kg) | AG x (m) | AG y (mm) | AG z (m) | %OAK (MAC) | Ixx | Iyy | Izz (kg·m²) |",
         "|---|---:|---:|---:|---:|---:|---:|---:|---:|"]
    for n, c in S["cases"].items():
        I = np.asarray(c["inertia"])
        mac = "—" if c["cg_mac"] is None else _f(100 * c["cg_mac"], 1)
        L.append(f"| {n} | {_f(c['mass'])} | {_f(c['cg'][0], 4)} | {_f(c['cg'][1]*1000, 1)} | {_f(c['cg'][2], 4)} | "
                 f"{mac} | {_f(I[0,0], 1)} | {_f(I[1,1], 1)} | {_f(I[2,2], 1)} |")
    L += ["", "## Gruplar", "", "| Grup | Kütle (kg) | AG x (m) |", "|---|---:|---:|"]
    for g, v in sorted(S["by_group"].items(), key=lambda kv: -kv[1]["mass"]):
        L.append(f"| {g} | {_f(v['mass'])} | {_f(v['cg'][0], 3)} |")
    if S["budget"]:
        L += ["", "## Kütle bütçesi karşılaştırması", "", "| Grup | Hesap (kg) | Bütçe (kg) | Tolerans | Durum |",
              "|---|---:|---:|---:|---|"]
        for r in S["budget"]:
            L.append(f"| {r['group']} | {_f(r['computed_kg'])} | {_f(r['target_kg'])} | ±{_f(r['tol_kg'])} | "
                     f"{'✓' if r['ok'] else '✗'} |")
    L += ["", "## En ağır 25 parça", "", "| Parça | Kütle (kg) |", "|---|---:|"]
    L += [f"| {r['id']} | {_f(r['mass'], 3)} |" for r in S["top_parts"]]
    if S["failures"]:
        L += ["", "## Sorunlar", ""] + [f"- {x}" for x in S["failures"]]
    return "\n".join(L) + "\n"


def main(argv=None) -> int:
    from ..core.assemble import build_registry
    S = run(build_registry())
    print(json.dumps({"ok": S["ok"], "failures": S["failures"],
                      "cases": {k: round(v["mass"], 2) for k, v in S["cases"].items()}}))
    return 0 if S["ok"] else 1


if __name__ == "__main__":
    sys.exit(main())
