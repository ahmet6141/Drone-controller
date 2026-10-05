"""Bill of materials from the registry -> out/bom.csv (parts) and out/bom_fasteners.csv (hardware roll-up).

Columns (parts): part_number, name_tr, name_en, group, side, qty, material, process, thickness_mm, layup,
mass_kg_each, mass_kg_total, purchased, vendor, assembly_step, parent, notes.
Fasteners are rolled up by standard designation + nut/insert designation with total quantity and torque.
Consumables (fuel contents) are excluded.

CLI:  python3 -m ucav250.outputs.bom
"""
from __future__ import annotations

import csv
import sys
from collections import OrderedDict

from ..core.parts import GROUPS, Registry, layup_props


def part_rows(reg: Registry) -> list[dict]:
    mats = reg.spec.get("materials", {})
    procs = reg.spec.get("processes", {})
    rows = []
    for p in sorted(reg.parts.values(), key=lambda q: (list(GROUPS).index(q.group), q.id)):
        if p.process == "consumable" or p.group == "hardware":
            continue
        t = p.thickness
        if t is None and p.layup:
            t = layup_props(reg.spec, p.layup)["thickness"]
        m = reg.mass(p)
        rows.append(OrderedDict(
            part_number=p.id, name_tr=p.name_tr, name_en=p.name, group=p.group, side=p.side, qty=1,
            material=mats.get(p.material, {}).get("name", p.material), process=procs.get(p.process, {}).get(
                "name", p.process),
            thickness_mm="" if t is None else round(t * 1000, 2), layup=p.layup or "",
            mass_kg_each=round(m, 4), mass_kg_total=round(m, 4), purchased="evet" if p.purchased else "hayır",
            vendor=p.vendor, assembly_step=p.step, parent=p.parent or "", notes=p.notes))
    return rows


def fastener_rows(reg: Registry) -> list[dict]:
    roll: dict[tuple, dict] = OrderedDict()
    for f in reg.fasteners():
        key = (f.kind, f.spec, f.nut)
        r = roll.setdefault(key, OrderedDict(kind=f.kind, designation=f.spec, nut_or_insert=f.nut, qty=0,
                                             torque_nm="" if f.torque_nm is None else f.torque_nm,
                                             first_step=f.step, used_in=set()))
        r["qty"] += 1
        r["first_step"] = min(r["first_step"], f.step)
        r["used_in"].update(f.joins)
    out = []
    for r in roll.values():
        r = OrderedDict(r)
        used = sorted(r.pop("used_in"))
        r["used_in"] = " ".join(used[:12]) + (" …" if len(used) > 12 else "")
        out.append(r)
    return out


def write(reg: Registry) -> dict:
    from ..core.spec import OUT_DIR
    OUT_DIR.mkdir(parents=True, exist_ok=True)
    pr = part_rows(reg)
    fr = fastener_rows(reg)
    for name, rows in (("bom.csv", pr), ("bom_fasteners.csv", fr)):
        with open(OUT_DIR / name, "w", newline="", encoding="utf-8") as fh:
            if rows:
                w = csv.DictWriter(fh, fieldnames=list(rows[0].keys()))
                w.writeheader()
                w.writerows(rows)
    return {"parts": len(pr), "fastener_lines": len(fr), "fasteners": sum(r["qty"] for r in fr),
            "mass_kg": round(sum(r["mass_kg_total"] for r in pr), 3)}


def main(argv=None) -> int:
    from ..core.assemble import build_registry
    print(write(build_registry()))
    return 0


if __name__ == "__main__":
    sys.exit(main())
