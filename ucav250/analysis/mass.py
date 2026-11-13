"""Mass properties of the registered aircraft (ARCHITECTURE.md §6.8).

* Manufactured parts: mesh volume x effective density (layup areal mass / thickness, or material density).
* Purchased parts: datasheet ``mass_kg``; CG and inertia from their envelope mesh scaled to that mass.
* Consumables (``process == "consumable"``, e.g. fuel contents): scaled per loading case.
* Loading cases from ``spec.mass.cases``: ``[{name, fuel_fraction, payload: [payload item names] | bool, turret}]``.
* Budget cross-check against ``spec.mass.budget``: ``{group: {target_kg, tol_kg}}``.
* Budget group of a part = the group of the ``spec.mass.items`` entry it is booked to: the part's own ``mass_item``
  (producer), else the ``mass_item`` of the layout object that names it (``part`` + ``mass_item`` anywhere in
  ``spec.layout``, e.g. the EFI pump EQ-FUELPUMP in engine_group_installed), else the producer group. Parts booked to a
  ``spec.mass.payload_items`` entry are payload (left out of the no-payload cases).
* Detail-design mass closure (mass/CG closure round): ``spec.mass.items`` entries flagged ``registry_allowance`` (paint
  and the itemised items the geometry does not model, written by sizing) are point masses of the empty aircraft; a
  case's ``spec.mass.payload_items`` that no registry part represents (mission computer, tray, research payload
  allowance, E180 delta) are point masses at their spec positions, and the payload item ``eo_ir_turret_hd59_mount``
  (``registry_parts: payload_group``) adds the remainder of its mass over the registry payload parts. The loading cases
  then carry the whole loading of sizing.mass_cases (the earlier report left 16.25 kg of payload out).
* ``--update-spec`` writes ``spec.mass.registry`` (registry group masses / centroids, empty mass, fuel contents, payload
  parts) which sizing.mass_items closes on; the plain run fails when that block is stale.

CLI:  python3 -m ucav250.analysis.mass [--update-spec]  -> out/mass.md, out/mass.json (exit 1 if a budget/CG limit fails)
"""
from __future__ import annotations

import json
import sys
from collections import defaultdict

import numpy as np

from ..core.parts import Part, Registry

CONSUMABLE = "consumable"
PAYLOAD_GROUP_ITEM_KEY = "payload_group"     # payload item whose registry parts are the producer payload group
REGISTRY_TOL_KG = 0.005                      # spec.mass.registry vs the registry (stale-block check)
REGISTRY_TOL_M = 0.0005
CG_RANGE_TOL = 1e-5                          # m: stability.cg_range_x is stored to 6 significant digits (0.01 mm)


def _base_id(pid: str) -> str:
    return pid[:-2] if pid.endswith(("-R", "-L")) else pid


def layout_booking(spec: dict) -> dict:
    """{part number: mass item} of every layout object that carries both ``part`` and ``mass_item``."""
    out = {}

    def walk(d):
        if isinstance(d, dict):
            if isinstance(d.get("part"), str) and isinstance(d.get("mass_item"), str):
                out.setdefault(d["part"], d["mass_item"])
            for v in d.values():
                walk(v)
        elif isinstance(d, list):
            for v in d:
                walk(v)
    walk(spec.get("layout", {}) or {})
    return out


def budget_groups(reg: Registry) -> dict:
    """{part id: (budget group, mass item or '')} (module docstring)."""
    sm = reg.spec.get("mass", {}) or {}
    items = {i["name"]: i["group"] for i in sm.get("items", []) or []}
    items.update({i["name"]: "payload" for i in sm.get("payload_items", []) or []})
    book = layout_booking(reg.spec)
    out = {}
    for pid, p in reg.parts.items():
        mi = p.mass_item or book.get(pid) or book.get(_base_id(pid)) or ""
        if mi and mi not in items:
            raise ValueError(f"{pid}: mass item {mi!r} not in spec.mass.items / payload_items")
        out[pid] = (items[mi] if mi else p.group, mi)
    return out


def part_props(reg: Registry, p: Part, group: str | None = None) -> dict:
    mesh = p.mesh
    vol_props = mesh.mass_props(1.0)                       # unit density: mass == volume
    m = reg.mass(p)
    k = m / max(vol_props["mass"], 1e-15)
    return {"id": p.id, "group": group or p.group, "mass": m, "cg": vol_props["cg"],
            "inertia": vol_props["inertia"] * k}


def point_mass(pid: str, group: str, m: float, x: float, y: float, z: float) -> dict:
    return {"id": pid, "group": group, "mass": float(m), "cg": np.array([x, y, z], float), "inertia": np.zeros((3, 3))}


def combine(items: list[dict]) -> dict:
    M = sum(i["mass"] for i in items)
    if abs(M) <= 0:
        return {"mass": 0.0, "cg": np.zeros(3), "inertia": np.zeros((3, 3))}
    cg = sum(i["mass"] * i["cg"] for i in items) / M
    I = np.zeros((3, 3))
    for i in items:
        d = i["cg"] - cg
        I += i["inertia"] + i["mass"] * (np.dot(d, d) * np.eye(3) - np.outer(d, d))
    return {"mass": M, "cg": cg, "inertia": I}


def allowance_items(spec: dict) -> list[dict]:
    """Empty-mass allowance items (``spec.mass.items`` with ``registry_allowance``) as point masses."""
    return [point_mass(f"[{i['name']}]", i["group"], i["mass_kg"], i["x"], i["y"], i["z"])
            for i in (spec.get("mass", {}) or {}).get("items", []) or [] if i.get("registry_allowance")]


def payload_booking(spec: dict, bg: dict, pid: str) -> str:
    """Payload item a registry payload part represents: its own mass item, else the item flagged
    ``registry_parts: payload_group`` (producer payload group without a mass item)."""
    mi = bg[pid][1]
    if mi:
        return mi
    for p in (spec.get("mass", {}) or {}).get("payload_items", []) or []:
        if p.get("registry_parts") == PAYLOAD_GROUP_ITEM_KEY:
            return p["name"]
    return ""


def case_items(reg: Registry, props: dict, case: dict, bg: dict | None = None) -> list[dict]:
    """Registry parts (fuel scaled), allowance point masses and the case's payload: registry payload parts of the
    listed payload items + point masses for what the registry does not model (module docstring)."""
    spec = reg.spec
    bg = bg if bg is not None else budget_groups(reg)
    out = []
    ff = float(case.get("fuel_fraction", 1.0))
    pl = case.get("payload", True)
    names = list(pl) if isinstance(pl, (list, tuple)) else None
    reg_pay = defaultdict(float)
    for pid, pr in props.items():
        p = reg.parts[pid]
        if p.process == CONSUMABLE:
            pr = dict(pr, mass=pr["mass"] * ff, inertia=pr["inertia"] * ff)
        if pr["group"] == "payload":
            if names is None:
                if not pl:
                    continue
            else:
                b = payload_booking(spec, bg, pid)
                if b not in names:
                    continue
                reg_pay[b] += pr["mass"]
        out.append(pr)
    out += allowance_items(spec)
    if names:
        pay = {p["name"]: p for p in (spec.get("mass", {}) or {}).get("payload_items", []) or []}
        for n in names:
            p = pay[n]
            m = float(p["mass_kg"]) - reg_pay.get(n, 0.0)
            if abs(m) < 1e-9:
                continue
            z = p["z_extended"] if (case.get("turret") == "extended" and "z_extended" in p) else p["z"]
            out.append(point_mass(f"[{n}]", "payload", m, p["x"], p.get("y", 0.0), z))
    return out


def mac_fraction(reg: Registry, x_cg: float) -> float | None:
    st = reg.spec.get("stability", {}) or {}
    x_le, mac = st.get("mac_le_x"), st.get("mac")
    if x_le is None or not mac:
        return None
    return (x_cg - float(x_le)) / float(mac)


def _r(v, n=6):
    return [round(float(x), n) for x in v] if hasattr(v, "__len__") else round(float(v), n)


def registry_block(reg: Registry, props: dict, bg: dict) -> dict:
    """``spec.mass.registry``: what sizing.mass_items closes on (registry parts only; allowance items are spec items)."""
    groups = defaultdict(list)
    fuel, pay = [], defaultdict(list)
    for pid, pr in props.items():
        if reg.parts[pid].process == CONSUMABLE:
            fuel.append(pr)
        elif pr["group"] == "payload":
            pay[payload_booking(reg.spec, bg, pid)].append(pr)
        else:
            groups[pr["group"]].append(pr)
    empty = combine([pr for v in groups.values() for pr in v])
    f = combine(fuel)
    return {"method": "analysis/mass.py --update-spec: registry of the detailed design (core.assemble.build_registry, "
                      "all producers): geometry x density, datasheet masses of purchased parts; group = budget group "
                      "(mass_item booking); the allowance items (spec.mass.items registry_allowance) are not in it",
            "parts": len(props), "empty_kg": _r(empty["mass"]), "cg": _r(empty["cg"]),
            "groups": {g: {"mass_kg": _r(c["mass"]), "cg": _r(c["cg"])}
                       for g, c in sorted(((g, combine(v)) for g, v in groups.items()), key=lambda t: t[0])},
            "fuel": {"mass_kg": _r(f["mass"]), "cg": _r(f["cg"])},
            "payload_parts": {k: {"mass_kg": _r(c["mass"]), "cg": _r(c["cg"])}
                              for k, c in sorted((k, combine(v)) for k, v in pay.items())}}


def registry_stale(spec_block: dict | None, new: dict) -> list[str]:
    """Differences of spec.mass.registry from the registry beyond REGISTRY_TOL_KG / REGISTRY_TOL_M."""
    if not spec_block:
        return ["spec.mass.registry missing (run python3 -m ucav250.analysis.mass --update-spec)"]
    out = []

    def cmp(tag, a, b):
        if a is None:
            out.append(f"spec.mass.registry {tag} missing")
            return
        if abs(float(a["mass_kg"]) - float(b["mass_kg"])) > REGISTRY_TOL_KG or \
                float(np.abs(np.asarray(a["cg"], float) - np.asarray(b["cg"], float)).max()) > REGISTRY_TOL_M:
            out.append(f"spec.mass.registry {tag} stale: {float(a['mass_kg']):.3f} kg {_r(a['cg'], 4)} vs registry "
                       f"{float(b['mass_kg']):.3f} kg {_r(b['cg'], 4)}")
    for g, b in new["groups"].items():
        cmp(f"groups.{g}", (spec_block.get("groups") or {}).get(g), b)
    for g in (spec_block.get("groups") or {}):
        if g not in new["groups"]:
            out.append(f"spec.mass.registry groups.{g} has no registry parts")
    cmp("fuel", spec_block.get("fuel"), new["fuel"])
    for k, b in new["payload_parts"].items():
        cmp(f"payload_parts.{k}", (spec_block.get("payload_parts") or {}).get(k), b)
    return out


def run(reg: Registry, write: bool = True) -> dict:
    bg = budget_groups(reg)
    props = {pid: part_props(reg, p, bg[pid][0]) for pid, p in reg.parts.items()}
    rebooked = sorted(({"id": pid, "producer_group": reg.parts[pid].group, "group": g, "mass_item": mi,
                        "mass": props[pid]["mass"]} for pid, (g, mi) in bg.items() if g != reg.parts[pid].group),
                      key=lambda r: r["id"])
    allow = allowance_items(reg.spec)
    groups = defaultdict(list)
    for pid, pr in props.items():
        if reg.parts[pid].process != CONSUMABLE:
            groups[pr["group"]].append(pr)
    for a in allow:
        groups[a["group"]].append(a)
    by_group = {g: combine(v) for g, v in groups.items()}
    empty = combine([pr for pid, pr in props.items() if reg.parts[pid].process != CONSUMABLE
                     and pr["group"] != "payload"] + allow)
    spec_mass = reg.spec.get("mass", {}) or {}
    cases = spec_mass.get("cases") or [{"name": "mtow", "fuel_fraction": 1.0, "payload": True}]
    res_cases = {}
    st = reg.spec.get("stability", {}) or {}
    cg_lo, cg_hi = (st.get("cg_range_x") or [None, None])
    failures = []
    for c in cases:
        its = case_items(reg, props, c, bg)
        tot = combine(its)
        frac = mac_fraction(reg, float(tot["cg"][0]))
        res_cases[c["name"]] = {"mass": tot["mass"], "cg": tot["cg"].tolist(), "inertia": tot["inertia"].tolist(),
                                "cg_mac": frac,
                                "payload_point_masses_kg": sum(i["mass"] for i in its if i["id"].startswith("[")
                                                               and i["group"] == "payload")}
        if cg_lo is not None and not (cg_lo - CG_RANGE_TOL <= tot["cg"][0] <= cg_hi + CG_RANGE_TOL):
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
    for n, c in res_cases.items():
        if mtow and c["mass"] > float(mtow) + 1e-6:
            failures.append(f"{n}: {c['mass']:.2f} kg exceeds spec mtow {mtow}")
    blk = registry_block(reg, props, bg)
    stale = registry_stale(spec_mass.get("registry"), blk)
    failures += stale
    S = {"parts": len(props), "empty": {"mass": empty["mass"], "cg": empty["cg"].tolist()},
         "by_group": {g: {"mass": v["mass"], "cg": v["cg"].tolist()} for g, v in by_group.items()},
         "allowance_items": [{"name": a["id"].strip("[]"), "group": a["group"], "mass": a["mass"],
                              "cg": a["cg"].tolist()} for a in allow],
         "cases": res_cases, "budget": rows, "rebooked": rebooked, "registry": blk, "registry_stale": stale,
         "failures": failures, "ok": not failures,
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
         f"Boş kütle (faydalı yük ve yakıt hariç; kayıt defteri parçaları + kalemlendirilmiş paylar): "
         f"**{_f(S['empty']['mass'])} kg**, AG x = {_f(S['empty']['cg'][0], 3)} m "
         f"(kayıt defteri parçaları yalnız: {_f(S['registry']['empty_kg'])} kg, {S['parts']} parça).", "",
         "## Yükleme durumları", "",
         "Her durum sizing.mass_cases ile aynı yüklemeyi taşır: kayıt defteri parçaları (yakıt içeriği durumun yakıt "
         "oranıyla), kalemlendirilmiş boş kütle payları ve durumun faydalı yük kalemleri (kayıt defterinde parçası "
         "olmayanlar — görev bilgisayarı, tepsi, araştırma yükü payı, E180 farkı — kendi spec konumlarında noktasal "
         "kütle).", "",
         "| Durum | Kütle (kg) | Noktasal faydalı yük (kg) | AG x (m) | AG y (mm) | AG z (m) | %OAK (MAC) | Ixx | Iyy | "
         "Izz (kg·m²) |",
         "|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|"]
    for n, c in S["cases"].items():
        I = np.asarray(c["inertia"])
        mac = "—" if c["cg_mac"] is None else _f(100 * c["cg_mac"], 1)
        L.append(f"| {n} | {_f(c['mass'])} | {_f(c['payload_point_masses_kg'])} | {_f(c['cg'][0], 4)} | "
                 f"{_f(c['cg'][1]*1000, 1)} | {_f(c['cg'][2], 4)} | {mac} | {_f(I[0,0], 1)} | {_f(I[1,1], 1)} | "
                 f"{_f(I[2,2], 1)} |")
    L += ["", "## Gruplar", "", "| Grup | Kütle (kg) | AG x (m) |", "|---|---:|---:|"]
    for g, v in sorted(S["by_group"].items(), key=lambda kv: -kv[1]["mass"]):
        L.append(f"| {g} | {_f(v['mass'])} | {_f(v['cg'][0], 3)} |")
    if S.get("allowance_items"):
        L += ["", "## Kalemlendirilmiş boş kütle payları (geometride modellenmeyen; spec.mass.items registry_allowance)",
              "", "| Kalem | Grup | Kütle (kg) | x (m) |", "|---|---|---:|---:|"]
        L += [f"| {a['name']} | {a['group']} | {_f(a['mass'], 3)} | {_f(a['cg'][0], 3)} |" for a in S["allowance_items"]]
    if S["budget"]:
        L += ["", "## Kütle bütçesi karşılaştırması", "", "| Grup | Hesap (kg) | Bütçe (kg) | Tolerans | Durum |",
              "|---|---:|---:|---:|---|"]
        for r in S["budget"]:
            L.append(f"| {r['group']} | {_f(r['computed_kg'])} | {_f(r['target_kg'])} | ±{_f(r['tol_kg'])} | "
                     f"{'✓' if r['ok'] else '✗'} |")
    if S.get("rebooked"):
        L += ["", "## Başka bütçe grubuna yazılan parçalar (layout / üretici mass_item)", "",
              "| Parça | Üretici grubu | Bütçe grubu | Kütle kalemi | Kütle (kg) |", "|---|---|---|---|---:|"]
        L += [f"| {r['id']} | {r['producer_group']} | {r['group']} | {r['mass_item']} | {_f(r['mass'], 3)} |"
              for r in S["rebooked"]]
    L += ["", "## En ağır 25 parça", "", "| Parça | Kütle (kg) |", "|---|---:|"]
    L += [f"| {r['id']} | {_f(r['mass'], 3)} |" for r in S["top_parts"]]
    if S["failures"]:
        L += ["", "## Sorunlar", ""] + [f"- {x}" for x in S["failures"]]
    return "\n".join(L) + "\n"


def update_spec(S_run: dict, path=None) -> int:
    """Write ``spec.mass.registry`` (sizing.write_spec layout of the file); returns the number of changed values."""
    from ..core import spec as SPEC
    from .sizing import write_spec
    p = path or SPEC.SPEC_PATH
    with open(p, encoding="utf-8") as fh:
        import yaml
        S = yaml.safe_load(fh)
    old = (S.get("mass") or {}).get("registry")
    S["mass"]["registry"] = S_run["registry"]
    write_spec(S, p)
    SPEC.load.cache_clear()
    return 0 if old == S_run["registry"] else 1


def main(argv=None) -> int:
    argv = list(sys.argv[1:] if argv is None else argv)
    from ..core.assemble import build_registry
    S = run(build_registry())
    if "--update-spec" in argv:
        changed = update_spec(S)
        print(f"[mass] spec.mass.registry {'updated' if changed else 'unchanged'}: empty (registry parts) "
              f"{S['registry']['empty_kg']:.3f} kg, CG x {S['registry']['cg'][0]:.4f} m; run sizing --update-spec")
        S["failures"] = [f for f in S["failures"] if not f.startswith("spec.mass.registry")]
        S["ok"] = not S["failures"]
    print(json.dumps({"ok": S["ok"], "failures": S["failures"],
                      "cases": {k: round(v["mass"], 2) for k, v in S["cases"].items()}}))
    return 0 if S["ok"] else 1


if __name__ == "__main__":
    sys.exit(main())
