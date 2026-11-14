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
* Poses (verification MC-01 / MC-03): the producers model every moving part at its joint rest pose = the GROUND pose
  (gear down, gear doors open, turret retracted). The flight loading cases pose the registry at the last state of the
  ``gear_retraction`` sequence (gear up, doors closed) and, for ``turret: extended``, also at the last state of
  ``turret_extension`` (Registry.posed_mesh: the part's own mass at the posed centroid / inertia). spec.mass.registry
  carries both: ``cg`` = flight (gear up, turret retracted), ``cg_turret_extended`` and ``cg_ground`` (rest pose; read
  by sizing.ground_cases).
* Fuel level (verification REG-01): the fuel module's level table (Registry.tables["fuel_levels"], common free surface
  of the three communicating bladders, fed from the aft cell) gives the fuel CG as a function of the fuel on board;
  every case puts its fuel (contents mass x fuel_fraction) at that CG (the contents' own inertia scaled); the table is
  written to spec.mass.registry.fuel.levels for sizing (case CGs and the fuel-burn static margin of R-09).

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
MTOW_TOL_KG = 5e-4                           # kg: the case loads are spec values stored to 0.1 g (fuel, items, payload)
FLIGHT_SEQ = "gear_retraction"               # flight pose: last state (gear up, gear doors closed)
TURRET_SEQ = "turret_extension"              # turret extended: last state
SIZING_TOL_X = 2e-4                          # m: case CG x of analysis/mass.py vs sizing.mass_cases (self-check)
SIZING_TOL_Z = 2e-3                          # m: case CG z (the sizing turret item moves as one point mass)


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


def poses(reg: Registry) -> dict:
    """{pose name: joint state}: ground = rest pose of every joint; flight = gear_retraction last state; flight with
    the turret extended = gear_retraction + turret_extension last states (module docstring)."""
    g = dict((reg.sequences.get(FLIGHT_SEQ) or [{}])[-1])
    t = dict((reg.sequences.get(TURRET_SEQ) or [{}])[-1])
    return {"ground": {}, "flight": g, "flight_turret_extended": {**g, **t}}


def posed_props(reg: Registry, props: dict, state: dict) -> dict:
    """Part properties with the joints at ``state``: every part whose joint chain holds a joint of the state is
    re-evaluated on its posed mesh (same mass, posed centroid and inertia); the others are shared with ``props``."""
    if not state:
        return props
    out = dict(props)
    for pid, p in reg.parts.items():
        if p.joint is None or not any(j.name in state for j in reg.joint_chain(p.joint)):
            continue
        vp = reg.posed_mesh(p, state).mass_props(1.0)
        pr = props[pid]
        k = pr["mass"] / max(vp["mass"], 1e-15)
        out[pid] = dict(pr, cg=vp["cg"], inertia=vp["inertia"] * k)
    return out


def fuel_level_table(reg: Registry) -> list | None:
    """[[m_fuel, x, y, z], ...] of the fuel module (Registry.tables["fuel_levels"]), ascending in mass."""
    t = getattr(reg, "tables", {}).get("fuel_levels")
    return [list(map(float, r)) for r in t] if t else None


def fuel_cg_at(levels, m: float):
    """CG of ``m`` kg of fuel from the level table (linear in mass; below the first row: the first row)."""
    A = np.asarray(levels, float)
    m = min(max(float(m), A[0, 0]), A[-1, 0])
    return np.array([np.interp(m, A[:, 0], A[:, k]) for k in (1, 2, 3)])


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


def case_pose(case: dict) -> str:
    return "flight_turret_extended" if case.get("turret") == "extended" else "flight"


def case_items(reg: Registry, props: dict, case: dict, bg: dict | None = None, levels=None) -> list[dict]:
    """Registry parts (``props`` of the case's pose), the fuel (contents x fuel fraction at the level-table CG),
    allowance point masses and the case's payload: registry payload parts of the listed payload items + point masses
    for what the registry does not model (module docstring)."""
    spec = reg.spec
    bg = bg if bg is not None else budget_groups(reg)
    out = []
    ff = float(case.get("fuel_fraction", 1.0))
    pl = case.get("payload", True)
    names = list(pl) if isinstance(pl, (list, tuple)) else None
    reg_pay = defaultdict(float)
    fuel = []
    for pid, pr in props.items():
        p = reg.parts[pid]
        if p.process == CONSUMABLE:
            fuel.append(dict(pr, mass=pr["mass"] * ff, inertia=pr["inertia"] * ff))
            continue
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
    if fuel and ff > 0.0:
        f = combine(fuel)
        cg = fuel_cg_at(levels, f["mass"]) if levels else f["cg"]
        out.append({"id": "[fuel]", "group": "fuel_contents", "mass": f["mass"], "cg": cg, "inertia": f["inertia"]})
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


def registry_block(reg: Registry, pp: dict, bg: dict, levels=None, refs: dict | None = None) -> dict:
    """``spec.mass.registry``: what sizing.mass_items closes on (registry parts only; allowance items are spec items).
    ``pp`` = {pose: props} (poses()); group / payload ``cg`` = flight pose, ``cg_turret_extended`` and ``cg_ground``
    the other poses; fuel = design contents + the level table; ``refs`` = named part sets the allowance items of
    sizing.registry_closure_items are measured against (ALLOWANCE_REFS)."""
    def split(props):
        groups, fuel, pay = defaultdict(list), [], defaultdict(list)
        for pid, pr in props.items():
            if reg.parts[pid].process == CONSUMABLE:
                fuel.append(pr)
            elif pr["group"] == "payload":
                pay[payload_booking(reg.spec, bg, pid)].append(pr)
            else:
                groups[pr["group"]].append(pr)
        return groups, fuel, pay
    gf, fuel, pf = split(pp["flight"])
    gt, _f, pt = split(pp["flight_turret_extended"])
    gg, _f, pg = split(pp["ground"])
    empty = combine([pr for v in gf.values() for pr in v])
    empty_g = combine([pr for v in gg.values() for pr in v])
    f = combine(fuel)

    def blk(a, t, g):
        ca, ct, cg_ = combine(a), combine(t), combine(g)
        return {"mass_kg": _r(ca["mass"]), "cg": _r(ca["cg"]), "cg_turret_extended": _r(ct["cg"]),
                "cg_ground": _r(cg_["cg"])}
    out = {"method": "analysis/mass.py --update-spec: registry of the detailed design (core.assemble.build_registry, "
                     "all producers): geometry x density, datasheet masses of purchased parts; group = budget group "
                     "(mass_item booking); the allowance items (spec.mass.items registry_allowance) are not in it; cg = "
                     "flight pose (gear_retraction last state: gear up, doors closed; turret retracted), "
                     "cg_turret_extended (+ turret_extension last state), cg_ground (rest pose: gear down, doors open)",
           "parts": len(pp["flight"]), "empty_kg": _r(empty["mass"]), "cg": _r(empty["cg"]),
           "cg_ground": _r(empty_g["cg"]),
           "groups": {g: blk(gf[g], gt[g], gg[g]) for g in sorted(gf)},
           "fuel": {"mass_kg": _r(f["mass"]), "cg": _r(f["cg"])},
           "payload_parts": {k: blk(pf[k], pt[k], pg[k]) for k in sorted(pf)}}
    if levels:
        out["fuel"]["levels"] = [[round(float(r[0]), 4)] + [round(float(v), 5) for v in r[1:]] for r in levels]
        out["fuel"]["levels_note"] = ("fuel module level table: [fuel kg, x, y, z] of the fuel on board with a common "
                                      "free surface in the three communicating bladders (bottom interconnects / saddle "
                                      "drain into the aft feed cell, fed from the aft cell), body axes level")
    if refs:
        out["refs"] = refs
    return out


# named part sets the itemised allowances of sizing.registry_closure_items are measured against (spec.mass.registry
# refs: mass and flight-pose centroid): part-id prefixes
ALLOWANCE_REFS = {"exhaust_pipes_silencers": ("YK250-PR-504",),
                  "main_gear_retraction_emas": ("YK250-LG-630",),
                  "nose_gear_retraction_ema": ("YK250-LG-632",)}


def allowance_refs(reg: Registry, props: dict) -> dict:
    out = {}
    for k, pre in ALLOWANCE_REFS.items():
        its = [pr for pid, pr in props.items() if pid.startswith(pre) and not pid.startswith("YK250-HW-")]
        if its:
            c = combine(its)
            out[k] = {"parts": sorted(pid for pid in props if pid.startswith(pre)), "mass_kg": _r(c["mass"]),
                      "cg": _r(c["cg"])}
    return out


def registry_stale(spec_block: dict | None, new: dict) -> list[str]:
    """Differences of spec.mass.registry from the registry beyond REGISTRY_TOL_KG / REGISTRY_TOL_M."""
    if not spec_block:
        return ["spec.mass.registry missing (run python3 -m ucav250.analysis.mass --update-spec)"]
    out = []

    def cmp(tag, a, b):
        if a is None:
            out.append(f"spec.mass.registry {tag} missing")
            return
        bad = abs(float(a["mass_kg"]) - float(b["mass_kg"])) > REGISTRY_TOL_KG
        for k in ("cg", "cg_turret_extended", "cg_ground"):
            if k in b:
                if k not in a:
                    bad = True
                elif float(np.abs(np.asarray(a[k], float) - np.asarray(b[k], float)).max()) > REGISTRY_TOL_M:
                    bad = True
        if bad:
            out.append(f"spec.mass.registry {tag} stale: {float(a['mass_kg']):.3f} kg {_r(a['cg'], 4)} vs registry "
                       f"{float(b['mass_kg']):.3f} kg {_r(b['cg'], 4)}")
    for g, b in new["groups"].items():
        cmp(f"groups.{g}", (spec_block.get("groups") or {}).get(g), b)
    for g in (spec_block.get("groups") or {}):
        if g not in new["groups"]:
            out.append(f"spec.mass.registry groups.{g} has no registry parts")
    cmp("fuel", spec_block.get("fuel"), new["fuel"])
    lv_a, lv_b = (spec_block.get("fuel") or {}).get("levels"), new["fuel"].get("levels")
    if lv_b and (not lv_a or len(lv_a) != len(lv_b) or
                 float(np.abs(np.asarray(lv_a, float) - np.asarray(lv_b, float)).max()) > REGISTRY_TOL_M):
        out.append("spec.mass.registry fuel.levels stale")
    for k, b in new["payload_parts"].items():
        cmp(f"payload_parts.{k}", (spec_block.get("payload_parts") or {}).get(k), b)
    for k, b in (new.get("refs") or {}).items():
        cmp(f"refs.{k}", (spec_block.get("refs") or {}).get(k), b)
    return out


def sizing_compare(reg: Registry, res_cases: dict) -> tuple[list, list]:
    """Self-check (MC-03): case CG x / z of this registry evaluation vs sizing.mass_cases on the spec (the loading
    the stability rules use); rows + failures beyond SIZING_TOL_X / SIZING_TOL_Z. I_yy is reported (sizing's own
    pitch inertia is a distributed estimate, not a check)."""
    from .sizing import mass_cases
    try:
        sz = {c["name"]: c for c in mass_cases(reg.spec)}
    except (KeyError, TypeError, ValueError) as exc:
        return [], [f"sizing.mass_cases not evaluable: {exc}"]
    rows, bad = [], []
    for n, c in res_cases.items():
        s = sz.get(n)
        if s is None:
            continue
        dx, dz = c["cg"][0] - s["x"], c["cg"][2] - s["z"]
        rows.append({"case": n, "dx_m": dx, "dz_m": dz, "Iyy_registry": c["inertia"][1][1], "m_sizing": s["m"]})
        if abs(dx) > SIZING_TOL_X or abs(dz) > SIZING_TOL_Z:
            bad.append(f"sizing compare: {n}: case CG vs sizing.mass_cases dx {dx * 1000:+.2f} mm, dz {dz * 1000:+.2f} mm "
                       f"(tolerance {SIZING_TOL_X * 1000:.1f} / {SIZING_TOL_Z * 1000:.1f} mm)")
    return rows, bad


def run(reg: Registry, write: bool = True) -> dict:
    bg = budget_groups(reg)
    props = {pid: part_props(reg, p, bg[pid][0]) for pid, p in reg.parts.items()}      # rest pose = ground
    pp = {k: posed_props(reg, props, st) for k, st in poses(reg).items()}
    levels = fuel_level_table(reg)
    rebooked = sorted(({"id": pid, "producer_group": reg.parts[pid].group, "group": g, "mass_item": mi,
                        "mass": props[pid]["mass"]} for pid, (g, mi) in bg.items() if g != reg.parts[pid].group),
                      key=lambda r: r["id"])
    allow = allowance_items(reg.spec)
    groups = defaultdict(list)
    for pid, pr in pp["flight"].items():
        if reg.parts[pid].process != CONSUMABLE:
            groups[pr["group"]].append(pr)
    for a in allow:
        groups[a["group"]].append(a)
    by_group = {g: combine(v) for g, v in groups.items()}
    empty = combine([pr for pid, pr in pp["flight"].items() if reg.parts[pid].process != CONSUMABLE
                     and pr["group"] != "payload"] + allow)
    empty_g = combine([pr for pid, pr in pp["ground"].items() if reg.parts[pid].process != CONSUMABLE
                       and pr["group"] != "payload"] + allow)
    spec_mass = reg.spec.get("mass", {}) or {}
    cases = spec_mass.get("cases") or [{"name": "mtow", "fuel_fraction": 1.0, "payload": True}]
    res_cases = {}
    st = reg.spec.get("stability", {}) or {}
    cg_lo, cg_hi = (st.get("cg_range_x") or [None, None])
    failures = []
    for c in cases:
        its = case_items(reg, pp[case_pose(c)], c, bg, levels)
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
        if mtow and c["mass"] > float(mtow) + MTOW_TOL_KG:
            failures.append(f"{n}: {c['mass']:.2f} kg exceeds spec mtow {mtow}")
    blk = registry_block(reg, pp, bg, levels, allowance_refs(reg, pp["flight"]))
    stale = registry_stale(spec_mass.get("registry"), blk)
    failures += stale
    cmp_rows, cmp_bad = sizing_compare(reg, res_cases) if not stale else ([], [])
    failures += cmp_bad
    S = {"parts": len(props), "empty": {"mass": empty["mass"], "cg": empty["cg"].tolist(),
                                        "cg_ground": empty_g["cg"].tolist()},
         "sizing_compare": cmp_rows,
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
         f"**{_f(S['empty']['mass'])} kg**, AG x = {_f(S['empty']['cg'][0], 3)} m uçuş duruşunda (takım yukarıda, "
         f"kapaklar kapalı); yerde (takım aşağıda, kapaklar açık) x = {_f(S['empty']['cg_ground'][0], 3)} m "
         f"(kayıt defteri parçaları yalnız: {_f(S['registry']['empty_kg'])} kg, {S['parts']} parça).", "",
         "Uçuş durumları kayıt defterini `gear_retraction` dizisinin son durumunda (takım yukarıda) ve taret açık "
         "durumlarda ayrıca `turret_extension` son durumunda konumlar; yakıt, yakıt modülünün seviye tablosundaki "
         "(üç hücrede ortak serbest yüzey) ağırlık merkezinde durur.", "",
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
    if S.get("sizing_compare"):
        L += ["", "## sizing.mass_cases ile karşılaştırma (öz denetim)", "",
              "| Durum | Δx (mm) | Δz (mm) | Iyy kayıt defteri (kg·m²) |", "|---|---:|---:|---:|"]
        L += [f"| {r['case']} | {_f(r['dx_m'] * 1000, 2)} | {_f(r['dz_m'] * 1000, 2)} | {_f(r['Iyy_registry'], 1)} |"
              for r in S["sizing_compare"]]
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
        S["failures"] = [f for f in S["failures"] if not f.startswith(("spec.mass.registry", "sizing compare:"))]
        S["ok"] = not S["failures"]
    print(json.dumps({"ok": S["ok"], "failures": S["failures"],
                      "cases": {k: round(v["mass"], 2) for k, v in S["cases"].items()}}))
    return 0 if S["ok"] else 1


if __name__ == "__main__":
    sys.exit(main())
