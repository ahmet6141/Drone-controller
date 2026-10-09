"""Assemble spec.layout + spec.assembly from the builder modules and write spec.yaml (sizing.write_spec, canonical).

usage: python3 build.py [--write]"""
from __future__ import annotations

import sys

from b_common import *  # noqa: F401,F403
import b_assembly
import b_chassis
import b_loads
import b_mass
import b_mech
import b_shell
import b_stations
import b_systems

CONTROL_KEYS = ("gear_up", "turret")


def _stations():
    st = b_stations.stations()
    for s_ in st:
        if s_["type"] == "spar frame":
            s_["type"], s_["subtype"] = "fitting frame", "spar frame"
        elif s_["type"] == "firewall":
            s_["type"], s_["subtype"] = "bulkhead", "firewall"
        t = float(s_.get("t", 0.0068))
        if s_["id"] == "FS3670":
            s_["x_faces"] = r3([X_FW - t, X_FW])
            s_["faces_note"] = "layout.firewall_x is the aft (stainless) face; stack forward of it"
        else:
            s_["x_faces"] = r3([float(s_["x"]) - 0.5 * t, float(s_["x"]) + 0.5 * t])
    return st


def _sequences(me):
    out = {}
    for name, states in me["sequences"].items():
        ctl = [k for k in states[0] if k in CONTROL_KEYS]
        J = {j["name"]: j for j in me["joints"]}
        sts = []
        for s_ in states:
            d = {}
            for k, v in s_.items():
                if k in CONTROL_KEYS:
                    continue
                d[k] = r3(min(max(float(v), float(J[k]["lo"])), float(J[k]["hi"])), 6)
            sts.append(d)
        out[name] = {"control": ctl[0] if ctl else None,
                     "values": [s_[ctl[0]] for s_ in states] if ctl else None, "states": sts}
    me["sequences"] = out
    me["sequences_note"] = ("each sequence: 'states' = sampled joint values (rad / m) for Registry.add_sequence(name, "
                            "states); 'values' = the control property value of each state")
    return me


def build(S_in: dict | None = None) -> tuple[dict, dict]:
    L = copy.deepcopy(S["layout"])
    b_systems.apply_rule_edits(L)
    # ------------------------------------------------------------------ keep sizing-owned keys, replace layout-owned
    keep = {k: L[k] for k in ("datum", "rules", "fuel_cells", "zones_preliminary", "firewall_x", "ground_z",
                              "bay_contents_check") if k in L}
    src = dict(L.get("sources") or {})
    src.update({
        "layout_phase": "layout phase v1 (this section): interface definition for the detail modules; checked by "
                        "ucav250.analysis.layout_check (out/layout.md, docs/03_yerlesim_ve_yapi_konsepti.md)",
        "rules.boxes.z": "layout phase: power-switching / avionics-deck zones raised 3 mm and the side bays lowered "
                         "2 mm so that the 6.8 mm deck sandwich (M-DECK-NOSE, z 0.0140-0.0208) fits between them; "
                         "bay_contents centres follow",
        "mass_placement": "layout phase: positions of the listed mass items = centroids of the placed layout objects "
                          "(sizing.mass_items applies them; layout_check verifies them)"})
    fittings = b_chassis.fittings()
    cg_mtow = [2.62985, 0.0, 0.0375796]
    try:
        import json
        d = json.load(open(str(Z.OUT_DIR / "sizing.json")))
        c0 = d["mass"]["cases"][0]
        cg_mtow = [float(c0["x"]), 0.0, float(c0["z"])]
    except Exception:
        pass
    me = _sequences(b_mech.mechanisms())
    ko = b_mech.keep_outs()
    for k in ko:
        if k["id"] == "KO-ENGINE":
            k["margin"] = b_mech.CLEARANCE_VALUES["engine_keep_out"]
    out = {}
    out["datum"] = keep["datum"]
    out["note"] = ("layout phase v1: interface definition (stations, chassis members and fittings, shell panel "
                   "breakdown, mechanisms, keep-outs, clearances, systems, harness) for the detail modules, who build "
                   "from it without reading each other's geometry; zones_preliminary / fuel_cells / bay_contents_check "
                   "remain sizing outputs")
    out["sources"] = src
    out["rules"] = keep["rules"]
    out["fuel_cells"] = keep["fuel_cells"]
    out["zones_preliminary"] = keep["zones_preliminary"]
    out["firewall_x"] = keep["firewall_x"]
    out["ground_z"] = keep["ground_z"]
    out["bay_contents_check"] = keep.get("bay_contents_check")
    out["root_part"] = "YK250-CH-001"
    out["part_numbers"] = b_stations.PART_NUMBERS
    out["part_numbering"] = b_stations.PART_NUMBERING
    out["stations"] = _stations()
    out["chassis"] = {
        "concept": "chassis + shell: the chassis (frames, chine longerons, keel members, decks, the centre wing box "
                   "and all fittings) carries every primary load; the shell (body skins, hatches, fairings) is "
                   "secondary, fastened to the chassis, and removable panels never carry primary load. The LERX/glove "
                   "skins are the exception: bonded primary sandwich skins of the glove box (closed with the "
                   "chassis, never removed).",
        "members": b_chassis.members(),
        "wing_joint": b_chassis.wing_joint(),
        "fittings": fittings,
        "engine_mount": b_loads.engine_mount(fittings),
        "turret_elevator": b_loads.turret_elevator(),
        "parachute": b_loads.parachute(fittings, cg_mtow),
        "fuel_supports": b_loads.fuel_supports(),
        "trays": b_loads.trays(),
        "design_loads": b_loads.design_loads()}
    out["shell"] = {"rules": b_shell.RULES, "panels": b_shell.panels()}
    out["mechanisms"] = me
    out["keep_outs"] = ko
    out["clearance_values"] = b_mech.CLEARANCE_VALUES
    out["clearances"] = b_mech.clearances()
    out["systems"] = {"equipment": b_systems.equipment(out), "antennas": b_systems.antennas(),
                      "air_data_lights": b_systems.air_data_lights(), "actuators": b_systems.wing_actuators(),
                      "harness": b_systems.harness()}
    from ucav250.analysis import layout_check as LC
    S2 = copy.deepcopy(S)
    S2["layout"] = out
    out["mass_placement"] = LC.mass_placements(LC.Ctx(S2), out)
    import yaml as _y
    head = {i["name"]: i for i in _y.safe_load(open("/tmp/claude-0/layout/spec_head.yaml"))["mass"]["items"]}
    for k, v in out["mass_placement"].items():
        if k in head:
            v["position_sizing_phase"] = r3([head[k]["x"], head[k]["y"], head[k]["z"]])
    A = {"general": b_assembly.GENERAL, "steps": b_assembly.steps(), "transport": b_assembly.TRANSPORT,
         "field_assembly": b_assembly.FIELD, "maintenance_access": b_assembly.MAINTENANCE}
    return out, A


if __name__ == "__main__":
    L, A = build()
    if "--write" in sys.argv:
        S2 = copy.deepcopy(S)
        S2["layout"] = L
        S2["assembly"] = A
        # minimal consistent edits elsewhere (layout phase)
        S2["payload"]["turret"]["bay"]["doors"] = (
            "two sliding doors on rails outboard of the bay walls, moving sideways along the V belly; driven from the "
            "elevator carriage by cam-slot levers (open within the first 20 mm of stroke, close in the last 20 mm of "
            "the retraction); layout phase: inward-folding doors would sweep through the retracted ball")
        Z.write_spec(S2, SPEC.SPEC_PATH)
        print("spec written")
    else:
        import yaml
        print(yaml.safe_dump({"n_stations": len(L["stations"]), "n_members": len(L["chassis"]["members"]),
                              "n_panels": len(L["shell"]["panels"]), "n_joints": len(L["mechanisms"]["joints"]),
                              "n_clear": len(L["clearances"]), "n_steps": len(A["steps"])}))
        for k, v in L["mass_placement"].items():
            print(f"{k:45s} {v['position']}")
