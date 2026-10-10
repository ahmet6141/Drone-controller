"""Assemble ``spec.layout`` + ``spec.assembly`` from the builder modules and write spec.yaml (sizing.write_spec,
canonical format).

    python3 -m ucav250.layout_build.build            summary of the generated layout (nothing written)
    python3 -m ucav250.layout_build.build --check    exit 1 if spec.yaml differs from the regenerated text
    python3 -m ucav250.layout_build.build --write    rewrite spec.yaml (layout, assembly, turret door text)

Ownership: the layout phase owns ``layout`` (except the sizing-owned keys ``datum``, ``rules`` (edited only through
``b_systems.RULE_EDITS``), ``fuel_cells``, ``zones_preliminary``, ``firewall_x``, ``ground_z``,
``bay_contents_check``, which are carried over) and ``assembly``. The workflow after a layout change:
``build --write`` -> ``sizing --update-spec`` (applies ``layout.mass_placement`` to the mass items) -> ``build --check``
(must be unchanged: fixed point) -> ``sizing --check`` and ``layout_check --check``."""
from __future__ import annotations

import copy
import sys
import tempfile
from pathlib import Path

from . import b_assembly, b_chassis, b_loads, b_mech, b_shell, b_stations, b_systems
from .b_common import SPEC, X_FW, Z, S, r3

CONTROL_KEYS = ("gear_up", "turret")

# Positions of the mass items the layout places, as the closed sizing phase had them (spec.yaml of commit b9d0b87,
# "HANCER sizing closed"); kept in layout.mass_placement[*].position_sizing_phase for traceability of the CG shift.
SIZING_PHASE_POSITIONS = {
    "engine_group_installed": [3.83, 0.0, 0.19],
    "cooling_baffles_firewall_cowl_flap": [3.73, 0.0, 0.25],
    "dorsal_cooling_inlet_s_duct": [3.4, 0.0, 0.32],
    "actuators_stabilators_2x_DA30": [3.7375, 0.0, 0.2183],
    "actuators_nose_steering_brake_2x_DA26": [1.8253, 0.0, -0.05],
    "wiring_harness_connectors_coax": [1.9, 0.0, 0.0],
    "flight_termination_lights": [1.6836, 0.0, 0.02],
    "keel_beams_longerons": [2.0, 0.0, -0.05],
    "frames_bulkheads": [1.88, 0.0, 0.0],
    "engine_mount_4130": [3.67, 0.0, 0.22],
    "parachute_attach_fitting": [1.65, 0.0, -0.0345],
    "floors_trays_rails": [1.4, 0.0, -0.02],
    "hatch_frames_quick_access_fasteners": [1.5, 0.0, 0.08],
    "fin_ventral_root_fittings": [3.5722, 0.0, 0.05],
    # fix round 2 (PK2-07): items placed from the layout since this round, same source (commit b9d0b87)
    "actuators_ailerons_2x_DA26": [3.0999, 0.0, 0.0425],
    "actuators_flaps_2x_DA30": [3.0063, 0.0, 0.0],
    "actuators_rudders_2x_DA26": [3.921, 0.0, 0.6312],
    "main_gear_legs_wheels_brakes_emas_pair": [3.0006, 0.0, -0.127],
    "nose_gear_leg_wheel_steering": [0.8443, 0.0, -0.1244],
    "gear_doors_wells_locks_sensors": [2.4987, 0.0, -0.1],
    "turret_lift_mechanism_doors": [1.22, 0.0, 0.007],
    "parachute_uavos_200": [1.65, 0.0, -0.0345],
    "fuel_system_3_cells": [2.6421, 0.0, 0.05]}

SIZING_OWNED = ("datum", "rules", "fuel_cells", "zones_preliminary", "firewall_x", "ground_z", "bay_contents_check")

TURRET_DOORS_TEXT = (
    "two sliding doors on rails outboard of the bay walls, moving sideways along the V belly; each driven by its own "
    "Volz DA 22 through pinion and rack (layout.systems.equipment EQ-TDOORACT), interlocked with the elevator: doors "
    "fully open before the elevator moves, closed only after full retraction (layout.mechanisms.sequences."
    "turret_extension); layout phase: inward-folding doors would sweep through the retracted ball")


def _stations() -> list:
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


def _sequences(me: dict) -> dict:
    out = {}
    J = {j["name"]: j for j in me["joints"]}
    for name, states in me["sequences"].items():
        ctl = [k for k in states[0] if k in CONTROL_KEYS]
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


def parachute_cg() -> list:
    """CG the bridle confluence is placed over: stability.cg_design (design-mission MTOW CG of the closed sizing),
    rounded to 1 mm so that the layout is a fixed point of the sizing update."""
    c = S["stability"]["cg_design"]
    return [round(float(c[0]), 3), 0.0, round(float(c[2]), 3)]


def build() -> tuple[dict, dict]:
    """Return (layout, assembly) generated from the loaded spec."""
    L = copy.deepcopy(S["layout"])
    b_systems.apply_rule_edits(L)
    keep = {k: L[k] for k in SIZING_OWNED if k in L}
    src = dict(L.get("sources") or {})
    src.update({
        "layout_phase": "layout phase v1 (this section): interface definition for the detail modules; checked by "
                        "ucav250.analysis.layout_check (out/layout.md, docs/03_yerlesim_ve_yapi_konsepti.md); "
                        "generated by ucav250.layout_build.build",
        "rules.boxes.z": "layout phase: power-switching / avionics-deck zones raised 3 mm and the side bays lowered "
                         "2 mm so that the 6.8 mm deck sandwich (M-DECK-NOSE, z 0.0140-0.0208) fits between them; "
                         "bay_contents centres follow",
        "mass_placement": "layout phase: positions of the listed mass items = centroids of the placed layout objects "
                          "(sizing.mass_items applies them; layout_check verifies them)"})
    fittings = b_chassis.fittings()
    me = _sequences(b_mech.mechanisms())
    out = {}
    out["datum"] = keep["datum"]
    out["note"] = ("layout phase v1: interface definition (stations, chassis members and fittings, shell panel "
                   "breakdown, mechanisms, keep-outs, clearances, systems, harness) for the detail modules, who build "
                   "from it without reading each other's geometry; zones_preliminary / fuel_cells / bay_contents_check "
                   "remain sizing outputs")
    out["sources"] = src
    for k in ("rules", "fuel_cells", "zones_preliminary", "firewall_x", "ground_z"):
        out[k] = keep[k]
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
        "parachute": b_loads.parachute(fittings, parachute_cg()),
        "fuel_supports": b_loads.fuel_supports(),
        "trays": b_loads.trays(),
        "ground_handling": b_loads.ground_handling(),
        "design_loads": b_loads.design_loads()}
    out["shell"] = {"rules": b_shell.RULES, "panels": b_shell.panels(), "root_cut_lines": b_shell.root_cut_lines(),
                    "wing_root_fairing": b_shell.wing_root_fairing()}
    out["mechanisms"] = me
    out["keep_outs"] = b_mech.keep_outs()
    out["clearance_values"] = b_mech.CLEARANCE_VALUES
    out["clearances"] = b_mech.clearances()
    out["systems"] = {"equipment": b_systems.equipment(out), "antennas": b_systems.antennas(),
                      "air_data_lights": b_systems.air_data_lights(), "actuators": b_systems.wing_actuators(),
                      "harness": b_systems.harness()}
    out["fuel_lines"] = b_systems.fuel_lines()
    out["fuel_system"] = b_systems.fuel_system()
    out["keep_outs"] += b_mech.sweep_keep_outs(out)
    out["heat_protection"] = b_mech.heat_protection(out["keep_outs"])
    out["chassis"]["engine_mount"]["firewall_stackup"]["mass"] = b_loads.firewall_mass(out)
    from ..analysis import layout_check as LC
    S2 = copy.deepcopy(S)
    S2["layout"] = out
    round2_edits(S2)
    # fix round 2 (mass closure): areas and net masses of the heat-protection parts and metal cowl pieces; the cowl
    # skin area (booked in the shell item) for the cooling roll-up split
    hpa = LC.heat_protection_areas(LC.Ctx(S2), out)
    b_mech.heat_protection_mass(out["heat_protection"], hpa, out)
    cooling_split(S2, hpa["cowl_skin_area_m2"])
    out["mass_placement"] = LC.mass_placements(LC.Ctx(S2), out)
    for k, v in out["mass_placement"].items():
        if k in SIZING_PHASE_POSITIONS:
            v["position_sizing_phase"] = list(SIZING_PHASE_POSITIONS[k])
    A = {"general": b_assembly.GENERAL, "steps": b_assembly.steps(), "transport": b_assembly.transport(),
         "field_assembly": b_assembly.FIELD, "maintenance_access": b_assembly.MAINTENANCE}
    return out, A


RUDDER_SPAN = {"eta0": 0.1443, "eta1": 0.8438,
               "span_note": "fix round 1 (VPK-05/VPK-09): rudder ends as fractions of the fin span "
                            "(tail.surfaces.fin.params.span) measured from the root reference section along the canted "
                            "span (0.13-0.76 m from it); the root end stays >= 8 mm above the cowl "
                            "and the fin-root fairing over +-25 deg (layout_check C05); sizing evaluates the rudder hinge "
                            "moments on the full fin span (conservative for the actuator)"}
NOSE_LEG_WIDTH = 0.036          # structures.sizing.gear.nose_leg.od_m (S1-08)


def _unshare(o):
    """New containers everywhere (no shared sub-objects: the YAML dumper would write anchors / aliases that a re-load
    and re-write by sizing.write_spec does not reproduce)."""
    if isinstance(o, dict):
        return {k: _unshare(v) for k, v in o.items()}
    if isinstance(o, (list, tuple)):
        return [_unshare(v) for v in o]
    return o


COWL_SKIN_M2 = [None]          # cowl skin area of the last build (both sides), for the generated spec copy


def cooling_split(S2: dict, cowl_m2: float) -> None:
    """Fix round 2 (mass closure, PK2-09 heat protection booked): the engine cooling / fire-protection item is re-based
    bottom-up on the layout (sizing.cooling_installation_mass). The baseline roll-up mass.rules.cooling_firewall_kg
    (1.5 kg = "about 1 m2 composite cowl/ducts at about 1 kg/m2 + 0.38 mm stainless firewall about 0.45 kg",
    baseline.yaml) is split: the stainless firewall layer and the heat protection come from the layout; the composite
    share keeps the baseline areal rule on the baseline area minus the cowl skin, which the shell item already books
    (P-COWL-* pieces, layout_check.heat_protection_areas)."""
    COWL_SKIN_M2[0] = float(cowl_m2)
    a_base, k_ar = 1.0, 1.0
    S2["mass"]["rules"]["cooling_split"] = {
        "baseline_composite_area_m2": a_base, "baseline_areal_kg_m2": k_ar,
        "cowl_skin_area_m2": round(float(cowl_m2), 4),
        "baffles_plenum_ducts_lip_kg": round(k_ar * (a_base - float(cowl_m2)), 4),
        "note": "fix round 2: composite share of the baseline roll-up (about 1 m2 composite cowl/ducts at about 1 kg/m2, "
                "baseline.yaml#mass_targets.systems_rollup_kg.cowling_cooling_ducts_baffles_firewall) minus the cowl "
                "skin booked in the shell item = cylinder baffles, plenum, internal ducts and the exit lip (estimate); "
                "the firewall layer (layout.chassis.engine_mount.firewall_stackup.mass) and the heat protection "
                "(layout.heat_protection.mass) are bottom-up from the layout"}


def round2_edits(S2: dict) -> None:
    """Fix round 2 consistent edits outside spec.layout (also applied to the spec copy the mass placement is computed
    on, so that build() is a fixed point)."""
    # PK2-06: the aileron DA 26 sits 0.16 m ahead of the hinge (deeper section); the four-bar of the sizing linkage
    # check uses the same base
    S2["wing"]["controls"]["aileron"]["linkage"]["pushrod_base_m"] = b_systems.AILERON_PUSHROD_BASE
    # PK2-13: one Volz DA 22 for both nose clamshell doors (EQ-NDOORACT, centre-line bellcrank + two links, NDOOR-LINKAGE)
    # in addition to the two main inner-door drives; the trunnion doors are slaved to the inner-door drive
    # VS2-10: the inboard spindle bearing sits in the firewall-mounted node fitting F-SPINDLE-NODE (VPK-01)
    src = S2["tail"]["surfaces"]["stabilator"]["controls"]["sources"]
    src["spindle"] = str(src["spindle"]).replace(
        "inboard bearing in an engine-bay ring frame beside the crankcase/SG750 (y >= 0.10 m + engine keep-out)",
        "inboard bearing in the firewall-mounted node fitting (layout F-SPINDLE-NODE, layout phase VPK-01; y >= 0.10 m "
        "+ engine keep-out; 61805-ZZ, fix round 2 PK2-09)")
    if COWL_SKIN_M2[0] is not None:
        cooling_split(S2, COWL_SKIN_M2[0])
    gda = S2["mass"]["rules"]["gear_doors"]["inner_door_actuator"]
    gda["count"] = 3
    gda["note"] = ("fix round 2 (PK2-13): 2 main inner-door drives (EQ-DOORACT, inner door only; fix round 3, PK3-07: "
                   "the trunnion door is leg-driven through a slotted lost-motion link + opening spring, within "
                   "linkage_kg_each) + 1 nose clamshell-door drive (EQ-NDOORACT) for both doors through a centre-line "
                   "bellcrank (mass.rules.gear_doors.nose_door_drive)")
    nde = next(e for e in S2["layout"]["systems"]["equipment"] if e["id"] == "EQ-NDOORACT")["drive_envelope"]
    S2["mass"]["rules"]["gear_doors"]["nose_door_drive"] = {
        "bellcrank_links_kg": nde["mass_kg"], "basis": nde["mass_basis"],
        "note": "fix round 2: one DA 22 (counted in inner_door_actuator) drives both clamshell doors through this "
                "bellcrank, two links and two horns on the hinge-pin extensions (layout NDOOR-LINKAGE)"}


def generated_spec() -> dict:
    """The full spec with the regenerated layout / assembly and the consistent edits the layout / structures phase
    makes elsewhere (turret door text, rudder span, nose-leg envelope width, UD material without a bearing value)."""
    L, A = build()
    S2 = copy.deepcopy(S)
    S2["layout"] = L
    S2["assembly"] = A
    S2["payload"]["turret"]["bay"]["doors"] = TURRET_DOORS_TEXT
    S2["tail"]["surfaces"]["fin"]["controls"]["rudder"].update(copy.deepcopy(RUDDER_SPAN))
    ng = S2["landing_gear"]["nose"]
    ng["leg_frontal_width"] = max(float(ng["leg_frontal_width"]), NOSE_LEG_WIDTH)
    round2_edits(S2)
    ud = S2["materials"]["cfrp_ud_mtm45_as4"]
    if "Fbru" in ud:                  # S1-05: the UD value was a copy of the PW/QI bearing value, not a UD property
        ud.pop("Fbru")
        ud["note_bearing"] = ("no bearing allowable for the UD tape: bolted / pinned zones use a quasi-isotropic "
                              "(>= 40 % +-45) build-up with fasteners.joint_geometry_rules.bearing_design_allowables "
                              "(e/D >= 3) or the open-hole compression value (structures phase, fix round 1)")
    return _unshare(S2)


def render(S2: dict) -> str:
    """spec.yaml text of ``S2`` exactly as sizing.write_spec writes it."""
    with tempfile.TemporaryDirectory() as td:
        p = Path(td) / "spec.yaml"
        Z.write_spec(S2, p)
        return p.read_text(encoding="utf-8")


def main(argv=None) -> int:
    argv = sys.argv[1:] if argv is None else argv
    S2 = generated_spec()
    if "--write" in argv:
        Z.write_spec(S2, SPEC.SPEC_PATH)
        print(f"[layout_build] written {SPEC.SPEC_PATH}")
        return 0
    if "--check" in argv:
        new = render(S2)
        cur = SPEC.SPEC_PATH.read_text(encoding="utf-8")
        if new == cur:
            print("[layout_build] spec.yaml is up to date (regenerated text identical)")
            return 0
        import difflib
        d = list(difflib.unified_diff(cur.splitlines(), new.splitlines(), "spec.yaml", "regenerated", n=1,
                                      lineterm=""))
        print(f"[layout_build] spec.yaml differs from the regenerated layout ({len(d)} diff lines):")
        print("\n".join(d[:80]))
        return 1
    L = S2["layout"]
    print(f"stations {len(L['stations'])}, members {len(L['chassis']['members'])}, fittings "
          f"{len(L['chassis']['fittings'])}, panels {len(L['shell']['panels'])}, joints "
          f"{len(L['mechanisms']['joints'])}, clearances {len(L['clearances'])}, equipment "
          f"{len(L['systems']['equipment'])}, assembly steps {len(S2['assembly']['steps'])}")
    for k, v in L["mass_placement"].items():
        print(f"  {k:45s} {v['position']}  (sizing phase {v.get('position_sizing_phase')})")
    return 0


if __name__ == "__main__":
    sys.exit(main())
