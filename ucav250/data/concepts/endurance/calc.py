#!/usr/bin/env python3
"""YK-250 concept study "endurance" (MALE-type): reproducible first-pass sizing.

Run from the repository root:

    PYTHONPATH=. python3 ucav250/data/concepts/endurance/calc.py

Concept (decided by the trade studies in this file, reasons recorded in concept.yaml -> trades):
  * Limbach L 275 EF pusher (Mejzlik 32x18 2B) at the end of a slim single fuselage (0.44 m wide).
  * Shoulder-mounted, high-aspect-ratio, straight-tapered NLF(1)-0416 wing (13 % tip), two halves joined on the
    fuselage centre line (sailplane-style spar tongue/fork), plain flaps inboard, ailerons outboard.
  * Upright V-tail on the aft fuselage ahead of the propeller, ventral tail bumper under the engine bay.
  * Fixed tricycle gear: GFRP spring bow with faired TOST wheels, steerable faired nose leg.
  * Structure: chassis (CFRP keel beams + frames + machined fittings carrying all point loads) and shell
    (removable sandwich panels); see notes_tr.md.

Method (every number is read from the research files - key path in ``ref`` -, computed here - formula in
``basis`` - or a labelled engineering estimate):
  propulsion  engine WOT torque curve (datasheet) x sigma^1.23 lapse (engine.yaml), Mejzlik 32x18 2B table
              (manufacturer) with J-similarity, part-load BSFC from baseline.yaml (iterated with the actual power
              setting incl. the generator load).
  aero        aero.surface_analysis (NeuralFoil + Glauert lifting line) on the real wing sections for CL_alpha,
              e, CLmax (critical section, research cl_max factor 0.94) and stall onset; wing profile drag by strip
              integration of TRIPPED polars (x_tr = 0.075 c, NeuralFoil, same coordinates as aero.py) x 1.15 (the
              research sizing rule); component build-up with aerolib for fuselage, tail, gear, turret, cooling,
              interference and miscellaneous items; trimmed polar (tail load + thrust-line moment).
  mass        sizinglib.MassModel closed loop: payload + fixed equipment (bottom-up) + mission fuel fraction
              (sizinglib segments) + airframe (structural concept: process minimum gauges, spar caps sized by the
              CS-LUAS gust/manoeuvre envelope with structlib) with a 5 % mass-growth allowance.
  stability   stablib (V-tail projection, downwash, fuselage Cm_alpha, neutral point, static margin, Cn_beta).
  performance aerolib speeds, Vmax, rate of climb, ceiling, take-off/landing rolls; Breguet segments.
Outputs (this folder): concept.yaml, sketch.png, constraint.png.
"""
from __future__ import annotations

import functools
import math
import sys
from dataclasses import dataclass, replace
from pathlib import Path

import numpy as np
import yaml
from scipy.interpolate import RegularGridInterpolator

HERE = Path(__file__).resolve().parent
REPO = HERE.parents[3]
if str(REPO) not in sys.path:
    sys.path.insert(0, str(REPO))

from ucav250.analysis import aero as AE        # noqa: E402
from ucav250.analysis import aerolib as AL     # noqa: E402
from ucav250.analysis import sizinglib as SZ   # noqa: E402
from ucav250.analysis import stablib as SL     # noqa: E402
from ucav250.analysis import structlib as ST   # noqa: E402
from ucav250.design import oml                 # noqa: E402

RESEARCH = REPO / "ucav250" / "data" / "research"
G = AL.G0
RHO0 = 1.225


# =====================================================================================================================
# 0. helpers: research-file loading, input record, flags
# =====================================================================================================================
def _load(name: str) -> dict:
    loader = getattr(yaml, "CSafeLoader", yaml.SafeLoader)
    with open(RESEARCH / name, encoding="utf-8") as f:
        return yaml.load(f, Loader=loader)


BL = _load("baseline.yaml")
EN = _load("engine.yaml")
AY = _load("aero.yaml")
CP = _load("comparables.yaml")
CM = _load("components.yaml")

INPUTS: dict = {}       # every input with its source/basis -> concept.yaml
FLAGS: list = []        # warnings, infeasibilities, open items found by this study


def rec(key: str, value, src: str, tag: str = "estimate", basis: str | None = None):
    """Record an input (value + source or basis) for concept.yaml and return the value."""
    d = {"value": value, "tag": tag}
    if src:
        d["ref"] = src
    if basis:
        d["basis"] = basis
    INPUTS[key] = d
    return value


def flag(text: str):
    if text not in FLAGS:
        FLAGS.append(text)


def py(x, sig: int = 5):
    """numpy -> plain python, floats rounded to ``sig`` significant digits (YAML output)."""
    if isinstance(x, dict):
        return {str(k): py(v, sig) for k, v in x.items()}
    if isinstance(x, (list, tuple)):
        return [py(v, sig) for v in x]
    if isinstance(x, np.ndarray):
        return [py(v, sig) for v in x.tolist()]
    if isinstance(x, (np.integer,)):
        return int(x)
    if isinstance(x, (float, np.floating)):
        x = float(x)
        if not math.isfinite(x):
            return None
        if x == 0.0:
            return 0.0
        return float(f"{x:.{sig}g}")
    if isinstance(x, (np.bool_,)):
        return bool(x)
    return x


# =====================================================================================================================
# 1. research inputs
# =====================================================================================================================
MTG = BL["mission_targets"]
END_REQ_H = rec("endurance_requirement_h", MTG["endurance_h"]["value"], "baseline.yaml#mission_targets.endurance_h")
V_LOIT_MIN_EAS = rec("loiter_speed_floor_m_per_s_eas", float(MTG["loiter_speed_m_per_s_eas"]["range"][0]),
                     "baseline.yaml#mission_targets.loiter_speed_m_per_s_eas.range[0]", basis=(
                         "lower end of the 26-30 m/s EAS loiter band; the endurance concept loiters at the speed of "
                         "minimum fuel flow but never below this floor (wind penetration) nor below 1.2 VS"))
V_CRUISE_MIN, V_CRUISE_MAX = (float(v) for v in MTG["cruise_speed_m_per_s"]["range"])
rec("cruise_speed_band_m_per_s", [V_CRUISE_MIN, V_CRUISE_MAX], "baseline.yaml#mission_targets.cruise_speed_m_per_s.range")
H_CEIL_REQ = rec("service_ceiling_requirement_m", float(MTG["service_ceiling_m"]["value"]),
                 "baseline.yaml#mission_targets.service_ceiling_m")
H_LOITER = rec("loiter_altitude_m", float(MTG["operating_altitude_m"]["value"][1]),
               "baseline.yaml#mission_targets.operating_altitude_m[1]",
               basis="top of the 1000-3000 m EO/IR band: the most fuel-hungry loiter altitude (higher TAS)")
RUNWAY_M = rec("runway_length_m", float(MTG["runway_length_m"]["value"]), "baseline.yaml#mission_targets.runway_length_m")
ROC_REQ = rec("climb_rate_sea_level_requirement_m_per_s", float(MTG["climb_rate_sea_level_m_per_s"]["value"]),
              "baseline.yaml#mission_targets.climb_rate_sea_level_m_per_s")
R_TRANSIT_M = rec("transit_radius_m", 100e3, "", basis=(
    "concept assumption: 100 km out and 100 km back at best-range speed before/after the loiter (typical line-of-sight "
    "datalink radius of the class); the baseline gives no radius"))
RESERVE_FRAC = rec("reserve_fraction_of_trip_time", 0.10, "baseline.yaml#mission_targets.endurance_h.basis",
                   basis="'about 10 h with a 10 % reserve': reserve loiter = 10 % of the airborne trip time")
TRAPPED = rec("trapped_fuel_fraction", 0.02, "ucav250/analysis/sizinglib.py#mission_fuel_fraction (default)")

MTOM_CAP = rec("mtom_hard_cap_kg", float(BL["mass_targets"]["mtow_kg"]["hard_cap"]["value"]),
               "baseline.yaml#mass_targets.mtow_kg.hard_cap", "standard")
MTOM_DESIGN = rec("mtom_design_kg", float(BL["design_point"]["mtow_kg"]["value"]), "baseline.yaml#design_point.mtow_kg",
                  basis="design MTOM held at the baseline 145 kg (4.9 kg below the M2 cap); the endurance concept turns "
                        "the mass left after payload and empty mass into fuel")
GROWTH = rec("mass_growth_allowance_fraction_of_empty", 0.05, "", basis=(
    "concept-phase allowance on every estimated empty-mass item (fixed equipment + airframe) for detail growth"))

# payload (EO/IR and mission equipment only)
PS = BL["payload_set"]
PAYLOAD = rec("payload_design_kg", float(BL["mass_targets"]["payload_design_kg"]["value"]),
              "baseline.yaml#mass_targets.payload_design_kg")
M_TURRET = PS["eo_ir_turret"]["mass_kg"]["value"] + PS["eo_ir_turret"]["mount_and_isolator_kg"]["value"]
M_MCOMP = PS["mission_computer_and_recorder"]["mass_kg"]["value"]
M_TRAY = PS["payload_tray_harness_kg"]["value"]
M_RESEARCH = PS["research_payload_allowance_kg"]["value"]
TUR_D_GROWTH = PS["turret_bay_growth_envelope"]["diameter_m"]["value"]
TUR_H_GROWTH = PS["turret_bay_growth_envelope"]["height_m"]["value"]
TUR_D = PS["eo_ir_turret"]["diameter_m"]["value"]
rec("payload_items_kg", {"eo_ir_turret_hd59_with_mount": M_TURRET, "mission_computer_recorder": M_MCOMP,
                         "payload_tray_harness": M_TRAY, "research_payload_allowance": M_RESEARCH},
    "baseline.yaml#payload_set")
rec("turret_bay_growth_envelope_m", {"diameter": TUR_D_GROWTH, "height": TUR_H_GROWTH},
    "baseline.yaml#payload_set.turret_bay_growth_envelope", "manufacturer")

# engine
ENG = BL["engine"]
P_MAX = rec("engine_power_max_W", float(ENG["power_max_W"]["value"]), "baseline.yaml#engine.power_max_W", "manufacturer")
P_MCP = rec("engine_power_max_continuous_W", float(ENG["power_max_continuous_W"]["value"]),
            "baseline.yaml#engine.power_max_continuous_W")
WOT_RPM = np.array(ENG["wot_curve"]["rpm"], float)
WOT_TQ = np.array(ENG["wot_curve"]["torque_N_m"], float)
rec("engine_wot_curve", {"rpm": WOT_RPM.tolist(), "torque_N_m": WOT_TQ.tolist()}, "baseline.yaml#engine.wot_curve",
    "datasheet")
LAPSE_EXP = rec("engine_power_lapse_exponent", 1.23, "engine.yaml#installation.altitude.power_lapse_model",
                basis="P(h)/P0 = sigma^1.23 (Hirth 4201 fit); used instead of aerolib Gagg-Ferrar (compared in output)")
BSFC_PTS = np.array(ENG["bsfc_curve"]["points"], float)
BSFC_UNC = ENG["bsfc_curve"]["uncertainty_fraction"]
rec("bsfc_curve_power_fraction_g_per_kWh", BSFC_PTS.tolist(), "baseline.yaml#engine.bsfc_curve.points",
    basis="fraction of 18 kW; linear interpolation; below 0.20 linear extrapolation with the 0.20-0.30 slope (flagged)")
RHO_FUEL = rec("fuel_density_kg_per_m3", float(ENG["fuel"]["density_kg_per_m3"]["value"]),
               "baseline.yaml#engine.fuel.density_kg_per_m3")
M_ENGINE_GROUP = rec("engine_group_installed_kg", float(ENG["installed_mass_kg"]["total"]["value"]),
                     "baseline.yaml#engine.installed_mass_kg.total")
GEN_W_7500 = ENG["generator"]["power_continuous_W"]["value"]
P_ELEC = rec("electrical_load_continuous_W", float(BL["subsystems"]["electrical_load_W"]["continuous_total"]["value"]),
             "baseline.yaml#subsystems.electrical_load_W.continuous_total")
ETA_GEN = rec("generator_efficiency", 0.85, "baseline.yaml#cross_checks.loiter_fuel_flow.basis")
P_GEN_SHAFT = P_ELEC / ETA_GEN
ENV = ENG["envelope_m"]
ENV_L = ENV["length_prop_hub_face_to_generator_flange"]["value"]
ENV_L_SG = ENV["length_with_sg750"]["value"]
ENV_W = ENV["width_incl_spark_plug_caps"]["value"]
ENV_H = ENV["height_cylinder_side_to_intake_cover_top"]["value"]
ENV_ZC = ENV["crank_axis_to_cylinder_side_extreme"]["value"]
rec("engine_envelope_m", {"length": ENV_L, "length_with_sg750": ENV_L_SG, "width": ENV_W, "height": ENV_H,
                          "crank_axis_to_cylinder_side": ENV_ZC}, "baseline.yaml#engine.envelope_m", "paper")

# propeller
PRP = BL["propeller"]["primary"]
D_PROP = rec("propeller_diameter_m", PRP["diameter_m"]["value"], "baseline.yaml#propeller.primary.diameter_m",
             "manufacturer")
M_PROP = PRP["mass_kg"]["value"]
K_WOT = rec("propeller_wot_thrust_factor", 0.92, "baseline.yaml#propeller.primary.static_thrust_N_for_sizing",
            basis="Mejzlik table thrust x 0.92 (Falcon measured/Mejzlik ratio, lower half) for every WOT case")
K_INST = rec("propeller_installation_factor", PRP["installation_factor"]["value"],
             "baseline.yaml#propeller.primary.installation_factor",
             basis="pusher behind the fuselage: thrust at a given shaft power x 0.95 in part-throttle flight")
TIP_LIMIT = PRP["tip_speed_limit_m_per_s"]["value"]
PROP_ROWS = AY["propeller_tables_mejzlik"]["0161"]["rows_rpm_thrust_N_torque_Nm_power_W"]
rec("propeller_table", "Mejzlik 32x18 2B sheet 0161 (rpm, thrust, torque) at 0-50 m/s",
    "aero.yaml#propeller_tables_mejzlik.0161", "manufacturer")
CLEAR_PROP_GROUND = rec("prop_ground_clearance_min_m", 0.18, "standards.yaml#propeller_clearance.ground_m", "standard")
CLEAR_PROP_LONG = 0.013   # CS-VLA 925(c)(2) standards.yaml#propeller_clearance.longitudinal_m

# aerodynamics corrections (research sizing rule)
CORR = BL["aero"]["corrections_for_sizing"]
K_CLMAX_SEC = rec("cl_max_section_factor", CORR["cl_max_factor"]["value"], "baseline.yaml#aero.corrections_for_sizing")
K_CD_TRIP = rec("cd_factor_tripped", CORR["cd_factor_tripped"]["value"], "baseline.yaml#aero.corrections_for_sizing")
XTR_TRIP = rec("trip_location_x_over_c", 0.075, "aero.yaml#method.polars.cases.tripped")
K_CLMAX_3D = rec("k_clmax_3d", 0.95, "ucav250/analysis/aero.py#surface_analysis (default k_clmax)")

# loads
DL = BL["design_loads"]
N_POS = DL["manoeuvre"]["n_limit_pos"]["value"]
N_NEG = DL["manoeuvre"]["n_limit_neg_at_VC"]["value"]
UDE_C = DL["gust"]["Ude_at_VC_m_per_s"]["value"]
UDE_D = DL["gust"]["Ude_at_VD_m_per_s"]["value"]
FOS = DL["factors"]["factor_of_safety"]["value"]
VC_EAS = rec("VC_m_per_s_eas", BL["design_speeds_m_per_s_eas"]["VC"]["value"], "baseline.yaml#design_speeds_m_per_s_eas.VC")
TYRE_V_LIMIT = BL["design_speeds_m_per_s_eas"]["tyre_speed_limit"]["value"]

# materials (design values) and process minimum gauges
MAT = BL["materials_to_use"]
UD = MAT["cfrp_ud_mtm45_as4"]
PW = MAT["cfrp_pw_mtm45_as4"]
EPS_DT = DL["composites"]["damage_tolerance_strain_limits_microstrain"]["thick_gt_2mm"]["compression"] * 1e-6
SIG_CAP = rec("spar_cap_allowable_ultimate_Pa", min(UD["E"] * EPS_DT, UD["Fcu"] * 0.85 / 1.2),
              "baseline.yaml#design_loads.composites + materials_to_use.cfrp_ud_mtm45_as4",
              basis="min(E x 3000 microstrain damage-tolerance limit, Fcu B-basis ETW x 0.85 (single load path) / 1.2 "
                    "special factor)")
TAU_WEB = rec("shear_web_allowable_ultimate_Pa", MAT["cfrp_qi_laminate_design_values"]["G_qi_Pa"]["value"] * 5200e-6,
              "baseline.yaml#materials_to_use.cfrp_qi_laminate_design_values.G_qi_Pa + damage-tolerance shear 5200 ue")
PLY_PW = PW["ply_t"] * PW["density"]             # kg/m2 per plain-weave ply
CORE_FOAM = 52.0                                 # Rohacell 51 WF(-HT), kg/m3 (baseline materials_to_use.cores)
PAINT = 0.10                                     # kg/m2 primer + paint, estimate (about 80 um at 1.3 g/cm3)
SKIN_PRIMARY = 3 * PLY_PW + 0.005 * CORE_FOAM + 2 * PLY_PW + PAINT     # 0.6 / 5 mm foam / 0.4 mm
SKIN_SECONDARY = 2 * PLY_PW + 0.005 * CORE_FOAM + 2 * PLY_PW + PAINT   # 0.4 / 5 mm foam / 0.4 mm
SKIN_TAIL = 3 * PLY_PW + 0.004 * CORE_FOAM + 2 * PLY_PW + PAINT
RIB_AREAL = 2 * PLY_PW + 0.006 * CORE_FOAM + 2 * PLY_PW              # 0.4 / 6 mm foam / 0.4 mm sandwich rib
rec("areal_masses_kg_per_m2", {"wing_skin_primary": SKIN_PRIMARY, "fuselage_shell_secondary": SKIN_SECONDARY,
                               "tail_skin": SKIN_TAIL, "rib_panel": RIB_AREAL, "paint_primer": PAINT},
    "baseline.yaml#processes.min_laminate_thickness_m + materials_to_use", basis=(
        "MTM45-1/AS4 plain weave 0.2 mm plies; primary sandwich faces 0.6/0.4 mm (min external 0.6 mm), secondary "
        "0.4/0.4 mm; ROHACELL 51 WF-HT foam core co-cured (no film adhesive); paint 0.10 kg/m2 (estimate)"))

# fixed equipment (bottom-up from baseline systems roll-up, adjusted for this configuration)
SR = BL["mass_targets"]["systems_rollup_kg"]
FIXED_ITEMS = {
    "engine_group_installed": (SR["engine_group_installed"]["value"], "baseline.yaml#mass_targets.systems_rollup_kg"),
    "propeller": (SR["propeller"]["value"], "baseline.yaml#mass_targets.systems_rollup_kg"),
    "spinner_hub_adapter_and_80mm_spacer": (SR["spinner_and_hub_adapter"]["value"] + 0.10,
                                            "baseline 0.40 + 0.10 aluminium hub spacer (estimate)"),
    "baffles_ducts_firewall_cowl_flap": (SR["cowling_cooling_ducts_baffles_firewall"]["value"],
                                         "baseline.yaml (cowl skin itself counted in the fuselage shell)"),
    "fuel_system_bladder_55L": (SR["fuel_system"]["value"] + 0.35,
                                "baseline 1.73 (one 45 L bladder) + 0.35 kg for two interconnected cells of ~27 L "
                                "(extra bladder area, interconnect, second vent; estimate)"),
    "flight_control_actuators": (6 * 0.27 + 0.35 + 0.10,
                                 "6 x Volz DA 26 0.27 kg (2 ailerons, 2 ruddervators, nose steering, brake) + 0.35 kg "
                                 "installation (baseline share) + 0.10 kg cowl-flap servo; the 2 x DA 30 flap drives of "
                                 "the baseline are deleted (no flaps)"),
    "avionics": (SR["avionics"]["value"], "baseline.yaml#mass_targets.systems_rollup_kg"),
    "electrical_power": (SR["electrical_power"]["value"], "baseline.yaml#mass_targets.systems_rollup_kg"),
    "wiring_harness": (SR["wiring_harness_connectors_coax"]["value"] + 0.20,
                       "baseline 2.50 + 0.20 kg for the longer wing harness (estimate)"),
    "recovery_and_safety": (SR["recovery_and_safety"]["value"], "baseline.yaml#mass_targets.systems_rollup_kg"),
}
M_FIXED = sum(v for v, _ in FIXED_ITEMS.values())
FLAP_KIT = rec("flap_kit_kg", 2 * 0.63 + 0.10 + 2 * 0.10, "baseline.yaml#subsystems.flight_control_actuators.flaps",
               "datasheet", basis="2 x Volz DA 30 0.63 kg + 0.10 kg installation + 2 x 0.10 kg hinges/horns (estimate)")


def fixed_mass(d) -> float:
    return M_FIXED + (FLAP_KIT if d.flaps else 0.0) + (PROP_MASS_DELTA if PROP_KEY != "0161" else 0.0)
rec("fixed_equipment_items_kg", {k: v for k, (v, _) in FIXED_ITEMS.items()},
    "baseline.yaml#mass_targets.systems_rollup_kg (landing gear moved to the airframe group)",
    basis="; ".join(f"{k}: {s}" for k, (_, s) in FIXED_ITEMS.items()))

# landing gear (components.yaml recommended set) - moved to the airframe group because it scales with MTOW
GEAR_WHEELS = 2 * 1.79 + 0.365 + 0.45 + 0.08     # 2 braked main wheel assemblies + nose wheel/tyre/tube
GEAR_LEGS_145 = SR["landing_gear"]["value"] - GEAR_WHEELS
GEAR_FAIRINGS = rec("gear_fairings_kg", 0.95, "", basis=(
    "estimate: 2 main wheel spats 0.25 kg + nose spat 0.20 kg + bow leg fairings 0.25 kg (GFRP 0.5 mm)"))
_TOST = next(i for i in CM["categories"]["landing_gear"]["items"] if i.get("id") == "tost_sb_max2_70_50_20")
WHEEL_D = _TOST["tyre"]["outer_diameter_mounted_m"]["value"]
WHEEL_W = 0.054
rec("main_wheel_tyre_m", {"diameter": WHEEL_D, "width": WHEEL_W}, "components.yaml#categories.landing_gear "
    "(TOST 200x50 tyre)", "manufacturer")

# parachute container (Galaxy GRS 4/240 B8)
CHUTE_BOX = [0.205, 0.110, 0.375]


# =====================================================================================================================
# 2. propulsion model
# =====================================================================================================================
def lapse(sigma: float) -> float:
    return sigma ** LAPSE_EXP


def engine_torque_wot(n, sigma: float):
    """WOT torque (N m) at rpm n (datasheet curve; held at 25.3 N m below 4000 rpm - no data, flagged in notes)."""
    n = np.asarray(n, float)
    q = np.interp(n, WOT_RPM, WOT_TQ) * lapse(sigma)
    return np.where(n > 8000.0, 0.0, q)


BSFC_SCALE = 1.0      # sensitivity runs only (baseline uncertainty +/-12 %)


def bsfc_g_kwh(p_total: float) -> float:
    return BSFC_SCALE * _bsfc(p_total)


def _bsfc(p_total: float) -> float:
    f = p_total / P_MAX
    f0, f1 = BSFC_PTS[0, 0], BSFC_PTS[1, 0]
    b0, b1 = BSFC_PTS[0, 1], BSFC_PTS[1, 1]
    if f < f0:
        return float(b0 + (b1 - b0) / (f1 - f0) * (f - f0))
    return float(np.interp(f, BSFC_PTS[:, 0], BSFC_PTS[:, 1]))


class Prop:
    """Mejzlik 32x18 2B from the manufacturer table; J-similarity (CT, CQ constant at equal J) outside the tabulated
    rpm range; density scaling sigma at altitude (same J, T and Q proportional to rho)."""
    NGRID = np.linspace(1200.0, 8000.0, 273)

    def __init__(self, rows: dict, D: float):
        Vs = sorted(float(k.split("_")[1]) for k in rows)
        tabs = {float(k.split("_")[1]): np.array(v, float) for k, v in rows.items()}
        rpm = tabs[Vs[0]][:, 0]
        for V in Vs:
            assert np.allclose(tabs[V][:, 0], rpm), "Mejzlik rows must share the rpm grid"
        self.V = np.array(Vs)
        self.N = rpm
        T = np.array([tabs[V][:, 1] for V in Vs])
        Q = np.array([tabs[V][:, 2] for V in Vs])
        self.fT = RegularGridInterpolator((self.V, self.N), T, bounds_error=False, fill_value=None)
        self.fQ = RegularGridInterpolator((self.V, self.N), Q, bounds_error=False, fill_value=None)
        self.D = D

    def sl(self, V: float, n):
        n = np.atleast_1d(np.asarray(n, float))
        nr = np.clip(n, self.N[0], self.N[-1])
        s = n / nr
        Vr = V / s
        pts = np.column_stack([np.clip(Vr, 0.0, 60.0), nr])
        return self.fT(pts) * s * s, self.fQ(pts) * s * s

    def cruise(self, T_req: float, V: float, h: float) -> dict:
        """Part-throttle point: rpm, shaft power, installed efficiency for thrust T_req at TAS V, altitude h."""
        atm = AL.isa(h)
        sig = atm["sigma"]
        T, Q = self.sl(V, self.NGRID)
        T = np.maximum.accumulate(sig * T * K_INST)
        if T_req > T[-1]:
            return {"ok": False}
        n = float(np.interp(T_req, T, self.NGRID))
        q = sig * float(np.interp(n, self.NGRID, Q))
        P = q * 2 * math.pi * n / 60
        tip = math.hypot(math.pi * self.D * n / 60, V) / atm["a"]
        return {"ok": True, "rpm": n, "P_shaft": P, "eta": T_req * V / P if P > 0 else 0.0, "tip_mach": tip}

    def wot(self, V: float, h: float, p_gen: float = P_GEN_SHAFT) -> dict:
        """Full-throttle point (engine torque minus generator load = prop torque), thrust x K_WOT."""
        atm = AL.isa(h)
        sig = atm["sigma"]
        n = self.NGRID
        T, Q = self.sl(V, n)
        omega = 2 * math.pi * n / 60
        diff = engine_torque_wot(n, sig) - p_gen / omega - sig * Q
        m = n <= 7500.0
        d = diff[m]
        if d[-1] > 0:                                           # prop under-loads the engine: rpm-limited
            nn = 7500.0
        else:
            i = int(np.argmax(d <= 0))
            nn = float(np.interp(0.0, [-d[i - 1], -d[i]], [n[i - 1], n[i]])) if i > 0 else float(n[0])
        Tn, Qn = self.sl(V, nn)
        P = sig * float(Qn[0]) * 2 * math.pi * nn / 60
        Tt = sig * float(Tn[0]) * K_WOT
        tip = math.hypot(math.pi * self.D * nn / 60, V) / atm["a"]
        return {"rpm": nn, "T": Tt, "P_shaft": P, "eta": Tt * V / P if P > 0 and V > 0 else 0.0, "tip_mach": tip}


PROP = Prop(PROP_ROWS, D_PROP)
PROP_KEY = "0161"
PROP_MASS_DELTA = 0.0
PROP_ALT = BL["propeller"]["alternative"]


def set_prop(key: str):
    """Switch between the Mejzlik 32x18 2B (0161, primary) and 31x12 3B (0164, alternative) for the propeller trade."""
    global PROP, PROP_KEY, D_PROP, PROP_MASS_DELTA
    rows = AY["propeller_tables_mejzlik"][key]["rows_rpm_thrust_N_torque_Nm_power_W"]
    D = PRP["diameter_m"]["value"] if key == "0161" else PROP_ALT["diameter_m"]["value"]
    PROP = Prop(rows, D)
    PROP_KEY = key
    D_PROP = D
    PROP_MASS_DELTA = 0.0 if key == "0161" else PROP_ALT["mass_kg"]["value"] - M_PROP


# =====================================================================================================================
# 3. tripped section polars (aero.py has free transition only; same coordinates and alpha grid, x_tr = 0.075 c)
# =====================================================================================================================
@functools.lru_cache(maxsize=None)
def _raw_tripped(airfoil: str, Re: float, ts: float) -> dict:
    import neuralfoil as nf
    P = AE.section_coords(airfoil, ts)
    a = np.array(AE.ALPHAS)
    r = nf.get_aero_from_coordinates(P, alpha=a, Re=float(Re), n_crit=9.0, xtr_upper=XTR_TRIP, xtr_lower=XTR_TRIP,
                                     model_size="xlarge")
    return {"alpha": list(AE.ALPHAS), "cl": [float(v) for v in r["CL"]], "cd": [float(v) for v in r["CD"]],
            "cm": [float(v) for v in r["CM"]], "confidence": [float(v) for v in r["analysis_confidence"]]}


@functools.lru_cache(maxsize=None)
def tripped_chars(airfoil: str, Re: float, ts: float = 1.0) -> dict:
    """aero.characteristics() equivalent for the tripped case (log-Re interpolation on aero.RE_GRID)."""
    grid = AE.RE_GRID
    Re = float(np.clip(Re, grid[0], grid[-1]))
    j = min(max(int(np.searchsorted(grid, Re)), 1), len(grid) - 1)
    r0, r1 = grid[j - 1], grid[j]
    t = (math.log(Re) - math.log(r0)) / (math.log(r1) - math.log(r0))
    c0 = AE.polar_characteristics(_raw_tripped(airfoil, r0, ts))
    c1 = AE.polar_characteristics(_raw_tripped(airfoil, r1, ts))
    out = {k: (1 - t) * c0[k] + t * c1[k] for k in c0 if k != "cd_table"}
    out["_tables"] = (c0["cd_table"], c1["cd_table"], t)
    return out


def strip_profile_drag(sa: dict, sections: list, V: float, h: float, tripped: bool = True):
    """Returns f(CL) -> profile drag coefficient on S_ref by strip integration of the tripped (or clean) section
    polars at the lifting-line cl distribution of ``sa`` (aero.surface_analysis result)."""
    T = sa["tables"]
    LL = sa["ll"]
    atm = AL.isa(h)
    re = atm["rho"] * V * T["chord"] / atm["mu"]
    ys = LL["y"]
    chars = []
    for yk in ys:
        j = int(np.argmin(np.abs(T["y"] - yk)))
        i, f = int(T["idx"][j]), float(T["frac"][j])
        sa_, sb_ = sections[i], sections[i + 1]
        if tripped:
            ca = tripped_chars(sa_["airfoil"], float(re[j]), float(sa_.get("thickness_scale", 1.0)))
            cb = tripped_chars(sb_["airfoil"], float(re[j]), float(sb_.get("thickness_scale", 1.0)))
        else:
            ca = AE.characteristics(sa_["airfoil"], float(re[j]), 9.0, float(sa_.get("thickness_scale", 1.0)))
            cb = AE.characteristics(sb_["airfoil"], float(re[j]), 9.0, float(sb_.get("thickness_scale", 1.0)))
        chars.append((ca, cb, f))
    order = np.argsort(ys)

    def cdp(CL: float) -> float:
        cl = LL["cl_local"](sa["alpha_for"](CL))
        cd = np.array([(1 - f) * AE.section_cd(ca, float(c)) + f * AE.section_cd(cb, float(c))
                       for (ca, cb, f), c in zip(chars, cl)])
        return float(2 * np.trapz((cd * LL["chord"])[order], ys[order]) / LL["S_ref"])
    cm0 = float(np.mean([(1 - f) * ca["cm0"] + f * cb["cm0"] for ca, cb, f in chars]))
    return cdp, cm0


# =====================================================================================================================
# 4. configuration parameters and fixed packaging
# =====================================================================================================================
@dataclass(frozen=True)
class Design:
    mtom: float = MTOM_DESIGN
    ws: float = 433.0            # wing loading, Pa (trade: endurance optimum, constraint diagram)
    AR: float = 14.0             # trade 12-15
    taper: float = 0.45
    washout: float = 2.5         # deg, baseline.yaml#aero.wing_airfoil.washout_deg
    dihedral: float = 1.0        # deg (high wing + upright V already give positive dihedral effect)
    ts_tip: float = 0.8125       # NLF(1)-0416 scaled to 13 % at the tip (baseline)
    VH: float = 0.45             # horizontal tail volume target (projected), see rec below
    cnb_req: float = 0.057       # 1/rad (0.001 per deg) directional stability target
    VV_min: float = 0.020
    tail_AR: float = 4.5         # unfolded V-tail aspect ratio (2 s)^2 / S_V
    tail_taper: float = 0.60
    tail_le_sweep: float = 30.0  # deg (MALE look)
    sm_min: float = 0.10         # minimum static margin over all loading cases (sets the wing station)
    flaps: bool = True           # inboard plain take-off flaps (15 deg); landing flaps-up (touch-down attitude)
    flap_to_deg: float = 15.0
    flap_span_frac: float = 0.55 # flap from the fuselage side to 55 % semispan; ailerons 57-95 % semispan
    cf_c: float = 0.25           # flap/aileron chord ratio
    tail: str = "V"


rec("tail_volume_target_VH", 0.45, "", basis=(
    "between sailplane/homebuilt 0.50 and the minimum that keeps SM >= 0.10 over the CG range (checked); Raymer, "
    "Aircraft Design 6th ed. Table 6.4 typical values"))
rec("directional_stability_target_per_rad", 0.057, "", basis="Cn_beta >= 0.001 per deg (common conceptual-design "
    "minimum, Nicolai/Raymer); sizes the vertical projection of the V-tail")
rec("static_margin_min", 0.10, "", basis="stick-fixed minimum over all loading cases; leaves margin for the estimate "
    "uncertainty (NP and CG +/-3 % MAC) above the 0.05 usually accepted for autopilot-flown UAVs")

# fuselage packaging (X aft from the nose tip, Z up from the fuselage datum line through the nose tip)
HUB_SPACER = rec("prop_hub_spacer_m", 0.08, "", basis=(
    "aluminium spacer so the cowl can close to the spinner in 0.10 m (Limbach approval needed; open item)"))
Z_T = rec("thrust_line_z_m", 0.21, "", basis="crank axis 0.06 below the engine-bay top (intakes hang below, datasheet "
          "orientation): highest thrust line the 0.44 m body allows -> prop ground clearance")
FUS_BASE = np.array([
    # x,    w,     h,     zc,    n_top, n_bot, top_frac        (stations at x >= 2.15 move aft with the stretch)
    [0.00, 0.000, 0.000, 0.000, 2.2, 1.7, 0.50],
    [0.10, 0.150, 0.130, 0.000, 2.2, 1.7, 0.52],
    [0.30, 0.300, 0.290, 0.004, 2.3, 1.7, 0.54],
    [0.55, 0.400, 0.400, 0.004, 2.4, 1.8, 0.55],
    [0.90, 0.440, 0.440, 0.000, 2.5, 1.8, 0.55],
    [2.15, 0.440, 0.440, 0.000, 2.5, 1.8, 0.55],
    [2.75, 0.440, 0.445, 0.100, 3.0, 2.2, 0.42],
    [3.20, 0.440, 0.455, 0.210, 3.5, 2.5, 0.26],
    [3.37, 0.440, 0.455, 0.210, 3.5, 2.5, 0.26],
    [3.47, 0.300, 0.290, 0.210, 2.2, 2.2, 0.50],
])
X_HUB_BASE = 3.42


def set_fuselage(stretch: float):
    """(Re)build the fuselage for an aft-body stretch (constant-section plug between the wing and the engine bay):
    every station from x = 2.15 aft, the engine, propeller and tail move aft by ``stretch``."""
    global FUS, FUSE, L_FUS, X_HUB, X_PROP, X_LIP, SPINNER, AFT_STRETCH
    F = FUS_BASE.copy()
    F[F[:, 0] > 2.15 + 1e-9, 0] += stretch
    if stretch > 0:
        F = np.insert(F, 6, [2.15 + stretch, 0.440, 0.440, 0.000, 2.5, 1.8, 0.55], axis=0) if stretch > 0.05 else F
    FUS = F
    FUSE = oml.Fuselage(F)
    L_FUS = float(F[-1, 0])
    X_HUB = X_HUB_BASE + stretch
    X_PROP = X_HUB + HUB_SPACER + 0.02          # prop plane (mid-hub)
    X_LIP = X_HUB + 0.05                        # cowl lip / annular cooling exit
    SPINNER = {"x0": X_HUB + 0.07, "x1": X_PROP + 0.12, "d": 0.17}
    AFT_STRETCH = stretch
    fuselage_mesh_props.cache_clear()


X_TURRET = 0.38                             # EO/IR ball centre (chin bay)
X_NG = 0.72                                 # nose gear leg (frame F3), behind the turret bay
X_BATT = 0.30                               # 14S2P LiFePO4 pack + PDU in the nose bay (balances the aft engine)
X_AVION = 0.42                              # autopilot, IMU, GNSS, transponder, datalink tray
BAY_LEN = rec("payload_bay_length_m", 0.40, "", basis="research payload bay (nadir window) centred on the CG between "
              "two interconnected fuel cells: payload changes and fuel burn leave the CG in place; 0.40 x 0.38 x 0.30 m "
              "internal (about 45 L)")
X_CHUTE = (0.62, 1.00)                      # parachute bay (top hatch) ahead of the wing: canopy far from the prop
TANK_INSET = 0.03                           # skin + frame flange + clearance around the bladder
K_TANK = rec("tank_volume_efficiency", 0.85, "", basis="bladder conformity, plumbing, probe and frames in the bay")


def fus_section(x):
    w, h, zc, nt, nb = FUSE.section(np.atleast_1d(np.asarray(x, float)))
    tf = FUSE.top_frac(np.atleast_1d(np.asarray(x, float)))
    return w, h, zc, nt, nb, tf


def z_top(x):
    w, h, zc, nt, nb, tf = fus_section(x)
    return zc + tf * h


def z_bot(x):
    w, h, zc, nt, nb, tf = fus_section(x)
    return zc - (1 - tf) * h


def superellipse_half_area(a, b, n):
    return 2 * a * b * math.gamma(1 + 1 / n) ** 2 / math.gamma(1 + 2 / n)


def internal_area(x, inset=TANK_INSET):
    w, h, zc, nt, nb, tf = (float(v[0]) for v in fus_section(x))
    a = max(w / 2 - inset, 0.0)
    return superellipse_half_area(a, max(tf * h - inset, 0), nt) + superellipse_half_area(a, max((1 - tf) * h - inset, 0), nb)


def inside_oml(x, y, z, margin=0.0) -> bool:
    w, h, zc, nt, nb, tf = (float(v[0]) for v in fus_section(x))
    a = w / 2 - margin
    if z >= zc:
        b, n = tf * h - margin, nt
    else:
        b, n = (1 - tf) * h - margin, nb
    if a <= 0 or b <= 0:
        return False
    return (abs(y) / a) ** n + (abs(z - zc) / b) ** n <= 1.0


@functools.lru_cache(maxsize=1)
def fuselage_mesh_props() -> dict:
    m = FUSE.mesh(160, 96)
    V, F = m.V, m.F
    A = 0.5 * np.linalg.norm(np.cross(V[F[:, 1]] - V[F[:, 0]], V[F[:, 2]] - V[F[:, 0]]), axis=1)
    C = V[F].mean(axis=1)
    w, h, *_ = fus_section(L_FUS)
    end_cap = math.pi / 4 * float(w[0]) * float(h[0])
    cap = C[:, 0] > L_FUS - 1e-4
    area = float(A[~cap].sum())
    xc = float((A[~cap] * C[~cap, 0]).sum() / area)
    xs = np.linspace(0, L_FUS, 400)
    ws, hs, *_ = fus_section(xs)
    side = float(np.trapz(hs, xs))
    amax = max(internal_area(x, 0.0) for x in np.linspace(0.5, 3.4, 30))
    return {"S_wet": area, "x_centroid": xc, "volume": float(abs(m.volume())), "S_side": side, "A_max": amax,
            "end_cap_area": end_cap, "w_max": float(ws.max()), "h_max": float(hs.max()), "mesh": m}


set_fuselage(0.0)


# =====================================================================================================================
# 5. geometry builders
# =====================================================================================================================
def wing_planform(d: Design, S: float) -> dict:
    b = math.sqrt(d.AR * S)
    cr = 2 * S / (b * (1 + d.taper))
    ct = d.taper * cr
    mac = 2 / 3 * cr * (1 + d.taper + d.taper ** 2) / (1 + d.taper)
    y_mac = b / 6 * (1 + 2 * d.taper) / (1 + d.taper)
    return {"S": S, "b": b, "cr": cr, "ct": ct, "mac": mac, "y_mac": y_mac, "AR": d.AR, "taper": d.taper}


def wing_sections(d: Design, pf: dict, x_le_root: float, i_w: float) -> list:
    """LiftingSurface sections (starboard half). Quarter-chord line unswept; linear chord, twist, thickness."""
    s = pf["b"] / 2
    z0 = Z_WING_ROOT
    return [
        {"y": 0.0, "x_le": x_le_root, "z_le": z0, "chord": pf["cr"], "twist_deg": i_w, "airfoil": "nlf416",
         "thickness_scale": 1.0},
        {"y": s, "x_le": x_le_root + 0.25 * (pf["cr"] - pf["ct"]), "z_le": z0 + s * math.tan(math.radians(d.dihedral)),
         "chord": pf["ct"], "twist_deg": i_w - d.washout, "airfoil": "nlf416", "thickness_scale": d.ts_tip},
    ]


Z_WING_ROOT = rec("wing_root_le_z_m", 0.270, "", basis="shoulder wing: lower surface rests on the 0.24 m fuselage top "
                  "(saddle fairing); NLF(1)-0416 lower surface about 0.03 c below the chord")
TC_ROOT = 0.16


def tail_sections(d: Design, S_V: float, gamma_deg: float, x_te_root: float) -> list:
    """Upright V-tail, starboard panel: root on the aft-fuselage top, panel span s along the dihedral line."""
    span2 = math.sqrt(d.tail_AR * S_V)            # unfolded tip-to-tip span
    s = span2 / 2
    cr = S_V / (s * (1 + d.tail_taper))           # S_V = 2 * s * (cr + ct) / 2
    ct = d.tail_taper * cr
    g = math.radians(gamma_deg)
    y0 = 0.06
    x_le0 = x_te_root - cr
    z0 = float(z_top(x_le0 + 0.4 * cr)[0]) - 0.02
    dx = s * math.tan(math.radians(d.tail_le_sweep))
    return [
        {"y": y0, "x_le": x_le0, "z_le": z0, "chord": cr, "twist_deg": 0.0, "airfoil": "n0012"},
        {"y": y0 + s * math.cos(g), "x_le": x_le0 + dx, "z_le": z0 + s * math.sin(g), "chord": ct, "twist_deg": 0.0,
         "airfoil": "n0012"},
    ]


def surface_mac(sections: list) -> dict:
    """MAC length and LE x along the panel span (works for the V panel too)."""
    P = np.array([[s["y"], s["z_le"]] for s in sections])
    eta = np.r_[0.0, np.cumsum(np.linalg.norm(np.diff(P, axis=0), axis=1))]
    secs = [dict(s, y=float(e)) for s, e in zip(sections, eta)]
    m = oml.mean_aerodynamic_chord(secs)
    frac = m["y_mac"] / eta[-1]
    z = sections[0]["z_le"] + frac * (sections[-1]["z_le"] - sections[0]["z_le"])
    y = sections[0]["y"] + frac * (sections[-1]["y"] - sections[0]["y"])
    return {"mac": m["mac"], "x_le_mac": m["x_le_mac"], "z_mac": z, "y_mac": y, "panel_area": m["half_area"],
            "panel_span": float(eta[-1])}


# =====================================================================================================================
# 6. aerodynamic model of one configuration
# =====================================================================================================================
def flap_effect(d: Design, pf: dict, w_fus: float, delta: float) -> dict:
    """Plain flaps from the fuselage side to ``flap_span_frac`` of the semispan (Raymer 6th ed. eqs. 12.21, 12.22,
    12.61): dCLmax = 0.9 dClmax (S_flapped/S), dClmax(plain) = 0.9 at about 40 deg and linear below (estimate);
    dalpha0 = -15 deg x (delta/40) x (S_flapped/S); dCD0 = 0.0144 (cf/c) (S_flapped/S) (delta - 10)."""
    s = pf["b"] / 2
    y0, y1 = w_fus / 2, d.flap_span_frac * s
    c = lambda y: pf["cr"] + (pf["ct"] - pf["cr"]) * y / s
    r = 2 * (y1 - y0) * (c(y0) + c(y1)) / 2 / pf["S"]
    f = min(max(delta, 0.0) / 40.0, 1.0)
    return {"delta_deg": delta, "S_flapped_over_S": r, "dCLmax": 0.9 * 0.9 * f * r, "dalpha0_deg": -15.0 * f * r,
            "dCD0": 0.0144 * d.cf_c * r * max(delta - 10.0, 0.0)}


@dataclass
class Aircraft:
    d: Design
    m0: float
    pf: dict
    wing_secs: list
    tail_secs: list
    sa: dict
    S: float
    cd_table: tuple
    fit: dict
    clmax: dict
    geo: dict
    drag_items: dict
    stab: dict
    mass: dict
    layout: dict

    def CD(self, CL):
        return np.interp(CL, self.cd_table[0], self.cd_table[1])


def build_aero(d: Design, pf: dict, wsecs: list, V_ref: float, h_ref: float, i_w: float) -> dict:
    sa = AE.surface_analysis(wsecs, V_ref, h_ref, k_clmax=K_CLMAX_3D)
    LL = sa["ll"]
    # CLmax with the research section factor 0.94 (critical-section method of aero.py re-evaluated)
    clmax_s = np.interp(LL["y"], sa["tables"]["y"], sa["clmax_sections"]) * K_CLMAX_SEC
    a_crit = (clmax_s - LL["cl_0_local"]) / LL["cl_a_local"]
    i = int(np.argmin(a_crit))
    clmax_w = K_CLMAX_3D * LL["CL"](float(a_crit[i]))
    cdp, cm0 = strip_profile_drag(sa, wsecs, V_ref, h_ref, tripped=True)
    cdp_clean, _ = strip_profile_drag(sa, wsecs, V_ref, h_ref, tripped=False)
    return {"sa": sa, "CL_alpha": sa["CL_alpha"], "e_inv": sa["e_inviscid"], "CLmax_wing": clmax_w,
            "alpha_stall_deg": math.degrees(float(a_crit[i])) , "stall_eta": float(LL["y"][i] / sa["semispan"]),
            "cdp": cdp, "cdp_clean": cdp_clean, "cm0": cm0, "CL_0": sa["CL_0"], "Re_root": sa["Re_root"],
            "Re_tip": sa["Re_tip"], "min_conf": sa["min_confidence"]}


def drag_buildup(d: Design, pf: dict, tail: dict, gear: dict, V: float, h: float, fuel_flow_ratio: float) -> dict:
    """Parasite drag items other than the wing profile drag, as D/q (m2) at TAS V, altitude h (aerolib)."""
    atm = AL.isa(h)
    fp = fuselage_mesh_props()
    items = {}
    # fuselage (turbulent, pusher, hatches): wetted area from the OML mesh minus the wing saddle footprint
    saddle = pf["cr"] * 0.30
    S_wet_f = fp["S_wet"] - saddle
    d_eq = math.sqrt(4 * fp["A_max"] / math.pi)
    fin = L_FUS / d_eq
    Re_f = AL.reynolds(V, L_FUS, h)
    items["fuselage"] = AL.cf_flat(Re_f, 0.0) * AL.ff_body(fin) * 1.0 * S_wet_f
    # aft-body upsweep (Raymer eq. 12.36: D/q = 3.83 u^2.5 A_max, u in rad)
    u = math.atan2(float(z_bot(3.20 + AFT_STRETCH)[0] - z_bot(2.15 + AFT_STRETCH)[0]), 3.20 - 2.15)
    items["fuselage_upsweep"] = 3.83 * abs(u) ** 2.5 * fp["A_max"]
    # tail (V): aerolib wing formulas, NACA 0012, Q = 1.03 (Raymer 12.5.4 V-tail)
    S_exp_t = tail["S_V"] - 2 * 0.5 * tail["cr"] * 0.02
    Re_t = AL.reynolds(V, tail["mac"], h)
    items["tail"] = AL.cf_flat(Re_t, 0.0) * AL.ff_wing(0.12, 0.30, tail["sweep_mt_deg"], V / atm["a"]) * 1.03 * \
        AL.wing_wetted(S_exp_t, 0.12)
    # landing gear (Raymer 6th ed. Table 12.6 D/q per frontal area): faired main wheels 0.13, streamlined bow legs
    # 0.05, faired nose fork + wheel 0.35, streamlined nose strut 0.05; x 1.2 gear/fuselage interference
    A_wheel = WHEEL_D * WHEEL_W
    leg_main = gear["h_gear"] * 1.35 * 0.035          # each bow leg: slant length x 35 mm faired thickness
    leg_nose = gear["h_nose_leg"] * 0.040
    items["landing_gear"] = 1.2 * (2 * 0.13 * A_wheel + 2 * 0.05 * leg_main + 0.35 * A_wheel + 0.05 * leg_nose)
    # EO/IR turret: exposed ball (subcritical sphere CD 0.47, Hoerner) on 0.12 m of exposed height
    items["eo_ir_turret"] = 0.47 * TUR_D * 0.12
    # cooling: momentum loss of the cooling air (engine.yaml airflow 0.42 kg/s at 18 kW scaled with fuel flow, exit
    # velocity 0.5 V with the cowl flap), D = m_dot V (1 - 0.5)
    m_dot = EN["installation"]["cooling"]["cooling_airflow_estimate"]["mass_flow_kg_per_s_at_max_power"]["value"] * \
        fuel_flow_ratio
    items["engine_cooling"] = m_dot * V * 0.5 / (0.5 * atm["rho"] * V * V)
    # antennas, pitot, lights, vents, control-surface gaps (estimate)
    items["antennas_pitot_lights_vents"] = 0.0025
    items["control_surface_gaps"] = 0.0002 * pf["S"] * (3 if d.flaps else 2)   # dCD 0.0002 per sealed pair (estimate)
    base = sum(items.values())
    items["leakage_protuberance_5pct"] = 0.05 * base     # Raymer 12.5.7, well-sealed composite airframe (estimate)
    return items


def trimmed_polar(d: Design, ac_aero: dict, pf: dict, lay: dict, cd_rest: float, cg: tuple) -> tuple:
    """CD(CL) table with the tail trim load (Cm0, CG, thrust-line moment) and tail induced drag."""
    x_cg, z_cg = cg
    mac = pf["mac"]
    k_w = 1.0 / (math.pi * pf["AR"] * ac_aero["e_inv"])
    exp_frac = 1.0 - 0.5 * pf["cr"] * lay["w_fus"] / pf["S"]          # shoulder wing: saddle hides the lower side
    arm_t = lay["x_ac_t"] - x_cg
    b_t = lay["tail_b_proj"]
    CLs = np.linspace(-0.2, 1.75, 79)
    out = []
    cltS = []
    for CL in CLs:
        CLw = CL
        for _ in range(6):
            cdp = ac_aero["cdp"](min(CLw, 1.9)) * K_CD_TRIP * exp_frac
            cd_w = cdp + k_w * CLw ** 2
            cm = ac_aero["cm0"] + CLw * (x_cg - lay["x_ac_w"]) / mac - (cd_rest + cd_w) * (Z_T - z_cg) / mac
            clt = cm * mac / arm_t
            CLw = CL - clt
        cdi_t = (clt * pf["S"]) ** 2 / (math.pi * b_t ** 2 * 0.8) / pf["S"]
        out.append(cd_rest + cd_w + cdi_t)
        cltS.append(clt)
    return CLs, np.array(out), np.array(cltS)


def fit_polar(CLs, CDs, lo=0.25, hi=1.25):
    m = (CLs >= lo) & (CLs <= hi)
    k, cd0 = np.polyfit(CLs[m] ** 2, CDs[m], 1)
    return float(cd0), float(k)


# =====================================================================================================================
# 7. mass model (structural concept) and component placement
# =====================================================================================================================
def gust_limit(ws: float, cla: float, mac: float, clmax: float, VD: float) -> dict:
    out = {}
    for h in (0.0, H_CEIL_REQ):
        vn = ST.vn_diagram(ws, clmax, -0.9, N_POS, N_NEG, VC_EAS, VD, cla, mac, rho=AL.isa(h)["rho"])
        out[h] = vn
    worst = max(out.values(), key=lambda v: v["n_limit_pos"])
    return {"n_limit": worst["n_limit_pos"], "n_limit_neg": min(v["n_limit_neg"] for v in out.values()),
            "by_alt": {int(h): {"n_pos": v["n_limit_pos"], "n_neg": v["n_limit_neg"], "kg": v["kg"],
                                "gust_C": v["gust_C"], "gust_D": v["gust_D"], "VS": v["VS"], "VA": v["VA"]}
                       for h, v in out.items()}}


def wing_mass(d: Design, pf: dict, m0: float, n_lim: float, wsecs: list) -> dict:
    s = pf["b"] / 2
    y = np.linspace(0.0, s, 81)
    c = pf["cr"] + (pf["ct"] - pf["cr"]) * y / s
    tc = TC_ROOT * (1 + (d.ts_tip - 1) * y / s)
    l = ST.schrenk(y, c, s)
    w = FOS * n_lim * m0 * G / 2 * l                      # ultimate running load, one half (no inertia relief)
    Vs, Ms = ST.beam_loads(y, w)
    h_eff = np.maximum(0.95 * tc * c - 0.010, 0.012)      # cap centroid distance at the 30 % chord spar
    A_cap = np.maximum(Ms / (h_eff * SIG_CAP), 40e-6)
    caps = 2 * 2 * np.trapz(A_cap, y) * UD["density"] * 1.10               # 2 caps x 2 halves, ply drops/overlaps
    t_web = np.maximum(Vs / (h_eff * TAU_WEB), 0.6e-3)
    web = 2 * np.trapz(t_web * h_eff, y) * PW["density"] * 1.25            # incl. flanges and local doublers
    rear = 2 * np.trapz(0.6e-3 * 0.55 * tc * c + 2 * 20e-6 * UD["density"] / PW["density"], y) * PW["density"]
    n_ribs = int(math.ceil(s / 0.35)) + 1
    rib_area = 0.68 * np.interp(np.linspace(0, s, n_ribs), y, tc * c * c) * 0.6
    ribs = 2 * float(rib_area.sum()) * RIB_AREAL
    surf = oml.LiftingSurface(wsecs, n_chord=60).mesh()
    S_wet_half = surf.area() - 2 * 0.68 * TC_ROOT * pf["cr"] ** 2 * 0.5        # minus root cap (approx)
    skins = 2 * S_wet_half * SKIN_PRIMARY
    joints = 2 * 0.80          # main pins, bushings, root rib doublers, drag pins, tongue/fork (estimate)
    hinges = 2 * 0.10          # aileron hinge fittings, horns (estimate)
    sub = caps + web + rear + ribs + skins + joints + hinges
    total = sub * 1.05         # sealant, erosion tape, bonding/lightning strip, local reinforcements (estimate)
    return {"total": total, "caps": caps, "webs": web, "rear_spar": rear, "ribs": ribs, "skins": skins,
            "joints": joints, "hinges": hinges, "S_wet": 2 * S_wet_half, "M_root_ult_Nm": float(Ms[0]),
            "A_cap_root_mm2": float(A_cap[0] * 1e6), "h_eff_root_m": float(h_eff[0]), "n_ult": FOS * n_lim}


def tail_mass(tsecs: list, S_V: float) -> dict:
    surf = oml.LiftingSurface(tsecs, n_chord=40).mesh()
    S_wet = 2 * surf.area()
    skins = S_wet * SKIN_TAIL
    span = surface_mac(tsecs)["panel_span"]
    spar = 2 * span * (2 * 30e-6 * UD["density"] + 0.6e-3 * 0.05 * PW["density"])
    ribs = 2 * 5 * 0.004 * RIB_AREAL
    fittings = 2 * 0.35 + 2 * 0.10
    total = (skins + spar + ribs + fittings) * 1.05
    return {"total": total, "skins": skins, "spar": spar, "fittings_hinges": fittings, "S_wet": S_wet}


CHASSIS = {   # chassis items: (mass kg, x m) - estimates (process minimum gauges, machined fittings)
    "keel_beams_2x_cfrp_hat_0p20kg_per_m": (2 * 2.9 * 0.20, 1.75),
    "frames_10x_sandwich_bulkheads": (10 * 0.16, 1.70),
    "wing_attach_fittings_7075_ti_pins": (1.20, None),     # at the wing spar station
    "main_gear_saddle_clamps_7075": (0.60, None),          # at the main gear
    "nose_gear_trunnion_fitting": (0.30, X_NG),
    "engine_mount_frame_4130_truss": (0.60, "hub-0.30"),
    "parachute_hard_point_tray": (0.35, 0.85),
    "floors_trays_equipment_rails": (1.20, 1.10),
    "hatch_frames_quick_release_fasteners": (0.80, 1.40),
    "fuel_bay_liner_supports": (0.40, None),               # at the tank
    "tail_attach_fittings_fuselage_side": (0.30, "hub-0.30"),
    "wing_saddle_fairing": (0.30, None),
}


def fuselage_mass() -> dict:
    fp = fuselage_mesh_props()
    shell = fp["S_wet"] * SKIN_SECONDARY
    chassis = sum(m for m, _ in CHASSIS.values())
    return {"shell": shell, "chassis": chassis, "total": (shell + chassis) * 1.03, "S_wet": fp["S_wet"]}


def gear_mass(m0: float) -> float:
    return GEAR_WHEELS + GEAR_LEGS_145 * m0 / 145.0 + GEAR_FAIRINGS


# =====================================================================================================================
# 8. layout, CG, stability, gear placement (one consistent pass for a given wing/tail size)
# =====================================================================================================================
def loading_cases():
    return [
        {"name": "mtow_full_fuel_design_payload", "fuel": 1.0, "payload": "design"},
        {"name": "full_fuel_baseline_sensors_only", "fuel": 1.0, "payload": "baseline_set"},
        {"name": "zero_fuel_design_payload", "fuel": 0.0, "payload": "design"},
        {"name": "reserve_fuel_design_payload", "fuel": 0.12, "payload": "design"},
        {"name": "minimum_flying", "fuel": 0.10, "payload": "turret_only"},
    ]


def payload_items(kind: str, x_bay: float) -> list:
    tur = [("payload_eo_ir_turret", M_TURRET, X_TURRET, -0.24)]
    base = tur + [("payload_mission_computer", M_MCOMP, x_bay - 0.12, 0.08), ("payload_tray", M_TRAY, x_bay, -0.15)]
    if kind == "turret_only":
        return tur
    if kind == "baseline_set":
        return base
    return base + [("payload_research_allowance", M_RESEARCH, x_bay, -0.08)]


def component_list(d, pf, wsecs, tail, masses, gearpos, tank, fuel_kg):
    """(name, mass, x, z) of every item at MTOW, design payload excluded (added per case)."""
    wm = surface_mac(wsecs)
    xw = wm["x_le_mac"] + 0.42 * wm["mac"]
    zw = Z_WING_ROOT + 0.03
    x_spar = wsecs[0]["x_le"] + 0.30 * pf["cr"]
    x_ail = wm["x_le_mac"] + 0.80 * wm["mac"]
    tm = tail["macd"]
    items = [
        ("wing_structure", masses["wing"], xw, zw),
        ("tail_structure", masses["tail"], tm["x_le_mac"] + 0.40 * tm["mac"], tm["z_mac"]),
        ("fuselage_shell", masses["fus_shell"], fuselage_mesh_props()["x_centroid"], 0.02),
        ("landing_gear_main", masses["gear"] - 0.365 - 0.45 - 0.08 - 0.9, gearpos["x_mg"], gearpos["z_g"] + 0.15),
        ("landing_gear_nose", 0.365 + 0.45 + 0.08 + 0.9, X_NG, gearpos["z_g"] + 0.17),
        ("engine_group", FIXED_ITEMS["engine_group_installed"][0], X_HUB - 0.12, Z_T - 0.05),
        ("propeller", FIXED_ITEMS["propeller"][0], X_PROP, Z_T),
        ("spinner_hub_spacer", FIXED_ITEMS["spinner_hub_adapter_and_80mm_spacer"][0], X_PROP - 0.01, Z_T),
        ("baffles_ducts_firewall", FIXED_ITEMS["baffles_ducts_firewall_cowl_flap"][0], X_HUB - 0.15, Z_T - 0.05),
        ("fuel_system", FIXED_ITEMS["fuel_system_bladder_55L"][0], tank["x_c"], -0.05),
        ("actuators_wing_2", 2 * 0.27 + 0.15, x_ail, zw),
        ("actuators_tail_2", 2 * 0.27 + 0.05, tm["x_le_mac"] + 0.6 * tm["mac"], tm["z_mac"]),
        ("actuators_steer_brake_cowlflap", 2 * 0.27 + 0.10 + 0.15, (X_NG + gearpos["x_mg"]) / 2, -0.10),
        ("avionics", FIXED_ITEMS["avionics"][0], X_AVION, 0.05),
        ("electrical_battery_pdu", FIXED_ITEMS["electrical_power"][0], X_BATT, 0.00),
        ("wiring_harness", FIXED_ITEMS["wiring_harness"][0], 1.65, 0.05),
        ("parachute_fts", 5.9 + 0.15, sum(X_CHUTE) / 2, 0.15),
        ("lights", 0.25, xw - 0.1, zw),
    ]
    if d.flaps:
        items.append(("flap_kit", FLAP_KIT, wm["x_le_mac"] + 0.85 * pf["cr"], zw))
    if PROP_KEY != "0161":
        items.append(("propeller_delta_3B", PROP_MASS_DELTA, X_PROP, Z_T))
    for name, (m, x) in CHASSIS.items():
        if isinstance(x, str):
            x = X_HUB + float(x[3:])
        if x is None:
            x = {"wing_attach_fittings_7075_ti_pins": x_spar, "main_gear_saddle_clamps_7075": gearpos["x_mg"],
                 "fuel_bay_liner_supports": tank["x_c"], "wing_saddle_fairing": x_spar + 0.1}[name]
        items.append(("chassis_" + name, m * 1.03, x, 0.0))
    empty_est = sum(m for _, m, _, _ in items)
    xe = sum(m * x for _, m, x, _ in items) / empty_est
    ze = sum(m * z for _, m, _, z in items) / empty_est
    items.append(("mass_growth_allowance", GROWTH * empty_est, xe, ze))
    return items


def cg_of(items, fuel_kg, tank, payload_kind, fuel_fraction):
    its = list(items) + [(n, m, x, z) for n, m, x, z in payload_items(payload_kind, sum(tank["bay"]) / 2)]
    its.append(("fuel", fuel_kg * fuel_fraction, tank["x_c"], tank["z_c"]))
    m = sum(i[1] for i in its)
    return m, sum(i[1] * i[2] for i in its) / m, sum(i[1] * i[3] for i in its) / m


def tank_layout(fuel_kg: float, x_center: float) -> dict:
    """Payload bay centred at ``x_center`` with a fuel cell ahead and behind (interconnected ATL bladders feeding a
    common header): each cell holds half the fuel, so the fuel CG stays at the bay centroid."""
    vol_req = fuel_kg / RHO_FUEL * 1.02
    xb0, xb1 = x_center - BAY_LEN / 2 - 0.015, x_center + BAY_LEN / 2 + 0.015    # bay + frame thickness

    def cell(x_start, sign):
        L = 0.05
        while True:
            xs = np.linspace(x_start, x_start + sign * L, 30)
            A = np.array([internal_area(x) for x in xs])
            v = K_TANK * abs(np.trapz(A, xs))
            if v >= vol_req / 2 or L > 1.5:
                return {"x0": min(x_start, x_start + sign * L), "x1": max(x_start, x_start + sign * L), "length": L,
                        "volume_m3": v, "x_c": float(np.trapz(A * xs, xs) / np.trapz(A, xs))}
            L += 0.005
    fwd, aft = cell(xb0, -1.0), cell(xb1, +1.0)
    xc = (fwd["volume_m3"] * fwd["x_c"] + aft["volume_m3"] * aft["x_c"]) / (fwd["volume_m3"] + aft["volume_m3"])
    return {"x_c": xc, "z_c": -0.01, "cells": {"forward": fwd, "aft": aft}, "bay": (xb0 + 0.015, xb1 - 0.015),
            "x0": fwd["x0"], "x1": aft["x1"], "volume_required_m3": vol_req,
            "volume_available_m3": fwd["volume_m3"] + aft["volume_m3"]}


THETA_TD_TARGET = rec("theta_touchdown_target_deg", 3.0, "", basis="main wheels touch first (nose wheel clear) at "
                      "1.15 VS; caps the wing incidence (no flaps, see trades.flaps)")
GEAR_TRAVEL = rec("fuselage_bottom_static_height_min_m", 0.20, "baseline.yaml#subsystems.landing_gear.stroke_requirement",
                  basis=">= 0.18-0.22 m effective deflection of the spring bow: about 0.15 m travel beyond static + 0.05 m")


def ground_geometry(d, pf, aero, cg_cases, w_fus) -> dict:
    """Flap settings for the take-off/touch-down attitudes, main gear station (tip-back), nose-gear load split, gear
    height from the clearance rules (propeller, turret, gear travel) and track (turnover)."""
    xs = [c["x"] for c in cg_cases]
    zs = [c["z"] for c in cg_cases]
    x_aft, x_fwd = max(xs), min(xs)
    z_cg = max(zs)
    CLa = aero["CL_alpha"]
    clmax = aero["CLmax_wing"]
    th_lof = max(math.degrees((clmax / 1.1 ** 2 - aero["CL_0"]) / CLa), 0.0)       # lift-off at 1.1 VS
    th_td = max(math.degrees((clmax / 1.15 ** 2 - aero["CL_0"]) / CLa), 0.0)       # touch-down at 1.15 VS
    th_flare = max(th_td, 4.0)
    th_design = th_flare + 3.0
    z_tur_bot = float(z_bot(X_TURRET)[0]) - 0.03 - TUR_H_GROWTH * 0.45
    R = D_PROP / 2
    z_g = -0.40
    for _ in range(40):
        h_cg = z_cg - z_g
        tb = max(15.0, th_design + 2.0)
        x_mg = x_aft + h_cg * math.tan(math.radians(tb))
        arm = X_PROP - x_mg
        req = [
            Z_T - R - CLEAR_PROP_GROUND,                                                        # level, 0.18 m
            Z_T - R - (CLEAR_PROP_GROUND + arm * math.sin(math.radians(th_lof))) / math.cos(math.radians(th_lof)),
            Z_T - R - (0.05 + arm * math.sin(math.radians(th_design))) / math.cos(math.radians(th_design)),
            z_tur_bot - 0.12,                                                                    # turret (growth)
            float(z_bot(x_mg)[0]) - GEAR_TRAVEL,                                                 # spring-bow travel
        ]
        z_new = min(req)
        if abs(z_new - z_g) < 1e-6:
            break
        z_g = z_new
    active = ["prop_level", "prop_liftoff", "prop_flare_design", "turret", "gear_travel"][int(np.argmin(req))]
    h_cg = z_cg - z_g
    x_bump = X_HUB - 0.22
    z_bump = float(z_bot(x_bump)[0]) - 0.03
    th_bump = math.degrees(math.atan2(z_bump - z_g, x_bump - x_mg))
    th_prop = math.degrees(math.atan2(Z_T - R - z_g, X_PROP - x_mg))
    clear_at = lambda th: (Z_T - R - z_g) * math.cos(math.radians(th)) - (X_PROP - x_mg) * math.sin(math.radians(th))
    nose_aft = (x_mg - x_aft) / (x_mg - X_NG)
    nose_fwd = (x_mg - x_fwd) / (x_mg - X_NG)
    ln = x_aft - X_NG
    track = 0.6
    while True:
        delta = math.atan((track / 2) / (x_mg - X_NG))
        psi = math.degrees(math.atan(h_cg / (ln * math.sin(delta))))
        if psi <= 55.0 or track > 2.0:
            break
        track += 0.01
    h_gear = float(z_bot(x_mg)[0]) - z_g - WHEEL_D / 2
    return {"x_mg": x_mg, "z_g": z_g, "h_cg": h_cg, "tipback_deg": tb, "theta_lof_deg": th_lof, "theta_td_deg": th_td,
            "theta_flare_deg": th_flare, "theta_design_deg": th_design, "track": track, "turnover_deg": psi,
            "nose_load_aft_cg": nose_aft, "nose_load_fwd_cg": nose_fwd, "prop_clear_level": Z_T - R - z_g,
            "prop_clear_lof": clear_at(th_lof), "prop_clear_design": clear_at(th_design),
            "prop_strike_deg": th_prop, "bumper_contact_deg": th_bump, "x_bumper": x_bump, "z_bumper": z_bump,
            "h_gear": max(h_gear, 0.05), "h_nose_leg": float(z_bot(X_NG)[0]) - z_g - WHEEL_D / 2,
            "turret_clear": z_tur_bot - z_g, "wheelbase": x_mg - X_NG, "active_height_rule": active,
            "fuselage_bottom_height_at_mg": float(z_bot(x_mg)[0]) - z_g,
            "clmax_to_wing": clmax, "clmax_ld_wing": clmax, "dCL0_to": 0.0, "dCL0_ld": 0.0}


# =====================================================================================================================
# 9. the configuration solve (geometry <-> CG <-> stability <-> gear <-> masses <-> aero)
# =====================================================================================================================
TAIL_SLOPE_OVERRIDE = None    # final design: lifting-line value from aero.surface_analysis on the V-tail sections


def tail_slope(d: Design, S_V: float, sweep_le: float, h: float, V: float) -> float:
    """Panel-pair lift slope (Helmbold, aerolib) with the NACA 0012 section slope at the tail Reynolds number."""
    span2 = math.sqrt(d.tail_AR * S_V)
    cr = S_V / (span2 / 2 * (1 + d.tail_taper))
    ch = AE.characteristics("n0012", AL.reynolds(V, 0.8 * cr, h))
    ct_ = d.tail_taper * cr
    lam = ct_ / cr
    tan_half = math.tan(math.radians(sweep_le)) - 4 * 0.5 / d.tail_AR * (1 - lam) / (1 + lam)
    return AL.lift_slope(d.tail_AR, math.degrees(math.atan(tan_half)), cla_2d=ch["cl_alpha"])


def solve_configuration(d: Design, m0: float, fuel_kg_guess: float, V_ref: float, h_ref: float = H_LOITER,
                        a_t_override: float | None = None, verbose: bool = False) -> Aircraft:
    S = m0 * G / d.ws
    pf = wing_planform(d, S)
    fp = fuselage_mesh_props()
    w_fus = fp["w_max"]
    # wing incidence: fuselage level at the mid-loiter CL (0.85 MTOM at the 26 m/s EAS floor)
    secs0 = wing_sections(d, pf, 1.4, 0.0)
    a0 = build_aero(d, pf, secs0, V_ref, h_ref, 0.0)
    q_floor = 0.5 * RHO0 * V_LOIT_MIN_EAS ** 2
    CL_mid = 0.85 * m0 * G / (q_floor * S)
    i_level = math.degrees(a0["sa"]["alpha_for"](min(CL_mid, 1.1)))
    i_td = math.degrees((a0["CLmax_wing"] / 1.15 ** 2 - a0["CL_0"]) / a0["CL_alpha"]) - THETA_TD_TARGET
    i_w = math.floor(min(i_level, i_td) * 4) / 4
    x_w = 1.35
    S_V, gam = 0.9, 40.0
    gearpos = {"x_mg": 1.95, "z_g": -0.45}
    tank = tank_layout(fuel_kg_guess, 1.6)
    aero = None
    for outer in range(12):
        wsecs = wing_sections(d, pf, x_w, i_w)
        if aero is None:
            aero = build_aero(d, pf, wsecs, V_ref, h_ref, i_w)
        wm = surface_mac(wsecs)
        x_ac_w = wm["x_le_mac"] + 0.25 * wm["mac"]
        # ---- tail sizing at the current wing station
        x_te_root = X_PROP - 0.15                               # 0.15 m axial gap tail root TE -> prop plane
        for _ in range(30):
            tsecs = tail_sections(d, S_V, gam, x_te_root)
            tmac = surface_mac(tsecs)
            x_ac_t = tmac["x_le_mac"] + 0.25 * tmac["mac"]
            l_h = x_ac_t - x_ac_w
            a_t = a_t_override or TAIL_SLOPE_OVERRIDE or tail_slope(d, S_V, d.tail_le_sweep, h_ref, V_ref)
            S_h = d.VH * S * pf["mac"] / l_h
            cnb_fus = min(-1.3 * fp["volume"] / (S * pf["b"]) * (fp["h_max"] / fp["w_max"]),
                          SL.cn_beta_fuselage(0.0015, 1.75, fp["S_side"], L_FUS, S, pf["b"]))
            S_v_req = (d.cnb_req - cnb_fus) * S * pf["b"] / (0.9 * a_t * l_h * 1.1)
            S_v = max(S_v_req, d.VV_min * S * pf["b"] / l_h)
            S_new = S_h + S_v
            gam_new = math.degrees(math.atan(math.sqrt(S_v / S_h)))
            if abs(S_new - S_V) < 1e-5 and abs(gam_new - gam) < 1e-4:
                break
            S_V, gam = S_new, gam_new
        sweep_mt = math.degrees(math.atan(math.tan(math.radians(d.tail_le_sweep)) -
                                          (tsecs[0]["chord"] - tsecs[1]["chord"]) * 0.3 / tmac["panel_span"]))
        tail = {"secs": tsecs, "S_V": S_V, "gamma_deg": gam, "S_h_eff": S_h, "S_v_eff": S_v, "macd": tmac,
                "mac": tmac["mac"], "cr": tsecs[0]["chord"], "ct": tsecs[1]["chord"], "x_ac": x_ac_t, "a_t": a_t,
                "sweep_mt_deg": sweep_mt, "cnb_fus": cnb_fus,
                "b_proj": 2 * (tsecs[1]["y"]), "span_panel": tmac["panel_span"]}
        # ---- masses at m0
        gl = gust_limit(d.ws, aero["CL_alpha"], pf["mac"], aero["CLmax_wing"], 57.0)
        wmass = wing_mass(d, pf, m0, max(gl["n_limit"], N_POS), wsecs)
        tmass = tail_mass(tsecs, S_V)
        fmass = fuselage_mass()
        masses = {"wing": wmass["total"], "tail": tmass["total"], "fus_shell": fmass["shell"] * 1.03,
                  "gear": gear_mass(m0), "chassis": fmass["chassis"] * 1.03}
        items = component_list(d, pf, wsecs, tail, masses, gearpos, tank, fuel_kg_guess)
        # ---- CG for the loading cases
        cases = []
        for c in loading_cases():
            m, x, z = cg_of(items, fuel_kg_guess, tank, c["payload"], c["fuel"])
            cases.append({"name": c["name"], "m": m, "x": x, "z": z})
        # ---- neutral point
        deps = SL.downwash_gradient(d.AR, d.taper, l_h, tmac["z_mac"] - Z_WING_ROOT, pf["b"])
        kf = SL.kf_fuselage((wsecs[0]["x_le"] + 0.25 * pf["cr"]) / L_FUS)
        cmaf = SL.cm_alpha_fuselage(kf, w_fus, L_FUS, pf["mac"], S)
        x_np = SL.neutral_point(aero["CL_alpha"], x_ac_w, a_t, S_h, x_ac_t, S, pf["mac"], deps, 0.9, cmaf)
        sms = [SL.static_margin(x_np, c["x"], pf["mac"]) for c in cases]
        # ---- move the wing so that min SM = target (CG moves with the wing mass share)
        err = min(sms) - d.sm_min
        mw_share = (masses["wing"] + 2.6) / cases[0]["m"]       # wing + wing-mounted items share
        dx = -err * pf["mac"] / (1.0 - mw_share)
        # ---- gear and tank follow the CG
        gearpos = ground_geometry(d, pf, aero, cases, w_fus)
        x_design = cases[0]["x"]
        tank = tank_layout(fuel_kg_guess, x_design if outer < 8 else tank["x_c"])
        if verbose:
            print(f"  layout it{outer}: x_w={x_w:.3f} SMmin={min(sms):.3f} x_np={x_np:.3f} S_V={S_V:.3f} "
                  f"gam={gam:.1f} x_mg={gearpos['x_mg']:.3f} z_g={gearpos['z_g']:.3f}")
        if abs(dx) < 2e-4 and outer > 3:
            break
        x_w += dx
    lay = {"x_w": x_w, "i_w": i_w, "x_ac_w": x_ac_w, "x_ac_t": x_ac_t, "l_h": l_h, "w_fus": w_fus,
           "tail_b_proj": tail["b_proj"], "x_np": x_np, "deps_da": deps, "cm_alpha_fus": cmaf, "kf": kf,
           "cases": cases, "sms": sms, "gear": gearpos, "tank": tank, "items": items,
           "incidence_rule": {"i_level_loiter_deg": i_level, "i_touchdown_deg": i_td},
           "tail": tail, "gust": gl, "wing_mac": wm}
    return Aircraft(d=d, m0=m0, pf=pf, wing_secs=wsecs, tail_secs=tsecs, sa=aero["sa"], S=S, cd_table=None, fit=None,
                    clmax=None, geo={}, drag_items={}, stab={}, mass={"wing": wmass, "tail": tmass, "fus": fmass,
                                                                     "groups": masses}, layout=lay), aero


def finish_aero(ac: Aircraft, aero: dict, V_ref: float, h_ref: float, fuel_flow_ratio: float):
    d, pf, lay = ac.d, ac.pf, ac.layout
    items = drag_buildup(d, pf, lay["tail"], lay["gear"], V_ref, h_ref, fuel_flow_ratio)
    cd_rest = sum(items.values()) / pf["S"]
    cg_d = (lay["cases"][0]["x"], lay["cases"][0]["z"])
    CLs, CDs, clt = trimmed_polar(d, aero, pf, lay, cd_rest, cg_d)
    cd0, k = fit_polar(CLs, CDs)
    e = 1.0 / (math.pi * pf["AR"] * k)
    ld = CLs / CDs
    i = int(np.argmax(ld))
    end_par = np.clip(CLs, 0.0, None) ** 1.5 / CDs
    j = int(np.argmax(np.where(CLs > 0, end_par, 0)))
    # trimmed CLmax: wing CLmax + tail load at the most forward CG (flaps: Cm0 - 0.25 dCL0)
    x_fwd = min(c["x"] for c in lay["cases"])
    arm = lay["x_ac_t"] - x_fwd

    def trim(clw, dcm):
        cm = aero["cm0"] + dcm + clw * (x_fwd - lay["x_ac_w"]) / pf["mac"]
        return clw + cm * pf["mac"] / arm
    clmax = {"wing_clean": aero["CLmax_wing"], "clean_trimmed": trim(aero["CLmax_wing"], 0.0)}
    clmax["to_trimmed"] = clmax["ld_trimmed"] = clmax["clean_trimmed"]          # landing flaps-up
    fe = flap_effect(d, pf, lay["w_fus"], d.flap_to_deg) if d.flaps else None
    if fe:
        dcl0 = aero["CL_alpha"] * math.radians(-fe["dalpha0_deg"])
        clmax["to_trimmed"] = trim(aero["CLmax_wing"] + fe["dCLmax"], -0.25 * dcl0)
    ac.flap_to = fe
    ac.cd_table = (CLs, CDs)
    ac.fit = {"cd0": cd0, "k": k, "e": e, "LD_max": float(ld[i]), "CL_LDmax": float(CLs[i]),
              "CL_endurance": float(CLs[j]), "endurance_param_max": float(end_par[j]), "cd_rest": cd_rest,
              "cd0_to": cd0 + (fe["dCD0"] if fe else 0.0), "cd0_ld": cd0,
              "e_nita_scholz": AL.oswald_nita_scholz(pf["AR"], pf["taper"], 0.0, lay["w_fus"] / pf["b"]),
              "e_raymer": AL.oswald_straight(pf["AR"]), "cl_tail_S": (CLs, clt)}
    ac.clmax = clmax
    ac.drag_items = items
    ac.aero_cm0 = aero["cm0"]
    return ac


# =====================================================================================================================
# 10. flight mechanics: level point, climb, mission
# =====================================================================================================================
def vstall(ac: Aircraft, W: float, h: float, clmax: float | None = None) -> float:
    return AL.stall_speed(W, ac.S, clmax or ac.clmax["clean_trimmed"], AL.isa(h)["rho"])


def level_point(ac: Aircraft, W: float, V: float, h: float) -> dict | None:
    atm = AL.isa(h)
    q = 0.5 * atm["rho"] * V * V
    CL = W / (q * ac.S)
    if CL > ac.cd_table[0][-1]:
        return None
    CD = float(ac.CD(CL))
    D = q * ac.S * CD
    p = PROP.cruise(D, V, h)
    if not p["ok"]:
        return None
    P_tot = p["P_shaft"] + P_GEN_SHAFT
    b = bsfc_g_kwh(P_tot)
    ff = b * P_tot / 3.6e9
    return {"V": V, "h": h, "CL": CL, "CD": CD, "LD": CL / CD, "D": D, "P_shaft": p["P_shaft"], "P_total": P_tot,
            "eta": p["eta"], "rpm": p["rpm"], "tip_mach": p["tip_mach"], "power_fraction": P_tot / P_MAX,
            "bsfc_g_kWh": b, "ff_kg_s": ff, "ff_kg_h": ff * 3600, "gen_W": GEN_W_7500 * p["rpm"] / 7500.0,
            "bsfc_eff_kg_J": b / 3.6e9 * P_tot / p["P_shaft"], "EAS": V * math.sqrt(atm["sigma"])}


def v_floor(ac: Aircraft, W: float, h: float) -> float:
    sig = AL.isa(h)["sigma"]
    return max(V_LOIT_MIN_EAS / math.sqrt(sig), 1.2 * vstall(ac, W, h))


def golden(f, a, b, n=28):
    gr = (math.sqrt(5) - 1) / 2
    c, d_ = b - gr * (b - a), a + gr * (b - a)
    fc, fd = f(c), f(d_)
    for _ in range(n):
        if fc < fd:
            b, d_, fd = d_, c, fc
            c = b - gr * (b - a)
            fc = f(c)
        else:
            a, c, fc = c, d_, fd
            d_ = a + gr * (b - a)
            fd = f(d_)
    return 0.5 * (a + b)


def best_loiter(ac, W, h, floor=True):
    vlo = v_floor(ac, W, h) if floor else 1.2 * vstall(ac, W, h)
    f = lambda V: (level_point(ac, W, V, h) or {"ff_kg_s": 1e9})["ff_kg_s"]
    V = golden(f, vlo, vlo + 20.0)
    V = max(V, vlo)
    return level_point(ac, W, V, h)


def best_range(ac, W, h):
    vlo = max(1.2 * vstall(ac, W, h), V_CRUISE_MIN)
    f = lambda V: (lambda p: p["ff_kg_s"] / V if p else 1e9)(level_point(ac, W, V, h))
    V = golden(f, vlo, vlo + 30.0)
    return level_point(ac, W, max(V, vlo), h)


def climb_point(ac, W, h) -> dict:
    """Max rate of climb at WOT (prop model, trimmed polar), scanned over speed."""
    atm = AL.isa(h)
    best = None
    vs = vstall(ac, W, h)
    for V in np.linspace(1.15 * vs, 2.4 * vs, 40):
        q = 0.5 * atm["rho"] * V * V
        CL = W / (q * ac.S)
        D = q * ac.S * float(ac.CD(CL))
        w = PROP.wot(V, h)
        roc = (w["T"] - D) * V / W
        if best is None or roc > best["roc"]:
            P_tot = w["P_shaft"] + P_GEN_SHAFT
            b = bsfc_g_kwh(P_tot)
            best = {"V": V, "roc": roc, "CL": CL, "LD": CL / float(ac.CD(CL)), "eta": w["eta"], "rpm": w["rpm"],
                    "P_shaft": w["P_shaft"], "bsfc_eff_kg_J": b / 3.6e9 * P_tot / w["P_shaft"], "T": w["T"],
                    "tip_mach": w["tip_mach"], "ff_kg_h": b * P_tot / 3.6e6}
    return best


def fly(ac: Aircraft, m0: float, t_loiter_s: float, R_transit: float = R_TRANSIT_M, h: float = H_LOITER,
        ferry: bool = False, chunk_loiter_s: float = 3600.0, chunk_cruise_m: float = 50e3) -> dict:
    """Builds the sizinglib segment list by marching the weight (each segment evaluated at its start weight)."""
    segs, log = [], []
    W = m0 * G
    t_air = 0.0

    def add(seg, dt, extra=None):
        nonlocal W, t_air
        f = SZ.segment_fraction(seg, 1.0)
        segs.append(seg)
        log.append({"kind": seg.kind, "name": seg.name, "dt_s": dt, "W_start_N": W, "fraction": f, **(extra or {})})
        W *= f
        t_air += dt if seg.kind not in ("warmup", "taxi", "reserve") else 0.0

    for k in ("warmup", "taxi", "takeoff"):
        add(SZ.Segment(k, name=k), 0.0)
    cp = climb_point(ac, W, h / 2)
    add(SZ.Segment("climb", value=h, V=cp["V"], LD=cp["LD"], eta=cp["eta"], bsfc=cp["bsfc_eff_kg_J"],
                   gamma=cp["roc"] / cp["V"], name="climb"), h / cp["roc"], {"V": cp["V"], "roc": cp["roc"]})

    def cruise(dist):
        rem = dist
        while rem > 1.0:
            dx = min(chunk_cruise_m, rem)
            p = best_range(ac, W, h)
            add(SZ.Segment("cruise", value=dx, V=p["V"], LD=p["LD"], eta=p["eta"], bsfc=p["bsfc_eff_kg_J"],
                           name="cruise"), dx / p["V"], {"V": p["V"], "ff_kg_h": p["ff_kg_h"], "rpm": p["rpm"]})
            rem -= dx
    cruise(R_transit)
    rem = t_loiter_s
    while rem > 1.0:
        dt = min(chunk_loiter_s, rem)
        p = best_loiter(ac, W, h)
        add(SZ.Segment("loiter", value=dt, V=p["V"], LD=p["LD"], eta=p["eta"], bsfc=p["bsfc_eff_kg_J"],
                       name="loiter"), dt, {"V": p["V"], "ff_kg_h": p["ff_kg_h"], "rpm": p["rpm"], "CL": p["CL"],
                                            "pf": p["power_fraction"], "gen_W": p["gen_W"]})
        rem -= dt
    if not ferry:
        cruise(R_transit)
    t_desc = h / 2.5
    add(SZ.Segment("descent", name="descent"), t_desc)
    add(SZ.Segment("landing", name="landing"), 0.0)
    p = best_loiter(ac, W, 1000.0)
    add(SZ.Segment("reserve", value=RESERVE_FRAC * t_air, V=p["V"], LD=p["LD"], eta=p["eta"], bsfc=p["bsfc_eff_kg_J"],
                   name="reserve"), RESERVE_FRAC * t_air)
    ff, fr = SZ.mission_fuel_fraction(segs, trapped=TRAPPED)
    return {"segments": segs, "log": log, "ff": ff, "fractions": fr, "t_air_s": t_air, "W_end": W}


def solve_loiter_for_fuel(ac, m0, fuel_kg, **kw) -> dict:
    lo, hi = 0.0, 40 * 3600.0
    if fly(ac, m0, 0.0, **kw)["ff"] * m0 > fuel_kg:
        return {"feasible": False, **fly(ac, m0, 0.0, **kw)}
    for _ in range(26):
        mid = 0.5 * (lo + hi)
        if fly(ac, m0, mid, **kw)["ff"] * m0 > fuel_kg:
            hi = mid
        else:
            lo = mid
    r = fly(ac, m0, lo, **kw)
    r["t_loiter_s"] = lo
    r["feasible"] = True
    return r


def solve_loiter_for_endurance(ac, m0, endurance_s, **kw) -> dict:
    base = fly(ac, m0, 0.0, **kw)
    t = max(endurance_s - base["t_air_s"], 0.0)
    r = fly(ac, m0, t, **kw)
    r["t_loiter_s"] = t
    return r


def solve_range(ac, m0, fuel_kg, h=H_LOITER) -> dict:
    lo, hi = 0.0, 4000e3
    for _ in range(24):
        mid = 0.5 * (lo + hi)
        r = fly(ac, m0, 0.0, R_transit=mid, h=h, ferry=True, chunk_cruise_m=200e3)
        if r["ff"] * m0 > fuel_kg:
            hi = mid
        else:
            lo = mid
    return {"range_m": lo, **fly(ac, m0, 0.0, R_transit=lo, h=h, ferry=True, chunk_cruise_m=200e3)}


# =====================================================================================================================
# 11. evaluation of one design point (fixed MTOM: fuel = MTOM - payload - empty) and the closed MassModel loop
# =====================================================================================================================
def v_ref_loiter() -> float:
    return V_LOIT_MIN_EAS / math.sqrt(AL.isa(H_LOITER)["sigma"])


def empty_breakdown(ac: Aircraft) -> dict:
    g = ac.mass["groups"]
    airframe = g["wing"] + g["tail"] + g["fus_shell"] + g["chassis"] + g["gear"]
    items_sum = sum(m for n, m, _, _ in ac.layout["items"] if n != "mass_growth_allowance")
    mf = fixed_mass(ac.d)
    return {"fixed": mf, "airframe": airframe, "growth": GROWTH * (mf + airframe),
            "empty": (mf + airframe) * (1 + GROWTH), "items_check": items_sum}


def airframe_fn_factory(ac: Aircraft, aero: dict):
    """m0 -> airframe mass fraction (incl. growth allowance) at constant W/S, AR, layout (for sizinglib.MassModel)."""
    d = ac.d
    g = ac.mass["groups"]
    S0 = ac.S
    n_lim = max(ac.layout["gust"]["n_limit"], N_POS)

    @functools.lru_cache(maxsize=256)
    def frac(m0_r: float) -> float:
        m0 = float(m0_r)
        S = m0 * G / d.ws
        pf = wing_planform(d, S)
        wsecs = wing_sections(d, pf, ac.layout["x_w"], ac.layout["i_w"])
        wing = wing_mass(d, pf, m0, n_lim, wsecs)["total"]
        tail = g["tail"] * (S / S0) ** 1.5
        af = wing + tail + g["fus_shell"] + g["chassis"] + gear_mass(m0)
        return af * (1 + GROWTH) / m0
    return lambda m0: frac(round(m0, 4))


def evaluate(d: Design, stretch: float, mode: str = "mtom", endurance_target_h: float | None = None,
             extra: dict | None = None, verbose: bool = False) -> dict:
    """mode 'mtom': MTOM fixed (design), loiter time from the fuel left; mode 'endurance': loiter time from the
    endurance target, MTOW from the MassModel loop (geometry re-sized at each MTOW)."""
    extra = extra or {}
    set_fuselage(stretch)
    V_ref = v_ref_loiter()
    m0 = d.mtom
    fuel_guess, ffr = 37.0, 0.33
    for it in range(8):
        ac, aero = solve_configuration(d, m0, fuel_guess, V_ref)
        finish_aero(ac, aero, V_ref, H_LOITER, ffr)
        if extra.get("dDq"):
            ac.drag_items["trade_delta"] = extra["dDq"]
            finish_aero_with_items(ac, aero)
        eb = empty_breakdown(ac)
        empty = eb["empty"] + extra.get("dm", 0.0) * (1 + GROWTH)
        if mode == "mtom":
            fuel = m0 - PAYLOAD - empty
            mis = solve_loiter_for_fuel(ac, m0, fuel)
            m_new = m0
        else:
            mis = solve_loiter_for_endurance(ac, m0, endurance_target_h * 3600.0)
            mm = SZ.MassModel(PAYLOAD, (fixed_mass(d) + extra.get("dm", 0.0)) * (1 + GROWTH), mis["ff"],
                              airframe_fn_factory(ac, aero))
            sol = mm.solve(m0_guess=m0)
            m_new, fuel = sol["mtow"], sol["fuel"]
        lo = [l for l in mis["log"] if l["kind"] == "loiter"]
        ff_loiter = np.mean([l["ff_kg_h"] for l in lo]) if lo else 2.5
        ffr_new = ff_loiter / (BSFC_PTS[-1, 1] * P_MAX / 1e6)
        done = abs(fuel - fuel_guess) < 0.05 and abs(ffr_new - ffr) < 0.01 and abs(m_new - m0) < 0.05
        if verbose:
            print(f"   eval it{it}: m0 {m0:.2f} fuel {fuel:.2f} ff {mis['ff']:.4f} t_air {mis['t_air_s']/3600:.2f} h")
        fuel_guess, ffr = fuel, ffr_new
        if mode != "mtom":
            m0 = m_new
        if done:
            break
    return {"ac": ac, "aero": aero, "mission": mis, "fuel": fuel, "m0": m0, "empty": empty, "eb": eb,
            "endurance_h": mis["t_air_s"] / 3600.0, "stretch": stretch}


def finish_aero_with_items(ac: Aircraft, aero: dict):
    """Re-runs the trimmed polar with the current drag_items (used by the trades)."""
    lay = ac.layout
    cd_rest = sum(ac.drag_items.values()) / ac.pf["S"]
    CLs, CDs, clt = trimmed_polar(ac.d, aero, ac.pf, lay, cd_rest, (lay["cases"][0]["x"], lay["cases"][0]["z"]))
    cd0, k = fit_polar(CLs, CDs)
    ac.cd_table = (CLs, CDs)
    ac.fit.update({"cd0": cd0, "k": k, "e": 1 / (math.pi * ac.pf["AR"] * k), "LD_max": float(np.max(CLs / CDs)),
                   "cd_rest": cd_rest})


# =====================================================================================================================
# 12. performance of the final design
# =====================================================================================================================
def k_ground(ac: Aircraft) -> float:
    """Raymer 6th ed. eq. 12.61-style ground effect on induced drag: k_eff = k (16h/b)^2 / (1 + (16h/b)^2)."""
    h = Z_WING_ROOT - ac.layout["gear"]["z_g"]
    r = (16 * h / ac.pf["b"]) ** 2
    return ac.fit["k"] * r / (1 + r)


def takeoff(ac: Aircraft, m: float, h: float = 0.0) -> dict:
    """Ground roll (aerolib, Raymer energy method) with the lift-off speed set by the physics of this layout:
    the aircraft lifts off at the lowest of (a) the ground-attitude lift-off speed (high wing incidence + take-off
    flap) and (b) the speed at which the tail can rotate it about the main wheels against weight and the high thrust
    line, but never below 1.1 VS_TO. Airborne distance to 15 m per Raymer 17.8.3."""
    atm = AL.isa(h)
    W = m * G
    lay = ac.layout
    g = lay["gear"]
    fe = ac.flap_to
    dcl0 = ac.sa["CL_alpha"] * math.radians(-fe["dalpha0_deg"]) if fe else 0.0
    dcd0 = fe["dCD0"] if fe else 0.0
    clmax = ac.clmax["to_trimmed"]
    VS = AL.stall_speed(W, ac.S, clmax, atm["rho"])
    CLg = ac.sa["CL_0"] + dcl0
    V_flat = math.sqrt(2 * W / (atm["rho"] * ac.S * CLg))
    x_cg = lay["cases"][0]["x"]
    arm_t = lay["x_ac_t"] - g["x_mg"]
    cap = lay["tail"]["S_h_eff"] * 0.9 * arm_t + ac.S * CLg * (g["x_mg"] - lay["x_ac_w"]) + \
        ac.S * ac.pf["mac"] * (ac.aero_cm0 - 0.25 * dcl0)
    V_R = None
    for V in np.linspace(10, 45, 351):
        T = PROP.wot(V, h)["T"]
        need = W * (g["x_mg"] - x_cg) + T * (Z_T - g["z_g"])
        if 0.5 * atm["rho"] * V * V * cap >= need:
            V_R = float(V)
            break
    V_lof = max(1.1 * VS, min(V_flat, V_R if V_R is not None else float("inf")))
    T0 = PROP.wot(0.0, h)["T"]
    Tl = PROP.wot(V_lof, h)["T"]
    r = AL.takeoff_ground_roll(W, ac.S, clmax, ac.fit["cd0_to"], k_ground(ac), T0, Tl, mu=0.04, rho=atm["rho"],
                               cl_roll=CLg, v_lof_factor=V_lof / VS)
    V_tr = max(1.15 * VS, V_lof)
    R = V_tr ** 2 / (0.2 * G)
    q = 0.5 * atm["rho"] * V_tr ** 2
    CL = W / (q * ac.S)
    D = q * ac.S * (float(ac.CD(min(CL, 1.7))) + dcd0)
    gam = math.asin(max(min((PROP.wot(V_tr, h)["T"] - D) / W, 0.5), 0.01))
    h_tr = R * (1 - math.cos(gam))
    s_air = math.sqrt(R ** 2 - (R - 15.0) ** 2) if h_tr >= 15.0 else R * math.sin(gam) + (15.0 - h_tr) / math.tan(gam)
    return {"ground_roll_m": r["ground_roll"], "V_lof_m_s": V_lof, "VS_TO_m_s": VS, "V_flat_liftoff_m_s": V_flat,
            "V_rotation_authority_m_s": V_R, "T_static_N": T0, "T_lof_N": Tl, "air_distance_15m_m": s_air,
            "distance_15m_m": r["ground_roll"] + s_air, "climb_gradient": math.sin(gam), "CL_ground": CLg,
            "flap_deg": fe["delta_deg"] if fe else 0.0,
            "liftoff_mode": "flat (ground attitude)" if V_lof >= V_flat - 1e-6 else
                            ("rotated" if V_lof > 1.1 * VS + 1e-6 else "rotated at 1.1 VS")}


def landing(ac: Aircraft, m: float, h: float = 0.0) -> dict:
    atm = AL.isa(h)
    W = m * G
    clmax = ac.clmax["ld_trimmed"]
    r = AL.landing_roll(W, ac.S, clmax, ac.fit["cd0_ld"], k_ground(ac), mu_brake=0.3, rho=atm["rho"],
                        v_td_factor=1.15, cl_roll=ac.sa["CL_0"], t_free=1.0)
    VS = r["VS_ld"]
    Vf = 1.23 * VS
    R = Vf ** 2 / (0.2 * G)
    gam = math.radians(3.0)
    hf = R * (1 - math.cos(gam))
    s_air = (15.0 - hf) / math.tan(gam) + R * math.sin(gam)
    return {"ground_roll_m": r["ground_roll"], "V_td_m_s": r["V_td"], "VS_m_s": VS, "air_distance_15m_m": s_air,
            "distance_15m_m": r["ground_roll"] + s_air}


def p_avail_fn(h):
    return lambda V: PROP.wot(V, h)["T"] * V


def ceiling(ac: Aircraft, m: float, roc_target: float = 0.5) -> float:
    lo, hi = 0.0, 9000.0
    for _ in range(30):
        mid = 0.5 * (lo + hi)
        roc, _ = AL.rate_of_climb(m * G, ac.S, ac.fit["cd0"], ac.fit["k"], p_avail_fn(mid), AL.isa(mid)["rho"])
        lo, hi = (mid, hi) if roc >= roc_target else (lo, mid)
    return lo


def performance(ac: Aircraft, m0: float, fuel: float, mis: dict) -> dict:
    W = m0 * G
    out = {}
    for h in (0.0, H_LOITER):
        atm = AL.isa(h)
        sp = AL.speeds(W, ac.S, ac.fit["cd0"], ac.fit["k"], atm["rho"])
        vmax = AL.max_level_speed(W, ac.S, ac.fit["cd0"], ac.fit["k"], p_avail_fn(h), atm["rho"])
        roc, v_roc = AL.rate_of_climb(W, ac.S, ac.fit["cd0"], ac.fit["k"], p_avail_fn(h), atm["rho"])
        cp = climb_point(ac, W, h)
        bl = best_loiter(ac, W, h)
        blu = best_loiter(ac, W, h, floor=False)
        br = best_range(ac, W, h)
        out[int(h)] = {"VS_clean_m_s": vstall(ac, W, h), "V_min_power_polar_m_s": sp["V_min_power"],
                       "V_LDmax_polar_m_s": sp["V_ld_max"], "V_max_m_s": vmax, "RoC_max_m_s": roc,
                       "V_RoC_m_s": v_roc, "RoC_trimmed_prop_model_m_s": cp["roc"], "V_climb_m_s": cp["V"],
                       "climb_rpm": cp["rpm"], "climb_tip_mach": cp["tip_mach"],
                       "loiter": {k: bl[k] for k in ("V", "EAS", "CL", "LD", "P_shaft", "P_total", "eta", "rpm",
                                                      "power_fraction", "bsfc_g_kWh", "ff_kg_h", "gen_W", "tip_mach")},
                       "loiter_unconstrained_V_m_s": blu["V"], "loiter_unconstrained_ff_kg_h": blu["ff_kg_h"],
                       "best_range": {k: br[k] for k in ("V", "CL", "LD", "P_shaft", "eta", "rpm", "power_fraction",
                                                         "bsfc_g_kWh", "ff_kg_h")},
                       "power_available_W": P_MAX * lapse(atm["sigma"]),
                       "power_lapse_research": lapse(atm["sigma"]), "power_lapse_gagg_ferrar": AL.power_lapse(atm["sigma"])}
    out["ceiling_service_m"] = ceiling(ac, m0)
    out["ceiling_absolute_m"] = ceiling(ac, m0, 0.0)
    out["takeoff_sl_mtow"] = takeoff(ac, m0, 0.0)
    out["takeoff_1500m_isa_mtow"] = takeoff(ac, m0, 1500.0)
    m_land = m0 - fuel * 0.88
    out["landing_sl_mtow"] = landing(ac, m0)
    out["landing_sl_end_of_mission"] = landing(ac, m_land)
    out["landing_mass_end_of_mission_kg"] = m_land
    out["static_wot"] = PROP.wot(0.0, 0.0)
    out["static_tip_speed_m_s"] = math.pi * D_PROP * out["static_wot"]["rpm"] / 60
    return out


# =====================================================================================================================
# 13. constraint diagram (sizinglib), V-n
# =====================================================================================================================
def constraint_diagram(ac: Aircraft, m0: float, perf: dict) -> dict:
    a = SZ.Aero(cd0=ac.fit["cd0"], k=ac.fit["k"], clmax=ac.clmax["clean_trimmed"], clmax_to=ac.clmax["to_trimmed"],
                clmax_ld=ac.clmax["ld_trimmed"], cd0_to=ac.fit["cd0_to"])
    ws = np.linspace(200.0, 800.0, 121)
    corr = lambda h: AL.power_lapse(AL.isa(h)["sigma"]) / lapse(AL.isa(h)["sigma"])   # sizinglib uses Gagg-Ferrar
    to = perf["takeoff_sl_mtow"]
    eta_to = to["T_lof_N"] * to["V_lof_m_s"] / math.sqrt(2) / PROP.wot(to["V_lof_m_s"] / math.sqrt(2), 0)["P_shaft"]
    eta_cl = perf[0]["loiter"]["eta"] * 0.0 + PROP.wot(perf[0]["V_climb_m_s"], 0)["eta"]
    V_loit3000 = perf[int(H_LOITER)]["loiter"]["V"]
    curves = {
        "takeoff_ground_roll_200m": SZ.pw_takeoff(ws, 200.0, a, eta_to=eta_to, cl_roll=ac.sa["CL_0"]),
        "climb_4p9_m_s_sea_level": SZ.pw_climb(ws, ROC_REQ, 0.0, a, eta=eta_cl),
        "ceiling_4500m_0p5_m_s": SZ.pw_ceiling(ws, H_CEIL_REQ, a, eta=0.72) * corr(H_CEIL_REQ),
        "cruise_36_m_s_3000m_75pct": SZ.pw_cruise(ws, V_CRUISE_MAX, H_LOITER, a, eta=0.78, throttle=0.75) * corr(H_LOITER),
        "loiter_turn_30deg_bank_3000m": SZ.pw_turn(ws, V_loit3000, H_LOITER, 1 / math.cos(math.radians(30)), a,
                                                   eta=0.75) * corr(H_LOITER),
    }
    ws_stall = SZ.ws_stall(24.0, ac.clmax["clean_trimmed"])
    ws_land = SZ.ws_landing(200.0, ac.clmax["ld_trimmed"])
    ws_loiter = 0.5 * RHO0 * V_LOIT_MIN_EAS ** 2 * ac.clmax["clean_trimmed"] / 1.2 ** 2
    ws_max = min(ws_stall, ws_land)
    dp = SZ.design_point(ws, curves, ws_max)
    pw_avail_mcp = P_MCP / (m0 * G)
    pw_prop_climb = PROP.wot(perf[0]["V_climb_m_s"], 0.0)["P_shaft"] / (m0 * G)
    pw_avail_max = P_MAX / (m0 * G)
    ws_design = m0 * G / ac.S
    req_at_design = {k: float(np.interp(ws_design, ws, v)) for k, v in curves.items()}
    return {"ws_grid": ws, "curves": curves, "ws_stall_24": ws_stall, "ws_landing_200": ws_land,
            "ws_loiter_floor_1p2VS": ws_loiter, "design_point_min_power": dp, "pw_available_mcp": pw_avail_mcp,
            "pw_available_max": pw_avail_max, "ws_design": ws_design, "pw_required_at_design": req_at_design,
            "pw_prop_absorbed_wot_climb": pw_prop_climb,
            "eta_takeoff": eta_to, "eta_climb": eta_cl}


def vn_summary(ac: Aircraft, m0: float, VD: float) -> dict:
    ws = m0 * G / ac.S
    out = {}
    for h in (0.0, H_CEIL_REQ):
        vn = ST.vn_diagram(ws, ac.clmax["clean_trimmed"], -0.9, N_POS, N_NEG, VC_EAS, VD, ac.sa["CL_alpha"],
                           ac.pf["mac"], rho=AL.isa(h)["rho"])
        out[int(h)] = {"VS": vn["VS"], "VA": min(vn["VA"], VC_EAS), "VC": VC_EAS, "VD": VD, "kg": vn["kg"],
                       "n_gust_VC": vn["gust_C"], "n_gust_VD": vn["gust_D"], "n_limit_pos": vn["n_limit_pos"],
                       "n_limit_neg": vn["n_limit_neg"]}
    return out


# =====================================================================================================================
# 14. trade studies
# =====================================================================================================================
SPAN_MAX = rec("span_max_m", float(CP["recommended_ranges"]["span_m"]["max"]), "comparables.yaml#recommended_ranges.span_m.max",
               basis="6.8 m: practical limit for one-piece-per-side transport panels")
TIP_CHORD_MIN = rec("tip_chord_min_m", 0.25, "", basis="aileron servo (Volz DA 26 case 0.103 m tall) and hinge "
                    "installation in the 13 % tip section, tip Re >= 0.4e6 at the loiter speed (estimate)")
LB_MAX = rec("length_over_span_max", 0.60, "comparables.yaml#recommended_ranges.length_m.basis",
             basis="long-span endurance types L/b 0.51-0.60")
FIELD_ROLL_MAX = rec("field_ground_roll_max_m", 200.0, "baseline.yaml#mission_targets.runway_length_m",
                     basis="300 m strip / 1.5 field factor for take-off (MTOM, ISA SL) and landing (end-of-mission mass)")


def point_summary(r: dict) -> dict:
    ac = r["ac"]
    to = takeoff(ac, r["m0"])
    ld = landing(ac, r["m0"] - r["fuel"] * 0.88)
    L_all = SPINNER["x1"]
    return {"AR": ac.pf["AR"], "ws_Pa": ac.d.ws, "S_m2": ac.S, "span_m": ac.pf["b"], "c_tip_m": ac.pf["ct"],
            "stretch_m": r["stretch"], "length_m": L_all, "endurance_h": r["endurance_h"], "fuel_kg": r["fuel"],
            "empty_kg": r["empty"], "wing_kg": ac.mass["groups"]["wing"], "tail_kg": ac.mass["groups"]["tail"],
            "S_V_m2": ac.layout["tail"]["S_V"], "l_h_m": ac.layout["l_h"], "cd0": ac.fit["cd0"], "e": ac.fit["e"],
            "LD_max": ac.fit["LD_max"], "VS_m_s": vstall(ac, r["m0"] * G, 0.0), "to_roll_m": to["ground_roll_m"],
            "ldg_roll_m": ld["ground_roll_m"], "n_gust_limit": ac.layout["gust"]["n_limit"]}


def feasible(sm: dict) -> list:
    bad = []
    if sm["span_m"] > SPAN_MAX + 1e-6:
        bad.append("span")
    if sm["c_tip_m"] < TIP_CHORD_MIN:
        bad.append("tip chord")
    if sm["VS_m_s"] > 24.0:
        bad.append("stall")
    if sm["to_roll_m"] > FIELD_ROLL_MAX:
        bad.append("take-off roll")
    if sm["ldg_roll_m"] > FIELD_ROLL_MAX:
        bad.append("landing roll")
    return bad


def ws_stall_corner(base: Design, AR: float, stretch: float) -> float:
    """Wing loading that puts the clean trimmed 1-g stall speed at 24 m/s (MTOM, sea level) for this AR."""
    set_fuselage(stretch)
    d = replace(base, AR=AR, ws=466.0)
    ac, aero = solve_configuration(d, d.mtom, 35.0, v_ref_loiter())
    finish_aero(ac, aero, v_ref_loiter(), H_LOITER, 0.31)
    return round(0.5 * RHO0 * 24.0 ** 2 * ac.clmax["clean_trimmed"] * 0.99, 1)


def run_trades(base: Design) -> dict:
    out = {}
    print(f"\n[trade 1] aft-body stretch (tail arm vs wetted area), AR {base.AR:.0f}, W/S {base.ws:.0f} Pa")
    rows = []
    for st in (0.10, 0.25, 0.40, 0.55):
        r = evaluate(base, st)
        sm = point_summary(r)
        sm["violations"] = feasible(sm) + (["L/b"] if sm["length_m"] > LB_MAX * sm["span_m"] else [])
        rows.append(sm)
        print(f"   stretch {st:.2f}: L {sm['length_m']:.2f} m, l_h {sm['l_h_m']:.2f} m, S_V {sm['S_V_m2']:.2f} m2, "
              f"empty {sm['empty_kg']:.1f} kg, E {sm['endurance_h']:.2f} h {sm['violations']}")
    ok = [x for x in rows if not x["violations"]] or [min(rows, key=lambda x: len(x["violations"]))]
    best_st = max(ok, key=lambda x: x["endurance_h"])["stretch_m"]
    out["aft_stretch"] = {"rows": rows, "chosen_m": best_st,
                          "rule": "max endurance subject to L <= 0.60 b and the field/stall/geometry limits"}
    print(f"   -> stretch {best_st:.2f} m")

    print("\n[trade 2] aspect ratio x wing loading at MTOM 145 kg (endurance with the design mission)")
    rows = []
    for AR in (12.0, 13.0, 14.0, 15.0):
        for ws in (400.0, 433.0, ws_stall_corner(base, AR, best_st), 500.0):
            d = replace(base, AR=AR, ws=round(ws, 1))
            r = evaluate(d, best_st)
            sm = point_summary(r)
            sm["violations"] = feasible(sm)
            rows.append(sm)
            print(f"   AR {AR:4.1f} W/S {ws:5.0f}: b {sm['span_m']:.2f} ct {sm['c_tip_m']:.3f} wing {sm['wing_kg']:.1f} "
                  f"empty {sm['empty_kg']:.1f} L/D {sm['LD_max']:.2f} E {sm['endurance_h']:.2f} h "
                  f"TO {sm['to_roll_m']:.0f} LDG {sm['ldg_roll_m']:.0f} {sm['violations']}")
    ok = [x for x in rows if not x["violations"]]
    if not ok:
        flag("no aspect-ratio/wing-loading point met every hard limit; the least-violating point was taken")
        nmin = min(len(x["violations"]) for x in rows)
        ok = [x for x in rows if len(x["violations"]) == nmin]
    best = max(ok, key=lambda x: x["endurance_h"])
    out["aspect_ratio_wing_loading"] = {"rows": rows, "chosen": {"AR": best["AR"], "ws_Pa": best["ws_Pa"]},
                                        "rule": "max endurance among points meeting span <= 6.8 m, tip chord >= 0.25 m, "
                                                "VS <= 24 m/s, field rolls <= 200 m (L/b is reported, not a limit)",
                                        "finding": "endurance rises with wing loading because the process minimum-gauge "
                                                   "skins make wing mass scale with area, while the loiter CL is capped "
                                                   "by the 1.2 VS margin; the optimum therefore sits on the 24 m/s stall "
                                                   "limit, and at that loading the span limit decides the aspect ratio"}
    print(f"   -> AR {best['AR']:.0f}, W/S {best['ws_Pa']:.0f} Pa")
    return out


def run_config_trades(d: Design, stretch: float, ref: dict) -> dict:
    out = {}
    ac = ref["ac"]
    E0 = ref["endurance_h"]
    # ---------------- flaps
    print("\n[trade 3] take-off flaps")
    r = evaluate(replace(d, flaps=False), stretch)
    sm = point_summary(r)
    out["flaps"] = {
        "with_takeoff_flaps": {"endurance_h": E0, "to_roll_m": takeoff(ac, ref["m0"])["ground_roll_m"]},
        "without_flaps": {"endurance_h": sm["endurance_h"], "to_roll_m": sm["to_roll_m"], "violations": feasible(sm)},
        "landing_flap_rejected": "with the incidence set for a level fuselage in loiter, any landing flap lowers the "
                                 "touch-down attitude below the 3 deg mains-first target (nose-wheel-first risk); the flaps "
                                 "are take-off flaps only (15 deg), giving a flat lift-off below the rotation-authority speed",
        "decision": "keep inboard plain take-off flaps" if sm["to_roll_m"] > FIELD_ROLL_MAX else
                    "flaps optional (field length met without them)"}
    print(f"   no flaps: E {sm['endurance_h']:.2f} h, TO roll {sm['to_roll_m']:.0f} m; with flaps: E {E0:.2f} h")
    # ---------------- propeller
    print("\n[trade 4] propeller 32x18 2B vs 31x12 3B")
    set_prop("0164")
    r3 = evaluate(d, stretch)
    p3 = {"endurance_h": r3["endurance_h"], "roc_sl_m_s": climb_point(r3["ac"], r3["m0"] * G, 0.0)["roc"],
          "to_roll_m": takeoff(r3["ac"], r3["m0"])["ground_roll_m"],
          "loiter_rpm_3000m": best_loiter(r3["ac"], r3["m0"] * G, H_LOITER)["rpm"]}
    set_prop("0161")
    p2 = {"endurance_h": E0, "roc_sl_m_s": climb_point(ac, ref["m0"] * G, 0.0)["roc"],
          "to_roll_m": takeoff(ac, ref["m0"])["ground_roll_m"],
          "loiter_rpm_3000m": best_loiter(ac, ref["m0"] * G, H_LOITER)["rpm"]}
    out["propeller"] = {"mejzlik_32x18_2B": p2, "mejzlik_31x12_3B": p3,
                        "decision": "32x18 2B (endurance); 31x12 3B is the climb/hot-day option"}
    print(f"   32x18 2B: {p2}\n   31x12 3B: {p3}")
    # ---------------- tail type
    print("\n[trade 5] tail type")
    lay = ac.layout
    t = lay["tail"]
    g = lay["gear"]
    Dq_v = ac.drag_items["tail"]
    m_v = ac.mass["groups"]["tail"]
    q_ratio = lambda Q: Q / 1.03
    # inverted V: tips below the aft fuselage
    x_root = t["secs"][0]["x_le"] + 0.5 * t["cr"]
    z_root_inv = float(z_bot(x_root)[0]) + 0.02
    z_tip_inv = z_root_inv - t["span_panel"] * math.sin(math.radians(t["gamma_deg"]))
    x_tip = t["secs"][1]["x_le"] + 0.5 * t["ct"]
    th = math.radians(g["theta_design_deg"])
    need_zg = z_tip_inv - (0.10 + (x_tip - g["x_mg"]) * math.sin(th)) / math.cos(th)
    dh_gear = max(g["z_g"] - need_zg, 0.0)
    dm_gear = GEAR_LEGS_145 * dh_gear / max(g["h_gear"] + WHEEL_D / 2, 0.1)
    dDq_gear = 1.2 * 2 * 0.05 * dh_gear * 1.35 * 0.035 + 1.2 * 0.05 * dh_gear * 0.04
    variants = {
        "upright_V": {"dDq_m2": 0.0, "dm_kg": 0.0, "junctions": 2, "Q": 1.03,
                      "notes": "2 identical panels (one mould pair), root on the engine-bay frame, tips clear of ground; "
                               "panel wakes cross the upper prop disc (outer 40 % of each panel outside the disc)"},
        "inverted_V": {"dDq_m2": dDq_gear, "dm_kg": dm_gear, "junctions": 2, "Q": 1.03,
                       "tip_z_m": z_tip_inv, "extra_gear_height_m": dh_gear,
                       "notes": "proverse roll-yaw coupling and prop guard, but the tips hang "
                                f"{-z_tip_inv + g['z_g']:.2f} m below the ground line of the upright-V layout: the gear "
                                f"would grow by {dh_gear:.2f} m (bow mass, drag, tip-over) - rejected"},
        "T_tail": {"dDq_m2": Dq_v * (q_ratio(1.05) - 1) + 0.0004, "dm_kg": 0.45 * m_v * t["S_v_eff"] / t["S_V"] + 0.30,
                   "junctions": 3, "Q": 1.05,
                   "notes": "fin carries the stabiliser loads (+50 % fin mass, T-fitting 0.3 kg), deep-stall exposure "
                            "with a high-AR wing, flutter-critical; stabiliser well clear of the prop"},
        "Y_tail": {"dDq_m2": Dq_v * 0.12 + 0.0003, "dm_kg": 0.12 * m_v + 0.2, "junctions": 3, "Q": 1.04,
                   "notes": "V + ventral fin (MQ-9 style): ventral fin is the prop guard but adds a surface and a "
                            "junction; the ventral bumper skid of the V layout does the guard job for 0.1 kg"},
    }
    for k, v in variants.items():
        if k == "upright_V":
            v["endurance_h"] = E0
            continue
        r = evaluate(d, stretch, extra={"dDq": v["dDq_m2"], "dm": v["dm_kg"]})
        v["endurance_h"] = r["endurance_h"]
        print(f"   {k}: dD/q {v['dDq_m2']:.4f} m2, dm {v['dm_kg']:.2f} kg -> E {v['endurance_h']:.2f} h")
    out["tail_type"] = {"variants": variants, "decision": "upright V-tail with a ventral bumper skid",
                        "purser_campbell_note": "V-tail total area equals S_h + S_v for the same volume coefficients "
                                                "(stablib.v_tail_projection), so the gain is in junctions, parts count "
                                                "and the ground-clearance-free geometry, not in area"}
    # ---------------- landing gear
    print("\n[trade 6] fixed faired vs retractable gear")
    dDq_r = -0.9 * ac.drag_items["landing_gear"]
    dm_r = (11.5 - 9.1) + 1.0 - GEAR_FAIRINGS
    r = evaluate(d, stretch, extra={"dDq": dDq_r, "dm": dm_r})
    out["landing_gear"] = {
        "fixed_faired": {"endurance_h": E0, "gear_Dq_m2": ac.drag_items["landing_gear"]},
        "retractable": {"endurance_h": r["endurance_h"], "dDq_m2": dDq_r, "dm_kg": dm_r,
                        "basis": "components.yaml: custom electric retract (SAGITTA) 11.5 kg vs fixed 9.1 kg, +1.0 kg "
                                 "bays/doors (estimate), minus the 0.95 kg fairings; 90 % of the gear drag removed"},
        "decision": "fixed tricycle with fairings: retraction buys "
                    f"{r['endurance_h'] - E0:+.2f} h for a retraction failure mode, gear bays in the CG zone "
                    "(where the fuel cells and payload bay sit) and no purchasable unit"}
    print(f"   retractable: E {r['endurance_h']:.2f} h vs fixed {E0:.2f} h")
    return out


# =====================================================================================================================
# 15. outputs: sketch (3-view), constraint diagram, concept.yaml
# =====================================================================================================================
def draw_sketch(ac: Aircraft, perf: dict, res: dict, path: Path):
    """3-view (plan, side, front) at one common scale from the actual OML/sections, with the internal layout as hidden
    lines and the key dimensions."""
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.patches import Circle, Ellipse, Polygon, Rectangle

    lay = ac.layout
    g = lay["gear"]
    t = lay["tail"]
    pf = ac.pf
    b2 = pf["b"] / 2
    W_IN, H_IN = 18.0, 11.0
    sc = 1.30                                    # inches per metre, all three views at the same scale
    fig = plt.figure(figsize=(W_IN, H_IN), dpi=150)
    fill, skin, edge, dark = "#d6dbe1", "#e9ecf0", "#1f2a37", "#4b5563"
    acc, hid = "#b45309", "#2563eb"

    def axes(x0, y0, wm, hm, xlim, ylim):
        ax = fig.add_axes([x0 / W_IN, y0 / H_IN, wm * sc / W_IN, hm * sc / H_IN])
        ax.set_xlim(*xlim)
        ax.set_ylim(*ylim)
        ax.set_aspect("equal")
        ax.axis("off")
        return ax

    def dim(ax, p0, p1, text, off=(0, 0), rot=0, fs=8.5, col=dark):
        ax.annotate("", xy=p1, xytext=p0, arrowprops=dict(arrowstyle="<->", color=col, lw=0.8, shrinkA=0, shrinkB=0))
        ax.text((p0[0] + p1[0]) / 2 + off[0], (p0[1] + p1[1]) / 2 + off[1], text, ha="center", va="center",
                fontsize=fs, color=col, rotation=rot, bbox=dict(fc="white", ec="none", pad=0.6), zorder=20)

    def wheel(ax, xc, zc_, side=True):
        if side:
            ax.add_patch(Circle((xc, zc_), WHEEL_D / 2, fc="#2b2f36", ec=edge, lw=0.8, zorder=6))
            ax.add_patch(Circle((xc, zc_), 0.045, fc="#9ca3af", ec=edge, lw=0.5, zorder=7))
            ax.add_patch(Polygon(np.array([[xc - 0.17, zc_ + 0.02], [xc - 0.10, zc_ + 0.11], [xc + 0.12, zc_ + 0.10],
                                           [xc + 0.16, zc_ + 0.03], [xc + 0.12, zc_ - 0.01], [xc - 0.12, zc_ - 0.01]]),
                                 fc=skin, ec=edge, lw=0.8, zorder=8))

    xs = np.linspace(0.0, L_FUS, 400)
    w, h, zc, nt, nb, tf = fus_section(xs)
    ztop, zbot = zc + tf * h, zc - (1 - tf) * h
    secs = ac.wing_secs
    ys = np.array([s_["y"] for s_ in secs])
    xle = np.array([s_["x_le"] for s_ in secs])
    ch = np.array([s_["chord"] for s_ in secs])
    tsec = t["secs"]
    L_tot = SPINNER["x1"]
    tk = lay["tank"]
    xcg = lay["cases"][0]["x"]
    zcg = lay["cases"][0]["z"]
    zt_ball = float(z_bot(X_TURRET)[0]) - 0.04

    # ---------------- plan view (span horizontal, nose up)
    axp = axes(0.35, 3.0, pf["b"] + 0.6, L_tot + 0.75, (-b2 - 0.3, b2 + 0.3), (-L_tot - 0.35, 0.4))
    wing_poly = np.vstack([np.column_stack([ys, -xle]), np.column_stack([ys[::-1], -(xle + ch)[::-1]])])
    wing_full = np.vstack([np.column_stack([-wing_poly[:, 0][::-1], wing_poly[:, 1][::-1]]), wing_poly])
    axp.add_patch(Polygon(np.column_stack([np.r_[w / 2, -w[::-1] / 2], np.r_[-xs, -xs[::-1]]]), fc=fill, ec=edge,
                          lw=1.0, zorder=2))
    axp.add_patch(Polygon(np.column_stack([[SPINNER["d"] / 2, 0, -SPINNER["d"] / 2], [-SPINNER["x0"], -SPINNER["x1"],
                                                                                      -SPINNER["x0"]]]),
                          fc="#9ca3af", ec=edge, lw=0.8, zorder=2))
    # panel lines / hatches on the fuselage (shell panels)
    for xx in (0.55, X_CHUTE[0], X_CHUTE[1], X_HUB - 0.30):
        wx = float(fus_section(xx)[0][0]) / 2
        axp.plot([-wx, wx], [-xx, -xx], color=dark, lw=0.5, zorder=3)
    axp.add_patch(Polygon(wing_full, fc=skin, ec=edge, lw=1.1, zorder=4))
    for sgn in (1, -1):
        for y0, y1, ls in ((0.57 * b2, 0.95 * b2, "-"), (lay["w_fus"] / 2, ac.d.flap_span_frac * b2, "--")):
            yy = np.array([y0, y1])
            xh = np.interp(yy, ys, xle + 0.75 * ch)
            xt = np.interp(yy, ys, xle + ch)
            axp.plot(sgn * yy, -xh, color=dark, lw=0.6, ls=ls, zorder=5)
            axp.plot([sgn * y0, sgn * y0], [-xh[0], -xt[0]], color=dark, lw=0.6, ls=ls, zorder=5)
            axp.plot([sgn * y1, sgn * y1], [-xh[1], -xt[1]], color=dark, lw=0.6, ls=ls, zorder=5)
        tp = np.array([[tsec[0]["y"], -tsec[0]["x_le"]], [tsec[1]["y"], -tsec[1]["x_le"]],
                       [tsec[1]["y"], -(tsec[1]["x_le"] + tsec[1]["chord"])],
                       [tsec[0]["y"], -(tsec[0]["x_le"] + tsec[0]["chord"])]])
        tp[:, 0] *= sgn
        axp.add_patch(Polygon(tp, fc=skin, ec=edge, lw=1.0, zorder=5))
        yh = np.array([tsec[0]["y"], tsec[1]["y"]])
        xh = np.array([tsec[0]["x_le"] + 0.70 * tsec[0]["chord"], tsec[1]["x_le"] + 0.70 * tsec[1]["chord"]])
        axp.plot(sgn * yh, -xh, color=dark, lw=0.6, zorder=6)
        axp.add_patch(Rectangle((sgn * g["track"] / 2 - WHEEL_W / 2, -g["x_mg"] - WHEEL_D / 2), WHEEL_W, WHEEL_D,
                                fc="none", ec=dark, lw=0.7, ls=":", zorder=6))
    axp.add_patch(Rectangle((-WHEEL_W / 2, -X_NG - 0.06 - WHEEL_D / 2), WHEEL_W, WHEEL_D, fc="none", ec=dark, lw=0.7,
                            ls=":", zorder=6))
    axp.add_patch(Rectangle((-D_PROP / 2, -X_PROP - 0.012), D_PROP, 0.024, fc="#6b7280", ec=edge, lw=0.6, zorder=3))
    axp.add_patch(Circle((0, -X_TURRET), TUR_D / 2, fc="none", ec=dark, lw=0.7, ls=":", zorder=6))
    axp.plot([0], [-xcg], marker="o", ms=7, mfc="white", mec=acc, mew=1.5, zorder=8)
    axp.plot([0], [-lay["x_np"]], marker="x", ms=7, color=acc, mew=1.5, zorder=8)
    axp.text(0.10, -xcg + 0.03, "AM", fontsize=8, color=acc, zorder=9)
    axp.text(0.10, -lay["x_np"] - 0.11, "NN", fontsize=8, color=acc, zorder=9)
    m = lay["wing_mac"]
    axp.plot([m["y_mac"], m["y_mac"]], [-m["x_le_mac"], -(m["x_le_mac"] + m["mac"])], color=acc, lw=1.4, zorder=7)
    axp.text(m["y_mac"] + 0.06, -m["x_le_mac"] - m["mac"] / 2, f"OAK {m['mac']:.3f} m", fontsize=7.5, color=acc,
             va="center", zorder=9)
    dim(axp, (-b2, 0.22), (b2, 0.22), f"Kanat açıklığı b = {pf['b']:.2f} m")
    dim(axp, (-b2 - 0.15, 0.0), (-b2 - 0.15, -L_tot), f"Toplam uzunluk {L_tot:.2f} m", rot=90)
    yt = tsec[1]["y"]
    ytl = -tsec[1]["x_le"] - tsec[1]["chord"] - 0.20
    dim(axp, (-yt, ytl), (yt, ytl), f"V-kuyruk uçtan uca {2 * yt:.2f} m", fs=8)
    axp.text(b2 * 0.52, -0.25, f"S = {ac.S:.2f} m²   AR = {pf['AR']:.1f}   λ = {pf['taper']:.2f}\n"
             f"NLF(1)-0416 %16 kök → %13 uç, {ac.d.washout:.1f}° burulma, i = {lay['i_w']:.2f}°", fontsize=8.5,
             color=edge, ha="center")
    axp.text(-b2 * 0.52, -0.25, "kesik çizgi: kalkış flabı (iç)\nsürekli: kanatçık / kuyruk dümeni", fontsize=7.5,
             color=dark, ha="center")
    axp.text(-b2 - 0.25, 0.33, "ÜSTTEN GÖRÜNÜŞ", fontsize=10, weight="bold", color=edge)

    # ---------------- side view (nose left)
    zmin, zmax = g["z_g"] - 0.12, t["secs"][1]["z_le"] + 0.20
    axs = axes(10.35, 6.95, L_tot + 0.6, zmax - zmin, (-0.25, L_tot + 0.35), (zmin, zmax))
    axs.plot([-0.2, L_tot + 0.3], [g["z_g"], g["z_g"]], color=dark, lw=0.8)
    for xx in np.arange(-0.2, L_tot + 0.3, 0.12):
        axs.plot([xx, xx - 0.06], [g["z_g"], g["z_g"] - 0.05], color="#9ca3af", lw=0.5)
    tside = np.array([[tsec[0]["x_le"], tsec[0]["z_le"]], [tsec[1]["x_le"], tsec[1]["z_le"]],
                      [tsec[1]["x_le"] + tsec[1]["chord"], tsec[1]["z_le"]],
                      [tsec[0]["x_le"] + tsec[0]["chord"], tsec[0]["z_le"]]])
    axs.add_patch(Polygon(tside, fc=skin, ec=edge, lw=1.0, zorder=2))
    axs.plot([tsec[0]["x_le"] + 0.7 * tsec[0]["chord"], tsec[1]["x_le"] + 0.7 * tsec[1]["chord"]],
             [tsec[0]["z_le"], tsec[1]["z_le"]], color=dark, lw=0.6, zorder=2)
    axs.add_patch(Polygon(np.column_stack([np.r_[xs, xs[::-1]], np.r_[ztop, zbot[::-1]]]), fc=fill, ec=edge, lw=1.0,
                          zorder=3))
    axs.plot(xs, zc, color=dark, lw=0.4, zorder=3)                      # chine line
    axs.add_patch(Polygon(np.array([[SPINNER["x0"], Z_T + SPINNER["d"] / 2], [SPINNER["x1"], Z_T],
                                    [SPINNER["x0"], Z_T - SPINNER["d"] / 2]]), fc="#9ca3af", ec=edge, lw=0.8, zorder=3))
    ls_ = oml.LiftingSurface(ac.wing_secs)
    P0 = ls_.section_points(0)
    axs.add_patch(Polygon(P0[:, [0, 2]], fc=skin, ec=edge, lw=1.0, zorder=5))
    xw0, xw1 = secs[0]["x_le"] - 0.08, secs[0]["x_le"] + ch[0] + 0.12
    xf = np.linspace(xw0, xw1, 40)
    zf_top = np.interp(xf, xs, ztop)
    zsad = zf_top + 0.035 * np.sin(np.pi * (xf - xw0) / (xw1 - xw0))          # wing saddle fairing
    axs.fill_between(xf, zf_top - 0.001, zsad, color=fill, ec=edge, lw=0.6, zorder=4)
    axs.add_patch(Rectangle((X_PROP - 0.012, Z_T - D_PROP / 2), 0.024, D_PROP, fc="#6b7280", ec=edge, lw=0.6, zorder=2))
    # EO/IR turret (gimbal ball with window)
    axs.add_patch(Circle((X_TURRET, zt_ball), TUR_D / 2, fc="#b8c0ca", ec=edge, lw=0.8, zorder=4))
    axs.add_patch(Ellipse((X_TURRET - 0.04, zt_ball - 0.012), 0.035, 0.06, fc="#1e3a8a", ec="none", zorder=5))
    # gear
    for xg in (X_NG, g["x_mg"]):
        zf_ = float(z_bot(xg)[0])
        za = g["z_g"] + WHEEL_D / 2
        xa = xg + (0.06 if xg == X_NG else 0.0)
        axs.add_patch(Polygon(np.array([[xg - 0.035, zf_ + 0.01], [xg + 0.035, zf_ + 0.01], [xa + 0.02, za + 0.05],
                                        [xa - 0.02, za + 0.05]]), fc="#9ca3af", ec=edge, lw=0.7, zorder=2))
        wheel(axs, xa, za)
    xb, zb = g["x_bumper"], g["z_bumper"]
    axs.add_patch(Polygon(np.array([[xb - 0.10, float(z_bot(xb - 0.10)[0]) + 0.01],
                                    [xb + 0.05, float(z_bot(xb + 0.05)[0]) + 0.01], [xb + 0.01, zb]]),
                          fc="#9ca3af", ec=edge, lw=0.7, zorder=2))
    # hidden internal layout
    boxes = [((0.14, 0.55), (-0.13, 0.10), "aviyonik\nbatarya"), ((X_CHUTE[0], X_CHUTE[1]), (0.10, 0.22), "paraşüt"),
             ((tk["cells"]["forward"]["x0"], tk["cells"]["forward"]["x1"]), (-0.17, 0.20), "yakıt"),
             ((tk["bay"][0], tk["bay"][1]), (-0.18, 0.06), "faydalı\nyük"),
             ((tk["cells"]["aft"]["x0"], tk["cells"]["aft"]["x1"]), (-0.17, 0.20), "yakıt"),
             ((X_HUB - ENV_L_SG, X_HUB), (Z_T - (ENV_H - ENV_ZC), Z_T + ENV_ZC), "L 275 EF")]
    for (x0_, x1_), (z0_, z1_), lab in boxes:
        axs.add_patch(Rectangle((x0_, z0_), x1_ - x0_, z1_ - z0_, fc="none", ec=hid, lw=0.7, ls=(0, (3, 2)), zorder=6))
        axs.text(0.5 * (x0_ + x1_), 0.5 * (z0_ + z1_), lab, fontsize=6.3, color=hid, ha="center", va="center", zorder=6)
    axs.plot([xcg], [zcg], marker="o", ms=7, mfc="white", mec=acc, mew=1.5, zorder=9)
    th = math.radians(g["prop_strike_deg"])
    x_end = X_PROP + 0.05
    axs.plot([g["x_mg"], x_end], [g["z_g"], g["z_g"] + (x_end - g["x_mg"]) * math.tan(th)], color=acc, lw=0.6, ls="--",
             zorder=1)
    axs.text(g["x_mg"] + 0.15, g["z_g"] - 0.085, f"pervane temas açısı {g['prop_strike_deg']:.1f}°, "
             f"kuyruk tamponu {g['bumper_contact_deg']:.1f}°", fontsize=6.8, color=acc)
    dim(axs, (X_PROP + 0.22, g["z_g"]), (X_PROP + 0.22, Z_T - D_PROP / 2), f"{g['prop_clear_level'] * 1000:.0f} mm",
        off=(0.15, 0), fs=7.5)
    ztop_all = t["secs"][1]["z_le"]
    dim(axs, (-0.15, g["z_g"]), (-0.15, ztop_all), f"Yükseklik {ztop_all - g['z_g']:.2f} m", rot=90, fs=7.5)
    axs.text(-0.2, zmax - 0.07, "YANDAN GÖRÜNÜŞ", fontsize=10, weight="bold", color=edge)
    axs.text(X_PROP + 0.06, Z_T + D_PROP / 2 - 0.05, f"Ø{D_PROP:.3f} m\nitici\npervane", fontsize=7.2, color=dark,
             ha="left", va="top")

    # ---------------- front view
    axf = axes(0.35, 0.40, pf["b"] + 0.6, zmax - zmin, (-b2 - 0.3, b2 + 0.3), (zmin, zmax))
    axf.plot([-b2 - 0.25, b2 + 0.25], [g["z_g"], g["z_g"]], color=dark, lw=0.8)
    axf.add_patch(Circle((0, Z_T), D_PROP / 2, fc="none", ec="#6b7280", lw=0.7, ls="--", zorder=1))
    for sgn in (1, -1):
        top, bot = [], []
        for i in range(len(secs)):
            P = ls_.section_points(i)
            top.append([P[:, 1].mean(), P[:, 2].max()])
            bot.append([P[:, 1].mean(), P[:, 2].min()])
        top, bot = np.array(top), np.array(bot)
        poly = np.vstack([top, bot[::-1]])
        poly[:, 0] *= sgn
        axf.add_patch(Polygon(poly, fc=skin, ec=edge, lw=1.0, zorder=4))
        r0 = np.array([tsec[0]["y"], tsec[0]["z_le"]])
        r1 = np.array([tsec[1]["y"], tsec[1]["z_le"]])
        dvec = (r1 - r0) / np.linalg.norm(r1 - r0)
        nvec = np.array([-dvec[1], dvec[0]])
        th0, th1 = 0.12 * tsec[0]["chord"] / 2, 0.12 * tsec[1]["chord"] / 2
        tpoly = np.array([r0 + nvec * th0, r1 + nvec * th1, r1 - nvec * th1, r0 - nvec * th0])
        tpoly[:, 0] *= sgn
        axf.add_patch(Polygon(tpoly, fc=skin, ec=edge, lw=1.0, zorder=3))
        za = g["z_g"] + WHEEL_D / 2
        zf_ = float(z_bot(g["x_mg"])[0])
        axf.add_patch(Polygon(np.array([[sgn * 0.08, zf_ + 0.01], [sgn * 0.17, zf_ + 0.01],
                                        [sgn * (g["track"] / 2 - 0.015), za + 0.03],
                                        [sgn * (g["track"] / 2 - 0.06), za]]), fc="#9ca3af", ec=edge, lw=0.7, zorder=2))
        axf.add_patch(Rectangle((sgn * g["track"] / 2 - WHEEL_W / 2, g["z_g"]), WHEEL_W, WHEEL_D * 0.6, fc="#2b2f36",
                                ec=edge, lw=0.7, zorder=3))
        axf.add_patch(Ellipse((sgn * g["track"] / 2, za + 0.02), 0.10, 0.18, fc=skin, ec=edge, lw=0.8, zorder=4))
    phi = np.linspace(0, 2 * math.pi, 240)
    Pf = FUSE.point(np.full_like(phi, 1.6), phi)
    Pe = FUSE.point(np.full_like(phi, X_HUB - 0.12), phi)
    axf.add_patch(Polygon(Pe[:, 1:], fc=fill, ec=edge, lw=0.6, ls="--", zorder=4))
    axf.add_patch(Polygon(Pf[:, 1:], fc=fill, ec=edge, lw=1.0, zorder=5))
    axf.add_patch(Circle((0, Z_T), SPINNER["d"] / 2, fc="none", ec=dark, lw=0.6, ls="--", zorder=5))
    axf.add_patch(Circle((0, zt_ball), TUR_D / 2, fc="#b8c0ca", ec=edge, lw=0.8, zorder=6))
    axf.add_patch(Ellipse((0, zt_ball - 0.012), 0.05, 0.06, fc="#1e3a8a", ec="none", zorder=7))
    axf.add_patch(Rectangle((-0.018, g["z_g"] + WHEEL_D * 0.5), 0.036, float(z_bot(X_NG)[0]) - g["z_g"] - WHEEL_D * 0.5,
                            fc="#9ca3af", ec=edge, lw=0.6, zorder=1))
    axf.add_patch(Rectangle((-WHEEL_W / 2, g["z_g"]), WHEEL_W, WHEEL_D * 0.6, fc="#2b2f36", ec=edge, lw=0.7, zorder=2))
    axf.add_patch(Ellipse((0, g["z_g"] + WHEEL_D / 2 + 0.02), 0.10, 0.18, fc=skin, ec=edge, lw=0.8, zorder=3))
    dim(axf, (-g["track"] / 2, g["z_g"] - 0.075), (g["track"] / 2, g["z_g"] - 0.075), f"İz {g['track']:.2f} m", fs=7.5)
    zdim = secs[-1]["z_le"] + 0.22
    axf.annotate("", xy=(b2, zdim), xytext=(-b2, zdim), arrowprops=dict(arrowstyle="<->", color=dark, lw=0.8,
                                                                         shrinkA=0, shrinkB=0))
    axf.text(-b2 * 0.55, zdim, f"b = {pf['b']:.2f} m  (dihedral {ac.d.dihedral:.0f}°)", ha="center", va="center",
             fontsize=8, color=dark, bbox=dict(fc="white", ec="none", pad=0.6), zorder=20)
    axf.text(b2 * 0.55, zdim, f"V-kuyruk {t['gamma_deg']:.0f}°, panel {t['span_panel']:.2f} m", ha="center",
             va="center", fontsize=8, color=dark, bbox=dict(fc="white", ec="none", pad=0.6), zorder=20)
    axf.text(-b2 - 0.25, zmin + 0.03, "ÖNDEN GÖRÜNÜŞ", fontsize=10, weight="bold", color=edge)

    # ---------------- title + data block
    fig.text(0.35 / W_IN, 10.55 / H_IN, "YK-250  —  Konsept “Dayanım” (MALE tipi, itici, V-kuyruk)", fontsize=16,
             weight="bold", color=edge)
    fig.text(0.35 / W_IN, 10.25 / H_IN, "Sivil EO/IR gözetleme ve araştırma platformu · silah/askı noktası yok · ilk "
             "boyutlandırma (calc.py) · tüm görünüşler aynı ölçekte · ölçüler m", fontsize=9.5, color=dark)
    P = perf
    loi = P[int(H_LOITER)]["loiter"]
    rows = [
        ("MTOM / boş / yakıt / faydalı yük", f"{res['m0']:.1f} / {res['empty']:.1f} / {res['fuel']:.1f} / {PAYLOAD:.1f} kg"),
        ("Kanat S, b, AR, W/S", f"{ac.S:.2f} m², {pf['b']:.2f} m, {pf['AR']:.1f}, {ac.d.ws / G:.1f} kg/m²"),
        ("Kök / uç veteri, OAK", f"{pf['cr']:.3f} / {pf['ct']:.3f} m, {pf['mac']:.3f} m"),
        ("Gövde uzunluğu / genişlik", f"{L_FUS:.2f} m (+ spinner {SPINNER['x1']:.2f}) / {lay['w_fus']:.2f} m"),
        ("CD0 / e / (L/D)maks", f"{ac.fit['cd0']:.4f} / {ac.fit['e']:.3f} / {ac.fit['LD_max']:.1f}"),
        ("VS temiz / VS kalkış flabı", f"{P[0]['VS_clean_m_s']:.1f} / {P['takeoff_sl_mtow']['VS_TO_m_s']:.1f} m/s"),
        ("Bekleme 3000 m (EAS, TAS)", f"{loi['EAS']:.1f}, {loi['V']:.1f} m/s · {loi['ff_kg_h']:.2f} kg/h · "
                                      f"{loi['rpm']:.0f} dev/dk"),
        ("Vmaks DS / 3000 m", f"{P[0]['V_max_m_s']:.1f} / {P[int(H_LOITER)]['V_max_m_s']:.1f} m/s"),
        ("Dayanım (görev, %10 yedek)", f"{res['endurance_h']:.1f} h"),
        ("Menzil (feribot, %10 yedek)", f"{res['range_km']:.0f} km"),
        ("Tırmanma DS / 3000 m", f"{P[0]['RoC_max_m_s']:.2f} / {P[int(H_LOITER)]['RoC_max_m_s']:.2f} m/s"),
        ("Servis tavanı", f"{P['ceiling_service_m']:.0f} m"),
        ("Kalkış / iniş yerde koşu", f"{P['takeoff_sl_mtow']['ground_roll_m']:.0f} / "
                                     f"{P['landing_sl_end_of_mission']['ground_roll_m']:.0f} m"),
        ("Statik marj (tüm yüklemeler)", f"{min(lay['sms']):.3f} – {max(lay['sms']):.3f}"),
        ("V_H / V_V (izdüşüm)", f"{res['VH']:.3f} / {res['VV']:.4f}"),
        ("Pervane", f"Mejzlik 32x18 2B, statik uç Mach {P['static_wot']['tip_mach']:.2f}"),
        ("Pervane–yer açıklığı", f"{g['prop_clear_level'] * 1000:.0f} mm yatay, "
                                 f"{g['prop_clear_lof'] * 1000:.0f} mm @ {g['theta_lof_deg']:.1f}°"),
    ]
    y0 = 6.25
    fig.text(10.35 / W_IN, (y0 + 0.3) / H_IN, "ANA DEĞERLER", fontsize=10, weight="bold", color=edge)
    for i, (k, v) in enumerate(rows):
        yy = (y0 - 0.322 * i) / H_IN
        fig.text(10.35 / W_IN, yy, k, fontsize=8.8, color=dark)
        fig.text(13.35 / W_IN, yy, v, fontsize=8.8, color=edge, weight="bold")
    fig.text(10.35 / W_IN, 0.42 / H_IN, "AM: ağırlık merkezi (tüm yüklemelerde ±2 mm) · NN: nötr nokta · OAK: ortalama "
             "aerodinamik kiriş\nnoktalı: görünmeyen · mavi kesik: iç yerleşim", fontsize=7.5, color=dark)
    fig.savefig(path, dpi=150, facecolor="white")
    plt.close(fig)


def draw_constraints(cd: dict, path: Path):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    fig, ax = plt.subplots(figsize=(9, 6), dpi=130)
    ws = cd["ws_grid"]
    names = {"takeoff_ground_roll_200m": "Kalkış yerde koşu ≤ 200 m (DS)", "climb_4p9_m_s_sea_level": "Tırmanma 4,9 m/s (DS)",
             "ceiling_4500m_0p5_m_s": "Servis tavanı 4500 m", "cruise_36_m_s_3000m_75pct": "Seyir 36 m/s, 3000 m, %75 güç",
             "loiter_turn_30deg_bank_3000m": "Bekleme dönüşü 30°, 3000 m"}
    cols = ["#1d4ed8", "#b45309", "#047857", "#7c3aed", "#be123c"]
    for (k, v), c in zip(cd["curves"].items(), cols):
        ax.plot(ws / G, v, color=c, lw=1.6, label=names.get(k, k))
    ax.axvline(cd["ws_stall_24"] / G, color="#111827", lw=1.2, ls="--", label="VS ≤ 24 m/s (temiz)")
    ax.axvline(cd["ws_landing_200"] / G, color="#6b7280", lw=1.2, ls="-.", label="İniş yerde koşu ≤ 200 m")
    ax.axhline(cd["pw_available_mcp"], color="#111827", lw=1.0, ls=":", label="L 275 EF MCP 16,2 kW")
    ax.axhline(cd["pw_available_max"], color="#111827", lw=1.0, ls="-", alpha=0.5, label="L 275 EF maks. 18 kW")
    ax.axhline(cd["pw_prop_absorbed_wot_climb"], color="#b45309", lw=1.0, ls="--",
               label="32x18 2B tam gazda çektiği güç (tırmanma hızında)")
    ax.plot([cd["ws_design"] / G], [max(cd["pw_required_at_design"].values())], "o", ms=9, mfc="white", mec="#b45309",
            mew=2, label="Tasarım noktası")
    ax.set_xlim(ws[0] / G, ws[-1] / G)
    ax.set_ylim(0, 22)
    ax.set_xlabel("Kanat yüklemesi W/S [kg/m²]")
    ax.set_ylabel("Gerekli şaft gücü / ağırlık P/W [W/N] (deniz seviyesi eşdeğeri)")
    ax.set_title("YK-250 “Dayanım” – kısıt diyagramı (sizinglib)")
    ax.grid(alpha=0.3)
    ax.legend(fontsize=7.5, loc="upper left")
    fig.tight_layout()
    fig.savefig(path, facecolor="white")
    plt.close(fig)


# =====================================================================================================================
# 16. packaging checks
# =====================================================================================================================
INTAKE_W = rec("engine_intake_box_width_m", 0.30, "", basis="width of the throttle bodies + filters below the crank axis "
               "not published (engine.yaml note); 0.30 m assumed, the full 0.397 m box is checked as well")


def packaging_checks(ac: Aircraft) -> dict:
    out = {}
    m = 0.010
    zc_hi, zc_lo = Z_T + ENV_ZC, Z_T - (ENV_H - ENV_ZC)
    boxes = {
        "cylinder_slab": ((X_HUB - 0.18, X_HUB - 0.05), ENV_W / 2, (Z_T - ENV_ZC, zc_hi)),
        "intake_box_below_crank": ((X_HUB - 0.18, X_HUB - 0.05), INTAKE_W / 2, (zc_lo, Z_T - ENV_ZC)),
        "crankcase_sg750_core": ((X_HUB - ENV_L_SG, X_HUB - 0.02), 0.10, (Z_T - 0.10, Z_T + 0.05)),
    }
    for k, ((x0, x1), yh, (z0, z1)) in boxes.items():
        ok = all(inside_oml(x, sy * yh, z, m) for x in np.linspace(x0, x1, 15) for sy in (1, -1) for z in (z0, z1))
        out[k] = {"x_range_m": [x0, x1], "half_width_m": yh, "z_range_m": [z0, z1], "fits_with_10mm": bool(ok)}
    xs = np.linspace(X_HUB - ENV_L, X_HUB, 45)
    fit = [x for x in xs if all(inside_oml(x, sy * ENV_W / 2, z, m) for sy in (1, -1) for z in (zc_lo, zc_hi))]
    out["documented_full_box"] = {"size_m": [ENV_L, ENV_W, ENV_H], "x_range_m": [X_HUB - ENV_L, X_HUB],
                                  "fits_between_x_m": [min(fit), max(fit)] if fit else None,
                                  "note": "aft end of the box (prop flange, 0.072 m OD) lies in the cowl closure; the "
                                          "spinner/flange region holds no engine material wider than 0.10 m"}
    if not out["cylinder_slab"]["fits_with_10mm"] or not out["intake_box_below_crank"]["fits_with_10mm"]:
        flag("engine envelope does not fit the OML with 10 mm clearance")
    tk = ac.layout["tank"]
    out["fuel_cells"] = {"required_m3": tk["volume_required_m3"], "available_m3": tk["volume_available_m3"],
                         "forward_cell_x_m": [tk["cells"]["forward"]["x0"], tk["cells"]["forward"]["x1"]],
                         "aft_cell_x_m": [tk["cells"]["aft"]["x0"], tk["cells"]["aft"]["x1"]],
                         "fuel_cg_x_m": tk["x_c"]}
    xb0, xb1 = tk["bay"]
    out["payload_bay"] = {"x_m": [xb0, xb1], "volume_m3": 0.8 * internal_area(0.5 * (xb0 + xb1)) * (xb1 - xb0),
                          "internal_area_m2": internal_area(0.5 * (xb0 + xb1))}
    w, h, *_ = (float(v[0]) for v in fus_section(X_TURRET))
    out["turret_bay"] = {"x_m": X_TURRET, "fuselage_width_m": w, "fuselage_height_m": h,
                         "growth_envelope_m": [TUR_D_GROWTH, TUR_H_GROWTH],
                         "fits": bool(w - 0.02 >= TUR_D_GROWTH and h >= TUR_H_GROWTH - 0.13)}
    out["parachute_bay"] = {"x_m": list(X_CHUTE), "container_m": CHUTE_BOX,
                            "fits": bool(all(inside_oml(x, sy * (CHUTE_BOX[0] / 2 + 0.01), z, 0.01)
                                             for x in (X_CHUTE[0], X_CHUTE[1]) for sy in (1, -1)
                                             for z in (0.24 - CHUTE_BOX[1] - 0.03, 0.15))),
                            "x_container_length_m": X_CHUTE[1] - X_CHUTE[0]}
    out["cooling_exit_annulus_m2"] = math.pi / 4 * (float(fus_section(X_LIP)[0][0]) * 0.97) ** 2 - \
        math.pi / 4 * SPINNER["d"] ** 2
    out["prop_to_cowl_lip_axial_m"] = X_PROP - 0.02 - X_LIP
    out["prop_to_tail_te_axial_m"] = X_PROP - (ac.tail_secs[0]["x_le"] + ac.tail_secs[0]["chord"])
    if out["prop_to_cowl_lip_axial_m"] < CLEAR_PROP_LONG:
        flag("propeller longitudinal clearance to the cowl lip below 13 mm (CS-VLA 925(c)(2))")
    return out


# =====================================================================================================================
# 17. main
# =====================================================================================================================
def main():
    global TAIL_SLOPE_OVERRIDE, BSFC_SCALE
    import time
    t0 = time.time()
    base = Design()
    quick = "--quick" in sys.argv
    if quick:      # development shortcut: skip trades 1-2 (not used for the published results)
        trades = {"aft_stretch": {"chosen_m": 0.40}, "aspect_ratio_wing_loading": {"chosen": {
            "AR": 15.0, "ws_Pa": ws_stall_corner(base, 15.0, 0.40)}}}
    else:
        trades = run_trades(base)
    st = trades["aft_stretch"]["chosen_m"]
    ch = trades["aspect_ratio_wing_loading"]["chosen"]
    d = replace(base, AR=ch["AR"], ws=ch["ws_Pa"])

    print("\n[final] design point with the lifting-line tail slope")
    r0 = evaluate(d, st)
    sa_t = AE.surface_analysis(r0["ac"].tail_secs, v_ref_loiter(), H_LOITER)
    TAIL_SLOPE_OVERRIDE = sa_t["CL_alpha"]
    res = evaluate(d, st, verbose=True)
    ac, aero, mis = res["ac"], res["aero"], res["mission"]
    m0, fuel = res["m0"], res["fuel"]
    lay = ac.layout
    if not quick:
        trades.update(run_config_trades(d, st, res))
    set_fuselage(st)

    # closed MassModel loop (sizinglib): must return the design MTOM for the mission flown above
    mm = SZ.MassModel(PAYLOAD, fixed_mass(d) * (1 + GROWTH), mis["ff"], airframe_fn_factory(ac, aero))
    mm_sol = mm.solve(m0_guess=120.0)
    print(f"\n[mass model] MTOW from sizinglib.MassModel = {mm_sol['mtow']:.2f} kg (design {m0:.2f} kg)")

    # baseline 10 h mission -> MTOW (closed loop, geometry re-sized)
    print("[10 h mission] closed-loop MTOW for the baseline 10 h requirement")
    r10 = evaluate(d, st, mode="endurance", endurance_target_h=END_REQ_H, verbose=True)
    TAIL_SLOPE_OVERRIDE = sa_t["CL_alpha"]
    set_fuselage(st)
    res = evaluate(d, st)                      # restore the design-point aircraft (globals)
    ac, aero, mis = res["ac"], res["aero"], res["mission"]
    lay = ac.layout

    rng = solve_range(ac, m0, fuel)
    perf = performance(ac, m0, fuel, mis)
    cdiag = constraint_diagram(ac, m0, perf)
    VH_level = perf[0]["V_max_m_s"]
    VD = max(1.25 * VC_EAS, VH_level, BL["design_speeds_m_per_s_eas"]["VD"]["value"])
    vn = vn_summary(ac, m0, VD)
    pk = packaging_checks(ac)

    # stability details
    t = lay["tail"]
    tv = SL.tail_volumes(ac.S, ac.pf["mac"], ac.pf["b"], t["S_h_eff"], lay["l_h"], t["S_v_eff"], lay["l_h"])
    fp = fuselage_mesh_props()
    cnb_fin = SL.cn_beta_vertical(TAIL_SLOPE_OVERRIDE, t["S_v_eff"], lay["l_h"], ac.S, ac.pf["b"], 0.9, 0.1)
    cnb_fus_raymer = -1.3 * fp["volume"] / (ac.S * ac.pf["b"]) * (fp["h_max"] / fp["w_max"])
    cnb_fus_datcom = SL.cn_beta_fuselage(0.0015, 1.75, fp["S_side"], L_FUS, ac.S, ac.pf["b"])
    vproj = SL.v_tail_projection(t["S_V"], t["gamma_deg"])

    # sensitivities (endurance at the same aircraft)
    sens = {}
    for k_, scale in (("bsfc_minus_12pct", 0.88), ("bsfc_plus_12pct", 1.12)):
        BSFC_SCALE = scale
        sens[k_] = solve_loiter_for_fuel(ac, m0, fuel)["t_air_s"] / 3600
    BSFC_SCALE = 1.0
    CLs, CDs = ac.cd_table
    ac.cd_table = (CLs, CDs + 0.1 * ac.fit["cd0"])
    sens["cd0_plus_10pct"] = solve_loiter_for_fuel(ac, m0, fuel)["t_air_s"] / 3600
    ac.cd_table = (CLs, CDs)
    sens["empty_plus_5pct"] = solve_loiter_for_fuel(ac, m0, fuel - 0.05 * res["empty"])["t_air_s"] / 3600
    sens["loiter_at_1000m"] = solve_loiter_for_fuel(ac, m0, fuel, h=1000.0)["t_air_s"] / 3600
    sens["no_transit_loiter_only"] = solve_loiter_for_fuel(ac, m0, fuel, R_transit=0.0)["t_air_s"] / 3600
    # max-endurance variant: baseline sensor set only (3.25 kg), the research allowance becomes an auxiliary bladder in
    # the payload bay (on the CG); capacity = K_TANK x bay internal volume
    tkb = lay["tank"]["bay"]
    aux_cap = K_TANK * internal_area(0.5 * (tkb[0] + tkb[1])) * (tkb[1] - tkb[0]) * RHO_FUEL / 1.02
    aux_fuel = min(M_RESEARCH, aux_cap)
    r_aux = solve_loiter_for_fuel(ac, m0, fuel + aux_fuel)
    sens["max_endurance_baseline_sensors_aux_bladder"] = r_aux["t_air_s"] / 3600
    sens["aux_bladder_fuel_kg"] = aux_fuel
    sens["aux_bladder_capacity_kg"] = aux_cap

    # electrical margin and engine band at loiter
    loi3 = perf[int(H_LOITER)]["loiter"]
    loi0 = perf[0]["loiter"]
    if loi0["rpm"] < 4000:
        flag(f"sea-level loiter at {loi0['rpm']:.0f} rpm is below the Limbach 4000-6000 rpm regular-flight band "
             "(3000 m loiter is inside it); confirm idle/low-load operation with Limbach or loiter at >= 1500 m")
    if perf[0]["RoC_max_m_s"] < ROC_REQ:
        flag(f"sea-level rate of climb {perf[0]['RoC_max_m_s']:.2f} m/s is below the 4.9 m/s baseline target with the "
             "32x18 2B propeller (prop-limited, not engine-limited); the 31x12 3B option trades endurance for climb")
    for case, lp in (("3000 m", loi3), ("sea level", loi0)):
        if lp["gen_W"] < P_ELEC * 1.2:
            flag(f"generator margin at the {case} loiter only {lp['gen_W'] / P_ELEC:.2f}x the 303 W load")
    if min(loi0["power_fraction"], loi3["power_fraction"]) < BSFC_PTS[0, 0]:
        flag(f"loiter power fraction {min(loi0['power_fraction'], loi3['power_fraction']):.3f} is below the lowest "
             "BSFC point (0.20): BSFC linearly extrapolated (+60 g/kWh per -0.1); a light-load dyno map is needed")
    if res["empty"] / m0 > 0.62:
        flag(f"empty-mass fraction {res['empty'] / m0:.3f} (incl. 5 % growth allowance) is above the comparables range "
             "0.55-0.62; without the allowance it is " + f"{res['eb']['empty'] / (1 + GROWTH) / m0:.3f}")
    flag("flutter, wing torsional stiffness and aileron reversal of the AR "
         f"{ac.pf['AR']:.0f} wing are not analysed here (CS-LUAS.629 clearance to 1.2 VD = {1.2 * VD:.1f} m/s)")
    flag("aft-body closure from 0.44 m to the 0.30 m cowl lip in 0.10 m relies on the pusher inflow (prop suction) "
         "and the cooling exit flow; confirm with CFD; the 80 mm prop hub spacer needs Limbach approval")
    flag("lift-off is flat (ground attitude, 15 deg take-off flap) because the tail cannot rotate against the high thrust "
         f"line below {perf['takeoff_sl_mtow']['V_rotation_authority_m_s']:.1f} m/s; the autopilot take-off law must "
         "hold the nose wheel down until the wing lifts the aircraft")

    # ---------------------------------------------------------------- console table
    P = perf
    L_tot = SPINNER["x1"]
    rows = [
        ("MTOM design / cap", f"{m0:.1f} / {MTOM_CAP:.1f} kg (sizinglib.MassModel: {mm_sol['mtow']:.2f} kg)"),
        ("Empty / fuel / payload", f"{res['empty']:.1f} / {fuel:.1f} / {PAYLOAD:.1f} kg (empty frac {res['empty'] / m0:.3f})"),
        ("MTOW for the 10 h mission", f"{r10['m0']:.1f} kg (fuel {r10['fuel']:.1f} kg)"),
        ("Wing S / b / AR / taper", f"{ac.S:.3f} m2 / {ac.pf['b']:.2f} m / {ac.pf['AR']:.1f} / {ac.pf['taper']:.2f}"),
        ("Chords root / tip / MAC", f"{ac.pf['cr']:.3f} / {ac.pf['ct']:.3f} / {ac.pf['mac']:.3f} m"),
        ("W/S", f"{d.ws:.0f} Pa = {d.ws / G:.1f} kg/m2"),
        ("Length (OML / overall)", f"{L_FUS:.2f} / {L_tot:.2f} m, width {lay['w_fus']:.2f} m"),
        ("V-tail S / dihedral / l_h", f"{t['S_V']:.3f} m2 / {t['gamma_deg']:.1f} deg / {lay['l_h']:.2f} m"),
        ("V_H / V_V", f"{tv['V_H']:.3f} / {tv['V_V']:.4f}"),
        ("CD0 / e / k / (L/D)max", f"{ac.fit['cd0']:.4f} / {ac.fit['e']:.3f} / {ac.fit['k']:.4f} / {ac.fit['LD_max']:.2f}"),
        ("CL_alpha / e_inv / CLmax clean trimmed", f"{aero['CL_alpha']:.3f} /rad / {aero['e_inv']:.3f} / "
                                                   f"{ac.clmax['clean_trimmed']:.3f} (TO flap {ac.clmax['to_trimmed']:.3f})"),
        ("Stall onset eta", f"{aero['stall_eta']:.2f}"),
        ("VS clean / TO flap (SL, MTOM)", f"{P[0]['VS_clean_m_s']:.1f} / {P['takeoff_sl_mtow']['VS_TO_m_s']:.1f} m/s"),
        ("Loiter 3000 m (EAS/TAS, ff)", f"{loi3['EAS']:.1f} / {loi3['V']:.1f} m/s, {loi3['ff_kg_h']:.2f} kg/h, "
                                        f"{loi3['rpm']:.0f} rpm, PF {loi3['power_fraction']:.3f}"),
        ("V min power / V (L/D)max (SL, polar)", f"{P[0]['V_min_power_polar_m_s']:.1f} / {P[0]['V_LDmax_polar_m_s']:.1f} m/s"),
        ("Best-range speed 3000 m", f"{P[int(H_LOITER)]['best_range']['V']:.1f} m/s"),
        ("Vmax SL / 3000 m", f"{P[0]['V_max_m_s']:.1f} / {P[int(H_LOITER)]['V_max_m_s']:.1f} m/s"),
        ("Endurance (mission, 10 % reserve)", f"{res['endurance_h']:.2f} h (loiter {mis['t_loiter_s'] / 3600:.2f} h)"),
        ("Range (ferry, 3000 m, 10 % reserve)", f"{rng['range_m'] / 1000:.0f} km"),
        ("Max endurance (3.25 kg sensors + aux fuel)", f"{sens['max_endurance_baseline_sensors_aux_bladder']:.1f} h "
                                                       f"(+{sens['aux_bladder_fuel_kg']:.1f} kg fuel in the payload bay)"),
        ("Endurance sensitivity", f"BSFC -12/+12 %: {sens['bsfc_minus_12pct']:.1f}/{sens['bsfc_plus_12pct']:.1f} h, "
                                  f"CD0 +10 %: {sens['cd0_plus_10pct']:.1f} h, empty +5 %: {sens['empty_plus_5pct']:.1f} h"),
        ("RoC SL / 3000 m", f"{P[0]['RoC_max_m_s']:.2f} / {P[int(H_LOITER)]['RoC_max_m_s']:.2f} m/s"),
        ("Ceiling service / absolute", f"{P['ceiling_service_m']:.0f} / {P['ceiling_absolute_m']:.0f} m"),
        ("TO roll / to 15 m (SL, MTOM)", f"{P['takeoff_sl_mtow']['ground_roll_m']:.0f} / "
                                         f"{P['takeoff_sl_mtow']['distance_15m_m']:.0f} m ({P['takeoff_sl_mtow']['liftoff_mode']})"),
        ("Landing roll end-of-mission / MTOM", f"{P['landing_sl_end_of_mission']['ground_roll_m']:.0f} / "
                                               f"{P['landing_sl_mtow']['ground_roll_m']:.0f} m"),
        ("Static margin (all cases)", f"{min(lay['sms']):.3f} - {max(lay['sms']):.3f}"),
        ("CG x (all cases) / NP x", f"{min(c['x'] for c in lay['cases']):.3f}-{max(c['x'] for c in lay['cases']):.3f} / "
                                    f"{lay['x_np']:.3f} m"),
        ("Cn_beta fin / fuselage / total", f"{cnb_fin:.3f} / {t['cnb_fus']:.3f} / {cnb_fin + t['cnb_fus']:.3f} /rad"),
        ("Prop clearance level / LOF / 7 deg", f"{lay['gear']['prop_clear_level']:.3f} / {lay['gear']['prop_clear_lof']:.3f}"
                                               f" / {lay['gear']['prop_clear_design']:.3f} m"),
        ("Static tip Mach / speed", f"{P['static_wot']['tip_mach']:.3f} / {P['static_tip_speed_m_s']:.0f} m/s"),
        ("Gust limit n (VC 45, 4500 m)", f"{lay['gust']['n_limit']:.2f}, ultimate {FOS * lay['gust']['n_limit']:.2f}"),
    ]
    print("\n" + "=" * 100)
    print("YK-250 concept 'endurance' (MALE-type) - results")
    print("=" * 100)
    for k_, v_ in rows:
        print(f"  {k_:<40s} {v_}")
    print("  flags:")
    for f_ in FLAGS:
        print("   -", f_)

    # ---------------------------------------------------------------- concept.yaml
    wm = lay["wing_mac"]
    tm = t["macd"]
    gear = lay["gear"]
    y = {
        "meta": {"project": "YK-250 (ucav250)", "concept_key": "endurance", "name_tr": "Dayanım (MALE tipi)",
                 "phase": "concept trade study - first-pass sizing", "date": "2026-10-05",
                 "generated_by": "ucav250/data/concepts/endurance/calc.py",
                 "run": "PYTHONPATH=. python3 ucav250/data/concepts/endurance/calc.py",
                 "frame": "X aft from the nose tip, Y starboard, Z up from the fuselage datum line through the nose tip; "
                          "SI units, angles in degrees",
                 "scope": "civil EO/IR surveillance and research platform with a UCAV/MALE look; payload is sensors and "
                          "mission equipment only - no weapons, hardpoints, pylons or release mechanisms",
                 "libraries": ["analysis/aerolib.py", "analysis/stablib.py", "analysis/sizinglib.py",
                               "analysis/structlib.py", "analysis/aero.py", "design/oml.py"],
                 "runtime_s": time.time() - t0},
        "configuration": {
            "propulsion": "single Limbach L 275 EF pusher, Mejzlik 32x18 2B on an 80 mm hub spacer, annular cooling "
                          "exit around the spinner",
            "tail": f"upright V-tail ({t['gamma_deg']:.0f} deg dihedral, NACA 0012, ruddervators) + ventral bumper skid",
            "wing_position": "shoulder (on the fuselage top, saddle fairing), two halves joined on the centre line",
            "gear": "fixed tricycle: GFRP spring bow + faired TOST 200x50 wheels, steerable faired nose leg",
            "fuselage": "slim single fuselage 0.44 m wide, chined lower half, upswept tail cone to the engine bay",
            "planform": f"straight taper, AR {ac.pf['AR']:.1f}, taper {ac.pf['taper']:.2f}, unswept quarter chord, "
                        f"NLF(1)-0416 16 % root to 13 % tip, {d.washout:.1f} deg washout, inboard take-off flaps",
            "layout": "payload bay on the CG between two interconnected fuel cells; EO/IR turret in the chin; battery, "
                      "avionics and parachute forward; engine aft",
        },
        "summary": {
            "mtow_kg": m0, "empty_kg": res["empty"], "fuel_kg": fuel, "payload_kg": PAYLOAD,
            "endurance_h": res["endurance_h"], "range_km": rng["range_m"] / 1000, "span_m": ac.pf["b"],
            "length_m": L_tot, "area_m2": ac.S, "AR": ac.pf["AR"], "ld_max": ac.fit["LD_max"],
            "mtow_for_10h_mission_kg": r10["m0"],
            "variant_10h_mission": {"mtow_kg": r10["m0"], "fuel_kg": r10["fuel"], "empty_kg": r10["empty"],
                                    "S_m2": r10["ac"].S, "span_m": r10["ac"].pf["b"],
                                    "endurance_h": r10["endurance_h"],
                                    "basis": "same W/S, AR and layout; MTOW from sizinglib.MassModel with the 10 h "
                                             "mission fuel fraction (geometry re-sized each iteration)"},
            "max_endurance_baseline_sensors_aux_bladder_h": sens["max_endurance_baseline_sensors_aux_bladder"],
            "feasible": True,
        },
        "inputs": INPUTS,
        "trades": trades,
        "geometry": {
            "wing": {"S_m2": ac.S, "b_m": ac.pf["b"], "AR": ac.pf["AR"], "taper": ac.pf["taper"],
                     "root_chord_m": ac.pf["cr"], "tip_chord_m": ac.pf["ct"], "mac_m": wm["mac"],
                     "mac_le_x_m": wm["x_le_mac"], "mac_y_m": wm["y_mac"], "x_le_root_m": lay["x_w"],
                     "z_le_root_m": Z_WING_ROOT, "incidence_deg": lay["i_w"], "washout_deg": d.washout,
                     "dihedral_deg": d.dihedral, "incidence_rule": lay["incidence_rule"],
                     "sections": ac.wing_secs, "airfoils": {"root": "nlf416 (NASA NLF(1)-0416)",
                                                            "tip": "nlf416 thickness_scale 0.8125 (13 %)"},
                     "flaps": {"type": "plain, inboard, take-off only", "span_eta": [lay["w_fus"] / ac.pf["b"],
                                                                                    d.flap_span_frac],
                               "chord_fraction": d.cf_c, "takeoff_deg": d.flap_to_deg,
                               "effect": ac.flap_to},
                     "ailerons": {"span_eta": [0.57, 0.95], "chord_fraction": d.cf_c},
                     "S_wet_m2": ac.mass["wing"]["S_wet"]},
            "tail": {"type": "upright V", "S_panels_total_m2": t["S_V"], "dihedral_deg": t["gamma_deg"],
                     "S_h_eff_m2": t["S_h_eff"], "S_v_eff_m2": t["S_v_eff"], "v_tail_projection_check": vproj,
                     "panel_span_m": t["span_panel"], "root_chord_m": t["cr"], "tip_chord_m": t["ct"],
                     "le_sweep_deg": d.tail_le_sweep, "aspect_ratio_unfolded": d.tail_AR, "mac_m": tm["mac"],
                     "mac_le_x_m": tm["x_le_mac"], "x_ac_m": lay["x_ac_t"], "airfoil": "n0012 (NACA 0012)",
                     "ruddervator_chord_fraction": 0.30, "sections": ac.tail_secs,
                     "tip_to_tip_span_m": t["b_proj"], "S_wet_m2": ac.mass["tail"]["S_wet"]},
            "fuselage": {"format": "oml.Fuselage stations [x, w, h, zc, n_top, n_bot, top_frac]",
                         "stations": FUS, "length_m": L_FUS, "width_max_m": fp["w_max"], "height_max_m": fp["h_max"],
                         "S_wet_m2": fp["S_wet"], "volume_m3": fp["volume"], "side_area_m2": fp["S_side"],
                         "aft_stretch_m": st},
            "propulsion": {"engine": "Limbach L 275 EF", "prop_hub_face_x_m": X_HUB, "thrust_line_z_m": Z_T,
                           "hub_spacer_m": HUB_SPACER, "prop_plane_x_m": X_PROP, "cowl_lip_x_m": X_LIP,
                           "spinner": SPINNER, "propeller": {"model": "Mejzlik 32x18 2B (pusher hand)", "D_m": D_PROP,
                                                             "mass_kg": M_PROP},
                           "cooling_exit_annulus_m2": pk["cooling_exit_annulus_m2"]},
            "landing_gear": {k: gear[k] for k in ("x_mg", "z_g", "track", "wheelbase", "h_gear", "h_nose_leg",
                                                  "tipback_deg", "turnover_deg", "nose_load_aft_cg", "nose_load_fwd_cg",
                                                  "prop_clear_level", "prop_clear_lof", "prop_clear_design",
                                                  "theta_lof_deg", "theta_td_deg", "theta_design_deg",
                                                  "prop_strike_deg", "bumper_contact_deg", "x_bumper", "z_bumper",
                                                  "turret_clear", "fuselage_bottom_height_at_mg", "active_height_rule")},
            "bays": {"turret_chin_x_m": X_TURRET, "battery_pdu_x_m": X_BATT, "avionics_x_m": X_AVION,
                     "parachute_x_m": list(X_CHUTE), "nose_gear_x_m": X_NG, "payload_bay_x_m": list(lay["tank"]["bay"]),
                     "fuel_cells": lay["tank"]["cells"]},
            "overall": {"length_m": L_tot, "span_m": ac.pf["b"], "height_m": ac.tail_secs[1]["z_le"] - gear["z_g"],
                        "ground_z_m": gear["z_g"]},
        },
        "aero": {"drag_items_D_over_q_m2": ac.drag_items, "cd_items": {k: v / ac.S for k, v in ac.drag_items.items()},
                 "cd_rest_non_wing": ac.fit["cd_rest"], "cd0": ac.fit["cd0"], "k": ac.fit["k"], "e": ac.fit["e"],
                 "e_inviscid_lifting_line": aero["e_inv"], "e_nita_scholz": ac.fit["e_nita_scholz"],
                 "e_raymer_straight": ac.fit["e_raymer"], "ld_max": ac.fit["LD_max"], "cl_ld_max": ac.fit["CL_LDmax"],
                 "cl_best_endurance": ac.fit["CL_endurance"], "CL_alpha_per_rad": aero["CL_alpha"],
                 "CL_0_at_fuselage_level": aero["CL_0"], "cm0_tripped": aero["cm0"],
                 "clmax": ac.clmax, "alpha_stall_deg": aero["alpha_stall_deg"], "stall_onset_eta": aero["stall_eta"],
                 "Re_root": aero["Re_root"], "Re_tip": aero["Re_tip"], "neuralfoil_min_confidence": aero["min_conf"],
                 "polar_basis": "tripped section polars (x_tr 0.075 c) x 1.15, strip-integrated on the lifting-line cl, "
                                "+ aerolib component build-up, trimmed (tail load + thrust moment), at the 3000 m loiter "
                                "speed; parabolic fit over CL 0.25-1.25",
                 "polar_table": {"CL": ac.cd_table[0], "CD": ac.cd_table[1]},
                 "clean_profile_drag_at_cl_0p9_upside": aero["cdp_clean"](0.9),
                 "tripped_profile_drag_at_cl_0p9": aero["cdp"](0.9) * K_CD_TRIP,
                 "tail_CL_alpha_lifting_line_per_rad": sa_t["CL_alpha"]},
        "mass": {"mtow_kg": m0, "empty_kg": res["empty"], "fuel_kg": fuel, "payload_kg": PAYLOAD,
                 "fixed_equipment_kg": fixed_mass(d), "airframe_kg": res["eb"]["airframe"],
                 "growth_allowance_kg": res["eb"]["growth"],
                 "groups_kg": ac.mass["groups"], "wing_breakdown": ac.mass["wing"], "tail_breakdown": ac.mass["tail"],
                 "fuselage_breakdown": {k: v for k, v in ac.mass["fus"].items()},
                 "chassis_items": {k: v[0] for k, v in CHASSIS.items()},
                 "empty_fraction": res["empty"] / m0,
                 "empty_fraction_without_growth": res["eb"]["empty"] / (1 + GROWTH) / m0,
                 "comparables_empty_fraction": CP["recommended_ranges"]["empty_mass_fraction"],
                 "fuel_fraction": fuel / m0, "mission_fuel_fraction_sizinglib": mis["ff"],
                 "massmodel": {"mtow_kg": mm_sol["mtow"], "fuel_kg": mm_sol["fuel"], "airframe_kg": mm_sol["airframe"],
                               "fixed_kg": mm_sol["fixed"], "payload_kg": mm_sol["payload"]},
                 "items": [{"name": n, "mass_kg": mm_, "x_m": x_, "z_m": z_} for n, mm_, x_, z_ in lay["items"]],
                 "cases": lay["cases"], "mtow_margin_to_cap_kg": MTOM_CAP - m0},
        "stability": {"mac_m": ac.pf["mac"], "mac_le_x_m": wm["x_le_mac"], "x_ac_wing_m": lay["x_ac_w"],
                      "np_x_m": lay["x_np"], "cg_design_m": [lay["cases"][0]["x"], 0.0, lay["cases"][0]["z"]],
                      "cg_range_x_m": [min(c["x"] for c in lay["cases"]), max(c["x"] for c in lay["cases"])],
                      "static_margin_range": [min(lay["sms"]), max(lay["sms"])],
                      "static_margin_by_case": {c["name"]: sm_ for c, sm_ in zip(lay["cases"], lay["sms"])},
                      "V_H": tv["V_H"], "V_V": tv["V_V"], "l_h_m": lay["l_h"], "deps_dalpha": lay["deps_da"],
                      "cm_alpha_fuselage_per_rad": lay["cm_alpha_fus"], "kf_fuselage_per_rad": lay["kf"],
                      "a_tail_per_rad": TAIL_SLOPE_OVERRIDE, "eta_tail": 0.9,
                      "cn_beta_fin_per_rad": cnb_fin, "cn_beta_fuselage_raymer_per_rad": cnb_fus_raymer,
                      "cn_beta_fuselage_datcom_per_rad": cnb_fus_datcom,
                      "cn_beta_total_per_rad": cnb_fin + min(cnb_fus_raymer, cnb_fus_datcom),
                      "cn_beta_requirement_per_rad": d.cnb_req,
                      "note": "pusher propeller normal force (stabilising) neglected; DATCOM K_N 0.0015 / K_Rl 1.75 "
                              "are estimates, the more destabilising fuselage value is used"},
        "performance": {**{str(k): v for k, v in perf.items()}, "endurance_h": res["endurance_h"],
                        "loiter_time_h": mis["t_loiter_s"] / 3600, "range_km": rng["range_m"] / 1000,
                        "mission_log": mis["log"], "range_mission_fuel_fraction": rng["ff"],
                        "VD_m_s": VD, "VC_m_s": VC_EAS},
        "constraint_diagram": {"ws_grid_Pa": cdiag["ws_grid"], "curves_W_per_N": cdiag["curves"],
                               "ws_stall_24_Pa": cdiag["ws_stall_24"], "ws_landing_200_Pa": cdiag["ws_landing_200"],
                               "ws_loiter_floor_Pa": cdiag["ws_loiter_floor_1p2VS"],
                               "pw_available_mcp_W_per_N": cdiag["pw_available_mcp"],
                               "pw_available_max_W_per_N": cdiag["pw_available_max"], "ws_design_Pa": cdiag["ws_design"],
                               "pw_prop_absorbed_wot_climb_W_per_N": cdiag["pw_prop_absorbed_wot_climb"],
                               "pw_required_at_design_W_per_N": cdiag["pw_required_at_design"],
                               "sizinglib_min_power_point": {k: v for k, v in cdiag["design_point_min_power"].items()
                                                             if k != "envelope"},
                               "eta_takeoff": cdiag["eta_takeoff"], "eta_climb": cdiag["eta_climb"],
                               "note": "altitude curves corrected from sizinglib's Gagg-Ferrar lapse to sigma^1.23; "
                                       "the design W/S is set by the endurance trade, not the minimum-power point"},
        "structures": {"vn": vn, "n_limit_design": lay["gust"]["n_limit"], "n_ultimate_wing": FOS * lay["gust"]["n_limit"],
                       "wing_root_bending_ultimate_N_m": ac.mass["wing"]["M_root_ult_Nm"],
                       "spar_cap_area_root_mm2": ac.mass["wing"]["A_cap_root_mm2"],
                       "spar_depth_root_m": ac.mass["wing"]["h_eff_root_m"], "spar_cap_allowable_Pa": SIG_CAP,
                       "standards": "JARUS CS-LUAS / STANAG 4703: +3.8/-1.5 g, FoS 1.5, gust 15.24 m/s at VC"},
        "checks": pk,
        "sensitivities_endurance_h": sens,
        "electrical": {"load_W": P_ELEC, "generator_W_loiter_3000m": loi3["gen_W"], "generator_W_loiter_sl": loi0["gen_W"],
                       "margin_3000m": loi3["gen_W"] / P_ELEC, "margin_sl": loi0["gen_W"] / P_ELEC},
        "flags": FLAGS,
        "files": ["ucav250/data/concepts/endurance/calc.py", "ucav250/data/concepts/endurance/concept.yaml",
                  "ucav250/data/concepts/endurance/sketch.png", "ucav250/data/concepts/endurance/constraint.png",
                  "ucav250/data/concepts/endurance/notes_tr.md"],
    }
    out = py(y)
    with open(HERE / "concept.yaml", "w", encoding="utf-8") as f:
        f.write("# YK-250 concept study 'endurance' (MALE-type) - generated by calc.py, do not edit by hand\n")
        yaml.safe_dump(out, f, sort_keys=False, allow_unicode=True, width=120)
    res_for_plot = {"m0": m0, "empty": res["empty"], "fuel": fuel, "endurance_h": res["endurance_h"],
                    "range_km": rng["range_m"] / 1000, "VH": tv["V_H"], "VV": tv["V_V"]}
    draw_sketch(ac, perf, res_for_plot, HERE / "sketch.png")
    draw_constraints(cdiag, HERE / "constraint.png")
    print(f"\nwrote {HERE / 'concept.yaml'}, sketch.png, constraint.png  ({time.time() - t0:.0f} s)")


if __name__ == "__main__":
    main()
