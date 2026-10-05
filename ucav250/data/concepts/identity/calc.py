#!/usr/bin/env python3
"""YK-250 concept study "identity": the strongest UCAV identity that still meets the mission.

Run from the repository root:

    PYTHONPATH=. python3 ucav250/data/concepts/identity/calc.py

Outputs (this folder only): concept.yaml (every input and result; units in the key names; a source or basis for
every number), sketch.png (3-view drawn from the actual OML meshes), constraint.png (constraint diagram) and
polars/ (NeuralFoil cache for Reynolds numbers and tripped polars that are not yet in ucav250/data/polars; the
shared cache is read but never written).

Method (first-pass sizing, closed loop, SI units):
    m0 -> geometry (wing from W/S and AR, tail from volume coefficients, fuselage generated around the packaging
    keep-outs, wing placed for the static-margin target) -> drag build-up (aerolib) + lifting line on the actual
    wing sections (aero.surface_analysis, tripped polars x 1.15) -> mission segments (sizinglib) with the
    Mejzlik 32x18 2B table and the part-load BSFC of the Limbach L 275 EF at the actual power setting
    -> sizinglib.MassModel(payload + fixed equipment + fuel fraction + airframe fraction(m0)) -> m0, repeated
    until m0 changes by less than 0.02 kg.
Frame: X aft from the nose tip (FS), Y starboard, Z up; Z = 0 on the chine line of the centre fuselage.

Engineering estimates are named ``EST_*`` or carry a ``basis`` string next to the number; values read from the
research files keep their file/key reference in ``SRC`` and in concept.yaml.
"""
from __future__ import annotations

import copy
import functools
import json
import math
import sys
import time
from dataclasses import dataclass, field, replace
from pathlib import Path

import numpy as np
import yaml
from scipy.optimize import minimize

HERE = Path(__file__).resolve().parent
REPO = HERE.parents[3]
if str(REPO) not in sys.path:
    sys.path.insert(0, str(REPO))

from ucav250.analysis import aero as AE          # noqa: E402
from ucav250.analysis import aerolib as AL       # noqa: E402
from ucav250.analysis import sizinglib as SZ     # noqa: E402
from ucav250.analysis import stablib as SL       # noqa: E402
from ucav250.analysis import structlib as ST     # noqa: E402
from ucav250.design import oml                   # noqa: E402

G = AL.G0
RES = REPO / "ucav250" / "data" / "research"


def _load(name: str) -> dict:
    return yaml.safe_load((RES / name).read_text(encoding="utf-8"))


BL = _load("baseline.yaml")
ENGY = _load("engine.yaml")
AEY = _load("aero.yaml")
CMPY = _load("components.yaml")
COMPY = _load("comparables.yaml")


def val(node):
    return node["value"] if isinstance(node, dict) and "value" in node else node


SRC = {
    "baseline": "ucav250/data/research/baseline.yaml",
    "engine": "ucav250/data/research/engine.yaml",
    "aero": "ucav250/data/research/aero.yaml",
    "components": "ucav250/data/research/components.yaml",
    "comparables": "ucav250/data/research/comparables.yaml",
    "materials": "ucav250/data/research/materials.yaml",
    "standards": "ucav250/data/research/standards.yaml",
    "raymer": "Raymer, Aircraft Design: A Conceptual Approach, 6th ed. (2018) - textbook value as recalled, verify",
    "hoerner": "Hoerner, Fluid-Dynamic Drag (1965) - textbook value as recalled, verify",
    "this": "computed in ucav250/data/concepts/identity/calc.py",
}

# =====================================================================================================================
# 0. NeuralFoil polar cache: read the shared cache, write new polars only into this folder; tripped polars
# =====================================================================================================================
LOCAL_POLARS = HERE / "polars"
XTR_TRIP = 0.075            # aero.yaml#method.polars.cases.tripped: transition fixed at x/c 0.075 (NASA TP-1861 trip)
TRIPPED = -1.0              # sentinel n_crit value -> tripped polar
_shared_polar_path = AE._polar_path
_shared_raw_polar = AE.raw_polar


def _polar_path(airfoil, Re, n_crit, ts=1.0):
    p = _shared_polar_path(airfoil, Re, n_crit, ts)
    if p.exists():
        return p
    LOCAL_POLARS.mkdir(parents=True, exist_ok=True)
    return LOCAL_POLARS / p.name


@functools.lru_cache(maxsize=None)
def _raw_polar(airfoil, Re, n_crit=9.0, thickness_scale=1.0):
    if n_crit >= 0:
        return _shared_raw_polar(airfoil, Re, n_crit, thickness_scale)
    tag = "" if abs(thickness_scale - 1.0) < 1e-9 else f"_t{thickness_scale:.4f}"
    path = LOCAL_POLARS / f"{airfoil.lower()}{tag}_Re{int(round(Re)):d}_trip{XTR_TRIP:g}.json"
    if path.exists():
        return json.loads(path.read_text())
    import neuralfoil as nf
    P = AE.section_coords(airfoil, thickness_scale)
    a = np.array(AE.ALPHAS)
    r = nf.get_aero_from_coordinates(P, alpha=a, Re=float(Re), n_crit=9.0, xtr_upper=XTR_TRIP, xtr_lower=XTR_TRIP,
                                     model_size="xlarge")
    out = {"airfoil": airfoil, "thickness_scale": float(thickness_scale), "Re": float(Re), "n_crit": 9.0,
           "xtr": XTR_TRIP, "alpha": list(AE.ALPHAS),
           "cl": [float(v) for v in r["CL"]], "cd": [float(v) for v in r["CD"]], "cm": [float(v) for v in r["CM"]],
           "confidence": [float(v) for v in r["analysis_confidence"]],
           "xtr_top": [float(v) for v in r["Top_Xtr"]], "xtr_bot": [float(v) for v in r["Bot_Xtr"]],
           "method": f"NeuralFoil xlarge, transition forced at x/c {XTR_TRIP} both surfaces"}
    LOCAL_POLARS.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(out, indent=1))
    return out


AE._polar_path = _polar_path
AE.raw_polar = _raw_polar

# =====================================================================================================================
# 1. research inputs (engine, propeller, mission, payload, systems, aero corrections, loads, materials)
# =====================================================================================================================
ENG = BL["engine"]
P_MAX = val(ENG["power_max_W"])                       # 18000 W at 7500 rpm (manufacturer)
P_TO = val(ENG["power_takeoff_W"])                    # 17150 W at 6950 rpm (paper)
P_MCP = val(ENG["power_max_continuous_W"])            # 16200 W at 6000 rpm (estimate from the WOT curve)
WOT_RPM = np.array(ENG["wot_curve"]["rpm"], float)
WOT_Q = np.array(ENG["wot_curve"]["torque_N_m"], float)
BSFC_PTS = np.array(ENG["bsfc_curve"]["points"], float)          # [power fraction of 18 kW, g/kWh]
_lapse_formula = ENGY["installation"]["altitude"]["power_lapse_model"]["formula"]
assert "1.23" in _lapse_formula, _lapse_formula
LAPSE_EXP = 1.23                                       # engine.yaml: P(h)/P0 = sigma^1.23 (Hirth 4201 fit)
RHO_FUEL = val(ENG["fuel"]["density_kg_per_m3"])       # 740 kg/m3
ENV_ENG = {k: val(v) for k, v in ENG["envelope_m"].items()}
P_ELEC = val(BL["subsystems"]["electrical_load_W"]["continuous_total"])   # 303 W continuous
BATTERY_M = val(BL["subsystems"]["electrical"]["battery"]["mass_kg"])       # 2.43 kg 14S2P LiFePO4
P_ELEC_PEAK_GROWTH = val(BL["subsystems"]["electrical_load_W"]["peak_with_e180_growth"])
ETA_GEN = 0.85              # baseline.yaml#cross_checks.loiter_fuel_flow basis ("303 W / 0.85 generator efficiency")
GEN_RATED_W, GEN_RATED_RPM = val(ENG["generator"]["power_continuous_W"]), ENG["generator"]["power_continuous_W"]["rpm"]

PROPY = BL["propeller"]["primary"]
D_PROP = val(PROPY["diameter_m"])                      # 0.8128 m Mejzlik 32x18 2B
M_PROP = val(PROPY["mass_kg"])
INSTALL = val(PROPY["installation_factor"])            # 0.95 pusher installation factor (estimate)
KNOCK_STATIC = val(PROPY["static_thrust_N_for_sizing"]) / val(PROPY["static_thrust_N"])   # 460/500 = 0.92
TIP_LIMIT = val(PROPY["tip_speed_limit_m_per_s"])      # 255 m/s static helical tip speed
PROP_GROUND_CLEAR = val(BL["propeller"]["ground_clearance_min_m"])          # 0.18 m CS-VLA 925(a)
PROP_RADIAL_CLEAR = val(BL["propeller"]["radial_tip_clearance_to_structure_m"])  # 0.026 m CS-VLA 925(c)(1)
PROP_TABLE = AEY["propeller_tables_mejzlik"]["0161"]["rows_rpm_thrust_N_torque_Nm_power_W"]

MT = BL["mission_targets"]
ENDURANCE_H = val(MT["endurance_h"])                    # 10 h
V_LOITER_EAS = val(MT["loiter_speed_m_per_s_eas"])      # 28 m/s EAS
V_CRUISE_EAS = val(MT["cruise_speed_m_per_s"])          # 33 m/s, flown as EAS (conservative interpretation)
H_CEILING = val(MT["service_ceiling_m"])                # 4500 m
H_OPS = val(MT["operating_altitude_m"])                 # [1000, 3000]
H_LOITER = float(H_OPS[1])                              # design loiter at 3000 m (worse endurance than 1000 m)
RUNWAY = val(MT["runway_length_m"])                     # 300 m
ROC_SL_REQ = val(MT["climb_rate_sea_level_m_per_s"])    # 4.9 m/s
TRANSIT_KM = 100.0          # EST: design radius (line-of-sight C2/video link class); not in baseline -> concept choice
RESERVE_H = 1.0             # baseline "about 10 h with a 10 % reserve" -> 1 h loiter reserve at 1000 m
V_STALL_MAX = 24.0          # comparables.yaml#recommended_ranges.wing_loading_kg_m2 basis (stall <= 24 m/s, clean)
TYRE_SPEED = val(BL["design_speeds_m_per_s_eas"]["tyre_speed_limit"])     # 36 m/s TOST
MTOM_CAP = 150.0            # SHT-IHA Madde 8(1): 150 kg is M3 -> MTOM must stay strictly below

PAY = BL["payload_set"]
PAYLOAD_DESIGN = val(BL["mass_targets"]["payload_design_kg"])           # 20 kg
TURRET_M = val(PAY["eo_ir_turret"]["mass_kg"]) + val(PAY["eo_ir_turret"]["mount_and_isolator_kg"])   # 1.75
TURRET_D, TURRET_H = val(PAY["eo_ir_turret"]["diameter_m"]), val(PAY["eo_ir_turret"]["height_m"])
E180_D, E180_H, E180_M = (val(PAY["turret_bay_growth_envelope"][k]) for k in ("diameter_m", "height_m", "mass_kg"))
MISSION_COMPUTER_M = val(PAY["mission_computer_and_recorder"]["mass_kg"])
PAY_TRAY_M = val(PAY["payload_tray_harness_kg"])
RESEARCH_PAY_M = val(PAY["research_payload_allowance_kg"])              # 16.75 kg
BASE_SENSORS_M = val(PAY["baseline_sensor_set_total_kg"])                # 3.25 kg

SYS = {k: val(v) for k, v in BL["mass_targets"]["systems_rollup_kg"].items() if isinstance(v, dict) and k != "total"}

AEC = BL["aero"]["corrections_for_sizing"]
CLMAX_FACTOR = val(AEC["cl_max_factor"])               # 0.94 NeuralFoil -> experiment
CD_TRIP_FACTOR = val(AEC["cd_factor_tripped"])         # 1.15 on tripped polars (sizing rule)
K_CLMAX_3D = 0.95           # aero.surface_analysis default 2-D -> 3-D / production knock-down (kept on top of 0.94)
CM_AC_WING = -0.10          # aero.yaml#recommendation...trim_drag cm_c4 at cl 0.6 (-0.104 clean, -0.096 tripped)

DS = BL["design_speeds_m_per_s_eas"]
VC, VD = val(DS["VC"]), val(DS["VD"])
DL = BL["design_loads"]
N_POS = val(DL["manoeuvre"]["n_limit_pos"])
N_NEG = val(DL["manoeuvre"]["n_limit_neg_at_VC"])
FOS = val(DL["factors"]["factor_of_safety"])
UDE_C = val(DL["gust"]["Ude_at_VC_m_per_s"])
UDE_D = val(DL["gust"]["Ude_at_VD_m_per_s"])
STRAIN_C_THICK = DL["composites"]["damage_tolerance_strain_limits_microstrain"]["thick_gt_2mm"]["compression"] * 1e-6
A_BASIS = val(BL["materials_to_use"]["a_basis_for_single_load_path"])
SPECIAL_F = DL["composites"]["special_factor_with_ETW_B_basis"]["value"]

MAT = BL["materials_to_use"]
UD = MAT["cfrp_ud_mtm45_as4"]
PW = MAT["cfrp_pw_mtm45_as4"]
PLY_PW = PW["density"] * PW["ply_t"]                    # kg/m2 per cured fabric ply (0.304)
PLY_UD = UD["density"] * UD["ply_t"]
RHO_NOMEX = MAT["cores"]["nomex_hrh10_3p2_48"]["density"]
RHO_R51 = MAT["cores"]["rohacell_51wf"]["density"]
RHO_R71 = MAT["cores"]["rohacell_71wf"]["density"]
EST_FILM = 0.15             # EST kg/m2 epoxy film adhesive per face (0.03 psf catalogue class); not in materials.yaml
EST_PAINT = 0.12            # EST kg/m2 primer + light-grey topcoat (~80 um at ~1.5 g/cm3)
AREAL = {
    # materials.yaml#processes min laminate: primary sandwich face >= 3 fabric plies, secondary >= 2 plies
    "skin_primary": 2 * 3 * PLY_PW + RHO_NOMEX * 6.35e-3 + 2 * EST_FILM + EST_PAINT,
    "skin_secondary": 2 * 2 * PLY_PW + RHO_R51 * 3.0e-3 + EST_PAINT,
    "shell": 2 * 2 * PLY_PW + RHO_R51 * 5.0e-3 + EST_PAINT,           # fuselage shell = secondary, co-cured WF
    "web": 2 * 2 * PLY_PW + RHO_R71 * 6.0e-3,
    "rib": 2 * 2 * PLY_PW + RHO_R51 * 5.0e-3,
}

# ---------------------------------------------------------------------------------------------------- estimates
EST = {
    "keepout_clearance_m": (0.020, "shell sandwich ~6 mm + frame flange ~10 mm + 4 mm gap"),
    "engine_cooling_gap_m": (0.025, "baffle + cooling-air gap around cylinders and intake"),
    "fuselage_line_slope": (0.25, "max slope of the dorsal/belly ramps aft of the forebody (~14 deg): faceted look "
                                  "without filling the whole body"),
    "tank_bay_length_m": (0.22, "forward bladder bay length (two custom bladders ~23-25 L conformal each)"),
    "aft_tank_length_ratio": (1.0, "aft bladder bay length / forward"),
    "engine_intake_half_width_m": (0.11, "throttle bodies + filters between the cylinder roots; width not published "
                                         "(engine.yaml envelope gives only the 0.2946 m height); +/-0.04 m, needs "
                                         "the Limbach STEP"),
    "turret_inside_m": (0.09, "E180 envelope: pan motor + isolators above the mount plane"),
    "chine_FF_factor": (1.05, "sharp longitudinal chines: vortex shedding at alpha/beta != 0; no published "
                              "correlation used"),
    "Q_wing_filleted": (1.00, SRC["raymer"] + " (high/mid or well-filleted wing Q = 1.0)"),
    "Q_wing_unfilleted": (1.04, "shoulder wing meeting a sharp chine at < 90 deg corner angle without fillet "
                                "(junction separation); estimate between Raymer 1.0 and 1.1"),
    "Q_tail_Y": (1.05, SRC["raymer"] + " (1.03 V-tail, +0.02 for the extra ventral junction)"),
    "Q_tail_conv": (1.04, SRC["raymer"] + " (conventional tail 1.04-1.05)"),
    "Q_fuselage": (1.00, SRC["raymer"]),
    "cooling_flush_factor": (1.25, "NACA flush inlet / S-duct: lower ram recovery than a scoop -> more spill and "
                                   "duct loss; engineering estimate"),
    "leakage_protuberance_frac": (0.05, SRC["raymer"] + " (2-5 % for propeller aircraft; upper value used)"),
    "antenna_probe_DQ_m2": (0.0015, "2 blade antennas + ADS-B blade + pitot boom + lights; drag area estimate"),
    "turret_Cd_frontal": (0.40, SRC["hoerner"] + " (sphere 0.47 subcritical; partly buried ball ~0.3-0.5)"),
    "gear_interference": (1.20, "gear-to-fuselage interference factor; engineering estimate"),
    "gear_DQ": ({"wheel_bare": 0.25, "wheel_faired": 0.13, "strut_streamlined": 0.05, "strut_round": 0.30,
                 "flat_spring_leg": 1.40, "fork_bare": 1.00}, SRC["raymer"] + " Table 12.6 (D/q per frontal area)"),
    "retract_residual_frac": (0.05, "door gaps / well leakage left after retraction, fraction of faired gear drag"),
    "base_Cd_hoerner": ("C_Db = 0.029/sqrt(C_Df)", SRC["hoerner"] + " ch. 3, base drag behind a body with turbulent "
                        "boundary layer; ignores pusher-propeller suction (conservative)"),
    "structure_maturity_margin": (0.10, "conceptual bottom-up structure omits joints, local reinforcement, "
                                        "sealant; +10 % allowance"),
    "flap_dClmax_plain": (0.90, SRC["raymer"] + " Table 12.2 plain flap; dCLmax = 0.9 dClmax (Sflapped/S) cos(L)"),
    "flap_TO_fraction": (0.60, "take-off flap setting gives ~60 % of the landing increment"),
    "flap_dCm_per_dCL": (-0.25, "flap load centroid near 50 % chord -> dCm_c/4 ~ -0.25 dCL"),
    "tail_CLmax_down": (0.80, "V-tail panel with full-up ruddervators (30 % chord), AR ~5"),
    "K_N_datcom": (0.0013, "DATCOM Fig. 5.2.3.1-8 typical 0.001-0.002 for a body with x_m/l ~0.45"),
    "K_Rl_datcom": (1.50, "DATCOM Reynolds factor at fuselage Re ~7e6"),
    "cn_beta_min_per_rad": (0.057, "0.001 per deg, usual minimum weathercock stability for GA/UAV"),
    "sm_limits": ((0.05, 0.25), "static margin band for an autopilot-flown UAV (estimate)"),
    "prop_spacer_m": (0.060, "prop plane >= 0.1 D behind the cowl exit (engine.yaml pusher note) -> 60 mm spacer"),
    "spinner_dia_m": (0.14, "covers the 72 mm prop flange + 32x18 hub"),
    "cowl_exit_annulus_outer_dia_m": (0.27, "exit area 0.040 m2 (baseline cooling.exit_area_m2) around the spinner"),
    "gear_stroke_m": (0.20, "effective vertical deflection of the spring bow, baseline.yaml design_loads.landing "
                            "(0.18-0.22 m for n_inertia 4.0-4.47); +0.05 m belly clearance at full stroke"),
    "gear_leg_ref_height_m": (0.30, "height the baseline 2.5 kg bow / 2.0 kg nose leg is assumed to have"),
    "gear_fairings_kg": (1.0, "3 wheel spats + 3 leg fairings, secondary sandwich ~0.6 m2"),
    "retract_mech_delta_kg": (2.75, "components.yaml: custom electric retraction +2 to +3.5 kg vs fixed (mid)"),
    "retract_bays_doors_kg": (2.5, "5 doors ~0.25 m2 + hinges/links + well close-outs + cut-out reinforcement + "
                                   "up/down-lock sensors"),
    "retract_well_volume_m3": (None, "3 wells: leg length x 0.19 m wheel x 0.085 m, +30 % clearance"),
    "cooling_ducts_extra_kg": (0.30, "two NACA inlets + S-ducts to the cylinder baffles"),
    "prop_spacer_kg": (0.15, "60 mm aluminium spacer + longer bolts"),
}


def est(key):
    return EST[key][0]


# =====================================================================================================================
# 2. engine and propeller models
# =====================================================================================================================
def lapse(h: float) -> float:
    """engine.yaml#installation.altitude.power_lapse_model: P(h)/P0 = sigma^1.23."""
    return AL.isa(h)["sigma"] ** LAPSE_EXP


def bsfc_kgJ(frac: float) -> float:
    """baseline.yaml engine.bsfc_curve, linear interpolation; held at 600 g/kWh below 20 % (flagged)."""
    g = float(np.interp(frac, BSFC_PTS[:, 0], BSFC_PTS[:, 1]))
    return g / 1000.0 / 3.6e6


def engine_torque(n: float) -> float:
    return float(np.interp(n, WOT_RPM, WOT_Q))


class PropModel:
    """Mejzlik 32x18 2B manufacturer table (sea level), bilinear in rpm and V. Outside the tabulated rpm range the
    boundary row is scaled at constant advance ratio (T, Q ~ n^2 at fixed J) - the method of baseline.yaml
    propeller.primary.static_thrust_N. Altitude: same J and rpm -> T, Q scale with density ratio."""

    def __init__(self, rows: dict):
        keys = ["V_0_m_s", "V_20_m_s", "V_30_m_s", "V_40_m_s", "V_50_m_s"]
        arr = [np.array(rows[k], float) for k in keys]
        self.Vg = np.array([0.0, 20.0, 30.0, 40.0, 50.0])
        self.rpm = arr[0][:, 0]
        self.T = np.array([a[:, 1] for a in arr])
        self.Q = np.array([a[:, 2] for a in arr])

    def _tab(self, n, V):
        V = min(max(V, 0.0), 50.0)
        Tn = [np.interp(n, self.rpm, row) for row in self.T]
        Qn = [np.interp(n, self.rpm, row) for row in self.Q]
        return float(np.interp(V, self.Vg, Tn)), float(np.interp(V, self.Vg, Qn))

    def sl(self, n, V):
        n0, n1 = self.rpm[0], self.rpm[-1]
        if n0 <= n <= n1:
            return self._tab(n, V)
        nb = n1 if n > n1 else n0
        T, Q = self._tab(nb, V * nb / n)
        s = (n / nb) ** 2
        return T * s, Q * s

    def wot(self, V, h, p_gen_shaft=0.0):
        """Full-throttle operating point: prop torque = lapsed engine WOT torque - generator torque."""
        sig = AL.isa(h)["sigma"]
        lp = lapse(h)

        def f(n):
            _, Qp = self.sl(n, V)
            return Qp * sig - (engine_torque(n) * lp - p_gen_shaft / (2 * math.pi * n / 60.0))
        lo, hi = 2000.0, 7500.0
        if f(hi) < 0:
            n = hi
        else:
            for _ in range(60):
                m = 0.5 * (lo + hi)
                lo, hi = (m, hi) if f(m) < 0 else (lo, m)
            n = 0.5 * (lo + hi)
        T, Q = self.sl(n, V)
        P = Q * sig * 2 * math.pi * n / 60.0
        return {"rpm": n, "T": T * sig, "P_shaft": P, "eta": (T * sig * V / P) if V > 0 else 0.0,
                "tip_mach_helical": math.hypot(math.pi * D_PROP * n / 60.0, V) / AL.isa(h)["a"]}

    def at_thrust(self, T_req, V, h):
        sig = AL.isa(h)["sigma"]
        lo, hi = 1200.0, 7500.0
        if self.sl(hi, V)[0] * sig < T_req:
            return None
        for _ in range(60):
            m = 0.5 * (lo + hi)
            lo, hi = (m, hi) if self.sl(m, V)[0] * sig < T_req else (lo, m)
        n = 0.5 * (lo + hi)
        T, Q = self.sl(n, V)
        P = Q * sig * 2 * math.pi * n / 60.0
        return {"rpm": n, "P_shaft": P, "eta_iso": T_req * V / P, "torque_frac": Q * sig / (engine_torque(n) *
                                                                                            lapse(h))}


PROP = PropModel(PROP_TABLE)
P_GEN_SHAFT = P_ELEC / ETA_GEN


# =====================================================================================================================
# 3. design variables
# =====================================================================================================================
@dataclass
class DV:
    ws_kg_m2: float = 46.0          # wing loading at MTOW (selected by the W/S-AR sweep, section 9)
    AR: float = 12.0
    taper: float = 0.45
    sweep_le_deg: float = 10.0      # identity: moderately swept leading edge
    dihedral_deg: float = 2.5
    washout_deg: float = 3.0        # 2.5 deg baseline + 0.5 deg for the sweep-induced outboard loading
    x_hub: float = 4.20             # prop-hub face station = aft end of the engine (sets fuselage length)
    z_engine: float = 0.12          # crank axis above the centre chine line (upswept aft fuselage)
    V_H: float = 0.40
    V_V: float = 0.030
    sm_min_target: float = 0.10
    style: str = "chined"           # chined | smooth | flat (n = 1 facets)
    n_top: float = 1.25             # chined style: superellipse exponent of the upper facets (< 2 -> chine edge)
    n_bot: float = 1.6              # chined style: lower facets
    fillets: bool = True
    tail: str = "Y"                 # Y = V-tail + ventral fin | conv = horizontal + vertical fin + skid
    inlets: str = "flush"           # flush | scoop
    gear: str = "fixed_faired"      # fixed_faired | fixed_bare | retract
    turret_drag: str = "HD59"       # HD59 | E180 (drag case; the bay is always sized for the E180)
    tail_AR_panel: float = 2.4
    tail_taper: float = 0.55
    tail_sweep_le_deg: float = 32.0


# =====================================================================================================================
# 4. fuselage generated around the packaging keep-outs
# =====================================================================================================================
def smoothstep(t):
    t = np.clip(t, 0.0, 1.0)
    return t * t * (3 - 2 * t)


@dataclass
class Layout:
    """x stations of every bay as a function of the wing position (x_w = wing LE at the side of body)."""
    dv: DV
    x_w: float
    c_s: float
    y_s: float
    x_c: float                      # CG target (x_le_mac + 0.21 mac): payload bay and fuel centroid go here

    def __post_init__(self):
        dv = self.dv
        self.x_tur = 0.62                                      # chin turret centre (wedge nose ahead of it)
        self.z_mount = -0.17                                   # turret mount plane
        self.x_ng = 0.90                                       # nose gear leg (behind the turret bay)
        # research payload bay ON the CG, under the carry-through (belly hatch / sensor window)
        self.pay = (self.x_c - 0.20, self.x_c + 0.20)
        # fuel split fore/aft of the payload bay, equal volumes -> fuel centroid on the CG, common header tank
        Lt = est("tank_bay_length_m")
        self.tank_f = (self.pay[0] - 0.015 - Lt, self.pay[0] - 0.015)
        self.tank_a = (self.pay[1] + 0.015, self.pay[1] + 0.015 + Lt * est("aft_tank_length_ratio"))
        self.tank = (self.tank_f[0], self.tank_a[1])
        # GRS B8 container on top of the avionics bay: canopy leaves upward from the forebody, far from the prop
        self.chute = (0.80, 0.80 + 0.375)
        self.av = (0.74, self.tank_f[0] - 0.02)                # avionics / power bay (under the parachute)
        self.bat = (0.30, 0.54)                                # LiFePO4 buffer battery in the nose (balance)
        self.box = (self.x_w + 0.12 * self.c_s, self.x_w + 0.66 * self.c_s)   # spar carry-through box
        self.x_fw = dv.x_hub - ENV_ENG["length_with_sg750"] - 0.06                # firewall
        self.aft = (self.tank_a[1] + 0.02, self.x_fw - 0.01)  # ECU / pump / gascolator / gen. controller
        self.eng = (dv.x_hub - ENV_ENG["length_with_sg750"], dv.x_hub)
        self.x_prop = dv.x_hub + est("prop_spacer_m") + 0.02
        self.x_up0 = self.x_w + self.c_s + 0.15
        self.x_up1 = dv.x_hub - 0.30

    def zc(self, x):
        x = np.asarray(x, float)
        z_tip = -0.06
        z = z_tip * (1 - smoothstep(x / 0.9))
        z = z + self.dv.z_engine * smoothstep((x - self.x_up0) / max(self.x_up1 - self.x_up0, 0.1))
        return z

    def keepouts(self):
        """[(name, x0, x1, [(y, z), ...])] with clearance already added; y >= 0 (mirrored)."""
        c = est("keepout_clearance_m")
        ce = est("engine_cooling_gap_m")
        ze = self.dv.z_engine
        hi = est("turret_inside_m")
        ko = [
            ("battery_fwd_equipment", self.bat[0], self.bat[1], [(0.07 + c, -0.04), (0.07 + c, 0.06 + c)]),
            ("turret_E180_bay", self.x_tur - E180_D / 2 - 0.01, self.x_tur + E180_D / 2 + 0.01,
             [(E180_D / 2 + 0.01, self.z_mount), (E180_D / 2 + 0.01, self.z_mount + hi + c)]),
            ("avionics_power", self.av[0], self.av[1], [(0.11 + c, -0.09 - c), (0.11 + c, 0.01)]),
            ("nose_gear_leg", self.x_ng - 0.07, self.x_ng + 0.07, [(0.05 + c, -0.12), (0.05 + c, -0.05)]),
            ("payload_bay", self.pay[0], self.pay[1], [(0.12 + c, -0.24 - c), (0.12 + c, -0.065)]),
            ("parachute_GRS_B8", self.chute[0], self.chute[1], [(0.1025 + c, 0.03), (0.1025 + c, 0.14 + c)]),
            ("fuel_tank_fwd_core", self.tank_f[0], self.tank_f[1], [(0.12 + c, -0.22 - c), (0.12 + c, 0.01)]),
            ("fuel_tank_aft_core", self.tank_a[0], self.tank_a[1], [(0.12 + c, -0.22 - c), (0.12 + c, 0.01)]),
            ("carry_through", self.box[0], self.box[1], [(0.18, 0.075 + c), (0.18, -0.045 - c)]),
            ("aft_equipment", self.aft[0], self.aft[1], [(0.10 + c, -0.08 - c + 0.6 * ze), (0.10 + c, 0.05 + c + ze)]),
            ("engine_mount", self.x_fw, self.eng[0], [(0.12 + c, ze - 0.12), (0.12 + c, ze + 0.12)]),
            ("engine_generator", self.eng[0], self.eng[0] + 0.08, [(0.06 + ce, ze - 0.06 - ce), (0.06 + ce, ze + 0.06 + ce)]),
            ("engine_intake", self.dv.x_hub - 0.19, self.dv.x_hub - 0.02,
             [(est("engine_intake_half_width_m") + ce,
               ze + ENV_ENG["height_cylinder_side_to_intake_cover_top"] - 0.060 + ce)]),
            ("engine_cylinders", self.dv.x_hub - 0.17, self.dv.x_hub - 0.03,
             [(ENV_ENG["width_incl_spark_plug_caps"] / 2 + ce, ze + 0.060 + ce),
              (ENV_ENG["width_incl_spark_plug_caps"] / 2 + ce, ze - 0.060 - ce)]),
            ("silencers", self.dv.x_hub - 0.30, self.dv.x_hub - 0.06, [(0.10 + c, ze - 0.15 - c), (0.10 + c, ze - 0.085)]),
        ]
        return ko


def section_style(dv: DV, L: Layout, x):
    """(n_top, n_bot) per station: near-flat facets (identity), ellipses (smooth) or true flat facets."""
    x = np.asarray(x, float)
    if dv.style == "smooth":
        return np.full_like(x, 2.0), np.full_like(x, 2.0)
    nt0, nb0 = (1.0, 1.0) if dv.style == "flat" else (dv.n_top, dv.n_bot)
    # dorsal engine cowl rounds up over the intake (packaging), round at the exit (spinner junction)
    t_cowl = smoothstep((x - (L.eng[0] - 0.35)) / 0.30)
    t_exit = smoothstep((x - (L.dv.x_hub - 0.05)) / 0.05)
    nt = nt0 + (max(nt0, 1.9) - nt0) * t_cowl
    nb = nb0 + (max(nb0, 1.6) - nb0) * t_cowl
    nt = nt + (2.0 - nt) * t_exit
    nb = nb + (2.0 - nb) * t_exit
    return nt, nb


def _F(y, dz, a, bt, bb, nt, nb):
    if dz >= 0:
        return (abs(y) / a) ** nt + (dz / bt) ** nt
    return (abs(y) / a) ** nb + (-dz / bb) ** nb


_PH = np.linspace(0, 2 * math.pi, 65)


def perimeter(a, bt, bb, nt, nb):
    s, c = np.sin(_PH), np.cos(_PH)
    n = np.where(c >= 0, nt, nb)
    hh = np.where(c >= 0, bt, bb)
    y = a * np.sign(s) * np.abs(s) ** (2.0 / n)
    z = hh * np.sign(c) * np.abs(c) ** (2.0 / n)
    return float(np.sum(np.hypot(np.diff(y), np.diff(z))))


def solve_section(pts, nt, nb, mins):
    amin, btmin, bbmin = mins
    if not pts:
        return np.array([amin, btmin, bbmin])
    x0 = np.array([max(amin, 0.02), max(btmin, 0.02), max(bbmin, 0.02)])
    for _ in range(300):
        if all(_F(y, dz, *x0, nt, nb) <= 1.0 for y, dz in pts):
            break
        x0 = x0 * 1.04
    cons = [{"type": "ineq", "fun": (lambda v, y=y, dz=dz: 1.0 - _F(y, dz, v[0], v[1], v[2], nt, nb))}
            for y, dz in pts]
    res = minimize(lambda v: perimeter(v[0], v[1], v[2], nt, nb), x0, method="SLSQP",
                   bounds=[(amin, 1.5), (btmin, 1.5), (bbmin, 1.5)], constraints=cons,
                   options={"ftol": 1e-10, "maxiter": 300})
    v = res.x
    if not all(_F(y, dz, *v, nt, nb) <= 1.0 + 1e-6 for y, dz in pts):
        v = x0
    return np.maximum(v, [amin, btmin, bbmin]) * 1.001


def unimodal(v):
    v = np.asarray(v, float)
    i = int(np.argmax(v))
    out = v.copy()
    out[: i + 1] = np.maximum.accumulate(v[: i + 1])
    out[i:] = np.maximum.accumulate(v[i:][::-1])[::-1]
    return out


def _hull(x, y, upper=True):
    """Least concave majorant (upper=True) or greatest convex minorant of the samples, evaluated at x."""
    pts = []
    for p in zip(x, y):
        while len(pts) >= 2:
            o, a = pts[-2], pts[-1]
            cr = (a[0] - o[0]) * (p[1] - o[1]) - (a[1] - o[1]) * (p[0] - o[0])
            if (cr >= 0) if upper else (cr <= 0):
                pts.pop()
            else:
                break
        pts.append(p)
    hx, hy = zip(*pts)
    return np.interp(x, hx, hy)


def _slope_env(x, y, s_up, upper=True):
    """Smallest envelope above (upper) / below the samples whose slope magnitude is limited to s_up (ramps)."""
    x = np.asarray(x)
    y = np.asarray(y)
    D = np.abs(x[:, None] - x[None, :]) * s_up
    return (y[None, :] - D).max(axis=1) if upper else (y[None, :] + D).min(axis=1)


def _profile(x, y, x_split, s_lim, upper=True):
    """Forebody (x <= x_split): hull from the nose tip (straight wedge lines); aft of it: slope-limited ramps."""
    out = _slope_env(x, y, s_lim, upper)
    m = x <= x_split
    if np.sum(m) >= 2:
        h = _hull(x[m], out[m], upper=upper)
        out[m] = np.maximum(out[m], h) if upper else np.minimum(out[m], h)
    return out


def build_fuselage(dv: DV, L: Layout):
    """Station table from the packaging keep-outs. (1) minimal-perimeter section at every sample (20 mm + every
    keep-out edge); (2) least concave majorant of the half-width (straight faceted planform lines, wedge nose);
    (3) with that width, the smallest top and bottom heights in closed form, then the concave majorant of the top
    line and the convex minorant of the belly line; (4) stations every 50 mm, verified at 10 mm and grown locally
    where the PCHIP surface would cut a keep-out. The same rules generate every style (fair comparison)."""
    ko = L.keepouts()
    x_end = dv.x_hub
    edges = [v for _, x0, x1, _ in ko for v in (x0, x1)]
    xs = np.unique(np.clip(np.r_[np.arange(0.0, x_end + 1e-9, 0.02), edges, [x_end]], 0.0, x_end))
    nt_s, nb_s = section_style(dv, L, xs)
    zc = L.zc(xs)
    identity = dv.style != "smooth"
    r_exit = est("cowl_exit_annulus_outer_dia_m") / 2

    def pts_at(x, z0):
        out = []
        for name, x0, x1, P in ko:
            if x0 - 1e-9 <= x <= x1 + 1e-9:
                out += [(y, z - z0) for y, z in P]
        return out

    def mins(x):
        amin, bmin = 0.004, 0.003
        if identity and dv.fillets and L.x_w - 0.30 <= x <= L.x_w + L.c_s:
            amin = 0.25                        # wide chine at the wing root -> blended wing/fuselage
        if x >= x_end - 0.02:
            amin, bmin = r_exit, r_exit
        return amin, bmin
    A = []
    for x, nt, nb, z0 in zip(xs, nt_s, nb_s, zc):
        amin, bmin = mins(x)
        A.append(solve_section(pts_at(x, z0), nt, nb, (amin, bmin, bmin))[0])
    A = _hull(xs, np.array(A), upper=True)
    ZT, ZB = [], []
    for x, a, nt, nb, z0 in zip(xs, A, nt_s, nb_s, zc):
        amin, bmin = mins(x)
        bt, bb = bmin, bmin
        for y, dz in pts_at(x, z0):
            f = 1.0 - min(abs(y) / a, 0.999) ** (nt if dz >= 0 else nb)
            if dz > 0:
                bt = max(bt, dz / f ** (1.0 / nt))
            elif dz < 0:
                bb = max(bb, -dz / f ** (1.0 / nb))
        ZT.append(z0 + bt)
        ZB.append(z0 - bb)
    ZT, ZB = np.array(ZT), np.array(ZB)
    fore = xs <= L.x_w                                   # forebody: nose to the wing leading edge
    x_top = xs[fore][int(np.argmax(ZT[fore]))]
    x_bot = xs[fore][int(np.argmin(ZB[fore]))] if np.any(fore) else 0.0
    ZT = unimodal(_profile(xs, ZT, x_top, est("fuselage_line_slope"), upper=True))      # no dips in the spine
    ZB = -unimodal(-_profile(xs, ZB, x_bot, est("fuselage_line_slope"), upper=False))   # no bumps in the belly
    xst = np.unique(np.r_[np.arange(0.0, x_end - 0.02, 0.05), [x_end - 0.02, x_end]])
    rows = []
    for i, x in enumerate(xst):
        a = float(np.interp(x, xs, A))
        z0 = float(L.zc(x))
        bt = max(float(np.interp(x, xs, ZT)) - z0, 0.003)
        bb = max(z0 - float(np.interp(x, xs, ZB)), 0.003)
        nt, nb = section_style(dv, L, x)
        if i == 0:
            a, bt, bb = 0.004, 0.003, 0.003
        rows.append([x, 2 * a, bt + bb, z0, float(nt), float(nb), bt / (bt + bb)])
    st = np.array(rows)
    st[:, 6] = np.clip(st[:, 6], 0.06, 0.94)
    for _ in range(60):
        F = oml.Fuselage(st)
        worst = []
        for name, x0, x1, P in ko:
            for x in np.linspace(x0, x1, max(int((x1 - x0) / 0.01), 2)):
                w, h, z0, nt, nb = (float(v) for v in F.section(x))
                tf = float(F.top_frac(x))
                for y, z in P:
                    if _F(y, z - z0, w / 2, tf * h, (1 - tf) * h, nt, nb) > 1.0 + 1e-6:
                        worst.append(x)
        if not worst:
            break
        for x in worst:
            j = int(np.argmin(np.abs(st[:, 0] - x)))
            for k in (j - 1, j, j + 1):
                if 0 < k < len(st):
                    st[k, 1] *= 1.005
                    st[k, 2] *= 1.005
    F = oml.Fuselage(st)
    return F, st, ko


def fuselage_props(F: oml.Fuselage, st):
    m = F.mesh(120, 96)
    area = m.area()
    # remove the flat end cap (cowl exit annulus + spinner base; not wetted)
    w_end, h_end = st[-1, 1], st[-1, 2]
    cap = math.pi / 4 * w_end * h_end
    xs = np.linspace(F.x0, F.x1, 400)
    sec = []
    for x in xs:
        w, h, z0, nt, nb = (float(v) for v in F.section(x))
        tf = float(F.top_frac(x))
        # section area of the two superellipse halves: 2 a b G(1+1/n)^2 / G(1+2/n)
        a = w / 2
        A = 0.0
        for b, n in ((tf * h, nt), ((1 - tf) * h, nb)):
            A += 2 * a * b * math.gamma(1 + 1 / n) ** 2 / math.gamma(1 + 2 / n)
        sec.append((x, w, h, A))
    sec = np.array(sec)
    i = int(np.argmax(sec[:, 3]))
    side = float(np.trapz(sec[:, 2], sec[:, 0]))
    tri = m.triangles()
    ta = 0.5 * np.linalg.norm(np.cross(tri[:, 1] - tri[:, 0], tri[:, 2] - tri[:, 0]), axis=1)
    xc = float(np.sum(ta * tri[:, :, 0].mean(axis=1)) / ta.sum())
    return {"S_wet": area - cap, "volume": m.volume(), "A_max": float(sec[i, 3]), "x_Amax": float(sec[i, 0]),
            "w_max": float(sec[:, 1].max()), "h_max": float(sec[:, 2].max()), "length": F.x1 - F.x0,
            "S_side": side, "x_shell_centroid": xc, "sections": sec, "mesh": m}


# =====================================================================================================================
# 5. wing, tail, gear geometry
# =====================================================================================================================
def planform(dv: DV, m0: float):
    S = m0 / dv.ws_kg_m2
    b = math.sqrt(dv.AR * S)
    cr0 = 2 * S / (b * (1 + dv.taper))
    return {"S": S, "b": b, "semispan": b / 2, "c_r0": cr0, "c_t": dv.taper * cr0}


def chord_at(P, y):
    return P["c_r0"] - (P["c_r0"] - P["c_t"]) * (y / P["semispan"])


def wing_sections(dv: DV, P, x_w, y_s, z_root, i_w, centreline=False):
    """LiftingSurface sections (starboard). Geometry: side of body -> flap/aileron break -> tip.
    Analysis (centreline=True): trapezoid continued to the centre line (reference-area convention)."""
    tanle = math.tan(math.radians(dv.sweep_le_deg))
    tand = math.tan(math.radians(dv.dihedral_deg))
    x_le0 = x_w - y_s * tanle
    ys = [y_s, 0.58 * P["semispan"], P["semispan"]]
    if centreline:                    # analysis: only the 16 % root and 13 % tip sections (linear blend between)
        ys = [0.0, y_s, P["semispan"]]

    def sec(y):
        t = max(0.0, (y - y_s) / (P["semispan"] - y_s))
        return {"y": float(y), "x_le": float(x_le0 + y * tanle), "z_le": float(z_root + max(0.0, y - y_s) * tand),
                "chord": float(chord_at(P, y)), "twist_deg": float(i_w - dv.washout_deg * t), "airfoil": "nlf416",
                "thickness_scale": float(1.0 - (1.0 - 0.8125) * t)}
    return [sec(y) for y in ys]


def tail_geometry(dv: DV, S_h, S_v, x_prop, top_fn, y_root=0.05):
    """V-tail panels from the required effective areas (Purser-Campbell projection): tan^2(G) = S_v/S_h."""
    S_tot = S_h + S_v
    gam = math.degrees(math.atan(math.sqrt(S_v / S_h)))
    Sp = S_tot / 2
    s = math.sqrt(dv.tail_AR_panel * Sp)
    cr = 2 * Sp / (s * (1 + dv.tail_taper))
    ct = dv.tail_taper * cr
    tl = math.tan(math.radians(dv.tail_sweep_le_deg))
    x_r = x_prop - 0.02 - s * tl - ct                       # tip TE 20 mm ahead of the propeller plane
    z_r = top_fn(x_r + 0.4 * cr) - 0.01
    g = math.radians(gam)
    secs = [{"y": y_root + s * k * math.cos(g), "x_le": x_r + s * k * tl, "z_le": z_r + s * k * math.sin(g),
             "chord": cr + (ct - cr) * k, "twist_deg": 0.0, "airfoil": "n0012"} for k in (0.0, 1.0)]
    y_mac = s / 3 * (1 + 2 * dv.tail_taper) / (1 + dv.tail_taper)
    mac = 2 / 3 * cr * (1 + dv.tail_taper + dv.tail_taper ** 2) / (1 + dv.tail_taper)
    x_ac = x_r + y_mac * tl + 0.25 * mac
    z_ac = z_r + y_mac * math.sin(g)
    AR_V = (2 * s) ** 2 / (2 * Sp)                    # aspect ratio of the flattened V (both panels)
    tc_half = math.degrees(math.atan(tl - (cr - ct) / (2 * s)))
    a_V = AL.lift_slope(AR_V, tc_half)
    return {"sections": secs, "dihedral_deg": gam, "S_panels": S_tot, "S_panel": Sp, "span_panel": s, "c_root": cr,
            "c_tip": ct, "mac": mac, "x_ac": x_ac, "z_ac": z_ac, "x_root_le": x_r, "z_root": z_r, "AR_V": AR_V,
            "a_per_rad": a_V, "S_h_eff": S_tot * math.cos(math.radians(gam)) ** 2,
            "S_v_eff": S_tot * math.sin(math.radians(gam)) ** 2, "tip_te_x": x_r + s * tl + ct}


def ventral_geometry(dv: DV, L: Layout, bottom_fn):
    """Ventral fin = propeller guard + tail bumper: tip 60 mm below the propeller disc, replaceable skid."""
    R = D_PROP / 2
    z_tip = dv.z_engine - R - 0.06
    tl = math.tan(math.radians(40.0))
    ct = 0.20
    x_tip_le = L.x_prop - 0.06 - ct
    # root on the fuselage bottom: solve span so that the root LE lies on the swept line
    span = 0.3
    for _ in range(20):
        x_root_le = x_tip_le - span * tl
        z_root = bottom_fn(x_root_le + 0.15) + 0.01
        span = max(0.05, z_root - z_tip)
    cr = 0.42
    secs = [{"y": 0.0, "x_le": x_root_le, "z_le": z_root, "chord": cr, "twist_deg": 0.0, "airfoil": "n0012"},
            {"y": 0.0, "x_le": x_tip_le, "z_le": z_tip, "chord": ct, "twist_deg": 0.0, "airfoil": "n0012"}]
    S = span * (cr + ct) / 2
    AR_eff = 2 * span * span / S                    # fuselage end-plate doubles the effective AR
    y_mac = span / 3 * (1 + 2 * ct / cr) / (1 + ct / cr)
    mac = 2 / 3 * cr * (1 + ct / cr + (ct / cr) ** 2) / (1 + ct / cr)
    return {"sections": secs, "S": S, "span": span, "c_root": cr, "c_tip": ct, "z_skid": z_tip,
            "x_skid": x_tip_le + 0.5 * ct, "a_per_rad": AL.lift_slope(AR_eff, 35.0),
            "x_ac": x_root_le + y_mac * tl + 0.25 * mac, "AR_eff": AR_eff}


# =====================================================================================================================
# 6. drag build-up and wing aerodynamics
# =====================================================================================================================
class Polar:
    """Aircraft drag polar at one flight condition: non-wing CD0 + strip-integrated tripped wing profile drag
    (x 1.15, exposed share) + induced drag from the lifting line; parabolic fit for the aerolib performance
    functions."""

    def __init__(self, CL, CD, AR):
        self.CLt = np.asarray(CL)
        self.CDt = np.asarray(CD)
        m = (self.CLt >= 0.2) & (self.CLt <= 1.25)
        k, cd0 = np.polyfit(self.CLt[m] ** 2, self.CDt[m], 1)
        self.cd0, self.k = float(cd0), float(k)
        self.e = 1.0 / (math.pi * AR * self.k)
        ld = self.CLt / self.CDt
        i = int(np.argmax(ld))
        self.ld_max, self.cl_ld_max = float(ld[i]), float(self.CLt[i])
        end = self.CLt ** 1.5 / self.CDt
        j = int(np.argmax(end))
        self.cl_end_max = float(self.CLt[j])

    def cd(self, CL):
        if CL <= self.CLt[-1]:
            return float(np.interp(CL, self.CLt, self.CDt))
        return float(self.CDt[-1] + 2 * self.k * self.CLt[-1] * (CL - self.CLt[-1]) + 3 * self.k * (CL - self.CLt[-1]) ** 2)


def gear_drag(dv: DV, gear):
    d = est("gear_DQ")
    tyre_D, tyre_W = 0.190, 0.054                       # TOST 200x50 mounted OD/width (components.yaml)
    fw = tyre_D * tyre_W
    if dv.gear == "fixed_bare":
        dq = 3 * d["wheel_bare"] * fw + 2 * d["flat_spring_leg"] * gear["h_main_leg"] * 0.035 + \
            d["strut_round"] * gear["h_nose_leg"] * 0.04 + d["fork_bare"] * 0.10 * 0.03
    else:
        dq = 3 * d["wheel_faired"] * fw + 2 * d["strut_streamlined"] * gear["h_main_leg"] * 0.035 + \
            d["strut_streamlined"] * gear["h_nose_leg"] * 0.04
    dq *= est("gear_interference")
    if dv.gear == "retract":
        dq *= est("retract_residual_frac")
    return dq


def turret_drag(dv: DV):
    if dv.turret_drag == "E180":
        d, prot = E180_D, E180_H - est("turret_inside_m")
    else:
        d, prot = TURRET_D, TURRET_H - 0.06
    return est("turret_Cd_frontal") * d * prot, {"diameter_m": d, "protrusion_m": prot}


def cooling_drag_DQ(V, h, dv: DV):
    """Raymer (D/q)_cooling = 4.9e-7 bhp T^2 / (sigma V) [ft2, bhp, degR, ft/s] for an air-cooled piston engine
    (as recalled), with the rated power (inlet sized for climb) x flush-inlet factor."""
    atm = AL.isa(h)
    bhp = P_MAX / 745.7
    TR = atm["T"] * 1.8
    dq_ft2 = 4.9e-7 * bhp * TR ** 2 / (atm["sigma"] * V / 0.3048)
    f = est("cooling_flush_factor") if dv.inlets == "flush" else 1.0
    return dq_ft2 * 0.3048 ** 2 * f


def cooling_drag_momentum(P_shaft, V):
    """Cross-check: heat to cooling air ~0.95 x shaft power (engine.yaml 0.8-1.1), dT 40 K, exit at 0.5 V."""
    mdot = 0.95 * P_shaft / (1005.0 * 40.0)
    return mdot * V * 0.5


def drag_buildup(dv: DV, geo, V, h):
    """Non-wing parasite drag items (drag area D/q, m2) at TAS V and altitude h."""
    atm = AL.isa(h)
    M = V / atm["a"]
    S = geo["S"]
    items = {}
    fz = geo["fus"]
    Re_f = AL.reynolds(V, fz["length"], h)
    fineness = fz["length"] / math.sqrt(4 * fz["A_max"] / math.pi)
    ff = AL.ff_body(fineness) * (est("chine_FF_factor") if dv.style != "smooth" else 1.0)
    cf_f = AL.cf_flat(Re_f, 0.0)
    items["fuselage"] = cf_f * ff * est("Q_fuselage") * fz["S_wet"]
    if dv.fillets and dv.style != "smooth":
        items["wing_root_fillets"] = cf_f * 1.1 * geo["fillet_S_wet"]
    # base drag of the steep aft cowl closure (boxer cylinders 30-170 mm ahead of the hub face)
    A_b = geo["base_area"]
    if A_b > 0:
        C_Df = cf_f * ff * fz["S_wet"] / A_b
        items["aft_cowl_base"] = 0.029 / math.sqrt(C_Df) * A_b
    tl = geo["tail"]
    if dv.tail == "Y":
        Re_t = AL.reynolds(V, tl["mac"], h)
        items["v_tail"] = AL.cf_flat(Re_t) * AL.ff_wing(0.12, 0.3, tl["sweep_tmax_deg"], M) * est("Q_tail_Y") * \
            AL.wing_wetted(tl["S_panels"], 0.12)
        vf = geo["ventral"]
        Re_v = AL.reynolds(V, 0.5 * (vf["c_root"] + vf["c_tip"]), h)
        items["ventral_fin"] = AL.cf_flat(Re_v) * AL.ff_wing(0.12, 0.3, 35.0, M) * est("Q_tail_Y") * \
            AL.wing_wetted(vf["S"], 0.12)
    else:
        for key, Sx, c in (("h_tail", tl["S_h_eff"], tl["mac"]), ("v_fin", tl["S_v_eff"], tl["mac"])):
            Re_t = AL.reynolds(V, c, h)
            items[key] = AL.cf_flat(Re_t) * AL.ff_wing(0.12, 0.3, 25.0, M) * est("Q_tail_conv") * \
                AL.wing_wetted(Sx, 0.12)
        items["tail_skid"] = 0.0004
    items["landing_gear"] = gear_drag(dv, geo["gear"])
    items["eo_ir_turret"], _ = turret_drag(dv)
    items["cooling"] = cooling_drag_DQ(V, h, dv)
    items["antennas_probes_lights"] = est("antenna_probe_DQ_m2")
    sub = sum(items.values())
    items["leakage_protuberances"] = est("leakage_protuberance_frac") * sub
    cd = {k: v / S for k, v in items.items()}
    return items, cd


def wing_aero(dv: DV, geo, V, h, n_crit=TRIPPED):
    secs = geo["wing_sections_analysis"]
    sa = AE.surface_analysis(secs, V, h, n_crit=n_crit, k_clmax=CLMAX_FACTOR * K_CLMAX_3D)
    # sweep correction (the Glauert solution neglects sweep): Helmbold slope ratio and Nita-Scholz e ratio
    P = geo["planform"]
    sw_c4 = geo["sweep_c4_deg"]
    sw_c2 = geo["sweep_c2_deg"]
    k_a = AL.lift_slope(dv.AR, sw_c2) / AL.lift_slope(dv.AR, 0.0)
    k_e = AL.oswald_nita_scholz(dv.AR, dv.taper, sw_c4) / AL.oswald_nita_scholz(dv.AR, dv.taper, 0.0)
    return sa, k_a, k_e


def aircraft_polar(dv: DV, geo, V, h):
    items, cd_items = drag_buildup(dv, geo, V, h)
    cd0_nonwing = sum(cd_items.values())
    sa, k_a, k_e = wing_aero(dv, geo, V, h, TRIPPED)
    S, S_exp = geo["S"], geo["S_exp"]
    Qw = est("Q_wing_filleted") if (dv.fillets or dv.style == "smooth") else est("Q_wing_unfilleted")
    kF = 1 - 2 * (geo["fus"]["w_max"] / geo["planform"]["b"]) ** 2         # Nita-Scholz fuselage factor
    e_ind = sa["e_inviscid"] * k_e * kF
    CLs = np.round(np.arange(0.0, 1.62, 0.05), 3)
    CDw, CDs = [], []
    for CL in CLs:
        cdp = sa["cd_profile"](max(CL, 0.0)) * CD_TRIP_FACTOR * (S_exp / S) * Qw
        CDw.append(cdp)
        CDs.append(cd0_nonwing + cdp + CL * CL / (math.pi * dv.AR * e_ind))
    pol = Polar(CLs, CDs, dv.AR)
    pol.cd0_nonwing = cd0_nonwing
    pol.items = items
    pol.cd_items = cd_items
    pol.cd_wing_profile = dict(zip([float(c) for c in CLs], CDw))
    pol.e_ind = e_ind
    pol.sa = sa
    pol.k_a = k_a
    return pol


# =====================================================================================================================
# 7. masses: fixed equipment, structure, CG
# =====================================================================================================================
def fixed_equipment(dv: DV, geo):
    """Bottom-up from baseline.yaml mass_targets.systems_rollup_kg, adjusted for this configuration."""
    g = geo["gear"]
    items = dict(SYS)
    href = est("gear_leg_ref_height_m")
    bow, nose_leg = 2.5, 2.0                      # baseline 9.6 = 9.1 (components) + 0.5 longer-stroke bow
    rest = SYS["landing_gear"] - bow - nose_leg
    items["landing_gear"] = rest + bow * max(1.0, g["h_main_leg"] / href) + nose_leg * max(1.0, g["h_nose_leg"] / href)
    if dv.gear == "fixed_faired":
        items["gear_fairings"] = est("gear_fairings_kg")
    if dv.gear == "retract":
        items["landing_gear"] += est("retract_mech_delta_kg")
        items["gear_bays_doors"] = est("retract_bays_doors_kg")
    if dv.inlets == "flush":
        items["cowling_cooling_ducts_baffles_firewall"] += est("cooling_ducts_extra_kg")
    items["prop_spacer"] = est("prop_spacer_kg")
    items["wiring_harness_connectors_coax"] = SYS["wiring_harness_connectors_coax"] * dv.x_hub / 3.4
    return items


def wing_structure(dv: DV, P, y_s, n_ult, m0, mac):
    """Bottom-up wing: sandwich skins (materials.yaml minimum gauges), CFRP UD spar caps sized at ultimate by the
    damage-tolerance compression strain limit (or A-basis x special factor), min-gauge sandwich webs, ribs."""
    semi = P["semispan"]
    y = np.linspace(0.0, semi, 121)
    c = np.array([chord_at(P, yy) for yy in y])
    tc = np.where(y <= y_s, 0.16, 0.16 - 0.03 * (y - y_s) / (semi - y_s))
    W = m0 * G
    l = ST.schrenk(y, c, semi)
    w = n_ult * W / 2 * l                                     # lift per side, inertia relief ignored (conservative)
    V, M = ST.beam_loads(y, w)
    h = 0.85 * tc * c
    sig = min(STRAIN_C_THICK * UD["E"], A_BASIS * UD["Fcu"] / SPECIAL_F)
    A_cap = np.maximum(M / (h * sig), 30e-6)
    caps = 2 * np.trapz(A_cap, y) * UD["density"] * 1.10 * 2           # 2 caps, 10 % ply drops/laps, 2 sides
    webs = (1 + 0.6) * np.trapz(h, y) * AREAL["web"] * 2
    tau = V / (h * 4 * PW["ply_t"])                                      # shear in the min-gauge web faces
    S_exp = 2 * np.trapz(c[y >= y_s], y[y >= y_s])
    t_avg = float(np.mean(tc[y >= y_s]))
    S_wet = AL.wing_wetted(S_exp, t_avg)
    skins = S_wet * (0.70 * AREAL["skin_primary"] + 0.30 * AREAL["skin_secondary"])
    n_ribs = int(math.ceil((semi - y_s) / 0.45)) + 2
    yr = np.linspace(y_s, semi, n_ribs)
    ribs = 2 * sum(0.70 * chord_at(P, yy) ** 2 * float(np.interp(yy, y, tc)) for yy in yr) * AREAL["rib"] * 1.15
    out = {"skins": skins, "spar_caps": caps, "spar_webs": webs, "ribs": ribs, "control_hinges_horns": 4 * 0.12,
           "root_joints_removable_panels": 2 * 0.90, "le_erosion_tape": 0.25}
    info = {"M_root_ult_N_m": float(M[0]), "V_root_ult_N": float(V[0]), "sigma_allow_cap_Pa": sig,
            "cap_area_root_m2": float(A_cap[0]), "spar_depth_root_m": float(h[0]),
            "web_shear_root_Pa": float(tau[0]), "S_exp_m2": S_exp, "S_wet_m2": S_wet, "n_ribs_per_side": n_ribs}
    return out, info


def tail_structure(dv: DV, tail, ventral):
    out = {}
    if dv.tail == "Y":
        Sw = AL.wing_wetted(tail["S_panels"], 0.12)
        out["v_tail_skins"] = Sw * (0.75 * AREAL["skin_primary"] + 0.25 * AREAL["skin_secondary"])
        out["v_tail_spars_ribs"] = 2 * (0.30 + 4 * 0.03)
        out["v_tail_root_fittings"] = 2 * 0.25
        out["ruddervator_hinges"] = 2 * 0.12
        out["ventral_fin"] = AL.wing_wetted(ventral["S"], 0.12) * AREAL["skin_primary"] + 0.15 + 0.15
    else:
        Sw = AL.wing_wetted(tail["S_h_eff"] + tail["S_v_eff"], 0.12)
        out["tail_skins"] = Sw * (0.75 * AREAL["skin_primary"] + 0.25 * AREAL["skin_secondary"])
        out["tail_spars_ribs"] = 3 * (0.30 + 4 * 0.03)
        out["tail_root_fittings"] = 3 * 0.25
        out["elevator_rudder_hinges"] = 3 * 0.12
        out["tail_skid"] = 0.15
    return out


def fuselage_structure(dv: DV, geo, L: Layout):
    fz = geo["fus"]
    sec = fz["sections"]
    Lc = dv.x_hub - 0.35
    out = {"shell_panels": fz["S_wet"] * AREAL["shell"] * 1.15,          # +15 % hatches, doublers, receptacles
           "chine_longerons": 2 * Lc * 0.30,
           "keel_beam": 0.25 * (dv.x_hub - 0.6)}
    xs_f = np.arange(0.35, dv.x_hub, 0.35)
    perim = []
    for x in xs_f:
        w, h, z0, nt, nb = (float(v) for v in geo["F"].section(x))
        tf = float(geo["F"].top_frac(x))
        perim.append(perimeter(w / 2, tf * h, (1 - tf) * h, nt, nb))
    out["light_frames"] = float(np.sum(perim)) * 0.15
    out["main_frames"] = 0.8 + 1.0 + 0.6 + 0.4 + 0.9 + 0.5        # spar x2, nose gear/turret, chute, firewall, tail
    out["floors_trays_tank_cradle"] = 1.2
    if dv.style != "smooth":
        out["chine_edge_closeouts"] = 2 * Lc * 0.04
    if dv.fillets and dv.style != "smooth":
        out["wing_root_fillets"] = geo["fillet_S_wet"] * AREAL["shell"]
    if dv.gear == "retract":
        dL = geo["retract_dL"]
        p_mid = float(np.interp(L.x_w, sec[:, 0], [perimeter(r[1] / 2, r[2] / 2, r[2] / 2, 2, 2) for r in sec]))
        out["fuselage_plug_for_wells"] = dL * (p_mid * AREAL["shell"] * 1.15 + 2 * 0.30 + 0.25 + 0.6)
    return out


def structure(dv: DV, geo, m0, L: Layout):
    P = planform(dv, m0)
    sc = P["S"] / geo["S"]                                              # tail scales with S x mac (V_H fixed)
    wing, info = wing_structure(dv, P, geo["y_s"], geo["n_ult"], m0, geo["mac"])
    tail = {k: v * sc ** 1.25 for k, v in tail_structure(dv, geo["tail"], geo["ventral"]).items()}
    fus = fuselage_structure(dv, geo, L)
    tot = sum(wing.values()) + sum(tail.values()) + sum(fus.values())
    margin = est("structure_maturity_margin") * tot
    return {"wing": wing, "tail": tail, "fuselage": fus, "maturity_margin": margin,
            "total": tot + margin, "wing_info": info}


def payload_items(case: str, L: Layout):
    """Payload split by loading case. 'design': 20 kg (turret + mission computer + tray + research allowance)."""
    tur = (TURRET_M, L.x_tur, L.z_mount - 0.05)
    mc = (MISSION_COMPUTER_M, 0.5 * (L.av[0] + L.av[1]), 0.0)
    tray = (PAY_TRAY_M, 0.5 * (L.av[0] + L.av[1]), -0.02)
    res = (RESEARCH_PAY_M, 0.5 * (L.pay[0] + L.pay[1]), -0.16)
    if case == "design":
        return {"turret": tur, "mission_computer": mc, "payload_tray": tray, "research_payload": res}
    if case == "e180_growth":
        res2 = (RESEARCH_PAY_M - (E180_M - TURRET_M), res[1], res[2])
        return {"turret_E180": (E180_M, L.x_tur, L.z_mount - 0.06), "mission_computer": mc, "payload_tray": tray,
                "research_payload": res2}
    if case == "baseline_sensors":
        return {"turret": tur, "mission_computer": mc, "payload_tray": tray}
    if case == "turret_only":
        return {"turret": tur}
    raise ValueError(case)


def mass_items(dv: DV, geo, L: Layout, struct, fixed, fuel_kg, payload_case="design"):
    """(mass, x, z) of every item for the CG."""
    ze = dv.z_engine
    wing_x = geo["x_le_mac"] + 0.42 * geo["mac"]
    tl = geo["tail"]
    g = geo["gear"]
    av_x = 0.5 * (L.av[0] + L.av[1])
    it = {
        "wing_structure": (sum(struct["wing"].values()), wing_x, 0.02),
        "tail_structure": (sum(struct["tail"].values()), tl["x_ac"] + 0.05, tl["z_ac"] - 0.05),
        "fuselage_shell": (struct["fuselage"]["shell_panels"] + struct["fuselage"].get("chine_edge_closeouts", 0.0),
                           geo["fus"]["x_shell_centroid"], 0.0),
        "fuselage_chassis": (sum(v for k, v in struct["fuselage"].items() if k not in ("shell_panels",
                             "chine_edge_closeouts")), geo["fus"]["x_shell_centroid"], -0.05),
        "structure_margin": (struct["maturity_margin"], 0.5 * (geo["fus"]["x_shell_centroid"] + wing_x), 0.0),
        "engine_group": (fixed["engine_group_installed"], dv.x_hub - 0.12, ze + 0.02),
        "propeller_spinner_spacer": (fixed["propeller"] + fixed["spinner_and_hub_adapter"] + fixed["prop_spacer"],
                                     L.x_prop, ze),
        "cowling_cooling": (fixed["cowling_cooling_ducts_baffles_firewall"], dv.x_hub - 0.22, ze + 0.05),
        "fuel_system": (fixed["fuel_system"], L.x_c, -0.05),
        "main_gear": (fixed["landing_gear"] * 0.66 + fixed.get("gear_fairings", 0.0) * 0.6 +
                      fixed.get("gear_bays_doors", 0.0) * 0.6, g["x_mg"], g["z_ground"] + 0.25),
        "nose_gear": (fixed["landing_gear"] * 0.34 + fixed.get("gear_fairings", 0.0) * 0.4 +
                      fixed.get("gear_bays_doors", 0.0) * 0.4, g["x_ng"], g["z_ground"] + 0.25),
        # 2 aileron DA 26 + 2 flap DA 30 in the wing, 2 ruddervator DA 26 in the tail, steering + brake DA 26
        "actuators_wing": (fixed["flight_control_actuators"] * 2.0 / 3.28, wing_x + 0.15, 0.02),
        "actuators_tail": (fixed["flight_control_actuators"] * 0.64 / 3.28, tl["x_ac"], tl["z_ac"] - 0.1),
        "actuators_steer_brake": (fixed["flight_control_actuators"] * 0.64 / 3.28, 0.5 * (g["x_ng"] + g["x_mg"]),
                                  -0.15),
        "avionics": (fixed["avionics"], av_x, 0.02),
        "battery_lifepo4": (BATTERY_M, 0.5 * (L.bat[0] + L.bat[1]), 0.0),
        "power_distribution": (fixed["electrical_power"] - BATTERY_M, av_x - 0.05, -0.03),
        "wiring": (fixed["wiring_harness_connectors_coax"], 0.45 * dv.x_hub, 0.0),
        "parachute_fts_lights": (fixed["recovery_and_safety"], 0.5 * (L.chute[0] + L.chute[1]), 0.06),
        "fuel": (fuel_kg, geo["tank_centroid_x"], -0.15),
    }
    for k, v in payload_items(payload_case, L).items():
        it["payload_" + k] = v
    return it


def cg_of(items):
    m = sum(v[0] for v in items.values())
    x = sum(v[0] * v[1] for v in items.values()) / m
    z = sum(v[0] * v[2] for v in items.values()) / m
    return m, x, z


# =====================================================================================================================
# 8. mission, performance
# =====================================================================================================================
def flight_point(W, V, h, pol: Polar, S):
    rho = AL.isa(h)["rho"]
    q = 0.5 * rho * V * V
    CL = W / (q * S)
    CD = pol.cd(CL)
    D = q * S * CD
    pp = PROP.at_thrust(D / INSTALL, V, h)
    if pp is None:
        return None
    P_tot = pp["P_shaft"] + P_GEN_SHAFT
    frac = P_tot / P_MAX
    b = bsfc_kgJ(frac)
    return {"CL": CL, "CD": CD, "LD": CL / CD, "D_N": D, "rpm": pp["rpm"], "P_prop_W": pp["P_shaft"],
            "P_total_W": P_tot, "power_frac": frac, "bsfc_g_kWh": b * 3.6e9, "fuel_kg_h": b * P_tot * 3600,
            "eta_installed": D * V / pp["P_shaft"], "bsfc_eff": b * P_tot / pp["P_shaft"], "V_tas": V, "h": h,
            "gen_output_W": GEN_RATED_W * pp["rpm"] / GEN_RATED_RPM, "torque_frac": pp["torque_frac"]}


def tas(V_eas, h):
    return V_eas / math.sqrt(AL.isa(h)["sigma"])


@functools.lru_cache(maxsize=None)
def wot_curve(h: float):
    """Full-throttle propeller operating line at altitude h on a 1 m/s grid (isolated propeller, generator load)."""
    Vg = np.arange(0.0, 91.0, 1.0)
    rows = [PROP.wot(V, h, P_GEN_SHAFT) for V in Vg]
    return (Vg, np.array([r["T"] for r in rows]), np.array([r["P_shaft"] for r in rows]),
            np.array([r["rpm"] for r in rows]))


def wot_at(V, h):
    Vg, T, P, n = wot_curve(round(float(h), 0))
    return float(np.interp(V, Vg, T)), float(np.interp(V, Vg, P)), float(np.interp(V, Vg, n))


def p_avail_fn(h, knock=KNOCK_STATIC):
    """Installed WOT thrust power T V (W) at TAS V (prop table, lapse, generator load, 0.92 data knock-down)."""
    Vg, T, _, _ = wot_curve(round(float(h), 0))
    return lambda V: float(np.interp(V, Vg, T)) * INSTALL * knock * V


def climb_point(W, h, pol: Polar, S, Vs_tas):
    rho = AL.isa(h)["rho"]
    roc, V = AL.rate_of_climb(W, S, pol.cd0, pol.k, p_avail_fn(h), rho, v_lo=1.25 * Vs_tas, v_hi=60.0)
    Tw, Pw, nw = wot_at(V, h)
    q = 0.5 * rho * V * V
    CL = W / (q * S)
    T = Tw * INSTALL * KNOCK_STATIC
    P_tot = Pw + P_GEN_SHAFT
    b = bsfc_kgJ(P_tot / P_MAX)
    return {"roc": roc, "V": V, "CL": CL, "LD": CL / pol.cd(CL), "eta": T * V / Pw,
            "bsfc_eff": b * P_tot / Pw, "rpm": nw, "fuel_kg_h": b * P_tot * 3600}


def fly_mission(m0, geo, pols, loiter_s=None, fuel_avail=None, n_loiter=24):
    """Design mission: warm-up/taxi/take-off, climb to 3000 m, 100 km out at 33 m/s EAS, loiter at 28 m/s EAS
    (3000 m), 100 km back, descent, landing, 1 h reserve loiter at 1000 m; 2 % trapped fuel (sizinglib default).
    loiter_s given -> fuel fraction; fuel_avail given -> loiter time that uses exactly that fuel."""
    S = geo["S"]
    if fuel_avail is not None:
        lo, hi = 0.0, 30 * 3600.0
        for _ in range(50):
            mid = 0.5 * (lo + hi)
            r = fly_mission(m0, geo, pols, loiter_s=mid, n_loiter=n_loiter)
            lo, hi = (mid, hi) if r["fuel_kg"] < fuel_avail else (lo, mid)
        return fly_mission(m0, geo, pols, loiter_s=0.5 * (lo + hi), n_loiter=n_loiter)
    segs, log = [], []
    W = m0 * G
    t_air = 0.0

    def add(seg, note, dt):
        nonlocal W, t_air
        f = SZ.segment_fraction(seg, W)
        segs.append(seg)
        log.append({"segment": note, "W_start_N": W, "fraction": f, "time_s": dt})
        W *= f
        t_air += dt
    for k in ("warmup", "taxi", "takeoff"):
        add(SZ.Segment(k, name=k), k, 0.0)
    # climb in 3 bands
    for h0, h1 in ((0, 1000), (1000, 2000), (2000, H_LOITER)):
        hm = 0.5 * (h0 + h1)
        cp = climb_point(W, hm, pols["SL"] if hm < 1500 else pols["ALT"], S, geo["Vs_tas"](W, hm))
        add(SZ.Segment("climb", value=h1 - h0, V=cp["V"], h=hm, LD=cp["LD"], eta=cp["eta"], bsfc=cp["bsfc_eff"],
                       gamma=cp["roc"] / cp["V"], name="climb"), f"climb {h0}-{h1} m", (h1 - h0) / cp["roc"])
    V_cr = tas(V_CRUISE_EAS, H_LOITER)

    def cruise(note):
        for _ in range(4):
            fp = flight_point(W, V_cr, H_LOITER, pols["ALT"], S)
            R = TRANSIT_KM * 1000 / 4
            add(SZ.Segment("cruise", value=R, V=V_cr, h=H_LOITER, LD=fp["LD"], eta=fp["eta_installed"],
                           bsfc=fp["bsfc_eff"]), note, R / V_cr)
    cruise("cruise out")
    V_l = tas(V_LOITER_EAS, H_LOITER)
    lo_pts = []
    dt = (loiter_s or 0.0) / n_loiter
    for _ in range(n_loiter):
        fp = flight_point(W, V_l, H_LOITER, pols["ALT"], S)
        lo_pts.append(fp)
        if dt > 0:
            add(SZ.Segment("loiter", value=dt, V=V_l, h=H_LOITER, LD=fp["LD"], eta=fp["eta_installed"],
                           bsfc=fp["bsfc_eff"]), "loiter", dt)
    cruise("cruise back")
    add(SZ.Segment("descent", name="descent"), "descent", H_LOITER / 2.5)
    W_land_nores = W
    V_r = tas(V_LOITER_EAS, 1000.0)
    for _ in range(4):
        fp = flight_point(W, V_r, 1000.0, pols["SL"], S)
        add(SZ.Segment("reserve", value=RESERVE_H * 3600 / 4, V=V_r, h=1000.0, LD=fp["LD"], eta=fp["eta_installed"],
                       bsfc=fp["bsfc_eff"]), "reserve 1000 m", 0.0)
    add(SZ.Segment("landing", name="landing"), "landing", 0.0)
    ff, fr = SZ.mission_fuel_fraction(segs, trapped=0.02)
    return {"ff": ff, "fuel_kg": ff * m0, "segments": log, "block_time_h": t_air / 3600.0,
            "loiter_s": loiter_s or 0.0, "loiter_points": lo_pts, "W_land_N": W, "W_land_noreserve_N": W_land_nores}


def mission_for_endurance(m0, geo, pols, target_h=ENDURANCE_H):
    """Loiter time such that the block time (take-off to landing, reserve excluded) equals the target."""
    r0 = fly_mission(m0, geo, pols, loiter_s=0.0)
    t_loiter = max(0.0, target_h * 3600 - r0["block_time_h"] * 3600)
    return fly_mission(m0, geo, pols, loiter_s=t_loiter)


# =====================================================================================================================
# 9. geometry assembly + closed loop
# =====================================================================================================================
def gust_limit(dv: DV, a_w, mac):
    ws = dv.ws_kg_m2 * G
    n = N_POS
    rows = {}
    for h in (0.0, 4500.0):
        r = ST.vn_diagram(ws, 1.4, -0.8, N_POS, N_NEG, VC, VD, a_w, mac, UDE_C, UDE_D, rho=AL.isa(h)["rho"])
        rows[f"h{int(h)}"] = {"n_pos": r["n_limit_pos"], "n_neg": r["n_limit_neg"], "gust_C": r["gust_C"],
                              "gust_D": r["gust_D"], "kg": r["kg"]}
        n = max(n, r["n_limit_pos"])
    return n, rows


def gear_layout(dv: DV, L: Layout, F, cg_aft, cg_fwd, z_cg, theta_req_deg, ventral):
    """Ground plane and main/nose gear stations: static propeller clearance (CS-VLA 925(a)), tail-strike angle with
    the ventral skid >= required rotation, tip-back >= max(15 deg, tail-strike), nose load 8-20 %."""
    R = D_PROP / 2
    z_g_prop = dv.z_engine - R - PROP_GROUND_CLEAR
    turret_bottom = L.z_mount - (E180_H - est("turret_inside_m"))
    z_g_turret = turret_bottom - 0.15
    r_w = 0.095
    stroke = est("gear_stroke_m")
    x_guess = cg_aft + 0.30
    wg, hg, zcg_, *_ = (float(v) for v in F.section(x_guess))
    z_belly_guess = zcg_ - (1 - float(F.top_frac(x_guess))) * hg
    z_g_stroke = z_belly_guess - r_w - stroke - 0.05
    best = None
    for nose_max in (0.20, 0.25, 0.30, 0.40, 0.60):          # relaxed only while the outer loop converges
        for z_g in np.arange(min(z_g_prop, z_g_turret, z_g_stroke), -1.6, -0.005):
            for x_mg in np.arange(cg_aft + 0.05, cg_aft + 0.9, 0.005):
                tb = math.degrees(math.atan2(x_mg - cg_aft, z_cg - z_g))
                ts = math.degrees(math.atan2(ventral["z_skid"] - z_g, ventral["x_skid"] - x_mg))
                if ts < theta_req_deg or tb < max(15.0, ts):
                    continue
                nose_fwd = (x_mg - cg_fwd) / (x_mg - L.x_ng)
                nose_aft = (x_mg - cg_aft) / (x_mg - L.x_ng)
                if nose_fwd > nose_max or nose_aft < 0.08:
                    continue
                best = (z_g, x_mg, tb, ts, nose_fwd, nose_aft, nose_max)
                break
            if best:
                break
        if best:
            break
    if best is None:
        raise RuntimeError("no gear layout satisfies clearance/tail-strike/tip-back/nose-load rules")
    z_g, x_mg, tb, ts, nf, na, nose_max = best
    wm, hm, zcm, *_ = (float(v) for v in F.section(x_mg))
    tfm = float(F.top_frac(x_mg))
    z_belly_m = zcm - (1 - tfm) * hm
    wn, hn, zcn, *_ = (float(v) for v in F.section(L.x_ng))
    z_belly_n = zcn - (1 - float(F.top_frac(L.x_ng))) * hn
    h_main = z_belly_m - (z_g + r_w)
    h_nose = z_belly_n - (z_g + r_w)
    # track for an overturn angle <= 55 deg (Raymer limit 63 deg; margin for crosswind taxi)
    h_cg = z_cg - z_g
    track = 0.6
    for track in np.arange(0.6, 2.5, 0.01):
        # distance from CG to the nose-wheel/main-wheel line in the ground plane
        ax, ay = L.x_ng, 0.0
        bx, by = x_mg, track / 2
        px, py = cg_fwd, 0.0
        d = abs((bx - ax) * (ay - py) - (ax - px) * (by - ay)) / math.hypot(bx - ax, by - ay)
        psi = math.degrees(math.atan2(h_cg, d))
        if psi <= 55.0:
            break
    return {"z_ground": z_g, "x_mg": x_mg, "x_ng": L.x_ng, "tipback_deg": tb, "tailstrike_deg": ts,
            "nose_load_frac_fwd_cg": nf, "nose_load_frac_aft_cg": na, "nose_load_limit_used": nose_max, "h_main_leg": h_main, "h_nose_leg": h_nose,
            "track": track, "overturn_deg": psi, "wheel_radius": r_w, "wheelbase": x_mg - L.x_ng,
            "prop_static_clearance": dv.z_engine - R - z_g, "turret_static_clearance": turret_bottom - z_g,
            "z_belly_main": z_belly_m, "z_belly_nose": z_belly_n}


def build_geometry(dv: DV, m0: float, x_w: float, y_s: float, i_w: float, gear_prev=None, cg_prev=None,
                   theta_req=12.0):
    P = planform(dv, m0)
    c_s = chord_at(P, y_s)
    secs_a = wing_sections(dv, P, x_w, y_s, 0.0, i_w, centreline=True)
    for s in secs_a:                          # analysis: flat reference line through the centre section
        s["z_le"] = max(0.0, s["y"] - y_s) * math.tan(math.radians(dv.dihedral_deg))
    macd = oml.mean_aerodynamic_chord(secs_a)
    L = Layout(dv, x_w, c_s, y_s, macd["x_le_mac"] + 0.21 * macd["mac"])
    F, st, ko = build_fuselage(dv, L)
    fz = fuselage_props(F, st)
    y_s_new = float(F.section(x_w + 0.3 * c_s)[0]) / 2
    z_root = float(L.zc(x_w + 0.3 * c_s)) - 0.015 * c_s
    secs = wing_sections(dv, P, x_w, y_s, z_root, i_w)
    tanle = math.tan(math.radians(dv.sweep_le_deg))
    sweep_c4 = math.degrees(math.atan(tanle - (P["c_r0"] - P["c_t"]) / (4 * P["semispan"])))
    sweep_c2 = math.degrees(math.atan(tanle - (P["c_r0"] - P["c_t"]) / (2 * P["semispan"])))
    x_ac_w = macd["x_le_mac"] + 0.25 * macd["mac"]
    S_exp = P["S"] - (P["c_r0"] + c_s) * y_s

    def top(x):
        w, h, z0, *_ = (float(v) for v in F.section(x))
        return z0 + float(F.top_frac(x)) * h

    def bottom(x):
        w, h, z0, *_ = (float(v) for v in F.section(x))
        return z0 - (1 - float(F.top_frac(x))) * h
    ventral = ventral_geometry(dv, L, bottom)
    x_cg_ref = cg_prev if cg_prev is not None else x_ac_w - 0.15 * macd["mac"]
    # tail sizing by volume coefficients (V_V counts the ventral fin's S l)
    S_h, S_v = 0.15 * P["S"], 0.10 * P["S"]
    for _ in range(30):
        tl = tail_geometry(dv, S_h, S_v, L.x_prop, top)
        l_h = max(tl["x_ac"] - x_ac_w, 0.5)
        l_v = max(tl["x_ac"] - x_cg_ref, 0.5)
        S_h_new = dv.V_H * P["S"] * macd["mac"] / l_h
        vent_term = ventral["S"] * (ventral["x_ac"] - x_cg_ref) if dv.tail == "Y" else 0.0
        S_v_new = max(0.02, (dv.V_V * P["S"] * P["b"] - vent_term) / l_v)
        if abs(S_h_new - S_h) < 1e-5 and abs(S_v_new - S_v) < 1e-5:
            break
        S_h, S_v = S_h_new, S_v_new
    tl["sweep_tmax_deg"] = math.degrees(math.atan(math.tan(math.radians(dv.tail_sweep_le_deg)) -
                                                  0.3 * (tl["c_root"] - tl["c_tip"]) / tl["span_panel"]))
    tl["l_h"], tl["l_v"] = l_h, l_v
    # fillet: LERX-like blend from the chine 40 % root chord ahead of the LE to 0.10 m outboard on the LE
    fl_len, fl_w = 0.40 * c_s, 0.10
    fillet_S_wet = 2 * 2 * 0.5 * fl_len * fl_w * 1.3
    # aft cowl base: section area at the cylinders minus cooling exit and spinner disc
    xs_c = dv.x_hub - 0.10
    A_cyl = float(np.interp(xs_c, fz["sections"][:, 0], fz["sections"][:, 3]))
    A_exit = val(ENG["cooling"]["exit_area_m2"])
    A_spin = math.pi / 4 * est("spinner_dia_m") ** 2
    base_area = max(0.0, A_cyl - A_exit - A_spin)
    # bladder conformal volumes (inside the shell inset by the keep-out clearance, below the parachute/floor)
    def conformal(x0, x1, z_top):
        xs_t = np.linspace(x0, x1, 25)
        A = []
        c = est("keepout_clearance_m")
        for x in xs_t:
            w, h, z0, nt, nb = (float(v) for v in F.section(x))
            tf = float(F.top_frac(x))
            a, bb, bt = w / 2 - c, (1 - tf) * h - c, tf * h - c
            zz = np.linspace(z0 - bb, z_top, 80)
            dz = zz - z0
            n = np.where(dz >= 0, nt, nb)
            bh = np.where(dz >= 0, bt, bb)
            width = 2 * a * np.clip(1 - (np.abs(dz) / bh) ** n, 0, 1) ** (1 / n)
            A.append(float(np.trapz(width, zz)))
        V = float(np.trapz(A, xs_t))
        xc = float(np.trapz(np.array(A) * xs_t, xs_t) / max(np.trapz(A, xs_t), 1e-9))
        return V, xc
    c = est("keepout_clearance_m")
    V_f, x_f = conformal(*L.tank_f, 0.03 - c)              # below the parachute container floor
    V_a, x_a = conformal(*L.tank_a, 0.03 - c)
    vol = V_f + V_a
    tank_cx = (V_f * x_f + V_a * x_a) / vol
    retract_dL = 0.0
    geo = {"planform": P, "S": P["S"], "b": P["b"], "mac": macd["mac"], "x_le_mac": macd["x_le_mac"],
           "y_mac": macd["y_mac"], "x_ac_w": x_ac_w, "c_s": c_s, "y_s": y_s, "y_s_fus": y_s_new, "x_w": x_w,
           "z_root": z_root, "i_w": i_w, "wing_sections": secs, "wing_sections_analysis": secs_a, "S_exp": S_exp,
           "sweep_c4_deg": sweep_c4, "sweep_c2_deg": sweep_c2, "F": F, "stations": st, "keepouts": ko, "fus": fz,
           "layout": L, "tail": tl, "ventral": ventral, "fillet_S_wet": fillet_S_wet if dv.fillets else 0.0,
           "fillet": (fl_len, fl_w), "base_area": base_area, "A_cyl": A_cyl, "tank_volume_m3": vol,
           "tank_volumes_m3": (V_f, V_a), "tank_centroids_x": (x_f, x_a),
           "tank_centroid_x": tank_cx, "top": top, "bottom": bottom}
    geo["gear"] = gear_prev or {"z_ground": -0.7, "x_mg": x_w + 0.8, "x_ng": L.x_ng, "h_main_leg": 0.35,
                                "h_nose_leg": 0.4, "track": 1.0}
    if dv.gear == "retract":
        g = geo["gear"]
        well = (2 * (g["h_main_leg"] + 0.1) + (g["h_nose_leg"] + 0.1)) * 0.19 * 0.085 * 1.3
        A_use = 0.10                                    # EST usable belly cross-section for wells (m2)
        geo["retract_well_volume_m3"] = well
        geo["retract_dL"] = well / A_use
    return geo


def evaluate(dv: DV, m0_guess=140.0, verbose=False, fixed_mtom=None):
    """Closed loop: geometry -> aero -> mission -> MassModel -> m0 (and wing position for the SM target)."""
    m0 = m0_guess
    x_w, y_s, i_w = 1.45, 0.25, 4.0
    gear, cg_aft, cg_fwd, z_cg, cg_ref = None, None, None, 0.0, None
    hist = []
    for it in range(30):
        geo = build_geometry(dv, m0, x_w, y_s, i_w, gear, cg_ref)
        y_s = 0.5 * y_s + 0.5 * geo["y_s_fus"]
        V_sl = 30.0
        V_alt = tas(V_LOITER_EAS, H_LOITER)
        sa0, _, _ = wing_aero(dv, geo, V_alt, H_LOITER)
        # incidence: body level at CL 0.8 (between loiter ~1.0 and cruise ~0.65)
        a08 = math.degrees(sa0["alpha_for"](0.8))
        i_w = i_w + 0.8 * a08
        pols = {"SL": aircraft_polar(dv, geo, V_sl, 0.0), "ALT": aircraft_polar(dv, geo, V_alt, H_LOITER)}
        a_w = pols["ALT"].sa["CL_alpha"] * pols["ALT"].k_a
        n_lim, gust_rows = gust_limit(dv, a_w, geo["mac"])
        geo["n_ult"] = FOS * n_lim
        geo["n_lim"] = n_lim
        geo["gust"] = gust_rows
        # stall speed helper (trimmed clean CLmax, refined below)
        clmax_w = pols["SL"].sa["CLmax"]
        geo["Vs_tas"] = lambda W, h, c=clmax_w * 0.95: math.sqrt(2 * W / (AL.isa(h)["rho"] * geo["S"] * c))
        if fixed_mtom is None:
            mis = mission_for_endurance(m0, geo, pols)
            ff = mis["ff"]
        else:
            ff = None
        fixed = fixed_equipment(dv, geo)
        L = geo["layout"]
        S_ref_m0 = geo["S"]

        def airframe_fn(m, geo=geo, L=L):
            return structure(dv, geo, m, L)["total"] / m
        if fixed_mtom is None:
            sol = SZ.MassModel(PAYLOAD_DESIGN, sum(fixed.values()), ff, airframe_fn).solve(m0)
            m_new = sol["mtow"]
        else:
            m_new = fixed_mtom
            sol = None
        struct = structure(dv, geo, m_new, L)
        if fixed_mtom is not None:
            fuel_kg = m_new - struct["total"] - sum(fixed.values()) - PAYLOAD_DESIGN
        else:
            fuel_kg = sol["fuel"]
        # CG cases and wing position for the static-margin target
        stab = stability(dv, geo, pols, struct, fixed, fuel_kg, m_new)
        dx = (stab["sm_min"] - dv.sm_min_target) * geo["mac"]
        # moving the wing aft by d moves NP ~0.85 d and CG ~0.3 d aft; damped, step-limited update
        x_w_new = x_w - float(np.clip(dx * 1.2, -0.06, 0.06))
        cg_aft, cg_fwd, z_cg = stab["x_cg_aft"], stab["x_cg_fwd"], stab["z_cg_mtow"]
        cg_ref = stab["x_cg_mtow"]
        # tail-strike requirement: body angle at 0.9 CLmax(TO flaps) in ground effect ignored (conservative)
        hl = high_lift(dv, geo, pols, stab)
        theta_req = hl["alpha_body_rot_deg"]
        gear = gear_layout(dv, L, geo["F"], cg_aft, cg_fwd, z_cg, theta_req, geo["ventral"])
        hist.append((it, m_new, x_w, stab["sm_min"], y_s, i_w))
        if verbose:
            print(f"  it {it}: m0 {m_new:7.2f} kg  x_w {x_w:.3f}  SMmin {stab['sm_min']:.3f}  y_s {y_s:.3f}  "
                  f"i_w {i_w:.2f}  gear z {gear['z_ground']:.3f}")
        conv = abs(m_new - m0) < 0.02 and abs(dx) < 0.002 and it > 3
        m0, x_w = m_new, x_w_new
        if conv:
            break
    geo = build_geometry(dv, m0, x_w, y_s, i_w, gear, cg_ref)
    geo["n_ult"], geo["n_lim"], geo["gust"] = FOS * n_lim, n_lim, gust_rows
    pols = {"SL": aircraft_polar(dv, geo, 30.0, 0.0), "ALT": aircraft_polar(dv, geo, tas(V_LOITER_EAS, H_LOITER),
                                                                             H_LOITER)}
    clmax_w = pols["SL"].sa["CLmax"]
    geo["Vs_tas"] = lambda W, h, c=clmax_w * 0.95: math.sqrt(2 * W / (AL.isa(h)["rho"] * geo["S"] * c))
    fixed = fixed_equipment(dv, geo)
    L = geo["layout"]
    struct = structure(dv, geo, m0, L)
    if fixed_mtom is None:
        mis = mission_for_endurance(m0, geo, pols)
        fuel_kg = mis["fuel_kg"]
    else:
        fuel_kg = m0 - struct["total"] - sum(fixed.values()) - PAYLOAD_DESIGN
        mis = fly_mission(m0, geo, pols, fuel_avail=fuel_kg)
    stab = stability(dv, geo, pols, struct, fixed, fuel_kg, m0)
    hl = high_lift(dv, geo, pols, stab)
    return {"dv": dv, "m0": m0, "geo": geo, "pols": pols, "fixed": fixed, "struct": struct, "fuel_kg": fuel_kg,
            "mission": mis, "stab": stab, "hl": hl, "hist": hist,
            "empty_kg": m0 - fuel_kg - PAYLOAD_DESIGN}


# =====================================================================================================================
# 10. stability, high lift
# =====================================================================================================================
def stability(dv: DV, geo, pols, struct, fixed, fuel_kg, m0):
    L = geo["layout"]
    pol = pols["ALT"]
    a_w = pol.sa["CL_alpha"] * pol.k_a
    tl = geo["tail"]
    fz = geo["fus"]
    P = geo["planform"]
    h_h = tl["z_ac"] - geo["z_root"]
    deps = SL.downwash_gradient(dv.AR, dv.taper, tl["l_h"], h_h, P["b"], geo["sweep_c4_deg"])
    x_c4_root = geo["wing_sections_analysis"][0]["x_le"] + 0.25 * P["c_r0"]
    Kf = SL.kf_fuselage(x_c4_root / fz["length"])
    cma_f = SL.cm_alpha_fuselage(Kf, fz["w_max"], fz["length"], geo["mac"], geo["S"])
    if dv.tail == "Y":
        a_t, S_t = tl["a_per_rad"], tl["S_h_eff"]
    else:
        a_t, S_t = AL.lift_slope(4.5, 20.0), tl["S_h_eff"]
    x_np = SL.neutral_point(a_w, geo["x_ac_w"], a_t, S_t, tl["x_ac"], geo["S"], geo["mac"], deps, 0.9, cma_f)
    cases = {}
    full = fuel_kg
    for name, ffrac, pc in (("mtow_full_fuel_design_payload", 1.0, "design"),
                            ("full_fuel_baseline_sensors_only", 1.0, "baseline_sensors"),
                            ("zero_fuel_design_payload", 0.0, "design"),
                            ("minimum_flying", 0.1, "turret_only"),
                            ("e180_growth_full_fuel", 1.0, "e180_growth"),
                            ("e180_growth_zero_fuel", 0.0, "e180_growth")):
        it = mass_items(dv, geo, L, struct, fixed, full * ffrac, pc)
        m, x, z = cg_of(it)
        cases[name] = {"mass_kg": m, "x_cg": x, "z_cg": z, "fuel_kg": full * ffrac,
                       "sm": SL.static_margin(x_np, x, geo["mac"]),
                       "x_cg_pct_mac": (x - geo["x_le_mac"]) / geo["mac"] * 100}
    sms = [c["sm"] for c in cases.values()]
    xs = [c["x_cg"] for c in cases.values()]
    items_mtow = mass_items(dv, geo, L, struct, fixed, full, "design")
    # directional stability
    m_mtow, x_cg, z_cg = cg_of(items_mtow)
    if dv.tail == "Y":
        cnb_v = SL.cn_beta_vertical(tl["a_per_rad"], tl["S_v_eff"], tl["x_ac"] - x_cg, geo["S"], P["b"])
        vf = geo["ventral"]
        cnb_vent = SL.cn_beta_vertical(vf["a_per_rad"], vf["S"], vf["x_ac"] - x_cg, geo["S"], P["b"])
    else:
        cnb_v = SL.cn_beta_vertical(AL.lift_slope(3.0, 25.0), tl["S_v_eff"], tl["x_ac"] - x_cg, geo["S"], P["b"])
        cnb_vent = 0.0
    cnb_f_datcom = SL.cn_beta_fuselage(est("K_N_datcom"), est("K_Rl_datcom"), fz["S_side"], fz["length"], geo["S"],
                                       P["b"])
    cnb_f_raymer = -1.3 * fz["volume"] / (geo["S"] * P["b"]) * (fz["h_max"] / fz["w_max"])
    cnb_f = min(cnb_f_datcom, cnb_f_raymer)
    vols = SL.tail_volumes(geo["S"], geo["mac"], P["b"], tl["S_h_eff"], tl["l_h"],
                           tl["S_v_eff"] + (geo["ventral"]["S"] * (geo["ventral"]["x_ac"] - x_cg) / tl["l_v"]
                                            if dv.tail == "Y" else 0.0), tl["l_v"])
    return {"x_np": x_np, "a_w": a_w, "deps_da": deps, "Kf": Kf, "cm_alpha_fus": cma_f, "a_t": a_t, "S_t": S_t,
            "cases": cases, "sm_min": min(sms), "sm_max": max(sms), "x_cg_fwd": min(xs), "x_cg_aft": max(xs),
            "x_cg_mtow": x_cg, "z_cg_mtow": z_cg, "items_mtow": items_mtow, "cn_beta": {
                "v_tail": cnb_v, "ventral": cnb_vent, "fuselage_datcom": cnb_f_datcom, "fuselage_raymer": cnb_f_raymer,
                "fuselage_used": cnb_f, "total": cnb_v + cnb_vent + cnb_f}, "volumes": vols, "h_h": h_h}


def high_lift(dv: DV, geo, pols, stab):
    """Trimmed CLmax (forward CG) clean / take-off / landing, plain flaps 25 % chord side-of-body to eta 0.58."""
    P = geo["planform"]
    sa = pols["SL"].sa
    clmax_w = sa["CLmax"]
    eta_f1 = 0.58
    yb = np.linspace(geo["y_s"], eta_f1 * P["semispan"], 50)
    S_flapped = 2 * float(np.trapz([chord_at(P, y) for y in yb], yb))
    sweep_hl = math.degrees(math.atan(math.tan(math.radians(dv.sweep_le_deg)) - 0.75 * (P["c_r0"] - P["c_t"]) /
                                      P["semispan"]))
    dCL_ld = 0.9 * est("flap_dClmax_plain") * S_flapped / geo["S"] * math.cos(math.radians(sweep_hl))
    dCL_to = est("flap_TO_fraction") * dCL_ld
    x_fwd = min(c["x_cg"] for c in stab["cases"].values() if c["fuel_kg"] >= 0.999 * max(
        cc["fuel_kg"] for cc in stab["cases"].values()))                      # most forward CG at MTOW weight
    tl = geo["tail"]

    def trimmed(CLw, dcm):
        # moment balance about the CG: CLw (x_cg - x_ac) / c + Cm_ac + CLt S_t/S (x_cg - x_act)/c = 0
        cm = CLw * (x_fwd - geo["x_ac_w"]) / geo["mac"] + CM_AC_WING + dcm
        CLt_St = -cm * geo["mac"] / (x_fwd - tl["x_ac"])       # = CL_t S_t / S (dimensionless)
        return CLw + CLt_St, CLt_St
    cl_clean, clt_clean = trimmed(clmax_w, 0.0)
    cl_to, clt_to = trimmed(clmax_w + dCL_to, est("flap_dCm_per_dCL") * dCL_to)
    cl_ld, clt_ld = trimmed(clmax_w + dCL_ld, est("flap_dCm_per_dCL") * dCL_ld)
    # tail authority: landing trim at the most forward CG of all cases (zero fuel, E180 growth), full flaps
    x_all = stab["x_cg_fwd"]
    cm_ld = (clmax_w + dCL_ld) * (x_all - geo["x_ac_w"]) / geo["mac"] + CM_AC_WING + est("flap_dCm_per_dCL") * dCL_ld
    clt_ld_fwd = -cm_ld * geo["mac"] / (x_all - tl["x_ac"]) * geo["S"] / tl["S_h_eff"]
    # rotation attitude: wing at 0.9 CLmax(TO) (flap increment as a zero-lift shift)
    a_w = sa["CL_alpha"] * pols["SL"].k_a
    alpha_rot = math.degrees(sa["alpha_for"](0.9 * clmax_w))
    cf = 0.25
    dcd_ld = 0.9 * cf ** 1.38 * S_flapped / geo["S"] * math.sin(math.radians(40.0)) ** 2
    dcd_to = 0.9 * cf ** 1.38 * S_flapped / geo["S"] * math.sin(math.radians(15.0)) ** 2
    return {"x_cg_fwd_mtow": x_fwd, "tail_CL_landing_trim_fwd_cg": clt_ld_fwd,
            "tail_CL_stall_trim_clean": clt_clean * geo["S"] / tl["S_h_eff"],
            "tail_CL_limit": est("tail_CLmax_down"),
            "CLmax_wing_untrimmed": clmax_w, "CLmax_clean_trimmed": cl_clean, "CLmax_TO_trimmed": cl_to,
            "CLmax_LD_trimmed": cl_ld, "dCLmax_flap_LD": dCL_ld, "dCLmax_flap_TO": dCL_to, "S_flapped": S_flapped,
            "dCD0_flap_TO": dcd_to, "dCD0_flap_LD": dcd_ld, "alpha_body_rot_deg": alpha_rot + 1.0,
            "stall_onset_eta": sa["stall_onset_eta"], "alpha_stall_body_deg": sa["alpha_stall_deg"]}


# =====================================================================================================================
# 11. performance
# =====================================================================================================================
def ground_roll_cl(res):
    """Wing CL in the level ground attitude with take-off flaps (zero-lift shift ~ flap dCLmax), capped below the
    lift-off CL so the aircraft does not leave the ground before V_LOF."""
    sa, hl = res["pols"]["SL"].sa, res["hl"]
    return min(sa["CL_0"] + 0.8 * hl["dCLmax_flap_TO"], 0.85 * hl["CLmax_TO_trimmed"] / 1.21)


def raymer_takeoff_air(W, S, clmax_to, pol: Polar, T_lof, rho=1.225, h_obs=15.0):
    """Raymer 17.8: transition arc at V_TR = 1.15 V_S, n = 1.2, then steady climb to the 15 m obstacle."""
    Vs = AL.stall_speed(W, S, clmax_to, rho)
    V_tr = 1.15 * Vs
    R = V_tr ** 2 / (G * 0.2)
    CL = W / (0.5 * rho * V_tr ** 2 * S)
    D = 0.5 * rho * V_tr ** 2 * S * pol.cd(CL)
    gam = max((T_lof - D) / W, 1e-3)
    h_tr = R * (1 - math.cos(gam))
    if h_tr >= h_obs:
        return math.sqrt(R ** 2 - (R - h_obs) ** 2), gam
    return R * math.sin(gam) + (h_obs - h_tr) / math.tan(gam), gam


def performance(res):
    dv, geo, pols, hl, stab = res["dv"], res["geo"], res["pols"], res["hl"], res["stab"]
    m0, S = res["m0"], geo["S"]
    W = m0 * G
    out = {}
    rho0 = 1.225
    pol0, pola = pols["SL"], pols["ALT"]
    Vs = AL.stall_speed(W, S, hl["CLmax_clean_trimmed"], rho0)
    Vs_to = AL.stall_speed(W, S, hl["CLmax_TO_trimmed"], rho0)
    Vs_ld = AL.stall_speed(W, S, hl["CLmax_LD_trimmed"], rho0)
    out["V_stall_clean_m_s"] = Vs
    out["V_stall_TO_flap_m_s"] = Vs_to
    out["V_stall_LD_flap_m_s"] = Vs_ld
    for tag, pol, h in (("SL", pol0, 0.0), ("3000m", pola, H_LOITER)):
        rho = AL.isa(h)["rho"]
        sp = AL.speeds(W, S, pol.cd0, pol.k, rho)
        out[f"V_min_power_tas_{tag}_m_s"] = sp["V_min_power"]
        out[f"V_ld_max_tas_{tag}_m_s"] = sp["V_ld_max"]
        out[f"LD_max_parabolic_{tag}"] = sp["LD_max"]
        out[f"LD_max_table_{tag}"] = pol.ld_max
        out[f"CL_LD_max_{tag}"] = pol.cl_ld_max
        out[f"CL_endurance_max_{tag}"] = pol.cl_end_max
        Vh = AL.max_level_speed(W, S, pol.cd0, pol.k, p_avail_fn(h), rho, v_lo=25.0, v_hi=90.0)
        out[f"V_max_level_tas_{tag}_m_s"] = Vh
        Vst = AL.stall_speed(W, S, hl["CLmax_clean_trimmed"], rho)
        roc, vroc = AL.rate_of_climb(W, S, pol.cd0, pol.k, p_avail_fn(h), rho, v_lo=1.25 * Vst, v_hi=60.0)
        out[f"RoC_max_{tag}_m_s"] = roc
        out[f"V_best_climb_tas_{tag}_m_s"] = vroc
    # ceilings
    def roc_at(h):
        rho = AL.isa(h)["rho"]
        Vst = AL.stall_speed(W, S, hl["CLmax_clean_trimmed"], rho)
        return AL.rate_of_climb(W, S, pola.cd0, pola.k, p_avail_fn(h), rho, v_lo=1.25 * Vst, v_hi=70.0)[0]
    for name, target in (("service_ceiling_m", 0.5), ("absolute_ceiling_m", 0.0)):
        lo, hi = 0.0, 9000.0
        if roc_at(hi) > target:
            out[name] = hi
            continue
        for _ in range(22):
            mid = round(0.5 * (lo + hi), -1)
            lo, hi = (mid, hi) if roc_at(mid) > target else (lo, mid)
            if hi - lo <= 10.0:
                break
        out[name] = 0.5 * (lo + hi)
    # take-off (ISA sea level, paved mu 0.04, TO flaps 15 deg)
    stat = PROP.wot(0.0, 0.0, P_GEN_SHAFT)
    T_static = stat["T"] * KNOCK_STATIC
    V_lof = 1.1 * Vs_to
    T_lof = PROP.wot(V_lof, 0.0, P_GEN_SHAFT)["T"] * KNOCK_STATIC * INSTALL
    cl_roll = ground_roll_cl(res)
    to = AL.takeoff_ground_roll(W, S, hl["CLmax_TO_trimmed"], pol0.cd0 + hl["dCD0_flap_TO"], pol0.k, T_static, T_lof,
                                mu=0.04, rho=rho0, cl_roll=cl_roll, v_lof_factor=1.1)
    s_air, gam = raymer_takeoff_air(W, S, hl["CLmax_TO_trimmed"], pol0, T_lof)
    # rotation at V_LOF about the main-wheel contact, forward CG at MTOW, take-off flaps, full power
    g = geo["gear"]
    tl = geo["tail"]
    q = 0.5 * rho0 * V_lof ** 2
    x_cg = hl["x_cg_fwd_mtow"]
    L_w = q * S * cl_roll
    M_ac = q * S * geo["mac"] * (CM_AC_WING + est("flap_dCm_per_dCL") * hl["dCLmax_flap_TO"])
    h_T = dv.z_engine - g["z_ground"]
    M_rest = (x_cg - g["x_mg"]) * W + (g["x_mg"] - geo["x_ac_w"]) * L_w + M_ac - h_T * T_lof
    clt_rot = M_rest / (tl["x_ac"] - g["x_mg"]) / (q * tl["S_h_eff"])
    # trim change with full power in the climb (thrust line above the CG)
    Vc = out["V_best_climb_tas_SL_m_s"]
    qc = 0.5 * rho0 * Vc ** 2
    Tc = wot_at(Vc, 0.0)[0] * INSTALL * KNOCK_STATIC
    dcm_T = -Tc * (dv.z_engine - res["stab"]["z_cg_mtow"]) / (qc * S * geo["mac"])
    dclt_T = -dcm_T * geo["mac"] / tl["l_h"] * S / tl["S_h_eff"]
    out.update({"rotation_tail_CL_required": clt_rot, "rotation_ok": clt_rot >= -est("tail_CLmax_down"),
                "ground_roll_CL": cl_roll, "power_trim_dCm_climb": dcm_T, "power_trim_tail_dCL": dclt_T,
                "tail_CL_landing_trim_fwd_cg": hl["tail_CL_landing_trim_fwd_cg"],
                "landing_trim_ok": abs(hl["tail_CL_landing_trim_fwd_cg"]) <= est("tail_CLmax_down")})
    out.update({"static_thrust_model_N": stat["T"], "static_rpm": stat["rpm"], "static_tip_mach": stat["tip_mach_helical"],
                "static_thrust_sizing_N": T_static, "T_lof_N": T_lof, "V_lof_m_s": to["V_lof"],
                "TO_ground_roll_m": to["ground_roll"], "TO_air_distance_15m_m": s_air,
                "TO_distance_15m_m": to["ground_roll"] + s_air, "TO_climb_gradient": gam})
    # landing (MTOW abort-landing case and normal landing with reserve fuel)
    for tag, Wl in (("mtow", W), ("normal", res["mission"]["W_land_noreserve_N"])):
        ld = AL.landing_roll(Wl, S, hl["CLmax_LD_trimmed"], pol0.cd0 + hl["dCD0_flap_LD"], pol0.k, mu_brake=0.3,
                             rho=rho0, v_td_factor=1.15, cl_roll=0.3, t_free=1.0)
        out[f"landing_ground_roll_{tag}_m"] = ld["ground_roll"]
        out[f"V_touchdown_{tag}_m_s"] = ld["V_td"]
    # loiter and cruise points
    for tag, Ve, h in (("loiter_3000m", V_LOITER_EAS, H_LOITER), ("loiter_SL", V_LOITER_EAS, 0.0),
                       ("cruise_3000m", V_CRUISE_EAS, H_LOITER)):
        pol = pola if h > 1500 else pol0
        fp = flight_point(W, tas(Ve, h), h, pol, S)
        out[f"{tag}_point"] = {k: v for k, v in fp.items()}
    # endurance / range with the design fuel
    mis = res["mission"]
    out["design_mission_block_time_h"] = mis["block_time_h"]
    out["design_mission_loiter_h"] = mis["loiter_s"] / 3600
    fuel_full_tank = min(res["geo"]["tank_volume_m3"], 0.045) * RHO_FUEL * 0.98
    out["fuel_capacity_kg"] = fuel_full_tank
    # pure loiter at 3000 m, all usable fuel minus 1 h reserve and climb
    out["range_km_best"] = breguet_range(res, res["fuel_kg"])
    return out


def breguet_range(res, fuel_kg):
    """One-way still-air range at the best-L/D speed at 3000 m: climb + cruise (constant CL) + 1 h reserve."""
    geo, pols = res["geo"], res["pols"]
    S, m0 = geo["S"], res["m0"]
    W = m0 * G
    W_end_min = (m0 - fuel_kg / 1.02) * G
    # climb + reserve fuel from the design mission log
    seg = res["mission"]["segments"]
    w_climb = np.prod([s["fraction"] for s in seg if s["segment"].startswith(("warmup", "taxi", "takeoff", "climb"))])
    w_res = np.prod([s["fraction"] for s in seg if s["segment"].startswith(("reserve", "descent", "landing"))])
    W = W * w_climb
    pol = pols["ALT"]
    R = 0.0
    rho = AL.isa(H_LOITER)["rho"]
    CL = pol.cl_ld_max
    while W * w_res > W_end_min:
        V = math.sqrt(2 * W / (rho * S * CL))
        fp = flight_point(W, V, H_LOITER, pol, S)
        dW = fp["fuel_kg_h"] * G / 3600 * 60.0
        R += V * 60.0
        W -= dW
    return R / 1000.0


# =====================================================================================================================
# 12. constraint diagram
# =====================================================================================================================
def constraints(res):
    geo, pols, hl, dv = res["geo"], res["pols"], res["hl"], res["dv"]
    pol0, pola = pols["SL"], pols["ALT"]
    W = res["m0"] * G
    a0 = SZ.Aero(cd0=pol0.cd0, k=pol0.k, clmax=hl["CLmax_clean_trimmed"], clmax_to=hl["CLmax_TO_trimmed"],
                 clmax_ld=hl["CLmax_LD_trimmed"], cd0_to=pol0.cd0 + hl["dCD0_flap_TO"])
    aa = replace(a0, cd0=pola.cd0, k=pola.k)

    def corr(h):        # sizinglib uses the Gagg-Ferrar lapse; correct to engine.yaml sigma^1.23 through eta
        s = AL.isa(h)["sigma"]
        return lapse(h) / AL.power_lapse(s)
    V_cr = tas(V_CRUISE_EAS, H_LOITER)
    eta_cr = flight_point(W, V_cr, H_LOITER, pola, geo["S"])["eta_installed"]
    eta_cl = PROP.wot(28.0, 0.0, P_GEN_SHAFT)["eta"] * INSTALL * KNOCK_STATIC
    eta_ce = PROP.wot(32.0, H_CEILING, P_GEN_SHAFT)["eta"] * INSTALL * KNOCK_STATIC
    V_lof = 1.1 * AL.stall_speed(W, geo["S"], hl["CLmax_TO_trimmed"])
    T_avg = 0.5 * (PROP.wot(0, 0, P_GEN_SHAFT)["T"] + PROP.wot(V_lof, 0, P_GEN_SHAFT)["T"] * INSTALL) * KNOCK_STATIC
    eta_to = T_avg * (V_lof / math.sqrt(2)) / (P_TO - P_GEN_SHAFT)
    ws = np.linspace(250.0, 750.0, 201)
    s_g_lim = 2.0 / 3.0 * RUNWAY
    curves = {
        "cruise_33EAS_3000m_75pct": SZ.pw_cruise(ws, V_cr, H_LOITER, aa, eta=eta_cr * corr(H_LOITER), throttle=0.75),
        "climb_4p9_SL": SZ.pw_climb(ws, ROC_SL_REQ, 0.0, a0, eta=eta_cl),
        "ceiling_4500m": SZ.pw_ceiling(ws, H_CEILING, aa, eta=eta_ce * corr(H_CEILING)),
        "takeoff_ground_200m": SZ.pw_takeoff(ws, s_g_lim, a0, mu=0.04, eta_to=eta_to),
    }
    ws_st = SZ.ws_stall(V_STALL_MAX, hl["CLmax_clean_trimmed"])
    ws_ld = SZ.ws_landing(0.6 * RUNWAY, hl["CLmax_LD_trimmed"], mu_brake=0.3)
    dp = SZ.design_point(ws, curves, min(ws_st, ws_ld))
    pw_avail = (P_MCP - P_GEN_SHAFT) / W
    ws_sel = dv.ws_kg_m2 * G
    pw_sel = max(float(np.interp(ws_sel, ws, c)) for c in curves.values())
    return {"ws_grid_N_m2": ws, "curves": curves, "ws_stall_limit_N_m2": ws_st, "ws_landing_limit_N_m2": ws_ld,
            "design_point_sizinglib": dp, "pw_available_mcp_W_N": pw_avail, "ws_selected_N_m2": ws_sel,
            "pw_required_at_selected_W_N": pw_sel, "eta": {"cruise": eta_cr, "climb": eta_cl, "ceiling": eta_ce,
                                                           "takeoff_avg": eta_to}}


# =====================================================================================================================
# 13. trade studies
# =====================================================================================================================
def key_numbers(res):
    g, p = res["geo"], res["pols"]["ALT"]
    return {"mtow_kg": res["m0"], "fuel_kg": res["fuel_kg"], "empty_kg": res["empty_kg"],
            "structure_kg": res["struct"]["total"], "fixed_equipment_kg": sum(res["fixed"].values()),
            "cd0": p.cd0, "oswald_e": p.e, "ld_max_3000m": p.ld_max, "S_wet_fuselage_m2": g["fus"]["S_wet"],
            "fuselage_w_max_m": g["fus"]["w_max"], "fuselage_h_max_m": g["fus"]["h_max"],
            "S_tail_panels_m2": g["tail"]["S_panels"], "S_ref_m2": g["S"], "span_m": g["b"],
            "sm_range": [res["stab"]["sm_min"], res["stab"]["sm_max"]],
            "V_stall_clean_m_s": AL.stall_speed(res["m0"] * G, g["S"], res["hl"]["CLmax_clean_trimmed"])}


def _job(args):
    """Worker for the trade runs: (tag, dv, fixed_mtom) -> (tag, key numbers, block time)."""
    tag, dv, mtom = args
    try:
        r = evaluate(dv, fixed_mtom=mtom)
    except Exception as exc:                     # a variant that cannot be closed is reported, not hidden
        return tag, {"error": str(exc)}, None
    kn = key_numbers(r)
    if mtom is not None:
        kn["fuel_kg"] = r["fuel_kg"]
        return tag, kn, r["mission"]["block_time_h"] if r["fuel_kg"] > 0 else 0.0
    return tag, kn, r["mission"]["block_time_h"]


def run_parallel(jobs, workers=4):
    import concurrent.futures as cf
    import multiprocessing as mp
    ctx = mp.get_context("fork")
    with cf.ProcessPoolExecutor(max_workers=workers, mp_context=ctx) as ex:
        return list(ex.map(_job, jobs))


def unswept_le_deg(dv: DV, m0=145.0):
    P = planform(dv, m0)
    return math.degrees(math.atan((P["c_r0"] - P["c_t"]) / (4 * P["semispan"])))


def identity_variants(dv: DV):
    """One identity feature replaced by its plain alternative at a time, plus the all-plain reference."""
    plain_sweep = unswept_le_deg(dv)
    return {
        "smooth_elliptic_fuselage": replace(dv, style="smooth", fillets=False),
        "true_flat_facets_n1": replace(dv, style="flat"),
        "lower_facets_soft_n2": replace(dv, n_bot=2.0),
        "no_wing_root_fillets": replace(dv, fillets=False),
        "unswept_c4_wing": replace(dv, sweep_le_deg=plain_sweep, washout_deg=2.5),
        "conventional_tail": replace(dv, tail="conv"),
        "scoop_inlets": replace(dv, inlets="scoop"),
        "retractable_gear": replace(dv, gear="retract"),
        "fixed_unfaired_gear": replace(dv, gear="fixed_bare"),
        "e180_turret_drag": replace(dv, turret_drag="E180"),
        "plain_reference_all": replace(dv, style="smooth", fillets=False, sweep_le_deg=plain_sweep, washout_deg=2.5,
                                       tail="conv", inlets="scoop"),
    }


def quick_field(res):
    """Take-off ground roll / distance to 15 m and landing roll (for the sweep table)."""
    perf = performance_field_only(res)
    return perf


def performance_field_only(res):
    geo, pols, hl = res["geo"], res["pols"], res["hl"]
    W, S = res["m0"] * G, geo["S"]
    pol0 = pols["SL"]
    Vs_to = AL.stall_speed(W, S, hl["CLmax_TO_trimmed"])
    stat = PROP.wot(0.0, 0.0, P_GEN_SHAFT)
    T_lof = PROP.wot(1.1 * Vs_to, 0.0, P_GEN_SHAFT)["T"] * KNOCK_STATIC * INSTALL
    to = AL.takeoff_ground_roll(W, S, hl["CLmax_TO_trimmed"], pol0.cd0 + hl["dCD0_flap_TO"], pol0.k,
                                stat["T"] * KNOCK_STATIC, T_lof, mu=0.04, cl_roll=ground_roll_cl(res))
    s_air, _ = raymer_takeoff_air(W, S, hl["CLmax_TO_trimmed"], pol0, T_lof)
    ld = AL.landing_roll(W, S, hl["CLmax_LD_trimmed"], pol0.cd0 + hl["dCD0_flap_LD"], pol0.k, mu_brake=0.3)
    return {"to_ground_m": to["ground_roll"], "to_15m_m": to["ground_roll"] + s_air, "ldg_roll_mtow_m":
            ld["ground_roll"], "V_lof": to["V_lof"], "V_td": ld["V_td"]}


# =====================================================================================================================
# 14. 3-view sketch from the actual OML meshes
# =====================================================================================================================
def _silhouette(mesh, proj):
    import shapely
    T = mesh.triangles()
    P2 = np.stack([proj(T[:, k, :]) for k in range(3)], axis=1)
    a = 0.5 * np.abs((P2[:, 1, 0] - P2[:, 0, 0]) * (P2[:, 2, 1] - P2[:, 0, 1]) -
                     (P2[:, 2, 0] - P2[:, 0, 0]) * (P2[:, 1, 1] - P2[:, 0, 1]))
    P2 = P2[a > 1e-9]
    polys = shapely.polygons(np.concatenate([P2, P2[:, :1]], axis=1))
    return shapely.union_all(polys).buffer(0.004, join_style=2).buffer(-0.004, join_style=2)


def _draw_poly(ax, geom, fc="#e9edf1", ec="#1d2733", lw=0.9, z=2, ls="-"):
    from matplotlib.patches import PathPatch
    from matplotlib.path import Path as MPath
    gs = getattr(geom, "geoms", [geom])
    for g in gs:
        if g.is_empty or g.geom_type != "Polygon":
            continue
        verts, codes = [], []
        for ring in [g.exterior]:
            c = np.asarray(ring.coords)
            verts += c.tolist()
            codes += [MPath.MOVETO] + [MPath.LINETO] * (len(c) - 2) + [MPath.CLOSEPOLY]
        ax.add_patch(PathPatch(MPath(verts, codes), fc=fc, ec=ec, lw=lw, zorder=z, ls=ls, joinstyle="round"))


def _dim(ax, p0, p1, text, off=(0, 0), fs=8, z=9, txt_off=(0, 0), rot=None):
    p0, p1 = np.asarray(p0, float) + off, np.asarray(p1, float) + off
    ax.annotate("", xy=p1, xytext=p0, arrowprops=dict(arrowstyle="<|-|>", lw=0.7, color="#33507a",
                                                      shrinkA=0, shrinkB=0, mutation_scale=7), zorder=z)
    m = 0.5 * (p0 + p1) + np.asarray(txt_off, float)
    d = p1 - p0
    ang = rot if rot is not None else math.degrees(math.atan2(d[1], d[0]))
    if ang > 90 or ang < -90:
        ang += 180
    ax.text(m[0], m[1], text, fontsize=fs, color="#33507a", ha="center", va="center", rotation=ang, zorder=z + 1,
            bbox=dict(fc="white", ec="none", pad=0.6, alpha=0.9))


def draw_sketch(res, perf, path):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from shapely.geometry import Polygon as SPoly, Point as SPt, LineString
    from shapely import affinity
    dv, geo = res["dv"], res["geo"]
    L, F, g = geo["layout"], geo["F"], geo["gear"]
    wing = oml.LiftingSurface(geo["wing_sections"], name="wing")
    vt = oml.LiftingSurface(geo["tail"]["sections"], name="vtail")
    vf = oml.LiftingSurface(geo["ventral"]["sections"], name="ventral")
    m_fus = F.mesh(110, 96)
    m_w = wing.mesh(refine=3)
    m_wl = m_w.mirrored_y()
    m_t = vt.mesh(refine=2)
    m_tl = m_t.mirrored_y()
    m_v = vf.mesh(refine=2)
    from ucav250.core.geom import sphere, cylinder
    zt = L.z_mount - TURRET_H / 2 - 0.02
    m_tur = sphere(TURRET_D / 2, (L.x_tur, 0.0, zt), 36)
    R = D_PROP / 2
    ze = dv.z_engine
    spin = cylinder(est("spinner_dia_m") / 2, (dv.x_hub, 0, ze), (L.x_prop + 0.10, 0, ze), 32, r1=0.005)
    # projections: plan (nose down, span horizontal: u = -y, v = x), front (u = -y, v = z), side (u = x, v = z)
    v0_plan = 1.55
    u0_side = geo["b"] / 2 + 1.05
    plan = lambda P: np.column_stack([-P[:, 1], v0_plan + P[:, 0]])
    front = lambda P: np.column_stack([-P[:, 1], P[:, 2]])
    side = lambda P: np.column_stack([u0_side + P[:, 0], P[:, 2]])
    fig, ax = plt.subplots(figsize=(17.0, 11.0), dpi=150)
    ax.set_aspect("equal")
    ax.axis("off")
    FILL, EDGE, DARK = "#e6ebf0", "#1b2430", "#c5cdd6"
    # ---------------------------------------------------------------- plan view
    for m, z, fc in ((m_wl, 2, FILL), (m_w, 2, FILL), (m_v, 3, DARK), (m_fus, 4, FILL), (m_tl, 5, FILL),
                     (m_t, 5, FILL)):
        _draw_poly(ax, _silhouette(m, plan), fc=fc, ec=EDGE, z=z)
    _draw_poly(ax, _silhouette(spin, plan), fc=DARK, ec=EDGE, z=5)
    # propeller disc (edge-on in plan), chine line, fillets, hinge lines, inlets
    ax.plot([-R, R], [v0_plan + L.x_prop] * 2, color=EDGE, lw=2.2, zorder=6, solid_capstyle="round")
    ax.plot([-R, R], [v0_plan + L.x_prop] * 2, color="#7f8c99", lw=0.6, ls="--", zorder=6)
    xs = np.linspace(0.02, dv.x_hub, 200)
    wch = np.array([float(F.section(x)[0]) for x in xs]) / 2
    for sgn in (1, -1):
        ax.plot(sgn * wch, v0_plan + xs, color="#55626f", lw=0.5, zorder=5)
    secs = geo["wing_sections"]
    P = geo["planform"]
    for sgn in (1, -1):
        fl_len, fl_w = geo["fillet"]
        yb = geo["y_s"]
        xb = geo["x_w"]
        fil = SPoly([(sgn * -yb, v0_plan + xb - fl_len), (sgn * -(yb + fl_w), v0_plan + xb + 0.02),
                     (sgn * -yb, v0_plan + xb + 0.05)])
        _draw_poly(ax, fil, fc=FILL, ec=EDGE, z=4.5, lw=0.7)
        # flap (side of body -> 0.58 b/2) and aileron (0.60 -> 0.95 b/2) hinge lines at 75 % chord
        for e0, e1 in ((geo["y_s"] / P["semispan"], 0.58), (0.60, 0.95)):
            pts = []
            for e in (e0, e1):
                y = e * P["semispan"]
                c = chord_at(P, y)
                xle = secs[0]["x_le"] + (y - geo["y_s"]) * math.tan(math.radians(dv.sweep_le_deg))
                pts.append((-sgn * y, v0_plan + xle + 0.75 * c, xle + c))
            ax.plot([pts[0][0], pts[1][0]], [pts[0][1], pts[1][1]], color="#55626f", lw=0.6, zorder=3)
            for pp in pts:
                ax.plot([pp[0], pp[0]], [pp[1], v0_plan + pp[2]], color="#55626f", lw=0.6, zorder=3)
        # NACA flush inlets on the upper facets ahead of the V-tail (cooling)
        xi = L.eng[0] - 0.42
        yi = 0.09
        inl = SPoly([(sgn * yi, v0_plan + xi), (sgn * (yi + 0.03), v0_plan + xi + 0.16),
                     (sgn * (yi - 0.03), v0_plan + xi + 0.16)])
        _draw_poly(ax, inl, fc="#9aa6b2", ec=EDGE, z=6, lw=0.5)
    # main wheels (dashed, under the wing)
    for sgn in (1, -1):
        ax.add_patch(plt.Rectangle((sgn * g["track"] / 2 - 0.027, v0_plan + g["x_mg"] - 0.095), 0.054, 0.19,
                                   fc="none", ec=EDGE, lw=0.6, ls="--", zorder=7))
    ax.add_patch(plt.Circle((0, v0_plan + L.x_tur), TURRET_D / 2, fc="none", ec=EDGE, lw=0.6, ls="--", zorder=7))
    # CG and MAC marks
    xcg = res["stab"]["x_cg_mtow"]
    ax.plot(0, v0_plan + xcg, marker="o", ms=7, mfc="white", mec=EDGE, zorder=8)
    ax.plot(0, v0_plan + xcg, marker="+", ms=7, color=EDGE, zorder=9)
    ax.text(0.08, v0_plan + xcg, "AM (MTOW)", fontsize=7, va="center", zorder=9)
    ymac = geo["y_mac"]
    ax.plot([-ymac, -ymac], [v0_plan + geo["x_le_mac"], v0_plan + geo["x_le_mac"] + geo["mac"]], color="#a33",
            lw=1.2, zorder=6)
    ax.text(-ymac - 0.05, v0_plan + geo["x_le_mac"] + 0.5 * geo["mac"], f"OAK {geo['mac']:.3f} m", fontsize=7,
            color="#a33", ha="right", va="center", zorder=7)
    # dims plan
    b = geo["b"]
    _dim(ax, (-b / 2, v0_plan - 0.25), (b / 2, v0_plan - 0.25), f"Açıklık b = {b:.2f} m")
    tipx = secs[-1]["x_le"]
    _dim(ax, (b / 2 + 0.12, v0_plan + secs[0]["x_le"]), (b / 2 + 0.12, v0_plan + tipx),
         f"HK geri kayma {tipx - secs[0]['x_le']:.2f} m", fs=7)
    ax.text(-b / 2 + 0.02, v0_plan + tipx + secs[-1]["chord"] + 0.10, f"uç veteri {secs[-1]['chord']:.3f} m",
            fontsize=7, zorder=9)
    ax.text(-geo["y_s"] - 0.04, v0_plan + secs[0]["x_le"] - 0.04,
            f"kök veteri (gövde yanı) {secs[0]['chord']:.3f} m", fontsize=7, ha="right", va="top", zorder=9)
    _dim(ax, (-geo['tail']['span_panel'] * math.cos(math.radians(geo['tail']['dihedral_deg'])) - 0.06 - 0.05,
              v0_plan + L.x_prop + 0.12), (geo['tail']['span_panel'] * math.cos(math.radians(geo['tail']['dihedral_deg'])) + 0.06 + 0.05,
              v0_plan + L.x_prop + 0.12),
         f"V-kuyruk izdüşüm açıklığı {2 * (geo['tail']['span_panel'] * math.cos(math.radians(geo['tail']['dihedral_deg'])) + 0.05):.2f} m",
         fs=7)
    ax.text(0, v0_plan - 0.55, "ÜSTTEN GÖRÜNÜŞ (burun aşağıda)", ha="center", fontsize=9, weight="bold")
    # ---------------------------------------------------------------- front view
    zg = g["z_ground"]
    for m, z, fc in ((m_t, 2, FILL), (m_tl, 2, FILL), (m_v, 2, DARK), (m_w, 3, FILL), (m_wl, 3, FILL),
                     (m_fus, 4, FILL), (m_tur, 6, "#8693a0")):
        _draw_poly(ax, _silhouette(m, front), fc=fc, ec=EDGE, z=z)
    ax.add_patch(plt.Circle((0, ze), R, fc="none", ec="#7f8c99", lw=0.8, ls="--", zorder=1.5))
    # gear (front): spring bow from the belly to the axles, wheels, nose leg
    for sgn in (1, -1):
        yb0 = 0.10
        ax.plot([sgn * yb0, sgn * g["track"] / 2], [g["z_belly_main"] + 0.01, zg + g["wheel_radius"]],
                color=EDGE, lw=3.0, zorder=5, solid_capstyle="round")
        ax.add_patch(plt.Rectangle((sgn * g["track"] / 2 - 0.027, zg), 0.054, 0.19, fc="#59636e", ec=EDGE, lw=0.6,
                                   zorder=6))
    ax.plot([0, 0], [g["z_belly_nose"], zg + g["wheel_radius"]], color=EDGE, lw=2.4, zorder=7)
    ax.add_patch(plt.Rectangle((-0.027, zg), 0.054, 0.19, fc="#59636e", ec=EDGE, lw=0.6, zorder=8))
    ax.plot([-b / 2 - 0.2, b / 2 + 0.2], [zg, zg], color="#5b6670", lw=0.8, zorder=1)
    ax.text(0, zg - 0.30, "ÖNDEN GÖRÜNÜŞ", ha="center", fontsize=9, weight="bold")
    _dim(ax, (-g["track"] / 2, zg - 0.10), (g["track"] / 2, zg - 0.10), f"iz genişliği {g['track']:.2f} m", fs=7)
    gam = geo["tail"]["dihedral_deg"]
    ax.text(0.55, ze + 0.75, f"V-kuyruk dihedral {gam:.0f}°", fontsize=7, zorder=9)
    # ---------------------------------------------------------------- side view
    for m, z, fc in ((m_w, 2, FILL), (m_tl, 2, FILL), (m_fus, 4, FILL), (m_t, 5, FILL), (m_v, 5, DARK),
                     (m_tur, 6, "#8693a0")):
        _draw_poly(ax, _silhouette(m, side), fc=fc, ec=EDGE, z=z)
    _draw_poly(ax, _silhouette(spin, side), fc=DARK, ec=EDGE, z=6)
    ax.plot([u0_side + L.x_prop] * 2, [ze - R, ze + R], color=EDGE, lw=2.2, zorder=6, solid_capstyle="round")
    zc_line = np.array([float(F.section(x)[2]) for x in xs])
    ax.plot(u0_side + xs, zc_line, color="#55626f", lw=0.6, zorder=5)
    # gear side
    for xg, zb in ((g["x_mg"], g["z_belly_main"]), (g["x_ng"], g["z_belly_nose"])):
        ax.plot([u0_side + xg, u0_side + xg], [zb, zg + g["wheel_radius"]], color=EDGE, lw=2.4, zorder=7)
        ax.add_patch(plt.Circle((u0_side + xg, zg + g["wheel_radius"]), g["wheel_radius"], fc="#59636e", ec=EDGE,
                                lw=0.6, zorder=8))
        ax.add_patch(plt.Circle((u0_side + xg, zg + g["wheel_radius"]), 0.035, fc="#c5cdd6", ec=EDGE, lw=0.4,
                                zorder=9))
    ax.plot([u0_side - 0.3, u0_side + L.x_prop + 0.4], [zg, zg], color="#5b6670", lw=0.8, zorder=1)
    # tail-strike line
    ts = math.radians(g["tailstrike_deg"])
    xa, xb_ = g["x_mg"], geo["ventral"]["x_skid"] + 0.25
    ax.plot([u0_side + xa, u0_side + xb_], [zg, zg + (xb_ - xa) * math.tan(ts)], color="#a33", lw=0.6, ls="--",
            zorder=1)
    ax.text(u0_side + xb_ + 0.02, zg + (xb_ - xa) * math.tan(ts), f"kuyruk vurma {g['tailstrike_deg']:.1f}°",
            fontsize=7, color="#a33", va="center")
    ax.plot(u0_side + xcg, res["stab"]["z_cg_mtow"], marker="o", ms=7, mfc="white", mec=EDGE, zorder=10)
    ax.plot(u0_side + xcg, res["stab"]["z_cg_mtow"], marker="+", ms=7, color=EDGE, zorder=11)
    L_all = L.x_prop + 0.10
    zmax = max(float(m_t.V[:, 2].max()), float(m_fus.V[:, 2].max()))
    _dim(ax, (u0_side, zg - 0.14), (u0_side + L_all, zg - 0.14), f"Toplam boy {L_all:.2f} m (pervane göbeği dahil)")
    _dim(ax, (u0_side + L_all + 0.18, zg), (u0_side + L_all + 0.18, zmax), f"{zmax - zg:.2f} m", fs=7)
    _dim(ax, (u0_side + L.x_prop + 0.08, ze - R), (u0_side + L.x_prop + 0.08, ze + R), f"Ø{D_PROP:.3f}", fs=7)
    _dim(ax, (u0_side + L.x_prop - 0.12, zg), (u0_side + L.x_prop - 0.12, ze - R),
         f"{perf['prop_clearance_m']:.2f}", fs=7)
    _dim(ax, (u0_side + g["x_ng"], zg - 0.32), (u0_side + g["x_mg"], zg - 0.32), f"tekerlek açıklığı {g['wheelbase']:.2f} m",
         fs=7)
    ax.text(u0_side + 0.5 * L_all, zg - 0.52, "YANDAN GÖRÜNÜŞ (sol)", ha="center", fontsize=9, weight="bold")
    ax.text(u0_side + L.x_tur, zt - TURRET_D / 2 - 0.08, "EO/IR (HD59;\nyuva E180 için)", fontsize=6.5, ha="center",
            va="top")
    # ---------------------------------------------------------------- title block
    k = perf
    lines = [
        "YK-250  ·  Kavram \"identity\" (UCAV kimliği)",
        "Sivil EO/IR gözetleme/araştırma İHA'sı — silah, askı noktası, pilon yok",
        "",
        f"MTOW {res['m0']:.1f} kg   boş {res['empty_kg']:.1f} kg   yakıt {res['fuel_kg']:.1f} kg   faydalı yük {PAYLOAD_DESIGN:.0f} kg",
        f"S {geo['S']:.2f} m²   b {geo['b']:.2f} m   AR {dv.AR:.1f}   λ {dv.taper:.2f}   ΛHK {dv.sweep_le_deg:.0f}°   W/S {dv.ws_kg_m2:.1f} kg/m²",
        f"Motor Limbach L 275 EF 18 kW, itici, Mejzlik 32x18 2B Ø{D_PROP:.3f} m",
        f"CD0 {res['pols']['ALT'].cd0:.4f}   e {res['pols']['ALT'].e:.2f}   (L/D)maks {res['pols']['ALT'].ld_max:.1f}",
        f"Vs {k['V_stall_clean_m_s']:.1f} m/s (temiz)   Vmaks {k['V_max_level_tas_SL_m_s']:.0f} m/s   RoC {k['RoC_max_SL_m_s']:.1f} m/s",
        f"Görev: 10 h blok (3000 m, 28 m/s EAS) + 2x100 km + 1 h yedek",
        f"Statik marj {res['stab']['sm_min']:.2f}–{res['stab']['sm_max']:.2f}   VH {res['stab']['volumes']['V_H']:.2f}   VV {res['stab']['volumes']['V_V']:.3f}",
        "Ölçek: tüm görünüşler aynı ölçekte (m). Kaynak: calc.py, concept.yaml",
    ]
    tx, ty = u0_side - 0.25, v0_plan + 3.95
    ax.text(tx, ty, "\n".join(lines), fontsize=8.6, va="top", ha="left", family="DejaVu Sans",
            bbox=dict(fc="#f6f8fa", ec="#9aa6b2", pad=8))
    ax.set_xlim(-b / 2 - 0.35, u0_side + L_all + 0.55)
    ax.set_ylim(zg - 0.65, v0_plan + L_all + 0.45)
    fig.tight_layout()
    fig.savefig(path, dpi=150)
    plt.close(fig)


def draw_constraints(cons, res, path):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    ws = cons["ws_grid_N_m2"] / G
    fig, ax = plt.subplots(figsize=(8.0, 5.2), dpi=140)
    names = {"cruise_33EAS_3000m_75pct": "seyir 33 m/s EAS @3000 m, %75 güç",
             "climb_4p9_SL": "tırmanma 4,9 m/s @DS", "ceiling_4500m": "servis tavanı 4500 m",
             "takeoff_ground_200m": "kalkış yer koşusu ≤ 200 m"}
    cols = ["#2a6f97", "#c0392b", "#7d3c98", "#1e8449"]
    for (k, v), c in zip(cons["curves"].items(), cols):
        ax.plot(ws, v, color=c, lw=1.6, label=names.get(k, k))
    ax.axvline(cons["ws_stall_limit_N_m2"] / G, color="#555", ls="--", lw=1.2, label="Vs ≤ 24 m/s (temiz, trimli)")
    ax.axvline(cons["ws_landing_limit_N_m2"] / G, color="#999", ls=":", lw=1.2, label="iniş koşusu ≤ 180 m")
    ax.axhline(cons["pw_available_mcp_W_N"], color="#000", lw=1.0, label="mevcut (MCP − jeneratör)")
    ax.plot(cons["ws_selected_N_m2"] / G, cons["pw_required_at_selected_W_N"], "o", mfc="#f1c40f", mec="k", ms=8,
            label="seçilen tasarım noktası")
    ax.set_xlabel("Kanat yüklemesi W/S [kg/m²]")
    ax.set_ylabel("Gereken mil gücü / ağırlık P/W [W/N] (deniz seviyesi eşdeğeri)")
    ax.set_ylim(0, max(cons["pw_available_mcp_W_N"] * 1.4, 8))
    ax.grid(alpha=0.3)
    ax.legend(fontsize=7.5, loc="upper left")
    ax.set_title("YK-250 'identity' — kısıt diyagramı (sizinglib)", fontsize=10)
    fig.tight_layout()
    fig.savefig(path)
    plt.close(fig)


# =====================================================================================================================
# 15. YAML output helpers
# =====================================================================================================================
def clean(o, sig=5):
    if isinstance(o, dict):
        return {str(k): clean(v, sig) for k, v in o.items()}
    if isinstance(o, (list, tuple)):
        return [clean(v, sig) for v in o]
    if isinstance(o, np.ndarray):
        return [clean(v, sig) for v in o.tolist()]
    if isinstance(o, (np.floating, float)):
        f = float(o)
        if not math.isfinite(f):
            return str(f)
        return float(f"{f:.{sig}g}")
    if isinstance(o, (np.integer,)):
        return int(o)
    if isinstance(o, (np.bool_,)):
        return bool(o)
    return o


# =====================================================================================================================
# 16. main
# =====================================================================================================================
TRADE_PROPULSION = [
    {"option": "pusher, direct drive (selected)", "pros_tr": "Burun ve çene EO/IR görüş alanı açık; sivri, köşeli UCAV "
     "burnu mümkün; pervane izi gövdeyi yalamaz.", "cons_tr": "Kurulum verimi 0,95 (baseline); motor kanatçıklarına pervane "
     "rüzgârı yok -> kanallı soğutma; boxer silindirleri pervane göbeğinin 30-170 mm önünde -> kısa ve küt arka kaporta "
     "(taban sürüklemesi hesaplandı); pervane yerden yüksek olmalı -> uzun iniş takımı."},
    {"option": "tractor", "pros_tr": "Soğutma kolay (pervane rüzgârı), kurulum verimi ~1,0; motor öne -> ağırlık "
     "merkezi öne, kuyruk kolu uzar.", "cons_tr": "Burun pervaneye ayrılır: UCAV burnu ve çene tareti kaybolur, taret "
     "gövde altına iner ve iniş takımı/pervane izi görüşü kapatır; pervane izi gövde ve kanat kökü sürtünmesini artırır. "
     "Kimlik hedefiyle çelişir -> reddedildi."},
]


def main():
    t0 = time.time()
    np.seterr(all="ignore")
    base = DV()
    print("YK-250 concept 'identity' - closed-loop sizing (this takes a few minutes)")
    res0 = evaluate(base)                                            # also warms the tripped-polar cache
    print(f"  base closed loop: MTOW {res0['m0']:.1f} kg ({time.time() - t0:.0f} s)")
    # ------------------------------------------------------------------ sweeps and identity price (parallel)
    jobs = [(f"x_hub={xh}", replace(base, x_hub=xh), None) for xh in (3.9, 4.2, 4.5)]
    jobs += [(f"ws={ws:g},AR={ar:g}", replace(base, ws_kg_m2=ws, AR=ar), None)
             for ws in (44.0, 45.0, 46.0) for ar in (11.0, 12.0, 13.0)]
    variants = identity_variants(base)
    jobs += [(f"var:{k}", v, None) for k, v in variants.items()]
    jobs += [(f"fix:{k}", v, res0["m0"]) for k, v in variants.items()]
    jobs += [("fix:design", base, res0["m0"])]
    out = run_parallel(jobs)
    R_ = {tag: (kn, bt) for tag, kn, bt in out}
    print(f"  {len(jobs)} trade runs done ({time.time() - t0:.0f} s)")
    # ------------------------------------------------------------------ final design evaluation
    res = res0
    perf = performance(res)
    perf["prop_clearance_m"] = res["geo"]["gear"]["prop_static_clearance"]
    perf["V_stall_clean_m_s"] = perf["V_stall_clean_m_s"]
    cons = constraints(res)
    field = performance_field_only(res)
    geo, pols, stab, hl, dv = res["geo"], res["pols"], res["stab"], res["hl"], res["dv"]
    m0 = res["m0"]
    feasible = m0 < MTOM_CAP
    # ------------------------------------------------------------------ identity price table
    base_fix_bt = R_["fix:design"][1]
    price = []
    for k in variants:
        kn, bt = R_[f"var:{k}"]
        knf, btf = R_[f"fix:{k}"]
        if "error" in kn:
            price.append({"variant": k, "error": kn["error"]})
            continue
        price.append({"variant": k, "mtow_for_10h_mission_kg": kn["mtow_kg"],
                      "delta_mtow_vs_design_kg": kn["mtow_kg"] - m0,
                      "cd0": kn["cd0"], "delta_cd0": kn["cd0"] - pols["ALT"].cd0,
                      "empty_kg": kn["empty_kg"], "delta_empty_kg": kn["empty_kg"] - res["empty_kg"],
                      "S_wet_fuselage_m2": kn["S_wet_fuselage_m2"], "fuselage_w_h_max_m": [kn["fuselage_w_max_m"],
                                                                                         kn["fuselage_h_max_m"]],
                      "block_endurance_at_design_mtom_h": btf,
                      "delta_endurance_at_design_mtom_h": (btf - base_fix_bt) if btf is not None else None})
    sweep_len = [{"x_hub_m": float(t.split("=")[1]), **R_[t][0]} for t in R_ if t.startswith("x_hub=")]
    sweep_ws = []
    for t in R_:
        if t.startswith("ws="):
            ws, ar = (float(v.split("=")[1]) for v in t.split(","))
            kn = R_[t][0]
            sweep_ws.append({"ws_kg_m2": ws, "AR": ar, **kn,
                             "stall_ok": kn.get("V_stall_clean_m_s", 99) <= V_STALL_MAX + 0.05,
                             "span_ok": kn.get("span_m", 99) <= 6.8})
    # ------------------------------------------------------------------ gear trade
    g_ff = {"mtow_kg": m0, "cd0": pols["ALT"].cd0, "gear_DQ_m2": pols["ALT"].items["landing_gear"],
            "gear_mass_kg": res["fixed"]["landing_gear"] + res["fixed"].get("gear_fairings", 0.0),
            "endurance_at_design_mtom_h": base_fix_bt}
    gr = {k: R_[f"var:{k}"][0] for k in ("retractable_gear", "fixed_unfaired_gear")}
    gear_trade = {
        "fixed_faired_selected": g_ff,
        "retractable": {"mtow_for_mission_kg": gr["retractable_gear"].get("mtow_kg"),
                        "cd0": gr["retractable_gear"].get("cd0"),
                        "endurance_at_design_mtom_h": R_["fix:retractable_gear"][1],
                        "mass_delta_items_kg": {"mechanism_EMAs_locks": est("retract_mech_delta_kg"),
                                                "bays_doors_cutout_reinforcement": est("retract_bays_doors_kg"),
                                                "fuselage_plug_for_wells": "computed (fuselage_plug_for_wells)"},
                        "extra_parts": ["3 retraction EMAs (Volz DA 26 class) + controller", "3 down-locks + 3 up-locks "
                                        "with sensors", "5 doors + hinges + linkages", "gear sequencing in the FCS",
                                        "emergency free-fall/spring extension", "well close-outs and seals"],
                        "new_failure_modes_tr": ["takım açılmazsa gövde üstü iniş (taret ve pervane hasarı)",
                                                 "kapı açık kalması / sıkışması", "kilit sensörü hatası"],
                        "packaging_tr": "Ana takım kuyuları yakıt torbaları ve faydalı yük bölmesiyle aynı yerde "
                                        "(ağırlık merkezi); ön takım kuyusu çene tareti ve aviyonikle çakışır -> gövde "
                                        "uzaması gerekir.",
                        "cost_tr": "Satın alınabilir 100-200 kg sınıfı katlanır takım yok (components.yaml); özel "
                                   "elektrikli tasarım (SAGITTA mimarisi) + nitelendirme testleri gerekir."},
        "fixed_unfaired": {"mtow_for_mission_kg": gr["fixed_unfaired_gear"].get("mtow_kg"),
                           "cd0": gr["fixed_unfaired_gear"].get("cd0"),
                           "endurance_at_design_mtom_h": R_["fix:fixed_unfaired_gear"][1]},
        "decision_tr": "Sabit, kaplamalı (tekerlek kılıfı + profilli bacak) üç tekerlekli takım. Katlanır takım "
                       "sürüklemeyi azaltır ama kütle, yakıt torbası/taret ile yer çakışması, maliyet ve arıza "
                       "kipleri bunu fazlasıyla geri alır: aynı görevde MTOW artar, aynı MTOW'da dayanım azalır.",
    }
    # ------------------------------------------------------------------ tail trade
    tl = geo["tail"]
    tip_drop = tl["span_panel"] * math.sin(math.radians(tl["dihedral_deg"]))
    tail_trade = {
        "Y_tail_selected": {"S_panels_m2": tl["S_panels"], "S_ventral_m2": geo["ventral"]["S"], "mtow_kg": m0,
                            "cd0": pols["ALT"].cd0, "actuators": 2, "junctions": 3},
        "conventional_H_plus_fin": {"mtow_kg": R_["var:conventional_tail"][0].get("mtow_kg"),
                                    "cd0": R_["var:conventional_tail"][0].get("cd0"), "actuators": 2,
                                    "junctions": 2, "note_tr": "Aynı hacim katsayıları; pervane koruması için ayrı "
                                                              "kuyruk kızağı gerekir; UCAV görünümü zayıf."},
        "inverted_V": {"panel_tip_drop_below_root_m": tip_drop,
                       "note_tr": f"Ters V panel uçları kökün {tip_drop:.2f} m altına iner; kalkış dönüşünde yerle "
                                  "temas etmemesi için iniş takımı yaklaşık bu kadar uzamalı (kütle + sürükleme) ve "
                                  "panel uçları egzoz/pervane bölgesine yaklaşır. Reddedildi."},
        "decision_tr": "Dik V-kuyruk (kanted ikiz kanatçık görünümü) + karın yüzgeci (Y-kuyruk): karın yüzgeci pervane "
                       "koruyucu ve kuyruk kızağıdır, yön kararlılığına da katkı verir.",
    }
    # ------------------------------------------------------------------ console table
    W = m0 * G
    p_alt, p_sl = pols["ALT"], pols["SL"]
    lp = perf["loiter_3000m_point"]
    rows = [
        ("MTOW (10 h mission, closed loop)", f"{m0:.1f} kg", "< 150 kg (SHT-IHA M2)", "OK" if feasible else "VIOLATED"),
        ("empty / fuel / payload", f"{res['empty_kg']:.1f} / {res['fuel_kg']:.1f} / {PAYLOAD_DESIGN:.0f} kg", "", ""),
        ("structure (incl. 10 % margin) / fixed eq.", f"{res['struct']['total']:.1f} / {sum(res['fixed'].values()):.1f} kg",
         "", ""),
        ("S / b / AR / W/S", f"{geo['S']:.2f} m2 / {geo['b']:.2f} m / {dv.AR:.1f} / {dv.ws_kg_m2:.1f} kg/m2", "b <= 6.8 m",
         "OK" if geo["b"] <= 6.8 else "CHECK"),
        ("fuselage L x w x h", f"{dv.x_hub:.2f} x {geo['fus']['w_max']:.2f} x {geo['fus']['h_max']:.2f} m", "", ""),
        ("CD0 / e / (L/D)max @3000 m", f"{p_alt.cd0:.4f} / {p_alt.e:.3f} / {p_alt.ld_max:.1f}", "", ""),
        ("Vs clean / TO flap / LD flap", f"{perf['V_stall_clean_m_s']:.1f} / {perf['V_stall_TO_flap_m_s']:.1f} / "
         f"{perf['V_stall_LD_flap_m_s']:.1f} m/s", "clean <= 24", "OK" if perf['V_stall_clean_m_s'] <= 24.05 else "CHECK"),
        ("V min power / best L/D (TAS, 3000 m)", f"{perf['V_min_power_tas_3000m_m_s']:.1f} / "
         f"{perf['V_ld_max_tas_3000m_m_s']:.1f} m/s", "", ""),
        ("Vmax level SL / 3000 m (TAS)", f"{perf['V_max_level_tas_SL_m_s']:.1f} / {perf['V_max_level_tas_3000m_m_s']:.1f} m/s",
         f"<= VD {VD:.0f}", "OK" if perf['V_max_level_tas_SL_m_s'] <= VD else "CHECK"),
        ("loiter 28 EAS @3000 m: rpm, P, fuel", f"{lp['rpm']:.0f} rpm, {lp['P_total_W'] / 1000:.2f} kW "
         f"({lp['power_frac'] * 100:.0f} %), {lp['fuel_kg_h']:.2f} kg/h", "", ""),
        ("design mission block / loiter", f"{perf['design_mission_block_time_h']:.2f} h / "
         f"{perf['design_mission_loiter_h']:.2f} h", ">= 10 h", "OK"),
        ("range (best L/D, 3000 m, same fuel)", f"{perf['range_km_best']:.0f} km", "", ""),
        ("RoC SL / 3000 m", f"{perf['RoC_max_SL_m_s']:.2f} / {perf['RoC_max_3000m_m_s']:.2f} m/s", f">= {ROC_SL_REQ}",
         "OK" if perf['RoC_max_SL_m_s'] >= ROC_SL_REQ else "CHECK"),
        ("service / absolute ceiling", f"{perf['service_ceiling_m']:.0f} / {perf['absolute_ceiling_m']:.0f} m",
         f">= {H_CEILING:.0f}", "OK" if perf['service_ceiling_m'] >= H_CEILING else "CHECK"),
        ("TO ground roll / to 15 m", f"{perf['TO_ground_roll_m']:.0f} / {perf['TO_distance_15m_m']:.0f} m",
         f"15 m <= {RUNWAY:.0f}", "OK" if perf['TO_distance_15m_m'] <= RUNWAY else "CHECK"),
        ("landing roll MTOW / normal", f"{perf['landing_ground_roll_mtow_m']:.0f} / "
         f"{perf['landing_ground_roll_normal_m']:.0f} m", f"<= {0.6 * RUNWAY:.0f}",
         "OK" if perf['landing_ground_roll_mtow_m'] <= 0.6 * RUNWAY else "CHECK"),
        ("static margin (all cases)", f"{stab['sm_min']:.3f} .. {stab['sm_max']:.3f}", "0.05 .. 0.25",
         "OK" if 0.05 <= stab['sm_min'] and stab['sm_max'] <= 0.25 else "CHECK"),
        ("V_H / V_V", f"{stab['volumes']['V_H']:.3f} / {stab['volumes']['V_V']:.4f}", "", ""),
        ("Cn_beta total", f"{stab['cn_beta']['total']:.3f} /rad", f">= {est('cn_beta_min_per_rad')}",
         "OK" if stab['cn_beta']['total'] >= est('cn_beta_min_per_rad') else "CHECK"),
        ("fuel bladders conformal volume", f"{geo['tank_volume_m3'] * 1000:.1f} L (need "
         f"{res['fuel_kg'] / RHO_FUEL * 1000 / 0.98:.1f} L)", "", "OK" if geo['tank_volume_m3'] * 0.98 * RHO_FUEL >=
         res['fuel_kg'] else "CHECK"),
        ("prop static clearance / tip Mach", f"{geo['gear']['prop_static_clearance']:.2f} m / "
         f"{perf['static_tip_mach']:.3f}", ">= 0.18 m / <= 0.75", "OK"),
    ]
    print("\n" + "=" * 118)
    print(f"{'item':44s} {'value':44s} {'target':18s} status")
    print("-" * 118)
    for r in rows:
        print(f"{r[0]:44s} {r[1]:44s} {r[2]:18s} {r[3]}")
    print("-" * 118)
    print("identity price (one feature replaced at a time; closed loop for the 10 h mission, and endurance at the "
          f"design MTOM {m0:.1f} kg):")
    for p in price:
        if "error" in p:
            print(f"  {p['variant']:28s} ERROR {p['error']}")
            continue
        de = p['delta_endurance_at_design_mtom_h']
        print(f"  {p['variant']:28s} MTOW {p['mtow_for_10h_mission_kg']:6.1f} ({p['delta_mtow_vs_design_kg']:+5.1f}) "
              f"CD0 {p['cd0']:.4f} ({p['delta_cd0']:+.4f})  empty {p['delta_empty_kg']:+5.1f} kg  "
              f"endurance@MTOM {p['block_endurance_at_design_mtom_h']:.2f} h ({de:+.2f})")
    print("=" * 118)
    # ------------------------------------------------------------------ concept.yaml
    from dataclasses import asdict
    wing_secs = geo["wing_sections"]
    P = geo["planform"]
    struct = res["struct"]
    items = stab["items_mtow"]
    doc = {
        "meta": {
            "concept": "identity", "project": "YK-250 (ucav250)", "date": "2026-10-05",
            "name_tr": "YK-250 'Kimlik' — en güçlü UCAV görünümü, görevi bozmadan",
            "scope": "Civil EO/IR surveillance / research UAV with a UCAV look. Payload is EO/IR sensors and mission "
                     "equipment only; no weapons, munitions, hardpoints, pylons or release mechanisms.",
            "how_to_run": "PYTHONPATH=. python3 ucav250/data/concepts/identity/calc.py (from the repository root)",
            "method": __doc__.split("Method")[1].split("Frame:")[0].strip(),
            "frame": "X aft from the nose tip (FS), Y starboard, Z up; Z = 0 on the centre-fuselage chine line; SI",
            "runtime_s": time.time() - t0,
            "feasible": feasible,
        },
        "sources": SRC,
        "estimates": {k: {"value": v[0], "basis": v[1]} for k, v in EST.items()},
        "inputs": {
            "engine": {"model": ENG["model"], "power_max_W": P_MAX, "power_takeoff_W": P_TO, "power_mcp_W": P_MCP,
                       "wot_rpm": WOT_RPM, "wot_torque_N_m": WOT_Q, "bsfc_curve_frac_g_per_kWh": BSFC_PTS,
                       "power_lapse": "sigma^1.23", "fuel_density_kg_m3": RHO_FUEL, "envelope_m": ENV_ENG,
                       "installed_mass_kg": SYS["engine_group_installed"], "source": SRC["baseline"] + "#engine"},
            "propeller": {"model": PROPY["model"], "diameter_m": D_PROP, "mass_kg": M_PROP,
                          "installation_factor": INSTALL, "static_knockdown": KNOCK_STATIC,
                          "table": SRC["aero"] + "#propeller_tables_mejzlik.0161",
                          "model_check_static_thrust_N": PROP.wot(0, 0, 0)["T"],
                          "model_check_static_rpm": PROP.wot(0, 0, 0)["rpm"],
                          "research_static_thrust_N": val(PROPY["static_thrust_N"])},
            "electrical": {"continuous_load_W": P_ELEC, "generator_efficiency": ETA_GEN,
                           "generator_shaft_W": P_GEN_SHAFT, "source": SRC["baseline"] + "#subsystems.electrical_load_W"},
            "mission": {"endurance_block_h": ENDURANCE_H, "loiter_eas_m_s": V_LOITER_EAS, "loiter_altitude_m": H_LOITER,
                        "cruise_eas_m_s": V_CRUISE_EAS, "transit_radius_km": TRANSIT_KM, "reserve_h": RESERVE_H,
                        "reserve_altitude_m": 1000.0, "service_ceiling_m": H_CEILING, "runway_m": RUNWAY,
                        "climb_sl_m_s": ROC_SL_REQ, "stall_max_m_s": V_STALL_MAX, "mtom_cap_kg": MTOM_CAP,
                        "trapped_fuel_fraction": 0.02, "source": SRC["baseline"] + "#mission_targets"},
            "payload": {"design_kg": PAYLOAD_DESIGN, "turret_HD59_with_mount_kg": TURRET_M,
                        "bay_growth_E180": {"diameter_m": E180_D, "height_m": E180_H, "mass_kg": E180_M},
                        "mission_computer_kg": MISSION_COMPUTER_M, "tray_harness_kg": PAY_TRAY_M,
                        "research_allowance_kg": RESEARCH_PAY_M, "source": SRC["baseline"] + "#payload_set"},
            "systems_rollup_baseline_kg": SYS,
            "aero_corrections": {"cl_max_factor": CLMAX_FACTOR, "k_clmax_3d": K_CLMAX_3D,
                                 "cd_factor_tripped": CD_TRIP_FACTOR, "transition_trip_xc": XTR_TRIP,
                                 "cm_ac_wing": CM_AC_WING, "source": SRC["baseline"] + "#aero.corrections_for_sizing"},
            "loads": {"n_pos": N_POS, "n_neg": N_NEG, "FoS": FOS, "VC_m_s": VC, "VD_m_s": VD, "Ude_C_m_s": UDE_C,
                      "Ude_D_m_s": UDE_D, "compression_strain_limit": STRAIN_C_THICK, "A_basis_factor": A_BASIS,
                      "special_factor": SPECIAL_F, "source": SRC["baseline"] + "#design_loads"},
            "areal_masses_kg_m2": AREAL,
        },
        "design_variables": asdict(dv),
        "configuration": {
            "propulsion": "single pusher, direct drive, Limbach L 275 EF + Mejzlik 32x18 2B, 60 mm spacer",
            "tail": "Y-tail: upright V-tail (canted twin surfaces, ruddervators) + ventral fin with skid (prop guard)",
            "wing_position": "shoulder wing on the chine line, blended LERX-type root fillets",
            "gear": "fixed tricycle, faired: GFRP spring bow mains (0.20 m stroke) + steerable damped nose leg, spats",
            "fuselage_style": f"chined/faceted superellipse sections, n_top {dv.n_top}, n_bot {dv.n_bot}; dorsal "
                              "engine hump with flush NACA inlets; chin EO/IR turret",
            "planform": f"tapered (lambda {dv.taper}), leading-edge sweep {dv.sweep_le_deg} deg, quarter-chord "
                        f"{geo['sweep_c4_deg']:.1f} deg, dihedral {dv.dihedral_deg} deg, washout {dv.washout_deg} deg",
            "transport_breakdown": "2 outer wing panels (side-of-body joints), 2 V-tail panels, propeller, main gear "
                                   "bow (4 bolts); fuselage with centre box, ventral fin and nose leg = 1 crate",
            "reasons_tr": [
                "İtici: burun ve çene tareti açık görür; köşeli, sivri UCAV burnu ancak itici ile mümkün.",
                "Y-kuyruk: dik V kanatçıklar UCAV görünümünü verir, karın yüzgeci pervaneyi ve kuyruğu yere sürtünmeye "
                "karşı korur ve yön kararlılığına katkı verir; ters V ise takımı uzatırdı.",
                "Omuz kanat köşe çizgisinde: taşıyıcı kutu yakıt torbaları ve faydalı yük bölmesinin üstünden geçer, "
                "kök kaplamaları gövde-kanat geçişini UCAV gibi birleştirir.",
                "Sabit kaplamalı takım: katlanır takım bu sınıfta hem kütle hem dayanımda kaybettiriyor (tablo).",
                "Faydalı yük bölmesi ağırlık merkezinde, yakıt öne/arkaya iki torbaya bölünmüş: yük olsa da olmasa da "
                "ağırlık merkezi kayması küçük, kuyruk küçük kalır.",
            ],
        },
        "trade_studies": {
            "propulsion": TRADE_PROPULSION,
            "tail": tail_trade,
            "landing_gear": gear_trade,
            "fuselage_length": sweep_len,
            "wing_loading_aspect_ratio": sweep_ws,
            "identity_price": price,
        },
        "geometry": {
            "wing": {"S_ref_m2": geo["S"], "span_m": geo["b"], "aspect_ratio": dv.AR, "taper": dv.taper,
                     "root_chord_centreline_m": P["c_r0"], "chord_side_of_body_m": geo["c_s"], "tip_chord_m": P["c_t"],
                     "sweep_le_deg": dv.sweep_le_deg, "sweep_c4_deg": geo["sweep_c4_deg"],
                     "dihedral_deg": dv.dihedral_deg, "incidence_root_deg": geo["i_w"], "washout_deg": dv.washout_deg,
                     "airfoils": {"root": "nlf416 (NASA NLF(1)-0416, t/c 0.16)", "tip": "nlf416 thickness_scale 0.8125"
                                                                                         " (t/c 0.13)"},
                     "sections": wing_secs, "sections_analysis": geo["wing_sections_analysis"],
                     "mac_m": geo["mac"], "mac_le_x_m": geo["x_le_mac"], "mac_y_m": geo["y_mac"],
                     "x_ac_m": geo["x_ac_w"], "S_exposed_m2": geo["S_exp"], "y_side_of_body_m": geo["y_s"],
                     "controls": {"flap": {"eta0": geo["y_s"] / P["semispan"], "eta1": 0.58, "xc_hinge": 0.75,
                                           "range_deg": [0, 40]},
                                  "aileron": {"eta0": 0.60, "eta1": 0.95, "xc_hinge": 0.75, "range_deg": [-20, 20]}},
                     "root_fillet": {"length_m": geo["fillet"][0], "width_m": geo["fillet"][1],
                                     "S_wet_m2": geo["fillet_S_wet"]}},
            "tail": {"type": "Y (V-tail + ventral fin)",
                     "v_tail": {k: tl[k] for k in ("sections", "dihedral_deg", "S_panels", "S_panel", "span_panel",
                                                    "c_root", "c_tip", "mac", "x_ac", "z_ac", "AR_V", "a_per_rad",
                                                    "S_h_eff", "S_v_eff", "tip_te_x", "l_h", "l_v")},
                     "ruddervator": {"xc_hinge": 0.70, "range_deg": [-25, 25]},
                     "ventral_fin": {k: geo["ventral"][k] for k in ("sections", "S", "span", "c_root", "c_tip",
                                                                     "z_skid", "x_skid", "x_ac", "AR_eff")},
                     "sweep_le_deg": dv.tail_sweep_le_deg, "taper": dv.tail_taper, "airfoil": "n0012"},
            "fuselage": {"stations_x_w_h_zc_ntop_nbot_topfrac": geo["stations"], "length_m": geo["fus"]["length"],
                         "w_max_m": geo["fus"]["w_max"], "h_max_m": geo["fus"]["h_max"],
                         "S_wet_m2": geo["fus"]["S_wet"], "volume_m3": geo["fus"]["volume"],
                         "A_max_m2": geo["fus"]["A_max"], "S_side_m2": geo["fus"]["S_side"],
                         "aft_cowl_base_area_m2": geo["base_area"],
                         "keepouts_with_clearance": [{"name": n, "x0": a, "x1": b_, "yz_points": pts}
                                                     for n, a, b_, pts in geo["keepouts"]],
                         "bays": {"turret_x_m": geo["layout"].x_tur, "turret_mount_z_m": geo["layout"].z_mount,
                                  "battery_x_m": geo["layout"].bat, "avionics_x_m": geo["layout"].av,
                                  "parachute_x_m": geo["layout"].chute, "payload_bay_x_m": geo["layout"].pay,
                                  "fuel_fwd_x_m": geo["layout"].tank_f, "fuel_aft_x_m": geo["layout"].tank_a,
                                  "carry_through_x_m": geo["layout"].box, "aft_equipment_x_m": geo["layout"].aft,
                                  "firewall_x_m": geo["layout"].x_fw, "engine_x_m": geo["layout"].eng,
                                  "cg_target_x_m": geo["layout"].x_c},
                         "fuel_bladders_m3": geo["tank_volumes_m3"], "fuel_bladder_centroids_x_m":
                             geo["tank_centroids_x"], "fuel_centroid_x_m": geo["tank_centroid_x"]},
            "propeller": {"diameter_m": D_PROP, "x_plane_m": geo["layout"].x_prop, "hub_z_m": dv.z_engine,
                          "spacer_m": est("prop_spacer_m"), "static_tip_mach": perf["static_tip_mach"],
                          "static_tip_speed_m_s": math.pi * D_PROP * perf["static_rpm"] / 60,
                          "tip_speed_limit_m_s": TIP_LIMIT,
                          "ground_clearance_static_m": geo["gear"]["prop_static_clearance"],
                          "clearance_min_m": PROP_GROUND_CLEAR},
            "landing_gear": geo["gear"],
        },
        "aero": {
            "drag_buildup_DQ_m2_at_3000m": pols["ALT"].items, "cd_items_3000m": pols["ALT"].cd_items,
            "drag_buildup_DQ_m2_at_SL": pols["SL"].items,
            "cd0_nonwing_3000m": pols["ALT"].cd0_nonwing, "cd0": p_alt.cd0, "k": p_alt.k, "oswald_e": p_alt.e,
            "cd0_SL": p_sl.cd0, "e_SL": p_sl.e, "ld_max_3000m": p_alt.ld_max, "ld_max_SL": p_sl.ld_max,
            "cl_ld_max": p_alt.cl_ld_max, "cl_endurance_max": p_alt.cl_end_max,
            "e_inviscid_lifting_line": p_alt.sa["e_inviscid"], "e_used_induced": p_alt.e_ind,
            "CL_alpha_per_rad": p_alt.sa["CL_alpha"] * p_alt.k_a, "CL_alpha_sweep_factor": p_alt.k_a,
            "e_nita_scholz_check": AL.oswald_nita_scholz(dv.AR, dv.taper, geo["sweep_c4_deg"],
                                                         geo["fus"]["w_max"] / geo["b"]),
            "Re_root_tip_3000m": [p_alt.sa["Re_root"], p_alt.sa["Re_tip"]],
            "clmax": hl, "polar_3000m": {"CL": p_alt.CLt, "CD": p_alt.CDt},
            "polar_SL": {"CL": p_sl.CLt, "CD": p_sl.CDt},
            "cooling_drag_check": {"raymer_DQ_m2_SL_30": cooling_drag_DQ(30.0, 0.0, dv),
                                   "momentum_N_loiter": cooling_drag_momentum(lp["P_total_W"], lp["V_tas"]),
                                   "raymer_N_loiter": cooling_drag_DQ(lp["V_tas"], H_LOITER, dv) * 0.5 *
                                   AL.isa(H_LOITER)["rho"] * lp["V_tas"] ** 2},
            "turret_drag": turret_drag(dv)[1],
            "polar_rule": "tripped (x/c 0.075) NeuralFoil polars x 1.15 (baseline.yaml sizing rule); clean = upside",
        },
        "mass": {
            "mtow_kg": m0, "empty_kg": res["empty_kg"], "fuel_kg": res["fuel_kg"], "payload_kg": PAYLOAD_DESIGN,
            "fuel_fraction": res["fuel_kg"] / m0, "empty_fraction": res["empty_kg"] / m0,
            "structure_fraction": struct["total"] / m0,
            "comparables_empty_fraction_median": 0.60, "baseline_structure_budget_kg": 45.25,
            "fixed_equipment_kg": res["fixed"], "fixed_equipment_total_kg": sum(res["fixed"].values()),
            "structure_kg": {"wing": struct["wing"], "tail": struct["tail"], "fuselage": struct["fuselage"],
                             "maturity_margin": struct["maturity_margin"], "total": struct["total"]},
            "wing_structure_info": struct["wing_info"],
            "items_mtow_mass_x_z": items, "mtom_cap_kg": MTOM_CAP, "margin_to_cap_kg": MTOM_CAP - m0,
            "closure_history": res["hist"],
        },
        "stability": {k: stab[k] for k in ("x_np", "a_w", "deps_da", "Kf", "cm_alpha_fus", "a_t", "S_t", "cases",
                                           "sm_min", "sm_max", "x_cg_fwd", "x_cg_aft", "x_cg_mtow", "z_cg_mtow",
                                           "cn_beta", "volumes", "h_h")},
        "performance": {k: v for k, v in perf.items()},
        "field": field,
        "constraint_diagram": {"ws_grid_N_m2": cons["ws_grid_N_m2"][::10], "curves_W_N": {k: v[::10] for k, v in
                                                                                       cons["curves"].items()},
                               "ws_stall_limit_N_m2": cons["ws_stall_limit_N_m2"],
                               "ws_landing_limit_N_m2": cons["ws_landing_limit_N_m2"],
                               "design_point_sizinglib": {"ws_N_m2": cons["design_point_sizinglib"]["ws"],
                                                          "pw_W_N": cons["design_point_sizinglib"]["pw"],
                                                          "active": cons["design_point_sizinglib"]["active"]},
                               "selected_ws_N_m2": cons["ws_selected_N_m2"],
                               "pw_required_at_selected_W_N": cons["pw_required_at_selected_W_N"],
                               "pw_available_mcp_W_N": cons["pw_available_mcp_W_N"], "eta": cons["eta"],
                               "note": "engine fixed -> P/W is not a free variable; W/S is chosen by the closed-loop "
                                       "MTOW sweep within the stall limit"},
        "mission": {"segments": res["mission"]["segments"], "fuel_fraction": res["mission"]["ff"],
                    "block_time_h": res["mission"]["block_time_h"], "loiter_h": res["mission"]["loiter_s"] / 3600},
        "structures": {"n_limit_pos": geo["n_lim"], "n_ultimate": geo["n_ult"], "gust": geo["gust"],
                       "spar": struct["wing_info"]},
    }
    (HERE / "concept.yaml").write_text(
        "# YK-250 concept 'identity' - generated by calc.py; do not edit by hand.\n" +
        yaml.safe_dump(clean(doc), sort_keys=False, allow_unicode=True, width=120), encoding="utf-8")
    draw_sketch(res, perf, HERE / "sketch.png")
    draw_constraints(cons, res, HERE / "constraint.png")
    print(f"wrote concept.yaml, sketch.png, constraint.png ({time.time() - t0:.0f} s)")
    return res, perf, price


if __name__ == "__main__":
    main()
