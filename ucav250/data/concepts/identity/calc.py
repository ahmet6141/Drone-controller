#!/usr/bin/env python3
"""YK-250 concept study "identity": the strongest UCAV identity that still meets the mission.

Run from the repository root:

    PYTHONPATH=. python3 ucav250/data/concepts/identity/calc.py            (full study, about 6-8 min on 4 cores)
    PYTHONPATH=. python3 ucav250/data/concepts/identity/calc.py --quick    (development: no trades)

Concept (decided by the trade studies in this file; reasons in concept.yaml -> trades and notes_tr.md):
  * Limbach L 275 EF pusher, Mejzlik 32x18 2B on an 80 mm hub spacer, dorsal S-duct cooling inlet with a
    boundary-layer diverter, downdraft baffles, annular exit around the spinner.
  * Chined, deliberately faceted fuselage: superellipse sections with near-flat upper facets (n 1.35) meeting a sharp
    chine line, fuller lower facets (n 1.75, flat chin under the turret); every line is a C1/C2 blend of a few
    control points (no kinks), the chine runs straight along the centre body and rises into the engine hump.
  * Mid wing on the chine line, two halves joined by a spar tongue/fork through a spar tunnel; moderately swept
    leading edge (12 deg), taper 0.42, NLF(1)-0416 16 % -> 13 %, LERX-type root glove where the chine runs into the
    leading edge; inboard take-off flaps, outboard ailerons.
  * Y-tail: canted V panels (ruddervators) + ventral fin whose tip skid guards the propeller disc.
  * Fixed tricycle gear (GFRP spring bow, faired TOST wheels); retraction evaluated with numbers and rejected.
  * Chin EO/IR turret (HD59, bay for the E180 growth turret), payload bay on the CG between two fuel cells.
  Civil EO/IR surveillance and research platform: no weapons, munitions, hardpoints, pylons or release mechanisms.

Method. The reviewed endurance study (ucav250/data/concepts/endurance/calc.py) is imported as a library, so every
concept is judged with the same equations: engine WOT torque x sigma^1.23, Mejzlik 32x18 2B table with J-similarity,
part-load BSFC at the actual power setting (iterated with the generator load), tripped NLF polars x 1.15 strip-
integrated on the lifting-line cl, aero.surface_analysis CLmax with the research factors, CS-LUAS gust envelope,
spar caps from structlib beam loads, the mission (sizinglib segments: warm-up, take-off, climb, 2 x 100 km at best
range speed, loiter at minimum fuel flow above the 26 m/s EAS floor, descent, 10 % reserve, 2 % trapped fuel), level,
climb, loiter and range points, ceiling and the constraint diagram. This file adds what differs for the identity
configuration: the faceted OML and its keep-out fit, the swept wing with root glove (sweep corrections to the lifting
line), the Y-tail, the identity drag items (chine form factor, root glove, inlet ram recovery, ventral fin), the
mid-wing chassis, the ground geometry with the ventral skid, the identity-price / gear / tail / inlet trades and the
outputs. Tripped polars are cached in ./polars (identical NeuralFoil call); the shared polar cache is read only.

Frame: X aft from the nose tip, Y starboard, Z up; Z = 0 on the centre-body chine line. SI units, angles in degrees.
Outputs (this folder only): concept.yaml, sketch.png, constraint.png, polars/.
"""
from __future__ import annotations

import functools
import importlib.util
import json
import math
import sys
import time
from dataclasses import asdict, dataclass, replace
from pathlib import Path

import numpy as np
import yaml

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


def _load_endurance_study():
    """The reviewed endurance concept study, imported as a library (its files are never modified)."""
    name = "ucav250_concept_endurance_calc"
    if name in sys.modules:
        return sys.modules[name]
    spec = importlib.util.spec_from_file_location(name, HERE.parent / "endurance" / "calc.py")
    mod = importlib.util.module_from_spec(spec)
    sys.modules[name] = mod
    spec.loader.exec_module(mod)
    return mod


EC = _load_endurance_study()
G = AL.G0
RHO0 = 1.225

# =====================================================================================================================
# 0. polar caches: shared cache read-only, new polars written to ./polars (this concept owns only its own folder)
# =====================================================================================================================
LOCAL_POLARS = HERE / "polars"
LOCAL_POLARS.mkdir(exist_ok=True)
_shared_polar_path = AE._polar_path


def _polar_path(airfoil, Re, n_crit, ts=1.0):
    p = _shared_polar_path(airfoil, Re, n_crit, ts)
    return p if p.exists() else LOCAL_POLARS / p.name


AE._polar_path = _polar_path
_ec_raw_tripped = EC._raw_tripped


@functools.lru_cache(maxsize=None)
def _raw_tripped_cached(airfoil: str, Re: float, ts: float) -> dict:
    """Disk cache around the endurance study's tripped NeuralFoil polar (same call, same alpha grid)."""
    tag = "" if abs(ts - 1.0) < 1e-9 else f"_t{ts:.4f}"
    path = LOCAL_POLARS / f"{airfoil.lower()}{tag}_Re{int(round(Re)):d}_trip{EC.XTR_TRIP:g}.json"
    if path.exists():
        d = json.loads(path.read_text())
        return {k: d[k] for k in ("alpha", "cl", "cd", "cm", "confidence")}
    r = _ec_raw_tripped(airfoil, Re, ts)
    out = {"airfoil": airfoil, "thickness_scale": float(ts), "Re": float(Re), "n_crit": 9.0, "xtr": EC.XTR_TRIP, **r,
           "method": f"NeuralFoil xlarge, transition forced at x/c {EC.XTR_TRIP} both surfaces"}
    path.write_text(json.dumps(out, indent=1))
    return r


EC._raw_tripped = _raw_tripped_cached

# =====================================================================================================================
# 1. inputs: the research inputs shared with the endurance study + identity-specific values (source or basis each)
# =====================================================================================================================
SHARED_INPUT_KEYS = [
    "endurance_requirement_h", "loiter_speed_floor_m_per_s_eas", "cruise_speed_band_m_per_s",
    "service_ceiling_requirement_m", "loiter_altitude_m", "runway_length_m", "climb_rate_sea_level_requirement_m_per_s",
    "transit_radius_m", "reserve_fraction_of_trip_time", "trapped_fuel_fraction", "mtom_hard_cap_kg", "mtom_design_kg",
    "mass_growth_allowance_fraction_of_empty", "payload_design_kg", "payload_items_kg", "turret_bay_growth_envelope_m",
    "engine_power_max_W", "engine_power_max_continuous_W", "engine_wot_curve", "engine_power_lapse_exponent",
    "bsfc_curve_power_fraction_g_per_kWh", "fuel_density_kg_per_m3", "engine_group_installed_kg",
    "electrical_load_continuous_W", "generator_efficiency", "engine_envelope_m", "engine_intake_box_width_m",
    "propeller_diameter_m", "propeller_wot_thrust_factor", "propeller_installation_factor", "propeller_table",
    "prop_ground_clearance_min_m", "prop_hub_spacer_m", "cl_max_section_factor", "cd_factor_tripped",
    "trip_location_x_over_c", "k_clmax_3d", "VC_m_per_s_eas", "spar_cap_allowable_ultimate_Pa",
    "shear_web_allowable_ultimate_Pa", "areal_masses_kg_per_m2", "flap_kit_kg", "gear_fairings_kg", "main_wheel_tyre_m",
    "tank_volume_efficiency", "theta_touchdown_target_deg", "fuselage_bottom_static_height_min_m",
    "directional_stability_target_per_rad", "static_margin_min", "tail_volume_target_VH", "span_max_m",
    "tip_chord_min_m", "field_ground_roll_max_m",
]
INPUTS: dict = {k: {**EC.INPUTS[k], "shared_with": "endurance study (same research input / same rule)"}
                for k in SHARED_INPUT_KEYS if k in EC.INPUTS}
FLAGS: list = []


def rec(key: str, value, src: str, tag: str = "estimate", basis: str | None = None):
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


py = EC.py

# shared constants (same objects as the endurance study)
H_LOITER, H_CEIL_REQ, ROC_REQ = EC.H_LOITER, EC.H_CEIL_REQ, EC.ROC_REQ
MTOM_CAP, MTOM_DESIGN, GROWTH = EC.MTOM_CAP, EC.MTOM_DESIGN, EC.GROWTH
PAYLOAD, END_REQ_H = EC.PAYLOAD, EC.END_REQ_H
M_TURRET, M_MCOMP, M_TRAY, M_RESEARCH = EC.M_TURRET, EC.M_MCOMP, EC.M_TRAY, EC.M_RESEARCH
TUR_D, TUR_D_GROWTH, TUR_H_GROWTH = EC.TUR_D, EC.TUR_D_GROWTH, EC.TUR_H_GROWTH
ENV_L, ENV_L_SG, ENV_W, ENV_H, ENV_ZC = EC.ENV_L, EC.ENV_L_SG, EC.ENV_W, EC.ENV_H, EC.ENV_ZC
D_PROP, M_PROP, CLEAR_PROP_GROUND = EC.D_PROP, EC.M_PROP, EC.CLEAR_PROP_GROUND
HUB_SPACER, INTAKE_W = EC.HUB_SPACER, EC.INTAKE_W
UD, PW, TC_ROOT = EC.UD, EC.PW, EC.TC_ROOT
SKIN_PRIMARY, SKIN_SECONDARY, SKIN_TAIL, RIB_AREAL, PLY_PW = (EC.SKIN_PRIMARY, EC.SKIN_SECONDARY, EC.SKIN_TAIL,
                                                             EC.RIB_AREAL, EC.PLY_PW)
WHEEL_D, WHEEL_W, CHUTE_BOX = EC.WHEEL_D, EC.WHEEL_W, EC.CHUTE_BOX
K_TANK, RHO_FUEL, FOS, N_POS = EC.K_TANK, EC.RHO_FUEL, EC.FOS, EC.N_POS
E180_M = float(EC.BL["payload_set"]["turret_bay_growth_envelope"]["mass_kg"]["value"])
TUR_H = float(EC.PS["eo_ir_turret"]["height_m"]["value"])
rec("e180_growth_turret_mass_kg", E180_M, "baseline.yaml#payload_set.turret_bay_growth_envelope.mass_kg",
    basis="3.75 kg + mount; used for the growth loading case (research allowance reduced to keep 20 kg payload)")

# ---------------------------------------------------------------------------------------- identity-specific estimates
Q_CHINE = rec("fuselage_chine_form_factor_increment", 1.05, "", basis=(
    "sharp chines shed vortices at body incidence/sideslip; Raymer/DATCOM have no chine term, so 5 % is added to the "
    "chined fuselage friction drag (engineering estimate; the loiter body angle is within +/-3 deg by the incidence "
    "rule, where the chine vortex is weak)"))
Q_WING_NO_FILLET = rec("wing_interference_without_root_fairing", 1.04, "", basis=(
    "mid wing meeting the sharp chine at an acute corner without fairing (junction separation); between Raymer's 1.0 "
    "(well-filleted mid/high wing) and 1.1-1.4 (unfilleted low wing): estimate, applied to the wing profile drag"))
GLOVE_L = rec("root_glove_length_ahead_of_le_m", 0.30, "", basis="LERX-type root glove: starts 0.30 m ahead of the "
              "side-of-body leading edge on the chine line (identity design choice)")
GLOVE_DY = rec("root_glove_span_m", 0.14, "", basis="glove meets the main leading edge 0.14 m outboard of the side of "
               "body (about 70 deg glove sweep, same airfoil thickness as the root -> thinner t/c)")
FILLET_TE_L = rec("root_te_fillet_length_m", 0.15, "", basis="small trailing-edge root fairing on the chine "
                  "(identity design choice)")
INLET = {   # name: (ram recovery eta_r, external lip/diverter D/q m2, extra duct mass kg, description)
    "sduct": (0.88, 0.0008, 0.25, "dorsal S-duct with chevron lip and boundary-layer diverter (identity)"),
    "scoop": (0.92, 0.0015, 0.10, "raised pitot scoop with diverter (plain reference)"),
    "naca": (0.70, 0.0002, 0.20, "two flush NACA submerged inlets on the upper aft facets"),
}
FIN_PACK_DP_Q = 0.67
rec("cooling_inlets", {k: {"ram_recovery": v[0], "external_D_over_q_m2": v[1], "duct_mass_kg": v[2], "type": v[3]}
                       for k, v in INLET.items()}, "",
    basis=("ram recovery: order of magnitude from the NACA submerged-inlet tests (Mossman & Randall 1948, NACA RM "
           "A7I30; values as recalled, 0.6-0.9 with boundary-layer thickness and mass-flow ratio) and pitot-inlet "
           "practice; the boundary layer at the inlet station is about 45 mm thick (0.37 x Re_x^-0.2), so a flush inlet "
           "ingests mostly boundary-layer air. External drag and duct mass: estimates (lip/diverter frontal area x "
           "Cd 0.1-0.2; 0.3 m2 of 2-ply duct wall + lip)"))
rec("cooling_fin_pack_pressure_drop_over_q", FIN_PACK_DP_Q, "endurance study drag_buildup (exit velocity 0.5 V)",
    basis="cooling drag D = m_dot (V - V_e), V_e/V = sqrt(eta_r - dp/q); dp/q 0.67 makes the pitot scoop reproduce "
          "the endurance study's V_e = 0.5 V, so only the inlet recovery differs between concepts")
VENT_AR_FACTOR = rec("ventral_fin_effective_aspect_ratio_factor", 1.55, "", basis=(
    "fin with one end on the fuselage: effective AR about 1.5-1.6 x geometric (DATCOM end-plate trend, estimate)"))
RETRACT = rec("retractable_gear_mass_items_kg", {
    "sagitta_type_retract_gear_incl_emas_minus_fixed": 11.5 - 9.1,
    "doors_0p15m2_sandwich_hinges_links": 0.41, "well_close_outs_3x": 0.36,
    "cut_out_reinforcement_frames_longerons": 0.50, "up_down_lock_sensors_controller_harness": 0.25,
    "minus_wheel_and_leg_fairings": -EC.GEAR_FAIRINGS}, "components.yaml#categories.landing_gear.items"
    "[sagitta_retractable_gear].total_gear_mass_kg (11.5 kg) vs recommended fixed set 9.1 kg",
    basis="doors, close-outs, reinforcement and sensors are estimates (areal masses of the secondary sandwich + edge "
          "bands); wells: 2 x 0.45 x 0.10 x 0.20 m (main, SAGITTA-type telescopic leg + 0.19 m wheel) + nose "
          "0.40 x 0.08 x 0.20 m")
RETRACT_DM = sum(RETRACT.values())
RETRACT_WELL_M3 = 2 * 0.45 * 0.10 * 0.20 + 0.40 * 0.08 * 0.20
RETRACT_DRAG_LEFT = rec("retractable_gear_residual_drag_fraction", 0.10, "endurance study run_config_trades "
                        "(90 % of the gear drag removed)", basis="door gaps and well leakage (same rule as endurance)")
CHINE_BAND = rec("chine_edge_band_kg_per_m", 0.04, "", basis=(
    "solid-laminate band along each sharp chine edge (no core within 15 mm of the edge, 2 extra 0.2 mm plies over "
    "30 mm, joggle filler): 0.03 x 0.4e-3 x 1517 + filler = about 0.04 kg/m (estimate)"))
VENT_FITTINGS = rec("ventral_fin_root_fitting_and_skid_kg", 0.30, "", basis=(
    "replaceable skid shoe (UHMW-PE on a 4130 strap) 0.15 kg + root fitting 0.15 kg (estimate)"))


# =====================================================================================================================
# 2. design variables
# =====================================================================================================================
@dataclass(frozen=True)
class Design:
    mtom: float = MTOM_DESIGN
    ws: float = 450.0             # Pa (trade: stall-limited)
    AR: float = 13.0              # trade
    taper: float = 0.42
    sweep_le: float = 12.0        # deg, identity: moderately swept leading edge
    washout: float = 3.0          # deg: baseline 2.5 + 0.5 for the sweep-induced outboard loading
    dihedral: float = 2.5         # deg (mid wing)
    ts_tip: float = 0.8125        # NLF(1)-0416 scaled to 13 % at the tip (baseline)
    VH: float = 0.45
    cnb_req: float = 0.057
    VV_min: float = 0.020
    tail_AR: float = 4.0          # unfolded V-tail aspect ratio (2 s)^2 / S_V
    tail_taper: float = 0.55
    tail_le_sweep: float = 38.0   # deg (canted swept panels, UCAV look)
    sm_min: float = 0.10
    flaps: bool = True            # inboard plain take-off flaps (15 deg)
    flap_to_deg: float = 15.0
    flap_span_frac: float = 0.55
    cf_c: float = 0.25
    # identity features (each is replaced by its plain alternative in the identity-price trade)
    body: str = "chined"          # chined | smooth (n = 2 sections) | flat (n = 1, true flat facets)
    glove: bool = True            # LERX-type root glove + trailing-edge root fairing
    tail: str = "Y"               # Y (V + ventral fin skid) | V_bumper (endurance style) | invV
    inlet: str = "sduct"          # sduct | scoop | naca
    gear: str = "fixed_faired"    # fixed_faired | fixed_bare | retract


rec("planform_identity", {"taper": 0.42, "sweep_le_deg": 12.0, "washout_deg": 3.0, "dihedral_deg": 2.5}, "",
    basis="moderately swept leading edge (quarter chord about 8 deg, inside the lifting-line validity of aero.py); "
          "washout 2.5 deg (baseline) + 0.5 deg for the outboard loading of the sweep; taper 0.42 for a tip chord "
          ">= 0.25 m at AR 13")
rec("v_tail_panels", {"tail_AR": 4.0, "taper": 0.55, "le_sweep_deg": 38.0}, "",
    basis="canted, swept panels for the UCAV look (panel AR 2.0 each); lift slope from aero.surface_analysis")


# =====================================================================================================================
# 3. fuselage OML: few smooth control lines -> dense oml.Fuselage stations -> keep-out fit
# =====================================================================================================================
@dataclass(frozen=True)
class Body:
    x_hub: float = 3.95           # propeller-hub face (aft end of the engine); trade (aft-body length)
    x_nose: float = 1.15          # plan-view nose length (convex Hermite ogive)
    a_c: float = 0.25             # chine half-width, centre body
    a_e: float = 0.24             # chine half-width at the cylinders (0.397 m + cooling gap)
    r_exit: float = 0.16          # cowl-lip radius (annular exit 0.058 m2 around the 0.17 m spinner, cowl-flap ring)
    z_tip: float = -0.12          # nose tip below the chine datum (drooped "beak")
    zt_c: float = 0.20            # dorsal line, centre body
    zb_c: float = -0.25           # keel line, centre body
    z_t: float = 0.14             # thrust line = chine height at the engine bay
    hump: float = 0.17            # engine hump top above the thrust line (cylinders 0.06 + plenum 0.07 + skin)
    keel_e: float = 0.32          # engine-bay keel below the thrust line (intake box 0.2346 + clearance)
    x_top: float = 1.25           # nose top line length
    x_bot: float = 0.70           # chin length
    x_chine: float = 1.35         # nose chine-line rise length
    s_top: float = 0.50           # tip slope of the top line (dz/dx)
    s_bot: float = 0.55           # tip slope of the chin line
    s_side: float = 0.33          # tip slope of the plan-view chine (half angle about 18 deg)
    beta_end: float = 24.0        # cowl boat-tail angle at the lip
    n_top: float = 1.35           # upper facets (n < 2: sharp chine and dorsal ridge)
    n_bot: float = 1.75           # lower facets
    n_chin: float = 2.4           # flat chin under the turret and nose leg (blends to n_bot by x = 1.6 m)
    n_cowl: float = 2.4           # boxy engine cowl (plenum over the cylinders, intake box below)
    l_hump: float = 1.15          # hump ramp starts this far ahead of the hub
    l_upsweep: float = 1.30       # keel upsweep starts this far ahead of the hub


BODY_STYLES = {"chined": "upper facets n 1.35, lower facets n 1.75, flat chin n 2.4, boxy cowl n 2.4",
               "smooth": "elliptic sections n = 2 (plain reference)", "flat": "true flat upper facets n = 1"}
rec("fuselage_lines", asdict(Body()), "", basis=(
    "designer's control lines (identity choice): plan-view chine = convex cubic Hermite ogive (tip half-angle 18 deg) "
    "to the constant 0.50 m centre body, smootherstep taper to the engine bay and a cubic boat-tail to the cowl lip; "
    "top/keel/chine lines likewise; dense stations (25 mm) so the oml.Fuselage PCHIP surface reproduces the lines; "
    "every keep-out checked with 10-20 mm clearance (checks.keepouts)"))


def _herm(t, y0, y1, m0, m1):
    t = np.clip(t, 0.0, 1.0)
    return ((2 * t ** 3 - 3 * t ** 2 + 1) * y0 + (t ** 3 - 2 * t ** 2 + t) * m0 + (-2 * t ** 3 + 3 * t ** 2) * y1 +
            (t ** 3 - t ** 2) * m1)


def _sstep(t):
    t = np.clip(t, 0.0, 1.0)
    return t * t * t * (t * (6 * t - 15) + 10)            # C2 smootherstep


def body_lines(x, b: Body, style: str = "chined"):
    """(half width, top z, keel z, chine z, n_top, n_bot) at stations x."""
    x = np.asarray(x, float)
    xe, xend = b.x_hub - 0.05, b.x_hub + 0.05                 # cylinder slab aft edge -> cowl lip
    tb = math.tan(math.radians(b.beta_end)) * (xend - xe)
    te = (x - xe) / (xend - xe)
    # plan-view chine
    a = np.where(x <= b.x_nose, _herm(x / b.x_nose, 0.0, b.a_c, b.s_side * b.x_nose, 0.0), b.a_c)
    x_c0 = b.x_hub - 1.20
    a = np.where(x > x_c0, b.a_c + (b.a_e - b.a_c) * _sstep((x - x_c0) / (xe - 0.15 - x_c0)), a)
    a = np.where(x > xe, _herm(te, b.a_e, b.r_exit, 0.0, -tb), a)
    # chine line: drooped tip -> datum -> thrust line at the engine bay
    zc = np.where(x <= b.x_chine, _herm(x / b.x_chine, b.z_tip, 0.0, 0.25 * abs(b.z_tip), 0.0), 0.0)
    zc = np.where(x > x_c0, b.z_t * _sstep((x - x_c0) / (xe - 0.10 - x_c0)), zc)
    # top line: convex nose -> dorsal line -> engine hump -> cowl closure
    zt = np.where(x <= b.x_top, _herm(x / b.x_top, b.z_tip, b.zt_c, b.s_top * b.x_top, 0.0), b.zt_c)
    x_h0, x_h1 = b.x_hub - b.l_hump, b.x_hub - 0.55
    zt_e = b.z_t + b.hump
    zt = np.where(x > x_h0, b.zt_c + (zt_e - b.zt_c) * _sstep((x - x_h0) / (x_h1 - x_h0)), zt)
    zt = np.where(x > xe, _herm(te, zt_e, b.z_t + b.r_exit, 0.0, -1.15 * tb), zt)
    # keel line: chin -> flat belly -> upsweep -> closure
    zb = np.where(x <= b.x_bot, _herm(x / b.x_bot, b.z_tip, b.zb_c, -b.s_bot * b.x_bot, 0.0), b.zb_c)
    x_u0 = b.x_hub - b.l_upsweep
    zb_e = b.z_t - b.keel_e
    zb = np.where(x > x_u0, b.zb_c + (zb_e - b.zb_c) * _sstep((x - x_u0) / (xe - 0.05 - x_u0)), zb)
    zb = np.where(x > xe, _herm(te, zb_e, b.z_t - b.r_exit, 0.0, 1.15 * tb), zb)
    # facet exponents: faceted body, flat chin forward, round cowl lip around the spinner
    nt = np.full_like(x, b.n_top)
    nb = b.n_chin + (b.n_bot - b.n_chin) * _sstep((x - 0.95) / 0.65)
    if style == "smooth":                                   # identity-price variant: elliptic sections
        nt = np.full_like(x, 2.0)
        nb = np.full_like(x, 2.0)
    elif style == "flat":                                   # identity-price variant: true flat upper facets
        nt = np.full_like(x, 1.0)
    tc = _sstep((x - (b.x_hub - 0.95)) / 0.60)            # faceting fades into the boxy engine cowl
    nt = nt + (b.n_cowl - nt) * tc
    nb = nb + (b.n_cowl - nb) * tc
    tl = _sstep((x - (xe - 0.10)) / (xend - (xe - 0.10)))   # round cowl lip around the spinner
    nt = nt + (2.0 - nt) * tl
    nb = nb + (2.0 - nb) * tl
    return a, zt, zb, zc, nt, nb


def body_stations(b: Body, style: str = "chined", dx: float = 0.025) -> np.ndarray:
    xend = b.x_hub + 0.05
    xs = np.unique(np.r_[np.arange(0.0, xend, dx), xend])
    a, zt, zb, zc, nt, nb = body_lines(xs, b, style)
    h = np.maximum(zt - zb, 0.003)
    tf = np.clip((zt - zc) / h, 0.06, 0.94)
    st = np.column_stack([xs, 2 * np.maximum(a, 0.002), h, zc, nt, nb, tf])
    st[0, 1:4] = [0.004, 0.003, b.z_tip]
    st[0, 6] = 0.5
    return st


# global fuselage state (same pattern as the endurance study's set_fuselage)
BODY = Body()
STYLE = "chined"
FUS = FUSE = None
L_FUS = X_HUB = X_PROP = X_LIP = Z_T = 0.0
SPINNER: dict = {}


def set_body(b: Body, style: str = "chined"):
    global BODY, STYLE, FUS, FUSE, L_FUS, X_HUB, X_PROP, X_LIP, Z_T, SPINNER
    BODY, STYLE = b, style
    FUS = body_stations(b, style)
    FUSE = oml.Fuselage(FUS)
    L_FUS = float(FUS[-1, 0])
    X_HUB = b.x_hub
    X_PROP = X_HUB + HUB_SPACER + 0.02                    # prop plane (mid hub), endurance-study rule
    X_LIP = X_HUB + 0.05                                  # cowl lip / annular cooling exit
    Z_T = b.z_t
    SPINNER = {"x0": X_HUB + 0.07, "x1": X_PROP + 0.12, "d": 0.17}
    fuselage_mesh_props.cache_clear()


def fus_section(x):
    xx = np.atleast_1d(np.asarray(x, float))
    w, h, zc, nt, nb = FUSE.section(xx)
    return w, h, zc, nt, nb, FUSE.top_frac(xx)


def z_top(x):
    w, h, zc, nt, nb, tf = fus_section(x)
    return zc + tf * h


def z_bot(x):
    w, h, zc, nt, nb, tf = fus_section(x)
    return zc - (1 - tf) * h


def _sec(x):
    w, h, zc, nt, nb, tf = (float(v[0]) for v in fus_section(x))
    return w / 2, tf * h, (1 - tf) * h, zc, nt, nb


def z_surface(x, y, upper=True):
    """Upper/lower OML height at (x, |y|)."""
    a, bt, bb, zc, nt, nb = _sec(x)
    r = min(abs(y) / max(a, 1e-6), 1.0)
    if upper:
        return zc + bt * (1 - r ** nt) ** (1 / nt)
    return zc - bb * (1 - r ** nb) ** (1 / nb)


def half_width_at(x, z, inset=0.0):
    """Half width of the OML (inset by ``inset`` normal-ish) at height z."""
    a, bt, bb, zc, nt, nb = _sec(x)
    a, bt, bb = a - inset, bt - inset, bb - inset
    if a <= 0:
        return 0.0
    dz = z - zc
    if dz >= 0:
        if bt <= 0 or dz >= bt:
            return 0.0
        return a * (1 - (dz / bt) ** nt) ** (1 / nt)
    if bb <= 0 or -dz >= bb:
        return 0.0
    return a * (1 - (-dz / bb) ** nb) ** (1 / nb)


def inside_oml(x, y, z, margin=0.0) -> bool:
    return abs(y) <= half_width_at(x, z, margin) + 1e-12


def band_area(x, inset, z_lo=-9.0, z_hi=9.0, n=90):
    """Cross-section area of the inset OML between z_lo and z_hi (vectorised numerical integration)."""
    a, bt, bb, zc, nt, nb = _sec(x)
    a, bt, bb = a - inset, bt - inset, bb - inset
    lo, hi = max(z_lo, zc - bb), min(z_hi, zc + bt)
    if hi <= lo or a <= 0:
        return 0.0
    zz = np.linspace(lo, hi, n)
    dz = zz - zc
    up = dz >= 0
    r = np.where(up, np.abs(dz) / max(bt, 1e-9), np.abs(dz) / max(bb, 1e-9))
    n_ = np.where(up, nt, nb)
    w = 2 * a * np.clip(1 - np.clip(r, 0, 1) ** n_, 0, 1) ** (1 / n_)
    return float(np.trapz(w, zz))


def superellipse_half_area(a, b, n):
    return 2 * a * b * math.gamma(1 + 1 / n) ** 2 / math.gamma(1 + 2 / n)


@functools.lru_cache(maxsize=1)
def fuselage_mesh_props() -> dict:
    m = FUSE.mesh(170, 112)
    V, F = m.V, m.F
    A = 0.5 * np.linalg.norm(np.cross(V[F[:, 1]] - V[F[:, 0]], V[F[:, 2]] - V[F[:, 0]]), axis=1)
    C = V[F].mean(axis=1)
    cap = C[:, 0] > L_FUS - 1e-4                          # flat end cap = annular cooling exit + spinner base
    area = float(A[~cap].sum())
    xc = float((A[~cap] * C[~cap, 0]).sum() / area)
    xs = np.linspace(0, L_FUS, 500)
    ws, hs, *_ = fus_section(xs)
    side = float(np.trapz(hs, xs))
    sec_area = []
    for x in np.linspace(0.3, L_FUS - 0.1, 60):
        a, bt, bb, zc, nt, nb = _sec(x)
        sec_area.append(superellipse_half_area(a, bt, nt) + superellipse_half_area(a, bb, nb))
    return {"S_wet": area, "x_centroid": xc, "volume": float(abs(m.volume())), "S_side": side,
            "A_max": float(max(sec_area)), "w_max": float(ws.max()), "h_max": float(hs.max()), "mesh": m}


set_body(Body())

# ------------------------------------------------------------------------------------------------ fixed bays (x, m)
X_TURRET = rec("turret_x_m", 0.55, "", basis="chin turret under the chine nose: clear forward/lateral field of view, "
               "ahead of the nose leg (identity layout)")
X_BATT = 0.55                                    # 14S2P LiFePO4 buffer pack above the turret (balances the engine)
X_AVION = 0.84                                   # autopilot, IMU, GNSS, transponder, datalink tray over the nose leg
X_PDU = 1.12                                     # VAT 1000 W PDU + converters under the parachute container
X_NG = 0.80                                      # nose-gear trunnion frame behind the turret bay
X_CHUTE = (0.99, 1.365)                          # GRS 4/240 container under a dorsal hatch, ahead of the fuel cells
BAY_LEN = rec("payload_bay_length_m", 0.40, "endurance study payload_bay_length_m",
              basis="research payload bay (nadir hatch) centred on the CG under the wing spar tunnel, between two "
                    "interconnected fuel cells (same rule as the endurance study)")
TANK_INSET = 0.03
Z_WING = rec("wing_root_le_z_m", -0.015, "", basis="mid wing: root chord plane 15 mm below the centre-body chine so the "
             "NLF(1)-0416 thickness straddles the chine; the spar tongue runs through a tunnel at this height")
SPAR_BAND = (Z_WING - 0.06, Z_WING + 0.10)       # spar tunnel / carry-through band (root airfoil + 10 mm)


def turret_mount_z() -> float:
    """Turret mount plane: 55 mm above the chin keel line, the ball hangs through the chin opening."""
    return float(z_bot(X_TURRET)[0]) + 0.055


def chute_top_z() -> float:
    """Top of the parachute container: the container sits on the equipment deck 10 mm above the local chine line; the
    extraction rocket and canopy leave upward through a blow-off dorsal hatch."""
    return float(max(fus_section(np.linspace(*X_CHUTE, 5))[2])) + 0.01 + CHUTE_BOX[1]


def battery_z() -> float:
    """Bottom of the battery pack: 10 mm above the E180 envelope over the turret mount plane."""
    return turret_mount_z() + 0.10


def keepouts(b: Body) -> list:
    """Fixed packaging keep-outs [(name, x0, x1, [(y, z), ...], clearance m)]; y >= 0 corners (mirrored)."""
    zt = b.z_t
    zm = turret_mount_z()
    r = TUR_D_GROWTH / 2 + 0.005
    ko = []
    for k, u in enumerate(np.linspace(-0.95, 0.95, 9)):        # E180 growth turret above the mount plane (cylinder)
        ko.append((f"turret_E180_bay_above_mount_plane[{k}]", X_TURRET + u * r - 0.005, X_TURRET + u * r + 0.005,
                   [(r * math.sqrt(1 - u * u), zm), (r * math.sqrt(1 - u * u), zm + 0.09)], 0.008))
    zb_ = battery_z()
    ko += [
        ("battery_14S2P_26650_case", X_BATT - 0.13, X_BATT + 0.13, [(0.06, zb_), (0.06, zb_ + 0.08)], 0.010),
        ("avionics_datalink_tray", X_AVION - 0.14, X_AVION + 0.14, [(0.11, -0.09), (0.11, 0.04)], 0.010),
        ("pdu_converters_under_chute", X_PDU - 0.12, X_PDU + 0.12, [(0.09, -0.20), (0.09, -0.07)], 0.010),
        ("nose_gear_trunnion_steering", X_NG - 0.06, X_NG + 0.06, [(0.06, float(z_bot(X_NG)[0]) + 0.02),
                                                                  (0.06, float(z_bot(X_NG)[0]) + 0.15)], 0.0),
        ("parachute_GRS_4_240", X_CHUTE[0], X_CHUTE[1], [(CHUTE_BOX[0] / 2, chute_top_z() - CHUTE_BOX[1]),
                                                         (CHUTE_BOX[0] / 2, chute_top_z())], 0.010),
        ("cylinder_slab", b.x_hub - 0.18, b.x_hub - 0.05, [(ENV_W / 2, zt - ENV_ZC), (ENV_W / 2, zt + ENV_ZC)], 0.020),
        ("intake_box_below_crank", b.x_hub - 0.18, b.x_hub - 0.05,
         [(INTAKE_W / 2, zt - (ENV_H - ENV_ZC)), (INTAKE_W / 2, zt - ENV_ZC)], 0.015),
        ("crankcase_sg750_core", b.x_hub - ENV_L_SG, b.x_hub - 0.02, [(0.10, zt - 0.10), (0.10, zt + 0.05)], 0.015),
        ("downdraft_plenum_over_cylinders", b.x_hub - 0.20, b.x_hub - 0.06,
         [(0.17, zt + ENV_ZC + 0.005), (0.17, zt + ENV_ZC + 0.055)], 0.010),
        ("sduct_duct_into_plenum", b.x_hub - 0.45, b.x_hub - 0.22, [(0.11, zt + 0.06), (0.11, zt + 0.12)], 0.010),
        ("aft_equipment_ecu_pump_header", b.x_hub - 0.62, b.x_hub - 0.32, [(0.10, zt - 0.20), (0.10, zt - 0.04)], 0.010),
    ]
    return ko


def keepout_margins(b: Body) -> dict:
    out = {}
    for name, x0, x1, pts, c in keepouts(b):
        worst = math.inf
        for x in np.linspace(x0, x1, 9):
            for y, z in pts:
                worst = min(worst, half_width_at(x, z, c) - y)
        out[name] = {"x_m": [x0, x1], "corners_yz_m": pts, "clearance_m": c, "min_margin_m": worst,
                     "fits": bool(worst >= -1e-4)}
    return out


def fit_body(b: Body, style: str) -> Body:
    """Grow the control lines until every keep-out fits (same rule for every style, so the identity-price trade is
    fair): centre width/height for the forebody items, engine-bay width/hump/keel for the engine items."""
    for _ in range(80):
        set_body(b, style)
        m = keepout_margins(b)
        bad = {k: v for k, v in m.items() if not v["fits"]}
        if not bad:
            return b
        ch = {}
        for k in bad:
            if k.startswith(("cylinder", "plenum", "downdraft")):
                ch["a_e"] = 1.02
                ch["hump"] = 1.02
            elif k.startswith(("intake", "crankcase", "aft_equipment")):
                ch["keel_e"] = 1.02
                ch["a_e"] = 1.01
            elif k.startswith("sduct"):
                ch["hump"] = 1.02
            elif k.startswith(("turret", "nose_gear", "battery", "avionics", "pdu")):
                if style == "chined":
                    raise RuntimeError(f"forebody keep-out {k} does not fit the designed nose; rearrange the bays")
                ch["s_top"] = 1.04                     # variants: fuller nose near the tip, same nose length
                ch["s_side"] = 1.02
            else:
                ch["zt_c"] = 1.02
                ch["a_c"] = 1.01
        b = replace(b, **{k: getattr(b, k) * f for k, f in ch.items()})
    raise RuntimeError("fuselage keep-out fit did not converge: " + ", ".join(
        f"{k} {v['min_margin_m'] * 1000:.0f} mm" for k, v in keepout_margins(b).items() if not v["fits"]))


# =====================================================================================================================
# 4. geometry builders: swept wing with root glove, Y-tail (canted V + ventral fin skid)
# =====================================================================================================================
def y_sob() -> float:
    """Side-of-body half-width at the wing root (chine of the constant centre body)."""
    return BODY.a_c


def sweep_angles(d: Design, pf: dict) -> dict:
    s = pf["b"] / 2
    tle = math.tan(math.radians(d.sweep_le))
    dc = (pf["cr"] - pf["ct"]) / s
    f = lambda xc: math.degrees(math.atan(tle - xc * dc))
    return {"le": d.sweep_le, "c4": f(0.25), "c2": f(0.50), "spar": f(0.30), "hinge": f(0.75), "te": f(1.0)}


def wing_sections(d: Design, pf: dict, x_w: float, i_w: float, analysis: bool = True, glove: bool | None = None):
    """LiftingSurface sections, starboard. ``x_w`` = leading edge x at the side of body. Analysis: reference
    trapezoid continued to the centre line (aero.py convention), side of body, tip. OML: exposed panel from the side
    of body, with the LERX-type glove (constant absolute thickness, thinner t/c) when ``glove``."""
    s = pf["b"] / 2
    ys = y_sob()
    tle = math.tan(math.radians(d.sweep_le))
    x_le0 = x_w - ys * tle
    glove = d.glove if glove is None else glove

    def sec(y):
        t = max(0.0, (y - ys) / (s - ys))
        return {"y": float(y), "x_le": float(x_le0 + y * tle),
                "z_le": float(Z_WING + max(0.0, y - ys) * math.tan(math.radians(d.dihedral))),
                "chord": float(pf["cr"] + (pf["ct"] - pf["cr"]) * y / s), "twist_deg": float(i_w - d.washout * t),
                "airfoil": "nlf416", "thickness_scale": float(1.0 + (d.ts_tip - 1.0) * t)}
    if analysis:
        return [sec(0.0), sec(ys), sec(s)]
    root, tip = sec(ys), sec(s)
    if not glove:
        return [root, tip]
    kink = sec(ys + GLOVE_DY)
    g = dict(root)
    g["x_le"] = root["x_le"] - GLOVE_L
    g["chord"] = root["chord"] + GLOVE_L
    g["thickness_scale"] = root["thickness_scale"] * root["chord"] / g["chord"]
    return [g, kink, tip]


def wing_ref(d: Design, pf: dict, x_w: float) -> dict:
    """Reference-trapezoid MAC (from the centre line) and aerodynamic centre."""
    tle = math.tan(math.radians(d.sweep_le))
    x_le0 = x_w - y_sob() * tle
    x_le_mac = x_le0 + pf["y_mac"] * tle
    return {"mac": pf["mac"], "x_le_mac": x_le_mac, "y_mac": pf["y_mac"], "x_ac": x_le_mac + 0.25 * pf["mac"],
            "x_le0": x_le0, "z_mac": Z_WING + max(pf["y_mac"] - y_sob(), 0) * math.tan(math.radians(d.dihedral))}


def tail_sections(d: Design, S_V: float, gamma_deg: float, x_te_root: float, inverted: bool = False) -> list:
    """V-tail panel (starboard): root on the upper facets of the engine hump (lower facets if inverted)."""
    span2 = math.sqrt(d.tail_AR * S_V)
    s = span2 / 2
    cr = S_V / (s * (1 + d.tail_taper))
    ct = d.tail_taper * cr
    g = math.radians(gamma_deg)
    y0 = 0.10
    x_le0 = x_te_root - cr
    if inverted:
        z0 = z_surface(x_le0 + 0.4 * cr, y0, upper=False) + 0.015
        sz = -1.0
    else:
        z0 = z_surface(x_le0 + 0.4 * cr, y0, upper=True) - 0.015
        sz = 1.0
    dx = s * math.tan(math.radians(d.tail_le_sweep))
    return [{"y": y0, "x_le": x_le0, "z_le": z0, "chord": cr, "twist_deg": 0.0, "airfoil": "n0012"},
            {"y": y0 + s * math.cos(g), "x_le": x_le0 + dx, "z_le": z0 + sz * s * math.sin(g), "chord": ct,
             "twist_deg": 0.0, "airfoil": "n0012"}]


VENT_TIP_CHORD = 0.17
VENT_ROOT_CHORD = 0.40
SKID_LEAD_DEG = rec("ventral_skid_lead_angle_deg", 14.0, "", basis=(
    "the skid tip lies below the line through the propeller-disc bottom inclined at 14 deg (typical tail-down "
    "attitude at the prop-strike limit), so in any tail-down rotation the skid touches before the blade tip"))


def ventral_fin() -> dict:
    """Ventral fin = propeller guard + tail skid (MQ-9-style Y-tail): tip skid 30 mm below the propeller disc and
    below the 14 deg line through the disc bottom, tip trailing edge 60 mm ahead of the propeller plane, root on the
    cowl keel ending 40 mm ahead of the hub face."""
    R = EC.D_PROP / 2
    x_tip_te = X_PROP - 0.06
    x_tip_le = x_tip_te - VENT_TIP_CHORD
    x_skid = 0.5 * (x_tip_le + x_tip_te)
    z_skid = Z_T - R - 0.03 - (X_PROP - x_skid) * math.tan(math.radians(SKID_LEAD_DEG))
    x_root_te = X_HUB - 0.04
    x_root_le = x_root_te - VENT_ROOT_CHORD
    z_root = float(z_bot(0.5 * (x_root_le + x_root_te))[0]) + 0.015
    span = z_root - z_skid
    cr, ct = VENT_ROOT_CHORD, VENT_TIP_CHORD
    S = span * (cr + ct) / 2
    lam = ct / cr
    y_mac = span / 3 * (1 + 2 * lam) / (1 + lam)
    mac = 2 / 3 * cr * (1 + lam + lam ** 2) / (1 + lam)
    tl = (x_tip_le - x_root_le) / span
    secs = [{"y": 0.0, "x_le": x_root_le, "z_le": z_root, "chord": cr, "twist_deg": 0.0, "airfoil": "n0012"},
            {"y": 0.0, "x_le": x_tip_le, "z_le": z_skid, "chord": ct, "twist_deg": 0.0, "airfoil": "n0012"}]
    AR_g = span ** 2 / S
    sw_half = math.degrees(math.atan(tl - 0.5 * (cr - ct) / span))
    return {"secs": secs, "S": S, "span": span, "cr": cr, "ct": ct, "mac": mac, "z_skid": z_skid, "x_skid": x_skid,
            "x_ac": x_root_le + y_mac * tl + 0.25 * mac, "a": AL.lift_slope(VENT_AR_FACTOR * AR_g, sw_half),
            "AR_geometric": AR_g, "sweep_half_deg": sw_half, "le_sweep_deg": math.degrees(math.atan(tl))}


# =====================================================================================================================
# 5. aerodynamics: endurance-study lifting line + strip drag, sweep corrections, identity drag items, trimmed polar
# =====================================================================================================================
def build_aero(d: Design, pf: dict, wsecs: list, V_ref: float, h_ref: float) -> dict:
    """EC.build_aero (lifting line, critical-section CLmax with the research factors, tripped strip drag) plus the
    sweep corrections the Glauert solution neglects: CL_alpha x Helmbold ratio (half-chord sweep), e_inviscid x
    Nita-Scholz ratio (quarter-chord sweep), CLmax x cos(sweep c/4) (simple sweep theory, Raymer)."""
    a = EC.build_aero(d, pf, wsecs, V_ref, h_ref, 0.0)
    sw = sweep_angles(d, pf)
    k_a = AL.lift_slope(d.AR, sw["c2"]) / AL.lift_slope(d.AR, 0.0)
    k_e = AL.oswald_nita_scholz(d.AR, d.taper, sw["c4"]) / AL.oswald_nita_scholz(d.AR, d.taper, 0.0)
    k_c = math.cos(math.radians(sw["c4"]))
    sa = dict(a["sa"])
    cla = a["CL_alpha"] * k_a
    cl0 = a["CL_0"]
    sa["CL_alpha"] = cla
    sa["alpha_for"] = lambda CL, _c=cla, _0=cl0: (CL - _0) / _c
    # basic-loading pitching moment of the twisted swept wing: zero-lift (washout) loading x sweep arm of the local
    # aerodynamic centres about the MAC quarter chord (DATCOM/Raymer method; zero for an unswept quarter chord)
    T, LL = a["sa"]["tables"], a["sa"]["ll"]
    y, c = LL["y"], LL["chord"]
    cl_b = LL["cl_local"](a["sa"]["alpha_for"](0.0))
    x_ac_loc = np.interp(y, T["y"], T["x_le"]) + 0.25 * c
    x_ac_ref = np.interp(pf["y_mac"], T["y"], T["x_le"]) + 0.25 * pf["mac"]
    o = np.argsort(y)
    dcm0 = float(-2 * np.trapz((c * cl_b * (x_ac_loc - x_ac_ref))[o], y[o]) / (pf["S"] * pf["mac"]))
    out = dict(a)
    out.update({"sa": sa, "sa_raw": a["sa"], "CL_alpha": cla, "e_inv": a["e_inv"] * k_e, "CLmax_wing": a["CLmax_wing"] *
                k_c, "k_sweep": {"CL_alpha": k_a, "e": k_e, "CLmax": k_c}, "sweep": sw,
                "alpha_for": sa["alpha_for"], "cm0_sections": a["cm0"], "dcm0_sweep_washout": dcm0,
                "cm0": a["cm0"] + dcm0})
    return out


def gear_drag(kind: str, gear: dict) -> float:
    """Endurance-study gear formula (Raymer Table 12.6 D/q per frontal area, x 1.2 interference) for the faired
    gear; bare variant: bare wheels 0.25, flat spring legs 1.0, bare nose fork 1.0 (same table); retractable:
    residual door/well drag."""
    A_wheel = WHEEL_D * WHEEL_W
    leg_main = gear["h_gear"] * 1.35 * 0.035
    leg_nose = gear["h_nose_leg"] * 0.040
    faired = 1.2 * (2 * 0.13 * A_wheel + 2 * 0.05 * leg_main + 0.35 * A_wheel + 0.05 * leg_nose)
    if kind == "fixed_bare":
        return 1.2 * (3 * 0.25 * A_wheel + 2 * 1.0 * leg_main + 1.0 * leg_nose)
    if kind == "retract":
        return RETRACT_DRAG_LEFT * faired
    return faired


def glove_wetted(d: Design, pf: dict, x_w: float) -> float:
    """Wetted area of the glove triangle + trailing-edge root fairings (both sides)."""
    if not d.glove:
        return 0.0
    return 2 * (2 * 0.5 * GLOVE_L * GLOVE_DY * 1.02 + 2 * 0.5 * FILLET_TE_L * 0.06)


def drag_buildup(d: Design, pf: dict, lay: dict, V: float, h: float, fuel_flow_ratio: float) -> dict:
    """Parasite drag items other than the wing profile drag, as D/q (m2) at TAS V, altitude h. Same formulas as the
    endurance study for the common items; identity items: chine form factor, root glove, ventral fin, inlet type."""
    atm = AL.isa(h)
    M = V / atm["a"]
    fp = fuselage_mesh_props()
    items = {}
    # fuselage: OML mesh wetted area minus the mid-wing root footprint on both sides
    c_s = pf["cr"] + (pf["ct"] - pf["cr"]) * y_sob() / (pf["b"] / 2)
    foot = 2 * 0.75 * TC_ROOT * c_s * c_s
    d_eq = math.sqrt(4 * fp["A_max"] / math.pi)
    Re_f = AL.reynolds(V, L_FUS, h)
    q_ch = Q_CHINE if STYLE != "smooth" else 1.0      # flat facets: same chine increment
    items["fuselage"] = AL.cf_flat(Re_f, 0.0) * AL.ff_body(L_FUS / d_eq) * q_ch * (fp["S_wet"] - foot)
    x_u0 = BODY.x_hub - BODY.l_upsweep
    xe = BODY.x_hub - 0.05
    u = math.atan2(float(z_bot(xe)[0] - z_bot(x_u0)[0]), xe - x_u0)
    items["fuselage_upsweep"] = 3.83 * abs(u) ** 2.5 * fp["A_max"]
    # root glove + TE fairings (thin, t/c about 0.11): wing formulas, Q 1.0
    if d.glove:
        Re_g = AL.reynolds(V, GLOVE_L + c_s, h)
        items["root_glove_fairings"] = AL.cf_flat(Re_g) * AL.ff_wing(0.11, 0.30, 0.0, M) * glove_wetted(d, pf, 0.0)
    # tail surfaces (endurance formula, NACA 0012, Q 1.03 V-tail; ventral fin Q 1.04, extra junction)
    t = lay["tail"]
    S_exp_t = t["S_V"] - 2 * 0.5 * t["cr"] * 0.02
    Re_t = AL.reynolds(V, t["mac"], h)
    items["tail"] = AL.cf_flat(Re_t, 0.0) * AL.ff_wing(0.12, 0.30, t["sweep_mt_deg"], M) * 1.03 * \
        AL.wing_wetted(S_exp_t, 0.12)
    if d.tail == "Y":
        vf = lay["ventral"]
        Re_v = AL.reynolds(V, vf["mac"], h)
        items["ventral_fin_skid"] = AL.cf_flat(Re_v, 0.0) * AL.ff_wing(0.12, 0.30, vf["sweep_half_deg"], M) * 1.04 * \
            AL.wing_wetted(vf["S"], 0.12) + 0.0002
    else:
        items["tail_bumper_skid"] = 0.0002
    items["landing_gear"] = gear_drag(d.gear, lay["gear"])
    # EO/IR turret: endurance-study item (exposed ball, subcritical sphere CD 0.47, 0.12 m exposed height)
    items["eo_ir_turret"] = 0.47 * TUR_D * 0.12
    # cooling: endurance-study momentum model, exit velocity from the inlet ram recovery
    eta_r, dq_ext, _, _ = INLET[d.inlet]
    ve = math.sqrt(max(eta_r - FIN_PACK_DP_Q, 0.0))
    m_dot = EC.EN["installation"]["cooling"]["cooling_airflow_estimate"]["mass_flow_kg_per_s_at_max_power"]["value"] * \
        fuel_flow_ratio
    items["engine_cooling"] = m_dot * V * (1 - ve) / (0.5 * atm["rho"] * V * V)
    items["cooling_inlet_lip_diverter"] = dq_ext
    items["antennas_pitot_lights_vents"] = 0.0025
    items["control_surface_gaps"] = 0.0002 * pf["S"] * (3 if d.flaps else 2)
    base = sum(items.values())
    items["leakage_protuberance_5pct"] = 0.05 * base
    return items


def exposed_fraction(pf: dict) -> float:
    """Mid wing: both surfaces of the centre section are inside the fuselage."""
    c_s = pf["cr"] + (pf["ct"] - pf["cr"]) * y_sob() / (pf["b"] / 2)
    return 1.0 - (pf["cr"] + c_s) * y_sob() / pf["S"]


def trimmed_polar(d: Design, aero: dict, pf: dict, lay: dict, cd_rest: float, cg: tuple) -> tuple:
    """Endurance-study trimmed polar (tail trim load with Cm0, CG and thrust-line moment, tail induced drag) with the
    mid-wing exposed fraction, the sweep-corrected span efficiency and the junction factor without root fairing."""
    x_cg, z_cg = cg
    mac = pf["mac"]
    k_w = 1.0 / (math.pi * pf["AR"] * aero["e_inv"])
    exp_frac = exposed_fraction(pf) * (1.0 if d.glove else Q_WING_NO_FILLET)
    arm_t = lay["x_ac_t"] - x_cg
    b_t = lay["tail_b_proj"]
    CLs = np.linspace(-0.2, 1.75, 79)
    out, cltS = [], []
    for CL in CLs:
        CLw = CL
        for _ in range(6):
            cdp = aero["cdp"](min(CLw, 1.9)) * EC.K_CD_TRIP * exp_frac
            cd_w = cdp + k_w * CLw ** 2
            cm = aero["cm0"] + CLw * (x_cg - lay["x_ac_w"]) / mac - (cd_rest + cd_w) * (Z_T - z_cg) / mac
            clt = cm * mac / arm_t
            CLw = CL - clt
        cdi_t = (clt * pf["S"]) ** 2 / (math.pi * b_t ** 2 * 0.8) / pf["S"]
        out.append(cd_rest + cd_w + cdi_t)
        cltS.append(clt)
    return CLs, np.array(out), np.array(cltS)


# =====================================================================================================================
# 6. masses (structural concept: chassis + shell; endurance-study wing/tail sizing with the identity changes)
# =====================================================================================================================
def wing_mass(d: Design, pf: dict, m0: float, n_lim: float, wsecs_oml: list) -> dict:
    """Endurance-study wing mass (CS-LUAS gust/manoeuvre limit x FoS 1.5, Schrenk lift, structlib beam loads, UD caps
    at the damage-tolerance strain, min-gauge webs, sandwich ribs and skins) with: spar length and bending along the
    swept spar (1/cos), skins on the exposed panels incl. the glove, spar-tunnel box covers inside the fuselage."""
    s = pf["b"] / 2
    ys = y_sob()
    y = np.linspace(0.0, s, 81)
    c = pf["cr"] + (pf["ct"] - pf["cr"]) * y / s
    tc = TC_ROOT * (1 + (d.ts_tip - 1) * np.clip((y - ys) / (s - ys), 0, 1))
    l = ST.schrenk(y, c, s)
    w = FOS * n_lim * m0 * G / 2 * l
    Vs, Ms = ST.beam_loads(y, w)
    k_sw = 1.0 / math.cos(math.radians(sweep_angles(d, pf)["spar"]))
    h_eff = np.maximum(0.95 * tc * c - 0.010, 0.012)
    A_cap = np.maximum(Ms * k_sw / (h_eff * EC.SIG_CAP), 40e-6)
    caps = 2 * 2 * np.trapz(A_cap, y) * UD["density"] * 1.10 * k_sw
    t_web = np.maximum(Vs / (h_eff * EC.TAU_WEB), 0.6e-3)
    web = 2 * np.trapz(t_web * h_eff, y) * PW["density"] * 1.25 * k_sw
    rear = 2 * np.trapz(0.6e-3 * 0.55 * tc * c + 2 * 20e-6 * UD["density"] / PW["density"], y) * PW["density"] * k_sw
    n_ribs = int(math.ceil((s - ys) / 0.35)) + 1
    rib_area = 0.68 * np.interp(np.linspace(ys, s, n_ribs), y, tc * c * c) * 0.6
    ribs = 2 * float(rib_area.sum()) * RIB_AREAL
    surf = oml.LiftingSurface(wsecs_oml, n_chord=60).mesh()
    c0 = wsecs_oml[0]["chord"]
    S_wet_half = surf.area() - 2 * 0.68 * TC_ROOT * wsecs_oml[0]["thickness_scale"] * c0 * c0 * 0.5
    skins = 2 * S_wet_half * SKIN_PRIMARY
    box_covers = 2 * (2 * ys) * 0.5 * c[0] * 3 * PLY_PW
    joints = 2 * 0.80
    hinges = 2 * 0.10
    sub = caps + web + rear + ribs + skins + box_covers + joints + hinges
    total = sub * 1.05
    return {"total": total, "caps": caps, "webs": web, "rear_spar": rear, "ribs": ribs, "skins": skins,
            "spar_tunnel_box_covers": box_covers, "joints": joints, "hinges": hinges, "S_wet": 2 * S_wet_half,
            "M_root_ult_Nm": float(Ms[0]), "A_cap_root_mm2": float(A_cap[0] * 1e6), "h_eff_root_m": float(h_eff[0]),
            "n_ult": FOS * n_lim, "spar_sweep_factor": k_sw}


def tail_mass(d: Design, tsecs: list, S_V: float, vent: dict | None) -> dict:
    t = EC.tail_mass(tsecs, S_V)
    out = {"v_panels": t["total"], "v_panels_S_wet": t["S_wet"]}
    if vent is not None:
        Sw = AL.wing_wetted(vent["S"], 0.12)
        out["ventral_fin"] = (Sw * SKIN_TAIL + vent["span"] * 0.10 + VENT_FITTINGS) * 1.05
        out["ventral_S_wet"] = Sw
    else:
        out["tail_bumper"] = 0.10
    out["total"] = out["v_panels"] + out.get("ventral_fin", 0.0) + out.get("tail_bumper", 0.0)
    return out


def chassis_items() -> dict:
    """Chassis (primary structure) items: (mass kg, x rule) - endurance-study list adapted to the mid wing."""
    L_keel = (X_HUB - 0.30) - (X_NG - 0.05)
    fp = fuselage_mesh_props()
    perim = fp["S_wet"] / L_FUS
    return {
        "keel_beams_2x_cfrp_hat_0p20kg_per_m": (2 * L_keel * 0.20, "keel"),
        "frames_11x_sandwich_bulkheads": (11 * 0.16 * perim / 1.42, "keel"),
        "spar_tunnel_and_wing_attach_fittings_7075_ti_pins": (1.50, "spar"),
        "main_gear_bow_clamps_7075": (0.60, "mg"),
        "nose_gear_trunnion_fitting": (0.30, X_NG),
        "engine_mount_frame_4130_truss": (0.60, "hub-0.30"),
        "parachute_attach_tray": (0.35, 0.5 * sum(X_CHUTE)),
        "floors_trays_equipment_rails": (1.20, 1.10),
        "hatch_frames_quick_release_fasteners": (0.90, 1.40),
        "fuel_bay_liner_supports": (0.40, "tank"),
        "tail_attach_fittings_v_panels": (0.30, "hub-0.30"),
        "chine_edge_bands": (2 * CHINE_BAND * (L_FUS - 0.6) if STYLE != "smooth" else 0.0, "keel"),
    }


def fuselage_mass() -> dict:
    fp = fuselage_mesh_props()
    shell = fp["S_wet"] * SKIN_SECONDARY
    chassis = sum(m for m, _ in chassis_items().values())
    return {"shell": shell, "chassis": chassis, "total": (shell + chassis) * 1.03, "S_wet": fp["S_wet"]}


SYS_FIXED = dict(EC.FIXED_ITEMS)


def fixed_items(d: Design) -> dict:
    out = {k: v for k, (v, _) in SYS_FIXED.items()}
    out["cooling_inlet_duct_" + d.inlet] = INLET[d.inlet][2]
    if d.flaps:
        out["flap_kit"] = EC.FLAP_KIT
    if EC.PROP_KEY != "0161":
        out["propeller_delta_3B"] = EC.PROP_MASS_DELTA
    return out


def fixed_mass(d: Design) -> float:
    return sum(fixed_items(d).values())


def gear_mass(d: Design, m0: float) -> float:
    m = EC.gear_mass(m0)
    if d.gear == "retract":
        m += RETRACT_DM
    elif d.gear == "fixed_bare":
        m -= EC.GEAR_FAIRINGS
    return m


# =====================================================================================================================
# 7. layout, CG, stability, ground geometry
# =====================================================================================================================
def loading_cases():
    return EC.loading_cases() + [{"name": "e180_growth_turret_full_fuel", "fuel": 1.0, "payload": "e180"},
                                 {"name": "e180_growth_turret_zero_fuel", "fuel": 0.0, "payload": "e180"}]


def payload_items(kind: str, x_bay: float) -> list:
    zt = turret_mount_z() - 0.06
    tur = [("payload_eo_ir_turret", M_TURRET, X_TURRET, zt)]
    base = tur + [("payload_mission_computer", M_MCOMP, x_bay - 0.12, -0.10), ("payload_tray", M_TRAY, x_bay, -0.18)]
    if kind == "turret_only":
        return tur
    if kind == "baseline_set":
        return base
    if kind == "e180":
        return [("payload_eo_ir_turret_E180", E180_M, X_TURRET, zt - 0.01)] + base[1:] + \
            [("payload_research_allowance", M_RESEARCH - (E180_M - M_TURRET), x_bay, -0.14)]
    return base + [("payload_research_allowance", M_RESEARCH, x_bay, -0.14)]


def component_list(d: Design, pf: dict, wref: dict, tail: dict, masses: dict, gearpos: dict, tank: dict) -> list:
    """(name, mass, x, z) of every item at MTOM, payload excluded (added per loading case)."""
    xw = wref["x_le_mac"] + 0.42 * wref["mac"]
    zw = Z_WING + 0.04
    x_spar = wref["x_le0"] + y_sob() * math.tan(math.radians(d.sweep_le)) + 0.30 * (
        pf["cr"] + (pf["ct"] - pf["cr"]) * y_sob() / (pf["b"] / 2))
    x_ail = wref["x_le_mac"] + 0.80 * wref["mac"] + 0.6 * (pf["b"] / 2 - wref["y_mac"]) * \
        math.tan(math.radians(d.sweep_le))
    tm = tail["macd"]
    fx = fixed_items(d)
    gm = masses["gear"]
    m_nose = 0.365 + 0.45 + 0.08 + 0.9 + (0.9 if d.gear == "retract" else 0.0)
    items = [
        ("wing_structure", masses["wing"], xw, zw),
        ("v_tail_structure", masses["tail_v"], tm["x_le_mac"] + 0.40 * tm["mac"], tm["z_mac"]),
        ("fuselage_shell", masses["fus_shell"], fuselage_mesh_props()["x_centroid"], 0.0),
        ("landing_gear_main", gm - m_nose, gearpos["x_mg"], gearpos["z_g"] + 0.15),
        ("landing_gear_nose", m_nose, X_NG, gearpos["z_g"] + 0.17),
        ("engine_group", fx["engine_group_installed"], X_HUB - 0.12, Z_T - 0.05),
        ("propeller", fx["propeller"], X_PROP, Z_T),
        ("spinner_hub_spacer", fx["spinner_hub_adapter_and_80mm_spacer"], X_PROP - 0.01, Z_T),
        ("baffles_ducts_firewall", fx["baffles_ducts_firewall_cowl_flap"], X_HUB - 0.15, Z_T - 0.03),
        ("cooling_inlet_duct", INLET[d.inlet][2], X_HUB - 0.40, Z_T + 0.10),
        ("fuel_system", fx["fuel_system_bladder_55L"], tank["x_c"], -0.05),
        ("actuators_wing_2", 2 * 0.27 + 0.15, x_ail, zw),
        ("actuators_tail_2", 2 * 0.27 + 0.05, tm["x_le_mac"] + 0.6 * tm["mac"], tm["z_mac"]),
        ("actuators_steer_brake_cowlflap", 2 * 0.27 + 0.10 + 0.15, (X_NG + gearpos["x_mg"]) / 2, -0.12),
        ("avionics", fx["avionics"], X_AVION, -0.03),
        ("electrical_battery_14S2P", 2.43, X_BATT, battery_z() + 0.04),
        ("electrical_pdu_converters", fx["electrical_power"] - 2.43, X_PDU, -0.13),
        ("wiring_harness", fx["wiring_harness"], 1.65, 0.0),
        ("parachute_fts", 5.9 + 0.15, 0.5 * sum(X_CHUTE), chute_top_z() - 0.06),
        ("lights", 0.25, xw, zw),
    ]
    if "ventral_fin" in masses:
        vf = tail["ventral"]
        items.append(("ventral_fin_skid", masses["ventral_fin"], vf["x_ac"], 0.5 * (vf["z_skid"] + vf["secs"][0]["z_le"])))
    if d.flaps:
        items.append(("flap_kit", EC.FLAP_KIT, wref["x_le0"] + y_sob() * math.tan(math.radians(d.sweep_le)) +
                      0.85 * pf["cr"], zw))
    if "propeller_delta_3B" in fx:
        items.append(("propeller_delta_3B", fx["propeller_delta_3B"], X_PROP, Z_T))
    for name, (m, x) in chassis_items().items():
        if isinstance(x, str):
            x = {"keel": 0.5 * (X_NG + X_HUB - 0.30), "spar": x_spar, "mg": gearpos["x_mg"], "tank": tank["x_c"]}.get(
                x, X_HUB - 0.30 if x.startswith("hub") else None)
        items.append(("chassis_" + name, m * 1.03, x, -0.02))
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


def tank_area(x: float, spar_x: tuple) -> float:
    """Usable bladder cross-section: OML inset by 30 mm; inside the spar-tunnel x range the tunnel band is excluded."""
    if spar_x[0] <= x <= spar_x[1]:
        return band_area(x, TANK_INSET, z_hi=SPAR_BAND[0]) + band_area(x, TANK_INSET, z_lo=SPAR_BAND[1])
    return band_area(x, TANK_INSET)


def tank_layout(fuel_kg: float, x_center: float, spar_x: tuple) -> dict:
    """Endurance-study rule: payload bay centred on ``x_center`` with an interconnected fuel cell ahead and behind,
    each holding half the fuel (fuel CG stays on the bay centroid)."""
    vol_req = fuel_kg / RHO_FUEL * 1.02
    xb0, xb1 = x_center - BAY_LEN / 2 - 0.015, x_center + BAY_LEN / 2 + 0.015

    def cell(x_start, sign):
        """Cell length found continuously (10 mm march + secant polish): no centroid jumps. The spar-tunnel band
        ends are inserted as break points so the cell volume is a continuous function of the cell length."""
        def vol(L):
            a_, b_ = sorted((x_start, x_start + sign * L))
            xs = list(np.linspace(a_, b_, 16))
            for xb in spar_x:
                if a_ < xb < b_:
                    xs += [xb - 1e-6, xb + 1e-6]
            xs = np.array(sorted(xs))
            A = np.array([tank_area(x, spar_x) for x in xs])
            return K_TANK * float(np.trapz(A, xs)), xs, A
        L, v_prev = 0.05, vol(0.05)[0]
        while v_prev < vol_req / 2 and L < 1.2:
            L += 0.01
            v_new = vol(L)[0]
            if v_new >= vol_req / 2:
                L = L - 0.01 + 0.01 * (vol_req / 2 - v_prev) / max(v_new - v_prev, 1e-12)
                break
            v_prev = v_new
        for _ in range(3):                                   # secant polish: cell volume = half the requirement
            v1 = vol(L)[0]
            v2 = vol(L + 0.002)[0]
            L += (vol_req / 2 - v1) * 0.002 / max(v2 - v1, 1e-12)
        v, xs, A = vol(L)
        return {"x0": min(x_start, x_start + sign * L), "x1": max(x_start, x_start + sign * L), "length": L,
                "volume_m3": v, "x_c": float(np.trapz(A * xs, xs) / np.trapz(A, xs))}
    fwd, aft = cell(xb0, -1.0), cell(xb1, +1.0)
    xc = (fwd["volume_m3"] * fwd["x_c"] + aft["volume_m3"] * aft["x_c"]) / (fwd["volume_m3"] + aft["volume_m3"])
    bay = (xb0 + 0.015, xb1 - 0.015)
    xs = np.linspace(*bay, 12)
    bay_vol = float(np.trapz([band_area(x, TANK_INSET, z_hi=SPAR_BAND[0] - 0.01) for x in xs], xs))
    deck_vol = float(np.trapz([band_area(x, TANK_INSET, z_lo=SPAR_BAND[1] + 0.01) for x in xs], xs))
    return {"x_c": xc, "z_c": -0.06, "cells": {"forward": fwd, "aft": aft}, "bay": bay, "bay_volume_m3": bay_vol,
            "dorsal_deck_volume_m3": deck_vol,
            "x0": fwd["x0"], "x1": aft["x1"], "volume_required_m3": vol_req,
            "volume_available_m3": fwd["volume_m3"] + aft["volume_m3"]}


THETA_TD_TARGET = EC.THETA_TD_TARGET
GEAR_TRAVEL = EC.GEAR_TRAVEL


def ground_geometry(d: Design, pf: dict, aero: dict, cg_cases: list, tail: dict) -> dict:
    """Endurance-study ground geometry (lift-off/touch-down attitudes, tip-back, nose load, turnover, propeller
    clearances) with the ventral skid of the Y-tail: the skid tip is the first contact aft (it guards the disc), so the
    flare-attitude rule checks the skid instead of the propeller; inverted V: the panel tips are checked."""
    xs = [c["x"] for c in cg_cases]
    zs = [c["z"] for c in cg_cases]
    x_aft, x_fwd = max(xs), min(xs)
    z_cg = max(zs)
    CLa, clmax, CL0 = aero["CL_alpha"], aero["CLmax_wing"], aero["CL_0"]
    th_lof = max(math.degrees((clmax / 1.1 ** 2 - CL0) / CLa), 0.0)
    th_td = max(math.degrees((clmax / 1.15 ** 2 - CL0) / CLa), 0.0)
    th_flare = max(th_td, 4.0)
    th_design = th_flare + 3.0
    z_tur_bot = float(z_bot(X_TURRET)[0]) - 0.03 - TUR_H_GROWTH * 0.45
    R = EC.D_PROP / 2
    vf = tail.get("ventral")
    tip_inv = None
    if d.tail == "invV":
        ts_ = tail["secs"][1]
        tip_inv = (ts_["x_le"] + 0.5 * ts_["chord"], ts_["z_le"] - 0.01)
    z_g = -0.45
    for _ in range(40):
        h_cg = z_cg - z_g
        tb = max(15.0, th_design + 2.0)
        x_mg = x_aft + h_cg * math.tan(math.radians(tb))
        arm = X_PROP - x_mg
        c_l, s_l = math.cos(math.radians(th_lof)), math.sin(math.radians(th_lof))
        c_d, s_d = math.cos(math.radians(th_design)), math.sin(math.radians(th_design))
        req = {"prop_level": Z_T - R - CLEAR_PROP_GROUND,
               "prop_liftoff": Z_T - R - (CLEAR_PROP_GROUND + arm * s_l) / c_l,
               "turret": z_tur_bot - 0.12,
               "gear_travel": float(z_bot(x_mg)[0]) - GEAR_TRAVEL}
        if vf is not None:
            req["skid_flare_design"] = vf["z_skid"] - (0.02 + (vf["x_skid"] - x_mg) * s_d) / c_d
        else:
            req["prop_flare_design"] = Z_T - R - (0.05 + arm * s_d) / c_d
        if tip_inv is not None:
            req["inverted_v_tips"] = tip_inv[1] - (0.10 + (tip_inv[0] - x_mg) * s_d) / c_d
        z_new = min(req.values())
        if abs(z_new - z_g) < 1e-6:
            break
        z_g = z_new
    active = min(req, key=req.get)
    h_cg = z_cg - z_g
    th_prop = math.degrees(math.atan2(Z_T - R - z_g, X_PROP - x_mg))
    clear_at = lambda th: (Z_T - R - z_g) * math.cos(math.radians(th)) - (X_PROP - x_mg) * math.sin(math.radians(th))
    if vf is not None:
        x_b, z_b = vf["x_skid"], vf["z_skid"]
    else:
        x_b, z_b = X_HUB - 0.22, float(z_bot(X_HUB - 0.22)[0]) - 0.03
    th_bump = math.degrees(math.atan2(z_b - z_g, x_b - x_mg))
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
            "prop_strike_deg": th_prop, "skid_contact_deg": th_bump, "x_skid": x_b, "z_skid": z_b,
            "skid_guards_prop": bool(th_bump < th_prop), "h_gear": max(h_gear, 0.05),
            "h_nose_leg": float(z_bot(X_NG)[0]) - z_g - WHEEL_D / 2, "turret_clear": z_tur_bot - z_g,
            "wheelbase": x_mg - X_NG, "active_height_rule": active, "height_rules_z_m": req,
            "fuselage_bottom_height_at_mg": float(z_bot(x_mg)[0]) - z_g}


# =====================================================================================================================
# 8. configuration solve (geometry <-> CG <-> stability <-> gear <-> masses <-> aero), endurance-study structure
# =====================================================================================================================
TAIL_SLOPE_OVERRIDE = None    # final design: lifting-line value from aero.surface_analysis on the V-tail sections


def chord_sob(pf: dict) -> float:
    return pf["cr"] + (pf["ct"] - pf["cr"]) * y_sob() / (pf["b"] / 2)


def spar_x_range(d: Design, pf: dict, x_w: float) -> tuple:
    c = chord_sob(pf)
    return (x_w + 0.08 * c, x_w + 0.62 * c)


def solve_configuration(d: Design, m0: float, fuel_kg_guess: float, V_ref: float, h_ref: float = H_LOITER,
                        verbose: bool = False):
    S = m0 * G / d.ws
    pf = EC.wing_planform(d, S)
    fp = fuselage_mesh_props()
    w_fus = 2 * y_sob()
    # wing incidence (endurance rule): fuselage level at the mid-loiter CL, capped by the 3 deg mains-first touch-down
    a0 = build_aero(d, pf, wing_sections(d, pf, 1.8, 0.0), V_ref, h_ref)
    q_floor = 0.5 * RHO0 * EC.V_LOIT_MIN_EAS ** 2
    CL_mid = 0.85 * m0 * G / (q_floor * S)
    i_level = math.degrees(a0["alpha_for"](min(CL_mid, 1.1)))
    i_td = math.degrees((a0["CLmax_wing"] / 1.15 ** 2 - a0["CL_0"]) / a0["CL_alpha"]) - THETA_TD_TARGET
    i_w = math.floor(min(i_level, i_td) * 4) / 4
    x_w = 1.70
    S_V, gam = 0.9, 40.0
    gearpos = {"x_mg": 2.35, "z_g": -0.47}
    tank = tank_layout(fuel_kg_guess, 2.1, spar_x_range(d, pf, x_w))
    vent = ventral_fin() if d.tail == "Y" else None
    aero = None
    x_bay = 2.1
    for outer in range(40):
        wsecs = wing_sections(d, pf, x_w, i_w)
        if aero is None:
            aero = build_aero(d, pf, wsecs, V_ref, h_ref)
        wref = wing_ref(d, pf, x_w)
        x_ac_w = wref["x_ac"]
        # ---- Y-tail sizing: V_H on the horizontal projection, Cn_beta target with the ventral fin contribution
        x_te_root = X_PROP - 0.15
        for _ in range(30):
            tsecs = tail_sections(d, S_V, gam, x_te_root, inverted=(d.tail == "invV"))
            tmac = EC.surface_mac(tsecs)
            x_ac_t = tmac["x_le_mac"] + 0.25 * tmac["mac"]
            l_h = x_ac_t - x_ac_w
            a_t = TAIL_SLOPE_OVERRIDE or EC.tail_slope(d, S_V, d.tail_le_sweep, h_ref, V_ref)
            S_h = d.VH * S * pf["mac"] / l_h
            cnb_fus = min(-1.3 * fp["volume"] / (S * pf["b"]) * (fp["h_max"] / fp["w_max"]),
                          SL.cn_beta_fuselage(0.0015, 1.75, fp["S_side"], L_FUS, S, pf["b"]))
            if vent is not None:
                l_v = vent["x_ac"] - x_ac_w
                cnb_vent = SL.cn_beta_vertical(vent["a"], vent["S"], l_v, S, pf["b"], 0.9, 0.0)
                vv_vent = vent["S"] * l_v / (S * pf["b"])
            else:
                cnb_vent, vv_vent = 0.0, 0.0
            S_v_req = (d.cnb_req - cnb_fus - cnb_vent) * S * pf["b"] / (0.9 * a_t * l_h * 1.1)
            S_v = max(S_v_req, (d.VV_min - vv_vent) * S * pf["b"] / l_h)
            S_new = S_h + S_v
            gam_new = math.degrees(math.atan(math.sqrt(S_v / S_h)))
            if abs(S_new - S_V) < 1e-5 and abs(gam_new - gam) < 1e-4:
                break
            S_V, gam = S_new, gam_new
        sweep_mt = math.degrees(math.atan(math.tan(math.radians(d.tail_le_sweep)) -
                                          (tsecs[0]["chord"] - tsecs[1]["chord"]) * 0.3 / tmac["panel_span"]))
        tail = {"secs": tsecs, "S_V": S_V, "gamma_deg": gam, "S_h_eff": S_h, "S_v_eff": S_v, "macd": tmac,
                "mac": tmac["mac"], "cr": tsecs[0]["chord"], "ct": tsecs[1]["chord"], "x_ac": x_ac_t, "a_t": a_t,
                "sweep_mt_deg": sweep_mt, "cnb_fus": cnb_fus, "cnb_vent": cnb_vent, "vv_vent": vv_vent,
                "b_proj": 2 * tsecs[1]["y"], "span_panel": tmac["panel_span"], "ventral": vent}
        # ---- masses at m0
        gl = EC.gust_limit(d.ws, aero["CL_alpha"], pf["mac"], aero["CLmax_wing"], 57.0)
        wmass = wing_mass(d, pf, m0, max(gl["n_limit"], N_POS), wing_sections(d, pf, x_w, i_w, analysis=False))
        tmass = tail_mass(d, tsecs, S_V, vent)
        fmass = fuselage_mass()
        masses = {"wing": wmass["total"], "tail_v": tmass["v_panels"], "fus_shell": fmass["shell"] * 1.03,
                  "gear": gear_mass(d, m0), "chassis": fmass["chassis"] * 1.03}
        if "ventral_fin" in tmass:
            masses["ventral_fin"] = tmass["ventral_fin"]
        else:
            masses["tail_bumper"] = tmass["tail_bumper"]
        items = component_list(d, pf, wref, tail, masses, gearpos, tank)
        cases = []
        for c in loading_cases():
            m, x, z = cg_of(items, fuel_kg_guess, tank, c["payload"], c["fuel"])
            cases.append({"name": c["name"], "m": m, "x": x, "z": z})
        # ---- neutral point (stablib): downwash with the quarter-chord sweep, fuselage Gilruth term
        deps = SL.downwash_gradient(d.AR, d.taper, l_h, tmac["z_mac"] - Z_WING, pf["b"], aero["sweep"]["c4"])
        kf = SL.kf_fuselage((wref["x_le0"] + 0.25 * pf["cr"]) / L_FUS)
        cmaf = SL.cm_alpha_fuselage(kf, w_fus, L_FUS, pf["mac"], S)
        x_np = SL.neutral_point(aero["CL_alpha"], x_ac_w, a_t, S_h, x_ac_t, S, pf["mac"], deps, 0.9, cmaf)
        sms = [SL.static_margin(x_np, c["x"], pf["mac"]) for c in cases]
        err = min(sms) - d.sm_min
        mw_share = (masses["wing"] + 2.6) / cases[0]["m"]
        dx = -err * pf["mac"] / (1.0 - mw_share)
        gearpos = ground_geometry(d, pf, aero, cases, tail)
        # payload bay + fuel cells follow the MTOM CG (damped), the wing follows the static-margin target
        dbay = cases[0]["x"] - x_bay
        x_bay += 0.7 * dbay
        tank = tank_layout(fuel_kg_guess, x_bay, spar_x_range(d, pf, x_w + dx))
        if verbose:
            print(f"  layout it{outer}: x_w={x_w:.4f} SMmin={min(sms):.4f} x_np={x_np:.4f} S_V={S_V:.4f} "
                  f"gam={gam:.2f} x_mg={gearpos['x_mg']:.3f} z_g={gearpos['z_g']:.3f} bay={x_bay:.4f}")
        if abs(dx) < 5e-5 and abs(dbay) < 5e-4 and outer > 3:
            break
        x_w += dx
    wref = wing_ref(d, pf, x_w)
    lay = {"x_w": x_w, "i_w": i_w, "x_ac_w": x_ac_w, "x_ac_t": x_ac_t, "l_h": l_h, "w_fus": w_fus,
           "tail_b_proj": tail["b_proj"], "x_np": x_np, "deps_da": deps, "cm_alpha_fus": cmaf, "kf": kf,
           "cases": cases, "sms": sms, "gear": gearpos, "tank": tank, "items": items, "spar_x": spar_x_range(d, pf, x_w),
           "incidence_rule": {"i_level_loiter_deg": i_level, "i_touchdown_deg": i_td},
           "tail": tail, "ventral": vent, "gust": gl, "wing_ref": wref}
    ac = EC.Aircraft(d=d, m0=m0, pf=pf, wing_secs=wing_sections(d, pf, x_w, i_w, analysis=False),
                     tail_secs=tail["secs"], sa=aero["sa"], S=S, cd_table=None, fit=None, clmax=None, geo={},
                     drag_items={}, stab={}, mass={"wing": wmass, "tail": tmass, "fus": fmass, "groups": masses},
                     layout=lay)
    ac.wing_secs_analysis = wing_sections(d, pf, x_w, i_w)
    return ac, aero


def finish_aero(ac, aero: dict, V_ref: float, h_ref: float, fuel_flow_ratio: float):
    d, pf, lay = ac.d, ac.pf, ac.layout
    items = drag_buildup(d, pf, lay, V_ref, h_ref, fuel_flow_ratio)
    cd_rest = sum(items.values()) / pf["S"]
    cg_d = (lay["cases"][0]["x"], lay["cases"][0]["z"])
    CLs, CDs, clt = trimmed_polar(d, aero, pf, lay, cd_rest, cg_d)
    cd0, k = EC.fit_polar(CLs, CDs)
    e = 1.0 / (math.pi * pf["AR"] * k)
    ld = CLs / CDs
    i = int(np.argmax(ld))
    end_par = np.clip(CLs, 0.0, None) ** 1.5 / CDs
    j = int(np.argmax(np.where(CLs > 0, end_par, 0)))
    x_fwd = min(c["x"] for c in lay["cases"])
    arm = lay["x_ac_t"] - x_fwd

    def trim(clw, dcm):
        cm = aero["cm0"] + dcm + clw * (x_fwd - lay["x_ac_w"]) / pf["mac"]
        return clw + cm * pf["mac"] / arm
    clmax = {"wing_clean": aero["CLmax_wing"], "clean_trimmed": trim(aero["CLmax_wing"], 0.0)}
    clmax["to_trimmed"] = clmax["ld_trimmed"] = clmax["clean_trimmed"]
    fe = EC.flap_effect(d, pf, lay["w_fus"], d.flap_to_deg) if d.flaps else None
    if fe:
        fe = dict(fe)
        fe["dCLmax"] *= math.cos(math.radians(aero["sweep"]["hinge"]))        # Raymer 12.21: x cos(hinge sweep)
        dcl0 = aero["CL_alpha"] * math.radians(-fe["dalpha0_deg"])
        clmax["to_trimmed"] = trim(aero["CLmax_wing"] + fe["dCLmax"], -0.25 * dcl0)
    ac.flap_to = fe
    ac.cd_table = (CLs, CDs)
    ac.fit = {"cd0": cd0, "k": k, "e": e, "LD_max": float(ld[i]), "CL_LDmax": float(CLs[i]),
              "CL_endurance": float(CLs[j]), "endurance_param_max": float(end_par[j]), "cd_rest": cd_rest,
              "cd0_to": cd0 + (fe["dCD0"] if fe else 0.0), "cd0_ld": cd0,
              "e_nita_scholz": AL.oswald_nita_scholz(pf["AR"], pf["taper"], aero["sweep"]["c4"], lay["w_fus"] / pf["b"]),
              "e_raymer": AL.oswald_straight(pf["AR"]), "cl_tail_S": (CLs, clt)}
    ac.clmax = clmax
    ac.drag_items = items
    ac.aero_cm0 = aero["cm0"]
    return ac


def empty_breakdown(ac) -> dict:
    g = ac.mass["groups"]
    airframe = sum(v for k, v in g.items())
    mf = fixed_mass(ac.d)
    items_sum = sum(m for n, m, _, _ in ac.layout["items"])
    return {"fixed": mf, "airframe": airframe, "growth": GROWTH * (mf + airframe),
            "empty": (mf + airframe) * (1 + GROWTH), "items_check": items_sum}


def airframe_fn_factory(ac):
    """m0 -> airframe mass fraction (incl. growth) at constant W/S, AR and layout (for sizinglib.MassModel)."""
    d = ac.d
    g = ac.mass["groups"]
    S0 = ac.S
    lay = ac.layout
    n_lim = max(lay["gust"]["n_limit"], N_POS)

    @functools.lru_cache(maxsize=256)
    def frac(m0_r: float) -> float:
        m0 = float(m0_r)
        S = m0 * G / d.ws
        pf = EC.wing_planform(d, S)
        wing = wing_mass(d, pf, m0, n_lim, wing_sections(d, pf, lay["x_w"], lay["i_w"], analysis=False))["total"]
        tail = (g["tail_v"] + g.get("ventral_fin", 0.0)) * (S / S0) ** 1.5 + g.get("tail_bumper", 0.0)
        af = wing + tail + g["fus_shell"] + g["chassis"] + gear_mass(d, m0)
        return af * (1 + GROWTH) / m0
    return lambda m0: frac(round(m0, 4))


def retract_plug(body: Body) -> float:
    """Fuselage plug length that gives back the bladder volume taken by the three gear wells."""
    A = band_area(BODY.x_hub - 1.6, TANK_INSET)
    return RETRACT_WELL_M3 / (K_TANK * A)


@functools.lru_cache(maxsize=64)
def fitted_body(body: Body, style: str) -> Body:
    return fit_body(equal_volume_body(body, style), style)


@functools.lru_cache(maxsize=64)
def body_volume(b: Body, style: str) -> float:
    set_body(b, style)
    return fuselage_mesh_props()["volume"]


def equal_volume_body(b: Body, style: str) -> Body:
    """Identity-price variants (smooth n = 2 or true-flat n = 1 sections) are compared at the same enclosed volume as
    the chined design (same control lines, cross-section scaled), then grown if a keep-out does not fit."""
    if style == "chined":
        return b
    V_ref = body_volume(b, "chined")

    def scaled(f):
        return replace(b, a_c=b.a_c * f, a_e=b.a_e * f, zt_c=b.zt_c * f, zb_c=b.zb_c * f, hump=b.hump * f,
                       keel_e=b.keel_e * f)
    lo, hi = 0.7, 1.4
    for _ in range(16):
        mid = 0.5 * (lo + hi)
        lo, hi = (lo, mid) if body_volume(scaled(mid), style) > V_ref else (mid, hi)
    return scaled(0.5 * (lo + hi))


def evaluate(d: Design, body: Body, mode: str = "mtom", endurance_target_h: float | None = None,
             verbose: bool = False) -> dict:
    """mode 'mtom': MTOM fixed (design), loiter time from the fuel left (endurance-study rule); mode 'endurance':
    loiter time from the endurance target, MTOW from sizinglib.MassModel (geometry re-sized at each MTOW)."""
    b = body
    if d.gear == "retract":
        set_body(fitted_body(body, d.body), d.body)
        b = replace(body, x_hub=body.x_hub + retract_plug(body))
    b = fitted_body(b, d.body)
    set_body(b, d.body)
    V_ref = EC.v_ref_loiter()
    m0 = d.mtom
    fuel_guess, ffr = 33.0, 0.33
    for it in range(14):
        ac, aero = solve_configuration(d, m0, fuel_guess, V_ref)
        finish_aero(ac, aero, V_ref, H_LOITER, ffr)
        eb = empty_breakdown(ac)
        empty = eb["empty"]
        if mode == "mtom":
            fuel = m0 - PAYLOAD - empty
            mis = EC.solve_loiter_for_fuel(ac, m0, fuel)
            m_new = m0
        else:
            mis = EC.solve_loiter_for_endurance(ac, m0, endurance_target_h * 3600.0)
            mm = SZ.MassModel(PAYLOAD, fixed_mass(d) * (1 + GROWTH), mis["ff"], airframe_fn_factory(ac))
            sol = mm.solve(m0_guess=m0)
            m_new, fuel = sol["mtow"], sol["fuel"]
        lo = [l for l in mis["log"] if l["kind"] == "loiter"]
        ff_loiter = np.mean([l["ff_kg_h"] for l in lo]) if lo else 2.5
        ffr_new = ff_loiter / (EC.BSFC_PTS[-1, 1] * EC.P_MAX / 1e6)
        done = abs(fuel - fuel_guess) < 0.05 and abs(ffr_new - ffr) < 0.01 and abs(m_new - m0) < 0.05
        if verbose:
            print(f"   eval it{it}: m0 {m0:.2f} fuel {fuel:.2f} ff {mis['ff']:.4f} t_air {mis['t_air_s'] / 3600:.2f} h")
        fuel_guess, ffr = fuel, ffr_new
        if mode != "mtom":
            m0 = m0 + 0.6 * (m_new - m0)                # damped MTOW update (the geometry re-sizes with m0)
        if done:
            break
    return {"ac": ac, "aero": aero, "mission": mis, "fuel": fuel, "m0": m0, "empty": empty, "eb": eb,
            "endurance_h": mis["t_air_s"] / 3600.0, "body": b, "feasible_fuel": bool(fuel > 0 and mis.get("feasible",
                                                                                                         True))}


# =====================================================================================================================
# 9. performance (endurance-study functions; take-off/landing with this layout's thrust line and wing height)
# =====================================================================================================================
def k_ground(ac) -> float:
    h = Z_WING - ac.layout["gear"]["z_g"]
    r = (16 * h / ac.pf["b"]) ** 2
    return ac.fit["k"] * r / (1 + r)


def takeoff(ac, m: float, h: float = 0.0) -> dict:
    """Endurance-study take-off model: lift-off at the lowest of the ground-attitude lift-off speed (incidence + take-off
    flap) and the speed at which the tail can rotate the aircraft about the main wheels against weight and the thrust
    line, but never below 1.1 VS_TO; Raymer energy-method ground roll; airborne distance to 15 m (Raymer 17.8.3)."""
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
        T = EC.PROP.wot(V, h)["T"]
        need = W * (g["x_mg"] - x_cg) + T * (Z_T - g["z_g"])
        if 0.5 * atm["rho"] * V * V * cap >= need:
            V_R = float(V)
            break
    V_lof = max(1.1 * VS, min(V_flat, V_R if V_R is not None else float("inf")))
    T0 = EC.PROP.wot(0.0, h)["T"]
    Tl = EC.PROP.wot(V_lof, h)["T"]
    r = AL.takeoff_ground_roll(W, ac.S, clmax, ac.fit["cd0_to"], k_ground(ac), T0, Tl, mu=0.04, rho=atm["rho"],
                               cl_roll=CLg, v_lof_factor=V_lof / VS)
    V_tr = max(1.15 * VS, V_lof)
    R = V_tr ** 2 / (0.2 * G)
    q = 0.5 * atm["rho"] * V_tr ** 2
    CL = W / (q * ac.S)
    D = q * ac.S * (float(ac.CD(min(CL, 1.7))) + dcd0)
    gam = math.asin(max(min((EC.PROP.wot(V_tr, h)["T"] - D) / W, 0.5), 0.01))
    h_tr = R * (1 - math.cos(gam))
    s_air = math.sqrt(R ** 2 - (R - 15.0) ** 2) if h_tr >= 15.0 else R * math.sin(gam) + (15.0 - h_tr) / math.tan(gam)
    return {"ground_roll_m": r["ground_roll"], "V_lof_m_s": V_lof, "VS_TO_m_s": VS, "V_flat_liftoff_m_s": V_flat,
            "V_rotation_authority_m_s": V_R, "T_static_N": T0, "T_lof_N": Tl, "air_distance_15m_m": s_air,
            "distance_15m_m": r["ground_roll"] + s_air, "climb_gradient": math.sin(gam), "CL_ground": CLg,
            "flap_deg": fe["delta_deg"] if fe else 0.0,
            "liftoff_mode": "flat (ground attitude)" if V_lof >= V_flat - 1e-6 else
                            ("rotated" if V_lof > 1.1 * VS + 1e-6 else "rotated at 1.1 VS")}


def landing(ac, m: float, h: float = 0.0) -> dict:
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


def performance(ac, m0: float, fuel: float) -> dict:
    W = m0 * G
    out = {}
    for h in (0.0, H_LOITER):
        atm = AL.isa(h)
        sp = AL.speeds(W, ac.S, ac.fit["cd0"], ac.fit["k"], atm["rho"])
        vmax = AL.max_level_speed(W, ac.S, ac.fit["cd0"], ac.fit["k"], EC.p_avail_fn(h), atm["rho"])
        roc, v_roc = AL.rate_of_climb(W, ac.S, ac.fit["cd0"], ac.fit["k"], EC.p_avail_fn(h), atm["rho"])
        cp = EC.climb_point(ac, W, h)
        bl = EC.best_loiter(ac, W, h)
        blu = EC.best_loiter(ac, W, h, floor=False)
        br = EC.best_range(ac, W, h)
        out[int(h)] = {"VS_clean_m_s": EC.vstall(ac, W, h), "V_min_power_polar_m_s": sp["V_min_power"],
                       "V_LDmax_polar_m_s": sp["V_ld_max"], "V_max_m_s": vmax, "RoC_max_m_s": roc,
                       "V_RoC_m_s": v_roc, "RoC_trimmed_prop_model_m_s": cp["roc"], "V_climb_m_s": cp["V"],
                       "climb_rpm": cp["rpm"], "climb_tip_mach": cp["tip_mach"],
                       "loiter": {k: bl[k] for k in ("V", "EAS", "CL", "LD", "P_shaft", "P_total", "eta", "rpm",
                                                      "power_fraction", "bsfc_g_kWh", "ff_kg_h", "gen_W", "tip_mach")},
                       "loiter_unconstrained_V_m_s": blu["V"], "loiter_unconstrained_ff_kg_h": blu["ff_kg_h"],
                       "best_range": {k: br[k] for k in ("V", "CL", "LD", "P_shaft", "eta", "rpm", "power_fraction",
                                                         "bsfc_g_kWh", "ff_kg_h")},
                       "power_available_W": EC.P_MAX * EC.lapse(atm["sigma"]),
                       "power_lapse_research": EC.lapse(atm["sigma"]),
                       "power_lapse_gagg_ferrar": AL.power_lapse(atm["sigma"])}
    out["ceiling_service_m"] = EC.ceiling(ac, m0)
    out["ceiling_absolute_m"] = EC.ceiling(ac, m0, 0.0)
    out["takeoff_sl_mtow"] = takeoff(ac, m0, 0.0)
    out["takeoff_1500m_isa_mtow"] = takeoff(ac, m0, 1500.0)
    m_land = m0 - fuel * 0.88
    out["landing_sl_mtow"] = landing(ac, m0)
    out["landing_sl_end_of_mission"] = landing(ac, m_land)
    out["landing_mass_end_of_mission_kg"] = m_land
    out["static_wot"] = EC.PROP.wot(0.0, 0.0)
    out["static_tip_speed_m_s"] = math.pi * D_PROP * out["static_wot"]["rpm"] / 60
    return out


# =====================================================================================================================
# 10. trade studies (parallel workers; every run is a full closed configuration solve + mission)
# =====================================================================================================================
SPAN_MAX, TIP_CHORD_MIN, FIELD_ROLL_MAX = EC.SPAN_MAX, EC.TIP_CHORD_MIN, EC.FIELD_ROLL_MAX
LB_MAX = rec("length_over_span_max", 0.65, "comparables.yaml#recommended_ranges.length_m.basis", basis=(
    "long-span endurance comparables 0.51-0.60, short-span tactical types 0.73-0.83; the chined UCAV forebody is "
    "accepted up to 0.65 (soft limit of this concept; the endurance study uses 0.60)"))


def ws_stall_corner(d: Design, body: Body) -> float:
    """Wing loading that puts the clean trimmed 1-g stall speed at 24 m/s (MTOM, sea level), endurance-study rule."""
    set_body(fitted_body(body, d.body), d.body)
    dd = replace(d, ws=450.0)
    ac, aero = solve_configuration(dd, dd.mtom, 34.0, EC.v_ref_loiter())
    finish_aero(ac, aero, EC.v_ref_loiter(), H_LOITER, 0.31)
    return round(0.5 * RHO0 * 24.0 ** 2 * ac.clmax["clean_trimmed"] * 0.99, 1)


def summarize(r: dict) -> dict:
    ac = r["ac"]
    lay, pf, g = ac.layout, ac.pf, ac.mass["groups"]
    to = takeoff(ac, r["m0"])
    ld = landing(ac, r["m0"] - r["fuel"] * 0.88)
    fm = ac.mass["fus"]
    return {"m0_kg": r["m0"], "empty_kg": r["empty"], "fuel_kg": r["fuel"], "endurance_h": r["endurance_h"],
            "AR": pf["AR"], "ws_Pa": ac.d.ws, "S_m2": ac.S, "span_m": pf["b"], "c_tip_m": pf["ct"],
            "length_m": SPINNER["x1"], "x_hub_m": X_HUB, "cd0": ac.fit["cd0"], "e": ac.fit["e"],
            "LD_max": ac.fit["LD_max"], "S_wet_fuselage_m2": fm["S_wet"], "S_V_m2": lay["tail"]["S_V"],
            "l_h_m": lay["l_h"], "wing_kg": g["wing"], "tail_kg": g["tail_v"] + g.get("ventral_fin", 0) +
            g.get("tail_bumper", 0), "fuselage_kg": g["fus_shell"] + g["chassis"], "gear_kg": g["gear"],
            "fixed_kg": fixed_mass(ac.d), "VS_m_s": EC.vstall(ac, r["m0"] * G, 0.0), "to_roll_m": to["ground_roll_m"],
            "to_15m_m": to["distance_15m_m"], "ldg_roll_m": ld["ground_roll_m"], "sm_min": min(lay["sms"]),
            "z_ground_m": lay["gear"]["z_g"], "gear_rule": lay["gear"]["active_height_rule"],
            "drag_items_D_over_q_m2": dict(ac.drag_items), "n_gust_limit": lay["gust"]["n_limit"]}


def _job(args):
    tag, d, body, mode, prop = args
    try:
        if prop:
            EC.set_prop(prop)
        r = evaluate(d, body, mode=mode, endurance_target_h=END_REQ_H if mode == "endurance" else None)
        out = summarize(r)
        if prop:
            ac = r["ac"]
            out["roc_sl_m_s"] = EC.climb_point(ac, r["m0"] * G, 0.0)["roc"]
            out["loiter_rpm_3000m"] = EC.best_loiter(ac, r["m0"] * G, H_LOITER)["rpm"]
    except Exception as exc:                       # a variant that cannot be closed is reported, not hidden
        out = {"error": repr(exc)}
    finally:
        if prop:
            EC.set_prop("0161")
    return tag, out


def _corner_job(args):
    AR, d, body = args
    return AR, ws_stall_corner(replace(d, AR=AR), body)


def run_parallel(fn, jobs, workers=4):
    import multiprocessing as mp
    with mp.get_context("fork").Pool(workers) as pool:
        return dict(pool.map(fn, jobs, chunksize=1))


def violations(sm: dict) -> list:
    bad = []
    if "error" in sm:
        return ["not closed: " + sm["error"]]
    if sm["span_m"] > SPAN_MAX + 1e-6:
        bad.append("span")
    if sm["c_tip_m"] < TIP_CHORD_MIN:
        bad.append("tip chord")
    if sm["VS_m_s"] > 24.0 + 1e-3:
        bad.append("stall")
    if sm["to_roll_m"] > FIELD_ROLL_MAX:
        bad.append("take-off roll")
    if sm["ldg_roll_m"] > FIELD_ROLL_MAX:
        bad.append("landing roll")
    return bad


def unswept_le(d: Design) -> float:
    """Leading-edge sweep that gives an unswept quarter chord (plain-reference wing)."""
    S = d.mtom * G / d.ws
    pf = EC.wing_planform(d, S)
    return math.degrees(math.atan(0.25 * (pf["cr"] - pf["ct"]) / (pf["b"] / 2)))


def identity_variants(d: Design) -> dict:
    """Each identity feature replaced by its plain alternative (one at a time), the all-plain reference, and the gear,
    tail and inlet alternatives of the brief."""
    plain_le = unswept_le(d)
    return {
        "smooth_elliptic_body_n2": replace(d, body="smooth"),
        "true_flat_upper_facets_n1": replace(d, body="flat"),
        "no_root_glove_fairings": replace(d, glove=False),
        "unswept_quarter_chord_wing": replace(d, sweep_le=plain_le, washout=2.5),
        "v_tail_with_bumper_no_ventral": replace(d, tail="V_bumper"),
        "inverted_v_tail": replace(d, tail="invV"),
        "pitot_scoop_inlet": replace(d, inlet="scoop"),
        "flush_naca_inlets": replace(d, inlet="naca"),
        "retractable_gear": replace(d, gear="retract"),
        "fixed_unfaired_gear": replace(d, gear="fixed_bare"),
        "plain_reference_all_off": replace(d, body="smooth", glove=False, sweep_le=plain_le, washout=2.5,
                                           tail="V_bumper", inlet="scoop"),
    }




# =====================================================================================================================
# 11. packaging and consistency checks
# =====================================================================================================================
def packaging_checks(ac) -> dict:
    out = {"keepouts": keepout_margins(BODY)}
    for k, v in out["keepouts"].items():
        if not v["fits"]:
            flag(f"keep-out {k} does not fit the OML ({v['min_margin_m'] * 1000:.0f} mm)")
    tk = ac.layout["tank"]
    out["fuel_cells"] = {"required_m3": tk["volume_required_m3"], "available_m3": tk["volume_available_m3"],
                         "forward_cell_x_m": [tk["cells"]["forward"]["x0"], tk["cells"]["forward"]["x1"]],
                         "aft_cell_x_m": [tk["cells"]["aft"]["x0"], tk["cells"]["aft"]["x1"]], "fuel_cg_x_m": tk["x_c"],
                         "note": "conformal ATL-type bladders in the OML inset by 30 mm, volume efficiency 0.85; the "
                                 "spar-tunnel band is excluded where the tunnel crosses a cell"}
    out["payload_bay"] = {"x_m": list(tk["bay"]), "volume_below_spar_tunnel_m3": tk["bay_volume_m3"],
                          "dorsal_deck_above_spar_tunnel_m3": tk["dorsal_deck_volume_m3"],
                          "note": "nadir hatch with sensor window below the spar tunnel (research sensors); the dorsal "
                                  "deck above the tunnel (top hatch) takes mission computer / recorder / radios; "
                                  "research payload allowance 16.75 kg"}
    x_free = (X_CHUTE[1] + 0.02, BODY.x_hub - 0.62 - 0.02)
    out["fuel_payload_block_fits"] = {"block_x_m": [tk["x0"], tk["x1"]], "free_x_m": list(x_free),
                                      "fits": bool(tk["x0"] >= x_free[0] and tk["x1"] <= x_free[1])}
    if not out["fuel_payload_block_fits"]["fits"]:
        flag("fuel cells + payload bay do not fit between the parachute bay and the aft equipment bay")
    out["turret_bay"] = {"x_m": X_TURRET, "mount_plane_z_m": turret_mount_z(),
                         "growth_envelope_m": [TUR_D_GROWTH, TUR_H_GROWTH],
                         "ball_below_keel_m": TUR_H_GROWTH - 0.09 - 0.055, "fits": all(
                             v["fits"] for k, v in out["keepouts"].items() if k.startswith("turret"))}
    out["cooling_exit_annulus_m2"] = math.pi * (BODY.r_exit ** 2 - (SPINNER["d"] / 2) ** 2)
    out["prop_to_cowl_lip_axial_m"] = X_PROP - 0.02 - X_LIP
    out["prop_to_tail_root_te_axial_m"] = X_PROP - (ac.tail_secs[0]["x_le"] + ac.tail_secs[0]["chord"])
    vf = ac.layout["ventral"]
    if vf is not None:
        out["prop_to_ventral_tip_te_axial_m"] = X_PROP - (vf["secs"][1]["x_le"] + vf["ct"])
    out["aft_closure"] = {"half_width_cylinders_m": BODY.a_e, "cowl_lip_radius_m": BODY.r_exit, "length_m": 0.10,
                          "mean_angle_deg": math.degrees(math.atan((BODY.a_e - BODY.r_exit) / 0.10))}
    if out["prop_to_cowl_lip_axial_m"] < EC.CLEAR_PROP_LONG:
        flag("propeller longitudinal clearance to the cowl lip below 13 mm (CS-VLA 925(c)(2))")
    return out


# =====================================================================================================================
# 12. outputs: 3-view sketch from the actual meshes, shaded perspective inset, constraint diagram
# =====================================================================================================================
def _silhouette(mesh, proj, buf=0.002):
    import shapely
    T = mesh.V[mesh.F]
    P2 = np.stack([proj(T[:, k, :]) for k in range(3)], axis=1)
    a = 0.5 * np.abs((P2[:, 1, 0] - P2[:, 0, 0]) * (P2[:, 2, 1] - P2[:, 0, 1]) -
                     (P2[:, 2, 0] - P2[:, 0, 0]) * (P2[:, 1, 1] - P2[:, 0, 1]))
    P2 = P2[a > 1e-10]
    polys = shapely.polygons(np.concatenate([P2, P2[:, :1]], axis=1))
    return shapely.union_all(polys).buffer(buf, join_style=2).buffer(-buf, join_style=2)


def _patch(ax, geom, fc, ec, lw=0.9, z=2, ls="-", alpha=1.0):
    from matplotlib.patches import PathPatch
    from matplotlib.path import Path as MPath
    for g in getattr(geom, "geoms", [geom]):
        if g.is_empty or g.geom_type != "Polygon":
            continue
        verts, codes = [], []
        for ring in [g.exterior, *g.interiors]:
            c = np.asarray(ring.coords)
            verts += c.tolist()
            codes += [MPath.MOVETO] + [MPath.LINETO] * (len(c) - 2) + [MPath.CLOSEPOLY]
        ax.add_patch(PathPatch(MPath(verts, codes), fc=fc, ec=ec, lw=lw, zorder=z, ls=ls, alpha=alpha,
                               joinstyle="round"))


def aircraft_meshes(ac) -> dict:
    """Meshes of the outer shape (starboard + mirrored), all from the same sections the analysis used."""
    from ucav250.core.geom import cylinder, sphere
    lay = ac.layout
    m = {"fuselage": FUSE.mesh(150, 72)}
    w = oml.LiftingSurface(ac.wing_secs, n_chord=40).mesh(refine=3)
    m["wing_r"], m["wing_l"] = w, w.mirrored_y()
    t = oml.LiftingSurface(ac.tail_secs, n_chord=32).mesh(refine=2)
    m["tail_r"], m["tail_l"] = t, t.mirrored_y()
    if lay["ventral"] is not None:
        m["ventral"] = oml.LiftingSurface(lay["ventral"]["secs"], n_chord=32).mesh(refine=2)
    zb = turret_mount_z() - 0.09 + 0.02
    m["turret"] = sphere(TUR_D / 2, (X_TURRET, 0.0, zb - TUR_D / 2 + 0.05), 28)
    m["spinner"] = cylinder(SPINNER["d"] / 2, (SPINNER["x0"], 0, Z_T), (SPINNER["x1"], 0, Z_T), 28, r1=0.006)
    return m


def _iso_render(ax, meshes: dict, az: float, el: float, colors: dict, light=(-0.45, -0.60, 0.65)):
    """Flat-shaded painter's-algorithm render (facets and chines read clearly)."""
    from matplotlib.collections import PolyCollection
    ca, sa_ = math.cos(math.radians(az)), math.sin(math.radians(az))
    ce, se = math.cos(math.radians(el)), math.sin(math.radians(el))
    Rz = np.array([[ca, -sa_, 0], [sa_, ca, 0], [0, 0, 1]])
    Rx = np.array([[1, 0, 0], [0, ce, -se], [0, se, ce]])
    R = Rx @ Rz
    Ld = np.array(light, float)
    Ld /= np.linalg.norm(Ld)
    tris, cols, depth = [], [], []
    for name, msh in meshes.items():
        T = msh.V[msh.F]
        n = np.cross(T[:, 1] - T[:, 0], T[:, 2] - T[:, 0])
        n /= np.maximum(np.linalg.norm(n, axis=1, keepdims=True), 1e-12)
        k = 0.50 + 0.50 * np.clip(n @ Ld, 0.0, 1.0)
        base = np.array(matplotlib_rgb(colors.get(name, "#cdd3da")))
        Tv = T @ R.T
        nv = n @ R.T
        vis = nv[:, 1] < 0.02                       # back-face culling (view direction +y in view space)
        tris.append(Tv[vis][:, :, [0, 2]])
        cols.append(np.clip(base[None, :] * k[vis, None], 0, 1))
        depth.append(Tv[vis][:, :, 1].mean(axis=1))
    tris, cols, depth = np.vstack(tris), np.vstack(cols), np.concatenate(depth)
    o = np.argsort(-depth)
    pc = PolyCollection(tris[o], facecolors=cols[o], edgecolors=cols[o], linewidths=0.15)
    ax.add_collection(pc)
    allp = tris.reshape(-1, 2)
    ax.set_xlim(allp[:, 0].min() - 0.1, allp[:, 0].max() + 0.1)
    ax.set_ylim(allp[:, 1].min() - 0.1, allp[:, 1].max() + 0.1)


def matplotlib_rgb(c):
    from matplotlib.colors import to_rgb
    return to_rgb(c)


def draw_sketch(ac, perf: dict, res: dict, path: Path):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.patches import Circle, Ellipse, Polygon, Rectangle

    lay, pf, d = ac.layout, ac.pf, ac.d
    g, t, vf = lay["gear"], lay["tail"], lay["ventral"]
    b2 = pf["b"] / 2
    L_tot = SPINNER["x1"]
    W_IN, H_IN = 18.0, 11.0
    sc = 1.22
    fig = plt.figure(figsize=(W_IN, H_IN), dpi=150)
    fill, skin, edge, dark, hid, acc = "#d5dbe2", "#e9edf1", "#1b2430", "#4b5563", "#2563eb", "#b45309"
    msh = aircraft_meshes(ac)

    def axes(x0, y0, wm, hm, xlim, ylim):
        ax = fig.add_axes([x0 / W_IN, y0 / H_IN, wm * sc / W_IN, hm * sc / H_IN])
        ax.set_xlim(*xlim)
        ax.set_ylim(*ylim)
        ax.set_aspect("equal")
        ax.axis("off")
        return ax

    def dim(ax, p0, p1, text, off=(0, 0), rot=0, fs=8.0, col=dark):
        ax.annotate("", xy=p1, xytext=p0, arrowprops=dict(arrowstyle="<->", color=col, lw=0.8, shrinkA=0, shrinkB=0))
        ax.text((p0[0] + p1[0]) / 2 + off[0], (p0[1] + p1[1]) / 2 + off[1], text, ha="center", va="center",
                fontsize=fs, color=col, rotation=rot, bbox=dict(fc="white", ec="none", pad=0.6), zorder=20)

    xs = np.linspace(0.0, L_FUS, 500)
    w, h, zc, nt, nb, tf = fus_section(xs)
    ztop, zbot = zc + tf * h, zc - (1 - tf) * h
    wsec = ac.wing_secs
    tsec = ac.tail_secs
    xcg, zcg = lay["cases"][0]["x"], lay["cases"][0]["z"]
    tk = lay["tank"]
    R = D_PROP / 2

    # ------------------------------------------------------------------ plan view (span horizontal, nose up)
    plan = lambda P: np.column_stack([P[:, 1], -P[:, 0]])
    axp = axes(0.30, 2.55, pf["b"] + 0.6, L_tot + 0.8, (-b2 - 0.3, b2 + 0.3), (-L_tot - 0.4, 0.4))
    for k_, fc_, z_ in (("ventral", fill, 1), ("wing_l", skin, 3), ("wing_r", skin, 3), ("fuselage", fill, 4),
                        ("tail_l", skin, 5), ("tail_r", skin, 5), ("spinner", "#9aa3ad", 4)):
        if k_ in msh:
            _patch(axp, _silhouette(msh[k_], plan), fc_, edge, lw=0.9, z=z_)
    axp.plot([0, 0], [-0.02, -(L_FUS - 0.02)], color=dark, lw=0.45, zorder=6)            # dorsal ridge line
    for xx in (X_CHUTE[0], X_CHUTE[1], X_AVION - 0.16, BODY.x_hub - 0.62):              # hatch / panel split lines
        wx = float(fus_section(xx)[0][0]) / 2 * 0.62
        axp.plot([-wx, wx], [-xx, -xx], color=dark, lw=0.45, zorder=6)
    # dorsal S-duct inlet (chevron lip) ahead of the engine hump
    xi = BODY.x_hub - 0.55
    axp.add_patch(Polygon(np.array([[-0.11, -xi], [0.0, -xi + 0.06], [0.11, -xi], [0.09, -xi - 0.10],
                                    [-0.09, -xi - 0.10]]), fc="#8a96a3", ec=edge, lw=0.6, zorder=7))
    # control surfaces: flaps (dashed), ailerons (solid), ruddervators
    ys = np.array([s_["y"] for s_ in wsec])
    xle = np.array([s_["x_le"] for s_ in wsec])
    ch = np.array([s_["chord"] for s_ in wsec])
    xte = xle + ch
    for sgn in (1, -1):
        for y0, y1, ls in ((0.57 * b2, 0.95 * b2, "-"), (y_sob() + 0.02, d.flap_span_frac * b2, "--")):
            yy = np.array([y0, y1])
            xh = np.interp(yy, ys, xte) - d.cf_c * np.interp(yy, ys, xte - np.interp(ys, ys, xle))
            xh = np.interp(yy, ys, xte) - d.cf_c * (pf["cr"] + (pf["ct"] - pf["cr"]) * yy / b2)
            xt_ = np.interp(yy, ys, xte)
            axp.plot(sgn * yy, -xh, color=dark, lw=0.6, ls=ls, zorder=6)
            axp.plot([sgn * y0, sgn * y0], [-xh[0], -xt_[0]], color=dark, lw=0.6, ls=ls, zorder=6)
            axp.plot([sgn * y1, sgn * y1], [-xh[1], -xt_[1]], color=dark, lw=0.6, ls=ls, zorder=6)
        yh = np.array([tsec[0]["y"], tsec[1]["y"]])
        xh = np.array([tsec[0]["x_le"] + 0.70 * tsec[0]["chord"], tsec[1]["x_le"] + 0.70 * tsec[1]["chord"]])
        axp.plot(sgn * yh, -xh, color=dark, lw=0.6, zorder=7)
        axp.add_patch(Rectangle((sgn * g["track"] / 2 - WHEEL_W / 2, -g["x_mg"] - WHEEL_D / 2), WHEEL_W, WHEEL_D,
                                fc="none", ec=dark, lw=0.7, ls=":", zorder=7))
    axp.add_patch(Rectangle((-WHEEL_W / 2, -X_NG - 0.05 - WHEEL_D / 2), WHEEL_W, WHEEL_D, fc="none", ec=dark, lw=0.7,
                            ls=":", zorder=7))
    axp.add_patch(Rectangle((-R, -X_PROP - 0.012), 2 * R, 0.024, fc="#6b7280", ec=edge, lw=0.6, zorder=3))
    axp.add_patch(Circle((0, -X_TURRET), TUR_D_GROWTH / 2, fc="none", ec=dark, lw=0.7, ls=":", zorder=7))
    axp.plot([0], [-xcg], marker="o", ms=7, mfc="white", mec=acc, mew=1.5, zorder=9)
    axp.plot([0], [-lay["x_np"]], marker="x", ms=7, color=acc, mew=1.5, zorder=9)
    axp.text(0.09, -xcg + 0.05, "AM", fontsize=7.5, color=acc, zorder=9)
    axp.text(0.09, -lay["x_np"] - 0.12, "NN", fontsize=7.5, color=acc, zorder=9)
    wr = lay["wing_ref"]
    axp.plot([wr["y_mac"]] * 2, [-wr["x_le_mac"], -(wr["x_le_mac"] + wr["mac"])], color=acc, lw=1.4, zorder=8)
    axp.text(wr["y_mac"] + 0.06, -wr["x_le_mac"] - wr["mac"] / 2, f"OAK {wr['mac']:.3f} m", fontsize=7.5, color=acc,
             va="center", zorder=9)
    dim(axp, (-b2, 0.24), (b2, 0.24), f"Kanat açıklığı b = {pf['b']:.2f} m")
    dim(axp, (-b2 - 0.15, 0.0), (-b2 - 0.15, -L_tot), f"Toplam uzunluk {L_tot:.2f} m", rot=90)
    yt = tsec[1]["y"]
    ytl = -(tsec[1]["x_le"] + tsec[1]["chord"]) - 0.22
    dim(axp, (-yt, ytl), (yt, ytl), f"V-kuyruk uçtan uca {2 * yt:.2f} m", fs=7.5)
    axp.text(b2 * 0.50, -0.30, f"S = {ac.S:.2f} m²   AR = {pf['AR']:.1f}   λ = {pf['taper']:.2f}   "
             f"Λ_HK = {d.sweep_le:.0f}°\nNLF(1)-0416 %16 → %13, {d.washout:.1f}° burulma, LERX kök eldiveni",
             fontsize=8, color=edge, ha="center")
    axp.text(-b2 * 0.50, -0.30, "kesik: kalkış flabı · sürekli: kanatçık / kuyruk dümeni\nnoktalı: tekerlek, "
             "taret yuvası (E180)", fontsize=7.2, color=dark, ha="center")
    axp.text(-b2 - 0.25, 0.36, "ÜSTTEN GÖRÜNÜŞ", fontsize=10, weight="bold", color=edge)

    # ------------------------------------------------------------------ side view (nose left)
    side = lambda P: P[:, [0, 2]]
    zmin = g["z_g"] - 0.14
    zmax = max(tsec[1]["z_le"], float(ztop.max())) + 0.16
    axs = axes(10.15, 4.55, L_tot + 0.65, zmax - zmin, (-0.3, L_tot + 0.35), (zmin, zmax))
    axs.plot([-0.25, L_tot + 0.3], [g["z_g"]] * 2, color=dark, lw=0.8)
    for xx in np.arange(-0.25, L_tot + 0.3, 0.12):
        axs.plot([xx, xx - 0.06], [g["z_g"], g["z_g"] - 0.05], color="#9ca3af", lw=0.5)
    for k_, fc_, z_ in (("tail_l", skin, 1), ("wing_l", skin, 2), ("fuselage", fill, 3), ("ventral", skin, 4),
                        ("tail_r", skin, 5), ("spinner", "#9aa3ad", 4)):
        if k_ in msh:
            _patch(axs, _silhouette(msh[k_], side), fc_, edge, lw=0.9, z=z_)
    axs.plot(xs, zc, color=dark, lw=0.55, zorder=6)                                        # chine line
    P0 = oml.LiftingSurface(wsec).section_points(0)
    axs.add_patch(Polygon(P0[:, [0, 2]], fc=skin, ec=edge, lw=0.8, zorder=6))               # wing root + glove
    axs.plot([tsec[0]["x_le"] + 0.7 * tsec[0]["chord"], tsec[1]["x_le"] + 0.7 * tsec[1]["chord"]],
             [tsec[0]["z_le"], tsec[1]["z_le"]], color=dark, lw=0.6, zorder=6)
    zi = float(z_top(xi)[0])
    axs.add_patch(Polygon(np.array([[xi - 0.10, zi - 0.004], [xi, zi + 0.045], [xi + 0.16, zi + 0.02],
                                    [xi + 0.30, zi - 0.004]]), fc="#8a96a3", ec=edge, lw=0.6, zorder=7))
    axs.add_patch(Rectangle((X_PROP - 0.012, Z_T - R), 0.024, 2 * R, fc="#6b7280", ec=edge, lw=0.6, zorder=2))
    _patch(axs, _silhouette(msh["turret"], side), "#b8c0ca", edge, lw=0.8, z=7)
    ztb = float(msh["turret"].V[:, 2].mean())
    axs.add_patch(Ellipse((X_TURRET - 0.045, ztb - 0.01), 0.03, 0.055, fc="#1e3a8a", ec="none", zorder=8))
    for xg, nose in ((X_NG, True), (g["x_mg"], False)):
        zf_ = float(z_bot(xg)[0])
        za = g["z_g"] + WHEEL_D / 2
        xa = xg + (0.05 if nose else 0.0)
        axs.add_patch(Polygon(np.array([[xg - 0.03, zf_ + 0.01], [xg + 0.03, zf_ + 0.01], [xa + 0.018, za + 0.04],
                                        [xa - 0.018, za + 0.04]]), fc="#9ca3af", ec=edge, lw=0.7, zorder=2))
        axs.add_patch(Circle((xa, za), WHEEL_D / 2, fc="#2b2f36", ec=edge, lw=0.8, zorder=8))
        axs.add_patch(Circle((xa, za), 0.04, fc="#9ca3af", ec=edge, lw=0.5, zorder=9))
        axs.add_patch(Polygon(np.array([[xa - 0.16, za + 0.02], [xa - 0.09, za + 0.11], [xa + 0.12, za + 0.10],
                                        [xa + 0.16, za + 0.03], [xa + 0.12, za - 0.01], [xa - 0.12, za - 0.01]]),
                              fc=skin, ec=edge, lw=0.8, zorder=10))
    boxes = [((X_BATT - 0.13, X_BATT + 0.13), (battery_z(), battery_z() + 0.08), "batarya"),
             ((X_AVION - 0.14, X_AVION + 0.14), (-0.09, 0.04), "aviyonik"),
             ((X_CHUTE[0], X_CHUTE[1]), (chute_top_z() - CHUTE_BOX[1], chute_top_z()), "paraşüt"),
             ((X_PDU - 0.12, X_PDU + 0.12), (-0.20, -0.07), "PDU"),
             ((tk["cells"]["forward"]["x0"], tk["cells"]["forward"]["x1"]), (-0.21, 0.15), "yakıt"),
             ((tk["bay"][0], tk["bay"][1]), (-0.22, SPAR_BAND[0] - 0.01), "faydalı\nyük"),
             ((tk["cells"]["aft"]["x0"], tk["cells"]["aft"]["x1"]), (-0.21, 0.15), "yakıt"),
             ((X_HUB - ENV_L_SG, X_HUB), (Z_T - (ENV_H - ENV_ZC), Z_T + ENV_ZC), "L 275 EF")]
    for (x0_, x1_), (z0_, z1_), lab in boxes:
        axs.add_patch(Rectangle((x0_, z0_), x1_ - x0_, z1_ - z0_, fc="none", ec=hid, lw=0.65, ls=(0, (3, 2)), zorder=11))
        axs.text(0.5 * (x0_ + x1_), 0.5 * (z0_ + z1_), lab, fontsize=5.0, color=hid, ha="center", va="center",
                 zorder=11)
    sx0, sx1 = lay["spar_x"]
    axs.add_patch(Rectangle((sx0, SPAR_BAND[0]), sx1 - sx0, SPAR_BAND[1] - SPAR_BAND[0], fc="none", ec=acc, lw=0.6,
                            ls=(0, (2, 2)), zorder=11))
    axs.plot([xcg], [zcg], marker="o", ms=7, mfc="white", mec=acc, mew=1.5, zorder=12)
    th = math.radians(g["skid_contact_deg"])
    x_end = g["x_skid"] + 0.25
    axs.plot([g["x_mg"], x_end], [g["z_g"], g["z_g"] + (x_end - g["x_mg"]) * math.tan(th)], color=acc, lw=0.6, ls="--",
             zorder=1)
    axs.text(g["x_mg"] + 0.12, g["z_g"] - 0.095, f"karın yüzgeci kızağı {g['skid_contact_deg']:.1f}° (pervane "
             f"{g['prop_strike_deg']:.1f}°)", fontsize=6.6, color=acc)
    dim(axs, (X_PROP + 0.20, g["z_g"]), (X_PROP + 0.20, Z_T - R), f"{g['prop_clear_level'] * 1000:.0f} mm",
        off=(0.15, 0), fs=7)
    ztop_all = zmax - 0.16
    dim(axs, (-0.18, g["z_g"]), (-0.18, ztop_all), f"Yükseklik {ztop_all - g['z_g']:.2f} m", rot=90, fs=7)
    axs.text(-0.25, zmax - 0.06, "YANDAN GÖRÜNÜŞ", fontsize=10, weight="bold", color=edge)
    axs.text(X_PROP + 0.06, Z_T + R - 0.04, f"Ø{D_PROP:.3f} m\nitici", fontsize=6.8, color=dark, ha="left", va="top")
    axs.annotate("S-kanallı soğutma girişi", xy=(xi, zi + 0.04), xytext=(xi - 0.75, zmax - 0.10), fontsize=6.5,
                 color=dark, ha="center", arrowprops=dict(arrowstyle="-", color=dark, lw=0.5))
    axs.annotate("çene EO/IR tareti", xy=(X_TURRET - 0.03, ztb - 0.06), xytext=(-0.22, g["z_g"] + 0.05),
                 fontsize=6.5, color=dark, ha="left", arrowprops=dict(arrowstyle="-", color=dark, lw=0.5))

    # ------------------------------------------------------------------ front view
    front = lambda P: P[:, [1, 2]]
    axf = axes(0.30, 0.35, pf["b"] + 0.6, zmax - zmin, (-b2 - 0.3, b2 + 0.3), (zmin, zmax))
    axf.plot([-b2 - 0.25, b2 + 0.25], [g["z_g"]] * 2, color=dark, lw=0.8)
    axf.add_patch(Circle((0, Z_T), R, fc="none", ec="#6b7280", lw=0.7, ls="--", zorder=1))
    for k_, fc_, z_ in (("tail_l", skin, 2), ("tail_r", skin, 2), ("ventral", skin, 2), ("wing_l", skin, 4),
                        ("wing_r", skin, 4), ("fuselage", fill, 5), ("turret", "#b8c0ca", 6)):
        if k_ in msh:
            _patch(axf, _silhouette(msh[k_], front), fc_, edge, lw=0.9, z=z_)
    ph = np.linspace(0, 2 * math.pi, 240)
    Pe = FUSE.point(np.full_like(ph, X_HUB - 0.12), ph)
    axf.plot(Pe[:, 1], Pe[:, 2], color=dark, lw=0.4, ls="--", zorder=6)                  # engine-bay section (hidden)
    Pf = FUSE.point(np.full_like(ph, 1.6), ph)
    axf.add_patch(Polygon(Pf[:, 1:], fc="#c9d0d8", ec=edge, lw=0.8, zorder=6))            # faceted forebody section
    xn = np.linspace(0.005, 1.6, 120)
    wn, hn, zcn, *_r, tfn = fus_section(xn)
    for sgn in (1, -1):                                                                     # chine lines -> nose tip
        axf.plot(sgn * wn / 2, zcn, color=dark, lw=0.6, zorder=7)
    axf.plot([0, 0], [float(zcn[0]), float((zcn + tfn * hn)[-1])], color=dark, lw=0.5, zorder=7)   # dorsal ridge
    axf.plot([0, 0], [float(zcn[0]), float((zcn - (1 - tfn) * hn)[-1])], color=dark, lw=0.5, zorder=7)  # keel line
    za = g["z_g"] + WHEEL_D / 2
    zf_ = float(z_bot(g["x_mg"])[0])
    for sgn in (1, -1):
        axf.add_patch(Polygon(np.array([[sgn * 0.07, zf_ + 0.01], [sgn * 0.16, zf_ + 0.01],
                                        [sgn * (g["track"] / 2 - 0.015), za + 0.03],
                                        [sgn * (g["track"] / 2 - 0.06), za]]), fc="#9ca3af", ec=edge, lw=0.7, zorder=3))
        axf.add_patch(Rectangle((sgn * g["track"] / 2 - WHEEL_W / 2, g["z_g"]), WHEEL_W, WHEEL_D * 0.6, fc="#2b2f36",
                                ec=edge, lw=0.7, zorder=7))
        axf.add_patch(Ellipse((sgn * g["track"] / 2, za + 0.02), 0.10, 0.18, fc=skin, ec=edge, lw=0.8, zorder=8))
    axf.add_patch(Rectangle((-0.018, za), 0.036, float(z_bot(X_NG)[0]) - za, fc="#9ca3af", ec=edge, lw=0.6, zorder=6))
    axf.add_patch(Rectangle((-WHEEL_W / 2, g["z_g"]), WHEEL_W, WHEEL_D * 0.6, fc="#2b2f36", ec=edge, lw=0.7, zorder=7))
    axf.add_patch(Ellipse((0, za + 0.02), 0.10, 0.18, fc=skin, ec=edge, lw=0.8, zorder=8))
    dim(axf, (-g["track"] / 2, g["z_g"] - 0.08), (g["track"] / 2, g["z_g"] - 0.08), f"İz {g['track']:.2f} m", fs=7)
    zdim = wsec[-1]["z_le"] + 0.25
    axf.annotate("", xy=(b2, zdim), xytext=(-b2, zdim), arrowprops=dict(arrowstyle="<->", color=dark, lw=0.8,
                                                                         shrinkA=0, shrinkB=0))
    axf.text(-b2 * 0.55, zdim, f"b = {pf['b']:.2f} m  (dihedral {d.dihedral:.1f}°)", ha="center", va="center",
             fontsize=8, color=dark, bbox=dict(fc="white", ec="none", pad=0.6), zorder=20)
    axf.text(b2 * 0.55, zdim, f"V-kuyruk {t['gamma_deg']:.0f}°, panel {t['span_panel']:.2f} m · karın yüzgeci",
             ha="center", va="center", fontsize=8, color=dark, bbox=dict(fc="white", ec="none", pad=0.6), zorder=20)
    axf.text(-b2 - 0.25, zmin + 0.03, "ÖNDEN GÖRÜNÜŞ", fontsize=10, weight="bold", color=edge)

    # ------------------------------------------------------------------ perspective inset (flat shaded)
    axi = fig.add_axes([10.05 / W_IN, 6.85 / H_IN, 7.7 / W_IN, 3.45 / H_IN])
    axi.set_aspect("equal")
    axi.axis("off")
    cols = {k_: ("#bfc7d0" if k_ == "fuselage" else "#d9dfe6") for k_ in msh}
    cols["turret"], cols["spinner"] = "#7d8794", "#8d97a3"
    _iso_render(axi, msh, az=22.0, el=26.0, colors=cols)
    axi.text(0.02, 1.015, "PERSPEKTİF (düz gölgeli: köşe çizgisi ve fasetler)", transform=axi.transAxes, fontsize=8.5,
             weight="bold", color=edge, va="bottom")

    # ------------------------------------------------------------------ title + data block
    fig.text(0.30 / W_IN, 10.55 / H_IN, "YK-250  —  Konsept “Kimlik” (UCAV görünümlü, itici, Y-kuyruk)", fontsize=16,
             weight="bold", color=edge)
    fig.text(0.30 / W_IN, 10.25 / H_IN, "Sivil EO/IR gözetleme ve araştırma platformu · silah/askı noktası/pilon yok · "
             "ilk boyutlandırma (calc.py) · tüm görünüşler aynı ölçekte · ölçüler m", fontsize=9.3, color=dark)
    P = perf
    loi = P[int(H_LOITER)]["loiter"]
    rows = [
        ("MTOM / boş / yakıt / faydalı yük", f"{res['m0']:.1f} / {res['empty']:.1f} / {res['fuel']:.1f} / {PAYLOAD:.0f} kg"),
        ("10 h görev için MTOW (MassModel)", f"{res['m10']:.1f} kg"),
        ("Kanat S, b, AR, W/S", f"{ac.S:.2f} m², {pf['b']:.2f} m, {pf['AR']:.0f}, {d.ws / G:.1f} kg/m²"),
        ("Gövde boy / genişlik / yükseklik", f"{L_FUS:.2f} / {fuselage_mesh_props()['w_max']:.2f} / "
                                             f"{fuselage_mesh_props()['h_max']:.2f} m"),
        ("CD0 / e / (L/D)maks", f"{ac.fit['cd0']:.4f} / {ac.fit['e']:.3f} / {ac.fit['LD_max']:.1f}"),
        ("VS temiz / kalkış flabı", f"{P[0]['VS_clean_m_s']:.1f} / {P['takeoff_sl_mtow']['VS_TO_m_s']:.1f} m/s"),
        ("Bekleme 3000 m (EAS/TAS, yakıt)", f"{loi['EAS']:.1f}/{loi['V']:.1f} m/s · {loi['ff_kg_h']:.2f} kg/h · "
                                            f"{loi['rpm']:.0f} dev/dk"),
        ("Vmaks DS / 3000 m", f"{P[0]['V_max_m_s']:.1f} / {P[int(H_LOITER)]['V_max_m_s']:.1f} m/s"),
        ("Dayanım (görev, %10 yedek)", f"{res['endurance_h']:.1f} h"),
        ("Menzil (feribot, %10 yedek)", f"{res['range_km']:.0f} km"),
        ("Tırmanma DS / 3000 m · tavan", f"{P[0]['RoC_max_m_s']:.2f} / {P[int(H_LOITER)]['RoC_max_m_s']:.2f} m/s · "
                                         f"{P['ceiling_service_m']:.0f} m"),
        ("Kalkış / iniş yerde koşu", f"{P['takeoff_sl_mtow']['ground_roll_m']:.0f} / "
                                     f"{P['landing_sl_end_of_mission']['ground_roll_m']:.0f} m"),
        ("Statik marj · V_H / V_V", f"{min(lay['sms']):.3f}–{max(lay['sms']):.3f} · {res['VH']:.3f} / {res['VV']:.4f}"),
        ("Kimlik bedeli (sade referansa göre)", res["price_text"]),
    ]
    y0 = 3.92
    fig.text(10.15 / W_IN, (y0 + 0.30) / H_IN, "ANA DEĞERLER", fontsize=10, weight="bold", color=edge)
    for i, (k_, v_) in enumerate(rows):
        yy = (y0 - 0.257 * i) / H_IN
        fig.text(10.15 / W_IN, yy, k_, fontsize=8.2, color=dark)
        fig.text(13.05 / W_IN, yy, v_, fontsize=8.2, color=edge, weight="bold")
    fig.text(10.15 / W_IN, 0.18 / H_IN, "AM: ağırlık merkezi · NN: nötr nokta · OAK: ortalama aerodinamik kiriş · mavi "
             "kesik: iç yerleşim · turuncu kesik: kanat kiriş tüneli", fontsize=7.0, color=dark)
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
    ax.axvline(cd["ws_stall_24"] / G, color="#111827", lw=1.2, ls="--", label="VS ≤ 24 m/s (temiz, trimli)")
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
    ax.set_title("YK-250 “Kimlik” – kısıt diyagramı (sizinglib)")
    ax.grid(alpha=0.3)
    ax.legend(fontsize=7.5, loc="upper left")
    fig.tight_layout()
    fig.savefig(path, facecolor="white")
    plt.close(fig)


# =====================================================================================================================
# 13. main
# =====================================================================================================================
TRADE_PROPULSION = {
    "pusher_direct_drive": {"selected": True, "reasons": [
        "nose and chin are free for the chined UCAV nose and the EO/IR turret field of view (11 of 13 comparables "
        "with a known layout are pushers, comparables.yaml)",
        "propeller wake stays off the NLF wing root and the turret; the V-tail and ventral fin sit ahead of the disc",
        "cost: installation factor 0.95 (baseline), ducted cooling needed (no prop wash over the cylinders), the "
        "cylinders are 0.05-0.18 m ahead of the prop flange -> steep cowl closure, propeller ground clearance sets "
        "the gear height"]},
    "tractor": {"selected": False, "reasons": [
        "the nose belongs to the propeller: no chined UCAV nose, the turret moves under the fuselage behind the nose "
        "gear and its forward view is blocked; the slipstream scrubs the fuselage and the wing root (NLF lost there)",
        "rejected for this concept: it contradicts the identity brief and the EO/IR field of view"]},
}


def main():
    global TAIL_SLOPE_OVERRIDE
    t0 = time.time()
    np.seterr(all="ignore")
    quick = "--quick" in sys.argv
    base = Design()
    body0 = Body()
    print("YK-250 concept 'identity' - closed-loop sizing (endurance-study equations)")
    trades = {"propulsion": TRADE_PROPULSION}
    if quick:
        x_hub, d = 3.95, replace(base, ws=440.0)
    else:
        corners = run_parallel(_corner_job, [(AR, base, body0) for AR in (12.0, 13.0, 14.0)])
        print(f"  stall-corner wing loadings (VS 24 m/s): {corners}  ({time.time() - t0:.0f} s)")
        # ---------------- trade 1: aft-body length (tail arm vs wetted area), AR 13 at the stall corner
        jobs = [(f"x_hub={xh:.2f}", replace(base, ws=corners[13.0]), replace(body0, x_hub=xh), "mtom", None)
                for xh in (3.75, 3.95, 4.15)]
        out = run_parallel(_job, jobs)
        rows = []
        for tag, sm in out.items():
            sm = dict(sm, violations=violations(sm))
            if "error" not in sm and sm["length_m"] > LB_MAX * sm["span_m"]:
                sm["violations"].append("L/b")
            rows.append(sm)
            if "error" not in sm:
                print(f"   {tag}: L {sm['length_m']:.2f} m, l_h {sm['l_h_m']:.2f} m, S_V {sm['S_V_m2']:.2f} m2, "
                      f"empty {sm['empty_kg']:.1f} kg, E {sm['endurance_h']:.2f} h {sm['violations']}")
        ok = [x for x in rows if not x["violations"]] or [min(rows, key=lambda x: len(x["violations"]))]
        x_hub = max(ok, key=lambda x: x["endurance_h"])["x_hub_m"]
        trades["aft_body_length"] = {"rows": rows, "chosen_x_hub_m": x_hub,
                                     "rule": "max endurance subject to L <= 0.65 b and the field/stall/geometry limits"}
        # ---------------- trade 2: aspect ratio x wing loading
        jobs = [(f"AR={AR:.0f},ws={ws:.0f}", replace(base, AR=AR, ws=ws), replace(body0, x_hub=x_hub), "mtom", None)
                for AR in (12.0, 13.0, 14.0) for ws in (410.0, corners[AR], 470.0)]
        out = run_parallel(_job, jobs)
        rows = []
        for tag, sm in out.items():
            sm = dict(sm, violations=violations(sm))
            rows.append(sm)
            if "error" not in sm:
                print(f"   {tag}: b {sm['span_m']:.2f} ct {sm['c_tip_m']:.3f} empty {sm['empty_kg']:.1f} "
                      f"L/D {sm['LD_max']:.2f} VS {sm['VS_m_s']:.2f} E {sm['endurance_h']:.2f} h TO {sm['to_roll_m']:.0f} "
                      f"LDG {sm['ldg_roll_m']:.0f} {sm['violations']}")
        ok = [x for x in rows if not x["violations"]]
        if not ok:
            flag("no aspect-ratio/wing-loading point met every hard limit; the least-violating point was taken")
            nmin = min(len(x["violations"]) for x in rows)
            ok = [x for x in rows if len(x["violations"]) == nmin]
        best = max(ok, key=lambda x: x["endurance_h"])
        d = replace(base, AR=best["AR"], ws=best["ws_Pa"])
        trades["aspect_ratio_wing_loading"] = {
            "rows": rows, "stall_corner_ws_Pa": corners, "chosen": {"AR": best["AR"], "ws_Pa": best["ws_Pa"]},
            "rule": "max endurance among points meeting span <= 6.8 m, tip chord >= 0.25 m, VS <= 24 m/s, field rolls "
                    "<= 200 m (endurance-study rule)"}
        print(f"   -> x_hub {x_hub:.2f} m, AR {d.AR:.0f}, W/S {d.ws:.0f} Pa  ({time.time() - t0:.0f} s)")
    body = replace(body0, x_hub=x_hub)

    # ---------------- final design point: lifting-line tail slope, fixed MTOM, 10 h closed loop
    print("\n[final] design point with the lifting-line V-tail slope")
    r0 = evaluate(d, body)
    sa_t = AE.surface_analysis(r0["ac"].tail_secs, EC.v_ref_loiter(), H_LOITER)
    TAIL_SLOPE_OVERRIDE = sa_t["CL_alpha"]
    res = evaluate(d, body, verbose=True)
    print("[10 h mission] closed-loop MTOW (sizinglib.MassModel, geometry re-sized each iteration)")
    r10 = evaluate(d, body, mode="endurance", endurance_target_h=END_REQ_H, verbose=True)
    if r10["m0"] > d.mtom + 0.05:
        if r10["m0"] >= MTOM_CAP:
            flag(f"the 10 h mission needs MTOW {r10['m0']:.1f} kg >= the {MTOM_CAP:.1f} kg M2 cap: INFEASIBLE")
        d = replace(d, mtom=min(math.ceil(r10["m0"] * 2) / 2, MTOM_CAP))
        flag(f"design MTOM raised to {d.mtom:.1f} kg so the 10 h mission closes")
    sm10 = summarize(r10)

    # ---------------- identity price, gear / tail / inlet / propeller trades (all closed solves)
    if not quick:
        print("\n[identity price + configuration trades] (parallel)")
        variants = identity_variants(d)
        jobs = [("design", d, body, "mtom", None), ("design_10h", d, body, "endurance", None)]
        jobs += [(k, v, body, "mtom", None) for k, v in variants.items()]
        jobs += [(k + "_10h", v, body, "endurance", None) for k, v in variants.items()]
        jobs += [("prop_31x12_3B", d, body, "mtom", "0164"), ("prop_32x18_2B", d, body, "mtom", "0161")]
        out = run_parallel(_job, jobs)
        ref, ref10 = out["design"], out["design_10h"]
        price = {}
        for k, v in variants.items():
            a, a10 = out[k], out[k + "_10h"]
            if "error" in a or "error" in a10:
                price[k] = {"error": a.get("error") or a10.get("error")}
                continue
            price[k] = {
                "endurance_at_design_mtom_h": a["endurance_h"], "delta_endurance_h": a["endurance_h"] - ref["endurance_h"],
                "mtow_for_10h_kg": a10["m0_kg"], "delta_mtow_for_10h_kg": a10["m0_kg"] - ref10["m0_kg"],
                "cd0": a["cd0"], "delta_cd0": a["cd0"] - ref["cd0"], "e": a["e"], "LD_max": a["LD_max"],
                "empty_kg": a["empty_kg"], "delta_empty_kg": a["empty_kg"] - ref["empty_kg"],
                "S_wet_fuselage_m2": a["S_wet_fuselage_m2"], "S_V_m2": a["S_V_m2"], "l_h_m": a["l_h_m"],
                "VS_m_s": a["VS_m_s"], "to_roll_m": a["to_roll_m"], "z_ground_m": a["z_ground_m"],
                "gear_height_rule": a["gear_rule"], "violations": violations(a)}
            print(f"   {k:32s} E {a['endurance_h']:5.2f} h ({price[k]['delta_endurance_h']:+5.2f})  MTOW10h "
                  f"{a10['m0_kg']:6.1f} ({price[k]['delta_mtow_for_10h_kg']:+5.1f})  CD0 {a['cd0']:.4f} "
                  f"({price[k]['delta_cd0']:+.4f})  empty {price[k]['delta_empty_kg']:+5.1f} kg {price[k]['violations']}")
        trades["identity_price"] = {
            "reference": {"design_endurance_h": ref["endurance_h"], "design_mtow_for_10h_kg": ref10["m0_kg"],
                          "design_cd0": ref["cd0"], "design_empty_kg": ref["empty_kg"]},
            "variants": price,
            "method": "one identity feature replaced by its plain alternative at a time (and all at once); every row "
                      "is a full closed solve: endurance at the design MTOM (fuel = MTOM - payload - empty) and MTOW "
                      "for the 10 h mission (sizinglib.MassModel); the same keep-out fit rules generate every body",
            "sign_convention": "delta_* = variant minus design: a positive delta_endurance_h (or a negative "
                               "delta_mtow_for_10h_kg) means the plain alternative is better, i.e. the identity feature "
                               "costs that much"}
        g_ = {k: price.get(k, {}) for k in ("retractable_gear", "fixed_unfaired_gear")}
        trades["landing_gear"] = {
            "fixed_faired_selected": {"endurance_h": ref["endurance_h"], "mtow_for_10h_kg": ref10["m0_kg"],
                                      "gear_D_over_q_m2": ref["drag_items_D_over_q_m2"]["landing_gear"],
                                      "gear_mass_kg": ref["gear_kg"]},
            "retractable": {**g_["retractable_gear"], "mass_items_kg": RETRACT, "mass_delta_kg": RETRACT_DM,
                            "well_volume_m3": RETRACT_WELL_M3, "fuselage_plug_m": None,
                            "extra_parts": ["3 retraction EMAs + controller", "3 down-locks + 2 up-locks with sensors",
                                            "5 doors with hinges and links", "gear sequencing in the flight control "
                                            "computer", "emergency free-fall / spring extension", "well close-outs and "
                                            "seals"],
                            "new_failure_modes": ["gear not extended -> belly landing on the chin turret and the "
                                                  "ventral fin", "door jammed open/closed", "lock sensor fault"],
                            "packaging": "main wells fall in the CG zone where the fuel cells and the payload bay sit; "
                                         "the volume is given back by a fuselage plug (modelled)",
                            "cost": "no purchasable 100-200 kg retract set (components.yaml); custom SAGITTA-type "
                                    "design + qualification"},
            "fixed_unfaired": g_["fixed_unfaired_gear"],
            "decision": "fixed tricycle with wheel spats and faired legs (numbers above)"}
        trades["tail_type"] = {
            "Y_selected": {"endurance_h": ref["endurance_h"], "S_V_m2": ref["S_V_m2"], "mtow_for_10h_kg": ref10["m0_kg"],
                           "notes": "canted V panels (ruddervators) + ventral fin: the fin tip skid touches before the "
                                    "propeller in any tail-down rotation (checked) and adds directional stability"},
            "V_with_bumper": price.get("v_tail_with_bumper_no_ventral", {}),
            "inverted_V": price.get("inverted_v_tail", {}),
            "conventional_or_T": "not modelled here: the endurance study shows T and Y within 0.7 h of the V; a "
                                 "horizontal stabiliser and a straight fin read as a light aircraft, not a UCAV "
                                 "(identity brief)"}
        trades["cooling_inlet"] = {"dorsal_sduct_selected": {"endurance_h": ref["endurance_h"], "inlet": INLET["sduct"]},
                                   "pitot_scoop": price.get("pitot_scoop_inlet", {}),
                                   "flush_naca": price.get("flush_naca_inlets", {})}
        p3, p2 = out["prop_31x12_3B"], out["prop_32x18_2B"]
        trades["propeller"] = {"mejzlik_32x18_2B": {k: p2.get(k) for k in ("endurance_h", "roc_sl_m_s", "to_roll_m",
                                                                         "loiter_rpm_3000m")},
                               "mejzlik_31x12_3B": {k: p3.get(k) for k in ("endurance_h", "roc_sl_m_s", "to_roll_m",
                                                                         "loiter_rpm_3000m")},
                               "decision": "32x18 2B (endurance); the 31x12 3B is the climb / hot-day option"}
    else:
        price = {}

    # ---------------- final design (re-sets the module state)
    res = evaluate(d, body)
    ac, aero, mis = res["ac"], res["aero"], res["mission"]
    m0, fuel = res["m0"], res["fuel"]
    lay = ac.layout
    mm = SZ.MassModel(PAYLOAD, fixed_mass(d) * (1 + GROWTH), mis["ff"], airframe_fn_factory(ac))
    mm_sol = mm.solve(m0_guess=120.0)
    rng = EC.solve_range(ac, m0, fuel)
    perf = performance(ac, m0, fuel)
    cdiag = EC.constraint_diagram(ac, m0, perf)
    VD = max(1.25 * EC.VC_EAS, perf[0]["V_max_m_s"], EC.BL["design_speeds_m_per_s_eas"]["VD"]["value"])
    vn = EC.vn_summary(ac, m0, VD)
    pk = packaging_checks(ac)
    if "retractable_gear" in price and "error" not in price["retractable_gear"]:
        trades["landing_gear"]["retractable"]["fuselage_plug_m"] = retract_plug(body)

    # stability details
    t = lay["tail"]
    vf = lay["ventral"]
    l_v = (vf["x_ac"] - lay["x_ac_w"]) if vf else 0.0
    tv = SL.tail_volumes(ac.S, ac.pf["mac"], ac.pf["b"], t["S_h_eff"], lay["l_h"],
                         t["S_v_eff"] + (vf["S"] * l_v / lay["l_h"] if vf else 0.0), lay["l_h"])
    fp = fuselage_mesh_props()
    cnb_fin = SL.cn_beta_vertical(TAIL_SLOPE_OVERRIDE, t["S_v_eff"], lay["l_h"], ac.S, ac.pf["b"], 0.9, 0.1)
    cnb_vent = t["cnb_vent"]
    cnb_fus_raymer = -1.3 * fp["volume"] / (ac.S * ac.pf["b"]) * (fp["h_max"] / fp["w_max"])
    cnb_fus_datcom = SL.cn_beta_fuselage(0.0015, 1.75, fp["S_side"], L_FUS, ac.S, ac.pf["b"])
    cnb_total = cnb_fin + cnb_vent + min(cnb_fus_raymer, cnb_fus_datcom)
    vproj = SL.v_tail_projection(t["S_V"], t["gamma_deg"])

    # sensitivities (endurance of the same aircraft)
    sens = {}
    for k_, scale in (("bsfc_minus_12pct", 0.88), ("bsfc_plus_12pct", 1.12)):
        EC.BSFC_SCALE = scale
        sens[k_] = EC.solve_loiter_for_fuel(ac, m0, fuel)["t_air_s"] / 3600
    EC.BSFC_SCALE = 1.0
    CLs, CDs = ac.cd_table
    ac.cd_table = (CLs, CDs + 0.1 * ac.fit["cd0"])
    sens["cd0_plus_10pct"] = EC.solve_loiter_for_fuel(ac, m0, fuel)["t_air_s"] / 3600
    ac.cd_table = (CLs, CDs)
    sens["empty_plus_5pct"] = EC.solve_loiter_for_fuel(ac, m0, fuel - 0.05 * res["empty"])["t_air_s"] / 3600
    sens["loiter_at_1000m"] = EC.solve_loiter_for_fuel(ac, m0, fuel, h=1000.0)["t_air_s"] / 3600
    sens["no_transit_loiter_only"] = EC.solve_loiter_for_fuel(ac, m0, fuel, R_transit=0.0)["t_air_s"] / 3600
    tkb = lay["tank"]
    aux_cap = K_TANK * tkb["bay_volume_m3"] * RHO_FUEL / 1.02
    aux_fuel = min(M_RESEARCH, aux_cap)
    sens["max_endurance_baseline_sensors_aux_bladder"] = EC.solve_loiter_for_fuel(ac, m0, fuel + aux_fuel)["t_air_s"] / 3600
    sens["aux_bladder_fuel_kg"] = aux_fuel

    # flags (honest open points of this concept)
    loi3, loi0 = perf[int(H_LOITER)]["loiter"], perf[0]["loiter"]
    g = lay["gear"]
    if loi0["rpm"] < 4000:
        flag(f"sea-level loiter at {loi0['rpm']:.0f} rpm is below the Limbach 4000-6000 rpm regular-flight band "
             f"(3000 m loiter: {loi3['rpm']:.0f} rpm); confirm low-load operation with Limbach")
    if perf[0]["RoC_max_m_s"] < ROC_REQ:
        flag(f"sea-level rate of climb {perf[0]['RoC_max_m_s']:.2f} m/s is below the 4.9 m/s baseline target with the "
             "32x18 2B propeller (prop-limited, as in the endurance study); the 31x12 3B option trades endurance for "
             "climb")
    for case, lp in (("3000 m", loi3), ("sea level", loi0)):
        if lp["gen_W"] < EC.P_ELEC * 1.2:
            flag(f"generator margin at the {case} loiter only {lp['gen_W'] / EC.P_ELEC:.2f}x the 303 W load")
    if min(loi0["power_fraction"], loi3["power_fraction"]) < EC.BSFC_PTS[0, 0]:
        flag(f"loiter power fraction {min(loi0['power_fraction'], loi3['power_fraction']):.3f} is below the lowest "
             "BSFC point (0.20): BSFC extrapolated (endurance-study rule); a light-load dyno map is needed")
    if res["empty"] / m0 > 0.62:
        flag(f"empty-mass fraction {res['empty'] / m0:.3f} (incl. 5 % growth) is above the comparables range 0.55-0.62")
    if not g["skid_guards_prop"]:
        flag("the ventral skid does not touch before the propeller in a tail-down rotation")
    if perf[0]["VS_clean_m_s"] > 24.0 + 1e-3:
        flag(f"clean stall speed {perf[0]['VS_clean_m_s']:.2f} m/s above the 24 m/s target")
    if cnb_total < d.cnb_req - 1e-3:
        flag(f"Cn_beta {cnb_total:.3f}/rad below the {d.cnb_req} target with the lifting-line tail slope")
    flag(f"aft cowl closure from the {2 * BODY.a_e:.2f} m cylinder bay to the {2 * BODY.r_exit:.2f} m cowl lip in 0.10 m "
         f"(mean {pk['aft_closure']['mean_angle_deg']:.0f} deg) relies on the pusher inflow and the annular cooling exit, "
         "as in the endurance study (no base drag booked); confirm with CFD; the 80 mm hub spacer needs Limbach approval")
    flag("chine vortex drag (5 % on the fuselage), the S-duct ram recovery (0.88) and the swept-wing corrections are "
         "estimates; confirm the faceted forebody in a wind-tunnel or CFD campaign (sideslip, chine vortex, "
         "turret wake)")
    flag("lifting line neglects sweep in the loading; quarter-chord sweep "
         f"{aero['sweep']['c4']:.1f} deg is at the 10 deg validity edge of aero.py: tip-stall margin (washout 3 deg, "
         f"stall onset eta {aero['stall_eta']:.2f}) must be confirmed with a vortex-lattice/CFD run")
    flag("flutter, wing torsion (swept-wing bending-torsion coupling) and aileron reversal not analysed "
         f"(CS-LUAS.629 clearance to 1.2 VD = {1.2 * VD:.1f} m/s)")
    to = perf["takeoff_sl_mtow"]
    if perf["landing_sl_mtow"]["ground_roll_m"] > FIELD_ROLL_MAX:
        flag(f"abort landing at MTOM needs a {perf['landing_sl_mtow']['ground_roll_m']:.0f} m ground roll (> 200 m "
             "field rule); end-of-mission landing is inside it; lift dumping with ailerons up is the mitigation "
             "(same finding as the endurance study)")
    if to["distance_15m_m"] > EC.RUNWAY_M:
        flag(f"take-off distance to 15 m {to['distance_15m_m']:.0f} m exceeds the {EC.RUNWAY_M:.0f} m strip")
    if to["V_rotation_authority_m_s"] is None or to["liftoff_mode"].startswith("flat"):
        flag("lift-off is flat (ground attitude + take-off flap): the high thrust line limits rotation authority; the "
             "take-off law must hold the nose wheel down until the wing lifts the aircraft (as in the endurance study)")

    # ---------------------------------------------------------------- console table
    P = perf
    L_tot = SPINNER["x1"]
    ref_plain = price.get("plain_reference_all_off", {})
    price_text = (f"{-ref_plain['delta_endurance_h']:+.1f} h / {-ref_plain['delta_mtow_for_10h_kg']:+.1f} kg (10 h MTOW)"
                  if ref_plain and "error" not in ref_plain else "-")
    rows = [
        ("MTOM design / cap", f"{m0:.1f} / {MTOM_CAP:.1f} kg (sizinglib.MassModel at this mission: {mm_sol['mtow']:.2f} kg)"),
        ("Empty / fuel / payload", f"{res['empty']:.1f} / {fuel:.1f} / {PAYLOAD:.1f} kg (empty frac {res['empty'] / m0:.3f})"),
        ("MTOW for the 10 h mission", f"{r10['m0']:.1f} kg (fuel {r10['fuel']:.1f} kg, empty {r10['empty']:.1f} kg)"),
        ("Wing S / b / AR / taper / sweep LE", f"{ac.S:.3f} m2 / {ac.pf['b']:.2f} m / {ac.pf['AR']:.1f} / "
                                               f"{ac.pf['taper']:.2f} / {d.sweep_le:.0f} deg (c/4 {aero['sweep']['c4']:.1f})"),
        ("Chords root(CL) / side of body / tip / MAC", f"{ac.pf['cr']:.3f} / {chord_sob(ac.pf):.3f} / {ac.pf['ct']:.3f} / "
                                                       f"{ac.pf['mac']:.3f} m"),
        ("W/S", f"{d.ws:.0f} Pa = {d.ws / G:.1f} kg/m2"),
        ("Fuselage L / overall / w / h", f"{L_FUS:.2f} / {L_tot:.2f} / {fp['w_max']:.2f} / {fp['h_max']:.2f} m "
                                         f"(L/b {L_tot / ac.pf['b']:.2f})"),
        ("V-tail S / dihedral / l_h; ventral fin S", f"{t['S_V']:.3f} m2 / {t['gamma_deg']:.1f} deg / {lay['l_h']:.2f} m; "
                                                     f"{vf['S']:.3f} m2"),
        ("V_H / V_V (incl. ventral)", f"{tv['V_H']:.3f} / {tv['V_V']:.4f}"),
        ("CD0 / e / k / (L/D)max", f"{ac.fit['cd0']:.4f} / {ac.fit['e']:.3f} / {ac.fit['k']:.4f} / {ac.fit['LD_max']:.2f}"),
        ("CL_alpha / e_inv / CLmax clean trimmed", f"{aero['CL_alpha']:.3f} /rad / {aero['e_inv']:.3f} / "
                                                   f"{ac.clmax['clean_trimmed']:.3f} (TO flap {ac.clmax['to_trimmed']:.3f})"),
        ("Stall onset eta / Cm0 (sections + sweep)", f"{aero['stall_eta']:.2f} / {aero['cm0']:.3f} "
                                                     f"({aero['cm0_sections']:.3f} {aero['dcm0_sweep_washout']:+.3f})"),
        ("VS clean / TO flap (SL, MTOM)", f"{P[0]['VS_clean_m_s']:.1f} / {P['takeoff_sl_mtow']['VS_TO_m_s']:.1f} m/s"),
        ("Loiter 3000 m (EAS/TAS, ff)", f"{loi3['EAS']:.1f} / {loi3['V']:.1f} m/s, {loi3['ff_kg_h']:.2f} kg/h, "
                                        f"{loi3['rpm']:.0f} rpm, PF {loi3['power_fraction']:.3f}"),
        ("V min power / V (L/D)max (SL, polar)", f"{P[0]['V_min_power_polar_m_s']:.1f} / {P[0]['V_LDmax_polar_m_s']:.1f} m/s"),
        ("Best-range speed 3000 m", f"{P[int(H_LOITER)]['best_range']['V']:.1f} m/s"),
        ("Vmax SL / 3000 m", f"{P[0]['V_max_m_s']:.1f} / {P[int(H_LOITER)]['V_max_m_s']:.1f} m/s"),
        ("Endurance (mission, 10 % reserve)", f"{res['endurance_h']:.2f} h (loiter {mis['t_loiter_s'] / 3600:.2f} h)"),
        ("Range (ferry, 3000 m, 10 % reserve)", f"{rng['range_m'] / 1000:.0f} km"),
        ("Endurance sensitivity", f"BSFC -12/+12 %: {sens['bsfc_minus_12pct']:.1f}/{sens['bsfc_plus_12pct']:.1f} h, "
                                  f"CD0 +10 %: {sens['cd0_plus_10pct']:.1f} h, empty +5 %: {sens['empty_plus_5pct']:.1f} h"),
        ("RoC SL / 3000 m", f"{P[0]['RoC_max_m_s']:.2f} / {P[int(H_LOITER)]['RoC_max_m_s']:.2f} m/s"),
        ("Ceiling service / absolute", f"{P['ceiling_service_m']:.0f} / {P['ceiling_absolute_m']:.0f} m"),
        ("TO roll / to 15 m (SL, MTOM)", f"{to['ground_roll_m']:.0f} / {to['distance_15m_m']:.0f} m ({to['liftoff_mode']})"),
        ("Landing roll end-of-mission / MTOM", f"{P['landing_sl_end_of_mission']['ground_roll_m']:.0f} / "
                                               f"{P['landing_sl_mtow']['ground_roll_m']:.0f} m"),
        ("Static margin (all cases)", f"{min(lay['sms']):.3f} - {max(lay['sms']):.3f}"),
        ("CG x (all cases) / NP x", f"{min(c['x'] for c in lay['cases']):.3f}-{max(c['x'] for c in lay['cases']):.3f} / "
                                    f"{lay['x_np']:.3f} m"),
        ("Cn_beta V-tail / ventral / fuselage / total", f"{cnb_fin:.3f} / {cnb_vent:.3f} / "
                                                        f"{min(cnb_fus_raymer, cnb_fus_datcom):.3f} / {cnb_total:.3f} /rad"),
        ("Prop clearance level / LOF; skid vs prop", f"{g['prop_clear_level']:.3f} / {g['prop_clear_lof']:.3f} m; "
                                                     f"{g['skid_contact_deg']:.1f} vs {g['prop_strike_deg']:.1f} deg"),
        ("Static tip Mach / speed", f"{P['static_wot']['tip_mach']:.3f} / {P['static_tip_speed_m_s']:.0f} m/s"),
        ("Gust limit n (VC 45, 4500 m)", f"{lay['gust']['n_limit']:.2f}, ultimate {FOS * lay['gust']['n_limit']:.2f}"),
        ("Identity price vs plain reference", price_text),
    ]
    print("\n" + "=" * 110)
    print("YK-250 concept 'identity' (UCAV-styled MALE) - results")
    print("=" * 110)
    for k_, v_ in rows:
        print(f"  {k_:44s} {v_}")
    print("  flags:")
    for f_ in FLAGS:
        print("   -", f_)

    # ---------------------------------------------------------------- concept.yaml
    wr = lay["wing_ref"]
    tm = t["macd"]
    y = {
        "meta": {"project": "YK-250 (ucav250)", "concept_key": "identity", "name_tr": "Kimlik (UCAV görünümlü MALE)",
                 "phase": "concept trade study - first-pass sizing", "date": "2026-10-05",
                 "generated_by": "ucav250/data/concepts/identity/calc.py",
                 "run": "PYTHONPATH=. python3 ucav250/data/concepts/identity/calc.py",
                 "frame": "X aft from the nose tip, Y starboard, Z up; Z = 0 on the centre-body chine line; SI units, "
                          "angles in degrees",
                 "scope": "civil EO/IR surveillance and research platform with a UCAV look; payload is sensors and "
                          "mission equipment only - no weapons, munitions, hardpoints, pylons or release mechanisms",
                 "method": "the reviewed endurance study (ucav250/data/concepts/endurance/calc.py) is imported as a "
                           "library: same propulsion, polars, lifting line, structures, mission, performance and "
                           "constraint equations; this file adds the identity geometry, drag items, chassis, ground "
                           "geometry, trades and outputs",
                 "libraries": ["analysis/aerolib.py", "analysis/stablib.py", "analysis/sizinglib.py",
                               "analysis/structlib.py", "analysis/aero.py", "design/oml.py",
                               "data/concepts/endurance/calc.py"],
                 "runtime_s": time.time() - t0},
        "configuration": {
            "propulsion": "single Limbach L 275 EF pusher, Mejzlik 32x18 2B on an 80 mm hub spacer; dorsal S-duct "
                          "cooling inlet with chevron lip and boundary-layer diverter, downdraft baffles, annular exit "
                          "around the spinner",
            "tail": f"Y-tail: canted V panels ({t['gamma_deg']:.0f} deg, NACA 0012, ruddervators) + ventral fin whose "
                    "tip skid guards the propeller disc",
            "wing_position": "mid wing on the chine line; two halves joined by a spar tongue/fork through a spar tunnel "
                             "in the fuselage; LERX-type root glove",
            "gear": "fixed tricycle: GFRP spring bow + TOST 200x50 wheels in spats, steerable faired nose leg",
            "fuselage": "chined, faceted superellipse sections (upper facets n 1.35, lower n 1.75, flat chin n 2.4), "
                        "pointed chined nose with pitot boom, chin EO/IR turret, dorsal engine hump, boxy round cowl",
            "planform": f"tapered (lambda {ac.pf['taper']:.2f}), LE sweep {d.sweep_le:.0f} deg (c/4 "
                        f"{aero['sweep']['c4']:.1f} deg), AR {ac.pf['AR']:.1f}, NLF(1)-0416 16 % -> 13 %, washout "
                        f"{d.washout:.1f} deg, dihedral {d.dihedral:.1f} deg, inboard take-off flaps",
            "layout": "payload bay on the CG under the spar tunnel, between two interconnected fuel cells; turret, "
                      "battery, avionics, PDU and parachute forward; engine aft",
            "transport_breakdown": "2 wing halves (spar tongue/fork, 2 main pins + drag pins), 2 V-tail panels, "
                                   "propeller; fuselage with ventral fin, nose leg and main bow in one crate",
        },
        "summary": {
            "mtow_kg": m0, "empty_kg": res["empty"], "fuel_kg": fuel, "payload_kg": PAYLOAD,
            "endurance_h": res["endurance_h"], "range_km": rng["range_m"] / 1000, "span_m": ac.pf["b"],
            "length_m": L_tot, "area_m2": ac.S, "AR": ac.pf["AR"], "ld_max": ac.fit["LD_max"],
            "mtow_for_10h_mission_kg": r10["m0"],
            "variant_10h_mission": {k: sm10[k] for k in ("m0_kg", "fuel_kg", "empty_kg", "S_m2", "span_m", "endurance_h")},
            "max_endurance_baseline_sensors_aux_bladder_h": sens["max_endurance_baseline_sensors_aux_bladder"],
            "identity_price_vs_plain_reference": ({
                "plain_reference": "equal-volume smooth n=2 body, no root gloves, unswept quarter-chord wing (washout "
                                   "2.5 deg), V-tail with tail bumper and no ventral fin, pitot scoop inlet; same "
                                   "gear, payload, engine and sizing rules",
                "endurance_cost_h": ref_plain["delta_endurance_h"],
                "mtow_for_10h_cost_kg": -ref_plain["delta_mtow_for_10h_kg"],
                "empty_mass_cost_kg": -ref_plain["delta_empty_kg"],
                "reference_endurance_h": ref_plain["endurance_at_design_mtom_h"],
                "reference_mtow_for_10h_kg": ref_plain["mtow_for_10h_kg"],
                "reference_cd0": ref_plain["cd0"],
                "sign": "cost = design penalty relative to the plain reference (positive = the identity design is worse)"}
                if ref_plain and "error" not in ref_plain else None),
            "feasible": bool(m0 < MTOM_CAP and r10["m0"] < MTOM_CAP and res["endurance_h"] >= END_REQ_H - 1e-3),
        },
        "inputs": INPUTS,
        "trades": trades,
        "geometry": {
            "wing": {"S_m2": ac.S, "b_m": ac.pf["b"], "AR": ac.pf["AR"], "taper": ac.pf["taper"],
                     "root_chord_centreline_m": ac.pf["cr"], "chord_side_of_body_m": chord_sob(ac.pf),
                     "tip_chord_m": ac.pf["ct"], "mac_m": wr["mac"], "mac_le_x_m": wr["x_le_mac"], "mac_y_m": wr["y_mac"],
                     "x_le_side_of_body_m": lay["x_w"], "y_side_of_body_m": y_sob(), "z_le_root_m": Z_WING,
                     "sweep_deg": aero["sweep"], "incidence_deg": lay["i_w"], "washout_deg": d.washout,
                     "dihedral_deg": d.dihedral, "incidence_rule": lay["incidence_rule"],
                     "sections_oml": ac.wing_secs, "sections_analysis": ac.wing_secs_analysis,
                     "airfoils": {"root": "nlf416 (NASA NLF(1)-0416)", "tip": "nlf416 thickness_scale 0.8125 (13 %)",
                                  "glove": "nlf416 scaled to the root absolute thickness"},
                     "root_glove": {"length_ahead_m": GLOVE_L, "span_m": GLOVE_DY, "te_fairing_m": FILLET_TE_L},
                     "spar_tunnel_x_m": list(lay["spar_x"]), "spar_tunnel_z_m": list(SPAR_BAND),
                     "flaps": {"type": "plain, inboard, take-off only", "span_eta": [y_sob() / (ac.pf["b"] / 2),
                                                                                    d.flap_span_frac],
                               "chord_fraction": d.cf_c, "takeoff_deg": d.flap_to_deg, "effect": ac.flap_to},
                     "ailerons": {"span_eta": [0.57, 0.95], "chord_fraction": d.cf_c},
                     "S_wet_m2": ac.mass["wing"]["S_wet"]},
            "tail": {"type": "Y (canted V + ventral fin)", "S_panels_total_m2": t["S_V"], "dihedral_deg": t["gamma_deg"],
                     "S_h_eff_m2": t["S_h_eff"], "S_v_eff_m2": t["S_v_eff"], "v_tail_projection_check": vproj,
                     "panel_span_m": t["span_panel"], "root_chord_m": t["cr"], "tip_chord_m": t["ct"],
                     "le_sweep_deg": d.tail_le_sweep, "aspect_ratio_unfolded": d.tail_AR, "mac_m": tm["mac"],
                     "mac_le_x_m": tm["x_le_mac"], "x_ac_m": lay["x_ac_t"], "airfoil": "n0012 (NACA 0012)",
                     "ruddervator_chord_fraction": 0.30, "sections": ac.tail_secs, "tip_to_tip_span_m": t["b_proj"],
                     "ventral_fin": {k: vf[k] for k in ("secs", "S", "span", "cr", "ct", "mac", "z_skid", "x_skid",
                                                        "x_ac", "a", "AR_geometric", "le_sweep_deg")} if vf else None},
            "fuselage": {"format": "oml.Fuselage stations [x, w, h, zc, n_top, n_bot, top_frac]", "stations": FUS,
                         "control_lines": asdict(BODY), "length_m": L_FUS, "width_max_m": fp["w_max"],
                         "height_max_m": fp["h_max"], "S_wet_m2": fp["S_wet"], "volume_m3": fp["volume"],
                         "side_area_m2": fp["S_side"], "A_max_m2": fp["A_max"], "style": BODY_STYLES["chined"]},
            "propulsion": {"engine": "Limbach L 275 EF", "prop_hub_face_x_m": X_HUB, "thrust_line_z_m": Z_T,
                           "hub_spacer_m": HUB_SPACER, "prop_plane_x_m": X_PROP, "cowl_lip_x_m": X_LIP,
                           "spinner": SPINNER, "propeller": {"model": "Mejzlik 32x18 2B (pusher hand)", "D_m": D_PROP,
                                                             "mass_kg": M_PROP},
                           "cooling_inlet": {"type": INLET[d.inlet][3], "x_m": X_HUB - 0.55,
                                             "ram_recovery": INLET[d.inlet][0]},
                           "cooling_exit_annulus_m2": pk["cooling_exit_annulus_m2"]},
            "landing_gear": {k: g[k] for k in ("x_mg", "z_g", "track", "wheelbase", "h_gear", "h_nose_leg",
                                               "tipback_deg", "turnover_deg", "nose_load_aft_cg", "nose_load_fwd_cg",
                                               "prop_clear_level", "prop_clear_lof", "prop_clear_design",
                                               "theta_lof_deg", "theta_td_deg", "theta_design_deg", "prop_strike_deg",
                                               "skid_contact_deg", "skid_guards_prop", "x_skid", "z_skid",
                                               "turret_clear", "fuselage_bottom_height_at_mg", "active_height_rule",
                                               "height_rules_z_m")},
            "bays": {"turret_chin_x_m": X_TURRET, "turret_mount_z_m": turret_mount_z(), "battery_x_m": X_BATT,
                     "avionics_x_m": X_AVION, "pdu_x_m": X_PDU, "parachute_x_m": list(X_CHUTE), "nose_gear_x_m": X_NG,
                     "payload_bay_x_m": list(lay["tank"]["bay"]), "fuel_cells": lay["tank"]["cells"],
                     "nose_cone": "heated pitot-static boom at the tip; no equipment ahead of x 0.42 m (slender nose)"},
            "overall": {"length_m": L_tot, "span_m": ac.pf["b"], "height_m": max(ac.tail_secs[1]["z_le"],
                                                                                 float(z_top(X_HUB - 0.3)[0])) - g["z_g"],
                        "ground_z_m": g["z_g"], "length_over_span": L_tot / ac.pf["b"]},
        },
        "aero": {"drag_items_D_over_q_m2": ac.drag_items, "cd_items": {k: v / ac.S for k, v in ac.drag_items.items()},
                 "cd_rest_non_wing": ac.fit["cd_rest"], "cd0": ac.fit["cd0"], "k": ac.fit["k"], "e": ac.fit["e"],
                 "e_inviscid_lifting_line_sweep_corrected": aero["e_inv"], "sweep_corrections": aero["k_sweep"],
                 "e_nita_scholz": ac.fit["e_nita_scholz"], "e_raymer_straight": ac.fit["e_raymer"],
                 "ld_max": ac.fit["LD_max"], "cl_ld_max": ac.fit["CL_LDmax"], "cl_best_endurance": ac.fit["CL_endurance"],
                 "CL_alpha_per_rad": aero["CL_alpha"], "CL_0_at_fuselage_level": aero["CL_0"],
                 "cm0_wing": aero["cm0"], "cm0_sections_tripped": aero["cm0_sections"],
                 "cm0_sweep_washout_basic_loading": aero["dcm0_sweep_washout"], "clmax": ac.clmax,
                 "alpha_stall_deg": aero["alpha_stall_deg"], "stall_onset_eta": aero["stall_eta"],
                 "Re_root": aero["Re_root"], "Re_tip": aero["Re_tip"], "neuralfoil_min_confidence": aero["min_conf"],
                 "polar_basis": "tripped section polars (x_tr 0.075 c) x 1.15 strip-integrated on the lifting-line cl "
                                "(mid-wing exposed share), + aerolib component build-up, trimmed (tail load, sweep-"
                                "washout Cm0, thrust-line moment), at the 3000 m loiter speed; parabolic fit CL 0.25-1.25",
                 "polar_table": {"CL": ac.cd_table[0], "CD": ac.cd_table[1]},
                 "clean_profile_drag_at_cl_0p9_upside": aero["cdp_clean"](0.9),
                 "tripped_profile_drag_at_cl_0p9": aero["cdp"](0.9) * EC.K_CD_TRIP,
                 "tail_CL_alpha_lifting_line_per_rad": sa_t["CL_alpha"]},
        "mass": {"mtow_kg": m0, "empty_kg": res["empty"], "fuel_kg": fuel, "payload_kg": PAYLOAD,
                 "fixed_equipment_kg": fixed_mass(d), "fixed_equipment_items_kg": fixed_items(d),
                 "airframe_kg": res["eb"]["airframe"], "growth_allowance_kg": res["eb"]["growth"],
                 "groups_kg": ac.mass["groups"], "wing_breakdown": ac.mass["wing"], "tail_breakdown": ac.mass["tail"],
                 "fuselage_breakdown": dict(ac.mass["fus"]),
                 "chassis_items_kg": {k: v[0] for k, v in chassis_items().items()},
                 "empty_fraction": res["empty"] / m0, "empty_fraction_without_growth": res["eb"]["empty"] / (1 + GROWTH) / m0,
                 "comparables_empty_fraction": EC.CP["recommended_ranges"]["empty_mass_fraction"],
                 "fuel_fraction": fuel / m0, "mission_fuel_fraction_sizinglib": mis["ff"],
                 "massmodel": {k: mm_sol[k] for k in ("mtow", "fuel", "airframe", "fixed", "payload", "empty")},
                 "items": [{"name": n, "mass_kg": mm_, "x_m": x_, "z_m": z_} for n, mm_, x_, z_ in lay["items"]],
                 "items_sum_check_kg": res["eb"]["items_check"],
                 "cases": lay["cases"], "mtow_margin_to_cap_kg": MTOM_CAP - m0,
                 "notes": ["groups_kg fus_shell and chassis include the 3 % bonding / insert / fastener allowance of the "
                           "endurance study (fuselage_breakdown shows them without it)",
                           "growth allowance 5 % of the computed empty mass (endurance study rule)"]},
        "stability": {"mac_m": ac.pf["mac"], "mac_le_x_m": wr["x_le_mac"], "x_ac_wing_m": lay["x_ac_w"],
                      "np_x_m": lay["x_np"], "cg_design_m": [lay["cases"][0]["x"], 0.0, lay["cases"][0]["z"]],
                      "cg_range_x_m": [min(c["x"] for c in lay["cases"]), max(c["x"] for c in lay["cases"])],
                      "static_margin_range": [min(lay["sms"]), max(lay["sms"])],
                      "static_margin_by_case": {c["name"]: sm_ for c, sm_ in zip(lay["cases"], lay["sms"])},
                      "V_H": tv["V_H"], "V_V_incl_ventral": tv["V_V"], "l_h_m": lay["l_h"], "deps_dalpha": lay["deps_da"],
                      "cm_alpha_fuselage_per_rad": lay["cm_alpha_fus"], "kf_fuselage_per_rad": lay["kf"],
                      "a_tail_per_rad": TAIL_SLOPE_OVERRIDE, "eta_tail": 0.9,
                      "cn_beta_v_tail_per_rad": cnb_fin, "cn_beta_ventral_per_rad": cnb_vent,
                      "cn_beta_fuselage_raymer_per_rad": cnb_fus_raymer, "cn_beta_fuselage_datcom_per_rad": cnb_fus_datcom,
                      "cn_beta_total_per_rad": cnb_total, "cn_beta_requirement_per_rad": d.cnb_req,
                      "note": "pusher propeller normal force neglected; DATCOM K_N 0.0015 / K_Rl 1.75 (endurance study); "
                              "the more destabilising fuselage value is used; ventral fin without sidewash credit"},
        "performance": {**{str(k): v for k, v in perf.items()}, "endurance_h": res["endurance_h"],
                        "loiter_time_h": mis["t_loiter_s"] / 3600, "range_km": rng["range_m"] / 1000,
                        "mission_log": mis["log"], "range_mission_fuel_fraction": rng["ff"], "VD_m_s": VD,
                        "VC_m_s": EC.VC_EAS},
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
                               "note": "endurance-study constraint_diagram (sizinglib curves, altitude curves corrected "
                                       "to sigma^1.23); the design W/S is set by the endurance trade at the stall limit. "
                                       "The 4.9 m/s climb curve uses the propeller efficiency at the climb speed on the "
                                       "full shaft power, so the design point sits on it; the trimmed propeller "
                                       "model of the performance section (pusher installation 0.95, generator load, "
                                       "cooling drag at climb power) gives the lower RoC reported there (flagged)"},
        "structures": {"vn": vn, "n_limit_design": lay["gust"]["n_limit"], "n_ultimate_wing": FOS * lay["gust"]["n_limit"],
                       "wing_root_bending_ultimate_N_m": ac.mass["wing"]["M_root_ult_Nm"],
                       "spar_cap_area_root_mm2": ac.mass["wing"]["A_cap_root_mm2"],
                       "spar_depth_root_m": ac.mass["wing"]["h_eff_root_m"], "spar_cap_allowable_Pa": EC.SIG_CAP,
                       "standards": "JARUS CS-LUAS / STANAG 4703: +3.8/-1.5 g, FoS 1.5, gust 15.24 m/s at VC"},
        "checks": pk,
        "sensitivities_endurance_h": sens,
        "electrical": {"load_W": EC.P_ELEC, "generator_W_loiter_3000m": loi3["gen_W"], "generator_W_loiter_sl": loi0["gen_W"],
                       "margin_3000m": loi3["gen_W"] / EC.P_ELEC, "margin_sl": loi0["gen_W"] / EC.P_ELEC},
        "flags": FLAGS,
        "files": ["ucav250/data/concepts/identity/calc.py", "ucav250/data/concepts/identity/concept.yaml",
                  "ucav250/data/concepts/identity/sketch.png", "ucav250/data/concepts/identity/constraint.png",
                  "ucav250/data/concepts/identity/notes_tr.md", "ucav250/data/concepts/identity/polars/"],
    }
    out = py(y)
    with open(HERE / "concept.yaml", "w", encoding="utf-8") as f:
        f.write("# YK-250 concept study 'identity' (UCAV-styled MALE) - generated by calc.py, do not edit by hand\n")
        yaml.safe_dump(out, f, sort_keys=False, allow_unicode=True, width=120)
    res_plot = {"m0": m0, "empty": res["empty"], "fuel": fuel, "endurance_h": res["endurance_h"],
                "range_km": rng["range_m"] / 1000, "VH": tv["V_H"], "VV": tv["V_V"], "m10": r10["m0"],
                "price_text": price_text}
    draw_sketch(ac, perf, res_plot, HERE / "sketch.png")
    draw_constraints(cdiag, HERE / "constraint.png")
    print(f"\nwrote {HERE / 'concept.yaml'}, sketch.png, constraint.png  ({time.time() - t0:.0f} s)")


if __name__ == "__main__":
    main()
