#!/usr/bin/env python3
"""YK-250 concept study "blended": blended wing-body / semi-tailless UCAV - feasibility and first-pass sizing.

Run from the repository root:

    PYTHONPATH=. python3 ucav250/data/concepts/blended/calc.py            (full study with trades)
    PYTHONPATH=. python3 ucav250/data/concepts/blended/calc.py --quick    (development: no trades)

Question answered here: can the most UCAV-like layout - a blended wing-body with a cranked-kite ("lambda") planform
and no horizontal tail - fly the YK-250 mission on one Limbach L 275 EF pusher (10 h endurance with 20 kg payload,
300 m runway, MTOM < 150 kg) with honest longitudinal and directional stability, a CG range that survives fuel burn
and payload changes, a pusher propeller that clears the ground, and a runway take-off/landing that works?

Method. The reviewed endurance study (ucav250/data/concepts/endurance/calc.py) is imported as a library, so every
concept is judged with the same equations: engine WOT torque x sigma^1.23 lapse, Mejzlik 32x18 2B table with
J-similarity, part-load BSFC at the actual power setting (iterated with the generator load), tripped section polars
x 1.15 (research sizing rule), CS-LUAS gust envelope and spar-cap sizing, the mission (sizinglib segments: warm-up,
take-off, climb, 2 x 100 km at best-range speed, loiter at minimum fuel flow above the 26 m/s EAS floor, descent,
10 % reserve, 2 % trapped fuel), level/climb/loiter/range points, ceiling and the constraint diagram.
What this file adds for a tailless aircraft:
  * a vortex-lattice model of the whole wing-body planform (the Glauert lifting line of aero.py neglects sweep, and
    sweep is what sets the neutral point and the trim of a tailless aircraft). It is validated against
    aero.lifting_line (straight wing) and AeroSandbox's VLM (swept wing) on every run;
  * trim by washout and elevons (thin-surface flap panels with a DATCOM-type effectiveness factor), the trimmed polar
    (induced drag from the trimmed span load, profile drag strip-integrated on the trimmed local cl), the trimmed
    CLmax (critical-section method with the research factors and the simple-sweep factor) at the forward CG;
  * directional stability without a tail (fins, body, swept-wing term), elevon trim and rotation authority;
  * the centre-body packaging (internal height map of the lofted OML for fuel, payload bay, turret, parachute and
    engine), the dorsal engine nacelle and the propeller ground clearance of a low flying wing;
  * the mass of a cranked swept wing-body (swept spar, carry-through, body shell, fins).
aero.surface_analysis is still run on the same sections (required reference): its CL_alpha, e and CLmax are written
next to the VLM values so the effect of sweep is visible.

Frame: X aft from the nose tip (apex of the centre body), Y starboard, Z up; Z = 0 on the centre-body root chord
line (datum). SI units, angles in degrees. Outputs (this folder only): concept.yaml, sketch.png, constraint.png,
polars/ (NeuralFoil polars missing from the shared cache; the shared ucav250/data/polars cache is read only).
Scope: civil EO/IR surveillance and research platform. No weapons, munitions, hardpoints, pylons or release mechanisms.
"""
from __future__ import annotations

import functools
import importlib.util
import json
import math
import sys
import time
from dataclasses import dataclass, replace
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
py = EC.py

# =====================================================================================================================
# 0. polar caches: the shared cache is read only; polars it does not hold are written to ./polars
# =====================================================================================================================
LOCAL_POLARS = HERE / "polars"
_shared_polar_path = AE._polar_path


def _polar_path(airfoil, Re, n_crit, ts=1.0):
    p = _shared_polar_path(airfoil, Re, n_crit, ts)
    if p.exists():
        return p
    LOCAL_POLARS.mkdir(exist_ok=True)
    return LOCAL_POLARS / p.name


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
    LOCAL_POLARS.mkdir(exist_ok=True)
    out = {"airfoil": airfoil, "thickness_scale": float(ts), "Re": float(Re), "n_crit": 9.0, "xtr": EC.XTR_TRIP, **r,
           "method": f"NeuralFoil xlarge, transition forced at x/c {EC.XTR_TRIP} on both surfaces"}
    path.write_text(json.dumps(out, indent=1))
    return r


EC._raw_tripped = _raw_tripped_cached


@functools.lru_cache(maxsize=None)
def tripped(airfoil: str, Re: float, ts: float) -> dict:
    return EC.tripped_chars(airfoil, float(f"{Re:.3g}"), ts)


@functools.lru_cache(maxsize=None)
def clean(airfoil: str, Re: float, ts: float) -> dict:
    return AE.characteristics(airfoil, float(f"{Re:.3g}"), 9.0, ts)


# =====================================================================================================================
# 1. inputs: research inputs shared with the endurance study + blended-specific values (source or basis each)
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
    "shear_web_allowable_ultimate_Pa", "areal_masses_kg_per_m2", "gear_fairings_kg", "main_wheel_tyre_m",
    "tank_volume_efficiency", "fuselage_bottom_static_height_min_m", "directional_stability_target_per_rad",
    "span_max_m", "tip_chord_min_m", "field_ground_roll_max_m",
]
INPUTS: dict = {k: {**EC.INPUTS[k], "shared_with": "endurance study (same research input / same rule)"}
                for k in SHARED_INPUT_KEYS if k in EC.INPUTS}
FLAGS: list = []


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


H_LOITER = EC.H_LOITER
V_FLOOR_EAS = EC.V_LOIT_MIN_EAS
PAYLOAD = EC.PAYLOAD
GROWTH = EC.GROWTH
MTOM_CAP = EC.MTOM_CAP
K_CD_TRIP = EC.K_CD_TRIP
K_CLMAX_SEC = EC.K_CLMAX_SEC
K_CLMAX_3D = EC.K_CLMAX_3D
FIELD_ROLL_MAX = EC.FIELD_ROLL_MAX
SPAN_MAX = EC.SPAN_MAX
TIP_CHORD_MIN = EC.TIP_CHORD_MIN
CNB_REQ = EC.INPUTS["directional_stability_target_per_rad"]["value"]
FOS = EC.FOS
N_POS = EC.N_POS
UD, PW = EC.UD, EC.PW
WHEEL_D, WHEEL_W = EC.WHEEL_D, EC.WHEEL_W
TUR_D, TUR_D_GROWTH, TUR_H_GROWTH = EC.TUR_D, EC.TUR_D_GROWTH, EC.TUR_H_GROWTH
M_TURRET, M_MCOMP, M_TRAY, M_RESEARCH = EC.M_TURRET, EC.M_MCOMP, EC.M_TRAY, EC.M_RESEARCH
ENV_L, ENV_L_SG, ENV_W, ENV_H, ENV_ZC = EC.ENV_L, EC.ENV_L_SG, EC.ENV_W, EC.ENV_H, EC.ENV_ZC
INTAKE_W = EC.INTAKE_W
CHUTE_BOX = EC.CHUTE_BOX
RHO_FUEL = EC.RHO_FUEL
K_TANK = EC.K_TANK
TYRE_V_LIMIT = EC.TYRE_V_LIMIT
CLEAR_PROP_GROUND = EC.CLEAR_PROP_GROUND
GEAR_TRAVEL = EC.GEAR_TRAVEL
HUB_SPACER = EC.HUB_SPACER

# ---- section choice: tailless trim needs a low pitching-moment section (numbers computed below, at Re 1.5e6) ----
AIRFOIL = "naca23015"
_c23 = tripped(AIRFOIL, 1.5e6, 1.0)
_cnlf = tripped("nlf416", 1.5e6, 1.0)
rec("wing_body_airfoil", {"name": "NACA 23015 (5-digit, 230 mean line)", "file": "ucav250/data/airfoils/naca23015.dat",
                          "cm0_tripped_Re1p5e6": _c23["cm0"], "cm0_nlf416_tripped_Re1p5e6": _cnlf["cm0"],
                          "cd_cl0p7_tripped_Re1p5e6": AE.section_cd(_c23, 0.7),
                          "cd_cl0p7_nlf416_tripped_Re1p5e6": AE.section_cd(_cnlf, 0.7),
                          "clmax_tripped_Re1p5e6": _c23["clmax"]},
    "aero.yaml#airfoils.naca23015 (UIUC coordinates, NeuralFoil polars; Report 824 stall note)",
    basis="a tailless aircraft trims its own section moment with washout/elevon: NACA 23015 cm_c/4 is about -0.007 "
          "against -0.09 for the research NLF(1)-0416, while the tripped drag (the sizing rule) is the same within "
          "about 1 %. The 230 mean line is the classic low-moment choice for swept tailless wings. Its abrupt "
          "leading-edge stall (NACA Report 824) is the price; it is flagged and must be handled by washout (stall "
          "onset inboard of the elevons), stall strips and an FCS angle-of-attack limiter")
TS_BODY = rec("thickness_scale_centre_body", 1.15, "", basis="NACA 23015 thickened to 17.3 % on the centre-body root "
              "chord: payload bay and fuel cells must sit on the CG, at about 55-65 % of the root chord where a 15 % "
              "section is too thin; derived section (thickness scaled about the camber line, same as aero.py)")
TS_ROOT = rec("thickness_scale_outer_root", 1.0, "", basis="15 % at the outer-panel root (spar depth for the swept "
              "spar and wing-root fuel cells)")
TS_TIP = rec("thickness_scale_tip", 0.85, "", basis="12.75 % at the tip (lighter tip, lower profile drag); NeuralFoil "
             "confidence is lower for this derived section at Re < 0.8e6 (reported per run)")
ELEVON_CF = rec("elevon_chord_fraction", 0.25, "baseline.yaml#subsystems.flight_control_actuators",
                basis="plain elevons on the outer panels, 25 % chord (same chord ratio as the research aileron/flap "
                      "screening); two surfaces per side so that one disconnected actuator leaves pitch and roll "
                      "control (CS-LUAS.629(f) free-floating-surface case is then a flutter, not a control, issue)")
ELEVON_MAX = rec("elevon_max_deflection_deg", 25.0, "", basis="mechanical stop of a plain flap with a Volz DA 26 "
                 "linkage (estimate)")
ELEVON_TRIM_MAX = rec("elevon_trim_limit_deg", 15.0, "", basis="largest steady trim deflection allowed so that at "
                      "least 10 deg remain for roll and pitch manoeuvring/gust control (estimate)")
FLAP_KPRIME = rec("plain_flap_effectiveness_K_prime", {"deflection_deg": [0.0, 10.0, 20.0, 30.0],
                                                       "K_prime": [1.0, 1.0, 0.80, 0.65]},
                  "", basis="thin-surface (vortex-lattice) flap lift times the DATCOM plain-flap non-linearity factor "
                            "K' (trend of DATCOM 6.1.1.1, cf/c 0.25); estimate")
rec("clmax_method_tailless", "critical section: wing CL at which the first strip reaches k_sec x cl_max(Re) x "
    "cos(Lambda_c/4 outer panel) on its angle-of-attack part of cl (elevon increment excluded: plain flap shifts cl "
    "and cl_max together), times k_clmax_3d; trimmed with the elevons at the forward CG", "baseline.yaml#aero."
    "corrections_for_sizing + Raymer, Aircraft Design 6th ed. eq. 12.15 (CLmax ~ 0.9 clmax cos Lambda)",
    basis="same research factors as the endurance study (0.94 x 0.95) plus the simple-sweep factor of the outer panel")
SM_MIN = rec("static_margin_min", 0.06, "", basis="stick-fixed minimum over all loading cases for an autopilot-flown "
             "tailless aircraft (>= 0.05 usually accepted) + 0.01; with a MAC of about 0.8 m one percent of MAC is "
             "8 mm, so 6 % keeps the CG about 50 mm ahead of the neutral point. The 10 % of the tailed concepts costs "
             "washout and trim drag here (trade 'static_margin')")
rec("fin_lift_slope_endplate_factor", 1.5, "", basis="fin effective aspect ratio = 1.5 x geometric (end-plate effect "
    "of the wing/body at the fin root; DATCOM 5.3.1.1 trend, Raymer 16.4); estimate")
rec("fin_section", "NACA 0012 (ucav250/data/airfoils/n0012.dat)", "baseline.yaml#aero.tail_airfoil")
X_TURRET = rec("turret_x_m", 0.36, "", basis="chin turret just behind the apex where the centre body is already 0.30 m "
               "deep; the ball protrudes below the lower surface (same exposed-height rule as the endurance study)")
X_BATT = rec("battery_pdu_x_m", 0.26, "", basis="14S2P LiFePO4 + PDU in the nose bay: the forward moment that balances "
             "the aft engine")
X_AVION = rec("avionics_x_m", 0.44, "", basis="autopilot, IMU, GNSS, transponder, datalink tray above the turret bay")
X_NG = rec("nose_gear_x_m", 0.62, "", basis="nose-leg trunnion on the frame behind the turret bay")
CHUTE_X = rec("parachute_bay_x_m", [0.64, 1.02], "", basis="Galaxy GRS 4/240 container (0.375 m long) under a top hatch "
              "in the deepest part of the centre body, ahead of the payload bay; riser to an attach fitting at the CG so "
              "that the aircraft hangs level; canopy leaves upward, far ahead of the pusher disc")
BAY_HALF_W = rec("payload_bay_half_width_m", 0.19, "", basis="0.38 m wide research payload bay (same width as the "
                 "endurance study) between the two centre-body fuel cells")
BAY_LEN = rec("payload_bay_length_m", 0.36, "", basis="research payload bay length centred on the design CG")
TANK_INSET = rec("tank_inset_m", 0.025, "", basis="skin + frame flange + bladder clearance per side")
SPAR_FWD, SPAR_AFT = 0.18, 0.66
rec("fuel_cell_chord_band", [SPAR_FWD, SPAR_AFT], "", basis="fuel cells between the front (18 % local chord) and rear "
    "(66 %) spars of the centre body and the outer-panel roots: ATL bladders in closed bays, no integral tank in CFRP")
rec("engine_installation_blended", "engine in a dorsal nacelle on the aft centre body: cylinders and SG750 in the "
    "nacelle, intake box below the crank axis reaching into the body (ventral blister 0.05 m deep), hub face 0.10 m "
    "behind the centre-body trailing edge, Mejzlik 32x18 2B on the 80 mm spacer", "endurance study installation",
    basis="the centre body is only 0.05-0.10 m thick at 90 % chord, so the 0.29 m engine cannot sit inside it; an "
          "extension shaft to bury it at the CG is rejected (torsional resonance risk with a simultaneous-firing "
          "twin, Limbach approval)")
BLISTER = 0.05
NACELLE_W = rec("nacelle_width_m", 0.43, "baseline.yaml#engine.envelope_m", basis="0.397 m across the spark-plug caps "
                "+ 2 x 0.015 m baffle/cowl clearance")
CL_DESIGN_RULE = rec("washout_design_point", "elevon neutral at the mid-loiter CL: 0.85 MTOM at the 26 m/s EAS floor, "
                     "design CG (MTOM case), thrust moment included", "", basis="loiter is 70-80 % of the flight time; "
                     "trimming it with washout instead of elevon keeps the elevon free and the trim drag lowest there")


# =====================================================================================================================
# 2. vortex-lattice model of the wing-body planform (sweep-aware; flat lattice, dihedral neglected)
# =====================================================================================================================
def _sec_interp(secs: list, y, key: str):
    ys = np.array([s["y"] for s in secs], float)
    v = np.array([float(s.get(key, 0.0)) for s in secs], float)
    return np.interp(np.abs(y), ys, v)


def vlm_lattice(secs: list, n_span: list, nc: int = 8, hinges: list = ()) -> dict:
    """Full-span horseshoe lattice on the planform of LiftingSurface sections (starboard, y ascending from the plane of
    symmetry). Spanwise: cosine spacing inside each section segment (n_span[k] panels); chordwise: nc uniform panels
    per strip, with a node on the hinge line of control-surface strips. Bound vortex at the panel quarter chord,
    control point at the three-quarter chord (Falkner/Weissinger rule). hinges: [{'name', 'y0', 'y1', 'xc'}] in m of
    |y| and chord fraction of the hinge line."""
    edges = [0.0]
    for k in range(len(secs) - 1):
        y0, y1 = secs[k]["y"], secs[k + 1]["y"]
        t = 0.5 * (1 - np.cos(np.linspace(0, math.pi, n_span[k] + 1)))
        edges += list(y0 + (y1 - y0) * t[1:])
    ys = np.array(edges)
    ye = np.concatenate([-ys[::-1], ys[1:]])
    ya, yb = ye[:-1], ye[1:]
    ym = 0.5 * (ya + yb)
    xa, xb_ = _sec_interp(secs, ya, "x_le"), _sec_interp(secs, yb, "x_le")
    ca, cb = _sec_interp(secs, ya, "chord"), _sec_interp(secs, yb, "chord")
    ns = len(ym)
    xh = np.full(ns, np.nan)
    hname = np.full(ns, -1)
    for ih, h in enumerate(hinges):
        m = (np.abs(ym) >= h["y0"]) & (np.abs(ym) <= h["y1"])
        xh[m] = h["xc"]
        hname[m] = ih
    rows = []
    for j in range(ns):
        if hname[j] >= 0:
            n2 = max(2, int(round(nc * (1 - xh[j]))))
            xi = np.r_[np.linspace(0.0, xh[j], nc - n2 + 1), np.linspace(xh[j], 1.0, n2 + 1)[1:]]
        else:
            xi = np.linspace(0.0, 1.0, nc + 1)
        xle_m, c_m = 0.5 * (xa[j] + xb_[j]), 0.5 * (ca[j] + cb[j])
        for i in range(nc):
            q = xi[i] + 0.25 * (xi[i + 1] - xi[i])
            r = xi[i] + 0.75 * (xi[i + 1] - xi[i])
            ax_, bx_ = xa[j] + q * ca[j], xb_[j] + q * cb[j]
            fl = hname[j] if (hname[j] >= 0 and xi[i] >= xh[j] - 1e-9) else -1
            rows.append((ax_, ya[j], bx_, yb[j], xle_m + r * c_m, ym[j], j, fl, 0.5 * (ax_ + bx_)))
    P = np.array(rows, float)
    return {"P": P, "ya": ya, "yb": yb, "ym": ym, "dy": yb - ya, "c": 0.5 * (ca + cb), "xle": 0.5 * (xa + xb_),
            "semispan": float(ys[-1]), "nc": nc, "strip": P[:, 6].astype(int), "flap": P[:, 7].astype(int),
            "xbound": P[:, 8], "n_strips": ns}


def _seg_w(px, py, x1, y1, x2, y2):
    """Normal (z) velocity at in-plane points P from straight vortex segments 1->2 of unit circulation (Biot-Savart,
    Katz & Plotkin eq. 10.115 for a planar lattice)."""
    r0x, r0y = x2 - x1, y2 - y1
    r1x, r1y = px - x1, py - y1
    r2x, r2y = px - x2, py - y2
    cz = r1x * r2y - r1y * r2x
    n1 = np.sqrt(r1x ** 2 + r1y ** 2)
    n2 = np.sqrt(r2x ** 2 + r2y ** 2)
    dot = (r0x * r1x + r0y * r1y) / np.maximum(n1, 1e-12) - (r0x * r2x + r0y * r2y) / np.maximum(n2, 1e-12)
    small = np.abs(cz) < 1e-12
    return np.where(small, 0.0, dot / (4 * math.pi * np.where(small, 1.0, cz)))


def vlm_aic(L: dict) -> np.ndarray:
    P = L["P"]
    px, py_ = P[:, 4][:, None], P[:, 5][:, None]
    ax_, ay, bx_, by = P[:, 0][None, :], P[:, 1][None, :], P[:, 2][None, :], P[:, 3][None, :]
    far = 1.0e4
    return (_seg_w(px, py_, ax_ + far, ay, ax_, ay) + _seg_w(px, py_, ax_, ay, bx_, by)
            + _seg_w(px, py_, bx_, by, bx_ + far, by))


def vlm_trefftz(L: dict, Gs: np.ndarray, S: float) -> float:
    """Induced drag coefficient of the strip circulation Gs (V = 1): Gamma(y) piecewise linear through the strip centres
    and zero at the tips, i.e. a wake sheet of piecewise-constant strength whose Trefftz-plane downwash is closed form
    (logarithm); the drag integral -1/S * int Gamma w dy by 8-point Gauss-Legendre per interval."""
    s = L["semispan"]
    yk = np.r_[-s, L["ym"], s]
    gk = np.r_[0.0, Gs, 0.0]
    a, b = yk[:-1], yk[1:]
    sig = -(gk[1:] - gk[:-1]) / (b - a)
    xg, wg = np.polynomial.legendre.leggauss(8)
    yq = (0.5 * (a + b))[:, None] + (0.5 * (b - a))[:, None] * xg[None, :]
    wq = (0.5 * (b - a))[:, None] * wg[None, :]
    gq = gk[:-1][:, None] + (gk[1:] - gk[:-1])[:, None] * (yq - a[:, None]) / (b - a)[:, None]
    Y = yq.ravel()
    w = (sig[None, :] * np.log(np.abs((Y[:, None] - a[None, :]) / (Y[:, None] - b[None, :])))).sum(1) / (2 * math.pi)
    return float(-np.sum(gq.ravel() * w * wq.ravel()) / S)


def vlm_validation() -> dict:
    """Run on every execution: (1) straight tapered AR 12 wing against aero.lifting_line (Glauert), (2) 30 deg swept
    AR 8.6 wing against AeroSandbox's VLM (independent code) when available, (3) elliptic planform span efficiency."""
    out = {}
    b, S, lam = 6.0, 3.0, 0.5
    cr = 2 * S / (b * (1 + lam))
    secs = [{"y": 0.0, "x_le": 0.0, "chord": cr}, {"y": b / 2, "x_le": 0.25 * (cr - lam * cr), "chord": lam * cr}]
    L = vlm_lattice(secs, [40], 8)
    Gm = np.linalg.solve(vlm_aic(L), -np.ones(len(L["P"])))
    Gs = np.bincount(L["strip"], weights=Gm, minlength=L["n_strips"])
    CLa = 2 * np.sum(Gs * L["dy"]) / S
    e = CLa ** 2 / (math.pi * b * b / S * vlm_trefftz(L, Gs, S))
    yy = np.linspace(0, b / 2, 60)
    LL = AE.lifting_line(yy, np.interp(yy, [0, b / 2], [cr, lam * cr]), np.zeros(60), np.full(60, 2 * math.pi),
                         np.zeros(60), b / 2)
    out["straight_AR12_taper0p5"] = {"vlm_CL_alpha": CLa, "lifting_line_CL_alpha": LL["CL_alpha"], "vlm_e": e,
                                     "lifting_line_e": LL["e_inviscid"]}
    sw, b2 = 30.0, 6.0
    secs = [{"y": 0.0, "x_le": 0.0, "chord": 1.0}, {"y": 3.0, "x_le": 3.0 * math.tan(math.radians(sw)), "chord": 0.4}]
    S2 = 4.2
    mac = 2 / 3 * (1 + 0.4 + 0.16) / 1.4
    L = vlm_lattice(secs, [40], 8)
    Gm = np.linalg.solve(vlm_aic(L), -np.ones(len(L["P"])))
    CLa = 2 * np.sum(Gm * (L["P"][:, 3] - L["P"][:, 1])) / S2
    Cma = -2 * np.sum(Gm * (L["P"][:, 3] - L["P"][:, 1]) * L["xbound"]) / (S2 * mac)
    r = {"vlm_CL_alpha": CLa, "vlm_x_np_m": -Cma / CLa * mac,
         "helmbold_CL_alpha": AL.lift_slope(b2 * b2 / S2, math.degrees(math.atan(
             (3.0 * math.tan(math.radians(sw)) + 0.5 * 0.4 - 0.5) / 3.0)))}
    try:
        import aerosandbox as asb
        af = asb.Airfoil("naca0012")
        wing = asb.Wing(symmetric=True, xsecs=[asb.WingXSec(xyz_le=[0, 0, 0], chord=1.0, airfoil=af),
                                               asb.WingXSec(xyz_le=[secs[1]["x_le"], 3.0, 0], chord=0.4, airfoil=af)])
        ap = asb.Airplane(wings=[wing], s_ref=S2, c_ref=mac, b_ref=b2, xyz_ref=[0, 0, 0])
        res = [asb.VortexLatticeMethod(ap, asb.OperatingPoint(velocity=30, alpha=a_), spanwise_resolution=30,
                                       chordwise_resolution=8).run() for a_ in (0.0, 2.0)]
        cla = (res[1]["CL"] - res[0]["CL"]) / math.radians(2)
        cma = (res[1]["Cm"] - res[0]["Cm"]) / math.radians(2)
        r.update({"aerosandbox_CL_alpha": float(cla), "aerosandbox_x_np_m": float(-cma / cla * mac),
                  "aerosandbox_version": asb.__version__})
    except Exception as exc:                                   # pragma: no cover - optional cross-check
        r["aerosandbox"] = f"not run ({exc.__class__.__name__})"
    out["swept30_AR8p6_taper0p4"] = r
    be, c0 = 8.0, 1.0
    yse = np.linspace(0, be / 2, 41)
    secs = [{"y": float(y), "x_le": float(0.25 * c0 * (1 - math.sqrt(max(1 - (y / (be / 2)) ** 2, 0)))),
             "chord": float(max(c0 * math.sqrt(max(1 - (y / (be / 2)) ** 2, 0)), 1e-3))} for y in yse]
    Se = math.pi * be * c0 / 4
    L = vlm_lattice(secs, [2] * 40, 6)
    Gm = np.linalg.solve(vlm_aic(L), -np.ones(len(L["P"])))
    Gs = np.bincount(L["strip"], weights=Gm, minlength=L["n_strips"])
    CLa = 2 * np.sum(Gs * L["dy"]) / Se
    out["elliptic_AR8"] = {"vlm_e": CLa ** 2 / (math.pi * be * be / Se * vlm_trefftz(L, Gs, Se)), "theory_e": 1.0}
    return out


# =====================================================================================================================
# 3. configuration parameters and planform
# =====================================================================================================================
@dataclass(frozen=True)
class Design:
    mtom: float = EC.MTOM_DESIGN
    ws: float = 345.0            # wing loading on the projected wing-body planform (Pa); trade: stall corner
    span: float = 6.80           # m (transport limit of the research)
    c0: float = 2.20             # centre-body root chord: apex to centre-line trailing edge (m)
    y_b: float = 0.55            # blend station: centre body -> outer panel (m)
    sweep_le: float = 25.0       # outer-panel leading-edge sweep (deg)
    taper: float = 0.45          # outer-panel taper c_tip / c_blend
    i_w: float | None = None     # outer-panel root incidence relative to the body datum (deg); None: least drag
    dihedral: float = 2.0        # outer-panel dihedral (deg)
    sm_min: float = SM_MIN
    fins: str = "tip"            # 'tip' (outward-canted tip fins), 'nacelle' (canted fins on the nacelle), 'none'
    fin_cant: float = 15.0       # deg from vertical, outward
    fin_AR: float = 2.0          # geometric aspect ratio of one fin panel (height^2 / area)
    fin_taper: float = 0.50
    fin_sweep_le: float = 35.0   # deg
    cnb_req: float = CNB_REQ
    elevon_in_frac: float = 0.22  # inboard end of the elevons, fraction of the outer-panel span from the blend
    elevon_y1_frac: float = 0.95  # outboard end, fraction of the semispan
    endplate_credit: bool = False  # Raymer end-plate credit of the tip fins on the wing span efficiency (sensitivity)


def planform(d: Design, S: float, x_le_b: float, washout: float, i_w: float) -> list:
    """LiftingSurface sections (starboard): centre-body root on the plane of symmetry (datum chord, apex at the origin),
    blend station (outer-panel root, incidence i_w) and tip (i_w - washout). Straight leading edges: the strake
    (apex -> blend) and the outer panel (sweep_le); the centre-body trailing edge runs forward from the centre-line
    trailing edge to the blend station (cranked-kite / lambda planform)."""
    s = d.span / 2
    c_b = (S - d.c0 * d.y_b) / (d.y_b + (1 + d.taper) * (s - d.y_b))
    c_t = d.taper * c_b
    return [
        {"y": 0.0, "x_le": 0.0, "z_le": 0.0, "chord": d.c0, "twist_deg": 0.0, "airfoil": AIRFOIL,
         "thickness_scale": TS_BODY, "g_inc": 0.0, "g_wo": 0.0},
        {"y": d.y_b, "x_le": x_le_b, "z_le": 0.0, "chord": c_b, "twist_deg": i_w, "airfoil": AIRFOIL,
         "thickness_scale": TS_ROOT, "g_inc": 1.0, "g_wo": 0.0},
        {"y": s, "x_le": x_le_b + (s - d.y_b) * math.tan(math.radians(d.sweep_le)),
         "z_le": (s - d.y_b) * math.tan(math.radians(d.dihedral)), "chord": c_t, "twist_deg": i_w - washout,
         "airfoil": AIRFOIL, "thickness_scale": TS_TIP, "g_inc": 1.0, "g_wo": 1.0},
    ]


def planform_area(secs: list) -> float:
    return 2 * sum(0.5 * (a["chord"] + b["chord"]) * (b["y"] - a["y"]) for a, b in zip(secs, secs[1:]))


def sweep_line(secs: list, k: int, frac: float) -> float:
    """Sweep (deg) of the chord-fraction line of segment k (between sections k and k+1)."""
    a, b = secs[k], secs[k + 1]
    return math.degrees(math.atan2(b["x_le"] + frac * b["chord"] - a["x_le"] - frac * a["chord"], b["y"] - a["y"]))


def kprime(delta_deg: float) -> float:
    t = FLAP_KPRIME
    return float(np.interp(abs(delta_deg), t["deflection_deg"], t["K_prime"]))


def delta_geo(delta_eff_deg: float) -> float:
    """Geometric deflection that gives the effective (linear-theory) deflection delta_eff = K'(delta) x delta."""
    if abs(delta_eff_deg) < 1e-9:
        return 0.0
    lo, hi = 0.0, 60.0
    for _ in range(50):
        m = 0.5 * (lo + hi)
        lo, hi = (m, hi) if kprime(m) * m < abs(delta_eff_deg) else (lo, m)
    return math.copysign(0.5 * (lo + hi), delta_eff_deg)


# =====================================================================================================================
# 4. aerodynamic model of the wing-body: vortex lattice + section data
# =====================================================================================================================
class WingBodyAero:
    """Linear wing-body model. Unit vortex-lattice solutions for the angle of attack (datum), the outer-panel incidence,
    the washout, the section zero-lift angles (camber, NeuralFoil) and the elevons; strip section data (tripped and
    clean NeuralFoil polars at the local Reynolds number, blended between the bounding sections like aero.py) for the
    profile drag, the section pitching moment and the critical-section CLmax. Everything is linear in
    (alpha, i_w, washout, delta), so trim, twist design and the polar are closed-form or 1-D."""

    def __init__(self, d: Design, secs: list, V: float, h: float, n_span=(12, 32), nc: int = 8):
        self.d, self.secs, self.V, self.h = d, secs, V, h
        s = secs[-1]["y"]
        self.s, self.b = s, 2 * s
        self.S = planform_area(secs)
        self.AR = self.b ** 2 / self.S
        m = oml.mean_aerodynamic_chord(secs)
        self.mac, self.x_le_mac, self.y_mac = m["mac"], m["x_le_mac"], m["y_mac"]
        y_root = max(q["y"] for q in secs if q.get("g_wo", 0.0) <= 1e-9)
        y_e0 = y_root + d.elevon_in_frac * (s - y_root)
        y_e1 = d.elevon_y1_frac * s
        y_em = 0.5 * (y_e0 + y_e1)
        self.elevons = [{"name": "elevon_inboard", "y0": y_e0, "y1": y_em, "xc": 1 - ELEVON_CF},
                        {"name": "elevon_outboard", "y0": y_em, "y1": y_e1, "xc": 1 - ELEVON_CF}]
        L = vlm_lattice(secs, list(n_span), nc, self.elevons)
        self.L = L
        A = vlm_aic(L)
        ym, c = L["ym"], L["c"]
        ns = L["n_strips"]
        ay = np.abs(ym)
        atm = AL.isa(h)
        self.Re = atm["rho"] * V * c / atm["mu"]
        ys = np.array([q["y"] for q in secs])
        k = np.clip(np.searchsorted(ys, ay, side="right") - 1, 0, len(secs) - 2)
        f = np.clip((ay - ys[k]) / (ys[k + 1] - ys[k]), 0.0, 1.0)
        self.kseg, self.fseg = k, f
        self.ch_trip, self.ch_clean = [], []
        a0, cm0, clmax, conf = (np.zeros(ns) for _ in range(4))
        for j in range(ns):
            sa, sb = secs[k[j]], secs[k[j] + 1]
            ta = tripped(sa["airfoil"], float(self.Re[j]), float(sa["thickness_scale"]))
            tb = tripped(sb["airfoil"], float(self.Re[j]), float(sb["thickness_scale"]))
            ca = clean(sa["airfoil"], float(self.Re[j]), float(sa["thickness_scale"]))
            cb = clean(sb["airfoil"], float(self.Re[j]), float(sb["thickness_scale"]))
            self.ch_trip.append((ta, tb, f[j]))
            self.ch_clean.append((ca, cb, f[j]))
            a0[j] = (1 - f[j]) * ca["alpha0_deg"] + f[j] * cb["alpha0_deg"]
            cm0[j] = (1 - f[j]) * ta["cm0"] + f[j] * tb["cm0"]
            clmax[j] = (1 - f[j]) * ca["clmax"] + f[j] * cb["clmax"]
            conf[j] = min(ta["min_confidence"], tb["min_confidence"])
        self.a0, self.cm0, self.clmax_sec, self.conf = a0, cm0, clmax, conf
        # unit right-hand sides (normal wash, V = 1): w = -(alpha + twist - alpha0 + delta on flap panels)
        st = L["strip"]
        g_inc = _sec_interp(secs, ay, "g_inc")                          # outer-panel incidence shape (0..1)
        g_wo = _sec_interp(secs, ay, "g_wo")                            # washout shape (0 inboard, 1 at the tip)
        flap = L["flap"]
        R = np.column_stack([-np.ones(len(st)), -g_inc[st], g_wo[st], np.radians(a0)[st],
                             -(flap == 0).astype(float), -(flap == 1).astype(float)])
        self.G = np.linalg.solve(A, R)                                   # columns: alpha, i_w, washout, camber, e1, e2
        dyp = L["P"][:, 3] - L["P"][:, 1]
        self.CLu = 2 * (self.G * dyp[:, None]).sum(0) / self.S
        self.Cmu0 = -2 * (self.G * (dyp * L["xbound"])[:, None]).sum(0) / (self.S * self.mac)   # about x = 0
        self.cm0_sections = float(np.sum(cm0 * c ** 2 * L["dy"]) / (self.S * self.mac))
        self.x_np_wing = -self.Cmu0[0] / self.CLu[0] * self.mac
        self.CL_alpha = float(self.CLu[0])
        self.sweep_c4_outer = sweep_line(secs, 1, 0.25)
        self.sweep_spar_outer = sweep_line(secs, 1, 0.30)
        self.lim = K_CLMAX_SEC * clmax * math.cos(math.radians(self.sweep_c4_outer))
        self.flap_strip_area = float(np.sum(c * L["dy"] * np.isin(np.arange(ns), np.unique(st[flap >= 0]))))
        self.i_w, self.washout = None, None
        self.x_np = self.x_np_wing
        self.cma_extra = 0.0

    # ---------------------------------------------------------------- twist state and total coefficients
    def set_twist(self, i_w: float, washout: float):
        self.i_w, self.washout = i_w, washout
        iw, wo = math.radians(i_w), math.radians(washout)
        self.CL0 = float(self.CLu[1] * iw + self.CLu[2] * wo + self.CLu[3])
        self.Cm00 = float(self.Cmu0[1] * iw + self.Cmu0[2] * wo + self.Cmu0[3]) + self.cm0_sections
        self.CLd = float(self.CLu[4] + self.CLu[5])
        self.Cmd0 = float(self.Cmu0[4] + self.Cmu0[5])

    def set_neutral_point(self, x_np: float):
        """Neutral point of the complete aircraft (VLM wing-body + nacelle Munk term + fin horizontal projection)."""
        self.x_np = x_np

    def coeffs(self, xref: float) -> tuple:
        """(CL0, CLa, CLd, Cm0, Cma, Cmd) about x = xref, with the complete-aircraft neutral point."""
        f = xref / self.mac
        Cm0 = self.Cm00 + self.CL0 * f
        Cma = -self.CL_alpha * (self.x_np - xref) / self.mac
        Cmd = self.Cmd0 + self.CLd * f
        return self.CL0, self.CL_alpha, self.CLd, Cm0, Cma, Cmd

    def trim(self, CL: float, xcg: float, dcm: float = 0.0) -> tuple:
        """(alpha_rad, delta_eff_rad) for lift CL and zero pitching moment about the CG (dcm: extra moment, e.g. thrust)."""
        CL0, CLa, CLd, Cm0, Cma, Cmd = self.coeffs(xcg)
        a, dl = np.linalg.solve(np.array([[CLa, CLd], [Cma, Cmd]]), np.array([CL - CL0, -Cm0 - dcm]))
        return float(a), float(dl)

    def gamma(self, alpha: float, delta_eff: float) -> np.ndarray:
        g = self.G
        return (g[:, 0] * alpha + g[:, 1] * math.radians(self.i_w) + g[:, 2] * math.radians(self.washout) + g[:, 3]
                + (g[:, 4] + g[:, 5]) * delta_eff)

    def strips(self, alpha: float, delta_eff: float) -> dict:
        L = self.L
        ns = L["n_strips"]
        G = self.gamma(alpha, delta_eff)
        Gs = np.bincount(L["strip"], weights=G, minlength=ns)
        Gd = np.bincount(L["strip"], weights=(self.G[:, 4] + self.G[:, 5]) * delta_eff, minlength=ns)
        return {"Gs": Gs, "cl": 2 * Gs / L["c"], "cl_flap": 2 * Gd / L["c"]}

    def point(self, CL: float, xcg: float, dcm: float = 0.0, tripped_polars: bool = True) -> dict:
        a, de = self.trim(CL, xcg, dcm)
        st = self.strips(a, de)
        L = self.L
        CDi = vlm_trefftz(L, st["Gs"], self.S)
        chs = self.ch_trip if tripped_polars else self.ch_clean
        cdl = np.array([(1 - f) * AE.section_cd(ca, float(x)) + f * AE.section_cd(cb, float(x))
                        for (ca, cb, f), x in zip(chs, st["cl"])])
        CDp = float(np.sum(cdl * L["c"] * L["dy"]) / self.S) * (K_CD_TRIP if tripped_polars else EC.CORR[
            "cd_factor_clean"]["value"])
        dg = delta_geo(math.degrees(de))
        dCD_flap = 0.0144 * ELEVON_CF * (self.flap_strip_area / self.S) * max(abs(dg) - 10.0, 0.0)
        return {"alpha_rad": a, "alpha_deg": math.degrees(a), "delta_eff_deg": math.degrees(de), "delta_deg": dg,
                "CDi": CDi, "CDp": CDp, "dCD_elevon": dCD_flap, "e_trim": CL ** 2 / (math.pi * self.AR * CDi)
                if CDi > 0 else float("nan"), "cl": st["cl"], "cl_flap": st["cl_flap"]}

    def clmax(self, xcg: float, dcm_fn=None, cl_lo=0.4, cl_hi=1.8) -> dict:
        """Trimmed CLmax: the wing CL (x k_clmax_3d) at which the angle-of-attack part of the first strip's cl reaches
        k_sec x cl_max x cos(Lambda_c/4 outer); dcm_fn(CL) gives the extra (thrust) moment. Also stops when the
        trim deflection reaches the trim limit."""
        L = self.L
        prev = None
        for CL in np.arange(cl_lo, cl_hi, 0.005):
            dcm = dcm_fn(CL) if dcm_fn else 0.0
            a, de = self.trim(CL, xcg, dcm)
            st = self.strips(a, de)
            r = (st["cl"] - st["cl_flap"]) / self.lim
            dg = delta_geo(math.degrees(de))
            if r.max() >= 1.0 or abs(dg) > ELEVON_TRIM_MAX:
                j = int(np.argmax(r))
                limit = "section stall" if r.max() >= 1.0 else "elevon trim limit"
                return {"CLmax": K_CLMAX_3D * float(CL), "CL_critical": float(CL), "alpha_deg": math.degrees(a),
                        "delta_deg": dg, "stall_onset_eta": float(abs(L["ym"][j]) / self.s),
                        "stall_onset_y_m": float(abs(L["ym"][j])), "limited_by": limit,
                        "cl_over_limit_elevon_strips": float(np.max(r[np.isin(np.arange(L["n_strips"]),
                                                                              np.unique(L["strip"][L["flap"] >= 0]))]))}
            prev = CL
        return {"CLmax": K_CLMAX_3D * float(prev), "CL_critical": float(prev), "limited_by": "scan end"}

    def design_twist(self, i_w: float, CL_d: float, xcg: float, dcm: float = 0.0) -> float:
        """Washout (deg) that trims CL_d about xcg with the elevons neutral (2 x 2 linear solve in alpha, washout)."""
        f = xcg / self.mac
        iw = math.radians(i_w)
        CLc = self.CLu[1] * iw + self.CLu[3]
        Cmc = self.Cmu0[1] * iw + self.Cmu0[3] + self.cm0_sections + CLc * f
        Cma = -self.CL_alpha * (self.x_np - xcg) / self.mac
        Cmw = self.Cmu0[2] + self.CLu[2] * f
        M = np.array([[self.CL_alpha, self.CLu[2]], [Cma, Cmw]])
        a, wo = np.linalg.solve(M, np.array([CL_d - CLc, -Cmc - dcm]))
        return math.degrees(wo)

    def spanload(self, CL: float, xcg: float) -> dict:
        """Trimmed span load (starboard half): y, c, cl, cl*c / (CL * mean chord) shape for the structure."""
        a, de = self.trim(CL, xcg)
        st = self.strips(a, de)
        m = self.L["ym"] > 0
        return {"y": self.L["ym"][m], "c": self.L["c"][m], "cl": st["cl"][m], "Gs": st["Gs"][m],
                "dy": self.L["dy"][m]}


# =====================================================================================================================
# 5. outer mould line: lofted wing-body, dorsal spine / engine nacelle, fins; packaging height map
# =====================================================================================================================
H_SPINE = rec("dorsal_spine_height_over_bay_m", 0.11, "", basis="dorsal spine (UCAV hump) on the centre line from the "
              "avionics bay to the engine nacelle: gives the payload bay and the fuel cells the depth that the 17 % "
              "centre-body section lacks at 60-65 % of its chord, where a tailless aircraft must carry its CG")
X_HUB_BEHIND_TE = rec("hub_face_behind_centre_body_te_m", 0.10, "", basis="engine (0.26 m with SG750) straddles the "
                      "centre-body trailing edge; the nacelle closes to the cowl lip 0.05 m behind the hub face (same "
                      "annular cooling exit as the endurance study)")


class OML:
    """Wing-body loft (oml.LiftingSurface on the planform sections) with an internal height map, the dorsal spine /
    engine nacelle (oml.Fuselage) and the propeller station. All packaging checks read this object."""

    NX, NY = 260, 120

    def __init__(self, d: Design, secs: list, x_bay: float):
        self.d, self.secs = d, secs
        self.ls = oml.LiftingSurface(secs, n_chord=90, name="wing_body")
        s = secs[-1]["y"]
        self.s = s
        self.c0 = d.c0
        x_te_tip = secs[-1]["x_le"] + secs[-1]["chord"]
        self.x_max = max(d.c0, x_te_tip) + 0.01
        ys = np.r_[np.linspace(0.0, d.y_b, 56)[:-1], np.linspace(d.y_b, s - 1e-6, self.NY - 55)]
        xs = np.linspace(0.0, self.x_max, self.NX)
        ZU = np.full((len(xs), len(ys)), np.nan)
        ZL = np.full((len(xs), len(ys)), np.nan)
        eta = self.ls.span_coords()
        yk = np.array([q["y"] for q in secs])
        for j, y in enumerate(ys):
            e = float(np.interp(y, yk, eta))
            P = self.ls.loop_at(min(e, eta[-1]))
            n = (len(P) + 2) // 2
            up = P[:n][::-1]
            lo = P[n - 1:]
            up = up[np.argsort(up[:, 0])]
            lo = lo[np.argsort(lo[:, 0])]
            m = (xs >= up[0, 0]) & (xs <= up[-1, 0])
            ZU[m, j] = np.interp(xs[m], up[:, 0], up[:, 2])
            ZL[m, j] = np.interp(xs[m], lo[:, 0], lo[:, 2])
        self.xs, self.ys, self.ZU, self.ZL = xs, ys, ZU, ZL
        self.x_bay = x_bay
        self.build_spine()

    # ------------------------------------------------------------------ height map access
    def _ij(self, x, y):
        i = float(np.interp(x, self.xs, np.arange(len(self.xs))))
        j = float(np.interp(abs(y), self.ys, np.arange(len(self.ys))))
        return i, j

    def _bil(self, Z, x, y):
        i, j = self._ij(x, y)
        i0, j0 = min(int(i), len(self.xs) - 2), min(int(j), len(self.ys) - 2)
        fi, fj = i - i0, j - j0
        v = (Z[i0, j0] * (1 - fi) * (1 - fj) + Z[i0 + 1, j0] * fi * (1 - fj) + Z[i0, j0 + 1] * (1 - fi) * fj
             + Z[i0 + 1, j0 + 1] * fi * fj)
        return float(v)

    def z_up(self, x, y=0.0):
        return self._bil(self.ZU, x, y)

    def z_lo(self, x, y=0.0):
        return self._bil(self.ZL, x, y)

    def thickness(self, x, y=0.0):
        v = self.z_up(x, y) - self.z_lo(x, y)
        return v if math.isfinite(v) else 0.0

    # ------------------------------------------------------------------ dorsal spine and engine nacelle
    def build_spine(self):
        d = self.d
        self.x_hub = d.c0 + X_HUB_BEHIND_TE
        self.x_prop = self.x_hub + HUB_SPACER + 0.02
        self.x_lip = self.x_hub + 0.05
        self.spinner = {"x0": self.x_hub + 0.07, "x1": self.x_prop + 0.12, "d": 0.17}
        x_e0 = self.x_hub - ENV_L_SG                  # SG750 end of the engine box
        x_int = self.x_hub - 0.18                     # intake box (throttle bodies) fore-aft centre
        zlo_eng = min(self.z_lo(x, 0.0) for x in np.linspace(x_e0, min(self.x_hub, d.c0 - 0.005), 12))
        # crank axis: intake cover (ENV_H - ENV_ZC below the crank axis) 0.02 m above the floor of the ventral blister
        self.zt = zlo_eng - BLISTER + (ENV_H - ENV_ZC) + 0.02
        top_eng = self.zt + ENV_ZC + 0.035            # cylinder fins + baffle + cowl skin
        xb = self.x_bay
        x0 = X_AVION - 0.06
        xs_ = [x0, CHUTE_X[0], 0.5 * (CHUTE_X[1] + xb - BAY_LEN / 2), xb, x_e0 - 0.06, self.x_hub - 0.04, self.x_lip]
        tops = [self.z_up(x0) + 0.004, self.z_up(CHUTE_X[0]) + 0.045, None, self.z_up(xb) + H_SPINE, top_eng, top_eng,
                self.zt + 0.15]
        tops[2] = 0.5 * (tops[1] + tops[3])
        ws_ = [0.16, 0.34, 0.40, 0.42, NACELLE_W, NACELLE_W, 0.30]
        bots = []
        for x in xs_:
            if x <= x_e0 - 0.07:
                bots.append(self.z_lo(min(x, d.c0 - 0.01)) + 0.015)
            elif x < self.x_lip - 1e-6:
                bots.append(min(self.zt - (ENV_H - ENV_ZC) - 0.02, self.z_lo(min(x, d.c0 - 0.01)) - 0.005))
            else:
                bots.append(self.zt - 0.15)
        tops = [max(t, b + 0.02) for t, b in zip(tops, bots)]
        st = []
        for x, w, t, b in zip(xs_, ws_, tops, bots):
            h = t - b
            st.append([x, w, h, b + 0.5 * h, 2.6, 2.4, 0.5])
        self.spine_stations = np.array(st)
        self.spine = oml.Fuselage(self.spine_stations)
        self.top_eng, self.zlo_eng = top_eng, zlo_eng

    def spine_top(self, x):
        w, h, zc, nt, nb = self.spine.section(np.atleast_1d(x))
        tf = self.spine.top_frac(np.atleast_1d(x))
        return float((zc + tf * h)[0])

    def spine_bot(self, x):
        w, h, zc, nt, nb = self.spine.section(np.atleast_1d(x))
        tf = self.spine.top_frac(np.atleast_1d(x))
        return float((zc - (1 - tf) * h)[0])

    def spine_width(self, x):
        return float(self.spine.section(np.atleast_1d(x))[0][0])

    def inside_wingbody(self, x, y, z) -> bool:
        if x < 0 or x > self.x_max or abs(y) > self.s:
            return False
        zu, zl = self.z_up(x, y), self.z_lo(x, y)
        return math.isfinite(zu) and math.isfinite(zl) and zl <= z <= zu

    @functools.cached_property
    def spine_props(self) -> dict:
        """Exposed wetted area (triangles whose centroid lies outside the wing-body), exposed volume estimate, side
        area, length and maximum width/height of the spine-nacelle."""
        m = self.spine.mesh(140, 64)
        V, F = m.V, m.F
        A = 0.5 * np.linalg.norm(np.cross(V[F[:, 1]] - V[F[:, 0]], V[F[:, 2]] - V[F[:, 0]]), axis=1)
        C = V[F].mean(axis=1)
        exposed = np.array([not self.inside_wingbody(cx, cy, cz) for cx, cy, cz in C])
        cap = C[:, 0] > self.x_lip - 1e-4
        S_exp = float(A[exposed & ~cap].sum())
        xs = np.linspace(self.spine.x0, self.spine.x1, 200)
        hump = np.array([max(self.spine_top(x) - (self.z_up(x) if x < self.d.c0 else self.spine_bot(x)), 0.0)
                         for x in xs])
        wid = np.array([self.spine_width(x) for x in xs])
        vol_exp = float(np.trapz(hump * wid * 0.8, xs))
        side = float(np.trapz(hump, xs))
        return {"S_wet_exposed": S_exp, "S_wet_total": float(A[~cap].sum()), "volume_exposed": vol_exp,
                "side_area_exposed": side, "length": self.spine.x1 - self.spine.x0, "w_max": float(wid.max()),
                "x_centroid_exposed": float((A[exposed] * C[exposed, 0]).sum() / max(A[exposed].sum(), 1e-9)),
                "mesh": m}

    @functools.cached_property
    def wingbody_props(self) -> dict:
        m = self.ls.mesh(refine=6)
        Sw = 2 * float(m.area())
        root = self.ls.section_points(0)
        cap = 0.5 * abs(np.sum(root[:, 0] * np.roll(root[:, 2], -1) - np.roll(root[:, 0], -1) * root[:, 2]))
        S_wet = Sw - 2 * cap
        # centre-body region (|y| <= y_b): wetted area, volume, side area
        j = self.ys <= self.d.y_b + 1e-9
        T = np.nan_to_num(self.ZU - self.ZL)
        dx = self.xs[1] - self.xs[0]
        vol_b = 2 * float(np.trapz(T[:, j].sum(0) * dx, self.ys[j]))
        side_b = float(np.nansum(T[:, 0]) * dx)
        return {"S_wet": S_wet, "mesh": m, "volume_centre_body": vol_b, "side_area_centre_line": side_b,
                "t_max_centre": float(np.nanmax(T[:, 0])), "volume_total_half": float(np.trapz(T.sum(0) * dx, self.ys))}

    def internal_height(self, x, y, inset=TANK_INSET):
        """Usable internal height at (x, y): body (plus spine where it covers) minus insets."""
        zu, zl = self.z_up(x, y), self.z_lo(x, y)
        if not (math.isfinite(zu) and math.isfinite(zl)):
            return 0.0
        if abs(y) < 0.5 * self.spine_width(x) - 0.02 and self.spine.x0 <= x <= self.spine.x1:
            zu = max(zu, self.spine_top(x) - 0.012)
        return max(zu - zl - 2 * inset, 0.0)


def fin_sections(d: Design, ob: OML, secs: list, S_panel: float) -> list:
    """One fin panel (starboard), LiftingSurface format; area S_panel (m2), geometric AR d.fin_AR, taper, LE sweep and
    outward cant from the vertical. 'tip': root on the wing-tip chord (trailing edges aligned); 'nacelle': root on the
    nacelle side, trailing edge 0.15 m ahead of the propeller plane."""
    h = math.sqrt(d.fin_AR * S_panel)
    cr = 2 * S_panel / (h * (1 + d.fin_taper))
    ct = d.fin_taper * cr
    g = math.radians(d.fin_cant)
    if d.fins == "tip":
        t = secs[-1]
        x_te = t["x_le"] + t["chord"]
        y0, z0 = t["y"], t["z_le"] + 0.01
    else:
        x_te = ob.x_prop - 0.15
        y0 = 0.5 * NACELLE_W - 0.03
        z0 = ob.zt + 0.02
    x0 = x_te - cr
    dx = h * math.tan(math.radians(d.fin_sweep_le))
    return [{"y": y0, "x_le": x0, "z_le": z0, "chord": cr, "twist_deg": 0.0, "airfoil": "n0012"},
            {"y": y0 + h * math.sin(g), "x_le": x0 + dx, "z_le": z0 + h * math.cos(g), "chord": ct, "twist_deg": 0.0,
             "airfoil": "n0012"}]
