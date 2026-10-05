#!/usr/bin/env python3
"""YK-250 concept study "production" (lowest manufacturing cost, risk and maintenance burden): first-pass sizing.

Run from the repository root:

    PYTHONPATH=. python3 ucav250/data/concepts/production/calc.py          (full run with all trades, about 11 min)
    PYTHONPATH=. python3 ucav250/data/concepts/production/calc.py --quick  (development: skips the sweeps)

Concept (decided by the trade studies in this file; reasons recorded in concept.yaml -> trades and notes_tr.md):
  * Limbach L 275 EF pusher (Mejzlik 32x18 2B, 80 mm hub spacer) at the end of a short, chined pod; the propeller
    turns between two tail booms.
  * Shoulder wing in three transportable pieces: a constant-chord centre section (one rib family, cylindrical =
    developable skins) bolted on the pod chassis, and two linearly tapered outer panels with the same NLF(1)-0416
    section and a small linear washout (3 deg from trade 1: nearly conical, i.e. nearly developable skins) plugged in
    with a spar tongue.
  * Twin bought CFRP tube booms clamped under the centre section; constant-chord H stabiliser between the boom ends
    (one mould: symmetric section and planform), two identical outward-canted fins (one mould pair). No flaps.
  * Fixed tricycle gear: GFRP spring bow + TOST wheels, steerable nose leg.
  * Structure = CHASSIS + SHELL: aluminium sheet-metal/extrusion chassis with machined 7075 hard points carries every
    point load (wing, gear, engine, parachute); the composite shell panels are bolt-on and non-structural, so any panel
    can be removed for access without affecting the load paths.

Shared equations (same numbers for every concept): the propulsion model (engine WOT torque with the sigma^1.23 lapse,
part-load BSFC iterated with the actual power incl. the generator load, Mejzlik 32x18 2B table), tripped section
polars x 1.15, wing lifting-line aero, level-flight/loiter/range/climb points, mission segments (sizinglib), ceiling,
gust envelope, V-n and the constraint diagram are IMPORTED from the reviewed endurance chain
(ucav250/data/concepts/endurance/calc.py, module ``EC``). This file implements what differs: pod, boom and H-tail
geometry, drag items, the mass model (aluminium chassis, booms, H-tail, constant-chord centre section), stability with
the H-tail, slipstream-aided take-off rotation, ground geometry, trades, packaging and the 3-view.

Every number is read from the research files (key path in ``ref``), computed here (formula in ``basis``) or a
labelled engineering estimate (``tag: estimate`` with a basis). Frame: X aft from the nose tip, Y starboard, Z up from
the pod datum line through the nose tip; SI units, angles in degrees in the YAML.
"""
from __future__ import annotations

import functools
import importlib.util
import math
import sys
import time
from dataclasses import dataclass, field, replace
from pathlib import Path

import numpy as np
import yaml

HERE = Path(__file__).resolve().parent
REPO = HERE.parents[3]
if str(REPO) not in sys.path:
    sys.path.insert(0, str(REPO))
sys.dont_write_bytecode = True          # never write __pycache__ into another concept's folder


def _import_endurance_chain():
    path = HERE.parent / "endurance" / "calc.py"
    spec = importlib.util.spec_from_file_location("yk250_endurance_chain", path)
    mod = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = mod
    spec.loader.exec_module(mod)
    return mod


EC = _import_endurance_chain()           # reviewed endurance chain: shared equations

from ucav250.analysis import aero as AE        # noqa: E402
from ucav250.analysis import aerolib as AL     # noqa: E402
from ucav250.analysis import sizinglib as SZ   # noqa: E402
from ucav250.analysis import stablib as SL     # noqa: E402
from ucav250.analysis import structlib as ST   # noqa: E402
from ucav250.design import oml                 # noqa: E402

# NeuralFoil polars missing from the shared cache are written to this concept's folder only (the shared
# ucav250/data/polars cache is read, never written by this script).
POLAR_LOCAL = HERE / "polars"
_shared_polar_path = AE._polar_path


def _polar_path_local(airfoil, Re, n_crit, ts=1.0):
    p = _shared_polar_path(airfoil, Re, n_crit, ts)
    if p.exists():
        return p
    POLAR_LOCAL.mkdir(exist_ok=True)
    return POLAR_LOCAL / p.name


AE._polar_path = _polar_path_local

G = AL.G0
RHO0 = 1.225
py = EC.py

# =====================================================================================================================
# 0. input record
# =====================================================================================================================
INPUTS: dict = {}
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


SHARED_KEYS = [
    "endurance_requirement_h", "loiter_speed_floor_m_per_s_eas", "cruise_speed_band_m_per_s",
    "service_ceiling_requirement_m", "loiter_altitude_m", "runway_length_m", "climb_rate_sea_level_requirement_m_per_s",
    "transit_radius_m", "reserve_fraction_of_trip_time", "trapped_fuel_fraction", "mtom_hard_cap_kg",
    "mass_growth_allowance_fraction_of_empty", "payload_design_kg", "payload_items_kg",
    "turret_bay_growth_envelope_m", "engine_power_max_W", "engine_power_max_continuous_W", "engine_wot_curve",
    "engine_power_lapse_exponent", "bsfc_curve_power_fraction_g_per_kWh", "fuel_density_kg_per_m3",
    "engine_group_installed_kg", "electrical_load_continuous_W", "generator_efficiency", "engine_envelope_m",
    "propeller_diameter_m", "propeller_wot_thrust_factor", "propeller_installation_factor", "propeller_table",
    "prop_ground_clearance_min_m", "cl_max_section_factor", "cd_factor_tripped", "trip_location_x_over_c",
    "k_clmax_3d", "VC_m_per_s_eas", "spar_cap_allowable_ultimate_Pa", "shear_web_allowable_ultimate_Pa",
    "areal_masses_kg_per_m2", "gear_fairings_kg", "main_wheel_tyre_m", "tail_volume_target_VH",
    "directional_stability_target_per_rad", "static_margin_min", "prop_hub_spacer_m", "thrust_line_z_m",
    "payload_bay_length_m", "tank_volume_efficiency", "theta_touchdown_target_deg",
    "fuselage_bottom_static_height_min_m", "span_max_m", "tip_chord_min_m", "field_ground_roll_max_m",
    "engine_intake_box_width_m"]
for _k in SHARED_KEYS:
    INPUTS[_k] = dict(EC.INPUTS[_k], shared="identical value in the endurance chain (EC)")

# =====================================================================================================================
# 1. research inputs (aliases of the shared chain - identical numbers for every concept)
# =====================================================================================================================
BL, EN, AY, CP, CM = EC.BL, EC.EN, EC.AY, EC.CP, EC.CM
END_REQ_H = EC.END_REQ_H
V_LOIT_MIN_EAS = EC.V_LOIT_MIN_EAS
V_CRUISE_MIN, V_CRUISE_MAX = EC.V_CRUISE_MIN, EC.V_CRUISE_MAX
H_CEIL_REQ, H_LOITER = EC.H_CEIL_REQ, EC.H_LOITER
RUNWAY_M, ROC_REQ = EC.RUNWAY_M, EC.ROC_REQ
MTOM_CAP = EC.MTOM_CAP
MTOM_DESIGN = rec("mtom_design_kg", float(BL["design_point"]["mtow_kg"]["value"]), "baseline.yaml#design_point.mtow_kg",
                  basis="design MTOM held at the baseline 145 kg (4.9 kg below the 149.9 kg M2 cap), the same design "
                        "point as the other concepts so they compare at equal MTOM; the mass left after payload and "
                        "empty mass is fuel")
GROWTH = EC.GROWTH
PAYLOAD = EC.PAYLOAD
M_TURRET, M_MCOMP, M_TRAY, M_RESEARCH = EC.M_TURRET, EC.M_MCOMP, EC.M_TRAY, EC.M_RESEARCH
TUR_D, TUR_D_GROWTH, TUR_H_GROWTH = EC.TUR_D, EC.TUR_D_GROWTH, EC.TUR_H_GROWTH
P_MAX, P_MCP = EC.P_MAX, EC.P_MCP
RHO_FUEL = EC.RHO_FUEL
P_ELEC, P_GEN_SHAFT = EC.P_ELEC, EC.P_GEN_SHAFT
ENV_L, ENV_L_SG, ENV_W, ENV_H, ENV_ZC = EC.ENV_L, EC.ENV_L_SG, EC.ENV_W, EC.ENV_H, EC.ENV_ZC
M_PROP = EC.M_PROP
CLEAR_PROP_GROUND, CLEAR_PROP_LONG = EC.CLEAR_PROP_GROUND, EC.CLEAR_PROP_LONG
CLEAR_PROP_RADIAL = rec("prop_radial_tip_clearance_min_m", 0.026, "standards.yaml#propeller_clearance.radial_tip_m",
                        "standard", basis="CS-VLA 925(c)(1); the boom spacing below gives about 4x this value")
K_CD_TRIP = EC.K_CD_TRIP
N_POS, N_NEG, FOS = EC.N_POS, EC.N_NEG, EC.FOS
VC_EAS = EC.VC_EAS
UD, PW = EC.UD, EC.PW
SIG_CAP, TAU_WEB = EC.SIG_CAP, EC.TAU_WEB
PLY_PW, CORE_FOAM, PAINT = EC.PLY_PW, EC.CORE_FOAM, EC.PAINT
SKIN_PRIMARY, SKIN_SECONDARY, SKIN_TAIL, RIB_AREAL = EC.SKIN_PRIMARY, EC.SKIN_SECONDARY, EC.SKIN_TAIL, EC.RIB_AREAL
SR = BL["mass_targets"]["systems_rollup_kg"]
GEAR_WHEELS, GEAR_LEGS_145, GEAR_FAIRINGS = EC.GEAR_WHEELS, EC.GEAR_LEGS_145, EC.GEAR_FAIRINGS
WHEEL_D, WHEEL_W = EC.WHEEL_D, EC.WHEEL_W
CHUTE_BOX = EC.CHUTE_BOX
HUB_SPACER, Z_T = EC.HUB_SPACER, EC.Z_T
BAY_LEN, K_TANK, TANK_INSET = EC.BAY_LEN, EC.K_TANK, EC.TANK_INSET
THETA_TD_TARGET, GEAR_TRAVEL = EC.THETA_TD_TARGET, EC.GEAR_TRAVEL
SPAN_MAX, TIP_CHORD_MIN, FIELD_ROLL_MAX = EC.SPAN_MAX, EC.TIP_CHORD_MIN, EC.FIELD_ROLL_MAX
INTAKE_W = EC.INTAKE_W
TC_ROOT = 0.16
A_DISC = math.pi / 4 * EC.D_PROP ** 2

MAT = BL["materials_to_use"]
AL6061 = MAT["metals"]["al_6061_t6_sheet"]
AL2024 = MAT["metals"]["al_2024_t3_sheet"]
AL7075 = MAT["metals"]["al_7075_t651_plate"]
rec("aluminium_alloys", {"al_6061_t6_density_kg_per_m3": AL6061["density"], "al_6061_t6_Fty_Pa": AL6061["Fty"],
                         "al_2024_t3_density_kg_per_m3": AL2024["density"], "al_2024_t3_Fty_Pa": AL2024["Fty"],
                         "al_7075_t651_Ftu_Pa": AL7075["Ftu"]},
    "baseline.yaml#materials_to_use.metals (MIL-HDBK-5G A-basis)", "standard",
    basis="chassis: 6061-T6 extrusions and bulkheads (formable, weldable, corrosion resistant), 2024-T3 shear webs, "
          "7075-T651 machined hard points; isolation ply + primer + wet-installed fasteners at every Al/CFRP faying "
          "surface (baseline.yaml#fasteners.materials_in_cfrp)")

# EC propulsion objects (PROP, set_prop) are used through the module so that EC.set_prop switches them everywhere.

# =====================================================================================================================
# 2. concept-specific choices
# =====================================================================================================================
X_TURRET = rec("turret_x_m", 0.38, "", basis="EO/IR ball centre in the chin, ahead of the nose gear (same station as "
               "the endurance chain): the forward and nadir fields of view are free of propeller and gear")
X_NG = rec("nose_gear_x_m", 0.66, "", basis="nose-gear trunnion frame behind the turret bay; leg rakes forward 0.06 m")
X_BATT = rec("battery_pdu_x_m", 0.30, "", basis="14S2P LiFePO4 + PDU tray beside the turret bay (nose ballast role)")
X_AVION = rec("avionics_tray_x_m", 0.45, "", basis="slide-out avionics tray (one LRU) above the turret bay")
X_CHUTE = rec("parachute_bay_x_m", [0.58, 0.96], "", basis="GRS 4/240 container (0.375 m long) under a top hatch ahead "
              "of the wing: canopy leaves forward of the wing and far from the pusher disc")
X_PARACHUTE = tuple(X_CHUTE)
Z_WING_ROOT = rec("wing_root_le_z_m", 0.280, "", basis="shoulder wing on the pod top (0.253 m) in a shallow saddle; "
                  "NLF(1)-0416 lower surface 0.055 c below the chord at 0.3 c")
ENGINE_BAY_LEN = rec("firewall_to_hub_face_m", 0.29, "baseline.yaml#engine.envelope_m.length_with_sg750", basis=(
    "SG750 front face 0.26 m ahead of the hub face + 0.03 m to the firewall (mount truss around the SG750)"))
TRANS_MIN = rec("aft_body_transition_min_m", 0.22, "", basis="minimum length from the constant pod section to the "
                "firewall in which the chine rises 0.11 m and the dorsal hump 0.08 m (slopes <= 25 deg)")
FW_CLEAR = rec("tank_to_firewall_clearance_m", 0.028, "baseline.yaml#design_loads.fuel_system_rules.firewall_clearance_m",
               "standard", basis="13 mm CS-LUAS minimum + 15 mm frame")
LEAK_FRAC = rec("leakage_protuberance_fraction", 0.07, "", basis="5 % of the endurance chain (well-sealed composite "
                "airframe) + 2 % for the larger number of removable-panel gaps and flush quarter-turn fastener rows of "
                "the bolt-on shell (estimate)")
SHELL_EDGE = rec("shell_panel_edge_allowance", 0.12, "", basis="joggled panel edges, edge close-outs, quarter-turn "
                 "receptacle doublers of the bolt-on shell as a fraction of the sandwich panel mass (estimate)")
K_FIN_AR = rec("fin_effective_aspect_ratio_factor", 1.4, "", basis="end-plate effect of the stabiliser and boom at the "
               "fin root: effective AR between 1.0 (free half-wing) and 2.0 (infinite reflection plane) x h^2/S; 1.4 "
               "taken (estimate, no end-plate credit on the stabiliser itself)")
ETA_T = rec("tail_efficiency_power_off", 0.90, "", basis="dynamic-pressure ratio at the tail used for stability (power "
            "off; the slipstream raises it with power, a stabilising effect not credited)")
CLT_MAX_ROT = rec("tail_cl_max_rotation", 0.90, "", basis="tail lift coefficient with full up elevator used for the "
                  "rotation check (same value as the endurance chain)")
TAIL_DEV_X = rec("slipstream_development", "actuator disk: w(x) = w_disk (1 + x / sqrt(R^2 + x^2)), r(x) from "
                 "continuity", "", basis="momentum theory (Glauert); q in the immersed stabiliser strip = 0.5 rho (V + "
                 "w(x))^2 with T from the Mejzlik table; immersion = slipstream chord at the stabiliser height / span")
SPATS_MASS = rec("wheel_spats_kg", 0.70, "", basis="endurance-chain gear fairings 0.95 kg = 2 main spats 0.25 + nose spat "
                 "0.20 + bow leg fairings 0.25; the leg fairings stay in every variant (an unfaired flat spring leg has "
                 "D/q of about 1.4 x frontal area). Gear D/q per frontal area (Raymer 6th ed. Table 12.6, values as "
                 "recalled - verify): with spats 0.13 main / 0.35 nose wheel + fork (endurance chain); without spats "
                 "0.25 per wheel + 1.0 x 0.0045 m2 for the nose fork (estimate of the fork frontal area)")


@dataclass(frozen=True)
class Design:
    mtom: float = MTOM_DESIGN
    ws: float = 470.0            # wing loading Pa (stall-limited, set in the trade)
    AR: float = 12.0             # trade
    y_cs: float = 1.00           # constant-chord centre-section half span = outer panel joint station (trade)
    taper_o: float = 0.50        # outer panel tip/root chord
    washout: float = 2.0         # outer panel linear washout, deg (0 = conical skins; trade: stall onset)
    dihedral_o: float = 1.5      # outer panel dihedral (centre section flat)
    ts_tip: float = 1.0          # tip thickness scale (1.0 = constant 16 %, one rib family; trade)
    tail: str = "H"              # "H": twin boom + H tail (this concept); "V": single fuselage + V tail (trade)
    y_boom: float = 0.56         # boom axis station
    d_boom: float = 0.080        # boom tube outer diameter
    t_boom_min: float = 1.5e-3   # bought-tube minimum wall (handling damage), m
    tail_gap: float = 0.65       # propeller plane -> stabiliser LE (sets the boom length; trade)
    VH: float = 0.45
    cnb_req: float = 0.057
    VV_min: float = 0.020
    fin_AR: float = 1.6          # fin h^2 / S (one fin)
    fin_taper: float = 0.55
    fin_le_sweep: float = 35.0
    fin_cant: float = 12.0       # outward cant (shape language; effective areas by projection)
    v_plug: float = 0.0          # V variant: constant-section plug that moves the engine aft (tail arm)
    tail_AR: float = 4.5         # V variant panel-pair AR (endurance chain value)
    tail_taper: float = 0.60
    tail_le_sweep: float = 30.0
    sm_min: float = 0.10
    flaps: bool = False          # trade: no flaps (production) vs centre-section take-off flaps
    flap_deg: float = 15.0
    spats: bool = True           # trade
    chassis: str = "aluminium"   # trade: aluminium sheet metal vs CFRP (endurance-chain type)


rec("design_choices", {"y_boom_m": Design.y_boom, "d_boom_m": Design.d_boom, "t_boom_min_m": Design.t_boom_min,
                       "fin_AR": Design.fin_AR, "fin_taper": Design.fin_taper, "fin_le_sweep_deg": Design.fin_le_sweep,
                       "fin_cant_deg": Design.fin_cant, "outer_panel_taper": Design.taper_o,
                       "outer_panel_dihedral_deg": Design.dihedral_o},
    "", basis=("y_boom: propeller tip to boom surface >= 0.10 m (about 4 x the CS-VLA 26 mm, limits blade-passage "
               "pressure pulses on the booms) with an 80 mm tube; d/t 80/1.5 mm: standard roll-wrapped CFRP tube size, "
               "wall set by handling damage, strength checked below; fins: identical parts from one mould pair, 35 deg "
               "LE sweep and 12 deg outward cant for the UCAV shape language at no tooling cost (cant reduces the "
               "vertical projection by cos^2); outer panel taper 0.5 keeps the tip chord >= 0.25 m (aileron servo)"))


# =====================================================================================================================
# 3. pod (fuselage) geometry
# =====================================================================================================================
class Pod:
    """Short chined pod: forebody (turret, avionics, battery, nose gear, parachute), constant mid-body (forward fuel
    cell + payload bay), transition (aft fuel cell, chine and dorsal hump rise to the engine), engine bay, cowl lip.
    ``xa``: end of the constant section (payload-bay aft frame); ``x_fw``: firewall; ``plug``: V-variant only."""

    def __init__(self, xa: float = 1.75, x_fw: float = 2.00, plug: float = 0.0):
        self.xa0, self.x_fw0 = xa, x_fw
        self.xa = xa + plug
        self.x_fw = max(x_fw + plug, self.xa + TRANS_MIN)
        self.plug = plug
        self.X_HUB = self.x_fw + ENGINE_BAY_LEN
        self.X_LIP = self.X_HUB + 0.05
        self.X_PROP = self.X_HUB + HUB_SPACER + 0.02
        self.SPINNER = {"x0": self.X_HUB + 0.07, "x1": self.X_PROP + 0.12, "d": 0.17}
        xe = self.X_HUB - 0.19
        F = np.array([
            # x,     w,     h,     zc,    n_top, n_bot, top_frac
            [0.00, 0.000, 0.000, -0.010, 1.5, 1.3, 0.50],
            [0.10, 0.130, 0.105, -0.004, 1.6, 1.35, 0.50],
            [0.30, 0.300, 0.275, 0.022, 1.9, 1.5, 0.44],
            [0.56, 0.415, 0.400, 0.058, 2.2, 1.6, 0.38],
            [0.95, 0.460, 0.450, 0.090, 2.4, 1.65, 0.36],
            [self.xa, 0.460, 0.450, 0.100, 2.5, 1.7, 0.34],
            [xe, 0.450, 0.470, Z_T, 3.2, 2.3, 0.26],
            [self.X_HUB - 0.05, 0.445, 0.470, Z_T, 3.4, 2.5, 0.26],
            [self.X_LIP, 0.300, 0.290, Z_T, 2.2, 2.2, 0.50],
        ])
        if self.xa <= 0.95 + 1e-6:
            F = np.delete(F, 4, axis=0)
        self.F = F
        self.fus = oml.Fuselage(F)
        self.L = float(F[-1, 0])
        self._props = None

    def section(self, x):
        x = np.atleast_1d(np.asarray(x, float))
        w, h, zc, nt, nb = self.fus.section(x)
        return w, h, zc, nt, nb, self.fus.top_frac(x)

    def z_top(self, x):
        w, h, zc, nt, nb, tf = self.section(x)
        return zc + tf * h

    def z_bot(self, x):
        w, h, zc, nt, nb, tf = self.section(x)
        return zc - (1 - tf) * h

    def internal_area(self, x, inset=TANK_INSET):
        w, h, zc, nt, nb, tf = (float(v[0]) for v in self.section(x))
        a = max(w / 2 - inset, 0.0)
        return EC.superellipse_half_area(a, max(tf * h - inset, 0), nt) + \
            EC.superellipse_half_area(a, max((1 - tf) * h - inset, 0), nb)

    def inside(self, x, y, z, margin=0.0) -> bool:
        w, h, zc, nt, nb, tf = (float(v[0]) for v in self.section(x))
        a = w / 2 - margin
        b, n = ((tf * h - margin, nt) if z >= zc else ((1 - tf) * h - margin, nb))
        if a <= 0 or b <= 0:
            return False
        return (abs(y) / a) ** n + (abs(z - zc) / b) ** n <= 1.0

    def width_at(self, x, z) -> float:
        w, h, zc, nt, nb, tf = (float(v[0]) for v in self.section(x))
        b, n = ((tf * h, nt) if z >= zc else ((1 - tf) * h, nb))
        u = min(abs(z - zc) / max(b, 1e-9), 1.0)
        return w * (1 - u ** n) ** (1 / n)

    def props(self) -> dict:
        if self._props is None:
            m = self.fus.mesh(160, 96)
            V, Fc = m.V, m.F
            A = 0.5 * np.linalg.norm(np.cross(V[Fc[:, 1]] - V[Fc[:, 0]], V[Fc[:, 2]] - V[Fc[:, 0]]), axis=1)
            C = V[Fc].mean(axis=1)
            cap = C[:, 0] > self.L - 1e-4
            area = float(A[~cap].sum())
            xc = float((A[~cap] * C[~cap, 0]).sum() / area)
            xs = np.linspace(0, self.L, 400)
            ws, hs, *_ = self.section(xs)
            self._props = {"S_wet": area, "x_centroid": xc, "volume": float(abs(m.volume())),
                           "S_side": float(np.trapz(hs, xs)), "S_plan": float(np.trapz(ws, xs)),
                           "A_max": max(self.internal_area(x, 0.0) for x in np.linspace(0.5, self.L - 0.1, 30)),
                           "w_max": float(ws.max()), "h_max": float(hs.max()), "mesh": m}
        return self._props


# =====================================================================================================================
# 4. wing, boom and tail geometry
# =====================================================================================================================
def wing_planform(d: Design, S: float) -> dict:
    """Constant-chord centre section to y_cs, linear taper to the tip; quarter-chord line straight and unswept."""
    b = math.sqrt(d.AR * S)
    s = b / 2
    ycs = min(d.y_cs, 0.8 * s)
    cr = (S / 2) / (ycs + (s - ycs) * (1 + d.taper_o) / 2)
    ct = d.taper_o * cr
    y = np.linspace(0.0, s, 4001)
    c = np.where(y <= ycs, cr, cr + (ct - cr) * (y - ycs) / (s - ycs))
    area = np.trapz(c, y)
    mac = float(np.trapz(c * c, y) / area)
    y_mac = float(np.trapz(c * y, y) / area)
    return {"S": S, "b": b, "s": s, "cr": cr, "ct": ct, "mac": mac, "y_mac": y_mac, "AR": d.AR, "taper": ct / cr,
            "y_cs": ycs, "outer_panel_span": s - ycs}


def chord_at(pf: dict, y):
    y = np.asarray(y, float)
    return np.where(y <= pf["y_cs"], pf["cr"], pf["cr"] + (pf["ct"] - pf["cr"]) * (y - pf["y_cs"]) / (pf["s"] - pf["y_cs"]))


def tscale_at(d: Design, pf: dict, y):
    y = np.asarray(y, float)
    return np.where(y <= pf["y_cs"], 1.0, 1.0 + (d.ts_tip - 1.0) * (y - pf["y_cs"]) / (pf["s"] - pf["y_cs"]))


def wing_sections(d: Design, pf: dict, x_le_root: float, i_w: float) -> list:
    s, ycs = pf["s"], pf["y_cs"]
    return [
        {"y": 0.0, "x_le": x_le_root, "z_le": Z_WING_ROOT, "chord": pf["cr"], "twist_deg": i_w, "airfoil": "nlf416",
         "thickness_scale": 1.0},
        {"y": ycs, "x_le": x_le_root, "z_le": Z_WING_ROOT, "chord": pf["cr"], "twist_deg": i_w, "airfoil": "nlf416",
         "thickness_scale": 1.0},
        {"y": s, "x_le": x_le_root + 0.25 * (pf["cr"] - pf["ct"]),
         "z_le": Z_WING_ROOT + (s - ycs) * math.tan(math.radians(d.dihedral_o)), "chord": pf["ct"],
         "twist_deg": i_w - d.washout, "airfoil": "nlf416", "thickness_scale": d.ts_tip},
    ]


def boom_geometry(d: Design, wsecs: list, pf: dict) -> dict:
    """Boom axis under the centre-section lower surface at y_boom (machined saddle at the spar stations)."""
    ls = oml.LiftingSurface(wsecs)
    p_lo = ls.surface_point(0, 0.30, "lower")                  # constant chord: same at y_boom
    z_axis = float(p_lo[2]) - 0.005 - d.d_boom / 2
    x_spar = wsecs[0]["x_le"] + 0.30 * pf["cr"]
    return {"y": d.y_boom, "z": z_axis, "r": d.d_boom / 2, "x_spar": x_spar,
            "x_rear_spar": wsecs[0]["x_le"] + 0.65 * pf["cr"],
            "x0": wsecs[0]["x_le"] - 0.22,                     # nose of the moulded boom fairing (UCAV-style chined)
            "x_tube0": x_spar - 0.05}                          # bought tube starts just ahead of the spar saddle


def htail_geometry(d: Design, pod: Pod, S_h: float, S_fins: float, boom: dict) -> dict:
    """Constant-chord stabiliser between the boom axes, two identical canted fins on the boom ends (TEs aligned)."""
    b_h = 2 * d.y_boom
    c_h = S_h / b_h
    x_le_h = pod.X_PROP + d.tail_gap
    x_te = x_le_h + c_h
    z_h = boom["z"]
    stab = [{"y": 0.0, "x_le": x_le_h, "z_le": z_h, "chord": c_h, "twist_deg": 0.0, "airfoil": "n0012"},
            {"y": d.y_boom, "x_le": x_le_h, "z_le": z_h, "chord": c_h, "twist_deg": 0.0, "airfoil": "n0012"}]
    S_f = S_fins / 2
    h_f = math.sqrt(d.fin_AR * S_f)
    c_fr = 2 * S_f / (h_f * (1 + d.fin_taper))
    c_ft = d.fin_taper * c_fr
    cant = math.radians(d.fin_cant)
    z_f0 = boom["z"] + boom["r"] - 0.005
    x_f0 = x_te - c_fr
    fin = [{"y": d.y_boom, "x_le": x_f0, "z_le": z_f0, "chord": c_fr, "twist_deg": 0.0, "airfoil": "n0012"},
           {"y": d.y_boom + h_f * math.sin(cant), "x_le": x_f0 + h_f * math.tan(math.radians(d.fin_le_sweep)),
            "z_le": z_f0 + h_f * math.cos(cant), "chord": c_ft, "twist_deg": 0.0, "airfoil": "n0012"}]
    fm = EC.surface_mac(fin)
    sweep_mt = math.degrees(math.atan(math.tan(math.radians(d.fin_le_sweep)) - 0.3 * (c_fr - c_ft) / h_f))
    sweep_half = math.degrees(math.atan(math.tan(math.radians(d.fin_le_sweep)) - 0.5 * (c_fr - c_ft) / h_f))
    x_end = max(x_te, x_f0 + c_fr) + 0.03
    return {"type": "H", "stab_secs": stab, "fin_secs": fin, "S_h": S_h, "S_fins": S_fins, "S_fin_each": S_f,
            "b_h": b_h, "c_h": c_h, "x_le_h": x_le_h, "x_te": x_te, "z_h": z_h, "x_ac_h": x_le_h + 0.25 * c_h,
            "mac_h": c_h, "h_fin": h_f, "c_fin_root": c_fr, "c_fin_tip": c_ft, "fin_mac": fm["mac"],
            "x_ac_v": fm["x_le_mac"] + 0.25 * fm["mac"], "z_mac_v": fm["z_mac"], "sweep_mt_fin_deg": sweep_mt,
            "sweep_half_fin_deg": sweep_half, "S_v_eff": S_fins * math.cos(cant) ** 2,
            "S_h_from_fins": S_fins * math.sin(cant) ** 2, "boom_x_end": x_end,
            "tip_y_fin": fin[1]["y"], "tip_z_fin": fin[1]["z_le"]}


def vtail_geometry(d: Design, pod: Pod, S_V: float, gamma_deg: float) -> dict:
    """Upright V-tail on the engine-bay hump ahead of the propeller (endurance-chain geometry rule)."""
    span2 = math.sqrt(d.tail_AR * S_V)
    s = span2 / 2
    cr = S_V / (s * (1 + d.tail_taper))
    ct = d.tail_taper * cr
    g = math.radians(gamma_deg)
    x_te_root = pod.X_PROP - 0.15
    x_le0 = x_te_root - cr
    z0 = float(pod.z_top(x_le0 + 0.4 * cr)[0]) - 0.02
    dx = s * math.tan(math.radians(d.tail_le_sweep))
    secs = [{"y": 0.06, "x_le": x_le0, "z_le": z0, "chord": cr, "twist_deg": 0.0, "airfoil": "n0012"},
            {"y": 0.06 + s * math.cos(g), "x_le": x_le0 + dx, "z_le": z0 + s * math.sin(g), "chord": ct,
             "twist_deg": 0.0, "airfoil": "n0012"}]
    tm = EC.surface_mac(secs)
    sweep_mt = math.degrees(math.atan(math.tan(math.radians(d.tail_le_sweep)) - (cr - ct) * 0.3 / tm["panel_span"]))
    return {"type": "V", "secs": secs, "S_V": S_V, "gamma_deg": gamma_deg, "cr": cr, "ct": ct, "macd": tm,
            "mac": tm["mac"], "x_ac": tm["x_le_mac"] + 0.25 * tm["mac"], "z_mac": tm["z_mac"],
            "span_panel": tm["panel_span"], "b_proj": 2 * secs[1]["y"], "sweep_mt_deg": sweep_mt}


@functools.lru_cache(maxsize=256)
def _stab_slope(b_h: float, c_h: float, V: float, h: float) -> float:
    secs = [{"y": 0.0, "x_le": 0.0, "z_le": 0.0, "chord": c_h, "twist_deg": 0.0, "airfoil": "n0012"},
            {"y": b_h / 2, "x_le": 0.0, "z_le": 0.0, "chord": c_h, "twist_deg": 0.0, "airfoil": "n0012"}]
    return float(AE.surface_analysis(secs, V, h)["CL_alpha"])


def stab_slope(t: dict, V: float, h: float) -> float:
    """Lifting-line slope of the constant-chord stabiliser (no end-plate credit for the fins: conservative)."""
    return _stab_slope(round(t["b_h"], 4), round(t["c_h"], 4), round(V, 2), round(h, 0))


def fin_slope(d: Design, t: dict, V: float, h: float) -> float:
    h_f = t["h_fin"]
    ar_eff = K_FIN_AR * h_f ** 2 / t["S_fin_each"]
    ch = AE.characteristics("n0012", AL.reynolds(V, t["fin_mac"], h))
    return AL.lift_slope(ar_eff, t["sweep_half_fin_deg"], cla_2d=ch["cl_alpha"])


def vtail_slope(t: dict, V: float, h: float) -> float:
    """Lifting-line slope of the V-panel pair (endurance chain: TAIL_SLOPE_OVERRIDE), used for pitch and yaw."""
    return float(AE.surface_analysis(t["secs"], V, h)["CL_alpha"])


# =====================================================================================================================
# 5. mass model
# =====================================================================================================================
FIXED_ITEMS_H = {
    "engine_group_installed": (SR["engine_group_installed"]["value"], "baseline.yaml#mass_targets.systems_rollup_kg"),
    "propeller": (SR["propeller"]["value"], "baseline.yaml#mass_targets.systems_rollup_kg"),
    "spinner_hub_adapter_and_80mm_spacer": (SR["spinner_and_hub_adapter"]["value"] + 0.10,
                                            "baseline 0.40 + 0.10 aluminium hub spacer (endurance chain)"),
    "baffles_ducts_firewall_cowl_flap": (SR["cowling_cooling_ducts_baffles_firewall"]["value"],
                                         "baseline.yaml (cowl skin counted in the pod shell)"),
    "fuel_system_two_identical_cells": (SR["fuel_system"]["value"] + 0.35,
                                        "baseline 1.73 + 0.35 kg for two interconnected cells (endurance chain); "
                                        "both cells have the same shape and part number"),
    "flight_control_actuators": (8 * 0.27 + 0.45 + 0.10,
                                 "8 x Volz DA 26 0.27 kg (2 ailerons, 2 elevator halves, 2 rudders, nose steering, "
                                 "brake) + 0.45 kg installation (baseline 0.35 share + 0.10 for the two extra tail "
                                 "channels) + 0.10 kg cowl-flap servo; no flaps; split elevator = no single servo "
                                 "failure leaves a free-floating pitch surface (CS-LUAS.629(f))"),
    "avionics": (SR["avionics"]["value"], "baseline.yaml#mass_targets.systems_rollup_kg"),
    "electrical_power": (SR["electrical_power"]["value"], "baseline.yaml#mass_targets.systems_rollup_kg"),
    "wiring_harness": (SR["wiring_harness_connectors_coax"]["value"] + 0.35,
                       "baseline 2.50 + 0.35 kg for the two boom runs to the tail servos (2 x 2.3 m shielded, one "
                       "connector at each boom joint; estimate)"),
    "recovery_and_safety": (SR["recovery_and_safety"]["value"], "baseline.yaml#mass_targets.systems_rollup_kg"),
}
FIXED_ITEMS_V = dict(FIXED_ITEMS_H)
FIXED_ITEMS_V["flight_control_actuators"] = (6 * 0.27 + 0.35 + 0.10, "6 x Volz DA 26 (2 ailerons, 2 ruddervators, "
                                             "steering, brake) + 0.35 installation + 0.10 cowl flap (endurance chain)")
FIXED_ITEMS_V["wiring_harness"] = (SR["wiring_harness_connectors_coax"]["value"] + 0.20, "baseline 2.50 + 0.20 kg")
FLAP_KIT = rec("flap_kit_kg", 2 * 0.63 + 0.10 + 2 * 0.10, "baseline.yaml#subsystems.flight_control_actuators.flaps",
               "datasheet", basis="2 x Volz DA 30 0.63 kg + 0.10 kg installation + 2 x 0.10 kg hinges/horns (trade only)")
rec("fixed_equipment_items_kg", {k: v for k, (v, _) in FIXED_ITEMS_H.items()},
    "baseline.yaml#mass_targets.systems_rollup_kg (landing gear in the airframe group)",
    basis="; ".join(f"{k}: {s}" for k, (_, s) in FIXED_ITEMS_H.items()))


def fixed_items(d: Design) -> dict:
    return FIXED_ITEMS_H if d.tail == "H" else FIXED_ITEMS_V


def fixed_mass(d: Design) -> float:
    m = sum(v for v, _ in fixed_items(d).values())
    if d.flaps:
        m += FLAP_KIT
    if EC.PROP_KEY != "0161":
        m += EC.PROP_MASS_DELTA
    return m


def gust_limit(d: Design, cla: float, mac: float, clmax: float) -> dict:
    return EC.gust_limit(d.ws, cla, mac, clmax, 57.0)


def wing_mass(d: Design, pf: dict, m0: float, n_lim: float, wsecs: list, w_pod: float) -> dict:
    """Spar caps (UD CFRP) from the Schrenk bending moment at the ultimate gust/manoeuvre factor (no inertia relief),
    +/-45 webs, rear spar, ribs, OOA-prepreg sandwich skins from the actual surface, joints and fittings. The centre
    spar is continuous across the pod; the outer panels join at y_cs with a tongue + 2 pins + drag pin."""
    s = pf["s"]
    y = np.linspace(0.0, s, 161)
    c = chord_at(pf, y)
    tc = TC_ROOT * tscale_at(d, pf, y)
    l = ST.schrenk(y, c, s)
    w = FOS * n_lim * m0 * G / 2 * l
    Vs, Ms = ST.beam_loads(y, w)
    h_eff = np.maximum(0.95 * tc * c - 0.010, 0.012)
    A_cap = np.maximum(Ms / (h_eff * SIG_CAP), 40e-6)
    caps = 2 * 2 * np.trapz(A_cap, y) * UD["density"] * 1.10
    t_web = np.maximum(Vs / (h_eff * TAU_WEB), 0.6e-3)
    web = 2 * np.trapz(t_web * h_eff, y) * PW["density"] * 1.25
    rear = 2 * np.trapz(0.6e-3 * 0.55 * tc * c + 2 * 20e-6 * UD["density"] / PW["density"], y) * PW["density"]
    # ribs: centre section at the pod sides, booms (2 each) and the joint (2); outer panels every 0.35 m
    y_ribs = [w_pod / 2, d.y_boom - 0.06, d.y_boom + 0.06, pf["y_cs"] - 0.01, pf["y_cs"] + 0.01]
    n_out = int(math.ceil((s - pf["y_cs"]) / 0.35))
    y_ribs += list(np.linspace(pf["y_cs"], s, n_out + 1)[1:])
    y_ribs = np.array(y_ribs)
    rib_area = 0.68 * np.interp(y_ribs, y, tc * c * c) * 0.6
    ribs = 2 * float(rib_area.sum()) * RIB_AREAL
    surf = oml.LiftingSurface(wsecs, n_chord=60).mesh()
    S_wet_half = surf.area() - 2 * 0.68 * TC_ROOT * pf["cr"] ** 2 * 0.5
    skins = 2 * S_wet_half * SKIN_PRIMARY
    joints = {"outer_panel_joints_2x": 2 * 0.80, "centre_section_to_chassis_4_fittings": 0.50,
              "boom_saddles_on_spars_2x": 2 * 0.30}
    hinges = 2 * 0.10 + (2 * 0.10 if d.flaps else 0.0)
    sub = caps + web + rear + ribs + skins + sum(joints.values()) + hinges
    total = sub * 1.05
    i_cs = int(np.argmin(np.abs(y - pf["y_cs"])))
    return {"total": total, "caps": caps, "webs": web, "rear_spar": rear, "ribs": ribs, "skins": skins,
            "joints": joints, "hinges": hinges, "S_wet": 2 * S_wet_half, "M_root_ult_Nm": float(Ms[0]),
            "M_joint_ult_Nm": float(Ms[i_cs]), "V_joint_ult_N": float(Vs[i_cs]),
            "A_cap_root_mm2": float(A_cap[0] * 1e6), "h_eff_root_m": float(h_eff[0]), "n_ult": FOS * n_lim}


def surface_mass(secs: list, n_ribs: int, fittings: float, mirror: bool = True) -> dict:
    """Tail surface: sandwich skins (SKIN_TAIL) on the actual surface, UD-capped spar, ribs, fittings/hinges."""
    surf = oml.LiftingSurface(secs, n_chord=40).mesh()
    k = 2 if mirror else 1
    S_wet = k * surf.area()
    span = EC.surface_mac(secs)["panel_span"]
    spar = k * span * (2 * 30e-6 * UD["density"] + 0.6e-3 * 0.05 * PW["density"])
    ribs = k * n_ribs * 0.004 * RIB_AREAL
    total = (S_wet * SKIN_TAIL + spar + ribs + fittings) * 1.05
    return {"total": total, "skins": S_wet * SKIN_TAIL, "spar": spar, "ribs": ribs, "fittings": fittings,
            "S_wet": S_wet}


def tail_loads_limit(d: Design, t: dict, VD: float = 57.0) -> dict:
    """Limit tail loads for the boom sizing: stabiliser at q(VD) x CL 1.0 (full elevator, or a 15.24 m/s gust at VC,
    the larger), fin at q(VD) x CL 1.0 (estimate: conservative envelope of rudder kick and lateral gust)."""
    qD = 0.5 * RHO0 * VD ** 2
    a_h = t.get("a_t", 4.0)
    L_gust_h = 0.5 * RHO0 * VC_EAS * EC.UDE_C * a_h * t["S_h"]
    L_h = max(qD * t["S_h"] * 1.0, L_gust_h)
    L_v = qD * t["S_fin_each"] * 1.0
    return {"L_h_total_N": L_h, "L_fin_each_N": L_v, "q_D_Pa": qD}


def boom_design(d: Design, t: dict, boom: dict, tail_mass_each: float) -> dict:
    """CFRP tube sized for bending (stabiliser half + fin) and torsion (fin side load x arm) at ultimate, with the
    damage-tolerance strain limits; wall >= the bought-tube minimum. First bending frequency with the tail mass."""
    tl = tail_loads_limit(d, t)
    L_b = t["boom_x_end"] - boom["x_tube0"]
    arm = t["x_ac_h"] - boom["x_rear_spar"]
    arm_v = t["x_ac_v"] - boom["x_rear_spar"]
    M_v = tl["L_h_total_N"] / 2 * arm
    M_l = tl["L_fin_each_N"] * arm_v
    M_ult = FOS * math.hypot(M_v, M_l)
    T_ult = FOS * tl["L_fin_each_N"] * (t["z_mac_v"] - boom["z"])
    E_lam = 0.6 * UD["E"] + 0.4 * MAT["cfrp_qi_laminate_design_values"]["E_qi_Pa"]["value"]
    G_lam = MAT["cfrp_qi_laminate_design_values"]["G_qi_Pa"]["value"]
    sig = E_lam * 3000e-6
    tau = G_lam * 5200e-6
    r = boom["r"]
    t_b = max(d.t_boom_min, M_ult / (math.pi * r ** 2 * sig), T_ult / (2 * math.pi * r ** 2 * tau))
    rho = 0.6 * UD["density"] + 0.4 * PW["density"]
    m_tube = math.pi * 2 * r * t_b * L_b * rho
    EI = E_lam * math.pi * r ** 3 * t_b
    k = 3 * EI / arm ** 3
    f1 = math.sqrt(k / (tail_mass_each + 0.24 * m_tube * arm / L_b)) / (2 * math.pi)
    fittings = 0.10 + 0.25 + 0.10 + 0.05          # front clamp collar, tail fitting (stab + fin spars), moulded nose
    return {"L_m": L_b, "t_wall_m": t_b, "mass_tube_kg": m_tube, "fittings_kg": fittings,   # fairing, bumper skid
            "mass_each_kg": m_tube + fittings, "M_ult_Nm": M_ult, "T_ult_Nm": T_ult, "EI_Nm2": EI,
            "f1_bending_Hz": f1, "tip_deflection_limit_m": (M_v / arm) * arm ** 3 / (3 * EI), "loads": tl,
            "allowables_Pa": {"sigma": sig, "tau": tau}}


def chassis_items(d: Design, pod: Pod, x_spar: float, x_rear: float, x_mg: float, tank: dict) -> dict:
    """Bottom-up chassis, (mass kg, x m) per item. Aluminium (this concept): 6061-T6 extruded longerons, three heavy
    bulkheads (wing front / wing rear + gear / firewall), light formed 2024-T3 ring frames, 0.5 mm liners closing the
    two fuel-cell compartments, payload-bay rails, 2024-T3 shear webs closing the centre torsion box, machined 7075
    hard points, 4130 engine truss. CFRP alternative (trade): the endurance-chain keel beams + sandwich bulkheads."""
    L_ch = pod.x_fw - 0.50
    xm = 0.5 * (0.50 + pod.x_fw)
    A_sec = pod.internal_area(1.2, 0.0)
    per = math.pi * (0.46 + 0.45) / 2 * 1.05
    per_in = per * 0.90
    fw, af = tank["cells"]["forward"], tank["cells"]["aft"]
    light_x = [0.55, X_NG, X_PARACHUTE[1], fw["x0"], tank["bay"][0], tank["bay"][1], af["x1"]]
    light_x += list(np.arange(af["x1"] + 0.5, pod.x_fw - 0.1, 0.5))        # V-variant plug: one frame per 0.5 m
    heavy_x = [x_spar, x_rear, pod.x_fw]
    if d.chassis == "aluminium":
        rho6, rho2 = AL6061["density"], AL2024["density"]
        bulk = A_sec * 0.5 * 1.0e-3 * rho6 + per * 0.020 * 1.0e-3 * rho6       # 1.0 mm, 50 % lightened, flanged
        ring = per * (0.025 + 2 * 0.015) * 0.6e-3 * rho2                      # 0.6 mm Z-ring, 25 mm web
        L_cells = fw["length"] + af["length"]
        items = {
            "lower_longerons_2x_6061_angle_25x25x1p6": (2 * L_ch * 77.4e-6 * rho6, xm),
            "upper_longerons_2x_6061_angle_20x20x1p6": (2 * L_ch * 61.4e-6 * rho6, xm),
            "heavy_bulkheads_3x_6061_1mm_lightened": (3 * bulk, float(np.mean(heavy_x))),
            f"light_ring_frames_{len(light_x)}x_2024T3_0p6mm": (len(light_x) * ring, float(np.mean(light_x))),
            "fuel_compartment_liners_6061_0p5mm": (per_in * L_cells * 0.5e-3 * rho6, tank["x_c"]),
            "payload_bay_rails_and_door_frame": (0.40, 0.5 * sum(tank["bay"])),
            "equipment_shelves_nose_and_systems_bay": (0.50, 0.75),
            "side_shear_webs_2024T3_0p6mm_torsion_box": (2 * 0.6e-3 * rho2 * (x_rear - x_spar + 0.6) * 0.35,
                                                         0.5 * (x_spar + x_rear)),
            "wing_attach_fittings_7075_4x": (0.80, 0.5 * (x_spar + x_rear)),
            "main_gear_bow_saddle_7075": (0.60, x_mg),
            "nose_gear_trunnion_7075": (0.30, X_NG),
            "engine_mount_4130_truss": (0.60, pod.X_HUB - 0.20),
            "parachute_hard_point": (0.35, sum(X_PARACHUTE) / 2),
            "quarter_turn_receptacles_and_hatch_frames": (0.86, 1.2),
            "rivets_bolts_sealant": (0.40, xm),
        }
    else:
        items = {
            "keel_beams_2x_cfrp_hat_0p20kg_per_m": (2 * L_ch * 0.20, xm),
            f"frames_{len(light_x) + len(heavy_x)}x_sandwich_bulkheads": ((len(light_x) + len(heavy_x)) * 0.16,
                                                                          float(np.mean(light_x + heavy_x))),
            "wing_attach_fittings_7075_ti_pins": (1.20, 0.5 * (x_spar + x_rear)),
            "main_gear_saddle_clamps_7075": (0.60, x_mg),
            "nose_gear_trunnion_fitting": (0.30, X_NG),
            "engine_mount_frame_4130_truss": (0.60, pod.X_HUB - 0.20),
            "parachute_hard_point_tray": (0.35, sum(X_PARACHUTE) / 2),
            "floors_trays_equipment_rails": (1.20, 1.10),
            "hatch_frames_quick_release_fasteners": (0.80, 1.20),
            "fuel_bay_liner_supports": (0.40, tank["x_c"]),
        }
    if d.tail == "V":
        items["v_tail_root_fittings"] = (0.30, pod.X_HUB - 0.30)
    return items


def pod_mass(d: Design, pod: Pod, x_spar, x_rear, x_mg, tank) -> dict:
    fp = pod.props()
    shell = fp["S_wet"] * SKIN_SECONDARY * (1 + SHELL_EDGE)
    items = chassis_items(d, pod, x_spar, x_rear, x_mg, tank)
    chassis = sum(m for m, _ in items.values())
    return {"shell": shell, "chassis": chassis, "chassis_items": items, "S_wet": fp["S_wet"]}


def gear_mass(d: Design, m0: float) -> float:
    return EC.gear_mass(m0) - (0.0 if d.spats else SPATS_MASS)


def chassis_check(d: Design, items: list, x_spar: float, x_rear: float, n_ult: float, pod: Pod) -> dict:
    """Pod as a beam on the two wing-attach frames under n_ult x the item weights (fuel, payload, equipment, shell):
    bending moment -> longeron force couple -> stress vs 6061-T6 column allowable between frames (0.30 m pitch)."""
    xs = np.linspace(0.0, pod.X_HUB, 400)
    q = np.zeros_like(xs)
    for name, m, x, _ in items:
        if name.startswith(("wing", "boom", "htail", "vtail", "actuators_wing", "actuators_tail", "lights")):
            continue
        i = int(np.clip(np.searchsorted(xs, x), 0, len(xs) - 1))
        q[i] += m * G * n_ult
    M = np.zeros_like(xs)
    for i, x in enumerate(xs):          # cantilever moments outside the supports (conservative for the bay between)
        if x < x_spar:
            M[i] = float(np.sum(q[:i] * (x - xs[:i])))
        elif x > x_rear:
            M[i] = float(np.sum(q[i:] * (xs[i:] - x)))
    M_max = float(np.max(np.abs(M)))
    h_box = 0.36
    F = M_max / h_box
    A_pair = 2 * (77.4e-6 + 61.4e-6) / 2
    r_gyr = 0.0050
    sig_cr = math.pi ** 2 * AL6061["E"] / (0.30 / r_gyr) ** 2
    sig_allow = min(AL6061["Fty"], sig_cr)
    sig = F / A_pair
    return {"M_max_ult_Nm": M_max, "longeron_pair_force_N": F, "stress_Pa": sig, "allowable_Pa": sig_allow,
            "MS": sig_allow / sig - 1 if sig > 0 else math.inf, "n_ult": n_ult,
            "note": "minimum-gauge longerons; the margin shows the chassis is gauge- not strength-governed"}


# =====================================================================================================================
# 6. aerodynamics of one configuration
# =====================================================================================================================
@dataclass
class Aircraft:
    d: Design
    m0: float
    pf: dict
    wing_secs: list
    tail: dict
    pod: Pod
    boom: dict | None
    sa: dict
    S: float
    layout: dict
    mass: dict
    cd_table: tuple = None
    fit: dict = None
    clmax: dict = None
    drag_items: dict = field(default_factory=dict)
    flap_to: dict | None = None
    aero_cm0: float = 0.0

    def CD(self, CL):
        return np.interp(CL, self.cd_table[0], self.cd_table[1])


def slipstream(V: float, T: float, x_behind: float, h: float) -> dict:
    """Actuator-disk slipstream at x_behind the propeller plane: velocity, dynamic pressure and radius."""
    rho = AL.isa(h)["rho"]
    R = EC.D_PROP / 2
    w0 = 0.5 * (-V + math.sqrt(V * V + 2 * max(T, 0.0) / (rho * A_DISC)))
    wx = w0 * (1 + x_behind / math.sqrt(R * R + x_behind * x_behind))
    r = R * math.sqrt((V + w0) / max(V + wx, 1e-6))
    return {"w_disk": w0, "w": wx, "q_s": 0.5 * rho * (V + wx) ** 2, "q": 0.5 * rho * V * V, "r": r}


def stab_immersion(t: dict, r_s: float) -> float:
    dz = abs(t["z_h"] - Z_T)
    if dz >= r_s:
        return 0.0
    return min(1.0, 2 * math.sqrt(r_s * r_s - dz * dz) / t["b_h"])


def tail_q_ratio(ac_or_t, pod: Pod, V: float, T: float, h: float) -> tuple:
    t = ac_or_t
    if t["type"] != "H":
        return 1.0, 0.0
    x_b = t["x_le_h"] + 0.5 * t["c_h"] - pod.X_PROP
    ss = slipstream(V, T, x_b, h)
    f = stab_immersion(t, ss["r"])
    return (1 - f) + f * ss["q_s"] / max(ss["q"], 1e-9), f


def flap_effect(d: Design, pf: dict, w_pod: float) -> dict | None:
    """Trade only: plain take-off flaps on the centre section, split at the boom (Raymer 6th ed. eqs. 12.21, 12.22,
    12.61; same constants as the endurance chain)."""
    if not d.flaps:
        return None
    gap = d.d_boom + 0.06
    spans = [(w_pod / 2, d.y_boom - gap / 2), (d.y_boom + gap / 2, pf["y_cs"])] if d.tail == "H" else \
        [(w_pod / 2, pf["y_cs"])]
    r = sum(2 * (b_ - a_) * pf["cr"] for a_, b_ in spans if b_ > a_) / pf["S"]
    f = min(max(d.flap_deg, 0.0) / 40.0, 1.0)
    return {"delta_deg": d.flap_deg, "S_flapped_over_S": r, "dCLmax": 0.9 * 0.9 * f * r, "dalpha0_deg": -15.0 * f * r,
            "dCD0": 0.0144 * 0.25 * r * max(d.flap_deg - 10.0, 0.0)}


def drag_buildup(ac: Aircraft, V: float, h: float, fuel_flow_ratio: float) -> dict:
    """Parasite items other than the wing profile drag, as D/q (m2) at TAS V and altitude h (aerolib)."""
    d, pf, pod, t, lay = ac.d, ac.pf, ac.pod, ac.tail, ac.layout
    atm = AL.isa(h)
    M = V / atm["a"]
    fp = pod.props()
    items = {}
    S_wet_f = fp["S_wet"] - pf["cr"] * 0.30
    d_eq = math.sqrt(4 * fp["A_max"] / math.pi)
    items["pod"] = AL.cf_flat(AL.reynolds(V, pod.L, h), 0.0) * AL.ff_body(pod.L / d_eq) * S_wet_f
    xe = pod.X_HUB - 0.05
    u = math.atan2(float(pod.z_bot(xe)[0] - pod.z_bot(pod.xa)[0]), xe - pod.xa)
    items["pod_aft_upsweep"] = 3.83 * abs(u) ** 2.5 * fp["A_max"]
    if t["type"] == "H":
        b = ac.boom
        L_b = t["boom_x_end"] - b["x0"]                   # wetted: fairing nose to tail fitting
        L_under = pf["cr"] + 0.22
        Q_b = (1.5 * L_under + 1.0 * (L_b - L_under)) / L_b
        Sw_b = math.pi * d.d_boom * L_b
        items["booms_2x"] = 2 * AL.cf_flat(AL.reynolds(V, L_b, h)) * AL.ff_body(L_b / d.d_boom) * Q_b * Sw_b
        S_exp_h = t["c_h"] * (t["b_h"] - d.d_boom)
        cd_h = AL.cf_flat(AL.reynolds(V, t["c_h"], h)) * AL.ff_wing(0.12, 0.30, 0.0, M) * 1.08 * \
            AL.wing_wetted(S_exp_h, 0.12)
        items["stabiliser"] = cd_h
        items["fins_2x"] = 2 * AL.cf_flat(AL.reynolds(V, t["fin_mac"], h)) * \
            AL.ff_wing(0.12, 0.30, t["sweep_mt_fin_deg"], M) * 1.08 * AL.wing_wetted(t["S_fin_each"], 0.12)
        T_ref = ac.m0 * G / 15.0
        qr, f_imm = tail_q_ratio(t, pod, V, T_ref, h)
        items["stabiliser_slipstream_scrubbing"] = cd_h * (qr - 1.0)
    else:
        S_exp_t = t["S_V"] - 2 * 0.5 * t["cr"] * 0.02
        items["v_tail"] = AL.cf_flat(AL.reynolds(V, t["mac"], h)) * AL.ff_wing(0.12, 0.30, t["sweep_mt_deg"], M) * \
            1.03 * AL.wing_wetted(S_exp_t, 0.12)
    g = lay["gear"]
    A_wheel = WHEEL_D * WHEEL_W
    leg_main = g["h_gear"] * 1.35 * 0.035
    leg_nose = g["h_nose_leg"] * 0.040
    if d.spats:
        items["landing_gear"] = 1.2 * (2 * 0.13 * A_wheel + 2 * 0.05 * leg_main + 0.35 * A_wheel + 0.05 * leg_nose)
    else:
        items["landing_gear"] = 1.2 * (2 * 0.25 * A_wheel + 2 * 0.05 * leg_main + 0.25 * A_wheel + 1.0 * 0.0045 +
                                       0.05 * leg_nose)
    items["eo_ir_turret"] = 0.47 * TUR_D * 0.12
    m_dot = EN["installation"]["cooling"]["cooling_airflow_estimate"]["mass_flow_kg_per_s_at_max_power"]["value"] * \
        fuel_flow_ratio
    items["engine_cooling"] = m_dot * V * 0.5 / (0.5 * atm["rho"] * V * V)
    items["antennas_pitot_lights_vents"] = 0.0025
    n_pairs = 3 if t["type"] == "H" else 2
    items["control_surface_gaps"] = 0.0002 * pf["S"] * (n_pairs + (1 if d.flaps else 0))
    base = sum(items.values())
    items["leakage_protuberance"] = LEAK_FRAC * base
    return items


def trimmed_polar(ac: Aircraft, aero: dict, cd_rest: float, cg: tuple) -> tuple:
    """CD(CL) with the tail trim load (Cm0, CG, thrust-line moment) and tail induced drag (endurance-chain form)."""
    pf, lay = ac.pf, ac.layout
    x_cg, z_cg = cg
    mac = pf["mac"]
    k_w = 1.0 / (math.pi * pf["AR"] * aero["e_inv"])
    exp_frac = 1.0 - 0.5 * pf["cr"] * lay["w_pod"] / pf["S"]
    arm_t = lay["x_ac_t"] - x_cg
    b_t = lay["tail_span_induced"]
    CLs = np.linspace(-0.2, 1.75, 79)
    out, cltS = [], []
    for CL in CLs:
        CLw = CL
        for _ in range(6):
            cdp = aero["cdp"](min(CLw, 1.9)) * K_CD_TRIP * exp_frac
            cd_w = cdp + k_w * CLw ** 2
            cm = aero["cm0"] + CLw * (x_cg - lay["x_ac_w"]) / mac - (cd_rest + cd_w) * (Z_T - z_cg) / mac
            clt = cm * mac / arm_t
            CLw = CL - clt
        cdi_t = (clt * pf["S"]) ** 2 / (math.pi * b_t ** 2 * 0.8) / pf["S"]
        out.append(cd_rest + cd_w + cdi_t)
        cltS.append(clt)
    return CLs, np.array(out), np.array(cltS)


# =====================================================================================================================
# 7. layout: items, CG, tanks, ground geometry
# =====================================================================================================================
def payload_items(kind: str, x_bay: float) -> list:
    tur = [("payload_eo_ir_turret", M_TURRET, X_TURRET, -0.24)]
    base = tur + [("payload_mission_computer", M_MCOMP, x_bay - 0.12, 0.08), ("payload_tray", M_TRAY, x_bay, -0.15)]
    if kind == "turret_only":
        return tur
    if kind == "baseline_set":
        return base
    return base + [("payload_research_allowance", M_RESEARCH, x_bay, -0.08)]


def cg_of(items, fuel_kg, tank, payload_kind, fuel_fraction):
    its = list(items) + payload_items(payload_kind, sum(tank["bay"]) / 2)
    its.append(("fuel", fuel_kg * fuel_fraction, tank["x_c"], tank["z_c"]))
    m = sum(i[1] for i in its)
    return m, sum(i[1] * i[2] for i in its) / m, sum(i[1] * i[3] for i in its) / m


def tank_layout(pod: Pod, fuel_kg: float, x_center: float) -> dict:
    """Payload bay centred at ``x_center``, one fuel cell ahead and one behind (two identical interconnected
    bladders feeding a header): the fuel CG stays at the bay centroid (endurance-chain rule)."""
    vol_req = fuel_kg / RHO_FUEL * 1.02
    xb0, xb1 = x_center - BAY_LEN / 2 - 0.015, x_center + BAY_LEN / 2 + 0.015

    def cell(x_start, sign):
        L = 0.05
        while True:
            xs = np.linspace(x_start, x_start + sign * L, 30)
            A = np.array([pod.internal_area(x) for x in xs])
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


def ground_geometry(ac_parts: dict, aero: dict, cases: list) -> dict:
    """Endurance-chain ground rules (tip-back 15 deg or design attitude + 2 deg, propeller clearance level / at
    lift-off / at the design attitude, turret growth clearance, spring-bow travel, turnover <= 55 deg) with the boom
    tail bumpers (H) or a ventral bumper (V)."""
    pod, t, boom = ac_parts["pod"], ac_parts["tail"], ac_parts["boom"]
    xs = [c["x"] for c in cases]
    x_aft, x_fwd = max(xs), min(xs)
    z_cg = max(c["z"] for c in cases)
    CLa, clmax = aero["CL_alpha"], aero["CLmax_wing"]
    th_lof = max(math.degrees((clmax / 1.1 ** 2 - aero["CL_0"]) / CLa), 0.0)
    th_td = max(math.degrees((clmax / 1.15 ** 2 - aero["CL_0"]) / CLa), 0.0)
    th_flare = max(th_td, 4.0)
    th_design = th_flare + 3.0
    z_tur_bot = float(pod.z_bot(X_TURRET)[0]) - 0.03 - TUR_H_GROWTH * 0.45
    R = EC.D_PROP / 2
    X_PROP = pod.X_PROP
    if t["type"] == "H":
        x_bump, z_bump = t["boom_x_end"] - 0.02, boom["z"] - boom["r"] - 0.03
    else:
        x_bump = pod.X_HUB - 0.22
        z_bump = float(pod.z_bot(x_bump)[0]) - 0.03
    z_g = -0.40
    for _ in range(60):
        h_cg = z_cg - z_g
        tb = max(15.0, th_design + 2.0)
        x_mg = x_aft + h_cg * math.tan(math.radians(tb))
        arm = X_PROP - x_mg
        req = [
            Z_T - R - CLEAR_PROP_GROUND,
            Z_T - R - (CLEAR_PROP_GROUND + arm * math.sin(math.radians(th_lof))) / math.cos(math.radians(th_lof)),
            Z_T - R - (0.05 + arm * math.sin(math.radians(th_design))) / math.cos(math.radians(th_design)),
            z_tur_bot - 0.12,
            float(pod.z_bot(x_mg)[0]) - GEAR_TRAVEL,
            z_bump - (0.03 + (x_bump - x_mg) * math.sin(math.radians(th_design))) / math.cos(math.radians(th_design)),
        ]
        z_new = min(req)
        if abs(z_new - z_g) < 1e-7:
            break
        z_g = z_new
    names = ["prop_level", "prop_liftoff", "prop_flare_design", "turret", "gear_travel", "tail_bumper_design"]
    active = names[int(np.argmin(req))]
    h_cg = z_cg - z_g
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
    h_gear = float(pod.z_bot(x_mg)[0]) - z_g - WHEEL_D / 2
    return {"x_mg": x_mg, "z_g": z_g, "h_cg": h_cg, "tipback_deg": tb, "theta_lof_deg": th_lof, "theta_td_deg": th_td,
            "theta_flare_deg": th_flare, "theta_design_deg": th_design, "track": track, "turnover_deg": psi,
            "nose_load_aft_cg": nose_aft, "nose_load_fwd_cg": nose_fwd, "prop_clear_level": Z_T - R - z_g,
            "prop_clear_lof": clear_at(th_lof), "prop_clear_design": clear_at(th_design), "prop_strike_deg": th_prop,
            "bumper_contact_deg": th_bump, "x_bumper": x_bump, "z_bumper": z_bump, "h_gear": max(h_gear, 0.05),
            "h_nose_leg": float(pod.z_bot(X_NG)[0]) - z_g - WHEEL_D / 2, "turret_clear": z_tur_bot - z_g,
            "wheelbase": x_mg - X_NG, "active_height_rule": active,
            "fuselage_bottom_height_at_mg": float(pod.z_bot(x_mg)[0]) - z_g}


def component_list(d, pf, wsecs, t, boom, masses, gearpos, tank, pod):
    """(name, mass, x, z) of every empty-aircraft item (payload and fuel are added per loading case)."""
    wm = EC.surface_mac(wsecs)
    xw = wm["x_le_mac"] + 0.42 * wm["mac"]
    zw = Z_WING_ROOT + 0.03
    x_ail = wm["x_le_mac"] + 0.80 * wm["mac"]
    fx = fixed_items(d)
    items = [
        ("wing_structure", masses["wing"], xw, zw),
        ("pod_shell", masses["pod_shell"], pod.props()["x_centroid"], 0.03),
        ("landing_gear_main", masses["gear"] - 0.365 - 0.45 - 0.08 - 0.9, gearpos["x_mg"], gearpos["z_g"] + 0.15),
        ("landing_gear_nose", 0.365 + 0.45 + 0.08 + 0.9, X_NG, gearpos["z_g"] + 0.17),
        ("engine_group", fx["engine_group_installed"][0], pod.X_HUB - 0.12, Z_T - 0.05),
        ("propeller", fx["propeller"][0], pod.X_PROP, Z_T),
        ("spinner_hub_spacer", fx["spinner_hub_adapter_and_80mm_spacer"][0], pod.X_PROP - 0.01, Z_T),
        ("baffles_ducts_firewall", fx["baffles_ducts_firewall_cowl_flap"][0], pod.X_HUB - 0.15, Z_T - 0.05),
        ("fuel_system", fx["fuel_system_two_identical_cells"][0], tank["x_c"], -0.05),
        ("actuators_wing_2", 2 * 0.27 + 0.15, x_ail, zw),
        ("actuators_steer_brake_cowlflap", 2 * 0.27 + 0.10 + 0.15, (X_NG + gearpos["x_mg"]) / 2, -0.10),
        ("avionics", fx["avionics"][0], X_AVION, 0.05),
        ("electrical_battery_pdu", fx["electrical_power"][0], X_BATT, 0.00),
        ("wiring_harness", fx["wiring_harness"][0], 1.30, 0.08),
        ("parachute_fts", 5.9 + 0.15, sum(X_PARACHUTE) / 2, 0.15),
        ("lights", 0.25, xw - 0.1, zw),
    ]
    if t["type"] == "H":
        bd = masses["boom_design"]
        xb = boom["x_tube0"] + bd["L_m"] / 2
        items += [("booms_2x", 2 * bd["mass_each_kg"], xb, boom["z"]),
                  ("htail_stabiliser", masses["stab"], t["x_le_h"] + 0.40 * t["c_h"], t["z_h"]),
                  ("htail_fins_2x", masses["fins"], t["x_ac_v"] + 0.15 * t["fin_mac"], t["z_mac_v"]),
                  ("actuators_tail_4", 4 * 0.27 + 0.15, t["x_le_h"] + 0.6 * t["c_h"], t["z_h"])]
    else:
        items += [("vtail_structure", masses["vtail"], t["x_ac"] + 0.15 * t["mac"], t["z_mac"]),
                  ("actuators_tail_2", 2 * 0.27 + 0.05, t["x_ac"] + 0.35 * t["mac"], t["z_mac"])]
    if d.flaps:
        items.append(("flap_kit", FLAP_KIT, wsecs[0]["x_le"] + 0.85 * pf["cr"], zw))
    if EC.PROP_KEY != "0161":
        items.append(("propeller_delta_3B", EC.PROP_MASS_DELTA, pod.X_PROP, Z_T))
    for name, (m, x) in masses["chassis_items"].items():
        items.append(("chassis_" + name, m, x, 0.0))
    empty_est = sum(m for _, m, _, _ in items)
    xe = sum(m * x for _, m, x, _ in items) / empty_est
    ze = sum(m * z for _, m, _, z in items) / empty_est
    items.append(("mass_growth_allowance", GROWTH * empty_est, xe, ze))
    return items


# =====================================================================================================================
# 8. configuration solve (geometry <-> masses <-> CG <-> stability <-> gear <-> tanks <-> pod length)
# =====================================================================================================================
def cnb_pod(pod: Pod, S: float, b: float) -> float:
    fp = pod.props()
    return min(-1.3 * fp["volume"] / (S * b) * (fp["h_max"] / fp["w_max"]),
               SL.cn_beta_fuselage(0.0015, 1.75, fp["S_side"], pod.L, S, b))


def solve_configuration(d: Design, m0: float, fuel_kg: float, V_ref: float, h_ref: float = H_LOITER,
                        verbose: bool = False, init: dict | None = None):
    """Wing station for the static-margin target (secant on x_w), tail sized from V_H and Cn_beta at the current arm,
    masses, CG cases, neutral point, gear, fuel cells around the payload bay at the MTOW CG, pod length from the
    aft cell (firewall clearance). ``init``: warm start from a previous layout."""
    S = m0 * G / d.ws
    pf = wing_planform(d, S)
    secs0 = wing_sections(d, pf, 1.3, 0.0)
    a0 = EC.build_aero(d, pf, secs0, V_ref, h_ref, 0.0)
    q_floor = 0.5 * RHO0 * V_LOIT_MIN_EAS ** 2
    CL_mid = 0.85 * m0 * G / (q_floor * S)
    i_level = math.degrees(a0["sa"]["alpha_for"](min(CL_mid, 1.1)))
    i_td = math.degrees((a0["CLmax_wing"] / 1.15 ** 2 - a0["CL_0"]) / a0["CL_alpha"]) - THETA_TD_TARGET
    i_w = math.floor(min(i_level, i_td) * 4) / 4
    plug = d.v_plug if d.tail == "V" else 0.0
    if init:
        x_w = init["x_w"]
        pod = Pod(init["xa0"], init["x_fw0"], plug)
        S_h, S_fins, S_V, gam = init["S_h"], init["S_fins"], init["S_V"], init["gam"]
        gearpos = dict(init["gear"])
        tank = tank_layout(pod, fuel_kg, init["x_cg"])
    else:
        x_w = 1.55 + 0.6 * plug
        pod = Pod(1.95, 2.22, plug)
        S_h, S_fins, S_V, gam = 0.42, 0.33, 0.9, 40.0
        gearpos = {"x_mg": 1.90, "z_g": -0.43}
        tank = tank_layout(pod, fuel_kg, 1.75 + 0.6 * plug)
    aero = None
    hist = []
    for outer in range(60):
        wsecs = wing_sections(d, pf, x_w, i_w)
        if aero is None:
            aero = EC.build_aero(d, pf, wsecs, V_ref, h_ref, i_w)
        wm = EC.surface_mac(wsecs)
        x_ac_w = wm["x_le_mac"] + 0.25 * wm["mac"]
        w_pod = pod.props()["w_max"]
        cnb_f = cnb_pod(pod, S, pf["b"])
        boom = boom_geometry(d, wsecs, pf) if d.tail == "H" else None
        a_vt = vtail_slope(vtail_geometry(d, pod, S_V, gam), V_ref, h_ref) if d.tail == "V" else None
        for _ in range(40):
            if d.tail == "H":
                t = htail_geometry(d, pod, S_h, S_fins, boom)
                l_h = t["x_ac_h"] - x_ac_w
                l_v = t["x_ac_v"] - x_ac_w
                a_v = fin_slope(d, t, V_ref, h_ref)
                S_h_new = d.VH * S * pf["mac"] / l_h
                S_v_req = (d.cnb_req - cnb_f) * S * pf["b"] / (0.9 * a_v * l_v * 1.1)
                S_v = max(S_v_req, d.VV_min * S * pf["b"] / l_v)
                S_fins_new = S_v / math.cos(math.radians(d.fin_cant)) ** 2
                done = abs(S_h_new - S_h) < 1e-5 and abs(S_fins_new - S_fins) < 1e-5
                S_h, S_fins = S_h_new, S_fins_new
            else:
                t = vtail_geometry(d, pod, S_V, gam)
                l_h = l_v = t["x_ac"] - x_ac_w
                a_v = a_vt
                S_hh = d.VH * S * pf["mac"] / l_h
                S_v_req = (d.cnb_req - cnb_f) * S * pf["b"] / (0.9 * a_v * l_h * 1.1)
                S_v = max(S_v_req, d.VV_min * S * pf["b"] / l_h)
                S_new = S_hh + S_v
                gam_new = math.degrees(math.atan(math.sqrt(S_v / S_hh)))
                done = abs(S_new - S_V) < 1e-5 and abs(gam_new - gam) < 1e-4
                S_V, gam = S_new, gam_new
                S_h = S_hh
            if done:
                break
        if d.tail == "H":
            t = htail_geometry(d, pod, S_h, S_fins, boom)
            a_t = stab_slope(t, V_ref, h_ref)
            S_t = S_h
            x_ac_t, z_t = t["x_ac_h"], t["z_h"]
            t["a_t"], t["a_v"], t["cnb_fus"] = a_t, a_v, cnb_f
        else:
            t = vtail_geometry(d, pod, S_V, gam)
            a_t = vtail_slope(t, V_ref, h_ref)
            S_t = S_V * math.cos(math.radians(gam)) ** 2
            x_ac_t, z_t = t["x_ac"], t["z_mac"]
            t["a_t"], t["a_v"], t["cnb_fus"] = a_t, a_t, cnb_f
            t["S_h_eff"], t["S_v_eff"] = S_t, S_V * math.sin(math.radians(gam)) ** 2
        # ---- masses
        gl = gust_limit(d, aero["CL_alpha"], pf["mac"], aero["CLmax_wing"])
        n_lim = max(gl["n_limit"], N_POS)
        wmass = wing_mass(d, pf, m0, n_lim, wsecs, w_pod)
        x_spar, x_rear = wsecs[0]["x_le"] + 0.30 * pf["cr"], wsecs[0]["x_le"] + 0.65 * pf["cr"]
        pm = pod_mass(d, pod, x_spar, x_rear, gearpos["x_mg"], tank)
        masses = {"wing": wmass["total"], "pod_shell": pm["shell"], "chassis": pm["chassis"],
                  "chassis_items": pm["chassis_items"], "gear": gear_mass(d, m0)}
        if d.tail == "H":
            sm_ = surface_mass(t["stab_secs"], 5, 0.10 + 2 * 0.05)
            fm_ = surface_mass(t["fin_secs"], 3, 2 * 0.06, mirror=True)
            # surface_mass(mirror=True) doubles one fin's panel (both skins come from the mesh area): the mesh of one
            # fin already holds both skins, so the factor 2 counts the two fins.
            masses["stab"], masses["fins"] = sm_["total"], fm_["total"]
            per_boom_tail = 0.5 * sm_["total"] + 0.5 * fm_["total"] + 2 * 0.27
            bd = boom_design(d, t, boom, per_boom_tail)
            masses["boom_design"] = bd
            masses["booms"] = 2 * bd["mass_each_kg"]
            masses["tail"] = masses["stab"] + masses["fins"] + masses["booms"]
            tail_masses = {"stab": sm_, "fins": fm_}
        else:
            vm = surface_mass(t["secs"], 5, 2 * 0.35 + 2 * 0.10)
            masses["vtail"] = vm["total"]
            masses["tail"] = vm["total"]
            tail_masses = {"vtail": vm}
        items = component_list(d, pf, wsecs, t, boom, masses, gearpos, tank, pod)
        cases = []
        for c in EC.loading_cases():
            m, x, z = cg_of(items, fuel_kg, tank, c["payload"], c["fuel"])
            cases.append({"name": c["name"], "m": m, "x": x, "z": z})
        # ---- neutral point
        deps = SL.downwash_gradient(d.AR, pf["taper"], l_h, z_t - Z_WING_ROOT, pf["b"])
        kf = SL.kf_fuselage((wsecs[0]["x_le"] + 0.25 * pf["cr"]) / pod.L)
        cmaf = SL.cm_alpha_fuselage(kf, w_pod, pod.L, pf["mac"], S)
        x_np = SL.neutral_point(aero["CL_alpha"], x_ac_w, a_t, S_t, x_ac_t, S, pf["mac"], deps, ETA_T, cmaf)
        sms = [SL.static_margin(x_np, c["x"], pf["mac"]) for c in cases]
        err = min(sms) - d.sm_min
        hist.append((x_w, err))
        dx = -err * pf["mac"] / 0.4                      # fixed-point step (CG, tanks and engine follow the wing)
        if len(hist) >= 2:
            (x1, e1), (x2, e2) = hist[-2], hist[-1]
            if abs(x2 - x1) > 1e-6 and (e2 - e1) / (x2 - x1) > 0.05:
                dx = -e2 * (x2 - x1) / (e2 - e1)          # secant
        dx = float(np.clip(dx, -0.12, 0.12))
        gearpos = ground_geometry({"pod": pod, "tail": t, "boom": boom}, aero, cases)
        x_design = cases[0]["x"]
        tank = tank_layout(pod, fuel_kg, x_design)
        new_pod = Pod(tank["bay"][1] + 0.015, tank["cells"]["aft"]["x1"] + FW_CLEAR, plug)
        pod_moved = abs(new_pod.x_fw - pod.x_fw) > 3e-4 or abs(new_pod.xa - pod.xa) > 3e-4
        if verbose:
            print(f"  layout it{outer}: x_w={x_w:.3f} SMmin={min(sms):.3f} x_np={x_np:.3f} S_h={S_h:.3f} "
                  f"x_fw={pod.x_fw:.3f} x_mg={gearpos['x_mg']:.3f} z_g={gearpos['z_g']:.3f}")
        if abs(dx) < 3e-4 and not pod_moved and outer > 1:
            break
        x_w += dx
        if pod_moved:
            pod = Pod(0.5 * (pod.xa0 + new_pod.xa0), 0.5 * (pod.x_fw0 + new_pod.x_fw0), plug) if outer < 6 else new_pod
    if d.tail == "H":
        span_ind = t["b_h"] * 1.0
    else:
        span_ind = t["b_proj"]
    lay = {"x_w": x_w, "i_w": i_w, "x_ac_w": x_ac_w, "x_ac_t": x_ac_t, "l_h": l_h, "l_v": l_v, "w_pod": w_pod,
           "tail_span_induced": span_ind, "x_np": x_np, "deps_da": deps, "cm_alpha_fus": cmaf, "kf": kf,
           "cases": cases, "sms": sms, "gear": gearpos, "tank": tank, "items": items, "S_t": S_t, "a_t": a_t,
           "incidence_rule": {"i_level_loiter_deg": i_level, "i_touchdown_deg": i_td}, "gust": gl, "wing_mac": wm,
           "boom_design": masses.get("boom_design"),
           "state": {"x_w": x_w, "xa0": pod.xa0, "x_fw0": pod.x_fw0, "S_h": S_h, "S_fins": S_fins, "S_V": S_V,
                     "gam": gam, "gear": gearpos, "x_cg": cases[0]["x"]}, "iterations": outer + 1}
    ac = Aircraft(d=d, m0=m0, pf=pf, wing_secs=wsecs, tail=t, pod=pod, boom=boom, sa=aero["sa"], S=S, layout=lay,
                  mass={"wing": wmass, "groups": {k: v for k, v in masses.items()
                                                   if k not in ("chassis_items", "boom_design")},
                        "chassis_items": masses["chassis_items"], "tail_surfaces": tail_masses,
                        "boom_design": masses.get("boom_design")})
    return ac, aero


def finish_aero(ac: Aircraft, aero: dict, V_ref: float, h_ref: float, fuel_flow_ratio: float, extra_dDq: float = 0.0):
    d, pf, lay = ac.d, ac.pf, ac.layout
    items = drag_buildup(ac, V_ref, h_ref, fuel_flow_ratio)
    if extra_dDq:
        items["trade_delta"] = extra_dDq
    cd_rest = sum(items.values()) / pf["S"]
    cg_d = (lay["cases"][0]["x"], lay["cases"][0]["z"])
    CLs, CDs, clt = trimmed_polar(ac, aero, cd_rest, cg_d)
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
        return clw + cm * pf["mac"] / arm, cm * pf["mac"] / arm * pf["S"] / lay["S_t"]
    cl_clean, clt_clean = trim(aero["CLmax_wing"], 0.0)
    clmax = {"wing_clean": aero["CLmax_wing"], "clean_trimmed": cl_clean, "tail_cl_to_trim_clmax_fwd_cg": clt_clean}
    clmax["to_trimmed"] = clmax["ld_trimmed"] = cl_clean
    fe = flap_effect(d, pf, lay["w_pod"])
    if fe:
        dcl0 = aero["CL_alpha"] * math.radians(-fe["dalpha0_deg"])
        clmax["to_trimmed"] = trim(aero["CLmax_wing"] + fe["dCLmax"], -0.25 * dcl0)[0]
    ac.flap_to = fe
    ac.cd_table = (CLs, CDs)
    ac.fit = {"cd0": cd0, "k": k, "e": e, "LD_max": float(ld[i]), "CL_LDmax": float(CLs[i]),
              "CL_endurance": float(CLs[j]), "endurance_param_max": float(end_par[j]), "cd_rest": cd_rest,
              "cd0_to": cd0 + (fe["dCD0"] if fe else 0.0), "cd0_ld": cd0,
              "e_nita_scholz": AL.oswald_nita_scholz(pf["AR"], pf["taper"], 0.0, lay["w_pod"] / pf["b"]),
              "e_raymer": AL.oswald_straight(pf["AR"]), "cl_tail_S": (CLs, clt)}
    ac.clmax = clmax
    ac.drag_items = items
    ac.aero_cm0 = aero["cm0"]
    return ac


# =====================================================================================================================
# 9. evaluation (fixed MTOM -> loiter time; or the 10 h mission -> MTOW with sizinglib.MassModel)
# =====================================================================================================================
def v_ref_loiter() -> float:
    return EC.v_ref_loiter()


def empty_breakdown(ac: Aircraft) -> dict:
    g = ac.mass["groups"]
    airframe = g["wing"] + g["tail"] + g["pod_shell"] + g["chassis"] + g["gear"]
    mf = fixed_mass(ac.d)
    return {"fixed": mf, "airframe": airframe, "growth": GROWTH * (mf + airframe), "empty": (mf + airframe) * (1 + GROWTH)}


def airframe_fn_factory(ac: Aircraft):
    """m0 -> airframe mass fraction (incl. growth) at constant W/S, AR and layout (sizinglib.MassModel)."""
    d, g, S0 = ac.d, ac.mass["groups"], ac.S
    n_lim = max(ac.layout["gust"]["n_limit"], N_POS)

    @functools.lru_cache(maxsize=256)
    def frac(m0_r: float) -> float:
        m0 = float(m0_r)
        S = m0 * G / d.ws
        pf = wing_planform(d, S)
        wsecs = wing_sections(d, pf, ac.layout["x_w"], ac.layout["i_w"])
        wing = wing_mass(d, pf, m0, n_lim, wsecs, ac.layout["w_pod"])["total"]
        tail = g["tail"] * (S / S0) ** 1.5 if d.tail == "V" else \
            (g["stab"] + g["fins"]) * (S / S0) ** 1.5 + g["booms"]
        af = wing + tail + g["pod_shell"] + g["chassis"] + gear_mass(d, m0)
        return af * (1 + GROWTH) / m0
    return lambda m0: frac(round(m0, 4))


def evaluate(d: Design, mode: str = "mtom", endurance_target_h: float | None = None, extra: dict | None = None,
             verbose: bool = False) -> dict:
    extra = extra or {}
    V_ref = v_ref_loiter()
    m0 = d.mtom
    fuel_guess, ffr = 35.0, 0.33
    init = None
    for it in range(10):
        ac, aero = solve_configuration(d, m0, fuel_guess, V_ref, init=init)
        init = ac.layout["state"]
        finish_aero(ac, aero, V_ref, H_LOITER, ffr, extra.get("dDq", 0.0))
        eb = empty_breakdown(ac)
        empty = eb["empty"] + extra.get("dm", 0.0) * (1 + GROWTH)
        if mode == "mtom":
            fuel = m0 - PAYLOAD - empty
            mis = EC.solve_loiter_for_fuel(ac, m0, fuel)
            m_new = m0
        else:
            mis = EC.solve_loiter_for_endurance(ac, m0, endurance_target_h * 3600.0)
            mm = SZ.MassModel(PAYLOAD, (fixed_mass(d) + extra.get("dm", 0.0)) * (1 + GROWTH), mis["ff"],
                              airframe_fn_factory(ac))
            sol = mm.solve(m0_guess=m0)
            m_new, fuel = sol["mtow"], sol["fuel"]
        lo = [l for l in mis["log"] if l["kind"] == "loiter"]
        ff_loiter = np.mean([l["ff_kg_h"] for l in lo]) if lo else 2.5
        ffr_new = ff_loiter / (EC.BSFC_PTS[-1, 1] * P_MAX / 1e6)
        done = abs(fuel - fuel_guess) < 0.05 and abs(ffr_new - ffr) < 0.01 and abs(m_new - m0) < 0.05
        if verbose:
            print(f"   eval it{it}: m0 {m0:.2f} fuel {fuel:.2f} ff {mis['ff']:.4f} t_air {mis['t_air_s'] / 3600:.2f} h")
        fuel_guess, ffr = fuel, ffr_new
        if mode != "mtom":
            m0 = m_new
        if done:
            break
    return {"ac": ac, "aero": aero, "mission": mis, "fuel": fuel, "m0": m0, "empty": empty, "eb": eb,
            "endurance_h": mis["t_air_s"] / 3600.0}


# =====================================================================================================================
# 10. field performance (slipstream-aided rotation), climb, ceiling
# =====================================================================================================================
def k_ground(ac: Aircraft) -> float:
    h = Z_WING_ROOT - ac.layout["gear"]["z_g"]
    r = (16 * h / ac.pf["b"]) ** 2
    return ac.fit["k"] * r / (1 + r)


def rotation_speed(ac: Aircraft, m: float, h: float, slip: bool = True) -> dict:
    """Lowest speed at which full up elevator rotates the aircraft about the main wheels against weight, wing
    pitching moment and the thrust-line moment (endurance-chain moment balance) with the stabiliser in the propeller
    slipstream (H tail: immersed strip at q_s)."""
    atm = AL.isa(h)
    W = m * G
    lay, g, t = ac.layout, ac.layout["gear"], ac.tail
    fe = ac.flap_to
    dcl0 = ac.sa["CL_alpha"] * math.radians(-fe["dalpha0_deg"]) if fe else 0.0
    CLg = ac.sa["CL_0"] + dcl0
    x_cg = lay["cases"][0]["x"]
    arm_t = lay["x_ac_t"] - g["x_mg"]
    for V in np.linspace(5.0, 45.0, 401):
        T = EC.PROP.wot(V, h)["T"]
        q = 0.5 * atm["rho"] * V * V
        qr, f = tail_q_ratio(t, ac.pod, V, T, h) if slip else (1.0, 0.0)
        if V < 1.0:
            continue
        tail = CLT_MAX_ROT * lay["S_t"] * arm_t * q * qr
        wing = q * (ac.S * CLg * (g["x_mg"] - lay["x_ac_w"]) + ac.S * ac.pf["mac"] * (ac.aero_cm0 - 0.25 * dcl0))
        need = W * (g["x_mg"] - x_cg) + T * (Z_T - g["z_g"])
        if tail + wing >= need:
            return {"V_R": float(V), "q_ratio_tail": qr, "immersion": f, "T_N": T}
    return {"V_R": None, "q_ratio_tail": None, "immersion": None, "T_N": None}


def takeoff(ac: Aircraft, m: float, h: float = 0.0, slip: bool = True) -> dict:
    atm = AL.isa(h)
    W = m * G
    fe = ac.flap_to
    dcl0 = ac.sa["CL_alpha"] * math.radians(-fe["dalpha0_deg"]) if fe else 0.0
    dcd0 = fe["dCD0"] if fe else 0.0
    clmax = ac.clmax["to_trimmed"]
    VS = AL.stall_speed(W, ac.S, clmax, atm["rho"])
    CLg = ac.sa["CL_0"] + dcl0
    V_flat = math.sqrt(2 * W / (atm["rho"] * ac.S * CLg))
    rot = rotation_speed(ac, m, h, slip)
    V_R = rot["V_R"]
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
            "V_rotation_authority_m_s": V_R, "tail_q_ratio_at_VR": rot["q_ratio_tail"],
            "stab_immersion_at_VR": rot["immersion"], "T_static_N": T0, "T_lof_N": Tl, "air_distance_15m_m": s_air,
            "distance_15m_m": r["ground_roll"] + s_air, "climb_gradient": math.sin(gam), "CL_ground": CLg,
            "flap_deg": fe["delta_deg"] if fe else 0.0,
            "liftoff_mode": "flat (ground attitude)" if V_lof >= V_flat - 1e-6 else
                            ("rotated" if V_lof > 1.1 * VS + 1e-6 else "rotated at 1.1 VS")}


def landing(ac: Aircraft, m: float, h: float = 0.0) -> dict:
    atm = AL.isa(h)
    W = m * G
    r = AL.landing_roll(W, ac.S, ac.clmax["ld_trimmed"], ac.fit["cd0_ld"], k_ground(ac), mu_brake=0.3, rho=atm["rho"],
                        v_td_factor=1.15, cl_roll=ac.sa["CL_0"], t_free=1.0)
    VS = r["VS_ld"]
    Vf = 1.23 * VS
    R = Vf ** 2 / (0.2 * G)
    gam = math.radians(3.0)
    hf = R * (1 - math.cos(gam))
    s_air = (15.0 - hf) / math.tan(gam) + R * math.sin(gam)
    return {"ground_roll_m": r["ground_roll"], "V_td_m_s": r["V_td"], "VS_m_s": VS, "air_distance_15m_m": s_air,
            "distance_15m_m": r["ground_roll"] + s_air}


def performance(ac: Aircraft, m0: float, fuel: float) -> dict:
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
                       "power_available_W": P_MAX * EC.lapse(atm["sigma"]),
                       "power_lapse_research": EC.lapse(atm["sigma"]), "power_lapse_gagg_ferrar": AL.power_lapse(atm["sigma"])}
    out["ceiling_service_m"] = EC.ceiling(ac, m0)
    out["ceiling_absolute_m"] = EC.ceiling(ac, m0, 0.0)
    out["takeoff_sl_mtow"] = takeoff(ac, m0, 0.0)
    out["takeoff_sl_mtow_no_slipstream_credit"] = takeoff(ac, m0, 0.0, slip=False)
    out["takeoff_1500m_isa_mtow"] = takeoff(ac, m0, 1500.0)
    m_land = m0 - fuel * 0.88
    out["landing_sl_mtow"] = landing(ac, m0)
    out["landing_sl_end_of_mission"] = landing(ac, m_land)
    out["landing_mass_end_of_mission_kg"] = m_land
    out["static_wot"] = EC.PROP.wot(0.0, 0.0)
    out["static_tip_speed_m_s"] = math.pi * EC.D_PROP * out["static_wot"]["rpm"] / 60
    return out


# =====================================================================================================================
# 11. stall progression, structures summary, packaging, production indicators
# =====================================================================================================================
AIL_ETA = (0.57, 0.95)


def stall_margins(aero: dict) -> dict:
    """At the wing stall angle (critical section, research cl_max factor 0.94): stall-onset station and the highest
    cl/cl_max in the aileron span (roll control at the stall)."""
    sa = aero["sa"]
    LL = sa["ll"]
    clmax_s = np.interp(LL["y"], sa["tables"]["y"], sa["clmax_sections"]) * EC.K_CLMAX_SEC
    a_st = math.radians(aero["alpha_stall_deg"])
    ratio = LL["cl_local"](a_st) / clmax_s
    eta = LL["y"] / sa["semispan"]
    m = (eta >= AIL_ETA[0]) & (eta <= AIL_ETA[1])
    return {"stall_onset_eta": aero["stall_eta"], "aileron_cl_over_clmax_at_stall": float(np.max(ratio[m])),
            "aileron_cl_over_clmax_mean": float(np.mean(ratio[m])), "alpha_stall_deg": aero["alpha_stall_deg"]}


def structures_summary(ac: Aircraft, m0: float, VD: float) -> dict:
    lay = ac.layout
    wm = ac.mass["wing"]
    vn = EC.vn_summary(ac, m0, VD)
    out = {"vn": vn, "n_limit_design": lay["gust"]["n_limit"], "n_ultimate_wing": FOS * lay["gust"]["n_limit"],
           "wing_root_bending_ultimate_N_m": wm["M_root_ult_Nm"],
           "outer_panel_joint_bending_ultimate_N_m": wm["M_joint_ult_Nm"],
           "outer_panel_joint_shear_ultimate_N": wm["V_joint_ult_N"],
           "spar_cap_area_root_mm2": wm["A_cap_root_mm2"], "spar_depth_root_m": wm["h_eff_root_m"],
           "spar_cap_allowable_Pa": SIG_CAP,
           "standards": "JARUS CS-LUAS / STANAG 4703: +3.8/-1.5 g, FoS 1.5, gust 15.24 m/s at VC; fitting factor 1.15; "
                        "frequent-assembly factor 1.5 on the outer-panel, centre-section and boom joints"}
    # outer panel joint: 2 vertical pins form the bending couple across the spar depth at y_cs (tongue in a box)
    pf = ac.pf
    h_j = 0.95 * TC_ROOT * pf["cr"] - 0.010
    F_pin = wm["M_joint_ult_Nm"] / h_j * 1.15 * 1.5 / FOS             # ultimate x fitting x frequent assembly
    d_pin = 0.016
    tau_ti = MAT["metals"]["ti_6al_4v_annealed"]["Fsu"]
    P_pin = 2 * math.pi / 4 * d_pin ** 2 * tau_ti                        # double shear, Ti-6Al-4V
    out["outer_joint_pin_check"] = {"pin_force_ult_with_factors_N": F_pin, "pin": "Ti-6Al-4V 16 mm, double shear",
                                    "pin_allowable_N": P_pin, "MS": P_pin / F_pin - 1,
                                    "basis": "couple M/h at 0.95 t/c x c - 10 mm, x 1.15 fitting x 1.5 frequent "
                                             "assembly (STANAG 4703 UL2.4)"}
    if ac.tail["type"] == "H":
        bd = ac.mass["boom_design"]
        out["booms"] = {k: bd[k] for k in ("L_m", "t_wall_m", "mass_tube_kg", "M_ult_Nm", "T_ult_Nm", "EI_Nm2",
                                           "f1_bending_Hz", "tip_deflection_limit_m", "loads", "allowables_Pa")}
        out["booms"]["note"] = ("bought roll-wrapped CFRP tube; wall set by the 1.5 mm handling minimum; first "
                                "bending mode below the 33 Hz idle 1st-order excitation (crossed only at start/stop); "
                                "boom/tail flutter (bending-torsion) needs a GVT and analysis to 1.2 VD")
    items = lay["items"]
    n_ult = FOS * max(lay["gust"]["n_limit"], 4.47)
    out["chassis_check"] = chassis_check(ac.d, items, ac.wing_secs[0]["x_le"] + 0.30 * pf["cr"],
                                         ac.wing_secs[0]["x_le"] + 0.65 * pf["cr"], n_ult, ac.pod)
    return out


def packaging_checks(ac: Aircraft) -> dict:
    pod = ac.pod
    out = {}
    m = 0.010
    zc_hi, zc_lo = Z_T + ENV_ZC, Z_T - (ENV_H - ENV_ZC)
    boxes = {
        "cylinder_slab": ((pod.X_HUB - 0.18, pod.X_HUB - 0.05), ENV_W / 2, (Z_T - ENV_ZC, zc_hi)),
        "intake_box_below_crank": ((pod.X_HUB - 0.18, pod.X_HUB - 0.05), INTAKE_W / 2, (zc_lo, Z_T - ENV_ZC)),
        "crankcase_sg750_core": ((pod.X_HUB - ENV_L_SG, pod.X_HUB - 0.02), 0.10, (Z_T - 0.10, Z_T + 0.05)),
    }
    for k, ((x0, x1), yh, (z0, z1)) in boxes.items():
        ok = all(pod.inside(x, sy * yh, z, m) for x in np.linspace(x0, x1, 15) for sy in (1, -1) for z in (z0, z1))
        out[k] = {"x_range_m": [x0, x1], "half_width_m": yh, "z_range_m": [z0, z1], "fits_with_10mm": bool(ok)}
    if not out["cylinder_slab"]["fits_with_10mm"] or not out["intake_box_below_crank"]["fits_with_10mm"]:
        flag("engine envelope does not fit the pod OML with 10 mm clearance")
    xs = np.linspace(pod.X_HUB - ENV_L, pod.X_HUB, 45)
    fit = [x for x in xs if all(pod.inside(x, sy * ENV_W / 2, z, m) for sy in (1, -1) for z in (zc_lo, zc_hi))]
    out["documented_full_box"] = {"size_m": [ENV_L, ENV_W, ENV_H], "x_range_m": [pod.X_HUB - ENV_L, pod.X_HUB],
                                  "fits_between_x_m": [min(fit), max(fit)] if fit else None,
                                  "note": "the full rectangular box is conservative: the lower corners hold the "
                                          "0.30 m intake box only (endurance-chain packaging rule)"}
    tk = ac.layout["tank"]
    out["fuel_cells"] = {"required_m3": tk["volume_required_m3"], "available_m3": tk["volume_available_m3"],
                         "forward_cell_x_m": [tk["cells"]["forward"]["x0"], tk["cells"]["forward"]["x1"]],
                         "aft_cell_x_m": [tk["cells"]["aft"]["x0"], tk["cells"]["aft"]["x1"]],
                         "fuel_cg_x_m": tk["x_c"], "firewall_x_m": pod.x_fw,
                         "aft_cell_to_firewall_m": pod.x_fw - tk["cells"]["aft"]["x1"],
                         "forward_cell_to_parachute_bay_m": tk["cells"]["forward"]["x0"] - X_PARACHUTE[1],
                         "identical_cells_length_difference_m": tk["cells"]["aft"]["length"] -
                                                                 tk["cells"]["forward"]["length"]}
    if tk["cells"]["forward"]["x0"] < X_PARACHUTE[1]:
        flag("forward fuel cell overlaps the parachute bay")
    xb0, xb1 = tk["bay"]
    out["payload_bay"] = {"x_m": [xb0, xb1], "volume_m3": 0.8 * pod.internal_area(0.5 * (xb0 + xb1)) * (xb1 - xb0),
                          "internal_area_m2": pod.internal_area(0.5 * (xb0 + xb1))}
    w, h, *_ = (float(v[0]) for v in pod.section(X_TURRET))
    out["turret_bay"] = {"x_m": X_TURRET, "pod_width_m": w, "pod_height_m": h,
                         "growth_envelope_m": [TUR_D_GROWTH, TUR_H_GROWTH],
                         "fits": bool(w - 0.02 >= TUR_D_GROWTH and h >= TUR_H_GROWTH - 0.13)}
    out["parachute_bay"] = {"x_m": list(X_PARACHUTE), "container_m": CHUTE_BOX,
                            "fits": bool(all(pod.inside(x, sy * (CHUTE_BOX[0] / 2 + 0.01), z, 0.01)
                                             for x in X_PARACHUTE for sy in (1, -1)
                                             for z in (float(pod.z_top(x)[0]) - CHUTE_BOX[1] - 0.03,
                                                       float(pod.z_top(x)[0]) - 0.03)))}
    out["cooling_exit_annulus_m2"] = math.pi / 4 * (float(pod.section(pod.X_LIP)[0][0]) * 0.97) ** 2 - \
        math.pi / 4 * pod.SPINNER["d"] ** 2
    out["prop_to_cowl_lip_axial_m"] = pod.X_PROP - 0.02 - pod.X_LIP
    if out["prop_to_cowl_lip_axial_m"] < CLEAR_PROP_LONG:
        flag("propeller longitudinal clearance to the cowl lip below 13 mm (CS-VLA 925(c)(2))")
    t = ac.tail
    if t["type"] == "H":
        b = ac.boom
        rad = math.hypot(b["y"], b["z"] - Z_T) - b["r"] - EC.D_PROP / 2
        out["prop_tip_to_boom_radial_m"] = rad
        out["prop_plane_to_stabiliser_le_m"] = t["x_le_h"] - pod.X_PROP
        out["wing_te_to_prop_plane_m"] = pod.X_PROP - (ac.wing_secs[0]["x_le"] + ac.pf["cr"])
        if rad < CLEAR_PROP_RADIAL:
            flag("propeller tip to boom radial clearance below 26 mm (CS-VLA 925(c)(1))")
    else:
        out["prop_to_tail_te_axial_m"] = pod.X_PROP - (t["secs"][0]["x_le"] + t["secs"][0]["chord"])
    return out


def production_indicators(ac: Aircraft) -> dict:
    """Counts derived from the configuration definition (no cost data exist in the research; these are the drivers of
    tooling, assembly and maintenance effort). Control surfaces are moulded with their parent skins and cut out, so
    they need no tools of their own; the bolt-on shell panels between the nose cone and the cowl (sides, top hatches,
    belly door) are trimmed from two half-shell tools of the near-constant mid-body and its transition."""
    pf, pod, t = ac.pf, ac.pod, ac.tail
    if t["type"] == "H":
        moulds = {"wing_centre_section_upper_lower (constant chord, extruded tooling)": 2,
                  "outer_panels_upper_lower_L_R (linear taper)": 4,
                  "stabiliser (symmetric section and planform: one mould gives both skins)": 1,
                  "fins (identical parts, side A + side B)": 2, "boom_nose_fairings (identical L/R)": 1,
                  "pod_nose_cone_radome": 1,
                  "pod_mid_and_transition_body_upper_lower (panels, hatches and belly door trimmed from them)": 2,
                  "engine_cowl_upper_lower": 2, "wing_saddle_fairing": 1}
        servos = 8
        joints = {"centre_section_to_pod": "4 bolts (2 spar fittings, 2 rear fittings)",
                  "outer_panels": "2 x (spar tongue + 2 pins + 1 drag pin) + 1 connector each",
                  "tail_unit_to_centre_section": "2 x 2 bolts (boom saddles) + 1 connector per boom"}
        pieces = {"pod_without_propeller_m": pod.L, "centre_section_span_m": 2 * pf["y_cs"],
                  "outer_panel_m": pf["outer_panel_span"],
                  "tail_unit_booms_stab_fins_m": [t["boom_x_end"] - ac.boom["x_tube0"], 2 * t["tip_y_fin"]]}
        longest = max(pod.L, 2 * pf["y_cs"], pf["outer_panel_span"], t["boom_x_end"] - ac.boom["x_tube0"])
        bought = ["2 x CFRP tube 80 x 1.5 mm (booms)"]
    else:
        moulds = {"wing_centre_section_upper_lower": 2, "outer_panels_upper_lower_L_R": 4,
                  "v_tail_panels (identical, side A + side B)": 2, "fuselage_nose_cone_radome": 1,
                  "fuselage_mid_and_transition_body_upper_lower (panels, hatches and belly door trimmed from them)": 2,
                  "engine_cowl_upper_lower (cut around the V-tail roots)": 2, "wing_saddle_fairing": 1,
                  "tail_cone_hump": 1}
        servos = 6
        joints = {"centre_section_to_fuselage": "4 bolts", "outer_panels": "2 x (tongue + 2 pins + drag pin)",
                  "v_tail_panels": "2 x (root spigot + 2 bolts)"}
        pieces = {"fuselage_without_propeller_m": pod.L, "centre_section_span_m": 2 * pf["y_cs"],
                  "outer_panel_m": pf["outer_panel_span"], "v_tail_panel_m": t["span_panel"]}
        longest = max(pod.L, 2 * pf["y_cs"], pf["outer_panel_span"])
        bought = []
    return {"moulds": moulds, "mould_count": sum(moulds.values()), "servo_count": servos, "field_joints": joints,
            "transport_pieces": pieces, "longest_piece_m": longest, "bought_structural_parts": bought,
            "composite_process_lines": 1, "metal_processes": ["water-jet/laser cutting", "press-brake bending",
                                                               "CNC milling (7075 hard points)", "TIG (4130 mount)",
                                                               "blind/solid riveting"]}


# =====================================================================================================================
# 12. trades
# =====================================================================================================================
def point_summary(r: dict) -> dict:
    ac = r["ac"]
    to = takeoff(ac, r["m0"])
    ld = landing(ac, r["m0"] - r["fuel"] * 0.88)
    sm = stall_margins(r["aero"])
    L_all = (ac.tail["boom_x_end"] if ac.tail["type"] == "H" else ac.pod.SPINNER["x1"])
    return {"AR": ac.pf["AR"], "ws_Pa": ac.d.ws, "S_m2": ac.S, "span_m": ac.pf["b"], "c_root_m": ac.pf["cr"],
            "c_tip_m": ac.pf["ct"], "y_cs_m": ac.pf["y_cs"], "outer_panel_m": ac.pf["outer_panel_span"],
            "length_m": L_all, "endurance_h": r["endurance_h"], "fuel_kg": r["fuel"], "empty_kg": r["empty"],
            "wing_kg": ac.mass["groups"]["wing"], "tail_kg": ac.mass["groups"]["tail"], "l_h_m": ac.layout["l_h"],
            "cd0": ac.fit["cd0"], "e": ac.fit["e"], "LD_max": ac.fit["LD_max"],
            "VS_m_s": EC.vstall(ac, r["m0"] * G, 0.0), "to_roll_m": to["ground_roll_m"],
            "V_rot_m_s": to["V_rotation_authority_m_s"], "ldg_roll_m": ld["ground_roll_m"],
            "stall_onset_eta": sm["stall_onset_eta"], "aileron_cl_ratio_at_stall": sm["aileron_cl_over_clmax_at_stall"],
            "n_gust_limit": ac.layout["gust"]["n_limit"]}


def violations(sm: dict) -> list:
    bad = []
    if sm["span_m"] > SPAN_MAX + 1e-6:
        bad.append("span")
    if sm["c_tip_m"] < TIP_CHORD_MIN:
        bad.append("tip chord")
    if sm["VS_m_s"] > 24.0 + 1e-6:
        bad.append("stall")
    if sm["to_roll_m"] > FIELD_ROLL_MAX:
        bad.append("take-off roll")
    if sm["ldg_roll_m"] > FIELD_ROLL_MAX:
        bad.append("landing roll")
    if sm["aileron_cl_ratio_at_stall"] > 0.95:
        bad.append("aileron stall margin")
    return bad


def ws_stall_corner(d: Design) -> float:
    """W/S that puts the clean trimmed 1-g stall at 24 m/s (MTOM, sea level) for this wing (no flaps)."""
    d0 = replace(d, ws=470.0)
    ac, aero = solve_configuration(d0, d0.mtom, 35.0, v_ref_loiter())
    finish_aero(ac, aero, v_ref_loiter(), H_LOITER, 0.31)
    return round(0.5 * RHO0 * 24.0 ** 2 * ac.clmax["clean_trimmed"] * 0.995, 1)


def run_wing_tail_trades(base: Design) -> tuple:
    out = {}
    V = v_ref_loiter()
    # ---------------- wheel spats (decided first: it changes the drag of every later point)
    print("\n[trade 0] wheel spats")
    r1 = evaluate(replace(base, spats=True))
    r0 = evaluate(replace(base, spats=False))
    keep = r1["endurance_h"] - r0["endurance_h"] > 0.10
    out["wheel_spats"] = {"with_spats": {"endurance_h": r1["endurance_h"],
                                         "gear_Dq_m2": r1["ac"].drag_items["landing_gear"]},
                          "without_spats": {"endurance_h": r0["endurance_h"],
                                            "gear_Dq_m2": r0["ac"].drag_items["landing_gear"],
                                            "mass_saved_kg": SPATS_MASS},
                          "decision": "spats fitted (quick-release)" if keep else "no wheel spats",
                          "rule": "keep the spats only if they buy more than 0.1 h; the bow-leg fairings stay in every "
                                  "case (an unfaired flat spring leg has D/q of about 1.4 x its frontal area)",
                          "reason": "the 0.70 kg of spats nearly cancel their drag saving; without them tyres and brakes "
                                    "are inspected without tools and mud/grass strips do not pack the spats"}
    base = replace(base, spats=keep)
    print(f"   with spats E {r1['endurance_h']:.2f} h, without {r0['endurance_h']:.2f} h -> "
          f"{'spats' if keep else 'no spats'}")
    # ---------------- outer-panel taper x washout (stall progression vs developable outer-panel skins)
    print("\n[trade 1] outer-panel taper x linear washout: stall progression (critical-section method, aero.py)")
    S0 = base.mtom * G / base.ws
    rows = []
    for lam in (0.5, 0.6, 0.7):
        for wo in (0.0, 1.0, 2.0, 3.0, 4.0):
            dd = replace(base, taper_o=lam, washout=wo)
            pf = wing_planform(dd, S0)
            aero = EC.build_aero(dd, pf, wing_sections(dd, pf, 1.6, 4.0), V, H_LOITER, 4.0)
            sm = stall_margins(aero)
            rows.append({"taper_outer": lam, "washout_deg": wo, "tip_chord_m": pf["ct"],
                         "CLmax_wing": aero["CLmax_wing"], "e_inviscid": aero["e_inv"],
                         "stall_onset_eta": sm["stall_onset_eta"],
                         "aileron_cl_over_clmax_max": sm["aileron_cl_over_clmax_at_stall"],
                         "aileron_cl_over_clmax_mean": sm["aileron_cl_over_clmax_mean"]})
            print(f"   taper {lam:.1f} washout {wo:.1f}: onset eta {sm['stall_onset_eta']:.2f}, aileron cl/clmax max "
                  f"{sm['aileron_cl_over_clmax_at_stall']:.3f} mean {sm['aileron_cl_over_clmax_mean']:.3f}, CLmax "
                  f"{aero['CLmax_wing']:.3f}, e_inv {aero['e_inv']:.4f}")
    rule_ok = lambda x: x["stall_onset_eta"] < 0.30 and x["aileron_cl_over_clmax_max"] <= 0.95
    cands = []
    for lam in (0.5, 0.6, 0.7):
        ok = [x for x in rows if x["taper_outer"] == lam and rule_ok(x)]
        if ok:
            c = dict(min(ok, key=lambda x: x["washout_deg"]))
            dd = replace(base, taper_o=lam, washout=c["washout_deg"])
            c["ws_Pa"] = ws_stall_corner(dd)
            r = evaluate(replace(dd, ws=c["ws_Pa"]))
            c["endurance_h"] = r["endurance_h"]
            c["wing_kg"] = r["ac"].mass["groups"]["wing"]
            cands.append(c)
            print(f"   candidate taper {lam:.1f} / washout {c['washout_deg']:.1f}: W/S {c['ws_Pa']:.1f} Pa, "
                  f"E {c['endurance_h']:.2f} h")
    pick = max(cands, key=lambda x: x["endurance_h"])
    out["stall_progression"] = {
        "rows": rows, "candidates": cands, "chosen": {"taper_outer": pick["taper_outer"],
                                                     "washout_deg": pick["washout_deg"]},
        "rule": "stall must start at the root (eta < 0.30) with cl <= 0.95 cl_max everywhere on the aileron span "
                "(eta 0.57-0.95) at the wing stall; per taper the smallest washout meeting it, then the best endurance "
                "at each candidate's own stall-corner W/S",
        "production_note": "an untwisted outer panel with one section would have exactly conical (developable) skins, "
                           "but this planform is nearly equal-stall along the span: without washout the first stall "
                           "is in the aileron span; a linear twist of a few degrees over 2 m keeps the skins nearly "
                           "developable (prepreg drapes it without darts)"}
    base = replace(base, taper_o=pick["taper_outer"], washout=pick["washout_deg"], ws=pick["ws_Pa"])
    print(f"   -> outer taper {pick['taper_outer']:.1f}, washout {pick['washout_deg']:.1f} deg, W/S {pick['ws_Pa']:.1f} Pa")

    # ---------------- constant-chord centre-section span
    print("\n[trade 2] constant-chord centre-section span (outer-panel joint station)")
    rows = []
    for ycs in (base.y_boom, 0.80, 1.00, 1.20):
        dd = replace(base, y_cs=ycs)
        dd = replace(dd, ws=ws_stall_corner(dd))
        r = evaluate(dd)
        sm = point_summary(r)
        sm["violations"] = violations(sm)
        sm["joint_M_ult_Nm"] = r["ac"].mass["wing"]["M_joint_ult_Nm"]
        sm["largest_wing_piece_m"] = max(2 * sm["y_cs_m"], sm["outer_panel_m"])
        rows.append(sm)
        print(f"   y_cs {ycs:4.2f}: centre {2 * ycs:.2f} m, outer panel {sm['outer_panel_m']:.2f} m, wing "
              f"{sm['wing_kg']:.2f} kg, E {sm['endurance_h']:.2f} h, joint M {sm['joint_M_ult_Nm']:.0f} N m, aileron "
              f"{sm['aileron_cl_ratio_at_stall']:.3f} {sm['violations']}")
    best_E = max(x["endurance_h"] for x in rows if not x["violations"])
    ok = [x for x in rows if not x["violations"] and x["endurance_h"] >= best_E - 0.10]
    pick = min(ok, key=lambda x: (round(x["largest_wing_piece_m"], 2), -x["y_cs_m"]))
    out["centre_section_span"] = {"rows": rows, "chosen_y_cs_m": pick["y_cs_m"],
                                  "rule": "within 0.1 h of the best endurance, the smallest largest wing piece: centre "
                                          "section and outer panels of about equal length share one transport case "
                                          "and one handling fixture; ties -> larger constant-chord share (one rib "
                                          "family, extruded skin tool)"}
    base = replace(base, y_cs=pick["y_cs_m"], ws=pick["ws_Pa"])
    print(f"   -> y_cs {pick['y_cs_m']:.2f} m")

    # ---------------- boom length (stabiliser gap behind the propeller)
    print("\n[trade 3] boom length (propeller plane -> stabiliser LE)")
    rows = []
    for gap in (0.45, 0.65, 0.85, 1.05, 1.25):
        dd = replace(base, tail_gap=gap)
        r = evaluate(replace(dd, ws=ws_stall_corner(dd)))
        sm = point_summary(r)
        sm["violations"] = violations(sm)
        bd = r["ac"].mass["boom_design"]
        sm.update({"tail_gap_m": gap, "S_h_m2": r["ac"].tail["S_h"], "S_fins_m2": r["ac"].tail["S_fins"],
                   "boom_L_m": bd["L_m"], "boom_f1_Hz": bd["f1_bending_Hz"], "L_over_b": sm["length_m"] / sm["span_m"],
                   "pod_L_m": r["ac"].pod.L})
        rows.append(sm)
        print(f"   gap {gap:4.2f}: L {sm['length_m']:.2f} m (L/b {sm['L_over_b']:.2f}), l_h {sm['l_h_m']:.2f}, S_h "
              f"{sm['S_h_m2']:.3f}, fins {sm['S_fins_m2']:.3f} m2, tail+booms {sm['tail_kg']:.2f} kg, boom f1 "
              f"{sm['boom_f1_Hz']:.1f} Hz, E {sm['endurance_h']:.2f} h, V_R {sm['V_rot_m_s']} {sm['violations']}")
    ok = [x for x in rows if not x["violations"] and x["boom_f1_Hz"] >= 12.0 and
          x["boom_L_m"] <= x["pod_L_m"] + 1e-6]
    pick = max(ok, key=lambda x: x["endurance_h"]) if ok else rows[1]
    out["boom_length"] = {"rows": rows, "chosen_gap_m": pick["tail_gap_m"],
                          "rule": "max endurance with the boom (tail-unit) length <= the pod length (one transport "
                                  "case length for every piece), boom first bending >= 12 Hz (estimate: well above "
                                  "the rigid-body/flight-control band) and all field/stall limits; the gap also sets "
                                  "the propeller-wake distance to the stabiliser (>= 0.5 D preferred)",
                          "note": "L/b is reported, not limited: twin-boom comparables have L/b 0.73-0.83 "
                                  "(comparables.yaml#recommended_ranges.length_m)"}
    base = replace(base, tail_gap=pick["tail_gap_m"], ws=pick["ws_Pa"])
    print(f"   -> gap {pick['tail_gap_m']:.2f} m")

    # ---------------- aspect ratio x wing loading
    print("\n[trade 4] aspect ratio at the stall-limited wing loading (VS <= 24 m/s clean, no flaps)")
    rows = []
    corners = {}
    for AR in (10.0, 11.0, 12.0, 13.0, 14.0):
        corners[AR] = ws_stall_corner(replace(base, AR=AR))
        for ws in ((corners[AR], 433.0) if AR == 12.0 else (corners[AR],)):
            r = evaluate(replace(base, AR=AR, ws=ws))
            sm = point_summary(r)
            sm["violations"] = violations(sm)
            rows.append(sm)
            print(f"   AR {AR:4.1f} W/S {ws:5.1f}: b {sm['span_m']:.2f}, outer panel {sm['outer_panel_m']:.2f}, ct "
                  f"{sm['c_tip_m']:.3f}, wing {sm['wing_kg']:.1f} kg, empty {sm['empty_kg']:.1f}, L/D "
                  f"{sm['LD_max']:.2f}, E {sm['endurance_h']:.2f} h, TO {sm['to_roll_m']:.0f}, LDG "
                  f"{sm['ldg_roll_m']:.0f} {sm['violations']}")
    case_len = 2 * base.y_cs + 0.05
    ok = [x for x in rows if not x["violations"] and x["outer_panel_m"] <= case_len and x["AR"] <= 13.0 and
          abs(x["ws_Pa"] - corners[x["AR"]]) < 1.0]
    if not ok:
        flag("no aspect ratio met the equal-wing-piece production rule; the best point was taken")
        ok = [max(rows, key=lambda x: x["endurance_h"])]
    pick = max(ok, key=lambda x: x["endurance_h"])
    out["aspect_ratio_wing_loading"] = {
        "rows": rows, "stall_corner_ws_Pa": {str(k): v for k, v in corners.items()},
        "chosen": {"AR": pick["AR"], "ws_Pa": pick["ws_Pa"]},
        "rule": f"production rule: the best endurance among the stall-corner points whose outer panel is not longer than "
                f"the constant-chord centre section + 5 cm ({case_len:.2f} m): all three wing pieces share one transport "
                "case and one handling fixture (same rule as the centre-section span); research AR band 10-13 "
                "(comparables.yaml: above about 13 spar depth, hinge space and ground handling get hard)",
        "finding": "between AR 10 and 12 the wing mass changes by only a few hundred grams (skins dominate and the area "
                   "is set by the stall corner), so the lower AR saves almost nothing in cost or mass but costs "
                   "endurance; above AR 12 the outer panels outgrow the centre-section case"}
    base = replace(base, AR=pick["AR"], ws=pick["ws_Pa"])
    print(f"   -> AR {pick['AR']:.0f}, W/S {pick['ws_Pa']:.1f} Pa")
    return base, out


def run_config_trades(d: Design, ref: dict) -> dict:
    out = {}
    ac = ref["ac"]
    E0 = ref["endurance_h"]
    ref_sm = point_summary(ref)
    pi_H = production_indicators(ac)
    # ---------------- tail configuration: twin-boom H vs single-fuselage V (same wing, same tail volume)
    print("\n[trade 5] tail configuration: twin boom + H tail vs single fuselage + V tail (same wing, V_H, Cn_beta)")
    rv, plug = None, None
    for pl in (0.3, 0.6, 0.9):
        dv_ = replace(d, tail="V", v_plug=pl)
        r_ = evaluate(replace(dv_, ws=ws_stall_corner(dv_)))
        print(f"   V variant plug {pl:.1f} m: E {r_['endurance_h']:.2f} h, l_h {r_['ac'].layout['l_h']:.2f} m, "
              f"empty {r_['empty']:.1f} kg")
        if rv is None or r_["endurance_h"] > rv["endurance_h"]:
            rv, plug = r_, pl
    acv = rv["ac"]
    smv = point_summary(rv)
    pi_V = production_indicators(acv)
    rvf = evaluate(replace(rv["ac"].d, flaps=True))
    smvf = point_summary(rvf)
    gH, gV = ac.layout["gear"], acv.layout["gear"]
    var = {
        "twin_boom_H": {"endurance_h": E0, "empty_kg": ref["empty"], "tail_and_booms_kg": ac.mass["groups"]["tail"],
                        "pod_shell_kg": ac.mass["groups"]["pod_shell"], "chassis_kg": ac.mass["groups"]["chassis"],
                        "cd0": ac.fit["cd0"], "LD_max": ac.fit["LD_max"], "to_roll_m": ref_sm["to_roll_m"],
                        "V_rotation_m_s": ref_sm["V_rot_m_s"], "ldg_roll_m": ref_sm["ldg_roll_m"],
                        "overall_length_m": ref_sm["length_m"], "fuselage_length_m": ac.pod.SPINNER["x1"],
                        "longest_piece_m": pi_H["longest_piece_m"], "moulds": pi_H["mould_count"],
                        "servos": pi_H["servo_count"], "prop_strike_deg": gH["prop_strike_deg"],
                        "tail_bumper_deg": gH["bumper_contact_deg"], "gear_height_rule": gH["active_height_rule"],
                        "l_h_m": ac.layout["l_h"]},
        "single_fuselage_V": {"endurance_h": rv["endurance_h"], "empty_kg": rv["empty"], "ws_Pa": acv.d.ws,
                              "tail_and_booms_kg": acv.mass["groups"]["tail"],
                              "pod_shell_kg": acv.mass["groups"]["pod_shell"], "chassis_kg": acv.mass["groups"]["chassis"],
                              "cd0": acv.fit["cd0"], "LD_max": acv.fit["LD_max"], "to_roll_m": smv["to_roll_m"],
                              "V_rotation_m_s": smv["V_rot_m_s"], "ldg_roll_m": smv["ldg_roll_m"],
                              "overall_length_m": smv["length_m"], "fuselage_length_m": acv.pod.SPINNER["x1"],
                              "longest_piece_m": pi_V["longest_piece_m"], "moulds": pi_V["mould_count"],
                              "servos": pi_V["servo_count"], "prop_strike_deg": gV["prop_strike_deg"],
                              "tail_bumper_deg": gV["bumper_contact_deg"], "gear_height_rule": gV["active_height_rule"],
                              "l_h_m": acv.layout["l_h"], "plug_m": plug, "violations": violations(smv)},
        "single_fuselage_V_with_takeoff_flaps": {"endurance_h": rvf["endurance_h"], "empty_kg": rvf["empty"],
                                                 "to_roll_m": smvf["to_roll_m"], "V_rotation_m_s": smvf["V_rot_m_s"],
                                                 "servos": pi_V["servo_count"] + 2, "violations": violations(smvf),
                                                 "note": "endurance-concept remedy (15 deg plain flaps on the centre "
                                                         "section, 2 more actuators)"},
    }
    for k, v in var.items():
        print(f"   {k}: E {v['endurance_h']:.2f} h, empty {v['empty_kg']:.1f} kg, TO {v['to_roll_m']:.0f} m "
              f"(V_R {v['V_rotation_m_s']}), " + (f"CD0 {v['cd0']:.4f}, L {v['overall_length_m']:.2f} m, longest piece "
                                                  f"{v['longest_piece_m']:.2f} m, moulds {v['moulds']}, " if 'cd0' in v
                                                  else "") + f"servos {v['servos']}")
    out["tail_configuration"] = {
        "variants": var,
        "decision": "twin boom + H tail",
        "reasons": [
            f"field: the stabiliser sits in the propeller slipstream, so full up elevator rotates the aircraft at "
            f"{ref_sm['V_rot_m_s']:.1f} m/s instead of {smv['V_rot_m_s']} m/s: lift-off at 1.1 VS without flaps "
            f"(TO roll {ref_sm['to_roll_m']:.0f} vs {smv['to_roll_m']:.0f} m)",
            f"transport and handling: longest piece {pi_H['longest_piece_m']:.2f} m vs {pi_V['longest_piece_m']:.2f} m "
            f"(one-piece fuselage); the tail unit comes off pre-rigged with 4 bolts",
            "maintenance: the engine bay is open on top and both sides (no tail roots on the cowl); the propeller is "
            "guarded laterally by the booms; the short pod puts every bay within arm's reach",
            "risk: twin-boom H/inverted-V tails are the structurally proven layout of the 50-300 kg runway class "
            "(comparables.yaml configuration_census: Shadow, Pioneer, Scout, Ranger, Aerostar, Primoco); the split "
            "elevator and two rudders leave no single servo failure with a free-floating pitch/yaw surface",
            "booms are bought CFRP tubes (no mould); stabiliser one mould, fins one pair",
        ],
        "costs_accepted": [
            f"endurance {E0 - rv['endurance_h']:+.2f} h vs the V variant, {pi_H['servo_count'] - pi_V['servo_count']} "
            f"more servos, {pi_H['mould_count'] - pi_V['mould_count']:+d} moulds",
            "stabiliser in the propeller wake (2/rev blade-passage loading, acoustic fatigue): stiff, well-damped "
            "stabiliser, 0.5 D or more gap, vibration survey",
            "boom/tail flutter and the boom-to-wing joints need a GVT and rational analysis (FAA Report 45 is not "
            "valid for boom tails, standards.yaml FLU-001) - the same applies to the V tail",
            "less MALE/UCAV-like silhouette than a single fuselage: shape language carried by the chined pod, the "
            "dorsal engine hump and the swept, outward-canted fins",
        ]}

    # ---------------- flaps
    print("\n[trade 6] centre-section take-off flaps")
    rf = evaluate(replace(d, flaps=True))
    smf = point_summary(rf)
    out["flaps"] = {"none": {"endurance_h": E0, "to_roll_m": ref_sm["to_roll_m"], "ldg_roll_m": ref_sm["ldg_roll_m"]},
                    "centre_section_plain_flaps_15deg": {"endurance_h": rf["endurance_h"], "to_roll_m": smf["to_roll_m"],
                                                         "S_flapped_over_S": rf["ac"].flap_to["S_flapped_over_S"],
                                                         "added_mass_kg": FLAP_KIT},
                    "decision": "no flaps" if ref_sm["to_roll_m"] <= FIELD_ROLL_MAX else "take-off flaps needed",
                    "reason": "the slipstream-aided rotation already gives a lift-off at 1.1 VS within the 200 m roll; "
                              "the boom saddles split the centre-section flap into short pieces (small flapped area), "
                              "so flaps would add 2 actuators, hinges and wiring for little gain"}
    print(f"   flaps: E {rf['endurance_h']:.2f} h, TO {smf['to_roll_m']:.0f} m; none: E {E0:.2f} h, TO {ref_sm['to_roll_m']:.0f} m")
    # ---------------- outer-panel tip thickness
    print("\n[trade 7] tip thickness: constant 16 % vs 16 -> 13 % (baseline)")
    rt = evaluate(replace(d, ts_tip=0.8125))
    out["tip_thickness"] = {"constant_16pct": {"endurance_h": E0, "wing_kg": ac.mass["groups"]["wing"]},
                            "tapered_to_13pct": {"endurance_h": rt["endurance_h"], "wing_kg": rt["ac"].mass["groups"]["wing"]},
                            "decision": "constant 16 %" if rt["endurance_h"] - E0 < 0.20 else "16 -> 13 %",
                            "reason": "one rib family (all ribs scaled copies), conical outer skins, deeper tip spar; "
                                      "the 13 % tip gains less than 0.2 h"}
    print(f"   13 % tip: E {rt['endurance_h']:.2f} h, wing {rt['ac'].mass['groups']['wing']:.2f} kg; 16 %: E {E0:.2f} h")
    # ---------------- wheel spats
    # ---------------- chassis material
    print("\n[trade 8] chassis: aluminium sheet metal vs CFRP keel beams + sandwich bulkheads")
    rc = evaluate(replace(d, chassis="cfrp"))
    out["chassis_material"] = {
        "aluminium": {"endurance_h": E0, "chassis_kg": ac.mass["groups"]["chassis"]},
        "cfrp": {"endurance_h": rc["endurance_h"], "chassis_kg": rc["ac"].mass["groups"]["chassis"]},
        "decision": "aluminium",
        "reason": "MIL-HDBK-5 A-basis allowables (no composite qualification programme for the most loaded, most "
                  "detailed parts: every hard point, gear and engine load path), flat-pattern parts cut and bent from "
                  "CAD without tooling, visible damage, field-repairable with rivets and doublers, isotropic fitting "
                  f"design; costs {rc['endurance_h'] - E0:+.2f} h"}
    print(f"   CFRP chassis: E {rc['endurance_h']:.2f} h ({rc['ac'].mass['groups']['chassis']:.2f} kg) vs Al "
          f"{ac.mass['groups']['chassis']:.2f} kg")
    # ---------------- propeller
    print("\n[trade 9] propeller 32x18 2B vs 31x12 3B")
    EC.set_prop("0164")
    r3 = evaluate(d)
    p3 = {"endurance_h": r3["endurance_h"], "roc_sl_m_s": EC.climb_point(r3["ac"], r3["m0"] * G, 0.0)["roc"],
          "to_roll_m": takeoff(r3["ac"], r3["m0"])["ground_roll_m"],
          "loiter_rpm_3000m": EC.best_loiter(r3["ac"], r3["m0"] * G, H_LOITER)["rpm"]}
    EC.set_prop("0161")
    p2 = {"endurance_h": E0, "roc_sl_m_s": EC.climb_point(ac, ref["m0"] * G, 0.0)["roc"],
          "to_roll_m": ref_sm["to_roll_m"], "loiter_rpm_3000m": EC.best_loiter(ac, ref["m0"] * G, H_LOITER)["rpm"]}
    out["propeller"] = {"mejzlik_32x18_2B": p2, "mejzlik_31x12_3B": p3,
                        "decision": "32x18 2B (research primary); 31x12 3B is the climb/hot-day option on the same hub"}
    print(f"   32x18 2B: {p2}\n   31x12 3B: {p3}")
    # ---------------- retractable gear (brief: fixed; quantified for the record)
    print("\n[trade 10] fixed vs retractable gear (delta method of the endurance chain)")
    dDq_r = -0.9 * ac.drag_items["landing_gear"]
    dm_r = (11.5 - 9.1) + 1.0 - GEAR_FAIRINGS
    rr = evaluate(d, extra={"dDq": dDq_r, "dm": dm_r})
    out["landing_gear"] = {"fixed": {"endurance_h": E0}, "retractable": {"endurance_h": rr["endurance_h"], "dDq_m2": dDq_r,
                                                                          "dm_kg": dm_r},
                           "decision": "fixed tricycle (brief): no retraction mechanism, no bays in the CG zone, no "
                                       "purchasable 100-200 kg retract unit (components.yaml)"}
    print(f"   retractable: E {rr['endurance_h']:.2f} h vs fixed {E0:.2f} h")
    # ---------------- pusher vs tractor (qualitative + slipstream scrubbing estimate)
    fp = ac.pod.props()
    T_ref = ref["m0"] * G / ac.fit["LD_max"]
    V3 = EC.best_loiter(ac, ref["m0"] * G, H_LOITER)["V"]
    q3 = 0.5 * AL.isa(H_LOITER)["rho"] * V3 ** 2
    dq_ratio = (T_ref / A_DISC) / q3
    dDq_t = ac.drag_items["pod"] * 0.6 * dq_ratio
    rtr = evaluate(d, extra={"dDq": dDq_t, "dm": 0.3})
    out["propulsion_layout"] = {
        "pusher": {"endurance_h": E0},
        "tractor_estimate": {"endurance_h": rtr["endurance_h"], "dDq_m2": dDq_t, "dm_kg": 0.3,
                             "basis": "slipstream dynamic-pressure increase T/(A q) on 60 % of the pod wetted area at "
                                      "the 3000 m loiter, +0.3 kg longer nose leg for propeller clearance (estimate)"},
        "decision": "pusher",
        "reasons": ["two-stroke premix exhaust (oil mist) and the propeller disc stay behind the EO/IR turret and its "
                    "window; the turret keeps the chin with a free forward/nadir field of view",
                    "11 of 13 comparables with a known layout are pushers (comparables.yaml)",
                    "the propeller between the booms is guarded laterally and its wake drives the stabiliser"],
        "tractor_advantages_not_taken": ["propeller wash cools the cylinders on the ground (no CHT-limited ground "
                                         "runs)", "simpler cowl and cooling inlets"]}
    print(f"   tractor (estimate): E {rtr['endurance_h']:.2f} h vs pusher {E0:.2f} h")
    return out


# =====================================================================================================================
# 13. drawings
# =====================================================================================================================
def draw_sketch(ac: Aircraft, perf: dict, res: dict, path: Path):
    """3-view (plan, side, front) at one common scale from the actual OML/sections, internal layout as hidden lines,
    key dimensions annotated."""
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.patches import Circle, Ellipse, Polygon, Rectangle

    lay, pod, t, b_ = ac.layout, ac.pod, ac.tail, ac.boom
    g = lay["gear"]
    pf = ac.pf
    b2 = pf["b"] / 2
    R = EC.D_PROP / 2
    W_IN, H_IN = 18.0, 11.0
    sc = 1.30
    fig = plt.figure(figsize=(W_IN, H_IN), dpi=150)
    fill, skin, edge, dark = "#d6dbe1", "#e9ecf0", "#1f2a37", "#4b5563"
    acc, hid, boomc = "#b45309", "#2563eb", "#c9cfd6"

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

    def wheel_side(ax, xc, zc_):
        ax.add_patch(Circle((xc, zc_), WHEEL_D / 2, fc="#2b2f36", ec=edge, lw=0.8, zorder=6))
        ax.add_patch(Circle((xc, zc_), 0.045, fc="#9ca3af", ec=edge, lw=0.5, zorder=7))
        if ac.d.spats:
            ax.add_patch(Polygon(np.array([[xc - 0.17, zc_ + 0.02], [xc - 0.10, zc_ + 0.11], [xc + 0.12, zc_ + 0.10],
                                           [xc + 0.16, zc_ + 0.03], [xc + 0.12, zc_ - 0.01], [xc - 0.12, zc_ - 0.01]]),
                                 fc=skin, ec=edge, lw=0.8, zorder=8))

    xs = np.linspace(0.0, pod.L, 400)
    w, h, zc, nt, nb, tf = pod.section(xs)
    ztop, zbot = zc + tf * h, zc - (1 - tf) * h
    secs = ac.wing_secs
    ys = np.array([s_["y"] for s_ in secs])
    xle = np.array([s_["x_le"] for s_ in secs])
    ch = np.array([s_["chord"] for s_ in secs])
    L_tot = t["boom_x_end"]
    tk = lay["tank"]
    xcg, zcg = lay["cases"][0]["x"], lay["cases"][0]["z"]
    zt_ball = float(pod.z_bot(X_TURRET)[0]) - 0.04
    SP = pod.SPINNER

    # ---------------- plan view (span horizontal, nose up)
    axp = axes(0.35, 3.25, pf["b"] + 0.6, L_tot + 0.70, (-b2 - 0.3, b2 + 0.3), (-L_tot - 0.30, 0.40))
    wing_poly = np.vstack([np.column_stack([ys, -xle]), np.column_stack([ys[::-1], -(xle + ch)[::-1]])])
    wing_full = np.vstack([np.column_stack([-wing_poly[:, 0][::-1], wing_poly[:, 1][::-1]]), wing_poly])
    for sgn in (1, -1):         # booms below the wing
        axp.add_patch(Rectangle((sgn * b_["y"] - b_["r"], -t["boom_x_end"]), 2 * b_["r"],
                                t["boom_x_end"] - b_["x_tube0"], fc=boomc, ec=edge, lw=0.9, zorder=2))
    axp.add_patch(Polygon(np.column_stack([np.r_[w / 2, -w[::-1] / 2], np.r_[-xs, -xs[::-1]]]), fc=fill, ec=edge,
                          lw=1.0, zorder=3))
    axp.add_patch(Polygon(np.column_stack([[SP["d"] / 2, 0, -SP["d"] / 2], [-SP["x0"], -SP["x1"], -SP["x0"]]]),
                          fc="#9ca3af", ec=edge, lw=0.8, zorder=3))
    for xx in (0.55, X_PARACHUTE[0], X_PARACHUTE[1], tk["bay"][0], tk["bay"][1], pod.x_fw):
        wx = float(pod.section(xx)[0][0]) / 2
        axp.plot([-wx, wx], [-xx, -xx], color=dark, lw=0.5, zorder=4)
    axp.add_patch(Polygon(wing_full, fc=skin, ec=edge, lw=1.1, zorder=5))
    for sgn in (1, -1):
        y0, y1 = AIL_ETA[0] * b2, AIL_ETA[1] * b2
        yy = np.array([y0, y1])
        xh = np.interp(yy, ys, xle + 0.75 * ch)
        xt = np.interp(yy, ys, xle + ch)
        axp.plot(sgn * yy, -xh, color=dark, lw=0.6, zorder=6)
        axp.plot([sgn * y0, sgn * y0], [-xh[0], -xt[0]], color=dark, lw=0.6, zorder=6)
        axp.plot([sgn * y1, sgn * y1], [-xh[1], -xt[1]], color=dark, lw=0.6, zorder=6)
        ycs = pf["y_cs"]
        axp.plot([sgn * ycs, sgn * ycs], [-xle[1], -(xle[1] + ch[1])], color=acc, lw=0.9, ls="--", zorder=6)
        # boom nose fairing ahead of the wing
        xf0, xf1 = b_["x0"], secs[0]["x_le"] + 0.06
        axp.add_patch(Polygon(np.array([[sgn * b_["y"], -xf0], [sgn * (b_["y"] + 1.15 * b_["r"]), -(xf0 + 0.12)],
                                        [sgn * (b_["y"] + 1.15 * b_["r"]), -xf1], [sgn * (b_["y"] - 1.15 * b_["r"]), -xf1],
                                        [sgn * (b_["y"] - 1.15 * b_["r"]), -(xf0 + 0.12)]]),
                              fc=boomc, ec=edge, lw=0.8, zorder=6))
        fs_ = t["fin_secs"]
        fp_ = np.array([[fs_[0]["y"], -fs_[0]["x_le"]], [fs_[1]["y"], -fs_[1]["x_le"]],
                        [fs_[1]["y"], -(fs_[1]["x_le"] + fs_[1]["chord"])],
                        [fs_[0]["y"], -(fs_[0]["x_le"] + fs_[0]["chord"])]])
        fp_[:, 0] *= sgn
        axp.add_patch(Polygon(fp_, fc=skin, ec=edge, lw=0.9, zorder=8))
        axp.add_patch(Rectangle((sgn * g["track"] / 2 - WHEEL_W / 2, -g["x_mg"] - WHEEL_D / 2), WHEEL_W, WHEEL_D,
                                fc="none", ec=dark, lw=0.7, ls=":", zorder=6))
    bh_in = t["b_h"] / 2 - b_["r"]
    axp.add_patch(Rectangle((-bh_in, -t["x_te"]), 2 * bh_in, t["c_h"], fc=skin, ec=edge, lw=1.0, zorder=7))
    axp.plot([-bh_in, bh_in], [-(t["x_le_h"] + 0.70 * t["c_h"])] * 2, color=dark, lw=0.6, zorder=8)
    axp.plot([0, 0], [-(t["x_le_h"] + 0.70 * t["c_h"]), -t["x_te"]], color=dark, lw=0.6, zorder=8)
    axp.add_patch(Rectangle((-WHEEL_W / 2, -X_NG - 0.06 - WHEEL_D / 2), WHEEL_W, WHEEL_D, fc="none", ec=dark, lw=0.7,
                            ls=":", zorder=6))
    axp.add_patch(Rectangle((-R, -pod.X_PROP - 0.012), 2 * R, 0.024, fc="#6b7280", ec=edge, lw=0.6, zorder=4))
    axp.add_patch(Circle((0, -X_TURRET), TUR_D / 2, fc="none", ec=dark, lw=0.7, ls=":", zorder=6))
    axp.plot([0], [-xcg], marker="o", ms=7, mfc="white", mec=acc, mew=1.5, zorder=9)
    axp.plot([0], [-lay["x_np"]], marker="x", ms=7, color=acc, mew=1.5, zorder=9)
    axp.text(0.10, -xcg + 0.05, "AM", fontsize=8, color=acc, zorder=10)
    axp.text(0.10, -lay["x_np"] - 0.12, "NN", fontsize=8, color=acc, zorder=10)
    m_ = lay["wing_mac"]
    axp.plot([m_["y_mac"], m_["y_mac"]], [-m_["x_le_mac"], -(m_["x_le_mac"] + m_["mac"])], color=acc, lw=1.4, zorder=7)
    axp.text(m_["y_mac"] + 0.06, -m_["x_le_mac"] - m_["mac"] / 2, f"OAK {m_['mac']:.3f} m", fontsize=7.5, color=acc,
             va="center", zorder=9)
    dim(axp, (-b2, 0.24), (b2, 0.24), f"Kanat açıklığı b = {pf['b']:.2f} m")
    dim(axp, (-b2 - 0.15, 0.0), (-b2 - 0.15, -L_tot), f"Toplam uzunluk {L_tot:.2f} m", rot=90)
    dim(axp, (-pf["y_cs"], 0.06), (pf["y_cs"], 0.06), f"sabit veterli orta kanat {2 * pf['y_cs']:.2f} m", fs=7.5)
    dim(axp, (-b_["y"], -L_tot - 0.12), (b_["y"], -L_tot - 0.12), f"kuyruk kirişi aralığı {2 * b_['y']:.2f} m",
        off=(0, -0.10), fs=7.5)
    axp.text(b2 * 0.55, -0.30, f"S = {ac.S:.2f} m²   AR = {pf['AR']:.1f}   λ_dış = {ac.d.taper_o:.2f}\n"
             f"NLF(1)-0416 %16 sabit, dış panelde {ac.d.washout:.1f}° burulma, i = {lay['i_w']:.2f}°",
             fontsize=8.5, color=edge, ha="center")
    axp.text(-b2 * 0.55, -0.30, "turuncu kesik: dış panel bağlantısı\nsürekli ince: kanatçık / irtifa dümeni",
             fontsize=7.5, color=dark, ha="center")
    axp.text(-b2 - 0.25, 0.33, "ÜSTTEN GÖRÜNÜŞ", fontsize=10, weight="bold", color=edge)

    # ---------------- side view (nose left)
    zmin, zmax = g["z_g"] - 0.12, t["tip_z_fin"] + 0.18
    axs = axes(10.35, 6.85, L_tot + 0.6, zmax - zmin, (-0.25, L_tot + 0.35), (zmin, zmax))
    axs.plot([-0.2, L_tot + 0.3], [g["z_g"], g["z_g"]], color=dark, lw=0.8)
    for xx in np.arange(-0.2, L_tot + 0.3, 0.12):
        axs.plot([xx, xx - 0.06], [g["z_g"], g["z_g"] - 0.05], color="#9ca3af", lw=0.5)
    axs.add_patch(Polygon(np.column_stack([np.r_[xs, xs[::-1]], np.r_[ztop, zbot[::-1]]]), fc=fill, ec=edge, lw=1.0,
                          zorder=3))
    axs.plot(xs, zc, color=edge, lw=0.9, zorder=3)          # continuous chine line (shape language)
    for xx in (0.55, X_PARACHUTE[0], X_PARACHUTE[1], tk["bay"][0], tk["bay"][1], pod.x_fw):
        axs.plot([xx, xx], [float(pod.z_bot(xx)[0]), float(pod.z_top(xx)[0])], color=dark, lw=0.45, zorder=3)
    axs.add_patch(Polygon(np.array([[SP["x0"], Z_T + SP["d"] / 2], [SP["x1"], Z_T], [SP["x0"], Z_T - SP["d"] / 2]]),
                          fc="#9ca3af", ec=edge, lw=0.8, zorder=3))
    ls_ = oml.LiftingSurface(secs)
    P0 = ls_.section_points(0)
    axs.add_patch(Polygon(P0[:, [0, 2]], fc=skin, ec=edge, lw=1.0, zorder=5))
    xw0, xw1 = secs[0]["x_le"] - 0.08, secs[0]["x_le"] + ch[0] + 0.10
    xf = np.linspace(xw0, xw1, 40)
    zf_top = np.array([float(pod.z_top(x_)[0]) for x_ in xf])
    zsad = zf_top + 0.035 * np.sin(np.pi * (xf - xw0) / (xw1 - xw0))
    axs.fill_between(xf, zf_top - 0.001, zsad, color=fill, ec=edge, lw=0.6, zorder=4)
    axs.add_patch(Rectangle((pod.X_PROP - 0.012, Z_T - R), 0.024, 2 * R, fc="#6b7280", ec=edge, lw=0.6, zorder=2))
    # starboard boom (nearest the viewer), stabiliser edge-on, fin
    axs.add_patch(Rectangle((b_["x_tube0"], b_["z"] - b_["r"]), t["boom_x_end"] - b_["x_tube0"], 2 * b_["r"],
                            fc=boomc, ec=edge, lw=0.9, zorder=7, alpha=0.92))
    xf0, xf1 = b_["x0"], secs[0]["x_le"] + 0.06
    axs.add_patch(Polygon(np.array([[xf0, b_["z"] - 0.2 * b_["r"]], [xf0 + 0.12, b_["z"] + 1.15 * b_["r"]],
                                    [xf1, b_["z"] + 1.15 * b_["r"]], [xf1, b_["z"] - 1.15 * b_["r"]],
                                    [xf0 + 0.10, b_["z"] - 1.15 * b_["r"]]]), fc=boomc, ec=edge, lw=0.8, zorder=7))
    fs_ = t["fin_secs"]
    fside = np.array([[fs_[0]["x_le"], fs_[0]["z_le"]], [fs_[1]["x_le"], fs_[1]["z_le"]],
                      [fs_[1]["x_le"] + fs_[1]["chord"], fs_[1]["z_le"]], [fs_[0]["x_le"] + fs_[0]["chord"], fs_[0]["z_le"]]])
    axs.add_patch(Polygon(fside, fc=skin, ec=edge, lw=1.0, zorder=8))
    axs.plot([fs_[0]["x_le"] + 0.7 * fs_[0]["chord"], fs_[1]["x_le"] + 0.7 * fs_[1]["chord"]],
             [fs_[0]["z_le"], fs_[1]["z_le"]], color=dark, lw=0.6, zorder=8)
    th_h = 0.12 * t["c_h"] / 2
    axs.add_patch(Ellipse((t["x_le_h"] + 0.5 * t["c_h"], t["z_h"]), t["c_h"], 2 * th_h, fc=skin, ec=edge, lw=0.8,
                          zorder=6))
    xb_, zb_ = g["x_bumper"], g["z_bumper"]
    axs.add_patch(Polygon(np.array([[xb_ - 0.06, b_["z"] - b_["r"]], [xb_ + 0.02, b_["z"] - b_["r"]], [xb_, zb_]]),
                          fc="#9ca3af", ec=edge, lw=0.7, zorder=7))
    axs.add_patch(Circle((X_TURRET, zt_ball), TUR_D / 2, fc="#b8c0ca", ec=edge, lw=0.8, zorder=4))
    axs.add_patch(Ellipse((X_TURRET - 0.04, zt_ball - 0.012), 0.035, 0.06, fc="#1e3a8a", ec="none", zorder=5))
    for xg in (X_NG, g["x_mg"]):
        zf_ = float(pod.z_bot(xg)[0])
        za = g["z_g"] + WHEEL_D / 2
        xa_ = xg + (0.06 if xg == X_NG else 0.0)
        axs.add_patch(Polygon(np.array([[xg - 0.035, zf_ + 0.01], [xg + 0.035, zf_ + 0.01], [xa_ + 0.02, za + 0.05],
                                        [xa_ - 0.02, za + 0.05]]), fc="#9ca3af", ec=edge, lw=0.7, zorder=2))
        wheel_side(axs, xa_, za)
    boxes = [((0.14, 0.55), (-0.12, 0.10), "aviyonik\nbatarya"),
             ((X_PARACHUTE[0], X_PARACHUTE[1]), (0.09, 0.21), "paraşüt"),
             ((tk["cells"]["forward"]["x0"], tk["cells"]["forward"]["x1"]), (-0.16, 0.20), "yakıt"),
             ((tk["bay"][0], tk["bay"][1]), (-0.17, 0.05), "faydalı\nyük"),
             ((tk["cells"]["aft"]["x0"], tk["cells"]["aft"]["x1"]), (-0.15, 0.21), "yakıt"),
             ((pod.X_HUB - ENV_L_SG, pod.X_HUB), (Z_T - (ENV_H - ENV_ZC), Z_T + ENV_ZC), "L 275\nEF")]
    for (x0_, x1_), (z0_, z1_), lab in boxes:
        axs.add_patch(Rectangle((x0_, z0_), x1_ - x0_, z1_ - z0_, fc="none", ec=hid, lw=0.7, ls=(0, (3, 2)), zorder=9))
        axs.text(0.5 * (x0_ + x1_), z0_ + 0.035 if lab.startswith("L 275") else 0.5 * (z0_ + z1_), lab, fontsize=6.0,
                 color=hid, ha="center", va="center", zorder=9, bbox=dict(fc="white", ec="none", pad=0.3, alpha=0.75))
    axs.plot([xcg], [zcg], marker="o", ms=7, mfc="white", mec=acc, mew=1.5, zorder=9)
    th = math.radians(g["prop_strike_deg"])
    x_end = pod.X_PROP + 0.05
    axs.plot([g["x_mg"], x_end], [g["z_g"], g["z_g"] + (x_end - g["x_mg"]) * math.tan(th)], color=acc, lw=0.6, ls="--",
             zorder=1)
    axs.text(g["x_mg"] + 0.10, g["z_g"] - 0.085, f"pervane temas açısı {g['prop_strike_deg']:.1f}°, "
             f"kuyruk kirişi tamponu {g['bumper_contact_deg']:.1f}°", fontsize=6.8, color=acc,
             bbox=dict(fc="white", ec="none", pad=0.4, alpha=0.85), zorder=20)
    dim(axs, (pod.X_PROP + 0.06, g["z_g"]), (pod.X_PROP + 0.06, Z_T - R), f"{g['prop_clear_level'] * 1000:.0f} mm",
        off=(0.14, 0), fs=7.2)
    ztop_all = t["tip_z_fin"]
    dim(axs, (-0.15, g["z_g"]), (-0.15, ztop_all), f"Yükseklik {ztop_all - g['z_g']:.2f} m", rot=90, fs=7.5)
    axs.text(-0.2, zmax - 0.07, "YANDAN GÖRÜNÜŞ", fontsize=10, weight="bold", color=edge)

    # ---------------- front view
    axf = axes(0.35, 0.35, pf["b"] + 0.6, zmax - zmin, (-b2 - 0.3, b2 + 0.3), (zmin, zmax))
    axf.plot([-b2 - 0.25, b2 + 0.25], [g["z_g"], g["z_g"]], color=dark, lw=0.8)
    axf.add_patch(Circle((0, Z_T), R, fc="none", ec="#6b7280", lw=0.7, ls="--", zorder=1))
    top, bot = [], []
    for i in range(len(secs)):
        P = ls_.section_points(i)
        top.append([P[:, 1].mean(), P[:, 2].max()])
        bot.append([P[:, 1].mean(), P[:, 2].min()])
    top, bot = np.array(top), np.array(bot)
    for sgn in (1, -1):
        poly = np.vstack([top, bot[::-1]])
        poly[:, 0] *= sgn
        axf.add_patch(Polygon(poly, fc=skin, ec=edge, lw=1.0, zorder=4))
        axf.add_patch(Circle((sgn * b_["y"], b_["z"]), b_["r"], fc=boomc, ec=edge, lw=0.9, zorder=5))
        fr = np.array([fs_[0]["y"], fs_[0]["z_le"]])
        ft = np.array([fs_[1]["y"], fs_[1]["z_le"]])
        dv = (ft - fr) / np.linalg.norm(ft - fr)
        nv = np.array([-dv[1], dv[0]])
        t0_, t1_ = 0.12 * fs_[0]["chord"] / 2, 0.12 * fs_[1]["chord"] / 2
        fpoly = np.array([fr + nv * t0_, ft + nv * t1_, ft - nv * t1_, fr - nv * t0_])
        fpoly[:, 0] *= sgn
        axf.add_patch(Polygon(fpoly, fc=skin, ec=edge, lw=1.0, zorder=5))
        za = g["z_g"] + WHEEL_D / 2
        zf_ = float(pod.z_bot(g["x_mg"])[0])
        axf.add_patch(Polygon(np.array([[sgn * 0.08, zf_ + 0.01], [sgn * 0.17, zf_ + 0.01],
                                        [sgn * (g["track"] / 2 - 0.015), za + 0.03],
                                        [sgn * (g["track"] / 2 - 0.06), za]]), fc="#9ca3af", ec=edge, lw=0.7, zorder=2))
        axf.add_patch(Rectangle((sgn * g["track"] / 2 - WHEEL_W / 2, g["z_g"]), WHEEL_W, WHEEL_D * 0.6, fc="#2b2f36",
                                ec=edge, lw=0.7, zorder=3))
        if ac.d.spats:
            axf.add_patch(Ellipse((sgn * g["track"] / 2, za + 0.02), 0.10, 0.18, fc=skin, ec=edge, lw=0.8, zorder=4))
    axf.add_patch(Rectangle((-t["b_h"] / 2, t["z_h"] - 0.12 * t["c_h"] / 2), t["b_h"], 0.12 * t["c_h"], fc=skin,
                            ec=edge, lw=0.9, zorder=4))
    phi = np.linspace(0, 2 * math.pi, 240)
    xm_ = 0.5 * (tk["bay"][0] + tk["bay"][1])
    Pf = pod.fus.point(np.full_like(phi, xm_), phi)
    Pe = pod.fus.point(np.full_like(phi, pod.X_HUB - 0.12), phi)
    axf.add_patch(Polygon(Pe[:, 1:], fc=fill, ec=edge, lw=0.6, ls="--", zorder=4))
    axf.add_patch(Polygon(Pf[:, 1:], fc=fill, ec=edge, lw=1.0, zorder=6))
    axf.add_patch(Circle((0, Z_T), SP["d"] / 2, fc="none", ec=dark, lw=0.6, ls="--", zorder=6))
    axf.add_patch(Circle((0, zt_ball), TUR_D / 2, fc="#b8c0ca", ec=edge, lw=0.8, zorder=7))
    axf.add_patch(Ellipse((0, zt_ball - 0.012), 0.05, 0.06, fc="#1e3a8a", ec="none", zorder=8))
    axf.add_patch(Rectangle((-0.018, g["z_g"] + WHEEL_D * 0.5), 0.036, float(pod.z_bot(X_NG)[0]) - g["z_g"] - WHEEL_D * 0.5,
                            fc="#9ca3af", ec=edge, lw=0.6, zorder=1))
    axf.add_patch(Rectangle((-WHEEL_W / 2, g["z_g"]), WHEEL_W, WHEEL_D * 0.6, fc="#2b2f36", ec=edge, lw=0.7, zorder=2))
    if ac.d.spats:
        axf.add_patch(Ellipse((0, g["z_g"] + WHEEL_D / 2 + 0.02), 0.10, 0.18, fc=skin, ec=edge, lw=0.8, zorder=3))
    dim(axf, (-g["track"] / 2, g["z_g"] - 0.075), (g["track"] / 2, g["z_g"] - 0.075), f"İz {g['track']:.2f} m", fs=7.5)
    zdim = max(secs[-1]["z_le"], t["tip_z_fin"]) + 0.08      # span dimension above the fin tips
    axf.annotate("", xy=(b2, zdim), xytext=(-b2, zdim), arrowprops=dict(arrowstyle="<->", color=dark, lw=0.8,
                                                                         shrinkA=0, shrinkB=0))
    axf.text(-b2 * 0.55, zdim, f"b = {pf['b']:.2f} m  (dış panel dihedral {ac.d.dihedral_o:.1f}°)", ha="center",
             va="center", fontsize=8, color=dark, bbox=dict(fc="white", ec="none", pad=0.6), zorder=20)
    axf.text(t["tip_y_fin"] + 0.20, t["tip_z_fin"] - 0.10, f"çift dikey kuyruk: {ac.d.fin_cant:.0f}° dışa yatık, "
             f"yükseklik {t['h_fin']:.2f} m", ha="left", va="center", fontsize=7.5, color=dark, zorder=20)
    axf.text(-b2 - 0.25, zdim + 0.07, "ÖNDEN GÖRÜNÜŞ", fontsize=10, weight="bold", color=edge, va="bottom")

    # ---------------- title + data block
    fig.text(0.35 / W_IN, 10.55 / H_IN, "YK-250  —  Konsept “Üretim” (itici, çift kuyruk kirişi, H-kuyruk)", fontsize=16,
             weight="bold", color=edge)
    fig.text(0.35 / W_IN, 10.25 / H_IN, "Sivil EO/IR gözetleme ve araştırma platformu · silah/askı noktası yok · ilk "
             "boyutlandırma (calc.py) · tüm görünüşler aynı ölçekte · ölçüler m", fontsize=9.5, color=dark)
    P = perf
    loi = P[int(H_LOITER)]["loiter"]
    rows = [
        ("MTOM / boş / yakıt / faydalı yük", f"{res['m0']:.1f} / {res['empty']:.1f} / {res['fuel']:.1f} / {PAYLOAD:.1f} kg"),
        ("Kanat S, b, AR, W/S", f"{ac.S:.2f} m², {pf['b']:.2f} m, {pf['AR']:.1f}, {ac.d.ws / G:.1f} kg/m²"),
        ("Kök / uç veteri, OAK", f"{pf['cr']:.3f} / {pf['ct']:.3f} m, {pf['mac']:.3f} m"),
        ("Gövde uzunluğu / genişlik", f"{pod.L:.2f} m (+ spinner {pod.SPINNER['x1']:.2f}) / {lay['w_pod']:.2f} m"),
        ("CD0 / e / (L/D)maks", f"{ac.fit['cd0']:.4f} / {ac.fit['e']:.3f} / {ac.fit['LD_max']:.1f}"),
        ("VS temiz (DS, MTOM)", f"{P[0]['VS_clean_m_s']:.1f} m/s (flap yok)"),
        ("Bekleme 3000 m (EAS, TAS)", f"{loi['EAS']:.1f}, {loi['V']:.1f} m/s · {loi['ff_kg_h']:.2f} kg/h · "
                                      f"{loi['rpm']:.0f} dev/dk"),
        ("Vmaks DS / 3000 m", f"{P[0]['V_max_m_s']:.1f} / {P[int(H_LOITER)]['V_max_m_s']:.1f} m/s"),
        ("Dayanım (görev, %10 yedek)", f"{res['endurance_h']:.1f} h"),
        ("Menzil (feribot, %10 yedek)", f"{res['range_km']:.0f} km"),
        ("Tırmanma DS / 3000 m", f"{P[0]['RoC_max_m_s']:.2f} / {P[int(H_LOITER)]['RoC_max_m_s']:.2f} m/s"),
        ("Servis tavanı", f"{P['ceiling_service_m']:.0f} m"),
        ("Kalkış / iniş yerde koşu", f"{P['takeoff_sl_mtow']['ground_roll_m']:.0f} / "
                                     f"{P['landing_sl_end_of_mission']['ground_roll_m']:.0f} m "
                                     f"(dönüş {P['takeoff_sl_mtow']['V_rotation_authority_m_s']:.1f} m/s)"),
        ("Statik marj (tüm yüklemeler)", f"{min(lay['sms']):.3f} – {max(lay['sms']):.3f}"),
        ("V_H / V_V", f"{res['VH']:.3f} / {res['VV']:.4f}"),
        ("Pervane", f"Mejzlik 32x18 2B, statik uç Mach {P['static_wot']['tip_mach']:.2f}"),
        ("Pervane–yer / pervane–kuyruk kirişi", f"{g['prop_clear_level'] * 1000:.0f} mm / {res['prop_boom_mm']:.0f} mm"),
        ("En uzun taşıma parçası", f"{res['longest_piece_m']:.2f} m · kalıp sayısı {res['moulds']}"),
    ]
    y0 = 6.25
    fig.text(10.35 / W_IN, (y0 + 0.3) / H_IN, "ANA DEĞERLER", fontsize=10, weight="bold", color=edge)
    for i, (k, v) in enumerate(rows):
        yy = (y0 - 0.31 * i) / H_IN
        fig.text(10.35 / W_IN, yy, k, fontsize=8.6, color=dark)
        fig.text(13.35 / W_IN, yy, v, fontsize=8.6, color=edge, weight="bold")
    fig.text(10.35 / W_IN, 0.35 / H_IN, "AM: ağırlık merkezi (tüm yüklemelerde) · NN: nötr nokta · OAK: ortalama "
             "aerodinamik kiriş\nnoktalı: görünmeyen · mavi kesik: iç yerleşim · gri: karbon boru kuyruk kirişleri",
             fontsize=7.5, color=dark)
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
    ax.axvline(cd["ws_stall_24"] / G, color="#111827", lw=1.2, ls="--", label="VS ≤ 24 m/s (temiz, flapsız)")
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
    ax.set_title("YK-250 “Üretim” – kısıt diyagramı (sizinglib)")
    ax.grid(alpha=0.3)
    ax.legend(fontsize=7.5, loc="upper left")
    fig.tight_layout()
    fig.savefig(path, facecolor="white")
    plt.close(fig)


# =====================================================================================================================
# 14. main
# =====================================================================================================================
QUICK_CHOICES = {"taper_o": 0.6, "washout": 3.0, "y_cs": 1.00, "tail_gap": 1.05, "AR": 12.0, "spats": False}


def main():
    t0 = time.time()
    quick = "--quick" in sys.argv
    base = Design()
    if quick:
        d = replace(base, **QUICK_CHOICES)
        d = replace(d, ws=ws_stall_corner(d))
        trades = {"note": "--quick run: trades skipped (development only, not the published result)"}
    else:
        base = replace(base, ws=ws_stall_corner(base))
        d, trades = run_wing_tail_trades(base)

    print("\n[final] design point")
    res = evaluate(d, verbose=True)
    ac, aero, mis = res["ac"], res["aero"], res["mission"]
    m0, fuel = res["m0"], res["fuel"]
    if not quick:
        trades.update(run_config_trades(d, res))
        res = evaluate(d)                                  # restore EC globals (propeller) and the final aircraft
        ac, aero, mis = res["ac"], res["aero"], res["mission"]
        m0, fuel = res["m0"], res["fuel"]
    lay = ac.layout
    t = ac.tail

    # closed MassModel loop (sizinglib) for the mission flown above: must return the design MTOM
    mm = SZ.MassModel(PAYLOAD, fixed_mass(d) * (1 + GROWTH), mis["ff"], airframe_fn_factory(ac))
    mm_sol = mm.solve(m0_guess=120.0)
    print(f"\n[mass model] MTOW from sizinglib.MassModel = {mm_sol['mtow']:.2f} kg (design {m0:.2f} kg)")
    print("[10 h mission] closed-loop MTOW for the baseline 10 h requirement")
    r10 = evaluate(d, mode="endurance", endurance_target_h=END_REQ_H, verbose=True)
    res = evaluate(d)
    ac, aero, mis = res["ac"], res["aero"], res["mission"]
    lay, t = ac.layout, ac.tail

    rng = EC.solve_range(ac, m0, fuel)
    perf = performance(ac, m0, fuel)
    cdiag = EC.constraint_diagram(ac, m0, perf)
    VH_level = perf[0]["V_max_m_s"]
    VD = max(1.25 * VC_EAS, VH_level, BL["design_speeds_m_per_s_eas"]["VD"]["value"])
    struct = structures_summary(ac, m0, VD)
    pk = packaging_checks(ac)
    pi = production_indicators(ac)
    smg = stall_margins(aero)

    # stability details
    fp = ac.pod.props()
    S_v_eff = t["S_v_eff"]
    tv = SL.tail_volumes(ac.S, ac.pf["mac"], ac.pf["b"], t["S_h"], lay["l_h"], S_v_eff, lay["l_v"])
    cnb_fin = SL.cn_beta_vertical(t["a_v"], S_v_eff, lay["l_v"], ac.S, ac.pf["b"], 0.9, 0.1)
    cnb_fus_raymer = -1.3 * fp["volume"] / (ac.S * ac.pf["b"]) * (fp["h_max"] / fp["w_max"])
    cnb_fus_datcom = SL.cn_beta_fuselage(0.0015, 1.75, fp["S_side"], ac.pod.L, ac.S, ac.pf["b"])

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
    tkb = lay["tank"]["bay"]
    aux_cap = K_TANK * ac.pod.internal_area(0.5 * (tkb[0] + tkb[1])) * (tkb[1] - tkb[0]) * RHO_FUEL / 1.02
    aux_fuel = min(M_RESEARCH, aux_cap)
    sens["max_endurance_baseline_sensors_aux_bladder"] = EC.solve_loiter_for_fuel(ac, m0, fuel + aux_fuel)["t_air_s"] / 3600
    sens["aux_bladder_fuel_kg"] = aux_fuel
    sens["aux_bladder_capacity_kg"] = aux_cap

    # flags (honest open points)
    loi3, loi0 = perf[int(H_LOITER)]["loiter"], perf[0]["loiter"]
    if loi0["rpm"] < 4000:
        flag(f"sea-level loiter at {loi0['rpm']:.0f} rpm is below the Limbach 4000-6000 rpm regular-flight band "
             "(3000 m loiter: " + f"{loi3['rpm']:.0f} rpm); confirm low-load operation with Limbach or loiter >= 1500 m")
    if perf[0]["RoC_max_m_s"] < ROC_REQ:
        flag(f"sea-level rate of climb {perf[0]['RoC_max_m_s']:.2f} m/s is below the 4.9 m/s baseline target with the "
             "32x18 2B propeller (propeller-limited, not engine-limited); the 31x12 3B option trades endurance for climb")
    for case, lp in (("3000 m", loi3), ("sea level", loi0)):
        if lp["gen_W"] < P_ELEC * 1.2:
            flag(f"generator margin at the {case} loiter only {lp['gen_W'] / P_ELEC:.2f}x the 303 W load")
    if min(loi0["power_fraction"], loi3["power_fraction"]) < EC.BSFC_PTS[0, 0]:
        flag(f"loiter power fraction {min(loi0['power_fraction'], loi3['power_fraction']):.3f} is below the lowest BSFC "
             "point (0.20): BSFC linearly extrapolated; a light-load dyno map is needed")
    if res["empty"] / m0 > 0.62:
        flag(f"empty-mass fraction {res['empty'] / m0:.3f} (incl. 5 % growth allowance) is above the comparables range "
             f"0.55-0.62; without the allowance {res['eb']['empty'] / (1 + GROWTH) / m0:.3f}: the aluminium chassis, "
             "twin booms and bolt-on shell cost mass (production choice, see trades)")
    flag("stabiliser in the propeller slipstream: 2/rev blade-passage pressure loading and acoustic fatigue of the "
         "stabiliser and elevators must be covered by a vibration survey and a fatigue test; slipstream-dependent pitch "
         "trim (power changes) to be handled by the autopilot")
    flag(f"boom/tail flutter (boom bending {struct['booms']['f1_bending_Hz']:.0f} Hz, torsion, stabiliser bending) and "
         f"the wing (AR {ac.pf['AR']:.0f}) are not analysed here; CS-LUAS.629 clearance to 1.2 VD = {1.2 * VD:.1f} m/s "
         "needs a GVT and rational analysis (Report 45 not valid for boom tails)")
    flag("aft-pod closure from 0.45 m to the 0.30 m cowl lip relies on the pusher inflow and the cooling exit flow "
         "(same as the endurance chain); confirm with CFD; the 80 mm hub spacer needs Limbach approval")
    if perf["landing_sl_mtow"]["ground_roll_m"] > FIELD_ROLL_MAX:
        flag(f"abort landing at MTOM needs a {perf['landing_sl_mtow']['ground_roll_m']:.0f} m ground roll (> 200 m "
             "factored target, < 300 m strip): no flaps and the shoulder-wing incidence keep lift on the wheels; "
             "aileron-up lift dumping and an operating limit (fuel jettison is not provided) cover it")
    flag("rotation speed relies on the slipstream over the stabiliser (actuator-disk estimate); without slipstream "
         f"credit the take-off roll is {perf['takeoff_sl_mtow_no_slipstream_credit']['ground_roll_m']:.0f} m "
         f"(lift-off {perf['takeoff_sl_mtow_no_slipstream_credit']['V_lof_m_s']:.1f} m/s); verify in taxi tests")

    # console table
    P = perf
    L_tot = t["boom_x_end"]
    rows = [
        ("MTOM design / cap", f"{m0:.1f} / {MTOM_CAP:.1f} kg (sizinglib.MassModel: {mm_sol['mtow']:.2f} kg)"),
        ("Empty / fuel / payload", f"{res['empty']:.1f} / {fuel:.1f} / {PAYLOAD:.1f} kg (empty frac {res['empty'] / m0:.3f})"),
        ("MTOW for the 10 h mission", f"{r10['m0']:.1f} kg (fuel {r10['fuel']:.1f} kg)"),
        ("Wing S / b / AR", f"{ac.S:.3f} m2 / {ac.pf['b']:.2f} m / {ac.pf['AR']:.1f}"),
        ("Chords root / tip / MAC, y_cs", f"{ac.pf['cr']:.3f} / {ac.pf['ct']:.3f} / {ac.pf['mac']:.3f} m, "
                                          f"{ac.pf['y_cs']:.2f} m"),
        ("W/S", f"{d.ws:.0f} Pa = {d.ws / G:.1f} kg/m2"),
        ("Length pod / overall, width", f"{ac.pod.L:.2f} / {L_tot:.2f} m, {lay['w_pod']:.2f} m"),
        ("Stab S / fins S / l_h", f"{t['S_h']:.3f} m2 (c {t['c_h']:.3f}) / 2 x {t['S_fin_each']:.3f} m2 / {lay['l_h']:.2f} m"),
        ("V_H / V_V", f"{tv['V_H']:.3f} / {tv['V_V']:.4f}"),
        ("Booms", f"2 x {struct['booms']['L_m']:.2f} m, 80 x {struct['booms']['t_wall_m'] * 1e3:.1f} mm, "
                  f"f1 {struct['booms']['f1_bending_Hz']:.1f} Hz"),
        ("CD0 / e / k / (L/D)max", f"{ac.fit['cd0']:.4f} / {ac.fit['e']:.3f} / {ac.fit['k']:.4f} / {ac.fit['LD_max']:.2f}"),
        ("CL_alpha / e_inv / CLmax trimmed", f"{aero['CL_alpha']:.3f} /rad / {aero['e_inv']:.3f} / "
                                             f"{ac.clmax['clean_trimmed']:.3f}"),
        ("Stall onset eta / aileron cl ratio", f"{smg['stall_onset_eta']:.2f} / {smg['aileron_cl_over_clmax_at_stall']:.3f}"),
        ("VS clean (SL, MTOM)", f"{P[0]['VS_clean_m_s']:.1f} m/s"),
        ("Loiter 3000 m (EAS/TAS, ff)", f"{loi3['EAS']:.1f} / {loi3['V']:.1f} m/s, {loi3['ff_kg_h']:.2f} kg/h, "
                                        f"{loi3['rpm']:.0f} rpm, PF {loi3['power_fraction']:.3f}"),
        ("V min power / V (L/D)max (SL)", f"{P[0]['V_min_power_polar_m_s']:.1f} / {P[0]['V_LDmax_polar_m_s']:.1f} m/s"),
        ("Best-range speed 3000 m", f"{P[int(H_LOITER)]['best_range']['V']:.1f} m/s"),
        ("Vmax SL / 3000 m", f"{P[0]['V_max_m_s']:.1f} / {P[int(H_LOITER)]['V_max_m_s']:.1f} m/s"),
        ("Endurance (mission, 10 % reserve)", f"{res['endurance_h']:.2f} h (loiter {mis['t_loiter_s'] / 3600:.2f} h)"),
        ("Range (ferry, 3000 m, 10 % reserve)", f"{rng['range_m'] / 1000:.0f} km"),
        ("Endurance sensitivity", f"BSFC -12/+12 %: {sens['bsfc_minus_12pct']:.1f}/{sens['bsfc_plus_12pct']:.1f} h, "
                                  f"CD0 +10 %: {sens['cd0_plus_10pct']:.1f} h, empty +5 %: {sens['empty_plus_5pct']:.1f} h"),
        ("RoC SL / 3000 m", f"{P[0]['RoC_max_m_s']:.2f} / {P[int(H_LOITER)]['RoC_max_m_s']:.2f} m/s"),
        ("Ceiling service / absolute", f"{P['ceiling_service_m']:.0f} / {P['ceiling_absolute_m']:.0f} m"),
        ("TO roll / to 15 m (SL, MTOM)", f"{P['takeoff_sl_mtow']['ground_roll_m']:.0f} / "
                                         f"{P['takeoff_sl_mtow']['distance_15m_m']:.0f} m (V_R "
                                         f"{P['takeoff_sl_mtow']['V_rotation_authority_m_s']:.1f} m/s, "
                                         f"{P['takeoff_sl_mtow']['liftoff_mode']})"),
        ("Landing roll end-of-mission / MTOM", f"{P['landing_sl_end_of_mission']['ground_roll_m']:.0f} / "
                                               f"{P['landing_sl_mtow']['ground_roll_m']:.0f} m"),
        ("Static margin (all cases)", f"{min(lay['sms']):.3f} - {max(lay['sms']):.3f}"),
        ("CG x (all cases) / NP x", f"{min(c['x'] for c in lay['cases']):.3f}-{max(c['x'] for c in lay['cases']):.3f} / "
                                    f"{lay['x_np']:.3f} m"),
        ("Cn_beta fins / pod / total", f"{cnb_fin:.3f} / {t['cnb_fus']:.3f} / {cnb_fin + t['cnb_fus']:.3f} /rad"),
        ("Prop clearance level / LOF / boom", f"{lay['gear']['prop_clear_level']:.3f} / {lay['gear']['prop_clear_lof']:.3f}"
                                              f" / {pk['prop_tip_to_boom_radial_m']:.3f} m"),
        ("Static tip Mach / speed", f"{P['static_wot']['tip_mach']:.3f} / {P['static_tip_speed_m_s']:.0f} m/s"),
        ("Gust limit n (VC 45, 4500 m)", f"{lay['gust']['n_limit']:.2f}, ultimate {FOS * lay['gust']['n_limit']:.2f}"),
        ("Production", f"{pi['mould_count']} moulds, {pi['servo_count']} servos, longest piece {pi['longest_piece_m']:.2f} m"),
    ]
    print("\n" + "=" * 100)
    print("YK-250 concept 'production' (twin boom, H tail, pusher) - results")
    print("=" * 100)
    for k_, v_ in rows:
        print(f"  {k_:<40s} {v_}")
    print("  flags:")
    for f_ in FLAGS:
        print("   -", f_)

    # concept.yaml
    wm = lay["wing_mac"]
    gear = lay["gear"]
    groups = ac.mass["groups"]
    y = {
        "meta": {"project": "YK-250 (ucav250)", "concept_key": "production",
                 "name_tr": "Üretim (en düşük maliyet, risk ve bakım yükü)",
                 "phase": "concept trade study - first-pass sizing", "date": "2026-10-05",
                 "generated_by": "ucav250/data/concepts/production/calc.py",
                 "run": "PYTHONPATH=. python3 ucav250/data/concepts/production/calc.py",
                 "quick_run": quick,
                 "frame": "X aft from the nose tip, Y starboard, Z up from the pod datum line through the nose tip; SI "
                          "units, angles in degrees",
                 "scope": "civil EO/IR surveillance and research platform with a UCAV look; payload is sensors and "
                          "mission equipment only - no weapons, hardpoints, pylons or release mechanisms",
                 "libraries": ["analysis/aerolib.py", "analysis/stablib.py", "analysis/sizinglib.py",
                               "analysis/structlib.py", "analysis/aero.py", "design/oml.py"],
                 "shared_chain": {"module": "ucav250/data/concepts/endurance/calc.py (imported, not modified)",
                                  "functions": ["Prop (Mejzlik table, J-similarity, density scaling)", "lapse",
                                                "engine_torque_wot", "bsfc_g_kwh (part load incl. generator)",
                                                "tripped_chars / strip_profile_drag", "build_aero", "fit_polar",
                                                "level_point", "best_loiter", "best_range", "climb_point", "fly",
                                                "solve_loiter_for_fuel", "solve_loiter_for_endurance", "solve_range",
                                                "ceiling", "gust_limit", "vn_summary", "constraint_diagram",
                                                "loading_cases", "gear_mass", "surface_mac"],
                                  "this_file": ["Pod geometry", "constant-chord centre + tapered outer wing",
                                                "booms and H tail (V tail for the trade)", "drag build-up items",
                                                "wing/boom/tail/chassis/shell masses", "layout and CG",
                                                "H-tail stability", "slipstream-aided rotation and take-off",
                                                "ground geometry", "trades", "packaging", "3-view"]},
                 "runtime_s": time.time() - t0},
        "configuration": {
            "propulsion": "single Limbach L 275 EF pusher at the aft end of the pod, Mejzlik 32x18 2B on an 80 mm hub "
                          "spacer between the tail booms, annular cooling exit around the spinner, cowl flap",
            "tail": f"twin CFRP tube booms ({2 * d.y_boom:.2f} m apart) + constant-chord H stabiliser in the propeller "
                    f"slipstream (split elevator) + two identical fins, {d.fin_le_sweep:.0f} deg LE sweep, "
                    f"{d.fin_cant:.0f} deg outward cant",
            "wing_position": "shoulder: constant-chord centre section bolted on the pod chassis (4 bolts), tapered outer "
                             "panels with spar tongue + 2 pins + drag pin at y = +/-" + f"{ac.pf['y_cs']:.2f} m",
            "gear": "fixed tricycle: GFRP spring bow with faired legs + TOST 200x50 wheels " +
                    ("with quick-release spats" if d.spats else "without spats (trade: spats buy < 0.1 h)") +
                    ", steerable nose leg",
            "fuselage": f"short chined pod {ac.pod.L:.2f} m long, {lay['w_pod']:.2f} m wide: continuous chine rising "
                        "from the nose to the thrust line, dorsal engine hump behind the wing",
            "planform": f"constant chord {ac.pf['cr']:.3f} m to y = {ac.pf['y_cs']:.2f} m, then linear taper "
                        f"{d.taper_o:.2f} to the tip, straight unswept quarter chord, NLF(1)-0416 16 % throughout, "
                        f"{d.washout:.1f} deg linear washout on the outer panels, no flaps",
            "structure": "chassis (aluminium sheet metal + extrusions, 7075 machined hard points, 4130 engine truss) "
                         "carries every point load; bolt-on composite shell panels are non-structural; wing and tail in "
                         "OOA prepreg (one composite process line)",
            "layout": "payload bay on the CG between two identical interconnected fuel cells; EO/IR turret in the chin; "
                      "battery, avionics and parachute forward; engine aft",
            "transport_breakdown": pi["transport_pieces"],
        },
        "summary": {
            "mtow_kg": m0, "empty_kg": res["empty"], "fuel_kg": fuel, "payload_kg": PAYLOAD,
            "endurance_h": res["endurance_h"], "range_km": rng["range_m"] / 1000, "span_m": ac.pf["b"],
            "length_m": L_tot, "area_m2": ac.S, "AR": ac.pf["AR"], "ld_max": ac.fit["LD_max"],
            "mtow_for_10h_mission_kg": r10["m0"],
            "variant_10h_mission": {"mtow_kg": r10["m0"], "fuel_kg": r10["fuel"], "empty_kg": r10["empty"],
                                    "S_m2": r10["ac"].S, "span_m": r10["ac"].pf["b"], "endurance_h": r10["endurance_h"],
                                    "basis": "same W/S, AR and layout; MTOW from sizinglib.MassModel with the 10 h "
                                             "mission fuel fraction (geometry re-sized each iteration)"},
            "max_endurance_baseline_sensors_aux_bladder_h": sens["max_endurance_baseline_sensors_aux_bladder"],
            "feasible": bool(res["endurance_h"] >= END_REQ_H and m0 < MTOM_CAP and
                             perf["takeoff_sl_mtow"]["ground_roll_m"] <= FIELD_ROLL_MAX and
                             perf["landing_sl_end_of_mission"]["ground_roll_m"] <= FIELD_ROLL_MAX),
        },
        "inputs": INPUTS,
        "trades": trades,
        "geometry": {
            "wing": {"S_m2": ac.S, "b_m": ac.pf["b"], "AR": ac.pf["AR"], "taper_outer": d.taper_o,
                     "taper_overall": ac.pf["taper"], "root_chord_m": ac.pf["cr"], "tip_chord_m": ac.pf["ct"],
                     "centre_section_half_span_m": ac.pf["y_cs"], "outer_panel_span_m": ac.pf["outer_panel_span"],
                     "mac_m": wm["mac"], "mac_le_x_m": wm["x_le_mac"], "mac_y_m": wm["y_mac"],
                     "x_le_root_m": lay["x_w"], "z_le_root_m": Z_WING_ROOT, "incidence_deg": lay["i_w"],
                     "washout_outer_deg": d.washout, "dihedral_outer_deg": d.dihedral_o, "sweep_c4_deg": 0.0,
                     "incidence_rule": lay["incidence_rule"], "sections": ac.wing_secs,
                     "airfoils": {"all": "nlf416 (NASA NLF(1)-0416), thickness_scale " + f"{d.ts_tip}"},
                     "ailerons": {"span_eta": list(AIL_ETA), "chord_fraction": 0.25}, "flaps": None,
                     "S_wet_m2": ac.mass["wing"]["S_wet"]},
            "tail": {"type": "twin boom + H tail", "stabiliser": {"S_m2": t["S_h"], "span_m": t["b_h"],
                                                                   "chord_m": t["c_h"], "x_le_m": t["x_le_h"],
                                                                   "z_m": t["z_h"], "airfoil": "n0012",
                                                                   "elevator_chord_fraction": 0.30,
                                                                   "sections": t["stab_secs"]},
                     "fins": {"count": 2, "S_each_m2": t["S_fin_each"], "height_m": t["h_fin"],
                              "root_chord_m": t["c_fin_root"], "tip_chord_m": t["c_fin_tip"],
                              "le_sweep_deg": d.fin_le_sweep, "cant_out_deg": d.fin_cant, "airfoil": "n0012",
                              "rudder_chord_fraction": 0.30, "sections_starboard": t["fin_secs"],
                              "S_v_eff_m2": t["S_v_eff"]},
                     "booms": {"y_m": ac.boom["y"], "z_axis_m": ac.boom["z"], "x_fairing_nose_m": ac.boom["x0"],
                               "x_tube_front_m": ac.boom["x_tube0"],
                               "x_end_m": t["boom_x_end"], "d_m": d.d_boom, "wall_m": struct["booms"]["t_wall_m"],
                               "material": "roll-wrapped CFRP tube (60 % 0 deg / 40 % +/-45 deg), bought"},
                     "x_ac_h_m": t["x_ac_h"], "x_ac_v_m": t["x_ac_v"], "l_h_m": lay["l_h"], "l_v_m": lay["l_v"],
                     "a_stab_per_rad_lifting_line": t["a_t"], "a_fin_per_rad": t["a_v"]},
            "fuselage": {"format": "oml.Fuselage stations [x, w, h, zc, n_top, n_bot, top_frac]",
                         "stations": ac.pod.F, "length_m": ac.pod.L, "width_max_m": fp["w_max"],
                         "height_max_m": fp["h_max"], "S_wet_m2": fp["S_wet"], "volume_m3": fp["volume"],
                         "side_area_m2": fp["S_side"], "x_constant_section_end_m": ac.pod.xa,
                         "x_firewall_m": ac.pod.x_fw},
            "propulsion": {"engine": "Limbach L 275 EF", "prop_hub_face_x_m": ac.pod.X_HUB, "thrust_line_z_m": Z_T,
                           "hub_spacer_m": HUB_SPACER, "prop_plane_x_m": ac.pod.X_PROP, "cowl_lip_x_m": ac.pod.X_LIP,
                           "spinner": ac.pod.SPINNER, "propeller": {"model": "Mejzlik 32x18 2B (pusher hand)",
                                                                    "D_m": EC.D_PROP, "mass_kg": M_PROP},
                           "cooling_exit_annulus_m2": pk["cooling_exit_annulus_m2"]},
            "landing_gear": {k: gear[k] for k in ("x_mg", "z_g", "track", "wheelbase", "h_gear", "h_nose_leg",
                                                  "tipback_deg", "turnover_deg", "nose_load_aft_cg", "nose_load_fwd_cg",
                                                  "prop_clear_level", "prop_clear_lof", "prop_clear_design",
                                                  "theta_lof_deg", "theta_td_deg", "theta_design_deg",
                                                  "prop_strike_deg", "bumper_contact_deg", "x_bumper", "z_bumper",
                                                  "turret_clear", "fuselage_bottom_height_at_mg", "active_height_rule")},
            "bays": {"turret_chin_x_m": X_TURRET, "battery_pdu_x_m": X_BATT, "avionics_x_m": X_AVION,
                     "parachute_x_m": list(X_PARACHUTE), "nose_gear_x_m": X_NG,
                     "payload_bay_x_m": list(lay["tank"]["bay"]), "fuel_cells": lay["tank"]["cells"]},
            "overall": {"length_m": L_tot, "span_m": ac.pf["b"], "height_m": t["tip_z_fin"] - gear["z_g"],
                        "ground_z_m": gear["z_g"]},
        },
        "aero": {"drag_items_D_over_q_m2": ac.drag_items, "cd_items": {k: v / ac.S for k, v in ac.drag_items.items()},
                 "cd_rest_non_wing": ac.fit["cd_rest"], "cd0": ac.fit["cd0"], "k": ac.fit["k"], "e": ac.fit["e"],
                 "e_inviscid_lifting_line": aero["e_inv"], "e_nita_scholz": ac.fit["e_nita_scholz"],
                 "e_raymer_straight": ac.fit["e_raymer"], "ld_max": ac.fit["LD_max"], "cl_ld_max": ac.fit["CL_LDmax"],
                 "cl_best_endurance": ac.fit["CL_endurance"], "CL_alpha_per_rad": aero["CL_alpha"],
                 "CL_0_at_pod_level": aero["CL_0"], "cm0_tripped": aero["cm0"], "clmax": ac.clmax,
                 "alpha_stall_deg": aero["alpha_stall_deg"], "stall_onset_eta": aero["stall_eta"],
                 "stall_margins": smg, "Re_root": aero["Re_root"], "Re_tip": aero["Re_tip"],
                 "neuralfoil_min_confidence": aero["min_conf"],
                 "polar_basis": "tripped section polars (x_tr 0.075 c) x 1.15, strip-integrated on the lifting-line cl, "
                                "+ aerolib component build-up, trimmed (tail load + thrust moment), at the 3000 m loiter "
                                "speed; parabolic fit over CL 0.25-1.25",
                 "polar_table": {"CL": ac.cd_table[0], "CD": ac.cd_table[1]},
                 "clean_profile_drag_at_cl_0p9_upside": aero["cdp_clean"](0.9),
                 "tripped_profile_drag_at_cl_0p9": aero["cdp"](0.9) * K_CD_TRIP},
        "mass": {"mtow_kg": m0, "empty_kg": res["empty"], "fuel_kg": fuel, "payload_kg": PAYLOAD,
                 "fixed_equipment_kg": fixed_mass(d), "airframe_kg": res["eb"]["airframe"],
                 "growth_allowance_kg": res["eb"]["growth"], "groups_kg": groups,
                 "wing_breakdown": ac.mass["wing"], "tail_surfaces": ac.mass["tail_surfaces"],
                 "boom_design": {k: v for k, v in ac.mass["boom_design"].items()},
                 "chassis_items_kg": {k: v[0] for k, v in ac.mass["chassis_items"].items()},
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
                      "V_H": tv["V_H"], "V_V": tv["V_V"], "l_h_m": lay["l_h"], "l_v_m": lay["l_v"],
                      "deps_dalpha": lay["deps_da"], "cm_alpha_fuselage_per_rad": lay["cm_alpha_fus"],
                      "kf_fuselage_per_rad": lay["kf"], "a_tail_per_rad": t["a_t"], "eta_tail": ETA_T,
                      "tail_cl_to_trim_clmax_fwd_cg": ac.clmax["tail_cl_to_trim_clmax_fwd_cg"],
                      "cn_beta_fins_per_rad": cnb_fin, "cn_beta_pod_raymer_per_rad": cnb_fus_raymer,
                      "cn_beta_pod_datcom_per_rad": cnb_fus_datcom,
                      "cn_beta_total_per_rad": cnb_fin + min(cnb_fus_raymer, cnb_fus_datcom),
                      "cn_beta_requirement_per_rad": d.cnb_req,
                      "note": "power-off tail efficiency 0.9 (slipstream not credited for stability); booms ignored in "
                              "Cm_alpha/Cn_beta (slender, aft of the CG); pusher propeller normal force (stabilising) "
                              "neglected; fin end-plate factor 1.4 is an estimate"},
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
                               "note": "endurance-chain constraint_diagram (altitude curves corrected from sizinglib's "
                                       "Gagg-Ferrar lapse to sigma^1.23); the design W/S is the stall corner (no flaps)"},
        "structures": struct,
        "checks": pk,
        "production_indicators": pi,
        "sensitivities_endurance_h": sens,
        "electrical": {"load_W": P_ELEC, "generator_W_loiter_3000m": loi3["gen_W"], "generator_W_loiter_sl": loi0["gen_W"],
                       "margin_3000m": loi3["gen_W"] / P_ELEC, "margin_sl": loi0["gen_W"] / P_ELEC},
        "flags": FLAGS,
        "files": ["ucav250/data/concepts/production/calc.py", "ucav250/data/concepts/production/concept.yaml",
                  "ucav250/data/concepts/production/sketch.png", "ucav250/data/concepts/production/constraint.png",
                  "ucav250/data/concepts/production/notes_tr.md"],
    }
    out = py(y)
    out_dir = HERE
    if "--out" in sys.argv:                      # development runs write elsewhere (e.g. a scratch directory)
        out_dir = Path(sys.argv[sys.argv.index("--out") + 1])
        out_dir.mkdir(parents=True, exist_ok=True)
    elif quick:
        import tempfile
        out_dir = Path(tempfile.gettempdir())
    with open(out_dir / "concept.yaml", "w", encoding="utf-8") as f:
        f.write("# YK-250 concept study 'production' (twin boom, H tail, pusher) - generated by calc.py, do not "
                "edit by hand\n")
        yaml.safe_dump(out, f, sort_keys=False, allow_unicode=True, width=120)
    res_for_plot = {"m0": m0, "empty": res["empty"], "fuel": fuel, "endurance_h": res["endurance_h"],
                    "range_km": rng["range_m"] / 1000, "VH": tv["V_H"], "VV": tv["V_V"],
                    "prop_boom_mm": pk["prop_tip_to_boom_radial_m"] * 1000, "longest_piece_m": pi["longest_piece_m"],
                    "moulds": pi["mould_count"]}
    draw_sketch(ac, perf, res_for_plot, out_dir / "sketch.png")
    draw_constraints(cdiag, out_dir / "constraint.png")
    print(f"\nwrote {out_dir}/concept.yaml, sketch.png, constraint.png  ({time.time() - t0:.0f} s)")


if __name__ == "__main__":
    main()
