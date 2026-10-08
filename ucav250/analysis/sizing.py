"""YK-250 HANCER sizing study: reads ``ucav250/spec.yaml``, closes the design point and verifies every derived value.

CLI (run from the repository root)::

    python3 -m ucav250.analysis.sizing             # evaluate the spec design point -> out/sizing.json, out/sizing.md, figures
    python3 -m ucav250.analysis.sizing --check     # + re-close the design from the spec inputs, compare every derived
                                                   #   spec value and evaluate every requirement (exit 1 on a violation)
    python3 -m ucav250.analysis.sizing --design    # design closure from the spec design rules -> out/sizing_design.yaml
    python3 -m ucav250.analysis.sizing --update-spec   # design closure + evaluation -> rewrite every derived value
                                                       #   and reference copy in spec.yaml (then run --check)
    python3 -m ucav250.analysis.sizing --trades    # propeller, gear, turret, span/wing loading, tail arm -> out/sizing_trades.json
    python3 -m ucav250.analysis.sizing --render    # Workbench renders from the spec geometry -> docs/fig (needs bpy)

Configuration (user decision after the v2 design-direction study, ucav250/data/concepts_v2): "D - HANCER": a chined,
faceted lifting body (diamond-like sections) whose chines run into curved leading-edge root extensions (LERX) that
blend into a high-aspect-ratio NLF wing; twin outward-canted fins with rudders; all-moving horizontal tails
(stabilators); an OPEN pusher propeller guarded by the canted fins and a ventral fin with a tail-bumper skid;
retractable tricycle gear; an EO/IR turret lowered from a belly bay for the mission and retracted for take-off and
landing. Civil EO/IR surveillance and research platform only (see spec meta.scope_tr).

Method. The reviewed conventional 'endurance' concept study (ucav250/data/concepts/endurance/calc.py) is the
yardstick: its propulsion model (Mejzlik table with J-similarity, L 275 EF WOT torque x sigma^1.23, part-load BSFC at
the actual power incl. the generator load), its tripped-polar sizing rule (NeuralFoil, x_tr 0.075 c, cd x 1.15,
cl_max x 0.94 x 0.95), its mission (sizinglib segments), mass rules and performance equations are reproduced here so
HANCER and the conventional concepts are judged on one basis (``tests/test_ucav250_sizing.py`` checks the propulsion
reproduction). New for HANCER:

* geometry generators: body control lines -> dense oml.Fuselage stations, LERX Bezier + reference trapezoid -> wing
  sections, tail panels from planform parameters (closure inputs live in the spec: fuselage.lines, wing.planform,
  tail.surfaces.*.params);
* aerodynamics: lifting line on the actual wing incl. the LERX (span loading, CL0, critical-section CLmax outboard of
  the strake, strip profile drag), vortex lattice for lift slope, aerodynamic centre, Trefftz efficiency and the
  configuration neutral point; drag build-up on OML-mesh wetted areas of the exposed parts;
* stability: Multhopp strip method for the lifting body with the VLM upwash, canted-fin projections, NP = forward of
  the classic and the VLM value; Cn_beta with the more destabilising of the Raymer/DATCOM body terms;
* closure rules: fixed MTOM (fuel = MTOM - empty - payload), wing station for the minimum static margin, wing area
  for the stall-speed target, stabilator size from the forward-CG trim tail-lift limit, fin size from the Cn_beta
  target, belly payload bay centred on the empty-mass CG, main gear behind it (tip-back, nose-wheel load, turnover
  track), gear height from the propeller-clearance and bumper rules, retraction kinematics checked on the OML;
* EO/IR turret bay (E180 growth envelope, flush retracted ball, field of regard by ray casting), packaging zones and
  fuel cells checked against the OML, retractable-gear and turret-mechanism masses.

Frame (ARCHITECTURE.md): X aft from the nose tip, Y starboard, Z up; Z = 0 on the centre-body chine plane. SI units,
angles in degrees in the spec and the reports.
"""
from __future__ import annotations

import argparse
import copy
import functools
import json
import math
import sys
import time
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np
import yaml
from scipy.interpolate import RegularGridInterpolator

from ..core import spec as SPEC
from ..core.geom import Mesh
from ..design import oml
from . import aero as AE
from . import aerolib as AL
from . import sizinglib as SZ
from . import stablib as SL
from . import structlib as ST

ROOT = SPEC.ROOT
REPO = ROOT.parent
RESEARCH = ROOT / "data" / "research"
OUT_DIR = SPEC.OUT_DIR
FIG_DIR = ROOT / "docs" / "fig"
G = AL.G0
RHO0 = 1.225


# =====================================================================================================================
# 0. research files (large tables that the spec references by key) and small helpers
# =====================================================================================================================
@functools.lru_cache(maxsize=None)
def research(name: str) -> dict:
    loader = getattr(yaml, "CSafeLoader", yaml.SafeLoader)
    with open(RESEARCH / name, encoding="utf-8") as f:
        return yaml.load(f, Loader=loader)


def ref_get(ref: str):
    """``"aero.yaml#propeller_tables_mejzlik.0161"`` -> value from a research file."""
    fname, path = ref.split("#", 1)
    cur = research(fname)
    for k in path.split("."):
        cur = cur[k]
    return cur


def py(x, sig: int = 6):
    """numpy -> plain python, floats rounded to ``sig`` significant digits (JSON/YAML output)."""
    if isinstance(x, dict):
        return {str(k): py(v, sig) for k, v in x.items()}
    if isinstance(x, (list, tuple)):
        return [py(v, sig) for v in x]
    if isinstance(x, np.ndarray):
        return [py(v, sig) for v in x.tolist()]
    if isinstance(x, (np.integer,)):
        return int(x)
    if isinstance(x, (np.bool_,)):
        return bool(x)
    if isinstance(x, (float, np.floating)):
        x = float(x)
        if not math.isfinite(x):
            return None
        return 0.0 if x == 0.0 else float(f"{x:.{sig}g}")
    return x


def q_s(V: float, h: float) -> float:
    """Dynamic pressure (Pa) at TAS V and altitude h (ISA)."""
    return 0.5 * AL.isa(h)["rho"] * V * V


def golden(f, a: float, b: float, n: int = 30) -> float:
    """Golden-section minimiser (same routine as the endurance study)."""
    gr = (math.sqrt(5) - 1) / 2
    c, d = b - gr * (b - a), a + gr * (b - a)
    fc, fd = f(c), f(d)
    for _ in range(n):
        if fc < fd:
            b, d, fd = d, c, fc
            c = b - gr * (b - a)
            fc = f(c)
        else:
            a, c, fc = c, d, fd
            d = a + gr * (b - a)
            fd = f(d)
    return 0.5 * (a + b)


def herm(t, y0, y1, m0, m1):
    """Cubic Hermite on t in [0, 1] (values y0, y1; slopes m0, m1 per unit t)."""
    t = np.clip(t, 0.0, 1.0)
    return ((2 * t ** 3 - 3 * t ** 2 + 1) * y0 + (t ** 3 - 2 * t ** 2 + t) * m0 + (-2 * t ** 3 + 3 * t ** 2) * y1 +
            (t ** 3 - t ** 2) * m1)


def sstep(t):
    """C2 smootherstep 0 -> 1."""
    t = np.clip(t, 0.0, 1.0)
    return t * t * t * (t * (6 * t - 15) + 10)


# =====================================================================================================================
# 1. propulsion (reproduces endurance/calc.py section 2 with the spec as input)
# =====================================================================================================================
class Engine:
    """Limbach L 275 EF: datasheet WOT torque x sigma^lapse, part-load BSFC (fraction of max power), generator load."""

    def __init__(self, eng: dict, elec_load_W: float, eta_gen: float):
        wc = eng["wot_curve"]
        self.rpm = np.asarray(wc["rpm"], float)
        self.tq = np.asarray(wc["torque_N_m"], float)
        self.P_max = float(eng["power_max_W"])
        self.P_mcp = float(eng["power_max_continuous_W"])
        self.lapse_exp = float(eng["altitude_power_lapse_exponent"])
        self.bsfc_pts = np.asarray(eng["bsfc_curve"]["points"], float)
        self.n_max = float(eng["rpm_max_operating"])
        self.n_cut = float(eng["rpm_hard_cut"])
        self.p_gen_shaft = elec_load_W / eta_gen
        self.bsfc_scale = 1.0                                   # sensitivity runs only

    def lapse(self, sigma: float) -> float:
        return sigma ** self.lapse_exp

    def torque_wot(self, n, sigma: float):
        n = np.asarray(n, float)
        q = np.interp(n, self.rpm, self.tq) * self.lapse(sigma)
        return np.where(n > self.n_cut, 0.0, q)

    def bsfc(self, p_total: float) -> float:
        """g/kWh at total shaft power (propeller + generator); linear, extrapolated below the lowest point."""
        f = p_total / self.P_max
        P = self.bsfc_pts
        if f < P[0, 0]:
            b = P[0, 1] + (P[1, 1] - P[0, 1]) / (P[1, 0] - P[0, 0]) * (f - P[0, 0])
        else:
            b = float(np.interp(f, P[:, 0], P[:, 1]))
        return self.bsfc_scale * float(b)


class Prop:
    """Mejzlik table (manufacturer) with J-similarity outside the tabulated rpm range and density scaling; WOT thrust
    x k_wot (Falcon/Mejzlik ratio), part-throttle thrust x k_inst (pusher installation). Endurance study class."""
    NGRID = np.linspace(1200.0, 8000.0, 273)

    def __init__(self, rows: dict, D: float, k_wot: float, k_inst: float, eng: Engine):
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
        self.k_wot = k_wot
        self.k_inst = k_inst
        self.eng = eng

    def sl(self, V: float, n):
        n = np.atleast_1d(np.asarray(n, float))
        nr = np.clip(n, self.N[0], self.N[-1])
        s = n / nr
        Vr = V / s
        pts = np.column_stack([np.clip(Vr, 0.0, 60.0), nr])
        return self.fT(pts) * s * s, self.fQ(pts) * s * s

    def cruise(self, T_req: float, V: float, h: float) -> dict:
        atm = AL.isa(h)
        sig = atm["sigma"]
        T, Q = self.sl(V, self.NGRID)
        T = np.maximum.accumulate(sig * T * self.k_inst)
        if T_req > T[-1]:
            return {"ok": False}
        n = float(np.interp(T_req, T, self.NGRID))
        q = sig * float(np.interp(n, self.NGRID, Q))
        P = q * 2 * math.pi * n / 60
        tip = math.hypot(math.pi * self.D * n / 60, V) / atm["a"]
        return {"ok": True, "rpm": n, "P_shaft": P, "eta": T_req * V / P if P > 0 else 0.0, "tip_mach": tip}

    def wot(self, V: float, h: float, p_gen: float | None = None) -> dict:
        atm = AL.isa(h)
        sig = atm["sigma"]
        p_gen = self.eng.p_gen_shaft if p_gen is None else p_gen
        n = self.NGRID
        T, Q = self.sl(V, n)
        omega = 2 * math.pi * n / 60
        diff = self.eng.torque_wot(n, sig) - p_gen / omega - sig * Q
        m = n <= self.eng.n_max
        d = diff[m]
        if d[-1] > 0:
            nn = self.eng.n_max
        else:
            i = int(np.argmax(d <= 0))
            nn = float(np.interp(0.0, [-d[i - 1], -d[i]], [n[i - 1], n[i]])) if i > 0 else float(n[0])
        Tn, Qn = self.sl(V, nn)
        P = sig * float(Qn[0]) * 2 * math.pi * nn / 60
        Tt = sig * float(Tn[0]) * self.k_wot
        tip = math.hypot(math.pi * self.D * nn / 60, V) / atm["a"]
        return {"rpm": nn, "T": Tt, "P_shaft": P, "eta": Tt * V / P if P > 0 and V > 0 else 0.0, "tip_mach": tip}


# =====================================================================================================================
# 2. tripped section polars (endurance/calc.py section 3; disk cache next to aero.py's polar cache)
# =====================================================================================================================
XTR_TRIP = 0.075


@functools.lru_cache(maxsize=None)
def _raw_tripped(airfoil: str, Re: float, ts: float) -> dict:
    tag = "" if abs(ts - 1.0) < 1e-9 else f"_t{ts:.4f}"
    path = AE.POLAR_DIR / f"{airfoil.lower()}{tag}_Re{int(round(Re)):d}_trip{XTR_TRIP:g}.json"
    if path.exists():
        d = json.loads(path.read_text())
        return {k: d[k] for k in ("alpha", "cl", "cd", "cm", "confidence")}
    import neuralfoil as nf
    P = AE.section_coords(airfoil, ts)
    a = np.array(AE.ALPHAS)
    r = nf.get_aero_from_coordinates(P, alpha=a, Re=float(Re), n_crit=9.0, xtr_upper=XTR_TRIP, xtr_lower=XTR_TRIP,
                                     model_size="xlarge")
    out = {"alpha": list(AE.ALPHAS), "cl": [float(v) for v in r["CL"]], "cd": [float(v) for v in r["CD"]],
           "cm": [float(v) for v in r["CM"]], "confidence": [float(v) for v in r["analysis_confidence"]]}
    AE.POLAR_DIR.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps({"airfoil": airfoil, "thickness_scale": ts, "Re": Re, "xtr": XTR_TRIP, **out,
                                "method": f"NeuralFoil xlarge, transition forced at x/c {XTR_TRIP} both surfaces"}))
    return out


def _ts_key(ts: float) -> float:
    return round(float(ts), 4)


@functools.lru_cache(maxsize=None)
def tripped_chars(airfoil: str, Re: float, ts: float = 1.0) -> dict:
    """aero.characteristics() equivalent for the tripped case (log-Re interpolation on aero.RE_GRID)."""
    grid = AE.RE_GRID
    Re = float(np.clip(Re, grid[0], grid[-1]))
    j = min(max(int(np.searchsorted(grid, Re)), 1), len(grid) - 1)
    r0, r1 = grid[j - 1], grid[j]
    t = (math.log(Re) - math.log(r0)) / (math.log(r1) - math.log(r0))
    c0 = AE.polar_characteristics(_raw_tripped(airfoil, r0, _ts_key(ts)))
    c1 = AE.polar_characteristics(_raw_tripped(airfoil, r1, _ts_key(ts)))
    out = {k: (1 - t) * c0[k] + t * c1[k] for k in c0 if k != "cd_table"}
    out["_tables"] = (c0["cd_table"], c1["cd_table"], t)
    return out


def section_chars(airfoil: str, Re: float, ts: float, tripped: bool) -> dict:
    if tripped:
        return tripped_chars(airfoil, float(Re), _ts_key(ts))
    return AE.characteristics(airfoil, float(Re), 9.0, _ts_key(ts))


# =====================================================================================================================
# 3. geometry generators (design closure) and the OML model built from the spec
# =====================================================================================================================
def body_lines(x, L: dict, x_c0: float | None = None):
    """Designer's control lines of the chined lifting body at stations ``x``: plan-view chine half width ``a``, dorsal
    ridge ``zt``, keel ``zb``, chine height ``zc`` and the facet exponents (n_top, n_bot). Every line is a C1/C2 blend
    of a few control values (Hermite ogives, smootherstep transitions), so the dense oml.Fuselage stations reproduce
    smooth lines with crisp chines (n close to 1 = flat facets meeting at a sharp chine and a dorsal/ventral ridge).
    ``x_c0``: start of the chine rise toward the thrust line (build_geometry holds the chine on the wing plane until
    behind the wing-root trailing edge, so the LERX/glove upper surface meets the chine along the whole root chord);
    default ``x_hub - l_chine_rise``."""
    x = np.asarray(x, float)
    xh = L["x_hub"]
    xe, xend = xh - L["l_slab_aft"], xh + L["l_cowl_tail"]
    tb = math.tan(math.radians(L["beta_end_deg"])) * (xend - xe)
    te = (x - xe) / (xend - xe)
    # plan view: convex ogive -> constant centre body -> waist toward the engine slab -> cowl closure
    a = np.where(x <= L["l_nose"], herm(x / L["l_nose"], 0.0, L["a_max"], L["s_side"] * L["l_nose"], 0.0), L["a_max"])
    xw = L["x_waist"]
    a = np.where(x > xw, L["a_max"] + (L["a_eng"] - L["a_max"]) * sstep((x - xw) / (xe - L["l_waist_end"] - xw)), a)
    a = np.where(x > xe, herm(te, L["a_eng"], L["r_lip"], 0.0, -tb), a)
    # chine height: drooped tip -> datum (wing plane) -> rises to the thrust line at the engine
    zc = np.where(x <= L["l_chine"], herm(x / L["l_chine"], L["z_tip"], 0.0, L["s_chine"] * L["l_chine"], 0.0), 0.0)
    x_c0 = xh - L["l_chine_rise"] if x_c0 is None else x_c0
    zc = np.where(x > x_c0, L["z_t"] * sstep((x - x_c0) / (xe - 0.10 - x_c0)), zc)
    # dorsal ridge: nose ogive -> dorsal line -> engine hump -> cowl closure
    zt = np.where(x <= L["l_top"], herm(x / L["l_top"], L["z_tip"], L["zt_c"], L["s_top"] * L["l_top"], 0.0), L["zt_c"])
    x_h0, x_h1 = xh - L["l_hump"], xh - L["l_hump_end"]
    zt_e = L["z_t"] + L["hump"]
    zt = np.where(x > x_h0, L["zt_c"] + (zt_e - L["zt_c"]) * sstep((x - x_h0) / (x_h1 - x_h0)), zt)
    zt = np.where(x > xe, herm(te, zt_e, L["z_t"] + L["r_lip"], 0.0, -L["k_close"] * tb), zt)
    # keel: chin ogive -> flat belly -> upsweep to the engine-bay keel -> closure
    zb = np.where(x <= L["l_bot"], herm(x / L["l_bot"], L["z_tip"], L["zb_c"], -L["s_bot"] * L["l_bot"], 0.0),
                  L["zb_c"])
    x_u0 = xh - L["l_upsweep"]
    zb_e = L["z_t"] - L["keel_e"]
    zb = np.where(x > x_u0, L["zb_c"] + (zb_e - L["zb_c"]) * sstep((x - x_u0) / (xe - 0.10 - x_u0)), zb)
    zb = np.where(x > xe, herm(te, zb_e, L["z_t"] - L["r_lip"], 0.0, L["k_close"] * tb), zb)
    # facets: crisp (n ~ 1) on the forebody and centre body, fuller around the engine, round at the cowl lip
    nt = np.full_like(x, L["n_top"])
    nb = np.full_like(x, L["n_bot"])
    # fuller belly (payload bay, main-gear wells) between x_belly0 and x_belly1, smooth ramps of length l_belly_ramp
    up = sstep((x - L["x_belly0"]) / L["l_belly_ramp"]) * (1 - sstep((x - L["x_belly1"]) / L["l_belly_ramp"]))
    nb = nb + (L["n_belly"] - nb) * up
    tc = sstep((x - (xh - L["l_cowl_blend"])) / L["l_cowl_blend_len"])
    nt = nt + (L["n_cowl_top"] - nt) * tc
    nb = nb + (L["n_cowl_bot"] - nb) * tc
    tl = sstep((x - (xe - 0.05)) / (xend - (xe - 0.05)))
    nt = nt + (2.0 - nt) * tl
    nb = nb + (2.0 - nb) * tl
    return a, zt, zb, zc, nt, nb


def body_stations(L: dict, x_c0: float | None = None) -> np.ndarray:
    """Dense oml.Fuselage stations [x, w, h, zc, n_top, n_bot, top_frac]: 12.5 mm at the tip, 50 mm along the body,
    25 mm through the chine rise and the cowl closure; first station is the pointed tip (pole)."""
    xh = L["x_hub"]
    xend = xh + L["l_cowl_tail"]
    x_d = min(xh - 0.45, (x_c0 if x_c0 is not None else xh - L["l_chine_rise"]) - 0.05)
    x_d = math.floor(x_d / 0.025 + 1e-9) * 0.025          # on the 25 mm grid: the station x do not move with x_c0
    xs = np.unique(np.round(np.r_[0.0, 0.0125, 0.025, 0.05, np.arange(0.10, x_d, 0.05),
                                  np.arange(x_d, xend, 0.025), xend], 6))
    a, zt, zb, zc, nt, nb = body_lines(xs, L, x_c0)
    h = np.maximum(zt - zb, 0.0)
    tf = np.clip((zt - zc) / np.maximum(h, 1e-9), 0.06, 0.94)
    st = np.column_stack([xs, 2 * a, h, zc, nt, nb, tf])
    st[0, 1:3] = 0.0
    st[0, 6] = 0.5
    return np.round(st, 5)


def trapezoid(P: dict) -> dict:
    """Reference trapezoid continued to the centre line (ARCHITECTURE.md: wing.area)."""
    b2 = P["span"] / 2
    cr = 2 * P["area"] / (P["span"] * (1 + P["taper"]))
    ct = P["taper"] * cr
    tl = math.tan(math.radians(P["sweep_c4_deg"]))
    c = lambda y: cr + (ct - cr) * np.asarray(y, float) / b2          # noqa: E731
    xle = lambda y: P["x_c4_root"] + np.asarray(y, float) * tl - 0.25 * c(y)   # noqa: E731
    mac = 2 / 3 * cr * (1 + P["taper"] + P["taper"] ** 2) / (1 + P["taper"])
    y_mac = P["span"] / 6 * (1 + 2 * P["taper"]) / (1 + P["taper"])
    return {"b2": b2, "cr": cr, "ct": ct, "c": c, "xle": xle, "xte": lambda y: xle(y) + c(y), "mac": mac,
            "y_mac": y_mac, "x_le_mac": float(xle(y_mac)), "AR": P["span"] ** 2 / P["area"]}


def _bezier(P0, P1, P2, P3, n=400):
    t = np.linspace(0, 1, n)[:, None]
    return (1 - t) ** 3 * P0 + 3 * (1 - t) ** 2 * t * P1 + 3 * (1 - t) * t ** 2 * P2 + t ** 3 * P3


@functools.lru_cache(maxsize=None)
def airfoil_thickness(name: str) -> tuple:
    """(x/c, t/c) thickness distribution of a section at thickness_scale 1 (oml resampling, closed trailing edge)."""
    x, yu, yl = oml.resampled(name, 201, 0.0, 1.0)
    return np.asarray(x), np.asarray(yu - yl)


def section_depth_at(name: str, ts: float, chord: float, xc: float) -> float:
    """Absolute OML depth (m) of a section at chord fraction ``xc`` (0 outside the chord)."""
    if not 0.0 <= xc <= 1.0:
        return 0.0
    x, t = airfoil_thickness(name)
    return float(np.interp(xc, x, t)) * ts * chord


def spar_depth_targets(P: dict) -> dict:
    """Spar-depth targets of the glove (body side to the outer-panel joint): main spar line >= ``spar_depth_main_frac`` x
    the junction thickness t_j (largest section thickness of the outer panel at the joint); rear spar line >=
    ``spar_depth_rear_frac`` x the depth of the outer-panel joint section itself along the rear spar line (the rear
    spar runs from the joint into the body without stepping down)."""
    T = trapezoid(P)
    c_j = float(T["c"](P["y_junction"]))
    t_j = 0.1595 * c_j                                       # NLF(1)-0416 max t/c (oml.max_thickness)
    d_rear_joint = section_depth_at("nlf416", 1.0, c_j, float(P["rear_spar_frac"]))
    d_main_joint = section_depth_at("nlf416", 1.0, c_j, float(P["main_spar_frac"]))
    return {"t_j": t_j, "c_j": c_j, "main_joint_depth": d_main_joint, "rear_joint_depth": d_rear_joint,
            "main_required": float(P.get("spar_depth_main_frac", 0.0)) * t_j,
            "rear_required": float(P.get("spar_depth_rear_frac", 0.0)) * d_rear_joint,
            "rear_depth_ratio_outer": d_rear_joint / t_j}


def wing_sections(P: dict, fus: oml.Fuselage) -> tuple[list, dict]:
    """LiftingSurface sections (starboard) from the planform parameters: LERX/glove from the chine apex to the
    junction with the outer-panel leading edge (cubic Bezier tangent to the chine at the apex and to the outer leading
    edge at the junction), straight trailing edge of the reference trapezoid, outer panel 16 % -> ``ts_tip`` x 16 %
    with linear washout and dihedral from the junction.

    LERX/glove sections (NACA 0012 near the apex, NLF(1)-0416 toward the junction):
    * twist (incidence about the section leading edge) ramps from ``lerx_twist_root_frac`` x incidence at the apex to
      the full incidence at ``lerx_twist_ramp_u`` (span fraction of the LERX), so the long, far-forward root sections
      lie on the chine plane and the glove trailing edge meets the chine instead of dropping below it;
    * thickness: the larger of (a) the blend rule (absolute thickness from ``glove_t0`` x junction thickness at the apex
      to the junction value) and (b) the spar-depth rule (spar_depth_targets): OML depth at the main spar line >=
      ``spar_depth_main_frac`` x junction thickness and at the rear spar line >= ``spar_depth_rear_frac`` x the joint
      section's own rear-spar depth, so the spar box runs from the outer-panel joint into the body without a step
      (capped at ``glove_ts_max``)."""
    T = trapezoid(P)
    xa = P["x_apex"]
    a0 = float(fus.section(np.array([xa]))[0][0]) / 2
    a1 = float(fus.section(np.array([xa + 1e-3]))[0][0]) / 2
    s_ch = (a1 - a0) / 1e-3
    yj = P["y_junction"]
    p0 = np.array([xa, a0])
    p3 = np.array([float(T["xle"](yj)), yj])
    d0 = np.array([1.0, s_ch]) / math.hypot(1.0, s_ch)
    d1 = np.array([float(T["xle"](yj + 1e-3) - T["xle"](yj)), 1e-3])
    d1 /= np.linalg.norm(d1)
    Lc = float(np.linalg.norm(p3 - p0))
    curve = _bezier(p0, p0 + P["k_apex"] * Lc * d0, p3 - P["k_junction"] * Lc * d1, p3)
    if np.any(np.diff(curve[:, 1]) <= 0):
        raise ValueError("LERX leading edge must be monotonic in span (adjust k_apex/k_junction)")
    t_af = 0.1595                                            # NLF(1)-0416 max t/c (oml.max_thickness)
    sdt = spar_depth_targets(P)
    t_j = sdt["t_j"]
    d_main, d_rear = sdt["main_required"], sdt["rear_required"]
    ts_max = float(P.get("glove_ts_max", 1.0))
    f_tw0, u_tw = float(P.get("lerx_twist_root_frac", 1.0)), float(P.get("lerx_twist_ramp_u", 1e-6))
    secs = []
    u_l = np.linspace(0.0, 1.0, int(P["n_lerx"]) + 1)[:-1]
    ys_l = a0 + (yj - a0) * (1 - np.cos(0.5 * math.pi * u_l))    # denser near the apex
    for y in ys_l:
        xl = float(np.interp(y, curve[:, 1], curve[:, 0]))
        c = float(T["xte"](y)) - xl
        u = (y - a0) / (yj - a0)
        t_abs = t_j * (P["glove_t0"] + (1 - P["glove_t0"]) * u)
        sym = u < P["lerx_symmetric_until"]
        af_name = "n0012" if sym else "nlf416"
        t_ref = 0.12 if sym else t_af
        ts = t_abs / (t_ref * c)
        xs_m = (float(T["xle"](y)) + P["main_spar_frac"] * float(T["c"](y)) - xl) / c
        xs_r = (float(T["xle"](y)) + P["rear_spar_frac"] * float(T["c"](y)) - xl) / c
        for d_req, xs in ((d_main, xs_m), (d_rear, xs_r)):
            dep = section_depth_at(af_name, 1.0, c, xs)
            if d_req > 0 and dep > 0:
                ts = max(ts, d_req / dep)
        ts = float(np.clip(ts, P["glove_ts_min"], ts_max))
        tw = P["incidence_deg"] * (f_tw0 + (1.0 - f_tw0) * float(sstep(u / u_tw)))
        secs.append({"y": round(float(y), 5), "x_le": round(xl, 5), "z_le": round(P["z_root"], 5),
                     "chord": round(c, 5), "twist_deg": round(tw, 4),
                     "airfoil": af_name, "thickness_scale": round(ts, 4)})
    b2 = T["b2"]
    n_o = int(P["n_outer"])
    ys_o = np.r_[yj + (b2 - P["tip_round"] - yj) * np.linspace(0, 1, n_o - 1), b2]
    for y in ys_o:
        f = (y - yj) / (b2 - yj)
        secs.append({"y": round(float(y), 5), "x_le": round(float(T["xle"](y)), 5),
                     "z_le": round(P["z_root"] + (y - yj) * math.tan(math.radians(P["dihedral_deg"])), 5),
                     "chord": round(float(T["c"](y)), 5),
                     "twist_deg": round(P["incidence_deg"] - P["washout_deg"] * f, 4), "airfoil": "nlf416",
                     "thickness_scale": round(1.0 + (P["ts_tip"] - 1.0) * f, 4)})
    ys_all = np.array([s_["y"] for s_ in secs])
    xl_all = np.array([s_["x_le"] for s_ in secs])
    sweep = np.degrees(np.arctan(np.gradient(xl_all, ys_all)))
    y_strake = float(ys_all[np.argmax(sweep < P["strake_sweep_deg"])])
    info = {"trapezoid": {k: v for k, v in T.items() if not callable(v)}, "apex": [xa, a0], "junction": p3.tolist(),
            "le_sweep_deg": sweep.round(2).tolist(), "y_strake_end": y_strake,
            "lerx_le_curve": curve[::20].tolist(), "chine_slope_at_apex": s_ch}
    return secs, info


def surface_from_params(kind: str, P: dict) -> list:
    """Tail surface sections (starboard / centre line) from planform parameters.
    stabilator: root at the body side, flat (dihedral), swept LE; fin: canted outward ``cant_deg`` from vertical;
    ventral: on the centre line pointing down (span_dir -Z)."""
    n = int(P.get("n_sections", 3))
    s = np.linspace(0.0, 1.0, n)
    sw = math.tan(math.radians(P["sweep_le_deg"]))
    out = []
    for f in s:
        c = P["root_chord"] + (P["tip_chord"] - P["root_chord"]) * f
        if kind == "stabilator":
            d = math.radians(P.get("dihedral_deg", 0.0))
            sec = {"y": P["y_root"] + f * P["span"] * math.cos(d), "x_le": P["x_le_root"] + f * P["span"] * sw,
                   "z_le": P["z_root"] + f * P["span"] * math.sin(d)}
        elif kind == "fin":
            g = math.radians(P["cant_deg"])
            sec = {"y": P["y_root"] + f * P["span"] * math.sin(g), "x_le": P["x_le_root"] + f * P["span"] * sw,
                   "z_le": P["z_root"] + f * P["span"] * math.cos(g)}
        elif kind == "ventral":
            sec = {"y": 0.0, "x_le": P["x_le_root"] + f * P["span"] * sw, "z_le": P["z_root"] - f * P["span"],
                   "span_dir": [0.0, 0.0, -1.0]}
        else:
            raise ValueError(kind)
        sec.update({"chord": c, "twist_deg": 0.0, "airfoil": P["airfoil"]})
        out.append({k: (round(float(v), 5) if isinstance(v, (float, np.floating)) else v) for k, v in sec.items()})
    return out


def panel_planform(sections: list) -> dict:
    """Area, span (along the section line), MAC, MAC LE x and the MAC point of one panel (works for canted panels)."""
    P = np.array([[s["y"], s["z_le"]] for s in sections], float)
    eta = np.r_[0.0, np.cumsum(np.linalg.norm(np.diff(P, axis=0), axis=1))]
    secs = [dict(s, y=float(e)) for s, e in zip(sections, eta)]
    m = oml.mean_aerodynamic_chord(secs)
    frac = m["y_mac"] / max(eta[-1], 1e-9)
    y = sections[0]["y"] + frac * (sections[-1]["y"] - sections[0]["y"])
    z = sections[0]["z_le"] + frac * (sections[-1]["z_le"] - sections[0]["z_le"])
    return {"area": m["half_area"], "span": float(eta[-1]), "mac": m["mac"], "x_le_mac": m["x_le_mac"],
            "y_mac": y, "z_mac": z, "x_ac": m["x_le_mac"] + 0.25 * m["mac"],
            "aspect_ratio": float(eta[-1]) ** 2 / m["half_area"]}


class Airframe:
    """OML objects built from the spec (fuselage stations, wing and tail sections) + geometric queries."""

    def __init__(self, S: dict):
        self.S = S
        self.fus = oml.Fuselage(np.asarray(S["fuselage"]["stations"], float))
        self.wing = oml.LiftingSurface(S["wing"]["sections"], n_chord=80, name="wing")
        self.tail = {k: oml.LiftingSurface(v["sections"], n_chord=60, name=k)
                     for k, v in S["tail"]["surfaces"].items()}
        self.tail_mirror = {k: bool(v.get("mirror", True)) for k, v in S["tail"]["surfaces"].items()}
        self.L = self.fus.x1

    # ---------------------------------------------------------------- body sections
    def sec(self, x):
        xx = np.atleast_1d(np.asarray(x, float))
        w, h, zc, nt, nb = self.fus.section(xx)
        tf = self.fus.top_frac(xx)
        return w / 2, tf * h, (1 - tf) * h, zc, nt, nb

    def z_top(self, x, y=0.0):
        return self._z_surface(x, y, True)

    def z_bot(self, x, y=0.0):
        return self._z_surface(x, y, False)

    def _z_surface(self, x, y, upper):
        a, bt, bb, zc, nt, nb = self.sec(x)
        r = np.clip(np.abs(np.asarray(y, float)) / np.maximum(a, 1e-9), 0.0, 1.0)
        n = nt if upper else nb
        hh = bt if upper else bb
        dz = hh * np.clip(1.0 - r ** n, 0.0, 1.0) ** (1.0 / n)
        z = zc + dz if upper else zc - dz
        return z if np.ndim(x) or np.ndim(y) else float(np.asarray(z).ravel()[0])

    def half_width(self, x, z) -> np.ndarray:
        """Body half width at station(s) x and height z (0 where z is outside the section)."""
        a, bt, bb, zc, nt, nb = self.sec(x)
        z = np.broadcast_to(np.asarray(z, float), np.shape(a))
        up = z >= zc
        hh = np.where(up, bt, bb)
        n = np.where(up, nt, nb)
        r = np.clip(np.abs(z - zc) / np.maximum(hh, 1e-9), 0.0, 1.0)
        return a * np.clip(1.0 - r ** n, 0.0, 1.0) ** (1.0 / n)

    def exposed_panel(self, sections: list, n_span: int = 60, n_chord: int = 80) -> dict:
        """Exposed (outside-the-body) part of a canted/vertical panel given by sections (chord plane only):
        area, mean aerodynamic chord, x of the exposed quarter-chord centroid (AC), y/z of the exposed area centroid,
        exposed span along the section line. Used for fins whose root is buried in the body."""
        P = np.array([[s["x_le"], s["y"], s["z_le"], s["chord"]] for s in sections], float)
        seg = np.r_[0.0, np.cumsum(np.linalg.norm(np.diff(P[:, 1:3], axis=0), axis=1))]
        s_ = np.linspace(0.0, seg[-1], n_span + 1)
        sm = 0.5 * (s_[1:] + s_[:-1])
        ds = np.diff(s_)
        xle = np.interp(sm, seg, P[:, 0])
        y = np.interp(sm, seg, P[:, 1])
        z = np.interp(sm, seg, P[:, 2])
        c = np.interp(sm, seg, P[:, 3])
        f = np.linspace(0.0, 1.0, n_chord)
        X = xle[:, None] + f[None, :] * c[:, None]
        Y = np.broadcast_to(y[:, None], X.shape)
        Z = np.broadcast_to(z[:, None], X.shape)
        ins = self.inside(np.column_stack([X.ravel(), Y.ravel(), Z.ravel()])).reshape(X.shape)
        ce = (~ins).mean(axis=1) * c                          # exposed chord per strip
        A = float(np.sum(ce * ds))
        if A <= 0:
            return {"area": 0.0, "mac": 0.0, "x_ac": float(xle.mean()), "y": float(y.mean()), "z": float(z.mean()),
                    "span": 0.0}
        x_le_exp = np.array([X[i][~ins[i]].min() if np.any(~ins[i]) else xle[i] for i in range(len(sm))])
        mac = float(np.sum(ce ** 2 * ds) / A)
        x_ac = float(np.sum(ce * ds * (x_le_exp + 0.25 * ce)) / A)
        return {"area": A, "mac": mac, "x_ac": x_ac, "y": float(np.sum(ce * ds * y) / A),
                "z": float(np.sum(ce * ds * z) / A), "span": float(np.sum(ds[ce > 1e-6]))}

    def inside(self, P, margin: float = 0.0) -> np.ndarray:
        """Points (n, 3) strictly inside the body OML (``margin`` m inside the skin, approximately)."""
        P = np.atleast_2d(np.asarray(P, float))
        x, y, z = P[:, 0], P[:, 1], P[:, 2]
        ok = (x > self.fus.x0) & (x < self.fus.x1)
        xc = np.clip(x, self.fus.x0, self.fus.x1)
        a, bt, bb, zc, nt, nb = self.sec(xc)
        up = z >= zc
        hh = np.where(up, bt, bb) - margin
        n = np.where(up, nt, nb)
        aa = a - margin
        with np.errstate(divide="ignore", invalid="ignore"):
            val = (np.abs(y) / aa) ** n + (np.abs(z - zc) / hh) ** n
        return ok & (aa > 0) & (hh > 0) & (val < 1.0)

    def area(self, x, inset: float = 0.0) -> float:
        """Internal cross-section area at x (superellipse halves, inset by ``inset``)."""
        a, bt, bb, zc, nt, nb = (float(v[0]) for v in self.sec(x))

        def half(aa, bb_, n):
            if aa <= 0 or bb_ <= 0:
                return 0.0
            return 2 * aa * bb_ * math.gamma(1 + 1 / n) ** 2 / math.gamma(1 + 2 / n)
        return half(a - inset, bt - inset, nt) + half(a - inset, bb - inset, nb)

    # ---------------------------------------------------------------- meshes and wetted areas
    @functools.cached_property
    def body_mesh(self) -> Mesh:
        return self.fus.mesh(220, 144)

    @functools.cached_property
    def wing_mesh(self) -> Mesh:
        return self.wing.mesh(refine=2)

    @functools.cached_property
    def tail_meshes(self) -> dict:
        return {k: s.mesh(refine=2) for k, s in self.tail.items()}

    def exposed_area(self, mesh: Mesh) -> float:
        """Area of the triangles whose centroid lies outside the body (exposed wetted area of a lifting surface)."""
        tri = mesh.V[mesh.F]
        c = tri.mean(axis=1)
        ar = 0.5 * np.linalg.norm(np.cross(tri[:, 1] - tri[:, 0], tri[:, 2] - tri[:, 0]), axis=1)
        return float(ar[~self.inside(c)].sum())

    def body_exposed_area(self) -> float:
        """Body wetted area minus the parts covered by the wing glove/LERX and the tail roots."""
        m = self.body_mesh
        tri = m.V[m.F]
        c = tri.mean(axis=1)
        ar = 0.5 * np.linalg.norm(np.cross(tri[:, 1] - tri[:, 0], tri[:, 2] - tri[:, 0]), axis=1)
        covered = np.zeros(len(c), bool)
        for surf, mir in [(self.wing, True)] + [(self.tail[k], self.tail_mirror[k]) for k in self.tail]:
            covered |= self._inside_surface(surf, c, mir)
        return float(ar[~covered].sum())

    @staticmethod
    def _inside_surface(surf: oml.LiftingSurface, P: np.ndarray, mirror: bool) -> np.ndarray:
        """Approximate point-in-lifting-surface test (piecewise sections, linear loft) used for wetted-area
        bookkeeping only: a body point is covered if it lies within the local section thickness band."""
        out = np.zeros(len(P), bool)
        secs = surf.sections
        span_dir_z = any("span_dir" in s for s in secs) or abs(secs[-1]["z_le"] - secs[0]["z_le"]) > \
            abs(secs[-1]["y"] - secs[0]["y"])
        Q = P.copy()
        if mirror:
            Q[:, 1] = np.abs(Q[:, 1])
        if span_dir_z:                                       # fins / ventral: span coordinate ~ z
            return out                                       # root footprint small: ignored (conservative)
        ys = np.array([s["y"] for s in secs])
        m = (Q[:, 1] >= ys[0]) & (Q[:, 1] <= ys[-1])
        if not np.any(m):
            return out
        idx = np.where(m)[0]
        for i in idx:
            y = Q[i, 1]
            j = int(np.clip(np.searchsorted(ys, y) - 1, 0, len(ys) - 2))
            t = (y - ys[j]) / max(ys[j + 1] - ys[j], 1e-9)
            a, b = secs[j], secs[j + 1]
            xle = a["x_le"] + t * (b["x_le"] - a["x_le"])
            c = a["chord"] + t * (b["chord"] - a["chord"])
            zle = a["z_le"] + t * (b["z_le"] - a["z_le"])
            if xle <= Q[i, 0] <= xle + c:
                tsc = a.get("thickness_scale", 1.0) + t * (b.get("thickness_scale", 1.0) - a.get("thickness_scale", 1.0))
                half = 0.5 * 0.16 * tsc * c * 1.15
                out[i] = abs(Q[i, 2] - zle) <= half
        return out


# =====================================================================================================================
# 4. aerodynamics: lifting line (aero.py) on the actual wing incl. the LERX, strip profile drag, vortex lattice
# =====================================================================================================================
def wing_analysis(S: dict, af, V: float, h: float, k_sec: float, k_3d: float) -> dict:
    """Wing (glove/LERX + outer panel; the innermost section is extended to the centre line as body carry-over).

    * aero.surface_analysis (Glauert lifting line, NeuralFoil sections): span loading, CL_0 (incidence, twist,
      camber), inviscid span efficiency, strip profile drag and CLmax by the critical-section method with the research
      factors (section cl_max x ``k_sec``, 3-D x ``k_3d``, endurance study). The stall search is restricted to the
      span outboard of the strake (local leading-edge sweep below ``strake_sweep_deg``): a sharp, highly swept LERX
      separates into a stable leading-edge vortex instead of a trailing-edge stall, so its 2-D cl_max is not a wing
      stall criterion (its vortex lift is not credited either).
    * vortex lattice of the same planform (flat camber line): lift slope and aerodynamic centre (the lifting line
      neglects sweep, invalid on the 60-70 deg strake) and the Trefftz-plane span efficiency.
    Coefficients are per the TRAPEZOIDAL reference area of the spec; kS = lifting-line area / reference area."""
    secs = S["wing"]["sections"]
    S_ref, cbar = float(S["wing"]["area"]), float(S["wing"]["mac"])
    sa = AE.surface_analysis(secs, V, h, k_clmax=k_3d)
    LL = sa["ll"]
    kS = sa["S_ref"] / S_ref
    clmax_s = np.interp(LL["y"], sa["tables"]["y"], sa["clmax_sections"]) * k_sec
    a_crit = (clmax_s - LL["cl_0_local"]) / LL["cl_a_local"]
    y_min = float(S["wing"]["planform_derived"]["y_strake_end"])
    a_crit = np.where(LL["y"] >= y_min, a_crit, np.inf)
    i = int(np.argmin(a_crit))
    CLmax_LL = k_3d * LL["CL"](float(a_crit[i]))
    # vortex lattice: lift slope, aerodynamic centre, Trefftz-plane induced drag of the additional loading
    g = vlm_panels(wing_vlm_strips(S, 40, False), 8, tag="wing")
    sol = vlm_solve([g], S_ref, cbar, float(S["wing"]["mac_le_x"]))
    n_c = 8
    gam = sol["gamma"].reshape(-1, n_c).sum(axis=1)
    A_, B_ = sol["A"][::n_c], sol["B"][::n_c]
    dy = B_[:, 1] - A_[:, 1]
    yc = 0.5 * (A_[:, 1] + B_[:, 1])
    zc_ = 0.5 * (A_[:, 2] + B_[:, 2])
    Pt = np.column_stack([np.full_like(yc, 1.0e3), yc, zc_])
    w = vlm_induced(sol, Pt)[:, 2]
    CL1 = 2 * float(np.sum(gam * dy)) / S_ref
    CDi1 = -float(np.sum(gam * w * dy)) / S_ref
    AR = float(S["wing"]["span"]) ** 2 / S_ref
    e_vlm = CL1 ** 2 / (math.pi * AR * CDi1)
    k_ll = 1.0 / (math.pi * sa["AR"] * sa["e_inviscid"] * kS)
    k_vlm = 1.0 / (math.pi * AR * e_vlm)
    out = {"sa": sa, "kS": kS, "S_LL": sa["S_ref"], "CL_alpha_LL": sa["CL_alpha"] * kS, "CL_alpha": sol["CL_alpha"],
           "CL_0": sa["CL_0"] * kS, "e_inv_LL": sa["e_inviscid"], "e_vlm": e_vlm, "k_i_LL": k_ll, "k_i_vlm": k_vlm,
           "k_i": max(k_ll, k_vlm), "x_ac_vlm": sol["x_np"], "CLmax": CLmax_LL * kS,
           "alpha_stall_deg": math.degrees(float(a_crit[i])), "stall_eta": float(LL["y"][i] / sa["semispan"]),
           "y_stall_search_min": y_min, "Re_root": sa["Re_root"], "Re_tip": sa["Re_tip"],
           "min_confidence": sa["min_confidence"], "V": V, "h": h}
    out["alpha_for"] = lambda CL_ref: sa["alpha_for"](CL_ref / kS)
    out["cl_local"] = lambda CL_ref: LL["cl_local"](sa["alpha_for"](CL_ref / kS))
    return out


def strip_profile_drag(S: dict, wa: dict, y_exposed: float, tripped: bool = True):
    """f(CL_ref) -> wing profile drag coefficient on the reference area, strip-integrated over the EXPOSED span
    (y >= y_exposed; the carry-over part inside the body is counted with the body friction) at the lifting-line cl
    distribution, tripped (x_tr 0.075 c) or free-transition section polars; plus the section cm0 integral."""
    sa = wa["sa"]
    T = sa["tables"]
    LL = sa["ll"]
    secs = S["wing"]["sections"]
    atm = AL.isa(wa["h"])
    re = atm["rho"] * wa["V"] * T["chord"] / atm["mu"]
    ys = LL["y"]
    keep = ys >= y_exposed
    chars = []
    for yk in ys:
        j = int(np.argmin(np.abs(T["y"] - yk)))
        i, f = int(T["idx"][j]), float(T["frac"][j])
        sa_, sb_ = secs[i], secs[i + 1]
        ca = section_chars(sa_["airfoil"], float(re[j]), float(sa_.get("thickness_scale", 1.0)), tripped)
        cb = section_chars(sb_["airfoil"], float(re[j]), float(sb_.get("thickness_scale", 1.0)), tripped)
        chars.append((ca, cb, f))
    order = np.argsort(ys)
    yo, co, ko = ys[order], LL["chord"][order], keep[order]
    S_ref = float(S["wing"]["area"])

    def cdp(CL_ref: float) -> float:
        cl = wa["cl_local"](CL_ref)
        cd = np.array([(1 - f) * AE.section_cd(ca, float(c)) + f * AE.section_cd(cb, float(c))
                       for (ca, cb, f), c in zip(chars, cl)])[order]
        y1, c1, d1 = yo[ko], co[ko], cd[ko]
        # extend the integral to exactly y_exposed (linear in the first exposed strip)
        if y1[0] > y_exposed:
            y1 = np.r_[y_exposed, y1]
            c1 = np.r_[np.interp(y_exposed, yo, co), c1]
            d1 = np.r_[d1[0], d1]
        return float(2 * np.trapz(d1 * c1, y1) / S_ref)
    cm0 = np.array([(1 - f) * ca["cm0"] + f * cb["cm0"] for ca, cb, f in chars])[order]
    cbar = float(S["wing"]["mac"])
    Cm0_sections = float(2 * np.trapz(cm0 * co ** 2, yo) / (S_ref * cbar))
    return cdp, Cm0_sections


def lifting_line_moment(S: dict, wa: dict) -> dict:
    """Aerodynamic centre and zero-lift moment of the wing (+ LERX + carry-over) from the lifting-line loading: each
    strip's lift acts at its local quarter chord (x_le + c/4); x_ac is the load-weighted quarter chord of the additional
    (per-alpha) loading, Cm0 about x_ac comes from the basic (twist/camber) loading plus the section cm0 integral."""
    sa = wa["sa"]
    LL = sa["ll"]
    T = sa["tables"]
    ys = LL["y"]
    c = LL["chord"]
    xle = np.interp(ys, T["y"], T["x_le"])
    xq = xle + 0.25 * c
    order = np.argsort(ys)
    la = (LL["cl_a_local"] * c)[order]                      # lift per unit span per rad (x q)
    l0 = (LL["cl_0_local"] * c)[order]
    yo, xo = ys[order], xq[order]
    A_a = np.trapz(la, yo)
    x_ac = float(np.trapz(la * xo, yo) / A_a)
    S_ref, cbar = float(S["wing"]["area"]), float(S["wing"]["mac"])
    # basic loading moment about x_ac (nose-up positive: lift ahead of x_ac -> +)
    Cm_basic = float(2 * np.trapz(l0 * (x_ac - xo), yo) / (S_ref * cbar))
    return {"x_ac": x_ac, "Cm_basic": Cm_basic, "x_c4_strips": xq, "y": ys}


# ---------------------------------------------------------------------------------------------- vortex lattice (VLM)
def _seg_v(P, A, B):
    """Induced velocity at points P (n,3) of unit-strength finite vortex segments A->B (m,3): (n,m,3)."""
    r1 = P[:, None, :] - A[None, :, :]
    r2 = P[:, None, :] - B[None, :, :]
    r0 = (B - A)[None, :, :]
    c = np.cross(r1, r2)
    c2 = np.einsum("nmk,nmk->nm", c, c)
    n1 = np.linalg.norm(r1, axis=2)
    n2 = np.linalg.norm(r2, axis=2)
    with np.errstate(divide="ignore", invalid="ignore"):
        k = np.einsum("nmk,nmk->nm", r0, r1 / n1[..., None] - r2 / n2[..., None]) / (4 * math.pi * c2)
    k = np.where(c2 < 1e-12, 0.0, k)
    return c * k[..., None]


def _semi_v(P, A, d):
    """Unit-strength semi-infinite vortex from A along unit direction d: (n,m,3)."""
    r = P[:, None, :] - A[None, :, :]
    dd = np.broadcast_to(d, r.shape)
    c = np.cross(dd, r)
    c2 = np.einsum("nmk,nmk->nm", c, c)
    nr = np.linalg.norm(r, axis=2)
    with np.errstate(divide="ignore", invalid="ignore"):
        k = (1.0 + np.einsum("nmk,nmk->nm", dd, r) / nr) / (4 * math.pi * c2)
    k = np.where(c2 < 1e-12, 0.0, k)
    return c * k[..., None]


def vlm_panels(strips: list, n_chord: int, eta: float = 1.0, tag: str = "") -> dict:
    """Horseshoe panels from spanwise strips [(le_a, te_a, le_b, te_b)] (3-D points of the strip edges, edge a at the
    lower-y/inner side). Bound vortex at 1/4 panel chord, control point at 3/4 panel chord mid-strip."""
    A, B, C, N, M = [], [], [], [], []
    for le_a, te_a, le_b, te_b in strips:
        for j in range(n_chord):
            f0, f1 = j / n_chord, (j + 1) / n_chord
            pa0 = le_a + f0 * (te_a - le_a)
            pa1 = le_a + f1 * (te_a - le_a)
            pb0 = le_b + f0 * (te_b - le_b)
            pb1 = le_b + f1 * (te_b - le_b)
            A.append(pa0 + 0.25 * (pa1 - pa0))
            B.append(pb0 + 0.25 * (pb1 - pb0))
            ca = pa0 + 0.75 * (pa1 - pa0)
            cb = pb0 + 0.75 * (pb1 - pb0)
            C.append(0.5 * (ca + cb))
            n = np.cross(pb1 - pa0, pa1 - pb0)               # diagonals: chordwise x spanwise -> "up" for wings
            n = n / np.linalg.norm(n)
            if n[2] < 0 or (abs(n[2]) < 1e-9 and n[1] < 0):
                n = -n
            N.append(n)
            M.append(eta)
    return {"A": np.array(A), "B": np.array(B), "C": np.array(C), "N": np.array(N), "eta": np.array(M), "tag": tag}


def vlm_solve(groups: list, S_ref: float, cbar: float, x_ref: float) -> dict:
    """Linear VLM: unit-alpha solution (V = 1, rho = 1). Returns CL_alpha, Cm_alpha about x_ref, x_np, panel loads."""
    A = np.vstack([g["A"] for g in groups])
    B = np.vstack([g["B"] for g in groups])
    C = np.vstack([g["C"] for g in groups])
    N = np.vstack([g["N"] for g in groups])
    eta = np.concatenate([g["eta"] for g in groups])
    d = np.array([1.0, 0.0, 0.0])
    V = _seg_v(C, A, B) + _semi_v(C, B, d) - _semi_v(C, A, d)
    AIC = np.einsum("nmk,nk->nm", V, N)
    rhs = -N[:, 2]                                            # d(V_inf . n)/d alpha = n_z
    gam = np.linalg.solve(AIC, rhs)
    l = B - A
    F = gam[:, None] * np.cross(np.broadcast_to(d, l.shape), l) * eta[:, None]
    mid = 0.5 * (A + B)
    Fz = F[:, 2]
    q = 0.5
    CLa = float(Fz.sum() / (q * S_ref))
    My = -(mid[:, 0] - x_ref) * Fz                            # lift aft of x_ref -> nose down
    Cma = float(My.sum() / (q * S_ref * cbar))
    x_np = x_ref - Cma / CLa * cbar
    sizes = [len(g["A"]) for g in groups]
    idx = np.cumsum([0] + sizes)
    parts = {}
    for g, i0, i1 in zip(groups, idx[:-1], idx[1:]):
        parts[g["tag"]] = {"CL_alpha": float(Fz[i0:i1].sum() / (q * S_ref)),
                           "x_load": float((Fz[i0:i1] * mid[i0:i1, 0]).sum() / max(abs(Fz[i0:i1].sum()), 1e-12))}
    return {"CL_alpha": CLa, "Cm_alpha": Cma, "x_np": x_np, "gamma": gam, "parts": parts, "A": A, "B": B, "N": N,
            "eta": eta}


def vlm_induced(sol: dict, P: np.ndarray) -> np.ndarray:
    """Induced velocity (per unit alpha, V = 1) at points P from a solved lattice."""
    d = np.array([1.0, 0.0, 0.0])
    A, B = sol["A"], sol["B"]
    V = _seg_v(P, A, B) + _semi_v(P, B, d) - _semi_v(P, A, d)
    return np.einsum("nmk,m->nk", V, sol["gamma"])


def _mirror_strips(strips):
    out = []
    for le_a, te_a, le_b, te_b in strips:
        f = np.array([1.0, -1.0, 1.0])
        out.append((le_b * f, te_b * f, le_a * f, te_a * f))
    return out[::-1] + strips


def surface_strips(sections: list, n_span: int, mirror: bool, cosine: bool = True) -> list:
    """VLM strips along the leading-edge polyline of a lifting surface (spec sections; flat camber line)."""
    P_le = np.array([[s["x_le"], s["y"], s["z_le"]] for s in sections], float)
    P_te = np.array([[s["x_le"] + s["chord"], s["y"], s["z_le"]] for s in sections], float)
    seg = np.r_[0.0, np.cumsum(np.linalg.norm(np.diff(P_le[:, 1:], axis=0), axis=1))]
    u = 0.5 * (1 - np.cos(np.linspace(0, math.pi, n_span + 1))) if cosine else np.linspace(0, 1, n_span + 1)
    s = u * seg[-1]
    le = np.column_stack([np.interp(s, seg, P_le[:, k]) for k in range(3)])
    te = np.column_stack([np.interp(s, seg, P_te[:, k]) for k in range(3)])
    strips = [(le[i], te[i], le[i + 1], te[i + 1]) for i in range(n_span)]
    return _mirror_strips(strips) if mirror else strips


# =====================================================================================================================
# 5. structure mass estimates (endurance-study structural concept: process minimum gauges + spar caps from the loads)
# =====================================================================================================================
def areal_masses(S: dict) -> dict:
    """Sandwich areal masses (kg/m2) from the spec materials (MTM45-1/AS4 plain weave plies, ROHACELL 51 WF core) and
    the paint allowance: primary wing skin 0.6/5 mm/0.4, secondary shell 0.4/5 mm/0.4, tail 0.6/4 mm/0.4, rib panel
    0.4/6 mm/0.4 (endurance study rules)."""
    M = S["materials"]
    pw = M["cfrp_pw_mtm45_as4"]
    ply = pw["ply_t"] * pw["density"]
    core = M["core_rohacell_51wf"]["density"]
    paint = float(S["structures"]["paint_areal_kg_per_m2"])
    return {"ply_pw": ply, "core": core, "paint": paint,
            "wing_skin": 3 * ply + 0.005 * core + 2 * ply + paint,
            "shell": 2 * ply + 0.005 * core + 2 * ply + paint,
            "tail_skin": 3 * ply + 0.004 * core + 2 * ply + paint,
            "rib": 2 * ply + 0.006 * core + 2 * ply}


def allowables(S: dict) -> dict:
    """Ultimate design allowables of the wing box (endurance study): spar cap = min(E x 3000 ue damage-tolerance limit,
    Fcu B-basis ETW x 0.85 single load path / 1.2 special factor); web shear = G_qi x 5200 ue."""
    M = S["materials"]
    ud = M["cfrp_ud_mtm45_as4"]
    dl = S["structures"]
    eps_c = dl["damage_tolerance_strain_ue"]["compression_thick"] * 1e-6
    sig = min(ud["E"] * eps_c, ud["Fcu"] * dl["a_basis_factor_single_load_path"] / dl["composite_special_factor"])
    tau = dl["laminate_G_qi_Pa"] * dl["damage_tolerance_strain_ue"]["shear_thick"] * 1e-6
    return {"spar_cap_Pa": sig, "web_Pa": tau, "rho_ud": ud["density"], "rho_pw": M["cfrp_pw_mtm45_as4"]["density"]}


def wing_structure(S: dict, af: Airframe, m0: float, n_lim: float) -> dict:
    """Wing (both halves incl. the LERX glove; the carry-through box is a chassis item) for the ultimate load
    FoS x n_lim x MTOM x g on the Schrenk distribution of the REFERENCE TRAPEZOID chord (the LERX chord is not
    extended to the centre line, which would move load inboard and relieve the root moment; no inertia relief).
    Spar depth: outer panel 0.95 t - 10 mm; glove (body side to the outer-panel joint): the actual OML depth along the
    main spar line (spar_depth_profile); inside the body: the carry-through box (junction thickness). Rear spar web:
    the actual depth along the rear spar line."""
    W = S["wing"]
    P = W["planform"]
    T = trapezoid(P)
    am, alw = areal_masses(S), allowables(S)
    fos = float(S["structures"]["factor_of_safety"])
    secs = W["sections"]
    ys_s = np.array([s["y"] for s in secs])
    b2 = T["b2"]
    y = np.linspace(0.0, b2, 121)
    c_act = np.interp(y, ys_s, [s["chord"] for s in secs])
    l = ST.schrenk(y, T["c"](y), b2)
    w = fos * n_lim * m0 * G / 2 * l
    Vs, Ms = ST.beam_loads(y, w)
    yj = P["y_junction"]
    t_out = 0.1595 * np.interp(y, ys_s, [s.get("thickness_scale", 1.0) for s in secs]) * T["c"](y)
    t_j = 0.1595 * float(T["c"](yj))
    prof = spar_depth_profile(S, af, P["main_spar_frac"])
    y_p = np.array([r_["y"] for r_ in prof["rows"]])
    d_p = np.array([r_["depth"] for r_ in prof["rows"]])
    t_abs = np.where(y < y_p[0], t_j, np.where(y < yj, np.interp(y, y_p, d_p), t_out))
    # rear spar web depth: the actual OML depth along the rear spar line (outer panel: NLF(1)-0416 depth at the rear
    # spar chord fraction; glove: spar_depth_profile; inside the body: the joint value)
    sdt = spar_depth_targets(P)
    prof_r = spar_depth_profile(S, af, P["rear_spar_frac"])
    y_r = np.array([r_["y"] for r_ in prof_r["rows"]])
    d_r = np.array([r_["depth"] for r_ in prof_r["rows"]])
    t_rear = np.where(y < y_r[0], sdt["rear_joint_depth"], np.where(y < yj, np.interp(y, y_r, d_r),
                                                                     sdt["rear_depth_ratio_outer"] * t_out))
    h_eff = np.maximum(0.95 * t_abs - 0.010, 0.012)
    A_cap = np.maximum(Ms / (h_eff * alw["spar_cap_Pa"]), 40e-6)
    caps = 2 * 2 * np.trapz(A_cap, y) * alw["rho_ud"] * 1.10
    t_web = np.maximum(Vs / (h_eff * alw["web_Pa"]), 0.6e-3)
    webs = 2 * np.trapz(t_web * h_eff, y) * alw["rho_pw"] * 1.25
    rear = 2 * np.trapz(0.6e-3 * t_rear + 2 * 20e-6 * alw["rho_ud"] / alw["rho_pw"], y) * alw["rho_pw"]
    n_ribs = int(math.ceil((b2 - P["y_root_structure"]) / float(S["structures"]["rib_pitch_m"]))) + 1
    yr = np.linspace(P["y_root_structure"], b2, n_ribs)
    ribs = 2 * float(np.sum(0.68 * np.interp(yr, y, t_abs) * np.interp(yr, y, np.minimum(c_act, 1.2 * T["c"](y))) *
                            0.6)) * am["rib"]
    S_wet = 2 * af.exposed_area(af.wing_mesh)
    skins = S_wet * am["wing_skin"]
    joints = 2 * float(S["structures"]["outer_panel_joint_kg"])
    sub = caps + webs + rear + ribs + skins + joints
    total = sub * 1.05
    i_g = int(np.argmin(np.where((y >= y_p[0]) & (y < yj), h_eff, np.inf)))
    return {"total": total, "caps": caps, "webs": webs, "rear_spar": rear, "ribs": ribs, "skins": skins,
            "joints": joints, "S_wet": S_wet, "M_root_ult_Nm": float(Ms[0]),
            "M_junction_ult_Nm": float(np.interp(yj, y, Ms)), "A_cap_root_mm2": float(A_cap[0] * 1e6),
            "h_eff_root_m": float(h_eff[0]), "h_eff_glove_min_m": float(h_eff[i_g]),
            "A_cap_glove_max_mm2": float(A_cap[i_g] * 1e6), "y_glove_min_depth": float(y[i_g]),
            "n_ult": fos * n_lim, "spar_cap_allowable_Pa": alw["spar_cap_Pa"], "schrenk_basis": "reference trapezoid"}


def tail_structure(S: dict, af: Airframe) -> dict:
    """Tail surfaces: sandwich skins on the exposed wetted area + spar + ribs; fittings per surface from the spec."""
    am, alw = areal_masses(S), allowables(S)
    out = {}
    for k, v in S["tail"]["surfaces"].items():
        n = 2 if v.get("mirror", True) else 1
        S_wet = n * af.exposed_area(af.tail_meshes[k])
        pp = panel_planform(v["sections"])
        spar = n * pp["span"] * (2 * 30e-6 * alw["rho_ud"] + 0.6e-3 * 0.05 * alw["rho_pw"])
        ribs = n * 5 * 0.004 * am["rib"]
        fit = n * float(v["fittings_kg"])
        tot = (S_wet * am["tail_skin"] + spar + ribs + fit) * 1.05
        out[k] = {"total": tot, "S_wet": S_wet, "skins": S_wet * am["tail_skin"], "spar": spar, "fittings": fit}
    return out


def body_shell(S: dict, af: Airframe) -> dict:
    am = areal_masses(S)
    S_exp = af.body_exposed_area()
    chine_len = 2 * float(np.sum(np.hypot(np.diff(np.asarray(S["fuselage"]["stations"])[:, 0]),
                                          np.diff(np.asarray(S["fuselage"]["stations"])[:, 1] / 2))))
    band = chine_len * float(S["structures"]["chine_edge_band_kg_per_m"])
    return {"S_wet_exposed": S_exp, "skin": S_exp * am["shell"], "chine_bands": band, "chine_length": chine_len,
            "total": S_exp * am["shell"] + band}


# =====================================================================================================================
# 6. mass properties from the spec mass items (+ fuel, payload per loading case)
# =====================================================================================================================
def mass_cases(S: dict) -> list[dict]:
    """CG of every loading case from spec mass items (empty incl. growth), payload items and fuel."""
    M = S["mass"]
    items = M["items"]
    pay = {p["name"]: p for p in M["payload_items"]}
    fuel = M["fuel_kg"]
    fx, fy, fz = M["fuel_cg"]
    out = []
    for c in M["cases"]:
        its = [(i["mass_kg"], i["x"], i["y"], i["z"]) for i in items]
        names = c.get("payload", c.get("payload_items", []))
        for name in names:
            p = pay[name]
            z = p["z_extended"] if (c.get("turret") == "extended" and "z_extended" in p) else p["z"]
            its.append((p["mass_kg"], p["x"], p.get("y", 0.0), z))
        its.append((fuel * c["fuel_fraction"], fx, fy, fz))
        m = sum(i[0] for i in its)
        cg = [sum(i[0] * i[k] for i in its) / m for k in (1, 2, 3)]
        out.append({"name": c["name"], "m": m, "x": cg[0], "y": cg[1], "z": cg[2],
                    "fuel_fraction": c["fuel_fraction"], "payload_kg": sum(pay[n]["mass_kg"] for n in names)})
    return out


def empty_mass(S: dict) -> dict:
    items = S["mass"]["items"]
    m = sum(i["mass_kg"] for i in items)
    by = {}
    for i in items:
        by[i["group"]] = by.get(i["group"], 0.0) + i["mass_kg"]
    x = sum(i["mass_kg"] * i["x"] for i in items) / m
    z = sum(i["mass_kg"] * i["z"] for i in items) / m
    return {"empty": m, "groups": by, "x": x, "z": z}


# =====================================================================================================================
# 7. drag build-up (aerolib on mesh wetted areas) and trimmed polars
# =====================================================================================================================
def body_fineness(S: dict, af: Airframe) -> dict:
    st = np.asarray(S["fuselage"]["stations"], float)
    xs = np.linspace(af.fus.x0, af.fus.x1, 400)
    A = np.array([af.area(x) for x in xs])
    w, h, *_ = af.fus.section(xs)
    i = int(np.argmax(A))
    d_eq = math.sqrt(4 * A[i] / math.pi)
    return {"A_max": float(A[i]), "x_A_max": float(xs[i]), "d_eq": d_eq, "fineness": float(af.L / d_eq),
            "w_max": float(w.max()), "h_max": float(h.max()), "S_side": float(np.trapz(h, xs)),
            "volume": float(np.trapz(A, xs)), "length": float(st[-1, 0])}


def aft_closure(S: dict, af: Airframe) -> dict:
    """Aft closure of the cowl: equivalent radius r_eq(x) = sqrt(A/pi); separation onset where the closure slope
    -dr_eq/dx first exceeds tan(boattail_separation_deg) aft of the body's largest engine-bay section; base area =
    A(separation) - area of the cowl lip circle (spinner + cooling-exit annulus)."""
    L = S["fuselage"]["lines"]
    A_r = S["aero"]["drag_rules"]
    xh = float(L["x_hub"])
    xs = np.linspace(xh - 0.45, xh + float(L["l_cowl_tail"]), 181)
    A = np.array([af.area(x) for x in xs])
    r = np.sqrt(A / math.pi)
    sl = -np.gradient(r, xs)
    lim = math.tan(math.radians(float(A_r["boattail_separation_deg"])))
    i0 = int(np.argmax(A))
    idx = np.where((np.arange(len(xs)) >= i0) & (sl > lim))[0]
    A_lip = math.pi * float(L["r_lip"]) ** 2
    if not len(idx):
        return {"x_sep": None, "base_area_m2": 0.0, "max_closure_deg": float(np.degrees(np.arctan(sl.max())))}
    j = int(idx[0])
    return {"x_sep": float(xs[j]), "A_sep_m2": float(A[j]), "A_lip_m2": A_lip,
            "base_area_m2": max(float(A[j]) - A_lip, 0.0),
            "max_closure_deg": float(np.degrees(np.arctan(sl[i0:].max())))}


def wing_root_junction(S: dict, af: Airframe) -> tuple:
    """(thickness, t/c) of the wing at the body side (chine) for the junction-drag estimate: the largest section
    thickness of the exposed glove root (first exposed section)."""
    a_b = float(S["fuselage"]["lines"]["a_max"])
    for s_ in S["wing"]["sections"]:
        if s_["y"] >= a_b - 1e-9:
            tc = oml.max_thickness(s_["airfoil"])[0] * float(s_.get("thickness_scale", 1.0))
            return tc * s_["chord"], tc
    s_ = S["wing"]["sections"][-1]
    tc = oml.max_thickness(s_["airfoil"])[0]
    return tc * s_["chord"], tc


def drag_items(S: dict, af: Airframe, V: float, h: float, cfg: dict, ff_ratio: float) -> dict:
    """Parasite drag items other than the wing profile drag, as D/q (m2), at TAS V and altitude h.
    cfg: {"turret": "extended"|"retracted", "gear": "up"|"down"}."""
    A = S["aero"]["drag_rules"]
    atm = AL.isa(h)
    M = V / atm["a"]
    fb = body_fineness(S, af)
    items = {}
    S_b = af.body_exposed_area()
    items["body"] = AL.cf_flat(AL.reynolds(V, af.L, h)) * AL.ff_body(fb["fineness"]) * A["Q_chine"] * S_b
    # aft-body upsweep (Raymer eq. 12.36: D/q = 3.83 u^2.5 A_max, u in rad) from the keel line
    L = S["fuselage"]["lines"]
    x0, x1 = L["x_hub"] - L["l_upsweep"], L["x_hub"] - L["l_slab_aft"] - 0.10
    u = math.atan2(af.z_bot(x1) - af.z_bot(x0), x1 - x0)
    items["aft_body_upsweep"] = 3.83 * abs(u) ** 2.5 * fb["A_max"]
    for k, v in S["tail"]["surfaces"].items():
        n = 2 if v.get("mirror", True) else 1
        pp = panel_planform(v["sections"])
        tc = oml.max_thickness(v["sections"][0]["airfoil"])[0]
        vp = v["params"]
        sw = math.degrees(math.atan(math.tan(math.radians(vp.get("sweep_le_deg", 0.0))) -
                                    0.3 * (vp.get("root_chord", 0.0) - vp.get("tip_chord", 0.0)) / max(pp["span"], 1e-6)))
        S_wet = n * af.exposed_area(af.tail_meshes[k])
        items[f"tail_{k}"] = AL.cf_flat(AL.reynolds(V, pp["mac"], h)) * AL.ff_wing(tc, 0.30, sw, M) * A["Q_tail"] * S_wet
    # landing gear (Raymer 6th ed. Table 12.6 D/q per frontal area) x gear/body interference; retracted: residual
    G_ = S["landing_gear"]
    tyre_d, tyre_w = G_["tyre"]["diameter"], G_["tyre"]["width"]
    a_wheel = tyre_d * tyre_w
    legs = 2 * G_["main"]["leg_length"] * G_["main"]["leg_frontal_width"] + G_["nose"]["leg_length"] * \
        G_["nose"]["leg_frontal_width"]
    down = A["gear_interference"] * (3 * A["wheel_Dq_per_area"] * a_wheel + A["strut_Dq_per_area"] * legs +
                                    A["door_Dq_open"])
    items["landing_gear"] = down if cfg.get("gear") == "down" else A["gear_residual_fraction"] * down
    # EO/IR turret: exposed ball (Hoerner sphere CD) + stem when extended; door gaps when retracted
    P = S["payload"]["turret"]
    if cfg.get("turret") == "extended":
        d_b = P["ball_diameter"]
        zs = af.z_bot(P["bay_center_x"])
        f_exp = min(max((zs - (P["ball_center_extended_z"] - d_b / 2)) / d_b, 0.0), 1.0)
        a_ball = math.pi / 4 * d_b ** 2 * (0.5 * (1 - math.cos(math.pi * f_exp)))    # exposed share of the disc
        base = max(zs - (P["ball_center_extended_z"] + d_b / 2), 0.0)
        # open bay around the lowered ball (doors folded inward): cavity drag on the open area at the skin plane
        hw = P["growth_envelope"]["diameter"] / 2 + 0.01
        dz = zs - P["ball_center_extended_z"]
        r_skin = math.sqrt(max((d_b / 2) ** 2 - dz ** 2, 0.0)) if abs(dz) < d_b / 2 else 0.0
        stem = math.pi / 4 * P["stem_diameter"] ** 2 if dz >= d_b / 2 else 0.0
        a_open = max((2 * hw) ** 2 - math.pi * r_skin ** 2 - stem, 0.0)
        items["eo_ir_turret"] = A["turret_ball_cd"] * a_ball + A["turret_stem_cd"] * P["stem_diameter"] * base + \
            A["turret_door_gaps_Dq"]
        items["turret_bay_cavity"] = A["cavity_cd_open_area"] * a_open
    else:
        items["eo_ir_turret"] = A["turret_door_gaps_Dq"]
    # cooling: momentum loss of the cooling air (identity study method) for the dorsal S-duct inlet
    items["engine_cooling"] = cooling_Dq(S, V, h, ff_ratio)
    # aft closure: the cowl closes steeply onto the spinner/cooling-exit lip; the part of the closure steeper than
    # drag_rules.boattail_separation_deg is treated as a separated base (Hoerner 1965 base-pressure relation
    # C_Db = 0.029 / sqrt(C_Df), C_Df = forebody friction drag referred to the base area); no pusher suction credit
    ac = aft_closure(S, af)
    if ac["base_area_m2"] > 0:
        cdf = items["body"] / ac["base_area_m2"]
        items["aft_closure_base"] = 0.029 / math.sqrt(cdf) * ac["base_area_m2"]
    # wing-body junctions (Hoerner 1965 plain-junction interference, conservative for the LERX blend; both sides)
    t_j, tc_j = wing_root_junction(S, af)
    items["wing_body_junctions"] = 2 * max(0.75 * tc_j - 0.0003 / tc_j ** 2, 0.0) * t_j ** 2
    items["antennas_pitot_lights_vents"] = A["misc_Dq"]
    items["control_surface_gaps"] = A["gap_dCD_per_pair"] * float(S["wing"]["area"]) * A["control_gap_pairs"]
    base = sum(items.values())
    items["leakage_protuberance"] = A["leakage_fraction"] * base
    return items


@dataclass
class Polar:
    """Trimmed drag polar of one configuration (reference area S). CD(CL) table + parabolic fit."""
    name: str
    S: float
    CLs: np.ndarray
    CDs: np.ndarray
    CLt: np.ndarray
    clmax: dict
    fit: dict = field(default_factory=dict)
    items: dict = field(default_factory=dict)

    def CD(self, CL):
        return np.interp(CL, self.CLs, self.CDs)


def fit_polar(CLs, CDs, lo=0.25, hi=1.25):
    m = (CLs >= lo) & (CLs <= hi)
    k, cd0 = np.polyfit(CLs[m] ** 2, CDs[m], 1)
    return float(cd0), float(k)


def trimmed_polar(name: str, S: dict, wa: dict, cdp, k_cd_trip: float, stab: dict, cd_rest: float, cg: tuple,
                  tail: dict, clmax: dict, items: dict) -> Polar:
    """CD(CL) with the tail trim load (wing-body Cm0 and AC incl. the body, CG, thrust-line moment with T = D/cos(eps)
    about the CG, thrust_arm) and the stabilator induced drag (endurance study trimmed_polar with the wing-body terms
    of this configuration)."""
    x_cg, z_cg = cg
    S_ref, cbar = float(S["wing"]["area"]), float(S["wing"]["mac"])
    arm = tail["x_ac"] - x_cg
    e = math.radians(float(S["propeller"].get("thrust_line_inclination_deg", 0.0)))
    d_thr = thrust_arm(S, x_cg, z_cg) / math.cos(e)
    CLs = np.linspace(-0.2, 1.9, 85)
    CD, CLT = [], []
    for CL in CLs:
        CLw = CL
        for _ in range(8):
            cdw = cdp(min(CLw, 2.2)) * k_cd_trip + wa["k_i"] * CLw ** 2
            cm = stab["Cm0_wb"] + CLw * (x_cg - stab["x_ac_wb"]) / cbar - (cd_rest + cdw) * d_thr / cbar
            clt = cm * cbar / arm
            CLw = CL - clt
        cdi_t = clt ** 2 * S_ref / (math.pi * tail["b_eff"] ** 2 * tail["e"])
        CD.append(cd_rest + cdw + cdi_t)
        CLT.append(clt)
    CLs, CDs = np.asarray(CLs), np.asarray(CD)
    cd0, k = fit_polar(CLs, CDs)
    ld = CLs / CDs
    i = int(np.argmax(ld))
    ep = np.where(CLs > 0, np.clip(CLs, 0, None) ** 1.5 / CDs, 0)
    j = int(np.argmax(ep))
    AR = float(S["wing"]["aspect_ratio"])
    fit = {"cd0": cd0, "k": k, "e": 1.0 / (math.pi * AR * k), "LD_max": float(ld[i]), "CL_LDmax": float(CLs[i]),
           "CL_endurance": float(CLs[j]), "endurance_param_max": float(ep[j]), "cd_rest": cd_rest}
    return Polar(name, S_ref, CLs, CDs, np.asarray(CLT), clmax, fit, items)


# =====================================================================================================================
# 8. stability: wing AC + LERX (lifting line), lifting-body Multhopp moment, stabilators, canted fins, VLM cross-check
# =====================================================================================================================
def tail_props(S: dict, V: float, h: float, af: "Airframe | None" = None) -> dict:
    """Stabilator lift slope from the lifting line on the moving panel sections (force on the exposed area only; the
    fixed root stub keeps a constant 8 mm root gap, so the body carry-over of the slope is kept), canted-fin and
    ventral-fin side-force slopes (Helmbold with the end-plate aspect-ratio factor), MAC/AC points. Fins: the root is
    buried in the body along the whole chord, so area, MAC, AC and the area centroid are those of the EXPOSED panel
    (Airframe.exposed_panel) when ``af`` is given."""
    T = S["tail"]["surfaces"]
    st = T["stabilator"]
    pp = panel_planform(st["sections"])
    sa = AE.surface_analysis(st["sections"], V, h)
    out = {"stabilator": {**pp, "S_exposed": 2 * pp["area"], "a": sa["CL_alpha"], "e": float(S["tail"]["e_tail"]),
                          "b_eff": 2 * (st["sections"][-1]["y"]), "x_ac": pp["x_ac"], "z": pp["z_mac"],
                          "sa_e": sa["e_inviscid"], "clmax_panel": sa["CLmax"]}}
    k_ep = float(S["tail"]["fin_endplate_ar_factor"])
    for k in ("fin", "ventral"):
        v = T[k]
        pp = panel_planform(v["sections"])
        if k == "fin" and af is not None:
            ex = af.exposed_panel(v["sections"])
            pp = dict(pp, area=ex["area"], mac=ex["mac"], x_ac=ex["x_ac"], y_mac=ex["y"], z_mac=ex["z"],
                      span=ex["span"], aspect_ratio=ex["span"] ** 2 / max(ex["area"], 1e-9), area_panel=pp["area"])
        ch = AE.characteristics(v["sections"][0]["airfoil"], AL.reynolds(V, pp["mac"], h))
        P = v["params"]
        lam = P["tip_chord"] / P["root_chord"]
        AR = pp["aspect_ratio"] * k_ep
        tan_half = math.tan(math.radians(P["sweep_le_deg"])) - 2.0 / pp["aspect_ratio"] * (1 - lam) / (1 + lam)
        a = AL.lift_slope(AR, math.degrees(math.atan(tan_half)), cla_2d=ch["cl_alpha"])
        out[k] = {**pp, "a": a, "AR_eff": AR, "cant_deg": P.get("cant_deg", 0.0)}
    return out


def wing_vlm_strips(S: dict, n_span: int, with_body: bool, af: Airframe | None = None) -> list:
    """Starboard+port strips of the wing (incl. LERX). with_body: the planform of the chined forebody ahead of the
    LERX apex is added (body treated as a flat lifting surface from the nose to the wing trailing edge; the afterbody
    behind the wing trailing edge is non-lifting, i.e. separated slender afterbody, the usual assumption)."""
    secs = S["wing"]["sections"]
    ys = np.array([s["y"] for s in secs])
    xle = np.array([s["x_le"] for s in secs])
    xte = xle + np.array([s["chord"] for s in secs])
    zle = np.array([s["z_le"] for s in secs])
    b2 = ys[-1]
    u = 0.5 * (1 - np.cos(np.linspace(0, math.pi, n_span + 1)))
    yy = u * b2
    if with_body:
        xs = np.linspace(0.0, S["wing"]["planform"]["x_apex"], 400)
        a = af.sec(xs)[0]
        a_mono = np.maximum.accumulate(a)
        LE = np.where(yy < ys[0], np.interp(yy, a_mono, xs), np.interp(yy, ys, xle))
    else:
        LE = np.interp(yy, ys, xle)
        LE = np.where(yy < ys[0], xle[0], LE)
    TE = np.interp(yy, ys, xte)
    Z = np.interp(yy, ys, zle)
    pts_le = np.column_stack([LE, yy, Z])
    pts_te = np.column_stack([TE, yy, Z])
    strips = [(pts_le[i], pts_te[i], pts_le[i + 1], pts_te[i + 1]) for i in range(n_span)]
    return _mirror_strips(strips)


def multhopp_body(S: dict, af: Airframe, upwash, x_le_root: float, x_te_root: float, dx: float = 0.02) -> dict:
    """Body pitching-moment slope by Multhopp's strip method (Perkins & Hage; Nelson eq. 2.33 in per-rad form):
    Cm_alpha_f = (pi/2)/(S c) sum w_f^2 (d beta/d alpha) dx over the body ahead of the wing-root leading edge (upwash
    field of the wing) and behind the root trailing edge (downwash field); the part covered by the wing root chord is
    excluded. d beta/d alpha = 1 + w_induced/V per unit alpha, taken from the vortex-lattice solution of the wing."""
    S_ref, cbar = float(S["wing"]["area"]), float(S["wing"]["mac"])
    xa = np.arange(dx / 2, x_le_root, dx)
    xb = np.arange(x_te_root + dx / 2, af.L, dx)
    xs = np.r_[xa, xb]
    a, bt, bb, zc, nt, nb = af.sec(xs)
    P = np.column_stack([xs, np.zeros_like(xs), zc])
    vz = upwash(P)
    dbda = 1.0 + vz
    w = 2 * a
    contrib = math.pi / 2 * w ** 2 * dbda * dx / (S_ref * cbar)
    fwd = float(contrib[: len(xa)].sum())
    aft = float(contrib[len(xa):].sum())
    return {"Cm_alpha": fwd + aft, "forebody": fwd, "afterbody": aft, "dbeta_dalpha_fwd_max": float(dbda[:len(xa)].max())
            if len(xa) else 1.0, "dbeta_dalpha_aft_min": float(dbda[len(xa):].min()) if len(xb) else 1.0}


def stability(S: dict, af: Airframe, wa: dict, llm: dict, Cm0_sections: float, tp: dict, cases: list) -> dict:
    """Neutral point by two methods; the more forward one is used for the static margins (conservative):
    (a) classic: lifting-line wing (+LERX, carry-over) AC, Multhopp lifting-body moment, stabilators with the VLM
        downwash gradient at the tail (DATCOM value reported), canted fins' horizontal projection;
    (b) vortex lattice of body planform + LERX + wing + stabilators + canted fins (eta_t on the tail loads)."""
    S_ref, cbar, b = float(S["wing"]["area"]), float(S["wing"]["mac"]), float(S["wing"]["span"])
    A = S["aero"]["stability_rules"]
    eta_t = float(A["eta_tail"])
    st = tp["stabilator"]
    fin = tp["fin"]
    # ---- (a) classic
    wing_only = vlm_panels(wing_vlm_strips(S, 40, False), 6, tag="wing")
    sol_w = vlm_solve([wing_only], S_ref, cbar, float(S["wing"]["mac_le_x"]))
    up = lambda P: vlm_induced(sol_w, P)[:, 2] / max(sol_w["CL_alpha"], 1e-9) * wa["CL_alpha"]   # noqa: E731
    secs = S["wing"]["sections"]
    x_le_root = secs[0]["x_le"]
    x_te_root = secs[0]["x_le"] + secs[0]["chord"]
    mh = multhopp_body(S, af, up, x_le_root, x_te_root)
    P_t = np.array([[st["x_ac"], st["y_mac"], st["z"]]])
    deps_vlm = float(-up(P_t)[0])
    l_h = st["x_ac"] - wa["x_ac_vlm"]
    deps_datcom = SL.downwash_gradient(float(S["wing"]["aspect_ratio"]), float(S["wing"]["planform"]["taper"]),
                                       l_h, st["z"] - float(S["wing"]["planform"]["z_root"]), b)
    deps = max(deps_vlm, deps_datcom) if A["downwash"] == "max" else deps_vlm
    S_h = st["S_exposed"]
    # canted fins: horizontal projection sin^2(cant) of the fin pair acts as extra horizontal tail
    g = math.radians(fin["cant_deg"])
    S_fin_h = 2 * fin["area"] * math.sin(g) ** 2
    a_fin_h = fin["a"]
    k_tail = eta_t * st["a"] * (S_h / S_ref) * (1 - deps) + eta_t * a_fin_h * (S_fin_h / S_ref) * (1 - deps)
    x_ac_t_mix = (eta_t * st["a"] * S_h * st["x_ac"] + eta_t * a_fin_h * S_fin_h * fin["x_ac"]) / \
        (eta_t * st["a"] * S_h + eta_t * a_fin_h * S_fin_h)
    a_w = wa["CL_alpha"]
    x_ac_w = wa["x_ac_vlm"]
    x_np_a = (a_w * x_ac_w + k_tail * x_ac_t_mix - mh["Cm_alpha"] * cbar) / (a_w + k_tail)
    x_ac_wb = x_ac_w - mh["Cm_alpha"] * cbar / a_w
    # ---- (b) VLM of the whole configuration
    g_wb = vlm_panels(wing_vlm_strips(S, 40, True, af), 8, tag="wing_body")
    tsurf = S["tail"]["surfaces"]
    g_st = vlm_panels(surface_strips(tsurf["stabilator"]["sections"], 8, True), 4, eta=eta_t, tag="stabilator")
    g_fin = vlm_panels(surface_strips(tsurf["fin"]["sections"], 6, True), 4, eta=eta_t, tag="fins")
    groups = [g_wb, g_st, g_fin]
    if "stabilator_stub" in tsurf:                            # fixed root stubs (part of the horizontal tail)
        groups.append(vlm_panels(surface_strips(tsurf["stabilator_stub"]["sections"], 2, True, cosine=False), 4,
                                 eta=eta_t, tag="stabilator_stub"))
    sol_all = vlm_solve(groups, S_ref, cbar, float(S["wing"]["mac_le_x"]))
    sol_wb = vlm_solve([g_wb], S_ref, cbar, float(S["wing"]["mac_le_x"]))
    x_np_b = sol_all["x_np"]
    x_np = min(x_np_a, x_np_b) if A["np_rule"] == "forward" else x_np_a
    sms = {c["name"]: (x_np - c["x"]) / cbar for c in cases}
    # Cm0 of the wing-body (sections + basic loading); body camber (droop/upsweep) moment by Multhopp's Cm0 term
    Cm0_wb = Cm0_sections + llm["Cm_basic"]
    # ---- directional stability
    fb = body_fineness(S, af)
    x_cg = cases[0]["x"]
    l_v = fin["x_ac"] - x_cg
    k_v = float(A["eta_v"]) * (1 + float(A["dsigma_dbeta"]))
    cnb_fins = k_v * fin["a"] * 2 * fin["area"] * math.cos(g) ** 2 * l_v / (S_ref * b)
    vf = tp["ventral"]
    cnb_vent = k_v * vf["a"] * vf["area"] * (vf["x_ac"] - x_cg) / (S_ref * b)
    cnb_body_raymer = -1.3 * fb["volume"] / (S_ref * b) * (fb["h_max"] / fb["w_max"])
    cnb_body_datcom = SL.cn_beta_fuselage(float(A["K_N"]), float(A["K_Rl"]), fb["S_side"], af.L, S_ref, b)
    cnb_body = min(cnb_body_raymer, cnb_body_datcom)
    cnb = cnb_fins + cnb_vent + cnb_body
    # roll stiffness (dihedral effect, estimate): wing dihedral + canted fins above the CG
    lam = float(S["wing"]["planform"]["taper"])
    clb_w = -a_w * math.radians(float(S["wing"]["dihedral_deg"])) * (1 + 2 * lam) / (6 * (1 + lam))
    z_v = fin["z_mac"] - cases[0]["z"]
    clb_fin = -k_v * fin["a"] * 2 * fin["area"] * math.cos(g) ** 2 * z_v / (S_ref * b)
    tv = SL.tail_volumes(S_ref, cbar, b, S_h, st["x_ac"] - x_cg, 2 * fin["area"] * math.cos(g) ** 2, l_v)
    tv["arm_h"], tv["arm_v"] = st["x_ac"] - x_cg, l_v
    return {"x_ac_wing_LL": llm["x_ac"], "x_ac_wing_vlm": x_ac_w, "x_ac_wb": x_ac_wb, "Cm0_wb": Cm0_wb, "Cm0_sections": Cm0_sections,
            "Cm_basic": llm["Cm_basic"], "multhopp": mh, "deps_vlm": deps_vlm, "deps_datcom": deps_datcom, "deps": deps,
            "x_np_classic": x_np_a, "x_np_vlm": x_np_b, "x_np": x_np, "static_margin": sms,
            "sm_min": min(sms.values()), "sm_max": max(sms.values()), "l_h": st["x_ac"] - x_ac_w,
            "V_H": tv["V_H"], "V_V": tv["V_V"], "arm_h_VH": tv["arm_h"], "arm_v_VV": tv["arm_v"],
            "S_h_exposed": S_h, "S_fins_h_proj": S_fin_h, "S_fin_exposed_each": fin["area"],
            "S_fins_v_proj": 2 * fin["area"] * math.cos(g) ** 2, "a_t": st["a"], "a_fin": fin["a"],
            "vlm": {"CL_alpha_all": sol_all["CL_alpha"], "CL_alpha_wing_body": sol_wb["CL_alpha"],
                    "x_np_wing_body": sol_wb["x_np"], "parts": sol_all["parts"], "CL_alpha_wing_only": sol_w["CL_alpha"],
                    "x_ac_wing_only": sol_w["x_np"]},
            "cn_beta": {"fins": cnb_fins, "ventral": cnb_vent, "body_raymer": cnb_body_raymer,
                        "body_datcom": cnb_body_datcom, "body_used": cnb_body, "total": cnb},
            "cl_beta": {"wing_dihedral": clb_w, "fins": clb_fin, "total": clb_w + clb_fin}, "eta_t": eta_t}


# =====================================================================================================================
# 9. flight mechanics and mission (endurance/calc.py sections 10 and 12, per-segment configuration)
# =====================================================================================================================
def cooling_Dq(S: dict, V: float, h: float, ff_ratio: float) -> float:
    """Cooling drag D/q (m2): momentum loss of the cooling air (identity-study method) for a cooling mass flow
    proportional to the fuel flow (heat rejection ~ fuel flow), + external inlet drag."""
    c = S["engine"]["cooling"]
    atm = AL.isa(h)
    m_dot = float(c["airflow_kg_per_s_at_max_power"]) * max(ff_ratio, 0.0)
    ve = math.sqrt(max(float(c["inlet_ram_recovery"]) - float(c["fin_pack_dp_over_q"]), 0.0))
    return m_dot * V * (1.0 - ve) / (0.5 * atm["rho"] * V * V) + float(c["inlet_external_Dq"])


class Flight:
    """Point performance and the design mission. Polars: 'clean' (gear up, turret retracted), 'loiter' (turret
    extended), 'gear_down' (take-off/landing, turret retracted).

    Thrust line inclined by ``eps`` (down-thrust, propeller.thrust_line_inclination_deg): the propeller delivers
    T cos(eps) along the flight path and T sin(eps) downward (carried by extra lift). Cooling drag: the polars hold
    the cooling D/q of the reference (loiter) condition; every point adds the difference to the cooling D/q at its
    own speed, altitude and fuel flow (climb and take-off at full power, cruise at its power)."""

    def __init__(self, S: dict, eng: Engine, prop: Prop, polars: dict, ground: dict | None = None,
                 cool_ref: dict | None = None):
        self.S = S
        self.Sw = float(S["wing"]["area"])
        self.eng = eng
        self.prop = prop
        self.pol = polars
        mis = S["mission"]
        self.v_floor_eas = float(mis["loiter_speed_floor_eas"])
        self.v_cruise_min = float(mis["cruise_speed_band"][0])
        self.h_loiter = float(mis["loiter_altitude"])
        self.R_transit = float(mis["transit_radius"])
        self.reserve_frac = float(mis["reserve_fraction_of_trip_time"])
        self.trapped = float(mis["trapped_fuel_fraction"])
        self.h_reserve = float(mis["reserve_altitude"])
        self.descent_rate = float(mis["descent_rate"])
        self.ground = ground or {}
        self.eps = math.radians(float(S["propeller"].get("thrust_line_inclination_deg", 0.0)))
        self.cool_ref = cool_ref                      # {"Dq": reference cooling D/q in the polars, "k_leak": 1.05}
        self.ff_max = float(S["engine"]["cooling"]["fuel_flow_at_max_power_kg_per_h"]) / 3600.0

    # ------------------------------------------------------------------ helpers
    def cooling_extra_Dq(self, V: float, h: float, ff_kg_s: float) -> float:
        """Cooling D/q at (V, h, fuel flow) minus the reference value held in the polars (x leakage factor)."""
        if not self.cool_ref:
            return 0.0
        return (cooling_Dq(self.S, V, h, ff_kg_s / self.ff_max) - self.cool_ref["Dq"]) * self.cool_ref["k_leak"]

    def wot_ff(self, w: dict) -> float:
        P_tot = w["P_shaft"] + self.eng.p_gen_shaft
        return self.eng.bsfc(P_tot) * P_tot / 3.6e9

    # ------------------------------------------------------------------ points
    def vstall(self, W: float, h: float, clmax: float | None = None) -> float:
        return AL.stall_speed(W, self.Sw, clmax or self.pol["clean"].clmax["clean_trimmed"], AL.isa(h)["rho"])

    def level_point(self, W: float, V: float, h: float, pol: str = "clean") -> dict | None:
        P = self.pol[pol]
        atm = AL.isa(h)
        q = 0.5 * atm["rho"] * V * V
        dDq = 0.0
        p = None
        for _ in range(2):                            # (1) polar + reference cooling, (2) cooling at this power
            CL = W / (q * self.Sw)
            if CL > P.CLs[-1]:
                return None
            D = q * self.Sw * float(P.CD(CL)) + q * dDq
            CL = (W + D * math.tan(self.eps)) / (q * self.Sw)      # down-thrust component carried by lift
            if CL > P.CLs[-1]:
                return None
            CD = float(P.CD(CL)) + dDq / self.Sw
            D = q * self.Sw * CD
            p = self.prop.cruise(D / math.cos(self.eps), V, h)
            if not p["ok"]:
                return None
            P_tot = p["P_shaft"] + self.eng.p_gen_shaft
            ff = self.eng.bsfc(P_tot) * P_tot / 3.6e9
            dDq = self.cooling_extra_Dq(V, h, ff)
        P_tot = p["P_shaft"] + self.eng.p_gen_shaft
        b = self.eng.bsfc(P_tot)
        ff = b * P_tot / 3.6e9
        return {"V": V, "h": h, "CL": CL, "CD": CD, "LD": CL / CD, "D": D, "P_shaft": p["P_shaft"], "P_total": P_tot,
                "eta": p["eta"], "rpm": p["rpm"], "tip_mach": p["tip_mach"], "power_fraction": P_tot / self.eng.P_max,
                "bsfc_g_kWh": b, "ff_kg_s": ff, "ff_kg_h": ff * 3600, "bsfc_eff_kg_J": b / 3.6e9 * P_tot / p["P_shaft"],
                "EAS": V * math.sqrt(atm["sigma"]),
                "gen_W": float(self.S["engine"]["generator"]["power_continuous_W"]) * p["rpm"] / 7500.0}

    def v_floor(self, W: float, h: float, pol: str = "clean") -> float:
        sig = AL.isa(h)["sigma"]
        return max(self.v_floor_eas / math.sqrt(sig), 1.2 * self.vstall(W, h))

    def best_loiter(self, W: float, h: float, pol: str = "loiter", floor: bool = True) -> dict:
        vlo = self.v_floor(W, h) if floor else 1.2 * self.vstall(W, h)
        f = lambda V: (self.level_point(W, V, h, pol) or {"ff_kg_s": 1e9})["ff_kg_s"]   # noqa: E731
        V = max(golden(f, vlo, vlo + 20.0), vlo)
        return self.level_point(W, V, h, pol)

    def best_range(self, W: float, h: float, pol: str = "clean") -> dict:
        vlo = max(1.2 * self.vstall(W, h), self.v_cruise_min)
        f = lambda V: (lambda p: p["ff_kg_s"] / V if p else 1e9)(self.level_point(W, V, h, pol))   # noqa: E731
        V = max(golden(f, vlo, vlo + 30.0), vlo)
        return self.level_point(W, V, h, pol)

    def wot_excess(self, W: float, V: float, h: float, pol: str = "clean") -> dict:
        """Full-throttle point: forward thrust T cos(eps), drag incl. the full-power cooling drag, lift W + T sin(eps)."""
        atm = AL.isa(h)
        q = 0.5 * atm["rho"] * V * V
        w = self.prop.wot(V, h)
        P = self.pol[pol]
        CL = (W + w["T"] * math.sin(self.eps)) / (q * self.Sw)
        D = q * self.Sw * float(P.CD(CL)) + q * self.cooling_extra_Dq(V, h, self.wot_ff(w))
        return {"w": w, "CL": CL, "D": D, "excess": w["T"] * math.cos(self.eps) - D}

    def climb_point(self, W: float, h: float, pol: str = "clean") -> dict:
        best = None
        vs = self.vstall(W, h)
        for V in np.linspace(1.15 * vs, 2.4 * vs, 48):
            e = self.wot_excess(W, V, h, pol)
            w = e["w"]
            roc = e["excess"] * V / W
            if best is None or roc > best["roc"]:
                P_tot = w["P_shaft"] + self.eng.p_gen_shaft
                b = self.eng.bsfc(P_tot)
                best = {"V": V, "roc": roc, "CL": e["CL"], "LD": e["CL"] * q_s(V, h) * self.Sw / e["D"],
                        "eta": w["eta"], "rpm": w["rpm"],
                        "P_shaft": w["P_shaft"], "bsfc_eff_kg_J": b / 3.6e9 * P_tot / w["P_shaft"], "T": w["T"],
                        "tip_mach": w["tip_mach"], "ff_kg_h": b * P_tot / 3.6e6}
        return best

    # ------------------------------------------------------------------ mission
    def fly(self, m0: float, t_loiter_s: float, R_transit: float | None = None, h: float | None = None,
            ferry: bool = False, chunk_loiter_s: float = 3600.0, chunk_cruise_m: float = 50e3,
            loiter_pol: str = "loiter") -> dict:
        R_transit = self.R_transit if R_transit is None else R_transit
        h = self.h_loiter if h is None else h
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
        cp = self.climb_point(W, h / 2)
        add(SZ.Segment("climb", value=h, V=cp["V"], LD=cp["LD"], eta=cp["eta"], bsfc=cp["bsfc_eff_kg_J"],
                       gamma=cp["roc"] / cp["V"], name="climb"), h / cp["roc"], {"V": cp["V"], "roc": cp["roc"]})

        def cruise(dist, name):
            rem = dist
            while rem > 1.0:
                dx = min(chunk_cruise_m, rem)
                p = self.best_range(W, h)
                add(SZ.Segment("cruise", value=dx, V=p["V"], LD=p["LD"], eta=p["eta"], bsfc=p["bsfc_eff_kg_J"],
                               name=name), dx / p["V"], {"V": p["V"], "ff_kg_h": p["ff_kg_h"], "rpm": p["rpm"]})
                rem -= dx
        cruise(R_transit, "transit_out")
        rem = t_loiter_s
        while rem > 1.0:
            dt = min(chunk_loiter_s, rem)
            p = self.best_loiter(W, h, loiter_pol)
            add(SZ.Segment("loiter", value=dt, V=p["V"], LD=p["LD"], eta=p["eta"], bsfc=p["bsfc_eff_kg_J"],
                           name="loiter"), dt, {"V": p["V"], "ff_kg_h": p["ff_kg_h"], "rpm": p["rpm"], "CL": p["CL"],
                                                "pf": p["power_fraction"], "gen_W": p["gen_W"], "EAS": p["EAS"]})
            rem -= dt
        if not ferry:
            cruise(R_transit, "transit_back")
        add(SZ.Segment("descent", name="descent"), h / self.descent_rate)
        add(SZ.Segment("landing", name="landing"), 0.0)
        p = self.best_loiter(W, self.h_reserve, "clean")
        add(SZ.Segment("reserve", value=self.reserve_frac * t_air, V=p["V"], LD=p["LD"], eta=p["eta"],
                       bsfc=p["bsfc_eff_kg_J"], name="reserve"), self.reserve_frac * t_air)
        ff, fr = SZ.mission_fuel_fraction(segs, trapped=self.trapped)
        return {"segments": segs, "log": log, "ff": ff, "fractions": fr, "t_air_s": t_air, "W_end": W}

    def solve_loiter_for_fuel(self, m0: float, fuel_kg: float, **kw) -> dict:
        lo, hi = 0.0, 40 * 3600.0
        base = self.fly(m0, 0.0, **kw)
        if base["ff"] * m0 > fuel_kg:
            return {"feasible": False, **base, "t_loiter_s": 0.0}
        for _ in range(26):
            mid = 0.5 * (lo + hi)
            if self.fly(m0, mid, **kw)["ff"] * m0 > fuel_kg:
                hi = mid
            else:
                lo = mid
        r = self.fly(m0, lo, **kw)
        r["t_loiter_s"] = lo
        r["feasible"] = True
        return r

    def solve_loiter_for_endurance(self, m0: float, endurance_s: float, **kw) -> dict:
        base = self.fly(m0, 0.0, **kw)
        t = max(endurance_s - base["t_air_s"], 0.0)
        r = self.fly(m0, t, **kw)
        r["t_loiter_s"] = t
        return r

    def solve_range(self, m0: float, fuel_kg: float, h: float | None = None) -> dict:
        lo, hi = 0.0, 4000e3
        for _ in range(24):
            mid = 0.5 * (lo + hi)
            r = self.fly(m0, 0.0, R_transit=mid, h=h, ferry=True, chunk_cruise_m=200e3)
            if r["ff"] * m0 > fuel_kg:
                hi = mid
            else:
                lo = mid
        return {"range_m": lo, **self.fly(m0, 0.0, R_transit=lo, h=h, ferry=True, chunk_cruise_m=200e3)}

    # ------------------------------------------------------------------ field performance
    def k_ground(self, pol: str) -> float:
        h_w = float(self.S["wing"]["planform"]["z_root"]) - self.ground["z_g"]
        r = (16 * h_w / float(self.S["wing"]["span"])) ** 2
        return self.pol[pol].fit["k"] * r / (1 + r)

    def takeoff(self, m: float, h: float = 0.0, cg: tuple | None = None) -> dict:
        """Take-off at mass ``m`` and CG ``cg`` = (x, z) with the gear extended (default: ground design case).

        1. Ground run at the ground attitude with the take-off flap, integrated in time (dt 0.02 s): WOT propeller
           thrust T(V) (forward T cos eps, downward T sin eps), drag CD0_TO + k_ground CL_g^2 incl. the full-power
           cooling drag, rolling friction mu (W + T sin eps - L).
        2. Rotation speed V_R: the lowest speed at which the stabilators at their maximum download
           (eta q S_h CLt_max, local coefficient) lift the nose wheel. Moments about the main-wheel contact (inertia
           and drag act at the CG height; friction at the ground): need = W (x_mg - x_cg) + T cos(eps)(z_t - z_cg)
           - T sin(eps)(x_t,thr - x_mg) + mu N h_cg; capacity = q [eta S_h CLt_max (x_ac,t - x_mg)
           + S CL_g (x_mg - x_ac,wb) + S c Cm0_TO].
        3. Rotation for ``t_rot`` (Raymer 6th ed. 17.8.2: about 1 s for small aircraft) while accelerating; lift-off at
           V_LOF = max(speed at the end of the rotation, 1.1 VS_TO).
        4. Checks: main wheels still loaded at V_R (otherwise the main wheels unload first and the aircraft
           wheelbarrows on the nose wheel), power-on trim at V_LOF (local stabilator CL), lift-off attitude.
        5. Airborne distance to 15 m (Raymer 17.8.3)."""
        atm = AL.isa(h)
        rho = atm["rho"]
        W = m * G
        P = self.pol["gear_down"]
        gr = self.ground
        x_cg, z_cg = cg if cg is not None else (gr["x_cg_to"], gr["z_cg_to"])
        fl = P.clmax["flap_to"]
        VS = AL.stall_speed(W, self.Sw, P.clmax["to_trimmed"], rho)
        CLg = gr["CL_ground_to"]
        cd0 = P.fit["cd0"] + fl["dCD0"]
        kg = self.k_ground("gear_down")
        S, c = self.Sw, float(self.S["wing"]["mac"])
        e = self.eps
        h_cg = z_cg - gr["z_g"]
        st = gr["stab"]
        Sh, xt, eta, clt = st["S_h"], st["x_ac"], st["eta"], st["clt_max"]
        Cm0 = gr["Cm0_to"]
        x_ac, x_mg = gr["x_ac_wb"], gr["x_mg"]

        def forces(V):
            w = self.prop.wot(V, h) if V > 0.0 else self.prop.wot(0.0, h)
            q = 0.5 * rho * V * V
            L = q * S * CLg
            D = q * S * (cd0 + kg * CLg ** 2) + q * self.cooling_extra_Dq(max(V, 1.0), h, self.wot_ff(w))
            return w["T"], q, L, D

        V_R = None
        for V in np.arange(2.0, 50.0, 0.02):
            T, q, L, D = forces(V)
            Ft = eta * q * Sh * clt
            N = max(W + T * math.sin(e) - L + Ft, 0.0)
            need = (W * (x_mg - x_cg) + T * math.cos(e) * (gr["z_t"] - z_cg) - T * math.sin(e) * (gr["x_thrust"] - x_mg)
                    + gr["mu"] * N * h_cg)
            cap = q * (eta * Sh * clt * (xt - x_mg) + S * CLg * (x_mg - x_ac) + S * c * Cm0)
            if cap >= need:
                V_R = float(V)
                break
        if V_R is None:
            return {"feasible": False, "ground_roll_m": float("inf"), "V_R_m_s": None, "VS_TO_m_s": VS}
        T, q, L, D = forces(V_R)
        N_main_VR = W + T * math.sin(e) - L + eta * q * Sh * clt
        # time integration of the ground run
        dt, V, s, t, t_r = 0.02, 0.0, 0.0, 0.0, None
        V_lo = 1.1 * VS
        while True:
            T, q, L, D = forces(V)
            a = (T * math.cos(e) - D - gr["mu"] * max(W + T * math.sin(e) - L, 0.0)) / m
            if t_r is None and V >= V_R:
                t_r = t
            if t_r is not None and t - t_r >= gr["t_rot"] and V >= V_lo:
                break
            if a <= 0.0 or t > 120.0:
                return {"feasible": False, "ground_roll_m": float("inf"), "V_R_m_s": V_R, "VS_TO_m_s": VS}
            V += a * dt
            s += V * dt
            t += dt
        V_lof = V
        # power-on trim and attitude at lift-off (airborne: gear loads zero)
        T, q, _, _ = forces(V_lof)
        d_thr = (gr["z_t"] - z_cg) * math.cos(e) - (gr["x_thrust"] - x_cg) * math.sin(e)
        Ft = 0.0
        for _ in range(8):
            L = W + T * math.sin(e) + Ft
            M_wb = q * S * c * Cm0 + L * (x_cg - x_ac) - T * d_thr        # nose-up positive, tail excluded
            Ft = -M_wb / (xt - x_cg)
        L = W + T * math.sin(e) + Ft
        CL_wb = L / (q * S)
        clt_lof = Ft / (eta * q * Sh)
        th_lof = math.degrees((CL_wb - gr["CL0"] - gr["dCL0_to"]) / gr["CLa"])
        # airborne distance to 15 m
        V_tr = max(1.15 * VS, V_lof)
        R = V_tr ** 2 / (0.2 * G)
        ex = self.wot_excess(W, V_tr, h, "gear_down")
        gam = math.asin(max(min((ex["excess"] - q_s(V_tr, h) * S * fl["dCD0"]) / W, 0.5), 0.01))
        h_tr = R * (1 - math.cos(gam))
        s_air = math.sqrt(R ** 2 - (R - 15.0) ** 2) if h_tr >= 15.0 else R * math.sin(gam) + (15.0 - h_tr) / math.tan(gam)
        V_flat = math.sqrt(2 * W / (rho * S * CLg))
        return {"feasible": True, "ground_roll_m": s, "time_s": t, "V_lof_m_s": V_lof, "V_R_m_s": V_R,
                "VS_TO_m_s": VS, "V_flat_ground_attitude_m_s": V_flat, "main_gear_load_at_VR_N": N_main_VR,
                "wheelbarrow_free": bool(N_main_VR > 0.0), "stab_local_cl_at_lof": clt_lof,
                "theta_lof_deg": th_lof, "CL_wb_lof": CL_wb, "T_static_N": self.prop.wot(0.0, h)["T"],
                "T_lof_N": T, "air_distance_15m_m": s_air, "distance_15m_m": s + s_air, "climb_gradient": math.sin(gam),
                "CL_ground": CLg, "flap_deg": fl["delta_deg"], "x_cg": x_cg, "z_cg": z_cg, "mass_kg": m,
                "rotation_time_s": gr["t_rot"]}

    def takeoff_cases(self, m: float, h: float, cases: list) -> dict:
        """Take-off for every loading case of mass ``m`` (gear extended); the governing case is the longest roll."""
        res = [dict(self.takeoff(m, h, (c["x"], c["z"])), case=c["name"]) for c in cases]
        gov = max(res, key=lambda r: r["ground_roll_m"])
        return dict(gov, cases={r["case"]: {k: r.get(k) for k in ("ground_roll_m", "V_R_m_s", "V_lof_m_s",
                                                                 "main_gear_load_at_VR_N", "stab_local_cl_at_lof",
                                                                 "theta_lof_deg", "x_cg")} for r in res})

    def landing(self, m: float, h: float = 0.0) -> dict:
        atm = AL.isa(h)
        W = m * G
        P = self.pol["gear_down"]
        clmax = P.clmax["ld_trimmed"]
        fl = P.clmax["flap_ld"]
        r = AL.landing_roll(W, self.Sw, clmax, P.fit["cd0"] + fl["dCD0"], self.k_ground("gear_down"),
                            mu_brake=float(self.S["mission"]["braking_friction"]), rho=atm["rho"],
                            v_td_factor=float(self.S["mission"]["touchdown_speed_factor"]),
                            cl_roll=self.ground["CL_ground_ld"], t_free=1.0)
        VS = r["VS_ld"]
        Vf = 1.23 * VS
        R = Vf ** 2 / (0.2 * G)
        gam = math.radians(3.0)
        hf = R * (1 - math.cos(gam))
        s_air = (15.0 - hf) / math.tan(gam) + R * math.sin(gam)
        return {"ground_roll_m": r["ground_roll"], "V_td_m_s": r["V_td"], "VS_m_s": VS, "air_distance_15m_m": s_air,
                "distance_15m_m": r["ground_roll"] + s_air, "flap_deg": fl["delta_deg"]}

    def p_avail(self, h):
        """Available power along the flight path at full throttle: T cos(eps) V minus the extra full-power cooling
        drag power (the drag polar holds the loiter cooling drag)."""
        def f(V):
            w = self.prop.wot(V, h)
            return (w["T"] * math.cos(self.eps) - q_s(V, h) * self.cooling_extra_Dq(V, h, self.wot_ff(w))) * V
        return f

    def ceiling(self, m: float, roc_target: float = 0.5) -> float:
        P = self.pol["clean"]
        lo, hi = 0.0, 9000.0
        for _ in range(30):
            mid = 0.5 * (lo + hi)
            roc, _ = AL.rate_of_climb(m * G, self.Sw, P.fit["cd0"], P.fit["k"], self.p_avail(mid), AL.isa(mid)["rho"])
            lo, hi = (mid, hi) if roc >= roc_target else (lo, mid)
        return lo


# =====================================================================================================================
# 10. ground geometry, retractable gear, turret, packaging, loads
# =====================================================================================================================
def flap_effect(S: dict, delta: float) -> dict:
    """Plain flaps (Raymer 6th ed. eqs. 12.21/12.22/12.61, endurance study): dCLmax = 0.9 dClmax (S_flapped/S),
    dClmax 0.9 at 40 deg (linear below); dalpha0 = -15 deg (delta/40) (S_flapped/S); dCD0 = 0.0144 (cf/c)(Sf/S)(d-10)."""
    W = S["wing"]
    fl = W["controls"]["flap"]
    secs = W["sections"]
    ys = np.array([s["y"] for s in secs])
    ch = np.array([s["chord"] for s in secs])
    b2 = float(W["span"]) / 2
    y0, y1 = fl["eta0"] * b2, fl["eta1"] * b2
    yy = np.linspace(y0, y1, 50)
    r = 2 * np.trapz(np.interp(yy, ys, ch), yy) / float(W["area"])
    f = min(max(delta, 0.0) / 40.0, 1.0)
    return {"delta_deg": delta, "S_flapped_over_S": r, "dCLmax": 0.9 * 0.9 * f * r, "dalpha0_deg": -15.0 * f * r,
            "dCD0": 0.0144 * fl["chord_fraction"] * r * max(delta - 10.0, 0.0)}


def clmax_set(S: dict, wa: dict, stab: dict, x_fwd: float, tail: dict) -> dict:
    """Trimmed CLmax at the most forward CG (wing CLmax + tail load; flaps: Cm0 - 0.25 dCL0 shift). The tail
    coefficients ``*_tail_cl`` are LOCAL stabilator coefficients (tail force / (eta_t q S_h)), the quantity the
    stabilator CLmax limits."""
    cbar = float(S["wing"]["mac"])
    arm = tail["x_ac"] - x_fwd
    eta = float(S["aero"]["stability_rules"]["eta_tail"])

    def trim(clw, dcm):
        cm = stab["Cm0_wb"] + dcm + clw * (x_fwd - stab["x_ac_wb"]) / cbar
        clt = cm * cbar / arm
        return clw + clt, clt * float(S["wing"]["area"]) / (tail["S_exposed"] * eta)
    fto = flap_effect(S, float(S["wing"]["controls"]["flap"]["takeoff_deg"]))
    fld = flap_effect(S, float(S["wing"]["controls"]["flap"]["landing_deg"]))
    out = {"wing_clean": wa["CLmax"], "flap_to": fto, "flap_ld": fld}
    out["clean_trimmed"], out["clean_tail_cl"] = trim(wa["CLmax"], 0.0)
    d0 = wa["CL_alpha"] * math.radians(-fto["dalpha0_deg"])
    out["to_trimmed"], out["to_tail_cl"] = trim(wa["CLmax"] + fto["dCLmax"], -0.25 * d0)
    d1 = wa["CL_alpha"] * math.radians(-fld["dalpha0_deg"])
    out["ld_trimmed"], out["ld_tail_cl"] = trim(wa["CLmax"] + fld["dCLmax"], -0.25 * d1)
    out["dCL0_to"], out["dCL0_ld"] = d0, d1
    return out


def thrust_arm(S: dict, x_cg: float, z_cg: float) -> float:
    """Moment arm (m) of the (down-inclined) thrust line about the CG; positive = thrust passes above the CG
    (nose-down moment for positive thrust)."""
    e = math.radians(float(S["propeller"].get("thrust_line_inclination_deg", 0.0)))
    x_t, _, z_t = S["propeller"]["hub"]
    return (z_t - z_cg) * math.cos(e) - (x_t - x_cg) * math.sin(e)


def power_on_trim(S: dict, wa: dict, stab: dict, tail: dict, clm: dict, prop: "Prop", cases: list) -> dict:
    """Local stabilator CL needed to trim with full throttle (thrust-line moment) at the forward CGs:
    * lift-off: MTOM loading cases, take-off flap, V = 1.1 VS_TO (the lowest lift-off speed, most critical);
    * go-around (balked landing): every loading case, flaps at the landing setting, V = 1.2 VS (landing CLmax)."""
    S_ref, cbar = float(S["wing"]["area"]), float(S["wing"]["mac"])
    eta = float(S["aero"]["stability_rules"]["eta_tail"])
    e = math.radians(float(S["propeller"].get("thrust_line_inclination_deg", 0.0)))
    Sh, xt = tail["S_exposed"], tail["x_ac"]
    m0 = float(S["mass"]["mtow_kg"])

    def trim(m, x_cg, z_cg, V, Cm0):
        q = 0.5 * RHO0 * V * V
        W = m * G
        T = prop.wot(V, 0.0)["T"]
        d = thrust_arm(S, x_cg, z_cg)
        Ft = 0.0
        for _ in range(8):
            L = W + T * math.sin(e) + Ft
            M_wb = q * S_ref * cbar * Cm0 + L * (x_cg - stab["x_ac_wb"]) - T * d
            Ft = -M_wb / (xt - x_cg)
        return {"case": None, "V": V, "T": T, "tail_cl_local": Ft / (eta * q * Sh), "thrust_arm_m": d,
                "x_cg": x_cg, "CL_wb": (W + T * math.sin(e) + Ft) / (q * S_ref)}
    out = {"liftoff": [], "go_around": []}
    m_top = max([c["m"] for c in cases] + [0.0])
    for c in cases:
        if abs(c["m"] - m_top) < 0.5 or abs(c["m"] - m0) < 0.5:
            V = 1.1 * AL.stall_speed(c["m"] * G, S_ref, clm["to_trimmed"])
            out["liftoff"].append(dict(trim(c["m"], c["x"], c["z"], V, stab["Cm0_wb"] - 0.25 * clm["dCL0_to"]),
                                       case=c["name"]))
        V = 1.2 * AL.stall_speed(c["m"] * G, S_ref, clm["ld_trimmed"])
        out["go_around"].append(dict(trim(c["m"], c["x"], c["z"], V, stab["Cm0_wb"] - 0.25 * clm["dCL0_ld"]),
                                     case=c["name"]))
    worst = max(out["liftoff"] + out["go_around"], key=lambda r: abs(r["tail_cl_local"]))
    out["max_abs_tail_cl_local"] = abs(worst["tail_cl_local"])
    out["governing"] = worst
    return out


def _rot(P, c, axis: str, ang: float) -> np.ndarray:
    """Rotate points P (n, 3) by ``ang`` (rad, right-handed) about the axis through ``c`` parallel to x or y."""
    c = np.asarray(c, float)
    P = np.atleast_2d(np.asarray(P, float)) - c
    ca, sa = math.cos(ang), math.sin(ang)
    if axis == "x":
        Rm = np.array([[1.0, 0.0, 0.0], [0.0, ca, -sa], [0.0, sa, ca]])
    else:
        Rm = np.array([[ca, 0.0, sa], [0.0, 1.0, 0.0], [-sa, 0.0, ca]])
    return P @ Rm.T + c


def tyre_points(center, axle, r: float, w: float, n: int = 36, m: int = 8) -> np.ndarray:
    """Surface samples of a tyre envelope: torus with outer radius ``r`` and tube diameter ``w`` about ``axle``."""
    a = np.asarray(axle, float) / np.linalg.norm(axle)
    u = np.cross(a, [0.0, 0.0, 1.0]) if abs(a[2]) < 0.9 else np.cross(a, [1.0, 0.0, 0.0])
    u /= np.linalg.norm(u)
    v = np.cross(a, u)
    R0, rt = r - w / 2, w / 2
    th = np.linspace(0.0, 2 * math.pi, n, endpoint=False)
    ph = np.linspace(0.0, 2 * math.pi, m, endpoint=False)
    E = np.cos(th)[:, None] * u + np.sin(th)[:, None] * v                       # (n, 3) radial directions
    pts = (np.asarray(center, float)[None, None, :] + (R0 + rt * np.cos(ph))[None, :, None] * E[:, None, :] +
           (rt * np.sin(ph))[None, :, None] * a[None, None, :])
    return pts.reshape(-1, 3)


def main_gear_layout(S: dict, af: "Airframe", x_mg: float, z_g: float, y_mg_req: float) -> dict:
    """Main-gear kinematics (starboard leg; port mirrored). Trunnion axis along x in the body gear frame, leg splayed
    outward by ``splay_deg``, axle horizontal. Inward retraction by 90 deg + splay about the trunnion axis lays the leg
    horizontal under the wing box with the tyre tilted by the splay angle; the two wells sit side by side with
    ``well_gap`` at the centre line. The trunnion height follows from the track that the turnover rule needs
    (y_axle >= ``y_mg_req``); the stowage (tyre envelope + clearance, leg, trunnion fitting) is then verified
    against the body OML."""
    LG = S["landing_gear"]
    mg, R = LG["main"], LG["rules"]
    r_t, w = LG["tyre"]["diameter"] / 2, LG["tyre"]["width"]
    defl = LG["tyre"]["static_deflection"]
    clr, skin = float(R["well_clearance"]), float(R["well_skin_margin"])
    sp = math.radians(mg["splay_deg"])
    z_ax = z_g + r_t - defl
    half_y = r_t * math.cos(sp) + 0.5 * w * math.sin(sp)               # stowed tyre half extent in y
    gap = 0.5 * float(mg["well_gap"])
    L = max((y_mg_req - half_y - clr - gap) / (1.0 + math.sin(sp)), float(mg["leg_length_min"]))
    z_t = z_ax + L * math.cos(sp)
    y_t = max(float(mg["trunnion_y_min"]), L + half_y + clr + gap)
    return _main_gear_geom(S, af, np.array([x_mg, y_t, z_t]), L, sp, z_ax)


def _main_gear_geom(S: dict, af: "Airframe", T: np.ndarray, L: float, sp: float, z_ax: float) -> dict:
    LG = S["landing_gear"]
    mg, R = LG["main"], LG["rules"]
    r_t, w = LG["tyre"]["diameter"] / 2, LG["tyre"]["width"]
    clr, skin = float(R["well_clearance"]), float(R["well_skin_margin"])
    A = T + L * np.array([0.0, math.sin(sp), -math.cos(sp)])
    ang = -(math.pi / 2 + sp)
    A_s = _rot(A, T, "x", ang)[0]
    ax_s = _rot(np.array([0.0, 1.0, 0.0]), np.zeros(3), "x", ang)[0]
    tyre = tyre_points(A_s, ax_s, r_t + clr, w + 2 * clr)
    leg = np.array([T + f * (A_s - T) for f in np.linspace(0.0, 0.8, 12)])
    f = float(R["trunnion_fitting_half_size"])
    fit = np.array([T + np.array([dx, dy, dz]) for dx in (-f, f) for dy in (-f, f) for dz in (-f, f)])
    ok_t = af.inside(tyre, margin=skin)
    ok_l = af.inside(leg, margin=skin + 0.5 * float(mg["leg_frontal_width"]))
    ok_f = af.inside(fit, margin=skin)
    lo, hi = np.minimum(tyre.min(axis=0), fit.min(axis=0)), np.maximum(tyre.max(axis=0), fit.max(axis=0))
    return {"trunnion": T, "axle": A, "leg_length": L, "splay": sp, "stowed_center": A_s, "stowed_axle": ax_s,
            "retraction_deg": -math.degrees(ang), "tyre_inside": bool(np.all(ok_t)), "leg_inside": bool(np.all(ok_l)),
            "fitting_inside": bool(np.all(ok_f)), "n_tyre_out": int(np.sum(~ok_t)),
            "well_box": [lo.tolist(), hi.tolist()], "centre_gap": float(tyre[:, 1].min()) * 2.0,
            "tyre_top_z": float(tyre[:, 2].max()), "tyre_bottom_z": float(tyre[:, 2].min()), "z_axle": z_ax}


def nose_gear_layout(S: dict, af: "Airframe", x_ng: float, z_axn: float) -> dict:
    """Nose leg: pivot ahead of the axle by the trail, leg retracting aft by 90 deg about a lateral axis into a keel
    slot well (tyre vertical). The pivot is as low as the stowed tyre envelope (+ clearance) allows inside the
    chined keel with the skin margin (shortest, stiffest leg)."""
    LG = S["landing_gear"]
    ng, R = LG["nose"], LG["rules"]
    r_t, w = LG["tyre"]["diameter"] / 2, LG["tyre"]["width"]
    clr, skin = float(R["well_clearance"]), float(R["well_skin_margin"])
    x_p = x_ng - float(ng["trail"])
    best = None
    for z_p in np.arange(z_axn + float(ng["leg_length_min"]), 0.12, 0.0025):
        P = np.array([x_p, 0.0, z_p])
        A = np.array([x_ng, 0.0, z_axn])
        A_s = _rot(A, P, "y", -math.pi / 2)[0]
        tyre = tyre_points(A_s, [0.0, 1.0, 0.0], r_t + clr, w + 2 * clr)
        leg = np.array([P + f * (A_s - P) for f in np.linspace(0.0, 0.8, 10)])
        ok = af.inside(tyre, margin=skin)
        ok_l = af.inside(leg, margin=skin + 0.5 * float(ng["leg_frontal_width"]))
        if np.all(ok) and np.all(ok_l):
            best = (z_p, P, A, A_s, tyre)
            break
    if best is None:
        z_p = z_axn + float(ng["leg_length_min"])
        P = np.array([x_p, 0.0, z_p])
        A = np.array([x_ng, 0.0, z_axn])
        A_s = _rot(A, P, "y", -math.pi / 2)[0]
        tyre = tyre_points(A_s, [0.0, 1.0, 0.0], r_t + clr, w + 2 * clr)
        best = (z_p, P, A, A_s, tyre)
        ok_all = False
    else:
        ok_all = True
    z_p, P, A, A_s, tyre = best
    lo = np.minimum(tyre.min(axis=0), P - 0.03)
    hi = np.maximum(tyre.max(axis=0), P + 0.03)
    return {"pivot": P, "axle": A, "stowed_center": A_s, "leg_length": float(np.linalg.norm(A - P)),
            "retraction_deg": 90.0, "inside": ok_all, "well_box": [lo.tolist(), [hi[0], hi[1], hi[2]]],
            "tyre_top_z": float(tyre[:, 2].max())}


def gear_stowage(S: dict, af: "Airframe") -> dict:
    """Stowage checks of the spec landing-gear geometry: tyre envelopes, legs and fittings inside the OML with the skin
    margin, centre-line gap between the two main wells, main wells clear of the wing carry-through box."""
    LG = S["landing_gear"]
    mg, ng = LG["main"], LG["nose"]
    T = np.asarray(mg["trunnion"], float)
    A = np.asarray(mg["axle_static"], float)
    L = float(np.linalg.norm(A - T))
    sp = math.atan2(A[1] - T[1], T[2] - A[2])
    m = _main_gear_geom(S, af, T, L, sp, float(A[2]))
    zb = S["layout"]["zones_preliminary"]["wing_carry_through"]["box"]
    box_clear = float(zb[0][2]) - m["tyre_top_z"]
    P = np.asarray(ng["pivot"], float)
    An = np.asarray(ng["axle_static"], float)
    A_s = _rot(An, P, "y", -math.pi / 2)[0]
    r_t, w = LG["tyre"]["diameter"] / 2, LG["tyre"]["width"]
    clr, skin = float(LG["rules"]["well_clearance"]), float(LG["rules"]["well_skin_margin"])
    tyre = tyre_points(A_s, [0.0, 1.0, 0.0], r_t + clr, w + 2 * clr)
    leg = np.array([P + f * (A_s - P) for f in np.linspace(0.0, 0.8, 10)])
    n_ok = bool(np.all(af.inside(tyre, margin=skin)) and
                np.all(af.inside(leg, margin=skin + 0.5 * float(ng["leg_frontal_width"]))))
    return {"main": {"tyre_inside": m["tyre_inside"], "leg_inside": m["leg_inside"],
                     "fitting_inside": m["fitting_inside"], "centre_gap_m": m["centre_gap"],
                     "clear_of_wing_box_m": box_clear, "well_box": m["well_box"],
                     "ok": bool(m["tyre_inside"] and m["leg_inside"] and m["fitting_inside"] and
                                m["centre_gap"] >= float(mg["well_gap"]) - 1e-6 and box_clear >= 0.0)},
            "nose": {"inside": n_ok, "stowed_center": A_s.tolist(), "ok": n_ok}}


def turret_geometry(S: dict, af: "Airframe") -> dict:
    """EO/IR turret bay: retracted ball centre from the keel (door + clearance + growth radius), growth envelope =
    sphere of the growth diameter + cylinder of the same diameter up to the growth height, lift mechanism above."""
    T = S["payload"]["turret"]
    xc = float(S["layout"]["rules"]["turret_x"])
    g = T["growth_envelope"]
    zk = float(af.z_bot(xc))
    z_ret = zk + float(T["bay"]["door_thickness"]) + float(T["bay"]["door_clearance"]) + float(g["diameter"]) / 2
    return {"x": xc, "z_keel": zk, "ball_center_retracted_z": z_ret,
            "ball_center_extended_z": z_ret - float(T["stroke"]),
            "envelope_top_z": z_ret + float(g["height"]) - float(g["diameter"]) / 2,
            "mechanism_top_z": z_ret + float(g["height"]) - float(g["diameter"]) / 2 + float(T["bay"]["mechanism_height"])}


def _box_overlap(a, b, ya_sym: bool = True, yb_sym: bool = True) -> float:
    """Overlap volume (m3) of two zone boxes (symmetric boxes span -y1..y1)."""
    a, b = np.asarray(a, float), np.asarray(b, float)
    ay = (-a[1, 1], a[1, 1]) if ya_sym else (a[0, 1], a[1, 1])
    by = (-b[1, 1], b[1, 1]) if yb_sym else (b[0, 1], b[1, 1])
    dx = min(a[1, 0], b[1, 0]) - max(a[0, 0], b[0, 0])
    dy = min(ay[1], by[1]) - max(ay[0], by[0])
    dz = min(a[1, 2], b[1, 2]) - max(a[0, 2], b[0, 2])
    return max(dx, 0.0) * max(dy, 0.0) * max(dz, 0.0)


def turret_checks_light(S: dict, af: Airframe) -> bool:
    """Growth envelope inside + HD59 ball above the doors (no field-of-regard ray casting)."""
    P = S["payload"]["turret"]
    xc = float(P["bay_center_x"])
    z_ret = float(P["ball_center_retracted_z"])
    g = P["growth_envelope"]
    r_g = float(g["diameter"]) / 2
    zb = af.z_bot(xc) + float(P["bay"]["door_thickness"])
    pts = []
    for th in np.linspace(0, 2 * math.pi, 24, endpoint=False):
        for ph in np.linspace(-math.pi / 2, 0.0, 7):
            pts.append([xc + r_g * math.cos(ph) * math.cos(th), r_g * math.cos(ph) * math.sin(th),
                        z_ret + r_g * math.sin(ph)])
        for zz in np.linspace(z_ret, z_ret + float(g["height"]) - r_g + float(P["bay"]["mechanism_height"]), 6):
            pts.append([xc + r_g * math.cos(th), r_g * math.sin(th), zz])
    ok = np.all(af.inside(np.array(pts), margin=float(P["bay"]["wall_margin"])))
    return bool(ok and (z_ret - r_g) >= zb - 1e-9)



def liftoff_attitude(wa: dict, clm: dict, pot: dict | None) -> float:
    """Body attitude at lift-off (deg): the larger of the untrimmed 1.1 VS_TO value and the power-on trimmed value
    (wing-body CL incl. the stabilator download at 1.1 VS_TO, the lowest lift-off speed) of the MTOM cases."""
    th = math.degrees((clm["to_trimmed"] / 1.1 ** 2 - wa["CL_0"] - clm["dCL0_to"]) / wa["CL_alpha"])
    for r in (pot or {}).get("liftoff", []):
        th = max(th, math.degrees((r["CL_wb"] - wa["CL_0"] - clm["dCL0_to"]) / wa["CL_alpha"]))
    return th


def ground_geometry(S: dict, af: Airframe, wa: dict, clm: dict, cases_ground: list, prop_D: float,
                    pot: dict | None = None) -> dict:
    """Static ground line, tip-back/turnover angles, load split, propeller ground clearances and attitudes.

    Propeller ground clearance (CS-VLA 925(a), standards.yaml#propeller_clearance): the most critical of the static
    attitude and the normal take-off (lift-off) attitude with the gear statically deflected, and the touch-down
    attitude with the gear unloaded (struts and tyres extended at first contact); flat main tyre + bottomed strut at
    the level take-off attitude (positive clearance). CS-VLA 925(b) (aft-mounted propeller): the tail bumper touches
    before the propeller (prop-strike vs bumper-contact angles)."""
    LG = S["landing_gear"]
    mg, ng = LG["main"], LG["nose"]
    r_t = LG["tyre"]["diameter"] / 2
    defl = LG["tyre"]["static_deflection"]
    x_mg, y_mg, z_ax = mg["axle_static"]
    x_ng, _, z_axn = ng["axle_static"]
    z_g = z_ax - (r_t - defl)
    # static attitude (+ nose up): with the nose wheel contact BELOW the main-wheel contact in body axes the
    # aeroplane sits nose up (world z = -x sin(th) + z cos(th) equal at both contacts)
    th_s = math.degrees(math.atan2(z_g - (z_axn - (r_t - defl)), x_mg - x_ng))
    pr = S["propeller"]
    Z_T, x_p = float(pr["hub"][2]), float(pr["plane_x"])
    R = prop_D / 2
    xs = [c["x"] for c in cases_ground]
    zs = [c["z"] for c in cases_ground]
    x_aft, x_fwd = max(xs), min(xs)
    z_cg = max(zs)
    h_cg = z_cg - z_g
    tipback = math.degrees(math.atan2(x_mg - x_aft, h_cg))
    wb = x_mg - x_ng
    nose_aft = (x_mg - x_aft) / wb
    nose_fwd = (x_mg - x_fwd) / wb
    ln = x_fwd - x_ng
    delta = math.atan(y_mg / wb)
    turnover = math.degrees(math.atan(h_cg / (ln * math.sin(delta))))
    clr_level = (Z_T - R - z_g)
    d_ext = float(mg["static_compression"]) + defl              # main gear unloaded at first contact

    def clear_at(th, dz=0.0):
        return (Z_T - R - z_g + dz) * math.cos(math.radians(th)) - (x_p - x_mg) * math.sin(math.radians(th))
    # attitudes (body angle): lift-off (take-off flap, power-on trimmed at 1.1 VS_TO), touch-down (k_td VS, landing flap)
    CLa = wa["CL_alpha"]
    k_td = float(S["mission"]["touchdown_speed_factor"])
    th_lof = liftoff_attitude(wa, clm, pot)
    th_td = math.degrees((clm["ld_trimmed"] / k_td ** 2 - wa["CL_0"] - clm["dCL0_ld"]) / CLa)
    th_flare = th_td + float(S["landing_gear"]["rules"]["flare_margin_deg"])
    clr_static = clear_at(th_s)
    clr_lof = clear_at(max(th_lof, th_s))
    clr_td = clear_at(max(th_td, th_s), d_ext)
    dz_main = (mg["stroke"] - mg["static_compression"]) + (r_t - LG["tyre"]["rim_radius"])
    th_flat = th_s + math.degrees(math.atan2(dz_main, wb))
    clr_flat = clear_at(th_flat) - dz_main
    th_prop = math.degrees(math.atan2(Z_T - R - z_g, x_p - x_mg))
    vb = S["tail"]["surfaces"]["ventral"]
    bump = vb["bumper"]
    x_b, z_b = bump["contact_point"][0], bump["contact_point"][2]
    th_bump = math.degrees(math.atan2(z_b - z_g, x_b - x_mg))
    stow = gear_stowage(S, af)
    return {"z_g": z_g, "x_mg": x_mg, "y_mg": y_mg, "x_ng": x_ng, "track": 2 * y_mg, "wheelbase": wb, "h_cg": h_cg,
            "static_attitude_deg": th_s, "tipback_deg": tipback, "turnover_deg": turnover,
            "nose_load_aft_cg": nose_aft, "nose_load_fwd_cg": nose_fwd, "prop_clear_level": clr_level,
            "prop_clear_static": clr_static, "prop_clear_liftoff": clr_lof, "prop_clear_touchdown_unloaded": clr_td,
            "prop_clear_min_925a": min(clr_static, clr_lof, clr_td),
            "prop_clear_flat_tyre_bottomed": clr_flat, "prop_strike_deg": th_prop,
            "bumper_contact_deg": th_bump, "bumper_point": [x_b, z_b], "theta_lof_deg": th_lof, "theta_td_deg": th_td,
            "theta_flare_deg": th_flare, "x_cg_aft": x_aft, "x_cg_fwd": x_fwd, "z_cg_max": z_cg, "stowed": stow,
            "gear_unloaded_extension_m": d_ext,
            "CL_ground_to": wa["CL_0"] + clm["dCL0_to"] + CLa * math.radians(th_s),
            "CL_ground_ld": wa["CL_0"] + clm["dCL0_ld"] + CLa * math.radians(th_s)}


def ray_tri_hit(O: np.ndarray, D: np.ndarray, tri: np.ndarray, t_max: float) -> np.ndarray:
    """Moller-Trumbore: True for every ray (origin O[i], unit direction D[i]) that hits any triangle (m, 3, 3) at a
    distance 0 < t < t_max."""
    O = np.atleast_2d(O)
    D = np.atleast_2d(D)
    v0, e1, e2 = tri[:, 0], tri[:, 1] - tri[:, 0], tri[:, 2] - tri[:, 0]
    out = np.zeros(len(O), bool)
    for i in range(len(O)):
        p = np.cross(D[i], e2)
        det = np.einsum("ij,ij->i", e1, p)
        ok = np.abs(det) > 1e-12
        inv = np.where(ok, 1.0 / np.where(ok, det, 1.0), 0.0)
        tv = O[i] - v0
        u = np.einsum("ij,ij->i", tv, p) * inv
        q = np.cross(tv, e1)
        v = (q @ D[i]) * inv
        t = np.einsum("ij,ij->i", e2, q) * inv
        out[i] = bool(np.any(ok & (u >= 0) & (v >= 0) & (u + v <= 1) & (t > 1e-6) & (t < t_max)))
    return out


def structure_triangles(af: Airframe, include_wing: bool = False) -> np.ndarray:
    """Triangles of the tail surfaces (both sides) and optionally the wing: obstacles for ray tests."""
    tris = []
    for k, m in af.tail_meshes.items():
        tris.append(m.V[m.F])
        if af.tail_mirror[k]:
            mm = m.mirrored_y()
            tris.append(mm.V[mm.F])
    if include_wing:
        tris += [af.wing_mesh.V[af.wing_mesh.F], af.wing_mesh.mirrored_y().V[af.wing_mesh.mirrored_y().F]]
    return np.vstack(tris)


def turret_checks(S: dict, af: Airframe, fov: bool = True) -> dict:
    """Retracted: HD59 ball above the closed doors, E180 growth envelope (sphere + cylinder) and the lift mechanism
    inside the body; extended: field of regard (highest unobstructed elevation per azimuth). Rays leave the sensor
    window on the ball surface: its centre and four points of the aperture rim (radius
    payload.turret.aperture_radius) are traced along the line of sight; a direction counts as clear only if every ray
    clears the body OML (the open bay cavity is not solid; its walls are), the tail surfaces incl. the ventral fin and
    bumper, the propeller disc and the retracted gear (inside the OML). The bay doors fold inward along the bay walls
    (no obstruction below the skin)."""
    P = S["payload"]["turret"]
    xc = float(P["bay_center_x"])
    z_ret = float(P["ball_center_retracted_z"])
    z_ext = float(P["ball_center_extended_z"])
    r_b = float(P["ball_diameter"]) / 2
    g = P["growth_envelope"]
    r_g = float(g["diameter"]) / 2
    zb_skin = af.z_bot(xc)
    door_t = float(P["bay"]["door_thickness"])
    flush_margin = (z_ret - r_b) - (zb_skin + door_t)              # > 0: HD59 ball above the door inner face
    growth_margin = (z_ret - r_g) - (zb_skin + door_t)
    pts = []
    for th in np.linspace(0, 2 * math.pi, 24, endpoint=False):
        for ph in np.linspace(-math.pi / 2, 0.0, 7):
            pts.append([xc + r_g * math.cos(ph) * math.cos(th), r_g * math.cos(ph) * math.sin(th),
                        z_ret + r_g * math.sin(ph)])
        top = z_ret + float(g["height"]) - r_g
        for zz in np.linspace(z_ret, top + float(P["bay"]["mechanism_height"]), 6):
            pts.append([xc + r_g * math.cos(th), r_g * math.sin(th), zz])
    pts = np.array(pts)
    ins = af.inside(pts, margin=float(P["bay"]["wall_margin"]))
    growth_inside = bool(np.all(ins[: len(pts)]))
    ext_below = zb_skin - (z_ext + r_b)
    # bay cavity (open when the turret is extended): not solid
    hw = r_g + 0.01
    cav = (xc - hw, xc + hw, hw, zb_skin - 0.01, z_ret + float(g["height"]) - r_g + float(P["bay"]["mechanism_height"]))

    def blocked_body(Q):
        inc = (Q[:, 0] > cav[0]) & (Q[:, 0] < cav[1]) & (np.abs(Q[:, 1]) < cav[2]) & (Q[:, 2] > cav[3]) & \
            (Q[:, 2] < cav[4])
        return af.inside(Q) & ~inc
    tri = structure_triangles(af) if fov else np.zeros((0, 3, 3))
    pr = S["propeller"]
    x_p, z_h, R_p = float(pr["plane_x"]), float(pr["hub"][2]), 0.5 * float(pr["diameter"])
    r_ap = float(P["aperture_radius"])
    c = np.array([xc, 0.0, z_ext])
    s = np.linspace(0.002, 4.0, 200)

    def clear(d):
        u = np.cross(d, [0.0, 0.0, 1.0]) if abs(d[2]) < 0.99 else np.array([0.0, 1.0, 0.0])
        u /= np.linalg.norm(u)
        v = np.cross(d, u)
        O = np.array([c + r_b * d + r_ap * k for k in (np.zeros(3), u, -u, v, -v)])
        for o in O:
            if np.any(blocked_body(o + s[:, None] * d)):
                return False
        if len(tri) and np.any(ray_tri_hit(O, np.repeat(d[None], len(O), 0), tri, 4.0)):
            return False
        if abs(d[0]) > 1e-9:                                   # propeller disc (plane x = x_p)
            t = (x_p - O[:, 0]) / d[0]
            hit = O + t[:, None] * d
            if np.any((t > 0) & (t < 4.0) & (np.hypot(hit[:, 1], hit[:, 2] - z_h) <= R_p)):
                return False
        return True
    az = np.radians(np.arange(0, 360, 10.0)) if fov else np.zeros(0)
    els = np.radians(np.arange(-90.0, 30.01, 1.0))
    lim = []
    for a_ in az:
        best = -90.0
        for e in els:
            d = np.array([-math.cos(e) * math.cos(a_), math.cos(e) * math.sin(a_), math.sin(e)])   # az 0 = forward
            if not clear(d):
                break
            best = math.degrees(e)
        lim.append(best)
    lim = np.array(lim) if len(lim) else np.array([float("nan")])
    fwd = lim[(np.degrees(az) <= 60) | (np.degrees(az) >= 300)] if len(az) else lim
    return {"bay_center_x": xc, "skin_z_at_bay": zb_skin, "flush_margin_m": flush_margin,
            "retracted_ball_above_door": flush_margin >= 0.0, "growth_margin_to_door_m": growth_margin,
            "growth_envelope_inside_retracted": bool(growth_inside and growth_margin >= -1e-9),
            "extended_ball_top_below_skin_m": ext_below, "stroke_m": z_ret - z_ext,
            "aperture_radius_m": r_ap,
            "fov_upper_limit_deg_by_azimuth": dict(zip([int(round(math.degrees(a_))) for a_ in az], lim.tolist())),
            "fov_upper_min_deg": float(lim.min()), "fov_upper_forward_sector_min_deg": float(fwd.min()),
            "fov_upper_max_deg": float(lim.max()), "nadir_clear": bool(lim.min() > -89.0)}


def blade_axial_half_extent(S: dict, r: np.ndarray) -> np.ndarray:
    """Axial half extent (m) of the propeller blade at radius r: 0.5 c sin(beta) + 0.5 t + tolerance from the blade
    planform model in propeller.blade_model (estimate; the Mejzlik drawing replaces it in the detail phase)."""
    bm = S["propeller"]["blade_model"]
    R = 0.5 * float(S["propeller"]["diameter"])
    r0 = float(bm["r_root"])
    f = np.clip((np.asarray(r, float) - r0) / (R - r0), 0.0, 1.0)
    c = float(bm["chord_root"]) + (float(bm["chord_tip"]) - float(bm["chord_root"])) * f
    beta = np.radians(float(bm["beta_root_deg"]) + (float(bm["beta_tip_deg"]) - float(bm["beta_root_deg"])) * f)
    return 0.5 * c * np.sin(beta) + 0.5 * float(bm["thickness_ratio"]) * c + float(bm["tolerance"])


def prop_clearances(S: dict, af: Airframe) -> dict:
    """CS-VLA 925(c) structural clearances of the pusher propeller against the body (cowl), the tail surfaces
    (fins, stabilators, stubs, ventral fin): (1) radial: structure within the axial band of the blade tips must lie at
    least 0.026 m outside the tip circle; (2) longitudinal: structure inside the disc radius must be at least 0.013 m
    ahead of / behind the blade swept volume (blade axial extent from propeller.blade_model). Disc axis inclined by the
    thrust-line angle. Also the propeller-guard geometry of the canted fins (trailing-edge crossing radius)."""
    pr = S["propeller"]
    H = np.array(pr["hub"], float)
    e = math.radians(float(pr.get("thrust_line_inclination_deg", 0.0)))
    n = np.array([math.cos(e), 0.0, math.sin(e)])
    R = 0.5 * float(pr["diameter"])
    pts = [af.body_mesh.V]
    for k, m in af.tail_meshes.items():
        pts.append(m.V)
        if af.tail_mirror[k]:
            pts.append(m.mirrored_y().V)
    names = ["body"] + [k for k, m in af.tail_meshes.items() for _ in range(2 if af.tail_mirror[k] else 1)]
    rows = {}
    rad_min, lon_min = float("inf"), float("inf")
    for nm, Pp in zip(names, pts):
        v = Pp - H
        a = v @ n
        r = np.linalg.norm(v - a[:, None] * n[None, :], axis=1)
        ex = blade_axial_half_extent(S, np.minimum(r, R))
        # longitudinal: points inside the tip circle (blade roots from the spinner outward)
        mi = (r <= R) & (r >= float(pr["blade_model"]["r_root"]) - 0.01)
        lon = float(np.min(np.abs(a[mi]) - ex[mi])) if np.any(mi) else float("inf")
        # radial: points in the axial band of the tips (+ the longitudinal minimum), outside the tip circle
        e_tip = float(blade_axial_half_extent(S, np.array([R]))[0])
        mr = (np.abs(a) <= e_tip + float(pr["clearances"]["longitudinal_blade_to_structure_min"])) & (r >= R - 1e-6)
        rad = float(np.min(r[mr] - R)) if np.any(mr) else float("inf")
        prev = rows.get(nm, {"radial_m": float("inf"), "longitudinal_m": float("inf")})
        rows[nm] = {"radial_m": min(prev["radial_m"], rad), "longitudinal_m": min(prev["longitudinal_m"], lon)}
        rad_min, lon_min = min(rad_min, rad), min(lon_min, lon)
    L = S["fuselage"]["lines"]
    x_end = float(L["x_hub"]) + float(L["l_cowl_tail"])
    spinner_gap = (H[0] - float(pr["spinner"]["backplate_ahead_of_plane"])) - x_end
    fp = S["tail"]["surfaces"]["fin"]["params"]
    return {"radial_min_m": rad_min, "longitudinal_min_m": lon_min, "by_part": rows,
            "spinner_to_cowl_gap_m": spinner_gap, "fin_te_crossing_radius_m": float(fp["guard_radius"]),
            "fin_te_crossing_span_fraction": float(fp["te_crossing_span_fraction"]),
            "fin_guard_margin_over_tip_m": float(fp["guard_radius"]) - R,
            "blade_tip_axial_half_extent_m": float(blade_axial_half_extent(S, np.array([R]))[0])}


def tail_root_checks(S: dict, af: Airframe, tp: dict | None = None) -> dict:
    """Stabilator root (F3): gap between the moving root rib and the fixed stub/body along the root chord and over
    the deflection range (body chine half width swept by the root airfoil <= y_root - gap); fin roots (F4): gap
    between the fin root faces and the dorsal skin along the whole root chord (<= 0: buried)."""
    TS = S["tail"]["surfaces"]
    hp = TS["stabilator"]["params"]
    st = TS["stabilator"]
    piv = np.array(st["pivot"], float)
    rng_ = st["controls"]["range_deg"]
    r0 = st["sections"][0]
    x, yu, yl = oml.resampled(r0["airfoil"], 81, 0.0, 1.0)
    worst_body = -float("inf")
    for d in np.linspace(rng_[0], rng_[1], 11):
        th = math.radians(d)                                  # positive = trailing edge down
        xs = r0["x_le"] + x * r0["chord"]
        for zz in (r0["z_le"] + yu * r0["chord"], r0["z_le"] + yl * r0["chord"]):
            dx, dz = xs - piv[0], zz - piv[2]
            xr = piv[0] + dx * math.cos(th) + dz * math.sin(th)
            zr = piv[2] - dx * math.sin(th) + dz * math.cos(th)
            w = af.half_width(xr, zr)
            worst_body = max(worst_body, float(np.max(w)))
    gap_body = hp["y_root"] - worst_body                       # >= y_root_gap: body never inside the moving root
    stub_gap = hp["y_root"] - TS["stabilator_stub"]["params"]["y_out"]
    # fin roots
    fp = TS["fin"]["params"]
    g = math.radians(fp["cant_deg"])
    fr = TS["fin"]["sections"][0]
    xf, tf = airfoil_thickness(fr["airfoil"])
    xs = fr["x_le"] + xf * fr["chord"]
    th = tf * fr["chord"]
    gaps = []
    for sgn in (1.0, -1.0):
        yy = fr["y"] + sgn * 0.5 * th * math.cos(g)
        zz = fr["z_le"] - sgn * 0.5 * th * math.sin(g)
        gaps.append(zz - af.z_top(xs, yy))
    fin_gap = float(np.max(np.maximum(*gaps)))
    out = {"stab_root_gap_m": stub_gap, "stab_root_body_clearance_m": gap_body,
           "stab_root_deflection_range_deg": list(rng_), "fin_root_max_gap_m": fin_gap,
           "fin_root_embed_min_m": -fin_gap}
    if tp is not None:
        out["fin_exposed_area_each_m2"] = tp["fin"]["area"]
        out["fin_panel_area_each_m2"] = tp["fin"].get("area_panel", tp["fin"]["area"])
    return out


def stab_hinge_moments(S: dict) -> dict:
    """Hinge moments of one all-moving stabilator panel about its spindle: lift x (AC - spindle) with the AC anywhere
    in controls.ac_mac_fraction_range of the panel MAC (low-AR swept panel, estimate), local dynamic pressure
    eta_t q. Cases: VA at the stabilator CLmax (CS-LUAS.423 full control movement at VA), VD at 1/3 of it, and the
    largest steady trim (continuous duty). Actuator check (CS-LUAS.395(a)(1), standards.yaml): actuator peak torque x
    linkage ratio >= 1.25 x max hinge moment; rated torque x linkage ratio >= continuous trim hinge moment; actuator
    travel / linkage ratio covers the deflection range."""
    st = S["tail"]["surfaces"]["stabilator"]
    hp, C = st["params"], st["controls"]
    sp = stab_panel(hp)
    mac, Sp = sp["mac"], sp["area_panel"]
    f_lo, f_hi = C["ac_mac_fraction_range"]
    e_max = (f_hi - hp["pivot_mac_fraction"]) * mac
    e_min = (f_lo - hp["pivot_mac_fraction"]) * mac
    eta = float(S["aero"]["stability_rules"]["eta_tail"])
    clt = float(S["aero"]["stability_rules"]["stabilator_clmax"])
    ST_ = S["structures"]
    VA, VD = float(ST_["VC_eas"]), float(ST_["VD_eas"])
    H_VA = eta * 0.5 * RHO0 * VA ** 2 * Sp * clt * e_max
    H_VD = eta * 0.5 * RHO0 * VD ** 2 * Sp * clt / 3.0 * e_max
    act = C["actuator"]
    k = float(C["linkage_ratio"])
    H_design = max(H_VA, H_VD)
    trim_cl = float(C.get("trim_cl_local_max", 0.4))
    V_trim = float(ST_["VC_eas"])
    H_trim = eta * 0.5 * RHO0 * V_trim ** 2 * Sp * trim_cl * e_max
    travel = float(act["travel_deg"]) / k
    rng_ = C["range_deg"]
    return {"panel_area_m2": Sp, "panel_mac_m": mac, "spindle_x": sp["x_pivot"], "ac_offset_max_m": e_max,
            "ac_offset_min_m": e_min, "statically_stable_surface": bool(e_min > 0.0),
            "H_VA_Nm": H_VA, "H_VD_Nm": H_VD, "H_design_Nm": H_design, "H_trim_continuous_Nm": H_trim,
            "actuator": act["model"], "linkage_ratio": k,
            "peak_capacity_Nm": float(act["torque_peak_Nm"]) * k, "rated_capacity_Nm": float(act["torque_rated_Nm"]) * k,
            "peak_margin": float(act["torque_peak_Nm"]) * k / (1.25 * H_design),
            "rated_margin": float(act["torque_rated_Nm"]) * k / max(H_trim, 1e-9),
            "surface_travel_deg": travel, "travel_ok": bool(travel >= max(abs(rng_[0]), abs(rng_[1])))}


def spar_depth_profile(S: dict, af: Airframe, frac: float, n: int = 40) -> dict:
    """OML depth along a spar line (fraction ``frac`` of the reference-trapezoid chord) from the body side to the
    outer-panel joint: union of the wing section (linear loft between sections, twist about the section leading
    edge) and the body; depth = top - bottom; offset = mid-line - wing root plane."""
    P = S["wing"]["planform"]
    T = trapezoid(P)
    secs = S["wing"]["sections"]
    ys_s = np.array([s_["y"] for s_ in secs])
    yj = float(P["y_junction"])
    cache = {}

    def surf(i, x):
        s_ = secs[i]
        key = (s_["airfoil"], round(float(s_.get("thickness_scale", 1.0)), 4))
        if key not in cache:
            cache[key] = oml.resampled(s_["airfoil"], 201, 0.0015 / max(s_["chord"], 1e-6), key[1])
        xc, yu, yl = cache[key]
        u = (x - s_["x_le"]) / s_["chord"]
        if u < 0 or u > 1:
            return None
        tw = math.radians(s_["twist_deg"])
        dz = -(x - s_["x_le"]) * math.sin(tw)
        return (s_["z_le"] + s_["chord"] * float(np.interp(u, xc, yu)) + dz,
                s_["z_le"] + s_["chord"] * float(np.interp(u, xc, yl)) + dz)
    rows = []
    y0 = float(af.sec(float(T["xle"](0.4) + frac * T["c"](0.4)))[0][0])
    for y in np.linspace(y0, yj, n):
        x = float(T["xle"](y) + frac * T["c"](y))
        j = int(np.clip(np.searchsorted(ys_s, y) - 1, 0, len(ys_s) - 2))
        t = (y - ys_s[j]) / max(ys_s[j + 1] - ys_s[j], 1e-9)
        a, b = surf(j, x), surf(j + 1, x)
        if a is None or b is None:
            continue
        top = (1 - t) * a[0] + t * b[0]
        bot = (1 - t) * a[1] + t * b[1]
        a_b = float(af.sec(x)[0][0])
        if y < a_b:
            top = max(top, float(af.z_top(x, y)))
            bot = min(bot, float(af.z_bot(x, y)))
        rows.append({"y": float(y), "x": x, "depth": top - bot, "mid": 0.5 * (top + bot) - float(P["z_root"])})
    d = np.array([r_["depth"] for r_ in rows])
    return {"rows": rows, "min_depth_m": float(d.min()), "y_min_depth": rows[int(np.argmin(d))]["y"],
            "max_mid_offset_m": float(max(abs(r_["mid"]) for r_ in rows))}


def gust_matrix(S: dict, masses: list, clmax_fn, cla: float, VD: float) -> list:
    """CS-LUAS/VLA 341 gust and manoeuvre limit load factors for every mass x altitude (sea level, loiter altitude,
    service ceiling) with the aeroplane lift-curve slope ``cla`` (configuration value); clmax_fn(m) -> trimmed CLmax."""
    ST_ = S["structures"]
    out = []
    for m in masses:
        ws = m * G / float(S["wing"]["area"])
        for h in (0.0, float(S["mission"]["loiter_altitude"]), float(S["mission"]["service_ceiling"])):
            vn = ST.vn_diagram(ws, clmax_fn(m), ST_["clmin_negative"], ST_["n_limit_pos"], ST_["n_limit_neg"],
                               ST_["VC_eas"], VD, cla, float(S["wing"]["mac"]), ST_["gust"]["Ude_VC"],
                               ST_["gust"]["Ude_VD"], rho=AL.isa(h)["rho"])
            out.append({"mass_kg": m, "altitude_m": h, "n_pos": vn["n_limit_pos"], "n_neg": vn["n_limit_neg"],
                        "n_gust_VC": vn["gust_C"], "n_gust_VD": vn["gust_D"], "kg": vn["kg"],
                        "lift_factor_N": vn["n_limit_pos"] * m * G})
    return out


def electrical_budget(S: dict, gen_W: float) -> dict:
    """Continuous electrical load cases vs the SG750 output at the loiter rpm (R-32): baseline (HD59), E180 growth
    turret; both with the research-payload power allowance. Peaks above the generator output come from the 14S2P
    LiFePO4 buffer battery (components.yaml electrical: 28 min hold-up at 400 W)."""
    E = S["engine"]["electrical_budget"]
    base = float(E["continuous_base_W"])
    rp = float(E["research_payload_allowance_W"])
    hd59, e180_avg, e180_pk = float(E["hd59_average_W"]), float(E["e180_average_W"]), float(E["e180_peak_W"])
    cont_hd59 = base + rp
    cont_e180 = base - hd59 + e180_avg + rp
    peak_e180 = base - hd59 + e180_pk + rp
    worst = max(cont_hd59, cont_e180)
    return {"generator_W_loiter": gen_W, "continuous_hd59_W": cont_hd59, "continuous_e180_W": cont_e180,
            "peak_e180_W": peak_e180, "margin_continuous": gen_W / worst,
            "peak_deficit_W": max(peak_e180 - gen_W, 0.0),
            "battery_holdup_peak_deficit_min": (float(E["battery_usable_Wh"]) / max(peak_e180 - gen_W, 1e-9) * 60.0
                                                if peak_e180 > gen_W else float("inf"))}


def packaging(S: dict, af: Airframe, mass_c: list) -> dict:
    """Keep-out boxes of the spec layout zones checked against the OML (corners and edge points inside with the zone
    clearance; the gear wells and the turret bay are checked on their real envelopes), pairwise overlap of the zones
    and the fuel cells, fuel volume (bladder cells x tank efficiency) vs the required volume."""
    Z = S["layout"]["zones_preliminary"]
    out = {}
    envelope_checked = {"nose_gear_well", "main_gear_wells", "turret_bay"}
    for k, z in Z.items():
        if "box" not in z or k in envelope_checked:
            continue
        box = np.asarray(z["box"], float)
        m = float(z.get("clearance", 0.010))
        ys = [-box[1, 1], 0.0, box[1, 1]] if z.get("symmetric", False) else [box[0, 1], box[1, 1]]
        pts = np.array([[x, y, zz] for x in np.linspace(box[0, 0], box[1, 0], 5) for y in ys
                        for zz in (box[0, 2], box[1, 2])])
        ok = bool(np.all(af.inside(pts, margin=m)))
        out[k] = {"box": box.tolist(), "fits": ok, "clearance_m": m}
    gs = gear_stowage(S, af)
    out["main_gear_wells"] = {"fits": gs["main"]["ok"], **{k: v for k, v in gs["main"].items() if k != "ok"}}
    out["nose_gear_well"] = {"fits": gs["nose"]["ok"]}
    out["turret_bay"] = {"fits": bool(turret_checks_light(S, af))}
    # pairwise overlaps (boxes; the main wells and the turret bay are boxes around their envelopes)
    names = [k for k, z in Z.items() if "box" in z and k != "wing_carry_through"]
    ov = []
    for i in range(len(names)):
        for j in range(i + 1, len(names)):
            if names[i].startswith("engine_") and names[j].startswith("engine_"):
                continue                                     # boxes of the same engine
            v = _box_overlap(Z[names[i]]["box"], Z[names[j]]["box"])
            if v > 1e-7:
                ov.append({"a": names[i], "b": names[j], "volume_m3": v})
    for c in S["layout"]["fuel_cells"]:
        cb = [[c["x"][0], 0.0, c["z"][0]], [c["x"][1], 1.0, c["z"][1]]]
        for n_ in names:
            v = _box_overlap(cb, Z[n_]["box"])
            if v > 1e-7:
                ov.append({"a": c["name"], "b": n_, "volume_m3": v})
    out["overlaps"] = {"fits": len(ov) == 0, "pairs": ov}
    fu = S["mass"]["fuel_kg"]
    rho = float(S["engine"]["fuel"]["density_kg_per_m3"])
    vol_req = fu / rho * (1 + float(S["structures"]["fuel"]["expansion_fraction"]))
    cells = S["layout"]["fuel_cells"]
    vol, mom = 0.0, 0.0
    cell_out = []
    for c in cells:
        xs = np.linspace(c["x"][0], c["x"][1], 30)
        Ais = np.array([_area_band(af, x, c["z"][0], c["z"][1], float(c.get("inset", 0.025))) for x in xs])
        v = float(np.trapz(Ais, xs)) * float(S["structures"]["fuel"]["tank_volume_efficiency"])
        vol += v
        mom += v * float(np.trapz(Ais * xs, xs) / max(np.trapz(Ais, xs), 1e-12))
        cell_out.append({"name": c["name"], "x": c["x"], "z": c["z"], "volume_m3": v})
    out["fuel"] = {"required_m3": vol_req, "available_m3": vol, "fits": vol >= vol_req, "cells": cell_out,
                   "fuel_cg_x_volume_centroid": mom / max(vol, 1e-12), "design_cg_x": mass_c[0]["x"]}
    return out


def _area_band(af: Airframe, x: float, z0: float, z1: float, inset: float, n: int = 60) -> float:
    """Internal area of the body section at x between heights z0..z1 (inset from the skin)."""
    a, bt, bb, zc, nt, nb = (float(v[0]) for v in af.sec(x))
    zs = np.linspace(z0, z1, n)
    w = []
    for z in zs:
        if z >= zc:
            hh, nn = bt - inset, nt
        else:
            hh, nn = bb - inset, nb
        r = abs(z - zc) / max(hh, 1e-9)
        w.append(0.0 if r >= 1 or hh <= 0 else 2 * max(a - inset, 0.0) * (1 - r ** nn) ** (1 / nn))
    return float(np.trapz(w, zs))


def vn_summary(S: dict, m0: float, clmax: float, cla: float, VD: float) -> dict:
    ST_ = S["structures"]
    ws = m0 * G / float(S["wing"]["area"])
    out = {}
    for h in (0.0, float(S["mission"]["service_ceiling"])):
        vn = ST.vn_diagram(ws, clmax, ST_["clmin_negative"], ST_["n_limit_pos"], ST_["n_limit_neg"], ST_["VC_eas"], VD,
                           cla, float(S["wing"]["mac"]), ST_["gust"]["Ude_VC"], ST_["gust"]["Ude_VD"],
                           rho=AL.isa(h)["rho"])
        out[int(h)] = {"VS": vn["VS"], "VA": min(vn["VA"], ST_["VC_eas"]), "VC": ST_["VC_eas"], "VD": VD, "kg": vn["kg"],
                       "n_gust_VC": vn["gust_C"], "n_gust_VD": vn["gust_D"], "n_limit_pos": vn["n_limit_pos"],
                       "n_limit_neg": vn["n_limit_neg"]}
    return out


# =====================================================================================================================
# 11. design closure: geometry generators + layout rules + mass items + MTOM/fuel + wing station + gear height
# =====================================================================================================================
def _round(x, n=5):
    if isinstance(x, dict):
        return {k: _round(v, n) for k, v in x.items()}
    if isinstance(x, (list, tuple)):
        return [_round(v, n) for v in x]
    if isinstance(x, (float, np.floating)):
        return round(float(x), n)
    if isinstance(x, np.ndarray):
        return _round(x.tolist(), n)
    return x


def spar_x(P: dict, y: float, frac: float) -> float:
    T = trapezoid(P)
    return float(T["xle"](y) + frac * T["c"](y))


def stab_panel(hp: dict) -> dict:
    """Planform of one stabilator panel from its parameters: MAC, MAC LE x (root at y_root), spindle x at
    ``pivot_mac_fraction`` of the panel MAC (all-moving surface hinged near its aerodynamic centre)."""
    cr, ct, b = float(hp["root_chord"]), float(hp["tip_chord"]), float(hp["span"])
    lam = ct / cr
    mac = 2.0 / 3.0 * cr * (1 + lam + lam ** 2) / (1 + lam)
    y_mac = b / 3.0 * (1 + 2 * lam) / (1 + lam)
    x_le_mac = float(hp["x_le_root"]) + y_mac * math.tan(math.radians(hp["sweep_le_deg"]))
    return {"mac": mac, "y_mac_from_root": y_mac, "x_le_mac": x_le_mac,
            "x_pivot": x_le_mac + float(hp["pivot_mac_fraction"]) * mac, "area_panel": 0.5 * (cr + ct) * b}


def fin_te_crossing(fp: dict, z_hub: float) -> float:
    """Span fraction t* along the fin trailing edge where its distance from the propeller axis (y, z - z_hub)
    reaches ``fp['guard_radius']`` (inf if the fin never reaches it)."""
    g = math.radians(fp["cant_deg"])
    t = np.linspace(0.0, 1.0, 2001)
    y = fp["y_root"] + t * fp["span"] * math.sin(g)
    z = fp["z_root"] + t * fp["span"] * math.cos(g)
    r = np.hypot(y, z - z_hub)
    i = np.where(r >= fp["guard_radius"])[0]
    if not len(i):
        return float("inf")
    j = int(i[0])
    if j == 0:
        return 0.0
    return float(np.interp(fp["guard_radius"], [r[j - 1], r[j]], [t[j - 1], t[j]]))


def build_geometry(S: dict) -> None:
    """Fuselage stations, wing sections, tail sections and the propeller plane from the design parameters (in place).

    * chine held on the wing plane until ``chine_hold_aft_of_wing_te`` behind the wing-root trailing edge (the LERX/glove
      upper surface meets the chine along the whole root chord), then rising to the thrust line at the engine;
    * propeller plane = engine flange (fuselage.lines.x_hub) + hub spacer + propeller hub half thickness;
    * stabilator: spindle at ``pivot_mac_fraction`` of the panel MAC (near the panel aerodynamic centre, small hinge
      moments), root rib at the largest chine half width swept by the root chord (+ gap); a FIXED root stub from inside
      the body out to the moving surface (constant root gap over the whole deflection range);
    * fin: root buried along the whole root chord (no gap between fin and body), position from the propeller-guard
      rule (trailing edge crosses the propeller plane at ``guard_radius`` = R + ``prop_guard_radial_margin``);
    * ventral fin: root on the keel, span from the bumper rule (design_closure)."""
    L = S["fuselage"]["lines"]
    W = S["wing"]
    P = W["planform"]
    T = trapezoid(P)
    x_te_root = float(T["xte"](L["a_max"]))
    x_c0 = max(L["x_hub"] - L["l_chine_rise"], x_te_root + float(L.get("chine_hold_aft_of_wing_te", 0.0)))
    st = body_stations(L, x_c0)
    S["fuselage"]["stations"] = st.tolist()
    S["fuselage"]["lines_derived"] = {"x_chine_rise_start": round(x_c0, 5), "wing_root_te_x_at_chine": round(x_te_root, 5)}
    fus = oml.Fuselage(st)
    secs, info = wing_sections(P, fus)
    W.update({"sections": secs, "span": P["span"], "area": P["area"], "aspect_ratio": T["AR"], "taper": P["taper"],
              "sweep_c4_deg": P["sweep_c4_deg"], "dihedral_deg": P["dihedral_deg"],
              "incidence_deg": P["incidence_deg"], "washout_deg": P["washout_deg"], "mac": T["mac"],
              "mac_le_x": T["x_le_mac"], "mac_y": T["y_mac"],
              "planform_derived": {"root_chord_centreline": T["cr"], "tip_chord": T["ct"], "apex": info["apex"],
                                   "junction": info["junction"], "y_strake_end": info["y_strake_end"],
                                   "le_sweep_deg_by_section": info["le_sweep_deg"],
                                   "chine_slope_at_apex": info["chine_slope_at_apex"],
                                   "main_spar_frac": P["main_spar_frac"], "rear_spar_frac": P["rear_spar_frac"]}})
    # propeller plane (engine flange + hub spacer + hub half thickness) and thrust line
    pr = S["propeller"]
    x_plane = float(L["x_hub"]) + float(pr["hub_spacer"]) + float(pr["hub_half_thickness"])
    pr["plane_x"] = round(x_plane, 5)
    pr["hub"] = [round(x_plane, 5), 0.0, float(L["z_t"])]
    R_p = 0.5 * float(pr["diameter"])
    af0 = Airframe({"fuselage": S["fuselage"], "wing": W, "tail": {"surfaces": {}}})
    TS = S["tail"]["surfaces"]
    # ---- stabilator (moving panel) + fixed root stub
    hp = TS["stabilator"]["params"]
    sp = stab_panel(hp)
    x_piv = sp["x_pivot"]
    m_sw = float(hp["root_sweep_margin"])
    xs = np.linspace(hp["x_le_root"] - m_sw, hp["x_le_root"] + hp["root_chord"] + m_sw, 80)
    a_root = float(np.max(af0.sec(xs)[0]))
    zc_p = float(af0.sec(x_piv)[3][0])
    hp["y_root"] = round(a_root + hp["y_root_gap"], 5)
    hp["z_root"] = round(zc_p, 5)
    TS["stabilator"]["sections"] = surface_from_params("stabilator", hp)
    TS["stabilator"]["pivot"] = _round([x_piv, hp["y_root"], zc_p])
    sb = TS["stabilator_stub"]
    sbp = sb["params"]
    xs = np.linspace(hp["x_le_root"], hp["x_le_root"] + hp["root_chord"], 60)
    w_z = af0.half_width(xs, zc_p)
    y_out = hp["y_root"] - hp["y_root_gap"]
    y_in = min(float(np.min(w_z)) - float(sbp["root_embed"]), y_out - 0.01)
    sbp.update({"y_in": round(max(y_in, 0.02), 5), "y_out": round(y_out, 5), "x_le": hp["x_le_root"],
                "chord": hp["root_chord"], "z": round(zc_p, 5), "airfoil": hp["airfoil"]})
    sb["sections"] = [{"y": sbp["y_in"], "x_le": round(float(hp["x_le_root"]), 5), "z_le": sbp["z"],
                       "chord": round(float(hp["root_chord"]), 5), "twist_deg": 0.0, "airfoil": hp["airfoil"]},
                      {"y": sbp["y_out"], "x_le": round(float(hp["x_le_root"]), 5), "z_le": sbp["z"],
                       "chord": round(float(hp["root_chord"]), 5), "twist_deg": 0.0, "airfoil": hp["airfoil"]}]
    # ---- fins: buried root + propeller-guard position
    fp = TS["fin"]["params"]
    fp["guard_radius"] = round(R_p + float(fp["prop_guard_radial_margin"]), 5)
    g = math.radians(fp["cant_deg"])
    t_fin = oml.max_thickness(fp["airfoil"])[0] * fp["root_chord"]
    for _ in range(6):
        xs = np.linspace(fp["x_le_root"], fp["x_le_root"] + fp["root_chord"], 40)
        th = np.interp(np.linspace(0, 1, 40), *airfoil_thickness(fp["airfoil"])) * fp["root_chord"]
        z_need = []
        for sgn in (1.0, -1.0):                              # both faces of the canted root airfoil
            yy = fp["y_root"] + sgn * 0.5 * th * math.cos(g)
            zz = af0.z_top(xs, yy) - sgn * 0.5 * th * math.sin(g)
            z_need.append(zz)
        fp["z_root"] = round(float(np.min(np.minimum(*z_need))) - float(fp["root_embed"]), 5)
        tc = fin_te_crossing(fp, float(L["z_t"]))
        if not math.isfinite(tc):
            raise ValueError("fin too short to reach the propeller-guard radius")
        dx_te = tc * (fp["span"] * math.tan(math.radians(fp["sweep_le_deg"])) + fp["tip_chord"] - fp["root_chord"])
        x_new = round(x_plane - fp["root_chord"] - dx_te, 5)
        if abs(x_new - fp["x_le_root"]) < 1e-5:
            break
        fp["x_le_root"] = x_new
    fp["te_crossing_span_fraction"] = round(tc, 5)
    fp["root_max_thickness"] = round(t_fin, 5)
    TS["fin"]["sections"] = surface_from_params("fin", fp)
    # ---- ventral fin / bumper
    vp = TS["ventral"]["params"]
    xm = vp["x_le_root"] + 0.5 * vp["root_chord"]
    vp["z_root"] = round(float(af0.z_bot(xm)) + vp["root_embed"], 5)
    TS["ventral"]["sections"] = surface_from_params("ventral", vp)
    tip = TS["ventral"]["sections"][-1]
    TS["ventral"]["bumper"] = {"contact_point": _round([tip["x_le"] + vp["skid_chord_fraction"] * tip["chord"], 0.0,
                                                        tip["z_le"] - vp["skid_height"]]),
                               "skid": "replaceable UHMW-PE shoe on a 4130 strap at the ventral-fin tip"}
    for k, v in TS.items():
        pp = panel_planform(v["sections"])
        v["area"] = round(pp["area"] * (2 if v.get("mirror", True) else 1), 5)
        v["span"] = round(pp["span"], 5)
        v["mac"] = round(pp["mac"], 5)
        v["x_ac"] = round(pp["x_ac"], 5)


def gear_placement(S: dict, af: Airframe, wa: dict, clm: dict, cases_ground: list, x_mg_prev: float | None,
                   x_payload: float | None = None, pot: dict | None = None) -> dict:
    """Retractable tricycle placement.
    Main gear x: aft of the most aft ground CG by the tip-back rule, far enough for the minimum nose-wheel load and
    behind the belly payload bay (centred on the empty-mass CG, ``x_payload``);
    ground height: lowest of the propeller static-clearance, attitude/bumper and flat-tyre rules; track: from the
    turnover rule (most forward CG); trunnion height and position from the stowage (main_gear_layout).
    Nose gear: x from the layout rule, height from the static ground attitude, pivot from the keel stowage."""
    LG = S["landing_gear"]
    R = LG["rules"]
    r_t = LG["tyre"]["diameter"] / 2
    defl = LG["tyre"]["static_deflection"]
    mg, ng = LG["main"], LG["nose"]
    pr = S["propeller"]
    Z_T, x_p = float(pr["hub"][2]), float(pr["plane_x"])
    Rp = float(pr["diameter"]) / 2
    CLa = wa["CL_alpha"]
    k_td = float(S["mission"]["touchdown_speed_factor"])
    th_lof = liftoff_attitude(wa, clm, pot)
    th_td = math.degrees((clm["ld_trimmed"] / k_td ** 2 - wa["CL_0"] - clm["dCL0_ld"]) / CLa)
    th_need = max(th_lof, th_td + R["flare_margin_deg"]) + R["bumper_margin_deg"] + R["design_margin_deg"]
    th_s = R["static_attitude_deg"]
    d_ext = float(mg["static_compression"]) + defl
    x_ng = ng["axle_x"]
    x_aft = max(c["x"] for c in cases_ground)
    x_fwd = min(c["x"] for c in cases_ground)
    z_cg = max(c["z"] for c in cases_ground)
    x_mg = x_mg_prev if x_mg_prev is not None else x_aft + 0.2
    z_g = 0.0
    tb = max(R["tipback_min_deg"], th_need + R["tipback_over_bumper_deg"])
    LR = S["layout"]["rules"]
    x_bay = -1.0
    if x_payload is not None:
        x_bay = x_payload + 0.5 * LR["payload_bay_length"] + LR["payload_bay_to_well_gap"] + r_t + \
            float(R["well_clearance"])
    def zg_clear(arm, th, dz=0.0):
        """Ground height giving the CS-VLA 925(a) clearance (+ design margin) at attitude th (deg)."""
        c_req = R["prop_clear_static_min"] + R["clearance_design_margin"]
        return Z_T - Rp + dz - (c_req + arm * math.sin(math.radians(th))) / math.cos(math.radians(th))
    for _ in range(40):
        arm = x_p - x_mg
        zg_a = zg_clear(arm, th_s)                                  # static attitude, gear statically deflected
        zg_d = zg_clear(arm, max(th_lof, th_s))                     # normal take-off (lift-off) attitude, static gear
        zg_e = zg_clear(arm, max(th_td, th_s), d_ext)               # touch-down attitude, gear unloaded
        zg_b = Z_T - Rp - arm * math.tan(math.radians(th_need + R["prop_margin_deg"]))
        # flat main tyres + bottomed main struts, nose leg static: the aircraft pitches up about the nose wheel
        wb_ = x_mg - x_ng
        dz_m = (mg["stroke"] - mg["static_compression"]) + (r_t - LG["tyre"]["rim_radius"])
        zg_c = Z_T - Rp - R["prop_clear_flat_min"] - dz_m * (x_p - x_ng) / wb_
        z_g = min(zg_a, zg_b, zg_c, zg_d, zg_e)
        h_cg = z_cg - z_g
        x_tb = x_aft + h_cg * math.tan(math.radians(tb)) + R["tipback_x_margin"]
        x_nl = (x_aft - R["nose_load_min"] * x_ng) / (1.0 - R["nose_load_min"])
        x_new = max(x_tb, x_nl, x_bay)
        if abs(x_new - x_mg) < 1e-7:
            break
        x_mg = x_new
    wb = x_mg - x_ng
    h_cg = z_cg - z_g
    ln = x_fwd - x_ng
    sin_d = h_cg / (ln * math.tan(math.radians(R["turnover_max_deg"] - R["turnover_margin_deg"])))
    y_mg_req = wb * math.tan(math.asin(min(sin_d, 0.99)))
    mgl = main_gear_layout(S, af, x_mg, z_g, y_mg_req)
    # nose-up static attitude th_s: the nose contact lies (x_mg - x_ng) tan(th_s) below the main contact in body axes
    z_axn = z_g - (x_mg - x_ng) * math.tan(math.radians(th_s)) + r_t - defl
    ngl = nose_gear_layout(S, af, x_ng, z_axn)
    xb = S["tail"]["surfaces"]["ventral"]["bumper"]["contact_point"][0]
    z_b_req = z_g + (xb - x_mg) * math.tan(math.radians(th_need))
    T = mgl["trunnion"]
    return {"x_mg": x_mg, "z_g": z_g, "z_axle": mgl["z_axle"], "leg_length": mgl["leg_length"], "y_trunnion": float(T[1]),
            "z_trunnion": float(T[2]), "y_axle": float(mgl["axle"][1]), "y_mg_required": y_mg_req,
            "main": mgl, "nose": ngl, "x_ng": x_ng, "z_axle_nose": z_axn, "nose_leg_length": ngl["leg_length"],
            "theta_lof": th_lof, "theta_td": th_td, "theta_need": th_need, "tipback_required_deg": tb,
            "z_bumper_required": z_b_req, "x_mg_tipback": x_tb, "x_mg_nose_load": x_nl, "x_mg_payload_bay": x_bay,
            "x_payload": x_payload,
            "governing_x_mg": ["tip-back", "nose-wheel load", "payload bay"][int(np.argmax([x_tb, x_nl, x_bay]))],
            "rule": ["static clearance", "attitude/bumper", "flat tyre + bottomed strut", "lift-off clearance",
                     "touch-down clearance"][int(np.argmin([zg_a, zg_b, zg_c, zg_d, zg_e]))]}


def layout_zones(S: dict, af: Airframe, gp: dict) -> dict:
    """Preliminary keep-out zones (sizing phase) from the layout rules; the layout phase refines them. Boxes are
    [[x0, y0, z0], [x1, y1, z1]] (``symmetric``: mirrored about the centre line). Rule boxes (avionics, parachute,
    mission computer, aft equipment) come from spec.layout.rules.boxes; the gear wells, turret bay, wing box, payload
    bay and engine boxes are derived from the geometry."""
    R = S["layout"]["rules"]
    L = S["fuselage"]["lines"]
    P = S["wing"]["planform"]
    xh, zt = L["x_hub"], L["z_t"]
    x_f0, x_r0 = spar_x(P, 0.0, P["main_spar_frac"]), spar_x(P, 0.0, P["rear_spar_frac"])
    t_j = 0.1595 * trapezoid(P)["c"](P["y_junction"])
    h_box = 0.95 * float(t_j) - 0.010
    z_box = (P["z_root"] - h_box / 2, P["z_root"] + h_box / 2)
    E = S["engine"]["envelope"]
    zones = {}
    for k, b in R["boxes"].items():
        zones[k] = {"box": [[b["x"][0], 0.0, b["z"][0]], [b["x"][1], b["half_width"], b["z"][1]]], "symmetric": True,
                    "clearance": b.get("clearance", 0.008), "content": b.get("content", "")}
    ngl = gp["nose"]
    zones["nose_gear_well"] = {"box": _round(ngl["well_box"]), "symmetric": True, "clearance": 0.0,
                               "content": "stowed nose wheel (vertical) + leg slot; checked on the tyre envelope"}
    tg = turret_geometry(S, af)
    T = S["payload"]["turret"]
    rg = T["growth_envelope"]["diameter"] / 2
    zones["turret_bay"] = {"box": _round([[tg["x"] - rg - 0.01, 0.0, tg["z_keel"]],
                                          [tg["x"] + rg + 0.01, rg + 0.01, tg["mechanism_top_z"]]]), "symmetric": True,
                           "clearance": 0.0, "content": "E180 growth envelope (sphere + cylinder) + lift mechanism; "
                                                         "checked on the envelope (turret_checks)",
                           "ball_center_retracted_z": tg["ball_center_retracted_z"]}
    zones["wing_carry_through"] = {"box": [[x_f0 - 0.02, 0.0, z_box[0]], [x_r0 + 0.02, 0.30, z_box[1]]],
                                   "symmetric": True, "clearance": 0.0, "content": "spar box through the body"}
    mw = gp["main"]["well_box"]
    zones["main_gear_wells"] = {"box": _round([[mw[0][0], 0.0, mw[0][2]], [mw[1][0], mw[1][1], mw[1][2]]]),
                                "symmetric": True, "clearance": 0.0,
                                "content": "stowed main wheels side by side + trunnion fittings; checked on the tyre "
                                           "envelopes (gear_stowage)"}
    zp = R["payload_bay_roof_z"]
    x_well0 = mw[0][0]
    if gp.get("x_payload") is not None:
        xpc = gp["x_payload"]
        pbx = [xpc - 0.5 * R["payload_bay_length"], min(xpc + 0.5 * R["payload_bay_length"],
                                                         x_well0 - R["payload_bay_to_well_gap"])]
    else:                                                    # bay ends at the main wells
        pbx = [x_well0 - R["payload_bay_to_well_gap"] - R["payload_bay_length"], x_well0 - R["payload_bay_to_well_gap"]]
    hw = R["payload_bay_half_width"]
    z_floor = max(float(af.z_bot(x, hw)) for x in np.linspace(pbx[0], pbx[1], 9)) + 0.012
    zones["payload_bay"] = {"box": _round([[pbx[0], 0.0, z_floor], [pbx[1], hw, zp]]), "symmetric": True,
                            "clearance": 0.0, "content": "belly research-payload bay under the forward fuel cell"}
    zones["engine_cylinder_slab"] = {"box": [[xh - 0.18, 0.0, zt - E["crank_axis_to_cylinder_side"]],
                                             [xh - 0.05, E["width"] / 2, zt + E["crank_axis_to_cylinder_side"]]],
                                     "symmetric": True, "clearance": 0.010, "content": "L 275 EF cylinders/heads"}
    zones["engine_intake_box"] = {"box": [[xh - 0.18, 0.0, zt - (E["height"] - E["crank_axis_to_cylinder_side"])],
                                          [xh - 0.05, E["intake_box_width"] / 2, zt - E["crank_axis_to_cylinder_side"]]],
                                  "symmetric": True, "clearance": 0.010, "content": "intake box below the crank"}
    zones["engine_crankcase_sg750"] = {"box": [[xh - E["length_with_sg750"], 0.0, zt - 0.10], [xh - 0.02, 0.10, zt + 0.05]],
                                       "symmetric": True, "clearance": 0.010, "content": "crankcase + SG750"}
    zones["firewall_x"] = {"x": xh - E["length_with_sg750"] - R["firewall_gap"]}
    return {"zones": zones, "x_spar_main_0": x_f0, "x_spar_rear_0": x_r0, "z_box": z_box, "h_box": h_box,
            "x_pivot_nose": float(ngl["pivot"][0]), "z_pivot_nose": float(ngl["pivot"][2]), "turret": tg}


def fuel_cells(S: dict, af: Airframe, lz: dict) -> list:
    R = S["layout"]["rules"]
    x_f0, x_r0 = lz["x_spar_main_0"], lz["x_spar_rear_0"]
    zp = R["payload_bay_roof_z"]
    top = R["fuel_top_z"]
    zb = lz["z_box"]
    wz = lz["zones"]["main_gear_wells"]["box"][1][2]
    cells = [{"name": "forward_cell", "x": [x_f0 - 0.03 - R["fuel_cell_fwd_length"], x_f0 - 0.03], "z": [zp + 0.008, top]},
             {"name": "saddle_cell", "x": [x_f0 + 0.01, x_r0 - 0.01], "z": [zb[1] + 0.012, top]}]
    if R["fuel_cell_aft_length"] > 0:
        cells.append({"name": "aft_cell", "x": [x_r0 + 0.03, x_r0 + 0.03 + R["fuel_cell_aft_length"]],
                      "z": [max(zp, wz) + 0.008, top]})
    return [_round(c) for c in cells]


def control_surface_areas(S: dict) -> dict:
    """Planform areas of the movable surfaces on the fixed wing/fins (both sides): ailerons, flaps, rudders."""
    W = S["wing"]
    secs = W["sections"]
    ys = np.array([s["y"] for s in secs])
    xle = np.array([s["x_le"] for s in secs])
    ch = np.array([s["chord"] for s in secs])
    b2 = float(W["span"]) / 2
    T = trapezoid(W["planform"])
    out = {}
    for k in ("aileron", "flap"):
        c = W["controls"][k]
        yy = np.linspace(c["eta0"] * b2, c["eta1"] * b2, 60)
        cf = c["chord_fraction"] * T["c"](yy)                     # surface chord: fraction of the outer-panel chord
        x_h = np.interp(yy, ys, xle + ch) - cf
        out[k] = {"area": 2 * float(np.trapz(cf, yy)), "x": float(np.trapz(cf * (x_h + 0.4 * cf), yy) / np.trapz(cf, yy)),
                  "y": float(np.trapz(cf * yy, yy) / np.trapz(cf, yy))}
    fin = S["tail"]["surfaces"]["fin"]
    fp = fin["params"]
    out["rudder"] = {"area": fin["area"] * fp["rudder_chord_fraction"],
                     "x": fin["x_ac"] + 0.5 * fin["mac"], "z": float(np.mean([s["z_le"] for s in fin["sections"]]))}
    return out


def mass_items(S: dict, af: Airframe, wing_m: dict, tail_m: dict, shell: dict, lz: dict, gp: dict,
               fuel_x: float, m0: float) -> list:
    """Empty-mass items (name, group, base mass, position, basis). Every item x (1 + growth allowance)."""
    MR = S["mass"]["rules"]
    gr = 1.0 + float(MR["growth_allowance"])
    am = areal_masses(S)
    L = S["fuselage"]["lines"]
    pr = S["propeller"]
    xh, zt = L["x_hub"], L["z_t"]
    x_p = float(pr["plane_x"])
    Z = lz["zones"]
    W = S["wing"]
    P = W["planform"]
    T = trapezoid(P)
    x_f0, x_r0 = lz["x_spar_main_0"], lz["x_spar_rear_0"]
    cs = control_surface_areas(S)
    cs_mass = {k: v["area"] * 2 * am["wing_skin"] * 0.85 + (0.20 if k != "rudder" else 0.08) for k, v in cs.items()}

    def ctr(name):
        b = np.asarray(Z[name]["box"], float)
        return 0.5 * (b[0] + b[1])
    x_mac40 = T["x_le_mac"] + 0.40 * T["mac"]
    z_wing = P["z_root"] + (T["y_mac"] - P["y_junction"]) * math.tan(math.radians(P["dihedral_deg"]))
    ts = S["tail"]["surfaces"]
    pv = ts["stabilator"]["pivot"]
    fin_pp = panel_planform(ts["fin"]["sections"])
    st_pp = panel_planform(ts["stabilator"]["sections"])
    vf_pp = panel_planform(ts["ventral"]["sections"])
    k_ms = m0 / float(MR["gear_reference_mtom"])
    x_pn, z_pn = lz["x_pivot_nose"], lz["z_pivot_nose"]
    LN = gp["nose_leg_length"]
    av = ctr("avionics_power_bay")
    pc = ctr("parachute_bay")
    tb = ctr("turret_bay")
    eq = ctr("equipment_bay_aft")
    I = []

    def add(name, group, m, x, z, basis, y=0.0):
        I.append({"name": name, "group": group, "mass_base_kg": round(float(m), 4), "mass_kg": round(float(m) * gr, 4),
                  "x": round(float(x), 4), "y": round(float(y), 4), "z": round(float(z), 4), "basis": basis})
    E = S["engine"]
    add("engine_group_installed", "propulsion", E["installed_mass_kg"], xh - 0.12, zt - 0.03,
        "baseline.yaml#engine.installed_mass_kg.total (L 275 EF 7.0 kg + ECU, pump, SG750, exhaust, mount isolators)")
    add("propeller", "propulsion", pr["mass_kg"], x_p, zt, "Mejzlik datasheet (propeller.mass_kg)")
    add("spinner_hub_adapter_spacer", "propulsion", MR["spinner_hub_spacer_kg"], xh + 0.07, zt,
        "baseline.yaml#mass_targets.systems_rollup_kg.spinner_and_hub_adapter 0.40 + 0.10 hub spacer (endurance study)")
    add("cooling_baffles_firewall_cowl_flap", "propulsion", MR["cooling_firewall_kg"], xh - 0.22, zt + 0.03,
        "baseline.yaml#mass_targets.systems_rollup_kg.cowling_cooling_ducts_baffles_firewall (cowl skin in the shell)")
    add("dorsal_cooling_inlet_s_duct", "propulsion", MR["cooling_inlet_duct_kg"], xh - 0.55, zt + 0.10,
        "identity study INLET['sduct'] duct mass (estimate)")
    add("fuel_system_3_cells", "fuel", MR["fuel_system_kg"], fuel_x, 0.05,
        "components.yaml fuel_system 1.73 kg + 0.50 kg for three interconnected cells (endurance study +0.35 for two)")
    xa = cs["aileron"]["x"]
    add("actuators_ailerons_2x_DA26", "controls", 2 * 0.27 + 0.10, xa, z_wing,
        "Volz DA 26 0.27 kg datasheet x 2 + horns/pushrods 0.10 (components.yaml)")
    add("actuators_flaps_2x_DA30", "controls", 2 * 0.63 + 0.10 + 0.20, cs["flap"]["x"], P["z_root"],
        "Volz DA 30 0.63 kg datasheet x 2 (one per flap; each flap lies on its removable outer panel, inboard end "
        "outboard of the panel joint) + installation 0.10 + hinges/horns 0.20 (endurance study flap kit)")
    add("actuators_rudders_2x_DA26", "controls", 2 * 0.27 + 0.06, cs["rudder"]["x"], cs["rudder"]["z"],
        "Volz DA 26 x 2 (in the fins) + linkages 0.06 (estimate)")
    sc = ts["stabilator"]["controls"]
    add("actuators_stabilators_2x_DA30", "controls", 2 * float(sc["actuator"]["mass_kg"]) + 0.12 + 0.10, pv[0], pv[2],
        f"{sc['actuator']['model']} {sc['actuator']['mass_kg']} kg datasheet x 2 (8 N m rated, 16 N m peak) driving the "
        f"spindles through {sc['linkage_ratio']:.1f}:1 bellcrank linkages 0.10 + spindle cranks 0.12 (estimate); "
        "hinge-moment check stab_hinge_moments")
    add("actuators_nose_steering_brake_2x_DA26", "controls", 2 * 0.27 + 0.10, 0.5 * (x_pn + gp["x_mg"]), -0.05,
        "Volz DA 26 x 2 (SAGITTA practice: steering + brake master cylinder) + 0.10 installation")
    for k in ("aileron", "flap"):
        add(f"control_surfaces_{k}s_pair", "controls", cs_mass[k], cs[k]["x"], z_wing if k == "aileron" else P["z_root"],
            f"{cs[k]['area']:.3f} m2 x 2 faces x wing-skin sandwich {am['wing_skin']:.2f} kg/m2 x 0.85 + hinges")
    add("control_surfaces_rudders_pair", "controls", cs_mass["rudder"], cs["rudder"]["x"], cs["rudder"]["z"],
        f"{cs['rudder']['area']:.3f} m2 x 2 faces x skin x 0.85 + hinges")
    add("main_gear_legs_wheels_brakes_emas_pair", "gear", 2 * MR["sagitta_main_leg_kg"] * k_ms, gp["x_mg"],
        gp["z_trunnion"], "components.yaml SAGITTA main leg 4.0 kg each (flight version, telescopic damper, EMA "
        "retraction, TOST wheel + brake) x MTOM/150; flight CG = retracted position (wheels flat in the belly wells)")
    add("nose_gear_leg_wheel_steering", "gear", MR["sagitta_nose_leg_kg"] * k_ms, x_pn + 0.6 * LN, z_pn,
        "components.yaml SAGITTA nose leg 3.5 kg x MTOM/150; flight CG = retracted (aft, keel well)")
    add("gear_doors_wells_locks_sensors", "gear", MR["gear_doors_locks_kg"], 0.75 * gp["x_mg"] + 0.25 * x_pn, -0.10,
        "identity study RETRACT items: doors 0.41 + well close-outs 0.36 + cut-out reinforcement 0.50 + locks/sensors 0.25")
    add("avionics", "systems", 0.62, av[0], av[2], "components.yaml avionics.recommended.mass_estimate_kg")
    add("battery_14S2P_lifepo4", "systems", 2.43, av[0] - 0.08, av[2] - 0.01, "baseline.yaml#subsystems.electrical.battery")
    add("pdu_dcdc_fuses", "systems", 2.10, av[0] + 0.10, av[2], "components.yaml electrical: PDU 1.60 + DC-DC 0.10 + "
        "fuses/contactor 0.40")
    add("wiring_harness_connectors_coax", "systems", MR["harness_kg"], 1.9, 0.0,
        "baseline 2.50 kg + 0.20 kg longer wing harness (endurance study); distributed")
    pa = MR["parachute"]
    add(pa["name"], "systems", pa["mass_kg"], pc[0], pc[2], pa["basis"])
    add("flight_termination_lights", "systems", 0.15 + 3 * 0.083, 0.6 * av[0] + 0.4 * x_mac40, 0.02,
        "independent FTS 0.15 + 3 x AveoFlash 0.083 (components.yaml recovery_and_safety)")
    add("turret_lift_mechanism_doors", "systems", MR["turret_mechanism_kg"], tb[0], tb[2] + 0.06,
        "ball-screw linear stage + BLDC/brake 0.45, guide rails/carriage 0.35, two bay doors + linkage 0.30, bay "
        "liner/frame 0.40, controller/sensors 0.10 (estimate, no catalogue unit)")
    keel_L = 0.82 * af.L
    add("keel_beams_longerons", "chassis", 2 * keel_L * 0.20, 0.5 * af.L, -0.05,
        "2 CFRP hat-section keel beams 0.20 kg/m (endurance study)")
    add("frames_bulkheads", "chassis", MR["frames_kg"], 0.47 * af.L, 0.0, "13 sandwich frames/bulkheads (estimate, "
        "endurance study 0.16 kg each, larger lifting-body sections)")
    add("wing_carry_through_box_fittings", "chassis", MR["carry_through_kg"], 0.5 * (x_f0 + x_r0), P["z_root"],
        "spar-cap carry-through box, root fittings, 7075 lugs and Ti pins (estimate, endurance 1.2 kg + gear loads)")
    add("main_gear_frame_trunnions_side_braces", "chassis", MR["main_trunnion_kg"], gp["x_mg"], gp["z_trunnion"],
        "gear frame (sandwich bulkhead pair) + 2 x 7075 trunnion fittings + side-brace/EMA mounts (estimate)")
    add("nose_gear_trunnion_fitting", "chassis", 0.30, x_pn, z_pn, "endurance study")
    add("engine_mount_4130", "chassis", 0.60, xh - 0.28, zt, "endurance study (4130 truss + isolator ring)")
    add("parachute_attach_fitting", "chassis", 0.35, pc[0], pc[2], "endurance study (riser attach fitting on the "
        "parachute-bay frame)")
    add("turret_bay_frame_guides", "chassis", MR["turret_bay_frame_kg"], tb[0], tb[2], "estimate: cut-out frame + "
        "linear-guide mounts around the 0.22 m belly opening")
    add("floors_trays_rails", "chassis", 1.20, 1.4, -0.02, "endurance study")
    add("hatch_frames_quick_access_fasteners", "chassis", MR["hatch_frames_kg"], 1.5, 0.08, "endurance study 0.80 + "
        "belly payload hatch and turret-bay frame lands (estimate)")
    add("fuel_bay_liners_supports", "chassis", 0.45, fuel_x, 0.05, "endurance study 0.40 + saddle cell")
    add("stabilator_spindle_bearing_housings", "chassis", 0.40, pv[0], pv[2], "estimate: 2 bearing housings per "
        "side in a cross-tube through the aft body")
    add("fin_ventral_root_fittings", "chassis", 0.30, fin_pp["x_le_mac"], 0.05, "estimate")
    add("body_skin_sandwich", "shell", shell["skin"], shell["x_c"], shell["z_c"],
        f"exposed body wetted area {shell['S_wet_exposed']:.2f} m2 x secondary sandwich {am['shell']:.2f} kg/m2")
    add("chine_edge_bands", "shell", shell["chine_bands"], 0.45 * af.L, 0.0,
        f"solid-laminate edge band along both chines {shell['chine_length']:.2f} m x "
        f"{S['structures']['chine_edge_band_kg_per_m']} kg/m (identity study)")
    w_fix = wing_m["total"] - cs_mass["aileron"] - cs_mass["flap"]
    add("wing_structure_pair", "wing", w_fix, x_mac40, z_wing,
        "spar caps from the CS-LUAS gust/manoeuvre envelope (structlib), webs, ribs, primary sandwich skins, outer-"
        "panel joints (endurance-study wing model on the actual chord), minus the control surfaces")
    add("stabilators_pair", "tail", tail_m["stabilator"]["total"], st_pp["x_le_mac"] + 0.40 * st_pp["mac"],
        st_pp["z_mac"], "tail sandwich skins on the exposed area + spar + ribs + spindle/root fittings")
    sb_pp = panel_planform(ts["stabilator_stub"]["sections"])
    add("stabilator_root_stubs_pair", "tail", tail_m["stabilator_stub"]["total"], sb_pp["x_le_mac"] + 0.4 * sb_pp["mac"],
        sb_pp["z_mac"], "fixed root stubs (constant 8 mm root gap to the moving stabilators): tail sandwich on the "
        "exposed area + spar + ribs")
    add("fins_pair_fixed", "tail", tail_m["fin"]["total"] - cs_mass["rudder"], fin_pp["x_le_mac"] + 0.35 * fin_pp["mac"],
        fin_pp["z_mac"], "tail sandwich skins + spar + ribs + root fittings, minus the rudders")
    add("ventral_fin_bumper_skid", "tail", tail_m["ventral"]["total"], vf_pp["x_le_mac"] + 0.4 * vf_pp["mac"],
        vf_pp["z_mac"], "tail sandwich + replaceable skid shoe (identity study 0.30 kg fitting + skid)")
    struct = sum(i["mass_base_kg"] for i in I if i["group"] in ("chassis", "shell", "wing", "tail", "gear"))
    add("fasteners_inserts_nutplates", "hardware", MR["hardware_fraction"] * struct, 1.95, 0.0,
        f"{MR['hardware_fraction']:.3f} x structure + gear base mass (estimate)")
    return I


def payload_items(S: dict, lz: dict) -> list:
    PS = S["payload"]
    tb = lz["zones"]["turret_bay"]
    T = PS["turret"]
    R = S["layout"]["rules"]
    pb = np.asarray(lz["zones"]["payload_bay"]["box"], float)
    xb = 0.5 * (pb[0, 0] + pb[1, 0])
    zb = 0.5 * (pb[0, 2] + pb[1, 2])
    xt = 0.5 * (tb["box"][0][0] + tb["box"][1][0])
    mcb = np.asarray(lz["zones"]["mission_computer"]["box"], float)
    mc = 0.5 * (mcb[0] + mcb[1])
    return [{"name": "eo_ir_turret_hd59_mount", "mass_kg": PS["turret_mass_kg"], "x": round(xt, 4), "y": 0.0,
             "z": round(T["ball_center_retracted_z"], 4), "z_extended": round(T["ball_center_extended_z"], 4),
             "basis": "Trillium HD59-LLVV 1.55 kg datasheet + isolator/mount 0.20 kg (baseline.yaml#payload_set)"},
            {"name": "mission_computer_recorder", "mass_kg": PS["mission_computer_kg"], "x": round(mc[0], 4), "y": 0.0,
             "z": round(mc[2], 4), "basis": "baseline.yaml#payload_set"},
            {"name": "payload_tray_harness", "mass_kg": PS["tray_harness_kg"], "x": round(xb, 4), "y": 0.0,
             "z": round(zb, 4), "basis": "baseline.yaml#payload_set.payload_tray_harness_kg"},
            {"name": "research_payload_allowance", "mass_kg": PS["research_allowance_kg"], "x": round(xb, 4), "y": 0.0,
             "z": round(zb, 4), "basis": "baseline.yaml#payload_set.research_payload_allowance_kg (belly bay on the CG)"},
            {"name": "e180_growth_turret_delta", "mass_kg": round(PS["growth_turret_mass_kg"] - PS["turret_mass_kg"], 4),
             "x": round(xt, 4), "y": 0.0, "z": round(T["ball_center_retracted_z"], 4),
             "z_extended": round(T["ball_center_extended_z"], 4),
             "basis": "Octopus E180 class 4.0 kg incl. mount (baseline growth envelope) minus the HD59 set"},
            {"name": "research_payload_reduced_for_e180", "mass_kg": round(-(PS["growth_turret_mass_kg"] -
                                                                           PS["turret_mass_kg"]), 4),
             "x": round(xb, 4), "y": 0.0, "z": round(zb, 4), "basis": "keeps the 20 kg design payload with the E180"}]


def ground_cases(S: dict, gp: dict, lz: dict) -> list:
    """Loading cases with the gear EXTENDED (ground): gear items moved to their extended positions."""
    S2 = copy.deepcopy(S)
    r_t = S["landing_gear"]["tyre"]["diameter"] / 2
    for it in S2["mass"]["items"]:
        if it["name"].startswith("main_gear_legs"):
            it["z"] = round(0.5 * (gp["z_trunnion"] + gp["z_axle"]), 4)
        elif it["name"].startswith("nose_gear_leg"):
            it["x"] = round(gp["x_ng"], 4)
            it["z"] = round(gp["z_axle_nose"] + 0.4 * gp["nose_leg_length"], 4)
    return mass_cases(S2)


def landing_gear_block(S: dict, gp: dict, lz: dict) -> None:
    LG = S["landing_gear"]
    mg, ng = LG["main"], LG["nose"]
    m, n = gp["main"], gp["nose"]
    mg["axle_static"] = _round(m["axle"])
    mg["trunnion"] = _round(m["trunnion"])
    mg["trunnion_axis"] = [1.0, 0.0, 0.0]
    mg["leg_length"] = round(m["leg_length"], 5)
    mg["track"] = round(2 * float(m["axle"][1]), 5)
    mg["retraction"] = {"kind": "inward about the longitudinal trunnion axis (starboard leg; port mirrored)",
                        "angle_deg": round(m["retraction_deg"], 3),
                        "stowed_wheel_center": _round(m["stowed_center"]),
                        "stowed_axle_direction": _round(m["stowed_axle"]),
                        "wheel_attitude_stowed": f"tilted {math.degrees(m['splay']):.0f} deg from flat (= leg splay), "
                                                 "leg horizontal under the wing box, wells side by side"}
    mg["stowed_envelope"] = {"box": _round(m["well_box"]),
                             "note": "bounding box of the tyre envelope (+ well clearance) and the trunnion fitting, "
                                     "starboard well (port mirrored)"}
    ng["axle_static"] = _round(n["axle"])
    ng["pivot"] = _round(n["pivot"])
    ng["pivot_axis"] = [0.0, 1.0, 0.0]
    ng["leg_length"] = round(n["leg_length"], 5)
    ng["retraction"] = {"kind": "aft about the lateral pivot axis", "angle_deg": 90.0,
                        "stowed_wheel_center": _round(n["stowed_center"]),
                        "wheel_attitude_stowed": "vertical (keel slot well)"}
    ng["stowed_envelope"] = {"box": _round(n["well_box"]), "note": "bounding box of the stowed tyre envelope (+ well "
                                                                   "clearance) and the pivot fitting"}
    LG["wheelbase"] = round(gp["x_mg"] - gp["x_ng"], 5)
    LG["ground_z"] = round(gp["z_g"], 5)


AREA_DEADBAND = 0.004            # m2: wing-area updates below this are not applied (closure convergence)
SM_CLOSURE_MARGIN = 0.001        # static-margin closure target above aero.stability_rules.sm_min (convergence band)
TAIL_DEADBAND = 0.004            # tail scale updates below 0.4 % are not applied


def design_closure(S_in: dict, verbose: bool = True, max_iter: int = 30) -> dict:
    """Close the HANCER design point from the spec design rules (fixed MTOM; wing station for the static margin and
    tip-back; gear height for the propeller clearance and bumper rules; fuel = MTOM - empty - payload)."""
    S = copy.deepcopy(S_in)
    mis = S["mission"]
    m0 = float(S["mass"]["mtow_kg"])
    k_sec = float(S["aero"]["corrections"]["cl_max_section_factor"])
    k_3d = float(S["aero"]["corrections"]["k_clmax_3d"])
    h_ref = float(mis["loiter_altitude"])
    V_ref = float(mis["loiter_speed_floor_eas"]) / math.sqrt(AL.isa(h_ref)["sigma"])
    x_cg_guess = S["wing"]["planform"]["x_c4_root"] - 0.05
    gp = None
    hist = []
    eng_, prop_ = make_propulsion(S)
    for it in range(max_iter):
        build_geometry(S)
        af = Airframe(S)
        wa = wing_analysis(S, af, V_ref, h_ref, k_sec, k_3d)
        tp = tail_props(S, V_ref, h_ref, af)
        # stability with the previous CG (first pass: guess)
        llm = lifting_line_moment(S, wa)
        cdp, cm0s = strip_profile_drag(S, wa, S["wing"]["sections"][0]["y"], True)
        if gp is None and not S["mass"].get("items"):
            cases0 = [{"name": "guess", "x": x_cg_guess, "z": 0.0, "m": m0}]
        else:                                   # previous iteration, or the mass items of the spec (re-closure)
            cases0 = mass_cases(S)
        stab = stability(S, af, wa, llm, cm0s, tp, cases0)
        clm = clmax_set(S, wa, stab, min(c["x"] for c in cases0), tp["stabilator"])
        pot = power_on_trim(S, wa, stab, tp["stabilator"], clm, prop_, cases0)
        # gust/manoeuvre limit loads: every mass x altitude with the configuration lift slope; the wing is sized for
        # the largest lift n x m g (MTOM governs: the gust lift increment is nearly mass independent)
        gm = gust_matrix(S, sorted({m0, min(c["m"] for c in cases0)}), lambda m: clm["clean_trimmed"],
                         stab["vlm"]["CL_alpha_all"], float(S["structures"]["VD_eas"]))
        n_lim = max(max(r_["lift_factor_N"] for r_ in gm) / (m0 * G), float(S["structures"]["n_limit_pos"]))
        wing_m = wing_structure(S, af, m0, n_lim)
        tail_m = tail_structure(S, af)
        shell = body_shell(S, af)
        mb = af.body_mesh
        tri = mb.V[mb.F]
        cc = tri.mean(axis=1)
        ar = 0.5 * np.linalg.norm(np.cross(tri[:, 1] - tri[:, 0], tri[:, 2] - tri[:, 0]), axis=1)
        shell["x_c"] = float((ar * cc[:, 0]).sum() / ar.sum())
        shell["z_c"] = float((ar * cc[:, 2]).sum() / ar.sum())
        # gear
        LG = S["landing_gear"]
        if gp is None and S["mass"].get("items") and "trunnion" in LG["main"]:
            gcases0 = ground_cases(S, {"z_trunnion": LG["main"]["trunnion"][2], "z_axle": LG["main"]["axle_static"][2],
                                       "x_ng": LG["nose"]["axle_static"][0], "z_axle_nose": LG["nose"]["axle_static"][2],
                                       "nose_leg_length": LG["nose"]["leg_length"]}, None)
        else:
            gcases0 = cases0 if gp is None else ground_cases(S, gp, None)
        x_mg_prev = None if gp is None else gp["x_mg"]
        if S["layout"]["rules"].get("payload_bay_centre", "empty_cg") == "empty_cg":
            have = gp is not None or bool(S["mass"].get("items"))
            x_pay = round(empty_mass(S)["x"], 4) if have else S["wing"]["planform"]["x_c4_root"] + 0.15
        else:
            x_pay = None
        gp = gear_placement(S, af, wa, clm, gcases0, x_mg_prev, x_pay, pot)
        lz = layout_zones(S, af, gp)
        landing_gear_block(S, gp, lz)
        # ventral fin span from the bumper rule
        vfp = S["tail"]["surfaces"]["ventral"]["params"]
        z_cp = S["tail"]["surfaces"]["ventral"]["bumper"]["contact_point"][2]
        vfp["span"] = round(max(vfp["span"] + (z_cp - gp["z_bumper_required"]), vfp["span_min"]), 5)
        # fuel cells and fuel CG
        cells = fuel_cells(S, af, lz)
        S["layout"]["fuel_cells"] = cells
        S["layout"]["zones_preliminary"] = {k: v for k, v in lz["zones"].items()}
        S["layout"]["firewall_x"] = lz["zones"]["firewall_x"]["x"]
        vols, xs_ = [], []
        for c in cells:
            xx = np.linspace(c["x"][0], c["x"][1], 25)
            A_ = np.array([_area_band(af, x, c["z"][0], c["z"][1], 0.025) for x in xx])
            vols.append(float(np.trapz(A_, xx)))
            xs_.append(float(np.trapz(A_ * xx, xx) / max(np.trapz(A_, xx), 1e-12)))
        fuel_x = float(np.dot(vols, xs_) / sum(vols))
        fuel_z = float(np.mean([0.5 * (c["z"][0] + c["z"][1]) for c in cells]))
        # turret positions
        T = S["payload"]["turret"]
        tg = lz["turret"]
        T["bay_center_x"] = round(tg["x"], 5)
        T["ball_center_retracted_z"] = round(tg["ball_center_retracted_z"], 5)
        T["ball_center_extended_z"] = round(tg["ball_center_extended_z"], 5)
        # masses
        items = mass_items(S, af, wing_m, tail_m, shell, lz, gp, fuel_x, m0)
        S["mass"]["items"] = items
        S["mass"]["payload_items"] = payload_items(S, lz)
        empty = sum(i["mass_kg"] for i in items)
        pay = float(mis["payload_design_kg"])
        fuel = m0 - empty - pay
        S["mass"].update({"empty_kg": round(empty, 4), "fuel_kg": round(fuel, 4), "payload_kg": pay,
                          "fuel_cg": _round([fuel_x, 0.0, fuel_z], 4)})
        cases = mass_cases(S)
        gcases = ground_cases(S, gp, lz)
        stab = stability(S, af, wa, llm, cm0s, tp, cases)
        clm = clmax_set(S, wa, stab, min(c["x"] for c in cases), tp["stabilator"])
        pot = power_on_trim(S, wa, stab, tp["stabilator"], clm, prop_, cases)
        # wing move: static margin and tip-back (the wing, main gear, fuel and carry-through move together)
        movers = ("wing_structure", "wing_carry", "fuel_", "control_surfaces_a", "control_surfaces_f",
                  "actuators_ailerons", "actuators_flaps")
        f_move = (sum(i["mass_kg"] for i in items if i["name"].startswith(movers)) + fuel + pay * 0.88) / m0
        cbar = float(S["wing"]["mac"])
        # aim 0.1 % MAC above the R-09 minimum so the converged point (dead band 0.4 mm) never lands below it
        d_sm = (float(S["aero"]["stability_rules"]["sm_min"]) + SM_CLOSURE_MARGIN - stab["sm_min"]) * cbar / (1.0 - f_move)
        x_aft = max(c["x"] for c in gcases)
        h_cg = max(c["z"] for c in gcases) - gp["z_g"]
        d_tb = (h_cg * math.tan(math.radians(gp["tipback_required_deg"])) - (gp["x_mg"] - x_aft))
        dx = d_sm
        d_mg = 0.0 if x_mg_prev is None else gp["x_mg"] - x_mg_prev
        # wing area from the stall-speed rule (trimmed clean CLmax at the forward CG, MTOM, ISA sea level)
        P_ = S["wing"]["planform"]
        rule = P_.get("area_rule")
        dS = 0.0
        if rule:
            S_new = 2 * m0 * G / (RHO0 * float(rule["stall_speed_target"]) ** 2 * clm["clean_trimmed"])
            dS = S_new - float(P_["area"])
            dS = 0.0 if abs(dS) < AREA_DEADBAND else round(0.6 * dS, 4)        # damped, dead band (convergence)
        hist.append({"it": it, "x_c4_root": S["wing"]["planform"]["x_c4_root"], "sm_min": stab["sm_min"],
                     "x_np": stab["x_np"], "x_cg_aft": max(c["x"] for c in cases), "fuel": fuel, "empty": empty,
                     "x_mg": gp["x_mg"], "z_g": gp["z_g"], "d_sm": d_sm, "d_tb": d_tb, "d_mg": d_mg,
                     "area": float(P_["area"]), "dS": dS, "clean_trimmed": clm["clean_trimmed"],
                     "governing_x_mg": gp["governing_x_mg"], "x_payload": gp["x_payload"]})
        if verbose:
            h = hist[-1]
            print(f"  closure {it}: x_c4 {h['x_c4_root']:.4f} S {h['area']:.4f} (dS {dS:+.4f}) SMmin {h['sm_min']:.3f} "
                  f"NP {h['x_np']:.3f} CGaft {h['x_cg_aft']:.3f} xmg {h['x_mg']:.3f} ({h['governing_x_mg']}) "
                  f"zg {h['z_g']:.3f} empty {empty:.2f} fuel {fuel:.2f} dSM {d_sm*1000:+.1f} mm", flush=True)
        # tail sizing rules: stabilator from the trim limit on the LOCAL stabilator CL at the forward CGs (clean and
        # take-off CLmax power off; lift-off and go-around at full throttle with the thrust-line moment), fins from
        # the directional-stability target (scales chords and span together: aspect ratio kept)
        trs = S["tail"].get("sizing_rules")
        f_h = f_v = 1.0
        if trs:
            clt = max(abs(clm["to_tail_cl"]), abs(clm["clean_tail_cl"]), pot["max_abs_tail_cl_local"])
            f_h = 1.0 + 0.6 * (math.sqrt(clt / float(trs["stabilator_trim_cl_max"])) - 1.0)
            cn = stab["cn_beta"]
            need = (float(trs["cn_beta_target"]) - cn["body_used"] - cn["ventral"]) / max(cn["fins"], 1e-6)
            f_v = 1.0 + 0.6 * (math.sqrt(max(need, 0.25)) - 1.0)
            f_h = 1.0 if abs(f_h - 1.0) < TAIL_DEADBAND else f_h
            f_v = 1.0 if abs(f_v - 1.0) < TAIL_DEADBAND else f_v
        hist[-1].update({"stab_scale": f_h, "fin_scale": f_v})
        if verbose and trs:
            print(f"      tail: stabilator x{f_h:.4f} (tail CL {clt:.3f}), fins x{f_v:.4f} (Cnb {stab['cn_beta']['total']:.4f})",
                  flush=True)
        done = abs(dx) < 4e-4 and abs(d_mg) < 1.5e-3 and dS == 0.0 and f_h == 1.0 and f_v == 1.0
        if done and it > 1:
            break
        S["wing"]["planform"]["x_c4_root"] = round(S["wing"]["planform"]["x_c4_root"] + 0.85 * dx, 5)
        if rule:
            P_["area"] = round(float(P_["area"]) + dS, 4)
        if trs:
            for k_, f_ in (("stabilator", f_h), ("fin", f_v)):
                pp = S["tail"]["surfaces"][k_]["params"]
                for q_ in ("root_chord", "tip_chord", "span"):
                    pp[q_] = round(float(pp[q_]) * f_, 4)
    S["_closure_history"] = hist
    return S


# =====================================================================================================================
# 12. spec-built scene (renders) and the dimensioned 3-view
# =====================================================================================================================
COLORS = {"skin": "#C9CED3", "skin_dark": "#9AA2AA", "accent": "#5D646C", "dark": "#2E3338", "glass": "#1E2228",
          "prop": "#1F2226", "gear": "#4A5057", "tyre": "#16181A", "turret": "#2A2E33", "metal": "#8C939B",
          "ground": "#F2F4F6"}


def _wheel_mesh(center, r, w, axis, n=40):
    from ..core.geom import revolve
    prof = []
    k = 14
    for i in range(k + 1):
        a = -math.pi / 2 + math.pi * i / k
        prof.append((r - 0.5 * w + 0.5 * w * math.cos(a), 0.5 * w * math.sin(a)))
    prof = [(r * 0.45, -0.5 * w)] + prof + [(r * 0.45, 0.5 * w)]
    return revolve(prof, n=n, axis_origin=center, axis=axis)


def _prop_meshes(hub, D, blades=2, phase_deg=18.0, beta_root=48.0, beta_tip=18.0):
    """Pusher propeller (twisted tapered blades in the YZ plane) + spinner pointing aft (render geometry)."""
    from ..core.geom import revolve
    hub = np.asarray(hub, float)
    R = 0.5 * D
    rs = np.linspace(0.075, R, 8)
    out = []
    for k in range(blades):
        secs = []
        for r in rs:
            f = (r - rs[0]) / (R - rs[0])
            beta = beta_root + (beta_tip - beta_root) * f
            c = (0.070 if blades == 2 else 0.060) * (1 - 0.55 * f) + 0.012
            secs.append({"y": 0.0, "x_le": hub[0] - 0.5 * c * math.sin(math.radians(beta)), "z_le": r, "chord": c,
                         "twist_deg": 90.0 - beta, "airfoil": "naca4412", "thickness_scale": 0.9 - 0.5 * f,
                         "span_dir": (0.0, 0.0, 1.0)})
        m = oml.LiftingSurface(secs, n_chord=24, te_thickness=0.0008).mesh()
        ang = math.radians(phase_deg + 360.0 * k / blades)
        m = m.rotated_about((0, 0, 0), (1, 0, 0), ang).translated((0, hub[1], hub[2]))
        out.append(m)
    sp = revolve([(0.0, 0.0), (0.075, 0.0), (0.073, 0.04), (0.058, 0.10), (0.032, 0.15), (0.0, 0.18)], n=48,
                 axis_origin=hub - np.array([0.03, 0, 0]), axis=(1, 0, 0))
    out.append(sp)
    return out


def scene_parts(S: dict, turret: str = "extended", gear: str = "down") -> list:
    """(name, mesh, colour) of the spec OML + propeller + EO/IR turret + landing gear (render geometry only)."""
    from ..core.geom import cylinder, sphere
    af = Airframe(S)
    parts = [("body", af.body_mesh, "skin"), ("wing_r", af.wing_mesh, "skin"),
             ("wing_l", af.wing_mesh.mirrored_y(), "skin")]
    for k, m in af.tail_meshes.items():
        col = "accent" if k != "ventral" else "skin_dark"
        parts.append((f"{k}_r", m, col))
        if af.tail_mirror[k]:
            parts.append((f"{k}_l", m.mirrored_y(), col))
    pr = S["propeller"]
    bm = pr.get("blade_model", {})
    eps = math.radians(float(pr.get("thrust_line_inclination_deg", 0.0)))
    for i, m in enumerate(_prop_meshes(pr["hub"], float(pr["diameter"]), int(pr.get("blades", 2)),
                                       beta_root=float(bm.get("beta_root_deg", 48.0)),
                                       beta_tip=float(bm.get("beta_tip_deg", 18.0)))):
        if eps:                                   # disc axis (cos eps, 0, sin eps): rotation about +y by -eps
            m = m.rotated_about(tuple(pr["hub"]), (0, 1, 0), -eps)
        parts.append((f"prop{i}", m, "prop"))
    T = S["payload"]["turret"]
    if turret == "extended":
        xc, zc = float(T["bay_center_x"]), float(T["ball_center_extended_z"])
        r = float(T["ball_diameter"]) / 2
        z_top = af.z_bot(xc) + 0.03
        parts.append(("turret_stem", cylinder(0.055, (xc, 0, zc + 0.5 * r), (xc, 0, z_top), n=40), "turret"))
        parts.append(("turret_ball", sphere(r, (xc, 0, zc), 40), "turret"))
        win = sphere(1.0, (0, 0, 0), 28).transformed(np.diag([0.012, 0.040, 0.032]), (xc - r + 0.004, 0, zc - 0.01))
        parts.append(("turret_window", win, "glass"))
    LG = S["landing_gear"]
    rt = LG["tyre"]["diameter"] / 2
    tw = LG["tyre"]["width"]
    if gear == "down":
        mg = LG["main"]
        for sgn in (1, -1):
            t = np.array(mg["trunnion"], float) * [1, sgn, 1]
            a = np.array(mg["axle_static"], float) * [1, sgn, 1]
            d = a - t
            top = t + 0.0 * d
            mid = t + 0.62 * d
            parts.append((f"mg_cyl{sgn}", cylinder(0.024, top, mid, n=24), "gear"))
            parts.append((f"mg_rod{sgn}", cylinder(0.017, mid - 0.02 * d, a + np.array([0, -sgn * 0.035, 0.0]), n=24),
                          "metal"))
            parts.append((f"mg_axle{sgn}", cylinder(0.012, a + np.array([0, -sgn * 0.045, 0]),
                                                    a + np.array([0, sgn * 0.005, 0]), n=16), "metal"))
            parts.append((f"mg_wheel{sgn}", _wheel_mesh(a + np.array([0, sgn * 0.0, 0]), rt, tw, (0, 1, 0)), "tyre"))
        ng = LG["nose"]
        p = np.array(ng["pivot"], float)
        a = np.array(ng["axle_static"], float)
        parts.append(("ng_cyl", cylinder(0.020, p, p + 0.6 * (a - p) + np.array([0, 0, 0.0]), n=24), "gear"))
        parts.append(("ng_rod", cylinder(0.014, p + 0.55 * (a - p), a + np.array([0, 0, rt + 0.012]), n=24), "metal"))
        fork = [(a + np.array([0, s * (tw / 2 + 0.010), 0]), a + np.array([0, s * (tw / 2 + 0.010), rt + 0.02]))
                for s in (1, -1)]
        for i, (f0, f1) in enumerate(fork):
            parts.append((f"ng_fork{i}", cylinder(0.008, f0, f1, n=12), "metal"))
        parts.append(("ng_bridge", cylinder(0.010, a + np.array([0, -(tw / 2 + 0.012), rt + 0.02]),
                                            a + np.array([0, (tw / 2 + 0.012), rt + 0.02]), n=12), "metal"))
        parts.append(("ng_wheel", _wheel_mesh(a, rt, tw, (0, 1, 0)), "tyre"))
    return parts


VIEWS = {"iso": ((-1.0, -0.95, 0.62), (0, 0, 1), False), "rear": ((1.0, -0.85, 0.50), (0, 0, 1), False),
         "top": ((0, 0, 1), (-1, 0, 0), True), "side": ((0, -1, 0), (0, 0, 1), True),
         "front": ((-1, 0, 0), (0, 0, 1), True)}


def render_scene(parts: list, out_dir: Path, prefix: str, ground, views=None, res=(1600, 1000),
                 views_def=None) -> dict:
    """Workbench renders with the settings of the v2 direction study (concepts_v2/directions.py render()): outdoor
    studio light, specular, soft shadows, cavity, outline, light ground plane in the perspective and side views."""
    import bpy
    from ..blender import build as B
    views_def = views_def or VIEWS
    B.reset_scene()
    col = B._collection("YK250_HANCER")
    mats = {k: B._material(k, v) for k, v in COLORS.items()}
    for name, m, ck in parts:
        B._mesh_object(name, m.V, m.F, col, mats[ck], smooth_deg=34.0)
    B.setup_workbench(res)
    sc = bpy.context.scene
    sc.view_settings.view_transform = "Standard"
    sc.world.color = (0.93, 0.94, 0.95)
    sh = sc.display.shading
    sh.light = "STUDIO"
    sh.studio_light = "outdoor.sl"
    sh.show_specular_highlight = True
    sh.show_shadows = True
    sh.shadow_intensity = 0.28
    sc.display.light_direction = (-0.45, -0.35, 0.82)
    sh.cavity_ridge_factor = sh.cavity_valley_factor = 1.0
    # ground: (x, z) of the main-wheel contact and the nose-up static attitude (deg); or a plain height
    gx, gz, gth = ground if isinstance(ground, (tuple, list)) else (1.8, float(ground), 0.0)
    bpy.ops.mesh.primitive_plane_add(size=60, location=(gx, 0.0, gz), rotation=(0.0, -math.radians(gth), 0.0))
    ground = bpy.context.active_object
    ground.data.materials.append(mats["ground"])
    out = {}
    out_dir.mkdir(parents=True, exist_ok=True)
    for v in (views or views_def):
        dv, up, ortho = views_def[v]
        p = out_dir / f"{prefix}{v}.png"
        ground.hide_render = True
        ground.hide_set(True)
        B.render_view(v, p, dv, up, ortho, margin=1.05)
        show = v in ("iso", "rear", "side") or v.startswith("iso")
        ground.hide_render = not show
        ground.hide_set(not show)
        sc.render.filepath = str(p)
        bpy.ops.render.render(write_still=True)
        out[v] = str(p)
    return out


# =====================================================================================================================
# 13. evaluation of the spec design point
# =====================================================================================================================
def make_propulsion(S: dict, table_key: str | None = None) -> tuple:
    E = S["engine"]
    eb = E.get("electrical_budget")
    load = (float(eb["continuous_base_W"]) + float(eb["research_payload_allowance_W"])) if eb else \
        float(E["electrical_load_continuous_W"])
    eng = Engine(E, load, float(E["generator"]["efficiency"]))
    P = S["propeller"]
    key = table_key or P["table_ref"]
    rows = ref_get(key)["rows_rpm_thrust_N_torque_Nm_power_W"]
    D = float(P["diameter"]) if table_key is None else float(P["alternatives"][table_key]["diameter"])
    return eng, Prop(rows, D, float(P["k_wot"]), float(P["k_inst"]), eng)


def evaluate(S: dict, verbose: bool = False, table_key: str | None = None, sens: bool = True,
             light: bool = False) -> dict:
    """Full analysis of the spec design point. Returns the results dict (out/sizing.json). ``light`` (trade
    studies): no range solution, payload-endurance sweep, turret field-of-regard ray casting or sensitivities."""
    t0 = time.time()
    mis = S["mission"]
    m0 = float(S["mass"]["mtow_kg"])
    k_sec = float(S["aero"]["corrections"]["cl_max_section_factor"])
    k_trip = float(S["aero"]["corrections"]["cd_factor_tripped"])
    k_3d = float(S["aero"]["corrections"]["k_clmax_3d"])
    h_l = float(mis["loiter_altitude"])
    V_ref = float(mis["loiter_speed_floor_eas"]) / math.sqrt(AL.isa(h_l)["sigma"])
    af = Airframe(S)
    eng, prop = make_propulsion(S, table_key)
    wa = wing_analysis(S, af, V_ref, h_l, k_sec, k_3d)
    tp = tail_props(S, V_ref, h_l, af)
    llm = lifting_line_moment(S, wa)
    y_exp = float(S["wing"]["sections"][0]["y"])
    cdp, cm0s = strip_profile_drag(S, wa, y_exp, True)
    cdp_clean, _ = strip_profile_drag(S, wa, y_exp, False)
    cases = mass_cases(S)
    stab = stability(S, af, wa, llm, cm0s, tp, cases)
    x_fwd = min(c["x"] for c in cases)
    clm = clmax_set(S, wa, stab, x_fwd, tp["stabilator"])
    pot = power_on_trim(S, wa, stab, tp["stabilator"], clm, prop, cases)
    # ground geometry (gear extended)
    LG = S["landing_gear"]
    gp_like = {"z_trunnion": LG["main"]["trunnion"][2], "z_axle": LG["main"]["axle_static"][2],
               "x_ng": LG["nose"]["axle_static"][0], "z_axle_nose": LG["nose"]["axle_static"][2],
               "nose_leg_length": LG["nose"]["leg_length"]}
    gcases = ground_cases(S, gp_like, None)
    gr = ground_geometry(S, af, wa, clm, gcases, float(S["propeller"]["diameter"]), pot)
    # drag items and trimmed polars (design CG = MTOW case)
    cg_d = (cases[0]["x"], cases[0]["z"])
    ffr = 0.33
    polars = {}
    items_all = {}
    CFG = (("clean", {"turret": "retracted", "gear": "up"}), ("loiter", {"turret": "extended", "gear": "up"}),
           ("gear_down", {"turret": "retracted", "gear": "down"}))

    def build_polars(ffr_):
        for name, cfg in CFG:
            items = drag_items(S, af, V_ref, h_l, cfg, ffr_)
            cd_rest = sum(items.values()) / float(S["wing"]["area"])
            polars[name] = trimmed_polar(name, S, wa, cdp, k_trip, stab, cd_rest, cg_d, tp["stabilator"], clm, items)
            items_all[name] = items
    build_polars(ffr)
    k_leak = 1.0 + float(S["aero"]["drag_rules"]["leakage_fraction"])
    st_rules = S["aero"]["stability_rules"]
    m_top = max(c["m"] for c in gcases)                 # the MTOM loading cases (trade studies may shift the fuel)
    g_cases_m0 = [c for c in gcases if abs(c["m"] - m_top) < 0.5]
    gc0 = g_cases_m0[0]
    ground = {"z_g": gr["z_g"], "x_mg": gr["x_mg"], "x_cg_to": gc0["x"], "z_cg_to": gc0["z"],
              "z_t": float(S["propeller"]["hub"][2]), "x_thrust": float(S["propeller"]["plane_x"]),
              "x_ac_wb": stab["x_ac_wb"], "CL_ground_to": gr["CL_ground_to"], "CL_ground_ld": gr["CL_ground_ld"],
              "Cm0_to": stab["Cm0_wb"] - 0.25 * clm["dCL0_to"], "CL0": wa["CL_0"], "dCL0_to": clm["dCL0_to"],
              "CLa": wa["CL_alpha"], "mu": float(mis["rolling_friction"]), "t_rot": float(mis["rotation_time_s"]),
              "stab": {"S_h": tp["stabilator"]["S_exposed"], "x_ac": tp["stabilator"]["x_ac"],
                       "eta": float(st_rules["eta_tail"]), "clt_max": float(st_rules["stabilator_clmax"])}}

    def make_flight():
        return Flight(S, eng, prop, polars, ground, {"Dq": items_all["loiter"]["engine_cooling"], "k_leak": k_leak})
    fl = make_flight()
    fuel = float(S["mass"]["fuel_kg"])
    # mission: iterate the cooling-drag fuel-flow ratio with the loiter fuel flow (endurance study)
    for _ in range(3):
        mis_r = fl.solve_loiter_for_fuel(m0, fuel)
        lo = [l_ for l_ in mis_r["log"] if l_["kind"] == "loiter"]
        ff_l = float(np.mean([l_["ff_kg_h"] for l_ in lo])) if lo else 2.5
        ffr_new = ff_l / float(S["engine"]["cooling"]["fuel_flow_at_max_power_kg_per_h"])
        if abs(ffr_new - ffr) < 0.005:
            break
        ffr = ffr_new
        build_polars(ffr)
        fl = make_flight()
    endurance_h = mis_r["t_air_s"] / 3600.0
    rng = {"range_m": float("nan")} if light else fl.solve_range(m0, fuel)
    perf = performance_block(S, fl, m0, fuel, mis_r, g_cases_m0)
    # 10 h mission closure (sizinglib MassModel): MTOW needed for the requirement, same aircraft scaled
    r10 = fl.solve_loiter_for_endurance(m0, float(mis["endurance_requirement_h"]) * 3600.0)
    payload = float(S["mass"]["payload_kg"])
    empty = float(S["mass"]["empty_kg"])
    struct_names = ("wing", "tail", "shell", "chassis", "gear")
    af_mass = sum(i["mass_kg"] for i in S["mass"]["items"] if i["group"] in struct_names)
    fixed = empty - af_mass
    mm_design = SZ.MassModel(payload, fixed, mis_r["ff"], lambda m: af_mass / m0 * (m / m0) ** 0.0).solve(m0_guess=m0)
    mm_10 = SZ.MassModel(payload, fixed, r10["ff"], lambda m: af_mass / m0).solve(m0_guess=m0)
    # payload-endurance trade (fuel = MTOM - empty - payload, limited by the tank volume)
    pack = packaging(S, af, cases)
    fuel_cap = pack["fuel"]["available_m3"] / (1 + float(S["structures"]["fuel"]["expansion_fraction"])) * \
        float(S["engine"]["fuel"]["density_kg_per_m3"])
    pe = []
    for pl in (() if light else (0.0, 3.25, 10.0, 15.0, 20.0, 25.0)):
        fu = min(m0 - empty - pl, fuel_cap)
        m_to = empty + pl + fu
        r = fl.solve_loiter_for_fuel(m_to, fu)
        pe.append({"payload_kg": pl, "fuel_kg": fu, "takeoff_mass_kg": m_to, "endurance_h": r["t_air_s"] / 3600.0,
                   "fuel_volume_limited": bool(m0 - empty - pl > fuel_cap)})
    VH_level = perf["0"]["V_max_m_s"]
    VC = float(S["structures"]["VC_eas"])
    VD = max(1.25 * VC, VH_level, float(S["structures"]["VD_min_eas"]))
    cla_cfg = stab["vlm"]["CL_alpha_all"]
    vn = vn_summary(S, m0, polars["clean"].clmax["clean_trimmed"], cla_cfg, VD)
    masses = sorted({m0, min(c["m"] for c in cases)})
    gm = gust_matrix(S, masses, lambda m: polars["clean"].clmax["clean_trimmed"], cla_cfg, VD)
    n_wing = max(max(r_["lift_factor_N"] for r_ in gm) / (m0 * G), float(S["structures"]["n_limit_pos"]))
    tur = turret_checks(S, af, fov=not light)
    fb = body_fineness(S, af)
    wing_m = wing_structure(S, af, m0, n_wing)
    tail_m = tail_structure(S, af)
    shell = body_shell(S, af)
    cdiag = {} if light else constraint_block(S, fl, polars, perf, m0, eng)
    pcl = prop_clearances(S, af)
    roots = tail_root_checks(S, af, tp)
    hinge = stab_hinge_moments(S)
    spars = {"main": spar_depth_profile(S, af, S["wing"]["planform"]["main_spar_frac"]),
             "rear": spar_depth_profile(S, af, S["wing"]["planform"]["rear_spar_frac"])}
    sdt = spar_depth_targets(S["wing"]["planform"])
    loi3 = perf[str(int(h_l))]["loiter"]
    elec = electrical_budget(S, loi3["gen_W"])
    sens_d = {}
    if sens and not light:
        for k_, sc in (("bsfc_minus_12pct", 0.88), ("bsfc_plus_12pct", 1.12)):
            eng.bsfc_scale = sc
            sens_d[k_] = fl.solve_loiter_for_fuel(m0, fuel)["t_air_s"] / 3600
        eng.bsfc_scale = 1.0
        for pn in ("clean", "loiter"):
            polars[pn].CDs = polars[pn].CDs * 1.10
        sens_d["cd0_plus_10pct_all_drag"] = fl.solve_loiter_for_fuel(m0, fuel)["t_air_s"] / 3600
        for pn in ("clean", "loiter"):
            polars[pn].CDs = polars[pn].CDs / 1.10
        sens_d["empty_plus_5pct"] = fl.solve_loiter_for_fuel(m0, fuel - 0.05 * empty)["t_air_s"] / 3600
        sens_d["loiter_at_1000m"] = fl.solve_loiter_for_fuel(m0, fuel, h=1000.0)["t_air_s"] / 3600
        sens_d["no_transit_loiter_only"] = fl.solve_loiter_for_fuel(m0, fuel, R_transit=0.0)["t_air_s"] / 3600
        sens_d["turret_retracted_whole_mission"] = fl.solve_loiter_for_fuel(m0, fuel, loiter_pol="clean")["t_air_s"] / 3600
    res = {
        "meta": {"spec": _rel(SPEC.SPEC_PATH), "generated_by": "ucav250/analysis/sizing.py"},
        "geometry": {"wing_area_m2": float(S["wing"]["area"]), "span_m": float(S["wing"]["span"]),
                     "aspect_ratio": float(S["wing"]["aspect_ratio"]), "mac_m": float(S["wing"]["mac"]),
                     "mac_le_x_m": float(S["wing"]["mac_le_x"]), "lifting_line_area_m2": wa["S_LL"],
                     "body_length_m": af.L, "overall_length_m": overall_length(S, af),
                     "body_width_max_m": fb["w_max"], "body_height_max_m": fb["h_max"],
                     "body_fineness": fb["fineness"], "body_volume_m3": fb["volume"],
                     "S_wet_body_exposed_m2": af.body_exposed_area(), "S_wet_wing_exposed_m2": wing_m["S_wet"],
                     "S_wet_tail_m2": {k: v["S_wet"] for k, v in tail_m.items()},
                     "tail_areas_m2": {k: float(v["area"]) for k, v in S["tail"]["surfaces"].items()},
                     "fin_exposed_area_pair_m2": 2 * tp["fin"]["area"], "aft_closure": aft_closure(S, af),
                     "chine_step_max_m": chine_wing_step(S, af)["chine_step_max_m"],
                     "chine_step": chine_wing_step(S, af)},
        "aero": {"CL_alpha_vlm": wa["CL_alpha"], "CL_alpha_lifting_line": wa["CL_alpha_LL"], "CL_0": wa["CL_0"],
                 "CL_alpha_configuration_vlm": cla_cfg,
                 "e_lifting_line": wa["e_inv_LL"], "e_vlm_trefftz": wa["e_vlm"], "k_induced_used": wa["k_i"],
                 "CLmax_wing": wa["CLmax"], "alpha_stall_deg": wa["alpha_stall_deg"], "stall_onset_eta": wa["stall_eta"],
                 "Re_root": wa["Re_root"], "Re_tip": wa["Re_tip"], "neuralfoil_min_confidence": wa["min_confidence"],
                 "clmax": {k: v for k, v in clm.items() if not isinstance(v, dict)}, "flaps": {"takeoff": clm["flap_to"],
                                                                                         "landing": clm["flap_ld"]},
                 "power_on_trim": pot,
                 "polars": {k: {"fit": p.fit, "items_D_over_q_m2": items_all[k],
                                "cd_items": {kk: vv / float(S["wing"]["area"]) for kk, vv in items_all[k].items()},
                                "table": {"CL": p.CLs, "CD": p.CDs, "CL_tail": p.CLt}} for k, p in polars.items()},
                 "profile_drag_tripped_x1p15_at_CL": {str(c): cdp(c) * k_trip for c in (0.4, 0.7, 1.0, 1.2)},
                 "profile_drag_clean_at_CL": {str(c): cdp_clean(c) for c in (0.4, 0.7, 1.0, 1.2)},
                 "cd0": polars["clean"].fit["cd0"], "e": polars["clean"].fit["e"], "k": polars["clean"].fit["k"],
                 "ld_max": polars["clean"].fit["LD_max"], "cd0_loiter_turret_out": polars["loiter"].fit["cd0"],
                 "ld_max_loiter": polars["loiter"].fit["LD_max"], "cooling_fuel_flow_ratio": ffr},
        "stability": {k: v for k, v in stab.items() if k not in ("vlm",)} | {"vlm": stab["vlm"]},
        "mass": {"mtow_kg": m0, "empty_kg": empty, "fuel_kg": fuel, "payload_kg": payload,
                 "groups_kg": empty_mass(S)["groups"], "cases": cases, "ground_cases_gear_down": gcases,
                 "fuel_capacity_kg": fuel_cap, "mtow_margin_to_cap_kg": float(S["mass"]["mtow_cap_kg"]) - m0,
                 "massmodel_design_mission": mm_design, "massmodel_10h_mission": mm_10,
                 "wing_structure": wing_m, "tail_structure": tail_m, "body_shell": shell},
        "ground": gr, "turret": tur, "packaging": pack,
        "loads": {"vn": vn, "VD_m_s": VD, "VC_m_s": VC, "VA_m_s": min(v["VA"] for v in vn.values()),
                  "gust_matrix": gm, "n_limit_wing_design": n_wing, "cl_alpha_used": cla_cfg},
        "performance": perf | {"endurance_h": endurance_h, "loiter_time_h": mis_r["t_loiter_s"] / 3600,
                               "range_km": rng["range_m"] / 1000, "mission_fuel_fraction": mis_r["ff"],
                               "mission_log": mis_r["log"], "payload_endurance": pe,
                               "mtow_for_10h_mission_kg": mm_10["mtow"], "loiter_3000m": loi3},
        "constraint_diagram": cdiag, "sensitivities_endurance_h": sens_d,
        "electrical": elec,
        "propeller": {"table": table_key or S["propeller"]["table_ref"], "static_wot": perf["static_wot"],
                      "static_tip_speed_m_s": perf["static_tip_speed_m_s"], "clearances": pcl},
        "tail_roots": roots, "stabilator_hinge": hinge,
        "spar_depth": {"main": {k: v for k, v in spars["main"].items() if k != "rows"},
                       "rear": {k: v for k, v in spars["rear"].items() if k != "rows"},
                       "required_main_m": sdt["main_required"], "required_rear_m": sdt["rear_required"],
                       "junction_thickness_m": sdt["t_j"], "joint_section_rear_depth_m": sdt["rear_joint_depth"],
                       "rows_main": spars["main"]["rows"], "rows_rear": spars["rear"]["rows"]},
    }
    if verbose:
        print(f"[evaluate] {time.time() - t0:.0f} s")
    return res


def chine_wing_step(S: dict, af: Airframe, n: int = 120) -> dict:
    """LERX/chine blend (F8): along the wing root (stations x where the wing exists at the body chine y = a(x)), the
    height of the chine above the wing upper surface (exposed body side wall between the chine and the wing) and of
    the wing lower surface... max of (chine z - wing upper z); > 0 means a visible step."""
    secs = S["wing"]["sections"]
    ys_s = np.array([s_["y"] for s_ in secs])
    xs = np.linspace(secs[0]["x_le"], secs[0]["x_le"] + secs[0]["chord"], n)
    a, bt, bb, zc, nt, nb = af.sec(xs)
    worst, x_w = -float("inf"), None
    for x, y, z0 in zip(xs, a, zc):
        if y < ys_s[0] or y > ys_s[-1]:
            continue
        j = int(np.clip(np.searchsorted(ys_s, y) - 1, 0, len(ys_s) - 2))
        t = (y - ys_s[j]) / max(ys_s[j + 1] - ys_s[j], 1e-9)
        zu = []
        for s_ in (secs[j], secs[j + 1]):
            xc, yu, _ = oml.resampled(s_["airfoil"], 201, 0.0015 / s_["chord"], float(s_.get("thickness_scale", 1.0)))
            u = (x - s_["x_le"]) / s_["chord"]
            if not 0.0 <= u <= 1.0:
                zu = None
                break
            zu.append(s_["z_le"] + s_["chord"] * float(np.interp(u, xc, yu)) - (x - s_["x_le"]) *
                      math.sin(math.radians(s_["twist_deg"])))
        if not zu:
            continue
        step = float(z0) - ((1 - t) * zu[0] + t * zu[1])
        if step > worst:
            worst, x_w = step, float(x)
    return {"chine_step_max_m": max(worst, 0.0), "x_at_max": x_w}


def overall_length(S: dict, af: Airframe) -> float:
    """Nose tip to the aft end of the propeller spinner / fin tips, whichever is further aft."""
    pr = S["propeller"]
    x_sp = float(pr["plane_x"]) - float(pr["spinner"]["backplate_ahead_of_plane"]) + float(pr["spinner"]["length"])
    x_tail = max(float(np.max(m.V[:, 0])) for m in af.tail_meshes.values())
    return max(x_sp, x_tail)


def performance_block(S: dict, fl: Flight, m0: float, fuel: float, mis_r: dict, g_cases_m0: list) -> dict:
    W = m0 * G
    out = {}
    for h in (0.0, float(S["mission"]["loiter_altitude"])):
        atm = AL.isa(h)
        P = fl.pol["clean"]
        sp = AL.speeds(W, fl.Sw, P.fit["cd0"], P.fit["k"], atm["rho"])
        vmax = AL.max_level_speed(W, fl.Sw, P.fit["cd0"], P.fit["k"], fl.p_avail(h), atm["rho"])
        roc, v_roc = AL.rate_of_climb(W, fl.Sw, P.fit["cd0"], P.fit["k"], fl.p_avail(h), atm["rho"])
        cp = fl.climb_point(W, h)
        bl = fl.best_loiter(W, h)
        blc = fl.best_loiter(W, h, "clean")
        br = fl.best_range(W, h)
        keys_l = ("V", "EAS", "CL", "LD", "P_shaft", "P_total", "eta", "rpm", "power_fraction", "bsfc_g_kWh", "ff_kg_h",
                  "gen_W", "tip_mach")
        out[str(int(h))] = {"VS_clean_m_s": fl.vstall(W, h), "V_min_power_polar_m_s": sp["V_min_power"],
                            "V_LDmax_polar_m_s": sp["V_ld_max"], "V_max_m_s": vmax, "RoC_max_polar_m_s": roc,
                            "V_RoC_m_s": v_roc, "RoC_max_m_s": cp["roc"], "V_climb_m_s": cp["V"],
                            "climb_rpm": cp["rpm"], "climb_tip_mach": cp["tip_mach"],
                            "loiter": {k: bl[k] for k in keys_l}, "loiter_turret_retracted": {k: blc[k] for k in keys_l},
                            "best_range": {k: br[k] for k in ("V", "CL", "LD", "P_shaft", "eta", "rpm", "power_fraction",
                                                              "bsfc_g_kWh", "ff_kg_h")},
                            "power_available_W": fl.eng.P_max * fl.eng.lapse(atm["sigma"])}
    out["ceiling_service_m"] = fl.ceiling(m0)
    out["ceiling_absolute_m"] = fl.ceiling(m0, 0.0)
    out["takeoff_sl_mtow"] = fl.takeoff_cases(m0, 0.0, g_cases_m0)
    out["takeoff_1500m_isa_mtow"] = fl.takeoff_cases(m0, 1500.0, g_cases_m0)
    m_land = m0 - fuel * 0.88
    out["landing_sl_mtow"] = fl.landing(m0)
    out["landing_sl_end_of_mission"] = fl.landing(m_land)
    out["landing_mass_end_of_mission_kg"] = m_land
    out["static_wot"] = fl.prop.wot(0.0, 0.0)
    out["static_tip_speed_m_s"] = math.pi * fl.prop.D * out["static_wot"]["rpm"] / 60
    return out


def constraint_block(S: dict, fl: Flight, polars: dict, perf: dict, m0: float, eng: Engine) -> dict:
    P = polars["clean"]
    a = SZ.Aero(cd0=P.fit["cd0"], k=P.fit["k"], clmax=P.clmax["clean_trimmed"], clmax_to=P.clmax["to_trimmed"],
                clmax_ld=P.clmax["ld_trimmed"], cd0_to=polars["gear_down"].fit["cd0"] + P.clmax["flap_to"]["dCD0"])
    ws = np.linspace(200.0, 900.0, 141)
    mis = S["mission"]
    h_l = float(mis["loiter_altitude"])
    corr = lambda h: AL.power_lapse(AL.isa(h)["sigma"]) / eng.lapse(AL.isa(h)["sigma"])   # noqa: E731
    to = perf["takeoff_sl_mtow"]
    eta_to = to["T_lof_N"] * to["V_lof_m_s"] / math.sqrt(2) / max(fl.prop.wot(to["V_lof_m_s"] / math.sqrt(2), 0)["P_shaft"],
                                                                  1.0)
    eta_cl = fl.prop.wot(perf["0"]["V_climb_m_s"], 0.0)["eta"]
    V_l = perf[str(int(h_l))]["loiter"]["V"]
    roc_req = next((float(r["value"]) for r in S["requirements"] if r["metric"] == "roc_sl_m_s"),
                   float(mis["climb_rate_sl_target"]))
    curves = {
        "takeoff_ground_roll": SZ.pw_takeoff(ws, float(mis["field_roll_max"]), a, eta_to=eta_to,
                                             cl_roll=fl.ground["CL_ground_to"]),
        "climb_sea_level": SZ.pw_climb(ws, roc_req, 0.0, a, eta=eta_cl),
        "climb_sea_level_baseline_goal": SZ.pw_climb(ws, float(mis["climb_rate_sl_target"]), 0.0, a, eta=eta_cl),
        "ceiling": SZ.pw_ceiling(ws, float(mis["service_ceiling"]), a, eta=0.72) * corr(float(mis["service_ceiling"])),
        "cruise_max_band_3000m_75pct": SZ.pw_cruise(ws, float(mis["cruise_speed_band"][1]), h_l, a, eta=0.78,
                                                    throttle=0.75) * corr(h_l),
        "loiter_turn_30deg_3000m": SZ.pw_turn(ws, V_l, h_l, 1 / math.cos(math.radians(30)), a, eta=0.75) * corr(h_l)}
    ws_stall = SZ.ws_stall(float(mis["stall_speed_max"]), P.clmax["clean_trimmed"])
    ws_land = SZ.ws_landing(float(mis["field_roll_max"]), P.clmax["ld_trimmed"])
    dp = SZ.design_point(ws, curves, min(ws_stall, ws_land))
    ws_d = m0 * G / fl.Sw
    return {"ws_grid_Pa": ws, "curves_W_per_N": curves, "ws_stall_Pa": ws_stall, "ws_landing_Pa": ws_land,
            "design_point_min_power": {k: v for k, v in dp.items() if k != "envelope"},
            "pw_available_mcp": eng.P_mcp / (m0 * G), "pw_available_max": eng.P_max / (m0 * G), "ws_design_Pa": ws_d,
            "pw_required_at_design": {k: float(np.interp(ws_d, ws, v)) for k, v in curves.items()},
            "pw_prop_absorbed_wot_climb": fl.prop.wot(perf["0"]["V_climb_m_s"], 0.0)["P_shaft"] / (m0 * G),
            "climb_rate_requirement_m_s": roc_req, "climb_rate_baseline_goal_m_s": float(mis["climb_rate_sl_target"]),
            "eta_takeoff": eta_to, "eta_climb": eta_cl}


# =====================================================================================================================
# 14. metrics, requirements and the derived-value check
# =====================================================================================================================
def metrics(S: dict, R: dict) -> dict:
    """Flat metric table (out/sizing.json -> metrics); the spec requirements refer to these keys."""
    p, g, st, tu, pk, m = (R["performance"], R["ground"], R["stability"], R["turret"], R["packaging"], R["mass"])
    clm = R["aero"]["clmax"]
    zones_failed = [k for k, v in pk.items() if k not in ("fuel", "overlaps") and isinstance(v, dict)
                    and not v.get("fits", True)]
    climb = [s_ for s_ in p["mission_log"] if s_["kind"] == "climb"]
    th_need = max(g["theta_lof_deg"], g["theta_flare_deg"])
    W = S["wing"]
    to = p["takeoff_sl_mtow"]
    pcl = R["propeller"]["clearances"]
    rt = R["tail_roots"]
    hm = R["stabilator_hinge"]
    sd = R["spar_depth"]
    pot = R["aero"]["power_on_trim"]
    return {
        "mtow_kg": m["mtow_kg"], "empty_kg": m["empty_kg"], "fuel_kg": m["fuel_kg"], "payload_kg": m["payload_kg"],
        "mtow_margin_to_cap_kg": m["mtow_margin_to_cap_kg"],
        "fuel_volume_margin": pk["fuel"]["available_m3"] / pk["fuel"]["required_m3"] - 1.0,
        "endurance_h": p["endurance_h"], "loiter_time_h": p["loiter_time_h"], "range_km": p["range_km"],
        "ceiling_service_m": p["ceiling_service_m"], "roc_sl_m_s": p["0"]["RoC_max_m_s"],
        "roc_3000m_m_s": p["3000"]["RoC_max_m_s"],
        "climb_time_to_loiter_altitude_min": (climb[0]["dt_s"] / 60.0) if climb else None,
        "takeoff_ground_roll_m": to["ground_roll_m"], "takeoff_distance_15m_m": to["distance_15m_m"],
        "takeoff_rotation_speed_m_s": to["V_R_m_s"], "takeoff_liftoff_speed_m_s": to["V_lof_m_s"],
        "takeoff_main_gear_load_at_rotation_N": min(c["main_gear_load_at_VR_N"] for c in to["cases"].values()),
        "landing_ground_roll_m": p["landing_sl_end_of_mission"]["ground_roll_m"],
        "vs_clean_sl_mtow_m_s": p["0"]["VS_clean_m_s"], "v_max_sl_m_s": p["0"]["V_max_m_s"],
        "loiter_eas_m_s": p["loiter_3000m"]["EAS"], "loiter_tas_3000m_m_s": p["loiter_3000m"]["V"],
        "ld_max": R["aero"]["ld_max"], "cd0": R["aero"]["cd0"], "clmax_clean_trimmed": clm["clean_trimmed"],
        "static_margin_min": st["sm_min"], "static_margin_max": st["sm_max"],
        "cn_beta_per_rad": st["cn_beta"]["total"], "cl_beta_per_rad": st["cl_beta"]["total"],
        "stab_trim_cl_local_max": max(abs(clm["clean_tail_cl"]), abs(clm["to_tail_cl"]), pot["max_abs_tail_cl_local"]),
        "tipback_deg": g["tipback_deg"], "turnover_deg": g["turnover_deg"],
        "nose_load_aft_cg": g["nose_load_aft_cg"], "nose_load_fwd_cg": g["nose_load_fwd_cg"],
        "prop_clear_static_m": g["prop_clear_static"], "prop_clear_min_925a_m": g["prop_clear_min_925a"],
        "prop_clear_flat_tyre_m": g["prop_clear_flat_tyre_bottomed"],
        "prop_clear_radial_m": pcl["radial_min_m"], "prop_clear_longitudinal_m": pcl["longitudinal_min_m"],
        "prop_guard_fin_crossing_margin_m": pcl["fin_guard_margin_over_tip_m"],
        "bumper_before_prop_margin_deg": g["prop_strike_deg"] - g["bumper_contact_deg"],
        "bumper_rotation_margin_deg": g["bumper_contact_deg"] - th_need,
        "touchdown_attitude_deg": g["theta_td_deg"],
        "main_gear_stowed": 1.0 if g["stowed"]["main"]["ok"] else 0.0,
        "nose_gear_stowed": 1.0 if g["stowed"]["nose"]["ok"] else 0.0,
        "main_wells_clear_of_wing_box_m": g["stowed"]["main"]["clear_of_wing_box_m"],
        "turret_flush_margin_m": tu["flush_margin_m"],
        "turret_growth_inside": 1.0 if tu["growth_envelope_inside_retracted"] else 0.0,
        "turret_fov_upper_min_deg": tu["fov_upper_min_deg"],
        "packaging_zones_failed": float(len(zones_failed)), "zone_overlaps": float(len(pk["overlaps"]["pairs"])),
        "span_m": float(W["span"]), "outer_panel_length_m": float(W["span"]) / 2 - float(W["planform"]["y_junction"]),
        "centre_section_width_m": 2 * float(W["planform"]["y_junction"]),
        "flap_inboard_end_outboard_of_joint_m": float(W["controls"]["flap"]["eta0"]) * float(W["span"]) / 2 -
        float(W["planform"]["y_junction"]),
        "prop_tip_mach_static": R["propeller"]["static_wot"]["tip_mach"], "prop_tip_mach_climb": p["0"]["climb_tip_mach"],
        "generator_margin_loiter": R["electrical"]["margin_continuous"],
        "n_limit_pos": max(v["n_limit_pos"] for v in R["loads"]["vn"].values()),
        "n_limit_wing_design": R["loads"]["n_limit_wing_design"],
        "cg_range_mac": st["sm_max"] - st["sm_min"],
        "stab_root_gap_m": rt["stab_root_gap_m"], "stab_root_body_clearance_m": rt["stab_root_body_clearance_m"],
        "fin_root_max_gap_m": rt["fin_root_max_gap_m"],
        "stab_hinge_peak_margin": hm["peak_margin"], "stab_hinge_rated_margin": hm["rated_margin"],
        "stab_surface_stable_about_spindle": 1.0 if hm["statically_stable_surface"] else 0.0,
        "spar_depth_main_min_m": sd["main"]["min_depth_m"], "spar_depth_rear_min_m": sd["rear"]["min_depth_m"],
        "spar_depth_main_ratio": sd["main"]["min_depth_m"] / sd["required_main_m"],
        "spar_depth_rear_ratio": sd["rear"]["min_depth_m"] / sd["required_rear_m"],
        "wing_root_chine_step_m": R["geometry"].get("chine_step_max_m", 0.0),
    }


OPS = {"<=": lambda a, b: a <= b, ">=": lambda a, b: a >= b, "<": lambda a, b: a < b, ">": lambda a, b: a > b,
       "==": lambda a, b: abs(a - b) < 1e-9}


def evaluate_requirements(S: dict, M: dict) -> list:
    out = []
    for r in S["requirements"]:
        v = M.get(r["metric"])
        ok = v is not None and OPS[r["op"]](float(v), float(r["value"]))
        out.append({"id": r["id"], "metric": r["metric"], "op": r["op"], "value": r["value"], "unit": r.get("unit", ""),
                    "actual": v, "pass": bool(ok), "text_tr": r["text_tr"]})
    return out


def _cmp(a, b, tol: float, path: str, bad: list, n: list):
    """Recursive comparison of a spec value with the recomputed one (numbers within ``tol``)."""
    if isinstance(a, dict) and isinstance(b, dict):
        for k in b:
            if k in a:
                _cmp(a[k], b[k], tol, f"{path}.{k}", bad, n)
            else:
                bad.append({"path": f"{path}.{k}", "spec": None, "computed": py(b[k]), "issue": "missing in spec"})
        return
    if isinstance(a, (list, tuple)) and isinstance(b, (list, tuple)):
        if len(a) != len(b):
            bad.append({"path": path, "spec": f"len {len(a)}", "computed": f"len {len(b)}", "issue": "length"})
            return
        for i, (x, y) in enumerate(zip(a, b)):
            _cmp(x, y, tol, f"{path}[{i}]", bad, n)
        return
    if isinstance(b, bool) or isinstance(a, bool):
        n[0] += 1
        if bool(a) != bool(b):
            bad.append({"path": path, "spec": a, "computed": b, "issue": "differs"})
        return
    if isinstance(b, (int, float, np.floating, np.integer)) and isinstance(a, (int, float)):
        n[0] += 1
        if abs(float(a) - float(b)) > tol:
            bad.append({"path": path, "spec": a, "computed": py(b), "issue": f"|diff| {abs(float(a) - float(b)):.4g} > {tol}"})
        return
    if isinstance(b, str) and isinstance(a, str):
        return
    if a != b and not (a is None and b is None):
        n[0] += 1
        if str(a) != str(b):
            bad.append({"path": path, "spec": str(a)[:60], "computed": str(b)[:60], "issue": "differs"})


# derived spec values recomputed by the design closure: (dotted path, absolute tolerance)
DERIVED_CLOSURE = [
    ("wing.area", 0.004), ("wing.aspect_ratio", 0.03), ("wing.mac", 0.002), ("wing.mac_le_x", 0.003), ("wing.mac_y", 0.003),
    ("wing.planform.x_c4_root", 0.003), ("wing.sections", 0.003), ("wing.planform_derived.apex", 0.003),
    ("wing.planform_derived.junction", 0.003), ("fuselage.stations", 0.0015),
    ("tail.surfaces.stabilator.sections", 0.004), ("tail.surfaces.fin.sections", 0.004),
    ("tail.surfaces.ventral.sections", 0.004), ("tail.surfaces.stabilator.area", 0.005), ("tail.surfaces.fin.area", 0.006),
    ("tail.surfaces.ventral.area", 0.003), ("tail.surfaces.ventral.bumper.contact_point", 0.004),
    ("landing_gear.main.axle_static", 0.004), ("landing_gear.main.trunnion", 0.004), ("landing_gear.main.leg_length", 0.004),
    ("landing_gear.nose.axle_static", 0.004), ("landing_gear.nose.pivot", 0.004), ("landing_gear.ground_z", 0.004),
    ("payload.turret.ball_center_retracted_z", 0.003), ("payload.turret.ball_center_extended_z", 0.003),
    ("mass.empty_kg", 0.15), ("mass.fuel_kg", 0.15), ("mass.fuel_cg", 0.01), ("layout.fuel_cells", 0.006),
    ("tail.surfaces.stabilator_stub.sections", 0.004), ("tail.surfaces.stabilator_stub.params", 0.004),
    ("tail.surfaces.stabilator.pivot", 0.004), ("tail.surfaces.stabilator.params.y_root", 0.003),
    ("tail.surfaces.stabilator.params.z_root", 0.003), ("tail.surfaces.fin.params.x_le_root", 0.004),
    ("tail.surfaces.fin.params.z_root", 0.004), ("tail.surfaces.fin.params.guard_radius", 0.002),
    ("tail.surfaces.fin.params.te_crossing_span_fraction", 0.01), ("tail.surfaces.ventral.params.span", 0.004),
    ("tail.surfaces.ventral.params.z_root", 0.003), ("propeller.plane_x", 0.002), ("propeller.hub", 0.002),
    ("fuselage.lines_derived", 0.003), ("wing.planform_derived.y_strake_end", 0.003),
    ("landing_gear.main.retraction.stowed_wheel_center", 0.004), ("landing_gear.nose.retraction.stowed_wheel_center",
                                                                   0.004),
    ("layout.firewall_x", 0.003),
]


def _get(d: dict, dotted: str):
    cur = d
    for k in dotted.split("."):
        cur = cur[k]
    return cur


def check_derived(S: dict, R: dict, verbose: bool = True) -> dict:
    """Re-close the design from the spec design inputs and compare every derived spec value (DERIVED_CLOSURE, mass
    items) with the recomputed one; compare the spec reference copies (aero/stability/performance/landing_gear
    checks) with the evaluation ``R``."""
    D = design_closure(S, verbose=False)
    bad, n = [], [0]
    for path, tol in DERIVED_CLOSURE:
        try:
            a = _get(S, path)
        except KeyError:
            bad.append({"path": path, "spec": None, "computed": "?", "issue": "missing in spec"})
            continue
        _cmp(a, _get(D, path), tol, path, bad, n)
    items_s = {i["name"]: i for i in S["mass"]["items"]}
    for i in D["mass"]["items"]:
        s_ = items_s.get(i["name"])
        if s_ is None:
            bad.append({"path": f"mass.items[{i['name']}]", "spec": None, "computed": i["mass_kg"], "issue": "missing"})
            continue
        _cmp(s_["mass_kg"], i["mass_kg"], 0.06, f"mass.items[{i['name']}].mass_kg", bad, n)
        for k in ("x", "y", "z"):
            _cmp(s_[k], i[k], 0.012, f"mass.items[{i['name']}].{k}", bad, n)
    if len(items_s) != len(D["mass"]["items"]):
        bad.append({"path": "mass.items", "spec": len(items_s), "computed": len(D["mass"]["items"]), "issue": "count"})
    # reference copies vs the evaluation
    ref = reference_blocks(S, R)
    for blk, tol in (("aero", 0.004), ("stability", 0.004), ("performance", None), ("aero_top", 0.004)):
        if blk == "aero_top":
            cur = S.get("aero", {})
        else:
            cur = S.get(blk, {}).get("computed" if blk == "stability" else "reference", {})
        for k, v in ref[blk].items():
            if k not in cur:
                bad.append({"path": f"{blk}.{k}", "spec": None, "computed": py(v), "issue": "missing in spec"})
                continue
            t = tol if tol is not None else PERF_TOL.get(k, 0.01 * max(abs(float(v)), 1.0))
            _cmp(cur[k], v, t, f"{blk}.{k}", bad, n)
    budget = S["mass"].get("budget", {})
    groups = empty_mass(S)["groups"]
    for gname, b in budget.items():
        n[0] += 1
        if abs(groups.get(gname, 0.0) - float(b["target_kg"])) > float(b["tol_kg"]):
            bad.append({"path": f"mass.budget.{gname}", "spec": b["target_kg"], "computed": py(groups.get(gname, 0.0)),
                        "issue": f"outside +/-{b['tol_kg']} kg"})
    if verbose:
        for b in bad[:40]:
            print(f"  [derived] {b['path']}: spec {b['spec']} vs computed {b['computed']} ({b['issue']})")
    return {"n_compared": n[0], "n_bad": len(bad), "bad": bad, "closure_history": D.get("_closure_history", [])}


PERF_TOL = {"endurance_h": 0.05, "loiter_time_h": 0.05, "range_km": 8.0, "ceiling_service_m": 60.0, "roc_sl_m_s": 0.05,
            "roc_3000m_m_s": 0.05, "takeoff_ground_roll_m": 3.0, "landing_ground_roll_m": 3.0, "vs_clean_sl_mtow_m_s": 0.05,
            "v_max_sl_m_s": 0.3, "loiter_tas_3000m_m_s": 0.2, "loiter_eas_m_s": 0.2, "loiter_fuel_flow_kg_h": 0.02,
            "takeoff_distance_15m_m": 5.0, "climb_time_to_loiter_altitude_min": 0.3, "mtow_for_10h_mission_kg": 0.5,
            "loiter_power_W": 40.0, "loiter_rpm": 30.0, "takeoff_rotation_speed_m_s": 0.1,
            "takeoff_liftoff_speed_m_s": 0.1, "takeoff_main_gear_load_at_rotation_N": 8.0, "prop_clear_min_925a_m": 0.002,
            "prop_clear_radial_m": 0.003, "prop_clear_longitudinal_m": 0.002, "generator_margin_loiter": 0.01,
            "stab_trim_cl_local_max": 0.006, "stab_hinge_peak_margin": 0.02}


def reference_blocks(S: dict, R: dict) -> dict:
    """The reference copies the spec carries (aero.reference, stability.computed, performance.reference)."""
    a, st, p = R["aero"], R["stability"], R["performance"]
    M = metrics(S, R)
    aero = {"cd0_clean": a["cd0"], "k_clean": a["k"], "e_clean": a["e"], "ld_max_clean": a["ld_max"],
            "cd0_loiter_turret_extended": a["cd0_loiter_turret_out"], "ld_max_loiter": a["ld_max_loiter"],
            "cd0_gear_down": a["polars"]["gear_down"]["fit"]["cd0"], "CL_alpha_per_rad": a["CL_alpha_vlm"],
            "CL_0": a["CL_0"], "clmax_wing": a["CLmax_wing"], "clmax_clean_trimmed": a["clmax"]["clean_trimmed"],
            "clmax_to_trimmed": a["clmax"]["to_trimmed"], "clmax_ld_trimmed": a["clmax"]["ld_trimmed"]}
    stab = {"np_x": st["x_np"], "np_x_classic": st["x_np_classic"], "np_x_vlm": st["x_np_vlm"],
            "x_ac_wing_body": st["x_ac_wb"], "Cm0_wing_body": st["Cm0_wb"], "volume_h": st["V_H"], "volume_v": st["V_V"],
            "cn_beta_per_rad": st["cn_beta"]["total"], "static_margin_min": st["sm_min"], "static_margin_max": st["sm_max"],
            "downwash_gradient": st["deps"]}
    perf = {k: M[k] for k in ("endurance_h", "loiter_time_h", "range_km", "ceiling_service_m", "roc_sl_m_s",
                              "roc_3000m_m_s", "takeoff_ground_roll_m", "takeoff_distance_15m_m",
                              "landing_ground_roll_m", "vs_clean_sl_mtow_m_s", "v_max_sl_m_s", "loiter_tas_3000m_m_s",
                              "loiter_eas_m_s", "climb_time_to_loiter_altitude_min")}
    perf["loiter_fuel_flow_kg_h"] = p["loiter_3000m"]["ff_kg_h"]
    perf["loiter_power_W"] = p["loiter_3000m"]["P_total"]
    perf["loiter_rpm"] = p["loiter_3000m"]["rpm"]
    perf["mtow_for_10h_mission_kg"] = p["mtow_for_10h_mission_kg"]
    for k in ("takeoff_rotation_speed_m_s", "takeoff_liftoff_speed_m_s", "takeoff_main_gear_load_at_rotation_N",
              "prop_clear_min_925a_m", "prop_clear_radial_m", "prop_clear_longitudinal_m", "generator_margin_loiter",
              "stab_trim_cl_local_max", "stab_hinge_peak_margin"):
        perf[k] = M[k]
    # ARCHITECTURE.md §9 top-level aero keys (F15): drag polar fit, CLmax set, polar tables in out/sizing.json
    aero_top = {"cd0": a["cd0"], "e": a["e"], "k": a["k"], "ld_max": a["ld_max"],
                "clmax_clean": a["clmax"]["clean_trimmed"], "clmax_to": a["clmax"]["to_trimmed"],
                "clmax_ld": a["clmax"]["ld_trimmed"]}
    return {"aero": aero, "stability": stab, "performance": perf, "aero_top": aero_top}


# =====================================================================================================================
# 15. figures (docs/fig) and the dimensioned 3-view
# =====================================================================================================================
def _plt():
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    plt.rcParams.update({"font.size": 9, "axes.grid": True, "grid.alpha": 0.3, "figure.dpi": 140,
                         "axes.spines.top": False, "axes.spines.right": False})
    return plt


def fig_constraint(S: dict, R: dict, path: Path) -> None:
    plt = _plt()
    c = R["constraint_diagram"]
    ws = np.asarray(c["ws_grid_Pa"]) / G
    fig, ax = plt.subplots(figsize=(7.2, 4.6))
    rr = c.get("climb_rate_requirement_m_s", 4.0)
    rg = c.get("climb_rate_baseline_goal_m_s", 4.9)
    names = {"takeoff_ground_roll": "Kalkış koşusu ≤ 200 m",
             "climb_sea_level": f"Tırmanma DS ≥ {rr:.1f} m/s (R-05)".replace(".", ","),
             "climb_sea_level_baseline_goal": f"Tırmanma DS {rg:.1f} m/s (temel hedef)".replace(".", ","),
             "ceiling": "Servis tavanı 4500 m", "cruise_max_band_3000m_75pct": "Seyir 36 m/s, 3000 m (%75)",
             "loiter_turn_30deg_3000m": "Bekleme dönüşü 30°, 3000 m"}
    for k, v in c["curves_W_per_N"].items():
        ax.plot(ws, np.asarray(v), lw=1.6 if k != "climb_sea_level_baseline_goal" else 1.0,
                ls="-" if k != "climb_sea_level_baseline_goal" else "--", label=names.get(k, k))
    ax.axvline(c["ws_stall_Pa"] / G, color="#111827", ls="--", lw=1.2, label="VS ≤ 24 m/s (temiz, trimli)")
    ax.axvline(c["ws_landing_Pa"] / G, color="#6B7280", ls=":", lw=1.2, label="İniş koşusu ≤ 200 m")
    ax.axhline(c["pw_available_max"], color="#B91C1C", lw=0.9, ls=":", label="L 275 EF azami mil gücü (18 kW)")
    ax.axhline(c["pw_prop_absorbed_wot_climb"], color="#111827", lw=1.6,
               label="kullanılabilir: sabit hatveli pervanenin tam gazda\nemdiği mil gücü (tırmanma hızında)")
    # design point = (design wing loading, usable power loading); the curves are the simplified sizing equations
    ax.plot([c["ws_design_Pa"] / G], [c["pw_prop_absorbed_wot_climb"]], "o", color="#B91C1C", ms=6, zorder=5)
    ax.annotate("HANÇER", (c["ws_design_Pa"] / G, c["pw_prop_absorbed_wot_climb"]), xytext=(6, 6),
                textcoords="offset points", color="#B91C1C", fontweight="bold")
    ax.axvline(c["ws_design_Pa"] / G, color="#B91C1C", lw=0.8, alpha=0.5)
    p0 = R["performance"]
    roc_d = f"{p0['0']['RoC_max_m_s']:.2f}".replace(".", ",")
    ax.text(0.99, 0.02, ("Eğriler basitleştirilmiş boyutlandırma denklemleridir (sabit η, temiz polar).\n"
                         "Ayrıntılı model (pervane haritası, tam gazda soğutma sürüklemesi, aşağı itki): "
                         f"DS tırmanma {roc_d} m/s, kalkış koşusu {p0['takeoff_sl_mtow']['ground_roll_m']:.0f} m."),
            transform=ax.transAxes, ha="right", va="bottom", fontsize=6.5, color="#4B5563")
    ax.set_xlabel("Kanat yüklemesi W/S (kg/m²)")
    ax.set_ylabel("Güç yüklemesi P/W (W/N)")
    ax.set_ylim(0, 22)
    ax.set_xlim(ws[0], ws[-1])
    ax.legend(fontsize=7, loc="upper left", ncol=2)
    ax.set_title("YK-250 HANÇER — kısıt diyagramı (MTOM, ISA)")
    fig.tight_layout()
    fig.savefig(path)
    plt.close(fig)


def fig_polars(S: dict, R: dict, path: Path) -> None:
    plt = _plt()
    fig, (a1, a2) = plt.subplots(1, 2, figsize=(9.0, 3.8))
    lab = {"clean": "temiz (takım ve taret içeride)", "loiter": "bekleme (taret dışarıda)",
           "gear_down": "takım açık (kalkış/iniş)"}
    for k, p in R["aero"]["polars"].items():
        CL, CD = np.asarray(p["table"]["CL"]), np.asarray(p["table"]["CD"])
        m = (CL > 0.1) & (CL < 1.6)
        a1.plot(CD[m], CL[m], lw=1.5, label=lab.get(k, k))
        a2.plot(CL[m], CL[m] / CD[m], lw=1.5, label=lab.get(k, k))
    a1.set_xlabel("CD (trimli, S_ref)")
    a1.set_ylabel("CL")
    a2.set_xlabel("CL")
    a2.set_ylabel("L/D")
    loi = R["performance"]["loiter_3000m"]
    a2.axvline(loi["CL"], color="#6B7280", ls=":", lw=1.0)
    a2.annotate(f"bekleme CL {loi['CL']:.2f}", (loi["CL"], 6), fontsize=7)
    a1.legend(fontsize=7)
    fig.suptitle("Trimli sürükleme polarları (geçiş x/c 0,075'e zorlanmış × 1,15)", fontsize=9)
    fig.tight_layout()
    fig.savefig(path)
    plt.close(fig)


CASE_TR = {"mtow_design_payload_turret_retracted": "MTOM, tasarım yükü", "mtow_design_payload_turret_extended":
           "MTOM, taret dışarıda", "full_fuel_baseline_sensors_only": "tam yakıt, temel sensörler",
           "zero_fuel_design_payload": "yakıtsız, tasarım yükü", "reserve_fuel_design_payload": "yedek yakıt, tasarım yükü",
           "minimum_flying_turret_only": "asgari uçuş (yalnız taret)", "e180_growth_turret_full_fuel": "E180 taret, tam yakıt"}


def fig_cg(S: dict, R: dict, path: Path) -> None:
    plt = _plt()
    st = R["stability"]
    cb, le = float(S["wing"]["mac"]), float(S["wing"]["mac_le_x"])
    pct = lambda x: (x - le) / cb * 100                                  # noqa: E731
    fig, ax = plt.subplots(figsize=(7.4, 4.4))
    cases = R["mass"]["cases"]
    xs = [pct(c["x"]) for c in cases]
    for i, c in enumerate(cases):
        ax.plot(pct(c["x"]), c["m"], "o", ms=6, color=f"C{i}", label=CASE_TR.get(c["name"], c["name"]))
    np_ = pct(st["x_np"])
    smin = float(S["aero"]["stability_rules"]["sm_min"]) * 100
    ax.axvline(np_, color="#B91C1C", lw=1.5)
    ax.axvline(np_ - smin, color="#B91C1C", ls="--", lw=1.1)
    y_txt = min(c["m"] for c in cases) + 1.0
    ax.text(np_ + 0.4, y_txt, f"nötr nokta %{np_:.1f}", color="#B91C1C", fontsize=8, rotation=90, va="bottom")
    ax.text(np_ - smin + 0.4, y_txt, f"arka sınır (SM %{smin:.0f})", color="#B91C1C", fontsize=8, rotation=90,
            va="bottom")
    ax.set_xlim(min(xs) - 6, np_ + 6)
    ax.set_xlabel(f"Ağırlık merkezi (% referans OAK; OAK ön kenarı x = {le:.3f} m)")
    ax.set_ylabel("Kütle (kg)")
    ax.legend(fontsize=7, loc="lower left")
    ax.set_title("Ağırlık merkezi zarfı ve statik marj (uçuş: takım ve taret içeride)")
    ax.text(0.99, 0.02, f"ana takım: %{pct(R['ground']['x_mg']):.0f} OAK (zemin)", transform=ax.transAxes, ha="right",
            fontsize=7, color="#374151")
    fig.tight_layout()
    fig.savefig(path)
    plt.close(fig)


def fig_payload_endurance(S: dict, R: dict, path: Path) -> None:
    plt = _plt()
    pe = R["performance"]["payload_endurance"]
    fig, ax = plt.subplots(figsize=(6.0, 3.8))
    ax.plot([r["payload_kg"] for r in pe], [r["endurance_h"] for r in pe], "o-", color="#1F4E79", lw=1.6)
    for r in pe:
        if r["fuel_volume_limited"]:
            ax.annotate("yakıt hacmi sınırı", (r["payload_kg"], r["endurance_h"]), fontsize=7, xytext=(4, 4),
                        textcoords="offset points")
    ax.axhline(float(S["mission"]["endurance_requirement_h"]), color="#B91C1C", ls="--", lw=1.0, label="Gereksinim")
    ax.axvline(float(S["mission"]["payload_design_kg"]), color="#6B7280", ls=":", lw=1.0, label="Tasarım faydalı yükü")
    ax.set_xlabel("Faydalı yük (kg)")
    ax.set_ylabel("Dayanım (h, tasarım görevi)")
    ax.legend(fontsize=7)
    ax.set_title("Faydalı yük – dayanım (MTOM sabit, yakıt = MTOM − boş − yük)")
    fig.tight_layout()
    fig.savefig(path)
    plt.close(fig)


def fig_vn(S: dict, R: dict, path: Path) -> None:
    """V-n diagram in EAS: manoeuvre envelope (stall boundaries from the trimmed CLmax and CLmin at MTOM, +n/-n limits,
    -n kept to VD as in the loads) and the gust lines at sea level and the service ceiling."""
    plt = _plt()
    vn = R["loads"]["vn"]
    STR = S["structures"]
    npos, nneg = float(STR["n_limit_pos"]), float(STR["n_limit_neg"])
    ws = float(S["mass"]["mtow_kg"]) * G / float(S["wing"]["area"])
    VS = math.sqrt(2 * ws / (RHO0 * R["aero"]["clmax"]["clean_trimmed"]))
    VSn = math.sqrt(2 * ws / (RHO0 * abs(float(STR["clmin_negative"]))))
    VC, VD = R["loads"]["VC_m_s"], R["loads"]["VD_m_s"]
    V = np.linspace(0.0, VD, 300)
    fig, ax = plt.subplots(figsize=(6.8, 4.2))
    ax.plot(V, np.minimum((V / VS) ** 2, npos), color="#1F4E79", lw=1.6, label="manevra zarfı")
    ax.plot(V, np.maximum(-(V / VSn) ** 2, nneg), color="#1F4E79", lw=1.6)
    ax.plot([VD, VD], [nneg, npos], color="#1F4E79", lw=1.6)
    for i, (h, v) in enumerate(sorted(vn.items(), key=lambda t: float(t[0]))):
        c = f"C{i + 1}"
        ax.plot([0, VC, VD], [1, v["n_gust_VC"][0], v["n_gust_VD"][0]], ls="--", lw=1.0, color=c,
                label=f"rüzgâr hamlesi MTOM, {int(float(h))} m")
        ax.plot([0, VC, VD], [1, v["n_gust_VC"][1], v["n_gust_VD"][1]], ls="--", lw=1.0, color=c)
    gm = R["loads"].get("gust_matrix", [])
    if gm:
        m_lo = min(r_["mass_kg"] for r_ in gm)
        r_lo = max((r_ for r_ in gm if r_["mass_kg"] == m_lo), key=lambda r_: r_["n_gust_VC"][0])
        ax.plot([0, VC, VD], [1, r_lo["n_gust_VC"][0], r_lo["n_gust_VD"][0]], ls="-.", lw=1.0, color="#7C3AED",
                label=f"rüzgâr hamlesi {m_lo:.1f} kg (en hafif), {int(r_lo['altitude_m'])} m".replace(".", ","))
        ax.plot([0, VC, VD], [1, r_lo["n_gust_VC"][1], r_lo["n_gust_VD"][1]], ls="-.", lw=1.0, color="#7C3AED")
    VA = R["loads"]["VA_m_s"]
    marks = [(VS, "VS"), (VC, "VC = VA" if abs(VA - VC) < 1.0 else "VC"), (VD, "VD")]
    if abs(VA - VC) >= 1.0:
        marks.append((VA, "VA"))
    n_lab = min([nneg] + [r_["n_gust_VC"][1] for r_ in R["loads"].get("gust_matrix", [])]
                + [v["n_gust_VC"][1] for v in vn.values()]) - 0.55
    for x, t in marks:
        ax.axvline(x, color="#9CA3AF", lw=0.6, ls=":")
        ax.text(x, n_lab, t, ha="center", fontsize=8)
    ax.axhline(0.0, color="#6B7280", lw=0.6)
    ax.set_xlabel("EAS (m/s)")
    ax.set_ylabel("Yük katsayısı n")
    n_top = max([npos] + [v["n_gust_VC"][0] for v in vn.values()] + [r_["n_gust_VC"][0] for r_ in gm])
    n_bot = min([nneg] + [v["n_gust_VC"][1] for v in vn.values()] + [r_["n_gust_VC"][1] for r_ in gm])
    ax.set_ylim(n_bot - 1.9, n_top + 0.6)
    ax.legend(fontsize=7, loc="upper left")
    ax.set_title(f"V-n diyagramı (CS-LUAS +{npos:.1f}/{nneg:.2f}, Ude 15,24/7,62 m/s; yapılandırma CLα)".replace(".", ","))
    fig.tight_layout()
    fig.savefig(path)
    plt.close(fig)


def fig_3view(S: dict, R: dict, path: Path) -> dict:
    """Dimensioned general arrangement from the spec geometry (manifold3d silhouettes of the spec OML, propeller,
    turret and landing gear, gear and turret extended): front view and plan view (nose up, starboard right) on the
    left, side view (port, nose left) and the Turkish data table on the right. Metres."""
    from ..outputs import drawings as DR
    from matplotlib.patches import Polygon as MplPolygon
    plt = _plt()
    plt.rcParams.update({"axes.grid": False})
    parts = scene_parts(S, "extended", "down")
    # front and side views in the static ground attitude (nose up, both wheel contacts on the ground line): rotate
    # about +y through the main-wheel contact; the plan view stays in body axes
    LGd = S["landing_gear"]
    th_s = math.radians(float(LGd["checks"]["static_attitude_deg"])) if "checks" in LGd else 0.0
    c_mg = (float(LGd["main"]["axle_static"][0]), 0.0, float(LGd["ground_z"]))
    parts_g = [(n, m.rotated_about(c_mg, (0, 1, 0), th_s) if th_s else m, c) for n, m, c in parts]
    face = {"skin": "#DDE2E7", "skin_dark": "#BCC4CC", "accent": "#97A0A9", "prop": "#6B7280", "gear": "#7C838B",
            "tyre": "#3A3F44", "turret": "#4B5259", "metal": "#9AA1A8", "glass": "#2F3A44", "dark": "#4B5563"}
    sil = {}
    # draw order = visibility: farthest from the viewer first (plan: lowest z; side from port: largest y; front:
    # most aft x), so hidden parts are covered by the nearer silhouettes
    depth = {"plan": lambda m: float(m.V[:, 2].mean()), "side": lambda m: -float(m.V[:, 1].mean()),
             "front": lambda m: -float(m.V[:, 0].mean())}
    for v in ("plan", "side", "front"):
        polys = []
        for name, m, col in sorted(parts if v == "plan" else parts_g, key=lambda t: depth[v](t[1])):
            try:
                cs = DR._project(m, v)
            except Exception:                                            # noqa: BLE001
                continue
            polys += [(P, col) for P in DR._polys(cs)]
        sil[v] = polys
    gz = float(S["landing_gear"]["ground_z"])
    b2 = float(S["wing"]["span"]) / 2
    L_all = max(float(np.max(m.V[:, 0])) for n, m, c in parts)
    z_top = max(float(np.max(m.V[:, 2])) for n, m, c in parts_g)
    x_side0 = min(float(np.min(m.V[:, 0])) for n, m, c in parts_g)
    x_side1 = max(float(np.max(m.V[:, 0])) for n, m, c in parts_g)
    gap = 0.75
    v_front = 0.85 - gz                       # front/side views: ground line 0.85 m above the plan view's nose
    u_side = b2 + gap + 0.2
    tf = {"plan": lambda P: np.column_stack([P[:, 1], -P[:, 0]]),
          "front": lambda P: np.column_stack([P[:, 0], P[:, 1] + v_front]),
          "side": lambda P: np.column_stack([P[:, 0] + u_side, P[:, 1] + v_front])}
    fig = plt.figure(figsize=(18.0, 11.0))
    ax = fig.add_axes([0.02, 0.02, 0.96, 0.90])
    ax.set_aspect("equal")
    ax.axis("off")
    for v, polys in sil.items():
        for P, col in polys:
            ax.add_patch(MplPolygon(tf[v](P), closed=True, fc=face.get(col, "#CCCCCC"), ec="#1F2937", lw=0.4))
    ax.plot([-b2 - 0.2, b2 + 0.2], [v_front + gz] * 2, color="#6B7280", lw=0.6)
    ax.plot([u_side - 0.2, u_side + L_all + 0.2], [v_front + gz] * 2, color="#6B7280", lw=0.6)

    def dim(p0, p1, off, label, horiz=True, fs=8.5):
        p0, p1 = np.asarray(p0, float), np.asarray(p1, float)
        if horiz:
            y = p0[1] + off
            a, b = np.array([p0[0], y]), np.array([p1[0], y])
            for q in (p0, p1):
                ax.plot([q[0], q[0]], [q[1], y + 0.04 * np.sign(off)], lw=0.5, color="k")
            t, rot = (a + b) / 2 + [0, 0.06 * np.sign(off) if off else 0.06], 0
        else:
            x = p0[0] + off
            a, b = np.array([x, p0[1]]), np.array([x, p1[1]])
            for q in (p0, p1):
                ax.plot([q[0], x + 0.04 * np.sign(off)], [q[1], q[1]], lw=0.5, color="k")
            t, rot = (a + b) / 2 + [0.07 * np.sign(off), 0], 90
        ax.annotate("", xy=b, xytext=a, arrowprops=dict(arrowstyle="<|-|>", lw=0.6, mutation_scale=8, color="k"))
        ax.text(*t, label, fontsize=fs, ha="center", va="center", rotation=rot,
                bbox=dict(boxstyle="square,pad=0.15", fc="white", ec="none"))
    W = S["wing"]
    yj = float(W["planform"]["y_junction"])
    rot_g = lambda P: Mesh(np.atleast_2d(np.asarray(P, float)), np.zeros((0, 3), int)).rotated_about(    # noqa: E731
        c_mg, (0, 1, 0), th_s).V[0] if th_s else np.asarray(P, float)
    mg = rot_g(S["landing_gear"]["main"]["axle_static"])
    ng = rot_g(S["landing_gear"]["nose"]["axle_static"])
    D = float(S["propeller"]["diameter"])
    hub = rot_g(S["propeller"]["hub"])
    # plan view
    dim((-b2, -L_all), (b2, -L_all), -0.30, f"Kanat açıklığı {2 * b2:.2f} m")
    dim((-b2, 0.0), (-b2, -L_all), -0.35, f"Toplam boy {L_all:.2f} m", horiz=False)
    xj = float(W["sections"][0]["x_le"]) + 0.5 * float(W["sections"][0]["chord"])
    dim((-yj, -xj - 0.55), (yj, -xj - 0.55), 0.0, f"Orta kesit {2 * yj:.2f} m")
    dim((yj, -float(W["mac_le_x"]) - 1.4), (b2, -float(W["mac_le_x"]) - 1.4), 0.0,
        f"Dış panel {b2 - yj:.2f} m")
    # front view
    dim((-b2, z_top + v_front), (-b2, gz + v_front), -0.30, f"Yükseklik {z_top - gz:.2f} m", horiz=False)
    dim((-mg[1], gz + v_front), (mg[1], gz + v_front), -0.25, f"İz {2 * mg[1]:.2f} m")
    # propeller disc (dash-dot) in the front view, labelled by a leader; disc edge line in the plan and side views
    th = np.linspace(0.0, 2 * np.pi, 181)
    ax.plot(hub[1] + D / 2 * np.cos(th), hub[2] + v_front + D / 2 * np.sin(th), ls="-.", lw=0.6, color="#374151")
    ax.plot([hub[1] - D / 2, hub[1] + D / 2], [-hub[0]] * 2, ls="-.", lw=0.6, color="#374151")
    ax.plot([u_side + hub[0]] * 2, [hub[2] + v_front - D / 2, hub[2] + v_front + D / 2], ls="-.", lw=0.6,
            color="#374151")
    ax.annotate(f"Pervane diski Ø{D:.3f} m", xy=(hub[1] + D / 2 * np.cos(np.pi / 5), hub[2] + v_front +
                D / 2 * np.sin(np.pi / 5)), xytext=(hub[1] + D / 2 + 0.75, hub[2] + v_front + D / 2 + 0.05),
                fontsize=8.5, va="center", arrowprops=dict(arrowstyle="-", lw=0.5, color="k"))
    # side view (static ground attitude)
    dim((u_side + ng[0], gz + v_front), (u_side + mg[0], gz + v_front), -0.25,
        f"Dingil açıklığı {mg[0] - ng[0]:.2f} m")
    dim((u_side + x_side0, gz + v_front), (u_side + x_side1, gz + v_front), -0.55,
        f"Yerde boy {x_side1 - x_side0:.2f} m")
    ax.text(u_side + x_side1, gz + v_front - 0.12, f"statik tutum {math.degrees(th_s):.1f}° burun yukarı",
            fontsize=8, ha="right", va="center", color="#4B5563")
    ax.text(-b2, v_front + z_top + 0.35, "ÖNDEN GÖRÜNÜŞ", fontsize=11, weight="bold")
    ax.text(u_side, v_front + z_top + 0.35, "YANDAN GÖRÜNÜŞ (sol)", fontsize=11, weight="bold")
    ax.text(-b2, 0.25, "ÜSTTEN GÖRÜNÜŞ (burun yukarıda, sancak sağda)", fontsize=11, weight="bold")
    M = metrics(S, R)
    rows = [("Ad", S["meta"]["name"]), ("Revizyon", f"{S['meta']['revision']} ({S['meta']['date']})"),
            ("MTOM / boş / yakıt / yük", f"{M['mtow_kg']:.1f} / {M['empty_kg']:.1f} / {M['fuel_kg']:.1f} / "
                                         f"{M['payload_kg']:.1f} kg"),
            ("Kanat", f"S {float(W['area']):.2f} m², AR {float(W['aspect_ratio']):.1f}, OAK {float(W['mac']):.3f} m"),
            ("Motor / pervane", f"Limbach L 275 EF 18 kW / Mejzlik {S['propeller'].get('designation', '32x18 2B')} "
                                f"itici"),
            ("Dayanım (tasarım görevi)", f"{M['endurance_h']:.2f} h (3000 m, taret dışarıda)"),
            ("Tutunma hızı (temiz, MTOM)", f"{M['vs_clean_sl_mtow_m_s']:.1f} m/s"),
            ("(L/D)maks", f"{M['ld_max']:.1f}"),
            ("Statik marj", f"%{M['static_margin_min'] * 100:.0f} – %{M['static_margin_max'] * 100:.0f} OAK"),
            ("İniş takımı", "içeri katlanır üç tekerlekli"),
            ("EO/IR taret", "geri çekilir (HD59; E180 büyüme zarfı)"),
            ("Kapsam", "sivil EO/IR gözetleme ve araştırma")]
    x0, y0 = u_side, -0.75
    ax.text(x0, y0 + 0.25, "TEKNİK VERİLER", fontsize=11, weight="bold")
    for i, (k, v) in enumerate(rows):
        ax.text(x0, y0 - 0.27 * i, f"{k}", fontsize=9, weight="bold")
        ax.text(x0 + 1.75, y0 - 0.27 * i, v, fontsize=9)
    ax.text(x0, y0 - 0.27 * len(rows) - 0.2, "Ölçüler metre. Silüetler spec.yaml geometrisinden (takım ve taret açık).\n"
            "Önden ve yandan görünüş yerdeki statik tutumda; üstten görünüş gövde eksenlerinde.\n"
            "Orta kesit: gövde + LERX parçası (dış kanat panelleri sökülür).\n"
            "X burundan geriye, Z = 0 gövde kenar çizgisi (chine) düzlemi.", fontsize=8, color="#4B5563", va="top")
    ax.set_xlim(-b2 - 0.7, u_side + max(L_all, x_side1) + 0.4)
    ax.set_ylim(-L_all - 0.75, v_front + z_top + 0.6)
    fig.suptitle("YK-250 HANÇER — genel yerleşim (3 görünüş)", fontsize=15, weight="bold", x=0.02, ha="left")
    fig.savefig(path, dpi=110)
    plt.close(fig)
    return {"length_m": L_all, "span_m": 2 * b2, "height_m": z_top - gz}


FIGURES = (("yk250_constraint.png", fig_constraint), ("yk250_polars.png", fig_polars),
           ("yk250_cg_envelope.png", fig_cg), ("yk250_payload_endurance.png", fig_payload_endurance),
           ("yk250_vn.png", fig_vn), ("yk250_3view.png", fig_3view))


def _rel(p: Path) -> str:
    p = Path(p).resolve()
    try:
        return str(p.relative_to(REPO.resolve()))
    except ValueError:
        return str(p)


def write_figures(S: dict, R: dict, fig_dir: Path | None = None) -> dict:
    fig_dir = Path(fig_dir or FIG_DIR)
    fig_dir.mkdir(parents=True, exist_ok=True)
    out = {}
    for name, fn in FIGURES:
        p = fig_dir / name
        fn(S, R, p)
        out[name] = _rel(p)
    return out


def existing_figures(fig_dir: Path | None = None) -> dict:
    """The figure files already in docs/fig (--no-figures keeps listing them in the outputs)."""
    fig_dir = Path(fig_dir or FIG_DIR)
    return {name: _rel(fig_dir / name) for name, _ in FIGURES if (fig_dir / name).exists()}


# =====================================================================================================================
# 16. trade studies (--trades)
# =====================================================================================================================
def _endurance_with(S: dict, R: dict, dm_empty: float = 0.0, d_items: dict | None = None, table_key=None,
                    phases_extra_Dq: float = 0.0) -> float:
    """Endurance of the design mission after a mass change (fuel absorbs it at fixed MTOM) and/or a drag change in
    every phase (D/q, m2) — used by the gear/turret/propeller trades (same polars otherwise)."""
    S2 = copy.deepcopy(S)
    S2["mass"]["fuel_kg"] = float(S["mass"]["fuel_kg"]) - dm_empty
    if phases_extra_Dq:
        S2["aero"]["drag_rules"]["misc_Dq"] = float(S["aero"]["drag_rules"]["misc_Dq"]) + phases_extra_Dq
    r = evaluate(S2, table_key=table_key, sens=False, light=True)
    return r["performance"]["endurance_h"], r


def swap_propeller(S: dict, key: str) -> dict:
    """Copy of the spec with the propeller alternative ``key`` (propeller.alternatives) as the primary propeller."""
    S2 = copy.deepcopy(S)
    pr = S2["propeller"]
    alt = pr["alternatives"][key]
    old = {"model": pr["model"], "blades": pr["blades"], "diameter": pr["diameter"], "pitch_nominal": pr["pitch_nominal"],
           "mass_kg": pr["mass_kg"], "source": "primary of the design point", "blade_model": pr["blade_model"]}
    pr["alternatives"] = {k: v for k, v in pr["alternatives"].items() if k != key}
    pr["alternatives"][pr["table_ref"]] = old
    pr["table_ref"] = key
    for k in ("model", "blades", "diameter", "pitch_nominal", "mass_kg", "blade_model"):
        pr[k] = copy.deepcopy(alt[k])
    return S2


def run_trades(S: dict, R: dict, verbose: bool = True) -> dict:
    """Trade studies around the design point. Every point uses the same equations as the design point."""
    out = {}
    t0 = time.time()
    gr = 1.0 + float(S["mass"]["rules"]["growth_allowance"])
    E0 = R["performance"]["endurance_h"]
    M0 = metrics(S, R)
    # (1) propeller: the design propeller and every alternative, each with its own design closure (gear height for
    # the propeller clearance, fin guard position, stabilator for the power-on trim, masses)
    rows = []
    for key in [S["propeller"]["table_ref"]] + list(S["propeller"].get("alternatives", {}).keys()):
        if key == S["propeller"]["table_ref"]:
            D1, r, M1 = S, R, M0
        else:
            D1 = design_closure(swap_propeller(S, key), verbose=False)
            D1.pop("_closure_history", None)
            r = evaluate(D1, sens=False, light=True)
            M1 = metrics(D1, r)
        p = r["performance"]
        fails = [q["id"] for q in evaluate_requirements(D1, M1) if not q["pass"] and q["metric"] != "turret_fov_upper_min_deg"]
        rows.append({"propeller": key, "model": D1["propeller"]["model"], "endurance_h": M1["endurance_h"],
                     "roc_sl_m_s": M1["roc_sl_m_s"], "takeoff_roll_m": M1["takeoff_ground_roll_m"],
                     "landing_roll_m": M1["landing_ground_roll_m"], "loiter_rpm_3000m": p["loiter_3000m"]["rpm"],
                     "static_thrust_N": p["static_wot"]["T"], "static_tip_mach": p["static_wot"]["tip_mach"],
                     "ground_z_m": float(D1["landing_gear"]["ground_z"]), "empty_kg": M1["empty_kg"],
                     "generator_margin_loiter": M1["generator_margin_loiter"], "requirements_failed": fails,
                     "design": key == S["propeller"]["table_ref"]})
        if verbose:
            print(f"  trade propeller {D1['propeller']['model']}: E {M1['endurance_h']:.2f} h, RoC {M1['roc_sl_m_s']:.2f} "
                  f"m/s, take-off {M1['takeoff_ground_roll_m']:.0f} m, failed {fails}", flush=True)
    out["propeller"] = rows
    # endurance sensitivity to drag (+0.001 CD in every phase, D/q = 0.001 S)
    E_d, _ = _endurance_with(S, R, phases_extra_Dq=0.001 * float(S["wing"]["area"]))
    out["endurance_per_0p001_cd_h"] = E_d - E0
    # (2) landing gear: retractable (design) vs fixed faired tricycle
    G_ = S["landing_gear"]
    items = {i["name"]: i for i in S["mass"]["items"]}
    retr = sum(i["mass_kg"] for i in S["mass"]["items"] if i["group"] == "gear")
    fixed_kg = float(S["mass"]["rules"]["fixed_gear_reference_kg"]) * gr
    faired = float(S["mass"]["rules"]["fixed_gear_faired_Dq_m2"])
    res_ = R["aero"]["polars"]["clean"]["items_D_over_q_m2"]["landing_gear"]
    dm = fixed_kg - retr
    E_fixed, _ = _endurance_with(S, R, dm_empty=dm, phases_extra_Dq=faired - res_)
    out["landing_gear"] = {"retractable": {"mass_kg": retr, "endurance_h": E0},
                           "fixed_faired": {"mass_kg": fixed_kg, "extra_Dq_m2": faired - res_, "endurance_h": E_fixed},
                           "delta_h_retractable_minus_fixed": E0 - E_fixed,
                           "basis": "fixed: components.yaml landing_gear_fixed_tricycle_kg x growth and the faired "
                                    "fixed-gear D/q of the endurance concept study; retractable: SAGITTA legs + doors, "
                                    "residual gap drag (gear up)"}
    # (3) turret: retractable (design) vs fixed (always extended, no lift mechanism/doors)
    mech = sum(i["mass_kg"] for i in S["mass"]["items"] if i["name"].startswith(("turret_lift", "turret_bay_frame")))
    loi_t = R["aero"]["polars"]["loiter"]["items_D_over_q_m2"]["eo_ir_turret"]
    cl_t = R["aero"]["polars"]["clean"]["items_D_over_q_m2"]["eo_ir_turret"]
    E_tf, _ = _endurance_with(S, R, dm_empty=-mech, phases_extra_Dq=0.0)
    S3 = copy.deepcopy(S)
    S3["aero"]["drag_rules"]["turret_door_gaps_Dq"] = loi_t                    # extended in transit/climb too
    S3["mass"]["fuel_kg"] = float(S["mass"]["fuel_kg"]) + mech
    r3 = evaluate(S3, sens=False, light=True)
    out["turret"] = {"retractable": {"mechanism_kg": mech, "endurance_h": E0},
                     "fixed_extended": {"endurance_h": r3["performance"]["endurance_h"],
                                        "extra_Dq_transit_m2": loi_t - cl_t},
                     "delta_h_retractable_minus_fixed": E0 - r3["performance"]["endurance_h"],
                     "other_reasons": "retracted turret: protected at take-off/landing (gravel, prop debris), clean "
                                      "belly on the ground and in transit, lower RCS/visual signature not claimed"}
    if verbose:
        print(f"  trade gear: retractable {E0:.2f} h vs fixed faired {E_fixed:.2f} h; turret: retractable {E0:.2f} h "
              f"vs fixed {r3['performance']['endurance_h']:.2f} h")
    # (4) design closures: span and wing loading (stall-speed target) and the tail arm
    rows = []
    for span, vs in ((float(S["wing"]["span"]) - 0.4, None), (float(S["wing"]["span"]) + 0.4, None),
                     (float(S["wing"]["span"]), 23.0), (float(S["wing"]["span"]), 24.5)):
        S4 = copy.deepcopy(S)
        S4["wing"]["planform"]["span"] = span
        if vs is not None:
            S4["wing"]["planform"]["area_rule"] = {"stall_speed_target": vs}
        D4 = design_closure(S4, verbose=False)
        r4 = evaluate(D4, sens=False, light=True)
        M4 = metrics(D4, r4)
        rows.append({"span_m": span, "stall_speed_target_m_s": vs or float(S["wing"]["planform"]["area_rule"]["stall_speed_target"]),
                     "area_m2": float(D4["wing"]["area"]), "aspect_ratio": float(D4["wing"]["aspect_ratio"]),
                     "wing_kg": empty_mass(D4)["groups"]["wing"], "empty_kg": M4["empty_kg"], "fuel_kg": M4["fuel_kg"],
                     "ld_max": M4["ld_max"], "endurance_h": M4["endurance_h"], "vs_m_s": M4["vs_clean_sl_mtow_m_s"],
                     "takeoff_roll_m": M4["takeoff_ground_roll_m"], "landing_roll_m": M4["landing_ground_roll_m"],
                     "outer_panel_m": M4["outer_panel_length_m"]})
        if verbose:
            print(f"  trade wing: b {span:.2f} VS {rows[-1]['stall_speed_target_m_s']:.1f} -> S {rows[-1]['area_m2']:.3f} "
                  f"E {rows[-1]['endurance_h']:.2f} h", flush=True)
    rows.insert(0, {"span_m": float(S["wing"]["span"]), "stall_speed_target_m_s":
                    float(S["wing"]["planform"]["area_rule"]["stall_speed_target"]), "area_m2": float(S["wing"]["area"]),
                    "aspect_ratio": float(S["wing"]["aspect_ratio"]), "wing_kg": empty_mass(S)["groups"]["wing"],
                    "empty_kg": float(S["mass"]["empty_kg"]), "fuel_kg": float(S["mass"]["fuel_kg"]),
                    "ld_max": R["aero"]["ld_max"], "endurance_h": E0,
                    "vs_m_s": R["performance"]["0"]["VS_clean_m_s"],
                    "takeoff_roll_m": R["performance"]["takeoff_sl_mtow"]["ground_roll_m"],
                    "landing_roll_m": R["performance"]["landing_sl_end_of_mission"]["ground_roll_m"],
                    "outer_panel_m": float(S["wing"]["span"]) / 2 - float(S["wing"]["planform"]["y_junction"]),
                    "design": True})
    out["wing_span_loading"] = rows
    rows = []
    for d in (-0.10, 0.10):
        S5 = copy.deepcopy(S)
        L = S5["fuselage"]["lines"]
        L["x_hub"] += d
        for k in ("stabilator", "fin", "ventral"):
            S5["tail"]["surfaces"][k]["params"]["x_le_root"] += d
        bx = S5["layout"]["rules"]["boxes"]["equipment_bay_aft"]
        bx["x"] = [bx["x"][0] + d, bx["x"][1] + d]
        S5["propeller"]["hub"][0] += d
        S5["propeller"]["plane_x"] += d
        D5 = design_closure(S5, verbose=False)
        r5 = evaluate(D5, sens=False, light=True)
        M5 = metrics(D5, r5)
        rows.append({"body_stretch_m": d, "body_length_m": D5["fuselage"]["stations"][-1][0],
                     "volume_h": r5["stability"]["V_H"], "empty_kg": M5["empty_kg"], "endurance_h": M5["endurance_h"],
                     "cn_beta": M5["cn_beta_per_rad"], "static_margin_min": M5["static_margin_min"]})
        if verbose:
            print(f"  trade tail arm: stretch {d:+.2f} -> E {M5['endurance_h']:.2f} h, Cnb {M5['cn_beta_per_rad']:.4f}",
                  flush=True)
    out["tail_arm"] = rows
    if verbose:
        print(f"  trades: {time.time() - t0:.0f} s")
    return out


# =====================================================================================================================
# 17. Turkish report (out/sizing.md)
# =====================================================================================================================
def _f(x, n=2):
    if x is None:
        return "–"
    if isinstance(x, bool):
        return "evet" if x else "hayır"
    s = f"{x:,.{n}f}"
    return s.replace(",", "X").replace(".", ",").replace("X", ".")


def write_report(S: dict, R: dict, M: dict, reqs: list, chk: dict | None, figs: dict, trades: dict | None) -> str:
    p, a, st, g, m, tu = R["performance"], R["aero"], R["stability"], R["ground"], R["mass"], R["turret"]
    L = []
    w = L.append
    w(f"# YK-250 HANÇER — boyutlandırma raporu (otomatik)\n")
    w(f"Kaynak: `ucav250/spec.yaml` (rev. {S['meta']['revision']}, {S['meta']['date']}); üreten: "
      f"`python3 -m ucav250.analysis.sizing`. Bu dosya elle düzenlenmez.\n")
    w(f"> {S['meta']['scope_tr']}\n")
    n_ok = sum(r["pass"] for r in reqs)
    w(f"**Gereksinimler:** {n_ok}/{len(reqs)} karşılanıyor. " + (
        f"**Türetilmiş değer kontrolü:** {chk['n_compared']} değer, {chk['n_bad']} sapma." if chk else ""))
    w("\n## 1. Özet\n")
    w("| Büyüklük | Değer |\n|---|---|")
    rows = [("MTOM / boş / yakıt / faydalı yük", f"{_f(M['mtow_kg'], 1)} / {_f(M['empty_kg'], 1)} / {_f(M['fuel_kg'], 1)} / "
             f"{_f(M['payload_kg'], 1)} kg"),
            ("Kanat açıklığı / alan / AR", f"{_f(M['span_m'])} m / {_f(float(S['wing']['area']), 3)} m² / "
             f"{_f(float(S['wing']['aspect_ratio']), 1)}"),
            ("Gövde boyu / genişlik / yükseklik", f"{_f(R['geometry']['body_length_m'])} / {_f(R['geometry']['body_width_max_m'])} / "
             f"{_f(R['geometry']['body_height_max_m'])} m"),
            ("Dayanım (tasarım görevi)", f"{_f(M['endurance_h'])} h (bekleme {_f(M['loiter_time_h'])} h)"),
            ("Menzil (feribot, yedek dahil)", f"{_f(M['range_km'], 0)} km"),
            ("Bekleme 3000 m", f"{_f(p['loiter_3000m']['V'], 1)} m/s TAS ({_f(p['loiter_3000m']['EAS'], 1)} EAS), CL "
             f"{_f(p['loiter_3000m']['CL'])}, L/D {_f(p['loiter_3000m']['LD'], 1)}, {_f(p['loiter_3000m']['ff_kg_h'])} kg/h"),
            ("CD0 temiz / bekleme (taret dışarıda) / takım açık", f"{_f(a['cd0'], 4)} / {_f(a['cd0_loiter_turret_out'], 4)} / "
             f"{_f(a['polars']['gear_down']['fit']['cd0'], 4)}"),
            ("(L/D)maks temiz / bekleme", f"{_f(a['ld_max'], 1)} / {_f(a['ld_max_loiter'], 1)}"),
            ("CLmax kanat / trimli temiz", f"{_f(a['CLmax_wing'], 3)} / {_f(a['clmax']['clean_trimmed'], 3)}"),
            ("Tutunma hızı (temiz, MTOM, DS)", f"{_f(M['vs_clean_sl_mtow_m_s'])} m/s"),
            ("Tırmanma DS / 3000 m", f"{_f(M['roc_sl_m_s'])} / {_f(M['roc_3000m_m_s'])} m/s"),
            ("Servis tavanı", f"{_f(M['ceiling_service_m'], 0)} m"),
            ("Kalkış / iniş koşusu (DS)", f"{_f(M['takeoff_ground_roll_m'], 0)} / {_f(M['landing_ground_roll_m'], 0)} m"),
            ("Statik marj aralığı", f"%{_f(M['static_margin_min'] * 100, 1)} – %{_f(M['static_margin_max'] * 100, 1)} OAK"),
            ("Cnβ", f"{_f(M['cn_beta_per_rad'], 4)} 1/rad")]
    for k, v in rows:
        w(f"| {k} | {v} |")
    w("\n## 2. Gereksinim uyumu\n")
    w("| No | Gereksinim | Ölçüt | Hedef | Sonuç | Durum |\n|---|---|---|---|---|---|")
    for r in reqs:
        v = r["actual"]
        w(f"| {r['id']} | {r['text_tr']} | `{r['metric']}` | {r['op']} {_f(float(r['value']), 3)} {r['unit']} | "
          f"{_f(float(v), 3) if v is not None else '–'} | {'✔' if r['pass'] else '✘'} |")
    w("\n## 3. Kütle\n")
    w("| Grup | Kütle (kg) | Bütçe hedefi ± tolerans |\n|---|---|---|")
    bud = S["mass"].get("budget", {})
    for k, v in sorted(m["groups_kg"].items(), key=lambda t: -t[1]):
        b = bud.get(k)
        w(f"| {k} | {_f(v)} | {(_f(b['target_kg']) + ' ± ' + _f(b['tol_kg'])) if b else '–'} |")
    w(f"| **boş (büyüme payı %{_f(float(S['mass']['rules']['growth_allowance']) * 100, 0)} dahil)** | **{_f(M['empty_kg'])}** | |")
    w("\nYükleme durumları (ağırlık merkezi, uçuş: takım ve taret içeride):\n")
    w("| Durum | Kütle (kg) | x_AM (m) | z_AM (m) | SM (% OAK) |\n|---|---|---|---|---|")
    cb = float(S["wing"]["mac"])
    for c in m["cases"]:
        w(f"| {c['name']} | {_f(c['m'])} | {_f(c['x'], 3)} | {_f(c['z'], 3)} | {_f((st['x_np'] - c['x']) / cb * 100, 1)} |")
    w("\n## 4. Aerodinamik\n")
    w(f"* Kaldırma eğimi (girdap kafesi, gövde planı hariç): {_f(a['CL_alpha_vlm'], 3)} 1/rad; taşıma hattı "
      f"{_f(a['CL_alpha_lifting_line'], 3)} 1/rad. CL0 = {_f(a['CL_0'], 3)}.")
    w(f"* Açıklık verimi: taşıma hattı {_f(a['e_lifting_line'], 3)}, Trefftz {_f(a['e_vlm_trefftz'], 3)}; polar uyumu "
      f"e = {_f(a['e'], 3)} (profil sürüklemesinin CL ile değişimi ve trim dahil).")
    w(f"* Kritik kesit: η = {_f(a['stall_onset_eta'], 2)} (LERX dışı), α_stall = {_f(a['alpha_stall_deg'], 1)}°; "
      f"Re kök/uç {a['Re_root']:.2e} / {a['Re_tip']:.2e}.")
    w("\nSürükleme kalemleri (temiz, CD = D/q / S_ref):\n")
    w("| Kalem | CD |\n|---|---|")
    for k, v in a["polars"]["clean"]["cd_items"].items():
        w(f"| {k} | {_f(v, 5)} |")
    w(f"| kanat profil sürüklemesi (CL 0,7; geçiş zorlanmış ×1,15) | {_f(a['profile_drag_tripped_x1p15_at_CL']['0.7'], 5)} |")
    w(f"\nBekleme durumunda taret dışarıda: +{_f(a['polars']['loiter']['cd_items']['eo_ir_turret'] - a['polars']['clean']['cd_items']['eo_ir_turret'], 5)} CD.")
    w("\n## 5. Kararlılık\n")
    w(f"* Nötr nokta: klasik {_f(st['x_np_classic'], 3)} m, girdap kafesi {_f(st['x_np_vlm'], 3)} m → kullanılan "
      f"(öndeki) {_f(st['x_np'], 3)} m.")
    w(f"* Taşıyıcı gövde (Multhopp) Cmα = {_f(st['multhopp']['Cm_alpha'], 3)} 1/rad (ön gövde "
      f"{_f(st['multhopp']['forebody'], 3)}, arka {_f(st['multhopp']['afterbody'], 3)}); kanat-gövde AM "
      f"{_f(st['x_ac_wb'], 3)} m, Cm0_wb {_f(st['Cm0_wb'], 3)}.")
    w(f"* Kuyruk hacmi V_H {_f(st['V_H'], 3)}, V_V {_f(st['V_V'], 4)}; aşağı sapma dε/dα {_f(st['deps'], 3)}.")
    cn = st["cn_beta"]
    w(f"* Cnβ: eğik dikeyler {_f(cn['fins'], 4)}, ventral {_f(cn['ventral'], 4)}, gövde {_f(cn['body_used'], 4)} → "
      f"toplam {_f(cn['total'], 4)} 1/rad. Clβ {_f(st['cl_beta']['total'], 4)} 1/rad.")
    w("\n## 6. İniş takımı ve taret\n")
    w(f"* Zemin z = {_f(g['z_g'], 3)} m; dingil açıklığı {_f(g['wheelbase'], 3)} m; iz {_f(g['track'], 3)} m; "
      f"geri devrilme {_f(g['tipback_deg'], 1)}°; yana devrilme {_f(g['turnover_deg'], 1)}°; burun yükü "
      f"%{_f(g['nose_load_aft_cg'] * 100, 1)} – %{_f(g['nose_load_fwd_cg'] * 100, 1)}.")
    w(f"* Statik zemin tutumu {_f(g['static_attitude_deg'], 1)}° burun yukarı (burun tekeri teması ana tekerlerin "
      f"{_f(g['wheelbase'] * math.tan(math.radians(g['static_attitude_deg'])), 3)} m altında, gövde ekseninde).")
    w(f"* Pervane yer açıklığı (CS-VLA 925(a), MTOM): statik {_f(g['prop_clear_static'], 3)} m, yerden kesilme tutumu "
      f"{_f(g['theta_lof_deg'], 1)}° (amortisör statik) {_f(g['prop_clear_liftoff'], 3)} m, teker koyma tutumu "
      f"{_f(g['theta_td_deg'], 1)}° (takım yüksüz, +{_f(g['gear_unloaded_extension_m'], 3)} m) "
      f"{_f(g['prop_clear_touchdown_unloaded'], 3)} m → en az {_f(g['prop_clear_min_925a'], 3)} m; sönük lastik + dibe "
      f"oturmuş amortisör {_f(g['prop_clear_flat_tyre_bottomed'], 3)} m. Pervane temas açısı "
      f"{_f(g['prop_strike_deg'], 1)}°, kuyruk tamponu temas açısı {_f(g['bumper_contact_deg'], 1)}°; flare "
      f"{_f(g['theta_flare_deg'], 1)}°.")
    pc = R["propeller"]["clearances"]
    w(f"* CS-VLA 925(c): radyal açıklık en az {_f(pc['radial_min_m'], 3)} m, boyuna açıklık en az "
      f"{_f(pc['longitudinal_min_m'], 3)} m (pala ekseni boyutu uç {_f(pc['blade_tip_axial_half_extent_m'], 3)} m; "
      f"parçalara göre: " + ", ".join(f"{k} {_f(v['longitudinal_m'], 3)}" for k, v in pc["by_part"].items()) +
      f"). Eğik dikeylerin firar kenarı pervane düzlemini {_f(pc['fin_te_crossing_radius_m'], 3)} m yarıçapta keser "
      f"(disk ucunun {_f(pc['fin_guard_margin_over_tip_m'] * 1000, 0)} mm dışı).")
    sm, sn = g["stowed"]["main"], g["stowed"]["nose"]
    w(f"* Toplanmış takım: ana teker zarfı içeride {_f(sm['tyre_inside'])}, bacak {_f(sm['leg_inside'])}, mafsal "
      f"{_f(sm['fitting_inside'])}, merkez boşluğu {_f(sm['centre_gap_m'], 3)} m, kanat kutusu açıklığı "
      f"{_f(sm['clear_of_wing_box_m'], 3)} m; burun takımı {_f(sn['ok'])}.")
    w(f"* Taret: içeride top kapak üstünde {_f(tu['flush_margin_m'], 3)} m; E180 büyüme zarfı içeride "
      f"{_f(tu['growth_envelope_inside_retracted'])}; strok {_f(tu['stroke_m'], 3)} m; görüş alanı üst sınırı en az "
      f"{_f(tu['fov_upper_min_deg'], 1)}° (ön ±60°: {_f(tu['fov_upper_forward_sector_min_deg'], 1)}°).")
    w("\n## 7. Performans\n")
    w("| Durum | Değer |\n|---|---|")
    to = p["takeoff_sl_mtow"]
    w(f"| Kalkış (DS, MTOM, belirleyici yükleme: {CASE_TR.get(to['case'], to['case'])}) | koşu "
      f"{_f(to['ground_roll_m'], 0)} m, 15 m'ye {_f(to['distance_15m_m'], 0)} m; burun kaldırma V_R "
      f"{_f(to['V_R_m_s'], 1)} m/s (ana tekerlerde {_f(to['main_gear_load_at_VR_N'], 0)} N), V_LOF "
      f"{_f(to['V_lof_m_s'], 1)} m/s (VS_TO {_f(to['VS_TO_m_s'], 1)} m/s, flap {_f(to['flap_deg'], 0)}°) |")
    for cn_, cv in to.get("cases", {}).items():
        w(f"| — {CASE_TR.get(cn_, cn_)} | koşu {_f(cv['ground_roll_m'], 0)} m, V_R {_f(cv['V_R_m_s'], 1)}, V_LOF "
          f"{_f(cv['V_lof_m_s'], 1)} m/s, x_AM {_f(cv['x_cg'], 3)} m |")
    to2 = p["takeoff_1500m_isa_mtow"]
    w(f"| Kalkış (1500 m ISA) | koşu {_f(to2['ground_roll_m'], 0)} m |")
    ld = p["landing_sl_end_of_mission"]
    w(f"| İniş (görev sonu {_f(p['landing_mass_end_of_mission_kg'], 1)} kg) | koşu {_f(ld['ground_roll_m'], 0)} m, "
      f"V_TD {_f(ld['V_td_m_s'], 1)} m/s |")
    for h in ("0", "3000"):
        q = p[h]
        w(f"| {h} m | VS {_f(q['VS_clean_m_s'], 1)} m/s, Vmaks {_f(q['V_max_m_s'], 1)} m/s, tırmanma "
          f"{_f(q['RoC_max_m_s'], 2)} m/s @ {_f(q['V_climb_m_s'], 1)} m/s |")
    w(f"| Tavan | servis {_f(p['ceiling_service_m'], 0)} m, mutlak {_f(p['ceiling_absolute_m'], 0)} m |")
    w("\nFaydalı yük – dayanım:\n")
    w("| Yük (kg) | Yakıt (kg) | Dayanım (h) |\n|---|---|---|")
    for r in p["payload_endurance"]:
        w(f"| {_f(r['payload_kg'], 2)} | {_f(r['fuel_kg'], 2)}{' (hacim sınırı)' if r['fuel_volume_limited'] else ''} | "
          f"{_f(r['endurance_h'], 2)} |")
    sens = R.get("sensitivities_endurance_h", {})
    if sens:
        w("\nDuyarlılıklar (dayanım, h): " + ", ".join(f"{k} {_f(v, 2)}" for k, v in sens.items()))
    w("\n## 8. Kumanda yüzeyleri, kökler, kiriş derinliği, elektrik, yükler\n")
    hm, rt, sd = R["stabilator_hinge"], R["tail_roots"], R["spar_depth"]
    w(f"* Stabilatör mili panel OAK'ının %{_f(float(S['tail']['surfaces']['stabilator']['params']['pivot_mac_fraction']) * 100, 0)}"
      f"'sinde (x = {_f(hm['spindle_x'], 3)} m); AM belirsizliği %22–%30 OAK → kol {_f(hm['ac_offset_min_m'] * 1000, 0)}–"
      f"{_f(hm['ac_offset_max_m'] * 1000, 0)} mm. Menteşe momenti VA'da {_f(hm['H_VA_Nm'], 1)} N·m, VD'de "
      f"{_f(hm['H_VD_Nm'], 1)} N·m, sürekli trim {_f(hm['H_trim_continuous_Nm'], 1)} N·m; {hm['actuator']} × "
      f"{_f(hm['linkage_ratio'], 1)}:1 bağlantı → tepe pay {_f(hm['peak_margin'], 2)} (1,25 katsayısı dahil), sürekli pay "
      f"{_f(hm['rated_margin'], 2)}; yüzey hareketi ±{_f(hm['surface_travel_deg'], 1)}°.")
    w(f"* Stabilatör kökü: sabit kök parçasına boşluk {_f(rt['stab_root_gap_m'] * 1000, 1)} mm (bütün sapma aralığında "
      f"sabit), gövdeye en az pay {_f(rt['stab_root_body_clearance_m'] * 1000, 1)} mm "
      f"({_f(rt['stab_root_deflection_range_deg'][0], 0)}°…{_f(rt['stab_root_deflection_range_deg'][1], 0)}°). Dikey "
      f"kuyruk kökü bütün veter boyunca en az {_f(rt['fin_root_embed_min_m'] * 1000, 1)} mm gömülü.")
    w(f"* Kiriş derinliği (gövde yanı → dış panel bağlantısı): ana kiriş en az {_f(sd['main']['min_depth_m'] * 1000, 1)} mm "
      f"(gerekli {_f(sd['required_main_m'] * 1000, 1)} mm), arka kiriş en az {_f(sd['rear']['min_depth_m'] * 1000, 1)} mm "
      f"(gerekli {_f(sd['required_rear_m'] * 1000, 1)} mm = bağlantı kesitinin kendi arka kiriş derinliği).")
    el = R["electrical"]
    w(f"* Elektrik (R-32): jeneratör beklemede {_f(el['generator_W_loiter'], 0)} W; sürekli yük HD59 ile "
      f"{_f(el['continuous_hd59_W'], 0)} W, E180 ile {_f(el['continuous_e180_W'], 0)} W (araştırma yükü payı dahil) → "
      f"pay {_f(el['margin_continuous'], 2)}; E180 tepe {_f(el['peak_e180_W'], 0)} W" +
      (f", açık {_f(el['peak_deficit_W'], 0)} W tampon bataryadan ({_f(el['battery_holdup_peak_deficit_min'], 0)} dk)."
       if el["peak_deficit_W"] > 0 else " jeneratörden karşılanır."))
    gm = R["loads"]["gust_matrix"]
    w(f"* Rüzgâr hamlesi matrisi (yapılandırma CLα {_f(R['loads']['cl_alpha_used'], 3)} 1/rad): " +
      "; ".join(f"{_f(r_['mass_kg'], 1)} kg / {int(r_['altitude_m'])} m: +{_f(r_['n_pos'], 2)} / {_f(r_['n_neg'], 2)}"
                for r_ in gm) + f". Kanat tasarım yük katsayısı {_f(R['loads']['n_limit_wing_design'], 2)} (en büyük "
      f"n·m·g, MTOM); kök momenti (nihai) {_f(R['mass']['wing_structure']['M_root_ult_Nm'], 0)} N·m.")
    w("\n## 9. Yerleşim ve hacimler\n")
    w("| Bölge | Sığıyor |\n|---|---|")
    for k, v in R["packaging"].items():
        if isinstance(v, dict) and "fits" in v and k not in ("fuel",):
            w(f"| {k} | {_f(bool(v['fits']))} |")
    fu = R["packaging"]["fuel"]
    w(f"\nYakıt hacmi: gereken {_f(fu['required_m3'] * 1000, 1)} L, kullanılabilir {_f(fu['available_m3'] * 1000, 1)} L "
      f"(tank verimi dahil); yakıt AM'si x = {_f(fu['fuel_cg_x_volume_centroid'], 3)} m.")
    if trades:
        w("\n## 10. Ödünleşimler\n")
        w("| Pervane (her biri kendi kapanışıyla) | Dayanım (h) | Tırmanma DS (m/s) | Kalkış / iniş koşusu (m) | "
          "Statik itki (N) | Uç Mach | Karşılanmayan gereksinimler |\n|---|---|---|---|---|---|---|")
        for r in trades["propeller"]:
            w(f"| {r.get('model', r['propeller'].split('.')[-1])}{' (tasarım)' if r.get('design') else ''} | "
              f"{_f(r['endurance_h'])} | {_f(r['roc_sl_m_s'])} | {_f(r['takeoff_roll_m'], 0)} / "
              f"{_f(r.get('landing_roll_m'), 0)} | {_f(r['static_thrust_N'], 0)} | {_f(r['static_tip_mach'], 3)} | "
              f"{', '.join(r.get('requirements_failed', [])) or '–'} |")
        if "endurance_per_0p001_cd_h" in trades:
            w(f"\nDayanımın sürüklemeye duyarlılığı: her uçuş evresinde +0,001 CD → "
              f"{_f(trades['endurance_per_0p001_cd_h'], 3)} h.")
        tg = trades["landing_gear"]
        w(f"\nİniş takımı: içeri katlanır {_f(tg['retractable']['endurance_h'])} h ({_f(tg['retractable']['mass_kg'])} kg) "
          f"– sabit kaportalı {_f(tg['fixed_faired']['endurance_h'])} h ({_f(tg['fixed_faired']['mass_kg'])} kg).")
        tt = trades["turret"]
        w(f"Taret: geri çekilir {_f(tt['retractable']['endurance_h'])} h – sabit (sürekli dışarıda) "
          f"{_f(tt['fixed_extended']['endurance_h'])} h.\n")
        w("| Açıklık (m) | VS hedefi (m/s) | S (m²) | AR | Kanat (kg) | Boş (kg) | Dayanım (h) | Kalkış (m) |\n"
          "|---|---|---|---|---|---|---|---|")
        for r in trades["wing_span_loading"]:
            w(f"| {_f(r['span_m'])}{' (tasarım)' if r.get('design') else ''} | {_f(r['stall_speed_target_m_s'], 1)} | "
              f"{_f(r['area_m2'], 3)} | {_f(r['aspect_ratio'], 1)} | {_f(r['wing_kg'])} | {_f(r['empty_kg'])} | "
              f"{_f(r['endurance_h'])} | {_f(r['takeoff_roll_m'], 0)} |")
        w("\n| Gövde uzatma (m) | Boy (m) | V_H | Boş (kg) | Dayanım (h) | Cnβ |\n|---|---|---|---|---|---|")
        for r in trades["tail_arm"]:
            w(f"| {_f(r['body_stretch_m'])} | {_f(r['body_length_m'])} | {_f(r['volume_h'], 3)} | {_f(r['empty_kg'])} | "
              f"{_f(r['endurance_h'])} | {_f(r['cn_beta'], 4)} |")
    if figs:
        w("\n## Şekiller\n")
        for k, v in figs.items():
            w(f"* `{v}`")
    if chk and chk["n_bad"]:
        w("\n## Türetilmiş değer sapmaları\n")
        for b in chk["bad"][:60]:
            w(f"* `{b['path']}`: spec {b['spec']} – hesap {b['computed']} ({b['issue']})")
    return "\n".join(L) + "\n"


# =====================================================================================================================
# 18. spec writer (--update-spec): re-close the design and refresh every derived value and reference copy in the spec
# =====================================================================================================================
SPEC_HEAD = """# =====================================================================================================================
# ucav250/spec.yaml — YK-250 HANÇER, v1 (sizing phase)
# Single source of truth (ARCHITECTURE.md §1, §9). SI units, metres, kg, W, degrees in keys ending _deg.
# Frame: X aft from the nose tip, Y starboard, Z up; Z = 0 on the centre-body chine plane (layout.datum).
# Derived blocks (wing sections, fuselage stations, tail sections, landing-gear positions, mass items, layout zones,
# reference copies) are written by `python3 -m ucav250.analysis.sizing --update-spec` (design closure from the inputs
# and rules in this file) and verified by `python3 -m ucav250.analysis.sizing --check` (exit 1 on any deviation or
# requirement violation). Workflow: edit inputs/rules here -> --update-spec -> --check.
# =====================================================================================================================
"""

SPEC_SECTION_TITLES = {
    "meta": "identity and scope", "requirements": "requirements (metric = key of out/sizing.json -> metrics)",
    "mission": "design mission", "engine": "engine (Limbach L 275 EF)", "propeller": "propeller (open pusher)",
    "configuration": "configuration decisions", "wing": "wing (reference trapezoid + LERX/glove + outer panel sections)",
    "tail": "tail (canted fins, stabilators, ventral fin / bumper)", "fuselage": "fuselage (chined lifting body)",
    "landing_gear": "landing gear (retractable tricycle)", "payload": "payload (EO/IR turret, mission equipment)",
    "aero": "aerodynamic rules and reference results", "mass": "mass (MTOM, items, cases, budget)",
    "stability": "stability", "performance": "performance (reference copy)", "structures": "structural design basis",
    "materials": "materials (materials.yaml spec_ready)", "adhesives": "adhesives (materials.yaml spec_ready)",
    "layups": "layups", "processes": "processes (materials.yaml spec_ready)", "display": "display",
    "layout": "layout (sizing-phase zones; filled by the layout phase)", "assembly": "assembly (placeholder)"}

GEAR_CHECK_KEYS = ("z_g", "track", "wheelbase", "h_cg", "static_attitude_deg", "tipback_deg", "turnover_deg",
                   "nose_load_aft_cg", "nose_load_fwd_cg", "prop_clear_level", "prop_clear_static",
                   "prop_clear_liftoff", "prop_clear_touchdown_unloaded", "prop_clear_min_925a",
                   "gear_unloaded_extension_m", "prop_clear_flat_tyre_bottomed", "prop_strike_deg",
                   "bumper_contact_deg", "theta_lof_deg", "theta_td_deg", "theta_flare_deg")
TURRET_CHECK_KEYS = ("flush_margin_m", "growth_margin_to_door_m", "growth_envelope_inside_retracted",
                     "extended_ball_top_below_skin_m", "stroke_m", "aperture_radius_m", "fov_upper_min_deg",
                     "fov_upper_forward_sector_min_deg", "fov_upper_limit_deg_by_azimuth")


class _SpecDumper(yaml.SafeDumper):
    """Block style for mappings; flow style for short lists of scalars and for tables of short scalar rows."""


def _spec_repr_list(dumper, data):
    flow = all(not isinstance(v, (dict, list)) for v in data) and len(data) <= 12
    flow = flow or (len(data) > 0 and all(isinstance(v, list) and len(v) <= 8 and
                                          all(not isinstance(w, (dict, list)) for w in v) for v in data))
    return dumper.represent_sequence("tag:yaml.org,2002:seq", data, flow_style=flow)


_SpecDumper.add_representer(list, _spec_repr_list)


def write_spec(S: dict, path) -> None:
    """Write the spec with the file header and one commented banner per top-level block."""
    bar = "# " + "-" * 117 + "\n"
    text = [SPEC_HEAD]
    for k, v in S.items():
        text.append(f"\n{bar}# {k}: {SPEC_SECTION_TITLES.get(k, '')}\n{bar}")
        text.append(yaml.dump({k: v}, Dumper=_SpecDumper, sort_keys=False, width=118, allow_unicode=True,
                              default_flow_style=False))
    Path(path).write_text("".join(text), encoding="utf-8")


def refresh_spec(S_in: dict, verbose: bool = True) -> tuple[dict, dict]:
    """Re-close the design from the spec inputs and design rules and refresh every derived value and reference copy
    that ``--check`` compares. Authored content (texts, sources, requirements, rules) is kept; the mass budget stays a
    deliberate allocation (``--check`` reports a group that leaves its band). Returns (new spec, its evaluation)."""
    D = design_closure(S_in, verbose=verbose)
    D.pop("_closure_history", None)
    R = evaluate(D, sens=True)
    ref = reference_blocks(D, R)
    st, g, p, pr = R["stability"], R["ground"], R["performance"], R["propeller"]
    D["propeller"]["static_wot_reference"] = {"rpm": pr["static_wot"]["rpm"], "thrust_N": pr["static_wot"]["T"],
                                              "tip_mach": pr["static_wot"]["tip_mach"],
                                              "tip_speed_m_s": pr["static_tip_speed_m_s"]}
    cases = R["mass"]["cases"]
    xs = [c["x"] for c in cases]
    TL = D["tail"]
    # the arms actually used for the tail volumes (design CG to the stabilator / exposed-fin aerodynamic centres)
    TL["volume_h"], TL["volume_v"] = st["V_H"], st["V_V"]
    TL["arm_h"], TL["arm_v"] = st["arm_h_VH"], st["arm_v_VV"]
    TL["arm_h_wing_ac_to_tail_ac"] = st["l_h"]
    TL["arms_note"] = ("arm_h / arm_v: design CG (MTOM case) to the stabilator AC / the exposed-fin AC, the arms of "
                       "volume_h / volume_v; arm_h_wing_ac_to_tail_ac: wing AC to the stabilator AC (downwash)")
    hm = R["stabilator_hinge"]
    TL["surfaces"]["stabilator"]["controls"]["checks"] = {
        k: hm[k] for k in ("spindle_x", "panel_mac_m", "ac_offset_min_m", "ac_offset_max_m", "H_VA_Nm", "H_VD_Nm",
                           "H_trim_continuous_Nm", "peak_capacity_Nm", "peak_margin", "rated_margin",
                           "surface_travel_deg", "travel_ok", "statically_stable_surface")}
    rt = R["tail_roots"]
    TL["surfaces"]["stabilator"]["root_checks"] = {k: rt[k] for k in ("stab_root_gap_m", "stab_root_body_clearance_m",
                                                                      "stab_root_deflection_range_deg")}
    TL["surfaces"]["fin"]["root_checks"] = {k: rt[k] for k in ("fin_root_max_gap_m", "fin_root_embed_min_m")}
    pcl = pr["clearances"]
    D["propeller"]["clearance_checks"] = {
        "radial_min_m": pcl["radial_min_m"], "longitudinal_min_m": pcl["longitudinal_min_m"],
        "by_part": pcl["by_part"], "spinner_to_cowl_gap_m": pcl["spinner_to_cowl_gap_m"],
        "fin_te_crossing_radius_m": pcl["fin_te_crossing_radius_m"],
        "fin_guard_margin_over_tip_m": pcl["fin_guard_margin_over_tip_m"],
        "blade_tip_axial_half_extent_m": pcl["blade_tip_axial_half_extent_m"]}
    A_ = R["aero"]
    D["aero"].update(ref["aero_top"])
    D["aero"]["polars"] = {k: f"out/sizing.json#aero.polars.{k} (trimmed CL-CD table, design CG; fit cd0/k/e)"
                           for k in A_["polars"]}
    D["aero"]["power_on_trim"] = {"max_abs_tail_cl_local": A_["power_on_trim"]["max_abs_tail_cl_local"],
                                  "governing": A_["power_on_trim"]["governing"]}
    D["aero"]["flaps"] = A_["flaps"]
    fb = R["geometry"]
    D["fuselage"].update({"length": fb["body_length_m"], "width_max": fb["body_width_max_m"],
                          "height_max": fb["body_height_max_m"], "volume_m3": fb["body_volume_m3"],
                          "wetted_area_exposed_m2": fb["S_wet_body_exposed_m2"], "fineness": fb["body_fineness"]})
    chk = {k: g[k] for k in GEAR_CHECK_KEYS}
    chk["static_load_split_main_aft_cg"] = 1.0 - g["nose_load_aft_cg"]
    D["landing_gear"]["checks"] = chk
    D["payload"]["turret"]["checks"] = {k: R["turret"][k] for k in TURRET_CHECK_KEYS}
    D["aero"]["reference"] = ref["aero"]
    D["aero"]["drag_build_up_clean_cd"] = R["aero"]["polars"]["clean"]["cd_items"]
    D["stability"].update({"mac": D["wing"]["mac"], "mac_le_x": D["wing"]["mac_le_x"], "np_x": st["x_np"],
                           "cg_design": [cases[0]["x"], cases[0]["y"], cases[0]["z"]], "cg_range_x": [min(xs), max(xs)],
                           "static_margin_range": [st["sm_min"], st["sm_max"]],
                           "cases_cg": [{"name": c["name"], "m": c["m"], "x": c["x"], "z": c["z"]} for c in cases],
                           "computed": ref["stability"]})
    D["performance"].update({"reference": ref["performance"], "payload_endurance": p["payload_endurance"],
                             "sensitivities_endurance_h": R["sensitivities_endurance_h"],
                             "loiter_3000m": p["loiter_3000m"]})
    vn = R["loads"]["vn"]
    ws = float(D["mass"]["mtow_kg"]) * G / float(D["wing"]["area"])
    gm = R["loads"]["gust_matrix"]
    ws_ = R["mass"]["wing_structure"]
    D["structures"]["derived"] = {"VA_eas": R["loads"]["VA_m_s"], "VD_eas_used": R["loads"]["VD_m_s"],
                                  "n_limit_gust_pos": max(v["n_limit_pos"] for v in vn.values()),
                                  "n_limit_gust_neg": min(v["n_limit_neg"] for v in vn.values()),
                                  "cl_alpha_configuration_per_rad": R["loads"]["cl_alpha_used"],
                                  "n_limit_wing_design": R["loads"]["n_limit_wing_design"],
                                  "gust_matrix": [{"mass_kg": r_["mass_kg"], "altitude_m": r_["altitude_m"],
                                                   "n_pos": r_["n_pos"], "n_neg": r_["n_neg"]} for r_ in gm],
                                  "gust_matrix_note": ("CS-LUAS/VLA 341 gust and manoeuvre limit load factors, "
                                                       "configuration lift slope (VLM, all surfaces), every mass x "
                                                       "altitude; the wing is sized for the largest lift n m g "
                                                       "(MTOM); equipment and attachments use n of the lightest case"),
                                  "sink_speed_m_s": min(max(0.51 * ws ** 0.25, 2.13), 3.05),     # CS-VLA 473(d)
                                  "wing_root_moment_ult_Nm": ws_["M_root_ult_Nm"],
                                  "wing_junction_moment_ult_Nm": ws_["M_junction_ult_Nm"],
                                  "spar_cap_area_root_mm2": ws_["A_cap_root_mm2"],
                                  "spar_cap_area_glove_max_mm2": ws_["A_cap_glove_max_mm2"],
                                  "spar_h_eff_glove_min_m": ws_["h_eff_glove_min_m"],
                                  "schrenk_basis": ws_["schrenk_basis"]}
    D["layout"]["ground_z"] = D["landing_gear"]["ground_z"]
    # keep the spec's shape: its top-level blocks and their keys in the spec's order (closure working keys stay out);
    # the reference/handover keys written above are added after the authored keys of their block
    added = {"aero": ("cd0", "e", "k", "ld_max", "clmax_clean", "clmax_to", "clmax_ld", "polars", "power_on_trim",
                      "flaps"),
             "propeller": ("clearance_checks",), "tail": ("arm_h_wing_ac_to_tail_ac", "arms_note"),
             "fuselage": ("lines_derived",)}
    out = {}
    for k, v in S_in.items():
        nv = D.get(k, v)
        if isinstance(v, dict) and isinstance(nv, dict):
            keys = list(v.keys()) + [kk for kk in added.get(k, ()) if kk in nv and kk not in v]
            nv = {kk: nv.get(kk, v.get(kk)) for kk in keys}
        out[k] = nv
    return py(out), R


# =====================================================================================================================
# 19. renders (--render) and CLI
# =====================================================================================================================
def render_all(S: dict, fig_dir: Path | None = None) -> dict:
    """Workbench renders from the spec geometry (gear down + turret extended; and gear/turret retracted, iso). The
    ground plane passes through the main-wheel contact and is inclined by the static attitude (nose wheel on it)."""
    fig_dir = Path(fig_dir or FIG_DIR)
    LG = S["landing_gear"]
    th = float(LG["checks"]["static_attitude_deg"]) if "checks" in LG else 0.0
    ground = (float(LG["main"]["axle_static"][0]), float(LG["ground_z"]), th)
    out = {}
    parts = scene_parts(S, "extended", "down")
    out.update(render_scene(parts, fig_dir, "yk250_", ground))
    parts2 = scene_parts(S, "retracted", "up")
    out.update({"retracted_iso": render_scene(parts2, fig_dir, "yk250_turret_gear_retracted_", ground,
                                              views=["iso"])["iso"]})
    return out


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description="YK-250 HANCER sizing study")
    ap.add_argument("--check", action="store_true", help="compare derived spec values and evaluate requirements")
    ap.add_argument("--design", action="store_true", help="design closure -> out/sizing_design.yaml")
    ap.add_argument("--trades", action="store_true", help="trade studies -> out/sizing_trades.json")
    ap.add_argument("--render", action="store_true", help="Workbench renders (needs bpy)")
    ap.add_argument("--update-spec", action="store_true",
                    help="re-close the design and rewrite the derived values of the spec file in place")
    ap.add_argument("--no-figures", action="store_true")
    ap.add_argument("--spec", default=None, help="alternative spec file")
    ap.add_argument("--out", default=None, help="output directory for sizing.json/.md (and the figures in <out>/fig); "
                                                "default ucav250/out and ucav250/docs/fig")
    a = ap.parse_args(argv)
    t0 = time.time()
    S = copy.deepcopy(SPEC.load(a.spec)) if a.spec else copy.deepcopy(SPEC.load())
    out_dir = Path(a.out) if a.out else OUT_DIR
    fig_dir = (out_dir / "fig") if a.out else FIG_DIR
    out_dir.mkdir(parents=True, exist_ok=True)
    if a.design:
        D = design_closure(S, verbose=True)
        hist = D.pop("_closure_history", [])
        with open(out_dir / "sizing_design.yaml", "w", encoding="utf-8") as fh:
            yaml.safe_dump(py(D), fh, sort_keys=False, width=150, allow_unicode=True)
        print(f"[sizing] design closure: {len(hist)} iterations -> {out_dir / 'sizing_design.yaml'}")
        return 0
    if a.render:
        r = render_all(S, fig_dir)
        print("[sizing] renders:", r)
        return 0
    if a.update_spec:
        path = Path(a.spec) if a.spec else SPEC.SPEC_PATH
        S_new, _ = refresh_spec(S, verbose=True)
        diff, n = [], [0]
        _cmp(S, S_new, 0.0, "spec", diff, n)
        write_spec(S_new, path)
        print(f"[sizing] {path}: {len(diff)} of {n[0]} values changed ({time.time() - t0:.0f} s); run --check")
        for d in diff[:25]:
            print(f"  {d['path']}: {d['spec']} -> {d['computed']}")
        return 0
    R = evaluate(S, verbose=False)
    if a.spec:
        R["meta"]["spec"] = str(Path(a.spec).resolve())
    M = metrics(S, R)
    reqs = evaluate_requirements(S, M)
    chk = check_derived(S, R, verbose=True) if a.check else None
    trades = None
    if a.trades:
        trades = run_trades(S, R)
        with open(out_dir / "sizing_trades.json", "w", encoding="utf-8") as fh:
            json.dump(py(trades), fh, indent=1, ensure_ascii=False)
    else:
        for d in (out_dir, OUT_DIR):                         # the last trade study (expensive; read only)
            if (d / "sizing_trades.json").exists():
                trades = json.loads((d / "sizing_trades.json").read_text(encoding="utf-8"))
                break
    figs = existing_figures(FIG_DIR if not a.out else fig_dir) if a.no_figures else write_figures(S, R, fig_dir)
    # no run time or date in the written files: repeated runs on the same spec give identical outputs
    doc = {"meta": R["meta"], "metrics": M, "requirements": reqs, "derived_check": chk, "figures": figs, **R}
    with open(out_dir / "sizing.json", "w", encoding="utf-8") as fh:
        json.dump(py(doc), fh, indent=1, ensure_ascii=False)
    (out_dir / "sizing.md").write_text(write_report(S, R, M, reqs, chk, figs, trades), encoding="utf-8")
    bad_req = [r for r in reqs if not r["pass"]]
    for r in reqs:
        print(f"  {r['id']:5s} {'PASS' if r['pass'] else 'FAIL'}  {r['metric']} = {py(r['actual'], 4)} "
              f"{r['op']} {r['value']} {r['unit']}")
    print(f"[sizing] endurance {M['endurance_h']:.2f} h, MTOM {M['mtow_kg']:.1f} kg, requirements "
          f"{len(reqs) - len(bad_req)}/{len(reqs)} pass" + (f", derived values {chk['n_compared'] - chk['n_bad']}/"
                                                             f"{chk['n_compared']} within tolerance" if chk else "") +
          f" ({time.time() - t0:.0f} s)")
    if a.check and (bad_req or chk["n_bad"]):
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
