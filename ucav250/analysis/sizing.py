"""YK-250 HANCER sizing study: reads ``ucav250/spec.yaml``, closes the design point and verifies every derived value.

CLI (run from the repository root)::

    python3 -m ucav250.analysis.sizing             # evaluate the spec design point -> out/sizing.json, out/sizing.md, figures
                                                   #   (the explicit command that writes the repository outputs)
    python3 -m ucav250.analysis.sizing --check     # verify only: re-close the design from the spec inputs, compare every
                                                   #   derived spec value and evaluate every requirement (exit 1 on a
                                                   #   violation); writes NOTHING (add --out DIR to keep the outputs there)
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


def generator_dc_W(S: dict, rpm: float) -> float:
    """DC bus power the starter/generator delivers at ``rpm`` (V1-02): the SG750 rating (800 W at 7500 rpm, 50 V AC
    3-phase; engine.yaml) scales with the rpm and passes the rectifier/regulator (boost) stage
    engine.generator.power_electronics_efficiency before it reaches the DC loads."""
    g = S["engine"]["generator"]
    return (float(g["power_continuous_W"]) * float(rpm) / float(g.get("rated_rpm", 7500.0)) *
            float(g.get("power_electronics_efficiency", 1.0)))


def generator_rpm_for_dc(S: dict, P_dc: float) -> float:
    """Lowest rpm at which the generator delivers ``P_dc`` W on the DC bus (inverse of generator_dc_W)."""
    return P_dc / generator_dc_W(S, 1.0)


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
        self.elec_load_W = elec_load_W
        self.p_gen_shaft = elec_load_W / eta_gen
        self.bsfc_scale = 1.0                                   # sensitivity runs only
        # below the lowest BSFC point (20 % power: the descent at about 5-7 % and the reserve loiter at about 18 %):
        # "bsfc_linear" = the BSFC line through the two lowest points extrapolated (design model, endurance-study
        # reproduction); "willans" = the FUEL FLOW line through the two lowest points extrapolated (Willans line:
        # fuel flow linear in power with a positive zero-power intercept, the usual part-load behaviour of a
        # spark-ignition engine) - a more conservative extrapolation (not a bound: no data below 20 % power) used by
        # the V3-07 sensitivity
        self.low_load_model = "bsfc_linear"

    def lapse(self, sigma: float) -> float:
        return sigma ** self.lapse_exp

    def torque_wot(self, n, sigma: float):
        n = np.asarray(n, float)
        q = np.interp(n, self.rpm, self.tq) * self.lapse(sigma)
        return np.where(n > self.n_cut, 0.0, q)

    def bsfc(self, p_total: float) -> float:
        """g/kWh at total shaft power (propeller + generator); linear, extrapolated below the lowest point
        (``low_load_model``)."""
        f = p_total / self.P_max
        P = self.bsfc_pts
        if f < P[0, 0]:
            if self.low_load_model == "willans" and f > 1e-6:
                ff0, ff1 = P[0, 0] * P[0, 1], P[1, 0] * P[1, 1]           # fuel flow / P_max at the two lowest points
                ff = ff0 + (ff1 - ff0) / (P[1, 0] - P[0, 0]) * (f - P[0, 0])
                b = ff / f
            else:
                b = P[0, 1] + (P[1, 1] - P[0, 1]) / (P[1, 0] - P[0, 0]) * (f - P[0, 0])
        else:
            b = float(np.interp(f, P[:, 0], P[:, 1]))
        return self.bsfc_scale * float(b)


class Prop:
    """Mejzlik table (manufacturer) with J-similarity outside the tabulated rpm range and density scaling; WOT thrust
    x k_wot (Falcon/Mejzlik ratio), part-throttle thrust x k_inst (pusher installation). Endurance study class.

    ``k_inst_wot_ramp`` (V2-04, fix round 3; None = endurance-study behaviour): the pusher installation loss also acts
    at full throttle once the aircraft moves; the WOT thrust is multiplied by k(V) = 1 - (1 - k_inst) min(V / ramp, 1),
    i.e. 1.0 static (aero.yaml: static thrust ~ uninstalled) rising to the full k_inst at ``ramp`` m/s."""
    NGRID = np.linspace(1200.0, 8000.0, 273)

    def __init__(self, rows: dict, D: float, k_wot: float, k_inst: float, eng: Engine,
                 k_inst_wot_ramp: float | None = None):
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
        self.k_inst_wot_ramp = k_inst_wot_ramp
        self.eng = eng

    def k_wot_installed(self, V: float) -> float:
        """Installation factor on the full-throttle thrust at speed V (1.0 without the V2-04 ramp)."""
        if not self.k_inst_wot_ramp:
            return 1.0
        return 1.0 - (1.0 - self.k_inst) * min(max(V, 0.0) / float(self.k_inst_wot_ramp), 1.0)

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
        Tt = sig * float(Tn[0]) * self.k_wot * self.k_wot_installed(V)
        tip = math.hypot(math.pi * self.D * nn / 60, V) / atm["a"]
        return {"rpm": nn, "T": Tt, "P_shaft": P, "eta": Tt * V / P if P > 0 and V > 0 else 0.0, "tip_mach": tip}

    def at_rpm(self, V: float, h: float, n: float) -> dict:
        """Part-throttle point at a fixed rpm ``n`` (descent at the generator floor): installed thrust (x k_inst, may
        be negative = windmilling drag) and the propeller shaft power (negative = the propeller drives the engine)."""
        atm = AL.isa(h)
        sig = atm["sigma"]
        T, Q = self.sl(V, n)
        P = sig * float(Q[0]) * 2 * math.pi * n / 60
        Ts = sig * float(T[0])
        return {"rpm": float(n), "T": Ts * self.k_inst if Ts > 0.0 else Ts, "P_shaft": P}


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
    up = sstep((x - L["x_belly0"]) / L.get("l_belly_ramp_fwd", L["l_belly_ramp"])) * \
        (1 - sstep((x - L["x_belly1"]) / L["l_belly_ramp"]))
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


# ---------------------------------------------------------------------------------------------- LERX/glove airfoils
# V1-06: the LERX/glove sections morph from a symmetric LERX section at the chine to NLF(1)-0416 at the outer-panel
# joint in GLOVE_BLEND_STEPS (no change of airfoil family between neighbouring sections). The LERX section is a NACA
# four-digit-modified symmetric section 00tt-Im (Abbott & von Doenhoff 1959 sec. 6.5; ordinates as in NASA TM-4741,
# Ladson et al. 1996) given by wing.planform.lerx_section {t, I, m}: leading-edge radius index I and maximum thickness
# at x/c = m, aft toward the spar box of the long LERX chord, so the spar-depth rule does not thicken the forward LERX
# into a blister. The blends interpolate the two unit sections point by point (thickness and camber). File names encode
# the section parameters (the NeuralFoil polar cache is keyed by airfoil name).
GLOVE_LERX_DEFAULT = {"t": 0.16, "I": 3.0, "m": 0.6}
GLOVE_BLEND_STEPS = (0.0, 0.25, 0.5, 0.75)


def naca4_modified(t: float, I: float, m: float, n: int = 161) -> np.ndarray:
    """Symmetric NACA four-digit-modified section 00tt-Im (Selig order, unit chord): y_t = (t/0.2)(a0 sqrt(x) + a1 x
    + a2 x^2 + a3 x^3) ahead of the maximum thickness at x = m, (t/0.2)(d0 + d1 (1-x) + d2 (1-x)^2 + d3 (1-x)^3) aft
    of it; a0 = 0.296904 I/6 (leading-edge radius 1.1019 (t I/6)^2), d0 = 0.002, d1 from the published table
    (m 0.2..0.6: 0.200, 0.234, 0.315, 0.465, 0.700); d2, d3 from y = 0.1 and y' = 0 at x = m; a1..a3 from y = 0.1,
    y' = 0 and continuous curvature at x = m."""
    d1 = float(np.interp(m, [0.2, 0.3, 0.4, 0.5, 0.6], [0.200, 0.234, 0.315, 0.465, 0.700]))
    d0, s = 0.002, 1.0 - m
    A = np.array([[s ** 2, s ** 3], [2 * s, 3 * s ** 2]])
    d2, d3 = np.linalg.solve(A, [0.1 - d0 - d1 * s, -d1])
    ypp = 2 * d2 + 6 * d3 * s                                       # y'' at x = m (aft polynomial in 1 - x)
    a0 = 0.296904 * I / 6.0
    M = np.array([[m, m ** 2, m ** 3], [1.0, 2 * m, 3 * m ** 2], [0.0, 2.0, 6 * m]])
    rhs = [0.1 - a0 * math.sqrt(m), -0.5 * a0 / math.sqrt(m), ypp + 0.25 * a0 * m ** -1.5]
    a1, a2, a3 = np.linalg.solve(M, rhs)
    x = 0.5 * (1 - np.cos(np.linspace(0.0, math.pi, n)))
    yf = a0 * np.sqrt(x) + a1 * x + a2 * x ** 2 + a3 * x ** 3
    ya = d0 + d1 * (1 - x) + d2 * (1 - x) ** 2 + d3 * (1 - x) ** 3
    yt = t / 0.2 * np.where(x <= m, yf, ya)
    return np.vstack([np.column_stack([x, yt])[::-1], np.column_stack([x, -yt])[1:]])


def lerx_section(P: dict | None) -> dict:
    L = dict(GLOVE_LERX_DEFAULT)
    L.update((P or {}).get("lerx_section", {}))
    return {k: float(v) for k, v in L.items()}


def glove_airfoil_name(w: float, L: dict | None = None) -> str:
    """Airfoil of a LERX/glove section with blend weight w (0 = LERX section, 1 = NLF(1)-0416)."""
    L = L or GLOVE_LERX_DEFAULT
    tag = f"{int(round(100 * L['t'])):02d}{int(round(L['I']))}{int(round(10 * L['m']))}"
    return "nlf416" if w >= 1.0 - 1e-9 else f"hancer_lerx{tag}_w{int(round(100 * w)):03d}"


def glove_airfoil_coords(w: float, L: dict | None = None, n: int = 161) -> np.ndarray:
    """Unit coordinates (Selig order) of the blend w between the LERX section and NLF(1)-0416 at common cosine
    stations (upper and lower surfaces blended point by point; blunt TE of the LERX section kept in proportion)."""
    L = L or GLOVE_LERX_DEFAULT
    A = naca4_modified(L["t"], L["I"], L["m"], n)
    xa, yua, yla = A[:n][::-1, 0], A[:n][::-1, 1], A[n - 1:, 1]
    xb, yub, ylb = oml.resampled("nlf416", n, 0.0, 1.0)
    assert np.allclose(xa, xb)
    yu = (1 - w) * yua + w * yub
    yl = (1 - w) * yla + w * ylb
    return np.vstack([np.column_stack([xa, yu])[::-1], np.column_stack([xa, yl])[1:]])


def write_glove_airfoils(S: dict | None = None, check_only: bool = False) -> list:
    """Write (or with ``check_only`` compare) the LERX/glove blend sections of the spec (wing.planform.lerx_section) as
    ucav250/data/airfoils/<name>.dat (Selig format). Returns the names whose file is missing or differs."""
    bad = []
    L = lerx_section(S["wing"]["planform"] if S else None)
    for w in GLOVE_BLEND_STEPS:
        name = glove_airfoil_name(w, L)
        P = glove_airfoil_coords(w, L)
        head = (f"HANCER LERX/GLOVE BLEND w={w:.2f}: (1-w) NACA 00{int(round(100 * L['t'])):02d}-{int(L['I'])}"
                f"{int(round(10 * L['m']))} (four-digit modified) + w NLF(1)-0416 (sizing.py glove_airfoil_coords)")
        text = head + "\n" + "\n".join(f"  {x:.7f}  {y:.7f}" for x, y in P) + "\n"
        path = oml.AIRFOIL_DIR / f"{name}.dat"
        if not path.exists() or path.read_text(encoding="utf-8") != text:
            bad.append(name)
            if not check_only:
                path.write_text(text, encoding="utf-8")
    if not check_only:
        oml.read_airfoil.cache_clear()
        oml.resampled.cache_clear()
    return bad


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

    LERX/glove sections (V1-06: the LERX section near the apex morphing in GLOVE_BLEND_STEPS into NLF(1)-0416 between
    the span fractions ``glove_blend_u``, so no two neighbouring sections change airfoil family):
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
        af_name = glove_airfoil_name(glove_blend_weight(P, u), lerx_section(P))
        t_ref = oml.max_thickness(af_name)[0]
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


def glove_blend_weight(P: dict, u: float) -> float:
    """Blend weight of the LERX/glove section at span fraction u of the LERX (0 at the chine apex, 1 at the joint):
    linear between planform.glove_blend_u = [u0, u1], rounded to the nearest of GLOVE_BLEND_STEPS + [1]."""
    u0, u1 = (float(v) for v in P["glove_blend_u"])
    w = float(np.clip((u - u0) / max(u1 - u0, 1e-9), 0.0, 1.0))
    return min(list(GLOVE_BLEND_STEPS) + [1.0], key=lambda s_: abs(s_ - w))


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
      factors (section cl_max x ``k_sec``, 3-D x ``k_3d``, endurance study) and x cos(quarter-chord sweep). The stall search is restricted to the
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
    # simple-sweep reduction of the stall lift (Raymer 6th ed. eq. 12.15: CLmax ~ clmax cos(sweep c/4)), the outer-panel
    # quarter-chord sweep (fix round 2: the lifting line itself neglects sweep)
    k_sweep = math.cos(math.radians(float(S["wing"]["planform"]["sweep_c4_deg"])))
    CLmax_LL = k_3d * k_sweep * LL["CL"](float(a_crit[i]))
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
def mass_cases(S: dict, own_inertia: dict | None = None) -> list[dict]:
    """CG of every loading case from spec mass items (empty incl. growth), payload items and fuel. With
    ``own_inertia`` (distributed_pitch_inertia: specific own pitch inertia per item name and for the fuel, m^2) every
    case also carries its pitch moment of inertia about its CG, I_yy (kg m^2; point masses + the own inertia of the
    distributed items)."""
    M = S["mass"]
    items = M["items"]
    pay = {p["name"]: p for p in M["payload_items"]}
    fuel = M["fuel_kg"]
    fx, fy, fz = M["fuel_cg"]
    out = []
    for c in M["cases"]:
        its = [(i["mass_kg"], i["x"], i["y"], i["z"], i["name"]) for i in items]
        names = c.get("payload", c.get("payload_items", []))
        for name in names:
            p = pay[name]
            z = p["z_extended"] if (c.get("turret") == "extended" and "z_extended" in p) else p["z"]
            its.append((p["mass_kg"], p["x"], p.get("y", 0.0), z, name))
        its.append((fuel * c["fuel_fraction"], fx, fy, fz, "_fuel"))
        m = sum(i[0] for i in its)
        cg = [sum(i[0] * i[k] for i in its) / m for k in (1, 2, 3)]
        row = {"name": c["name"], "m": m, "x": cg[0], "y": cg[1], "z": cg[2],
               "fuel_fraction": c["fuel_fraction"], "payload_kg": sum(pay[n]["mass_kg"] for n in names)}
        if own_inertia is not None:
            row["I_yy"] = sum(i[0] * ((i[1] - cg[0]) ** 2 + (i[3] - cg[2]) ** 2 + own_inertia.get(i[4], 0.0))
                              for i in its)
        out.append(row)
    return out


def distributed_pitch_inertia(S: dict, af: "Airframe") -> dict:
    """V4-05 (fix round 5): specific own pitch inertia (m^2 = variance of the mass distribution in x and z about the
    distribution's own centroid; I_own = m x value) of the distributed mass items, for the pitch moment of inertia of
    the loading cases (mass_cases). Estimate: skins and lifting-surface structures spread uniformly over their exposed
    wetted areas (OML meshes; body skin minus the parts covered by the wing and tail roots); the chine bands, keel beams,
    frames and harness as uniform rods along the body (chine bands and frames over the body length, keel beams over
    their 0.82 L length, harness over the body length); fuel uniform over the fuel-cell rectangles (x-z, layout.fuel_cells).
    Every other item (engine, gear, equipment, payload, control surfaces) is a point mass at its position."""
    def tri_var(mesh, keep):
        tri = mesh.V[mesh.F]
        c = tri.mean(axis=1)
        ar = 0.5 * np.linalg.norm(np.cross(tri[:, 1] - tri[:, 0], tri[:, 2] - tri[:, 0]), axis=1)
        k = keep(c)
        c, ar = c[k], ar[k]
        w = ar / max(ar.sum(), 1e-12)
        xm, zm = float(np.sum(w * c[:, 0])), float(np.sum(w * c[:, 2]))
        return float(np.sum(w * ((c[:, 0] - xm) ** 2 + (c[:, 2] - zm) ** 2)))

    def body_keep(c):
        cov = np.zeros(len(c), bool)
        for surf, mir in [(af.wing, True)] + [(af.tail[k], af.tail_mirror[k]) for k in af.tail]:
            cov |= af._inside_surface(surf, c, mir)
        return ~cov
    outside = lambda c: ~af.inside(c)                        # noqa: E731
    L = float(af.L)
    out = {"wing_structure_pair": tri_var(af.wing_mesh, outside),
           "body_skin_sandwich": tri_var(af.body_mesh, body_keep),
           "chine_edge_bands": L ** 2 / 12.0, "frames_bulkheads": L ** 2 / 12.0,
           "keel_beams_longerons": (0.82 * L) ** 2 / 12.0, "wiring_harness_connectors_coax": L ** 2 / 12.0}
    tm = af.tail_meshes
    for item, k in (("stabilators_pair", "stabilator"), ("stabilator_root_stubs_pair", "stabilator_stub"),
                    ("fins_pair_fixed", "fin"), ("ventral_fin_bumper_skid", "ventral")):
        if k in tm:
            out[item] = tri_var(tm[k], outside)
    cells = S["layout"].get("fuel_cells") or []
    if cells:
        A = np.array([(c["x"][1] - c["x"][0]) * (c["z"][1] - c["z"][0]) for c in cells])
        xc = np.array([0.5 * (c["x"][0] + c["x"][1]) for c in cells])
        zc = np.array([0.5 * (c["z"][0] + c["z"][1]) for c in cells])
        own = np.array([((c["x"][1] - c["x"][0]) ** 2 + (c["z"][1] - c["z"][0]) ** 2) / 12.0 for c in cells])
        w = A / A.sum()
        xm, zm = float(np.sum(w * xc)), float(np.sum(w * zc))
        out["_fuel"] = float(np.sum(w * (own + (xc - xm) ** 2 + (zc - zm) ** 2)))
    return out


def loading_cg_fn(S: dict, case_index: int = 0):
    """(f, m_zero_fuel): x of the CG of loading case ``case_index`` as a function f(m_fuel) of the fuel mass on board
    (fuel at the fuel-cell centroid mass.fuel_cg) and the zero-fuel mass of that loading."""
    M = S["mass"]
    c = M["cases"][case_index]
    pay = {p["name"]: p for p in M["payload_items"]}
    m_z = sum(i["mass_kg"] for i in M["items"])
    mx = sum(i["mass_kg"] * i["x"] for i in M["items"])
    for n in c.get("payload", []):
        m_z += pay[n]["mass_kg"]
        mx += pay[n]["mass_kg"] * pay[n]["x"]
    fx = float(M["fuel_cg"][0])
    return (lambda mf: (mx + mf * fx) / (m_z + mf)), m_z


def trimmed_clmax_at(S: dict, wa: dict, stab: dict, tail: dict, x_cg: float) -> float:
    """Trimmed clean CLmax at the CG ``x_cg`` (same relation as clmax_set: wing CLmax + stabilator trim load)."""
    cbar = float(S["wing"]["mac"])
    cm = stab["Cm0_wb"] + wa["CLmax"] * (x_cg - stab["x_ac_wb"]) / cbar
    return wa["CLmax"] + cm * cbar / (tail["x_ac"] - x_cg)


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


def hoerner_junction_Dq(t: float, tc: float) -> float:
    """Interference drag D/q (m2) of one plain (unfilleted) junction of a surface root of thickness ``t`` and
    thickness ratio ``tc`` with a body (Hoerner 1965, Fluid-Dynamic Drag ch. 8: dD/q = t^2 (0.75 t/c - 0.0003/(t/c)^2))."""
    return max(0.75 * tc - 0.0003 / tc ** 2, 0.0) * t * t


def gear_door_geometry(S: dict) -> dict:
    """Plan-view door areas (m2) and sealed joint lengths (m) of the gear wells from the stowed-envelope boxes and the
    door scheme (landing_gear.doors, V3-10; V4-03 fix round 5): per main well a sequenced inner door (centre-line gap
    to main_inner_door_outer_edge_y) and a leg door (from there to the outer well edge); the nose keel slot closed by
    two clamshell doors. Joints: the outline of each main well, the inner/leg door split of each main well, the
    outline of the nose slot and the clamshell centre split."""
    LG = S["landing_gear"]
    mb = np.asarray(LG["main"]["stowed_envelope"]["box"], float)
    nb = np.asarray(LG["nose"]["stowed_envelope"]["box"], float)
    y0 = 0.5 * float(LG["main"]["well_gap"])
    y_s = float(LG.get("doors", {}).get("main_inner_door_outer_edge_y", y0))
    dx_m, dx_n = mb[1, 0] - mb[0, 0], nb[1, 0] - nb[0, 0]
    a_inner = 2 * dx_m * max(y_s - y0, 0.0)
    a_leg = 2 * dx_m * (mb[1, 1] - max(y_s, y0))
    a_nose = dx_n * (nb[1, 1] - nb[0, 1])
    j_main = 2 * 2 * (dx_m + (mb[1, 1] - y0)) + (2 * dx_m if y_s > y0 else 0.0)
    j_nose = 2 * (dx_n + (nb[1, 1] - nb[0, 1])) + dx_n
    return {"area_main_inner_m2": float(a_inner), "area_main_leg_m2": float(a_leg), "area_nose_m2": float(a_nose),
            "area_total_m2": float(a_inner + a_leg + a_nose), "joint_main_m": float(j_main), "joint_nose_m": float(j_nose),
            "joint_total_m": float(j_main + j_nose), "x_main": float(0.5 * (mb[0, 0] + mb[1, 0])),
            "x_nose": float(0.5 * (nb[0, 0] + nb[1, 0]))}


def gear_door_joint_length(S: dict) -> float:
    """Length (m) of the sealed door joints of the gear wells (gear_door_geometry: well outlines + the door split
    lines; V4-03, fix round 5: the inner/leg door splits and the clamshell centre split are joints too)."""
    return gear_door_geometry(S)["joint_total_m"]


def gear_doors_mass(S: dict) -> dict:
    """V4-03 (fix round 5): base mass (kg, before the growth allowance) and position of the gear doors, wells, locks and
    door drives from the door scheme (landing_gear.doors) and mass.rules.gear_doors: door area x the areal mass of the
    identity-study basis (sandwich doors incl. hinges and links), well close-outs, cut-out reinforcement, gear
    locks/sensors, perimeter seals per metre of door joint (only with sealed doors), two door actuators for the
    sequenced inner doors (catalogue unit) + their linkages, and the standoff brackets of the leg doors. x: mass-weighted
    over the main-well and nose-well parts (close-outs, reinforcement and locks 2/3 main, 1/3 nose: three wells)."""
    R = S["mass"]["rules"]["gear_doors"]
    g = gear_door_geometry(S)
    sealed = bool(S["aero"]["drag_rules"].get("gear_doors_sealed"))
    ak = float(R["door_areal_kg_per_m2"])
    seal = float(R["seal_kg_per_m"]) if sealed else 0.0
    act = R["inner_door_actuator"]
    m_act = float(research_item(act["mass_ref"])) * int(act["count"])
    br = R["leg_door_brackets"]
    fixed = float(R["well_close_outs_kg"]) + float(R["cut_out_reinforcement_kg"]) + float(R["locks_sensors_kg"])
    parts = {"doors": ak * g["area_total_m2"], "seals": seal * g["joint_total_m"], "well_close_outs": float(R["well_close_outs_kg"]),
             "cut_out_reinforcement": float(R["cut_out_reinforcement_kg"]), "locks_sensors": float(R["locks_sensors_kg"]),
             "inner_door_actuators": m_act, "inner_door_linkages": float(act["linkage_kg_each"]) * int(act["count"]),
             "leg_door_brackets": float(br["mass_kg_each"]) * int(br["count"])}
    m_main = (ak * (g["area_main_inner_m2"] + g["area_main_leg_m2"]) + seal * g["joint_main_m"] + 2.0 / 3.0 * fixed +
              parts["inner_door_actuators"] + parts["inner_door_linkages"] + parts["leg_door_brackets"])
    m_nose = ak * g["area_nose_m2"] + seal * g["joint_nose_m"] + fixed / 3.0
    total = sum(parts.values())
    return {"total_kg": total, "parts_kg": parts, "geometry": g, "sealed": sealed,
            "x": (m_main * g["x_main"] + m_nose * g["x_nose"]) / total}


def tail_junctions(S: dict) -> list:
    """Root junctions of the tail surfaces with the body: [{name, n, t, tc, Dq}] (F12)."""
    TS = S["tail"]["surfaces"]
    out = []
    for k, n in (("fin", 2), ("stabilator_stub", 2), ("ventral", 1)):
        if k not in TS:
            continue
        s0 = TS[k]["sections"][0]
        tc = oml.max_thickness(s0["airfoil"])[0] * float(s0.get("thickness_scale", 1.0))
        t = tc * float(s0["chord"])
        out.append({"name": k, "n": n, "t": t, "tc": tc, "Dq": n * hoerner_junction_Dq(t, tc)})
    return out


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
    if cfg.get("gear") == "down":
        items["landing_gear"] = down
    elif A.get("gear_doors_sealed"):
        # flush doors with perimeter seals (design choice): gap drag per metre of sealed door joint, the rule of the
        # turret bay doors and of the sealed control-surface hinge gaps (no well leakage)
        items["landing_gear"] = float(A["door_joint_Dq_per_m"]) * gear_door_joint_length(S)
    else:                                                     # unsealed doors: residual share of the extended drag
        items["landing_gear"] = A["gear_residual_fraction"] * down
    # EO/IR turret: exposed ball (Hoerner sphere CD) + stem when extended; door gaps when retracted
    P = S["payload"]["turret"]
    if cfg.get("turret") == "extended":
        d_b = P["ball_diameter"]
        zs = af.z_bot(P["bay_center_x"])
        f_exp = min(max((zs - (P["ball_center_extended_z"] - d_b / 2)) / d_b, 0.0), 1.0)
        a_ball = math.pi / 4 * d_b ** 2 * (0.5 * (1 - math.cos(math.pi * f_exp)))    # exposed share of the disc
        base = max(zs - (P["ball_center_extended_z"] + d_b / 2), 0.0)
        # open bay around the lowered ball (doors folded inward): cavity drag on the open area at the skin plane;
        # with the aperture ring (payload.turret.bay.aperture_ring) only the annulus between the ring and the ball
        # section at the skin plane is open
        hw = P["growth_envelope"]["diameter"] / 2 + 0.01
        dz = zs - P["ball_center_extended_z"]
        r_skin = math.sqrt(max((d_b / 2) ** 2 - dz ** 2, 0.0)) if abs(dz) < d_b / 2 else 0.0
        stem = math.pi / 4 * P["stem_diameter"] ** 2 if dz >= d_b / 2 else 0.0
        ring = P["bay"].get("aperture_ring")
        if ring:
            # V1-07: the aperture ring follows the real (V-keel) skin; a point of the ring opening is open where the
            # lowered ball's upper surface lies below the local skin (gap between ball and skin), integrated on a
            # polar grid inside the ring radius (the stem plugs the centre when the ball is below the skin)
            r_ring = 0.5 * d_b + float(ring["radial_clearance"])
            rr = (np.arange(40) + 0.5) / 40 * r_ring
            th_ = (np.arange(72) + 0.5) / 72 * 2 * math.pi
            R_, T_ = np.meshgrid(rr, th_)
            X_ = P["bay_center_x"] + R_ * np.cos(T_)
            Y_ = R_ * np.sin(T_)
            z_skin = np.asarray(af.z_bot(X_.ravel(), Y_.ravel()), float).reshape(R_.shape)
            r_b = 0.5 * d_b
            z_ball = np.where(R_ < r_b, P["ball_center_extended_z"] + np.sqrt(np.clip(r_b ** 2 - R_ ** 2, 0, None)),
                              -np.inf)
            z_ball = np.where(R_ < 0.5 * P["stem_diameter"], np.inf, z_ball) if dz >= r_b else z_ball
            dA = R_ * (r_ring / 40) * (2 * math.pi / 72)
            a_open = float(np.sum(dA * (z_skin > z_ball)))
        else:
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
    items["wing_body_junctions"] = 2 * hoerner_junction_Dq(t_j, tc_j)
    # tail-body junctions (F12): canted-fin roots on the dorsal skin, fixed stabilator root stubs on the body sides,
    # ventral-fin root on the keel; same plain-junction relation (no fillet credit)
    items["tail_body_junctions"] = sum(r_["Dq"] for r_ in tail_junctions(S))
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
    # V2-08 (fix round 3): L/D and endurance-parameter maxima only up to the trimmed CLmax of the configuration (the
    # table continues beyond the stall for interpolation only); gear down = take-off/landing, landing flap setting
    cl_top = float(clmax["ld_trimmed"] if name == "gear_down" else clmax["clean_trimmed"])
    ok = CLs <= cl_top + 1e-9
    ld = np.where(ok, CLs / CDs, -np.inf)
    i = int(np.argmax(ld))
    ep = np.where((CLs > 0) & ok, np.clip(CLs, 0, None) ** 1.5 / CDs, 0)
    j = int(np.argmax(ep))
    AR = float(S["wing"]["aspect_ratio"])
    fit = {"cd0": cd0, "k": k, "e": 1.0 / (math.pi * AR * k), "LD_max": float(ld[i]), "CL_LDmax": float(CLs[i]),
           "CL_endurance": float(CLs[j]), "endurance_param_max": float(ep[j]), "cd_rest": cd_rest,
           "CL_max_trimmed": cl_top}
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
                 cool_ref: dict | None = None, mission_clmax=None, floor_peak_W: float | None = None):
        self.S = S
        self.Sw = float(S["wing"]["area"])
        self.eng = eng
        self.prop = prop
        self.pol = polars
        # trimmed clean CLmax of the design-mission loading as a function of the weight (N): the stall-speed floors of
        # the mission (1.2 VS) use the CG the aircraft actually has on that mission (design payload, fuel burning
        # off); None = the forward-most CG of all loading cases (R-08 basis)
        self.mission_clmax = mission_clmax
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
        # loiter rpm floor: the SG750 DC output is proportional to the rpm (generator_dc_W), so the loiter speed is
        # raised where needed to keep the rpm at which the generator carries (mission.loiter_rpm_floor):
        # "e180_peak_battery_support" (fix round 3, R-52): the E180 peak minus peak_support_deficit_cap_W (the battery
        # peak-support share covers the deficit); "generator_peak" (v1.2): the whole E180 peak; "installed_turret_peak"
        # (v1.3, withdrawn): the peak of the installed turret (``floor_peak_W``); None = minimum fuel flow only
        self.rpm_floor = None
        mode = str(mis.get("loiter_rpm_floor", "none"))
        if mode in ("generator_peak", "installed_turret_peak", "e180_peak_battery_support") and \
                S["engine"].get("electrical_budget"):
            eb = electrical_budget(S, 0.0)
            if mode == "generator_peak":
                pk = eb["peak_e180_W"]
            elif mode == "e180_peak_battery_support":
                # V2-01: generator output at most peak_support_deficit_cap_W below the E180 peak; the battery peak-support
                # share supplies the rest (R-52 checks the energy)
                pk = eb["peak_e180_W"] - float(mis["peak_support_deficit_cap_W"])
            else:
                pk = floor_peak_W or eb["peak_hd59_W"]
            self.rpm_floor = generator_rpm_for_dc(S, pk)
        self.floor_peak_W = floor_peak_W
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

    def vstall_mission(self, W: float, h: float) -> float:
        """Stall speed of the design-mission loading (clean, trimmed at its own CG)."""
        return self.vstall(W, h, self.mission_clmax(W) if self.mission_clmax else None)

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
                "gen_W": generator_dc_W(self.S, p["rpm"])}

    def v_floor(self, W: float, h: float, pol: str = "clean") -> float:
        sig = AL.isa(h)["sigma"]
        return max(self.v_floor_eas / math.sqrt(sig), 1.2 * self.vstall_mission(W, h))

    def best_loiter(self, W: float, h: float, pol: str = "loiter", floor: bool = True, rpm_floor: bool = False) -> dict:
        vlo = self.v_floor(W, h) if floor else 1.2 * self.vstall_mission(W, h)
        f = lambda V: (self.level_point(W, V, h, pol) or {"ff_kg_s": 1e9})["ff_kg_s"]   # noqa: E731
        V = max(golden(f, vlo, vlo + 20.0), vlo)
        p = self.level_point(W, V, h, pol)
        if rpm_floor and self.rpm_floor and p and p["rpm"] < self.rpm_floor:
            # raise the speed until the rpm reaches the generator floor (rpm rises with speed above the minimum-fuel
            # speed): bisection on V
            lo, hi = V, V + 20.0
            for _ in range(40):
                mid = 0.5 * (lo + hi)
                pm = self.level_point(W, mid, h, pol)
                if pm is not None and pm["rpm"] >= self.rpm_floor:
                    hi = mid
                else:
                    lo = mid
            p = self.level_point(W, hi, h, pol)
        return p

    def best_range(self, W: float, h: float, pol: str = "clean") -> dict:
        vlo = max(1.2 * self.vstall_mission(W, h), self.v_cruise_min)
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
                        "tip_mach": w["tip_mach"], "ff_kg_h": b * P_tot / 1.0e6}      # g/kWh x W -> kg/h
        return best

    # ------------------------------------------------------------------ mission (direct fuel-flow integration)
    # V1-01: the fuel of every flown segment is the time integral of the point fuel flow (bsfc x (propeller shaft
    # power + generator shaft power) at the trimmed level/climb point, thrust T = D / cos(eps)); midpoint rule in
    # steps of STEP_CLIMB_M altitude, STEP_CRUISE_M distance and STEP_LOITER_S time (weight at the step midpoint from
    # a predictor step). The closed-form Breguet fractions of sizinglib are no longer used for flown segments: with
    # the inclined thrust line their L/D (= CL/CD incl. the lift that carries the thrust component) understated the
    # fuel flow by ~1 %. Warm-up, taxi, take-off and landing keep the sizinglib fixed fractions; the descent is
    # integrated too (V2-07, fix round 3: engine at the generator floor rpm, propeller table, force balance).
    STEP_CLIMB_M = 500.0
    STEP_CRUISE_M = 25.0e3
    STEP_LOITER_S = 1800.0

    def _state_key(self, *a) -> tuple:
        """Cache key: arguments + every model state the sensitivity runs change (BSFC scale, polar tables)."""
        return (tuple(round(float(x), 9) if isinstance(x, (int, float)) else x for x in a), self.eng.bsfc_scale,
                self.eng.low_load_model, tuple(round(float(np.sum(P.CDs)), 12) for P in self.pol.values()))

    def _cache(self, name: str) -> dict:
        if not hasattr(self, "_caches"):
            self._caches = {}
        return self._caches.setdefault(name, {})

    def _row(self, kind: str, name: str, W0: float, dt: float, fuel_kg: float, p0: dict, pm: dict, p1: dict | None,
             extra: dict | None = None) -> dict:
        """Log row of one integration step: start / midpoint / end point fuel flows and the step fuel."""
        r = {"kind": kind, "name": name, "dt_s": dt, "W_start_N": W0, "W_end_N": W0 - fuel_kg * G, "fuel_kg": fuel_kg,
             "fraction": 1.0 - fuel_kg * G / W0, "ff_kg_h": pm["ff_kg_h"], "ff_start_kg_h": p0["ff_kg_h"],
             "ff_mid_kg_h": pm["ff_kg_h"], "ff_end_kg_h": None if p1 is None else p1["ff_kg_h"]}
        for k in ("V", "rpm", "CL", "power_fraction", "gen_W", "EAS", "LD", "roc"):
            if k in pm:
                r[k] = pm[k]
        r.update(extra or {})
        return r

    def _climb(self, m0: float, h: float) -> dict:
        """Full-throttle climb to h at the best rate of climb, integrated in STEP_CLIMB_M steps (energy height:
        altitude + kinetic energy change between steps)."""
        key = self._state_key("climb", m0, h)
        C = self._cache("climb")
        if key in C:
            return C[key]
        W = m0 * G
        for k in ("warmup", "taxi", "takeoff"):
            W *= SZ.FIXED[k]
        n = max(int(math.ceil(h / self.STEP_CLIMB_M - 1e-9)), 1)
        dh = h / n
        rows, t, V_prev = [], 0.0, None
        for i in range(n):
            hm = (i + 0.5) * dh
            p0 = self.climb_point(W, hm)
            V0 = p0["V"]
            dhe0 = dh + (0.0 if V_prev is None else (V0 ** 2 - V_prev ** 2) / (2 * G))
            dt0 = dhe0 / p0["roc"]
            pm = self.climb_point(W - 0.5 * p0["ff_kg_h"] / 3600.0 * dt0 * G, hm)
            dhe = dh + (0.0 if V_prev is None else (pm["V"] ** 2 - V_prev ** 2) / (2 * G))
            dt = dhe / pm["roc"]
            fuel = pm["ff_kg_h"] / 3600.0 * dt
            rows.append(self._row("climb", "climb", W, dt, fuel, p0, pm, None, {"h_from_m": i * dh, "h_to_m": (i + 1) * dh,
                                                                               "dh_energy_m": dhe}))
            rows[-1]["ff_start_kg_h"] = None                 # climb steps: the point is at the step mid-altitude
            W -= fuel * G
            t += dt
            V_prev = pm["V"]
        C[key] = {"rows": rows, "W": W, "t": t}
        return C[key]

    def _trajectory(self, kind: str, W0: float, h: float, need: float, pol: str) -> list:
        """Cached integration nodes of a cruise (best range, distance steps) or loiter (best loiter with the
        generator rpm floor, time steps) from the start weight W0, extended until the coordinate ``need``:
        [{"x": coordinate at the step start, "W": weight, "t": time, "row": step row (to the next node)}]."""
        key = self._state_key(kind, W0, h, pol)
        C = self._cache("traj")
        nodes = C.setdefault(key, [{"x": 0.0, "W": W0, "t": 0.0, "row": None, "p": None}])
        if kind == "cruise":
            pt = lambda W: self.best_range(W, h, pol)                                      # noqa: E731
            step = self.STEP_CRUISE_M
        else:
            pt = lambda W: self.best_loiter(W, h, pol, rpm_floor=True)                     # noqa: E731
            step = self.STEP_LOITER_S
        while nodes[-1]["x"] < need - 1e-6:
            nd = nodes[-1]
            if nd["W"] < 0.5 * W0:
                raise ValueError("mission trajectory extended beyond half the start weight")
            W = nd["W"]
            p0 = nd["p"] or pt(W)
            if p0 is None:
                raise ValueError(f"no {kind} point at W = {W:.1f} N, h = {h:.0f} m")
            if kind == "cruise":
                pm = pt(W - 0.5 * p0["ff_kg_s"] * step / p0["V"] * G)
                dt = step / pm["V"]
            else:
                pm = pt(W - 0.5 * p0["ff_kg_s"] * step * G)
                dt = step
            fuel = pm["ff_kg_s"] * dt
            W1 = W - fuel * G
            p1 = pt(W1)
            nd["p"] = p0
            nd["row"] = self._row(kind, "", W, dt, fuel, p0, pm, p1)
            nodes.append({"x": nd["x"] + step, "W": W1, "t": nd["t"] + dt, "row": None, "p": p1})
        return nodes

    def _along(self, kind: str, name: str, W0: float, h: float, length: float, pol: str) -> dict:
        """Rows of a cruise/loiter segment of ``length`` (m or s) from W0: the cached full steps plus a partial last
        step (weight and time linear between the two nodes; fuel flow constant inside a step = midpoint value)."""
        rows = []
        if length <= 1e-9:
            return {"rows": rows, "W": W0, "t": 0.0}
        nodes = self._trajectory(kind, W0, h, length, pol)
        W, t = W0, 0.0
        for i in range(len(nodes) - 1):
            a, b = nodes[i], nodes[i + 1]
            if b["x"] <= length + 1e-6:
                r = dict(a["row"], name=name)
                rows.append(r)
                W, t = b["W"], b["t"]
                if b["x"] >= length - 1e-6:
                    break
            else:
                f = (length - a["x"]) / (b["x"] - a["x"])
                r = dict(a["row"], name=name)
                dt = f * r["dt_s"]
                fuel = f * r["fuel_kg"]
                ff_e = r["ff_start_kg_h"] + f * (r["ff_end_kg_h"] - r["ff_start_kg_h"])
                r.update({"dt_s": dt, "fuel_kg": fuel, "W_end_N": a["W"] - fuel * G, "fraction": 1.0 - fuel * G / a["W"],
                          "ff_end_kg_h": ff_e, "ff_mid_kg_h": 0.5 * (r["ff_start_kg_h"] + ff_e), "partial_step": True})
                r["ff_kg_h"] = fuel / dt * 3600.0 if dt > 0 else r["ff_kg_h"]
                rows.append(r)
                W, t = a["W"] - fuel * G, a["t"] + dt
                break
        return {"rows": rows, "W": W, "t": t}

    def _integrate(self, kind: str, name: str, W0: float, h: float, length: float, pol: str) -> dict:
        """Uncached cruise/loiter segment (post-loiter transit and the reserve): midpoint steps of at most the
        nominal step length."""
        step = self.STEP_CRUISE_M if kind == "cruise" else self.STEP_LOITER_S
        n = max(int(math.ceil(length / step - 1e-9)), 1)
        dl = length / n
        if kind == "cruise":
            pt = lambda W: self.best_range(W, h, pol)                                      # noqa: E731
        else:
            pt = lambda W: self.best_loiter(W, h, pol)                                     # noqa: E731
        rows, W, t = [], W0, 0.0
        p0 = pt(W)
        for _ in range(n):
            if kind == "cruise":
                pm = pt(W - 0.5 * p0["ff_kg_s"] * dl / p0["V"] * G)
                dt = dl / pm["V"]
            else:
                pm = pt(W - 0.5 * p0["ff_kg_s"] * dl * G)
                dt = dl
            fuel = pm["ff_kg_s"] * dt
            p1 = pt(W - fuel * G)
            rows.append(self._row(kind, name, W, dt, fuel, p0, pm, p1))
            W -= fuel * G
            t += dt
            p0 = p1
        return {"rows": rows, "W": W, "t": t}

    def descent_rpm(self) -> float:
        """Lowest engine rpm in the descent: the generator must carry the continuous electrical load (the buffer battery
        is not drawn in normal flight), so the engine cannot idle below this rpm (V2-07)."""
        return generator_rpm_for_dc(self.S, self.eng.elec_load_W)

    def descent_point(self, W: float, h: float, pol: str = "clean") -> dict:
        """Steady descent at the mission sink rate (mission.descent_rate) with the engine at the generator floor rpm
        (descent_rpm): propeller thrust/torque from the table at that rpm (thrust x k_inst; windmilling thrust and
        torque may be negative), clean polar with the down-thrust lift component, cooling drag at this fuel flow.
        Path balance: W sin(gamma) = D - T cos(eps), sin(gamma) = sink / V. The speed that gives the mission sink rate
        is solved between the loiter speed floor (max(26 m/s EAS, 1.2 VS of the mission loading)) and the FCS speed
        limit (VNO); outside that band the speed is held at the band end and the sink rate follows from the balance.
        Fuel: bsfc x (generator shaft draw + propeller shaft power, the windmilling propeller credited with nothing)."""
        n = self.descent_rpm()
        atm = AL.isa(h)
        sig = math.sqrt(atm["sigma"])
        s_req = self.descent_rate
        P = self.pol[pol]
        V_lo = self.v_floor(W, h)
        ol = self.S["structures"].get("operating_limits", {})
        V_hi = max(float(ol.get("fcs_speed_limit_eas", self.S["structures"]["VC_eas"])) / sig, V_lo + 1.0)

        def state(V, sink):
            q = 0.5 * atm["rho"] * V * V
            pr = self.prop.at_rpm(V, h, n)
            T = pr["T"]
            P_tot = self.eng.p_gen_shaft + max(pr["P_shaft"], 0.0)
            ff = self.eng.bsfc(P_tot) * P_tot / 3.6e9
            gam = math.asin(min(max(sink / V, -0.5), 0.5))
            CL = (W * math.cos(gam) + T * math.sin(self.eps)) / (q * self.Sw)
            if CL > P.CLs[-1]:
                return None
            D = q * self.Sw * float(P.CD(CL)) + q * self.cooling_extra_Dq(V, h, ff)
            return {"V": V, "q": q, "T": T, "D": D, "CL": CL, "P_prop": pr["P_shaft"], "P_total": P_tot, "ff_kg_s": ff,
                    "f": D - T * math.cos(self.eps) - W * sink / V}
        lo, hi = state(V_lo, s_req), state(V_hi, s_req)
        if lo is not None and lo["f"] >= 0.0:
            st, V = lo, V_lo
        elif hi["f"] <= 0.0:
            st, V = hi, V_hi
        else:
            a, b = V_lo, V_hi
            for _ in range(40):
                m = 0.5 * (a + b)
                sm = state(m, s_req)
                if sm is not None and sm["f"] >= 0.0:
                    b = m
                else:
                    a = m
            V = b
            st = state(V, s_req)
        sink = s_req
        for _ in range(3):                                    # sink rate that closes the balance at the band ends
            sink = max((st["D"] - st["T"] * math.cos(self.eps)) * V / W, 0.05)
            st = state(V, sink)
        b_ = self.eng.bsfc(st["P_total"])
        return {"V": V, "EAS": V * sig, "h": h, "rpm": n, "sink_m_s": sink, "CL": st["CL"], "T": st["T"], "D": st["D"],
                "P_prop": st["P_prop"], "P_total": st["P_total"], "power_fraction": st["P_total"] / self.eng.P_max,
                "bsfc_g_kWh": b_, "ff_kg_s": st["ff_kg_s"], "ff_kg_h": st["ff_kg_s"] * 3600.0,
                "gen_W": generator_dc_W(self.S, n), "LD": st["CL"] * st["q"] * self.Sw / st["D"],
                "speed_band_end": "low" if V <= V_lo + 1e-9 else ("high" if V >= V_hi - 1e-9 else None)}

    def _descent(self, W0: float, h: float) -> dict:
        """Descent from h to sea level in STEP_CLIMB_M altitude steps (midpoint rule with a predictor step, the same
        scheme as the climb): dt = dh / sink, fuel = ff dt (V2-07)."""
        key = self._state_key("descent", W0, h)
        C = self._cache("descent")
        if key in C:
            return C[key]
        n = max(int(math.ceil(h / self.STEP_CLIMB_M - 1e-9)), 1)
        dh = h / n
        rows, W, t = [], W0, 0.0
        for i in range(n):
            h_top = h - i * dh
            hm = h_top - 0.5 * dh
            p0 = self.descent_point(W, hm)
            dt0 = dh / p0["sink_m_s"]
            pm = self.descent_point(W - 0.5 * p0["ff_kg_s"] * dt0 * G, hm)
            dt = dh / pm["sink_m_s"]
            fuel = pm["ff_kg_s"] * dt
            rows.append(self._row("descent", "descent", W, dt, fuel, p0, pm, None,
                                  {"h_from_m": h_top, "h_to_m": h_top - dh, "sink_m_s": pm["sink_m_s"],
                                   "P_prop_W": pm["P_prop"], "speed_band_end": pm["speed_band_end"]}))
            rows[-1]["ff_start_kg_h"] = None
            W -= fuel * G
            t += dt
        C[key] = {"rows": rows, "W": W, "t": t}
        return C[key]

    def fly(self, m0: float, t_loiter_s: float, R_transit: float | None = None, h: float | None = None,
            ferry: bool = False, loiter_pol: str = "loiter", **_ignored) -> dict:
        """Design mission (or a ferry flight): fixed fractions for warm-up/taxi/take-off/landing, integrated climb,
        transit (best range, turret retracted), loiter (best loiter with the generator rpm floor, turret extended),
        transit back, descent (generator floor rpm, mission sink rate; V2-07), reserve (10 % of the airborne time, best
        loiter at the reserve altitude, clean)."""
        R_transit = self.R_transit if R_transit is None else R_transit
        h = self.h_loiter if h is None else h
        W0 = m0 * G
        log = [{"kind": k, "name": k, "dt_s": 0.0, "W_start_N": W0 * math.prod(SZ.FIXED[q] for q in
                                                                                   ("warmup", "taxi", "takeoff")[:i]),
                "fraction": SZ.FIXED[k]} for i, k in enumerate(("warmup", "taxi", "takeoff"))]
        cl = self._climb(m0, h)
        log += cl["rows"]
        W, t_air = cl["W"], cl["t"]
        out = self._along("cruise", "transit_out", W, h, R_transit, "clean")
        log += out["rows"]
        W, t_air = out["W"], t_air + out["t"]
        lo = self._along("loiter", "loiter", W, h, t_loiter_s, loiter_pol)
        log += lo["rows"]
        W, t_air = lo["W"], t_air + lo["t"]
        W_loiter_end = W
        if not ferry and R_transit > 0:
            back = self._integrate("cruise", "transit_back", W, h, R_transit, "clean")
            log += back["rows"]
            W, t_air = back["W"], t_air + back["t"]
        de = self._descent(W, h)
        log += de["rows"]
        W, t_air = de["W"], t_air + de["t"]
        log.append({"kind": "landing", "name": "landing", "dt_s": 0.0, "W_start_N": W, "fraction": SZ.FIXED["landing"]})
        W *= SZ.FIXED["landing"]
        res = self._integrate("loiter", "reserve", W, self.h_reserve, self.reserve_frac * t_air, "clean")
        for r in res["rows"]:
            r["kind"] = "reserve"
        log += res["rows"]
        W = res["W"]
        segs = [SZ.Segment(r["kind"], name=r["name"], fraction=r["fraction"]) for r in log]
        ff, fr = SZ.mission_fuel_fraction(segs, trapped=self.trapped)
        return {"segments": segs, "log": log, "ff": ff, "fractions": fr, "t_air_s": t_air, "W_end": W,
                "W_loiter_end": W_loiter_end, "t_climb_s": cl["t"]}

    def _solve(self, f, x1: float, tol: float, lo: float = 0.0) -> tuple:
        """Root of the increasing function f (f(lo) <= 0) by bracketed secant (Illinois) steps; returns (x, f(x)) on
        the feasible side (f <= 0) within ``tol``."""
        f_lo = f(lo)
        hi, f_hi = x1, f(x1)
        while f_hi <= 0.0:                                    # expand the bracket
            lo, f_lo = hi, f_hi
            hi = 2.0 * hi + 600.0
            f_hi = f(hi)
        side = 0
        for _ in range(60):
            if hi - lo < tol:
                break
            x = hi - f_hi * (hi - lo) / (f_hi - f_lo)
            if not lo < x < hi:
                x = 0.5 * (lo + hi)
            fx = f(x)
            if fx <= 0.0:
                lo, f_lo = x, fx
                if side == -1:
                    f_hi *= 0.5
                side = -1
            else:
                hi, f_hi = x, fx
                if side == 1:
                    f_lo *= 0.5
                side = 1
        return lo, f_lo

    def solve_loiter_for_fuel(self, m0: float, fuel_kg: float, **kw) -> dict:
        base = self.fly(m0, 0.0, **kw)
        if base["ff"] * m0 > fuel_kg:
            return {"feasible": False, **base, "t_loiter_s": 0.0}
        h = kw.get("h") or self.h_loiter
        p = self.best_loiter(base["W_loiter_end"], h, kw.get("loiter_pol", "loiter"), rpm_floor=True)
        t1 = (fuel_kg - base["ff"] * m0) / (p["ff_kg_s"] * (1.0 + self.trapped) * (1.0 + self.reserve_frac))
        t, _ = self._solve(lambda t_: self.fly(m0, t_, **kw)["ff"] * m0 - fuel_kg, t1, 0.5)
        r = self.fly(m0, t, **kw)
        r["t_loiter_s"] = t
        r["feasible"] = True
        return r

    def solve_loiter_for_endurance(self, m0: float, endurance_s: float, **kw) -> dict:
        """Loiter time at which the mission air time equals ``endurance_s``. The return transit is flown at the
        post-loiter weight (slower best-range speed), so the air time is not the zero-loiter air time + t: solved."""
        base = self.fly(m0, 0.0, **kw)
        t1 = max(endurance_s - base["t_air_s"], 1.0)
        t, _ = self._solve(lambda t_: self.fly(m0, t_, **kw)["t_air_s"] - endurance_s, t1, 0.5)
        r = self.fly(m0, t, **kw)
        r["t_loiter_s"] = t
        return r

    def solve_range(self, m0: float, fuel_kg: float, h: float | None = None) -> dict:
        base = self.fly(m0, 0.0, R_transit=0.0, h=h, ferry=True)
        hh = self.h_loiter if h is None else h
        p = self.best_range(base["W_loiter_end"], hh)
        x1 = (fuel_kg - base["ff"] * m0) / (p["ff_kg_s"] / p["V"] * (1.0 + self.trapped) * (1.0 + self.reserve_frac))
        R, _ = self._solve(lambda R_: self.fly(m0, 0.0, R_transit=R_, h=h, ferry=True)["ff"] * m0 - fuel_kg, x1, 5.0)
        return {"range_m": R, **self.fly(m0, 0.0, R_transit=R, h=h, ferry=True)}

    # ------------------------------------------------------------------ field performance
    def k_ground(self, pol: str) -> float:
        h_w = float(self.S["wing"]["planform"]["z_root"]) - self.ground["z_g"]
        r = (16 * h_w / float(self.S["wing"]["span"])) ** 2
        return self.pol[pol].fit["k"] * r / (1 + r)

    def takeoff(self, m: float, h: float = 0.0, cg: tuple | None = None, I_yy: float | None = None,
                t_spin: float | None = None, rot_cap_factor: float = 1.0) -> dict:
        """Take-off at mass ``m`` and CG ``cg`` = (x, z) with the gear extended (default: ground design case).

        Geometry (V3-03, fix round 4): every moment is taken about the main-wheel ground contact C = (x_mg, z_g) in
        the earth frame at the actual body attitude theta (nose-up rotation about C moves the points above C aft):
        horizontal arm a = (x - x_mg) cos theta + (z - z_g) sin theta, height h = -(x - x_mg) sin theta + (z - z_g)
        cos theta. Forces: weight W at the CG; wing-body lift L = q S (CL0 + CLa theta) at the wing-body AC (vertical,
        level ground run); stabilator download F_t at the stabilator AC; thrust T along the body thrust line inclined
        eps below the body axis (earth components: forward T cos(theta - eps), up T sin(theta - eps)) at the hub;
        Cm0 of the take-off configuration; drag + longitudinal inertia (= forward thrust - friction) at the CG height;
        wheel reactions (main, nose) and rolling friction mu N at the ground (no moment about C).
        1. Ground run at the ground attitude theta_g with the take-off flap, time-integrated (dt 0.02 s): WOT thrust
           T(V) with the installation ramp, drag CD0_TO + k_ground CL^2 + full-power cooling drag, friction mu N.
        2. FCS ground-run schedule: the smallest stabilator download that keeps the main wheels loaded (no
           wheelbarrowing).
        3. Rotation law (V3-03; pitch inertia V4-05, fix round 5): the FCS commands a pitch rate that is ramped from
           zero to the commanded rate (the 1.1 VS_TO trimmed attitude, the R-16 attitude, in ``t_rot``; Raymer 6th ed.
           17.8.2: about 1 s) in the spin-up time ``t_spin`` (constant pitch acceleration thdd_s = rate / t_spin), then
           held. With the nose wheel off the moment about C must equal (I_yy + m a_cg^2) thdd (pitch inertia about the
           CG ``I_yy`` from the loading case, mass_cases; the m a_cg^2 part is the vertical acceleration of the CG
           ahead of C, which also enters the wheel reaction N = W - T sin(theta - eps) + F_t - L + m a_z, a_z = -thdd
           a_cg; the horizontal part is in the longitudinal integration); the stabilator download is the value that
           gives it. The rotation starts at V_R = the lowest speed at which the maximum download eta q S_h CLt_max
           gives the spin-up moment at the ground attitude (nose wheel unloaded, main wheels loaded). The download
           then falls as the lift grows and becomes the airborne download (moment about the CG = I_yy thdd at the same
           pitch acceleration and vertical CG acceleration; the power-on trim once the rate is held) exactly when N
           reaches zero: lift-off. The download NEEDED is checked against the maximum at every step and the margin is
           recorded unclipped (V4-01, fix round 5: negative when the stabilator cannot give it); on such a step the
           maximum is applied and the aircraft follows the pitch acceleration that the maximum gives (the step is
           counted). Below 1.1 VS_TO the FCS limits the attitude to the value at which the main wheels stay loaded (no
           lift-off below 1.1 VS_TO). ``I_yy`` None or ``t_spin`` 0: the v1.5 quasi-static law (no pitch
           acceleration, constant rate from V_R). ``rot_cap_factor`` (tests only; 1.0 in the analysis) scales the
           maximum download in the rotation phase to emulate a stabilator that cannot hold it.
        4. Checks: main wheels loaded at V_R (R-35); rotation download needed within the maximum (R-60, unclipped);
           moment balance about C at every rotation step (residual) and the airborne balance at lift-off (download
           continuity); V_LOF >= 1.1 VS_TO.
        5. Airborne distance to 15 m (Raymer 17.8.3)."""
        atm = AL.isa(h)
        rho = atm["rho"]
        W = m * G
        P = self.pol["gear_down"]
        gr = self.ground
        x_cg, z_cg = cg if cg is not None else (gr["x_cg_to"], gr["z_cg_to"])
        I_cg = float(I_yy) if I_yy is not None else 0.0
        t_sp = float(gr.get("t_spin", 0.0) if t_spin is None else t_spin)
        dynamic = I_cg > 0.0 and t_sp > 0.0
        fl = P.clmax["flap_to"]
        VS = AL.stall_speed(W, self.Sw, P.clmax["to_trimmed"], rho)
        CLg = gr["CL_ground_to"]
        cd0 = P.fit["cd0"] + fl["dCD0"]
        kg = self.k_ground("gear_down")
        S, c = self.Sw, float(self.S["wing"]["mac"])
        e = self.eps
        st = gr["stab"]
        Sh, xt, zt_ac, eta, clt = st["S_h"], st["x_ac"], st.get("z_ac", gr["z_t"]), st["eta"], st["clt_max"]
        Cm0 = gr["Cm0_to"]
        x_ac, z_ac, x_mg, z_g = gr["x_ac_wb"], gr.get("z_ac_wb", 0.0), gr["x_mg"], gr["z_g"]
        x_thr, z_thr = gr["x_thrust"], gr["z_t"]
        x_ng = gr.get("x_ng")
        th_s = math.radians(gr.get("static_attitude_deg", 0.0))
        z_ng = z_g - (x_mg - x_ng) * math.tan(th_s) if x_ng is not None else None   # nose-wheel contact (body frame)
        CL0, CLa = gr["CL0"] + gr["dCL0_to"], gr["CLa"]
        th_g = (CLg - CL0) / CLa                                  # ground attitude (rad; body = wing-body datum)
        mu = gr["mu"]

        def thrust(V):
            return self.prop.wot(max(V, 0.0), h)

        def drag(V, q, CL, w):
            return q * S * (cd0 + kg * CL ** 2) + q * self.cooling_extra_Dq(max(V, 1.0), h, self.wot_ff(w))

        def arm(x, z, th):
            return (x - x_mg) * math.cos(th) + (z - z_g) * math.sin(th)

        def hgt(x, z, th):
            return -(x - x_mg) * math.sin(th) + (z - z_g) * math.cos(th)

        def I_eff(th):
            """Pitch inertia in the moment equation about C (kg m^2): I_yy + m a_cg^2 (see the docstring)."""
            return I_cg + m * arm(x_cg, z_cg, th) ** 2

        def state(V, T, th, Ft, thdd=0.0):
            """Total wheel reaction N (nose + main; with the vertical CG acceleration of a pitch acceleration thdd about
            C) and the moment about C without the nose-wheel reaction (nose-up +)."""
            q = 0.5 * rho * V * V
            L = q * S * (CL0 + CLa * th)
            Tf, Tv = T * math.cos(th - e), T * math.sin(th - e)
            N = W - Tv + Ft - L - m * thdd * arm(x_cg, z_cg, th)
            Nf = max(N, 0.0)
            M = (W * arm(x_cg, z_cg, th) - L * arm(x_ac, z_ac, th) + Ft * arm(xt, zt_ac, th) + q * S * c * Cm0
                 - Tf * hgt(x_thr, z_thr, th) - Tv * arm(x_thr, z_thr, th) + hgt(x_cg, z_cg, th) * (Tf - mu * Nf))
            return N, M, L, q

        def rot_download(V, T, th, thdd=0.0):
            """Download that gives the moment I_eff thdd about C with the nose wheel off; the moment is linear in F_t on
            each side of N = 0 (rolling friction mu N only while the main wheels are loaded)."""
            Mreq = I_eff(th) * thdd
            N0, M0, _, _ = state(V, T, th, 0.0, thdd)
            hc, at = hgt(x_cg, z_cg, th), arm(xt, zt_ac, th)
            M0nf = M0 + mu * hc * max(N0, 0.0)                    # moment without the friction term
            Ft = (Mreq - M0nf + mu * hc * N0) / (at - mu * hc)    # main wheels loaded: N = N0 + F_t >= 0
            if N0 + Ft >= 0.0:
                return Ft
            return (Mreq - M0nf) / at                             # wheels unloaded (N = 0, no friction)

        def pitch_accel(V, T, th, Ft):
            """Pitch acceleration about C that a given download produces (nose wheel off): M(thdd) = I_eff thdd, fixed
            point (the friction term depends on thdd through N)."""
            thdd = 0.0
            for _ in range(20):
                thdd_new = state(V, T, th, Ft, thdd)[1] / I_eff(th)
                if abs(thdd_new - thdd) < 1e-12:
                    break
                thdd = thdd_new
            return thdd

        def trim_download(V, T, th, thdd=0.0):
            """Airborne download at attitude th: moment about the CG = I_yy thdd with L = W - T sin(th - eps) + F_t +
            m a_z (a_z = -thdd a_cg, the vertical CG acceleration of the rotation about C at lift-off; level flight
            path and the power-on trim for thdd = 0)."""
            q = 0.5 * rho * V * V
            Tf, Tv = T * math.cos(th - e), T * math.sin(th - e)
            a_ac = arm(x_ac, z_ac, th) - arm(x_cg, z_cg, th)
            a_t = arm(xt, zt_ac, th) - arm(x_cg, z_cg, th)
            h_t = hgt(x_thr, z_thr, th) - hgt(x_cg, z_cg, th)
            a_thr = arm(x_thr, z_thr, th) - arm(x_cg, z_cg, th)
            maz = -m * thdd * arm(x_cg, z_cg, th)
            return ((W - Tv + maz) * a_ac - q * S * c * Cm0 + Tf * h_t + Tv * a_thr + I_cg * thdd) / (a_t - a_ac)

        def trimmed_attitude(V, T):
            th = th_g
            q = 0.5 * rho * V * V
            for _ in range(30):
                Ft = trim_download(V, T, th)
                th_new = ((W - T * math.sin(th - e) + Ft) / (q * S) - CL0) / CLa
                if abs(th_new - th) < 1e-9:
                    break
                th = th_new
            return th, trim_download(V, T, th)

        def n_main(V, T, Ft):
            """Main-wheel reaction on the ground run (both wheels on the ground, attitude th_g)."""
            N, M, _, _ = state(V, T, th_g, Ft)
            if x_ng is None:
                return N
            n_nose = max(-M / max(-arm(x_ng, z_ng, th_g), 1e-6), 0.0)
            return N - n_nose

        def schedule(V, T):
            """FCS ground-run download: the smallest F_t (0 .. maximum) that keeps the main wheels loaded."""
            Ft_max = eta * 0.5 * rho * V * V * Sh * clt
            if x_ng is None or n_main(V, T, 0.0) >= 0.0:
                return 0.0, Ft_max
            if n_main(V, T, Ft_max) < 0.0:
                return Ft_max, Ft_max
            a, b = 0.0, Ft_max
            for _ in range(40):
                mid = 0.5 * (a + b)
                a, b = (a, mid) if n_main(V, T, mid) >= 0.0 else (mid, b)
            return b, Ft_max

        # 1.1 VS_TO trimmed attitude (R-16 attitude), the commanded rotation rate and the spin-up pitch acceleration
        V11 = 1.1 * VS
        th_11, _ = trimmed_attitude(V11, thrust(V11)["T"])
        th_rate = max(th_11 - th_g, math.radians(0.5)) / max(gr["t_rot"], 1e-3)          # rad/s
        thdd_s = th_rate / t_sp if dynamic else 0.0

        def vr_scan(thdd):
            """Lowest speed (0.02 m/s grid) at which the maximum download gives the moment I_eff thdd about C at the
            ground attitude (nose wheel off; the main-wheel reaction there is R-35)."""
            for V in np.arange(2.0, 50.0, 0.02):
                T = thrust(V)["T"]
                Ft = eta * 0.5 * rho * V * V * Sh * clt
                if state(V, T, th_g, Ft, thdd)[1] >= I_eff(th_g) * thdd:
                    return float(V)
            return None
        V_R = vr_scan(thdd_s)
        V_R_qs = vr_scan(0.0) if dynamic else V_R
        if V_R is None:
            return {"feasible": False, "ground_roll_m": float("inf"), "V_R_m_s": None, "VS_TO_m_s": VS}
        T_R = thrust(V_R)["T"]
        q_R = 0.5 * rho * V_R * V_R
        Ft_R = eta * q_R * Sh * clt
        N_main_VR = state(V_R, T_R, th_g, Ft_R, thdd_s)[0]
        # time integration of the ground run and the rotation
        dt, V, s, t, t_r = 0.02, 0.0, 0.0, 0.0, None
        V_dl0 = None
        lof = None
        th, thd = th_g, 0.0
        rot = {"download_margin_min_N": float("inf"), "moment_residual_max_Nm": 0.0, "rate_limited_steps": 0,
               "attitude_hold_steps": 0, "N_min_N": float("inf"), "steps": 0, "Ft_min_N": float("inf"),
               "Ft_max_N": -float("inf"), "thdd_max": 0.0, "nose_wheel_recontact_steps": 0}
        while True:
            w = thrust(V)
            T = w["T"]
            if t_r is None and V >= V_R:
                t_r = t
            if t_r is None:                                       # ground run at the ground attitude
                Ft, _ = schedule(V, T)
                if Ft > 0.0 and V_dl0 is None:
                    V_dl0 = V
                N, _, L, q = state(V, T, th_g, Ft)
                th = th_g
            else:                                                 # rotation on the main wheels
                Ft_cap = rot_cap_factor * eta * 0.5 * rho * V * V * Sh * clt
                if dynamic:                                       # spin-up to the commanded rate, then hold it
                    thdd = min(thdd_s, max(th_rate - thd, 0.0) / dt)
                    thd_new = thd + thdd * dt
                else:                                             # quasi-static law: commanded rate from V_R
                    thdd, thd_new = 0.0, (th_rate if t > t_r else 0.0)
                th_cmd = th + thd_new * dt
                Ft_req = rot_download(V, T, th_cmd, thdd)
                margin = Ft_cap - Ft_req                          # V4-01: unclipped (negative = cannot be held)
                Ft = Ft_req
                if Ft_req > Ft_cap:                               # the maximum is applied; the aircraft follows the
                    rot["rate_limited_steps"] += 1                # pitch acceleration it gives (not below th_g)
                    Ft = Ft_cap
                    if dynamic:
                        th_cmd = th
                        for _ in range(30):                       # pitch acceleration at the end-of-step attitude
                            thdd = pitch_accel(V, T, th_cmd, Ft)
                            thd_new = max(thd + thdd * dt, 0.0)
                            th_new_ = max(th + thd_new * dt, th_g)
                            if abs(th_new_ - th_cmd) < 1e-15:
                                break
                            th_cmd = th_new_
                        th_cmd = th_new_
                    else:                                         # quasi-static law: attitude held this step
                        thdd, thd_new, th_cmd = 0.0, 0.0, th
                        Ft = min(rot_download(V, T, th), Ft_cap)
                    if th_cmd <= th_g:                            # back on the nose wheel: it reacts the moment
                        thd_new, thdd = 0.0, 0.0
                        rot["nose_wheel_recontact_steps"] += 1
                N, M, L, q = state(V, T, th_cmd, Ft, thdd)
                if N <= 0.0 and V < V11:
                    # no lift-off below 1.1 VS_TO: the FCS limits the attitude to keep the main wheels loaded (rate
                    # stopped: no pitch acceleration at the held attitude)
                    lo_, hi_ = th_g, th_cmd
                    for _ in range(50):
                        mid = 0.5 * (lo_ + hi_)
                        lo_, hi_ = (mid, hi_) if state(V, T, mid, rot_download(V, T, mid))[0] > 0.0 else (lo_, mid)
                    th_cmd, thdd, thd_new = lo_ - math.radians(0.02), 0.0, 0.0
                    Ft = rot_download(V, T, th_cmd)
                    margin = min(margin, Ft_cap - Ft)
                    N, M, L, q = state(V, T, th_cmd, Ft)
                    rot["attitude_hold_steps"] += 1
                if N <= 0.0:                                      # lift-off inside this step: exact attitude N = 0
                    lo_, hi_ = th, th_cmd
                    if state(V, T, lo_, rot_download(V, T, lo_, thdd), thdd)[0] <= 0.0:
                        lo_ = th_g
                    for _ in range(60):
                        mid = 0.5 * (lo_ + hi_)
                        lo_, hi_ = (mid, hi_) if state(V, T, mid, rot_download(V, T, mid, thdd), thdd)[0] > 0.0 \
                            else (lo_, mid)
                    th_l = hi_
                    Ft_l = rot_download(V, T, th_l, thdd)
                    N_l, M_l, L_l, q_l = state(V, T, th_l, Ft_l, thdd)
                    Ft_trim = trim_download(V, T, th_l, thdd)
                    rot["download_margin_min_N"] = min(rot["download_margin_min_N"], Ft_cap - Ft_l)
                    lof = {"V": V, "theta": th_l, "Ft": Ft_l, "CL": CL0 + CLa * th_l, "T": T, "N": N_l, "M": M_l,
                           "Ft_trim": Ft_trim, "L": L_l, "thdd": thdd, "thd": thd_new, "I_eff": I_eff(th_l),
                           "residual": abs(M_l - I_eff(th_l) * thdd)}
                    break
                th, thd = th_cmd, thd_new
                rot["download_margin_min_N"] = min(rot["download_margin_min_N"], margin)
                if th_cmd > th_g or M > 0.0:                      # nose wheel off: moment balance about C
                    rot["moment_residual_max_Nm"] = max(rot["moment_residual_max_Nm"], abs(M - I_eff(th_cmd) * thdd))
                rot["N_min_N"] = min(rot["N_min_N"], N)
                rot["Ft_min_N"] = min(rot["Ft_min_N"], Ft)
                rot["Ft_max_N"] = max(rot["Ft_max_N"], Ft)
                rot["thdd_max"] = max(rot["thdd_max"], thdd)
                rot["steps"] += 1
            CL_now = L / max(q * S, 1e-9)
            D = drag(V, q, CL_now, w)
            a = (T * math.cos(th - e) - D - mu * max(N, 0.0)) / m
            if a <= 0.0 or t > 120.0:
                return {"feasible": False, "ground_roll_m": float("inf"), "V_R_m_s": V_R, "VS_TO_m_s": VS}
            V += a * dt
            s += V * dt
            t += dt
        V_lof, th_lof, q = lof["V"], lof["theta"], 0.5 * rho * lof["V"] ** 2
        clt_lof = lof["Ft"] / (eta * q * Sh)
        CL_wb = (W - lof["T"] * math.sin(th_lof - e) + lof["Ft"] - m * lof["thdd"] * arm(x_cg, z_cg, th_lof)) / (q * S)
        # airborne distance to 15 m
        V_tr = max(1.15 * VS, V_lof)
        R = V_tr ** 2 / (0.2 * G)
        ex = self.wot_excess(W, V_tr, h, "gear_down")
        gam = math.asin(max(min((ex["excess"] - q_s(V_tr, h) * S * fl["dCD0"]) / W, 0.5), 0.01))
        h_tr = R * (1 - math.cos(gam))
        s_air = math.sqrt(R ** 2 - (R - 15.0) ** 2) if h_tr >= 15.0 else R * math.sin(gam) + (15.0 - h_tr) / math.tan(gam)
        # lift-off speed at the ground attitude without any stabilator download (information: below it the FCS
        # download keeps the main wheels on the ground)
        V_flat = math.sqrt(2 * W / (rho * S * CLg))
        D_ = math.degrees
        rot["moment_residual_max_Nm"] = max(rot["moment_residual_max_Nm"], lof["residual"])
        if rot["steps"] == 0:
            rot.update(N_min_N=0.0, Ft_min_N=lof["Ft"], Ft_max_N=lof["Ft"])
        return {"feasible": True, "ground_roll_m": s, "time_s": t, "V_lof_m_s": V_lof, "V_R_m_s": V_R,
                "VS_TO_m_s": VS, "V_flat_ground_attitude_m_s": V_flat, "main_gear_load_at_VR_N": N_main_VR,
                "wheelbarrow_free": bool(N_main_VR > 0.0), "stab_local_cl_at_lof": clt_lof,
                "theta_lof_deg": D_(th_lof), "theta_ground_deg": D_(th_g), "theta_1p1VS_trimmed_deg": D_(th_11),
                "CL_wb_lof": CL_wb, "CL_available_lof": lof["CL"], "T_static_N": self.prop.wot(0.0, h)["T"],
                "T_lof_N": lof["T"], "air_distance_15m_m": s_air, "distance_15m_m": s + s_air,
                "climb_gradient": math.sin(gam), "CL_ground": CLg, "flap_deg": fl["delta_deg"], "x_cg": x_cg,
                "z_cg": z_cg, "mass_kg": m, "rotation_time_s": t - t_r, "rotation_rate_deg_s": D_(th_rate),
                "rotation": {"download_at_V_R_N": Ft_R, "download_max_at_V_R_N": Ft_R,
                             "download_at_lof_N": lof["Ft"], "trim_download_at_lof_N": lof["Ft_trim"],
                             "download_continuity_at_lof_N": lof["Ft"] - lof["Ft_trim"],
                             "main_wheel_reaction_at_lof_N": lof["N"], "moment_about_C_at_lof_Nm": lof["M"],
                             "download_margin_min_N": rot["download_margin_min_N"],
                             "moment_residual_max_Nm": rot["moment_residual_max_Nm"],
                             "main_wheel_reaction_min_N": rot["N_min_N"], "download_min_N": rot["Ft_min_N"],
                             "download_max_N": rot["Ft_max_N"], "rate_limited_steps": rot["rate_limited_steps"],
                             "attitude_hold_steps": rot["attitude_hold_steps"], "steps": rot["steps"],
                             "nose_wheel_recontact_steps": rot["nose_wheel_recontact_steps"],
                             "pitch_inertia_I_yy_kg_m2": I_cg, "pitch_inertia_about_C_eff_at_V_R_kg_m2": I_eff(th_g),
                             "spin_up_time_s": t_sp if dynamic else 0.0, "spin_up_pitch_accel_deg_s2": D_(thdd_s),
                             "spin_up_moment_at_V_R_Nm": I_eff(th_g) * thdd_s,
                             "pitch_rate_at_lof_deg_s": D_(lof["thd"]), "pitch_accel_at_lof_deg_s2": D_(lof["thdd"]),
                             "V_R_without_pitch_inertia_m_s": V_R_qs},
                "fcs_stabilator_schedule": {
                    "download_start_m_s": V_dl0, "full_nose_up_at_V_R_m_s": V_R,
                    "full_nose_up_local_cl": clt, "trim_download_local_cl_at_V_R":
                        trim_download(V_R, T_R, th_g) / (eta * q_R * Sh),
                    "rotation_download_local_cl_at_lof": clt_lof, "liftoff_not_below_m_s": V11,
                    "rule": "ground run: smallest stabilator download that keeps the main wheels loaded (from "
                            "download_start), full nose-up at V_R; rotation: pitch-rate command ramped from zero to the "
                            "commanded rate in the spin-up time, then held, with the download that gives the moment "
                            "(I_yy + m a_cg^2) x pitch acceleration about the main-wheel contact (nose wheel off), "
                            "falling from full nose-up at V_R to the airborne value at lift-off, where the main-wheel "
                            "reaction reaches zero; attitude limited below 1.1 VS_TO (no lift-off below it)"}}

    def takeoff_cases(self, m: float, h: float, cases: list, **kw) -> dict:
        """Take-off for every loading case of mass ``m`` (gear extended; pitch inertia I_yy of each case when the case
        carries it); the governing case is the longest roll."""
        res = [dict(self.takeoff(m, h, (c["x"], c["z"]), I_yy=c.get("I_yy"), **kw), case=c["name"]) for c in cases]
        gov = max(res, key=lambda r: r["ground_roll_m"])
        return dict(gov, cases={r["case"]: {k: r.get(k) for k in ("ground_roll_m", "V_R_m_s", "V_lof_m_s",
                                                                 "main_gear_load_at_VR_N", "stab_local_cl_at_lof",
                                                                 "theta_lof_deg", "x_cg", "rotation")} for r in res})

    def landing(self, m: float, h: float = 0.0, flap: str = "ld") -> dict:
        """Landing ground roll (Raymer energy method incl. 1 s free roll) at mass m with the landing flap (``flap`` =
        "ld", normal landing) or the take-off flap ("to": V1-10 abort/return landing at MTOM)."""
        atm = AL.isa(h)
        W = m * G
        P = self.pol["gear_down"]
        clmax = P.clmax["to_trimmed" if flap == "to" else "ld_trimmed"]
        fl = P.clmax["flap_to" if flap == "to" else "flap_ld"]
        r = AL.landing_roll(W, self.Sw, clmax, P.fit["cd0"] + fl["dCD0"], self.k_ground("gear_down"),
                            mu_brake=float(self.S["mission"]["braking_friction"]), rho=atm["rho"],
                            v_td_factor=float(self.S["mission"]["touchdown_speed_factor"]),
                            cl_roll=self.ground["CL_ground_to" if flap == "to" else "CL_ground_ld"], t_free=1.0)
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


def power_on_trim(S: dict, wa: dict, stab: dict, tail: dict, clm: dict, prop: "Prop", cases: list,
                  liftoff_cases: list | None = None) -> dict:
    """Local stabilator CL needed to trim with full throttle (thrust-line moment) at the forward CGs:
    * lift-off: MTOM loading cases WITH THE GEAR EXTENDED (``liftoff_cases`` = ground_cases; the extended nose leg
      moves the CG forward), take-off flap, V = 1.1 VS_TO (the lowest lift-off speed, most critical);
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
    lo_cases = liftoff_cases or cases
    m_top = max([c["m"] for c in lo_cases] + [0.0])
    for c in lo_cases:
        if abs(c["m"] - m_top) < 0.5 or abs(c["m"] - m0) < 0.5:
            V = 1.1 * AL.stall_speed(c["m"] * G, S_ref, clm["to_trimmed"])
            out["liftoff"].append(dict(trim(c["m"], c["x"], c["z"], V, stab["Cm0_wb"] - 0.25 * clm["dCL0_to"]),
                                       case=c["name"], gear="extended" if liftoff_cases else "as loaded"))
    for c in cases:
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
    r_g = float(g["diameter"]) / 2
    # layout phase: the door clearance holds over the whole ball footprint on the curved belly (the closed doors follow
    # the skin), not only on the keel line
    ys = np.linspace(0.0, 0.999 * r_g, 41)
    z_ret = max(float(af.z_bot(xc, y)) + float(T["bay"]["door_thickness"]) + float(T["bay"]["door_clearance"]) +
                math.sqrt(r_g ** 2 - y ** 2) for y in ys)
    return {"x": xc, "z_keel": zk, "ball_center_retracted_z": z_ret,
            "ball_center_extended_z": z_ret - float(T["stroke"]),
            "envelope_top_z": z_ret + float(g["height"]) - float(g["diameter"]) / 2,
            "mechanism_top_z": z_ret + float(g["height"]) - float(g["diameter"]) / 2 + float(T["bay"]["mechanism_height"])}


def _box_overlap(a, b, ya_sym: bool = True, yb_sym: bool = True) -> float:
    """Overlap volume (m3) of two zone boxes. A symmetric box spans -y1..y1 when y0 <= 0, and is the pair of boxes
    y0..y1 / -y1..-y0 (starboard + port) when y0 > 0."""
    a, b = np.asarray(a, float), np.asarray(b, float)

    def yr(box, sym):
        if not sym:
            return [(box[0, 1], box[1, 1])]
        return [(box[0, 1], box[1, 1]), (-box[1, 1], -box[0, 1])] if box[0, 1] > 0 else [(-box[1, 1], box[1, 1])]
    dx = min(a[1, 0], b[1, 0]) - max(a[0, 0], b[0, 0])
    dz = min(a[1, 2], b[1, 2]) - max(a[0, 2], b[0, 2])
    dy = sum(max(min(p[1], q[1]) - max(p[0], q[0]), 0.0) for p in yr(a, ya_sym) for q in yr(b, yb_sym))
    return max(dx, 0.0) * dy * max(dz, 0.0)


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
    # lateral protection on the ground (F10): roll about the main-wheel contact until the wing tip touches the ground
    # (wing-down landing, ground loop); the propeller tip circle must stay clear of the ground
    tip = S["wing"]["sections"][-1]
    xw, yuw, ylw = oml.resampled(tip["airfoil"], 81, 0.0, float(tip.get("thickness_scale", 1.0)))
    z_tip_low = float(tip["z_le"]) + float(tip["chord"]) * float(np.min(ylw))
    y_tip = float(tip["y"])
    phi = math.atan2(z_tip_low - z_g, y_tip - y_mg)
    th_ = np.linspace(0.0, 2 * math.pi, 361)
    dy = R * np.sin(th_) - y_mg
    dz = Z_T + R * np.cos(th_) - z_g
    clr_roll = float(np.min(dz * math.cos(phi) - dy * math.sin(phi)))
    return {"z_g": z_g, "x_mg": x_mg, "y_mg": y_mg, "x_ng": x_ng, "track": 2 * y_mg, "wheelbase": wb, "h_cg": h_cg,
            "wingtip_ground_roll_deg": math.degrees(phi), "prop_clear_wingtip_on_ground_m": clr_roll,
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


def structure_triangles(af: Airframe, include_wing: bool = True) -> np.ndarray:
    """Triangles of the tail surfaces (both sides) and the wing incl. the LERX (both sides): obstacles for ray tests
    (F13). The retracted landing gear lies inside the OML (R-21, R-22) and is covered by the body test."""
    tris = []
    for k, m in af.tail_meshes.items():
        tris.append(m.V[m.F])
        if af.tail_mirror[k]:
            mm = m.mirrored_y()
            tris.append(mm.V[mm.F])
    if include_wing:
        wm = af.wing.mesh(refine=1)
        wmm = wm.mirrored_y()
        tris += [wm.V[wm.F], wmm.V[wmm.F]]
    return np.vstack(tris)


def turret_checks(S: dict, af: Airframe, fov: bool = True) -> dict:
    """Retracted: HD59 ball above the closed doors, E180 growth envelope (sphere + cylinder) and the lift mechanism
    inside the body; extended: field of regard (highest unobstructed elevation per azimuth). Rays leave the sensor
    window on the ball surface: its centre and four points of the aperture rim (radius
    payload.turret.aperture_radius) are traced along the line of sight; a direction counts as clear only if every ray
    clears the body OML (the open bay cavity is not solid; its walls are), the wing incl. the LERX (both sides), the
    tail surfaces incl. the ventral fin and bumper, the propeller disc and the retracted gear (inside the OML). The bay
    doors fold inward along the bay walls (no obstruction below the skin)."""
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
    # bay cavity (open when the turret is extended): not solid; with the aperture ring only the ring opening is open
    # at the skin plane (the ring is a flush skin insert), the cavity above it is the bay
    hw = r_g + 0.01
    cav = (xc - hw, xc + hw, hw, zb_skin - 0.01, z_ret + float(g["height"]) - r_g + float(P["bay"]["mechanism_height"]))
    ring = P["bay"].get("aperture_ring")
    r_ring = r_b + float(ring["radial_clearance"]) if ring else None
    t_skin = 0.012                                                 # skin + ring thickness band at the skin plane

    def blocked_body(Q):
        inc = (Q[:, 0] > cav[0]) & (Q[:, 0] < cav[1]) & (np.abs(Q[:, 1]) < cav[2]) & (Q[:, 2] > cav[3]) & \
            (Q[:, 2] < cav[4])
        if ring:                                               # ring band on the real (V) skin: open only inside
            band = np.abs(Q[:, 2] - np.asarray(af.z_bot(Q[:, 0], Q[:, 1]), float)) < t_skin     # the ring radius
            inc = inc & ~(band & (np.hypot(Q[:, 0] - xc, Q[:, 1]) > r_ring))
        return af.inside(Q) & ~inc
    tri = structure_triangles(af) if fov else np.zeros((0, 3, 3))
    tri_zmin, tri_zmax = (tri[:, :, 2].min(axis=1), tri[:, :, 2].max(axis=1)) if len(tri) else (None, None)
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
        if len(tri):
            # a ray going down (up) can only hit triangles reaching below (above) its origin height
            sel = (tri_zmin < O[:, 2].max()) if d[2] < 0 else (tri_zmax > O[:, 2].min())
            if np.any(sel) and np.any(ray_tri_hit(O, np.repeat(d[None], len(O), 0), tri[sel], 4.0)):
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
    # dense samples of the exact ruled lofts (80 span stations x 2 x 40 chord points per surface; the mesh vertices
    # alone can miss a trailing edge that crosses the disc between two mesh rings)
    pts = [af.body_mesh.V]
    for k, srf in af.tail.items():
        P = dense_surface_points(srf)
        pts.append(P)
        if af.tail_mirror[k]:
            pts.append(P * np.array([1.0, -1.0, 1.0]))
    names = ["body"] + [k for k in af.tail for _ in range(2 if af.tail_mirror[k] else 1)]
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
    gm = prop_guard_map(S, af)
    return {"radial_min_m": rad_min, "longitudinal_min_m": lon_min, "by_part": rows,
            "spinner_to_cowl_gap_m": spinner_gap, "fin_te_crossing_radius_m": float(fp["guard_radius"]),
            "fin_te_crossing_span_fraction": float(fp["te_crossing_span_fraction"]),
            "fin_guard_margin_over_tip_m": float(min(g_["margin_over_tip_m"] for g_ in gm["guards"]
                                                     if g_["part"] == "fin")),
            "ventral_guard_margin_over_tip_m": gm["ventral_margin_over_tip_m"],
            "plane_behind_cowl_te_m": H[0] - x_end, "plane_behind_cowl_te_over_D": (H[0] - x_end) / (2 * R),
            "guard_map": gm,
            "blade_tip_axial_half_extent_m": float(blade_axial_half_extent(S, np.array([R]))[0])}


def dense_surface_points(srf: oml.LiftingSurface, n_span: int = 80, n_chord: int = 40) -> np.ndarray:
    """Points on the exact ruled loft of a lifting surface (section loops at ``n_span`` span stations)."""
    e = srf.span_coords()
    return np.vstack([srf.loop_at(float(x), n_chord) for x in np.linspace(e[0], e[-1], n_span)])


def prop_guard_map(S: dict, af: Airframe, sector_deg: float = 15.0) -> dict:
    """What protects the open pusher propeller (F10), in the (thrust-line inclined) disc plane:
    * guards: where the trailing edges of the canted fins, the ventral fin and the stabilators cross the disc plane
      (polar angle from the top, positive to starboard; radius and margin over the tip circle);
    * sectors: per ``sector_deg`` sector of the disc, the nearest structure point (body, wing, tails) outside the tip
      circle inside the axial band of the blades +/- 0.05 m, as margin over the tip radius (None = nothing within
      0.30 m: open sector).
    Personnel protection is not claimed anywhere: the disc is open; the guards keep the ground, the runway and
    objects approaching in their planes away from the blade tips."""
    pr = S["propeller"]
    H = np.array(pr["hub"], float)
    e = math.radians(float(pr.get("thrust_line_inclination_deg", 0.0)))
    n = np.array([math.cos(e), 0.0, math.sin(e)])
    R = 0.5 * float(pr["diameter"])
    up = np.array([-math.sin(e), 0.0, math.cos(e)])                  # in-plane "top" direction
    side = np.array([0.0, 1.0, 0.0])

    def polar(P):
        v = P - H
        v = v - np.outer(v @ n, n) if v.ndim == 2 else v - (v @ n) * n
        return np.degrees(np.arctan2(v @ side, v @ up)), np.linalg.norm(v, axis=-1)

    guards = []
    for k, v in S["tail"]["surfaces"].items():
        if k == "stabilator_stub":
            continue
        secs = v["sections"]
        te = np.array([[s_["x_le"] + s_["chord"], s_["y"], s_["z_le"]] for s_ in secs], float)
        d = (te - H) @ n
        for i in range(len(te) - 1):
            if d[i] * d[i + 1] < 0 or d[i + 1] == 0.0:
                t = d[i] / (d[i] - d[i + 1])
                P = te[i] + t * (te[i + 1] - te[i])
                for sgn in ((1.0, -1.0) if v.get("mirror", True) else (1.0,)):
                    Q = P * np.array([1.0, sgn, 1.0])
                    ang, r = polar(Q)
                    guards.append({"part": k, "angle_deg": float(ang), "radius_m": float(r),
                                   "margin_over_tip_m": float(r - R)})
    vent = [g_ for g_ in guards if g_["part"] == "ventral"]
    # sector map from the OML meshes
    pts = [af.body_mesh.V, af.wing_mesh.V, af.wing_mesh.mirrored_y().V]
    for k, m in af.tail_meshes.items():
        pts.append(m.V)
        if af.tail_mirror[k]:
            pts.append(m.mirrored_y().V)
    Pall = np.vstack(pts)
    a = (Pall - H) @ n
    band = float(blade_axial_half_extent(S, np.array([R]))[0]) + 0.05
    Pm = Pall[np.abs(a) <= band]
    ang, r = polar(Pm)
    sectors = []
    for a0 in np.arange(-180.0, 180.0, sector_deg):
        m = (ang >= a0) & (ang < a0 + sector_deg) & (r >= R) & (r <= R + 0.30)
        sectors.append({"from_deg": float(a0), "to_deg": float(a0 + sector_deg),
                        "nearest_margin_m": float(np.min(r[m]) - R) if np.any(m) else None})
    n_open = sum(1 for s_ in sectors if s_["nearest_margin_m"] is None)
    return {"guards": guards, "sectors": sectors, "open_sector_fraction": n_open / len(sectors),
            "ventral_margin_over_tip_m": float(min(g_["margin_over_tip_m"] for g_ in vent)) if vent else float("-inf")}


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
    # ventral fin root (downward surface): the body bottom surface at both root faces must lie below the root section
    vr = TS["ventral"]["sections"][0]
    xv, tv = airfoil_thickness(vr["airfoil"])
    xs_v = vr["x_le"] + xv * vr["chord"]
    z_face = np.maximum(af.z_bot(xs_v, 0.5 * tv * vr["chord"]), af.z_bot(xs_v, 0.0 * xs_v))
    ventral_gap = float(np.max(z_face - vr["z_le"]))
    out = {"stab_root_gap_m": stub_gap, "stab_root_body_clearance_m": gap_body,
           "stab_root_deflection_range_deg": list(rng_), "fin_root_max_gap_m": fin_gap,
           "fin_root_embed_min_m": -fin_gap, "ventral_root_max_gap_m": ventral_gap,
           "tail_root_max_gap_m": max(fin_gap, ventral_gap)}
    if tp is not None:
        out["fin_exposed_area_each_m2"] = tp["fin"]["area"]
        out["fin_panel_area_each_m2"] = tp["fin"].get("area_panel", tp["fin"]["area"])
    return out


def loft_volume_points(srf: oml.LiftingSurface, n_span: int = 160, n_chord: int = 61, n_thick: int = 7) -> np.ndarray:
    """Points filling the solid of a lifting-surface loft (ruled between sections): at ``n_span`` span stations,
    ``n_chord`` chord stations, ``n_thick`` points from the lower to the upper surface."""
    e = srf.span_coords()
    out = []
    f = np.linspace(0.0, 1.0, n_thick)[:, None, None]
    for eta in np.linspace(e[0], e[-1], n_span):
        L = srf.loop_at(float(eta), n_chord)
        up = L[:n_chord][::-1]                                   # LE -> TE
        lo = L[n_chord - 1:]                                     # LE -> TE
        out.append((up[None, :, :] + f * (lo - up)[None, :, :]).reshape(-1, 3))
    return np.vstack(out)


def tail_root_structure(S: dict, af: Airframe) -> dict:
    """V1-04: the tail lofts (fins, fixed stabilator stubs, ventral fin) are TRIMMED AT THE BODY OML. Their planar root
    sections lie below the skin only so that the loft closes on the body without a gap (R-40); the part of a loft deeper
    than ``tail.root_structure.fitting_band_depth`` below the skin is not structure (the CAD trims it at the OML). The
    structure inside the body is the root band: the loft solid between the skin and that depth (contoured root rib,
    spar root fittings on the body frames). Returns, per surface (starboard): the band points, the deepest loft point
    below the skin (planar-root extension, information) and the band depth used."""
    RS = S["tail"]["root_structure"]
    d_band = float(RS["fitting_band_depth"])
    d_aft = float(RS.get("band_depth_aft_of_firewall", d_band))
    x_fw = float(S["layout"].get("firewall_x", 1e9))
    out = {}
    for k in ("fin", "stabilator_stub", "ventral"):
        if k not in af.tail:
            continue
        P = loft_volume_points(af.tail[k])
        ins = af.inside(P)
        deep = af.inside(P, margin=np.where(P[:, 0] < x_fw, d_band, d_aft))
        band = P[ins & ~deep]
        # depth of the planar-root extension below the skin: deepest inside point, by the inset that excludes it
        depth = 0.0
        if np.any(ins):
            Q = P[ins]
            lo, hi = 0.0, 0.40
            for _ in range(30):
                mid = 0.5 * (lo + hi)
                lo, hi = (mid, hi) if np.any(af.inside(Q, margin=mid)) else (lo, mid)
            depth = lo
        out[k] = {"band_points": band, "extension_depth_m": depth, "band_depth_m": d_band,
                  "band_depth_aft_of_firewall_m": d_aft}
    return out


def tail_root_interference(S: dict, af: Airframe, trs: dict | None = None) -> dict:
    """V1-04 interference check of the tail-root structure (trimmed lofts: root bands of tail_root_structure) and the
    stabilator spindles against the internal layout zones (each zone box grown by its own clearance) and against each
    other (band-to-band and band-to-spindle distance >= tail.root_structure.min_clearance). Pairs that are joined by
    design are excluded: the stub and its own spindle (outboard bearing in the stub rib). Returns the conflicts
    (count = R-53 metric) and the minimum clearances."""
    from scipy.spatial import cKDTree
    trs = trs or tail_root_structure(S, af)
    RS = S["tail"]["root_structure"]
    c_min = float(RS["min_clearance"])
    Z_ = S["layout"]["zones_preliminary"]
    joined = {("stabilator_stub", "stabilator_spindle")}

    def in_box(P, box, cl, sym=True):
        b = np.asarray(box, float)
        y0 = -b[1, 1] if (sym and b[0, 1] <= 0) else b[0, 1]
        m = ((P[:, 0] > b[0, 0] - cl) & (P[:, 0] < b[1, 0] + cl) & (P[:, 1] > y0 - cl) & (P[:, 1] < b[1, 1] + cl) &
             (P[:, 2] > b[0, 2] - cl) & (P[:, 2] < b[1, 2] + cl))
        return int(np.sum(m))
    conflicts, rows = [], {}
    for k, v in trs.items():
        B = v["band_points"]
        rows[k] = {"band_points": int(len(B)), "extension_depth_m": v["extension_depth_m"]}
        for zn, z in Z_.items():
            if "box" not in z or (k, zn) in joined:
                continue
            n = in_box(B, z["box"], float(z.get("clearance", 0.0)), bool(z.get("symmetric", True)))
            if n:
                conflicts.append({"a": k, "b": zn, "points": n})
    # spindle cylinder (starboard): axis along y at the pivot, from the inboard bearing to the stub root
    st_ = S["tail"]["surfaces"]["stabilator"]
    pv = np.asarray(st_["pivot"], float)
    r_s = float(st_["params"]["spindle_housing_radius"])
    th = np.linspace(0, 2 * math.pi, 16, endpoint=False)
    ys = np.linspace(spindle_inboard_y(S), float(S["tail"]["surfaces"]["stabilator_stub"]["params"]["y_out"]), 25)
    spindle = np.array([[pv[0] + r * math.cos(t), y, pv[2] + r * math.sin(t)] for y in ys for t in th
                        for r in (0.0, 0.5 * r_s, r_s)])
    sets = {k: v["band_points"] for k, v in trs.items()}
    sets["stabilator_spindle"] = spindle
    names = list(sets)
    dmin = {}
    for i in range(len(names)):
        for j in range(i + 1, len(names)):
            a, b = names[i], names[j]
            if (a, b) in joined or (b, a) in joined or not len(sets[a]) or not len(sets[b]):
                continue
            d = float(cKDTree(sets[b]).query(sets[a], k=1)[0].min())
            dmin[f"{a}/{b}"] = d
            if d < c_min:
                conflicts.append({"a": a, "b": b, "min_distance_m": d})
    return {"conflicts": conflicts, "n_conflicts": len(conflicts), "surfaces": rows, "min_distance_m": dmin,
            "band_depth_m": float(RS["fitting_band_depth"]),
            "band_depth_aft_of_firewall_m": float(RS.get("band_depth_aft_of_firewall", RS["fitting_band_depth"])),
            "min_clearance_m": c_min}


def stab_panel_normal_force(S: dict) -> dict:
    """Normal-force maximum and centre-of-pressure travel of the stabilator panel for the full-deflection hinge-moment
    cases (V1-05), from the NeuralFoil polar of the panel section (free transition, n_crit 9) at the two RE_GRID
    Reynolds numbers bracketing the panel Re at VA (the larger value is used): cn = cl cos(a) + cd sin(a);
    CN_max = max cn of the section, used as the panel bound (no 3-D reduction credited; DATCOM CLmax/clmax about
    0.85-0.9 for this aspect ratio and sweep would lower it); 2-D centre of pressure x_cp = 0.25 - cm/cn, its largest
    travel aft of the quarter chord between 2 deg and the cn maximum (attached range; at the reachable panel incidence,
    |delta| 20 deg + aircraft alpha, the effective 2-D angle a x a_3D/a_2D stays below the section stall)."""
    st = S["tail"]["surfaces"]["stabilator"]
    hp = st["params"]
    sp = stab_panel(hp)
    VA = float(S["structures"]["VC_eas"])
    Re = AL.reynolds(VA, sp["mac"], 0.0)
    grid = AE.RE_GRID
    j = min(max(int(np.searchsorted(grid, Re)), 1), len(grid) - 1)
    best = None
    for R_ in (grid[j - 1], grid[j]):
        p = AE.raw_polar(hp["airfoil"], float(R_), 9.0)
        a = np.radians(np.asarray(p["alpha"], float))
        cl, cd, cm = (np.asarray(p[k], float) for k in ("cl", "cd", "cm"))
        cn = cl * np.cos(a) + cd * np.sin(a)
        i = int(np.argmax(cn))
        m = (np.degrees(a) >= 2.0) & (np.arange(len(a)) <= i)
        travel = float(np.max(0.25 - cm[m] / cn[m]) - 0.25) if np.any(m) else 0.0
        r = {"Re": float(R_), "CN_max": float(cn[i]), "alpha_CN_max_deg": float(np.degrees(a[i])),
             "cp_travel_aft_of_c4": max(travel, 0.0), "min_confidence": float(np.min(p["confidence"]))}
        if best is None or r["CN_max"] > best["CN_max"]:
            best = r
    best["Re_panel_VA"] = Re
    return best


def stab_hinge_moments(S: dict) -> dict:
    """Hinge moments of one all-moving stabilator panel about its spindle (actuated through the four-bar linkage
    controls.linkage, V2-06): normal force x (centre of pressure -
    spindle), local dynamic pressure eta_t q. Centre of pressure: anywhere in controls.ac_mac_fraction_range of the panel
    MAC (vortex-lattice AC of the panels with and without the root stubs, widened by ac_band_fwd / ac_band_aft;
    build_geometry) plus the 2-D centre-of-pressure travel aft of c/4 from the section polar (stab_panel_normal_force).
    V1-05: normal force at full deflection = the section CN_max (stab_panel_normal_force), NOT the trim-rule constant
    aero.stability_rules.stabilator_clmax (R-36 basis). Cases: VA at CN_max (CS-LUAS.423 full control movement at VA),
    VD at 1/3 of it, and the largest steady trim (continuous duty). Actuator check (CS-LUAS.395(a)(1), standards.yaml):
    actuator peak torque x linkage ratio >= 1.25 x max hinge moment; rated torque x linkage ratio >= 1.1 x max hinge
    moment (R-47) and >= continuous trim hinge moment; actuator travel / linkage ratio covers the deflection range;
    surface rate at rated torque reported."""
    st = S["tail"]["surfaces"]["stabilator"]
    hp, C = st["params"], st["controls"]
    sp = stab_panel(hp)
    mac, Sp = sp["mac"], sp["area_panel"]
    nf = stab_panel_normal_force(S)
    f_lo, f_hi = C["ac_mac_fraction_range"]
    f_hi_cp = f_hi + nf["cp_travel_aft_of_c4"]
    e_max = (f_hi_cp - hp["pivot_mac_fraction"]) * mac
    e_min = (f_lo - hp["pivot_mac_fraction"]) * mac
    eta = float(S["aero"]["stability_rules"]["eta_tail"])
    cn = nf["CN_max"]
    ST_ = S["structures"]
    VA, VD = float(ST_["VC_eas"]), float(ST_["VD_eas"])
    H_VA = eta * 0.5 * RHO0 * VA ** 2 * Sp * cn * e_max
    H_VD = eta * 0.5 * RHO0 * VD ** 2 * Sp * cn / 3.0 * e_max
    act = C["actuator"]
    H_design = max(H_VA, H_VD)
    trim_cl = float(C.get("trim_cl_local_max", 0.4))
    V_trim = float(ST_["VC_eas"])
    H_trim = eta * 0.5 * RHO0 * V_trim ** 2 * Sp * trim_cl * e_max
    rng_ = C["range_deg"]
    # V2-06 (fix round 3): four-bar linkage with exact kinematics instead of a constant 'bellcrank ratio'; the design
    # hinge moment is applied at EVERY deflection (conservative), so the margins are set by the smallest torque ratio
    link = dict(C["linkage"], arm_ratio=float(C["linkage_ratio"]))
    lk = linkage_check(link, act, rng_, lambda d: H_design)
    k_min = lk["ratio_min"]
    return {"panel_area_m2": Sp, "panel_mac_m": mac, "spindle_x": sp["x_pivot"], "ac_offset_max_m": e_max,
            "ac_offset_min_m": e_min, "CN_max_panel": cn, "CN_max_basis": nf,
            "trim_rule_clmax_not_used": float(S["aero"]["stability_rules"]["stabilator_clmax"]),
            "cp_mac_fraction_max": f_hi_cp,
            # the spindle sits at the forward end of the AC band: the AC is never ahead of it (neutral at that end,
            # stable everywhere else), so the panel never diverges about its spindle
            "statically_stable_surface": bool(e_min >= -1e-9),
            "H_VA_Nm": H_VA, "H_VD_Nm": H_VD, "H_design_Nm": H_design, "H_trim_continuous_Nm": H_trim,
            "actuator": act["model"], "linkage_ratio": float(C["linkage_ratio"]), "linkage": lk,
            "peak_capacity_Nm": float(act["torque_peak_Nm"]) * k_min, "rated_capacity_Nm": float(act["torque_rated_Nm"]) * k_min,
            "peak_margin": lk["peak_margin_min"],
            "rated_margin": float(act["torque_rated_Nm"]) * k_min / max(H_trim, 1e-9),
            "rated_margin_max_hinge_moment": lk["rated_margin_min"],
            "pivot_mac_fraction": float(hp["pivot_mac_fraction"]), "ac_mac_fraction_range": [f_lo, f_hi],
            "surface_travel_deg": lk["max_surface_deflection_from_neutral_deg"],
            "travel_ok": bool(lk["reachable"] and lk["travel_margin_deg"] >= 0.0),
            "servo_angle_at_range_deg": lk["servo_angle_at_range_deg"],
            "surface_rate_at_rated_torque_deg_s": lk["surface_rate_min_deg_s"],
            "surface_rate_neutral_deg_s": lk["surface_rate_neutral_deg_s"]}


class FourBar:
    """Crank-rocker four-bar control linkage (V2-06, fix round 3). Servo arm r_s about O = (0, 0), surface horn
    r_h = N r_s about H = (d, r_s - r_h); at the neutral position both arms are parallel and normal to the pushrod
    (servo angle 0, surface at its neutral deflection), pushrod length l = d (symmetric linkage). Exact kinematics: the
    surface angle phi(theta) solves |A(theta) - B(phi)| = l on the branch through (0, 0). The velocity ratio
    dphi/dtheta is not constant: about cos(theta) / (N cos(phi)), so the torque multiplication grows and the surface
    rate falls toward large deflections, and a symmetric N:1 linkage reaches only about phi = asin(1/N) (exactly in the
    long-pushrod limit; a finite pushrod l = d moves one side slightly past it, at the toggle, and the other side short
    of it: the asymmetry is about Y^2 / (d r_h cos phi), Y the lateral offset of the rod ends)."""

    def __init__(self, r_s: float, N: float, d: float):
        self.r_s, self.N, self.d = float(r_s), float(N), float(d)
        self.r_h = self.N * self.r_s
        self.l = self.d

    def _g(self, th, ph):
        ax, ay = self.r_s * math.sin(th), self.r_s * math.cos(th)
        bx, by = self.d + self.r_h * math.sin(ph), (self.r_s - self.r_h) + self.r_h * math.cos(ph)
        return (ax - bx) ** 2 + (ay - by) ** 2 - self.l ** 2

    def phi(self, th: float) -> float:
        """Surface angle (rad, from neutral) for the servo angle ``th`` (rad); nan if the linkage locks before."""
        x = self.r_s * math.sin(th) / self.r_h
        if abs(x) >= 1.0:
            return float("nan")
        ph = math.asin(x)
        for _ in range(30):                                   # Newton on the exact constraint from the long-rod guess
            g = self._g(th, ph)
            dg = (self._g(th, ph + 1e-7) - self._g(th, ph - 1e-7)) / 2e-7
            if dg == 0.0:
                break
            step = g / dg
            ph -= step
            if abs(step) < 1e-12:
                break
        return ph if abs(self._g(th, ph)) < 1e-12 else float("nan")

    def theta(self, ph: float) -> float:
        """Servo angle (rad) for the surface angle ``ph`` (rad from neutral): bisection on the monotonic branch."""
        lo, hi = -math.pi / 2 + 1e-6, math.pi / 2 - 1e-6
        f = lambda t: (self.phi(t) if math.isfinite(self.phi(t)) else math.copysign(9.0, t)) - ph   # noqa: E731
        if f(lo) > 0 or f(hi) < 0:
            return float("nan")
        for _ in range(80):
            m = 0.5 * (lo + hi)
            lo, hi = (m, hi) if f(m) < 0 else (lo, m)
        return 0.5 * (lo + hi)

    def ratio(self, th: float) -> float:
        """Torque multiplication dtheta/dphi at the servo angle ``th`` (surface torque / servo torque)."""
        h = 1e-5
        dph = (self.phi(th + h) - self.phi(th - h)) / (2 * h)
        return 1.0 / dph if dph > 0 else float("nan")


def linkage_check(link: dict, act: dict, range_deg, H_fn, n: int = 41) -> dict:
    """Four-bar actuation of one surface over its deflection range: servo angles at the range ends against the
    actuator travel, and at every deflection the rated / peak torque through the linkage against the hinge moment
    H_fn(delta_deg) (rated >= 1.1 H, the project rule R-47; peak >= 1.25 H, CS-LUAS.395(a)(1)); surface rate at the
    rated actuator speed."""
    fb = FourBar(link["servo_arm_m"], link["arm_ratio"], link["pushrod_base_m"])
    d_n = float(link.get("neutral_deg", 0.0))
    lo, hi = float(range_deg[0]), float(range_deg[1])
    dd = np.linspace(lo, hi, n)
    th = np.array([fb.theta(math.radians(d - d_n)) for d in dd])
    trav = float(act["travel_deg"])
    reach = bool(np.all(np.isfinite(th)))
    th_deg = np.degrees(th)
    rat = np.array([fb.ratio(t) if math.isfinite(t) else float("nan") for t in th])
    H = np.array([max(H_fn(d), 1e-9) for d in dd])
    rated = float(act["torque_rated_Nm"]) * rat / H
    peak = float(act["torque_peak_Nm"]) * rat / (1.25 * H)
    rate = float(act["speed_rated_deg_per_s"]) / rat
    i_r, i_p = int(np.nanargmin(rated)), int(np.nanargmin(peak))
    return {"type": "four-bar crank-rocker (exact kinematics)", "arm_ratio_neutral": fb.N, "servo_arm_m": fb.r_s,
            "horn_m": fb.r_h, "pushrod_m": fb.l, "neutral_deg": d_n, "range_deg": [lo, hi], "reachable": reach,
            "max_surface_deflection_from_neutral_deg": math.degrees(math.asin(1.0 / fb.N)) if fb.N > 1 else 90.0,
            "servo_angle_at_range_deg": [float(th_deg[0]), float(th_deg[-1])],
            "servo_angle_max_abs_deg": float(np.nanmax(np.abs(th_deg))) if reach else float("nan"),
            "actuator_travel_deg": trav,
            "travel_margin_deg": (trav - float(np.nanmax(np.abs(th_deg)))) if reach else -90.0,
            "ratio_min": float(np.nanmin(rat)), "ratio_max": float(np.nanmax(rat)),
            "H_max_Nm": float(H.max()), "rated_margin_min": float(np.nanmin(rated)) if reach else 0.0,
            "rated_margin_at_deg": float(dd[i_r]), "peak_margin_min": float(np.nanmin(peak)) if reach else 0.0,
            "peak_margin_at_deg": float(dd[i_p]),
            "surface_rate_min_deg_s": float(np.nanmin(rate)), "surface_rate_neutral_deg_s":
                float(act["speed_rated_deg_per_s"]) / fb.ratio(0.0)}


def control_hinge_moments(S: dict, VA: float, VD: float, VF: float) -> dict:
    """Hinge moments and actuation of the ailerons, flaps and rudders (V2-03, fix round 3), the stabilator through its
    four-bar linkage (V2-06). Plain sealed surfaces, strip integral H = eta q |Ch(delta)| integral(c_f^2 dy) (per side);
    |Ch| = Ch_delta |delta| + Ch_0 (aero.hinge_moment_rules: components.yaml hinge-moment screening, Ch_delta about
    -0.6/rad plus the Ch_alpha term; 0.30 at 20 deg), flaps at 30-40 deg the screening upper value 0.55. Cases:
    ailerons and rudders full deflection at VA (CS-LUAS.423/.441/.455), 1/3 deflection at VD; flaps at VF = max(1.4 VS,
    1.8 VSF) (CS-LUAS.345(b)) over their whole range; stabilator: the CN_max hinge moment of stab_hinge_moments at VA
    applied at EVERY deflection (conservative). The four-bar torque ratio is evaluated at every deflection."""
    HR = S["aero"]["hinge_moment_rules"]
    chd, ch0 = float(HR["ch_delta_per_rad"]), float(HR["ch_0"])
    flap_ch, flap_band = float(HR["flap_ch_high_deflection"]), HR["flap_high_deflection_band_deg"]
    W = S["wing"]
    T = trapezoid(W["planform"])
    b2 = float(W["span"]) / 2
    out = {}

    def ch_primary(d_deg):
        return chd * abs(math.radians(d_deg)) + ch0

    def ch_flap(d_deg):
        return flap_ch if abs(d_deg) >= float(flap_band[0]) else min(ch_primary(d_deg), flap_ch)

    for k in ("aileron", "flap"):
        c = W["controls"][k]
        yy = np.linspace(c["eta0"] * b2, c["eta1"] * b2, 80)
        cf = c["chord_fraction"] * T["c"](yy)
        I2 = float(np.trapz(cf * cf, yy))                     # m^3, per side
        if k == "aileron":
            qA, qD = 0.5 * RHO0 * VA ** 2, 0.5 * RHO0 * VD ** 2
            H_fn = lambda d, qA=qA, qD=qD, I2=I2: max(qA * ch_primary(d) * I2, qD * ch_primary(d / 3.0) * I2)  # noqa: E731
            cond = {"V_eas_m_s": VA, "VD_eas_m_s": VD, "basis": "full deflection at VA, 1/3 at VD"}
        else:
            qF = 0.5 * RHO0 * VF ** 2
            H_fn = lambda d, qF=qF, I2=I2: qF * ch_flap(d) * I2                                          # noqa: E731
            cond = {"V_eas_m_s": VF, "basis": "VF = max(1.4 VS, 1.8 VSF), CS-LUAS.345(b)"}
        lk = linkage_check(c["linkage"], c["actuator"], c["range_deg"], H_fn)
        out[k] = {"area_per_side_m2": float(np.trapz(cf, yy)), "chord_mean_m": float(np.trapz(cf * cf, yy) / np.trapz(cf, yy)),
                  "H_design_Nm": max(H_fn(c["range_deg"][0]), H_fn(c["range_deg"][1])), "actuator": c["actuator"]["model"],
                  **cond, **lk}
    fin = S["tail"]["surfaces"]["fin"]
    rc = fin["controls"]["rudder"]
    secs = fin["sections"]
    cr, ct = float(secs[0]["chord"]), float(secs[-1]["chord"])
    span = float(fin["span"])
    cfr = float(rc["chord_fraction"])
    I2r = span * cfr ** 2 * (cr * cr + cr * ct + ct * ct) / 3.0
    eta_v = float(S["aero"]["stability_rules"]["eta_v"])
    qA, qD = 0.5 * RHO0 * VA ** 2, 0.5 * RHO0 * VD ** 2
    H_r = lambda d: eta_v * max(qA * ch_primary(d) * I2r, qD * ch_primary(d / 3.0) * I2r)                    # noqa: E731
    lk = linkage_check(rc["linkage"], rc["actuator"], rc["range_deg"], H_r)
    out["rudder"] = {"area_per_side_m2": span * cfr * 0.5 * (cr + ct), "chord_mean_m": I2r / (span * cfr * 0.5 * (cr + ct)),
                     "H_design_Nm": max(H_r(rc["range_deg"][0]), H_r(rc["range_deg"][1])),
                     "actuator": rc["actuator"]["model"], "V_eas_m_s": VA, "VD_eas_m_s": VD,
                     "basis": "full deflection at VA (CS-LUAS.441), 1/3 at VD; whole fin panel span (conservative)",
                     **lk}
    sc = S["tail"]["surfaces"]["stabilator"]["controls"]
    hm = stab_hinge_moments(S)
    out["stabilator"] = {"H_design_Nm": hm["H_design_Nm"], "actuator": sc["actuator"]["model"],
                         "V_eas_m_s": VA, "VD_eas_m_s": VD,
                         "basis": "CN_max hinge moment at VA applied at every deflection (conservative; "
                                  "stab_hinge_moments)", **hm["linkage"]}
    prim = [out[k] for k in ("aileron", "flap", "rudder")]
    out["summary"] = {"rated_margin_min_wing_fin": min(r["rated_margin_min"] for r in prim),
                      "peak_margin_min_wing_fin": min(r["peak_margin_min"] for r in prim),
                      "travel_margin_min_deg": min(out[k]["travel_margin_deg"] for k in ("aileron", "flap", "rudder",
                                                                                        "stabilator")),
                      "all_reachable": all(out[k]["reachable"] for k in ("aileron", "flap", "rudder", "stabilator"))}
    return out


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
    """Continuous electrical load cases vs the SG750 output ``gen_W`` (evaluate(): the lowest output of the design-
    mission loiter, R-32): baseline (HD59), E180 growth turret; both with the research-payload power allowance; E180
    and HD59 peaks. Buffer battery (V2-01, fix round 3): usable energy = nominal x usable fraction; the generator-loss
    / engine-start reserve (reserve power x reserve time) is kept; the rest is the E180 peak-support share (R-52)."""
    E = S["engine"]["electrical_budget"]
    base = float(E["continuous_base_W"])
    rp = float(E["research_payload_allowance_W"])
    hd59, e180_avg, e180_pk = float(E["hd59_average_W"]), float(E["e180_average_W"]), float(E["e180_peak_W"])
    hd59_pk = float(E.get("hd59_peak_W", E["e180_peak_W"]))
    cont_hd59 = base + rp
    cont_e180 = base - hd59 + e180_avg + rp
    peak_e180 = base - hd59 + e180_pk + rp
    peak_hd59 = base - hd59 + hd59_pk + rp
    worst = max(cont_hd59, cont_e180)
    usable = float(E["battery_energy_nominal_Wh"]) * float(E["battery_usable_fraction"])
    reserve = float(E["battery_reserve_power_W"]) * float(E["battery_reserve_time_min"]) / 60.0
    return {"generator_W_loiter": gen_W, "continuous_hd59_W": cont_hd59, "continuous_e180_W": cont_e180,
            "peak_e180_W": peak_e180, "peak_hd59_W": peak_hd59, "margin_continuous": gen_W / worst,
            "margin_peak_generator_only": gen_W / peak_e180,
            "peak_deficit_W": max(peak_e180 - gen_W, 0.0),
            "battery_model": E["battery_model"], "battery_usable_Wh": usable, "battery_reserve_Wh": reserve,
            "battery_peak_share_Wh": usable - reserve,
            "battery_reserve_holdup_min_at_reserve_power": float(E["battery_reserve_time_min"])}


def research_item(ref: str):
    """``components.yaml#categories.avionics.autopilots[veronte_autopilot_1x].dimensions_m`` -> value from a research
    file; ``[id]`` selects the list entry with that id; a ``{value: ...}`` leaf returns its value."""
    fname, path = ref.split("#", 1)
    cur = research(fname)
    for part in path.split("."):
        if "[" in part:
            key, sel = part[:-1].split("[")
            cur = next(e for e in cur[key] if e.get("id") == sel)
        else:
            cur = cur[part]
    return cur["value"] if isinstance(cur, dict) and "value" in cur else cur


def content_envelope(it: dict) -> dict:
    """Installed envelope of one bay content item (layout.rules.bay_contents): datasheet box (``ref`` into the research
    files, or ``dims``, or a cell-pack rule ``pack``) in the installed orientation (``orient``: datasheet axis for x, y,
    z), plus the connector / harness allowance on one face (``connector_face`` +x/-x/+y/-y/+z, ``connector_allowance``).
    Returns the body and the allowance-inclusive boxes [[x0, y0, z0], [x1, y1, z1]] about ``center`` (the body centre)."""
    if "pack" in it:
        p = it["pack"]
        d, h = float(p["cell_diameter"]), float(p["cell_height"])
        dims = [p["n_x"] * d + 2 * p["wall"], p["n_y"] * d + 2 * p["wall"], h + p["bottom"] + p["top"]]
    else:
        raw = research_item(it["ref"]) if "ref" in it else it["dims"]
        raw = [float(v) for v in raw]
        o = it.get("orient", [0, 1, 2])
        dims = [raw[o[0]], raw[o[1]], raw[o[2]]]
    c = np.asarray(it["center"], float)
    half = 0.5 * np.asarray(dims, float)
    body = np.array([c - half, c + half])
    env = body.copy()
    a = float(it.get("connector_allowance", 0.0))
    face = it.get("connector_face", "+x")
    ax = {"x": 0, "y": 1, "z": 2}[face[1]]
    if face[0] == "+":
        env[1, ax] += a
    else:
        env[0, ax] -= a
    return {"dims": dims, "body": body, "envelope": env}


def _box_surface_points(b: np.ndarray, n: int = 5) -> np.ndarray:
    g = [np.linspace(b[0, k], b[1, k], n) for k in range(3)]
    P = np.array(np.meshgrid(*g, indexing="ij")).reshape(3, -1).T
    on = np.zeros(len(P), bool)
    for k in range(3):
        on |= np.isclose(P[:, k], b[0, k]) | np.isclose(P[:, k], b[1, k])
    return P[on]


def bay_contents_check(S: dict, af: Airframe) -> dict:
    """V3-02 (fix round 4): the real component envelopes declared for the equipment zones (layout.rules.bay_contents)
    packed inside their zone boxes: every envelope (datasheet box + connector allowance) inside the OML with
    ``clearance_to_oml``, inside its zone box (port-side items in the mirrored half of a symmetric side-bay pair) with
    ``clearance_to_wall``, and at least ``gap_between_items`` from every other content item; the zone boxes themselves
    are checked by packaging() (inside the OML, no overlap with the other zones and the fuel cells)."""
    BC = S["layout"]["rules"].get("bay_contents")
    if not BC:
        return {"fits": True, "items": [], "failures": []}
    Z = S["layout"]["zones_preliminary"]
    c_oml, c_wall, gap = (float(BC[k]) for k in ("clearance_to_oml", "clearance_to_wall", "gap_between_items"))
    rows, fails = [], []
    envs = {}
    for it in BC["items"]:
        e = content_envelope(it)
        env = e["envelope"]
        envs[it["name"]] = env
        P = _box_surface_points(env)
        lo_, hi_ = -0.05, 0.10                                 # largest OML margin of the envelope (bisection)
        for _ in range(28):
            mid = 0.5 * (lo_ + hi_)
            lo_, hi_ = (mid, hi_) if np.all(af.inside(P, margin=mid)) else (lo_, mid)
        zb = np.asarray(Z[it["zone"]]["box"], float)
        zlo, zhi = zb[0].copy(), zb[1].copy()
        if it.get("side") == "port":                          # symmetric zone: the port box mirrors the starboard one
            zlo[1], zhi[1] = -zb[1, 1], -zb[0, 1]
        elif zb[0, 1] <= 0.0:
            zlo[1] = -zb[1, 1]
        wall = float(min(np.min(env[0] - zlo), np.min(zhi - env[1])))
        rows.append({"name": it["name"], "zone": it["zone"], "dims_m": e["dims"], "envelope": env.tolist(),
                     "oml_margin_m": lo_, "zone_wall_margin_m": wall, "mass_kg": it.get("mass_kg")})
        if lo_ < c_oml - 1e-9:
            fails.append(f"{it['name']}: {lo_ * 1000:.1f} mm inside the OML < {c_oml * 1000:.0f} mm")
        if wall < c_wall - 1e-9:
            fails.append(f"{it['name']}: outside its zone {it['zone']} (wall margin {wall * 1000:.1f} mm)")
    names = list(envs)
    gaps = {}
    for i in range(len(names)):
        for j in range(i + 1, len(names)):
            a, b = envs[names[i]], envs[names[j]]
            sep = np.maximum(np.maximum(b[0] - a[1], a[0] - b[1]), 0.0)
            ov = np.minimum(a[1], b[1]) - np.maximum(a[0], b[0])
            d = float(np.linalg.norm(sep)) if np.any(sep > 0) else -float(np.min(ov))
            gaps[f"{names[i]}/{names[j]}"] = d
            if d < gap - 1e-9:
                fails.append(f"{names[i]} / {names[j]}: gap {d * 1000:.1f} mm < {gap * 1000:.0f} mm")
    for r in rows:
        r["min_gap_to_other_items_m"] = min([v for k, v in gaps.items() if r["name"] in k.split("/")] or [float("inf")])
    return {"fits": not fails, "items": rows, "failures": fails, "min_gap_m": min(gaps.values()) if gaps else None,
            "clearance_to_oml_m": c_oml, "clearance_to_wall_m": c_wall, "gap_between_items_m": gap}


def bay_content_centroid(S: dict, names: tuple) -> tuple:
    """Mass-weighted centre (x, y, z) of the bay content items ``names`` (body boxes; mass_kg of the items)."""
    its = [it for it in S["layout"]["rules"]["bay_contents"]["items"] if it["name"] in names]
    m = sum(float(it["mass_kg"]) for it in its)
    c = sum(float(it["mass_kg"]) * np.asarray(it["center"], float) for it in its) / m
    return float(c[0]), float(c[1]), float(c[2])


def fuel_capacity_kg(S: dict, af: Airframe) -> float:
    """Usable fuel the bladder cells hold (kg): cell volume (area band x length, inset) x tank volume efficiency /
    (1 + expansion space) x fuel density (the packaging() volume)."""
    eff = float(S["structures"]["fuel"]["tank_volume_efficiency"])
    vol = 0.0
    for c in S["layout"]["fuel_cells"]:
        xs = np.linspace(c["x"][0], c["x"][1], 30)
        Ais = np.array([_area_band(af, x, c["z"][0], c["z"][1], float(c.get("inset", 0.025))) for x in xs])
        vol += float(np.trapz(Ais, xs)) * eff
    return vol / (1 + float(S["structures"]["fuel"]["expansion_fraction"])) * \
        float(S["engine"]["fuel"]["density_kg_per_m3"])


def packaging(S: dict, af: Airframe, mass_c: list) -> dict:
    """Keep-out boxes of the spec layout zones checked against the OML (corners and edge points inside with the zone
    clearance; the gear wells and the turret bay are checked on their real envelopes), pairwise overlap of the zones
    and the fuel cells, fuel volume (bladder cells x tank efficiency) vs the required volume."""
    Z = S["layout"]["zones_preliminary"]
    out = {}
    envelope_checked = {"nose_gear_well", "main_gear_wells", "turret_bay", "stabilator_spindle"}
    for k, z in Z.items():
        if "box" not in z or k in envelope_checked:
            continue
        box = np.asarray(z["box"], float)
        m = float(z.get("clearance", 0.010))
        if z.get("symmetric", False) and box[0, 1] > 0.0:      # side-bay pair (y0..y1, mirrored): starboard box
            ys = [box[0, 1], 0.5 * (box[0, 1] + box[1, 1]), box[1, 1]]
        else:
            ys = [-box[1, 1], 0.0, box[1, 1]] if z.get("symmetric", False) else [box[0, 1], box[1, 1]]
        pts = np.array([[x, y, zz] for x in np.linspace(box[0, 0], box[1, 0], 5) for y in ys
                        for zz in (box[0, 2], box[1, 2])])
        ok = bool(np.all(af.inside(pts, margin=m)))
        out[k] = {"box": box.tolist(), "fits": ok, "clearance_m": m}
    # V3-02 (fix round 4): the declared contents of the equipment zones, packed with their real envelopes
    out["bay_contents"] = bay_contents_check(S, af)
    gs = gear_stowage(S, af)
    out["main_gear_wells"] = {"fits": gs["main"]["ok"], **{k: v for k, v in gs["main"].items() if k != "ok"}}
    out["nose_gear_well"] = {"fits": gs["nose"]["ok"]}
    out["turret_bay"] = {"fits": bool(turret_checks_light(S, af))}
    if "stabilator_spindle" in Z:
        sc = stab_spindle_check(S, af)
        out["stabilator_spindle"] = {"fits": sc["ok"], **sc}
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


def stab_spindle_check(S: dict, af: Airframe) -> dict:
    """Stabilator stub spindles (bearing housings of radius spindle_housing_radius), starboard (port mirrored):
    ahead of the cylinder/head envelope with the clearance, inboard end clear of the crankcase/SG750 envelope, the
    spindle inside the body between the inboard bearing and the stub root, and inside the fixed root stub section
    (thickness at the spindle chord station >= housing diameter + 2 x 3 mm skin) out to the moving root."""
    st_ = S["tail"]["surfaces"]["stabilator"]
    hp = st_["params"]
    pv = np.asarray(st_["pivot"], float)
    r_s = float(hp["spindle_housing_radius"])
    cyl_gap = (engine_cylinder_front_x(S, pv[2]) - pv[0]) * math.cos(_cyl_eps(S)) - r_s   # normal to the inclined face
    y_in = spindle_inboard_y(S)
    sbp = S["tail"]["surfaces"]["stabilator_stub"]["params"]
    ys = np.linspace(y_in, float(sbp["y_in"]), 12)
    inside = af.inside(np.column_stack([np.full_like(ys, pv[0]), ys, np.full_like(ys, pv[2])]), margin=r_s)
    sb = S["tail"]["surfaces"]["stabilator_stub"]["sections"][0]
    xc = (pv[0] - sb["x_le"]) / sb["chord"]
    t_stub = section_depth_at(sb["airfoil"], float(sb.get("thickness_scale", 1.0)), float(sb["chord"]), xc)
    stub_margin = t_stub - 2 * (r_s + 0.003)
    return {"cylinder_clearance_m": float(cyl_gap), "inboard_end_y_m": y_in, "stub_thickness_at_spindle_m": float(t_stub),
            "stub_section_margin_m": float(stub_margin), "spindle_chord_fraction_at_root": float(xc),
            "inside_body": bool(np.all(inside)),
            "ok": bool(cyl_gap >= float(hp["spindle_engine_clearance"]) - 1e-6 and stub_margin >= 0.0 and
                       np.all(inside))}


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


def engine_cylinder_front_x(S: dict, z: float | None = None) -> float:
    """Front face of the cylinder/head envelope: engine flange - engine.envelope.cylinder_slab_from_flange[1]. With
    ``z`` (layout phase): x of that face at height z, the envelope being square to the crank / thrust axis inclined by
    propeller.thrust_line_inclination_deg through propeller.hub (5 deg aft-up: the face leans forward above the axis)."""
    x0 = float(S["fuselage"]["lines"]["x_hub"]) - float(S["engine"]["envelope"]["cylinder_slab_from_flange"][1])
    if z is None:
        return x0
    pr = S["propeller"]
    eps = math.radians(float(pr.get("thrust_line_inclination_deg", 0.0)))
    hub = np.asarray(pr["hub"], float)
    u1 = float(hub[0]) - x0
    fx, fz = hub[0] - u1 * math.cos(eps), hub[2] - u1 * math.sin(eps)
    return float(fx - (float(z) - fz) * math.tan(eps))


def _cyl_eps(S: dict) -> float:
    return math.radians(float(S["propeller"].get("thrust_line_inclination_deg", 0.0)))


def spindle_inboard_y(S: dict) -> float:
    """Inboard end of the stabilator stub spindles: crankcase/SG750 half width + the engine clearance rule."""
    lay = S["layout"]
    cv = lay.get("clearance_values") or (lay.get("clearances") if isinstance(lay.get("clearances"), dict) else {})
    return 0.10 + float(cv["engine_keep_out"])


def stab_ac_fractions(hp: dict, sbp: dict) -> tuple:
    """Aerodynamic centre of the moving stabilator panels as a fraction of the panel MAC, by vortex lattice (16 x 6
    panels per side, flat camber line): (panels alone, panels next to the fixed root stubs). The AC fraction does not
    depend on the panel's x station or height, so provisional sections are used."""
    y0 = float(hp.get("y_root", 0.35))
    prov = surface_from_params("stabilator", dict(hp, y_root=y0, z_root=0.0))
    sp = stab_panel(hp)
    pp = panel_planform(prov)
    g = vlm_panels(surface_strips(prov, 16, True), 6, tag="stab")
    sol = vlm_solve([g], 2 * pp["area"], sp["mac"], float(prov[0]["x_le"]) + sp["x_le_mac"] - float(hp["x_le_root"]))
    x_le_mac = float(prov[0]["x_le"]) + sp["x_le_mac"] - float(hp["x_le_root"])
    f_iso = (sol["x_np"] - x_le_mac) / sp["mac"]
    y_in = min(float(sbp.get("y_in", 0.2)), y0 - 0.03)
    y_out = y0 - float(hp["y_root_gap"])
    stub = [{"y": y_in, "x_le": float(prov[0]["x_le"]), "z_le": 0.0, "chord": float(hp["root_chord"]), "twist_deg": 0.0,
             "airfoil": hp["airfoil"]},
            {"y": y_out, "x_le": float(prov[0]["x_le"]), "z_le": 0.0, "chord": float(hp["root_chord"]), "twist_deg": 0.0,
             "airfoil": hp["airfoil"]}]
    g2 = vlm_panels(surface_strips(stub, 3, True, cosine=False), 6, tag="stub")
    sol2 = vlm_solve([g, g2], 2 * pp["area"], sp["mac"], x_le_mac)
    f_stub = (sol2["parts"]["stab"]["x_load"] - x_le_mac) / sp["mac"]
    return float(f_iso), float(f_stub)


def fin_te_crossing(fp: dict, z_hub: float, eps: float = 0.0) -> float:
    """Span fraction t* along the fin trailing edge where it reaches ``fp['guard_radius']`` from the hub in the
    propeller disc plane, which is inclined by the thrust-line angle ``eps`` (rad): a point at height z on the disc
    plane lies at x = x_hub - (z - z_hub) tan(eps) and at the radius hypot(y, (z - z_hub) / cos(eps)) (inf if the fin
    never reaches the guard radius)."""
    g = math.radians(fp["cant_deg"])
    t = np.linspace(0.0, 1.0, 2001)
    y = fp["y_root"] + t * fp["span"] * math.sin(g)
    z = fp["z_root"] + t * fp["span"] * math.cos(g)
    r = np.hypot(y, (z - z_hub) / math.cos(eps))
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
    C = TS["stabilator"]["controls"]
    # (1) panel aerodynamic centre by vortex lattice -> AC band -> spindle at its forward end (F2): the surface is
    #     never statically unstable about the spindle and the hinge moments come from the band width only
    f_iso, f_stub = stab_ac_fractions(hp, TS["stabilator_stub"]["params"])
    band = [round(min(f_iso, f_stub) - float(C["ac_band_fwd"]), 4), round(max(f_iso, f_stub) + float(C["ac_band_aft"]), 4)]
    C["ac_mac_fraction_vlm"] = {"panels_only": round(f_iso, 4), "with_root_stubs": round(f_stub, 4)}
    C["ac_mac_fraction_range"] = band
    hp["pivot_mac_fraction"] = band[0]
    # (2) two stub spindles (no cross-tube: the crankcase/SG750 fills the centre of the bay behind the firewall),
    #     each in an inboard bearing in an engine-bay ring frame beside the SG750 and an outboard bearing in the fixed
    #     root stub; the spindle station is as far aft as the cylinders allow (longest tail arm) and the panel root
    #     leading edge follows from it
    x_piv_max = engine_cylinder_front_x(S, float(TS["stabilator"]["pivot"][2])) - \
        (float(hp["spindle_housing_radius"]) + float(hp["spindle_engine_clearance"])) / math.cos(_cyl_eps(S))
    hp["x_le_root"] = round(float(hp["x_le_root"]) + x_piv_max - stab_panel(hp)["x_pivot"], 5)
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
        eps_t = math.radians(float(pr.get("thrust_line_inclination_deg", 0.0)))
        tc = fin_te_crossing(fp, float(L["z_t"]), eps_t)
        if not math.isfinite(tc):
            raise ValueError("fin too short to reach the propeller-guard radius")
        dx_te = tc * (fp["span"] * math.tan(math.radians(fp["sweep_le_deg"])) + fp["tip_chord"] - fp["root_chord"])
        z_tc = fp["z_root"] + tc * fp["span"] * math.cos(g)
        x_disc = x_plane - (z_tc - float(L["z_t"])) * math.tan(eps_t)        # inclined disc plane at that height
        x_new = round(x_disc - fp["root_chord"] - dx_te, 5)
        if abs(x_new - fp["x_le_root"]) < 1e-5:
            break
        fp["x_le_root"] = x_new
    fp["te_crossing_span_fraction"] = round(tc, 5)
    fp["root_max_thickness"] = round(t_fin, 5)
    TS["fin"]["sections"] = surface_from_params("fin", fp)
    # ---- ventral fin / bumper: lower propeller guard (F10). Root buried in the keel along the whole root chord (both
    # faces of the root airfoil above the body bottom surface + root_embed); tip trailing edge on the (thrust-line
    # inclined) propeller disc plane + guard_te_aft_of_disc, so the trailing edge crosses the disc plane outside the
    # tip circle and the replaceable bumper skid sits directly under the disc; span from the bumper rule (closure)
    vp = TS["ventral"]["params"]
    xs = np.linspace(vp["x_le_root"], vp["x_le_root"] + vp["root_chord"], 40)
    th = np.interp(np.linspace(0, 1, 40), *airfoil_thickness(vp["airfoil"])) * vp["root_chord"]
    z_face = np.maximum(af0.z_bot(xs, 0.5 * th), af0.z_bot(xs, 0.0))
    vp["z_root"] = round(float(np.max(z_face)) + vp["root_embed"], 5)
    eps_t = math.radians(float(pr.get("thrust_line_inclination_deg", 0.0)))
    z_tip = vp["z_root"] - vp["span"]
    x_te_tip = x_plane + (float(L["z_t"]) - z_tip) * math.tan(eps_t) + float(vp["guard_te_aft_of_disc"])
    vp["sweep_le_deg"] = round(math.degrees(math.atan2(x_te_tip - vp["tip_chord"] - vp["x_le_root"], vp["span"])), 4)
    TS["ventral"]["sections"] = surface_from_params("ventral", vp)
    tip = TS["ventral"]["sections"][-1]
    TS["ventral"]["bumper"] = {"contact_point": _round([tip["x_le"] + vp["skid_chord_fraction"] * tip["chord"], 0.0,
                                                        tip["z_le"] - vp["skid_height"]]),
                               "skid": "replaceable UHMW-PE shoe on a 4130 strap at the ventral-fin tip, under the "
                                       "propeller disc"}
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
        # y_inner > 0: a pair of side boxes (y_inner..half_width, mirrored), e.g. beside the nose-gear keel slot
        zones[k] = {"box": [[b["x"][0], float(b.get("y_inner", 0.0)), b["z"][0]], [b["x"][1], b["half_width"], b["z"][1]]],
                    "symmetric": True, "clearance": b.get("clearance", 0.008), "content": b.get("content", "")}
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
    c0, c1 = (float(v) for v in E["cylinder_slab_from_flange"])            # cylinders: c0..c1 ahead of the flange
    zones["engine_cylinder_slab"] = {"box": [[xh - c1, 0.0, zt - E["crank_axis_to_cylinder_side"]],
                                             [xh - c0, E["width"] / 2, zt + E["crank_axis_to_cylinder_side"]]],
                                     "symmetric": True, "clearance": 0.010, "content": "L 275 EF cylinders/heads"}
    zones["engine_intake_box"] = {"box": [[xh - c1, 0.0, zt - (E["height"] - E["crank_axis_to_cylinder_side"])],
                                          [xh - c0, E["intake_box_width"] / 2, zt - E["crank_axis_to_cylinder_side"]]],
                                  "symmetric": True, "clearance": 0.010, "content": "intake box below the crank"}
    zones["engine_crankcase_sg750"] = {"box": [[xh - E["length_with_sg750"], 0.0, zt - 0.10], [xh - 0.02, 0.10, zt + 0.05]],
                                       "symmetric": True, "clearance": 0.010, "content": "crankcase + SG750"}
    zones["firewall_x"] = {"x": xh - E["length_with_sg750"] - R["firewall_gap"]}
    st_ = S["tail"]["surfaces"]["stabilator"]
    pv, r_s = st_["pivot"], float(st_["params"]["spindle_housing_radius"])
    y_sp = spindle_inboard_y(S)
    zones["stabilator_spindle"] = {"box": _round([[pv[0] - r_s, y_sp, pv[2] - r_s], [pv[0] + r_s, pv[1], pv[2] + r_s]]),
                                   "symmetric": True, "clearance": 0.0,
                                   "content": "two stabilator stub spindles (starboard box shown, port mirrored): inboard "
                                              "bearing in the engine-bay ring frame beside the crankcase/SG750, outboard "
                                              "bearing in the fixed root stub; ahead of the cylinders; checked by "
                                              "stab_spindle_check"}
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
    av = ctr("avionics_power_deck")
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
    n_cells = len(S["layout"].get("fuel_cells") or []) or (3 if float(S["layout"]["rules"]["fuel_cell_aft_length"]) > 0
                                                          else 2)
    add(f"fuel_system_{n_cells}_cells", "fuel", MR["fuel_system_kg"], fuel_x, 0.05,
        f"components.yaml fuel_system 1.73 kg + interconnection of {n_cells} bladder cells "
        "(endurance study: +0.35 kg for two, +0.50 kg for three)")
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
        f"spindles through {sc['linkage_ratio']:.1f}:1 four-bar linkages 0.10 + spindle cranks 0.12 (estimate); "
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
    gd = gear_doors_mass(S)
    gpk, gg, GR = gd["parts_kg"], gd["geometry"], MR["gear_doors"]
    add("gear_doors_wells_locks_sensors", "gear", gd["total_kg"], gd["x"], -0.10,
        f"V4-03 (fix round 5), door scheme landing_gear.doors: 6 doors {gg['area_total_m2']:.3f} m2 (main inner "
        f"{gg['area_main_inner_m2']:.3f} + leg {gg['area_main_leg_m2']:.3f} + nose clamshells {gg['area_nose_m2']:.3f}) x "
        f"{float(GR['door_areal_kg_per_m2']):.3f} kg/m2 (identity study 0.41 kg / 0.15 m2, sandwich doors incl. hinges and "
        f"links) {gpk['doors']:.3f} + well close-outs {gpk['well_close_outs']:.2f} + cut-out reinforcement "
        f"{gpk['cut_out_reinforcement']:.2f} + gear locks/sensors {gpk['locks_sensors']:.2f} + "
        + (f"perimeter door seals {gpk['seals']:.3f} ({float(GR['seal_kg_per_m']) * 1000:.1f} g/m x "
           f"{gg['joint_total_m']:.2f} m of joint) + " if gd["sealed"] else "") +
        f"inner-door actuators {gpk['inner_door_actuators']:.3f} ({int(GR['inner_door_actuator']['count'])} x "
        f"{GR['inner_door_actuator']['model']}, datasheet) + their linkages {gpk['inner_door_linkages']:.2f} + leg-door "
        f"standoff brackets {gpk['leg_door_brackets']:.2f} (estimates)")
    # V3-02 (fix round 4): positions of the avionics/power items = mass-weighted centres of their packed contents
    # (layout.rules.bay_contents: real envelopes in the nose deck above the nose-gear well and in the side bays)
    ax_, ay_, az_ = bay_content_centroid(S, ("autopilot", "datalink_primary", "datalink_backup", "transponder",
                                            "remote_id"))
    add("avionics", "systems", 0.62, ax_, az_, "components.yaml avionics.recommended.mass_estimate_kg (at the packed "
        "autopilot / datalinks / transponder / Remote ID; probes and antennas counted in the same item)", y=ay_)
    EB = S["engine"]["electrical_budget"]
    bx_, by_, bz_ = bay_content_centroid(S, ("buffer_battery",))
    add("buffer_battery_12S2P_liion", "systems", float(EB["battery_mass_kg"]), bx_, bz_,
        f"{EB['battery_model']}: components.yaml#categories.electrical.buffer_batteries[liion_12s2p_molicel_p45b] "
        "1.98 kg (24 x 0.070 kg cells + 0.30 kg BMS/case/wiring); replaces the 14S2P LiFePO4 2.43 kg (V2-01, R-52 "
        "peak-support share; engine.sources.buffer_battery); port side bay", y=by_)
    px_, py_, pz_ = bay_content_centroid(S, ("pdu", "dcdc_28_12", "contactor_fuses"))
    add("pdu_dcdc_fuses", "systems", 2.10, px_, pz_, "components.yaml electrical: PDU 1.60 + DC-DC 0.10 + "
        "fuses/contactor 0.40 (deck above the nose-gear well)", y=py_)
    add("wiring_harness_connectors_coax", "systems", MR["harness_kg"], 1.9, 0.0,
        "baseline 2.50 kg + 0.20 kg longer wing harness (endurance study); distributed")
    pa = MR["parachute"]
    add(pa["name"], "systems", pa["mass_kg"], pc[0], pc[2], pa["basis"])
    add("flight_termination_lights", "systems", 0.15 + 3 * 0.083, 0.6 * av[0] + 0.4 * x_mac40, 0.02,
        "independent FTS 0.15 + 3 x AveoFlash 0.083 (components.yaml recovery_and_safety)")
    add("turret_lift_mechanism_doors", "systems", MR["turret_mechanism_kg"], tb[0], tb[2] + 0.06,
        "ball-screw linear stage + BLDC/brake 0.45, guide rails/carriage 0.35, two bay doors + linkage 0.30, bay "
        "liner/frame 0.40, controller/sensors 0.10" + (" + HD59 aperture ring 0.06 (flush skin insert)" if
                                                      S["payload"]["turret"]["bay"].get("aperture_ring") else "") +
        " (estimate, no catalogue unit)")
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
    add("stabilator_spindle_bearing_housings", "chassis", 0.40, pv[0], pv[2], "estimate: two stub spindles "
        "(no cross-tube) + 2 bearing housings per side (engine-bay ring frame beside the SG750 and root-stub rib)")
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
    apply_mass_placement(S, I)
    return I


def apply_mass_placement(S: dict, I: list) -> None:
    """Layout-phase positions: ``layout.mass_placement[name].position`` (centroid of the objects the layout places for
    that mass item: members, fittings, equipment, harness trunks, panels) replaces the sizing heuristic position of
    the item; ucav250.analysis.layout_check recomputes the centroids from the layout and verifies them."""
    mp = (S.get("layout") or {}).get("mass_placement") or {}
    for it in I:
        p = mp.get(it["name"])
        if not p:
            continue
        x, y, z = (float(v) for v in p["position"])
        it.update({"x": round(x, 4), "y": round(y, 4), "z": round(z, 4),
                   "basis": it["basis"] + "; position: layout.mass_placement (layout phase)"})


BASELINE_PAYLOAD_ITEMS = ("eo_ir_turret_hd59_mount", "mission_computer_recorder", "payload_tray_harness")


def payload_baseline_kg(S: dict) -> float:
    """Baseline EO/IR set (HD59 turret + mount, mission computer/recorder, tray/harness): the payload every mission
    carries; the rest of a payload is research-payload allowance in the belly bay."""
    PS = S["payload"]
    return float(PS["turret_mass_kg"]) + float(PS["mission_computer_kg"]) + float(PS["tray_harness_kg"])


def payload_items(S: dict, lz: dict) -> list:
    """Payload mass items. Requirement decision (fix round 4): the design-mission payload (mission.payload_design_kg,
    derived by the payload-endurance rule) = baseline set + research_payload_allowance; the maximum payload
    (mission.payload_max_kg, R-02b) adds research_payload_max_increment in the same belly bay."""
    PS = S["payload"]
    mis = S["mission"]
    tb = lz["zones"]["turret_bay"]
    T = PS["turret"]
    pb = np.asarray(lz["zones"]["payload_bay"]["box"], float)
    xb = 0.5 * (pb[0, 0] + pb[1, 0])
    zb = 0.5 * (pb[0, 2] + pb[1, 2])
    xt = 0.5 * (tb["box"][0][0] + tb["box"][1][0])
    mcb = np.asarray(lz["zones"]["mission_computer"]["box"], float)
    mc = 0.5 * (mcb[0] + mcb[1])
    p_d, p_x, p_b = float(mis["payload_design_kg"]), float(mis["payload_max_kg"]), payload_baseline_kg(S)
    if not p_b - 1e-9 <= p_d <= p_x + 1e-9:
        raise ValueError(f"design payload {p_d} kg outside [baseline set {p_b}, maximum {p_x}] kg")
    if abs(p_b + float(PS["research_allowance_max_kg"]) - p_x) > 1e-6:
        raise ValueError("payload.research_allowance_max_kg + baseline set must equal mission.payload_max_kg")
    return [{"name": "eo_ir_turret_hd59_mount", "mass_kg": PS["turret_mass_kg"], "x": round(xt, 4), "y": 0.0,
             "z": round(T["ball_center_retracted_z"], 4), "z_extended": round(T["ball_center_extended_z"], 4),
             "basis": "Trillium HD59-LLVV 1.55 kg datasheet + isolator/mount 0.20 kg (baseline.yaml#payload_set)"},
            {"name": "mission_computer_recorder", "mass_kg": PS["mission_computer_kg"], "x": round(mc[0], 4), "y": 0.0,
             "z": round(mc[2], 4), "basis": "baseline.yaml#payload_set"},
            {"name": "payload_tray_harness", "mass_kg": PS["tray_harness_kg"], "x": round(xb, 4), "y": 0.0,
             "z": round(zb, 4), "basis": "baseline.yaml#payload_set.payload_tray_harness_kg"},
            {"name": "research_payload_allowance", "mass_kg": round(p_d - p_b, 4), "x": round(xb, 4), "y": 0.0,
             "z": round(zb, 4), "basis": "design-mission payload (mission.payload_design_kg, payload-endurance rule) "
                                         "minus the baseline set; belly bay on the CG"},
            {"name": "research_payload_max_increment", "mass_kg": round(p_x - p_d, 4), "x": round(xb, 4), "y": 0.0,
             "z": round(zb, 4), "basis": "maximum payload (mission.payload_max_kg, R-02b; baseline.yaml research "
                                         "allowance 16.75 kg) minus the design-mission payload; same belly bay"},
            {"name": "e180_growth_turret_delta", "mass_kg": round(PS["growth_turret_mass_kg"] - PS["turret_mass_kg"], 4),
             "x": round(xt, 4), "y": 0.0, "z": round(T["ball_center_retracted_z"], 4),
             "z_extended": round(T["ball_center_extended_z"], 4),
             "basis": "Octopus E180 class 4.0 kg incl. mount (baseline growth envelope) minus the HD59 set"},
            {"name": "research_payload_reduced_for_e180", "mass_kg": round(-(PS["growth_turret_mass_kg"] -
                                                                           PS["turret_mass_kg"]), 4),
             "x": round(xb, 4), "y": 0.0, "z": round(zb, 4),
             "basis": "keeps the design-mission payload with the E180 (research allowance reduced by the turret "
                      "mass difference)"}]


def sm_range_items(S: dict, stab: dict, cbar: float, m_add: float, x_add: float, fuel_kg: float) -> list:
    """Static-margin range [min, max] (fraction of MAC) of the empty aircraft + one added mass (m_add at x_add) over the
    fuel load 0 .. fuel_kg (fuel at mass.fuel_cg)."""
    M = S["mass"]
    m_z = sum(float(i["mass_kg"]) for i in M["items"]) + m_add
    mx = sum(float(i["mass_kg"]) * float(i["x"]) for i in M["items"]) + m_add * x_add
    fx = float(M["fuel_cg"][0])
    xs = [mx / m_z, (mx + fuel_kg * fx) / (m_z + fuel_kg)]
    return [(stab["x_np"] - max(xs)) / cbar, (stab["x_np"] - min(xs)) / cbar]


def payload_loading_cg_fn(S: dict, payload_kg: float):
    """(f, m_zero_fuel) of a loading with ``payload_kg`` of payload (0 = no payload at all; otherwise the baseline set
    at its items + the rest as research payload in the belly bay): x of the CG as a function f(m_fuel) of the fuel on
    board (fuel at mass.fuel_cg). Payload-endurance sweep and the design-payload rule; for the design payload it is
    the loading of mass.cases[0]."""
    M = S["mass"]
    pay = {p["name"]: p for p in M["payload_items"]}
    its = [(float(i["mass_kg"]), float(i["x"])) for i in M["items"]]
    p_b = payload_baseline_kg(S)
    if payload_kg > 1e-9:
        if payload_kg < p_b - 1e-9:
            raise ValueError(f"payload {payload_kg} kg below the baseline set {p_b} kg")
        its += [(float(pay[n]["mass_kg"]), float(pay[n]["x"])) for n in BASELINE_PAYLOAD_ITEMS]
        its.append((payload_kg - p_b, float(pay["research_payload_allowance"]["x"])))
    m_z = sum(m for m, _ in its)
    mx = sum(m * x for m, x in its)
    fx = float(M["fuel_cg"][0])
    return (lambda mf: (mx + mf * fx) / (m_z + mf)), m_z


def update_case_fuel(S: dict, fuel_cap: float | None = None) -> None:
    """Loading cases with ``fuel_rule: mtom`` (maximum-payload cases at MTOM): fuel fraction = (MTOM - zero-fuel
    mass) / mass.fuel_kg (the design-payload fuel); ``fuel_rule: capacity`` (V4-02, fix round 5: light payload with
    full tanks, the loadings of the payload-endurance table): fuel fraction = min(fuel capacity, MTOM - zero-fuel mass)
    / mass.fuel_kg. Written as a number (analysis/mass.py reads fuel_fraction)."""
    M = S["mass"]
    pay = {p["name"]: p["mass_kg"] for p in M["payload_items"]}
    empty = sum(i["mass_kg"] for i in M["items"])
    fuel = float(M["fuel_kg"])
    for c in M["cases"]:
        mz = empty + sum(pay[n] for n in c["payload"])
        if c.get("fuel_rule") == "mtom":
            c["fuel_fraction"] = round(min(max((float(M["mtow_kg"]) - mz) / fuel, 0.0), 1.0), 6)
        elif c.get("fuel_rule") == "capacity" and fuel_cap is not None:
            c["fuel_fraction"] = round(max(min(fuel_cap, float(M["mtow_kg"]) - mz), 0.0) / fuel, 6)


def ground_cases(S: dict, gp: dict, lz: dict, own_inertia: dict | None = None) -> list:
    """Loading cases with the gear EXTENDED (ground): gear items moved to their extended positions (with
    ``own_inertia``: pitch moment of inertia I_yy of every case, mass_cases)."""
    S2 = copy.deepcopy(S)
    r_t = S["landing_gear"]["tyre"]["diameter"] / 2
    for it in S2["mass"]["items"]:
        if it["name"].startswith("main_gear_legs"):
            it["z"] = round(0.5 * (gp["z_trunnion"] + gp["z_axle"]), 4)
        elif it["name"].startswith("nose_gear_leg"):
            it["x"] = round(gp["x_ng"], 4)
            it["z"] = round(gp["z_axle_nose"] + 0.4 * gp["nose_leg_length"], 4)
    return mass_cases(S2, own_inertia)


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
TAIL_DEADBAND = 0.004            # tail SHRINK updates below 0.4 % are not applied (growth is always applied: V1-11)
TAIL_OVERSHOOT = 0.002           # a needed tail growth is applied in full + 0.2 % (lands on the safe side of the rule)


def design_closure(S_in: dict, verbose: bool = True, max_iter: int = 30) -> dict:
    """Close the HANCER design point from the spec design rules (fixed MTOM; wing station for the static margin and
    tip-back; gear height for the propeller clearance and bumper rules; fuel = MTOM - empty - payload)."""
    S = copy.deepcopy(S_in)
    missing = write_glove_airfoils(S, check_only=True)
    if missing:
        raise RuntimeError(f"LERX/glove airfoil files missing or stale: {missing} (run --update-spec or --design)")
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
        S["payload"]["research_allowance_design_kg"] = round(pay - payload_baseline_kg(S), 4)
        update_case_fuel(S, fuel_capacity_kg(S, af))
        cases = mass_cases(S)
        gcases = ground_cases(S, gp, lz)
        stab = stability(S, af, wa, llm, cm0s, tp, cases)
        clm = clmax_set(S, wa, stab, min(c["x"] for c in cases), tp["stabilator"])
        pot = power_on_trim(S, wa, stab, tp["stabilator"], clm, prop_, cases, gcases)
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
            # V1-11: one-sided rules: a surface below its rule grows in full (+ TAIL_OVERSHOOT) at once; a surface above
            # it shrinks damped and only beyond the dead band, so the converged design always meets the rule
            clt = max(abs(clm["to_tail_cl"]), abs(clm["clean_tail_cl"]), pot["max_abs_tail_cl_local"])
            r_h = clt / float(trs["stabilator_trim_cl_max"])
            if r_h > 1.0:
                f_h = math.sqrt(r_h) * (1.0 + TAIL_OVERSHOOT)
            else:
                f_h = 1.0 + 0.6 * (math.sqrt(r_h) - 1.0)
                f_h = 1.0 if abs(f_h - 1.0) < TAIL_DEADBAND else f_h
            cn = stab["cn_beta"]
            need = (float(trs["cn_beta_target"]) - cn["body_used"] - cn["ventral"]) / max(cn["fins"], 1e-6)
            if need > 1.0:
                f_v = math.sqrt(need) * (1.0 + TAIL_OVERSHOOT)
            else:
                f_v = 1.0 + 0.6 * (math.sqrt(max(need, 0.25)) - 1.0)
                f_v = 1.0 if abs(f_v - 1.0) < TAIL_DEADBAND else f_v
        hist[-1].update({"stab_scale": f_h, "fin_scale": f_v})
        if verbose and trs:
            print(f"      tail: stabilator x{f_h:.4f} (tail CL {clt:.3f}), fins x{f_v:.4f} (Cnb {stab['cn_beta']['total']:.4f})",
                  flush=True)
        # V1-11: converged = no rule update AND the closure state repeats (fuel and wing station of the last two
        # iterations equal within 5 g / 0.1 mm); the wing-station step is halved when it changes sign (no 2-cycle)
        same = len(hist) > 1 and abs(hist[-1]["fuel"] - hist[-2]["fuel"]) < 0.005 and \
            abs(hist[-1]["x_c4_root"] - hist[-2]["x_c4_root"]) < 1e-4
        done = abs(dx) < 4e-4 and abs(d_mg) < 1.5e-3 and dS == 0.0 and f_h == 1.0 and f_v == 1.0 and same
        hist[-1]["converged"] = bool(done)
        if done and it > 1:
            break
        relax = 0.5 if (len(hist) > 1 and hist[-2]["d_sm"] * dx < 0.0) else 0.85
        hist[-1]["relax"] = relax
        S["wing"]["planform"]["x_c4_root"] = round(S["wing"]["planform"]["x_c4_root"] + relax * dx, 5)
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
          "cavity": "#0E1012",
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
    parts += _belly_seams(S, af, gear == "up", turret == "retracted")
    return parts


def _skin_patch(af: Airframe, xy_grid: np.ndarray, off: float = 0.0025, t: float = 0.002, close_u: bool = False):
    """Thin render-only solid that follows the belly skin over the (nu, nv, 2) grid of plan positions, ``off`` below
    the skin (outside) and ``t`` thick toward it (no z-fighting with the OML)."""
    from ..core.geom import shell_from_grid
    g = np.asarray(xy_grid, float)
    z = np.asarray(af.z_bot(g[..., 0].ravel(), g[..., 1].ravel()), float).reshape(g.shape[:2])
    P = np.dstack([g[..., 0], g[..., 1], z - off])
    inward = np.zeros_like(P)
    inward[..., 2] = 1.0
    return shell_from_grid(P, t, inward=inward, close_u=close_u)


def _plate(P: np.ndarray, normal, t: float = 0.004):
    """Thin closed plate (door) from a (nu, nv, 3) grid; thickness along ``normal``."""
    from ..core.geom import shell_from_grid
    P = np.asarray(P, float)
    inward = np.broadcast_to(np.asarray(normal, float), P.shape).copy()
    return shell_from_grid(P, t, inward=inward)


def _belly_seams(S: dict, af: Airframe, gear_up: bool, turret_in: bool) -> list:
    """Render-only belly details. Retracted states (V2-08): joint lines of the closed flush gear doors and of the turret
    aperture ring / bay doors, drawn as thin dark rods just outside the skin (the OML itself is continuous).
    Extended states (V3-10, fix round 4): the open wells (dark openings with the real well footprints) and the open
    doors (landing_gear.doors) - nose well: two clamshell doors hinged at the keel-slot edges, open 90 deg; main wells:
    sequenced inner doors (hinged at the inboard well edges, open only while the gear moves, closed again after the
    down-lock: shown closed) and a leg door carried by the leg on two standoff brackets (rotated with the leg by the
    retraction angle about the trunnion axis), leaving the leg slot open; turret: the aperture ring with the open annulus
    between the ring opening and the lowered ball (the bay doors are folded inside along the bay walls,
    payload.turret.bay)."""
    from ..core.geom import cylinder, sweep_circle
    Z = S["layout"].get("zones_preliminary", {})
    LG = S["landing_gear"]
    out = []

    def on_skin(xy):
        return np.array([[x, y, float(af.z_bot(x, y)) - 0.0015] for x, y in xy])

    def rect(name, x0, x1, y0, y1, split=None):
        n = 40
        xs = np.linspace(x0, x1, n)
        ys = np.linspace(y0, y1, max(int((y1 - y0) / 0.01), 8))
        loop = ([(x, y0) for x in xs] + [(x1, y) for y in ys[1:]] + [(x, y1) for x in xs[::-1][1:]] +
                [(x0, y) for y in ys[::-1][1:]])
        out.append((name, sweep_circle(on_skin(loop), 0.0025, 8), "dark"))
        if split is not None:
            out.append((name + "_split", sweep_circle(on_skin([(x, split) for x in xs]), 0.0025, 8), "dark"))

    def grid(x0, x1, y0, y1, nx=28, ny=10):
        X, Y = np.meshgrid(np.linspace(x0, x1, nx), np.linspace(y0, y1, ny), indexing="ij")
        return np.dstack([X, Y])
    mb = Z.get("main_gear_wells", {}).get("box")
    nb = Z.get("nose_gear_well", {}).get("box")
    y_in = float(LG["main"]["stowed_envelope"]["box"][0][1])
    if gear_up:
        if mb:                                                   # V4-07: inner doors + leg doors (landing_gear.doors)
            y_split = float(LG["doors"]["main_inner_door_outer_edge_y"])
            rect("seam_main_inner_doors", mb[0][0], mb[1][0], -y_split, y_split, split=0.0)
            for sg in (1, -1):
                y_a, y_b = sorted((sg * y_split, sg * mb[1][1]))
                rect(f"seam_main_leg_door{sg}", mb[0][0], mb[1][0], y_a, y_b)
        if nb:
            rect("seam_nose_doors", nb[0][0], nb[1][0], -nb[1][1], nb[1][1], split=0.0)
    else:
        if nb:                                                   # nose keel slot: open well + clamshell doors
            x0, x1, hw = nb[0][0], nb[1][0], nb[1][1]
            out.append(("nose_well_open", _skin_patch(af, grid(x0, x1, -hw, hw, ny=6)), "cavity"))
            xs = np.linspace(x0, x1, 24)
            for sg in (1, -1):
                zh = np.array([float(af.z_bot(x, sg * hw)) for x in xs])
                P = np.array([[[x, sg * hw, z - f * hw] for f in (0.0, 1.0)] for x, z in zip(xs, zh)])
                out.append((f"nose_door{sg}", _plate(P, (0.0, sg * 1.0, 0.0)), "skin_dark"))
        if mb:                                                   # main wells: leg slot open, inner doors re-closed
            x0, x1, y1 = mb[0][0], mb[1][0], mb[1][1]
            T = np.asarray(LG["main"]["trunnion"], float)
            A = np.asarray(LG["main"]["axle_static"], float)
            Cs = np.asarray(LG["main"]["retraction"]["stowed_wheel_center"], float)
            ang = math.atan2(A[2] - T[2], A[1] - T[1]) - math.atan2(Cs[2] - T[2], Cs[1] - T[1])
            ang = (ang + math.pi) % (2 * math.pi) - math.pi                 # stowed -> extended about +x
            y_split = float(LG["doors"]["main_inner_door_outer_edge_y"])
            xs = np.linspace(x0, x1, 24)
            rect("seam_main_inner_doors", x0, x1, -y_split, y_split, split=0.0)   # sequenced: closed after down-lock
            for sg in (1, -1):
                out.append((f"main_leg_slot_open{sg}", _skin_patch(af, grid(x0, x1, sg * y_split, sg * y1)), "cavity"))
                ys = np.linspace(y_split + 0.004, T[1] - 0.03, 6)
                Pst = np.array([[[x, y, float(af.z_bot(x, y)) - 0.004] for y in ys] for x in xs])
                c, s_ = math.cos(ang), math.sin(ang)
                dy, dz = Pst[..., 1] - T[1], Pst[..., 2] - T[2]
                Pex = np.dstack([Pst[..., 0], T[1] + c * dy - s_ * dz, T[2] + s_ * dy + c * dz])
                Ax = A.copy()
                if sg < 0:
                    Pex = Pex * np.array([1.0, -1.0, 1.0])
                    Ax = Ax * np.array([1.0, -1.0, 1.0])
                Tx = T * np.array([1.0, sg, 1.0])
                nrm = np.cross(Pex[1, 0] - Pex[0, 0], Pex[0, 1] - Pex[0, 0])
                out.append((f"main_leg_door{sg}", _plate(Pex, nrm / np.linalg.norm(nrm)), "skin_dark"))
                for k_, j_ in ((0, 1), (1, len(ys) - 2)):        # two standoff brackets leg -> door
                    q_ = Pex[len(xs) // 2 + (k_ * 2 - 1) * len(xs) // 5, j_]
                    d_ = Ax - Tx
                    f_ = float(np.clip(np.dot(q_ - Tx, d_) / np.dot(d_, d_), 0.1, 0.6))
                    out.append((f"main_leg_door_bracket{sg}_{k_}", cylinder(0.006, Tx + f_ * d_, q_, n=10), "metal"))
    tb = Z.get("turret_bay", {}).get("box")
    if tb:
        T_ = S["payload"]["turret"]
        ring = T_.get("bay", {}).get("aperture_ring")             # flush skin ring around the ball opening (V1-07)
        r_ap = 0.5 * float(T_["ball_diameter"]) + (float(ring["radial_clearance"]) if ring else 0.005)
        xc = float(T_["bay_center_x"])
        rect("seam_turret_ring_insert", tb[0][0], tb[1][0], -tb[1][1], tb[1][1])
        th = np.linspace(0.0, 2 * math.pi, 73)
        out.append(("seam_turret_ring", sweep_circle(on_skin([(xc + r_ap * math.cos(t), r_ap * math.sin(t))
                                                               for t in th]), 0.0025, 8), "dark"))
        if turret_in:                                              # bay doors closing the ring opening (split line)
            out.append(("seam_turret_doors_split", sweep_circle(on_skin([(xc + r_ap * f, 0.0)
                                                                         for f in np.linspace(-1, 1, 20)]), 0.0025, 8),
                        "dark"))
        else:                                                      # open annulus around the lowered ball
            zs = float(af.z_bot(xc))
            r_b = 0.5 * float(T_["ball_diameter"])
            dz = zs - float(T_["ball_center_extended_z"])
            r_in = math.sqrt(max(r_b ** 2 - dz ** 2, 0.0)) if abs(dz) < r_b else 0.5 * float(T_["stem_diameter"])
            tt = np.linspace(0.0, 2 * math.pi, 72, endpoint=False)
            rr = np.linspace(r_in + 0.0005, r_ap, 4)
            G_ = np.array([[(xc + r * math.cos(t), r * math.sin(t)) for r in rr] for t in tt])
            out.append(("turret_aperture_open", _skin_patch(af, G_, close_u=True), "cavity"))
    return out


VIEWS = {"iso": ((-1.0, -0.95, 0.62), (0, 0, 1), False), "rear": ((1.0, -0.85, 0.50), (0, 0, 1), False),
         "top": ((0, 0, 1), (-1, 0, 0), True), "side": ((0, -1, 0), (0, 0, 1), True),
         "front": ((-1, 0, 0), (0, 0, 1), True),
         # V2-08: low view of the belly from the front quarter (flush gear doors and the retracted turret); no ground
         "belly": ((-0.55, -0.50, -0.90), (0, 0, 1), False)}


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
def make_propulsion(S: dict, table_key: str | None = None, elec_load_W: float | None = None,
                    k_inst: float | None = None) -> tuple:
    """(Engine, Prop) of the spec. ``elec_load_W``: continuous DC load the generator carries (default: base load with
    the HD59 + research-payload allowance; the E180 growth mission passes its own load, V3-08); ``k_inst``: pusher
    installation factor override (sensitivity runs)."""
    E = S["engine"]
    eb = E.get("electrical_budget")
    load = (float(eb["continuous_base_W"]) + float(eb["research_payload_allowance_W"])) if eb else \
        float(E["electrical_load_continuous_W"])
    if elec_load_W is not None:
        load = float(elec_load_W)
    # shaft draw of the generator: DC load / (machine efficiency x power-electronics efficiency) (V1-02)
    eng = Engine(E, load, float(E["generator"]["efficiency"]) *
                 float(E["generator"].get("power_electronics_efficiency", 1.0)))
    P = S["propeller"]
    key = table_key or P["table_ref"]
    rows = ref_get(key)["rows_rpm_thrust_N_torque_Nm_power_W"]
    D = float(P["diameter"]) if table_key is None else float(P["alternatives"][table_key]["diameter"])
    ramp = P.get("k_inst_wot_ramp_speed")
    return eng, Prop(rows, D, float(P["k_wot"]), float(P["k_inst"]) if k_inst is None else float(k_inst), eng,
                     k_inst_wot_ramp=float(ramp) if ramp else None)


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
    # ground geometry (gear extended)
    LG = S["landing_gear"]
    gp_like = {"z_trunnion": LG["main"]["trunnion"][2], "z_axle": LG["main"]["axle_static"][2],
               "x_ng": LG["nose"]["axle_static"][0], "z_axle_nose": LG["nose"]["axle_static"][2],
               "nose_leg_length": LG["nose"]["leg_length"]}
    own_I = distributed_pitch_inertia(S, af)          # V4-05: pitch inertia of the take-off loading cases
    gcases = ground_cases(S, gp_like, None, own_I)
    pot = power_on_trim(S, wa, stab, tp["stabilator"], clm, prop, cases, gcases)
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
    ground = {"z_g": gr["z_g"], "x_mg": gr["x_mg"], "x_ng": gr["x_ng"], "x_cg_to": gc0["x"], "z_cg_to": gc0["z"],
              "z_t": float(S["propeller"]["hub"][2]), "x_thrust": float(S["propeller"]["plane_x"]),
              "x_ac_wb": stab["x_ac_wb"], "CL_ground_to": gr["CL_ground_to"], "CL_ground_ld": gr["CL_ground_ld"],
              "Cm0_to": stab["Cm0_wb"] - 0.25 * clm["dCL0_to"], "CL0": wa["CL_0"], "dCL0_to": clm["dCL0_to"],
              "CLa": wa["CL_alpha"], "mu": float(mis["rolling_friction"]), "t_rot": float(mis["rotation_time_s"]),
              "t_spin": float(mis.get("rotation_spin_up_s", 0.0)),
              "z_ac_wb": float(S["wing"]["planform"]["z_root"]),
              "static_attitude_deg": float(LG["rules"]["static_attitude_deg"]),
              "stab": {"S_h": tp["stabilator"]["S_exposed"], "x_ac": tp["stabilator"]["x_ac"],
                       "z_ac": float(tp["stabilator"]["z"]),
                       "eta": float(st_rules["eta_tail"]), "clt_max": float(st_rules["stabilator_clmax"])}}

    fuel = float(S["mass"]["fuel_kg"])
    empty = float(S["mass"]["empty_kg"])
    use_mission_cg = str(mis.get("stall_floor_cg", "design_mission")) == "design_mission"
    eb0 = electrical_budget(S, 0.0) if S["engine"].get("electrical_budget") else None

    def clmax_fn(cg_f, m_zf: float, f_max: float):
        """Trimmed clean CLmax of a loading as a function of the weight (N): CG with the fuel burning off (mission
        1.2 VS floors); None when the forward-CG rule is used instead (mission.stall_floor_cg)."""
        if not use_mission_cg:
            return None

        def f(W: float) -> float:
            mf = min(max(W / G - m_zf, 0.0), f_max)
            return trimmed_clmax_at(S, wa, stab, tp["stabilator"], cg_f(mf))
        return f

    def flight_for(clmax_f, pols=None, items=None, eng_prop=None, floor_peak=None):
        e_, p_ = eng_prop or (eng, prop)
        it_ = items or items_all
        return Flight(S, e_, p_, pols or polars, ground, {"Dq": it_["loiter"]["engine_cooling"], "k_leak": k_leak},
                      mission_clmax=clmax_f, floor_peak_W=floor_peak if floor_peak is not None else
                      (eb0["peak_hd59_W"] if eb0 else None))

    # design-mission loading (design payload, mass.cases[0]): CG and trimmed CLmax as the fuel burns off
    cg_mis, m_zf_mis = loading_cg_fn(S, 0)
    mission_clmax = clmax_fn(cg_mis, m_zf_mis, fuel)
    fl = flight_for(mission_clmax)
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
        fl = flight_for(mission_clmax)
    endurance_h = mis_r["t_air_s"] / 3600.0

    def polars_at(case: dict, S_drag: dict | None = None) -> tuple:
        """Trimmed polars (and drag items) at the CG of a loading case (``S_drag``: spec copy for the drag build-up)."""
        pol_, it_all = {}, {}
        for name_, cfg_ in CFG:
            it_ = drag_items(S_drag or S, af, V_ref, h_l, cfg_, ffr)
            pol_[name_] = trimmed_polar(name_, S, wa, cdp, k_trip, stab, sum(it_.values()) / float(S["wing"]["area"]),
                                        (case["x"], case["z"]), tp["stabilator"], clm, it_)
            it_all[name_] = it_
        return pol_, it_all
    # E180 growth mission (V1-02): growth turret extended in the loiter (ball of the growth-envelope diameter in the
    # drag build-up), its own peak load -> its own loiter rpm floor, E180 loading CG for trim and the 1.2 VS floor;
    # same MTOM and fuel (the research allowance is reduced by the turret mass difference). V3-08 (fix round 4): the
    # generator carries the E180 continuous load (base - HD59 average + E180 average + research allowance) on this
    # mission (shaft draw, descent rpm floor), not the HD59 load of the design mission
    e180 = None
    ci = next((i for i, c in enumerate(S["mass"]["cases"]) if c["name"].startswith("e180")), None)
    if ci is not None and eb0:
        Se = copy.deepcopy(S)
        Se["payload"]["turret"]["ball_diameter"] = float(Se["payload"]["turret"]["growth_envelope"]["diameter"])
        ce = mass_cases(S)[ci]
        pol_e, items_e = polars_at(ce, Se)
        cg_e, m_zf_e = loading_cg_fn(S, ci)
        eng_e, prop_e = make_propulsion(S, table_key, elec_load_W=eb0["continuous_e180_W"])
        fl_e = flight_for(clmax_fn(cg_e, m_zf_e, fuel), pol_e, items_e, (eng_e, prop_e), eb0["peak_e180_W"])
        mis_e = fl_e.solve_loiter_for_fuel(m0, fuel)
        e180 = {"flight": fl_e, "mission": mis_e, "case": ce["name"], "endurance_h": mis_e["t_air_s"] / 3600.0,
                "loiter_time_h": mis_e["t_loiter_s"] / 3600.0, "rpm_floor": fl_e.rpm_floor,
                "continuous_load_W": eng_e.elec_load_W, "descent_rpm": fl_e.descent_rpm(),
                "cd0_loiter": pol_e["loiter"].fit["cd0"], "turret_drag_Dq_m2": items_e["loiter"]["eo_ir_turret"] +
                items_e["loiter"].get("turret_bay_cavity", 0.0)}
    rng = {"range_m": float("nan")} if light else fl.solve_range(m0, fuel)
    perf = performance_block(S, fl, m0, fuel, mis_r, g_cases_m0)
    # 10 h mission closure (sizinglib MassModel): MTOW needed for the requirement, same aircraft scaled
    r10 = fl.solve_loiter_for_endurance(m0, float(mis["endurance_requirement_h"]) * 3600.0)
    payload = float(S["mass"]["payload_kg"])
    struct_names = ("wing", "tail", "shell", "chassis", "gear")
    af_mass = sum(i["mass_kg"] for i in S["mass"]["items"] if i["group"] in struct_names)
    fixed = empty - af_mass
    mm_design = SZ.MassModel(payload, fixed, mis_r["ff"], lambda m: af_mass / m0 * (m / m0) ** 0.0).solve(m0_guess=m0)
    mm_10 = SZ.MassModel(payload, fixed, r10["ff"], lambda m: af_mass / m0).solve(m0_guess=m0)
    pack = packaging(S, af, cases)
    fuel_cap = pack["fuel"]["available_m3"] / (1 + float(S["structures"]["fuel"]["expansion_fraction"])) * \
        float(S["engine"]["fuel"]["density_kg_per_m3"])
    # requirement decision (fix round 4): R-02b - the maximum payload (mission.payload_max_kg) on the same mission at
    # MTOM (fuel = MTOM - empty - maximum payload); its own loading CG (trim polars and 1.2 VS floors)
    p_max = float(mis["payload_max_kg"])
    ci_x = next((i for i, c in enumerate(S["mass"]["cases"]) if c["name"] == "mtow_max_payload_turret_retracted"), None)
    cg_x, m_zf_x = payload_loading_cg_fn(S, p_max)
    fuel_x = min(m0 - empty - p_max, fuel_cap)
    if ci_x is not None:
        pol_x, items_x = polars_at(mass_cases(S)[ci_x])
    else:
        pol_x, items_x = polars, items_all
    fl_x = flight_for(clmax_fn(cg_x, m_zf_x, fuel_x), pol_x, items_x)
    mis_x = fl_x.solve_loiter_for_fuel(empty + p_max + fuel_x, fuel_x)
    max_payload = {"payload_kg": p_max, "fuel_kg": fuel_x, "takeoff_mass_kg": empty + p_max + fuel_x,
                   "endurance_h": mis_x["t_air_s"] / 3600.0, "loiter_time_h": mis_x["t_loiter_s"] / 3600.0,
                   "requirement_h": float(mis["endurance_max_payload_requirement_h"]),
                   "case": S["mass"]["cases"][ci_x]["name"] if ci_x is not None else None, "mission_log": mis_x["log"]}

    def peak_draw_Wh(mr):
        """R-52: battery energy the E180 peak would draw over the loiter of a mission if it lasted the whole loiter
        (peak minus the generator DC output at the midpoint of every integration step)."""
        if not eb0:
            return 0.0
        return sum(max(eb0["peak_e180_W"] - l_["gen_W"], 0.0) * l_["dt_s"] / 3600.0
                   for l_ in mr["log"] if l_["kind"] == "loiter" and "gen_W" in l_)

    def supported_loiter_h(mr):
        """V3-09: loiter time (h) after which the E180 peak, drawn continuously, would have used the battery
        peak-support share (None = the whole loiter of that mission is covered)."""
        if not eb0:
            return None
        cum, t = 0.0, 0.0
        share = eb0["battery_peak_share_Wh"]
        for l_ in mr["log"]:
            if l_["kind"] != "loiter" or "gen_W" not in l_:
                continue
            d = max(eb0["peak_e180_W"] - l_["gen_W"], 0.0)
            e_ = d * l_["dt_s"] / 3600.0
            if cum + e_ >= share and d > 0:
                return (t + (share - cum) / d * 3600.0) / 3600.0
            cum += e_
            t += l_["dt_s"]
        return None

    # payload -> endurance at MTOM (fuel = MTOM - empty - payload, limited by the tank volume): one flight model per
    # payload loading (CG for the 1.2 VS floors). Design polars (trimmed at the design CG) for every point. V4-02 (fix
    # round 5): every payload carries at least the baseline EO/IR set (the lightest permitted loading; without it the
    # static margin falls below R-09, see payload_permitted_loadings); each row carries its static-margin range over its
    # fuel load. The E180 peak-support columns only for payloads that can carry the E180 set (V4-02).
    PS_ = S["payload"]
    e180_set_kg = float(PS_["growth_turret_mass_kg"]) + float(PS_["mission_computer_kg"]) + float(PS_["tray_harness_kg"])
    cbar_ = float(S["wing"]["mac"])
    sm_rule = float(S["aero"]["stability_rules"]["sm_min"])

    def sm_range(pl: float, fu_: float) -> list:
        f_, _ = payload_loading_cg_fn(S, pl)
        xs_ = (f_(0.0), f_(fu_))
        return [(stab["x_np"] - max(xs_)) / cbar_, (stab["x_np"] - min(xs_)) / cbar_]
    pe_cache = {}

    def payload_mission(pl: float) -> dict:
        key = round(pl, 6)
        if key in pe_cache:
            return pe_cache[key]
        fu = min(m0 - empty - pl, fuel_cap)
        m_to = empty + pl + fu
        if abs(pl - payload) < 1e-9 and abs(fu - fuel) < 1e-9:
            r = mis_r                                             # the design mission itself
        elif abs(pl - p_max) < 1e-9 and abs(fu - fuel_x) < 1e-9:
            r = mis_x
        else:
            cg_p, m_zf_p = payload_loading_cg_fn(S, pl)
            fl_p = flight_for(clmax_fn(cg_p, m_zf_p, fu))
            r = fl_p.solve_loiter_for_fuel(m_to, fu, loiter_pol="loiter" if pl > 1e-9 else "clean")
        smr = sm_range(pl, fu)
        e180_ok = pl >= e180_set_kg - 1e-9
        pe_cache[key] = {"payload_kg": pl, "fuel_kg": fu, "takeoff_mass_kg": m_to, "endurance_h": r["t_air_s"] / 3600.0,
                         "loiter_time_h": r["t_loiter_s"] / 3600.0, "fuel_volume_limited": bool(m0 - empty - pl > fuel_cap),
                         "turret": pl > 1e-9, "static_margin_range": smr, "permitted_loading": bool(smr[0] >= sm_rule - 1e-9),
                         "e180_set_fits": e180_ok,
                         "e180_peak_battery_draw_Wh": peak_draw_Wh(r) if e180_ok else None,
                         "e180_peak_supported_loiter_h": supported_loiter_h(r) if e180_ok else None}
        return pe_cache[key]
    # design-mission payload rule (requirement decision, fix round 4): the largest payload on the rule grid (0.5 kg,
    # rounded DOWN) that gives at least endurance_requirement_h + robustness_margin_h on the design mission at MTOM;
    # never above the maximum payload. Derived here from the closed aircraft; the spec value must equal it (--check)
    rule = mis["payload_design_rule"]
    E_tgt = float(mis["endurance_requirement_h"]) + float(rule["robustness_margin_h"])
    step = float(rule["step_kg"])
    p_d = None
    if not light:
        p = min(math.floor(payload / step + 1e-9) * step, p_max)
        if payload_mission(p)["endurance_h"] >= E_tgt:
            while p + step <= p_max + 1e-9 and payload_mission(p + step)["endurance_h"] >= E_tgt:
                p += step
        else:
            while p - step >= payload_baseline_kg(S) - 1e-9 and payload_mission(p)["endurance_h"] < E_tgt:
                p -= step
        p_d = round(p, 6)
    p_next = None if p_d is None or p_d + step > p_max + 1e-9 else round(p_d + step, 6)
    # V4-03 (fix round 5): headroom of the derived design payload to the rule threshold - endurance above the target, the
    # empty-mass growth that brings the design mission down to the target at fixed MTOM (every kg of empty mass is a kg
    # of fuel; the design loading, as R-56) and the payload at the target (linear between the grid points)
    head = {}
    if p_d is not None:
        E_d = payload_mission(p_d)["endurance_h"]
        head = {"endurance_margin_to_target_h": E_d - E_tgt}
        if abs(p_d - payload) < 1e-9:
            r_t = fl.solve_loiter_for_endurance(m0, E_tgt * 3600.0)
            head["empty_mass_headroom_kg"] = fuel - r_t["ff"] * m0
        if p_next is not None:
            E_n = payload_mission(p_next)["endurance_h"]
            p_star = p_d + step * (E_d - E_tgt) / max(E_d - E_n, 1e-9)
            head.update({"payload_at_target_kg_linear": p_star, "payload_headroom_kg": p_star - p_d})
    payload_rule = {"rule": f"largest payload on a {step} kg grid (rounded down) with endurance >= "
                            f"{E_tgt:.2f} h ({float(mis['endurance_requirement_h']):.2f} h + "
                            f"{float(rule['robustness_margin_h']):.2f} h robustness margin), at most the maximum payload",
                    "rule_tr": f"{_f(step, 1)} kg'lık ızgarada (aşağı yuvarlanmış), dayanımı ≥ {_f(E_tgt)} h "
                               f"({_f(float(mis['endurance_requirement_h']))} h + {_f(float(rule['robustness_margin_h']))} h "
                               "sağlamlık payı) olan en büyük faydalı yük; azami yükü aşmaz",
                    "target_h": E_tgt, "step_kg": step, "spec_kg": payload, "derived_kg": p_d,
                    "endurance_at_derived_h": None if p_d is None else payload_mission(p_d)["endurance_h"],
                    "next_step_kg": p_next,
                    "endurance_at_next_step_h": None if p_next is None else payload_mission(p_next)["endurance_h"],
                    "consistent": None if p_d is None else bool(abs(p_d - payload) < 1e-9)} | head
    pe = []
    p_base = payload_baseline_kg(S)
    p_break = m0 - empty - fuel_cap                       # below it the fuel volume limits the fuel (full tanks)
    if not light:
        pts = {p_base, 5.0, 7.5, 10.0, 12.5, 15.0, 17.5, payload, p_max}
        if p_base < p_break < p_max:                      # V4-06: the fuel-volume break and its 0.5 kg neighbours
            pts |= {round(p_break, 4), math.floor(p_break / 0.5) * 0.5, math.ceil(p_break / 0.5) * 0.5}
        if p_next is not None:
            pts.add(p_next)
        pe = [payload_mission(pl) for pl in sorted(p for p in pts if p_base - 1e-9 <= p <= p_max + 1e-9)]
    # V4-02 (fix round 5): loadings below the baseline set are not permitted (R-09): without any payload the static
    # margin is below the minimum over the whole fuel range (nose ballast at the turret mount that would restore it);
    # turret only: the largest fuel load that keeps R-09
    pay_it = {p_["name"]: p_ for p_ in S["mass"]["payload_items"]}
    x_lim = stab["x_np"] - sm_rule * cbar_
    f0, mz0 = payload_loading_cg_fn(S, 0.0)
    mx0 = f0(0.0) * mz0
    fx_ = float(S["mass"]["fuel_cg"][0])
    xb_ = float(pay_it["eo_ir_turret_hd59_mount"]["x"])
    ballast = max(max((mx0 + f_ * fx_ - x_lim * (mz0 + f_)) / (x_lim - xb_), 0.0) for f_ in (0.0, fuel_cap))
    m_t = float(pay_it["eo_ir_turret_hd59_mount"]["mass_kg"])
    mz_t, mx_t = mz0 + m_t, mx0 + m_t * xb_
    f_t = fuel_cap if fx_ <= x_lim else min(max((x_lim * mz_t - mx_t) / (fx_ - x_lim), 0.0), fuel_cap)
    permitted = {"static_margin_min_rule": sm_rule, "x_cg_aft_limit_m": x_lim,
                 "baseline_set_kg": p_base, "fuel_volume_break_payload_kg": p_break, "fuel_capacity_kg": fuel_cap,
                 "e180_set_kg": e180_set_kg,
                 "no_payload": {"static_margin_range": sm_range(0.0, fuel_cap), "permitted": False,
                                "nose_ballast_at_turret_mount_kg": ballast, "ballast_x_m": xb_},
                 "turret_only": {"static_margin_range_full_tanks": sm_range_items(S, stab, cbar_, m_t, xb_, fuel_cap),
                                 "fuel_max_for_R09_kg": f_t},
                 "rule_tr": "izin verilen en hafif yükleme temel EO/IR setidir (taret + görev bilgisayarı + tepsi); "
                            "yalnız taretle yakıt fuel_max_for_R09_kg ile sınırlıdır"}
    permitted["no_payload"]["permitted"] = bool(permitted["no_payload"]["static_margin_range"][0] >= sm_rule - 1e-9)
    # V1-03 / R-56 (fix round 4: both payload requirements): empty mass at which R-02 (design payload, 10 h) and R-02b
    # (maximum payload, 9.5 h) are met exactly at fixed MTOM (every kg of empty mass is a kg of fuel); the budget
    # (group ceilings) + reserve must stay below the smaller of the two
    fuel_10h = r10["ff"] * m0
    empty_at_r02 = empty + (fuel - fuel_10h)
    r95 = fl_x.solve_loiter_for_endurance(m0, max_payload["requirement_h"] * 3600.0)
    fuel_95 = r95["ff"] * m0
    empty_at_r02b = m0 - p_max - fuel_95
    bud = S["mass"].get("budget", {})
    bud_sum = sum(float(b["target_kg"]) for b in bud.values())
    bud_reserve = float(S["mass"].get("budget_rules", {}).get("reserve_kg", 0.0))
    lim = min(empty_at_r02, empty_at_r02b)
    budget_chk = {"fuel_for_10h_kg": fuel_10h, "air_time_h_at_fuel_for_10h": r10["t_air_s"] / 3600.0,
                  "empty_kg_at_R02_limit": empty_at_r02, "fuel_for_R02b_kg": fuel_95,
                  "air_time_h_at_fuel_for_R02b": r95["t_air_s"] / 3600.0, "empty_kg_at_R02b_limit": empty_at_r02b,
                  "empty_kg_limit": lim, "governing": "R-02" if empty_at_r02 <= empty_at_r02b else "R-02b",
                  "budget_sum_kg": bud_sum, "reserve_kg": bud_reserve, "margin_kg": lim - bud_reserve - bud_sum,
                  "margin_R02_kg": empty_at_r02 - bud_reserve - bud_sum,
                  "margin_R02b_kg": empty_at_r02b - bud_reserve - bud_sum,
                  "groups_over_ceiling": [g for g, b in bud.items()
                                          if empty_mass(S)["groups"].get(g, 0.0) > float(b["target_kg"]) + 1e-6]}
    VH_level = perf["0"]["V_max_m_s"]
    VC = float(S["structures"]["VC_eas"])
    VD = max(1.25 * VC, VH_level, float(S["structures"]["VD_min_eas"]))
    # V1-09: operating limits (CS-LUAS 1505, standards.yaml GEN-001): VNE = 0.9 VD, VNO = min(VC, 0.89 VNE); the FCS
    # envelope protection keeps the speed at or below VNO (level flight at full power would exceed VNE)
    OL = S["structures"].get("operating_limits", {})
    VNE = 0.9 * VD
    VNO = min(VC, 0.89 * VNE)
    fcs_lim = float(OL.get("fcs_speed_limit_eas", float("nan")))
    op_lim = {"VNE_eas": VNE, "VNO_eas": VNO, "fcs_speed_limit_eas": fcs_lim,
              "fcs_speed_limit_margin_m_s": VNO - fcs_lim, "V_max_level_sl_eas": VH_level,
              "V_max_level_3000m_eas": perf["3000"]["V_max_m_s"] * math.sqrt(AL.isa(h_l)["sigma"]),
              "fcs_protection_required": bool(VH_level > VNE)}
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
    tri = tail_root_interference(S, af)
    hinge = stab_hinge_moments(S)
    # V2-03: every control surface's hinge moment and actuator through its four-bar linkage; flap speed VF from
    # CS-LUAS.345(b) with the clean and take-off-flap stall speeds at MTOM
    VF = max(1.4 * perf["0"]["VS_clean_m_s"], 1.8 * perf["takeoff_sl_mtow"]["VS_TO_m_s"])
    ctl = control_hinge_moments(S, float(S["structures"]["VC_eas"]), float(S["structures"]["VD_eas"]), VF)
    spars = {"main": spar_depth_profile(S, af, S["wing"]["planform"]["main_spar_frac"]),
             "rear": spar_depth_profile(S, af, S["wing"]["planform"]["rear_spar_frac"])}
    sdt = spar_depth_targets(S["wing"]["planform"])
    loi3 = perf[str(int(h_l))]["loiter"]
    # R-32 / R-52 at the lowest generator output of each mission's loiter: the rpm falls as the fuel burns off, so the
    # loiter points at the start and at the END of the loiter (lightest weight) are evaluated explicitly
    def loiter_ends(fl_, mr):
        rows_ = [l_ for l_ in mr["log"] if l_["kind"] == "loiter"]
        if not rows_:
            return loi3, loi3, loi3["gen_W"], rows_
        a_ = fl_.best_loiter(rows_[0]["W_start_N"], h_l, "loiter", rpm_floor=True)
        b_ = fl_.best_loiter(mr["W_loiter_end"], h_l, "loiter", rpm_floor=True)
        return a_, b_, min([a_["gen_W"], b_["gen_W"]] + [l_["gen_W"] for l_ in rows_ if "gen_W" in l_]), rows_
    p_ls, p_le, gen_min, lrows = loiter_ends(fl, mis_r)
    elec = electrical_budget(S, gen_min)
    elec["generator_W_loiter_start_mtom"] = loi3["gen_W"]
    elec["generator_rpm_floor"] = fl.rpm_floor
    elec["power_electronics_efficiency"] = float(S["engine"]["generator"].get("power_electronics_efficiency", 1.0))
    elec["margin_peak_hd59_design_mission"] = gen_min / elec["peak_hd59_W"]
    elec["margin_peak_e180_at_design_mission_rpm"] = gen_min / elec["peak_e180_W"]

    # R-52 (V2-01, fix round 3): the battery energy the E180 peak would draw over the loiter if it lasted the whole
    # loiter (deficit below the generator DC output, midpoint of every integration step; peak_draw_Wh above) vs the
    # battery peak-support share
    elec["e180_peak_battery_draw_design_mission_Wh"] = peak_draw_Wh(mis_r)
    elec["e180_peak_deficit_max_design_mission_W"] = max(elec["peak_e180_W"] - gen_min, 0.0)
    draws = [elec["e180_peak_battery_draw_design_mission_Wh"]]
    if e180:
        e_ls, e_le, gen_e, _ = loiter_ends(e180["flight"], e180["mission"])
        elec.update({"generator_W_loiter_e180_mission": gen_e, "generator_rpm_floor_e180_mission": e180["rpm_floor"],
                     "margin_peak_e180_mission": gen_e / elec["peak_e180_W"],
                     "margin_continuous_e180_mission": gen_e / elec["continuous_e180_W"],
                     "e180_mission_endurance_h": e180["endurance_h"],
                     "e180_peak_battery_draw_e180_mission_Wh": peak_draw_Wh(e180["mission"]),
                     "e180_peak_deficit_max_e180_mission_W": max(elec["peak_e180_W"] - gen_e, 0.0)})
        draws.append(elec["e180_peak_battery_draw_e180_mission_Wh"])
        # R-32: the largest continuous load at the lowest loiter output of either mission (generator alone)
        elec["margin_continuous"] = min(gen_min, gen_e) / max(elec["continuous_hd59_W"], elec["continuous_e180_W"])
        e180["loiter_points"] = {"start": {k: e_ls.get(k) for k in ("V", "EAS", "CL", "rpm", "gen_W", "ff_kg_h")},
                                 "end": {k: e_le.get(k) for k in ("V", "EAS", "CL", "rpm", "gen_W", "ff_kg_h")}}
    # generator alone vs the E180 peak at the lowest design-mission loiter rpm (the v1.2 measure; information)
    elec["margin_peak"] = elec["margin_peak_e180_at_design_mission_rpm"]
    elec["peak_support_margin"] = elec["battery_peak_share_Wh"] / max(max(draws), 1e-3)
    elec["peak_support_rule"] = str(S["mission"].get("loiter_rpm_floor"))
    # V3-09 (fix round 4): the peak-support share is sized on the loiters R-52 checks (design mission, E180 mission).
    # Operating limit: with the E180 peak drawn continuously at the loiter rpm floor the share lasts at least
    # share / deficit cap hours of loiter (deficit at the cap all the time); a longer loiter (lighter payload, more
    # fuel) is covered only while the share lasts - the power management then raises the loiter rpm to the
    # generator-only E180 floor or the peak duty is limited. Supported loiter time = time at which the cumulative draw
    # reaches the share along each mission's loiter (None = the whole loiter is covered)
    cap_W = float(S["mission"].get("peak_support_deficit_cap_W", 0.0) or 0.0)
    elec["peak_support_guaranteed_loiter_h_at_cap"] = elec["battery_peak_share_Wh"] / cap_W if cap_W > 0 else None
    elec["e180_peak_battery_draw_max_payload_mission_Wh"] = peak_draw_Wh(mis_x)
    elec["generator_rpm_e180_peak_generator_only"] = generator_rpm_for_dc(S, elec["peak_e180_W"])
    for r_ in pe:
        r_["e180_peak_share_exceeded"] = None if r_["e180_peak_battery_draw_Wh"] is None else \
            bool(r_["e180_peak_battery_draw_Wh"] > elec["battery_peak_share_Wh"])
    elec["payload_endurance_peak_support"] = [
        {"payload_kg": r_["payload_kg"], "loiter_time_h": r_["loiter_time_h"], "draw_Wh": r_["e180_peak_battery_draw_Wh"],
         "share_exceeded": r_["e180_peak_share_exceeded"], "e180_set_fits": r_["e180_set_fits"],
         "e180_peak_supported_loiter_h": r_["e180_peak_supported_loiter_h"]} for r_ in pe]
    elec["e180_peak_supported_loiter_h_design_mission"] = supported_loiter_h(mis_r)
    keys_lp = ("W_N", "V", "EAS", "CL", "rpm", "gen_W", "ff_kg_h", "P_total", "power_fraction")
    mission_loiter = {"start": {k: (p_ls.get(k) if k != "W_N" else lrows[0]["W_start_N"]) for k in keys_lp} if lrows
                      else None,
                      "end": {k: (p_le.get(k) if k != "W_N" else mis_r["W_loiter_end"]) for k in keys_lp} if lrows
                      else None}
    sens_d, low_load = {}, {}
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
        # V3-06 (fix round 4): k_inst is an estimate (bottom of the research-phase judgement range 0.93-0.97 for a pusher
        # behind a fuselage; a bluff-base installation is not bounded by that range): endurance with other values
        for k_ in (0.90, 0.95, 0.97):
            fl_k = flight_for(mission_clmax, eng_prop=make_propulsion(S, table_key, k_inst=k_))
            sens_d[f"k_inst_{k_:.2f}".replace(".", "p")] = fl_k.solve_loiter_for_fuel(m0, fuel)["t_air_s"] / 3600
        # V3-07: below 20 % power (descent about 5-7 %, reserve loiter about 18 %) the BSFC line is extrapolated; the
        # Willans-line extrapolation (fuel flow linear in power through the two lowest points) is a more conservative
        # extrapolation, not a bound (no map data below 20 % power; V4-08)
        eng.low_load_model = "willans"
        r_w = fl.solve_loiter_for_fuel(m0, fuel)
        eng.low_load_model = "bsfc_linear"
        sens_d["low_load_fuel_flow_willans_line"] = r_w["t_air_s"] / 3600
        low_load = {"endurance_h_bsfc_line": endurance_h, "endurance_h_willans_line": r_w["t_air_s"] / 3600}
        for k_ in ("descent", "reserve"):
            for nm_, r_m in (("bsfc_line", mis_r), ("willans_line", r_w)):
                rows_ = [r_ for r_ in r_m["log"] if r_["kind"] == k_]
                low_load[f"{k_}_fuel_kg_{nm_}"] = sum(r_["fuel_kg"] for r_ in rows_)
                low_load[f"{k_}_power_fraction_range_{nm_}"] = [min(r_.get("power_fraction", 1.0) for r_ in rows_),
                                                                max(r_.get("power_fraction", 0.0) for r_ in rows_)]
        low_load["lowest_bsfc_point_power_fraction"] = float(eng.bsfc_pts[0, 0])
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
                 "massmodel_design_mission": mm_design, "massmodel_10h_mission": mm_10, "budget_check": budget_chk,
                 "wing_structure": wing_m, "tail_structure": tail_m, "body_shell": shell},
        "ground": gr, "turret": tur, "packaging": pack,
        "loads": {"vn": vn, "VD_m_s": VD, "VC_m_s": VC, "VA_m_s": min(v["VA"] for v in vn.values()),
                  "gust_matrix": gm, "n_limit_wing_design": n_wing, "cl_alpha_used": cla_cfg,
                  "operating_limits": op_lim},
        "performance": perf | {"endurance_h": endurance_h, "loiter_time_h": mis_r["t_loiter_s"] / 3600,
                               "range_km": rng["range_m"] / 1000, "mission_fuel_fraction": mis_r["ff"],
                               "mission_log": mis_r["log"], "payload_endurance": pe,
                               "mtow_for_10h_mission_kg": mm_10["mtow"], "loiter_3000m": loi3,
                               "mission_loiter_points": mission_loiter,
                               "payload_design_rule": payload_rule, "payload_permitted_loadings": permitted,
                               "max_payload_mission": {k: v for k, v in max_payload.items() if k != "mission_log"},
                               "max_payload_mission_log": max_payload["mission_log"],
                               "e180_growth_mission": ({k: v for k, v in e180.items() if k not in ("flight", "mission")}
                                                       | {"mission_log": e180["mission"]["log"]}) if e180 else None},
        "constraint_diagram": cdiag, "sensitivities_endurance_h": sens_d, "low_load_bsfc_bound": low_load,
        "electrical": elec,
        "propeller": {"table": table_key or S["propeller"]["table_ref"], "static_wot": perf["static_wot"],
                      "static_tip_speed_m_s": perf["static_tip_speed_m_s"], "clearances": pcl},
        "tail_roots": roots, "tail_root_interference": tri, "stabilator_hinge": hinge, "control_hinges": ctl,
        "spar_depth": {"main": {k: v for k, v in spars["main"].items() if k != "rows"},
                       "rear": {k: v for k, v in spars["rear"].items() if k != "rows"},
                       "required_main_m": sdt["main_required"], "required_rear_m": sdt["rear_required"],
                       "junction_thickness_m": sdt["t_j"], "joint_section_rear_depth_m": sdt["rear_joint_depth"],
                       "rows_main": spars["main"]["rows"], "rows_rear": spars["rear"]["rows"]},
    }
    res["_flight"] = fl                     # the solved mission model (tests); main() drops it before writing
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
    # V4-05 (fix round 5): effect of the pitch inertia on the governing take-off (the v1.5 quasi-static law without it,
    # and a faster 0.1 s spin-up)
    to_ = out["takeoff_sl_mtow"]
    gc_ = next(c for c in g_cases_m0 if c["name"] == to_["case"])
    if gc_.get("I_yy") is not None:
        q0 = fl.takeoff(m0, 0.0, (gc_["x"], gc_["z"]))
        q1 = fl.takeoff(m0, 0.0, (gc_["x"], gc_["z"]), I_yy=gc_["I_yy"], t_spin=0.1)
        out["takeoff_pitch_inertia_effect"] = {
            "case": to_["case"], "I_yy_kg_m2": gc_["I_yy"], "spin_up_time_s": fl.ground.get("t_spin", 0.0),
            "V_R_m_s": to_["V_R_m_s"], "ground_roll_m": to_["ground_roll_m"], "V_lof_m_s": to_["V_lof_m_s"],
            "without_pitch_inertia": {k: q0[k] for k in ("V_R_m_s", "ground_roll_m", "V_lof_m_s")},
            "spin_up_0p1s": {k: q1[k] for k in ("V_R_m_s", "ground_roll_m", "V_lof_m_s")} |
            {"download_margin_min_N": q1["rotation"]["download_margin_min_N"]},
            "delta_V_R_m_s": to_["V_R_m_s"] - q0["V_R_m_s"], "delta_ground_roll_m": to_["ground_roll_m"] -
            q0["ground_roll_m"]}
    m_land = m0 - fuel * 0.88
    out["landing_sl_mtow"] = fl.landing(m0)
    out["landing_sl_mtow_takeoff_flap"] = fl.landing(m0, flap="to")
    # V1-10: largest landing mass meeting the factored field rule (300 m / 1.5) flapless; heavier (early return) ->
    # unfactored on the 300 m runway (R-55) or hold to burn fuel down to this mass (no fuel dump)
    f_max = float(S["mission"]["field_roll_max"])
    lo_, hi_ = m_land, m0
    if fl.landing(m0)["ground_roll_m"] <= f_max:
        lo_ = m0
    else:
        for _ in range(30):
            mid = 0.5 * (lo_ + hi_)
            lo_, hi_ = (mid, hi_) if fl.landing(mid)["ground_roll_m"] <= f_max else (lo_, mid)
    out["landing_max_mass_factored_field_kg"] = lo_
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
R60_CONSISTENCY_TOL = 1e-6        # N m / N: moment residual about C and lift-off download continuity (R-60 gate)


def r60_metric(to: dict) -> float:
    """R-60 metric (V4-01, fix round 5): the smallest download margin (maximum - needed, UNCLIPPED: negative when the
    stabilator cannot give the download the rotation law needs) of every rotation step of every MTOM loading case. The
    rotation law must also hold: a moment residual about the main-wheel contact or a lift-off download continuity error
    above R60_CONSISTENCY_TOL makes the metric negative (minus the larger error), and so does a rate-limited step whose
    margin was not negative."""
    rots = [c["rotation"] for c in to["cases"].values()]
    margin = min(r["download_margin_min_N"] for r in rots)
    err = max(max(r["moment_residual_max_Nm"], abs(r["download_continuity_at_lof_N"])) for r in rots)
    if err > R60_CONSISTENCY_TOL:
        margin = min(margin, -err)
    if any(r["rate_limited_steps"] > 0 for r in rots):
        margin = min(margin, -R60_CONSISTENCY_TOL)
    return margin


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
        # requirement decision (fix round 4): R-02 at the design-mission payload, R-02b at the maximum payload, R-03 the
        # largest payload of the MTOM loading cases that every check (CG, stability, take-off, R-02b) is run with
        "payload_design_kg": m["payload_kg"], "endurance_max_payload_h": p["max_payload_mission"]["endurance_h"],
        "payload_max_checked_kg": max(c["payload_kg"] for c in m["cases"] if abs(c["m"] - m["mtow_kg"]) < 0.05),
        "ceiling_service_m": p["ceiling_service_m"], "roc_sl_m_s": p["0"]["RoC_max_m_s"],
        "roc_3000m_m_s": p["3000"]["RoC_max_m_s"],
        "climb_time_to_loiter_altitude_min": (sum(c_["dt_s"] for c_ in climb) / 60.0) if climb else None,
        "takeoff_ground_roll_m": to["ground_roll_m"], "takeoff_distance_15m_m": to["distance_15m_m"],
        "takeoff_rotation_speed_m_s": to["V_R_m_s"], "takeoff_liftoff_speed_m_s": to["V_lof_m_s"],
        "takeoff_main_gear_load_at_rotation_N": min(c["main_gear_load_at_VR_N"] for c in to["cases"].values()),
        # V3-03 (fix round 4): rotation law - download needed for the moment balance about the main-wheel contact vs the
        # maximum download, every rotation step of every MTOM case; download continuity with the trim at lift-off.
        # V4-01 (fix round 5): the margin is the UNCLIPPED one and the rotation-law consistency gates it (r60_metric)
        "takeoff_rotation_download_margin_min_N": r60_metric(to),
        "takeoff_rotation_moment_residual_max_Nm": max(c["rotation"]["moment_residual_max_Nm"]
                                                       for c in to["cases"].values()),
        "takeoff_rotation_rate_limited_steps": float(sum(c["rotation"]["rate_limited_steps"]
                                                         for c in to["cases"].values())),
        "takeoff_liftoff_trim_continuity_N": max(abs(c["rotation"]["download_continuity_at_lof_N"])
                                                 for c in to["cases"].values()),
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
        "tail_root_max_gap_m": rt["tail_root_max_gap_m"],
        "stab_hinge_rated_margin_max": hm["rated_margin_max_hinge_moment"],
        "prop_guard_ventral_margin_m": pcl["ventral_guard_margin_over_tip_m"],
        "prop_plane_behind_cowl_over_D": pcl["plane_behind_cowl_te_over_D"],
        "prop_clear_wingtip_on_ground_m": g["prop_clear_wingtip_on_ground_m"],
        "generator_peak_margin_loiter": R["electrical"]["margin_peak"],
        "e180_peak_support_margin": R["electrical"]["peak_support_margin"],
        "control_actuator_rated_margin_min": R["control_hinges"]["summary"]["rated_margin_min_wing_fin"],
        "control_actuator_peak_margin_min": R["control_hinges"]["summary"]["peak_margin_min_wing_fin"],
        "control_linkage_travel_margin_deg": R["control_hinges"]["summary"]["travel_margin_min_deg"],
        "tail_root_interferences": float(R["tail_root_interference"]["n_conflicts"]),
        "fcs_speed_limit_margin_m_s": R["loads"]["operating_limits"]["fcs_speed_limit_margin_m_s"],
        "landing_ground_roll_mtow_m": p["landing_sl_mtow"]["ground_roll_m"],
        "mass_budget_margin_kg": m["budget_check"]["margin_kg"],
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
    ("tail.surfaces.stabilator.params.x_le_root", 0.004), ("tail.surfaces.stabilator.params.pivot_mac_fraction", 0.003),
    ("tail.surfaces.stabilator.controls.ac_mac_fraction_range", 0.003), ("tail.surfaces.ventral.params.sweep_le_deg", 0.3),
    ("payload.research_allowance_design_kg", 1e-4), ("mass.cases", 2e-4),
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
    # requirement decision (fix round 4): the design-mission payload of the spec is the value the payload-endurance
    # rule gives for the closed aircraft (largest payload on the rule grid with >= R-02 + robustness margin)
    pr_ = R["performance"].get("payload_design_rule", {})
    n[0] += 1
    if pr_.get("derived_kg") is None:
        bad.append({"path": "mission.payload_design_kg", "spec": S["mission"]["payload_design_kg"], "computed": None,
                    "issue": "payload-endurance rule not evaluated"})
    elif abs(float(S["mission"]["payload_design_kg"]) - float(pr_["derived_kg"])) > 1e-6:
        bad.append({"path": "mission.payload_design_kg", "spec": S["mission"]["payload_design_kg"],
                    "computed": pr_["derived_kg"], "issue": "differs from the payload-endurance rule"})
    budget = S["mass"].get("budget", {})
    groups = empty_mass(S)["groups"]
    for gname, b in budget.items():
        n[0] += 1
        est, tgt = groups.get(gname, 0.0), float(b["target_kg"])
        if est > tgt + 1e-6:                                   # V1-03: one-sided ceiling
            bad.append({"path": f"mass.budget.{gname}", "spec": tgt, "computed": py(est), "issue": "above the ceiling"})
        elif tgt - est > float(b["tol_kg"]):
            bad.append({"path": f"mass.budget.{gname}", "spec": tgt, "computed": py(est),
                        "issue": f"more than {b['tol_kg']} kg below the ceiling (stale allocation)"})
    if verbose:
        for b in bad[:40]:
            print(f"  [derived] {b['path']}: spec {b['spec']} vs computed {b['computed']} ({b['issue']})")
    hist = D.get("_closure_history", [])
    n[0] += 1
    if not (hist and hist[-1].get("converged")):
        bad.append({"path": "closure", "spec": None, "computed": len(hist), "issue": "design closure not converged"})
    return {"n_compared": n[0], "n_bad": len(bad), "bad": bad, "closure_history": hist}


PERF_TOL = {"endurance_h": 0.05, "endurance_max_payload_h": 0.05, "loiter_time_h": 0.05, "range_km": 8.0, "ceiling_service_m": 60.0, "roc_sl_m_s": 0.05,
            "roc_3000m_m_s": 0.05, "takeoff_ground_roll_m": 3.0, "landing_ground_roll_m": 3.0, "vs_clean_sl_mtow_m_s": 0.05,
            "v_max_sl_m_s": 0.3, "loiter_tas_3000m_m_s": 0.2, "loiter_eas_m_s": 0.2, "loiter_fuel_flow_kg_h": 0.02,
            "takeoff_distance_15m_m": 5.0, "climb_time_to_loiter_altitude_min": 0.3, "mtow_for_10h_mission_kg": 0.5,
            "loiter_power_W": 40.0, "loiter_rpm": 30.0, "takeoff_rotation_speed_m_s": 0.1,
            "takeoff_liftoff_speed_m_s": 0.1, "takeoff_main_gear_load_at_rotation_N": 8.0, "prop_clear_min_925a_m": 0.002,
            "prop_clear_radial_m": 0.003, "prop_clear_longitudinal_m": 0.002, "generator_margin_loiter": 0.01,
            "stab_trim_cl_local_max": 0.006, "stab_hinge_peak_margin": 0.02,
            "mission_loiter_start_tas_m_s": 0.2, "mission_loiter_start_eas_m_s": 0.2, "mission_loiter_start_rpm": 30.0,
            "mission_loiter_start_fuel_flow_kg_h": 0.02, "mission_loiter_start_generator_W": 4.0,
            "mission_loiter_end_tas_m_s": 0.2, "mission_loiter_end_eas_m_s": 0.2, "mission_loiter_end_rpm": 30.0,
            "mission_loiter_end_fuel_flow_kg_h": 0.02, "mission_loiter_end_generator_W": 4.0}


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
                              "loiter_eas_m_s", "climb_time_to_loiter_altitude_min", "endurance_max_payload_h")}
    perf["loiter_fuel_flow_kg_h"] = p["loiter_3000m"]["ff_kg_h"]
    perf["loiter_power_W"] = p["loiter_3000m"]["P_total"]
    perf["loiter_rpm"] = p["loiter_3000m"]["rpm"]
    perf["mtow_for_10h_mission_kg"] = p["mtow_for_10h_mission_kg"]
    # V1-08: the loiter_* keys above are the MTOM point at 3000 m; the design mission's own loiter start and end points
    # (after climb and the outbound transit, and before the return transit) are carried separately
    for end in ("start", "end"):
        lp = p["mission_loiter_points"][end]
        perf[f"mission_loiter_{end}_tas_m_s"] = lp["V"]
        perf[f"mission_loiter_{end}_eas_m_s"] = lp["EAS"]
        perf[f"mission_loiter_{end}_rpm"] = lp["rpm"]
        perf[f"mission_loiter_{end}_fuel_flow_kg_h"] = lp["ff_kg_h"]
        perf[f"mission_loiter_{end}_generator_W"] = lp["gen_W"]
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
    ax.annotate("HANÇER", (c["ws_design_Pa"] / G, c["pw_prop_absorbed_wot_climb"]), xytext=(-64, 7),
                textcoords="offset points", color="#B91C1C", fontweight="bold",
                bbox=dict(boxstyle="round,pad=0.15", fc="white", ec="none", alpha=0.85))
    ax.axvline(c["ws_design_Pa"] / G, color="#B91C1C", lw=0.8, alpha=0.5,
               label=f"tasarım kanat yüklemesi {c['ws_design_Pa'] / G:.1f} kg/m²".replace(".", ","))
    p0 = R["performance"]
    roc_d = f"{p0['0']['RoC_max_m_s']:.2f}".replace(".", ",")
    # V4-06: the footnote sits below the axes (no data lines cross it)
    fig.text(0.01, 0.01, ("Eğriler basitleştirilmiş boyutlandırma denklemleridir (sabit η, temiz polar).\n"
                          "Ayrıntılı model (pervane haritası, tam gazda soğutma sürüklemesi, aşağı itki): "
                          f"DS tırmanma {roc_d} m/s, kalkış koşusu {p0['takeoff_sl_mtow']['ground_roll_m']:.0f} m."),
             ha="left", va="bottom", fontsize=6.5, color="#4B5563")
    ax.set_xlabel("Kanat yüklemesi W/S (kg/m²)")
    ax.set_ylabel("Güç yüklemesi P/W (W/N)")
    ax.set_ylim(0, 22)
    ax.set_xlim(ws[0], ws[-1])
    ax.legend(fontsize=7, loc="upper left", ncol=2)
    ax.set_title("YK-250 HANÇER — kısıt diyagramı (MTOM, ISA)")
    fig.tight_layout(rect=(0, 0.06, 1, 1))
    fig.savefig(path)
    plt.close(fig)


def fig_polars(S: dict, R: dict, path: Path) -> None:
    """Trimmed polars of the three configurations (design CG), each ending at its trimmed CLmax. V3-11: the gear-down
    polar is the flaps-up one (the take-off/landing flap increments are added in the field-performance models), and the
    marked CL band is the design mission's own loiter (start -> end, turret extended), not an MTOM point."""
    plt = _plt()
    fig, (a1, a2) = plt.subplots(1, 2, figsize=(9.0, 3.9))
    lab = {"clean": "temiz (takım ve taret içeride)", "loiter": "bekleme (taret dışarıda)",
           "gear_down": "takım açık, flapsız (kalkış/iniş flap artımı ayrıca)"}
    col = {"clean": "#1F4E79", "loiter": "#C2410C", "gear_down": "#15803D"}
    for k, p in R["aero"]["polars"].items():
        CL, CD = np.asarray(p["table"]["CL"]), np.asarray(p["table"]["CD"])
        m = (CL > 0.1) & (CL <= float(p["fit"].get("CL_max_trimmed", 1.6)) + 1e-9)
        a1.plot(CD[m], CL[m], lw=1.5, color=col.get(k), label=lab.get(k, k))
        a2.plot(CL[m], CL[m] / CD[m], lw=1.5, color=col.get(k), label=lab.get(k, k))
    a1.set_xlabel("CD (trimli, S_ref; bütün sürükleme kalemleri)")
    a1.set_ylabel("CL")
    a2.set_xlabel("CL")
    a2.set_ylabel("L/D")
    ml = R["performance"].get("mission_loiter_points") or {}
    if ml.get("start") and ml.get("end"):
        c0, c1 = sorted((ml["end"]["CL"], ml["start"]["CL"]))
        a2.axvspan(c0, c1, color="#C2410C", alpha=0.12, lw=0)
        a2.text(0.5 * (c0 + c1), 0.04, f"görev beklemesi\nCL {_f(ml['start']['CL'], 2)} → {_f(ml['end']['CL'], 2)}",
                transform=a2.get_xaxis_transform(), ha="center", va="bottom", fontsize=7, color="#9A3412")
    h_, l_ = a1.get_legend_handles_labels()                 # V4-06: legend below the panels (clear of every curve)
    fig.legend(h_, l_, fontsize=7.5, loc="lower center", ncol=3, frameon=False)
    fig.suptitle("Trimli sürükleme polarları (tasarım AM'si; kanat profil sürüklemesi geçiş x/c 0,075'e zorlanmış × 1,15; "
                 "eğriler trimli CLmaks'ta biter)", fontsize=8.5)
    fig.tight_layout(rect=(0, 0.07, 1, 1))
    fig.savefig(path)
    plt.close(fig)


CASE_TR = {"mtow_design_payload_turret_retracted": "MTOM, görev yükü", "mtow_design_payload_turret_extended":
           "MTOM, görev yükü, taret dışarıda", "mtow_max_payload_turret_retracted": "MTOM, azami yük",
           "mtow_max_payload_turret_extended": "MTOM, azami yük, taret dışarıda",
           "full_fuel_baseline_sensors_only": "dolu depolar, temel sensör seti",
           "zero_fuel_design_payload": "yakıtsız, görev yükü", "zero_fuel_max_payload": "yakıtsız, azami yük",
           "reserve_fuel_design_payload": "yedek yakıt, görev yükü",
           "minimum_flying_turret_only": "asgari uçuş (yalnız taret)", "e180_growth_turret_full_fuel": "E180 taret, tam yakıt"}
CASE_MARK = ("o", "s", "D", "^", "v", "P", "X", "<", ">", "h", "*", "p")


def fig_cg(S: dict, R: dict, path: Path) -> None:
    """CG envelope (flight, gear up). V3-11: every loading case has its own marker shape; markers are drawn from the
    largest to the smallest so that coincident cases stay visible; the neutral-point / aft-limit labels and the
    main-gear note sit outside the data."""
    plt = _plt()
    st = R["stability"]
    cb, le = float(S["wing"]["mac"]), float(S["wing"]["mac_le_x"])
    pct = lambda x: (x - le) / cb * 100                                  # noqa: E731
    fig, ax = plt.subplots(figsize=(7.6, 4.6))
    cases = R["mass"]["cases"]
    xs = [pct(c["x"]) for c in cases]
    # V4-06: cases that coincide on the plot (within 0.6 % MAC and 1.5 kg) are drawn as concentric open markers, the
    # first of the cluster largest and underneath, each 4.5 pt smaller than the one before (all stay visible)
    rank = [0] * len(cases)
    size_n = [1] * len(cases)
    for i, c in enumerate(cases):
        mates = [j for j, d in enumerate(cases) if abs(pct(d["x"]) - pct(c["x"])) < 0.6 and abs(d["m"] - c["m"]) < 1.5]
        rank[i] = mates.index(i)
        size_n[i] = len(mates)
    for i, c in enumerate(cases):
        ms = 6.0 + 4.5 * (size_n[i] - 1 - rank[i])
        ax.plot(pct(c["x"]), c["m"], CASE_MARK[i % len(CASE_MARK)], ms=ms, mfc="none", mec=f"C{i % 10}", mew=1.5,
                color=f"C{i % 10}", label=CASE_TR.get(c["name"], c["name"]), zorder=3 + rank[i])
    np_ = pct(st["x_np"])
    smin = float(S["aero"]["stability_rules"]["sm_min"]) * 100
    ax.axvline(np_, color="#B91C1C", lw=1.5)
    ax.axvline(np_ - smin, color="#B91C1C", ls="--", lw=1.1)
    ms_ = sorted(c["m"] for c in cases)
    y_txt = 0.5 * (ms_[0] + ms_[-1])                     # labels in the middle band, right of their lines
    ax.text(np_ + 0.3, y_txt, f"nötr nokta %{np_:.1f}".replace(".", ","), color="#B91C1C", fontsize=8, rotation=90,
            va="center")
    ax.text(np_ - smin + 0.3, y_txt, f"arka sınır (SM %{smin:.0f})", color="#B91C1C", fontsize=8, rotation=90,
            va="center")
    ax.set_xlim(min(xs) - 7, np_ + 4)
    ax.set_ylim(min(c["m"] for c in cases) - 4, max(c["m"] for c in cases) + 4)
    ax.set_xlabel(f"Ağırlık merkezi (% referans OAK; OAK ön kenarı x = {le:.3f} m)".replace(".", ","))
    ax.set_ylabel("Kütle (kg)")
    leg = ax.legend(fontsize=6.5, loc="center left", bbox_to_anchor=(1.01, 0.5), frameon=False)
    for h_ in leg.legend_handles:                         # uniform legend symbols (the plot sizes encode clusters)
        h_.set_markersize(7.0)
    ax.set_title("Ağırlık merkezi zarfı ve statik marj (uçuş, takım içeride)", fontsize=9)
    fig.text(0.01, 0.01, f"Ana takım dingili %{pct(R['ground']['x_mg']):.0f} OAK'ta (zemin; eksen dışında). "
             "Çakışan durumlar iç içe halkalar olarak çizilir (ilki en büyük ve en altta).", fontsize=7, color="#374151")
    fig.tight_layout(rect=(0, 0.04, 1, 1))
    fig.savefig(path)
    plt.close(fig)


def fig_payload_endurance(S: dict, R: dict, path: Path) -> None:
    """Payload-endurance at MTOM (requirement decision, fix round 4): design-mission payload from the rule (largest
    payload on the 0.5 kg grid with >= 10.25 h), R-02 (10 h) and R-02b (maximum payload, 9.5 h); V3-11: the fuel-volume
    limited range is one shaded band. V4-02/V4-06 (fix round 5): the sweep starts at the lightest permitted loading (the
    baseline EO/IR set; below it R-09 is not met, shaded) and includes the fuel-volume break and its 0.5 kg neighbours."""
    plt = _plt()
    p = R["performance"]
    pe = p["payload_endurance"]
    mis = S["mission"]
    pl_ = p.get("payload_permitted_loadings", {})
    fig, ax = plt.subplots(figsize=(6.8, 4.1))
    x = [r["payload_kg"] for r in pe]
    y = [r["endurance_h"] for r in pe]
    p_b = float(pl_.get("baseline_set_kg", x[0]))
    y_lo = min(min(y), p["max_payload_mission"]["requirement_h"]) - 0.7
    ax.axvspan(-0.5, p_b, color="#FCA5A5", alpha=0.25, lw=0)
    sm0 = (pl_.get("no_payload") or {}).get("static_margin_range")
    ax.text(0.25 * p_b - 0.3, y_lo + 0.12, "izin verilen\nyükleme değil\n(temel EO/IR\nseti yok:\n"
            + (f"SM %{_f(100 * sm0[0], 1)} < %{_f(100 * float(pl_['static_margin_min_rule']), 0)})" if sm0 else "SM < R-09)"),
            fontsize=6.5, color="#991B1B", va="bottom")
    x_lim = float(pl_.get("fuel_volume_break_payload_kg", float(S["mass"]["mtow_kg"]) - float(S["mass"]["empty_kg"]) -
                          R["mass"]["fuel_capacity_kg"]))
    if x_lim > p_b:
        ax.axvspan(p_b, x_lim, color="#9CA3AF", alpha=0.18, lw=0)
        ax.text(p_b + 0.3, y_lo + 0.12, f"yakıt hacmi sınırı (hücreler dolu,\n{_f(R['mass']['fuel_capacity_kg'], 2)} kg; "
                f"yük < {_f(x_lim, 2)} kg)", fontsize=7, color="#374151", va="bottom")
    ax.plot(x, y, "-", color="#1F4E79", lw=1.6, zorder=3)
    ax.plot(x, y, "o", color="#1F4E79", ms=3.5, zorder=4)
    req, rob = float(mis["endurance_requirement_h"]), float(mis["payload_design_rule"]["robustness_margin_h"])
    ax.axhline(req, color="#B91C1C", ls="--", lw=1.0, label=f"R-02: {_f(req, 1)} h (görev yüküyle)")
    ax.axhline(req + rob, color="#B91C1C", ls=":", lw=1.0, label=f"görev yükü kuralı: {_f(req + rob, 2)} h")
    pr = p.get("payload_design_rule", {})
    pd_ = float(pr.get("derived_kg") or S["mass"]["payload_kg"])
    mx = p["max_payload_mission"]
    e_d = pr.get("endurance_at_derived_h") or p["endurance_h"]
    ax.plot([pd_], [e_d], "o", ms=8, color="#B91C1C", zorder=5)
    ax.annotate(f"görev yükü {_f(pd_, 1)} kg\n{_f(e_d, 2)} h", (pd_, e_d), xytext=(-12, 40), textcoords="offset points",
                fontsize=7.5, color="#B91C1C", arrowprops=dict(arrowstyle="-", lw=0.6, color="#B91C1C"))
    ax.plot([mx["payload_kg"]], [mx["endurance_h"]], "s", ms=7, color="#7C3AED", zorder=5)
    ax.plot([mx["payload_kg"] - 0.6, mx["payload_kg"] + 0.6], [mx["requirement_h"]] * 2, color="#7C3AED", lw=2.0)
    ax.annotate(f"azami yük {_f(mx['payload_kg'], 0)} kg: {_f(mx['endurance_h'], 2)} h\n(R-02b ≥ {_f(mx['requirement_h'], 1)} h)",
                (mx["payload_kg"], mx["endurance_h"]), xytext=(-150, -24), textcoords="offset points", fontsize=7.5,
                color="#7C3AED", arrowprops=dict(arrowstyle="-", lw=0.6, color="#7C3AED"))
    ax.set_xlim(-0.5, mx["payload_kg"] + 1.0)
    ax.set_ylim(y_lo, max(y) + 0.5)
    ax.set_xlabel("Faydalı yük (kg)")
    ax.set_ylabel("Dayanım (h, tasarım görevi)")
    ax.legend(fontsize=7, loc="upper right")
    ax.set_title("Faydalı yük – dayanım (MTOM 149,9 kg; yakıt = MTOM − boş − yük, hacimle sınırlı)", fontsize=9)
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
    _n = lambda x, d: f"{x:.{d}f}".replace(".", ",")                       # noqa: E731  Turkish decimal comma
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
    dim((-b2, -L_all), (b2, -L_all), -0.30, f"Kanat açıklığı {_n(2 * b2, 2)} m")
    dim((-b2, 0.0), (-b2, -L_all), -0.35, f"Toplam boy {_n(L_all, 2)} m", horiz=False)
    xj = float(W["sections"][0]["x_le"]) + 0.5 * float(W["sections"][0]["chord"])
    dim((-yj, -xj - 0.55), (yj, -xj - 0.55), 0.0, f"Orta kesit {_n(2 * yj, 2)} m")
    dim((yj, -float(W["mac_le_x"]) - 1.4), (b2, -float(W["mac_le_x"]) - 1.4), 0.0,
        f"Dış panel {_n(b2 - yj, 2)} m")
    # front view
    dim((-b2, z_top + v_front), (-b2, gz + v_front), -0.30, f"Yükseklik {_n(z_top - gz, 2)} m", horiz=False)
    dim((-mg[1], gz + v_front), (mg[1], gz + v_front), -0.25, f"İz {_n(2 * mg[1], 2)} m")
    # propeller disc (dash-dot) in the front view, labelled by a leader; disc edge line in the plan and side views
    th = np.linspace(0.0, 2 * np.pi, 181)
    ax.plot(hub[1] + D / 2 * np.cos(th), hub[2] + v_front + D / 2 * np.sin(th), ls="-.", lw=0.6, color="#374151")
    ax.plot([hub[1] - D / 2, hub[1] + D / 2], [-hub[0]] * 2, ls="-.", lw=0.6, color="#374151")
    ax.plot([u_side + hub[0]] * 2, [hub[2] + v_front - D / 2, hub[2] + v_front + D / 2], ls="-.", lw=0.6,
            color="#374151")
    ax.annotate(f"Pervane diski Ø{_n(D, 3)} m", xy=(hub[1] + D / 2 * np.cos(np.pi / 5), hub[2] + v_front +
                D / 2 * np.sin(np.pi / 5)), xytext=(hub[1] + D / 2 + 0.75, hub[2] + v_front + D / 2 + 0.05),
                fontsize=8.5, va="center", arrowprops=dict(arrowstyle="-", lw=0.5, color="k"))
    # side view (static ground attitude)
    dim((u_side + ng[0], gz + v_front), (u_side + mg[0], gz + v_front), -0.25,
        f"Dingil açıklığı {_n(mg[0] - ng[0], 2)} m")
    dim((u_side + x_side0, gz + v_front), (u_side + x_side1, gz + v_front), -0.55,
        f"Yerde boy {_n(x_side1 - x_side0, 2)} m")
    ax.text(u_side + x_side1, gz + v_front - 0.12, f"statik tutum {_n(math.degrees(th_s), 1)}° burun yukarı",
            fontsize=8, ha="right", va="center", color="#4B5563")
    ax.text(-b2, v_front + z_top + 0.35, "ÖNDEN GÖRÜNÜŞ", fontsize=11, weight="bold")
    ax.text(u_side, v_front + z_top + 0.35, "YANDAN GÖRÜNÜŞ (sol)", fontsize=11, weight="bold")
    ax.text(-b2, 0.25, "ÜSTTEN GÖRÜNÜŞ (burun yukarıda, sancak sağda)", fontsize=11, weight="bold")
    M = metrics(S, R)
    rows = [("Ad", S["meta"]["name"]), ("Revizyon", f"{S['meta']['revision']} ({S['meta']['date']})"),
            ("MTOM / boş / yakıt / yük", f"{_n(M['mtow_kg'], 1)} / {_n(M['empty_kg'], 1)} / {_n(M['fuel_kg'], 1)} / "
                                         f"{_n(M['payload_kg'], 1)} kg (görev yükü)"),
            ("Azami faydalı yük", f"{_n(M['payload_max_checked_kg'], 1)} kg ({_n(M['endurance_max_payload_h'], 2)} h)"),
            ("Kanat", f"S {_n(float(W['area']), 2)} m², AR {_n(float(W['aspect_ratio']), 1)}, OAK {_n(float(W['mac']), 3)} m"),
            ("Motor / pervane", f"Limbach L 275 EF 18 kW / Mejzlik {S['propeller'].get('designation', '32x18 2B')} "
                                f"itici"),
            ("Dayanım (tasarım görevi)", f"{_n(M['endurance_h'], 2)} h ({_n(M['payload_kg'], 1)} kg yük; 3000 m, "
                                         f"taret dışarıda)"),
            ("Tutunma hızı (temiz, MTOM)", f"{_n(M['vs_clean_sl_mtow_m_s'], 1)} m/s"),
            ("(L/D)maks", f"{_n(M['ld_max'], 1)}"),
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
        note, D5, err = None, None, None
        # the closure moves the wing with the CG; with the LERX apex fixed on the chine the LERX leading edge can stop
        # being single-valued in span: then move the apex aft with the stretch (d, 2 d) and close again
        for k_apex in (0.0, 1.0, 2.0):
            S5a = copy.deepcopy(S5)
            S5a["wing"]["planform"]["x_apex"] += k_apex * d
            try:
                D5 = design_closure(S5a, verbose=False)
                note = None if k_apex == 0.0 else "LERX apex moved with the stretch (x_apex %+.2f m)" % (k_apex * d)
                break
            except ValueError as e:
                err = str(e)
        if D5 is None:
            rows.append({"body_stretch_m": d, "endurance_h": None, "error": err})
            continue
        r5 = evaluate(D5, sens=False, light=True)
        M5 = metrics(D5, r5)
        rows.append({"body_stretch_m": d, "body_length_m": D5["fuselage"]["stations"][-1][0],
                     "volume_h": r5["stability"]["V_H"], "empty_kg": M5["empty_kg"], "endurance_h": M5["endurance_h"],
                     "cn_beta": M5["cn_beta_per_rad"], "static_margin_min": M5["static_margin_min"], "note": note})
        if verbose:
            print(f"  trade tail arm: stretch {d:+.2f} -> E {M5['endurance_h']:.2f} h, Cnb {M5['cn_beta_per_rad']:.4f}",
                  flush=True)
    out["tail_arm"] = rows
    out["r02_ladder"] = r02_ladder(S, R, verbose)
    if verbose:
        print(f"  trades: {time.time() - t0:.0f} s")
    return out


# R-02 ladder (doc 02 sec. 14): each entry reverts or changes ONE item of the design and re-closes it in full. Fix round 3
# items first (the decisions R-02 now depends on), then the v1.3 / v1.2 design changes still in the design.
def _battery_lifepo4(S_):
    E_ = S_["engine"]["electrical_budget"]
    E_.update(battery_model="14S2P A123 ANR26650M1-B LiFePO4", battery_energy_nominal_Wh=231.0, battery_mass_kg=2.43,
              battery_continuous_current_A=100.0)


R02_REVERTS = (
    ("k_inst_095", "k_inst 0,95 (tahmin 0,93 yerine; pervane/gövde kurulum testi bekliyor)",
     lambda S_: S_["propeller"].update(k_inst=0.95)),
    ("r52_v13_installed_turret", "R-52 v1.3 biçimi (yalnız takılı taretin tepesi; kullanıcı onaylamadı, uygulanmadı)",
     lambda S_: S_["mission"].update(loiter_rpm_floor="installed_turret_peak")),
    ("r52_v12_generator_lifepo4", "v1.2 çözümü: LiFePO4 14S2P batarya, E180 tepesi yalnız jeneratörle (E180 devir tabanı)",
     lambda S_: (_battery_lifepo4(S_), S_["mission"].update(loiter_rpm_floor="generator_peak"))),
    ("k_inst_095_and_r52_v13", "k_inst 0,95 ve R-52 v1.3 birlikte (ikisi de uygulanmadı)",
     lambda S_: (S_["propeller"].update(k_inst=0.95), S_["mission"].update(loiter_rpm_floor="installed_turret_peak"))),
    ("sweep_c4", "c/4 süpürme 8° → 6° (v1.2)", lambda S_: S_["wing"]["planform"].update(sweep_c4_deg=6.0)),
    ("washout", "burulma 4° → 3° (v1.2)", lambda S_: S_["wing"]["planform"].update(washout_deg=3.0)),
    ("lerx_apex_v13", "LERX tepesi x 1,80 → 1,85 m (v1.2)", lambda S_: S_["wing"]["planform"].update(x_apex=1.85)),
    ("sealed_gear_doors", "kapaklar contalı değil (v1.1: uzatılmış takım sürüklemesinin %10'u)",
     lambda S_: S_["aero"]["drag_rules"].update(gear_doors_sealed=False)),       # no seal mass (gear_doors_mass)
    ("glove_junction", "eldiven/dış panel birleşimi y = 0,82 m (v1.1)",
     lambda S_: S_["wing"]["planform"].update(y_junction=0.82)),
    ("taper", "sivrilme 0,42 (v1.1)", lambda S_: S_["wing"]["planform"].update(taper=0.42)),
)


def r02_ladder(S: dict, R: dict, verbose: bool = True) -> list:
    """Endurance of the design mission with each R-02 design change reverted alone (full design closure each)."""
    rows = [{"change": "design", "text_tr": f"{S['meta']['revision']} tasarımı", "endurance_h": R["performance"]["endurance_h"],
             "empty_kg": float(S["mass"]["empty_kg"])}]
    for key, txt, fn in R02_REVERTS:
        S6 = copy.deepcopy(S)
        fn(S6)
        try:
            D6 = design_closure(S6, verbose=False)
            D6.pop("_closure_history", None)
            r6 = evaluate(D6, sens=False, light=True)
            M6 = metrics(D6, r6)
            rows.append({"change": key, "text_tr": txt, "endurance_h": M6["endurance_h"], "empty_kg": M6["empty_kg"],
                         "delta_h": rows[0]["endurance_h"] - M6["endurance_h"]})
        except ValueError as e:                                 # e.g. an infeasible LERX for the reverted pair
            rows.append({"change": key, "text_tr": txt, "endurance_h": None, "error": str(e)})
        if verbose:
            print(f"  R-02 ladder: without {key}: {rows[-1].get('endurance_h')}", flush=True)
    return rows


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
    pr_ = p.get("payload_design_rule", {})
    mx_ = p.get("max_payload_mission", {})
    rows = [("MTOM / boş / yakıt / görev faydalı yükü", f"{_f(M['mtow_kg'], 1)} / {_f(M['empty_kg'], 1)} / "
             f"{_f(M['fuel_kg'], 1)} / {_f(M['payload_kg'], 1)} kg"),
            ("Azami faydalı yük (R-02b)", f"{_f(mx_.get('payload_kg'), 1)} kg: dayanım {_f(mx_.get('endurance_h'))} h "
             f"(yakıt {_f(mx_.get('fuel_kg'), 1)} kg)"),
            ("Kanat açıklığı / alan / AR", f"{_f(M['span_m'])} m / {_f(float(S['wing']['area']), 3)} m² / "
             f"{_f(float(S['wing']['aspect_ratio']), 1)}"),
            ("Gövde boyu / genişlik / yükseklik", f"{_f(R['geometry']['body_length_m'])} / {_f(R['geometry']['body_width_max_m'])} / "
             f"{_f(R['geometry']['body_height_max_m'])} m"),
            ("Dayanım (tasarım görevi, görev yüküyle)", f"{_f(M['endurance_h'])} h (bekleme {_f(M['loiter_time_h'])} h)"),
            ("Menzil (feribot, yedek dahil)", f"{_f(M['range_km'], 0)} km"),
            ("Görev beklemesi başı / sonu (3000 m)",
             " / ".join(f"{_f(lp['V'])} m/s TAS ({_f(lp['EAS'])} EAS), {_f(lp['rpm'], 0)} rpm, {_f(lp['ff_kg_h'])} kg/h"
                        for lp in (p["mission_loiter_points"]["start"], p["mission_loiter_points"]["end"]))),
            ("MTOM'da 3000 m bekleme noktası (karşılaştırma)", f"{_f(p['loiter_3000m']['V'], 1)} m/s TAS "
             f"({_f(p['loiter_3000m']['EAS'], 1)} EAS), CL {_f(p['loiter_3000m']['CL'])}, L/D {_f(p['loiter_3000m']['LD'], 1)}, "
             f"{_f(p['loiter_3000m']['ff_kg_h'])} kg/h"),
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
    if pr_:
        w("\n### Gereksinim kararı: görev faydalı yükü kuralı\n")
        w(f"Kural: {pr_.get('rule_tr', pr_.get('rule'))}. Spec değeri {_f(pr_.get('spec_kg'), 1)} kg; kapalı tasarımdan "
          f"türetilen {_f(pr_.get('derived_kg'), 1)} kg ({_f(pr_.get('endurance_at_derived_h'), 3)} h; bir adım fazlası "
          f"{_f(pr_.get('next_step_kg'), 1)} kg ile {_f(pr_.get('endurance_at_next_step_h'), 3)} h). Azami yük "
          f"{_f(mx_.get('payload_kg'), 1)} kg ile dayanım {_f(mx_.get('endurance_h'), 3)} h (R-02b ≥ "
          f"{_f(mx_.get('requirement_h'), 1)} h).")
        if pr_.get("endurance_margin_to_target_h") is not None:
            w(f"\nKural eşiğine pay (V4-03): görev yüküyle dayanım eşiğin {_f(pr_['endurance_margin_to_target_h'], 3)} h "
              f"üstündedir. Bu, sabit MTOM'da boş kütlenin {_f(pr_.get('empty_mass_headroom_kg'), 3)} kg artmasına denktir "
              f"(her kg boş kütle bir kg yakıttır); faydalı yük olarak eşik ≈ {_f(pr_.get('payload_at_target_kg_linear'), 2)} "
              f"kg'dır (ızgara noktaları arasında doğrusal). Boş kütle bundan fazla artarsa kural bir alt adımı verir.")
        bc0 = m["budget_check"]
        w(f"\nKütle bütçesi (R-56): R-02'yi tam karşılayan boş kütle {_f(bc0['empty_kg_at_R02_limit'], 3)} kg, R-02b'yi "
          f"tam karşılayan {_f(bc0.get('empty_kg_at_R02b_limit'), 3)} kg; belirleyen {bc0.get('governing')}; tavanlar "
          f"{_f(bc0['budget_sum_kg'], 2)} kg + yedek {_f(bc0['reserve_kg'], 2)} kg → pay {_f(bc0['margin_kg'], 3)} kg.")
    w("\n## 3. Kütle\n")
    w("| Grup | Kütle (kg) | Bütçe hedefi ± tolerans |\n|---|---|---|")
    bud = S["mass"].get("budget", {})
    for k, v in sorted(m["groups_kg"].items(), key=lambda t: -t[1]):
        b = bud.get(k)
        w(f"| {k} | {_f(v)} | {(_f(b['target_kg']) + ' ± ' + _f(b['tol_kg'])) if b else '–'} |")
    w(f"| **boş (büyüme payı %{_f(float(S['mass']['rules']['growth_allowance']) * 100, 0)} dahil)** | **{_f(M['empty_kg'])}** | |")
    w("\nYükleme durumları (ağırlık merkezi, uçuş, takım içeride; taret durumu yükleme adında):\n")
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
      f"). Pervane düzlemi kaporta firar kenarının {_f(pc['plane_behind_cowl_te_m'] * 1000, 0)} mm "
      f"({_f(pc['plane_behind_cowl_te_over_D'], 3)} D) gerisinde.")
    gm_ = pc["guard_map"]
    w("* Pervane koruması (itki hattı açısıyla eğik disk düzleminde): " + "; ".join(
        f"{ {'fin': 'eğik dikey', 'ventral': 'ventral kanatçık', 'stabilator': 'stabilatör'}.get(g_['part'], g_['part'])} "
        f"{_f(g_['angle_deg'], 0)}° (tepeden), uç çemberinin {_f(g_['margin_over_tip_m'] * 1000, 0)} mm dışı"
        for g_ in gm_["guards"]) + f". Uç çemberinin 0,30 m içinde hiçbir yapının bulunmadığı açık bölge payı "
      f"%{_f(gm_['open_sector_fraction'] * 100, 0)}; disk açıktır, personel koruması iddia edilmez. Kanat ucu yere değene "
      f"kadar yatışta ({_f(g['wingtip_ground_roll_deg'], 1)}°) pervane ucu yerden {_f(g['prop_clear_wingtip_on_ground_m'], 3)} m "
      f"yukarıda kalır.")
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
    fcs_ = to.get("fcs_stabilator_schedule", {})
    w(f"| Kalkış (DS, MTOM, belirleyici yükleme: {CASE_TR.get(to['case'], to['case'])}) | koşu "
      f"{_f(to['ground_roll_m'], 0)} m, 15 m'ye {_f(to['distance_15m_m'], 0)} m; burun kaldırma V_R "
      f"{_f(to['V_R_m_s'], 1)} m/s (ana tekerlerde {_f(to['main_gear_load_at_VR_N'], 0)} N), V_LOF "
      f"{_f(to['V_lof_m_s'], 1)} m/s, yerden kesilme tutumu {_f(to.get('theta_lof_deg'), 1)}° (zemin tutumu "
      f"{_f(to.get('theta_ground_deg'), 1)}°, dönüş {_f(to.get('rotation_time_s'), 2)} s) (VS_TO {_f(to['VS_TO_m_s'], 1)} m/s, "
      f"flap {_f(to['flap_deg'], 0)}°) |")
    if fcs_:
        rot_ = to.get("rotation", {})
        w(f"| — uçuş kontrol sistemi stabilatör programı | {_f(fcs_.get('download_start_m_s'), 1)} m/s'den itibaren ana "
          f"tekerleri yüklü tutan en küçük aşağı kuvvet, V_R'de tam burun yukarı (yerel CL {_f(fcs_['full_nose_up_local_cl'], 2)}); "
          f"dönüşte sabit yunuslama hızı ({_f(to.get('rotation_rate_deg_s'), 2)}°/s), aşağı kuvvet ana teker temas "
          f"noktasına göre moment dengesinden: {_f(rot_.get('download_at_V_R_N'), 0)} N'dan yerden kesilmede güç-açık trim "
          f"değerine {_f(rot_.get('download_at_lof_N'), 0)} N (trim {_f(rot_.get('trim_download_at_lof_N'), 0)} N); "
          f"en büyük aşağı kuvvete en küçük pay {_f(rot_.get('download_margin_min_N'), 1)} N, moment artığı en çok "
          f"{rot_.get('moment_residual_max_Nm', 0.0):.1e} N·m; {_f(fcs_['liftoff_not_below_m_s'], 1)} m/s (1,1 VS_TO) "
          f"altında yerden kesilme yok |")
    pie = p.get("takeoff_pitch_inertia_effect")
    if pie:
        rot_ = to.get("rotation", {})
        w(f"| — yunuslama ataleti (V4-05) | I_yy {_f(pie['I_yy_kg_m2'], 1)} kg·m² (kütle kalemlerinden; deri, kanat ve kuyruk "
          f"yüzeylerine yayılı, uzun kalemler çubuk); yunuslama hızı komutu {_f(pie['spin_up_time_s'], 1)} s'de "
          f"rampalanır ({_f(rot_.get('spin_up_pitch_accel_deg_s2'), 2)}°/s², V_R'de ana teker noktasına göre "
          f"{_f(rot_.get('spin_up_moment_at_V_R_Nm'), 1)} N·m). V_R {_f(pie['without_pitch_inertia']['V_R_m_s'], 2)} → "
          f"{_f(pie['V_R_m_s'], 2)} m/s, koşu {_f(pie['without_pitch_inertia']['ground_roll_m'], 1)} → "
          f"{_f(pie['ground_roll_m'], 1)} m (+{_f(pie['delta_ground_roll_m'], 1)} m); 0,1 s rampayla "
          f"{_f(pie['spin_up_0p1s']['ground_roll_m'], 1)} m |")
    for cn_, cv in to.get("cases", {}).items():
        w(f"| — {CASE_TR.get(cn_, cn_)} | koşu {_f(cv['ground_roll_m'], 0)} m, V_R {_f(cv['V_R_m_s'], 1)}, V_LOF "
          f"{_f(cv['V_lof_m_s'], 1)} m/s, θ_LOF {_f(cv.get('theta_lof_deg'), 1)}°, x_AM {_f(cv['x_cg'], 3)} m, "
          f"aşağı kuvvet payı {_f(cv['rotation']['download_margin_min_N'], 1)} N |")
    to2 = p["takeoff_1500m_isa_mtow"]
    w(f"| Kalkış (1500 m ISA) | koşu {_f(to2['ground_roll_m'], 0)} m |")
    ld = p["landing_sl_end_of_mission"]
    w(f"| İniş (görev sonu {_f(p['landing_mass_end_of_mission_kg'], 1)} kg) | koşu {_f(ld['ground_roll_m'], 0)} m, "
      f"V_TD {_f(ld['V_td_m_s'], 1)} m/s |")
    lm = p["landing_sl_mtow"]
    w(f"| İniş (MTOM, acil dönüş, flapsız; R-55) | koşu {_f(lm['ground_roll_m'], 0)} m, V_TD {_f(lm['V_td_m_s'], 1)} m/s; "
      f"35° flapla {_f(p['landing_sl_mtow_takeoff_flap']['ground_roll_m'], 0)} m |")
    for h in ("0", "3000"):
        q = p[h]
        w(f"| {h} m | VS {_f(q['VS_clean_m_s'], 1)} m/s, Vmaks {_f(q['V_max_m_s'], 1)} m/s, tırmanma "
          f"{_f(q['RoC_max_m_s'], 2)} m/s @ {_f(q['V_climb_m_s'], 1)} m/s |")
    w(f"| Tavan | servis {_f(p['ceiling_service_m'], 0)} m, mutlak {_f(p['ceiling_absolute_m'], 0)} m |")
    ppl = p.get("payload_permitted_loadings", {})
    w("\nFaydalı yük – dayanım (MTOM 149,9 kg; yakıt = MTOM − boş − yük, yakıt hacmiyle sınırlı). Tablo izin verilen en "
      "hafif yüklemeden, temel EO/IR setinden başlar (V4-02):\n")
    w("| Yük (kg) | Yakıt (kg) | Kalkış kütlesi (kg) | Dayanım (h) | Bekleme (h) | SM aralığı (% OAK) | E180 tepesi bütün "
      "bekleme sürseydi batarya (Wh) | E180 tepesinin desteklendiği bekleme (h) |\n|---|---|---|---|---|---|---|---|")
    el0 = R["electrical"]
    for r in p["payload_endurance"]:
        sup = r.get("e180_peak_supported_loiter_h")
        smr = r.get("static_margin_range") or [float("nan")] * 2
        e180_txt = (f"{_f(r.get('e180_peak_battery_draw_Wh'), 1)}{' > pay' if r.get('e180_peak_share_exceeded') else ''} | "
                    f"{'bütün bekleme' if sup is None else _f(sup, 2)}") if r.get("e180_set_fits", True) else \
            "– (E180 seti sığmaz) | –"
        w(f"| {_f(r['payload_kg'], 2)} | {_f(r['fuel_kg'], 2)}{' (hacim sınırı)' if r['fuel_volume_limited'] else ''} | "
          f"{_f(r['takeoff_mass_kg'], 1)} | {_f(r['endurance_h'], 2)} | {_f(r.get('loiter_time_h'), 2)} | "
          f"{_f(100 * smr[0], 1)}–{_f(100 * smr[1], 1)} | {e180_txt} |")
    w(f"\nTepe destek payı {_f(el0['battery_peak_share_Wh'], 1)} Wh. '> pay' satırlarında E180 tepesi bütün bekleme "
      f"boyunca karşılanamaz (R-52 işletme sınırı, bkz. §8). E180 sütunları yalnız E180 setini ({_f(ppl.get('e180_set_kg'), 1)} "
      f"kg: E180 4,0 + görev bilgisayarı 1,0 + tepsi 0,5) taşıyabilen yükler içindir.")
    if ppl:
        np_ = ppl["no_payload"]
        w(f"\nİzin verilmeyen yüklemeler (R-09, SM ≥ %{_f(100 * ppl['static_margin_min_rule'], 0)}): faydalı yüksüz "
          f"(taret de yok) SM %{_f(100 * np_['static_margin_range'][0], 1)}–%{_f(100 * np_['static_margin_range'][1], 1)} aralığındadır; "
          f"uçmak için taret bağlantısına (x = {_f(np_['ballast_x_m'], 3)} m) {_f(np_['nose_ballast_at_turret_mount_kg'], 2)} "
          f"kg safra gerekir. Yalnız taretle (görev bilgisayarı ve tepsi yok) dolu depolarda SM "
          f"%{_f(100 * ppl['turret_only']['static_margin_range_full_tanks'][0], 1)} olur; bu yüklemede yakıt "
          f"{_f(ppl['turret_only']['fuel_max_for_R09_kg'], 1)} kg ile sınırlıdır.")
    sens = R.get("sensitivities_endurance_h", {})
    SENS_TR = {"bsfc_minus_12pct": "BSFC −%12", "bsfc_plus_12pct": "BSFC +%12", "cd0_plus_10pct_all_drag":
               "bütün sürükleme +%10", "empty_plus_5pct": "boş kütle +%5", "loiter_at_1000m": "bekleme 1000 m'de",
               "no_transit_loiter_only": "geçişsiz", "turret_retracted_whole_mission": "taret hep içeride",
               "k_inst_0p90": "k_inst 0,90", "k_inst_0p95": "k_inst 0,95", "k_inst_0p97": "k_inst 0,97",
               "low_load_fuel_flow_willans_line": "düşük yükte Willans doğrusu"}
    if sens:
        w("\nDuyarlılıklar (görev yüküyle dayanım, h): " + ", ".join(f"{SENS_TR.get(k, k)} {_f(v, 2)}"
                                                                    for k, v in sens.items()))
    ll = R.get("low_load_bsfc_bound", {})
    if ll:
        w(f"\nDüşük yük BSFC dışdeğerlemesi (V3-07): en düşük BSFC noktası %{_f(ll['lowest_bsfc_point_power_fraction'] * 100, 0)} "
          f"güçtedir; alçalma %{_f(ll['descent_power_fraction_range_bsfc_line'][0] * 100, 1)}–"
          f"%{_f(ll['descent_power_fraction_range_bsfc_line'][1] * 100, 1)}, yedek bekleme "
          f"%{_f(ll['reserve_power_fraction_range_bsfc_line'][0] * 100, 1)}–"
          f"%{_f(ll['reserve_power_fraction_range_bsfc_line'][1] * 100, 1)} güçtedir (dışdeğerleme). Alçalma yakıtı BSFC "
          f"doğrusuyla {_f(ll['descent_fuel_kg_bsfc_line'], 3)} kg, Willans doğrusuyla (yakıt akışı güçle doğrusal; daha ihtiyatlı bir dışdeğerleme, sınır değil) "
          f"{_f(ll['descent_fuel_kg_willans_line'], 3)} kg; yedek {_f(ll['reserve_fuel_kg_bsfc_line'], 3)} / "
          f"{_f(ll['reserve_fuel_kg_willans_line'], 3)} kg; dayanım {_f(ll['endurance_h_bsfc_line'], 3)} / "
          f"{_f(ll['endurance_h_willans_line'], 3)} h.")
    w("\n## 8. Kumanda yüzeyleri, kökler, kiriş derinliği, elektrik, yükler\n")
    hm, rt, sd = R["stabilator_hinge"], R["tail_roots"], R["spar_depth"]
    acv = S["tail"]["surfaces"]["stabilator"]["controls"].get("ac_mac_fraction_vlm", {})
    band_ = hm["ac_mac_fraction_range"]
    w(f"* Stabilatör mili panel OAK'ının %{_f(hm['pivot_mac_fraction'] * 100, 1)}'inde (x = {_f(hm['spindle_x'], 3)} m); "
      f"girdap kafesi AM'si %{_f(acv.get('panels_only', float('nan')) * 100, 1)} (yalnız paneller) / "
      f"%{_f(acv.get('with_root_stubs', float('nan')) * 100, 1)} (kök parçalarıyla); AM bandı %{_f(band_[0] * 100, 1)}–"
      f"%{_f(band_[1] * 100, 1)} OAK → kol {_f(hm['ac_offset_min_m'] * 1000, 0)}–{_f(hm['ac_offset_max_m'] * 1000, 0)} mm "
      f"(yüzey mil etrafında hiçbir durumda kararsız değil). Tam sapmada normal kuvvet katsayısı kesit polarından "
      f"CN_maks {_f(hm['CN_max_panel'], 3)} (NeuralFoil, Re {hm['CN_max_basis']['Re']:.2e}; trim kuralı sabiti "
      f"{_f(hm['trim_rule_clmax_not_used'], 2)} kullanılmaz); basınç merkezi en geride %{_f(hm['cp_mac_fraction_max'] * 100, 1)} "
      f"OAK. Menteşe momenti VA'da {_f(hm['H_VA_Nm'], 1)} N·m, VD'de "
      f"{_f(hm['H_VD_Nm'], 1)} N·m, sürekli trim {_f(hm['H_trim_continuous_Nm'], 1)} N·m; {hm['actuator']}, "
      f"{_f(hm['linkage_ratio'], 1)}:1 dört çubuk bağlantısı (tam kinematik; tork oranı {_f(hm['linkage']['ratio_min'], 2)}–"
      f"{_f(hm['linkage']['ratio_max'], 2)}; −20/+15° için servo {_f(hm['servo_angle_at_range_deg'][0], 0)}°/"
      f"+{_f(hm['servo_angle_at_range_deg'][1], 0)}°) → bütün sapma aralığında tepe pay {_f(hm['peak_margin'], 2)} "
      f"(1,25 katsayısı dahil), anma torku / en büyük moment {_f(hm['rated_margin_max_hinge_moment'], 2)}, sürekli pay "
      f"{_f(hm['rated_margin'], 2)}; bağlantının ulaşabildiği en büyük sapma ±{_f(hm['surface_travel_deg'], 1)}°; anma "
      f"hızında yüzey hızı nötrde {_f(hm['surface_rate_neutral_deg_s'], 0)}°/s, −20°'de {_f(hm['surface_rate_at_rated_torque_deg_s'], 0)}°/s.")
    ch_ = R.get("control_hinges", {})
    if ch_:
        w("\n| Yüzey | Eyleyici | Durum | Menteşe momenti (N·m) | Servo açısı (aralık uçları) | Tork oranı | Anma payı (≥ 1,1) | Tepe payı (≥ 1,0) | Yüzey hızı (°/s) |\n"
          "|---|---|---|---|---|---|---|---|---|")
        for k_, ad_ in (("aileron", "kanatçık"), ("flap", "flap"), ("rudder", "dümen"), ("stabilator", "stabilatör")):
            c_ = ch_[k_]
            w(f"| {ad_} | {c_['actuator']} | {_f(c_.get('V_eas_m_s', float('nan')), 1)} m/s EAS | {_f(c_['H_design_Nm'], 2)} | "
              f"{_f(c_['servo_angle_at_range_deg'][0], 0)}° / {_f(c_['servo_angle_at_range_deg'][1], 0)}° (sınır "
              f"±{_f(c_['actuator_travel_deg'], 0)}°) | {_f(c_['ratio_min'], 2)}–{_f(c_['ratio_max'], 2)} | "
              f"{_f(c_['rated_margin_min'], 2)} | {_f(c_['peak_margin_min'], 2)} | {_f(c_['surface_rate_min_deg_s'], 0)}–"
              f"{_f(c_['surface_rate_neutral_deg_s'], 0)} |")
    sc_ = R["packaging"].get("stabilator_spindle", {})
    if sc_:
        w(f"* Stabilatör milleri: iki kısa mil; iç yatak motor bölmesi halka çerçevesinde krank/SG750 zarfının yanında "
          f"(y = {_f(sc_['inboard_end_y_m'], 3)} m), dış yatak sabit kök parçasında; silindirlerin "
          f"{_f(sc_['cylinder_clearance_m'] * 1000, 0)} mm önünde; kök parçasının mil istasyonundaki kalınlığı "
          f"{_f(sc_['stub_thickness_at_spindle_m'] * 1000, 1)} mm (yatak yuvası + kaplama payı "
          f"{_f(sc_['stub_section_margin_m'] * 1000, 1)} mm).")
    w(f"* Stabilatör kökü: sabit kök parçasına boşluk {_f(rt['stab_root_gap_m'] * 1000, 1)} mm (bütün sapma aralığında "
      f"sabit), gövdeye en az pay {_f(rt['stab_root_body_clearance_m'] * 1000, 1)} mm "
      f"({_f(rt['stab_root_deflection_range_deg'][0], 0)}°…{_f(rt['stab_root_deflection_range_deg'][1], 0)}°). Dikey "
      f"kuyruk kökü bütün veter boyunca en az {_f(rt['fin_root_embed_min_m'] * 1000, 1)} mm, ventral kanatçık kökü en az "
      f"{_f(-rt['ventral_root_max_gap_m'] * 1000, 1)} mm gömülü.")
    w(f"* Kiriş derinliği (gövde yanı → dış panel bağlantısı): ana kiriş en az {_f(sd['main']['min_depth_m'] * 1000, 1)} mm "
      f"(gerekli {_f(sd['required_main_m'] * 1000, 1)} mm), arka kiriş en az {_f(sd['rear']['min_depth_m'] * 1000, 1)} mm "
      f"(gerekli {_f(sd['required_rear_m'] * 1000, 1)} mm = bağlantı kesitinin kendi arka kiriş derinliği).")
    el = R["electrical"]
    w(f"* Elektrik (R-32, R-52): jeneratörün DC çıkışı = 800 W × rpm/7500 × güç elektroniği verimi "
      f"{_f(el.get('power_electronics_efficiency', 1.0), 2)}. E180 tepe yükü {_f(el['peak_e180_W'], 0)} W bütün bekleme "
      f"boyunca jeneratör + bataryanın tepe destek payıyla karşılanır (R-52): bekleme devri alt sınırı "
      f"{_f(el.get('generator_rpm_floor'), 0)} rpm (jeneratör en çok {_f(float(S['mission'].get('peak_support_deficit_cap_W', 0.0)), 0)} W "
      f"eksik kalır); tepe yük bütün bekleme sürseydi bataryadan çekilecek enerji tasarım görevinde "
      f"{_f(el.get('e180_peak_battery_draw_design_mission_Wh'), 1)} Wh, E180 görevinde "
      f"{_f(el.get('e180_peak_battery_draw_e180_mission_Wh'), 1)} Wh; tepe destek payı {_f(el['battery_peak_share_Wh'], 1)} Wh "
      f"({el['battery_model']}: kullanılabilir {_f(el['battery_usable_Wh'], 1)} Wh − jeneratör kaybı/marş yedeği "
      f"{_f(el['battery_reserve_Wh'], 1)} Wh) → pay {_f(el['peak_support_margin'], 3)}. Jeneratör tek başına tasarım "
      f"görevinin en düşük bekleme devrinde E180 tepesini {_f(el['margin_peak'], 3)}, HD59 tepesini "
      f"{_f(el.get('margin_peak_hd59_design_mission'), 3)} payla karşılar. E180 görevinin dayanımı "
      f"{_f(el.get('e180_mission_endurance_h'), 2)} h. Sürekli yük (en büyüğü E180 ile {_f(el['continuous_e180_W'], 0)} W) "
      f"iki görevin en düşük çıkışına göre pay {_f(el['margin_continuous'], 3)} (R-32).")
    e180_ = p.get("e180_growth_mission") or {}
    if e180_:
        w(f"* E180 büyüme görevi (V3-08): jeneratör bu görevde E180 sürekli yükünü taşır "
          f"({_f(e180_.get('continuous_load_W'), 0)} W; alçalma devir tabanı {_f(e180_.get('descent_rpm'), 0)} rpm).")
    if el.get("peak_support_guaranteed_loiter_h_at_cap"):
        w(f"* R-52 işletme sınırı (V3-09): tepe destek payı, R-52'nin denetlediği bekleme sürelerine göre boyutlanmıştır "
          f"(tasarım görevi {_f(p['loiter_time_h'], 2)} h). E180 tepesi devir tabanında sürekli çekilirse pay en az "
          f"{_f(el['peak_support_guaranteed_loiter_h_at_cap'], 2)} h bekleme yeter (açık her an üst sınırda); daha uzun "
          f"beklemede (daha hafif yük, daha çok yakıt) pay bitince güç yönetimi bekleme devrini jeneratörün tek başına "
          f"E180 tepesini taşıdığı {_f(el['generator_rpm_e180_peak_generator_only'], 0)} rpm'e çıkarır ya da tepe yük "
          f"süresi sınırlanır.")
    de_ = [r_ for r_ in p["mission_log"] if r_["kind"] == "descent"]
    if de_:
        w(f"* Alçalma (V2-07): motor jeneratörün sürekli yükü taşıdığı devirde ({_f(de_[0]['rpm'], 0)} rpm), "
          f"{_f(float(S['mission']['descent_rate']), 1)} m/s alçalma hızı için çözülen {_f(min(r_['V'] for r_ in de_), 1)}–"
          f"{_f(max(r_['V'] for r_ in de_), 1)} m/s TAS'ta; {_f(sum(r_['dt_s'] for r_ in de_) / 60.0, 1)} dk, "
          f"{_f(sum(r_['fuel_kg'] for r_ in de_), 3)} kg yakıt (eski sabit kesir 0,998 ile 0,25 kg).")
    ol = R["loads"].get("operating_limits", {})
    if ol:
        w(f"* İşletme sınırları (R-54): VNE {_f(ol['VNE_eas'], 1)} m/s EAS (0,9 VD), VNO {_f(ol['VNO_eas'], 1)} m/s EAS; "
          f"uçuş kontrol sistemi hızı {_f(ol['fcs_speed_limit_eas'], 1)} m/s EAS ile sınırlar. Tam güçte düz uçuş hızı "
          f"{_f(ol['V_max_level_sl_eas'], 1)} m/s (DS) VNE'yi aştığından zarf koruması zorunlu bir işlevdir.")
    tri_ = R.get("tail_root_interference", {})
    if tri_:
        w(f"* Kuyruk kökleri (R-53): gövde dış yüzeyinde budanmış kök yapıları (kök bandı {_f(tri_['band_depth_m'] * 1000, 0)} mm, "
          f"motor duvarının gerisinde {_f(tri_['band_depth_aft_of_firewall_m'] * 1000, 0)} mm) ve stabilatör milleri iç "
          f"bölgelerle çakışmıyor ({tri_['n_conflicts']} çakışma); en küçük aralıklar: " +
          ", ".join(f"{k} {_f(v * 1000, 0)} mm" for k, v in tri_["min_distance_m"].items()) + ".")
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
    bcn = R["packaging"].get("bay_contents", {})
    if bcn.get("items"):
        w("\nBölme içerikleri (V3-02; gerçek zarflar + bağlayıcı payı; R-26'nın parçası):\n")
        w("| Öğe | Bölge | Boyut (mm) | OML'ye pay (mm) | Bölge duvarına pay (mm) | Komşuya en yakın (mm) |\n"
          "|---|---|---|---|---|---|")
        for r_ in bcn["items"]:
            w(f"| {r_['name']} | {r_['zone']} | {' × '.join(_f(v * 1000, 1) for v in r_['dims_m'])} | "
              f"{_f(r_['oml_margin_m'] * 1000, 1)} | {_f(r_['zone_wall_margin_m'] * 1000, 1)} | "
              f"{_f(r_['min_gap_to_other_items_m'] * 1000, 1)} |")
        if bcn.get("failures"):
            w("\nSığmayanlar: " + "; ".join(bcn["failures"]))
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
            w(f"| {_f(r['span_m'])}{' (tasarım)' if r.get('design') else ''} | {_f(r['stall_speed_target_m_s'], 2)} | "
              f"{_f(r['area_m2'], 3)} | {_f(r['aspect_ratio'], 1)} | {_f(r['wing_kg'])} | {_f(r['empty_kg'])} | "
              f"{_f(r['endurance_h'])} | {_f(r['takeoff_roll_m'], 0)} |")
        w("\n| Gövde uzatma (m) | Boy (m) | V_H | Boş (kg) | Dayanım (h) | Cnβ |\n|---|---|---|---|---|---|")
        for r in trades["tail_arm"]:
            w(f"| {_f(r['body_stretch_m'])} | {_f(r.get('body_length_m'))} | {_f(r.get('volume_h'), 3)} | "
              f"{_f(r.get('empty_kg'))} | {_f(r.get('endurance_h'))} | {_f(r.get('cn_beta'), 4)} |"
              + (" LERX tepesi de kaydırıldı" if r.get("note") else "") + (" kapanış hatası" if r.get("error") else ""))
        if trades.get("r02_ladder"):
            w("\nTasarım değişiklikleri merdiveni (görev yüküyle; her değişiklik tek başına geri alınırsa, tam tasarım "
              "kapanışıyla):\n")
            w("| Durum | Dayanım (h) | Fark (h) |\n|---|---|---|")
            for r in trades["r02_ladder"]:
                w(f"| {r['text_tr']} | {_f(r.get('endurance_h'))} | {_f(-r['delta_h']) if 'delta_h' in r else '–'} |")
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
    "layout": "layout (interface definition: stations, chassis, shell, mechanisms, keep-outs, clearances, systems; "
              "zones from sizing)",
    "assembly": "assembly (sequence, transport, field assembly, maintenance access)"}

GEAR_CHECK_KEYS = ("z_g", "track", "wheelbase", "h_cg", "static_attitude_deg", "tipback_deg", "turnover_deg",
                   "nose_load_aft_cg", "nose_load_fwd_cg", "prop_clear_level", "prop_clear_static",
                   "prop_clear_liftoff", "prop_clear_touchdown_unloaded", "prop_clear_min_925a",
                   "gear_unloaded_extension_m", "prop_clear_flat_tyre_bottomed", "prop_strike_deg",
                   "bumper_contact_deg", "theta_lof_deg", "theta_td_deg", "theta_flare_deg",
                   "wingtip_ground_roll_deg", "prop_clear_wingtip_on_ground_m")
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
    write_glove_airfoils(S_in)
    # requirement decision (fix round 4): the design-mission payload is derived by the payload-endurance rule on the
    # closed aircraft; the closure depends on it (loading cases, fuel), so close -> evaluate -> update the payload
    # until it repeats (a 0.5 kg grid: one or two passes)
    S_cur = copy.deepcopy(S_in)
    for k_ in range(6):
        D = design_closure(S_cur, verbose=verbose)
        D.pop("_closure_history", None)
        R = evaluate(D, sens=False)
        p_new = R["performance"]["payload_design_rule"]["derived_kg"]
        if verbose:
            print(f"  [payload rule] pass {k_}: spec {D['mission']['payload_design_kg']} kg -> derived {p_new} kg "
                  f"(E {R['performance']['payload_design_rule']['endurance_at_derived_h']:.3f} h)", flush=True)
        if abs(float(p_new) - float(D["mission"]["payload_design_kg"])) < 1e-9:
            break
        S_cur["mission"]["payload_design_kg"] = float(p_new)
    else:
        raise RuntimeError("design-payload rule did not repeat in 6 closure passes")
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
        k: hm[k] for k in ("spindle_x", "panel_mac_m", "ac_offset_min_m", "ac_offset_max_m", "CN_max_panel",
                           "cp_mac_fraction_max", "H_VA_Nm", "H_VD_Nm",
                           "H_trim_continuous_Nm", "peak_capacity_Nm", "peak_margin", "rated_margin",
                           "rated_margin_max_hinge_moment", "surface_travel_deg", "travel_ok",
                           "servo_angle_at_range_deg", "surface_rate_at_rated_torque_deg_s", "surface_rate_neutral_deg_s",
                           "statically_stable_surface")}
    # V2-03: hinge moments and four-bar actuation of the ailerons, flaps and rudders
    CH = R["control_hinges"]
    ck = ("H_design_Nm", "V_eas_m_s", "servo_angle_at_range_deg", "travel_margin_deg", "ratio_min", "ratio_max",
          "rated_margin_min", "peak_margin_min", "surface_rate_min_deg_s", "surface_rate_neutral_deg_s")
    for k_ in ("aileron", "flap"):
        D["wing"]["controls"][k_]["checks"] = {kk: CH[k_][kk] for kk in ck}
    TL["surfaces"]["fin"]["controls"]["rudder"]["checks"] = {kk: CH["rudder"][kk] for kk in ck}
    rt = R["tail_roots"]
    TL["surfaces"]["stabilator"]["root_checks"] = {k: rt[k] for k in ("stab_root_gap_m", "stab_root_body_clearance_m",
                                                                      "stab_root_deflection_range_deg")}
    sc_ = R["packaging"].get("stabilator_spindle", {})
    TL["surfaces"]["stabilator"]["spindle_check"] = {k: sc_[k] for k in (
        "cylinder_clearance_m", "inboard_end_y_m", "stub_thickness_at_spindle_m", "stub_section_margin_m",
        "spindle_chord_fraction_at_root", "inside_body", "ok") if k in sc_}
    TL["surfaces"]["fin"]["root_checks"] = {k: rt[k] for k in ("fin_root_max_gap_m", "fin_root_embed_min_m")}
    ti = R["tail_root_interference"]
    TL["root_structure"]["checks"] = {"n_conflicts": ti["n_conflicts"], "conflicts": ti["conflicts"],
                                      "min_distance_m": ti["min_distance_m"],
                                      "planar_root_extension_depth_m": {k: v["extension_depth_m"]
                                                                        for k, v in ti["surfaces"].items()}}
    TL["surfaces"]["ventral"]["root_checks"] = {"ventral_root_max_gap_m": rt["ventral_root_max_gap_m"]}
    pcl = pr["clearances"]
    D["propeller"]["clearance_checks"] = {
        "radial_min_m": pcl["radial_min_m"], "longitudinal_min_m": pcl["longitudinal_min_m"],
        "by_part": pcl["by_part"], "spinner_to_cowl_gap_m": pcl["spinner_to_cowl_gap_m"],
        "fin_te_crossing_radius_m": pcl["fin_te_crossing_radius_m"],
        "fin_guard_margin_over_tip_m": pcl["fin_guard_margin_over_tip_m"],
        "ventral_guard_margin_over_tip_m": pcl["ventral_guard_margin_over_tip_m"],
        "plane_behind_cowl_te_m": pcl["plane_behind_cowl_te_m"],
        "plane_behind_cowl_te_over_D": pcl["plane_behind_cowl_te_over_D"],
        "guards": [{k: g_[k] for k in ("part", "angle_deg", "radius_m", "margin_over_tip_m")}
                   for g_ in pcl["guard_map"]["guards"]],
        "open_sector_fraction_within_0p3m": pcl["guard_map"]["open_sector_fraction"],
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
                             "loiter_3000m": p["loiter_3000m"], "payload_design_rule": p["payload_design_rule"],
                             "max_payload_mission": p["max_payload_mission"],
                             "low_load_bsfc_bound": R["low_load_bsfc_bound"],
                             "payload_permitted_loadings": p["payload_permitted_loadings"],
                             "takeoff_pitch_inertia_effect": p.get("takeoff_pitch_inertia_effect")})
    bc_ = R["packaging"]["bay_contents"]
    D["layout"]["bay_contents_check"] = {"fits": bc_["fits"], "failures": bc_["failures"], "min_gap_m": bc_["min_gap_m"],
                                         "items": [{k: r_[k] for k in ("name", "zone", "dims_m", "oml_margin_m",
                                                                         "zone_wall_margin_m", "min_gap_to_other_items_m")}
                                                   for r_ in bc_["items"]]}
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
                                  "schrenk_basis": ws_["schrenk_basis"],
                                  "operating_limits": R["loads"]["operating_limits"]}
    D["layout"]["ground_z"] = D["landing_gear"]["ground_z"]
    # keep the spec's shape: its top-level blocks and their keys in the spec's order (closure working keys stay out);
    # the reference/handover keys written above are added after the authored keys of their block
    added = {"aero": ("cd0", "e", "k", "ld_max", "clmax_clean", "clmax_to", "clmax_ld", "polars", "power_on_trim",
                      "flaps"),
             "propeller": ("clearance_checks",), "tail": ("arm_h_wing_ac_to_tail_ac", "arms_note"),
             "fuselage": ("lines_derived",),
             "performance": ("payload_design_rule", "max_payload_mission", "low_load_bsfc_bound",
                             "payload_permitted_loadings", "takeoff_pitch_inertia_effect"),
             "layout": ("bay_contents_check",)}
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
    """Workbench renders from the spec geometry (gear down + turret extended; and gear/turret retracted, from below). The
    ground plane passes through the main-wheel contact and is inclined by the static attitude (nose wheel on it)."""
    fig_dir = Path(fig_dir or FIG_DIR)
    LG = S["landing_gear"]
    th = float(LG["checks"]["static_attitude_deg"]) if "checks" in LG else 0.0
    ground = (float(LG["main"]["axle_static"][0]), float(LG["ground_z"]), th)
    out = {}
    parts = scene_parts(S, "extended", "down")
    out.update(render_scene(parts, fig_dir, "yk250_", ground))
    parts2 = scene_parts(S, "retracted", "up")
    out.update({"retracted_belly": render_scene(parts2, fig_dir, "yk250_turret_gear_retracted_", ground,
                                                views=["belly"])["belly"]})
    return out


def outputs_written(a) -> bool:
    """F18: ``--check`` is a read-only verification. sizing.json/.md and the figures are written by the explicit
    output command (no flag) or into the ``--out`` directory; --design/--trades/--render/--update-spec write their own
    named outputs only."""
    return (not a.check) or bool(a.out)


def build_parser() -> argparse.ArgumentParser:
    ap = argparse.ArgumentParser(description="YK-250 HANCER sizing study")
    ap.add_argument("--check", action="store_true", help="compare derived spec values and evaluate requirements "
                                                         "(read only unless --out is given)")
    ap.add_argument("--design", action="store_true", help="design closure -> out/sizing_design.yaml")
    ap.add_argument("--trades", action="store_true", help="trade studies -> out/sizing_trades.json")
    ap.add_argument("--render", action="store_true", help="Workbench renders (needs bpy)")
    ap.add_argument("--update-spec", action="store_true",
                    help="re-close the design and rewrite the derived values of the spec file in place")
    ap.add_argument("--no-figures", action="store_true")
    ap.add_argument("--spec", default=None, help="alternative spec file")
    ap.add_argument("--out", default=None, help="output directory for sizing.json/.md (and the figures in <out>/fig); "
                                                "default ucav250/out and ucav250/docs/fig; with --check the outputs "
                                                "are written only when --out is given")
    return ap


def main(argv=None) -> int:
    a = build_parser().parse_args(argv)
    t0 = time.time()
    S = copy.deepcopy(SPEC.load(a.spec)) if a.spec else copy.deepcopy(SPEC.load())
    out_dir = Path(a.out) if a.out else OUT_DIR
    fig_dir = (out_dir / "fig") if a.out else FIG_DIR
    write_outputs = outputs_written(a)
    if write_outputs or a.design or a.trades:
        out_dir.mkdir(parents=True, exist_ok=True)
    if a.design:
        write_glove_airfoils(S)
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
    R.pop("_flight", None)
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
    if write_outputs:
        figs = existing_figures(FIG_DIR if not a.out else fig_dir) if a.no_figures else write_figures(S, R, fig_dir)
        # no run time or date in the written files: repeated runs on the same spec give identical outputs
        doc = {"meta": R["meta"], "metrics": M, "requirements": reqs, "derived_check": chk, "figures": figs, **R}
        with open(out_dir / "sizing.json", "w", encoding="utf-8") as fh:
            json.dump(py(doc), fh, indent=1, ensure_ascii=False)
        (out_dir / "sizing.md").write_text(write_report(S, R, M, reqs, chk, figs, trades), encoding="utf-8")
    else:
        print("[sizing] --check: verification only, no files written (use --out DIR to keep the outputs)")
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
