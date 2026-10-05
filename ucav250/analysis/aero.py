"""Aerodynamic analysis for ucav250: section polars (NeuralFoil), Glauert lifting line, critical-section CLmax and a
strip-integrated profile drag. Geometry comes from ``spec.yaml`` in the :class:`ucav250.design.oml.LiftingSurface`
section format, so the sizing study and the 3-D model use exactly the same planform.

Methods
* Section polars: NeuralFoil (``model_size='xlarge'``, e^N transition, N_crit from the caller). Polars are computed on a
  fixed Reynolds grid and cached as JSON in ``data/polars``; characteristic values (lift slope, zero-lift angle,
  cl_max, cd(cl)) are interpolated in log(Re).
* Lifting line: Glauert's Fourier solution of Prandtl's monoplane equation for a symmetric wing (odd terms),
  spanwise-varying chord, twist, section lift slope and zero-lift angle; sweep is neglected (valid for the low sweep
  of a high-aspect-ratio wing, Λc/4 < 10°).
* Wing CL_max: critical-section method — the wing angle at which the local cl first reaches the local cl_max
  (local Reynolds number) — times ``k_clmax`` (0.95 default) for the 2-D → 3-D and roughness/production knock-down.
* Profile drag: strip integration of the section cd at the local cl and Reynolds number.
"""
from __future__ import annotations

import functools
import json
import math
from pathlib import Path

import numpy as np

from ..core.spec import DATA_DIR
from ..design.oml import airfoil_coords
from . import aerolib as AL

POLAR_DIR = Path(DATA_DIR) / "polars"
RE_GRID = (1.5e5, 2.5e5, 4e5, 6e5, 8e5, 1.1e6, 1.5e6, 2.0e6, 3.0e6)
ALPHAS = tuple(float(a) for a in np.arange(-8.0, 20.01, 0.5))


# =====================================================================================================================
# section polars
# =====================================================================================================================
def _polar_path(airfoil: str, Re: float, n_crit: float) -> Path:
    return POLAR_DIR / f"{airfoil.lower()}_Re{int(round(Re)):d}_N{n_crit:g}.json"


@functools.lru_cache(maxsize=None)
def raw_polar(airfoil: str, Re: float, n_crit: float = 9.0) -> dict:
    """NeuralFoil polar on ``ALPHAS`` (deg) at one Reynolds number; cached on disk."""
    path = _polar_path(airfoil, Re, n_crit)
    if path.exists():
        return json.loads(path.read_text())
    import neuralfoil as nf
    P = airfoil_coords(airfoil)
    a = np.array(ALPHAS)
    r = nf.get_aero_from_coordinates(P, alpha=a, Re=float(Re), n_crit=float(n_crit), model_size="xlarge")
    out = {"airfoil": airfoil, "Re": float(Re), "n_crit": float(n_crit), "alpha": list(ALPHAS),
           "cl": [float(v) for v in r["CL"]], "cd": [float(v) for v in r["CD"]], "cm": [float(v) for v in r["CM"]],
           "confidence": [float(v) for v in r["analysis_confidence"]],
           "xtr_top": [float(v) for v in r["Top_Xtr"]], "xtr_bot": [float(v) for v in r["Bot_Xtr"]],
           "method": "NeuralFoil xlarge (e^N transition)"}
    POLAR_DIR.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(out, indent=1))
    return out


def polar_characteristics(pol: dict) -> dict:
    """Lift slope (per rad, least squares over the linear range), zero-lift angle (deg), cl_max and its angle (first
    local maximum above alpha 4°), cd_min, max cl/cd, cm0, and the cd(cl) table up to cl_max."""
    a = np.asarray(pol["alpha"])
    cl = np.asarray(pol["cl"])
    cd = np.asarray(pol["cd"])
    cm = np.asarray(pol["cm"])
    lin = (a >= -2.0) & (a <= 4.0)
    k, c0 = np.polyfit(np.radians(a[lin]), cl[lin], 1)
    alpha0 = -math.degrees(c0 / k)
    i0 = int(np.searchsorted(a, 4.0))
    imax = i0
    for i in range(i0, len(a)):
        if cl[i] >= cl[imax]:
            imax = i
        elif cl[i] < cl[imax] - 0.02:                  # past the first peak
            break
    ld = cl / cd
    up = slice(int(np.argmin(cl[: imax + 1])), imax + 1)
    return {"cl_alpha": float(k), "alpha0_deg": float(alpha0), "clmax": float(cl[imax]),
            "alpha_clmax_deg": float(a[imax]), "cd_min": float(cd.min()), "cl_cd_max": float(ld.max()),
            "cl_at_cl_cd_max": float(cl[int(np.argmax(ld))]), "cm0": float(np.interp(0.0, cl[lin], cm[lin])),
            "cd_table": (cl[up].tolist(), cd[up].tolist()), "min_confidence": float(min(pol["confidence"]))}


@functools.lru_cache(maxsize=None)
def characteristics(airfoil: str, Re: float, n_crit: float = 9.0) -> dict:
    """Characteristics at any Re, log-linearly interpolated between the two bracketing grid polars."""
    Re = float(np.clip(Re, RE_GRID[0], RE_GRID[-1]))
    j = int(np.searchsorted(RE_GRID, Re))
    j = min(max(j, 1), len(RE_GRID) - 1)
    r0, r1 = RE_GRID[j - 1], RE_GRID[j]
    t = (math.log(Re) - math.log(r0)) / (math.log(r1) - math.log(r0))
    c0 = polar_characteristics(raw_polar(airfoil, r0, n_crit))
    c1 = polar_characteristics(raw_polar(airfoil, r1, n_crit))
    out = {k: (1 - t) * c0[k] + t * c1[k] for k in c0 if k not in ("cd_table",)}
    out["_tables"] = (c0["cd_table"], c1["cd_table"], t)
    out["Re"] = Re
    return out


def section_cd(ch: dict, cl: float) -> float:
    """Section drag at lift coefficient ``cl`` (beyond the table: quadratic growth toward stall)."""
    (cl0, cd0), (cl1, cd1), t = ch["_tables"]

    def one(cls, cds):
        cls, cds = np.asarray(cls), np.asarray(cds)
        if cl <= cls[-1]:
            return float(np.interp(cl, cls, cds))
        return float(cds[-1] + 0.5 * (cl - cls[-1]) ** 2)
    return (1 - t) * one(cl0, cd0) + t * one(cl1, cd1)


# =====================================================================================================================
# lifting line
# =====================================================================================================================
def lifting_line(y: np.ndarray, chord: np.ndarray, twist_deg: np.ndarray, a0: np.ndarray, alpha0_deg: np.ndarray,
                 semispan: float, n: int = 40) -> dict:
    """Glauert solution for a symmetric wing. Inputs are spanwise tables from the centreline (y = 0) to the tip
    (y = semispan). Returns linear coefficients so any root angle can be evaluated without re-solving:
    ``cl(y) = cl_a(y)·α + cl_0(y)`` (α in rad, measured at the root chord line), ``CL = CL_a·α + CL_0``,
    ``CDi = k_i·CL²`` style via Fourier coefficients, and the inviscid span efficiency at CL = 1 (``e``)."""
    b = 2.0 * semispan
    th = np.arange(1, n + 1) * (math.pi / 2) / n          # θ in (0, π/2]: y = semispan·cos θ (one half span)
    ys = semispan * np.cos(th)
    c = np.interp(ys, y, chord)
    tw = np.radians(np.interp(ys, y, twist_deg))
    aa = np.interp(ys, y, a0)
    al0 = np.radians(np.interp(ys, y, alpha0_deg))
    m = np.arange(1, 2 * n, 2)                            # odd harmonics
    S = np.sin(np.outer(th, m))
    M = S * (4 * b / (aa * c))[:, None] + S * m[None, :] / np.sin(th)[:, None]
    A_alpha = np.linalg.solve(M, np.ones(n))              # per radian of root angle
    A_0 = np.linalg.solve(M, tw - al0)                    # twist and camber part
    area = 2 * np.trapz(chord, y)
    AR = b * b / area

    def coeffs(alpha_rad):
        A = A_alpha * alpha_rad + A_0
        return A

    def cl_local(alpha_rad):
        A = coeffs(alpha_rad)
        return 4 * b * (S @ A) / c

    def CL(alpha_rad):
        return math.pi * AR * coeffs(alpha_rad)[0]

    def CDi(alpha_rad):
        A = coeffs(alpha_rad)
        return math.pi * AR * float(np.sum(m * A * A))

    CL_a = math.pi * AR * A_alpha[0]
    CL_0 = math.pi * AR * A_0[0]
    alpha_cl1 = (1.0 - CL_0) / CL_a
    A1 = coeffs(alpha_cl1)
    e = A1[0] ** 2 / float(np.sum(m * A1 * A1))
    return {"y": ys, "chord": c, "theta": th, "S_ref": area, "AR": AR, "CL_alpha": CL_a, "CL_0": CL_0,
            "alpha_zero_lift_deg": math.degrees(-CL_0 / CL_a), "e_inviscid": e, "cl_local": cl_local, "CL": CL,
            "CDi": CDi, "cl_a_local": 4 * b * (S @ A_alpha) / c, "cl_0_local": 4 * b * (S @ A_0) / c}


# =====================================================================================================================
# surfaces from spec
# =====================================================================================================================
def spanwise_tables(sections: list[dict], n: int = 60) -> dict:
    """Spanwise tables (centreline → tip) along the span coordinate of a lifting surface given in the
    LiftingSurface section format. Panel span is measured along the section planes (dihedral/cant included), so the
    same function serves a V-tail panel. A first section off the centreline is extended inboard (carry-through
    portion counted in the reference area, standard practice). Airfoil parameters blend linearly between sections."""
    pts = np.array([[s["y"], s.get("z_le", 0.0)] for s in sections], float)
    seg = np.r_[0.0, np.cumsum(np.linalg.norm(np.diff(pts, axis=0), axis=1))]
    root_off = float(pts[0, 0]) if abs(pts[0, 0]) > 1e-9 else 0.0
    eta_sec = root_off + seg                               # span coordinate from the plane of symmetry
    semispan = float(eta_sec[-1])
    yy = np.linspace(0.0, semispan, n)
    chord = np.interp(yy, eta_sec, [s["chord"] for s in sections])
    twist = np.interp(yy, eta_sec, [s.get("twist_deg", 0.0) for s in sections])
    x_le = np.interp(yy, eta_sec, [s["x_le"] for s in sections])
    # airfoil blending weights: index of the inboard section and fraction toward the outboard one
    idx = np.clip(np.searchsorted(eta_sec, yy, side="right") - 1, 0, len(sections) - 2)
    frac = np.clip((yy - eta_sec[idx]) / np.maximum(eta_sec[idx + 1] - eta_sec[idx], 1e-9), 0.0, 1.0)
    return {"y": yy, "chord": chord, "twist_deg": twist, "x_le": x_le, "semispan": semispan, "idx": idx,
            "frac": frac, "eta_sec": eta_sec}


def surface_analysis(sections: list[dict], V: float, h: float = 0.0, n_crit: float = 9.0, k_clmax: float = 0.95,
                     n_span: int = 60, n_ll: int = 40) -> dict:
    """Lifting-line analysis of one symmetric surface at airspeed V (TAS, m/s) and altitude h (m)."""
    T = spanwise_tables(sections, n_span)
    atm = AL.isa(h)
    re = atm["rho"] * V * T["chord"] / atm["mu"]
    chars = []
    for k in range(len(T["y"])):
        i, f = int(T["idx"][k]), float(T["frac"][k])
        ca = characteristics(sections[i]["airfoil"], float(re[k]), n_crit)
        cb = characteristics(sections[i + 1]["airfoil"], float(re[k]), n_crit)
        chars.append((ca, cb, f))

    def blend(key):
        return np.array([(1 - f) * ca[key] + f * cb[key] for ca, cb, f in chars])
    a0, al0, clmax = blend("cl_alpha"), blend("alpha0_deg"), blend("clmax")
    LL = lifting_line(T["y"], T["chord"], T["twist_deg"], a0, al0, T["semispan"], n_ll)
    ys = LL["y"]
    clmax_s = np.interp(ys, T["y"], clmax)
    a_crit = (clmax_s - LL["cl_0_local"]) / LL["cl_a_local"]
    i_crit = int(np.argmin(a_crit))
    alpha_stall = float(a_crit[i_crit])
    CLmax = k_clmax * LL["CL"](alpha_stall)

    def alpha_for(CL):
        return (CL - LL["CL_0"]) / LL["CL_alpha"]

    def cd_profile(CL):
        """Strip-integrated profile drag coefficient (on S_ref) at wing CL."""
        cl = LL["cl_local"](alpha_for(CL))
        cdl = []
        for k, yk in enumerate(ys):
            j = int(np.argmin(np.abs(T["y"] - yk)))
            ca, cb, f = chars[j]
            cdl.append((1 - f) * section_cd(ca, float(cl[k])) + f * section_cd(cb, float(cl[k])))
        cdl = np.array(cdl)
        order = np.argsort(ys)
        return float(2 * np.trapz((cdl * LL["chord"])[order], ys[order]) / LL["S_ref"])

    return {"tables": T, "ll": LL, "Re_root": float(re[0]), "Re_tip": float(re[-1]), "a0": a0,
            "alpha0_deg": al0, "clmax_sections": clmax, "CL_alpha": LL["CL_alpha"], "CL_0": LL["CL_0"],
            "e_inviscid": LL["e_inviscid"], "S_ref": LL["S_ref"], "AR": LL["AR"], "semispan": T["semispan"],
            "alpha_stall_deg": math.degrees(alpha_stall), "CLmax": CLmax, "stall_onset_eta": float(ys[i_crit] /
            T["semispan"]), "alpha_for": alpha_for, "cd_profile": cd_profile,
            "min_confidence": float(min(min(ca["min_confidence"], cb["min_confidence"]) for ca, cb, _ in chars))}
