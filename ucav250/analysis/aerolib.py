"""Aerodynamics and performance library for ucav250 (SI units). Aircraft-independent, unit tested.

Methods (standard conceptual-design references):
* ISA atmosphere (troposphere).
* Skin friction: flat-plate turbulent (Schlichting/Prandtl-Schlichting, 0.455/(log10 Re)^2.58 with compressibility
  ignored, M < 0.15) blended with laminar (1.328/sqrt(Re)) by a laminar run fraction; Raymer form factors for wings
  and bodies; interference factors (Raymer Table 12.x typical values); miscellaneous drag as ΔCD·S.
* Oswald span efficiency: Nita & Scholz (2012) (preferred for high AR) and the Raymer straight-wing fit
  e = 1.78(1 − 0.045 AR^0.68) − 0.64 (conservative lower bound for AR > 12).
* Lift-curve slope: Helmbold/DATCOM a = 2πAR / (2 + sqrt(4 + AR²β²/η² (1 + tan²Λ½c/β²))).
* Engine power lapse with altitude (normally aspirated piston): Gagg-Ferrar P/P0 = 1.132 σ − 0.132.
* Propeller efficiency: actuator-disk ideal efficiency times a profile/viscous factor, capped (simple, conservative).
* Endurance/range for propeller aircraft: Breguet with BSFC (kg/J) and propeller efficiency.
* Take-off ground roll and landing roll: Raymer energy method with average thrust/drag/friction.
"""
from __future__ import annotations

import math

import numpy as np

G0 = 9.80665
R_AIR = 287.053


# =====================================================================================================================
# atmosphere
# =====================================================================================================================
def isa(h: float) -> dict:
    """ISA properties at geometric altitude h (m), troposphere (h <= 11 km)."""
    T = 288.15 - 0.0065 * h
    p = 101325.0 * (T / 288.15) ** 5.25588
    rho = p / (R_AIR * T)
    mu = 1.458e-6 * T ** 1.5 / (T + 110.4)
    return {"T": T, "p": p, "rho": rho, "sigma": rho / 1.225, "mu": mu, "a": math.sqrt(1.4 * R_AIR * T)}


def reynolds(V: float, L: float, h: float = 0.0) -> float:
    a = isa(h)
    return a["rho"] * V * L / a["mu"]


# =====================================================================================================================
# drag build-up
# =====================================================================================================================
def cf_flat(Re: float, laminar_fraction: float = 0.0) -> float:
    """Mean skin-friction coefficient; laminar run fraction blended (Raymer 12.25/12.27 style)."""
    Re = max(Re, 1e3)
    cf_t = 0.455 / (math.log10(Re) ** 2.58)
    cf_l = 1.328 / math.sqrt(Re)
    return laminar_fraction * cf_l + (1.0 - laminar_fraction) * cf_t


def ff_wing(t_c: float, x_c_max: float = 0.3, sweep_max_t_deg: float = 0.0, M: float = 0.1) -> float:
    return (1 + 0.6 / x_c_max * t_c + 100 * t_c ** 4) * (1.34 * M ** 0.18 * math.cos(math.radians(sweep_max_t_deg))
                                                           ** 0.28)


def ff_body(fineness: float) -> float:
    f = max(fineness, 1.5)
    return 0.9 + 5 / f ** 1.5 + f / 400


def component_cd0(S_wet: float, S_ref: float, Re: float, ff: float, Q: float = 1.0, laminar: float = 0.0) -> float:
    return cf_flat(Re, laminar) * ff * Q * S_wet / S_ref


def wing_wetted(S_exposed: float, t_c: float) -> float:
    return S_exposed * (1.977 + 0.52 * t_c)


def body_wetted(length: float, width: float, height: float) -> float:
    """Raymer-style approximation for a fuselage of roughly elliptic sections: π·D_mean·L·(1 − 2/f)^(2/3)·(1 + 1/f²)."""
    D = math.sqrt(width * height)
    f = length / D
    return math.pi * D * length * (1 - 2 / f) ** (2 / 3) * (1 + 1 / f ** 2)


# =====================================================================================================================
# lift
# =====================================================================================================================
def oswald_straight(AR: float, k_fuse: float = 1.0) -> float:
    return k_fuse * (1.78 * (1 - 0.045 * AR ** 0.68) - 0.64)


def oswald_nita_scholz(AR: float, taper: float, sweep_c4_deg: float = 0.0, d_fuse_over_b: float = 0.0,
                       k_cd0: float = 0.804) -> float:
    """Nita & Scholz (2012) Oswald factor: e = e_theo · k_F · k_D0 (k_M = 1 at low Mach).
    e_theo = 1 / (1 + f(λ − Δλ)·AR), f(λ) = 0.0524λ⁴ − 0.15λ³ + 0.1659λ² − 0.0706λ + 0.0119,
    Δλ = −0.357 + 0.45·exp(0.0375·φ25); k_F = 1 − 2(d_F/b)²; k_D0 = 0.804 (general-aviation/turboprop class).
    The Raymer straight-wing fit underpredicts e for AR > 12; this is the preferred estimate."""
    lam = taper - (-0.357 + 0.45 * math.exp(0.0375 * sweep_c4_deg))
    f = 0.0524 * lam ** 4 - 0.15 * lam ** 3 + 0.1659 * lam ** 2 - 0.0706 * lam + 0.0119
    e_theo = 1.0 / (1.0 + f * AR)
    return e_theo * (1 - 2 * d_fuse_over_b ** 2) * k_cd0


def lift_slope(AR: float, sweep_half_chord_deg: float = 0.0, cla_2d: float = 2 * math.pi, M: float = 0.1) -> float:
    beta = math.sqrt(1 - M ** 2)
    eta = cla_2d / (2 * math.pi)
    t = math.tan(math.radians(sweep_half_chord_deg))
    return 2 * math.pi * AR / (2 + math.sqrt(4 + (AR * beta / eta) ** 2 * (1 + t ** 2 / beta ** 2)))


def k_induced(AR: float, e: float) -> float:
    return 1.0 / (math.pi * AR * e)


# =====================================================================================================================
# propulsion
# =====================================================================================================================
def power_lapse(sigma: float) -> float:
    """Gagg-Ferrar normally aspirated piston: P/P0 = 1.132 σ − 0.132 (EFI keeps mixture, still loses charge)."""
    return max(0.0, 1.132 * sigma - 0.132)


def prop_efficiency(V: float, P_shaft: float, D: float, rho: float, eta_max: float = 0.82, viscous: float = 0.86) -> float:
    """Ideal actuator-disk efficiency solved from P·η_i = T·V with T from momentum theory, times a viscous/profile
    factor, capped at ``eta_max``. Returns propulsive efficiency at airspeed V (V > 0)."""
    if V <= 0.1:
        return 0.0
    A = math.pi * D ** 2 / 4
    P = P_shaft * viscous
    # solve T from P = T(V + w), w from T = 2ρA w (V + w)  => iterate on w
    w = 1.0
    for _ in range(100):
        T = P / (V + w)
        w_new = 0.5 * (-V + math.sqrt(V * V + 2 * T / (rho * A)))
        if abs(w_new - w) < 1e-9:
            break
        w = 0.5 * (w + w_new)
    eta_i = V / (V + w)
    return min(eta_max, eta_i * viscous)


def static_thrust(P_shaft: float, D: float, rho: float = 1.225, fm: float = 0.70) -> float:
    """Momentum-theory static thrust with figure of merit ``fm``: T = (fm·P)^(2/3)·(2ρA)^(1/3)."""
    A = math.pi * D ** 2 / 4
    return (fm * P_shaft) ** (2 / 3) * (2 * rho * A) ** (1 / 3)


# =====================================================================================================================
# performance
# =====================================================================================================================
def power_required(V: float, W: float, S: float, cd0: float, k: float, rho: float) -> float:
    q = 0.5 * rho * V * V
    CL = W / (q * S)
    return q * S * (cd0 + k * CL * CL) * V


def stall_speed(W: float, S: float, clmax: float, rho: float = 1.225) -> float:
    return math.sqrt(2 * W / (rho * S * clmax))


def speeds(W: float, S: float, cd0: float, k: float, rho: float) -> dict:
    """Speed for (L/D)max (max range, propeller) and for minimum power (max endurance)."""
    v_ld = math.sqrt(2 * W / (rho * S) * math.sqrt(k / cd0))
    v_mp = math.sqrt(2 * W / (rho * S) * math.sqrt(k / (3 * cd0)))
    ld_max = 0.5 / math.sqrt(cd0 * k)
    return {"V_ld_max": v_ld, "V_min_power": v_mp, "LD_max": ld_max, "LD_min_power": ld_max * math.sqrt(3) / 2}


def max_level_speed(W, S, cd0, k, P_avail_fn, rho, v_lo=10.0, v_hi=120.0) -> float:
    """Highest V where P_avail(V) >= P_required(V) (bisection on the upper crossing)."""
    f = lambda V: P_avail_fn(V) - power_required(V, W, S, cd0, k, rho)
    vs = np.linspace(v_lo, v_hi, 400)
    vals = np.array([f(v) for v in vs])
    idx = np.where(vals > 0)[0]
    if not len(idx):
        return float("nan")
    i = idx[-1]
    if i == len(vs) - 1:
        return v_hi
    a, b = vs[i], vs[i + 1]
    for _ in range(60):
        m = 0.5 * (a + b)
        if f(m) > 0:
            a = m
        else:
            b = m
    return 0.5 * (a + b)


def rate_of_climb(W, S, cd0, k, P_avail_fn, rho, v_lo=10.0, v_hi=80.0) -> tuple[float, float]:
    """(max RoC, speed) from excess power."""
    vs = np.linspace(v_lo, v_hi, 300)
    roc = np.array([(P_avail_fn(v) - power_required(v, W, S, cd0, k, rho)) / W for v in vs])
    i = int(np.argmax(roc))
    return float(roc[i]), float(vs[i])


def breguet_endurance_prop(eta_p: float, bsfc_kg_per_J: float, CL: float, CD: float, rho: float, S: float,
                           W0: float, W1: float) -> float:
    """Endurance (s) of a propeller aircraft flown at constant CL (Breguet): E = (η/c)·(CL^1.5/CD)·sqrt(2ρS)·(W1^-½ − W0^-½)
    with c = BSFC·g (1/m)."""
    c = bsfc_kg_per_J * G0
    return eta_p / c * CL ** 1.5 / CD * math.sqrt(2 * rho * S) * (W1 ** -0.5 - W0 ** -0.5)


def breguet_range_prop(eta_p: float, bsfc_kg_per_J: float, LD: float, W0: float, W1: float) -> float:
    c = bsfc_kg_per_J * G0
    return eta_p / c * LD * math.log(W0 / W1)


def takeoff_ground_roll(W, S, clmax_to, cd0_to, k, T_static, T_lof, mu=0.04, rho=1.225, cl_roll=0.6,
                        v_lof_factor=1.1) -> dict:
    """Raymer 17.8-17.10: S_G = 1/(2 g K_A) ln((K_T + K_A V_f²)/(K_T + K_A V_i²)) with linear thrust variation."""
    VS = stall_speed(W, S, clmax_to, rho)
    VLO = v_lof_factor * VS
    T_avg = 0.5 * (T_static + T_lof)
    KT = T_avg / W - mu
    KA = rho / (2 * W / S) * (mu * cl_roll - cd0_to - k * cl_roll ** 2)
    sg = 1 / (2 * G0 * KA) * math.log((KT + KA * VLO ** 2) / KT)
    return {"ground_roll": sg, "V_lof": VLO, "VS_to": VS}


def landing_roll(W, S, clmax_ld, cd0_ld, k, mu_brake=0.3, rho=1.225, v_td_factor=1.15, cl_roll=0.3,
                 t_free=1.0) -> dict:
    VS = stall_speed(W, S, clmax_ld, rho)
    VTD = v_td_factor * VS
    KT = -mu_brake
    KA = rho / (2 * W / S) * (mu_brake * cl_roll - cd0_ld - k * cl_roll ** 2)
    sg = 1 / (2 * G0 * KA) * math.log(KT / (KT + KA * VTD ** 2))
    return {"ground_roll": sg + VTD * t_free, "V_td": VTD, "VS_ld": VS}
