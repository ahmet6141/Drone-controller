"""Longitudinal/directional static stability library for ucav250 (SI units). Aircraft-independent, unit tested.

Methods:
* Tail volume coefficients V_H = S_h·l_h/(S·c̄), V_V = S_v·l_v/(S·b).
* V-tail / inverted-V decomposition: horizontal and vertical projected areas S·cos²Γ and S·sin²Γ (Purser-Campbell
  NACA 823 effective-area approach), lift slopes from the panel aspect ratio.
* Downwash gradient at the tail: DATCOM/Roskam  dε/dα = 4.44·[K_A·K_λ·K_H·sqrt(cos Λc/4)]^1.19.
* Fuselage destabilising moment: Gilruth/Raymer  Cmα_fus = K_f·W_f²·L_f/(c̄·S) per rad (K_f per rad from the
  wing-root quarter-chord position, Raymer Fig. 16.14 fit).
* Neutral point (stick fixed):  x_np = [a_w·x_ac,w + η·a_t·(S_t/S)·(1 − dε/dα)·x_ac,t − Cmα_fus·c̄] /
  [a_w + η·a_t·(S_t/S)·(1 − dε/dα)]  (all x in the aircraft frame, aft positive).
* Static margin SM = (x_np − x_cg)/c̄.
"""
from __future__ import annotations

import math


def tail_volumes(S: float, mac: float, b: float, S_h: float, l_h: float, S_v: float, l_v: float) -> dict:
    return {"V_H": S_h * l_h / (S * mac), "V_V": S_v * l_v / (S * b)}


def v_tail_projection(S_panels_total: float, dihedral_deg: float) -> dict:
    """Effective horizontal/vertical areas of a V (or inverted V) tail with total panel area ``S_panels_total``."""
    g = math.radians(dihedral_deg)
    return {"S_h_eff": S_panels_total * math.cos(g) ** 2, "S_v_eff": S_panels_total * math.sin(g) ** 2}


def downwash_gradient(AR: float, taper: float, l_h: float, h_h: float, b: float, sweep_c4_deg: float = 0.0) -> float:
    """dε/dα at the tail. ``l_h`` = longitudinal distance wing MAC quarter chord → tail MAC quarter chord, ``h_h`` =
    vertical distance of the tail above the wing root chord plane (absolute value used)."""
    KA = 1 / AR - 1 / (1 + AR ** 1.7)
    Kl = (10 - 3 * taper) / 7
    KH = (1 - abs(h_h) / b) / (2 * l_h / b) ** (1 / 3)
    return 4.44 * (KA * Kl * KH * math.sqrt(math.cos(math.radians(sweep_c4_deg)))) ** 1.19


def kf_fuselage(x_c4_root_frac_of_length: float) -> float:
    """K_f (per rad) vs wing-root quarter-chord position as a fraction of fuselage length. Raymer Fig. 16.14 per-deg
    chart (0.1 → 0.0015, 0.3 → 0.0050, 0.5 → 0.015, 0.6 → 0.028 per deg) fitted as an exponential, times 57.3."""
    x = min(max(x_c4_root_frac_of_length, 0.1), 0.6)
    return 57.2958 * 0.00078 * math.exp(5.95 * x)


def cm_alpha_fuselage(Kf: float, W_f: float, L_f: float, mac: float, S: float) -> float:
    """Destabilising fuselage pitching-moment slope (per rad, positive = destabilising)."""
    return Kf * W_f ** 2 * L_f / (mac * S)


def neutral_point(a_w: float, x_ac_w: float, a_t: float, S_t: float, x_ac_t: float, S: float, mac: float,
                  deps_da: float, eta_t: float = 0.9, cm_alpha_fus: float = 0.0) -> float:
    k = eta_t * a_t * (S_t / S) * (1 - deps_da)
    return (a_w * x_ac_w + k * x_ac_t - cm_alpha_fus * mac) / (a_w + k)


def static_margin(x_np: float, x_cg: float, mac: float) -> float:
    return (x_np - x_cg) / mac


def cn_beta_vertical(a_v: float, S_v: float, l_v: float, S: float, b: float, eta_v: float = 0.9,
                     dsig_dbeta: float = 0.1) -> float:
    """Weathercock stability contribution of the fin (per rad): Cnβ_v = η_v·a_v·(S_v·l_v)/(S·b)·(1 + dσ/dβ)."""
    return eta_v * a_v * S_v * l_v / (S * b) * (1 + dsig_dbeta)


def cn_beta_fuselage(K_N: float, K_Rl: float, S_side: float, L_f: float, S: float, b: float) -> float:
    """Raymer/DATCOM body directional instability (per rad): Cnβ_fus = −57.3·K_N·K_Rl·(S_B,S/S)·(L_f/b)."""
    return -57.3 * K_N * K_Rl * (S_side / S) * (L_f / b)
