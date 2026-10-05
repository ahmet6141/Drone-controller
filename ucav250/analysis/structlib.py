"""Hand-calculation library for ucav250 structures (SI units throughout). Aircraft-independent and unit tested.

References (standard aeronautical practice):
* Gust load factor: CS-VLA 341 / CS-23.341 (Pratt formula) — n = 1 ± kg·ρ0·Ude·V·a / (2·W/S),
  kg = 0.88·μ / (5.3 + μ), μ = 2·(W/S) / (ρ·c̄·a·g).
* Schrenk approximation for spanwise lift (average of elliptic and planform-chord distributions).
* Lug static strength: Melcon & Hoblit / Bruhn D1 simplified (net-section tension and shear-out/bearing with
  conservative efficiency factors); fitting factor applied separately.
* Margins of safety: MS = allowable / (FoS · fitting · applied) − 1 (limit loads in, ultimate allowables).
"""
from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

RHO0 = 1.225
G0 = 9.80665


# =====================================================================================================================
# flight envelope
# =====================================================================================================================
def gust_factor(ws_npm2: float, cla_per_rad: float, mac: float, rho: float = RHO0) -> float:
    mu = 2.0 * ws_npm2 / (rho * mac * cla_per_rad * G0)
    return 0.88 * mu / (5.3 + mu)


def gust_n(V: float, Ude: float, ws_npm2: float, cla_per_rad: float, mac: float, rho: float = RHO0) -> tuple:
    """(n_up, n_down) for an equivalent airspeed V (m/s) and derived gust velocity Ude (m/s)."""
    kg = gust_factor(ws_npm2, cla_per_rad, mac, rho)
    dn = kg * RHO0 * Ude * V * cla_per_rad / (2.0 * ws_npm2)
    return 1.0 + dn, 1.0 - dn


def vn_diagram(ws_npm2: float, clmax: float, clmin: float, n_pos: float, n_neg: float, VC: float, VD: float,
               cla_per_rad: float, mac: float, Ude_C: float = 15.24, Ude_D: float = 7.62, rho: float = RHO0) -> dict:
    """Manoeuvre + gust envelope (EAS). Returns key speeds and the governing positive/negative load factors."""
    VS = math.sqrt(2 * ws_npm2 / (rho * clmax))
    VA = VS * math.sqrt(n_pos)
    VSneg = math.sqrt(2 * ws_npm2 / (rho * abs(clmin)))
    VG = VSneg * math.sqrt(abs(n_neg))
    gc = gust_n(VC, Ude_C, ws_npm2, cla_per_rad, mac, rho)
    gd = gust_n(VD, Ude_D, ws_npm2, cla_per_rad, mac, rho)
    n_up = max(n_pos, gc[0], gd[0])
    n_dn = min(n_neg, gc[1], gd[1])
    return {"VS": VS, "VA": VA, "VG": VG, "VC": VC, "VD": VD, "gust_C": gc, "gust_D": gd,
            "n_limit_pos": n_up, "n_limit_neg": n_dn, "kg": gust_factor(ws_npm2, cla_per_rad, mac, rho)}


# =====================================================================================================================
# spanwise loads
# =====================================================================================================================
def schrenk(y: np.ndarray, chord: np.ndarray, semispan: float) -> np.ndarray:
    """Normalised spanwise lift per unit span ``l(y)`` with ∫ l dy over the half span = 1 (Schrenk average of
    planform chord and elliptic distributions). ``y`` from root (0) to tip (semispan)."""
    y = np.asarray(y, float)
    c = np.asarray(chord, float)
    area = np.trapz(c, y)
    ell = 4.0 * area / (math.pi * semispan) * np.sqrt(np.clip(1 - (y / semispan) ** 2, 0, None))
    l = 0.5 * (c + ell)
    return l / np.trapz(l, y)


def beam_loads(y: np.ndarray, w: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """Shear V(y) and bending M(y) of a cantilever (root at y[0], free tip at y[-1]) under distributed load w(y)
    (N/m, positive up). Returns arrays at the same stations (V positive up outboard of the cut)."""
    y = np.asarray(y, float)
    w = np.asarray(w, float)
    V = np.zeros_like(y)
    M = np.zeros_like(y)
    for i in range(len(y) - 2, -1, -1):
        dy = y[i + 1] - y[i]
        V[i] = V[i + 1] + 0.5 * (w[i] + w[i + 1]) * dy
        M[i] = M[i + 1] + 0.5 * (V[i] + V[i + 1]) * dy
    return V, M


# =====================================================================================================================
# sections
# =====================================================================================================================
def tube(ro: float, ri: float) -> dict:
    A = math.pi * (ro ** 2 - ri ** 2)
    I = math.pi / 4 * (ro ** 4 - ri ** 4)
    return {"A": A, "I": I, "J": 2 * I, "Z": I / ro, "c": ro}


def rect(b: float, h: float) -> dict:
    return {"A": b * h, "I": b * h ** 3 / 12, "Z": b * h ** 2 / 6, "c": h / 2}


def spar_caps(h_eff: float, cap_area: float) -> dict:
    """Idealised I/box spar: two caps of ``cap_area`` separated by ``h_eff`` (centroid distance)."""
    return {"I": 2 * cap_area * (h_eff / 2) ** 2, "c": h_eff / 2}


# =====================================================================================================================
# joints and fittings
# =====================================================================================================================
def ms(allowable: float, applied_limit: float, fos: float = 1.5, fitting: float = 1.0) -> float:
    """Margin of safety for a limit load against an ultimate allowable."""
    if applied_limit <= 0:
        return math.inf
    return allowable / (fos * fitting * applied_limit) - 1.0


def bolt_shear(d: float, Fsu: float, planes: int = 1) -> float:
    return planes * math.pi / 4 * d ** 2 * Fsu


def bearing(d: float, t: float, Fbru: float) -> float:
    return d * t * Fbru


def lug_axial(Ftu: float, Fbru_or_Ftu: float, w: float, D: float, t: float, e: float) -> dict:
    """Simplified static strength of a single lug loaded axially along its centre line (pin hole D, width w, edge
    distance e from hole centre to the lug end, thickness t). Net-section tension with efficiency Kt (0.9 for
    D/w <= 0.5 decreasing) and shear-bearing with Kbr from e/D (Bruhn D1.5 trend, conservative fit)."""
    Kt = max(0.7, 1.0 - 0.3 * max(0.0, D / w - 0.3) / 0.4)
    P_net = Kt * Ftu * (w - D) * t
    eD = e / D
    Kbr = float(np.clip(0.9 * (eD - 0.5), 0.15, 1.8))
    P_br = Kbr * Fbru_or_Ftu * D * t
    return {"P_net": P_net, "P_shear_bearing": P_br, "P_allow": min(P_net, P_br), "Kt": Kt, "Kbr": Kbr}


def euler_buckling(E: float, I: float, L: float, K: float = 1.0) -> float:
    return math.pi ** 2 * E * I / (K * L) ** 2


def thin_plate_shear_buckling(E: float, t: float, b: float, nu: float = 0.33, ks: float = 5.35) -> float:
    """Critical shear stress of a long flat panel of width b with simply supported edges."""
    return ks * math.pi ** 2 * E / (12 * (1 - nu ** 2)) * (t / b) ** 2


@dataclass
class Margin:
    item: str
    load: str
    applied: float
    allowable: float
    fos: float = 1.5
    fitting: float = 1.0
    unit: str = "N"
    ref: str = ""

    @property
    def ms(self) -> float:
        return ms(self.allowable, self.applied, self.fos, self.fitting)

    def row(self) -> dict:
        return {"item": self.item, "load": self.load, "applied": self.applied, "allowable": self.allowable,
                "fos": self.fos, "fitting": self.fitting, "ms": self.ms, "unit": self.unit, "ref": self.ref}
