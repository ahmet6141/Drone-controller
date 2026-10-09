"""Conceptual sizing library for propeller-driven UAVs (SI units). Aircraft-independent and unit tested; the concept
studies and ``analysis/sizing.py`` share it so every concept is judged with the same equations.

Contents
* Constraint analysis (Gudmundsson ch. 3 / Raymer ch. 5 forms, written as required shaft power per unit weight
  P/W [W/N] at sea level for a given wing loading W/S [N/m²], corrected for altitude power lapse and propeller
  efficiency): cruise speed, sustained climb, service ceiling, take-off ground run, sustained turn; wing-loading
  limits from stall speed and landing ground run.
* Mission fuel: segment weight fractions (warm-up/taxi/take-off fixed fractions, climb by energy, cruise by Breguet
  range, loiter by Breguet endurance, reserve).
* Take-off mass iteration: MTOW = payload + fixed equipment + fuel(MTOW) + structure/airframe(MTOW) with an
  empty-mass-fraction regression ``m_e/m0 = A·m0^C`` fitted to comparables.
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field

import numpy as np

from . import aerolib as AL

G0 = AL.G0


# =====================================================================================================================
# constraint analysis  (returns P/W in W/N = shaft power per newton of weight at sea level, full throttle)
# =====================================================================================================================
@dataclass
class Aero:
    cd0: float
    k: float
    clmax: float
    clmax_to: float
    clmax_ld: float
    cd0_to: float | None = None


def _pw(ws, V, rho, a: Aero, n=1.0, roc=0.0, eta=0.8, lapse=1.0, cd0=None):
    q = 0.5 * rho * V * V
    cd0 = a.cd0 if cd0 is None else cd0
    tw = q * cd0 / ws + a.k * n * n * ws / q + roc / V
    return tw * V / (eta * lapse)


def pw_cruise(ws, V, h, a: Aero, eta=0.8, throttle=1.0):
    """P/W to cruise at TAS V and altitude h using ``throttle`` (fraction of the available power at h)."""
    atm = AL.isa(h)
    return _pw(ws, V, atm["rho"], a, eta=eta, lapse=AL.power_lapse(atm["sigma"]) * throttle)


def pw_climb(ws, roc, h, a: Aero, eta=0.75, V=None):
    """P/W for a sustained rate of climb ``roc`` at altitude h; V defaults to the speed for minimum power x 1.1."""
    atm = AL.isa(h)
    rho = atm["rho"]
    ws = np.asarray(ws, float)
    if V is None:
        V = 1.1 * np.sqrt(2 * ws / rho * np.sqrt(a.k / (3 * a.cd0)))
    return _pw(ws, V, rho, a, roc=roc, eta=eta, lapse=AL.power_lapse(atm["sigma"]))


def pw_ceiling(ws, h_ceiling, a: Aero, eta=0.72, roc=0.5):
    """Service ceiling: 0.5 m/s rate of climb at ``h_ceiling``."""
    return pw_climb(ws, roc, h_ceiling, a, eta=eta)


def pw_turn(ws, V, h, n, a: Aero, eta=0.8):
    atm = AL.isa(h)
    return _pw(ws, V, atm["rho"], a, n=n, eta=eta, lapse=AL.power_lapse(atm["sigma"]))


def pw_takeoff(ws, s_g, a: Aero, h=0.0, mu=0.04, eta_to=0.55, k_lof=1.1, cl_roll=0.6):
    """Gudmundsson eq. 3-11 form: T/W = V_LOF²/(2 g S_G) + q·CD_TO/(W/S) + μ(1 − q·CL_TO/(W/S)) with q at
    V_LOF/√2; P/W = T/W · (V_LOF/√2) / η_TO (η_TO: average propulsive efficiency during the run)."""
    atm = AL.isa(h)
    rho = atm["rho"]
    ws = np.asarray(ws, float)
    vlof = k_lof * np.sqrt(2 * ws / (rho * a.clmax_to))
    q = 0.5 * rho * (vlof / math.sqrt(2)) ** 2
    cd_to = (a.cd0_to if a.cd0_to is not None else a.cd0 + 0.01) + a.k * cl_roll ** 2
    tw = vlof ** 2 / (2 * G0 * s_g) + q * cd_to / ws + mu * (1 - q * cl_roll / ws)
    return tw * (vlof / math.sqrt(2)) / (eta_to * AL.power_lapse(atm["sigma"]))


def ws_stall(v_stall, clmax, h=0.0):
    return 0.5 * AL.isa(h)["rho"] * v_stall ** 2 * clmax


def ws_landing(s_ground, clmax_ld, mu_brake=0.3, h=0.0, k_td=1.15, t_free=1.0):
    """Largest W/S whose landing ground roll (Raymer energy method incl. ``t_free`` s free roll) ≤ ``s_ground``."""
    rho = AL.isa(h)["rho"]
    lo, hi = 50.0, 3000.0
    for _ in range(80):
        m = 0.5 * (lo + hi)
        vtd = k_td * math.sqrt(2 * m / (rho * clmax_ld))
        s = vtd ** 2 / (2 * G0 * mu_brake) + vtd * t_free
        lo, hi = (m, hi) if s <= s_ground else (lo, m)
    return lo


# =====================================================================================================================
# mission fuel
# =====================================================================================================================
@dataclass
class Segment:
    kind: str                       # warmup | taxi | takeoff | climb | cruise | loiter | descent | landing | reserve
    value: float = 0.0              # cruise: range m; loiter: s; climb: altitude gain m; fixed: weight fraction
    V: float = 0.0                  # TAS m/s (cruise/loiter)
    h: float = 0.0
    LD: float = 0.0                 # lift-to-drag at the flown point
    eta: float = 0.8
    bsfc: float = 0.0               # kg/J at the flown power setting
    gamma: float = 0.08             # climb: average flight-path gradient (drag work over the climb distance)
    name: str = ""
    fraction: float | None = None   # W_end/W_start integrated by the caller (direct fuel-flow integration); overrides
                                    # the closed-form Breguet/energy fraction when given


FIXED = {"warmup": 0.996, "taxi": 0.997, "takeoff": 0.995, "descent": 0.998, "landing": 0.997}


def segment_fraction(s: Segment, W: float) -> float:
    """W_end/W_start for one segment (``s.fraction`` when the caller integrated the fuel flow itself)."""
    if s.fraction is not None:
        return float(s.fraction)
    if s.kind in FIXED:
        return s.value if s.value else FIXED[s.kind]
    c = s.bsfc * G0                                   # 1/m
    if s.kind == "climb":                             # energy height gain + drag work over the climb distance
        dhe = s.value + 0.5 * s.V ** 2 / G0
        return math.exp(-c * dhe * (1 + 1 / (s.gamma * s.LD)) / s.eta)
    if s.kind == "cruise":
        return math.exp(-s.value * c / (s.eta * s.LD))
    if s.kind in ("loiter", "reserve"):
        return math.exp(-s.value * c * s.V / (s.eta * s.LD))
    raise ValueError(f"unknown segment {s.kind}")


def mission_fuel_fraction(segments: list[Segment], trapped: float = 0.02) -> tuple[float, list[float]]:
    """Total fuel mass fraction m_fuel/m0 (with ``trapped`` unusable fuel share) and the per-segment fractions."""
    w = 1.0
    fr = []
    for s in segments:
        f = segment_fraction(s, w)
        fr.append(f)
        w *= f
    return (1 - w) * (1 + trapped), fr


# =====================================================================================================================
# take-off mass iteration
# =====================================================================================================================
def fit_empty_fraction(m0: np.ndarray, me: np.ndarray) -> tuple[float, float]:
    """Least-squares fit of ``me/m0 = A·m0^C`` in log space. Returns (A, C)."""
    m0 = np.asarray(m0, float)
    f = np.asarray(me, float) / m0
    C, lnA = np.polyfit(np.log(m0), np.log(f), 1)
    return float(math.exp(lnA)), float(C)


@dataclass
class MassModel:
    payload: float                  # kg (mission payload, EO/IR + mission equipment)
    fixed: float                    # kg (engine installed, avionics, servos, generator... known equipment)
    fuel_fraction: float            # m_fuel/m0 from the mission
    airframe_fraction_fn: object    # m0 -> airframe (structure + shell + gear + controls) mass fraction
    notes: list = field(default_factory=list)

    def solve(self, m0_guess=100.0, tol=1e-6, it=200) -> dict:
        m0 = m0_guess
        for _ in range(it):
            af = float(self.airframe_fraction_fn(m0))
            denom = 1 - self.fuel_fraction - af
            if denom <= 0.05:
                raise ValueError("mass iteration diverges: fuel + airframe fractions >= 0.95")
            new = (self.payload + self.fixed) / denom
            if abs(new - m0) < tol:
                m0 = new
                break
            m0 = new
        af = float(self.airframe_fraction_fn(m0))
        return {"mtow": m0, "fuel": self.fuel_fraction * m0, "airframe": af * m0, "payload": self.payload,
                "fixed": self.fixed, "empty": m0 - self.fuel_fraction * m0 - self.payload}


def design_point(ws_grid: np.ndarray, curves: dict[str, np.ndarray], ws_max: float, margin: float = 1.05) -> dict:
    """Pick the design point: the minimum-power W/S within the wing-loading limit. ``curves`` maps constraint
    names to P/W arrays over ``ws_grid``. Returns W/S, required P/W (envelope x margin) and the active constraint."""
    ws_grid = np.asarray(ws_grid, float)
    env = np.max(np.vstack(list(curves.values())), axis=0)
    ok = ws_grid <= ws_max
    if not np.any(ok):
        raise ValueError("no wing loading satisfies the stall/landing limits")
    i = int(np.argmin(np.where(ok, env, np.inf)))
    active = max(curves, key=lambda k: curves[k][i])
    return {"ws": float(ws_grid[i]), "pw": float(env[i] * margin), "active": active, "envelope": env}
