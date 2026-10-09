"""Layout builder (scratch): shared helpers and geometry anchors computed from the closed sizing geometry.

The layout values written to spec.yaml are authored interface decisions; anchors that must coincide with the sizing
geometry (spar lines, gear pivots, turret centre, fuel cells, engine envelope) are taken from the spec here and rounded,
and ucav250.analysis.layout_check verifies them against the geometry with stated tolerances."""
from __future__ import annotations

import copy
import math
import sys

sys.path.insert(0, "/home/user/Drone-controller")
import numpy as np  # noqa: E402

from ucav250.analysis import sizing as Z  # noqa: E402
from ucav250.core import spec as SPEC  # noqa: E402

SPEC.load.cache_clear()
S = copy.deepcopy(SPEC.load())
af = Z.Airframe(S)
P = S["wing"]["planform"]
T = Z.trapezoid(P)
LG = S["landing_gear"]
TL = S["tail"]["surfaces"]
ZP = S["layout"]["zones_preliminary"]
FC = {c["name"]: c for c in S["layout"]["fuel_cells"]}
PR = S["propeller"]
ENG = S["engine"]
TU = S["payload"]["turret"]


def r3(v, n=4):
    """Round a scalar or a nested list to n decimals (plain python floats)."""
    if isinstance(v, (list, tuple, np.ndarray)):
        return [r3(x, n) for x in v]
    return float(round(float(v), n))


def half_width(x, z):
    return float(af.half_width(np.atleast_1d(float(x)), float(z))[0])


def z_top(x, y=0.0):
    return float(af.z_top(float(x), float(y)))


def z_bot(x, y=0.0):
    return float(af.z_bot(float(x), float(y)))


def zc(x):
    return float(af.sec(float(x))[3][0])


def chine_halfwidth(x):
    return float(af.sec(float(x))[0][0])


def spar_x(y, frac):
    return float(Z.spar_x(P, abs(float(y)), frac))


def sweep_deg(frac):
    """Plan sweep of a constant-chord-fraction line of the reference trapezoid (deg)."""
    return math.degrees(math.atan2(spar_x(1.0, frac) - spar_x(0.0, frac), 1.0))


def surf_point(srf, eta, xc, frac=0.5, n=161):
    """Point at chord fraction xc on the exact loft at span coordinate eta; frac 0 lower, 1 upper, 0.5 mid."""
    loop = srf.loop_at(float(eta), n)
    up = loop[:n][::-1]
    lo = loop[n - 1:]
    o, c, u, _ = srf.frame_at(float(eta))
    ch = srf.chord_at(float(eta))

    def at(Q):
        s = (Q - o) @ c / ch
        return np.array([np.interp(xc, s, Q[:, k]) for k in range(3)])
    pu, pl = at(up), at(lo)
    return pl + frac * (pu - pl)


def wing_eta(y):
    """Span coordinate of the wing loft for a starboard station y (flat centre section, 3 deg dihedral outboard)."""
    yj = float(P["y_junction"])
    return float(y) if y <= yj else yj + (float(y) - yj) / math.cos(math.radians(float(P["dihedral_deg"])))


def wing_pt(y, xc, frac=0.5):
    return surf_point(af.wing, wing_eta(y), xc, frac)


# ------------------------------------------------------------------------------------------------ anchors
X_MS0 = spar_x(0.0, float(P["main_spar_frac"]))           # main spar line at the centre line
X_RS0 = spar_x(0.0, float(P["rear_spar_frac"]))
SW_MS = sweep_deg(float(P["main_spar_frac"]))
SW_RS = sweep_deg(float(P["rear_spar_frac"]))
YJ = float(P["y_junction"])
Y_SOB = 0.400                                              # side-of-body rib (body half width on the chine)
BOX = ZP["wing_carry_through"]["box"]
Z_BOX = (float(BOX[0][2]), float(BOX[1][2]))
X_FW = float(S["layout"]["firewall_x"])
PIV = TL["stabilator"]["pivot"]
X_RING = float(PIV[0])
HUB = PR["hub"]
EPS = math.radians(float(PR["thrust_line_inclination_deg"]))
D_THRUST = np.array([math.cos(EPS), 0.0, math.sin(EPS)])  # crank / thrust axis, pointing aft
MG = LG["main"]
NG = LG["nose"]
X_TUR = float(TU["bay_center_x"])
FIN = TL["fin"]
STUB = TL["stabilator_stub"]
VEN = TL["ventral"]
STAB = TL["stabilator"]


PB = ZP["payload_bay"]["box"]
X_PB0, X_PB1 = float(PB[0][0]), float(PB[1][0])
WB = ZP["main_gear_wells"]["box"]
X_W0, X_W1 = float(WB[0][0]), float(WB[1][0])
X_FUELF = round(float(FC["forward_cell"]["x"][0]) - 0.0275, 4)    # forward fuel-bay bulkhead (FS-FUEL)
X_GEARF = round(X_W1 + 0.0124, 4)                                  # aft main-gear frame / aft fuel bulkhead (FS-GEAR)
X_DUCT = round(0.5 * (X_PB1 + X_W0), 4)                            # lateral harness duct between payload bay and wells
T_WALL = 0.0068
PARA_X = [1.500, 1.800]                     # parachute bay (layout.rules.boxes.parachute_bay, unchanged)
X_PFR = round(PARA_X[1] + 0.0100, 4)        # parachute-bay aft bulkhead / fwd bridle fitting / LERX apex frame
X_PFF = round(PARA_X[0] - 0.0100, 4)        # parachute-bay forward bulkhead
FWD_BAY_X = [0.380, 0.596]                  # forward equipment bay (FS0300-FS0600): buffer battery + FTS unit
Z_TROOF = round(float(ZP['turret_bay']['box'][1][2]) + 0.002, 4)   # turret-bay roof (above the mechanism)


def engine_point(dist_ahead_of_plane, dy=0.0, dz=0.0):
    """Point on/around the crank axis ``dist_ahead_of_plane`` (m) ahead of the propeller plane centre along the
    inclined thrust axis, offset by (dy, dz) in the engine's own lateral/vertical axes."""
    h = np.array(HUB, float)
    up = np.array([-math.sin(EPS), 0.0, math.cos(EPS)])
    return h - dist_ahead_of_plane * D_THRUST + dy * np.array([0.0, 1.0, 0.0]) + dz * up


HUB_FACE_AHEAD = float(PR["hub_spacer"]) + float(PR["hub_half_thickness"])     # prop plane -> engine flange
MOUNT_FACE_AHEAD = HUB_FACE_AHEAD + 0.1814                                       # engine.yaml datum distance
SG750_FRONT_AHEAD = HUB_FACE_AHEAD + float(ENG["envelope"]["length_with_sg750"])

if __name__ == "__main__":
    print("X_MS0", X_MS0, "X_RS0", X_RS0, "sweeps", SW_MS, SW_RS, "Z_BOX", Z_BOX, "FW", X_FW, "ring", X_RING)
    print("mount face", engine_point(MOUNT_FACE_AHEAD), "SG750 front", engine_point(SG750_FRONT_AHEAD),
          "hub face", engine_point(HUB_FACE_AHEAD))
    for x in (0.6, 1.11, 1.33, 1.49, 1.81, 2.215, 2.5125, 2.826, 3.12, 3.48, 3.67, 3.7375):
        print(f"x {x}: chine hw {chine_halfwidth(x):.4f} zc {zc(x):.4f} ztop {z_top(x):.4f} zbot {z_bot(x):.4f}")
