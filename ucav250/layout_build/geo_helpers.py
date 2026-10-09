"""Scratch: geometry probes for the layout design (body sections, wing/fin/stab points, hinge lines)."""
import sys, math
sys.path.insert(0, '/home/user/Drone-controller')
import numpy as np
from ucav250.core import spec as SPEC
from ucav250.analysis import sizing as Z
from ucav250.design import oml

S = SPEC.load()
af = Z.Airframe(S)


def surf_point(srf, eta, xc, frac=0.5, n=161):
    """Point at chord fraction xc on the exact loft at span coordinate eta; frac 0 = lower, 1 = upper, 0.5 = mid."""
    loop = srf.loop_at(float(eta), n)
    up = loop[:n][::-1]          # LE -> TE upper
    lo = loop[n - 1:]            # LE -> TE lower
    o, c, u, _ = srf.frame_at(float(eta))
    ch = srf.chord_at(float(eta))

    def at(P):
        s = (P - o) @ c / ch
        return np.array([np.interp(xc, s, P[:, k]) for k in range(3)])
    pu, pl = at(up), at(lo)
    return pl + frac * (pu - pl)


if __name__ == "__main__":
    W = af.wing
    print("wing span coords", W.span_coords()[[0, 8, -1]])
    for eta in (0.40, 0.455, 0.55, 0.68, 0.70, 0.73, 2.16, 3.42, 3.55):
        e = eta if eta <= 0.7 else 0.7 + (eta - 0.7) / math.cos(math.radians(3))
        for xc in (0.25, 0.72, 0.75):
            print(f"wing y~{eta:.3f} xc {xc}: up {surf_point(W, e, xc, 1)} lo {surf_point(W, e, xc, 0)}")
    F = af.tail["fin"]
    print("fin span coords", F.span_coords())
    for eta in (0.12, 0.25, 0.30, 0.35, 0.95, 1.02):
        for xc in (0.0, 0.25, 0.65, 0.70, 1.0):
            print(f"fin eta {eta}: xc {xc} mid {surf_point(F, eta, xc)}")
    for x in (3.27, 3.48, 3.67, 3.74, 3.89):
        for y in (0.12, 0.15, 0.18, 0.20):
            print(f"x {x} y {y} ztop {af.z_top(x, y):.4f} zbot {af.z_bot(x, y):.4f}")
