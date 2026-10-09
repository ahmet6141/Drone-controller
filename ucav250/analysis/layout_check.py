"""Layout-phase checks of the interface definition in ``spec.layout`` / ``spec.assembly`` (ARCHITECTURE.md §9).

The layout section is the contract the detail modules (chassis, wing, tail, shell, propulsion, fuel, gear, systems,
payload) build from without reading each other's geometry. This module verifies it on simplified geometry
(boxes, capsules, cylinders, spheres; the OML of the body and the wing loft from ``sizing.Airframe``):

* ids and part numbering (convention, module ranges, uniqueness, references between layout objects);
* stations: materials/processes, thickness, cut-outs inside the section, frames do not cut fuel / turret / gear /
  payload / parachute volumes, every harness trunk, push-rod and bridle crossing of a frame passes a declared cut-out;
* every envelope inside the OML with its clearance;
* no overlaps between contents (equipment, fuel cells, harness, turret, stowed gear, engine, mount, exhaust) and
  between contents and structure (members, fittings) except declared mounts;
* mechanism swept volumes (gear + doors sequence, turret + doors sequence, control surfaces, parachute hatch,
  propeller disc) against everything around them;
* mass placement: centroids recomputed from the layout = ``layout.mass_placement`` = spec mass items; CG shift;
* turret field of regard (external protrusions above the -5 deg cone) and RF windows of the antennas;
* keep-out rules (battery-fuel, fuel-firewall, engine envelope, hot zones, exhaust, propeller disc);
* shell (edge margins, RF materials, hinges), maintenance reachability through removable panels, assembly list,
  transport sizes, mechanism definitions;
* first-cut pre-sizing of the interface fittings (wing joint pins, parachute bridle fittings, engine mount bolts).

Outputs: ``out/layout.md`` (Turkish), ``out/layout.json`` and the figures ``docs/fig/yk250_layout_*.png``.

CLI:  python3 -m ucav250.analysis.layout_check [--check] [--no-figures] [--out DIR] [--fig-dir DIR]
      (--check: exit 1 if any check fails; nothing is written to tracked files with --check unless --write is given)
"""
from __future__ import annotations

import argparse
import copy
import json
import math
import re
import sys
import time
from pathlib import Path

import numpy as np

from ..core import spec as SPEC
from . import sizing as Z

OUT_DIR = SPEC.OUT_DIR
FIG_DIR = Z.FIG_DIR
G0 = 9.80665
ID_RE = re.compile(r"^YK250-(CH|SH|WG|TL|FC|PR|FU|LG|SY|PL|HW)-(\d{3})(-[LR])?$")
STATION_TYPES = ("bulkhead", "ring", "fitting frame")


# =====================================================================================================================
# geometry primitives (signed distance + surface samples)
# =====================================================================================================================
# parachute opening shock used for the bridle fittings (layout.chassis.design_loads DL-PARACHUTE): Galaxy GRS 4/240
# published value at 240 kg / 240 km/h (components.yaml), larger than UAVOS 200 5 g x MTOM; ultimate-only (CRASH-004)
PARA_OPEN_N = 13100.0


def _unit(v):
    v = np.asarray(v, float)
    return v / max(float(np.linalg.norm(v)), 1e-15)


def _rot(axis, ang):
    """Rotation matrix about a unit axis (Rodrigues)."""
    a = _unit(axis)
    K = np.array([[0, -a[2], a[1]], [a[2], 0, -a[0]], [-a[1], a[0], 0]])
    return np.eye(3) + math.sin(ang) * K + (1 - math.cos(ang)) * (K @ K)


class OBB:
    """Oriented box: centre c, axes R (columns), half extents h."""

    def __init__(self, c, R, h):
        self.c, self.R, self.h = np.asarray(c, float), np.asarray(R, float), np.maximum(np.asarray(h, float), 1e-6)

    @classmethod
    def aabb(cls, box):
        b = np.asarray(box, float)
        lo, hi = np.minimum(b[0], b[1]), np.maximum(b[0], b[1])
        return cls(0.5 * (lo + hi), np.eye(3), 0.5 * (hi - lo))

    def sdf(self, P):
        q = np.abs((np.atleast_2d(P) - self.c) @ self.R) - self.h
        out = np.linalg.norm(np.maximum(q, 0.0), axis=1)
        return out + np.minimum(q.max(axis=1), 0.0)

    def samples(self, step=0.008):
        pts = []
        for k in range(3):
            i, j = [m for m in range(3) if m != k]
            ni = max(2, int(math.ceil(2 * self.h[i] / step)) + 1)
            nj = max(2, int(math.ceil(2 * self.h[j] / step)) + 1)
            u, v = np.meshgrid(np.linspace(-self.h[i], self.h[i], ni), np.linspace(-self.h[j], self.h[j], nj))
            for s in (-1, 1):
                L = np.zeros((u.size, 3))
                L[:, i], L[:, j], L[:, k] = u.ravel(), v.ravel(), s * self.h[k]
                pts.append(L)
        L = np.vstack(pts)
        return self.c + L @ self.R.T

    def corners(self):
        s = np.array([[a, b, c] for a in (-1, 1) for b in (-1, 1) for c in (-1, 1)], float)
        return self.c + (s * self.h) @ self.R.T

    def bounds(self):
        C = self.corners()
        return C.min(0), C.max(0)

    def moved(self, M, t):
        return OBB(M @ self.c + t, M @ self.R, self.h)


class Capsule:
    def __init__(self, p0, p1, r):
        self.p0, self.p1, self.r = np.asarray(p0, float), np.asarray(p1, float), float(r)

    def sdf(self, P):
        P = np.atleast_2d(P)
        d = self.p1 - self.p0
        L2 = max(float(d @ d), 1e-15)
        t = np.clip((P - self.p0) @ d / L2, 0.0, 1.0)
        return np.linalg.norm(P - (self.p0 + t[:, None] * d), axis=1) - self.r

    def samples(self, step=0.008):
        d = self.p1 - self.p0
        L = float(np.linalg.norm(d))
        a = _unit(d) if L > 1e-9 else np.array([1.0, 0, 0])
        e1 = _unit(np.cross(a, [0, 0, 1.0] if abs(a[2]) < 0.9 else [1.0, 0, 0]))
        e2 = np.cross(a, e1)
        n_l = max(2, int(math.ceil(L / step)) + 1)
        n_c = max(6, int(math.ceil(2 * math.pi * self.r / step)))
        t = np.linspace(0, 1, n_l)
        ph = np.linspace(0, 2 * math.pi, n_c, endpoint=False)
        ring = self.r * (np.cos(ph)[:, None] * e1 + np.sin(ph)[:, None] * e2)
        pts = (self.p0 + t[:, None, None] * d + ring[None, :, :]).reshape(-1, 3)
        caps = np.vstack([self.p0 - self.r * a, self.p1 + self.r * a])
        return np.vstack([pts, caps])

    def bounds(self):
        return np.minimum(self.p0, self.p1) - self.r, np.maximum(self.p0, self.p1) + self.r

    def moved(self, M, t):
        return Capsule(M @ self.p0 + t, M @ self.p1 + t, self.r)


class Sphere:
    def __init__(self, c, r):
        self.c, self.r = np.asarray(c, float), float(r)

    def sdf(self, P):
        return np.linalg.norm(np.atleast_2d(P) - self.c, axis=1) - self.r

    def samples(self, step=0.008):
        n = max(50, int(4 * math.pi * self.r ** 2 / step ** 2))
        i = np.arange(n) + 0.5
        phi = np.arccos(1 - 2 * i / n)
        th = math.pi * (1 + 5 ** 0.5) * i
        return self.c + self.r * np.column_stack([np.cos(th) * np.sin(phi), np.sin(th) * np.sin(phi), np.cos(phi)])

    def bounds(self):
        return self.c - self.r, self.c + self.r

    def moved(self, M, t):
        return Sphere(M @ self.c + t, self.r)


class Cyl:
    """Finite solid cylinder: centre c, unit axis a, radius r, half length h."""

    def __init__(self, c, a, r, h):
        self.c, self.a, self.r, self.h = np.asarray(c, float), _unit(a), float(r), float(h)

    def sdf(self, P):
        Q = np.atleast_2d(P) - self.c
        ax = Q @ self.a
        rad = np.linalg.norm(Q - ax[:, None] * self.a, axis=1)
        d = np.column_stack([rad - self.r, np.abs(ax) - self.h])
        return np.linalg.norm(np.maximum(d, 0.0), axis=1) + np.minimum(d.max(axis=1), 0.0)

    def samples(self, step=0.008):
        e1 = _unit(np.cross(self.a, [0, 0, 1.0] if abs(self.a[2]) < 0.9 else [1.0, 0, 0]))
        e2 = np.cross(self.a, e1)
        n_c = max(12, int(math.ceil(2 * math.pi * self.r / step)))
        n_h = max(2, int(math.ceil(2 * self.h / step)) + 1)
        ph = np.linspace(0, 2 * math.pi, n_c, endpoint=False)
        ring = np.cos(ph)[:, None] * e1 + np.sin(ph)[:, None] * e2
        side = (self.c + self.r * ring[None, :, :] + np.linspace(-self.h, self.h, n_h)[:, None, None] * self.a)
        n_r = max(2, int(math.ceil(self.r / step)))
        disc = (np.linspace(0, self.r, n_r)[:, None, None] * ring[None, :, :]).reshape(-1, 3)
        return np.vstack([side.reshape(-1, 3), self.c + self.h * self.a + disc, self.c - self.h * self.a + disc])

    def bounds(self):
        ext = self.r * np.sqrt(np.clip(1 - self.a ** 2, 0, 1)) + self.h * np.abs(self.a)
        return self.c - ext, self.c + ext

    def moved(self, M, t):
        return Cyl(M @ self.c + t, M @ self.a, self.r, self.h)


class Torus:
    """Tyre envelope (as sizing.tyre_points): centre c, axle unit a, outer radius R, width w (tube radius w/2)."""

    def __init__(self, c, a, R, w):
        self.c, self.a, self.R, self.w = np.asarray(c, float), _unit(a), float(R), float(w)
        self.R0, self.rt = self.R - 0.5 * self.w, 0.5 * self.w

    def sdf(self, P):
        Q = np.atleast_2d(P) - self.c
        ax = Q @ self.a
        rad = np.linalg.norm(Q - ax[:, None] * self.a, axis=1)
        return np.hypot(rad - self.R0, ax) - self.rt

    def samples(self, step=0.008):
        e1 = _unit(np.cross(self.a, [0, 0, 1.0] if abs(self.a[2]) < 0.9 else [1.0, 0, 0]))
        e2 = np.cross(self.a, e1)
        n1 = max(24, int(2 * math.pi * self.R / step))
        n2 = max(12, int(2 * math.pi * self.rt / step))
        th = np.linspace(0, 2 * math.pi, n1, endpoint=False)
        ph = np.linspace(0, 2 * math.pi, n2, endpoint=False)
        E = np.cos(th)[:, None] * e1 + np.sin(th)[:, None] * e2
        pts = (self.c + (self.R0 + self.rt * np.cos(ph))[None, :, None] * E[:, None, :] +
               (self.rt * np.sin(ph))[None, :, None] * self.a)
        return pts.reshape(-1, 3)

    def bounds(self):
        ext = self.R * np.sqrt(np.clip(1 - self.a ** 2, 0, 1)) + self.rt * np.abs(self.a)
        return self.c - ext, self.c + ext

    def moved(self, M, t):
        return Torus(M @ self.c + t, M @ self.a, self.R, self.w)


def polyline(path, r):
    P = np.asarray(path, float)
    return [Capsule(P[i], P[i + 1], r) for i in range(len(P) - 1)]


def mirror_prim(p):
    """Mirror a primitive about y = 0."""
    Mm = np.diag([1.0, -1.0, 1.0])
    if isinstance(p, OBB):
        return OBB(Mm @ p.c, Mm @ p.R, p.h)
    if isinstance(p, Capsule):
        return Capsule(Mm @ p.p0, Mm @ p.p1, p.r)
    if isinstance(p, Sphere):
        return Sphere(Mm @ p.c, p.r)
    if isinstance(p, Torus):
        return Torus(Mm @ p.c, Mm @ p.a, p.R, p.w)
    return Cyl(Mm @ p.c, Mm @ p.a, p.r, p.h)


def _bounds(prims):
    B = [p.bounds() for p in prims]
    return np.min([b[0] for b in B], axis=0), np.max([b[1] for b in B], axis=0)


def gap(A: list, B: list, cutoff: float = 0.1, step: float = 0.008) -> float:
    """Minimum distance between two primitive sets (negative = penetration depth estimate); ``cutoff`` returned when
    the bounding boxes are further apart than it."""
    la, ha = _bounds(A)
    lb, hb = _bounds(B)
    sep = np.maximum(lb - ha, la - hb).max()
    if sep > cutoff:
        return float(cutoff)
    best = float(cutoff)
    for a in A:
        Pa = a.samples(step)
        for b in B:
            lo, hi = b.bounds()
            m = np.all((Pa >= lo - cutoff) & (Pa <= hi + cutoff), axis=1)
            if np.any(m):
                best = min(best, float(b.sdf(Pa[m]).min()))
    for b in B:
        Pb = b.samples(step)
        for a in A:
            lo, hi = a.bounds()
            m = np.all((Pb >= lo - cutoff) & (Pb <= hi + cutoff), axis=1)
            if np.any(m):
                best = min(best, float(a.sdf(Pb[m]).min()))
    return best


# =====================================================================================================================
# context: spec + OML
# =====================================================================================================================
class Ctx:
    def __init__(self, S: dict | None = None):
        if S is None:
            SPEC.load.cache_clear()
            S = copy.deepcopy(SPEC.load())
        self.S = S
        self.L = S["layout"]
        self.af = Z.Airframe(S)
        self.P = S["wing"]["planform"]
        self._wing_cache = {}
        pr = S["propeller"]
        self.hub = np.asarray(pr["hub"], float)
        eps = math.radians(float(pr["thrust_line_inclination_deg"]))
        self.d_thrust = np.array([math.cos(eps), 0.0, math.sin(eps)])
        self.e_up = np.array([-math.sin(eps), 0.0, math.cos(eps)])

    # ---------------------------------------------------------------- OML queries
    def wing_eta(self, y):
        yj = float(self.P["y_junction"])
        y = abs(float(y))
        return y if y <= yj else yj + (y - yj) / math.cos(math.radians(float(self.P["dihedral_deg"])))

    def _wing_section(self, y):
        key = round(abs(float(y)), 3)
        if key not in self._wing_cache:
            srf = self.af.wing
            e = srf.span_coords()
            eta = self.wing_eta(key)
            if eta < float(e[0]) or eta > float(e[-1]):
                self._wing_cache[key] = None
            else:
                n = 121
                loop = srf.loop_at(eta, n)
                up = loop[:n][::-1]
                lo = loop[n - 1:]
                iu, il = np.argsort(up[:, 0]), np.argsort(lo[:, 0])
                self._wing_cache[key] = (up[iu, 0], up[iu, 2], lo[il, 0], lo[il, 2])
        return self._wing_cache[key]

    def inside_wing(self, P, margin=0.0):
        P = np.atleast_2d(np.asarray(P, float))
        out = np.zeros(len(P), bool)
        for i, (x, y, z) in enumerate(P):
            w = self._wing_section(y)
            if w is None:
                continue
            xu, zu, xl, zl = w
            if x <= max(xu[0], xl[0]) + margin or x >= min(xu[-1], xl[-1]) - margin:
                continue
            out[i] = (np.interp(x, xl, zl) + margin < z < np.interp(x, xu, zu) - margin)
        return out

    def inside(self, P, margin=0.0, tail=False):
        """Inside the body or the wing loft (``margin`` m from the skin); near the side-of-body seam (|y| 0.36-0.41)
        a point inside the body also counts when it is inside the glove root profile extruded inboard (body and
        glove are one closed shell there); ``tail``: also inside the fins / stubs / ventral."""
        P = np.atleast_2d(np.asarray(P, float))
        ok = self.af.inside(P, margin)
        rest = ~ok
        if np.any(rest):
            ok[rest] = self.inside_wing(P[rest], margin)
        rest = ~ok
        y_root = float(self.S["wing"]["sections"][0]["y"])
        seam = rest & (np.abs(P[:, 1]) > 0.36) & (np.abs(P[:, 1]) < 0.41)
        if np.any(seam):
            Q = P[seam].copy()
            in_body = self.af.inside(Q, 0.0) | self.inside_wing(Q, 0.0)
            Q2 = Q.copy()
            Q2[:, 1] = np.sign(Q2[:, 1]) * (y_root + 0.002)
            ok[seam] = in_body & self.inside_wing(Q2, margin)
        if tail:
            rest = ~ok
            for name in ("fin", "stabilator_stub", "ventral"):
                if np.any(rest):
                    sub_ = self.inside_surface(name, P[rest], margin)
                    idx = np.where(rest)[0]
                    ok[idx[sub_]] = True
                    rest = ~ok
        return ok

    def inside_surface(self, name: str, P, margin=0.0):
        """Inside a tail lifting surface (both sides if mirrored) by the section loop at the point's span station."""
        from matplotlib.path import Path as MPath
        srf = self.af.tail[name]
        P = np.atleast_2d(np.asarray(P, float))
        out = np.zeros(len(P), bool)
        e = srf.span_coords()
        secs = srf.sections
        LE = np.array([[s_["y"], s_["z_le"]] for s_ in secs], float)
        for i, p in enumerate(P):
            q = p.copy()
            if self.af.tail_mirror.get(name, True) and q[1] < 0:
                q[1] = -q[1]
            yz = q[1:]
            best, eta = None, None
            for k in range(len(LE) - 1):
                a, b = LE[k], LE[k + 1]
                d = b - a
                t = float(np.clip((yz - a) @ d / max(d @ d, 1e-12), 0, 1))
                dist = float(np.linalg.norm(yz - (a + t * d)))
                if best is None or dist < best:
                    best, eta = dist, e[k] + t * (e[k + 1] - e[k])
            if eta is None or eta < e[0] or eta > e[-1]:
                continue
            loop = srf.loop_at(float(eta), 61)
            o, c, u, n = srf.frame_at(float(eta))
            if abs((q - o) @ n) > 0.05:
                continue
            poly = np.column_stack([(loop - o) @ c, (loop - o) @ u])
            pt = np.array([(q - o) @ c, (q - o) @ u])
            path = MPath(poly)
            if not path.contains_point(pt):
                continue
            seg_d = min(float(np.linalg.norm(np.cross(np.append(poly[j + 1] - poly[j], 0),
                                                       np.append(pt - poly[j], 0)))) /
                        max(float(np.linalg.norm(poly[j + 1] - poly[j])), 1e-12) for j in range(len(poly) - 1))
            out[i] = seg_d >= margin
        return out

    def z_top(self, x, y=0.0):
        return float(self.af.z_top(float(x), float(y)))

    def z_bot(self, x, y=0.0):
        return float(self.af.z_bot(float(x), float(y)))

    def engine_point(self, ahead, dy=0.0, dz=0.0):
        return self.hub - ahead * self.d_thrust + dy * np.array([0.0, 1.0, 0.0]) + dz * self.e_up

    def engine_obb(self, b):
        """Engine-axes box {u, v, w} -> OBB (u ahead of the propeller plane along the thrust axis)."""
        u0, u1 = (float(v) for v in b["u"])
        v0, v1 = (float(v) for v in b["v"])
        w0, w1 = (float(v) for v in b["w"])
        c = self.engine_point(0.5 * (u0 + u1), 0.5 * (v0 + v1), 0.5 * (w0 + w1))
        R = np.column_stack([-self.d_thrust, [0.0, 1.0, 0.0], self.e_up])
        return OBB(c, R, [0.5 * (u1 - u0), 0.5 * (v1 - v0), 0.5 * (w1 - w0)])


# =====================================================================================================================
# small helpers
# =====================================================================================================================
def _box_c(b, centre_y=False):
    b = np.asarray(b, float)
    c = 0.5 * (b[0] + b[1])
    if centre_y:
        c[1] = 0.0
    return c


def _path_len_c(path):
    P = np.asarray(path, float)
    seg = np.linalg.norm(np.diff(P, axis=0), axis=1)
    mid = 0.5 * (P[1:] + P[:-1])
    return float(seg.sum()), (seg[:, None] * mid).sum(0) / max(float(seg.sum()), 1e-12)


def _r(v, n=4):
    if isinstance(v, (list, tuple, np.ndarray)):
        return [_r(x, n) for x in v]
    return float(round(float(v), n))


def _row(check, item, ok, value=None, limit=None, detail=""):
    return {"check": check, "item": item, "ok": bool(ok), "value": value, "limit": limit, "detail": detail}


def _ids(L, key):
    return {o["id"]: o for o in L.get(key, [])}


# =====================================================================================================================
# mass placement (layout-phase centroids of the sizing mass items; applied by sizing.mass_items)
# =====================================================================================================================
def mass_placements(ctx: Ctx, L: dict | None = None) -> dict:
    """Centroids of the layout objects that carry each listed spec mass item. The same function produces
    ``layout.mass_placement`` (layout phase) and verifies it (check C06)."""
    S, af = ctx.S, ctx.af
    L = L if L is not None else ctx.L
    ch = _ids(L["chassis"], "members")
    fit = _ids(L["chassis"], "fittings")
    eqs = _ids(L["systems"], "equipment")
    lt = _ids(L["systems"], "air_data_lights")
    E = S["engine"]["installed_items_kg"]
    pr = S["propeller"]
    hub_face = float(pr["hub_spacer"]) + float(pr["hub_half_thickness"])
    mount_face = hub_face + float(L["chassis"]["engine_mount"]["mount_face"]["distance_ahead_of_prop_plane"]) - hub_face
    sg_front = hub_face + float(S["engine"]["envelope"]["length_with_sg750"])
    out = {}

    def put(name, comps, basis):
        m = np.array([c[1] for c in comps], float)
        P = np.array([c[2] for c in comps], float)
        cen = (m[:, None] * P).sum(0) / m.sum()
        out[name] = {"mode": "absolute", "position": _r(cen), "basis": basis,
                     "components": [{"what": c[0], "fraction": _r(c[1] / m.sum(), 4), "at": _r(c[2])} for c in comps]}

    u_eng = 0.5 * (hub_face + mount_face) + 0.5 * 0.0   # crankcase centre between the flange and the mount face
    put("engine_group_installed", [
        ("engine_bare (crankcase/cylinders centre between the flange and the mount face, estimate)", E["engine_bare"],
         ctx.engine_point(u_eng)),
        ("ECU + harness + coils (ECU location)", E["ecu_harness_ignition_coils"], _box_c(eqs["EQ-ECU"]["box"])),
        ("fuel pump/regulator/filter (aft equipment bay)", E["fuel_pump_regulator_filter"],
         _box_c(eqs["EQ-FUELPUMP"]["box"])),
        ("SG750 (crank rear, ahead of the mount face)", E["starter_generator_sg750"], ctx.engine_point(sg_front - 0.0143)),
        ("generator power electronics (port avionics side bay)", E["generator_power_electronics"],
         _box_c(eqs["EQ-GENERATOR_PE"]["box"])),
        ("generator adapter/coupling", E["generator_adapter_coupling"], ctx.engine_point(mount_face + 0.02)),
        ("exhaust stacks + silencers (both sides, centroid)", E["exhaust_with_silencers"],
         np.mean([_box_c(b, True) for k in L["keep_outs"] if k["id"].startswith("KO-EXHAUST") for b in k["boxes"]],
                 axis=0)),
        ("mount isolators + bolts", E["engine_mount_isolators_bolts"], ctx.engine_point(mount_face + 0.01)),
        ("throttle servo + CHT/EGT sensors", E["throttle_servo_cht_egt_sensors"], ctx.engine_point(0.24, 0.0, -0.10))],
        basis="engine.installed_items_kg at the layout positions (engine on the inclined crank axis; ECU in the "
              "mission bay, fuel pump in the aft equipment bay, generator PE in the port avionics side bay)")
    fw = float(L["firewall_x"])
    put("cooling_baffles_firewall_cowl_flap", [
        ("firewall stainless shield + edge angle (section centroid)", 0.8, [fw - 0.0003, 0.0, 0.125]),
        ("cylinder baffles + plenum", 0.6, ctx.engine_point(hub_face + 0.115, 0.0, 0.03)),
        ("cowl-flap / exit lip parts", 0.1, [3.98, 0.0, 0.22])],
        basis="split of the allowance (estimate): firewall shield 0.5 mm 304 sheet ~0.8 kg, baffles/plenum 0.6, exit "
              "lip 0.1")
    inl = next(p for p in L["shell"]["panels"] if p["id"] == "P-INLET")
    put("dorsal_cooling_inlet_s_duct", [("inlet lip", 0.3, [0.5 * sum(inl["x"]), 0.0, 0.285]),
                                        ("S-duct", 0.7, [0.5 * (inl["x"][1] + fw), 0.0, 0.27])],
        basis="flush dorsal inlet (P-INLET) + S-duct to the firewall duct cut-out")
    sa = eqs["EQ-STABACT"]
    put("actuators_stabilators_2x_DA30", [("2 x DA 30 on the firewall forward face", 1.26, _box_c(sa["box"], True)),
                                          ("pushrods + spindle horns", 0.22,
                                           [0.5 * (sa["linkage"]["servo_axis"][0] + sa["linkage"]["horn_axis"][0]),
                                            0.0, sa["linkage"]["horn_axis"][2]])],
        basis="DA 30 lying along y on the cool side of the firewall, pushrods through fireproof boots")
    ng = S["landing_gear"]["nose"]
    put("actuators_nose_steering_brake_2x_DA26",
        [("steering DA 26 on the nose leg (retracted)", 0.32, [float(ng["retraction"]["stowed_wheel_center"][0]) - 0.13,
                                                               0.0, -0.08]),
         ("brake DA 26 + master cylinder (port avionics side bay)", 0.32, _box_c(eqs["EQ-BRAKE_UNIT"]["box"]))],
        basis="steering actuator travels with the leg (flight CG = retracted); brake-by-wire master cylinder in the "
              "nose, brake lines along H-MAIN")
    cab = []
    for t in L["systems"]["harness"]["trunks"]:
        ln, c = _path_len_c(t["path"])
        w = ln * float(t["diameter"]) ** 2
        cab.append((t["id"] + (" (x2)" if t.get("mirror") else ""), (2 if t.get("mirror") else 1) * w,
                    [c[0], 0.0, c[2]] if t.get("mirror") else c.tolist()))
    b2 = 0.5 * float(S["wing"]["span"])
    yj = float(S["wing"]["planform"]["y_junction"])
    w_wing = 2 * (b2 - yj) * 0.012 ** 2
    yw = 0.5 * (yj + b2)
    secs = S["wing"]["sections"]
    ys = np.array([s_["y"] for s_ in secs])
    xw = float(np.interp(yw, ys, [s_["x_le"] + 0.45 * s_["chord"] for s_ in secs]))
    zw = float(np.interp(yw, ys, [s_["z_le"] for s_ in secs]))
    cab.append(("outer-panel harnesses (x2, to the tips)", w_wing, [xw, 0.0, zw]))
    pts = []
    for e in L["systems"]["equipment"]:
        pts += [_box_c(e["box"])] * (2 if e.get("mirror") else 1)
    for a in L["systems"]["actuators"]:
        pts += [_box_c(a["box"])] * (2 if a.get("mirror") else 1)
    for a in L["systems"]["antennas"]:
        pts.append(np.asarray(a["point"], float))
    for a in L["systems"]["air_data_lights"]:
        pts.append(_box_c(a["box"]) if "box" in a else 0.5 * (np.asarray(a["p0"], float) + np.asarray(a["p1"], float)))
    lg = S["landing_gear"]
    pts += [np.asarray(lg["main"]["trunnion"], float)] * 2 + [np.asarray(lg["nose"]["pivot"], float)] * 2
    wc = sum(c[1] for c in cab)
    comps = [(c[0], 0.70 * c[1] / wc, c[2]) for c in cab]
    comps.append((f"connectors at {len(pts)} connection points", 0.30,
                  np.mean([[p[0], 0.0, p[2]] for p in pts], axis=0).tolist()))
    put("wiring_harness_connectors_coax", comps,
        basis="cables 70 % (estimate) by trunk length x diameter^2 (layout.systems.harness) + outer-panel harnesses; "
              "connectors, backshells and coax terminations 30 % (estimate) equally over the connection points of the "
              "layout (equipment, actuators, antennas, lights, gear EMAs)")
    put("flight_termination_lights", [("FTS unit (forward bay)", 0.15, _box_c(eqs["EQ-FTS_UNIT"]["box"])),
                                      ("2 wing-tip lights", 0.166, _box_c(lt["LT-WING"]["box"], True)),
                                      ("tail light (fin tip)", 0.083, _box_c(lt["LT-TAIL"]["box"]))],
        basis="FTS 0.15 kg + 3 x AveoFlash 0.083 kg at their installed places")
    comps = []
    for mid in ("M-CHINE", "M-KEELWALL", "M-KEEL", "M-DORSAL", "M-AFTKEEL"):
        m = ch[mid]
        k = 2.0 if m.get("mirror") else 1.0
        if "paths" in m:
            for p in m["paths"]:
                ln, c = _path_len_c(p)
                comps.append((mid, k * ln, [c[0], 0.0, c[2]]))
        else:
            b = np.asarray(m["box"], float)
            comps.append((mid, k * (b[1][0] - b[0][0]), _box_c(b, True).tolist()))
    put("keel_beams_longerons", comps, basis="length-weighted centroid of the longitudinal members (chine longerons, "
                                             "keel walls, keel beams, dorsal longerons, aft keel)")
    comps = []
    zb = L["zones_preliminary"]["wing_carry_through"]["box"]
    for s_ in L["stations"]:
        x = float(s_["x"])
        yy, zz = np.meshgrid(np.arange(-0.45, 0.45, 0.005), np.arange(-0.25, 0.40, 0.005))
        P = np.column_stack([np.full(yy.size, x), yy.ravel(), zz.ravel()])
        ins = af.inside(P, margin=0.006)
        if s_["type"] == "ring":
            ins &= ~af.inside(P, margin=0.006 + float(s_.get("ring_depth", 0.04)))
        for c in s_.get("cutouts", []):
            for sg in ((1.0, -1.0) if c.get("mirror") else (1.0,)):
                y0, y1 = sorted([sg * c["y"][0], sg * c["y"][1]])
                ins &= ~((P[:, 1] >= y0) & (P[:, 1] <= y1) & (P[:, 2] >= c["z"][0]) & (P[:, 2] <= c["z"][1]))
        if s_.get("subtype") == "spar frame":
            ins &= ~((P[:, 2] >= zb[0][2]) & (P[:, 2] <= zb[1][2]))
        a = float(ins.sum()) * 0.005 ** 2
        zc_ = float(P[ins, 2].mean()) if ins.any() else 0.0
        rho = 2700.0 if str(s_["material"]).startswith("al_") else 400.0
        comps.append((s_["id"], max(a, 1e-4) * float(s_.get("t", 0.0068)) * rho, [x, 0.0, zc_]))
    put("frames_bulkheads", comps, basis="13 stations, mass ~ net web area (section inside the 6 mm skin inset minus "
                                         "the declared cut-outs and, for the spar frames, the box) x thickness x "
                                         "effective density (sandwich ~400 kg/m3, aluminium ring 2700 kg/m3; estimate)")
    em = L["chassis"]["engine_mount"]
    put("engine_mount_4130", [("4130 truss + isolator ring", 1.0,
                               np.mean([0.5 * (np.asarray(t["a"]) + np.asarray(t["b"])) for t in em["tubes"]] +
                                       [np.asarray(n) for n in em["ring_nodes"]], axis=0) * [1, 0, 1])],
        basis="mean of the truss tube mid-points and the ring nodes (layout.chassis.engine_mount)")
    pb = L["zones_preliminary"]["parachute_bay"]["box"]
    put("parachute_attach_fitting", [("forward bridle fitting", 0.4, fit["F-RISER-FWD"]["point"]),
                                     ("aft bridle fitting (rear-spar frame)", 0.4, fit["F-RISER-AFT"]["point"]),
                                     ("container restraint brackets", 0.2, [0.5 * (pb[0][0] + pb[1][0]), 0.0, -0.03])],
        basis="two bridle fittings (Y-bridle) + container restraint")
    comps = []
    for mid in ("M-DECK-NOSE", "M-MIDFLOOR", "M-FWDDECK", "M-TURRETROOF", "M-PARAFLOOR"):
        b = np.asarray(ch[mid]["box"], float)
        comps.append((mid, (b[1][0] - b[0][0]) * (b[1][1] - b[0][1]), _box_c(b, True).tolist()))
    pay = L["zones_preliminary"]["payload_bay"]["box"]
    comps.append(("payload-tray rails", 0.06, [0.5 * (pay[0][0] + pay[1][0]), 0.0, -0.065]))
    eb = L["zones_preliminary"]["equipment_bay_aft"]["box"]
    comps.append(("aft equipment tray", 0.06, [0.5 * (eb[0][0] + eb[1][0]), 0.0, -0.045]))
    sb = L["zones_preliminary"]["avionics_side_bays"]["box"]
    comps.append(("side-bay trays", 0.04, [0.5 * (sb[0][0] + sb[1][0]), 0.0, -0.09]))
    put("floors_trays_rails", comps, basis="area-weighted decks/floors + trays")
    comps = []
    for p in L["shell"]["panels"]:
        if p["attach"] in ("removable", "hinged"):
            x0, x1 = p["x"]
            y0, y1 = p["y"]
            per = 2 * ((x1 - x0) + (y1 - y0)) * (2.0 if p.get("mirror") else 1.0)
            comps.append((p["id"], per, [0.5 * (x0 + x1), 0.0, 0.0]))
    put("hatch_frames_quick_access_fasteners", comps, basis="perimeter-weighted removable panels/hatches (frame lands, "
                                                           "Camlocs, nutplates)")
    comps = []
    for f in L["chassis"]["fittings"]:
        if f["id"].startswith(("F-FIN", "F-STUB", "F-VENTRAL")):
            p = np.asarray(f["point"], float)
            comps.append((f["id"], 2.0 if f.get("mirror") else 1.0, [p[0], 0.0, p[2]]))
    put("fin_ventral_root_fittings", comps, basis="fin, stub and ventral root fittings at the frames")
    return out


# =====================================================================================================================
# layout objects (simplified geometry)
# =====================================================================================================================
class Obj:
    """A layout object: id, part, kind (structure / content / zone / external), primitives, flags."""

    def __init__(self, oid, part, kind, prims, group="", material="", skin=(), source=""):
        self.id, self.part, self.kind, self.prims = oid, part, kind, list(prims)
        self.group, self.material, self.skin, self.source = group, material, tuple(skin), source

    def __repr__(self):
        return f"Obj({self.id})"


def obb_of(d: dict):
    """'cylinder' {center, axis, radius, half_length} or 'obb' {center, axes (rows), half} if given, else the
    axis-aligned 'box'."""
    if d.get("cylinder"):
        c = d["cylinder"]
        return Cyl(c["center"], c["axis"], c["radius"], c["half_length"])
    if d.get("obb"):
        o = d["obb"]
        return OBB(o["center"], np.asarray(o["axes"], float).T, o["half"])
    return OBB.aabb(d["box"])


def ctbox_prims(m: dict) -> list:
    """Centre wing box as spar-cap capsules along the main and rear spar lines (both halves) + web boxes."""
    z0, z1 = (float(v) for v in m["z"])
    out = []
    for key, ck in (("main_spar_line", "main_spar_caps_z"), ("rear_spar_line", "rear_spar_caps_z")):
        P = np.asarray(m[key], float)
        C = np.asarray(m.get(ck) or [[z0 + 0.004, z1 - 0.004]] * len(P), float)
        P = np.vstack([P[::-1] * [1, -1, 1], P[1:]])
        C = np.vstack([C[::-1], C[1:]])
        for k in (0, 1):
            zz = C[:, k] + (0.004 if k == 0 else -0.004)
            out += polyline(np.column_stack([P[:, 0], P[:, 1], zz]), 0.004)
    return out


def _mirror_obj(o: Obj) -> Obj:
    m = Obj(o.id + "@L", (o.part + "-L") if o.part else "", o.kind, [mirror_prim(p) for p in o.prims], o.group,
            o.material, o.skin, o.source)
    o.id, o.part = o.id + "@R", ((o.part + "-R") if o.part else "")
    return m


def _with_mirror(o: Obj, mirror: bool) -> list:
    return [o, _mirror_obj(o)] if mirror else [o]


def turret_prims(ctx: Ctx, travel: float = 0.0) -> list:
    """E180 growth envelope (sphere + cylinder stem up to the growth height) at elevator travel ``travel`` (m)."""
    T = ctx.S["payload"]["turret"]
    g = T["growth_envelope"]
    r = 0.5 * float(g["diameter"])
    c = np.array([float(T["bay_center_x"]), 0.0, float(T["ball_center_retracted_z"]) - travel])
    h = float(g["height"]) - r
    stem_r = 0.5 * float(T.get("stem_diameter", 0.1))
    return [Sphere(c, r), Cyl(c + np.array([0, 0, 0.5 * h]), [0, 0, 1], stem_r, 0.5 * h)]


def gear_prims(ctx: Ctx, which: str, ang: float, side: str = "R") -> dict:
    """Tyre (solid cylinder) and leg (capsule) of a gear leg rotated by ``ang`` (rad) from the down position about its
    retraction axis (mechanisms joints main_gear_R/L, nose_gear)."""
    LG = ctx.S["landing_gear"]
    ty = LG["tyre"]
    J = {j["name"]: j for j in ctx.L["mechanisms"]["joints"]}
    if which == "main":
        jn = "main_gear_" + side
        g = LG["main"]
        T = np.asarray(g["trunnion"], float)
        A = np.asarray(g["axle_static"], float)
        ax_wheel = np.array([0.0, 1.0, 0.0])
        if side == "L":
            T, A = T * [1, -1, 1], A * [1, -1, 1]
        width = float(g["leg_frontal_width"])
    else:
        jn = "nose_gear"
        g = LG["nose"]
        T = np.asarray(g["pivot"], float)
        A = np.asarray(g["axle_static"], float)
        ax_wheel = np.array([0.0, 1.0, 0.0])
        width = float(g["leg_frontal_width"])
    j = J[jn]
    o, a = np.asarray(j["origin"], float), _unit(j["axis"])
    M = _rot(a, ang)
    t = o - M @ o
    tyre = Torus(A, ax_wheel, 0.5 * float(ty["diameter"]), float(ty["width"])).moved(M, t)
    leg = Capsule(T, A, 0.5 * width).moved(M, t)
    return {"tyre": tyre, "leg": leg}


def layout_objects(ctx: Ctx) -> list:
    S, L = ctx.S, ctx.L
    O = []
    for m in L["chassis"]["members"]:
        if "main_spar_line" in m:
            prims = ctbox_prims(m)
        elif "box" in m:
            prims = [OBB.aabb(m["box"])]
        else:
            prims = [c for p in m["paths"] for c in polyline(p, 0.008)]
        skin = [k for k in ("top", "bottom", "sides") if m.get(k) == "skin"]
        O += _with_mirror(Obj(m["id"], m["part"], "structure", prims, "chassis", m.get("material", ""), skin,
                              "layout.chassis.members"), bool(m.get("mirror")))
    for f in L["chassis"]["fittings"]:
        if "box" not in f:
            continue
        O += _with_mirror(Obj(f["id"], f["part"], "structure", [obb_of(f)], "chassis",
                              f.get("material", ""), (), "layout.chassis.fittings"), bool(f.get("mirror")))
    for e in L["systems"]["equipment"]:
        box = e.get("envelope_with_connector", e["box"])
        if e["id"] in ("EQ-TURRET",):
            continue                                          # the turret is checked on its growth envelope
        O += _with_mirror(Obj(e["id"], e["part"], "content", [OBB.aabb(box)], e.get("group", ""), "",
                              (), "layout.systems.equipment"), bool(e.get("mirror")))
    for a in L["systems"]["actuators"]:
        O += _with_mirror(Obj(a["id"], a["part"], "content", [obb_of(a)], "controls", "", (),
                              "layout.systems.actuators"), bool(a.get("mirror")))
    for a in L["systems"]["antennas"]:
        p, s = np.asarray(a["point"], float), 0.5 * np.asarray(a["size"], float)
        ext = str(a.get("window", "")).startswith("external")
        O.append(Obj(a["id"], a["part"], "external" if ext else "content", [OBB.aabb([p - s, p + s])], "systems",
                     "", (), "layout.systems.antennas"))
    for a in L["systems"]["air_data_lights"]:
        if "box" in a:
            prims = [OBB.aabb(a["box"])]
        else:
            prims = [Capsule(a["p0"], a["p1"], 0.5 * float(a["diameter"]))]
        O += _with_mirror(Obj(a["id"], a["part"], "external", prims, "systems", "", (),
                              "layout.systems.air_data_lights"), bool(a.get("mirror")))
    for t in L["systems"]["harness"]["trunks"]:
        O += _with_mirror(Obj(t["id"], t["part"], "harness", polyline(t["path"], 0.5 * float(t["diameter"])),
                              "systems", "", (), "layout.systems.harness"), bool(t.get("mirror")))
    em = L["chassis"]["engine_mount"]
    O.append(Obj("ENGINE-MOUNT", em["part"], "mount", [Capsule(t["a"], t["b"], 0.008) for t in em["tubes"]] +
                 [Capsule(em["ring_nodes"][i], em["ring_nodes"][j], 0.008)
                  for i, j in ((0, 2), (2, 3), (3, 1), (1, 0))], "chassis", "steel_4130_n", (),
                 "layout.chassis.engine_mount"))
    for k in L["keep_outs"]:
        if k["id"] == "KO-ENGINE":
            O.append(Obj("ENGINE", "YK250-PR-500", "engine", [ctx.engine_obb(b) for b in k["boxes"]], "propulsion",
                         "", (), "layout.keep_outs.KO-ENGINE"))
        if k["id"].startswith("KO-EXHAUST"):
            O.append(Obj(k["id"], "YK250-PR-504-" + k["id"][-1], "exhaust", [OBB.aabb(b) for b in k["boxes"]],
                         "propulsion", "", (), "layout.keep_outs"))
    tb = L["chassis"]["turret_elevator"]
    O.append(Obj("TURRET", "YK250-PL-820", "content", turret_prims(ctx, 0.0), "payload", "", (),
                 "payload.turret.growth_envelope (retracted)"))
    for rl in tb["rails"]:
        O.append(Obj(rl["id"], rl["part"], "content", [Capsule(rl["line"][0], rl["line"][1], 0.0065)], "payload", "",
                     (), "layout.chassis.turret_elevator"))
    O.append(Obj("ELEV-SCREW", tb["part"], "content", [Capsule(tb["drive"]["screw_axis"][0],
                                                                tb["drive"]["screw_axis"][1], 0.006)], "payload", "",
                 (), "layout.chassis.turret_elevator"))
    for jn, sd in (("main", "R"), ("main", "L"), ("nose", "")):
        ang = float(next(j["hi"] for j in L["mechanisms"]["joints"]
                         if j["name"] == ("main_gear_" + sd if jn == "main" else "nose_gear")))
        g = gear_prims(ctx, jn, ang, sd or "R")
        O.append(Obj(f"TYRE-{jn}{sd}-STOWED", "YK250-LG-622-" + sd if sd else "YK250-LG-626", "gear",
                     [g["tyre"]], "gear", "", (), "landing_gear (stowed)"))
    sw = {s_["id"]: float(s_.get("sweep_deg", 0.0)) for s_ in L["stations"]}
    cell_sw = {"forward_cell": (0.0, sw.get("FS-MS", 0.0)), "saddle_cell": (sw.get("FS-MS", 0.0), sw.get("FS-RS", 0.0)),
               "aft_cell": (sw.get("FS-RS", 0.0), 0.0)}
    for c in L["fuel_cells"]:
        O.append(Obj("FUEL-" + c["name"], "", "fuel", [FuelBand(ctx, c, cell_sw.get(c["name"], (0.0, 0.0)))], "fuel",
                     "", (), "layout.fuel_cells (chevron bays)"))
    for k in L["keep_outs"]:
        if k["id"] == "KO-COOLING-DUCT":
            P = np.asarray(k["path"], float)
            O.append(Obj("DUCT-COOLING", "YK250-PR-546", "content",
                         [c for dy in k["lateral_offsets"] for c in polyline(P + [0, dy, 0], float(k["radius"]))],
                         "propulsion", "", (), "layout.keep_outs.KO-COOLING-DUCT"))
    st = L["chassis"]["parachute"]["bridle"]
    O.append(Obj("BRIDLE-AFT-LEG", "YK250-SY-802", "content",
                 [Capsule(np.asarray(st["forward_leg"]["point"]) + [0.03, 0, 0.0],
                          np.asarray(st["aft_leg"]["point"]) - [0.03, 0, 0.0], 0.008)], "systems", "", (),
                 "layout.chassis.parachute.bridle (stowed aft leg)"))
    for e in L["systems"]["equipment"]:
        lk = e.get("linkage")
        if lk:
            sv = np.asarray(lk["servo_axis"], float)
            hz = np.asarray(lk["horn_axis"], float)
            p0 = sv + np.array([0.0, 0.0, float(lk["servo_arm_m"])])
            p1 = hz + np.array([0.0, 0.0, float(lk["horn_m"])])
            O += _with_mirror(Obj(e["id"] + "-PUSHROD", e["part"], "linkage", [Capsule(p0, p1, 0.005)],
                                  "controls", "", (), "layout.systems.equipment.linkage"), bool(e.get("mirror")))
    return O


class FuelBand:
    """Fuel cell: the body section band between z0..z1 over x0..x1, 25 mm inside the OML (sizing._area_band)."""

    def __init__(self, ctx: Ctx, c: dict, sweeps=(0.0, 0.0)):
        self.ctx, self.x, self.z, self.inset = ctx, [float(v) for v in c["x"]], [float(v) for v in c["z"]], \
            float(c.get("inset", 0.025))
        self.t0, self.t1 = (math.tan(math.radians(float(v))) for v in sweeps)
        xs = np.arange(self.x[0], self.x[1] + 0.07, 0.01)
        yy = np.arange(-0.45, 0.4501, 0.01)
        zz = np.arange(self.z[0], self.z[1] + 1e-9, 0.01)
        G = np.array(np.meshgrid(xs, yy, zz, indexing="ij")).reshape(3, -1).T
        ay = np.abs(G[:, 1])
        keep = (G[:, 0] >= self.x[0] + ay * self.t0) & (G[:, 0] <= self.x[1] + ay * self.t1)
        G = G[keep]
        self._pts = G[ctx.af.inside(G, self.inset)]

    def sdf(self, P):
        P = np.atleast_2d(P)
        ay = np.abs(P[:, 1])
        dx = np.maximum(self.x[0] + ay * self.t0 - P[:, 0], P[:, 0] - self.x[1] - ay * self.t1)
        dz = np.maximum(self.z[0] - P[:, 2], P[:, 2] - self.z[1])
        d = np.maximum(dx, dz)
        ins = self.ctx.af.inside(P, self.inset)
        out = np.where(d > 0, d, np.where(ins, -0.001, 0.001))
        return out

    def samples(self, step=0.01):
        return self._pts

    def bounds(self):
        if len(self._pts) == 0:
            return np.array([self.x[0], 0, self.z[0]]), np.array([self.x[1], 0, self.z[1]])
        return self._pts.min(0) - 0.01, self._pts.max(0) + 0.01


# =====================================================================================================================
# C01 ids, numbering, references
# =====================================================================================================================
def _all_parts(L: dict) -> list:
    """(source, part id, mirror) of every object of the layout that names a part."""
    out = []
    for s_ in L["stations"]:
        out.append(("stations." + s_["id"], s_["part"], False))
    for m in L["chassis"]["members"]:
        out.append(("chassis.members." + m["id"], m["part"], bool(m.get("mirror"))))
    for f in L["chassis"]["fittings"]:
        out.append(("chassis.fittings." + f["id"], f["part"], bool(f.get("mirror"))))
    for p in L["shell"]["panels"]:
        out.append(("shell.panels." + p["id"], p["part"], bool(p.get("mirror"))))
    for k in ("equipment", "antennas", "air_data_lights", "actuators"):
        for e in L["systems"][k]:
            if e.get("integral_to"):
                continue                          # integral to another listed item (same part)
            out.append((f"systems.{k}." + e["id"], e["part"], bool(e.get("mirror"))))
    for t in L["systems"]["harness"]["trunks"]:
        out.append(("systems.harness." + t["id"], t["part"], bool(t.get("mirror"))))
    out.append(("chassis.engine_mount", L["chassis"]["engine_mount"]["part"], False))
    return out


def check_ids(ctx: Ctx) -> list:
    L = ctx.L
    R = []
    rng = L["part_numbers"]
    codes = L["part_numbering"]["codes"]
    mods = L["part_numbering"]["modules"]
    # ranges disjoint
    iv = sorted((v[0], v[1], k) for k, v in rng.items())
    ok = all(iv[i][1] < iv[i + 1][0] for i in range(len(iv) - 1))
    R.append(_row("C01", "part_numbers: module ranges disjoint", ok, None, None, str(iv)))
    seen = {}
    bad = []
    for src, pid, mir in _all_parts(L):
        m = ID_RE.match(pid)
        if not m or m.group(3):
            bad.append(f"{src}: {pid} (convention; mirrored parts are named without the side suffix)")
            continue
        code, num = m.group(1), int(m.group(2))
        owner = [k for k, (a, b) in rng.items() if a <= num <= b]
        if not owner:
            bad.append(f"{src}: {pid} number outside every module range")
            continue
        grp = [g for g, c in codes.items() if c == code]
        if not any(g in mods[owner[0]]["groups"] for g in grp):
            bad.append(f"{src}: {pid} code {code} not produced by module {owner[0]}")
        seen.setdefault(pid, []).append(src)
    R.append(_row("C01", "part ids: convention YK250-<CODE>-NNN, number in a module range of that code", not bad,
                  len(bad), 0, "; ".join(bad[:8])))
    dup = {k: v for k, v in seen.items() if len(v) > 1}
    R.append(_row("C01", "part ids unique across the layout", not dup, len(dup), 0,
                  "; ".join(f"{k}: {v}" for k, v in list(dup.items())[:6])))
    root = L.get("root_part")
    R.append(_row("C01", "root_part is a chassis member", root in [m["part"] for m in L["chassis"]["members"]], root,
                  "YK250-CH-...", ""))
    # references
    st_ids = {"ST-" + s_["id"] for s_ in L["stations"]}
    mem_ids = {m["id"] for m in L["chassis"]["members"]}
    fit_ids = {f["id"] for f in L["chassis"]["fittings"]}
    pan_ids = {p["id"] for p in L["shell"]["panels"]}
    eq_ids = {e["id"] for e in L["systems"]["equipment"]}
    known = st_ids | mem_ids | fit_ids | pan_ids | eq_ids
    miss = []
    for m in L["chassis"]["members"]:
        for t in m.get("touch", []):
            if t.endswith("*"):
                if not any(k.startswith(t[:-1]) for k in known):
                    miss.append(f"{m['id']}.touch {t}")
            elif t not in known:
                miss.append(f"{m['id']}.touch {t}")
    for p in L["shell"]["panels"]:
        for t in p.get("lands", []):
            if t not in known:
                miss.append(f"{p['id']}.lands {t}")
    parts = {pid for _, pid, _ in _all_parts(L)}
    for t in L["systems"]["harness"]["trunks"]:
        for pn in t.get("penetrations", []):
            if pn["member"] not in mem_ids:
                miss.append(f"{t['id']}.penetrations {pn['member']}")
    for a in L["systems"]["antennas"]:
        if a.get("integral_to") and a["integral_to"] not in eq_ids:
            miss.append(f"{a['id']}.integral_to {a['integral_to']}")
        w = str(a.get("window", ""))
        mm_ = ID_RE.match(w)
        if not w.startswith("external") and w not in parts and not (mm_ and any(
                a_ <= int(mm_.group(2)) <= b_ for a_, b_ in rng.values())):
            miss.append(f"{a['id']}.window {w}")
    jn = {j["name"] for j in L["mechanisms"]["joints"]}
    for p in L["shell"]["panels"]:
        if p.get("joint") and p["joint"] not in jn:
            miss.append(f"{p['id']}.joint {p['joint']}")
    for r in L["clearances"]:
        for j in r.get("joints", []) or []:
            if j not in jn:
                miss.append(f"clearance '{r['name']}' joint {j}")
        for side in ("a", "b"):
            sel = r[side] if isinstance(r[side], list) else [r[side]]
            for s_ in sel:
                if s_.startswith("joint:") and s_[6:] not in jn:
                    miss.append(f"clearance '{r['name']}' {s_}")
                elif s_.startswith("YK250-"):
                    mm = ID_RE.match(s_)
                    if not mm or not any(a <= int(mm.group(2)) <= b for a, b in rng.values()):
                        miss.append(f"clearance '{r['name']}' part {s_}")
    items = {i["name"] for i in ctx.S["mass"]["items"]}
    for k in L.get("mass_placement", {}):
        if k not in items:
            miss.append(f"mass_placement {k} not a mass item")
    acc = []
    for row in ctx.S["assembly"].get("maintenance_access", []):
        for a in row["access"]:
            if a.startswith("P-") and a not in pan_ids:
                acc.append(a)
    miss += [f"maintenance_access panel {a}" for a in acc]
    R.append(_row("C01", "references (touch, lands, windows, joints, clearance selectors, mass items, access panels) "
                  "resolve", not miss, len(miss), 0, "; ".join(miss[:10])))
    return R


# =====================================================================================================================
# C02 stations
# =====================================================================================================================
def station_x(s_: dict, y: float) -> float:
    return float(s_["x"]) + abs(float(y)) * math.tan(math.radians(float(s_.get("sweep_deg", 0.0))))


def _in_cutout(s_: dict, y: float, z: float, r: float = 0.0) -> str | None:
    for c in s_.get("cutouts", []):
        for sg in ((1.0, -1.0) if c.get("mirror") else (1.0,)):
            y0, y1 = sorted([sg * c["y"][0], sg * c["y"][1]])
            if y0 + r - 1e-9 <= y <= y1 - r + 1e-9 and c["z"][0] + r - 1e-9 <= z <= c["z"][1] - r + 1e-9:
                return c["id"]
    return None


def path_crossings(s_: dict, path) -> list:
    """Points where a polyline crosses the station plane (swept frames: x = x0 + |y| tan(sweep))."""
    P = np.asarray(path, float)
    out = []
    for i in range(len(P) - 1):
        a, b = P[i], P[i + 1]
        fa = a[0] - station_x(s_, a[1])
        fb = b[0] - station_x(s_, b[1])
        if fa == 0 and fb == 0:
            continue
        if fa * fb <= 0 and fa != fb:
            t = fa / (fa - fb)
            out.append(a + t * (b - a))
    return out


def check_stations(ctx: Ctx) -> list:
    S, L, af = ctx.S, ctx.L, ctx.af
    R = []
    st = L["stations"]
    xs = [float(s_["x"]) for s_ in st]
    R.append(_row("C02", "stations ordered along x, unique ids", xs == sorted(xs) and len({s_["id"] for s_ in st})
                  == len(st), None, None, ""))
    procs, mats, lays = S["processes"], S["materials"], S["layups"]
    bad = []
    for s_ in st:
        if s_["type"] not in STATION_TYPES:
            bad.append(f"{s_['id']} type {s_['type']}")
        if s_["material"] not in mats or s_["process"] not in procs or (s_.get("layup") and s_["layup"] not in lays):
            bad.append(f"{s_['id']} material/process/layup key")
        tmin = float(procs.get(s_["process"], {}).get("min_thickness", 0.0))
        if float(s_["t"]) < tmin - 1e-9:
            bad.append(f"{s_['id']} t {s_['t']} < process min {tmin}")
        x0, x1 = s_["x_faces"]
        if not (x0 < float(s_["x"]) + 1e-9 or abs(x0 - float(s_["x"]) + float(s_["t"])) < 1e-6) or \
                abs((x1 - x0) - float(s_["t"])) > 1e-4:
            bad.append(f"{s_['id']} x_faces")
    R.append(_row("C02", "stations: type, material/process/layup keys, thickness >= process minimum, faces", not bad,
                  len(bad), 0, "; ".join(bad)))
    # cut-outs inside the section
    bad = []
    for s_ in st:
        for c in s_.get("cutouts", []):
            if c.get("kind") == "bay":
                continue
            yc, zc = 0.5 * (c["y"][0] + c["y"][1]), 0.5 * (c["z"][0] + c["z"][1])
            x = station_x(s_, yc)
            if not ctx.inside([[x, yc, zc]], 0.004)[0]:
                bad.append(f"{s_['id']}.{c['id']}")
    R.append(_row("C02", "cut-outs (pass-throughs) inside the frame web", not bad, len(bad), 0, "; ".join(bad)))
    # frames do not cut fuel / turret / gear / payload / parachute / equipment volumes
    Zp = L["zones_preliminary"]
    vols = {"fuel:" + c["name"]: (c["x"], None) for c in L["fuel_cells"]}
    for k in ("turret_bay", "nose_gear_well", "main_gear_wells", "payload_bay", "parachute_bay", "mission_computer",
              "avionics_power_deck", "power_switching_bay", "avionics_side_bays", "forward_bay", "forward_bay_lower",
              "equipment_bay_aft"):
        if k in Zp:
            b = Zp[k]["box"]
            vols[k] = ([b[0][0], b[1][0]], b)
    bad = []
    for s_ in st:
        x0, x1 = s_["x_faces"]
        for k, (xr, b) in vols.items():
            if k.startswith("fuel:") and s_.get("sweep_deg", 0.0):
                continue                   # fuel bays conform to the swept spar frames (checked below: chevron bays)
            if s_["type"] == "ring" and b is not None and x0 < xr[1] and x1 > xr[0]:
                lo_, hi_ = np.asarray(b[0], float), np.asarray(b[1], float)
                ylo = -hi_[1] if lo_[1] <= 0 else lo_[1]
                Pz = np.array([[float(s_["x"]), yy, zz] for yy in (ylo, hi_[1]) for zz in (lo_[2], hi_[2])])
                if ctx.af.inside(Pz, 0.006 + float(s_.get("ring_depth", 0.04))).all():
                    continue
            if s_.get("sweep_deg", 0.0):
                ylo = 0.0 if b is None else (b[0][1] if b[0][1] > 0 else 0.0)
                yhi = 0.4 if b is None else b[1][1]
                xa, xb = station_x(s_, ylo) - 0.0034, station_x(s_, yhi) + 0.0034
            else:
                xa, xb = x0, x1
            if xb > xr[0] + 1e-6 and xa < xr[1] - 1e-6:
                cut_bay = any(c.get("kind") == "bay" for c in s_.get("cutouts", [])) and k == "payload_bay"
                if not cut_bay:
                    bad.append(f"{s_['id']} x[{xa:.4f},{xb:.4f}] cuts {k} x{_r(xr)}")
    R.append(_row("C02", "frames do not cut fuel cells, turret bay, gear wells, payload/parachute/equipment volumes "
                  "(except declared bay cut-outs; ring frames: the volume passes through the ring opening)", not bad,
                  len(bad), 0, "; ".join(bad[:8])))
    # fuel bays bounded by the swept spar frames (chevron): each cell boundary next to a spar frame follows the frame
    # (x + |y| tan(sweep)); gap to the frame web >= 13 mm and the usable volume >= the required fuel volume
    cells = {c["name"]: c for c in L["fuel_cells"]}
    fs = {s_["id"]: s_ for s_ in st}
    S_ = ctx.S
    eff = float(S_["structures"]["fuel"]["tank_volume_efficiency"])
    rho = float(S_["engine"]["fuel"]["density_kg_per_m3"])
    need = float(S_["mass"]["fuel_kg"]) / rho * (1 + float(S_["structures"]["fuel"]["expansion_fraction"]))
    bounds = {"forward_cell": (None, "FS-MS"), "saddle_cell": ("FS-MS", "FS-RS"), "aft_cell": ("FS-RS", None)}
    vol = 0.0
    gmin = 1.0
    for name, (fa, fb) in bounds.items():
        c = cells[name]
        g = np.array(np.meshgrid(np.arange(c["x"][0] - 0.06, c["x"][1] + 0.08, 0.005), np.arange(-0.42, 0.42, 0.005),
                                 np.arange(c["z"][0], c["z"][1] + 1e-9, 0.005), indexing="ij")).reshape(3, -1).T
        ins = ctx.af.inside(g, float(c.get("inset", 0.025)))
        ay = np.abs(g[:, 1])
        x0c = float(c["x"][0]) + (ay * math.tan(math.radians(float(fs[fa]["sweep_deg"]))) if fa else 0.0)
        x1c = float(c["x"][1]) + (ay * math.tan(math.radians(float(fs[fb]["sweep_deg"]))) if fb else 0.0)
        cell = ins & (g[:, 0] >= x0c) & (g[:, 0] <= x1c)
        vol += float(cell.sum()) * 0.005 ** 3 * eff
        for fid, side in ((fa, -1), (fb, +1)):
            if fid:
                f_ = fs[fid]
                xf = float(f_["x"]) + ay[cell] * math.tan(math.radians(float(f_["sweep_deg"])))
                d = (xf - 0.5 * float(f_["t"]) - g[cell, 0]) if side > 0 else (g[cell, 0] - xf - 0.5 * float(f_["t"]))
                gmin = min(gmin, float(d.min()))
    R.append(_row("C02", "fuel bays conform to the swept spar frames (chevron bladders): gap to the frame webs",
                  gmin >= 0.0, round(gmin * 1000, 1), ">= 0 mm", "cell ends follow x + |y| tan(sweep) of the spar frame"))
    R.append(_row("C02", "usable fuel volume of the chevron bays vs required (mass.fuel_kg, expansion space)",
                  vol >= need, round(vol * 1000, 2), f">= {need * 1000:.2f} L", f"tank efficiency {eff}"))
    # every trunk / pushrod / bridle crossing of a frame plane passes a declared cut-out
    paths = []
    for t in L["systems"]["harness"]["trunks"]:
        P = np.asarray(t["path"], float)
        paths.append((t["id"], P, 0.5 * float(t["diameter"])))
        if t.get("mirror"):
            paths.append((t["id"] + "@L", P * [1, -1, 1], 0.5 * float(t["diameter"])))
    for e in L["systems"]["equipment"]:
        lk = e.get("linkage")
        if lk:
            p0 = np.asarray(lk["servo_axis"], float) + [0, 0, float(lk["servo_arm_m"])]
            p1 = np.asarray(lk["horn_axis"], float) + [0, 0, float(lk["horn_m"])]
            paths.append((e["id"] + " pushrod", np.array([p0, p1]), 0.005))
            if e.get("mirror"):
                paths.append((e["id"] + " pushrod@L", np.array([p0, p1]) * [1, -1, 1], 0.005))
    br = L["chassis"]["parachute"]["bridle"]
    paths.append(("bridle aft leg", np.array([br["forward_leg"]["point"], br["aft_leg"]["point"]], float), 0.008))
    for k in L["keep_outs"]:
        if k["id"] == "KO-COOLING-DUCT":
            for dy in k["lateral_offsets"]:
                paths.append((f"cooling duct (y {dy:+.2f})", np.asarray(k["path"], float) + [0, dy, 0],
                              float(k["radius"])))
    bad, n = [], 0
    for name, P, r in paths:
        for s_ in st:
            for q in path_crossings(s_, P):
                if not ctx.af.inside([q], 0.0)[0] and not ctx.inside_wing([q], 0.0)[0]:
                    continue
                n += 1
                if _in_cutout(s_, q[1], q[2], r):
                    continue
                if s_["type"] == "ring":
                    if not ctx.af.inside([q], 0.006 + float(s_.get("ring_depth", 0.04)) + r)[0]:
                        bad.append(f"{name} x {s_['id']} at y {q[1]:.3f} z {q[2]:.3f}: in the ring web")
                    continue
                if not _in_cutout(s_, q[1], q[2], r):
                    bad.append(f"{name} x {s_['id']} at y {q[1]:.3f} z {q[2]:.3f}")
    R.append(_row("C02", f"harness / push-rod / bridle crossings of frame webs pass declared cut-outs ({n} crossings)",
                  not bad, len(bad), 0, "; ".join(bad[:10])))
    return R


# =====================================================================================================================
# C03 envelopes inside the OML
# =====================================================================================================================
def _box_points(box, n=4, skin=()):
    b = np.asarray(box, float)
    lo, hi = np.minimum(b[0], b[1]), np.maximum(b[0], b[1])
    g = [np.linspace(lo[k], hi[k], n) for k in range(3)]
    P = np.array(np.meshgrid(*g, indexing="ij")).reshape(3, -1).T
    on = np.zeros(len(P), bool)
    for k in range(3):
        on |= np.isclose(P[:, k], lo[k]) | np.isclose(P[:, k], hi[k])
    P = P[on]
    keep = np.ones(len(P), bool)
    if "top" in skin:
        keep &= ~np.isclose(P[:, 2], hi[2])
    if "bottom" in skin:
        keep &= ~np.isclose(P[:, 2], lo[2])
    if "sides" in skin:
        keep &= ~(np.isclose(P[:, 1], hi[1]) | np.isclose(P[:, 1], lo[1]))
    return P[keep]


def check_inside(ctx: Ctx) -> list:
    L = ctx.L
    R = []
    cv = L["clearance_values"]
    bc = L["rules"]["bay_contents"]["clearance_to_oml"]
    groups = []
    for e in L["systems"]["equipment"]:
        if e["id"] == "EQ-TURRET":
            continue
        groups.append(("equipment", e["id"], e.get("envelope_with_connector", e["box"]), float(bc),
                       bool(e.get("mirror")), (), None))
    contour = []
    for m in L["chassis"]["members"]:
        if "main_spar_line" in m:
            groups.append(("member", m["id"], None, 0.0, False, (), ctbox_prims(m)))
        elif m.get("contour") == "wing_loft":
            contour.append(m)
        elif "box" in m:
            groups.append(("member", m["id"], m["box"], 0.0, bool(m.get("mirror")),
                           tuple(k for k in ("top", "bottom", "sides") if m.get(k) == "skin"), None))
    for f in L["chassis"]["fittings"]:
        if "box" in f:
            groups.append(("fitting", f["id"], f["box"], 0.0, bool(f.get("mirror")), (), [obb_of(f)]))
    for a in L["systems"]["actuators"]:
        groups.append(("actuator", a["id"], a["box"], 0.003, bool(a.get("mirror")), (), [obb_of(a)]))
    bad = {}
    for kind, oid, box, m, mir, skin, prims in groups:
        if prims is not None:
            P = np.vstack([p.corners() if isinstance(p, OBB) else p.samples(0.01) if isinstance(p, Cyl) else
                           np.array([p.p0, p.p1]) for p in prims])
            if isinstance(prims[0], OBB):
                P = np.vstack([P, prims[0].samples(0.01)])
        else:
            P = _box_points(box, 4, skin)
        if mir:
            P = np.vstack([P, P * [1, -1, 1]])
        ok = ctx.inside(P, m, tail=True)
        if skin:                                      # faces declared on the skin: points beyond the skin side count
            zc_ = np.array([float(ctx.af.sec(x)[3][0]) for x in P[:, 0]])
            beyond = np.zeros(len(P), bool)
            if "bottom" in skin:
                beyond |= P[:, 2] < zc_
            if "top" in skin:
                beyond |= P[:, 2] > zc_
            if "sides" in skin:
                beyond |= np.ones(len(P), bool)
            ok |= beyond & ~ctx.af.inside(P, 0.0) & ctx.af.inside(P * [1, 0, 1], 0.0)
        if not ok.all():
            bad.setdefault(kind, []).append(f"{oid} ({int((~ok).sum())} pts, e.g. {_r(P[~ok][0], 3)})")
    for m in contour:                                  # ribs trimmed to the wing loft: chord-line extent inside
        b = np.asarray(m["box"], float)
        y = 0.5 * (b[0][1] + b[1][1])
        w = ctx._wing_section(y)
        if w is None:
            bad.setdefault("member", []).append(f"{m['id']} (no loft at y {y:.3f})")
            continue
        xu, zu, xl, zl = w
        if b[0][0] < max(xu[0], xl[0]) - 1e-3 or b[1][0] > min(xu[-1], xl[-1]) + 1e-3:
            bad.setdefault("member", []).append(f"{m['id']} chord extent outside the loft at y {y:.3f}")
    for kind in ("equipment", "member", "fitting", "actuator"):
        lim = {"equipment": f">= {bc * 1000:.0f} mm", "actuator": ">= 3 mm"}.get(kind, ">= 0 (skin faces excepted)")
        R.append(_row("C03", f"{kind} envelopes inside the OML", kind not in bad, len(bad.get(kind, [])), lim,
                      "; ".join(bad.get(kind, [])[:6])))
    bad = []
    for t in L["systems"]["harness"]["trunks"]:
        for c in polyline(t["path"], 0.5 * float(t["diameter"])):
            P = np.array([c.p0 + f * (c.p1 - c.p0) for f in np.linspace(0, 1, 12)])
            ok = ctx.inside(P, c.r + 0.003)
            if not ok.all():
                bad.append(f"{t['id']} near {_r(P[~ok][0], 3)}")
                break
    R.append(_row("C03", "harness trunks inside the OML (radius + 3 mm)", not bad, len(bad), ">= r + 3 mm",
                  "; ".join(bad)))
    em = L["chassis"]["engine_mount"]
    P = np.array([np.asarray(t["a"]) + f * (np.asarray(t["b"]) - np.asarray(t["a"])) for t in em["tubes"]
                  for f in np.linspace(0, 1, 6)] + [np.asarray(n) for n in em["ring_nodes"]])
    ok = ctx.inside(P, 0.008 + 0.005)
    R.append(_row("C03", "engine mount truss inside the cowl (tube radius + 5 mm)", ok.all(), int((~ok).sum()), 0,
                  "" if ok.all() else str(_r(P[~ok][0], 3))))
    T = turret_prims(ctx, 0.0)
    ok = ctx.inside(T[0].samples(0.01), 0.0)
    R.append(_row("C03", "turret growth envelope (retracted) inside the OML", bool(ok.all()), int((~ok).sum()), 0, ""))
    return R


# =====================================================================================================================
# C04 static overlaps
# =====================================================================================================================
CONTENT_KINDS = ("content", "harness", "fuel", "gear", "engine", "exhaust", "mount", "linkage")


def _allowed(a: Obj, b: Obj) -> float | None:
    """None: pair not checked; otherwise the minimum gap (m) required (negative = contact tolerance)."""
    ka, kb = a.kind, b.kind
    pair = {ka, kb}
    if "external" in pair:
        return None
    if ka == "structure" and kb == "structure":
        return None
    if pair == {"engine", "mount"} or pair == {"engine", "exhaust"} or pair == {"engine", "linkage"}:
        return None
    if ka == "harness" and kb == "harness":
        return None
    if "harness" in pair:
        other = b if ka == "harness" else a
        h = a if ka == "harness" else b
        if other.kind == "engine":
            return None if h.id.startswith("H-ENGINE") else 0.0
        if other.kind in ("content", "structure", "linkage"):
            return -0.003                      # trunks start at equipment and run along members (clamped)
        return 0.0
    if "linkage" in pair:
        other = b if ka == "linkage" else a
        lk = a if ka == "linkage" else b
        if other.kind in ("content",) and lk.id.split("-PUSHROD")[0] == other.id.split("@")[0]:
            return None                        # pushrod on its own actuator
        if other.kind == "structure":
            return -0.002
        return 0.0
    if "mount" in pair:
        other = b if ka == "mount" else a
        if other.kind == "structure":
            return None if other.id.startswith(("F-EMOUNT",)) else 0.0
        return 0.0
    if "structure" in pair:
        return -0.002                           # equipment resting on trays / decks / walls
    if {a.id.split("@")[0], b.id.split("@")[0]} == {"TURRET", "EQ-ELEVATOR"}:
        return -0.002
    if a.id.startswith(("RAIL-", "ELEV-")) or b.id.startswith(("RAIL-", "ELEV-")):
        other = b if a.id.startswith(("RAIL-", "ELEV-")) else a
        if other.id.split("@")[0] in ("EQ-ELEVATOR",):
            return None
    return 0.0


def _penetration_ok(ctx: Ctx, a: Obj, b: Obj, req: float) -> bool:
    """A harness trunk may pass a member only at a declared penetration (trunk 'penetrations': member + point)."""
    h, m = (a, b) if a.kind == "harness" else (b, a) if b.kind == "harness" else (None, None)
    if h is None or m.kind != "structure":
        return False
    tid = h.id.split("@")[0]
    side = h.id.split("@")[1] if "@" in h.id else ""
    t = next((t for t in ctx.L["systems"]["harness"]["trunks"] if t["id"] == tid), None)
    if not t:
        return False
    pens = [np.asarray(pn["point"], float) * ([1, -1, 1] if side == "L" else [1, 1, 1])
            for pn in t.get("penetrations", []) if pn["member"] == m.id.split("@")[0]]
    if not pens:
        return False
    P = np.vstack([p.samples(0.004) for p in h.prims])
    d = np.min(np.vstack([pr.sdf(P) for pr in m.prims]), axis=0)
    bad = P[d < req - 1e-9]
    r = max(p.r for p in h.prims if isinstance(p, Capsule)) + 0.015
    return all(min(float(np.linalg.norm(q - c)) for c in pens) <= r + 0.02 for q in bad)


def check_overlaps(ctx: Ctx, objs: list) -> tuple[list, list]:
    R, pairs = [], []
    B = {o.id: _bounds(o.prims) for o in objs}
    viol = []
    n = 0
    for i in range(len(objs)):
        for j in range(i + 1, len(objs)):
            a, b = objs[i], objs[j]
            req = _allowed(a, b)
            if req is None:
                continue
            la, ha = B[a.id]
            lb, hb = B[b.id]
            if np.maximum(lb - ha, la - hb).max() > 0.01:
                continue
            n += 1
            g = gap(a.prims, b.prims, cutoff=0.02, step=0.006)
            pairs.append((a.id, b.id, g, req))
            if g < req - 1e-9 and _penetration_ok(ctx, a, b, req):
                continue
            if g < req - 1e-9:
                viol.append(f"{a.id} / {b.id}: {g * 1000:.1f} mm (need {req * 1000:.0f})")
    R.append(_row("C04", f"no overlaps between contents (equipment, fuel cells, harness, turret, stowed gear, engine, "
                  f"mount, exhaust, linkages) and with structure ({n} close pairs evaluated)", not viol, len(viol), 0,
                  "; ".join(viol[:12])))
    return R, pairs


# =====================================================================================================================
# C05 mechanisms (swept volumes)
# =====================================================================================================================
def _door_main(ctx: Ctx, side: str, ang: float) -> OBB:
    L, S = ctx.L, ctx.S
    J = {j["name"]: j for j in L["mechanisms"]["joints"]}
    j = J["main_inner_door_" + side]
    wb = L["zones_preliminary"]["main_gear_wells"]["box"]
    y0 = abs(float(j["origin"][1]))
    y1 = float(S["landing_gear"]["doors"]["main_inner_door_outer_edge_y"])
    sg = 1.0 if side == "R" else -1.0
    zc = float(j["origin"][2])
    c = np.array([0.5 * (wb[0][0] + wb[1][0]), sg * 0.5 * (y0 + y1), zc])
    door = OBB(c, np.eye(3), [0.5 * (wb[1][0] - wb[0][0]), 0.5 * (y1 - y0), 0.002])
    o, a = np.asarray(j["origin"], float), _unit(j["axis"])
    M = _rot(a, ang)
    return door.moved(M, o - M @ o)


def _door_turret_points(ctx: Ctx, side: str, travel: float) -> np.ndarray:
    """Upper (inner) face of a sliding bay door: the door (payload.turret.bay.door_thickness) lies on the inner face
    of the belly skin and slides along it; closed it covers 0..0.105 m from the centre line."""
    S = ctx.S
    T = S["payload"]["turret"]
    xc = float(T["bay_center_x"])
    tb = ctx.L["zones_preliminary"]["turret_bay"]["box"]
    t_d = float(T["bay"]["door_thickness"])
    sg = 1.0 if side == "R" else -1.0
    xs = np.linspace(tb[0][0], tb[1][0], 9)
    pts = []
    for x in xs:
        ys = np.linspace(0.0, 0.25, 251)
        zb = np.array([float(ctx.af.z_bot(x, y)) for y in ys])
        s_ = np.r_[0.0, np.cumsum(np.hypot(np.diff(ys), np.diff(zb)))]           # arc length along the skin
        for a in np.linspace(travel, travel + 0.105, 22):
            y = float(np.interp(a, s_, ys))
            pts.append([x, sg * y, float(np.interp(a, s_, zb)) + t_d])
    return np.asarray(pts)


def _surface_cloud(ctx: Ctx, srf, etas, xc0=0.0, n=41):
    pts = []
    for eta in etas:
        loop = srf.loop_at(float(eta), n)
        o, cdir, u, _ = srf.frame_at(float(eta))
        ch = srf.chord_at(float(eta))
        s = (loop - o) @ cdir / ch
        pts.append(loop[s >= xc0 - 1e-9])
    return np.vstack(pts)


def check_mechanisms(ctx: Ctx, objs: list) -> list:
    S, L = ctx.S, ctx.L
    R = []
    cv = L["clearance_values"]
    J = {j["name"]: j for j in L["mechanisms"]["joints"]}
    seq = L["mechanisms"]["sequences"]
    static = [o for o in objs if o.kind in ("structure", "content", "harness", "fuel", "linkage")]
    # ---------------------------------------------------------------- landing gear + doors
    worst = {}

    def upd(key, g, who):
        if key not in worst or g < worst[key][0]:
            worst[key] = (g, who)
    for st in seq["gear_retraction"]["states"]:
        for side in ("R", "L"):
            g = gear_prims(ctx, "main", float(st["main_gear_" + side]), side)
            door = _door_main(ctx, side, float(st["main_inner_door_" + side]))
            for o in static:
                if o.id.startswith(("F-TRUNNION", "M-GEARBEAM", "M-WELLROOF")) and o.kind == "structure":
                    excl_leg = True
                else:
                    excl_leg = False
                gt = gap([g["tyre"]], o.prims, cutoff=0.03)
                upd("main tyre vs structure/contents", gt, f"{o.id} at gear_up state {st['main_gear_' + side]:.2f}")
                if not excl_leg:
                    gl = gap([g["leg"]], o.prims, cutoff=0.03)
                    upd("main leg vs structure/contents", gl, o.id)
            for gg in (g["tyre"], g["leg"]):
                upd("main inner door vs moving gear", gap([door], [gg], cutoff=0.05), f"side {side}")
        gn = gear_prims(ctx, "nose", float(st["nose_gear"]))
        for o in static:
            upd("nose tyre vs structure/contents", gap([gn["tyre"]], o.prims, cutoff=0.03), o.id)
            if not o.id.startswith(("F-NG-PIVOT", "M-KEELWALL")):
                upd("nose leg vs structure/contents", gap([gn["leg"]], o.prims, cutoff=0.03), o.id)
    lim = {"main tyre vs structure/contents": float(cv["tyre_to_well"]),
           "nose tyre vs structure/contents": float(cv["tyre_to_well"]),
           "main leg vs structure/contents": float(cv["harness_to_moving_parts"]),
           "nose leg vs structure/contents": float(cv["harness_to_moving_parts"]),
           "main inner door vs moving gear": float(cv["door_to_moving_gear"])}
    for k, (g, who) in worst.items():
        R.append(_row("C05", f"gear retraction sequence ({len(seq['gear_retraction']['states'])} states): {k}",
                      g >= lim[k] - 1e-9, round(g * 1000, 1), f">= {lim[k] * 1000:.0f} mm", who))
    # ---------------------------------------------------------------- turret elevator + sliding doors
    walls = [o for o in objs if o.id.split("@")[0] in ("M-TURRETWALL", "M-TURRETROOF")]
    tb = L["zones_preliminary"]["turret_bay"]["box"]
    fr = [s_ for s_ in L["stations"] if s_["id"] in ("FS1110", "FS1330")]
    plates = [OBB.aabb([[s_["x_faces"][0], -0.12, -0.21], [s_["x_faces"][1], 0.12, 0.11]]) for s_ in fr]
    rails = [o for o in objs if o.id.startswith(("RAIL-", "ELEV-SCREW"))]
    w_min = d_min = r_min = 1.0
    for st in seq["turret_extension"]["states"]:
        T = turret_prims(ctx, float(st["turret_elevator"]))
        w_min = min(w_min, min(gap(T, o.prims, cutoff=0.05) for o in walls), gap(T, plates, cutoff=0.05))
        r_min = min(r_min, min(gap(T, o.prims, cutoff=0.05) for o in rails))
        for sd in ("R", "L"):
            Pd = _door_turret_points(ctx, sd, float(st["turret_door_" + sd]))
            d_min = min(d_min, float(np.min([p.sdf(Pd).min() for p in T])))
    R.append(_row("C05", "turret E180 envelope along the elevator stroke vs bay walls, roof and frames FS1110/FS1330",
                  w_min >= float(cv["turret_to_bay_wall"]) - 1e-9, round(w_min * 1000, 1),
                  f">= {cv['turret_to_bay_wall'] * 1000:.0f} mm"))
    R.append(_row("C05", "turret E180 envelope vs elevator rails / ball screw", r_min >= 0.005 - 1e-9,
                  round(r_min * 1000, 1), ">= 5 mm"))
    R.append(_row("C05", "turret E180 envelope vs sliding bay doors over the coupled sequence",
                  d_min >= float(cv["turret_to_aperture_ring"]) - 1e-9, round(d_min * 1000, 1),
                  f">= {cv['turret_to_aperture_ring'] * 1000:.0f} mm"))
    T_ = S["payload"]["turret"]
    ring = float(T_["aperture_radius"]) if float(T_["aperture_radius"]) > 0.05 else \
        0.5 * float(T_["ball_diameter"]) + float(T_["bay"]["aperture_ring"]["radial_clearance"])
    rc = ring - 0.5 * float(T_["ball_diameter"])
    R.append(_row("C05", "HD59 ball vs aperture ring (radial)", rc >= 0.005 - 1e-9, round(rc * 1000, 1), ">= 5 mm",
                  f"ring radius {ring:.4f} m"))
    # ---------------------------------------------------------------- stabilators vs exhaust hot zones and plumes
    af = ctx.af
    st_srf = af.tail["stabilator"]
    e = st_srf.span_coords()
    cloud = _surface_cloud(ctx, st_srf, np.linspace(e[0], e[-1], 9))
    js = J["stabilator_R"]
    ex = [k for k in L["keep_outs"] if k["id"].startswith("KO-EXHAUST")]
    m_box, m_plume = 1.0, 1.0
    for side, sg in (("R", 1.0), ("L", -1.0)):
        C0 = cloud * [1, sg, 1]
        o = np.asarray(js["origin"], float) * [1, sg, 1]
        for ang in np.linspace(float(js["lo"]), float(js["hi"]), 7):
            M = _rot([0, 1.0, 0], ang)
            P = (C0 - o) @ M.T + o
            for k in ex:
                for b in k["boxes"]:
                    m_box = min(m_box, float(OBB.aabb(b).sdf(P).min()))
                m_plume = min(m_plume, float(_plume_sdf(k, P).min()))
    R.append(_row("C05", "stabilators (whole range, both sides) vs exhaust routing envelopes",
                  m_box >= ex[0]["margin_composite"] - 1e-9, round(m_box * 1000, 1),
                  f">= {ex[0]['margin_composite'] * 1000:.0f} mm"))
    R.append(_row("C05", "stabilators (whole range) outside the exhaust plume cones", m_plume >= 0.0,
                  round(m_plume * 1000, 1), ">= 0"))
    # ---------------------------------------------------------------- ailerons / flaps vs wing actuators
    acts = {a["id"]: a for a in L["systems"]["actuators"]}
    m_act = 1.0
    W = S["wing"]
    b2 = 0.5 * float(W["span"])
    for k in ("aileron", "flap"):
        c = W["controls"][k]
        j = J[f"{k}_R"]
        o, ax = np.asarray(j["origin"], float), _unit(j["axis"])
        ys = np.linspace(float(c["eta0"]) * b2, float(c["eta1"]) * b2, 9)
        pts = []
        for y in ys:
            w = ctx._wing_section(y)
            if w is None:
                continue
            xu, zu, xl, zl = w
            xh = o[0] + (y - o[1]) * ax[0] / max(ax[1], 1e-9)
            for x in np.linspace(xh, max(xu[-1], xl[-1]), 6):
                pts += [[x, y, np.interp(x, xu, zu)], [x, y, np.interp(x, xl, zl)]]
        P0 = np.asarray(pts, float)
        for ang in np.linspace(float(j["lo"]), float(j["hi"]), 7):
            M = _rot(ax, ang)
            P = (P0 - o) @ M.T + o
            for a in acts.values():
                m_act = min(m_act, float(OBB.aabb(a["box"]).sdf(P).min()))
    R.append(_row("C05", "ailerons / flaps (whole range) vs wing actuators", m_act >= 0.003, round(m_act * 1000, 1),
                  ">= 3 mm"))
    # ---------------------------------------------------------------- parachute hatch
    hp = next(p for p in L["shell"]["panels"] if p["id"] == "P-PARAHATCH")
    jh = J["para_hatch"]
    z_h = float(jh["origin"][2])
    hatch = OBB.aabb([[hp["x"][0], hp["y"][0], z_h - 0.002], [hp["x"][1], hp["y"][1], z_h + 0.002]])
    ext = [o for o in objs if o.kind == "external"]
    m_h = 1.0
    for ang in np.linspace(0.0, float(jh["hi"]), 12):
        M = _rot(jh["axis"], ang)
        o = np.asarray(jh["origin"], float)
        hh = hatch.moved(M, o - M @ o)
        for x in ext:
            m_h = min(m_h, gap([hh], x.prims, cutoff=0.1))
    R.append(_row("C05", "parachute hatch (0-110 deg) vs external antennas, probes, lights", m_h >= 0.01,
                  round(m_h * 1000, 1), ">= 10 mm"))
    return R


def _plume_sdf(k: dict, P: np.ndarray) -> np.ndarray:
    """Approximate signed distance to the exhaust plume cone (apex radius 15 mm at the exit, half angle, length)."""
    e = np.asarray(k["exit"]["point"], float)
    d = _unit(k["exit"]["direction"])
    th = math.radians(float(k["plume_cone_half_angle_deg"]))
    Lp = float(k["plume_length"])
    Q = np.atleast_2d(P) - e
    t = Q @ d
    rad = np.linalg.norm(Q - t[:, None] * d, axis=1)
    rr = 0.015 + np.clip(t, 0, Lp) * math.tan(th)
    d_rad = (rad - rr) * math.cos(th)
    d_ax = np.maximum(-t, t - Lp)
    return np.where(d_ax > 0, np.sqrt(np.maximum(d_rad, 0) ** 2 + d_ax ** 2), np.maximum(d_rad, d_ax))


# =====================================================================================================================
# C06 mass placement / CG
# =====================================================================================================================
CG_TOL_ITEM = 0.002       # m, placed centroid vs spec mass item (sizing applies layout.mass_placement)
CG_TOL_TOTAL = 0.001      # m, empty CG from the spec items vs empty CG with the recomputed placements


def check_cg(ctx: Ctx) -> tuple[list, dict]:
    S, L = ctx.S, ctx.L
    R = []
    mp = mass_placements(ctx)
    spec_mp = L.get("mass_placement", {})
    items = {i["name"]: i for i in S["mass"]["items"]}
    d1, d2 = [], []
    for k, v in mp.items():
        p = np.asarray(v["position"], float)
        if k in spec_mp:
            d1.append((k, float(np.abs(p - np.asarray(spec_mp[k]["position"], float)).max())))
        it = items.get(k)
        if it:
            d2.append((k, float(np.abs(p - np.array([it["x"], it["y"], it["z"]], float)).max())))
    missing = [k for k in mp if k not in spec_mp]
    w1 = max(d1, key=lambda t: t[1]) if d1 else ("-", 0.0)
    w2 = max(d2, key=lambda t: t[1]) if d2 else ("-", 0.0)
    R.append(_row("C06", "layout.mass_placement = centroids recomputed from the layout objects", not missing and
                  w1[1] <= CG_TOL_ITEM, round(w1[1] * 1000, 2), f"<= {CG_TOL_ITEM * 1000:.0f} mm", f"worst {w1[0]}"))
    R.append(_row("C06", "spec mass items at the placed centroids (sizing --update-spec applied the placement)",
                  w2[1] <= CG_TOL_ITEM, round(w2[1] * 1000, 2), f"<= {CG_TOL_ITEM * 1000:.0f} mm", f"worst {w2[0]}"))
    m = np.array([float(i["mass_kg"]) for i in S["mass"]["items"]])
    P = np.array([[i["x"], i["y"], i["z"]] for i in S["mass"]["items"]], float)
    cg_spec = (m[:, None] * P).sum(0) / m.sum()
    P2 = P.copy()
    for n_, i in enumerate(S["mass"]["items"]):
        if i["name"] in mp:
            P2[n_] = mp[i["name"]]["position"]
    bc = {it["name"]: it for it in L["rules"]["bay_contents"]["items"]}
    for n_, i in enumerate(S["mass"]["items"]):                       # bay-content items (sizing centroids)
        names = {"buffer_battery_12S2P_liion": ("buffer_battery",), "pdu_dcdc_fuses": ("pdu", "dcdc_28_12",
                                                                                        "contactor_fuses"),
                 "avionics": ("autopilot", "datalink_primary", "datalink_backup", "transponder", "remote_id")}.get(
            i["name"])
        if names:
            mm = np.array([float(bc[k]["mass_kg"]) for k in names])
            cc = np.array([bc[k]["center"] for k in names], float)
            P2[n_] = (mm[:, None] * cc).sum(0) / mm.sum()
    cg_lay = (m[:, None] * P2).sum(0) / m.sum()
    dcg = cg_lay - cg_spec
    R.append(_row("C06", "empty-aircraft CG: spec mass items vs layout-placed items", float(np.abs(dcg).max()) <=
                  CG_TOL_TOTAL, _r(dcg * 1000, 2), f"<= {CG_TOL_TOTAL * 1000:.0f} mm",
                  f"spec CG {_r(cg_spec)} m, layout CG {_r(cg_lay)} m, empty {m.sum():.2f} kg"))
    info = {"cg_spec": _r(cg_spec), "cg_layout": _r(cg_lay), "empty_kg": round(float(m.sum()), 3),
            "placements": mp}
    return R, info


# =====================================================================================================================
# C07 turret field of regard, RF windows
# =====================================================================================================================
def check_fov_rf(ctx: Ctx, objs: list) -> list:
    S, L = ctx.S, ctx.L
    R = []
    T = S["payload"]["turret"]
    apex = np.array([float(T["bay_center_x"]), 0.0, float(T["ball_center_extended_z"])])
    lim = float(next(k for k in L["keep_outs"] if k["id"] == "KO-TURRET-FOV")["min_clear_elevation_deg"])
    worst = (90.0, "")
    for o in objs:
        if o.kind != "external":
            continue
        P = np.vstack([p.samples(0.005) for p in o.prims])
        h = np.linalg.norm(P[:, :2] - apex[:2], axis=1)
        el = np.degrees(np.arctan2(P[:, 2] - apex[2], np.maximum(h, 1e-6)))
        if float(el.min()) < worst[0]:
            worst = (float(el.min()), o.id)
    R.append(_row("C07", "turret field of regard: every external protrusion above the -5 deg cone from the extended "
                  "ball centre", worst[0] >= lim, round(worst[0], 2), f">= {lim} deg", f"lowest: {worst[1]}"))
    pans = {p["part"]: p for p in L["shell"]["panels"]}
    bad = []
    for a in L["systems"]["antennas"]:
        w = str(a["window"])
        if w.startswith("external"):
            continue
        base = re.sub(r"-[LR]$", "", w)
        if base in pans:
            p = pans[base]
            mat = str(p.get("material", ""))
            if not p.get("rf_window") or mat.startswith("cfrp"):
                bad.append(f"{a['id']}: window {w} not RF-transparent ({mat})")
            x, y = a["point"][0], a["point"][1]
            ys = sorted(p["y"]) if not p.get("mirror") else sorted([abs(v) for v in p["y"]])
            yy = abs(y) if p.get("mirror") else y
            if not (p["x"][0] - 1e-6 <= x <= p["x"][1] + 1e-6) or (p["surface"] != "body_full" and
                                                                  not ys[0] - 1e-6 <= yy <= ys[1] + 1e-6):
                bad.append(f"{a['id']}: not under its window {w}")
        elif w.startswith("YK250-TL-258"):
            if "gfrp" not in (a.get("notes", "") + a.get("name", "")).lower() and "GFRP" not in a.get("notes", ""):
                bad.append(f"{a['id']}: fin-tip cap window material not stated as GFRP")
        else:
            bad.append(f"{a['id']}: window {w} unknown")
    R.append(_row("C07", "RF windows: every internal antenna under a GFRP (RF-transparent) panel / tip cap", not bad,
                  len(bad), 0, "; ".join(bad)))
    gn = [a for a in L["systems"]["antennas"] if "GNSS" in a["id"]]
    up = all(pans[a["window"]]["surface"] in ("body_upper",) for a in gn if a["window"] in pans)
    R.append(_row("C07", "GNSS antennas under upper-surface windows (sky view)", up, len(gn), None, ""))
    return R


# =====================================================================================================================
# C08 keep-outs and separation rules
# =====================================================================================================================
def check_keepouts(ctx: Ctx, objs: list) -> list:
    S, L = ctx.S, ctx.L
    R = []
    cv = L["clearance_values"]
    by = {o.id: o for o in objs}
    fuel = [o for o in objs if o.kind == "fuel"]
    bat = by.get("EQ-BUFFER_BATTERY")
    if bat is None:
        R.append(_row("C08", "buffer battery present", False, None, None, ""))
    else:
        d = min(gap(bat.prims, f.prims, cutoff=3.0, step=0.02) for f in fuel)
        R.append(_row("C08", "Li-ion buffer battery to every fuel cell", d >= float(cv["battery_to_fuel"]) - 1e-9,
                      round(d, 3), f">= {cv['battery_to_fuel']} m", "engine.sources.buffer_battery"))
    fwd_face = next(s_["x_faces"][0] for s_ in L["stations"] if s_.get("subtype") == "firewall")
    d = fwd_face - max(float(c["x"][1]) for c in L["fuel_cells"])
    R.append(_row("C08", "fuel cells to the firewall forward face (CS-LUAS.967(c))", d >= float(cv["fuel_to_firewall"]),
                  round(d, 4), f">= {cv['fuel_to_firewall']} m"))
    eng = by["ENGINE"]
    allowed = ("ENGINE-MOUNT", "KO-EXHAUST", "H-ENGINE", "EQ-STABACT")
    worst = (1.0, "")
    for o in objs:
        if o is eng or o.kind == "fuel" or o.id.startswith(allowed):
            continue
        g = gap(eng.prims, o.prims, cutoff=0.05)
        if g < worst[0]:
            worst = (g, o.id)
    R.append(_row("C08", "engine dynamic envelope vs every other layout object", worst[0] >= float(cv["engine_keep_out"])
                  - 1e-6, round(worst[0] * 1000, 1), f">= {cv['engine_keep_out'] * 1000:.0f} mm", worst[1]))
    em = L["chassis"]["engine_mount"]
    pr = S["propeller"]
    hub_face = float(pr["hub_spacer"]) + float(pr["hub_half_thickness"])
    sg_front = hub_face + float(S["engine"]["envelope"]["length_with_sg750"])
    sg = Cyl(ctx.engine_point(sg_front - 0.0143), ctx.d_thrust, 0.0505, 0.0143)
    tubes = [Capsule(t["a"], t["b"], 0.008) for t in em["tubes"]]
    g = gap([sg], tubes, cutoff=0.05)
    R.append(_row("C08", "engine-mount truss vs SG750 (d 101 x 28.6 mm) on its isolators", g >= float(cv[
        "engine_keep_out"]) - 1e-6, round(g * 1000, 1), f">= {cv['engine_keep_out'] * 1000:.0f} mm"))
    cyl = OBB(eng.prims[1].c, eng.prims[1].R, eng.prims[1].h)
    worst = (1.0, "")
    for o in objs:
        comp = o.kind == "structure" and str(o.material).startswith(("cfrp", "gfrp"))
        if comp:
            g = gap([cyl], o.prims, cutoff=0.06)
            if g < worst[0]:
                worst = (g, o.id)
    R.append(_row("C08", "cylinder/head hot zone vs composite structure", worst[0] >= float(
        cv["composite_to_cylinder_heads"]) - 1e-9, round(worst[0] * 1000, 1),
        f">= {cv['composite_to_cylinder_heads'] * 1000:.0f} mm", worst[1]))
    ex = [o for o in objs if o.kind == "exhaust"]
    worst_c, worst_h = (1.0, ""), (1.0, "")
    for o in objs:
        if o.kind == "exhaust" or o.id in ("ENGINE",):
            continue
        g = min(gap(e.prims, o.prims, cutoff=0.1) for e in ex)
        if o.kind == "harness" and g < worst_h[0]:
            worst_h = (g, o.id)
        if o.kind == "structure" and str(o.material).startswith(("cfrp", "gfrp")) and g < worst_c[0]:
            worst_c = (g, o.id)
    R.append(_row("C08", "exhaust routing envelopes vs unshielded composite structure", worst_c[0] >=
                  float(cv["composite_to_exhaust"][1]) - 1e-9, round(worst_c[0] * 1000, 1),
                  f">= {cv['composite_to_exhaust'][1] * 1000:.0f} mm", worst_c[1]))
    R.append(_row("C08", "exhaust routing envelopes vs harness", worst_h[0] >= float(cv["harness_to_exhaust"]) - 1e-9,
                  round(worst_h[0] * 1000, 1), f">= {cv['harness_to_exhaust'] * 1000:.0f} mm", worst_h[1]))
    af = ctx.af
    vs = af.tail["ventral"]
    e = vs.span_coords()
    vent = _surface_cloud(ctx, vs, np.linspace(e[0], e[-1], 7))
    kx = [k for k in L["keep_outs"] if k["id"].startswith("KO-EXHAUST")]
    gb = min(float(OBB.aabb(b).sdf(vent).min()) for k in kx for b in k["boxes"])
    gp = min(float(_plume_sdf(k, vent).min()) for k in kx)
    R.append(_row("C08", "ventral fin vs exhaust routing envelopes / plume", gb >= float(cv["composite_to_exhaust"][1])
                  - 1e-9 and gp >= 0.0, round(min(gb, gp) * 1000, 1), f">= {cv['composite_to_exhaust'][1] * 1000:.0f} "
                  "mm (boxes) / outside (plume)", f"boxes {gb * 1000:.1f} mm, plume {gp * 1000:.1f} mm"))
    ko = next(k for k in L["keep_outs"] if k["id"] == "KO-PROP")
    disc = Cyl(ko["centre"], ko["axis"], float(ko["radius"]), float(ko["half_thickness"]))
    hub = Cyl(ko["centre"], ko["axis"], float(ko["hub_radius"]) + 0.01, float(ko["half_thickness"]) + 0.01)
    bad = []
    for o in objs:
        if o.kind in ("engine",):
            continue
        P = np.vstack([p.samples(0.006) for p in o.prims])
        ins = (disc.sdf(P) < 0) & (hub.sdf(P) > 0)
        if ins.any():
            bad.append(o.id)
    R.append(_row("C08", "propeller disc keep-out (R + 26 mm, blade half extent + 13 mm) free of layout objects",
                  not bad, len(bad), 0, ", ".join(bad)))
    pd = next(k for k in L["keep_outs"] if k["id"] == "KO-PARA-DEPLOY")
    box = OBB.aabb(pd["box"])
    bad = [o.id for o in objs if o.kind == "external" and any(float(box.sdf(p.samples(0.01)).min()) < 0
                                                               for p in o.prims)]
    R.append(_row("C08", "parachute deployment volume free of antennas, probes, lights", not bad, len(bad), 0,
                  ", ".join(bad)))
    return R


# =====================================================================================================================
# C09 shell, C10 maintenance reachability
# =====================================================================================================================
FASTENER_D = {"camloc": 0.0048, "nutplate+screw": 0.004, "insert+screw": 0.004}


def check_shell(ctx: Ctx) -> list:
    S, L = ctx.S, ctx.L
    R = []
    bad = []
    for p in L["shell"]["panels"]:
        f = p["fastening"]
        D = FASTENER_D.get(f["type"])
        mat = str(p.get("material", ""))
        comp = mat.startswith(("cfrp", "gfrp"))
        if D:
            em = float(f.get("edge_margin") or 0.0)
            k = 2.5 if comp else 2.0
            if em < k * D - 1e-9:
                bad.append(f"{p['id']} edge {em * 1000:.1f} mm < {k} D")
            pt = f.get("pitch")
            if pt and float(pt[0]) < 3 * D - 1e-9:
                bad.append(f"{p['id']} pitch {pt[0]} < 3 D")
            if pt and p["attach"] == "fixed" and f["type"] == "nutplate+screw" and float(pt[1]) > 8 * D + 1e-9:
                bad.append(f"{p['id']} structural pitch {pt[1]} > 8 D")
        if p["attach"] == "hinged" and not (p.get("hinge") or p.get("joint")) and f.get("type") != "hinge+latch":
            bad.append(f"{p['id']} hinged without hinge")
        if p.get("rf_window") and mat.startswith("cfrp"):
            bad.append(f"{p['id']} RF window in CFRP")
        if mat not in S["materials"]:
            bad.append(f"{p['id']} material {mat}")
    R.append(_row("C09", "shell panels: edge margin >= 2.5 D (composite) / 2 D (metal), pitch >= 3 D, structural "
                  "nutplate pitch <= 8 D, hinges, RF materials", not bad, len(bad), 0, "; ".join(bad[:8])))
    xs = sorted([p["x"] for p in L["shell"]["panels"] if p["surface"] in ("body_upper", "body_full", "cowl_upper")],
                key=lambda v: v[0])
    gaps_ = []
    x_end = 0.0
    for x0, x1 in xs:
        if x0 > x_end + 0.002:
            gaps_.append(f"{x_end:.3f}-{x0:.3f}")
        x_end = max(x_end, x1)
    R.append(_row("C09", "upper body covered by shell panels from the nose to the cowl exit", not gaps_ and
                  x_end >= float(S["propeller"]["plane_x"]) - 0.2, len(gaps_), 0, ", ".join(gaps_)))
    return R


def check_access(ctx: Ctx) -> list:
    L, S = ctx.L, ctx.S
    R = []
    acc = [p for p in L["shell"]["panels"] if p["attach"] in ("removable", "hinged")]

    def reach(box, mirror=False):
        b = np.asarray(box, float)
        hits = []
        fw = float(L["firewall_x"])
        for p in acc:
            if p["x"][1] < b[0][0] or p["x"][0] > b[1][0]:
                continue
            if (p["surface"].startswith("cowl") and b[1][0] < fw) or (not p["surface"].startswith("cowl") and
                                                                      b[0][0] > fw):
                continue                                   # the firewall separates cowl access from the bays
            ys = [sorted(p["y"])] + ([sorted([-p["y"][1], -p["y"][0]])] if p.get("mirror") else [])
            if p["surface"] in ("body_full", "cowl_upper", "cowl_lower") or any(
                    y[0] <= b[1][1] + 0.08 and y[1] >= b[0][1] - 0.08 for y in ys):
                hits.append(p["id"])
        return hits
    bad = []
    n = 0
    for e in L["systems"]["equipment"]:
        n += 1
        if not reach(e["box"]):
            bad.append(e["id"])
    for c in L["fuel_cells"]:
        n += 1
        if not reach([[c["x"][0], -0.3, c["z"][0]], [c["x"][1], 0.3, c["z"][1]]]):
            bad.append(c["name"])
    for jn in ("P-JOINTACCESS",):
        n += 1
        if jn not in [p["id"] for p in acc]:
            bad.append(jn)
    R.append(_row("C10", f"maintenance: every equipment item and fuel cell under a removable / hinged panel ({n} "
                  "items), wing joint access panel present", not bad, len(bad), 0, ", ".join(bad)))
    mm = S["assembly"].get("maintenance_access", [])
    ok = bool(mm) and all(r.get("primary_structure_removed") is False for r in mm)
    R.append(_row("C10", "maintenance access matrix: no primary structure removed for any item", ok, len(mm), None, ""))
    return R


# =====================================================================================================================
# C11 assembly and transport, C12 mechanism definitions
# =====================================================================================================================
def check_assembly(ctx: Ctx) -> list:
    S = ctx.S
    A = S["assembly"]
    R = []
    st = A.get("steps", [])
    ok = [s_["step"] for s_ in st] == list(range(1, len(st) + 1))
    miss = [s_["step"] for s_ in st if not all(s_.get(k) for k in ("title_tr", "subassembly", "text", "tools",
                                                                   "checks"))]
    tr = sum(any(c in (s_["title_tr"] + s_["text"]) for c in "çğıöşüÇĞİÖŞÜ") for s_ in st)
    R.append(_row("C11", "assembly steps numbered 1..N with title_tr, subassembly, text, tools, checks (Turkish)",
                  ok and not miss and tr == len(st), len(st), None, f"missing fields {miss}; Turkish text {tr}/{len(st)}"))
    need = ("chassis", "frame", "fuel", "harness", "engine", "gear", "wing", "stabilator", "shell", "rigging")
    txt = " ".join((s_["title_tr"] + " " + s_["subassembly"] + " " + s_["text"]).lower() for s_ in st)
    tr_words = {"chassis": "şasi", "frame": "çerçeve", "fuel": "yakıt", "harness": "tesisat", "engine": "motor",
                "gear": "iniş takımı", "wing": "kanat", "stabilator": "stabilatör", "shell": "kabu",
                "rigging": "ayar"}
    lack = [k for k in need if tr_words[k] not in txt]
    R.append(_row("C11", "assembly covers jig/chassis, frames, fuel, harness, engine, gear, wing, tail, shell closure, "
                  "rigging", not lack, len(lack), 0, ", ".join(lack)))
    req = {r["id"]: r for r in S["requirements"]}
    b2 = 0.5 * float(S["wing"]["span"])
    yj = float(S["wing"]["planform"]["y_junction"])
    outer = (b2 - yj) / math.cos(math.radians(float(S["wing"]["planform"]["dihedral_deg"])))
    units = {u["unit"]: u for u in A["transport"]["units"]}
    op = units.get("outer wing panel (x2)", {}).get("size_m", [0])[0]
    cw = units.get("centre body with LERX/glove, fins, stubs, ventral, gear, engine", {}).get("size_m", [0, 0])[1]
    ok1 = outer <= float(req["R-29"]["value"]) + 1e-9 and op + 1e-9 >= outer - 0.02
    ok2 = 2 * yj <= float(req["R-30"]["value"]) + 1e-9 and abs(cw - 2 * yj) < 0.05
    R.append(_row("C11", "transport units vs R-29 (outer panel) / R-30 (centre section)", ok1 and ok2,
                  [round(outer, 3), round(2 * yj, 3)], [req["R-29"]["value"], req["R-30"]["value"]],
                  f"listed sizes {op} / {cw} m"))
    R.append(_row("C11", "field re-assembly and maintenance matrix present",
                  bool(A.get("field_assembly")) and bool(A.get("maintenance_access")), len(A.get("field_assembly", [])),
                  None, ""))
    return R


def check_mech_defs(ctx: Ctx) -> list:
    L = ctx.L
    R = []
    me = L["mechanisms"]
    ctl = set(me["controls"])
    J = {j["name"]: j for j in me["joints"]}
    bad = []
    for j in me["joints"]:
        for k in ("name", "kind", "origin", "axis", "lo", "hi", "rest"):
            if k not in j:
                bad.append(f"{j.get('name')} missing {k}")
        if j["kind"] not in ("revolute", "prismatic"):
            bad.append(f"{j['name']} kind")
        if not float(j["lo"]) < float(j["hi"]) or not float(j["lo"]) - 1e-9 <= float(j["rest"]) <= float(j["hi"]) + 1e-9:
            bad.append(f"{j['name']} range/rest")
        if abs(np.linalg.norm(j["axis"]) - 1.0) > 1e-3:
            bad.append(f"{j['name']} axis not unit")
        if j.get("prop") and j["prop"] not in ctl:
            bad.append(f"{j['name']} prop {j['prop']}")
        if j.get("expr"):
            names = set(re.findall(r"[A-Za-z_][A-Za-z_0-9]*", j["expr"])) - {"clamp", "min", "max", "abs"}
            if not names <= ctl:
                bad.append(f"{j['name']} expr uses {sorted(names - ctl)}")
        if j.get("parent") and j["parent"] not in J:
            bad.append(f"{j['name']} parent")
    for base in ("aileron", "flap", "rudder", "stabilator", "main_gear", "main_inner_door", "nose_door", "turret_door"):
        if not (base + "_R" in J and base + "_L" in J):
            bad.append(f"{base} L/R pair")
    for need in ("nose_gear", "nose_steer", "turret_elevator", "para_hatch", "prop_spin"):
        if need not in J:
            bad.append(f"missing joint {need}")
    for name, sq in me["sequences"].items():
        for s_ in sq["states"]:
            for k, v in s_.items():
                if k not in J:
                    bad.append(f"sequence {name}: unknown joint {k}")
                elif not float(J[k]["lo"]) - 1e-6 <= float(v) <= float(J[k]["hi"]) + 1e-6:
                    bad.append(f"sequence {name}: {k}={v} out of range")
    R.append(_row("C12", f"mechanism definitions ({len(J)} joints, {len(me['sequences'])} sequences): fields, ranges, "
                  "props, expressions, L/R pairs, sequence states", not bad, len(bad), 0, "; ".join(bad[:8])))
    cl = L["clearances"]
    ok = isinstance(cl, list) and all({"name", "a", "b", "min_mm"} <= set(r) for r in cl)
    R.append(_row("C12", "layout.clearances in checks.py LIST format {name, a, b, min_mm, joints?}", ok, len(cl), None,
                  ""))
    return R


# =====================================================================================================================
# C13 first-cut pre-sizing of the interface fittings
# =====================================================================================================================
def fitting_presizing(ctx: Ctx) -> list:
    from . import structlib as ST
    S, L = ctx.S, ctx.L
    st = S["structures"]
    M = S["materials"]
    fos = float(st["factor_of_safety"])
    f_fit = float(st["fitting_factor"])
    f_br = float(st["bearing_factor_pinned_joints"])
    f_fa = float(st["frequent_assembly_attachment_factor"])
    rows = []

    def ms(name, applied, allow, basis, name_tr, basis_tr):
        rows.append({"item": name, "applied": _r(applied, 1), "allowable": _r(allow, 1),
                     "MS": _r(allow / max(applied, 1e-9) - 1.0, 2), "basis": basis, "item_tr": name_tr,
                     "basis_tr": basis_tr})
    # ---- wing joint (Schrenk on the reference trapezoid, as sizing.wing_structure)
    W = S["wing"]
    P = W["planform"]
    T = Z.trapezoid(P)
    b2 = T["b2"]
    y = np.linspace(0.0, b2, 241)
    n_lim = float(st["derived"]["n_limit_wing_design"])
    m0 = float(S["mass"]["mtow_kg"])
    w = fos * n_lim * m0 * G0 / 2 * ST.schrenk(y, T["c"](y), b2)
    V, Mb = ST.beam_loads(y, w)
    yj = float(P["y_junction"])
    Vj, Mj = float(np.interp(yj, y, V)), float(np.interp(yj, y, Mb))
    wj = L["chassis"]["wing_joint"]
    pins = wj["main_spar"]["pins"]
    p1, p2 = np.asarray(pins[0]["position"], float), np.asarray(pins[1]["position"], float)
    d = float(np.linalg.norm(p2 - p1))
    sw = math.radians(float(np.degrees(np.arctan2(abs(p2[0] - p1[0]), abs(p2[1] - p1[1])))))
    a = (yj - max(p1[1], p2[1])) / math.cos(sw)
    Ms = Mj / math.cos(sw)
    R_in = (Ms + Vj * a) / d
    R_out = Vj + R_in
    Rp = max(R_in, R_out)
    D = float(pins[0]["diameter"])
    ti = M["ti_6al_4v_annealed_sheet"]
    al = M["al_7075_t651_plate"]
    A = math.pi * D ** 2 / 4
    ms("wing joint main pin, double shear", Rp * f_fa / (2 * A) / 1e6, ti["Fsu"] / 1e6,
       f"R = {Rp:.0f} N (M_ult {Mj:.0f} N m, V_ult {Vj:.0f} N at y {yj}; pins {d * 1000:.0f} mm apart) x "
       f"{f_fa} frequent assembly; MPa", "kanat birleşimi ana pimi, çift kesme",
       f"R = {Rp:.0f} N (y = {yj} m'de M_nihai {Mj:.0f} N m, V_nihai {Vj:.0f} N; pimler arası {d * 1000:.0f} mm) x "
       f"{f_fa} sık sökülen bağlantı katsayısı; MPa")
    t_pr = 0.006
    t_tg = float(wj["main_spar"]["fork"]["slot"]["width"])
    arm = t_pr / 2 + t_tg / 4
    Mpin = Rp * f_fa / 2 * arm
    ms("wing joint main pin, bending", 32 * Mpin / (math.pi * D ** 3) / 1e6, ti["Ftu"] / 1e6,
       f"M = R/2 (t_prong/2 + t_tongue/4) = {Mpin:.0f} N m; Ftu (no plastic bending credit); MPa",
       "kanat birleşimi ana pimi, eğilme",
       f"M = R/2 (t_çatal/2 + t_dil/4) = {Mpin:.0f} N m; Ftu ile (plastik eğilme kazancı alınmadı); MPa")
    ms("wing joint tongue bearing (7075)", Rp * f_br / (D * t_tg) / 1e6, al["Fbru"] / 1e6,
       f"x {f_br} bearing factor, tongue width {t_tg * 1000:.1f} mm; MPa", "kanat birleşimi dil ezilmesi (7075)",
       f"x {f_br} ezilme katsayısı, dil kalınlığı {t_tg * 1000:.1f} mm; MPa")
    ms("wing joint fork prong bearing (7075, 2 prongs)", Rp * f_br / (2 * D * t_pr) / 1e6, al["Fbru"] / 1e6,
       f"x {f_br} bearing factor, prongs {t_pr * 1000:.0f} mm; MPa", "kanat birleşimi çatal ezilmesi (7075, 2 kulak)",
       f"x {f_br} ezilme katsayısı, kulaklar {t_pr * 1000:.0f} mm; MPa")
    # ---- parachute bridle fittings
    P_open = PARA_OPEN_N
    P_d = P_open * f_fit
    rm_129, tau_k = 1220e6, 0.6
    As6 = 20.1e-6
    ms("bridle fitting bolts 4 x M6 12.9 (single shear, one leg takes the full shock)", P_d / (4 * As6) / 1e6,
       tau_k * rm_129 / 1e6, "13.1 kN (GRS 4/240 published opening shock, > UAVOS 200 5 g x MTOM) ultimate-only "
       f"(CRASH-004) x {f_fit} fitting factor; ISO 898-1 12.9 Rm 1220 MPa, tau = 0.6 Rm on the stress area; MPa",
       "kayış bağlantısı cıvataları 4 x M6 12.9 (tek kesme, şokun tamamını tek ayak taşır)",
       f"{P_open / 1000:.1f} kN (GRS 4/240 yayımlanmış açılma şoku, > UAVOS 200 5 g x MTOM) yalnız nihai (CRASH-004) x "
       f"{f_fit} bağlantı katsayısı; ISO 898-1 12.9 Rm 1220 MPa, gerilme alanında tau = 0,6 Rm; MPa")
    st4130 = M["steel_4130_n"]
    Dp = 0.010
    ms("bridle shackle pin d 10 (4130, double shear)", P_d / (2 * math.pi * Dp ** 2 / 4) / 1e6, st4130["Fsu"] / 1e6,
       "MPa", "kayış kilit pimi Ø10 (4130, çift kesme)", "MPa")
    ms("bridle U-lug bearing (7075, 2 cheeks 8 mm)", P_d * f_br / (2 * Dp * 0.008) / 1e6, al["Fbru"] / 1e6,
       f"x {f_br} bearing factor; MPa", "kayış U-kulak ezilmesi (7075, 2 yanak 8 mm)", f"x {f_br} ezilme katsayısı; MPa")
    m_conf = PARA_OPEN_N / (m0 * G0)
    rows.append({"item": "opening load factor at MTOM (information)", "applied": _r(m_conf, 2), "allowable": None,
                 "MS": None, "basis": "13.1 kN / (MTOM g); UAVOS 200 rated 5 g",
                 "item_tr": "MTOM'da açılma yük katsayısı (bilgi)",
                 "basis_tr": "13,1 kN / (MTOM g); UAVOS 200 anma değeri 5 g"})
    # ---- engine mount bolts (4 x M8 12.9)
    E = S["engine"]["installed_items_kg"]
    m_e = float(E["engine_bare"]) + float(E["starter_generator_sg750"]) + float(E["generator_adapter_coupling"])
    em = L["chassis"]["engine_mount"]
    holes = np.asarray(em["bolts"]["points"], float)
    mf = np.asarray(em["mount_face"]["center"], float)
    ey, ez = np.array([0.0, 1.0, 0.0]), ctx.e_up
    yz = np.column_stack([(holes - mf) @ ey, (holes - mf) @ ez])
    r = np.linalg.norm(yz, axis=1)
    T_lim = 154.7                                                 # standards.yaml engine_mount.limit_torque_MCP_Nm
    arm_cg = 0.09                                                 # engine CG ahead of the mount face (estimate)
    cases = [("limit torque + 3.8 g vertical", 3.8, 0.0, T_lim, fos),
             ("1.47 g side", 0.0, 1.47, 0.0, fos),
             ("6 g down emergency landing (ultimate)", 6.0, 0.0, 0.0, 1.0)]
    worst = 0.0
    for nm, nz, ny, Tq, f in cases:
        Fz, Fy = nz * m_e * G0, ny * m_e * G0
        shear = math.hypot(Fz, Fy) / 4 + Tq / (4 * float(r.mean()))
        Mb_ = math.hypot(Fz, Fy) * arm_cg
        ten = Mb_ * float(np.abs(yz[:, 1]).max()) / float((yz[:, 1] ** 2).sum())
        load = math.hypot(shear, ten) * f * f_fit
        worst = max(worst, load)
    As8 = 36.6e-6
    ms("engine mount bolt M8 12.9 (worst of torque + 3.8 g / 1.47 g side / 6 g down)", worst / As8 / 1e6,
       tau_k * rm_129 / 1e6, f"engine + SG750 + adapter {m_e:.2f} kg, CG 90 mm ahead of the mount face (estimate), "
       f"x {fos} (flight cases) x {f_fit}; combined shear/tension on the stress area vs 0.6 Rm; MPa; crankcase thread "
       "pull-out and isolator capacity: open (Limbach data)",
       "motor bağlantı cıvatası M8 12.9 (tork + 3,8 g / 1,47 g yan / 6 g aşağı durumlarının en kötüsü)",
       f"motor + SG750 + adaptör {m_e:.2f} kg, AM bağlantı yüzünün 90 mm önünde (tahmin), x {fos} (uçuş durumları) "
       f"x {f_fit}; gerilme alanında birleşik kesme/çekme, 0,6 Rm ile karşılaştırma; MPa; karter dişi sıyrılması ve "
       "sönümleyici kapasitesi açık konu (Limbach verisi)")
    F15 = 15.0 * m_e * G0 * f_fit
    rows.append({"item": "15 g forward crash retention (engine pushes into the mount, truss in compression)",
                 "applied": _r(F15, 0), "allowable": None, "MS": None,
                 "basis": "N; carried by the welded truss in compression (detail sizing of the 4130 tubes)",
                 "item_tr": "15 g ileri çarpma tutması (motor kafese basar, kafes basıda)",
                 "basis_tr": "N; kaynaklı kafes basıyla taşır (4130 boruların ayrıntılı boyutlandırması)"})
    return rows


def check_presizing(ctx: Ctx) -> tuple[list, list]:
    rows = fitting_presizing(ctx)
    bad = [r["item"] for r in rows if r["MS"] is not None and r["MS"] < 0.0]
    mn = min(r["MS"] for r in rows if r["MS"] is not None)
    return [_row("C13", "first-cut fitting pre-sizing: margins of safety >= 0 (wing joint pins, bridle fittings, "
                 "engine mount bolts)", not bad, mn, ">= 0", "; ".join(bad))], rows


# =====================================================================================================================
# run
# =====================================================================================================================
def run_checks(ctx: Ctx | None = None, verbose: bool = False) -> dict:
    ctx = ctx or Ctx()
    t0 = time.time()
    objs = layout_objects(ctx)
    rows = []
    timing = {}

    def step(name, fn):
        t = time.time()
        out = fn()
        timing[name] = round(time.time() - t, 2)
        if verbose:
            print(f"  {name}: {timing[name]} s", file=sys.stderr)
        return out
    rows += step("ids", lambda: check_ids(ctx))
    rows += step("stations", lambda: check_stations(ctx))
    rows += step("inside", lambda: check_inside(ctx))
    r_ov, pairs = step("overlaps", lambda: check_overlaps(ctx, objs))
    rows += r_ov
    rows += step("mechanisms", lambda: check_mechanisms(ctx, objs))
    r_cg, cg = step("cg", lambda: check_cg(ctx))
    rows += r_cg
    rows += step("fov_rf", lambda: check_fov_rf(ctx, objs))
    rows += step("keepouts", lambda: check_keepouts(ctx, objs))
    rows += step("shell", lambda: check_shell(ctx))
    rows += step("access", lambda: check_access(ctx))
    rows += step("assembly", lambda: check_assembly(ctx))
    rows += step("mech_defs", lambda: check_mech_defs(ctx))
    r_ps, ps = step("presizing", lambda: check_presizing(ctx))
    rows += r_ps
    return {"rows": rows, "ok": all(r["ok"] for r in rows), "n_pass": sum(r["ok"] for r in rows), "n": len(rows),
            "cg": cg, "presizing": ps, "n_objects": len(objs), "close_pairs": len(pairs), "timing_s": timing,
            "elapsed_s": round(time.time() - t0, 1)}


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description="YK-250 layout checks (spec.layout / spec.assembly)")
    ap.add_argument("--check", action="store_true", help="exit 1 if any check fails")
    ap.add_argument("--no-figures", action="store_true")
    ap.add_argument("--no-write", action="store_true", help="do not write out/layout.* and figures")
    ap.add_argument("--out", default=None)
    ap.add_argument("--fig-dir", default=None)
    ap.add_argument("-v", "--verbose", action="store_true")
    a = ap.parse_args(argv)
    ctx = Ctx()
    res = run_checks(ctx, verbose=a.verbose)
    for r in res["rows"]:
        print(f"  {r['check']}  {'PASS' if r['ok'] else 'FAIL'}  {r['item']}  [{r['value']}"
              f"{' / ' + str(r['limit']) if r['limit'] is not None else ''}]" + (f"  {r['detail']}" if r['detail'] and
                                                                                  not r['ok'] else ""))
    if not a.no_write:
        out_dir = Path(a.out) if a.out else OUT_DIR
        fig_dir = Path(a.fig_dir) if a.fig_dir else FIG_DIR
        figs = {} if a.no_figures else write_figures(ctx, res, fig_dir)
        write_report(ctx, res, figs, out_dir)
    print(f"[layout_check] {res['n_pass']}/{res['n']} checks pass ({res['elapsed_s']} s)")
    return 0 if (res["ok"] or not a.check) else 1


# =====================================================================================================================
# figures
# =====================================================================================================================
GROUP_COL = {"systems": "#2f7fbf", "payload": "#8e44ad", "propulsion": "#c0392b", "fuel": "#2e86c1", "controls": "#16a085",
             "gear": "#7f8c8d", "chassis": "#555555"}


def _oml_side(ctx: Ctx):
    xs = np.linspace(ctx.af.fus.x0 + 1e-4, ctx.af.fus.x1 - 1e-4, 300)
    return xs, np.array([ctx.z_top(x) for x in xs]), np.array([ctx.z_bot(x) for x in xs])


def _oml_top(ctx: Ctx):
    xs = np.linspace(ctx.af.fus.x0 + 1e-4, ctx.af.fus.x1 - 1e-4, 300)
    return xs, np.array([float(ctx.af.sec(x)[0][0]) for x in xs])


def _planform_clip(ax, ctx: Ctx):
    """Invisible patch of the body planform + both glove panels (y <= y_junction), used as clip path in top views."""
    from matplotlib.path import Path as MPath
    from matplotlib.patches import PathPatch
    xs, hw = _oml_top(ctx)
    body = np.vstack([np.column_stack([xs, hw]), np.column_stack([xs[::-1], -hw[::-1]])])
    yj = float(ctx.S["wing"]["planform"]["y_junction"])
    W = [s_ for s_ in ctx.S["wing"]["sections"] if s_["y"] <= yj + 1e-6]
    paths = [MPath(np.vstack([body, body[:1]]), closed=True)]
    for sg in (1, -1):
        le = [(s_["x_le"], sg * s_["y"]) for s_ in W]
        te = [(s_["x_le"] + s_["chord"], sg * s_["y"]) for s_ in W][::-1]
        P = np.array(le + te + le[:1], float)
        P[0, 1] = P[-1, 1] = sg * (float(W[0]["y"]) - 0.01)
        paths.append(MPath(P, closed=True))
    pp = PathPatch(MPath.make_compound_path(*paths), fc="none", ec="none", transform=ax.transData)
    ax.add_patch(pp)
    return pp


def _rect(ax, b, ix, iy, **kw):
    from matplotlib.patches import Rectangle
    b = np.asarray(b, float)
    lo, hi = np.minimum(b[0], b[1]), np.maximum(b[0], b[1])
    ax.add_patch(Rectangle((lo[ix], lo[iy]), hi[ix] - lo[ix], hi[iy] - lo[iy], **kw))


def _poly_obb(ax, o: OBB, ix, iy, **kw):
    from matplotlib.patches import Polygon
    from scipy.spatial import ConvexHull
    C = o.corners()[:, [ix, iy]]
    try:
        h = ConvexHull(C)
        ax.add_patch(Polygon(C[h.vertices], closed=True, **kw))
    except Exception:
        pass


def _draw_layout(ax, ctx: Ctx, view: str, objs: list, detail: bool = True):
    """view 'side' (x-z) or 'top' (x-y, starboard + port body)."""
    from matplotlib.patches import Circle
    L, S = ctx.L, ctx.S
    ix, iy = (0, 2) if view == "side" else (0, 1)
    clip = _planform_clip(ax, ctx) if view == "top" else None
    n0 = len(ax.patches)
    if view == "side":
        xs, zt, zb = _oml_side(ctx)
        ax.fill_between(xs, zb, zt, color="#f2f2f2", zorder=0)
        ax.plot(xs, zt, "k-", lw=1.0)
        ax.plot(xs, zb, "k-", lw=1.0)
        ax.plot(xs, [float(ctx.af.sec(x)[3][0]) for x in xs], color="#bbbbbb", lw=0.6, ls="--")
        for name in ("fin", "ventral"):
            T = S["tail"]["surfaces"][name]["sections"]
            le_ = np.array([[s_["x_le"], s_["z_le"]] for s_ in T])
            te_ = np.array([[s_["x_le"] + s_["chord"], s_["z_le"]] for s_ in T])
            P = np.vstack([le_, te_[::-1], le_[:1]])
            ax.plot(P[:, 0], P[:, 1], color="#666666", lw=0.7)
    else:
        xs, hw = _oml_top(ctx)
        ax.fill_between(xs, -hw, hw, color="#f2f2f2", zorder=0)
        ax.plot(xs, hw, "k-", lw=1.0)
        ax.plot(xs, -hw, "k-", lw=1.0)
        W = S["wing"]["sections"]
        le = np.array([[s_["x_le"], s_["y"]] for s_ in W])
        te = np.array([[s_["x_le"] + s_["chord"], s_["y"]] for s_ in W])
        for sg in (1, -1):
            ax.plot(le[:, 0], sg * le[:, 1], "k-", lw=0.8)
            ax.plot(te[:, 0], sg * te[:, 1], "k-", lw=0.8)
            ax.plot([le[-1, 0], te[-1, 0]], [sg * le[-1, 1], sg * te[-1, 1]], "k-", lw=0.8)
        for name in ("fin", "stabilator", "stabilator_stub"):
            T = S["tail"]["surfaces"][name]["sections"]
            le_ = np.array([[s_["x_le"], s_["y"]] for s_ in T])
            te_ = np.array([[s_["x_le"] + s_["chord"], s_["y"]] for s_ in T])
            for sg in (1, -1):
                ax.plot(le_[:, 0], sg * le_[:, 1], color="#666666", lw=0.6)
                ax.plot(te_[:, 0], sg * te_[:, 1], color="#666666", lw=0.6)
                ax.plot([le_[0, 0], te_[0, 0]], [sg * le_[0, 1], sg * te_[0, 1]], color="#666666", lw=0.6)
                ax.plot([le_[-1, 0], te_[-1, 0]], [sg * le_[-1, 1], sg * te_[-1, 1]], color="#666666", lw=0.6)
    # stations
    for s_ in L["stations"]:
        x = float(s_["x"])
        if view == "side":
            ax.plot([x, x], [ctx.z_bot(x), ctx.z_top(x)], color="#1a5276", lw=1.4 if s_["type"] != "ring" else 0.9,
                    ls="-" if s_["type"] != "ring" else "--", zorder=3)
            ax.text(x, ctx.z_top(x) + 0.012, s_["id"], rotation=90, fontsize=6, ha="center", va="bottom",
                    color="#1a5276")
        else:
            yy = np.linspace(-0.4, 0.4, 41)
            hw_ = float(ctx.af.sec(x)[0][0])
            yy = yy[np.abs(yy) <= hw_]
            ax.plot([station_x(s_, y) for y in yy], yy, color="#1a5276", lw=1.2, zorder=3)
    # members
    for o in objs:
        if o.kind == "structure" and o.id.startswith("M-"):
            for p in o.prims:
                if isinstance(p, OBB):
                    _poly_obb(ax, p, ix, iy, fc="#a6acaf", ec="#5d6d7e", lw=0.4, alpha=0.55, zorder=2)
                elif isinstance(p, Capsule):
                    ax.plot([p.p0[ix], p.p1[ix]], [p.p0[iy], p.p1[iy]], color="#5d6d7e", lw=1.6, zorder=2)
        if o.kind == "structure" and o.id.startswith("F-"):
            for p in o.prims:
                if isinstance(p, OBB):
                    _poly_obb(ax, p, ix, iy, fc="#f5b041", ec="#9c640c", lw=0.5, zorder=5)
                elif isinstance(p, Cyl):
                    lo, hi = p.bounds()
                    _rect(ax, [lo, hi], ix, iy, fc="#f5b041", ec="#9c640c", lw=0.5, zorder=5)
    # fuel
    for o in objs:
        if o.kind == "fuel":
            P = o.prims[0].samples()
            if len(P):
                ax.scatter(P[:, ix], P[:, iy], s=1.0, c="#5dade2", alpha=0.25, lw=0, zorder=1)
    # zones
    Zp = L["zones_preliminary"]
    for k, col in (("payload_bay", "#d7bde2"), ("parachute_bay", "#f9e79f"), ("mission_computer", "#d6eaf8")):
        b = np.asarray(Zp[k]["box"], float)
        bb = b.copy()
        if view == "top" and Zp[k].get("symmetric"):
            bb[0][1] = -b[1][1]
        _rect(ax, bb, ix, iy, fc=col, ec="none", alpha=0.6, zorder=1)
    for o in objs:
        if o.kind in ("content",) and o.id.startswith(("EQ-", "ACT-")):
            col = GROUP_COL.get(o.group, "#2f7fbf")
            for p in o.prims:
                _poly_obb(ax, p, ix, iy, fc=col, ec="k", lw=0.3, alpha=0.75, zorder=4) if isinstance(p, OBB) else None
        if o.kind == "harness":
            for p in o.prims:
                ax.plot([p.p0[ix], p.p1[ix]], [p.p0[iy], p.p1[iy]], color="#e67e22", lw=1.3, zorder=6)
        if o.kind in ("engine", "exhaust"):
            for p in o.prims:
                _poly_obb(ax, p, ix, iy, fc="#f1948a" if o.kind == "engine" else "#e74c3c", ec="#922b21", lw=0.4,
                          alpha=0.45, zorder=3)
        if o.kind == "mount":
            for p in o.prims:
                ax.plot([p.p0[ix], p.p1[ix]], [p.p0[iy], p.p1[iy]], color="#1e8449", lw=1.2, zorder=6)
        if o.id == "DUCT-COOLING" and view == "side":
            for p in o.prims[:len(o.prims) // 3]:
                ax.plot([p.p0[ix], p.p1[ix]], [p.p0[iy], p.p1[iy]], color="#85c1e9", lw=6, alpha=0.5, zorder=2)
        if o.kind == "gear":
            for p in o.prims:
                lo, hi = p.bounds()
                _rect(ax, [lo, hi], ix, iy, fc="none", ec="#424949", lw=0.8, ls="-", zorder=5)
    # turret
    T = S["payload"]["turret"]
    r = 0.5 * float(T["growth_envelope"]["diameter"])
    xc = float(T["bay_center_x"])
    if view == "side":
        ax.add_patch(Circle((xc, float(T["ball_center_retracted_z"])), r, fc="#bb8fce", ec="#6c3483", lw=0.6,
                            alpha=0.6, zorder=5))
        ax.add_patch(Circle((xc, float(T["ball_center_extended_z"])), r, fc="none", ec="#6c3483", lw=0.6, ls="--",
                            zorder=5))
    else:
        ax.add_patch(Circle((xc, 0.0), r, fc="#bb8fce", ec="#6c3483", lw=0.6, alpha=0.6, zorder=5))
    # gear down (side): tyres at rest
    if view == "side":
        for jn, sd in (("main", "R"), ("nose", "")):
            g = gear_prims(ctx, jn, 0.0, sd or "R")
            c = g["tyre"].c
            ax.add_patch(Circle((c[0], c[2]), g["tyre"].R, fc="none", ec="#424949", lw=0.8, ls="--", zorder=5))
            ax.plot([g["leg"].p0[0], g["leg"].p1[0]], [g["leg"].p0[2], g["leg"].p1[2]], color="#424949", lw=1.2,
                    ls="--")
        ko = next(k for k in L["keep_outs"] if k["id"] == "KO-PROP")
        c, a, R = np.asarray(ko["centre"]), np.asarray(ko["axis"]), float(ko["radius"])
        e = np.array([-a[2], 0.0, a[0]])
        ax.plot([c[0] - R * e[0], c[0] + R * e[0]], [c[2] - R * e[2], c[2] + R * e[2]], color="#566573", lw=2.0,
                alpha=0.5)
        br = L["chassis"]["parachute"]["bridle"]
        cg = br["cg_used"]
        ax.plot([cg[0]], [cg[2]], marker="o", ms=7, mfc="w", mec="k", zorder=8)
        ax.text(cg[0] + 0.03, cg[2] + 0.03, "AM (MTOM)", fontsize=7)
    else:
        for pn in L["chassis"]["wing_joint"]["main_spar"]["pins"]:
            p = pn["position"]
            for sg in (1, -1):
                ax.plot([p[0]], [sg * p[1]], marker="o", ms=3.5, color="#922b21", zorder=8)
        for k in ("aileron_R", "flap_R"):
            j = next(j_ for j_ in L["mechanisms"]["joints"] if j_["name"] == k)
            o, a = np.asarray(j["origin"]), np.asarray(j["axis"])
            c = S["wing"]["controls"][k.split("_")[0]]
            b2 = 0.5 * float(S["wing"]["span"])
            y0, y1 = float(c["eta0"]) * b2, float(c["eta1"]) * b2
            t0, t1 = (y0 - o[1]) / a[1], (y1 - o[1]) / a[1]
            for sg in (1, -1):
                ax.plot([o[0] + t0 * a[0], o[0] + t1 * a[0]], [sg * (o[1] + t0 * a[1]), sg * (o[1] + t1 * a[1])],
                        color="#16a085", lw=1.0, ls="--")
    if clip is not None:
        for pt in ax.patches[n0:]:
            if pt is not clip:
                pt.set_clip_path(clip)


def write_figures(ctx: Ctx, res: dict, fig_dir) -> dict:
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.lines import Line2D
    from matplotlib.patches import Patch
    fig_dir = Path(fig_dir)
    fig_dir.mkdir(parents=True, exist_ok=True)
    objs = layout_objects(ctx)
    out = {}
    leg = [Line2D([0], [0], color="#1a5276", lw=1.4, label="çerçeve / perde"),
           Patch(fc="#a6acaf", ec="#5d6d7e", label="şasi elemanı"), Patch(fc="#f5b041", label="bağlantı"),
           Patch(fc="#5dade2", alpha=0.5, label="yakıt hücresi"), Patch(fc="#2f7fbf", label="teçhizat"),
           Patch(fc="#bb8fce", label="taret (E180 zarfı)"), Line2D([0], [0], color="#e67e22", lw=1.3, label="kablo demeti"),
           Patch(fc="#f1948a", label="motor / egzoz zarfı"), Line2D([0], [0], color="#1e8449", lw=1.2, label="motor bağlantı kafesi"),
           Patch(fc="#d7bde2", label="faydalı yük bölmesi"), Patch(fc="#f9e79f", label="paraşüt bölmesi")]
    # ---- side
    z_hi = max(float(s_["z_le"]) for s_ in ctx.S["tail"]["surfaces"]["fin"]["sections"]) + 0.04
    LG = ctx.S["landing_gear"]
    zg, xm = float(LG["ground_z"]), float(LG["main"]["axle_static"][0])
    att = math.radians(float(LG["checks"]["static_attitude_deg"]))
    gx = np.array([-0.15, 4.75])
    gz = zg + (gx - xm) * math.tan(att)
    z_lo = float(gz.min()) - 0.03
    fig, ax = plt.subplots(figsize=(17, 1.6 + 16.0 * (z_hi - z_lo) / 4.9))
    _draw_layout(ax, ctx, "side", objs)
    ax.plot(gx, gz, color="#8d6e63", lw=1.0, ls="-.")
    ax.text(4.70, gz[1] + 0.008, f"yer (takım açık, statik duruş {math.degrees(att):.1f}° burun yukarı)", fontsize=7,
            color="#6d4c41", ha="right", va="bottom")
    ax.set_aspect("equal")
    ax.set_xlim(-0.15, 4.75)
    ax.set_ylim(z_lo, z_hi)
    ax.set_xlabel("x (m, burundan geriye)")
    ax.set_ylabel("z (m)")
    ax.set_title("YK-250 HANÇER — yan kesit yerleşimi (y = 0 düzlemine izdüşüm): çerçeveler, şasi, bölmeler, teçhizat, "
                 "takım (açık: kesik çizgi, toplanmış: düz), taret", fontsize=10)
    ax.legend(handles=leg, loc="upper left", fontsize=7.5, ncol=3, frameon=True)
    ax.grid(alpha=0.25)
    fig.tight_layout()
    p = fig_dir / "yk250_layout_side.png"
    fig.savefig(p, dpi=125)
    plt.close(fig)
    out["side"] = p
    # ---- top
    fig, (ax, axw) = plt.subplots(1, 2, figsize=(19, 6.4), gridspec_kw={"width_ratios": [2.55, 1.0]})
    for a_ in (ax, axw):
        _draw_layout(a_, ctx, "top", objs)
        a_.set_aspect("equal")
        a_.grid(alpha=0.25)
        a_.set_xlabel("x (m)")
    ax.set_xlim(-0.08, 4.45)
    ax.set_ylim(-0.80, 0.80)
    ax.set_ylabel("y (m, sancak +)")
    ax.set_title("gövde ve orta kanat (yakın): ok açılı kiriş çerçeveleri, ana pimler (kırmızı), teçhizat, yakıt "
                 "hücreleri, kablo demetleri", fontsize=9.5)
    ax.legend(handles=leg, loc="lower left", fontsize=6.8, ncol=4, frameon=True)
    axw.set_xlim(-0.1, 4.6)
    axw.set_ylim(-0.6, 3.75)
    axw.set_title("sancak yarı planform: dış panel, kanat bağlantısı,\nkumanda menteşe hatları (yeşil kesik)", fontsize=9.5)
    fig.suptitle("YK-250 HANÇER — üst görünüş yerleşimi", fontsize=11)
    fig.tight_layout()
    p = fig_dir / "yk250_layout_top.png"
    fig.savefig(p, dpi=115)
    plt.close(fig)
    out["top"] = p
    out["structure"] = _fig_structure(ctx, objs, fig_dir)
    out["shell"] = _fig_shell(ctx, fig_dir)
    out["sections"] = _fig_sections(ctx, objs, fig_dir)
    return {k: str(v) for k, v in out.items()}


def _callouts(ax, items, corner, start=1):
    """Numbered load-path arrows: a circled number at each tail, the explanations listed in a box at ``corner``
    (axes fraction, upper-left anchor)."""
    lines = []
    for k, (tail, heads, col, txt) in enumerate(items, start):
        for h in heads:
            ax.annotate("", xy=h, xytext=tail, arrowprops=dict(arrowstyle="-|>", color=col, lw=1.6,
                                                              mutation_scale=12, shrinkA=7), zorder=9)
        ax.text(tail[0], tail[1], str(k), fontsize=8, color="w", ha="center", va="center", zorder=11,
                fontweight="bold", bbox=dict(boxstyle="circle,pad=0.25", fc=col, ec="none"))
        lines.append(f"{k}  {txt}")
    ax.text(corner[0], corner[1], "\n".join(lines), transform=ax.transAxes, fontsize=7.5, va="top", ha="left",
            zorder=12, bbox=dict(fc="w", ec="#999999", alpha=0.92, pad=4))


def _fig_structure(ctx: Ctx, objs: list, fig_dir: Path):
    import matplotlib.pyplot as plt
    L, S = ctx.L, ctx.S
    fig, (a1, a2) = plt.subplots(2, 1, figsize=(16, 12), gridspec_kw={"height_ratios": [1, 1.5]})
    for ax, view in ((a1, "side"), (a2, "top")):
        ix, iy = (0, 2) if view == "side" else (0, 1)
        clip = _planform_clip(ax, ctx) if view == "top" else None
        n0 = len(ax.patches)
        if view == "side":
            xs, zt, zb = _oml_side(ctx)
            ax.plot(xs, zt, color="#aaaaaa", lw=0.8)
            ax.plot(xs, zb, color="#aaaaaa", lw=0.8)
        else:
            xs, hw = _oml_top(ctx)
            ax.plot(xs, hw, color="#aaaaaa", lw=0.8)
            ax.plot(xs, -hw, color="#aaaaaa", lw=0.8)
            W = S["wing"]["sections"]
            ax.plot([s_["x_le"] for s_ in W], [s_["y"] for s_ in W], color="#aaaaaa", lw=0.8)
            ax.plot([s_["x_le"] + s_["chord"] for s_ in W], [s_["y"] for s_ in W], color="#aaaaaa", lw=0.8)
        for s_ in L["stations"]:
            x = float(s_["x"])
            if view == "side":
                ax.plot([x, x], [ctx.z_bot(x), ctx.z_top(x)], color="#1a5276", lw=2.2 if s_["type"] != "ring" else 1.2)
                ax.text(x, ctx.z_top(x) + 0.01, s_["id"], rotation=90, fontsize=6, ha="center", va="bottom")
            else:
                yy = np.linspace(-0.4, 0.4, 41)
                yy = yy[np.abs(yy) <= float(ctx.af.sec(x)[0][0])]
                ax.plot([station_x(s_, y) for y in yy], yy, color="#1a5276", lw=2.0)
        for o in objs:
            if o.kind == "structure":
                for p in o.prims:
                    if isinstance(p, Capsule):
                        ax.plot([p.p0[ix], p.p1[ix]], [p.p0[iy], p.p1[iy]], color="#2c3e50", lw=2.5)
                    elif isinstance(p, OBB):
                        _poly_obb(ax, p, ix, iy, fc="#d5d8dc" if o.id.startswith("M-") else "#f5b041",
                                  ec="#2c3e50", lw=0.5, alpha=0.8)
                    else:
                        lo, hi = p.bounds()
                        _rect(ax, [lo, hi], ix, iy, fc="#f5b041", ec="#2c3e50", lw=0.5)
            if o.kind == "mount":
                for p in o.prims:
                    ax.plot([p.p0[ix], p.p1[ix]], [p.p0[iy], p.p1[iy]], color="#1e8449", lw=1.5)
        if clip is not None:
            for pt in ax.patches[n0:]:
                if pt is not clip:
                    pt.set_clip_path(clip)
        ax.set_aspect("equal")
        ax.grid(alpha=0.2)
    fit = {f["id"]: f for f in L["chassis"]["fittings"]}
    tr = np.asarray(fit["F-TRUNNION"]["pivot"])
    msx = float(next(s_ for s_ in L["stations"] if s_["id"] == "FS-MS")["x"])
    gx = float(next(s_ for s_ in L["stations"] if s_["id"] == "FS-GEAR")["x"])
    fr, fa = np.asarray(fit["F-RISER-FWD"]["point"]), np.asarray(fit["F-RISER-AFT"]["point"])
    apex = (0.5 * (fr[0] + fa[0]), 0.47)
    para = PARA_OPEN_N / 1000.0
    side = [  # (tail, [heads], colour, text)
        ((0.65, -0.34), [(0.65, -0.125)], "#7d3c98", "burun takımı → omurga duvarları → FS0600 / FS1110"),
        ((1.22, -0.34), [(1.22, -0.12)], "#8e44ad", "taret ataleti → raylar → FS1110 / FS1330"),
        (apex, [(fr[0], fr[2]), (fa[0], fa[2])], "#b9770e",
         f"paraşüt açılma yükü {para:.1f} kN (Y-kayış) → FS1810 / FS-RS".replace(".", ",")),
        ((tr[0], -0.34), [(tr[0], tr[2])], "#7d3c98", "ana takım yükü → mafsal → takım kirişi → FS-GEAR / FS-RS"),
        ((3.22, 0.36), [(3.30, 0.19)], "#1e8449", "yangın perdesi → sırt uzun kirişleri / arka omurga → FS-GEAR"),
        ((3.62, 0.50), [(3.52, 0.31)], "#2874a6", "dikey / stabilatör yükü → kök bağlantıları → FS3480 / FS3670"),
        ((4.25, 0.10), [(3.76, 0.17)], "#1e8449", "itki / motor ataleti → 4130 kafes → 4 perde bağlantı parçası"),
    ]
    _callouts(a1, side, (0.02, 0.97))
    a1.plot([tr[0], gx], [tr[2], -0.07], color="#7d3c98", lw=1.2, ls=":")
    a1.set_xlim(-0.1, 4.6)
    a1.set_ylim(-0.42, 0.58)
    a1.set_title("Yapısal kavram — yan görünüş: birincil yük yolları (çerçeveler kalın mavi, uzun kirişler / omurga koyu, "
                 "bağlantılar turuncu)", fontsize=10)
    pins = L["chassis"]["wing_joint"]["main_spar"]["pins"]
    p2 = np.asarray(pins[1]["position"])
    top = [
        ((p2[0] + 0.30, 1.30), [(p2[0], p2[1])], "#c0392b", "dış panel momenti / kesmesi → dil + 2 pim Ø14 (her yan)"),
        ((msx - 0.25, 0.75), [(msx + 0.01, 0.03)], "#c0392b",
         "çatal → kiriş başlıkları → orta kutu (simetrik eğilme kutu içinde dengelenir)"),
        ((msx - 0.70, 0.62), [(msx - 0.55, 0.36)], "#c0392b", "kiriş çerçeveleri → kenar uzun kirişleri (asimetrik yük)"),
        ((tr[0] + 0.35, 0.95), [(tr[0], tr[1])], "#7d3c98", "ana takım mafsalı → takım kirişleri"),
        ((4.15, -0.62), [(3.70, -0.12)], "#1e8449", "motor kafesi → 4 perde bağlantı parçası (yangın perdesi)"),
    ]
    _callouts(a2, top, (0.02, 0.97), start=len(side) + 1)
    a2.set_xlim(-0.1, 4.6)
    a2.set_ylim(-0.8, 1.8)
    a2.set_title("Yapısal kavram — üst görünüş: ok açılı orta kutu (kök parça YK250-CH-001), dış panel birleşimi "
                 f"y = {float(S['wing']['planform']['y_junction']):.2f} m, takım / motor / kuyruk bağlantıları"
                 .replace("0.", "0,"), fontsize=10)
    fig.tight_layout()
    p = fig_dir / "yk250_layout_structure.png"
    fig.savefig(p, dpi=115)
    plt.close(fig)
    return p


def _fig_shell(ctx: Ctx, fig_dir: Path):
    import matplotlib.pyplot as plt
    from matplotlib.patches import Rectangle
    L = ctx.L
    col = {"fixed": "#d5d8dc", "removable": "#82e0aa", "hinged": "#f8c471", "fairing": "#aed6f1"}
    fig, axs = plt.subplots(2, 1, figsize=(16, 10.5))
    for ax, surf, title in ((axs[0], ("body_upper", "body_full", "cowl_upper", "glove_upper"), "üst yüzey"),
                            (axs[1], ("body_lower", "body_full", "cowl_lower", "glove_lower", "body_side"), "alt yüzey")):
        xs, hw = _oml_top(ctx)
        ax.plot(xs, hw, "k-", lw=0.8)
        ax.plot(xs, -hw, "k-", lw=0.8)
        W = ctx.S["wing"]["sections"]
        ax.plot([s_["x_le"] for s_ in W if s_["y"] < 0.75], [s_["y"] for s_ in W if s_["y"] < 0.75], "k-", lw=0.6)
        ax.plot([s_["x_le"] + s_["chord"] for s_ in W if s_["y"] < 0.75], [s_["y"] for s_ in W if s_["y"] < 0.75],
                "k-", lw=0.6)
        clip = _planform_clip(ax, ctx)
        for p in sorted(L["shell"]["panels"], key=lambda q: q["attach"] == "fixed", reverse=True):
            if p["surface"] not in surf:
                continue
            yy = [sorted(p["y"])] + ([sorted([-p["y"][1], -p["y"][0]])] if p.get("mirror") else [])
            for y0, y1 in yy:
                fc = col.get(p["attach"], "#d5d8dc")
                if p.get("layup") == "wing_skin_primary" and p["attach"] == "fixed":
                    fc = "#d2b4de"
                pt = ax.add_patch(Rectangle((p["x"][0], y0), p["x"][1] - p["x"][0], y1 - y0, fc=fc, ec="#34495e",
                                            lw=0.7, alpha=0.85 if p["attach"] != "fixed" else 0.5,
                                            hatch="///" if p.get("rf_window") else None,
                                            zorder=3 if p["attach"] != "fixed" else 2))
                pt.set_clip_path(clip)
                lbl = p["id"].replace("P-", "")
                if p["attach"] != "fixed":
                    ax.text(0.5 * (p["x"][0] + p["x"][1]), 0.5 * (y0 + y1), lbl, fontsize=5.5, ha="center",
                            va="center", rotation=90 if (p["x"][1] - p["x"][0]) < 0.12 else 0, zorder=6,
                            bbox=dict(fc="w", ec="none", alpha=0.6, pad=0.3))
                elif (y1 - y0) > 0.3 and y1 > 0:
                    # label in the widest x interval of the panel not covered by a removable / hinged panel
                    xm = 0.5 * (p["x"][0] + p["x"][1])
                    ym = min(y1, float(ctx.af.sec(xm)[0][0])) - 0.035
                    cov = sorted((max(q["x"][0], p["x"][0]), min(q["x"][1], p["x"][1]))
                                 for q in L["shell"]["panels"] if q["surface"] in surf and q["attach"] != "fixed"
                                 and q["x"][1] > p["x"][0] and q["x"][0] < p["x"][1] and max(q["y"]) > ym - 0.06)
                    free, x_ = [], float(p["x"][0])
                    for c0, c1 in cov:
                        if c0 > x_:
                            free.append((x_, c0))
                        x_ = max(x_, c1)
                    free.append((x_, float(p["x"][1])))
                    f0, f1 = max(free, key=lambda f: f[1] - f[0])
                    xm = 0.5 * (f0 + f1)
                    ym = min(y1, float(ctx.af.sec(xm)[0][0])) - 0.035
                    ax.text(xm, ym, lbl, fontsize=5.5, ha="center", va="top", color="#566573", zorder=6)
                elif y1 > 0 and p.get("layup") != "wing_skin_primary":
                    ax.text(0.5 * (p["x"][0] + p["x"][1]), 0.5 * (y0 + y1), lbl, fontsize=5, ha="center",
                            va="center", color="#566573", zorder=6)
        ax.set_aspect("equal")
        ax.set_xlim(-0.05, 4.3)
        ax.set_ylim(-0.75, 0.75)
        ax.set_title(f"Kabuk paneli bölümlemesi — {title} (gri: sabit somun plakalı, yeşil: sökülebilir Camloc/vida, "
                     "turuncu: menteşeli, mavi: fileto, mor: yapıştırılmış birincil eldiven, tarama: RF penceresi)",
                     fontsize=9)
        ax.grid(alpha=0.2)
    fig.tight_layout()
    p = fig_dir / "yk250_layout_shell.png"
    fig.savefig(p, dpi=120)
    plt.close(fig)
    return p


def _fig_sections(ctx: Ctx, objs: list, fig_dir: Path):
    import matplotlib.pyplot as plt
    from matplotlib.patches import Rectangle
    L = ctx.L
    ids = ["FS0600", "FS1110", "FS1490", "FS-FUEL", "FS-MS", "FS-GEAR", "FS3480", "FS3670"]
    st = {s_["id"]: s_ for s_ in L["stations"]}
    fig, axs = plt.subplots(2, 4, figsize=(17, 8.5))
    for ax, sid in zip(axs.ravel(), ids):
        s_ = st[sid]
        x = float(s_["x"])
        ph = np.linspace(0, 2 * np.pi, 361)
        yy = np.linspace(-0.45, 0.45, 361)
        zt = np.array([ctx.z_top(x, y) for y in yy])
        zb = np.array([ctx.z_bot(x, y) for y in yy])
        hw = float(ctx.af.sec(x)[0][0])
        m = np.abs(yy) <= hw
        ax.fill_between(yy[m], zb[m], zt[m], color="#d6eaf8" if s_["type"] != "ring" else "#fdebd0", zorder=1)
        ax.plot(yy[m], zt[m], "k-", lw=1)
        ax.plot(yy[m], zb[m], "k-", lw=1)
        if s_["type"] == "ring":
            G = np.array(np.meshgrid(yy, np.linspace(-0.25, 0.4, 200))).reshape(2, -1).T
            P = np.column_stack([np.full(len(G), x), G])
            inner = ctx.af.inside(P, 0.006 + float(s_.get("ring_depth", 0.04)))
            ax.scatter(G[inner, 0], G[inner, 1], s=0.5, c="w", zorder=2)
        for c in s_.get("cutouts", []):
            for sg in ((1.0, -1.0) if c.get("mirror") else (1.0,)):
                y0, y1 = sorted([sg * c["y"][0], sg * c["y"][1]])
                ax.add_patch(Rectangle((y0, c["z"][0]), y1 - y0, c["z"][1] - c["z"][0], fc="w", ec="#c0392b", lw=0.8,
                                       zorder=3))
        for o in objs:
            if o.kind not in ("content", "harness", "fuel", "gear", "structure", "mount", "engine"):
                continue
            for p in o.prims:
                if isinstance(p, Capsule):
                    d = p.p1 - p.p0
                    if abs(d[0]) > 1e-9:
                        t = (x - p.p0[0]) / d[0]
                        if 0 <= t <= 1:
                            q = p.p0 + t * d
                            ax.add_patch(plt.Circle((q[1], q[2]), p.r, fc="#e67e22" if o.kind == "harness" else
                                                    "#5d6d7e", ec="k", lw=0.3, zorder=5))
                elif isinstance(p, OBB) and o.kind in ("content", "structure", "engine"):
                    lo, hi = p.bounds()
                    if lo[0] <= x <= hi[0]:
                        ax.add_patch(Rectangle((lo[1], lo[2]), hi[1] - lo[1], hi[2] - lo[2],
                                               fc=GROUP_COL.get(o.group, "#999999"), alpha=0.35, ec="k", lw=0.3,
                                               zorder=4))
        ax.set_aspect("equal")
        ax.set_xlim(-0.45, 0.45)
        ax.set_ylim(-0.25, 0.40)
        ax.set_title(f"{sid} (x = {x:.3f} m, {s_['type']}{', ' + s_['subtype'] if s_.get('subtype') else ''})",
                     fontsize=8)
        ax.grid(alpha=0.2)
    fig.suptitle("Çerçeve kesitleri: kesikler (kırmızı), kesişen teçhizat / şasi (kutular), kablo demetleri (turuncu), "
                 "halka çerçevelerin açıklığı (beyaz)", fontsize=10)
    fig.tight_layout()
    p = fig_dir / "yk250_layout_sections.png"
    fig.savefig(p, dpi=110)
    plt.close(fig)
    return p


# =====================================================================================================================
# report (Turkish)
# =====================================================================================================================
TR_ITEMS = [
    ("part_numbers: module ranges", "parça numarası aralıkları modüller arasında çakışmıyor"),
    ("part ids: convention", "parça kimlikleri YK250-<KOD>-NNN kuralına ve modül aralığına uyuyor"),
    ("part ids unique", "parça kimlikleri yerleşim içinde tekil"),
    ("root_part", "kök parça (orta kanat kutusu) bir şasi elemanı"),
    ("references", "çapraz başvurular çözülüyor (temas, iniş yüzeyleri, RF pencereleri, mafsallar, açıklık seçicileri, "
                   "kütle kalemleri, erişim kapakları)"),
    ("stations ordered", "istasyonlar x boyunca sıralı, kimlikler tekil"),
    ("stations: type", "istasyon türü, malzeme/süreç/katman anahtarları, kalınlık >= süreç alt sınırı, yüzler"),
    ("cut-outs (pass-throughs)", "geçiş kesikleri çerçeve gövdesi içinde"),
    ("frames do not cut", "çerçeveler yakıt, taret, takım kuyusu, yük/paraşüt/teçhizat hacimlerini kesmiyor"),
    ("fuel bays conform", "yakıt bölmeleri ok açılı kiriş çerçevelerini izliyor (çerçeve gövdesine boşluk)"),
    ("usable fuel volume", "ok açılı bölmelerin kullanılabilir yakıt hacmi >= gerekli hacim"),
    ("harness / push-rod / bridle", "kablo, itme çubuğu ve kayış geçişleri tanımlı kesiklerden"),
    ("equipment envelopes inside", "teçhizat zarfları dış yüzeyin (OML) içinde, 10 mm pay"),
    ("member envelopes inside", "şasi elemanları OML içinde (kaplamaya oturan yüzler hariç)"),
    ("fitting envelopes inside", "bağlantı parçaları OML / kuyruk içinde"),
    ("actuator envelopes inside", "kanat / dikey eyleyicileri kesit içinde, 3 mm pay"),
    ("harness trunks inside", "kablo demetleri OML içinde (yarıçap + 3 mm)"),
    ("engine mount truss inside", "motor bağlantı kafesi kaporta içinde"),
    ("turret growth envelope", "taret büyüme zarfı (toplanmış) OML içinde"),
    ("no overlaps", "içerik / yapı çakışması yok"),
    ("gear retraction sequence", "takım toplama dizisi"),
    ("turret E180 envelope along", "taret E180 zarfı strok boyunca bölme duvarları / tavan / çerçevelere"),
    ("turret E180 envelope vs elevator", "taret E180 zarfı asansör raylarına / bilyalı vidaya"),
    ("turret E180 envelope vs sliding", "taret E180 zarfı kayar kapaklara (bağlı dizi)"),
    ("HD59 ball vs aperture", "HD59 topu ile açıklık halkası arası (radyal)"),
    ("stabilators (whole range, both sides)", "stabilatörler (tüm sapma) egzoz zarflarına"),
    ("stabilators (whole range) outside", "stabilatörler egzoz duman konisinin dışında"),
    ("ailerons / flaps", "kanatçık / flap (tüm sapma) kanat eyleyicilerine"),
    ("parachute hatch", "paraşüt kapağı (0-110°) dış antenlere, sondalara, ışıklara"),
    ("layout.mass_placement =", "kütle yerleşimi yerleşim nesnelerinden yeniden hesaplananla aynı"),
    ("spec mass items at", "spec kütle kalemleri yerleşim konumlarında (sizing --update-spec uygulandı)"),
    ("empty-aircraft CG", "boş uçak AM'si: spec kalemleri ile yerleşim kalemleri arasındaki fark"),
    ("turret field of regard", "taret görüş alanı: bütün dış çıkıntılar -5° konisinin üstünde"),
    ("RF windows", "RF pencereleri: iç antenlerin tümü GFRP panel / uç kapağı altında"),
    ("GNSS antennas", "GNSS antenleri üst yüzey pencerelerinin altında"),
    ("Li-ion buffer battery", "Li-ion tampon batarya ile yakıt hücreleri arası"),
    ("fuel cells to the firewall", "yakıt hücreleri ile yangın perdesi ön yüzü arası (CS-LUAS.967(c))"),
    ("engine dynamic envelope", "motor dinamik zarfı ile diğer nesneler arası"),
    ("engine-mount truss vs SG750", "motor bağlantı kafesi ile SG750 arası"),
    ("cylinder/head hot zone", "silindir/kafa sıcak bölgesi ile kompozit yapı arası"),
    ("exhaust routing envelopes vs unshielded", "egzoz zarfı ile kalkansız kompozit yapı arası"),
    ("exhaust routing envelopes vs harness", "egzoz zarfı ile kablo demetleri arası"),
    ("ventral fin vs exhaust", "ventral kanatçık ile egzoz zarfı / duman konisi"),
    ("propeller disc keep-out", "pervane diski yasak bölgesi boş"),
    ("parachute deployment volume", "paraşüt açılma hacmi boş"),
    ("shell panels:", "kabuk panelleri: kenar payı, aralık, menteşe, RF malzemesi"),
    ("upper body covered", "üst gövde burundan kaporta çıkışına kadar panellerle kaplı"),
    ("maintenance: every", "bakım: her teçhizat ve yakıt hücresi sökülebilir/menteşeli bir kapağın altında"),
    ("maintenance access matrix", "bakım erişim matrisi: hiçbir kalem için birincil yapı sökülmüyor"),
    ("assembly steps numbered", "montaj adımları 1..N, Türkçe başlık/alt montaj/metin/takım/kontrol"),
    ("assembly covers", "montaj şasi tezgâhından ayara kadar bütün grupları kapsıyor"),
    ("transport units", "taşıma birimleri R-29 / R-30 ile uyumlu"),
    ("field re-assembly", "sahada montaj ve bakım matrisi mevcut"),
    ("mechanism definitions", "mekanizma tanımları (alanlar, aralıklar, özellikler, ifadeler, L/R çiftleri, diziler)"),
    ("layout.clearances in checks.py", "layout.clearances checks.py LISTE biçiminde"),
    ("first-cut fitting pre-sizing", "bağlantıların ilk ön boyutlandırması: emniyet payları >= 0"),
]


_TR_WORDS = (("main inner door", "ana takım iç kapağı"), ("main tyre", "ana takım lastiği"), ("main leg", "ana takım bacağı"),
             ("nose tyre", "burun takımı lastiği"), ("nose leg", "burun takımı bacağı"),
             ("vs structure/contents", "– yapı / içerik"), ("vs moving gear", "– hareketli takım"))
_TR_LIMITS = (("(skin faces excepted)", "(kaplamaya oturan yüzler hariç)"), (" (boxes) / outside (plume)",
              " (kutular) / koni dışında (duman)"), (" deg", "°"))


def _dec(text: str) -> str:
    """Decimal comma for Turkish text (the bolt property class 12.9 is kept)."""
    return re.sub(r"(\d)\.(\d)", r"\1,\2", text.replace("12.9", "12§9")).replace("12§9", "12.9")


def _tr_limit(v) -> str:
    if not isinstance(v, str):
        return _fmt(v)
    for k, t in _TR_LIMITS:
        v = v.replace(k, t)
    return re.sub(r"(\d)\.(\d)", r"\1,\2", v)


def _tr(item: str) -> str:
    m = re.match(r"gear retraction sequence \((\d+) states\): (.*)", item)
    if m:
        t = m.group(2)
        for k, v in _TR_WORDS:
            t = t.replace(k, v)
        return f"takım toplama dizisi ({m.group(1)} durum): {t}"
    for k, v in TR_ITEMS:
        if item.startswith(k):
            return v
    return item


def _fmt(v):
    if v is None:
        return "–"
    if isinstance(v, float):
        return f"{v:.4g}".replace(".", ",")
    if isinstance(v, list):
        return "[" + "; ".join(_fmt(x) for x in v) + "]"
    return str(v)


def write_report(ctx: Ctx, res: dict, figs: dict, out_dir) -> Path:
    S, L = ctx.S, ctx.L
    out_dir = Path(out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    W = []
    w = W.append
    w("# YK-250 HANÇER — Yerleşim ve yapı arayüzü kontrolleri\n")
    w("Bu rapor `python3 -m ucav250.analysis.layout_check` tarafından `ucav250/spec.yaml` (`layout`, `assembly`) "
      "bölümlerinden üretilir; ayrıntılı tasarım modülleri (şasi, kanat, kuyruk, kabuk, itki, yakıt, takım, sistemler, "
      "faydalı yük) birbirlerinin geometrisini okumadan bu arayüzden çalışır. Açıklama ve gerekçeler: "
      "`docs/03_yerlesim_ve_yapi_konsepti.md`.\n")
    w(f"**Sonuç: {res['n_pass']}/{res['n']} kontrol geçti** ({res['n_objects']} yerleşim nesnesi, {res['close_pairs']} "
      f"yakın çift, süre {_fmt(res['elapsed_s'])} s).\n")
    w("## 1. Kontrol özeti\n")
    w("| No | Kontrol | Değer | Sınır | Sonuç |\n|---|---|---|---|---|")
    for r in res["rows"]:
        det = f" — {r['detail']}" if r["detail"] and not r["ok"] else ""
        w(f"| {r['check']} | {_tr(r['item'])}{det} | {_fmt(r['value'])} | {_tr_limit(r['limit'])} | "
          f"{'GEÇTİ' if r['ok'] else '**KALDI**'} |")
    cg = res["cg"]
    w("\n## 2. Kütle yerleşimi ve ağırlık merkezi\n")
    w(f"Boş kütle {_fmt(cg['empty_kg'])} kg; spec kalemlerinden AM {_fmt(cg['cg_spec'])} m, yerleşim nesnelerinden "
      f"AM {_fmt(cg['cg_layout'])} m. Aşağıdaki kalemlerin konumu boyutlandırma evresinde varsayımdı; yerleşim evresinde "
      "yerleştirilen nesnelerin ağırlık merkezinden hesaplanır (`layout.mass_placement`) ve `sizing.mass_items` bunu "
      "uygular. Boyutlandırma evresi konumları `position_sizing_phase` alanındadır.\n")
    w("| Kütle kalemi | Boyutlandırma evresi x (m) | Yerleşim x (m) | Δx (m) | Esas |\n|---|---|---|---|---|")
    for k, v in L["mass_placement"].items():
        ps = v.get("position_sizing_phase")
        dx = (v["position"][0] - ps[0]) if ps else None
        w(f"| {k} | {_fmt(ps[0]) if ps else '–'} | {_fmt(v['position'][0])} | {_fmt(dx)} | {v['basis'][:110]} |")
    w("\n## 3. Arayüz bağlantılarının ilk ön boyutlandırması\n")
    w("Yöntem: kapalı biçimli pim kesme / eğilme / ezilme ve cıvata grubu kesmesi, `spec.materials` izin verilen "
      "değerleri; yük katsayıları `structures` (FoS 1,5, bağlantı 1,15, pimli bağlantı ezilme 2,0, sık sökülen "
      "bağlantı 1,5) ve standards.yaml önerileri. Ayrıntılı tasarım SE/test ile değiştirir.\n")
    w("| Kalem | Uygulanan | İzin verilen | Emniyet payı | Esas |\n|---|---|---|---|---|")
    for r in res["presizing"]:
        w(f"| {r.get('item_tr', r['item'])} | {_fmt(r['applied'])} | {_fmt(r['allowable'])} | {_fmt(r['MS'])} | "
          f"{_dec(r.get('basis_tr', r['basis']))} |")
    w("\n## 4. İstasyonlar (çerçeveler / perdeler)\n")
    w("| Kimlik | x (m) | Tür | Malzeme / süreç / katman | t (mm) | Kesikler |\n|---|---|---|---|---|---|")
    for s_ in L["stations"]:
        cuts = ", ".join(c["id"] for c in s_.get("cutouts", [])) or "–"
        w(f"| {s_['id']} | {_fmt(float(s_['x']))} | {s_['type']}{' / ' + s_['subtype'] if s_.get('subtype') else ''} "
          f"| {s_['material']} / {s_['process']} / {s_.get('layup') or '–'} | {_fmt(float(s_['t']) * 1000)} | {cuts} |")
    w("\n## 5. Şasi elemanları ve bağlantılar\n")
    w("| Kimlik | Parça | Ad | Malzeme | Yük yolu |\n|---|---|---|---|---|")
    for m in L["chassis"]["members"]:
        w(f"| {m['id']} | {m['part']}{' (L/R)' if m.get('mirror') else ''} | {m.get('name_tr', m['name'])} | "
          f"{m['material']} | {m['load_path'][:120]} |")
    for f in L["chassis"]["fittings"]:
        w(f"| {f['id']} | {f['part']}{' (L/R)' if f.get('mirror') else ''} | {f.get('name_tr', f['name'])} | "
          f"{f['material']} | {str(f.get('attach', ''))[:120]} |")
    wj = L["chassis"]["wing_joint"]
    pins = wj["main_spar"]["pins"]
    w(f"\nDış panel birleşimi y = {_fmt(float(wj['plane']['y']))} m: dil-çatal, iki Ø{pins[0]['diameter'] * 1000:.0f} mm "
      f"Ti-6Al-4V pim ({', '.join(str(_r(p['position'], 3)) for p in pins)}), arka kirişte Ø"
      f"{wj['rear_spar']['pin']['diameter'] * 1000:.0f} mm sürükleme pimi; takma yolu ana kiriş ekseni boyunca "
      f"{_fmt(wj['insertion']['stroke'])} m.\n")
    em = L["chassis"]["engine_mount"]
    w(f"Motor bağlantısı: {em['type']} — 4 x M8 cıvata, sönümleyici: {em['isolators']['make_model']}; yangın perdesi "
      f"yığını {', '.join(l_['layer'] + ' ' + _fmt(l_['t'] * 1000) + ' mm' for l_ in em['firewall_stackup']['layers_fwd_to_aft'])}.\n")
    br = L["chassis"]["parachute"]["bridle"]
    w(f"Paraşüt: Y-kayış, ön ayak {_fmt(br['forward_leg']['length'])} m ({br['forward_leg']['fitting']}), arka ayak "
      f"{_fmt(br['aft_leg']['length'])} m ({br['aft_leg']['fitting']}); açılma yükü {_fmt(PARA_OPEN_N / 1000)} kN tek "
      "ayakta, nihai yalnız "
      "(CRASH-004).\n")
    w("## 6. Mekanizmalar\n")
    w("| Mafsal | Tür | Eksen | Alt / üst | Özellik / ifade |\n|---|---|---|---|---|")
    for j in L["mechanisms"]["joints"]:
        w(f"| {j['name']} | {j['kind']} | {_r(j['axis'], 3)} | {_fmt(float(j['lo']))} / {_fmt(float(j['hi']))} | "
          f"{j.get('prop') or ''} {('`' + j['expr'] + '`') if j.get('expr') else ''} |")
    for k, sq in L["mechanisms"]["sequences"].items():
        w(f"\nDizi `{k}`: {len(sq['states'])} örnek durum, denetim özelliği `{sq['control']}`.")
    w("\n## 7. Kabuk\n")
    cnt = {}
    for p in L["shell"]["panels"]:
        cnt[p["attach"]] = cnt.get(p["attach"], 0) + (2 if p.get("mirror") else 1)
    rf = [p["id"] for p in L["shell"]["panels"] if p.get("rf_window")]
    att_tr = {"removable": "sökülebilir", "fixed": "sabit", "hinged": "menteşeli", "fairing": "fileto"}
    w(f"Panel sayısı (L/R ayrı): {', '.join(f'{att_tr.get(k, k)} {v}' for k, v in cnt.items())}; RF pencereleri: "
      f"{', '.join(rf)}.\n")
    w("## 8. Bakım erişim matrisi\n")
    w("| Kalem | Erişim | Bağlantı elemanı | Birincil yapı sökülür mü |\n|---|---|---|---|")
    for r in S["assembly"]["maintenance_access"]:
        w(f"| {r.get('item_tr', r['item'])} | {', '.join(r.get('access_tr', r['access']))} | "
          f"{r.get('fasteners_tr', r['fasteners'])} | "
          f"{'evet' if r['primary_structure_removed'] else 'hayır'} |")
    w("\n## 9. Montaj ve taşıma\n")
    st_ = S["assembly"]["steps"]
    w(f"{len(st_)} montaj adımı (spec.assembly.steps): " + "; ".join(f"{s_['step']}. {s_['title_tr']}" for s_ in st_) +
      ".\n")
    w(S["assembly"]["transport"]["text_tr"] + "\n")
    w("## 10. Şekiller\n")
    for k, v in (figs or {}).items():
        w(f"![{k}](../docs/fig/{Path(v).name})")
    text = "\n".join(W) + "\n"
    (out_dir / "layout.md").write_text(text, encoding="utf-8")
    js = {k: v for k, v in res.items() if k != "cg"}
    js["cg"] = {k: v for k, v in res["cg"].items() if k != "placements"}
    (out_dir / "layout.json").write_text(json.dumps(js, indent=1, default=str), encoding="utf-8")
    return out_dir / "layout.md"


if __name__ == "__main__":
    sys.exit(main())
