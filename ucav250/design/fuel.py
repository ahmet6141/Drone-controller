"""Fuel producer (``layout.part_numbers.fuel``, YK250-FU-570..619): three flexible bladder cells with their vulcanised
fitting flanges, the plumbing of ``layout.fuel_lines`` (refuel, saddle fill, bottom interconnects, saddle drain, feed,
return, top vents with the anti-siphon loop, sump drains), check / float valves, the rigid collector (header tank) in
the feed cell, capacitive level probes, the dry-break refuel coupling and its bracket, the frame pass-through seal sets,
the firewall bulkhead block with the shut-off valve, the EFI pump / regulator / gascolator unit, the aft-bay line
support bracket with its cushioned clamps, the skin-end fittings (quick drains, flush vent) and the fuel contents
(``process: consumable``, mass = volume x density at the design fuel load).

Interfaces (``spec.layout`` only, never another module's geometry):

* cells: ``fuel_cells`` + ``chassis.fuel_supports`` (bay boundaries swept with the spar frames, OML inset, floors,
  liner thickness), ``chassis.members`` (forward fuel deck, well roof, centre box, chine longeron path / section, spine)
  and ``chassis.fittings`` (keep-outs: F-RISER-AFT nuts under the spine floor, F-TRUNNION dome nutplates on the well
  roof); the bladder floor sits on the deck / well roof liner, the saddle cell on the box covers (layout z0);
* lines: ``fuel_lines`` (path, diameter, penetrations, valves, end fittings) - every line follows its declared path
  (corners filleted / 90-degree hose ends where the legs are short), crosses frames inside the declared cut-outs
  (``stations[*].cutouts``) and decks / roofs in the declared penetrations;
* equipment: ``systems.equipment`` EQ-FUELPUMP (on TR-AFTBAY) and EQ-SHUTOFF (forward face of the firewall),
  ``chassis.trays`` TR-AFTBAY, ``stations`` FS3670 (C-FW-FUEL), ``shell.panels`` P-REFUEL / P-CENTRE-LOWER / P-AFTHATCH
  / P-AFT-LOWER (skin-end fittings; holes are drilled in the panels when the shell module is built);
* assembly: ``assembly.steps`` step 18 (fuel system), 38 (fuel loaded for the function tests).

Manufacturing: bladders are bought to our drawing (ATL ultralight 0.25 mm coated fabric with vulcanised 6061-T6
fitting flanges, components.yaml atl_ultralight_bladder); lines are bought PTFE-lined braided hose assemblies (feed,
return, fill, interconnects) or PA12 tube assemblies (vents, drains) made to length with crimped / swaged ends; the
collector is SLS PA12 (fuel resistant, 2 mm wall); brackets are 6061-T6 sheet (press brake); the firewall block is
machined AISI 304; pass-through seals are moulded fluorosilicone. Fastener edge distances >= 2 D (metal) / 2.5 D
(composite), ISO 2768-mK, ISO 273 medium clearance holes, metric fasteners (design/joints.py).
"""
from __future__ import annotations

import math

import numpy as np
from shapely.geometry import Point, Polygon, box as sbox
from shapely.ops import unary_union

from ..core import geom as G
from ..core.parts import Part, Registry, mirror_part, part_number
from . import fastener_catalog as FC
from . import joints as J
from . import oml as O
from . import structgen as SG

# =====================================================================================================================
# module constants (detailing choices; interface values come from spec.layout)
# =====================================================================================================================
OV = 1e-4                   # boolean overlap of fused features
GAP = 0.0002                # bladder outer surface to liner / floor (bladder conforms to the supported cavity)
TOP_GAP = 0.0005            # bladder top below the layout cell top z1 (spine floor 1 mm above)
T_BLAD = 0.00025            # ATL ultralight bladder wall (components.yaml atl_ultralight_bladder.wall_thickness_m)
BLAD_AREAL = 0.33           # kg/m2 coated-fabric wall (components.yaml atl_ultralight_bladder.mass_kg basis)
FIT_T = 0.0075              # vulcanised bottom-fitting flange (6061-T6): blind M3 thread, engagement >= 1.2 D
FIT_RIM = 0.0065            # flange material around a bolt (>= 2 D metal for M3, + 0.5 mm)
FIT_SEAL = 0.004            # flange land around the structure bore (O-ring face seal)
BOLT_FIT = 3                # M3 A2-70 bottom-fitting screws from the dry side, tapped into the flange
CSK = 0.006                 # countersunk sump around the outlet bore (the outlet drains to the bladder floor)
BORE_CLR = 0.00005          # radial clearance line / bore (O-ring seal; declared contact)
BOSS_OUT = 0.006            # wall-port boss length outside the bladder (limited by the gap to the frame)
BOSS_RIM = 0.005            # wall-port boss radius beyond the line
GROM_FL = 0.0015            # pass-through seal flange thickness on each frame face
GROM_RIM = 0.004            # seal flange beyond the cut-out
GROM_CLR = 0.0002           # seal plug to cut-out clearance
GROM_FACE = 0.00005         # seal flange to frame face (declared contact)
FLUORO_RHO = 1400.0         # fluorosilicone density (kg/m3, estimate)
HOSE_RHO = 2400.0           # PTFE liner + stainless braid effective density over the hose wall (estimate)
PA12_RHO = 1010.0           # PA12 tube (estimate)
FIT_RHO = 2713.0            # aluminium fittings (spec.materials al_6061_t6_sheet density)
SS_RHO = 7916.0             # AISI 304 (spec.materials ss_304_annealed density)
FV = dict(r=0.011, h=0.020, mass=0.05)          # float / roll-over vent valve (components.yaml rollover_vent_valve)
CV = dict(r_add=0.003, L=0.020, sock=0.006, mass=0.03)   # inline flapper check valve body (estimate)
CVB = dict(r=0.011, h=0.003, mass=0.02)        # flush flapper valve on a bottom fitting (estimate)
PROBE = dict(r=0.006, head_r=0.015, head_h=0.008, boss_r=0.019, mass=0.060)  # Gill capacitive probe (60 g)
COUPLING = dict(r=0.016, L=0.040, flange_r=0.019, flange_h=0.006, nut_h=0.006, mass=0.067)  # OBP AN6 airframe half
COLLECTOR = dict(depth=0.036, y=(-0.062, 0.102), top=0.010, wall=0.002, inlet_r=0.006)
PUMP = dict(mass=0.35)      # engine.installed_items_kg.fuel_pump_regulator_filter (Limbach EFI supply kit)
GASCOLATOR = dict(r=0.019, mass=0.152)   # components.yaml andair_gas375 (0.152 kg manufacturer); bowl d estimate
SHUTOFF = dict(mass=0.08)   # layout EQ-SHUTOFF (estimate)
DRAIN = dict(r=0.0055, nut_r=0.0085, nut_h=0.004, mass=0.02)   # flush quick-drain valve (estimate)
VENT_OUT = dict(r=0.008, flange_r=0.014, flange_h=0.0015, mass=0.025)  # flush vent outlet with flame arrestor
BRACKET_T = 0.002           # 6061-T6 sheet brackets
# aft-bay line support: web plane (forward face) at about mid-span of FL-FEED-2 / FL-RETURN, bolts on the tray web
# between its lightening holes (edge distance checked on the tray geometry), bush wall, web margin round the bushes
SUPPORT = dict(x_web=3.377, y_bolts=(0.022, 0.088), edge=0.009, bush_wall=0.0025, web_rim=0.003)
SS_PLATE_T = 0.0015         # firewall block flange (cnc_milling_metal minimum 1.5 mm)
STEP = 18                   # assembly step "Yakıt sistemi"
STEP_LOAD = 38              # fuel contents: loaded for the function tests

MAT_AL, MAT_SS, MAT_PA12 = "al_6061_t6_sheet", "ss_304_annealed", "pa12_sls"
P_SHEET, P_CNC, P_SLS, P_BUY = "sheet_metal_aluminium", "cnc_milling_metal", "sls_pa12", "purchased"

# part numbers (layout.part_numbers.fuel = 570..619): layout-fixed ids are read from the layout; these are the ids
# this module adds inside its range
N = dict(collector=573, coupling=574, coupling_bracket=575, fw_block=577, probe_f=578, probe_s=579, probe_a=595,
         fuel_f=596, fuel_s=597, fuel_a=598, grom_ms=600, grom_rs=601, grom_gear=602, cv_fs=603, cv_fa=604,
         cv_sa=605, fv_f=606, fv_s=607, fv_s2=608, fv_a=609, fv_a2=610, vent_out=611, drain_f=612, drain_a=613,
         drain_g=614, support=615, clamps=616, gascolator=617)
CELL_KEYS = ("forward_cell", "saddle_cell", "aft_cell")
PROBE_AT = {"forward_cell": (2.33, 0.15), "saddle_cell": (2.64, 0.10), "aft_cell": (2.95, 0.15)}   # (x, y)
RING_TRIES = (256, 244, 268, 232, 280)   # bladder loft ring point counts tried in turn


# =====================================================================================================================
# small geometry helpers
# =====================================================================================================================
def box3(lo, hi) -> G.Mesh:
    lo, hi = np.asarray(lo, float), np.asarray(hi, float)
    return G.box(hi - lo, center=0.5 * (lo + hi))


def prism_x(poly, x0: float, x1: float) -> G.Mesh:
    """(y, z) polygon extruded along +x from x0 to x1."""
    return G.extrude(poly, x1 - x0, origin=(x0, 0.0, 0.0), u=(0.0, 1.0, 0.0), v=(0.0, 0.0, 1.0))


def prism_z(poly, z0: float, z1: float) -> G.Mesh:
    """(x, y) polygon extruded along +z from z0 to z1."""
    return G.extrude(poly, z1 - z0, origin=(0.0, 0.0, z0), u=(1.0, 0.0, 0.0), v=(0.0, 1.0, 0.0))


def finish(m: G.Mesh) -> G.Mesh:
    """Round-trip through manifold with a 1 micrometre simplification: removes the sliver triangles that booleans
    leave where a cutting face passes within a few micrometres of a vertex (shape and volume unchanged)."""
    return G.Mesh.from_manifold(m.to_manifold().simplify(1e-6))


def union(ms) -> G.Mesh:
    ms = [m for m in ms if m is not None]
    return ms[0] if len(ms) == 1 else G.union(ms)


def diff(a: G.Mesh, cutters) -> G.Mesh:
    cutters = [c for c in cutters if c is not None]
    return G.difference(a, cutters) if cutters else a


def inter(a: G.Mesh, b: G.Mesh) -> G.Mesh:
    r = G.intersection(a, b)
    if r is None:
        raise ValueError("empty intersection")
    return r


def shear_x(m: G.Mesh, k: float, x_ref: float = 0.0) -> G.Mesh:
    """x += |y| k (one half of a chevron: build the mesh on one side of y = 0 only)."""
    V = m.V.copy()
    V[:, 0] += np.abs(V[:, 1]) * k
    return G.Mesh(V, m.F)


def chevron_prism(poly, x_face0: float, k: float, t0: float, t1: float) -> G.Mesh:
    """Solid between the swept planes x = x_face0 + |y| k + t0 and + t1 over the (y, z) polygon ``poly``: built per
    half (y >= 0, y <= 0, overlapping 0.1 mm across the kink) and sheared."""
    parts = []
    lo, hi = poly.bounds[0], poly.bounds[2]
    for s in (1, -1):
        clip = sbox(-OV, -10, 10, 10) if s > 0 else sbox(-10, -10, OV, 10)
        half = poly.intersection(clip)
        if half.is_empty or half.area < 1e-10:
            continue
        for p in getattr(half, "geoms", [half]):
            if p.area < 1e-10:
                continue
            m = prism_x(p, x_face0 + t0, x_face0 + t1)
            parts.append(shear_x(m, k) if k else m)
    if not parts:
        raise ValueError("chevron_prism: empty")
    del lo, hi
    return union(parts) if len(parts) > 1 else parts[0]


def wedge(xa: float, ka: float, xb: float, kb: float, zlo: float = -1.0, zhi: float = 1.0, ylim: float = 0.6):
    """Region x_a(y) <= x <= x_b(y) with x(y) = x_c + |y| k (swept chevron boundaries), as the union of two hulls."""
    halves = []
    for s in (1, -1):
        V = []
        for y in (-OV, ylim):
            ya = abs(y)
            V += [(xa + ya * ka, s * y, zlo), (xb + ya * kb, s * y, zlo), (xb + ya * kb, s * y, zhi),
                  (xa + ya * ka, s * y, zhi)]
        halves.append(G.hull(np.asarray(V)))
    return G.union(halves)


def _perp(a):
    a = G.unit(a)
    ref = np.array([0.0, 0.0, 1.0]) if abs(a[2]) < 0.9 else np.array([1.0, 0.0, 0.0])
    e1 = G.unit(np.cross(a, ref))
    return e1, np.cross(a, e1)


def disc(p, axis, r: float, t0: float, t1: float, n: int = 32) -> G.Mesh:
    a = G.unit(axis)
    p = np.asarray(p, float)
    return G.cylinder(r, p + t0 * a, p + t1 * a, n=n)


def ring(p, axis, ro: float, ri: float, t0: float, t1: float, n: int = 32) -> G.Mesh:
    a = G.unit(axis)
    p = np.asarray(p, float)
    return G.tube(ro, ri, p + t0 * a, p + t1 * a, n=n)


# ------------------------------------------------------------------ line paths
def seg_len(P) -> np.ndarray:
    return np.linalg.norm(np.diff(np.asarray(P, float), axis=0), axis=1)


def fillet(P, r: float, n_arc: int = 8, share: float = 0.48):
    """Polyline with every corner replaced by a circular arc of radius r (reduced to fit ``share`` of the shared legs,
    the whole end legs). Returns (points, list of (index range, radius) of the arcs)."""
    P = np.asarray(P, float)
    keep = np.r_[True, seg_len(P) > 1e-9]
    P = P[keep]
    if len(P) < 3:
        return P, []
    L = seg_len(P)
    out = [P[0]]
    arcs = []
    for i in range(1, len(P) - 1):
        a, b, c = P[i - 1], P[i], P[i + 1]
        u, v = G.unit(a - b), G.unit(c - b)
        cphi = float(np.clip(np.dot(u, v), -1.0, 1.0))
        phi = math.acos(cphi)                       # interior angle
        if math.pi - phi < 1e-3:                    # straight through
            out.append(b)
            continue
        avail_a = L[i - 1] * (share if i - 1 > 0 else 0.9)
        avail_c = L[i] * (share if i + 1 < len(P) - 1 else 0.9)
        t = min(r / math.tan(phi / 2), avail_a, avail_c)
        R = t * math.tan(phi / 2)
        p1, p2 = b + u * t, b + v * t
        cen = b + G.unit(u + v) * (R / math.sin(phi / 2))
        d1, d2 = p1 - cen, p2 - cen
        ang = math.acos(float(np.clip(np.dot(G.unit(d1), G.unit(d2)), -1, 1)))
        k0 = len(out)
        for s in np.linspace(0.0, 1.0, n_arc):
            # slerp between d1 and d2
            w1 = math.sin((1 - s) * ang) / math.sin(ang)
            w2 = math.sin(s * ang) / math.sin(ang)
            out.append(cen + w1 * d1 + w2 * d2)
        arcs.append(((k0, len(out) - 1), R))
    out.append(P[-1])
    Q = np.asarray(out)
    keep = np.r_[True, seg_len(Q) > 1e-7]
    return Q[keep], arcs


def trim(Q, s0: float, s1: float) -> np.ndarray:
    """Polyline with arc length s0 removed at the start and s1 at the end (negative values extend straight)."""
    Q = np.asarray(Q, float).copy()
    for which, s in ((0, s0), (1, s1)):
        if which == 1:
            Q = Q[::-1].copy()
        if s < 0:
            Q[0] = Q[0] + s * G.unit(Q[1] - Q[0])
        elif s > 0:
            L = np.r_[0.0, np.cumsum(seg_len(Q))]
            i = int(np.searchsorted(L, s, side="right")) - 1
            i = min(i, len(Q) - 2)
            t = (s - L[i]) / max(L[i + 1] - L[i], 1e-12)
            q = Q[i] + t * (Q[i + 1] - Q[i])
            Q = np.vstack([q, Q[i + 1:]])
        if which == 1:
            Q = Q[::-1].copy()
    return Q


def rod(Q, r: float, n: int = 20) -> G.Mesh:
    return G.sweep_circle(np.asarray(Q, float), r, n=n)


def _frames(P):
    """Parallel-transport frames (T, N, B) along the polyline P (no repeated points)."""
    T = np.gradient(P, axis=0)
    T /= np.maximum(np.linalg.norm(T, axis=1, keepdims=True), 1e-12)
    ref = np.array([0.0, 0.0, 1.0]) if abs(T[0, 2]) < 0.9 else np.array([1.0, 0.0, 0.0])
    N_ = G.unit(np.cross(T[0], ref))
    Ns, Bs = [], []
    for i in range(len(P)):
        if i > 0:
            b = np.cross(T[i - 1], T[i])
            sb = np.linalg.norm(b)
            if sb > 1e-9:
                N_ = G.rot_axis_angle(b, math.atan2(sb, float(np.dot(T[i - 1], T[i])))) @ N_
            N_ = G.unit(N_ - np.dot(N_, T[i]) * T[i])
        Ns.append(N_.copy())
        Bs.append(np.cross(T[i], N_))
    return T, np.asarray(Ns), np.asarray(Bs)


def tube_sweep(stations, n: int = 20) -> G.Mesh:
    """Closed hollow tube along a polyline built directly (no booleans): ``stations`` = [(point, r_out, r_in)];
    repeated points with different radii make planar steps (nipples, sockets)."""
    pts = np.asarray([s[0] for s in stations], float)
    uniq, idx = [], []
    for p in pts:
        if not uniq or np.linalg.norm(p - uniq[-1]) > 1e-9:
            uniq.append(p)
        idx.append(len(uniq) - 1)
    U = np.asarray(uniq)
    _T, Ns, Bs = _frames(U)
    th = np.linspace(0.0, 2 * math.pi, n, endpoint=False)
    c, s_ = np.cos(th)[:, None], np.sin(th)[:, None]
    V, ko, ki = [], [], []
    for k, (p, ro, ri) in enumerate(stations):
        j = idx[k]
        ring_o = np.asarray(p) + ro * (c * Ns[j] + s_ * Bs[j])
        ring_i = np.asarray(p) + ri * (c * Ns[j] + s_ * Bs[j])
        ko.append(len(V))
        V.extend(ring_o)
        ki.append(len(V))
        V.extend(ring_i)
    V = np.asarray(V)
    F = []
    m = len(stations)
    for i in range(m - 1):
        for j in range(n):
            j2 = (j + 1) % n
            a, b, cc, d = ko[i] + j, ko[i + 1] + j, ko[i + 1] + j2, ko[i] + j2
            F += [(a, b, cc), (a, cc, d)]
            a, b, cc, d = ki[i] + j, ki[i] + j2, ki[i + 1] + j2, ki[i + 1] + j
            F += [(a, b, cc), (a, cc, d)]
    for j in range(n):
        j2 = (j + 1) % n
        F += [(ko[0] + j, ko[0] + j2, ki[0] + j2), (ko[0] + j, ki[0] + j2, ki[0] + j)]
        e = m - 1
        F += [(ko[e] + j2, ko[e] + j, ki[e] + j), (ko[e] + j2, ki[e] + j, ki[e] + j2)]
    F = np.asarray(F)
    # drop the zero-area side faces between repeated stations of equal radius
    tri = V[F]
    ar = 0.5 * np.linalg.norm(np.cross(tri[:, 1] - tri[:, 0], tri[:, 2] - tri[:, 0]), axis=1)
    mesh = G.weld(G.Mesh(V, F[ar > 1e-16]), 1e-10)
    return G.fix_orientation(mesh)


def plane_cross(P, f):
    """First point where the scalar function f changes sign along the polyline P (linear interpolation)."""
    P = np.asarray(P, float)
    vals = [f(p) for p in P]
    for i in range(len(P) - 1):
        a, b = vals[i], vals[i + 1]
        if a == 0:
            return P[i], i
        if a * b < 0:
            t = a / (a - b)
            return P[i] + t * (P[i + 1] - P[i]), i
    return None, None


def _resample_chain(P, k: int) -> np.ndarray:
    """k points evenly spaced by arc length along the open chain P, first point included, last excluded."""
    P = np.asarray(P, float)
    L = np.r_[0.0, np.cumsum(seg_len(P))]
    s = np.linspace(0.0, L[-1], k, endpoint=False)
    return np.column_stack([np.interp(s, L, P[:, 0]), np.interp(s, L, P[:, 1])])


def section_ring(poly: Polygon, z0: float, z1: float, n: int) -> np.ndarray:
    """Bay section ring with a fixed point layout (so that lofted rings correspond point by point): floor, starboard
    side split at its sharpest corner (the chine), top cap, port side; each piece resampled with a fixed count."""
    from shapely.geometry.polygon import orient as _orient
    P = np.asarray(_orient(poly, 1.0).exterior.coords)[:-1]
    tol = 1e-7
    i_fr = max((i for i in range(len(P)) if abs(P[i, 1] - z0) < tol), key=lambda i: P[i, 0])   # floor right
    i_tr = max((i for i in range(len(P)) if abs(P[i, 1] - z1) < tol), key=lambda i: P[i, 0])   # top right
    i_tl = min((i for i in range(len(P)) if abs(P[i, 1] - z1) < tol), key=lambda i: P[i, 0])   # top left
    i_fl = min((i for i in range(len(P)) if abs(P[i, 1] - z0) < tol), key=lambda i: P[i, 0])   # floor left

    def chain(i, j):
        idx = [i]
        while idx[-1] != j:
            idx.append((idx[-1] + 1) % len(P))
        return P[idx]

    def split_corner(Ch):
        best, bi = 0.0, None
        for k in range(1, len(Ch) - 1):
            u, v = Ch[k] - Ch[k - 1], Ch[k + 1] - Ch[k]
            nu, nv = np.linalg.norm(u), np.linalg.norm(v)
            if nu < 1e-12 or nv < 1e-12:
                continue
            ang = math.degrees(math.acos(float(np.clip(np.dot(u, v) / (nu * nv), -1, 1))))
            if ang > best:
                best, bi = ang, k
        return bi if best > 20.0 else None
    right, left = chain(i_fr, i_tr), chain(i_tl, i_fl)
    n_f, n_t = n // 4, n // 8
    n_s = (n - n_f - n_t) // 2
    pieces = [_resample_chain(chain(i_fl, i_fr), n_f)]
    k = split_corner(right)
    if k is None:
        pieces.append(_resample_chain(right, n_s))
    else:
        a_ = max(2, int(round(n_s * 0.3)))
        pieces += [_resample_chain(right[:k + 1], a_), _resample_chain(right[k:], n_s - a_)]
    pieces.append(_resample_chain(chain(i_tr, i_tl), n_t))
    k = split_corner(left)
    if k is None:
        pieces.append(_resample_chain(left, n_s))
    else:
        b_ = max(2, int(round(n_s * 0.3)))
        pieces += [_resample_chain(left[:k + 1], n_s - b_), _resample_chain(left[k:], b_)]
    return np.vstack(pieces)


# =====================================================================================================================
# context: layout lookups
# =====================================================================================================================
class Ctx:
    def __init__(self, reg: Registry, spec: dict):
        self.reg, self.S = reg, spec
        L = self.L = spec["layout"]
        self.fus = O.fuselage_from_spec(spec)
        self.st = {s["id"]: s for s in L["stations"]}
        self.mem = {m["id"]: m for m in L["chassis"]["members"]}
        self.fit = {f["id"]: f for f in L["chassis"]["fittings"]}
        self.fs = {f["cell"]: f for f in L["chassis"]["fuel_supports"]}
        self.cells = {c["name"]: c for c in L["fuel_cells"]}
        self.lines = {f["id"]: f for f in L["fuel_lines"]}
        self.eq = {e["id"]: e for e in L["systems"]["equipment"]}
        self.panels = {p["id"]: p for p in L["shell"]["panels"]}
        self.trays = {t["id"]: t for t in L["chassis"]["trays"]}
        procs = spec["processes"]
        lin = self.fs["forward_cell"]["liner"]
        # the liner is laid up from prepreg: never thinner than the process minimum (chassis builds it at that value)
        self.liner_t = max(float(lin["t_m"]), float(procs["prepreg_ooa_vacbag"]["min_thickness"]))
        self.rho = float(spec["engine"]["fuel"]["density_kg_per_m3"])
        self.fuel_kg = float(spec["mass"]["fuel_kg"])
        from ..core.parts import layup_props
        self.skin_t = layup_props(spec, "shell_secondary")["thickness"]
        self._sec = {}
        self.cell = {}

    # ------------------------------------------------------------------ OML
    def sec(self, x: float, inset: float) -> Polygon:
        key = (round(float(x), 6), round(float(inset), 7))
        if key not in self._sec:
            self._sec[key] = SG.fuselage_section2d(self.fus, float(x), float(inset), n=256)
        return self._sec[key]

    def z_bot(self, x: float, y: float) -> float:
        w, h, zc, nt, nb = self.fus.section(np.asarray(x, float))
        tf = self.fus.top_frac(np.asarray(x, float))
        r = np.clip(abs(y) / max(0.5 * float(w), 1e-9), 0, 1)
        return float(zc - (1 - tf) * h * np.clip(1 - r ** nb, 0, 1) ** (1 / nb))

    def z_top(self, x: float, y: float) -> float:
        w, h, zc, nt, nb = self.fus.section(np.asarray(x, float))
        tf = self.fus.top_frac(np.asarray(x, float))
        r = np.clip(abs(y) / max(0.5 * float(w), 1e-9), 0, 1)
        return float(zc + tf * h * np.clip(1 - r ** nt, 0, 1) ** (1 / nt))

    # ------------------------------------------------------------------ stations
    def face(self, sid: str, side: int):
        """(x at the centre line, sweep tangent) of the forward (side -1) or aft (side +1) web face of a frame."""
        st = self.st[sid]
        xf = st.get("x_faces") or [st["x"] - 0.5 * st["t"], st["x"] + 0.5 * st["t"]]
        return float(xf[0] if side < 0 else xf[1]), math.tan(math.radians(float(st.get("sweep_deg", 0.0))))

    def cutout(self, sid: str, cid: str, mirror: bool = False):
        c = next(c for c in self.st[sid]["cutouts"] if c["id"] == cid)
        y0, y1 = (float(v) for v in c["y"])
        if mirror:
            y0, y1 = -y1, -y0
        return y0, y1, float(c["z"][0]), float(c["z"][1])

    def member_box(self, mid: str):
        b = self.mem[mid]["box"]
        return np.asarray(b[0], float), np.asarray(b[1], float)


# =====================================================================================================================
# bladder cells
# =====================================================================================================================
class Cell:
    """One bladder cell: outer envelope (band of the bay inside the liner), inner (fuel) volume, fittings."""

    def __init__(self, C: Ctx, key: str):
        self.C, self.key = C, key
        fs, c = C.fs[key], C.cells[key]
        bf, ba = fs["boundary_fwd"], fs["boundary_aft"]
        self.xa, self.ka = float(bf["x_at_centre_line"]), math.tan(math.radians(float(bf["sweep_deg"])))
        self.xb, self.kb = float(ba["x_at_centre_line"]), math.tan(math.radians(float(ba["sweep_deg"])))
        self.inset = float(fs["inset_from_oml_m"])
        self.ylim = max(abs(float(v)) for v in fs["y_limits_m"])
        z0, z1 = (float(v) for v in c["z"])
        floor_mem = {"forward_cell": "M-FWDDECK", "aft_cell": "M-WELLROOF"}.get(key)
        if floor_mem:                      # bladder floor on the liner of the deck / well roof
            self.deck_top = float(C.member_box(floor_mem)[1][2])
            self.floor = self.deck_top + C.liner_t + GAP
            self.floor_member = C.mem[floor_mem]["part"]
        else:                              # saddle: on the centre-box covers / spar caps (layout z0)
            self.deck_top = None
            self.floor = z0
            self.floor_member = C.mem["M-CTBOX"]["part"]
        self.top = z1 - TOP_GAP
        self.fittings = []                 # meshes fused to the shell (flanges, bosses)
        self.keepouts = []                 # (lo, hi) boxes the bladder clears (structure hardware)
        self.bores = []                    # cutters through the shell / fittings
        self.ports = {}                    # name -> point on the wall
        self._outer = self._inner = None

    # ------------------------------------------------------------------ bay band
    def x_fwd(self, y: float) -> float:
        return self.xa + abs(y) * self.ka

    def x_aft(self, y: float) -> float:
        return self.xb + abs(y) * self.kb

    def x_start(self, t: float) -> float:
        """Smallest x of the envelope at the centre line (the loft starts there)."""
        return self.xa + t

    def region(self, t: float, z0: float, z1: float) -> G.Mesh:
        """Swept-boundary region of the bay (offset inward by t at the ends)."""
        return wedge(self.xa + t / math.cos(math.atan(self.ka)), self.ka,
                     self.xb - t / math.cos(math.atan(self.kb)), self.kb, z0 - 0.05, z1 + 0.05)

    def band(self, inset: float, z0: float, z1: float, t: float, n_ring: int) -> G.Mesh:
        x_lo = self.x_start(t) - 0.004
        x_hi = self.xb + self.ylim * self.kb + 0.004
        n = max(2, int(math.ceil((x_hi - x_lo) / 0.01)) + 1)
        rings = []
        for x in np.linspace(x_lo, x_hi, n):
            poly = SG.largest(self.C.sec(x, inset).intersection(sbox(-self.ylim, z0, self.ylim, z1)))
            P2 = section_ring(poly, z0, z1, n_ring)
            rings.append(np.column_stack([np.full(len(P2), x), P2[:, 0], P2[:, 1]]))
        return inter(G.loft(rings), self.region(t, z0, z1))

    def keepout_meshes(self, grow: float):
        out = []
        for lo, hi in self.keepouts:
            out.append(box3(np.asarray(lo) - grow, np.asarray(hi) + grow))
        return out

    def _build(self):
        """Outer envelope, inner (fuel) volume and the wall between them; the loft ring count is chosen so that
        all three are free of self-intersections (corner snapping of the resampled sections can fold a quad)."""
        t = T_BLAD
        last = None
        for n_ring in RING_TRIES:
            o = finish(diff(self.band(self.inset + GAP, self.floor, self.top, 0.0, n_ring), self.keepout_meshes(GAP)))
            i = finish(diff(self.band(self.inset + GAP + t, self.floor + t, self.top - t, t, n_ring),
                            self.keepout_meshes(GAP + t)))
            w = finish(diff(o, [i]))
            last = (o, i, w)
            if all(G.self_intersections(m) == 0 for m in (o, i, w)):
                break
        self._outer, self._inner, self._wall = last

    def outer_solid(self) -> G.Mesh:
        if self._outer is None:
            self._build()
        return self._outer

    def inner_solid(self) -> G.Mesh:
        if self._inner is None:
            self._build()
        return self._inner

    def shell(self) -> G.Mesh:
        if self._outer is None:
            self._build()
        m = self._wall
        if self.fittings:
            m = union([m] + self.fittings)
        return m


class AftCell(Cell):
    """Feed cell: the bladder's low forward edge reaches the rear-spar frame under the centre box (sump lip that
    carries the interconnect inlets forward of the main-well wall)."""

    def lip(self, t: float):
        C = self.C
        xr, k = C.face("FS-RS", +1)
        z_box = float(C.member_box("M-CTBOX")[0][2])
        return xr + C.liner_t + GAP + t, k, z_box - 0.001 - t

    def x_start(self, t: float) -> float:
        return self.lip(t)[0]

    def region(self, t: float, z0: float, z1: float) -> G.Mesh:
        x0, k, z_lip = self.lip(t)
        band = super().region(t, z0, z1)
        lip = wedge(x0, k, self.xa + 0.03, self.ka, z0 - 0.05, z_lip)
        return G.union([band, lip])


# =====================================================================================================================
# keep-outs of the bladders (structure and hardware inside the bays, from the layout)
# =====================================================================================================================
RISER_STACK = 0.012         # under the spine floor: 7075 washer plate 3 mm + ISO 7089 0.8 + ISO 7040 M4 5.0 + thread
DOME_NP_H = 0.012           # sealed dome nutplate M6 on the fuel side of the well roof (NUTPLATE H 7 mm + dome + shank)


def cell_keepouts(C: Ctx, cell: Cell) -> list:
    out = []
    lt = C.liner_t
    # chine longeron (J section along its path) crossing the low outboard corner of the forward / aft bays
    m = C.mem["M-CHINE"]
    w, h = float(m["section"]["w"]), float(m["section"]["h"])
    x_lo = cell.x_start(0.0) - 0.01
    x_hi = cell.xb + cell.ylim * cell.kb + 0.01
    for path in m["paths"]:
        P = np.asarray(path, float)
        a, b = max(P[0, 0] - 0.02, x_lo), min(P[-1, 0] + 0.02, x_hi)
        if a >= b:
            continue
        xs = np.linspace(a, b, 25)
        ys = np.interp(xs, P[:, 0], P[:, 1])
        zs = np.interp(xs, P[:, 0], P[:, 2])
        y0 = float(ys.min()) - 0.5 * w - lt
        # 10 mm below the J so that the notch swallows the mitred chine corner of the inset section (no sliver)
        z0, z1 = float(zs.min()) - 0.5 * h - lt - 0.010, float(zs.max()) + 0.5 * h + lt
        out.append(((a, y0, z0), (b, 1.0, z1)))
        out.append(((a, -1.0, z0), (b, -y0, z1)))
    # bridle aft fitting: bolts, washer plate and nuts under the spine floor (saddle cell top)
    sp_floor = float(min(bx[0][2] for bx in C.mem["M-SPINE"]["boxes"]))
    f = C.fit["F-RISER-AFT"]
    lo, hi = np.asarray(f["box"][0], float), np.asarray(f["box"][1], float)
    if hi[0] > x_lo and lo[0] < x_hi:
        out.append(((lo[0] - 0.003, lo[1] - 0.003, sp_floor - RISER_STACK), (hi[0] + 0.003, hi[1] + 0.003, 1.0)))
    # trunnion fittings: sealed dome nutplates of the well-roof bolts on the fuel side (aft cell floor)
    f = C.fit["F-TRUNNION"]
    roof = [b for b in f["bolts"] if "M-WELLROOF" in b.get("joins", [])]
    if roof and cell.key == "aft_cell":
        lo, hi = np.asarray(f["box"][0], float), np.asarray(f["box"][1], float)
        L_np = FC.NUTPLATE[6][0]
        y_lo = min(min(float(b["point"][1]) for b in roof), lo[1]) - 0.5 * L_np - 0.003
        z_roof = float(C.member_box("M-WELLROOF")[1][2])
        for s in (1, -1):
            a = (lo[0] - 0.003, y_lo, -1.0)
            b = (hi[0] + 0.003, 1.0, z_roof + DOME_NP_H)
            if s > 0:
                out.append((a, b))
            else:
                out.append(((a[0], -b[1], a[2]), (b[0], -a[1], b[2])))
    return out


# =====================================================================================================================
# line paths (layout.fuel_lines) adapted to the fittings they end in
# =====================================================================================================================
HOSE_WALL = {0.016: 0.002, 0.012: 0.0015, 0.010: 0.0015, 0.008: 0.00125}
PRESSURE_ROLES = ("refuel", "fill", "interconnect", "feed", "return")


class Line:
    def __init__(self, C: Ctx, lid: str):
        d = C.lines[lid]
        self.lid, self.d = lid, d
        self.pid = d["part"]
        self.D = float(d["diameter"])
        self.ro = 0.5 * self.D
        self.ri = self.ro - HOSE_WALL.get(round(self.D, 3), 0.0015)
        self.role = d.get("role", "")
        self.mirror = bool(d.get("mirror"))
        self.path = np.asarray(d["path"], float)
        self.bend_r = 2.5 * self.D
        self.nipple = {}             # end ("start"/"end") -> (r_out, r_in, length beyond the path end, overlap)
        self.extra = []              # meshes fused to the line (pickups, weights)
        self.contacts = []
        self.parent = None
        self.hose = self.role in PRESSURE_ROLES

    def centerline(self) -> np.ndarray:
        if getattr(self, "_Q", None) is None or self._Q_src is not self.path:
            Q, arcs = fillet(self.path, self.bend_r)
            for _rng, R in arcs:
                if R < 1.2 * self.ro:
                    raise ValueError(f"{self.lid}: bend radius {R * 1000:.1f} mm < 1.2 x hose radius")
            self._Q, self._Q_src = Q, self.path
        return self._Q

    def mesh(self) -> G.Mesh:
        Q = self.centerline()
        d0, d1 = G.unit(Q[1] - Q[0]), G.unit(Q[-1] - Q[-2])
        st = []
        body_s, body_e = 0.0, 0.0
        if "start" in self.nipple:
            r_o, r_i, ext, ov = self.nipple["start"]
            st += [(Q[0] - ext * d0, r_o, r_i), (Q[0] + ov * d0, r_o, r_i), (Q[0] + ov * d0, self.ro, r_i),
                   (Q[0] + 2 * ov * d0, self.ro, r_i), (Q[0] + 2 * ov * d0, self.ro, self.ri)]
            body_s = 2 * ov
        body = trim(Q, body_s, 0.0)
        tail = []
        if "end" in self.nipple:
            r_o, r_i, ext, ov = self.nipple["end"]
            body = trim(body, 0.0, 2 * ov)
            tail = [(Q[-1] - 2 * ov * d1, self.ro, r_i), (Q[-1] - ov * d1, self.ro, r_i), (Q[-1] - ov * d1, r_o, r_i),
                    (Q[-1] + ext * d1, r_o, r_i)]
            st += [(q, self.ro, self.ri) for q in body] + [(Q[-1] - 2 * ov * d1, self.ro, self.ri)] + tail
        else:
            st += [(q, self.ro, self.ri) for q in body]
        m = tube_sweep(st)
        if self.extra:
            m = union([m] + self.extra)
        return finish(m)

    def envelope(self, grow: float = BORE_CLR) -> G.Mesh:
        """Solid of the line grown by ``grow`` (socket / bore cutter for the parts it passes through); the 20-gon is
        circumscribed so that its flats clear the hose's own vertices by ``grow``."""
        return rod(self.centerline(), (self.ro + grow) / math.cos(math.pi / 20))

    def length(self) -> float:
        return float(seg_len(self.centerline()).sum())


# =====================================================================================================================
# bottom fittings (vulcanised flanges bolted through the deck / well roof from the dry side)
# =====================================================================================================================
class BottomFitting:
    def __init__(self, C: Ctx, cell: Cell, name: str, xy, line_d: float, struct_part: str, struct_top: float,
                 struct_t: float, inlet: bool, head_ok=lambda x, y: True):
        self.C, self.cell, self.name = C, cell, name
        self.xy = np.asarray(xy, float)
        self.r_line = 0.5 * line_d
        self.bore_r = self.r_line + 0.003                   # deck / roof bore (chassis.penetration_cuts margin)
        self.struct_part, self.struct_top, self.struct_t = struct_part, struct_top, struct_t
        self.z0 = struct_top + 0.0001
        self.z1 = self.z0 + FIT_T
        self.inlet = inlet
        self.bolts = self._choose(head_ok)
        self.poly = self._outline()

    def allowed(self) -> Polygon:
        """Flange footprint allowed: inside the bladder (slice of its envelope just above the flange, which already
        excludes the keep-outs), inboard of the liner floor return (20 mm band along the skin-side wall)."""
        if getattr(self, "_allowed", None) is None:
            cs = self.cell.outer_solid().to_manifold().slice(self.z1 + 0.0005).to_polygons()
            geo = Polygon()
            for P in cs:
                if len(P) >= 3:
                    geo = geo.symmetric_difference(Polygon(np.asarray(P)).buffer(0))
            reg = SG.largest(geo.buffer(-0.0008, join_style=2))
            y_lim = reg.bounds[3] - 0.0200
            reg = reg.intersection(sbox(-10, -y_lim, 10, y_lim))
            self._allowed = SG.largest(reg)
        return self._allowed

    def _choose(self, head_ok):
        d = BOLT_FIT * 1e-3
        allowed = self.allowed()
        best = None
        for r_b in (self.bore_r + 2.5 * d + 0.0005, self.bore_r + 2.5 * d + 0.0025, self.bore_r + 2.5 * d + 0.0045):
            cands = []
            for th in np.radians(np.arange(0.0, 360.0, 7.5)):
                q = self.xy + r_b * np.array([math.cos(th), math.sin(th)])
                if not allowed.contains(Point(q).buffer(FIT_RIM, 16)):
                    continue
                if not head_ok(q[0], q[1]):
                    continue
                cands.append(q)
            import itertools
            for n_b in (4, 3):
                for combo in itertools.combinations(range(len(cands)), n_b):
                    Q = [cands[i] for i in combo]
                    dmin = min(np.linalg.norm(a - b) for a, b in itertools.combinations(Q, 2))
                    if dmin < 3 * d + 0.002:
                        continue
                    # spread: the bolts must surround the bore (largest angular gap < 180 deg)
                    ang = sorted(math.atan2(q[1] - self.xy[1], q[0] - self.xy[0]) for q in Q)
                    gaps = [b - a for a, b in zip(ang, ang[1:])] + [ang[0] + 2 * math.pi - ang[-1]]
                    score = (n_b, -max(gaps), dmin)
                    if max(gaps) < math.radians(200) and (best is None or score > best[0]):
                        best = (score, Q)
                if best is not None and best[0][0] == n_b:
                    break
            if best is not None:
                break
        if best is None:
            raise ValueError(f"bottom fitting {self.name}: no bolt pattern fits")
        return [np.asarray(q) for q in best[1]]

    def _outline(self) -> Polygon:
        geo = [Point(q).buffer(FIT_RIM, 24) for q in self.bolts] + [Point(self.xy).buffer(self.bore_r + FIT_SEAL, 32)]
        hull = unary_union(geo).convex_hull
        poly = hull.intersection(self.allowed())
        return SG.largest(poly)

    def solid(self) -> G.Mesh:
        return prism_z(self.poly, self.z0, self.z1)

    def bore_cutters(self):
        cx, cy = self.xy
        out = [G.cylinder(self.r_line + BORE_CLR, (cx, cy, self.z0 - 0.002), (cx, cy, self.z1 + 0.002), n=32)]
        if not self.inlet:            # outlet / sump: countersunk so the floor drains into the line
            out.append(G.cylinder(self.r_line + BORE_CLR, (cx, cy, self.cell.floor + T_BLAD),
                                  (cx, cy, self.z1 + 0.0002), n=32, r1=self.r_line + CSK))
        return out

    def line_end_z(self) -> float:
        """Top of the line in the fitting: the bladder inner floor (outlet flush with the floor)."""
        return self.cell.floor + T_BLAD


# =====================================================================================================================
# producer
# =====================================================================================================================
class Fuel:
    def __init__(self, reg: Registry, spec: dict):
        self.reg, self.spec = reg, spec
        C = self.C = Ctx(reg, spec)
        L = C.L
        n0, n1 = L["part_numbers"]["fuel"]
        self.n0, self.n1 = n0, n1
        self.cell = {}
        for k in CELL_KEYS:
            c = (AftCell if k == "aft_cell" else Cell)(C, k)
            c.keepouts = cell_keepouts(C, c)
            c.pid = C.fs[k]["parts"]["cell"]
            c.liner = C.fs[k]["parts"]["liner"]
            self.cell[k] = c
        self.lines = {lid: Line(C, lid) for lid in C.lines}
        self.ids = {}

    # ------------------------------------------------------------------ helpers
    def pid(self, key: str, side: str = "C") -> str:
        n = N[key]
        if not self.n0 <= n <= self.n1:
            raise ValueError(f"fuel part number {n} outside {self.n0}..{self.n1}")
        return part_number("fuel", n, side=side)

    def add(self, pid, name, name_tr, material, process, mesh_fn, *, thickness=None, purchased=False, vendor="",
            mass_kg=None, parent=None, step=STEP, explode=(0.0, 0.0, -0.1), contacts=(), notes="", side=None,
            color="fuel") -> Part:
        side = side or ("R" if pid.endswith("-R") else ("L" if pid.endswith("-L") else "C"))
        p = Part(id=pid, name=name, name_tr=name_tr, group="fuel", material=material, process=process,
                 mesh_fn=mesh_fn, thickness=thickness, purchased=purchased, vendor=vendor, mass_kg=mass_kg,
                 side=side, parent=parent, step=step, explode=tuple(float(v) for v in explode),
                 contacts=tuple(c for c in contacts if c), notes=notes, color=color)
        return self.reg.add(p)

    # ------------------------------------------------------------------ main
    def build(self):
        self.plan_lines()
        self.plan_fittings()
        self.register_cells()
        self.register_lines()
        self.register_valves()
        self.register_collector()
        self.register_probes()
        self.register_seals()
        self.register_refuel()
        self.register_firewall()
        self.register_pump()
        self.register_skin_fittings()
        self.register_support()
        self.register_contents()

    # ------------------------------------------------------------------ planning: line ends at their fittings
    def eq_box(self, eid: str):
        b = self.C.eq[eid]["box"]
        return np.asarray(b[0], float), np.asarray(b[1], float)

    def plan_lines(self):
        C, Lx = self.C, self.lines
        fwd, aft = self.cell["forward_cell"], self.cell["aft_cell"]
        # collector (header tank) against the aft wall of the feed cell, on its floor
        self.col_x1 = aft.xb - T_BLAD - 0.0005        # clear of the inner ends of the wall-port bosses
        self.col_x0 = self.col_x1 - COLLECTOR["depth"]
        self.col_z0 = aft.floor + T_BLAD + 0.00005
        self.col_z1 = COLLECTOR["top"]
        col_floor_in = self.col_z0 + COLLECTOR["wall"]
        # refuel: coupling face (layout start) -> coupling top -> forward-cell bottom fitting
        r = Lx["FL-REFUEL"]
        p0 = r.path[0]
        self.coupling_face = p0.copy()
        top = p0 + np.array([0.0, 0.0, COUPLING["L"]])
        r.path = np.array([top, [p0[0], p0[1], fwd.floor + T_BLAD]])
        # bottom interconnect: outlet / inlet flush with the bladder floors
        x = Lx["FL-XFER-LO"]
        P = x.path.copy()
        P[0, 2] = fwd.floor + T_BLAD
        P[-1, 2] = aft.floor + T_BLAD
        x.path = P
        # forward sump drain: from the bottom fitting to the quick drain in the lower centre skin
        d = Lx["FL-DRAIN-F"]
        P = d.path.copy()
        P[0, 2] = fwd.floor + T_BLAD
        d.path = P
        # feed: flop pickup at the collector floor -> FS-GEAR -> pump inlet face
        f1 = Lx["FL-FEED-1"]
        pb0, pb1 = self.eq_box("EQ-FUELPUMP")
        P = f1.path.copy()
        P[0] = [self.col_x0 + 0.016, P[0, 1], col_floor_in + f1.ro + 0.0010]
        P[-1, 0] = pb0[0] - 0.0001
        f1.path = P
        ex = np.array([1.0, 0.0, 0.0])                                             # weighted flop pickup (brass)
        f1.extra.append(G.cylinder(f1.ro + 0.0005, P[0] - 0.004 * ex, P[0] + 0.006 * ex, n=20))
        # collector drain (sump of the feed cell) -> quick drain in the aft hatch
        da = Lx["FL-DRAIN-A"]
        P = da.path.copy()
        P[0] = [self.col_x0 + 0.012, P[0, 1], col_floor_in + da.ro + 0.0003]
        da.path = P
        # pump -> shut-off valve -> firewall -> engine hose nipple (propulsion hose slides over it)
        sv0, sv1 = self.eq_box("EQ-SHUTOFF")
        f2 = Lx["FL-FEED-2"]
        P = f2.path.copy()
        P[0, 0] = pb1[0] + 0.0001
        P[-1, 0] = sv0[0] - 0.0001
        ax = np.array([0.012, 0.0, 0.0])            # straight hose ends square to the pump and valve ports
        f2.path = np.vstack([P[:1], P[:1] + ax, P[1:-1], P[-1:] - ax, P[-1:]])
        f3 = Lx["FL-FEED-3"]
        P = f3.path.copy()
        P[0, 0] = sv1[0] + 0.0001
        f3.path = P
        f3.nipple["end"] = (0.004, 0.0025, 0.008, 0.008)
        rt = Lx["FL-RETURN"]
        rt.nipple["start"] = (0.004, 0.0025, 0.008, 0.008)
        # gascolator drain: from the bowl nipple under the tray to the quick drain in the aft hatch
        g = Lx["FL-DRAIN"]
        P = g.path.copy()
        self.pump_drain = np.array([P[0, 0], P[0, 1], pb0[2]])
        g.path = P
        for ln in Lx.values():
            if ln.role in ("vent", "drain"):
                ln.hose = False

    # ------------------------------------------------------------------ planning: fittings on the bladders
    def plan_fittings(self):
        C, Lx = self.C, self.lines
        fwd, aft = self.cell["forward_cell"], self.cell["aft_cell"]
        deck = C.mem["M-FWDDECK"]["part"]
        roof = C.mem["M-WELLROOF"]["part"]
        dlo, dhi = C.member_box("M-FWDDECK")
        rlo, rhi = C.member_box("M-WELLROOF")
        wlo, whi = C.member_box("M-WELLWALL-FWD")
        x_wall = 0.5 * (wlo[0] + whi[0])
        hw = 0.5 * (whi[0] - wlo[0]) + 0.5 * FC.ISO4762[BOLT_FIT][0] + 0.001
        xlo = Lx["FL-XFER-LO"]
        y_x, x_up = float(xlo.path[-1, 1]), float(xlo.path[-1, 0])
        clr_line = xlo.ro + 0.5 * FC.ISO4762[BOLT_FIT][0] + 0.001

        def head_ok_roof(xq, yq):            # well-roof underside: clear of the well wall and the interconnect
            if abs(xq - x_wall) < hw:
                return False
            if xq < x_up + 0.002 and abs(abs(yq) - abs(y_x)) < clr_line:
                return False
            return True
        bf = self.bottom = {}
        rf = Lx["FL-REFUEL"]
        bf["refuel"] = BottomFitting(C, fwd, "refuel", rf.path[0][:2], rf.D, deck, float(dhi[2]), float(dhi[2] - dlo[2]),
                                     inlet=True)
        for s, side in ((1, "R"), (-1, "L")):
            q = xlo.path[0][:2] * np.array([1.0, s])
            bf["xlo_f_" + side] = BottomFitting(C, fwd, "xlo_f_" + side, q, xlo.D, deck, float(dhi[2]),
                                                float(dhi[2] - dlo[2]), inlet=False)
            q = xlo.path[-1][:2] * np.array([1.0, s])
            bf["xlo_a_" + side] = BottomFitting(C, aft, "xlo_a_" + side, q, xlo.D, roof, float(rhi[2]),
                                                float(rhi[2] - rlo[2]), inlet=True, head_ok=head_ok_roof)
        dr = Lx["FL-DRAIN-F"]
        bf["drain_f"] = BottomFitting(C, fwd, "drain_f", dr.path[0][:2], dr.D, deck, float(dhi[2]),
                                      float(dhi[2] - dlo[2]), inlet=False)
        for b in bf.values():
            b.cell.fittings.append(b.solid())
        # wall ports where lines leave / enter a bladder through its end walls
        self.ports = []                      # (cell, line, point, direction)
        frames = {("forward_cell", +1): "FS-MS", ("saddle_cell", -1): "FS-MS", ("saddle_cell", +1): "FS-RS",
                  ("aft_cell", -1): "FS-RS", ("aft_cell", +1): "FS-GEAR"}
        for ln in Lx.values():
            for key, c in self.cell.items():
                for side in (-1, +1):
                    f = (lambda p, c=c: p[0] - c.x_aft(p[1])) if side > 0 else (lambda p, c=c: p[0] - c.x_fwd(p[1]))
                    pt, i = plane_cross(ln.path, f)
                    if pt is None:
                        continue
                    if not (c.floor - 0.001 < pt[2] < c.top + 0.001):
                        continue
                    # the crossing must be inside the bay section (not a line passing below / beside the cell)
                    poly = C.sec(pt[0], c.inset + GAP)
                    if not poly.contains(Point(pt[1], pt[2])):
                        continue
                    d = G.unit(ln.path[i + 1] - ln.path[i])
                    sid = frames.get((key, side))
                    x_face, k = C.face(sid, -side)
                    k_w = c.kb if side > 0 else c.ka
                    # distance along the line from the wall to the frame face
                    gx = (x_face + abs(pt[1]) * k) - pt[0]
                    gap = abs(gx / d[0]) if abs(d[0]) > 1e-6 else 0.05
                    r_b = ln.ro + BOSS_RIM
                    tilt = r_b * k_w
                    out_len = max(0.0015, min(BOSS_OUT, gap - GROM_FL - 0.0015 - tilt))
                    dir_out = d if side * d[0] > 0 else -d
                    cosd = abs(float(dir_out[0])) * math.cos(math.atan(k_w))
                    reach = r_b * math.tan(math.acos(min(1.0, cosd))) + 0.001
                    boss = G.cylinder(r_b, pt - (T_BLAD + 0.0003 + reach) * dir_out,
                                      pt + (out_len + reach) * dir_out, n=32)
                    x_w = c.x_aft(0.0) if side > 0 else c.x_fwd(0.0)
                    t_in, t_out = (-(T_BLAD + 0.0003), out_len) if side > 0 else (-out_len, T_BLAD + 0.0003)
                    slab = chevron_prism(sbox(-1.0, c.floor + 0.00005, 1.0, c.top - 0.00005), x_w, k_w, t_in, t_out)
                    boss = inter(boss, slab)
                    c.fittings.append(boss)
                    self.ports.append((c, ln, pt, dir_out))
                    if ln.mirror:
                        q = pt * np.array([1.0, -1.0, 1.0])
                        c.fittings.append(boss.mirrored_y())
                        self.ports.append((c, ln, q, dir_out * np.array([1.0, -1.0, 1.0])))
        # probe top fittings (one capacitive probe per cell, under the fuel-bay access panel)
        self.probes = {}
        for key, (x_p, y_p) in PROBE_AT.items():
            c = self.cell[key]
            x_p = c.xa + x_p * (c.xb - c.xa) if x_p < 1.0 else x_p
            top_pt, nrm = self.top_point(c, x_p, y_p)
            self.probes[key] = (top_pt, nrm)
            boss = disc(top_pt, nrm, PROBE["boss_r"], -0.003, 0.0015)
            c.fittings.append(boss)

    def top_point(self, c: Cell, x: float, y: float):
        """Point on the bladder outer top surface above (x, y) and the outward normal there (section slope)."""
        poly = self.C.sec(x, c.inset + GAP)

        def ztop(yy):
            seg = poly.intersection(sbox(yy - 1e-6, -1, yy + 1e-6, 1))
            return min(float(seg.bounds[3]), c.top)
        z = ztop(y)
        dz = (ztop(y + 0.002) - ztop(y - 0.002)) / 0.004
        n = G.unit(np.array([0.0, -dz, 1.0]))
        return np.array([x, y, z]), n

    # ------------------------------------------------------------------ cells
    def cell_cutters(self, c: Cell) -> list:
        """Bores of the lines through the cell's fittings and ports, local to each fitting."""
        out = []
        for b in self.bottom.values():
            if b.cell is c:
                out += b.bore_cutters()
        for cc, ln, pt, _d in self.ports:
            if cc is c:
                env = ln.envelope() if pt[1] * ln.path[0, 1] >= 0 or not ln.mirror else \
                    ln.envelope().mirrored_y()
                out.append(inter(env, box3(pt - 0.03, pt + 0.03)))
        for key, (pt, n) in self.probes.items():
            if self.cell[key] is c:
                out.append(disc(pt, n, PROBE["r"] + BORE_CLR, -0.01, 0.01, n=24))
        return out

    def register_cells(self):
        C = self.C
        names = {"forward_cell": ("forward fuel cell (bladder)", "ön yakıt hücresi (esnek)"),
                 "saddle_cell": ("saddle fuel cell (bladder)", "eyer yakıt hücresi (esnek)"),
                 "aft_cell": ("aft fuel cell / feed cell (bladder)", "arka yakıt hücresi / besleme hücresi (esnek)")}
        for key, c in self.cell.items():
            area = c.outer_solid().area()
            v_fit = sum(m.volume() for m in c.fittings)
            mass = round(BLAD_AREAL * area + FIT_RHO * v_fit, 3)
            cut = self.cell_cutters(c)

            def mf(c=c, cut=cut):
                return finish(diff(c.shell(), cut))
            n_bf = sum(1 for b in self.bottom.values() if b.cell is c)
            n_pt = sum(1 for cc, *_r in self.ports if cc is c)
            self.add(c.pid, names[key][0], names[key][1], P_BUY, P_BUY, mf, purchased=True,
                     vendor="Aero Tec Laboratories (ATL) custom ultralight UAV fuel bladder, 0.25 mm coated fabric, "
                            "vulcanised 6061-T6 fitting flanges / port bosses, made to the bay drawing",
                     mass_kg=mass, parent=c.liner, contacts=(c.liner,), explode=(0.0, 0.0, 0.32), color="fuel",
                     notes=f"envelope = fuel bay band of layout.chassis.fuel_supports ({key}) inside the liner "
                           f"(OML inset {c.inset * 1000:.0f} mm + {GAP * 1000:.1f} mm), floor z {c.floor:.4f}, top z "
                           f"{c.top:.4f}; keep-outs: chine longeron, bridle-fitting nuts, trunnion dome nutplates; "
                           f"{n_bf} bottom fitting flange(s) bolted through the floor, {n_pt} wall port(s), one "
                           f"level-probe top fitting; hung from the liner hook-and-loop tabs and 2 restraint straps "
                           f"(chassis); mass = {BLAD_AREAL} kg/m2 x {area:.3f} m2 wall (components.yaml basis) + "
                           f"fitting flanges {FIT_RHO * v_fit:.3f} kg (estimate)")
            self.ids[key] = c.pid
        # bottom-fitting screws: M3 A2-70 from the dry side through the deck / well roof, tapped into the flange
        for name, b in self.bottom.items():
            for k, q in enumerate(b.bolts, 1):
                J.bolt_through(self.reg, f"{b.cell.pid}-{name.upper()}-B{k}", BOLT_FIT,
                               (q[0], q[1], b.struct_top - 0.5 * b.struct_t), (0.0, 0.0, 1.0), [b.struct_part],
                               owner=b.cell.pid, nut="tapped", tapped_part=b.cell.pid,
                               tapped_depth=FIT_T - 0.0005, step=STEP, washer_head=False,
                               notes="bladder fitting flange screw from the dry side (thread locker, O-ring face seal "
                                     "round the bore); helical insert in the 6061-T6 flange")

    # ------------------------------------------------------------------ lines
    def line_contacts(self, ln: Line) -> list:
        out = []
        for c, l2, *_r in self.ports:
            if l2 is ln:
                out.append(c.pid)
        for b in self.bottom.values():
            if ln.lid in ("FL-XFER-LO",) and b.name.startswith("xlo") or \
                    ln.lid == "FL-REFUEL" and b.name == "refuel" or ln.lid == "FL-DRAIN-F" and b.name == "drain_f":
                out.append(b.cell.pid)
        return list(dict.fromkeys(out))

    def register_lines(self):
        for ln in self.lines.values():
            m = ln.mesh()
            rho = HOSE_RHO if ln.hose else PA12_RHO
            mass = round(m.volume() * rho, 3)
            kind = ("PTFE-lined stainless-braided hose assembly, AN crimped ends, made to length" if ln.hose else
                    "PA12 fuel / vent tube assembly (SAE J2260 class), swaged ends, made to length")
            cont = self.line_contacts(ln)
            parent = cont[0] if cont else None
            side = "R" if ln.mirror else "C"
            pid = ln.pid + ("-R" if ln.mirror else "")
            ln.part_id = pid

            def mf(m=m):
                return m
            self.add(pid, ln.d["name"], ln.d["name_tr"], P_BUY, P_BUY, mf, purchased=True, vendor=kind,
                     mass_kg=mass, parent=parent, contacts=cont, side=side,
                     explode=(0.0, 0.0, -0.18) if ln.role != "vent" else (0.0, 0.0, 0.2),
                     notes=f"layout.fuel_lines {ln.lid}: d {ln.D * 1000:.0f} mm, length {ln.length():.3f} m, bends "
                           f">= {ln.bend_r * 1000:.0f} mm or 90-degree hose ends; mass = wall volume x "
                           f"{rho:.0f} kg/m3 (estimate); {ln.d.get('text', '')[:200]}")
        for ln in self.lines.values():
            if ln.mirror:
                pr = self.reg.parts[ln.part_id]
                idm = {c.pid: c.pid for c in self.cell.values()}
                self.reg.add(mirror_part(pr, ln.pid + "-L", id_map=idm))

    # ------------------------------------------------------------------ fuel contents (consumable)
    def register_contents(self):
        C = self.C
        vols = {k: c.inner_solid().volume() for k, c in self.cell.items()}
        v_fuel = C.fuel_kg / C.rho
        f = v_fuel / sum(vols.values())
        if f > 1.0:
            raise ValueError("design fuel does not fit the bladders")
        self.fill_fraction = f
        names = {"forward_cell": ("fuel, forward cell", "yakıt, ön hücre", "fuel_f"),
                 "saddle_cell": ("fuel, saddle cell", "yakıt, eyer hücre", "fuel_s"),
                 "aft_cell": ("fuel, aft cell", "yakıt, arka hücre", "fuel_a")}
        for key, c in self.cell.items():
            inner = c.inner_solid()
            target = f * vols[key]
            lo_b, hi_b = inner.bounds()
            z_lo, z_hi = float(lo_b[2]), float(hi_b[2])

            def vol_at(z, inner=inner, lo_b=lo_b, hi_b=hi_b):
                cut = box3((lo_b[0] - 0.01, lo_b[1] - 0.01, lo_b[2] - 0.01), (hi_b[0] + 0.01, hi_b[1] + 0.01, z))
                r = G.intersection(inner, cut)
                return 0.0 if r is None else r.volume()
            a, b = z_lo, z_hi
            for _ in range(40):
                mid = 0.5 * (a + b)
                if vol_at(mid) < target:
                    a = mid
                else:
                    b = mid
            z_fill = 0.5 * (a + b)
            cut = box3((lo_b[0] - 0.01, lo_b[1] - 0.01, lo_b[2] - 0.01), (hi_b[0] + 0.01, hi_b[1] + 0.01, z_fill))
            mesh = finish(inter(inner, cut))
            mass = round(mesh.volume() * C.rho, 4)
            nm = names[key]
            self.add(self.pid(nm[2]), nm[0], nm[1], "fuel_mogas_oil_50_1", "consumable", (lambda m=mesh: m),
                     mass_kg=mass, parent=c.pid, contacts=(c.pid,), step=STEP_LOAD, explode=(0.0, 0.0, 0.32),
                     notes=f"design fuel load mass.fuel_kg {C.fuel_kg} kg distributed in proportion to the bladder "
                           f"volumes (fill fraction {f:.3f}, level z {z_fill:.4f}); density "
                           f"engine.fuel.density_kg_per_m3 {C.rho:.0f}; scaled by the loading case fuel_fraction "
                           f"(analysis/mass.py)")

    # ------------------------------------------------------------------ valves
    def inline_valve(self, ln: Line) -> G.Mesh:
        """Flapper check valve body on the line end: socket for the hose end, flow bore, outlet into the cell."""
        Q = ln.centerline()
        p, d = Q[-1], G.unit(Q[-1] - Q[-2])
        r = ln.ro + CV["r_add"]
        body = G.cylinder(r, p - CV["sock"] * d, p + (CV["L"] - CV["sock"]) * d, n=32)
        sock = G.cylinder(ln.ro + BORE_CLR, p - (CV["sock"] + 0.001) * d, p + 0.0002 * d, n=32)
        flow = G.cylinder(ln.ri, p - 0.0005 * d, p + (CV["L"] - CV["sock"] + 0.001) * d, n=24)
        return finish(diff(body, [sock, flow]))

    def inner_top(self, c: Cell, x: float, y: float, r: float) -> float:
        """Lowest point of the bladder's inner top surface over a disc of radius r at (x, y)."""
        man = c.inner_solid().to_manifold()
        zs = []
        for ang in np.linspace(0.0, 2 * math.pi, 12, endpoint=False):
            for rr in (0.0, r):
                q = np.array([x + rr * math.cos(ang), y + rr * math.sin(ang)])
                h = G.ray_hits(man, (q[0], q[1], 0.5), (q[0], q[1], -0.5))
                zs.append(0.5 - float(h[0]))
        return min(zs)

    def register_valves(self):
        Lx, cs = self.lines, self.cell
        # inline flapper check valves at the interconnect ends inside the receiving cell
        for key, lid, cell, tag in (("cv_fs", "FL-FILL-S", "saddle_cell", "CV-FS"),
                                    ("cv_sa", "FL-XFER-SA", "aft_cell", "CV-SA")):
            ln = Lx[lid]
            v = next(v for v in ln.d["valves"] if v["id"] == tag)
            mesh = self.inline_valve(ln)
            self.add(self.pid(key), f"flapper check valve {tag}", f"klapeli çek valf {tag}", P_BUY, P_BUY,
                     (lambda m=mesh: m), purchased=True, vendor="Andair-type in-line flapper check valve, AN-6 "
                     "(components.yaml andair_check_valve class), fuel-resistant FKM seal", mass_kg=CV["mass"],
                     parent=ln.part_id, contacts=(ln.part_id,), explode=(0.0, 0.0, 0.32),
                     notes=f"{v['type']} at layout point {v['point']} (layout.fuel_lines {lid}); mass estimate")
        # flush flapper valves on the aft-cell bottom fittings of the interconnects
        ln = Lx["FL-XFER-LO"]
        b = self.bottom["xlo_a_R"]
        c = b.cell
        q = np.array([b.xy[0], b.xy[1], b.z1])
        body = G.cylinder(CVB["r"], q + np.array([0, 0, 0.00005]), q + np.array([0, 0, CVB["h"]]), n=32)
        bore = G.cylinder(ln.ri, q - np.array([0, 0, 0.001]), q + np.array([0, 0, CVB["h"] - 0.001]), n=24)
        wins = [G.cylinder(0.0008, q + np.array([0, 0, 0.0015]) - 0.02 * np.array([math.cos(a_), math.sin(a_), 0]),
                           q + np.array([0, 0, 0.0015]) + 0.02 * np.array([math.cos(a_), math.sin(a_), 0]), n=12)
                for a_ in (0.0, math.pi / 2)]
        mesh = finish(diff(body, [bore] + wins))
        pid = self.pid("cv_fa", "R")
        self.add(pid, "flush flapper check valve CV-FA, starboard", "gömme klapeli çek valf CV-FA, sağ", P_BUY,
                 P_BUY, (lambda m=mesh: m), purchased=True, vendor="flush flapper check valve on the bladder fitting "
                 "(cage with side windows, FKM flap), bladder vendor kit", mass_kg=CVB["mass"], parent=c.pid,
                 contacts=(c.pid,), explode=(0.0, 0.0, 0.32),
                 notes="layout.fuel_lines FL-XFER-LO valve CV-FA (forward -> aft only): seated on the aft-cell bottom "
                       "fitting flange, 4 x M2.5 screws in the flange (vendor kit, not modelled); mass estimate")
        self.reg.add(mirror_part(self.reg.parts[pid], pid[:-1] + "L", id_map={c.pid: c.pid}))
        # float / roll-over vent valves in the cell tops (vent lines start / end in them)
        fvs = {("FL-VENT-1", "FV-F"): ("fv_f", "forward_cell"), ("FL-VENT-1", "FV-S"): ("fv_s", "saddle_cell"),
               ("FL-VENT-2", "FV-S2"): ("fv_s2", "saddle_cell"), ("FL-VENT-2", "FV-A"): ("fv_a", "aft_cell"),
               ("FL-VENT", "FV-A2"): ("fv_a2", "aft_cell")}
        for (lid, tag), (key, ck) in fvs.items():
            ln, c = Lx[lid], cs[ck]
            v = next(v for v in ln.d["valves"] if v["id"] == tag)
            pt = np.asarray(v["point"], float)
            z_top = self.inner_top(c, pt[0], pt[1], FV["r"]) - 0.00005
            body = G.cylinder(FV["r"], (pt[0], pt[1], z_top - FV["h"]), (pt[0], pt[1], z_top), n=32)
            local = inter(ln.envelope(), box3(pt - 0.02, pt + 0.02))
            mesh = finish(diff(body, [local]))
            self.add(self.pid(key), f"float / roll-over vent valve {tag}", f"şamandıralı / devrilme havalandırma "
                     f"valfi {tag}", P_BUY, P_BUY, (lambda m=mesh: m), purchased=True,
                     vendor="motorsport-type float / roll-over vent valve (components.yaml rollover_vent_valve)",
                     mass_kg=FV["mass"], parent=c.pid, contacts=(c.pid, ln.part_id), explode=(0.0, 0.0, 0.32),
                     notes=f"{v['type']} (layout point {v['point']}), clipped to the bladder top fitting; closes the "
                           "cell top in manoeuvres and inverted flight; mass components.yaml (estimate)")

    # ------------------------------------------------------------------ collector (header tank)
    def collector_mesh(self) -> G.Mesh:
        x0, x1, z0, z1 = self.col_x0, self.col_x1, self.col_z0, self.col_z1
        y0, y1 = COLLECTOR["y"]
        w = COLLECTOR["wall"]
        outer = box3((x0, y0, z0), (x1, y1, z1))
        inner = box3((x0 + w, y0 + w, z0 + w), (x1 - w, y1 - w, z1 - w))
        cut = [inner]
        zi = z0 + w + 0.001 + COLLECTOR["inlet_r"]
        for yy in (y0 + 0.020, 0.5 * (y0 + y1), y1 - 0.020):       # flapper inlets 1 mm above the floor
            cut.append(G.cylinder(COLLECTOR["inlet_r"], (x0 - 0.001, yy, zi), (x0 + w + 0.001, yy, zi), n=24))
        for yy in (y0 + 0.030, y1 - 0.030):                          # air vents in the lid
            cut.append(G.cylinder(0.003, (0.5 * (x0 + x1), yy, z1 - w - 0.001), (0.5 * (x0 + x1), yy, z1 + 0.001),
                                  n=16))
        for lid in ("FL-FEED-1", "FL-DRAIN-A", "FL-RETURN"):
            ln = self.lines[lid]
            cut.append(inter(ln.envelope(), box3((x0 - 0.01, y0 - 0.01, z0 - 0.01), (x1 + 0.01, y1 + 0.01, z1 + 0.01))))
        return finish(diff(outer, cut))

    def register_collector(self):
        c = self.cell["aft_cell"]
        mesh = self.collector_mesh()
        lns = [self.lines[k].part_id for k in ("FL-FEED-1", "FL-DRAIN-A", "FL-RETURN")]
        self.add(self.pid("collector"), "collector / header tank (feed cell)", "toplayıcı / başlık deposu",
                 MAT_PA12, P_SLS, (lambda m=mesh: m), thickness=COLLECTOR["wall"], parent=c.pid,
                 contacts=(c.pid,) + tuple(lns), explode=(0.0, 0.0, 0.32),
                 notes="SLS PA12 (fuel resistant) box 2 mm, about 0.3 L inside, on the floor of the feed cell against "
                       "its aft wall: three flapper inlets 1 mm above the bladder floor (FKM flaps, vendor), two lid "
                       "vents; flop pickup of FL-FEED-1, sump drain of FL-DRAIN-A and the return FL-RETURN inside; "
                       "installed through the bladder's aft fitting before the cell goes into the bay; low-level "
                       "switch in the lid (open item: switch type)")

    # ------------------------------------------------------------------ level probes
    def register_probes(self):
        keys = {"forward_cell": "probe_f", "saddle_cell": "probe_s", "aft_cell": "probe_a"}
        for ck, key in keys.items():
            c = self.cell[ck]
            pt, n = self.probes[ck]
            z_end = c.floor + T_BLAD + 0.004
            L = (pt[2] - z_end) / max(abs(n[2]), 1e-6)
            tube = G.cylinder(PROBE["r"], pt + 0.0016 * n, pt - L * n, n=24)
            head = G.cylinder(PROBE["head_r"], pt + 0.00155 * n, pt + (0.00155 + PROBE["head_h"]) * n, n=32)
            mesh = finish(union([tube, head]))
            self.add(self.pid(key), f"capacitive fuel level probe, {ck.replace('_', ' ')}",
                     f"kapasitif yakıt seviye algılayıcısı, {c.key.split('_')[0]}", P_BUY, P_BUY,
                     (lambda m=mesh: m), purchased=True, vendor="Gill Sensors miniature capacitive fuel level sensor, "
                     f"custom length {L * 1000:.0f} mm (components.yaml gill_micro_level_sensor)",
                     mass_kg=PROBE["mass"], parent=c.pid, contacts=(c.pid,), explode=(0.0, 0.0, 0.40),
                     notes="installed along the top-wall normal through the bladder's top fitting, under the fuel-bay "
                           "access panel; tilt compensated in the fuel quantity computation; probe cable exit through "
                           "the access panel: open item (layout.systems.harness has no fuel-probe branch)")
            self.reg.parts[c.liner].add_hole(pt - 0.004 * n, pt + 0.004 * n, PROBE["boss_r"] + 0.0008)

    # ------------------------------------------------------------------ frame pass-through seals
    def grommet(self, sid: str, cid: str, mirror: bool, forbid) -> tuple:
        """Seal for one frame cut-out: plug filling the cut-out (0.2 mm clearance) and flanges on both web faces;
        ``forbid`` = {side: [(z_lo, z_hi), ...]} z-bands kept free beyond the forward (-1) / aft (+1) face (decks,
        roof, tray, spar caps that meet the frame there) and {0: [...]} free bands of the plug itself (members that
        enter the web). Returns (seal mesh, liner cutter)."""
        C = self.C
        y0, y1, z0, z1 = C.cutout(sid, cid, mirror)
        xf, k = C.face(sid, -1)
        xa, _ = C.face(sid, +1)
        w = xa - xf
        plug = Polygon([(y0 + GROM_CLR, z0 + GROM_CLR), (y1 - GROM_CLR, z0 + GROM_CLR), (y1 - GROM_CLR, z1 - GROM_CLR),
                        (y0 + GROM_CLR, z1 - GROM_CLR)])
        rim = sbox(y0 - GROM_RIM, z0 - GROM_RIM, y1 + GROM_RIM, z1 + GROM_RIM)
        parts = [chevron_prism(plug, xf, k, -GROM_FACE - GROM_FL, w + GROM_FACE + GROM_FL),
                 chevron_prism(rim, xf, k, -GROM_FACE - GROM_FL, -GROM_FACE),
                 chevron_prism(rim, xf, k, w + GROM_FACE, w + GROM_FACE + GROM_FL)]
        g = union(parts)
        cut = []
        if k:                                   # chevron frames: the kink of the web faces stands proud by <= 0.5 mm
            kink = sbox(-0.0025, -10, 0.0025, 10).difference(sbox(y0 - 0.0001, z0 - 0.0001, y1 + 0.0001, z1 + 0.0001))
            if not kink.is_empty:
                for t0, t1 in ((-GROM_FACE - 0.0007, 0.0), (w, w + GROM_FACE + 0.0007)):
                    for pk in getattr(kink, "geoms", [kink]):
                        cut.append(chevron_prism(pk.intersection(sbox(-1, z0 - GROM_RIM - 0.001, 1, z1 + GROM_RIM + 0.001)),
                                                 xf, k, t0, t1))
        span = {-1: (-0.01, -0.00001), +1: (w + 0.00001, w + 0.01), 0: (-0.01, w + 0.01)}
        for side, bands in forbid.items():
            for zlo, zhi in bands:
                cut.append(chevron_prism(sbox(y0 - 0.02, zlo, y1 + 0.02, zhi), xf, k, *span[side]))
        if cut:
            g = diff(g, cut)
        liner_cut = chevron_prism(rim.buffer(0.0003, join_style=2), xf, k, -0.004, w + 0.004)
        return g, liner_cut

    def register_seals(self):
        C = self.C
        Lx = self.lines
        box_lo, box_hi = C.member_box("M-CTBOX")
        dlo, _ = C.member_box("M-FWDDECK")
        rlo, rhi = C.member_box("M-WELLROOF")
        pb0, _ = self.eq_box("EQ-FUELPUMP")
        m = 0.0005
        deck = {-1: [(float(dlo[2]) - m, 1.0)], 0: [(float(dlo[2]) - m, 1.0)]}
        roof_aft = {+1: [(float(rlo[2]) - m, 1.0)], 0: [(float(rlo[2]) - m, 1.0)]}
        caps = {-1: [(-1.0, float(box_hi[2]) + m)], +1: [(-1.0, float(box_hi[2]) + m)], 0: [(-1.0, float(box_hi[2]) + m)]}
        drn = {-1: [(-1.0, float(rhi[2]) + m)], +1: [(-1.0, float(pb0[2]) + m)]}
        sets = {
            "grom_ms": ("FS-MS", [("C-FUEL-MS", False, {}, ["FL-FILL-S"]), ("C-VENT-MS", False, {}, ["FL-VENT-1"]),
                                  ("C-FUEL-MS-LO", False, deck, ["FL-XFER-LO"]),
                                  ("C-FUEL-MS-LO", True, deck, ["FL-XFER-LO"])], ("forward_cell", "saddle_cell")),
            "grom_rs": ("FS-RS", [("C-FUEL-SAD", False, caps, ["FL-XFER-SA"]),
                                  ("C-VENT-RS", False, {}, ["FL-VENT-2"]),
                                  ("C-FUEL-RS-LO", False, roof_aft, ["FL-XFER-LO"]),
                                  ("C-FUEL-RS-LO", True, roof_aft, ["FL-XFER-LO"])], ("saddle_cell", "aft_cell")),
            "grom_gear": ("FS-GEAR", [("C-FUEL-FEED", False, {}, ["FL-FEED-1"]),
                                      ("C-FUEL-RET", False, {}, ["FL-RETURN"]),
                                      ("C-FUEL-VENT", False, {}, ["FL-VENT"]),
                                      ("C-FUEL-DRN", False, drn, ["FL-DRAIN-A"])], ("aft_cell",))}
        names = {"grom_ms": ("pass-through seal set, main-spar frame", "geçiş contası takımı, ana kiriş çerçevesi"),
                 "grom_rs": ("pass-through seal set, rear-spar frame", "geçiş contası takımı, arka kiriş çerçevesi"),
                 "grom_gear": ("pass-through seal set, FS-GEAR bulkhead", "geçiş contası takımı, FS-GEAR perdesi")}
        for key, (sid, items, bays) in sets.items():
            bodies, lines = [], []
            liners = [self.cell[b].liner for b in bays]
            for cid, mir, forbid, lids in items:
                g, lcut = self.grommet(sid, cid, mir, forbid)
                for lp in liners:                 # the bay liner is trimmed round the seal flange
                    self.reg.parts[lp].holes.append(lcut)
                y0, y1, z0, z1 = C.cutout(sid, cid, mir)
                x_c = C.st[sid]["x"] + abs(0.5 * (y0 + y1)) * math.tan(math.radians(float(C.st[sid].get("sweep_deg", 0))))
                cut = []
                for lid in lids:
                    ln = Lx[lid]
                    env = ln.envelope().mirrored_y() if (mir and ln.mirror) else ln.envelope()
                    cut.append(inter(env, box3((x_c - 0.03, y0 - 0.01, z0 - 0.01), (x_c + 0.03, y1 + 0.01, z1 + 0.01))))
                    pid_l = ln.pid + ("-L" if (mir and ln.mirror) else ("-R" if ln.mirror else ""))
                    lines.append(pid_l)
                bodies.append(finish(diff(g, cut)))
            mesh = G.merge(bodies)
            v = mesh.volume()
            frame = C.st[sid]["part"]
            self.add(self.pid(key), names[key][0], names[key][1], P_BUY, P_BUY, (lambda m_=mesh: m_), purchased=True,
                     vendor="moulded fluorosilicone (FVMQ) bulkhead seal, split for installation round the hose, "
                            "bonded with fuel-resistant sealant",
                     mass_kg=round(v * FLUORO_RHO, 4), parent=frame, contacts=(frame,) + tuple(dict.fromkeys(lines)),
                     explode=(0.0, 0.0, 0.25 if key != "grom_gear" else -0.1),
                     notes=f"{len(items)} seals filling the declared cut-outs of {sid} ("
                           + ", ".join(i[0] + ("-L" if i[1] else "") for i in items)
                           + f") with {GROM_FL * 1000:.1f} mm flanges on both web faces (trimmed where a deck, the "
                             "well roof, the tray or the spar caps meet the frame; the bay liner is trimmed round "
                             "the flange); sealed (vapour-tight) pass-through of the fuel lines; mass = volume x "
                             "1400 kg/m3 (estimate)")

    # ------------------------------------------------------------------ refuel coupling and its bracket
    def register_refuel(self):
        C = self.C
        deck = C.mem["M-FWDDECK"]["part"]
        dlo, dhi = C.member_box("M-FWDDECK")
        f = self.coupling_face
        x0, y0, zf = float(f[0]), float(f[1]), float(f[2])
        z_top = zf + COUPLING["L"]
        t = BRACKET_T
        zp1 = z_top - 0.012                    # bracket plate top (jam nut above it, hex flange below)
        zp0 = zp1 - t
        z_deck = float(dlo[2])
        half, wy, fl = 0.031, 0.036, 0.024     # legs at x0 -+ half (centre lines), bracket width, flange reach
        zc_f = z_deck - 0.00005 - 0.5 * t
        zc_p = zp0 + 0.5 * t
        Pc = np.array([[x0 - half - fl, 0, zc_f], [x0 - half, 0, zc_f], [x0 - half, 0, zc_p], [x0 + half, 0, zc_p],
                       [x0 + half, 0, zc_f], [x0 + half + fl, 0, zc_f]])
        Q, _arcs = fillet(Pc, 3.0 * t + 0.5 * t, n_arc=10)             # inside bend radius 3 t (6061-T6)
        from shapely.geometry import LineString
        prof = LineString(Q[:, [0, 2]]).buffer(0.5 * t, cap_style=2, join_style=2, mitre_limit=2.0)
        br = G.extrude(prof, wy, origin=(0.0, y0 - 0.5 * wy, 0.0), u=(1.0, 0.0, 0.0), v=(0.0, 0.0, 1.0))
        if br.bounds()[0][1] > y0:             # extrusion along -y: shift to centre
            br = br.translated((0.0, -wy, 0.0))
        lo, hi = br.bounds()
        br = br.translated((0.0, y0 - 0.5 * (lo[1] + hi[1]), 0.0))
        hole = G.cylinder(COUPLING["r"] + 0.0001, (x0, y0, zp0 - 0.002), (x0, y0, zp1 + 0.002), n=32)
        br_mesh = finish(diff(br, [hole]))
        bid = self.pid("coupling_bracket")
        self.add(bid, "refuel coupling bracket", "yakıt ikmal bağlantısı braketi", MAT_AL, P_SHEET,
                 (lambda m=br_mesh: m), thickness=t, parent=deck, contacts=(deck,), explode=(0.0, 0.0, -0.22),
                 notes="6061-T6 sheet 2 mm, U-bracket with outward flanges under the forward fuel deck, inside bend "
                       "radius 3 t (processes.sheet_metal_aluminium); 4 x M3 into blind potted inserts in the deck "
                       "(the fuel-side facesheet is not pierced); carries the dry-break coupling in front of the "
                       "P-REFUEL door; bonding stud for the refuelling earth lead (CS-LUAS.867(d))")
        for k, (xb, yb) in enumerate(((x0 - half - fl + 0.0075, y0 - 0.009), (x0 - half - fl + 0.0075, y0 + 0.009),
                                      (x0 + half + fl - 0.0075, y0 - 0.009), (x0 + half + fl - 0.0075, y0 + 0.009)), 1):
            J.bolt(self.reg, f"{bid}-B{k}", 3, (xb, yb, z_deck - 0.00005 - t), (0.0, 0.0, 1.0), [(bid, t)],
                   nut="insert", insert_part=deck, insert_depth=0.005, step=STEP, head="ISO 7380",
                   notes="blind potted insert M3 from the dry side of the forward fuel deck")
        # coupling: body through the plate, hex flange under it, jam nut on top
        body = G.cylinder(COUPLING["r"], (x0, y0, zf), (x0, y0, z_top), n=32)
        flange = G.cylinder(COUPLING["flange_r"], (x0, y0, zp0 - COUPLING["flange_h"]), (x0, y0, zp0 - 0.00005), n=6)
        nut = G.cylinder(COUPLING["flange_r"], (x0, y0, zp1 + 0.00005), (x0, y0, zp1 + COUPLING["nut_h"]), n=6)
        bore = G.cylinder(0.006, (x0, y0, zf - 0.001), (x0, y0, z_top + 0.001), n=24)
        cp = finish(diff(union([body, flange, nut]), [bore]))
        cid = self.pid("coupling")
        rl = self.lines["FL-REFUEL"].part_id
        self.add(cid, "dry-break refuel coupling (airframe half)", "kuru bağlantı yakıt ikmal kaplini (uçak tarafı)",
                 P_BUY, P_BUY, (lambda m=cp: m), purchased=True,
                 vendor="obp Motorsport OBP-DRYBC-6A dry-break coupling AN6, airframe half (components.yaml "
                        "obp_dry_break_an6)", mass_kg=COUPLING["mass"], parent=bid, contacts=(bid, rl),
                 explode=(0.0, 0.0, -0.3),
                 notes="face at the layout FL-REFUEL start point, recessed behind the P-REFUEL door; envelope "
                       "d 32 x 40 mm (estimate, datasheet dimensions not in components.yaml); refuel and defuel point")
        self.reg.parts[rl].parent = cid
        self.reg.parts[rl].contacts = tuple(dict.fromkeys(self.reg.parts[rl].contacts + (cid,)))

    # ------------------------------------------------------------------ firewall bulkhead block + shut-off valve
    def register_firewall(self):
        C = self.C
        from ..core.parts import layup_props
        st = C.st["FS3670"]
        fw = st["part"]
        x_f = float(st["x_faces"][0])
        x_shield = float(st["x_faces"][1]) - float(st.get("shield_t", 0.0004))
        t_cfrp = layup_props(self.spec, st["layup"])["thickness"]
        y0, y1, z0, z1 = C.cutout("FS3670", "C-FW-FUEL")
        sv0, sv1 = self.eq_box("EQ-SHUTOFF")
        t = SS_PLATE_T
        f3, rt = self.lines["FL-FEED-3"], self.lines["FL-RETURN"]
        # flange on the forward face (closes the cut-out), bosses through the sandwich and the air gap up to the shield
        e = 0.008                               # M3 insert: 2.5 D = 7.5 mm from the cut-out edge (+0.5)
        bolts = [(y0 - e, z0 - e), (y1 + e, z0 - e), (y0 - e, z1 + e), (y1 + e, z1 + e)]
        rim = 0.0065
        zl = sv0[2] - 0.00005 - t               # shelf underside
        fl = box3((x_f - 0.00005 - t, y0 - e - rim, zl), (x_f - 0.00005, y1 + e + rim, z1 + e + rim))
        bosses = []
        for ln, yy, zz in ((f3, f3.path[-1][1], f3.path[-1][2]), (rt, rt.path[0][1], rt.path[0][2])):
            bosses.append(G.cylinder(ln.ro + 0.0015, (x_f - t, yy, zz), (x_shield - 0.0002, yy, zz), n=32))
        shelf = box3((sv0[0] - 0.002, sv0[1] - 0.002, zl), (x_f - 0.00005 - t + OV, sv1[1] + 0.002, sv0[2] - 0.00005))
        cut = [inter(ln.envelope(), box3((x_f - 0.02, y0 - 0.01, z0 - 0.01), (x_shield + 0.01, y1 + 0.01, z1 + 0.01)))
               for ln in (f3, rt)]
        blk = finish(diff(union([fl, shelf] + bosses), cut))
        bid = self.pid("fw_block")
        self.add(bid, "firewall fuel bulkhead block with shut-off valve shelf",
                 "yangın perdesi yakıt geçiş bloğu ve kesme valfi rafı", MAT_SS, P_CNC, (lambda m=blk: m), thickness=t,
                 parent=fw, contacts=(fw, f3.part_id, rt.part_id), explode=(-0.1, 0.0, 0.0),
                 notes=f"AISI 304 machined: {t * 1000:.1f} mm flange closing C-FW-FUEL on the forward face of the "
                       "firewall sandwich, two bosses carrying the feed and return hoses (fire sleeves) through the "
                       "sandwich and the air gap to the stainless shield opening (fireproof pass-through, CS-VLA "
                       "1191), shelf under the shut-off valve; inside corners R 3 (processes.cnc_milling_metal); "
                       "4 x M3 into potted inserts of the firewall sandwich")
        for k, (yb, zb) in enumerate(bolts, 1):
            J.bolt(self.reg, f"{bid}-B{k}", 3, (x_f - 0.00005 - t, yb, zb), (1.0, 0.0, 0.0), [(bid, t)],
                   nut="insert", insert_part=fw, insert_depth=min(0.0055, t_cfrp - 0.0012), step=STEP,
                   head="ISO 7380", notes="potted insert M3 in the firewall sandwich (forward face)")
        # shut-off valve (layout EQ-SHUTOFF box) on the shelf, 2 x M3 from below into its base
        e_ = self.C.eq["EQ-SHUTOFF"]
        yv = 0.5 * (sv0[1] + sv1[1])
        zv = 0.5 * (f3.path[0][2] + self.lines["FL-FEED-2"].path[-1][2])
        vbase = box3(sv0, (sv1[0], sv1[1], sv0[2] + 0.006))
        vbody = G.cylinder(0.010, (sv0[0], yv, zv), (sv1[0], yv, zv), n=32)
        ped = box3((sv0[0] + 0.006, yv - 0.006, sv0[2] + 0.006 - OV), (sv1[0] - 0.006, yv + 0.006, zv - 0.008))
        act = box3((sv0[0] + 0.008, yv - 0.010, zv + 0.010 - 0.002), (sv1[0] - 0.008, yv + 0.010, sv1[2]))
        valve = finish(union([vbase, vbody, ped, act]))
        vid = e_["part"]
        f2 = self.lines["FL-FEED-2"]
        self.add(vid, e_["name"], e_["name_tr"], P_BUY, P_BUY, (lambda m=valve: m), purchased=True,
                 vendor="electrically actuated fuel shut-off ball valve, 12 V, AN-6 ports (estimate)",
                 mass_kg=float(e_["mass_kg"]), parent=bid, contacts=(bid, f2.part_id, f3.part_id),
                 explode=(-0.1, 0.0, 0.05),
                 notes=f"layout EQ-SHUTOFF box {e_['box']} on the forward face of the firewall (no valve on the engine "
                       "side, CS-VLA 995); commanded by the flight-termination / engine-kill logic; mass "
                       f"{e_['mass_kg']} kg (layout, estimate)")
        for k, (xb, yb) in enumerate(((sv0[0] + 0.008, sv0[1] + 0.0065), (sv1[0] - 0.008, sv1[1] - 0.0065)), 1):
            J.bolt(self.reg, f"{vid}-B{k}", 3, (xb, yb, zl), (0.0, 0.0, 1.0), [(bid, t + 0.00005)],
                   owner=vid, nut="tapped", tapped_part=vid, tapped_depth=0.0055, step=STEP,
                   notes="valve base screws from below the shelf")
        for ln in (f2, f3):
            self.reg.parts[ln.part_id].contacts = tuple(dict.fromkeys(self.reg.parts[ln.part_id].contacts + (vid,)))
        for ln in (f3, rt):
            self.reg.parts[ln.part_id].contacts = tuple(dict.fromkeys(self.reg.parts[ln.part_id].contacts + (bid,)))
        self.reg.parts[f3.part_id].parent = vid
        self.reg.parts[f2.part_id].parent = vid

    # ------------------------------------------------------------------ EFI pump / regulator / gascolator unit
    def register_pump(self):
        C = self.C
        e = C.eq["EQ-FUELPUMP"]
        lo, hi = self.eq_box("EQ-FUELPUMP")
        tray = C.trays["TR-AFTBAY"]["part"]
        f1, f2, dr = self.lines["FL-FEED-1"], self.lines["FL-FEED-2"], self.lines["FL-DRAIN"]
        base_t = 0.003
        zb = lo[2] + 0.00005
        base = box3((lo[0] + 0.001, lo[1], zb), (hi[0] - 0.001, hi[1], zb + base_t))
        p_in, p_out = f1.path[-1], f2.path[0]
        r_b = 0.017
        yb_, zb_ = 0.5 * (lo[1] + hi[1]) - 0.008, zb + base_t + r_b - 0.0001
        body = G.cylinder(r_b, (lo[0] + 0.016, yb_, zb_), (hi[0] - 0.016, yb_, zb_), n=32)
        bowl_c = np.array([dr.path[0][0], dr.path[0][1]])
        reg_box = box3((hi[0] - 0.036, yb_ - 0.012, zb_ + r_b - 0.0005), (hi[0] - 0.014, yb_ + 0.010, zb_ + r_b + 0.015))
        port_in = G.cylinder(0.0075, (lo[0], p_in[1], p_in[2]), (lo[0] + 0.017, p_in[1], p_in[2]), n=24)
        port_out = G.cylinder(0.0075, (hi[0] - 0.017, p_out[1], p_out[2]), (hi[0], p_out[1], p_out[2]), n=24)
        z_nip = dr.path[0][2] + 0.00005
        nip = G.cylinder(0.004, (bowl_c[0], bowl_c[1], z_nip), (bowl_c[0], bowl_c[1], zb + base_t + OV), n=24)
        # gascolator (separate purchased unit, fuel budget): bowl standing on the base plate, drain nipple through the
        # plate and the tray; the pump / regulator housing is relieved round it (0.1 mm)
        gas = finish(union([G.cylinder(GASCOLATOR["r"], (bowl_c[0], bowl_c[1], zb + base_t + 0.00005),
                                       (bowl_c[0], bowl_c[1], hi[2] - 0.004), n=32), nip]))
        relief = [G.cylinder(GASCOLATOR["r"] + 0.0001, (bowl_c[0], bowl_c[1], zb + base_t),
                             (bowl_c[0], bowl_c[1], hi[2]), n=32),
                  G.cylinder(0.004 + 0.0005, (bowl_c[0], bowl_c[1], zb - 0.001), (bowl_c[0], bowl_c[1], zb + base_t + OV),
                             n=24)]
        pump = finish(diff(union([base, body, reg_box, port_in, port_out]), relief))
        pid = e["part"]
        self.add(pid, e["name"], e["name_tr"], P_BUY, P_BUY, (lambda m=pump: m), purchased=True,
                 vendor="Limbach EFI fuel supply kit: positive-displacement 12 V pump, 2.5 bar regulator, filter "
                        "(components.yaml limbach_efi_supply_kit) on a common base plate with the gascolator",
                 mass_kg=float(e["mass_kg"]), parent=tray, contacts=(tray, f1.part_id, f2.part_id),
                 explode=(0.0, 0.0, -0.25),
                 notes=f"layout EQ-FUELPUMP box {e['box']} on TR-AFTBAY; inlet from FL-FEED-1 through the gascolator, "
                       "outlet to FL-FEED-2; filter reached through P-AFTHATCH; mass "
                       f"{e['mass_kg']} kg = engine.installed_items_kg.fuel_pump_regulator_filter (booked in the "
                       "propulsion budget item engine_group_installed)")
        gid = self.pid("gascolator")
        self.add(gid, "gascolator (water trap and strainer) before the pump", "gaskolatör (su tutucu ve süzgeç)",
                 P_BUY, P_BUY, (lambda m=gas: m), purchased=True,
                 vendor="Andair GAS375 gascolator (3/8 in lines), PTFE-coated 70 micron washable screen "
                        "(components.yaml andair_gas375)",
                 mass_kg=GASCOLATOR["mass"], parent=pid, contacts=(pid, dr.part_id), explode=(0.0, 0.0, -0.25),
                 notes="on the base plate of the EQ-FUELPUMP unit (screwed to the plate by the unit supplier, "
                       "counterbored from below), in the line between FL-FEED-1 and the pump; bowl drain nipple "
                       "through the plate and the tray to FL-DRAIN and the flush drain valve in P-AFTHATCH (daily water "
                       "check, CS-LUAS drain + strainer rule); mass 0.152 kg manufacturer (excluding fittings), item "
                       "of the fuel-system mass basis (components.yaml fuel_system, gascolator 0.15 kg)")
        iv = J.measure_stack(self.reg, [tray], (lo[0] + 0.009, lo[1] + 0.009, float(lo[2])), (0.0, 0.0, -1.0), 0.003)
        t_tray = float(iv[0][2] - iv[0][1])
        y_b1 = p_in[1] + 0.0075 + 0.007        # clear of the inlet boss above (installation access from the top)
        for k, (xb, yb) in enumerate(((lo[0] + 0.009, y_b1), (hi[0] - 0.009, lo[1] + 0.009),
                                      (lo[0] + 0.009, hi[1] - 0.009), (hi[0] - 0.009, hi[1] - 0.009)), 1):
            J.bolt(self.reg, f"{pid}-B{k}", 4, (xb, yb, zb + base_t), (0.0, 0.0, -1.0),
                   [(pid, base_t + 0.00005), (tray, t_tray)], washer_head=True, step=STEP,
                   notes="pump base to the aft-bay tray, ISO 7040 nyloc nut + washer under the tray")
        for ln in (f1, f2):
            self.reg.parts[ln.part_id].contacts = tuple(dict.fromkeys(self.reg.parts[ln.part_id].contacts + (pid,)))
        self.reg.parts[dr.part_id].contacts = tuple(dict.fromkeys(self.reg.parts[dr.part_id].contacts + (gid,)))
        self.reg.parts[f2.part_id].parent = pid
        self.reg.parts[dr.part_id].parent = gid
        # pass-through holes in the tray for the gascolator drain, the aft-cell drain and the vent descent
        trayp = self.reg.parts[tray]
        z_t = float(lo[2])
        trayp.add_hole((bowl_c[0], bowl_c[1], z_t - 0.006), (bowl_c[0], bowl_c[1], z_t + 0.001), 0.004 + 0.0015)
        for ln in (self.lines["FL-DRAIN-A"], self.lines["FL-VENT"]):
            env = rod(ln.centerline(), ln.ro + 0.0015)
            cut = G.intersection(env, box3((lo[0] - 0.2, -0.2, z_t - 0.004), (hi[0] + 0.3, 0.2, z_t + 0.001)))
            if cut is not None:
                trayp.holes.append(cut)

    # ------------------------------------------------------------------ skin-end fittings (drains, flush vent)
    def skin_fitting(self, ln: Line, r_body: float, nut_r: float, nut_h: float, hex_: bool, skin: str | None = None):
        """Flush skin-end fitting at the end of ``ln``: body flush with the OML, socket for the line, jam nut / flange on
        the inner skin face. With the shell built (``skin`` = panel part id) the faces are measured on the panel and a
        contoured washer fills the space between the curved inner skin face and the flat nut (0.05 mm above the
        skin); otherwise the OML and the shell_secondary laminate thickness give the faces."""
        C = self.C
        Q = ln.centerline()
        p = Q[-1]
        d = G.unit(Q[-1] - Q[-2])
        z_oml = C.z_bot(p[0], p[1])
        z_in = z_oml + C.skin_t
        pad = None
        if skin is not None:
            man = self.reg.parts[skin].mesh.to_manifold()
            outs, ins = [], []
            for rr in (r_body + 0.001, 0.5 * (r_body + nut_r), nut_r + 0.0005):
                for th in np.linspace(0.0, 2 * math.pi, 24, endpoint=False):
                    q = (p[0] + rr * math.cos(th), p[1] + rr * math.sin(th))
                    h = G.ray_hits(man, (q[0], q[1], z_oml - 0.02), (q[0], q[1], z_oml + 0.03)) + z_oml - 0.02
                    if len(h) >= 2:
                        outs.append(h[0])
                        ins.append(h[1])
            if not ins:
                raise ValueError(f"{ln.lid}: skin panel {skin} not found under the fitting")
            z_oml = float(np.median(outs[:24]))
            z_in_lo, z_in = float(min(ins)), float(max(ins))
            # the undrilled panel lifted 0.05 mm trims the washer and the nut (the body passes the drilled hole)
            skin_cut = inter(self.reg.parts[skin].mesh.translated((0.0, 0.0, 0.00005)),
                             box3((p[0] - 0.03, p[1] - 0.03, z_oml - 0.03), (p[0] + 0.03, p[1] + 0.03, z_in + 0.01)))
            pad = diff(G.tube(nut_r, r_body - OV, (p[0], p[1], z_in_lo - 0.0005), (p[0], p[1], z_in + 0.00005 + OV),
                              n=32), [skin_cut])
        body = G.cylinder(r_body, (p[0], p[1], z_oml), (p[0], p[1], max(p[2], z_in + nut_h + 0.001)), n=32)
        sock = G.cylinder(ln.ro + 0.0015, p - 0.0002 * d, p - 0.009 * d, n=32)
        nut = G.cylinder(nut_r, (p[0], p[1], z_in + 0.00005), (p[0], p[1], z_in + nut_h), n=6 if hex_ else 32)
        if pad is not None:
            nut = diff(nut, [skin_cut])
        bore = rod(trim(Q, 0.0, 0.0), ln.ro + BORE_CLR)
        local = inter(bore, box3(p - 0.03, p + 0.03))
        flow = G.cylinder(min(ln.ri, r_body - 0.0015), (p[0], p[1], z_oml - 0.001), p + 0.001 * d, n=24)
        return finish(diff(union([body, sock, nut, pad]), [local, flow])), (p, z_oml, z_in)

    def register_skin_fittings(self):
        C = self.C
        items = (("drain_f", "FL-DRAIN-F", "P-CENTRE-LOWER", "flush quick-drain valve, forward-cell sump",
                  "hızlı boşaltma valfi, ön hücre sump", DRAIN),
                 ("drain_a", "FL-DRAIN-A", "P-AFTHATCH", "flush quick-drain valve, collector sump",
                  "hızlı boşaltma valfi, toplayıcı sump", DRAIN),
                 ("drain_g", "FL-DRAIN", "P-AFTHATCH", "flush quick-drain valve, gascolator",
                  "hızlı boşaltma valfi, filtre", DRAIN),
                 ("vent_out", "FL-VENT", "P-AFT-LOWER", "flush fuel vent outlet with flame arrestor",
                  "alev tutuculu gömme yakıt havalandırma çıkışı", VENT_OUT))
        for key, lid, panel, name, name_tr, D in items:
            ln = self.lines[lid]
            pid = self.pid(key)
            sk = C.panels[panel]["part"]
            skin = sk if sk in self.reg.parts else None
            if key == "vent_out":
                mesh, (p, z_oml, z_in) = self.skin_fitting(ln, D["r"], D["flange_r"], D["flange_h"], False, skin)
            else:
                mesh, (p, z_oml, z_in) = self.skin_fitting(ln, D["r"], D["nut_r"], D["nut_h"], True, skin)
            cont = [ln.part_id]
            if sk in self.reg.parts:            # shell built: drill the skin, the nut / flange bears on it
                # hole = 24-gon (Part.holes) circumscribing the body + 0.05 mm
                self.reg.parts[sk].add_hole((p[0], p[1], z_oml - 0.002), (p[0], p[1], z_in + 0.002),
                                            (D["r"] + 0.00005) / math.cos(math.pi / 24))
                cont.append(sk)
            self.add(pid, name, name_tr, P_BUY, P_BUY, (lambda m=mesh: m), purchased=True,
                     vendor=("flush quick-drain valve (push-to-drain, FKM seal), AN thread" if key != "vent_out" else
                             "flush fuel vent outlet with stainless flame-arrestor screen, NACA-type recess in the skin"),
                     mass_kg=D["mass"], parent=ln.part_id, contacts=tuple(cont), explode=(0.0, 0.0, -0.3),
                     notes=f"end fitting of layout.fuel_lines {lid} in {panel} ({sk}): flush with the OML, jam nut / "
                           "flange on the inner skin face (skin hole drilled when the shell module is built); daily "
                           "water check without opening a hatch; mass estimate")
            self.reg.parts[ln.part_id].contacts = tuple(dict.fromkeys(self.reg.parts[ln.part_id].contacts + (pid,)))

    # ------------------------------------------------------------------ aft-bay line support
    def register_support(self):
        """Support bracket on TR-AFTBAY at about mid-span of FL-FEED-2 (pump -> shut-off valve, 0.38 m) and FL-RETURN
        (FS-GEAR seal -> firewall block, 0.56 m): 6061-T6 L-bracket (web across the lines) with two split cushion
        bushes that hold the hoses laterally and let them slide axially (thermal / pressure length change)."""
        C = self.C
        tray = C.trays["TR-AFTBAY"]["part"]
        f2, rt = self.lines["FL-FEED-2"], self.lines["FL-RETURN"]
        t, r_in = BRACKET_T, 3.0 * BRACKET_T            # inside bend radius 3 t (processes.sheet_metal_aluminium)
        S = SUPPORT
        z_t = float(C.L["rules"]["boxes"]["equipment_bay_aft"]["z"][0])      # tray top = bay floor
        x_w0 = S["x_web"]
        x_w1 = x_w0 + t
        x_mid = x_w0 + 0.5 * t
        z_f0 = z_t + 0.00005
        z_f1 = z_f0 + t
        x_b = x_w0 - r_in - 0.5 * FC.ISO7089[4][1] - 0.001          # washer clear of the bend
        x_f0 = x_b - S["edge"]
        y0, y1 = S["y_bolts"][0] - S["edge"], S["y_bolts"][1] + S["edge"]
        # L profile (x-z centre line, filleted) extruded over the base width, web plate with the bush holes above it
        from shapely.geometry import LineString
        Pc = np.array([[x_f0, 0.0, z_f0 + 0.5 * t], [x_mid, 0.0, z_f0 + 0.5 * t], [x_mid, 0.0, z_f1 + r_in + 0.004]])
        Q, _arcs = fillet(Pc, r_in + 0.5 * t, n_arc=10)
        prof = LineString(Q[:, [0, 2]]).buffer(0.5 * t, cap_style=2, join_style=2, mitre_limit=2.0)
        base = G.extrude(prof, y1 - y0, origin=(0.0, y0, 0.0), u=(1.0, 0.0, 0.0), v=(0.0, 0.0, 1.0))
        lo_, hi_ = base.bounds()
        base = base.translated((0.0, 0.5 * (y0 + y1) - 0.5 * (lo_[1] + hi_[1]), 0.0))
        bushes = []
        for ln in (f2, rt):
            p, _i = plane_cross(ln.centerline(), lambda q: q[0] - x_mid)
            if p is None:
                raise ValueError(f"{ln.lid} does not cross the support web at x {x_mid:.4f}")
            bushes.append((ln, p, ln.ro + S["bush_wall"]))
        z_web0 = z_f1 + r_in - OV
        outline = [Polygon([(y0, z_web0), (y1, z_web0), (y1, z_web0 + 0.001), (y0, z_web0 + 0.001)])]
        outline += [Point(float(p[1]), float(p[2])).buffer(r_p + GROM_RIM + S["web_rim"], 32) for _ln, p, r_p in bushes]
        web_poly = unary_union(outline).convex_hull.buffer(-0.004, join_style=1).buffer(0.004, join_style=1)
        web = prism_x(web_poly, x_w0, x_w1)
        holes = [G.cylinder(r_p + GROM_CLR, (x_w0 - 0.002, p[1], p[2]), (x_w1 + 0.002, p[1], p[2]), n=40)
                 for _ln, p, r_p in bushes]
        br_mesh = finish(diff(union([base, web]), holes))
        bid = self.pid("support")
        self.add(bid, "aft-bay fuel line support bracket", "arka bölme yakıt hattı destek braketi", MAT_AL, P_SHEET,
                 (lambda m=br_mesh: m), thickness=t, parent=tray, contacts=(tray,), explode=(0.0, 0.0, -0.2),
                 notes=f"6061-T6 sheet {t * 1000:.0f} mm L-bracket, inside bend radius 3 t (processes."
                       "sheet_metal_aluminium), web across FL-FEED-2 / FL-RETURN at about mid-span between the pump / "
                       "FS-GEAR and the firewall (unsupported hose spans <= 0.3 m); 2 x M4 A2-70 through the tray "
                       "web between its lightening holes, ISO 7040 nyloc + washer under the tray; split cushion "
                       "bushes YK250-FU-616 in the two web holes")
        iv = J.measure_stack(self.reg, [tray], (x_b, S["y_bolts"][0], z_t), (0.0, 0.0, -1.0), 0.003)
        t_tray = float(iv[0][2] - iv[0][1])
        for k, yb in enumerate(S["y_bolts"], 1):
            J.bolt(self.reg, f"{bid}-B{k}", 4, (x_b, yb, z_f1), (0.0, 0.0, -1.0),
                   [(bid, t + 0.00005), (tray, t_tray)], washer_head=True, step=STEP,
                   notes="line support bracket to the aft-bay tray, ISO 7040 nyloc nut + washer under the tray")
        # split cushion bushes: plug in the web hole, flanges on both web faces, bore = hose + 0.05 mm
        parts, lines = [], []
        for ln, p, r_p in bushes:
            x_a, x_b2 = x_w0 - GROM_FACE - GROM_FL, x_w1 + GROM_FACE + GROM_FL
            plug = G.cylinder(r_p, (x_a, p[1], p[2]), (x_b2, p[1], p[2]), n=40)
            fl0 = G.cylinder(r_p + GROM_RIM, (x_a, p[1], p[2]), (x_w0 - GROM_FACE, p[1], p[2]), n=40)
            fl1 = G.cylinder(r_p + GROM_RIM, (x_w1 + GROM_FACE, p[1], p[2]), (x_b2, p[1], p[2]), n=40)
            bore = inter(ln.envelope(), box3(p - 0.02, p + 0.02))
            parts.append(finish(diff(union([plug, fl0, fl1]), [bore])))
            lines.append(ln.part_id)
        bush = G.merge(parts)
        self.add(self.pid("clamps"), "split cushion bushes, aft-bay line support",
                 "yarık yastıklı burçlar, arka bölme hat desteği", P_BUY, P_BUY, (lambda m=bush: m), purchased=True,
                 vendor="moulded fluorosilicone (FVMQ, 60 Shore A) split grommet bush, fuel resistant (estimate)",
                 mass_kg=round(bush.volume() * FLUORO_RHO, 4), parent=bid, contacts=(bid,) + tuple(lines),
                 explode=(0.0, 0.0, -0.2),
                 notes=f"2 bushes (FL-FEED-2, FL-RETURN), wall {S['bush_wall'] * 1000:.1f} mm, flanges "
                       f"{GROM_FL * 1000:.1f} mm on both web faces; split along the axis, fitted round the hose and "
                       "pressed into the web hole; lateral support and chafe protection, the hose slides axially; "
                       "mass = volume x 1400 kg/m3 (estimate)")
        for ln in (f2, rt):
            self.reg.parts[ln.part_id].contacts = tuple(dict.fromkeys(self.reg.parts[ln.part_id].contacts +
                                                                      (self.pid("clamps"),)))


# =====================================================================================================================
# entry point
# =====================================================================================================================
def register(reg: Registry, spec: dict) -> None:
    if any(p.group == "fuel" for p in reg.parts.values()):
        reg.note("fuel: already registered, second call ignored")
        return
    Fuel(reg, spec).build()
