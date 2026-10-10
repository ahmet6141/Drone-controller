"""Tail producer (YK-250 HANCER): twin canted fins with rudders, fixed stabilator root stubs, all-moving stabilators on
Ti stub spindles, the ventral fin with the replaceable bumper skid, and the rudder / stabilator drives.

What it builds (starboard parts in +y, mirrored to port; part numbers from ``spec.layout.part_numbers.tail``: TL 250-299
structure, FC 300-349 controls; the ids fixed by the layout are kept: fin TL-250, stub TL-252, ventral TL-254,
stabilator TL-256, fin-tip cap TL-258, bumper skid TL-260, spindle TL-301, rudder FC-300, DA 30 FC-302, DA 26 FC-303):

* fin: closed sandwich skin (``layups.tail_skin``) trimmed at the body OML, front spar (0.241 c) and rear (hinge) spar
  (0.625 c) CFRP C-channels, contoured root rib on the OML, servo rib, box ribs, aft closure rib under the rudder root,
  tip rib, GFRP antenna tip cap with its base plate, removable servo hatch cover on the inboard face, and two machined
  7075 spar root fittings whose lugs enter the chassis clevises F-FIN-FRONT (FS3480) and F-FW-CORNER (firewall);
* rudder: round-nose CFRP skin (cove on the fin), C-channel spar at the hinge, ROHACELL core, three hinges (clevis on
  the rear spar, tongue on the rudder spar, M4 shoulder-bolt pins), surface horn on potted inserts, DA 26 actuator on the
  servo rib (case above the output shaft), servo arm and pushrod (spatial four-bar, ``rudder_R`` + coupled joints);
* stabilator stub: sandwich skin trimmed at the OML, 7075 spindle sleeve (root flange on the node outboard cheek with
  4 x M6 12.9 = layout F-SPINDLE-NODE B8..B11, outboard 61805 bearing seat at its tip), front spar, root and tip ribs;
* spindle: Ti-6Al-4V, machined from bar, OD 25 (neck OD 21 through the node cheek hole), inboard 61805-ZZ bearing in the
  node boss with two DIN 472 circlips, end washer + M6 bolt, outboard bearing in the sleeve, collar outboard of it;
* stabilator panel: sandwich skin (closed tip), CFRP spar, root rib, socket rib, mid rib, 7075 root fitting (socket
  29 x 2 with the cross-bolt bosses), Ti end plug; cross bolt M5 from the lower skin;
* stabilator drive: DA 30 in a 7075 cradle on the firewall forward face (potted inserts), 15 mm arm, 8 mm pushrod through
  the firewall window C-FW-PUSHROD, 37.5 mm horn clamped on the spindle spline (planar 2.5:1 four-bar, coupled joints);
* ventral fin: sandwich skin trimmed at the OML, CFRP spar, contoured root rib, three 7075 root lugs in F-VENTRAL-1..3
  (M5 12.9, double shear), 4130 strap with a replaceable PA12 skid shoe at the tip under the propeller disc.

Interfaces read (never another module's geometry): ``spec.tail.surfaces`` (OML), ``spec.fuselage`` (body OML for the
root trim), ``spec.layout`` (part numbers, chassis fittings F-FIN-FRONT / F-FW-CORNER / F-SPINDLE-NODE / F-VENTRAL-*,
mechanisms joints, systems actuators EQ-STABACT / ACT-RUDDER, stations FS3670), ``spec.structures.sizing.tail``
(spindle, socket, spar caps), ``spec.tail.surfaces.*.controls`` (linkage arms, actuator data), ``spec.layups`` /
``spec.processes`` / ``spec.materials``.

Module-private detailing constants and the documented deviations from the layout are listed in
``ucav250/docs/detail/tail.md`` (bolt positions shifted inside the chassis clevises for lug edge distance, DA 26 case
above its output shaft per the datasheet shaft offset, spindle wall 1.5 mm = CNC minimum, ventral lug bolts M5).
"""
from __future__ import annotations

import math

import numpy as np
from shapely.geometry import LineString, MultiPolygon, Point, Polygon
from shapely.geometry import box as sbox
from shapely.ops import unary_union

from ..core import geom as G
from ..core.parts import Joint, Part, Registry, layup_props, mirror_part, part_number
from . import actuation as A
from . import joints as J
from . import oml as O
from . import structgen as SG

# =====================================================================================================================
# module-private detailing constants (docs/detail/tail.md)
# =====================================================================================================================
RING_N = 160                # points per lofted section ring
TRIM = 0.001                # tail lofts end 1 mm above the body OML (sealant fillet line; avoids a tangent cut)
BOND = 0.0001               # bond line between bonded parts (contact <= 0.3 mm, no volume overlap)
OV = 2e-4                   # boolean overlap of fused features
T_SKIN_CS = 0.0006          # control-surface skins: 3 plies PW (solid laminate, prepreg min thickness)
T_WEB = 0.0012              # spar webs: 6 plies PW (+-45)
T_CAP = 0.0017              # spar caps: 12 plies UD (structures.sizing.tail.*_spar_cap_plies)
W_CAP = 0.025               # spar cap width (structures.sizing.tail.*_spar_cap_width_m)
T_GFRP = 0.002              # fin-tip cap: 8 plies GFRP 7781 (RF window)
FIN_FRONT_XC = 0.241        # layout F-FIN-FRONT fin_chord_fraction
FIN_REAR_XC = 0.58          # rear (hinge) spar: aft of the DA 26 case, 0.02 c behind the layout's 0.56 c root lug
STUB_FRONT_XC = 0.25
RUDDER_GAP_END = 0.006      # spanwise gap rudder ends / fixed fin (rudder top swings 3.9 mm toward the tip cap)
COVE_GAP = 0.002            # radial cove gap round the rudder nose
HINGE_S = (0.18, 0.45, 0.68)   # rudder hinge stations: fin span from the root reference section (m)
HINGE = A.HingeSpec(pin_d=0.003, lug_t=0.004, lug_r=0.0065, gap=0.0005, base_t=0.003, base_w=0.026, base_h=0.018,
                    bolt_d_mm=3.0, pin_clear=0.00002)
RUDDER_SPAR_E = 0.011       # rudder spar web front face aft of the hinge axis (clevis lug end r 6.5 + 4.5 mm)
SPINDLE_R = 0.01247         # spindle OD 25 (polygon-safe radius inside the 25 H7 bores)
SPINDLE_NECK_R = 0.0105     # neck OD 21 through the node cheek hole (>= 5 mm moving-part clearance, layout.clearances)
SPINDLE_ID = 0.022          # bore 22 (wall 1.5 = processes.cnc_milling_metal.min_thickness; structures 1.2)
SPINDLE_ID_NECK = 0.018    # bore 18 from the inboard end plug to past the neck (neck wall 1.5)
BRG = (0.025, 0.037, 0.007)  # 61805-ZZ: d, D, B
BRG_RO = 0.01847            # outer race radius inside a 37 H7 bore modelled with 48 facets
BRG_RI = 0.01253            # inner race bore
WASHER_R = 0.0134           # moving parts inside the node boss stay >= 5 mm off the 37 mm bore
DEG = math.pi / 180.0


def unit(v) -> np.ndarray:
    v = np.asarray(v, float)
    return v / np.linalg.norm(v)


def box3(lo, hi) -> G.Mesh:
    lo, hi = np.asarray(lo, float), np.asarray(hi, float)
    return G.box(hi - lo, 0.5 * (lo + hi))


def obox(center, axes, half) -> G.Mesh:
    """Oriented box: ``axes`` rows = unit local axes, ``half`` = half extents."""
    return G.box(2 * np.asarray(half, float), np.asarray(center, float), R=np.asarray(axes, float).T)


def ring(r_in: float, r_out: float, p0, p1, n: int = 48) -> G.Mesh:
    return G.tube(r_out, r_in, p0, p1, n=n)


def cyl(r: float, p0, p1, n: int = 48) -> G.Mesh:
    return G.cylinder(r, p0, p1, n=n)


def U(ms) -> G.Mesh:
    ms = [m for m in ms if m is not None]
    return ms[0] if len(ms) == 1 else G.union(ms)


def D(a: G.Mesh, cut) -> G.Mesh:
    cut = [c for c in cut if c is not None]
    return a if not cut else G.difference(a, cut)


def I(a: G.Mesh, b: G.Mesh) -> G.Mesh:
    r = G.intersection(a, b)
    if r is None:
        raise ValueError("empty intersection")
    return r


def largest_piece(m: G.Mesh) -> G.Mesh:
    parts = m.to_manifold().decompose()
    if len(parts) <= 1:
        return m
    return G.Mesh.from_manifold(max(parts, key=lambda p: p.volume()))


def pieces_above(m: G.Mesh, vmin: float = 2e-8) -> G.Mesh:
    """Drop loose slivers left by a trim (kept pieces must be real material, > 0.02 cm^3)."""
    parts = [p for p in m.to_manifold().decompose() if p.volume() > vmin]
    if not parts:
        raise ValueError("nothing left")
    if len(parts) == 1:
        return G.Mesh.from_manifold(parts[0])
    import manifold3d as m3
    return G.Mesh.from_manifold(m3.Manifold.batch_boolean(parts, m3.OpType.Add))


def polys(g) -> list:
    if isinstance(g, Polygon):
        return [g] if not g.is_empty else []
    return [p for p in getattr(g, "geoms", []) if isinstance(p, Polygon) and p.area > 1e-10]


def extrude_cs(poly, t: float, origin, u, v) -> G.Mesh:
    """Prism from a shapely (Multi)Polygon drawn in the (u, v) plane at ``origin``, extruded along u x v by ``t``
    (manifold3d cross-section triangulator: robust with holes)."""
    import manifold3d as m3
    from shapely.geometry.polygon import orient
    contours = []
    for p in polys(poly):
        p = orient(p, 1.0)
        contours.append(np.asarray(p.exterior.coords)[:-1].astype(np.float64))
        contours += [np.asarray(h.coords)[:-1].astype(np.float64) for h in p.interiors]
    cs = m3.CrossSection(contours, m3.FillRule.Positive)
    m = G.Mesh.from_manifold(m3.Manifold.extrude(cs, float(t)))
    u, v = np.asarray(u, float), np.asarray(v, float)
    w = np.cross(u, v)
    V = np.asarray(origin, float) + m.V[:, :1] * u + m.V[:, 1:2] * v + m.V[:, 2:3] * w
    return G.fix_orientation(G.Mesh(V, m.F))


# =====================================================================================================================
# lifting-surface helper (exact on the ruled OML loft of oml.LiftingSurface)
# =====================================================================================================================
class Surf:
    """A tail surface from ``spec.tail.surfaces.<key>.sections``. ``eta`` is the LiftingSurface span coordinate; every
    tail surface has a constant span direction ``w`` (untwisted, straight leading edge), so the section planes are
    parallel and a point at span coordinate eta lies at ``(eta - eta_ref) * w`` from the reference plane."""

    def __init__(self, sections, name: str):
        self.s = O.LiftingSurface(sections, n_chord=90, name=name)
        self.name = name
        e = self.s.span_coords()
        self.e0, self.e1 = float(e[0]), float(e[-1])
        o, c, u, w = self.s.frame_at(self.e0)
        self.c, self.u, self.w = unit(c), unit(u), unit(w)
        self._poly = {}

    def clamp(self, eta):
        return min(max(eta, self.e0), self.e1)

    def frame(self, eta):
        e = self.clamp(eta)
        o, c, u, w = self.s.frame_at(e)
        return o + (eta - e) * self.w, self.c, self.u

    def chord(self, eta) -> float:
        return self.s.chord_at(self.clamp(eta))

    def poly(self, eta, inset: float = 0.0) -> Polygon:
        """OML section (s, t) at eta (clamped to the surface), optionally inset (mitred)."""
        e = round(self.clamp(eta), 7)
        key = (e, round(inset, 7))
        if key not in self._poly:
            p, _fr = SG.section2d(self.s, e, n=220)
            if inset > 0:
                p = SG.largest(p.buffer(-inset, join_style=2))
            self._poly[key] = p
        return self._poly[key]

    def p3(self, eta, s, t=0.0) -> np.ndarray:
        o, c, u = self.frame(eta)
        return o + s * c + t * u

    def xc_pt(self, eta, xc, t=0.0) -> np.ndarray:
        return self.p3(eta, xc * self.chord(eta), t)

    def to2d(self, eta_ref, P) -> np.ndarray:
        o, c, u = self.frame(eta_ref)
        P = np.atleast_2d(np.asarray(P, float)) - o
        return np.column_stack([P @ c, P @ u])

    def poly_in(self, eta, eta_ref, inset=0.0) -> Polygon:
        """Section polygon at eta expressed in the frame of eta_ref (parallel planes)."""
        p = self.poly(eta, inset)
        o, c, u = self.frame(eta)
        P2 = np.asarray(p.exterior.coords)
        P3 = o + P2[:, :1] * c + P2[:, 1:2] * u
        return Polygon(self.to2d(eta_ref, P3)).buffer(0)

    def half_t(self, eta, xc, inset=0.0) -> tuple[float, float]:
        """(upper t, lower t) of the section at chord fraction xc."""
        p = self.poly(eta, inset)
        x = xc * self.chord(eta)
        g = LineString([(x, -1.0), (x, 1.0)]).intersection(p)
        ys = np.asarray(g.coords)[:, 1] if hasattr(g, "coords") else np.concatenate(
            [np.asarray(q.coords)[:, 1] for q in g.geoms])
        return float(ys.max()), float(ys.min())

    def stations(self, eta0, eta1, step=None):
        """Loft stations: both ends, every original section in between (+ optional spacing)."""
        e = self.s.span_coords()
        st = [eta0] + [float(x) for x in e if eta0 + 1e-6 < x < eta1 - 1e-6] + [eta1]
        if step:
            n = max(2, int(math.ceil((eta1 - eta0) / step)) + 1)
            st = sorted(set(st) | set(np.linspace(eta0, eta1, n).tolist()))
        return st

    def loft(self, eta0, eta1, fn, n=RING_N, step=None, start_dir=(1.0, 0.31), corners=False) -> G.Mesh:
        """Closed solid through per-station regions ``fn(poly, chord, eta) -> Polygon`` (simply connected). Stations
        outside the surface reuse the end section translated along the span direction (cutting tools). ``corners``:
        resample segment by segment between the sharp corners (thin channels: corner-to-corner correspondence)."""
        rings = []
        counts = None
        for eta in self.stations(eta0, eta1, step):
            e = self.clamp(eta)
            reg = SG.largest(fn(self.poly(e), self.chord(e), e))
            if len(reg.interiors):
                reg = Polygon(reg.exterior)
            if corners:
                P2, counts = resample_corners(reg, n, start_dir, counts)
            else:
                P2 = SG.resample_ring(reg, n, start_dir)
            o, c, u = self.frame(eta)
            rings.append(o + P2[:, :1] * c + P2[:, 1:2] * u)
        return G.loft(rings)

    def slab(self, eta0, eta1, s_fn, d0, d1, h=0.3) -> G.Mesh:
        """Convex slab whose section at every eta is the chord band s_fn(eta) + [d0, d1] (s_fn linear in eta)."""
        pts = []
        for eta in (eta0, eta1):
            for d in (d0, d1):
                for sgn in (-1.0, 1.0):
                    pts.append(self.p3(eta, s_fn(eta) + d, sgn * h))
        return G.hull(np.asarray(pts))

    def channel(self, eta0, eta1, s_fn, inset, aft=True, t_web=T_WEB, t_cap=T_CAP, w_cap=W_CAP, inner=None,
                inner_cap=None) -> G.Mesh:
        """C-channel spar between eta0 and eta1 along the straight line s_fn(eta) (web centre, chord position in the
        section frame): caps of width w_cap on the inner skin surface (``inset`` from the OML), opening aft or forward;
        booleans of convex slabs with the inset lofts (robust for thin webs)."""
        inner = inner or self.loft(eta0 - 0.01, eta1 + 0.01, f_oml(inset))
        inner_cap = inner_cap or self.loft(eta0 - 0.02, eta1 + 0.02, f_oml(inset + t_cap))
        if aft:
            a = self.slab(eta0, eta1, s_fn, -0.5 * t_web, -0.5 * t_web + w_cap)
            b = self.slab(eta0 - 0.01, eta1 + 0.01, s_fn, 0.5 * t_web, w_cap + 0.01)
        else:
            a = self.slab(eta0, eta1, s_fn, 0.5 * t_web - w_cap, 0.5 * t_web)
            b = self.slab(eta0 - 0.01, eta1 + 0.01, s_fn, -w_cap - 0.01, -0.5 * t_web)
        return D(I(inner, a), [I(inner_cap, b)])

    def plate(self, eta0, eta1, region2d) -> G.Mesh:
        """Planar part between the section planes eta0 < eta1; ``region2d`` in the frame of eta0."""
        o0, c, u = self.frame(eta0)
        origin = o0 + (eta1 - eta0) * self.w
        return extrude_cs(region2d, eta1 - eta0, origin, c, u)

    def inner_common(self, eta0, eta1, inset) -> Polygon:
        """Inset outline common to both faces of a plate between eta0 and eta1 (frame of eta0; convex sections)."""
        return self.poly_in(eta0, eta0, inset).intersection(self.poly_in(eta1, eta0, inset))


def _corner_idx(P: np.ndarray, min_turn: float = 30.0) -> list[int]:
    out = []
    k = len(P)
    for i in range(k):
        a, b, c = P[i - 1], P[i], P[(i + 1) % k]
        u, v = b - a, c - b
        nu, nv = np.linalg.norm(u), np.linalg.norm(v)
        if nu < 1e-12 or nv < 1e-12:
            continue
        if math.degrees(math.acos(np.clip(np.dot(u, v) / (nu * nv), -1, 1))) > min_turn:
            out.append(i)
    return out


def resample_corners(poly: Polygon, n: int, start_dir=(1.0, 0.31), counts=None):
    """Ring of ``n`` points through every sharp corner of ``poly`` (CCW), starting at the corner extreme in
    ``start_dir``; each corner-to-corner segment gets ``counts[i]`` points (computed from the first station when
    None and returned for the next stations, so the loft quads connect corresponding segments)."""
    from shapely.geometry.polygon import orient
    P = np.asarray(orient(poly, 1.0).exterior.coords)[:-1]
    keep = np.r_[True, np.linalg.norm(np.diff(P, axis=0), axis=1) > 1e-9]
    P = P[keep]
    ci = _corner_idx(P)
    if len(ci) < 3:
        raise ValueError("resample_corners: fewer than 3 corners")
    d = np.asarray(start_dir, float)
    j0 = int(np.argmax([P[i] @ d for i in ci]))
    ci = ci[j0:] + ci[:j0]
    segs = []
    for a, b in zip(ci, ci[1:] + ci[:1]):
        idx = list(range(a, b + 1)) if b > a else list(range(a, len(P))) + list(range(0, b + 1))
        segs.append(P[idx])
    lens = [float(np.sum(np.linalg.norm(np.diff(sg, axis=0), axis=1))) for sg in segs]
    if counts is None:
        tot = sum(lens)
        counts = []
        for sg, L in zip(segs, lens):
            a, b = sg[0], sg[-1]
            dv = b - a
            dev = np.abs((sg[:, 0] - a[0]) * dv[1] - (sg[:, 1] - a[1]) * dv[0]) / max(np.linalg.norm(dv), 1e-12)
            counts.append(1 if dev.max() < 1e-6 else max(2, int(round(n * L / tot))))   # straight edge: corners only
    if len(counts) != len(segs):
        raise ValueError(f"resample_corners: {len(segs)} segments, expected {len(counts)}")
    out = []
    for sg, L, m in zip(segs, lens, counts):
        cum = np.r_[0.0, np.cumsum(np.linalg.norm(np.diff(sg, axis=0), axis=1))]
        t = np.linspace(0.0, L, m, endpoint=False)
        out.append(np.column_stack([np.interp(t, cum, sg[:, 0]), np.interp(t, cum, sg[:, 1])]))
    return np.vstack(out), counts


def f_oml(inset=0.0):
    def fn(poly, ch, eta):
        return poly if inset <= 0 else poly.buffer(-inset, join_style=2)
    return fn


def f_band(x0_fn, x1_fn, inset=0.0):
    """Chord band between absolute chord positions x0_fn(ch, eta) .. x1_fn(ch, eta) of the (inset) section."""
    def fn(poly, ch, eta):
        base = poly.buffer(-inset, join_style=2) if inset > 0 else poly
        return base.intersection(sbox(x0_fn(ch, eta), -1.0, x1_fn(ch, eta), 1.0))
    return fn


def f_channel(xw_fn, inset, t_web=T_WEB, t_cap=T_CAP, w_cap=W_CAP, aft=True):
    """C-channel spar in the section: web centred on xw_fn(ch, eta), caps of width w_cap (incl. web) on the inner skin
    surface (inset), opening aft (caps aft of the web) or forward."""
    def fn(poly, ch, eta):
        inner = poly.buffer(-inset, join_style=2)
        xw = xw_fn(ch, eta)
        if aft:
            outer = inner.intersection(sbox(xw - 0.5 * t_web, -1.0, xw - 0.5 * t_web + w_cap, 1.0))
            hole = inner.buffer(-t_cap, join_style=2).intersection(sbox(xw + 0.5 * t_web, -1.0, 2.0, 1.0))
        else:
            outer = inner.intersection(sbox(xw + 0.5 * t_web - w_cap, -1.0, xw + 0.5 * t_web, 1.0))
            hole = inner.buffer(-t_cap, join_style=2).intersection(sbox(-2.0, -1.0, xw - 0.5 * t_web, 1.0))
        return SG.largest(outer.difference(hole))
    return fn


# =====================================================================================================================
# context
# =====================================================================================================================
class Ctx:
    def __init__(self, reg: Registry, spec: dict):
        self.reg = reg
        self.S = spec
        self.L = spec["layout"]
        T = spec["tail"]["surfaces"]
        self.T = T
        self.fin = Surf(T["fin"]["sections"], "fin")
        self.stab = Surf(T["stabilator"]["sections"], "stabilator")
        self.stub = Surf(T["stabilator_stub"]["sections"], "stabilator_stub")
        self.ven = Surf(T["ventral"]["sections"], "ventral")
        self.fus = O.fuselage_from_spec(spec)
        self.fit = {f["id"]: f for f in self.L["chassis"]["fittings"]}
        self.st = {s["id"]: s for s in self.L["stations"]}
        self.mech = {j["name"]: j for j in self.L["mechanisms"]["joints"]}
        self.act = {a["id"]: a for a in self.L["systems"]["actuators"]}
        self.eq = {e["id"]: e for e in self.L["systems"]["equipment"]}
        self.n0, self.n1 = self.L["part_numbers"]["tail"]
        self.t_skin = layup_props(spec, "tail_skin")["thickness"]
        self.t_rib = layup_props(spec, "rib_panel")["thickness"]
        self.sz = spec["structures"]["sizing"]["tail"]
        self._body = {}
        self.mirror_ids: list[str] = []          # starboard parts to mirror
        self.port_joint: dict[str, str] = {}

    # ---------------------------------------------------------------- ids
    def pid(self, num: int, side: str = "R", group: str = "tail") -> str:
        if not self.n0 <= num <= self.n1:
            raise ValueError(f"part number {num} outside the tail range")
        return part_number(group, num, side)

    # ---------------------------------------------------------------- body OML
    def body(self, offset: float, x0: float = 3.24, x1: float = 4.0, dx: float = 0.01) -> G.Mesh:
        """Closed body solid between x0 and x1: OML offset outward by ``offset`` (negative = inset)."""
        key = (round(offset, 6), round(x0, 4), round(x1, 4), round(dx, 4))
        if key not in self._body:
            n = max(2, int(math.ceil((x1 - x0) / dx)) + 1)
            rings = []
            for x in np.linspace(x0, x1, n):
                p = SG.fuselage_section2d(self.fus, float(x), 0.0, n=256)
                if offset > 0:
                    p = p.buffer(offset, join_style=2)
                elif offset < 0:
                    p = SG.largest(p.buffer(offset, join_style=2))
                P2 = SG.resample_ring(p, 192, start_dir=(0.0, 1.0))
                rings.append(np.column_stack([np.full(len(P2), x), P2[:, 0], P2[:, 1]]))
            self._body[key] = G.loft(rings)
        return self._body[key]

    # ---------------------------------------------------------------- registry
    def add(self, pid: str, name: str, name_tr: str, group: str, material: str, process: str, fn, *, thickness=None,
            layup=None, parent=None, step=0, explode=(0.0, 0.0, 0.0), contacts=(), joint=None, purchased=False,
            vendor="", mass_kg=None, notes="", mirror=True, color="") -> Part:
        side = "R" if pid.endswith("-R") else ("L" if pid.endswith("-L") else "C")
        p = Part(id=pid, name=name, name_tr=name_tr, group=group, material=material, process=process, mesh_fn=fn,
                 thickness=thickness, layup=layup, parent=parent, step=step, explode=tuple(explode),
                 contacts=tuple(contacts), joint=joint, purchased=purchased, vendor=vendor, mass_kg=mass_kg,
                 side=side, notes=notes, color=color)
        self.reg.add(p)
        if mirror and side == "R":
            self.mirror_ids.append(pid)
        return p


def _memo(fn):
    cache = {}

    def wrapped(*a):
        if a not in cache:
            cache[a] = fn(*a)
        return cache[a]
    return wrapped


# =====================================================================================================================
# stabilator system: spindle, bearings, stub (sleeve + skin), panel, drive
# =====================================================================================================================
class StabGeo:
    """Starboard stabilator-system geometry (spindle axis along +y through the layout pivot)."""

    def __init__(self, C: Ctx):
        self.C = C
        sp = C.T["stabilator"]
        piv = np.asarray(sp["pivot"], float)
        self.x0, self.z0 = float(piv[0]), float(piv[2])
        node = C.fit["F-SPINDLE-NODE"]
        sd = node["spindle"]
        cy = node["cylinder"]
        self.boss_y = (float(cy["center"][1]) - float(cy["half_length"]), float(cy["center"][1]) + float(cy["half_length"]))
        self.cheek_y = (float(node["boxes"][4][0][1]), float(node["boxes"][4][1][1]))     # outboard cheek faces
        self.y_in = self.boss_y[0] + 0.0015                     # spindle inboard end = bearing inboard face
        self.y_out = float(sd["span_y"][1])                     # 0.4385 (0.10 m in the root socket)
        self.y_ob = float(sd["outboard_bearing_y"])             # 0.32
        self.y_stub_tip = float(C.T["stabilator_stub"]["sections"][-1]["y"])
        self.y_root = float(sp["sections"][0]["y"])              # panel root 0.33851
        self.y_horn = float(sd["horn_y"])                       # 0.18
        self.bolts_cheek = [np.asarray(b["point"], float) for b in node["bolts"] if b["group"] == "stub root"]
        self.r_cheek_hole = 0.5 * float(node["spindle_hole_outboard_cheek"]["diameter"])
        # inboard bearing seat at the inboard end of the node boss, circlips either side
        self.brg_in = (self.y_in, self.y_in + BRG[2])
        self.brg_out = (self.y_ob - 0.5 * BRG[2], self.y_ob + 0.5 * BRG[2])
        # neck through the outboard cheek: +-10 mm beyond the 7 mm cheek
        self.neck = (self.cheek_y[0] - 0.010, self.cheek_y[1] + 0.010)
        self.collar = (self.brg_out[1], self.brg_out[1] + 0.003)
        self.y_cb = 0.420                                      # cross bolt station (in the plug)
        self.plug = (self.y_cb - 0.012, self.y_cb + 0.012)
        self.flange = (self.cheek_y[1], self.cheek_y[1] + 0.006)   # sleeve root flange on the cheek outboard face
        # socket / panel
        so = C.sz["socket"]
        self.r_sock = 0.5 * float(so["od_m"])
        self.socket = (self.y_root, self.y_root + float(so["length_m"]))
        self.root_rib = (self.y_root, self.y_root + C.t_rib)
        self.sock_rib = (self.socket[1], self.socket[1] + C.t_rib)
        # drive (layout EQ-STABACT linkage)
        lk = C.eq["EQ-STABACT"]["linkage"]
        self.S = np.asarray(lk["servo_axis"], float)
        self.H = np.asarray(lk["horn_axis"], float)
        self.arm = float(lk["servo_arm_m"])
        self.horn = float(lk["horn_m"])
        self.y_link = float(self.S[1])                          # pushrod plane y 0.18

    def axis_pt(self, y) -> np.ndarray:
        return np.array([self.x0, y, self.z0])

    # ---------------------------------------------------------------- spindle (TL-301)
    def spindle(self) -> G.Mesh:
        r, rn = SPINDLE_R, SPINDLE_NECK_R
        ri, rin = 0.5 * SPINDLE_ID, 0.5 * SPINDLE_ID_NECK
        y_solid = self.y_in + 0.015
        y_bore = self.neck[1] + 0.0105
        prof = [(0.0, self.y_in), (r, self.y_in), (r, self.neck[0]), (rn, self.neck[0]), (rn, self.neck[1]),
                (r, self.neck[1]), (r, self.collar[0]), (0.014, self.collar[0]), (0.014, self.collar[1]),
                (r, self.collar[1]), (r, self.y_out), (ri, self.y_out), (ri, y_bore), (rin, y_bore),
                (rin, y_solid), (0.0, y_solid)]
        return G.revolve(prof, n=48, axis_origin=(self.x0, 0.0, self.z0), axis=(0.0, 1.0, 0.0), ref=(0.0, 0.0, 1.0))

    def plug_mesh(self) -> G.Mesh:
        return cyl(0.5 * SPINDLE_ID - BOND, self.axis_pt(self.plug[0]), self.axis_pt(self.plug[1]))

    def bearing(self, y0, y1) -> G.Mesh:
        return ring(BRG_RI, BRG_RO, self.axis_pt(y0), self.axis_pt(y1))

    def circlip(self, y0, y1) -> G.Mesh:
        return ring(0.0160, BRG_RO, self.axis_pt(y0), self.axis_pt(y1))

    def end_washer(self) -> G.Mesh:
        return cyl(WASHER_R, self.axis_pt(self.y_in - 0.003), self.axis_pt(self.y_in))

    # ---------------------------------------------------------------- stub sleeve (root flange + bearing seat)
    def sleeve(self) -> G.Mesh:
        C = self.C
        f0, f1 = self.flange
        a = self.axis_pt
        pts = self.bolts_cheek
        # cruciform root flange: hub disc + 4 bolt bosses joined by arms (6 mm 7075)
        hub = Point(self.x0, self.z0).buffer(0.030, 64)
        parts = [hub]
        for p in pts:
            boss = Point(p[0], p[2]).buffer(0.0125, 48)
            arm = LineString([(self.x0, self.z0), (p[0], p[2])]).buffer(0.008, cap_style=2)
            parts += [boss, arm]
        fl2 = unary_union(parts).difference(Point(self.x0, self.z0).buffer(self.r_cheek_hole, 64))
        # (x, z) region extruded along +y from the cheek face
        fl = extrude_cs(fl2, f1 - f0, (0.0, f1, 0.0), (1.0, 0.0, 0.0), (0.0, 0.0, 1.0))
        y_tip = self.y_stub_tip
        tube = ring(0.0215, 0.0245, a(f1 - OV), a(y_tip - 0.020))
        seat = ring(0.0160, 0.0245, a(y_tip - 0.020 - OV), a(y_tip))
        body = U([fl, tube, seat])
        body = D(body, [cyl(BRG_RO + 0.000045, a(self.brg_out[0]), a(y_tip + 0.001))])     # 37 H7 seat to the tip
        env = U([C.stub.loft(C.stub.e0 - 0.01, C.stub.e1 + 0.01, f_oml(C.t_skin + BOND)),
                 box3((self.x0 - 0.06, f0 - 0.001, self.z0 - 0.06), (self.x0 + 0.06, f1 + 0.0005, self.z0 + 0.06))])
        return I(body, env)

    # ---------------------------------------------------------------- stab root fitting (socket + cross-bolt bosses)
    def root_fitting(self) -> G.Mesh:
        C = self.C
        a = self.axis_pt
        sock = ring(BRG_RI, self.r_sock, a(self.socket[0] + BOND), a(self.socket[1]))
        yb = self.y_cb
        bosses = [cyl(0.010, (self.x0, yb, self.z0 + s * 0.010), (self.x0, yb, self.z0 + s * 0.045)) for s in (-1, 1)]
        body = U([sock] + bosses)
        body = D(body, [cyl(BRG_RI, a(self.socket[0]), a(self.socket[1] + 0.001))])
        env = C.stab.loft(C.stab.e0 - 0.01, C.stab.e1, f_oml(C.t_skin + BOND))
        return I(body, env)


def build_stab_geo(C: Ctx) -> StabGeo:
    return StabGeo(C)


def _split(m: G.Mesh, vmin: float = 2e-8) -> list[G.Mesh]:
    """Connected solids of a mesh (largest first), slivers below vmin dropped."""
    parts = sorted([p for p in m.to_manifold().decompose() if p.volume() > vmin], key=lambda p: -p.volume())
    return [G.Mesh.from_manifold(p) for p in parts]


class StubPanelGeo:
    """Stub skin / ribs / front spar and stabilator panel skin / ribs / spar (starboard)."""

    def __init__(self, C: Ctx, g: StabGeo):
        self.C, self.g = C, g
        self._m = {}

    def m(self, key, fn):
        if key not in self._m:
            self._m[key] = fn()
        return self._m[key]

    # ------------------------------------------------------------ stub
    def stub_inner(self, extra=0.0):
        C = self.C
        return self.m(("stub_in", extra), lambda: C.stub.loft(C.stub.e0 - 0.01, C.stub.e1 + 0.01,
                                                              f_oml(C.t_skin + extra)))

    def stub_skin(self) -> G.Mesh:
        C = self.C
        outer = C.stub.loft(C.stub.e0, C.stub.e1, f_oml())
        return pieces_above(D(outer, [self.stub_inner(), C.body(TRIM)]))

    def sleeve_clear(self, r=0.0245 + BOND, y0=None, y1=None):
        g = self.g
        return cyl(r, g.axis_pt(y0 if y0 is not None else g.flange[0] - 0.01), g.axis_pt(y1 if y1 is not None else 0.40))

    def stub_root_rib(self) -> list[G.Mesh]:
        C = self.C
        band = D(C.body(TRIM + C.t_rib, 3.38, 4.0), [C.body(TRIM, 3.36, 4.0)])
        rib = D(I(self.stub_inner(BOND), band), [self.sleeve_clear()])
        return _split(rib)

    def stub_tip_rib(self) -> list[G.Mesh]:
        C, g = self.C, self.g
        e1 = C.stub.e1
        e0 = e1 - C.t_rib
        reg = C.stub.inner_common(e0, e1, C.t_skin + BOND)
        hole2 = C.stub.to2d(e0, [g.axis_pt(e0)])[0]
        reg = reg.difference(Point(hole2[0], hole2[1]).buffer(0.0245 + BOND, 64))
        return [C.stub.plate(e0, e1, p) for p in sorted(polys(reg), key=lambda q: -q.area)]

    def stub_front_spar(self) -> G.Mesh:
        C = self.C
        e1 = C.stub.e1 - C.t_rib - BOND
        sp = C.stub.channel(C.stub.e0 - 0.005, e1, lambda eta: STUB_FRONT_XC * C.stub.chord(eta), C.t_skin + BOND,
                            aft=False)
        return pieces_above(D(sp, [C.body(TRIM + C.t_rib + BOND, 3.38, 4.0)]))

    # ------------------------------------------------------------ stabilator panel
    def stab_inner(self, extra=0.0):
        C = self.C
        return self.m(("stab_in", extra), lambda: C.stab.loft(C.stab.e0 - 0.01, C.stab.e1 - 0.005,
                                                              f_oml(C.t_skin + extra)))

    def stab_skin(self) -> G.Mesh:
        C = self.C
        outer = C.stab.loft(C.stab.e0, C.stab.e1, f_oml())
        return D(outer, [self.stab_inner()])

    def spar_x(self, eta) -> float:
        """Stabilator spar web x: behind the socket at the root rib, 0.40 c at the tip."""
        C, g = self.C, self.g
        y0 = g.root_rib[1]
        x0 = g.x0 + g.r_sock + 0.010
        y1 = C.stab.e1 - 0.006
        x1 = C.stab.xc_pt(y1, 0.40)[0]
        return x0 + (x1 - x0) * (eta - y0) / (y1 - y0)

    def stab_spar(self) -> G.Mesh:
        C, g = self.C, self.g
        return C.stab.channel(g.root_rib[1] + BOND, C.stab.e1 - 0.006,
                              lambda eta: self.spar_x(eta) - C.stab.frame(eta)[0][0], C.t_skin + BOND, aft=True,
                              inner=self.stab_inner(BOND))

    def stab_root_rib(self) -> G.Mesh:
        C, g = self.C, self.g
        e0, e1 = g.root_rib
        reg = C.stab.inner_common(e0, e1, C.t_skin + BOND)
        c2 = C.stab.to2d(e0, [g.axis_pt(e0)])[0]
        reg = reg.difference(Point(c2[0], c2[1]).buffer(g.r_sock + BOND, 64))
        return C.stab.plate(e0, e1, reg)

    def _fwd_rib(self, e0, e1) -> G.Mesh:
        C = self.C
        reg = C.stab.inner_common(e0, e1, C.t_skin + BOND)
        xw = min(self.spar_x(e0), self.spar_x(e1)) - C.stab.frame(e0)[0][0] - 0.5 * T_WEB - BOND
        reg = SG.largest(reg.intersection(sbox(-1.0, -1.0, xw, 1.0)))
        return C.stab.plate(e0, e1, reg)

    def stab_socket_rib(self) -> G.Mesh:
        return self._fwd_rib(*self.g.sock_rib)

    def stab_mid_rib(self) -> G.Mesh:
        e = float(self.C.T["stabilator"]["sections"][1]["y"])
        return self._fwd_rib(e, e + self.C.t_rib)
