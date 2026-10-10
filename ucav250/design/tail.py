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
T_WEB = 0.0010              # spar webs: 5 plies PW (+-45; structures 4 plies, +1 for the bolted root / hinge lands)
T_CAP = 0.0017              # spar caps: 12 plies UD (structures.sizing.tail.*_spar_cap_plies)
W_CAP = 0.025               # spar cap width (structures.sizing.tail.*_spar_cap_width_m)
T_GFRP = 0.001              # fin-tip cap: 4 plies GFRP 7781 (RF window, 1.0 mm)
T_CAP_TIP = 0.0006          # spar caps taper (ply drops 1:20) to 4 plies UD at the tip
REAR_CAP = (0.0008, 0.020)  # fin rear (hinge) spar caps: 6 plies UD x 20 mm
FIN_FRONT_XC = 0.241        # layout F-FIN-FRONT fin_chord_fraction
FIN_REAR_DH = 0.040         # rear (hinge) spar web 40 mm ahead of the hinge line in every section (parallel to it): the
                            # DA 26 case (aft face 17.7 mm aft of its shaft) clears the rear-spar cap for the servo hatch
STUB_FRONT_XC = 0.25
RUDDER_GAP_END = 0.006      # spanwise gap rudder ends / fixed fin (rudder top swings 3.9 mm toward the tip cap)
CAP_SKIRT = 0.025           # tip-cap skirt length below the cap joint (2 x M3 into nutplates on the fin skin)
COVE_GAP = 0.002            # radial cove gap round the rudder nose
HINGE_S = (0.18, 0.45, 0.68)   # rudder hinge stations: fin span from the root reference section (m)
BOX_RIBS = (0.35, 0.70)     # fin box ribs (span coordinate eta), clear of the hinge-bracket nutplates and the servo bay
HINGE = A.HingeSpec(pin_d=0.003, lug_t=0.004, lug_r=0.0062, gap=0.0005, base_t=0.003, base_w=0.026, base_h=0.018,
                    bolt_d_mm=3.0, pin_clear=0.00002)
RUDDER_SPAR_E = 0.011       # rudder spar web front face 11 mm aft of the hinge line in the section (9.8 mm normal)
RUDDER_CAP = (0.0010, 0.032)   # rudder spar caps: 7 plies UD x 32 mm
HORN_R = 0.0268             # rudder horn ball 26.8 mm off the 26.6 deg swept hinge axis (24 mm lever in the rod plane)
HORN_ANG = -10.0            # horn ball 10 deg forward of the inboard normal (deg about the hinge axis, toward -cp)
ARM_ANG = -10.0             # servo arm 10 deg forward of the inboard normal at neutral: 1.98:1 at neutral, servo
                            # -58.7/+55.3 deg at +-25 deg rudder (sizing checks: -57.1/+58.4 deg, ratio 2.0 min)
ROD_D = 0.006               # rudder pushrod: 7075 rod 6 mm (machined, ball sockets integral)
BALL = (0.0035, 0.00365, 0.0055, 0.0015)   # rudder ball joints: ball r, socket cavity r, socket outer r, stud r
BALL_S = (0.0040, 0.00415, 0.0065, 0.0018)  # stabilator ball joints (pushrod force ~0.55 kN)
SOCKET_OPEN = 60.0          # socket opening half angle (deg): stud free for +-35 deg relative tilt, ball captured
STUD_L = 0.0047             # ball centre to the arm / horn plate face (socket rim clear at 20 deg tilt)
PLATE_T = 0.003             # arm and horn plates (7075)
HORN_PAD_W = 0.010          # rudder horn pad: 10 mm either side of the horn plate (span), 28 mm along the chord
DA26 = dict(L=0.054, H=0.1028, W=0.026, shaft_edge=0.0177, holes=(0.0612, 0.016), hole_d=0.0041, mass=0.270)
DA26_ETA0 = 0.4278          # case bottom face (output end) span coordinate: 7.8 mm above the arm mid-plane
DA26_FLANGE = (0.003, 0.0367, 0.0141)   # case mounting flange at the top end: thickness, half length (x), half width (t)
SERVO_RIB_T = 0.003         # 7075 servo rib (machined, bonded between the spars)
HATCH_X0 = 3.729            # servo hatch opening forward edge (x); aft edge 1 mm ahead of the rear-spar cap
HATCH_ETA = (0.412, 0.540)  # opening span range
HATCH_T = 0.0010            # cover: 5 plies PW in a joggled monolithic land (core ramped out), flush with the OML
HATCH_LAND = 0.020          # land width above / below the opening (2 x 2.5 D for M4)
HATCH_SCREW_X = (3.742, 3.775)
SPINDLE_R = 0.01247         # spindle OD 25 (polygon-safe radius inside the 25 H7 bores)
SPINDLE_NECK_R = 0.0105     # neck OD 21 through the node cheek hole (>= 5 mm moving-part clearance, layout.clearances)
SPINDLE_ID = 0.022          # bore 22 (wall 1.5 = processes.cnc_milling_metal.min_thickness; structures 1.2)
SPINDLE_ID_NECK = 0.018    # bore 18 from the inboard end plug to past the neck (neck wall 1.5)
BRG = (0.025, 0.037, 0.007)  # 61805-ZZ: d, D, B
BRG_RO = 0.01847            # outer race radius inside a 37 H7 bore modelled with 48 facets
BRG_RI = 0.01253            # inner race bore
WASHER_R = 0.0134           # moving parts inside the node boss stay >= 5 mm off the 37 mm bore
DEG = math.pi / 180.0
# stabilator drive (EQ-STABACT): DA 30 lying along y on the firewall forward face, shaft along +y at the outboard end
DA30 = dict(L=0.1585, H=0.085, W=0.030, holes=(0.077, 0.018), axis_from_holes=0.0216, mass=0.630)
DA30_SHIFT = (-0.0025, -0.0095)   # case moved 2.5 mm forward (base-bolt heads behind the case) and 9.5 mm inboard
                                  # (output arm in the pushrod plane y 0.18) from the layout box
STUD_L_S = 0.0045           # stabilator ball centre to the arm / horn plate face
CRADLE_T = 0.003            # 7075 cradle plates
HORN_HUB = (0.170, 0.190, 0.0195)   # spindle horn pinch hub: y range, outer radius
HORN_EARS = (0.035, 0.0125, 0.011)  # pinch ears forward of the spindle: x reach, x start, half height
# ventral fin (F-VENTRAL-1..3): tongue in the chassis clevis slot, fork cheeks on the mid-plane web (1, 2) or tongue in a
# slot of the solid trailing-edge region (3); replaceable PA12 skid shoe on a 4130 strap round the tip
V_SLOT = (0.004, -0.0655)   # chassis clevis slot half width (y) and slot top z (CH-099..101 ears 5.3 mm, slot 8 mm)
V_LUG_HX = 0.0158           # tongue half length along x (fitting box 32 mm, 0.2 mm off the firewall sheet)
V_WEB_T = 0.003             # mid-plane web: 15 plies PW
V_CHEEK_T = 0.003
V_CAVITY_XC = 0.80          # sandwich cavity forward of 0.80 c; core-filled trailing-edge closeout aft of it
SKID = dict(x=(4.065, 4.125), strap_t=0.002, shoe_side=0.004, shoe_top=0.025, strap_top=0.035, height=0.012)
# fin spar root fittings (7075, machined): chassis clevis slots from the layout boxes (F-FIN-FRONT: 6 mm ears either side
# of an 8 mm slot; F-FW-CORNER: slot x 3.680-3.688 between the firewall-side web and the 6 mm aft ear, chassis.md
# interface table), lug faces 0.1 mm off the ears, flange on the spar web
FF_SLOT = (3.4894, 3.4974)  # F-FIN-FRONT box x 3.4834-3.5034: ears 6 mm, slot 8 mm
FF_FLOOR = 0.2532           # slot floor = clevis base (box z0 0.2292 + 24 mm base)
FF_LUG = ((0.127, 0.1715), 0.2885)      # lug y range, lug top z
FF_NECK = ((0.141, 0.159), 0.300)       # neck y range (through the dorsal hat gap), neck top z
FF_BOLTS = ((0.1400, 0.2655), (0.1584, 0.2655))   # 2 x M6 12.9 double shear (y, z), axis +x (layout B1/B2 axis)
RF_SLOT = (3.680, 3.688)    # F-FW-CORNER clevis slot (chassis.md: kept free for y >= 0.140)
RF_FLOOR = 0.263
RF_LUG = (((0.134, 0.172), (0.2633, 0.290)), ((0.140, 0.1715), (0.2895, 0.303)))
RF_NECK_X0 = 3.6835         # neck / upper block start aft of the firewall heat-shield lip (x <= 3.683)
RF_EAR_TOP = 0.3045         # upper block above the aft ear (top z 0.304)
RF_BOLTS = ((0.1505, 0.2745), (0.161, 0.2925))    # 2 x M5 12.9 (y, z), head on the aft ear, tapped into the web
FL_T, FL_W, FL_LEN = 0.005, 0.012, 0.060          # web flange: thickness, half width (along t), length along the span
DBL_T = 0.002               # rear-spar slot doubler (7075 C-section)


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
        e = self.clamp(eta)
        key = (round(e, 9), round(inset, 7))
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
        if inner_cap is None:
            if callable(t_cap):
                tc = t_cap
                inner_cap = self.loft(eta0 - 0.02, eta1 + 0.02,
                                      lambda p, ch, eta: p.buffer(-(inset + tc(eta)), join_style=2))
            else:
                inner_cap = self.loft(eta0 - 0.02, eta1 + 0.02, f_oml(inset + t_cap))
        if aft:
            a = self.slab(eta0, eta1, s_fn, -0.5 * t_web, -0.5 * t_web + w_cap)
            b = self.slab(eta0 - 0.01, eta1 + 0.01, s_fn, 0.5 * t_web, w_cap + 0.01)
        else:
            a = self.slab(eta0, eta1, s_fn, 0.5 * t_web - w_cap, 0.5 * t_web)
            b = self.slab(eta0 - 0.01, eta1 + 0.01, s_fn, -w_cap - 0.01, -0.5 * t_web)
        return D(I(inner, a), [I(inner_cap, b)])

    def reg_in(self, eta, eta_ref, fn) -> Polygon:
        """Region fn(poly, chord, eta) at eta expressed in the frame of eta_ref."""
        e = self.clamp(eta)
        p = SG.largest(fn(self.poly(e), self.chord(e), e))
        o, c, u = self.frame(eta)
        P2 = np.asarray(p.exterior.coords)
        return Polygon(self.to2d(eta_ref, o + P2[:, :1] * c + P2[:, 1:2] * u)).buffer(0)

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


def clean(poly, tol: float = 2e-5):
    """Remove slivers / spikes thinner than 2 tol (morphological opening) and near-collinear vertices."""
    p = poly.buffer(-tol, join_style=2).buffer(tol, join_style=2)
    p = SG.largest(p) if not isinstance(p, Polygon) else p
    return p.simplify(1e-6, preserve_topology=True)


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
        self._sec = {}
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

    def oml_dist(self, P) -> float:
        """Signed distance (m) of P from the body OML in its x station plane (positive outside the body)."""
        x = round(float(P[0]), 5)
        if x not in self._sec:
            self._sec[x] = SG.fuselage_section2d(self.fus, x, 0.0, n=256)
        poly = self._sec[x]
        pt = Point(float(P[1]), float(P[2]))
        d = poly.exterior.distance(pt)
        return -d if poly.contains(pt) else d

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


class StabDrive:
    """DA 30 + cradle, servo arm, pushrod and spindle horn of the starboard stabilator (planar four-bar in y = 0.18)."""

    def __init__(self, C: Ctx, g: StabGeo):
        self.C, self.g = C, g
        lk = C.eq["EQ-STABACT"]["linkage"]
        S0 = np.asarray(lk["servo_axis"], float)
        self.S = S0 + np.array([DA30_SHIFT[0], 0.0, 0.0])
        self.yl = float(S0[1])                                   # pushrod plane
        self.arm_l = float(lk["servo_arm_m"])
        self.horn_l = float(lk["horn_m"])
        box = np.asarray(C.eq["EQ-STABACT"]["box"], float)
        self.case_lo = box[0] + np.array([DA30_SHIFT[0], DA30_SHIFT[1], 0.0])
        self.case_hi = box[1] + np.array([DA30_SHIFT[0], DA30_SHIFT[1], 0.0])
        self.A0 = self.S + np.array([0.0, 0.0, self.arm_l])
        self.H = g.axis_pt(self.yl)
        self.B0 = self.H + np.array([0.0, 0.0, self.horn_l])
        self.lk = Linkage(self.S, (0.0, 1.0, 0.0), self.A0, self.H, (0.0, 1.0, 0.0), self.B0)
        # firewall forward face (layout FS3670 x_faces[0]): cradle base behind the case
        self.x_fw = float(C.st["FS3670"]["x_faces"][0])

    def da30(self) -> G.Mesh:
        lo, hi = self.case_lo, self.case_hi
        case = box3(lo, hi)
        stub = cyl(0.005, (self.S[0], hi[1] - OV, self.S[2]), (self.S[0], self.arm_y()[0] - BOND, self.S[2]), n=32)
        return U([case, stub])

    def da30_holes(self):
        """Lug-pattern bolt lines along z (datasheet 77 x 18 mm, 21.6 mm from the output axis)."""
        hy, hx = DA30["holes"]
        yc = 0.5 * (self.case_lo[1] + self.case_hi[1])
        x1 = self.S[0] - DA30["axis_from_holes"]
        return [(x, yc + sy * 0.5 * hy) for x in (x1, x1 - hx) for sy in (-1, 1)]

    def cradle_dims(self):
        xs = [x for x, _y in self.da30_holes()]
        ys = [y for _x, y in self.da30_holes()]
        return (min(xs) - 0.009, self.x_fw - BOND - CRADLE_T), (min(ys) - 0.008, max(ys) + 0.008)

    def cradle(self) -> G.Mesh:
        lo, hi = self.case_lo, self.case_hi
        (x0, xb), (y0, y1) = self.cradle_dims()
        top = box3((x0, y0, hi[2] + BOND), (xb + OV, y1, hi[2] + BOND + CRADLE_T))
        bot = box3((x0, y0, lo[2] - BOND - CRADLE_T), (xb + OV, y1, lo[2] - BOND))
        base = box3((xb, y0, lo[2] - BOND - CRADLE_T - 0.012), (xb + CRADLE_T, y1, hi[2] + BOND + CRADLE_T))
        return U([top, bot, base])

    def base_bolts(self):
        """(head point on the base forward face, axis +x) of the 4 base screws into the firewall inserts."""
        (x0, xb), (y0, y1) = self.cradle_dims()
        z = self.case_lo[2] - BOND - CRADLE_T - 0.006
        return [((xb, y, z), (1.0, 0.0, 0.0)) for y in (y0 + 0.010, y1 - 0.010)]

    def arm_y(self):
        y1 = self.yl - STUD_L_S
        return y1 - PLATE_T, y1

    def arm(self) -> G.Mesh:
        y0, y1 = self.arm_y()
        reg = unary_union([Point(self.S[0], self.S[2]).buffer(0.008, 48), Point(self.A0[0], self.A0[2]).buffer(0.006, 48),
                           LineString([(self.S[0], self.S[2]), (self.A0[0], self.A0[2])]).buffer(0.005, cap_style=2)])
        plate = extrude_cs(reg, y1 - y0, (0.0, y1, 0.0), (1.0, 0.0, 0.0), (0.0, 0.0, 1.0))
        stud = cyl(BALL_S[3], self.A0, (self.A0[0], y1 - OV, self.A0[2]), n=24)
        return U([plate, stud, G.sphere(BALL_S[0], self.A0, n=32)])

    def rod(self) -> G.Mesh:
        return rod_mesh(self.A0, self.B0, 0.004, BALL_S, ((0.0, -1.0, 0.0), (0.0, 1.0, 0.0)))

    def horn_plate_y(self):
        y0 = self.yl + STUD_L_S
        return y0, y0 + PLATE_T

    def horn(self) -> G.Mesh:
        g = self.g
        x0, z0 = g.x0, g.z0
        hy0, hy1, ro = HORN_HUB
        hub = ring(SPINDLE_R + BOND, ro, g.axis_pt(hy0), g.axis_pt(hy1), n=64)
        ex, es, eh = HORN_EARS
        ears = box3((x0 - ex, hy0, z0 - eh), (x0 - es, hy1, z0 + eh))
        body = D(U([hub, ears]), [cyl(SPINDLE_R + BOND, g.axis_pt(hy0 - 0.001), g.axis_pt(hy1 + 0.001), n=64),
                                  box3((x0 - ex - 0.001, hy0 - 0.001, z0 - 0.0005), (x0 - 0.010, hy1 + 0.001, z0 + 0.0005))])
        py0, py1 = self.horn_plate_y()
        reg = unary_union([Point(x0, z0).buffer(ro - 0.001, 64), Point(self.B0[0], self.B0[2]).buffer(0.006, 48)]).convex_hull
        reg = reg.difference(Point(x0, z0).buffer(SPINDLE_R + 0.002, 64))
        plate = extrude_cs(reg, py1 - py0, (0.0, py1, 0.0), (1.0, 0.0, 0.0), (0.0, 0.0, 1.0))
        stud = cyl(BALL_S[3], self.B0, (self.B0[0], py0 + OV, self.B0[2]), n=24)
        return U([body, plate, stud, G.sphere(BALL_S[0], self.B0, n=32)])

    def pinch_bolt(self):
        """(head point on the upper ear, axis -z, ear stack thickness) of the horn pinch bolt."""
        g = self.g
        ex, _es, eh = HORN_EARS
        return (g.x0 - ex + 0.009, 0.5 * (HORN_HUB[0] + HORN_HUB[1]), g.z0 + eh), (0.0, 0.0, -1.0), 2 * eh


class VenGeo:
    """Ventral fin (centre line): sandwich skin, root rib, mid-plane web, root fittings, two monolithic CFRP lug pads
    (third root lug, skid), skid strap and shoe."""

    def __init__(self, C: Ctx):
        self.C = C
        self.V = C.ven
        self.fits = [C.fit[f"F-VENTRAL-{i}"] for i in (1, 2, 3)]
        self._m = {}

    def m(self, key, fn):
        if key not in self._m:
            self._m[key] = fn()
        return self._m[key]

    def cavity(self, extra=0.0):
        """Sandwich cavity forward of 0.80 c (core-filled trailing-edge closeout aft of it)."""
        V, C = self.V, self.C

        def fn(poly, ch, eta):
            return SG.largest(poly.buffer(-(C.t_skin + extra), join_style=2).intersection(
                sbox(-1.0, -1.0, V_CAVITY_XC * ch, 1.0)))
        return self.m(("cav", extra), lambda: V.loft(V.e0 - 0.01, V.e1 - C.t_skin - extra, fn, step=0.01))

    def z_oml(self, x) -> float:
        p = SG.fuselage_section2d(self.C.fus, float(x), 0.0, n=256)
        g = LineString([(0.0, -1.0), (0.0, 0.0)]).intersection(p)
        zs = np.asarray(g.coords)[:, 1] if hasattr(g, "coords") else np.concatenate([np.asarray(q.coords)[:, 1]
                                                                                       for q in g.geoms])
        return float(zs.min())

    def outer(self):
        """OML with the trailing edge cut square at 0.995 c (1.3 mm trailing-edge land: no knife edge at the root trim)."""
        V = self.V
        return self.m("outer", lambda: V.loft(V.e0, V.e1, lambda p, ch, eta: SG.largest(
            p.intersection(sbox(-1.0, -1.0, 0.995 * ch, 1.0))), step=0.01))

    def zones(self, grow=0.0) -> list:
        """Monolithic CFRP lug pad of the third root lug (the skid bolts sit in the core-filled trailing edge)."""
        x3 = self.fit_x(2)
        z3 = self.fork_bolt_z(2)[-1] - 0.016
        return [box3((x3 - 0.030 - grow, -0.05, z3 - grow), (x3 + 0.030 + grow, 0.05, 0.0))]

    def skin_raw(self) -> G.Mesh:
        C = self.C
        return pieces_above(D(self.outer(), [self.cavity(), C.body(TRIM, 3.40, 4.2), self.slot3()]))

    def pad(self, k) -> G.Mesh:
        C = self.C
        cut = [C.body(TRIM, 3.40, 4.2)] + ([self.slot3()] if k == 0 else [])
        return pieces_above(D(I(self.cavity(BOND), self.zones()[k]), cut))

    def root_rib(self) -> G.Mesh:
        C = self.C
        band = D(C.body(TRIM + C.t_rib, 3.45, 4.1), [C.body(TRIM, 3.43, 4.1)])
        return pieces_above(D(I(self.cavity(BOND), band), self.zones(BOND) + self.rib_holes()))

    def web(self) -> G.Mesh:
        C = self.C
        w = I(self.cavity(BOND), box3((3.4, -0.5 * V_WEB_T, -0.4), (4.3, 0.5 * V_WEB_T, 0.0)))
        return pieces_above(D(w, [C.body(TRIM + C.t_rib + BOND, 3.43, 4.1)] + [self.crotch_clear(i) for i in (0, 1)]
                              + self.zones(BOND)))

    def fit_x(self, i) -> float:
        return float(self.fits[i]["point"][0])

    def z_crotch(self, i) -> float:
        x = self.fit_x(i)
        return min(self.z_oml(x + dx) for dx in (-V_LUG_HX, 0.0, V_LUG_HX)) - TRIM - self.C.t_rib - 0.002

    def crotch_clear(self, i) -> G.Mesh:
        x, zc = self.fit_x(i), self.z_crotch(i)
        return box3((x - V_LUG_HX - BOND, -0.02, zc - 0.006 - BOND), (x + V_LUG_HX + BOND, 0.02, 0.0))

    def fitting(self, i) -> G.Mesh:
        x = self.fit_x(i)
        hy, ztop = V_SLOT[0] - BOND, V_SLOT[1] - BOND
        if i < 2:
            zc = self.z_crotch(i)
            tongue = box3((x - V_LUG_HX, -hy, zc - OV), (x + V_LUG_HX, hy, ztop))
            crotch = box3((x - V_LUG_HX, -(0.5 * V_WEB_T + BOND + V_CHEEK_T), zc - 0.006),
                          (x + V_LUG_HX, 0.5 * V_WEB_T + BOND + V_CHEEK_T, zc))
            cheeks = [box3((x - V_LUG_HX, s0, zc - 0.046), (x + V_LUG_HX, s1, zc - 0.006 + OV))
                      for s0, s1 in ((0.5 * V_WEB_T + BOND, 0.5 * V_WEB_T + BOND + V_CHEEK_T),
                                     (-(0.5 * V_WEB_T + BOND + V_CHEEK_T), -(0.5 * V_WEB_T + BOND)))]
            return U([tongue, crotch] + cheeks)
        zb = self.fork_bolt_z(i)[-1] - 0.0105
        return box3((x - V_LUG_HX, -hy, zb), (x + V_LUG_HX, hy, ztop))

    def fork_bolt_z(self, i):
        if i < 2:
            zc = self.z_crotch(i)
            return [zc - 0.006 - 0.0125 - BOND, zc - 0.006 - 0.0285]
        z0 = self.z_oml(self.fit_x(i)) - TRIM - 0.0135
        return [z0, z0 - 0.016]

    def slot3(self) -> G.Mesh:
        x = self.fit_x(2)
        return box3((x - V_LUG_HX - BOND, -V_SLOT[0], self.fork_bolt_z(2)[-1] - 0.0105 - BOND),
                    (x + V_LUG_HX + BOND, V_SLOT[0], 0.0))

    def rib_holes(self) -> list:
        out = []
        for i in (0, 1, 2):
            x = self.fit_x(i)
            hw = (0.5 * V_WEB_T + BOND + V_CHEEK_T + BOND) if i < 2 else V_SLOT[0]
            out.append(box3((x - V_LUG_HX - BOND, -hw, -0.4), (x + V_LUG_HX + BOND, hw, 0.0)))
        return out

    # ------------------------------------------------------------ skid
    def tip_loft(self, e0, e1, off):
        V = self.V
        return V.loft(e0, e1, lambda p, ch, eta: p.buffer(off, join_style=2))

    def strap(self) -> G.Mesh:
        V = self.V
        x0, x1 = SKID["x"]
        t = SKID["strap_t"]
        outer = self.tip_loft(V.e1 - SKID["strap_top"], V.e1 + t, t)
        inner = self.tip_loft(V.e1 - SKID["strap_top"] - 0.01, V.e1 + BOND, BOND)
        return I(D(outer, [inner]), box3((x0, -0.05, -0.4), (x1, 0.05, 0.0)))

    def shoe(self) -> G.Mesh:
        V = self.V
        x0, x1 = SKID["x"]
        t = SKID["strap_t"]
        outer = self.tip_loft(V.e1 - SKID["shoe_top"], V.e1 + SKID["height"], t + BOND + SKID["shoe_side"])
        inner = self.tip_loft(V.e1 - SKID["shoe_top"] - 0.01, V.e1 + t + BOND, t + BOND)
        return I(D(outer, [inner]), box3((x0 + 0.001, -0.05, -0.4), (x1 - 0.001, 0.05, 0.0)))

    def skid_bolts(self):
        V = self.V
        z = float(V.frame(V.e1)[0][2]) + 0.012
        return [((x, -0.05, z), (0.0, 1.0, 0.0)) for x in (4.088, 4.110)]


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
        e0, e1 = g.root_rib[1] + BOND, C.stab.e1 - 0.006
        tc = lambda eta: T_CAP + (T_CAP_TIP - T_CAP) * min(1.0, max(0.0, (eta - e0) / (e1 - e0)))
        return C.stab.channel(e0, e1, lambda eta: self.spar_x(eta) - C.stab.frame(eta)[0][0], C.t_skin + BOND,
                              aft=True, inner=self.stab_inner(BOND), t_cap=tc)

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


def socket_mesh(c, r_i, r_o, opening, half_angle_deg=SOCKET_OPEN) -> G.Mesh:
    """Snap-on ball socket (DIN 71802 type) of a pushrod end: spherical shell r_i..r_o round the ball centre c, open
    toward the stud (``opening``) with a cone of ``half_angle_deg``; the opening (r_i sin a) is smaller than the ball."""
    shell = D(G.sphere(r_o, c, n=32), [G.sphere(r_i, c, n=32)])
    L = r_o + 0.002
    cone = G.cylinder(1e-5, c, np.asarray(c, float) + L * unit(opening), n=32,
                      r1=L * math.tan(math.radians(half_angle_deg)))
    return D(shell, [cone])


def rod_mesh(A, B, r_rod, ball, studs) -> G.Mesh:
    """Pushrod with a ball socket at each end (ball centres A and B, sockets open toward the studs)."""
    _r_b, r_i, r_o, _r_s = ball
    A, B = np.asarray(A, float), np.asarray(B, float)
    d = unit(B - A)
    off = math.sqrt(max(r_o ** 2 - r_rod ** 2, 0.0)) - 0.0003
    return U([socket_mesh(A, r_i, r_o, studs[0]), socket_mesh(B, r_i, r_o, studs[1]),
              cyl(r_rod, A + off * d, B - off * d, n=24)])


# =====================================================================================================================
# four-bar linkage kinematics (servo arm about the output shaft, horn about the hinge / spindle axis, rigid pushrod
# between two spherical rod ends)
# =====================================================================================================================
def rot(p, origin, axis, ang):
    return G.rotate_about(np.atleast_2d(p), origin, axis, ang)[0]


class Linkage:
    """Rest geometry: shaft (S, sa), arm tip A0; hinge (H, ha), horn ball centre B0. ``solve(delta)`` returns the arm
    angle theta (rad, right hand about sa) for a surface angle delta (about ha) with |A - B| = rod length."""

    def __init__(self, S, sa, A0, H, ha, B0):
        self.S, self.sa = np.asarray(S, float), unit(sa)
        self.A0 = np.asarray(A0, float)
        self.H, self.ha = np.asarray(H, float), unit(ha)
        self.B0 = np.asarray(B0, float)
        self.L = float(np.linalg.norm(self.B0 - self.A0))
        u0 = unit(self.B0 - self.A0)
        e1 = unit(np.cross(u0, self.sa)) if abs(np.dot(u0, self.sa)) < 0.95 else unit(np.cross(u0, [0, 0, 1.0]))
        e2 = np.cross(u0, e1)
        self.u0, self.e1, self.e2 = u0, e1, e2           # rod frame: e1, e2 _|_ rod, e3 = u0

    def A(self, th):
        return rot(self.A0, self.S, self.sa, th)

    def B(self, d):
        return rot(self.B0, self.H, self.ha, d)

    def solve(self, d, th0=0.0):
        th = th0
        Bd = self.B(d)
        for _ in range(80):
            f = np.linalg.norm(self.A(th) - Bd) - self.L
            if abs(f) < 1e-11:
                break
            h = 1e-6
            df = (np.linalg.norm(self.A(th + h) - Bd) - np.linalg.norm(self.A(th - h) - Bd)) / (2 * h)
            if abs(df) < 1e-12:
                raise ValueError("linkage toggles")
            th -= f / df
        if abs(np.linalg.norm(self.A(th) - Bd) - self.L) > 1e-8:
            raise ValueError("linkage has no solution")
        return th

    def sweep(self, lo, hi, n=41):
        """[(delta, theta, phi1, phi2)] over the range (Newton continuation from neutral both ways)."""
        out = {0.0: (0.0, 0.0, 0.0)}
        for rng in (np.linspace(0.0, hi, n), np.linspace(0.0, lo, n)):
            th = 0.0
            for d in rng[1:]:
                th = self.solve(d, th)
                out[float(d)] = (th,) + self.rod_angles(d, th)
        return [(d,) + out[d] for d in sorted(out)]

    def rod_angles(self, d, th):
        """Rod pose as two rotations at the arm-end ball (axes e1 then e2 of the rest rod frame, applied after... see
        Registry.posed_vertices: child joints first) so that R_arm(th) R_e1(p1) R_e2(p2) u0 = rod direction."""
        u = unit(self.B(d) - self.A(th))
        R = G.rot_axis_angle(self.sa, th)
        wv = R.T @ u
        w1, w2, w3 = wv @ self.e1, wv @ self.e2, wv @ self.u0
        p2 = math.asin(max(-1.0, min(1.0, w1)))
        p1 = math.atan2(-w2, w3)
        return p1, p2

    def ratio(self, d, th, h=1e-5):
        """Surface-to-arm torque ratio = d(theta) / d(delta) (mechanical advantage of the four-bar)."""
        t2 = self.solve(d + h, th)
        t1 = self.solve(d - h, th)
        return (t2 - t1) / (2 * h)


# =====================================================================================================================
# fin + rudder
# =====================================================================================================================
class FinGeo:
    """Starboard fin and rudder geometry (section frame of the fin: s along the chord, t along u = inboard normal)."""

    def __init__(self, C: Ctx):
        self.C = C
        F = self.F = C.fin
        prm = C.T["fin"]["params"]
        span = float(prm["span"])
        rc = C.T["fin"]["controls"]["rudder"]
        self.e_r0 = F.e0 + float(rc["eta0"]) * span
        self.e_r1 = F.e0 + float(rc["eta1"]) * span
        self.e_cap = self.e_r1 + RUDDER_GAP_END
        self.e_aft = (self.e_r0 - RUDDER_GAP_END - C.t_rib, self.e_r0 - RUDDER_GAP_END)
        self.e_tip = (self.e_cap - C.t_rib, self.e_cap)
        self.xh = float(rc["xc_hinge"])
        self.fixed_fn, self.moving_fn, self.hinge_fn = SG.control_surface_regions(self.xh, gap=COVE_GAP, hinge_frac=0.5)
        j = C.mech["rudder_R"]
        self.h_o, self.h_a = np.asarray(j["origin"], float), unit(j["axis"])
        self.aw = float(self.h_a @ F.w)
        act = C.act["ACT-RUDDER"]
        self.Ssh = np.asarray(act["servo_axis"], float)
        self.Hp = np.asarray(act["hinge_point"], float)
        self.e_L = F.e0 + float((self.Ssh - F.frame(F.e0)[0]) @ F.w)
        self.ti = C.t_skin + BOND
        self.cp = unit(F.c - (F.c @ self.h_a) * self.h_a)       # chord direction normal to the hinge axis
        self._m = {}

    def m(self, key, fn):
        if key not in self._m:
            self._m[key] = fn()
        return self._m[key]

    # ---------------------------------------------------------------- stations / lines
    def axis_at(self, eta) -> np.ndarray:
        return self.h_o + (eta - self.e_r0) / self.aw * self.h_a

    def s_hinge(self, eta) -> float:
        return float((self.axis_at(eta) - self.F.frame(eta)[0]) @ self.F.c)

    def s_front(self, eta) -> float:
        return FIN_FRONT_XC * self.F.chord(eta)

    def s_rear(self, eta) -> float:
        return self.s_hinge(eta) - FIN_REAR_DH

    def r_nose(self, eta) -> float:
        tu, tl = self.F.half_t(eta, self.s_hinge(eta) / self.F.chord(eta))
        return min(tu, -tl)

    # ---------------------------------------------------------------- lofts
    def outer(self):
        F = self.F
        return self.m("outer", lambda: F.loft(F.e0, self.e_cap, f_oml()))

    def inner(self, extra=0.0):
        F = self.F
        return self.m(("inner", extra), lambda: F.loft(F.e0 - 0.01, self.e_cap + 0.01, f_oml(self.C.t_skin + extra)))

    def cove_cut(self):
        """Volume removed from the fixed fin for the rudder: aft of the hinge line + the cove disc (r + gap)."""
        def fn(poly, ch, eta):
            s, t = self.hinge_fn(poly, ch)
            tu, tl = SG._span_t(poly, s)
            r = min(tu - t, t - tl)
            return unary_union([Point(s, t).buffer(r + COVE_GAP, 96), sbox(s, t - 0.1, s + 0.40, t + 0.1)])
        F = self.F
        return self.m("cove", lambda: F.loft(self.e_r0 - RUDDER_GAP_END, self.e_cap + 0.02, fn, n=480))

    def root_band(self, lo=0.0, hi=None):
        C = self.C
        hi = C.t_rib if hi is None else hi
        return D(C.body(TRIM + hi, 3.24, 4.0), [C.body(TRIM + lo, 3.22, 4.0)])

    # ---------------------------------------------------------------- fixed fin structure
    def skin_raw(self) -> G.Mesh:
        C = self.C
        return D(self.outer(), [self.inner(), C.body(TRIM), self.cove_cut()])

    def front_spar(self) -> G.Mesh:
        F, C = self.F, self.C
        e1 = self.e_tip[0] - BOND
        tc = lambda eta: T_CAP + (T_CAP_TIP - T_CAP) * min(1.0, max(0.0, (eta - F.e0) / (e1 - F.e0)))
        sp = F.channel(F.e0, e1, self.s_front, self.ti, aft=True, inner=self.inner(BOND), t_cap=tc)
        return pieces_above(D(sp, [C.body(TRIM + C.t_rib + BOND, 3.24, 4.0)]))

    def rear_spar_raw(self) -> G.Mesh:
        F, C = self.F, self.C
        sp = F.channel(F.e0, self.e_tip[0] - BOND, self.s_rear, self.ti, aft=False, inner=self.inner(BOND),
                       t_cap=REAR_CAP[0], w_cap=REAR_CAP[1])
        return pieces_above(D(sp, [C.body(TRIM + C.t_rib + BOND, 3.24, 4.0), self.cove_cut()]))

    def root_rib_raw(self) -> G.Mesh:
        return I(self.inner(BOND), self.root_band())

    def box_rib_region(self, e0, e1) -> Polygon:
        """Rib between the spar webs (frame of e0), notched round the front-spar caps (aft) and rear-spar caps
        (forward)."""
        F = self.F
        reg = F.inner_common(e0, e1, self.ti)
        regc = F.inner_common(e0, e1, self.ti + T_CAP + BOND)
        dx1 = float((F.frame(e1)[0] - F.frame(e0)[0]) @ F.c)
        sf = [self.s_front(e0), self.s_front(e1) + dx1]
        sr = [self.s_rear(e0), self.s_rear(e1) + dx1]
        a = max(sf) + 0.5 * T_WEB + BOND
        b = min(sr) - 0.5 * T_WEB - BOND
        rib = reg.intersection(sbox(a, -1.0, b, 1.0))
        capz = reg.difference(regc)
        regr = F.inner_common(e0, e1, self.ti + REAR_CAP[0] + BOND)
        notch = capz.intersection(sbox(a - 0.01, -1.0, max(sf) - 0.5 * T_WEB + W_CAP + BOND, 1.0)).union(
            reg.difference(regr).intersection(sbox(min(sr) + 0.5 * T_WEB - REAR_CAP[1] - BOND, -1.0, b + 0.01, 1.0)))
        return clean(SG.largest(rib.difference(notch)))

    def box_rib(self, e0, e1) -> G.Mesh:
        return self.F.plate(e0, e1, self.box_rib_region(e0, e1))

    def aft_rib(self) -> G.Mesh:
        F = self.F
        e0, e1 = self.e_aft
        reg = F.inner_common(e0, e1, self.ti)
        dx1 = float((F.frame(e1)[0] - F.frame(e0)[0]) @ F.c)
        a = max(self.s_rear(e0), self.s_rear(e1) + dx1) + 0.5 * T_WEB + BOND
        return F.plate(e0, e1, SG.largest(reg.intersection(sbox(a, -1.0, 1.0, 1.0))))

    def tip_rib(self) -> G.Mesh:
        F = self.F
        e0, e1 = self.e_tip

        def g(poly, ch, eta):
            return SG.largest(self.fixed_fn(poly, ch, eta).buffer(-self.ti, join_style=2))
        reg = F.reg_in(e0, e0, g).intersection(F.reg_in(e1, e0, g))
        return F.plate(e0, e1, SG.largest(reg))

    def cap_shell(self) -> G.Mesh:
        F = self.F
        outer = F.loft(self.e_cap, F.e1, f_oml())
        inner = F.loft(self.e_cap - 0.01, F.e1 - T_GFRP, f_oml(T_GFRP))
        return D(outer, [inner])

    def _fixed3d(self, e0, e1, grow=0.0):
        """Fixed part of the section forward of the cove, 2 mm (- grow) clear of the cove disc."""
        F = self.F

        def fn(poly, ch, eta):
            s, t = self.hinge_fn(poly, ch)
            tu, tl = SG._span_t(poly, s)
            r = min(tu - t, t - tl)
            big = poly.buffer(0.01, join_style=2)
            return SG.largest(big.intersection(sbox(-1.0, -1.0, s, 1.0)).difference(
                Point(s, t).buffer(r + COVE_GAP + 0.002 - grow, 96)))
        return F.loft(e0, e1, fn)

    def cap_flange(self, grow=0.0) -> G.Mesh:
        """Skirt of the tip cap (GFRP, T_GFRP) in a joggled recess of the fixed-fin skin below the cap joint, forward of
        the cove (the cap drops onto the tapering fin from above); ``grow`` > 0 gives the recess cutter."""
        F = self.F
        e0, e1 = self.e_cap - CAP_SKIRT - grow, self.e_cap + (OV if grow == 0 else 0.002)
        band = D(F.loft(e0, e1, lambda p, ch, eta: p.buffer(0.001 if grow else 0.0, join_style=2)),
                 [F.loft(e0 - 0.01, e1 + 0.01, f_oml(T_GFRP + grow))])
        return I(band, self._fixed3d(e0 - 0.001, e1 + 0.001, grow))

    def tip_cap(self) -> G.Mesh:
        """GFRP tip cap (antenna window): closed shell above the cap joint plus the skirt below it (forward of the cove)."""
        F = self.F
        e0 = self.e_cap - CAP_SKIRT
        outer = F.loft(e0, F.e1, f_oml())
        inner = F.loft(e0 - 0.01, F.e1 - T_GFRP, f_oml(T_GFRP))
        aft = D(F.loft(e0 - 0.01, self.e_cap, lambda p, ch, eta: p.buffer(0.01, join_style=2)),
                [self._fixed3d(e0 - 0.02, self.e_cap + 0.001)])
        return D(outer, [inner, aft])

    def cap_screws(self):
        """(point on the skirt outer surface, axis into the fin) of the two M3 cap screws (inboard / outboard face)."""
        F = self.F
        eta = self.e_cap - CAP_SKIRT + 0.009
        o = F.frame(eta)[0]
        s_ = 0.35 * F.chord(eta)
        tu, tl = SG._span_t(F.poly(eta), s_)
        return [(F.p3(eta, s_, tu), -F.u), (F.p3(eta, s_, tl), F.u)]

    # ---------------------------------------------------------------- rudder
    def rudder_outer(self):
        F = self.F
        return self.m("r_out", lambda: F.loft(self.e_r0, self.e_r1, self.moving_fn, n=240))

    def rudder_inner(self, extra=0.0):
        F = self.F

        def fn(poly, ch, eta):
            return SG.largest(self.moving_fn(poly, ch, eta).buffer(-(T_SKIN_CS + extra), join_style=2))
        return self.m(("r_in", extra), lambda: F.loft(self.e_r0 + T_SKIN_CS + extra, self.e_r1 - T_SKIN_CS - extra,
                                                      fn, n=240))

    def s_rspar(self, eta) -> float:
        return self.s_hinge(eta) + RUDDER_SPAR_E + 0.5 * T_WEB

    def rudder_spar(self) -> G.Mesh:
        F = self.F
        tc, wc = RUDDER_CAP
        inner = self.rudder_inner(BOND)
        e0, e1 = self.e_r0 + T_SKIN_CS + BOND, self.e_r1 - T_SKIN_CS - BOND
        capin = F.loft(e0 - 0.01, e1 + 0.01, lambda p, ch, eta: SG.largest(
            self.moving_fn(p, ch, eta).buffer(-(T_SKIN_CS + BOND + tc), join_style=2)), n=240)
        return F.channel(e0, e1, self.s_rspar, 0.0, aft=True, t_cap=tc, w_cap=wc, inner=inner, inner_cap=capin)

    def rudder_core(self) -> G.Mesh:
        F = self.F
        tc, wc = RUDDER_CAP
        e0, e1 = self.e_r0 + T_SKIN_CS + BOND, self.e_r1 - T_SKIN_CS - BOND
        inner = self.rudder_inner(BOND)
        capin = F.loft(e0 - 0.01, e1 + 0.01, lambda p, ch, eta: SG.largest(
            self.moving_fn(p, ch, eta).buffer(-(T_SKIN_CS + 2 * BOND + tc), join_style=2)), n=240)
        x0 = 0.5 * T_WEB + BOND
        x1 = -0.5 * T_WEB + wc + BOND
        a = I(capin, F.slab(e0 - 0.001, e1 + 0.001, self.s_rspar, x0, x1 + OV))
        b = I(inner, F.slab(e0 - 0.001, e1 + 0.001, self.s_rspar, x1, 0.5))
        return U([a, b])

    def rudder_skin_raw(self) -> G.Mesh:
        return D(self.rudder_outer(), [self.rudder_inner()])

    # ---------------------------------------------------------------- hinges (clevis on the rear spar, tongue on the rudder spar)
    def hinge_eta(self, i) -> float:
        return self.F.e0 + HINGE_S[i]

    def clevis(self, i) -> dict:
        reach = (FIN_REAR_DH - 0.5 * T_WEB) * self.aw - BOND
        return self.m(("clevis", i), lambda: A.hinge_bracket_fixed(self.axis_at(self.hinge_eta(i)), self.h_a, self.cp,
                                                                   HINGE, reach))

    def tongue(self, i) -> dict:
        reach = RUDDER_SPAR_E * self.aw - BOND
        return self.m(("tongue", i), lambda: A.hinge_bracket_moving(self.axis_at(self.hinge_eta(i)), self.h_a, -self.cp,
                                                                    HINGE, reach))

    def nose_notch(self, i, deg=29.0, n=13) -> G.Mesh:
        """Cut-out in the rudder nose skin for the clevis lugs over the travel (clevis envelope + 1 mm, rotated through
        -deg..+deg about the hinge axis in the rudder frame)."""
        p = self.axis_at(self.hinge_eta(i))
        ef = -self.cp
        ea = self.h_a
        et = np.cross(ef, ea)
        span = HINGE.lug_t * 1.5 + HINGE.gap + 0.001 + 0.0005
        half_t = 0.5 * min(HINGE.base_h, 2.2 * HINGE.lug_r) + 0.001
        bx = obox(p + 0.017 * ef, [ef, ea, et], [0.017, span + 0.0015, half_t])
        return U([G.Mesh(G.rotate_about(bx.V, p, ea, a), bx.F) for a in np.radians(np.linspace(-deg, deg, n))])

    # ---------------------------------------------------------------- rudder drive (DA 26, arm, rod, horn)
    def linkage(self) -> "Linkage":
        F = self.F
        a, b = math.radians(ARM_ANG), math.radians(HORN_ANG)
        A0 = self.Ssh + float(self.C.act["ACT-RUDDER"]["linkage"]["servo_arm_m"]) * (math.cos(a) * F.u + math.sin(a) * F.c)
        B0 = self.Hp + HORN_R * (math.cos(b) * F.u + math.sin(b) * self.cp)
        return Linkage(self.Ssh, -F.w, A0, self.h_o, self.h_a, B0)

    def rudder_states(self, n=13):
        """[(delta, theta, phi1, phi2)] of the rudder four-bar over the joint range (n steps per side)."""
        j = self.C.mech["rudder_R"]
        return self.m(("states", n), lambda: self.linkage().sweep(float(j["lo"]), float(j["hi"]), n=n))

    def rod_sweep(self, grow=0.0015, cut0=0.0095, cut1=0.0085) -> G.Mesh:
        """Volume swept by the pushrod tube (radius + grow) between its sockets over the travel (hulls of neighbouring
        states): cut from the fixed fin skin and the rear spar (rod slot)."""
        lk = self.linkage()
        segs = [(lk.A(th), lk.B(d)) for d, th, _p1, _p2 in self.rudder_states()]
        r = ROD_D / 2 + grow
        hulls = []
        for (A1, B1), (A2, B2) in zip(segs, segs[1:]):
            pts = []
            for A_, B_ in ((A1, B1), (A2, B2)):
                d = unit(B_ - A_)
                e1, e2 = J._perp(d)
                for P in (A_ + cut0 * d, B_ - cut1 * d):
                    for k in range(16):
                        a = 2 * math.pi * k / 16
                        pts.append(P + r * (math.cos(a) * e1 + math.sin(a) * e2))
            hulls.append(G.hull(np.asarray(pts)))
        return self.m(("rodsweep", grow), lambda: U(hulls))

    def eta_of(self, P) -> float:
        F = self.F
        return float(F.e0 + (np.asarray(P, float) - F.frame(F.e0)[0]) @ F.w)

    def arm_eta(self):
        e1 = DA26_ETA0 - BOND
        return e1 - PLATE_T, e1

    def arm(self) -> G.Mesh:
        F = self.F
        lk = self.linkage()
        e0, e1 = self.arm_eta()
        hub = F.to2d(e0, [self.Ssh])[0]
        tip = F.to2d(e0, [lk.A0])[0]
        reg = unary_union([Point(*hub).buffer(0.007, 48), Point(*tip).buffer(0.005, 48),
                           LineString([hub, tip]).buffer(0.004, cap_style=2)])
        plate = F.plate(e0, e1, reg)
        stud = cyl(BALL[3], lk.A0, lk.A0 + (e0 - self.e_L + OV) * F.w, n=24)
        return U([plate, stud, G.sphere(BALL[0], lk.A0, n=32)])

    def horn_eta(self):
        eB = self.eta_of(self.linkage().B0)
        return eB + STUD_L, eB + STUD_L + PLATE_T

    def horn_pad_s(self, eta):
        sh = self.s_hinge(eta)
        return sh + 0.013, sh + 0.041

    def horn(self) -> G.Mesh:
        F = self.F
        lk = self.linkage()
        e0, e1 = self.horn_eta()
        s1, s2 = self.horn_pad_s(e0)

        def pad_fn(poly, ch, eta):
            band = poly.buffer(0.0025, join_style=2).difference(poly)
            return SG.largest(band.intersection(sbox(s1, 0.0, s2, 1.0)))
        pad = F.loft(e0 - HORN_PAD_W, e1 + HORN_PAD_W, pad_fn, n=160)
        poly = F.poly(e0)
        bst = F.to2d(e0, [lk.B0])[0]
        top = []
        for s_ in np.linspace(s1 + 0.003, s2 - 0.003, 5):
            tu, _tl = SG._span_t(poly, s_)
            top.append((s_, tu + 0.0024))
        hull = unary_union([Point(*bst).buffer(0.005, 48)] + [Point(*q).buffer(0.0001, 8) for q in top]).convex_hull
        reg = SG.largest(hull.difference(poly.buffer(0.0023, join_style=2)))
        plate = F.plate(e0, e1, reg)
        stud = cyl(BALL[3], lk.B0, lk.B0 + (STUD_L + OV) * F.w, n=24)
        return U([pad, plate, stud, G.sphere(BALL[0], lk.B0, n=32)])

    def horn_bolts(self):
        """(point on the pad outer surface, axis into the rudder) of the two M3 horn screws (beside the horn plate)."""
        F = self.F
        e0, e1 = self.horn_eta()
        out = []
        for eta in (e0 - 0.004, e1 + 0.004):
            s_ = self.s_hinge(eta) + 0.027
            tu, _tl = SG._span_t(F.poly(eta), s_)
            out.append((F.p3(eta, s_, tu + 0.0025), -F.u))
        return out

    def rod(self) -> G.Mesh:
        lk = self.linkage()
        return rod_mesh(lk.A0, lk.B0, ROD_D / 2, BALL, (self.C.fin.w, self.C.fin.w))

    def da26(self) -> G.Mesh:
        F = self.F
        L, H, W = DA26["L"], DA26["H"], DA26["W"]
        e0, e1 = DA26_ETA0, DA26_ETA0 + H
        xa = self.Ssh[0] + DA26["shaft_edge"]                      # case aft face: shaft 17.7 mm from it
        xc = xa - 0.5 * L
        oc = F.frame(e0)[0][0]
        reg = sbox(xa - L - oc, -0.5 * W, xa - oc, 0.5 * W)
        case = F.plate(e0, e1, reg)
        ft, fl, fw = DA26_FLANGE
        of = F.frame(e1 - ft)[0][0]
        fl2 = sbox(xc - fl - of, -fw, xc + fl - of, fw)
        flange = F.plate(e1 - ft, e1, fl2)
        return U([case, flange])

    def da26_holes(self):
        """(point on the flange bottom face, axis) of the four mounting screws (datasheet pattern 61.2 x 16)."""
        F = self.F
        e1 = DA26_ETA0 + DA26["H"]
        xc = self.Ssh[0] + DA26["shaft_edge"] - 0.5 * DA26["L"]
        oc = F.frame(e1)[0][0]
        hx, ht = DA26["holes"]
        return [(F.p3(e1 - DA26_FLANGE[0], xc + sx * 0.5 * hx - oc, st * 0.5 * ht), F.w) for sx in (-1, 1)
                for st in (-1, 1)]

    def servo_rib_eta(self):
        e0 = DA26_ETA0 + DA26["H"] + BOND
        return e0, e0 + SERVO_RIB_T

    def servo_rib(self) -> G.Mesh:
        return self.box_rib(*self.servo_rib_eta())

    # ---------------------------------------------------------------- servo hatch (inboard face)
    def cap_front_x(self, eta) -> float:
        return float(self.F.frame(eta)[0][0] + self.s_rear(eta) + 0.5 * T_WEB - REAR_CAP[1])

    def _footprint(self, x0, eta0, eta1, aft_off, t0=0.0, t1=0.06) -> G.Mesh:
        F = self.F
        pts = []
        for eta in (eta0, eta1):
            ox = F.frame(eta)[0][0]
            for x in (x0, self.cap_front_x(eta) + aft_off):
                for t in (t0, t1):
                    pts.append(F.p3(eta, x - ox, t))
        return G.hull(np.asarray(pts))

    def hatch_cover_fp(self, grow=0.0):
        e0, e1 = HATCH_ETA
        return self._footprint(HATCH_X0 - 0.003 - grow, e0 - 0.0225 - grow, e1 + 0.0225 + grow, 0.002 + grow)

    def hatch_opening(self):
        return self._footprint(HATCH_X0, HATCH_ETA[0], HATCH_ETA[1], -0.001, t0=-0.001)

    def hatch_band(self, depth):
        F = self.F
        e0, e1 = HATCH_ETA[0] - 0.04, HATCH_ETA[1] + 0.04
        return D(F.loft(e0, e1, f_oml()), [F.loft(e0 - 0.01, e1 + 0.01, f_oml(depth))])

    def hatch_cover(self) -> G.Mesh:
        return I(self.hatch_band(HATCH_T), self.hatch_cover_fp())

    def hatch_recess(self) -> G.Mesh:
        F = self.F
        e0, e1 = HATCH_ETA[0] - 0.04, HATCH_ETA[1] + 0.04
        outer = F.loft(e0, e1, lambda p, ch, eta: p.buffer(0.002, join_style=2))
        return I(D(outer, [F.loft(e0 - 0.01, e1 + 0.01, f_oml(HATCH_T + BOND))]), self.hatch_cover_fp(BOND))

    def hatch_screws(self):
        """(point on the cover outer surface, axis into the fin) of the 4 x M4 cover screws."""
        F = self.F
        out = []
        for eta in (HATCH_ETA[0] - 0.011, HATCH_ETA[1] + 0.011):
            o = F.frame(eta)[0]
            for x in HATCH_SCREW_X:
                tu, _tl = SG._span_t(F.poly(eta), x - o[0])
                out.append((F.p3(eta, x - o[0], tu), -F.u))
        return out

    # ---------------------------------------------------------------- spar root fittings
    def eta_on_rib(self, s_fn, t=0.0, h=None) -> float:
        """Span coordinate where the line (s_fn(eta), t) leaves the root rib (OML + TRIM + t_rib + BOND)."""
        C, F = self.C, self.F
        h = TRIM + C.t_rib + BOND if h is None else h
        lo, hi = F.e0, F.e0 + 0.30
        for _ in range(60):
            mid = 0.5 * (lo + hi)
            if C.oml_dist(F.p3(mid, s_fn(mid), t)) > h:
                hi = mid
            else:
                lo = mid
        return 0.5 * (lo + hi)

    def block(self, eta0, eta1, s_fn, d0, d1, t0, t1) -> G.Mesh:
        """Convex block between eta0 and eta1 whose section is the chord band s_fn(eta) + [d0, d1] x t [t0, t1]."""
        F = self.F
        return G.hull(np.asarray([F.p3(e, s_fn(e) + d, t) for e in (eta0, eta1) for d in (d0, d1) for t in (t0, t1)]))

    def flange_span(self, s_fn):
        eb = self.eta_on_rib(s_fn)
        return eb - 0.004, eb + FL_LEN

    def flange(self, s_fn, front_face=True) -> tuple:
        """(mesh, (eta0, eta1), (d0, d1)) of the 5 mm web flange on the web front face of the spar along s_fn."""
        e0, e1 = self.flange_span(s_fn)
        d0, d1 = -0.5 * T_WEB - BOND - FL_T, -0.5 * T_WEB - BOND
        return self.block(e0, e1, s_fn, d0, d1, -FL_W, FL_W), (e0, e1), (d0, d1)

    def _bridge(self, top_pts, s_fn, e0, d0, d1) -> G.Mesh:
        F = self.F
        bot = [F.p3(e0, s_fn(e0) + d, t) for d in (d0, d1) for t in (-FL_W, FL_W)]
        return G.hull(np.asarray(list(top_pts) + bot))

    def _fitting_pieces(self, front: bool) -> list:
        """Convex pieces of a spar root fitting (vertex sets): lug / neck boxes, bridge hull, web flange."""
        def bx(lo, hi):
            return [(x, y, z) for x in (lo[0], hi[0]) for y in (lo[1], hi[1]) for z in (lo[2], hi[2])]
        F = self.F
        if front:
            x0, x1 = FF_SLOT[0] + BOND, FF_SLOT[1] - BOND
            (y0, y1), zt = FF_LUG
            (ny0, ny1), zn = FF_NECK
            pcs = [bx((x0, y0, FF_FLOOR + BOND), (x1, y1, zt)), bx((x0, ny0, zt - OV), (x1, ny1, zn))]
            top = [(x, y, zn - OV) for x in (x0, x1) for y in (ny0, ny1)]
            s_fn = self.s_front
        else:
            x0, x1 = RF_SLOT[0] + BOND, RF_SLOT[1] - BOND
            (ya, za), (yb, zb) = RF_LUG
            ze = RF_EAR_TOP + 0.008
            pcs = [bx((x0, ya[0], za[0]), (x1, ya[1], za[1])), bx((x0, yb[0], za[1] - OV), (x1, yb[1], zb[1])),
                   bx((RF_NECK_X0, 0.142, zb[1] - OV), (x1, 0.165, RF_EAR_TOP + OV)),
                   bx((RF_NECK_X0, 0.142, RF_EAR_TOP), (RF_SLOT[1] + 0.0035, 0.165, ze))]
            top = [(x, y, ze - OV) for x in (RF_NECK_X0, RF_SLOT[1] + 0.0035) for y in (0.142, 0.165)]
            s_fn = self.s_rear
        e0, e1 = self.flange_span(s_fn)
        d0, d1 = -0.5 * T_WEB - BOND - FL_T, -0.5 * T_WEB - BOND
        bot = [tuple(F.p3(e0, s_fn(e0) + d, t)) for d in (d0, d1) for t in (-FL_W, FL_W)]
        pcs.append(top + bot)
        pcs.append([tuple(F.p3(e, s_fn(e) + d, t)) for e in (e0, e1) for d in (d0, d1) for t in (-FL_W, FL_W)])
        return pcs

    def fitting(self, front: bool, grow: float = 0.0) -> G.Mesh:
        out = []
        for pts in self._fitting_pieces(front):
            P = np.asarray(pts, float)
            if grow > 0:
                P = np.vstack([P + grow * d for d in np.vstack([np.eye(3), -np.eye(3)])])
            out.append(G.hull(P))
        return U(out)

    def front_fitting(self) -> G.Mesh:
        return self.fitting(True)

    def rear_fitting(self) -> G.Mesh:
        return self.fitting(False)

    def flange_bolts(self, front: bool):
        """(point on the flange outer face, axis through flange then web) of the two M5 flange bolts."""
        F = self.F
        s_fn = self.s_front if front else self.s_rear
        e0, _e1 = self.flange_span(s_fn)
        eb = e0 + 0.004
        out = []
        for e in (eb + 0.016, eb + 0.046):
            p0 = F.p3(e, s_fn(e) - 0.5 * T_WEB - BOND - FL_T, 0.0)
            dp = F.p3(e + 1e-3, s_fn(e + 1e-3), 0.0) - F.p3(e - 1e-3, s_fn(e - 1e-3), 0.0)
            n = unit(np.cross(dp, F.u))
            n = n if n @ F.c > 0 else -n
            out.append((p0, n))
        return out

    def slot_doubler(self) -> G.Mesh:
        """7075 C-section doubler nested in the rear spar round the pushrod slot (carries the inboard cap round it)."""
        F = self.F
        e0, e1 = self.e_L - 0.040, self.e_L + 0.040
        ti = self.ti + REAR_CAP[0] + BOND
        outer_l = F.loft(e0 - 0.01, e1 + 0.01, f_oml(ti))
        inner_l = F.loft(e0 - 0.02, e1 + 0.02, f_oml(ti + DBL_T))
        a = F.slab(e0, e1, self.s_rear, -0.5 * T_WEB - BOND - 0.018, -0.5 * T_WEB - BOND)
        b = F.slab(e0 - 0.01, e1 + 0.01, self.s_rear, -0.05, -0.5 * T_WEB - BOND - DBL_T)
        return pieces_above(D(D(I(outer_l, a), [I(inner_l, b)]), [self.rod_sweep()]))
