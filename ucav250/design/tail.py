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
STUB_N = 320                # stub rings (two stations only): 3.4 mm spacing, chord sag at the inner nose < 0.25 mm
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
# stub front-spar root fitting in F-STUB-FRONT (CH-098): chassis detailing agreement for the clevis = 4 mm ears either
# side of an 8 mm slot open outboard, slot floor 10 mm outboard of the frame bolts B1/B2 (chassis.build_stub_front)
SF_EAR, SF_FLOOR_DY = 0.004, 0.010
SF_D = 0.005                # lug bolts M5 12.9 (layout B3/B4 M6): 10 mm = 2 D to the slot floor and to the ear edge
SF_LUG_E = 0.011            # lug edge above / below the bolts
SF_NECK_Y1 = 0.268          # 7.8 mm lug/neck zone ends here; 5 mm flange zone outboard of it
SF_FL_Y, SF_FL_END = 0.303, 0.012   # flange bolt station (12.5 mm+ from the web root and tip ends) and flange end beyond
SF_FL_HZ, SF_FL_DZ, SF_FL_T = 0.021, 0.010, 0.005   # flange half height, flange bolts +-10 mm about the stub plane, t
RUDDER_GAP_END = 0.006      # spanwise gap rudder ends / fixed fin (rudder top swings 3.9 mm toward the tip cap)
CAP_SKIRT = 0.025           # tip-cap skirt length below the cap joint (2 x M3 into nutplates on the fin skin)
COVE_GAP = 0.002            # radial cove gap round the rudder nose
HINGE_S = (0.18, 0.45, 0.68)   # rudder hinge stations: fin span from the root reference section (m)
BOX_RIBS = (0.35, 0.70)     # fin box ribs (span coordinate eta), clear of the hinge-bracket nutplates and the servo bay
HINGE = A.HingeSpec(pin_d=0.003, lug_t=0.004, lug_r=0.0062, gap=0.0005, base_t=0.003, base_w=0.033, base_h=0.018,
                    bolt_d_mm=3.0, pin_clear=0.00002)
RUDDER_SPAR_E = 0.0145      # rudder spar web front face 14.5 mm aft of the hinge line in the section (13 mm normal):
                            # tongue-bolt holes >= 3 D from the hinge-pin bore
RUDDER_CAP = (0.0010, 0.020)   # rudder spar caps: 7 plies UD x 20 mm
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
HORN_PAD_W = 0.011          # rudder horn pad: 11 mm either side of the horn plate (span), 28 mm along the chord
DA26 = dict(L=0.054, H=0.1028, W=0.026, shaft_edge=0.0177, holes=(0.0612, 0.016), hole_d=0.0041, mass=0.270)
DA26_ETA0 = 0.4278          # case bottom face (output end) span coordinate: 7.8 mm above the arm mid-plane
DA26_FLANGE = (0.003, 0.0367, 0.0141)   # case mounting flange at the top end: thickness, half length (x), half width (t)
SERVO_RIB_T = 0.003         # 7075 servo rib (machined, bonded between the spars)
HATCH_X0 = 3.729            # servo hatch opening forward edge (x); aft edge 1 mm ahead of the rear-spar cap
HATCH_ETA = (0.412, 0.540)  # opening span range
HATCH_T = 0.0010            # cover: 5 plies PW in a joggled monolithic land (core ramped out), flush with the OML
HATCH_LAND = 0.020          # land width above / below the opening (2 x 2.5 D for M4)
HATCH_SCREW_X = ((3.742, 3.775), (3.766, 3.801))   # lower / upper screw pairs (upper pair and its nutplate strip aft
                                                    # of the front-spar cap)
SPINDLE_R = 0.01247         # spindle OD 25 (polygon-safe radius inside the 25 H7 bores)
SPINDLE_NECK_R = 0.0105     # neck OD 21 through the node cheek hole (>= 5 mm moving-part clearance, layout.clearances)
SPINDLE_ID = 0.022          # bore 22 (wall 1.5 = processes.cnc_milling_metal.min_thickness; structures 1.2)
SPINDLE_ID_NECK = 0.018    # bore 18 from the inboard end plug to past the neck (neck wall 1.5)
BRG = (0.025, 0.037, 0.007)  # 61805-ZZ: d, D, B
BRG_RO = 0.01847            # outer race radius inside a 37 H7 bore modelled with 48 facets
BRG_RI = 0.01253            # inner race bore
WASHER_R = 0.0134           # moving parts inside the node boss stay >= 5 mm off the 37 mm bore
DEG = math.pi / 180.0
SLEEVE_T = 0.002            # sleeve tube wall (OD 49): 65 MPa at the ultimate outboard-bearing moment (tail.md)
# stabilator drive (EQ-STABACT): DA 30 lying along y on the firewall forward face, shaft along +y at the outboard end
DA30 = dict(L=0.1585, H=0.085, W=0.030, holes=(0.077, 0.018), axis_from_holes=0.0216, mass=0.630)
DA30_SHIFT = (-0.0025, -0.0095)   # case moved 2.5 mm forward (base-bolt heads behind the case) and 9.5 mm inboard
                                  # (output arm in the pushrod plane y 0.18) from the layout box
STUD_L_S = 0.0045           # stabilator ball centre to the arm / horn plate face
CRADLE_T = 0.003            # 7075 cradle plates
CRADLE_WIN = 0.009          # cradle lightening windows stay 9 mm inside the DA 30 lug-bolt rows
HORN_HUB = (0.170, 0.190, 0.0175)   # spindle horn pinch hub: y range, outer radius (wall 5 mm)
HORN_EARS = (0.035, 0.0125, 0.011)  # pinch ears forward of the spindle: x reach, x start, half height
# ventral fin (F-VENTRAL-1..3): tongue in the chassis clevis slot, fork cheeks on the mid-plane web (1, 2) or tongue in a
# slot of the solid trailing-edge region (3); replaceable PA12 skid shoe on a 4130 strap round the tip
V_SLOT = (0.004, -0.0655)   # chassis clevis slot half width (y) and slot top z (CH-099..101 ears 5.3 mm, slot 8 mm)
V_LUG_HX = 0.0158           # tongue half length along x (fitting box 32 mm, 0.2 mm off the firewall sheet)
V_WEB_T = 0.003             # mid-plane web: 15 plies PW
V_CHEEK_T = 0.003
V_FORK_DX = (0.018, 0.0)     # fork-bolt pair x offset from the fitting point (fitting 1: 18 mm aft, where the cavity
                            # takes the nut; its cheeks end 3.5 mm ahead of fitting 2)
V_CAVITY_XC = 0.80          # sandwich cavity forward of 0.80 c; core-filled trailing-edge closeout aft of it
SKID = dict(x=(4.054, 4.114), strap_t=0.002, shoe_side=0.004, shoe_top=0.025, strap_top=0.035, height=0.012)
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
FL_T, FL_W, FL_LEN = 0.005, 0.012, 0.062          # web flange: thickness, half width (along t), length along the span
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
        h = 1e-4 * (self.e1 - self.e0)
        # leading-edge line slope at both ends: frames outside the surface continue the straight LE (cutting tools)
        self._d0 = (self.s.frame_at(self.e0 + h)[0] - o) / h
        self._d1 = (self.s.frame_at(self.e1)[0] - self.s.frame_at(self.e1 - h)[0]) / h
        self._c0 = (self.s.chord_at(self.e0 + h) - self.s.chord_at(self.e0)) / h
        self._c1 = (self.s.chord_at(self.e1) - self.s.chord_at(self.e1 - h)) / h

    def clamp(self, eta):
        return min(max(eta, self.e0), self.e1)

    def frame(self, eta):
        e = self.clamp(eta)
        o, c, u, w = self.s.frame_at(e)
        if eta < e:
            o = o + (eta - e) * self._d0
        elif eta > e:
            o = o + (eta - e) * self._d1
        return o, self.c, self.u

    def chord(self, eta) -> float:
        """Chord at eta; outside the surface the end taper continues linearly (like ``frame``), so chord-fraction
        lines (spar webs) stay straight in the cutting tools that reach past the ends."""
        e = self.clamp(eta)
        c = self.s.chord_at(e)
        if eta < e:
            c += (eta - e) * self._c0
        elif eta > e:
            c += (eta - e) * self._c1
        return c

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
        x = xc * self.chord(self.clamp(eta))
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
        tube = ring(0.0245 - SLEEVE_T, 0.0245, a(f1 - OV), a(y_tip - 0.020))
        seat = ring(0.0160, 0.0245, a(y_tip - 0.020 - OV), a(y_tip))
        body = U([fl, tube, seat])
        body = D(body, [cyl(BRG_RO + 0.000045, a(self.brg_out[0]), a(y_tip + 0.001))])     # 37 H7 seat to the tip
        env = U([C.stub.loft(C.stub.e0 - 0.01, C.stub.e1 + 0.01, f_oml(C.t_skin + BOND), n=STUB_N),
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
        base = box3((xb, y0, lo[2] - BOND - CRADLE_T - 0.0145), (xb + CRADLE_T, y1, hi[2] + BOND + CRADLE_T))
        # lightening windows (r 4 mm corners): plates keep 9 mm round the lug bolts (> 2 D) and a frame to the base;
        # the base keeps its screw strip below the case and the plate roots
        ys = sorted(y for _x, y in self.da30_holes())
        wy = (ys[0] + CRADLE_WIN, ys[-1] - CRADLE_WIN)
        win = sbox(x0 + 0.007, wy[0], xb - 0.006, wy[1]).buffer(-0.004, join_style=1).buffer(0.004, join_style=1)
        cut_p = extrude_cs(win, 0.08, (0.0, 0.0, lo[2] - 0.04), (1.0, 0.0, 0.0), (0.0, 1.0, 0.0))
        bw = sbox(wy[0], lo[2] + 0.004, wy[1], hi[2] - 0.004).buffer(-0.004, join_style=1).buffer(0.004, join_style=1)
        cut_b = extrude_cs(bw, 0.02, (xb - 0.01, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0))
        return D(U([top, bot, base]), [cut_p, cut_b])

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
        """Sandwich cavity forward of 0.80 c (core-filled trailing-edge closeout aft of it), minus the two monolithic
        zones (third root lug, skid) where the skin laminate is built up solid."""
        V, C = self.V, self.C

        def fn(poly, ch, eta):
            return SG.largest(poly.buffer(-(C.t_skin + extra), join_style=2).intersection(
                sbox(-1.0, -1.0, V_CAVITY_XC * ch, 1.0)))
        return self.m(("cav", extra), lambda: pieces_above(D(V.loft(V.e0 - 0.01, V.e1 - C.t_skin - extra, fn,
                                                                     step=0.01), self.zones(extra))))

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
        """Monolithic build-ups of the skin laminate: round root lug 3 and under the skid strap."""
        x3 = self.fit_x(2)
        z3 = self.fork_bolt_z(2)[-1] - 0.016
        V = self.V
        ztip = float(V.frame(V.e1)[0][2])
        return [box3((x3 - 0.030 - grow, -0.05, z3 - grow), (x3 + 0.030 + grow, 0.05, 0.0)),
                box3((SKID["x"][0] - 0.020 - grow, -0.05, -0.5), (4.3, 0.05, ztip + 0.045 + grow))]

    def skin_raw(self) -> G.Mesh:
        C = self.C
        return pieces_above(D(self.outer(), [self.cavity(), C.body(TRIM, 3.40, 4.2), self.slot3()]))

    def root_rib(self) -> G.Mesh:
        C = self.C
        band = D(C.body(TRIM + C.t_rib, 3.45, 4.1), [C.body(TRIM, 3.43, 4.1)])
        return pieces_above(D(I(self.cavity(BOND), band), self.rib_holes()))

    def web(self) -> G.Mesh:
        C = self.C
        w = I(self.cavity(BOND), box3((3.4, -0.5 * V_WEB_T, -0.4), (4.3, 0.5 * V_WEB_T, 0.0)))
        return pieces_above(D(w, [C.body(TRIM + C.t_rib + BOND, 3.43, 4.1)] + [self.crotch_clear(i) for i in (0, 1)]))

    def cheek_env(self) -> G.Mesh:
        """Prism over the region where the cavity is wide enough for full 3 mm cheeks (half width >= 4.7 mm)."""
        V, C = self.V, self.C
        need = C.t_skin + 2 * BOND + 0.5 * V_WEB_T + BOND + V_CHEEK_T

        def fn(poly, ch, eta):
            q = poly.buffer(-need, join_style=2).intersection(sbox(-1.0, -1.0, V_CAVITY_XC * ch - 0.001, 1.0))
            x0, _y0, x1, _y1 = q.bounds
            return sbox(x0, -0.05, x1, 0.05)
        return self.m("cheek_env", lambda: V.loft(V.e0 - 0.01, V.e1 - 0.02, fn, step=0.01))

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
            w0 = 0.5 * V_WEB_T + BOND
            crotch = box3((x - V_LUG_HX, -(w0 + V_CHEEK_T), zc - 0.006), (x + V_LUG_HX, w0 + V_CHEEK_T, zc))
            zb = self.fork_bolts(i)[0][1] - 0.010
            xe = self.fork_bolts(i)[-1][0] + 0.0105
            cheeks = [I(box3((x - V_LUG_HX, s0, zb), (max(x + V_LUG_HX, xe), s1, zc - 0.006 + OV)), self.cheek_env())
                      for s0, s1 in ((w0, w0 + V_CHEEK_T), (-(w0 + V_CHEEK_T), -w0))]
            return U([tongue, crotch] + cheeks)
        zb = self.fork_bolts(i)[-1][1] - 0.013
        return box3((x - V_LUG_HX, -hy, zb), (x + V_LUG_HX, hy, ztop))

    def fork_bolts(self, i):
        """(x, z) of the two M5 bolts fixing root fitting i to the web (1, 2: horizontal pair under the crotch) or to
        the lug zone of the skin (3: vertical pair)."""
        x = self.fit_x(i)
        if i < 2:
            z = self.z_crotch(i) - 0.006 - 0.013 - BOND
            dx = V_FORK_DX[i]
            return [(x + dx, z), (x + dx + 0.016, z)]
        z0 = self.z_oml(x) - TRIM - 0.0135
        return [(x, z0), (x, z0 - 0.016)]

    def fork_bolt_z(self, i):
        return [z for _x, z in self.fork_bolts(i)]

    def slot3(self) -> G.Mesh:
        x = self.fit_x(2)
        return box3((x - V_LUG_HX - BOND, -V_SLOT[0], self.fork_bolt_z(2)[-1] - 0.013 - BOND),
                    (x + V_LUG_HX + BOND, V_SLOT[0], 0.0))

    def rib_holes(self) -> list:
        out = []
        for i in (0, 1, 2):
            x = self.fit_x(i)
            hw = (0.5 * V_WEB_T + BOND + V_CHEEK_T + BOND) if i < 2 else V_SLOT[0]
            out.append(box3((x - V_LUG_HX - BOND, -hw, -0.4), (x + V_LUG_HX + 0.0105 + BOND, hw, 0.0)))
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
        return [((x, -0.05, z), (0.0, 1.0, 0.0)) for x in (4.074, 4.094)]


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
                                                              f_oml(C.t_skin + extra), n=STUB_N))

    def stub_skin(self) -> G.Mesh:
        C = self.C
        outer = C.stub.loft(C.stub.e0, C.stub.e1, f_oml(), n=STUB_N)
        return pieces_above(D(outer, [self.stub_inner(), C.body(TRIM)]))

    def sleeve_clear(self, r=0.0245 + BOND, y0=None, y1=None):
        g = self.g
        return cyl(r, g.axis_pt(y0 if y0 is not None else g.flange[0] - 0.01), g.axis_pt(y1 if y1 is not None else 0.40))

    def stub_root_rib(self) -> list[G.Mesh]:
        C = self.C
        band = D(C.body(TRIM + C.t_rib, 3.38, 4.0), [C.body(TRIM, 3.36, 4.0)])
        rib = D(I(self.stub_inner(BOND), band), [self.sleeve_clear(), self.stub_fitting(BOND)])
        return _split(rib)

    def stub_tip_rib(self) -> list[G.Mesh]:
        C, g = self.C, self.g
        e1 = C.stub.e1
        e0 = e1 - C.t_rib
        reg = C.stub.inner_common(e0, e1, C.t_skin + 3 * BOND)
        hole2 = C.stub.to2d(e0, [g.axis_pt(e0)])[0]
        reg = reg.difference(Point(hole2[0], hole2[1]).buffer(0.0245 + BOND, 64))
        return [C.stub.plate(e0, e1, p) for p in sorted(polys(reg), key=lambda q: -q.area)]

    # ------------------------------------------------------------ stub front-spar root fitting (F-STUB-FRONT)
    def sf(self) -> dict:
        """Stub front-fitting interface from layout F-STUB-FRONT: clevis slot (ears SF_EAR either side, floor
        SF_FLOOR_DY outboard of the frame bolts), lug bolt stations, fitting plate x range and the spar web line."""
        C = self.C
        f = C.fit["F-STUB-FRONT"]
        lo, hi = np.asarray(f["box"][0], float), np.asarray(f["box"][1], float)
        y_floor = max(float(b["point"][1]) for b in f["bolts"] if b["group"] == "frame") + SF_FLOOR_DY
        bz = sorted(float(b["point"][2]) for b in f["bolts"] if b["group"] == "stub clevis")
        slot = (float(lo[0]) + SF_EAR, float(hi[0]) - SF_EAR)
        xa, xb = slot[0] + BOND, slot[1] - BOND
        zc = float(C.T["stabilator_stub"]["params"]["z"])
        return dict(slot=slot, y_floor=y_floor, y_ear=float(hi[1]), z_ear=(float(lo[2]), float(hi[2])), bz=bz,
                    y_lug=float(hi[1]) - 2 * SF_D, xa=xa, xb=xb, x_web=xb + BOND + 0.5 * T_WEB, zc=zc,
                    fl_bz=(zc - SF_FL_DZ, zc + SF_FL_DZ))

    def stub_fitting(self, grow=0.0) -> G.Mesh:
        """7075 plate fitting (machined): 7.8 mm lug in the CH-098 clevis slot (end face on the slot floor), neck
        through the body side and the stub root rib, 5 mm flange on the forward face of the stub front-spar web."""
        d = self.sf()
        zlo, zhi = d["bz"][0] - SF_LUG_E, d["bz"][1] + SF_LUG_E
        lug = sbox(d["y_floor"] - grow, zlo - grow, SF_NECK_Y1 + grow, zhi + grow)
        fl = sbox(SF_NECK_Y1 - 0.006 - grow, d["zc"] - SF_FL_HZ - grow, SF_FL_Y + SF_FL_END + grow,
                  d["zc"] + SF_FL_HZ + grow)
        fl = fl.buffer(-0.004, join_style=1).buffer(0.004, join_style=1)      # 4 mm corner radii
        a = extrude_cs(lug, d["xb"] - d["xa"] + 2 * grow, (d["xa"] - grow, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0))
        b = extrude_cs(fl, SF_FL_T + 2 * grow, (d["xb"] - SF_FL_T - grow, 0.0, 0.0), (0.0, 1.0, 0.0),
                       (0.0, 0.0, 1.0))
        return U([a, b])

    def stub_front_spar(self) -> G.Mesh:
        C = self.C
        e1 = C.stub.e1 - C.t_rib - BOND
        x_web = self.sf()["x_web"]
        sp = C.stub.channel(C.stub.e0 - 0.005, e1, lambda eta: x_web - C.stub.frame(eta)[0][0], C.t_skin + BOND,
                            aft=True, inner=self.stub_inner(BOND))
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
        reg = C.stab.inner_common(e0, e1, C.t_skin + 3 * BOND)
        c2 = C.stab.to2d(e0, [g.axis_pt(e0)])[0]
        reg = reg.difference(Point(c2[0], c2[1]).buffer(g.r_sock + BOND, 64))
        return C.stab.plate(e0, e1, reg)

    def _fwd_rib(self, e0, e1) -> G.Mesh:
        C = self.C
        reg = C.stab.inner_common(e0, e1, C.t_skin + 3 * BOND)
        xw = min(self.spar_x(e0), self.spar_x(e1)) - C.stab.frame(e0)[0][0] - 0.5 * T_WEB - BOND
        reg = SG.largest(reg.intersection(sbox(-1.0, -1.0, xw, 1.0)))
        return C.stab.plate(e0, e1, reg)

    def stab_socket_rib(self) -> G.Mesh:
        return self._fwd_rib(*self.g.sock_rib)

    def stab_mid_rib(self) -> G.Mesh:
        e = float(self.C.T["stabilator"]["sections"][1]["y"])
        return self._fwd_rib(e, e + self.C.t_rib)


def hinge_bracket(axis_p, axis_dir, mount_normal, hs: "A.HingeSpec", reach: float, fixed: bool) -> dict:
    """Clevis (fixed) or tongue (moving) hinge bracket with the geometry of actuation.hinge_bracket_fixed / _moving
    (lug profile, base flange, bolt pattern), built with the manifold cross-section extruder (robust with the pin
    hole). ``mount_normal`` points from the spar web toward the hinge line, ``reach`` = web face to hinge axis."""
    a = unit(axis_dir)
    n = unit(np.asarray(mount_normal, float) - np.dot(mount_normal, a) * a)
    e3 = np.cross(n, a)
    p = np.asarray(axis_p, float)
    web_pt = p - reach * n
    prof = A._lug_profile(hs.lug_r, hs.pin_d / 2 + hs.pin_clear, reach - 0.5 * hs.base_t,
                          min(hs.base_h, 2.2 * hs.lug_r))
    offs = (-(hs.lug_t / 2 + hs.gap + hs.lug_t / 2), hs.lug_t / 2 + hs.gap + hs.lug_t / 2) if fixed else (0.0,)
    lugs = [extrude_cs(prof, hs.lug_t, p + (o - 0.5 * hs.lug_t) * a, e3, n) for o in offs]
    span = 3 * hs.lug_t + 2 * hs.gap
    bw = max(span, hs.base_w)
    base = obox(web_pt + 0.5 * hs.base_t * n, [n, a, e3], [0.5 * hs.base_t, 0.5 * bw, 0.5 * hs.base_h])
    bolts = [(web_pt + sg * (0.5 * bw - 2.0 * hs.bolt_d_mm * 1e-3) * a + hs.base_t * n, -n) for sg in (-1, 1)]
    return {"mesh": U(lugs + [base]), "bolts": bolts, "web_point": web_pt}


def nut_strip(pts, axis, inside: G.Mesh, half_w=0.009, margin=0.010, t_min=0.0015, el=None) -> G.Mesh:
    """7075 nutplate strip bonded under a curved skin: conformal top on ``inside`` (the region inside the skin, a bond
    line off it), flat bottom normal to the screw axis, so the nutplates seat flat. ``pts`` are the screw points on
    the outer surface, ``axis`` points into the structure."""
    a = unit(axis)
    P = [np.asarray(p, float) for p in pts]
    if el is None:
        el = P[1] - P[0] if len(P) > 1 else np.cross(a, (0.0, 0.0, 1.0))
    el = unit(np.asarray(el, float) - (np.asarray(el, float) @ a) * a)
    ew = np.cross(a, el)
    man = inside.to_manifold()
    s_all = [(q - P[0]) @ el for q in P]
    l0, l1 = min(s_all) - margin, max(s_all) + margin
    deep = 0.0
    for s_ in np.linspace(l0, l1, 7):
        for w_ in np.linspace(-half_w, half_w, 5):
            o = P[0] + s_ * el + w_ * ew - 0.02 * a
            h = G.ray_hits(man, o, o + 0.08 * a)
            if len(h):
                deep = max(deep, float(h[0]) - 0.02)
    d1 = deep + t_min
    c = P[0] + 0.5 * (l0 + l1) * el + 0.5 * (d1 - 0.03) * a
    blk = obox(c, [el, ew, a], [0.5 * (l1 - l0), half_w, 0.5 * (d1 + 0.03)])
    return largest_piece(I(blk, inside))


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
        reg = F.inner_common(e0, e1, self.ti + 2 * BOND)
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
        s_ = 0.40 * F.chord(eta)
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
        capin = F.loft(e0, e1, lambda p, ch, eta: SG.largest(
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
        return self.m(("clevis", i), lambda: hinge_bracket(self.axis_at(self.hinge_eta(i)), self.h_a, self.cp,
                                                           HINGE, reach, True))

    def tongue(self, i) -> dict:
        reach = RUDDER_SPAR_E * self.aw - BOND
        return self.m(("tongue", i), lambda: hinge_bracket(self.axis_at(self.hinge_eta(i)), self.h_a, -self.cp,
                                                           HINGE, reach, False))

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

    # ---------------------------------------------------------------- mid-surface outlines (edge distance on thin
    # curved laminates: the planar ray test of checks.py would see the curvature of a 0.6-1 mm skin as an edge)
    def rudder_skin_outline(self) -> list:
        """Boundaries of the three clevis notches in the rudder nose skin (the only free edges of the closed skin)."""
        out = []
        for i in range(3):
            eta = self.hinge_eta(i)
            p = self.axis_at(eta)
            ef, ea = -self.cp, self.h_a
            et = np.cross(ef, ea)
            rho = self.r_nose(eta) - 0.5 * T_SKIN_CS
            half_t = 0.5 * min(HINGE.base_h, 2.2 * HINGE.lug_r) + 0.001
            ah = HINGE.lug_t * 1.5 + HINGE.gap + 0.0015 + 0.0015
            tm = math.radians(29.0) + math.asin(min(1.0, half_t / rho))
            th = np.linspace(-tm, tm, 17)
            arc = [p + rho * (math.cos(t) * ef + math.sin(t) * et) for t in th]
            ring_ = [q + ah * ea for q in arc] + [q - ah * ea for q in arc[::-1]]
            out.append(np.asarray(ring_ + [ring_[0]]))
        return out

    def rudder_spar_outline(self) -> list:
        """Free edges of the rudder-spar mid-surface: cap aft edges along the span and the C mid-line at both ends."""
        F = self.F
        tc, wc = RUDDER_CAP
        e0, e1 = self.e_r0 + T_SKIN_CS + BOND, self.e_r1 - T_SKIN_CS - BOND

        def pts(eta):
            sw = self.s_rspar(eta)
            sa = sw - 0.5 * T_WEB + wc
            tu, tl = SG._span_t(F.poly(eta), sa)
            dt = T_SKIN_CS + BOND + 0.5 * tc
            return F.p3(eta, sa, tu - dt), F.p3(eta, sw, tu - dt), F.p3(eta, sw, tl + dt), F.p3(eta, sa, tl + dt)
        etas = np.linspace(e0, e1, 25)
        P = [pts(e) for e in etas]
        return [np.asarray([q[0] for q in P]), np.asarray([q[3] for q in P]), np.asarray(list(P[0])),
                np.asarray(list(P[-1]))]

    def hatch_cover_outline(self) -> list:
        F = self.F
        e0, e1 = HATCH_ETA[0] - 0.0225, HATCH_ETA[1] + 0.0225
        x0 = HATCH_X0 - 0.003

        def on(eta, x):
            o = F.frame(eta)[0]
            tu, _tl = SG._span_t(F.poly(eta), x - o[0])
            return F.p3(eta, x - o[0], tu - 0.5 * HATCH_T)
        corners = [(e0, x0), (e0, self.cap_front_x(e0) + 0.002), (e1, self.cap_front_x(e1) + 0.002), (e1, x0)]
        ring_ = []
        for (ea_, xa), (eb, xb) in zip(corners, corners[1:] + corners[:1]):
            for t in np.linspace(0.0, 1.0, 20, endpoint=False):
                ring_.append(on(ea_ + t * (eb - ea_), xa + t * (xb - xa)))
        ring_.append(ring_[0])
        return [np.asarray(ring_)]

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

    def horn_sweep(self, grow=0.0015, aft=0.010) -> G.Mesh:
        """Volume swept over the rudder travel by the forward part of the horn (ball, plate within ``aft`` of the hinge
        line) and by the pushrod's horn-end socket, grown by ``grow`` (hulls of neighbouring states): the horn
        clearance notch cut from the cove lip of the fixed fin skin."""
        def build():
            lk = self.linkage()
            F = self.F
            hm = self.m("horn_mesh", self.horn)
            e0, e1 = self.horn_eta()
            fwd = I(hm, F.slab(e0 - 0.05, e1 + 0.05, self.s_hinge, -0.10, aft))   # clipped: no long plate facets
            keep = list(fwd.V)
            n_s = 24
            sph = [lk.B0 + (BALL[2] + grow) * np.array([math.cos(a) * math.cos(b), math.sin(a) * math.cos(b),
                                                         math.sin(b)])
                   for a in np.linspace(0, 2 * math.pi, n_s, endpoint=False) for b in np.linspace(-1.4, 1.4, 7)]
            P = np.vstack([np.asarray(keep)] + [np.asarray(keep) + grow * d for d in
                                                np.vstack([np.eye(3), -np.eye(3)])] + [np.asarray(sph)])
            ds = [d for d, *_ in self.rudder_states()]
            hulls = []
            for d1, d2 in zip(ds, ds[1:]):
                Q = np.vstack([G.rotate_about(P, self.h_o, self.h_a, d1), G.rotate_about(P, self.h_o, self.h_a, d2)])
                hulls.append(G.hull(Q))
            return U(hulls)
        return self.m(("hornsweep", grow, aft), build)

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
        """Horn pad chord range: on the skin aft of the rudder-spar cap (screws through skin + core pocket nuts)."""
        s0 = self.s_hinge(eta) + RUDDER_SPAR_E + RUDDER_CAP[1] + 0.0015
        return s0, s0 + 0.028

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
        e0, e1 = self.horn_eta()
        pts = []
        for eta in (e0 - 0.004, e1 + 0.004):
            s_ = 0.5 * sum(self.horn_pad_s(eta))
            pts.append((self.oml_point(eta, s_), self.oml_normal(eta, s_)))
        n = unit(pts[0][1] + pts[1][1])                 # common normal: flat backing plate under both screws
        return [(p + 0.0025 * n, -n) for p, _n in pts]

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
        oc = F.frame(e1 - DA26_FLANGE[0])[0][0]
        hx, ht = DA26["holes"]
        return [(F.p3(e1 - DA26_FLANGE[0], xc + sx * 0.5 * hx - oc, st * 0.5 * ht), F.w) for sx in (-1, 1)
                for st in (-1, 1)]

    def servo_rib_eta(self):
        e0 = DA26_ETA0 + DA26["H"] + BOND
        return e0, e0 + SERVO_RIB_T

    def servo_rib(self) -> G.Mesh:
        """7075 servo rib with four 6 mm bosses on top (M3 threads + helical inserts for the DA 26 flange screws)."""
        F = self.F
        e0, e1 = self.servo_rib_eta()
        bosses = [cyl(0.0065, pt + (e1 - e0 + DA26_FLANGE[0] + BOND - OV) * F.w,
                      pt + (e1 - e0 + DA26_FLANGE[0] + BOND + 0.006) * F.w, n=32) for pt, _ax in self.da26_holes()]
        return U([self.box_rib(e0, e1)] + bosses)

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

    def hatch_strip(self, k) -> G.Mesh:
        """Nutplate strip under the hatch land: k = 0 lower screw pair, 1 upper pair."""
        sc = self.hatch_screws()[2 * k:2 * k + 2]
        return nut_strip([p for p, _a in sc], sc[0][1], self.inner(BOND), half_w=0.009, margin=0.010)

    def cap_strip(self, k) -> G.Mesh:
        p, a = self.cap_screws()[k]
        return nut_strip([p], a, self.inner(BOND), half_w=0.008, margin=0.008, el=self.F.c)

    def horn_plate(self) -> G.Mesh:
        """7075 backing plate under the rudder skin at the horn screws (nyloc nuts on its flat face)."""
        hb = self.horn_bolts()
        return nut_strip([p for p, _a in hb], hb[0][1], self.rudder_inner(BOND), half_w=0.010, margin=0.008)

    def oml_point(self, eta, s) -> np.ndarray:
        """Point of the inboard (+u) OML face at span coordinate eta, section chord coordinate s."""
        tu, _tl = SG._span_t(self.F.poly(eta), s)
        return self.F.p3(eta, s, tu)

    def oml_normal(self, eta, s, h=0.003) -> np.ndarray:
        """Outward unit normal of the inboard OML face at (eta, s): screw axes are laid along it so the flat head and
        the nutplate seat square on the curved skin."""
        ps = self.oml_point(eta, s + h) - self.oml_point(eta, s - h)
        pe = self.oml_point(eta + h, s) - self.oml_point(eta - h, s)
        n = unit(np.cross(ps, pe))
        return n if n @ self.F.u > 0 else -n

    def hatch_screws(self):
        """(point on the cover outer surface, axis into the fin) of the 4 x M4 cover screws; each pair shares the
        mean surface normal (one flat nutplate strip per pair)."""
        F = self.F
        out = []
        for eta, xs in zip((HATCH_ETA[0] - 0.011, HATCH_ETA[1] + 0.011), HATCH_SCREW_X):
            o = F.frame(eta)[0]
            ax = -unit(sum(self.oml_normal(eta, x - o[0]) for x in xs))
            out += [(self.oml_point(eta, x - o[0]), ax) for x in xs]
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

    @staticmethod
    def fitting_grow(m: G.Mesh, g: float) -> G.Mesh:
        """Mesh grown by ~g (Minkowski with an octahedron, convex hull of the whole mesh: for convex-ish plates)."""
        P = np.vstack([m.V + g * d for d in np.vstack([np.eye(3), -np.eye(3)])])
        return G.hull(P)

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
        for e in ((eb + 0.016, eb + 0.046) if front else (eb + 0.022, eb + 0.050)):
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


# =====================================================================================================================
# registration
# =====================================================================================================================
M7075, MTI, MSTEEL = "al_7075_t651_plate", "ti_6al_4v_annealed_sheet", "steel_4130_n"
CFRP_PW, CFRP_UD, GFRP, CORE = "cfrp_pw_mtm45_as4", "cfrp_ud_mtm45_as4", "gfrp_7781_mtm45", "core_rohacell_51wf"
P_CNC, P_PREPREG, P_SLS, P_BUY = "cnc_milling_metal", "prepreg_ooa_vacbag", "sls_pa12", "purchased"
G129 = "12.9"
TI = "Ti-6Al-4V"          # bolts in composite stacks and secondary joints (galvanically compatible with CFRP)
EX_FIN, EX_RUD, EX_ACT = (0.0, 0.10, 0.28), (0.12, 0.10, 0.28), (0.0, 0.22, 0.22)
EX_STUB, EX_STAB, EX_SPIN, EX_DRV, EX_VEN = (0.0, 0.18, 0.0), (0.0, 0.40, 0.0), (0.0, 0.28, 0.0), (-0.15, 0.05, 0.10), \
    (0.0, 0.0, -0.25)


def _dedupe_holes(p: Part) -> None:
    """Drop repeated identical cylinder cutters (coincident boolean cutters leave slivers in the result)."""
    seen, keep = set(), []
    for h in p.holes:
        if isinstance(h, G.Mesh):
            keep.append(h)
            continue
        k = tuple(np.round(np.r_[h[0], h[1], h[2]], 9))
        if k not in seen:
            seen.add(k)
            keep.append(h)
    p.holes[:] = keep


def _port(pid: str) -> str:
    return pid[:-2] + "-L" if pid.endswith("-R") else pid


def _horner(var: str, coef) -> str:
    """Blender simple expression of sum_k coef[k] * var^(k+1) (no constant term), Horner form."""
    expr = f"{coef[-1]:.6g}"
    for c in coef[-2::-1]:
        expr = f"({c:.6g}+{var}*{expr})"
    return f"{var}*{expr}"


def _fit(x, y, deg=5) -> np.ndarray:
    """Least-squares polynomial through the origin: y = sum c_k x^(k+1), k = 0..deg-1."""
    X = np.column_stack([np.asarray(x, float) ** (k + 1) for k in range(deg)])
    return np.linalg.lstsq(X, np.asarray(y, float), rcond=None)[0]


class _Fast:
    """Fastener factory: material intervals measured along the fastener line on the parts' base geometry (several
    intervals per part allowed, e.g. both ears of a clevis), stack checked for gaps, then joints.bolt / joints.pin.
    Problems are collected and raised at the end of register()."""

    def __init__(self, C: Ctx):
        self.C = C
        self.reg = C.reg
        self.n = {}
        self.problems = []
        self._man = {}

    def man(self, pid):
        if pid not in self._man:
            self._man[pid] = self.reg.parts[pid].base_mesh.to_manifold()
        return self._man[pid]

    def fid(self, owner):
        self.n[owner] = self.n.get(owner, 0) + 1
        return f"{owner}-B{self.n[owner]}"

    def intervals(self, pids, point, axis, r, window):
        """Material intervals along the line (four probes at radius r, median; r = 0: the centre line itself, exact on
        curved stacks such as a bolt across a tube - the base meshes have no holes yet)."""
        from ..core.geom import ray_hits
        a = unit(axis)
        c = np.asarray(point, float)
        e1, e2 = J._perp(a)
        lo, hi = window
        out = []
        for pid in dict.fromkeys(pids):
            probes = []
            for d in ((e1, -e1, e2, -e2) if r > 0 else (0.0 * e1,)):
                o = c + r * d
                h = ray_hits(self.man(pid), o + (lo - 0.05) * a, o + (hi + 0.05) * a) + (lo - 0.05)
                probes.append([(h[i], h[i + 1]) for i in range(0, len(h) - 1, 2) if h[i + 1] > lo and h[i] < hi])
            k = max(len(p) for p in probes)
            if k == 0:
                raise ValueError(f"fastener line misses {pid}")
            good = [p for p in probes if len(p) == k]
            for i in range(k):
                out.append((pid, float(np.median([p[i][0] for p in good])), float(np.median([p[i][1] for p in good]))))
        return sorted(out, key=lambda q: q[1])

    def step_of(self, pids) -> int:
        return max(int(self.reg.parts[p].step) for p in pids if p)

    def bolt(self, size, point, axis, pids, *, window=(-0.05, 0.05), max_gap=0.0005, owner=None, label="",
             on_axis=False, pocket=None, spot=(None, None), **kw):
        try:
            from .fastener_catalog import clearance
            a = unit(axis)
            iv = self.intervals(pids, point, a, 0.0 if on_axis else 0.5 * clearance(size) * 1.6, window)
            for (p0, _s0, e0), (p1, s1, _e1) in zip(iv, iv[1:]):
                if s1 - e0 > max_gap:
                    raise ValueError(f"gap {(s1 - e0) * 1000:.2f} mm between {p0} and {p1}")
                if s1 - e0 < -0.0003:
                    raise ValueError(f"overlap {(e0 - s1) * 1000:.2f} mm between {p0} and {p1}")
            head = np.asarray(point, float) + iv[0][1] * a
            stack = [(pid, e_ - s_) for pid, s_, e_ in iv]
            total = iv[-1][2] - iv[0][1]
            stack[-1] = (stack[-1][0], stack[-1][1] + (total - sum(t for _p, t in stack)))
            own = owner or next((p for p, _t in stack if p.startswith(("YK250-TL", "YK250-FC"))), stack[0][0])
            fid = self.fid(own)
            kw.setdefault("step", self.step_of(list(pids) + [kw.get("insert_part"), kw.get("tapped_part")]))
            fst = J.bolt(self.reg, fid, size, head, a, stack, owner=own, **kw)
            for pid in dict.fromkeys(p for p, _t in stack):         # a part met twice in the stack (both ears of a
                _dedupe_holes(self.reg.parts[pid])                  # clevis, a shoe round a strap): one bore
            q = fst.position + fst.grip * fst.axis
            if pocket is not None:                                  # nut pocket in a neighbouring part (core)
                self.reg.parts[pocket[0]].add_hole(q - 0.0002 * fst.axis, q + pocket[1] * fst.axis, pocket[2])
            from . import fastener_catalog as FC_
            if spot[0]:                                             # machined flat seat under the head
                dk = (FC_.ISO7380 if "7380" in kw.get("head", "") else FC_.ISO4762)[size][0]
                self.reg.parts[spot[0]].add_hole(fst.position - 0.015 * fst.axis, fst.position, 0.5 * dk + 0.0005)
            if spot[1]:                                             # flat seat under the nut / washer
                rn = max(FC_.ISO7040[size][0] / math.sqrt(3.0), FC_.ISO7089[size][1] / 2)
                self.reg.parts[spot[1]].add_hole(q, q + 0.015 * fst.axis, rn + 0.0005)
            return fst
        except Exception as exc:                                   # collected, raised at the end of register()
            self.problems.append((label or f"M{size} at {np.round(point, 4).tolist()}", str(exc)))
            return None

    def pin(self, d, point, axis, stack, *, owner, label="", **kw):
        try:
            fid = self.fid(owner)
            kw.setdefault("step", self.step_of([p for p, _t in stack]))
            return J.pin(self.reg, fid, d, point, axis, stack, owner=owner, **kw)
        except Exception as exc:
            self.problems.append((label or f"pin at {np.round(point, 4).tolist()}", str(exc)))
            return None

    def sym(self, size, point, axis, pids, **kw):
        """Starboard joint and its port mirror image."""
        self.bolt(size, point, axis, pids, **kw)
        mp = np.array([1.0, -1.0, 1.0])
        kw2 = dict(kw)
        for k in ("owner", "insert_part", "tapped_part"):
            if kw2.get(k):
                kw2[k] = _port(kw2[k])
        if kw2.get("pocket"):
            kw2["pocket"] = (_port(kw2["pocket"][0]),) + tuple(kw2["pocket"][1:])
        if kw2.get("spot"):
            kw2["spot"] = tuple(_port(x) if x else None for x in kw2["spot"])
        self.bolt(size, np.asarray(point, float) * mp, np.asarray(axis, float) * mp, [_port(p) for p in pids], **kw2)

    def sym_pin(self, d, point, axis, stack, *, owner, **kw):
        self.pin(d, point, axis, stack, owner=owner, **kw)
        mp = np.array([1.0, -1.0, 1.0])
        self.pin(d, np.asarray(point, float) * mp, np.asarray(axis, float) * mp,
                 [(_port(p), t) for p, t in stack], owner=_port(owner), **kw)


class _TailReg:
    """Part, joint and fastener registration of the tail (starboard built, port mirrored, ventral on the centre line)."""

    def __init__(self, C: Ctx):
        self.C = C
        self.f = FinGeo(C)
        self.g = StabGeo(C)
        self.sp = StubPanelGeo(C, self.g)
        self.sd = StabDrive(C, self.g)
        self.v = VenGeo(C)
        self.jmap: dict[str, str] = {}
        self.ids: dict[str, str] = {}

    def P(self, key) -> str:
        return self.ids[key]

    def add(self, key, num, side, group, name, name_tr, material, process, fn, **kw):
        pid = self.C.pid(num, side, group)
        self.ids[key] = pid
        self.C.add(pid, name, name_tr, group, material, process, fn, **kw)
        return pid

    # ------------------------------------------------------------------------------------------------------- joints
    def joints(self):
        C, f, sd = self.C, self.f, self.sd
        reg = C.reg
        # rudder: layout joint + coupled four-bar (arm about the DA 26 shaft, rod about the arm-end ball)
        jr = C.mech["rudder_R"]
        sc = float(jr["scale"])
        lk = f.linkage()
        st = f.rudder_states()
        rd = np.array([d / sc for d, *_ in st])                       # rudder_deg of each state
        self.rud = dict(lk=lk, states=st, rd=rd)
        reg.add_joint(Joint("rudder_R", "revolute", jr["origin"], jr["axis"], float(jr["lo"]), float(jr["hi"]), 0.0,
                            prop=jr["prop"], scale=sc, expr=f"{sc!r}*{jr['prop']}", notes=jr.get("notes", "")))
        vals = {"rudder_arm_R": [s_[1] for s_ in st], "rudder_rod1_R": [s_[2] for s_ in st],
                "rudder_rod2_R": [s_[3] for s_ in st]}
        geo = {"rudder_arm_R": (f.Ssh, -C.fin.w, None), "rudder_rod1_R": (lk.A0, lk.e1, "rudder_arm_R"),
               "rudder_rod2_R": (lk.A0, lk.e2, "rudder_rod1_R")}
        self.rud_coef = {}
        for name, (o, a, par) in geo.items():
            v = np.asarray(vals[name])
            coef = _fit(rd, v)
            self.rud_coef[name] = coef
            reg.add_joint(Joint(name, "revolute", o, a, float(v.min()) - 0.02, float(v.max()) + 0.02, 0.0, parent=par,
                                expr=_horner(jr["prop"], coef),
                                notes="coupled to rudder_R (DA 26 four-bar, Linkage.solve); sequence rudder_R"))
        # stabilator: layout joint + planar four-bar (arm about the DA 30 shaft, rod about the arm-end ball)
        js = C.mech["stabilator_R"]
        ss = float(js["scale"])
        lks = sd.lk
        sts = lks.sweep(float(js["lo"]), float(js["hi"]), n=13)
        ed = np.array([d / ss for d, *_ in sts])
        self.stab = dict(states=sts, ed=ed)
        reg.add_joint(Joint("stabilator_R", "revolute", js["origin"], js["axis"], float(js["lo"]), float(js["hi"]), 0.0,
                            prop=js["prop"], scale=ss, expr=f"{ss!r}*{js['prop']}", notes=js.get("notes", "")))
        th = np.array([s_[1] for s_ in sts])
        p2 = np.array([s_[3] for s_ in sts])
        if max(abs(s_[2]) for s_ in sts) > 1e-6:
            raise ValueError("stabilator four-bar is not planar")
        self.stab_coef = {"stab_arm_R": _fit(ed, th), "stab_rod_R": _fit(ed, p2)}
        reg.add_joint(Joint("stab_arm_R", "revolute", sd.S, (0.0, 1.0, 0.0), float(th.min()) - 0.02,
                            float(th.max()) + 0.02, 0.0, expr=_horner(js["prop"], self.stab_coef["stab_arm_R"]),
                            notes="coupled to stabilator_R (DA 30 four-bar, 2.5:1 at neutral); sequence stabilator_R"))
        reg.add_joint(Joint("stab_rod_R", "revolute", lks.A0, lks.e2, float(p2.min()) - 0.02, float(p2.max()) + 0.02,
                            0.0, parent="stab_arm_R", expr=_horner(js["prop"], self.stab_coef["stab_rod_R"]),
                            notes="pushrod about the arm-end ball (planar)"))
        for n_ in ("rudder", "rudder_arm", "rudder_rod1", "rudder_rod2", "stabilator", "stab_arm", "stab_rod"):
            self.jmap[f"{n_}_R"] = f"{n_}_L"

    def port_joints(self):
        C = self.C
        reg = C.reg
        mp = np.array([1.0, -1.0, 1.0])
        for name in ("rudder_L", "stabilator_L"):
            j = C.mech[name]
            reg.add_joint(Joint(name, "revolute", j["origin"], j["axis"], float(j["lo"]), float(j["hi"]), 0.0,
                                prop=j["prop"], scale=float(j["scale"]), expr=f"{float(j['scale'])!r}*{j['prop']}",
                                notes=j.get("notes", "")))
            # the port twin of the four-bar uses mirrored axes: port value = -(starboard value at the mirrored state)
        for name in ("rudder_arm_R", "rudder_rod1_R", "rudder_rod2_R", "stab_arm_R", "stab_rod_R"):
            j = reg.joints[name]
            coef = self.rud_coef.get(name, self.stab_coef.get(name))
            k = np.arange(1, len(coef) + 1)
            if name.startswith("rudder"):
                # rudder_L (mirrored axis) at rudder_deg x = mirror of starboard at -x: value = -P(-x)
                cl = coef * (-1.0) ** (k + 1)
                prop = C.mech["rudder_R"]["prop"]
            else:
                # stabilator_L (axis +y, = mirror of starboard at the same value): value = -P(x)
                cl = -coef
                prop = C.mech["stabilator_R"]["prop"]
            reg.add_joint(Joint(self.jmap[name], "revolute", j.origin * mp, j.axis * mp, -j.hi, -j.lo, 0.0,
                                parent=self.jmap.get(j.parent) if j.parent else None, expr=_horner(prop, cl),
                                notes=j.notes.replace("_R", "_L")))

    def sequences(self):
        reg = self.C.reg
        st = self.rud["states"]
        reg.add_sequence("rudder_R", [{"rudder_R": d, "rudder_arm_R": t, "rudder_rod1_R": a, "rudder_rod2_R": b}
                                      for d, t, a, b in st])
        reg.add_sequence("rudder_L", [{"rudder_L": -d, "rudder_arm_L": -t, "rudder_rod1_L": -a, "rudder_rod2_L": -b}
                                      for d, t, a, b in st])
        sts = self.stab["states"]
        reg.add_sequence("stabilator_R", [{"stabilator_R": d, "stab_arm_R": t, "stab_rod_R": b} for d, t, _a, b in sts])
        reg.add_sequence("stabilator_L", [{"stabilator_L": d, "stab_arm_L": -t, "stab_rod_L": -b}
                                          for d, t, _a, b in sts])

    # ------------------------------------------------------------------------------------------------------- fin
    def fin(self):
        C, f = self.C, self.f
        F = C.fin
        t_rib, t_skin = C.t_rib, C.t_skin
        ch96, ch86 = C.fit["F-FIN-FRONT"]["part"] + "-R", C.fit["F-FW-CORNER"]["part"] + "-R"
        A = self.add
        A("ffit", 266, "R", "tail", "fin front-spar root fitting, starboard", "dikey ön kiriş kök bağlantısı, sağ", M7075,
          P_CNC, f.front_fitting, thickness=FL_T, parent=ch96, step=32, explode=EX_FIN, contacts=(ch96,),
          notes="7075-T651 machined: 7.8 mm lug in the F-FIN-FRONT clevis (2 x M6 12.9 double shear), neck through the "
                "dorsal-hat gap, 5 mm flange on the front-spar web (2 x M5); ISO 2768-mK")
        A("rfit", 267, "R", "tail", "fin rear-spar root fitting, starboard", "dikey arka kiriş kök bağlantısı, sağ", M7075,
          P_CNC, f.rear_fitting, thickness=FL_T, parent=ch86, step=32, explode=EX_FIN, contacts=(ch86,),
          notes="7075-T651 machined: lug in the F-FW-CORNER slot (2 x M5 12.9, head on the aft ear, thread in the "
                "firewall-side web), neck aft of the heat-shield lip, 5 mm flange on the rear-spar web (2 x M5)")
        A("fspar", 251, "R", "tail", "fin front spar, starboard", "dikey ön kiriş, sağ", CFRP_UD, P_PREPREG,
          f.front_spar, thickness=T_WEB, parent=self.P("ffit"), step=32, explode=EX_FIN,
          notes="C-channel at 0.241 c, caps aft: 12 plies UD x 25 mm tapering (1:20 ply drops) to 4 plies at the tip "
                "(structures.sizing.tail), web 5 plies PW +-45")
        A("rspar", 253, "R", "tail", "fin rear (hinge) spar, starboard", "dikey arka (menteşe) kirişi, sağ", CFRP_UD,
          P_PREPREG, lambda: D(f.rear_spar_raw(), [f.rod_sweep()]), thickness=T_WEB, parent=self.P("rfit"),
          step=32, explode=EX_FIN,
          notes="C-channel 40 mm ahead of the hinge line (parallel to it), caps forward 6 plies UD x 20 mm, web 5 plies "
                "PW; pushrod slot at the servo station with the nested 7075 doubler")
        A("rrib", 255, "R", "tail", "fin root rib, starboard", "dikey kök kaburgası, sağ", CFRP_PW, P_PREPREG,
          lambda: pieces_above(D(f.root_rib_raw(), [f.fitting(True, BOND), f.fitting(False, BOND)])),
          layup="rib_panel", parent=self.P("fspar"), step=32, explode=EX_FIN,
          contacts=(self.P("fspar"), self.P("rspar"), self.P("ffit"), self.P("rfit")),
          notes="contoured on the body OML (+1 mm sealant line), sandwich rib_panel; the fitting necks pass through it")
        e_l, e_u = BOX_RIBS
        A("brib1", 257, "R", "tail", "fin lower box rib, starboard", "dikey alt kutu kaburgası, sağ", CFRP_PW, P_PREPREG,
          lambda: f.box_rib(e_l, e_l + t_rib), layup="rib_panel", parent=self.P("fspar"), step=32, explode=EX_FIN,
          contacts=(self.P("fspar"), self.P("rspar")))
        A("brib2", 259, "R", "tail", "fin upper box rib, starboard", "dikey üst kutu kaburgası, sağ", CFRP_PW, P_PREPREG,
          lambda: f.box_rib(e_u, e_u + t_rib), layup="rib_panel", parent=self.P("fspar"), step=32, explode=EX_FIN,
          contacts=(self.P("fspar"), self.P("rspar")))
        A("srib", 262, "R", "tail", "fin servo rib, starboard", "dikey servo kaburgası, sağ", M7075, P_CNC, f.servo_rib,
          thickness=SERVO_RIB_T, parent=self.P("fspar"), step=32, explode=EX_FIN,
          contacts=(self.P("fspar"), self.P("rspar")),
          notes="7075 plate 3 mm, bonded between the spar webs (anodised + glass isolation ply); DA 26 flange on its "
                "lower face, 4 x M3 nutplates on top")
        A("arib", 263, "R", "tail", "fin aft closure rib, starboard", "dikey arka kapama kaburgası, sağ", CFRP_PW,
          P_PREPREG, f.aft_rib, layup="rib_panel", parent=self.P("rspar"), step=32, explode=EX_FIN,
          contacts=(self.P("rspar"),), notes="closes the fixed trailing box below the rudder root (6 mm end gap)")
        A("trib", 264, "R", "tail", "fin tip rib, starboard", "dikey uç kaburgası, sağ", CFRP_PW, P_PREPREG, f.tip_rib,
          layup="rib_panel", parent=self.P("fspar"), step=32, explode=EX_FIN,
          contacts=(self.P("fspar"), self.P("rspar")))
        A("dbl", 270, "R", "tail", "rear-spar pushrod-slot doubler, starboard", "arka kiriş itme çubuğu yuvası takviyesi, sağ",
          M7075, P_CNC, f.slot_doubler, thickness=DBL_T, parent=self.P("rspar"), step=32, explode=EX_FIN,
          contacts=(self.P("rspar"),),
          notes="7075 C-section 2 mm nested in the rear spar over +-40 mm round the pushrod slot (carries the inboard "
                "cap round the slot), bonded + 4 blind rivets per cap (rivets: drawing note)")
        A("skin", 250, "R", "tail", "fin skin, starboard", "dikey kaplaması, sağ", CFRP_PW, P_PREPREG,
          lambda: pieces_above(D(f.skin_raw(), [f.hatch_recess(), f.hatch_opening(), f.rod_sweep(), f.horn_sweep(),
                                               f.cap_flange(BOND)])),
          layup="tail_skin", parent=self.P("fspar"), step=32, explode=EX_FIN,
          contacts=(self.P("fspar"), self.P("rspar"), self.P("rrib"), self.P("brib1"), self.P("brib2"),
                    self.P("srib"), self.P("arib"), self.P("trib")),
          notes="closed sandwich skin (layups.tail_skin) co-bonded to the spars and ribs; trimmed 1 mm above the body "
                "OML (fillet seal); joggled monolithic lands for the servo hatch and the tip-cap skirt; pushrod slot "
                "edge-sealed")
        A("cap", 258, "R", "tail", "fin tip cap (GFRP, antenna window), starboard", "dikey uç kapağı (GFRP, anten), sağ",
          GFRP, P_PREPREG, f.tip_cap, thickness=T_GFRP, parent=self.P("skin"), step=32, explode=EX_FIN,
          contacts=(self.P("skin"),),
          notes="4 plies GFRP 7781 (RF window for the 2.4 GHz dipole of layout ANT-BKP / ANT-C2B); skirt in the joggled "
                "fin-skin land, 2 x M3 into nutplates")
        A("hatch", 268, "R", "tail", "fin servo hatch cover, starboard", "dikey servo kapağı, sağ", CFRP_PW, P_PREPREG,
          f.hatch_cover, thickness=HATCH_T, parent=self.P("skin"), step=32, explode=(0.0, -0.10, 0.28),
          contacts=(self.P("skin"),), notes="5 plies PW, flush in the joggled land, 4 x M4 into nutplates "
                                             "(maintenance_access: fin servo hatches, inboard face)")
        C.reg.parts[self.P("hatch")].outline = f.hatch_cover_outline()
        for k in range(2):
            A(f"hstrip{k}", 288 + k, "R", "tail", f"servo hatch nutplate strip {'lower' if k == 0 else 'upper'}, starboard",
              f"servo kapağı somun plakası şeridi {'alt' if k == 0 else 'üst'}, sağ", M7075, P_CNC,
              (lambda k=k: f.hatch_strip(k)), thickness=0.0015, parent=self.P("skin"), step=32, explode=EX_FIN,
              contacts=(self.P("skin"),), notes="7075 strip bonded under the hatch land: conformal top, flat face for "
                                                "the 2 x M4 nutplates")
            A(f"cstrip{k}", 290 + k, "R", "tail", f"tip-cap nutplate pad {'inboard' if k == 0 else 'outboard'}, "
              "starboard", f"uç kapağı somun plakası pedi {'iç' if k == 0 else 'dış'}, sağ", M7075, P_CNC,
              (lambda k=k: f.cap_strip(k)), thickness=0.0015, parent=self.P("skin"), step=32, explode=EX_FIN,
              contacts=(self.P("skin"),))

    # ------------------------------------------------------------------------------------------------------- rudder
    def rudder(self):
        f = self.f
        A = self.add
        jr = "rudder_R"
        A("rskin", 300, "R", "controls", "rudder skin, starboard", "dümen kaplaması, sağ", CFRP_PW, P_PREPREG,
          lambda: pieces_above(D(f.rudder_skin_raw(), [f.nose_notch(i) for i in range(3)])), thickness=T_SKIN_CS,
          joint=jr, parent=self.P("rspar"), step=32, explode=EX_RUD,
          notes="3 plies PW 0.6 mm closed skin, round nose in the fin cove (2 mm gap), notches for the hinge clevises")
        A("rspar2", 304, "R", "controls", "rudder spar, starboard", "dümen kirişi, sağ", CFRP_UD, P_PREPREG,
          f.rudder_spar, thickness=T_WEB, joint=jr, parent=self.P("rskin"), step=32, explode=EX_RUD,
          contacts=(self.P("rskin"),), notes="C-channel 11 mm aft of the hinge line, caps aft 7 plies UD x 32 mm")
        A("rcore", 305, "R", "controls", "rudder core, starboard", "dümen çekirdeği, sağ", CORE, P_PREPREG,
          lambda: pieces_above(D(f.rudder_core(), [f.fitting_grow(f.horn_plate(), BOND)])), thickness=0.004, joint=jr, parent=self.P("rskin"), step=32, explode=EX_RUD,
          contacts=(self.P("rskin"), self.P("rspar2")),
          notes="ROHACELL 51 WF machined to the inner mould line, co-cured; potted M3 inserts for the hinge tongues "
                "and the horn")
        for i in range(3):
            A(f"tongue{i}", 306 + i, "R", "controls", f"rudder hinge tongue {i + 1}, starboard",
              f"dümen menteşe dili {i + 1}, sağ", M7075, P_CNC, (lambda i=i: f.tongue(i)["mesh"]),
              thickness=HINGE.base_t, joint=jr, parent=self.P("rspar2"), step=32, explode=EX_RUD,
              contacts=(self.P("rspar2"),))
            A(f"clevis{i}", 310 + i, "R", "controls", f"rudder hinge clevis {i + 1}, starboard",
              f"dümen menteşe çatalı {i + 1}, sağ", M7075, P_CNC, (lambda i=i: f.clevis(i)["mesh"]),
              thickness=HINGE.base_t, parent=self.P("rspar"), step=32, explode=EX_FIN, contacts=(self.P("rspar"),),
              notes="7075 clevis on the rear-spar web (2 x M3 into nutplates), 3 mm hinge pin H7/g6")
        self.C.reg.parts[self.P("rskin")].outline = f.rudder_skin_outline()
        A("hplate", 325, "R", "controls", "rudder horn backing plate, starboard", "dümen kolu karşı plakası, sağ",
          M7075, P_CNC, f.horn_plate, thickness=0.0015, joint=jr, parent=self.P("rskin"), step=32, explode=EX_RUD,
          contacts=(self.P("rskin"),), notes="7075 plate bonded under the rudder skin (core relieved), flat face for "
                                             "the horn-screw nuts")
        A("rhorn", 309, "R", "controls", "rudder horn, starboard", "dümen kolu, sağ", M7075, P_CNC, f.horn,
          thickness=PLATE_T, joint=jr, parent=self.P("rskin"), step=32, explode=EX_RUD, contacts=(self.P("rskin"),),
          notes="7075 conformal pad + 3 mm plate, integral ball (r 3.5) 26.8 mm off the hinge axis, 2 x M3 into "
                "potted inserts through skin and spar cap")

    def rudder_drive(self):
        f = self.f
        C = self.C
        A = self.add
        act = C.act["ACT-RUDDER"]
        A("da26", 303, "R", "controls", "rudder actuator Volz DA 26, starboard", "dümen eyleyicisi Volz DA 26, sağ",
          "purchased", P_BUY, f.da26, purchased=True, vendor="Volz DA 26 (DA 26-30-5024), extended-travel option",
          mass_kg=DA26["mass"], parent=self.P("srib"), step=32, explode=EX_ACT, contacts=(self.P("srib"),),
          notes=f"layout {act['id']}: datasheet case 54 x 102.8 x 26 mm, 0.270 kg, shaft 17.7 mm from the case edge "
                "(components.yaml volz_da26); case above the output shaft, mounting flange envelope with the 61.2 x 16 "
                "mm hole pattern on the servo rib (4 x M3)")
        A("rarm", 313, "R", "controls", "rudder servo arm, starboard", "dümen servo kolu, sağ", M7075, P_CNC, f.arm,
          thickness=PLATE_T, joint="rudder_arm_R", parent=self.P("da26"), step=32, explode=EX_ACT,
          contacts=(self.P("da26"),), notes="12 mm arm on the DA 26 spline, integral ball r 3.5")
        A("rrod", 314, "R", "controls", "rudder pushrod, starboard", "dümen itme çubuğu, sağ", M7075, P_CNC, f.rod,
          thickness=0.0019, joint="rudder_rod2_R", parent=self.P("rarm"), step=32, explode=EX_ACT,
          contacts=(self.P("rarm"), self.P("rhorn")),
          notes="7075 rod 6 mm with snap-on ball sockets (DIN 71802 type, 60 deg opening, retaining clip): spatial "
                "four-bar, 1.98:1 at neutral")

    # ------------------------------------------------------------------------------------------------------- stub
    def stub(self):
        C, g, sp = self.C, self.g, self.sp
        node = C.fit["F-SPINDLE-NODE"]["part"] + "-R"
        A = self.add
        A("sleeve", 269, "R", "tail", "stabilator stub spindle sleeve, starboard", "sabit kök mil yuvası, sağ", M7075,
          P_CNC, g.sleeve, thickness=0.003, parent=node, step=31, explode=EX_STUB, contacts=(node,),
          notes="7075 machined: 6 mm cruciform root flange on the node outboard cheek (layout B8-B11, 4 x M6 12.9), "
                "tube 49 x 3 to the stub tip, 37 H7 seat of the outboard 61805 bearing (line-bored with the node boss)")
        A("stskin", 252, "R", "tail", "stabilator root stub skin, starboard", "sabit kök kaplaması, sağ", CFRP_PW,
          P_PREPREG, sp.stub_skin, layup="tail_skin", parent=self.P("sleeve"), step=31, explode=EX_STUB,
          contacts=(self.P("sleeve"),), notes="sandwich skin trimmed 1 mm above the body OML, bonded to the sleeve")
        rr = sp.stub_root_rib()
        for k in range(len(rr)):
            A(f"strr{k}", 271 + k, "R", "tail", f"stub root rib {'fwd' if k == 0 else 'aft'}, starboard",
              f"sabit kök kaburgası {'ön' if k == 0 else 'arka'}, sağ", CFRP_PW, P_PREPREG,
              (lambda k=k: sp.stub_root_rib()[k]), layup="rib_panel", parent=self.P("stskin"), step=31,
              explode=EX_STUB, contacts=(self.P("stskin"), self.P("sleeve")))
        tr = sp.stub_tip_rib()
        for k in range(len(tr)):
            A(f"sttr{k}", 273 + k, "R", "tail", f"stub tip rib {'fwd' if k == 0 else 'aft'}, starboard",
              f"sabit kök uç kaburgası {'ön' if k == 0 else 'arka'}, sağ", CFRP_PW, P_PREPREG,
              (lambda k=k: sp.stub_tip_rib()[k]), layup="rib_panel", parent=self.P("stskin"), step=31,
              explode=EX_STUB, contacts=(self.P("stskin"), self.P("sleeve")))
        A("stspar", 275, "R", "tail", "stub front spar, starboard", "sabit kök ön kirişi, sağ", CFRP_UD, P_PREPREG,
          sp.stub_front_spar, thickness=T_WEB, parent=self.P("stskin"), step=31, explode=EX_STUB,
          contacts=(self.P("stskin"),),
          notes="C-channel in the plane of the F-STUB-FRONT slot (0.141 c), caps aft; the root fitting flange is bolted "
                "to the forward face of its web")
        ch98 = C.fit["F-STUB-FRONT"]["part"] + "-R"
        A("sffit", 265, "R", "tail", "stub front-spar root fitting, starboard", "sabit kök ön kiriş kök bağlantısı, sağ",
          M7075, P_CNC, sp.stub_fitting, thickness=SF_FL_T, parent=ch98, step=31, explode=EX_STUB,
          contacts=(ch98, self.P("stspar")),
          notes="7075-T651 plate machined: 7.8 mm lug in the F-STUB-FRONT clevis (end face seated on the slot floor, "
                "2 x M5 12.9 double shear, heads through the FS3480 access holes), neck through the body side and the "
                "stub root rib (shell slot: interface item), 5 mm flange on the front-spar web (2 x M5)")

    # ------------------------------------------------------------------------------------------------------- spindle + panel
    def stabilator(self):
        C, g, sp = self.C, self.g, self.sp
        node = C.fit["F-SPINDLE-NODE"]["part"] + "-R"
        A = self.add
        js = "stabilator_R"
        a = g.axis_pt
        A("spindle", 301, "R", "tail", "stabilator spindle, starboard", "stabilatör mili, sağ", MTI, P_CNC, g.spindle,
          thickness=0.0015, joint=js, parent=node, step=31, explode=EX_SPIN,
          notes="Ti-6Al-4V machined from bar: OD 25 (bearing seats ground), neck OD 21 through the node cheek hole, "
                "collar r 14 at the outboard bearing, bore 22/18, 15 mm solid inboard end (M6 end bolt), spline on "
                "the last 20 mm (structures.sizing.tail.spindle; wall 1.5 = CNC minimum, sizing 1.2)")
        A("brg_in", 315, "R", "controls", "stabilator inboard bearing 61805-ZZ, starboard",
          "stabilatör iç rulmanı 61805-ZZ, sağ", MSTEEL, P_BUY, (lambda: g.bearing(*g.brg_in)), purchased=True,
          vendor="61805-ZZ deep-groove ball bearing 25 x 37 x 7 mm, steel shields (mass: envelope x steel density, "
                 "estimate)", parent=node, step=31, explode=EX_SPIN, contacts=(node, self.P("spindle")))
        A("brg_out", 316, "R", "controls", "stabilator outboard bearing 61805-ZZ, starboard",
          "stabilatör dış rulmanı 61805-ZZ, sağ", MSTEEL, P_BUY, (lambda: g.bearing(*g.brg_out)), purchased=True,
          vendor="61805-ZZ deep-groove ball bearing 25 x 37 x 7 mm (mass: envelope x steel density, estimate)",
          parent=self.P("sleeve"), step=31, explode=EX_SPIN, contacts=(self.P("sleeve"), self.P("spindle")))
        y0, y1 = g.brg_in
        for k, (c0, c1) in enumerate(((y0 - 0.0015, y0), (y1, y1 + 0.0015))):
            A(f"clip{k}", 317 + k, "R", "controls", f"inboard bearing circlip DIN 472 37 {k + 1}, starboard",
              f"iç rulman segmanı DIN 472 37 {k + 1}, sağ", MSTEEL, P_BUY, (lambda c0=c0, c1=c1: g.circlip(c0, c1)),
              purchased=True, vendor="DIN 472 J37 internal retaining ring, spring steel (mass: geometry, estimate)",
              parent=node, step=31, explode=EX_SPIN, contacts=(node, self.P("brg_in")),
              notes="groove in the node boss bore (chassis interface, open item)")
        yo = g.brg_out[1]
        A("clip2", 319, "R", "controls", "outboard bearing circlip DIN 472 37, starboard",
          "dış rulman segmanı DIN 472 37, sağ", MSTEEL, P_BUY, (lambda: g.circlip(yo, yo + 0.0015)), purchased=True,
          vendor="DIN 472 J37 internal retaining ring, spring steel (mass: geometry, estimate)",
          parent=self.P("sleeve"), step=31, explode=EX_SPIN, contacts=(self.P("sleeve"), self.P("brg_out")))
        A("washer", 320, "R", "controls", "spindle end washer, starboard", "mil uç pulu, sağ", MTI, P_CNC, g.end_washer,
          thickness=0.003, joint=js, parent=self.P("spindle"), step=31, explode=EX_SPIN,
          contacts=(self.P("spindle"), self.P("brg_in")),
          notes="Ti washer r 13.4 on the inboard bearing inner race (axial location with the collar), M6 end bolt")
        A("shorn", 321, "R", "controls", "stabilator spindle horn, starboard", "stabilatör mil kolu, sağ", M7075, P_CNC,
          self.sd.horn, thickness=PLATE_T, joint=js, parent=self.P("spindle"), step=31, explode=EX_SPIN,
          contacts=(self.P("spindle"),),
          notes="7075 pinch hub on the spindle spline (M4 pinch bolt), 37.5 mm horn with integral ball r 4.0")
        # panel
        A("plug", 281, "R", "tail", "stabilator cross-bolt plug, starboard", "stabilatör çapraz cıvata tapası, sağ", MTI,
          P_CNC, g.plug_mesh, thickness=0.011, joint=js, parent=self.P("spindle"), step=34, explode=EX_STAB,
          contacts=(self.P("spindle"),), notes="Ti plug bonded in the spindle bore at the cross bolt")
        A("rootfit", 280, "R", "tail", "stabilator root fitting (socket), starboard", "stabilatör kök soketi, sağ",
          M7075, P_CNC, g.root_fitting, thickness=0.002, joint=js, parent=self.P("spindle"), step=34,
          explode=EX_STAB, contacts=(self.P("spindle"),),
          notes="7075 socket 29 x 2 x 100 mm with the cross-bolt bosses (structures.sizing.tail.socket), bonded in")
        A("sskin", 256, "R", "tail", "stabilator skin, starboard", "stabilatör kaplaması, sağ", CFRP_PW, P_PREPREG,
          sp.stab_skin, layup="tail_skin", joint=js, parent=self.P("rootfit"), step=34, explode=EX_STAB,
          contacts=(self.P("rootfit"),))
        A("sspar", 276, "R", "tail", "stabilator spar, starboard", "stabilatör kirişi, sağ", CFRP_UD, P_PREPREG,
          sp.stab_spar, thickness=T_WEB, joint=js, parent=self.P("sskin"), step=34, explode=EX_STAB,
          contacts=(self.P("sskin"),), notes="C-channel behind the socket to 0.40 c at the tip, caps aft 12 -> 4 plies")
        A("srr", 277, "R", "tail", "stabilator root rib, starboard", "stabilatör kök kaburgası, sağ", CFRP_PW, P_PREPREG,
          sp.stab_root_rib, layup="rib_panel", joint=js, parent=self.P("sskin"), step=34, explode=EX_STAB,
          contacts=(self.P("sskin"), self.P("rootfit"), self.P("sspar")))
        A("ssr", 278, "R", "tail", "stabilator socket rib, starboard", "stabilatör soket kaburgası, sağ", CFRP_PW,
          P_PREPREG, sp.stab_socket_rib, layup="rib_panel", joint=js, parent=self.P("sskin"), step=34,
          explode=EX_STAB, contacts=(self.P("sskin"), self.P("sspar"), self.P("rootfit")))
        A("smr", 279, "R", "tail", "stabilator mid rib, starboard", "stabilatör orta kaburgası, sağ", CFRP_PW, P_PREPREG,
          sp.stab_mid_rib, layup="rib_panel", joint=js, parent=self.P("sskin"), step=34, explode=EX_STAB,
          contacts=(self.P("sskin"), self.P("sspar")))

    def stab_drive(self):
        C, sd = self.C, self.sd
        fw = C.st["FS3670"]["part"]
        A = self.add
        eq = C.eq["EQ-STABACT"]
        A("cradle", 322, "R", "controls", "stabilator actuator cradle, starboard", "stabilatör eyleyici beşiği, sağ",
          M7075, P_CNC, sd.cradle, thickness=CRADLE_T, parent=fw, step=31, explode=EX_DRV, contacts=(fw,),
          notes="7075 machined: base on the firewall forward face (4 x M4 into potted inserts), plates above and "
                "below the case (4 x M4 through the DA 30 lug pattern)")
        A("da30", 302, "R", "controls", "stabilator actuator Volz DA 30, starboard", "stabilatör eyleyicisi Volz DA 30, sağ",
          "purchased", P_BUY, sd.da30, purchased=True, vendor="Volz DA 30 (DA 30.30.x), D-Sub, travel option +-85 deg",
          mass_kg=DA30["mass"], parent=self.P("cradle"), step=31, explode=EX_DRV, contacts=(self.P("cradle"),),
          notes=f"layout {eq['id']}: datasheet envelope 158.5 x 85 x 30 mm, 0.630 kg (components.yaml volz_da30); "
                "moved 2.5 mm forward and 9.5 mm inboard of the layout box so the arm sits in the pushrod plane")
        A("sarm", 323, "R", "controls", "stabilator servo arm, starboard", "stabilatör servo kolu, sağ", M7075, P_CNC,
          sd.arm, thickness=PLATE_T, joint="stab_arm_R", parent=self.P("da30"), step=31, explode=EX_DRV,
          contacts=(self.P("da30"),), notes="15 mm arm, integral ball r 4.0")
        A("srod", 324, "R", "controls", "stabilator pushrod, starboard", "stabilatör itme çubuğu, sağ", M7075, P_CNC,
          sd.rod, thickness=0.0021, joint="stab_rod_R", parent=self.P("sarm"), step=31, explode=EX_DRV,
          contacts=(self.P("sarm"), self.P("shorn")),
          notes="7075 rod 8 mm through C-FW-PUSHROD (fireproof bellows boot: systems), ball sockets both ends")

    # ------------------------------------------------------------------------------------------------------- ventral
    def ventral(self):
        C, v = self.C, self.v
        A = self.add
        for i in range(3):
            ch = C.fit[f"F-VENTRAL-{i + 1}"]["part"]
            A(f"vfit{i}", 284 + i, "C", "tail", f"ventral root fitting {i + 1}", f"ventral kök bağlantısı {i + 1}",
              M7075, P_CNC, (lambda i=i: v.fitting(i)), thickness=V_CHEEK_T, parent=ch, step=33, explode=EX_VEN,
              contacts=(ch,), notes="7075 tongue in the F-VENTRAL clevis (M5 12.9 double shear)" +
              (", fork cheeks on the mid-plane web (2 x M5)" if i < 2 else ", in the slot of the lug pad (2 x M5)"))
        A("vweb", 283, "C", "tail", "ventral mid-plane web", "ventral orta düzlem gövdesi", CFRP_PW, P_PREPREG, v.web,
          thickness=V_WEB_T, parent=self.P("vfit0"), step=33, explode=EX_VEN,
          notes="15 plies PW 3 mm in the fin mid-plane, carries the fork cheeks of root fittings 1 and 2")
        A("vskin", 254, "C", "tail", "ventral fin skin", "ventral kanatçık kaplaması", CFRP_PW, P_PREPREG, v.skin_raw,
          layup="tail_skin", parent=self.P("vweb"), step=33, explode=EX_VEN,
          contacts=(self.P("vweb"),),
          notes="sandwich skin trimmed 1 mm below the body OML, core-filled trailing edge aft of 0.80 c, 1.3 mm "
                "trailing-edge land; monolithic laminate build-ups (no core) round root lug 3 (slot for its tongue) "
                "and under the skid strap")
        A("vrib", 282, "C", "tail", "ventral root rib", "ventral kök kaburgası", CFRP_PW, P_PREPREG, v.root_rib,
          layup="rib_panel", parent=self.P("vskin"), step=33, explode=EX_VEN,
          contacts=(self.P("vskin"), self.P("vweb")))
        A("strap", 260, "C", "tail", "bumper skid strap (4130)", "tampon kızağı şeridi (4130)", MSTEEL, P_CNC, v.strap,
          thickness=SKID["strap_t"], parent=self.P("vskin"), step=33, explode=(0.0, 0.0, -0.32),
          contacts=(self.P("vskin"),),
          notes="4130 N machined U-strap round the ventral tip under the propeller disc (layout R-48), carries the shoe")
        A("shoe", 261, "C", "tail", "bumper skid shoe (replaceable)", "tampon kızağı pabucu (değiştirilebilir)",
          "pa12_sls", P_SLS, v.shoe, thickness=SKID["shoe_side"], parent=self.P("strap"), step=33,
          explode=(0.0, 0.0, -0.36), contacts=(self.P("strap"),),
          notes="PA12 SLS wear shoe 12 mm below the tip (spec.tail ventral skid_height; spec.assembly asks UHMW-PE, "
                "not in spec.materials - open item), 2 x M4 through shoe, strap and fin")

    # ------------------------------------------------------------------------------------------------------- mirror
    def mirror(self):
        reg = self.C.reg
        right = list(self.C.mirror_ids)
        idm = {pid: _port(pid) for pid in reg.parts if pid.endswith("-R")}
        for pid in right:
            p = reg.parts[pid]
            reg.add(mirror_part(p, _port(pid), joint=self.jmap.get(p.joint) if p.joint else None, id_map=idm))

    # ------------------------------------------------------------------------------------------------------- fasteners
    def fasteners(self, Fa: _Fast):
        C, f, g, sd, v = self.C, self.f, self.g, self.sd, self.v
        P = self.P
        reg = C.reg
        ch96, ch86 = C.fit["F-FIN-FRONT"]["part"] + "-R", C.fit["F-FW-CORNER"]["part"] + "-R"
        ch12 = C.st["FS3480"]["part"]
        node = C.fit["F-SPINDLE-NODE"]["part"] + "-R"
        fw = C.st["FS3670"]["part"]
        # ---- fin front lug: 2 x M6 12.9 double shear, head on the forward ear (access hole through FS3480)
        x_f = float(C.fit["F-FIN-FRONT"]["box"][0][0])
        for y, z in FF_BOLTS:
            Fa.sym(6, (x_f + 0.010, y, z), (1.0, 0.0, 0.0), [ch96, P("ffit")], window=(-0.0105, 0.0105), grade=G129,
                   hole_d=0.0064,
                   owner=P("ffit"), label=f"fin front lug y{y}", notes="F-FIN-FRONT lug bolt (layout B1/B2 moved for lug "
                                                                         "edge distance), head through the FS3480 access hole")
            for sgn in (1.0, -1.0):
                reg.parts[ch12].add_hole((x_f + 0.0005, sgn * y, z), (x_f - 0.0105, sgn * y, z), 0.0065)
        # ---- fin rear lug: 2 x M5 12.9, head on the aft ear, thread in the firewall-side web of F-FW-CORNER
        x_r = RF_SLOT[1] + 0.006
        for y, z in RF_BOLTS:
            Fa.sym(5, (x_r, y, z), (-1.0, 0.0, 0.0), [ch86, P("rfit")], window=(-0.002, 0.0138), grade=G129,
                   hole_d=0.0053,
                   nut="tapped", tapped_part=ch86, tapped_depth=0.0096, owner=P("rfit"), label=f"fin rear lug y{y}")
        # ---- web flanges (2 x M5 each), nyloc nuts behind the webs
        for front, spar, fit in ((True, P("fspar"), P("ffit")), (False, P("rspar"), P("rfit"))):
            for pt, ax in f.flange_bolts(front):
                Fa.sym(5, pt, ax, [fit, spar], window=(-0.002, 0.0065), grade=TI, owner=fit,
                       label=f"fin {'front' if front else 'rear'} flange")
        # ---- hinge clevises (2 x M3 into nutplates on the rear-spar web) and tongues (2 x M3 into core inserts)
        for i in range(3):
            for pt, ax in f.clevis(i)["bolts"]:
                Fa.sym(3, pt, ax, [P(f"clevis{i}"), P("rspar")], window=(-0.001, 0.0045), nut="nutplate", grade=TI,
                       label=f"clevis {i}")
            for pt, ax in f.tongue(i)["bolts"]:
                Fa.sym(3, pt, ax, [P(f"tongue{i}"), P("rspar2")], window=(-0.001, 0.0045), grade=TI,
                       pocket=(P("rcore"), 0.009, 0.0045), label=f"tongue {i}",
                       notes="nyloc nut in a machined core pocket (installed before closing)")
            # hinge pin through clevis lug, tongue, clevis lug (3 mm, H7/g6)
            a = f.h_a
            p0 = f.axis_at(f.hinge_eta(i)) - (0.5 * HINGE.lug_t + HINGE.gap + HINGE.lug_t) * a
            Fa.sym_pin(HINGE.pin_d, p0, a, [(P(f"clevis{i}"), HINGE.lug_t + HINGE.gap), (P(f"tongue{i}"),
                                            HINGE.lug_t + HINGE.gap), (P(f"clevis{i}"), HINGE.lug_t)],
                       owner=P(f"clevis{i}"), label=f"hinge pin {i}", spec="ISO 2341-B",
                       notes="rudder hinge pin, washer + split pin")
        # ---- rudder horn: 2 x M3 through pad, skin and spar cap into core inserts
        for pt, ax in f.horn_bolts():
            Fa.sym(3, pt, ax, [P("rhorn"), P("rskin"), P("hplate")], window=(-0.001, 0.012), on_axis=True, grade=TI,
                   pocket=(P("rcore"), 0.009, 0.0045), label="rudder horn",
                   notes="nyloc nut on the backing plate in a machined core pocket (horn fitted before closing)")
        # ---- DA 26 on the servo rib (4 x M3 into nutplates)
        for pt, ax in f.da26_holes():
            Fa.sym(3, pt, ax, [P("da26")], window=(-0.001, 0.0031), nut="tapped", grade=TI, tapped_part=P("srib"),
                   tapped_depth=0.0085, hole_d=0.0041, label="DA 26 flange",
                   notes="M3 helical insert in the servo-rib boss")
        # ---- servo hatch cover (4 x M4 into nutplates) and tip cap skirt (2 x M3 into nutplates)
        for k, (pt, ax) in enumerate(f.hatch_screws()):
            Fa.sym(4, pt, ax, [P("hatch"), P("skin"), P(f"hstrip{k // 2}")], window=(-0.001, 0.012), nut="nutplate",
                   grade=TI, on_axis=True, label="servo hatch")
        for k, (pt, ax) in enumerate(f.cap_screws()):
            Fa.sym(3, pt, ax, [P("cap"), P("skin"), P(f"cstrip{k}")], window=(-0.001, 0.012), nut="nutplate",
                   grade=TI, on_axis=True, label="tip cap")
        # ---- stub sleeve flange to the node outboard cheek (layout B8..B11, M6 12.9), head in the horn space
        for b in C.fit["F-SPINDLE-NODE"]["bolts"]:
            if b["group"] != "stub root":
                continue
            p_ = np.asarray(b["point"], float)
            Fa.sym(6, (p_[0], g.cheek_y[0], p_[2]), (0.0, 1.0, 0.0), [node, P("sleeve")], window=(-0.001, 0.0135),
                   grade=G129, label=f"stub root {b['id']}", notes=f"layout F-SPINDLE-NODE {b['id']}")
        # ---- stub front-spar fitting: lug in the F-STUB-FRONT clevis (2 x M5 12.9 double shear, heads through FS3480
        #      access holes) and flange on the front-spar web (2 x M5, nyloc nuts aft of the web, fitted before closing)
        d = self.sp.sf()
        ch98 = C.fit["F-STUB-FRONT"]["part"] + "-R"
        x_lo = float(C.fit["F-STUB-FRONT"]["box"][0][0])
        for z in d["bz"]:
            Fa.sym(5, (0.5 * (d["xa"] + d["xb"]), d["y_lug"], z), (1.0, 0.0, 0.0), [ch98, P("sffit")],
                   window=(-0.0085, 0.0085), grade=G129, hole_d=0.0053, owner=P("sffit"), label=f"stub front lug z{z}",
                   notes="F-STUB-FRONT B3/B4 (layout M6, 2 mm outboard): M5 for 2 D lug / ear edge distance; head "
                         "through the FS3480 access hole")
            for sgn in (1.0, -1.0):
                reg.parts[ch12].add_hole((x_lo + 0.0005, sgn * d["y_lug"], z), (x_lo - 0.0105, sgn * d["y_lug"], z),
                                         0.00525)
        # pass-through slot for the fitting neck in the FS3480 outer T-cap (0.4 mm round the plate; chassis doubler:
        # interface item in docs/detail/tail.md)
        slot = I(self.sp.stub_fitting(0.0004), box3((3.40, d["y_floor"], 0.10), (3.60, 0.32, 0.30)))
        reg.parts[ch12].holes.append(slot)
        reg.parts[ch12].holes.append(G.fix_orientation(G.Mesh(slot.V * np.array([1.0, -1.0, 1.0]), slot.F[:, ::-1])))
        for z in d["fl_bz"]:
            Fa.sym(5, (d["xa"], SF_FL_Y, z), (1.0, 0.0, 0.0), [P("sffit"), P("stspar")], window=(-0.001, 0.012),
                   grade=TI, owner=P("sffit"), label=f"stub front flange z{z}")
        # ---- spindle end bolt (M6, tapped into the solid inboard end) and horn pinch bolt (M4)
        Fa.sym(6, g.axis_pt(g.y_in - 0.003), (0.0, 1.0, 0.0), [P("washer")], window=(-0.001, 0.0031), nut="tapped",
               tapped_part=P("spindle"), tapped_depth=0.014, grade=G129, label="spindle end bolt")
        hp, hax, ht = sd.pinch_bolt()
        Fa.sym(4, hp, hax, [P("shorn")], window=(-0.001, ht + 0.001), grade=G129, max_gap=0.0011,
               label="horn pinch bolt")
        # ---- stabilator cross bolt (M5 from the lower skin through the counterbored skin, tapped in the upper boss)
        cb = g.axis_pt(g.y_cb)
        Fa.sym(5, cb, (0.0, 0.0, 1.0), [P("rootfit"), P("spindle"), P("plug")], window=(-0.030, SPINDLE_R - 0.0001),
               nut="tapped", tapped_part=P("rootfit"), tapped_depth=0.011, grade=G129, label="stabilator cross bolt",
               on_axis=True, hole_d=0.0053, spot=(P("rootfit"), None),
               notes="removable: stabilator slides off the spindle after this bolt is out")
        for sgn in (1.0, -1.0):
            reg.parts[P("sskin") if sgn > 0 else _port(P("sskin"))].add_hole(
                (cb[0], sgn * cb[1], cb[2] - 0.040), (cb[0], sgn * cb[1], cb[2] - 0.010), 0.0055)
        # ---- DA 30 lug bolts (4 x M4 through cradle plates and case) and cradle base (4 x M4 into firewall inserts)
        z_top = sd.case_hi[2] + BOND + CRADLE_T
        for x, y in sd.da30_holes():
            Fa.sym(4, (x, y, z_top), (0.0, 0.0, -1.0), [P("cradle"), P("da30")], window=(-0.001, 0.0363), grade=TI,
                   hole_d=0.0041, label="DA 30 lug")
        (x0c, xb), (y0c, y1c) = sd.cradle_dims()
        for z in (sd.case_lo[2] - BOND - CRADLE_T - 0.006,):
            for y in np.linspace(y0c + 0.009, y1c - 0.009, 4):
                Fa.sym(4, (xb, y, z), (1.0, 0.0, 0.0), [P("cradle")], window=(-0.001, CRADLE_T + 0.0001), nut="insert",
                       insert_part=fw, insert_depth=0.006, grade=TI, label="cradle base")
        # ---- ventral: lug bolts (M5 12.9 double shear), fork / pad bolts, skid bolts
        keel = "YK250-CH-033"
        for i in range(3):
            fit = C.fit[f"F-VENTRAL-{i + 1}"]
            b = [bb for bb in fit["bolts"] if bb["group"] == "ventral lug"][0]
            p_ = np.asarray(b["point"], float)
            Fa.bolt(5, (p_[0], 0.0, p_[2]), (0.0, 1.0, 0.0), [keel, fit["part"], P(f"vfit{i}")], window=(-0.016, 0.016),
                    grade=G129, hole_d=0.0053, owner=P(f"vfit{i}"), label=f"ventral lug {i + 1}",
                    notes=f"layout F-VENTRAL-{i + 1} B1 (M6 in the layout: M5 for the tongue edge distance)")
            for x, z in v.fork_bolts(i):
                pids = [P(f"vfit{i}"), P("vweb")] if i < 2 else [P("vskin"), P(f"vfit{i}")]
                Fa.bolt(5, (x, 0.0, z), (0.0, 1.0, 0.0), pids, window=(-0.02, 0.02), grade=TI, hole_d=0.0053,
                        owner=P(f"vfit{i}"), label=f"ventral fork {i + 1}", washer_nut=False, on_axis=(i == 2),
                        spot=(P("vskin"), P("vskin")) if i == 2 else (None, None))
        for pt, ax in v.skid_bolts():
            Fa.bolt(4, (pt[0], 0.0, pt[2]), ax, [P("shoe"), P("strap"), P("vskin")], window=(-0.02, 0.02),
                    head="ISO 7380", hole_d=0.0043, owner=P("shoe"), label="skid shoe", on_axis=True,
                    spot=(P("shoe"), P("shoe")))


def register(reg: Registry, spec: dict) -> None:
    """Register the tail parts, joints, coupled sequences and fasteners (ARCHITECTURE.md producer contract)."""
    if part_number("tail", 250, "R") in reg.parts:
        reg.note("tail: already registered, second call ignored")
        return
    C = Ctx(reg, spec)
    R = _TailReg(C)
    R.joints()
    R.fin()
    R.rudder()
    R.rudder_drive()
    R.stub()
    R.stabilator()
    R.stab_drive()
    R.ventral()
    R.port_joints()
    R.mirror()
    R.sequences()
    Fa = _Fast(C)
    R.fasteners(Fa)
    if Fa.problems:
        raise ValueError("tail fasteners: " + "; ".join(f"{a}: {b}" for a, b in Fa.problems))
