"""Shell producer (YK-250 HANÇER): every body skin, hatch, access door, fairing, RF window, the nose cone, the cowl
pieces, the LERX/glove skins and the wing-root fairing (``spec.layout.shell``), fastened to the chassis lands.

What it builds (part numbers from ``spec.layout.part_numbers.shell`` 350-499, ids fixed by ``layout.shell.panels``):

* body skins (``body_upper`` / ``body_lower`` / ``body_full``): OML patches thickened inward by the layup thickness in
  the section plane (the convention of the chassis frames: ``fuselage_section2d`` inset, mitred at the chine), split at
  the chine, cut round every removable panel, gear-door opening, root cut line and the wing-root fairing; 1.0 mm panel
  gaps; sacrificial land pads (3 plies PW, machined to fit) on the inner face over every chassis land so that the skin
  bears on the frame T-caps / longeron flanges; joggled lands (1.6 mm solid laminate, 25 mm under the removable
  panel's edge band) round every cut-out whose edge is not on a chassis land;
* removable panels and hatches: same construction, Camloc 4002 quarter-turn studs (75-100 mm, cowl 70-90 mm) into
  receptacles riveted to the land (frame cap, member flange or the joggled land of the surrounding skin); fuel-bay
  panels M4 + nutplates at 25-30 mm (gasket); RF windows GFRP (no carbon);
* the parachute hatch (lift-off, ``layout.mechanisms.joints.para_hatch``, prismatic) with its four locating tongues,
  the refuel door (hinged, ``layout.shell.panels[P-REFUEL].hinge``) with a flush piano hinge and its pin;
* the turret aperture ring insert (2.0 mm solid CFRP, HD59 opening = ball radius + radial clearance);
* the LERX/glove upper and lower skins (``wing_skin_primary``; ``lerx_skin_upper_root`` in the first LERX bay,
  ``wing_box_skin_upper`` between the spar caps, 1 mm solid laminate over the spar caps), the wing-root fairing
  (``layout.shell.wing_root_fairing``), the wing-joint access panel and the rear-pin bayonet cap;
* the cowl pieces (aft lip ring at the propeller hub, firewall land on the stainless edge angle, lower halves on
  the aft keel land flanges), the root strips (fin, stabilator stub, ventral), the dorsal cooling-inlet lip.

Interfaces read (never another module's geometry): ``spec.layout`` (part_numbers, stations: x, sweep, inset,
flange_w, cut-outs; chassis members: chine path / section, member ``lands``, CT box spar lines and cap levels;
shell panels / root_cut_lines / wing_root_fairing; mechanisms joints para_hatch and door_outlines; heat_protection
insert regions; keep-outs KO-COOLING-DUCT), ``spec.fuselage`` / ``spec.wing`` (OML), ``spec.payload.turret``,
``spec.layups`` / ``materials`` / ``processes``.

Module-private detailing constants (reflected in ``docs/detail/shell.md``) are listed below.
"""
from __future__ import annotations

import math

import numpy as np
from shapely.geometry import LineString, MultiPolygon, Point, Polygon
from shapely.geometry import box as sbox
from shapely.ops import unary_union

from ..core import geom as G
from ..core.parts import Joint, Part, Registry, layup_props, mirror_part, part_number
from . import fastener_catalog as FC
from . import joints as J
from . import oml as O
from . import structgen as SG

GROUP = "shell"

# ---------------------------------------------------------------------------------------------------------------------
# module-private detailing constants (docs/detail/shell.md)
# ---------------------------------------------------------------------------------------------------------------------
GAP = 0.0005                # half of the 1.0 mm panel gap (layout.shell.rules.joggles: 1.0 +- 0.3 mm)
GAP_LIFT = 0.0002           # lift-off parachute hatch: 0.2 mm trimmed fit to its neighbours (contacts; seal in the land)
SEAL = 0.0002               # sealant / seal line between a removable panel's inner face and its joggled land
FIT = 0.00015               # bond line of the sacrificial land pads on the chassis lands (liquid-shim class fit)
OV = 0.0002                 # boolean overlap of fused features (ARCHITECTURE §5)
LAND_T = 0.0016             # joggled land laminate under a removable panel edge (8 plies PW, = the solid edge band)
LAND_W = 0.025              # land width under the removable panel (layout.shell.rules.joggles: land 25 mm)
LAND_REACH = 0.020          # land fused under the surrounding fixed skin
ROW_CAP = 0.016             # nutplate row offset from a frame web centre (nutplate clear of the web, inside the cap)
EDGE_NUT = 0.0105           # M4 row distance from a panel edge (2.5 D = 10 mm + 0.5 mm)
EDGE_CAM = 0.0125           # Camloc stud row distance from a panel edge (2.5 D = 12 mm + 0.5 mm)
EDGE_LAND = 0.0105          # M4 distance from the end of a land (frame cap end, flange edge)
PITCH_NUT = (0.025, 0.032)  # structural skins (layout.shell.rules.concept)
PITCH_FUEL = (0.025, 0.030)
PITCH_CAM = (0.075, 0.100)
PITCH_COWL = (0.070, 0.090)
PITCH_INS = (0.060, 0.100)
MIN_SEP = 0.0135            # minimum spacing between two fasteners of one panel (3 D = 12 mm + margin)
RING_N = 288                # ring points of the lofted section envelopes
DX = 0.01                   # x spacing of the lofted section envelopes
SKIN_T_THIN = 0.0009        # solid laminate over the CT-box covers / caps (structures skin_solid_over_caps 1.0 mm
#                             minus the 0.1 mm bond line)
RAMP = 3.0                  # core ramp 1:3 (layout.shell.rules.sandwich_edges)

STEP_SKIN, STEP_GLOVE, STEP_FUEL, STEP_ANT, STEP_PARA, STEP_RING = 16, 17, 18, 21, 22, 24
STEP_STUB, STEP_FIN, STEP_VENTRAL, STEP_CLOSE = 31, 32, 33, 36

MAT_PW, MAT_GF, MAT_UD = "cfrp_pw_mtm45_as4", "gfrp_7781_mtm45", "cfrp_ud_mtm45_as4"
MAT_6061, MAT_7075, MAT_SS, MAT_TI = "al_6061_t6_sheet", "al_7075_t651_plate", "ss_304_annealed", \
    "ti_6al_4v_annealed_sheet"
P_PREG, P_SHEET, P_CNC = "prepreg_ooa_vacbag", "sheet_metal_aluminium", "cnc_milling_metal"


# =====================================================================================================================
# small geometry helpers
# =====================================================================================================================
def _as_polys(g) -> list[Polygon]:
    if g is None or g.is_empty:
        return []
    if isinstance(g, Polygon):
        return [g]
    return [p for p in getattr(g, "geoms", []) if isinstance(p, Polygon) and p.area > 1e-10]


def clean(g, min_area=1e-7):
    ps = [p for p in _as_polys(g) if p.area >= min_area]
    if not ps:
        return Polygon()
    return ps[0] if len(ps) == 1 else MultiPolygon(ps)


def extrude_cs(poly, t: float, origin, u, v) -> G.Mesh:
    """Prism from a shapely (Multi)Polygon in the (u, v) plane at ``origin``, extruded along u x v by ``t`` (manifold
    cross-section triangulator: robust with collinear and touching hole edges)."""
    import manifold3d as m3
    from shapely.geometry.polygon import orient
    contours = []
    for p in _as_polys(poly):
        p = orient(p, 1.0)
        contours.append(np.asarray(p.exterior.coords)[:-1].astype(np.float64))
        contours += [np.asarray(h.coords)[:-1].astype(np.float64) for h in p.interiors]
    cs = m3.CrossSection(contours, m3.FillRule.Positive)
    man = m3.Manifold.extrude(cs, float(t))
    m = G.Mesh.from_manifold(man)
    u, v = np.asarray(u, float), np.asarray(v, float)
    w = np.cross(u, v)
    V = np.asarray(origin, float) + m.V[:, :1] * u + m.V[:, 1:2] * v + m.V[:, 2:3] * w
    return G.fix_orientation(G.Mesh(V, m.F))


def prism_z(poly, z0: float = -1.0, z1: float = 1.0) -> G.Mesh:
    """Plan (x, y) polygon extruded along z."""
    return extrude_cs(poly, z1 - z0, (0.0, 0.0, z0), (1.0, 0.0, 0.0), (0.0, 1.0, 0.0))


def prism_y(poly_xz, y0: float, y1: float) -> G.Mesh:
    """(x, z) polygon extruded along +y from y0 to y1."""
    return extrude_cs(poly_xz, y1 - y0, (0.0, y0, 0.0), (1.0, 0.0, 0.0), (0.0, 0.0, 1.0)).transformed(
        np.diag([1.0, -1.0, 1.0]), (0.0, 2 * y0, 0.0))


def prism_x(poly_yz, x0: float, x1: float) -> G.Mesh:
    return extrude_cs(poly_yz, x1 - x0, (x0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0))


def man_and(a: G.Mesh, *bs: G.Mesh) -> G.Mesh | None:
    m = a.to_manifold()
    for b in bs:
        m = m ^ b.to_manifold()
        if m.is_empty():
            return None
    return G.Mesh.from_manifold(m)


def man_sub(a: G.Mesh, cutters) -> G.Mesh | None:
    cutters = [c for c in cutters if c is not None]
    if not cutters:
        return a
    import manifold3d as m3
    cut = m3.Manifold.batch_boolean([c.to_manifold() for c in cutters], m3.OpType.Add)
    r = a.to_manifold() - cut
    return None if r.is_empty() else G.Mesh.from_manifold(r)


def man_add(ms) -> G.Mesh:
    ms = [m for m in ms if m is not None]
    if len(ms) == 1:
        return ms[0]
    return G.union(ms)


def pieces_above(m: G.Mesh, vmin: float = 2e-8) -> G.Mesh:
    """Drop disconnected slivers smaller than ``vmin`` (m^3) left by the trims."""
    import manifold3d as m3
    allp = m.to_manifold().decompose()
    parts = [p for p in allp if p.volume() >= vmin]
    if not parts:
        raise ValueError("no piece left")
    if len(parts) == len(allp):
        return m
    return G.Mesh.from_manifold(parts[0] if len(parts) == 1 else m3.Manifold.compose(parts))


def largest_piece(m: G.Mesh) -> G.Mesh:
    parts = m.to_manifold().decompose()
    if len(parts) <= 1:
        return m
    return G.Mesh.from_manifold(max(parts, key=lambda p: p.volume()))


def finish(m: G.Mesh) -> G.Mesh:
    """Merge sub-micron sliver edges of a boolean result so that the triangle self-intersection test passes."""
    out = G.Mesh.from_manifold(m.to_manifold().simplify(1e-7))
    if out.check(self_intersect=True)["ok"]:
        return out
    out2 = G.Mesh.from_manifold(m.to_manifold().simplify(1e-6))
    return out2 if out2.check(self_intersect=True)["ok"] else out


def rect(x0, x1, y0, y1) -> Polygon:
    return sbox(min(x0, x1), min(y0, y1), max(x0, x1), max(y0, y1))


def mirror_poly(p):
    from shapely import affinity
    return affinity.scale(p, 1.0, -1.0, origin=(0.0, 0.0))


def _unit(v) -> np.ndarray:
    return G.unit(np.asarray(v, float))


# =====================================================================================================================
# context: OML (body + wing-root fairing + glove), envelopes, land footprints
# =====================================================================================================================
class SC:
    """Shell context: everything derived from the spec (no other module's geometry)."""

    def __init__(self, reg: Registry, spec: dict):
        self.reg, self.S, self.L = reg, spec, spec["layout"]
        self.fus = O.fuselage_from_spec(spec)
        self.wing = O.LiftingSurface(spec["wing"]["sections"], n_chord=80, name="wing")
        self.y_root = float(self.wing.sections[0]["y"])
        self.wrf = self.L["shell"]["wing_root_fairing"]
        self.y_in = float(self.wrf["y_inner_m"])
        self.st = {s["id"]: s for s in self.L["stations"]}
        ch = self.L["chassis"]
        self.mem = {m["id"]: m for m in ch["members"]}
        self.fit = {f["id"]: f for f in ch["fittings"]}
        self.pan = {p["id"]: p for p in self.L["shell"]["panels"]}
        self.n0, self.n1 = self.L["part_numbers"]["shell"]
        self.t_skin = self.layup_t("shell_secondary")
        self.inset = float(self.st["FS1110"]["inset"])          # skin line of every chassis land (frame T-caps)
        self.pad_bot = self.inset - FIT                          # inner face of the land pads
        self._sec, self._env, self._cache = {}, {}, {}
        self._groot_table()
        self._crease_table()

    # ------------------------------------------------------------------ ids / materials
    def pid(self, num: int, side: str = "C") -> str:
        if not self.n0 <= num <= self.n1:
            raise ValueError(f"part number {num} outside the shell range {self.n0}-{self.n1}")
        return part_number(GROUP, num, side)

    def layup_t(self, key: str) -> float:
        return float(layup_props(self.S, key)["thickness"])

    def ref(self, lid: str, side: str = "R") -> str:
        """Part id of a layout object (ST-*, M-*, F-*, P-*); mirrored objects get the side suffix."""
        if lid.startswith("ST-"):
            return self.st[lid[3:]]["part"]
        if lid.startswith("P-"):
            p = self.pan[lid]
            return p["part"] + (f"-{side}" if p.get("mirror") else "")
        obj = self.mem.get(lid) or self.fit.get(lid)
        return obj["part"] + (f"-{side}" if obj.get("mirror") else "")

    # ------------------------------------------------------------------ body OML
    def section(self, x):
        return self.fus.section(np.asarray(x, float))

    def w2(self, x) -> np.ndarray:
        return 0.5 * np.asarray(self.section(x)[0], float)

    def zc(self, x) -> np.ndarray:
        return np.asarray(self.section(x)[2], float)

    def z_top(self, x, y) -> np.ndarray:
        w, h, zc, nt, nb = self.section(x)
        tf = self.fus.top_frac(np.asarray(x, float))
        r = np.clip(np.abs(y) / np.maximum(0.5 * w, 1e-9), 0, 1)
        return zc + tf * h * np.clip(1 - r ** nt, 0, 1) ** (1 / nt)

    def z_bot(self, x, y) -> np.ndarray:
        w, h, zc, nt, nb = self.section(x)
        tf = self.fus.top_frac(np.asarray(x, float))
        r = np.clip(np.abs(y) / np.maximum(0.5 * w, 1e-9), 0, 1)
        return zc - (1 - tf) * h * np.clip(1 - r ** nb, 0, 1) ** (1 / nb)

    def sec(self, x: float, d: float = 0.0) -> Polygon:
        key = (round(float(x), 6), round(float(d), 7))
        if key not in self._sec:
            self._sec[key] = SG.fuselage_section2d(self.fus, float(x), float(d), n=720)
        return self._sec[key]

    # ------------------------------------------------------------------ glove root profile / wing-root fairing
    def _groot_table(self):
        poly, fr = SG.section2d(self.wing, self.y_root, n=200)
        P = np.asarray(poly.exterior.coords)
        Q = SG.to3d(fr, P)
        self.groot = Polygon(np.column_stack([Q[:, 0], Q[:, 2]])).buffer(0)
        x0, _, x1, _ = self.groot.bounds
        xs = np.linspace(x0 + 1e-5, x1 - 1e-5, 600)
        up, lo = [], []
        for xx in xs:
            g = LineString([(xx, -1), (xx, 1)]).intersection(self.groot)
            c = np.asarray(g.coords) if hasattr(g, "coords") else np.vstack([np.asarray(q.coords) for q in g.geoms])
            up.append(c[:, 1].max())
            lo.append(c[:, 1].min())
        self.gx0, self.gx1 = float(x0), float(x1)
        self._gt = (xs, np.asarray(up), np.asarray(lo))

    def groot_z(self, x):
        """(upper, lower) z of the glove root profile at x (-inf / +inf outside its chord)."""
        xs, up, lo = self._gt
        x = np.asarray(x, float)
        inside = (x >= xs[0]) & (x <= xs[-1])
        return (np.where(inside, np.interp(x, xs, up), -np.inf), np.where(inside, np.interp(x, xs, lo), np.inf))

    def groot_poly(self, d: float = 0.0) -> Polygon:
        if d <= 0:
            return self.groot if d == 0 else self.groot.buffer(-d, join_style=2)
        return SG.largest(self.groot.buffer(-d, join_style=2))

    def _crease_table(self):
        """Plan line where the wing-root fairing (glove root profile extruded inboard) meets the body upper surface."""
        xs = np.linspace(self.gx0 + 0.002, self.gx1 - 0.001, 260)
        ys = []
        for x in xs:
            gu = float(self.groot_z(x)[0])
            f = lambda y: float(self.z_top(x, y)) - gu      # noqa: E731  decreasing in y
            if f(self.y_root) >= 0:
                ys.append(np.nan)
                continue
            if f(self.y_in) <= 0:
                ys.append(self.y_in)
                continue
            a, b = self.y_in, self.y_root
            for _ in range(50):
                m = 0.5 * (a + b)
                a, b = (m, b) if f(m) > 0 else (a, m)
            ys.append(0.5 * (a + b))
        ys = np.asarray(ys)
        ok = np.isfinite(ys)
        self.crease = np.column_stack([xs[ok], ys[ok]])

    def fairing_plan(self, grow: float = 0.0, y_out: float = 1.0) -> Polygon:
        """Plan region (starboard) of the wing-root fairing: crease line .. y_out."""
        P = self.crease
        poly = Polygon(np.vstack([P, [[P[-1, 0], y_out], [P[0, 0], y_out]]])).buffer(0)
        return poly.buffer(grow, join_style=2) if grow else poly

    def z_up(self, x, y):
        """Upper OML z (body united with the wing-root fairing) at plan points."""
        x = np.asarray(x, float)
        y = np.abs(np.asarray(y, float))
        zt = self.z_top(x, y)
        gu, _ = self.groot_z(x)
        return np.where((y >= self.y_in) & (y <= self.y_root + 1e-3), np.maximum(zt, gu), zt)

    # ------------------------------------------------------------------ envelopes (closed solids)
    def body_env(self, d: float, x0: float, x1: float) -> G.Mesh:
        key = ("b", round(d, 7), round(x0, 4), round(x1, 4))
        if key not in self._env:
            n = max(2, int(math.ceil((x1 - x0) / DX)) + 1)
            rings = []
            for x in np.linspace(x0, x1, n):
                try:
                    poly = self.sec(x, d)
                except ValueError:
                    continue
                if poly.area < 2e-7:
                    continue
                P2 = SG.resample_ring(poly, RING_N, start_dir=(0.0, 1.0))
                rings.append(np.column_stack([np.full(len(P2), x), P2[:, 0], P2[:, 1]]))
            self._env[key] = G.fix_orientation(G.loft(rings))
        return self._env[key]

    def fair_env(self, d: float, x0: float, x1: float) -> G.Mesh | None:
        """Wing-root fairing envelope (glove root profile inset by d, extruded from y_inner to the wing root), both
        sides; None outside the glove root chord."""
        lo, hi = max(x0, self.gx0), min(x1, self.gx1)
        if hi <= lo + 1e-4:
            return None
        key = ("f", round(d, 7), round(lo, 4), round(hi, 4))
        if key not in self._env:
            poly = self.groot_poly(d).intersection(rect(x0, x1, -1, 1))
            poly = clean(poly, 1e-8)
            if poly.is_empty:
                self._env[key] = None
            else:
                f = prism_y(poly, self.y_in, self.y_root + 0.001)
                self._env[key] = G.union([f, f.mirrored_y()])
        return self._env[key]

    def env(self, d: float, x0: float, x1: float) -> G.Mesh:
        """Union OML (body + wing-root fairing) inset by d in the section plane, between x0 and x1."""
        key = ("u", round(d, 7), round(x0, 4), round(x1, 4))
        if key not in self._env:
            b = self.body_env(d, x0, x1)
            f = self.fair_env(d, x0, x1)
            self._env[key] = b if f is None else G.union([b, f])
        return self._env[key]

    def layer(self, d0: float, d1: float, x0: float, x1: float) -> G.Mesh:
        """Shell layer between the union OML inset by d0 and by d1 (section plane), x0..x1."""
        key = ("l", round(d0, 7), round(d1, 7), round(x0, 4), round(x1, 4))
        if key not in self._env:
            a = self.env(d0, x0, x1) if d0 > 0 else self.env(0.0, x0, x1)
            b = self.env(d1, x0 - 0.01, x1 + 0.01)
            self._env[key] = man_sub(a, [b])
        return self._env[key]

    def half(self, side: str, x0: float, x1: float, gap: float = GAP) -> G.Mesh:
        """Solid above (side 'U') or below ('L') the chine line z = zc(x) +- gap."""
        key = ("h", side, round(x0, 4), round(x1, 4), round(gap, 6))
        if key not in self._env:
            xs = np.linspace(x0, x1, max(2, int(math.ceil((x1 - x0) / 0.01)) + 1))
            rings = []
            for x in xs:
                z = float(self.zc(x))
                za, zb = (z + gap, 2.0) if side == "U" else (-2.0, z - gap)
                rings.append(np.array([[x, -2.0, za], [x, 2.0, za], [x, 2.0, zb], [x, -2.0, zb]]))
            self._env[key] = G.fix_orientation(G.loft(rings))
        return self._env[key]

    # ------------------------------------------------------------------ surface points / normals
    def phi_of(self, x: float, y: float, side: str) -> float:
        w, h, zc, nt, nb = (float(v) for v in self.section(x))
        r = min(abs(y) / max(0.5 * w, 1e-9), 1.0)
        if side == "U":
            return math.copysign(math.asin(r ** (nt / 2)), y if y != 0 else 1.0)
        return math.copysign(math.pi - math.asin(r ** (nb / 2)), y if y != 0 else 1.0)

    def body_point(self, x: float, phi: float):
        p = self.fus.point(x, phi)
        n = self.fus.normal(x, phi)
        return np.asarray(p, float), _unit(n)

    def oml_point(self, x: float, y: float, side: str):
        """OML point and outward normal at plan (x, y) on the upper ('U', incl. the fairing) or lower ('L') side."""
        if side == "U" and self.y_in <= abs(y) <= self.y_root + 1e-3 and float(self.groot_z(x)[0]) > \
                float(self.z_top(x, y)):
            z = float(self.groot_z(x)[0])
            e = 1e-4
            dz = (float(self.groot_z(x + e)[0]) - float(self.groot_z(x - e)[0])) / (2 * e)
            return np.array([x, y, z]), _unit([-dz, 0.0, 1.0])
        return self.body_point(x, self.phi_of(x, y, side))

    # ------------------------------------------------------------------ stations / chassis land footprints (plan)
    def x_web(self, sid: str, y) -> np.ndarray:
        """Frame web mid-plane x at plan y (chevron frames on the swept spar lines)."""
        s = self.st[sid]
        xf = s.get("x_faces") or [s["x"] - 0.5 * s["t"], s["x"] + 0.5 * s["t"]]
        xc = 0.5 * (float(xf[0]) + float(xf[1]))
        return xc + math.tan(math.radians(float(s.get("sweep_deg", 0.0)))) * np.abs(np.asarray(y, float))

    def web_half(self, sid: str) -> float:
        s = self.st[sid]
        xf = s.get("x_faces") or [s["x"] - 0.5 * s["t"], s["x"] + 0.5 * s["t"]]
        return 0.5 * (float(xf[1]) - float(xf[0]))

    def cap_fp(self, sid: str, lo: float | None = None, hi: float | None = None) -> Polygon:
        """Plan footprint of a frame T-cap: x_web(y) - lo .. x_web(y) + hi (default: half web + flange_w each side;
        the firewall stack has its CFRP cap forward only, the FS3738 U-ring flanges aft)."""
        s = self.st[sid]
        fw = float(s.get("flange_w", 0.028))
        h = self.web_half(sid)
        if lo is None:
            lo = h + fw
        if hi is None:
            hi = h + fw
        if sid == "FS3670":                         # firewall: the CFRP sandwich (forward of the stack) has its cap
            xf0 = float(s["x_faces"][0])            # forward only; aft of it the air gap and the stainless shield
            xc = float(self.x_web(sid, 0.0))
            lo, hi = xc - (xf0 - fw), xf0 + self.layup_t(s["layup"]) - xc
        ys = np.linspace(-0.6, 0.6, 61)
        a = np.column_stack([self.x_web(sid, ys) - lo, ys])
        b = np.column_stack([self.x_web(sid, ys[::-1]) + hi, ys[::-1]])
        return Polygon(np.vstack([a, b])).buffer(0)

    def member_land_fps(self, side: str) -> list[tuple[str, Polygon]]:
        """(part id, plan polygon) of the chassis member lands offered to the skins on side 'U' / 'L' (explicit
        layout 'lands' of the chassis members; the chine J skin flange; the keel-beam caps)."""
        key = ("mfp", side)
        if key in self._cache:
            return self._cache[key]
        out = []
        sname = "upper" if side == "U" else "lower"
        for mid in ("M-SPINE", "M-PARAWALL", "M-AFTKEEL", "M-VENTRALKEEL"):
            m = self.mem[mid]
            for ld in m.get("lands", []) or []:
                if ld.get("surface", "any") not in ("any", sname):
                    continue
                (x0, x1), (y0, y1) = ld["x"], sorted(ld["y"])
                if ld.get("mirror") or m.get("mirror"):
                    out.append((self.ref(mid, "R"), rect(x0, x1, y0, y1)))
                    out.append((self.ref(mid, "L"), rect(x0, x1, -y1, -y0)))
                else:
                    out.append((self.ref(mid), rect(x0, x1, y0, y1)))
        # chine J: the skin-side flange from the web to the chine, outside the wing-root fairing
        for k, pth in enumerate(self.mem["M-CHINE"]["paths"]):
            P = np.asarray(pth, float)
            w = float(self.mem["M-CHINE"]["section"]["w"])
            xs = np.linspace(P[0, 0], P[-1, 0], 80)
            if side == "U":
                xs = xs[(xs < self.crease[0, 0] - 0.002) | (xs > self.crease[-1, 0] + 0.002)]
            if len(xs) < 2:
                continue
            ok = np.diff(xs) < 0.03
            segs, cur = [], [xs[0]]
            for a_, b_, o_ in zip(xs, xs[1:], ok):
                if o_:
                    cur.append(b_)
                else:
                    segs.append(cur)
                    cur = [b_]
            segs.append(cur)
            for sx in segs:
                if len(sx) < 2:
                    continue
                sx = np.asarray(sx)
                yi = np.interp(sx, P[:, 0], P[:, 1]) - 0.5 * w
                poly = Polygon(np.vstack([np.column_stack([sx, yi]), [[sx[-1], 1.0], [sx[0], 1.0]]])).buffer(0)
                part = "YK250-CH-020" if k == 0 else "YK250-CH-040"
                out.append((part + "-R", poly))
                out.append((part + "-L", mirror_poly(poly)))
        if side == "L":                                  # payload-bay keel beams: UD cap outboard of the web at the skin
            m = self.mem["M-KEEL"]
            b = np.asarray(m["box"], float)
            fw = float(self.st["FS1110"].get("flange_w", 0.028))
            out.append((self.ref("M-KEEL", "R"), rect(b[0][0], b[1][0], b[1][1], b[1][1] + fw)))
            out.append((self.ref("M-KEEL", "L"), rect(b[0][0], b[1][0], -b[1][1] - fw, -b[1][1])))
        self._cache[key] = out
        return out

    def cap_parts(self) -> list[tuple[str, str]]:
        """(station id, part id) of the composite / metal frames whose caps are skin lands."""
        return [(sid, s["part"]) for sid, s in self.st.items()]

    def all_land_fp(self, side: str, grow: float = 0.0) -> Polygon:
        key = ("alf", side, round(grow, 5))
        if key not in self._cache:
            ps = [self.cap_fp(sid) for sid in self.st if sid != "FS3738"]
            ps += [p for _pid, p in self.member_land_fps(side)]
            u = unary_union(ps)
            self._cache[key] = u.buffer(grow, join_style=2) if grow else u
        return self._cache[key]
