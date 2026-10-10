"""Shell producer (YK-250 HANCER): every body skin, hatch, access door, fairing, RF window, the nose cone, the cowl
pieces, the LERX/glove skins and the wing-root fairing (``spec.layout.shell``), fastened to the chassis lands.

What it builds (part numbers from ``spec.layout.part_numbers.shell`` 350-499; the ids fixed by ``layout.shell.panels``
are kept, the extra detail parts use free numbers of the range, see ``docs/detail/shell.md``):

* body skins (``body_upper`` / ``body_lower`` / ``body_full``): the union OML (body + wing-root junction fairing)
  thickened inward by the layup thickness in the section plane (the convention of the chassis frames:
  ``structgen.fuselage_section2d`` inset, mitred at the chine), split at the chine plane ``z = zc(x)``, cut round every
  removable panel, gear-door opening, root cut line and the wing-root fairing; 1.0 mm panel gaps; land pads on the
  inner face over the chassis lands (frame T-caps, member flanges: 0.55 mm solid build-up of the edge band, machined
  to a 0.15 mm liquid-shim fit) so that the skin bears on the structure it is screwed to; joggled lands (1.6 mm solid
  laminate doubler, 25 mm under the neighbouring removable panel's edge band, 20 mm under its own skin) round every
  cut-out edge that does not lie on a chassis land; over the centre wing box the skin is the 1.0 mm solid laminate the
  box covers / caps are designed for (box outer face 1 mm under the OML, ``M-CTBOX.oml_clearance_m``);
* removable panels and hatches: the same construction, Camloc 4002 quarter-turn studs into receptacles riveted to the
  land (frame cap, member flange or the joggled land of the surrounding skin); fuel-bay panels M4 + nutplates
  (gasket); RF windows GFRP (no carbon);
* the parachute hatch (lift-off, ``layout.mechanisms.joints.para_hatch``, prismatic) and the refuel door (hinged,
  ``layout.shell.panels[P-REFUEL].hinge``, revolute) with its flush hinge;
* the turret aperture ring insert (2.0 mm solid CFRP, HD59 opening = ball radius + radial clearance) in the rebate of
  the lower skins;
* the LERX/glove upper and lower skins (primary wing skins, bonded, peel-stopper blind rivets), the wing-root fairing
  (``layout.shell.wing_root_fairing``), the wing-joint access panel and the rear-pin bayonet cap;
* the cowl pieces, the fin / stub / ventral root strips and the dorsal cooling-inlet lip.

Interfaces read (never another module's geometry for placement): ``spec.layout`` (part_numbers, stations: x, sweep,
inset, flange_w, cut-outs; chassis members: chine path / section, member ``lands``, CT box spar lines and cap levels;
shell panels / root_cut_lines / wing_root_fairing; mechanisms joints and door_outlines; heat_protection insert regions;
keep-outs), ``spec.fuselage`` / ``spec.wing`` (OML), ``spec.payload.turret``, ``spec.layups`` / ``materials`` /
``processes``. Fastener rows are laid out from these interfaces; every candidate is then checked the way the drilling
jig is proven (the same ray probes ``joints.bolt_through`` and ``analysis.checks`` use): the line must clamp solid
land material within 0.5 mm, keep the 2.5 D / 2.0 D edge distance in every clamped part, keep 3 D + hole radius to
every existing hole of those parts and the nutplate / receptacle must clear the structure behind the land; a
candidate that fails is not drilled (the row closes up round it).

Module-private detailing constants (reflected in ``docs/detail/shell.md``) are listed below.
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field

import numpy as np
from shapely.geometry import LineString, MultiPolygon, Point, Polygon
from shapely.geometry import box as sbox
from shapely.ops import unary_union

from ..core import geom as G
from ..core.parts import Fastener, Joint, Part, Registry, layup_props, mirror_part, part_number
from . import fastener_catalog as FC
from . import joints as J
from . import oml as O
from . import structgen as SG

GROUP = "shell"

# ---------------------------------------------------------------------------------------------------------------------
# module-private detailing constants (docs/detail/shell.md)
# ---------------------------------------------------------------------------------------------------------------------
GAP = 0.0005                # half of the 1.0 mm panel gap (layout.shell.rules.joggles: 1.0 +- 0.3 mm)
GAP_LIFT = 0.0002           # lift-off parachute hatch: 0.2 mm trimmed fit to its neighbours (seal in the land)
SEAL = 0.0002               # seal line between a removable panel's inner face and the joggled land under it
LAND_FIT = 0.0065 - 0.0002  # inner face of the land pads: 0.2 mm liquid-shim fit to the chassis land (skin line 6.5 mm)
OV = 0.0002                 # boolean overlap of fused features (ARCHITECTURE §5)
LAND_T = 0.0016             # joggled land laminate under a removable panel edge (8 plies PW = the solid edge band)
LAND_W = 0.025              # land width under the neighbouring panel (layout.shell.rules.joggles: land 25 mm)
LAND_REACH = 0.020          # land fused under its own skin
ROW_CAP = 0.0160            # fastener row offset from a frame web centre line (inside the T-cap, nutplate clear of
#                             the web)
EDGE_M4 = 0.0105            # M4 row distance from a panel edge (2.5 D = 10 mm + 0.5 mm)
EDGE_CAM = 0.0125           # Camloc stud row distance from a panel edge (2.5 D = 12 mm + 0.5 mm)
PITCH_NUT = 0.028           # structural skins 25-32 mm (layout.shell.rules.concept)
PITCH_FUEL = 0.028          # fuel-bay panels 25-30 mm
PITCH_CAM = 0.085           # hatches 75-100 mm
PITCH_COWL = 0.080          # cowl 70-90 mm
PITCH_INS = 0.080           # fairings 60-100 mm
RIDGE_CLEAR = 0.012         # no fastener within 12 mm of the V-roof ridge (mid-plane ray leaves the laminate there)
BOX_BOND = 0.0001           # skin to centre-box cover / cap bond line (1.0 mm solid skin over the box = 0.9 + 0.1)
RING_N = 288                # ring points of the lofted section envelopes
DX = 0.01                   # x spacing of the lofted section envelopes
T_RING = 0.002              # turret aperture ring insert (layout P-TURRETRING thickness_m)
T_AL = 0.0008               # P-COWL-UPS aluminium sheet (layout thickness_m)

STEP = {"skin": 16, "glove": 17, "fuel": 18, "ant": 21, "para": 22, "ring": 24, "stub": 31, "fin": 32,
        "ventral": 33, "joint": 35, "close": 36}

MAT_PW, MAT_GF = "cfrp_pw_mtm45_as4", "gfrp_7781_mtm45"
P_PREG, P_SHEET = "prepreg_ooa_vacbag", "sheet_metal_aluminium"
NUT_SPEC = dict(head="ISO 7380", grade="A2-70")


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


def box3(lo, hi) -> G.Mesh:
    lo, hi = np.asarray(lo, float), np.asarray(hi, float)
    return G.box(hi - lo, center=0.5 * (lo + hi))


def man_and(a: G.Mesh, *bs: G.Mesh) -> G.Mesh | None:
    m = a.to_manifold()
    for b in bs:
        if b is None:
            continue
        m = m ^ b.to_manifold()
        if m.is_empty():
            return None
    return G.Mesh.from_manifold(m)


def man_sub(a: G.Mesh, cutters) -> G.Mesh | None:
    cutters = [c for c in cutters if c is not None]
    if a is None:
        return None
    if not cutters:
        return a
    import manifold3d as m3
    cut = m3.Manifold.batch_boolean([c.to_manifold() for c in cutters], m3.OpType.Add)
    r = a.to_manifold() - cut
    return None if r.is_empty() else G.Mesh.from_manifold(r)


def man_add(ms) -> G.Mesh | None:
    ms = [m for m in ms if m is not None]
    if not ms:
        return None
    if len(ms) == 1:
        return ms[0]
    return G.union(ms)


def pieces_above(m: G.Mesh, vmin: float = 2e-8) -> G.Mesh | None:
    """Drop disconnected slivers smaller than ``vmin`` (m^3) left by the trims."""
    import manifold3d as m3
    if m is None:
        return None
    allp = m.to_manifold().decompose()
    parts = [p for p in allp if p.volume() >= vmin]
    if not parts:
        return None
    if len(parts) == len(allp):
        return m
    return G.Mesh.from_manifold(parts[0] if len(parts) == 1 else m3.Manifold.compose(parts))


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


def resample_line(P, step: float, margin: float = 0.0) -> np.ndarray:
    """Points evenly spaced (about ``step``) along the open polyline P (k, d), ``margin`` kept free at both ends and the
    spacing stretched so that the end points sit exactly at the margins."""
    P = np.asarray(P, float)
    seg = np.linalg.norm(np.diff(P, axis=0), axis=1)
    cum = np.r_[0.0, np.cumsum(seg)]
    L = cum[-1] - 2 * margin
    if L < 0:
        return np.zeros((0, P.shape[1]))
    n = max(1, int(math.floor(L / step + 0.5)) + 1)
    s = np.array([cum[-1] / 2]) if L < 1e-9 or n == 1 else np.linspace(margin, cum[-1] - margin, n)
    return np.column_stack([np.interp(s, cum, P[:, k]) for k in range(P.shape[1])])


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
        self.t_gfrp = self.gfrp_props()[0]
        self._sec, self._env, self._cache = {}, {}, {}
        self._groot_table()
        self._crease_table()
        B = self.mem["M-CTBOX"]
        self.ms_line = np.asarray(B["main_spar_line"], float)
        self.rs_line = np.asarray(B["rear_spar_line"], float)
        self.z_cov = float(B["z"][1])
        self.w_main = float(B["section"]["main_cap_width"])
        self.w_rear = float(B["section"]["rear_cap_width"])

    # ------------------------------------------------------------------ ids / materials
    def pid(self, num: int, side: str = "C") -> str:
        if not self.n0 <= num <= self.n1:
            raise ValueError(f"part number {num} outside the shell range {self.n0}-{self.n1}")
        return part_number(GROUP, num, side)

    def layup_t(self, key: str) -> float:
        return float(layup_props(self.S, key)["thickness"])

    def gfrp_props(self) -> tuple[float, float]:
        """(thickness, areal mass) of the RF-window sandwich: the shell_secondary schedule laid up in 7781 E-glass
        (2 plies / ROHACELL core / 2 plies, layout.shell.rules.layups) - no carbon in the window."""
        lay = self.S["layups"]["shell_secondary"]
        mats = self.S["materials"]
        g = mats[MAT_GF]
        n = sum(int(c) for _m, _a, c in list(lay["plies"]) + list(lay.get("inner_plies") or []))
        core = mats[lay["core"]]
        t = n * float(g["ply_t"]) + float(lay["core_t"])
        am = n * float(g["ply_t"]) * float(g["density"]) + float(lay["core_t"]) * float(core["density"])
        return t, am

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
            return self.groot
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
            poly = clean(self.groot_poly(d).intersection(rect(x0, x1, -1, 1)), 1e-8)
            if poly.is_empty:
                self._env[key] = None
            else:
                f = prism_y(poly, self.y_in - 0.002, self.y_root + 0.001)
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
            a = self.env(max(d0, 0.0), x0, x1)
            b = self.env(d1, x0 - 0.01, x1 + 0.01)
            self._env[key] = man_sub(a, [b])
        return self._env[key]

    def half(self, side: str, x0: float, x1: float, gap: float = GAP) -> G.Mesh:
        """Solid above (side 'U') or below ('L') the chine plane z = zc(x) +- gap ('F': everything)."""
        key = ("h", side, round(x0, 4), round(x1, 4), round(gap, 6))
        if key not in self._env:
            xs = np.linspace(x0, x1, max(2, int(math.ceil((x1 - x0) / 0.01)) + 1))
            rings = []
            for x in xs:
                z = float(self.zc(x))
                za, zb = (z + gap, 2.0) if side == "U" else ((-2.0, z - gap) if side == "L" else (-2.0, 2.0))
                rings.append(np.array([[x, -2.0, za], [x, 2.0, za], [x, 2.0, zb], [x, -2.0, zb]]))
            self._env[key] = G.fix_orientation(G.loft(rings))
        return self._env[key]

    # ------------------------------------------------------------------ centre wing box keep-out (layout M-CTBOX)
    def xm(self, y):
        return np.interp(np.abs(y), self.ms_line[:, 1], self.ms_line[:, 0])

    def xr(self, y):
        return np.interp(np.abs(y), self.rs_line[:, 1], self.rs_line[:, 0])

    def box_keepout(self) -> G.Mesh:
        """Solid below the centre-box outer surface + bond line inside the body (|y| <= SOB rib inner face): the box
        covers / caps lie at min(z_cov, union OML - 1 mm) (layout M-CTBOX z, oml_clearance_m) between the main-cap
        leading edge and the rear-cap trailing edge; skins keep the 0.1 mm bond line above it."""
        if "box" in self._cache:
            return self._cache["box"]
        clr = float(self.mem["M-CTBOX"].get("oml_clearance_m", 0.001))
        y_sob = float(self.mem["M-SOB"]["box"][0][1]) + 0.0005
        ys = np.linspace(-y_sob, y_sob, 161)
        nx = 41
        P = np.zeros((nx, len(ys), 3))
        for j, y in enumerate(ys):
            xa = float(self.xm(y)) - 0.5 * self.w_main - 0.0015
            xb = float(self.xr(y)) + 0.5 * self.w_rear + 0.0015
            xs = np.linspace(xa, xb, nx)
            zt = self.z_up(xs, np.full(nx, y))
            P[:, j, 0], P[:, j, 1] = xs, y
            P[:, j, 2] = np.minimum(self.z_cov, zt - clr) + BOX_BOND
        inward = np.zeros_like(P)
        inward[..., 2] = -1.0
        self._cache["box"] = G.shell_from_grid(P, 0.03, inward=inward)
        return self._cache["box"]

    # ------------------------------------------------------------------ stations
    def x_faces(self, sid: str):
        s = self.st[sid]
        return s.get("x_faces") or [s["x"] - 0.5 * s["t"], s["x"] + 0.5 * s["t"]]

    def x_web(self, sid: str, y) -> np.ndarray:
        """Frame web mid-plane x at plan y (chevron frames on the swept spar lines)."""
        xf = self.x_faces(sid)
        xc = 0.5 * (float(xf[0]) + float(xf[1]))
        return xc + math.tan(math.radians(float(self.st[sid].get("sweep_deg", 0.0)))) * np.abs(np.asarray(y, float))

    def x_line(self, sid: str, y) -> np.ndarray:
        """Station reference line x(y) (the layout x, swept with the frame)."""
        return float(self.st[sid]["x"]) + math.tan(math.radians(float(self.st[sid].get("sweep_deg", 0.0)))) * \
            np.abs(np.asarray(y, float))

    def web_half(self, sid: str) -> float:
        xf = self.x_faces(sid)
        return 0.5 * (float(xf[1]) - float(xf[0]))

    def cap_fp(self, sid: str, grow: float = 0.0) -> Polygon:
        """Plan footprint of a frame T-cap: web centre line +- (half web + flange_w) (+ grow)."""
        s = self.st[sid]
        fw = float(s.get("flange_w", 0.028)) + self.web_half(sid) + grow
        ys = np.linspace(-0.6, 0.6, 61)
        a = np.column_stack([self.x_web(sid, ys) - fw, ys])
        b = np.column_stack([self.x_web(sid, ys[::-1]) + fw, ys[::-1]])
        return Polygon(np.vstack([a, b])).buffer(0)

    def station_band(self, sid: str, lo: float, hi: float) -> Polygon:
        """Plan band x_web(y) + lo .. x_web(y) + hi."""
        ys = np.linspace(-0.6, 0.6, 61)
        a = np.column_stack([self.x_web(sid, ys) + lo, ys])
        b = np.column_stack([self.x_web(sid, ys[::-1]) + hi, ys[::-1]])
        return Polygon(np.vstack([a, b])).buffer(0)

    # ------------------------------------------------------------------ surface points / normals
    def phi_of(self, x: float, y: float, side: str) -> float:
        w, h, zc, nt, nb = (float(v) for v in self.section(x))
        r = min(abs(y) / max(0.5 * w, 1e-9), 1.0)
        if side == "U":
            return math.copysign(math.asin(min(1.0, r ** (nt / 2))), y if y != 0 else 1.0)
        return math.copysign(math.pi - math.asin(min(1.0, r ** (nb / 2))), y if y != 0 else 1.0)

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

    def section_curve(self, x_of_y, side: str, n: int = 400):
        """OML points (k, 3) along the curve x = x_of_y(y) on the upper ('U') or lower ('L') half, ordered from port
        to starboard by the polar angle (fixed-point iteration on x for swept frames)."""
        if side == "U":
            ph = np.linspace(-0.5 * math.pi + 1e-4, 0.5 * math.pi - 1e-4, n)
        else:
            ph = np.linspace(-0.5 * math.pi - 1e-4, -1.5 * math.pi + 1e-4, n)
        x = np.array([float(x_of_y(0.0))] * n)
        for _ in range(4):
            P = self.fus.point(x, ph)
            x = np.array([float(x_of_y(abs(py))) for py in P[:, 1]])
        P = self.fus.point(x, ph)
        N = self.fus.normal(x, ph)
        return P, N / np.linalg.norm(N, axis=1, keepdims=True)
