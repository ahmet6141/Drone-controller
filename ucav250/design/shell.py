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
GAP_LIFT = 0.0001           # lift-off parachute hatch: 0.2 mm trimmed fit to its neighbours (edge seal, declared
#                             contacts: they stay side by side while it lifts straight off)
SEAL = 0.0002               # seal line between a removable panel's inner face and the joggled land under it
LAND_FIT = 0.0065 - 0.0002  # inner face of the land pads: 0.2 mm liquid-shim fit to the chassis land (skin line 6.5 mm)
OV = 0.0002                 # boolean overlap of fused features (ARCHITECTURE §5)
LAND_T = 0.0016             # joggled land laminate under a removable panel edge (8 plies PW = the solid edge band)
LAND_W = 0.028              # land width under the neighbouring panel (layout.shell.rules: land >= 25 mm; 28 mm
#                             gives the Camloc row 2.5 D to the panel edge and to the land edge, 0.5 mm gap included)
LAND_REACH = 0.020          # land fused under its own skin
ROW_CAP = 0.0150            # fastener row offset from a frame web centre line (T-cap edge 28 mm off the centre
#                             line: 13 mm = 2.5 D of the Camloc stud + 1 mm; nutplate / receptacle clear of the web)
EDGE_M4 = 0.0105            # M4 row distance from a panel edge (2.5 D = 10 mm + 0.5 mm)
EDGE_CAM = 0.0125           # Camloc stud row distance from a panel edge (2.5 D = 12 mm + 0.5 mm)
PITCH_NUT = 0.028           # structural skins 25-32 mm (layout.shell.rules.concept)
PITCH_FUEL = 0.028          # fuel-bay panels 25-30 mm
PITCH_CAM = 0.085           # hatches 75-100 mm
PITCH_COWL = 0.080          # cowl 70-90 mm
PITCH_INS = 0.080           # fairings 60-100 mm
RIDGE_CLEAR = 0.012         # no fastener within 12 mm of the V-roof ridge (mid-plane ray leaves the laminate there)
BOX_BOND = 0.00025          # skin to centre-box cover / cap bond line: 1.0 mm (M-CTBOX oml_clearance) = 0.75 mm solid
#                             laminate + 0.25 mm bond line (covers the grid sag of the box cover surface)
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


def _fix_degenerate(m: G.Mesh) -> G.Mesh:
    """Remove zero-area (cap / needle) triangles left by a boolean: collapse a sub-0.1 um edge, otherwise flip the
    longest edge with its neighbour."""
    V, F = m.V.copy(), m.F.copy()
    for _ in range(50):
        a, b, c = V[F[:, 0]], V[F[:, 1]], V[F[:, 2]]
        areas = 0.5 * np.linalg.norm(np.cross(b - a, c - a), axis=1)
        bad = np.where(areas < 1e-13)[0]
        if not len(bad):
            break
        f = int(bad[0])
        tri = F[f]
        L = [np.linalg.norm(V[tri[(k + 1) % 3]] - V[tri[k]]) for k in range(3)]
        k_s, k_l = int(np.argmin(L)), int(np.argmax(L))
        if L[k_s] < 1e-7:
            i, j = int(tri[k_s]), int(tri[(k_s + 1) % 3])
            F[F == j] = i
            F = F[(F[:, 0] != F[:, 1]) & (F[:, 1] != F[:, 2]) & (F[:, 2] != F[:, 0])]
            continue
        ea, eb, ec = int(tri[k_l]), int(tri[(k_l + 1) % 3]), int(tri[(k_l + 2) % 3])
        hit = np.where(((F[:, 0] == eb) & (F[:, 1] == ea)) | ((F[:, 1] == eb) & (F[:, 2] == ea)) |
                       ((F[:, 2] == eb) & (F[:, 0] == ea)))[0]
        if len(hit) != 1:
            break
        g = int(hit[0])
        ed = int([v for v in F[g] if v not in (ea, eb)][0])
        F[f] = (ec, ea, ed)
        F[g] = (ec, ed, eb)
    used, inv = np.unique(F, return_inverse=True)
    return G.Mesh(V[used], inv.reshape(F.shape))


def finish(m: G.Mesh) -> G.Mesh:
    """Merge sub-micron sliver edges / zero-area faces of a boolean result (manifold simplify at growing tolerance,
    at most 0.1 mm; zero-area caps left over are flipped / collapsed) so that the mesh check (degenerate faces,
    self-intersections) passes."""
    man = m.to_manifold()
    out = None
    first = None
    for tol in (1e-7, 1e-6, 4e-6, 1e-5, 2e-5, 5e-5, 1e-4):
        out = G.Mesh.from_manifold(man.simplify(tol))
        c = out.check(self_intersect=True)
        if c["ok"]:
            return out
        if c["degenerate"] and not c["self_intersections"] and c["unpaired"] == 0 and c["dup_directed"] == 0:
            fx = _fix_degenerate(out)
            if fx.check(self_intersect=True)["ok"]:
                return fx
        first = first or out
    return out


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

    def fair_region(self) -> Polygon:
        """Plan region (starboard) of the wing-root fairing part SH-424: the junction fairing outboard of the crease;
        over the fuel bays (FS-FUEL .. FS-GEAR) it reaches inboard to the fuel-panel outboard edge (layout P-FUEL*
        outline y) and aft of the glove root out to the chine, so that it owns the joggled land under the fuel panels
        and no body-skin sliver is left between the panels and the crease."""
        if "fairreg" in self._cache:
            return self._cache["fairreg"]
        xf, xg = float(self.st["FS-FUEL"]["x"]), float(self.st["FS-GEAR"]["x"])
        y_f = float(np.asarray(self.pan["P-FUEL1"]["outline"], float)[:, 1].max())
        y_out = self.y_root + 0.0005
        reg = unary_union([self.fairing_plan(0.0, y_out), rect(xf, self.gx1, y_f, y_out),
                           rect(self.gx1 - 0.001, xg, y_f, 1.0)])
        self._cache["fairreg"] = clean(reg.intersection(rect(self.gx0, xg, 0.0, 1.0)), 1e-7)
        return self._cache["fairreg"]

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

    def body_env_normal(self, t: float, zref: float, x0: float, x1: float) -> G.Mesh:
        """Body OML inset by t measured along the 3-D surface normal of the side wall at z = zref (section-plane
        inset t sqrt(1 + (dw/dx)^2)): thin sheet parts on the steep cowl closure keep their gauge."""
        key = ("bn", round(t, 7), round(zref, 4), round(x0, 4), round(x1, 4))
        if key not in self._env:
            def w(x):
                g = LineString([(0.0, zref), (2.0, zref)]).intersection(self.sec(x))
                return float(g.bounds[2]) if not g.is_empty else 0.0
            n = max(2, int(math.ceil((x1 - x0) / 0.005)) + 1)
            rings = []
            for x in np.linspace(x0, x1, n):
                h = 0.002
                sl = (w(x + h) - w(x - h)) / (2 * h)
                d = min(t * math.sqrt(1.0 + sl * sl), 3.0 * t)
                poly = self.sec(float(x), d)
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

    def box_keepout(self, lower: bool = False) -> G.Mesh:
        """Solid inside the centre-box outer surface + bond line, out to the wing root (the SOB rib web and the caps
        run on to the glove): the box covers / caps lie at min(z_cov, union OML - 1 mm) (layout M-CTBOX z,
        oml_clearance_m) between the main-cap leading edge and the rear-cap trailing edge; skins keep the 0.25 mm bond
        line off it. ``lower``: the lower cover (z >= -z_cov - bond, up to the chine plane): the body side-wall skin
        below the glove root passes outboard of the cover end (spot-faced to the cover end + bond)."""
        key = ("box", lower)
        if key in self._cache:
            return self._cache[key]
        clr = float(self.mem["M-CTBOX"].get("oml_clearance_m", 0.001))
        y_e = self.y_root + 0.0008
        ys = np.linspace(-y_e, y_e, 641)          # 1.25 mm: the union OML kinks at the crease
        nx = 41
        P = np.zeros((nx, len(ys), 3))
        for j, y in enumerate(ys):
            xa = float(self.xm(y)) - 0.5 * self.w_main - 0.0015
            xb = float(self.xr(y)) + 0.5 * self.w_rear + 0.0015
            xs = np.linspace(xa, xb, nx)
            P[:, j, 0], P[:, j, 1] = xs, y
            if lower:
                P[:, j, 2] = -self.z_cov - BOX_BOND
            else:
                zt = self.z_up(xs, np.full(nx, y))
                P[:, j, 2] = np.minimum(self.z_cov, zt - clr) + BOX_BOND
        inward = np.zeros_like(P)
        inward[..., 2] = 1.0 if lower else -1.0
        Q = P[:, ::-1] if lower else P
        self._cache[key] = G.shell_from_grid(Q, self.z_cov if lower else 0.03,
                                             inward=inward[:, ::-1] if lower else inward)
        return self._cache[key]

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


# =====================================================================================================================
# fastening engine: candidates from the layout rows, proven like the drilling jig before they are drilled
# =====================================================================================================================
COMPOSITE_EDGE, METAL_EDGE = 2.5, 2.0       # edge distance factors (layout.shell.rules.edge_distance, ARCHITECTURE §6)
MAX_GAP = 0.0005                            # a bolted stack must clamp solid material (joints.bolt_through)
HW_TOL = 2e-10                              # hardware / structure overlap admitted by the proof (0.2 mm^3)


def _perp_basis(a):
    ref = np.array([0.0, 0.0, 1.0]) if abs(a[2]) < 0.9 else np.array([1.0, 0.0, 0.0])
    e1 = np.cross(a, ref)
    e1 /= np.linalg.norm(e1)
    return e1, np.cross(a, e1)


def _seg_dist(p0, p1, q0, q1) -> float:
    """Shortest distance between the segments p0-p1 and q0-q1."""
    d1, d2, r = p1 - p0, q1 - q0, p0 - q0
    a, e, f = float(d1 @ d1), float(d2 @ d2), float(d2 @ r)
    if a < 1e-18 and e < 1e-18:
        return float(np.linalg.norm(r))
    if a < 1e-18:
        s, t = 0.0, float(np.clip(f / e, 0, 1))
    else:
        c = float(d1 @ r)
        if e < 1e-18:
            t, s = 0.0, float(np.clip(-c / a, 0, 1))
        else:
            b = float(d1 @ d2)
            den = a * e - b * b
            s = float(np.clip((b * f - c * e) / den, 0, 1)) if den > 1e-18 else 0.0
            t = (b * s + f) / e
            if t < 0:
                t, s = 0.0, float(np.clip(-c / a, 0, 1))
            elif t > 1:
                t, s = 1.0, float(np.clip((b - c) / a, 0, 1))
    return float(np.linalg.norm((p0 + s * d1) - (q0 + t * d2)))


LAND_GROUPS = ("chassis", GROUP)    # fasteners land on the chassis and on the shell's own joggled lands only


@dataclass
class Cand:
    """A fastener candidate on a panel: OML point, inward axis, kind ('nut' M4 nutplate, 'cam' Camloc, 'ins' insert,
    'riv' blind rivet), the panel id and the land id(s) expected under it."""
    p: np.ndarray
    a: np.ndarray
    kind: str
    panel: str
    land: str
    size: float = 4
    orient: np.ndarray | None = None
    note: str = ""
    step: int = 0
    head: str = "ISO 7380"


class Fix:
    def __init__(self, sc: SC):
        self.sc, self.reg = sc, sc.reg
        self._man: dict = {}
        self._box: dict = {}
        self.holes: dict[str, list] = {}       # part id -> [(q0, q1, r_hole, d, kind)]
        self.hw: list = []                     # (lo, hi, manifold) of hardware meshes (existing + committed)
        self.count = {}
        self.dropped = []
        self._alt = 0
        mats = self.reg.spec.get("materials", {})
        self._kind = lambda p: "composite" if p.layup or mats.get(p.material, {}).get("kind") == "composite" \
            else str(mats.get(p.material, {}).get("kind", "metal"))
        from . import hardware as HW
        self.HW = HW
        for f in self.reg.fasteners():                  # holes and hardware already in the registry
            self._record(f)

    # ------------------------------------------------------------------ geometry caches
    def man(self, pid: str):
        if pid not in self._man:
            p = self.reg.parts[pid]
            m = p.base_mesh if p.group == GROUP else p.mesh
            self._man[pid] = m.to_manifold()
            self._box[pid] = m.bounds()
        return self._man[pid]

    def bounds(self, pid: str):
        self.man(pid)
        return self._box[pid]

    def forget(self, pid: str):
        self._man.pop(pid, None)
        self._box.pop(pid, None)

    def parts_near(self, lo, hi, pad=0.0):
        out = []
        for pid, p in self.reg.parts.items():
            if p.group == "hardware" or p.process == "consumable":
                continue
            b = self.bounds(pid)
            if np.all(b[1] + pad >= lo) and np.all(hi + pad >= b[0]):
                out.append(pid)
        return out

    def _record(self, f: Fastener):
        size = FC.size_from_spec(f.spec) or round(f.d * 1000)
        r = 0.5 * f.d * 1.02 if f.kind in ("pin", "clevis_pin") else 0.5 * FC.clearance(size)
        L = f.grip if f.grip is not None else f.length
        q0, q1 = f.position - 0.001 * f.axis, f.position + (L + 0.001) * f.axis
        for pid in f.joins:
            rr = r
            if "insert" in (f.nut or "").lower() and pid == f.joins[-1] and size in FC.INSERT:
                rr = max(r, 0.5 * FC.INSERT[size][1] + 0.0001)
            self.holes.setdefault(pid, []).append((q0, q1, rr, f.d))
        try:
            m = self.HW.fastener_mesh(f)
            self.hw.append((m.bounds()[0], m.bounds()[1], m.to_manifold()))
        except Exception:
            pass

    # ------------------------------------------------------------------ probes
    def intervals(self, pid: str, p, a, lo=-0.03, hi=0.08, r=0.0036):
        """Material intervals of part ``pid`` along p + t a (median of four probe lines at radius r)."""
        man = self.man(pid)
        e1, e2 = _perp_basis(a)
        ins = []
        for dv in (e1, -e1, e2, -e2):
            o = p + r * dv
            h = G.ray_hits(man, o + lo * a, o + hi * a) + lo
            if len(h) >= 2:
                ins.append([(float(h[i]), float(h[i + 1])) for i in range(0, len(h) - 1, 2)])
        if len(ins) < 3:
            return []
        n = min(len(v) for v in ins)
        return [(float(np.median([v[k][0] for v in ins])), float(np.median([v[k][1] for v in ins]))) for k in range(n)]

    def edge_room(self, pid: str, pos, a, d: float, size, grip: float, r_hole: float) -> tuple[float, bool]:
        """Edge distance on the geometry (analysis.checks._edge_distance_mesh)."""
        man = self.man(pid)
        lo_b, hi_b = self.bounds(pid)
        ext = float(np.linalg.norm(hi_b - lo_b)) + 0.01
        e1, e2 = _perp_basis(a)
        lo = -0.02 - 2 * d
        hi = grip + 0.02 + 2 * d
        g_hi = grip + 0.001
        mids, mids_g = [], []
        for dv in (e1, -e1, e2, -e2):
            o = pos + 1.6 * r_hole * dv
            h = G.ray_hits(man, o + lo * a, o + hi * a) + lo
            for i in range(0, len(h) - 1, 2):
                mids.append(0.5 * (h[i] + h[i + 1]))
                if h[i + 1] > -0.001 and h[i] < g_hi:
                    mids_g.append(0.5 * (h[i] + h[i + 1]))
        if not mids:
            return math.inf, False
        p0 = pos + float(np.median(mids_g or mids)) * a
        best = math.inf
        for th in np.linspace(0.0, 2 * math.pi, 36, endpoint=False):
            dv = math.cos(th) * e1 + math.sin(th) * e2
            h = G.ray_hits(man, p0, p0 + ext * dv)
            if not len(h):
                continue
            ed = h[1] if (h[0] < 1.6 * r_hole and len(h) > 1) else (math.inf if h[0] < 1.6 * r_hole else h[0])
            best = min(best, float(ed))
        return best, True

    def k_edge(self, pid: str) -> float:
        return COMPOSITE_EDGE if self._kind(self.reg.parts[pid]) == "composite" else METAL_EDGE

    # ------------------------------------------------------------------ the proof
    def prove(self, c: Cand, insert_depth: float | None = None):
        """Return the stack data of a provable candidate or None (reason appended to ``dropped``)."""
        a = _unit(c.a)
        p = np.asarray(c.p, float)
        iv_p = self.intervals(c.panel, p, a)
        if not iv_p:
            return self._drop(c, "panel not pierced")
        s0, e0 = iv_p[0]
        if abs(s0) > 0.002:
            return self._drop(c, f"panel face {s0 * 1000:.1f} mm off the OML point")
        s0 = self._head_seat(c, p, a, s0)
        d_ = 0.0048 if c.kind == "cam" else float(c.size) * 1e-3
        head0 = p + s0 * a
        for (h0, h1, rr, dd) in self.holes.get(c.panel, []):       # quick spacing test in the panel itself
            sd = _seg_dist(head0 - 0.001 * a, head0 + (e0 - s0 + 0.001) * a, h0, h1)
            if sd < 3.0 * max(d_, dd) + 0.0003:
                return self._drop(c, f"hole spacing {sd * 1000:.1f} mm in {c.panel}")
        lands = [c.land] if c.land else []
        if not lands:                       # the land: the first chassis / shell part behind the panel (a frame cap,
            prev, used = e0, {c.panel}      # a member flange or a joggled land); nutplates and rivets also clamp a
            for _k in range(3):             # second / third part lying within the seal gap behind it
                q0, q1 = p + (prev - 0.0005) * a, p + (prev + 0.003) * a
                best = None
                for pid in self.parts_near(np.minimum(q0, q1) - 0.004, np.maximum(q0, q1) + 0.004):
                    if pid in used or self.reg.parts[pid].group not in LAND_GROUPS:
                        continue
                    iv = [v for v in self.intervals(pid, p, a) if v[1] > prev + 1e-5]
                    if iv and (best is None or iv[0][0] < best[1]):
                        best = (pid, iv[0][0], iv[0][1])
                if best is None or (lands and (best[1] - prev > MAX_GAP or c.kind == "cam")):
                    break
                lands.append(best[0])
                used.add(best[0])
                prev = best[2]
            if not lands:
                return self._drop(c, "no land behind the panel")
        stack = [(c.panel, s0, e0)]
        prev = e0
        for land in lands:
            iv = [v for v in self.intervals(land, p, a) if v[1] > prev + 1e-5]
            if not iv:
                return self._drop(c, f"no land {land}")
            s1, e1 = iv[0]
            if s1 - prev > MAX_GAP or s1 < prev - 0.0002:
                return self._drop(c, f"gap {1000 * (s1 - prev):.2f} mm to {land}")
            stack.append((land, s1, e1))
            prev = e1
        t_end = stack[-1][2]
        if c.kind in ("nut", "cam") and len(stack) > 1:
            t_end = self._seat(c, p, a, stack, t_end)
            stack[-1] = (stack[-1][0], stack[-1][1], t_end)
        # nothing else in the clamped stack
        q0, q1 = p + (s0 - 0.0005) * a, p + (t_end + 0.0005) * a
        lo, hi = np.minimum(q0, q1) - 0.004, np.maximum(q0, q1) + 0.004
        names = {s[0] for s in stack}
        for pid in self.parts_near(lo, hi):
            if pid in names:
                continue
            iv = [v for v in self.intervals(pid, p, a, r=0.0028) if v[1] > s0 - 0.0002 and v[0] < t_end + 0.0002]
            if iv:
                return self._drop(c, f"{pid} in the stack")
        head = p + s0 * a
        grip = t_end - s0
        d = 0.0048 if c.kind == "cam" else float(c.size) * 1e-3
        size = 5 if c.kind == "cam" else c.size
        r_hole = 0.5 * FC.clearance(size)
        # edge distance in every clamped part
        for k, (pid, _s, _e) in enumerate(stack):
            rh = r_hole
            if c.kind == "ins" and k == len(stack) - 1 and int(c.size) in FC.INSERT:
                rh = max(r_hole, 0.5 * FC.INSERT[int(c.size)][1] + 0.0001)
            need = self.k_edge(pid) * d
            ed, ok = self.edge_room(pid, head, a, d, size, grip, rh)
            if not ok:
                return self._drop(c, f"not pierced {pid}")
            if ed < need + 0.0002:
                return self._drop(c, f"edge {ed * 1000:.1f} mm in {pid}")
            # separation from the existing holes of this part
            for (h0, h1, rr, dd) in self.holes.get(pid, []):
                sd = _seg_dist(head - 0.001 * a, head + (grip + 0.001) * a, h0, h1)
                if sd < max(need + rr, self.k_edge(pid) * dd + rh, 3.0 * max(d, dd)) + 0.0003:
                    return self._drop(c, f"hole spacing {sd * 1000:.1f} mm in {pid}")
        return {"head": head, "a": a, "stack": stack, "grip": grip, "d": d}

    def _head_seat(self, c: Cand, p, a, s0: float) -> float:
        """The flat bearing face of the head touches the highest OML point under it (the OML is curved)."""
        man = self.man(c.panel)
        if c.kind == "cam":
            r = 0.5 * FC.QUARTER_TURN["stud_head_d"]
        elif c.kind == "riv":
            r = float(c.size) * 1e-3
        else:
            r = 0.5 * FC.ISO7380.get(int(c.size), (1.9 * float(c.size) * 1e-3, 0.0))[0]
        e1, e2 = _perp_basis(a)
        best = s0
        for th in np.linspace(0.0, 2 * math.pi, 12, endpoint=False):
            q = p + r * (math.cos(th) * e1 + math.sin(th) * e2)
            h = G.ray_hits(man, q + (s0 - 0.003) * a, q + (s0 + 0.003) * a) + (s0 - 0.003)
            if len(h):
                best = min(best, float(h[0]))
        return best - (0.00002 if best < s0 - 1e-7 else 0.0)

    def _seat(self, c: Cand, p, a, stack, t_end: float) -> float:
        """Seat of the nutplate / receptacle base on the far face of the land: the base is flat, the land face is
        curved (concave seen from inside the body), so the base is turned to the flatter direction and its seat is
        the highest far-face point under its corners (the screw grip includes the 0.0x mm the base stands off)."""
        land = stack[-1][0]
        man = self.man(land)
        Lb, Wb = (FC.NUTPLATE[int(c.size)][0], FC.NUTPLATE[int(c.size)][1]) if c.kind == "nut" else \
            (FC.QUARTER_TURN["rec_plate"][0], FC.QUARTER_TURN["rec_plate"][1])
        o = np.asarray(c.orient if c.orient is not None else _perp_basis(a)[0], float)
        o = o - float(o @ a) * a
        o1 = _unit(o) if np.linalg.norm(o) > 1e-9 else _perp_basis(a)[0]
        o2 = np.cross(a, o1)
        opts = []
        for u, v in ((o1, o2), (o2, -o1)):
            far = t_end
            for su, sv in ((-1, -1), (-1, 1), (1, -1), (1, 1), (-1, 0), (1, 0), (0, -1), (0, 1)):
                q = p + (0.5 * Lb * su) * u + (0.5 * Wb * sv) * v
                h = G.ray_hits(man, q + (t_end - 0.004) * a, q + (t_end + 0.004) * a) + (t_end - 0.004)
                for i in range(0, len(h) - 1, 2):
                    if h[i] < t_end + 0.002 and h[i + 1] > t_end - 0.002:
                        far = max(far, float(h[i + 1]))
            opts.append((far, u))
        opts.sort(key=lambda o: o[0])
        far, u = opts[min(self._alt, len(opts) - 1)]
        c.orient = u
        return far + (0.00005 if far > t_end + 1e-7 else 0.0)

    def _drop(self, c: Cand, why: str):
        self.dropped.append((c.panel, c.land, c.kind, np.round(c.p, 4).tolist(), why))
        return None

    def _hw_clear(self, f: Fastener, joins) -> bool:
        """The hardware of ``f`` (outside the clearance holes) must not enter any part or other hardware."""
        m = self.HW.fastener_mesh(f)
        size = FC.size_from_spec(f.spec) or round(f.d * 1000)
        r = 0.5 * FC.clearance(size) + 0.0002 if f.kind != "camloc" else 0.5 * f.d + 0.0006
        L = (f.grip or 0.0)
        cyl = G.cylinder(r, f.position - 0.0005 * f.axis, f.position + (L + 0.0005) * f.axis, n=24)
        outside = man_sub(m, [cyl])
        if outside is None:
            return True
        om = outside.to_manifold()
        lo, hi = outside.bounds()
        for pid in self.parts_near(lo, hi, 0.0005):
            v = om ^ self.man(pid)
            if not v.is_empty() and v.volume() > HW_TOL:
                return False
        for (hlo, hhi, hm) in self.hw:
            if np.all(hhi >= lo) and np.all(hi >= hlo):
                v = om ^ hm
                if not v.is_empty() and v.volume() > HW_TOL:
                    return False
        return True

    # ------------------------------------------------------------------ commit
    def _fid(self, panel: str, kind: str) -> str:
        letter = {"cam": "C", "pin": "P"}.get(kind, "B")
        n = self.count.get((panel, letter), 0) + 1
        self.count[(panel, letter)] = n
        return f"{panel}-{letter}{n}"

    def make(self, c: Cand, pr: dict, fid: str, dry: bool):
        """Fastener for a proven candidate (registered unless ``dry``)."""
        reg = self.reg
        stack = pr["stack"]
        head, a, grip = pr["head"], pr["a"], pr["grip"]
        # gaps between faces are absorbed in the next part's thickness so the far-side element lands on the far face
        thick = [(stack[0][0], stack[1][1] - stack[0][1] if len(stack) > 1 else grip)] + \
                [(pid, (stack[k + 1][1] if k + 1 < len(stack) else e) - s) for k, (pid, s, e) in enumerate(stack)
                 if k > 0]
        if dry:
            holes = {pid: list(reg.parts[pid].holes) for pid, _t in thick}
            fl = [list(reg.parts[pid].fasteners) for pid, _t in thick]
        try:
            if c.kind == "cam":
                f = J.quarter_turn(reg, fid, head, a, thick[0][0], thick[0][1], thick[1][0],
                                   sum(t for _p, t in thick[1:]), step=c.step, notes=c.note, orient=c.orient)
            elif c.kind == "riv":
                f = J.rivet(reg, fid, float(c.size) * 1e-3, head, a, thick, step=c.step, notes=c.note,
                            spec="blind rivet (CherryMAX class, A286 / monel)")
            elif c.kind == "ins":
                f = J.bolt(reg, fid, int(c.size), head, a, thick[:-1], head=c.head, grade="A2-70", nut="insert",
                           insert_part=thick[-1][0], insert_depth=thick[-1][1], step=c.step, notes=c.note)
            else:
                f = J.bolt(reg, fid, int(c.size), head, a, thick, head=c.head, grade="A2-70", nut="nutplate",
                           step=c.step, notes=c.note, orient=c.orient)
        except ValueError as exc:
            if dry:
                self._restore(holes, fl, thick)
            self._drop(c, f"joint: {exc}")
            return None
        if dry:
            self._restore(holes, fl, thick)
        return f

    def _restore(self, holes, fl, thick):
        for (pid, _t), h, f in zip(thick, holes.values(), fl):
            self.reg.parts[pid].holes[:] = h
            self.reg.parts[pid].fasteners[:] = f
        for pid, _t in thick:
            p = self.reg.parts[pid]
            p._mesh, p._n_holes = None, -1

    def try_group(self, cands: list[Cand]) -> bool:
        """Prove and commit a group of candidates together (a symmetric pair): all or nothing. The nutplate /
        receptacle base is tried along the flatter direction of the land first, then across it."""
        proofs = []
        dry = []
        for c in cands:
            ok = None
            for alt in (0, 1):
                self._alt = alt
                pr = self.prove(c)
                if pr is None:
                    break
                f = self.make(c, pr, "DRY", dry=True)
                if f is not None and self._hw_clear(f, [s[0] for s in pr["stack"]]):
                    ok = pr
                    dry.append(f)
                    break
                if f is None:
                    break
            self._alt = 0
            if ok is None:
                if pr is not None and f is not None:
                    self._drop(c, "hardware interference")
                return False
            proofs.append(ok)
        # members of a group (a symmetric pair near the centre line) keep 3 D and the edge distance to each other
        for i in range(len(proofs)):
            for j in range(i + 1, len(proofs)):
                a_, b_ = proofs[i], proofs[j]
                d_ = max(a_["d"], b_["d"])
                sd = _seg_dist(a_["head"], a_["head"] + a_["grip"] * a_["a"], b_["head"], b_["head"] + b_["grip"] * b_["a"])
                if sd < max(3.0 * d_, COMPOSITE_EDGE * d_ + 0.5 * FC.clearance(4)) + 0.0003:
                    self._drop(cands[i], f"pair spacing {sd * 1000:.1f} mm")
                    return False
                ma, mb = self.HW.fastener_mesh(dry[i]).to_manifold(), self.HW.fastener_mesh(dry[j]).to_manifold()
                v = ma ^ mb
                if not v.is_empty() and v.volume() > HW_TOL:
                    self._drop(cands[i], "pair hardware interference")
                    return False
        # pairs must also keep their spacing to each other (two candidates of a group never share a part closely)
        for c, pr in zip(cands, proofs):
            f = self.make(c, pr, self._fid(c.panel, c.kind), dry=False)
            self._record(f)
        return True


# =====================================================================================================================
# panels: declarative record + mesh builder
# =====================================================================================================================
@dataclass
class Pan:
    key: str                    # layout id (P-...) or detail key
    num: int                    # part number (shell range)
    name: str
    name_tr: str
    surf: str                   # 'U' upper / 'L' lower / 'F' full wrap (split at the chine plane)
    region: Polygon             # plan region (starboard for mirrored panels, full for centre parts)
    mirror: bool = False
    material: str = MAT_PW
    layup: str | None = "shell_secondary"
    t: float = 0.0              # laminate thickness from the OML (section plane)
    thickness: float | None = None
    process: str = P_PREG
    step: int = 16
    removable: bool = False
    parent: str = ""
    contacts: tuple = ()
    explode: tuple = (0.0, 0.0, 0.0)
    x0: float = 0.0
    x1: float = 0.0
    pads: object = None         # plan polygon of the land pads (chassis lands under the panel)
    lands: list = field(default_factory=list)   # joggled lands: (strip under the neighbour, reach, t_neighbour)
    extra: list = field(default_factory=list)   # meshes fused to the panel (lugs, tongues, ribs)
    cut: list = field(default_factory=list)     # meshes subtracted
    zband: tuple | None = None
    keep: list = field(default_factory=list)    # meshes intersected (trim solids)
    pad_cut: list = field(default_factory=list)  # meshes subtracted from the land pads only
    areal: float | None = None  # areal mass of a non-layup laminate (GFRP sandwich windows)
    joint: str | None = None
    notes: str = ""
    land_refs: tuple = ()       # layout land ids (doc / BOM)
    keep_box: bool = True       # subtract the centre-box keep-out
    relief: bool = True         # chine relief of the inner face (chine_relief)
    t_normal: float | None = None   # z of the side wall where t is measured along the 3-D normal (sheet parts)
    color: str = ""
    rows: list = field(default_factory=list)    # fastener row specs (see Rows)
    cutouts: tuple = ()         # layout ids cut out of this panel (doc)


class LamPart(Part):
    """A sandwich part whose layup is not a spec layup (the GFRP RF windows): its mass is the mesh volume over the
    laminate thickness times the laminate areal mass (spec.materials ply data), like Registry.density for layups."""

    def __init__(self, *a, areal: float = 0.0, t_lam: float = 1.0, **kw):
        object.__setattr__(self, "_areal", float(areal))
        object.__setattr__(self, "_tlam", float(t_lam))
        super().__init__(*a, **kw)

    @property
    def mass_kg(self):
        if getattr(self, "_areal", 0.0) <= 0.0 or self.mesh_fn is None:
            return None
        return float(self.mesh.volume() / self._tlam * self._areal)

    @mass_kg.setter
    def mass_kg(self, v):
        pass


def chine_zone(sc: SC, x0: float, x1: float) -> G.Mesh:
    """Band +-4 mm about the chine corner (both sides): no land pad there (the mitred inset rings of the skin and of
    the chassis lands differ by their chord sag at the sharp corner)."""
    key = ("cz", round(x0, 4), round(x1, 4))
    if key not in sc._env:
        rings = []
        for x in np.linspace(x0, x1, max(2, int(math.ceil((x1 - x0) / 0.01)) + 1)):
            w, z = float(sc.w2(x)), float(sc.zc(x))
            rings.append(np.array([[x, w - 0.015, z - 0.004], [x, 1.0, z - 0.004], [x, 1.0, z + 0.004],
                                   [x, w - 0.015, z + 0.004]]))
        m = G.fix_orientation(G.loft(rings))
        sc._env[key] = G.union([m, m.mirrored_y()])
    return sc._env[key]


CHINE_RELIEF = (0.0012, 0.008, 0.004)   # inner-face relief along the chine: depth, below / above the chine plane


def chine_relief(sc: SC, x0: float, x1: float, t: float) -> G.Mesh | None:
    """Relief of the skin inner face along the chine corner (both sides, over the chine-longeron run): the J's skin
    flange wraps the corner at the 6.5 mm skin line, whose sharp mitred rings sag differently from the skin rings
    between stations; the skin is spot-faced 1.2 mm there (no bond pad within the band either)."""
    P0 = np.asarray(sc.mem["M-CHINE"]["paths"][0], float)
    P1 = np.asarray(sc.mem["M-CHINE"]["paths"][-1], float)
    a, b = max(x0, float(P0[0, 0]) - 0.01), min(x1, float(P1[-1, 0]) + 0.01)
    if b <= a + 1e-4:
        return None
    key = ("cr", round(a, 4), round(b, 4), round(t, 6))
    if key not in sc._env:
        rings = []
        dz0, dz1 = CHINE_RELIEF[1], CHINE_RELIEF[2]
        for x in np.linspace(a, b, max(2, int(math.ceil((b - a) / 0.01)) + 1)):
            w, z = float(sc.w2(x)), float(sc.zc(x))
            rings.append(np.array([[x, w - 0.04, z - dz0], [x, 1.0, z - dz0], [x, 1.0, z + dz1],
                                   [x, w - 0.04, z + dz1]]))
        m = G.fix_orientation(G.loft(rings))
        slab = G.union([m, m.mirrored_y()])
        sc._env[key] = man_and(sc.body_env(t - CHINE_RELIEF[0], a - 0.01, b + 0.01), slab)
    return sc._env[key]


def build_panel_mesh(sc: SC, pn: Pan) -> G.Mesh:
    """Skin layer (OML .. t) inside the plan region and the surface half, plus land pads and joggled lands, minus the
    centre-box keep-out and the cutters."""
    if getattr(pn, "mesh_override", None) is not None:
        return pn.mesh_override()
    x0, x1 = pn.x0 - 0.003, pn.x1 + 0.003
    half = sc.half(pn.surf, x0, x1)
    reg = pn.region
    pz = prism_z(reg)
    if pn.t_normal is not None:             # true-normal gauge (sheet metal on the steep cowl closure)
        lay = man_sub(sc.body_env(0.0, x0, x1), [sc.body_env_normal(pn.t, pn.t_normal, x0 - 0.01, x1 + 0.01)])
    else:
        lay = sc.layer(0.0, pn.t, x0, x1)
    parts = [man_and(lay, half, pz)]
    if pn.pads is not None and not pn.pads.is_empty:
        pads = clean(pn.pads.intersection(reg.buffer(-0.0005, join_style=2)), 1e-6)
        if not pads.is_empty:
            pm = man_and(sc.layer(pn.t - OV, LAND_FIT, x0, x1), half, prism_z(pads))
            parts.append(man_sub(pm, [chine_zone(sc, x0, x1)] + list(pn.pad_cut)))
    lhalf = sc.half(getattr(pn, "land_surf", pn.surf), x0, x1)
    for strip, reach, t_n in pn.lands:
        strip = clean(strip, 1e-6)
        if strip.is_empty:
            continue
        parts.append(man_and(sc.layer(t_n + SEAL, t_n + SEAL + LAND_T, x0, x1), lhalf, prism_z(strip)))
        rch = clean(reach, 1e-6)
        if not rch.is_empty and t_n + SEAL + OV > pn.t - OV + 1e-5:    # (a land deeper than the owner skin)
            parts.append(man_and(sc.layer(pn.t - OV, t_n + SEAL + OV, x0, x1), lhalf, prism_z(rch)))
    parts += pn.extra
    m = man_add(parts)
    if pn.zband is not None:
        m = man_and(m, box3((x0 - 0.1, -2, pn.zband[0]), (x1 + 0.1, 2, pn.zband[1])))
    for k in pn.keep:
        m = man_and(m, k)
    cut = list(pn.cut)
    if pn.relief:
        cr = chine_relief(sc, x0, x1, pn.t)
        if cr is not None:
            cut.append(cr)
    if pn.keep_box and x0 < 3.0 and x1 > 2.4:
        if pn.surf in ("U", "F"):
            cut.append(sc.box_keepout())
        if pn.surf in ("L", "F"):
            cut.append(sc.box_keepout(lower=True))
    m = man_sub(m, cut)
    m = pieces_above(m, 2e-8)
    if m is None:
        raise ValueError(f"{pn.key}: empty panel")
    return finish(m)


# =====================================================================================================================
# layout outlines, land footprints, deviations
# =====================================================================================================================
def lay_outline(sc: SC, lid: str) -> Polygon:
    p = sc.pan[lid]
    if p.get("outline"):
        return Polygon(np.asarray(p["outline"], float)).buffer(0)
    (x0, x1), (y0, y1) = p["x"], p["y"]
    return rect(x0, x1, y0, y1)


def outlines(sc: SC) -> dict:
    """Plan outlines of the removable / hinged panels (starboard for mirrored ones). Edges that the layout puts "on
    a frame cap" are moved onto the frame's web centre line (the panel then shares the cap with its neighbour like a
    fixed skin: row 16 mm off the web, 15.5 mm to the panel edge and 15.4 mm to the cap edge; the layout's
    0.3-6 mm offsets leave < 2.5 D on one side); documented in docs/detail/shell.md."""
    o = {lid: lay_outline(sc, lid) for lid in sc.pan}
    xs = lambda sid: float(sc.st[sid]["x"])                                           # noqa: E731
    f0300, f0600, f1110, f1810, ffuel, fgear = (xs(k) for k in ("FS0300", "FS0600", "FS1110", "FS1810", "FS-FUEL",
                                                                "FS-GEAR"))
    q = np.asarray(sc.pan["P-FWDHATCH"]["outline"], float)                          # trapezoid: extend the sides
    k = (q[1, 1] - q[0, 1]) / (q[1, 0] - q[0, 0])
    ya, yb = q[0, 1] + k * (f0300 - q[0, 0]), q[1, 1] + k * (f0600 - q[1, 0])
    o["P-FWDHATCH"] = Polygon([(f0300, ya), (f0600, yb), (f0600, -yb), (f0300, -ya)]).buffer(0)
    (a0, a1), (b0, b1) = sc.pan["P-AVHATCH"]["x"], sc.pan["P-AVHATCH"]["y"]
    o["P-AVHATCH"] = rect(f0600, f1110, b0, b1)
    for lid in ("P-SIDEBAY-L", "P-SIDEBAY-R"):
        P = np.asarray(sc.pan[lid]["outline"], float).copy()
        P[np.isclose(P[:, 0], 1.107, atol=0.004), 0] = f1110
        o[lid] = Polygon(P).buffer(0)
    (a0, a1), (b0, b1) = sc.pan["P-MBHATCH"]["x"], sc.pan["P-MBHATCH"]["y"]
    o["P-MBHATCH"] = rect(f1810, ffuel, b0, b1)
    P = np.asarray(sc.pan["P-FUEL1"]["outline"], float).copy()
    P[np.isclose(P[:, 0], P[:, 0].min()), 0] = ffuel
    o["P-FUEL1"] = Polygon(P).buffer(0)
    P = np.asarray(sc.pan["P-FUEL3"]["outline"], float).copy()
    P[np.isclose(P[:, 0], P[:, 0].max()), 0] = fgear
    o["P-FUEL3"] = Polygon(P).buffer(0)
    # payload hatch: fore edge on the FS-FUEL web line, aft edge parallel to the swept FS-RS web on its aft cap,
    # sides moved outboard onto the keel-beam caps (2.5 D Camloc row needs the edge 24 mm off the beam web face)
    yk = float(sc.mem["M-KEEL"]["box"][0][1])                    # inner face of the keel-beam web
    ys = np.linspace(-(yk + 0.0255), yk + 0.0255, 41)
    xa = np.full_like(ys, ffuel)
    xb = sc.x_web("FS-RS", ys) + sc.web_half("FS-RS") + 0.0251
    o["P-PAYHATCH"] = Polygon(np.vstack([np.column_stack([xa, ys]), np.column_stack([xb, ys])[::-1]])).buffer(0)
    (a0, a1), (b0, b1) = sc.pan["P-AFTHATCH"]["x"], sc.pan["P-AFTHATCH"]["y"]
    o["P-AFTHATCH"] = rect(fgear, xs("FS3480"), b0, b1)
    (a0, a1), (b0, b1) = sc.pan["P-STABACT"]["x"], sc.pan["P-STABACT"]["y"]
    o["P-STABACT"] = rect(xs("FS3480"), float(sc.x_faces("FS3670")[0]), b0, b1)
    return o


def doors_cut(sc: SC) -> list[Polygon]:
    """Gear-door openings (layout.mechanisms.door_outlines, closed positions), both sides."""
    D = sc.L["mechanisms"]["door_outlines"]
    out = []
    for keys in (("nose_door_R",), ("main_inner_door_R", "main_leg_door_R", "main_trunnion_door_R")):
        u = unary_union([Polygon(np.asarray(D[k]["outline"], float)).buffer(0) for k in keys])
        out += [u, mirror_poly(u)]
    return out


def land_fps(sc: SC, side: str, pads: bool = False) -> list[tuple[str, Polygon]]:
    """(part id, plan polygon) of the chassis skin lands from the layout: frame T-caps, member lands, the chine J skin
    flange, the keel-beam lands and the dorsal-longeron flanges. ``pads``: footprints that get land pads (the lower
    skins bear on the chine J only along its 12 mm side strip, too narrow for a pad without fouling its corner)."""
    key = ("lfp", side, pads)
    if key in sc._cache:
        return sc._cache[key]
    out = [(s["part"], sc.cap_fp(sid)) for sid, s in sc.st.items() if sid not in ("FS3738",)]
    sname = "upper" if side == "U" else "lower"
    for mid in ("M-SPINE", "M-PARAWALL", "M-AFTKEEL", "M-VENTRALKEEL"):
        m = sc.mem[mid]
        for ld in m.get("lands", []) or []:
            if ld.get("surface", "any") not in ("any", sname):
                continue
            (x0, x1), (y0, y1) = ld["x"], sorted(ld["y"])
            if ld.get("mirror") or m.get("mirror"):
                out.append((sc.ref(mid, "R"), rect(x0, x1, y0, y1)))
                out.append((sc.ref(mid, "L"), rect(x0, x1, -y1, -y0)))
            else:
                out.append((sc.ref(mid), rect(x0, x1, y0, y1)))
    w = float(sc.mem["M-CHINE"]["section"]["w"])
    fair = sc.fairing_plan(0.01, 1.0) if side == "U" else Polygon()
    for k, pth in enumerate(sc.mem["M-CHINE"]["paths"]):
        P = np.asarray(pth, float)
        poly = Polygon(np.vstack([np.column_stack([P[:, 0], P[:, 1] - 0.5 * w]),
                                  [[P[-1, 0], 1.0], [P[0, 0], 1.0]]])).buffer(0)
        poly = clean(poly.difference(fair), 1e-7)       # the J skin flange is clipped under the junction fairing
        part = sc.mem["M-CHINE"]["part"] if k == 0 else "YK250-CH-040"
        if not (pads and side == "L"):
            out += [(part + "-R", poly), (part + "-L", mirror_poly(poly))]
    if side == "L":
        b = np.asarray(sc.mem["M-KEEL"]["box"], float)
        # payload-hatch side land of the keel beam (layout: "payload hatch land"): web inner face out to the hatch
        # edge band (Camloc row 2.5 D + 2.5 D) = 40 mm outboard of the web
        out.append((sc.ref("M-KEEL", "R"), rect(b[0][0], b[1][0] + 0.03, b[0][1], b[1][1] + 0.040)))
        out.append((sc.ref("M-KEEL", "L"), rect(b[0][0], b[1][0] + 0.03, -b[1][1] - 0.040, -b[0][1])))
    else:
        m = sc.mem["M-DORSAL"]          # hat skin flanges (12 mm each side, layout section text): the pads stop
        P = np.asarray(m["paths"][0], float)    # 1 mm off the webs, whose corners sag between loft stations
        hw = 0.5 * float(m["section"]["w"])
        yc = float(P[0, 1])
        bands = ((yc - hw - 0.012, yc - hw - 0.001), (yc + hw + 0.001, yc + hw + 0.012)) if pads else \
            ((yc - hw - 0.012, yc + hw + 0.012),)      # (joggled lands keep off the whole hat)
        for a, b in bands:
            out.append((sc.ref("M-DORSAL", "R"), rect(P[0, 0], P[-1, 0], a, b)))
            out.append((sc.ref("M-DORSAL", "L"), rect(P[0, 0], P[-1, 0], -b, -a)))
    sc._cache[key] = out
    return out


def land_union(sc: SC, side: str, grow: float = 0.0, pads: bool = False) -> Polygon:
    key = ("lu", side, round(grow, 5), pads)
    if key not in sc._cache:
        sc._cache[key] = unary_union([p.buffer(grow, join_style=2) if grow else p for _pid, p in
                                      land_fps(sc, side, pads)])
    return sc._cache[key]


# =====================================================================================================================
# body panels (layout.shell.panels: body_upper / body_lower / body_full)
# =====================================================================================================================
def _strip(owner_region: Polygon, O: Polygon, sc: SC, side: str, t_n: float, extra_cut=()):
    """Joggled land of a fixed skin under the edges of the removable outline O that do not lie on a chassis land:
    25 mm under the panel edge band, 20 mm under the owner skin; stops 1.5 mm short of every chassis land (frame caps,
    member flanges, chine J) and of the cutters given."""
    band = O.buffer(LAND_REACH, join_style=2).difference(O.buffer(-LAND_W, join_style=2))
    band = band.difference(land_union(sc, side, 0.0015))
    for c in extra_cut:
        band = band.difference(c)
    band = band.intersection(owner_region.union(O))
    band = clean(band, 2e-6)
    reach = clean(band.difference(O.buffer(GAP, join_style=2)).intersection(owner_region), 2e-6)
    return (band, reach, t_n)


def glove_root_cutter(sc: SC) -> G.Mesh:
    """The body side wall above the glove lower surface lies inside the glove root (SOB rib, glove skins): the lower
    body skins end on the glove root profile (layout wing.sections[0]) + gap, both sides."""
    if "grc" not in sc._cache:
        poly = clean(sc.groot.buffer(GAP, join_style=2), 1e-8)
        m = prism_y(poly, 0.36, 0.45)
        sc._cache["grc"] = G.union([m, m.mirrored_y()])
    return sc._cache["grc"]


def body_panels(sc: SC) -> dict[str, Pan]:
    o = outlines(sc)
    S = sc.pan
    st = lambda sid: float(sc.st[sid]["x"])                                           # noqa: E731
    tg, ag = sc.gfrp_props()
    t = sc.t_skin
    fair = sc.fair_region().buffer(GAP, join_style=2)
    fair2 = unary_union([fair, mirror_poly(fair)])
    g_out = rect(sc.gx0 - 0.002, sc.gx1 + 0.001, sc.y_root + 0.0005, 1.0)
    g_out = unary_union([g_out, mirror_poly(g_out)])
    doors = doors_cut(sc)
    lam = lambda lid: dict(material=MAT_GF, layup=None, t=tg, thickness=tg, areal=ag) if S[lid]["material"] == MAT_GF \
        else dict(material=S[lid]["material"], layup=S[lid].get("layup") or "shell_secondary", t=t)   # noqa: E731
    out: dict[str, Pan] = {}

    def add(lid, surf, region, x0, x1, **kw):
        p = S.get(lid, {})
        num = int(p["part"].split("-")[2]) if p else kw.pop("num")
        base = lam(lid) if p else {}
        base.update({k: kw.pop(k) for k in list(kw) if k in ("material", "layup", "t", "thickness", "areal")})
        if surf == "U":             # inside the glove root (outboard of the junction fairing): wing structure
            region = region.difference(g_out)
        pn = Pan(key=lid, num=num, name=kw.pop("name", p.get("name", lid)), name_tr=kw.pop("name_tr", p.get("name_tr",
                 lid)), surf=surf, region=clean(region, 1e-7), x0=x0, x1=x1,
                 removable=p.get("attach") in ("removable", "hinged"), land_refs=tuple(p.get("lands") or ()),
                 cutouts=tuple(c["id"] for c in p.get("cutouts") or []), **base, **kw)
        out[lid] = pn
        return pn

    def grow(lid, g=GAP, mirror=None):
        poly = o[lid].buffer(g, join_style=2)
        if mirror if mirror is not None else S[lid].get("mirror"):
            return unary_union([poly, mirror_poly(poly)])
        return poly

    def pads_for(x0, x1, side):
        return land_union(sc, side, pads=True).intersection(rect(x0 - 0.05, x1 + 0.05, -1, 1))

    up_pads = lambda x0, x1: pads_for(x0, x1, "U")                                    # noqa: E731
    lo_pads = lambda x0, x1: pads_for(x0, x1, "L")                                    # noqa: E731
    full = lambda x0, x1: rect(x0, x1, -1.0, 1.0)                                     # noqa: E731

    # ---------------------------------------------------------------- nose
    f0300, f0600, f1110 = st("FS0300"), st("FS0600"), st("FS1110")
    add("P-NOSECONE", "F", full(-0.01, f0300 - GAP), 0.0, f0300, pads=up_pads(0, f0300).union(lo_pads(0, f0300)),
        step=STEP["close"], parent=sc.st["FS0300"]["part"], explode=(-0.35, 0.0, 0.0))
    fwd = add("P-FWDSKIN", "F", full(f0300 + GAP, f0600 - GAP), f0300, f0600,
              pads=up_pads(f0300, f0600).union(lo_pads(f0300, f0600)),
              step=STEP["skin"], parent=sc.st["FS0300"]["part"], explode=(0.0, 0.0, -0.18))
    # full-wrap panel: the hatch opening is cut from the upper half only (above the chine plane): the skin layer
    # (the joggled land under the hatch edge band stays), and the land pads under the hatch
    hcut = man_and(prism_z(grow("P-FWDHATCH")), sc.half("U", f0300 - 0.01, f0600 + 0.01, gap=-GAP))
    fwd.cut.append(man_sub(hcut, [sc.env(t + 0.0001, f0300 - 0.01, f0600 + 0.01)]))
    fwd.pad_cut.append(hcut)
    fwd.land_surf = "U"
    add("P-FWDHATCH", "U", o["P-FWDHATCH"].buffer(-GAP, join_style=2), f0300, f0600, pads=up_pads(f0300, f0600),
        step=STEP["close"], parent=fwd.key, explode=(0.0, 0.0, 0.3))
    fwd.lands.append(_strip(fwd.region, o["P-FWDHATCH"], sc, "U", t))
    nu = add("P-NOSE-UPPER", "U", full(f0600 + GAP, f1110 - GAP).difference(grow("P-AVHATCH")), f0600, f1110,
             pads=up_pads(f0600, f1110), step=STEP["skin"], parent=sc.st["FS0600"]["part"],
             explode=(0.0, 0.0, 0.22))
    add("P-AVHATCH", "U", o["P-AVHATCH"].buffer(-GAP, join_style=2), f0600, f1110, pads=up_pads(f0600, f1110),
        step=STEP["close"], parent=nu.key, explode=(0.0, 0.0, 0.4))
    nu.lands.append(_strip(nu.region, o["P-AVHATCH"], sc, "U", tg))
    # lower nose: keel slot (nose doors), side bays, front of the turret ring rebate
    ring = o["P-TURRETRING"]
    nl_reg = full(f0600 + GAP, f1110 - GAP).difference(unary_union(doors[:2]).buffer(GAP, join_style=2))
    nl_reg = nl_reg.difference(grow("P-SIDEBAY-L")).difference(grow("P-SIDEBAY-R"))
    nl_reg = nl_reg.difference(ring.buffer(-LAND_W, join_style=2))
    nl = add("P-NOSE-LOWER", "L", nl_reg, f0600, f1110, pads=lo_pads(f0600, f1110), step=STEP["skin"],
             parent=sc.st["FS0600"]["part"], explode=(0.0, 0.0, -0.22))
    for lid in ("P-SIDEBAY-L", "P-SIDEBAY-R"):
        add(lid, "L", o[lid].buffer(-GAP, join_style=2), 0.82, f1110, pads=lo_pads(0.8, f1110), step=STEP["close"],
            parent=nl.key, explode=(0.0, 0.0, -0.35))
        nl.lands.append(_strip(nl.region, o[lid], sc, "L", out[lid].t,
                               extra_cut=[ring.buffer(GAP + 0.001, join_style=2)]))
    # ---------------------------------------------------------------- mid body
    f1330, f1490 = st("FS1330"), st("FS1490")
    para = o["P-PARAHATCH"]
    mu = add("P-MID-UPPER", "U", full(f1110 + GAP, f1490 - GAP).difference(para.buffer(GAP_LIFT, join_style=2)),
             f1110, f1490, pads=up_pads(f1110, f1490), step=STEP["skin"], parent=sc.st["FS1110"]["part"],
             explode=(0.0, 0.0, 0.22))
    ml_reg = full(f1110 + GAP, f1490 - GAP).difference(grow("P-TDOORACC")).difference(
        ring.buffer(-LAND_W, join_style=2))
    ml = add("P-MID-LOWER", "L", ml_reg, f1110, f1490, pads=lo_pads(f1110, f1490), step=STEP["skin"],
             parent=sc.st["FS1110"]["part"], explode=(0.0, 0.0, -0.22))
    add("P-TDOORACC", "L", o["P-TDOORACC"].buffer(-GAP, join_style=2), f1330, f1490, pads=lo_pads(f1330, f1490),
        mirror=True, step=STEP["ring"], parent=ml.key, explode=(0.0, 0.05, -0.3))
    ml.lands.append(_strip(ml.region, o["P-TDOORACC"], sc, "L", t, extra_cut=[ring.buffer(GAP + 0.001,
                                                                                       join_style=2)]))
    ml.lands.append(_strip(ml.region, mirror_poly(o["P-TDOORACC"]), sc, "L", t,
                           extra_cut=[ring.buffer(GAP + 0.001, join_style=2)]))
    f1810 = st("FS1810")
    ps = add("P-PARA-SURR", "U", full(f1490 + GAP, f1810 - GAP).difference(para.buffer(GAP_LIFT, join_style=2))
             .difference(fair2),
             f1490, f1810, pads=up_pads(f1490, f1810), step=STEP["skin"], parent=sc.st["FS1490"]["part"],
             explode=(0.0, 0.0, 0.22))
    add("P-PARA-LOWER", "L", full(f1490 + GAP, f1810 - GAP), f1490, f1810, pads=lo_pads(f1490, f1810),
        step=STEP["skin"], parent=sc.st["FS1490"]["part"], explode=(0.0, 0.0, -0.22))
    # ---------------------------------------------------------------- mission bay
    ffuel = st("FS-FUEL")
    spine = o["P-SPINE"]
    # the tear-away strip starts 0.2 mm aft of the hatch edge (layout: 1 mm, side by side on the FS1810 flange)
    hx1, (sx0_, _sx1), (sy0_, sy1_) = float(para.bounds[2]), S["P-SPINE"]["x"], S["P-SPINE"]["y"]
    sp_ext = rect(hx1 + GAP_LIFT, sx0_ + GAP + 0.001, sy0_ + GAP, sy1_ - GAP)
    mb_reg = full(f1810 + GAP, ffuel - GAP).difference(para.buffer(GAP_LIFT, join_style=2)).difference(
        spine.buffer(GAP, join_style=2)).difference(sp_ext.buffer(GAP, join_style=2)).difference(
        grow("P-GNSS2")).difference(fair2)
    mb = add("P-MB-UPPER", "U", mb_reg, f1810, ffuel, pads=up_pads(f1810, ffuel), step=STEP["skin"],
             parent=sc.st["FS1810"]["part"], explode=(0.0, 0.0, 0.22))
    add("P-GNSS2", "U", o["P-GNSS2"].buffer(-GAP, join_style=2), 2.03, 2.15, step=STEP["ant"], parent=mb.key,
        explode=(0.0, 0.0, 0.3))
    mb.lands.append(_strip(mb.region, o["P-GNSS2"], sc, "U", tg))
    mbl = add("P-MB-LOWER", "L", full(f1810 + GAP, ffuel - GAP).difference(grow("P-MBHATCH")), f1810, ffuel,
              pads=lo_pads(f1810, ffuel), step=STEP["skin"], parent=sc.st["FS1810"]["part"],
              explode=(0.0, 0.0, -0.22))
    add("P-MBHATCH", "L", o["P-MBHATCH"].buffer(-GAP, join_style=2), f1810, ffuel, pads=lo_pads(f1810, ffuel),
        step=STEP["close"], parent=mbl.key, explode=(0.0, 0.0, -0.4))
    mbl.lands.append(_strip(mbl.region, o["P-MBHATCH"], sc, "L", t))
    # ---------------------------------------------------------------- centre body (fuel bays, wells)
    fgear = st("FS-GEAR")
    fuel = [grow(k) for k in ("P-FUEL1", "P-FUEL2", "P-FUEL3")]
    cu_reg = full(ffuel + GAP, fgear - GAP).difference(unary_union(fuel)).difference(
        spine.buffer(GAP, join_style=2)).difference(fair2)
    cu_reg = clean(cu_reg.buffer(-0.002, join_style=2).buffer(0.002, join_style=2).intersection(cu_reg), 1e-6)
    cu = add("P-CENTRE-UPPER", "U", cu_reg, ffuel, fgear, pads=up_pads(ffuel, fgear), step=STEP["skin"],
             parent=sc.st["FS-FUEL"]["part"], explode=(0.0, 0.0, 0.22))
    sp = add("P-SPINE", "U", spine.buffer(-GAP, join_style=2).union(sp_ext), f1810, 2.82, pads=up_pads(f1810, 2.82),
             step=STEP["para"], parent=sc.ref("M-SPINE"), explode=(0.0, 0.0, 0.35))
    for fid in ("F-RISER-FWD", "F-RISER-AFT"):          # relief pocket over the bridle U-lugs (layout fitting boxes;
        lo_, hi_ = (np.asarray(v, float) for v in sc.fit[fid]["box"])   # frame flange widened to +-24.5 mm by the
        sp.cut.append(box3((lo_[0] - 0.0025, -0.032, -1.0), (hi_[0] + 0.0025, 0.032, hi_[2] + 0.0006)))  # chassis
    for k in ("P-FUEL1", "P-FUEL2", "P-FUEL3"):
        add(k, "U", o[k].buffer(-GAP, join_style=2), float(o[k].bounds[0]), float(o[k].bounds[2]),
            pads=up_pads(*o[k].bounds[0::2]), mirror=True, step=STEP["fuel"], parent=cu.key,
            explode=(0.0, 0.08, 0.3))
        y_f = float(o[k].bounds[3])
        for sgn in (1, -1):         # outboard edges: on the fairing's land (fairing_parts); fore / aft on the caps
            ob = rect(0.0, 5.0, y_f - FUEL_SPLIT - 0.0015, 1.0)
            cu.lands.append(_strip(cu.region, o[k] if sgn > 0 else mirror_poly(o[k]), sc, "U", t,
                                   extra_cut=[fair2.buffer(0.001, join_style=2), ob, mirror_poly(ob)]))
    for pn in (out["P-PARA-LOWER"], mbl):
        pn.cut.append(glove_root_cutter(sc))
    cl_reg = full(ffuel + GAP, fgear - GAP).difference(grow("P-PAYHATCH")).difference(
        unary_union(doors[2:]).buffer(GAP, join_style=2))      # refuel door opening: refuel_parts
    cl = add("P-CENTRE-LOWER", "L", cl_reg, ffuel, fgear, pads=lo_pads(ffuel, fgear), step=STEP["skin"],
             parent=sc.st["FS-FUEL"]["part"], explode=(0.0, 0.0, -0.22))
    cl.cut.append(glove_root_cutter(sc))
    for b in sc.fit["F-TRUNNION"]["bolts"]:      # spot-faces d12 x 0.8 over the gear-beam bolt tips (outboard face)
        if abs(float(b["axis"][1])) > 0.9:
            q = np.asarray(b["point"], float)
            for sg in (1.0, -1.0):
                a_ = (q[0], sg * (q[1] - 0.002), q[2])
                b_ = (q[0], sg * (q[1] + 0.0075), q[2])
                cl.cut.append(G.cylinder(0.006, a_, b_, n=32))
    add("P-PAYHATCH", "L", o["P-PAYHATCH"].buffer(-GAP, join_style=2), ffuel, 2.86, pads=lo_pads(ffuel, 2.86),
        step=STEP["close"], parent=cl.key, explode=(0.0, 0.0, -0.45))
    cl.lands.append(_strip(cl.region, o["P-PAYHATCH"], sc, "L", t))
    # ---------------------------------------------------------------- aft body
    f3480 = st("FS3480")
    xfw = st("FS3670")
    al = add("P-AFT-LOWER", "L", full(fgear + GAP, xfw - GAP).difference(grow("P-AFTHATCH")).difference(
        grow("P-STABACT")), fgear, xfw, pads=lo_pads(fgear, xfw), step=STEP["skin"], parent=sc.st["FS-GEAR"]["part"],
        explode=(0.0, 0.0, -0.22))
    add("P-AFTHATCH", "L", o["P-AFTHATCH"].buffer(-GAP, join_style=2), fgear, f3480, pads=lo_pads(fgear, f3480),
        step=STEP["close"], parent=al.key, explode=(0.0, 0.0, -0.4))
    al.lands.append(_strip(al.region, o["P-AFTHATCH"], sc, "L", t))
    add("P-STABACT", "L", o["P-STABACT"].buffer(-GAP, join_style=2), f3480, xfw, pads=lo_pads(f3480, xfw),
        mirror=True, step=STEP["close"], parent=al.key, explode=(0.0, 0.1, -0.35))
    for sgn in (1, -1):
        al.lands.append(_strip(al.region, o["P-STABACT"] if sgn > 0 else mirror_poly(o["P-STABACT"]), sc, "L", t))
    add("P-AFT-UPPER", "U", full(fgear + GAP, xfw - GAP), fgear, xfw, pads=up_pads(fgear, xfw), step=STEP["skin"],
        parent=sc.st["FS-GEAR"]["part"], explode=(0.0, 0.0, 0.22))
    return out


# =====================================================================================================================
# aft body strips, cowl (layout.shell.root_cut_lines, cowl_upper / cowl_lower / body_side panels)
# =====================================================================================================================
FUEL_SPLIT = LAND_W + GAP     # fuel-panel land: outboard band (fairing) / inboard part (centre skin)
ROOT_CLEAR = 0.003          # skin / strip cut-out round a tail-surface root (sealant fillet, fittings pass through)
X_AFT = 3.9635              # common aft end of the fin-root fairings, outer upper cowl pieces and stub strips (layout
#                             3.962 / 3.965): the centre cowl piece and the lower halves close the cowl aft of it
X_UPS_LAND = 3.92           # the land under the outer cowl piece stops ahead of the steep cowl closure
X_COWL = 3.668              # split aft skins / cowl: 1.5 mm ahead of the firewall aft face so that the cowl's
#                             forward screw row (M3) keeps 2.5 D to its edge on the 13 mm firewall edge angle


def root_polys(sc: SC):
    """Plan polygon of the fin root (starboard), (x, z) polygon of the stub root (starboard), plan polygon of the
    ventral root (layout.shell.root_cut_lines: pairs of points on both faces, chord fractions 0..1)."""
    R = sc.L["shell"]["root_cut_lines"]
    out = {}
    for k in ("fin", "stabilator_stub", "ventral"):
        P = np.asarray(R[k]["points"], float)
        a, b = P[0::2], P[1::2]
        ring = np.vstack([a, b[::-1]])
        if k == "stabilator_stub":
            out[k] = Polygon(ring[:, [0, 2]]).buffer(0)
        else:
            out[k] = Polygon(ring[:, :2]).buffer(0)
    return out


def inlet_opening() -> Polygon:
    """Plan of the flush NACA-type cooling inlet opening (curved diverging walls, 30 -> 85 mm half width, throat lip
    at x 3.40): P-INLET frames it, the S-duct (YK250-PR-546, propulsion) forms the ramp below it."""
    xs = np.linspace(3.205, 3.400, 40)
    hw = 0.030 + 0.055 * ((xs - 3.205) / 0.195) ** 1.5
    return Polygon(np.vstack([np.column_stack([xs, -hw]), np.column_stack([xs[::-1], hw[::-1]])])).buffer(0)


BAND_LI, BAND_LO = 0.012, 0.03   # stub-band trim lines: reach inside / outside the OML along the normal


def stub_band(sc: SC) -> dict:
    """Stub-root strip band (layout P-STUBROOT: y >= y0 and z_band) trimmed normal to the skin: in every section the
    OML arc where the band holds is bounded by two lines along the surface normal (the strip and its neighbours get
    square edges, no feather edges where the z planes would cut the inclined side). 'keep': between the lines less
    the gap (the strip), 'clear': plus the gap (cut from the neighbours); both sides."""
    if "band" in sc._cache:
        return sc._cache["band"]
    sb = sc.pan["P-STUBROOT"]
    (sx0, _sx1), (sy0, _sy1), (sz0, sz1) = sb["x"], sb["y"], sb["z_band"]
    rk, rc = [], []
    xs = np.arange(sx0 - 0.002, X_AFT + GAP + 0.004, 0.004)
    last = None
    for x in xs:
        R = SG.resample_ring(sc.sec(float(x)), 3000, start_dir=(0.0, -1.0))
        R = R[:int(np.argmax(R[:, 1])) + 1]                       # starboard half: bottom -> top
        ok = np.where((R[:, 1] >= sz0) & (R[:, 1] <= sz1) & (R[:, 0] >= sy0))[0]
        if len(ok) < 3:
            if last is not None:
                break
            continue
        ib, it = int(ok[0]), int(ok[-1])

        def frame(i):
            a, b = R[max(i - 2, 0)], R[min(i + 2, len(R) - 1)]
            tau = _unit(b - a)
            return R[i], tau, np.array([-tau[1], tau[0]])
        pb, tb, nb = frame(ib)
        pt, tt, nt = frame(it)
        for out, g in ((rk, -GAP), (rc, GAP)):
            qb, qt = pb - g * tb, pt + g * tt
            Q = np.array([qb - BAND_LO * nb, qb + BAND_LI * nb, qt + BAND_LI * nt, qt - BAND_LO * nt])
            out.append(np.column_stack([np.full(4, x), Q[:, 0], Q[:, 1]]))
        last = x
    res = {}
    for k, rings in (("keep", rk), ("clear", rc)):
        m = G.fix_orientation(G.loft(rings))
        res[k] = G.union([m, m.mirrored_y()])
    res["x1"] = float(last)
    sc._cache["band"] = res
    return res


def aft_panels(sc: SC, pans: dict) -> None:
    S = sc.pan
    t = sc.t_skin
    st = lambda sid: float(sc.st[sid]["x"])                                           # noqa: E731
    roots = root_polys(sc)
    fin = roots["fin"].buffer(ROOT_CLEAR, join_style=2)
    fin2 = unary_union([fin, mirror_poly(fin)])
    stub_xz = roots["stabilator_stub"].buffer(ROOT_CLEAR, join_style=2)
    stub_cut = [prism_y(stub_xz, 0.18, 0.40)]
    stub_cut.append(stub_cut[0].mirrored_y())
    ven = roots["ventral"].buffer(ROOT_CLEAR, join_style=2)
    up_pads = lambda x0, x1: land_union(sc, "U", pads=True).intersection(rect(x0 - 0.05, x1 + 0.05, -1, 1))  # noqa
    lo_pads = lambda x0, x1: land_union(sc, "L", pads=True).intersection(rect(x0 - 0.05, x1 + 0.05, -1, 1))  # noqa
    fgear, xfw = st("FS-GEAR"), st("FS3670")
    num = lambda lid: int(S[lid]["part"].split("-")[2])                               # noqa: E731
    # stub-root strip band (body_side z band) as a box: the aft skins and cowl pieces are cut round it
    sb = S["P-STUBROOT"]
    (sx0, sx1), (sy0, sy1), (sz0, sz1) = sb["x"], sb["y"], sb["z_band"]
    band = stub_band(sc)
    stub_box = [band["clear"]] + stub_cut
    # --- aft skins: trim at the cowl split, cut round the inlet, fin-root strips, stub strip, ventral strip
    au, al = pans["P-AFT-UPPER"], pans["P-AFT-LOWER"]
    fr = S["P-FINROOT"]
    fr_rect = rect(fr["x"][0], fr["x"][1], fr["y"][0], fr["y"][1])
    vr = S["P-VENTRALROOT"]
    vr_rect = rect(vr["x"][0], vr["x"][1], vr["y"][0], vr["y"][1])
    inl = S["P-INLET"]
    inl_rect = rect(inl["x"][0], inl["x"][1], inl["y"][0], inl["y"][1])
    au.region = clean(rect(fgear + GAP, X_COWL - GAP, -1, 1).difference(inl_rect.buffer(GAP, join_style=2))
                      .difference(unary_union([fr_rect, mirror_poly(fr_rect)]).buffer(GAP, join_style=2)), 1e-6)
    au.x1 = X_COWL
    au.cut += stub_box
    au.lands.append(_strip(au.region, inl_rect, sc, "U", t))
    al.region = clean(al.region.intersection(rect(0, X_COWL - GAP, -1, 1)).difference(
        vr_rect.buffer(GAP, join_style=2)), 1e-6)
    al.x1 = X_COWL
    al.cut += stub_box
    # --- inlet lip (bonded on the joggled land of P-AFT-UPPER)
    pans["P-INLET"] = Pan(key="P-INLET", num=num("P-INLET"), name=S["P-INLET"]["name"],
                          name_tr=S["P-INLET"]["name_tr"], surf="U",
                          region=clean(inl_rect.buffer(-GAP, join_style=2).difference(inlet_opening()), 1e-6),
                          t=t, x0=inl["x"][0], x1=inl["x"][1], step=STEP["skin"], parent=au.key,
                          explode=(0.0, 0.0, 0.3), land_refs=("P-AFT-UPPER",))
    # --- fin-root strips (fairing, removable) round the fin root
    pans["P-FINROOT"] = Pan(key="P-FINROOT", num=num("P-FINROOT"), name=fr["name"], name_tr=fr["name_tr"],
                            surf="U", region=clean(fr_rect.buffer(-GAP, join_style=2).difference(fin), 1e-6),
                            mirror=True, t=t, x0=fr["x"][0], x1=X_COWL, step=STEP["fin"], parent=au.key,
                            explode=(0.0, 0.05, 0.3), pads=up_pads(*fr["x"]), removable=True,
                            land_refs=("M-DORSAL", "ST-FS3480", "ST-FS3670"))
    pans["P-FINROOT"].region = clean(pans["P-FINROOT"].region.intersection(rect(0, X_COWL - GAP, -1, 1)), 1e-6)
    au.lands.append(_strip(au.region, fr_rect, sc, "U", t))
    au.lands.append(_strip(au.region, mirror_poly(fr_rect), sc, "U", t))
    # --- ventral-root strip: machined solid CFRP fairing collar round the ventral root (bonded on the keel lands)
    pans["P-VENTRALROOT"] = Pan(key="P-VENTRALROOT", num=num("P-VENTRALROOT"), name=vr["name"],
                                name_tr=vr["name_tr"], surf="L",
                                region=clean(vr_rect.buffer(-GAP, join_style=2).difference(ven), 1e-6),
                                layup=None, thickness=t, t=t, x0=vr["x"][0], x1=vr["x"][1], step=STEP["ventral"],
                                parent=sc.ref("M-VENTRALKEEL"), explode=(0.0, 0.0, -0.3),
                                pads=lo_pads(*vr["x"]), land_refs=("M-VENTRALKEEL", "M-AFTKEEL"),
                                notes="solid laminate collar (5.8 mm, 29 plies PW) - the 3 mm wide sides of the "
                                      "44 mm strip cannot take a sandwich core")
    # --- stub-root strips (body side z band, removable for stub removal)
    pans["P-STUBROOT"] = Pan(key="P-STUBROOT", num=num("P-STUBROOT"), name=sb["name"], name_tr=sb["name_tr"],
                             surf="F", region=rect(sx0 + GAP, X_AFT - GAP, 0.0, 1.0), mirror=True, t=t,
                             x0=sx0, x1=X_AFT, step=STEP["stub"], parent=sc.st["FS3480"]["part"],
                             explode=(0.0, 0.3, 0.0), removable=True, keep=[band["keep"]],
                             pads=up_pads(sx0, sx1).union(lo_pads(sx0, sx1)), cut=[stub_cut[0]],
                             land_refs=("ST-FS3480", "ST-FS3670"))
    # --- cowl (x 3.668 .. 4.0): split by the layout outlines and z bands round the fin roots and the stub strips;
    #     the fin-root fairing, the outer upper pieces and the stub strips end on one line X_AFT (layout 3.962 / 3.965)
    cu, cus, clo, fra = S["P-COWL-UP"], S["P-COWL-UPS"], S["P-COWL-LO"], S["P-FINROOT-AFT"]
    xe = 4.0
    y_up = -float(cu["outline"][0][1])             # 0.125: centre piece / fin-root fairing split
    y_ups = float(fra["y"][1])                     # 0.197: fin-root fairing / outer piece split
    y_lo = float(clo["y"][0])                      # 0.022: lower halves / ventral strip split
    below_band = box3((X_COWL - 0.1, -1.0, -1.0), (xe + 0.1, 1.0, sz0 + GAP))
    pans["P-COWL-UP"] = Pan(key="P-COWL-UP", num=num("P-COWL-UP"), name=cu["name"], name_tr=cu["name_tr"],
                            surf="F", region=unary_union([rect(X_COWL + GAP, X_AFT - GAP, -(y_up - GAP), y_up - GAP),
                                                          rect(X_AFT + GAP, xe + 0.01, -1.0, 1.0)]),
                            t=t, x0=X_COWL, x1=xe, step=STEP["close"], parent=sc.st["FS3670"]["part"],
                            explode=(0.15, 0.0, 0.35), removable=True, cut=[below_band],
                            pads=up_pads(X_COWL, xe), land_refs=tuple(cu["lands"]))
    pans["P-FINROOT-AFT"] = Pan(key="P-FINROOT-AFT", num=num("P-FINROOT-AFT"), name=fra["name"],
                                name_tr=fra["name_tr"], surf="U",
                                region=clean(rect(X_COWL + GAP, X_AFT - GAP, y_up + GAP, y_ups - GAP).difference(
                                    fin), 1e-6), mirror=True, t=t, x0=X_COWL, x1=X_AFT, step=STEP["fin"],
                                parent=sc.st["FS3670"]["part"], explode=(0.0, 0.05, 0.35),
                                pads=up_pads(X_COWL, X_AFT), land_refs=("ST-FS3670",))
    # joggled land under the side edges of the centre cowl piece (its Camloc rows)
    cup_o = rect(X_COWL, X_AFT, -y_up, y_up)
    pans["P-FINROOT-AFT"].lands.append(_strip(pans["P-FINROOT-AFT"].region, cup_o, sc, "U", t,
                                              extra_cut=[rect(-1.0, 10.0, -1.0, y_up - LAND_W - GAP - 0.001),
                                                         rect(X_UPS_LAND, 5.0, -1.0, 1.0)]))
    # joggled land under the inboard edge of the aluminium outer piece (its Camloc row)
    ups_o = rect(X_COWL, X_AFT, y_ups, 0.40)
    pans["P-FINROOT-AFT"].lands.append(_strip(pans["P-FINROOT-AFT"].region, ups_o, sc, "U", T_AL,
                                              extra_cut=[rect(X_UPS_LAND, 5.0, -1.0, 1.0)]))
    # outer upper piece: above the stub band's upper trim line (above the chine aft of the band's end)
    pans["P-COWL-UPS"] = Pan(key="P-COWL-UPS", num=num("P-COWL-UPS"), name=cus["name"], name_tr=cus["name_tr"],
                             surf="U", region=rect(X_COWL + GAP, X_AFT - GAP, y_ups + GAP, 1.0), mirror=True,
                             material=cus["material"], layup=None, thickness=T_AL, t=T_AL, process=P_SHEET,
                             x0=X_COWL, x1=X_AFT, step=STEP["close"], parent=sc.st["FS3670"]["part"],
                             explode=(0.0, 0.3, 0.2), removable=True, cut=list(stub_box), keep_box=False,
                             land_refs=tuple(cus["lands"]), relief=False, t_normal=0.5 * (sz1 + 0.30))
    # lower halves: below the band's lower trim line / the chine ahead of X_AFT, below z_band[0] aft of it
    below = box3((X_AFT, -1.0, sz0 - GAP), (xe + 0.1, 1.0, 1.0))
    pans["P-COWL-LO"] = Pan(key="P-COWL-LO", num=num("P-COWL-LO"), name=clo["name"], name_tr=clo["name_tr"],
                            surf="F", region=rect(X_COWL + GAP, xe + 0.01, y_lo + GAP, 1.0), mirror=True, t=t,
                            x0=X_COWL, x1=xe, step=STEP["close"], parent=sc.ref("M-AFTKEEL"),
                            explode=(0.05, 0.15, -0.35), removable=True, pads=lo_pads(X_COWL, xe),
                            cut=[below, sc.half("U", X_COWL - 0.01, X_AFT, gap=-GAP)] + list(stub_box),
                            land_refs=tuple(clo["lands"]))
    kb = np.asarray(sc.mem["M-AFTKEEL"]["box"], float)        # aft keel channel (layout box) + bond line
    pans["P-VENTRALROOT"].cut.append(box3(kb[0] - 0.0003, kb[1] + 0.0003))
    pans["P-VENTRALROOT"].x1 = xe
    pans["P-VENTRALROOT"].region = clean(rect(float(vr["x"][0]) + GAP, xe + 0.01, -(y_lo - GAP), y_lo - GAP)
                                         .difference(ven), 1e-6)
    st_ = pans["P-STUBROOT"]
    st_.region = rect(sx0 + GAP, X_AFT - GAP, 0.0, 1.0)
    st_.x1 = X_AFT
    st_.cut = [stub_cut[0]]


# =====================================================================================================================
# parachute hatch, turret ring insert, refuel door, wing-root fairing
# =====================================================================================================================
TONGUE = (0.030, 0.003, 0.018)   # locating tongue: length along y, thickness along x, depth below the hatch inner face
TONGUE_Y = 0.120                 # tongue centre |y| (above the container top, clear of the bay walls)
TONGUE_CLR = 0.0005              # tongue face to frame web face
HINGE_R = 0.0018                 # refuel-door knuckle tube radius (flush with the OML at the door ends)
HINGE_PIN_R = 0.0008             # hinge pin d 1.6 mm (stainless)
HINGE_BORE = 0.00085             # knuckle / skin bores for the pin
HINGE_PIN_END = 0.005            # pin ends engaged in the skin beyond the door sides


def para_hatch(sc: SC, pans: dict) -> Pan:
    o = outlines(sc)
    S = sc.pan["P-PARAHATCH"]
    t = sc.t_skin
    reg = o["P-PARAHATCH"].buffer(-GAP_LIFT, join_style=2)
    x0, x1 = float(o["P-PARAHATCH"].bounds[0]), float(o["P-PARAHATCH"].bounds[2])
    tongues = []
    xa = float(sc.x_faces("FS1490")[1]) + TONGUE_CLR
    xb = float(sc.x_faces("FS1810")[0]) - TONGUE_CLR
    for xt in ((xa, xa + TONGUE[1]), (xb - TONGUE[1], xb)):
        for s in (1, -1):
            yc = s * TONGUE_Y
            ys = np.linspace(yc - TONGUE[0] / 2, yc + TONGUE[0] / 2, 7)
            zo = [float(sc.z_top(xx, yy)) for xx in xt for yy in ys]
            box = box3((xt[0], yc - TONGUE[0] / 2, min(zo) - 1.3 * t - TONGUE[2]),
                       (xt[1], yc + TONGUE[0] / 2, max(zo) + 0.004))
            tongues.append(man_and(box, sc.env(t - OV, xt[0] - 0.01, xt[1] + 0.01)))
    lp = land_union(sc, "U").intersection(rect(x0 - 0.05, x1 + 0.05, -1, 1))
    pn = Pan(key="P-PARAHATCH", num=int(S["part"].split("-")[2]), name=S["name"], name_tr=S["name_tr"], surf="U",
             region=reg, t=t, x0=x0, x1=x1, step=STEP["para"], removable=True, parent=pans["P-PARA-SURR"].key,
             explode=(0.0, 0.0, 0.45), pads=lp, extra=[m for m in tongues if m is not None],
             joint="para_hatch", land_refs=tuple(S["lands"]),
             notes="no hinge: four locating tongues (against the FS1490 / FS1810 web faces) and the corner pins of "
                   "the pin-puller latch YK250-SY-804; lifted straight off by the deploying canopy pack, 1.5 m tether")
    pans["P-PARAHATCH"] = pn
    return pn


def ring_parts(sc: SC, pans: dict) -> None:
    """Turret aperture ring insert (2.0 mm solid CFRP) in the rebate of the lower skins."""
    o = outlines(sc)
    S = sc.pan["P-TURRETRING"]
    T = sc.S["payload"]["turret"]
    r_open = 0.5 * float(T["ball_diameter"]) + float(T["bay"]["aperture_ring"]["radial_clearance"])
    xc = float(T["bay_center_x"])
    ring = o["P-TURRETRING"]
    reg = clean(ring.buffer(-GAP, join_style=2).difference(Point(xc, 0.0).buffer(r_open, 96)), 1e-6)
    x0, x1 = float(ring.bounds[0]), float(ring.bounds[2])
    t_ring = float(S.get("thickness_m") or T_RING)
    pans["P-TURRETRING"] = Pan(key="P-TURRETRING", num=int(S["part"].split("-")[2]), name=S["name"],
                               name_tr=S["name_tr"], surf="L", region=reg, layup=None, thickness=t_ring, t=t_ring,
                               x0=x0, x1=x1, step=STEP["ring"], removable=True, parent=pans["P-MID-LOWER"].key,
                               explode=(0.0, 0.0, -0.5), land_refs=tuple(S["lands"]))
    # rebate in the surrounding skins: outer 2.0 mm + seal line removed over the ring outline (+ gap)
    for k in ("P-NOSE-LOWER", "P-MID-LOWER"):
        pn = pans[k]
        xa, xb = pn.x0 - 0.01, pn.x1 + 0.01
        pz = prism_z(ring.buffer(GAP, join_style=2))
        cutter = man_sub(man_and(box3((xa, -0.5, -0.5), (xb, 0.5, 0.5)), pz), [sc.env(t_ring + SEAL, xa - 0.01,
                                                                                    xb + 0.01)])
        pn.cut.append(cutter)


def refuel_axis(sc: SC):
    """Hinge line of the refuel door: the chord of the door's forward OML edge offset 1.9 mm inward (layout axis point
    and direction are for a flat belly; at |y| 0.29-0.35 the belly is the curved chine corner, 14 mm drop)."""
    S = sc.pan["P-REFUEL"]
    h = S["hinge"]
    xa = float(h["axis_point"][0])
    ya, yb = sorted(S["y"])
    pa, na = sc.oml_point(xa, ya, "L")
    pb, nb = sc.oml_point(xa, yb, "L")
    depth = float(h["axis_point"][2]) - float(sc.oml_point(xa, float(h["axis_point"][1]), "L")[0][2])
    q0 = pa - depth * na
    q1 = pb - depth * nb
    d = _unit(q1 - q0)
    return q0, q1, d, depth


def refuel_parts(sc: SC, pans: dict) -> dict:
    """Refuel access door (hinged, flush) with its knuckle tube, the cove in the lower skin, the joggled land on the
    other three sides and the hinge pin."""
    S = sc.pan["P-REFUEL"]
    t = sc.t_skin
    O_ = lay_outline(sc, "P-REFUEL")
    q0, q1, d, depth = refuel_axis(sc)
    ya, yb = sorted(S["y"])
    # cove radius: every door point that can swing forward of the hinge line stays inside it. The straight axis
    # lies a = 2.0 .. 4.3 mm inside the curved belly; opened to theta the door's outer face crosses the OML level
    # a tan(theta / 2) ahead of the axis, at the radius a / cos(theta / 2); the hinge-edge corners swing on their own
    # radius (an elastomer cove seal closes the slot ahead of the door, layout P-REFUEL hinge)
    th = math.radians(float(S["hinge"]["range_deg"][1]))
    w_o = []
    for yy in np.linspace(ya, yb, 13):
        p, n = sc.oml_point(float(S["x"][0]), float(yy), "L")
        c = q0 + d * float(np.dot(p - q0, d))
        a_ = float(np.linalg.norm(p - c))
        w_o.append(a_ / math.cos(0.5 * th))
        for q in (p, p - t * n):                  # outer and inner corner of the door's hinge edge
            c = q0 + d * float(np.dot(q - q0, d))
            w_o.append(float(np.linalg.norm(q - c)))
    r_cove = 1.02 * max(w_o) + GAP
    # door ends and opening ends normal to the hinge axis (the axis follows the belly curve, 13 deg to y): a door end
    # in a y plane would swing into the skin beyond it
    door_reg = rect(O_.bounds[0] + GAP, O_.bounds[2] - GAP, ya - 0.006, yb + 0.006)
    slab_door = G.cylinder(0.15, q0 + GAP * d, q1 - GAP * d, n=64)
    slab_open = G.cylinder(0.15, q0 - GAP * d, q1 + GAP * d, n=64)
    x0, x1 = float(O_.bounds[0]), float(O_.bounds[2])
    tube_ends = (q0 + d * (GAP + 0.0002), q1 - d * (GAP + 0.0002))
    tube = man_sub(G.cylinder(HINGE_R, tube_ends[0], tube_ends[1], n=32), [G.cylinder(
        HINGE_BORE, q0 - 0.01 * d, q1 + 0.01 * d, n=24)])
    door = Pan(key="P-REFUEL", num=int(S["part"].split("-")[2]), name=S["name"], name_tr=S["name_tr"], surf="L",
               region=door_reg, t=t, x0=x0 - 0.01, x1=x1, step=STEP["fuel"], removable=True,
               parent=pans["P-CENTRE-LOWER"].key, explode=(0.0, -0.05, -0.3), joint="refuel_door",
               extra=[tube], cut=[G.cylinder(HINGE_BORE, q0 - 0.02 * d, q1 + 0.02 * d, n=24)], keep=[slab_door],
               land_refs=tuple(S["lands"]), notes="flush hinge: knuckle tube on the door nose, pin in the skin")
    pans["P-REFUEL"] = door
    # lower skin: cove round the hinge line over the door width, pin bores beyond, joggled land on 3 sides
    cl = pans["P-CENTRE-LOWER"]
    cove = G.cylinder(r_cove, q0 - (GAP + 0.0001) * d, q1 + (GAP + 0.0001) * d, n=48)
    opening = man_and(prism_z(rect(O_.bounds[0] - GAP, O_.bounds[2] + GAP, ya - 0.006, yb + 0.006)), slab_open)
    opening = man_sub(opening, [sc.env(t + 0.0001, x0 - 0.02, x1 + 0.02)])     # skin layer only (lands stay)
    cl.cut += [opening, cove, G.cylinder(HINGE_BORE, q0 - (HINGE_PIN_END + 0.003) * d,
                                         q1 + (HINGE_PIN_END + 0.003) * d, n=24)]
    hinge_side = rect(x0 - 0.05, x0 + 0.004, ya - 0.05, yb + 0.05)
    cl.lands.append(_strip(cl.region, O_, sc, "L", t, extra_cut=[hinge_side]))
    pin = (q0 - HINGE_PIN_END * d, q1 + HINGE_PIN_END * d)
    return {"axis": (q0, q1, d), "pin": pin, "r_cove": r_cove}


def chine_keepout(sc: SC, x0: float, x1: float) -> G.Mesh | None:
    """Envelope of the chine longeron (layout M-CHINE paths, J w x h about the path, out to the wing root) inside
    the union OML inset by 6.3 mm, grown 0.3 mm: inside the junction fairing the J's top flange lies at the skin line
    only near the fairing nose (thin glove root profile); the fairing's lands and pads keep off it."""
    key = ("ck", round(x0, 4), round(x1, 4))
    if key in sc._env:
        return sc._env[key]
    m = sc.mem["M-CHINE"]
    w, h = float(m["section"]["w"]), float(m["section"]["h"])
    ms = []
    for P in m["paths"]:
        P = np.asarray(P, float)
        a, b = max(x0, float(P[0, 0])), min(x1, float(P[-1, 0]))
        if b <= a + 1e-4:
            continue
        rings = []
        for x in np.linspace(a, b, max(2, int(math.ceil((b - a) / 0.004)) + 1)):
            yc, zc = float(np.interp(x, P[:, 0], P[:, 1])), float(np.interp(x, P[:, 0], P[:, 2]))
            y0, y1, z0, z1 = yc - 0.5 * w - 0.0003, sc.y_root + 0.01, zc - 0.5 * h - 0.0003, zc + 0.5 * h + 0.0003
            rings.append(np.array([[x, y0, z0], [x, y1, z0], [x, y1, z1], [x, y0, z1]]))
        r = G.fix_orientation(G.loft(rings))
        ms += [r, r.mirrored_y()]
    if not ms:
        sc._env[key] = None
        return None
    sc._env[key] = man_and(man_add(ms), sc.env(0.0063, x0 - 0.01, x1 + 0.01))
    return sc._env[key]


def fairing_parts(sc: SC, pans: dict) -> Pan:
    """Wing-root junction fairing (layout.shell.wing_root_fairing): glove root profile extruded inboard to the crease
    with the body, bonded on the SOB-rib flange and the centre-box cover (1.0 mm solid over the box); over the fuel
    bays it reaches the fuel-panel outboard edge (SC.fair_region). It owns the joggled land under the crease edge of
    the mission-bay upper skin and under the outboard edges of the fuel-bay panels."""
    W = sc.wrf
    t = sc.layup_t("wing_skin_primary")
    reg = clean(sc.fair_region().buffer(-GAP, join_style=2), 1e-6)
    x0, x1 = sc.gx0, float(sc.st["FS-GEAR"]["x"])
    gl = Glove(sc)
    junction = prism_z(sc.fairing_plan(0.0, 1.0))
    below_j = man_and(sc.body_env(t, x0 - 0.01, x1 + 0.01), junction, box3((x0 - 0.1, -1, -1), (x1 + 0.1, 1, 0.0155)))
    pn = Pan(key="P-WRF", num=int(W["part"].split("-")[2]), name="wing-root junction fairing",
             name_tr="kanat kökü birleşim kaplaması", surf="U", region=reg, mirror=True, layup="wing_skin_primary",
             t=t, x0=x0, x1=x1, step=STEP["glove"], parent=sc.ref("M-SOB"), explode=(0.0, 0.2, 0.25),
             pads=land_union(sc, "U", pads=True).intersection(rect(x0, x1, -1, 1)),
             land_refs=("M-SOB", "M-CTBOX", "M-CHINE"), cut=[below_j, gl.rib_env("SOB", 0.0002)],
             notes=W.get("construction", ""), relief=False)
    ck = chine_keepout(sc, x0 - 0.005, x1 + 0.005)
    if ck is not None:
        pn.cut.append(ck)
    # land under the mission-bay skin's crease edge (FS1810 .. FS-FUEL)
    xf = float(sc.st["FS-FUEL"]["x"])
    crease = LineString(sc.crease[sc.crease[:, 0] <= xf - 0.002])
    band = crease.buffer(LAND_W + GAP, cap_style=2, join_style=2)
    fside = sc.fairing_plan(0.0, 1.0)
    under_body = band.difference(fside).difference(land_union(sc, "U", 0.0015)).intersection(rect(x0, xf - 0.002, 0,
                                                                                                    1))
    reach = band.intersection(fside).intersection(reg)
    pn.lands.append((clean(under_body.union(reach), 2e-6), clean(reach, 2e-6), sc.t_skin))
    o = outlines(sc)
    y_f = float(np.asarray(sc.pan["P-FUEL1"]["outline"], float)[:, 1].max())
    for k in ("P-FUEL1", "P-FUEL2", "P-FUEL3"):     # the outboard edge band only (fore / aft edges: centre skin)
        pn.lands.append(_strip(reg, o[k], sc, "U", sc.t_skin, extra_cut=[rect(0.0, 5.0, -1.0, y_f - FUEL_SPLIT)]))
    pans["P-WRF"] = pn
    return pn


# =====================================================================================================================
# LERX / glove skins (primary wing skins, bonded), joint access panel, rear-pin bayonet cap
# =====================================================================================================================
Y_GLOVE = (0.3995, 0.6995)      # glove skins between the SOB rib web and the joint plane (0.5 mm seal line each)
RIVET_PITCH = 0.150             # peel-stopper blind rivets at the panel ends (layout.shell.panels P-GLOVE-*)
RIVET_D = 3.2                   # their diameter: the end-rib T-flanges are 20 mm wide (2.5 D each side of a 4 mm
#                                 rivet would need 20.4 mm), 1/8 in (3.2 mm) CherryMAX class
RIB_FL = 0.020                  # glove rib T-flange width (chassis rib convention, 1.6 mm)
REAR_HOLE_D, REAR_CAP_D, REAR_CB = 0.018, 0.030, 0.0022   # rear-pin port: hole, cap, counterbore depth
REAR_CAP_T, REAR_SPIGOT = 0.0020, (0.0085, 0.0065, 0.0035)  # cap disc, spigot (outer r, inner r, length)


def _mono(chain: np.ndarray):
    """(x, z) of a surface chain from leading to trailing edge, made monotonic in x for interpolation."""
    x = np.maximum.accumulate(chain[:, 0])
    k = np.r_[True, np.diff(x) > 1e-9]
    return x[k], chain[k, 1]


class Glove:
    """Wing-loft envelopes of the glove (streamwise sections inset in the section plane, the chassis glove-rib
    convention) and the centre-box cap keep-outs in the glove."""

    def __init__(self, sc: SC):
        self.sc = sc
        self.w = sc.wing
        self._env = {}
        self.ys = self._stations()

    def _stations(self):
        y0, y1 = Y_GLOVE
        e = [float(v) for v in self.w.span_coords() if y0 - 0.02 < v < y1 + 0.02]
        ys = sorted(set(np.round(np.r_[np.linspace(y0 - 0.006, y1 + 0.006, 41), e], 6).tolist()))
        return ys

    def poly(self, y: float, d: float = 0.0) -> Polygon:
        sc = self.sc
        poly, fr = SG.section2d(self.w, max(abs(y), sc.y_root), n=240)
        P = np.asarray(poly.exterior.coords)
        Q = SG.to3d(fr, P)
        reg = Polygon(np.column_stack([Q[:, 0], Q[:, 2]])).buffer(0)
        if d > 0:
            reg = SG.largest(reg.buffer(-d, join_style=2))
        return SG.largest(reg)

    def env(self, d: float) -> G.Mesh:
        key = round(d, 7)
        if key not in self._env:
            rings = []
            for y in self.ys:
                P2 = SG.resample_ring(self.poly(y, d), 240, start_dir=(1.0, 0.0))
                rings.append(np.column_stack([P2[:, 0], np.full(len(P2), y), P2[:, 1]]))
            self._env[key] = G.fix_orientation(G.loft(rings))
        return self._env[key]

    def layer(self, d0: float, d1: float) -> G.Mesh:
        key = ("l", round(d0, 7), round(d1, 7))
        if key not in self._env:
            self._env[key] = man_sub(self.env(d0), [self.env(d1)])
        return self._env[key]

    def chord_half(self, upper: bool) -> G.Mesh:
        """Solid above (upper) / below the camber line of every section (the upper / lower skin split; the reflexed
        root sections have lower surfaces above the chord line near the trailing edge)."""
        key = ("h", upper)
        if key not in self._env:
            rings = []
            nc = 48
            for y in self.ys:
                P = np.asarray(self.poly(y).exterior.coords)[:-1]
                i_le, i_te = int(np.argmin(P[:, 0])), int(np.argmax(P[:, 0]))
                R = np.roll(P, -i_le, axis=0)
                j = (i_te - i_le) % len(P)
                ca, cb = R[:j + 1], np.vstack([R[j:], R[:1]])[::-1]
                xa, xb = R[0, 0], R[j, 0]
                xs = xa + (xb - xa) * 0.5 * (1.0 - np.cos(np.linspace(0.0, np.pi, nc)))
                za = np.interp(xs, *_mono(ca))
                zb = np.interp(xs, *_mono(cb))
                zm = 0.5 * (za + zb)
                ext = 0.3
                k0 = (zm[1] - zm[0]) / max(xs[1] - xs[0], 1e-9)
                k1 = (zm[-1] - zm[-2]) / max(xs[-1] - xs[-2], 1e-9)
                k0, k1 = np.clip([k0, k1], -0.3, 0.3)
                zz = 1.0 if upper else -1.0
                g = GAP if upper else -GAP
                line = np.vstack([[xa - ext, zm[0] - k0 * ext], np.column_stack([xs, zm]),
                                  [xb + ext, zm[-1] + k1 * ext]])
                ring = np.vstack([line + [0.0, g], [[xb + ext, zz], [xa - ext, zz]]])
                rings.append(np.column_stack([ring[:, 0], np.full(len(ring), y), ring[:, 1]]))
            self._env[key] = G.fix_orientation(G.loft(rings))
        return self._env[key]

    def rib_outline(self, which: str) -> Polygon:
        """Glove-rib outline in its plane (x, z) by the rib convention of the chassis (layout M-SOB / M-GLOVERIB /
        M-JOINTRIB, contour wing_loft): glove section inset under the upper skin by the LERX root skin + bond line
        (layups.lerx_skin_upper_root 8.0 + 0.5 mm) and 6.5 mm (6.0 + 0.5) over the lower half (z < 0); the SOB rib
        reaches 6.5 mm under the upper skin aft of its rear-cap relief as well."""
        sc = self.sc
        m = sc.mem[{"SOB": "M-SOB", "GLOVE": "M-GLOVERIB", "JOINT": "M-JOINTRIB"}[which]]
        y = 0.5 * (float(m["box"][0][1]) + float(m["box"][1][1]))
        d_up = sc.layup_t("lerx_skin_upper_root") + 0.0005
        d_lo = sc.layup_t("wing_skin_primary") + 0.0005
        up, lo = self.poly(y, d_up), self.poly(y, d_lo)
        reg = up.union(lo.intersection(rect(-10, 10, -1.0, 0.0)))
        if which == "SOB":
            x_te = float(sc.xr(y + 0.024)) + 0.5 * sc.w_rear + 0.0005
            reg = reg.union(lo.intersection(rect(x_te, 10, -1.0, 1.0)))
        return SG.largest(reg.buffer(0))

    def rib_env(self, which: str, grow: float) -> G.Mesh:
        """The rib (web + 20 mm T-flanges: SOB outboard, glove rib both sides, joint rib inboard) as the outline
        prism over the flange span, grown by ``grow`` (bond line)."""
        key = ("rib", which, round(grow, 6))
        if key not in self._env:
            sc = self.sc
            m = sc.mem[{"SOB": "M-SOB", "GLOVE": "M-GLOVERIB", "JOINT": "M-JOINTRIB"}[which]]
            ya, yb = float(m["box"][0][1]), float(m["box"][1][1])
            ya, yb = {"SOB": (ya, yb + RIB_FL), "GLOVE": (ya - RIB_FL, yb + RIB_FL), "JOINT": (ya - RIB_FL, yb)}[which]
            poly = clean(self.rib_outline(which).buffer(grow, join_style=2).intersection(
                rect(float(m["box"][0][0]) - 0.01, float(m["box"][1][0]) + 0.06, -1, 1)), 1e-8)
            self._env[key] = prism_y(poly, ya - grow, yb + grow)
        return self._env[key]

    def cap_keepout(self, upper: bool) -> G.Mesh:
        """Box spar caps in the glove (layout M-CTBOX spar lines, cap centroid z and thickness, widths): skins keep
        the bond line above the cap outer face (skin_solid_over_caps 1.0 mm = 0.9 mm laminate + 0.1 mm bond)."""
        key = ("cap", upper)
        if key in self._env:
            return self._env[key]
        sc = self.sc
        B = sc.mem["M-CTBOX"]
        ms, rs = sc.ms_line, sc.rs_line
        mz, mt = np.asarray(B["main_spar_caps_z"], float), np.asarray(B["main_spar_caps_t"], float)
        rz, rt = np.asarray(B["rear_spar_caps_z"], float), np.asarray(B["rear_spar_caps_t"], float)
        w_fork = float(sc.S["structures"]["sizing"]["wing_joint"]["fork"]["cap_width_m"])
        j = 1 if upper else 0
        sgn = 1.0 if upper else -1.0
        ys = np.linspace(0.395, 0.705, 63)
        meshes = []
        for line, zz, tt, wfn in ((ms, mz, mt, lambda y: float(np.interp(y, [0.0, 0.4034, 0.43, 1.0],
                                                                          [sc.w_main, sc.w_main, w_fork, w_fork]))),
                                  (rs, rz, rt, lambda y: sc.w_rear)):
            nx = 7
            P = np.zeros((nx, len(ys), 3))
            for k, y in enumerate(ys):
                xc = float(np.interp(y, line[:, 1], line[:, 0]))
                w = wfn(y) + 0.003
                zc = float(np.interp(y, line[:, 1], zz[:, j]))
                t = max(float(np.interp(y, line[:, 1], tt)), 0.0009)
                face = zc + sgn * 0.5 * t
                xs = np.linspace(xc - 0.5 * w, xc + 0.5 * w, nx)
                # cap face never closer than 1 mm to the loft (layout skin_solid_over_caps_m)
                Pl = self.poly(y)
                zo = []
                for xx in xs:
                    g = LineString([(xx, -1), (xx, 1)]).intersection(Pl)
                    if g.is_empty:
                        zo.append(face)
                        continue
                    c = np.asarray(g.coords) if hasattr(g, "coords") else np.vstack(
                        [np.asarray(q.coords) for q in g.geoms])
                    zo.append(float(c[:, 1].max()) - 0.001 if upper else float(c[:, 1].min()) + 0.001)
                zf = np.minimum(face, zo) if upper else np.maximum(face, zo)
                P[:, k, 0], P[:, k, 1], P[:, k, 2] = xs, y, zf + sgn * BOX_BOND
            inward = np.zeros_like(P)
            inward[..., 2] = -sgn
            Q = P if upper else P[:, ::-1]
            meshes.append(G.shell_from_grid(Q, 0.08, inward=inward if upper else inward[:, ::-1]))
        self._env[key] = man_add(meshes)
        return self._env[key]


def glove_rib_pads(sc: SC, gl: Glove, upper: bool):
    """Plan bands of the glove-rib T-flanges (SOB outboard, glove rib both sides, joint rib inboard) with the flange
    depth from the skin line: upper 8.5 mm (6.5 aft of the SOB rear-cap relief), lower 6.5 mm (chassis rib outline)."""
    out = []
    sob = sc.mem["M-SOB"]["box"]
    gr = sc.mem["M-GLOVERIB"]["box"]
    jr = sc.mem["M-JOINTRIB"]["box"]
    for (lo, hi), (ya, yb) in ((sob, (sob[0][1], sob[1][1] + RIB_FL)), (gr, (gr[0][1] - RIB_FL, gr[1][1] + RIB_FL)),
                               (jr, (jr[0][1] - RIB_FL, jr[1][1]))):
        out.append(rect(lo[0], hi[0] + 0.06, ya, yb))
    return unary_union(out)


def glove_parts(sc: SC, pans: dict) -> Glove:
    gl = Glove(sc)
    S = sc.pan
    up, lo = S["P-GLOVE-UP"], S["P-GLOVE-LO"]
    y0, y1 = Y_GLOVE
    t_up = sc.layup_t("wing_skin_primary")
    t_box = sc.layup_t("wing_box_skin_upper")
    t_lerx = sc.layup_t("lerx_skin_upper_root")
    plan = rect(1.7, 3.1, y0, y1)
    # thickness zones of the upper skin (structures.sizing.wing): LERX first bay ahead of the main cap (SOB -> glove
    # rib), box skin between the caps, primary skin elsewhere
    ysz = np.linspace(y0, y1, 31)
    xm_lo = np.array([float(sc.xm(y)) - 0.5 * sc.w_main for y in ysz])
    xr_hi = np.array([float(sc.xr(y)) + 0.5 * sc.w_rear for y in ysz])
    xm_hi = np.array([float(sc.xm(y)) + 0.5 * sc.w_main for y in ysz])
    xr_lo = np.array([float(sc.xr(y)) - 0.5 * sc.w_rear for y in ysz])
    y_gr = float(sc.mem["M-GLOVERIB"]["box"][0][1]) + 0.5 * (sc.mem["M-GLOVERIB"]["box"][1][1] -
                                                              sc.mem["M-GLOVERIB"]["box"][0][1])
    box_zone = Polygon(np.vstack([np.column_stack([xm_hi, ysz]), np.column_stack([xr_lo, ysz])[::-1]])).buffer(0)
    k = ysz <= y_gr
    lerx_zone = Polygon(np.vstack([np.column_stack([np.full(k.sum(), 1.7), ysz[k]]),
                                   np.column_stack([xm_lo[k], ysz[k]])[::-1]])).buffer(0)

    def skin(upper: bool):
        half = gl.chord_half(upper)
        prz = prism_z(plan)
        if upper:
            parts = [man_and(gl.layer(0.0, t_up), half, prz),
                     man_and(gl.layer(0.0, t_box), half, prism_z(box_zone.intersection(plan))),
                     man_and(gl.layer(0.0, t_lerx), half, prism_z(lerx_zone.intersection(plan)))]
        else:
            parts = [man_and(gl.layer(0.0, t_up), half, prz)]
        # bond pads on the rib flanges: filled down to the rib outline + 0.2 mm bond line (rib_env)
        ribs = glove_rib_pads(sc, gl, upper).intersection(plan)
        parts.append(man_and(gl.layer(t_up - OV, 0.0095), half, prism_z(ribs)))
        m = man_add(parts)
        return man_sub(m, [gl.cap_keepout(upper)] + [gl.rib_env(w, 0.0002) for w in ("SOB", "GLOVE", "JOINT")])

    def gmesh(upper: bool, cuts=()):
        m = skin(upper)
        m = man_sub(m, list(cuts))
        return finish(pieces_above(m, 2e-8))

    pans["P-GLOVE-UP"] = Pan(key="P-GLOVE-UP", num=int(up["part"].split("-")[2]), name=up["name"],
                             name_tr=up["name_tr"], surf="U", region=plan, mirror=True, layup="wing_skin_primary",
                             t=t_up, x0=1.7, x1=3.1, step=STEP["glove"], parent=sc.ref("M-SOB"),
                             explode=(0.0, 0.25, 0.3), land_refs=tuple(up["lands"]),
                             notes="zones: LERX first bay lerx_skin_upper_root, box wing_box_skin_upper, 1.0 mm solid "
                                   "over the caps (structures.sizing.wing)")
    pans["P-GLOVE-UP"].mesh_override = lambda: gmesh(True)
    # lower skin: framed cut-out for the joint access panel, rear-pin port (hole + counterbore)
    ja = lay_outline(sc, "P-JOINTACCESS")
    jP = np.asarray(ja.exterior.coords)[:-1]
    jP[np.isclose(jP[:, 1], ja.bounds[3], atol=1e-4), 1] = y1           # outboard edge on the joint plane
    jP[np.isclose(jP[:, 1], ja.bounds[1], atol=1e-4), 1] = float(sc.mem["M-SOB"]["box"][0][1]) + 0.0034
    ja = Polygon(jP).buffer(0)
    ra = lay_outline(sc, "P-REARACCESS")
    rc = np.array(ra.centroid.coords[0])
    big = 0.3
    rcuts = [G.cylinder(0.5 * REAR_HOLE_D, (rc[0], rc[1], -big), (rc[0], rc[1], big), n=48)]
    zlo = float(LineString([(rc[0], -1), (rc[0], 1)]).intersection(gl.poly(rc[1])).bounds[1])
    # counterbore / cap: D-shaped, flat forward edge 0.5 mm aft of the rear-cap trailing edge (the skin over the cap
    # stays the 1.0 mm solid laminate)
    x_cte = float(sc.xr(rc[1])) + 0.5 * sc.w_rear + 0.0005
    aft_of_cap = box3((x_cte, rc[1] - 0.05, zlo - 0.06), (rc[0] + 0.05, rc[1] + 0.05, zlo + 0.05))
    rcuts.append(man_and(G.cylinder(0.5 * REAR_CAP_D + GAP, (rc[0], rc[1], zlo - 0.05), (rc[0], rc[1], zlo + REAR_CB),
                                    n=64), aft_of_cap))
    jcut = man_and(prism_z(ja.buffer(GAP, join_style=2)), box3((1.7, 0.3, -1), (3.1, 0.75, 1)))
    pans["P-GLOVE-LO"] = Pan(key="P-GLOVE-LO", num=int(lo["part"].split("-")[2]), name=lo["name"],
                             name_tr=lo["name_tr"], surf="L", region=plan, mirror=True, layup="wing_skin_primary",
                             t=t_up, x0=1.7, x1=3.1, step=STEP["glove"], parent=sc.ref("M-SOB"),
                             explode=(0.0, 0.25, -0.3), land_refs=tuple(lo["lands"]))
    # joggled land of the access cut-out (doubler on the inner face of the lower skin, away from the caps / ribs)
    band = ja.buffer(LAND_REACH, join_style=2).difference(ja.buffer(-LAND_W, join_style=2))
    band = band.difference(glove_rib_pads(sc, gl, False).buffer(0.0015, join_style=2))
    t_ja = sc.t_skin
    half = gl.chord_half(False)
    land = man_and(gl.layer(t_ja + SEAL, t_ja + SEAL + LAND_T), half, prism_z(band))
    reach = man_and(gl.layer(t_up - OV, t_ja + SEAL + OV), half, prism_z(band.difference(ja.buffer(GAP,
                                                                                                    join_style=2))))
    ribs_all = [gl.rib_env(w, 0.0002) for w in ("SOB", "GLOVE", "JOINT")]
    pans["P-GLOVE-LO"].mesh_override = lambda: finish(pieces_above(man_add([
        man_sub(man_add([skin(False), reach]), [gl.cap_keepout(False), jcut] + rcuts + ribs_all),
        man_sub(land, [gl.cap_keepout(False)] + rcuts + ribs_all)]), 2e-8))      # (the land runs under the panel)
    pans["P-GLOVE-LO"].ja = ja
    # joint access panel (non-structural cover, layups.shell_secondary, flush in the joggled land)
    jaS = S["P-JOINTACCESS"]
    jreg = ja.buffer(-GAP, join_style=2)

    def ja_mesh():
        m = man_and(gl.layer(0.0, t_ja), gl.chord_half(False), prism_z(jreg))
        pads = glove_rib_pads(sc, gl, False).intersection(jreg.buffer(-0.0005, join_style=2))
        m = man_add([m, man_and(gl.layer(t_ja - OV, 0.0095), gl.chord_half(False), prism_z(pads))])
        return finish(pieces_above(man_sub(m, [gl.cap_keepout(False)] + [gl.rib_env(w, 0.0002) for w in
                                                                            ("SOB", "GLOVE", "JOINT")]), 2e-8))
    pans["P-JOINTACCESS"] = Pan(key="P-JOINTACCESS", num=int(jaS["part"].split("-")[2]), name=jaS["name"],
                                name_tr=jaS["name_tr"], surf="L", region=jreg, mirror=True, t=t_ja,
                                x0=float(ja.bounds[0]), x1=float(ja.bounds[2]), step=STEP["close"], removable=True,
                                parent=pans["P-GLOVE-LO"].key, explode=(0.0, 0.1, -0.35),
                                land_refs=tuple(jaS["lands"]))
    pans["P-JOINTACCESS"].mesh_override = ja_mesh
    # rear-pin bayonet cap: flush disc in the counterbore, spigot in the 18 mm hole
    raS = S["P-REARACCESS"]

    def cap_mesh():
        z_in = zlo + REAR_CAP_T
        disc = G.cylinder(0.5 * REAR_CAP_D - GAP, (rc[0], rc[1], zlo - 0.004), (rc[0], rc[1], z_in), n=64)
        disc = man_and(disc, gl.env(0.0), box3((x_cte + GAP, rc[1] - 0.05, zlo - 0.06),
                                                (rc[0] + 0.05, rc[1] + 0.05, zlo + 0.05)))
        spig = man_sub(G.cylinder(REAR_SPIGOT[0], (rc[0], rc[1], z_in - OV), (rc[0], rc[1], z_in + REAR_SPIGOT[2]),
                                  n=48),
                       [G.cylinder(REAR_SPIGOT[1], (rc[0], rc[1], z_in), (rc[0], rc[1], z_in + 0.02), n=48),
                        box3((rc[0] - 0.05, rc[1] - 0.05, zlo - 0.06), (x_cte + GAP, rc[1] + 0.05, zlo + 0.05))])
        return finish(man_add([disc, spig]))
    pans["P-REARACCESS"] = Pan(key="P-REARACCESS", num=int(raS["part"].split("-")[2]), name=raS["name"],
                               name_tr=raS["name_tr"], surf="L", region=ra, mirror=True, layup=None,
                               thickness=REAR_CAP_T, t=REAR_CAP_T, x0=float(ra.bounds[0]), x1=float(ra.bounds[2]),
                               step=STEP["joint"], removable=True, parent=pans["P-GLOVE-LO"].key,
                               explode=(0.0, 0.0, -0.25), land_refs=tuple(raS["lands"]),
                               notes="flush bayonet cap d 30 mm (2.0 mm solid CFRP + spigot) in a 2.2 mm counterbore "
                                     "round the d 18 mm port")
    pans["P-REARACCESS"].mesh_override = cap_mesh
    return gl


# =====================================================================================================================
# fastener rows (layout.shell.rules.concept, panels[*].fastening): laid out on the chassis lands from the layout,
# every hole proven on the geometry (Fix.prove) before it is drilled
# =====================================================================================================================
ROW_KIND = {"nutplate+screw": ("nut", PITCH_NUT), "camloc": ("cam", PITCH_CAM), "insert+screw": ("nut", PITCH_INS),
            "bonded": ("riv", RIVET_PITCH)}
EDGE = {"nut": EDGE_M4, "cam": EDGE_CAM, "riv": EDGE_M4}
OWN_MARGIN = 0.0015             # plan margin added to the edge distance when a row point is given to a panel (the
#                                 edge distance is measured at the laminate mid-plane, the panel edges are cut plumb)
NO_ROWS = ("P-PARAHATCH", "P-REFUEL", "P-REARACCESS", "P-SPINE", "P-NOSECONE", "P-GLOVE-UP", "P-GLOVE-LO",
           "P-JOINTACCESS", "P-VENTRALROOT")
DENSE = 0.002                   # dense sampling step of a row line before the pitch is laid out


def row_kind(sc: SC, key: str):
    """(kind, pitch) of a panel's fastener rows from its layout fastening type (fairings: M4 screws; their lands
    are 1.6 mm solid laminate, too thin for the 1.2 D thread of a potted insert, so floating nutplates are used)."""
    if key == "P-WRF":
        return "riv", RIVET_PITCH
    p = sc.pan.get(key)
    if not p:
        return None
    t = (p.get("fastening") or {}).get("type")
    if t not in ROW_KIND:
        return None
    kind, pitch = ROW_KIND[t]
    if key.startswith("P-COWL") and kind == "cam":
        pitch = PITCH_COWL
    if key.startswith("P-FUEL"):
        pitch = PITCH_FUEL
    return kind, pitch


def _resample(P, N, pitch: float, centre_sym: bool = False):
    """Indices of a dense polyline P (k, 3) at ``pitch`` spacing along its arc length, centred in the run;
    ``centre_sym``: the run starts on the centre line and continues on the other side (positions +-pitch/2, ...)."""
    s = np.r_[0.0, np.cumsum(np.linalg.norm(np.diff(P, axis=0), axis=1))]
    L = float(s[-1])
    if centre_sym:
        pos = np.arange(0.5 * pitch, L + 1e-9, pitch)
    else:
        n = int(math.floor(L / pitch + 1e-9)) + 1
        pos = 0.5 * (L - (n - 1) * pitch) + pitch * np.arange(n)
    return [int(np.argmin(np.abs(s - q))) for q in pos]


class Rows:
    """Lays out and drills the shell's fastener rows. Candidates are generated on the starboard half and mirrored;
    each symmetric pair is proven and committed together (Fix.try_group)."""

    def __init__(self, sc: SC, reg: Registry, pans: dict, keys: dict):
        from shapely.prepared import prep
        self.sc, self.reg, self.pans, self.keys = sc, reg, pans, keys
        self.fix = Fix(sc)
        self.own = []
        self.full = {}
        for key, pn in pans.items():
            rk = row_kind(sc, key)
            if rk is None or key in NO_ROWS:
                continue
            e = EDGE[rk[0]] + OWN_MARGIN
            regs = [(keys[key], pn.region)]
            if pn.mirror:
                regs.append((keys[key][:-1] + "L", mirror_poly(pn.region)))
            for pid, rg in regs:
                inner = clean(rg.buffer(-e, join_style=2), 1e-8)
                if not inner.is_empty:
                    self.own.append((pid, key, prep(inner), pn.surf, rk))
                    self.full[pid] = (key, prep(rg), pn.surf, rk)
        self.done = 0

    # ------------------------------------------------------------------ ownership
    def _pierced(self, pid: str, p, n) -> bool:
        h = G.ray_hits(self.fix.man(pid), p + 0.002 * n, p - 0.004 * n)
        return len(h) > 0 and abs(float(h[0]) - 0.002) < 0.0015

    def owner(self, p, n):
        """(part id, panel key, (kind, pitch)) of the panel whose plan region (less the edge distance) holds the OML
        point p and whose material is there."""
        x, y, z = (float(v) for v in p)
        zc = float(self.sc.zc(x))
        hits = []
        for pid, key, pr, surf, rk in self.own:
            if surf == "U" and z < zc + 0.001:
                continue
            if surf == "L" and z > zc - 0.001:
                continue
            if pr.contains(Point(x, y)):
                hits.append((pid, key, rk))
        if len(hits) > 1 or (hits and self.pans[hits[0][1]].surf == "F"):
            hits = [h for h in hits if self._pierced(h[0], p, n)]
        return hits[0] if hits else None

    # ------------------------------------------------------------------ candidates
    def _centre(self, key: str) -> bool:
        b = self.pans[key].region.bounds
        return b[1] < -0.001 and b[3] > 0.001

    def _mirror_pid(self, pid: str) -> str:
        if pid.endswith("-R"):
            return pid[:-1] + "L"
        return pid

    def _commit(self, pid: str, key: str, kind: str, p, a, orient, note: str, size: float = 4, step=None):
        st = step if step is not None else self.pans[key].step
        c1 = Cand(p=np.asarray(p, float), a=_unit(a), kind=kind, panel=pid, land="", size=size,
                  orient=_unit(orient), note=note, step=st)
        group = [c1]
        if abs(float(p[1])) > 0.001:
            q = np.array([p[0], -p[1], p[2]])
            aq = np.array([a[0], -a[1], a[2]])
            oq = np.array([orient[0], -orient[1], orient[2]])
            mp = self._mirror_pid(pid)
            if not pid.endswith(("-R", "-L")) and not self._centre(key):
                mp = None                           # one-sided panel: its mirror image is another part (or none)
                for pid2, (key2, pr2, surf2, _rk) in self.full.items():
                    if pid2 != pid and surf2 == self.pans[key].surf and pr2.contains(Point(float(q[0]),
                                                                                         float(q[1]))):
                        mp = pid2
                        break
            if mp is not None:
                group.append(Cand(p=q, a=_unit(aq), kind=kind, panel=mp, land="", size=size, orient=_unit(oq),
                                  note=note, step=st))
        if self.fix.try_group(group):
            self.done += len(group)
            return True
        return False

    # ------------------------------------------------------------------ row families
    def station_rows(self):
        """Rows on the frame T-caps: 16 mm either side of the web centre line for the panels ending on the frame,
        one row on the aft flange where a panel runs over the frame."""
        sc = self.sc
        for sid in sc.st:
            if sid in ("FS3738",):
                continue
            for half in ("U", "L"):
                curves = {}
                for s in (-1, 1):
                    P, N = sc.section_curve(lambda yy, s=s: sc.x_web(sid, yy) + s * ROW_CAP, half, n=1200)
                    keep = P[:, 1] >= -1e-4
                    P, N = P[keep], N[keep]
                    if half == "U":                 # the junction fairing's OML over the body (glove root)
                        fz = (P[:, 1] >= sc.y_in) & (P[:, 1] <= sc.y_root + 1e-3)
                        for i in np.where(fz)[0]:
                            P[i], N[i] = sc.oml_point(float(P[i, 0]), float(P[i, 1]), "U")
                    s_ = np.r_[0.0, np.cumsum(np.linalg.norm(np.diff(P, axis=0), axis=1))]
                    q = np.arange(0.0, s_[-1], DENSE)
                    idx = np.unique([int(np.argmin(np.abs(s_ - v))) for v in q])
                    curves[s] = (P[idx], N[idx])
                for s in (-1, 1):
                    P, N = curves[s]
                    T = np.gradient(P, axis=0)
                    T = T / np.maximum(np.linalg.norm(T, axis=1, keepdims=True), 1e-12)
                    zc = sc.zc(P[:, 0])
                    ok = np.abs(P[:, 2] - zc) > 0.012           # chine relief band + J corner
                    if half == "U":
                        ok &= ~((P[:, 1] < RIDGE_CLEAR))           # V-roof ridge
                    own = []
                    for i in range(len(P)):
                        o = self.owner(P[i], N[i]) if ok[i] else None
                        if o is not None and s == -1:          # panel running over the frame: aft flange only
                            Q, M = curves[1]
                            j = int(np.argmin(np.linalg.norm(Q[:, 1:] - P[i, 1:], axis=1)))
                            o2 = self.owner(Q[j], M[j])
                            if o2 is not None and o2[0] == o[0]:
                                o = None
                        own.append(o)
                    self._runs(P, N, T, own, f"{sid} cap row", centre_sym=True)

    def _runs(self, P, N, T, own, note, centre_sym=False, size: float = 4):
        k = 0
        while k < len(P):
            if own[k] is None:
                k += 1
                continue
            j = k
            while j + 1 < len(P) and own[j + 1] is not None and own[j + 1][0] == own[k][0]:
                j += 1
            pid, key, (kd, pt) = own[k]
            idx = np.arange(k, j + 1)
            if self.pans[key].surf == "F":          # wrap-round panels: plan regions do not bound the arc
                sl = np.r_[0.0, np.cumsum(np.linalg.norm(np.diff(P[idx], axis=0), axis=1))]
                e_ = EDGE[kd] + OWN_MARGIN
                idx = idx[(sl >= e_) & (sl <= sl[-1] - e_)]
                if not len(idx):
                    k = j + 1
                    continue
            sym = False
            if centre_sym and abs(float(P[j][1])) < 0.003 <= abs(float(P[k][1])):
                idx = idx[::-1]                     # the run ends on the centre line: lay it out from there
            if centre_sym and abs(float(P[idx[0]][1])) < 0.003:
                sym = True
            for i in _resample(P[idx], N[idx], pt, sym):
                ii = int(idx[i])
                self._commit(pid, key, kd, P[ii], -N[ii], T[ii], note, size=size)
            k = j + 1

    def line_rows(self, x0: float, x1: float, y_of_x, half: str, note: str, owner_fixed=None, pitch=None,
                  kind=None, size: float = 4):
        """Longitudinal row at plan y = y_of_x(x) on the upper / lower half."""
        sc = self.sc
        xs = np.arange(x0, x1 + 1e-9, DENSE)
        P, N = [], []
        for x in xs:
            p, n = sc.oml_point(float(x), float(y_of_x(x)), half)
            P.append(p)
            N.append(n)
        P, N = np.asarray(P), np.asarray(N)
        T = np.gradient(P, axis=0)
        T = T / np.maximum(np.linalg.norm(T, axis=1, keepdims=True), 1e-12)
        zc = sc.zc(P[:, 0])
        own = []
        for i in range(len(P)):
            if abs(P[i, 2] - zc[i]) <= 0.012:
                own.append(None)
                continue
            if owner_fixed is not None:
                own.append(owner_fixed)
            else:
                own.append(self.owner(P[i], N[i]))
        if pitch or kind:
            own = [None if o is None else (o[0], o[1], (kind or o[2][0], pitch or o[2][1])) for o in own]
        self._runs(P, N, T, own, note, size=size)

    def member_rows(self):
        sc = self.sc
        M = sc.mem
        # spine-channel flanges: the skins round the tear-away strip land on the outer part of the flanges
        x0, x1 = (float(v) for v in M["M-SPINE"]["lands"][0]["x"])
        self.line_rows(x0 + 0.004, x1 - 0.004, lambda x: 0.045 + GAP + EDGE_M4, "U", "M-SPINE flange row")
        # parachute-bay wall top flanges: surround skin outboard of the hatch
        ld = M["M-PARAWALL"]["lands"][0]
        self.line_rows(ld["x"][0] + 0.004, ld["x"][1] - 0.004, lambda x: 0.188 + GAP_LIFT + EDGE_M4, "U",
                       "M-PARAWALL flange row")
        # ventral keel hat flanges: aft lower skin either side of the ventral root strip
        ld = M["M-VENTRALKEEL"]["lands"][0]
        self.line_rows(ld["x"][0] + 0.004, ld["x"][1] - 0.004, lambda x: 0.037, "L", "M-VENTRALKEEL flange row")
        # keel-beam land: centre lower skin outboard of the payload hatch
        b = np.asarray(M["M-KEEL"]["box"], float)
        yk = float(b[0][1])
        self.line_rows(b[0][0] + 0.02, b[1][0] + 0.02, lambda x: yk + 0.0255 + GAP + EDGE_M4, "L",
                       "M-KEEL land row")
        # dorsal longeron hat flanges
        P = np.asarray(M["M-DORSAL"]["paths"][0], float)
        hw = 0.5 * float(M["M-DORSAL"]["section"]["w"])
        for yy in (float(P[0, 1]) - hw - 0.006, float(P[0, 1]) + hw + 0.006):
            self.line_rows(P[0, 0] + 0.004, P[-1, 0] - 0.004, lambda x, yy=yy: yy, "U", "M-DORSAL flange row")
        # chine longeron (J) top flange under the upper skins, outside the junction fairing
        for P in M["M-CHINE"]["paths"]:
            P = np.asarray(P, float)
            a, b_ = float(P[0, 0]) + 0.004, float(P[-1, 0]) - 0.004
            segs = [(a, min(b_, sc.gx0 - 0.03)), (max(a, sc.gx1 + 0.03), b_)]
            for s0, s1 in segs:
                if s1 > s0 + 0.03:
                    self.line_rows(s0, s1, lambda x, P=P: float(np.interp(x, P[:, 0], P[:, 1])) - 0.005, "U",
                                   "M-CHINE J flange row")

    def ring_rows(self):
        """Removable panels and fairings: a row 2.5 D + 0.5 mm inside the outline, on the joggled lands and member
        flanges (the frame-cap edges are covered by the cap rows)."""
        sc = self.sc
        o = outlines(sc)
        for key, pn in self.pans.items():
            rk = row_kind(sc, key)
            if rk is None or key in NO_ROWS or pn.surf not in ("U", "L") or not (pn.removable or
                                                                                sc.pan.get(key, {}).get("attach") ==
                                                                                "fairing" or key == "P-INLET"):
                continue
            kind, pitch = rk
            if not pn.mirror and pn.region.bounds[3] < 0.0:
                continue                        # port-only panel: drilled as the mirror image of its partner
            O_ = o.get(key, pn.region)
            ring = O_.buffer(-(EDGE[kind] + OWN_MARGIN), join_style=2)
            if ring.is_empty:
                continue
            for poly in _as_polys(ring):
                R = np.asarray(poly.exterior.coords)
                s_ = np.r_[0.0, np.cumsum(np.linalg.norm(np.diff(R, axis=0), axis=1))]
                q = np.arange(0.0, s_[-1], DENSE)
                xy = np.column_stack([np.interp(q, s_, R[:, 0]), np.interp(q, s_, R[:, 1])])
                on_cap = np.zeros(len(xy), bool)
                for sid in sc.st:                   # cap rows there; the joggled lands stop 1.5 mm short of the caps
                    on_cap |= np.abs(xy[:, 0] - sc.x_web(sid, xy[:, 1])) < 0.034
                if key.startswith("P-FUEL"):        # inner edge: M3 row on the spine flange (special_rows)
                    on_cap |= np.abs(xy[:, 1]) < 0.075
                P, N = [], []
                for x, y in xy:
                    p, n = sc.oml_point(float(x), float(y), pn.surf)
                    P.append(p)
                    N.append(n)
                P, N = np.asarray(P), np.asarray(N)
                T = np.gradient(P, axis=0)
                T = T / np.maximum(np.linalg.norm(T, axis=1, keepdims=True), 1e-12)
                pid = self.keys[key]
                own = [None if on_cap[i] else (pid, key, rk) for i in range(len(P))]
                # a closed ring: start the runs at a cap gap so that a run is not split at the seam
                if not on_cap.any():
                    self._runs(P, N, T, own, f"{key} edge row")
                else:
                    k0 = int(np.argmax(on_cap))
                    sh = lambda A: np.roll(A, -k0, axis=0)          # noqa: E731
                    self._runs(sh(P), sh(N), sh(T), list(np.roll(np.array(own, dtype=object), -k0)),
                               f"{key} edge row")

    def special_rows(self):
        sc = self.sc
        keys = self.keys
        # nose cone: 8 x M4 radial into nutplates on the FS0300 forward flange, 4 a side evenly along the section
        # arc (centre line -> chine -> keel), shifted off the chine relief band
        x = float(sc.x_web("FS0300", 0.0)) - ROW_CAP
        PU, NU = sc.section_curve(lambda yy: x, "U", n=1200)
        PL, NL = sc.section_curve(lambda yy: x, "L", n=1200)
        ku, kl = PU[:, 1] >= 0.0, PL[:, 1] >= 0.0
        P = np.vstack([PU[ku], PL[kl][::-1]])
        N = np.vstack([NU[ku], NL[kl][::-1]])
        s_ = np.r_[0.0, np.cumsum(np.linalg.norm(np.diff(P, axis=0), axis=1))]
        zc = float(sc.zc(x))
        i_ch = int(np.argmin(np.abs(P[:, 2] - zc) + 10.0 * (P[:, 1] < 0.5 * P[:, 1].max())))
        for f_ in (0.125, 0.375, 0.625, 0.875):
            q = f_ * s_[-1]
            if abs(q - s_[i_ch]) < 0.016:
                q = s_[i_ch] + math.copysign(0.016, q - s_[i_ch])
            i = int(np.argmin(np.abs(s_ - q)))
            t = np.cross([1.0, 0.0, 0.0], N[i])
            self._commit(keys["P-NOSECONE"], "P-NOSECONE", "nut", P[i], -N[i], t, "nose cone radial screw (FS0300)")
        # tear-away strip: 4 nylon M3 shear screws into the spine-channel flanges (inner part)
        lx = sc.mem["M-SPINE"]["lands"][0]["x"]
        for xx in (float(lx[0]) + 0.070, float(lx[1]) - 0.070):
            p, n = sc.oml_point(xx, 0.0325, "U")
            self._commit(keys["P-SPINE"], "P-SPINE", "nut", p, -n, [1.0, 0.0, 0.0],
                         "nylon M3 shear screw (tear-away)", size=3)
        # fuel-bay panels, inner edge: the spine-channel flange reaches 17.5 mm under the panel (as built), so the
        # row is M3 (2.5 D = 7.5 mm to the panel edge and to the flange edge) at 22 mm (<= 8 D)
        o = outlines(sc)
        for k in ("P-FUEL1", "P-FUEL2", "P-FUEL3"):
            b = o[k].bounds
            y_e = float(np.asarray(sc.pan[k]["outline"], float)[:, 1].min())
            self.line_rows(b[0] + 0.03, b[2] - 0.03, lambda x, y_e=y_e: y_e + GAP + 0.0085, "U",
                           f"{k} inner edge row (M3)", owner_fixed=(keys[k], k, ("nut", 0.022)), size=3)
        # mission-bay upper skin: crease edge on the junction fairing's joggled land
        xf = float(sc.st["FS-FUEL"]["x"])
        C = sc.crease[(sc.crease[:, 0] > float(sc.st["FS1810"]["x"]) + 0.03) & (sc.crease[:, 0] < xf - 0.03)]
        if len(C) > 3:
            self.line_rows(float(C[0, 0]), float(C[-1, 0]),
                           lambda x: float(np.interp(x, C[:, 0], C[:, 1])) - GAP - EDGE_M4, "U",
                           "crease row (fairing land)")
        # centre cowl piece: Camloc rows on the fin-root fairings' joggled lands along its side edges
        y_up = -float(sc.pan["P-COWL-UP"]["outline"][0][1])
        self.line_rows(X_COWL + 0.03, X_UPS_LAND - 0.02, lambda x: y_up - GAP - EDGE_CAM - OWN_MARGIN - 0.0005, "U",
                       "centre cowl edge row", owner_fixed=(keys["P-COWL-UP"], "P-COWL-UP", ("cam", PITCH_COWL)))
        # lower cowl halves: Camloc row on the aft-keel land flanges (layout: receptacles at y +-0.040)
        ld = sc.mem["M-AFTKEEL"]["lands"][0]
        self.line_rows(float(ld["x"][0]) + 0.012, float(ld["x"][1]) - 0.012, lambda x: 0.040, "L",
                       "aft-keel land row", owner_fixed=(keys["P-COWL-LO"], "P-COWL-LO", ("cam", PITCH_COWL)))

    def glove_rows(self, gl):
        """Peel-stopper blind rivets (M4, 150 mm) at the glove skin ends: on the SOB-rib and joint-rib flanges."""
        sc = self.sc
        keys = self.keys
        sob = sc.mem["M-SOB"]["box"]
        jr = sc.mem["M-JOINTRIB"]["box"]
        for y, upper in ((float(sob[1][1]) + 0.010, True), (float(sob[1][1]) + 0.010, False),
                         (float(jr[0][1]) - 0.010, True), (float(jr[0][1]) - 0.010, False)):
            poly = gl.poly(y)
            x0, _z0, x1, _z1 = poly.bounds
            key = "P-GLOVE-UP" if upper else "P-GLOVE-LO"
            xs = np.arange(x0 + 0.03, x1 - 0.03, DENSE)
            P, N = [], []
            for x in xs:
                r = self.glove_point(gl, float(x), y, upper)
                if r is not None:
                    P.append(r[0])
                    N.append(r[1])
            P, N = np.asarray(P), np.asarray(N)
            T = np.gradient(P, axis=0)
            T = T / np.maximum(np.linalg.norm(T, axis=1, keepdims=True), 1e-12)
            own = [(keys[key], key, ("riv", RIVET_PITCH))] * len(P)
            self._runs(P, N, T, own, "peel-stopper rivet row (3.2 mm: 20 mm rib flange)", size=RIVET_D)

    @staticmethod
    def glove_point(gl, x: float, y: float, upper: bool):
        """Glove OML point and outward normal (streamwise section slope; the glove is flat spanwise)."""
        poly = gl.poly(y)
        zz = []
        for xx in (x, x + 0.001):
            g = LineString([(xx, -1.0), (xx, 1.0)]).intersection(poly)
            if g.is_empty:
                return None
            zz.append(float(g.bounds[3] if upper else g.bounds[1]))
        dz = (zz[1] - zz[0]) / 0.001
        n = _unit([-dz, 0.0, 1.0]) if upper else _unit([dz, 0.0, -1.0])
        return np.array([x, y, zz[0]]), n

    def joint_access_rows(self, gl):
        """Wing-joint access panel: Camloc row 2.5 D + 1.5 mm inside its outline, on the glove lower skin's joggled
        land and the rib flanges."""
        key = "P-JOINTACCESS"
        ja = self.pans["P-GLOVE-LO"].ja
        ring = ja.buffer(-(EDGE_CAM + OWN_MARGIN), join_style=2)
        if ring.is_empty:
            return
        R = np.asarray(_as_polys(ring)[0].exterior.coords)
        s_ = np.r_[0.0, np.cumsum(np.linalg.norm(np.diff(R, axis=0), axis=1))]
        q = np.arange(0.0, s_[-1], DENSE)
        P, N = [], []
        for x, y in np.column_stack([np.interp(q, s_, R[:, 0]), np.interp(q, s_, R[:, 1])]):
            r = self.glove_point(gl, float(x), float(y), False)
            if r is not None:
                P.append(r[0])
                N.append(r[1])
        P, N = np.asarray(P), np.asarray(N)
        T = np.gradient(P, axis=0)
        T = T / np.maximum(np.linalg.norm(T, axis=1, keepdims=True), 1e-12)
        own = [(self.keys[key], key, ("cam", PITCH_CAM))] * len(P)
        self._runs(P, N, T, own, "joint access panel edge row")

    def run(self, gl):
        self.station_rows()
        self.member_rows()
        self.ring_rows()
        self.special_rows()
        self.glove_rows(gl)
        self.joint_access_rows(gl)
        return self.done


# =====================================================================================================================
# registration
# =====================================================================================================================
CONTACT_GAP = 0.00028           # declared contacts touch (analysis.checks CONTACT_TOL 0.3 mm)


def define_all(sc: SC) -> tuple[dict, dict]:
    pans = body_panels(sc)
    aft_panels(sc, pans)
    para_hatch(sc, pans)
    ring_parts(sc, pans)
    info = {"refuel": refuel_parts(sc, pans)}
    fairing_parts(sc, pans)
    info["glove"] = glove_parts(sc, pans)
    return pans, info


def _part_for(sc: SC, pn: Pan, side: str, cache: dict) -> Part:
    pid = sc.pid(pn.num, side)

    def mfn(pn=pn):
        if pn.key not in cache:
            cache[pn.key] = build_panel_mesh(sc, pn)
        return cache[pn.key]
    kw = dict(id=pid, name=pn.name + (" (starboard)" if side == "R" else ""), name_tr=pn.name_tr +
              (" (sağ)" if side == "R" else ""), group=GROUP, material=pn.material, process=pn.process,
              mesh_fn=mfn, thickness=pn.thickness, layup=pn.layup, side=side, joint=pn.joint, parent=pn.parent,
              step=pn.step, explode=tuple(pn.explode), notes=pn.notes, color=pn.color)
    if pn.areal:
        return LamPart(areal=pn.areal, t_lam=pn.t, **kw)
    return Part(**kw)


def register(reg: Registry, spec: dict) -> None:
    sc = SC(reg, spec)
    pans, info = define_all(sc)
    cache: dict = {}
    keys = {}
    # ------------------------------------------------------------------ joints (layout.mechanisms / shell hinge)
    jl = {j["name"]: j for j in sc.L["mechanisms"]["joints"]}["para_hatch"]
    reg.add_joint(Joint(name="para_hatch", kind=jl["kind"], origin=jl["origin"], axis=jl["axis"], lo=float(jl["lo"]),
                        hi=float(jl["hi"]), rest=float(jl.get("rest", 0.0)), prop=jl.get("prop", ""),
                        scale=float(jl.get("scale", 1.0)), notes=jl.get("notes", "")))
    h = sc.pan["P-REFUEL"]["hinge"]
    q0, q1, d = info["refuel"]["axis"]
    lo_, hi_ = (math.radians(float(v)) for v in h["range_deg"])
    reg.add_joint(Joint(name="refuel_door", kind="revolute", origin=0.5 * (q0 + q1), axis=d, lo=lo_, hi=hi_,
                        rest=0.0, prop="refuel_door_deg", scale=math.pi / 180.0,
                        notes="flush hinge on the forward edge (layout.shell.panels[P-REFUEL].hinge); + opens outward"))
    # ------------------------------------------------------------------ parts (starboard / centre), then port copies
    for key, pn in pans.items():
        side = "R" if pn.mirror else "C"
        part = _part_for(sc, pn, side, cache)
        reg.add(part)
        keys[key] = part.id
    for key, pn in pans.items():
        p = reg.parts[keys[key]]
        if p.parent in keys:
            p.parent = keys[p.parent]
    for key, pn in pans.items():
        if pn.mirror:
            pr = reg.parts[keys[key]]
            idl = pr.id[:-1] + "L"
            id_map = {k: k[:-1] + "L" for k in list(reg.parts) if k.endswith("-R")}
            reg.add(mirror_part(pr, idl, joint=pr.joint, id_map=id_map))
    # hinge pin of the refuel door (part of the purchased flush-hinge kit: stainless dowel in the skin bores)
    pa, pb = info["refuel"]["pin"]
    pin_id = sc.pid(386)
    reg.add(Part(id=pin_id, name="refuel door hinge pin d1.6 (ISO 2338 h8, A2)", name_tr="yakıt kapağı menteşe pimi",
                 group=GROUP, material="ss_304_annealed", process="purchased", purchased=True,
                 vendor="ISO 2338 1.6 h8 x 70 A2 (cut to length)", thickness=2 * HINGE_PIN_R,
                 mesh_fn=lambda a=pa, b=pb: G.cylinder(HINGE_PIN_R, a, b, n=20), parent=keys["P-CENTRE-LOWER"],
                 step=STEP["fuel"], explode=(0.0, -0.05, -0.3),
                 contacts=(keys["P-CENTRE-LOWER"], keys["P-REFUEL"])))
    sc.pans, sc.keys, sc.info = pans, keys, info
    _contacts(sc, reg)
    # ------------------------------------------------------------------ fastener rows (after the contacts: holes
    # do not change whether two parts touch)
    rows = Rows(sc, reg, pans, keys)
    n_f = rows.run(info["glove"])
    sc.rows = rows
    reg.note(f"shell: {len([p for p in reg.parts.values() if p.group == GROUP])} parts, {n_f} fasteners "
             f"({len(rows.fix.dropped)} candidate holes not drilled)")


def _contacts(sc: SC, reg: Registry) -> None:
    """Declared contacts: the layout lands of each panel and its touching neighbours, kept only where the geometry
    really touches (gap <= 0.28 mm)."""
    mine = [p for p in reg.parts.values() if p.group == GROUP]
    boxes = {}

    def bb(pid):
        if pid not in boxes:
            boxes[pid] = reg.parts[pid].base_mesh.bounds()
        return boxes[pid]
    mans = {}

    def man(pid):
        if pid not in mans:
            p = reg.parts[pid]
            mans[pid] = (p.base_mesh if p.group == GROUP else p.mesh).to_manifold()
        return mans[pid]
    others = [pid for pid, p in reg.parts.items() if p.group != "hardware" and p.process != "consumable"]
    for p in mine:
        lo, hi = bb(p.id)
        found = list(p.contacts)
        for q in others:
            if q == p.id or q in found:
                continue
            qlo, qhi = bb(q)
            if not (np.all(qhi + 0.0005 >= lo) and np.all(hi + 0.0005 >= qlo)):
                continue
            if float(man(p.id).min_gap(man(q), 0.001)) <= CONTACT_GAP:
                found.append(q)
        p.contacts = tuple(found)
