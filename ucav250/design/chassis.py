"""Chassis producer (YK-250 HANÇER): the primary structure that carries every primary load.

What it builds (part numbers from ``spec.layout.part_numbers.chassis``, ids from ``spec.layout``):

* the centre wing box YK250-CH-001 (root part: continuous UD main / rear spar caps from joint to joint, sandwich
  box covers, frame-land web doublers, glove rear-spar web with the slot-fitting pad), the CFRP fork prongs with bonded
  4130 bushes (CH-053), the centre-line kink rib and the two 7075 kink fittings, the rear-spar slot fittings, the
  side-of-body / glove / joint ribs of the glove box;
* the 13 frames / bulkheads of ``layout.stations`` (sandwich webs with a solid T-cap at the skin line = the land of
  the shell fasteners, every declared cut-out and longeron notch), the firewall stack (CFRP sandwich, stainless
  spacer tubes and stand-offs, 0.4 mm AISI 304 heat shield with its edge angle) and the metallic engine-bay U-ring;
* longerons, keel members, decks, walls, the dorsal spine channel (``layout.chassis.members``) with their splices,
  shear clips and notch crossings;
* all interface fittings of ``layout.chassis.fittings`` (main-gear trunnions, up-locks, nose-gear pivot blocks,
  stabilator nodes, firewall corner / lower engine-mount fittings with backing plates, fin / stub / ventral root
  fittings, parachute bridle U-lugs), the welded 4130 engine mount with its four isolator cups;
* fuel-bay liners, equipment trays, parachute container brackets, turret elevator rail anchors.

Interfaces read (never another module's geometry): ``spec.layout`` (root_part, part_numbers, stations, chassis.*,
rules.boxes, keep_outs KO-ENGINE), ``spec.fuselage`` / ``spec.wing`` (OML), ``spec.structures.sizing`` (sized
dimensions), ``spec.layups`` / ``spec.processes`` / ``spec.materials``.

Module-private detailing constants (reflected in ``docs/detail/chassis.md``):

* ``CAP_T`` 1.6 mm: solid T-cap (8 plies PW) of every composite frame at the skin line (the layout's 1.6 mm edge band),
  ``flange_w`` each side of the web; members that reach the skin stop at ``inset + CAP_T`` so the caps run through;
* ``FRAME_LAND`` 3.2 mm: 16-ply solid lands of fitting attachments (structures.sizing.body.frame_land_plies);
* sandwich panels are modelled at their full layup thickness everywhere (the real core is ramped out to a solid land
  of the same envelope at fastener lines);
* fastener holes are cut by ``design/joints.py`` (ISO 273 medium); bores of pins and bushings are cut here.
"""
from __future__ import annotations

import math
from functools import lru_cache

import numpy as np
from shapely.geometry import LineString, MultiPolygon, Point, Polygon
from shapely.geometry import box as sbox
from shapely.ops import unary_union

from ..core import geom as G
from ..core.parts import Part, Registry, layup_props, mirror_part, part_number
from . import joints as J
from . import oml as O
from . import structgen as SG

CAP_T = 0.0016              # frame T-cap thickness (solid edge band, 8 plies PW)
FRAME_LAND = 0.0032         # 16-ply solid land at fitting attachments
OV = 2e-4                   # boolean overlap of fused features (>= 0.1 mm, ARCHITECTURE §5)
RING_N = 160                # points per section ring for lofted envelopes
GROUP = "chassis"


# =====================================================================================================================
# small geometry helpers
# =====================================================================================================================
def rect(a0, a1, b0, b1) -> Polygon:
    return sbox(min(a0, a1), min(b0, b1), max(a0, a1), max(b0, b1))


def extrude_cs(poly, t: float, origin, u, v) -> G.Mesh:
    """Prism from a shapely (Multi)Polygon drawn in the (u, v) plane at ``origin``, extruded along u x v by ``t``.
    Uses the manifold3d cross-section triangulator (robust with collinear hole edges, unlike ear clipping)."""
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


def prism_x(poly, x0, x1) -> G.Mesh:
    """(y, z) polygon extruded along +x from x0 to x1."""
    return extrude_cs(poly, x1 - x0, (x0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0))


def prism_z(poly, z0, z1) -> G.Mesh:
    """(x, y) polygon extruded along +z from z0 to z1."""
    return extrude_cs(poly, z1 - z0, (0.0, 0.0, z0), (1.0, 0.0, 0.0), (0.0, 1.0, 0.0))


def prism_y(poly_xz, y0, y1) -> G.Mesh:
    """(x, z) polygon extruded along +y from y0 to y1 (the polygon is given in (x, z))."""
    return extrude_cs(poly_xz, y1 - y0, (0.0, y0, 0.0), (1.0, 0.0, 0.0), (0.0, 0.0, 1.0)).transformed(
        np.diag([1.0, -1.0, 1.0]), (0.0, 2 * y0, 0.0))


def _as_polys(g) -> list[Polygon]:
    if isinstance(g, Polygon):
        return [g]
    return [p for p in getattr(g, "geoms", []) if isinstance(p, Polygon) and p.area > 1e-10]


def box3(lo, hi) -> G.Mesh:
    lo, hi = np.asarray(lo, float), np.asarray(hi, float)
    return G.box(hi - lo, 0.5 * (lo + hi))


def obox(center, axes, half) -> G.Mesh:
    """Oriented box: ``axes`` rows are the unit local axes, ``half`` the half extents."""
    R = np.asarray(axes, float).T
    return G.box(2 * np.asarray(half, float), center, R=R)


def union(ms) -> G.Mesh:
    ms = [m for m in ms if m is not None]
    return ms[0] if len(ms) == 1 else G.union(ms)


def diff(a, cutters) -> G.Mesh:
    cutters = [c for c in cutters if c is not None]
    return a if not cutters else G.difference(a, cutters)


def inter(a, b) -> G.Mesh:
    r = G.intersection(a, b)
    if r is None:
        raise ValueError("empty intersection")
    return r


def largest_piece(m: G.Mesh) -> G.Mesh:
    """Largest connected solid of a boolean result (drops slivers cut off by an envelope)."""
    man = m.to_manifold()
    parts = man.decompose()
    if len(parts) <= 1:
        return m
    best = max(parts, key=lambda p: p.volume())
    return G.Mesh.from_manifold(best)


def pieces_above(m: G.Mesh, vmin: float = 2e-8) -> G.Mesh:
    """Drop disconnected slivers smaller than ``vmin`` (m^3) left by trims."""
    import manifold3d as m3
    allp = m.to_manifold().decompose()
    parts = [p for p in allp if p.volume() >= vmin]
    if not parts:
        raise ValueError("no piece left")
    if len(parts) == len(allp):
        return m
    return G.Mesh.from_manifold(parts[0] if len(parts) == 1 else m3.Manifold.compose(parts))


def shear_x(m: G.Mesh, k: float) -> G.Mesh:
    """x' = x + k * y (affine shear: a straight frame becomes a swept half-frame)."""
    V = m.V.copy()
    V[:, 0] += k * V[:, 1]
    return G.Mesh(V, m.F.copy())


CHEV_D = 0.0015             # half width of the straight kink piece of a chevron part (y = +-1.5 mm)


def chevron(build, sweep_deg: float) -> G.Mesh:
    """Chevron solid symmetric about y = 0 (frames / parts on the swept spar lines).

    ``build(y_lo, y_hi, dx_aft)`` returns the straight geometry restricted to y in [y_lo, y_hi] with every x range
    extended aft by ``dx_aft``. The starboard half (y >= CHEV_D - OV) is sheared by tan(sweep) and mirrored; a short
    straight kink piece (|y| <= CHEV_D + 0.5 mm, extended aft to cover both sheared halves) joins them (fusing two
    halves sheared in opposite senses directly leaves sliver triangles)."""
    k = math.tan(math.radians(sweep_deg))
    yo = CHEV_D + 0.0005
    h = shear_x(build(CHEV_D - OV, 1.0, 0.0), k)
    mid = build(-yo, yo, k * yo + OV)
    return G.union([h, h.mirrored_y(), mid])


def bore(p0, p1, r, n=40) -> G.Mesh:
    return G.cylinder(r, p0, p1, n=n)


def finish(m: G.Mesh) -> G.Mesh:
    """Final clean-up of a boolean result: merge sub-micron sliver edges (manifold simplify 0.1 um), so the mesh passes
    the triangle self-intersection test; volume change << 1 mm^3."""
    out = G.Mesh.from_manifold(m.to_manifold().simplify(1e-7))
    if out.check(self_intersect=True)["ok"]:
        return out
    out2 = G.Mesh.from_manifold(m.to_manifold().simplify(1e-6))
    return out2 if out2.check(self_intersect=True)["ok"] else out


def clean_poly(g, min_area=2e-7):
    """Drop slivers; return a Polygon or MultiPolygon."""
    ps = [p for p in _as_polys(g) if p.area >= min_area]
    if not ps:
        raise ValueError("empty polygon")
    return ps[0] if len(ps) == 1 else MultiPolygon(ps)


# =====================================================================================================================
# context
# =====================================================================================================================
class Ctx:
    def __init__(self, reg: Registry, spec: dict):
        self.reg = reg
        self.S = spec
        self.L = spec["layout"]
        self.fus = O.fuselage_from_spec(spec)
        self.wing = O.LiftingSurface(spec["wing"]["sections"], n_chord=80, name="wing")
        self.st = {s["id"]: s for s in self.L["stations"]}
        ch = self.L["chassis"]
        self.mem = {m["id"]: m for m in ch["members"]}
        self.fit = {f["id"]: f for f in ch["fittings"]}
        self.n0, self.n1 = self.L["part_numbers"]["chassis"]
        self.steps = {s["step"]: s for s in spec["assembly"]["steps"]}
        self._sec = {}
        self._env = {}

    # ---------------------------------------------------------------- ids
    def pid(self, num: int, side: str = "C", group: str = GROUP) -> str:
        if not self.n0 <= num <= self.n1:
            raise ValueError(f"part number {num} outside the chassis range")
        return part_number(group, num, side)

    def ref(self, lid: str, side: str = "R") -> str:
        """Part id of a layout object id (ST-*, M-*, F-*); mirrored objects get the side suffix."""
        if lid.startswith("ST-"):
            return self.st[lid[3:]]["part"]
        obj = self.mem.get(lid) or self.fit.get(lid)
        if obj is None:
            raise KeyError(lid)
        return obj["part"] + (f"-{side}" if obj.get("mirror") else "")

    # ---------------------------------------------------------------- OML
    def sec(self, x: float, inset: float = 0.0) -> Polygon:
        key = (round(float(x), 6), round(float(inset), 6))
        if key not in self._sec:
            self._sec[key] = SG.fuselage_section2d(self.fus, float(x), float(inset), n=256)
        return self._sec[key]

    def z_top(self, x, y):
        """Upper OML z at (x, y) of the body."""
        w, h, zc, nt, nb = self.fus.section(np.asarray(x, float))
        tf = self.fus.top_frac(np.asarray(x, float))
        r = np.clip(np.abs(y) / np.maximum(0.5 * w, 1e-9), 0, 1)
        return zc + tf * h * np.clip(1 - r ** nt, 0, 1) ** (1 / nt)

    def z_bot(self, x, y):
        w, h, zc, nt, nb = self.fus.section(np.asarray(x, float))
        tf = self.fus.top_frac(np.asarray(x, float))
        r = np.clip(np.abs(y) / np.maximum(0.5 * w, 1e-9), 0, 1)
        return zc - (1 - tf) * h * np.clip(1 - r ** nb, 0, 1) ** (1 / nb)

    def body_env(self, inset: float, x0: float, x1: float, dx: float = 0.01) -> G.Mesh:
        """Closed solid of the body OML inset by ``inset`` between x0 and x1 (loft of inset sections)."""
        key = (round(inset, 6), round(x0, 5), round(x1, 5), round(dx, 5))
        if key not in self._env:
            n = max(2, int(math.ceil((x1 - x0) / dx)) + 1)
            rings = []
            for x in np.linspace(x0, x1, n):
                P2 = SG.resample_ring(self.sec(x, inset), RING_N, start_dir=(0.0, 1.0))
                rings.append(np.column_stack([np.full(len(P2), x), P2[:, 0], P2[:, 1]]))
            self._env[key] = G.loft(rings)
        return self._env[key]

    def wing_env(self, inset: float, y0: float, y1: float, x_fair: float | None = None) -> G.Mesh:
        """Glove / wing loft inset by ``inset`` between span stations y0..y1 (|y|, starboard), lofted through the
        inset sections; with ``x_fair`` the glove root profile is extruded inboard to y0 (wing-root junction fairing,
        layout.shell.wing_root_fairing)."""
        key = ("w", round(inset, 6), round(y0, 5), round(y1, 5))
        if key not in self._env:
            e = self.wing.span_coords()
            ys = [y0] + [float(v) for v in e if y0 + 1e-4 < v < y1 - 1e-4] + [y1]
            ys = sorted(set(ys) | set(np.linspace(y0, y1, max(3, int((y1 - y0) / 0.02))).tolist()))
            rings = []
            y_root = float(self.wing.sections[0]["y"])
            for y in ys:
                eta = max(y, y_root)
                poly, fr = SG.section2d(self.wing, eta, n=160)
                reg = SG.largest(poly.buffer(-inset, join_style=2))
                P2 = SG.resample_ring(reg, RING_N, start_dir=(1.0, 0.0))
                Q = SG.to3d(fr, P2)
                Q[:, 1] = y
                rings.append(Q)
            self._env[key] = G.loft(rings)
        return self._env[key]

    def wing_poly(self, y: float, inset: float = 0.0) -> tuple[Polygon, tuple]:
        """Glove / wing section at span station y as an (x, z) polygon (streamwise, the glove is flat: z_le = 0)."""
        y_root = float(self.wing.sections[0]["y"])
        poly, fr = SG.section2d(self.wing, max(abs(y), y_root), n=200)
        P = np.asarray(poly.exterior.coords)
        Q = SG.to3d(fr, P)
        reg = Polygon(np.column_stack([Q[:, 0], Q[:, 2]])).buffer(0)
        if inset > 0:
            reg = reg.buffer(-inset, join_style=2)
        return SG.largest(reg), fr

    # ---------------------------------------------------------------- materials
    def layup_t(self, key: str) -> float:
        return float(layup_props(self.S, key)["thickness"])

    # ---------------------------------------------------------------- registry helpers
    def add(self, num, side, name, name_tr, material, process, mesh_fn, *, thickness=None, layup=None, parent=None,
            step=0, explode=(0.0, 0.0, 0.0), contacts=(), group=GROUP, notes="", purchased=False, vendor="",
            mass_kg=None) -> Part:
        pid = self.pid(num, side, group)
        p = Part(id=pid, name=name, name_tr=name_tr, group=group, material=material, process=process,
                 mesh_fn=mesh_fn, thickness=thickness, layup=layup, parent=parent, step=step,
                 explode=tuple(explode), contacts=tuple(contacts), side=side, notes=notes, purchased=purchased,
                 vendor=vendor, mass_kg=mass_kg)
        return self.reg.add(p)

    def mirror(self, pid_r: str, id_map: dict | None = None) -> Part:
        p = self.reg.parts[pid_r]
        m = dict(id_map or {})
        for k in list(self.reg.parts):
            if k.endswith("-R") and k.startswith("YK250-") and k not in m:
                m[k] = k[:-1] + "L"
        return self.reg.add(mirror_part(p, pid_r[:-1] + "L", id_map=m))


# =====================================================================================================================
# frames and bulkheads (layout.stations)
# =====================================================================================================================
WEB_CUT_SKIP = {"C-BRIDLE-F"}   # realised as a T-cap relief only: the web keeps the land of the bridle-fitting bolts


def _cut_polys(st: dict, include=lambda c: True) -> list[Polygon]:
    out = []
    for c in st.get("cutouts", []):
        if not include(c) or c["id"] in WEB_CUT_SKIP:
            continue
        y0, y1 = c["y"]
        z0, z1 = c["z"]
        out.append(rect(y0, y1, z0, z1))
        if c.get("mirror"):
            out.append(rect(-y0, -y1, z0, z1))
    return out


FRAME_STEP = {"FS-MS": 3, "FS-RS": 3, "FS0300": 6, "FS0600": 6, "FS1110": 6, "FS1330": 6, "FS1490": 6, "FS1810": 6,
              "FS-FUEL": 6, "FS-GEAR": 11, "FS3480": 11, "FS3670": 12, "FS3738": 13}


def _sec_common(C: Ctx, xs, inset: float) -> Polygon:
    """Intersection of the inset sections at the given stations (a prism between them stays inside the OML)."""
    g = None
    for x in xs:
        p = C.sec(x, inset)
        g = p if g is None else g.intersection(p)
    return SG.largest(g)


def _cap_notches(C: Ctx, sid: str) -> list[Polygon]:
    """(y, z) rectangles where a member crosses or abuts the frame at the skin line: the T-cap is interrupted there
    (the member's own skin flange / land carries the skin edge)."""
    out = []
    st = C.st[sid]
    for c in st.get("cutouts", []):
        if c.get("kind") in ("longeron notch", "edge notch"):
            y0, y1 = c["y"]
            z0, z1 = c["z"]
            out.append(rect(y0, y1, z0, z1))
            if c.get("mirror"):
                out.append(rect(-y0, -y1, z0, z1))
    if sid in ("FS1810", "FS-FUEL", "FS-MS", "FS-RS"):           # dorsal spine channel flanges (y 0.020-0.080)
        out.append(rect(-0.0825, 0.0825, 0.150, 0.40))
    if sid in ("FS-GEAR", "FS3480", "FS3670"):                     # dorsal longeron hat (crown + skin flanges)
        for s in (1, -1):
            out.append(rect(s * DORSAL_Y0, s * DORSAL_Y1, 0.10, 0.45))
    if sid in ("FS3480", "FS3670"):                                # ventral keel strip land flanges
        out.append(rect(-0.0515, 0.0515, -0.30, -0.06))
    if sid == "FS3670":                                            # chine end fitting at the firewall
        pass
    return out


DORSAL_W, DORSAL_H, DORSAL_T, DORSAL_FL = 0.025, 0.020, 0.002, 0.012     # hat 25 x 20, t 2.0, skin flanges 12 mm
DORSAL_Y0, DORSAL_Y1 = 0.15 - 0.5 * DORSAL_W - DORSAL_FL - 0.0015, 0.15 + 0.5 * DORSAL_W + DORSAL_FL + 0.0015


def frame_web_poly(C: Ctx, sid: str, xs) -> Polygon:
    """Web outline of a composite frame in (y, z): OML inset to the inner face of the T-cap, minus the declared
    cut-outs (pass-throughs, bays, notches), the ring opening of ring frames and the member crossings."""
    st = C.st[sid]
    inset = float(st["inset"])
    web = _sec_common(C, xs, inset + CAP_T - OV)
    cuts = _cut_polys(st)
    if sid == "FS-MS":
        # the payload-bay opening below the box runs out to the keel beams' outboard faces (the keel beams pass
        # through the frame; the outboard posts are bonded to them)
        k = C.mem["M-KEEL"]["box"]
        yk = float(k[1][1])
        cuts = [c for c in cuts if not (abs(c.bounds[0] + 0.205) < 1e-6 and c.bounds[1] < -0.2)]
        cuts.append(rect(-yk, yk, -0.30, float(C.st[sid]["cutouts"][0]["z"][1])))
    if st["type"] == "ring":
        inner = C.sec(xs[0], inset + float(st["ring_depth"]))
        hole = inner
        for land in _ring_lands(C, sid):
            hole = hole.difference(land)
        cuts.append(hole)
    if sid == "FS-MS":                                             # relief for the kink-fitting plates under the caps
        for fid in ("F-KINK-UP", "F-KINK-LO"):
            lo, hi = C.fit[fid]["boxes"][0]
            cuts.append(rect(lo[1] - 0.0005, hi[1] + 0.0005, lo[2] - 0.0002, hi[2] + 0.0002))
    if sid == "FS3480":                                            # dorsal longeron hat passes the ring band
        z_d = _dorsal_z(C, float(st["x"]))
        for s in (1, -1):
            cuts.append(rect(s * (0.15 - 0.5 * DORSAL_W - 0.0008), s * (0.15 + 0.5 * DORSAL_W + 0.0008),
                             z_d - 0.5 * DORSAL_H - 0.0008, 0.50))
    for c in cuts:
        web = web.difference(c)
    return clean_poly(web)


def _ring_lands(C: Ctx, sid: str) -> list[Polygon]:
    """Local deepening of a ring frame round the root fittings bolted to it (fastener edge distance + land)."""
    out = []
    if sid != "FS3480":
        return out
    for fid, m in (("F-FIN-FRONT", 0.018), ("F-STUB-FRONT", 0.018)):
        f = C.fit[fid]
        lo, hi = f["box"]
        pts = [b["point"] for b in f["bolts"]]
        ys = [p[1] for p in pts] + [lo[1], hi[1]]
        zs = [p[2] for p in pts] + [lo[2], hi[2]]
        for s in (1, -1):
            out.append(rect(s * (min(ys) - m), s * (max(ys) + m), min(zs) - m, max(zs) + m))
    return out


def _dorsal_z(C: Ctx, x: float) -> float:
    P = np.asarray(C.mem["M-DORSAL"]["paths"][0], float)
    return float(np.interp(x, P[:, 0], P[:, 2]))


def frame_cap_poly(C: Ctx, sid: str, xs) -> Polygon:
    st = C.st[sid]
    inset = float(st["inset"])
    band = _sec_common(C, xs, inset).difference(C.sec(xs[0], inset + CAP_T).union(
        C.sec(xs[-1], inset + CAP_T)))
    for c in _cap_notches(C, sid):
        band = band.difference(c)
    return clean_poly(band)


def build_frame(C: Ctx, sid: str) -> G.Mesh:
    """Composite frame: sandwich web (layout t) + solid T-cap of flange_w each side of the web at the skin line (the
    cap follows the OML along x: difference of two lofted envelopes)."""
    st = C.st[sid]
    x, t = float(st["x"]), float(st["t"])
    fw = float(st.get("flange_w", 0.028))
    sweep = float(st.get("sweep_deg", 0.0))
    inset = float(st["inset"])
    if sid == "FS3670":
        x0, x1 = st["x_faces"][0], st["x_faces"][0] + C.layup_t("rib_panel")
        cx0, cx1 = x0 - fw, x0 + OV
    else:
        x0, x1 = x - 0.5 * t, x + 0.5 * t
        cx0, cx1 = x - fw, x + fw
    if sweep:
        web0 = frame_web_poly(C, sid, [x])
        cap0 = frame_cap_poly(C, sid, [x])

        def build(y_lo, y_hi, dxa):
            keep = rect(y_lo, y_hi, -1.0, 1.0)
            ms = [prism_x(p, x0, x1 + dxa) for p in _as_polys(web0.intersection(keep)) if p.area > 1e-9]
            ms += [prism_x(p, cx0, cx1 + dxa) for p in _as_polys(cap0.intersection(keep)) if p.area > 1e-9]
            return union(ms)
        return chevron(build, sweep)
    web = frame_web_poly(C, sid, [x0, x1])
    ms = [prism_x(p, x0, x1) for p in _as_polys(web)]
    outer = C.body_env(inset, cx0, cx1, dx=0.008)
    inner = C.body_env(inset + CAP_T, cx0 - 0.002, cx1 + 0.002, dx=0.008)
    notches = [prism_x(n, cx0 - 0.003, cx1 + 0.003) for n in _cap_notches(C, sid)]
    cap = diff(outer, [inner] + notches)
    return union(ms + [pieces_above(cap)])


# =====================================================================================================================
# OML unions near the wing root (body + wing-root junction fairing + glove loft)
# =====================================================================================================================
def glove_root_z(C: Ctx, x):
    """(upper, lower) z of the glove root profile (wing section at the root y) at x - the wing-root junction fairing
    extrudes it inboard to layout.shell.wing_root_fairing.y_inner_m."""
    key = "_groot"
    if key not in C._env:
        poly, _ = C.wing_poly(float(C.wing.sections[0]["y"]))
        P = np.asarray(poly.exterior.coords)
        xs = np.linspace(P[:, 0].min() + 1e-5, P[:, 0].max() - 1e-5, 400)
        up, lo = [], []
        for xx in xs:
            g = LineString([(xx, -1), (xx, 1)]).intersection(poly)
            c = np.asarray(g.coords) if hasattr(g, "coords") else np.vstack([np.asarray(q.coords) for q in g.geoms])
            up.append(c[:, 1].max())
            lo.append(c[:, 1].min())
        C._env[key] = (xs, np.asarray(up), np.asarray(lo))
    xs, up, lo = C._env[key]
    x = np.asarray(x, float)
    inside = (x >= xs[0]) & (x <= xs[-1])
    return (np.where(inside, np.interp(x, xs, up), -np.inf), np.where(inside, np.interp(x, xs, lo), np.inf))


def oml_union_z(C: Ctx, x, y):
    """Upper / lower z of the union OML (body, junction fairing, glove) at plan points (x, |y| <= 0.4)."""
    x = np.asarray(x, float)
    y = np.abs(np.asarray(y, float))
    zt, zb = C.z_top(x, y), C.z_bot(x, y)
    y_in = float((C.L["shell"].get("wing_root_fairing") or {}).get("y_inner_m", 0.3))
    gu, gl = glove_root_z(C, x)
    m = y >= y_in
    zt = np.where(m, np.maximum(zt, gu), zt)
    zb = np.where(m, np.minimum(zb, gl), zb)
    return zt, zb


def wing_z(C: Ctx, x, y):
    """(upper, lower) z of the glove / wing loft in the streamwise section at span station y."""
    poly, _ = C.wing_poly(y)
    out_u, out_l = [], []
    for xx in np.atleast_1d(x):
        g = LineString([(xx, -1), (xx, 1)]).intersection(poly)
        if g.is_empty:
            out_u.append(np.nan)
            out_l.append(np.nan)
            continue
        c = np.asarray(g.coords) if hasattr(g, "coords") else np.vstack([np.asarray(q.coords) for q in g.geoms])
        out_u.append(c[:, 1].max())
        out_l.append(c[:, 1].min())
    return np.asarray(out_u), np.asarray(out_l)


# =====================================================================================================================
# centre wing box (CH-001), fork prongs (CH-053), centre-line rib (CH-055), kink fittings (CH-056/057)
# =====================================================================================================================
Y_SOB = 0.40                # side-of-body rib plane (M-SOB), rib faces +-3.4 mm
T_RIB = 0.0068
SLOT_W, SLOT_H = 0.0304, 0.061
PRONG_T, PAD_T, PAD_L = 0.0016, 0.010, 0.050
REAR_WEB_T, REAR_FL_T = 0.0006, 0.0009
DOUBLER_T = 0.0016          # frame-land web doubler of the box (8 plies +-45 PW)
BOX_BOLT_Y = (0.10, 0.17, 0.24, 0.31)
SLOT_PAD_Y0 = 0.596         # rear-spar web pad (16 plies) under the slot fitting, inboard end
SLOT_WEB_Y = (0.612, 0.6964)    # slot-fitting web plate span (y)
SLOT_BOLT_Y = (0.622, 0.640)    # 2 x M5 Ti into the web pad (inboard of the clevis plates)
SLOT_PLATE_Y0 = 0.652           # clevis plates from here to the joint rib (pin edge distance 2 D)


class Box:
    """Centre-box geometry from layout.chassis.members[M-CTBOX] (spar lines, cap centroid z / thickness)."""

    def __init__(self, C: Ctx):
        B = C.mem["M-CTBOX"]
        self.ms = np.asarray(B["main_spar_line"], float)
        self.rs = np.asarray(B["rear_spar_line"], float)
        self.mz = np.asarray(B["main_spar_caps_z"], float)
        self.mt = np.asarray(B["main_spar_caps_t"], float)
        self.rz = np.asarray(B["rear_spar_caps_z"], float)
        self.rt = np.asarray(B["rear_spar_caps_t"], float)
        self.z_cov = float(B["z"][1])
        self.w_main = float(B["section"]["main_cap_width"])
        self.w_rear = float(B["section"]["rear_cap_width"])
        self.y_end = float(B["y_extent"][1])
        fk = C.L["chassis"]["wing_joint"]["main_spar"]["fork"]
        self.w_fork = float(C.S["structures"]["sizing"]["wing_joint"]["fork"]["cap_width_m"])
        self.slot = fk["slot"]
        self.ms_x0 = float(C.st["FS-MS"]["x"])
        self.rs_x0 = float(C.st["FS-RS"]["x"])
        self.k_ms = math.tan(math.radians(float(C.st["FS-MS"]["sweep_deg"])))
        self.k_rs = math.tan(math.radians(float(C.st["FS-RS"]["sweep_deg"])))

    def xm(self, y):
        return np.interp(np.abs(y), self.ms[:, 1], self.ms[:, 0])

    def xr(self, y):
        return np.interp(np.abs(y), self.rs[:, 1], self.rs[:, 0])

    def main_cap(self, y, upper: bool):
        """(z_outer_face, t) of the main cap at |y|."""
        y = abs(y)
        j = 1 if upper else 0
        zc = float(np.interp(y, self.ms[:, 1], self.mz[:, j]))
        t = float(np.interp(y, self.ms[:, 1], self.mt))
        return (zc + 0.5 * t, t) if upper else (zc - 0.5 * t, t)

    def rear_cap(self, y, upper: bool):
        y = abs(y)
        j = 1 if upper else 0
        zc = float(np.interp(y, self.rs[:, 1], self.rz[:, j]))
        t = float(np.interp(y, self.rs[:, 1], self.rt))
        return (zc + 0.5 * t, t) if upper else (zc - 0.5 * t, t)

    def w_main_at(self, y):
        y = abs(y)
        return float(np.interp(y, [0.0, Y_SOB + 0.5 * T_RIB, 0.43, 1.0], [self.w_main, self.w_main, self.w_fork,
                                                                       self.w_fork]))


def _cap_loft(ys, xc_fn, w_fn, face_fn, t_min, upper: bool) -> G.Mesh:
    """UD cap as a loft of streamwise rectangles (x width w, thickness from the outer face inward)."""
    rings = []
    for y in ys:
        xc, w = float(xc_fn(y)), float(w_fn(y))
        zf, t = face_fn(y, upper)
        t = max(t, t_min)
        z0, z1 = (zf - t, zf) if upper else (zf, zf + t)
        rings.append(np.array([[xc - w / 2, y, z0], [xc + w / 2, y, z0], [xc + w / 2, y, z1], [xc - w / 2, y, z1]]))
    return G.loft(rings)


def _span_stations(pts_y, y_end, extra=()):
    ys = sorted({round(v, 6) for v in list(pts_y) + list(extra) if 0 <= v <= y_end} | {y_end})
    return [-v for v in ys[::-1] if v > 0] + ys


def cover_shell(C: Ctx, bx: Box, upper: bool, t: float, y_max: float, x_lo_fn, x_hi_fn, nx=26, ny=60) -> G.Mesh:
    """Box cover (sandwich) as a height-field shell between the spar-frame faces: outer face at the nominal cover z,
    lowered to the union OML - 1 mm where the body / fairing surface is lower (near the chine, aft of mid-chord)."""
    ys = np.linspace(-y_max, y_max, ny)
    P = np.zeros((nx, ny, 3))
    for j, y in enumerate(ys):
        xs = np.linspace(float(x_lo_fn(y)), float(x_hi_fn(y)), nx)
        zt, zb = oml_union_z(C, xs, np.full(nx, y))
        if upper:
            z = np.minimum(bx.z_cov, zt - 0.001)
        else:
            z = np.maximum(-bx.z_cov, zb + 0.001)
        P[:, j, 0], P[:, j, 1], P[:, j, 2] = xs, y, z
    inward = np.zeros_like(P)
    inward[..., 2] = -1.0 if upper else 1.0
    # grid orientation: u along x, v along y -> dP/du x dP/dv = +z (outward for the upper cover)
    if not upper:
        P = P[:, ::-1]
    return G.shell_from_grid(P, t, inward=inward)


def build_ctbox(C: Ctx, bx: Box, frames: dict) -> G.Mesh:
    yE = bx.y_end - 0.5 * T_RIB                                  # caps end on the joint-rib inner face
    y_sob_o = Y_SOB + 0.5 * T_RIB
    t_cov = C.layup_t("ct_box_cover")
    # ---- main caps (continuous joint to joint; slotted where the spar-frame web passes) ----
    ys = _span_stations(np.r_[bx.ms[:, 1], np.linspace(0, yE, 36)], yE, (y_sob_o, 0.43))
    caps = [_cap_loft(ys, bx.xm, bx.w_main_at, bx.main_cap, 0.0, u) for u in (True, False)]
    # ---- rear caps + glove rear-spar flanges (UD strip on the web flange, modelled 0.9 mm) ----
    ys_r = _span_stations(np.r_[bx.rs[:, 1], np.linspace(0, yE, 36)], yE, (y_sob_o,))
    caps += [_cap_loft(ys_r, bx.xr, lambda y: bx.w_rear, bx.rear_cap, REAR_FL_T, u) for u in (True, False)]
    # ---- glove rear-spar web (body: the FS-RS frame is the web) ----
    webs = []
    for s in (1, -1):
        rings = []
        for y in np.linspace(y_sob_o, yE, 14):
            xc = float(bx.xr(y))
            zu = bx.rear_cap(y, True)[0] - REAR_FL_T + OV
            zl = bx.rear_cap(y, False)[0] + REAR_FL_T - OV
            rings.append(np.array([[xc - REAR_WEB_T / 2, s * y, zl], [xc + REAR_WEB_T / 2, s * y, zl],
                                   [xc + REAR_WEB_T / 2, s * y, zu], [xc - REAR_WEB_T / 2, s * y, zu]]))
        webs.append(G.loft(rings if s > 0 else rings[::-1]))
        # 16-ply web pad at the rear-spar slot fitting (aft face)
        pad = []
        for y in np.linspace(SLOT_PAD_Y0, yE, 6):
            xc = float(bx.xr(y)) + REAR_WEB_T / 2 - OV
            zu = bx.rear_cap(y, True)[0] - REAR_FL_T + OV
            zl = bx.rear_cap(y, False)[0] + REAR_FL_T - OV
            pad.append(np.array([[xc, s * y, zl], [xc + FRAME_LAND, s * y, zl], [xc + FRAME_LAND, s * y, zu],
                                 [xc, s * y, zu]]))
        webs.append(G.loft(pad if s > 0 else pad[::-1]))
    # ---- covers inside the body (sandwich, between the spar-frame faces, to the SOB rib inner face) ----
    t_ms, t_rs = float(C.st["FS-MS"]["t"]), float(C.st["FS-RS"]["t"])
    y_cov = Y_SOB - 0.5 * T_RIB
    x_lo = lambda y: bx.ms_x0 + abs(y) * bx.k_ms + 0.5 * t_ms - OV
    x_hi = lambda y: bx.rs_x0 + abs(y) * bx.k_rs - 0.5 * t_rs + OV
    covers = [cover_shell(C, bx, u, t_cov, y_cov, x_lo, x_hi) for u in (True, False)]
    # ---- frame-land web doublers inside the box (4 x M6 per side per frame, layout structures body) ----
    dbl = []
    for sid, sgn, zlim in (("FS-MS", 1.0, 0.0295), ("FS-RS", -1.0, 0.0318)):
        st = C.st[sid]
        xf = float(st["x"]) + sgn * 0.5 * float(st["t"])

        def bld(y_lo, y_hi, dxa, xf=xf, sgn=sgn, zlim=zlim):
            parts = []
            for a, b in ((0.055, 0.335), (-0.335, -0.055)):
                a2, b2 = max(a, y_lo), min(b, y_hi)
                if b2 - a2 > 1e-4:
                    x0, x1 = (xf - OV, xf + DOUBLER_T) if sgn > 0 else (xf - DOUBLER_T, xf + OV)
                    parts.append(prism_x(rect(a2, b2, -zlim, zlim), x0, x1 + dxa))
            return union(parts) if parts else None
        k = math.tan(math.radians(float(st["sweep_deg"])))
        for s in (1, -1):
            h = bld(0.055, 0.335, 0.0)
            dbl.append(shear_x(h, k) if s > 0 else shear_x(h, k).mirrored_y())
    body = union(caps + webs + covers + dbl)
    # the spar-frame webs pass through the caps (caps on both faces of the web); the kink-fitting plates pass under
    # the caps through a relief in the web
    body = diff(body, [frames["FS-MS"], frames["FS-RS"]])
    # stay 1 mm under the union OML (solid skin over the caps)
    env = ct_envelope(C, 0.001)
    body = inter(body, env)
    # H-WING sealed grommets in the lower cover (layout.systems.harness H-WING penetration)
    pen = next(p for t_ in C.L["systems"]["harness"]["trunks"] if t_["id"] == "H-WING" for p in t_["penetrations"]
               if p["member"] == "M-CTBOX")
    px, py, pz = pen["point"]
    cut = [bore((px, s * py, pz - 0.02), (px, s * py, pz + 0.02), 0.010) for s in (1, -1)]
    return pieces_above(diff(body, cut))


def ct_envelope(C: Ctx, inset: float) -> G.Mesh:
    """Union OML inset (body + junction fairing + glove loft) around the centre box, both sides."""
    key = ("ct", round(inset, 6))
    if key not in C._env:
        y_in = float((C.L["shell"].get("wing_root_fairing") or {}).get("y_inner_m", 0.3))
        y_root = float(C.wing.sections[0]["y"])
        body = C.body_env(inset, 2.30, 3.00, dx=0.02)
        fair = []
        poly, _ = C.wing_poly(y_root, inset)
        for s in (1, -1):
            f = prism_y(poly, y_in - 0.002, y_root + 0.003)
            w = C.wing_env(inset, y_root + 0.001, 0.72)
            fair += [f, w] if s > 0 else [f.mirrored_y(), w.mirrored_y()]
        C._env[key] = union([body] + fair)
    return C._env[key]


def fork_frame(bx: Box):
    """Plan frame of the fork: origin on the main-spar line at the SOB rib outer face, d along the spar (outboard),
    n chordwise normal (aft)."""
    y0 = Y_SOB + 0.5 * T_RIB
    y1 = 0.7 - 0.5 * T_RIB
    p0 = np.array([bx.xm(y0), y0])
    p1 = np.array([bx.xm(y1), y1])
    d = (p1 - p0) / np.linalg.norm(p1 - p0)
    n = np.array([d[1], -d[0]])
    return p0, d, n, float(np.linalg.norm(p1 - p0))


def build_fork(C: Ctx, bx: Box) -> G.Mesh:
    """Starboard fork (CH-053-R): two +-45 PW prongs 1.6 mm either side of the 30.4 mm slot, full depth between the UD
    cap faces of CH-001, padded to 10 mm (50 mm long) round the two pin bores, with the bonded 4130 bushes (bores 16 H8
    cut here; the pins are the wing module's)."""
    p0, d, n, Lf = fork_frame(bx)
    pins = C.L["chassis"]["wing_joint"]["main_spar"]["pins"]
    s_pins = [float(np.dot(np.array(p["position"][:2]) - p0, d)) for p in pins]
    gap = 1e-4

    def zz(xy):
        y = float(xy[1])
        zu = bx.main_cap(y, True)
        zl = bx.main_cap(y, False)
        return zl[0] + zl[1] + gap, zu[0] - zu[1] - gap

    def slab(a, b, s0, s1, ns):
        rings = []
        for s in np.linspace(s0, s1, ns):
            q = p0 + s * d
            ring = []
            for off, up in ((a, False), (b, False), (b, True), (a, True)):
                xy = q + off * n
                z0, z1 = zz(xy)
                ring.append([xy[0], xy[1], z1 if up else z0])
            rings.append(np.asarray(ring))
        return G.loft(rings)
    h = 0.5 * SLOT_W
    ms = []
    for sg in (1, -1):
        a, b = sg * h, sg * (h + PRONG_T + OV)
        ms.append(slab(min(a, b), max(a, b), 0.0, Lf, 40))
        for sp in s_pins:
            a, b = sg * (h + PRONG_T - OV), sg * (h + PAD_T)
            ms.append(slab(min(a, b), max(a, b), sp - 0.5 * PAD_L, sp + 0.5 * PAD_L, 8))
    m = union(ms)
    y0, y1 = Y_SOB + 0.5 * T_RIB, 0.7 - 0.5 * T_RIB
    m = inter(m, box3((2.3, y0, -0.2), (2.9, y1, 0.2)))          # streamwise end faces on the rib faces
    cut = []
    for p in pins:
        c = np.asarray(p["position"], float)
        ax = np.asarray(p["axis"], float)
        r = 0.5 * float(p["diameter"]) + 0.0000135
        cut.append(bore(c - 0.04 * ax, c + 0.04 * ax, r))
    return diff(m, cut)


def build_clrib(C: Ctx, bx: Box) -> G.Mesh:
    """Centre-line rib (CH-055): sandwich rib in y = 0 between the spar-frame webs and the box covers, kink-fitting
    lands at the fore end (under the kink plates), 20 mm T-flanges bonded to the covers aft of the fittings."""
    m = C.mem["M-CLRIB"]
    t = float(m["section"]["t"])
    xa = float(C.st["FS-MS"]["x"]) + 0.5 * float(C.st["FS-MS"]["t"]) + 0.0008     # clear of the chevron kink piece
    xb = float(C.st["FS-RS"]["x"]) - 0.5 * float(C.st["FS-RS"]["t"])
    zc = bx.z_cov - C.layup_t("ct_box_cover") - 2e-5             # cover inner face (layout box z 0.0323)
    xk = float(C.fit["F-KINK-UP"]["boxes"][1][0][0])             # kink tab fore end = cap aft edge
    zp = float(C.fit["F-KINK-UP"]["boxes"][0][0][2])             # kink plate underside
    xk2 = xk + 0.5 * t * bx.k_ms + 0.0004                       # the chevron cap edge at y = +-t/2
    web = Polygon([(xa, -(zp - 0.0002)), (xk2, -(zp - 0.0002)), (xk2, -zc), (xb, -zc), (xb, zc), (xk2, zc),
                   (xk2, zp - 0.0002), (xa, zp - 0.0002)])
    ms = [prism_y(web, -0.5 * t, 0.5 * t)]
    xf0 = xk + 0.104
    for s in (1, -1):
        z0, z1 = (zc - CAP_T, zc) if s > 0 else (-zc, -zc + CAP_T)
        ms.append(box3((xf0, -0.0134, z0), (xb - 0.002, 0.0134, z1)))
    return union(ms)


def build_kink(C: Ctx, fid: str) -> G.Mesh:
    f = C.fit[fid]
    (a0, a1), (b0, b1) = f["boxes"]
    plate = box3(a0, a1)
    tab = box3((b0[0] - OV, b0[1], b0[2]), b1)
    return union([plate, tab])


def glove_rib_poly(C: Ctx, y: float) -> Polygon:
    """Glove rib outline (x, z): glove section inset 8.5 mm under the upper skins (LERX root bay 0.6/7/0.4 + bond)
    and 6.5 mm over the lower skins."""
    up, _ = C.wing_poly(y, 0.0085)
    lo, _ = C.wing_poly(y, 0.0065)
    return SG.largest(up.union(lo.intersection(rect(-10, 10, -1.0, 0.0))).buffer(0))


def _cap_cut_xz(bx: Box, y: float, w_extra=0.0005) -> list[Polygon]:
    out = []
    for upper in (True, False):
        zf, t = bx.main_cap(y, upper)
        w = bx.w_main_at(y)
        z0, z1 = (zf - t, zf) if upper else (zf, zf + t)
        out.append(rect(bx.xm(y) - w / 2 - w_extra, bx.xm(y) + w / 2 + w_extra, z0 - w_extra,
                        z1 + (0.02 if upper else 0.0) + w_extra if upper else z1 + w_extra))
        zf, t = bx.rear_cap(y, upper)
        t = max(t, REAR_FL_T)
        z0, z1 = (zf - t, zf) if upper else (zf, zf + t)
        out.append(rect(bx.xr(y) - bx.w_rear / 2 - w_extra, bx.xr(y) + bx.w_rear / 2 + w_extra, z0 - w_extra,
                        z1 + w_extra))
    return out


def _rib_plate(C: Ctx, y: float, outline: Polygon, cuts, x_lo, x_hi, fl_side: int, fl_cuts=()) -> G.Mesh:
    """Glove rib in the plane y (thickness T_RIB) with a bonded T-flange (CAP_T, 20 mm) at the skin line on the
    ``fl_side`` (+1 outboard, -1 inboard, 0 both sides)."""
    reg = outline.intersection(rect(x_lo, x_hi, -1, 1))
    web = reg.buffer(-CAP_T + OV, join_style=2)
    for c in cuts:
        web = web.difference(c)
    band = reg.difference(reg.buffer(-CAP_T, join_style=2))
    for c in list(cuts) + list(fl_cuts):
        band = band.difference(c)
    y0, y1 = y - 0.5 * T_RIB, y + 0.5 * T_RIB
    ms = [prism_y(p, y0, y1) for p in _as_polys(web) if p.area > 2e-6]
    f0 = y0 - (0.020 if fl_side <= 0 else 0.0)
    f1 = y1 + (0.020 if fl_side >= 0 else 0.0)
    ms += [prism_y(p, f0 if fl_side <= 0 else y0 + OV, f1 if fl_side >= 0 else y1 - OV)
           for p in _as_polys(band) if p.area > 2e-6]
    return union(ms)


def _fork_band_xz(bx: Box, y: float, half: float, dy: float = 0.024) -> Polygon:
    """The fork (prongs, pads or slot) as an x band seen in the streamwise planes y-dy .. y+dy (half width measured
    normal to the spar): covers a rib web and its flanges."""
    _, d, n, _ = fork_frame(bx)
    hx = half / abs(n[0])
    xs = [bx.xm(v) for v in (y - dy, y, y + dy)]
    return rect(min(xs) - hx, max(xs) + hx, -1.0, 1.0)


def _cap_cut_band(bx: Box, y: float, dy: float = 0.024) -> list[Polygon]:
    """Cap notches of a rib in the plane y covering its thickness / flange span (caps sampled at y-dy .. y+dy)."""
    out = []
    for v in np.linspace(y - dy, y + dy, 7):
        out += _cap_cut_xz(bx, max(v, 0.0))
    return [unary_union(out)]


def build_glove_ribs(C: Ctx, bx: Box) -> dict:
    """SOB rib (y 0.40, LERX nose to aft of the rear spar), glove rib (y 0.55: nose piece ahead of the fork and box
    piece between the fork and the rear spar) and joint rib (y 0.70, with the fork mouth, the rear-lug passage and the
    wing connector opening). Returns {key: mesh} for the starboard side."""
    out = {}
    hw = {t_["id"]: t_ for t_ in C.L["systems"]["harness"]["trunks"]}["H-WING"]
    grom = {p["member"]: p["point"] for p in hw["penetrations"]}
    # --- side-of-body rib: continuous (the box webs end at the body / start at the rib outer face); caps notched
    sob = C.mem["M-SOB"]
    y = Y_SOB
    cuts = _cap_cut_band(bx, y)
    gx, gy, gz = grom["M-SOB"]
    cuts.append(Point(gx, gz).buffer(0.010, 32))
    fl_cuts = [_fork_band_xz(bx, y + 0.0134, 0.5 * SLOT_W + PAD_T + 0.0005, 0.011),
               rect(bx.xr(y) - 0.5 * bx.w_rear - 0.0005, bx.xr(y + 0.024) + 0.5 * bx.w_rear + 0.0005, -1, 1)]
    out["SOB"] = _rib_plate(C, y, glove_rib_poly(C, y), cuts, sob["box"][0][0], sob["box"][1][0], +1, fl_cuts)
    # --- glove rib: the fork band and the rear spar split it; nose piece and box piece
    gr = C.mem["M-GLOVERIB"]
    y = 0.55
    poly = glove_rib_poly(C, y)
    cuts = _cap_cut_band(bx, y) + [_fork_band_xz(bx, y, 0.5 * SLOT_W + PRONG_T + 0.0005)]
    xr0 = min(bx.xr(y - 0.024), bx.xr(y)) - 0.5 * bx.w_rear - 0.0005
    cuts.append(rect(xr0, 3.5, -1, 1))
    gx, gy, gz = grom["M-GLOVERIB"]
    cuts.append(Point(gx, gz).buffer(0.010, 32))
    full = _rib_plate(C, y, poly, cuts, gr["box"][0][0], gr["box"][1][0], 0)
    parts = sorted((G.Mesh.from_manifold(p) for p in full.to_manifold().decompose()), key=lambda m: m.bounds()[0][0])
    out["GLOVE_NOSE"], out["GLOVE_BOX"] = parts[0], parts[1]
    # --- joint rib: continuous with the fork mouth, the outer-panel rear-lug passage, the wing connector
    jr = C.mem["M-JOINTRIB"]
    y = 0.70
    poly = glove_rib_poly(C, y)
    pm = C.L["chassis"]["wing_joint"]["main_spar"]
    mouth = rect(bx.xm(y) - (0.5 * SLOT_W + 0.0005) / abs(fork_frame(bx)[2][0]),
                 bx.xm(y) + (0.5 * SLOT_W + 0.0005) / abs(fork_frame(bx)[2][0]), -0.5 * SLOT_H - 0.0010,
                 0.5 * SLOT_H + 0.0010)
    rp = C.L["chassis"]["wing_joint"]["rear_spar"]
    lug = rp["lug"]
    pr = rp["pin"]["position"]
    ax = np.asarray(C.L["chassis"]["wing_joint"]["insertion"]["axis_inboard"], float)
    xl = pr[0] + (y - pr[1]) * ax[0] / ax[1]                      # lug centre where it crosses the rib plane
    lug_cut = rect(xl - 0.5 * lug["width"] - 0.0015, xl + 0.5 * lug["width"] + 0.0015,
                   pr[2] - 0.5 * lug["thickness"] - 0.0015, pr[2] + 0.5 * lug["thickness"] + 0.0015)
    cn = next(c for c in C.L["systems"]["harness"]["connectors"] if c["id"] == "CN-WING")["point"]
    conn = Point(cn[0], cn[2]).buffer(0.0155, 40)
    fl_cuts = _cap_cut_band(bx, 0.6866, 0.011) + [_fork_band_xz(bx, 0.6866, 0.5 * SLOT_W + PAD_T + 0.0005, 0.011),
                                       rect(bx.xr(0.69) - 0.5 * bx.w_rear - 0.0005,
                                            bx.xr(0.69) + 0.5 * bx.w_rear + 0.0005, -1, 1),
                                       rect(2.85, 2.95, -1, 1)]
    out["JOINT"] = _rib_plate(C, y, poly, [mouth, lug_cut, conn], jr["box"][0][0], JOINT_RIB_X1, -1, fl_cuts)
    return out


JOINT_RIB_X1 = 2.905        # joint rib extended aft over the rear-spar slot fitting (layout box ends 2.8719)


def build_slot_fitting(C: Ctx, bx: Box) -> G.Mesh:
    """Rear-spar slot fitting (CH-054-R): web plate on the 16-ply rear-spar web pad (2 x M5 Ti) and two horizontal
    4 mm clevis plates 8.2 mm apart, open outboard, bore 8 H8 for the vertical rear pin."""
    rp = C.L["chassis"]["wing_joint"]["rear_spar"]
    sf = rp["slot_fitting"]
    pt, gap = float(sf["plate_t"]), float(sf["slot"])
    pin = rp["pin"]
    px, py, pz = pin["position"]
    yE = 0.7 - 0.5 * T_RIB - 0.0002

    def xface(y):                                    # aft face of the web pad
        return float(bx.xr(y)) + 0.5 * REAR_WEB_T + FRAME_LAND - OV + 2e-5
    zc = pz
    rings = []
    for y in np.linspace(SLOT_WEB_Y[0], yE, 5):
        xf = xface(y) + OV
        rings.append(np.array([[xf, y, zc - 0.0101], [xf + 0.004, y, zc - 0.0101], [xf + 0.004, y, zc + 0.0101],
                               [xf, y, zc + 0.0101]]))
    web = G.loft(rings)
    x1 = 2.894
    plates = []
    for s in (1, -1):
        z0, z1 = (zc + 0.5 * gap, zc + 0.5 * gap + pt) if s > 0 else (zc - 0.5 * gap - pt, zc - 0.5 * gap)
        xs = xface(SLOT_PLATE_Y0) + 0.004 - 0.001
        poly = Polygon([(xs, SLOT_PLATE_Y0), (x1, SLOT_PLATE_Y0), (x1, yE), (xface(yE) + 0.004 - 0.001, yE)])
        plates.append(prism_z(poly, z0, z1))
    m = union([web] + plates)
    r = 0.5 * float(pin["diameter"]) + 0.00001
    return diff(m, [bore((px, py, pz - 0.03), (px, py, pz + 0.03), r)])


# =====================================================================================================================
# members (layout.chassis.members)
# =====================================================================================================================
MEM_IN = 0.0065 + CAP_T     # members reaching the skin stop on the inner face of the frame T-caps
CHINE_SPLICE_PAD = 0.015    # splice land of the chine longeron outboard leg beyond the end bolts


def sec_union(C: Ctx, x: float, inset: float) -> Polygon:
    """Body section at x inset, united with the wing-root junction fairing (glove root profile extruded inboard to
    y_inner) where x lies inside the glove root chord, both sides."""
    p = C.sec(x, inset)
    gu, gl = glove_root_z(C, np.array([x]))
    if np.isfinite(gu[0]):
        zu, zl = float(gu[0]) - inset, float(gl[0]) + inset
        if zu > zl + 1e-4:
            y_in = float((C.L["shell"].get("wing_root_fairing") or {}).get("y_inner_m", 0.3))
            y_root = float(C.wing.sections[0]["y"])
            p = p.union(rect(y_in, y_root, zl, zu)).union(rect(-y_in, -y_root, zl, zu))
    return SG.largest(p.buffer(0))


def _loft_sections(polys_xy, start_dir=(0.0, 1.0), n=RING_N) -> G.Mesh:
    """Loft through (x, polygon in (y, z)) pairs."""
    rings = []
    for x, poly in polys_xy:
        P2 = SG.resample_ring(poly, n, start_dir=start_dir)
        rings.append(np.column_stack([np.full(len(P2), x), P2[:, 0], P2[:, 1]]))
    return G.loft(rings)


def _path_loft(C: Ctx, P, poly_fn, dx=0.02, n=96, start_dir=(0.0, 1.0)) -> G.Mesh:
    """Loft along a member path P (k, 3): ``poly_fn(x, yc, zc)`` returns a convex-ish (y, z) polygon at x."""
    xs = np.r_[P[:, 0], np.arange(P[0, 0], P[-1, 0], dx)]
    lo_, hi_ = max(P[0, 0], 1.79), min(P[-1, 0], 1.98)              # glove-root fairing onset: denser stations
    if hi_ > lo_:
        xs = np.r_[xs, np.arange(lo_, hi_, 0.004)]
    xs = sorted(set(xs.round(5).tolist()) | {round(P[-1, 0], 5)})
    secs = []
    for x in xs:
        yc, zc = float(np.interp(x, P[:, 0], P[:, 1])), float(np.interp(x, P[:, 0], P[:, 2]))
        secs.append((x, SG.largest(poly_fn(x, yc, zc).buffer(0))))
    return _loft_sections(secs, start_dir=start_dir, n=n)


def union_env(C: Ctx, inset: float, x0: float, x1: float) -> G.Mesh:
    """Body OML inset united with the wing-root junction fairing (glove root profile inset, extruded from the fairing
    inner edge to the wing root), both sides, between x0 and x1."""
    key = ("u", round(inset, 6), round(x0, 5), round(x1, 5))
    if key not in C._env:
        body = C.body_env(inset, x0, x1)
        y_in = float((C.L["shell"].get("wing_root_fairing") or {}).get("y_inner_m", 0.3))
        y_root = float(C.wing.sections[0]["y"])
        poly, _ = C.wing_poly(y_root, inset)
        poly = poly.intersection(rect(x0, x1, -1, 1))
        ms = [body]
        if not poly.is_empty and poly.area > 1e-8:
            f = prism_y(clean_poly(poly), y_in, y_root + 0.001)
            ms += [f, f.mirrored_y()]
        C._env[key] = union(ms)
    return C._env[key]


def build_chine(C: Ctx, piece: int) -> G.Mesh:
    """Chine longeron piece (J 35 x 30, t 2.4): outer region = the member rectangle inside the union OML inset to the
    skin line (the J's skin-side flange is the land of the chine skins), minus the same region inset by t on the
    skin side and the web, minus the open bottom except the 12 mm lip; splice zone with an outboard leg on the SOB
    rib inner face. Built from robust lofts (rectangles along the path) and lofted OML envelopes."""
    m = C.mem["M-CHINE"]
    P = np.asarray(m["paths"][piece], float)
    w, h, t = float(m["section"]["w"]), float(m["section"]["h"]), float(m["section"]["t"])
    yo = Y_SOB - 0.5 * T_RIB
    x0, x1 = float(P[0, 0]), float(P[-1, 0])
    P2 = P.copy()
    P2[0, 0] -= 0.003
    P2[-1, 0] += 0.003
    box_o = _path_loft(C, P, lambda x, yc, zc: rect(yc - w / 2, yo, zc - h / 2, zc + h / 2), n=24)
    box_i = _path_loft(C, P2, lambda x, yc, zc: rect(yc - w / 2 + t, yo + 0.01, zc - h / 2 - 0.01, zc + h / 2 + 0.01),
                       n=24)
    openb = _path_loft(C, P2, lambda x, yc, zc: rect(yc - w / 2 + 0.012, yo + 0.01, zc - h / 2 - 0.01,
                                                     zc - h / 2 + t), n=24)
    outer = inter(box_o, union_env(C, 0.0065, x0 - 0.01, x1 + 0.01))
    inner = inter(box_i, union_env(C, 0.0065 + t, x0 - 0.02, x1 + 0.02))
    J_ = diff(outer, [inner, openb])
    sp = m["splices"][0 if piece == 0 else 1]
    bxs = [b[0] for b in sp["bolts"]]
    xa, xb = max(min(bxs) - CHINE_SPLICE_PAD, x0), min(max(bxs) + CHINE_SPLICE_PAD, x1)
    zc = float(np.interp(0.5 * (xa + xb), P[:, 0], P[:, 2]))
    leg = inter(box3((xa, yo - t, zc - h / 2), (xb, yo, zc + h / 2 - OV)), union_env(C, 0.0065, x0 - 0.01, x1 + 0.01))
    return pieces_above(union([J_, leg]))


def box_member(C: Ctx, lo, hi, inset=MEM_IN, cut=()) -> G.Mesh:
    """Layout box clipped by the body OML inset (members whose sides/top/bottom are 'skin')."""
    lo, hi = np.asarray(lo, float), np.asarray(hi, float)
    env = C.body_env(inset, lo[0] - 0.005, hi[0] + 0.005)
    m = inter(box3(lo, hi), env)
    return pieces_above(diff(m, list(cut)))


def chevron_slab(C: Ctx, sid: str, side: int, pad: float = 0.0) -> G.Mesh:
    """Half-space slab of a chevron frame: the region forward (side -1) or aft (side +1) of its face, as a big solid
    (used to trim members to the swept frame faces)."""
    st = C.st[sid]
    k = math.tan(math.radians(float(st["sweep_deg"])))
    xf = float(st["x"]) + side * (0.5 * float(st["t"]) + pad)

    def b(y_lo, y_hi, dxa):
        x0, x1 = (xf - 1.0, xf) if side < 0 else (xf, xf + 1.0)
        return box3((x0, y_lo, -1.0), (x1 + dxa, y_hi, 1.0))
    return chevron(b, float(st["sweep_deg"]))


def build_deck_nose(C: Ctx, chine_env) -> G.Mesh:
    m = C.mem["M-DECK-NOSE"]
    ms = [box3(*b) for b in m["boxes"]]
    deck = inter(union(ms), C.body_env(MEM_IN, 0.59, 1.12))
    return pieces_above(diff(deck, [chine_env]))


def build_keelwall(C: Ctx) -> G.Mesh:
    m = C.mem["M-KEELWALL"]
    lo, hi = m["box"]
    return box_member(C, lo, hi)


def build_turretwall(C: Ctx) -> G.Mesh:
    """Bay side wall: full height at the FS1110 / FS1330 end posts, lower edge free 20 mm above the skin over the
    sliding-door band (door rail plane)."""
    m = C.mem["M-TURRETWALL"]
    lo, hi = m["box"]
    band = C.L["mechanisms"]["door_outlines"]["turret_door_R"]["band"]
    bx0, bx1 = band[0][0], band[1][0]
    wall = box_member(C, lo, hi)
    relief = inter(box3((bx0, lo[1] - 0.01, -0.5), (bx1, hi[1] + 0.01, 0.5)),
                   diff(box3((bx0 - 0.01, lo[1] - 0.02, -0.5), (bx1 + 0.01, hi[1] + 0.02, 0.5)),
                        [C.body_env(0.0065 + 0.020, bx0 - 0.02, bx1 + 0.02)]))
    return pieces_above(diff(wall, [relief]))


def build_turretroof(C: Ctx) -> G.Mesh:
    m = C.mem["M-TURRETROOF"]
    return union([box3(*b) for b in m["boxes"]])


def build_parawall(C: Ctx, fs_a: float, fs_b: float, fw: float) -> G.Mesh:
    """Parachute-bay side wall from the floor top to the skin, with the 50 mm outboard top flange (hatch land +
    surround-skin land) between the frame caps."""
    m = C.mem["M-PARAWALL"]
    lo, hi = np.asarray(m["box"][0], float), np.asarray(m["box"][1], float)
    flo = C.mem["M-PARAFLOOR"]["box"]
    lo[2] = float(flo[1][2])
    wall = inter(box3(lo, (hi[0], hi[1], 0.30)), C.body_env(MEM_IN - OV, lo[0] - 0.005, hi[0] + 0.005))
    land = m["lands"][0]
    x0, x1 = fs_a + fw + 0.0005, fs_b - fw - 0.0005
    fl = diff(inter(box3((x0, land["y"][0], 0.0), (x1, land["y"][1], 0.30)), C.body_env(0.0065, x0 - 0.01, x1 + 0.01)),
              [C.body_env(0.0065 + CAP_T, x0 - 0.02, x1 + 0.02)])
    return union([wall, pieces_above(fl)])


def build_midfloor(C: Ctx) -> G.Mesh:
    """Mission-bay floor (outer strips, cut-out for TR-MISSION) + two bonded hat stiffeners under it outboard of the
    tray lip."""
    m = C.mem["M-MIDFLOOR"]
    lo, hi = m["box"]
    co = m["cutout"]
    floor = box_member(C, lo, hi, cut=[box3((co["x"][0], co["y"][0], lo[2] - 0.01), (co["x"][1], co["y"][1],
                                                                                    hi[2] + 0.01))])
    hats = []
    for s in (1, -1):
        yc = s * MID_HAT_Y
        z0 = float(lo[2])
        crown = box3((lo[0] + 0.002, yc - 0.010, z0 - 0.015), (hi[0] - 0.002, yc + 0.010, z0 - 0.015 + 0.0008))
        walls = [box3((lo[0] + 0.002, yc + sw * 0.010 - 0.0004, z0 - 0.015), (hi[0] - 0.002, yc + sw * 0.010 + 0.0004,
                                                                              z0 + OV)) for sw in (1, -1)]
        fls = [box3((lo[0] + 0.002, yc + sw * 0.010 + (0 if sw > 0 else -0.010), z0 - 0.0008),
                    (hi[0] - 0.002, yc + sw * 0.010 + (0.010 if sw > 0 else 0), z0 + OV)) for sw in (1, -1)]
        hats.append(union([crown] + walls + fls))
    return union([floor] + hats)


MID_HAT_Y = 0.172           # mission-floor hat stiffeners outboard of the 25 mm tray lip (layout text: y +-0.125)
MID_TRAY_LIP = 0.025


def build_keel(C: Ctx, frames) -> G.Mesh:
    """Keel beam (payload-bay side wall): sandwich wall from the skin to the forward deck underside, through the FS-MS
    posts, ending on the FS-RS face; outboard skin land flange for the payload hatch."""
    m = C.mem["M-KEEL"]
    lo, hi = np.asarray(m["box"][0], float), np.asarray(m["box"][1], float)
    dk = C.mem["M-FWDDECK"]["box"]
    wall = inter(box3(lo, (hi[0] + 0.01, hi[1], float(dk[0][2]))),
                 C.body_env(MEM_IN - OV, lo[0] - 0.005, hi[0] + 0.02))
    aft = inter(box3((C.st["FS-MS"]["x"], lo[1], lo[2]), (hi[0] + 0.01, hi[1], hi[2])),
                C.body_env(MEM_IN - OV, lo[0] - 0.005, hi[0] + 0.02))
    wall = union([wall, aft])
    wall = diff(wall, [_aft_of(C, "FS-RS")])
    # payload-hatch side land: 1.6 mm flange at the skin line outboard of the wall, between the frame caps
    fw = 0.028
    x0 = float(C.st["FS-FUEL"]["x"]) + fw + 0.0005
    x1 = float(C.st["FS-RS"]["x"]) - fw - 0.0005
    land = diff(inter(box3((x0, hi[1] - OV, -0.30), (x1 + 0.04, KEEL_LAND_Y1, -0.10)),
                      C.body_env(0.0065, x0 - 0.01, x1 + 0.05)), [C.body_env(0.0065 + CAP_T, x0 - 0.02, x1 + 0.06)])
    land = diff(land, [_ms_cap_zone(C), _aft_of(C, "FS-RS", -0.028)])
    return pieces_above(union([wall, pieces_above(land)]))


KEEL_LAND_Y1 = 0.245        # payload-hatch side land out to the hatch edge (0.222) + 12 mm Camloc edge + 11 mm


def _aft_of(C: Ctx, sid: str, offset: float = 0.0) -> G.Mesh:
    """Solid aft of the forward face of a (chevron) frame, shifted by ``offset`` (m, + aft)."""
    st = C.st[sid]
    k = math.tan(math.radians(float(st.get("sweep_deg", 0.0))))
    xf = float(st["x"]) - 0.5 * float(st["t"]) + offset

    def b(y_lo, y_hi, dxa):
        return box3((xf - dxa, y_lo, -1.0), (xf + 1.5, y_hi, 1.0))
    if not k:
        return box3((xf, -1.0, -1.0), (xf + 1.5, 1.0, 1.0))
    h = shear_x(b(-OV, 1.0, 0.0), k)
    return G.union([h, h.mirrored_y()])


def _fwd_of(C: Ctx, sid: str, offset: float = 0.0) -> G.Mesh:
    """Solid forward of the aft face of a (chevron) frame, shifted by ``offset`` (m, - forward)."""
    st = C.st[sid]
    k = math.tan(math.radians(float(st.get("sweep_deg", 0.0))))
    xf = float(st["x"]) + 0.5 * float(st["t"]) + offset
    if not k:
        return box3((xf - 1.5, -1.0, -1.0), (xf, 1.0, 1.0))
    h = shear_x(box3((xf - 1.5, -OV, -1.0), (xf, 1.0, 1.0)), k)
    return G.union([h, h.mirrored_y(), box3((xf - 1.5, -0.002, -1.0), (xf, 0.002, 1.0))])


def _ms_cap_zone(C: Ctx) -> G.Mesh:
    """Zone of the FS-MS T-cap (+-flange_w about the chevron web)."""
    st = C.st["FS-MS"]
    k = math.tan(math.radians(float(st["sweep_deg"])))
    fw = float(st["flange_w"]) + 0.0005
    h = shear_x(box3((st["x"] - fw, -OV, -1.0), (st["x"] + fw, 1.0, 1.0)), k)
    return G.union([h, h.mirrored_y(), box3((st["x"] - fw, -0.003, -1.0), (st["x"] + fw + 0.001, 0.003, 1.0))])


def build_fwddeck(C: Ctx, frames) -> G.Mesh:
    m = C.mem["M-FWDDECK"]
    lo, hi = np.asarray(m["box"][0], float), np.asarray(m["box"][1], float)
    deck = inter(box3(lo, (hi[0] + 0.07, hi[1], hi[2])), C.body_env(MEM_IN, lo[0] - 0.005, hi[0] + 0.08))
    deck = diff(deck, [_aft_of(C, "FS-MS")])
    return pieces_above(deck)


def build_wellroof(C: Ctx) -> G.Mesh:
    m = C.mem["M-WELLROOF"]
    lo, hi = np.asarray(m["box"][0], float), np.asarray(m["box"][1], float)
    roof = inter(box3((lo[0] - 0.04, lo[1], lo[2]), hi), C.body_env(MEM_IN, lo[0] - 0.045, hi[0] + 0.005))
    roof = diff(roof, [_fwd_of(C, "FS-RS")])
    return pieces_above(roof)


def build_gearbeam(C: Ctx) -> G.Mesh:
    """Outboard well wall with the leg notch (one x-z outline extruded in y, clipped by the body OML inset); 8.3 mm
    solid land standing 1.5 mm proud of the inboard face under the trunnion-fitting flange (through-thickness inserts
    M6, engagement >= 1.2 D)."""
    m = C.mem["M-GEARBEAM"]
    xz = unary_union([rect(b[0][0], b[1][0] + (OV if k < 2 else 0), b[0][2], b[1][2])
                      for k, b in enumerate(m["boxes"])])
    y0, y1 = float(m["box"][0][1]), float(m["box"][1][1])
    f = C.fit["F-TRUNNION"]
    bb = [b["point"] for b in f["bolts"] if b["group"] == "gear beam"]
    xs, zs = [p[0] for p in bb], [p[2] for p in bb]
    lands = [rect(min(xs) - 0.013, 2.9399 - 0.0005, min(zs) - 0.013, max(zs) + 0.013),
             rect(3.0119 + 0.0005, max(xs) + 0.013, min(zs) - 0.013, max(zs) + 0.013)]
    wall = prism_y(clean_poly(xz), y0, y1)
    land = union([prism_y(l_, y0 - BEAM_LAND_PROUD, y0 + OV) for l_ in lands])
    beam = union([wall, land])
    return pieces_above(inter(beam, C.body_env(MEM_IN, float(m["box"][0][0]) - 0.005, float(m["box"][1][0]) + 0.005)))


BEAM_LAND_PROUD = 0.0015    # gear-beam insert land stands 1.5 mm proud of the inboard face (8.3 mm insert depth)
TRUN_FLANGE_T = 0.0065      # trunnion-fitting beam flange (layout 8 mm) over the proud land


def build_hat(C: Ctx, P, yc_fn, w: float, h: float, t: float, fl: float, crown_down: bool, z_key: str = "centre"):
    """Hat stiffener bonded to the skin along path P (k, 3): crown + two walls (lofted rectangles) and two skin
    flanges (the skin-line layer between the OML inset and the inset + t, limited to the flange width); the crown is
    away from the skin (down for the dorsal hats, up for the belly hats). The path z is the hat centre."""
    P = np.asarray(P, float)
    x0, x1 = float(P[0, 0]), float(P[-1, 0])

    def crown(x, yc, zc):
        zb = zc - h / 2 if crown_down else zc + h / 2 - t
        return rect(yc - w / 2, yc + w / 2, zb, zb + t)

    def wall(sg):
        def f(x, yc, zc):
            ya, yb = (yc + w / 2 - t, yc + w / 2) if sg > 0 else (yc - w / 2, yc - w / 2 + t)
            return rect(ya, yb, zc - h / 2, zc + 1.0) if crown_down else rect(ya, yb, zc - 1.0, zc + h / 2)
        return f
    env0 = C.body_env(0.0065, x0 - 0.01, x1 + 0.01)
    body = [_path_loft(C, P, crown, n=24)] + [_path_loft(C, P, wall(sg), n=24) for sg in (1, -1)]
    body = inter(union(body), C.body_env(0.0065 - OV, x0 - 0.01, x1 + 0.01))
    yc0 = float(P[0, 1])
    band = diff(env0, [C.body_env(0.0065 + t, x0 - 0.02, x1 + 0.02)])
    zlim = (0.0, 1.0) if crown_down else (-1.0, 0.0)
    fl_box = box3((x0, yc0 - w / 2 - fl, zlim[0]), (x1, yc0 + w / 2 + fl, zlim[1]))
    cav = box3((x0 - 0.01, yc0 - w / 2 + t, zlim[0] - 0.01), (x1 + 0.01, yc0 + w / 2 - t, zlim[1] + 0.01))
    flange = diff(inter(band, fl_box), [cav])
    return pieces_above(union([body, flange]))


def build_dorsal(C: Ctx, fin_fit_x) -> G.Mesh:
    """Dorsal longeron (hat 25 x 20, crown down) FS-GEAR -> firewall forward face; crown and walls relieved over the
    fin front-spar fitting (its clevis ears rise into the hat; the skin flanges run through)."""
    m = C.mem["M-DORSAL"]
    P = np.asarray(m["paths"][0], float)
    hat = build_hat(C, P, None, DORSAL_W, DORSAL_H, DORSAL_T, DORSAL_FL, True)
    xa, xb = fin_fit_x
    relief = box3((xa - 0.001, 0.15 - 0.5 * DORSAL_W - 0.001, 0.0), (xb + 0.001, 0.15 + 0.5 * DORSAL_W + 0.001, 0.6))
    keep_fl = diff(C.body_env(0.0065 - 0.001, xa - 0.01, xb + 0.01), [C.body_env(0.0065 + DORSAL_T, xa - 0.02,
                                                                                  xb + 0.02)])
    relief = diff(relief, [keep_fl])
    return pieces_above(diff(hat, [relief]))


def build_aftkeel(C: Ctx) -> G.Mesh:
    """Machined 7075 hat channel (open downward) firewall -> x 3.905 with the two lower land flanges of the split lower
    cowl (Camloc receptacles at y +-0.040)."""
    m = C.mem["M-AFTKEEL"]
    lo, hi = m["box"]
    s = m["section"]
    w, t = float(s["w"]), float(s["t"])
    tf = float(s["land_flange_t"])
    yL = float(m["lands"][0]["y"][1])
    z_top, z_bot = float(hi[2]), float(lo[2])
    poly = unary_union([rect(-w / 2, w / 2, z_top - t, z_top), rect(-w / 2, -w / 2 + t, z_bot, z_top),
                        rect(w / 2 - t, w / 2, z_bot, z_top), rect(-yL, yL, z_bot, z_bot + tf)])
    poly = poly.difference(rect(-w / 2 + t, w / 2 - t, z_bot - 0.01, z_top - t))
    x0, x1 = float(lo[0]), float(hi[0])
    keel = prism_x(clean_poly(poly), x0, x1)
    return pieces_above(inter(keel, C.body_env(0.0065, x0 - 0.005, x1 + 0.005)))


def build_ventralkeel(C: Ctx) -> G.Mesh:
    """Ventral keel strip: belly hat 24 x 20 (1.6 mm, crown up) FS3480 -> firewall forward face with 26 mm land
    flanges on the belly skin (y +-0.050)."""
    m = C.mem["M-VENTRALKEEL"]
    P = np.asarray(m["paths"][0], float)
    s_ = m["section"]
    w, h, t = float(s_["w"]), float(s_["h"]), float(s_["t"])
    fl = float(m["lands"][0]["y"][1]) - w / 2
    return build_hat(C, P, None, w, h, t, fl, False)


def build_spine(C: Ctx) -> G.Mesh:
    """Dorsal spine channel: U 44 x 24 (0.8 mm) on the centre line under the dorsal skin, FS1810 -> FS-RS, with the
    60 mm top flanges along the V roof (lands of P-SPINE / P-FUEL1-3) and 16-ply floor pads under the bridle
    fittings."""
    m = C.mem["M-SPINE"]
    s = m["section"]
    t, pad = float(s["t"]), float(s["pad_t"])
    yw = float(s["w"]) / 2
    yf = float(m["lands"][0]["y"][1])
    z_floor = SPINE_FLOOR_TOP
    x0 = float(C.st["FS1810"]["x"]) + 0.5 * float(C.st["FS1810"]["t"])
    x1 = float(C.st["FS-RS"]["x"]) - 0.5 * float(C.st["FS-RS"]["t"])
    floor = box3((x0, -yw, z_floor - t), (x1 + 0.01, yw, z_floor))
    walls = [box3((x0, s_ * yw - (t if s_ > 0 else 0), z_floor - t), (x1 + 0.01, s_ * yw + (0 if s_ > 0 else t), 0.3))
             for s_ in (1, -1)]
    pads = [box3((xa, -yw + t - OV, z_floor - pad), (xb, yw - t + OV, z_floor - t + OV))
            for xa, xb in ((x0, x0 + SPINE_PAD_L), (x1 - SPINE_PAD_L, x1 + 0.01))]
    U = inter(union([floor] + walls + pads), C.body_env(0.0065, x0 - 0.005, x1 + 0.02))
    fl = diff(inter(box3((x0, -yf, 0.10), (x1 + 0.01, yf, 0.3)), C.body_env(0.0065, x0 - 0.005, x1 + 0.02)),
              [C.body_env(0.0065 + t, x0 - 0.01, x1 + 0.03), box3((x0 - 0.1, -yw + t, 0.0), (x1 + 0.1, yw - t, 0.4))])
    sp = union([U, pieces_above(fl)])
    return pieces_above(diff(sp, [_aft_of(C, "FS-RS")]))


SPINE_FLOOR_TOP = 0.165     # top face of the spine floor (bridle-fitting base strips sit on it)
SPINE_PAD_L = 0.052         # 16-ply floor pad under each bridle fitting


def build_wellkeel(C: Ctx) -> G.Mesh:
    m = C.mem["M-WELLKEEL"]
    lo, hi = m["box"]
    return box3(lo, hi)


def build_plain_wall(C: Ctx, mid: str, z_top=None) -> G.Mesh:
    m = C.mem[mid]
    lo, hi = np.asarray(m["box"][0], float), np.asarray(m["box"][1], float)
    if z_top is not None:
        hi[2] = z_top
    return box_member(C, lo, hi)


# =====================================================================================================================
# fittings (layout.chassis.fittings)
# =====================================================================================================================
def boxes_union(boxes, ov=OV) -> G.Mesh:
    return union([box3(np.asarray(a) - ov * 0, np.asarray(b)) for a, b in boxes])


def build_trunnion(C: Ctx) -> G.Mesh:
    """Main-gear trunnion fitting (7075-T651, starboard): beam flange (6.5 mm over the proud insert land), two bearing
    lugs 26 mm with 20 H7 bores on the trunnion axis, top plate on the well roof with head pockets for the 4 roof bolts,
    clear of the retraction-EMA allocation envelope (ACT-MLG-EMA)."""
    f = C.fit["F-TRUNNION"]
    B = [list(map(list, b)) for b in f["boxes"]]
    yb = float(C.mem["M-GEARBEAM"]["box"][0][1]) - BEAM_LAND_PROUD          # land face
    ms = []
    for k, (lo, hi) in enumerate(B):
        lo, hi = np.asarray(lo, float), np.asarray(hi, float)
        if k < 3:                                     # beam flange boxes
            hi[1] = yb
        ms.append(box3(lo - np.array([0, 0, 0]), hi + np.array([OV if k in (3, 4, 5, 6) else 0, 0, 0])))
    lugs_top = box3((2.9139, 0.326, -0.0868), (3.0379, 0.377 + OV, -0.0668))
    m = union(ms + [lugs_top])
    px, py, pz = f["pivot"]
    cuts = [bore((b["x"] - 0.05, py, pz), (b["x"] + 0.05, py, pz), 0.5 * float(b["bore"]) + 0.0000105)
            for b in f["bearings"]]
    for b in f["bolts"]:
        if b["group"] == "well roof":
            x, y, z = b["point"]
            cuts.append(box3((x - 0.0065, 0.3255, -0.1135), (x + 0.0065, y + 0.0065, -0.0868)))
    ema = next(a for a in C.L["systems"]["actuators"] if a["id"] == "ACT-MLG-EMA")["cylinder"]
    c = np.asarray(ema["center"], float)
    ax = np.asarray(ema["axis"], float)
    cuts.append(bore(c - (ema["half_length"] + 0.0005) * ax, c + (ema["half_length"] + 0.0005) * ax,
                     ema["radius"] + 0.0005))
    return diff(m, cuts)


def build_uplock(C: Ctx) -> G.Mesh:
    f = C.fit["F-UPLOCK"]
    lo, hi = f["box"]
    base = box3((lo[0], lo[1], hi[2] - UPLOCK_BASE_T), hi)
    px, py, pz = f["point"]
    boss = box3((px - 0.008, py - 0.012, lo[2]), (px + 0.008, py + 0.012, hi[2] - UPLOCK_BASE_T + OV))
    m = union([base, boss])
    return diff(m, [bore((px, py - 0.02, pz + 0.006), (px, py + 0.02, pz + 0.006), 0.0026)])   # hook pivot pin


UPLOCK_BASE_T = 0.005


def build_ng_pivot(C: Ctx) -> G.Mesh:
    """Two 7075 pivot blocks (6 mm) on the inboard faces of the keel walls with the flanged 16 H7 bushings (bores)."""
    f = C.fit["F-NG-PIVOT"]
    m = union([box3(*b) for b in f["boxes"]])
    px, py, pz = f["pivot"]
    return diff(m, [bore((px, -0.05, pz), (px, 0.05, pz), 0.5 * float(f["bore"]) + 0.0000095)])


def build_spindle_node(C: Ctx) -> G.Mesh:
    """Stabilator node (7075, starboard): U base flange 8 mm on the firewall aft face, inboard arm with the 61805 bearing
    boss (bore 37, 7 mm seat + circlip land), outboard cheek 7 mm with the 32 mm spindle clearance hole."""
    f = C.fit["F-SPINDLE-NODE"]
    B = [np.asarray(b, float) for b in f["boxes"]]
    ms = []
    for k, (lo, hi) in enumerate(B):
        lo = lo.copy()
        if k >= 3:
            lo[0] -= OV
        ms.append(box3(lo, hi))
    cy = f["cylinder"]
    c = np.asarray(cy["center"], float)
    a = np.asarray(cy["axis"], float)
    ms.append(bore(c - cy["half_length"] * a, c + cy["half_length"] * a, cy["radius"], n=48))
    m = union(ms)
    cuts = [bore(c - 0.03 * a, c + 0.03 * a, 0.0185 + 0.000015, n=48)]                   # 61805 seat (OD 37 H7)
    sh = f["spindle_hole_outboard_cheek"]
    sc = np.asarray(sh["center"], float)
    cuts.append(bore(sc - 0.01 * a, sc + 0.02 * a, 0.5 * float(sh["diameter"]), n=48))
    return diff(m, cuts)


def build_fw_corner(C: Ctx) -> G.Mesh:
    """Firewall upper corner fitting, aft part (7075): base plate 10 mm (engine-mount upper foot), aft clevis ear 6 mm
    for the fin rear-spar root lug (8 mm slot), bridge under the lug."""
    f = C.fit["F-FW-CORNER"]
    B = [np.asarray(b, float) for b in f["boxes"]]
    base, ear = box3(B[1][0], B[1][1]), box3(B[3][0], B[3][1])
    web = box3(B[2][0], (B[2][1][0], B[2][1][1], B[2][1][2]))
    bridge = box3((B[2][1][0] - OV, B[3][0][1], B[3][0][2]), (B[3][0][0] + OV, B[3][1][1], B[3][0][2] + 0.005))
    return union([base, web, ear, bridge])


def build_fw_fwdplate(C: Ctx, dorsal_crown_z: float) -> G.Mesh:
    """Firewall corner fitting, forward splice / backing plate (7075, 5 mm on the sandwich forward face) with the
    dorsal-longeron splice tongue under the hat crown."""
    f = C.fit["F-FW-CORNER"]
    lo, hi = np.asarray(f["boxes"][0][0], float), np.asarray(f["boxes"][0][1], float)
    xf = float(C.st["FS3670"]["x_faces"][0])
    plate = box3((xf - 0.005, lo[1], lo[2]), (xf, hi[1], hi[2]))
    zt = dorsal_crown_z - 0.0004
    tongue = box3((DORSAL_SPLICE_X0, 0.15 - 0.5 * DORSAL_W, zt - DORSAL_TONGUE_T), (xf - 0.005 + OV,
                                                                                    0.15 + 0.5 * DORSAL_W, zt))
    plate = diff(plate, [box3((xf - 0.006, 0.15 - 0.5 * DORSAL_W - DORSAL_FL - 0.002, zt), (xf + 0.001, 0.15 + 0.5 *
                                                                                            DORSAL_W + DORSAL_FL +
                                                                                            0.002, 0.5))])
    return union([plate, tongue])


DORSAL_SPLICE_X0 = 3.612    # forward end of the dorsal splice tongue (2 x M5 Ti through the hat crown)
DORSAL_TONGUE_T = 0.005


def build_emount_lo(C: Ctx) -> tuple[G.Mesh, G.Mesh]:
    """Lower engine-mount firewall fitting: aft foot pad 10 mm (7075) and forward backing plate 5 mm."""
    f = C.fit["F-EMOUNT-LO"]
    lo, hi = np.asarray(f["box"][0], float), np.asarray(f["box"][1], float)
    xa = float(C.st["FS3670"]["x"])
    xf = float(C.st["FS3670"]["x_faces"][0])
    return box3((xa, lo[1], lo[2]), (xa + 0.010, hi[1], hi[2])), box3((xf - 0.005, lo[1], lo[2]), (xf, hi[1], hi[2]))


def build_fin_front(C: Ctx) -> G.Mesh:
    """Fin front-spar root clevis (7075): two 6 mm ears either side of the 8 mm lug slot, base block 20 mm on the
    FS3480 aft face for the 2 x M6 frame bolts."""
    f = C.fit["F-FIN-FRONT"]
    lo, hi = np.asarray(f["box"][0], float), np.asarray(f["box"][1], float)
    m = box3(lo, hi)
    zb = max(b["point"][2] for b in f["bolts"] if b["group"] == "frame") + 0.012
    slot = box3((lo[0] + 0.006, lo[1] - 0.01, zb), (hi[0] - 0.006, hi[1] + 0.01, hi[2] + 0.01))
    return diff(m, [slot])


def build_stub_front(C: Ctx) -> G.Mesh:
    """Stabilator-stub front-spar clevis (7075): base 16 mm on the FS3480 aft face (2 x M5 to the frame), two 4 mm ears
    either side of the 8 mm stub-lug slot (open outboard)."""
    f = C.fit["F-STUB-FRONT"]
    lo, hi = np.asarray(f["box"][0], float), np.asarray(f["box"][1], float)
    m = box3(lo, hi)
    ys = max(b["point"][1] for b in f["bolts"] if b["group"] == "frame") + 0.010
    slot = box3((lo[0] + 0.004, ys, lo[2] - 0.01), (hi[0] - 0.004, hi[1] + 0.01, hi[2] + 0.01))
    return diff(m, [slot])


def build_ventral(C: Ctx, fid: str, keel_inner_half: float, lug_t: float = 0.008) -> G.Mesh:
    """Ventral root clevis inside the aft keel hat: base plate under the hat web (1 x M6 vertical), two ears against the
    hat walls either side of the ventral lug slot (1 x M6 along y, double shear)."""
    f = C.fit[fid]
    lo, hi = np.asarray(f["box"][0], float), np.asarray(f["box"][1], float)
    k = C.mem["M-AFTKEEL"]
    z_web = float(k["box"][1][2]) - float(k["section"]["t"])
    yi = keel_inner_half - 0.0002
    m = box3((lo[0], -yi, lo[2]), (hi[0], yi, z_web))
    slot = box3((lo[0] - 0.01, -0.5 * lug_t, lo[2] - 0.01), (hi[0] + 0.01, 0.5 * lug_t, z_web - 0.008))
    return diff(m, [slot])


def build_riser(C: Ctx, fid: str) -> G.Mesh:
    """Parachute bridle U-lug (7075): frame flange 6 mm, two base strips beside the link slot, two 6 mm ears with the
    8 mm shackle-pin bore (e = 15.5 mm)."""
    f = C.fit[fid]
    B = [np.asarray(b, float) for b in f["boxes"]]
    sgn = 1.0 if B[0][0][0] < B[1][0][0] else -1.0
    ms = []
    for k, (lo, hi) in enumerate(B):
        lo, hi = lo.copy(), hi.copy()
        if k in (1, 2):
            if sgn > 0:
                lo[0] -= OV
            else:
                hi[0] += OV
        if k in (3, 4):
            lo[2] -= OV
        ms.append(box3(lo, hi))
    m = union(ms)
    px, py, pz = f["point"]
    return diff(m, [bore((px, -0.03, pz + RISER_PIN_DZ), (px, 0.03, pz + RISER_PIN_DZ), 0.004 + 0.00002)])


RISER_PIN_DZ = 0.0            # shackle pin on the layout point (e = 15.5 mm to the ear top)


# =====================================================================================================================
# firewall stack (CH-013 sandwich + spacer tubes + stand-offs + AISI 304 shield) and the welded 4130 engine mount
# =====================================================================================================================
def fw_planes(C: Ctx):
    """x of the firewall layers: sandwich forward face, sandwich aft face, shield forward face, shield aft face."""
    st = C.st["FS3670"]
    xf, xa = float(st["x_faces"][0]), float(st["x_faces"][1])
    t_s = float(st["shield_t"])
    return xf, xf + C.layup_t("rib_panel"), xa - t_s, xa


def fw_section(C: Ctx, x: float, inset: float) -> Polygon:
    return C.sec(x, inset)


def build_shield(C: Ctx, holes_yz) -> G.Mesh:
    """0.4 mm AISI 304 fireproof sheet over the firewall section with its riveted stainless edge angle (13 mm aft
    flange at the skin line = cowl land), cut-outs as the firewall's (fireproof grommets / unions / boots / duct)."""
    st = C.st["FS3670"]
    xf, xs, xs0, xa = fw_planes(C)
    inset = float(st["inset"])
    sheet = C.sec(xa, inset + CAP_T - OV)
    for c in _cut_polys(st):
        sheet = sheet.difference(c)
    ms = [prism_x(clean_poly(sheet), xs0, xa)]
    outer = C.body_env(inset, xs0 - 0.0002, xa + 0.013, dx=0.004)
    inner = C.body_env(inset + SHIELD_EDGE_T, xs0 - 0.002, xa + 0.015, dx=0.004)
    edge = diff(outer, [inner] + [prism_x(n, xs0 - 0.01, xa + 0.02) for n in _cap_notches(C, "FS3670")])
    ms.append(pieces_above(edge))
    return union(ms)


SHIELD_EDGE_T = 0.0008      # stainless edge angle 0.8 mm (13 mm aft flange)


def engine_frame(C: Ctx):
    eng = C.L["chassis"]["engine_mount"]
    d = np.asarray(eng["thrust_axis"]["direction_aft"], float)
    return d / np.linalg.norm(d)


def build_engine_mount(C: Ctx, foot_x: float) -> G.Mesh:
    """Welded 4130 N tube mount: 8 struts 12.7 x 0.89 from the four firewall feet to the ring (15.9 x 0.89) through the
    four ring nodes, four isolator cups (ID 41 for the 40 x 25 conical isolators, bottom plate with the 9 mm bolt
    hole) on gussets at the nodes, four foot plates 4 mm (2 x M8 each) on the firewall fittings."""
    eng = C.L["chassis"]["engine_mount"]
    sec = eng["section"]
    ro_s, t_s = 0.5 * float(sec["strut_tube_od_m"]), float(sec["strut_tube_t_m"])
    ro_r, t_r = 0.5 * float(sec["ring_tube_od_m"]), float(sec["ring_tube_t_m"])
    d = engine_frame(C)
    nodes = np.asarray(eng["ring_nodes"], float)
    cen = np.asarray(eng["ring_center"], float)
    ms = []
    # ring: closed polygon through the 4 nodes ordered round the centre (in the ring plane)
    e1 = np.array([0.0, 1.0, 0.0])
    e2 = np.cross(d, e1)
    ang = np.arctan2((nodes - cen) @ e2, (nodes - cen) @ e1)
    order = np.argsort(ang)
    loop = nodes[order]
    for i in range(4):
        a, b = loop[i], loop[(i + 1) % 4]
        u = (b - a) / np.linalg.norm(b - a)
        ms.append(G.tube(ro_r, ro_r - t_r, a - 0.4 * ro_r * u, b + 0.4 * ro_r * u, n=20))
    for k, nd in enumerate(nodes):
        ms.append(G.sphere(ro_r + 0.0005, nd, n=20))                   # welded node cluster
    # feet and struts
    feet = {}
    for i, ft in enumerate(eng["feet"]):
        p = np.asarray(ft, float).copy()
        p[0] = foot_x
        feet[f"foot{i + 1}"] = p
    for tb in eng["tubes"]:
        a = feet[tb["from"]].copy()
        b = nodes[int(tb["to"][-1]) - 1]
        a[0] += FOOT_T + 0.004
        ms.append(G.tube(ro_s, ro_s - t_s, a, b, n=16))
        ms.append(G.sphere(ro_s + 0.0005, a, n=16))
    for i, ft in feet.items():
        ms.append(box3((foot_x, ft[1] - 0.022, ft[2] - 0.012), (foot_x + FOOT_T, ft[1] + 0.022, ft[2] + 0.012)))
        ms.append(box3((foot_x + FOOT_T - OV, ft[1] - 0.008, ft[2] - 0.008), (foot_x + FOOT_T + 0.006, ft[1] + 0.008,
                                                                               ft[2] + 0.008)))
    # isolator cups on gussets
    for k, ic in enumerate(eng["isolators"]["centres"]):
        c = np.asarray(ic, float)
        c0 = c - 0.0125 * d
        ms.append(G.tube(CUP_ID / 2 + CUP_T, CUP_ID / 2, c0 - 0.0005 * d, c0 + 0.012 * d, n=36))
        ms.append(G.cylinder(CUP_ID / 2 + CUP_T, c0 - CUP_BOTTOM_T * d, c0 + 0.0003 * d, n=36))
        nd = nodes[int(np.argmin(np.linalg.norm(nodes - c, axis=1)))]
        ms.append(G.tube(0.006, 0.0045, nd, c0 + 0.003 * d + 0.4 * (nd - c0 - 0.003 * d), n=14))
    m = union(ms)
    cut = [G.cylinder(0.0045, np.asarray(ic, float) - 0.03 * d, np.asarray(ic, float) - 0.0125 * d + 0.001 * d, n=24)
           for ic in eng["isolators"]["centres"]]
    return diff(m, cut)


FOOT_T = 0.004
CUP_ID, CUP_T, CUP_BOTTOM_T = 0.041, 0.0012, 0.0025


# =====================================================================================================================
# small parts: frame U-ring legs and lower segment, clips, backing plates, spacers, trays, liners, brackets, anchors
# =====================================================================================================================
def build_uring(C: Ctx):
    """FS3738 engine-bay lower U-ring: 2024-T3 legs (2.0 mm web, 20 mm formed flange at the skin line, below
    ring_z_max) for |y| >= 0.15 and the machined 7075 lower I-segment (60 mm deep, flanges 20 x 1.6, web 2.0) between
    y +-0.15 with the aft-keel passage. Both keep >= 10 mm to the engine dynamic envelope."""
    st = C.st["FS3738"]
    x0, x1 = st["x_faces"]
    xm = 0.5 * (x0 + x1)
    inset = float(st["inset"])
    depth = float(st["ring_depth"])
    zmax = float(st["ring_z_max"])
    seg = st["lower_segment"]
    ys = float(seg["y"][1])
    sz = C.S["structures"]["sizing"]["body"]["fs3738_lower_segment"]
    outer = C.sec(xm, inset)
    band = outer.difference(C.sec(xm, inset + depth)).intersection(rect(-1, 1, -1, zmax))
    eng = engine_keepout(C, 0.010 + 0.001)
    legs = []
    for s in (1, -1):
        b = band.intersection(rect(s * (ys - SEG_OVERLAP), s * 1.0, -1, 1))
        web = prism_x(clean_poly(b), x0, x1)
        fl_band = outer.difference(C.sec(xm, inset + float(st["flange_w"]) * 0 + 0.0016)).intersection(
            rect(s * (ys + 0.0005), s * 1.0, -1, zmax))
        fl = prism_x(clean_poly(fl_band), x0 - 0.020, x0 + OV)
        legs.append(diff(union([web, fl]), [eng]))
    # lower segment
    d = float(sz["depth_m"])
    tw, tf, fw = float(sz["web_t_m"]), float(sz["flange_t_m"]), float(sz["flange_w_m"])
    sb = outer.difference(C.sec(xm, inset + d)).intersection(rect(-ys, ys, -1, 0.10))
    web = prism_x(clean_poly(sb), xm - tw / 2, xm + tw / 2)
    ofl = outer.difference(C.sec(xm, inset + tf)).intersection(rect(-ys, ys, -1, 0.10))
    ifl = C.sec(xm, inset + d - tf).difference(C.sec(xm, inset + d)).intersection(rect(-ys, ys, -1, 0.0))
    k = C.mem["M-AFTKEEL"]
    yl = float(k["lands"][0]["y"][1]) + 0.0005
    keel_cut = rect(-0.5 * float(k["section"]["w"]) - 0.0003, 0.5 * float(k["section"]["w"]) + 0.0003, -1,
                    float(k["box"][1][2]) + 0.0003).union(rect(-yl, yl, -1, float(k["box"][0][2]) + 0.0019))
    parts = [web, prism_x(clean_poly(ofl.difference(keel_cut)), xm - fw / 2, xm + fw / 2),
             prism_x(clean_poly(ifl), xm - fw / 2, xm + fw / 2)]
    segm = diff(union(parts), [prism_x(keel_cut, xm - 0.02, xm + 0.02)])
    return legs[0], diff(segm, [eng])


SEG_OVERLAP = 0.025         # 2024 legs lap the 7075 lower segment ends (3 x M5 splice each end)


def engine_keepout(C: Ctx, margin: float) -> G.Mesh:
    """KO-ENGINE boxes (engine axes from the propeller hub along the inclined thrust line) grown by ``margin``."""
    ko = next(k for k in C.L["keep_outs"] if k["id"] == "KO-ENGINE")
    pr = C.S["propeller"]
    hub = np.asarray(pr["hub"], float)
    eps = math.radians(float(pr["thrust_line_inclination_deg"]))
    d = np.array([math.cos(eps), 0.0, math.sin(eps)])
    up = np.array([-math.sin(eps), 0.0, math.cos(eps)])
    ms = []
    for b in ko["boxes"]:
        u0, u1 = b["u"]
        v0, v1 = b["v"]
        w0, w1 = b["w"]
        c = hub - 0.5 * (u0 + u1) * d + 0.5 * (v0 + v1) * np.array([0, 1.0, 0]) + 0.5 * (w0 + w1) * up
        ms.append(obox(c, [d, [0, 1.0, 0], up], [0.5 * (u1 - u0) + margin, 0.5 * (v1 - v0) + margin,
                                                0.5 * (w1 - w0) + margin]))
    return union(ms)


def build_clip(C: Ctx, sid: str, side: int):
    """7075 shear clip at a chine-longeron notch (frame leg 30 mm on the frame face, longeron leg 20 mm on the J web,
    28 mm long, 2.0 mm): returns (mesh, frame-bolt points, longeron-bolt points, x of the frame face)."""
    st = C.st[sid]
    m = C.mem["M-CHINE"]
    P = np.asarray(m["paths"][0 if float(st["x"]) < 2.6 else 1], float)
    w = float(m["section"]["w"])
    xf = float(st["x"]) + side * 0.5 * float(st["t"])
    yc, zc = float(np.interp(xf, P[:, 0], P[:, 1])), float(np.interp(xf, P[:, 0], P[:, 2]))
    yw = yc - w / 2                                       # inboard face of the J web
    ct, L, la, lb = CLIP_T, CLIP_L, CLIP_LEG_A, CLIP_LEG_B
    z0, z1 = zc - L / 2, zc + L / 2
    if side > 0:
        legA = box3((xf, yw - la, z0), (xf + ct, yw, z1))
        legB = box3((xf, yw - ct, z0), (xf + lb, yw, z1))
    else:
        legA = box3((xf - ct, yw - la, z0), (xf, yw, z1))
        legB = box3((xf - lb, yw - ct, z0), (xf, yw, z1))
    notch = next(c for c in st["cutouts"] if c.get("kind") == "longeron notch")
    ya = min(float(notch["y"][0]) - 0.0105, yw - la + 0.0085)
    pts_a = [(xf, ya, zc - 0.006), (xf, ya, zc + 0.006)]
    xb = xf + side * (lb - 0.0085)
    pts_b = [(xb, yw, zc - 0.006), (xb, yw, zc + 0.006)]
    return union([legA, legB]), pts_a, pts_b, xf


CLIP_T, CLIP_L, CLIP_LEG_A, CLIP_LEG_B = 0.002, 0.028, 0.030, 0.020
CLIP_SIDE = {"FS1110": 1, "FS1330": 1, "FS1490": 1, "FS1810": 1, "FS-FUEL": -1, "FS-GEAR": 1, "FS3480": 1}


def build_liner(C: Ctx, fs: dict, cut_meshes, t: float) -> G.Mesh:
    """Fuel-bay liner tub (CFRP, open top) round the cell envelope of layout.chassis.fuel_supports: floor and side
    walls inset ``inset_from_oml_m`` from the OML, end walls on the swept boundary planes, from z0 to z1; trimmed
    clear of the surrounding structure."""
    z0, z1 = float(fs["z"][0]), float(fs["z"][1])
    ins = float(fs["inset_from_oml_m"])
    bf, ba = fs["boundary_fwd"], fs["boundary_aft"]
    xa, xb = float(bf["x_at_centre_line"]), float(ba["x_at_centre_line"])
    ka, kb = math.tan(math.radians(float(bf["sweep_deg"]))), math.tan(math.radians(float(ba["sweep_deg"])))
    yl = float(fs["y_limits_m"][1])

    def wedge(xA, xB, zlo, zhi, ylim):
        half = []
        for s_ in (1, -1):
            V = []
            for y in (0.0, ylim):
                V += [(xA + y * ka, s_ * y, zlo), (xB + y * kb, s_ * y, zlo), (xB + y * kb, s_ * y, zhi),
                      (xA + y * ka, s_ * y, zhi)]
            half.append(G.hull(np.asarray(V)))
        return G.union(half)
    outer = inter(wedge(xa - t, xb + t, z0 - t, z1, yl + t), C.body_env(ins - t, xa - 0.03, xb + 0.10))
    inner = inter(wedge(xa, xb, z0, z1 + 0.05, yl), C.body_env(ins, xa - 0.04, xb + 0.11))
    tub = diff(outer, [inner] + list(cut_meshes))
    return pieces_above(tub, 1e-7)


# =====================================================================================================================
# registration
# =====================================================================================================================
MAT_PW, MAT_UD, MAT_7075, MAT_2024, MAT_6061 = ("cfrp_pw_mtm45_as4", "cfrp_ud_mtm45_as4", "al_7075_t651_plate",
                                               "al_2024_t3_sheet", "al_6061_t6_sheet")
MAT_4130, MAT_SS = "steel_4130_n", "ss_304_annealed"
P_PREG, P_CNC, P_SHEET, P_TIG, P_SS = ("prepreg_ooa_vacbag", "cnc_milling_metal", "sheet_metal_aluminium",
                                       "tig_welding_4130", "sheet_metal_steel")
STRICT = True
PROBLEMS: list = []

# part numbers allocated by this module beyond the layout ids (sub-ranges of layout.part_numbering.modules.chassis)
N_POSTS = {}
N_CHINE_AFT, N_CHINE_END = 40, 41
N_CLIPS = {"FS1110": 42, "FS1330": 43, "FS1490": 44, "FS1810": 45, "FS-FUEL": 46, "FS-GEAR": 47, "FS3480": 48}
N_GLOVE_BOX = 58
N_SHIELD, N_STANDOFF, N_SPACER = 88, 89, 90
N_NODE_BACK, N_CORNER_FWD, N_EMLO_BACK, N_VK_SPLICE = 91, 92, 93, 94
N_WASHER_F, N_WASHER_A = 110, 111
N_PARA_BRKT_AFT = 123
N_PAY_RAIL = 124
N_RAIL_FR, N_RAIL_AL = 125, 126
N_RING_LAND = 127


class _Memo:
    def __init__(self):
        self.d = {}

    def __call__(self, key, fn):
        if key not in self.d:
            self.d[key] = finish(fn())
        return self.d[key]


def register(reg: Registry, spec: dict) -> None:
    C = Ctx(reg, spec)
    bx = Box(C)
    M = _Memo()
    L = C.L
    PROBLEMS.clear()

    def fr(sid):
        return M(("frame", sid), lambda: build_frame(C, sid))

    def add(num, side, name, name_tr, material, process, fn, **kw):
        kw.setdefault("explode", (0.0, 0.0, 0.0))
        return C.add(num, side, name, name_tr, material, process, fn, **kw)

    root = L["root_part"]
    rid = lambda lid, side="R": C.ref(lid, side)              # noqa: E731

    # ------------------------------------------------------------------ frames (layout.stations)
    steps = FRAME_STEP
    for st in L["stations"]:
        sid = st["id"]
        if sid == "FS3738":
            continue
        num = int(st["part"].split("-")[-1])
        par = root
        add(num, "C", f"frame {sid} ({st['type']})", f"{st['role_tr']} ({sid})", st["material"], st["process"],
            (lambda sid=sid: fr(sid)), layup=st.get("layup"), parent=par, step=steps[sid],
            explode=(0.0, 0.0, 0.30 if st["x"] < 2.3 else 0.0) if sid not in ("FS-MS", "FS-RS") else (0, 0, 0.12),
            contacts=(root,), notes=st.get("construction", ""))
    # FS3738 metallic U-ring: 2024 legs (R/L) + 7075 lower segment
    st = C.st["FS3738"]
    legs_seg = lambda: build_uring(C)                          # noqa: E731
    add(14, "R", "engine-bay lower U-ring leg, starboard (FS3738)", "motor bölmesi alt U halkası bacağı, sağ (FS3738)",
        MAT_2024, P_SHEET, lambda: M(("uring",), legs_seg)[0] if False else finish(build_uring(C)[0]),
        thickness=float(st["t"]), parent=st["lower_segment"]["part"], step=13, explode=(0.0, 0.15, -0.05))
    add(15, "C", "engine-bay lower U-ring segment (FS3738, machined I)",
        "motor bölmesi alt U halkası alt parçası (FS3738, talaşlı I kesit)", MAT_7075, P_CNC,
        lambda: finish(build_uring(C)[1]), thickness=0.002, parent=rid("M-AFTKEEL"), step=13, explode=(0.0, 0.0, -0.12))
    C.mirror(C.pid(14, "R"))
    # ------------------------------------------------------------------ centre wing box group
    def ctbox():
        return build_ctbox(C, bx, {"FS-MS": fr("FS-MS"), "FS-RS": fr("FS-RS")})
    m_ct = C.mem["M-CTBOX"]
    reg.add(Part(id=root, name="centre wing box (carry-through)", name_tr=m_ct["name_tr"], group=GROUP,
                 material=MAT_UD, process=P_PREG, mesh_fn=lambda: M(("ct",), ctbox), thickness=C.layup_t(
                     "ct_box_cover"), step=2, explode=(0.0, 0.0, 0.0),
                 notes="multi-material box (UD spar caps, ct_box_cover sandwich covers, +-45 PW glove rear-spar webs "
                       "and frame-land doublers): mass_kg = sum of the sub-volumes x their densities"))
    add(55, "C", "CT-box centre-line rib", C.mem["M-CLRIB"]["name_tr"], MAT_PW, P_PREG,
        lambda: M(("clrib",), lambda: build_clrib(C, bx)), layup="rib_panel", parent=root, step=2,
        explode=(0.0, 0.0, 0.18), contacts=(root, C.st["FS-MS"]["part"], C.st["FS-RS"]["part"]))
    for fid, num in (("F-KINK-UP", 56), ("F-KINK-LO", 57)):
        add(num, "C", C.fit[fid]["name"], C.fit[fid]["name_tr"], MAT_7075, P_CNC,
            (lambda fid=fid: M(("kink", fid), lambda: build_kink(C, fid))), thickness=0.002, parent=C.pid(55),
            step=2, explode=(0.0, 0.0, 0.25 if num == 56 else -0.25), contacts=(root, C.pid(55), C.st["FS-MS"]["part"]))
    ribs = lambda: build_glove_ribs(C, bx)                       # noqa: E731
    add(53, "R", "outer-panel joint fork, starboard (prongs + bonded 4130 bushes)",
        "dış panel birleşim çatalı, sağ (kulaklar + yapıştırılmış 4130 burçlar)", MAT_PW, P_PREG,
        lambda: M(("fork",), lambda: build_fork(C, bx)), thickness=PRONG_T, parent=root, step=5,
        explode=(0.0, 0.25, 0.0), contacts=(root,),
        notes="two +-45 PW prongs 1.6 mm padded to 10 mm round the pin bores; 4130 bushes 16 H8 x OD 22 x 10 bonded "
              "and line-reamed (bores cut here); pins P-MAIN1/2 are the wing module's")
    add(50, "R", "side-of-body rib, starboard", "gövde yanı kaburgası, sağ", MAT_PW, P_PREG,
        lambda: M(("ribs",), ribs)["SOB"], layup="rib_panel", parent=root, step=4, explode=(0.0, 0.3, 0.0),
        contacts=(root, C.pid(53, "R")))
    add(51, "R", "glove rib, nose piece, starboard", "eldiven kaburgası, burun parçası, sağ", MAT_PW, P_PREG,
        lambda: M(("ribs",), ribs)["GLOVE_NOSE"], layup="rib_panel", parent=root, step=4, explode=(0.0, 0.35, 0.0),
        contacts=(root, C.pid(53, "R")))
    add(N_GLOVE_BOX, "R", "glove rib, box piece, starboard", "eldiven kaburgası, kutu parçası, sağ", MAT_PW, P_PREG,
        lambda: M(("ribs",), ribs)["GLOVE_BOX"], layup="rib_panel", parent=root, step=4, explode=(0.0, 0.35, 0.0),
        contacts=(root, C.pid(53, "R")))
    add(52, "R", "joint rib (centre-section side), starboard", "birleşim kaburgası (orta kesit), sağ", MAT_PW, P_PREG,
        lambda: M(("ribs",), ribs)["JOINT"], layup="rib_panel", parent=root, step=4, explode=(0.0, 0.4, 0.0),
        contacts=(root, C.pid(53, "R")))
    add(54, "R", "rear-spar slot fitting, starboard", "arka kiriş yuva bağlantısı, sağ", MAT_7075, P_CNC,
        lambda: M(("slot",), lambda: build_slot_fitting(C, bx)), thickness=0.004, parent=root, step=5,
        explode=(0.0, 0.3, 0.05), contacts=(root, C.pid(52, "R")))
    for n in (53, 50, 51, N_GLOVE_BOX, 52, 54):
        C.mirror(C.pid(n, "R"))
    _register_members(C, M, fr)
    _register_fittings(C, M, fr, bx)
    _register_small(C, M, fr)
    _set_ctbox_mass(C, M, bx)
    _fasteners(C, bx)
    if PROBLEMS and STRICT:
        raise ValueError("chassis fasteners: " + "; ".join(f"{a}: {b}" for a, b in PROBLEMS[:10]))
