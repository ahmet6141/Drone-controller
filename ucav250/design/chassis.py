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
        np.diag([1.0, -1.0, 1.0]), (0.0, 2 * y0 + (y1 - y0), 0.0))


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
def _cut_polys(st: dict, include=lambda c: True) -> list[Polygon]:
    out = []
    for c in st.get("cutouts", []):
        if not include(c):
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
