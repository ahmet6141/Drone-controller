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
import os
from collections import defaultdict

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


def _cap_notch_sides(C: Ctx, sid: str) -> list[tuple[Polygon, str]]:
    """(y, z) rectangles where a member crosses or abuts the frame at the skin line, with the cap half they cut
    ("both", "aft" or "fwd" of the web): the T-cap is interrupted there (the member's own skin flange / land carries the
    skin edge)."""
    out = []
    st = C.st[sid]
    for c in st.get("cutouts", []):
        if c.get("kind") in ("longeron notch", "edge notch"):
            y0, y1 = c["y"]
            z0, z1 = c["z"]
            out.append((rect(y0, y1, z0, z1), "both"))
            if c.get("mirror"):
                out.append((rect(-y0, -y1, z0, z1), "both"))
    spine = {"FS1810": "aft", "FS-FUEL": "both", "FS-MS": "both", "FS-RS": "fwd"}   # spine channel skin flanges
    if sid in spine:
        out.append((rect(-0.0825, 0.0825, 0.150, 0.40), spine[sid]))
    if sid in ("FS-GEAR", "FS3480"):                               # dorsal longeron hat (crown + skin flanges)
        for s in (1, -1):
            out.append((rect(s * DORSAL_Y0, s * DORSAL_Y1, 0.10, 0.45), "both"))
    if sid in ("FS3480", "FS3670"):                                # ventral keel strip land flanges
        out.append((rect(-0.0515, 0.0515, -0.30, -0.06), "both"))
    chine_ends = {"FS0600": (0, "aft"), "FS3670": (1, "fwd")}      # chine J skin flange starts / ends at the frame
    if sid in chine_ends:
        piece, side = chine_ends[sid]
        m = C.mem["M-CHINE"]
        P = np.asarray(m["paths"][piece], float)
        w, h = float(m["section"]["w"]), float(m["section"]["h"])
        fw = float(st.get("flange_w", 0.028))
        x0 = float(st["x_faces"][0]) if "x_faces" in st else float(st["x"]) - 0.5 * float(st["t"])
        x1 = float(st["x_faces"][1]) if "x_faces" in st else float(st["x"]) + 0.5 * float(st["t"])
        xs = np.linspace(x1, x1 + fw + 0.003, 5) if side == "aft" else np.linspace(x0 - fw - 0.003, x0, 5)
        yc = np.interp(xs, P[:, 0], P[:, 1])
        zc = np.interp(xs, P[:, 0], P[:, 2])
        for s in (1, -1):
            out.append((rect(s * (yc.min() - 0.5 * w - 0.0015), s * 1.0, zc.min() - 0.5 * h - 0.0015,
                             zc.max() + 0.5 * h + 0.0015), side))
    return out


def _cap_notches(C: Ctx, sid: str) -> list[Polygon]:
    return [p for p, _side in _cap_notch_sides(C, sid)]


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
        yk = float(k[1][1]) + MEM_GAP
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
    if CLIP_SIDE.get(sid):                                           # shear-clip land at the chine notch
        _P, xf, yw, zc, ya = _clip_geom(C, sid, CLIP_SIDE[sid])
        for s in (1, -1):
            out.append(rect(s * (ya - 0.012), s * (yw(xf) + 0.002), zc(xf) - 0.022, zc(xf) + 0.022))
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


def frame_cap_poly(C: Ctx, sid: str, xs, side: str = "both") -> Polygon:
    """T-cap band of a frame in (y, z); ``side`` "fwd" / "aft" keeps the notches that cut that cap half only."""
    st = C.st[sid]
    inset = float(st["inset"])
    band = _sec_common(C, xs, inset).difference(C.sec(xs[0], inset + CAP_T).union(
        C.sec(xs[-1], inset + CAP_T)))
    for c, sd in _cap_notch_sides(C, sid):
        if side == "both" or sd in ("both", side):
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
        cap_f, cap_a = frame_cap_poly(C, sid, [x], "fwd"), frame_cap_poly(C, sid, [x], "aft")

        def build(y_lo, y_hi, dxa):
            keep = rect(y_lo, y_hi, -1.0, 1.0)
            ms = [prism_x(p, x0, x1 + dxa) for p in _as_polys(web0.intersection(keep)) if p.area > 1e-9]
            ms += [prism_x(p, cx0, x) for p in _as_polys(cap_f.intersection(keep)) if p.area > 1e-9]
            ms += [prism_x(p, x - OV, cx1 + dxa) for p in _as_polys(cap_a.intersection(keep)) if p.area > 1e-9]
            return union(ms)
        return chevron(build, sweep)
    web = frame_web_poly(C, sid, [x0, x1])
    ms = [prism_x(p, x0, x1) for p in _as_polys(web)]
    outer = C.body_env(inset, cx0, cx1, dx=0.008)
    inner = C.body_env(inset + CAP_T, cx0 - 0.002, cx1 + 0.002, dx=0.008)
    xr = {"both": (cx0 - 0.003, cx1 + 0.003), "aft": (x, cx1 + 0.003), "fwd": (cx0 - 0.003, x1 if sid == "FS3670"
                                                                                 else x)}
    notches = [prism_x(n, *xr[side]) for n, side in _cap_notch_sides(C, sid)]
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


def build_ctbox(C: Ctx, bx: Box, frames: dict, parts_out: dict | None = None) -> G.Mesh:
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
    if parts_out is not None:
        parts_out.update({"caps": caps, "webs": webs, "covers": covers, "doublers": dbl})
        return None
    body = union(caps + webs + covers + dbl)
    return _ctbox_finish(C, body, frames)


def _ctbox_finish(C: Ctx, body: G.Mesh, frames: dict) -> G.Mesh:
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


def ctbox_mass(C: Ctx, bx: Box, frames: dict) -> tuple[float, dict]:
    """Mass of the multi-material centre box CH-001: UD spar caps (cfrp_ud density), sandwich covers
    (layups.ct_box_cover areal mass / thickness), +-45 PW webs, pads and frame-land doublers (cfrp_pw density)."""
    comp = {}
    build_ctbox(C, bx, frames, parts_out=comp)
    mats = C.S["materials"]
    lay = layup_props(C.S, "ct_box_cover")
    rho = {"caps": float(mats[MAT_UD]["density"]), "covers": lay["areal_mass"] / lay["thickness"],
           "webs": float(mats[MAT_PW]["density"]), "doublers": float(mats[MAT_PW]["density"])}
    out = {}
    for k, ms in comp.items():
        m = _ctbox_finish(C, union(ms), frames)
        out[k] = m.volume() * rho[k]
    return sum(out.values()), out


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
MEM_GAP = 0.0003            # bond line between a member edge and the frame T-cap / skin it is bonded to
MEM_IN = 0.0065 + CAP_T + MEM_GAP   # members reaching the skin stop 0.3 mm off the inner face of the frame T-caps
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


def _is_rect(poly) -> bool:
    if not isinstance(poly, Polygon) or poly.interiors:
        return False
    y0, z0, y1, z1 = poly.bounds
    return abs(poly.area - (y1 - y0) * (z1 - z0)) <= 1e-9 * max((y1 - y0) * (z1 - z0), 1e-12)


def _loft_sections(polys_xy, start_dir=(0.0, 1.0), n=RING_N) -> G.Mesh:
    """Loft through (x, polygon in (y, z)) pairs; rectangles are lofted through their exact corners (resampling a
    slender rectangle by arc length would cut its corners)."""
    exact = all(_is_rect(p) for _x, p in polys_xy)
    rings = []
    for x, poly in polys_xy:
        if exact:
            y0, z0, y1, z1 = poly.bounds
            P2 = np.array([[y0, z0], [y1, z0], [y1, z1], [y0, z1]])
        else:
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
    if piece in (0, 1):                         # fwd piece: ahead of the FS-MS cap; aft piece: ahead of its end fitting
        xe = chine_fwd_end_x(C) if piece == 0 else chine_aft_end_x(C)
        q = [float(np.interp(xe, P[:, 0], P[:, k])) for k in (1, 2)]
        P = np.vstack([P[P[:, 0] < xe - 1e-4], [xe, q[0], q[1]]])
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
    bxs = [b[0] for b in (chine_fwd_splice_points(C) if piece == 0 else sp["bolts"])]
    xa, xb = max(min(bxs) - CHINE_SPLICE_PAD, x0), min(max(bxs) + CHINE_SPLICE_PAD, x1)
    if piece == 1:                              # aft splice leg reaches forward to the rear-spar cap (2 x M4 Ti)
        xa = CHINE_AFT_LEG_X0
    zc = float(np.interp(0.5 * (xa + xb), P[:, 0], P[:, 2]))
    leg = inter(box3((xa, yo - t, zc - h / 2), (xb, yo, zc + h / 2)), union_env(C, 0.0065, x0 - 0.01, x1 + 0.01))
    return pieces_above(union([J_, leg]))


CHINE_AFT_LEG_X0 = 2.850         # aft splice leg forward end (rear-spar cap at y 0.40 ends at x 2.846)
CHINE_AFT_SPLICE_X = (2.870, 2.8825)
KINK_BOLT_X0, KINK_BOLT_PITCH = 2.520, 0.0185       # layout pitch 18 mm < 2.5 D + hole radius in the rib land
CHINE_FWD_SPLICE_PITCH = 0.0185


def chine_fwd_end_x(C: Ctx) -> float:
    """The forward chine piece ends 0.5 mm ahead of the FS-MS T-cap at its web line (chevron frame)."""
    m = C.mem["M-CHINE"]
    P = np.asarray(m["paths"][0], float)
    st = C.st["FS-MS"]
    k = math.tan(math.radians(float(st["sweep_deg"])))
    yw = float(P[-1, 1]) - 0.5 * float(m["section"]["w"])
    return float(st["x"]) + yw * k - float(st.get("flange_w", 0.028)) - 0.0005


def chine_fwd_splice_points(C: Ctx):
    sp = C.mem["M-CHINE"]["splices"][0]
    x_last = chine_fwd_end_x(C) - CHINE_SPLICE_PAD
    y, z = float(sp["bolts"][0][1]), float(sp["bolts"][0][2])
    return [(x_last - i * CHINE_FWD_SPLICE_PITCH, y, z) for i in range(len(sp["bolts"]))]


def chine_aft_splice_points(C: Ctx):
    """2 x M4 Ti through the aft chine splice leg and the SOB rib, at the rib's mid height (glove TE bay)."""
    yo = Y_SOB - 0.5 * T_RIB - 0.5 * float(C.mem["M-CHINE"]["section"]["t"])
    out = []
    for x in CHINE_AFT_SPLICE_X:
        zu, zl = wing_z(C, [x], Y_SOB)
        out.append((x, yo, 0.5 * ((float(zu[0]) - 0.0085) + (float(zl[0]) + 0.0065))))
    return out


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
    deck = inter(fuse_boxes(m["boxes"]), C.body_env(MEM_IN, 0.59, 1.12))
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
    return fuse_boxes(m["boxes"])


def build_parawall(C: Ctx, fs_a: float, fs_b: float, fw: float) -> G.Mesh:
    """Parachute-bay side wall from the floor top to the skin, with the 50 mm outboard top flange (hatch land +
    surround-skin land) between the frame caps."""
    m = C.mem["M-PARAWALL"]
    lo, hi = np.asarray(m["box"][0], float), np.asarray(m["box"][1], float)
    flo = C.mem["M-PARAFLOOR"]["box"]
    lo[2] = float(flo[1][2])
    land = m["lands"][0]
    x0, x1 = fs_a + fw + 0.0005, fs_b - fw - 0.0005
    wall = union([inter(box3(lo, (hi[0], hi[1], 0.30)), C.body_env(MEM_IN, lo[0] - 0.005, hi[0] + 0.005)),
                  inter(box3((x0, lo[1], lo[2]), (x1, hi[1], 0.30)),
                        C.body_env(0.0065 + CAP_T - OV, x0 - 0.005, x1 + 0.005))])
    fl = diff(inter(box3((x0, land["y"][0], 0.0), (x1, land["y"][1], 0.30)), C.body_env(0.0065, x0 - 0.01, x1 + 0.01)),
              [C.body_env(0.0065 + CAP_T, x0 - 0.02, x1 + 0.02)])
    return union([wall, pieces_above(fl)])


def build_midfloor(C: Ctx) -> G.Mesh:
    """Mission-bay floor: two outer sandwich strips with the framed centre cut-out (closed by the removable tray
    TR-MISSION, flush with the floor top) + two bonded hat stiffeners (PW 4 plies, 20 x 15 mm) along the cut-out
    edges: the inner wall lines the cut-out edge, a 20 mm inner flange under the cut-out carries the tray screws, the
    outer flange is bonded under the floor strip."""
    m = C.mem["M-MIDFLOOR"]
    lo, hi = m["box"]
    co = m["cutout"]
    cx0, cx1 = float(co["x"][0]), float(co["x"][1])
    ye = float(co["y"][1])
    floor = box_member(C, lo, hi, cut=[box3((cx0, -ye, lo[2] - 0.01), (cx1, ye, hi[2] + 0.01))])
    z0 = float(lo[2])
    zt = z0 + MID_TRAY_DROP                                      # inner-flange top = tray underside
    t = MID_HAT_T
    xa, xb = float(lo[0]) + 0.002, float(hi[0]) - 0.002
    hats = []
    for s in (1, -1):
        def yy(a, b):
            return (min(s * a, s * b), max(s * a, s * b))
        yi = ye - t - 0.0003                                     # inner wall along the cut-out edge (0.3 mm gap)
        yo = yi + MID_HAT_W
        zc = z0 - MID_HAT_H
        ms = [box3((xa, yy(yi, yo)[0], zc), (xb, yy(yi, yo)[1], zc + t)),                 # crown
              box3((xa, yy(yi, yi + t)[0], zc), (xb, yy(yi, yi + t)[1], z0 + OV)),        # inner wall
              box3((cx0 + 5e-4, yy(yi, yi + t)[0], z0 - OV), (cx1 - 5e-4, yy(yi, yi + t)[1], zt)),
              box3((cx0 + 5e-4, yy(yi - MID_HAT_IN_FL, yi + t)[0], zt - t),
                   (cx1 - 5e-4, yy(yi - MID_HAT_IN_FL, yi + t)[1], zt)),                  # inner flange (tray land)
              box3((xa, yy(yo - t, yo)[0], zc), (xb, yy(yo - t, yo)[1], z0 + OV)),        # outer wall
              box3((xa, yy(yo - t, yo + MID_HAT_OUT_FL)[0], z0 - t), (xb, yy(yo - t, yo + MID_HAT_OUT_FL)[1],
                                                                       z0 + OV))]       # outer flange
        hats.append(union(ms))
    return union([floor] + hats)


MID_HAT_W, MID_HAT_H, MID_HAT_T = 0.020, 0.015, 0.0008     # hat 20 x 15, 4 plies PW (layout M-MIDFLOOR.section)
MID_HAT_IN_FL, MID_HAT_OUT_FL = 0.0227, 0.010              # inner flange under the cut-out (tray screws), outer flange
MID_TRAY_T = 0.003                                         # TR-MISSION plate 3 mm, top flush with the floor top
MID_TRAY_DROP = 0.0068 - MID_TRAY_T                        # floor bottom -> tray underside


def build_keel(C: Ctx, frames) -> G.Mesh:
    """Keel beam (payload-bay side wall): sandwich wall from the skin to the forward deck underside, through the FS-MS
    posts, ending on the FS-RS face; outboard skin land flange for the payload hatch."""
    m = C.mem["M-KEEL"]
    lo, hi = np.asarray(m["box"][0], float), np.asarray(m["box"][1], float)
    dk = C.mem["M-FWDDECK"]["box"]
    fw = 0.028
    x0 = float(C.st["FS-FUEL"]["x"]) + fw + 0.0005
    x1 = float(C.st["FS-RS"]["x"]) - fw - 0.0005
    env_m = C.body_env(MEM_IN, lo[0] - 0.005, hi[0] + 0.02)
    wall = inter(box3(lo, (hi[0] + 0.01, hi[1], float(dk[0][2]))), env_m)
    # through the FS-MS posts up to the box lower cover: from the (chevron) forward face of FS-MS
    aft = inter(inter(box3((float(C.st["FS-MS"]["x"]) - 0.01, lo[1], lo[2]), (hi[0] + 0.01, hi[1], hi[2])), env_m),
                _aft_of(C, "FS-MS"))
    # between the frame caps the wall is fused with its own skin land flange
    mid = inter(box3((x0, lo[1], lo[2]), (x1, hi[1], float(dk[0][2]))),
                C.body_env(0.0065 + CAP_T - OV, x0 - 0.005, x1 + 0.005))
    wall = union([wall, aft, mid])
    wall = diff(wall, [_aft_of(C, "FS-RS", -MEM_GAP)])
    # payload-hatch side land: 1.6 mm flange at the skin line outboard of the wall, between the frame caps
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
    yo = CHEV_D + 0.0007                         # the frame's straight kink piece: forward face at xf
    return G.union([h, h.mirrored_y(), box3((xf, -yo, -1.0), (xf + 1.5, yo, 1.0))])


def _fwd_of(C: Ctx, sid: str, offset: float = 0.0) -> G.Mesh:
    """Solid forward of the aft face of a (chevron) frame, shifted by ``offset`` (m, - forward)."""
    st = C.st[sid]
    k = math.tan(math.radians(float(st.get("sweep_deg", 0.0))))
    xf = float(st["x"]) + 0.5 * float(st["t"]) + offset
    if not k:
        return box3((xf - 1.5, -1.0, -1.0), (xf, 1.0, 1.0))
    h = shear_x(box3((xf - 1.5, -OV, -1.0), (xf, 1.0, 1.0)), k)
    yo = CHEV_D + 0.0007                         # the kink piece reaches aft of the sheared faces by k yo + OV
    return G.union([h, h.mirrored_y(), box3((xf - 1.5, -yo, -1.0), (xf + k * yo + OV + 0.0001, yo, 1.0))])


def _ms_cap_zone(C: Ctx) -> G.Mesh:
    """Zone of the FS-MS T-cap (+-flange_w about the chevron web)."""
    st = C.st["FS-MS"]
    k = math.tan(math.radians(float(st["sweep_deg"])))
    fw = float(st["flange_w"]) + 0.0005
    h = shear_x(box3((st["x"] - fw, -OV, -1.0), (st["x"] + fw, 1.0, 1.0)), k)
    return G.union([h, h.mirrored_y(), box3((st["x"] - fw, -0.003, -1.0), (st["x"] + fw + 0.001, 0.003, 1.0))])


def penetration_cuts(C: Ctx, member: str, axis=(0.0, 0.0, 1.0), margin: float = 0.006, half: float = 0.03):
    """Bores for the fuel lines / harness trunks of layout.fuel_lines / systems.harness through ``member``
    (diameter + ``margin`` for the sealed bulkhead union / grommet)."""
    a = np.asarray(axis, float)
    out = []
    items = [(f, p) for f in C.L["fuel_lines"] for p in f.get("penetrations", [])]
    items += [(t_, p) for t_ in C.L["systems"]["harness"]["trunks"] for p in t_.get("penetrations", [])]
    for f, p in items:
        if p["member"] != member:
            continue
        c = np.asarray(p["point"], float)
        r = 0.5 * float(f["diameter"]) + 0.5 * margin
        out.append(bore(c - half * a, c + half * a, r))
        if f.get("mirror"):
            c2 = c * np.array([1.0, -1.0, 1.0])
            out.append(bore(c2 - half * a, c2 + half * a, r))
    return out


def build_fwddeck(C: Ctx, frames) -> G.Mesh:
    m = C.mem["M-FWDDECK"]
    lo, hi = np.asarray(m["box"][0], float), np.asarray(m["box"][1], float)
    deck = inter(box3(lo, (hi[0] + 0.07, hi[1], hi[2])), C.body_env(MEM_IN, lo[0] - 0.005, hi[0] + 0.08))
    deck = diff(deck, [_aft_of(C, "FS-MS")] + penetration_cuts(C, "M-FWDDECK"))
    return pieces_above(deck)


def build_wellroof(C: Ctx) -> G.Mesh:
    m = C.mem["M-WELLROOF"]
    lo, hi = np.asarray(m["box"][0], float), np.asarray(m["box"][1], float)
    roof = inter(box3((lo[0] - 0.04, lo[1], lo[2]), hi), C.body_env(MEM_IN, lo[0] - 0.045, hi[0] + 0.005))
    roof = diff(roof, [_fwd_of(C, "FS-RS")] + penetration_cuts(C, "M-WELLROOF"))
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
    xs_o = [x for x in xs if abs(x - 2.9759) > 0.01]
    zs_o = [z for x, z in zip(xs, zs) if abs(x - 2.9759) > 0.01]
    lands = [rect(min(xs_o) - 0.013, 2.9399 - 0.0005, min(zs_o) - 0.013, max(zs_o) + 0.013),
             rect(3.0119 + 0.0005, max(xs_o) + 0.013, min(zs_o) - 0.013, max(zs_o) + 0.013),
             rect(2.9759 - 0.013, 2.9759 + 0.013, TRUN_B3_Z - 0.013, TRUN_B3_Z + 0.013)]
    wall = prism_y(clean_poly(xz), y0, y1)
    land = union([prism_y(l_, y0 - BEAM_LAND_PROUD, y0 + OV) for l_ in lands])
    beam = union([wall, land])
    beam = diff(beam, [_fwd_of(C, "FS-RS", MEM_GAP)])
    return pieces_above(inter(beam, C.body_env(MEM_IN, float(m["box"][0][0]) - 0.005, float(m["box"][1][0]) + 0.005)))


BEAM_LAND_PROUD = 0.0015    # gear-beam insert land stands 1.5 mm proud of the inboard face (8.3 mm insert depth)
TRUN_B3_Z = -0.0935         # middle beam bolt B3 5.5 mm lower than the layout (head clear of the top plate)
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
    xe = dorsal_end_x(C)                        # ends ahead of the firewall forward T-cap (splice tongue)
    q = [float(np.interp(xe, P[:, 0], P[:, k])) for k in (1, 2)]
    P = np.vstack([P[P[:, 0] < xe - 1e-4], [xe, q[0], q[1]]])
    hat = build_hat(C, P, None, DORSAL_W, DORSAL_H, DORSAL_T, DORSAL_FL, True)
    xa, xb = fin_fit_x
    relief = box3((xa - 0.001, 0.15 - 0.5 * DORSAL_W - 0.001, 0.0), (xb + 0.001, 0.15 + 0.5 * DORSAL_W + 0.001, 0.6))
    keep_fl = diff(C.body_env(0.0065 - 0.001, xa - 0.01, xb + 0.01), [C.body_env(0.0065 + DORSAL_T, xa - 0.02,
                                                                                  xb + 0.02)])
    relief = diff(relief, [keep_fl])
    return pieces_above(diff(hat, [relief]))


def dorsal_end_x(C: Ctx) -> float:
    st = C.st["FS3670"]
    return float(st["x_faces"][0]) - float(st.get("flange_w", 0.028)) - 0.0005


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
    xa_, xb_ = x0 + RISER_FLANGE_T + MEM_GAP, x1 - RISER_FLANGE_T - MEM_GAP      # between the bridle-fitting flanges
    floor = box3((xa_, -yw, z_floor - t), (xb_, yw, z_floor))
    walls = [box3((xa_, s_ * yw - (t if s_ > 0 else 0), z_floor - t), (xb_, s_ * yw + (0 if s_ > 0 else t), 0.3))
             for s_ in (1, -1)]
    pads = [box3((xa, -SPINE_PAD_HW, z_floor - pad), (xb, SPINE_PAD_HW, z_floor - t + OV))
            for xa, xb in ((xa_, x0 + SPINE_PAD_L), (x1 - SPINE_PAD_L, xb_))]
    U = inter(union([floor] + walls + pads), C.body_env(0.0065, x0 - 0.005, x1 + 0.02))
    fl = diff(inter(box3((x0, -yf, 0.10), (x1 + 0.01, yf, 0.3)), C.body_env(0.0065, x0 - 0.005, x1 + 0.02)),
              [C.body_env(0.0065 + t, x0 - 0.01, x1 + 0.03), box3((x0 - 0.1, -yw + t, 0.0), (x1 + 0.1, yw - t, 0.4))])
    sp = union([U, pieces_above(fl)])
    return pieces_above(diff(sp, [_aft_of(C, "FS-RS")]))


SPINE_FLOOR_TOP = 0.165     # top face of the spine floor (bridle-fitting base strips sit on it)
SPINE_PAD_L = 0.052         # 16-ply floor pad under each bridle fitting
SPINE_PAD_HW = 0.032        # the pads run 10 mm outboard of the channel walls under the floor (bolt edge distance)
RISER_FLANGE_T = 0.006      # bridle-fitting frame flange (layout 6 mm); the channel ends on it


def build_wellkeel(C: Ctx) -> G.Mesh:
    m = C.mem["M-WELLKEEL"]
    lo, hi = m["box"]
    return box3(lo, hi)


def build_plain_wall(C: Ctx, mid: str, z_top=None) -> G.Mesh:
    m = C.mem[mid]
    lo, hi = np.asarray(m["box"][0], float), np.asarray(m["box"][1], float)
    if z_top is not None:
        hi[2] = z_top
    return box_member(C, lo, hi, cut=penetration_cuts(C, mid, axis=(1.0, 0.0, 0.0), margin=0.002))


# =====================================================================================================================
# fittings (layout.chassis.fittings)
# =====================================================================================================================
def fuse_boxes(boxes, ov=OV) -> G.Mesh:
    """Union of axis-aligned boxes [(lo, hi), ...] that may share faces. For every pair of boxes with a common face
    patch a 'weld' box (the patch x [face - ov, face + ov]) is added: it lies inside the union, and makes the boolean
    fuse the boxes into one shell (exactly touching solids stay separate shells, whose shared vertices are later
    re-merged into a distorted mesh)."""
    B = [(np.asarray(lo, float), np.asarray(hi, float)) for lo, hi in boxes]
    welds = []
    for i in range(len(B)):
        for j in range(len(B)):
            if i == j:
                continue
            for k in range(3):
                if abs(B[i][1][k] - B[j][0][k]) > 1e-9:
                    continue
                o = [a for a in range(3) if a != k]
                lo_o = [max(B[i][0][a], B[j][0][a]) for a in o]
                hi_o = [min(B[i][1][a], B[j][1][a]) for a in o]
                if any(h - l <= 1e-9 for l, h in zip(lo_o, hi_o)):
                    continue
                lo, hi = np.zeros(3), np.zeros(3)
                lo[k], hi[k] = B[i][1][k] - ov, B[i][1][k] + ov
                for a, l, h in zip(o, lo_o, hi_o):
                    lo[a], hi[a] = l, h
                welds.append(box3(lo, hi))
    return union([box3(lo, hi) for lo, hi in B] + welds)


def boxes_union(boxes, ov=OV) -> G.Mesh:
    return fuse_boxes(boxes, ov)


def build_trunnion(C: Ctx) -> G.Mesh:
    """Main-gear trunnion fitting (7075-T651, starboard): beam flange (6.5 mm over the proud insert land), two bearing
    lugs 26 mm with 20 H7 bores on the trunnion axis, top plate on the well roof with head pockets for the 4 roof bolts,
    clear of the retraction-EMA allocation envelope (ACT-MLG-EMA)."""
    f = C.fit["F-TRUNNION"]
    yb = float(C.mem["M-GEARBEAM"]["box"][0][1]) - BEAM_LAND_PROUD          # land face
    boxes = []
    for k, (lo, hi) in enumerate(f["boxes"]):
        lo, hi = list(map(float, lo)), list(map(float, hi))
        if k < 3:                                     # beam flange boxes
            hi[1] = yb
        boxes.append((lo, hi))
    x0, x1 = float(f["box"][0][0]), float(f["box"][1][0])
    # top plate on the well roof over the full fitting length (roof bolts fore and aft of the lugs)
    boxes.append(((x0, TRUN_ROOF_Y[0] - 0.012, -0.0868), (x1, 0.377, -0.0668)))
    m = fuse_boxes(boxes)
    px, py, pz = f["pivot"]
    cuts = [bore((b["x"] - 0.05, py, pz), (b["x"] + 0.05, py, pz), 0.5 * float(b["bore"]) + 0.0000105)
            for b in f["bearings"]]
    ema = next(a for a in C.L["systems"]["actuators"] if a["id"] == "ACT-MLG-EMA")["cylinder"]
    c = np.asarray(ema["center"], float)
    ax = np.asarray(ema["axis"], float)
    cuts.append(bore(c - (ema["half_length"] + 0.0005) * ax, c + (ema["half_length"] + 0.0005) * ax,
                     ema["radius"] + 0.0005))
    return diff(m, cuts)


TRUN_ROOF_Y = (0.333, 0.362)    # roof bolts B6-B9: rows 29 mm apart (sealed dome nutplates 28 mm long; layout 22 mm)
TRUN_ROOF_X = (2.9029, 3.0489)  # ... fore / aft of the lugs (layout: in the lugs, above the 20 H7 bores)


def trunnion_roof_point(b: dict):
    x, y, z = b["point"]
    return (TRUN_ROOF_X[0] if x < 2.9759 else TRUN_ROOF_X[1]), (TRUN_ROOF_Y[0] if y < 0.35 else TRUN_ROOF_Y[1]), z


def build_uplock(C: Ctx) -> G.Mesh:
    f = C.fit["F-UPLOCK"]
    lo, hi = f["box"]
    px, py, pz = f["point"]
    xs = (px - UPLOCK_BOLT_DX, px + UPLOCK_BOLT_DX)
    base = box3((xs[0] - 0.010, UPLOCK_BOLT_Y[0] - 0.010, hi[2] - UPLOCK_BASE_T),
                (xs[-1] + 0.010, UPLOCK_BOLT_Y[1] + 0.010, hi[2]))
    boss = box3((px - 0.008, py - 0.012, lo[2]), (px + 0.008, py + 0.012, hi[2] - UPLOCK_BASE_T + OV))
    m = union([base, boss])
    return diff(m, [bore((px, py - 0.02, pz + 0.006), (px, py + 0.02, pz + 0.006), 0.0026)])   # hook pivot pin


UPLOCK_BASE_T = 0.005
UPLOCK_BOLT_Y = (0.237, 0.263)  # 4 x M5 into sealed dome nutplates: 26 mm row pitch > nutplate length (layout 20 mm)
UPLOCK_BOLT_DX = 0.0135         # bolt columns +-13.5 mm about the hook point: heads clear of the hook boss


def build_ng_pivot(C: Ctx) -> G.Mesh:
    """Two 7075 pivot blocks (6 mm) on the inboard faces of the keel walls with the flanged 16 H7 bushings (bores)."""
    f = C.fit["F-NG-PIVOT"]
    m = fuse_boxes(f["boxes"])
    px, py, pz = f["pivot"]
    return diff(m, [bore((px, -0.05, pz), (px, 0.05, pz), 0.5 * float(f["bore"]) + 0.0000095)])


def build_spindle_node(C: Ctx) -> G.Mesh:
    """Stabilator node (7075, starboard): U base flange 8 mm on the firewall aft face, inboard arm with the 61805 bearing
    boss (bore 37, 7 mm seat + circlip land), outboard cheek 7 mm with the 32 mm spindle clearance hole."""
    f = C.fit["F-SPINDLE-NODE"]
    B = [np.asarray(b, float) for b in f["boxes"]]
    boxes = []
    for k, (lo, hi) in enumerate(B):
        lo = lo.copy()
        if k >= 3:
            lo[0] -= OV
        boxes.append((lo, hi))
    cy = f["cylinder"]
    c = np.asarray(cy["center"], float)
    a = np.asarray(cy["axis"], float)
    m = union([fuse_boxes(boxes), bore(c - cy["half_length"] * a, c + cy["half_length"] * a, cy["radius"], n=48)])
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
    B[1][0][1] = min(B[1][0][1], engine_foot_points(C)[0][1] - 0.016)      # 2 D edge for the spread foot bolts
    base, ear = box3(B[1][0], B[1][1]), box3(B[3][0], B[3][1])
    web = box3(B[2][0], (B[2][1][0], B[2][1][1], B[2][1][2]))
    bridge = box3((B[2][1][0] - OV, B[3][0][1], B[3][0][2]), (B[3][0][0] + OV, B[3][1][1], B[3][0][2] + 0.005))
    return union([base, web, ear, bridge])


DORSAL_TONGUE_T = 0.005


def build_emount_lo(C: Ctx) -> tuple[G.Mesh, G.Mesh]:
    """Lower engine-mount firewall fitting: aft foot pad 10 mm (7075) and forward backing plate 5 mm."""
    f = C.fit["F-EMOUNT-LO"]
    lo, hi = np.asarray(f["box"][0], float), np.asarray(f["box"][1], float)
    pts = engine_foot_points(C)[2:]
    lo[1] = min(lo[1], pts[0][1] - 0.016)
    hi[1] = max(hi[1], pts[1][1] + 0.016)
    hi[2] = max(hi[2], pts[0][2] + 0.034)                       # the foot plate reaches 34 mm above the bolt row
    xa = float(C.st["FS3670"]["x"])
    xf = float(C.st["FS3670"]["x_faces"][0])
    return box3((xa, lo[1], lo[2]), (xa + 0.010, hi[1], hi[2])), box3((xf - 0.005, lo[1], lo[2]), (xf, hi[1], hi[2]))


def build_fin_front(C: Ctx) -> G.Mesh:
    """Fin front-spar root clevis (7075): two 6 mm ears either side of the 8 mm lug slot, base block 20 mm on the
    FS3480 aft face for the 2 x M6 frame bolts."""
    f = C.fit["F-FIN-FRONT"]
    lo, hi = np.asarray(f["box"][0], float), np.asarray(f["box"][1], float)
    lo[1] -= FIN_FRONT_GROW
    hi[1] += FIN_FRONT_GROW
    m = box3(lo, hi)
    zb = max(b["point"][2] for b in f["bolts"] if b["group"] == "frame") + 0.012
    slot = box3((lo[0] + 0.006, lo[1] - 0.01, zb), (hi[0] - 0.006, hi[1] + 0.01, hi[2] + 0.01))
    return diff(m, [slot])


FIN_FRONT_GROW = 0.0005     # base 43 mm wide (layout 42): frame bolts 19 mm apart (2.5 D + hole in the ring land)


def fin_front_frame_points(C: Ctx):
    pts = [b["point"] for b in C.fit["F-FIN-FRONT"]["bolts"] if b["group"] == "frame"]
    yc = 0.5 * (pts[0][1] + pts[1][1])
    return [(p[0], yc + (0.0095 if p[1] > yc else -0.0095), p[2]) for p in pts]


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
    B[0] = np.array([[B[0][0][0], -RISER_FLANGE_HW, RISER_FLANGE_Z0], [B[0][1][0], RISER_FLANGE_HW, B[0][1][2]]])
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
    if fid == "F-RISER-AFT":
        # the aft fitting's frame flange follows the chevron forward face of FS-RS (kink piece flat at y = 0)
        lo, hi = B[0][0], B[0][1]
        k = math.tan(math.radians(float(C.st["FS-RS"]["sweep_deg"])))
        ms[0] = diff(box3(lo, (hi[0] + k * float(hi[1]) + 0.001, hi[1], hi[2])), [_aft_of(C, "FS-RS")])
    m = union(ms)
    px, py, pz = f["point"]
    return diff(m, [bore((px, -0.03, pz + RISER_PIN_DZ), (px, 0.03, pz + RISER_PIN_DZ), 0.004 + 0.00002)])


RISER_PIN_DZ = 0.0            # shackle pin on the layout point (e = 15.5 mm to the ear top)
RISER_FLANGE_HW = 0.0235      # frame flange +-23.5 mm (layout +-18): frame bolts B5/B6 outboard of the ears
RISER_FLANGE_Z0 = 0.1605      # ... reaching down past the channel end (layout 0.165): 12.5 mm frame-web edge
RISER_FRAME_BOLT = (0.0135, 0.1725)   # |y|, z of B5/B6


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
    edge = diff(outer, [inner] + [prism_x(n, xs0 - 0.01, xa + 0.02) for n, sd in _cap_notch_sides(C, "FS3670")
                                  if sd != "fwd"])
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
    # feet (4 mm plates 56 x 32 centred on their bolt pair, welded strut block between the bolt heads) and struts
    feet = {}
    for i, ft in enumerate(eng["feet"]):
        yc, zc = engine_foot_centre(C, float(ft[1]), float(ft[2]))
        s = 1.0 if yc > 0 else -1.0
        hw = 0.5 * FOOT_W
        if zc > 0.2:                     # upper feet: strut block below the bolt row, on a plate extension
            ms.append(box3((foot_x, yc - hw, zc - 0.016), (foot_x + FOOT_T, yc + hw, zc + 0.016)))
            ya, yb = sorted((yc - s * hw, s * FOOT_EXT_Y))
            ms.append(box3((foot_x, ya, zc - 0.016 - 0.018), (foot_x + FOOT_T, yb, zc - 0.016 + OV)))
            zb = zc - FOOT_BLOCK_DZ
        else:                            # lower feet: strut block above the bolt row
            ms.append(box3((foot_x, yc - hw, zc - 0.016), (foot_x + FOOT_T, yc + hw, zc + 0.034)))
            zb = zc + FOOT_BLOCK_DZ
        ms.append(box3((foot_x + FOOT_T - OV, yc - FOOT_BLOCK_HW, zb - 0.008), (foot_x + FOOT_T + FOOT_BLOCK,
                                                                                 yc + FOOT_BLOCK_HW, zb + 0.008)))
        feet[f"foot{i + 1}"] = np.array([foot_x + FOOT_T + FOOT_BLOCK + ro_s, yc, zb])
    for tb in eng["tubes"]:
        a = feet[tb["from"]].copy()
        b = nodes[int(tb["to"][-1]) - 1]
        ms.append(G.tube(ro_s, ro_s - t_s, a, b, n=16))
        ms.append(G.sphere(ro_s + 0.0005, a, n=16))
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
FOOT_W, FOOT_H = 0.057, 0.032           # foot plate 57 x 32 (F-EMOUNT-LO 56 x 32), bolts +-12.5 mm about its centre
FOOT_BLOCK, FOOT_BLOCK_HW = 0.010, 0.0045   # welded strut block between the two M8 heads


FOOT_BOLT_PITCH = 0.025     # 2 x M8 per foot 25 mm apart (layout 24 mm < 2.5 D + hole radius in the firewall)
FOOT_BLOCK_DZ = 0.020       # strut block 20 mm below (upper feet) / above (lower feet) the bolt row
FOOT_EXT_Y = 0.1395         # upper foot plate extension stays inboard of the corner fitting's clevis bridge


def engine_foot_points(C: Ctx):
    """Starboard engine-mount foot bolts through the firewall stack: upper pair (F-FW-CORNER B1/B2), lower pair
    (F-EMOUNT-LO B1/B2), each pair about its layout centre."""
    out = []
    for fid in ("F-FW-CORNER", "F-EMOUNT-LO"):
        pr = [b["point"] for b in C.fit[fid]["bolts"] if b["group"] == "engine foot"]
        yc = 0.5 * (pr[0][1] + pr[1][1])
        out += [(float(pr[0][0]), yc - 0.5 * FOOT_BOLT_PITCH, float(pr[0][2])),
                (float(pr[0][0]), yc + 0.5 * FOOT_BOLT_PITCH, float(pr[0][2]))]
    return out


def engine_foot_centre(C: Ctx, y: float, z: float):
    """(y, z) centre of the bolt pair of the foot nearest to (y, z) (either side)."""
    pts = engine_foot_points(C)
    s = 1.0 if y >= 0 else -1.0
    pairs = [pts[:2], pts[2:]]
    pair = min(pairs, key=lambda pr: abs(0.5 * (pr[0][2] + pr[1][2]) - z))
    return s * 0.5 * (pair[0][1] + pair[1][1]), 0.5 * (pair[0][2] + pair[1][2])


NODE_B46_Y = 0.2085         # node bolts B4 / B6 1.5 mm outboard of the layout: 13.5 mm (>= 2.5 D) to C-FW-PUSHROD


def node_firewall_points(C: Ctx):
    """Starboard stabilator-node bolts B1-B7 through the firewall stack."""
    out = []
    for b in C.fit["F-SPINDLE-NODE"]["bolts"]:
        if b["group"] != "firewall":
            continue
        x, y, z = map(float, b["point"])
        if b["id"] in ("B4", "B6"):
            y = NODE_B46_Y
        out.append((x, y, z))
    return out
CUP_ID, CUP_T, CUP_BOTTOM_T = 0.041, 0.0012, 0.0025


# =====================================================================================================================
# small parts: frame U-ring legs and lower segment, clips, backing plates, spacers, trays, liners, brackets, anchors
# =====================================================================================================================
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


def _clip_geom(C: Ctx, sid: str, side: int):
    st = C.st[sid]
    m = C.mem["M-CHINE"]
    P = np.asarray(m["paths"][0 if float(st["x"]) < 2.6 else 1], float)
    w = float(m["section"]["w"])
    xf = float(st["x"]) + side * 0.5 * float(st["t"])
    yw = lambda x: float(np.interp(x, P[:, 0], P[:, 1])) - 0.5 * w           # noqa: E731  J web inboard face
    zc = lambda x: float(np.interp(x, P[:, 0], P[:, 2]))                     # noqa: E731
    notch = next(c for c in st["cutouts"] if c.get("kind") == "longeron notch")
    ya = min(float(notch["y"][0]) - 0.0105, yw(xf) - CLIP_LEG_A + 0.0085)
    return P, xf, yw, zc, ya


def build_clip(C: Ctx, sid: str, side: int):
    """7075 shear clip at a chine-longeron notch (frame leg 30 mm on the frame face, longeron leg 20 mm along the J web
    following its plan line, 28 mm tall, 2.0 mm): returns (mesh, frame-bolt points, longeron-bolt points, x of the
    frame face)."""
    P, xf, yw, zc, ya = _clip_geom(C, sid, side)
    ct, L, la, lb = CLIP_T, CLIP_L, CLIP_LEG_A, CLIP_LEG_B
    z0, z1 = zc(xf) - L / 2, zc(xf) + L / 2
    xa_, xb_ = sorted((xf, xf + side * ct))
    legA = box3((xa_, yw(xf) - la, z0), (xb_, yw(xf), z1))
    xe = xf + side * lb
    xs = sorted({xf, xe} | {float(v) for v in P[:, 0] if min(xf, xe) < v < max(xf, xe)})
    rings = [np.array([[x, yw(x) - ct, zc(x) - L / 2], [x, yw(x), zc(x) - L / 2], [x, yw(x), zc(x) + L / 2],
                       [x, yw(x) - ct, zc(x) + L / 2]]) for x in xs]
    legB = G.loft(rings)
    pts_a = [(xf, ya, zc(xf) - CLIP_BOLT_DZ), (xf, ya, zc(xf) + CLIP_BOLT_DZ)]
    xb = xf + side * (lb - 0.0085)
    pts_b = [(xb, yw(xb), zc(xb))]
    return union([legA, legB]), pts_a, pts_b, xf


def clip_web_normal(C: Ctx, sid: str, side: int, x: float):
    """Unit normal (+y side) of the chine J web at x (the web follows the path in plan)."""
    P, _xf, yw, _zc, _ya = _clip_geom(C, sid, side)
    dx = 0.002
    d = np.array([2 * dx, yw(x + dx) - yw(x - dx), 0.0])
    n = np.array([-d[1], d[0], 0.0])
    return n / np.linalg.norm(n)


CLIP_T, CLIP_L, CLIP_LEG_A, CLIP_LEG_B = 0.002, 0.030, 0.030, 0.020
CLIP_BOLT_DZ = 0.0065       # frame-leg bolts at +-6.5 mm (pitch 13 >= 2.5 D + hole radius in the frame)
CLIP_SIDE = {"FS1110": 1, "FS1330": 1, "FS1490": 1, "FS1810": 1, "FS-FUEL": -1, "FS-GEAR": 1, "FS3480": 1}


def bent_l(La: float, Lb: float, t: float, r: float, n: int = 10) -> Polygon:
    """Cross-section of a bent sheet angle in local (u, v): leg A along +u (outer face v = 0), leg B along +v (outer
    face u = 0), inside bend radius r (processes.sheet_metal_*.min_bend_radius_t)."""
    ro = r + t
    pts = [(La, 0.0), (ro, 0.0)]
    for a in np.linspace(-0.5 * math.pi, -math.pi, n)[1:-1]:
        pts.append((ro + ro * math.cos(a), ro + ro * math.sin(a)))
    pts += [(0.0, ro), (0.0, Lb), (t, Lb), (t, ro)]
    for a in np.linspace(-math.pi, -0.5 * math.pi, n)[1:-1]:
        pts.append((ro + r * math.cos(a), ro + r * math.sin(a)))
    pts += [(ro, t), (La, t)]
    return Polygon(pts).buffer(0)


def build_uring(C: Ctx):
    """FS3738 engine-bay lower U-ring: 2024-T3 legs (2.0 mm web on the station faces, 20 mm flange formed forward at
    the skin line over the 12 mm bend radius of processes.sheet_metal_aluminium (6 t), below ring_z_max) for
    |y| >= 0.15 - SEG_OVERLAP, and the machined 7075 lower I-segment (60 mm deep, flanges 20 x 1.6, web 2.0, web on
    the forward face of the legs) between y +-0.15 with the aft-keel passage; the legs lap the segment web (3 x M5
    each end). Both keep >= 10 mm to the engine dynamic envelope. Returns (starboard leg, segment)."""
    st = C.st["FS3738"]
    x0, x1 = float(st["x_faces"][0]), float(st["x_faces"][1])
    t = x1 - x0
    inset = float(st["inset"])
    depth = float(st["ring_depth"])
    zmax = float(st["ring_z_max"])
    ys = float(st["lower_segment"]["y"][1])
    sz = C.S["structures"]["sizing"]["body"]["fs3738_lower_segment"]
    r = URING_BEND_R
    eng = engine_keepout(C, 0.010 + 0.001)
    lim = rect(ys - SEG_OVERLAP, 1.0, -1.0, zmax)
    # bend + flange (x0 - flange .. x0): inset between the outer bend surface and the inner bend surface
    xc = x0 - r                                                   # bend axis (x) of the inside radius
    rings_o, rings_i = [], []
    xs = list(np.linspace(x0 - 0.020, xc, 3)) + list(xc + r * np.sin(np.linspace(0.0, 0.5 * math.pi, 9))[1:])
    for x in xs:
        dx = max(x - xc, 0.0)
        n_o = inset + t + r - math.sqrt(max((r + t) ** 2 - dx ** 2, 0.0)) if dx > 0 else inset
        n_i = inset + t + r - math.sqrt(max(r ** 2 - dx ** 2, 0.0)) if dx > 0 else inset + t
        rings_o.append((x, C.sec(x, n_o)))
        rings_i.append((x, C.sec(x, n_i)))
    rings_o[-1] = (x0 + OV, rings_o[-1][1])
    E_o = _loft_sections([(x, p) for x, p in rings_o])
    E_i = _loft_sections([(x, p) for x, p in rings_i[:-1]] + [(x0 + OV + 0.001, rings_i[-1][1])])
    bend = diff(E_o, [E_i])
    # web (x0 .. x1): from the outer bend surface to the ring depth
    rw = []
    for x in np.linspace(x0, x1, 4):
        dx = x - xc
        rw.append((x, C.sec(x, inset + t + r - math.sqrt(max((r + t) ** 2 - dx ** 2, 0.0)))))
    web = diff(_loft_sections(rw), [prism_x(C.sec(0.5 * (x0 + x1), inset + depth), x0 - 0.01, x1 + 0.01)])
    keep_fl = prism_x(rect(ys + 0.0005, 1.0, -1.0, zmax), x0 - 0.03, x0 + OV)
    keep_web = prism_x(lim, x0 - 0.001, x1 + 0.001)
    leg = union([inter(bend, keep_fl), inter(web, keep_web)])
    leg = diff(leg, [eng])
    # lower segment (machined), web on the forward face of the legs
    xm = x0 - 0.5 * float(sz["web_t_m"])
    outer = C.sec(xm, inset)
    d = float(sz["depth_m"])
    tw, tf, fw = float(sz["web_t_m"]), float(sz["flange_t_m"]), float(sz["flange_w_m"])
    sb = outer.difference(C.sec(xm, inset + d)).intersection(rect(-ys, ys, -1, 0.10))
    webs = prism_x(clean_poly(sb), xm - tw / 2, xm + tw / 2)
    ofl = outer.difference(C.sec(xm, inset + tf)).intersection(rect(-ys, ys, -1, 0.10))
    ifl = C.sec(xm, inset + d - tf).difference(C.sec(xm, inset + d)).intersection(rect(-ys, ys, -1, 0.0))
    k = C.mem["M-AFTKEEL"]
    yl = float(k["lands"][0]["y"][1]) + 0.0005
    keel_cut = rect(-0.5 * float(k["section"]["w"]) - 0.0003, 0.5 * float(k["section"]["w"]) + 0.0003, -1,
                    float(k["box"][1][2]) + 0.0003).union(rect(-yl, yl, -1, float(k["box"][0][2]) + 0.0019))
    parts = [webs, prism_x(clean_poly(ofl.difference(keel_cut)), xm - fw / 2, xm + fw / 2),
             prism_x(clean_poly(ifl), xm - fw / 2, xm + fw / 2)]
    segm = diff(union(parts), [prism_x(keel_cut, xm - 0.02, xm + 0.02)])
    # the legs clear the segment's outer flange in the lap (notch 0.5 mm)
    leg = diff(leg, [prism_x(C.sec(x0, 0.0).difference(C.sec(x0, inset + tf + 0.0005)).intersection(
        rect(0.0, ys + 0.0005, -1, 1)), x0 - 0.03, x1 + 0.01)])
    return pieces_above(leg, 1e-8), diff(segm, [eng])


URING_BEND_R = 0.012        # 2024-T3 bend radius >= 6 t (processes.sheet_metal_aluminium.min_bend_radius_t)
SEG_OVERLAP = 0.050         # 2024 legs lap the 7075 lower segment web from y +-0.10 (3 x M5 at 15 mm pitch)


def uring_splice_points(C: Ctx):
    """Starboard leg-to-segment splice bolts (3 x M5, axis x): on the leg band centre in its flat web part."""
    st = C.st["FS3738"]
    x1 = float(st["x_faces"][1])
    ys = float(st["lower_segment"]["y"][1])
    inset = float(st["inset"])
    n = 0.026                                                    # flat web: 15.3 .. 36.5 mm from the OML
    pts = []
    for y in (ys - 0.040, ys - 0.025, ys - 0.010):
        g = LineString([(y, -1.0), (y, 1.0)]).intersection(C.sec(x1, n).exterior)
        zs = [p.y for p in (g.geoms if hasattr(g, "geoms") else [g])]
        pts.append((x1, y, min(zs)))
    return pts


# ---------------------------------------------------------------------------------------------------------------------
# firewall-side fittings added by the detail design
# ---------------------------------------------------------------------------------------------------------------------
CHE_T = 0.005               # chine end fitting base flange on the firewall forward face
CHE_TONGUE_T = 0.005        # tongue on the inboard face of the J web
CHE_TONGUE_X0 = 3.585       # forward end of the tongue (2 x M5 Ti through the J web at x 3.600 / 3.625)
CHINE_END_GAP = 0.0005      # aft chine piece ends 0.5 mm ahead of the end fitting base


def chine_aft_end_x(C: Ctx) -> float:
    return float(C.st["FS3670"]["x_faces"][0]) - CHE_T - CHINE_END_GAP


def _chine_line(C: Ctx, x: float):
    """(yc, zc) of the aft chine piece centre line at x."""
    P = np.asarray(C.mem["M-CHINE"]["paths"][1], float)
    return float(np.interp(x, P[:, 0], P[:, 1])), float(np.interp(x, P[:, 0], P[:, 2]))


def chine_web_normal(C: Ctx):
    """Unit normal (outboard) of the aft chine J web near the firewall (the path's last segment, in plan)."""
    P = np.asarray(C.mem["M-CHINE"]["paths"][1], float)
    d = P[-1, :2] - P[-2, :2]
    d = d / np.linalg.norm(d)
    return np.array([-d[1], d[0], 0.0]) * (1.0 if d[0] > 0 else -1.0)


def build_chine_end(C: Ctx) -> G.Mesh:
    """Chine-longeron end fitting (7075, starboard): 5 mm base flange on the firewall forward face carrying the outboard
    column of the stabilator-node bolts (B4-B7 through the firewall stack), and a 5 mm tongue on the inboard face of the
    J web (2 x M5 Ti through the web): the chine longeron's axial load enters the node on the firewall plane."""
    m = C.mem["M-CHINE"]
    w, h = float(m["section"]["w"]), float(m["section"]["h"])
    xf = float(C.st["FS3670"]["x_faces"][0])
    pts = node_firewall_points(C)[3:7]                          # B4-B7
    ys, zs = [p[1] for p in pts], [p[2] for p in pts]
    base = box3((xf - CHE_T, min(ys) - 0.010, min(zs) - 0.010), (xf, max(ys) + 0.010, max(zs) + 0.010))
    base = inter(base, C.body_env(MEM_IN, xf - 0.02, xf + 0.01))
    rings = []
    for x in (CHE_TONGUE_X0, xf - CHE_T + OV):
        yc, zc = _chine_line(C, x)
        yw = yc - 0.5 * w
        rings.append(np.array([[x, yw - CHE_TONGUE_T, zc - 0.5 * h + 0.002], [x, yw, zc - 0.5 * h + 0.002],
                               [x, yw, zc + 0.5 * h - 0.002], [x, yw - CHE_TONGUE_T, zc + 0.5 * h - 0.002]]))
    tongue = G.loft(rings)
    return union([base, tongue])


def chine_end_bolt_points(C: Ctx):
    """2 x M5 Ti through the tongue and the J web (on the web's inboard face, at the chine centre height)."""
    w = float(C.mem["M-CHINE"]["section"]["w"])
    out = []
    for x in (3.600, 3.625):
        yc, zc = _chine_line(C, x)
        out.append((x, yc - 0.5 * w - CHE_TONGUE_T, zc))
    return out


def _dorsal_crown_z(C: Ctx, x: float) -> float:
    """Underside of the dorsal hat crown at x."""
    return _dorsal_z(C, x) - 0.5 * DORSAL_H


def build_fw_upper_plate(C: Ctx) -> G.Mesh:
    """Firewall upper backing plate (7075, 5 mm on the sandwich forward face, starboard): forward plate of the corner
    fitting (engine-mount upper foot M8 x 2), the inboard column of the stabilator-node bolts (B1-B3), and the
    dorsal-longeron splice tongue against the outer face of the hat's inboard wall (2 x M4 Ti along y)."""
    f = C.fit["F-FW-CORNER"]
    lo, hi = np.asarray(f["boxes"][0][0], float), np.asarray(f["boxes"][0][1], float)
    lo[1] = min(lo[1], engine_foot_points(C)[0][1] - 0.016)
    xf = float(C.st["FS3670"]["x_faces"][0])
    nb = {b["id"]: b["point"] for b in C.fit["F-SPINDLE-NODE"]["bolts"]}
    yb1, zb1 = nb["B1"][1], nb["B1"][2]
    yb3, zb3 = nb["B3"][1], nb["B3"][2]
    poly = unary_union([rect(lo[1], hi[1], lo[2], hi[2]),
                        rect(yb1 - 0.010, yb1 + 0.010, zb1 - 0.010, lo[2] + OV),
                        rect(yb1 - 0.010, yb3 + 0.010, zb1 - 0.010, zb3 + 0.010)])
    plate = prism_x(clean_poly(poly), xf - 0.005, xf)
    plate = inter(plate, C.body_env(MEM_IN, xf - 0.02, xf + 0.01))
    yw = 0.15 - 0.5 * DORSAL_W                                   # outer face of the hat's inboard wall
    tongue = box3((DORSAL_TONGUE_X0, yw - DORSAL_TONGUE_T, DORSAL_TONGUE_Z[0]), (xf - 0.005 + OV, yw,
                                                                                DORSAL_TONGUE_Z[1]))
    return union([plate, tongue])


DORSAL_TONGUE_X0 = 3.592
DORSAL_TONGUE_Z = (0.2985, 0.3160)           # clear of the B2 nut (z <= 0.2975) and of the hat's skin flange
DORSAL_SPLICE_BOLTS = ((3.600, 0.3075), (3.615, 0.3075))   # 2 x M4 Ti along y through the tongue and the hat wall


def build_emlo_back(C: Ctx) -> G.Mesh:
    return build_emount_lo(C)[1]


FW_SPACER_WALL = 0.001      # stainless spacer tube wall beyond the bolt-probe radius


def build_spacers(C: Ctx, pts) -> G.Mesh:
    """AISI 304 spacer tubes in the firewall air gap (sandwich aft face -> shield forward face), one per through-bolt
    of the firewall stack: ``pts`` [(y, z, size_mm), ...]."""
    from .fastener_catalog import clearance
    xf, xs, xs0, xa = fw_planes(C)
    ms = []
    for y, z, size in pts:
        ri = 0.5 * clearance(size) + 0.0001
        ro = 1.6 * 0.5 * clearance(size) + FW_SPACER_WALL
        ms.append(G.tube(ro, ri, (xs, y, z), (xs0, y, z), n=32))
    return union(ms)


def build_standoffs(C: Ctx, keep_out_yz) -> tuple[G.Mesh, list]:
    """12 AISI 304 stand-offs (d 10 / 3.2 mm, riveted) carrying the shield 7.6 mm aft of the sandwich, spread round the
    firewall section 30 mm inside the edge band, clear of the fittings / cut-outs (``keep_out_yz`` polygons).
    Returns (mesh, [(y, z), ...])."""
    xf, xs, xs0, xa = fw_planes(C)
    st = C.st["FS3670"]
    ring = C.sec(xa, float(st["inset"]) + 0.030).exterior
    bad = unary_union(keep_out_yz).buffer(0.010)
    L = ring.length
    cand = [ring.interpolate(s) for s in np.linspace(0.0, L, 241)[:-1]]
    cand = [(p.x, p.y) for p in cand if p.x > 0.02 and not bad.contains(Point(p.x, p.y))]
    cand.sort(key=lambda q: math.atan2(q[1], q[0]))
    k = 6
    picks = [cand[int(round(i * (len(cand) - 1) / (k - 1)))] for i in range(k)]
    pts = picks + [(-y, z) for y, z in picks]
    ms = [G.tube(0.005, 0.0016, (xs, y, z), (xs0, y, z), n=24) for y, z in pts]
    return union(ms), pts


def build_keel_clip(C: Ctx) -> G.Mesh:
    """Aft keel / U-ring lower segment angle clip (7075, 2 mm, starboard): leg on the segment web forward face, leg on
    the keel wall outboard face (1 x M5 each), in the free channel bay between F-VENTRAL-1 and -2."""
    st = C.st["FS3738"]
    xw = float(st["x_faces"][0]) - float(C.S["structures"]["sizing"]["body"]["fs3738_lower_segment"]["web_t_m"])
    k = C.mem["M-AFTKEEL"]
    yo = 0.5 * float(k["section"]["w"])
    legA = box3((xw - KCLIP_T, yo + 0.0003, KCLIP_Z[0]), (xw, yo + 0.023, KCLIP_Z[1]))
    legB = box3((xw - KCLIP_L, yo, KCLIP_Z[0]), (xw - OV * 0 - 0.0, yo + KCLIP_T, KCLIP_Z[1]))
    return union([legA, legB])


KCLIP_T, KCLIP_L, KCLIP_Z = 0.002, 0.027, (-0.085, -0.060)


def keel_clip_points(C: Ctx):
    st = C.st["FS3738"]
    xw = float(st["x_faces"][0]) - float(C.S["structures"]["sizing"]["body"]["fs3738_lower_segment"]["web_t_m"])
    yo = 0.5 * float(C.mem["M-AFTKEEL"]["section"]["w"])
    zc = 0.5 * sum(KCLIP_Z)
    return (xw - KCLIP_T, yo + 0.0128, zc), (xw - KCLIP_L + 0.010, yo + KCLIP_T, zc)


# ---------------------------------------------------------------------------------------------------------------------
# equipment trays (layout.chassis.trays), parachute brackets, riser washer plates, turret rail anchors, fuel liners
# ---------------------------------------------------------------------------------------------------------------------
TRAY_T = 0.003              # CFRP trays 3 mm (layout.chassis.trays)
TRAY_FL = 0.030             # bolting flanges of the CFRP trays
SIDEBAY_PORT_RELIEF = 0.075  # port tray flange on FS1110 stops at |y| 0.072 (nose-door actuator EQ-NDOORACT on FS1110)


def build_sidebay_tray(C: Ctx, side: int) -> G.Mesh:
    """Avionics side-bay tray (CFRP 3 mm, moulded with two bolting flanges): floor of the bay at its lower bound, flange
    on the keel-wall outboard face and flange on the FS1110 forward face (M4 into potted inserts). ``side`` +1 builds
    TR-SIDEBAY-R in +y, -1 TR-SIDEBAY-L in -y (with the relief round the nose-door actuator EQ-NDOORACT)."""
    bb = C.L["rules"]["boxes"]["avionics_side_bays"]
    z0 = float(bb["z"][0])
    kw = C.mem["M-KEELWALL"]["box"]
    yk = float(kw[1][1])                                         # keel wall outboard face
    xf = float(C.st["FS1110"]["x"]) - 0.5 * float(C.st["FS1110"]["t"])
    x0 = float(bb["x"][0])
    plate = box3((x0, yk, z0 - TRAY_T), (xf, 0.30, z0))
    fl_k = box3((x0, yk, z0 - OV), (xf, yk + TRAY_T, z0 + TRAY_FL))
    y_fl1 = SIDEBAY_PORT_RELIEF - 0.003 if side < 0 else 0.30
    fl_f = box3((xf - TRAY_T, yk + TRAY_T - OV, z0 - OV), (xf, y_fl1, z0 + TRAY_FL))
    m = union([plate, fl_k, fl_f])
    if side < 0:                                                 # EQ-NDOORACT relief (mirror of the port box)
        eq = next(e for e in C.L["systems"]["equipment"] if e["id"] == "EQ-NDOORACT")["box"]
        lo, hi = eq
        m = diff(m, [box3((lo[0] - 0.003, -hi[1] - 0.003, lo[2] - 0.003), (xf + 0.01, -lo[1] + 0.003, hi[2] + 0.003))])
    m = inter(m, C.body_env(MEM_IN, x0 - 0.01, xf + 0.01))
    m = pieces_above(m, 1e-7)
    return m if side > 0 else m.mirrored_y()


def sidebay_tray_points(C: Ctx, side: int):
    """(keel-wall flange bolts axis -y, frame flange bolts axis +x) for the starboard-modelled tray."""
    bb = C.L["rules"]["boxes"]["avionics_side_bays"]
    z0 = float(bb["z"][0])
    yk = float(C.mem["M-KEELWALL"]["box"][1][1])
    xf = float(C.st["FS1110"]["x"]) - 0.5 * float(C.st["FS1110"]["t"])
    x0 = float(bb["x"][0])
    zb = z0 + 0.5 * TRAY_FL
    kb = [(x0 + 0.020, yk + TRAY_T, zb), (xf - 0.030, yk + TRAY_T, zb)]
    yfb = [yk + TRAY_T + 0.0105] if side < 0 else [yk + TRAY_T + 0.020, yk + TRAY_T + 0.070]
    fb = [(xf - TRAY_T, y, zb) for y in yfb]
    return kb, fb


def build_fwdbay_tray(C: Ctx) -> G.Mesh:
    """Forward-bay tray TR-FWDBAY (CFRP 3 mm): buffer-battery floor between FS0300 and FS0600 (the FTS unit hangs
    under it), with a downward bolting flange on each frame (M4 into potted inserts); clipped to the OML inset."""
    bb = C.L["rules"]["boxes"]["forward_bay"]
    z0 = float(bb["z"][0])
    xa = float(C.st["FS0300"]["x"]) + 0.5 * float(C.st["FS0300"]["t"])
    xb = float(C.st["FS0600"]["x"]) - 0.5 * float(C.st["FS0600"]["t"])
    plate = box3((xa, -0.2, z0 - TRAY_T), (xb, 0.2, z0))
    fl_a = box3((xa, -0.2, z0 - TRAY_FL), (xa + TRAY_T, 0.2, z0 - TRAY_T + OV))
    fts = C.L["rules"]["boxes"]["forward_bay_lower"]
    hw = float(fts["half_width"]) + float(fts["clearance"])
    fl_b = [box3((xb - TRAY_T, s * hw if s > 0 else -0.2, z0 - TRAY_FL),
                 (xb, 0.2 if s > 0 else -hw, z0 - TRAY_T + OV)) for s in (1, -1)]
    m = union([plate, fl_a] + fl_b)
    return pieces_above(inter(m, C.body_env(MEM_IN, xa - 0.01, xb + 0.01)), 1e-7)


def fwdbay_tray_points(C: Ctx):
    bb = C.L["rules"]["boxes"]["forward_bay"]
    z0 = float(bb["z"][0])
    xa = float(C.st["FS0300"]["x"]) + 0.5 * float(C.st["FS0300"]["t"])
    xb = float(C.st["FS0600"]["x"]) - 0.5 * float(C.st["FS0600"]["t"])
    zb = z0 - 0.5 * (TRAY_FL + TRAY_T)
    return [(xa + TRAY_T, 0.0, zb)], [(xb - TRAY_T, s * 0.058, zb) for s in (1, -1)]


def build_mission_tray(C: Ctx) -> G.Mesh:
    """Removable mission-bay tray TR-MISSION (CFRP 3 mm): plug in the floor cut-out, top flush with the floor top,
    resting on the inner flanges of the two edge hats (8 x M4 captive screws from below into nutplates on the tray)."""
    m = C.mem["M-MIDFLOOR"]
    lo, hi = m["box"]
    co = m["cutout"]
    ye = float(co["y"][1]) - MID_HAT_T - 0.0003 - 0.0005
    return box3((float(co["x"][0]) + 0.0005, -ye, float(hi[2]) - MID_TRAY_T), (float(co["x"][1]) - 0.0005, ye,
                                                                            float(hi[2])))


def mission_tray_points(C: Ctx):
    m = C.mem["M-MIDFLOOR"]
    co = m["cutout"]
    z = float(m["box"][0][2]) + MID_TRAY_DROP - MID_HAT_T
    xs = np.linspace(float(co["x"][0]) + 0.030, float(co["x"][1]) - 0.030, 4)
    yy = float(co["y"][1]) - MID_HAT_T - 0.0003 - 0.0125
    return [(float(x), s * yy, z) for s in (1, -1) for x in xs]


AFTBAY_T, AFTBAY_R = 0.002, 0.006       # TR-AFTBAY 6061-T6 2 mm, bend radius 3 t (processes.sheet_metal_aluminium)


def build_aftbay_tray(C: Ctx) -> G.Mesh:
    """Aft equipment-bay tray TR-AFTBAY (6061-T6 2 mm): floor of the bay at its lower bound from FS-GEAR to FS3480,
    flanges bent down (r 6 mm) onto both frames (M4 into potted inserts)."""
    bb = C.L["rules"]["boxes"]["equipment_bay_aft"]
    z0 = float(bb["z"][0])
    hw = float(bb["half_width"])
    xa = float(C.st["FS-GEAR"]["x"]) + 0.5 * float(C.st["FS-GEAR"]["t"])
    xb = float(C.st["FS3480"]["x"]) - 0.5 * float(C.st["FS3480"]["t"])
    La = 0.5 * (xb - xa) + 0.001
    sec_a = bent_l(La, AFTBAY_FL_A, AFTBAY_T, AFTBAY_R)        # (u = +x from xa, v = -z from z0)
    sec_b = bent_l(La, AFTBAY_FL_B, AFTBAY_T, AFTBAY_R)        # (u = -x from xb, v = -z from z0)
    ma = extrude_cs(sec_a, 2 * hw, (xa, -hw, z0), (1.0, 0.0, 0.0), (0.0, 0.0, -1.0))      # u x v = +y
    mb = extrude_cs(sec_b, 2 * hw, (xb, hw, z0), (-1.0, 0.0, 0.0), (0.0, 0.0, -1.0))      # u x v = -y
    return union([ma, mb])


AFTBAY_FL_A, AFTBAY_FL_B = 0.032, 0.040


def aftbay_tray_points(C: Ctx):
    bb = C.L["rules"]["boxes"]["equipment_bay_aft"]
    z0 = float(bb["z"][0])
    xa = float(C.st["FS-GEAR"]["x"]) + 0.5 * float(C.st["FS-GEAR"]["t"])
    xb = float(C.st["FS3480"]["x"]) - 0.5 * float(C.st["FS3480"]["t"])
    za = z0 - AFTBAY_FL_A + 0.010
    zb = z0 - AFTBAY_FL_B + 0.010
    return [(xa + AFTBAY_T, s * 0.060, za) for s in (1, -1)], [(xb - AFTBAY_T, s * 0.050, zb) for s in (1, -1)]


PB_T, PB_FOOT_T, PB_W, PB_FOOT_H, PB_WIN = 0.003, 0.0025, 0.024, 0.020, 0.025


def build_para_bracket(C: Ctx, xc: float, zc: float) -> G.Mesh:
    """Parachute-container strap bracket (7075, starboard wall): two feet on the bay wall (1 x M5 each, through the
    wall) and a bridge 3 mm standing 2.5 mm off the wall (strap passage), all inside the 6 mm gap between the
    container envelope and the wall."""
    yw = float(C.mem["M-PARAWALL"]["box"][0][1])                 # wall inner face
    yb = yw - PB_FOOT_T
    hz = 0.5 * PB_WIN + PB_FOOT_H
    bridge = box3((xc - 0.5 * PB_W, yb - PB_T, zc - hz), (xc + 0.5 * PB_W, yb + OV, zc + hz))
    feet = [box3((xc - 0.5 * PB_W, yb, z0), (xc + 0.5 * PB_W, yw, z1))
            for z0, z1 in ((zc - hz, zc - 0.5 * PB_WIN), (zc + 0.5 * PB_WIN, zc + hz))]
    return union([bridge] + feet)


PARA_BRKT_X = (1.560, 1.740)
PARA_BRKT_Z = -0.080


def para_bracket_points(C: Ctx, xc: float, zc: float):
    yw = float(C.mem["M-PARAWALL"]["box"][0][1])
    hz = 0.5 * PB_WIN + PB_FOOT_H
    return [(xc, yw - PB_FOOT_T - PB_T, zc + s * (hz - 0.5 * PB_FOOT_H)) for s in (1, -1)]


WASHER_T = 0.003


def build_washer_plate(C: Ctx, fid: str) -> G.Mesh:
    """7075 washer plate 3 mm under the spine-channel floor pad of a bridle fitting (layout F-RISER-*: 48 x 40 x 3 mm),
    kept forward of the saddle-bay liner wall."""
    f = C.fit[fid]
    bp = [b["point"] for b in f["bolts"] if b["group"] == "spine floor"]
    xs = [p[0] for p in bp]
    xc = 0.5 * (min(xs) + max(xs))
    z_top = SPINE_FLOOR_TOP - float(C.mem["M-SPINE"]["section"]["pad_t"])
    x0, x1 = xc - 0.024, xc + 0.024
    if fid == "F-RISER-AFT":
        xr = float(C.st["FS-RS"]["x"]) - 0.5 * float(C.st["FS-RS"]["t"]) - LINER_T - 0.0005
        x1 = min(x1, xr)
    return box3((x0, -0.021, z_top - WASHER_T), (x1, 0.021, z_top))


def build_rail_anchor(C: Ctx, rail: dict) -> G.Mesh:
    """Turret-elevator rail anchor (7075 machined corner angle, rail end on the frame): leg on the frame face, leg on
    the bay-wall inner face with the 5.5 mm rail seat; the MGN9-class rail (payload module) is bonded and screwed
    (M3 into potted inserts) to the seat. ``rail`` = layout.chassis.turret_elevator.rails[i]."""
    (x_r, y_r, z_a), (_, _, z_b) = rail["line"]
    sy = 1.0 if y_r > 0 else -1.0
    fr = "FS1110" if rail["corner"].startswith("F") else "FS1330"
    sx = 1.0 if fr == "FS1110" else -1.0                         # +1: anchor aft of FS1110, -1: forward of FS1330
    xf = float(C.st[fr]["x"]) + sx * 0.5 * float(C.st[fr]["t"])
    yw = float(C.mem["M-TURRETWALL"]["box"][0][1])               # wall inner face (|y|)
    z0, z1 = min(z_a, z_b), max(z_a, z_b)
    seat = abs(y_r) + RAIL_HALF_H                                # rail base plane (|y|)
    xr_far = x_r + sx * (RAIL_SEAT_X)
    leg_f = box3((min(xf, xf + sx * RAIL_ANCHOR_T), sy * (yw - RAIL_ANCHOR_LEG) if sy > 0 else -yw,
                  z0), (max(xf, xf + sx * RAIL_ANCHOR_T), yw if sy > 0 else -(yw - RAIL_ANCHOR_LEG), z1))
    xa_, xb_ = sorted((xf, xr_far))
    ya_, yb_ = sorted((sy * seat, sy * yw))
    leg_w = box3((xa_, ya_, z0), (xb_, yb_, z1))
    m = union([leg_f, leg_w])
    return pieces_above(inter(m, C.body_env(MEM_IN, xf - 0.06, xf + 0.06)))


RAIL_HALF_H = 0.00325       # half the rail height (MGN9 class 6.5 mm, payload module estimate)
RAIL_SEAT_X = 0.028         # seat leg runs 28 mm beyond the rail centre line (bolt heads clear of the carriage)
RAIL_ANCHOR_T = 0.003
RAIL_ANCHOR_LEG = 0.025


def rail_anchor_points(C: Ctx, rail: dict):
    """(frame-leg bolts axis x, wall-leg bolts axis y) at the anchor's lower / upper ends."""
    (x_r, y_r, z_a), (_, _, z_b) = rail["line"]
    sy = 1.0 if y_r > 0 else -1.0
    fr = "FS1110" if rail["corner"].startswith("F") else "FS1330"
    sx = 1.0 if fr == "FS1110" else -1.0
    xf = float(C.st[fr]["x"]) + sx * 0.5 * float(C.st[fr]["t"])
    yw = float(C.mem["M-TURRETWALL"]["box"][0][1])
    z0, z1 = min(z_a, z_b), max(z_a, z_b)
    zs = (z0 + 0.020, z1 - 0.020)
    fb = [(xf + sx * RAIL_ANCHOR_T, sy * (yw - 0.0125), z) for z in zs]
    seat = abs(y_r) + RAIL_HALF_H
    xm = x_r + sx * (RAIL_SEAT_X - 0.012)
    wb = [(xm, sy * seat, z) for z in (z0 + 0.060, z1 - 0.060)]
    return fb, wb


LINER_T = 0.0006            # fuel-bay liner: 3 plies PW (layout 2 plies 0.4 mm < processes.prepreg_ooa_vacbag 0.6 mm)


def build_liner(C: Ctx, fs: dict, z_floor: float, cut_meshes, gap: float = 0.0002) -> G.Mesh:
    """Fuel-bay liner tub (CFRP 3 plies PW, open top) of one cell bay: floor on the supporting member (its top face
    ``z_floor``), end walls on the forward / aft frame faces (bonded), side walls on the cell boundary (OML inset
    ``inset_from_oml_m``), top at the cell top z1; cut round the structure crossing the bay (``cut_meshes``) and through
    the frame cut-outs."""
    z1 = float(fs["z"][1])
    ins = float(fs["inset_from_oml_m"])
    t = LINER_T
    stf, sta = C.st[fs["boundary_fwd"]["station"]], C.st[fs["boundary_aft"]["station"]]
    xa = float(stf["x"]) + 0.5 * float(stf["t"])
    xb = float(sta["x"]) - 0.5 * float(sta["t"])
    ka = math.tan(math.radians(float(stf.get("sweep_deg", 0.0))))
    kb = math.tan(math.radians(float(sta.get("sweep_deg", 0.0))))
    yl = 0.6

    def wedge(xA, xB, zlo, zhi):
        half = []
        for s_ in (1, -1):
            V = []
            for y in (0.0, yl):
                V += [(xA + y * ka, s_ * y, zlo), (xB + y * kb, s_ * y, zlo), (xB + y * kb, s_ * y, zhi),
                      (xA + y * ka, s_ * y, zhi)]
            half.append(G.hull(np.asarray(V)))
        return G.union(half)
    xa, xb = xa + gap, xb - gap                    # bond line to the frame faces
    outer = inter(wedge(xa, xb, z_floor, z1), C.body_env(ins - t, xa - 0.01, xb + 0.11))
    inner = inter(wedge(xa + t, xb - t, z_floor + t, z1 + 0.05), C.body_env(ins, xa - 0.02, xb + 0.12))
    cuts = list(cut_meshes)
    for st_, x_face, k, sgn in ((stf, xa, ka, 1.0), (sta, xb, kb, -1.0)):
        for c in _cut_polys(st_):
            x0_, x1_ = (x_face - 0.002, x_face + t + 0.002) if sgn > 0 else (x_face - t - 0.002, x_face + 0.002)
            pr = prism_x(c, x0_, x1_)
            if k:
                cy = 0.5 * (c.bounds[0] + c.bounds[2])
                pr = shear_x(pr, k if cy > 0 else -k)
            cuts.append(pr)
    tub = diff(outer, [inner] + cuts)
    return pieces_above(tub, 1e-7)


# =====================================================================================================================
# registration
# =====================================================================================================================
MAT_PW, MAT_UD, MAT_7075, MAT_2024, MAT_6061 = ("cfrp_pw_mtm45_as4", "cfrp_ud_mtm45_as4", "al_7075_t651_plate",
                                               "al_2024_t3_sheet", "al_6061_t6_sheet")
MAT_4130, MAT_SS = "steel_4130_n", "ss_304_annealed"
P_PREG, P_CNC, P_SHEET, P_TIG, P_SS = ("prepreg_ooa_vacbag", "cnc_milling_metal", "sheet_metal_aluminium",
                                       "tig_welding_4130", "sheet_metal_steel")

# part numbers this module adds to the layout ids (inside layout.part_numbering.modules.chassis.sub_ranges)
N_CHINE_AFT, N_CHINE_END = 40, 41
N_CLIPS = {"FS1110": 42, "FS1330": 43, "FS1490": 44, "FS1810": 45, "FS-FUEL": 46, "FS-GEAR": 47, "FS3480": 48}
N_GLOVE_BOX = 58
N_SHIELD, N_SPACER, N_UPPER_PLATE, N_EMLO_BACK, N_STANDOFF = 88, 90, 92, 93, 94
N_KEEL_CLIP = 102
N_WASHER = {"F-RISER-FWD": 110, "F-RISER-AFT": 111}
N_PARA_BRKT = (114, 123)
N_RAIL = {"FR": 125, "AL": 126}
LINER_N = {"forward_cell": 115, "saddle_cell": 116, "aft_cell": 117}
TRAY_N = {"TR-SIDEBAY-L": 118, "TR-FWDBAY": 119, "TR-MISSION": 120, "TR-AFTBAY": 121, "TR-SIDEBAY-R": 122}


def _port(pid: str) -> str:
    return pid[:-2] + "-L" if pid.endswith("-R") else pid


class _Memo:
    """Mesh cache shared by parts whose geometry depends on other parts (frames for the box, the chine for the decks)."""

    def __init__(self):
        self.d = {}

    def __call__(self, key, fn, clean: bool = True):
        if key not in self.d:
            m = fn()
            self.d[key] = finish(m) if clean and isinstance(m, G.Mesh) else m
        return self.d[key]


def register(reg: Registry, spec: dict) -> None:
    """Register the chassis parts and their fasteners (ARCHITECTURE.md producer contract)."""
    L = spec["layout"]
    if L["root_part"] in reg.parts:            # already registered (e.g. checks --modules chassis,chassis)
        reg.note("chassis: already registered, second call ignored")
        return
    C = Ctx(reg, spec)
    bx = Box(C)
    M = _Memo()
    R = _Reg(C, M, bx)
    R.frames()
    R.ct_group()
    R.members()
    R.fittings()
    R.firewall()
    R.small()
    R.mirror_all()
    R.ct_mass()
    F = _Fast(C)
    _fasteners(C, R, F, bx)
    if F.problems:
        msg = "; ".join(f"{a}: {b}" for a, b in F.problems)
        if os.environ.get("YK250_CHASSIS_LENIENT"):
            reg.note("chassis fastener problems: " + msg)
        else:
            raise ValueError("chassis fasteners: " + msg)


class _Reg:
    """Part registration (starboard parts in +y, mirrored at the end)."""

    def __init__(self, C: Ctx, M: _Memo, bx: Box):
        self.C, self.M, self.bx = C, M, bx
        self.root = C.L["root_part"]
        self.right = []

    # ---------------------------------------------------------------- helpers
    def fr(self, sid):
        return self.M(("frame", sid), lambda: build_frame(self.C, sid))

    def add(self, num, side, name, name_tr, material, process, fn, **kw):
        kw.setdefault("explode", (0.0, 0.0, 0.0))
        p = self.C.add(num, side, name, name_tr, material, process, fn, **kw)
        if side == "R":
            self.right.append(p.id)
        return p

    def st_part(self, sid):
        return self.C.st[sid]["part"]

    def mirror_all(self):
        idm = {pid: _port(pid) for pid in self.right}
        for pid in self.right:
            self.C.reg.add(mirror_part(self.C.reg.parts[pid], _port(pid), id_map=idm))

    # ---------------------------------------------------------------- frames
    def frames(self):
        C, root = self.C, self.root
        for st in C.L["stations"]:
            sid = st["id"]
            if sid == "FS3738":
                continue
            num = int(st["part"].split("-")[-1])
            fwd = float(st["x"]) < 2.4
            ex = (0.0, 0.0, 0.12) if sid in ("FS-MS", "FS-RS") else (0.0, 0.0, 0.30 if fwd else 0.25)
            note = st.get("construction", "") or st.get("role", "")
            if sid == "FS3670":
                note = ("CFRP sandwich bulkhead (rib_panel) of the firewall stack, forward T-cap only; " + note)
            self.add(num, "C", f"frame {sid} ({st['type']})", f"{st['role_tr']} ({sid})", st["material"],
                     st["process"], (lambda sid=sid: self.fr(sid)), layup=st.get("layup"), parent=root,
                     step=FRAME_STEP[sid], explode=ex, contacts=(root,), notes=note[:400])
        st = C.st["FS3738"]
        seg = st["lower_segment"]
        ur = lambda: self.M(("uring",), lambda: build_uring(C), clean=False)        # noqa: E731
        self.add(14, "C", "engine-bay lower U-ring legs FS3738 (2024-T3, dash -1 starboard / -2 port)",
                 "motor bölmesi alt U halkası bacakları FS3738 (2024-T3, -1 sağ / -2 sol)", MAT_2024, P_SHEET,
                 lambda: finish(union([ur()[0], ur()[0].mirrored_y()])), thickness=float(st["t"]),
                 parent=seg["part"], step=13, explode=(0.10, 0.0, -0.10),
                 contacts=(seg["part"], C.ref("F-SPINDLE-NODE", "R"), C.ref("F-SPINDLE-NODE", "L")),
                 notes="formed flanges r 12 mm (6 t) at the skin line; lap the 7075 lower segment web (3 x M5 each "
                       "end); upper ends riveted to the node cheek feet; lower cowl Camloc land clips on the web "
                       "(shell / propulsion module)")
        self.add(15, "C", "engine-bay lower U-ring segment FS3738 (machined 7075 I-section)",
                 "motor bölmesi alt U halkası alt parçası FS3738 (talaşlı 7075 I kesit)", seg["material"],
                 seg["process"], lambda: finish(ur()[1]), thickness=float(
                     C.S["structures"]["sizing"]["body"]["fs3738_lower_segment"]["web_t_m"]),
                 parent=C.ref("M-AFTKEEL"), step=13, explode=(0.10, 0.0, -0.18), contacts=(C.ref("M-AFTKEEL"),))

    # ---------------------------------------------------------------- centre wing box group
    def ct_group(self):
        C, M, bx, root = self.C, self.M, self.bx, self.root
        fr2 = lambda: {"FS-MS": self.fr("FS-MS"), "FS-RS": self.fr("FS-RS")}             # noqa: E731
        m_ct = C.mem["M-CTBOX"]
        C.reg.add(Part(id=root, name="centre wing box (carry-through)", name_tr=m_ct["name_tr"], group=GROUP,
                       material=MAT_UD, process=P_PREG, mesh_fn=lambda: M(("ct",), lambda: build_ctbox(C, bx, fr2())),
                       thickness=C.layup_t("ct_box_cover"), step=2, explode=(0.0, 0.0, 0.0),
                       notes="one cured assembly: continuous UD spar caps (main 40 -> 52 mm at the fork, rear 25 mm) "
                             "from joint rib to joint rib, ct_box_cover sandwich covers between the spar frames, "
                             "+-45 PW glove rear-spar webs with the 16-ply slot-fitting pads, 8-ply frame-land web "
                             "doublers with nutplates; the spar frames FS-MS / FS-RS are the box webs inside the "
                             "body; mass from the sub-volumes (ctbox_mass)"))
        self.add(55, "C", "CT-box centre-line rib", C.mem["M-CLRIB"]["name_tr"], MAT_PW, P_PREG,
                 lambda: M(("clrib",), lambda: build_clrib(C, bx)), layup="rib_panel", parent=root, step=2,
                 explode=(0.0, 0.0, 0.18), contacts=(root, self.st_part("FS-MS"), self.st_part("FS-RS")))
        for fid, num in (("F-KINK-UP", 56), ("F-KINK-LO", 57)):
            self.add(num, "C", C.fit[fid]["name"], C.fit[fid]["name_tr"], MAT_7075, P_CNC,
                     (lambda fid=fid: M(("kink", fid), lambda: build_kink(C, fid))), thickness=0.002,
                     parent=C.pid(55), step=2, explode=(0.0, 0.0, 0.25 if num == 56 else -0.25),
                     contacts=(root, C.pid(55), self.st_part("FS-MS")),
                     notes="plate bonded (EA 9394) to the box-side face of the main cap over the chevron kink")
        ribs = lambda: M(("ribs",), lambda: build_glove_ribs(C, bx), clean=False)        # noqa: E731
        self.add(53, "R", "outer-panel joint fork, starboard (prongs + bonded 4130 bushes)",
                 "dış panel birleşim çatalı, sağ (kulaklar + yapıştırılmış 4130 burçlar)", MAT_PW, P_PREG,
                 lambda: M(("fork",), lambda: build_fork(C, bx)), thickness=PRONG_T, parent=root, step=5,
                 explode=(0.0, 0.25, 0.0), contacts=(root,),
                 notes="two +-45 PW prongs 1.6 mm padded to 10 mm (50 mm long) round the pin bores; 4130 bushes "
                       "16 H8 x OD 22 x 10 bonded and line-reamed with the master tongue (bores cut here); pins "
                       "P-MAIN1/2 are the wing module's")
        for key, num, nm, nmtr, ex in (("SOB", 50, "side-of-body rib", "gövde yanı kaburgası", 0.30),
                                       ("GLOVE_NOSE", 51, "glove rib, nose piece", "eldiven kaburgası, burun parçası",
                                        0.35),
                                       ("GLOVE_BOX", N_GLOVE_BOX, "glove rib, box piece",
                                        "eldiven kaburgası, kutu parçası", 0.35),
                                       ("JOINT", 52, "joint rib (centre-section side)",
                                        "birleşim kaburgası (orta kesit)", 0.40)):
            self.add(num, "R", nm + ", starboard", nmtr + ", sağ", MAT_PW, P_PREG,
                     (lambda key=key: finish(ribs()[key])), layup="rib_panel", parent=root, step=4,
                     explode=(0.0, ex, 0.0), contacts=(root, C.pid(53, "R")))
        self.add(54, "R", "rear-spar slot fitting, starboard", "arka kiriş yuva bağlantısı, sağ", MAT_7075, P_CNC,
                 lambda: M(("slot",), lambda: build_slot_fitting(C, bx)), thickness=0.004, parent=root, step=5,
                 explode=(0.0, 0.3, 0.05), contacts=(root, C.pid(52, "R")))

    def ct_mass(self):
        C = self.C
        tot, parts = ctbox_mass(C, self.bx, {"FS-MS": self.fr("FS-MS"), "FS-RS": self.fr("FS-RS")})
        p = C.reg.parts[self.root]
        p.mass_kg = float(tot)
        p.notes += " | mass split kg: " + ", ".join(f"{k} {v:.3f}" for k, v in parts.items())
        self.ct_split = parts

    # ---------------------------------------------------------------- members
    def members(self):
        C, M, root = self.C, self.M, self.root
        mem = C.mem
        stp = self.st_part

        def lay(mid):
            m = mem[mid]
            lk = m.get("layup")
            if lk and lk in C.S["layups"]:
                try:
                    layup_props(C.S, lk)
                    return {"layup": lk}
                except ValueError:
                    pass
            t = m.get("thickness") or (m.get("section") or {}).get("t")
            return {"thickness": float(t)}

        def mat(mid):
            return mem[mid]["material"], mem[mid]["process"]

        chine = lambda k: M(("chine", k), lambda: build_chine(C, k))                    # noqa: E731
        sec = mem["M-CHINE"]["section"]
        self.add(20, "R", "chine longeron, forward piece, starboard", "kenar çizgisi uzun kirişi, ön parça, sağ",
                 MAT_UD, P_PREG, lambda: chine(0), thickness=float(sec["t"]), parent=root, step=9,
                 explode=(0.0, 0.35, 0.0),
                 contacts=tuple(stp(s) for s in ("FS0600", "FS1110", "FS1330", "FS1490", "FS1810", "FS-FUEL"))
                 + (C.ref("M-SOB"), C.ref("M-DECK-NOSE"), C.ref("M-MIDFLOOR")),
                 notes="J 35 x 30, t 2.4: UD caps + +-45 PW web / skin flange (spar_cap_ud + spar_web); outboard "
                       "splice leg on the side-of-body rib (SPL-CH-FWD, 4 x M6 Ti)")
        self.add(N_CHINE_AFT, "R", "chine longeron, aft piece, starboard", "kenar çizgisi uzun kirişi, arka parça, sağ",
                 MAT_UD, P_PREG, lambda: chine(1), thickness=float(sec["t"]), parent=root, step=12,
                 explode=(0.0, 0.35, 0.0), contacts=(C.ref("M-SOB"), stp("FS-GEAR"), stp("FS3480"),
                                                    C.pid(N_CHINE_END, "R"), C.ref("M-WELLROOF")),
                 notes="J 35 x 30, t 2.4 from the side-of-body rib (SPL-CH-AFT, 4 x M6 Ti) to the firewall, ends on "
                       "its 7075 end fitting (2 x M5 Ti)")
        self.add(38, "C", "nose-gear sill", "burun takımı eşiği", *mat("M-NGSILL"),
                 lambda: M(("ngsill",), lambda: box_member(C, *mem["M-NGSILL"]["box"])), **lay("M-NGSILL"),
                 parent=C.ref("M-KEELWALL"), step=7, explode=(0.0, 0.0, -0.2),
                 contacts=(C.ref("M-KEELWALL", "R"), C.ref("M-KEELWALL", "L"), stp("FS1110")))
        self.add(21, "R", "nose keel wall, starboard", "burun omurga duvarı, sağ", *mat("M-KEELWALL"),
                 lambda: M(("keelwall",), lambda: build_keelwall(C)), **lay("M-KEELWALL"), parent=stp("FS1110"),
                 step=7, explode=(0.0, 0.2, -0.2), contacts=(stp("FS0600"), stp("FS1110"), C.ref("M-DECK-NOSE")))
        chine_env = lambda: union([chine(0), chine(0).mirrored_y()])                    # noqa: E731
        self.add(22, "C", "avionics deck (nose)", mem["M-DECK-NOSE"]["name_tr"], *mat("M-DECK-NOSE"),
                 lambda: M(("decknose",), lambda: build_deck_nose(C, chine_env())), **lay("M-DECK-NOSE"),
                 parent=stp("FS1110"), step=7, explode=(0.0, 0.0, 0.25),
                 contacts=(stp("FS0600"), stp("FS1110"), C.ref("M-KEELWALL", "R"), C.ref("M-KEELWALL", "L"),
                           C.pid(20, "R"), C.pid(20, "L")))
        self.add(23, "R", "turret-bay side wall, starboard", "taret bölmesi yan duvarı, sağ", *mat("M-TURRETWALL"),
                 lambda: M(("turretwall",), lambda: build_turretwall(C)), **lay("M-TURRETWALL"), parent=stp("FS1110"),
                 step=8, explode=(0.0, 0.2, 0.0), contacts=(stp("FS1110"), stp("FS1330"), C.ref("M-TURRETROOF")))
        self.add(24, "C", "turret-bay roof", mem["M-TURRETROOF"]["name_tr"], *mat("M-TURRETROOF"),
                 lambda: M(("turretroof",), lambda: build_turretroof(C)), **lay("M-TURRETROOF"), parent=stp("FS1110"),
                 step=8, explode=(0.0, 0.0, 0.25),
                 contacts=(stp("FS1110"), stp("FS1330"), C.ref("M-TURRETWALL", "R"), C.ref("M-TURRETWALL", "L")))
        s1490, s1810 = C.st["FS1490"], C.st["FS1810"]
        self.add(25, "R", "parachute-bay side wall, starboard", "paraşüt bölmesi yan duvarı, sağ", *mat("M-PARAWALL"),
                 lambda: M(("parawall",), lambda: build_parawall(C, float(s1490["x"]), float(s1810["x"]),
                                                                 float(s1490.get("flange_w", 0.028)))),
                 **lay("M-PARAWALL"), parent=stp("FS1490"), step=8, explode=(0.0, 0.2, 0.0),
                 contacts=(stp("FS1490"), stp("FS1810"), C.ref("M-PARAFLOOR")))
        self.add(26, "C", "parachute-bay floor", mem["M-PARAFLOOR"]["name_tr"], *mat("M-PARAFLOOR"),
                 lambda: M(("parafloor",), lambda: box3(*mem["M-PARAFLOOR"]["box"])), **lay("M-PARAFLOOR"),
                 parent=stp("FS1490"), step=8, explode=(0.0, 0.0, -0.25),
                 contacts=(stp("FS1490"), stp("FS1810"), C.ref("M-PARAWALL", "R"), C.ref("M-PARAWALL", "L")))
        self.add(27, "C", "mission-bay floor with cut-out edge hats", mem["M-MIDFLOOR"]["name_tr"],
                 *mat("M-MIDFLOOR"), lambda: M(("midfloor",), lambda: build_midfloor(C)), **lay("M-MIDFLOOR"),
                 parent=stp("FS1810"), step=8, explode=(0.0, 0.0, -0.25),
                 contacts=(stp("FS1810"), stp("FS-FUEL"), C.pid(20, "R"), C.pid(20, "L")),
                 notes="outer sandwich strips + two bonded hat stiffeners (PW 4 plies, 20 x 15) along the cut-out "
                       "edges; inner hat flanges carry TR-MISSION (8 x M4)")
        frames3 = lambda: {s: self.fr(s) for s in ("FS-FUEL", "FS-MS", "FS-RS")}         # noqa: E731
        self.add(28, "R", "payload-bay keel beam, starboard", "yük bölmesi omurga kirişi, sağ", *mat("M-KEEL"),
                 lambda: M(("keel",), lambda: build_keel(C, frames3())), **lay("M-KEEL"), parent=root, step=10,
                 explode=(0.0, 0.15, -0.25),
                 contacts=(stp("FS-FUEL"), stp("FS-MS"), stp("FS-RS"), root, C.ref("M-FWDDECK"), C.ref("M-WELLROOF")))
        self.add(29, "C", "forward fuel deck", mem["M-FWDDECK"]["name_tr"], *mat("M-FWDDECK"),
                 lambda: M(("fwddeck",), lambda: build_fwddeck(C, frames3())), **lay("M-FWDDECK"), parent=root,
                 step=10, explode=(0.0, 0.0, -0.3),
                 contacts=(stp("FS-FUEL"), stp("FS-MS"), C.ref("M-KEEL", "R"), C.ref("M-KEEL", "L"), root))
        self.add(30, "C", "main-gear well roof (aft-cell floor)", mem["M-WELLROOF"]["name_tr"], *mat("M-WELLROOF"),
                 lambda: M(("wellroof",), lambda: build_wellroof(C)), **lay("M-WELLROOF"), parent=root, step=10,
                 explode=(0.0, 0.0, -0.3), contacts=(stp("FS-RS"), stp("FS-GEAR"), C.ref("M-GEARBEAM", "R"),
                                                     C.ref("M-GEARBEAM", "L"), root))
        self.add(31, "R", "main-gear beam (outboard well wall), starboard", "ana takım kirişi, sağ",
                 *mat("M-GEARBEAM"), lambda: M(("gearbeam",), lambda: build_gearbeam(C)), **lay("M-GEARBEAM"),
                 parent=C.ref("M-WELLROOF"), step=10, explode=(0.0, 0.25, -0.2),
                 contacts=(stp("FS-RS"), stp("FS-GEAR"), C.ref("M-WELLROOF")),
                 notes="16-ply solid land 1.5 mm proud of the inboard face under the trunnion flange, bonded "
                       "through-thickness M6 inserts (flush with the outboard face)")
        fin_x = (float(C.fit["F-FIN-FRONT"]["box"][0][0]), float(C.fit["F-FIN-FRONT"]["box"][1][0]))
        self.add(32, "R", "dorsal longeron, starboard", "sırt uzun kirişi, sağ", MAT_UD, P_PREG,
                 lambda: M(("dorsal",), lambda: build_dorsal(C, fin_x)), thickness=DORSAL_T, parent=stp("FS3480"),
                 step=11, explode=(0.0, 0.1, 0.25), contacts=(stp("FS-GEAR"), stp("FS3480"), C.pid(N_UPPER_PLATE, "R"),
                                                             C.ref("F-FIN-FRONT")),
                 notes="hat 25 x 20, t 2.0 (UD crown, +-45 walls), skin flanges 12 mm; crown and walls relieved over "
                       "the fin front-spar fitting; ends ahead of the firewall on the splice tongue (2 x M5 Ti)")
        self.add(34, "C", "payload-bay aft wall", mem["M-PAYWALL-AFT"]["name_tr"], *mat("M-PAYWALL-AFT"),
                 lambda: M(("paywall",), lambda: build_plain_wall(C, "M-PAYWALL-AFT", -0.0668)),
                 **lay("M-PAYWALL-AFT"), parent=C.ref("M-KEEL"), step=10, explode=(0.0, 0.0, -0.3),
                 contacts=(C.ref("M-KEEL", "R"), C.ref("M-KEEL", "L"), stp("FS-RS"), C.ref("M-WELLROOF")))
        self.add(35, "C", "main-gear well forward wall", mem["M-WELLWALL-FWD"]["name_tr"], *mat("M-WELLWALL-FWD"),
                 lambda: M(("wellwall",), lambda: build_plain_wall(C, "M-WELLWALL-FWD")), **lay("M-WELLWALL-FWD"),
                 parent=C.ref("M-WELLROOF"), step=10, explode=(0.0, 0.0, -0.3),
                 contacts=(C.ref("M-WELLROOF"), C.ref("M-GEARBEAM", "R"), C.ref("M-GEARBEAM", "L")))
        self.add(36, "R", "main-gear well centre web, starboard", "ana takım kuyusu orta duvarı, sağ",
                 *mat("M-WELLKEEL"), lambda: M(("wellkeel",), lambda: build_wellkeel(C)), **lay("M-WELLKEEL"),
                 parent=C.ref("M-WELLROOF"), step=10, explode=(0.0, 0.05, -0.3),
                 contacts=(C.ref("M-WELLROOF"), stp("FS-GEAR"), C.ref("M-WELLWALL-FWD")))
        self.add(33, "C", "aft keel beam (engine bay)", mem["M-AFTKEEL"]["name_tr"], *mat("M-AFTKEEL"),
                 lambda: M(("aftkeel",), lambda: build_aftkeel(C)), thickness=float(mem["M-AFTKEEL"]["thickness"]),
                 parent=stp("FS3670"), step=11, explode=(0.0, 0.0, -0.3), contacts=(stp("FS3670"),))
        self.add(37, "C", "dorsal spine channel (parachute bridle tie)", mem["M-SPINE"]["name_tr"], *mat("M-SPINE"),
                 lambda: M(("spine",), lambda: build_spine(C)), thickness=float(mem["M-SPINE"]["thickness"]),
                 parent=stp("FS1810"), step=14, explode=(0.0, 0.0, 0.35),
                 contacts=(stp("FS1810"), stp("FS-FUEL"), stp("FS-MS"), stp("FS-RS")),
                 notes="U 44 x 24, 4 plies PW, 60 mm skin-line flanges, 16-ply floor pads under the bridle fittings; "
                       "sealed trough through the fuel-bay frames")
        vk = mem["M-VENTRALKEEL"]
        self.add(39, "C", "ventral keel strip", vk["name_tr"], *mat("M-VENTRALKEEL"),
                 lambda: M(("ventralkeel",), lambda: build_ventralkeel(C)), thickness=float(vk["section"]["t"]),
                 parent=stp("FS3480"), step=11, explode=(0.0, 0.0, -0.3), contacts=(stp("FS3480"), stp("FS3670")))

    # ---------------------------------------------------------------- fittings
    def fittings(self):
        C, M = self.C, self.M
        stp = self.st_part
        fit = C.fit

        def nm(fid):
            f = fit[fid]
            return f.get("name", fid), f.get("name_tr", fid)

        n, ntr = nm("F-TRUNNION")
        self.add(70, "R", n + ", starboard", ntr + ", sağ", MAT_7075, P_CNC,
                 lambda: M(("trunnion",), lambda: build_trunnion(C)), thickness=TRUN_FLANGE_T,
                 parent=C.ref("M-GEARBEAM"), step=10, explode=(0.0, -0.12, -0.2),
                 contacts=(C.ref("M-GEARBEAM"), C.ref("M-WELLROOF")),
                 notes="bushings 20 H7 line-reamed in the jig; roof bolts on 29 mm rows (dome nutplates 28 mm)")
        n, ntr = nm("F-UPLOCK")
        self.add(72, "R", n + ", starboard", ntr + ", sağ", MAT_7075, P_CNC,
                 lambda: M(("uplock",), lambda: build_uplock(C)), thickness=UPLOCK_BASE_T,
                 parent=C.ref("M-WELLROOF"), step=10, explode=(0.0, 0.0, -0.2), contacts=(C.ref("M-WELLROOF"),))
        n, ntr = nm("F-NG-PIVOT")
        self.add(71, "C", n + " (pair of blocks)", ntr + " (blok çifti)", MAT_7075, P_CNC,
                 lambda: M(("ngpivot",), lambda: build_ng_pivot(C)), thickness=0.006,
                 parent=C.ref("M-KEELWALL"), step=7, explode=(0.0, 0.0, -0.2),
                 contacts=(C.ref("M-KEELWALL", "R"), C.ref("M-KEELWALL", "L")),
                 notes="two 7075 blocks on the inboard faces of the keel walls, flanged bushings 16 H7 (OD 22) "
                       "bonded and line-reamed")
        n, ntr = nm("F-SPINDLE-NODE")
        self.add(95, "R", n + ", starboard", ntr + ", sağ", MAT_7075, P_CNC,
                 lambda: M(("node",), lambda: build_spindle_node(C)), thickness=0.007, parent=stp("FS3670"),
                 step=13, explode=(0.2, 0.1, 0.1), contacts=(C.pid(N_SHIELD, "C", "propulsion"),),
                 notes="61805-ZZ bearing seat 37 H7 in the inboard boss (bearings and Ti spindle: tail module)")
        n, ntr = nm("F-FW-CORNER")
        self.add(86, "R", n + " (aft part), starboard", ntr + " (arka parça), sağ", MAT_7075, P_CNC,
                 lambda: M(("corner",), lambda: build_fw_corner(C)), thickness=0.006, parent=stp("FS3670"),
                 step=12, explode=(0.2, 0.0, 0.15), contacts=(C.pid(N_SHIELD, "C", "propulsion"),),
                 notes="engine-mount upper foot pad (2 x M8 through the stack), fin rear-spar clevis (tail module "
                       "lug, 2 x M6 double shear)")
        self.add(N_UPPER_PLATE, "R", "firewall upper backing plate (corner fwd plate, node backing, dorsal splice), "
                 "starboard", "yangın perdesi üst destek plakası (köşe ön plakası, düğüm desteği, sırt eki), sağ",
                 MAT_7075, P_CNC, lambda: M(("upper",), lambda: build_fw_upper_plate(C)), thickness=0.005,
                 parent=stp("FS3670"), step=12, explode=(-0.2, 0.0, 0.15), contacts=(stp("FS3670"),),
                 notes="one machined plate per side on the sandwich forward face: engine-foot M8 x 2, node bolts "
                       "B1-B3, dorsal-longeron splice tongue under the hat crown (2 x M5 Ti)")
        n, ntr = nm("F-EMOUNT-LO")
        self.add(87, "R", n + " (foot pad), starboard", ntr + " (ayak pedi), sağ", MAT_7075, P_CNC,
                 lambda: M(("emlo",), lambda: build_emount_lo(C)[0]), thickness=0.010, parent=stp("FS3670"),
                 step=12, explode=(0.2, 0.0, -0.1), contacts=(C.pid(N_SHIELD, "C", "propulsion"),))
        self.add(N_EMLO_BACK, "R", "lower engine-mount backing plate, starboard",
                 "alt motor bağlantısı destek plakası, sağ", MAT_7075, P_CNC,
                 lambda: M(("emlob",), lambda: build_emount_lo(C)[1]), thickness=0.005, parent=stp("FS3670"),
                 step=12, explode=(-0.2, 0.0, -0.1), contacts=(stp("FS3670"),))
        n, ntr = nm("F-FIN-FRONT")
        self.add(96, "R", n + ", starboard", ntr + ", sağ", MAT_7075, P_CNC,
                 lambda: M(("finfront",), lambda: build_fin_front(C)), thickness=0.006, parent=stp("FS3480"),
                 step=11, explode=(0.15, 0.0, 0.2), contacts=(stp("FS3480"),))
        n, ntr = nm("F-STUB-FRONT")
        self.add(98, "R", n + ", starboard", ntr + ", sağ", MAT_7075, P_CNC,
                 lambda: M(("stubfront",), lambda: build_stub_front(C)), thickness=0.004, parent=stp("FS3480"),
                 step=11, explode=(0.15, 0.1, 0.1), contacts=(stp("FS3480"),))
        k = C.mem["M-AFTKEEL"]
        yi = 0.5 * float(k["section"]["w"]) - float(k["section"]["t"])
        for fid in ("F-VENTRAL-1", "F-VENTRAL-2", "F-VENTRAL-3"):
            n, ntr = nm(fid)
            num = int(fit[fid]["part"].split("-")[-1])
            self.add(num, "C", n, ntr, MAT_7075, P_CNC,
                     (lambda fid=fid: M(("ventral", fid), lambda: build_ventral(C, fid, yi))), thickness=0.005,
                     parent=C.ref("M-AFTKEEL"), step=11, explode=(0.0, 0.0, -0.2), contacts=(C.ref("M-AFTKEEL"),))
        for fid in ("F-RISER-FWD", "F-RISER-AFT"):
            n, ntr = nm(fid)
            num = int(fit[fid]["part"].split("-")[-1])
            self.add(num, "C", n, ntr, MAT_7075, P_CNC, (lambda fid=fid: M(("riser", fid), lambda: build_riser(C, fid))),
                     thickness=0.005, parent=C.ref("M-SPINE"), step=14, explode=(0.0, 0.0, 0.3),
                     contacts=(C.ref("M-SPINE"), stp("FS1810" if fid.endswith("FWD") else "FS-RS")))

    # ---------------------------------------------------------------- firewall stack, engine mount
    def firewall(self):
        C, M = self.C, self.M
        stp = self.st_part
        eng = C.L["chassis"]["engine_mount"]
        foot_x = float(C.st["FS3670"]["x"]) + 0.010
        self.add(85, "C", "engine mount (welded 4130 tube bed mount)", "motor yatağı (kaynaklı 4130 boru)", MAT_4130,
                 P_TIG, lambda: M(("emount",), lambda: build_engine_mount(C, foot_x)),
                 thickness=float(eng["section"]["strut_tube_t_m"]), parent=stp("FS3670"), step=12,
                 explode=(0.3, 0.0, 0.0), contacts=(C.pid(86, "R"), C.pid(86, "L"), C.pid(87, "R"), C.pid(87, "L")),
                 notes="struts 12.7 x 0.89, ring 15.9 x 0.89, four 4 mm foot plates 56 x 32 (2 x M8 each), four "
                       "isolator cups ID 41 (Limbach damper shock mounts, engine.yaml estimate envelope); normalised "
                       "after welding")
        st = C.st["FS3670"]
        cut_yz = _cut_polys(st)
        self.add(N_SHIELD, "C", "firewall heat shield (AISI 304, 0.4 mm, with edge angle)",
                 "yangın perdesi ısı kalkanı (AISI 304, 0,4 mm, kenar köşebendi ile)", MAT_SS, P_SS,
                 lambda: M(("shield",), lambda: build_shield(C, cut_yz)), thickness=float(st["shield_t"]),
                 group="propulsion", parent=stp("FS3670"), step=12, explode=(0.15, 0.0, 0.0),
                 contacts=(C.pid(N_STANDOFF, "C", "propulsion"),),
                 notes="fireproof without test (>= 0.38 mm, standards FIRE-001, CS-VLA 1191); riveted to the 0.8 mm "
                       "edge angle (13 mm aft flange = cowl land); mass booked in the propulsion cooling/firewall item")
        self.add(N_SPACER, "R", "firewall spacer tubes (AISI 304), starboard", "yangın perdesi ara boruları, sağ", MAT_SS,
                 P_CNC, lambda: M(("spacers",), lambda: build_spacers(C, self._spacer_pts())), thickness=0.002,
                 parent=stp("FS3670"), step=12, explode=(0.1, 0.0, 0.0),
                 contacts=(stp("FS3670"), C.pid(N_SHIELD, "C", "propulsion")),
                 notes="one tube per through-bolt of the stack in the 7.6 mm air gap (stack held at its design gap)")
        self.add(N_STANDOFF, "C", "firewall shield stand-offs (12 x AISI 304)",
                 "yangın perdesi kalkan mesafe parçaları (12 x AISI 304)", MAT_SS, "purchased",
                 lambda: M(("standoffs",), lambda: build_standoffs(C, self._fw_keepouts())[0]), group="propulsion",
                 purchased=True, vendor="stainless stand-off d 10 x 7.6 mm with blind rivet (estimate)",
                 mass_kg=float(st.get("standoff_kg", 0.0)) or 0.048, parent=stp("FS3670"), step=12,
                 explode=(0.1, 0.0, 0.0), contacts=(stp("FS3670"), C.pid(N_SHIELD, "C", "propulsion")),
                 notes="layout.chassis.engine_mount.firewall_stackup.mass.standoffs_kg (12 x 4 g, estimate)")
        self.add(N_CHINE_END, "R", "chine-longeron end fitting (firewall), starboard",
                 "kenar çizgisi uzun kirişi uç bağlantısı (yangın perdesi), sağ", MAT_7075, P_CNC,
                 lambda: M(("chineend",), lambda: build_chine_end(C)), thickness=CHE_T, parent=stp("FS3670"), step=12,
                 explode=(-0.15, 0.1, 0.0), contacts=(stp("FS3670"), C.pid(N_CHINE_AFT, "R")))

    def _spacer_pts(self):
        """Starboard through-bolts of the firewall stack: node B1-B7 (M5), engine feet (M8)."""
        out = [(p[1], p[2], 5) for p in node_firewall_points(self.C)]
        out += [(p[1], p[2], 8) for p in engine_foot_points(self.C)]
        return out

    def _fw_keepouts(self):
        C = self.C
        st = C.st["FS3670"]
        polys = list(_cut_polys(st))
        for y, z, size in self._spacer_pts():
            polys.append(Point(y, z).buffer(0.012))
            polys.append(Point(-y, z).buffer(0.012))
        for fid in ("F-SPINDLE-NODE", "F-FW-CORNER", "F-EMOUNT-LO"):
            lo, hi = C.fit[fid]["box"]
            polys += [rect(lo[1], hi[1], lo[2], hi[2]), rect(-lo[1], -hi[1], lo[2], hi[2])]
        lo, hi = C.mem["M-AFTKEEL"]["box"]
        polys.append(rect(-0.06, 0.06, lo[2] - 0.05, hi[2] + 0.01))
        return polys

    # ---------------------------------------------------------------- small parts
    def small(self):
        C, M = self.C, self.M
        stp = self.st_part
        # shear clips at the chine-longeron notches
        for sid, num in N_CLIPS.items():
            side = CLIP_SIDE[sid]
            self.add(num, "R", f"chine shear clip at {sid}, starboard", f"kenar çizgisi kesme köşebendi {sid}, sağ",
                     MAT_7075, P_CNC, (lambda sid=sid, side=side: M(("clip", sid), lambda: build_clip(C, sid, side)[0])),
                     thickness=CLIP_T, parent=stp(sid), step=9 if float(C.st[sid]["x"]) < 2.6 else 11,
                     explode=(0.0, 0.15, 0.05), contacts=(stp(sid), C.pid(20 if float(C.st[sid]["x"]) < 2.6
                                                                          else N_CHINE_AFT, "R")))
        self.add(N_KEEL_CLIP, "R", "aft keel / U-ring segment clip, starboard",
                 "arka omurga / U halkası alt parça köşebendi, sağ", MAT_7075, P_CNC,
                 lambda: M(("kclip",), lambda: build_keel_clip(C)), thickness=KCLIP_T, parent=C.ref("M-AFTKEEL"),
                 step=13, explode=(0.0, 0.08, -0.1), contacts=(C.ref("M-AFTKEEL"), C.pid(15)))
        # fuel-bay liners
        fs_list = C.L["chassis"]["fuel_supports"]
        for fs in fs_list:
            num = LINER_N[fs["cell"]]
            self.add(num, "C", f"fuel-bay liner, {fs['cell'].replace('_', ' ')}",
                     {"forward_cell": "ön yakıt bölmesi astarı", "saddle_cell": "eyer yakıt bölmesi astarı",
                      "aft_cell": "arka yakıt bölmesi astarı"}[fs["cell"]], MAT_PW, P_PREG,
                     (lambda fs=fs: M(("liner", fs["cell"]), lambda: self._liner(fs))), thickness=LINER_T,
                     parent=stp(fs["boundary_fwd"]["station"]), step=14, explode=(0.0, 0.0, 0.4),
                     contacts=(stp(fs["boundary_fwd"]["station"]), stp(fs["boundary_aft"]["station"]),
                               self._liner_floor_part(fs)),
                     notes="3 plies PW 0.6 mm (layout 2 plies 0.4 mm < prepreg minimum 0.6 mm), bonded to the boundary "
                           "frames and the bay floor; cell hung on 6 loop tabs + 2 straps (fuel module)")
        # equipment trays
        trays = {t["id"]: t for t in C.L["chassis"]["trays"]}
        self.add(TRAY_N["TR-SIDEBAY-R"], "C", "avionics side-bay tray, starboard (TR-SIDEBAY-R)",
                 "aviyonik yan bölme tepsisi, sağ (TR-SIDEBAY-R)", MAT_PW, P_PREG,
                 lambda: M(("tray_sbr",), lambda: build_sidebay_tray(C, 1)), thickness=TRAY_T, parent=stp("FS1110"),
                 step=14, explode=(0.0, 0.25, 0.0), contacts=(C.ref("M-KEELWALL", "R"), stp("FS1110")),
                 notes=trays["TR-SIDEBAY-R"]["spec"][:300])
        self.add(TRAY_N["TR-SIDEBAY-L"], "C", "avionics side-bay tray, port (TR-SIDEBAY-L)",
                 "aviyonik yan bölme tepsisi, sol (TR-SIDEBAY-L)", MAT_PW, P_PREG,
                 lambda: M(("tray_sbl",), lambda: build_sidebay_tray(C, -1)), thickness=TRAY_T, parent=stp("FS1110"),
                 step=14, explode=(0.0, -0.25, 0.0), contacts=(C.ref("M-KEELWALL", "L"), stp("FS1110")),
                 notes=trays["TR-SIDEBAY-L"]["spec"][:300])
        self.add(TRAY_N["TR-FWDBAY"], "C", "forward-bay tray (TR-FWDBAY)", "ön bölme tepsisi (TR-FWDBAY)", MAT_PW,
                 P_PREG, lambda: M(("tray_fwd",), lambda: build_fwdbay_tray(C)), thickness=TRAY_T,
                 parent=stp("FS0600"), step=14, explode=(0.0, 0.0, 0.3), contacts=(stp("FS0300"), stp("FS0600")),
                 notes=trays["TR-FWDBAY"]["spec"][:300])
        self.add(TRAY_N["TR-MISSION"], "C", "mission-bay removable tray (TR-MISSION)",
                 "görev bölmesi sökülebilir tepsisi (TR-MISSION)", MAT_PW, P_PREG,
                 lambda: M(("tray_mis",), lambda: build_mission_tray(C)), thickness=MID_TRAY_T,
                 parent=C.ref("M-MIDFLOOR"), step=14, explode=(0.0, 0.0, -0.35), contacts=(C.ref("M-MIDFLOOR"),),
                 notes=trays["TR-MISSION"]["spec"][:300])
        self.add(TRAY_N["TR-AFTBAY"], "C", "aft equipment-bay tray (TR-AFTBAY)", "arka teçhizat bölmesi tepsisi "
                 "(TR-AFTBAY)", MAT_6061, P_SHEET, lambda: M(("tray_aft",), lambda: build_aftbay_tray(C)),
                 thickness=AFTBAY_T, parent=stp("FS-GEAR"), step=14, explode=(0.0, 0.0, -0.3),
                 contacts=(stp("FS-GEAR"), stp("FS3480")), notes=trays["TR-AFTBAY"]["spec"][:300])
        # parachute container brackets
        for num, xc in zip(N_PARA_BRKT, PARA_BRKT_X):
            self.add(num, "R", f"parachute container strap bracket x {xc:.2f}, starboard",
                     f"paraşüt kabı kayış braketi x {xc:.2f}, sağ", MAT_7075, P_CNC,
                     (lambda xc=xc: M(("pbrkt", xc), lambda: build_para_bracket(C, xc, PARA_BRKT_Z))),
                     thickness=PB_FOOT_T, parent=C.ref("M-PARAWALL"), step=14, explode=(0.0, -0.1, 0.0),
                     contacts=(C.ref("M-PARAWALL"),),
                     notes="layout CH-114 x 4: strap passage 2.5 mm behind a 3 mm bridge inside the 6 mm container gap; "
                           "flush blind rivets (no head on the bay side)")
        # bridle washer plates
        for fid, num in N_WASHER.items():
            self.add(num, "C", f"bridle-fitting washer plate ({fid})", f"kayış bağlantısı pul plakası ({fid})",
                     MAT_7075, P_CNC, (lambda fid=fid: M(("washer", fid), lambda: build_washer_plate(C, fid))),
                     thickness=WASHER_T, parent=C.ref("M-SPINE"), step=14, explode=(0.0, 0.0, 0.2),
                     contacts=(C.ref("M-SPINE"),))
        # turret elevator rail anchors
        for rail in C.L["chassis"]["turret_elevator"]["rails"]:
            num = N_RAIL[rail["corner"]]
            fr = "FS1110" if rail["corner"].startswith("F") else "FS1330"
            side = "R" if rail["corner"].endswith("R") else "L"
            self.add(num, "C", f"turret-elevator rail anchor {rail['corner']} ({rail['id']})",
                     f"taret asansörü ray bağlantısı {rail['corner']} ({rail['id']})", MAT_7075, P_CNC,
                     (lambda rail=rail: M(("rail", rail["id"]), lambda: build_rail_anchor(C, rail))),
                     thickness=RAIL_ANCHOR_T, parent=stp(fr), step=14, explode=(0.0, 0.0, 0.2),
                     contacts=(stp(fr), C.ref("M-TURRETWALL", side)),
                     notes=f"seat of rail {rail['part']} (payload module), M3 into potted inserts of the seat")

    def _liner_floor_part(self, fs):
        return {"forward_cell": self.C.ref("M-FWDDECK"), "saddle_cell": self.root,
                "aft_cell": self.C.ref("M-WELLROOF")}[fs["cell"]]

    def _liner(self, fs):
        C, M = self.C, self.M
        mem = C.mem
        cell = fs["cell"]
        if cell == "forward_cell":
            z_floor = float(mem["M-FWDDECK"]["box"][1][2])
        elif cell == "saddle_cell":
            z_floor = self.bx.z_cov
        else:
            z_floor = float(mem["M-WELLROOF"]["box"][1][2])
        cuts = []
        m = C.mem["M-CHINE"]
        w, h = float(m["section"]["w"]), float(m["section"]["h"])
        for k in (0, 1):
            P = np.asarray(m["paths"][k], float)
            env = _path_loft(C, P, lambda x, yc, zc: rect(yc - 0.5 * w - 0.001, 1.0, zc - 0.5 * h - 0.001,
                                                          zc + 0.5 * h + 0.001), n=24)
            cuts += [env, env.mirrored_y()]
        member = {"forward_cell": "M-FWDDECK", "aft_cell": "M-WELLROOF"}.get(cell)
        if member:
            cuts += penetration_cuts(C, member, margin=0.010)
        parts = C.reg.parts
        cuts += [parts[pid_].base_mesh for pid_ in (self.root, C.pid(56), C.pid(57), self.st_part(
            fs["boundary_fwd"]["station"]), self.st_part(fs["boundary_aft"]["station"]))]
        if cell == "aft_cell":                                   # dome nutplates of the trunnion / up-lock bolts
            for b in C.fit["F-TRUNNION"]["bolts"]:
                if b["group"] == "well roof":
                    x, y, z = trunnion_roof_point(b)
                    for s in (1, -1):
                        cuts.append(box3((x - 0.008, s * y - 0.016, z_floor - 0.01), (x + 0.008, s * y + 0.016,
                                                                                    z_floor + 0.02)))
            f = C.fit["F-UPLOCK"]
            for x in (float(f["point"][0]) - UPLOCK_BOLT_DX, float(f["point"][0]) + UPLOCK_BOLT_DX):
                for y in UPLOCK_BOLT_Y:
                    for s in (1, -1):
                        cuts.append(box3((x - 0.0065, s * y - 0.0145, z_floor - 0.01), (x + 0.0065, s * y + 0.0145,
                                                                                       z_floor + 0.02)))
        return build_liner(C, fs, z_floor, cuts)


# =====================================================================================================================
# fasteners
# =====================================================================================================================
class _Fast:
    """Fastener factory: measures the clamped stack along the fastener line on the parts' base geometry (holes are cut
    later, so the order of the joints does not matter), checks the stack is solid, then calls joints.bolt / joints.pin
    (length, grip, clearance holes, owner). Starboard joints are repeated on port with ``sym``."""

    def __init__(self, C: Ctx):
        self.C = C
        self.reg = C.reg
        self._man = {}
        self.n = defaultdict(int)
        self.problems = []

    def man(self, pid):
        if pid not in self._man:
            self._man[pid] = self.reg.parts[pid].base_mesh.to_manifold()
        return self._man[pid]

    def fid(self, owner):
        self.n[owner] += 1
        return f"{owner}-B{self.n[owner]}"

    def measure(self, pids, point, a, r, search=1.5):
        from ..core.geom import ray_hits
        e1, e2 = J._perp(a)
        c = np.asarray(point, float)
        out = []
        for pid in pids:
            man = self.man(pid)
            ss, ee = [], []
            for d in (e1, -e1, e2, -e2):
                o = c + r * d
                h = ray_hits(man, o - search * a, o + search * a) - search
                if len(h) < 2:
                    continue
                pairs = [(h[i], h[i + 1]) for i in range(0, len(h) - 1, 2)]
                s_, e_ = min(pairs, key=lambda q: 0.0 if q[0] <= 0.0 <= q[1] else min(abs(q[0]), abs(q[1])))
                ss.append(s_)
                ee.append(e_)
            if not ss:
                raise ValueError(f"fastener line does not pass through {pid}")
            out.append((pid, float(np.median(ss)), float(np.median(ee))))
        return sorted(out, key=lambda q: q[1])

    def bolt(self, size, point, axis, pids, *, bridge=(), owner=None, max_gap=0.0005, label="", **kw):
        try:
            from .fastener_catalog import clearance
            a = np.asarray(axis, float)
            a = a / np.linalg.norm(a)
            c = np.asarray(point, float)
            r = 0.5 * clearance(size) * 1.6
            iv = self.measure(list(pids) + list(bridge), c, a, r)
            for (p0, _a0, e0), (p1, s1, _e1) in zip(iv, iv[1:]):
                if s1 - e0 > max_gap:
                    raise ValueError(f"gap {(s1 - e0) * 1000:.2f} mm between {p0} and {p1}")
                if s1 - e0 < -0.0003:
                    raise ValueError(f"overlap {(e0 - s1) * 1000:.2f} mm between {p0} and {p1}")
            head = c + iv[0][1] * a
            stack = []
            for pid, s_, e_ in iv:
                if pid in bridge:
                    if not stack:
                        raise ValueError("stack starts with a bridging part")
                    stack[-1] = (stack[-1][0], stack[-1][1] + (e_ - s_))
                else:
                    stack.append((pid, e_ - s_))
            total = iv[-1][2] - iv[0][1]
            stack[-1] = (stack[-1][0], stack[-1][1] + (total - sum(t for _p, t in stack)))
            fid = self.fid(owner or stack[0][0])
            return J.bolt(self.reg, fid, size, head, a, stack, owner=owner, **kw)
        except Exception as exc:                                   # collected, raised at the end of register()
            self.problems.append((label or f"M{size} at {np.round(point, 4).tolist()}", str(exc)))
            return None

    def pin(self, d, point, axis, pids, *, owner=None, label="", **kw):
        try:
            a = np.asarray(axis, float)
            a = a / np.linalg.norm(a)
            iv = self.measure(list(pids), point, a, 0.8 * d)
            head = np.asarray(point, float) + iv[0][1] * a
            stack = [(pid, e_ - s_) for pid, s_, e_ in iv]
            fid = self.fid(owner or stack[0][0])
            return J.pin(self.reg, fid, d, head, a, stack, owner=owner, **kw)
        except Exception as exc:
            self.problems.append((label or f"pin d{d * 1000:g} at {np.round(point, 4).tolist()}", str(exc)))
            return None

    def sym(self, size, point, axis, pids, **kw):
        """Starboard joint and its port mirror image."""
        f1 = self.bolt(size, point, axis, pids, **kw)
        mp = np.array([1.0, -1.0, 1.0])
        kw2 = dict(kw)
        for k in ("owner", "insert_part", "tapped_part"):
            if kw2.get(k):
                kw2[k] = _port(kw2[k])
        if kw2.get("bridge"):
            kw2["bridge"] = tuple(_port(b) for b in kw2["bridge"])
        f2 = self.bolt(size, np.asarray(point, float) * mp, np.asarray(axis, float) * mp,
                       [_port(p) for p in pids], **kw2)
        return f1, f2

    def sym_pin(self, d, point, axis, pids, **kw):
        mp = np.array([1.0, -1.0, 1.0])
        self.pin(d, point, axis, pids, **kw)
        kw2 = dict(kw)
        if kw2.get("owner"):
            kw2["owner"] = _port(kw2["owner"])
        self.pin(d, np.asarray(point, float) * mp, np.asarray(axis, float) * mp, [_port(p) for p in pids], **kw2)


TI = "Ti-6Al-4V"            # bolt material designation (hardware.py: titanium fastener material)
G129 = "12.9"


def _fasteners(C: Ctx, R: _Reg, F: _Fast, bx: Box) -> None:
    """Every bolted joint of the chassis (layout bolt groups where the layout gives them; detail joints otherwise)."""
    st, fit = C.st, C.fit
    stp = R.st_part
    root = R.root
    pid = C.pid

    # ---- 1. kink fittings to the centre-line rib (layout F-KINK-*, 5 x M6 Ti each)
    for fid, num in (("F-KINK-UP", 56), ("F-KINK-LO", 57)):
        for i, b in enumerate(fit[fid]["bolts"]):
            p = (KINK_BOLT_X0 + i * KINK_BOLT_PITCH, b["point"][1], b["point"][2])
            F.bolt(6, p, b["axis"], [pid(num), pid(55)], grade=TI, washer_head=True, step=2,
                   label=f"{fid} {b['id']}", notes="Ti, through the 16-ply land of the centre-line rib")

    # ---- 2. box-to-spar-frame bolts (4 x M6 Ti per side per frame, nutplates on the box doublers)
    for sid, sgn in (("FS-MS", 1.0), ("FS-RS", -1.0)):
        s_ = st[sid]
        k = math.tan(math.radians(float(s_["sweep_deg"])))
        n = np.array([1.0, -k, 0.0]) / math.hypot(1.0, k) * sgn          # from the fuel bay into the box
        liner = pid(LINER_N["forward_cell"] if sid == "FS-MS" else LINER_N["aft_cell"])
        for y in BOX_BOLT_Y:
            p = (float(s_["x"]) + y * k, y, 0.0)
            F.sym(6, p, n, [liner, s_["part"], root], nut="nutplate", grade=TI, step=3,
                  label=f"box bolt {sid} y {y}", notes="through the bay liner, the spar-frame web and the box "
                                                       "frame-land doubler into a nutplate inside the box")

    # ---- 3. chine-longeron splices on the side-of-body rib (layout SPL-CH-*, 4 x M6 Ti each)
    sp = C.mem["M-CHINE"]["splices"][0]
    for p in chine_fwd_splice_points(C):
        F.sym(6, p, sp["axis"], [pid(20, "R"), C.ref("M-SOB")], grade=TI, washer_head=True, step=9,
              label=f"{sp['id']} {np.round(p, 4).tolist()}")
    for p in chine_aft_splice_points(C):           # the glove trailing-edge bay is only ~20 mm deep at the SOB rib
        F.sym(4, p, (0.0, 1.0, 0.0), [pid(N_CHINE_AFT, "R"), C.ref("M-SOB")], grade=TI, washer_head=True, step=12,
              label=f"SPL-CH-AFT {np.round(p, 4).tolist()}")

    # ---- 4. shear clips at the chine notches (2 x M4 to the frame web, 1 x M4 to the J web)
    for sid, num in N_CLIPS.items():
        side = CLIP_SIDE[sid]
        _m, pa, pb, xf = build_clip(C, sid, side)
        piece = 20 if float(st[sid]["x"]) < 2.6 else N_CHINE_AFT
        clip = pid(num, "R")
        extra = [pid(LINER_N["forward_cell"])] if sid == "FS-FUEL" else []
        for p in pa:
            F.sym(4, p, (-side, 0.0, 0.0), [clip, st[sid]["part"]] + extra, grade=TI, step=9,
                  label=f"clip {sid} frame", washer_nut=True)
        F.sym(4, pb[0], clip_web_normal(C, sid, side, pb[0][0]), [clip, pid(piece, "R")], grade=TI, step=9,
              label=f"clip {sid} longeron")

    # ---- 5. main-gear trunnion fitting (layout F-TRUNNION B1-B9)
    f = fit["F-TRUNNION"]
    trn, beam = pid(70, "R"), C.ref("M-GEARBEAM")
    land_depth = float(C.mem["M-GEARBEAM"]["box"][1][1] - C.mem["M-GEARBEAM"]["box"][0][1]) + BEAM_LAND_PROUD
    for b in f["bolts"]:
        if b["group"] == "gear beam":
            pt = (b["point"][0], b["point"][1], TRUN_B3_Z) if b["id"] == "B3" else b["point"]
            F.sym(6, pt, (0.0, 1.0, 0.0), [trn], nut="insert", insert_part=beam, insert_depth=land_depth,
                  grade=G129, step=10, label=f"F-TRUNNION {b['id']}",
                  notes="bonded through-thickness M6 insert in the solid land of the gear beam (no nut outboard)")
        else:
            F.sym(6, trunnion_roof_point(b), (0.0, 0.0, 1.0), [trn, C.ref("M-WELLROOF")], nut="nutplate", grade=G129,
                  step=10, label=f"F-TRUNNION {b['id']}", notes="sealed dome nutplate on the fuel side of the roof")
    # up-lock (layout F-UPLOCK: 4 x M5; rows 26 mm apart for the dome nutplates)
    px_u = float(fit["F-UPLOCK"]["point"][0])
    z_r = fit["F-UPLOCK"]["bolts"][0]["point"][2]
    for x in (px_u - UPLOCK_BOLT_DX, px_u + UPLOCK_BOLT_DX):
        for y in UPLOCK_BOLT_Y:
            F.sym(5, (x, y, z_r), (0.0, 0.0, 1.0), [pid(72, "R"), C.ref("M-WELLROOF")], nut="nutplate", grade=G129,
                  step=10, label="F-UPLOCK", notes="sealed dome nutplate on the fuel side of the roof")

    # ---- 6. nose-gear pivot blocks (layout F-NG-PIVOT B1-B6, through the keel walls)
    for b in fit["F-NG-PIVOT"]["bolts"]:
        x, y, z = b["point"]
        s = 1.0 if y > 0 else -1.0
        F.bolt(6, (x, y, z), (0.0, s, 0.0), [pid(71), C.ref("M-KEELWALL", "R" if s > 0 else "L")], grade=G129,
               washer_head=False, step=7, label=f"F-NG-PIVOT {b['id']}")

    # ---- 7. firewall stack: stabilator node B1-B7 (M5), engine-mount feet (M8, upper / lower)
    sp_r = pid(N_SPACER, "R")
    shield = pid(N_SHIELD, "C", "propulsion")
    fw = stp("FS3670")
    for i, (x, y, z) in enumerate(node_firewall_points(C)):
        back = pid(N_UPPER_PLATE, "R") if i < 3 else pid(N_CHINE_END, "R")
        F.sym(5, (x, y, z), (-1.0, 0.0, 0.0), [pid(95, "R"), shield, fw, back], bridge=(sp_r,), grade=G129,
              step=13, label=f"F-SPINDLE-NODE B{i + 1}", notes="through the firewall stack (spacer tube in the gap)")
    pts = engine_foot_points(C)
    for i, p in enumerate(pts):
        upper = i < 2
        stack = [pid(85), pid(86 if upper else 87, "R"), shield, fw, pid(N_UPPER_PLATE if upper else N_EMLO_BACK, "R")]
        F.sym(8, p, (-1.0, 0.0, 0.0), stack, bridge=(sp_r,), owner=pid(85), grade=G129, washer_head=True, step=12,
              label=f"engine foot {'upper' if upper else 'lower'} {i % 2 + 1}",
              notes="NORD-LOCK washer pair under the head (estimate envelope = ISO 7089)")
    # chine end fitting tongue to the aft chine J web (2 x M5 Ti)
    nrm = chine_web_normal(C)
    for p in chine_end_bolt_points(C):
        F.sym(5, p, nrm, [pid(N_CHINE_END, "R"), pid(N_CHINE_AFT, "R")], grade=TI, washer_head=True, step=12,
              label="chine end tongue")
    # dorsal longeron splice (2 x M4 Ti through the tongue and the hat's inboard wall)
    yw = 0.15 - 0.5 * DORSAL_W
    for x, z in DORSAL_SPLICE_BOLTS:
        F.sym(4, (x, yw - DORSAL_TONGUE_T, z), (0.0, 1.0, 0.0), [pid(N_UPPER_PLATE, "R"), pid(32, "R")], grade=TI,
              step=12, label="dorsal splice")

    # ---- 8. tail root fittings on FS3480 (frame bolts of layout F-FIN-FRONT B3/B4, F-STUB-FRONT B1/B2)
    for p in fin_front_frame_points(C):
        F.sym(6, p, (-1.0, 0.0, 0.0), [pid(96, "R"), stp("FS3480")], grade=G129, step=11, label="F-FIN-FRONT frame")
    for b in fit["F-STUB-FRONT"]["bolts"]:
        if b["group"] == "frame":
            F.sym(5, b["point"], (-1.0, 0.0, 0.0), [pid(98, "R"), stp("FS3480")], grade=G129, step=11,
                  label=f"F-STUB-FRONT {b['id']}")
    # ventral root fittings to the aft keel web: 2 x M4 into tapped holes (layout 1 x M6: the lug slot leaves 8 mm)
    k = C.mem["M-AFTKEEL"]
    z_top = float(k["box"][1][2])
    kx0, kx1 = float(k["box"][0][0]), float(k["box"][1][0])
    for fid in ("F-VENTRAL-1", "F-VENTRAL-2", "F-VENTRAL-3"):
        lo, hi = fit[fid]["box"]
        a_, b_ = max(float(lo[0]), kx0), min(float(hi[0]), kx1)
        xc = 0.5 * (a_ + b_)
        num = int(fit[fid]["part"].split("-")[-1])
        for dx in ((-0.008, 0.008) if b_ - a_ >= 0.032 else (0.0,)):
            F.bolt(4, (xc + dx, 0.0, z_top), (0.0, 0.0, -1.0), [C.ref("M-AFTKEEL")], nut="tapped",
                   tapped_part=pid(num), tapped_depth=VENTRAL_TAP_DEPTH, grade=G129, step=11,
                   label=f"{fid} keel bolt")

    # ---- 9. parachute bridle fittings (layout F-RISER-*: 4 x M4 through the spine floor, 2 x M5 to the frame)
    for fid in ("F-RISER-FWD", "F-RISER-AFT"):
        num = int(fit[fid]["part"].split("-")[-1])
        frame = stp("FS1810") if fid.endswith("FWD") else stp("FS-RS")
        for b in fit[fid]["bolts"]:
            x, y, z = b["point"]
            if b["group"] == "spine floor":
                y = math.copysign(RISER_FLOOR_BOLT_Y, y)
                F.bolt(4, (x, y, z), (0.0, 0.0, -1.0), [pid(num), C.ref("M-SPINE"), pid(N_WASHER[fid])],
                       grade=G129, step=14, label=f"{fid} {b['id']}")
            else:
                sgn = 1.0 if fid.endswith("AFT") else -1.0
                y = math.copysign(RISER_FRAME_BOLT[0], y)
                F.bolt(5, (x, y, RISER_FRAME_BOLT[1]), (sgn, 0.0, 0.0), [pid(num), frame], grade=G129, step=14,
                       label=f"{fid} {b['id']}")

    # ---- 10. rear-spar slot fitting to the glove rear-spar web pad (2 x M5 Ti, nuts inside the glove box)
    for y in SLOT_BOLT_Y:
        dxdy = float(bx.xr(y + 0.002) - bx.xr(y - 0.002)) / 0.004
        n = np.array([1.0, -dxdy, 0.0]) / math.hypot(1.0, dxdy)
        F.sym(5, (float(bx.xr(y)) + 0.01, y, float(C.L["chassis"]["wing_joint"]["rear_spar"]["pin"]["position"][2])),
              -n, [pid(54, "R"), root], grade=TI, step=5, label=f"F-REARSLOT web y {y}")

    # ---- 11. FS3738 U-ring: leg / segment splices (3 x M5 each end), aft keel clips (2 x M5 per side)
    for p in uring_splice_points(C):
        F.sym(5, p, (-1.0, 0.0, 0.0), [pid(14), pid(15)], owner=pid(14), grade=G129, step=13, label="U-ring splice")
    pa, pb = keel_clip_points(C)
    F.sym(5, pa, (1.0, 0.0, 0.0), [pid(N_KEEL_CLIP, "R"), pid(15)], grade=G129, step=13, label="keel clip web")
    F.sym(5, pb, (0.0, -1.0, 0.0), [pid(N_KEEL_CLIP, "R"), C.ref("M-AFTKEEL")], grade=G129, step=13,
          label="keel clip wall")

    # ---- 12. trays (M4 into potted inserts / nutplates)
    t6 = R.C.layup_t("rib_panel")
    for side, tid in ((1, "TR-SIDEBAY-R"), (-1, "TR-SIDEBAY-L")):
        tray = pid(TRAY_N[tid])
        kb, fb = sidebay_tray_points(C, side)
        wall = C.ref("M-KEELWALL", "R" if side > 0 else "L")
        for x, y, z in kb:
            F.bolt(4, (x, side * y, z), (0.0, -side, 0.0), [tray], nut="insert", insert_part=wall, insert_depth=t6,
                   grade="A2-70", step=14, label=f"{tid} keel wall")
        for x, y, z in fb:
            F.bolt(4, (x, side * y, z), (1.0, 0.0, 0.0), [tray], nut="insert", insert_part=stp("FS1110"),
                   insert_depth=t6, grade="A2-70", step=14, label=f"{tid} FS1110")
    pa, pb = fwdbay_tray_points(C)
    for p in pa:
        F.bolt(4, p, (-1.0, 0.0, 0.0), [pid(TRAY_N["TR-FWDBAY"])], nut="insert", insert_part=stp("FS0300"),
               insert_depth=t6, grade="A2-70", step=14, label="TR-FWDBAY FS0300")
    for p in pb:
        F.bolt(4, p, (1.0, 0.0, 0.0), [pid(TRAY_N["TR-FWDBAY"])], nut="insert", insert_part=stp("FS0600"),
               insert_depth=t6, grade="A2-70", step=14, label="TR-FWDBAY FS0600")
    for p in mission_tray_points(C):
        F.bolt(4, p, (0.0, 0.0, 1.0), [C.ref("M-MIDFLOOR"), pid(TRAY_N["TR-MISSION"])], nut="nutplate",
               owner=pid(TRAY_N["TR-MISSION"]), grade="A2-70", step=14, label="TR-MISSION",
               notes="captive screw from below through the hat inner flange into a nutplate on the tray")
    pa, pb = aftbay_tray_points(C)
    for p in pa:
        F.bolt(4, p, (-1.0, 0.0, 0.0), [pid(TRAY_N["TR-AFTBAY"])], nut="insert", insert_part=stp("FS-GEAR"),
               insert_depth=t6, grade="A2-70", step=14, label="TR-AFTBAY FS-GEAR")
    for p in pb:
        F.bolt(4, p, (1.0, 0.0, 0.0), [pid(TRAY_N["TR-AFTBAY"])], nut="insert", insert_part=stp("FS3480"),
               insert_depth=t6, grade="A2-70", step=14, label="TR-AFTBAY FS3480")

    # ---- 13. parachute strap brackets: 2 flush blind rivets each (no head in the 6 mm container gap)
    for num, xc in zip(N_PARA_BRKT, PARA_BRKT_X):
        for p in para_bracket_points(C, xc, PARA_BRKT_Z):
            F.sym_pin(PARA_RIVET_D, p, (0.0, 1.0, 0.0), [pid(num, "R"), C.ref("M-PARAWALL")],
                      spec="blind rivet, countersunk, A286 / CherryMAX class d 4.0 (flush on the bay side, estimate)",
                      retention="blind (self-locking)", step=14, label="para bracket rivet")

    # ---- 14. turret rail anchors (M4 into potted inserts of the frame and of the bay wall)
    for rail in C.L["chassis"]["turret_elevator"]["rails"]:
        num = N_RAIL[rail["corner"]]
        fr = "FS1110" if rail["corner"].startswith("F") else "FS1330"
        sx = 1.0 if fr == "FS1110" else -1.0
        sy = 1.0 if rail["line"][0][1] > 0 else -1.0
        wall = C.ref("M-TURRETWALL", "R" if sy > 0 else "L")
        fb, wb = rail_anchor_points(C, rail)
        for p in fb:
            F.bolt(4, p, (-sx, 0.0, 0.0), [pid(num)], nut="insert", insert_part=stp(fr), insert_depth=t6,
                   grade="A2-70", step=14, label=f"rail anchor {rail['corner']} frame")
        for p in wb:
            F.bolt(4, p, (0.0, sy, 0.0), [pid(num)], nut="insert", insert_part=wall, insert_depth=t6,
                   grade="A2-70", step=14, label=f"rail anchor {rail['corner']} wall")


VENTRAL_TAP_DEPTH = 0.0075
RISER_FLOOR_BOLT_Y = 0.0127  # heads clear of the 6 mm ears (layout y +-0.013)
PARA_RIVET_D = 0.004
