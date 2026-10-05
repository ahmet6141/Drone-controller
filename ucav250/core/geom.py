"""Pure-Python mesh kernel for ucav250 (YK-250).

Everything here is plain numpy plus manifold3d (robust booleans, float64 via Mesh64), mapbox_earcut (cap
triangulation) and shapely (2-D profiles). No Blender. Optional: ``mathutils`` (shipped with the bpy module) is used
only by :func:`self_intersections`; every other function works without it.

Conventions (see ucav250/ARCHITECTURE.md):

* Units are SI (metres). Aircraft frame: X = FS (fuselage station, positive AFT, 0 at the nose tip), Y = BL (butt
  line, positive STARBOARD), Z = WL (water line, positive UP). This frame is right-handed and is used unchanged as
  the Blender world frame.
* A :class:`Mesh` is a closed, 2-manifold triangle mesh with consistent winding and OUTWARD normals (counter-clockwise
  seen from outside), i.e. ``Mesh.volume() > 0``. Every generator in this module returns such a mesh or raises.
* Parts are generated in their assembled (rest) position in the aircraft frame. Moving parts rotate about a
  :class:`ucav250.core.parts.Joint`; this module only provides the rigid-transform helpers.
"""
from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Iterable, Sequence

import manifold3d as m3
import mapbox_earcut as earcut
import numpy as np
from shapely.geometry import MultiPolygon, Polygon
from shapely.geometry.polygon import orient

EPS = 1e-12


# =====================================================================================================================
# Small linear-algebra helpers
# =====================================================================================================================
def unit(v) -> np.ndarray:
    v = np.asarray(v, float)
    n = np.linalg.norm(v)
    if n < EPS:
        raise ValueError("zero-length vector")
    return v / n


def rot_axis_angle(axis, angle: float) -> np.ndarray:
    """3x3 rotation matrix, right-hand rule about ``axis`` by ``angle`` (rad)."""
    a = unit(axis)
    x, y, z = a
    c, s = math.cos(angle), math.sin(angle)
    C = 1.0 - c
    return np.array([[c + x * x * C, x * y * C - z * s, x * z * C + y * s],
                     [y * x * C + z * s, c + y * y * C, y * z * C - x * s],
                     [z * x * C - y * s, z * y * C + x * s, c + z * z * C]])


def frame(u, v) -> np.ndarray:
    """Right-handed orthonormal frame (columns e1, e2, e3) with e1 = u and e2 in the (u, v) plane."""
    e1 = unit(u)
    e2 = np.asarray(v, float) - np.dot(v, e1) * e1
    e2 = unit(e2)
    return np.column_stack([e1, e2, np.cross(e1, e2)])


def rotate_about(points, point, axis, angle: float) -> np.ndarray:
    """Rotate ``points`` (n, 3) about the line through ``point`` with direction ``axis``."""
    p = np.asarray(point, float)
    R = rot_axis_angle(axis, angle)
    return (np.asarray(points, float) - p) @ R.T + p


# =====================================================================================================================
# Mesh
# =====================================================================================================================
@dataclass
class Mesh:
    """Closed triangle mesh. ``V`` (n, 3) float64 metres, ``F`` (m, 3) int64, outward (CCW) winding."""

    V: np.ndarray
    F: np.ndarray

    def __post_init__(self):
        self.V = np.ascontiguousarray(self.V, dtype=np.float64).reshape(-1, 3)
        self.F = np.ascontiguousarray(self.F, dtype=np.int64).reshape(-1, 3)

    # ------------------------------------------------------------------ basic
    def copy(self) -> "Mesh":
        return Mesh(self.V.copy(), self.F.copy())

    @property
    def n_verts(self) -> int:
        return len(self.V)

    @property
    def n_faces(self) -> int:
        return len(self.F)

    def transformed(self, R=None, t=None) -> "Mesh":
        """x' = R x + t. A reflection (det R < 0) flips the winding so normals stay outward."""
        V = self.V
        F = self.F
        if R is not None:
            R = np.asarray(R, float)
            V = V @ R.T
            if np.linalg.det(R) < 0:
                F = F[:, ::-1]
        if t is not None:
            V = V + np.asarray(t, float)
        return Mesh(V, F.copy())

    def translated(self, t) -> "Mesh":
        return self.transformed(None, t)

    def rotated_about(self, point, axis, angle: float) -> "Mesh":
        return Mesh(rotate_about(self.V, point, axis, angle), self.F.copy())

    def mirrored_y(self) -> "Mesh":
        """Mirror across the symmetry plane (BL -> -BL); winding is flipped to keep normals outward."""
        return self.transformed(np.diag([1.0, -1.0, 1.0]))

    def bounds(self) -> tuple[np.ndarray, np.ndarray]:
        return self.V.min(axis=0), self.V.max(axis=0)

    def triangles(self) -> np.ndarray:
        return self.V[self.F]

    def volume(self) -> float:
        a, b, c = self.V[self.F[:, 0]], self.V[self.F[:, 1]], self.V[self.F[:, 2]]
        return float(np.einsum("ij,ij->i", a, np.cross(b, c)).sum() / 6.0)

    def area(self) -> float:
        a, b, c = self.V[self.F[:, 0]], self.V[self.F[:, 1]], self.V[self.F[:, 2]]
        return float(0.5 * np.linalg.norm(np.cross(b - a, c - a), axis=1).sum())

    def face_normals(self) -> np.ndarray:
        a, b, c = self.V[self.F[:, 0]], self.V[self.F[:, 1]], self.V[self.F[:, 2]]
        n = np.cross(b - a, c - a)
        ln = np.linalg.norm(n, axis=1, keepdims=True)
        return n / np.maximum(ln, EPS)

    # ------------------------------------------------------------------ mass properties
    def mass_props(self, density: float) -> dict:
        """Solid mass properties for uniform ``density`` (kg/m^3): mass, cg (3,), inertia about cg (3, 3) in the
        aircraft frame. Exact for closed meshes (signed tetrahedra about the origin)."""
        a, b, c = self.V[self.F[:, 0]], self.V[self.F[:, 1]], self.V[self.F[:, 2]]
        dv = np.einsum("ij,ij->i", a, np.cross(b, c)) / 6.0
        vol = dv.sum()
        if vol <= 0:
            raise ValueError(f"mass_props: non-positive volume {vol:.3e} (open mesh or inward normals)")
        cg = ((a + b + c) * dv[:, None]).sum(0) / (4.0 * vol)
        # second moments: integral of x_i x_j over each tetra (0, a, b, c)
        S = np.zeros((3, 3))
        for i in range(3):
            for j in range(3):
                s = (2 * (a[:, i] * a[:, j] + b[:, i] * b[:, j] + c[:, i] * c[:, j])
                     + a[:, i] * b[:, j] + a[:, j] * b[:, i] + a[:, i] * c[:, j] + a[:, j] * c[:, i]
                     + b[:, i] * c[:, j] + b[:, j] * c[:, i])
                S[i, j] = (dv * s).sum() / 20.0
        S_cg = S - vol * np.outer(cg, cg)
        I = np.trace(S_cg) * np.eye(3) - S_cg
        return {"volume": float(vol), "mass": float(density * vol), "cg": cg, "inertia": density * I}

    # ------------------------------------------------------------------ manifold3d interop
    def to_manifold(self) -> m3.Manifold:
        # explicit writable copies: arrays that came from manifold3d are read-only views and nanobind rejects them
        mg = m3.Mesh64(vert_properties=np.array(self.V, dtype=np.float64, order="C", copy=True),
                       tri_verts=np.array(self.F, dtype=np.uint64, order="C", copy=True))
        man = m3.Manifold(mg)
        st = man.status()
        if str(st) not in ("Error.NoError", "NoError"):
            raise ValueError(f"to_manifold: {st}")
        return man

    @staticmethod
    def from_manifold(man: m3.Manifold) -> "Mesh":
        mg = man.to_mesh64()
        V = np.array(np.asarray(mg.vert_properties)[:, :3], dtype=np.float64, order="C", copy=True)
        F = np.array(mg.tri_verts, dtype=np.int64, order="C", copy=True)
        return Mesh(V, F)

    # ------------------------------------------------------------------ validation
    def edge_stats(self) -> dict:
        """Directed-edge pairing: every directed edge must appear exactly once and its reverse exactly once."""
        F = self.F
        E = np.concatenate([F[:, [0, 1]], F[:, [1, 2]], F[:, [2, 0]]])
        n = max(int(self.V.shape[0]), 1)
        key = E[:, 0] * n + E[:, 1]
        rkey = E[:, 1] * n + E[:, 0]
        uniq, cnt = np.unique(key, return_counts=True)
        dup = int((cnt > 1).sum())
        unpaired = int((~np.isin(rkey, key)).sum())
        return {"dup_directed": dup, "unpaired": unpaired}

    def check(self, *, self_intersect: bool = False) -> dict:
        """Watertight + consistent + outward + no degenerate faces (+ optional triangle self-intersection test)."""
        es = self.edge_stats()
        a, b, c = self.V[self.F[:, 0]], self.V[self.F[:, 1]], self.V[self.F[:, 2]]
        areas = 0.5 * np.linalg.norm(np.cross(b - a, c - a), axis=1)
        vol = self.volume()
        out = {"faces": self.n_faces, "verts": self.n_verts, **es, "volume": vol,
               "degenerate": int((areas < 1e-14).sum()), "unused_verts": int(self.n_verts - len(np.unique(self.F)))}
        out["ok"] = bool(es["dup_directed"] == 0 and es["unpaired"] == 0 and vol > 0 and out["degenerate"] == 0
                         and self.n_faces >= 4)
        if self_intersect:
            out["self_intersections"] = self_intersections(self)
            out["ok"] = out["ok"] and out["self_intersections"] == 0
        return out


def merge(meshes: Iterable[Mesh]) -> Mesh:
    """Concatenate meshes (disjoint shells; NOT a boolean union)."""
    Vs, Fs, off = [], [], 0
    for m in meshes:
        Vs.append(m.V)
        Fs.append(m.F + off)
        off += len(m.V)
    if not Vs:
        raise ValueError("merge: no meshes")
    return Mesh(np.vstack(Vs), np.vstack(Fs))


def weld(mesh: Mesh, tol: float = 1e-9) -> Mesh:
    """Merge coincident vertices (quantised to ``tol``) and drop faces that became degenerate."""
    q = np.round(mesh.V / tol).astype(np.int64)
    _, idx, inv = np.unique(q, axis=0, return_index=True, return_inverse=True)
    V = mesh.V[idx]
    F = inv.reshape(-1)[mesh.F]
    keep = (F[:, 0] != F[:, 1]) & (F[:, 1] != F[:, 2]) & (F[:, 2] != F[:, 0])
    return Mesh(V, F[keep])


def fix_orientation(mesh: Mesh) -> Mesh:
    """Flip all faces if the closed mesh has negative volume."""
    return mesh if mesh.volume() > 0 else Mesh(mesh.V, mesh.F[:, ::-1])


# =====================================================================================================================
# Booleans and distances (manifold3d)
# =====================================================================================================================
def union(meshes: Sequence[Mesh]) -> Mesh:
    mans = [m.to_manifold() for m in meshes]
    return Mesh.from_manifold(m3.Manifold.batch_boolean(mans, m3.OpType.Add))


def difference(a: Mesh, cutters: Sequence[Mesh]) -> Mesh:
    if not cutters:
        return a.copy()
    cut = m3.Manifold.batch_boolean([c.to_manifold() for c in cutters], m3.OpType.Add)
    res = a.to_manifold() - cut
    if res.is_empty():
        raise ValueError("difference: result is empty")
    return Mesh.from_manifold(res)


def intersection(a: Mesh, b: Mesh) -> Mesh | None:
    res = a.to_manifold() ^ b.to_manifold()
    return None if res.is_empty() else Mesh.from_manifold(res)


def intersection_volume(a: Mesh, b: Mesh) -> float:
    """Volume of A ∩ B (m^3). Faces that merely touch give ~0."""
    lo_a, hi_a = a.bounds()
    lo_b, hi_b = b.bounds()
    if np.any(hi_a < lo_b) or np.any(hi_b < lo_a):
        return 0.0
    res = a.to_manifold() ^ b.to_manifold()
    return 0.0 if res.is_empty() else float(res.volume())


def min_gap(a: Mesh, b: Mesh, search: float = 0.05) -> float:
    """Smallest distance between two disjoint solids, searched up to ``search`` metres (returns ``search`` if
    farther apart; 0 if they touch or overlap)."""
    return float(a.to_manifold().min_gap(b.to_manifold(), search))


def aabb_overlap(a: Mesh, b: Mesh, pad: float = 0.0) -> bool:
    lo_a, hi_a = a.bounds()
    lo_b, hi_b = b.bounds()
    return bool(np.all(hi_a + pad >= lo_b) and np.all(hi_b + pad >= lo_a))


def self_intersections(mesh: Mesh) -> int:
    """Number of intersecting triangle pairs that do not share a vertex (mathutils BVH; needs the bpy module)."""
    try:
        from mathutils.bvhtree import BVHTree  # noqa: WPS433
    except Exception:  # pragma: no cover - bpy not installed
        import bpy  # noqa: F401  (importing bpy makes mathutils importable)
        from mathutils.bvhtree import BVHTree
    t = BVHTree.FromPolygons(mesh.V.tolist(), mesh.F.tolist(), all_triangles=True, epsilon=0.0)
    pairs = t.overlap(t)
    if not pairs:
        return 0
    # Pairs that share a vertex "touch" there; the float32 tri-tri test sometimes reports them (thin fans at poles).
    # Re-test those with both triangles shrunk 0.2 % about their centroids: a real fold still intersects.
    real = 0
    F = mesh.F
    for a, b in pairs:
        if not set(F[a].tolist()) & set(F[b].tolist()):
            real += 1
            continue
        ta, tb = mesh.V[F[a]], mesh.V[F[b]]
        ta = ta.mean(0) + 0.998 * (ta - ta.mean(0))
        tb = tb.mean(0) + 0.998 * (tb - tb.mean(0))
        t2 = BVHTree.FromPolygons(np.vstack([ta, tb]).tolist(), [(0, 1, 2), (3, 4, 5)], all_triangles=True,
                                  epsilon=0.0)
        if any((i, j) in ((0, 1), (1, 0)) for i, j in t2.overlap(t2)):
            real += 1
    return real


# =====================================================================================================================
# 2-D profiles -> solids
# =====================================================================================================================
def _poly_parts(poly) -> list[Polygon]:
    if isinstance(poly, Polygon):
        return [poly]
    if isinstance(poly, MultiPolygon):
        return list(poly.geoms)
    raise TypeError(f"expected shapely Polygon/MultiPolygon, got {type(poly).__name__}")


def _earcut(rings: list[np.ndarray]) -> np.ndarray:
    pts = np.vstack(rings).astype(np.float64)
    ends = np.cumsum([len(r) for r in rings]).astype(np.uint32)
    tri = earcut.triangulate_float64(pts, ends)
    return np.asarray(tri, dtype=np.int64).reshape(-1, 3)


def _ring(coords) -> np.ndarray:
    a = np.asarray(coords, float)[:, :2]
    if len(a) > 1 and np.allclose(a[0], a[-1]):
        a = a[:-1]
    # drop consecutive duplicates
    keep = np.ones(len(a), bool)
    keep[1:] = np.linalg.norm(np.diff(a, axis=0), axis=1) > 1e-12
    return a[keep]


def extrude(poly, thickness: float, origin=(0.0, 0.0, 0.0), u=(1.0, 0.0, 0.0), v=(0.0, 1.0, 0.0),
            centered: bool = False) -> Mesh:
    """Prism from a shapely (Multi)Polygon (holes allowed) drawn in the (u, v) plane at ``origin``; extruded along
    w = u x v by ``thickness`` (from 0 to +t, or -t/2..+t/2 if ``centered``)."""
    if thickness <= 0:
        raise ValueError("extrude: thickness must be > 0")
    Fr = frame(u, v)
    e1, e2, w = Fr[:, 0], Fr[:, 1], Fr[:, 2]
    o = np.asarray(origin, float) - (0.5 * thickness * w if centered else 0.0)
    out = []
    for p in _poly_parts(poly):
        p = orient(p, 1.0)                                 # exterior CCW, holes CW
        rings = [_ring(p.exterior.coords)] + [_ring(h.coords) for h in p.interiors]
        tri = _earcut(rings)
        P2 = np.vstack(rings)
        n = len(P2)
        base = o + P2[:, :1] * e1 + P2[:, 1:2] * e2
        V = np.vstack([base, base + thickness * w])
        # earcut returns triangles with the winding of the input exterior (CCW in (u, v)) -> normal +w on the top cap
        F = [tri[:, ::-1], tri + n]
        start = 0
        for r in rings:
            k = len(r)
            idx = np.arange(start, start + k)
            nxt = np.roll(idx, -1)
            # side wall of a CCW exterior / CW hole: outward quad (i, nxt, nxt+n, i+n)
            F.append(np.column_stack([idx, nxt, nxt + n]))
            F.append(np.column_stack([idx, nxt + n, idx + n]))
            start += k
        out.append(Mesh(V, np.vstack(F)))
    m = merge(out) if len(out) > 1 else out[0]
    return fix_orientation(m)


def revolve(profile_rz, n: int = 48, axis_origin=(0.0, 0.0, 0.0), axis=(1.0, 0.0, 0.0), ref=(0.0, 0.0, 1.0),
            angle: float = 2 * math.pi) -> Mesh:
    """Solid of revolution. ``profile_rz`` is a closed polygon (k, 2) of (r, z) with r >= 0, z along ``axis``.
    Points with r == 0 are allowed (poles). Full revolutions only (``angle`` = 2*pi)."""
    if abs(angle - 2 * math.pi) > 1e-9:
        raise NotImplementedError("revolve: only full revolutions")
    P = _ring(profile_rz)
    ax = unit(axis)
    ref = np.asarray(ref, float)
    if abs(float(np.dot(unit(ref), ax))) > 0.99:            # reference parallel to the axis: pick another one
        ref = np.array([1.0, 0.0, 0.0]) if abs(ax[0]) < 0.9 else np.array([0.0, 1.0, 0.0])
    r0 = unit(ref - np.dot(ref, ax) * ax)
    r1 = np.cross(ax, r0)
    th = np.linspace(0.0, 2 * math.pi, n, endpoint=False)
    V, idx = [], []
    for (r, z) in P:
        if r < 1e-12:
            idx.append([len(V)] * n)
            V.append(np.asarray(axis_origin, float) + z * ax)
        else:
            row = []
            for t in th:
                row.append(len(V))
                V.append(np.asarray(axis_origin, float) + z * ax + r * (math.cos(t) * r0 + math.sin(t) * r1))
            idx.append(row)
    V = np.asarray(V)
    F = []
    k = len(P)
    for i in range(k):
        a, b = idx[i], idx[(i + 1) % k]
        for j in range(n):
            j2 = (j + 1) % n
            q = (a[j], a[j2], b[j2], b[j])
            tris = [(q[0], q[1], q[2]), (q[0], q[2], q[3])]
            for t in tris:
                if len(set(t)) == 3:
                    F.append(t)
    return fix_orientation(weld(Mesh(V, np.asarray(F))))


def loft(rings: Sequence[np.ndarray], cap: bool = True) -> Mesh:
    """Closed solid through planar-ish rings (each (k, 3), same k, consistent start point and direction).
    End caps are triangulated in the best-fit plane of the end ring."""
    R = [np.asarray(r, float) for r in rings]
    k = len(R[0])
    if any(len(r) != k for r in R):
        raise ValueError("loft: rings must have equal point counts")
    V = np.vstack(R)
    F = []
    for i in range(len(R) - 1):
        a = np.arange(i * k, (i + 1) * k)
        b = a + k
        a2, b2 = np.roll(a, -1), np.roll(b, -1)
        F.append(np.column_stack([a, a2, b2]))
        F.append(np.column_stack([a, b2, b]))
    if cap:
        for which, base in ((0, 0), (len(R) - 1, (len(R) - 1) * k)):
            ring = R[which]
            c = ring.mean(0)
            _, _, vt = np.linalg.svd(ring - c)
            e1, e2 = vt[0], vt[1]
            P2 = np.column_stack([(ring - c) @ e1, (ring - c) @ e2])
            if Polygon(P2).exterior.is_ccw is False:
                P2 = P2[::-1]
                order = np.arange(k)[::-1]
            else:
                order = np.arange(k)
            tri = _earcut([P2])
            F.append(base + order[tri])
    m = Mesh(V, np.vstack(F))
    # make winding consistent: caps were added in an arbitrary orientation -> rebuild via manifold check
    m = _orient_consistently(m)
    return fix_orientation(m)


def _orient_consistently(m: Mesh) -> Mesh:
    """Flip faces so that every shared edge is traversed in opposite directions (BFS over faces)."""
    F = m.F.copy()
    nF = len(F)
    edge_faces: dict[tuple[int, int], list[int]] = {}
    for fi, f in enumerate(F):
        for a, b in ((f[0], f[1]), (f[1], f[2]), (f[2], f[0])):
            edge_faces.setdefault((min(a, b), max(a, b)), []).append(fi)
    seen = np.zeros(nF, bool)
    for start in range(nF):
        if seen[start]:
            continue
        seen[start] = True
        stack = [start]
        while stack:
            fi = stack.pop()
            f = F[fi]
            for a, b in ((f[0], f[1]), (f[1], f[2]), (f[2], f[0])):
                for fj in edge_faces[(min(a, b), max(a, b))]:
                    if fj == fi or seen[fj]:
                        continue
                    g = F[fj]
                    same = any((g[i] == a and g[(i + 1) % 3] == b) for i in range(3))
                    if same:
                        F[fj] = g[::-1]
                    seen[fj] = True
                    stack.append(fj)
    return Mesh(m.V, F)


def shell_from_grid(P, t, inward=None, close_u: bool = False) -> Mesh:
    """Thick panel from a structured surface grid ``P`` (nu, nv, 3) -> closed solid of thickness ``t`` (scalar or
    (nu, nv)). The second surface is P + t * n where n is the unit normal pointing to the INSIDE of the aircraft:
    pass ``inward`` (nu, nv, 3) explicitly, or a point/axis is inferred as the normal whose direction points away
    from the grid's outward side, taken as dP/du x dP/dv (callers must order u, v accordingly).
    ``close_u`` makes the grid periodic in u (a full ring, e.g. a fuselage barrel) -> side walls only at v ends."""
    P = np.asarray(P, float)
    nu, nv, _ = P.shape
    if inward is None:
        du = np.gradient(P, axis=0)
        dv = np.gradient(P, axis=1)
        if close_u:
            du = 0.5 * (np.roll(P, -1, axis=0) - np.roll(P, 1, axis=0))
        n = np.cross(du, dv)
        n /= np.maximum(np.linalg.norm(n, axis=2, keepdims=True), EPS)
        inward = -n
    else:
        inward = np.asarray(inward, float)
        inward = inward / np.maximum(np.linalg.norm(inward, axis=2, keepdims=True), EPS)
    T = np.broadcast_to(np.asarray(t, float), (nu, nv))[..., None]
    Q = P + T * inward
    idx_o = np.arange(nu * nv).reshape(nu, nv)
    idx_i = idx_o + nu * nv
    V = np.vstack([P.reshape(-1, 3), Q.reshape(-1, 3)])
    F = []
    urange = range(nu) if close_u else range(nu - 1)
    for i in urange:
        i2 = (i + 1) % nu
        for j in range(nv - 1):
            a, b, c, d = idx_o[i, j], idx_o[i2, j], idx_o[i2, j + 1], idx_o[i, j + 1]
            F += [(a, b, c), (a, c, d)]
            a, b, c, d = idx_i[i, j], idx_i[i2, j], idx_i[i2, j + 1], idx_i[i, j + 1]
            F += [(a, c, b), (a, d, c)]

    def wall(o_line, i_line):
        for k in range(len(o_line) - 1):
            a, b = o_line[k], o_line[k + 1]
            c, d = i_line[k + 1], i_line[k]
            F.extend([(a, b, c), (a, c, d)])

    wall(idx_o[:, 0][::-1] if not close_u else np.r_[idx_o[:, 0], idx_o[0, 0]][::-1],
         idx_i[:, 0][::-1] if not close_u else np.r_[idx_i[:, 0], idx_i[0, 0]][::-1])
    wall(idx_o[:, -1] if not close_u else np.r_[idx_o[:, -1], idx_o[0, -1]],
         idx_i[:, -1] if not close_u else np.r_[idx_i[:, -1], idx_i[0, -1]])
    if not close_u:
        wall(idx_o[0, :], idx_i[0, :])
        wall(idx_o[-1, :][::-1], idx_i[-1, :][::-1])
    m = _orient_consistently(Mesh(V, np.asarray(F)))
    return fix_orientation(m)


# =====================================================================================================================
# Primitives
# =====================================================================================================================
def box(size, center=(0.0, 0.0, 0.0), R=None) -> Mesh:
    sx, sy, sz = (0.5 * float(s) for s in size)
    V = np.array([[-sx, -sy, -sz], [sx, -sy, -sz], [sx, sy, -sz], [-sx, sy, -sz],
                  [-sx, -sy, sz], [sx, -sy, sz], [sx, sy, sz], [-sx, sy, sz]])
    F = np.array([[0, 2, 1], [0, 3, 2], [4, 5, 6], [4, 6, 7], [0, 1, 5], [0, 5, 4],
                  [1, 2, 6], [1, 6, 5], [2, 3, 7], [2, 7, 6], [3, 0, 4], [3, 4, 7]])
    return Mesh(V, F).transformed(R, center)


def cylinder(r: float, p0, p1, n: int = 32, r1: float | None = None) -> Mesh:
    """Cylinder (or cone frustum with end radius ``r1``) from point p0 to p1."""
    p0, p1 = np.asarray(p0, float), np.asarray(p1, float)
    L = float(np.linalg.norm(p1 - p0))
    r1 = r if r1 is None else r1
    prof = [(0.0, 0.0), (r, 0.0), (r1, L), (0.0, L)]
    ax = unit(p1 - p0)
    ref = np.array([0.0, 0.0, 1.0]) if abs(ax[2]) < 0.9 else np.array([1.0, 0.0, 0.0])
    return revolve(prof, n=n, axis_origin=p0, axis=ax, ref=ref)


def tube(ro: float, ri: float, p0, p1, n: int = 32) -> Mesh:
    """Round tube (outer radius ro, inner radius ri) from p0 to p1."""
    if not 0 < ri < ro:
        raise ValueError("tube: need 0 < ri < ro")
    p0, p1 = np.asarray(p0, float), np.asarray(p1, float)
    L = float(np.linalg.norm(p1 - p0))
    ax = unit(p1 - p0)
    ref = np.array([0.0, 0.0, 1.0]) if abs(ax[2]) < 0.9 else np.array([1.0, 0.0, 0.0])
    return revolve([(ri, 0.0), (ro, 0.0), (ro, L), (ri, L)], n=n, axis_origin=p0, axis=ax, ref=ref)


def sphere(r: float, center=(0.0, 0.0, 0.0), n: int = 32) -> Mesh:
    th = np.linspace(0, math.pi, max(n // 2, 4) + 1)
    prof = [(r * math.sin(t), -r * math.cos(t)) for t in th]
    prof = [(0.0, -r)] + prof[1:-1] + [(0.0, r)]
    return revolve(prof, n=n, axis_origin=center, axis=(0, 0, 1), ref=(1, 0, 0))


def sweep_circle(path, r: float, n: int = 16) -> Mesh:
    """Round solid rod of radius ``r`` along a polyline ``path`` (k, 3) with parallel-transport frames."""
    P = np.asarray(path, float)
    if len(P) < 2:
        raise ValueError("sweep_circle: need >= 2 points")
    T = np.gradient(P, axis=0)
    T /= np.maximum(np.linalg.norm(T, axis=1, keepdims=True), EPS)
    ref = np.array([0.0, 0.0, 1.0]) if abs(T[0, 2]) < 0.9 else np.array([1.0, 0.0, 0.0])
    N = unit(np.cross(T[0], ref))
    rings = []
    th = np.linspace(0, 2 * math.pi, n, endpoint=False)
    for i in range(len(P)):
        if i > 0:
            b = np.cross(T[i - 1], T[i])
            s = np.linalg.norm(b)
            if s > 1e-9:
                ang = math.atan2(s, float(np.dot(T[i - 1], T[i])))
                N = rot_axis_angle(b, ang) @ N
            N = unit(N - np.dot(N, T[i]) * T[i])
        B = np.cross(T[i], N)
        rings.append(P[i] + r * (np.cos(th)[:, None] * N + np.sin(th)[:, None] * B))
    return loft(rings)


def hull(points) -> Mesh:
    """Convex hull solid of a point cloud (manifold3d)."""
    pts = np.asarray(points, float)
    return Mesh.from_manifold(m3.Manifold.hull_points(pts.tolist()))


# =====================================================================================================================
# Thickness probe
# =====================================================================================================================
def ray_hits(man, origin, end) -> np.ndarray:
    """All surface crossings of the segment origin->end on a manifold (or Mesh), as distances in metres from
    ``origin``, sorted. manifold3d returns every crossing with the distance normalised to the segment length."""
    if isinstance(man, Mesh):
        man = man.to_manifold()
    o = np.asarray(origin, float)
    e = np.asarray(end, float)
    L = float(np.linalg.norm(e - o))
    hits = man.ray_cast(tuple(o), tuple(e))
    return np.sort(np.array([float(h.distance) * L for h in hits], float))


def wall_thickness_samples(mesh: Mesh, n: int = 400, seed: int = 0) -> np.ndarray:
    """Ray-cast wall thickness at ``n`` area-weighted surface samples: distance from each sample (just inside the
    surface) along the inward normal to the next surface hit. Uses manifold3d ray casting (one call per ray)."""
    rng = np.random.default_rng(seed)
    tri = mesh.triangles()
    nrm = mesh.face_normals()
    ar = 0.5 * np.linalg.norm(np.cross(tri[:, 1] - tri[:, 0], tri[:, 2] - tri[:, 0]), axis=1)
    pick = rng.choice(len(tri), size=n, p=ar / ar.sum())
    u = rng.random((n, 2))
    m = u.sum(1) > 1
    u[m] = 1 - u[m]
    pts = tri[pick, 0] + u[:, :1] * (tri[pick, 1] - tri[pick, 0]) + u[:, 1:] * (tri[pick, 2] - tri[pick, 0])
    d = -nrm[pick]
    start = pts + 1e-7 * d
    end = pts + 1.0 * d
    man = mesh.to_manifold()
    out = np.full(n, np.nan)
    try:
        for i in range(n):
            h = ray_hits(man, start[i], end[i])
            if len(h):
                out[i] = h[0]
    except Exception:  # pragma: no cover - API mismatch fallback
        return _thickness_bvh(mesh, start, d)
    return out


def _thickness_bvh(mesh: Mesh, start, d) -> np.ndarray:  # pragma: no cover - fallback path
    import bpy  # noqa: F401
    from mathutils import Vector
    from mathutils.bvhtree import BVHTree
    t = BVHTree.FromPolygons(mesh.V.tolist(), mesh.F.tolist(), all_triangles=True)
    out = []
    for s, dd in zip(start, d):
        loc, _, _, dist = t.ray_cast(Vector(s), Vector(dd), 1.0)
        out.append(np.nan if loc is None else dist)
    return np.asarray(out)
