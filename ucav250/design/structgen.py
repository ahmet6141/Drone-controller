"""Shared structural-part generators built on the OML evaluators (shared math, registers no parts).

Every generator returns a closed, outward-oriented :class:`ucav250.core.geom.Mesh` that passes
``Mesh.check(self_intersect=True)``. Section-based parts work in a 2-D section frame and use shapely for the
planar logic (inset by skin thickness, spar cut-outs, lightening holes), so the parts fit the loft exactly:

* lifting surfaces: ``section2d`` (local (s, t) = (along chord from LE, along local up) in metres), ``rib``,
  ``loft_region`` (any per-section region, e.g. a spar box, a leading-edge part, a control surface or the fixed part
  in front of it), ``control_surface_regions`` (round-nose moving surface + cove on the fixed part, hinge axis);
* fuselage: ``fuselage_section2d`` (y, z) polygon at a station, ``bulkhead`` (planar frame plate), ``fuselage_panel``
  (skin panel between stations and polar angles, thickened inward).
"""
from __future__ import annotations

import math
from typing import Callable

import numpy as np
from shapely.geometry import LineString, MultiPolygon, Point, Polygon, box as sbox
from shapely.geometry.polygon import orient

from ..core.geom import Mesh, extrude, loft, shell_from_grid, unit
from .oml import Fuselage, LiftingSurface


# =====================================================================================================================
# polygon helpers
# =====================================================================================================================
def largest(poly) -> Polygon:
    """Largest polygon of a (Multi)Polygon / GeometryCollection result."""
    if poly is None or poly.is_empty:
        raise ValueError("empty region")
    if isinstance(poly, Polygon):
        return poly
    parts = [g for g in getattr(poly, "geoms", []) if isinstance(g, Polygon) and not g.is_empty]
    if not parts:
        raise ValueError("region has no polygon part")
    return max(parts, key=lambda g: g.area)


def resample_ring(poly: Polygon, n: int, start_dir=(1.0, 0.0)) -> np.ndarray:
    """``n`` points evenly spaced by arc length along the exterior (CCW), starting at the vertex extreme in
    ``start_dir`` (consistent start across a loft). Original corner vertices are snapped in so sharp corners survive."""
    ext = orient(poly, 1.0).exterior
    P = np.asarray(ext.coords)[:-1]
    d = np.asarray(start_dir, float)
    i0 = int(np.argmax(P @ d))
    P = np.roll(P, -i0, axis=0)
    seg = np.linalg.norm(np.diff(np.vstack([P, P[:1]]), axis=0), axis=1)
    cum = np.r_[0.0, np.cumsum(seg)]
    L = cum[-1]
    s = np.linspace(0.0, L, n, endpoint=False)
    # snap: replace the nearest sample by every sharp corner (turn > 30°)
    V = np.vstack([P, P[:1]])
    out = np.column_stack([np.interp(s, cum, V[:, 0]), np.interp(s, cum, V[:, 1])])
    for k in range(len(P)):
        a, b, c = P[k - 1], P[k], P[(k + 1) % len(P)]
        u, v = b - a, c - b
        nu, nv = np.linalg.norm(u), np.linalg.norm(v)
        if nu < 1e-12 or nv < 1e-12:
            continue
        if math.degrees(math.acos(np.clip(np.dot(u, v) / (nu * nv), -1, 1))) > 30:
            j = int(np.argmin(np.abs(s - cum[k])))
            out[j] = b
    return out


# =====================================================================================================================
# lifting surfaces
# =====================================================================================================================
def section2d(surf: LiftingSurface, eta: float, n: int = 160) -> tuple[Polygon, tuple]:
    """Exact OML section at span coordinate ``eta`` as a shapely polygon in the local (s, t) frame (metres; s along
    the chord from the leading edge, t along local up) and the frame ``(origin, chord_dir, up_dir, span_normal)``."""
    loop = surf.loop_at(eta, n)
    o, c, u, w = surf.frame_at(eta)
    d = loop - o
    P = np.column_stack([d @ c, d @ u])
    poly = Polygon(P).buffer(0)
    return largest(poly), (o, c, u, w)


def to3d(frame: tuple, pts2d: np.ndarray) -> np.ndarray:
    o, c, u, _w = frame
    pts2d = np.asarray(pts2d, float)
    return o + pts2d[:, :1] * c + pts2d[:, 1:2] * u


def plate_in_section(poly2d, frame: tuple, t: float) -> Mesh:
    """Planar part (holes allowed) drawn in a section frame, thickness ``t`` centred on the section plane."""
    o, c, u, _w = frame
    return extrude(poly2d, t, origin=o, u=c, v=u, centered=True)


def rib(surf: LiftingSurface, eta: float, t: float, inset: float, cutouts=(), holes=(), xc_range=(0.0, 1.0),
        n: int = 200) -> Mesh:
    """Rib plate at ``eta``: OML section inset by ``inset`` (skin thickness + bond gap), clipped to the chord range
    ``xc_range`` (fractions), minus rectangular ``cutouts`` [(xc_centre, width_m, height_m or None=full depth)] and
    circular lightening ``holes`` [(xc_centre, diameter_m)] (holes centred on the local camber line)."""
    poly, fr = section2d(surf, eta, n)
    ch = surf.chord_at(eta)
    region = poly.buffer(-inset, join_style=2).intersection(
        sbox(xc_range[0] * ch, -ch, xc_range[1] * ch, ch))
    region = largest(region)
    for xc, w, h in cutouts:
        x = xc * ch
        if h is None:
            region = region.difference(sbox(x - w / 2, -ch, x + w / 2, ch))
        else:
            zc = _camber_t(poly, x)
            region = region.difference(sbox(x - w / 2, zc - h / 2, x + w / 2, zc + h / 2))
    for xc, dia in holes:
        x = xc * ch
        region = region.difference(Point(x, _camber_t(poly, x)).buffer(dia / 2, 48))
    if isinstance(region, MultiPolygon):
        parts = [extrude(p, t, origin=fr[0], u=fr[1], v=fr[2], centered=True) for p in region.geoms if p.area > 1e-8]
        from ..core.geom import merge
        return merge(parts)
    return plate_in_section(region, fr, t)


def _camber_t(poly: Polygon, s: float) -> float:
    line = LineString([(s, -10.0), (s, 10.0)]).intersection(poly)
    if line.is_empty:
        return 0.0
    ys = np.asarray(line.coords)[:, 1] if hasattr(line, "coords") else np.concatenate(
        [np.asarray(g.coords)[:, 1] for g in line.geoms])
    return float(0.5 * (ys.min() + ys.max()))


def thickness_at(surf: LiftingSurface, eta: float, xc: float) -> tuple[float, float]:
    """(upper t, lower t) of the OML at chord fraction ``xc`` (local frame, metres)."""
    poly, _ = section2d(surf, eta)
    s = xc * surf.chord_at(eta)
    line = LineString([(s, -10.0), (s, 10.0)]).intersection(poly)
    ys = np.asarray(line.coords)[:, 1] if hasattr(line, "coords") else np.concatenate(
        [np.asarray(g.coords)[:, 1] for g in line.geoms])
    return float(ys.max()), float(ys.min())


def loft_region(surf: LiftingSurface, eta0: float, eta1: float,
                region_fn: Callable[[Polygon, float, float], Polygon], n_ring: int = 160, n_span: int | None = None,
                start_dir=(1.0, 0.0)) -> Mesh:
    """Closed solid lofted through per-station regions. ``region_fn(section_poly, chord, eta) -> Polygon`` (single,
    simply connected, same topology along the span) in the section (s, t) frame. Stations: both ends plus every
    original section inside (the OML is ruled between sections, so this is exact for chord-fraction cuts)."""
    e = surf.span_coords()
    stations = [eta0] + [float(x) for x in e if eta0 + 1e-6 < x < eta1 - 1e-6] + [eta1]
    if n_span and n_span > len(stations):
        stations = sorted(set(stations) | set(np.linspace(eta0, eta1, n_span).tolist()))
    rings = []
    for eta in stations:
        poly, fr = section2d(surf, eta)
        reg = largest(region_fn(poly, surf.chord_at(eta), eta))
        if len(reg.interiors):
            raise ValueError("loft_region: region must be simply connected")
        rings.append(to3d(fr, resample_ring(reg, n_ring, start_dir)))
    return loft(rings)


def chord_band(xc0: float, xc1: float, inset: float = 0.0):
    """Region function: the section between chord fractions xc0..xc1, optionally inset (e.g. a solid core inside the
    skins or a spar web box)."""
    def fn(poly, ch, _eta):
        base = poly.buffer(-inset, join_style=2) if inset > 0 else poly
        return base.intersection(sbox(xc0 * ch, -ch, xc1 * ch, ch))
    return fn


def control_surface_regions(xc_hinge: float, gap: float = 0.0015, hinge_frac: float = 0.5, nose_margin: float = 0.0):
    """Region functions for a plain control surface with a round nose and a matching cove:

    * hinge point H at chord fraction ``xc_hinge``, height ``hinge_frac`` between the lower (0) and upper (1) OML;
    * moving part = section aft of H plus a disc of radius r (distance from H to the nearer skin), i.e. a round nose
      that stays inside the cove disc in every deflection;
    * fixed part = section forward of H minus the cove disc of radius r + ``gap`` (and ``nose_margin`` extra set-back).

    Returns ``(fixed_fn, moving_fn, hinge_fn)``; ``hinge_fn(poly, chord) -> (s, t)`` gives H in the section frame so
    the caller can build the 3-D hinge axis from two stations (``to3d``). Swept clearance of the real range must
    still be checked (aft skin corners can swing forward beyond the cove at large deflections)."""
    def hinge(poly, ch):
        s = xc_hinge * ch
        tu, tl = _span_t(poly, s)
        t = tl + hinge_frac * (tu - tl)
        r = min(tu - t, t - tl)
        return s, t, r

    def moving(poly, ch, _eta):
        s, t, r = hinge(poly, ch)
        aft = poly.intersection(sbox(s, -ch, 2 * ch, ch))
        nose = Point(s, t).buffer(max(r - 1e-5, 1e-4), 64).intersection(poly)
        return largest(aft.union(nose).buffer(0))

    def fixed(poly, ch, _eta):
        s, t, r = hinge(poly, ch)
        fwd = poly.intersection(sbox(-ch, -ch, s, ch))
        cove = Point(s, t).buffer(r + gap + nose_margin, 64)
        return largest(fwd.difference(cove))

    def hinge_fn(poly, ch):
        s, t, _ = hinge(poly, ch)
        return s, t
    return fixed, moving, hinge_fn


def _span_t(poly: Polygon, s: float) -> tuple[float, float]:
    line = LineString([(s, -10.0), (s, 10.0)]).intersection(poly)
    ys = np.asarray(line.coords)[:, 1] if hasattr(line, "coords") else np.concatenate(
        [np.asarray(g.coords)[:, 1] for g in line.geoms])
    return float(ys.max()), float(ys.min())


def hinge_axis(surf: LiftingSurface, eta0: float, eta1: float, hinge_fn) -> tuple[np.ndarray, np.ndarray]:
    """3-D hinge-line points at ``eta0`` and ``eta1`` for a ``hinge_fn`` from :func:`control_surface_regions`."""
    pts = []
    for eta in (eta0, eta1):
        poly, fr = section2d(surf, eta)
        s, t = hinge_fn(poly, surf.chord_at(eta))
        pts.append(to3d(fr, np.array([[s, t]]))[0])
    return pts[0], pts[1]


# =====================================================================================================================
# fuselage
# =====================================================================================================================
def fuselage_section2d(fus: Fuselage, x: float, inset: float = 0.0, n: int = 192) -> Polygon:
    """OML cross-section at station ``x`` as a (y, z) polygon, optionally inset by ``inset`` (mitred)."""
    ph = np.linspace(0.0, 2 * math.pi, n, endpoint=False)
    P = fus.point(np.full_like(ph, x), ph)
    poly = Polygon(P[:, 1:3]).buffer(0)
    if inset > 0:
        poly = poly.buffer(-inset, join_style=2)
    return largest(poly)


def bulkhead(fus: Fuselage, x: float, t: float, inset: float, cutouts=(), centered: bool = True) -> Mesh:
    """Planar frame plate at station ``x`` (normal +X), outline = OML inset by ``inset``; ``cutouts`` is a list of
    shapely polygons in (y, z) to subtract (pass-throughs, lightening holes, the ring interior of a ring frame)."""
    reg = fuselage_section2d(fus, x, inset)
    for c in cutouts:
        reg = reg.difference(c)
    reg = largest(reg) if not isinstance(reg, Polygon) else reg
    return extrude(reg, t, origin=(x, 0.0, 0.0), u=(0.0, 1.0, 0.0), v=(0.0, 0.0, 1.0), centered=centered)


def ring_frame(fus: Fuselage, x: float, t: float, inset: float, depth: float, cutouts=()) -> Mesh:
    """Ring frame: band of radial ``depth`` inside the OML inset ``inset`` (open centre for bays)."""
    outer = fuselage_section2d(fus, x, inset)
    inner = outer.buffer(-depth, join_style=2)
    reg = outer.difference(inner)
    for c in cutouts:
        reg = reg.difference(c)
    return extrude(largest(reg), t, origin=(x, 0.0, 0.0), u=(0.0, 1.0, 0.0), v=(0.0, 0.0, 1.0), centered=True)


def fuselage_panel(fus: Fuselage, x0: float, x1: float, phi0: float, phi1: float, t: float, nx: int = 40,
                   nphi: int = 40, offset: float = 0.0) -> Mesh:
    """Skin panel (closed solid) on the OML between stations x0..x1 and polar angles phi0..phi1 (0 = top, +π/2 =
    starboard side), thickened inward by ``t``; ``offset`` moves the outer face inward (e.g. a doubler under a skin)."""
    P = fus.grid(x0, x1, phi0, phi1, nx, nphi)
    if offset:
        N = fus.normal(P[..., 0], np.linspace(phi0, phi1, nphi)[None, :].repeat(nx, 0))
        P = P - offset * N
    return shell_from_grid(P, t)


def fuselage_point(fus: Fuselage, x: float, phi: float, inset: float = 0.0) -> np.ndarray:
    """OML point (moved inward along the surface normal by ``inset``)."""
    p = fus.point(x, phi)
    return p - inset * fus.normal(x, phi) if inset else p
