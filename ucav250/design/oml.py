"""Outer mould line (OML) evaluators — shared math, registers no parts.

Fuselage
    Defined by a station table ``[[x, width, height, zc, n_top, n_bot], ...]`` (metres; superellipse exponents for
    the upper and lower halves, 2 = ellipse, larger = boxier/chined). Sections are interpolated with monotone cubic
    (PCHIP) curves, so the surface is smooth and never overshoots. Surface parameter ``phi`` is measured from the top
    centre line (+Z) toward starboard (+Y): phi = 0 top, pi/2 starboard maximum-width line, pi bottom, 3pi/2 port.

Lifting surfaces (wing, tail, fins, V-tails)
    Defined by spanwise sections ``[{y, x_le, z_le, chord, twist_deg, airfoil}]`` along a reference line whose local
    span direction may tilt (dihedral, V-tail, vertical fin). Each section lies in the plane spanned by the chord
    direction (+X, rotated by twist about the span direction) and the local "up" normal. Airfoils are read from
    ``ucav250/data/airfoils/<name>.dat`` (Selig format) and resampled with cosine spacing; a finite trailing edge
    (``te_thickness``) keeps every section manufacturable.

Everything returns plain numpy arrays or closed :class:`ucav250.core.geom.Mesh` objects.
"""
from __future__ import annotations

import functools
import math
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np
from scipy.interpolate import PchipInterpolator

from ..core.geom import EPS, Mesh, fix_orientation, loft, unit

AIRFOIL_DIR = Path(__file__).resolve().parents[1] / "data" / "airfoils"


# =====================================================================================================================
# airfoils
# =====================================================================================================================
@functools.lru_cache(maxsize=None)
def read_airfoil(name: str) -> np.ndarray:
    """Selig-format coordinates (TE -> upper -> LE -> lower -> TE), normalised to chord 1 with LE at (0, 0)."""
    path = AIRFOIL_DIR / f"{name}.dat"
    if not path.exists():
        alt = AIRFOIL_DIR / f"{name.lower()}.dat"
        path = alt if alt.exists() else path
    rows = []
    for line in path.read_text(errors="ignore").splitlines():
        parts = line.replace(",", " ").split()
        if len(parts) >= 2:
            try:
                x, y = float(parts[0]), float(parts[1])
            except ValueError:
                continue
            if abs(x) <= 1.5 and abs(y) <= 1.0:
                rows.append((x, y))
    P = np.array(rows, float)
    if len(P) < 10:
        raise ValueError(f"airfoil {name}: too few points in {path}")
    # Lednicer format (upper LE->TE then lower LE->TE) -> convert to Selig
    if P[0, 0] < 0.5 and P[-1, 0] > 0.5:
        i = int(np.argmax(np.diff(P[:, 0]) < -0.5)) + 1 if np.any(np.diff(P[:, 0]) < -0.5) else len(P) // 2
        up, lo = P[:i], P[i:]
        P = np.vstack([up[::-1], lo[1:] if np.allclose(lo[0], up[0]) else lo])
    ile = int(np.argmin(P[:, 0]))
    le = P[ile].copy()
    P = P - le
    c = P[:, 0].max()
    return P / c


def _naca4(code: str, n: int = 121) -> np.ndarray:
    m, p, t = int(code[0]) / 100, int(code[1]) / 10, int(code[2:]) / 100
    x = 0.5 * (1 - np.cos(np.linspace(0, math.pi, n)))
    yt = 5 * t * (0.2969 * np.sqrt(x) - 0.1260 * x - 0.3516 * x ** 2 + 0.2843 * x ** 3 - 0.1036 * x ** 4)
    if m == 0:
        yc = np.zeros_like(x)
        dy = np.zeros_like(x)
    else:
        yc = np.where(x < p, m / p ** 2 * (2 * p * x - x ** 2), m / (1 - p) ** 2 * ((1 - 2 * p) + 2 * p * x - x ** 2))
        dy = np.where(x < p, 2 * m / p ** 2 * (p - x), 2 * m / (1 - p) ** 2 * (p - x))
    th = np.arctan(dy)
    xu, yu = x - yt * np.sin(th), yc + yt * np.cos(th)
    xl, yl = x + yt * np.sin(th), yc - yt * np.cos(th)
    return np.vstack([np.column_stack([xu, yu])[::-1], np.column_stack([xl, yl])[1:]])


def airfoil_coords(name: str) -> np.ndarray:
    """Coordinates for a .dat name or a built-in ``naca0012``-style 4-digit code (no file needed)."""
    key = name.lower().replace(" ", "").replace("-", "")
    if key.startswith("naca") and len(key) == 8 and key[4:].isdigit() and not (AIRFOIL_DIR / f"{name}.dat").exists():
        return _naca4(key[4:])
    return read_airfoil(name)


@functools.lru_cache(maxsize=None)
def resampled(name: str, n: int = 60, te_thickness: float = 0.004, thickness_scale: float = 1.0) -> tuple:
    """(x, y_upper, y_lower) at ``n`` cosine-spaced chord stations (unit chord). The trailing edge is opened to
    ``te_thickness`` (fraction of chord) by a linear thickness ramp over the aft 30 %, and the section thickness may be
    scaled (``thickness_scale``) about the camber line."""
    P = airfoil_coords(name)
    ile = int(np.argmin(P[:, 0]))
    up = P[: ile + 1][::-1]                 # LE -> TE
    lo = P[ile:]                            # LE -> TE
    up = up[np.argsort(up[:, 0], kind="stable")]
    lo = lo[np.argsort(lo[:, 0], kind="stable")]
    up = up[np.r_[True, np.diff(up[:, 0]) > 1e-9]]
    lo = lo[np.r_[True, np.diff(lo[:, 0]) > 1e-9]]
    x = 0.5 * (1 - np.cos(np.linspace(0, math.pi, n)))
    yu = PchipInterpolator(up[:, 0], up[:, 1])(x)
    yl = PchipInterpolator(lo[:, 0], lo[:, 1])(x)
    camber = 0.5 * (yu + yl)
    half = 0.5 * (yu - yl) * thickness_scale
    ramp = np.clip((x - 0.7) / 0.3, 0.0, 1.0)
    half = np.maximum(half, 0.0) + 0.5 * te_thickness * ramp
    half[0] = 0.0
    return x, camber + half, camber - half


def max_thickness(name: str) -> tuple[float, float]:
    """(t/c, x/c at max thickness) of the resampled section."""
    x, yu, yl = resampled(name, 200, 0.0)
    t = yu - yl
    i = int(np.argmax(t))
    return float(t[i]), float(x[i])


# =====================================================================================================================
# fuselage
# =====================================================================================================================
@dataclass
class Fuselage:
    stations: np.ndarray                    # (n, 6): x, width, height, zc, n_top, n_bot

    def __post_init__(self):
        S = np.asarray(self.stations, float)
        if S.ndim != 2 or S.shape[1] != 6:
            raise ValueError("fuselage stations must be (n, 6): x, width, height, zc, n_top, n_bot")
        if np.any(np.diff(S[:, 0]) <= 0):
            raise ValueError("fuselage stations must have increasing x")
        self.stations = S
        self._f = {k: PchipInterpolator(S[:, 0], S[:, i]) for i, k in enumerate(["x", "w", "h", "zc", "nt", "nb"])
                   if k != "x"}

    @property
    def x0(self) -> float:
        return float(self.stations[0, 0])

    @property
    def x1(self) -> float:
        return float(self.stations[-1, 0])

    def section(self, x):
        """(width, height, zc, n_top, n_bot) at x. A zero-size first/last station is closed with an elliptical arc
        over the adjacent interval (round nose / round tail end, tangent-continuous at the next station)."""
        x = np.clip(np.asarray(x, float), self.x0, self.x1)
        w = np.maximum(self._f["w"](x), 0.0)
        h = np.maximum(self._f["h"](x), 0.0)
        S = self.stations
        for end, (i0, i1) in (("nose", (0, 1)), ("tail", (-1, -2))):
            if S[i0, 1] < 1e-9 and S[i0, 2] < 1e-9:
                xa, xb = S[i0, 0], S[i1, 0]
                lo, hi = min(xa, xb), max(xa, xb)
                m = (x >= lo) & (x <= hi)
                if np.any(m):
                    u = np.clip(np.abs(x[m] - xa) / abs(xb - xa), 0.0, 1.0)
                    f = np.sqrt(np.clip(1.0 - (1.0 - u) ** 2, 0.0, 1.0))
                    w = np.array(w, float, copy=True)
                    h = np.array(h, float, copy=True)
                    w[m] = S[i1, 1] * f
                    h[m] = S[i1, 2] * f
        return w, h, self._f["zc"](x), self._f["nt"](x), self._f["nb"](x)

    def point(self, x, phi) -> np.ndarray:
        """Surface point(s); x and phi broadcast."""
        x, phi = np.broadcast_arrays(np.asarray(x, float), np.asarray(phi, float))
        w, h, zc, nt, nb = self.section(x)
        s, c = np.sin(phi), np.cos(phi)
        n = np.where(c >= 0, nt, nb)
        y = 0.5 * w * np.sign(s) * np.abs(s) ** (2.0 / n)
        z = zc + 0.5 * h * np.sign(c) * np.abs(c) ** (2.0 / n)
        return np.stack([x, y, z], axis=-1)

    def grid(self, x0, x1, phi0, phi1, nx: int, nphi: int, x_spacing: str = "linear") -> np.ndarray:
        """Structured patch (nx, nphi, 3): rows along x, columns along phi (phi increasing from the top toward
        starboard). dP/dx x dP/dphi is the OUTWARD normal, so ``geom.shell_from_grid(P, t)`` thickens inward."""
        if x_spacing == "cosine":
            xs = x0 + (x1 - x0) * 0.5 * (1 - np.cos(np.linspace(0, math.pi, nx)))
        else:
            xs = np.linspace(x0, x1, nx)
        ph = np.linspace(phi0, phi1, nphi)
        X, PH = np.meshgrid(xs, ph, indexing="ij")
        return self.point(X, PH)

    def normal(self, x, phi, d: float = 1e-5) -> np.ndarray:
        """Unit outward normal (numerical)."""
        p_x = (self.point(x + d, phi) - self.point(x - d, phi)) / (2 * d)
        p_f = (self.point(x, phi + d) - self.point(x, phi - d)) / (2 * d)
        n = np.cross(p_x, p_f)
        return n / np.maximum(np.linalg.norm(n, axis=-1, keepdims=True), EPS)

    def mesh(self, nx: int = 120, nphi: int = 96, x0: float | None = None, x1: float | None = None) -> Mesh:
        """Closed OML solid between x0 and x1 (defaults: full length). Zero-size end sections become poles."""
        x0 = self.x0 if x0 is None else x0
        x1 = self.x1 if x1 is None else x1
        xs = x0 + (x1 - x0) * 0.5 * (1 - np.cos(np.linspace(0, math.pi, nx)))
        ph = np.linspace(0, 2 * math.pi, nphi, endpoint=False)
        rings, poles = [], [None, None]
        for k, x in enumerate(xs):
            w, h, zc, *_ = self.section(x)
            if max(w, h) < 1e-5 and k in (0, len(xs) - 1):
                poles[0 if k == 0 else 1] = np.array([x, 0.0, float(zc)])
                continue
            rings.append(self.point(np.full_like(ph, x), ph))
        return tube_mesh(rings, poles[0], poles[1])


def tube_mesh(rings: list[np.ndarray], start_pole=None, end_pole=None) -> Mesh:
    """Closed solid through rings (each (k, 3), same k and orientation); ends closed by a pole vertex (fan) when
    given, otherwise by a flat cap (geom.loft)."""
    if start_pole is None and end_pole is None:
        return fix_orientation(loft(rings))
    k = len(rings[0])
    V = [np.vstack(rings)]
    F = []
    for i in range(len(rings) - 1):
        a = np.arange(i * k, (i + 1) * k)
        b = a + k
        a2, b2 = np.roll(a, -1), np.roll(b, -1)
        F += [np.column_stack([a, a2, b2]), np.column_stack([a, b2, b])]
    n = len(rings) * k
    if start_pole is not None:
        V.append(np.asarray(start_pole, float)[None])
        a = np.arange(k)
        F.append(np.column_stack([np.full(k, n), np.roll(a, -1), a]))
        n += 1
    else:
        from ..core.geom import _earcut
        ring = rings[0]
        c = ring.mean(0)
        _, _, vt = np.linalg.svd(ring - c)
        P2 = np.column_stack([(ring - c) @ vt[0], (ring - c) @ vt[1]])
        F.append(_earcut([P2])[:, ::-1])
    if end_pole is not None:
        V.append(np.asarray(end_pole, float)[None])
        a = np.arange((len(rings) - 1) * k, len(rings) * k)
        F.append(np.column_stack([np.full(k, n), a, np.roll(a, -1)]))
    else:
        from ..core.geom import _earcut
        ring = rings[-1]
        c = ring.mean(0)
        _, _, vt = np.linalg.svd(ring - c)
        P2 = np.column_stack([(ring - c) @ vt[0], (ring - c) @ vt[1]])
        F.append((len(rings) - 1) * k + _earcut([P2]))
    from ..core.geom import _orient_consistently
    m = _orient_consistently(Mesh(np.vstack(V), np.vstack(F)))
    return fix_orientation(m)


def fuselage_from_spec(spec: dict) -> Fuselage:
    return Fuselage(np.asarray(spec["fuselage"]["stations"], float))


# =====================================================================================================================
# lifting surfaces
# =====================================================================================================================
@dataclass
class LiftingSurface:
    """Spanwise sections, each ``{y, x_le, z_le, chord, twist_deg, airfoil, [span_dir]}`` where (x_le, y, z_le) is the
    leading-edge point. The local span direction is taken from neighbouring leading-edge points unless ``span_dir``
    is given; the local "up" is chord_dir x span_dir rotated so that, for a flat right wing, up = +Z."""

    sections: list[dict]
    n_chord: int = 60
    te_thickness: float = 0.0015             # absolute trailing-edge thickness (m), manufacturable TE
    name: str = "surface"
    mirror: bool = False                     # build the port side as well when meshing the full surface
    _cache: dict = field(default_factory=dict, repr=False)

    def __post_init__(self):
        self.sections = [dict(s) for s in self.sections]
        if len(self.sections) < 2:
            raise ValueError("lifting surface needs >= 2 sections")

    def _le(self, i):
        s = self.sections[i]
        return np.array([s["x_le"], s["y"], s["z_le"]], float)

    def _span_dir(self, i):
        s = self.sections[i]
        if "span_dir" in s:
            return unit(s["span_dir"])
        a = self._le(max(i - 1, 0))
        b = self._le(min(i + 1, len(self.sections) - 1))
        d = b - a
        d[0] = 0.0                               # sweep does not tilt the section plane
        return unit(d)

    def section_frame(self, i):
        """(origin = LE point, chord unit vector, up unit vector) of section ``i``."""
        s = self.sections[i]
        sd = self._span_dir(i)
        chord = np.array([1.0, 0.0, 0.0])
        up = np.cross(chord, sd)                 # flat right wing: (1,0,0) x (0,1,0) = (0,0,1)
        up = unit(up)
        tw = math.radians(float(s.get("twist_deg", 0.0)))   # positive twist = leading edge up
        c, sn = math.cos(tw), math.sin(tw)
        chord_t = c * chord + sn * up
        up_t = -sn * chord + c * up
        return self._le(i), chord_t, up_t

    def section_points(self, i, n: int | None = None) -> np.ndarray:
        """Closed section loop (2n-2, 3): TE upper -> LE -> TE lower (no duplicate LE)."""
        s = self.sections[i]
        n = n or self.n_chord
        x, yu, yl = resampled(s["airfoil"], n, self.te_thickness / max(float(s["chord"]), 1e-6),
                              float(s.get("thickness_scale", 1.0)))
        o, cd, up = self.section_frame(i)
        ch = float(s["chord"])
        upper = o + (x[:, None] * cd + yu[:, None] * up) * ch
        lower = o + (x[:, None] * cd + yl[:, None] * up) * ch
        return np.vstack([upper[::-1], lower[1:]])

    def surface_point(self, i: int, xc: float, side: str) -> np.ndarray:
        """Point at chord fraction ``xc`` on the upper/lower surface of section ``i``."""
        s = self.sections[i]
        x, yu, yl = resampled(s["airfoil"], 400, self.te_thickness / max(float(s["chord"]), 1e-6),
                              float(s.get("thickness_scale", 1.0)))
        yy = np.interp(xc, x, yu if side == "upper" else yl)
        o, cd, up = self.section_frame(i)
        return o + (xc * cd + yy * up) * float(s["chord"])

    def mesh(self, refine: int = 1) -> Mesh:
        """Closed solid through all sections (root and tip capped). ``refine`` inserts linearly interpolated
        intermediate sections between neighbours for smoother lofts."""
        rings = []
        for i in range(len(self.sections) - 1):
            a = self.section_points(i)
            b = self.section_points(i + 1)
            for k in range(refine):
                t = k / refine
                rings.append((1 - t) * a + t * b)
        rings.append(self.section_points(len(self.sections) - 1))
        return fix_orientation(loft(rings))

    def interpolate_section(self, y: float) -> dict:
        """Section dict at span station ``y`` (linear in chord/LE/twist, airfoil of the nearer inboard section)."""
        ys = np.array([s["y"] for s in self.sections], float)
        if not (ys.min() - 1e-9 <= y <= ys.max() + 1e-9):
            raise ValueError(f"{self.name}: y={y} outside [{ys.min()}, {ys.max()}]")
        j = int(np.clip(np.searchsorted(ys, y) - 1, 0, len(ys) - 2))
        a, b = self.sections[j], self.sections[j + 1]
        t = (y - a["y"]) / max(b["y"] - a["y"], EPS)
        out = {k: (1 - t) * float(a[k]) + t * float(b[k]) for k in ("y", "x_le", "z_le", "chord", "twist_deg")}
        out["airfoil"] = a["airfoil"] if t < 0.5 else b["airfoil"]
        if "thickness_scale" in a or "thickness_scale" in b:
            out["thickness_scale"] = (1 - t) * a.get("thickness_scale", 1.0) + t * b.get("thickness_scale", 1.0)
        return out

    def split(self, y_cuts: list[float]) -> list["LiftingSurface"]:
        """Spanwise pieces between consecutive cuts (inclusive ends), e.g. for panels or control-surface spans."""
        ys = sorted(set([self.sections[0]["y"], *y_cuts, self.sections[-1]["y"]]))
        out = []
        for a, b in zip(ys, ys[1:]):
            secs = [self.interpolate_section(a)] + [s for s in self.sections if a < s["y"] < b] + \
                   [self.interpolate_section(b)]
            out.append(LiftingSurface(secs, self.n_chord, self.te_thickness, f"{self.name}[{a:.3f},{b:.3f}]"))
        return out


def mean_aerodynamic_chord(sections: list[dict]) -> dict:
    """MAC length, spanwise station and LE x of a (half) planform given by sections (trapezoid integration)."""
    y = np.array([s["y"] for s in sections], float)
    c = np.array([s["chord"] for s in sections], float)
    xle = np.array([s["x_le"] for s in sections], float)
    yy = np.linspace(y.min(), y.max(), 2001)
    cc = np.interp(yy, y, c)
    xx = np.interp(yy, y, xle)
    area = np.trapz(cc, yy)
    mac = np.trapz(cc ** 2, yy) / area
    ymac = np.trapz(cc * yy, yy) / area
    xmac = np.trapz(cc * xx, yy) / area
    return {"half_area": float(area), "mac": float(mac), "y_mac": float(ymac), "x_le_mac": float(xmac)}
