"""YELKOVAN YK-38 — gövde geometrisi: saf numpy yüzey ağları (bpy GEREKTİRMEZ).

Bu modül ``params``'tan gelen tanımları (gövde kesitleri, kanat/kuyruk kesitleri, menteşe hatları, takım
kuyuları, pervane, taret…) **kapalı, yönlendirilmiş (manifold) çokgen ağlara** çevirir. Blender tarafı
(``ucav/blender/airframe.py``) yalnızca bu ağları nesneye dönüştürür; baskı tarafı aynı ağları dilimleyebilir.

Kurallar
--------
* Her parça bir ``MeshData`` döndürür: köşeler ``verts`` (``space="spec"`` → gövde ekseni ``(s, y, z)``,
  ``space="local"`` → nesnenin yerel Blender ekseni), çokgenler ``faces`` (dörtgen/üçgen/n-gen), yüz başına
  malzeme indisi ``face_mat`` → ``mats`` (``UM_*`` adları, ``params.MATERIALS``), keskin kenarlar.
* Ağlar kapalıdır (her kenar iki yüzde, zıt yönlü); dış normaller ``oriented()`` ile garanti edilir
  (Blender ekseninde işaretli hacim > 0). ``check()`` kenar raporu verir.
* Loft düzeni: aynı nokta sayılı halkalar (kesitler) sıra sıra bağlanır; uçlar n-gen kapak, kutup yelpazesi
  ya da "fermuar" (kalınlığı sıfıra inen uç kesitte üst/alt noktaların birleştirilmesi) ile kapanır.

Ana fonksiyonlar (sol/iskele için ``side="L"``, sağ için ``"R"``):

* Gövde: ``fuselage()``, ``cowl()`` (gerçek 1,6 mm kabuk, arkası Ø92 açık; ``cowl(print_solid=True)`` baskı için
  kapalı katı), ``cowl_cavity()`` (yalnız render koyu boşluk diski), ``exhaust_ring()``, ``intake()``, ``hatch()``,
  ``stripe(side)``, ``chevron()``, ``cowl_louvers()``, ``muffler_pipe()``, ``scuff_pad()``; halka düzeni ``FUS_RING`` /
  ``fuselage_ring_yz(s)`` (chine pahı ve bant kenarları sabit indislerde).
* Motor bölmesi (R01/R05): ``engine_bay_clearance()`` (zarfların kabuğa 3B payı), ``cooling_flow_areas()`` (NACA ağzı,
  lüle, flanş açıklığı, çene yarığı), ``cooling_exit_check()`` (çene yarığı kabuğu boydan boya keser mi),
  ``cowl_aft_opening()`` / ``cooling_exit_cutter()`` / ``muffler_pipe_hole()`` (baskı delikleri), ``muffler_pipe_axis()``,
  ``engine_keepouts()`` (baskı kaporta parçalarından çıkarılacak yuvarlak 5 mm / 15,5 mm motor yasak bölgeleri).
* Taşıyıcı yüzeyler: ``wing_parts(side)`` → ``{"U_WingCenter_L": MeshData, …, "U_FlapIn_L": MeshData, …}``,
  ``stab_parts()``, ``fin_parts(side)``. Kumanda yüzeyleri yuvarlak burunlu, sabit tarafta eş eksenli oyuk ve
  ``params`` menteşe aralığı (1 mm) ile ayrı ağlardır. Kanatta yüzey uç kaburgaları ve boşluk duvarları menteşe
  eksenine diktir (dihedral/ok nedeniyle dönüşte uç kayması yok, aralık her açıda 1 mm); kuyrukta uçlar akım
  yönündedir ve uç boşluğu rig aralığındaki dönüş zarfından hesaplanır (``surface_end_gaps``, 2,5–3,2 mm).
* Birleşimler: ``wing_fillet(side)`` (kanat üstü — chine'dan kanada akan fileto), ``root_fairing(side)``
  (kuyuları taşıyan düz tabanlı kök kaportası), ``unit_blister(side)``, ``stab_fillet()``.
* İtki: ``prop()`` (yerel: X = mil ekseni geriye, pala 1 = +Z), ``spinner()``.
* Taret (yerel, top merkezli): ``turret_mount()``, ``turret_pan()``, ``turret_ball()``, ``turret_windows()``,
  ``turret_ring()``.
* Takım: ``gear_opening(leg)`` (kapak açıklığı çokgeni, testere dişli), ``gear_well_outline(leg)`` (dişsiz kuyu
  zarfı), ``gear_cutters(leg)`` (iki boolean kesici: dişli deri dudağı + düz duvarlı kuyu hacmi),
  ``gear_bay(leg)`` (turuncu kuyu astarı), ``gear_door(name)`` (deri parçası kapak, kalınlık 1,2 mm);
  ``turret_cutter()``, ``hatch_cutter()`` (taret yuvası ve kapak altı cebi kesicileri).
* Ayrıntılar: ``blade_antenna(feature)``, ``gnss_puck(feature)``, ``dipole(feature)``, ``pitot_tube()``,
  ``nav_light(feature)``, ``strobe_light(feature)``, ``landing_light()``.

Çalıştırma: ``python3 ucav/shapes.py`` bütün parçaları üretir ve kapalılık/nokta sayısı raporu yazar.
"""
from __future__ import annotations

import functools
import math
import re
from dataclasses import dataclass, field
from typing import Callable, Iterable, Sequence

import numpy as np

try:                                    # paket olarak (from ucav import shapes) ya da ucav/ dizininden
    from . import params as P
except ImportError:                     # pragma: no cover
    import params as P  # type: ignore[no-redef]

_rad, _deg = math.radians, math.degrees
_F, _W, _T, _G, _PR = P.SPEC["fuselage"], P.SPEC["wing"], P.SPEC["tail"], P.SPEC["landing_gear"], P.SPEC["propulsion"]
_D = P.SPEC["details"]
M = P.MATERIAL_ROLES                    # rol → UM_* adı

# Görsel çözünürlük ve küçük modelleme payları (geometri tanımı değil; ağ yoğunluğu ve gizli gömme payları)
FUS_NQ_DENSE = 400                      # kesit çeyreği yoğun örnekleme (yeniden örnekleme kaynağı)
EMBED = 0.003                           # fileto/kaporta katılarının gövdeye ve kanada gömülme payı (görünmez)
DOOR_GAP = 0.0005                       # kapak çevresi panel çizgisi (her yanda)
DOOR_T = float(next(z["wall_mm"] for z in P.SPEC["print"]["zones"] if z["zone"] == "gear_doors")) / 1000.0
HATCH_T = float(_D["hatch"]["wall_m"])


# =====================================================================================================
# Ağ veri yapısı
# =====================================================================================================
@dataclass
class MeshData:
    """Bir nesnenin çokgen ağı (bkz. modül açıklaması).

    ``smooth_angle``: None → düz (fasetli) gölgeleme; sayı → yumuşak gölgeleme + bu açının üstündeki kenarlar
    keskin. ``sharp_edges``: ayrıca keskin işaretlenecek köşe çiftleri. ``closed``: kapalı katı beklenir.
    """

    name: str
    verts: np.ndarray
    faces: list
    face_mat: np.ndarray
    mats: list
    space: str = "spec"
    smooth_angle: float | None = 32.0
    sharp_edges: list = field(default_factory=list)
    closed: bool = True

    # ------------------------------------------------------------------ dönüşümler
    def verts_blender(self) -> np.ndarray:
        """Köşeler Blender ekseninde (``space="spec"`` ise X = −s)."""
        return P.points_to_blender(self.verts) if self.space == "spec" else np.array(self.verts, float)

    def signed_volume(self) -> float:
        """Blender ekseninde işaretli hacim (dış normaller → pozitif)."""
        V = self.verts_blender()
        vol = 0.0
        for f in self.faces:
            a = V[f[0]]
            for i in range(1, len(f) - 1):
                vol += float(np.dot(a, np.cross(V[f[i]], V[f[i + 1]])))
        return vol / 6.0

    def oriented(self) -> "MeshData":
        """Kapalı ağda yüz yönlerini dışa çevirir (hacim negatifse bütün yüzleri ters çevirir)."""
        if self.closed and self.signed_volume() < 0:
            self.faces = [tuple(reversed(f)) for f in self.faces]
        return self

    def check(self) -> dict:
        """Kenar raporu: ``boundary`` (tek yüzlü), ``nonmanifold`` (>2 yüz), ``flipped`` (aynı yönde iki kez
        kullanılan kenar), ``degenerate`` (2'den az farklı köşeli yüz), ``n_verts``, ``n_faces``, ``volume``."""
        cnt: dict = {}
        directed: dict = {}
        deg = 0
        for f in self.faces:
            if len(set(f)) < 3:
                deg += 1
            for i in range(len(f)):
                a, b = f[i], f[(i + 1) % len(f)]
                key = (a, b) if a < b else (b, a)
                cnt[key] = cnt.get(key, 0) + 1
                directed[(a, b)] = directed.get((a, b), 0) + 1
        boundary = sum(1 for v in cnt.values() if v == 1)
        nonman = sum(1 for v in cnt.values() if v > 2)
        flipped = sum(1 for v in directed.values() if v > 1)
        return {"name": self.name, "n_verts": int(len(self.verts)), "n_faces": len(self.faces), "boundary": boundary,
                "nonmanifold": nonman, "flipped": flipped, "degenerate": deg,
                "volume": self.signed_volume() if self.closed else 0.0}

    def bbox(self) -> tuple[np.ndarray, np.ndarray]:
        V = np.asarray(self.verts, float)
        return V.min(axis=0), V.max(axis=0)


class MeshBuilder:
    """Halka/şerit tabanlı ağ kurucu (indisler ``add`` ile alınır, yüzler malzeme adıyla eklenir)."""

    def __init__(self, name: str, space: str = "spec"):
        self.name, self.space = name, space
        self._chunks: list[np.ndarray] = []
        self._n = 0
        self.faces: list[tuple] = []
        self.fmat: list[int] = []
        self.mats: list[str] = []
        self.sharp: list[tuple[int, int]] = []

    def mat(self, name: str) -> int:
        if name not in self.mats:
            self.mats.append(name)
        return self.mats.index(name)

    def add(self, pts) -> np.ndarray:
        """Noktaları ekler; aynı biçimde (son eksen hariç) indis dizisi döndürür."""
        p = np.asarray(pts, float)
        flat = p.reshape(-1, 3)
        idx = np.arange(self._n, self._n + len(flat))
        self._chunks.append(flat)
        self._n += len(flat)
        return idx.reshape(p.shape[:-1])

    def face(self, idx, mat: str) -> None:
        f = [int(i) for i in idx]
        out = []
        for i in f:                                    # ardışık tekrarları at (fermuarlı uçlar)
            if not out or out[-1] != i:
                out.append(i)
        while len(out) > 1 and out[0] == out[-1]:
            out.pop()
        if len(set(out)) >= 3 and len(set(out)) == len(out):
            self.faces.append(tuple(out))
            self.fmat.append(self.mat(mat))

    def strip(self, a, b, mats, closed: bool = True) -> None:
        """İki halka arasında dörtgen şerit: (a_j, a_j+1, b_j+1, b_j). ``mats``: ad, ad dizisi ya da f(j)."""
        m = len(a)
        n = m if closed else m - 1
        for j in range(n):
            j2 = (j + 1) % m
            mt = mats if isinstance(mats, str) else (mats(j) if callable(mats) else mats[j])
            self.face((a[j], a[j2], b[j2], b[j]), mt)

    def loft(self, rings: Sequence, mats, closed: bool = True) -> None:
        for k in range(len(rings) - 1):
            self.strip(rings[k], rings[k + 1], mats, closed)

    def cap(self, ring, mat: str, start: bool) -> None:
        """Halka kapağı: başlangıç halkası için ters sıra, bitiş halkası için düz sıra (loft yönüyle tutarlı)."""
        self.face(list(ring)[::-1] if start else list(ring), mat)

    def fan(self, pole: int, ring, mat: str, start: bool) -> None:
        m = len(ring)
        for j in range(m):
            j2 = (j + 1) % m
            self.face((pole, ring[j2], ring[j]) if start else (pole, ring[j], ring[j2]), mat)

    def build(self, smooth_angle: float | None = 32.0, closed: bool = True) -> MeshData:
        V = np.vstack(self._chunks) if self._chunks else np.zeros((0, 3))
        md = MeshData(self.name, V, self.faces, np.asarray(self.fmat, int), list(self.mats), self.space,
                      smooth_angle, list(self.sharp), closed)
        return md.oriented()


def merge(name: str, parts: Sequence[MeshData], smooth_angle: float | None = None) -> MeshData:
    """Aynı uzaydaki ağları tek nesnede birleştirir (her biri ayrı kapalı kabuk olarak kalır)."""
    V, F, FM, mats, sharp = [], [], [], [], []
    off = 0
    for p in parts:
        remap = []
        for mname in p.mats:
            if mname not in mats:
                mats.append(mname)
            remap.append(mats.index(mname))
        V.append(np.asarray(p.verts, float))
        F += [tuple(i + off for i in f) for f in p.faces]
        FM += [remap[i] for i in p.face_mat]
        sharp += [(a + off, b + off) for a, b in p.sharp_edges]
        off += len(p.verts)
    sa = parts[0].smooth_angle if smooth_angle is None else smooth_angle
    return MeshData(name, np.vstack(V), F, np.asarray(FM, int), mats, parts[0].space, sa, sharp, all(p.closed for p in parts))


# =====================================================================================================
# Genel yardımcılar
# =====================================================================================================
def _smoothstep(t):
    return P.smoothstep(t)


def _window(t, edge: float = 0.3, edge_end: float | None = None):
    """[0, 1] aralığında 0 → 1 → 0 yumuşak pencere (kenarlarda ``edge`` genişliğinde smoothstep; ``edge_end`` verilirse
    bitiş kenarı ayrı genişlikte)."""
    t = np.asarray(t, float)
    e1 = edge if edge_end is None else edge_end
    return _smoothstep(t / edge) * _smoothstep((1.0 - t) / e1) * ((t >= 0) & (t <= 1))


def _arc(curve: np.ndarray) -> np.ndarray:
    return np.r_[0.0, np.cumsum(np.hypot(*np.diff(curve, axis=0).T))]


def _at_arc(curve: np.ndarray, cum: np.ndarray, d) -> np.ndarray:
    d = np.asarray(d, float)
    return np.stack([np.interp(d, cum, curve[:, k]) for k in range(curve.shape[1])], axis=-1)


def _unit(v) -> np.ndarray:
    a = np.asarray(v, float)
    n = np.linalg.norm(a, axis=-1, keepdims=True)
    return a / np.where(n > 0, n, 1.0)


def _frame_from_axes(x_axis, z_hint) -> np.ndarray:
    """X ekseni verilen, Z'si ``z_hint``'e en yakın sağ el 3×3 (sütunlar X, Y, Z)."""
    X = _unit(x_axis)
    Z = np.asarray(z_hint, float) - X * float(np.dot(z_hint, X))
    if np.linalg.norm(Z) < 1e-9:
        Z = np.cross(X, [0.0, 1.0, 0.0])
    Z = _unit(Z)
    Y = np.cross(Z, X)
    return np.column_stack([X, Y, Z])


def lathe(profile: np.ndarray, nseg: int = 48, name: str = "lathe", mat: str | Callable = "UM_Steel",
          axis: str = "x", closed_profile: bool = False, space: str = "local",
          radius_fn: Callable | None = None, smooth_angle: float | None = 32.0) -> MeshData:
    """Döndürülmüş yüzey: ``profile`` (k, 2) ``(a, r)`` — ``a`` eksen boyunca, ``r`` yarıçap (0 → kutup).

    ``axis`` yerel eksen ("x", "y", "z"). ``closed_profile`` True ise profil kapalı döngüdür (simit benzeri).
    ``radius_fn(theta, k)`` yarıçap çarpanı (fasetli yaka gibi). ``mat`` ad ya da f(k) (profil parçası başına).
    """
    prof = np.asarray(profile, float)
    th = np.linspace(0.0, 2 * math.pi, nseg, endpoint=False)
    ax = {"x": (0, 1, 2), "y": (1, 2, 0), "z": (2, 0, 1)}[axis]
    mb = MeshBuilder(name, space)
    rings: list = []
    poles: list = []
    for k, (a, r) in enumerate(prof):
        if r <= 1e-9 and not closed_profile:
            p = np.zeros(3)
            p[ax[0]] = a
            rings.append(None)
            poles.append(int(mb.add(p[None])[0]))
            continue
        rr = r * (radius_fn(th, k) if radius_fn else 1.0)
        pts = np.zeros((nseg, 3))
        pts[:, ax[0]] = a
        pts[:, ax[1]] = rr * np.cos(th)
        pts[:, ax[2]] = rr * np.sin(th)
        rings.append(mb.add(pts))
        poles.append(None)
    mf = (lambda k: mat) if isinstance(mat, str) else mat
    npf = len(prof)
    rng = range(npf) if closed_profile else range(npf - 1)
    for k in rng:
        k2 = (k + 1) % npf
        a, b = rings[k], rings[k2]
        if a is None and b is None:
            continue
        if a is None:
            mb.fan(poles[k], b, mf(k), start=True)
        elif b is None:
            mb.fan(poles[k2], a, mf(k), start=False)
        else:
            mb.strip(a, b, mf(k))
    if not closed_profile:
        if rings[0] is not None:
            mb.cap(rings[0], mf(0), start=True)
        if rings[-1] is not None:
            mb.cap(rings[-1], mf(npf - 2), start=False)
    return mb.build(smooth_angle)


def superellipse_ring(a: float, zb: float, zt: float, n: float, m: int, yc: float = 0.0) -> np.ndarray:
    """(y, z) halka: yarı genişlik ``a``, alt/üst ``zb``/``zt``, üs ``n``; tepe noktasından başlar, iskeleye döner."""
    th = np.linspace(0.0, 2 * math.pi, m, endpoint=False)
    c, s = np.sin(th), np.cos(th)                     # th=0 → tepe; artan th → +y (iskele) → alt
    y = yc + a * np.sign(c) * np.abs(c) ** (2.0 / n)
    zc, b = 0.5 * (zb + zt), 0.5 * (zt - zb)
    z = zc + b * np.sign(s) * np.abs(s) ** (2.0 / n)
    return np.column_stack([y, z])


def poly_area(poly: np.ndarray) -> float:
    p = np.asarray(poly, float)
    return 0.5 * float(np.sum(p[:, 0] * np.roll(p[:, 1], -1) - np.roll(p[:, 0], -1) * p[:, 1]))


def poly_offset(poly: np.ndarray, d: float) -> np.ndarray:
    """2B çokgeni içe (``d`` > 0) ya da dışa öteler (köşelerde gönye, sınırlı)."""
    p = np.asarray(poly, float)
    if poly_area(p) < 0:
        return poly_offset(p[::-1], d)[::-1]
    e_prev = _unit(p - np.roll(p, 1, axis=0))
    e_next = _unit(np.roll(p, -1, axis=0) - p)
    n_prev = np.column_stack([-e_prev[:, 1], e_prev[:, 0]])     # CCW → sol normal içe
    n_next = np.column_stack([-e_next[:, 1], e_next[:, 0]])
    bis = _unit(n_prev + n_next)
    cosh = np.clip(np.sum(bis * n_next, axis=1), 0.35, 1.0)
    return p + bis * (d / cosh)[:, None]


def poly_resample(poly: np.ndarray, step: float) -> np.ndarray:
    """Kapalı çokgen kenarlarını en çok ``step`` uzunlukta parçalara böler (köşeler korunur)."""
    p = np.asarray(poly, float)
    out = []
    for i in range(len(p)):
        a, b = p[i], p[(i + 1) % len(p)]
        n = max(1, int(math.ceil(np.linalg.norm(b - a) / step)))
        for k in range(n):
            out.append(a + (b - a) * k / n)
    return np.asarray(out)


def poly_s_interval(poly: np.ndarray, y: float) -> tuple[float, float] | None:
    """``y`` doğrusu ile çokgen kesişiminin ``s`` aralığı (s-dışbükey çokgenler için)."""
    p = np.asarray(poly, float)
    ss = []
    for i in range(len(p)):
        (s0, y0), (s1, y1) = p[i], p[(i + 1) % len(p)]
        if abs(y1 - y0) < 1e-12:
            if abs(y - y0) < 1e-9:
                ss += [s0, s1]
            continue
        if min(y0, y1) - 1e-12 <= y <= max(y0, y1) + 1e-12:
            t = (y - y0) / (y1 - y0)
            ss.append(s0 + t * (s1 - s0))
    return (min(ss), max(ss)) if len(ss) >= 2 else None


def prism(name: str, poly: np.ndarray, z0: float, z1: float, mat: str, space: str = "spec") -> MeshData:
    """Plan çokgeninden (s, y) dikey prizma (boolean kesici): alt ``z0``, üst ``z1``."""
    p = np.asarray(poly, float)
    if poly_area(p) < 0:
        p = p[::-1]
    mb = MeshBuilder(name, space)
    bot = mb.add(np.column_stack([p, np.full(len(p), z0)]))
    top = mb.add(np.column_stack([p, np.full(len(p), z1)]))
    mb.strip(bot, top, mat)
    mb.cap(bot, mat, start=True)
    mb.cap(top, mat, start=False)
    return mb.build(None)


def heightfield_panel(name: str, poly: np.ndarray, z_out: Callable, thickness: float, mat_out: str, mat_in: str,
                      inset: float = 0.0, n_rows: int = 14, col_step: float = 0.004, up: float = 1.0,
                      z_in: Callable | None = None, smooth_angle: float | None = 40.0) -> MeshData:
    """Plan çokgenini (s, y; her y için tek s aralığı) izleyen ince panel: dış yüz ``z_out(s, y)``, iç yüz
    ``up·thickness`` kadar ötede (ya da ``z_in(s, y)``). ``inset`` çevre payı (panel çizgisi). Kapalı katı."""
    p = np.asarray(poly, float)
    ys_v = np.unique(np.round(p[:, 1], 9))
    y0, y1 = ys_v.min() + inset, ys_v.max() - inset
    cols = set(np.linspace(y0, y1, max(2, int(math.ceil((y1 - y0) / col_step)) + 1)).tolist())
    for v in ys_v:
        for dv in (0.0, -2e-4, 2e-4):
            if y0 < v + dv < y1:
                cols.add(float(v + dv))
    cols = np.array(sorted(cols))
    u = np.linspace(0.0, 1.0, n_rows)
    G = np.zeros((len(cols), n_rows, 2))
    for i, yv in enumerate(cols):
        iv = poly_s_interval(p, float(yv))
        a, b = (iv if iv else (0.0, 0.0))
        a, b = a + inset, b - inset
        if b < a:
            a = b = 0.5 * (a + b)
        G[i, :, 0] = a + u * (b - a)
        G[i, :, 1] = yv
    zo = np.array([[z_out(s, y) for s, y in row] for row in G])
    zi = zo + up * thickness if z_in is None else np.array([[z_in(s, y) for s, y in row] for row in G])
    outer = np.dstack([G, zo])
    inner = np.dstack([G, zi])
    mb = MeshBuilder(name)
    io = mb.add(outer)
    ii = mb.add(inner)
    nc, nr = io.shape
    for i in range(nc - 1):
        for j in range(nr - 1):
            mb.face((io[i, j], io[i + 1, j], io[i + 1, j + 1], io[i, j + 1]), mat_out)
            mb.face((ii[i, j], ii[i, j + 1], ii[i + 1, j + 1], ii[i + 1, j]), mat_in)
    ring_o = list(io[0, :]) + list(io[1:, -1]) + list(io[-1, -2::-1]) + list(io[-2:0:-1, 0])
    ring_i = list(ii[0, :]) + list(ii[1:, -1]) + list(ii[-1, -2::-1]) + list(ii[-2:0:-1, 0])
    mb.strip(ring_o, ring_i, mat_in)
    return mb.build(smooth_angle)


# =====================================================================================================
# Gövde: halka düzeni, gövde, kaporta, lüle halkası
# =====================================================================================================
@dataclass(frozen=True)
class RingLayout:
    """Gövde halkasında sabit konumlar (bütün istasyonlarda aynı).

    Sıra: tepe (0) → iskele üst yanak (``nu`` nokta) → bant üst kenarı → pah üstü → chine → pah altı → bant alt
    kenarı → alt yanak (``nl``) → karın (``keel``) → sancak ayna. Sancak konumu = ``m − k``.
    """

    nu: int = 20
    nl: int = 20

    @property
    def band_u(self) -> int: return self.nu + 1

    @property
    def bev_u(self) -> int: return self.nu + 2

    @property
    def chine(self) -> int: return self.nu + 3

    @property
    def bev_l(self) -> int: return self.nu + 4

    @property
    def band_l(self) -> int: return self.nu + 5

    @property
    def keel(self) -> int: return self.nu + self.nl + 6

    @property
    def m(self) -> int: return 2 * (self.nu + self.nl + 7) - 2

    def mirror(self, k: int) -> int:
        return (self.m - k) % self.m


FUS_RING = RingLayout()
CHINE_BAND_HALF = 0.5 * float(_D["markings"]["chine_band_m"])
CHINE_EDGE_R = float(_F["chine"]["edge_radius_m"])
BAND_TAPER_S = (0.98, 1.16)              # antrasit chine bandı burada sivrilerek biter (glove'a "ok ucu"). varsayım
NOSE_CAP_S = 0.020                       # antrasit burun ucu. varsayım


def _band_half(s: float) -> float:
    t = float(np.clip((s - BAND_TAPER_S[0]) / (BAND_TAPER_S[1] - BAND_TAPER_S[0]), 0.0, 1.0))
    return CHINE_BAND_HALF * (1.0 - 0.7 * t)


def fuselage_ring_yz(s: float, layout: RingLayout = FUS_RING) -> np.ndarray:
    """``s`` istasyonunda gövde halkası ``(m, 2)`` ``(y, z)``; chine'da 3 mm yarıçaplı pah satırları ve bant
    kenarları sabit konumlarda (malzeme ayrımı ve keskin ok çizgisi için)."""
    nq = FUS_NQ_DENSE
    sec = P.fuselage_section(s)
    if sec.width < 1e-6:
        return np.tile([0.0, sec.z_center], (layout.m, 1))
    o = P.fuselage_outline(s, nq)
    up = o[:nq][::-1]                              # chine → tepe
    lo = o[nq - 1:2 * nq - 1]                      # chine → karın
    cu, cl = _arc(up), _arc(lo)
    Lu, Ll = cu[-1], cl[-1]
    Lmin = min(Lu, Ll)
    b = min(_band_half(s), 0.20 * Lmin)
    du = _unit(up[3] - up[0])
    dl = _unit(lo[3] - lo[0])
    cos_int = float(np.clip(np.dot(du, dl), -1.0, 1.0))
    phi = math.pi - math.acos(cos_int)             # yön değişimi (sapma) açısı
    e = min(CHINE_EDGE_R * math.tan(0.5 * phi), 0.35 * b)
    e = max(e, 0.04 * b)
    shift = CHINE_EDGE_R * (1.0 / math.cos(0.5 * phi) - 1.0) if e < 0.35 * b else 0.0
    corner = up[0] + _unit(du + dl) * shift
    fu = (np.arange(1, layout.nu + 1) / (layout.nu + 1)) ** 1.0
    fl = (np.arange(1, layout.nl + 1) / (layout.nl + 1)) ** 1.0
    d_up = np.r_[Lu, b + (Lu - b) * fu[::-1], b, e]          # tepeden chine'a (chine'dan uzaklık)
    pts_up = _at_arc(up, cu, d_up)
    d_lo = np.r_[e, b, b + (Ll - b) * fl, Ll]
    pts_lo = _at_arc(lo, cl, d_lo)
    port = np.vstack([pts_up, corner[None], pts_lo])
    port[0, 0] = 0.0
    port[-1, 0] = 0.0
    stbd = port[-2:0:-1].copy()
    stbd[:, 0] *= -1.0
    ring = np.vstack([port, stbd])
    assert len(ring) == layout.m
    return ring


def fuselage_mat(s0: float, s1: float, j: int, layout: RingLayout = FUS_RING, band: bool = True) -> str:
    """``s0…s1`` arasındaki, halka konumu ``j → j+1`` olan yüzün malzemesi."""
    sm = 0.5 * (s0 + s1)
    if sm < NOSE_CAP_S:
        return M["accent"]
    k = j if j < layout.m // 2 else layout.mirror(j + 1)
    if band and sm < BAND_TAPER_S[1] and layout.band_u <= k < layout.band_l:
        return M["accent"]
    return M["skin_top"] if k < layout.chine else M["skin_bottom"]


def _ring3(s: float, yz: np.ndarray) -> np.ndarray:
    return np.column_stack([np.full(len(yz), s), yz[:, 0], yz[:, 1]])


def fuselage_s_stations(s_from: float = 0.0, s_to: float | None = None) -> np.ndarray:
    """Gövde loft istasyonları: burunda karesel sık, spec istasyonları ve modül/kaporta sınırları dahil."""
    L = P.FUSELAGE_LENGTH
    s_to = L if s_to is None else s_to
    u = np.linspace(0.0, 1.0, 165)
    base = L * (0.30 * u ** 2 + 0.70 * u)
    nose = 0.09 * np.linspace(0, 1, 22)[1:] ** 2
    cw = _PR["cowl"]
    keys = list(P.fuselage_stations()[:, 0]) + [0.40, 0.667, float(_F["modules"]["flange_s_m"]),
                                                 float(_F["modules"]["firewall_s_m"]),
                                                 float(_PR["exhaust_ring"]["s_from_m"]), NOSE_CAP_S,
                                                 BAND_TAPER_S[0], BAND_TAPER_S[1],
                                                 float(cw["cheek_left"]["s_from_m"]), float(cw["cheek_left"]["s_to_m"])]
    s_ring = float(_PR["exhaust_ring"]["s_from_m"])
    keys += list(np.linspace(cw["s_from_m"], s_ring, 24)) + list(np.linspace(s_ring - 0.018, s_ring, 13))
    allv = np.unique(np.round(np.r_[base, nose, keys], 6))
    allv = allv[(allv >= s_from - 1e-9) & (allv <= s_to + 1e-9)]
    keyset = set(np.round(keys, 6))
    out = [allv[0]]
    for v in allv[1:]:
        if v - out[-1] < 0.0015 and v not in keyset:
            continue
        if v - out[-1] < 0.0015 and out[-1] not in keyset:
            out[-1] = v
            continue
        out.append(v)
    return np.asarray(out)


def fuselage(band: bool = True) -> MeshData:
    """``U_Fuselage``: burun ucu (kutup) → yangın perdesi (s = 2,06). Üst/alt iki ton chine'da ayrılır,
    antrasit chine bandı burundan glove'a sivrilerek uzanır, burun ucu antrasit."""
    s_fw = float(_F["modules"]["firewall_s_m"])
    ss = fuselage_s_stations(0.0, s_fw)
    ss = ss[ss > 1e-6]
    mb = MeshBuilder("U_Fuselage")
    z0 = P.fuselage_section(0.0).z_center
    pole = int(mb.add(np.array([[0.0, 0.0, z0]]))[0])
    rings = [mb.add(_ring3(s, fuselage_ring_yz(s))) for s in ss]
    mb.fan(pole, rings[0], M["accent"], start=True)
    for k in range(len(rings) - 1):
        s0, s1 = ss[k], ss[k + 1]
        mb.strip(rings[k], rings[k + 1], lambda j, s0=s0, s1=s1: fuselage_mat(s0, s1, j, band=band))
    mb.cap(rings[-1], M["skin_bottom"], start=False)
    _mark_chine(mb, rings, ss)
    return mb.build(28.0)


def _mark_chine(mb: MeshBuilder, rings: list, ss: np.ndarray, layout: RingLayout = FUS_RING) -> None:
    """Chine pah satırlarını keskin işaretle (kırık belirgin olduğu yerde): ok çizgisi net okunur."""
    for k in range(len(rings) - 1):
        if P.fuselage_section(0.5 * (ss[k] + ss[k + 1])).crease < 0.45 or ss[k] < 0.004:
            continue
        for pos in (layout.bev_u, layout.bev_l):
            for p in (pos, layout.mirror(pos)):
                mb.sharp.append((int(rings[k][p]), int(rings[k + 1][p])))


def _ring_normals_yz(yz: np.ndarray) -> np.ndarray:
    """Saat yönündeki (tepe → iskele → karın) halkanın dış normalleri (y, z)."""
    t = np.roll(yz, -1, axis=0) - np.roll(yz, 1, axis=0)
    n = np.column_stack([-t[:, 1], t[:, 0]])
    return _unit(n)


KEEL_BLEND_M = 0.012                     # yanak kabartısı omurga çizgisine (y = 0) bu genişlikte yumuşakça söner: eski sert
                                        # yan maskesi (y·işaret > 0) sol kabartı karına indiği için kaporta altında,
                                        # omurgada ≈ 3 mm basamak bırakıyordu (s 2,06–2,19). varsayım


def _cheek_offset(s: float, yz: np.ndarray) -> np.ndarray:
    """Kaporta yanakları: sol susturucu kabartısı, sağ panjurlu yanak (dış normal boyunca yumuşak tümsek). Kabartı
    karında omurga çizgisine ``KEEL_BLEND_M`` genişliğinde smoothstep ile söner (basamaksız alt yüz)."""
    cw = _PR["cowl"]
    out = yz.copy()
    nrm = _ring_normals_yz(yz)
    for key, sg in (("cheek_left", 1.0), ("cheek_right", -1.0)):
        c = cw[key]
        ts = (s - float(c["s_from_m"])) / (float(c["s_to_m"]) - float(c["s_from_m"]))
        if not (0.0 < ts < 1.0):
            continue
        tz = (yz[:, 1] - float(c["z_from_m"])) / (float(c["z_to_m"]) - float(c["z_from_m"]))
        es = float(c.get("edge_s", 0.35))
        ez = float(c.get("edge_z", 0.40))
        keel = _smoothstep(yz[:, 0] * sg / KEEL_BLEND_M)
        w = float(_window(ts, float(c.get("edge_s_front", es)), float(c.get("edge_s_aft", es)))) \
            * _window(tz, float(c.get("edge_z_low", ez)), float(c.get("edge_z_high", ez))) * keel
        out += nrm * (w * float(c["bulge_m"]))[:, None]
    return out


def cowl_bottom_z(s: float, y: float) -> float | None:
    """Kaporta DIŞ alt yüzü yüksekliği (yanak kabartıları dahil) ``(s, y)``'de: ``cowl_section_yz`` çokgeninin
    düşey ``y`` çizgisiyle en alçak kesişimi; kaporta dışında gövde halkası. Kesişim yoksa None."""
    yz = cowl_section_yz(float(s))
    a, b = yz, np.roll(yz, -1, axis=0)
    zs = []
    for (y0, z0), (y1, z1) in zip(a, b):
        if (y0 - y) * (y1 - y) <= 0.0 and y0 != y1:
            zs.append(z0 + (y - y0) / (y1 - y0) * (z1 - z0))
    return float(min(zs)) if zs else None


def cowl_section_yz(s: float) -> np.ndarray:
    """Kaporta dış kesiti ``(m, 2)`` ``(y, z)`` (gövde halkası + yanak kabartıları); kaporta dışında gövde halkası."""
    yz = fuselage_ring_yz(s)
    return _cheek_offset(s, yz) if s >= float(_PR["cowl"]["s_from_m"]) - 1e-9 else yz


def _poly_signed_dist(poly: np.ndarray, pts: np.ndarray) -> np.ndarray:
    """2B kapalı çokgene işaretli uzaklık (içeride +)."""
    a = np.asarray(poly, float)
    b = np.roll(a, -1, axis=0)
    ab = b - a
    L2 = np.maximum((ab ** 2).sum(1), 1e-30)
    out = []
    for p in np.atleast_2d(pts):
        t = np.clip(((p - a) * ab).sum(1) / L2, 0.0, 1.0)
        q = a + t[:, None] * ab
        d = float(np.sqrt(((q - p) ** 2).sum(1)).min())
        # ışın sayımı (y + yönünde)
        y0, y1 = a[:, 1], b[:, 1]
        cross = ((y0 > p[1]) != (y1 > p[1]))
        xs = a[:, 0] + (p[1] - y0) / np.where(cross, y1 - y0, 1.0) * (b[:, 0] - a[:, 0])
        inside = int(np.sum(cross & (xs > p[0]))) % 2 == 1
        out.append(d if inside else -d)
    return np.asarray(out)


# -----------------------------------------------------------------------------------------------------
# Kaporta (R01): sahnede GERÇEK 1,6 mm kabuk — dış yüz, 3B normal boyunca içe ötelenmiş iç yüz, yangın perdesi
# düzleminde ön dudak ve arkada lüle halkasının iç çapı kadar AÇIK flanş (soğutma havası çıkışı; motor rulman burnu
# buradan geçer). Baskı için aynı dış yüzün kapalı katısı (``cowl(print_solid=True)``; printprep kabuğu kendisi
# oyar, arka açıklığı ``cowl_aft_opening`` ve çene yarığını ``cooling_exit_cutter`` ile delip açar). Koyu "boşluk"
# görünümü ayrı, yalnız render ağıdır (``cowl_cavity`` → ``U_Cowl_Cavity``; baskı, GLB ve çakışma denetimi dışı).
# -----------------------------------------------------------------------------------------------------
COWL_WALL = float(_PR["cowl"]["wall_m"])
COWL_CAVITY_GAP = 0.0004                 # koyu boşluk diski flanş arka yüzünün bu kadar gerisinde. varsayım
COWL_CAVITY_R_IN = 0.0230                # koyu disk iç yarıçapı: rulman burnu (Ø32; itki ekseni 5° eğik → halka merkezinden
                                        # 4,4 mm aşağıda) + 2,5 mm; iç delik spinnerin (Ø64) arkasında kalır. varsayım


def cowl_open_r() -> float:
    """Kaporta arka açıklığının yarıçapı = lüle halkası iç yarıçapı (s_end'de, itki göbeği yüksekliği merkezli)."""
    return 0.5 * float(_PR["exhaust_ring"]["id_m"])


def _cowl_s_range() -> tuple[float, float]:
    return float(_PR["cowl"]["s_from_m"]), float(_PR["exhaust_ring"]["s_from_m"])


@functools.lru_cache(maxsize=1)
def _cowl_grid() -> tuple[np.ndarray, np.ndarray]:
    """Kaporta dış yüzü: istasyonlar ``ss`` (K,) ve halkalar ``O`` (K, m, 3) — spec takımı, ``FUS_RING`` düzeni."""
    s0, s_end = _cowl_s_range()
    ss = fuselage_s_stations(s0, s_end)
    O = np.stack([_ring3(float(s), _cheek_offset(float(s), fuselage_ring_yz(float(s)))) for s in ss])
    return ss, O


@functools.lru_cache(maxsize=1)
def _cowl_inner() -> tuple[np.ndarray, np.ndarray]:
    """Kaporta iç yüzü: dış yüzün köşe normalleri (3B) boyunca ``COWL_WALL`` içe ötelenmiş noktaları her halka
    konumunda (j) s boyunca yeniden örneklenir → ``(s_in, I)``; ``I`` (K', m, 3). İlk halka yangın perdesi düzleminde
    (s0), son halka flanşın iç yüzünde (s_end − et). Dik çene kapanışında (s 2,178–2,186) et normal boyunca 1,6 mm
    kalır (düzlem içi ötelemede ≈ 0,4 mm'ye inerdi)."""
    ss, O = _cowl_grid()
    s0, s_end = _cowl_s_range()
    t = COWL_WALL
    K, m, _ = O.shape
    Ts = np.gradient(O, ss, axis=0)
    Tr = np.roll(O, -1, axis=1) - np.roll(O, 1, axis=1)
    n = _unit(np.cross(Ts, Tr))
    c = O.mean(axis=1, keepdims=True)
    if float(np.sum(n * (O - c))) < 0.0:                   # dışa bakan normal
        n = -n
    Q = O - t * n
    s_in = np.r_[ss[ss < s_end - t - 1e-6], s_end - t]
    s_in[0] = s0
    I = np.empty((len(s_in), m, 3))
    I[:, :, 0] = s_in[:, None]
    for j in range(m):
        qs = np.maximum.accumulate(Q[:, j, 0]) + np.arange(K) * 1e-12
        I[:, j, 1] = np.interp(s_in, qs, Q[:, j, 1])
        I[:, j, 2] = np.interp(s_in, qs, Q[:, j, 2])
    return s_in, I


def cowl(print_solid: bool = False) -> MeshData:
    """``U_Cowl``: PA-CF motor kaportası, yangın perdesi (s 2,045) → lüle halkası önü (2,19), yanak kabartıları.

    * ``print_solid=False`` (sahne/render): gerçek kabuk — dış yüz (gövde boyası), 1,6 mm iç yüz, yangın perdesinde ön
      dudak ve arkada lüle halkası iç çapı (Ø92) kadar açık flanş. Motor zarfları kabuğun İÇ boşluğundadır (hiçbir
      zarf kabuk etine girmez; ``engine_bay_clearance``). Panjur ve çene çıkış yarığı booleanları kabuğu deler.
    * ``print_solid=True`` (baskı kaynağı): aynı dış yüzün düz kapaklı kapalı katısı (iç boşluk/fincan YOK). printprep
      bunu içe öteleyerek oyar; arka açıklık ``cowl_aft_opening()``, çene yarığı ``cooling_exit_cutter()`` ile delinir."""
    ss, O = _cowl_grid()
    s0, s_end = _cowl_s_range()
    mb = MeshBuilder("U_Cowl")
    rings = [mb.add(o) for o in O]
    for k in range(len(rings) - 1):
        s_a, s_b = ss[k], ss[k + 1]
        mb.strip(rings[k], rings[k + 1], lambda j, a=s_a, b=s_b: fuselage_mat(a, b, j, band=False))
    _mark_chine(mb, rings, ss)
    if print_solid:
        mb.cap(rings[0], M["pacf"], start=True)
        mb.cap(rings[-1], M["pacf"], start=False)
        return mb.build(28.0)
    t = COWL_WALL
    zc = P.PROP.hub[2]
    rh = cowl_open_r()
    m = O.shape[1]
    ang = np.arctan2(O[-1][:, 2] - zc, O[-1][:, 1])
    circ = lambda s: np.column_stack([np.full(m, s), rh * np.cos(ang), zc + rh * np.sin(ang)])
    h_aft = mb.add(circ(s_end))
    h_fwd = mb.add(circ(s_end - t))
    _, I = _cowl_inner()
    irings = [mb.add(r) for r in I]
    # kapalı döngü: dış yüz → arka flanş yüzü → açıklık dudağı → flanş iç yüzü → iç yüz (geriye) → ön dudak
    mb.strip(rings[-1], h_aft, M["pacf"])
    mb.strip(h_aft, h_fwd, M["pacf"])
    mb.strip(h_fwd, irings[-1], M["pacf"])
    for k in range(len(irings) - 1, 0, -1):
        mb.strip(irings[k], irings[k - 1], M["pacf"])
    mb.strip(irings[0], rings[0], M["pacf"])
    for ring in (rings[-1], h_aft, h_fwd, rings[0]):           # flanş ve dudak kenarları keskin
        for j in range(m):
            mb.sharp.append((int(ring[j]), int(ring[(j + 1) % m])))
    return mb.build(28.0)


def cowl_aft_opening(fwd: float = 0.004, aft: float = 0.004) -> MeshData:
    """Baskı kaportasının arka flanşını açan delik katısı (``print_solid`` kabuğundan çıkarılır): itki göbeği
    yüksekliğinde, s ekseni boyunca Ø92 (lüle halkası iç çapı) silindir, ``s_end − et − fwd`` → ``s_end + aft``."""
    s0, s_end = _cowl_s_range()
    zc = P.PROP.hub[2]
    rh = cowl_open_r()
    md = lathe(np.array([[s_end - COWL_WALL - fwd, 0.0], [s_end - COWL_WALL - fwd, rh], [s_end + aft, rh],
                         [s_end + aft, 0.0]]), 72, "U_Cutter_CowlOpening", M["pacf"], "x", space="spec")
    md.verts = md.verts + np.array([0.0, 0.0, zc])
    return md.oriented()


def cowl_cavity() -> MeshData:
    """``U_Cowl_Cavity`` — YALNIZ RENDER: lüle halkası deliğinden (spinner çevresi) görünen koyu boşluk. Flanşın
    ``COWL_CAVITY_GAP`` gerisinde (s ≥ 2,19), Ø46–Ø91 ince mat koyu halka disk; motor zarflarına, baskıya, GLB'ye ve
    çakışma denetimlerine girmez (sahnede ``ucav_render_only``)."""
    s0, s_end = _cowl_s_range()
    zc = P.PROP.hub[2]
    ro = cowl_open_r() - 0.0004
    a0 = s_end + COWL_CAVITY_GAP
    md = lathe(np.array([[a0, COWL_CAVITY_R_IN], [a0, ro], [a0 + 0.0008, ro], [a0 + 0.0008, COWL_CAVITY_R_IN]]), 64,
               "U_Cowl_Cavity", M["seal"], "x", closed_profile=True, space="spec", smooth_angle=None)
    md.verts = md.verts + np.array([0.0, 0.0, zc])
    return md.oriented()


# ------------------------------------------------------------------ 3B uzaklık yardımcıları (numpy)
def _tri_array(md: MeshData) -> np.ndarray:
    """Ağın üçgenleri (t, 3, 3), spec takımı (n-genler yelpaze ile)."""
    V = np.asarray(md.verts, float)
    tris = []
    for f in md.faces:
        for i in range(1, len(f) - 1):
            tris.append((f[0], f[i], f[i + 1]))
    T = np.asarray(tris, int)
    return V[T]


def points_tris_distance(pts: np.ndarray, tris: np.ndarray, chunk: int = 48, window: float | None = None) -> np.ndarray:
    """Her noktanın üçgen kümesine en kısa 3B uzaklığı (Ericson, en yakın nokta; vektörel). ``window`` verilirse
    yalnız s'si nokta öbeğinin ±``window`` aralığına değen üçgenler denenir (hız)."""
    P3 = np.atleast_2d(np.asarray(pts, float))
    A, B, C = tris[:, 0], tris[:, 1], tris[:, 2]
    smin, smax = tris[:, :, 0].min(1), tris[:, :, 0].max(1)
    order = np.argsort(P3[:, 0])
    out = np.full(len(P3), np.inf)
    for i in range(0, len(P3), chunk):
        idx = order[i:i + chunk]
        p = P3[idx]
        if window is not None:
            sel = (smax >= p[:, 0].min() - window) & (smin <= p[:, 0].max() + window)
            if not sel.any():
                continue
            a, b, c = A[sel], B[sel], C[sel]
        else:
            a, b, c = A, B, C
        ab, ac = b - a, c - a
        pp = p[:, None, :]
        ap, bp, cp = pp - a, pp - b, pp - c
        d1, d2 = (ab * ap).sum(-1), (ac * ap).sum(-1)
        d3, d4 = (ab * bp).sum(-1), (ac * bp).sum(-1)
        d5, d6 = (ab * cp).sum(-1), (ac * cp).sum(-1)
        va, vb, vc = d3 * d6 - d5 * d4, d5 * d2 - d1 * d6, d1 * d4 - d3 * d2
        with np.errstate(divide="ignore", invalid="ignore"):
            den = va + vb + vc
            v = np.where(np.abs(den) > 1e-300, vb / den, 0.0)
            w = np.where(np.abs(den) > 1e-300, vc / den, 0.0)
            q = a + ab * v[..., None] + ac * w[..., None]                                    # yüz içi
            t6 = (d4 - d3) / np.where(np.abs((d4 - d3) + (d5 - d6)) > 1e-300, (d4 - d3) + (d5 - d6), 1.0)
            m6 = (va <= 0) & ((d4 - d3) >= 0) & ((d5 - d6) >= 0)
            q = np.where(m6[..., None], b + (c - b) * t6[..., None], q)                      # BC kenarı
            t5 = d2 / np.where(np.abs(d2 - d6) > 1e-300, d2 - d6, 1.0)
            m5 = (vb <= 0) & (d2 >= 0) & (d6 <= 0)
            q = np.where(m5[..., None], a + ac * t5[..., None], q)                          # AC kenarı
            m4 = (d6 >= 0) & (d5 <= d6)
            q = np.where(m4[..., None], np.broadcast_to(c, q.shape), q)                     # C köşesi
            t3 = d1 / np.where(np.abs(d1 - d3) > 1e-300, d1 - d3, 1.0)
            m3 = (vc <= 0) & (d1 >= 0) & (d3 <= 0)
            q = np.where(m3[..., None], a + ab * t3[..., None], q)                          # AB kenarı
            m2 = (d3 >= 0) & (d4 <= d3)
            q = np.where(m2[..., None], np.broadcast_to(b, q.shape), q)                     # B köşesi
            m1 = (d1 <= 0) & (d2 <= 0)
            q = np.where(m1[..., None], np.broadcast_to(a, q.shape), q)                     # A köşesi
        d = np.sqrt(((pp - q) ** 2).sum(-1)).min(axis=1)
        out[idx] = np.minimum(out[idx], d)
    return out


def _point_in_poly(poly: np.ndarray, pts: np.ndarray) -> np.ndarray:
    """2B nokta-çokgen içi testi (ışın sayımı), vektörel."""
    a = np.asarray(poly, float)
    b = np.roll(a, -1, axis=0)
    p = np.atleast_2d(pts)
    y0, y1 = a[:, 1][None, :], b[:, 1][None, :]
    cross = (y0 > p[:, 1:2]) != (y1 > p[:, 1:2])
    with np.errstate(divide="ignore", invalid="ignore"):
        xs = a[:, 0][None, :] + (p[:, 1:2] - y0) / np.where(cross, y1 - y0, 1.0) * (b[:, 0] - a[:, 0])[None, :]
    return (np.sum(cross & (xs > p[:, 0:1]), axis=1) % 2) == 1


def _ring_at(ss: np.ndarray, R: np.ndarray, s: float) -> np.ndarray:
    """Halka dizisinden (K, m, 3) ``s``'deki (y, z) halkası (komşu iki halka arasında doğrusal)."""
    k = int(np.clip(np.searchsorted(ss, s) - 1, 0, len(ss) - 2))
    f = float(np.clip((s - ss[k]) / max(ss[k + 1] - ss[k], 1e-12), 0.0, 1.0))
    return (R[k] + f * (R[k + 1] - R[k]))[:, 1:]


@functools.lru_cache(maxsize=1)
def _cowl_tris() -> np.ndarray:
    return _tri_array(cowl())


def cowl_clearance_points(pts: np.ndarray) -> np.ndarray:
    """Noktaların kaporta kabuğuna 3B payı (m): boşluktaki nokta → en yakın kabuk yüzüne (iç yüz, flanş, dudak)
    uzaklık (+); kabuk etinin içinde ya da kaportanın dışında → −uzaklık. Flanş/lüle bölgesinde (s ≥ s_end − et) pay
    = açıklık yarıçapı − eksene uzaklık (nokta açıklıktan geçmeli)."""
    P3 = np.atleast_2d(np.asarray(pts, float))
    ss, O = _cowl_grid()
    s_in, I = _cowl_inner()
    s0, s_end = _cowl_s_range()
    zc = P.PROP.hub[2]
    rh = cowl_open_r()
    dist = points_tris_distance(P3, _cowl_tris(), window=0.03)
    out = np.empty(len(P3))
    for i, p in enumerate(P3):
        s = float(p[0])
        if s >= s_end - COWL_WALL:
            out[i] = rh - math.hypot(p[1], p[2] - zc)
            continue
        if s < s0:
            out[i] = -dist[i]
            continue
        in_cavity = bool(_point_in_poly(_ring_at(s_in, I, s), p[1:][None])[0])
        out[i] = dist[i] if in_cavity else -dist[i]
    return out


def engine_bay_clearance(n: int = 10) -> dict[str, float]:
    """Motor bölmesi zarf payları (AERO-08, R01): her zarf parçasının (ters DLE-20: rulman burnu, karter, karbüratör,
    silindir, buji başlığı; susturucu) kaporta KABUĞUNA 3B en küçük payı (m) — iç yüz, dik çene kapanışı, ön dudak ve
    arka flanş dahil (``cowl_clearance_points``; eski kesit-içi 2B ölçüm dik yüzleri ve arka fincanı görmüyordu).
    Flanş bölgesinde pay = açıklık (Ø92) yarıçapı − eksene uzaklık. Ayrıca: ``carb_to_firewall``, ``spinner_to_ring``,
    ``washer_s`` ve ``min_part``/``min_value`` (susturucu hariç en dar zarf)."""
    er = _PR["exhaust_ring"]
    out: dict[str, float] = {}
    for part in P.engine_envelope():
        out[part.name] = float(cowl_clearance_points(part.surface_points(n)).min())
    s_fw = float(_F["modules"]["firewall_s_m"]) + float(_F["modules"]["firewall_t_m"])
    carb = next(p for p in P.engine_envelope() if p.name == "carb")
    out["carb_to_firewall"] = float(carb.surface_points(4)[:, 0].min()) - s_fw
    pr = P.PROP
    out["spinner_to_ring"] = (pr.spinner_base_s - SPINNER_BACKPLATE) - float(er["s_to_m"])
    out["washer_s"] = float(P.engine_washer()[0])
    eng = {k: v for k, v in out.items() if k in ("front_bearing", "crankcase", "carb", "cylinder", "spark_cap")}
    out["min_part"] = min(eng, key=eng.get)
    out["min_value"] = eng[out["min_part"]]
    return out


def _inside_env(part: "P.EnvPart", Q: np.ndarray) -> np.ndarray:
    """Noktalar (spec, (n, 3)) paketleme zarfının (kutu/silindir) içinde mi."""
    A = np.asarray(part.axes, float)
    q = (Q - np.asarray(part.center, float)) @ A.T
    h = np.asarray(part.half, float)
    if part.kind == "cyl":
        return (np.abs(q[:, 0]) <= h[0]) & (np.hypot(q[:, 1], q[:, 2]) <= h[1])
    return np.all(np.abs(q) <= h, axis=1)


def cooling_flow_areas(n: int = 161) -> dict[str, float]:
    """Soğutma havası alanları (m², R01): ``inlet`` = NACA ağzı (boğazda düz tavan − karın eğrisini izleyen dudak, deri +
    3 mm); ``ring`` = lüle akış alanı (halka iç çapı − spinner tabanı); ``aft_opening`` = kaporta arka açıklığından
    (Ø92, flanş → halka arka yüzü) geçen en dar SERBEST kesit — açıklık dairesi eksi o düzlemdeki motor zarfları
    (karter, rulman burnu), ızgara ile; ``chin`` = çene çıkış yarığı akış kesiti (en × yükseklik); ``exit`` =
    min(ring, aft_opening) + chin; ``ratio`` = exit / inlet."""
    I = P.intake_spec()
    s_t = float(I["throat_s"])
    w, h = 0.5 * float(I["mouth_w_m"]), float(I["mouth_h_m"])
    zb = _skin_bottom(s_t, 0.0)
    ys = np.linspace(-w, w, 241)
    lip = np.array([_skin_bottom(s_t, float(y)) + INTAKE_LIP_T for y in ys])
    gap = np.clip(zb + h - lip, 0.0, None)
    inlet = float(np.sum(0.5 * (gap[1:] + gap[:-1]) * np.diff(ys)))
    er = _PR["exhaust_ring"]
    ring = math.pi / 4 * (float(er["id_m"]) ** 2 - P.PROP.spinner_d ** 2)
    s0, s_end = _cowl_s_range()
    rh = cowl_open_r()
    zc = P.PROP.hub[2]
    g = np.linspace(-rh, rh, n)
    Y, Z = np.meshgrid(g, g)
    disc = (Y ** 2 + Z ** 2) <= rh ** 2
    cell = (g[1] - g[0]) ** 2
    parts = P.engine_envelope()
    free = []
    for s in np.linspace(s_end - COWL_WALL, float(er["s_to_m"]), 9):
        Q = np.column_stack([np.full(int(disc.sum()), s), Y[disc], zc + Z[disc]])
        blk = np.zeros(len(Q), bool)
        for part in parts:
            blk |= _inside_env(part, Q)
        free.append(float((~blk).sum()) * cell)
    aft_opening = min(free)
    ce = _PR["cowl"]["cooling_exit"]
    chin = float(ce["w_m"]) * float(ce["h_m"])
    ex = min(ring, aft_opening) + chin
    return {"inlet": inlet, "ring": ring, "aft_opening": aft_opening, "chin": chin, "exit": ex, "ratio": ex / inlet}


EXHAUST_RING_GAP_M = 0.0002              # lüle halkası ön yüzü kaporta arka flanşına alın alına (yapıştırma aralığı): eski
                                        # 2 mm geçme dili flanşla 4,3 cm³ çakışıyordu (baskı montaj denetimi)


def exhaust_ring() -> MeshData:
    """``U_ExhaustRing``: lüle halkası (s 2,19–2,205; dış Ø0,112, iç Ø0,092). Ön yüzü kaporta arka flanşına alın alına
    oturur (``EXHAUST_RING_GAP_M``; çakışma yok), önde 0,4 mm V-oluk (panel çizgisi). Arka dudak her açıda KENDİ dış
    yarıçapından iç yarıçapa yarım elipsle iner (printprep baskı katısıyla aynı; eski ortalama yarıçaplı daire 45°
    konumlarında 1–2 mm sivri sırt bırakıyordu)."""
    er = _PR["exhaust_ring"]
    s0, s1 = float(er["s_from_m"]), float(er["s_to_m"])
    sf = s0 + EXHAUST_RING_GAP_M
    ri = 0.5 * float(er["id_m"])
    zc = P.PROP.hub[2]
    mb = MeshBuilder("U_ExhaustRing")
    outer_s = [sf, s0 + 0.0012, s0 + 0.004, s1 - 0.006, s1 - 0.0025]
    shrink = [0.985, 0.992, 1.0, 1.0, 1.0]               # 0,4 mm V-oluk = panel çizgisi
    rings, pts = [], None
    for s, f in zip(outer_s, shrink):                   # dış yüz: kaporta ile aynı halka (basamaksız birleşim)
        yz = fuselage_ring_yz(min(max(s, s0), s1))
        pts = np.column_stack([np.full(len(yz), s), yz[:, 0] * f, zc + (yz[:, 1] - zc) * f])
        rings.append(mb.add(pts))
    ang = np.arctan2(pts[:, 2] - zc, pts[:, 1])
    ro = np.hypot(pts[:, 1], pts[:, 2] - zc)
    for a in np.linspace(0.0, math.pi, 9)[1:]:          # yuvarlak dudak: her açıda dış → iç yarım elips (geriye 2,5 mm)
        s = s1 - 0.0025 + 0.0025 * math.sin(a)
        r = 0.5 * (ro + ri) + 0.5 * (ro - ri) * math.cos(a)
        rings.append(mb.add(np.column_stack([np.full(len(ang), s), r * np.cos(ang), zc + r * np.sin(ang)])))
    rings.append(mb.add(np.column_stack([np.full(len(ang), sf), ri * np.cos(ang), zc + ri * np.sin(ang)])))
    mb.loft(rings, M["nozzle"])
    mb.strip(rings[-1], rings[0], M["nozzle"])
    return mb.build(40.0)


# =====================================================================================================
# Taşıyıcı yüzeyler: profil, aileler, oyuklu sabit parça + kumanda yüzeyleri
# =====================================================================================================
@dataclass
class Foil:
    """Yerel 2B kesit: ``x`` veter yönünde (geriye +), ``z`` kalınlık yönünde; üst/alt HK → FK."""

    xu: np.ndarray
    zu: np.ndarray
    xl: np.ndarray
    zl: np.ndarray

    @classmethod
    def from_selig(cls, p2: np.ndarray) -> "Foil":
        n = (len(p2) + 1) // 2
        up, lo = p2[:n][::-1], p2[n - 1:]
        return cls(np.maximum.accumulate(up[:, 0]), up[:, 1], np.maximum.accumulate(lo[:, 0]), lo[:, 1])

    @property
    def x_le(self) -> float:
        return float(min(self.xu[0], self.xl[0]))

    @property
    def x_te(self) -> float:
        return float(min(self.xu[-1], self.xl[-1]))

    def up(self, x):
        return np.interp(x, self.xu, self.zu)

    def lo(self, x):
        return np.interp(x, self.xl, self.zl)

    def scaled(self, f: float) -> "Foil":
        """Kamber çizgisi korunarak kalınlık ölçekleme (uç kapanışları)."""
        xs = np.union1d(self.xu, self.xl)
        zu, zl = self.up(xs), self.lo(xs)
        zc = 0.5 * (zu + zl)
        return Foil(xs, zc + (zu - zc) * f, xs.copy(), zc + (zl - zc) * f)


class _Family:
    """Kesit ailesi: açıklık parametresi ``t`` → yerel çerçeve (O, ex, ez) ve 2B profil; düzlem normali ``en``."""

    span_name = "t"

    def frame(self, t: float) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
        raise NotImplementedError

    def foil(self, t: float) -> Foil:
        raise NotImplementedError

    def plane(self, t: float) -> tuple[np.ndarray, np.ndarray]:
        raise NotImplementedError

    def tdir(self, t: float) -> np.ndarray:
        """Artan ``t`` yönündeki birim vektör (spec takımı)."""
        raise NotImplementedError

    def to3d(self, t: float, p2: np.ndarray) -> np.ndarray:
        O, ex, ez = self.frame(t)
        p2 = np.atleast_2d(p2)
        return O + np.outer(p2[:, 0], ex) + np.outer(p2[:, 1], ez)

    def to2d(self, t: float, p3) -> np.ndarray:
        O, ex, ez = self.frame(t)
        d = np.asarray(p3, float) - O
        return np.array([float(d @ ex), float(d @ ez)])

    @staticmethod
    def _sel_to_2d(sec: np.ndarray, O, ex, ez) -> np.ndarray:
        d = sec - O
        return np.column_stack([d @ ex, d @ ez])


N_SEC = 241                                  # params kesit yoğunluğu (yeniden örnekleme kaynağı)


class WingFamily(_Family):
    """Kanat (yarı): ``t = |y|``; kesit düzlemi y = sabit; çerçeve ana kiriş noktasında, yerel açıyla dönük."""

    def __init__(self, side: str):
        self.sg = 1.0 if side.upper() == "L" else -1.0

    @functools.lru_cache(maxsize=None)
    def frame(self, t: float):
        st = P.wing_station(self.sg * t)
        i = _rad(st.incidence_deg)
        O = np.array([st.spar_s, self.sg * t, st.z_ref])
        return O, np.array([math.cos(i), 0.0, -math.sin(i)]), np.array([math.sin(i), 0.0, math.cos(i)])

    @functools.lru_cache(maxsize=None)
    def foil(self, t: float) -> Foil:
        O, ex, ez = self.frame(t)
        return Foil.from_selig(self._sel_to_2d(P.wing_section(self.sg * t, N_SEC), O, ex, ez))

    def plane(self, t: float):
        return np.array([0.0, self.sg * t, 0.0]), np.array([0.0, 1.0, 0.0])

    def tdir(self, t: float) -> np.ndarray:
        return np.array([0.0, self.sg, 0.0])


class StabFamily(_Family):
    """Yatay stabilize (tek parça, iki yan): ``t = y`` (−0,525 … 0,525; uçlar dikeylerin içine gömülür)."""

    @functools.lru_cache(maxsize=None)
    def frame(self, t: float):
        yc = float(np.clip(t, -P.STAB_HALF_SPAN, P.STAB_HALF_SPAN))
        st = P.stab_station(yc)
        i = _rad(st.incidence_deg)
        xr = float(_T["stab"]["incidence_axis_chord_fraction"])
        O = np.array([st.le_s + xr * st.chord, t, st.z])
        return O, np.array([math.cos(i), 0.0, -math.sin(i)]), np.array([math.sin(i), 0.0, math.cos(i)])

    @functools.lru_cache(maxsize=None)
    def foil(self, t: float) -> Foil:
        yc = float(np.clip(t, -P.STAB_HALF_SPAN, P.STAB_HALF_SPAN))
        O, ex, ez = self.frame(t)
        sec = P.stab_section(yc, N_SEC)
        sec[:, 1] = t
        return Foil.from_selig(self._sel_to_2d(sec, O, ex, ez))

    def plane(self, t: float):
        return np.array([0.0, t, 0.0]), np.array([0.0, 1.0, 0.0])

    def tdir(self, t: float) -> np.ndarray:
        return np.array([0.0, 1.0, 0.0])


FIN_ROOT_ROUND = 0.012                       # dikey kökü stabilize altında yuvarlak kapanır. varsayım
FIN_TIP_ROUND = 0.006                        # dikey ucu yuvarlak kapanış (h = 0,32 içinde kalır). varsayım


class FinFamily(_Family):
    """Dikey: ``t = h`` (eğik açıklık boyunca); kesit düzlemi ``span_dir``'e dik; kök altında ve uçta
    kalınlık çeyrek daireyle sıfıra iner (fermuar)."""

    def __init__(self, side: str):
        self.side = "L" if side.upper() == "L" else "R"
        self.H = float(_T["fin"]["height_m"])
        st0 = P.fin_station(0.0, self.side)
        st1 = P.fin_station(self.H, self.side)
        self.le0 = np.asarray(st0.le)
        self.dle = (np.asarray(st1.le) - self.le0) / self.H
        self.span = np.asarray(st0.span_dir)
        self.normal = np.asarray(st0.normal)

    def frame(self, t: float):
        return self.le0 + t * self.dle, np.array([1.0, 0.0, 0.0]), self.normal

    def closure(self, t: float) -> float:
        if t < 0:
            q = min(1.0, -t / FIN_ROOT_ROUND)
            return math.sqrt(max(0.0, 1 - q * q))
        if t > self.H - FIN_TIP_ROUND:
            q = min(1.0, (t - (self.H - FIN_TIP_ROUND)) / FIN_TIP_ROUND)
            return math.sqrt(max(0.0, 1 - q * q))
        return 1.0

    @functools.lru_cache(maxsize=None)
    def foil(self, t: float) -> Foil:
        st = P.fin_station(float(np.clip(t, 0.0, self.H)), self.side)
        af = P.tail_airfoil(N_SEC, st.chord) * st.chord
        f = Foil.from_selig(af)
        c = self.closure(t)
        return f if c >= 1.0 else f.scaled(c)

    def plane(self, t: float):
        return self.le0 + t * self.dle, self.span

    def tdir(self, t: float) -> np.ndarray:
        return self.span


@dataclass
class Cutout:
    """Sabit parçada kumanda yüzeyi boşluğu: ``t0 < t1`` (açıklık parametresi), menteşe ``hinge``."""

    hinge: P.HingeLine
    t0: float
    t1: float


def _hinge_center(fam: _Family, t: float, h: P.HingeLine) -> np.ndarray:
    Q, n = fam.plane(t)
    a, b = np.asarray(h.p_in), np.asarray(h.p_out)
    lam = float(np.dot(Q - a, n) / np.dot(b - a, n))
    return fam.to2d(t, a + lam * (b - a))


@dataclass
class _Cove:
    xh: float
    zh: float
    ru: float
    rl: float
    phi_u: float
    phi_l: float
    gap: float

    def rho(self, phi):
        return self.ru + (self.rl - self.ru) * (1 - np.cos(np.asarray(phi) - 0.5 * math.pi)) / 2

    def point(self, phi, extra: float = 0.0) -> np.ndarray:
        r = self.rho(phi) + extra
        return np.column_stack([self.xh + r * np.cos(phi), self.zh + r * np.sin(phi)])


def _cove(foil: Foil, H2: np.ndarray, gap: float) -> _Cove:
    xh, zh = float(H2[0]), float(H2[1])
    ru = max(float(foil.up(xh)) - zh, 3e-4)
    rl = max(zh - float(foil.lo(xh)), 3e-4)
    c = _Cove(xh, zh, ru, rl, 0.0, 0.0, gap)

    def fu(phi):
        p = c.point(np.array([phi]), gap)[0]
        return p[1] - float(foil.up(p[0]))

    def fl(phi):
        p = c.point(np.array([phi]), gap)[0]
        return p[1] - float(foil.lo(p[0]))

    lo_, hi_ = 0.5 * math.pi, math.pi                  # fu: + → −
    for _ in range(50):
        mid = 0.5 * (lo_ + hi_)
        lo_, hi_ = (mid, hi_) if fu(mid) > 0 else (lo_, mid)
    c.phi_u = 0.5 * (lo_ + hi_)
    lo_, hi_ = math.pi, 1.5 * math.pi                  # fl: + → −
    for _ in range(50):
        mid = 0.5 * (lo_ + hi_)
        lo_, hi_ = (mid, hi_) if fl(mid) > 0 else (lo_, mid)
    c.phi_l = 0.5 * (lo_ + hi_)
    return c


LE_STRIP = 0.022                         # hücum kenarı erozyon bandı: üst ve altta veterin %2,2'si (F12). varsayım
LE_STRIP_K = 7                           # şerit kenarının ön-kısım dizisindeki sabit indisi (bütün kesitlerde aynı)


def _fwd_x(x_le: float, x_lip: float, nf: int, x_strip: float | None = None) -> np.ndarray:
    """HK → dudak x istasyonları (nf + 1): HK'de kosinüs sıklaşması; ``x_strip`` verilirse ``LE_STRIP_K``
    indisinde tam olarak şerit kenarı bulunur (malzeme sınırı düz çizgi olur)."""
    if x_strip is None or not (x_le < x_strip < x_lip):
        t = np.linspace(0.0, 1.0, nf + 1)
        return x_le + (x_lip - x_le) * (1.0 - np.cos(0.5 * math.pi * t))    # 0 = HK (sık) … 1 = dudak
    k = LE_STRIP_K
    u = np.linspace(0.0, 1.0, k + 1)
    a = x_le + (x_strip - x_le) * (1.0 - np.cos(0.5 * math.pi * u))
    v = np.linspace(0.0, 1.0, nf - k + 1)[1:]
    b = x_strip + (x_lip - x_strip) * (0.45 * v + 0.55 * (1.0 - np.cos(0.5 * math.pi * v)))
    return np.r_[a, b]


def _aft_x(x_lip: float, x_te: float, na: int) -> np.ndarray:
    t = np.linspace(0.0, 1.0, na + 1)
    g = 0.85 * t + 0.15 * np.sin(0.5 * math.pi * t)                          # FK'ye doğru hafif sık
    return x_lip + (x_te - x_lip) * g


def _section_ring(foil: Foil, xlu: float, xll: float, nf: int, na: int, cove: _Cove | None,
                  strip: float | None = LE_STRIP):
    """Tam halka (Selig sırası: FK üst → HK → FK alt) ve oyuklu halka (aynı ön kısım; arka kısım oyuk yayı).

    Konumlar: üst arka [0, na), üst ön [na, na+nf) (dudak = na), HK = na+nf, alt ön, alt arka. m = 2na+2nf+1.
    HK şeridi kenarı üstte ``na + nf − LE_STRIP_K``, altta ``na + nf + LE_STRIP_K`` konumundadır.
    """
    xs = None if strip is None else foil.x_le + strip * (foil.x_te - foil.x_le)
    xu_f, xl_f = _fwd_x(foil.x_le, xlu, nf, xs), _fwd_x(foil.x_le, xll, nf, xs)
    xu_a, xl_a = _aft_x(xlu, foil.x_te, na), _aft_x(xll, foil.x_te, na)
    le = np.array([[foil.x_le, 0.5 * (foil.up(foil.x_le) + foil.lo(foil.x_le))]])
    up_aft = np.column_stack([xu_a[1:][::-1], foil.up(xu_a[1:][::-1])])
    up_fwd = np.column_stack([xu_f[1:][::-1], foil.up(xu_f[1:][::-1])])
    lo_fwd = np.column_stack([xl_f[1:], foil.lo(xl_f[1:])])
    lo_aft = np.column_stack([xl_a[1:], foil.lo(xl_a[1:])])
    full = np.vstack([up_aft, up_fwd, le, lo_fwd, lo_aft])
    trunc = None
    if cove is not None:
        phis = cove.phi_l + (cove.phi_u - cove.phi_l) * np.arange(1, 2 * na + 1) / (2 * na + 1)
        cp = cove.point(phis, cove.gap)
        trunc = full.copy()
        m = len(full)
        trunc[m - na:] = cp[:na]
        trunc[:na] = cp[na:]
    return full, trunc


def _surface_ring(foil: Foil, cove: _Cove, nn: int, ns: int) -> np.ndarray:
    """Kumanda yüzeyi halkası: burun yayı (üst → ön → alt), alt yüzey → FK, üst yüzey → menteşeye."""
    phis = np.linspace(0.5 * math.pi, 1.5 * math.pi, nn)
    nose = cove.point(phis)
    xa = _aft_x(cove.xh, foil.x_te, ns)[1:]
    lo = np.column_stack([xa, foil.lo(xa)])
    up = np.column_stack([xa[::-1], foil.up(xa[::-1])])
    return np.vstack([nose, lo, up])


# Rig sürücü aralıkları (spec rig.ranges_deg: aileron ±25, flap 0…35, elevator ±25, rudder ±25); uç boşlukları
# bu aralık ile fiziksel sınırların birleşiminde çakışma olmayacak şekilde hesaplanır.
_RIG = {k: tuple(float(x) for x in v) for k, v in P.SPEC.get("rig", {}).get("ranges_deg", {}).items()}
RIG_RANGE_DEG = {"Aileron": _RIG.get("aileron", (-25.0, 25.0)), "FlapIn": _RIG.get("flap", (0.0, 35.0)),
                 "FlapOut": _RIG.get("flap", (0.0, 35.0)), "Elevator": _RIG.get("elevator", (-25.0, 25.0)),
                 "Rudder": _RIG.get("rudder", (-25.0, 25.0))}          # spec.yaml → rig.ranges_deg
SURFACE_END_MARGIN = 0.0004              # uç boşluğuna ek pay


def deflection_test_angles(h: P.HingeLine) -> list[float]:
    """Uç boşluğu hesabında denenen sapmalar (+ = firar kenarı aşağı/sancağa): rig aralığı ∪ fiziksel sınırlar."""
    lo, hi = RIG_RANGE_DEG[h.name]
    lo, hi = min(lo, -h.neg_max_deg), max(hi, h.pos_max_deg)
    return sorted({lo, 0.5 * lo, hi, 0.5 * hi} - {0.0})


def _end_need(fam: _Family, h: P.HingeLine, t_end: float, toward: float, nn: int, ns: int) -> float:
    """Menteşe ekseni kesit düzlemine dik değilse (oklu menteşe), yüzey dönünce uç kaburgası açıklık yönünde kayar.
    ``t_end`` kesitindeki halkayı test açılarında döndürür; uç duvarına doğru (``toward`` = −1 iç, +1 dış uç)
    en büyük kaymayı (m) döndürür."""
    foil = fam.foil(t_end)
    cv = _cove(foil, _hinge_center(fam, t_end, h), h.gap_m)
    p3 = fam.to3d(t_end, _surface_ring(foil, cv, nn, ns))
    pb = P.points_to_blender(p3)
    td = fam.tdir(t_end)
    need = 0.0
    for ang in deflection_test_angles(h):
        pr = P.points_to_blender(P.rotate_about_axis(pb, h.mid_b, h.axis_positive_b, ang))
        need = max(need, float(np.max(toward * ((pr - p3) @ td))))
    return need


def surface_end_gaps(fam: _Family, c: "Cutout", nn: int, ns: int) -> tuple[float, float]:
    """Kumanda yüzeyinin iç/dış uç boşlukları: menteşe aralığı + (oklu menteşede) dönüşte uç kaburgasının kayması."""
    g = c.hinge.gap_m
    out = []
    for t_end, toward in ((c.t0 + g, -1.0), (c.t1 - g, 1.0)):
        need = 0.0
        for _ in range(2):                                  # kaydırılmış uçta bir kez daha (koniklik)
            tt = t_end - toward * need
            need = _end_need(fam, c.hinge, tt, toward, nn, ns)
        out.append(g + (need + SURFACE_END_MARGIN if need > 2e-4 else 0.0))
    return out[0], out[1]


def hinge_plane(fam: _Family, h: P.HingeLine, t: float) -> tuple[np.ndarray, np.ndarray]:
    """Menteşe eksenine dik, ``t`` kesitindeki menteşe noktasından geçen düzlem ``(nokta, normal)`` (spec)."""
    H3 = fam.to3d(t, _hinge_center(fam, t, h))[0]
    return H3, _unit(np.subtract(h.p_out, h.p_in))


def _project_to_plane(pts: np.ndarray, plane, tdir: np.ndarray) -> np.ndarray:
    """Noktaları açıklık yönünde (``tdir``) kaydırarak düzleme oturtur."""
    H, a = plane
    lam = -((np.asarray(pts) - H) @ a) / float(tdir @ a)
    return np.asarray(pts) + lam[:, None] * tdir


OBLIQUE_MARGIN = 0.0015                  # eğik uç bölgesinde loft istasyonu bırakılmayan ek pay


def _lam_range(fam: _Family, t: float, plane, x_from: float) -> tuple[float, float]:
    """``t`` kesitinde ``x_from``'un gerisindeki profil noktalarının düzleme oturtma kaymaları (min, max)."""
    foil = fam.foil(t)
    xs = np.linspace(x_from, foil.x_te, 40)
    pts = np.vstack([np.column_stack([xs, foil.up(xs)]), np.column_stack([xs, foil.lo(xs)])])
    p3 = fam.to3d(t, pts)
    H, a = plane
    lam = -((p3 - H) @ a) / float(fam.tdir(t) @ a)
    return float(lam.min()), float(lam.max())


def _oblique_station_filter(fam: _Family, ts: np.ndarray, cutouts: Sequence[Cutout], start_plane) -> np.ndarray:
    """Eğik uç duvarlarının yanında, kaydırılan arka noktaların aşacağı istasyonları çıkarır (loft katlanmaz)."""
    keep = np.ones(len(ts), bool)
    protect = {round(float(v), 7) for c in cutouts for v in (c.t0, c.t1)} | {round(float(ts[0]), 7), round(float(ts[-1]), 7)}
    m = OBLIQUE_MARGIN
    for c in cutouts:
        for t_w, cut_side in ((c.t0, 1.0), (c.t1, -1.0)):
            x_h = float(_hinge_center(fam, t_w, c.hinge)[0]) - c.hinge.nose_radius_in
            lo, hi = _lam_range(fam, t_w, hinge_plane(fam, c.hinge, t_w), x_h)
            if cut_side > 0:      # sabit bölge t < t0
                bad = (ts >= t_w + min(lo, 0.0) - m) & (ts < t_w)
            else:                 # sabit bölge t > t1
                bad = (ts > t_w) & (ts <= t_w + max(hi, 0.0) + m)
            keep &= ~bad
    if start_plane is not None:
        foil = fam.foil(float(ts[0]))
        lo, hi = _lam_range(fam, float(ts[0]), start_plane, foil.x_le + 0.6 * (foil.x_te - foil.x_le))
        keep &= ~((ts > ts[0]) & (ts <= ts[0] + max(hi, 0.0) + m))
    for i, t in enumerate(ts):
        if round(float(t), 7) in protect:
            keep[i] = True
    return ts[keep]


def lifting_part(name: str, fam: _Family, stations: Sequence[float], cutouts: Sequence[Cutout], *,
                 nf: int, na: int, nn: int = 15, ns: int = 12, mat_up: str = M["skin_top"],
                 mat_lo: str = M["skin_bottom"], zip_start: bool = False, zip_end: bool = False,
                 lip_fraction: float = 0.72, smooth_angle: float = 30.0, le_strip: bool = True,
                 oblique_ends: bool = False, start_plane=None, le_mat: Callable[[float], str] | str | None = None):
    """Taşıyıcı yüzey parçası (sabit deri, oyuklu) ve içindeki kumanda yüzeyleri.

    Dönüş: ``(MeshData sabit parça, {nesne_adı: MeshData yüzey}, {nesne_adı: [(t, 2B halka)…]})``.
    Boşluk sınırlarında (t0, t1) tam ve oyuklu halka aynı t'de ön kısmı paylaşır; aradaki uç duvarı n-gendir.
    ``oblique_ends``: boşluk uç duvarları ve yüzey uç kaburgaları menteşe eksenine dik düzlemlerdedir (gerçek
    kanattaki gibi; dihedral/ok nedeniyle dönüşte uç kayması olmaz, aralık her açıda menteşe aralığıdır). Aksi
    halde uçlar kesit düzlemindedir ve uç boşluğu dönüş zarfından hesaplanır (``surface_end_gaps``).
    ``start_plane``: ilk halkanın arka kısmı bu düzleme oturtulur (komşu parçadaki eğik duvarla eşleşme).
    ``le_mat``: HK şeridi malzemesi (ad ya da açıklık parametresi t → ad); None → ``le_strip`` rolü (erozyon bandı).
    """
    ts = np.asarray(sorted(set(np.round(stations, 7))), float)
    if oblique_ends or start_plane is not None:
        ts = _oblique_station_filter(fam, ts, cutouts if oblique_ends else [], start_plane)
    m = 2 * na + 2 * nf + 1
    c_le = na + nf

    def in_cut(t):
        for c in cutouts:
            if c.t0 - 1e-7 <= t <= c.t1 + 1e-7:
                return c
        return None

    coves: dict[float, _Cove] = {}
    for t in ts:
        c = in_cut(t)
        if c is not None:
            coves[float(t)] = _cove(fam.foil(float(t)), _hinge_center(fam, float(t), c.hinge), c.hinge.gap_m)
    if coves:
        tk = np.array(sorted(coves))
        lu = np.array([coves[k].point(np.array([coves[k].phi_u]), coves[k].gap)[0, 0] for k in tk])
        ll = np.array([coves[k].point(np.array([coves[k].phi_l]), coves[k].gap)[0, 0] for k in tk])
        # tam kesitlerde dudak konumu: boşluk kesitlerindeki veter oranının enterpolasyonu (koniklik izlenir)
        le_k = np.array([fam.foil(float(q)).x_le for q in tk])
        ch_k = np.array([fam.foil(float(q)).x_te - fam.foil(float(q)).x_le for q in tk])
        fu_k, fl_k = (lu - le_k) / ch_k, (ll - le_k) / ch_k
    mb = MeshBuilder(name)
    ks = LE_STRIP_K if le_strip else -1
    le_fn = (lambda tt: M["le_strip"]) if le_mat is None else ((lambda tt: le_mat) if isinstance(le_mat, str) else le_mat)

    def mat_ring(j, tm=0.0):
        return le_fn(tm) if c_le - ks <= j < c_le + ks else (mat_up if j < c_le else mat_lo)
    aft_pos = list(range(m - na, m)) + list(range(0, na))
    prev = None
    first_ring = None
    for k, t in enumerate(ts):
        t = float(t)
        foil = fam.foil(t)
        c = in_cut(t)
        if coves:
            if t in coves:
                cv = coves[t]
                xlu = float(cv.point(np.array([cv.phi_u]), cv.gap)[0, 0])
                xll = float(cv.point(np.array([cv.phi_l]), cv.gap)[0, 0])
            else:
                ch = foil.x_te - foil.x_le
                xlu = foil.x_le + ch * float(np.interp(t, tk, fu_k))
                xll = foil.x_le + ch * float(np.interp(t, tk, fl_k))
        else:
            xlu = xll = foil.x_le + lip_fraction * (foil.x_te - foil.x_le)
        zipped = (zip_start and k == 0) or (zip_end and k == len(ts) - 1)
        if zipped:
            xlu = xll = 0.5 * (xlu + xll)
        full2, trunc2 = _section_ring(foil, xlu, xll, nf, na, coves.get(t) if c is not None else None,
                                      LE_STRIP if le_strip else None)
        is_t0 = c is not None and abs(t - c.t0) < 1e-7
        is_t1 = c is not None and abs(t - c.t1) < 1e-7
        plane = None
        if oblique_ends and (is_t0 or is_t1):
            plane = hinge_plane(fam, c.hinge, t)
        elif start_plane is not None and k == 0:
            plane = start_plane
        F3 = fam.to3d(t, full2)
        if plane is not None:
            pp = aft_pos + [na, m - 1 - na]                   # arka deri + dudaklar düzleme
            F3[pp] = _project_to_plane(F3[pp], plane, fam.tdir(t))
        if zipped:
            half = mb.add(F3[:c_le + 1])
            F = np.r_[half, half[:-1][::-1]]
        else:
            F = mb.add(F3)
        if c is not None:
            T = F.copy()
            T3 = fam.to3d(t, trunc2[aft_pos])
            if plane is not None:
                T3 = _project_to_plane(T3, plane, fam.tdir(t))
            T[aft_pos] = mb.add(T3)
        if c is None:
            cur_in, cur_out = F, F
        elif is_t0 and k > 0:
            cur_in, cur_out = F, T
            poly = [F[m - 1 - na]] + [F[p] for p in aft_pos] + [F[na]] + [T[p] for p in reversed(aft_pos)]
            mb.face(poly, mat_up)
        elif is_t1 and k < len(ts) - 1:
            cur_in, cur_out = T, F
            poly = [T[m - 1 - na]] + [T[p] for p in aft_pos] + [T[na]] + [F[p] for p in reversed(aft_pos)]
            mb.face(poly, mat_up)
        else:
            cur_in, cur_out = T, T
        if prev is None:
            first_ring = cur_in
        else:
            mb.strip(prev, cur_in, lambda j, tm=0.5 * (t_prev + t): mat_ring(j, tm))
        prev = cur_out
        t_prev = t
    if not zip_start:
        mb.cap(first_ring, mat_up, start=True)
    if not zip_end:
        mb.cap(prev, mat_up, start=False)
    fixed = mb.build(smooth_angle)

    # kumanda yüzeyleri
    surfaces, surf_rings = {}, {}
    for c in cutouts:
        g0, g1 = (c.hinge.gap_m, c.hinge.gap_m) if oblique_ends else surface_end_gaps(fam, c, nn, ns)
        a0, a1 = c.t0 + g0 + 1e-4, c.t1 - g1 - 1e-4
        if oblique_ends:                                   # eğik uç halkasının taşacağı iç istasyonları atla
            x_h = float(_hinge_center(fam, c.t0 + g0, c.hinge)[0]) - c.hinge.nose_radius_in
            a0 += max(0.0, _lam_range(fam, c.t0 + g0, hinge_plane(fam, c.hinge, c.t0 + g0), x_h)[1]) + OBLIQUE_MARGIN
            x_h = float(_hinge_center(fam, c.t1 - g1, c.hinge)[0]) - c.hinge.nose_radius_out
            a1 += min(0.0, _lam_range(fam, c.t1 - g1, hinge_plane(fam, c.hinge, c.t1 - g1), x_h)[0]) - OBLIQUE_MARGIN
        tt = [c.t0 + g0] + [float(t) for t in ts if a0 < t < a1] + [c.t1 - g1]
        sb = MeshBuilder(c.hinge.obj_name)
        rings2 = []
        idx = []
        for k, t in enumerate(tt):
            foil = fam.foil(t)
            cv = _cove(foil, _hinge_center(fam, t, c.hinge), c.hinge.gap_m)
            r2 = _surface_ring(foil, cv, nn, ns)
            rings2.append((t, r2))
            p3 = fam.to3d(t, r2)
            if oblique_ends and k in (0, len(tt) - 1):
                p3 = _project_to_plane(p3, hinge_plane(fam, c.hinge, t), fam.tdir(t))
            idx.append(sb.add(p3))
        mu = nn // 2
        msurf = lambda j: mat_up if (j < mu or j >= nn + ns - 1) else mat_lo
        sb.loft(idx, msurf)
        sb.cap(idx[0], mat_up, start=True)
        sb.cap(idx[-1], mat_up, start=False)
        surfaces[c.hinge.obj_name] = sb.build(smooth_angle)
        surf_rings[c.hinge.obj_name] = rings2
    return fixed, surfaces, surf_rings


def _dense_between(a: float, b: float, step: float) -> np.ndarray:
    n = max(1, int(math.ceil(abs(b - a) / step)))
    return np.linspace(a, b, n + 1)


def wing_stations(part: str) -> np.ndarray:
    """Kanat parçası açıklık istasyonları (sol, |y|): kırık/boşluk/glove sınırları dahil."""
    parts = P.wing_parts()
    y0, y1 = parts[part]
    bp = [v for v in P.wing_breakpoints() if y0 - 1e-9 <= v <= y1 + 1e-9]
    if part == "U_WingCenter":
        base = np.r_[_dense_between(0.0, 0.06, 0.02), _dense_between(0.06, 0.26, 0.006), _dense_between(0.26, y1, 0.012)]
    elif part == "U_WingOuter":
        base = np.r_[_dense_between(y0, 1.0, 0.02), _dense_between(1.0, y1, 0.018)]
    else:
        rr = float(_W["raked_tip"]["end_round_m"])
        base = np.r_[_dense_between(y0, y1 - rr, 0.008), y1 - rr * (1 - np.sin(np.linspace(0, 0.5 * math.pi, 7)))]
    return np.unique(np.round(np.r_[base, bp, y0, y1], 7))


_WING_N = {"nf": 34, "na": 12, "nn": 15, "ns": 12}
_TAIL_N = {"nf": 26, "na": 10, "nn": 13, "ns": 10}


def wing_parts(side: str = "L") -> dict[str, MeshData]:
    """Bir yandaki kanat nesneleri: ``U_WingCenter_<s>`` (kök/glove bloğu, gövde içinden y = 0'a), ``U_WingOuter_<s>``
    (sökülebilir dış panel), ``U_Tip_<s>`` (70 mm eğik uç, kapalı), ``U_FlapIn_<s>``, ``U_FlapOut_<s>``,
    ``U_Aileron_<s>`` (yuvarlak burunlu, menteşe ekseni etrafında dönmeye hazır)."""
    sd = "L" if side.upper() == "L" else "R"
    fam = WingFamily(sd)
    out: dict[str, MeshData] = {}
    cuts = {name: P.hinge_line(name, sd) for name in ("FlapIn", "FlapOut", "Aileron")}
    plan = {"U_WingCenter": ["FlapIn"], "U_WingOuter": ["FlapOut", "Aileron"], "U_Tip": []}
    ail = cuts["Aileron"]
    tip_plane = hinge_plane(fam, ail, ail.span_to)           # kanatçık dış ucu = uç kapağının iç yüzü (eğik)
    y_glove = float(_W["glove"]["end_y_m"])
    le_mat = lambda tt: M["accent"] if tt <= y_glove + 1e-6 else M["le_strip"]   # glove strake antrasit (chine okunu sürdürür)
    for part, names in plan.items():
        cc = [Cutout(cuts[n], cuts[n].span_from, cuts[n].span_to) for n in names]
        fixed, surfs, _ = lifting_part(f"{part}_{sd}", fam, wing_stations(part), cc, nf=_WING_N["nf"],
                                       na=_WING_N["na"], nn=_WING_N["nn"], ns=_WING_N["ns"], zip_end=(part == "U_Tip"),
                                       oblique_ends=True, start_plane=tip_plane if part == "U_Tip" else None,
                                       le_mat=le_mat)
        out[fixed.name] = fixed
        out.update(surfs)
    return out


def stab_parts() -> dict[str, MeshData]:
    """``U_Stab`` (iki yan tek parça, uçlar dikeylerin içine 5 mm gömülü) ve ``U_Elevator_L/R``."""
    fam = StabFamily()
    hs = P.STAB_HALF_SPAN
    ext = hs + 0.005
    hl, hr = P.hinge_line("Elevator", "L"), P.hinge_line("Elevator", "R")
    cuts = [Cutout(hr, -hr.span_to, -hr.span_from), Cutout(hl, hl.span_from, hl.span_to)]
    base = np.r_[_dense_between(-ext, -0.20, 0.02), _dense_between(-0.20, 0.20, 0.01), _dense_between(0.20, ext, 0.02)]
    st = np.r_[base, [c.t0 for c in cuts], [c.t1 for c in cuts], -hs, hs]
    fixed, surfs, _ = lifting_part("U_Stab", fam, st, cuts, nf=_TAIL_N["nf"], na=_TAIL_N["na"], nn=_TAIL_N["nn"],
                                   ns=_TAIL_N["ns"])
    out = {"U_Stab": fixed}
    out.update(surfs)
    return out


FIN_TIP_CAP = {"h_cap": 0.280, "stripe": (0.272, 0.275)}   # koyu dikey ucu + dış yüzde 3 mm turkuaz çizgi (F8). varsayım


def _recolor_fin(md: MeshData, fam: "FinFamily") -> None:
    """Dikey/dümen yüzlerini açıklık konumuna göre yeniden boyar: h ≥ ``h_cap`` antrasit uç kapağı; dış yüzde
    ``stripe`` bandı turkuaz ince çizgi (yüz merkezinin dikey açıklığı ve dış normali ile)."""
    V = np.asarray(md.verts, float)
    span, nrm_out = np.asarray(fam.span), np.asarray(fam.normal)
    mats = list(md.mats)

    def mi(name):
        if name not in mats:
            mats.append(name)
        return mats.index(name)

    fm = np.asarray(md.face_mat).copy()
    i_acc, i_str = mi(M["accent"]), mi(M["stripe"])
    a0, a1 = FIN_TIP_CAP["stripe"]
    for k, f in enumerate(md.faces):
        P3 = V[list(f)]
        c = P3.mean(0)
        hh = float((c - fam.le0) @ span)
        if hh >= FIN_TIP_CAP["h_cap"] - 1e-6:
            fm[k] = i_acc
            continue
        if a0 - 1e-6 <= hh <= a1 + 1e-6:
            n = np.cross(P3[1] - P3[0], P3[2] - P3[0])
            nb = np.linalg.norm(n)
            if nb > 0 and float(P.points_to_blender(n / nb)[..., :] @ P.vec_to_blender(nrm_out)) > 0.3:
                fm[k] = i_str
    md.mats, md.face_mat = mats, fm


def fin_parts(side: str = "L") -> dict[str, MeshData]:
    """``U_Fin_<s>`` (kök stabilize altında yuvarlak, uç yuvarlak kapalı) ve ``U_Rudder_<s>``. Üst 40 mm koyu uç kapağı
    ve dış yüzde 3 mm turkuaz ince çizgi (boya; F8)."""
    sd = "L" if side.upper() == "L" else "R"
    fam = FinFamily(sd)
    H = fam.H
    h = P.hinge_line("Rudder", sd)
    cuts = [Cutout(h, h.span_from, h.span_to)]
    root = -FIN_ROOT_ROUND * np.cos(np.linspace(0, 0.5 * math.pi, 6))
    tip = H - FIN_TIP_ROUND + FIN_TIP_ROUND * np.sin(np.linspace(0, 0.5 * math.pi, 7))
    paint = [FIN_TIP_CAP["stripe"][0], FIN_TIP_CAP["stripe"][1], FIN_TIP_CAP["h_cap"]]
    st = np.r_[root, _dense_between(0.0, H - FIN_TIP_ROUND, 0.012), tip, h.span_from, h.span_to, paint]
    fixed, surfs, _ = lifting_part(f"U_Fin_{sd}", fam, st, cuts, nf=_TAIL_N["nf"], na=_TAIL_N["na"], nn=_TAIL_N["nn"],
                                   ns=_TAIL_N["ns"], mat_lo=M["skin_top"], zip_start=True, zip_end=True)
    _recolor_fin(fixed, fam)
    out = {fixed.name: fixed}
    for k, v in surfs.items():
        v.face_mat[:] = v.mats.index(M["skin_top"]) if M["skin_top"] in v.mats else 0
        _recolor_fin(v, fam)
        out[k] = v
    return out


# =====================================================================================================
# Yüzey tabloları (fileto/kaporta için kanat ve stabilize üst/alt yüzeyi)
# =====================================================================================================
class SurfaceTable:
    """Selig kesit fonksiyonundan ``z(s, y)`` sorgusu: açıklık ızgarasında üst/alt eğriler, aralarda doğrusal."""

    def __init__(self, section_fn: Callable[[float], np.ndarray], ys: np.ndarray):
        self.ys = np.asarray(ys, float)
        self.up, self.lo = [], []
        for y in self.ys:
            sec = section_fn(float(y))
            n = (len(sec) + 1) // 2
            u, l = sec[:n][::-1], sec[n - 1:]
            self.up.append((np.maximum.accumulate(u[:, 0]), u[:, 2]))
            self.lo.append((np.maximum.accumulate(l[:, 0]), l[:, 2]))

    def _curve(self, k: int, which: str):
        return self.up[k] if which == "upper" else self.lo[k]

    def z(self, s: float, y: float, which: str = "upper") -> float:
        ay = abs(y)
        k = int(np.clip(np.searchsorted(self.ys, ay) - 1, 0, len(self.ys) - 2))
        w = (ay - self.ys[k]) / (self.ys[k + 1] - self.ys[k])
        vals = []
        for kk in (k, k + 1):
            S, Z = self._curve(kk, which)
            vals.append(float(np.interp(s, S, Z, left=np.nan, right=np.nan)))
        return (1 - w) * vals[0] + w * vals[1]

    def spanwise(self, s: float, which: str = "upper") -> tuple[np.ndarray, np.ndarray]:
        ys, zs = [], []
        for k, y in enumerate(self.ys):
            S, Z = self._curve(k, which)
            if S[0] <= s <= S[-1]:
                ys.append(y)
                zs.append(float(np.interp(s, S, Z)))
        return np.asarray(ys), np.asarray(zs)


@functools.lru_cache(maxsize=None)
def wing_table(side: str = "L") -> SurfaceTable:
    sg = 1.0 if side == "L" else -1.0
    ys = np.r_[np.arange(0.0, 0.40, 0.004), 0.40]
    return SurfaceTable(lambda y: P.wing_section(sg * y, 161), ys)


@functools.lru_cache(maxsize=None)
def stab_table() -> SurfaceTable:
    ys = np.r_[np.arange(0.0, 0.20, 0.003), 0.20]
    return SurfaceTable(lambda y: P.stab_section(y, 161), ys)


# =====================================================================================================
# Fileto ve kaportalar
# =====================================================================================================
def _body_curve(s: float) -> tuple[np.ndarray, np.ndarray, int]:
    """Gövde kesitinin iskele yarısı (tepe → chine → karın) yoğun çizgi, yay uzunluğu ve chine indisi."""
    nq = FUS_NQ_DENSE
    o = P.fuselage_outline(s, nq)
    half = o[:2 * nq - 1]
    return half, _arc(half), nq - 1


def _seg_intersect(p, d, q, e):
    A = np.array([[d[0], -e[0]], [d[1], -e[1]]])
    if abs(np.linalg.det(A)) < 1e-10:
        return None
    lam, mu = np.linalg.solve(A, q - p)
    return lam, mu


def fillet_solid(name: str, s_list: np.ndarray, table: SurfaceTable, which: str, dA_fn: Callable, dB_fn: Callable,
                 side: str = "L", mat: str = M["skin_top"], n_curve: int = 12, stop_at_chine: bool = False,
                 y_min: float = 0.02) -> MeshData:
    """Gövde ile taşıyıcı yüzey arasında içbükey fileto katısı (sabit-s kesitleri, ikinci derece Bezier).

    Her ``s``'de: kesişim C, gövde üzerinde C'den ``dA`` uzakta A (``which="upper"`` → yukarı; chine'ı
    geçmez), yüzey üzerinde ``dB`` dışarıda B; kontrol noktası A ve B teğetlerinin kesişimi. Katı, eğri ile
    iki gövdeye ``EMBED`` kadar gömülü iç noktalar arasında kapanır.
    """
    sg = 1.0 if side == "L" else -1.0
    sgn = 1.0 if which == "upper" else -1.0
    mb = MeshBuilder(name)
    rings = []
    for s in s_list:
        half, cum, i_ch = _body_curve(float(s))
        sec = P.fuselage_section(float(s))
        ys, zs = table.spanwise(float(s), which)
        keep = ys >= y_min
        ys, zs = ys[keep], zs[keep]
        inside = np.array([y < P.fuselage_half_width_at(sec, z) for y, z in zip(ys, zs)])
        k = int(np.argmax(~inside)) if (~inside).any() else len(ys) - 1
        k = max(k, 1)
        a, b = 0.0, 1.0
        p0, p1 = np.array([ys[k - 1], zs[k - 1]]), np.array([ys[k], zs[k]])
        for _ in range(40):
            mid = 0.5 * (a + b)
            q = p0 + mid * (p1 - p0)
            if q[0] < P.fuselage_half_width_at(sec, q[1]):
                a = mid
            else:
                b = mid
        C = p0 + 0.5 * (a + b) * (p1 - p0)
        dist = np.hypot(half[:, 0] - C[0], half[:, 1] - C[1])
        i_c = int(np.argmin(dist))
        dC = cum[i_c]
        dA = float(dA_fn(float(s)))
        if which == "upper":
            dA_eff = min(dA, max(dC - cum[i_ch], 0.0)) if stop_at_chine else dA
            dA_eff = max(dA_eff, 2e-4)
            A = _at_arc(half, cum, dC - dA_eff)
            tA = _unit(_at_arc(half, cum, dC - dA_eff + 1e-3) - A)
        else:
            dA_eff = max(dA, 2e-4)
            A = _at_arc(half, cum, dC + dA_eff)
            tA = _unit(_at_arc(half, cum, dC + dA_eff - 1e-3) - A)
        # yüzey eğrisi C'den dışa
        sc = np.vstack([C, np.column_stack([ys[k:], zs[k:]])])
        cs = _arc(sc)
        dB = float(np.clip(dB_fn(float(s), float(C[0])), 2e-4, cs[-1] - 1e-4))
        B = _at_arc(sc, cs, dB)
        tB = _unit(_at_arc(sc, cs, max(dB - 1e-3, 0.0)) - B)
        r = _seg_intersect(A, tA, B, tB)
        K = C if (r is None or r[0] <= 0 or r[1] <= 0 or r[0] > 4 * (dA_eff + dB)) else A + r[0] * tA
        tt = np.linspace(0.0, 1.0, n_curve + 1)[:, None]
        bez = (1 - tt) ** 2 * A + 2 * tt * (1 - tt) * K + tt ** 2 * B
        Bin = B + np.array([0.0, -sgn * EMBED])
        Cin = C + np.array([-EMBED, -sgn * EMBED])
        Ain = A + np.array([-EMBED, 0.0])
        ring = np.vstack([bez, Bin, Cin, Ain])
        rings.append(mb.add(np.column_stack([np.full(len(ring), s), sg * ring[:, 0], ring[:, 1]])))
    mb.loft(rings, mat)
    mb.cap(rings[0], mat, start=True)
    mb.cap(rings[-1], mat, start=False)
    return mb.build(35.0)


def _glove_le_y(s: float) -> float:
    """Glove HK çizgisinin ``s`` istasyonundaki açıklık konumu (36° çizgi; dışında büyük değer)."""
    g = _W["glove"]
    tan = math.tan(_rad(float(g["le_sweep_deg"])))
    return float(g["end_y_m"]) - (float(_W["le_root_s_m"]) - s) / tan


WING_FILLET = {"r_max": 0.022, "ramp_in": 0.11, "ramp_out": 0.16, "r_te": 0.004}   # kanat üstü fileto. varsayım


def wing_fillet(side: str = "L") -> MeshData:
    """``U_Fairing_Fillet_<s>``: kanat üstü–gövde filetosu. Önde chine'a kadar uzanır (chine "ok çizgisi" glove
    üzerine akar), orta veterde ~22 mm, firar kenarına doğru incelir."""
    sd = "L" if side.upper() == "L" else "R"
    tab = wing_table(sd)
    s_te = float(P.wing_station(0.09).te_s)
    # HK birleşimi: glove HK'nin gövde yanını deldiği s
    s_le = None
    for s in np.linspace(1.02, 1.12, 201):
        ys, zs = tab.spanwise(float(s), "upper")
        sec = P.fuselage_section(float(s))
        if len(ys) and any(y > P.fuselage_half_width_at(sec, z) + 0.002 for y, z in zip(ys, zs)):
            s_le = float(s)
            break
    s0, s1 = s_le + 0.002, s_te - 0.003
    F = WING_FILLET

    def dA(s):
        a = _smoothstep((s - s0) / F["ramp_in"])
        b = 1.0 - (1.0 - F["r_te"] / F["r_max"]) * _smoothstep((s - (s1 - F["ramp_out"])) / F["ramp_out"])
        return F["r_max"] * a * b + 3e-4

    def dB(s, yc):
        lim = max(0.75 * (_glove_le_y(s) - yc), 2e-4)
        return min(1.25 * dA(s) + 0.004 * _smoothstep((s - s0) / F["ramp_in"]), lim)

    ss = np.r_[s0 + (s1 - s0) * (1 - np.cos(np.linspace(0, 0.5 * math.pi, 14)))[:-1] * 0.25,
               np.linspace(s0 + 0.25 * (s1 - s0), s1, 46)]
    return fillet_solid(f"U_Fairing_Fillet_{sd}", np.unique(ss), tab, "upper", dA, dB, sd, M["skin_top"],
                        stop_at_chine=True)


def stab_fillet() -> MeshData:
    """``U_Fairing_StabFillet``: stabilize kökü–kuyruk konisi filetoları (üst ve alt, iki yan; PA-CF, boyalı: üst
    filetolar üst boya, alt filetolar alt boya — chine çizgisinde ayrılır)."""
    tab = stab_table()
    hl = P.hinge_line("Elevator", "L")
    s_hinge = hl.p_in[0]
    y_el = hl.span_from
    st0 = P.stab_station(0.055)
    s0, s1 = st0.le_s + 0.002, st0.te_s - 0.002
    parts = []
    for which, rmax in (("upper", 0.014), ("lower", 0.010)):
        def dA(s, rmax=rmax):
            a = _smoothstep((s - s0) / 0.05)
            b = 1.0 - 0.8 * _smoothstep((s - (s_hinge - 0.06)) / 0.05)
            return rmax * a * b + 3e-4

        def dB(s, yc, rmax=rmax):
            lim = (y_el - 0.0015 - yc) if s > s_hinge - 0.012 else 1.0
            return max(min(1.2 * dA(s), lim), 2e-4)

        ss = np.linspace(s0, s1, 40)
        for sd in ("L", "R"):
            parts.append(fillet_solid(f"stabfillet_{which}_{sd}", ss, tab, which, dA, dB, sd,
                                      M["skin_top"] if which == "upper" else M["skin_bottom"], y_min=0.02))
    return merge("U_Fairing_StabFillet", parts, 35.0)


RF = _W["root_fairing"]


def _fairing_ramp(s: float) -> float:
    """Kök kaportası derinlik çarpanı: uzun ön rampa → kuyu boyunca düz taban → firar kenarına teğet kapanış."""
    a, b = float(RF["s_from_m"]), float(RF["s_to_m"])
    rp = RF.get("ramp_m", {"front": 0.07, "aft": 0.05})
    return float(_smoothstep((s - a) / float(rp["front"])) * _smoothstep((b - s) / float(rp["aft"])))


def fairing_bottom_z(s: float, y: float, side: str = "L") -> float | None:
    """Kök kaportası alt yüzeyi (düz taban ``bottom_z``; ``y_to``'dan ``y_blend``'e kanada teğet geçiş)."""
    zl = wing_table(side).z(s, abs(y), "lower")
    if zl is None or not np.isfinite(zl):
        return None
    yb0, yb1 = _fairing_y_limits(s)
    zf = float(RF["bottom_z_m"])
    if abs(y) > yb1 + 1e-9:
        return None
    w = float(_smoothstep((abs(y) - yb0) / (yb1 - yb0)))
    zb = zf + (zl - zf) * w
    return _dip_into(zl, (zl - zb) * _fairing_ramp(s))


SKIN_DIP = 0.0008                         # kaporta kenarı kanadın 0,8 mm içine dalar (eş düzlemli yüzey çakışması yok)


def _dip_into(z_surface: float, depth: float) -> float:
    """Yüzeyin ``depth`` altındaki kaporta tabanı; derinlik ~0 olduğu kenarlarda ``SKIN_DIP`` kadar YUKARI (içeri)
    kayar → kaporta kanada kesişerek (teğet değil) birleşir, render'da kenar çizgisi/titreşim oluşmaz."""
    return z_surface - depth + SKIN_DIP * (1.0 - float(_smoothstep(depth / 0.0015)))


FAIRING_FLAP_CLEAR = 0.003               # kaporta dış kenarı ile iç flap ucu arası (menteşe gerisinde). varsayım


def _fairing_y_limits(s: float) -> tuple[float, float]:
    """Kaportanın ``s``'deki düz taban sonu ve kanada kavuşma ``y``'si: iç flap menteşesinin önünde içe daralır
    (flap kaportanın yanında serbestçe iner)."""
    yb0, yb1 = float(RF["y_to_m"]), float(RF["y_blend_m"])
    fl = P.hinge_line("FlapIn", "L")
    y_lim = fl.span_from - fl.gap_m - FAIRING_FLAP_CLEAR
    s_h = min(fl.p_in[0], fl.p_out[0]) - fl.nose_radius_in - fl.gap_m
    w = float(_smoothstep((s - (s_h - 0.075)) / 0.065))
    yb1e = yb1 + (y_lim - yb1) * w
    yb0e = min(yb0, yb1e - 0.015)
    mw = _G["main"]["well"]["y_m"][1]
    return max(yb0e, float(mw) + 0.002) if yb1e - 0.004 > float(mw) + 0.002 else yb0e, yb1e


def root_fairing(side: str = "L") -> MeshData:
    """``U_Fairing_Root_<s>``: kanat kökü altında düz tabanlı kaporta (ana takım kuyusunu taşır); arka-dış köşesi
    iç flap menteşesinin önünde içe daralır."""
    sd = "L" if side.upper() == "L" else "R"
    sg = 1.0 if sd == "L" else -1.0
    tab = wing_table(sd)
    ss = np.linspace(float(RF["s_from_m"]), float(RF["s_to_m"]), 52)
    mb = MeshBuilder(f"U_Fairing_Root_{sd}")
    rings = []
    for s in ss:
        yb0, yb1 = _fairing_y_limits(float(s))
        yy = np.r_[np.linspace(0.03, yb0, 20), np.linspace(yb0, yb1, 12)[1:]]
        zb = np.array([fairing_bottom_z(s, y, sd) for y in yy])
        zt = np.array([tab.z(s, y, "lower") + 0.002 for y in yy])
        ring = np.vstack([np.column_stack([yy, zb]), np.column_stack([yy[::-1][1:-1], zt[::-1][1:-1]])])
        rings.append(mb.add(np.column_stack([np.full(len(ring), s), sg * ring[:, 0], ring[:, 1]])))
    mb.loft(rings, M["skin_bottom"])
    mb.cap(rings[0], M["skin_bottom"], start=True)
    mb.cap(rings[-1], M["skin_bottom"], start=False)
    return mb.build(35.0)


UB = RF["unit_blister"]


def _blister_w(s: float, y: float) -> float:
    """Kabartma derinlik çarpanı 0…1: spec ``ramp_m`` boylarında smoothstep rampalarla düz tabana çıkar."""
    s0, s1 = float(UB["s_from_m"]), float(UB["s_to_m"])
    y0, y1 = float(UB["y_from_m"]), float(UB["y_to_m"])
    ay = abs(y)
    if not (s0 <= s <= s1 and y0 <= ay <= y1):
        return 0.0
    rp = UB.get("ramp_m", {"front": 0.38 * (s1 - s0), "aft": 0.5 * (s1 - s0), "side": 0.27 * (y1 - y0)})
    rf, ra, rs = float(rp["front"]), float(rp["aft"]), float(rp["side"])
    return float(_smoothstep((s - s0) / rf) * _smoothstep((s1 - s) / ra) * _smoothstep((ay - y0) / rs)
                 * _smoothstep((y1 - ay) / rs))


def blister_bottom_z(s: float, y: float, side: str = "L") -> float:
    zl = wing_table(side).z(s, abs(y), "lower")
    return _dip_into(zl, float(UB["depth_m"]) * _blister_w(s, y))


def unit_blister(side: str = "L") -> MeshData:
    """``U_Fairing_Blister_<s>``: ER-150 ünitesi ve pivotu için kanat altı kabartma (derinlik 9 mm)."""
    sd = "L" if side.upper() == "L" else "R"
    sg = 1.0 if sd == "L" else -1.0
    tab = wing_table(sd)
    ss = np.linspace(float(UB["s_from_m"]), float(UB["s_to_m"]), 26)
    yy = np.linspace(float(UB["y_from_m"]), float(UB["y_to_m"]), 30)
    mb = MeshBuilder(f"U_Fairing_Blister_{sd}")
    rings = []
    for s in ss:
        zb = np.array([blister_bottom_z(s, y, sd) for y in yy])
        zt = np.array([tab.z(s, y, "lower") + 0.002 for y in yy])
        ring = np.vstack([np.column_stack([yy, zb]), np.column_stack([yy[::-1], zt[::-1]])])
        rings.append(mb.add(np.column_stack([np.full(len(ring), s), sg * ring[:, 0], ring[:, 1]])))
    mb.loft(rings, M["skin_bottom"])
    mb.cap(rings[0], M["skin_bottom"], start=True)
    mb.cap(rings[-1], M["skin_bottom"], start=False)
    return mb.build(35.0)


_ER = re.search(r"(\d+)\s*×\s*(\d+)\s*×\s*(\d+)\s*mm", str(_G["product"]))
ER150_BODY = tuple(float(v) / 1000.0 for v in _ER.groups()) if _ER else (0.026, 0.102, 0.032)   # G × U × Y (spec ürün metni)


def _slot_edge_s(side: str, y: float, which: str) -> float:
    """Ana bacak yuvası açıklığının |y|'deki ön (``"front"``) ya da arka (``"aft"``) ``s`` kenarı."""
    sl = P._leg_slot(side)
    (sa0, ya0), (sa1, ya1) = sl[0], sl[1]
    (sb0, _), (sb1, _) = sl[3], sl[2]
    t_ = (abs(y) - abs(ya0)) / (abs(ya1) - abs(ya0))
    return sa0 + t_ * (sa1 - sa0) if which == "front" else sb0 + t_ * (sb1 - sb0)


def main_unit_box(side: str = "L") -> tuple[P.EnvPart, float]:
    """ER-150 ana takım ünitesi gövdesi (26 × 102 × 32 mm) yerleşimi, spec takımı: bacak yuvasının ARKASINDA
    (ön yüz = yuva arka kenarı + ``unit_clearance_m``), 102 mm açıklık boyunca (``unit_span_y_m``), 26 mm kenar düşey,
    32 mm veter boyunca; dihedral boyunca eğik, tabanı kabartma iç yüzüne (deri + 1,6 mm + pay) oturur.
    Dönüş: (zarf, dikey pay = kanat üst derisi altına kalan boşluk, m; ≥ 0 olmalı)."""
    mw = _G["main"]
    sg = 1.0 if side.upper() == "L" else -1.0
    bw, bl, bh = ER150_BODY
    h, ls = min(bw, bh), max(bw, bh)                       # düşey 26, veter boyunca 32
    y0, y1 = (float(v) for v in mw["unit_span_y_m"])
    clr = float(mw.get("unit_clearance_m", 0.0025))
    y_out = abs(P._leg_slot(side)[1][1])
    s0 = max(_slot_edge_s(side, y, "aft") for y in np.linspace(y0, min(y1, y_out), 11)) + clr
    s1 = s0 + ls
    dih = math.tan(_rad(P.WING_DIHEDRAL))
    ss, ys = np.linspace(s0, s1, 7), np.linspace(y0, y1, 11)
    tab = wing_table("L")
    base0 = max(belly_z(s, y) + 0.0021 - dih * (y - y0) for s in ss for y in ys)
    top0 = min(tab.z(s, y, "upper") - 0.0013 - dih * (y - y0) for s in ss for y in ys)
    yc = 0.5 * (y0 + y1)
    zc = base0 + 0.5 * h + dih * (yc - y0)
    a = math.atan(dih)
    axes = ((1.0, 0.0, 0.0), (0.0, sg * math.cos(a), math.sin(a)), (0.0, -sg * math.sin(a), math.cos(a)))
    env = P.EnvPart(f"er150_{side.upper()}", "box", (0.5 * (s0 + s1), sg * yc, zc), axes, (0.5 * ls, 0.5 * (y1 - y0), 0.5 * h))
    return env, float(top0 - base0 - h)


def belly_z(s: float, y: float) -> float:
    """Gövde altındaki en alçak deri yüksekliği (gövde, kök kaportası, kabartma, kanat altı) — kapaklar için."""
    side = "L" if y >= 0 else "R"
    cands = []
    zf = P.fuselage_z_at(s, y, "bottom")
    if zf is not None:
        cands.append(zf)
    if float(RF["s_from_m"]) <= s <= float(RF["s_to_m"]) and abs(y) < float(RF["y_blend_m"]):
        v = fairing_bottom_z(s, y, side)
        if v is not None:
            cands.append(v)              # dalış payı kanat alt yüzeyinin üstünde kalır → min() etkilemez
    if abs(y) <= 0.40:
        zl = wing_table(side).z(s, abs(y), "lower")
        if np.isfinite(zl):
            cands.append(zl)
            if _blister_w(s, y) > 0:
                cands.append(blister_bottom_z(s, y, side))
    return float(min(cands))


def glove_junction_s(side: str = "L") -> float:
    """Glove hücum kenarının gövde yanını deldiği istasyon (fileto ve turkuaz çizgi burada buluşur)."""
    tab = wing_table("L" if side.upper() == "L" else "R")
    for s in np.linspace(1.02, 1.12, 401):
        ys, zs = tab.spanwise(float(s), "upper")
        sec = P.fuselage_section(float(s))
        if len(ys) and any(y > P.fuselage_half_width_at(sec, z) + 0.002 for y, z in zip(ys, zs)):
            return float(s)
    return float(_W["glove"]["root_le_s_m"])


# =====================================================================================================
# Sırt hava alığı, aviyonik kapağı, işaretler, kaporta ayrıntıları
# =====================================================================================================
INTAKE_LIP_T = 0.003                    # NACA boğaz dudağı (karın derisi) kalınlığı. varsayım
INTAKE_DUCT = 0.018                     # boğazdan geriye görünen kanal cebi (koyu kapakla biter; yangın perdesine
                                        # 7 mm et kalır, kaporta ön yüzü görünmez). varsayım
INTAKE_FRAME = 0.0025                   # boğaz çerçevesi (PA-CF dudak) et kalınlığı. varsayım


def _skin_top(s: float, y: float = 0.0) -> float:
    z = P.fuselage_z_at(s, y, "top")
    return float(z if z is not None else P.fuselage_section(s).z_top)


def _skin_bottom(s: float, y: float = 0.0) -> float:
    z = P.fuselage_z_at(s, y, "bottom")
    return float(z if z is not None else P.fuselage_section(s).z_bottom)


def intake_cutter() -> MeshData:
    """Karın NACA hava alığı boolean kesicisi (``U_Fuselage``'dan çıkarılır): rampa (7°, ıraksak planform 40 → 75 mm)
    + boğazın arkasında dudak üstündeki kanal cebi. Kesilen yüzler: rampa tabanı ve yan duvarlar alt boya, kanal
    cebi koyu (``seal``). Ağız 75 mm geniş, tavan 36 mm; dudak karın eğrisini izler → ≈ 22 cm² (spec
    ``propulsion.intake.area_cm2``)."""
    I = P.intake_spec()
    s_r, s_t = float(I["ramp_s"]), float(I["throat_s"])
    w, h = 0.5 * float(I["mouth_w_m"]), float(I["mouth_h_m"])
    hw, dep = I["half_width_at"], I["depth_at"]
    ny = 9
    ss = np.r_[np.linspace(s_r - 0.002, s_t, 40), s_t + 0.0006, s_t + INTAKE_DUCT]
    mb = MeshBuilder("U_Cutter_Intake")
    rings, kinds = [], []
    for k, s in enumerate(ss):
        s = float(s)
        if s <= s_t + 1e-9:
            a = max(hw(s), 0.004)
            zb = _skin_bottom(s, 0.0)
            floor = zb + dep(s)
            lo = min(_skin_bottom(s, yy) for yy in np.linspace(0, a, 4)) - 0.012
            kind = "ramp"
        else:
            a = w
            zb = _skin_bottom(s_t, 0.0)
            floor = zb + h
            lo = None                       # dudak: kanal tabanı karın eğrisini izler (deri + 3 mm), köşeler dışarı taşmaz
            kind = "duct"
        yy = np.linspace(-a, a, ny)
        lo_y = (np.full(ny, lo) if lo is not None
                else np.array([_skin_bottom(s, float(v)) + INTAKE_LIP_T for v in yy]))
        top = np.column_stack([yy[::-1], np.maximum(floor, lo_y[::-1] + 0.0004)])   # iskeleden sancağa (üst)
        bot = np.column_stack([yy, lo_y])                                            # sancaktan iskeleye (alt)
        ring = np.vstack([top, bot])
        rings.append(mb.add(np.column_stack([np.full(len(ring), s), ring[:, 0], ring[:, 1]])))
        kinds.append(kind)
    m = 2 * ny
    for k in range(len(rings) - 1):
        dark = kinds[k + 1] == "duct"
        mb.strip(rings[k], rings[k + 1], lambda j, dark=dark: M["seal"] if dark else M["skin_bottom"])
    mb.cap(rings[0], M["skin_bottom"], start=True)
    mb.cap(rings[-1], M["seal"], start=False)
    return mb.build(None)


def intake() -> MeshData:
    """``U_Intake``: NACA boğazının PA-CF dudak çerçevesi (gövde kesiminde açılan boğazı çerçeveler; alt kenarı
    dudaktır, ön kenarı yuvarlatılmış). Sırtta artık çıkıntı yok: alık karında gömülüdür (``intake_cutter``)."""
    I = P.intake_spec()
    s_t = float(I["throat_s"])
    w, h = 0.5 * float(I["mouth_w_m"]), float(I["mouth_h_m"])
    zb = _skin_bottom(s_t, 0.0)
    t = INTAKE_FRAME
    n_c = 5

    n_b = 9

    def loop(s, a, dz0, z1, r):
        """Ağız çevresi (y, z), saat yönü tersine, tepe-iskele köşesinden: üst kenar düz (rampa tabanı), alt kenar
        karın eğrisini izler (``_skin_bottom(s, y) + dz0``) → dudak dikdörtgen köşeleriyle yuvarlak karından taşmaz."""
        zb_y = lambda y: _skin_bottom(s, y) + dz0
        r = min(r, 0.49 * (z1 - zb_y(0.0)), 0.49 * 2 * a)
        pts = []
        for cy, cz, a0 in ((a - r, z1 - r, 0.0), (-a + r, z1 - r, 90.0)):
            for kk in range(n_c + 1):
                ang = _rad(a0 + 90.0 * kk / n_c)
                pts.append((cy + r * math.cos(ang), cz + r * math.sin(ang)))
        for cy, a0 in ((-a + r, 180.0), (None, None), (a - r, 270.0)):
            if cy is None:                                   # alt kenar: köşe yayları arasında karın eğrisi
                for y in np.linspace(-a + r, a - r, n_b + 2)[1:-1]:
                    pts.append((float(y), zb_y(float(y))))
                continue
            cz = zb_y(cy) + r
            for kk in range(n_c + 1):                        # köşe yayı karın eğimiyle kaydırılır (deriye teğet)
                ang = _rad(a0 + 90.0 * kk / n_c)
                y = cy + r * math.cos(ang)
                pts.append((y, cz + r * math.sin(ang) + zb_y(y) - zb_y(cy)))
        return np.asarray(pts)

    s_list = [(s_t - 0.0004, 0.6), (s_t + 0.0008, 1.0), (s_t + 0.010, 1.0)]
    mb = MeshBuilder("U_Intake")
    rings = []
    for s, f in s_list:                     # ön kenar yuvarlak: dış çevre önde içe çekilir
        inner = loop(s, w, INTAKE_LIP_T, zb + h, 0.004)
        outer = loop(s, w + t, -0.0004, zb + h + t, 0.004 + t)
        o = inner + (outer - inner) * f
        rings.append((mb.add(np.column_stack([np.full(len(o), s), o])),
                      mb.add(np.column_stack([np.full(len(inner), s), inner]))))
    for k in range(len(rings) - 1):
        mb.strip(rings[k][0], rings[k + 1][0], M["pacf"])
        mb.strip(rings[k + 1][1], rings[k][1], M["pacf"])
    mb.strip(rings[0][1], rings[0][0], M["pacf"])
    mb.strip(rings[-1][0], rings[-1][1], M["pacf"])
    return mb.build(40.0)


def _hatch_geom():
    H = _D["hatch"]
    s0, s1, w = float(H["s_from_m"]), float(H["s_to_m"]), float(H["half_width_m"])
    k = w * math.tan(_rad(float(H["chevron_deg"])))
    return s0, s1, w, k, float(H["bulge_m"])


HATCH_FACET = {"v_top": 0.46, "d_end": 0.045, "rim": 0.0006}   # üst faset genişliği, uç rampaları, kenar basamağı. varsayım
HATCH_FRIT = 0.006                       # iç yüzde 6 mm opak siyah "frit" kenar bandı (cep duvarlarını gizler). varsayım
HATCH_FRAME = {"w": 0.006, "lift": 0.00025, "gap": 0.0004, "n_fast": 12, "d_fast": 0.0022}   # antrasit çerçeve (R12: 4 → 6 mm, üstten okunur). varsayım


def hatch() -> MeshData:
    """``U_Hatch``: füme PETG aviyonik kapağı — gövde sırtına neredeyse gömülü (dış yüz deriden en çok
    ``rim`` + ``bulge`` ≈ 1,6 mm yukarıda; AERO-16), 36° şevron ön/arka uçlu, üst + iki yan hafif faset, 1,6 mm kabuk.
    İç yüzde 6 mm opak siyah frit bandı (``seal``) cep duvarlarını gizler. Plan çokgeni ``params.hatch_outline()``;
    çevresinde antrasit çerçeve ve vida başları ``hatch_frame()``."""
    s0, s1, w, k, bulge = _hatch_geom()
    L = s1 - s0 - k
    vc, d_end, rim = HATCH_FACET["v_top"], HATCH_FACET["d_end"], HATCH_FACET["rim"]
    uc = d_end / L
    vf = 1.0 - HATCH_FRIT / w
    uf = HATCH_FRIT / (L * math.cos(_rad(float(_D["hatch"]["chevron_deg"]))))
    v = np.unique(np.round(np.r_[np.linspace(-1, -vc, 6), np.linspace(-vc, vc, 11), np.linspace(vc, 1, 6), -vf, vf], 9))
    u = np.unique(np.round(np.r_[np.linspace(0, uc, 5), np.linspace(uc, 1 - uc, 24), np.linspace(1 - uc, 1, 5), uf, 1 - uf], 9))
    VV, UU = np.meshgrid(v, u, indexing="ij")
    S = s0 + k * np.abs(VV) + UU * L
    Y = VV * w
    hh = bulge * np.minimum(1, (1 - np.abs(VV)) / (1 - vc)) * np.minimum(1, UU / uc) * np.minimum(1, (1 - UU) / uc) + rim
    Zs = np.array([[_skin_top(s, y) for s, y in zip(rs, ry)] for rs, ry in zip(S, Y)])
    Zo = Zs + hh
    mb = MeshBuilder("U_Hatch")
    io = mb.add(np.dstack([S, Y, Zo]))
    ii = mb.add(np.dstack([S, Y, Zo - HATCH_T]))
    nv, nu = io.shape
    mat = M["hatch"]
    for i in range(nv - 1):
        for j in range(nu - 1):
            mb.face((io[i, j], io[i + 1, j], io[i + 1, j + 1], io[i, j + 1]), mat)
            vm, um = 0.5 * (abs(v[i]) + abs(v[i + 1])), 0.5 * (u[j] + u[j + 1])
            frit = vm > vf or um < uf or um > 1 - uf
            mb.face((ii[i, j], ii[i, j + 1], ii[i + 1, j + 1], ii[i + 1, j]), M["seal"] if frit else mat)
    ring_o = list(io[0, :]) + list(io[1:, -1]) + list(io[-1, -2::-1]) + list(io[-2:0:-1, 0])
    ring_i = list(ii[0, :]) + list(ii[1:, -1]) + list(ii[-1, -2::-1]) + list(ii[-2:0:-1, 0])
    mb.strip(ring_o, ring_i, M["seal"])
    # faset kırık çizgileri keskin
    for vv in (-vc, vc):
        i = int(np.argmin(np.abs(v - vv)))
        for j in range(nu - 1):
            mb.sharp.append((int(io[i, j]), int(io[i, j + 1])))
    for uu in (uc, 1 - uc):
        j = int(np.argmin(np.abs(u - uu)))
        for i in range(nv - 1):
            mb.sharp.append((int(io[i, j]), int(io[i + 1, j])))
    return mb.build(25.0)


def hatch_frame() -> MeshData:
    """``U_Hatch_Frame``: kapak çevresinde 6 mm antrasit çerçeve bandı (deriye izdüşürülmüş boya kalınlığında ince katı,
    kapakla arasında 0,4 mm koyu ayrım çizgisi) ve üzerinde ≈ 60 mm aralıklı 12 adet Ø2,2 mm gömme vida başı."""
    F = HATCH_FRAME
    inner = poly_resample(poly_offset(P.hatch_outline(), -F["gap"]), 0.006)
    outer = poly_offset(inner, -F["w"])
    mb = MeshBuilder("band")

    def lift(p, dz):
        return np.array([[s, y, _skin_top(s, y) + dz] for s, y in p])

    ob, ot = mb.add(lift(outer, -0.0003)), mb.add(lift(outer, F["lift"]))
    it, ib = mb.add(lift(inner, F["lift"])), mb.add(lift(inner, -0.0003))
    mb.loft([ob, ot, it, ib, ob], M["accent"])
    parts = [mb.build(30.0)]
    mid = 0.5 * (inner + outer)
    cum = _arc(np.vstack([mid, mid[:1]]))
    n = int(F["n_fast"])
    for k in range(n):
        q = _at_arc(np.vstack([mid, mid[:1]]), cum, (k + 0.5) * cum[-1] / n)
        z = _skin_top(q[0], q[1]) + F["lift"]
        r = 0.5 * F["d_fast"]
        md = lathe(np.array([[-0.0003, 0.0], [-0.0003, r], [0.00025, r], [0.00045, 0.6 * r], [0.0005, 0.0]]), 12,
                   "fastener", M["steel"], "z", space="spec")
        md.verts = md.verts + np.array([q[0], q[1], z])
        parts.append(md.oriented())
    return merge("U_Hatch_Frame", parts, 30.0)


def hatch_pocket_floor_z() -> float:
    gn = [a for a in P.antennas() if a.kind == "puck"]
    return float(min(a.pos[2] for a in gn)) - 0.0005


def hatch_cutter() -> MeshData:
    """Kapak altı aviyonik cebi için boolean kesici (kapak çevresinden 3 mm içeride; taban GNSS tablası)."""
    poly = poly_offset(P.hatch_outline(), 0.003)
    return prism("U_Cutter_Hatch", poly, hatch_pocket_floor_z(), _skin_top(0.7) + 0.06, M["pacf"])


STRIPE_W = float(_D["markings"]["stripe_m"])
STRIPE_LIFT = 0.00025                   # boya kalınlığı (görsel; z-çakışmasını önler). varsayım


def stripe(side: str = "L") -> MeshData:
    """``U_Stripe_<s>``: turkuaz 3 mm ince çizgi — chine bandının hemen altında burundan ilerler, ``s = 0,88``'de
    chine'dan ayrılıp 9° alçalarak glove hücum kenarı köküne iner (boya; baskıda yok)."""
    sd = "L" if side.upper() == "L" else "R"
    sg = 1.0 if sd == "L" else -1.0
    li = _F["chine"]["stripe_lead_in"]
    s_lead, slope = float(li["s_from_m"]), math.tan(_rad(float(li["slope_deg"])))
    s_end = glove_junction_s(sd) + 0.004
    off = CHINE_BAND_HALF + 0.0025 + 0.5 * STRIPE_W

    def zc(s):
        sec = P.fuselage_section(min(s, s_lead))
        z = sec.z_chine - off * 0.93
        return z if s <= s_lead else z - slope * (s - s_lead)

    ss = np.r_[np.linspace(0.06, s_lead - 0.02, 70), np.linspace(s_lead - 0.02, s_end, 40)[1:]]
    mb = MeshBuilder(f"U_Stripe_{sd}")
    rings = []
    for s in ss:
        sec = P.fuselage_section(float(s))
        pts = []
        for dz in (-0.5 * STRIPE_W, 0.5 * STRIPE_W):
            z = zc(float(s)) + dz
            y = P.fuselage_half_width_at(sec, z)
            dy = (P.fuselage_half_width_at(sec, z + 1e-4) - P.fuselage_half_width_at(sec, z - 1e-4)) / 2e-4
            n = _unit(np.array([1.0, -dy]))
            pts.append((np.array([y, z]), n))
        (p0, n0), (p1, n1) = pts
        ring = np.array([p0 - 0.0002 * n0, p1 - 0.0002 * n1, p1 + STRIPE_LIFT * n1, p0 + STRIPE_LIFT * n0])
        rings.append(mb.add(np.column_stack([np.full(4, s), sg * ring[:, 0], ring[:, 1]])))
    mb.loft(rings, M["stripe"])
    mb.cap(rings[0], M["stripe"], start=True)
    mb.cap(rings[-1], M["stripe"], start=False)
    return mb.build(30.0)


CHEVRON = {"y_c": 1.30, "span": 0.26, "w": 0.026, "x0": 0.10}   # sol kanat altı yönelim şevronu: merkez y, açıklık,
                                                                  # kol genişliği, tepe veter oranı. Kolların ucu kanatçık
                                                                  # menteşe oyuğunun ≥ 7 mm önünde (AERO-14). varsayım


def chevron() -> MeshData:
    """``U_Marking_Chevron``: yalnız SOL kanat altında antrasit yönelim şevronu (36° kollar, tepe öne bakar;
    asimetrik alt işaret — RC pilotuna yönelim). Kanat alt yüzeyine 0,25 mm kabartılmış boya katmanı."""
    c = CHEVRON
    sweep = math.tan(_rad(float(_W["glove"]["le_sweep_deg"])))
    ys = np.linspace(c["y_c"] - 0.5 * c["span"], c["y_c"] + 0.5 * c["span"], 41)
    sts = {float(y): P.wing_station(float(y)) for y in ys}
    st0 = P.wing_station(c["y_c"])
    apex = st0.le_s + c["x0"] * st0.chord
    mb = MeshBuilder("U_Marking_Chevron")
    rings = []
    for y in ys:
        sf = apex + abs(y - c["y_c"]) * sweep                 # ön kenar (kol), 36° geriye
        ss = np.linspace(sf, sf + c["w"] / math.cos(math.atan(sweep)), 4)
        pts = []
        for sv in ss:
            z = P.wing_surface_z(float(sv), float(y), "lower")
            pts.append((sv, y, z))
        pts = np.asarray(pts)
        outer = pts - np.array([0.0, 0.0, STRIPE_LIFT])
        inner = pts + np.array([0.0, 0.0, 0.0003])
        ring = np.vstack([outer, inner[::-1]])
        rings.append(mb.add(ring))
    mb.loft(rings, M["accent"])
    mb.cap(rings[0], M["accent"], start=True)
    mb.cap(rings[-1], M["accent"], start=False)
    return mb.build(30.0)


def _cowl_y_at(s: float, z: float, side: str) -> tuple[float, np.ndarray]:
    """Kaporta (yanak kabartısı dahil) yüzeyinin ``(s, z)``'deki |y| değeri ve dış normal (y, z)."""
    yz = _cheek_offset(s, fuselage_ring_yz(s))
    nrm = _ring_normals_yz(yz)
    sg = 1.0 if side == "L" else -1.0
    sel = yz[:, 0] * sg > 1e-6
    a, nn = yz[sel], nrm[sel]
    order = np.argsort(a[:, 1])
    zz = a[order, 1]
    return (float(np.interp(z, zz, np.abs(a[order, 0]))),
            _unit(np.array([abs(float(np.interp(z, zz, nn[order, 0]))), float(np.interp(z, zz, nn[order, 1]))])))


def _louver_slots() -> list[tuple[np.ndarray, np.ndarray, np.ndarray]]:
    """Sağ yanak panjur yarıkları: her biri için yarık çevresi üzerindeki deri noktaları (k, 3), dış normaller (k, 3)
    ve yarık ekseni boyunca (geriye-aşağı, 36°) birim yön. Yarık 3 × 24 mm, köşeleri yuvarlak."""
    c = _PR["cowl"]["cheek_right"]
    n = int(c["louvers"])
    ang = _rad(float(c["louver_angle_deg"]))
    w_sl, l_sl = (float(v) for v in c.get("louver_slot_m", (0.003, 0.024)))
    d = np.array([math.sin(ang), -math.cos(ang)])          # (s, z): yarık ekseni, geriye-aşağı
    e = np.array([math.cos(ang), math.sin(ang)])           # (s, z): yarık eni, geriye-yukarı
    zc = 0.5 * (float(c["z_from_m"]) + float(c["z_to_m"]))
    s_a, s_b = float(c["s_from_m"]), float(c["s_to_m"])
    centers = np.linspace(s_a + 0.30 * (s_b - s_a), s_a + 0.66 * (s_b - s_a), n)
    r = 0.5 * w_sl
    ring2 = []
    for cx, a0 in ((0.5 * l_sl - r, -90.0), (-(0.5 * l_sl - r), 90.0)):      # stadyum çevresi (d, e)
        for k in range(9):
            th = _rad(a0 + 180.0 * k / 8)
            ring2.append((cx + r * math.cos(th), r * math.sin(th)))
    ring2 = np.asarray(ring2)
    out = []
    for sc in centers:
        pts, nrms = [], []
        for a_, b_ in ring2:
            s = sc + a_ * d[0] + b_ * e[0]
            z = zc + a_ * d[1] + b_ * e[1]
            y, nrm = _cowl_y_at(s, z, "R")
            pts.append((s, -y, z))
            nrms.append((0.0, -nrm[0], nrm[1]))
        out.append((np.asarray(pts), _unit(np.asarray(nrms)), np.array([d[0], 0.0, d[1]])))
    return out


def _band_solid(name: str, inner: np.ndarray, outer: np.ndarray, n_in: np.ndarray, n_out: np.ndarray,
                lo: float, hi: float, mat: str) -> MeshData:
    """İki eş çevre arasında (iç/dış) normal boyunca ``lo``…``hi`` kalınlıkta kapalı çerçeve katısı."""
    mb = MeshBuilder(name)
    ob = mb.add(outer + n_out * lo)
    ot = mb.add(outer + n_out * hi)
    it = mb.add(inner + n_in * hi)
    ib = mb.add(inner + n_in * lo)
    mb.loft([ob, ot, it, ib, ob], mat)
    return mb.build(30.0)


def cowl_louvers() -> MeshData:
    """``U_Cowl_Louvers``: sağ yanakta 6 adet 36° panjur yarığının 0,8 mm kabarık PA-CF dudak çerçeveleri (yarığın
    kendisi ``louver_cutters`` ile kaportadan 2 mm derin oyulur; iç yüz koyu PA-CF)."""
    c = _PR["cowl"]["cheek_right"]
    lip = float(c.get("louver_lip_m", 0.0008))
    parts = []
    for pts, nrm, dirv in _louver_slots():
        cen = pts.mean(0)
        rad = _unit(pts - cen) - nrm * np.sum(_unit(pts - cen) * nrm, axis=1, keepdims=True)
        rad = _unit(rad)
        outer = pts + rad * 0.0012
        parts.append(_band_solid("lip", pts, outer, nrm, nrm, -0.0008, lip, M["pacf"]))
    return merge("U_Cowl_Louvers", parts, 30.0)


def louver_cutters() -> list[MeshData]:
    """Panjur yarıkları için kaporta kesicileri: 1,6 mm kabuğu boydan boya delen açık yarık (R01; soğutma havası
    çıkışı — baskıdaki ``louver_slot_holes`` ile aynı) + dışarıda pay; kesilen yüzler koyu PA-CF."""
    c = _PR["cowl"]["cheek_right"]
    dep = max(float(c.get("louver_recess_m", 0.002)), COWL_WALL + 0.0035)
    out = []
    for k, (pts, nrm, _) in enumerate(_louver_slots()):
        mb = MeshBuilder(f"U_Cutter_Louver_{k}")
        a = mb.add(pts - nrm * dep)
        b = mb.add(pts + nrm * 0.004)
        mb.strip(a, b, M["pacf"])
        mb.cap(a, M["pacf"], start=True)
        mb.cap(b, M["pacf"], start=False)
        out.append(mb.build(None))
    return out


def cooling_exit_cutter() -> MeshData:
    """Kaporta çenesindeki arkaya bakan soğutma çıkış yarığı (60 × 10 mm) kesicisi: dik çene duvarını (s ≈ 2,180–2,185)
    boydan boya geçip kaporta boşluğuna açılan prizma (R01: eski 12 mm kör cep yerine gerçek açıklık). Sahnede
    ``U_Cowl`` kabuğundan, baskıda kaporta kabuğundan çıkarılır; kesilen yüzler koyu (``seal``)."""
    ce = _PR["cowl"]["cooling_exit"]
    w, h = 0.5 * float(ce["w_m"]), float(ce["h_m"])
    s_back = float(_PR["exhaust_ring"]["s_from_m"])
    z_b = P.fuselage_section(s_back - 0.016).z_bottom
    z0 = z_b + 0.011
    poly = np.array([(s_back - 0.026, -w + 0.004), (s_back - 0.026, w - 0.004), (s_back + 0.004, w), (s_back + 0.004, -w)])
    return prism("U_Cutter_CoolingExit", poly, z0, z0 + h, M["seal"])


def cooling_exit_check(n: int = 9) -> dict:
    """Çene yarığının gerçekten AÇIK olduğunu kesitlerle doğrular (R01): yarık kesitindeki her (y, z) doğrusu boyunca
    (s ekseni) kaporta eti tek bir aralıktır (dik çene duvarı) ve kesici prizma bu aralığı iki yandan aşar (önde boşluğa,
    arkada dışarıya açılır); prizma alt/üst yüzü çene duvarı dışında ete girmez (karın delinmez). Dönüş: ``through``
    (bool), ``wall_s`` (etin s aralığı), ``prism_s``, ``min_overlap_m`` (prizmanın duvarı aşma payı, iki yanda en az)."""
    ss, O = _cowl_grid()
    s_in, I = _cowl_inner()
    s0, s_end = _cowl_s_range()
    md = cooling_exit_cutter()
    V = np.asarray(md.verts, float)
    ps0, ps1 = float(V[:, 0].min()), float(V[:, 0].max())
    z0, z1 = float(V[:, 2].min()), float(V[:, 2].max())
    w = float(np.abs(V[:, 1]).max()) - 0.004
    sl = np.linspace(ps0, min(ps1, s_end - 1e-6), 400)
    ok, lo, hi, ov = True, math.inf, -math.inf, 1.0
    for y in np.linspace(-0.92 * w, 0.92 * w, n):
        for z in np.linspace(z0 + 0.0005, z1 - 0.0005, max(3, n // 2)):
            mat = []
            for s in sl:
                if s >= s_end - COWL_WALL:
                    inside_o = bool(_point_in_poly(_ring_at(ss, O, float(s)), np.array([[y, z]]))[0])
                    mat.append(inside_o and math.hypot(y, z - P.PROP.hub[2]) > cowl_open_r())
                    continue
                inside_o = bool(_point_in_poly(_ring_at(ss, O, float(s)), np.array([[y, z]]))[0])
                inside_i = bool(_point_in_poly(_ring_at(s_in, I, float(s)), np.array([[y, z]]))[0])
                mat.append(inside_o and not inside_i)
            mat = np.asarray(mat)
            if not mat.any():
                continue
            idx = np.flatnonzero(mat)
            if np.any(np.diff(idx) > 1):                      # birden çok et aralığı → karın/yan duvar da kesiliyor
                ok = False
            a, b = float(sl[idx[0]]), float(sl[idx[-1]])
            lo, hi = min(lo, a), max(hi, b)
            ov = min(ov, a - ps0, ps1 - b)
            if not (ps0 < a and b < ps1) or mat[0]:           # prizma önde boşlukta başlamalı
                ok = False
    return {"through": bool(ok and ov > 0.002), "wall_s": (lo, hi), "prism_s": (ps0, ps1), "min_overlap_m": ov}


def _tube_along(name: str, base: np.ndarray, direction: np.ndarray, length: float, ro: float, ri: float,
                back: float, mat: str, nseg: int = 32, scarf_deg: float = 0.0) -> MeshData:
    """``base``'den ``direction`` boyunca boru (``back`` kadar geriye gömülü). ``scarf_deg``: çıkış ucu bu açıyla
    eğik kesilir (uzun kenar yukarı-öne; egzoz gazını gövdeden uzağa yönlendirir)."""
    d = _unit(direction)
    e1 = _unit(np.cross(d, [0.0, 0.0, 1.0]) if abs(d[2]) < 0.9 else np.cross(d, [1.0, 0.0, 0.0]))
    e2 = np.cross(d, e1)
    prof = np.array([[-back, ri], [-back, ro], [length - 0.0015, ro], [length, ro - 0.0012], [length, ri + 0.0003],
                     [length - 0.002, ri]])
    loc = lathe(prof, nseg, name, mat, "x", closed_profile=True)
    x = loc.verts[:, 0].copy()
    if scarf_deg:
        f = np.clip((x + back) / (length + back), 0.0, 1.0) ** 3
        x = x + f * math.tan(_rad(scarf_deg)) * loc.verts[:, 2]
    V = base + np.outer(x, d) + np.outer(loc.verts[:, 1], e1) + np.outer(loc.verts[:, 2], e2)
    md = MeshData(name, V, loc.faces, loc.face_mat, loc.mats, "spec", 30.0)
    return md.oriented()


def muffler_pipe_axis() -> tuple[np.ndarray, np.ndarray, float, float]:
    """Susturucu çıkış borusu ekseni (R05): ``(başlangıç, birim yön, kaporta dış yüzüne kadar boy, toplam boy)`` (spec).
    Boru susturucunun dış yüzünden başlar, kaporta yanağını deler ve ``stick_out_m`` kadar dışarı taşar."""
    f = P.muffler_outlet()
    p0 = np.asarray(f.pos, float)
    d = _unit(np.asarray(f.direction, float))
    ss, O = _cowl_grid()
    s0, s_end = _cowl_s_range()
    t_exit = None
    for t in np.arange(0.0, 0.12, 0.0005):
        q = p0 + t * d
        if q[0] >= s_end or q[0] <= s0 or not bool(_point_in_poly(_ring_at(ss, O, float(q[0])), q[1:][None])[0]):
            t_exit = float(t)
            break
    t_exit = 0.03 if t_exit is None else t_exit
    return p0, d, t_exit, t_exit + float(f.params.get("stick_out_m", 0.020))


def muffler_pipe_tube() -> MeshData:
    """Susturucu çıkış borusu (R05): susturucu dış yüzünden başlar, yanağı deler, 20 mm dışarı taşar; 45° eğik kesik
    uç (``exhaust``). ``muffler_pipe`` bunu ısı kalkanıyla birleştirir; printprep ısı kuralı ikisini ayrı adlandırır."""
    f = P.muffler_outlet()
    p0, d, t_exit, L_tot = muffler_pipe_axis()
    ro, ri = (0.5 * float(v) for v in f.params.get("d_m", (0.0124, 0.009)))
    return _tube_along("pipe", p0, d, L_tot, ro, ri, 0.002, M["exhaust"], scarf_deg=float(f.params.get("scarf_deg", 0.0)))


def muffler_pipe() -> MeshData:
    """``U_Exhaust_Muffler``: sol susturucu kabartısının arka-alt bölgesinden aşağı-dışa-geriye bakan çıkış borusu
    (``muffler_pipe_tube``) ve borunun çıktığı yerde kaporta derisinden 0,5 mm aralıkla oturan 30 × 20 × 0,5 mm ısı
    kalkanı plakası (``muffler_shield``)."""
    return merge("U_Exhaust_Muffler", [muffler_pipe_tube(), muffler_shield()], 30.0)


def muffler_shield() -> MeshData:
    """Isı kalkanı plakası (0,5 mm Al): borunun kaporta derisinden çıktığı yerde, yanak dış yüzünden 0,5 mm aralıkla
    (yanak yan yüzünde kalır, dik arka yüze sarılmaz)."""
    f = P.muffler_outlet()
    p0, d, t_exit, L_tot = muffler_pipe_axis()
    q = p0 + t_exit * d                                   # borunun kaporta derisinden çıktığı nokta
    s_x, z_x = float(q[0]), float(q[2])
    L, H, T = (float(v) for v in f.params.get("shield_m", (0.030, 0.020, 0.0005)))
    s1 = min(s_x + 0.25 * L, _cowl_s_range()[1] - 0.013)   # plaka yanak yan yüzünde kalır (dik arka yüze sarılmaz)
    s0 = s1 - L
    z0, z1 = z_x - 0.5 * H, z_x + 0.5 * H
    poly = np.array([(s0, z0), (s1, z0 + 0.002), (s1, z1 - 0.002), (s0, z1)])

    def y_out(ss_, zz):
        return _cowl_y_at(float(ss_), float(zz), "L")[0] + 0.0010

    mb = MeshBuilder("shield")
    nu, nv = 12, 8
    U = np.linspace(0, 1, nu)
    Vv = np.linspace(0, 1, nv)
    grid = np.array([[poly[0] + (poly[1] - poly[0]) * u + (poly[3] - poly[0]) * v + (poly[2] - poly[1] - poly[3] + poly[0]) * u * v
                      for v in Vv] for u in U])
    outer = np.array([[(g[0], y_out(g[0], g[1]), g[1]) for g in row] for row in grid])
    inner = outer - np.array([0.0, T, 0.0])
    io, ii = mb.add(outer), mb.add(inner)
    for i in range(nu - 1):
        for j in range(nv - 1):
            mb.face((io[i, j], io[i + 1, j], io[i + 1, j + 1], io[i, j + 1]), M["exhaust"])
            mb.face((ii[i, j], ii[i, j + 1], ii[i + 1, j + 1], ii[i + 1, j]), M["exhaust"])
    rgo = list(io[0, :]) + list(io[1:, -1]) + list(io[-1, -2::-1]) + list(io[-2:0:-1, 0])
    rgi = list(ii[0, :]) + list(ii[1:, -1]) + list(ii[-1, -2::-1]) + list(ii[-2:0:-1, 0])
    mb.strip(rgo, rgi, M["exhaust"])
    return mb.build(40.0)


def muffler_pipe_hole(clear: float = 0.0015) -> MeshData:
    """Baskı: sol yanakta çıkış borusunun geçtiği delik katısı (boru dış yarıçapı + ``clear``), kaporta etini
    boydan boya keser (printprep ``cowl_cheek_L`` deliği)."""
    f = P.muffler_outlet()
    p0, d, t_exit, _ = muffler_pipe_axis()
    r = 0.5 * float(f.params.get("d_m", (0.0124, 0.009))[0]) + clear
    a0, a1 = max(t_exit - 0.012, 0.0), t_exit + 0.006
    loc = lathe(np.array([[a0, 0.0], [a0, r], [a1, r], [a1, 0.0]]), 32, "U_Cutter_MufflerPipe", M["exhaust"], "x")
    e1 = _unit(np.cross(d, [0.0, 0.0, 1.0]))
    e2 = np.cross(d, e1)
    V = p0 + np.outer(loc.verts[:, 0], d) + np.outer(loc.verts[:, 1], e1) + np.outer(loc.verts[:, 2], e2)
    return MeshData("U_Cutter_MufflerPipe", V, loc.faces, loc.face_mat, loc.mats, "spec", None).oriented()


SCUFF_EMBED_M = 0.0006                   # pabuç üst yüzü kaporta etine bu kadar gömülü (yapışma yüzü; 1,6 mm etin içinde)


def scuff_pad_plan() -> np.ndarray:
    """Pabuç plan izi ``(n, 2)`` ``(s, y)``: yuvarlatılmış dikdörtgen (süperelips, n = 4), spec ``scuff_pad`` merkezi
    ``s_m``/``y_m`` ve ``size_m`` (boy × en). Arka ucu kaporta alt yüzünün çene köşesine (s ≈ 2,178) kadar uzanır."""
    sp = _PR["cowl"]["scuff_pad"]
    sc, yc = float(sp["s_m"]), float(sp.get("y_m", 0.0))
    a, b = (0.5 * float(v) for v in sp.get("size_m", (0.048, 0.028)))
    th = np.linspace(0.0, 2.0 * math.pi, 48, endpoint=False)
    c, sn = np.cos(th), np.sin(th)
    return np.column_stack([sc + a * np.sign(c) * np.abs(c) ** 0.5, yc + b * np.sign(sn) * np.abs(sn) ** 0.5])


def scuff_pad() -> MeshData:
    """``U_ScuffPad``: kaporta altında değiştirilebilir koyu PA-CF sürtünme pabucu (kuyruk çarpmasında ilk değen
    nokta: kaporta alt yüzünün çene köşesini örter; ısıya dayanıklı — spec ``scuff_pad.material``). Dış (aşınma) yüzü
    GERÇEK kaporta dış alt yüzünden (``cowl_bottom_z``: yanak kabartıları dahil) ``t_m`` aşağıda; üst yüzü kaporta
    etine ``SCUFF_EMBED_M`` gömülü. Eski pabuç gövde kesitine göre konduğu için kabartılı kaporta yüzünün ≈ 1 mm
    İÇİNDE kalıyordu (görünmez ve ilk temas değil)."""
    sp = _PR["cowl"]["scuff_pad"]
    t = float(sp["t_m"])

    def zb(s, y):
        z = cowl_bottom_z(s, y)
        return (z if z is not None else P.fuselage_section(s).z_bottom) - t

    mat = M["tpu"] if str(sp.get("material", "PA-CF")).upper() == "TPU" else M["pacf"]
    return heightfield_panel("U_ScuffPad", scuff_pad_plan(), zb, t + SCUFF_EMBED_M, mat, mat, n_rows=14,
                             col_step=0.002)


# =====================================================================================================
# İtki: pervane ve spinner (yerel: X = mil ekseni geriye, pala 1 = +Z)
# =====================================================================================================
_BLADE_TAB = np.array([          # r/R, veter (m), t/c, kamber — 16x8 kayın itici, görsel. varsayım
    [0.06, 0.017, 0.60, 0.000],
    [0.12, 0.024, 0.38, 0.015],
    [0.20, 0.032, 0.21, 0.035],
    [0.30, 0.036, 0.150, 0.045],
    [0.45, 0.034, 0.120, 0.045],
    [0.60, 0.030, 0.105, 0.042],
    [0.75, 0.025, 0.095, 0.040],
    [0.90, 0.019, 0.085, 0.035],
    [1.00, 0.016, 0.080, 0.030],
])


def _naca_cambered(x: np.ndarray, t: float, m: float, p: float = 0.40):
    yt = 5 * t * (0.2969 * np.sqrt(x) - 0.1260 * x - 0.3516 * x ** 2 + 0.2843 * x ** 3 - 0.1036 * x ** 4)
    yc = np.where(x < p, m / p ** 2 * (2 * p * x - x ** 2), m / (1 - p) ** 2 * (1 - 2 * p + 2 * p * x - x ** 2))
    dy = np.where(x < p, 2 * m / p ** 2 * (p - x), 2 * m / (1 - p) ** 2 * (p - x))
    th = np.arctan(dy)
    return (x - yt * np.sin(th), yc + yt * np.cos(th)), (x + yt * np.sin(th), yc - yt * np.cos(th))


def prop(nr: int = 40, nc: int = 18) -> MeshData:
    """``U_Prop``: iki palalı 16x8 itici pervane + göbek. Sabit geometrik hatve (helis): β(r) = atan(P / 2πr);
    emme yüzü öne (uçağa) bakar; + dönüş yerel X (geriye) etrafında = arkadan bakınca saat yönü tersi.
    Pala uçlarında 20 mm turuncu uyarı bandı."""
    pr = P.PROP
    R = pr.radius
    band = float(_PR["prop"]["tip_band_m"])
    rr = 0.012 + (R - 0.012) * np.sin(0.5 * math.pi * np.linspace(0, 1, nr))
    rR = rr / R
    chord = P.pchip(_BLADE_TAB[:, 0], _BLADE_TAB[:, 1], rR)
    tc = P.pchip(_BLADE_TAB[:, 0], _BLADE_TAB[:, 2], rR)
    cam = P.pchip(_BLADE_TAB[:, 0], _BLADE_TAB[:, 3], rR)
    r_round = 0.94 * R
    q = np.clip((rr - r_round) / (R - r_round), 0, 1)
    cf = np.maximum(np.sqrt(1 - q ** 2), 0.40)
    tf = np.sqrt(np.clip(1 - q ** 2, 0, 1))
    x = 0.5 * (1 - np.cos(np.linspace(0, math.pi, nc + 1)))
    mb = MeshBuilder("U_Prop", "local")
    m = 2 * nc
    for blade in (0, 1):
        rings = []
        for k, r in enumerate(rr):
            c = chord[k] * cf[k]
            beta = math.atan(pr.pitch / (2 * math.pi * max(r, 0.035)))
            (xu, zu), (xl, zl) = _naca_cambered(x, max(tc[k] * tf[k], 1e-4), cam[k])
            xs = np.r_[xu[::-1], xl[1:-1]]
            zs = np.r_[zu[::-1], zl[1:-1]]
            ch = np.array([math.sin(beta), math.cos(beta), 0.0])
            nh = np.array([-math.cos(beta), math.sin(beta), 0.0])
            pts = np.array([0.0, 0.0, r]) + np.outer((xs - 0.33) * c, ch) + np.outer(zs * c, nh)
            if blade == 1:
                pts = pts * np.array([1.0, -1.0, -1.0])
            if k == nr - 1:
                half = mb.add(pts[:nc + 1])
                idx = np.r_[half, half[1:nc][::-1]]
            else:
                idx = mb.add(pts)
            rings.append(idx)
        for k in range(nr - 1):
            mt = M["warning"] if rr[k] >= R - band - 1e-9 else M["prop"]
            mb.strip(rings[k], rings[k + 1], mt)
        mb.cap(rings[0], M["prop"], start=True)
    blades = mb.build(30.0)
    hub = lathe(np.array([[-0.5 * pr.hub_len, 0.0], [-0.5 * pr.hub_len, 0.5 * pr.hub_d], [0.5 * pr.hub_len, 0.5 * pr.hub_d],
                          [0.5 * pr.hub_len, 0.0]]), 32, "hub", M["prop"], "x")
    return merge("U_Prop", [blades, hub], 30.0)


SPINNER_BACKPLATE = 0.0025              # spinner taban plakası dudağı, spinner_base_s'nin önünde. varsayım


def spinner() -> MeshData:
    """``U_Spinner``: Ø64 alüminyum spinner (yerel, pervane göbeği orijinli; X ekseni geriye)."""
    pr = P.PROP
    c = math.cos(_rad(pr.downthrust_deg))
    xb = (pr.spinner_base_s - pr.hub[0]) / c
    xt = (pr.spinner_tip_s - pr.hub[0]) / c
    R = 0.5 * pr.spinner_d
    u = np.linspace(0, 1, 30)[1:-1]
    og = np.column_stack([xb + 0.0008 + (xt - xb - 0.0008) * u, R * (1 - u ** 1.9) ** 0.58])
    bp = SPINNER_BACKPLATE
    prof = np.vstack([[xb - bp, 0.0], [xb - bp, R + 0.0005], [xb - 0.0003, R + 0.0005], [xb + 0.0008, R],
                      og, [xt, 0.0]])
    return lathe(prof, 64, "U_Spinner", M["spinner"], "x")


# =====================================================================================================
# EO/IR çene tareti (yerel; pan/tilt/pencere top merkezli, yaka gövde altı merkezli)
# =====================================================================================================
TURRET_BORE_GAP = 0.0025                 # top ile yaka deliği arası. varsayım
TURRET_COLLAR_FLARE = 0.016              # yaka üstte gövdeye doğru genişler (fasetli soket görünümü). varsayım


def _octagon_factor(n_facets: int):
    def f(theta, k):
        a = np.mod(theta, 2 * math.pi / n_facets) - math.pi / n_facets
        return math.cos(math.pi / n_facets) / np.cos(a)
    return f


def turret_mount_top_z() -> float:
    """Yakanın üst kotu: yaka ayak izi boyunca gövde alt yüzeyinin en az 4 mm üstü (gövdeye gömülü)."""
    t = P.TURRET
    r = 0.5 * t.collar_d + TURRET_COLLAR_FLARE
    zs = []
    for a in np.linspace(0, 2 * math.pi, 24, endpoint=False):
        z = P.fuselage_z_at(t.s + r * math.cos(a), r * math.sin(a), "bottom")
        zs.append(z if z is not None else t.belly_z)
    return float(max(zs)) + 0.004


def _collar_bottom() -> tuple[float, float]:
    """Eğik yaka tabanı (yerel, gövde altına göre): ``(z_orta, eğim dz/dx)``. Arkada ``collar_depth`` + 4 mm,
    önde EO lens halkasının 2,5 mm üstü → tilt 0°'da pencereler yaka tarafından örtülmez (çene soketi)."""
    t = P.TURRET
    Rb = 0.5 * t.collar_d
    z_back = -t.collar_depth - 0.004
    z_front = (t.ball_center[2] + 0.5 * t.window_eo_d + 0.0019 + 0.0025) - t.belly_z
    return 0.5 * (z_back + z_front), (z_front - z_back) / (2 * Rb)


TURRET_COLLAR_FILLET = 0.010            # yaka ile karın arası iç bükey geçiş yarıçapı (R07; dar yerde kısalır). varsayım
TURRET_COLLAR_EMBED = 0.004             # filetonun karın derisinin içine uzaması (görünmez birleşim). varsayım


def _belly_local(t: "P.TurretSpec", c: float, s_: float, r: float) -> float:
    """Yaka yerel ekseninde (orijin: taret s'si, gövde altı) θ yönünde ``r`` yarıçapında karın derisi z'si."""
    z = P.fuselage_z_at(t.s - r * c, r * s_, "bottom")
    return float((z if z is not None else t.belly_z) - t.belly_z)


def turret_mount() -> MeshData:
    """``U_Turret_Mount``: 24 fasetli (spec ``collar_facets``) yaka, gövde alt rengi (``skin_bottom``). Tabanı önde
    yukarı eğik (ileri görüş açık), alt kenarında 2,8 mm pah. Üstte karına ``TURRET_COLLAR_FILLET`` yarıçaplı iç bükey
    fileto ile teğete yakın bağlanır (R07: eski 8 faset + sert basamak yerine); fileto her açıda karın derisinin
    gerçek yüksekliğine (yanlarda yükselen yuvarlak karın) göre kurulur, dar yerde (ön) kısalır. Yerel orijin
    (s, 0, gövde altı), Blender eksenleri (+X ileri)."""
    t = P.TURRET
    zc, k = _collar_bottom()
    z_top = turret_mount_top_z() - t.belly_z + 0.006
    Rb = 0.5 * t.collar_d
    rb = 0.5 * t.ball_d + TURRET_BORE_GAP
    ch = 0.0028
    F, E = TURRET_COLLAR_FILLET, TURRET_COLLAR_EMBED
    n_fac = int(t.collar_facets)
    n_th = 4 * n_fac
    fac = _octagon_factor(n_fac)
    th = np.linspace(0.0, 2 * math.pi, n_th, endpoint=False)
    psi = np.linspace(0.0, 0.5 * math.pi, 10)
    R_top = Rb + F + E
    rows: list = []                                    # profil noktası başına (n_th, 3) halka
    prof_th = []
    for a in th:
        c, s_ = math.cos(a), math.sin(a)
        f = float(fac(np.array([a]), 0)[0])
        z_lip = zc + k * Rb * c
        zb_f = _belly_local(t, c, s_, Rb + F)
        Fe = float(np.clip(zb_f - (z_lip + ch) - 0.0012, 0.002, F))       # dar yerde fileto kısalır
        zb_f = _belly_local(t, c, s_, Rb + Fe)
        z_w = zb_f - Fe
        pr = [(rb, zc + k * rb * c), ((Rb - ch) * f, zc + k * (Rb - ch) * c), (Rb * f, z_lip + ch)]
        for ps in psi:                                 # iç bükey fileto: duvar teğetinden karın teğetine
            pr.append(((Rb + Fe - Fe * math.cos(ps)) * f, z_w + Fe * math.sin(ps)))
        pr.append((Rb + Fe + E, _belly_local(t, c, s_, Rb + Fe + E) + 0.003))
        pr.append((R_top, z_top))
        pr.append((rb, z_top))
        prof_th.append((c, s_, pr))
    n_p = len(prof_th[0][2])
    mb = MeshBuilder("U_Turret_Mount", space="local")
    for kk in range(n_p):
        pts = np.array([(r * c, r * s_, z) for c, s_, pr in prof_th for (r, z) in [pr[kk]]])
        rows.append(mb.add(pts))
    for kk in range(n_p):
        mb.strip(rows[(kk + 1) % n_p], rows[kk], M["skin_bottom"])
    return mb.build(14.0)


def turret_ring() -> MeshData:
    """``U_Light_Turret_Ring``: yaka tabanında turkuaz durum LED halkası (yerel: yaka orijinine göre)."""
    t = P.TURRET
    zc, k = _collar_bottom()
    rb = 0.5 * t.ball_d + TURRET_BORE_GAP
    Rin = 0.5 * t.collar_d * math.cos(math.pi / t.collar_facets) - 0.0028
    Rc, rr = 0.5 * (rb + Rin), 0.0012
    th = np.linspace(0, 2 * math.pi, 12, endpoint=False)
    prof = np.column_stack([zc - 0.0002 + 0.6 * rr * np.sin(th), Rc + rr * np.cos(th)])
    md = lathe(prof, 64, "U_Light_Turret_Ring", M["status"], "z", closed_profile=True)
    md.verts[:, 2] += k * md.verts[:, 0]
    return md


def turret_pan() -> MeshData:
    """``U_Turret_Pan``: yaw gövdesi (Ø62), topun üstünde gövde içinde. Yerel orijin top merkezi; yerel Z = pan."""
    t = P.TURRET
    r = 0.5 * t.pan_housing_d
    zc = t.ball_center[2]
    z_top = turret_mount_top_z() - zc - 0.002
    z_bot = 0.5 * t.ball_d * 0.62
    prof = np.array([[z_top, 0.0], [z_top, r], [z_bot + 0.004, r], [z_bot, r - 0.004], [z_bot, 0.0]])[::-1]
    return lathe(prof, 48, "U_Turret_Pan", M["turret"], "z")


TURRET_FACE_X = 0.55                     # topun ön düz yüzü (yarıçap oranı). varsayım


def turret_ball() -> MeshData:
    """``U_Turret_Tilt``: Ø74 kamera topu, önde pencere düzlüğü, yanlarda tilt mili kapakları. Yerel orijin top
    merkezi; yerel Y = tilt (+ dönüş aşağı bakar; ``TURRET.tilt_up_axis_b``)."""
    t = P.TURRET
    R = 0.5 * t.ball_d
    af = math.acos(TURRET_FACE_X)
    al = np.linspace(math.pi, af, 40)
    sph = np.column_stack([R * np.cos(al), R * np.sin(al)])
    xf = R * TURRET_FACE_X
    prof = np.vstack([[[-R, 0.0]], sph[1:], [[xf + 0.0008, R * math.sin(af) - 0.0012]], [[xf + 0.0008, 0.0]]])
    ball = lathe(prof, 64, "ball", M["turret"], "x")
    caps = []
    for sg in (1.0, -1.0):
        pc = np.array([[R - 0.004, 0.0], [R - 0.004, 0.0115], [R + 0.0016, 0.0115], [R + 0.0024, 0.010], [R + 0.0024, 0.0]])
        cap = lathe(pc, 40, "cap", M["turret"], "y")
        if sg < 0:
            cap.verts = cap.verts * np.array([1.0, -1.0, 1.0])
            cap.faces = [tuple(reversed(f)) for f in cap.faces]
        caps.append(cap)
    return merge("U_Turret_Tilt", [ball] + caps, 30.0)


def turret_windows() -> MeshData:
    """``U_Turret_Window``: EO (Ø26, iskele) ve IR (Ø20, sancak) pencereleri, siyah eloksal çerçeveler (``bezel``)
    (yerel: top merkezi, pencere düzlemi +X'te)."""
    t = P.TURRET
    R = 0.5 * t.ball_d
    xf = R * TURRET_FACE_X + 0.0008
    parts = []
    for yc, d in ((0.5 * t.window_spacing, t.window_eo_d), (-0.5 * t.window_spacing, t.window_ir_d)):
        r = 0.5 * d
        ring = lathe(np.array([[xf - 0.0004, r], [xf - 0.0004, r + 0.0019], [xf + 0.0010, r + 0.0019],
                               [xf + 0.0014, r + 0.0010], [xf + 0.0014, r]]), 56, "ring", M["bezel"], "x",
                     closed_profile=True)
        glass = lathe(np.array([[xf - 0.0012, 0.0], [xf - 0.0012, r], [xf + 0.0005, r], [xf + 0.0009, 0.7 * r],
                                [xf + 0.0011, 0.0]]), 56, "glass", M["glass"], "x")
        for p in (ring, glass):
            p.verts = p.verts + np.array([0.0, yc, 0.0])
            parts.append(p)
    return merge("U_Turret_Window", parts, 30.0)


# =====================================================================================================
# İniş takımı: kapak açıklıkları, boolean kesiciler, kuyu astarları, kapaklar
# =====================================================================================================
def _saw(s0: float, ya: float, yb: float, inward: float) -> list[tuple[float, float]]:
    p, d = P.SAWTOOTH["pitch_m"], P.SAWTOOTH["depth_m"]
    n = max(1, int(round(abs(yb - ya) / p)))
    ys = np.linspace(ya, yb, 2 * n + 1)
    return [(s0 + (inward * d if k % 2 else 0.0), float(v)) for k, v in enumerate(ys)]


def gear_opening(leg: str) -> np.ndarray:
    """Takım kapaklarının toplam açıklığı (plan çokgeni ``(s, y)``; kapak dişleriyle aynı, boolean kesicisi
    bunu kullanır). ``leg``: ``"N"``, ``"L"``, ``"R"``."""
    leg = leg.upper()
    if leg == "N":
        nw = _G["nose"]["well"]
        s0, s1 = nw["s_m"]
        hw = float(nw["half_width_m"])
        nh = float(_G["nose"]["doors"]["clamshell"]["strut_notch"]["half_width_m"])
        pts = _saw(s0, -hw, -nh, +1) + _saw(s0, hw, nh, +1)[::-1] + _saw(s1, 0.0, hw, -1)[::-1] + _saw(s1, 0.0, -hw, -1)[1:]
        return np.asarray(pts)
    d1 = next(d for d in P.gear_doors() if d.name == f"U_Door_{leg}_1")
    o = list(d1.outline)
    n_front = next(i for i in range(1, len(o)) if abs(o[i][0] - o[0][0]) > 0.02)    # ön diş dizisi sonu
    slot = P._leg_slot(leg)                    # açıklık pivotun 7,5 mm dışına; bacak kapağı pivotun 7,5 mm içinde biter
    return np.asarray(o[:n_front] + list(slot) + o[n_front:])


def _well(leg: str) -> P.GearWell:
    return next(w for w in P.gear_wells() if w.name == leg.upper())


def _skin_z_for_leg(leg: str) -> Callable[[float, float], float]:
    if leg.upper() == "N":
        def zf(s, y):
            z = P.fuselage_z_at(s, y, "bottom")
            return float(z if z is not None else P.fuselage_section(s).z_bottom)
        return zf
    return belly_z


WELL_ROOF_SKIN = 0.0015                  # kuyu tavanı ile kanat üst derisi arasında en az kalan et (0,6 mm deri + 0,9). varsayım


def well_roof_z(leg: str) -> Callable[[float, float], float]:
    """Kuyu tavanı ``z(s, y)``: spec ``roof_z``; ana kuyuda gövde dışında kanat üst yüzeyinin en az
    ``WELL_ROOF_SKIN`` altında kalacak şekilde sınırlanır (ince firar kenarı bölgesinde kanat delinmez)."""
    w = _well(leg)
    if leg.upper() == "N":
        pk = P.nose_plug_pocket()
        if pk is None:
            return lambda s, y: w.roof_z
        return lambda s, y: (w.roof_z if s >= pk["s1"] - 1e-9 else pk["z_top"])
    side = "L" if leg.upper() == "L" else "R"
    tab = wing_table(side)

    def f(s, y):
        if abs(y) < P.fuselage_half_width_at(s, w.roof_z) - 0.004:
            return w.roof_z
        zu = tab.z(s, abs(y), "upper")
        return w.roof_z if not np.isfinite(zu) else min(w.roof_z, zu - WELL_ROOF_SKIN)
    return f


WELL_LIP = DOOR_T + 0.0018               # dişli deri dudağı: kapak eti + 1,8 mm (2,5 mm kapak dönüş dudağına 0,5 mm pay). varsayım


def gear_well_outline(leg: str) -> np.ndarray:
    """Kuyu duvarlarının plan çokgeni ``(s, y)``: kapak açıklığının dişsiz dış zarfı (params ``GearWell.outline``;
    ana kuyuda tekerlek kuyusu dikdörtgeni + bacak yuvası). Testere dişleri yalnız deri dudağındadır."""
    return np.asarray(_well(leg).outline, float)


def _lip_z(leg: str) -> Callable[[float, float], float]:
    zf = _skin_z_for_leg(leg)
    return lambda s, y: zf(s, y) + WELL_LIP


def _well_parts(leg: str) -> list[tuple[np.ndarray, Callable[[float, float], float]]]:
    """Kuyu hacmi parçaları ``[(plan çokgeni, tavan z(s, y))]``: ana kuyu; burunda ayrıca önündeki tıkaç cebi
    (ayrı alçak tavanlı dikdörtgen, ana kuyuya 2 mm bindirir — tavan basamağı dik kalır)."""
    roof = well_roof_z(leg)
    if leg.upper() != "N" or P.nose_plug_pocket() is None:
        return [(gear_well_outline(leg), roof)]
    pk = P.nose_plug_pocket()
    nw = _G["nose"]["well"]
    s0, s1 = (float(v) for v in nw["s_m"])
    hw = float(nw["half_width_m"])
    main = np.array([(s0, -hw), (s1, -hw), (s1, hw), (s0, hw)])
    pocket = np.array([(pk["s0"], -pk["hw"]), (s0 + 0.002, -pk["hw"]), (s0 + 0.002, pk["hw"]), (pk["s0"], pk["hw"])])
    w = _well(leg)
    return [(main, lambda s, y: w.roof_z), (pocket, lambda s, y: pk["z_top"])]


TYRE_PROFILE = {"bead": 0.70, "rm": 0.32, "p_out": 1.85, "groove_a": 0.30, "groove_w": 0.0012, "groove_d": 0.0009}
                                         # lastik kesiti: topuk eni oranı, en geniş yerin kesit yüksekliğindeki yeri, sırt
                                         # süperelips üssü, oluk konumu (yarı ene oran)/eni/derinliği. varsayım
TYRE_CLEAR = 0.0025                      # toplu teker ↔ kuyu tavanı / kapak iç yüzü en az payı (AERO-06). varsayım


def tyre_half_width_factor(r, R: float, r_rim: float) -> np.ndarray:
    """Lastik kesitinin ``r`` yarıçapındaki yarı eninin en büyük yarı ene oranı (``gear.tyre_profile`` ile aynı eğri)."""
    r = np.asarray(r, float)
    Ab = TYRE_PROFILE["bead"]
    r_m = r_rim + TYRE_PROFILE["rm"] * (R - r_rim)
    B, p = R - r_m, TYRE_PROFILE["p_out"]
    lo = Ab + (1 - Ab) * np.sqrt(np.clip(1 - ((r_m - r) / (r_m - r_rim)) ** 2, 0, 1))
    hi = np.clip(1 - np.clip((r - r_m) / B, 0, 1) ** p, 0, 1) ** (1 / p)
    return np.where(r < r_m, lo, hi)


@functools.lru_cache(maxsize=None)
def tyre_fit_width(name: str) -> float:
    """Ana tekerin toplu konumda kuyuya sığan en büyük lastik eni (m, 0,5 mm'ye aşağı yuvarlı, en çok spec eni): kuyu
    tavanı (``well_roof_z`` − astar) tekerin orta düzleminden ``r`` yarıçapında her yönde ``hw(r) + TYRE_CLEAR``'dan
    yüksek olmalı. Tavan arka-iç köşede kanat üst derisiyle sınırlıdır (R14: toplu teker ``axle_offset_out_m`` kadar
    alçak → spec 26 mm sığar)."""
    g = P.gear_leg(name)
    c = np.asarray(g.axle_retracted, float)
    roof = well_roof_z(name)
    liner = BAY_INSET + BAY_WALL
    best = g.wheel_w
    th = np.linspace(0.0, 2 * math.pi, 96, endpoint=False)
    for r in np.linspace(0.5 * g.hub_d, g.wheel_r, 24):
        avail = min(roof(c[0] + r * math.cos(t), c[1] + r * math.sin(t)) - liner - c[2] for t in th)
        f = float(tyre_half_width_factor(r, g.wheel_r, 0.5 * g.hub_d))
        if f > 1e-6:
            best = min(best, 2.0 * (avail - TYRE_CLEAR) / f)
    return float(math.floor(best * 2000.0) / 2000.0)


def stowed_tyre_door_gap(name: str, width: float | None = None) -> float:
    """Toplu ana tekerin alt yüzü ile kapalı kuyu kapağının iç tavası arası en küçük pay (m): kapak dış yüzü (kaporta
    tabanı) + deri + tava; lastik alt yüzü ``hw(r)`` ile (R14 kontrolü)."""
    g = P.gear_leg(name)
    c = np.asarray(g.axle_retracted, float)
    W = g.wheel_w if width is None else width
    pan = DOOR_STRUCT["main"][3]
    best = 1.0
    th = np.linspace(0.0, 2 * math.pi, 72, endpoint=False)
    for r in np.linspace(0.5 * g.hub_d, g.wheel_r, 16):
        f = float(tyre_half_width_factor(r, g.wheel_r, 0.5 * g.hub_d))
        for t in th:
            s, y = c[0] + r * math.cos(t), c[1] + r * math.sin(t)
            z_door = belly_z(s, y) + DOOR_SKIN + pan
            best = min(best, (c[2] - 0.5 * W * f) - z_door)
    return best


def gear_cutters(leg: str) -> list[MeshData]:
    """Kuyu için boolean (EXACT) kesicileri, bu sırayla uygulanır: (1) dişli açıklık — kapak dişleriyle aynı
    çokgen, alttan deri yüzeyinin ``WELL_LIP`` (2,7 mm) üstüne kadar: testere dişi yalnız deri/kapak kalınlığında
    kalır (gerçek uçaktaki gibi); kesilen dudak yüzleri koyu ``seal`` (kapalı kapakta koyu ayrım çizgisi);
    (2) kuyu hacmi — dişsiz zarf (``gear_well_outline``), dudağın üstünden tavana (``well_roof_z``) düz duvarlar
    (``bay`` astar rengi); burunda ayrıca tıkaç cebi. Toplu teker diş uçlarına değmez."""
    lip = _lip_z(leg)
    L = leg.upper()
    out = [heightfield_panel(f"U_Cutter_{L}_Lip", gear_opening(leg), lambda s, y: -0.30, 0.0, M["seal"], M["seal"],
                             n_rows=6, col_step=0.0015, z_in=lambda s, y: lip(s, y) + 0.0006, smooth_angle=None)]
    for k, (poly, roof) in enumerate(_well_parts(leg)):           # 0,8 mm bindirme: iki kesici arasında ince film kalmaz
        out.append(heightfield_panel(f"U_Cutter_{L}_Well{k}", poly, lambda s, y: lip(s, y) - 0.0008, 0.0, M["bay"],
                                     M["bay"], n_rows=12, col_step=0.0015, z_in=roof, smooth_angle=None))
    return out


def gear_cutter(leg: str) -> MeshData:
    """Geriye uyumluluk: kuyu hacmi kesicisi (``gear_cutters(leg)[1]``). Sahne kurucusu ``gear_cutters`` kullanır."""
    return gear_cutters(leg)[1]


BAY_INSET, BAY_WALL = 0.0004, 0.0008


def gear_bay(leg: str) -> MeshData:
    """``U_Bay_<leg>``: kuyu astarı (açık gri ``bay``) — dişsiz kuyu zarfını (``gear_well_outline``) 0,4 mm içeriden
    izleyen 0,8 mm duvar halkası (alt kenarı deri dudağının üstüne oturur) ve tavana oturan plaka(lar) (kapalı kabuklar
    tek nesnede; burunda tıkaç cebinin ayrı alçak tavanı)."""
    w = _well(leg)
    lip = _lip_z(leg)
    roof = well_roof_z(leg)
    poly = gear_well_outline(leg)
    po = poly_resample(poly_offset(poly, BAY_INSET), 0.004)
    pi = poly_offset(po, BAY_WALL)
    zb = lambda p: np.array([lip(s, y) - 0.0002 for s, y in p])
    zt = lambda p, d: np.array([roof(s, y) - d for s, y in p])
    mb = MeshBuilder("walls")
    o_b = mb.add(np.column_stack([po, zb(po)]))
    o_t = mb.add(np.column_stack([po, zt(po, BAY_INSET)]))
    i_t = mb.add(np.column_stack([pi, zt(pi, BAY_INSET)]))
    i_b = mb.add(np.column_stack([pi, zb(pi)]))
    mb.loft([o_b, o_t, i_t, i_b, o_b], M["bay"])
    parts = [mb.build(40.0)]
    for k, (pp, rf) in enumerate(_well_parts(leg)):
        parts.append(heightfield_panel(f"plate{k}", poly_offset(pp, BAY_INSET),
                                       lambda s, y, rf=rf: rf(s, y) - BAY_INSET - BAY_WALL, BAY_WALL, M["bay"], M["bay"],
                                       n_rows=12, col_step=0.004))
    return merge(w.obj_name, parts, 40.0)


@dataclass
class DoorGeom:
    """Kapak ağı + menteşe tanımı. ``origin`` spec ``(s, y, z)``; ``frame_b`` 3×3 (sütunlar X, Y, Z; Blender):
    X = menteşe (+ dönüş açar) ya da ``strut`` kapakta takım toplama ekseni."""

    mesh: MeshData
    origin: tuple[float, float, float]
    frame_b: np.ndarray
    attach: str
    open_deg: float


DOOR_SKIN = 0.0008                      # kapak dış deri katmanı (testere dişli kenarlar yalnız bu katmanda). varsayım
DOOR_LIP = {"w": 0.0012, "t": 0.0025}   # dış deri çevresinde içe dönük dönüş dudağı: kenar 2,5 mm kalın okunur (R06). varsayım
# İç yapı (kapak iç yüzü): düz kenarlı iç tava + çevre çerçeve kaburgası + boyuna boncuk. (inset, kaburga eni,
# kaburga yüksekliği, tava yüksekliği, boncuk eni, boncuk yüksekliği) — yükseklikler deri üstünden. varsayım
DOOR_STRUCT = {"main": (0.006, 0.004, 0.0025, 0.0010, 0.004, 0.0022),
               "nose": (0.004, 0.003, 0.0020, 0.0006, 0.0, 0.0),
               "leg": (0.0015, 0.002, 0.0012, 0.0, 0.0, 0.0),
               "plug": (0.0015, 0.0025, 0.0010, 0.0, 0.0, 0.0)}
NOSE_DOOR_CLEAR_Y = 0.010               # burun kapağı iç yapısı serbest kenardan bu kadar dışarıda başlar (toplu tork bağlantıları)


def _door_grid_panel(name: str, poly: np.ndarray, z_out: Callable, t_fn: Callable, s_fixed: Sequence[float],
                     y_fixed: Sequence[float], mat_out: str, mat_in: str, mat_side: str, inset: float,
                     col_step: float = 0.002, row_step: float = 0.005, lip: dict | None = None,
                     lip_y: tuple[bool, bool] = (True, True), lip_hinge_clear: float = 0.006) -> MeshData:
    """Değişken kalınlıklı kapak katısı (tek kapalı gövde): plan çokgeni (her y sütununda tek s aralığı), dış yüz
    ``z_out(s, y)``, iç yüz ``z_out + t_fn(s, y)`` (yukarı = içe). ``s_fixed``/``y_fixed``: kalınlık basamağı
    sınırları — ızgara bu çizgilerin ±0,1 mm'sine sıkıştırılır, basamaklar keskin (dik) kalır. Satırlar: kenar
    a(y) → ilk sabit çizgiye eşit aralık → sabit çizgiler → son sabit çizgiden b(y)'ye eşit aralık (her sütunda aynı
    sayı). Dış yüz ``mat_out``, iç yüz ``mat_in``, çevre duvarı ``mat_side`` (koyu ayrım çizgisi).

    ``lip`` = {"w": en, "t": toplam kalınlık}: dış deri çevresinde (dişli s kenarları dişleri izleyerek, ``lip_y``
    ile seçilen y kenarları — menteşe kenarı hariç) içe dönük dönüş dudağı (R06): kenar kalınlığı ``t`` okunur,
    dudak iç kenarı 0,1 mm'lik dik basamakla deriye iner. Dişli kenarlardaki dudak menteşe kenarından
    ``lip_hinge_clear`` önce biter: menteşe çevresinde dönerken dudağın yanal kayması (h·sin θ) 0,5 mm kapak aralığında
    kalır, deri dişlerine sürtmez."""
    p = np.asarray(poly, float)
    ys_v = np.unique(np.round(p[:, 1], 9))
    y0, y1 = ys_v.min() + inset, ys_v.max() - inset
    lw = float(lip["w"]) if lip else 0.0
    dl = 1e-4
    cols = set(np.linspace(y0, y1, max(2, int(math.ceil((y1 - y0) / col_step)) + 1)).tolist())
    for v in list(ys_v) + list(y_fixed):
        for dv in (-1e-4, 1e-4):
            if y0 + 2e-5 < v + dv < y1 - 2e-5:
                cols.add(float(v + dv))
    if lip:
        if lip_y[0]:
            cols.update((y0 + lw, y0 + lw + dl))
        if lip_y[1]:
            cols.update((y1 - lw, y1 - lw - dl))
    cols = np.array(sorted(cols))
    ivs = []
    for yv in cols:
        iv = poly_s_interval(p, float(yv))
        a, b = (iv if iv else (0.0, 0.0))
        ivs.append((a + inset, b - inset))
    ivs = np.asarray(ivs)
    amax, bmin = float(ivs[:, 0].max()), float(ivs[:, 1].min())
    pad = lw + 2 * dl if lip else 0.0
    fx = []
    for v in sorted(s_fixed):
        for dv in (-1e-4, 1e-4):
            if amax + pad + 4e-4 < v + dv < bmin - pad - 4e-4:
                fx.append(v + dv)
    fx = sorted(set(np.round(fx, 9)))
    if not fx:
        fx = [0.5 * (amax + bmin)]
    dense = [fx[0]]                                     # sabit çizgiler arasını row_step'e böl
    for v in fx[1:]:
        n = max(1, int(math.ceil((v - dense[-1]) / row_step)))
        dense += list(np.linspace(dense[-1], v, n + 1)[1:])
    fx = np.asarray(dense)
    n_head = max(1, int(math.ceil((fx[0] - float(ivs[:, 0].min()) - pad) / row_step)))
    n_tail = max(1, int(math.ceil((float(ivs[:, 1].max()) - pad - fx[-1]) / row_step)))
    G = []
    for (a, b), yv in zip(ivs, cols):
        if lip:                                         # dudak: kenar → +en → +en+0,1 mm (dik iç basamak)
            head = [a, a + lw] + list(np.linspace(a + lw + dl, fx[0], n_head + 1)[:-1])
            tail = list(np.linspace(fx[-1], b - lw - dl, n_tail + 1)[1:]) + [b - lw, b]
        else:
            head = list(np.linspace(a, fx[0], n_head + 1)[:-1])
            tail = list(np.linspace(fx[-1], b, n_tail + 1)[1:])
        rows = np.r_[head, fx, tail]
        G.append(np.column_stack([rows, np.full(len(rows), yv)]))
    G = np.asarray(G)
    zo = np.array([[z_out(s, y) for s, y in row] for row in G])
    T = np.array([[t_fn(s, y) for s, y in row] for row in G])
    if lip:
        nr = G.shape[1]
        yy = G[:, :, 1]
        on = np.zeros(G.shape[:2], bool)
        far = np.ones(G.shape[:2], bool)                # dişli kenar dudağı menteşe kenarından uzakta
        if not lip_y[0]:
            far &= yy >= y0 + lip_hinge_clear
        if not lip_y[1]:
            far &= yy <= y1 - lip_hinge_clear
        on[:, :2] = far[:, :2]
        on[:, nr - 2:] = far[:, nr - 2:]
        if lip_y[0]:
            on |= yy <= y0 + lw + 1e-9
        if lip_y[1]:
            on |= yy >= y1 - lw - 1e-9
        T = np.where(on, np.maximum(T, float(lip["t"])), T)
    zi = zo + T
    mb = MeshBuilder(name)
    io = mb.add(np.dstack([G, zo]))
    ii = mb.add(np.dstack([G, zi]))
    nc, nr = io.shape
    for i in range(nc - 1):
        for j in range(nr - 1):
            mb.face((io[i, j], io[i + 1, j], io[i + 1, j + 1], io[i, j + 1]), mat_out)
            mb.face((ii[i, j], ii[i, j + 1], ii[i + 1, j + 1], ii[i + 1, j]), mat_in)
    ring_o = list(io[0, :]) + list(io[1:, -1]) + list(io[-1, -2::-1]) + list(io[-2:0:-1, 0])
    ring_i = list(ii[0, :]) + list(ii[1:, -1]) + list(ii[-1, -2::-1]) + list(ii[-2:0:-1, 0])
    mb.strip(ring_o, ring_i, mat_side)
    return mb.build(40.0)


def _door_struct(d: P.GearDoor) -> tuple[str, Callable, list, list]:
    """Kapak iç yapısı: (tür, kalınlık fonksiyonu t(s, y), s basamakları, y basamakları)."""
    pitch_d = P.SAWTOOTH["depth_m"]
    o = np.asarray(d.outline, float)
    if d.attach == "skin" and d.leg in ("L", "R"):
        kind = "main"
    elif d.attach == "skin":
        kind = "nose"
    elif d.leg == "N":
        kind = "plug"
    else:
        kind = "leg"
    ins, fw, fh, ph, bw, bh = DOOR_STRUCT[kind]
    ay = np.abs(o[:, 1])
    if kind == "main":
        a0, a1 = float(o[:, 0].min()), float(o[:, 0].max())
        s_lo, s_hi = a0 + pitch_d + ins, a1 - pitch_d - ins
        b0, b1 = float(ay.min()), float(ay.max())
        y_lo, y_hi = b0 + ins, b1 - ins
        y_bead = y_lo + 0.30 * (y_hi - y_lo)                   # boncuk menteşe tarafında: toplu aks kapağının dışında
    elif kind == "nose":
        nd = _G["nose"]["doors"]["clamshell"]["strut_notch"]
        s_lo = float(nd["s_m"][1]) + ins
        s_hi = float(o[:, 0].max()) - pitch_d - ins
        y_lo, y_hi = NOSE_DOOR_CLEAR_Y, float(ay.max()) - ins      # orta şerit (tork bağlantıları) yalnız deri
    else:
        s_lo, s_hi = float(o[:, 0].min()) + ins, float(o[:, 0].max()) - ins
        y_lo, y_hi = float(ay.min()) + ins, float(ay.max()) - ins
    sg = 1.0 if float(np.mean(o[:, 1])) >= 0 else -1.0
    ym = y_bead if kind == "main" else 0.5 * (y_lo + y_hi)

    def t_fn(s, y):
        u = abs(y)
        if not (s_lo <= s <= s_hi and y_lo <= u <= y_hi):
            return DOOR_SKIN
        if s < s_lo + fw or s > s_hi - fw or u < y_lo + fw or u > y_hi - fw:
            return DOOR_SKIN + fh
        if bw > 0 and abs(u - ym) <= 0.5 * bw:
            return DOOR_SKIN + bh
        return DOOR_SKIN + ph

    s_fix = [s_lo, s_lo + fw, s_hi - fw, s_hi]
    y_fix = [sg * v for v in (y_lo, y_lo + fw, y_hi - fw, y_hi)]
    if bw > 0:
        y_fix += [sg * (ym - 0.5 * bw), sg * (ym + 0.5 * bw)]
    return kind, t_fn, s_fix, y_fix


def gear_door(name: str) -> DoorGeom:
    """``U_Door_N_1/2`` (burun, iki yandan), ``U_Door_L/R_1`` (tekerlek kuyusu, içteki kenardan menteşeli),
    ``U_Door_L/R_2`` (bacak yuvası kapağı, bacağa bağlı; toplu konumda modellenir, orijin takım pivotunda),
    ``U_Door_N_3`` (bacak çentiği tıkacı, burun trunnion bloğuna bağlı).

    İki katman tek kapalı gövdede: 0,8 mm dış deri (testere dişli kenar yalnız bu katmanda; dış yüz alt boya) ve iç
    yapı — dişlerin kökünden 6 mm içeride düz kenarlı tava, 4 mm genişliğinde çevre çerçeve kaburgası ve boyuna boncuk
    (``DOOR_STRUCT``; iç yüz astar rengi). Çevre duvarları koyu (``seal``) → kapalı kapakta koyu ayrım çizgisi.
    Kapak çevresinde 0,5 mm aralık. Menteşe donanımı ayrı nesnededir (``door_hardware``)."""
    d = next(x for x in P.gear_doors() if x.name == name)
    zf = _skin_z_for_leg(d.leg)
    poly = np.asarray(d.outline)
    kind, t_fn, s_fix, y_fix = _door_struct(d)
    if kind == "plug":                                   # ön kenar: pivot etrafındaki yay deri dudağına girmesin
        poly = poly.copy()
        poly[:, 0] = np.maximum(poly[:, 0], _plug_front_s())
    if kind in ("main", "nose"):
        ys = np.asarray(poly, float)[:, 1]
        hy = float(d.hinge_p0[1])                        # menteşe kenarında dudak yok (bilekler orada)
        hinge_low = abs(hy - ys.min()) < abs(hy - ys.max())
        md = _door_grid_panel(name, poly, zf, t_fn, s_fix, y_fix, M["skin_bottom"], M["door_inner"], M["seal"],
                              inset=DOOR_GAP, col_step=0.002, lip=DOOR_LIP, lip_y=(not hinge_low, hinge_low))
    else:                                                # dar/eğik bacak kapakları ve tıkaç: sabit kalınlık (deri + iç levha)
        md = heightfield_panel(name, poly, zf, DOOR_T, M["skin_bottom"], M["door_inner"], inset=LEG_DOOR_GAP, n_rows=16,
                               col_step=0.0015)
    if kind == "plug":
        md = merge(name, [md, _plug_web(d)], 40.0)
    if d.attach == "skin":
        o = tuple(0.5 * (a + b) for a, b in zip(d.hinge_p0, d.hinge_p1))
        fr = _frame_from_axes(d.axis_open_b, [0.0, 0.0, 1.0])
    else:
        g = P.gear_leg(d.leg)
        o = g.pivot
        fr = _frame_from_axes(g.retract_axis_b, [0.0, 0.0, 1.0])
    return DoorGeom(md, o, fr, d.attach, d.open_deg)


LEG_DOOR_GAP = 0.0009                   # bacağa bağlı kapakların çevre aralığı (pivot ekseni etrafında dönüşte deri payı)


def _plug_front_s() -> float:
    """Çentik tıkacının ön kenarı: pivot etrafında açılırken alt-ön köşesinin yayı deri dudağının (``WELL_LIP``) önüne
    geçmeyecek en geri ``s`` (kuyu ön kenarından ≈ 5 mm geride)."""
    g = P.gear_leg("N")
    s0 = float(_G["nose"]["well"]["s_m"][0])
    zs = _skin_z_for_leg("N")(s0, 0.0)
    ps, pz = g.pivot[0], g.pivot[2]
    dl = pz - (zs + WELL_LIP)
    lo, hi = s0, ps
    for _ in range(50):
        sf = 0.5 * (lo + hi)
        R2 = (ps - sf) ** 2 + (pz - zs) ** 2
        s_l = ps - math.sqrt(max(R2 - dl * dl, 0.0))
        lo, hi = (lo, sf) if s_l >= s0 + 0.0004 else (sf, hi)
    return hi


def _plug_web(d: P.GearDoor) -> MeshData:
    """Burun çentik tıkacının taşıyıcı ağı: tıkaçtan (toplu konumda deride) yukarı, trunnion bloğuna (pivotun altı)."""
    g = P.gear_leg("N")
    s_c = g.pivot[0]
    zf = _skin_z_for_leg("N")
    z_bot = zf(s_c, 0.0) + DOOR_SKIN + 0.0004
    z_top = g.pivot[2] - 0.0068
    poly = np.array([(s_c - 0.006, -0.0025), (s_c + 0.006, -0.0025), (s_c + 0.006, 0.0025), (s_c - 0.006, 0.0025)])
    return prism("web", poly, z_bot - 0.0012, z_top, M["door_inner"])


def door_hardware(name: str) -> MeshData | None:
    """``U_DoorHw_<kapak>``: menteşe donanımı (kapakla döner; kapağın orijin/çerçevesini paylaşır). Deri kapaklarında
    menteşe ekseni üzerinde Ø3,2 mm menteşe bilekleri (ana kapakta 2 adet, menteşe boyunun %25/%75'inde; burun
    kapaklarında 3 adet) ve her bilekten kapak iç yüzüne uzanan 10 mm kulak (``gear`` anodize). Ana kapakta serbest
    kenar ortasında itme çubuğu kulağı (horn). Bacağa bağlı kapaklar için None."""
    d = next(x for x in P.gear_doors() if x.name == name)
    if d.attach != "skin":
        return None
    zf = _skin_z_for_leg(d.leg)
    h0, h1 = np.asarray(d.hinge_p0, float), np.asarray(d.hinge_p1, float)
    L = float(np.linalg.norm(h1 - h0))
    ax = (h1 - h0) / L
    o = np.asarray(d.outline, float)
    cen = np.r_[o.mean(0), 0.0]
    inward = np.array([0.0, np.sign(cen[1] - h0[1]), 0.0])           # menteşeden serbest kenara (y)
    fracs = (0.25, 0.75) if d.leg in ("L", "R") else (0.17, 0.5, 0.83)
    parts = []
    r_b, l_b = 0.0016, 0.012
    u_b = DOOR_GAP + 0.0034                            # bilek ekseni kapak kenarından içeride: kuyu astarına 1,2 mm pay
    for f in fracs:
        c = h0 + ax * f * L
        z_skin = zf(c[0], c[1] + inward[1] * 0.006)
        cb = np.array([c[0], c[1] + inward[1] * u_b, z_skin + DOOR_SKIN + r_b + 0.0002])
        b = lathe(np.array([[-0.5 * l_b, 0.0], [-0.5 * l_b, r_b], [0.5 * l_b, r_b], [0.5 * l_b, 0.0]]), 16, "knuckle",
                  M["gear"], "x", space="spec")
        b.verts = cb + b.verts @ np.column_stack([ax, np.cross([0, 0, 1.0], ax), [0, 0, 1.0]]).T
        parts.append(b.oriented())
        leaf = np.array([(c[0] - 0.5 * l_b + 0.001, 0.0), (c[0] + 0.5 * l_b - 0.001, 0.0),
                         (c[0] + 0.5 * l_b - 0.001, 0.010), (c[0] - 0.5 * l_b + 0.001, 0.010)])
        leaf[:, 1] = c[1] + inward[1] * (u_b + leaf[:, 1])
        zl = z_skin + DOOR_SKIN + 0.0001
        parts.append(prism("leaf", leaf, zl - 0.0003, zl + 0.0012, M["gear"]))
    if d.leg in ("L", "R"):                                            # itme çubuğu kulağı: serbest kenarın ön köşesi
        sm = float(o[:, 0].min()) + P.SAWTOOTH["depth_m"] + 0.012     # (toplu tekerin izdüşümü dışında)
        yf = float(o[np.argmax(np.abs(o[:, 1])), 1]) - inward[1] * 0.009
        z_s = zf(sm, yf) + DOOR_SKIN + 0.0025
        horn = np.array([(sm - 0.004, yf - 0.0015), (sm + 0.004, yf - 0.0015), (sm + 0.004, yf + 0.0015),
                         (sm - 0.004, yf + 0.0015)])
        parts.append(prism("horn", horn, z_s - 0.0004, z_s + 0.005, M["gear"]))
        parts.append(lathe(np.array([[-0.0025, 0.0], [-0.0025, 0.0012], [0.0025, 0.0012], [0.0025, 0.0]]), 12, "ball",
                           M["steel"], "y", space="spec"))
        parts[-1].verts = parts[-1].verts + np.array([sm, yf, z_s + 0.0038])
        parts[-1] = parts[-1].oriented()
    return merge(f"U_DoorHw_{name[7:]}", parts, 35.0)


def turret_cutter() -> MeshData:
    """Taret yaka deliği için gövde kesicisi (top ve yaw gövdesinin gövde içine girdiği silindir)."""
    t = P.TURRET
    r = 0.5 * t.ball_d + TURRET_BORE_GAP
    th = np.linspace(0, 2 * math.pi, 48, endpoint=False)
    poly = np.column_stack([t.s + r * np.cos(th), r * np.sin(th)])
    return prism("U_Cutter_Turret", poly, t.belly_z - 0.05, turret_mount_top_z() + 0.012, M["pacf"])


# =====================================================================================================
# Ayrıntılar: antenler, pitot, ışıklar
# =====================================================================================================
def blade_antenna(f: P.Feature, n: int = 21) -> MeshData:
    """``U_Antenna_Blade_*``: 36° oklu bıçak anten (NACA 0010 kesit), tabanı deriye 3 mm gömülü, ucu kapalı."""
    pr = f.params
    H, cr, ct = float(pr["h_m"]), float(pr["chord_root_m"]), float(pr["chord_tip_m"])
    sw = math.tan(_rad(float(pr["sweep_deg"])))
    dz = float(np.sign(f.direction[2]))
    s_le0 = f.pos[0] - 0.40 * cr
    hs = np.r_[-0.003, np.linspace(0.0, H - 0.003, 12), H - 0.003 + 0.003 * np.sin(np.linspace(0, 0.5 * math.pi, 5))[1:]]
    mb = MeshBuilder(f.obj_name)
    rings = []
    for k, h in enumerate(hs):
        hc = float(np.clip(h, 0, H))
        c = cr + (ct - cr) * hc / H
        af = P.tail_airfoil(n, c, 0.0006) * c
        if h > H - 0.003:
            q = min(1.0, (h - (H - 0.003)) / 0.003)
            af[:, 1] *= math.sqrt(max(0.0, 1 - q * q))
        pts = np.column_stack([s_le0 + hc * sw + af[:, 0], f.pos[1] + af[:, 1], np.full(len(af), f.pos[2] + dz * h)])
        if k == len(hs) - 1:
            half = mb.add(pts[:n])
            rings.append(np.r_[half, half[:-1][::-1]])
        else:
            rings.append(mb.add(pts))
    mb.loft(rings, M["antenna"])
    mb.cap(rings[0], M["antenna"], start=True)
    return mb.build(30.0)


def gnss_puck(f: P.Feature) -> MeshData:
    """``U_Antenna_GNSS_*``: Ø28 × 8 mm GNSS anteni (kapak altındaki koyu cepte, füme kapaktan görünür)."""
    r, h = 0.5 * float(f.params["d_m"]), float(f.params["h_m"])
    prof = np.array([[0.0, 0.0], [0.0, r], [h - 0.0025, r], [h - 0.0006, r - 0.0012], [h, r - 0.004], [h, 0.0]])
    md = lathe(prof, 40, f.obj_name, M["antenna"], "z", space="spec")
    md.verts = md.verts + np.asarray(f.pos)
    return md.oriented()


def dipole(f: P.Feature) -> MeshData:
    """``U_Antenna_Telemetry``: kapak altındaki telemetri dipolü. Kapak yüksekliğine sığmadığı için cep tabanında
    ``s`` boyunca yatık (params yönü +Z; sapma raporda)."""
    h = float(f.params["h_m"])
    prof = np.array([[-0.5 * h, 0.0], [-0.5 * h, 0.0022], [-0.12 * h, 0.0022], [-0.12 * h, 0.0035], [0.12 * h, 0.0035],
                     [0.12 * h, 0.0022], [0.5 * h - 0.001, 0.0022], [0.5 * h, 0.0012], [0.5 * h, 0.0]])
    md = lathe(prof, 20, f.obj_name, M["antenna"], "x", space="spec")
    md.verts = md.verts + np.array([f.pos[0], f.pos[1], hatch_pocket_floor_z() + 0.0036])
    return md.oriented()


def pitot_tube() -> MeshData:
    """``U_Pitot``: sol glove hücum kenarından öne 90 mm paslanmaz pitot (taban Ø12 → boru Ø6)."""
    f = P.pitot()
    L = float(f.params["protrusion_m"])
    r, rb = 0.5 * float(f.params["d_m"]), 0.5 * float(f.params["base_d_m"])
    prof = np.array([[-0.012, 0.0], [-0.012, rb], [0.0, rb], [0.006, 0.92 * rb], [0.026, r * 1.05], [0.030, r],
                     [L - 0.002, r], [L - 0.0004, r - 0.0008], [L, 0.0011], [L, 0.0]])
    md = lathe(prof, 32, "U_Pitot", M["steel"], "x", space="spec")
    md.verts = np.asarray(f.pos) + md.verts * np.array([-1.0, 1.0, 1.0])
    md.faces = [tuple(reversed(fc)) for fc in md.faces]
    return md.oriented()


def ellipsoid(name: str, center, axes: np.ndarray, radii, mat: str, nu: int = 28, nv: int = 14) -> MeshData:
    """Genel elipsoit (``axes`` 3×3 sütunlar birim eksenler, spec takımı)."""
    prof = np.column_stack([-np.cos(np.linspace(0, math.pi, nv)), np.sin(np.linspace(0, math.pi, nv))])
    md = lathe(prof, nu, name, mat, "x", space="spec")
    loc = md.verts * np.asarray(radii)
    md.verts = np.asarray(center) + loc @ np.asarray(axes).T
    return md.oriented()


def nav_light(f: P.Feature) -> MeshData:
    """``U_Light_Nav_L/R``: eğik uç hücum kenarına gömülü damla lens (iskele kırmızı, sancak yeşil)."""
    n = np.asarray(f.direction)
    tng = _unit(np.array([abs(n[1]), np.sign(n[1]) * abs(n[0]), 0.0]))      # HK boyunca geriye-dışa
    ax = np.column_stack([tng, n, [0.0, 0.0, 1.0]])
    mat = M["nav_red"] if f.params.get("color") == "red" else M["nav_green"]
    c = np.asarray(f.pos) - 0.0012 * n
    return ellipsoid(f.obj_name, c, ax, (0.011, 0.0032, 0.0034), mat)


def strobe_light(f: P.Feature) -> MeshData:
    """``U_Light_Strobe_L/R``: dikey ucu firar kenarında beyaz strobe lensi."""
    st = P.fin_station(float(_T["fin"]["height_m"]), "L" if f.pos[1] > 0 else "R")
    ax = np.column_stack([[1.0, 0.0, 0.0], st.normal, st.span_dir])
    return ellipsoid(f.obj_name, f.pos, ax, (0.010, 0.0034, 0.0042), M["strobe"])


def landing_light() -> MeshData:
    """``U_Light_Landing``: sağ glove hücum kenarında iniş farı lensi (pitotun aynası, y = −0,17). varsayım"""
    p = P.pitot()
    y = -p.pos[1]
    sec = P.wing_section(y)
    i = (len(sec) - 1) // 2
    pos = sec[i]
    ax = np.column_stack([[-1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]])
    return ellipsoid("U_Light_Landing", pos + np.array([0.0012, 0.0, 0.0]), ax, (0.0028, 0.010, 0.0062), M["strobe"])


FIN_ROOT_FAIRING = {"s0": 2.136, "s1": 2.414, "proud": 0.0018}   # dikey kökü mermisi: uçlar, yüzeylerden taşma (≥ 1,5 mm). varsayım


def _section_half_thickness(sec: np.ndarray, s: np.ndarray) -> np.ndarray:
    """Kesitin (üst FK→HK, alt HK→FK) ``s`` istasyonlarında iki yüzey arası yarı kalınlığı (3B uzaklığın yarısı)."""
    m = (len(sec) + 1) // 2
    up, lo = sec[:m][::-1], sec[m - 1:]
    out = np.zeros(len(s))
    for k in range(3):
        a = np.interp(s, np.maximum.accumulate(up[:, 0]), up[:, k])
        b = np.interp(s, np.maximum.accumulate(lo[:, 0]), lo[:, k])
        out += (a - b) ** 2
    inside = (s >= sec[:, 0].min()) & (s <= sec[:, 0].max())
    return np.where(inside, 0.5 * np.sqrt(out), 0.0)


def fin_root_fairing(side: str = "L") -> MeshData:
    """``U_Fairing_FinRoot_<s>``: stabilize ucu–dikey kökü birleşiminde mermi kaporta (s 2,136–2,414; boyalı). Yarıçapı
    her istasyonda dikey kök kesiti ve stabilize uç kesitinin yarı kalınlığından ``proud`` (1,8 mm) büyüktür → iki
    yüzeyden de ≥ 1,5 mm taşar ve ayrı basılabilir kabuk olur (eski Ø17 mermi %98 yüzeylerin içindeydi). Önde
    eliptik burun, en kalın yerden sonra tek tepeli (monoton) profil, dikey firar kenarının 2 cm gerisinde sivri uç.
    Dikey kirişi birleşimi, dümen alt yatağı, çakar/servo kablosunu taşır; kuyruk çarpma sırası değişmez."""
    F = FIN_ROOT_FAIRING
    sd = "L" if side.upper() == "L" else "R"
    sg = 1.0 if sd == "L" else -1.0
    y_root, z_root = float(_T["fin"]["y_root_m"]), float(_T["fin"]["z_root_m"])
    fin = P.fin_section(0.0, sd)
    stab = P.stab_section(sg * y_root)
    s_le = float(min(fin[:, 0].min(), stab[:, 0].min()))
    ss = np.linspace(s_le, F["s1"], 300)
    req = np.maximum(_section_half_thickness(fin, ss), _section_half_thickness(stab, ss)) + F["proud"]
    req = np.convolve(np.r_[np.full(4, req[0]), req, np.full(4, req[-1])], np.ones(9) / 9.0, mode="valid")
    k = int(np.argmax(req))
    r = np.r_[np.maximum.accumulate(req[:k + 1]), np.maximum.accumulate(req[k:][::-1])[::-1][1:]]
    s_te = float(fin[:, 0].max())
    tail = ss > s_te
    r[tail] = r[~tail][-1] * np.clip(1.0 - ((ss[tail] - s_te) / (F["s1"] - s_te)) ** 1.6, 0.0, 1.0) ** 0.8
    r[-1] = 0.0
    r_le = float(r[0])
    a = np.linspace(0.0, 0.5 * math.pi, 9)[:-1]                     # eliptik burun: s0 → HK
    s_nose = s_le - (s_le - F["s0"]) * np.cos(a)
    r_nose = r_le * np.sin(a)
    dense = np.flatnonzero(ss < s_le + 0.045)                            # hücum kenarında kalınlık hızlı artar: sık
    keep = np.unique(np.r_[dense, np.arange(0, len(ss), 5), k, len(ss) - 1])
    prof = np.column_stack([np.r_[s_nose, ss[keep]], np.r_[r_nose, r[keep]]])
    prof[0, 1] = 0.0
    prof[:, 0] -= F["s0"]
    md = lathe(prof, 32, f"U_Fairing_FinRoot_{sd}", M["skin_top"], "x", space="spec")
    md.verts = md.verts + np.array([F["s0"], sg * y_root, z_root])
    return md.oriented()


# =====================================================================================================
# Şablon yazılar, servis işaretleri ve anten tabanları (F8)
# =====================================================================================================
@dataclass
class StencilSpec:
    """Bir işaretin yerleşimi: ``host`` ev sahibi nesne adı, ``text`` (None → ``poly`` 2B ağ (V2 m, F)), ``height``
    büyük harf yüksekliği (m), ``center`` spec noktası (yüzeye yakın), ``u_b``/``n_b`` Blender okuma yönü ve dış normal."""

    name: str
    host: str
    text: str | None
    height: float
    mat: str
    center: tuple
    u_b: tuple
    n_b: tuple
    poly: tuple | None = None


def _disc2d(r: float, n: int = 24) -> tuple[np.ndarray, list]:
    th = np.linspace(0, 2 * math.pi, n, endpoint=False)
    V = np.vstack([[0.0, 0.0], np.column_stack([r * np.cos(th), r * np.sin(th)])])
    return V, [[0, 1 + k, 1 + (k + 1) % n] for k in range(n)]


def _ring2d(r0: float, r1: float, n: int = 32) -> tuple[np.ndarray, list]:
    th = np.linspace(0, 2 * math.pi, n, endpoint=False)
    V = np.vstack([np.column_stack([r0 * np.cos(th), r0 * np.sin(th)]), np.column_stack([r1 * np.cos(th), r1 * np.sin(th)])])
    return V, [[k, (k + 1) % n, n + (k + 1) % n, n + k] for k in range(n)]


def _rects2d(rects: list) -> tuple[np.ndarray, list]:
    """Dikdörtgenler [(x0, y0, x1, y1)] → 2B ağ."""
    V, F = [], []
    for x0, y0, x1, y1 in rects:
        b = len(V)
        V += [(x0, y0), (x1, y0), (x1, y1), (x0, y1)]
        F.append([b, b + 1, b + 2, b + 3])
    return np.asarray(V, float), F


def _dashes_along(p0, p1, w: float, on: float, off: float) -> list:
    """(x, y) doğru parçası boyunca kesik çizgi dikdörtgenleri (yalnız eksen paralel parçalar)."""
    (x0, y0), (x1, y1) = p0, p1
    L = math.hypot(x1 - x0, y1 - y0)
    n = max(1, int((L + off) // (on + off)))
    pad = 0.5 * (L - (n * on + (n - 1) * off))
    out = []
    for k in range(n):
        a = pad + k * (on + off)
        b = a + on
        if abs(y1 - y0) < 1e-9:
            xa, xb = sorted((x0 + a * np.sign(x1 - x0), x0 + b * np.sign(x1 - x0)))
            out.append((xa, y0 - 0.5 * w, xb, y0 + 0.5 * w))
        else:
            ya, yb = sorted((y0 + a * np.sign(y1 - y0), y0 + b * np.sign(y1 - y0)))
            out.append((x0 - 0.5 * w, ya, x0 + 0.5 * w, yb))
    return out


def stencil_specs() -> list[StencilSpec]:
    """``spec details.markings.stencils`` → yerleşimler (iki yan). Kumanda yüzeylerindeki "ADIM ATMA" yazıları menteşenin
    ``aft_of_hinge`` gerisinde başlar, yüzeye bağlıdır (birlikte döner); el tutma bölgesi iç flap oyuğunun önünde
    biter; statik port chine'ın 22 mm üstünde; yakıt ağzı CG'de sırtta; pervane uyarısı kaporta yanaklarında (panjur ve
    susturucu kabartısının üstünde)."""
    out: list[StencilSpec] = []
    up, down = (0.0, 0.0, 1.0), (0.0, 0.0, -1.0)
    for st in (_D.get("markings", {}) or {}).get("stencils", []) or []:
        mat = M[st.get("role", "accent")]
        h = float(st["h_m"])
        nm, txt, wh = st["name"], st["text"], st["where"]
        for sd in ("L", "R"):
            sg = 1.0 if sd == "L" else -1.0
            side_u = (-1.0, 0.0, 0.0) if sd == "L" else (1.0, 0.0, 0.0)      # yan yüz: soldan sağa okunur
            if wh == "surface_upper":
                for sname, yv in st["surfaces"].items():
                    hl = P.hinge_line(sname, sd)
                    y = sg * (float(yv) if yv is not None else abs(hl.mid[1]))
                    f = (abs(y) - abs(hl.p_in[1])) / (abs(hl.p_out[1]) - abs(hl.p_in[1]))
                    s_h = hl.p_in[0] + f * (hl.p_out[0] - hl.p_in[0])
                    z_h = hl.p_in[2] + f * (hl.p_out[2] - hl.p_in[2])
                    c = (s_h + float(st["aft_of_hinge_m"]) + 0.5 * h, y, z_h + 0.02)
                    host = f"U_{sname}_{sd}"
                    out.append(StencilSpec(f"U_Stencil_{nm}_{sname}_{sd}", host, txt, h, mat, c, (0.0, -1.0, 0.0), up))
            elif wh == "handling_zone":
                s0, s1 = (float(v) for v in st["s_m"])
                y0, y1 = (float(v) for v in st["y_m"])
                w = float(st["line_m"])
                on, off = (float(v) for v in st["dash_m"])
                # 2B: x = okuma yönü (−y), y = ileri (−s)
                xa, xb = -0.5 * (y1 - y0), 0.5 * (y1 - y0)
                ya, yb = -0.5 * (s1 - s0), 0.5 * (s1 - s0)
                rects = (_dashes_along((xa, ya), (xb, ya), w, on, off) + _dashes_along((xa, yb), (xb, yb), w, on, off)
                         + _dashes_along((xa, ya + w), (xa, yb - w), w, on, off)
                         + _dashes_along((xb, ya + w), (xb, yb - w), w, on, off))
                c = (0.5 * (s0 + s1), sg * 0.5 * (y0 + y1), 0.05)
                out.append(StencilSpec(f"U_Stencil_{nm}Zone_{sd}", f"U_WingCenter_{sd}", None, h, mat, c, (0.0, -1.0, 0.0),
                                       up, _rects2d(rects)))
                out.append(StencilSpec(f"U_Stencil_{nm}_{sd}", f"U_WingCenter_{sd}", txt, h, mat, c, (0.0, -1.0, 0.0), up))
            elif wh == "static_port":
                s = float(st["s_m"])
                sec = P.fuselage_section(s)
                z = sec.z_chine + float(st["dz_chine_m"])
                y = P.fuselage_half_width_at(sec, z)
                n_b = (0.0, sg, 0.0)
                r_o = 0.5 * float(st["ring_d_m"])
                out.append(StencilSpec(f"U_Stencil_{nm}Port_{sd}", "U_Fuselage", None, h, M["seal"], (s, sg * y, z), side_u,
                                       n_b, _disc2d(0.5 * float(st["port_d_m"]))))
                out.append(StencilSpec(f"U_Stencil_{nm}Ring_{sd}", "U_Fuselage", None, h, mat, (s, sg * y, z), side_u, n_b,
                                       _ring2d(r_o - float(st["ring_w_m"]), r_o)))
                zt = z - r_o - 0.0045
                yt = P.fuselage_half_width_at(sec, zt)
                out.append(StencilSpec(f"U_Stencil_{nm}_{sd}", "U_Fuselage", txt, h, mat, (s, sg * yt, zt), side_u, n_b))
            elif wh == "fuel_filler" and sd == "L":
                s = float(st["s_m"])
                z = P.fuselage_section(s).z_top
                ro, rc = 0.5 * float(st["ring_d_m"]), 0.5 * float(st["cap_d_m"])
                out.append(StencilSpec(f"U_Stencil_{nm}Ring", "U_Fuselage", None, h, mat, (s, 0.0, z), (-1.0, 0.0, 0.0), up,
                                       _ring2d(rc + 0.0006, ro)))
                out.append(StencilSpec(f"U_Stencil_{nm}Cap", "U_Fuselage", None, h, M["steel"], (s, 0.0, z), (-1.0, 0.0, 0.0),
                                       up, _disc2d(rc)))
                out.append(StencilSpec(f"U_Stencil_{nm}", "U_Fuselage", txt, h, mat, (s, 0.0 + ro + 0.006, z), (-1.0, 0.0, 0.0),
                                       up))
            elif wh == "cowl_side":
                s, z = float(st["s_m"]), float(st["z_m"])
                y, nrm = _cowl_y_at(s, z, sd)
                out.append(StencilSpec(f"U_Stencil_{nm}_{sd}", "U_Cowl", txt, h, mat, (s, sg * y, z), side_u,
                                       (0.0, sg * nrm[0], nrm[1])))
            elif wh == "fin_root":
                fs = P.fin_station(float(st["h_fin_m"]), sd)
                cf = P.hinge_line("Rudder", sd).chord_fraction
                c = np.asarray(fs.le) + np.array([0.5 * (1.0 - cf) * fs.chord, 0.0, 0.0])
                nb = P.vec_to_blender(fs.normal)
                nb = nb if nb[1] * sg > 0 else -nb
                out.append(StencilSpec(f"U_Stencil_{nm}_{sd}", f"U_Fin_{sd}", txt, h, mat, tuple(c), side_u, tuple(nb)))
            elif wh == "jack" and sd == "L":
                tri = float(st["tri_m"])
                for k, s in enumerate(st["s_m"]):
                    s = float(s)
                    z = P.fuselage_section(s).z_bottom
                    a = tri / math.sqrt(3.0)
                    V = np.array([(0.0, 0.0), (-0.5 * tri, -1.5 * a), (0.5 * tri, -1.5 * a)]) + np.array([0.0, 0.75 * a])
                    out.append(StencilSpec(f"U_Stencil_{nm}Tri_{k + 1}", "U_Fuselage", None, h, mat, (s, 0.0, z),
                                           (0.0, 1.0, 0.0), down, (V, [[0, 1, 2]])))
                    out.append(StencilSpec(f"U_Stencil_{nm}_{k + 1}", "U_Fuselage", txt, h, mat, (s + 0.016, 0.0, z),
                                           (0.0, 1.0, 0.0), down))
    return out


def antenna_doubler(f: P.Feature) -> MeshData:
    """``U_Antenna_Doubler_<ad>``: bıçak anten tabanında eliptik takviye plakası (1,2 mm kabarık, boy 1,35 × kök veteri,
    en 14 mm; ``antenna``) ve 4 adet Ø1,6 mm vida başı (``steel``)."""
    ad = _D["markings"].get("antenna_doubler", {"length_x_chord": 1.35, "w_m": 0.014, "t_m": 0.0012, "screw_d_m": 0.0016})
    L = float(ad["length_x_chord"]) * float(f.params["chord_root_m"])
    W, T = float(ad["w_m"]), float(ad["t_m"])
    dz = float(np.sign(f.direction[2]))
    s_c = f.pos[0] - 0.40 * float(f.params["chord_root_m"]) + 0.5 * float(f.params["chord_root_m"])
    th = np.linspace(0, 2 * math.pi, 40, endpoint=False)
    poly = np.column_stack([s_c + 0.5 * L * np.cos(th), f.pos[1] + 0.5 * W * np.sin(th)])
    side = "top" if dz > 0 else "bottom"

    def zs(s, y):
        z = P.fuselage_z_at(s, y, side)
        return float(z if z is not None else (P.fuselage_section(s).z_top if dz > 0 else P.fuselage_section(s).z_bottom))

    plate = heightfield_panel("doubler", poly, lambda s, y: zs(s, y) - dz * 0.0003, T + 0.0003, M["antenna"], M["antenna"],
                              n_rows=10, col_step=0.002, up=dz)
    parts = [plate]
    r = 0.5 * float(ad["screw_d_m"])
    for ds, dy in ((-0.38 * L, 0.0), (0.38 * L, 0.0), (-0.12 * L, 0.32 * W), (-0.12 * L, -0.32 * W)):
        s, y = s_c + ds, f.pos[1] + dy
        z = zs(s, y) + dz * T
        md = lathe(np.array([[-0.0003, 0.0], [-0.0003, r], [0.0002, r], [0.0004, 0.0]]), 10, "screw", M["steel"], "z",
                   space="spec")
        md.verts = md.verts * np.array([1.0, 1.0, dz]) + np.array([s, y, z])
        parts.append(md.oriented())
    return merge(f"U_Antenna_Doubler_{f.name.split('_')[-1]}", parts, 30.0)


# =====================================================================================================
# Paketleme zarfları (render dışı; motor, susturucu, depolar, akü — AERO-08)
# =====================================================================================================
def env_mesh(part: P.EnvPart, name: str | None = None, mat: str = M["engine"]) -> MeshData:
    """``params.EnvPart`` → kapalı ağ (spec takımı): kutu ya da silindir."""
    A = np.asarray(part.axes, float).T                    # sütunlar: eksenler
    c = np.asarray(part.center, float)
    hx, hy, hz = part.half
    if part.kind == "cyl":
        md = lathe(np.array([[-hx, 0.0], [-hx, hy], [hx, hy], [hx, 0.0]]), 24, name or part.name, mat, "x", space="spec")
    else:
        mb = MeshBuilder(name or part.name)
        sq = np.array([(-1, -1), (1, -1), (1, 1), (-1, 1)], float)
        bot = mb.add(np.column_stack([np.full(4, -hx), sq[:, 0] * hy, sq[:, 1] * hz]))
        top = mb.add(np.column_stack([np.full(4, hx), sq[:, 0] * hy, sq[:, 1] * hz]))
        mb.strip(bot, top, mat)
        mb.cap(bot, mat, start=True)
        mb.cap(top, mat, start=False)
        md = mb.build(None)
    md.verts = c + np.asarray(md.verts, float) @ A.T
    md.name = name or part.name
    return md.oriented()


def _cube_grid(ts: Sequence[np.ndarray]) -> tuple[np.ndarray, list]:
    """Birim küp ([−1, 1]³) yüzeyinde eksen başına verilen düğümlerle (``ts[i]``, ±1 dahil) paylaşılan köşeli ızgara
    (kapalı, dışa dönük dörtgenler)."""
    keyed: dict = {}
    V: list = []

    def vid(p):
        k = tuple(np.round(p, 9))
        if k not in keyed:
            keyed[k] = len(V)
            V.append(p)
        return keyed[k]

    F = []
    for ax in range(3):
        o1, o2 = [k for k in range(3) if k != ax]
        ta, tb = ts[o1], ts[o2]
        for sg in (-1.0, 1.0):
            for i in range(len(ta) - 1):
                for j in range(len(tb) - 1):
                    quad, pts = [], []
                    for a, b in ((ta[i], tb[j]), (ta[i + 1], tb[j]), (ta[i + 1], tb[j + 1]), (ta[i], tb[j + 1])):
                        q = np.zeros(3)
                        q[ax], q[o1], q[o2] = sg, a, b
                        quad.append(vid(q))
                        pts.append(q)
                    nrm = np.cross(pts[1] - pts[0], pts[2] - pts[0])
                    if nrm[ax] * sg < 0:                       # dışa dönük sıra
                        quad = quad[::-1]
                    F.append(tuple(quad))
    return np.asarray(V, float), F


def envelope_keepout(part: "P.EnvPart", r: float, n: int = 6, name: str | None = None) -> MeshData:
    """Zarfın ``r`` kadar YUVARLAK genişletilmiş hâli (Minkowski toplamı: zarf ⊕ küre) — kapalı ağ, spec takımı. Kutu:
    küp-küre ızgarası (yüzler düz, kenarlar silindir, köşeler küre); silindir: yarıçap + r, uçları torus kenarlı.
    Baskıda kaporta parçalarının çerçeve/flanşlarından çıkarılır: her motor zarfına ≥ r pay (R01). Gerçek uzaklık
    ölçüsü (köşegende de r), eksen boyunca büyütülmüş kutu gibi kabuğu gereksiz kesmez."""
    A = np.asarray(part.axes, float).T                    # sütunlar: eksenler
    c = np.asarray(part.center, float)
    h = np.asarray(part.half, float)
    nm = name or f"U_Keepout_{part.name}"
    if part.kind == "cyl":
        L, R = h[0], h[1]
        a = np.linspace(0.0, 0.5 * math.pi, max(3, n))
        prof = [(-L - r, 0.0)] + [(-L - r * math.cos(t), R + r * math.sin(t)) for t in a] \
            + [(L + r * math.sin(t), R + r * math.cos(t)) for t in a] + [(L + r, 0.0)]
        md = lathe(np.asarray(prof), 64, nm, M["engine"], "x", space="spec", smooth_angle=None)
        md.verts = c + np.asarray(md.verts, float) @ A.T
        return md.oriented()
    ts = []
    for hi in h:                                           # düz bölge seyrek, yuvarlak kenar bandı (h → h + r) sık
        e = hi / (hi + r)
        ts.append(np.unique(np.round(np.r_[np.linspace(-1.0, -e, n), np.linspace(-e, e, 5), np.linspace(e, 1.0, n)], 12)))
    G, F = _cube_grid(ts)
    big = G * (h + r)
    q = np.clip(big, -h, h)
    d = big - q
    nrm = np.linalg.norm(d, axis=1, keepdims=True)
    Vloc = q + r * d / np.where(nrm > 0, nrm, 1.0)
    V = c + Vloc @ A.T
    md = MeshData(nm, V, F, np.zeros(len(F), int), [M["engine"]], "spec", None)
    return md.oriented()


def engine_keepouts() -> list[MeshData]:
    """Baskı kaporta parçaları için motor yasak bölgeleri (R01): DLE-20 zarfları ``clearance_m.cowl`` (5 mm), susturucu
    ``clearance_m.muffler_air_gap`` (15,5 mm) yuvarlak genişletilmiş. Sahne kabuğu bunlara değmez (``engine_bay_clearance``
    ≥ bu paylar); printprep flanş/çerçeve ve iç yapıyı bunlarla kırpar → UP_cowl_* ↔ U_Env_* çakışması 0."""
    cl = _PR["engine"]["clearance_m"]
    out = []
    for part in P.engine_envelope():
        r = float(cl["muffler_air_gap"]) if part.name == "muffler" else float(cl["cowl"])
        out.append(envelope_keepout(part, r + 0.00015))    # çokgen yüz kirişleri ≥ pay kalsın (+0,15 mm)
    return out


def envelopes() -> dict[str, MeshData]:
    """Render dışı paketleme zarfları: ``U_Env_Engine`` (ters DLE-20: rulman burnu, karter, karbüratör, silindir,
    buji başlığı), ``U_Env_Muffler``, ``U_Env_Tank_L/R``, ``U_Env_Battery``."""
    eng = [env_mesh(p) for p in P.engine_envelope() if p.name != "muffler"]
    out = {"U_Env_Engine": merge("U_Env_Engine", eng, None)}
    mf = next(p for p in P.engine_envelope() if p.name == "muffler")
    out["U_Env_Muffler"] = env_mesh(mf, "U_Env_Muffler", M["exhaust"])
    for tp in P.tank_envelopes():
        nm = f"U_Env_Tank_{tp.name[-1]}"
        out[nm] = env_mesh(tp, nm, M["pacf"])
    out["U_Env_Battery"] = env_mesh(P.battery_envelope(), "U_Env_Battery", M["carbon"])
    return out


# =====================================================================================================
# Rapor (CLI)
# =====================================================================================================
def all_static() -> list[MeshData]:
    """Bütün spec-uzayı statik ağlar (rapor/test için; Blender kurucusu parçaları tek tek çağırır)."""
    out = [fuselage(), cowl(), cowl(print_solid=True), cowl_cavity(), exhaust_ring(), intake(), hatch(), cowl_louvers(),
           muffler_pipe(), scuff_pad(), stab_fillet(), chevron()]
    for sd in ("L", "R"):
        out += list(wing_parts(sd).values()) + list(fin_parts(sd).values())
        out += [wing_fillet(sd), root_fairing(sd), unit_blister(sd), stripe(sd)]
    out += list(stab_parts().values())
    for leg in ("N", "L", "R"):
        out.append(gear_bay(leg))
    for d in P.gear_doors():
        out.append(gear_door(d.name).mesh)
    for a in P.antennas():
        out.append(blade_antenna(a) if a.kind == "blade" else gnss_puck(a) if a.kind == "puck" else dipole(a))
    out += [pitot_tube(), landing_light()] + [nav_light(f) for f in P.lights() if f.kind == "nav"]
    out += [strobe_light(f) for f in P.lights() if f.kind == "strobe"]
    return out


if __name__ == "__main__":
    import time

    t0 = time.time()
    parts = all_static() + [prop(), spinner(), turret_mount(), turret_pan(), turret_ball(), turret_windows(), turret_ring()]
    bad = 0
    for md in parts:
        r = md.check()
        ok = r["boundary"] == 0 and r["nonmanifold"] == 0 and r["flipped"] == 0 and r["volume"] > 0
        bad += 0 if ok else 1
        print(f"{'OK ' if ok else 'HATA'} {r['name']:26s} köşe {r['n_verts']:6d}  yüz {r['n_faces']:6d}  "
              f"hacim {r['volume'] * 1e6:10.1f} cm³  malzeme {','.join(md.mats)}")
    print(f"{len(parts)} parça, {bad} hatalı, {time.time() - t0:.1f} s")
