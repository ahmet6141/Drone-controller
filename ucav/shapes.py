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

* Gövde: ``fuselage()``, ``cowl()``, ``exhaust_ring()``, ``intake()``, ``hatch()``, ``stripe(side)``,
  ``chevron()``, ``cowl_louvers()``, ``muffler_pipe()``, ``scuff_pad()``; halka düzeni ``FUS_RING`` /
  ``fuselage_ring_yz(s)`` (chine pahı ve bant kenarları sabit indislerde).
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


def _window(t, edge: float = 0.3):
    """[0, 1] aralığında 0 → 1 → 0 yumuşak pencere (kenarlarda ``edge`` genişliğinde smoothstep)."""
    t = np.asarray(t, float)
    return _smoothstep(t / edge) * _smoothstep((1.0 - t) / edge) * ((t >= 0) & (t <= 1))


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
                                                 float(_F["modules"]["firewall_s_m"]), 2.19, NOSE_CAP_S,
                                                 BAND_TAPER_S[0], BAND_TAPER_S[1],
                                                 float(cw["cheek_left"]["s_from_m"]), float(cw["cheek_left"]["s_to_m"])]
    keys += list(np.linspace(cw["s_from_m"], 2.19, 24))
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


def _cheek_offset(s: float, yz: np.ndarray) -> np.ndarray:
    """Kaporta yanakları: sol susturucu kabartısı, sağ panjurlu yanak (dış normal boyunca yumuşak tümsek)."""
    cw = _PR["cowl"]
    out = yz.copy()
    nrm = _ring_normals_yz(yz)
    for key, sg in (("cheek_left", 1.0), ("cheek_right", -1.0)):
        c = cw[key]
        ts = (s - float(c["s_from_m"])) / (float(c["s_to_m"]) - float(c["s_from_m"]))
        if not (0.0 < ts < 1.0):
            continue
        tz = (yz[:, 1] - float(c["z_from_m"])) / (float(c["z_to_m"]) - float(c["z_from_m"]))
        w = float(_window(ts, 0.35)) * _window(tz, 0.40) * (yz[:, 0] * sg > 0)
        out += nrm * (w * float(c["bulge_m"]))[:, None]
    return out


COWL_CAVITY = {"r": 0.0455, "s_floor": 2.150, "s_boss": 2.168}   # lüle iç boşluğu (koyu). varsayım


def cowl() -> MeshData:
    """``U_Cowl``: yangın perdesi (2,06) → lüle halkası önü (2,19); yanak kabartıları ve arkada koyu halka
    çıkış boşluğu (spinner çevresi)."""
    s0 = float(_PR["cowl"]["s_from_m"])
    s_end = float(_PR["exhaust_ring"]["s_from_m"])
    ss = fuselage_s_stations(s0, s_end)
    mb = MeshBuilder("U_Cowl")
    rings, yzs = [], []
    for s in ss:
        yz = _cheek_offset(s, fuselage_ring_yz(s))
        yzs.append(yz)
        rings.append(mb.add(_ring3(s, yz)))
    for k in range(len(rings) - 1):
        s_a, s_b = ss[k], ss[k + 1]
        mb.strip(rings[k], rings[k + 1], lambda j, a=s_a, b=s_b: fuselage_mat(a, b, j, band=False))
    mb.cap(rings[0], M["skin_bottom"], start=True)
    # arka: halka boşluğu (iç yarıçap COWL_CAVITY.r) → taban → göbek çıkıntısı
    zc = P.PROP.hub[2]
    yz = yzs[-1]
    ang = np.arctan2(yz[:, 1] - zc, yz[:, 0])
    rc = COWL_CAVITY["r"]
    circ = lambda s, r: np.column_stack([np.full(len(ang), s), r * np.cos(ang), zc + r * np.sin(ang)])
    c0 = mb.add(circ(s_end, rc))
    c1 = mb.add(circ(COWL_CAVITY["s_floor"], rc))
    c2 = mb.add(circ(COWL_CAVITY["s_floor"], 0.4 * rc))
    pole = int(mb.add(np.array([[COWL_CAVITY["s_boss"], 0.0, zc]]))[0])
    mb.strip(rings[-1], c0, M["pacf"])
    mb.strip(c0, c1, M["pacf"])
    mb.strip(c1, c2, M["pacf"])
    mb.fan(pole, c2, M["pacf"], start=False)
    _mark_chine(mb, rings, ss)
    return mb.build(28.0)


def exhaust_ring() -> MeshData:
    """``U_ExhaustRing``: lüle halkası (s 2,19–2,205; dış Ø0,112, iç Ø0,092), arka dudağı yuvarlatılmış."""
    er = _PR["exhaust_ring"]
    s0, s1 = float(er["s_from_m"]), float(er["s_to_m"])
    ri = 0.5 * float(er["id_m"])
    zc = P.PROP.hub[2]
    mb = MeshBuilder("U_ExhaustRing")
    outer_s = [s0 - 0.002, s0 + 0.0012, s0 + 0.004, s1 - 0.006, s1 - 0.0025]
    shrink = [0.985, 0.992, 1.0, 1.0, 1.0]               # kaportanın içinden başlar; 0,4 mm V-oluk = panel çizgisi
    yz_end = fuselage_ring_yz(s1)
    ang = np.arctan2(yz_end[:, 1] - zc, yz_end[:, 0])
    ro = float(np.mean(np.hypot(yz_end[:, 0], yz_end[:, 1] - zc)))
    rings = []
    for s, f in zip(outer_s, shrink):                   # dış yüz: kaporta ile aynı halka (basamaksız birleşim)
        yz = fuselage_ring_yz(min(max(s, s0), s1))
        yz = np.column_stack([yz[:, 0] * f, zc + (yz[:, 1] - zc) * f])
        rings.append(mb.add(np.column_stack([np.full(len(yz), s), yz[:, 0], yz[:, 1]])))
    yz_last = fuselage_ring_yz(outer_s[-1])
    ang = np.arctan2(yz_last[:, 1] - zc, yz_last[:, 0])
    ro = float(np.mean(np.hypot(yz_last[:, 0], yz_last[:, 1] - zc)))
    # yuvarlak dudak: dış → iç yarım elips (geriye 2,5 mm)
    lip_c = 0.5 * (ro + ri)
    lip_r = 0.5 * (ro - ri)
    for a in np.linspace(0.0, math.pi, 9)[1:]:
        s = s1 - 0.0025 + 0.0025 * math.sin(a)
        r = lip_c + lip_r * math.cos(a)
        rings.append(mb.add(np.column_stack([np.full(len(ang), s), r * np.cos(ang), zc + r * np.sin(ang)])))
    rings.append(mb.add(np.column_stack([np.full(len(ang), s0 - 0.002), ri * np.cos(ang), zc + ri * np.sin(ang)])))
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


LE_STRIP = 0.035                         # antrasit hücum kenarı (aşınma) şeridi: üst ve altta veterin %3,5'i. varsayım
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
                 oblique_ends: bool = False, start_plane=None):
    """Taşıyıcı yüzey parçası (sabit deri, oyuklu) ve içindeki kumanda yüzeyleri.

    Dönüş: ``(MeshData sabit parça, {nesne_adı: MeshData yüzey}, {nesne_adı: [(t, 2B halka)…]})``.
    Boşluk sınırlarında (t0, t1) tam ve oyuklu halka aynı t'de ön kısmı paylaşır; aradaki uç duvarı n-gendir.
    ``oblique_ends``: boşluk uç duvarları ve yüzey uç kaburgaları menteşe eksenine dik düzlemlerdedir (gerçek
    kanattaki gibi; dihedral/ok nedeniyle dönüşte uç kayması olmaz, aralık her açıda menteşe aralığıdır). Aksi
    halde uçlar kesit düzlemindedir ve uç boşluğu dönüş zarfından hesaplanır (``surface_end_gaps``).
    ``start_plane``: ilk halkanın arka kısmı bu düzleme oturtulur (komşu parçadaki eğik duvarla eşleşme).
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
    mat_ring = lambda j: (M["accent"] if c_le - ks <= j < c_le + ks else (mat_up if j < c_le else mat_lo))
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
            mb.strip(prev, cur_in, mat_ring)
        prev = cur_out
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
    for part, names in plan.items():
        cc = [Cutout(cuts[n], cuts[n].span_from, cuts[n].span_to) for n in names]
        fixed, surfs, _ = lifting_part(f"{part}_{sd}", fam, wing_stations(part), cc, nf=_WING_N["nf"],
                                       na=_WING_N["na"], nn=_WING_N["nn"], ns=_WING_N["ns"], zip_end=(part == "U_Tip"),
                                       oblique_ends=True, start_plane=tip_plane if part == "U_Tip" else None)
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


def fin_parts(side: str = "L") -> dict[str, MeshData]:
    """``U_Fin_<s>`` (kök stabilize altında yuvarlak, uç yuvarlak kapalı) ve ``U_Rudder_<s>``."""
    sd = "L" if side.upper() == "L" else "R"
    fam = FinFamily(sd)
    H = fam.H
    h = P.hinge_line("Rudder", sd)
    cuts = [Cutout(h, h.span_from, h.span_to)]
    root = -FIN_ROOT_ROUND * np.cos(np.linspace(0, 0.5 * math.pi, 6))
    tip = H - FIN_TIP_ROUND + FIN_TIP_ROUND * np.sin(np.linspace(0, 0.5 * math.pi, 7))
    st = np.r_[root, _dense_between(0.0, H - FIN_TIP_ROUND, 0.012), tip, h.span_from, h.span_to]
    fixed, surfs, _ = lifting_part(f"U_Fin_{sd}", fam, st, cuts, nf=_TAIL_N["nf"], na=_TAIL_N["na"], nn=_TAIL_N["nn"],
                                   ns=_TAIL_N["ns"], mat_lo=M["skin_top"], zip_start=True, zip_end=True)
    out = {fixed.name: fixed}
    for k, v in surfs.items():
        v.face_mat[:] = v.mats.index(M["skin_top"]) if M["skin_top"] in v.mats else 0
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
    """``U_Fairing_StabFillet``: stabilize kökü–kuyruk konisi filetoları (üst ve alt, iki yan; PA-CF)."""
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
            parts.append(fillet_solid(f"stabfillet_{which}_{sd}", ss, tab, which, dA, dB, sd, M["pacf"],
                                      y_min=0.02))
    return merge("U_Fairing_StabFillet", parts, 35.0)


RF = _W["root_fairing"]


def _fairing_ramp(s: float) -> float:
    a, b = float(RF["s_from_m"]), float(RF["s_to_m"])
    return float(_smoothstep((s - a) / 0.07) * _smoothstep((b - s) / 0.05))


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
    u = (s - float(UB["s_from_m"])) / (float(UB["s_to_m"]) - float(UB["s_from_m"]))
    v = (abs(y) - float(UB["y_from_m"])) / (float(UB["y_to_m"]) - float(UB["y_from_m"]))
    if not (0 <= u <= 1 and 0 <= v <= 1):
        return 0.0
    return float(_smoothstep(u / 0.38) * _smoothstep((1 - u) / 0.5) * _smoothstep(v / 0.25) * _smoothstep((1 - v) / 0.3))


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
INTAKE_WALL = 0.004                     # hava alığı et/dudak kalınlığı (görsel). varsayım
INTAKE_DUCT = 0.032                     # ağızdan iç bölmeye görünen kanal derinliği. varsayım
INTAKE_N = 3.2                          # kesit süperelips üssü (yuvarlatılmış dikdörtgen). varsayım


def _skin_top(s: float, y: float = 0.0) -> float:
    z = P.fuselage_z_at(s, y, "top")
    return float(z if z is not None else P.fuselage_section(s).z_top)


def intake() -> MeshData:
    """``U_Intake``: sınır tabaka ayırıcılı yükseltilmiş sırt hava alığı. Ağız ``params.intake_spec()``
    ölçülerinde (75 × 36 mm, ağız altı deriden ``diverter`` kadar yukarıda); gövde arkaya doğru sırt omurgasına
    teğet karışır. Dudak antrasit, içte turkuaz halka, kanal tabanı koyu."""
    I = P.intake_spec()
    s0, s1 = float(I["s_from_m"]), float(I["s_to_m"])
    w, h = 0.5 * float(I["mouth_w_m"]), float(I["mouth_h_m"])
    zmc = float(I["mouth_center"][2])
    zib, zit = zmc - 0.5 * h, zmc + 0.5 * h            # iç ağız alt/üst
    wall, lr = INTAKE_WALL, float(I["lip_r_m"])
    s_end = s1 + 0.035
    m = 64
    sk0 = _skin_top(s0)
    mb = MeshBuilder("U_Intake")

    def outer(s):
        u = float(np.clip((s - s0) / (s_end - s0), 0, 1))
        a = (w + wall) * (1 - 0.30 * _smoothstep(u))
        zt = _skin_top(s) + (zit + wall - sk0) * (1 - _smoothstep(u))
        zb = _skin_top(s) + (zib - wall - sk0) - (zib - wall - sk0 + 0.010) * _smoothstep((s - s0) / 0.045)
        return a, zb, max(zt, zb + 0.002)

    def inner(s):
        a, zb, zt = outer(s)
        return w, zib, min(zit, zt - wall)

    rings, mats = [], []
    for s in np.linspace(s_end, s0, 34):
        a, zb, zt = outer(s)
        rings.append(np.column_stack([np.full(m, s), superellipse_ring(a, zb, zt, INTAKE_N, m)]))
        mats.append(M["skin_top"])
    ao, zbo, zto = outer(s0)
    for al in np.linspace(0, math.pi, 11)[1:-1]:
        f = 0.5 * (1 - math.cos(al))
        a = ao + (w - ao) * f
        zb = zbo + (zib - zbo) * f
        zt = zto + (zit - zto) * f
        rings.append(np.column_stack([np.full(m, s0 - lr * math.sin(al)), superellipse_ring(a, zb, zt, INTAKE_N, m)]))
        mats.append(M["accent"])
    for s in np.linspace(s0, s0 + INTAKE_DUCT, 9):
        a, zb, zt = inner(s)
        rings.append(np.column_stack([np.full(m, s), superellipse_ring(a, zb, zt, INTAKE_N, m)]))
        mats.append(M["stripe"] if s <= s0 + 0.006 else M["accent"])
    idx = [mb.add(r) for r in rings]
    for k in range(len(idx) - 1):
        mb.strip(idx[k], idx[k + 1], mats[k + 1] if k >= 33 else mats[k])
    mb.cap(idx[0], M["skin_top"], start=True)
    mb.cap(idx[-1], M["pacf"], start=False)
    return mb.build(40.0)


def _hatch_geom():
    H = _D["hatch"]
    s0, s1, w = float(H["s_from_m"]), float(H["s_to_m"]), float(H["half_width_m"])
    k = w * math.tan(_rad(float(H["chevron_deg"])))
    return s0, s1, w, k, float(H["bulge_m"])


HATCH_FACET = {"v_top": 0.46, "d_end": 0.045, "rim": 0.0009}   # üst faset genişliği, uç rampaları, kenar basamağı. varsayım


def hatch() -> MeshData:
    """``U_Hatch``: füme PETG aviyonik kapağı ("sahte kanopi") — 36° şevron ön/arka uçlu, üst + iki yan faset,
    1,6 mm kabuk. Plan çokgeni ``params.hatch_outline()``."""
    s0, s1, w, k, bulge = _hatch_geom()
    L = s1 - s0 - k
    vc, d_end, rim = HATCH_FACET["v_top"], HATCH_FACET["d_end"], HATCH_FACET["rim"]
    uc = d_end / L
    v = np.unique(np.r_[np.linspace(-1, -vc, 6), np.linspace(-vc, vc, 11), np.linspace(vc, 1, 6)])
    u = np.unique(np.r_[np.linspace(0, uc, 5), np.linspace(uc, 1 - uc, 24), np.linspace(1 - uc, 1, 5)])
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
            mb.face((ii[i, j], ii[i, j + 1], ii[i + 1, j + 1], ii[i + 1, j]), mat)
    ring_o = list(io[0, :]) + list(io[1:, -1]) + list(io[-1, -2::-1]) + list(io[-2:0:-1, 0])
    ring_i = list(ii[0, :]) + list(ii[1:, -1]) + list(ii[-1, -2::-1]) + list(ii[-2:0:-1, 0])
    mb.strip(ring_o, ring_i, mat)
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


CHEVRON = {"y_c": 1.30, "span": 0.30, "w": 0.026, "x0": 0.12}   # sol kanat altı yönelim şevronu: merkez y, açıklık,
                                                                  # kol genişliği, tepe veter oranı. varsayım


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


def cowl_louvers() -> MeshData:
    """``U_Cowl_Louvers``: sağ yanakta 4 adet 36° "köpekbalığı solungacı" panjur. Her solungaç yanak yüzeyini
    boydan boya izleyen kama: ön kenarı deriye gömülü, arka kenarı 2,4 mm kalkık (çıkış ağzı geriye bakar)."""
    c = _PR["cowl"]["cheek_right"]
    n = int(c["louvers"])
    ang = _rad(float(c["louver_angle_deg"]))
    d = np.array([math.sin(ang), -math.cos(ang)])          # (s, z): gill ekseni, geriye-aşağı
    e = np.array([math.cos(ang), math.sin(ang)])           # (s, z): gill genişliği, geriye-yukarı
    zc = 0.5 * (float(c["z_from_m"]) + float(c["z_to_m"]))
    s_a, s_b = float(c["s_from_m"]), float(c["s_to_m"])
    centers = np.linspace(s_a + 0.26 * (s_b - s_a), s_a + 0.70 * (s_b - s_a), n)
    L, Wd, lift = 0.046, 0.0055, 0.0024
    parts = []
    for sc in centers:
        mb = MeshBuilder("louver")
        rings = []
        for k, a_ in enumerate(np.linspace(-0.5, 0.5, 13)):
            taper = math.sqrt(max(0.0, 1.0 - (2 * a_) ** 8))
            ring = []
            for b_, h_ in ((-0.5, -0.0012), (0.5, -0.0012), (0.5, lift * taper + 0.0003), (-0.5, 0.0003)):
                s = sc + a_ * L * d[0] + b_ * Wd * e[0]
                z = zc + a_ * L * d[1] + b_ * Wd * e[1]
                y, nrm = _cowl_y_at(s, z, "R")
                ring.append(np.array([s, -y, z]) + h_ * np.array([0.0, -nrm[0], nrm[1]]))
            rings.append(mb.add(np.array(ring)))
        mb.loft(rings, M["pacf"])
        mb.cap(rings[0], M["pacf"], start=True)
        mb.cap(rings[-1], M["pacf"], start=False)
        parts.append(mb.build(25.0))
    return merge("U_Cowl_Louvers", parts, 25.0)


def _tube_along(name: str, base: np.ndarray, direction: np.ndarray, length: float, ro: float, ri: float,
                back: float, mat: str, nseg: int = 32) -> MeshData:
    d = _unit(direction)
    e1 = _unit(np.cross(d, [0.0, 0.0, 1.0]) if abs(d[2]) < 0.9 else np.cross(d, [1.0, 0.0, 0.0]))
    e2 = np.cross(d, e1)
    prof = np.array([[-back, ri], [-back, ro], [length - 0.0015, ro], [length, ro - 0.0012], [length, ri + 0.0003],
                     [length - 0.002, ri]])
    loc = lathe(prof, nseg, name, mat, "x", closed_profile=True)
    V = base + np.outer(loc.verts[:, 0], d) + np.outer(loc.verts[:, 1], e1) + np.outer(loc.verts[:, 2], e2)
    md = MeshData(name, V, loc.faces, loc.face_mat, loc.mats, "spec", 30.0)
    return md.oriented()


def muffler_pipe() -> MeshData:
    """``U_Exhaust_Muffler``: sol yanaktan aşağı-dışa-geriye bakan susturucu çıkış borusu."""
    f = P.muffler_outlet()
    return _tube_along("U_Exhaust_Muffler", np.asarray(f.pos), np.asarray(f.direction), 0.020, 0.0062, 0.0045,
                       0.014, M["nozzle"])


def scuff_pad() -> MeshData:
    """``U_ScuffPad``: kaporta altında değiştirilebilir siyah TPU sürtünme pabucu (kuyruk çarpmasına)."""
    sp = _PR["cowl"]["scuff_pad"]
    sc, t = float(sp["s_m"]), float(sp["t_m"])
    th = np.linspace(0, 2 * math.pi, 40, endpoint=False)
    poly = np.column_stack([sc + 0.024 * np.cos(th), 0.014 * np.sin(th)])

    def zb(s, y):
        z = P.fuselage_z_at(s, y, "bottom")
        return (z if z is not None else P.fuselage_section(s).z_bottom) - t

    return heightfield_panel("U_ScuffPad", poly, zb, t + 0.0006, M["tpu"], M["tpu"], n_rows=10, col_step=0.003)


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


def spinner() -> MeshData:
    """``U_Spinner``: Ø64 alüminyum spinner (yerel, pervane göbeği orijinli; X ekseni geriye)."""
    pr = P.PROP
    c = math.cos(_rad(pr.downthrust_deg))
    xb = (pr.spinner_base_s - pr.hub[0]) / c
    xt = (pr.spinner_tip_s - pr.hub[0]) / c
    R = 0.5 * pr.spinner_d
    u = np.linspace(0, 1, 30)[1:-1]
    og = np.column_stack([xb + 0.0008 + (xt - xb - 0.0008) * u, R * (1 - u ** 1.9) ** 0.58])
    prof = np.vstack([[xb - 0.0025, 0.0], [xb - 0.0025, R + 0.0005], [xb - 0.0003, R + 0.0005], [xb + 0.0008, R],
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


def turret_mount() -> MeshData:
    """``U_Turret_Mount``: 8 fasetli antrasit yaka (yarı gömülü göz), üstte gövdeye doğru genişler; tabanı önde
    yukarı eğik (ileri görüş açık). Yerel orijin (s, 0, gövde altı), Blender eksenleri (+X ileri)."""
    t = P.TURRET
    zc, k = _collar_bottom()
    z_top = turret_mount_top_z() - t.belly_z + 0.012
    Rt, Rb = 0.5 * t.collar_d + TURRET_COLLAR_FLARE, 0.5 * t.collar_d
    rb = 0.5 * t.ball_d + TURRET_BORE_GAP
    ch = 0.0028
    prof = np.array([[z_top, Rt], [zc + ch, Rb], [zc, Rb - ch], [zc, rb], [z_top, rb]])
    fac = _octagon_factor(t.collar_facets)
    md = lathe(prof, 8 * t.collar_facets, "U_Turret_Mount", M["accent"], "z", closed_profile=True,
               radius_fn=lambda th, kk: fac(th, kk) if kk <= 2 else 1.0, smooth_angle=20.0)
    md.verts[:, 2] += k * md.verts[:, 0]
    return md


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
    """``U_Turret_Window``: EO (Ø26, iskele) ve IR (Ø20, sancak) pencereleri, turkuaz lens halkaları
    (yerel: top merkezi, pencere düzlemi +X'te)."""
    t = P.TURRET
    R = 0.5 * t.ball_d
    xf = R * TURRET_FACE_X + 0.0008
    parts = []
    for yc, d in ((0.5 * t.window_spacing, t.window_eo_d), (-0.5 * t.window_spacing, t.window_ir_d)):
        r = 0.5 * d
        ring = lathe(np.array([[xf - 0.0004, r], [xf - 0.0004, r + 0.0019], [xf + 0.0010, r + 0.0019],
                               [xf + 0.0014, r + 0.0010], [xf + 0.0014, r]]), 56, "ring", M["stripe"], "x",
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


WELL_ROOF_SKIN = 0.0025                  # kuyu tavanı ile kanat üst derisi arasında en az kalan et. varsayım


def well_roof_z(leg: str) -> Callable[[float, float], float]:
    """Kuyu tavanı ``z(s, y)``: spec ``roof_z``; ana kuyuda gövde dışında kanat üst yüzeyinin en az
    ``WELL_ROOF_SKIN`` altında kalacak şekilde sınırlanır (ince firar kenarı bölgesinde kanat delinmez)."""
    w = _well(leg)
    if leg.upper() == "N":
        return lambda s, y: w.roof_z
    side = "L" if leg.upper() == "L" else "R"
    tab = wing_table(side)

    def f(s, y):
        if abs(y) < P.fuselage_half_width_at(s, w.roof_z) - 0.004:
            return w.roof_z
        zu = tab.z(s, abs(y), "upper")
        return w.roof_z if not np.isfinite(zu) else min(w.roof_z, zu - WELL_ROOF_SKIN)
    return f


WELL_LIP = DOOR_T + 0.0015               # dişli deri dudağı: kapak eti + 1,5 mm (kaporta yanağı eğiminde pay). varsayım


def gear_well_outline(leg: str) -> np.ndarray:
    """Kuyu duvarlarının plan çokgeni ``(s, y)``: kapak açıklığının dişsiz dış zarfı (params ``GearWell.outline``;
    ana kuyuda tekerlek kuyusu dikdörtgeni + bacak yuvası). Testere dişleri yalnız deri dudağındadır."""
    return np.asarray(_well(leg).outline, float)


def _lip_z(leg: str) -> Callable[[float, float], float]:
    zf = _skin_z_for_leg(leg)
    return lambda s, y: zf(s, y) + WELL_LIP


def gear_cutters(leg: str) -> list[MeshData]:
    """Kuyu için iki boolean (EXACT) kesicisi, bu sırayla uygulanır: (1) dişli açıklık — kapak dişleriyle aynı
    çokgen, alttan deri yüzeyinin ``WELL_LIP`` (2 mm) üstüne kadar: testere dişi yalnız deri/kapak kalınlığında
    kalır (gerçek uçaktaki gibi); (2) kuyu hacmi — dişsiz zarf (``gear_well_outline``), dudağın üstünden tavana
    (``well_roof_z``) düz duvarlar. Toplu teker diş uçlarına değmez. Kesilen yüzler turuncu olur."""
    roof = well_roof_z(leg)
    lip = _lip_z(leg)
    L = leg.upper()
    opening = heightfield_panel(f"U_Cutter_{L}_Lip", gear_opening(leg), lambda s, y: -0.30, 0.0, M["bay"], M["bay"],
                                n_rows=6, col_step=0.0015, z_in=lambda s, y: lip(s, y) + 0.0003, smooth_angle=None)
    well = heightfield_panel(f"U_Cutter_{L}_Well", gear_well_outline(leg), lip, 0.0, M["bay"], M["bay"], n_rows=12,
                             col_step=0.0015, z_in=roof, smooth_angle=None)
    return [opening, well]


def gear_cutter(leg: str) -> MeshData:
    """Geriye uyumluluk: kuyu hacmi kesicisi (``gear_cutters(leg)[1]``). Sahne kurucusu ``gear_cutters`` kullanır."""
    return gear_cutters(leg)[1]


BAY_INSET, BAY_WALL = 0.0004, 0.0008


def gear_bay(leg: str) -> MeshData:
    """``U_Bay_<leg>``: turuncu kuyu astarı — dişsiz kuyu zarfını (``gear_well_outline``) 0,4 mm içeriden izleyen
    0,8 mm duvar halkası (alt kenarı deri dudağının üstüne oturur) ve tavana oturan plaka (iki kapalı kabuk tek
    nesnede)."""
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
    walls = mb.build(40.0)
    plate = heightfield_panel("plate", poly_offset(poly, BAY_INSET), lambda s, y: roof(s, y) - BAY_INSET - BAY_WALL,
                              BAY_WALL, M["bay"], M["bay"], n_rows=12, col_step=0.004)
    return merge(w.obj_name, [walls, plate], 40.0)


@dataclass
class DoorGeom:
    """Kapak ağı + menteşe tanımı. ``origin`` spec ``(s, y, z)``; ``frame_b`` 3×3 (sütunlar X, Y, Z; Blender):
    X = menteşe (+ dönüş açar) ya da ``strut`` kapakta takım toplama ekseni."""

    mesh: MeshData
    origin: tuple[float, float, float]
    frame_b: np.ndarray
    attach: str
    open_deg: float


def gear_door(name: str) -> DoorGeom:
    """``U_Door_N_1/2`` (burun, iki yandan), ``U_Door_L/R_1`` (tekerlek kuyusu, içteki kenardan menteşeli),
    ``U_Door_L/R_2`` (bacak yuvası kapağı, bacağa bağlı; toplu konumda modellenir, orijin takım pivotunda).
    Dış yüz deriyle aynı (alt boya), iç yüz ve kenarlar turuncu, et 1,2 mm, çevrede 0,5 mm panel aralığı."""
    d = next(x for x in P.gear_doors() if x.name == name)
    zf = _skin_z_for_leg(d.leg)
    poly = np.asarray(d.outline)
    md = heightfield_panel(name, poly, zf, DOOR_T, M["skin_bottom"], M["door_inner"], inset=DOOR_GAP, n_rows=16,
                           col_step=0.002)
    if d.attach == "skin":
        o = tuple(0.5 * (a + b) for a, b in zip(d.hinge_p0, d.hinge_p1))
        fr = _frame_from_axes(d.axis_open_b, [0.0, 0.0, 1.0])
    else:
        g = P.gear_leg(d.leg)
        o = g.pivot
        fr = _frame_from_axes(g.retract_axis_b, [0.0, 0.0, 1.0])
    return DoorGeom(md, o, fr, d.attach, d.open_deg)


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


# =====================================================================================================
# Rapor (CLI)
# =====================================================================================================
def all_static() -> list[MeshData]:
    """Bütün spec-uzayı statik ağlar (rapor/test için; Blender kurucusu parçaları tek tek çağırır)."""
    out = [fuselage(), cowl(), exhaust_ring(), intake(), hatch(), cowl_louvers(), muffler_pipe(), scuff_pad(),
           stab_fillet(), chevron()]
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
