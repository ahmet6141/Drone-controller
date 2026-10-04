"""YELKOVAN YK-38 — parametreler ve türetilmiş geometri (saf Python + numpy; bpy GEREKTİRMEZ).

``ucav/spec.yaml`` tek doğruluk kaynağıdır; bu modül onu okur, tipli veri sınıflarına çevirir ve bütün
modüllerin ihtiyaç duyduğu geometriyi türetir. Diğer modüller (Blender gövdesi, takım, baskı, malzeme,
raporlar) sayıları YALNIZCA buradan alır; sihirli sayı tekrarlanmaz.

Eksen takımları
---------------
* Spec / gövde ekseni ``(s, y, z)``: ``s`` burun ucundan geriye (m), ``y`` iskele (sol) +, ``z`` FRL'den yukarı.
  Fonksiyonlar sol (iskele, ``y > 0``) yarı için tanımlıdır; ``side="R"`` ya da negatif ``y`` sancak aynasını verir.
  DİKKAT: ``(s, y, z)`` sol-el takımıdır (s geriye). Eksen/dönüş vektörleri bu yüzden yalnız Blender
  takımında verilir (``*_b`` adlı alanlar) — sağ el kuralı orada geçerlidir.
* Blender ``(X, Y, Z)``: ``X = −s`` (ileri +), ``Y = y`` (sol +), ``Z = z``; metre. ``to_blender`` çevirir.

Ana API (diğer ajanlar için)
---------------------------
Koordinat: ``to_blender(s, y, z)``, ``points_to_blender(P)``, ``vec_to_blender(v)``, ``from_blender(X, Y, Z)``.
Gövde: ``fuselage_section(s)``, ``fuselage_outline(s, n_quarter)``, ``fuselage_half_width_at(s, z)``,
``fuselage_z_at(s, y, side)``, ``fuselage_stations()``, ``fuselage_s_stations(n)``, ``longeron_path(kind, side, n)``.
Kanat: ``wing_station(y)``, ``wing_airfoil(y, n)``, ``wing_section(y, n)``, ``wing_surface_z(s, y, side)``,
``wing_mid_z(s, y)``, ``wing_thickness_at(s, y)``, ``wing_planform()``, ``wing_reference()``, ``wing_breakpoints()``,
``wing_span_stations(n)``, ``wing_parts()``, ``glove_extension(y)``.
Kuyruk: ``stab_station(y)``, ``stab_section(y, n)``, ``fin_station(h, side)``, ``fin_section(h, side, n)``.
Kumanda yüzeyleri: ``hinge_line(name, side)``, ``hinge_lines()`` → ``HingeLine`` (menteşe, eksen, sınırlar).
Takım: ``gear_leg(name)``, ``gear_legs()``, ``gear_phase(gear)``, ``gear_wells()``, ``gear_doors()``.
İtki/faydalı yük/ayrıntı: ``PROP``, ``prop_spec()``, ``TURRET``, ``turret_spec()``, ``lights()``, ``antennas()``,
``pitot()``, ``hatch_outline()``, ``intake_spec()``, ``muffler_outlet()``, ``thrust_line_z(s)``, ``spar_tubes(side)``.
Baskı: ``print_cuts()`` (segment sınırları), ``SPEC["print"]`` (et, malzeme, tabla).
Sabitler: ``CG``, ``U_ROOT_B``, ``GROUND_Z``, ``MATERIALS`` (``UM_*``), ``MATERIAL_ROLES``.
Ham değer: ``spec("wing.dihedral_deg")``; tüm sözlük: ``SPEC``.

Kullanım: ``python3 ucav/params.py`` önemli türetilmiş değerleri yazdırır.
"""
from __future__ import annotations

import functools
import math
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Iterable

import numpy as np
import yaml

try:                                    # paket olarak (from ucav import params) ya da ucav/ dizininden
    from . import airfoils as AF
except ImportError:                     # pragma: no cover
    import airfoils as AF  # type: ignore[no-redef]

HERE = Path(__file__).resolve().parent
SPEC_PATH = HERE / "spec.yaml"
OUT_DIR = HERE / "out"
N_AIRFOIL = AF.DEFAULT_N                # kesit başına 2·N − 1 nokta


# =====================================================================================================
# Spec okuma
# =====================================================================================================
def load_spec(path: str | Path | None = None) -> dict:
    """``spec.yaml``'ı okur (varsayılan ``ucav/spec.yaml``) ve ham sözlüğü döndürür."""
    with open(path or SPEC_PATH, encoding="utf-8") as f:
        return yaml.safe_load(f)


SPEC: dict = load_spec()


def spec(path: str, default: Any = ...) -> Any:
    """Noktalı yol ile ham spec değeri: ``spec("wing.dihedral_deg")`` → 4.0. Yoksa ``default`` ya da KeyError."""
    node: Any = SPEC
    for key in path.split("."):
        if isinstance(node, dict) and key in node:
            node = node[key]
        elif isinstance(node, list) and key.isdigit():
            node = node[int(key)]
        else:
            if default is ...:
                raise KeyError(f"spec.yaml'da yok: {path}")
            return default
    return node


_W, _T, _F, _G, _P = SPEC["wing"], SPEC["tail"], SPEC["fuselage"], SPEC["landing_gear"], SPEC["propulsion"]
_rad, _deg = math.radians, math.degrees


# =====================================================================================================
# Koordinat dönüşümleri
# =====================================================================================================
def to_blender(s: float, y: float = 0.0, z: float = 0.0) -> tuple[float, float, float]:
    """Gövde istasyonu → Blender dünya koordinatı: ``(X, Y, Z) = (−s, y, z)`` (metre)."""
    return (-float(s), float(y), float(z))


def from_blender(X: float, Y: float = 0.0, Z: float = 0.0) -> tuple[float, float, float]:
    """Blender → gövde: ``(s, y, z) = (−X, Y, Z)``."""
    return (-float(X), float(Y), float(Z))


def points_to_blender(points: np.ndarray) -> np.ndarray:
    """(…, 3) ``(s, y, z)`` dizisini Blender ``(X, Y, Z)`` dizisine çevirir (kopya)."""
    p = np.array(points, dtype=float, copy=True)
    p[..., 0] *= -1.0
    return p


def vec_to_blender(v: Iterable[float]) -> np.ndarray:
    """Spec takımındaki bir yön vektörünü Blender takımına çevirir (``−ds, dy, dz``)."""
    a = np.asarray(list(v), dtype=float)
    return np.array([-a[0], a[1], a[2]])


def _unit(v: Iterable[float]) -> np.ndarray:
    a = np.asarray(list(v), dtype=float)
    n = float(np.linalg.norm(a))
    return a / n if n > 0 else a


def _side_sign(side: str | None = None, y: float | None = None) -> float:
    if side is not None:
        if side.upper() not in ("L", "R"):
            raise ValueError("side 'L' (iskele) ya da 'R' (sancak) olmalı")
        return 1.0 if side.upper() == "L" else -1.0
    return -1.0 if (y is not None and y < 0) else 1.0


def rotate_about_axis(points: np.ndarray, origin: Iterable[float], axis: Iterable[float], angle_deg: float) -> np.ndarray:
    """Rodrigues dönüşü (Blender takımında, sağ el kuralı): noktaları ``origin``'den geçen ``axis`` etrafında döndürür."""
    k = _unit(axis)
    o = np.asarray(list(origin), float)
    p = np.atleast_2d(np.asarray(points, float)) - o
    a = _rad(angle_deg)
    rot = p * math.cos(a) + np.cross(k, p) * math.sin(a) + np.outer(p @ k, k) * (1 - math.cos(a))
    out = rot + o
    return out if np.ndim(points) > 1 else out[0]


def _axis_sign_for(axis_b: np.ndarray, origin_b: np.ndarray, point_b: np.ndarray, want_b: np.ndarray) -> np.ndarray:
    """``axis_b`` yönünü, +küçük dönüş ``point_b``'yi ``want_b`` yönünde hareket ettirecek şekilde seçer."""
    moved = rotate_about_axis(point_b, origin_b, axis_b, 1.0)
    return axis_b if float(np.dot(moved - point_b, want_b)) > 0 else -axis_b


# =====================================================================================================
# PCHIP (monoton kübik Hermite) — numpy ile, scipy gerektirmez
# =====================================================================================================
def pchip_slopes(x: np.ndarray, y: np.ndarray, d0: float | None = None, dn: float | None = None) -> np.ndarray:
    """Fritsch–Carlson/Butland eğimleri (SciPy ``PchipInterpolator`` ile aynı kurallar). Uç eğimleri verilebilir."""
    x = np.asarray(x, float)
    y = np.asarray(y, float)
    h = np.diff(x)
    delta = np.diff(y) / h
    n = len(x)
    d = np.zeros(n)
    if n == 2:
        d[:] = delta[0]
    else:
        for k in range(1, n - 1):
            if delta[k - 1] * delta[k] > 0:
                w1, w2 = 2 * h[k] + h[k - 1], h[k] + 2 * h[k - 1]
                d[k] = (w1 + w2) / (w1 / delta[k - 1] + w2 / delta[k])

        def edge(h0, h1, m0, m1):
            e = ((2 * h0 + h1) * m0 - h0 * m1) / (h0 + h1)
            if np.sign(e) != np.sign(m0):
                return 0.0
            if np.sign(m0) != np.sign(m1) and abs(e) > abs(3 * m0):
                return 3 * m0
            return e

        d[0] = edge(h[0], h[1], delta[0], delta[1])
        d[-1] = edge(h[-1], h[-2], delta[-1], delta[-2])
    if d0 is not None:
        d[0] = d0
    if dn is not None:
        d[-1] = dn
    return d


def pchip(x: np.ndarray, y: np.ndarray, xq, d0: float | None = None, dn: float | None = None):
    """Monoton kübik Hermite enterpolasyonu (tek seferlik); ``xq`` aralık dışında uç değerlere kenetlenir."""
    return Pchip(x, y, d0, dn)(xq)


class Pchip:
    """Önceden hesaplanmış eğimlerle PCHIP enterpolanı: ``f = Pchip(x, y); f(xq)``."""

    def __init__(self, x, y, d0: float | None = None, dn: float | None = None):
        self.x = np.asarray(x, float)
        self.y = np.asarray(y, float)
        self.d = pchip_slopes(self.x, self.y, d0, dn)

    def __call__(self, xq):
        x, y, d = self.x, self.y, self.d
        xq_arr = np.clip(np.asarray(xq, float), x[0], x[-1])
        i = np.clip(np.searchsorted(x, xq_arr) - 1, 0, len(x) - 2)
        h = x[i + 1] - x[i]
        t = (xq_arr - x[i]) / h
        out = ((2 * t ** 3 - 3 * t ** 2 + 1) * y[i] + (t ** 3 - 2 * t ** 2 + t) * h * d[i]
               + (-2 * t ** 3 + 3 * t ** 2) * y[i + 1] + (t ** 3 - t ** 2) * h * d[i + 1])
        return float(out) if np.ndim(xq) == 0 else out


def smoothstep(t):
    """0 → 1 yumuşak basamak (3t² − 2t³), [0, 1]'e kenetli."""
    t = np.clip(t, 0.0, 1.0)
    return t * t * (3 - 2 * t)


# =====================================================================================================
# Kütle merkezi, kök nesne, zemin
# =====================================================================================================
@dataclass(frozen=True)
class CGSpec:
    """Tasarım ağırlık merkezi ve aralığı (``U_Root`` burada)."""

    s: float
    z: float
    s_range: tuple[float, float]
    mtow_kg: float

    def blender(self) -> tuple[float, float, float]:
        """``U_Root`` Blender konumu ``(−s, 0, z)``."""
        return to_blender(self.s, 0.0, self.z)


CG = CGSpec(s=float(SPEC["stability"]["cg_s_m"]), z=float(SPEC["stability"]["cg_z_m"]),
            s_range=tuple(SPEC["stability"]["cg_range_s_m"]), mtow_kg=float(SPEC["mass"]["mtow_kg"]))
U_ROOT_B = CG.blender()                         # (−1,232, 0, 0,006)
GROUND_Z = float(_G["ground_z_m"])              # takım açık, statik


# =====================================================================================================
# Gövde
# =====================================================================================================
@dataclass(frozen=True)
class FuselageSection:
    """Bir ``s`` istasyonunda gövde kesiti (iki süperelips yarısı chine'da birleşir).

    ``width``/``height`` kesitin tam genişliği/yüksekliği, ``z_center`` kesit merkezi, ``z_chine`` chine
    (en geniş yer) yüksekliği. ``crease`` 0…1 chine keskinliği; ``lean_*`` chine'daki yanak teğetlerinin
    içe yatma katsayısı (açı ≈ atan(lean·crease·a/b)).
    """

    s: float
    width: float
    height: float
    z_center: float
    n_upper: float
    n_lower: float
    z_chine: float
    crease: float
    lean_upper: float
    lean_lower: float

    @property
    def half_width(self) -> float:
        return 0.5 * self.width

    @property
    def z_top(self) -> float:
        return self.z_center + 0.5 * self.height

    @property
    def z_bottom(self) -> float:
        return self.z_center - 0.5 * self.height

    @property
    def b_upper(self) -> float:
        """Chine'dan tepeye yükseklik."""
        return self.z_top - self.z_chine

    @property
    def b_lower(self) -> float:
        """Chine'dan karına derinlik."""
        return self.z_chine - self.z_bottom


_FS = np.asarray(_F["stations"], float)
_FS_S, _FS_W, _FS_H, _FS_ZC = _FS[:, 0], _FS[:, 1], _FS[:, 2], _FS[:, 3]
_FS_NU = np.asarray(_F["n_upper"], float)
_FS_NL = np.asarray(_F["n_lower"], float)
_FS_CR = np.asarray(_F["chine_crease"], float)
FUSELAGE_LENGTH = float(_F["length_m"])
_U = np.sqrt(_FS_S)                                       # burunda √s uzayı (küt uç)
_D0_W = 2.0 * math.sqrt(2.0 * float(_F["nose"]["tip_radius_plan_m"]))
_D0_H = 2.0 * math.sqrt(2.0 * float(_F["nose"]["tip_radius_profile_m"]))
_IP_W, _IP_H = Pchip(_U, _FS_W, d0=_D0_W), Pchip(_U, _FS_H, d0=_D0_H)
_IP_ZC, _IP_NU, _IP_NL, _IP_CR = (Pchip(_FS_S, v) for v in (_FS_ZC, _FS_NU, _FS_NL, _FS_CR))


def fuselage_stations() -> np.ndarray:
    """Spec istasyon tablosu, (N, 4): ``[s, w, h, z_c]``."""
    return _FS.copy()


def fuselage_section(s: float) -> FuselageSection:
    """``s`` istasyonunda gövde kesiti (PCHIP; w ve h burunda √s uzayında → uç yarıçapları korunur).

    ``s`` [0, L] aralığına kenetlenir. Burun ucu ``z_center(0) = −0,014`` (sarkma), kuyruk kalkışı istasyon
    tablosundandır.
    """
    s = float(np.clip(s, 0.0, FUSELAGE_LENGTH))
    u = math.sqrt(s)
    w = max(0.0, _IP_W(u))
    h = max(0.0, _IP_H(u))
    zc, nu, nl = _IP_ZC(s), _IP_NU(s), _IP_NL(s)
    cr = float(np.clip(_IP_CR(s), 0.0, 1.0))
    ch = _F["chine"]
    z_ch = zc + float(ch["z_offset_h"]) * h
    return FuselageSection(s=s, width=w, height=h, z_center=zc, n_upper=nu, n_lower=nl, z_chine=z_ch, crease=cr,
                           lean_upper=float(ch["lean_upper"]), lean_lower=float(ch["lean_lower"]))


def _lean_factor(t, lean: float):
    """Chine kırığı: yanak genişliği çarpanı ``1 − lean·t·(1 − t)²`` (t = chine'dan uzaklık / b).

    Teğet chine'da ``atan(lean·a/b)`` kadar içe yatar; etki chine'a yakın yoğunlaşır, hacim dolgun kalır
    (en büyük daralma t = 1/3'te %14,8·lean)."""
    t = np.clip(t, 0.0, 1.0)
    return 1.0 - lean * t * (1.0 - t) ** 2


def _quarter(a: float, b: float, n: float, lean: float, count: int, upper: bool) -> np.ndarray:
    """Süperelips çeyreği (y ≥ 0): chine'dan (θ=0) tepeye/karına (θ=π/2); yay boyuna göre ``count`` nokta."""
    th = np.linspace(0.0, math.pi / 2, 600)
    zq = b * np.sin(th) ** (2.0 / n)
    yq = a * np.cos(th) ** (2.0 / n) * _lean_factor(zq / max(b, 1e-12), lean)
    yq = np.maximum(yq, 0.0)
    seg = np.r_[0.0, np.cumsum(np.hypot(np.diff(yq), np.diff(zq)))]
    if seg[-1] <= 0:
        return np.zeros((count, 2))
    t = np.linspace(0.0, seg[-1], count)
    y = np.interp(t, seg, yq)
    zz = np.interp(t, seg, zq)
    return np.column_stack([y, zz if upper else -zz])


def fuselage_outline(s: float, n_quarter: int = 24) -> np.ndarray:
    """Gövde kesitinin kapalı dış çizgisi, ``(4·nq − 4, 2)`` dizisi ``(y, z)``.

    Sıra ve indisler bütün istasyonlarda aynıdır (loft için): 0 = tepe (y = 0) → ``nq − 1`` = iskele chine →
    ``2nq − 2`` = karın (y = 0) → ``3nq − 3`` = sancak chine → tepeye döner (son nokta tekrar edilmez).
    Noktalar her çeyrekte yay boyuna eşit dağılır.
    """
    sec = fuselage_section(s)
    nq = int(n_quarter)
    if sec.width <= 1e-9 or sec.height <= 1e-9:
        return np.tile([0.0, sec.z_center], (4 * nq - 4, 1))
    a = sec.half_width
    up = _quarter(a, sec.b_upper, sec.n_upper, sec.lean_upper * sec.crease, nq, True)      # chine → tepe
    lo = _quarter(a, sec.b_lower, sec.n_lower, sec.lean_lower * sec.crease, nq, False)     # chine → karın
    up[:, 1] += sec.z_chine
    lo[:, 1] += sec.z_chine
    up[-1, 0] = lo[-1, 0] = 0.0
    port = np.vstack([up[::-1], lo[1:]])                    # tepe → chine → karın (2nq − 1)
    stbd = port[-2:0:-1].copy()                             # karından tepeye (uçlar hariç)
    stbd[:, 0] *= -1.0
    return np.vstack([port, stbd])


def fuselage_half_width_at(s: float | FuselageSection, z: float) -> float:
    """``s`` istasyonunda (ya da verilen kesitte), ``z`` yüksekliğinde gövde yarı genişliği (kesit dışındaysa 0)."""
    sec = s if isinstance(s, FuselageSection) else fuselage_section(s)
    if sec.width <= 0 or z >= sec.z_top or z <= sec.z_bottom:
        return 0.0
    a = sec.half_width
    if z >= sec.z_chine:
        b, n, lean = sec.b_upper, sec.n_upper, sec.lean_upper * sec.crease
        zp = z - sec.z_chine
    else:
        b, n, lean = sec.b_lower, sec.n_lower, sec.lean_lower * sec.crease
        zp = sec.z_chine - z
    r = max(0.0, 1.0 - (zp / b) ** n)
    return float(a * r ** (1.0 / n) * _lean_factor(zp / b, lean))


def fuselage_z_at(s: float, y: float, side: str = "bottom") -> float | None:
    """``(s, y)`` noktasında gövde alt (``"bottom"``) ya da üst (``"top"``) yüzey yüksekliği; |y| dışarıdaysa None."""
    sec = fuselage_section(s)
    ay = abs(y)
    if ay > sec.half_width or sec.width <= 0:
        return None
    lo, hi = sec.z_chine, (sec.z_top if side == "top" else sec.z_bottom)   # yarı genişlik lo'da en büyük, hi'de 0
    for _ in range(60):                                                    # ikiye bölme
        mid = 0.5 * (lo + hi)
        if fuselage_half_width_at(sec, mid) > ay:
            lo = mid
        else:
            hi = mid
    return float(0.5 * (lo + hi))


def _longeron_curve_point(kind: str, s: float) -> tuple[float, float]:
    """Gövdeyi izleyen (eğri) longeron konumu ``(|y|, z)``: chine köşesinden ``inset`` içeride ya da omuz."""
    L = _F["longerons"]
    sec = fuselage_section(float(s))
    if kind == "chine":
        return sec.half_width - float(L["chine"]["inset_m"]), sec.z_chine
    if kind == "shoulder":
        return (float(L["shoulder"]["y_frac_of_width"]) * sec.width,
                sec.z_center + float(L["shoulder"]["z_frac_of_height"]) * sec.height)
    raise ValueError("kind 'chine' ya da 'shoulder' olmalı")


def longeron_pieces(kind: str = "chine", side: str = "L") -> list[tuple[np.ndarray, np.ndarray]]:
    """Longeron DÜZ parçaları ``[(p0, p1)]`` (spec takımı): kırık istasyonları (``longerons.breaks_s_m``, halka
    sınırları) arasında gövde eğrisinin kirişi. Kesit dışbükey olduğundan kiriş gövdenin içinde kalır; parça uçları
    açılı basılı PETG soket bloklarına girer (P3: 1,66 m boru kıvrık kanallardan geçirilemez)."""
    return [(np.array([p0[0], _side_sign(side) * p0[1], p0[2]]), np.array([p1[0], _side_sign(side) * p1[1], p1[2]]))
            for p0, p1 in _longeron_pieces_l(kind)]


@functools.lru_cache(maxsize=None)
def _longeron_pieces_l(kind: str) -> tuple:
    """Sol longeron parçaları: kırık noktaları önce eğri üzerinde; sonra her parçanın gövde yanına en dar yanal
    payı chine kenar payının (``inset``) altına düşerse parça uçları içe kaydırılır (kirişin sehim payı; komşu
    parçalar ortak kırık noktasını paylaşır → kırıkta süreklilik)."""
    L = _F["longerons"]
    br = [float(L["s_from_m"])] + [float(b) for b in L.get("breaks_s_m", [])] + [float(L["s_to_m"])]
    P_ = [np.array([s, *_longeron_curve_point(kind, s)]) for s in br]
    target = float(L["chine"]["inset_m"]) if kind == "chine" else 0.010
    for _ in range(8):
        shift = np.zeros(len(P_))
        for k in range(len(P_) - 1):
            worst = 1.0
            for f in np.linspace(0.0, 1.0, 33):
                p = P_[k] + f * (P_[k + 1] - P_[k])
                worst = min(worst, fuselage_half_width_at(float(p[0]), float(p[2])) - p[1])
            d = max(0.0, target - worst)
            shift[k] = max(shift[k], d)
            shift[k + 1] = max(shift[k + 1], d)
        if shift.max() < 1e-5:
            break
        for k in range(len(P_)):
            P_[k] = P_[k] - np.array([0.0, shift[k] + (2e-4 if shift[k] > 0 else 0.0), 0.0])
    return tuple((P_[k], P_[k + 1]) for k in range(len(P_) - 1))


def longeron_path(kind: str = "chine", side: str = "L", n: int = 40) -> np.ndarray:
    """CF 8/6 longeron ekseni, ``(n, 3)`` ``(s, y, z)``: ``kind`` = ``"chine"`` ya da ``"shoulder"``.

    Chine longeronu kabuğa gömülüdür (chine köşesinden ``inset`` kadar içeride); omuz longeronu
    ``y = ±0,30·w``, ``z = z_c + 0,35·h``. Boru üç DÜZ parçadır (``longeron_pieces``; kırıklar halka sınırlarında,
    s 1,167 ve 1,55): her gövde halkasının longeron kanalı tek bir parçanın üzerinde ve eş eksenlidir. Örnekler kırık
    istasyonlarını içerir (doğrusal enterpolasyon yolu tam olarak verir).
    """
    L = _F["longerons"]
    pieces = longeron_pieces(kind, side)
    ss = np.unique(np.r_[np.linspace(float(L["s_from_m"]), float(L["s_to_m"]), n), [p[0][0] for p in pieces[1:]]])
    pts = []
    for s in ss:
        for p0, p1 in pieces:
            if p0[0] - 1e-12 <= s <= p1[0] + 1e-12:
                f = (s - p0[0]) / (p1[0] - p0[0])
                pts.append(tuple(p0 + f * (p1 - p0)))
                break
    return np.asarray(pts)


# =====================================================================================================
# Kanat
# =====================================================================================================
WING_SEMI_SPAN = float(_W["semi_span_m"])
WING_ROOT_CHORD = float(_W["root_chord_m"])
WING_TIP_CHORD = float(_W["tip_chord_m"])
WING_Y_KINK = float(_W["centre_section_y_m"])
WING_SPAR_FRAC = float(_W["spar_chord_fraction"])
WING_SPAR_S = float(_W["le_root_s_m"]) + WING_SPAR_FRAC * WING_ROOT_CHORD   # 1,2422
WING_DIHEDRAL = float(_W["dihedral_deg"])
WING_Z_ROOT = float(_W["root_chord_plane_z_m"])
WING_INCIDENCE = float(_W["incidence_deg"])


@dataclass(frozen=True)
class WingStation:
    """Kanadın ``y`` istasyonundaki yerel tanımı (kesitler sabit-y düzlemlerindedir, dihedral z kaydırmasıdır).

    * ``chord_ref`` — referans trapez veteri (alan/MAC tanımı).
    * ``chord`` — profilin ölçeklendiği veter (raked uçta HK geride → küçülür; glove uzaması HARİÇ).
    * ``le_s``/``te_s`` — gerçek hücum/firar kenarı (glove ve raked uç dahil), ``spar_s`` ana kiriş (%28, sabit).
    * ``z_ref`` — kiriş noktasında veter düzlemi yüksekliği (dihedral). ``incidence_deg`` yerel açı (kök + burulma).
    * ``blend`` 0 = kök profili (SD7062), 1 = uç profili (SD7032). ``thickness_scale`` kalınlık çarpanı (kök
      kalınlaşması × uç kapanışı). ``t_c`` yerel en büyük kalınlık oranı (glove'suz). ``glove_ext`` HK uzaması (m).
    """

    y: float
    chord_ref: float
    chord: float
    le_s: float
    te_s: float
    spar_s: float
    z_ref: float
    incidence_deg: float
    twist_deg: float
    blend: float
    thickness_scale: float
    t_c: float
    glove_ext: float

    @property
    def side(self) -> str:
        return "R" if self.y < 0 else "L"


def wing_chord_ref(y) -> float | np.ndarray:
    """Referans trapez veteri: |y| ≤ 1,00 → 0,29; 1,00 → 1,90 doğrusal → 0,15 (|y| > 1,90'da 0,15'e kenetli)."""
    ay = np.clip(np.abs(np.asarray(y, float)), 0.0, WING_SEMI_SPAN)
    c = np.where(ay <= WING_Y_KINK, WING_ROOT_CHORD,
                 WING_ROOT_CHORD - (WING_ROOT_CHORD - WING_TIP_CHORD) * (ay - WING_Y_KINK) / (WING_SEMI_SPAN - WING_Y_KINK))
    return float(c) if np.ndim(y) == 0 else c


def wing_le_ref(y) -> float | np.ndarray:
    """Referans trapez hücum kenarı ``s`` (oksuz %28 kiriş çizgisinden)."""
    return WING_SPAR_S - WING_SPAR_FRAC * wing_chord_ref(y)


def wing_te_s(y) -> float | np.ndarray:
    """Firar kenarı ``s`` (raked uçta da düz devam eder)."""
    return WING_SPAR_S + (1.0 - WING_SPAR_FRAC) * wing_chord_ref(y)


def wing_twist(y) -> float:
    """Geometrik burulma (derece, washout −): |y| ≤ 1,00'da 0, uca doğrusal −3°."""
    w = _W["washout"]
    ay = abs(float(y))
    t = np.clip((ay - float(w["y_from_m"])) / (float(w["y_to_m"]) - float(w["y_from_m"])), 0.0, 1.0)
    return -float(w["deg"]) * float(t)


def glove_extension(y) -> float:
    """Glove HK uzaması (m): |y| ≥ 0,240'ta 0; içeride 36° çizgisi (0,1025'te 0,10), ``inner_y``'den sonra sabit."""
    g = _W["glove"]
    ay = max(abs(float(y)), float(g["inner_y_m"]))
    return max(0.0, (float(g["end_y_m"]) - ay) * math.tan(_rad(float(g["le_sweep_deg"]))))


@functools.lru_cache(maxsize=None)
def airfoil_tmax(name: str) -> float:
    """Kütüphane profilinin en büyük kalınlık oranı (önbellekli)."""
    return AF.max_thickness(AF.section(name))[0]


def _root_thickness_factor(ay: float) -> float:
    rt = _W["airfoil"]["root_thickening"]
    t_base = airfoil_tmax(_W["airfoil"]["root"])
    k_max = float(rt["t_c"]) / t_base
    y0, y1 = float(rt["y_full_m"]), float(rt["y_end_m"])
    f = 1.0 - float(smoothstep((ay - y0) / (y1 - y0)))
    return 1.0 + (k_max - 1.0) * f


def _tip_closure(ay: float) -> float:
    r = float(_W["raked_tip"]["end_round_m"])
    y_end = WING_SEMI_SPAN
    if ay <= y_end - r:
        return 1.0
    q = min(1.0, (ay - (y_end - r)) / r)
    return math.sqrt(max(0.0, 1.0 - q * q))


def wing_station(y: float) -> WingStation:
    """``y`` istasyonunda kanat tanımı (bkz. ``WingStation``). |y| 0…1,90; işaret sol/sağ yarıyı seçer."""
    y = float(y)
    ay = min(abs(y), WING_SEMI_SPAN)
    c_ref = wing_chord_ref(ay)
    te = wing_te_s(ay)
    rt = _W["raked_tip"]
    y_rk = float(rt["y_from_m"])
    if ay > y_rk:
        le = wing_le_ref(y_rk) + (ay - y_rk) * math.tan(_rad(float(rt["le_sweep_deg"])))
    else:
        le = wing_le_ref(ay)
    chord = te - le
    gext = glove_extension(ay)
    a = _W["airfoil"]
    y0, y1 = a["blend_y_m"]
    blend = float(np.clip((ay - y0) / (y1 - y0), 0.0, 1.0))
    k = _root_thickness_factor(ay) * _tip_closure(ay)
    t_root, t_tip = airfoil_tmax(a["root"]), airfoil_tmax(a["tip"])
    t_c = ((1 - blend) * t_root + blend * t_tip) * k
    twist = wing_twist(ay)
    z_ref = WING_Z_ROOT + ay * math.tan(_rad(WING_DIHEDRAL))
    return WingStation(y=y, chord_ref=c_ref, chord=chord, le_s=le - gext, te_s=te, spar_s=WING_SPAR_S, z_ref=z_ref,
                       incidence_deg=WING_INCIDENCE + twist, twist_deg=twist, blend=blend, thickness_scale=k,
                       t_c=t_c, glove_ext=gext)


@functools.lru_cache(maxsize=4096)
def _wing_airfoil_cached(ay_key: float, n: int, te_m: float) -> np.ndarray:
    st = wing_station(ay_key)
    a = _W["airfoil"]
    base = AF.blend(AF.section(a["root"], n), AF.section(a["tip"], n), st.blend)
    base = AF.scale_thickness(base, _root_thickness_factor(ay_key))
    if te_m > 0:
        base = AF.blunt_te(base, te_m / st.chord)
    clo = _tip_closure(ay_key)
    if clo < 1.0:
        base = AF.scale_thickness(base, clo)
    return base


def wing_airfoil(y: float, n: int = N_AIRFOIL, te_thickness: float | None = None) -> np.ndarray:
    """Yerel 2B profil (veter birimi, glove hariç), (2n − 1, 2): karışım + kök kalınlaşması + TE + uç kapanışı.

    ``te_thickness`` mutlak firar kenarı kalınlığı (m); None → ``wing.te_thickness_m`` (1,5 mm), 0 → keskin.
    """
    te = float(_W["te_thickness_m"]) if te_thickness is None else float(te_thickness)
    return _wing_airfoil_cached(round(min(abs(float(y)), WING_SEMI_SPAN), 6), int(n), round(te, 7)).copy()


def wing_section(y: float, n: int = N_AIRFOIL, te_thickness: float | None = None) -> np.ndarray:
    """Kanat kesiti 3B noktalar ``(2n − 1, 3)`` ``(s, y, z)``, Selig sırası (TE üst → LE → TE alt).

    Aynı ``n`` ile bütün istasyonlarda nokta sayısı ve indis anlamı aynıdır (loft). Yerleştirme: profil
    ``chord`` ile ölçeklenir, firar kenarı ``te_s``'de sabit; glove bölgesinde profilin ön %30'u
    ``(1 − x/0,30)²`` ağırlığıyla ``glove_ext`` kadar öne uzatılır ve burun kalınlığı inceltilir (strake);
    sonra kesit ana kiriş noktası ``(spar_s, z_ref)`` etrafında yerel açıyla (burun yukarı +) döndürülür.
    """
    st = wing_station(y)
    af = wing_airfoil(y, n, te_thickness)
    x, yu, yl = AF.surfaces(af)
    g = _W["glove"]
    xb = float(g["blend_chord_fraction"])
    w = np.clip(1.0 - x / xb, 0.0, 1.0) ** 2
    g_root = glove_extension(float(g["root_y_m"]))
    strength = st.glove_ext / g_root if g_root > 0 else 0.0
    thin = 1.0 - (1.0 - float(g["nose_thickness_factor"])) * w * min(strength, 1.0)
    yc = 0.5 * (yu + yl)
    yu2 = yc + (yu - yc) * thin
    yl2 = yc + (yl - yc) * thin
    xc = x * st.chord - st.glove_ext * w                       # HK'den veter boyunca (m), glove uzaması dahil
    pts2 = AF.from_surfaces(xc, yu2 * st.chord, yl2 * st.chord)
    dx = (st.te_s - st.chord) + pts2[:, 0] - st.spar_s         # kiriş noktasına göre (veter doğrultusu, m)
    dz = pts2[:, 1]
    i = _rad(st.incidence_deg)
    s = st.spar_s + dx * math.cos(i) + dz * math.sin(i)
    z = st.z_ref - dx * math.sin(i) + dz * math.cos(i)
    return np.column_stack([s, np.full_like(s, st.y), z])


def wing_surface_z(s: float, y: float, side: str = "lower", n: int = N_AIRFOIL) -> float | None:
    """Kanadın ``(s, y)`` noktasındaki üst (``"upper"``) ya da alt (``"lower"``) yüzey yüksekliği; veter dışındaysa None."""
    sec = wing_section(y, n)
    m = (len(sec) + 1) // 2
    curve = sec[:m][::-1] if side == "upper" else sec[m - 1:]
    ss = np.maximum.accumulate(curve[:, 0])
    if s < ss[0] or s > ss[-1]:
        return None
    return float(np.interp(s, ss, curve[:, 2]))


def wing_mid_z(s: float, y: float, n: int = N_AIRFOIL) -> float | None:
    """Üst ve alt yüzeyin ortası (kalınlık ortası) — boru ve menteşe yerleşimi için."""
    u, l = wing_surface_z(s, y, "upper", n), wing_surface_z(s, y, "lower", n)
    return None if u is None or l is None else 0.5 * (u + l)


def wing_thickness_at(s: float, y: float, n: int = N_AIRFOIL) -> float | None:
    """``(s, y)`` noktasında dikey kanat kalınlığı (m)."""
    u, l = wing_surface_z(s, y, "upper", n), wing_surface_z(s, y, "lower", n)
    return None if u is None or l is None else u - l


def wing_planform(n: int = 400) -> dict[str, np.ndarray]:
    """Sol yarı planform çizgileri ``(s, y)``: ``le`` (glove + raked uç dahil), ``te``, ``le_ref``/``te_ref`` (trapez).

    y = 0 → 1,90. Üstten görünüş ve alan hesabı için.
    """
    yy = np.linspace(0.0, WING_SEMI_SPAN, n)
    yy = np.unique(np.r_[yy, [float(_W["glove"]["inner_y_m"]), float(_W["glove"]["root_y_m"]),
                              float(_W["glove"]["end_y_m"]), WING_Y_KINK, float(_W["raked_tip"]["y_from_m"])]])
    le = np.array([wing_station(v).le_s for v in yy])
    te = np.array([wing_station(v).te_s for v in yy])
    return {"y": yy, "le": le, "te": te, "le_ref": np.asarray(wing_le_ref(yy)), "te_ref": np.asarray(wing_te_s(yy))}


def _trapz(f, x):
    return float(np.trapezoid(f, x)) if hasattr(np, "trapezoid") else float(np.trapz(f, x))


@functools.lru_cache(maxsize=1)
def wing_reference() -> dict[str, float]:
    """Referans planform integralleri (iki yarı, gövde içi dahil; glove ve raked uç hariç).

    Dönüş: ``S``, ``AR``, ``MAC``, ``y_mac``, ``le_s_mac``, ``ac_s`` (= LE_MAC + MAC/4), ``taper``,
    ``sweep_le_deg``/``sweep_te_deg`` (dış panel), ``tip_z`` (uç veter düzlemi, kiriş hattında).
    """
    y = np.linspace(0.0, WING_SEMI_SPAN, 20001)
    c = np.asarray(wing_chord_ref(y))
    S = 2 * _trapz(c, y)
    mac = 2 / S * _trapz(c ** 2, y)
    ymac = 2 / S * _trapz(c * y, y)
    xc4 = 2 / S * _trapz(c * (np.asarray(wing_le_ref(y)) + 0.25 * c), y)
    le_mac = xc4 - 0.25 * mac
    dy = WING_SEMI_SPAN - WING_Y_KINK
    sweep_le = _deg(math.atan((wing_le_ref(WING_SEMI_SPAN) - wing_le_ref(WING_Y_KINK)) / dy))
    sweep_te = _deg(math.atan((wing_te_s(WING_SEMI_SPAN) - wing_te_s(WING_Y_KINK)) / dy))
    return {"S": S, "AR": (2 * WING_SEMI_SPAN) ** 2 / S, "MAC": mac, "y_mac": ymac, "le_s_mac": le_mac,
            "ac_s": le_mac + 0.25 * mac, "taper": WING_TIP_CHORD / WING_ROOT_CHORD, "spar_s": WING_SPAR_S,
            "sweep_le_deg": sweep_le, "sweep_te_deg": sweep_te,
            "tip_z": WING_Z_ROOT + WING_SEMI_SPAN * math.tan(_rad(WING_DIHEDRAL))}


def wing_breakpoints() -> list[float]:
    """Sol yarıda loft için ZORUNLU açıklık istasyonları (sıralı, tekrarsız): kök, glove iç/kök/uç, kök bloğu
    ekleri, kumanda yüzeyi uçları, merkez kırığı (1,00), raked uç başı (1,83), uç kapanışı başı ve uç (1,90)."""
    W, g, cs, rt = _W, _W["glove"], _W["control_surfaces"], _W["raked_tip"]
    ys = {0.0, float(g["inner_y_m"]), float(g["root_y_m"]), float(g["end_y_m"]), float(W["panel_joint_y_m"]),
          WING_Y_KINK, float(rt["y_from_m"]), WING_SEMI_SPAN - float(rt["end_round_m"]), WING_SEMI_SPAN}
    for k in ("aileron", "flap_in", "flap_out"):
        ys |= {float(cs[k]["y_from_m"]), float(cs[k]["y_to_m"])}
    return sorted(round(v, 6) for v in ys)


def wing_parts() -> dict[str, tuple[float, float]]:
    """Kanat nesneleri ve açıklık aralıkları (sol; ``_L``/``_R`` eki eklenir): ``U_WingCenter`` kök/glove bloğu
    (y 0 → panel eki 0,36; gövde içinden geçer, glove dahil), ``U_WingOuter`` sökülebilir dış panel (0,36 → 1,83),
    ``U_Tip`` 70 mm raked uç kapağı (1,83 → 1,90)."""
    yj, yr = float(_W["panel_joint_y_m"]), float(_W["raked_tip"]["y_from_m"])
    return {"U_WingCenter": (0.0, yj), "U_WingOuter": (yj, yr), "U_Tip": (yr, WING_SEMI_SPAN)}


def wing_span_stations(n: int = 90) -> np.ndarray:
    """Loft için önerilen sol yarı açıklık istasyonları: ``wing_breakpoints`` + aralarda ≈ eşit, uçta sıklaşan noktalar."""
    bp = wing_breakpoints()
    base = np.linspace(0.0, WING_SEMI_SPAN, n)
    tip = WING_SEMI_SPAN - float(_W["raked_tip"]["end_round_m"]) * (1 - np.cos(np.linspace(0, math.pi / 2, 8)))
    return np.unique(np.round(np.r_[base, bp, tip], 6))


def fuselage_s_stations(n: int = 140) -> np.ndarray:
    """Gövde loftu için önerilen ``s`` istasyonları: burunda √ dağılımla sık (küt uç), spec istasyonları dahil."""
    u = np.linspace(0.0, 1.0, n)
    s = FUSELAGE_LENGTH * (0.35 * u ** 2 + 0.65 * u)          # burunda sık, düzgün artan
    s = np.r_[s, 0.06 * np.linspace(0, 1, 12) ** 2, _FS_S]
    return np.unique(np.round(s, 6))


def print_cuts() -> dict[str, list[float]]:
    """Baskı segment sınırları (spec ``print.segments``): kanat ``y`` (sol yarı), gövde ``s``, kumanda yüzeyleri ve
    kuyruk. Her liste, ardışık iki değer bir segment olacak şekilde sınır koordinatlarıdır (m)."""
    sg = SPEC["print"]["segments"]
    out: dict[str, list[float]] = {}
    rb, wp = sg["root_block"], sg["wing_outer_panel"]
    y0, yj = float(rb["y_from_m"]), float(_W["panel_joint_y_m"])
    out["root_block_y"] = list(np.linspace(y0, yj, int(rb["count"]) + 1))
    ywp = [float(wp["y_from_m"]) + k * float(wp["length_m"]) for k in range(int(wp["count"]) + 1)]
    out["wing_panel_y"] = ywp + [ywp[-1] + float(wp["tip_cap_m"])]
    fs = [0.0, float(sg["fuselage_nose_module"]["s_m"][1])]
    for ring in sg["fuselage_rings"]:
        a, b = ring["s_m"]
        fs += list(np.linspace(a, b, int(ring["count"]) + 1))[1:]
    out["fuselage_s"] = fs + [FUSELAGE_LENGTH]                 # son segment: PA-CF kaporta
    for key, cs, src in (("aileron", "aileron", _CS_W), ("flap_out", "flap_out", _CS_W), ("flap_in", "flap_in", _CS_W)):
        out[f"{key}_y"] = list(np.linspace(float(src[cs]["y_from_m"]), float(src[cs]["y_to_m"]), int(sg[key]["count"]) + 1))
    out["stab_y"] = list(np.linspace(0.0, STAB_HALF_SPAN, int(sg["stab_half"]["count"]) + 1))
    el = _CS_T["elevator"]
    out["elevator_y"] = list(np.linspace(float(el["y_from_m"]), float(el["y_to_m"]), int(sg["elevator"]["count"]) + 1))
    out["fin_h"] = list(np.linspace(0.0, float(_FN["height_m"]), int(sg["fin"]["count"]) + 1))
    return {k: [round(float(v), 6) for v in vals] for k, vals in out.items()}


# =====================================================================================================
# Kuyruk
# =====================================================================================================
_ST, _FN = _T["stab"], _T["fin"]
STAB_HALF_SPAN = 0.5 * float(_ST["span_m"])


@dataclass(frozen=True)
class StabStation:
    """Yatay stabilize istasyonu: ``chord``, ``le_s``, ``te_s``, ``z`` (veter düzlemi), ``incidence_deg``."""

    y: float
    chord: float
    le_s: float
    te_s: float
    z: float
    incidence_deg: float


def stab_station(y: float) -> StabStation:
    """Stabilize ``y`` istasyonu (|y| ≤ 0,52; 30° oklu HK, doğrusal koniklik)."""
    ay = min(abs(float(y)), STAB_HALF_SPAN)
    cr, ct = float(_ST["root_chord_m"]), float(_ST["tip_chord_m"])
    c = cr - (cr - ct) * ay / STAB_HALF_SPAN
    le = float(_ST["root_le_s_m"]) + ay * math.tan(_rad(float(_ST["le_sweep_deg"])))
    return StabStation(y=float(y), chord=c, le_s=le, te_s=le + c, z=float(_ST["z_m"]),
                       incidence_deg=float(_ST["incidence_deg"]))


def tail_airfoil(n: int = N_AIRFOIL, chord: float = 0.15, te_thickness: float | None = None) -> np.ndarray:
    """Kuyruk profili (NACA 0010), firar kenarı en az ``te_thickness`` (m; None → ``tail.te_thickness_m``)."""
    te = float(_T["te_thickness_m"]) if te_thickness is None else float(te_thickness)
    af = AF.section(_T["airfoil"], n)
    return AF.blunt_te(af, te / chord) if te > 0 else af


def stab_tc(y: float) -> float:
    """Stabilize yerel kalınlık oranı: kökte NACA 0010 (0,10), ``tail.stab.thickness_blend.y_m`` aralığında doğrusal
    olarak uçta ``tc_tip``'e (0,12) kalınlaşır — Ø12 kiriş borusu dış uca kadar profil içinde kalır (AERO-10/P8)."""
    tb = _ST.get("thickness_blend")
    t0 = AF.max_thickness(AF.section(_T["airfoil"]))[0]
    if not tb:
        return t0
    y0, y1 = (float(v) for v in tb["y_m"])
    u = float(np.clip((abs(float(y)) - y0) / (y1 - y0), 0.0, 1.0))
    return t0 + (float(tb["tc_tip"]) - t0) * u


def stab_section(y: float, n: int = N_AIRFOIL, te_thickness: float | None = None) -> np.ndarray:
    """Stabilize kesiti ``(2n − 1, 3)`` ``(s, y, z)``; ``incidence_axis_chord_fraction`` noktası etrafında döndürülmüş.
    Kalınlık ``stab_tc(y)`` (kökte %10, uca doğru %12)."""
    st = stab_station(y)
    af = tail_airfoil(n, st.chord, te_thickness)
    k = stab_tc(y) / AF.max_thickness(AF.section(_T["airfoil"]))[0]
    if abs(k - 1.0) > 1e-9:
        af = AF.scale_thickness(af, k)
    xr = float(_ST["incidence_axis_chord_fraction"])
    dx = (af[:, 0] - xr) * st.chord
    dz = af[:, 1] * st.chord
    i = _rad(st.incidence_deg)
    s = st.le_s + xr * st.chord + dx * math.cos(i) + dz * math.sin(i)
    z = st.z - dx * math.sin(i) + dz * math.cos(i)
    return np.column_stack([s, np.full_like(s, st.y), z])


@dataclass(frozen=True)
class FinStation:
    """Dikey kuyruk istasyonu (``h`` = kökten dikey açıklığı boyunca mesafe, eğik).

    ``le`` HK noktası ``(s, y, z)``; ``span_dir``/``normal`` spec takımında birim vektörler (normal dışa-aşağı).
    """

    h: float
    side: str
    chord: float
    le: tuple[float, float, float]
    span_dir: tuple[float, float, float]
    normal: tuple[float, float, float]

    @property
    def te_s(self) -> float:
        return self.le[0] + self.chord


def fin_station(h: float, side: str = "L") -> FinStation:
    """Dikey ``h`` istasyonu (0 … 0,32): HK oku 30° (açıklık doğrultusuna göre), 12° dışa eğik."""
    sg = _side_sign(side)
    H = float(_FN["height_m"])
    h = float(np.clip(h, 0.0, H))
    cant = _rad(float(_FN["cant_deg"]))
    cr, ct = float(_FN["root_chord_m"]), float(_FN["tip_chord_m"])
    c = cr - (cr - ct) * h / H
    span = (0.0, sg * math.sin(cant), math.cos(cant))
    normal = (0.0, sg * math.cos(cant), -math.sin(cant))
    le = (float(_FN["root_le_s_m"]) + h * math.tan(_rad(float(_FN["le_sweep_deg"]))),
          sg * float(_FN["y_root_m"]) + h * span[1], float(_FN["z_root_m"]) + h * span[2])
    return FinStation(h=h, side="L" if sg > 0 else "R", chord=c, le=le, span_dir=span, normal=normal)


def fin_section(h: float, side: str = "L", n: int = N_AIRFOIL, te_thickness: float | None = None) -> np.ndarray:
    """Dikey kesiti ``(2n − 1, 3)`` ``(s, y, z)``: veter s yönünde, kalınlık dikeyin normali boyunca (dışa +)."""
    st = fin_station(h, side)
    af = tail_airfoil(n, st.chord, te_thickness)
    le = np.asarray(st.le)
    nrm = np.asarray(st.normal)
    pts = le + np.outer(af[:, 0] * st.chord, [1.0, 0.0, 0.0]) + np.outer(af[:, 1] * st.chord, nrm)
    return pts


# =====================================================================================================
# Kumanda yüzeyleri ve menteşe hatları
# =====================================================================================================
@dataclass(frozen=True)
class HingeLine:
    """Bir kumanda yüzeyinin menteşe hattı (sözleşme: nesne orijini menteşe üzerinde, açıklığın ortasında).

    Spec takımında uç noktalar: ``p_in`` (iç/alt uç) ve ``p_out`` (dış/üst uç), kalınlık ortasında, menteşe
    veter konumunda (``1 − chord_fraction``). Blender takımında eksenler:

    * ``axis_outboard_b`` — ``p_in → p_out`` (dikeylerde aşağıdan yukarı).
    * ``axis_positive_b`` — bu eksen etrafında sağ el kuralıyla + dönüş, yüzeyin + sapmasıdır:
      yatay yüzeylerde FİRAR KENARI AŞAĞI, dümende FİRAR KENARI SANCAĞA (sağa). İki yanda aynı işaret aynı
      anlamı taşır. Not: iskele yüzeylerinde bu eksen içe, sancakta dışa bakar (sağ el kuralı gereği).
      Sözleşmedeki "yerel X dışa ve + = TE aşağı" iskele için aynı anda sağlanamaz; yerel X = ``axis_positive_b``
      kullanın (``frame_b``).
    * ``pos_max_deg``/``neg_max_deg`` — + ve − yöndeki fiziksel sınırlar (ör. kanatçık +12 aşağı / 20 yukarı).
    """

    name: str
    obj_name: str
    side: str
    kind: str                     # "wing" | "stab" | "fin"
    p_in: tuple[float, float, float]
    p_out: tuple[float, float, float]
    chord_fraction: float
    chord_in: float               # menteşeden firar kenarına (m), iç uçta
    chord_out: float
    nose_radius_in: float         # menteşede yarım kalınlık (yuvarlak burun yarıçapı), m
    nose_radius_out: float
    pos_max_deg: float
    neg_max_deg: float
    positive_means: str
    gap_m: float
    axis_outboard_b: tuple[float, float, float]
    axis_positive_b: tuple[float, float, float]
    span_from: float              # açıklık sınırları (y ya da dikeyde h)
    span_to: float

    @property
    def mid(self) -> tuple[float, float, float]:
        return tuple(0.5 * (a + b) for a, b in zip(self.p_in, self.p_out))  # type: ignore[return-value]

    @property
    def length(self) -> float:
        return float(np.linalg.norm(np.subtract(self.p_out, self.p_in)))

    @property
    def mid_b(self) -> tuple[float, float, float]:
        return to_blender(*self.mid)

    @property
    def p_in_b(self) -> tuple[float, float, float]:
        return to_blender(*self.p_in)

    @property
    def p_out_b(self) -> tuple[float, float, float]:
        return to_blender(*self.p_out)

    def frame_b(self) -> np.ndarray:
        """Önerilen yerel eksen takımı (3×3, sütunlar X, Y, Z; Blender dünya): X = ``axis_positive_b``,
        Y = menteşeden firar kenarına (geriye) doğru, X'e dik; Z = X × Y."""
        X = np.asarray(self.axis_positive_b)
        aft = np.array([-1.0, 0.0, 0.0])          # Blender'da geriye
        Y = aft - X * float(np.dot(aft, X))
        Y = Y / np.linalg.norm(Y)
        Z = np.cross(X, Y)
        return np.column_stack([X, Y, Z])


_CS_W, _CS_T = _W["control_surfaces"], _T["control_surfaces"]
CONTROL_SURFACES = ("Aileron", "FlapIn", "FlapOut", "Elevator", "Rudder")


def _wing_hinge_point(y: float, xh: float, n: int = N_AIRFOIL) -> tuple[tuple[float, float, float], float, float]:
    """Kanadın ``y`` istasyonunda ``xh`` veter konumunda (referans veter) kalınlık ortası, kalan veter, yarım kalınlık."""
    st = wing_station(y)
    s_h = st.te_s - (1.0 - xh) * st.chord
    # menteşe noktası: döndürülmüş kesitte s_h'deki kalınlık ortası
    zu, zl = wing_surface_z(s_h, y, "upper", n), wing_surface_z(s_h, y, "lower", n)
    return (s_h, y, 0.5 * (zu + zl)), st.te_s - s_h, 0.5 * (zu - zl)


def _make_hinge(name: str, side: str, kind: str, p_in, p_out, cf, c_in, c_out, r_in, r_out, pos_max, neg_max,
                positive_means, gap, span_from, span_to, te_point, te_want_b) -> HingeLine:
    a_out_b = _unit(vec_to_blender(np.subtract(p_out, p_in)))
    mid_b = np.asarray(to_blender(*[0.5 * (a + b) for a, b in zip(p_in, p_out)]))
    axis_pos = _axis_sign_for(a_out_b, mid_b, np.asarray(to_blender(*te_point)), np.asarray(te_want_b))
    obj = f"U_{name}_{side}"
    return HingeLine(name=name, obj_name=obj, side=side, kind=kind, p_in=tuple(map(float, p_in)),
                     p_out=tuple(map(float, p_out)), chord_fraction=cf, chord_in=c_in, chord_out=c_out,
                     nose_radius_in=r_in, nose_radius_out=r_out, pos_max_deg=float(pos_max),
                     neg_max_deg=float(neg_max), positive_means=positive_means, gap_m=float(gap),
                     axis_outboard_b=tuple(map(float, a_out_b)), axis_positive_b=tuple(map(float, axis_pos)),
                     span_from=float(span_from), span_to=float(span_to))


@functools.lru_cache(maxsize=None)
def hinge_line(name: str, side: str = "L") -> HingeLine:
    """Kumanda yüzeyinin menteşe hattı: ``name`` ∈ ``Aileron``, ``FlapIn``, ``FlapOut``, ``Elevator``, ``Rudder``.

    Nesne adı ``U_<name>_<side>`` (sözleşme). Menteşe noktaları iki uçta kalınlık ortasındadır; hat düzdür
    (uç noktalar arası doğru). Kanatta menteşe %72 veterde, kuyrukta %65 veterde.
    """
    sg = _side_sign(side)
    side = "L" if sg > 0 else "R"
    if name in ("Aileron", "FlapIn", "FlapOut"):
        key = {"Aileron": "aileron", "FlapIn": "flap_in", "FlapOut": "flap_out"}[name]
        d = _CS_W[key]
        cf = float(d["chord_fraction"])
        y0, y1 = float(d["y_from_m"]), float(d["y_to_m"])
        p0, c0, r0 = _wing_hinge_point(sg * y0, 1.0 - cf)
        p1, c1, r1 = _wing_hinge_point(sg * y1, 1.0 - cf)
        if name == "Aileron":
            pos_max, neg_max = d["down_deg"], d["up_deg"]
        else:
            pos_max, neg_max = d["down_deg"], d.get("up_deg", 0)
        ym = sg * 0.5 * (y0 + y1)
        stm = wing_station(ym)
        te_pt = (stm.te_s, ym, wing_mid_z(stm.te_s - 1e-4, ym) or stm.z_ref)
        return _make_hinge(name, side, "wing", p0, p1, cf, c0, c1, r0, r1, pos_max, neg_max,
                           "firar kenarı aşağı", _CS_W["hinge_gap_m"], y0, y1, te_pt, (0.0, 0.0, -1.0))
    if name == "Elevator":
        d = _CS_T["elevator"]
        cf = float(d["chord_fraction"])
        y0, y1 = float(d["y_from_m"]), float(d["y_to_m"])
        pts, rr, cc = [], [], []
        af = tail_airfoil(N_AIRFOIL, 0.15, 0.0)
        half_t = 0.5 * float(AF.thickness_at(af, 1.0 - cf))
        t_ref = AF.max_thickness(AF.section(_T["airfoil"]))[0]
        for yy in (y0, y1):
            st = stab_station(sg * yy)
            s_h = st.le_s + (1.0 - cf) * st.chord
            sec = stab_section(sg * yy)
            m = (len(sec) + 1) // 2
            up, lo = sec[:m][::-1], sec[m - 1:]
            zu = np.interp(s_h, np.maximum.accumulate(up[:, 0]), up[:, 2])
            zl = np.interp(s_h, np.maximum.accumulate(lo[:, 0]), lo[:, 2])
            pts.append((s_h, sg * yy, 0.5 * (zu + zl)))
            cc.append(st.te_s - s_h)
            rr.append(half_t * st.chord * stab_tc(yy) / t_ref)
        ym = sg * 0.5 * (y0 + y1)
        stm = stab_station(ym)
        te_pt = (stm.te_s, ym, stm.z)
        return _make_hinge(name, side, "stab", pts[0], pts[1], cf, cc[0], cc[1], rr[0], rr[1], d["down_deg"],
                           d["up_deg"], "firar kenarı aşağı", _CS_T["hinge_gap_m"], y0, y1, te_pt, (0.0, 0.0, -1.0))
    if name == "Rudder":
        d = _CS_T["rudder"]
        cf = float(d["chord_fraction"])
        h0, h1 = float(d["h_from_m"]), float(d["h_to_m"])
        af = tail_airfoil(N_AIRFOIL, 0.2, 0.0)
        half_t = 0.5 * float(AF.thickness_at(af, 1.0 - cf))
        pts, cc, rr = [], [], []
        for hh in (h0, h1):
            st = fin_station(hh, side)
            pts.append((st.le[0] + (1.0 - cf) * st.chord, st.le[1], st.le[2]))
            cc.append(cf * st.chord)
            rr.append(half_t * st.chord)
        stm = fin_station(0.5 * (h0 + h1), side)
        te_pt = (stm.te_s, stm.le[1], stm.le[2])
        return _make_hinge(name, side, "fin", pts[0], pts[1], cf, cc[0], cc[1], rr[0], rr[1], d["deflection_deg"],
                           d["deflection_deg"], "firar kenarı sancağa (sağa)", _CS_T["hinge_gap_m"], h0, h1, te_pt,
                           (0.0, -1.0, 0.0))
    raise KeyError(f"bilinmeyen kumanda yüzeyi: {name} (seçenekler: {', '.join(CONTROL_SURFACES)})")


def hinge_lines() -> list[HingeLine]:
    """Bütün kumanda yüzeylerinin menteşe hatları (her ad için L ve R) — 10 adet."""
    return [hinge_line(n, s) for n in CONTROL_SURFACES for s in ("L", "R")]


def control_surface_area(name: str) -> float:
    """İki yanın toplam kumanda yüzeyi alanı (m²; menteşeden firar kenarına, referans veterle)."""
    if name in ("Aileron", "FlapIn", "FlapOut"):
        d = _CS_W[{"Aileron": "aileron", "FlapIn": "flap_in", "FlapOut": "flap_out"}[name]]
        y = np.linspace(float(d["y_from_m"]), float(d["y_to_m"]), 2001)
        return 2 * float(d["chord_fraction"]) * _trapz(np.asarray(wing_chord_ref(y)), y)
    if name == "Elevator":
        d = _CS_T["elevator"]
        y = np.linspace(float(d["y_from_m"]), float(d["y_to_m"]), 2001)
        return 2 * float(d["chord_fraction"]) * _trapz(np.array([stab_station(v).chord for v in y]), y)
    if name == "Rudder":
        d = _CS_T["rudder"]
        h = np.linspace(float(d["h_from_m"]), float(d["h_to_m"]), 2001)
        return 2 * float(d["chord_fraction"]) * _trapz(np.array([fin_station(v).chord for v in h]), h)
    raise KeyError(name)


# =====================================================================================================
# İniş takımı
# =====================================================================================================
@dataclass(frozen=True)
class GearLeg:
    """Bir takım bacağı (``name`` = ``"N"``, ``"L"``, ``"R"``). Konumlar spec takımında ``(s, y, z)``.

    * ``pivot`` — trunnion (``U_GearPivot_<name>`` empty'si burada; yerel X = ``retract_axis_b``).
    * ``leg_dir`` — açıkken pivottan aksa birim vektör (spec takımı), ``rake_deg`` geriye yatıklık.
    * ``axle_unloaded`` / ``axle_static`` — yüksüz (uçuşta) ve statik yükte aks merkezi; statik konumda tekerlek
      zemine (``GROUND_Z``) değer. ``static_sag`` (dikey) bu koşuldan türetilir.
    * ``axle_retracted`` — yüksüz bacak ``retract_deg`` kadar katlanınca aks merkezi.
    * ``trail`` — çatal ofseti (m): aks, bacak (yönlendirme) ekseninin bu kadar gerisinde, bacağa dik; temas noktası
      yönlendirme ekseninin zeminle kesiştiği noktanın ≈ ``trail`` gerisindedir (burun tekeri shimmy'ye karşı).
    * ``retract_axis_b`` — Blender'da birim eksen: + ``retract_deg`` dönüş bacağı TOPLAR (açık → kapalı).
    * ``wheel_axis_b`` — açık konumda tekerlek dönme ekseni (sözleşme: teker yerel Y etrafında döner).
    """

    name: str
    pivot: tuple[float, float, float]
    leg_length: float
    rake_deg: float
    wheel_d: float
    wheel_w: float
    hub_d: float
    strut_d: float
    slider_d: float
    retract_deg: float
    retract_direction: str
    leg_dir: tuple[float, float, float]
    axle_unloaded: tuple[float, float, float]
    axle_static: tuple[float, float, float]
    axle_retracted: tuple[float, float, float]
    static_sag: float
    static_compression: float
    retract_axis_b: tuple[float, float, float]
    wheel_axis_b: tuple[float, float, float]
    trail: float = 0.0

    @property
    def wheel_r(self) -> float:
        return 0.5 * self.wheel_d

    @property
    def contact_static(self) -> tuple[float, float, float]:
        """Statik yerde teker temas noktası (aksın tam altı)."""
        return (self.axle_static[0], self.axle_static[1], self.axle_static[2] - self.wheel_r)

    @property
    def pivot_b(self) -> tuple[float, float, float]:
        return to_blender(*self.pivot)

    @property
    def obj_pivot(self) -> str:
        return f"U_GearPivot_{self.name}"


def _make_leg(name: str, d: dict, sg: float) -> GearLeg:
    px, py, pz = (float(v) for v in d["pivot"])
    pivot = (px, sg * py, pz)
    L = float(d["leg_length_m"])
    rake = _rad(float(d.get("rake_aft_deg", 0.0)))
    leg_dir = (math.sin(rake), 0.0, -math.cos(rake))
    aft_dir = (math.cos(rake), 0.0, math.sin(rake))           # bacağa dik, geriye (çatal ofseti yönü)
    trail = float(d.get("trail_m", 0.0))
    r = 0.5 * float(d["wheel_d_m"])
    axle_u = tuple(p + L * u + trail * a for p, u, a in zip(pivot, leg_dir, aft_dir))
    # statik: bacak boyunca sıkışma, teker zemine değsin
    comp = (GROUND_Z - (axle_u[2] - r)) / math.cos(rake)
    axle_s = tuple(p + (L - comp) * u + trail * a for p, u, a in zip(pivot, leg_dir, aft_dir))
    # toplama ekseni ve hedef yön
    pivot_b = np.asarray(to_blender(*pivot))
    axle_b = np.asarray(to_blender(*axle_u))
    if d["retract_direction"] == "aft":
        axis0 = np.array([0.0, 1.0, 0.0])
        want = np.array([-1.0, 0.0, 0.0])                     # Blender'da geriye
    elif d["retract_direction"] == "inward":
        axis0 = np.array([1.0, 0.0, 0.0])
        want = np.array([0.0, -sg, 0.0])                      # simetri düzlemine doğru
    else:
        raise ValueError(d["retract_direction"])
    axis = _axis_sign_for(axis0, pivot_b, axle_b, want)
    axle_r_b = rotate_about_axis(axle_b, pivot_b, axis, float(d["retract_deg"]))
    return GearLeg(name=name, pivot=pivot, leg_length=L, rake_deg=_deg(rake), wheel_d=float(d["wheel_d_m"]),
                   wheel_w=float(d["wheel_w_m"]), hub_d=float(d["hub_d_m"]), strut_d=float(d["strut_d_m"]),
                   slider_d=float(d["slider_d_m"]), retract_deg=float(d["retract_deg"]),
                   retract_direction=d["retract_direction"], leg_dir=leg_dir, axle_unloaded=axle_u,
                   axle_static=axle_s, axle_retracted=from_blender(*axle_r_b),
                   static_sag=GROUND_Z - (axle_u[2] - r), static_compression=comp,
                   retract_axis_b=tuple(map(float, axis)), wheel_axis_b=(0.0, 1.0, 0.0), trail=trail)


@functools.lru_cache(maxsize=None)
def gear_leg(name: str) -> GearLeg:
    """Takım bacağı: ``"N"`` (burun), ``"L"`` (sol ana), ``"R"`` (sağ ana)."""
    name = name.upper()
    if name == "N":
        return _make_leg("N", _G["nose"], 1.0)
    if name in ("L", "R"):
        return _make_leg(name, _G["main"], _side_sign(name))
    raise KeyError(name)


def gear_legs() -> list[GearLeg]:
    """Üç bacak: N, L, R."""
    return [gear_leg(n) for n in ("N", "L", "R")]


def gear_phase(gear: float) -> dict[str, float]:
    """``gear`` (0 = toplu, 1 = açık) değerinden kapak ve bacak ilerlemesi (0…1) — sözleşme zaman dilimleri.

    İndirme: kapaklar 0–0,15 açılır, bacak 0,15–0,85 iner, kapaklar 0,85–1,0 kapanır. ``legs`` 0 = toplu, 1 = açık;
    ``doors`` 0 = kapalı, 1 = açık. (Blender sürücüsü bunun basit-ifade eşdeğerini kullanır.)
    """
    sq = _G["sequence"]
    g = float(np.clip(gear, 0.0, 1.0))
    a0, a1 = sq["doors_open"]
    l0, l1 = sq["legs"]
    c0, c1 = sq["doors_close"]
    legs = float(smoothstep((g - l0) / (l1 - l0)))
    if g <= a1:
        doors = float(smoothstep((g - a0) / (a1 - a0)))
    elif g < c0:
        doors = 1.0
    else:
        doors = 1.0 - float(smoothstep((g - c0) / (c1 - c0)))
    return {"legs": legs, "doors": doors}


@dataclass(frozen=True)
class GearWell:
    """Takım kuyusu: plan çokgeni ``outline`` ``(s, y)`` (sol için), ağız ve tavan yüksekliği."""

    name: str                      # "N", "L", "R"
    obj_name: str                  # U_Bay_<leg>
    outline: tuple[tuple[float, float], ...]
    mouth_z: float                 # kuyu ağzı (deri) yaklaşık yüksekliği
    roof_z: float


@dataclass(frozen=True)
class GearDoor:
    """Takım kapağı (``U_Door_<leg>_<n>``). ``outline`` plan çokgeni ``(s, y)`` (deriye izdüşürülür).

    ``attach = "skin"``: menteşe ``hinge_p0 → hinge_p1``, + ``open_deg`` dönüş (``axis_open_b`` etrafında) açar.
    ``attach = "strut"``: bacağa bağlı kapak; toplu konumda deriyle aynı hizadadır, bacakla birlikte döner.
    """

    name: str
    leg: str
    attach: str
    outline: tuple[tuple[float, float], ...]
    hinge_p0: tuple[float, float, float] | None
    hinge_p1: tuple[float, float, float] | None
    open_deg: float
    axis_open_b: tuple[float, float, float] | None


def _sawtooth_edge(s0: float, y_a: float, y_b: float, depth: float, pitch: float, inward: float) -> list[tuple[float, float]]:
    """``s = s0`` kenarı boyunca (y_a → y_b) testere dişi; dişler ``inward`` yönünde (±1, s ekseninde) ``depth`` derin."""
    n = max(1, int(round(abs(y_b - y_a) / pitch)))
    ys = np.linspace(y_a, y_b, 2 * n + 1)
    return [(s0 + (inward * depth if k % 2 else 0.0), float(v)) for k, v in enumerate(ys)]


_SAW = _G.get("door_sawtooth", {"pitch_m": 0.012, "flank_deg": 36.0})
SAWTOOTH = {"pitch_m": float(_SAW["pitch_m"]),
            "depth_m": float(_SAW["pitch_m"]) / 2 * math.tan(math.radians(float(_SAW["flank_deg"])))}   # diş yanları 36°


def nose_plug_pocket() -> dict[str, float] | None:
    """Burun kuyusunun önündeki tıkaç cebi (``U_Door_N_3`` takım açıkken burada durur): ``s0``…``s1``, yarı genişlik
    ``hw``, tavan ``z_top`` (pivotun ``top_above_pivot`` üstü). Spec'te yoksa None."""
    nw = _G["nose"]["well"]
    pk = nw.get("plug_pocket")
    if not pk:
        return None
    s0 = float(nw["s_m"][0])
    return {"s0": s0 - float(pk["length_m"]), "s1": s0, "hw": float(pk["half_width_m"]),
            "z_top": float(_G["nose"]["pivot"][2]) + float(pk["top_above_pivot_m"])}


def gear_wells() -> list[GearWell]:
    """Üç kuyu: burun (s 0,51–0,766; önünde tıkaç cebi) ve iki ana kuyu (tekerlek kuyusu + katlanmış bacak yuvası)."""
    out = []
    nw = _G["nose"]["well"]
    s0, s1 = nw["s_m"]
    hw = float(nw["half_width_m"])
    mouth = fuselage_section(0.5 * (s0 + s1)).z_bottom
    pk = nose_plug_pocket()
    poly = [(s0, -hw), (s1, -hw), (s1, hw), (s0, hw)]
    if pk is not None:
        poly += [(s0, pk["hw"]), (pk["s0"], pk["hw"]), (pk["s0"], -pk["hw"]), (s0, -pk["hw"])]
    out.append(GearWell("N", "U_Bay_N", tuple(poly), mouth, float(nw["roof_z_m"])))
    mw = _G["main"]["well"]
    fair = _W["root_fairing"]
    for side in ("L", "R"):
        sg = _side_sign(side)
        (a0, a1), (b0, b1) = mw["s_m"], mw["y_m"]
        slot = _leg_slot(side)
        poly = [(a0, sg * b0), (a1, sg * b0), (a1, sg * b1)] + [(p[0], p[1]) for p in slot[::-1]] + [(a0, sg * b1)]
        out.append(GearWell(side, f"U_Bay_{side}", tuple(poly), float(fair["bottom_z_m"]),
                            float(fair["bottom_z_m"]) + float(mw["depth_m"])))
    return out


LEG_SLOT_OUT = 0.0095       # yuva açıklığı pivot ekseninin bu kadar dışında biter (Ø13 trunnion kovanı + 1,8 mm pay). varsayım
LEG_DOOR_OUT = -0.0075      # bacak kapağı pivotun bu kadar İÇİNDE biter: dönerken kanada/kovana girmez. varsayım


def _leg_slot(side: str, out: float = LEG_SLOT_OUT) -> list[tuple[float, float]]:
    """Katlanmış ana bacağın deri yuvası (dörtgen, ``(s, y)``): kuyu dış kenarından pivot ekseninin ``out`` kadar
    dışına (açıklık: ``LEG_SLOT_OUT``; bacak kapağı ``U_Door_<X>_2``: ``LEG_DOOR_OUT`` — pivotun içinde biter, böylece
    takım açılırken kapağın hiçbir yeri yukarı, kanadın içine dönmez)."""
    leg = gear_leg(side)
    sg = _side_sign(side)
    mw = _G["main"]
    w = 0.5 * float(mw["leg_slot_w_m"])
    wf = w + float(mw.get("leg_slot_front_extra_m", 0.0))
    p = np.asarray(leg.pivot)
    a = np.asarray(leg.axle_retracted)
    y_in = float(mw["well"]["y_m"][1])
    y_out = abs(p[1]) + out

    def s_at(yv):
        t = (abs(p[1]) - yv) / max(abs(p[1]) - abs(a[1]), 1e-9)
        return p[0] + t * (a[0] - p[0])

    return [(s_at(y_in) - wf, sg * y_in), (s_at(y_out) - wf, sg * y_out), (s_at(y_out) + w, sg * y_out),
            (s_at(y_in) + w, sg * y_in)]


def gear_doors() -> list[GearDoor]:
    """Kapaklar: burunda iki yandan açılan (clamshell) kapak (``U_Door_N_1`` sol, ``_2`` sağ);
    ana takımda tekerlek kuyusu kapağı (``U_Door_L_1``/``R_1``, içteki kenardan menteşeli) ve bacak kapağı
    (``U_Door_L_2``/``R_2``, bacağa bağlı). Kenarlar testere dişlidir (36°)."""
    doors: list[GearDoor] = []
    nd = _G["nose"]["doors"]["clamshell"]
    nw = _G["nose"]["well"]
    s0, s1 = nw["s_m"]
    hw = float(nw["half_width_m"])
    notch = nd["strut_notch"]
    n1 = float(notch["s_m"][1])                  # çentik ön kenardan başlar
    nh = float(notch["half_width_m"])
    p, d = SAWTOOTH["pitch_m"], SAWTOOTH["depth_m"]
    for k, side in enumerate(("L", "R"), start=1):
        sg = _side_sign(side)
        front = _sawtooth_edge(s0, sg * hw, sg * nh, d, p, +1.0)
        poly = front + [(n1, sg * nh), (n1, 0.0)] + _sawtooth_edge(s1, 0.0, sg * hw, d, p, -1.0)
        z0 = fuselage_z_at(s0, sg * hw) or fuselage_section(s0).z_bottom
        z1 = fuselage_z_at(s1, sg * hw) or fuselage_section(s1).z_bottom
        h0, h1 = (s0, sg * hw, z0), (s1, sg * hw, z1)
        # açılış: serbest kenar (y = 0) aşağı gitsin
        mid_b = np.asarray(to_blender(0.5 * (s0 + s1), sg * hw, 0.5 * (z0 + z1)))
        free_b = np.asarray(to_blender(0.5 * (s0 + s1), 0.0, 0.5 * (z0 + z1)))
        ax = _axis_sign_for(_unit(vec_to_blender(np.subtract(h1, h0))), mid_b, free_b, np.array([0, 0, -1.0]))
        doors.append(GearDoor(f"U_Door_N_{k}", "N", "skin", tuple(poly), h0, h1, float(nd["open_deg"]),
                              tuple(map(float, ax))))
    if _G["nose"]["doors"].get("strut_plug") is not None:     # çentik tıkacı: trunnion bloğuna bağlı (toplu konumda)
        doors.append(GearDoor("U_Door_N_3", "N", "strut", ((s0, -nh), (n1, -nh), (n1, nh), (s0, nh)), None, None,
                              0.0, None))
    mw = _G["main"]
    fair_z = float(_W["root_fairing"]["bottom_z_m"])
    for side in ("L", "R"):
        sg = _side_sign(side)
        (a0, a1), (b0, b1) = mw["well"]["s_m"], mw["well"]["y_m"]
        poly = _sawtooth_edge(a0, sg * b0, sg * b1, d, p, +1.0) + _sawtooth_edge(a1, sg * b1, sg * b0, d, p, -1.0)
        h0, h1 = (a0, sg * b0, fair_z), (a1, sg * b0, fair_z)
        mid_b = np.asarray(to_blender(0.5 * (a0 + a1), sg * b0, fair_z))
        free_b = np.asarray(to_blender(0.5 * (a0 + a1), sg * b1, fair_z))
        ax = _axis_sign_for(_unit(vec_to_blender(np.subtract(h1, h0))), mid_b, free_b, np.array([0, 0, -1.0]))
        doors.append(GearDoor(f"U_Door_{side}_1", side, "skin", tuple(poly), h0, h1,
                              float(mw["doors"]["wheel"]["open_deg"]), tuple(map(float, ax))))
        doors.append(GearDoor(f"U_Door_{side}_2", side, "strut", tuple(_leg_slot(side, LEG_DOOR_OUT)), None, None,
                              0.0, None))
    return doors


# =====================================================================================================
# İtki, faydalı yük, ayrıntılar
# =====================================================================================================
@dataclass(frozen=True)
class PropSpec:
    """İtici pervane. ``hub`` göbek merkezi (pervane düzlemi), ``axis_aft_b`` mil ekseni geriye (Blender birim).

    Sözleşme: ``U_Prop`` orijini göbekte, yerel X = ``axis_aft_b``. + dönüş (sağ el, yerel X etrafında) =
    arkadan bakınca saat yönü tersi = motorun gerçek dönüş yönü (itici, ters hatveli pervane).
    """

    diameter: float
    radius: float
    pitch: float
    blades: int
    hub: tuple[float, float, float]
    downthrust_deg: float
    axis_aft: tuple[float, float, float]       # spec takımı
    axis_aft_b: tuple[float, float, float]
    hub_d: float
    hub_len: float
    blade_max_chord: float
    blade_tip_chord: float
    spinner_d: float
    spinner_base_s: float
    spinner_tip_s: float
    rpm_max: float

    def disk_point(self, angle_deg: float) -> tuple[float, float, float]:
        """Disk çevresinde nokta (spec takımı); 0° = en alt (eğim nedeniyle biraz geride), +90° = iskele."""
        a = _rad(angle_deg)
        t = _rad(self.downthrust_deg)
        down = np.array([math.sin(t), 0.0, -math.cos(t)])          # disk düzleminde "aşağı"
        port = np.array([0.0, 1.0, 0.0])
        p = np.asarray(self.hub) + self.radius * (math.cos(a) * down + math.sin(a) * port)
        return tuple(map(float, p))

    def plane_s_at_z(self, z: float) -> float:
        """Disk düzleminin ``z`` yüksekliğindeki ``s`` konumu (5° eğim: alçaldıkça geride)."""
        return self.hub[0] + (self.hub[2] - z) * math.tan(_rad(self.downthrust_deg))

    @property
    def hub_b(self) -> tuple[float, float, float]:
        return to_blender(*self.hub)


def prop_spec() -> PropSpec:
    """Pervane ve spinner tanımı (spec ``propulsion.prop``/``spinner``)."""
    p, sp = _P["prop"], _P["spinner"]
    t = _rad(float(p["downthrust_deg"]))
    axis = (math.cos(t), 0.0, math.sin(t))                          # geriye ve yukarı (itki öne-aşağı)
    d = float(p["diameter_in"]) * 0.0254
    return PropSpec(diameter=d, radius=d / 2, pitch=float(p["pitch_in"]) * 0.0254, blades=int(p["blades"]),
                    hub=(float(p["plane_s_m"]), 0.0, float(p["hub_z_m"])), downthrust_deg=float(p["downthrust_deg"]),
                    axis_aft=axis, axis_aft_b=tuple(map(float, vec_to_blender(axis))), hub_d=float(p["hub_d_m"]),
                    hub_len=float(p["hub_len_m"]), blade_max_chord=float(p["blade_max_chord_m"]),
                    blade_tip_chord=float(p["blade_tip_chord_m"]), spinner_d=float(sp["d_m"]),
                    spinner_base_s=float(sp["base_s_m"]), spinner_tip_s=float(sp["tip_s_m"]),
                    rpm_max=float(p["rpm_max"]))


PROP = prop_spec()


def thrust_line_z(s: float) -> float:
    """İtki hattının ``s`` istasyonundaki yüksekliği (pervane göbeğinden öne 5° alçalır)."""
    return PROP.hub[2] - (PROP.hub[0] - s) * math.tan(_rad(PROP.downthrust_deg))


@dataclass(frozen=True)
class TurretSpec:
    """EO/IR çene tareti (yalnız kamera). ``ball_center`` tilt ekseni merkezi; pan ekseni dikey (Blender +Z).

    Rig: ``U_Turret_Pan`` yerel Z etrafında (+ = sola/iskeleye bakış), ``U_Turret_Tilt`` yerel Y etrafında.
    ``tilt_up_axis_b`` etrafında + dönüş kamerayı YUKARI kaldırır (Blender +Y etrafında + dönüş aşağı bakar).
    """

    s: float
    ball_d: float
    ball_center: tuple[float, float, float]
    bottom_z: float
    pan_housing_d: float
    collar_d: float
    collar_facets: int
    collar_depth: float
    window_eo_d: float
    window_ir_d: float
    window_spacing: float
    pan_range_deg: tuple[float, float]
    tilt_range_deg: tuple[float, float]
    belly_z: float
    pan_axis_b: tuple[float, float, float] = (0.0, 0.0, 1.0)
    tilt_up_axis_b: tuple[float, float, float] = (0.0, -1.0, 0.0)

    @property
    def ground_clearance(self) -> float:
        return self.bottom_z - GROUND_Z


def turret_spec() -> TurretSpec:
    """Taret tanımı (spec ``payload.turret``)."""
    t = SPEC["payload"]["turret"]
    r = 0.5 * float(t["ball_d_m"])
    s = float(t["s_m"])
    bottom = float(t["bottom_z_m"])
    return TurretSpec(s=s, ball_d=2 * r, ball_center=(s, 0.0, bottom + r), bottom_z=bottom,
                      pan_housing_d=float(t["pan_housing_d_m"]), collar_d=float(t["collar_d_m"]),
                      collar_facets=int(t["collar_facets"]), collar_depth=float(t["collar_depth_m"]),
                      window_eo_d=float(t["window_eo_d_m"]), window_ir_d=float(t["window_ir_d_m"]),
                      window_spacing=float(t["window_spacing_m"]), pan_range_deg=tuple(t["rig_pan_deg"]),
                      tilt_range_deg=tuple(t["rig_tilt_deg"]), belly_z=fuselage_section(s).z_bottom)


TURRET = turret_spec()


def hatch_outline() -> np.ndarray:
    """Aviyonik kapağı plan çokgeni ``(s, y)``: önü ve arkası 36° şevron (öne bakan ok), (6, 2)."""
    h = SPEC["details"]["hatch"]
    s0, s1, w = float(h["s_from_m"]), float(h["s_to_m"]), float(h["half_width_m"])
    k = w * math.tan(_rad(float(h["chevron_deg"])))
    return np.array([(s0, 0.0), (s0 + k, w), (s1, w), (s1 - k, 0.0), (s1, -w), (s0 + k, -w)])


@dataclass(frozen=True)
class Feature:
    """Küçük ayrıntı (anten, ışık, pitot): ``pos`` ``(s, y, z)``, ``direction`` spec takımında birim yön."""

    name: str
    obj_name: str
    kind: str
    pos: tuple[float, float, float]
    direction: tuple[float, float, float]
    params: dict = field(default_factory=dict)

    @property
    def pos_b(self) -> tuple[float, float, float]:
        return to_blender(*self.pos)

    @property
    def direction_b(self) -> tuple[float, float, float]:
        return tuple(map(float, vec_to_blender(self.direction)))


def antennas() -> list[Feature]:
    """Antenler (``U_Antenna_<ad>``): iç GNSS/telemetri (kapak altında) ve dış bıçak antenler; konum taban noktası."""
    out = []
    for a in SPEC["details"]["antennas"]:
        s, y = float(a["s_m"]), float(a["y_m"])
        sec = fuselage_section(s)
        if a.get("internal"):
            z = sec.z_top - 0.020                      # kapağın 20 mm altında (varsayım, kapak dışı değil)
            dirn = (0.0, 0.0, 1.0)
        elif a.get("side") == "bottom":
            z = fuselage_z_at(s, y, "bottom") or sec.z_bottom
            dirn = (0.0, 0.0, -1.0)
        else:
            z = fuselage_z_at(s, y, "top") or sec.z_top
            dirn = (0.0, 0.0, 1.0)
        params = {k: v for k, v in a.items() if k not in ("name", "s_m", "y_m")}
        out.append(Feature(a["name"], f"U_Antenna_{a['name']}", a["kind"], (s, y, z), dirn, params))
    return out


def pitot() -> Feature:
    """Pitot tüpü (``U_Pitot``): sol glove hücum kenarında, uç öne bakar; ``pos`` HK'deki taban noktası."""
    p = SPEC["details"]["pitot"]
    y = _side_sign(p["side"]) * float(p["y_m"])
    sec = wing_section(y)
    i_le = (len(sec) - 1) // 2
    base = tuple(map(float, sec[i_le]))
    return Feature("Pitot", "U_Pitot", "pitot", base, (-1.0, 0.0, 0.0),
                   {"protrusion_m": float(p["protrusion_m"]), "d_m": float(p["d_m"]), "base_d_m": float(p["base_d_m"])})


def lights() -> list[Feature]:
    """Işıklar (``U_Light_<ad>``): raked uç HK'sinde seyrüsefer (iskele kırmızı, sancak yeşil), dikey uçlarında
    beyaz strobe, taret yakasında durum halkası. ``pos`` lens merkezi (deri üzerinde), ``direction`` ışık yönü."""
    out = []
    for L in SPEC["details"]["lights"]:
        where = L["where"]
        if where == "raked_tip_le":
            y = _side_sign(L["side"]) * float(L["y_m"])
            sec = wing_section(y)
            p = sec[(len(sec) - 1) // 2]
            sweep = _rad(float(_W["raked_tip"]["le_sweep_deg"]))
            n = _unit((-math.sin(sweep), _side_sign(L["side"]) * math.cos(sweep), 0.0))   # HK normali (öne-dışa)
            out.append(Feature(L["name"], f"U_Light_{L['name']}", L["kind"], tuple(map(float, p)), tuple(n),
                               {"color": L["color"]}))
        elif where == "fin_tip_te":
            st = fin_station(float(_FN["height_m"]), L["side"])
            p = (st.te_s - 0.012, st.le[1], st.le[2])
            out.append(Feature(L["name"], f"U_Light_{L['name']}", L["kind"], p, (1.0, 0.0, 0.0),
                               {"color": L["color"]}))
        elif where == "turret_collar":
            t = TURRET
            out.append(Feature(L["name"], f"U_Light_{L['name']}", L["kind"], (t.s, 0.0, t.belly_z - t.collar_depth),
                               (0.0, 0.0, -1.0), {"color": L["color"], "ring_d_m": t.collar_d - 0.008}))
        else:
            raise ValueError(where)
    return out


def intake_spec() -> dict[str, Any]:
    """Karın NACA (gömülü) hava alığı. Dönüş: spec değerleri + türetilmişler:

    * ``throat_s``, ``ramp_s`` (rampa başı = boğaz − h/tan(rampa açısı)), ``skin_z_throat`` (boğazda karın derisi z),
      ``ramp_floor_z(s)`` yerine ``ramp_depth_at(s)`` fonksiyonu, ``half_width_at(s)`` (NACA ıraksak planform).
    * Geriye uyumlu anahtarlar (baskı modülü kanal kesicisi için): ``mouth_center`` = boğaz kesitinin merkezi
      (kanal tavanı + 4 mm), ``skin_z_at_mouth`` = kanal tabanı + 30 mm → eski "alık arkasında kutu" kesicisi
      (s boğaz + 6…30 mm, z [skin_z_at_mouth − 30 mm, mouth_center − 4 mm]) tam olarak dudak üstündeki kanalı
      (karın derisi + 3 mm … boğaz tavanı) deler.
    """
    i = dict(_P["intake"])
    st = float(i["s_from_m"])
    w, h = float(i["mouth_w_m"]), float(i["mouth_h_m"])
    ramp = _rad(float(i.get("ramp_deg", 7.0)))
    s_r = st - h / math.tan(ramp)
    zb = fuselage_section(st).z_bottom
    lip = 0.003
    i["type"] = str(i.get("type", "naca_flush_ventral"))
    i["throat_s"], i["ramp_s"], i["skin_z_throat"] = st, s_r, zb
    i["duct_z"] = (zb + lip, zb + h)
    i["mouth_center"] = (st, 0.0, zb + h + 0.004)
    i["skin_z_at_mouth"] = zb + lip + 0.030
    w0 = float(i.get("ramp_w0_m", 0.4 * w))

    def half_width_at(s: float) -> float:
        """NACA planform yarı genişliği: rampa başında w0/2, boğazda w/2 (ıraksak, dışbükey eğri)."""
        u = float(np.clip((s - s_r) / (st - s_r), 0.0, 1.0))
        return 0.5 * (w0 + (w - w0) * (1.0 - (1.0 - u) ** 2.2))

    def depth_at(s: float) -> float:
        """Rampa tabanının karın derisinden derinliği (m): rampa başında 0, boğazda h."""
        return float(np.clip((s - s_r) / (st - s_r), 0.0, 1.0)) * h

    i["half_width_at"], i["depth_at"] = half_width_at, depth_at
    i["area_m2"] = w * h
    return i


# =====================================================================================================
# Motor bölmesi zarfları (ters DLE-20, susturucu) ve iç yerleşim zarfları (depo, akü)
# =====================================================================================================
@dataclass(frozen=True)
class EnvPart:
    """Paketleme zarfı (spec takımı): ``kind`` = ``"cyl"`` (eksen = ``axes[:, 0]``, ``half`` = (yarı boy, r, r)) ya da
    ``"box"`` (``half`` = üç eksen boyunca yarı ölçüler). ``axes`` sütunları birim vektörler."""

    name: str
    kind: str
    center: tuple[float, float, float]
    axes: tuple
    half: tuple[float, float, float]

    def surface_points(self, n: int = 14) -> np.ndarray:
        """Zarf yüzeyinden örnek noktalar (spec), ``(k, 3)``."""
        A = np.asarray(self.axes, float)
        c = np.asarray(self.center, float)
        hx, hy, hz = self.half
        pts = []
        if self.kind == "cyl":
            for u in np.linspace(-1.0, 1.0, n):
                for th in np.linspace(0.0, 2 * math.pi, 2 * n, endpoint=False):
                    pts.append(c + A[:, 0] * u * hx + hy * (math.cos(th) * A[:, 1] + math.sin(th) * A[:, 2]))
            for rr in np.linspace(0.0, hy, max(2, n // 3)):
                for th in np.linspace(0.0, 2 * math.pi, 2 * n, endpoint=False):
                    for u in (-1.0, 1.0):
                        pts.append(c + A[:, 0] * u * hx + rr * (math.cos(th) * A[:, 1] + math.sin(th) * A[:, 2]))
        else:
            g = np.linspace(-1.0, 1.0, n)
            for ax in range(3):
                o1, o2 = [k for k in range(3) if k != ax]
                for sgn in (-1.0, 1.0):
                    for a in g:
                        for b in g:
                            q = np.zeros(3)
                            q[ax], q[o1], q[o2] = sgn, a, b
                            pts.append(c + A @ (q * np.asarray(self.half)))
        return np.asarray(pts)


def thrust_axis() -> tuple[np.ndarray, np.ndarray]:
    """İtki ekseni (spec takımı): (pervane göbeği, birim yön geriye-yukarı)."""
    t = _rad(float(_P["prop"]["downthrust_deg"]))
    return np.array([float(_P["prop"]["plane_s_m"]), 0.0, float(_P["prop"]["hub_z_m"])]), \
        np.array([math.cos(t), 0.0, math.sin(t)])


def engine_washer() -> np.ndarray:
    """Motor pervane rondelası (krank ekseninde; uzatma milinin ön ucu), spec takımı."""
    hub, a = thrust_axis()
    return hub - a * (0.5 * float(_P["prop"]["hub_len_m"]) + float(_P["prop"].get("extension_m", 0.0)))


def engine_envelope() -> list[EnvPart]:
    """Ters bağlı DLE-20 ve susturucu zarfları (spec takımı): rulman burnu, karter, karbüratör + emme, silindir
    kanatçık bloğu (krank ekseninin ALTINDA), buji başlığı, susturucu (sol, silindirin yanında). Ölçüler
    ``propulsion.engine`` / ``muffler``; silindir ekseni rondelanın ``cyl_axis_from_washer_m`` önünde."""
    E, Mf = _P["engine"], _P["muffler"]
    hub, a = thrust_axis()
    d = np.array([a[2], 0.0, -a[0]])                      # silindir yönü: itki eksenine dik, aşağı (5° geriye)
    yv = np.array([0.0, 1.0, 0.0])
    W = engine_washer()
    inv = str(E.get("orientation", "inverted")) == "inverted"
    if not inv:
        d = -d
    ax_cyl = (tuple(a), tuple(yv), tuple(np.cross(a, yv)))
    fb_l, cc_l = float(E["front_bearing_len_m"]), float(E["crankcase_len_m"])
    out = []

    def cyl(name, p0, p1, r):
        p0, p1 = np.asarray(p0), np.asarray(p1)
        L = float(np.linalg.norm(p1 - p0))
        u = (p1 - p0) / L
        v = np.cross(u, yv) if abs(u[1]) < 0.9 else np.cross(u, [1.0, 0, 0])
        v = v / np.linalg.norm(v)
        w = np.cross(u, v)
        out.append(EnvPart(name, "cyl", tuple(0.5 * (p0 + p1)), (tuple(u), tuple(v), tuple(w)), (0.5 * L, r, r)))

    cyl("front_bearing", W - a * fb_l, W, 0.5 * float(E["front_bearing_d_m"]))
    c0 = W - a * (fb_l + cc_l)
    cyl("crankcase", c0, W - a * fb_l, 0.5 * float(E["crankcase_d_m"]))
    carb_tip = W - a * float(E["carb_to_washer_m"])
    cyl("carb", carb_tip, c0, 0.5 * float(E["carb_d_m"]))
    Pc = W - a * float(E["cyl_axis_from_washer_m"])
    r_cc = 0.5 * float(E["crankcase_d_m"])
    h_head = float(E["crank_to_head_top_m"])
    lo = 0.6 * r_cc
    out.append(EnvPart("cylinder", "box", tuple(Pc + d * 0.5 * (lo + h_head)), ax_cyl[:1] + (tuple(yv), tuple(d)),
                       (0.5 * float(E["cylinder_fin_l_m"]), 0.5 * float(E["cylinder_fin_w_m"]), 0.5 * (h_head - lo))))
    cyl("spark_cap", Pc + d * h_head, Pc + d * (h_head + float(E["spark_cap_m"])), 0.008)
    env = [float(v) for v in Mf["envelope_m"]]
    s_m = float(Mf["center_s_m"])
    P0 = hub + a * (s_m - hub[0]) / a[0]
    side = _side_sign(Mf.get("side", "L"))
    yc = side * (0.5 * float(E["cylinder_fin_w_m"]) + float(Mf.get("fin_gap_m", 0.004)) + 0.5 * env[1])
    cm = P0 + d * float(Mf["center_below_axis_m"]) + yv * yc
    out.append(EnvPart("muffler", "box", tuple(cm), (tuple(a), tuple(yv), tuple(d)), tuple(0.5 * v for v in env)))
    return out


def tank_envelopes() -> list[EnvPart]:
    """İki yakıt deposu zarfı (2 × 0,70 L + %30 et/şekil payı), kanat kutusunun (G10 köprü) üstünde, CG'de yan yana."""
    F = _F["internals"]["fuel_tanks"]
    vol = float(F["volume_l_each"]) * 1.30 / 1000.0
    sc = float(F["centre_s_m"])
    z0, z1 = -0.016, 0.066                               # köprü üstü → omuz longeronu altı. varsayım
    y0, y1 = 0.004, 0.072
    L = vol / ((z1 - z0) * (y1 - y0))
    out = []
    for sd in ("L", "R"):
        sg = _side_sign(sd)
        out.append(EnvPart(f"tank_{sd}", "box", (sc, sg * 0.5 * (y0 + y1), 0.5 * (z0 + z1)),
                           ((1.0, 0, 0), (0, 1.0, 0), (0, 0, 1.0)), (0.5 * L, 0.5 * (y1 - y0), 0.5 * (z1 - z0))))
    return out


def battery_envelope() -> EnvPart:
    """Ana akü (4S2P 21700, 130 Wh) zarfı: kızakta, burun kuyusu tavanının üstünde, nominal s'de."""
    sc = float(_F["internals"]["battery_nominal_s_m"])
    z0 = float(_G["nose"]["well"]["roof_z_m"]) + 0.003
    half = (0.044, 0.037, 0.024)
    return EnvPart("battery", "box", (sc, 0.0, z0 + half[2]), ((1.0, 0, 0), (0, 1.0, 0), (0, 0, 1.0)), half)


def muffler_outlet() -> Feature:
    """Susturucu çıkışı (sol alt yanak, susturucu zarfının arka ucu): konum (kaporta yüzeyi) ve yön (35° aşağı,
    35° dışa, geriye). ``params``: ``scarf_deg``, ``shield_m``."""
    m = _P["muffler"]
    s = float(m["outlet_s_m"])
    mf = next(p for p in engine_envelope() if p.name == "muffler")
    z = float(mf.center[2])
    cl = _P["cowl"]["cheek_left"]
    y = fuselage_half_width_at(s, z) + float(cl["bulge_m"]) * 0.75
    dn, ou = _rad(float(m["outlet_down_deg"])), _rad(float(m["outlet_out_deg"]))
    d = (math.cos(dn) * math.cos(ou), math.cos(dn) * math.sin(ou), -math.sin(dn))
    o = m.get("outlet", {}) or {}
    return Feature("Muffler", "U_Exhaust_Muffler", "exhaust", (s, y, z), d,
                   {"scarf_deg": float(o.get("scarf_deg", 0.0)), "shield_m": tuple(o.get("shield_m", (0.03, 0.02, 0.0005)))})


# =====================================================================================================
# Kiriş boruları
# =====================================================================================================
@dataclass(frozen=True)
class SparTube:
    """CF boru: ``p0 → p1`` eksen (spec takımı, sol yarı ya da gövde ortası), dış/iç çap."""

    name: str
    label: str
    od: float
    id: float
    p0: tuple[float, float, float]
    p1: tuple[float, float, float]
    side: str

    @property
    def length(self) -> float:
        return float(np.linalg.norm(np.subtract(self.p1, self.p0)))


def spar_tubes(side: str = "L") -> list[SparTube]:
    """Kanat ve kuyruk boruları (sol ya da sağ yarı). Kanat boruları yerel kalınlık ortasında, ``chord_fraction``
    noktalarında (referans veter); arka kiriş iki ucunda %68'dedir ve düzdür. Gövde longeronları için
    ``longeron_path`` kullanın (borular gövdeyi izleyerek hafif eğilir)."""
    sg = _side_sign(side)
    side = "L" if sg > 0 else "R"
    out = []
    for t in SPEC["print"]["spar_tubes"]:
        if "y_m" not in t:
            continue
        y0, y1 = (float(v) for v in t["y_m"])
        cf = float(t["chord_fraction"])
        pts = []
        for yy in (y0, y1):
            st = wing_station(sg * yy)
            s_c = WING_SPAR_S + (cf - WING_SPAR_FRAC) * st.chord_ref * math.cos(_rad(st.incidence_deg))
            z = wing_mid_z(s_c, sg * yy)
            pts.append((s_c, sg * yy, z if z is not None else st.z_ref))
        out.append(SparTube(t["name"], t["name_tr"], t["od_mm"] / 1000, t["id_mm"] / 1000, pts[0], pts[1], side))
    sp = _ST["spar"]
    L = float(sp["length_m"])
    sw = _rad(float(_ST["le_sweep_deg"]))
    s0 = float(_ST["root_le_s_m"]) + float(sp["offset_from_le_m"]) + float(sp["y_from_m"]) * math.tan(sw)
    y0 = float(sp["y_from_m"])
    p0 = (s0, sg * y0, float(_ST["z_m"]))
    p1 = (s0 + L * math.sin(sw), sg * (y0 + L * math.cos(sw)), float(_ST["z_m"]))
    out.append(SparTube("stab_spar", "Stabilize kirişi", float(sp["od_m"]), float(sp["id_m"]), p0, p1, side))
    rr = _ST["rear_rod"]
    cf = float(rr["chord_fraction"])
    a = stab_station(0.0)
    b_y = float(rr["length_m"]) * 0.88            # ilk yaklaşım; sonra boyu tam tutturulur
    for _ in range(20):
        b = stab_station(b_y)
        p0 = (a.le_s + cf * a.chord, 0.0, a.z)
        p1 = (b.le_s + cf * b.chord, sg * b_y, b.z)
        Lc = math.dist(p0, (p1[0], abs(p1[1]), p1[2]))
        b_y *= float(rr["length_m"]) / Lc
    out.append(SparTube("stab_rear_rod", "Stabilize arka çubuğu", float(rr["od_m"]), float(rr["id_m"]), p0, p1, side))
    fs = _FN["spar"]
    st0 = fin_station(0.0, side)
    dirv = _unit(np.asarray(st0.span_dir) + np.array([math.tan(_rad(float(_FN["le_sweep_deg"]))), 0, 0]))
    q0 = np.asarray(st0.le) + np.array([float(fs["offset_from_le_m"]), 0, 0])
    q1 = q0 + float(fs["length_m"]) * dirv
    out.append(SparTube("fin_spar", "Dikey kirişi", float(fs["od_m"]), float(fs["id_m"]), tuple(map(float, q0)),
                        tuple(map(float, q1)), side))
    return out


def stab_spar_fit(n: int = 60) -> dict[str, float]:
    """Stabilize kirişi (Ø12/10) boyunca profil payı: her y'de boru eksenindeki yerel kesit kalınlığı − (OD + 2 ×
    kuyruk kabuğu) (AERO-10, ≥ 0,5 mm) ve delik (OD + 0,4 mm) üstünde kalan en ince kabuk (P8, ≥ 0,8 mm). Dönüş (m):
    ``margin_min``, ``skin_min``, ``y_at_min``."""
    t = next(x for x in spar_tubes("L") if x.name == "stab_spar")
    skin = float(SPEC["print"]["walls_mm"]["tail_skin"]) / 1000.0
    p0, p1 = np.asarray(t.p0), np.asarray(t.p1)
    best = (1.0, 1.0, 0.0)
    for f in np.linspace(0.0, 1.0, n):
        p = p0 + f * (p1 - p0)
        sec = stab_section(float(p[1]), 161)
        m = (len(sec) + 1) // 2
        up, lo = sec[:m][::-1], sec[m - 1:]
        zu = float(np.interp(p[0], np.maximum.accumulate(up[:, 0]), up[:, 2]))
        zl = float(np.interp(p[0], np.maximum.accumulate(lo[:, 0]), lo[:, 2]))
        margin = (zu - zl) - (t.od + 2 * skin)
        skin_hole = min(zu - (p[2] + 0.5 * (t.od + 0.0004)), (p[2] - 0.5 * (t.od + 0.0004)) - zl)
        if margin < best[0]:
            best = (margin, min(best[1], skin_hole), float(p[1]))
        best = (best[0], min(best[1], skin_hole), best[2])
    return {"margin_min": best[0], "skin_min": best[1], "y_at_min": best[2]}


# =====================================================================================================
# Malzemeler
# =====================================================================================================
def srgb_to_linear(c: float) -> float:
    """sRGB (0…1) → lineer (Blender düğümlerinin beklediği renk)."""
    return c / 12.92 if c <= 0.04045 else ((c + 0.055) / 1.055) ** 2.4


def hex_to_rgb(hex_str: str) -> tuple[float, float, float]:
    """``"#RRGGBB"`` → sRGB (0…1)."""
    h = hex_str.lstrip("#")
    return tuple(int(h[i:i + 2], 16) / 255.0 for i in (0, 2, 4))  # type: ignore[return-value]


def hex_to_linear(hex_str: str) -> tuple[float, float, float]:
    """``"#RRGGBB"`` → lineer RGB (0…1)."""
    return tuple(srgb_to_linear(c) for c in hex_to_rgb(hex_str))  # type: ignore[return-value]


@dataclass(frozen=True)
class MaterialSpec:
    """Blender malzemesi ``UM_<ad>`` tanımı (materials modülü düğüm ağacını bundan kurar)."""

    name: str
    label: str
    ral: str | None
    hex: str
    rgb_srgb: tuple[float, float, float]
    rgb_linear: tuple[float, float, float]
    roughness: float
    metallic: float
    transmission: float = 0.0
    ior: float = 1.45
    emission: float = 0.0
    extra: dict = field(default_factory=dict)

    @property
    def rgba_linear(self) -> tuple[float, float, float, float]:
        return (*self.rgb_linear, 1.0)


def _materials() -> dict[str, MaterialSpec]:
    out = {}
    for name, m in SPEC["materials"].items():
        if not name.startswith("UM_"):
            continue
        known = {"ral", "name_tr", "hex", "roughness", "metallic", "transmission", "ior", "emission"}   # diğerleri → extra
        out[name] = MaterialSpec(name=name, label=m["name_tr"], ral=m.get("ral"), hex=m["hex"],
                                 rgb_srgb=hex_to_rgb(m["hex"]), rgb_linear=hex_to_linear(m["hex"]),
                                 roughness=float(m["roughness"]), metallic=float(m["metallic"]),
                                 transmission=float(m.get("transmission", 0.0)), ior=float(m.get("ior", 1.45)),
                                 emission=float(m.get("emission", 0.0)),
                                 extra={k: v for k, v in m.items() if k not in known})
    return out


MATERIALS: dict[str, MaterialSpec] = _materials()
LIVERIES: dict = SPEC["materials"].get("liveries", {})

# Rol → malzeme adı (gövde/takım modülleri slot atarken kullanır)
MATERIAL_ROLES: dict[str, str] = {
    "skin_top": "UM_SkinTop", "skin_bottom": "UM_SkinBottom", "accent": "UM_Accent", "stripe": "UM_Turquoise",
    "bay": "UM_Liner", "door_inner": "UM_Liner", "warning": "UM_Orange", "hatch": "UM_SmokeHatch",
    "seal": "UM_Seal", "bezel": "UM_Bezel", "exhaust": "UM_Exhaust", "le_strip": "UM_Erosion", "hazard": "UM_Hazard",
    "pacf": "UM_PACF", "nozzle": "UM_Nozzle", "prop": "UM_Prop", "spinner": "UM_Spinner", "gear": "UM_Gear",
    "hub": "UM_Hub", "tire": "UM_Tire", "turret": "UM_TurretBody", "glass": "UM_SensorGlass",
    "engine": "UM_Engine", "carbon": "UM_Carbon", "steel": "UM_Steel", "antenna": "UM_Antenna", "tpu": "UM_TPU",
    "nav_red": "UM_NavRed", "nav_green": "UM_NavGreen", "strobe": "UM_Strobe", "status": "UM_StatusLED",
}


# =====================================================================================================
# Özet (CLI)
# =====================================================================================================
def summary() -> dict[str, Any]:
    """Önemli türetilmiş değerler (hızlı bakış ve testler için)."""
    wr = wing_reference()
    return {
        "wing": wr,
        "stab_root": stab_station(0.0),
        "fin_tip": fin_station(float(_FN["height_m"])),
        "gear": {g.name: (g.axle_static, g.axle_retracted, g.static_sag) for g in gear_legs()},
        "prop_hub": PROP.hub,
        "turret_center": TURRET.ball_center,
        "cg_blender": U_ROOT_B,
    }


if __name__ == "__main__":
    import pprint

    pprint.pprint(summary(), width=120)
    for h in hinge_lines():
        print(f"{h.obj_name:15s} mid {np.round(h.mid, 4)}  axis+ (Blender) {np.round(h.axis_positive_b, 3)}  "
              f"+{h.pos_max_deg:.0f}/−{h.neg_max_deg:.0f}° ({h.positive_means})")
