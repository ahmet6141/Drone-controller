"""YELKOVAN YK-38 — profil (airfoil) araçları. Saf Python + numpy; bpy gerektirmez.

Kanat ve kuyruk kesitleri buradan üretilir:

* ``load("sd7062")`` / ``load("sd7032")``: UIUC veritabanından indirilmiş özgün koordinatlar
  (``ucav/data/airfoils/*.dat``; başlık korunur, kaynak satırı ``#`` ile eklenmiştir).
* ``naca4("0010")``: NACA 4 haneli üretici (kuyruk: stabilize ve dikeyler).
* ``resample(coords, n)``: normalize eder (hücum kenarı (0, 0), firar kenarı ortası (1, 0)) ve üst/alt
  yüzeyi **aynı kosinüs x istasyonlarında** yeniden örnekler. Bütün kesitler aynı nokta sayısına ve aynı
  parametrelemeye sahip olur; böylece iki profil doğrudan karıştırılabilir (``blend``) ve loft edilebilir.
* ``scale_thickness``: kamber çizgisi korunarak kalınlık ölçekleme (kökte %14 → %15,5).
* ``blunt_te``: baskı için en az firar kenarı kalınlığı (kalınlık farkı arka bölgeye yumuşak dağıtılır).

Nokta sırası Selig biçimidir: firar kenarı üstten başlar → hücum kenarı → firar kenarı alttan biter.
Örneklenmiş bir kesit ``2n − 1`` noktadır; ``i = n − 1`` indisli nokta hücum kenarıdır.
Bütün x, y değerleri veter birimindedir (veter = 1).
"""
from __future__ import annotations

import functools
import math
import re
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np

DATA_DIR = Path(__file__).resolve().parent / "data" / "airfoils"
DEFAULT_N = 81                     # yüzey başına nokta (hücum kenarı dahil) → kesit başına 2·81 − 1 = 161 nokta
_DENSE_PER_SEGMENT = 40            # parametrik spline'ın yoğun değerlendirme sıklığı (veri aralığı başına)


@dataclass(frozen=True)
class Airfoil:
    """Dosyadan okunmuş ya da üretilmiş bir profil.

    ``coords``: (M, 2) Selig sırası (TE üst → LE → TE alt), veter birimi. ``header``: dosyanın ``#`` ile
    başlamayan ilk satırı (profil adı), ``source``: ``#`` ile eklenmiş kaynak satırı.
    """

    name: str
    coords: np.ndarray
    header: str = ""
    source: str = ""
    meta: dict = field(default_factory=dict)


# ---------------------------------------------------------------------------------------------------
# Okuma ve üretim
# ---------------------------------------------------------------------------------------------------
_NUM = re.compile(r"^\s*[-+]?(\d+\.?\d*|\.\d+)([eE][-+]?\d+)?\s+[-+]?(\d+\.?\d*|\.\d+)([eE][-+]?\d+)?\s*$")


def read_dat(path: str | Path) -> Airfoil:
    """Selig ya da Lednicer biçimli ``.dat`` dosyasını okur.

    Sayısal olmayan satırlar (ad, ``#`` kaynak satırı) üstbilgi kabul edilir. Lednicer biçimi (ilk sayısal
    satır nokta sayılarıdır, ör. ``61. 61.``) algılanır ve Selig sırasına çevrilir.
    """
    path = Path(path)
    header, source, pts = "", "", []
    for line in path.read_text(encoding="utf-8", errors="replace").splitlines():
        s = line.strip()
        if not s:
            continue
        if s.startswith("#"):
            source = source or s.lstrip("#").strip()
            continue
        if _NUM.match(s):
            a, b = s.split()[:2]
            pts.append((float(a), float(b)))
        elif not pts and not header:
            header = s
    arr = np.asarray(pts, dtype=float)
    if arr.ndim != 2 or len(arr) < 8:
        raise ValueError(f"{path}: profil koordinatı okunamadı")
    if arr[0, 0] > 1.5 and arr[0, 1] > 1.5:          # Lednicer: (n_üst, n_alt), sonra LE→TE üst, LE→TE alt
        nu, nl = int(arr[0, 0]), int(arr[0, 1])
        up, lo = arr[1:1 + nu], arr[1 + nu:1 + nu + nl]
        arr = np.vstack([up[::-1], lo[1:]])
    return Airfoil(name=path.stem.lower(), coords=arr, header=header, source=source)


def naca4(code: str = "0010", n: int = DEFAULT_N, closed_te: bool = True) -> np.ndarray:
    """NACA 4 haneli profil (ör. ``"0010"``, ``"2412"``), Selig sırasında ``2n − 1`` nokta.

    Kalınlık dağılımı klasik NACA denklemidir; ``closed_te=True`` iken son katsayı −0,1036 alınır ve firar
    kenarı sıfır kalınlıkta kapanır (baskı kalınlığı sonra ``blunt_te`` ile verilir). Noktalar kamber
    çizgisine dik uygulanır; x istasyonları ``x_stations(n)`` ile aynıdır.
    """
    if len(code) != 4 or not code.isdigit():
        raise ValueError("NACA kodu 4 rakam olmalı, ör. '0010'")
    m, p, t = int(code[0]) / 100.0, int(code[1]) / 10.0, int(code[2:]) / 100.0
    x = x_stations(n)
    a4 = -0.1036 if closed_te else -0.1015
    yt = 5 * t * (0.2969 * np.sqrt(x) - 0.1260 * x - 0.3516 * x ** 2 + 0.2843 * x ** 3 + a4 * x ** 4)
    if m == 0 or p == 0:
        yc, dyc = np.zeros_like(x), np.zeros_like(x)
    else:
        yc = np.where(x < p, m / p ** 2 * (2 * p * x - x ** 2), m / (1 - p) ** 2 * ((1 - 2 * p) + 2 * p * x - x ** 2))
        dyc = np.where(x < p, 2 * m / p ** 2 * (p - x), 2 * m / (1 - p) ** 2 * (p - x))
    th = np.arctan(dyc)
    xu, yu = x - yt * np.sin(th), yc + yt * np.cos(th)
    xl, yl = x + yt * np.sin(th), yc - yt * np.cos(th)
    up = np.column_stack([xu, yu])[::-1]
    lo = np.column_stack([xl, yl])[1:]
    coords = np.vstack([up, lo])
    if m != 0 and p != 0:                    # kamberli profilde x istasyonlarını ortak hale getir
        coords = resample(coords, n)
    return coords


@functools.lru_cache(maxsize=None)
def _load_cached(name: str) -> Airfoil:
    key = name.lower().replace(" ", "")
    if key.startswith("naca") and len(key) == 8 and key[4:].isdigit():
        return Airfoil(name=key, coords=naca4(key[4:], DEFAULT_N), header=f"NACA {key[4:]}",
                       source="NACA 4 haneli denklem (üretildi, kapalı firar kenarı)")
    path = DATA_DIR / f"{key}.dat"
    if not path.exists():
        raise FileNotFoundError(f"profil bulunamadı: {name} ({path})")
    return read_dat(path)


def load(name: str) -> Airfoil:
    """Kütüphane profili: ``"sd7062"``, ``"sd7032"`` (dosyadan) ya da ``"naca0010"`` gibi NACA 4 haneli (üretilir)."""
    a = _load_cached(name)
    return Airfoil(a.name, a.coords.copy(), a.header, a.source, dict(a.meta))


@functools.lru_cache(maxsize=None)
def _section_cached(name: str, n: int) -> np.ndarray:
    return resample(_load_cached(name).coords, n)


def section(name: str, n: int = DEFAULT_N) -> np.ndarray:
    """Kütüphane profilinin normalize edilmiş ve ``n`` istasyona örneklenmiş kesiti, (2n − 1, 2). Önbellekli."""
    return _section_cached(name.lower(), int(n)).copy()


# ---------------------------------------------------------------------------------------------------
# Parametrik spline (yay boyu parametreli doğal kübik), normalizasyon ve yeniden örnekleme
# ---------------------------------------------------------------------------------------------------
def _natural_cubic_m(t: np.ndarray, v: np.ndarray) -> np.ndarray:
    """Doğal kübik spline'ın düğüm ikinci türevleri (üç köşegenli sistem, Thomas algoritması)."""
    n = len(t)
    h = np.diff(t)
    if n < 3:
        return np.zeros(n)
    a = h[:-1].copy()                       # alt köşegen
    b = 2.0 * (h[:-1] + h[1:])              # ana köşegen
    c = h[1:].copy()                        # üst köşegen
    d = 6.0 * ((v[2:] - v[1:-1]) / h[1:] - (v[1:-1] - v[:-2]) / h[:-1])
    for i in range(1, len(b)):              # ileri eleme
        w = a[i] / b[i - 1]
        b[i] -= w * c[i - 1]
        d[i] -= w * d[i - 1]
    m_in = np.zeros(len(b))
    m_in[-1] = d[-1] / b[-1]
    for i in range(len(b) - 2, -1, -1):     # geri yerine koyma
        m_in[i] = (d[i] - c[i] * m_in[i + 1]) / b[i]
    return np.concatenate([[0.0], m_in, [0.0]])


def _spline_eval(t: np.ndarray, v: np.ndarray, m: np.ndarray, tq: np.ndarray) -> np.ndarray:
    i = np.clip(np.searchsorted(t, tq) - 1, 0, len(t) - 2)
    h = t[i + 1] - t[i]
    a = (t[i + 1] - tq) / h
    b = (tq - t[i]) / h
    return a * v[i] + b * v[i + 1] + ((a ** 3 - a) * m[i] + (b ** 3 - b) * m[i + 1]) * h ** 2 / 6.0


def _dense(coords: np.ndarray, per_segment: int = _DENSE_PER_SEGMENT) -> np.ndarray:
    """Koordinatlardan geçen yay boyu parametreli doğal kübik spline'ın yoğun örneği."""
    p = np.asarray(coords, float)
    keep = np.r_[True, np.linalg.norm(np.diff(p, axis=0), axis=1) > 1e-12]   # çift noktaları at
    p = p[keep]
    t = np.r_[0.0, np.cumsum(np.linalg.norm(np.diff(p, axis=0), axis=1))]
    mx, my = _natural_cubic_m(t, p[:, 0]), _natural_cubic_m(t, p[:, 1])
    tq = np.concatenate([np.linspace(t[k], t[k + 1], per_segment, endpoint=False) for k in range(len(t) - 1)] + [t[-1:]])
    return np.column_stack([_spline_eval(t, p[:, 0], mx, tq), _spline_eval(t, p[:, 1], my, tq)])


def _normalized_dense(coords: np.ndarray) -> tuple[np.ndarray, int]:
    d = _dense(coords)
    te = 0.5 * (d[0] + d[-1])
    r = np.linalg.norm(d - te, axis=1)
    i_le = int(np.argmax(r))
    le = d[i_le]
    if 0 < i_le < len(d) - 1:               # tepe konumunu komşu üç noktadan parabolle incelt
        den = r[i_le - 1] - 2 * r[i_le] + r[i_le + 1]
        f = float(np.clip(0.5 * (r[i_le - 1] - r[i_le + 1]) / den, -1, 1)) if abs(den) > 1e-15 else 0.0
        le = d[i_le] + f * (d[i_le + 1] - d[i_le]) if f > 0 else d[i_le] + f * (d[i_le] - d[i_le - 1])
    chord_vec = te - le
    chord = float(np.hypot(*chord_vec))
    ang = math.atan2(chord_vec[1], chord_vec[0])
    ca, sa = math.cos(-ang), math.sin(-ang)
    rot = np.array([[ca, -sa], [sa, ca]])
    out = (d - le) @ rot.T / chord
    out[i_le] = (0.0, 0.0)                  # yoğun örnekte LE noktası tam (0, 0) olsun (fark < 1e-6)
    return out, i_le


def normalize(coords: np.ndarray) -> np.ndarray:
    """Profili veter birimine taşır: hücum kenarı (0, 0), firar kenarı ortası (1, 0).

    Hücum kenarı, spline üzerinde firar kenarı ortasına en uzak noktadır (veter tanımı). Dönüş yoğun spline
    örneğidir (Selig sırası korunur); ``resample`` bunu ortak istasyonlara indirger.
    """
    return _normalized_dense(coords)[0]


def x_stations(n: int = DEFAULT_N) -> np.ndarray:
    """Kosinüs dağılımlı x istasyonları, 0 (LE) → 1 (TE); hücum ve firar kenarında sık."""
    return 0.5 * (1.0 - np.cos(np.linspace(0.0, math.pi, n)))


def resample(coords: np.ndarray, n: int = DEFAULT_N) -> np.ndarray:
    """Normalize edip üst ve alt yüzeyi aynı ``x_stations(n)`` istasyonlarında örnekler → (2n − 1, 2).

    Ortak parametreleme: i. üst nokta ile i. alt nokta aynı x'tedir; farklı profillerin aynı indisli
    noktaları aynı veter konumuna karşılık gelir (karıştırma ve loft için şart).
    """
    d, i_le = _normalized_dense(coords)
    up = d[:i_le + 1][::-1]                 # LE → TE
    lo = d[i_le:]
    x = x_stations(n)
    yu = _interp_monotone(up, x)
    yl = _interp_monotone(lo, x)
    yu[0] = yl[0] = 0.0                     # LE tam (0, 0)
    return from_surfaces(x, yu, yl)


def _interp_monotone(curve: np.ndarray, x: np.ndarray) -> np.ndarray:
    xs = np.maximum.accumulate(curve[:, 0])
    keep = np.r_[True, np.diff(xs) > 1e-12]
    xs, ys = xs[keep], curve[keep, 1]
    xs[0] = min(xs[0], 0.0)
    xs[-1] = max(xs[-1], 1.0)
    return np.interp(x, xs, ys)


# ---------------------------------------------------------------------------------------------------
# Örneklenmiş kesitler üzerinde işlemler (hepsi (2n − 1, 2) Selig dizisi alır ve döndürür)
# ---------------------------------------------------------------------------------------------------
def surfaces(coords: np.ndarray) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Örneklenmiş kesiti ``(x, y_üst, y_alt)`` dizilerine ayırır (her biri n uzunlukta, LE → TE)."""
    c = np.asarray(coords, float)
    if len(c) % 2 == 0:
        raise ValueError("örneklenmiş kesit 2n − 1 noktadan oluşmalı (resample kullanın)")
    n = (len(c) + 1) // 2
    up = c[:n][::-1]
    lo = c[n - 1:]
    if not np.allclose(up[:, 0], lo[:, 0], atol=1e-9):
        raise ValueError("üst ve alt yüzey aynı x istasyonlarında değil (resample kullanın)")
    return up[:, 0].copy(), up[:, 1].copy(), lo[:, 1].copy()


def from_surfaces(x: np.ndarray, yu: np.ndarray, yl: np.ndarray) -> np.ndarray:
    """``surfaces`` işleminin tersi: (x, y_üst, y_alt) → Selig sırasında (2n − 1, 2)."""
    up = np.column_stack([x, yu])[::-1]
    lo = np.column_stack([x, yl])[1:]
    return np.vstack([up, lo])


def blend(a: np.ndarray, b: np.ndarray, t: float) -> np.ndarray:
    """İki örneklenmiş kesitin doğrusal karışımı: ``t = 0`` → a, ``t = 1`` → b. Nokta sayıları eşit olmalı."""
    a = np.asarray(a, float)
    b = np.asarray(b, float)
    if a.shape != b.shape:
        raise ValueError(f"kesitler aynı nokta sayısında olmalı: {a.shape} ≠ {b.shape}")
    t = float(np.clip(t, 0.0, 1.0))
    return (1.0 - t) * a + t * b


def thickness_distribution(coords: np.ndarray) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """(x, kalınlık t(x), kamber y_c(x)) — kalınlık dikey (y_üst − y_alt), veter birimi."""
    x, yu, yl = surfaces(coords)
    return x, yu - yl, 0.5 * (yu + yl)


def max_thickness(coords: np.ndarray) -> tuple[float, float]:
    """En büyük kalınlık oranı ve konumu ``(t/c, x/c)`` — ince parabolik tepe ile."""
    x, t, _ = thickness_distribution(coords)
    i = int(np.argmax(t))
    if 0 < i < len(t) - 1:
        x0, x1, x2 = x[i - 1:i + 2]
        t0, t1, t2 = t[i - 1:i + 2]
        A = np.array([[x0 ** 2, x0, 1], [x1 ** 2, x1, 1], [x2 ** 2, x2, 1]])
        a2, a1, a0 = np.linalg.solve(A, [t0, t1, t2])
        if a2 < 0:
            xm = -a1 / (2 * a2)
            if x0 <= xm <= x2:
                return float(a2 * xm ** 2 + a1 * xm + a0), float(xm)
    return float(t[i]), float(x[i])


def max_camber(coords: np.ndarray) -> tuple[float, float]:
    """En büyük kamber ``(f/c, x/c)``."""
    x, _, c = thickness_distribution(coords)
    i = int(np.argmax(c))
    return float(c[i]), float(x[i])


def thickness_at(coords: np.ndarray, xc: float | np.ndarray) -> float | np.ndarray:
    """Veter konumunda (x/c) kalınlık oranı."""
    x, t, _ = thickness_distribution(coords)
    return np.interp(xc, x, t)


def camber_at(coords: np.ndarray, xc: float | np.ndarray) -> float | np.ndarray:
    """Veter konumunda (x/c) kamber çizgisi yüksekliği (kalınlığın ortası)."""
    x, _, c = thickness_distribution(coords)
    return np.interp(xc, x, c)


def surface_at(coords: np.ndarray, xc: float | np.ndarray, side: str = "upper") -> float | np.ndarray:
    """Veter konumunda üst (``"upper"``) ya da alt (``"lower"``) yüzey yüksekliği."""
    x, yu, yl = surfaces(coords)
    return np.interp(xc, x, yu if side == "upper" else yl)


def scale_thickness(coords: np.ndarray, factor: float) -> np.ndarray:
    """Kalınlığı ``factor`` ile ölçekler, kamber çizgisini korur (ör. SD7062 %14 × 1,107 = %15,5)."""
    x, yu, yl = surfaces(coords)
    c, h = 0.5 * (yu + yl), 0.5 * (yu - yl) * factor
    return from_surfaces(x, c + h, c - h)


def set_thickness(coords: np.ndarray, t_c: float) -> np.ndarray:
    """En büyük kalınlık oranını ``t_c`` yapar (``scale_thickness`` ile)."""
    t0, _ = max_thickness(coords)
    return scale_thickness(coords, t_c / t0)


def blunt_te(coords: np.ndarray, te_thickness: float, x_start: float = 0.55) -> np.ndarray:
    """Firar kenarını en az ``te_thickness`` (veter birimi) kalınlığa getirir.

    Baskı: 0,4 mm nozulla iki çevre ≈ 0,9 mm; kanatta 1,5 mm istenir (spec ``print``). Eksik kalınlık
    ``x_start``'tan TE'ye ``((x − x0)/(1 − x0))²`` ağırlığıyla kamber çizgisinin iki yanına eşit eklenir;
    ön bölge (D-kutusu, maksimum kalınlık) değişmez ve yüzey eğimi süreklidir.
    """
    x, yu, yl = surfaces(coords)
    deficit = te_thickness - (yu[-1] - yl[-1])
    if deficit <= 0:
        return np.array(coords, float)
    w = np.clip((x - x_start) / (1.0 - x_start), 0.0, 1.0) ** 2
    return from_surfaces(x, yu + 0.5 * deficit * w, yl - 0.5 * deficit * w)


def te_thickness(coords: np.ndarray) -> float:
    """Firar kenarı kalınlığı (veter birimi)."""
    _, yu, yl = surfaces(coords)
    return float(yu[-1] - yl[-1])


def le_radius(coords: np.ndarray) -> float:
    """Hücum kenarı yarıçapı tahmini (veter birimi): LE yakınında y² ≈ 2·r·x uydurması, iki yüzey ortalaması."""
    x, yu, yl = surfaces(coords)
    m = (x > 1e-5) & (x < 0.004)
    if m.sum() < 2:
        m = (x > 0) & (x <= np.sort(x)[min(5, len(x) - 1)])
    r_u = np.median(yu[m] ** 2 / (2 * x[m]))
    r_l = np.median(yl[m] ** 2 / (2 * x[m]))
    return float(0.5 * (r_u + r_l))


def area(coords: np.ndarray) -> float:
    """Kesit alanı (veter² birimi) — kütle ve hacim tahmini için."""
    x, yu, yl = surfaces(coords)
    return float(np.trapezoid(yu - yl, x)) if hasattr(np, "trapezoid") else float(np.trapz(yu - yl, x))


def perimeter(coords: np.ndarray) -> float:
    """Kesit çevre uzunluğu (veter birimi; kabuk kütlesi için)."""
    c = np.asarray(coords, float)
    return float(np.sum(np.linalg.norm(np.diff(c, axis=0), axis=1)) + np.linalg.norm(c[0] - c[-1]))


def write_dat(path: str | Path, coords: np.ndarray, name: str) -> None:
    """Selig biçiminde ``.dat`` yazar (XFLR5/XFOIL ile kontrol için)."""
    lines = [name] + [f"{x:10.6f} {y:10.6f}" for x, y in np.asarray(coords, float)]
    Path(path).write_text("\n".join(lines) + "\n", encoding="utf-8")
