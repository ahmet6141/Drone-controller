"""YELKOVAN YK-38 — 3B baskı hazırlığı: ``build_print_parts(bed=(256, 256, 256), out_dir=...)``.

Gövde nesnelerinden (``airframe.build()`` sonrası; değiştiriciler uygulanmış, dinlenme pozu) **baskıya hazır,
kapalı (manifold) katı parçalar** üretir, ``UCAV_Print`` koleksiyonuna koyar, ikili STL (mm) olarak dışa aktarır
ve Türkçe baskı raporu yazar (``print_report.md`` + ``print_report.json``).

Yöntem
------
1. **Kabuk**: kaynak katının köşeleri, bölge et kalınlığı kadar içe kaydırılır (açı ağırlıklı normal, eşit et
   düzeltmesi). Karşı yüze ışın atılarak yerel kalınlık ölçülür; ``2·et + 0,8 mm``'den ince bölgelerde (firar
   kenarı, oyuk dudakları) iç yüzeyler kasıtlı olarak çaprazlanır → EXACT boolean (öz-kesişim açık) bu bölgeleri
   **dolu** bırakır; 0,8 mm'den ince boşluk (dilimleyicide sorun çıkaran yarık) oluşmaz. Kanatta ana kirişin
   önü D-kutu (1,2 mm), arkası 0,6 mm; ``spec.yaml → print.walls_mm``.
2. **Segment**: her segment düzlemlerle sınırlı dışbükey bölgedir (kutu). ``A = dış ∩ bölge`` (EXACT),
   ``boşluk = iç ∩ bölge'`` (plaka uçlarında kaburga kalınlığı kadar içeride, diğer uçlarda dışarı taşar),
   ``boşluk −= iç yapılar`` (kılavuz kovanları, kesme ağları, ±40° geodezik kafes, pim göbekleri, flanşlar),
   ``parça = A − boşluk``, sonra eklenenler (menteşe dilleri) ``∪`` ve delikler ``−`` (boru kanalları, pim
   delikleri, hafifletme delikleri, servo yuvası, horn yarığı, menteşe pimi, çentikler).
3. **Uç türleri**: ``plate`` (kapalı kaburga, tablaya oturan yüz; hafifletme ve pim delikli), ``frame`` (gövde
   halkasında halka çerçeve), ``land`` (üst uçta 2,4 mm yapıştırma flanşı, açık), ``open``.
4. **Doğrulama**: bmesh ile manifold olmayan / sınır kenar = 0, yüz yönleri dışa (hacim > 0), minimum et
   (iç ve dış yüz arası ışın ölçümü), tablaya sığma (yazdırma yönü ve Z ekseni etrafında en iyi açı; sığmazsa
   segment otomatik ikiye bölünür), çıkıntı (>45°) alanı → destek kararı, hacim → kütle, kaba baskı süresi.
5. **Dışa aktarım**: ``ucav/out/stl/<parça>.stl`` (mm, baskı yönünde, tabla merkezinde, Z = 0); ayna L/R
   parçalar tek dosya (rapor notuyla). Basılmayanlar (CF borular, G10, motor, servo, takım…) BOM'da listelenir.

Parçalar (sol yarı; sağ = ayna): kanat dış paneli 7 segment + uç kapağı, kök bloğu 2 segment + glove strake,
kanatçık 4, dış flap 4, iç flap 1, stabilize yarısı 2, elevatör 2, dikey 2, dümen (tablaya göre 1–2), gövde
burun konisi + modül halkası + 6 halka + 3 kuyruk konisi halkası, burun modülü PETG flanşı, PA-CF kaporta
(üst + iki yanak) ve lüle halkası, hava alığı, kök kaportaları, takım kapakları, aviyonik kapağı (PETG), taret
yakası (PETG), PA-CF sürtünme pabucu. Kanat üstü ve stabilize kök filetoları sıfıra inen kama olduğundan basılmaz
(BOM: epoksi + mikrobalon dolgu).

Kullanım::

    import bpy
    from ucav.blender import airframe, printprep
    airframe.build()                                  # yoksa build_print_parts kendisi çağırır
    res = printprep.build_print_parts(bed=(256, 256, 256))
    # komut satırı:
    #   python3 -m ucav.blender.printprep --bed 256 256 256
    #   python3 -m ucav.blender.printprep --bed 220 220 250 --no-stl
    #   python3 -m ucav.blender.printprep --only wing_panel --preview /tmp/onizleme.png

Sayılar ``params`` / ``spec.yaml``'dan okunur; bu modüldeki küçük montaj payları ve tahmin katsayıları
``# varsayım`` olarak işaretlidir.
"""
from __future__ import annotations

import argparse
import faulthandler
import json
import math
import struct
import sys
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Callable, Iterable, Sequence

import bpy  # noqa: I001 — bmesh, bpy yüklenmeden içe aktarılamaz (python3 -m ile çalıştırma)
import bmesh
import numpy as np
from mathutils import Matrix, Vector
from mathutils.bvhtree import BVHTree

from .. import params as P
from . import util as U

# =====================================================================================================
# Ayarlar (spec → print) ve montaj payları
# =====================================================================================================
_PR = P.SPEC["print"]
_WALLS = {k: float(v) / 1000.0 for k, v in _PR["walls_mm"].items()}
_FIL = _PR["filaments"]
_ZONES = {z["zone"]: z for z in _PR["zones"]}

PRINT_COL = "UCAV_Print"
WORK_COL = "UCAV_PrintWork"
ROOT = "U_Root"
BED_DEFAULT = tuple(int(v) for v in _PR["bed_default_mm"])
BED_ALT = tuple(int(v) for v in _PR["bed_alt_mm"])
BED_MARGIN = float(_PR["bed_margin_mm"])            # mm, her kenarda
BED_MARGIN_MIN = 2.0                                # mm; altında "sığmaz" (dar payla basılabilir uyarısı). varsayım
Z_MARGIN = 2.0                                      # mm; Z yüksekliğinde pay. varsayım
OVERHANG_DEG = 55.0                                 # düşeyden bu açıdan yatık aşağı bakan yüzler çıkıntı sayılır. varsayım

RIB_T = _WALLS["segment_end_ribs_petg"]             # segment uç kaburgası (1,2 mm)
FRAME_T = _WALLS["frames_petg"]                     # gövde çerçevesi kalınlığı (1,6 mm)
FRAME_W = 0.006                                     # entegre halka çerçeve genişliği (deriden içe; R04: 8 → 6 mm). varsayım
LAND_T = 0.0020                                     # üst uç yapıştırma flanşı toplam et (deri dahil; R04: 2,4 → 2,0). varsayım
LAND_H = 0.006                                      # flanş yüksekliği. varsayım
SLEEVE_T = 0.0008                                   # boru kılavuz kovanı eti (2 çizgi). varsayım
SHOULDER = 0.002                                    # kovan kademesi / kör uç kapağı kalınlığı. varsayım
WEB_T = 0.0008                                      # kesme ağı (kiriş gövdesi) eti. varsayım
CS_WEB_T = _WALLS["control_surface_skin"]           # kumanda yüzeyi menteşe ağı: 1 çizgi (0,5 mm)
LATTICE_T = max(_WALLS["lattice"], 0.0006)          # geodezik iç kafes: en az 1 LW-PLA çizgisi (spec 0,6 mm; R14)
LATTICE_Y_MAX = 1.19                                # kafes yalnız dış panel 1–4'te (y < 1,20); 5–7 açıklık boyu ağlar +
                                                    # D-kutu (R04: düşük yüklü dış paneller, kafes ≈ 10 g/yan)
LATTICE_DEG = 40.0                                  # kafes ağları açıklık eksenine ±40° (dik baskıda yatayla 50°;
                                                    # 45°'lik 0,45 mm çizgi tam sınırda sarkıyordu — P12)
LATTICE_PITCH = 0.090                               # kafes aralığı: 0,6 mm üye ile 90 mm → kütle eski 0,45 mm × 70 mm kafesle
                                                    # aynı (spec yorumu "≈ 70 mm"; R04/R14)
PIN_D = 0.003                                       # hizalama pimi Ø3 CF çubuk. varsayım
PIN_DEPTH = 0.010                                   # pim deliği derinliği (her iki yanda). varsayım
BOSS_D = 0.0080                                     # pim göbeği çapı (Ø3,4 delik + 2 × 2,3 mm). varsayım

# ---------------------------------------------------------------- delik telafisi (P7)
# FDM delikleri, özellikle köpüren LW-PLA'da, 0,2–0,4 mm küçük basar; çokgen delik yazılı (inscribed) verilirse
# yarı boşluk da kaybolur. Bütün geçme delikleri ÇEVREL çokgendir (iç yarıçap = istenen yarıçap) ve kenar sayısı
# yarıçapa göre seçilir: n = clamp(round(2πr / 0,6 mm), 24, 96). Çap boşlukları malzemeye göredir (varsayım; önce
# tolerans kuponu basılır → ``tolerance_coupon.stl``).
HOLE_CLEAR = {                                      # (çap boşluğu m) — "lw": LW-PLA/LW-ASA, "hard": PETG/PA-CF/TPU
    "lw": {"tube": 0.0007, "pin": 0.0004, "bolt": 0.0005, "hinge": 0.00035},
    "hard": {"tube": 0.0006, "pin": 0.0004, "bolt": 0.0005, "hinge": 0.00035},
}
TUBE_CLEAR = HOLE_CLEAR["lw"]["tube"]               # rapor/ayar için (LW-PLA boru deliği Ø + 0,7 mm)
PIN_HOLE_D = PIN_D + HOLE_CLEAR["lw"]["pin"]        # Ø3,4
M4_HOLE_D = 0.0045                                  # M4 geçiş deliği
M3_HOLE_D = 0.0034                                  # M3 geçiş deliği
INSERT = {"M3": (0.0040, 0.0060), "M3S": (0.0040, 0.0035), "M4": (0.0056, 0.0085)}   # ısıl gömme dişli deliği
                                                    # (Ø, derinlik); M3S = M3×3 kısa — Ruthex tipi. varsayım
HINGE_PIN_D = 0.00175                               # menteşe pimi: Ø1,75 PETG filament (P1; spec Ø1,5 tel yerine). varsayım
HINGE_PIN_HOLE_D = 0.0021                           # Ø2,1 çevrel çokgen delik (iç yarıçap 1,05 mm)
HINGE_SLEEVE_R = 0.5 * HINGE_PIN_HOLE_D / math.cos(math.pi / 24) + _WALLS["hinge_knuckles"]
_CUR_MAT = ["LW-PLA"]                               # özellik üreticileri çalışırken parçanın filamenti (delik boşluğu için)


def _mat_class(material: str | None = None) -> str:
    m = (material or _CUR_MAT[0]).upper()
    return "lw" if m.startswith("LW") else "hard"


def hole_d(nominal: float, kind: str = "tube", material: str | None = None) -> float:
    """Geçme deliği çapı (m): nominal + malzeme/tür boşluğu (``HOLE_CLEAR``)."""
    return float(nominal) + HOLE_CLEAR[_mat_class(material)][kind]


def hole_sides(r: float) -> int:
    return int(min(96, max(24, round(2 * math.pi * r / 0.0006))))
LUG_W = 0.005                                       # basılı menteşe dili genişliği (eksen boyunca). varsayım
LUG_CLEAR = 0.0005                                  # dil–çentik eksenel boşluk (her yanda). varsayım
LUG_PITCH = 0.090                                   # dil aralığı (en fazla). varsayım
HINGE_ZONE_EXTRA = 0.0012                           # menteşe oyuğu dolu bölgesi: r_oyuk + et + 1,2 mm (dudak kaması; 0,6 mm
                                                    # denendi: bölge sınırı derilere teğet kalıp kıymık bırakıyor). varsayım
HORN_SLOT = (0.0018, 0.014)                         # G10 horn yarığı (kalınlık, veter boyu). varsayım
SERVO = {"name": "10 mm ince kanat servosu (KST X10 sınıfı)", "open": (0.036, 0.024), "box_t": 0.0012}   # varsayım
LIGHT_MARGIN = 0.0028                               # hafifletme deliği ile kenar/engel arası en az et. varsayım
LIGHT_R = (0.003, 0.022)                            # hafifletme deliği yarıçap aralığı. varsayım
NOSE_PIN_D = 0.004                                  # burun modülü hizalama pimi (çelik Ø4). varsayım
CYL_SEG = 28                                        # silindir kenar sayısı (STL boyutu / pürüzsüzlük dengesi)
SMALL_ISLAND_M3 = 2e-9                              # 2 mm³'ten küçük boolean kırıntıları atılır
FRAGMENT_REL = 0.06                                 # ana kabuğun %6'sından küçük kopuk dolu kabuk atılır
TINY_SHELL_M3 = 2e-8                                # |hacim| < 20 mm³ kabuk (sıfır hacimli kıymık / µ-boşluk) atılır
THIN_COVER_FACTOR = 2.5                             # kaplama ortalama kalınlığı ≤ 2,5 × et ise dolu basılır. varsayım
GLUE_GAP = 0.0002                                   # kaplama/fileto ile gövde/kanat arası yapıştırma boşluğu. varsayım
VOXEL_REPAIR_MAX_M3 = 80e-6                         # manifold olmayan ≤ 80 cm³ parçaya son çare voksel onarım
VOXEL_REPAIR_MAX_FACES = 60000                      # büyük ağda voksel onarım dakikalar sürer → atlanır
N_PERTURB = 6                                       # µm kaydırmalı deneme sayısı (k · 11–13 µm düzlem/özellik kaydırması)
PART_TIMEOUT_S = 600                                # tek parça kurulumu bu süreyi aşarsa süreç iz dökümüyle durur
WELD_DIST = 5e-6                                    # boolean kesişim köşelerini birleştirme (5 µm ≪ baskı çözünürlüğü)
PINCH_COLLAPSE = 0.0015                             # >2 yüzlü (sıkışma) kenarlar bu boydan kısaysa çökertilir (≤ 0,75 mm yer değişimi)
EPS = 1e-6

# Baskı süresi (kaba tahmin): hacimsel hız (cm³/h, parça hacmi) ve katman yüksekliği. varsayım
PRINT_RATE = {
    "LW-PLA": {"cm3_h": 24.0, "layer_mm": 0.30, "nozzle_c": "230–245", "bed_c": "50–60"},
    "LW-ASA": {"cm3_h": 22.0, "layer_mm": 0.30, "nozzle_c": "250–265", "bed_c": "95–105"},
    "PETG": {"cm3_h": 18.0, "layer_mm": 0.20, "nozzle_c": "235–245", "bed_c": "75–85"},
    "PA-CF": {"cm3_h": 15.0, "layer_mm": 0.20, "nozzle_c": "270–290 (sertleştirilmiş nozul)", "bed_c": "80–100"},
    "TPU": {"cm3_h": 7.0, "layer_mm": 0.20, "nozzle_c": "220–230", "bed_c": "40–50"},
    "PLA": {"cm3_h": 30.0, "layer_mm": 0.24, "nozzle_c": "200–215", "bed_c": "55–60"},
}
TOOLING_INFILL = 0.20                               # kalıp: 3 çevre + %15 dolgu ≈ hacmin %20'si. varsayım
LAYER_OVERHEAD_S = 2.5                              # katman başına seyir/geri çekme/Z hareketi (s). varsayım
PLATE_SETUP_H = 0.10                                # tabla başına ısınma/kalibrasyon (h). varsayım

# Basılı parçaların sahne malzemesi (sözleşme: params.MATERIALS adları; UCAV_Print render dışı)
FILAMENT_MATERIAL = {"LW-PLA": "UM_SkinBottom", "LW-ASA": "UM_SkinTop", "PA-CF": "UM_PACF", "PETG": "UM_Accent",
                     "TPU": "UM_TPU", "PETG-füme": "UM_SmokeHatch", "PLA": "UM_SkinBottom"}


# R04 hafifletme etleri spec'tedir (``print.zones``): burun ve halkalar 1–6 0,7 mm (+ 4 iç stringer,
# ``fus_stringer_prims``); kök kaportası ve ER-150 kabartması (yük taşımayan PA-CF kaplamalar) 1,2 mm.
def _zone(name: str) -> tuple[str, float]:
    """Spec bölgesi → (filament, et m)."""
    z = _ZONES[name]
    return str(z["material"]), float(z["wall_mm"]) / 1000.0


_EXTRA_FIL = {"PLA": {"density_g_cm3": 1.24, "tg_c": 60}}     # alet (kalıp) filamenti; spec'te yok. varsayım


def density(material: str) -> float:
    """Filament etkin yoğunluğu (g/cm³; LW-PLA köpürmüş etkin değer)."""
    key = "PETG" if material.startswith("PETG") else material
    return float((_FIL.get(key) or _EXTRA_FIL[key])["density_g_cm3"])


# =====================================================================================================
# Küçük numpy geometri: birim vektör, çerçeve, ilkel katılar (Blender dünya ekseni, metre)
# =====================================================================================================
def _unit(v) -> np.ndarray:
    v = np.asarray(v, float)
    n = float(np.linalg.norm(v))
    return v / n if n > 0 else v


def _perp_frame(axis) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """``axis`` (birim) için dik iki eksen (u, v) ve axis; sağ el."""
    w = _unit(axis)
    a = np.array([0.0, 0.0, 1.0]) if abs(w[2]) < 0.9 else np.array([1.0, 0.0, 0.0])
    u = _unit(np.cross(a, w))
    v = np.cross(w, u)
    return u, v, w


@dataclass
class Solid:
    """Kapalı numpy katı: ``V`` (n, 3) Blender dünya (m), ``F`` çokgen indis listeleri."""

    V: np.ndarray
    F: list
    name: str = "prim"

    def transformed(self, M: np.ndarray) -> "Solid":
        V = self.V @ M[:3, :3].T + M[:3, 3]
        F = self.F if np.linalg.det(M[:3, :3]) > 0 else [tuple(reversed(f)) for f in self.F]
        return Solid(V, F, self.name)

    def mirrored_y(self) -> "Solid":
        return self.transformed(np.diag([1.0, -1.0, 1.0, 1.0]))


def merge_solids(solids: Sequence[Solid], name: str = "merged") -> Solid:
    """Ayrık katıları tek ``Solid``'de toplar (köşe indisleri kaydırılır; boolean işleneni tek nesne olsun)."""
    V, F, off = [], [], 0
    for sl in solids:
        V.append(np.asarray(sl.V, float))
        F += [tuple(int(i) + off for i in f) for f in sl.F]
        off += len(sl.V)
    return Solid(np.vstack(V) if V else np.zeros((0, 3)), F, name)


def md_solid(md, name: str | None = None) -> Solid:
    """``shapes.MeshData`` (kapalı, dışa dönük) → Blender ekseninde ``Solid``."""
    return Solid(np.asarray(md.verts_blender(), float), [tuple(int(i) for i in f) for f in md.faces], name or md.name)


def prim_frustum(p0, p1, r0: float, r1: float | None = None, n: int = CYL_SEG, name: str = "cyl") -> Solid:
    """``p0 → p1`` eksenli kesik koni / silindir (``r1`` None → silindir). Uç yarıçapı 0 olabilir (koni)."""
    r1 = r0 if r1 is None else r1
    p0, p1 = np.asarray(p0, float), np.asarray(p1, float)
    u, v, w = _perp_frame(p1 - p0)
    a = np.linspace(0, 2 * math.pi, n, endpoint=False)
    ring = np.outer(np.cos(a), u) + np.outer(np.sin(a), v)
    V, F = [], []
    if r0 > 1e-9:
        V.append(p0 + r0 * ring)
    else:
        V.append(p0[None, :])
    if r1 > 1e-9:
        V.append(p1 + r1 * ring)
    else:
        V.append(p1[None, :])
    V = np.vstack(V)
    n0 = n if r0 > 1e-9 else 1
    if n0 == n and r1 > 1e-9:
        for i in range(n):
            j = (i + 1) % n
            F.append((i, j, n + j, n + i))
        F.append(tuple(range(n - 1, -1, -1)))
        F.append(tuple(range(n, 2 * n)))
    elif n0 == 1:                                   # tepe p0'da
        for i in range(n):
            j = (i + 1) % n
            F.append((0, 1 + j, 1 + i))
        F.append(tuple(range(1, n + 1)))
    else:                                           # tepe p1'de
        for i in range(n):
            j = (i + 1) % n
            F.append((i, j, n))
        F.append(tuple(range(n - 1, -1, -1)))
    return Solid(V, F, name)


def prim_hole(p0, p1, d: float, name: str = "hole", n: int | None = None) -> Solid:
    """Geçme deliği: iç yarıçapı tam ``d/2`` olan ÇEVREL çokgen silindir (P7). Kenar sayısı ``hole_sides``."""
    r = 0.5 * float(d)
    n = n or hole_sides(r)
    return prim_frustum(p0, p1, r / math.cos(math.pi / n), n=n, name=name)


def prim_box(center, axes, half, name: str = "box") -> Solid:
    """Yönlü kutu: ``axes`` sütunları (3×3) yerel X, Y, Z; ``half`` yarı boyutlar."""
    c = np.asarray(center, float)
    A = np.asarray(axes, float)
    h = np.asarray(half, float)
    V = np.array([c + A @ (h * np.array([sx, sy, sz])) for sx in (-1, 1) for sy in (-1, 1) for sz in (-1, 1)])
    F = [(0, 1, 3, 2), (4, 6, 7, 5), (0, 4, 5, 1), (2, 3, 7, 6), (0, 2, 6, 4), (1, 5, 7, 3)]
    s = Solid(V, F, name)
    return s if np.linalg.det(A) > 0 else Solid(V, [tuple(reversed(f)) for f in F], name)


def prim_prism(poly2, origin, eu, ev, ew, w0: float, w1: float, name: str = "prism") -> Solid:
    """2B çokgen (``poly2`` (k, 2), (eu, ev) düzleminde, saat yönü tersi) ``ew`` boyunca w0 → w1 ekstrüzyonu."""
    poly2 = np.asarray(poly2, float)
    k = len(poly2)
    o, eu, ev, ew = (np.asarray(x, float) for x in (origin, eu, ev, ew))
    base = o + np.outer(poly2[:, 0], eu) + np.outer(poly2[:, 1], ev)
    V = np.vstack([base + w0 * ew, base + w1 * ew])
    F = [tuple(range(k - 1, -1, -1)), tuple(range(k, 2 * k))]
    for i in range(k):
        j = (i + 1) % k
        F.append((i, j, k + j, k + i))
    s = Solid(V, F, name)
    if np.dot(np.cross(eu, ev), ew) < 0:
        s = Solid(V, [tuple(reversed(f)) for f in F], name)
    return s


# =====================================================================================================
# Blender ağ G/Ç, çalışma koleksiyonu, boolean
# =====================================================================================================
def _work_collection(scene: bpy.types.Scene) -> bpy.types.Collection:
    col = bpy.data.collections.get(WORK_COL)
    if col is None:
        col = bpy.data.collections.new(WORK_COL)
    if col.name not in scene.collection.children.keys():
        scene.collection.children.link(col)
    col.hide_render = True
    return col


def _new_mesh(name: str, V: np.ndarray, F: Sequence) -> bpy.types.Mesh:
    me = bpy.data.meshes.new(name)
    V = np.asarray(V, np.float64)
    totals = np.fromiter((len(f) for f in F), dtype=np.int32, count=len(F))
    starts = np.r_[0, np.cumsum(totals)[:-1]].astype(np.int32) if len(F) else np.zeros(0, np.int32)
    loops = np.fromiter((i for f in F for i in f), dtype=np.int32, count=int(totals.sum()))
    me.vertices.add(len(V))
    me.vertices.foreach_set("co", V.astype(np.float32).ravel())
    me.loops.add(len(loops))
    me.loops.foreach_set("vertex_index", loops)
    me.polygons.add(len(F))
    me.polygons.foreach_set("loop_start", starts)
    me.update(calc_edges=True)
    me.validate(clean_customdata=False)
    return me


def _obj(name: str, solid: Solid | tuple, col: bpy.types.Collection) -> bpy.types.Object:
    V, F = (solid.V, solid.F) if isinstance(solid, Solid) else solid
    ob = bpy.data.objects.new(name, _new_mesh(name, V, F))
    col.objects.link(ob)
    return ob


def _mesh_np(me: bpy.types.Mesh, M: Matrix | None = None) -> tuple[np.ndarray, list]:
    V = np.zeros(len(me.vertices) * 3)
    me.vertices.foreach_get("co", V)
    V = V.reshape(-1, 3)
    if M is not None:
        A = np.array(M)
        V = V @ A[:3, :3].T + A[:3, 3]
    ls = np.zeros(len(me.polygons), np.int32)
    lt = np.zeros(len(me.polygons), np.int32)
    me.polygons.foreach_get("loop_start", ls)
    me.polygons.foreach_get("loop_total", lt)
    vi = np.zeros(len(me.loops), np.int32)
    me.loops.foreach_get("vertex_index", vi)
    F = [tuple(vi[a:a + b]) for a, b in zip(ls, lt)]
    return V, F


def _delete(ob) -> None:
    if ob is None:
        return
    me = ob.data
    bpy.data.objects.remove(ob, do_unlink=True)
    if isinstance(me, bpy.types.Mesh) and me.users == 0:
        bpy.data.meshes.remove(me)


def rest_matrix(ob: bpy.types.Object) -> Matrix:
    """Dinlenme pozu dünya matrisi: sürülen ``rotation_euler`` (kumanda/kapak/takım) yok sayılır, ``U_Root``
    tasarım CG'sinde kabul edilir (animasyon etkisiz). Yerel eksenler ``delta_rotation_euler``'dedir (sözleşme)."""
    if ob.name == ROOT:
        return Matrix.Translation(Vector(P.U_ROOT_B))
    rot = ob.delta_rotation_euler.to_matrix().to_4x4()
    sc = Matrix.Diagonal((*[a * b for a, b in zip(ob.scale, ob.delta_scale)], 1.0))
    basis = Matrix.Translation(ob.location + ob.delta_location) @ rot @ sc
    if ob.parent is not None:
        return rest_matrix(ob.parent) @ ob.matrix_parent_inverse @ basis
    return basis


def source_solid(name: str) -> Solid:
    """Sahnedeki nesnenin değerlendirilmiş (değiştiriciler uygulanmış) ağı, dinlenme pozunda dünya koordinatında."""
    ob = bpy.data.objects[name]
    dg = bpy.context.evaluated_depsgraph_get()
    ev = ob.evaluated_get(dg)
    me = ev.to_mesh()
    try:
        V, F = _mesh_np(me, rest_matrix(ob))
    finally:
        ev.to_mesh_clear()
    return Solid(V, F, name)


class BoolError(RuntimeError):
    pass


class Booler:
    """Boolean yürütücüsü (operatör yerine değiştirici + ``new_from_object``; nesne verisi yerinde değişir).

    Çözücü seçimi: kaynağın segment kutusuyla kesimi **EXACT** (temiz, kapalı girişte kesin aritmetik);
    kabuk/boşluk/özellik işlemleri **MANIFOLD** (Blender 4.5). İçe kaydırılmış yüzey ince bölgelerde kasıtlı
    öz-kesişimlidir; EXACT bu girdide (öz-kesişim açıkken bile) manifold olmayan/boş çıktı verdiği için bu
    adımlarda MANIFOLD kullanılır — her çıktı ``mesh_check`` ile doğrulanır.
    """

    def __init__(self, col: bpy.types.Collection, solver: str = "MANIFOLD"):
        self.col = col
        self.solver = solver
        self.n_ops = 0
        self.t_ops = 0.0
        self.debug = bool(int(__import__("os").environ.get("UCAV_PP_DEBUG", "0") or 0))

    def op(self, target: bpy.types.Object, operand, operation: str, self_: bool = False,
           solver: str | None = None) -> bpy.types.Object:
        t0 = time.time()
        solver = solver or self.solver
        if solver == "MANIFOLD":
            # MANIFOLD çözücü manifold olmayan girdiyi sessizce reddeder (sonuç = girdi). EXACT kesimlerin bıraktığı
            # 5 µm kırıntı/sınır kenarlarını önce temizle. (Kaydırılmış boşluk yüzeyi kasıtlı öz-kesişimlidir; EXACT'a
            # geçilmez — reddedilen işlem build_segment'teki hacim denetimiyle yakalanır ve kaydırmalı yeniden denenir.)
            if not mesh_check(target.data)["ok"]:
                clean_mesh(target, triangles=True)
            if isinstance(operand, bpy.types.Object) and not mesh_check(operand.data)["ok"]:
                clean_mesh(operand, triangles=True)
        mod = target.modifiers.new("pp_bool", "BOOLEAN")
        mod.operation = operation
        mod.solver = solver
        if isinstance(operand, bpy.types.Collection):
            mod.operand_type = "COLLECTION"
            mod.collection = operand
        else:
            mod.object = operand
        if mod.solver == "EXACT":
            mod.use_self = bool(self_)
            mod.use_hole_tolerant = False
        dg = bpy.context.evaluated_depsgraph_get()
        dg.update()
        ev = target.evaluated_get(dg)
        new = bpy.data.meshes.new_from_object(ev, depsgraph=dg)
        target.modifiers.remove(mod)
        old = target.data
        target.data = new
        if old.users == 0:
            bpy.data.meshes.remove(old)
        self.n_ops += 1
        self.t_ops += time.time() - t0
        if self.debug:
            c = mesh_check(target.data)
            on = operand.name if hasattr(operand, "name") else str(operand)
            print(f"   [bool] {target.name} {operation} {on} ({solver}) → faces {c['faces']} vol "
                  f"{c['volume_m3'] * 1e6:.2f} cm³ bnd {c['boundary']} nm {c['nonmanifold']} ok {c['ok']}", flush=True)
        return target

    def safe(self, target: bpy.types.Object, operand, operation: str, first: str, second: str,
             min_volume: float = 0.0) -> bpy.types.Object:
        """Önce ``first`` çözücüyle dener; sonuç boş/manifold değil/beklenenden küçükse ``second`` ile yeniden."""
        backup = target.data.copy()
        self.op(target, operand, operation, solver=first)
        chk = mesh_check(target.data)
        if not chk["ok"]:
            clean_mesh(target, triangles=True)
            chk = mesh_check(target.data)
        if chk["ok"] and chk["volume_m3"] > min_volume:
            bpy.data.meshes.remove(backup)
            return target
        bad = target.data
        target.data = backup
        bpy.data.meshes.remove(bad)
        return self.op(target, operand, operation, solver=second)

    def group(self, name: str, solids: Iterable[Solid]) -> bpy.types.Collection | None:
        """İlkel katılar → geçici koleksiyon (boolean işleneni). Boşsa None."""
        solids = [s for s in solids if s is not None and len(s.F)]
        if not solids:
            return None
        col = bpy.data.collections.new(name)
        self.col.children.link(col)
        for i, s in enumerate(solids):
            _obj(f"{name}_{i:02d}", s, col)
        return col

    @staticmethod
    def drop(col: bpy.types.Collection | None) -> None:
        if col is None:
            return
        for ob in list(col.objects):
            _delete(ob)
        bpy.data.collections.remove(col)


# =====================================================================================================
# Ağ denetimi ve temizlik (bmesh)
# =====================================================================================================
def mesh_check(me: bpy.types.Mesh) -> dict:
    """Manifold denetimi: sınır kenar, manifold olmayan kenar, tutarsız yönlü kenar, hacim, kabuk sayısı."""
    bm = bmesh.new()
    bm.from_mesh(me)
    bnd = sum(1 for e in bm.edges if e.is_boundary)
    nonman = sum(1 for e in bm.edges if not e.is_manifold and not e.is_boundary)
    wire = sum(1 for e in bm.edges if e.is_wire)
    flipped = 0
    for e in bm.edges:
        if len(e.link_loops) == 2:
            l1, l2 = e.link_loops
            if l1.vert == l2.vert:                      # aynı yönde iki kez kullanılan kenar → yön tutarsız
                flipped += 1
    vol = bm.calc_volume(signed=True)
    shells = _count_shells(bm)
    out = {"verts": len(bm.verts), "faces": len(bm.faces), "boundary": bnd, "nonmanifold": nonman, "wire": wire,
           "flipped": flipped, "volume_m3": float(vol), "shells": shells}
    bm.free()
    out["ok"] = (bnd == 0 and nonman == 0 and wire == 0 and flipped == 0 and vol > 0 and out["faces"] > 0)
    return out


def _count_shells(bm: bmesh.types.BMesh) -> int:
    seen = set()
    n = 0
    for f in bm.faces:
        if f.index in seen:
            continue
        n += 1
        stack = [f]
        seen.add(f.index)
        while stack:
            g = stack.pop()
            for e in g.edges:
                for h in e.link_faces:
                    if h.index not in seen:
                        seen.add(h.index)
                        stack.append(h)
    return n


def clean_mesh(ob: bpy.types.Object, triangles: bool = False) -> dict:
    """Boolean sonrası temizlik: 5 µm'den yakın köşeleri birleştir (kesişim kırıntıları), dejenere yüzleri erit,
    2 mm³'ten küçük kırıntı kabukları at. Yüz yönleri değiştirilmez (iç boşluk kabukları içe bakar). Atılan
    kabuk sayısını döndürür."""
    bm = bmesh.new()
    bm.from_mesh(ob.data)
    if triangles:                                       # önce üçgenle (birleştirme anahtar delikli n-gen üretmesin)
        bmesh.ops.triangulate(bm, faces=bm.faces[:], quad_method="FIXED", ngon_method="EAR_CLIP")
    bmesh.ops.remove_doubles(bm, verts=bm.verts[:], dist=WELD_DIST)
    bmesh.ops.dissolve_degenerate(bm, edges=bm.edges[:], dist=WELD_DIST)
    if triangles:
        _drop_fin_faces(bm)
    wires = [e for e in bm.edges if not e.link_faces]
    if wires:
        bmesh.ops.delete(bm, geom=wires, context="EDGES")
    loose = [v for v in bm.verts if not v.link_faces]
    if loose:
        bmesh.ops.delete(bm, geom=loose, context="VERTS")
    removed = _drop_islands(bm)
    repaired = 0
    if triangles:                                       # son hâl: üçgen ağ (STL ile birebir aynı topoloji)
        bmesh.ops.triangulate(bm, faces=[f for f in bm.faces if len(f.verts) > 3], quad_method="FIXED",
                              ngon_method="EAR_CLIP")
        repaired += _drop_fin_faces(bm)
        for _ in range(6):                              # sıkışma kenarlarını (>2 yüz, ≤ 1,5 mm) çökert
            bad = [e for e in bm.edges if len(e.link_faces) > 2 and e.calc_length() < PINCH_COLLAPSE]
            if not bad:
                break
            bmesh.ops.collapse(bm, edges=bad, uvs=False)
            bmesh.ops.remove_doubles(bm, verts=bm.verts[:], dist=WELD_DIST)
            bmesh.ops.dissolve_degenerate(bm, edges=bm.edges[:], dist=WELD_DIST)
            bmesh.ops.triangulate(bm, faces=[f for f in bm.faces if len(f.verts) > 3])
            repaired += _drop_fin_faces(bm) + len(bad)
        bnd = [e for e in bm.edges if e.is_boundary]
        if bnd and len(bnd) <= 24:                      # çökertmeden kalan küçük delikleri kapat
            res = bmesh.ops.holes_fill(bm, edges=bnd, sides=12)
            bmesh.ops.triangulate(bm, faces=[f for f in res["faces"] if len(f.verts) > 3])
            repaired += len(res["faces"])
        removed += _drop_islands(bm)                    # onarım artıkları (tel kenar, tek üçgen kırıntısı)
    bm.to_mesh(ob.data)
    bm.free()
    ob.data.update()
    return {"removed_islands": removed, "repaired": repaired}


def finalize_mesh(ob: bpy.types.Object) -> dict:
    """Son temizlik, ZARARSIZ: parça zaten kapalıysa yalnız üçgenlenir (gerekirse) ve kırıntı kabuklar atılır —
    5 µm kaynak/çökertme yapılmaz (MANIFOLD çıktısındaki µm ayrık yüzeyleri birleştirip sıkışma kenarı
    üretebiliyordu). Kapalı değilse tam ``clean_mesh``; sonuç daha kötüyse eski ağ geri gelir."""
    chk0 = mesh_check(ob.data)
    tri = all(len(p.vertices) == 3 for p in ob.data.polygons)
    if chk0["ok"]:
        keep = ob.data.copy()
        bm = _bm_triangulated(ob.data) if not tri else bmesh.new()
        if tri:
            bm.from_mesh(ob.data)
        removed = _drop_islands(bm)
        bm.to_mesh(ob.data)
        bm.free()
        ob.data.update()
        if mesh_check(ob.data)["ok"]:
            bpy.data.meshes.remove(keep)
            _micro_weld(ob)
            return {"removed_islands": removed, "repaired": 0}
        bad = ob.data                                   # üçgenleme bozdu (kendine değen n-gen): tam temizlik
        ob.data = keep
        bpy.data.meshes.remove(bad)
    keep = ob.data.copy()
    res = clean_mesh(ob, triangles=True)
    chk1 = mesh_check(ob.data)
    bad0 = chk0["boundary"] + chk0["nonmanifold"] + chk0["flipped"]
    bad1 = chk1["boundary"] + chk1["nonmanifold"] + chk1["flipped"]
    if not chk1["ok"] and bad1 >= bad0 and chk0["volume_m3"] > 0:
        bad = ob.data
        ob.data = keep
        bpy.data.meshes.remove(bad)
    else:
        bpy.data.meshes.remove(keep)
    return res


MICRO_WELD = 1e-6                                   # 1 µm: STL'de float32 / dilimleyici kaynağıyla birleşecek köşeler


def _micro_weld(ob: bpy.types.Object, dist: float = MICRO_WELD) -> bool:
    """1 µm'den yakın ayrık köşeleri (EXACT kesişim noktalarının µm altı kopyaları) birleştirir, dejenere üçgenleri
    eritir. Dilimleyici / denetçi STL'yi 0,1 µm kaynakla okuduğunda çift kenar kalmaz. Ağ kapalılığı bozulursa geri
    alınır (False)."""
    keep = ob.data.copy()
    bm = bmesh.new()
    bm.from_mesh(ob.data)
    n0 = len(bm.verts)
    bmesh.ops.remove_doubles(bm, verts=bm.verts[:], dist=dist)
    if len(bm.verts) == n0:
        bm.free()
        bpy.data.meshes.remove(keep)
        return True
    bmesh.ops.dissolve_degenerate(bm, edges=bm.edges[:], dist=dist)
    bmesh.ops.triangulate(bm, faces=[f for f in bm.faces if len(f.verts) > 3], quad_method="BEAUTY",
                          ngon_method="EAR_CLIP")
    _drop_fin_faces(bm)
    loose = [v for v in bm.verts if not v.link_faces]
    if loose:
        bmesh.ops.delete(bm, geom=loose, context="VERTS")
    bm.to_mesh(ob.data)
    bm.free()
    ob.data.update()
    c = mesh_check(ob.data)
    c0 = mesh_check(keep)
    if c["ok"] and abs(c["volume_m3"] - c0["volume_m3"]) <= 1e-3 * abs(c0["volume_m3"]) + 1e-12:
        bpy.data.meshes.remove(keep)
        return True
    bad = ob.data
    ob.data = keep
    bpy.data.meshes.remove(bad)
    return False


def _drop_islands(bm: bmesh.types.BMesh) -> int:
    """Tel kenarları/boş köşeleri siler; hacmi 2 mm³'ten küçük ya da açık ve 50 mm²'den küçük kabukları atar."""
    wires = [e for e in bm.edges if not e.link_faces]
    if wires:
        bmesh.ops.delete(bm, geom=wires, context="EDGES")
    bm.faces.ensure_lookup_table()
    for f in bm.faces:
        f.tag = False
    removed = 0
    shells = []
    for f in bm.faces:
        if f.tag:
            continue
        stack, sh = [f], []
        f.tag = True
        while stack:
            g = stack.pop()
            sh.append(g)
            for e in g.edges:
                for h in e.link_faces:
                    if not h.tag:
                        h.tag = True
                        stack.append(h)
        shells.append(sh)
    for sh in shells:
        vol = 0.0
        area = 0.0
        for g in sh:
            vs = [v.co for v in g.verts]
            area += g.calc_area()
            for i in range(1, len(vs) - 1):
                vol += vs[0].dot(vs[i].cross(vs[i + 1])) / 6.0
        open_ = any(e.is_boundary for g in sh for e in g.edges)
        if abs(vol) < SMALL_ISLAND_M3 or (open_ and area < 5e-5):     # kırıntı ya da açık küçük parça
            bmesh.ops.delete(bm, geom=sh, context="FACES")
            removed += 1
    loose = [v for v in bm.verts if not v.link_faces]
    if loose:
        bmesh.ops.delete(bm, geom=loose, context="VERTS")
    return removed


def _drop_fin_faces(bm: bmesh.types.BMesh) -> int:
    """Aynı köşe kümesine sahip yüz çiftlerini (sıfır hacimli yüzgeç) siler."""
    seen: dict = {}
    dup = []
    bm.verts.index_update()
    for f in bm.faces:
        k = tuple(sorted(v.index for v in f.verts))
        if k in seen:
            dup += [f, seen.pop(k)]
        else:
            seen[k] = f
    if dup:
        bmesh.ops.delete(bm, geom=list(set(dup)), context="FACES_ONLY")
        wires = [e for e in bm.edges if not e.link_faces]
        if wires:
            bmesh.ops.delete(bm, geom=wires, context="EDGES")
        loose = [v for v in bm.verts if not v.link_faces]
        if loose:
            bmesh.ops.delete(bm, geom=loose, context="VERTS")
    return len(dup)


def voxel_repair(ob: bpy.types.Object, voxel: float = 0.00015, max_faces: int = 25000) -> dict:
    """Son çare onarım (küçük parçalar): voksel yeniden ağlama (OpenVDB, her zaman kapalı/manifold) + sadeleştirme.
    Et doğruluğu ≈ ±voksel/2."""
    m = ob.modifiers.new("pp_vox", "REMESH")
    m.mode = "VOXEL"
    m.voxel_size = voxel
    m.adaptivity = 0.0
    dg = bpy.context.evaluated_depsgraph_get()
    dg.update()
    me = bpy.data.meshes.new_from_object(ob.evaluated_get(dg), depsgraph=dg)
    ob.modifiers.remove(m)
    old = ob.data
    ob.data = me
    n = len(me.polygons)
    if n > max_faces:
        d = ob.modifiers.new("pp_dec", "DECIMATE")
        d.decimate_type = "COLLAPSE"
        d.ratio = max_faces / n
        dg = bpy.context.evaluated_depsgraph_get()
        dg.update()
        me2 = bpy.data.meshes.new_from_object(ob.evaluated_get(dg), depsgraph=dg)
        ob.modifiers.remove(d)
        ob.data = me2
        bpy.data.meshes.remove(me)
    for mat in old.materials:
        ob.data.materials.append(mat)
    if old.users == 0:
        bpy.data.meshes.remove(old)
    clean_mesh(ob, triangles=True)
    return {"voxel_mm": voxel * 1000.0, "faces_remesh": n}


def drop_fragments(ob: bpy.types.Object, rel: float = FRAGMENT_REL) -> tuple[int, float]:
    """Ana katıdan kopuk küçük dolu kabukları siler (iç boşluk kabukları — negatif hacim — korunur).
    Kesim/çıkarma sonrası tablada tek başına kalacak iğne/kıymık parçalar içindir → (atılan sayı, hacim m³)."""
    bm = bmesh.new()
    bm.from_mesh(ob.data)
    for f in bm.faces:
        f.tag = False
    shells = []
    for f in bm.faces:
        if f.tag:
            continue
        stack, sh = [f], []
        f.tag = True
        while stack:
            g = stack.pop()
            sh.append(g)
            for e in g.edges:
                for h in e.link_faces:
                    if not h.tag:
                        h.tag = True
                        stack.append(h)
        vol = sum(g.verts[0].co.dot(g.verts[i].co.cross(g.verts[i + 1].co)) / 6.0
                  for g in sh for i in range(1, len(g.verts) - 1))
        shells.append((vol, sh))
    vmax = max((v for v, _ in shells), default=0.0)
    drop = [(v, sh) for v, sh in shells if (0.0 < v < rel * vmax) or abs(v) < TINY_SHELL_M3]   # kıymık + µ-boşluk
    if drop:
        bmesh.ops.delete(bm, geom=[g for _, sh in drop for g in sh], context="FACES")
        loose = [v for v in bm.verts if not v.link_faces]
        if loose:
            bmesh.ops.delete(bm, geom=loose, context="VERTS")
        bm.to_mesh(ob.data)
        ob.data.update()
    bm.free()
    return len(drop), float(sum(v for v, _ in drop))


def split_shells(ob: bpy.types.Object) -> list[tuple[np.ndarray, list]]:
    """Ağı bağlantılı kabuklarına ayırır → [(V, F)…] (büyükten küçüğe hacim sırası)."""
    V, F = _mesh_np(ob.data)
    n = len(V)
    parent = np.arange(n)

    def find(a):
        while parent[a] != a:
            parent[a] = parent[parent[a]]
            a = parent[a]
        return a

    for f in F:
        r0 = find(f[0])
        for i in f[1:]:
            r = find(i)
            if r != r0:
                parent[r] = r0
    roots = np.array([find(i) for i in range(n)])
    groups: dict[int, list] = {}
    for f in F:
        groups.setdefault(int(roots[f[0]]), []).append(f)
    out = []
    for r, fs in groups.items():
        idx = sorted({i for f in fs for i in f})
        remap = {o: k for k, o in enumerate(idx)}
        Vs = V[idx]
        Fs = [tuple(remap[i] for i in f) for f in fs]
        out.append((Vs, Fs, abs(_signed_volume(Vs, Fs))))
    out.sort(key=lambda t: -t[2])
    return [(a, b) for a, b, _ in out]


def _signed_volume(V: np.ndarray, F: Sequence) -> float:
    T = _triangulate_fan(F)
    if len(T) == 0:
        return 0.0
    a, b, c = V[T[:, 0]], V[T[:, 1]], V[T[:, 2]]
    return float(np.einsum("ij,ij->i", a, np.cross(b, c)).sum() / 6.0)


def _triangulate_fan(F: Sequence) -> np.ndarray:
    tris = [(f[0], f[i], f[i + 1]) for f in F for i in range(1, len(f) - 1)]
    return np.asarray(tris, np.int64).reshape(-1, 3)


def _bm_triangulated(me: bpy.types.Mesh) -> bmesh.types.BMesh:
    """``me``'nin üçgenlenmiş bmesh'i. Bir n-genin köşegeni komşu yüzün kenarıyla / köşegeniyle çakışırsa kenar başına
    > 2 yüz olur (STL'de manifold olmayan kenar). Sırayla denenir: ear-clip, beauty, çakışan n-genlerden küçükleri /
    hepsi merkezden yelpaze (poke). Kabul: çakışma yok VE toplam alan korunmuş (yelpaze içbükey n-gende üst üste
    üçgen üretirse alan büyür → ret). Hiçbiri tutmazsa en az çakışan döner."""
    area0 = sum(p.area for p in me.polygons)

    def bad_of(bm):
        return {tuple(sorted((e.verts[0].index, e.verts[1].index))) for e in bm.edges if len(e.link_faces) > 2}

    def attempt(method, bad=None, keep_big=True):
        bm = bmesh.new()
        bm.from_mesh(me)
        bm.verts.ensure_lookup_table()
        if bad:
            poke = set()
            for a_, b_ in bad:
                fs = [f for f in set(bm.verts[a_].link_faces) & set(bm.verts[b_].link_faces) if len(f.verts) > 3]
                fs.sort(key=lambda f: f.calc_area())
                poke.update(fs[:-1] if keep_big else fs)
            if poke:
                bmesh.ops.poke(bm, faces=list(poke))
        bmesh.ops.triangulate(bm, faces=[f for f in bm.faces if len(f.verts) > 3], quad_method="BEAUTY",
                              ngon_method=method)
        err = abs(sum(f.calc_area() for f in bm.faces) - area0) / max(area0, 1e-12)
        return bm, bad_of(bm), err

    bm, bad0, err = attempt("EAR_CLIP")
    if not bad0:
        return bm
    best = (bm, bad0, err)
    for method, bad, keep_big in (("BEAUTY", None, True), ("EAR_CLIP", bad0, True), ("BEAUTY", bad0, True),
                                  ("EAR_CLIP", bad0, False), ("BEAUTY", bad0, False)):
        bm, bad, err = attempt(method, bad, keep_big)
        if not bad and err < 1e-6:
            best[0].free()
            return bm
        if (len(bad), err) < (len(best[1]), best[2]):
            best[0].free()
            best = (bm, bad, err)
        else:
            bm.free()
    return best[0]


def triangulate(me: bpy.types.Mesh) -> tuple[np.ndarray, np.ndarray]:
    """Doğru (içbükey n-gen güvenli) üçgenleme: bmesh ``triangulate`` (ear-clip) → (V, T). Kendine değen bir n-genin
    köşegeni komşu yüzün kenarıyla çakışırsa (kenar başına 4 yüz → STL'de manifold olmayan kenar) o n-genler
    merkezden yelpaze (``poke``) ile yeniden üçgenlenir."""
    bm = _bm_triangulated(me)
    bm.verts.index_update()
    V = np.array([v.co[:] for v in bm.verts], float)
    T = np.array([[v.index for v in f.verts] for f in bm.faces], np.int64).reshape(-1, 3)
    bm.free()
    return V, _drop_fins(T)


def _drop_fins(T: np.ndarray) -> np.ndarray:
    """Aynı üç köşeli, ters yönlü üçgen çiftlerini (burulmuş n-gen üçgenlemesinin sıfır hacimli "yüzgeci") atar;
    kenar eşleşmesi korunur."""
    if len(T) == 0:
        return T
    key = np.sort(T, axis=1)
    _, inv, cnt = np.unique(key, axis=0, return_inverse=True, return_counts=True)
    dup = cnt[inv.ravel()] > 1
    return T[~dup] if dup.any() else T


# =====================================================================================================
# İçe kaydırma (kabuk iç yüzü) — ışınla kalınlık sınırlı
# =====================================================================================================
def _angles(p, q, r) -> np.ndarray:
    u, v = q - p, r - p
    cu = np.einsum("ij,ij->i", u, v) / np.maximum(np.linalg.norm(u, axis=1) * np.linalg.norm(v, axis=1), 1e-18)
    return np.arccos(np.clip(cu, -1, 1))


def offset_vertices(V: np.ndarray, T: np.ndarray, wall, *, gap_min: float = 0.0008, cross: float = 0.0001,
                    lam: float = 0.02, max_fac: float = 10.0) -> np.ndarray:
    """Üçgen ağın köşelerini içe kaydırır (kabuk iç yüzü). ``wall`` sabit ya da köşe başına dizi (m).

    Köşe yer değiştirmesi, komşu yüz düzlemlerinin ``wall`` kadar içe kaydırılmış hâllerine açı ağırlıklı en küçük
    kareler uzaklığıyla bulunur (``(Σ aᵢ nᵢnᵢᵀ + λW I) d = −w Σ aᵢ nᵢ − λW w N``): düzlemsel n-genler düzlemsel
    kalır, keskin kenarlarda tam gönye oluşur. Yer değiştirme yönünde karşı yüze ışın atılır; yerel kalınlık
    ``2·et + gap_min``'den küçükse iç yüzeyler kasıtlı çaprazlanır (``0,5·kalınlık + cross``) → boolean bu
    bölgeyi dolu sayar (firar kenarı, dudak). Negatif ``wall`` → dışa kaydırma (ışın sınırı yok).
    """
    n = len(V)
    a, b, c = V[T[:, 0]], V[T[:, 1]], V[T[:, 2]]
    fn = np.cross(b - a, c - a)
    ln = np.linalg.norm(fn, axis=1)
    ok = ln > 1e-16
    fn[ok] /= ln[ok, None]
    A = np.zeros((n, 3, 3))
    N = np.zeros((n, 3))
    Wt = np.zeros(n)
    nn = np.einsum("ij,ik->ijk", fn, fn)
    for k, (p, q, r) in enumerate(((a, b, c), (b, c, a), (c, a, b))):
        wgt = _angles(p, q, r) * ok
        np.add.at(A, T[:, k], wgt[:, None, None] * nn)
        np.add.at(N, T[:, k], fn * wgt[:, None])
        np.add.at(Wt, T[:, k], wgt)
    w = np.broadcast_to(np.asarray(wall, float), (n,)).copy()
    sgn = np.sign(w)
    w_abs = np.abs(w)
    Nn = N / np.maximum(np.linalg.norm(N, axis=1), 1e-18)[:, None]
    lamW = lam * np.maximum(Wt, 1e-9)
    M = A + lamW[:, None, None] * np.eye(3)[None]
    rhs = -w_abs[:, None] * N - (lamW * w_abs)[:, None] * Nn
    d = np.linalg.solve(M, rhs[..., None])[..., 0]
    off = np.linalg.norm(d, axis=1)
    dirn = d / np.maximum(off, 1e-18)[:, None]
    off = np.minimum(off, max_fac * w_abs)
    out = np.where(sgn[:, None] < 0, V - dirn * off[:, None], V)
    inward = sgn > 0
    if inward.any():
        bvh = BVHTree.FromPolygons(V.tolist(), T.tolist(), epsilon=0.0)
        dist = np.full(n, 1.0)
        for i in np.flatnonzero(inward):
            hit = bvh.ray_cast(Vector(V[i] + dirn[i] * 2e-6), Vector(dirn[i]), 0.2)
            if hit[0] is not None:
                dist[i] = hit[3] + 2e-6
        thin = inward & (dist < 2.0 * off + gap_min)
        off = np.where(thin, 0.5 * dist + cross, off)
        out = np.where(inward[:, None], V + dirn * off[:, None], out)
    return out


def solid_triangles(solid: "Solid", ctx_col: bpy.types.Collection | None = None) -> tuple[np.ndarray, np.ndarray]:
    """Katının doğru üçgenlemesi (içbükey n-gen güvenli; köşe indisleri korunur)."""
    me = _new_mesh("PPW_tri", solid.V, solid.F)
    try:
        V, T = triangulate(me)
    finally:
        bpy.data.meshes.remove(me)
    return V, T


def min_wall_mm(ob: bpy.types.Object, n_samples: int = 1500, seed: int = 7) -> dict:
    """Et ölçümü: rastgele yüz örneklerinden içe ışın → karşı yüze mesafe (mm). En küçük ve %5'lik değer."""
    V, T = triangulate(ob.data)
    if len(T) == 0:
        return {"min_mm": 0.0, "p05_mm": 0.0}
    a, b, c = V[T[:, 0]], V[T[:, 1]], V[T[:, 2]]
    fn = np.cross(b - a, c - a)
    area = 0.5 * np.linalg.norm(fn, axis=1)
    ok = area > 1e-12
    fn[ok] /= (2 * area[ok, None])
    rng = np.random.default_rng(seed)
    idx = rng.choice(np.flatnonzero(ok), size=min(n_samples, int(ok.sum())), p=area[ok] / area[ok].sum(),
                     replace=True)
    bvh = BVHTree.FromPolygons(V.tolist(), T.tolist(), epsilon=0.0)
    dist = []
    for i in idx:
        r1, r2 = rng.random(2)
        if r1 + r2 > 1:
            r1, r2 = 1 - r1, 1 - r2
        p = a[i] + r1 * (b[i] - a[i]) + r2 * (c[i] - a[i])
        hit = bvh.ray_cast(Vector(p - fn[i] * 1e-6), Vector(-fn[i]), 0.5)
        if hit[0] is not None:
            dist.append(hit[3] * 1000.0)
    if not dist:
        return {"min_mm": 0.0, "p05_mm": 0.0}
    d = np.asarray(dist)
    return {"min_mm": float(d.min()), "p05_mm": float(np.percentile(d, 5)), "median_mm": float(np.median(d)),
            "frac_lt_0p4": float((d < 0.4).mean())}


# =====================================================================================================
# Düzlem kesiti (2B) — kaburga hafifletme delikleri ve flanş delikleri için
# =====================================================================================================
def slice_segments(V: np.ndarray, F: Sequence, origin, normal, eu, ev) -> np.ndarray:
    """Ağın düzlemle kesiti: (k, 2, 2) 2B doğru parçaları (eu, ev eksenlerinde)."""
    T = _triangulate_fan(F)
    if len(T) == 0:
        return np.zeros((0, 2, 2))
    d = (V - np.asarray(origin)) @ np.asarray(normal)
    dt = d[T]
    sgn = dt > 0
    mixed = sgn.any(1) & (~sgn).any(1)
    segs = []
    for tri in T[mixed]:
        pts = []
        for i, j in ((0, 1), (1, 2), (2, 0)):
            a, b = tri[i], tri[j]
            da, db = d[a], d[b]
            if (da > 0) != (db > 0):
                t = da / (da - db)
                pts.append(V[a] + t * (V[b] - V[a]))
        if len(pts) == 2:
            segs.append(pts)
    if not segs:
        return np.zeros((0, 2, 2))
    S = np.asarray(segs) - np.asarray(origin)
    return np.stack([S @ np.asarray(eu), S @ np.asarray(ev)], axis=-1)


def _seg_dist(Pq: np.ndarray, S: np.ndarray) -> np.ndarray:
    """Noktalar (m, 2) ile parçalar (k, 2, 2) arası en kısa mesafe (m,)."""
    if len(S) == 0:
        return np.full(len(Pq), np.inf)
    a, b = S[:, 0], S[:, 1]
    ab = b - a
    L2 = np.maximum(np.einsum("ij,ij->i", ab, ab), 1e-20)
    out = np.full(len(Pq), np.inf)
    for k0 in range(0, len(S), 4000):
        aa, bb, LL = a[k0:k0 + 4000], ab[k0:k0 + 4000], L2[k0:k0 + 4000]
        ap = Pq[:, None, :] - aa[None]
        t = np.clip(np.einsum("mkj,kj->mk", ap, bb) / LL[None], 0, 1)
        q = aa[None] + t[..., None] * bb[None]
        dd = np.linalg.norm(Pq[:, None, :] - q, axis=2).min(axis=1)
        out = np.minimum(out, dd)
    return out


def _inside(Pq: np.ndarray, S: np.ndarray) -> np.ndarray:
    """Çift-tek kuralı (parça çorbası kapalı eğri(ler) oluşturmalı)."""
    if len(S) == 0:
        return np.zeros(len(Pq), bool)
    a, b = S[:, 0], S[:, 1]
    cnt = np.zeros(len(Pq), int)
    for k0 in range(0, len(S), 4000):
        aa, bb = a[k0:k0 + 4000], b[k0:k0 + 4000]
        y = Pq[:, 1][:, None]
        c1 = (aa[None, :, 1] > y) != (bb[None, :, 1] > y)
        xt = aa[None, :, 0] + (y - aa[None, :, 1]) * (bb[None, :, 0] - aa[None, :, 0]) / np.where(
            np.abs(bb[None, :, 1] - aa[None, :, 1]) < 1e-15, 1e-15, bb[None, :, 1] - aa[None, :, 1])
        cnt += (c1 & (Pq[:, 0][:, None] < xt)).sum(axis=1)
    return cnt % 2 == 1


def place_circles(region: np.ndarray, obstacles: list[np.ndarray], *, margin: float = LIGHT_MARGIN,
                  r_range=LIGHT_R, step: float = 0.0012, max_n: int = 14) -> list[tuple[float, float, float]]:
    """``region`` kapalı eğrisi içinde engellerden ``margin`` uzak, açgözlü en büyük daireler (u, v, r)."""
    if len(region) == 0:
        return []
    lo = region.reshape(-1, 2).min(0)
    hi = region.reshape(-1, 2).max(0)
    gu = np.arange(lo[0] + step, hi[0], step)
    gv = np.arange(lo[1] + step, hi[1], step)
    if len(gu) == 0 or len(gv) == 0:
        return []
    G = np.array([(u, v) for u in gu for v in gv])
    ins = _inside(G, region)
    for ob in obstacles:
        if len(ob):
            ins &= ~_inside(G, ob)
    G = G[ins]
    if len(G) == 0:
        return []
    clear = _seg_dist(G, region)
    for ob in obstacles:
        clear = np.minimum(clear, _seg_dist(G, ob))
    out = []
    for _ in range(max_n):
        r = np.minimum(clear - margin, r_range[1])
        i = int(np.argmax(r))
        if r[i] < r_range[0]:
            break
        u, v = G[i]
        out.append((float(u), float(v), float(r[i])))
        clear = np.minimum(clear, np.hypot(G[:, 0] - u, G[:, 1] - v) - r[i])
    return out


# =====================================================================================================
# Bölgeler (dışbükey; düzlem listesi) ve segment tanımı
# =====================================================================================================
@dataclass
class End:
    """Segment ucu: düzlem (nokta ``p``, dışa normal ``n``; bölge ``n·(x − p) ≤ 0``) ve uç türü.

    ``kind``: ``plate`` (kapalı kaburga), ``frame`` (halka çerçeve, genişlik ``width``), ``land`` (yapıştırma
    flanşı), ``open``. ``holes``: kaburgada hafifletme deliği açılsın mı.
    """

    p: np.ndarray
    n: np.ndarray
    kind: str = "plate"
    width: float = 0.0
    thick: float = 0.0
    holes: bool = True
    label: str = ""
    pins: bool = True

    def __post_init__(self):
        self.p = np.asarray(self.p, float)
        self.n = _unit(self.n)
        if self.thick <= 0:
            self.thick = {"plate": RIB_T, "frame": FRAME_T, "land": LAND_H}.get(self.kind, 0.0)
        if self.width <= 0:
            self.width = {"frame": FRAME_W, "land": LAND_T}.get(self.kind, 0.0)

    def flipped(self, kind: str | None = None) -> "End":
        return End(self.p.copy(), -self.n, kind or self.kind, 0.0 if kind else self.width, 0.0 if kind else self.thick,
                   self.holes, self.label, self.pins)

    def shifted(self, d: float) -> tuple[np.ndarray, np.ndarray]:
        """Düzlemi normal boyunca ``d`` kaydırılmış (p, n)."""
        return self.p + d * self.n, self.n


def region_solid(planes: Sequence[tuple[np.ndarray, np.ndarray]], center, size: float = 6.0) -> Solid:
    """Düzlemlerle sınırlı dışbükey bölge (büyük küpten bmesh bisect + kapak)."""
    bm = bmesh.new()
    c = Vector(tuple(center))
    bmesh.ops.create_cube(bm, size=size, matrix=Matrix.Translation(c))
    for p, n in planes:
        geom = bm.verts[:] + bm.edges[:] + bm.faces[:]
        res = bmesh.ops.bisect_plane(bm, geom=geom, plane_co=Vector(tuple(p)), plane_no=Vector(tuple(n)),
                                     clear_outer=True, dist=1e-9)
        cut_edges = [e for e in res["geom_cut"] if isinstance(e, bmesh.types.BMEdge)]
        if cut_edges:
            bmesh.ops.holes_fill(bm, edges=[e for e in bm.edges if e.is_boundary], sides=0)
    bmesh.ops.recalc_face_normals(bm, faces=bm.faces[:])
    V = np.array([v.co[:] for v in bm.verts], float)
    bm.verts.index_update()
    F = [tuple(v.index for v in f.verts) for f in bm.faces]
    bm.free()
    return Solid(V, F, "region")


@dataclass
class Feature:
    """Segment iç/dış özellikleri (ilkel katılar, dünya ekseni)."""

    structure: list = field(default_factory=list)   # boşluktan çıkarılır (iç yapı)
    holes: list = field(default_factory=list)       # parçadan çıkarılır
    adds: list = field(default_factory=list)        # parçaya eklenir (menteşe dilleri; bölge dışına taşanı kırpılır)
    notes: list = field(default_factory=list)
    keepout: list = field(default_factory=list)     # iç yapının giremeyeceği hacimler (boşlukta hava kalır)

    def extend(self, other: "Feature") -> "Feature":
        self.structure += other.structure
        self.holes += other.holes
        self.adds += other.adds
        self.notes += other.notes
        self.keepout += other.keepout
        return self


@dataclass
class SegSpec:
    """Bir baskı segmenti tarifi (sol/merkez; ``mirror`` → sağ eşi ayna)."""

    key: str                      # dosya adı (ör. "wing_panel_3")
    name_tr: str                  # rapor adı
    group: str                    # rapor grubu
    sources: list                 # kaynak katı anahtarları (ctx.sources)
    zone: str                     # spec print.zones anahtarı → filament, et
    ends: list                    # End listesi
    material: str | None = None   # None → bölge filamenti
    wall: float | None = None     # None → bölge eti
    mirror: bool = False          # sağ eşi var (ayna)
    qty: int = 1                  # (ayna dahil değil) adet
    bed_end: int | None = None    # tablaya oturan uç (ends indisi); None → serbest yön araması
    up_hint: np.ndarray | None = None   # bed_end yoksa tercih edilen yukarı yön
    subtract: list = field(default_factory=list)       # [(kaynak anahtarı, dışa pay m, kapalı duvar et m)]
    features: Callable | None = None                   # f(seg, ctx) → Feature
    shell: bool = True            # False → kaynak olduğu gibi (kapak, ince panel)
    cover: bool = False           # True → açık kapak kabuğu (kaporta/fileto): A − iç(kaynak)
    union_extra: list = field(default_factory=list)    # kabuktan sonra eklenecek kaynaklar (panjur)
    split_loose: bool = False     # bağlantısız kabuklar ayrı parça
    note: str = ""
    center: np.ndarray | None = None
    cutouts: list = field(default_factory=list)        # parçadan çıkarılan açıklıklar (flanş: wall_zones)
    wall_zones: list = field(default_factory=list)     # ek et bölgeleri (WallZone)
    cavity_planes: list = field(default_factory=list)  # yalnız boşluğu kırpan düzlemler [(p, n)]
    wall_source: str | None = None                     # et fonksiyonu kaynağı (None → sources[0])
    perturb: int = 0                                   # >0: çakışma kırma denemesi (µm düzeyi kaydırma)
    force_up: bool = False                             # tercih edilen yön sığmazsa yatırma, böl (dik baskı şart)
    builder: Callable | None = None                    # özel kurucu f(ctx, seg, out_col) → Built (yük yolu parçaları vb.)
    tooling: bool = False                              # alet (kalıp): uçak kütle/süre toplamına girmez, %20 dolgu
    keep_fn: Callable | None = None                    # f() → [Solid]: parça bunlarla kesişir (kaplama tüy kenarı kırpma)
    solid: bool = False                                # dolu STL (boşluk yok): dilimleyici çevre + seyrek dolgu basar
    cavity_keep_fn: Callable | None = None             # f() → [Solid]: boşluk yalnız bunların içinde (ince yerde dolu)


# =====================================================================================================
# Yerleşim: baskı yönü, tablaya sığma, çıkıntı
# =====================================================================================================
def _rot_to_z(u) -> np.ndarray:
    """``u`` yönünü +Z'ye çeviren en küçük dönüş (3×3)."""
    u = _unit(u)
    z = np.array([0.0, 0.0, 1.0])
    c = float(np.dot(u, z))
    if c > 1 - 1e-12:
        return np.eye(3)
    if c < -1 + 1e-12:
        return np.diag([1.0, -1.0, -1.0])
    ax = _unit(np.cross(u, z))
    s = math.sqrt(max(0.0, 1 - c * c))
    K = np.array([[0, -ax[2], ax[1]], [ax[2], 0, -ax[0]], [-ax[1], ax[0], 0]])
    return np.eye(3) + s * K + (1 - c) * (K @ K)


def _hull2(P2: np.ndarray) -> np.ndarray:
    """2B dışbükey zarf (monotone chain)."""
    pts = np.unique(np.round(P2, 7), axis=0)
    if len(pts) < 3:
        return pts
    pts = pts[np.lexsort((pts[:, 1], pts[:, 0]))]

    def cross(o, a, b):
        return (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0])

    lower, upper = [], []
    for p in pts:
        while len(lower) >= 2 and cross(lower[-2], lower[-1], p) <= 0:
            lower.pop()
        lower.append(tuple(p))
    for p in pts[::-1]:
        while len(upper) >= 2 and cross(upper[-2], upper[-1], p) <= 0:
            upper.pop()
        upper.append(tuple(p))
    return np.asarray(lower[:-1] + upper[:-1])


def fit_footprint(hull: np.ndarray, bed_xy: tuple[float, float], margin: float) -> dict:
    """Taban izini (mm) tabla içinde en iyi Z açısıyla yerleştirir. Dönüş: açı, ölçüler, sığar mı, pay."""
    best = None
    for deg in np.arange(0.0, 180.0, 1.0):
        a = math.radians(deg)
        R = np.array([[math.cos(a), -math.sin(a)], [math.sin(a), math.cos(a)]])
        Q = hull @ R.T
        w, h = Q.max(0) - Q.min(0)
        slack = min(bed_xy[0] - w, bed_xy[1] - h) / 2.0
        if best is None or slack > best["margin_mm"]:
            best = {"angle_deg": float(deg), "w_mm": float(w), "h_mm": float(h), "margin_mm": float(slack)}
    best["fits"] = best["margin_mm"] >= margin - 1e-6
    best["fits_tight"] = best["margin_mm"] >= BED_MARGIN_MIN - 1e-6
    return best


CONTACT_MIN_CM2 = 3.0                               # tabla teması en az (cm²) ve izin %3'ü (P6). varsayım
CONTACT_MIN_FRAC = 0.03
FLAT_TOL = 2e-5                                     # tabla yüzü: |z| < 0,02 mm ve n_z < −0,999 (denetçi tanımı)
SUPPORT_MIN_CM2 = 2.0                               # 45°'den dik ve altında tabla olan alan → "tabla desteği"
MODEL_SUPPORT_MIN_CM2 = 8.0                         # 45°'den dik, modelin > 5 mm üstünde → "model üstü destek"


def placement(V: np.ndarray, T: np.ndarray, up, bed: tuple[float, float, float], margin: float = BED_MARGIN) -> dict:
    """Parçayı ``up`` yukarı olacak biçimde çevirir, tabanı Z = 0'a, izi tabla merkezine koyar.
    Dönüş: 4×4 dönüşüm (m → m, tabla merkezli), ölçüler (mm), çıkıntı alanları (cm²; 45°), tabla teması (cm²;
    n_z < −0,999 ve tabanın 0,02 mm içinde — bağımsız denetçiyle aynı tanım), iz alanı (cm²)."""
    R = _rot_to_z(up)
    W = V @ R.T
    z0 = float(W[:, 2].min())
    hull = _hull2(W[:, :2] * 1000.0)
    fit = fit_footprint(hull, bed[:2], margin)
    a = math.radians(fit["angle_deg"])
    Rz = np.array([[math.cos(a), -math.sin(a), 0], [math.sin(a), math.cos(a), 0], [0, 0, 1.0]])
    W = W @ Rz.T
    cxy = 0.5 * (W[:, :2].min(0) + W[:, :2].max(0))
    W -= np.array([cxy[0], cxy[1], z0])
    height = float(W[:, 2].max() * 1000.0)
    M = np.eye(4)                                   # x → Rz·R·x − (cxy, z0): tabla merkezli, taban Z = 0
    M[:3, :3] = Rz @ R
    M[:3, 3] = -np.array([cxy[0], cxy[1], z0])
    a_, b_, c_ = W[T[:, 0]], W[T[:, 1]], W[T[:, 2]]
    fn = np.cross(b_ - a_, c_ - a_)
    ar = 0.5 * np.linalg.norm(fn, axis=1)
    nz = fn[:, 2] / np.maximum(2 * ar, 1e-18)
    zc = (a_[:, 2] + b_[:, 2] + c_[:, 2]) / 3.0
    on_bed = (nz < -0.999) & (zc < FLAT_TOL)
    over45 = (nz < -math.sin(math.radians(45.0))) & ~on_bed & (zc > 5e-5)
    fits_z = height <= bed[2] - Z_MARGIN
    hx = np.vstack([hull, hull[:1]]) if len(hull) >= 3 else hull
    hull_cm2 = float(0.5 * abs(np.sum(hx[:-1, 0] * hx[1:, 1] - hx[1:, 0] * hx[:-1, 1]))) / 100.0 if len(hull) >= 3 else 0.0
    contact = float(ar[on_bed].sum() * 1e4)
    need = max(CONTACT_MIN_CM2, CONTACT_MIN_FRAC * hull_cm2)
    return {"M": M, "up": tuple(float(x) for x in _unit(up)), "angle_deg": fit["angle_deg"],
            "size_mm": (fit["w_mm"], fit["h_mm"], height), "margin_mm": fit["margin_mm"],
            "fits": bool(fit["fits"] and fits_z), "fits_tight": bool(fit["fits_tight"] and fits_z),
            "overhang_cm2": float(ar[over45].sum() * 1e4), "bed_contact_cm2": contact, "footprint_cm2": hull_cm2,
            "contact_need_cm2": need, "contact_ok": contact >= need - 1e-9}


def _planar_ups(V: np.ndarray, T: np.ndarray, min_cm2: float = 2.0, max_n: int = 10) -> list:
    """Büyük düzlemsel yüz kümelerinin (normal 1°'de) "tabla yüzü" adayları: up = −n."""
    a, b, c = V[T[:, 0]], V[T[:, 1]], V[T[:, 2]]
    fn = np.cross(b - a, c - a)
    ln = np.linalg.norm(fn, axis=1)
    ok = ln > 1e-14
    n = fn[ok] / ln[ok, None]
    ar = 0.5 * ln[ok]
    d = np.einsum("ij,ij->i", n, (a[ok] + b[ok] + c[ok]) / 3.0)
    key = np.round(np.column_stack([n / 0.015, d / 0.0005])).astype(np.int64)       # yön ≈ 0,9°, düzlem 0,5 mm
    _, inv = np.unique(key, axis=0, return_inverse=True)
    inv = inv.ravel()
    area = np.bincount(inv, weights=ar)
    order = np.argsort(-area)
    out = []
    for g in order[:max_n]:
        if area[g] * 1e4 < min_cm2:
            break
        m = inv == g
        nn = _unit((n[m] * ar[m, None]).sum(0))
        if all(float(nn @ o) < 0.9995 for o in out):
            out.append(nn)
    return [-x for x in out]


def choose_orientation(V: np.ndarray, T: np.ndarray, bed, preferred: Sequence = (), margin: float = BED_MARGIN,
                       prefer_slack_cm2: float = 3.0, force: bool = False) -> dict:
    """Aday yönler: tercih edilenler (ör. tablaya oturan kaburga), büyük düzlemsel yüzler (−n), OBB eksenleri (±),
    dünya eksenleri (±). Sıra: sığma → tabla teması ≥ max(3 cm², izin %3'ü) (P6, ``force`` dahil sert koşul) →
    45° çıkıntı alanı → yükseklik. Tercih edilen yön, en iyiden en çok ``prefer_slack_cm2`` kötüyse seçilir;
    ``force`` → tercih edilen yön sığdığı ve teması yettiği sürece seçilir."""
    C = V - V.mean(0)
    _, _, Vt = np.linalg.svd(C[:: max(1, len(C) // 4000)], full_matrices=False)
    cands = [(_unit(u), True) for u in preferred if u is not None]
    for u in _planar_ups(V, T):
        cands.append((u, False))
    for ax in Vt:
        cands += [(_unit(ax), False), (_unit(-ax), False)]
    for ax in np.eye(3):                                # dünya eksenleri (PCA eksenleri kabuk kırpılınca kayar)
        cands += [(ax.copy(), False), (-ax, False)]
    res = []
    for u, pref in cands:
        pl = placement(V, T, u, bed, margin)
        pl["preferred"] = pref
        res.append(pl)

    def key(p):                                         # sığma → yeterli tabla teması → puan (çıkıntı + yükseklik)
        score = (p["overhang_cm2"] + 0.04 * p["size_mm"][2] - 0.02 * min(p["bed_contact_cm2"], 50.0)
                 - (0.5 if p["preferred"] else 0.0))     # 25 mm yükseklik ≈ 1 cm² çıkıntı (yatık ince parça yatar)
        return (0 if p["fits"] else (1 if p["fits_tight"] else 2), 0 if p["contact_ok"] else 1, score)

    res.sort(key=key)
    best = res[0]
    for p in res:
        if p["preferred"] and (p["fits"] or not best["fits"]) and (p["fits_tight"] or not best["fits_tight"]) \
                and (p["contact_ok"] or not best["contact_ok"]):
            if force or key(p)[2] <= key(best)[2] + prefer_slack_cm2:
                return p
            break
    return best


def support_class(V: np.ndarray, T: np.ndarray, M: np.ndarray) -> dict:
    """Seçilen baskı yönünde 45°'den dik aşağı bakan yüzler için aşağı ışın (denetçi yöntemi): altında model yoksa
    "tabla desteği" alanı, model varsa "model üstü" (düşüş > 5 mm ayrıca). Dönüş cm²."""
    W = V @ M[:3, :3].T + M[:3, 3]
    a, b, c = W[T[:, 0]], W[T[:, 1]], W[T[:, 2]]
    fn = np.cross(b - a, c - a)
    ar2 = np.linalg.norm(fn, axis=1)
    ar = 0.5 * ar2
    n = fn / np.maximum(ar2, 1e-18)[:, None]
    cen = (a + b + c) / 3.0
    on_bed = (n[:, 2] < -0.999) & (cen[:, 2] < FLAT_TOL)
    down = ~on_bed & (cen[:, 2] > 5e-5) & (n[:, 2] < -math.sin(math.radians(45.0)))
    idx = np.flatnonzero(down)
    to_bed = on_model = long_drop = 0.0
    if len(idx):
        bvh = BVHTree.FromPolygons(W.tolist(), T.tolist(), epsilon=0.0)
        dn = Vector((0.0, 0.0, -1.0))
        for i in idx:
            o = cen[i] - np.array([0.0, 0.0, 1e-5])
            h = bvh.ray_cast(Vector(o), dn, 10.0)
            if h[0] is None:
                to_bed += ar[i]
            else:
                on_model += ar[i]
                if o[2] - h[0][2] > 0.005:
                    long_drop += ar[i]
    out = {"oh45_to_bed_cm2": round(to_bed * 1e4, 2), "oh45_over_model_cm2": round(on_model * 1e4, 2),
           "oh45_over_model_gt5mm_cm2": round(long_drop * 1e4, 2)}
    if out["oh45_to_bed_cm2"] > SUPPORT_MIN_CM2:
        out["support"] = "tabla desteği"
    elif out["oh45_over_model_gt5mm_cm2"] > MODEL_SUPPORT_MIN_CM2:
        out["support"] = "model üstü destek"
    else:
        out["support"] = "yok"
    return out


# =====================================================================================================
# STL (ikili, mm)
# =====================================================================================================
def write_stl(path: Path, V: np.ndarray, T: np.ndarray, name: str = "") -> int:
    """İkili STL yazar (V metre → mm). Normaller dosyaya yazılan float32 köşelerden hesaplanır (dilimleyicinin
    yeniden hesabıyla aynı). Dosya boyutunu (bayt) döndürür."""
    Vm = _mm32(V)
    a, b, c = Vm[T[:, 0]], Vm[T[:, 1]], Vm[T[:, 2]]
    n = np.cross(b - a, c - a)
    ln = np.linalg.norm(n, axis=1)
    n[ln > 0] /= ln[ln > 0, None]
    rec = np.zeros(len(T), dtype=[("n", "<f4", 3), ("v", "<f4", (3, 3)), ("a", "<u2")])
    rec["n"] = n
    rec["v"] = np.stack([a, b, c], axis=1)
    header = f"YELKOVAN YK-38 {name} mm".encode("ascii", "replace")[:80].ljust(80, b" ")
    path.parent.mkdir(parents=True, exist_ok=True)
    with open(path, "wb") as fh:
        fh.write(header)
        fh.write(struct.pack("<I", len(T)))
        fh.write(rec.tobytes())
    return path.stat().st_size


def read_stl(path: Path) -> tuple[np.ndarray, np.ndarray]:
    """İkili STL okur ve köşeleri tam eşitlikle birleştirir → (V mm, T)."""
    data = Path(path).read_bytes()
    n = struct.unpack("<I", data[80:84])[0]
    rec = np.frombuffer(data[84:84 + 50 * n], dtype=[("n", "<f4", 3), ("v", "<f4", (3, 3)), ("a", "<u2")])
    P3 = rec["v"].reshape(-1, 3)
    V, inv = np.unique(P3, axis=0, return_inverse=True)
    return V.astype(float), inv.reshape(-1, 3)


def stl_edge_check(T: np.ndarray) -> dict:
    """Üçgen listesinin kenar denetimi (her yönlü kenar bir kez, ters eşi var)."""
    E = np.concatenate([T[:, [0, 1]], T[:, [1, 2]], T[:, [2, 0]]])
    key = E[:, 0].astype(np.int64) * (1 << 32) + E[:, 1]
    rkey = E[:, 1].astype(np.int64) * (1 << 32) + E[:, 0]
    u, cnt = np.unique(key, return_counts=True)
    dup = int((cnt > 1).sum())
    missing = int((~np.isin(rkey, key)).sum())
    return {"triangles": int(len(T)), "dup_directed": dup, "unpaired": missing, "ok": dup == 0 and missing == 0}


# =====================================================================================================
# Kaynaklar (dış katılar) ve çıkarıcı gövdeler
# =====================================================================================================
@dataclass
class Source:
    """Baskı kaynağı: bir ya da birkaç sahne nesnesinin birleşimi + köşe başına et fonksiyonu (m)."""

    key: str
    names: list
    wall_fn: Callable[[np.ndarray], np.ndarray]
    solid_fn: Callable[[], Solid] | None = None     # sahne yerine numpy katı (ör. kuyusuz temiz gövde)
    solid: Solid | None = None
    nudge: dict = field(default_factory=dict)       # nesne adı → µm öteleme (eş düzlemli temas yüzlerini birleşimde kır)
    merge: bool = False                             # bileşenler tek tek MANIFOLD birleşimi; değmeyen parçalar atılır


def _wall_const(w: float) -> Callable:
    return lambda V: np.full(len(V), w)


def _smooth(t):
    t = np.clip(t, 0.0, 1.0)
    return t * t * (3 - 2 * t)


def _wall_dbox(base: float, dbox: float = _WALLS["d_box"], s_spar: float = P.WING_SPAR_S,
               blend: float = 0.008) -> Callable:
    """Kanat: ana kirişin önü D-kutu (``dbox``), arkası ``base``; ``blend`` boyunca yumuşak geçiş."""
    def fn(V):
        s = -V[:, 0]
        return base + (dbox - base) * _smooth((s_spar + 0.5 * blend - s) / blend)
    return fn


def _wall_fuselage(V):
    """Gövde: burun (s < 0,40) ve halkalar 1–6 (0,40–1,55) LW-PLA 0,7 mm (R04; spec ``print.zones``); kuyruk konisi
    (s ≥ 1,55) LW-ASA 1,0 mm. Burun geçişi halka 1'in ilk 1–5 mm'sinde, kuyruk geçişi flanş düzleminde."""
    s = -V[:, 0]
    wn = _zone("fuselage_nose")[1]
    w0 = _zone("fuselage_mid")[1]
    w1 = _zone("tail_cone")[1]
    s_nm = float(P.SPEC["fuselage"]["modules"]["nose_module_s_m"][1])
    s_f = float(P.SPEC["fuselage"]["modules"]["flange_s_m"])
    w = wn + (w0 - wn) * _smooth((s - (s_nm + 0.001)) / 0.004)
    return w + (w1 - w) * _smooth((s - (s_f - 0.002)) / 0.004)


@dataclass
class WallZone:
    """Köşe başına ek et bölgesi: düzleme (``p``, malzemeye doğru normal ``n``) uzaklık ``0 ≤ s ≤ depth`` olan
    köşeler ``target`` toplam ete (en az ``base + extra``) çıkar; ``depth → depth + ramp`` arası doğrusal geçiş
    (45°'lik pah = basılabilir flanş alt yüzü). ``box``: (min, max) dünya sınırı (yalnız bu kutudaki köşeler)."""

    p: np.ndarray
    n: np.ndarray
    depth: float
    ramp: float
    extra: float = 0.0
    target: float = 0.0
    box: tuple | None = None
    pre: float = 0.0005                     # düzlemin dışına taşma: kesit yüzünde tam et (geçiş parça dışında)
    clip: tuple | None = None               # ((p, n, pay), …): yalnız (V − p)·n ≤ pay olan köşeler (eğik bölge sınırı)

    def levels(self) -> list[float]:
        """Ağın bölüneceği uzaklıklar (geçişlerin keskin kalması için)."""
        out = [-self.pre - 0.0002, -self.pre, self.depth]
        if self.ramp > 0:
            out.append(self.depth + self.ramp)
        return out

    def weights(self, V: np.ndarray) -> np.ndarray:
        s = (V - self.p) @ self.n
        w = np.where((s >= -self.pre - 1e-7) & (s <= self.depth + 1e-7), 1.0, 0.0)
        r0 = (s + self.pre) / -0.0002
        w = np.maximum(w, np.where((r0 > 0) & (r0 < 1), 1.0 - r0, 0.0))
        if self.ramp > 0:
            r = (s - self.depth) / self.ramp
            w = np.maximum(w, np.where((r > 0) & (r < 1), 1.0 - r, 0.0))
        if self.box is not None:
            lo, hi = (np.asarray(x) for x in self.box)
            w *= np.all((V >= lo) & (V <= hi), axis=1)
        for cp, cn, margin in (self.clip or ()):
            w *= ((V - np.asarray(cp)) @ np.asarray(cn)) <= margin
        return w


class Ctx:
    """Çalışma bağlamı: kaynaklar, boolean yürütücü, önbellekler, uyarılar. ``cleanup()`` geçicileri siler."""

    def __init__(self, scene: bpy.types.Scene, bed, solver: str = "MANIFOLD", cut_solver: str = "EXACT",
                 verbose: bool = False):
        self.scene = scene
        self.cut_solver = cut_solver
        self.bed = tuple(float(b) for b in bed)
        self.work = _work_collection(scene)
        self.B = Booler(self.work, solver)
        self.verbose = verbose
        self.t0 = time.time()
        self.sources: dict[str, Source] = {}
        self._outer: dict[str, bpy.types.Object] = {}
        self._grown: dict[tuple, bpy.types.Object] = {}
        self._n = 0
        self.warnings: list[str] = []

    def log(self, msg: str) -> None:
        if self.verbose:
            print(f"[printprep {time.time() - self.t0:7.1f} s] {msg}", flush=True)

    def name(self, base: str) -> str:
        self._n += 1
        return f"PPW_{base}_{self._n:04d}"

    def obj(self, base: str, solid: Solid) -> bpy.types.Object:
        return _obj(self.name(base), solid, self.work)

    def copy(self, ob: bpy.types.Object, base: str = "cp") -> bpy.types.Object:
        new = bpy.data.objects.new(self.name(base), ob.data.copy())
        new.matrix_world = ob.matrix_world.copy()
        self.work.objects.link(new)
        return new

    # ------------------------------------------------------------------ kaynaklar
    def add_source(self, key: str, names: Sequence[str], wall_fn: Callable, solid_fn: Callable | None = None,
                   nudge: dict | None = None, merge: bool = False) -> bool:
        missing = [n for n in names if n not in bpy.data.objects]
        if missing and solid_fn is None:
            self.warnings.append(f"kaynak eksik: {', '.join(missing)} ({key} atlandı)")
            return False
        self.sources[key] = Source(key, list(names), wall_fn, solid_fn, nudge=dict(nudge or {}), merge=merge)
        return True

    def _merge_components(self, key: str, solids: list) -> bpy.types.Object:
        """Örtüşen bileşenleri (kapak + menteşe dili/horn, iki gövdeli tapa kapak) büyükten küçüğe tek tek birleştirir;
        ana gövdeye değmeyen bileşen atılır ve not düşülür (``self.merge_notes[key]``)."""
        comps = []
        for sl in solids:
            tmp = self.obj("mc", sl)
            comps += split_shells(tmp)
            _delete(tmp)
        comps.sort(key=lambda c: -abs(_signed_volume(*c)))
        ob = self.obj(f"out_{key}", Solid(comps[0][0], comps[0][1], key))
        dropped = []
        for Vc, Fc in comps[1:]:
            v_c = abs(_signed_volume(Vc, Fc))
            backup = ob.data.copy()
            v0 = mesh_check(ob.data)["volume_m3"]
            o2 = self.obj("mcx", Solid(Vc, Fc))
            self.B.op(ob, o2, "UNION", solver="MANIFOLD")
            _delete(o2)
            c = mesh_check(ob.data)
            if c["ok"] and c["shells"] == 1 and c["volume_m3"] >= v0 - 1e-10:
                bpy.data.meshes.remove(backup)
                continue
            bad = ob.data
            ob.data = backup
            bpy.data.meshes.remove(bad)
            dropped.append(round(v_c * 1e6, 3))
        if dropped:
            self.__dict__.setdefault("merge_notes", {})[key] = dropped
            self.log(f"{key}: gövdeye değmeyen {len(dropped)} bileşen atıldı ({dropped} cm³)")
        return ob

    def has(self, key: str) -> bool:
        return key in self.sources

    def outer(self, key: str) -> bpy.types.Object:
        """Kaynağın dış katısı (dünya, dinlenme pozu; birden çok nesne → EXACT birleşim)."""
        if key in self._outer:
            return self._outer[key]
        src = self.sources[key]
        solids = [src.solid_fn()] if src.solid_fn else [source_solid(n) for n in src.names]
        for sl in solids:                               # eş düzlemli temas (sıfır kalınlıklı zar) olmasın: µm öteleme
            if sl.name in src.nudge:
                sl.V = sl.V + np.asarray(src.nudge[sl.name], float)
        if src.merge:
            ob = self._merge_components(key, solids)
        else:
            ob = self.obj(f"out_{key}", solids[0])
            for s in solids[1:]:
                o2 = self.obj(f"outx_{key}", s)
                self.B.op(ob, o2, "UNION", solver=self.cut_solver)
                _delete(o2)
        if not mesh_check(ob.data)["ok"]:               # yalnız gerekirse (temizlik geçerli birleşimi bozabilir)
            clean_mesh(ob)
        V, F = _mesh_np(ob.data)
        src.solid = Solid(V, F, key)
        self._outer[key] = ob
        return ob

    def solid(self, key: str) -> Solid:
        self.outer(key)
        return self.sources[key].solid

    def grown(self, key: str, pad: float) -> bpy.types.Object:
        """Dışa ``pad`` kadar büyütülmüş kaynak (montaj boşluklu çıkarıcı)."""
        k = (key, round(pad, 6))
        if k not in self._grown:
            if pad <= 0:
                self._grown[k] = self.outer(key)
            else:
                S = self.solid(key)
                V, T = solid_triangles(S)
                Vo = offset_vertices(V, T, -pad)
                self._grown[k] = self.obj(f"gr_{key}", Solid(Vo, [tuple(t) for t in T]))
        return self._grown[k]

    def custom_solid(self, key: str, parts_fn: Callable[[], dict]) -> Solid | None:
        """Boolean ile kurulan yardımcı katı (ör. komşu parçadaki cep), önbellekli → numpy Solid."""
        if not hasattr(self, "_custom"):
            self._custom = {}
        if key not in self._custom:
            parts = parts_fn()
            A = self.obj("cs", parts["base"])
            col = self.B.group(self.name("csa"), parts.get("adds", []))
            if col is not None:
                self.B.op(A, col, "UNION")
                Booler.drop(col)
            for cl in parts.get("clip", []) or []:
                co = self.obj("csc", cl)
                self.B.op(A, co, "INTERSECT")
                _delete(co)
            col = self.B.group(self.name("csh"), parts.get("holes", []))
            if col is not None:
                self.B.op(A, col, "DIFFERENCE")
                Booler.drop(col)
            finalize_mesh(A)
            V, F = _mesh_np(A.data)
            _delete(A)
            self._custom[key] = Solid(V, F, key) if len(F) else None
        return self._custom[key]

    def cleanup(self) -> None:
        for col in list(self.work.children):
            Booler.drop(col)
        for ob in list(self.work.objects):
            _delete(ob)
        if not self.work.objects and not self.work.children:
            bpy.data.collections.remove(self.work)
        for me in list(bpy.data.meshes):
            if me.users == 0 and me.name.startswith("PPW_"):
                bpy.data.meshes.remove(me)


# =====================================================================================================
# Segment kurucu
# =====================================================================================================
@dataclass
class Built:
    """Kurulan parça: nesne (dünya, montaj konumu), tarif, denetim/ölçüm sonuçları."""

    seg: SegSpec
    ob: bpy.types.Object
    check: dict
    info: dict = field(default_factory=dict)


LAND_RAMP = 0.0025                                  # flanş altı pah yüksekliği (≈ 35–45°). varsayım
LOCAL_MARGIN = 0.030                                # yerel kaydırma için bölge payı


def _planes(ends: Sequence[End], grow: float = 0.0) -> list:
    return [(e.p + grow * e.n, e.n) for e in ends]


def _cavity_planes(ends: Sequence[End], extra: Sequence = ()) -> list:
    out = []
    for e in ends:
        if e.kind == "plate":
            out.append(e.shifted(-e.thick))
        else:
            out.append(e.shifted(+0.02))
    return out + list(extra)


def _end_basis(e: End) -> tuple[np.ndarray, np.ndarray]:
    u, v, _ = _perp_frame(e.n)
    return u, v


def _wall_zones(seg: SegSpec) -> list:
    zones = []
    for e in seg.ends:
        if e.kind == "land":
            zones.append(WallZone(e.p, -e.n, e.thick, LAND_RAMP, extra=0.0006, target=e.width))
        elif e.kind == "frame":
            zones.append(WallZone(e.p, -e.n, e.thick, 0.0002, extra=e.width))
    return zones + list(seg.wall_zones)


def _bisected(ob: bpy.types.Object, zones: Sequence[WallZone]) -> None:
    """Et bölgesi sınırlarında ağı böler (köşe sırası oluşsun: keskin flanş/çerçeve geçişi)."""
    if not zones:
        return
    bm = bmesh.new()
    bm.from_mesh(ob.data)
    for z in zones:
        for d in z.levels():
            geom = bm.verts[:] + bm.edges[:] + bm.faces[:]
            bmesh.ops.bisect_plane(bm, geom=geom, plane_co=Vector(tuple(z.p + z.n * d)), plane_no=Vector(tuple(z.n)),
                                   dist=1e-7)
    bm.to_mesh(ob.data)
    bm.free()


def segment_cavity(ctx: Ctx, seg: SegSpec, base: bpy.types.Object, wall_src: str) -> tuple[Solid, bpy.types.Object]:
    """Segmentin iç boşluğu: yerel kaynağın köşe başına etle içe kaydırılması → bölge'∩ (MANIFOLD).
    Dönüş: (kaydırılmış katı (kesit/delik yerleşimi için), boşluk nesnesi)."""
    zones = _wall_zones(seg)
    tmp = ctx.copy(base, "loc")
    _bisected(tmp, zones)
    V, T = triangulate(tmp.data)
    _delete(tmp)
    w = ctx.sources[wall_src].wall_fn(V) * (1.0 + 0.002 * seg.perturb)
    for z in zones:
        wt = z.weights(V)
        tgt = np.maximum(w + z.extra, z.target)
        w = w + (tgt - w) * wt
    Vi = offset_vertices(V, T, w)
    inner = Solid(Vi, [tuple(t) for t in T], "inner")
    cav = ctx.obj("cav", inner)
    center = seg.center if seg.center is not None else (np.mean([e.p for e in seg.ends], axis=0) if seg.ends
                                                         else V.mean(0))
    creg = ctx.obj("creg", region_solid(_cavity_planes(seg.ends, seg.cavity_planes), center))
    ctx.B.op(cav, creg, "INTERSECT")
    _delete(creg)
    return inner, cav


def rib_holes(ctx: Ctx, seg: SegSpec, e: End, feat: Feature, inner: Solid) -> list:
    """Kaburga (plate) hafifletme delikleri: boşluk kesiti içinde, iç yapı/delik izlerinden uzak."""
    o = e.p - e.n * (0.5 * e.thick)
    eu, ev = _end_basis(e)
    region = slice_segments(inner.V, inner.F, o, e.n, eu, ev)
    if len(region) < 3:
        return []
    obstacles = []
    for s in feat.structure + feat.holes + feat.adds:
        seg2 = slice_segments(s.V, s.F, o, e.n, eu, ev)
        if len(seg2):
            obstacles.append(seg2)
    for e2 in seg.ends:                                 # komşu uç düzlemlerinin kaburga izi
        if e2 is e:
            continue
        d = np.cross(e.n, e2.n)
        if np.linalg.norm(d) < 1e-6:
            continue
        du = _unit(d)
        A = np.vstack([e.n, e2.n, du])
        rhs = np.array([e.n @ o, e2.n @ (e2.p - e2.n * (e2.thick + 0.003)), du @ o])
        try:
            x0 = np.linalg.solve(A, rhs)
        except np.linalg.LinAlgError:
            continue
        p0, p1 = x0 - 2 * du, x0 + 2 * du
        obstacles.append(np.array([[[(p0 - o) @ eu, (p0 - o) @ ev], [(p1 - o) @ eu, (p1 - o) @ ev]]]))
    circles = place_circles(region, obstacles)
    holes = []
    for u, v, r in circles:
        c = o + u * eu + v * ev
        holes.append(prim_frustum(c + e.n * (0.5 * e.thick + 0.002), c - e.n * (0.5 * e.thick + 0.0004), r,
                                  n=max(16, min(40, int(r * 2000))), name="light"))
    return holes


def _cut_region(ctx: Ctx, seg: SegSpec, base: bpy.types.Object, center) -> bpy.types.Object:
    """``A = kaynak ∩ bölge`` — sırayla dener: (1) tam kaynak katısından EXACT, (2) yerel (30 mm paylı) kopyadan
    EXACT, (3) MANIFOLD. Ardışık iki EXACT kesim, eş düzlemli kaynak birleşimlerinde (kanat dış paneli ∪ uç) sıfır
    kalınlıklı zar bırakabilir → manifold olmayan kenar; ilk kapalı sonuç alınır."""
    B = ctx.B
    tries = []
    if len(seg.sources) == 1:
        tries.append((ctx.outer(seg.sources[0]), ctx.cut_solver))
    tries += [(base, ctx.cut_solver), (base, "MANIFOLD")]
    best = None
    for src, solver in tries:
        A = ctx.copy(src, "A")
        reg = ctx.obj("reg", region_solid(_planes(seg.ends), center))
        B.op(A, reg, "INTERSECT", solver=solver)
        _delete(reg)
        c = mesh_check(A.data)
        if not c["ok"]:
            clean_mesh(A, triangles=True)
            c = mesh_check(A.data)
        if c["ok"]:
            if best is not None:
                _delete(best)
            return A
        if best is None:
            best = A
        else:
            _delete(A)
    return best


def build_segment(ctx: Ctx, seg: SegSpec, feat: Feature, out_col: bpy.types.Collection) -> Built | None:
    """Tarifi uygular ve parçayı ``out_col``'a koyar (dünya montaj konumunda).

    Sıra: yerel kaynak = kaynak ∩ (bölge + 30 mm) − duvarlı çıkarıcılar (EXACT) → A = yerel ∩ bölge (EXACT)
    − duvarsız çıkarıcılar → boşluk = içe kaydırma ∩ bölge' − iç yapı (MANIFOLD) → A − boşluk ∪ ekler − delikler.
    Her boolean işleminde en çok bir işlenen kaydırılmış (öz-kesişebilir) yüzey içerir.
    """
    B = ctx.B
    src = seg.sources[0]
    if seg.perturb:                                     # tam çakışmaları (sıkışma kenarı) kır
        k = seg.perturb
        jit = np.array([0.37, 0.61, 0.71]) * 1.3e-5 * k
        M = np.eye(4)
        M[:3, 3] = jit
        feat = Feature([x.transformed(M) for x in feat.structure], [x.transformed(M) for x in feat.holes],
                       [x.transformed(M) for x in feat.adds], list(feat.notes),
                       [x.transformed(M) for x in feat.keepout])
        seg = SegSpec(**{**seg.__dict__, "ends": [End(e.p + e.n * (1.1e-5 * k * (1 if i % 2 else -1)), e.n, e.kind,
                                                      e.width, e.thick, e.holes, e.label, e.pins)
                                                  for i, e in enumerate(seg.ends)],
                         "subtract": [(kk, pad + 2e-5 * k, w) for kk, pad, w in seg.subtract]})
    base = ctx.copy(ctx.outer(src), "base")
    for k in seg.sources[1:]:
        B.op(base, ctx.outer(k), "UNION", solver=ctx.cut_solver)
    center = seg.center if seg.center is not None else (np.mean([e.p for e in seg.ends], axis=0) if seg.ends
                                                         else None)
    if seg.ends:
        reg = ctx.obj("regx", region_solid(_planes(seg.ends, LOCAL_MARGIN), center))
        B.op(base, reg, "INTERSECT", solver=ctx.cut_solver)
        _delete(reg)
    walled = [(k, pad, wall) for k, pad, wall in seg.subtract if wall > 0]
    plain = [(k, pad, wall) for k, pad, wall in seg.subtract if wall <= 0]
    if seg.ends:
        A = _cut_region(ctx, seg, base, center)
    else:
        A = ctx.copy(base, "A")
    for k, pad, _ in walled:
        B.op(base, ctx.grown(k, pad), "DIFFERENCE")
        B.op(A, ctx.grown(k, pad), "DIFFERENCE")
    for cut in seg.cutouts:                             # açıklıklar yalnız parçadan (kenar flanşı et bölgesinden gelir)
        co = ctx.obj("cut", cut)
        B.safe(A, co, "DIFFERENCE", ctx.cut_solver, "MANIFOLD", min_volume=1e-9)
        _delete(co)
    for k, pad, _ in plain:                             # temiz sahne katıları: EXACT, olmazsa MANIFOLD (doğrulamalı)
        B.safe(A, ctx.grown(k, pad), "DIFFERENCE", ctx.cut_solver, B.solver, min_volume=1e-9)
    if seg.keep_fn is not None:                         # tüy kenar kırpma: yalnız et ≥ TRIM_T bölgesi kalır (AERO-20)
        for ks in seg.keep_fn() or []:
            ko = ctx.obj("keep", ks)
            B.safe(A, ko, "DIFFERENCE" if ks.name == "trim_hole" else "INTERSECT", ctx.cut_solver, B.solver,
                   min_volume=1e-9)
            _delete(ko)
    if len(A.data.polygons) == 0:
        _delete(A)
        _delete(base)
        return None
    chkA = mesh_check(A.data)
    if not chkA["ok"]:                                  # EXACT kesim kırıntıları (5 µm) → MANIFOLD adımları reddetmesin
        clean_mesh(A, triangles=True)
        chkA = mesh_check(A.data)
    if not chkA["ok"]:                                  # kesim düzlemi kaynakla çakışıyor: µm kaydırmayla yeniden dene
        _delete(base)
        ctx.work.objects.unlink(A)
        out_col.objects.link(A)
        A.name = f"UP_{seg.key}"
        chkA["cut_failed"] = True
        return Built(seg, A, chkA, {"notes": ["kesim manifold değil"]})
    cav_failed = False
    inner = None
    shell = seg.shell and not seg.solid
    if shell and seg.cover:                             # ince kaplama: ortalama kalınlık ≤ 2,5 × et → dolu basılır
        chk = mesh_check(A.data)
        bm = bmesh.new()
        bm.from_mesh(A.data)
        area = sum(f.calc_area() for f in bm.faces)
        bm.free()
        wall = seg.wall if seg.wall is not None else _zone(seg.zone)[1]
        t_mean = 2.0 * chk["volume_m3"] / max(area, 1e-12)
        if t_mean <= THIN_COVER_FACTOR * wall:
            shell = False
            feat.notes.append(f"ince kaplama (ortalama {t_mean * 1000:.1f} mm) — dolu basılır")
    if shell:
        cav_base = base
        if seg.cover:                                   # açık kapak: gömülü kısım dahil özgün katı kaydırılır
            cav_base = ctx.copy(ctx.outer(src), "cbase")
            if seg.ends:
                reg = ctx.obj("regx", region_solid(_planes(seg.ends, LOCAL_MARGIN), center))
                B.op(cav_base, reg, "INTERSECT", solver=ctx.cut_solver)
                _delete(reg)
        inner, cav = segment_cavity(ctx, seg, cav_base, seg.wall_source or src)
        if cav_base is not base:
            _delete(cav_base)
        if seg.cavity_keep_fn is not None:              # et < 2 × duvar + 0,4 mm olan yerde boşluk yok → dolu (AERO-20)
            for ks in seg.cavity_keep_fn() or []:
                ko = ctx.obj("ckeep", ks)
                B.op(cav, ko, "DIFFERENCE" if ks.name == "trim_hole" else "INTERSECT")
                _delete(ko)
        for e in seg.ends:
            if e.kind == "plate" and e.holes:
                feat.holes += rib_holes(ctx, seg, e, feat, inner)
        col = B.group(ctx.name("st"), feat.structure)
        if col is not None:
            B.op(cav, col, "DIFFERENCE")
            Booler.drop(col)
        if feat.keepout:                                # iç yapı bu hacimlere girmez (ünite, beşik, kanal): boşluğa geri ekle
            cav0 = ctx.obj("cav0", inner)
            creg = ctx.obj("creg", region_solid(_cavity_planes(seg.ends, seg.cavity_planes), center if center is not None
                                                else inner.V.mean(0)))
            B.op(cav0, creg, "INTERSECT")
            _delete(creg)
            solid = [x for x in feat.structure if x.name.startswith("solid")]   # R14: dolu bant keepout'a üstün
            sg = B.group(ctx.name("sol"), solid) if solid else None
            # her keepout AYRI kesişir: koleksiyonla INTERSECT n'li kesişimdir (ayrık iki keepout → boş; halka 5'te iki
            # kanat çerçevesi, kök bloklarında yatak parçaları — iç yapı keepout içinde kalıyordu, montaj çakışması)
            for i_k, ks in enumerate(feat.keepout):
                ck = ctx.copy(cav0, "cavk")
                ko = ctx.obj("ko", ks)
                B.op(ck, ko, "INTERSECT")
                _delete(ko)
                if sg is not None and len(ck.data.polygons):
                    B.op(ck, sg, "DIFFERENCE")
                if len(ck.data.polygons):
                    B.op(cav, ck, "UNION")
                _delete(ck)
            Booler.drop(sg)
            _delete(cav0)
        v_cav = abs(mesh_check(cav.data)["volume_m3"])
        v_before = mesh_check(A.data)["volume_m3"]
        B.op(A, cav, "DIFFERENCE")
        _delete(cav)
        v_after = mesh_check(A.data)["volume_m3"]
        if v_cav < 0.08 * v_before or (v_before - v_after) < 0.25 * min(v_cav, v_before) or v_after <= 0.0:
            cav_failed = True                           # boolean reddedildi / boşluk boş: parça dolu ya da bozuk → yeniden dene
            feat.notes.append("boşluk çıkarılamadı")
    _delete(base)
    for k in seg.union_extra:
        if ctx.has(k):
            B.op(A, ctx.outer(k), "UNION")
    col = B.group(ctx.name("add"), feat.adds)
    if col is not None:
        if seg.ends:                                    # eklenenler bölgeye kırpılır (komşu segmente taşmasın)
            reg = ctx.obj("rega", region_solid(_planes(seg.ends), center))
            for ob in list(col.objects):
                B.op(ob, reg, "INTERSECT")
                if len(ob.data.polygons) == 0:
                    _delete(ob)
            _delete(reg)
        if len(col.objects):
            B.op(A, col, "UNION")
        Booler.drop(col)
    col = B.group(ctx.name("hole"), feat.holes)
    if col is not None:
        B.op(A, col, "DIFFERENCE")
        Booler.drop(col)
    cl = finalize_mesh(A)
    me = A.data
    ctx.work.objects.unlink(A)
    out_col.objects.link(A)
    A.name = f"UP_{seg.key}"
    me.name = A.name
    chk = mesh_check(me)
    chk["removed_islands"] = cl["removed_islands"]
    if cav_failed:
        chk["ok"] = False
        chk["cavity_failed"] = True
    return Built(seg, A, chk, {"inner": inner, "pin_holes": sum(1 for h in feat.holes if h.name == "pin"),
                               "notes": list(feat.notes)})


# =====================================================================================================
# Özellik üreticileri (dünya = Blender ekseni; spec → Blender: X = −s)
# =====================================================================================================
def Bp(s: float, y: float = 0.0, z: float = 0.0) -> np.ndarray:
    return np.asarray(P.to_blender(s, y, z), float)


def Bv(v) -> np.ndarray:
    return np.asarray(P.vec_to_blender(v), float)


def _line_at(p0: np.ndarray, p1: np.ndarray, y: float) -> np.ndarray:
    """``p0 → p1`` doğrusunda (Blender) Y = ``y`` noktası (doğrusal)."""
    t = (y - p0[1]) / (p1[1] - p0[1])
    return p0 + t * (p1 - p0)


COLLAR_L = 0.016                                    # kovan bileziği boyu (R04: sürekli kovan yerine bilezikler)
COLLAR_GAP = 0.055                                  # bilezikler arası en çok boşluk


def collar_spans(L: float, length: float = COLLAR_L, gap: float = COLLAR_GAP) -> list[tuple[float, float]]:
    """``[0, L]`` boyunca bilezik aralıkları: iki uçta birer bilezik (segment ucu kaburgası/flanşı + kör uç omzu) ve
    aralar ``gap``'i aşmayacak kadar eşit aralıklı ara bilezikler. Kısa boyda tek sürekli kovan."""
    if L <= 2 * length + gap:
        return [(0.0, L)]
    k = int(math.ceil((L - length) / (length + gap)))           # boşluk sayısı
    step = (L - length) / k
    return [(i * step, i * step + length) for i in range(k + 1)]


def tube_prims(p0, p1, od: float, *, sleeve: bool = True, clear: float | None = None, ext: float = 0.0005,
               web_up=None, web_h: float = 0.08, name: str = "tube", collars: bool = False) -> Feature:
    """CF boru kanalı: delik (Ø od + malzeme boşluğu, çevrel çokgen; P7) + kılavuz kovanı (iç yapı) + isteğe bağlı
    kesme ağı (``web_up`` yönünde). ``collars``: sürekli kovan yerine ``collar_spans`` bilezikleri (R04 hafifletme:
    boru kabuğa segment uçlarında ve ≤ 55 mm aralıklı bileziklerde yapışır; ağ boru boyunca süreklidir)."""
    p0, p1 = np.asarray(p0, float), np.asarray(p1, float)
    d = _unit(p1 - p0)
    a, b = p0 - d * ext, p1 + d * ext
    dh = od + clear if clear is not None else hole_d(od, "tube")
    r = 0.5 * dh
    n = hole_sides(r)
    f = Feature()
    f.holes.append(prim_hole(a, b, dh, name=f"{name}_hole", n=n))
    if sleeve:                                          # kovan uçları deliği 2 mm aşar: kademe omzu / kör uç kapağı
        s0, s1 = a - d * SHOULDER, b + d * SHOULDER
        L = float(np.linalg.norm(s1 - s0))
        spans = collar_spans(L) if collars else [(0.0, L)]
        for u0, u1 in spans:
            f.structure.append(prim_frustum(s0 + d * u0, s0 + d * u1, (r + SLEEVE_T) / math.cos(math.pi / n), n=n,
                                            name=f"{name}_sleeve"))
    if web_up is not None:
        up = _unit(np.asarray(web_up, float) - d * float(np.dot(web_up, d)))
        nrm = np.cross(d, up)
        L = float(np.linalg.norm(b - a))
        f.structure.append(prim_box(0.5 * (a + b), np.column_stack([d, up, nrm]), (0.5 * L, web_h, 0.5 * WEB_T),
                                    name=f"{name}_web"))
    return f


def web_prim(p0, p1, up, h: float = 0.08, t: float = WEB_T) -> Solid:
    p0, p1 = np.asarray(p0, float), np.asarray(p1, float)
    d = _unit(p1 - p0)
    up = _unit(np.asarray(up, float) - d * float(np.dot(up, d)))
    return prim_box(0.5 * (p0 + p1), np.column_stack([d, up, np.cross(d, up)]),
                    (0.5 * float(np.linalg.norm(p1 - p0)), h, 0.5 * t), name="web")


def _section_frame(sec: np.ndarray) -> tuple:
    """Selig sıralı kesit (spec (s, y, z)) → (LE, veter birim, kalınlık birim, üst eğri, alt eğri) — 2B (x, t)."""
    n = (len(sec) + 1) // 2
    le = sec[n - 1]
    te = 0.5 * (sec[0] + sec[-1])
    c = _unit(te - le)
    d = sec[n // 2] - sec[n - 1 + n // 2]                # üst − alt yüzey (veter ortası): kalınlık yönü (dikeyde de)
    t = _unit(d - c * float(d @ c))
    up = sec[:n][::-1]
    lo = sec[n - 1:]
    to2 = lambda P3: np.column_stack([(P3 - le) @ c, (P3 - le) @ t])
    return le, c, t, to2(up), to2(lo)


def _thick_at(up2: np.ndarray, lo2: np.ndarray, x):
    xu = np.maximum.accumulate(up2[:, 0])
    xl = np.maximum.accumulate(lo2[:, 0])
    zu = np.interp(x, xu, up2[:, 1])
    zl = np.interp(x, xl, lo2[:, 1])
    return zu - zl, 0.5 * (zu + zl)


def band_prism(stations: Sequence, section_fn: Callable, x_fn: Callable, ext: float = 0.012,
               h: float = 0.03) -> Solid:
    """Kesit ailesi boyunca kayış prizma (spec kesitlerinden): her istasyonda ``x_fn(chord, up2, lo2) → (x0, x1)``
    veter aralığında, kalınlık yönünde ±h dikdörtgen; uçlar açıklık yönünde ``ext`` uzatılır."""
    rings = []
    secs = [section_fn(t) for t in stations]
    for sec in secs:
        le, c, tt, up2, lo2 = _section_frame(sec)
        chord = float(np.max(up2[:, 0]))
        x0, x1 = x_fn(chord, up2, lo2)
        _, zm0 = _thick_at(up2, lo2, min(max(x0, 0.0), chord))
        _, zm1 = _thick_at(up2, lo2, min(max(x1, 0.0), chord))
        pts = [le + c * x0 + tt * (zm0 - h), le + c * x1 + tt * (zm1 - h), le + c * x1 + tt * (zm1 + h),
               le + c * x0 + tt * (zm0 + h)]
        rings.append(np.array(pts))
    sd = _unit(np.mean(rings[-1], 0) - np.mean(rings[0], 0))
    rings[0] = rings[0] - sd * ext
    rings[-1] = rings[-1] + sd * ext
    V = np.vstack([P.points_to_blender(r) for r in rings])
    F = []
    m = len(rings)
    for k in range(m - 1):
        for j in range(4):
            j2 = (j + 1) % 4
            F.append((4 * k + j, 4 * k + j2, 4 * (k + 1) + j2, 4 * (k + 1) + j))
    F.append((3, 2, 1, 0))
    F.append(tuple(4 * (m - 1) + j for j in range(4)))
    s = Solid(V, F, "band")
    if _signed_volume(s.V, s.F) < 0:
        s = Solid(V, [tuple(reversed(f)) for f in F], "band")
    return s


def te_band(stations, section_fn, wall: float, gap: float = 0.0008, margin: float = 0.0004) -> Solid:
    """Firar kenarı dolu bölgesi: kalınlığın ``2·et + gap + margin``'den az olduğu arka şerit (boşluk burada yok)."""
    need = 2 * wall + gap + margin

    def xf(chord, up2, lo2):
        xs = np.linspace(0.5 * chord, chord, 400)
        th, _ = _thick_at(up2, lo2, xs)
        ok = np.flatnonzero(th < need)
        x0 = xs[ok[0]] if len(ok) else chord - 0.002
        return x0 - 0.0015, chord + 0.004

    return band_prism(stations, section_fn, xf)


def hinge_zone(h: P.HingeLine, wall: float, extra: float = HINGE_ZONE_EXTRA) -> Solid:
    """Menteşe oyuğu çevresi dolu bölge (dudakların ince kaması): eksen boyunca, yarıçap r_oyuk + et + extra."""
    X = np.asarray(h.axis_outboard_b)
    a = np.asarray(h.p_in_b) - X * (h.gap_m + 0.002)
    b = np.asarray(h.p_out_b) + X * (h.gap_m + 0.002)
    return prim_frustum(a, b, h.nose_radius_in + h.gap_m + wall + extra, h.nose_radius_out + h.gap_m + wall + extra,
                        name="hinge_zone")


def servo_bay(center, up, span_dir, chord_dir, host: Solid, size=SERVO["open"], box_t: float = SERVO["box_t"],
              clear_top: float = 0.0012) -> Feature:
    """Servo yuvası: ``up`` (dışa) yüzünde ``size`` (açıklık boyu, veter boyu) açıklık + çevresinde ``box_t`` kutu
    duvarı; derinlik karşı deriye ``clear_top`` kalana dek (kaynak katıya ışınla ölçülür)."""
    up, sd, cd = _unit(up), _unit(span_dir), _unit(chord_dir)
    cd = _unit(cd - sd * float(sd @ cd))
    up = _unit(np.cross(sd, cd)) * np.sign(float(np.cross(sd, cd) @ up))
    bvh = BVHTree.FromPolygons(host.V.tolist(), [list(f) for f in host.F], epsilon=0.0)
    c = np.asarray(center, float)
    hit_out = bvh.ray_cast(Vector(c), Vector(up), 0.2)
    hit_in = bvh.ray_cast(Vector(c), Vector(-up), 0.2)
    if hit_out[0] is None or hit_in[0] is None:
        return Feature(notes=["servo yuvası: kesit bulunamadı"])
    outer_pt = np.array(hit_out[0])
    far_pt = np.array(hit_in[0])
    depth = float((outer_pt - far_pt) @ up)
    # açıklığın en ince köşesine göre derinlik (deri eğimi: tavan karşı deriyi delmesin)
    span_far = []
    for a in (-0.5, 0.5):
        for b in (-0.5, 0.5):
            q = c + sd * (a * (size[0] + 2 * box_t)) + cd * (b * (size[1] + 2 * box_t))
            h = bvh.ray_cast(Vector(q), Vector(-up), 0.2)
            if h[0] is not None:
                span_far.append(float((outer_pt - np.array(h[0])) @ up))
    if span_far:
        depth = min([depth] + span_far)
    hole_len = max(0.004, depth - 0.0012 - clear_top)
    axes = np.column_stack([sd, cd, up])
    f = Feature()
    mid = outer_pt - up * (0.5 * hole_len - 0.004)
    f.holes.append(prim_box(mid, axes, (0.5 * size[0], 0.5 * size[1], 0.5 * hole_len + 0.004), name="servo_open"))
    f.structure.append(prim_box(outer_pt - up * (0.5 * depth), axes,
                                (0.5 * size[0] + box_t, 0.5 * size[1] + box_t, 0.5 * depth + 0.004), name="servo_box"))
    f.notes.append(f"servo yuvası {size[0] * 1000:.0f}×{size[1] * 1000:.0f} mm, derinlik {hole_len * 1000:.1f} mm")
    return f


def pin_boss(p0, p1, ends: Sequence[End], d: float = BOSS_D) -> Feature:
    """Hizalama pimi göbeği (iç yapı) ve göbek ekseninin pim'li uç düzlemlerini kestiği yerlerde Ø3 pim delikleri."""
    p0, p1 = np.asarray(p0, float), np.asarray(p1, float)
    f = Feature()
    ax = _unit(p1 - p0)
    for e in ends:                                      # göbek yalnız pimli uçlarda (pim boyu + 6 mm; hafiflik)
        if e.kind == "open" or not e.pins:
            continue
        den = float(ax @ e.n)
        if abs(den) < 0.2:
            continue
        t = float((e.p - p0) @ e.n) / den
        q = p0 + ax * t
        inward = -ax * np.sign(den)
        f.structure.append(prim_frustum(q - inward * 0.003, q + inward * (PIN_DEPTH + 0.006), 0.5 * d, n=24,
                                        name="boss"))
        f.holes.append(prim_hole(q + e.n * 0.002, q - e.n * PIN_DEPTH, hole_d(PIN_D, "pin"), name="pin"))
    return f


# ------------------------------------------------------------------ menteşe planı (dil + çentik + pim)
@dataclass
class HingePlan:
    """Bir kumanda yüzeyinin basılı menteşe planı: eksen (Blender), dil merkezleri (eksen boyunca u, m)."""

    h: P.HingeLine
    a: np.ndarray                  # p_in (Blender)
    X: np.ndarray                  # eksen birim (dışa / yukarı)
    Y: np.ndarray                  # veterde geriye
    Z: np.ndarray
    L: float
    lugs: list = field(default_factory=list)          # u değerleri
    lug_up: dict = field(default_factory=dict)        # u → sabit parçanın baskı yukarı yönü işareti (X boyunca)

    def point(self, u: float) -> np.ndarray:
        return self.a + self.X * u

    def r_nose(self, u: float) -> float:
        t = min(max(u / self.L, 0.0), 1.0)
        return self.h.nose_radius_in + (self.h.nose_radius_out - self.h.nose_radius_in) * t

    def u_of_plane(self, p, n) -> float | None:
        den = float(self.X @ n)
        if abs(den) < 1e-6:
            return None
        return float((np.asarray(p) - self.a) @ n) / den


def plan_hinge(h: P.HingeLine, fixed_cuts: Sequence[float], surf_cuts: Sequence[float],
               fixed_up_sign: Callable[[float], float]) -> HingePlan:
    """Dilleri yüzey segmentlerine dağıtır: her segmentte 2 (kısa segmentte 1); sabit/yüzey kesimlerinden ve
    uçlardan pah + çentik boyu kadar uzak tutar."""
    F = h.frame_b()
    X = np.asarray(h.axis_outboard_b)
    Y = F[:, 1]
    Z = np.cross(X, Y)
    a = np.asarray(h.p_in_b)
    L = h.length
    hp = HingePlan(h, a, X, Y, Z, L)
    bounds = [0.0] + sorted(u for u in surf_cuts if 0.0 < u < L) + [L]
    fixed = sorted(u for u in fixed_cuts if 0.0 < u < L)

    def ok(u):
        rn = hp.r_nose(u)
        wedge = rn + rn + h.gap_m + 0.002
        if u < 0.012 + wedge or u > L - 0.012 - wedge:
            return False
        for c in fixed + bounds[1:-1]:
            if abs(u - c) < 0.5 * LUG_W + wedge + 0.006:
                return False
        return True

    for k in range(len(bounds) - 1):
        lo, hi = bounds[k], bounds[k + 1]
        span = hi - lo
        targets = [lo + 0.5 * span] if span < 0.10 else [lo + 0.25 * span, lo + 0.75 * span]
        for u0 in targets:
            for du in [0.0] + [s * d for d in np.arange(0.002, 0.04, 0.002) for s in (1, -1)]:
                u = u0 + du
                if lo < u < hi and ok(u) and all(abs(u - v) > 0.03 for v in hp.lugs):
                    hp.lugs.append(u)
                    hp.lug_up[u] = fixed_up_sign(u)
                    break
    hp.lugs.sort()
    return hp


LUG_R = 0.0045                                      # dil diski yarıçapı (pimin çevresinde 3,4 mm et; R04: eski disk
                                                    # burun yarıçapı − 0,6 mm idi → 9 mm'ye varan disk + dev 45° kama)


def lug_radius(hp: HingePlan, u: float) -> float:
    return min(hp.r_nose(u) - 0.0006, LUG_R)


def hinge_lug_prims(hp: HingePlan, u: float) -> list:
    """Sabit parçaya eklenen dil: pim ekseni çevresinde disk (``lug_radius``) + oyuk duvarına köprü + baskı yönünde
    45° destek kaması. R04: disk burun yarıçapı yerine pimin çevresindeki ete göre (≤ 4,5 mm) boyutlanır; köprü ve
    kama da buna göre küçülür (dil başına ≈ 5,3 → 1,9 cm³; köprü 4,7 × 7,2 mm kesit, 10 N menteşe yükünde ≈ 3 MPa)."""
    h = hp.h
    rn = hp.r_nose(u)
    rl = lug_radius(hp, u)
    rc = rn + h.gap_m
    c = hp.point(u)
    X, Y, Z = hp.X, hp.Y, hp.Z
    out = [prim_frustum(c - X * 0.5 * LUG_W, c + X * 0.5 * LUG_W, rl, n=24, name="lug_disc")]
    depth = rc + 0.004                                  # oyuk duvarının 4 mm içine
    out.append(prim_box(c - Y * (0.5 * depth - 0.0002), np.column_stack([X, Y, Z]),
                        (0.5 * LUG_W - 0.00015, 0.5 * depth, 0.8 * rl), name="lug_bridge"))
    sgn = hp.lug_up.get(u, 1.0)                         # sabit parçanın yukarı yönü X boyunca (+/−)
    xb = -sgn * (0.5 * LUG_W - 0.0003)                  # tabla tarafı yüzü (diske 0,3 mm gömülü; eş düzlem yok)
    run = rl + depth
    y0 = -depth + 0.0004
    tri = np.array([[xb, y0], [xb, rl - 0.0002], [xb - sgn * (run - 0.0006), y0]])
    if sgn < 0:
        tri = tri[::-1]
    out.append(prim_prism(tri, c, X, Y, Z, -0.74 * rl, 0.74 * rl, name="lug_wedge"))
    return out


def hinge_notch_prims(hp: HingePlan, u: float, surf_up_sign: float = 1.0) -> list:
    """Yüzeyde dil çentiği (R04/R14): dil dilimi + sabit tarafın kama tarafına ``run`` boyunca
    (a) burun ÖN yarısı kutu (Y ≤ 0,5 mm; köprü ±30° sapmada bu yarıda kalır) — arka yüzü veter yönüne dik düzlem,
    deriyi dik keser (eski tam silindir çentik arka derileri teğet sıyırıp < 0,1 mm kıymık bırakıyordu);
    (b) eksen çevresinde disk + kama dönme payı silindiri (r = 1,24·r_dil + 0,8 mm; küçük burunda kutu arkası buna
    uzar); (c) yüzeyin baskı yukarı ucunda 45° çatı (prizma + koni)."""
    h = hp.h
    rn = hp.r_nose(u)
    rl = lug_radius(hp, u)
    rc = rn + h.gap_m
    c = hp.point(u)
    X, Y, Z = hp.X, hp.Y, hp.Z
    sgn = hp.lug_up.get(u, 1.0)
    run = rl + rc + 0.004 + 0.0005
    lo = -0.5 * LUG_W - LUG_CLEAR - (run if sgn > 0 else 0.0)
    hi = 0.5 * LUG_W + LUG_CLEAR + (run if sgn < 0 else 0.0)
    r_in = 1.24 * rl + 0.0008                           # disk + kama köşesi dönme yarıçapı + pay
    zr = rn + 0.004                                     # kutu derileri kalınlık yönünde tümüyle aşar (yatay kesim yüzü yok)
    y_aft = 0.0005 if r_in < rn - 0.0008 else r_in + 0.0003
    y_fwd = -(rc + 0.0015)
    axes = np.column_stack([X, Y, Z])
    out = [prim_box(c + X * (0.5 * (lo + hi)) + Y * (0.5 * (y_fwd + y_aft)), axes,
                    (0.5 * (hi - lo), 0.5 * (y_aft - y_fwd), zr), name="notch_box")]
    out.append(prim_frustum(c + X * lo, c + X * hi, r_in, n=24, name="notch"))
    top = (hi if surf_up_sign > 0 else lo) - surf_up_sign * 0.0003
    tri = np.array([[top, -zr], [top, zr], [top + surf_up_sign * zr, 0.0]])
    if _poly_area2(tri) < 0:                            # prim_prism: (eu, ev) düzleminde saat yönü tersi
        tri = tri[::-1]
    roof = prim_prism(tri, c, X, Z, -Y, -y_aft, -y_fwd, name="notch_roof")
    if _signed_volume(roof.V, roof.F) < 0:
        roof = Solid(roof.V, [tuple(reversed(f)) for f in roof.F], roof.name)
    out.append(roof)
    out.append(prim_frustum(c + X * top, c + X * (top + surf_up_sign * (r_in + 0.0003)), r_in + 0.0001, 0.0,
                            n=23, name="notch_cone"))
    return out


# Menteşe pimi takma tarafı (P1): delik, sabit parçada eksen boyunca dışarı açılır (u = 0 iç/alt uç, L dış/üst uç).
# (u0, u1 − L) m. Kanatçık: uç kapağından dışarı (Ø2,1 çıkış deliği, pimden sonra CA ile tıkanır); elevatör:
# stabilize ucundan (dikey takılmadan önce; dikey 1'in yuvası pimi tutar); dümen: dikey 2'nin tepesinden; flaplar:
# y = 0,36 kaburgalarından (panel sökülüyken; karşı kaburga/karşı pim tutar).
PIN_BORE_EXT = {"Aileron": (-0.006, 0.120), "Elevator": (-0.006, 0.045), "Rudder": (-0.006, 0.060),
                "FlapOut": (-0.006, 0.006), "FlapIn": (-0.006, 0.006)}
PIN_INSERT = {
    "Aileron": ("dış uç", "uç kapağının dış yüzündeki Ø2,1 delikten (uç kapağı yapıştırıldıktan sonra)",
                "iç uçta kör delik (dış panel / dış flap ucu); dış uçta delik CA + mikrobalonla tıkanır"),
    "FlapOut": ("iç uç", "dış panel sökülüyken panel 1'in y = 0,36 kök kaburgasındaki delikten",
                "panel takılınca kök bloğu 2 kaburgası ve iç flap piminin ucu; dış uçta kör delik (kanatçık ayrımı)"),
    "FlapIn": ("dış uç", "dış panel sökülüyken kök bloğu 2'nin y = 0,36 kaburgasındaki delikten",
               "panel takılınca panel 1 kaburgası ve dış flap piminin ucu; iç uçta kör delik (kök bloğu 1)"),
    "Elevator": ("dış uç", "dikeyler takılmadan önce stabilize ucundaki delikten",
                 "dikey 1'in stabilize ucu yuvası (dikey takılınca elevatör pimi sökülemez); iç uçta kör delik"),
    "Rudder": ("üst uç", "dikey 2'nin tepesindeki Ø2,1 delikten",
               "alt uçta kör delik (dikey 1); üst delik CA ile tıkanır"),
}


def hinge_pin_bore(hp: HingePlan, u0: float = -0.006, u1: float | None = None) -> Solid:
    """Menteşe pimi deliği: Ø2,1 çevrel çokgen (iç yarıçap 1,05 mm; Ø1,75 filament pim). ``u0``: iç/alt uç (eksen
    koordinatı), ``u1``: dış/üst uçta L'nin ötesine uzama (varsayılan 6 mm). Takma tarafında delik komşu parçadan
    dışarı uzatılır (``PIN_BORE_EXT``, P1)."""
    u1 = hp.L + (0.006 if u1 is None else u1)
    return prim_hole(hp.a + hp.X * u0, hp.a + hp.X * u1, HINGE_PIN_HOLE_D, name="pin_bore")


# =====================================================================================================
# Parça tarifleri
# =====================================================================================================
SIDE = "L"
S_CUT_STRAKE = float(P.SPEC["wing"]["le_root_s_m"]) + 0.015      # glove strake | kök bloğu ayrım düzlemi. varsayım
SERVO_STATIONS = {"Aileron": 1.30, "FlapOut": 0.68, "Elevator": 0.17, "Rudder": 0.10, "FlapIn": 0.20}   # varsayım
REAR_WEB_ROOT_CF = 0.60                                            # kök bloğu arka ağı (açı pimi hattı) veter oranı
TIP_JOINT_SHIFT = 0.002                                            # uç kapağı eki, eğik düzlemden 2 mm dışarıda
CAP_INSET = 0.0025                                                 # açık uç düzlemi kaynak kapağından > et uzakta (perde yuvası 5 mm)


def _yplane(y: float, sign: float) -> tuple:
    return Bp(0.0, y, 0.0), np.array([0.0, sign, 0.0])


def _splane(s: float, sign: float) -> tuple:
    """s düzlemi; ``sign`` = +1 → bölge s ≥ s0 (dışa normal +X_b = s'nin azaldığı yön)."""
    return Bp(s, 0.0, 0.0), np.array([sign, 0.0, 0.0])


def wing_section_b(y: float) -> np.ndarray:
    return P.wing_section(y)


def _wing_le_boss_points(y: float, wall: float = _WALLS["d_box"]) -> tuple[np.ndarray, np.ndarray]:
    """Hücum kenarı pim göbeği (kalınlığın göbek + 2·D-kutu olduğu ilk veter noktası) ve arka göbek (%80)."""
    sec = P.wing_section(y)
    le, c, t, up2, lo2 = _section_frame(sec)
    chord = float(np.max(up2[:, 0]))
    xs = np.linspace(0.002, 0.3 * chord, 300)
    th, zm = _thick_at(up2, lo2, xs)
    need = BOSS_D + 2 * wall - 0.0016                    # göbek iki deriye ~0,8 mm gömülür (yüzer kolon olmasın)
    i = int(np.argmax(th >= need)) if np.any(th >= need) else len(xs) - 1
    x = xs[i] + 0.5 * BOSS_D
    _, z = _thick_at(up2, lo2, x)
    p_le = le + c * x + t * z
    return Bp(*p_le), None


def _te_band_wing(y0: float, y1: float, wall: float) -> Solid:
    ys = np.unique(np.r_[np.arange(y0, y1, 0.05), y1])
    return te_band(ys, lambda y: P.wing_section(y), wall)


def _te_band_stab(y0: float, y1: float, wall: float) -> Solid:
    ys = np.unique(np.r_[np.arange(y0, y1, 0.04), y1])
    return te_band(ys, lambda y: P.stab_section(y), wall)


def _te_band_fin(h0: float, h1: float, wall: float, side: str = SIDE) -> Solid:
    hs = np.unique(np.r_[np.arange(h0, h1, 0.03), h1])
    return te_band(hs, lambda h: P.fin_section(h, side), wall)


@dataclass
class Recipe:
    """Tarif + özellik üreticisi ve rapor bilgisi."""

    seg: SegSpec
    feat_fn: Callable[[SegSpec], Feature] | None = None


def _wall_of(zone: str) -> float:
    return _zone(zone)[1]


def _spar_pieces() -> list:
    """Ana kiriş boru parçaları (y aralığı → spec borusu): soket Ø30 (0–0,40), panel Ø27 (0,40–1,10), uç Ø16 (1,10–1,70)."""
    T = {t.name: t for t in P.spar_tubes(SIDE)}
    out = []
    for name, y0, y1 in (("centre_socket", 0.0, 0.40), ("panel_tube", 0.40, 1.10), ("tip_tube", 1.10, 1.70)):
        t = T[name]
        out.append((name, y0, y1, Bp(*t.p0), Bp(*t.p1), t.od))
    return out


def _spar_line() -> tuple[np.ndarray, np.ndarray]:
    T = {t.name: t for t in P.spar_tubes(SIDE)}
    t = T["panel_tube"]
    return Bp(*t.p0), Bp(*t.p1)


def _rear_line() -> tuple[np.ndarray, np.ndarray]:
    T = {t.name: t for t in P.spar_tubes(SIDE)}
    t = T["rear_spar"]
    return Bp(*t.p0), Bp(*t.p1)


def _root_rear_line() -> tuple[np.ndarray, np.ndarray]:
    """Kök bloğu arka ağı (açı pimi hattı, %60 veter) — y 0 → 0,40."""
    pts = []
    for y in (0.0, 0.40):
        st = P.wing_station(y)
        s = P.WING_SPAR_S + (REAR_WEB_ROOT_CF - P.WING_SPAR_FRAC) * st.chord_ref
        z = P.wing_mid_z(s, y)
        pts.append(Bp(s, y, z if z is not None else st.z_ref))
    return pts[0], pts[1]


def wing_features(y0: float, y1: float, part: str, ends: Sequence[End], plans: dict, up_sign: float,
                  host: Solid, servo: bool = True) -> Feature:
    """Kanat segmenti (sol): boru kanalları + kovanlar + ağlar, arka kiriş, açı pimi, ±40° kafes, pim göbekleri,
    servo yuvası, menteşe bölgeleri/dilleri/pim deliği, firar kenarı dolu şeridi."""
    f = Feature()
    lo, hi = y0 - 0.001, y1 + 0.001
    Zb = np.array([0.0, 0.0, 1.0])
    # ana kiriş
    for name, a, b, p0, p1, od in _spar_pieces():
        ya, yb = max(a, lo), min(b, hi)
        if yb <= ya:
            continue
        f.extend(tube_prims(_line_at(p0, p1, ya), _line_at(p0, p1, yb), od, name=name, collars=True))
    s0, s1 = _spar_line()
    f.structure.append(web_prim(_line_at(s0, s1, y0 - 0.03), _line_at(s0, s1, y1 + 0.03), Zb))
    # arka kiriş / kök arka ağı
    r0, r1 = _rear_line()
    if part == "outer":
        ya, yb = max(0.38, lo), min(1.82, hi)
        if yb > ya:
            f.extend(tube_prims(_line_at(r0, r1, ya), _line_at(r0, r1, yb), 0.008, name="rear", collars=True))
        f.structure.append(web_prim(_line_at(r0, r1, y0 - 0.03), _line_at(r0, r1, y1 + 0.03), Zb))
    else:
        q0, q1 = _root_rear_line()
        f.structure.append(web_prim(_line_at(q0, q1, y0 - 0.03), _line_at(q0, q1, y1 + 0.03), Zb))
    # açı pimi (dolu CF Ø6, y 0,31–0,41, %60)
    T = {t.name: t for t in P.spar_tubes(SIDE)}
    ip = T["incidence_pin"]
    a, b = Bp(*ip.p0), Bp(*ip.p1)
    ya, yb = max(0.31, lo), min(0.41, hi)
    if yb > ya:
        f.extend(tube_prims(_line_at(a, b, ya), _line_at(a, b, yb), ip.od, name="inc_pin"))
    # ±40° geodezik kafes (dış panel): ana ağ ile arka ağ arasında; dik baskıda ağ yüzleri yatayla 50° (P12)
    if part == "outer" and y1 <= P.wing_parts()["U_WingOuter"][1] + 0.01 and y0 < LATTICE_Y_MAX:
        for k in range(-3, 40):
            yk = 0.36 + k * LATTICE_PITCH
            for sg in (1.0, -1.0):
                sa = float(-s0[0]) + 0.004
                sb = float(-_line_at(r0, r1, yk)[0]) - 0.004
                dy = sg * (sb - sa) / math.tan(math.radians(LATTICE_DEG))
                ya_, yb_ = yk, yk + dy
                if max(ya_, yb_) < y0 - 0.02 or min(ya_, yb_) > y1 + 0.02:
                    continue
                za = P.wing_mid_z(sa, max(yk, 0.37)) or 0.0
                zb = P.wing_mid_z(sb, min(max(yb_, 0.37), 1.82)) or 0.0
                pa, pb = Bp(sa, ya_, za), Bp(sb, yb_, zb)
                f.structure.append(web_prim(pa, pb, Zb, h=0.06, t=LATTICE_T))
    # hücum kenarı pim göbeği
    pa, _ = _wing_le_boss_points(max(y0, 0.001))
    pb, _ = _wing_le_boss_points(min(y1, 1.893))
    if part != "root1":
        f.extend(pin_boss(pa, pb, ends))
    else:                                       # kök 1: arka ağ üzerinde pim (hücum kenarı strake'te)
        q0, q1 = _root_rear_line()
        f.extend(pin_boss(_line_at(q0, q1, y0 - 0.01), _line_at(q0, q1, y1 + 0.01), [e for e in ends if abs(e.n[1]) > 0.9]))
    # servo yuvası
    if servo:
        for nm, ys in (("FlapOut", SERVO_STATIONS["FlapOut"]), ("Aileron", SERVO_STATIONS["Aileron"])):
            if y0 + 0.03 < ys < y1 - 0.03:
                m0, m1 = _spar_pieces()[1 if ys < 1.1 else 2][3:5]
                sm = -_line_at(m0, m1, ys)[0] + 0.5 * (0.027 if ys < 1.1 else 0.016) + SLEEVE_T + 0.003
                sr = -_line_at(r0, r1, ys)[0] - 0.004 - 0.003
                sc = 0.5 * (sm + sr)
                zc = P.wing_mid_z(sc, ys) or 0.0
                f.extend(servo_bay(Bp(sc, ys, zc), np.array([0, 0, -1.0]), np.array([0, 1.0, 0]),
                                   np.array([-1.0, 0, 0]), host))
    # menteşe bölgeleri, diller, pim deliği
    for nm, hp in plans.items():
        h = hp.h
        if h.span_to < y0 - 0.005 or h.span_from > y1 + 0.005:
            continue
        f.structure.append(hinge_zone(h, _WALLS["wing_skin"] if part == "outer" else _wall_of("root_block")))
        for u in hp.lugs:
            y = float(hp.point(u)[1])
            if y0 < y < y1:
                f.adds += hinge_lug_prims(hp, u)
        f.holes.append(hinge_pin_bore(hp, *PIN_BORE_EXT.get(nm, (-0.006, None))))
    f.structure.append(_te_band_wing(max(y0 - 0.02, 0.0), min(y1 + 0.02, 1.9), _WALLS["wing_skin"]
                                     if part == "outer" else _wall_of("root_block")))
    return f


def _tip_plane() -> tuple[np.ndarray, np.ndarray]:
    """Dış panel | uç kapağı ayrım düzlemi: kanatçık dış ucundaki menteşeye dik düzlem (shapes ile aynı)."""
    from .. import shapes as S
    h = P.hinge_line("Aileron", SIDE)
    pt, nrm = S.hinge_plane(S.WingFamily(SIDE), h, h.span_to)
    n = _unit(Bv(nrm))
    return Bp(*pt) + n * TIP_JOINT_SHIFT, n              # birleşim kıymıkları (panel|uç arayüzü) dışında kal


def _stab_q(y: float) -> np.ndarray:
    st = P.stab_station(y)
    return Bp(st.le_s + 0.25 * st.chord, y, st.z)


def _stab_dir() -> np.ndarray:
    return _unit(_stab_q(0.5) - _stab_q(0.0))


def _fin_q(h: float, side: str = SIDE) -> np.ndarray:
    st = P.fin_station(h, side)
    return Bp(st.le[0] + 0.25 * st.chord, st.le[1], st.le[2])


def _fin_dir(side: str = SIDE) -> np.ndarray:
    return _unit(_fin_q(0.30, side) - _fin_q(0.02, side))


def _fin_hplane(h: float, side: str = SIDE) -> tuple[np.ndarray, np.ndarray]:
    st = P.fin_station(h, side)
    return Bp(*st.le), _unit(Bv(st.span_dir))


def _fus_cuts() -> list[float]:
    """Gövde segment sınırları: ``params.print_cuts()['fuselage_s']`` + burun ayrımı; iç sınırlar kuyu/köprü,
    stabilize eyeri ve hava alığı bölgelerinden en yakın kenara kaydırılır (segment ≤ tabla Z)."""
    cuts = [c for c in P.print_cuts()["fuselage_s"] if c <= float(P.SPEC["fuselage"]["modules"]["firewall_s_m"]) + 1e-9]
    t = P.TURRET
    from .. import shapes as S
    s_nose = round(t.s - 0.5 * t.collar_d - S.TURRET_COLLAR_FLARE - 0.001, 3)   # taret yuvası tek parçada (0,148)
    cuts = sorted(set(cuts) | {s_nose})
    cuts[-1] = cuts[-1] - CAP_INSET                                # yangın perdesi: kaynağın kapağı bölge dışında
    I = P.intake_spec()
    wing_box = P.SPEC["fuselage"]["internals"]["wing_box_s_m"]
    wells = [np.asarray(w.outline, float)[:, 0] for w in P.gear_wells() if w.name != "N"]
    keep_out = [(min(wing_box[0], min(w.min() for w in wells)) - 0.005, max(w.max() for w in wells) + 0.005),
                (P.SPEC["fuselage"]["internals"]["stab_saddle_s_m"][0] - 0.005,
                 P.SPEC["fuselage"]["internals"]["stab_saddle_s_m"][1] + 0.005),
                (float(I["s_from_m"]) - 0.008, float(I["s_to_m"]) + 0.04)]
    out = list(cuts)
    for i in range(1, len(out) - 1):
        for a, b in keep_out:
            if a < out[i] < b:
                out[i] = a if (out[i] - a) < (b - out[i]) else b
    return [round(c, 4) for c in sorted(set(out))]


def fuselage_cuts() -> list[float]:
    """Gövdenin GERÇEK baskı ek istasyonları (s, m; burun ucu hariç, yangın perdesi kapağı dahil): ``_fus_cuts`` —
    render'daki panel çizgileri (``materials.panel_stations``) bunları kullanır, rapordaki ``print_cuts`` ile aynıdır."""
    return [c for c in _fus_cuts() if c > 0.0]


def make_recipes(ctx: Ctx, only: Sequence[str] | None = None) -> tuple[list, dict]:
    """Bütün baskı tariflerini üretir (sol/merkez). Dönüş: (Recipe listesi, menteşe planları)."""
    from .. import shapes as S
    cuts = P.print_cuts()
    W = P.wing_parts()
    y_join = float(P.SPEC["wing"]["panel_joint_y_m"])
    rb = cuts["root_block_y"]
    y_mid_root = rb[1]
    tip_p, tip_n = _tip_plane()
    recipes: list[Recipe] = []

    def want(key: str) -> bool:
        return not only or any(key.startswith(o) for o in only)

    # ------------------------------------------------------------------ kaynaklar
    # uç kökü (y = 1,83) kapağı panel kapağıyla tam çakışır; EXACT birleşim bunu temiz çözer. Eski 30 µm öteleme
    # birleşimde 30 µm basamak + çakışık köşe bırakıyordu (STL'de µm aralık → uç %1,1, panel 7 %0,45 ince örnek; R14)
    ctx.add_source("wing_outer", ["U_WingOuter_L", "U_Tip_L"], _wall_dbox(_WALLS["wing_skin"]))
    ctx.add_source("wing_center", ["U_WingCenter_L"], _wall_dbox(_wall_of("root_block")))
    ctx.add_source("stab", ["U_Stab"], _wall_const(_WALLS["tail_skin"]))
    ctx.add_source("fin", ["U_Fin_L"], _wall_const(_WALLS["tail_skin"]))
    ctx.add_source("fus", ["U_Fuselage"], _wall_fuselage)
    ctx.add_source("fus_clean", [], _wall_fuselage, solid_fn=lambda: Solid(S.fuselage().verts_blender(),
                                                                           S.fuselage().faces, "fus_clean"))
    wc_md = None

    def _wing_center_clean():
        nonlocal wc_md
        if wc_md is None:
            wc_md = S.wing_parts(SIDE)[f"U_WingCenter_{SIDE}"]
        return Solid(wc_md.verts_blender(), wc_md.faces, "wing_center_clean")

    ctx.add_source("wing_center_clean", [], _wall_dbox(_wall_of("root_block")), solid_fn=_wing_center_clean)
    # R01: sahnedeki U_Cowl gerçek 1,6 mm kabuktur (motor zarfları iç boşlukta); baskı kaynağı aynı dış yüzün kapalı
    # katısıdır (S.cowl(print_solid=True)) — printprep kendisi oyar, arka açıklık ve çene yarığını keser.
    ctx.add_source("cowl", [], _wall_const(_WALLS["cowl_pa_cf"]),
                   solid_fn=lambda: md_solid(S.cowl(print_solid=True), "cowl"))
    ctx.add_source("exhaust_ring", [], _wall_const(_WALLS["cowl_pa_cf"]), solid_fn=exhaust_ring_solid)
    ctx.add_source("louvers", [], _wall_const(_WALLS["cowl_pa_cf"]), solid_fn=lambda: merge_solids(louver_lip_solids(), "louvers"))
    ctx.add_source("intake", ["U_Intake"], _wall_const(_wall_of("intake")))
    ctx.add_source("root_fairing", ["U_Fairing_Root_L"], _wall_const(_wall_of("root_fairing")))
    ctx.add_source("gear_blister", ["U_Fairing_Blister_L"], _wall_const(_wall_of("root_fairing")))
    ctx.add_source("collar", ["U_Turret_Mount"], _wall_const(_wall_of("turret_collar")))
    ctx.add_source("hatch", ["U_Hatch"], _wall_const(_WALLS["hatch_petg"]))
    ctx.add_source("scuff", ["U_ScuffPad"], _wall_const(float(P.SPEC["propulsion"]["cowl"].get("scuff_pad", {})
                                                             .get("t_m", 0.002))))
    for d in ("N_1", "N_3", "L_1", "L_2"):                     # kapak + menteşe dili/horn donanımı (U_DoorHw_*) tek parça
        hw = [f"U_DoorHw_{d}"] if f"U_DoorHw_{d}" in bpy.data.objects else []
        ctx.add_source(f"door_{d}", [f"U_Door_{d}"] + hw, _wall_const(_wall_of("gear_doors")), merge=True)
    for nm in P.CONTROL_SURFACES:
        ctx.add_source(f"surf_{nm}", [f"U_{nm}_{SIDE}"], _wall_const(_WALLS["control_surface_skin"]))

    # ------------------------------------------------------------------ kanat segment uçları
    wing_ends: dict[str, list] = {}
    ys = cuts["wing_panel_y"]
    n_panel = len(ys) - 2
    for k in range(n_panel):
        lo = End(*_yplane(ys[k], -1.0), "plate", pins=(k > 0), label=f"y={ys[k]:.3f}")
        if k == n_panel - 1:
            hi = End(tip_p, tip_n, "land", label="uç düzlemi")
        else:
            hi = End(*_yplane(ys[k + 1], 1.0), "land", label=f"y={ys[k + 1]:.3f}")
        wing_ends[f"wing_panel_{k + 1}"] = [lo, hi]
    wing_ends["wing_tip"] = [End(tip_p, -tip_n, "plate", label="uç düzlemi")]
    wing_ends["root_block_2"] = [End(*_yplane(y_mid_root, -1.0), "land", label=f"y={y_mid_root:.3f}"),
                                 End(*_yplane(y_join, 1.0), "plate", pins=False, label=f"y={y_join:.3f} (sökülebilir)")]
    wing_ends["root_block_1"] = [End(*_yplane(y_mid_root, 1.0), "plate", label=f"y={y_mid_root:.3f}"),
                                 End(*_splane(S_CUT_STRAKE, 1.0), "plate", label="strake ayrımı"),
                                 End(*_yplane(0.0, -1.0), "open", label="gövde içi")]
    wing_ends["glove_strake"] = [End(*_splane(S_CUT_STRAKE, -1.0), "plate", label="strake ayrımı"),
                                 End(*_yplane(y_mid_root, 1.0), "plate", label=f"y={y_mid_root:.3f}"),
                                 End(*_yplane(0.0, -1.0), "open", label="gövde içi")]

    def up_of(ends, bed_idx):
        return -ends[bed_idx].n

    bed_of = {k: 0 for k in wing_ends}
    bed_of["root_block_2"] = 1
    bed_of["root_block_1"] = 0
    bed_of["glove_strake"] = 0

    # ------------------------------------------------------------------ kuyruk segment uçları
    d_st = _stab_dir()
    q_st = _stab_q(cuts["stab_y"][1])
    stab_ends = {"stab_1": [End(q_st, d_st, "plate", label="stabilize eki"), End(*_yplane(0.0, -1.0), "open")],
                 "stab_2": [End(q_st, -d_st, "plate", label="stabilize eki")]}
    d_fn = _fin_dir()
    q_fn = _fin_q(cuts["fin_h"][1])
    fin_ends = {"fin_1": [End(q_fn, d_fn, "plate", label="dikey eki")],
                "fin_2": [End(q_fn, -d_fn, "plate", label="dikey eki")]}

    # ------------------------------------------------------------------ kumanda yüzeyi kesimleri ve menteşe planları
    surf_cut_y = {"Aileron": cuts["aileron_y"], "FlapOut": cuts["flap_out_y"], "FlapIn": cuts["flap_in_y"],
                  "Elevator": cuts["elevator_y"], "Rudder": []}
    host_ends = {"Aileron": [k for k in wing_ends if k.startswith("wing_")],
                 "FlapOut": [k for k in wing_ends if k.startswith("wing_panel")],
                 "FlapIn": ["root_block_1", "root_block_2"], "Elevator": ["stab_1", "stab_2"], "Rudder": ["fin_1", "fin_2"]}
    all_ends = {**wing_ends, **stab_ends, **fin_ends}
    bed_all = {**bed_of, "stab_1": 0, "stab_2": 0, "fin_1": 0, "fin_2": 0}
    plans: dict[str, HingePlan] = {}
    surf_cuts_u: dict[str, list] = {}
    for nm in P.CONTROL_SURFACES:
        h = P.hinge_line(nm, SIDE)
        a = np.asarray(h.p_in_b)
        X = np.asarray(h.axis_outboard_b)
        su = []
        for yv in surf_cut_y[nm][1:-1]:
            t = (yv - h.span_from) / (h.span_to - h.span_from)
            su.append(t * h.length)
        surf_cuts_u[nm] = su
        fixed_u, up_at = [], []
        for hk in host_ends[nm]:
            ends = all_ends[hk]
            for e in ends:
                den = float(X @ e.n)
                if abs(den) > 1e-3:
                    fixed_u.append(float((e.p - a) @ e.n) / den)
            up = up_of(ends, bed_all[hk])
            up_at.append((hk, ends, float(np.sign(X @ up)) or 1.0))

        def sign_at(u, up_at=up_at, a=a, X=X):
            q = a + X * u
            for hk, ends, sg in up_at:
                if all(float((q - e.p) @ e.n) <= 1e-6 for e in ends):
                    return sg
            return 1.0

        plans[nm] = plan_hinge(h, fixed_u, su, sign_at)
    _HINGE_PLANS.clear()
    _HINGE_PLANS.update(plans)                          # özel parçalar (takım yatağı) komşu menteşe dillerini görsün

    # ------------------------------------------------------------------ kanat tarifleri
    def wing_recipe(key, name_tr, src, ends, bed, y0, y1, part, subtract=(), cavity_planes=(), note=""):
        zone = {"outer": "wing_panel", "tip": "tip_cap", "root1": "root_block", "root2": "root_block",
                "strake": "glove_strake"}[part]
        sel = {nm: plans[nm] for nm in ("Aileron", "FlapOut")} if part in ("outer", "tip") else \
            ({"FlapIn": plans["FlapIn"]} if part in ("root1", "root2") else {})
        seg = SegSpec(key, name_tr, "Kanat", [src], zone, ends, mirror=True, bed_end=bed, subtract=list(subtract),
                      force_up=(part != "strake"),
                      cavity_planes=list(cavity_planes), note=note,
                      center=Bp(1.27, 0.5 * (y0 + y1), 0.0))
        up_sign = 1.0

        def ff(seg, y0=y0, y1=y1, part=part, sel=sel):
            host = ctx.solid(src)
            if part == "strake":
                f = Feature()
                for yb in (0.13, 0.20):
                    zm = P.wing_mid_z(S_CUT_STRAKE, yb) or -0.04
                    pa, pb = Bp(S_CUT_STRAKE - 0.014, yb, zm), Bp(S_CUT_STRAKE + 0.001, yb, zm)
                    f.extend(pin_boss(pa, pb, seg.ends))
                pa, _ = _wing_le_boss_points(0.20)
                pb, _ = _wing_le_boss_points(y_mid_root)
                f.extend(pin_boss(pa, pb, [e for e in seg.ends if abs(e.n[1]) > 0.9]))
                f.structure.append(web_prim(Bp(S_CUT_STRAKE - 0.03, 0.0, -0.04), Bp(S_CUT_STRAKE - 0.03, 0.26, -0.03),
                                            np.array([0, 0, 1.0])))
                return f
            f = wing_features(y0, y1, "outer" if part in ("outer", "tip") else part, seg.ends, sel, up_sign, host,
                              servo=(part == "outer"))
            if part == "root1":
                for yb in (0.13, 0.20):
                    zm = P.wing_mid_z(S_CUT_STRAKE, yb) or -0.04
                    pa, pb = Bp(S_CUT_STRAKE - 0.001, yb, zm), Bp(S_CUT_STRAKE + 0.014, yb, zm)
                    f.extend(pin_boss(pa, pb, [e for e in seg.ends if abs(e.n[0]) > 0.9]))
                f.extend(g10_slot_features(0.05))                    # P4: G10 köprü yuvaları (gövde yanından)
                f.structure += well_roof_fill()                      # kuyu tavanı–üst deri ≤ 4 mm: dolu bant
            if part in ("root1", "root2"):                           # P4: ana takım yatağı / ER-150 hacmi + alt açıklık
                f.keepout += gear_mount_main_keepout()
                f.holes.append(gear_unit_opening())
                f.notes.append("ana takım yatağı yuvası + ER-150 alt açıklığı")
                f.holes += conduit_holes(root_conduit_polyline(), CONDUIT_D["root"])     # P9: kablo kanalı Ø6
            if part == "outer":
                f.holes += conduit_holes(wing_conduit_polyline(), CONDUIT_D["wing"])     # P9: servo kabloları Ø8
            if seg.key == "wing_panel_1" or part == "root2":                 # P9: sökülebilir ek
                f.extend(lock_tab_features("root2" if part == "root2" else "panel"))
                f.extend(mpx_pocket("root" if part == "root2" else "panel"))
                f.holes.append(flap_joiner_slot())
                f.notes.append("iç/dış flap bağlayıcı tel yay yarığı")
            if part in ("outer", "root1", "root2"):
                f.notes.append("kablo kanalı")
            return f

        return Recipe(seg, ff)

    for k in range(n_panel):
        key = f"wing_panel_{k + 1}"
        y0, y1 = ys[k], (ys[k + 1] if k < n_panel - 1 else float(tip_p[1]))
        if want(key):
            recipes.append(wing_recipe(key, f"Kanat dış paneli {k + 1}/{n_panel} (y {y0:.2f}–{y1:.2f})", "wing_outer",
                                       wing_ends[key], 0, y0, y1, "outer"))
    if want("wing_tip"):
        recipes.append(wing_recipe("wing_tip", "Kanat uç kapağı (eğik uç, seyrüsefer LED yuvası)", "wing_outer",
                                   wing_ends["wing_tip"], 0, float(tip_p[1]), 1.9, "tip",
                                   cavity_planes=[_yplane(1.893, 1.0)]))
    fus_sub = [("fus_clean", 0.0003, 1.0)]
    if want("root_block_2"):
        recipes.append(wing_recipe("root_block_2", f"Kök bloğu 2 (y {y_mid_root:.3f}–{y_join:.2f})", "wing_center",
                                   wing_ends["root_block_2"], 1, y_mid_root, y_join, "root2"))
    if want("root_block_1"):
        recipes.append(wing_recipe("root_block_1", f"Kök bloğu 1 (gövde yanı–y {y_mid_root:.3f})", "wing_center",
                                   wing_ends["root_block_1"], 0, 0.06, y_mid_root, "root1", subtract=fus_sub,
                                   note="gövdeye oturan kök yüzü gövde konturunu izler (0,3 mm boşluk)"))
    if want("glove_strake"):
        recipes.append(wing_recipe("glove_strake", "Glove strake (36° kök hücum kenarı)", "wing_center",
                                   wing_ends["glove_strake"], 0, 0.06, y_mid_root, "strake", subtract=fus_sub))

    # ------------------------------------------------------------------ kuyruk
    def stab_ff(seg, idx):
        f = Feature()
        T = {t.name: t for t in P.spar_tubes(SIDE)}
        for nm, od in (("stab_spar", 0.012), ("stab_rear_rod", 0.006)):
            t = T[nm]
            f.extend(tube_prims(Bp(*t.p0), Bp(*t.p1), od, web_up=np.array([0, 0, 1.0]), web_h=0.03, name=nm))
        hp = plans["Elevator"]
        f.structure.append(hinge_zone(hp.h, _WALLS["tail_skin"]))
        for u in hp.lugs:
            q = hp.point(u)
            if all(float((q - e.p) @ e.n) < -0.004 for e in seg.ends if e.kind != "open"):
                f.adds += hinge_lug_prims(hp, u)
        f.holes.append(hinge_pin_bore(hp, *PIN_BORE_EXT["Elevator"]))
        f.structure.append(_te_band_stab(0.0, P.STAB_HALF_SPAN + 0.006, _WALLS["tail_skin"]))
        f.holes += conduit_holes(tail_conduit_polyline(), CONDUIT_D["tail"])        # P9: kuyruk kablo kanalı (uçtan)
        f.notes.append("kablo kanalı Ø6 (dümen + elevatör → koni)")
        # hücum kenarı pimi (ek düzleminde)
        sec_pts = []
        for yv in (0.20, 0.32):
            sec = P.stab_section(yv)
            le, c, tt, up2, lo2 = _section_frame(sec)
            xs = np.linspace(0.002, 0.04, 100)
            th, zm = _thick_at(up2, lo2, xs)
            i = int(np.argmax(th >= BOSS_D + 2 * _WALLS["tail_skin"] - 0.0016))
            sec_pts.append(Bp(*(le + c * (xs[i] + 0.5 * BOSS_D) + tt * zm[i])))
        f.extend(pin_boss(sec_pts[0], sec_pts[1], [e for e in seg.ends if e.kind == "plate"]))
        if idx == 1:
            ys_ = SERVO_STATIONS["Elevator"]
            st = P.stab_station(ys_)
            s_sp = st.le_s + float(P.SPEC["tail"]["stab"]["spar"]["offset_from_le_m"]) + 0.006 + SLEEVE_T + 0.002
            s_rr = st.le_s + 0.5 * st.chord - 0.003 - SLEEVE_T - 0.002
            sc = 0.5 * (s_sp + s_rr)
            f.extend(servo_bay(Bp(sc, ys_, st.z), np.array([0, 0, -1.0]), np.array([0, 1.0, 0]),
                               np.array([-1.0, 0, 0]), ctx.solid("stab"),
                               size=(SERVO["open"][0], min(SERVO["open"][1], (s_rr - s_sp) - 2 * SERVO["box_t"]))))
        else:
            fs = T["fin_spar"]
            a, b = Bp(*fs.p0), Bp(*fs.p1)
            d = _unit(b - a)
            f.extend(tube_prims(a - d * 0.018, a + d * 0.02, 0.008, name="fin_spar_root"))
        return f

    for idx, key in ((1, "stab_1"), (2, "stab_2")):
        if want(key):
            seg = SegSpec(key, f"Stabilize yarısı {idx}/2 (%25 ok eksenine dik ek)", "Kuyruk", ["stab"], "stab",
                          stab_ends[key], mirror=True, bed_end=0, force_up=True,
                          subtract=(fus_sub + [("cowl", 0.0008, 0.0)] if idx == 1 else []),
                          center=Bp(2.05, 0.13 if idx == 1 else 0.39, 0.065),
                          note=("kök yüzü kuyruk konisi konturunu izler; borular koni içindeki eyerde birleşir"
                                if idx == 1 else "uç dikey içine 5 mm gömülür"))
            recipes.append(Recipe(seg, lambda seg, idx=idx: stab_ff(seg, idx)))

    def fin_ff(seg, idx):
        f = Feature()
        T = {t.name: t for t in P.spar_tubes(SIDE)}
        fs = T["fin_spar"]
        a, b = Bp(*fs.p0), Bp(*fs.p1)
        d = _unit(b - a)
        f.extend(tube_prims(a - d * 0.018, b, 0.008, web_up=Bv(P.fin_station(0.16, SIDE).normal), web_h=0.05,
                            name="fin_spar"))                # kesme ağı derilere dik (R04: eski ağ dikeyin orta düzlemindeydi)
        hp = plans["Rudder"]
        f.structure.append(hinge_zone(hp.h, _WALLS["tail_skin"]))
        for u in hp.lugs:
            q = hp.point(u)
            if all(float((q - e.p) @ e.n) < -0.004 for e in seg.ends):
                f.adds += hinge_lug_prims(hp, u)
        f.holes.append(hinge_pin_bore(hp, *PIN_BORE_EXT["Rudder"]))
        f.structure.append(_te_band_fin(0.0, 0.32, _WALLS["tail_skin"]))
        if idx == 1:
            f.holes += conduit_holes(tail_conduit_polyline()[:2], CONDUIT_D["tail"])  # P9: dümen servosu → stabilize ucu
            f.notes.append("kablo kanalı Ø6 (servo → stabilize ucu)")
        pts = []
        for hv in (0.10, 0.22):
            sec = P.fin_section(hv, SIDE)
            le, c, tt, up2, lo2 = _section_frame(sec)
            xs = np.linspace(0.002, 0.05, 100)
            th, zm = _thick_at(up2, lo2, xs)
            i = int(np.argmax(th >= BOSS_D + 2 * _WALLS["tail_skin"] - 0.0016))
            pts.append(Bp(*(le + c * (xs[i] + 0.5 * BOSS_D) + tt * zm[i])))
        f.extend(pin_boss(pts[0], pts[1], seg.ends))
        if idx == 1:
            hv = SERVO_STATIONS["Rudder"]
            st = P.fin_station(hv, SIDE)
            fsp = float(P.SPEC["tail"]["fin"]["spar"]["offset_from_le_m"])
            s_a = st.le[0] + fsp + 0.004 + SLEEVE_T + 0.003
            s_b = st.le[0] + 0.65 * st.chord - (hp.r_nose(0.05) + 0.001 + 0.0025 + 0.004)
            sc = 0.5 * (s_a + s_b)
            nrm = Bv(st.normal)
            inboard = -nrm if nrm[1] > 0 else nrm
            centre = Bp(sc, st.le[1], st.le[2])
            f.extend(servo_bay(centre, inboard, Bv(st.span_dir), np.array([-1.0, 0, 0]), ctx.solid("fin"),
                               size=(SERVO["open"][0], min(SERVO["open"][1], s_b - s_a - 2 * SERVO["box_t"]))))
        return f

    for idx, key in ((1, "fin_1"), (2, "fin_2")):
        if want(key):
            hp0, hn0 = _fin_hplane(0.002)
            hp1, hn1 = _fin_hplane(float(P.SPEC["tail"]["fin"]["height_m"]) - 0.0065)
            cp = [(hp0, -hn0)] if idx == 1 else [(hp1, hn1)]
            seg = SegSpec(key, f"Dikey {idx}/2 (%25 ok eksenine dik ek)", "Kuyruk", ["fin"], "fin", fin_ends[key],
                          mirror=True, bed_end=0, force_up=True,
                          subtract=([("stab", 0.0003, 1.0)] if idx == 1 else []),
                          cavity_planes=cp, center=Bp(2.3, 0.55, 0.12 if idx == 1 else 0.3),
                          note=("iç yüzde stabilize ucu yuvası (0,3 mm boşluk)" if idx == 1 else ""))
            recipes.append(Recipe(seg, lambda seg, idx=idx: fin_ff(seg, idx)))

    # ------------------------------------------------------------------ kumanda yüzeyleri
    def surf_ff(seg, nm, u0, u1):
        hp = plans[nm]
        h = hp.h
        f = Feature()
        a, b = hp.point(u0 - 0.01), hp.point(u1 + 0.01)
        f.structure.append(prim_frustum(a, b, HINGE_SLEEVE_R, n=24, name="pin_sleeve"))
        # R04: menteşe ekseninde DİK ağ (burun deri–pim kovanı–burun deri; burun hücresi kapanır). Eski veter
        # düzlemindeki orta levha (ön %40, 0,8 mm) derilere değmiyordu, ağırdı (≈ 2,5 g / segment).
        f.structure.append(prim_box(0.5 * (a + b), np.column_stack([hp.X, hp.Y, hp.Z]),
                                    (0.5 * float(np.linalg.norm(b - a)), 0.5 * CS_WEB_T, 0.03), name="hinge_web"))
        f.holes.append(hinge_pin_bore(hp))
        up_sign = 1.0
        for u in hp.lugs:
            if u0 - 0.03 < u < u1 + 0.03:
                f.holes += hinge_notch_prims(hp, u, up_sign)
        if h.kind == "wing":
            f.structure.append(_te_band_wing(max(h.span_from - 0.01, 0.0), h.span_to + 0.01, _WALLS["control_surface_skin"]))
        elif h.kind == "stab":
            f.structure.append(_te_band_stab(0.0, P.STAB_HALF_SPAN, _WALLS["control_surface_skin"]))
        else:
            f.structure.append(_te_band_fin(0.0, 0.32, _WALLS["control_surface_skin"]))
        if nm == "FlapIn" and u1 > hp.L - 0.01:                     # P9: dış flaba bağlayıcı tel yuvası
            f.holes.append(flap_joiner_socket("in"))
            f.notes.append("bağlayıcı tel yuvası (dış flap sürer)")
        if nm == "FlapOut" and u0 < 0.01:
            f.holes.append(flap_joiner_socket("out"))
            f.notes.append("bağlayıcı tel yuvası (iç flabı sürer)")
        st = SERVO_STATIONS.get(nm) if nm != "FlapIn" else None   # iç flap servosuz (bağlayıcı tel)
        if st is not None:
            us = (st - h.span_from) / (h.span_to - h.span_from) * h.length
            if u0 + 0.01 < us < u1 - 0.01:
                q = hp.point(us)
                down = np.array([0, 0, -1.0]) if h.kind != "fin" else -_unit(np.array([0.0, np.sign(q[1]), 0.0]))
                down = _unit(down - hp.X * float(down @ hp.X))
                side = np.cross(down, hp.X)
                c = q + hp.Y * (hp.r_nose(us) + 0.004 + 0.5 * HORN_SLOT[1]) + down * 0.012
                f.holes.append(prim_box(c, np.column_stack([hp.X, _unit(np.cross(hp.X, down)), down]),
                                        (0.5 * HORN_SLOT[0], 0.5 * HORN_SLOT[1], 0.012), name="horn_slot"))
                f.notes.append("G10 horn yarığı")
        return f

    for nm in P.CONTROL_SURFACES:
        hp = plans[nm]
        bounds = [0.0] + surf_cuts_u[nm] + [hp.L]
        nseg = len(bounds) - 1
        label = {"Aileron": "Kanatçık", "FlapOut": "Dış flap", "FlapIn": "İç flap", "Elevator": "Elevatör",
                 "Rudder": "Dümen"}[nm]
        group = "Kumanda yüzeyleri"
        for k in range(nseg):
            key = f"{nm.lower()}_{k + 1}" if nseg > 1 else nm.lower()
            if not want(key) and not want(nm.lower()):
                continue
            ends = []
            if k > 0:
                ends.append(End(hp.point(bounds[k]), -hp.X, "plate", label="yüzey eki"))
            if k < nseg - 1:
                ends.append(End(hp.point(bounds[k + 1]), hp.X, "plate", label="yüzey eki"))
            sub = [("cowl", 0.0010, 0.0)] if (nm == "Elevator" and k == 0) else []   # kök: kaporta yanağı çakışması
            seg = SegSpec(key, f"{label} {k + 1}/{nseg}" if nseg > 1 else label, group, [f"surf_{nm}"],
                          "control_surface", ends, mirror=True, bed_end=(0 if k > 0 else None),
                          up_hint=hp.X, center=hp.point(0.5 * (bounds[k] + bounds[k + 1])), force_up=True,
                          subtract=sub,
                          note=f"menteşe ekseni Z'de; Ø{HINGE_PIN_D * 1000:.2f} filament pim, basılı dil çentikleri"
                          .replace(".", ","))
            recipes.append(Recipe(seg, lambda seg, nm=nm, u0=bounds[k], u1=bounds[k + 1]: surf_ff(seg, nm, u0, u1)))

    # ------------------------------------------------------------------ gövde
    fcuts = _fus_cuts()
    s_flange = float(P.SPEC["fuselage"]["modules"]["flange_s_m"])
    s_fw = float(P.SPEC["fuselage"]["modules"]["firewall_s_m"])
    s_nose_mod = float(P.SPEC["fuselage"]["modules"]["nose_module_s_m"][1])
    nose_split = fcuts[1]
    ring_no = 0
    for k in range(len(fcuts) - 1):
        s0, s1 = fcuts[k], fcuts[k + 1]
        if s0 >= s_fw - 0.002:
            continue
        lo_kind, hi_kind = "frame", "land"
        bed = 0
        if k == 0:
            key, name_tr = "fus_nose_cone", f"Burun konisi (s 0–{s1:.3f})"
            ends = [End(*_splane(s1, -1.0), "frame", label=f"s={s1:.3f}")]
            bed = 0
        elif s1 <= s_nose_mod + 1e-6:
            key, name_tr = "fus_nose_module", f"Burun görev modülü halkası (s {s0:.3f}–{s1:.3f}, taret yuvası)"
            ends = [End(*_splane(s0, 1.0), "land", label=f"s={s0:.3f}"),
                    End(*_splane(s1, -1.0), "frame", label=f"s={s1:.3f} (sökülebilir)")]
            bed = 1
        else:
            ring_no += 1
            zone = "tail_cone" if s0 >= s_flange - 1e-6 else "fuselage_mid"
            key = f"fus_ring_{ring_no}"
            name_tr = (f"Kuyruk konisi halkası (s {s0:.3f}–{s1:.3f}, LW-ASA)" if zone == "tail_cone"
                       else f"Gövde halkası {ring_no} (s {s0:.3f}–{s1:.3f})")
            ends = [End(*_splane(s0, 1.0), "frame", label=f"s={s0:.3f}"),
                    End(*_splane(s1, -1.0), "land", label=f"s={s1:.3f}")]
        zone = "fuselage_nose" if s1 <= s_nose_mod + 1e-6 else ("tail_cone" if s0 >= s_flange - 1e-6 else "fuselage_mid")
        if not want(key) and not want("fus"):
            continue
        s_int = float(P.intake_spec()["throat_s"])
        sub = [("intake", GLUE_GAP, 0.0)] if (s0 < s_int + 0.02 and s1 > s_int - 0.02 and ctx.has("intake")) else []
        seg = SegSpec(key, name_tr, "Gövde", ["fus"], zone, ends, bed_end=bed, center=Bp(0.5 * (s0 + s1), 0.0, 0.05),
                      force_up=True, subtract=sub,
                      note=("NACA dudak çerçevesi cebi + yangın perdesine kanal geçişi" if sub else ""))
        recipes.append(Recipe(seg, lambda seg, s0=s0, s1=s1: fuselage_features(ctx, seg, s0, s1)))

    # ------------------------------------------------------------------ yük yolu parçaları (P4)
    def custom(key, name_tr, material, parts_fn, mirror=False, up=None, note=""):
        if not want(key):
            return
        seg = SegSpec(key, name_tr, "Yük yolları", [], "frames", [], material=material, shell=False, mirror=mirror,
                      up_hint=up, note=note,
                      builder=lambda ctx_, seg_, col_, fn=parts_fn: build_custom(ctx_, seg_, col_, fn()))
        recipes.append(Recipe(seg, None))

    custom("wing_frame_fwd", "Kanat kutusu ön çerçevesi (PA-CF 3 mm; G10 köprüye yaslanır)", "PA-CF",
           lambda: wing_frame_parts(0), up=np.array([1.0, 0, 0]),
           note="köprü plakasına epoksi + 2 × M4; 4 longeron soketi; depo zarfı çevresinde halka")
    custom("wing_frame_aft", "Kanat kutusu arka çerçevesi (PA-CF 3 mm; G10 köprüye yaslanır)", "PA-CF",
           lambda: wing_frame_parts(1), up=np.array([-1.0, 0, 0]),
           note="köprü plakasına epoksi + 2 × M4; 4 longeron soketi")
    custom("gear_mount_L", "Ana takım yatağı (PA-CF; ER-150 beşiği + sokete eyerli kol)", "PA-CF",
           gear_mount_main_parts, mirror=True,
           note="kök bloğu 2'nin açık iç ucundan kaydırılır; ünite ön duvara 2 × M3 ısıl gömme dişli")
    custom("gear_mount_N", "Burun takım yatağı (PA-CF; chine longeronlarına eyer, akü kızağı tabanı)", "PA-CF",
           gear_mount_nose_parts, up=np.array([0, 0, 1.0]),
           note="halka 1'in ön çerçevesinden kaydırılır; G10 plaka 4 × M3 ısıl gömme dişli (M3×3)")
    custom("engine_ring", "Motor halkası (PA-CF; yangın perdesi önü, 4 × M4 motor + 6 × M3 kaporta dişlisi)", "PA-CF",
           engine_ring_parts, up=np.array([1.0, 0, 0]),
           note="halka 9'a 30 mm bindirerek yapıştırılır; 4 longeron ucu soketlere girer")
    custom("panel_lock_tab", "Dış panel tutma dili (PETG; alttan M4 naylon cıvata, ısıl gömme dişli)", "PETG",
           lock_tab_parts, mirror=True, note="dış panel 1 kök kaburgasına yapıştırılır; kök bloğu 2 cebine girer")
    for kind, nm in (("chine", "chine"), ("shoulder", "omuz")):
        custom(f"longeron_joint_{kind}", f"Longeron kırık soketi — {nm} (PETG; s = {longeron_break_s():.3f})", "PETG",
               lambda kind=kind: longeron_joint_parts(kind), mirror=True,
               note="iki düz longeron parçası kırık açısıyla buluşur (halka 4–5 eki)")

    if want("fus_nose_flange") or want("fus"):
        seg = SegSpec("fus_nose_flange", "Burun modülü PETG bağlantı flanşı (4×M4, 2 pim)", "Gövde", ["fus_clean"],
                      "frames", [], material="PETG", wall=0.003, shell=False, bed_end=None,
                      up_hint=np.array([-1.0, 0, 0]))
        recipes.append(Recipe(seg, None))

    # ------------------------------------------------------------------ kaporta, lüle, hava alığı, kapaklar
    cw = P.SPEC["propulsion"]["cowl"]
    _, s_cowl_end, _ = cowl_s_range()
    cheeks = cowl_cheek_cuts()

    def cheek_region(sd):
        c = cheeks[sd]
        return region_solid([(p, n) for p, n, _ in c["planes"]], c["center"])

    def cheek_zones(sd):                                # kaporta tarafındaki ek flanşları (yanak açıklığı çevresi)
        c = cheeks[sd]
        out = []
        for i, (p, n, kind) in enumerate(c["planes"]):
            if kind != "frame":
                continue
            band = FRAME_T if abs(n[0]) > 0.99 else COWL_CUT_BAND
            clip = tuple((q, m, 0.008) for j, (q, m, _) in enumerate(c["planes"]) if j != i)
            out.append(WallZone(p, n, band, 0.0002, extra=COWL_CHEEK_FRAME_W, clip=clip))
        return out

    if want("cowl"):
        front = End(*_splane(s_fw + CAP_INSET, 1.0), "frame", width=0.008, label="yangın perdesi")
        front_cav = (Bp(s_cowl_end - _WALLS["cowl_pa_cf"] - 0.0001, 0, 0), np.array([-1.0, 0, 0]))
        seg = SegSpec("cowl_top", "Motor kaportası (PA-CF; yanak açıklıkları flanşlı, arka Ø92 açık)", "İtki", ["cowl"],
                      "cowl", [front], bed_end=0, cutouts=[cheek_region("L"), cheek_region("R")],
                      cavity_planes=[front_cav], wall_zones=cheek_zones("L") + cheek_zones("R"),
                      center=Bp(2.13, 0, 0.13))
        recipes.append(Recipe(seg, lambda seg: cowl_features(ctx, seg)))
        for sd, nm in (("L", "Sol yanak (susturucu tarafı)"), ("R", "Sağ yanak (panjurlu)")):
            c = cheeks[sd]
            ends = []
            for p, n, kind in c["planes"]:
                if kind == "frame" and abs(n[0]) > 0.99 and n[0] > 0:      # ön düzlem: yangın perdesi kapağı dışında
                    p = Bp(max(c["s"][0], s_fw + CAP_INSET), 0, 0)
                ends.append(End(p, n, kind, width=COWL_CHEEK_FRAME_W if kind == "frame" else 0.0,
                                thick=(COWL_CUT_BAND if kind == "frame" and abs(n[0]) < 0.99 else 0.0)))
            sg = 1.0 if sd == "L" else -1.0
            seg = SegSpec(f"cowl_cheek_{sd}", f"Kaporta {nm.lower()}", "İtki", ["cowl"], "cowl", ends,
                          union_extra=(["louvers"] if sd == "R" else []), bed_end=0,
                          cavity_planes=[front_cav] if sd == "L" else [],
                          up_hint=np.array([0, sg, 0.0]), center=c["center"],
                          note=f"eğik ekler deriye dik (deri normali {', '.join(f'{a:.0f}°' for a in c['angles_deg'])})")
            recipes.append(Recipe(seg, lambda seg: cheek_features(ctx, seg)))
        seg = SegSpec("exhaust_ring", "Lüle halkası (PA-CF)", "İtki", ["exhaust_ring"], "exhaust_ring", [],
                      bed_end=None, up_hint=np.array([1.0, 0, 0]))
        recipes.append(Recipe(seg, None))
        sp = cw.get("scuff_pad", {})
        sp_mat = str(sp.get("material", "PA-CF")).upper()
        seg = SegSpec("scuff_pad", f"Kaporta altı sürtünme pabucu ({sp_mat}; kuyruk çarpmasında ilk değen kenar)",
                      "İtki", ["scuff"], "cowl", [], material=sp_mat, wall=float(sp.get("t_m", 0.002)),
                      shell=False, up_hint=np.array([0, 0, 1.0]), force_up=True,
                      builder=lambda ctx_, seg_, col_: build_scuff_pad(ctx_, seg_, col_),
                      note="kaporta dış yüzünü izleyen 2 mm katman (yapışma yüzü + 0,2 mm); aşınma yüzü tablada; "
                           "yüksek sıcaklık epoksisiyle yapıştırılır, aşınınca sökülüp yenilenir")
        recipes.append(Recipe(seg, None))
    if want("intake"):                                 # karın NACA boğazının PA-CF dudak çerçevesi (olduğu gibi, dolu)
        seg = SegSpec("intake", "NACA hava alığı dudak çerçevesi (PA-CF, karın)", "İtki", ["intake"], "intake", [],
                      shell=False, up_hint=np.array([1.0, 0, 0]),
                      note="gövde halkası 9'daki cebe yapıştırılır; kanal yangın perdesindeki açıklıktan kaportaya geçer")
        recipes.append(Recipe(seg, None))
    # kök kaportası (kano), ER-150 kabartması, dikey kök kaportası (açık kapak kabukları; P6: ayrı STL, AERO-20: tüy
    # kenar kırpılır)
    hosts_root = ("wing_center_clean", "fus_clean")
    if want("root_fairing") and ctx.has("root_fairing"):
        seg = SegSpec("root_fairing", "Kök kaportası — kano (PA-CF)", "Kaplamalar", ["root_fairing"],
                      "root_fairing", [], mirror=True, subtract=[(h, GLUE_GAP, 0.0) for h in hosts_root],
                      cover=True, up_hint=np.array([0, 0, -1.0]),
                      keep_fn=lambda: cover_keep_prisms(ctx, "root_fairing", hosts_root),
                      cavity_keep_fn=lambda: cover_keep_prisms(ctx, "root_fairing", hosts_root, COVER_HOLLOW_T,
                                                               erode=1),
                      note="1,6 mm kabuk, et < 3,6 mm olan kenar/dudaklar dolu; tüy kenar ≥ 0,8 mm'de kırpılır, kalan "
                           "kama epoksi + mikrobalon ile sıfırlanır")
        recipes.append(Recipe(seg, None))
    if want("gear_blister") and ctx.has("gear_blister"):
        hosts_bl = hosts_root + (("root_fairing",) if ctx.has("root_fairing") else ())
        seg = SegSpec("gear_blister", "ER-150 ünite kabartması (PA-CF)", "Kaplamalar", ["gear_blister"],
                      "root_fairing", [], mirror=True, subtract=[(h, GLUE_GAP, 0.0) for h in hosts_bl],
                      cover=True, up_hint=np.array([0, 0, -1.0]),
                      keep_fn=lambda: cover_keep_prisms(ctx, "gear_blister", hosts_bl),
                      cavity_keep_fn=lambda: cover_keep_prisms(ctx, "gear_blister", hosts_bl, COVER_HOLLOW_T),
                      note="kano kaportasına ve kanat altına yapıştırılır; ince kenarlar dolu, tüy kenar kırpılır")
        recipes.append(Recipe(seg, None))
    # Dikey kök "mermi" kaportası (U_Fairing_FinRoot) henüz basılmaz: R3 geometrisinde 84 cm³'ün 10,4 cm³'ü dikey +
    # stabilize dışında (s 2,14–2,41); dikeye ve stabilizeye teğet yaklaştığı için kaplama kırpması (cover_keep_prisms,
    # alttan Z ışını) uymaz — yandan kırpmalı ayrı tarif gerekir. Şimdilik epoksi + mikrobalon dolgu / ince levha (BOM).
    # Kanat üstü fileto basılmaz: kanat–gövde arasında sıfıra inen kama (%5 et ≈ 0,06 mm) → BOM: dolgu fileto.
    # Stabilize kök filetosu basılmaz: stabilize/koni arasında 0,1–3 mm kama (baskıya uygun değil) → BOM'da
    # epoksi + mikrobalon dolgu (ısı bölgesinde LW-PLA yok kuralı korunur).
    if want("turret_collar"):
        n_fac = int(P.SPEC["payload"]["turret"].get("collar_facets", 24))
        cw_ = _wall_of("turret_collar")
        seg = SegSpec("turret_collar", f"Taret yakası ({n_fac} faset + karın filetosu, PETG)", "Faydalı yük",
                      ["collar"], "turret_collar", [], subtract=[("fus_clean", GLUE_GAP, 0.0)], cover=True,
                      up_hint=np.array([0, 0, 1.0]),
                      keep_fn=lambda: cover_keep_prisms(ctx, "collar", ("fus_clean",)),
                      cavity_keep_fn=lambda: cover_keep_prisms(ctx, "collar", ("fus_clean",), 2 * cw_ + 0.0004),
                      note="gövde dış yüzünde kesilir (montaj denetimi: modül derisiyle 2,4 cm³ çakışıyordu): fileto "
                           "tabanı karın derisine bindirmeli yapışır; tüy kenar ≥ 0,8 mm'de kırpılır, kalan kama epoksi "
                           "+ mikrobalonla sıfırlanır (R04: sahnedeki gömülü üst kutu basılmaz)")
        recipes.append(Recipe(seg, None))
    for half, tag in ((0, "a"), (1, "b")):             # P10: ısıl biçimlendirme kalıbı + PETG çerçeve
        if want(f"hatch_buck_{tag}") or want("hatch"):
            seg = SegSpec(f"hatch_buck_{tag}", f"Aviyonik kapağı ısıl biçimlendirme kalıbı {tag.upper()} (PLA; alet)",
                          "Kalıp (alet)", [], "frames", [], material="PLA", shell=False, tooling=True,
                          up_hint=np.array([0, 0, 1.0]),
                          note="1,0 mm füme PETG levha bu kalıp üstünde biçimlendirilir; uçak kütlesine dahil değil",
                          builder=lambda c_, s_, o_, h=half: build_custom(c_, s_, o_, hatch_buck_parts(h)))
            recipes.append(Recipe(seg, None))
        if want(f"hatch_frame_{tag}") or want("hatch"):
            seg = SegSpec(f"hatch_frame_{tag}", f"Aviyonik kapağı çerçevesi {tag.upper()} (PETG; 3 mıknatıs cebi)",
                          "Kaplamalar", [], "frames", [], material="PETG", shell=False, up_hint=np.array([0, 0, 1.0]),
                          note="levha altına yapıştırılır; düz alt yüz gövde basamağına oturur",
                          builder=lambda c_, s_, o_, h=half: build_custom(c_, s_, o_, hatch_frame_parts(h)))
            recipes.append(Recipe(seg, None))
    if want("tolerance_coupon"):                      # P7: önce kupon basılır, delik payı doğrulanır
        seg = SegSpec("tolerance_coupon", "Tolerans kuponu (LW-PLA; ilk baskı — delik payı denemesi)", "Test / alet",
                      [], "wing_panel", [], material="LW-PLA", shell=False, tooling=True,
                      up_hint=np.array([0, 0, 1.0]),
                      note="Ø27/16/8/6 boru, Ø3 pim, Ø1,75 menteşe pimi delikleri (parçalardaki payla aynı)",
                      builder=lambda c_, s_, o_: build_custom(c_, s_, o_, tolerance_coupon_parts()))
        recipes.append(Recipe(seg, None))
    ctx.print_cuts = {
        "wing_y_m": sorted({round(float(v), 4) for v in list(ys) + [y_mid_root, y_join]}),
        "wing_tip_plane": {"point_b": [round(float(v), 4) for v in tip_p], "normal_b": [round(float(v), 4) for v in tip_n]},
        "strake_s_m": round(float(S_CUT_STRAKE), 4),
        "fuselage_s_m": [round(float(v), 4) for v in fcuts if 0.0 < v < s_fw + 0.01],
        "stab_y_m": round(float(cuts["stab_y"][1]), 4), "fin_h_m": round(float(cuts["fin_h"][1]), 4),
        "control_surface_y_m": {k: [round(float(v), 4) for v in surf_cut_y[k][1:-1]] for k in surf_cut_y},
        "cowl_cheeks": {sd: {"s_m": [round(float(v), 4) for v in cheeks[sd]["s"]],
                             "skin_normal_deg": cheeks[sd]["angles_deg"]} for sd in cheeks},
        "groove_mm": [float(v) for v in _PR.get("joint_groove_mm", [0.4, 0.3])],
    }
    for d, nm, mir in (("N_1", "Burun takımı kapağı (menteşe dili + horn dahil)", True),
                       ("N_3", "Burun takımı bacak tapa kapağı (bacağa bağlı)", False),
                       ("L_1", "Ana takım kuyu kapağı (menteşe dili + horn dahil)", True),
                       ("L_2", "Ana takım bacak kapağı", True)):
        key = f"door_{d}"
        if (want(key) or want("door")) and ctx.has(key):
            seg = SegSpec(key, f"{nm} (PETG 1,2 mm)", "Takım kapakları", [key], "gear_doors", [], shell=False,
                          mirror=mir, up_hint=np.array([0, 0, -1.0]), cutouts=door_notch_cuts(d))
            recipes.append(Recipe(seg, None))
    for r in recipes:                                   # spec print.zones[].parts: adıyla atanan filament (ısı bölgesi)
        z = part_zone(r.seg.key)
        if z is not None and r.seg.material is None:
            r.seg.material = str(_ZONES[z]["material"])
    return recipes, plans


def door_notch_cuts(d: str) -> list[Solid]:
    """R14: kapak plan çokgenindeki iç y kenarları (burun kapağı dikme çentiği). ``shapes._door_grid_panel`` her y
    sütununda tek s aralığı kullanır; çentiğin y = v kenarında komşu sütunlar (v ± 0,1 mm) çok farklı s'de başlar,
    aradaki şerit çentik boyunca eğik kalır ve dönüş dudağı satırları ≈ 7 µm kalınlığında, 2,5 mm yüksek bir kanatçık
    oluşturur (dilimleyici atar; ince örnek payı %0,9). Şerit (v ± 0,15 mm, çentiğin dış ucundan iç köşenin
    ``DOOR_GAP − 0,1 mm`` ötesine) parçadan çıkarılır: çentik kenarı 0,15 mm geri çekilir (aralık 0,15 mm büyür)."""
    from .. import shapes as S
    dd = next((x for x in P.gear_doors() if x.name == f"U_Door_{d}"), None)
    ob = bpy.data.objects.get(f"U_Door_{d}")
    if dd is None or ob is None:
        return []
    o = np.asarray(dd.outline, float)
    ylo, yhi = float(o[:, 1].min()), float(o[:, 1].max())
    zs = [(ob.matrix_world @ Vector(c)).z for c in ob.bound_box]
    z0, z1 = min(zs) - 0.005, max(zs) + 0.005
    out = []
    n = len(o)
    for k in range(n):
        a, b = o[k], o[(k + 1) % n]
        if abs(a[1] - b[1]) > 1e-9 or not (ylo + 1e-6 < a[1] < yhi - 1e-6) or abs(a[0] - b[0]) < 1e-4:
            continue
        v = float(a[1])
        sm = 0.5 * (a[0] + b[0])
        solid_hi = _point_in_poly((sm, v + 5e-4), o)            # kenar boyunca dolu taraf
        inner = None                                             # iç köşe: komşu kenar dolu tarafın tersine gider
        for end, nb in ((a, o[k - 1]), (b, o[(k + 2) % n])):
            if (nb[1] < v - 1e-9) == solid_hi:
                inner = end
        if inner is None:
            continue
        outer = b if inner is a else a
        dirn = math.copysign(1.0, inner[0] - outer[0])
        s_a, s_b = sorted((float(outer[0]) - dirn * 0.002, float(inner[0]) + dirn * (S.DOOR_GAP - 1e-4)))
        out.append(prim_box(((-s_a - s_b) / 2, v, (z0 + z1) / 2), np.eye(3),
                            ((s_b - s_a) / 2, 1.5e-4, (z1 - z0) / 2), name="notch_fin"))
    return out


# =====================================================================================================
# Kaplama tüy kenarı kırpma (AERO-20 / P13): kök kaportası ve takım kabartması ev sahibine (kanat/gövde) teğet
# yaklaştığı yerde sıfır kalınlığa iner; dilimleyici < 0,4 mm'yi atar → yırtık yapıştırma kenarı. Alttan bakışta
# her ızgara noktasında dış yüzden içe normal boyunca et ölçülür; et ≥ TRIM_T bölgesinin sınırı (yürüyen kareler)
# dik prizmaya çevrilir ve parça bununla kesişir → kenar ≥ 0,8 mm dik duvar; kalan kama montajda epoksi +
# mikrobalon dolgu ile sıfırlanır (stabilize kök filetosundaki yöntem).
# =====================================================================================================
TRIM_T = 0.0009                                     # kırpma kalınlığı (0,8 mm + 0,1 mm ızgara payı). varsayım
TRIM_STEP = 0.001                                   # kalınlık ızgarası (1 mm)
COVER_HOLLOW_T = 2 * 0.0012 + 0.0004                # kaplama yalnız et ≥ 2 × 1,2 + 0,4 mm olan yerde oyulur


def _march_loops(Fv: np.ndarray, xs: np.ndarray, ys: np.ndarray) -> list[np.ndarray]:
    """Yürüyen kareler: ``Fv[i, j]`` (x_i, y_j) ≥ 0 içeri; ızgara kenarı dışarıda (< 0) olmalı. Dönüş: kapalı
    döngüler (k, 2); içerisi solda → dış sınır saat yönü tersi, delik saat yönü. Eyer hücresi merkez ortalamasıyla."""
    nx, ny = Fv.shape
    ins = Fv >= 0.0

    def point(key):
        kind, i, j = key
        if kind == 0:
            a, b = Fv[i, j], Fv[i + 1, j]
            t = a / (a - b)
            return (xs[i] + t * (xs[i + 1] - xs[i]), ys[j])
        a, b = Fv[i, j], Fv[i, j + 1]
        t = a / (a - b)
        return (xs[i], ys[j] + t * (ys[j + 1] - ys[j]))

    nxt: dict = {}
    for i in range(nx - 1):
        for j in range(ny - 1):
            c = (ins[i, j], ins[i + 1, j], ins[i + 1, j + 1], ins[i, j + 1])
            if all(c) or not any(c):
                continue
            edges = (((0, i, j), c[0], c[1]), ((1, i + 1, j), c[1], c[2]), ((0, i, j + 1), c[2], c[3]),
                     ((1, i, j), c[3], c[0]))                       # hücre çevresi saat yönü tersi
            xk = [(k, bool(a and not b)) for k, a, b in edges if a != b]
            if len(xk) == 2:
                ex, en = (xk[0], xk[1]) if xk[0][1] else (xk[1], xk[0])
                nxt[ex[0]] = en[0]
            else:                                                   # eyer: 4 geçiş
                c_in = (Fv[i, j] + Fv[i + 1, j] + Fv[i + 1, j + 1] + Fv[i, j + 1]) >= 0.0
                for q, (k, is_exit) in enumerate(xk):
                    if is_exit:
                        nxt[k] = xk[(q + 1) % 4][0] if c_in else xk[(q - 1) % 4][0]
    loops, seen = [], set()
    for start in list(nxt):
        if start in seen:
            continue
        loop, k = [], start
        while k is not None and k not in seen:
            seen.add(k)
            loop.append(point(k))
            k = nxt.get(k)
        if len(loop) >= 3:
            loops.append(np.asarray(loop, float))
    return loops


def _rdp_closed(Pl: np.ndarray, eps: float) -> np.ndarray:
    """Kapalı çokgen Douglas–Peucker sadeleştirme (yinelemesiz)."""
    n = len(Pl)
    if n < 8:
        return Pl
    i1 = int(np.argmax(np.linalg.norm(Pl - Pl[0], axis=1)))
    keep = np.zeros(n + 1, bool)
    Q = np.vstack([Pl, Pl[:1]])
    keep[[0, i1, n]] = True
    stack = [(0, i1), (i1, n)]
    while stack:
        a, b = stack.pop()
        if b - a < 2:
            continue
        seg = Q[b] - Q[a]
        L = float(np.hypot(seg[0], seg[1]))
        W = Q[a + 1:b] - Q[a]
        d = np.abs(seg[0] * W[:, 1] - seg[1] * W[:, 0]) / L if L > 1e-12 else np.hypot(W[:, 0], W[:, 1])
        k = int(np.argmax(d))
        if d[k] > eps:
            m = a + 1 + k
            keep[m] = True
            stack += [(a, m), (m, b)]
    return Q[:n][keep[:n]]


def _poly_area2(Pl: np.ndarray) -> float:
    return 0.5 * float(np.sum(Pl[:, 0] * np.roll(Pl[:, 1], -1) - np.roll(Pl[:, 0], -1) * Pl[:, 1]))


def _point_in_poly(p, Pl: np.ndarray) -> bool:
    x, y = p
    inside = False
    for k in range(len(Pl)):
        (x1, y1), (x2, y2) = Pl[k - 1], Pl[k]
        if (y1 > y) != (y2 > y) and x < x1 + (y - y1) * (x2 - x1) / (y2 - y1):
            inside = not inside
    return inside


def cover_keep_prisms(ctx: "Ctx", src: str, hosts: Sequence[str], t_min: float = TRIM_T,
                      step: float = TRIM_STEP, gap: float = GLUE_GAP, erode: int = 0) -> list[Solid]:
    """Alt yüz kaplaması (``src``, ev sahipleri ``hosts`` altında): et ≥ ``t_min`` bölgesinin dik (Z) prizmaları.

    Izgara noktası durumu: kaplama dış yüzü ev sahibi içinde → tut; çevrili boşluk (kuyu açıklığı) → tut; dış
    boşluk → at; aksi hâlde et = min(ev sahibine uzaklık − yapıştırma boşluğu, kaplamadan çıkış) içe normal boyunca.
    """
    cache = ctx.__dict__.setdefault("_keep_cache", {})
    ck = (src, tuple(hosts), round(t_min, 6), round(step, 6), round(gap, 6), erode)
    if ck in cache:
        return cache[ck]
    Ss = ctx.solid(src)
    bf = BVHTree.FromPolygons(Ss.V.tolist(), [list(f) for f in Ss.F], epsilon=0.0)
    hb = []
    for h in hosts:
        if ctx.has(h):
            Sh = ctx.solid(h)
            hb.append(BVHTree.FromPolygons(Sh.V.tolist(), [list(f) for f in Sh.F], epsilon=0.0))
    lo, hi = Ss.V.min(0), Ss.V.max(0)
    xs = np.arange(lo[0] - 3 * step, hi[0] + 3 * step, step)
    ys = np.arange(lo[1] - 3 * step, hi[1] + 3 * step, step)
    nx, ny = len(xs), len(ys)
    state = np.zeros((nx, ny), np.int8)                 # 0 boşluk, 1 ev sahibi içi, 2 ölçüldü
    T = np.zeros((nx, ny))
    up = Vector((0.0, 0.0, 1.0))
    z0 = float(lo[2]) - 0.01
    for i in range(nx):
        for j in range(ny):
            h = bf.ray_cast(Vector((float(xs[i]), float(ys[j]), z0)), up, 1.0)
            if h[0] is None:
                continue
            p = h[0]
            n = h[1] if h[1].z < 0 else -h[1]
            inside = False
            for b in hb:
                q = b.find_nearest(p)
                if q[0] is not None and (p - q[0]).dot(q[1]) < 0:
                    inside = True
                    break
            if inside:
                state[i, j] = 1
                continue
            o = p - n * 2e-6
            t = 0.05
            for b in hb:
                g = b.ray_cast(o, -n, 0.05)
                if g[0] is not None:
                    t = min(t, g[3] - gap)
            g = bf.ray_cast(o, -n, 0.05)
            if g[0] is not None:
                t = min(t, g[3])
            state[i, j] = 2
            T[i, j] = t
    ext = np.zeros((nx, ny), bool)                      # kenardan erişilen boşluk (dış)
    stack = [(i, j) for i in range(nx) for j in (0, ny - 1)] + [(i, j) for i in (0, nx - 1) for j in range(ny)]
    while stack:
        i, j = stack.pop()
        if i < 0 or j < 0 or i >= nx or j >= ny or ext[i, j] or state[i, j] != 0:
            continue
        ext[i, j] = True
        stack += [(i + 1, j), (i - 1, j), (i, j + 1), (i, j - 1)]
    Fv = np.where(state == 2, T - t_min, np.where(ext, -t_min, t_min))
    Fv[0, :] = Fv[-1, :] = -t_min
    Fv[:, 0] = Fv[:, -1] = -t_min
    for _ in range(erode):                              # R14: boşluk sınırı ince bölgeden ``erode`` hücre uzak (dik
        G = Fv.copy()                                   # basamakta — kuyu dudağı — içe kaydırma ters dönüp µm levha
        G[1:, :] = np.minimum(G[1:, :], Fv[:-1, :])     # bırakıyordu)
        G[:-1, :] = np.minimum(G[:-1, :], Fv[1:, :])
        G[:, 1:] = np.minimum(G[:, 1:], Fv[:, :-1])
        G[:, :-1] = np.minimum(G[:, :-1], Fv[:, 1:])
        Fv = G
    # R14: ölçülmüş kalın (≥ t_min) hücresi olmayan "ev sahibi içi / çevrili boşluk" adaları tutulmaz. Aksi hâlde ada
    # ile kalın bölge arasındaki ince kuşak bir delik döngüsü olur ve kırpılmaz (kök kaportası kuyruğunda kanat alt
    # yüzü boyunca 0,1–0,3 mm'lik kama kalıyordu).
    lab, n_lab = _label4(Fv >= 0)
    has_thick = np.bincount(lab[(state == 2) & (T >= t_min)], minlength=n_lab + 1) > 0
    has_thick[0] = True
    n_isl = int((~has_thick).sum())
    Fv[~has_thick[lab]] = -t_min
    loops = [_rdp_closed(L, 0.12 * step) for L in _march_loops(Fv, xs, ys)]
    outers = [L for L in loops if _poly_area2(L) > 1e-4]          # > 1 cm², saat yönü tersi
    outers.sort(key=_poly_area2, reverse=True)
    top = [L for k, L in enumerate(outers) if not any(_point_in_poly(L[0], M) for M in outers[:k])]
    # R14: kalın bölgenin içindeki ince (< t_min) kuşak/adalar (saat yönü döngüler, > 20 mm²) da kırpılır — dikey delik
    # prizması parçadan çıkarılır (build_segment: ad "trim_hole" → DIFFERENCE).
    holes = [L[::-1] for L in loops if _poly_area2(L) < -2e-5 and any(_point_in_poly(L[0], M) for M in top)
             and not any(_point_in_poly(M[0], L) for M in outers if not any(M is X for X in top))]
    z_lo, z_hi = float(lo[2]) - 0.05, float(hi[2]) + 0.05
    prisms = [prim_prism(L, np.zeros(3), np.array([1.0, 0, 0]), np.array([0, 1.0, 0]), np.array([0, 0, 1.0]),
                         z_lo, z_hi, name="keep") for L in top]
    hole_prisms = [prim_prism(L, np.zeros(3), np.array([1.0, 0, 0]), np.array([0, 1.0, 0]), np.array([0, 0, 1.0]),
                              z_lo - 0.01, z_hi + 0.01, name="trim_hole") for L in holes]
    if not prisms:
        cache[ck] = []
        return []
    V, Fl, off = [], [], 0                              # ayrık prizmalar tek katı (kesişim işleneni tek nesne olmalı)
    for pz in prisms:
        V.append(pz.V)
        Fl += [tuple(int(x) + off for x in f) for f in pz.F]
        off += len(pz.V)
    thin = int(((state == 2) & (T < t_min)).sum())
    ctx.log(f"{src}: tüy kenar kırpma — {thin} ızgara noktası < {t_min * 1000:.1f} mm, {len(top)} döngü, "
            f"{len(holes)} iç ince bölge, {n_isl} gömülü ada atıldı")
    cache[ck] = [Solid(np.vstack(V), Fl, "keep")] + (
        [merge_solids(hole_prisms, "trim_hole")] if hole_prisms else [])
    return cache[ck]


def _label4(mask: np.ndarray) -> tuple[np.ndarray, int]:
    """4-komşulu bağlı bileşen etiketleri (0 = maske dışı; scipy yok)."""
    lab = np.zeros(mask.shape, np.int32)
    nx, ny = mask.shape
    n = 0
    for i0, j0 in zip(*np.nonzero(mask)):
        if lab[i0, j0]:
            continue
        n += 1
        lab[i0, j0] = n
        stack = [(int(i0), int(j0))]
        while stack:
            i, j = stack.pop()
            for a, b in ((i + 1, j), (i - 1, j), (i, j + 1), (i, j - 1)):
                if 0 <= a < nx and 0 <= b < ny and mask[a, b] and not lab[a, b]:
                    lab[a, b] = n
                    stack.append((a, b))
    return lab, n


COUPON_T = 0.020                                    # kupon kalınlığı (delik boyu)
COUPON_WALL = 0.005


def coupon_holes() -> list[tuple[str, float, float]]:
    """(etiket, nominal Ø m, basılan delik Ø m) — parçalardaki kural (``hole_d``, ``PIN_HOLE_D``, ``HINGE_PIN_HOLE_D``)."""
    return [("panel borusu Ø27", 0.027, hole_d(0.027, "tube", "LW-PLA")),
            ("uç borusu Ø16", 0.016, hole_d(0.016, "tube", "LW-PLA")),
            ("arka kiriş / longeron Ø8", 0.008, hole_d(0.008, "tube", "LW-PLA")),
            ("açı pimi Ø6", 0.006, hole_d(0.006, "tube", "LW-PLA")),
            ("hizalama pimi Ø3", PIN_D, PIN_HOLE_D),
            ("menteşe pimi Ø1,75", HINGE_PIN_D, HINGE_PIN_HOLE_D)]


def tolerance_coupon_parts() -> dict:
    """``tolerance_coupon.stl`` (P7): 20 mm LW-PLA blok, delikler dik (segmentlerdeki gibi Z ekseninde), parçalarla
    aynı çevrel çokgen + malzeme payıyla. Köşe pahı = Ø27 tarafı."""
    H = coupon_holes()
    W = max(d for _, _, d in H) + 2 * COUPON_WALL
    x, xs = COUPON_WALL, []
    for _, _, d in H:
        xs.append(x + 0.5 * d)
        x += d + COUPON_WALL
    L = x
    o = np.array([3.0, 0.0, 0.0])                       # uçaktan uzakta (dünya)
    base = prim_box(o + np.array([0.5 * L, 0.0, 0.5 * COUPON_T]), np.eye(3), (0.5 * L, 0.5 * W, 0.5 * COUPON_T),
                    name="coupon")
    holes = [prim_hole(o + np.array([xc, 0.0, -0.002]), o + np.array([xc, 0.0, COUPON_T + 0.002]), d, name="ch")
             for xc, (_, _, d) in zip(xs, H)]
    c = 0.004                                           # yön işareti: Ø27 ucunda 4 mm pah
    holes.append(prim_box(o + np.array([0.0, 0.5 * W, 0.5 * COUPON_T]),
                          np.array([[math.sqrt(0.5), -math.sqrt(0.5), 0], [math.sqrt(0.5), math.sqrt(0.5), 0],
                                    [0, 0, 1.0]]).T, (c, c, COUPON_T), name="mark"))
    return {"base": base, "adds": [], "holes": holes}


def intake_duct_cut() -> Solid:
    """Karın NACA alığı (``shapes.intake_cutter``) kanal cebinin arkasından yangın perdesi düzlemine açılan hava
    geçişi: halka 9'un arka ucu delinir, G10 perdede aynı açıklık (BOM notu) → hava kaporta içine, silindire."""
    I = P.intake_spec()
    s_t = float(I["throat_s"])
    s_fw = float(P.SPEC["fuselage"]["modules"]["firewall_s_m"])
    w = 0.5 * float(I["mouth_w_m"]) - 0.004
    z0, z1 = (float(v) for v in I["duct_z"])
    lo, hi = Bp(s_fw + 0.006, -w, z0 + 0.0015), Bp(s_t + 0.006, w, z1 - 0.0015)
    lo2, hi2 = np.minimum(lo, hi), np.maximum(lo, hi)
    return prim_box(0.5 * (lo2 + hi2), np.eye(3), 0.5 * (hi2 - lo2), name="intake_duct")


def _hatch_opening(inset: float = 0.006) -> Solid:
    """Aviyonik kapağı altı erişim açıklığı (kapak çevresinde ``inset`` oturma kenarı kalır)."""
    poly = np.asarray(P.hatch_outline(), float)           # (s, y)
    c = poly.mean(0)
    out = []
    n = len(poly)
    for i in range(n):                                     # içe kaydırma (dışbükey çokgen; açıortay)
        a, b, cc = poly[i - 1], poly[i], poly[(i + 1) % n]
        e1, e2 = _unit(b - a), _unit(cc - b)
        n1, n2 = np.array([-e1[1], e1[0]]), np.array([-e2[1], e2[0]])
        if (c - b) @ n1 < 0:
            n1, n2 = -n1, -n2
        bis = _unit(n1 + n2)
        out.append(b + bis * inset / max(0.3, float(bis @ n1)))
    out = np.asarray(out)
    V2 = np.column_stack([-out[:, 0], out[:, 1]])         # Blender XY
    area = 0.5 * np.sum(V2[:, 0] * np.roll(V2[:, 1], -1) - np.roll(V2[:, 0], -1) * V2[:, 1])
    if area < 0:
        V2 = V2[::-1]
    return prim_prism(V2, np.zeros(3), np.array([1.0, 0, 0]), np.array([0, 1.0, 0]), np.array([0, 0, 1.0]),
                      0.0, 0.26, name="hatch_open")


def _section_ring_offset(s: float, inset: float) -> np.ndarray:
    """Temiz gövde kesiti (y, z), içe ``inset`` kaydırılmış (dışbükey kesit; köşe normalleri)."""
    yz = np.asarray(P.fuselage_outline(s, 48), float)
    c = yz.mean(0)
    n = len(yz)
    out = []
    for i in range(n):
        t = _unit(yz[(i + 1) % n] - yz[i - 1])
        nr = np.array([t[1], -t[0]])
        if (c - yz[i]) @ nr < 0:
            nr = -nr
        out.append(yz[i] + nr * inset)
    return np.asarray(out)


def _ring_points(s: float, inset: float, angles_deg: Sequence[float], avoid: Sequence = (),
                 clear: float = 0.010) -> list:
    """Kesit halkasında, merkezden verilen açılara en yakın ve engellerden ``clear`` uzak noktalar (y, z)."""
    ring = _section_ring_offset(s, inset)
    c = np.array([0.0, float(np.mean(ring[:, 1]))])
    ang = np.degrees(np.arctan2(ring[:, 1] - c[1], ring[:, 0] - c[0])) % 360
    out = []
    for a0 in angles_deg:
        order = np.argsort(np.abs(((ang - a0 + 180) % 360) - 180))
        for i in order:
            q = ring[i]
            if all(np.hypot(*(q - np.asarray(o))) >= clear for o in list(avoid) + out):
                out.append(q)
                break
    return out


def _longeron_at(kind: str, side: str, s: float) -> np.ndarray:
    pts = P.longeron_path(kind, side, 300)
    return Bp(float(s), float(np.interp(s, pts[:, 0], pts[:, 1])), float(np.interp(s, pts[:, 0], pts[:, 2])))


def nose_flange_bolts() -> tuple[list, list]:
    """Burun modülü flanşı (s = 0,40): 4×M4 cıvata ve 2×Ø4 pim konumları (y, z)."""
    s = float(P.SPEC["fuselage"]["modules"]["nose_module_s_m"][1])
    avoid = [(_longeron_at(k, sd, s)[1], _longeron_at(k, sd, s)[2]) for k in ("chine", "shoulder") for sd in ("L", "R")]
    avoid += [(0.0, -0.09), (0.02, -0.09), (-0.02, -0.09)]                # burun takımı kuyusu ağzı
    inset = _wall_of("fuselage_mid") + 0.005
    bolts = _ring_points(s, inset, [30, 150, 215, 325], avoid, 0.012)
    pins = _ring_points(s, inset, [75, 105], avoid + bolts, 0.012)
    return bolts, pins


STRINGER = {"t": 0.0008, "h": 0.005, "step": 0.020}   # iç stringer (R04): 0,8 × 5 mm (deri iç yüzünden), 20 mm istasyon


def _fus_skin_ray(s: float, phi: float) -> tuple[np.ndarray, np.ndarray] | None:
    """Gövde kesitinde (temiz dış yüz) kesit merkezinden ``phi`` açısındaki deri noktası (y, z) ve içe birim normal."""
    yz = np.asarray(P.fuselage_outline(s, 48), float)
    c = np.array([0.0, float(P.fuselage_section(s).z_center)])
    d = np.array([math.cos(phi), math.sin(phi)])
    best = None
    for i in range(len(yz)):
        a, b = yz[i], yz[(i + 1) % len(yz)]
        e = b - a
        M = np.array([[d[0], -e[0]], [d[1], -e[1]]])
        if abs(np.linalg.det(M)) < 1e-14:
            continue
        t, u = np.linalg.solve(M, a - c)
        if t > 0 and -1e-9 <= u <= 1 + 1e-9 and (best is None or t < best[0]):
            nr = _unit(np.array([-e[1], e[0]]))
            if float(nr @ (c - (a + u * e))) < 0:
                nr = -nr
            best = (t, a + u * e, nr)
    return None if best is None else (best[1], best[2])


def fus_stringer_angles(s: float) -> list[float]:
    """4 iç stringer açısı (rad, kesit merkezinden): sırt ortası, karın ortası ve her yanda omuz ile chine
    longeronlarının açı ortası (deri panellerinin en geniş, desteksiz yerleri)."""
    zc = float(P.fuselage_section(s).z_center)
    out = [0.5 * math.pi, -0.5 * math.pi]
    for sd in ("L", "R"):
        angs = []
        for kind in ("chine", "shoulder"):
            q = _longeron_at(kind, sd, s)
            angs.append(math.atan2(q[2] - zc, q[1]))
        a0, a1 = angs
        if sd == "R":                                   # sağda açılar ±π çevresinde: sürekli ortalama
            a0, a1 = (a0 % (2 * math.pi)), (a1 % (2 * math.pi))
        out.append(0.5 * (a0 + a1))
    return out


def fus_stringer_prims(s0: float, s1: float, wall: float) -> list[Solid]:
    """Halka içi boyuna stringerlar (R04: 0,7 mm deriyi burkulmaya karşı destekler): her açıda deri dış yüzünün
    1 mm dışından deri iç yüzünün ``h`` içine dek ``t`` kalın şerit, ``step`` aralıklı istasyonlarla loft (iç yapı:
    boşluktan çıkarılır → yalnız boşlukta malzeme olur; açıklıklar/delikler onu da keser)."""
    T, H = STRINGER["t"], STRINGER["h"]
    ss = np.linspace(s0, s1, max(2, int(math.ceil((s1 - s0) / STRINGER["step"])) + 1))
    out = []
    for k, phi in enumerate(fus_stringer_angles(0.5 * (s0 + s1))):
        rings = []
        for s in ss:
            hit = _fus_skin_ray(float(s), phi)
            if hit is None:
                break
            q, nr = hit
            tg = np.array([-nr[1], nr[0]])
            o, i_ = q - nr * 0.001, q + nr * (wall + H)
            pts2 = [o - tg * 0.5 * T, o + tg * 0.5 * T, i_ + tg * 0.5 * T, i_ - tg * 0.5 * T]
            rings.append([Bp(float(s), float(p[0]), float(p[1])) for p in pts2])
        if len(rings) < 2:
            continue
        V = np.asarray([v for r in rings for v in r], float)
        m = len(rings)
        F = [(0, 1, 2, 3), tuple(4 * (m - 1) + j for j in (3, 2, 1, 0))]
        for r in range(m - 1):
            for j in range(4):
                j2 = (j + 1) % 4
                F.append((4 * r + j, 4 * (r + 1) + j, 4 * (r + 1) + j2, 4 * r + j2))
        sol = Solid(V, F, f"stringer_{k}")
        if _signed_volume(sol.V, sol.F) < 0:
            sol = Solid(V, [tuple(reversed(f)) for f in F], f"stringer_{k}")
        out.append(sol)
    return out


def fuselage_features(ctx: Ctx, seg: SegSpec, s0: float, s1: float) -> Feature:
    """Gövde halkası: longeron kanalları (+ kovan, deriye ağ), kanat kutusu açıklığı, aviyonik kapak açıklığı,
    burun flanşı cıvata/pim delikleri, G10 flanş cıvataları, stabilize boru geçişleri, hava alığı geçişi."""
    f = Feature()
    L = P.SPEC["fuselage"]["longerons"]
    la, lb = max(s0, float(L["s_from_m"])), min(s1, float(L["s_to_m"]))
    host = ctx.solid("fus")
    if lb - la > 0.01:
        bvh = BVHTree.FromPolygons(host.V.tolist(), [list(f_) for f_ in host.F], epsilon=0.0)
        for kind in ("chine", "shoulder"):
            for sd in ("L", "R"):
                p0, p1 = _longeron_at(kind, sd, la - 0.004), _longeron_at(kind, sd, lb + 0.004)
                f.extend(tube_prims(p0, p1, float(L["od_m"]), name=f"longeron_{kind}_{sd}", collars=True))
                pm = 0.5 * (p0 + p1)
                near = bvh.find_nearest(Vector(pm))
                if near[0] is not None:
                    q = np.array(near[0])
                    dvec = q - pm
                    dist = float(np.linalg.norm(dvec))
                    if dist > 0.5 * float(L["od_m"]) + SLEEVE_T:
                        up = _unit(dvec)
                        f.structure.append(prim_box(pm + up * 0.5 * (dist + 0.002), np.column_stack(
                            [_unit(p1 - p0), up, np.cross(_unit(p1 - p0), up)]),
                            (0.5 * float(np.linalg.norm(p1 - p0)), 0.5 * (dist + 0.002), 0.5 * SLEEVE_T), name="lweb"))
    s_mod = float(P.SPEC["fuselage"]["modules"]["nose_module_s_m"][1])
    s_flg = float(P.SPEC["fuselage"]["modules"]["flange_s_m"])
    s_cone = float(P.TURRET.s - 0.5 * P.TURRET.collar_d) - 0.03                  # burun konisi ucu stringersiz
    if s0 >= min(s_mod, s_cone) - 1e-6 and s1 <= s_flg + 1e-6:  # R04: 0,7 mm derili halka/modülde 4 iç stringer
        f.structure += fus_stringer_prims(s0 - 0.002, s1 + 0.002, _wall_of("fuselage_mid"))
        f.notes.append(f"deri {_fmt_g(_wall_of('fuselage_mid') * 1000)} mm + 4 iç stringer "
                       f"{_fmt_g(STRINGER['t'] * 1000)} × {_fmt_g(STRINGER['h'] * 1000)} mm")
    # kanat kutusu (G10 köprü + Ø30 soket) geçişi
    if s0 < P.WING_SPAR_S < s1:
        T = {t.name: t for t in P.spar_tubes(SIDE)}
        zc = Bp(*T["centre_socket"].p0)[2]
        g10 = P.SPEC["print"]["g10_bridge"]
        hw = 0.5 * (0.030 + 2 * G10_T) + 0.0003                      # köprü zarfı + 0,3 mm (çerçeveler deriye yaslanır)
        hh = 0.5 * G10_H + 0.003
        f.holes.append(prim_box(Bp(P.WING_SPAR_S, 0, zc + 0.003), np.eye(3), (hw, 0.25, hh + 0.003), name="wing_box"))
        f.notes.append("kanat kutusu geçişi (G10 köprü + Ø30 soket)")
    for which in (0, 1):                                             # P4: kanat kutusu çerçeveleri (iç yapı girmez)
        fs0, fs1 = wing_frame_parts(which)["s"]
        if s0 - 0.01 < fs0 and fs1 < s1 + 0.01:
            f.keepout.append(wing_frame_keepout(which))
    if s0 < NOSE_MOUNT_S[1] and s1 > NOSE_MOUNT_S[0]:              # P4: burun takım yatağı
        f.keepout.append(gear_mount_nose_keepout())
    if s0 < ENGINE_RING_S[1] and s1 > ENGINE_RING_S[0]:            # P4: motor halkası
        f.keepout.append(engine_ring_keepout())
        f.holes.append(engine_ring_band_clear())                     # R14: arka flanş ↔ bilezik çakışması/kıymık
    tp = tail_conduit_polyline()                                      # P9: kuyruk kablo kanalı koni duvarından
    if s0 < -float(tp[-1][0]) + 0.02 and s1 > -float(tp[-1][0]) - 0.04:
        f.holes += conduit_holes(tp[1:], CONDUIT_D["tail"])
    rp = root_conduit_polyline()                                      # P9: kök kanalı gövde yan duvarından
    if s0 < 1.268 < s1:
        f.holes += conduit_holes(rp[-3:], CONDUIT_D["root"])
        f.holes += conduit_holes([p * np.array([1.0, -1.0, 1.0]) for p in rp[-3:]], CONDUIT_D["root"])
    sbk = longeron_break_s()                                         # P4: longeron kırık soketi cepleri
    if s0 - 0.025 < sbk < s1 + 0.025:
        for kind in ("chine", "shoulder"):
            for sd in ("L", "R"):
                pk = ctx.custom_solid(f"ljp_{kind}_{sd}", lambda kind=kind, sd=sd: longeron_joint_pocket_parts(kind, sd))
                if pk is not None:
                    f.holes.append(pk)
        f.notes.append("longeron kırık soketi cepleri (PETG blok)")
    # aviyonik kapağı oturma basamağı + erişim açıklığı + mıknatıs cepleri (P10)
    hs = P.SPEC["details"]["hatch"]
    if s0 < float(hs["s_to_m"]) + 0.01 and s1 > float(hs["s_from_m"]) - 0.01:
        f.extend(hatch_seat_features(ctx, s0, s1))
    # burun modülü bağlantısı (ring 1 ön çerçevesi)
    s_nm = float(P.SPEC["fuselage"]["modules"]["nose_module_s_m"][1])
    if abs(s0 - s_nm) < 1e-6 or abs(s1 - s_nm) < 1e-6:
        bolts, pins = nose_flange_bolts()
        for (y, z), d in [(b, M4_HOLE_D) for b in bolts] + [(p, hole_d(NOSE_PIN_D, "pin")) for p in pins]:
            f.holes.append(prim_hole(Bp(s_nm - 0.01, y, z), Bp(s_nm + 0.012, y, z), d, name="nose_bolt"))
        f.notes.append("burun modülü: 4×M4 kelebek + 2×Ø4 pim")
        if abs(s1 - s_nm) < 1e-6:                                    # modül arka çerçevesinde PETG flanş cebi
            f.holes.append(nose_flange_pocket())
            f.notes.append("arka çerçevede PETG flanş cebi (flanş yapıştırılır, halka 1'e cıvatalanır)")
    # G10 flanş (s = 1,55): kuyruk konisi ön çerçevesinde 4×M4
    s_fl = float(P.SPEC["fuselage"]["modules"]["flange_s_m"])
    if abs(s0 - s_fl) < 1e-6:
        avoid = [(_longeron_at(k, sd, s_fl)[1], _longeron_at(k, sd, s_fl)[2]) for k in ("chine", "shoulder")
                 for sd in ("L", "R")]
        for y, z in _ring_points(s_fl, _wall_of("tail_cone") + 0.005, [40, 140, 220, 320], avoid, 0.012):
            f.holes.append(prim_hole(Bp(s_fl - 0.01, y, z), Bp(s_fl + 0.012, y, z), M4_HOLE_D, name="g10_bolt"))
        f.notes.append("G10 flanş: 4×M4")
    # stabilize boruları (koni yan duvarından geçiş + takviye göbeği)
    T = {t.name: t for t in P.spar_tubes(SIDE)}
    for nm, od in (("stab_spar", 0.012), ("stab_rear_rod", 0.006)):
        t = T[nm]
        for sg in (1.0, -1.0):
            a = Bp(t.p0[0], 0.0, t.p0[2])
            b = Bp(t.p1[0], sg * t.p1[1], t.p1[2])
            d = _unit(b - a)
            ss = -a[0]
            if not (s0 - 0.01 < ss + 0.10 * abs(d[0]) / max(abs(d[1]), 1e-6) and ss < s1):
                continue
            f.holes.append(prim_hole(a - d * 0.01, a + d * 0.12, hole_d(od, "tube"), name="stab_tube"))
            hit = None
            bvh2 = BVHTree.FromPolygons(host.V.tolist(), [list(f_) for f_ in host.F], epsilon=0.0)
            r = bvh2.ray_cast(Vector(a + d * 0.001), Vector(d), 0.2)
            if r[0] is not None:
                hit = np.array(r[0])
                f.structure.append(prim_frustum(hit - d * 0.008, hit + d * 0.002, 0.5 * hole_d(od, "tube") + 0.003,
                                                n=32, name="stab_boss"))
    # hava alığı geçişi
    I = P.intake_spec()
    if s0 < float(I["s_from_m"]) + 0.03 and s1 > float(I["s_from_m"]):
        f.holes.append(intake_duct_cut())
    return f


COWL_CUT_CLEAR = 0.002                              # yanak kesimi çene kapanışının bu kadar önünde biter (R14)


def cowl_s_range() -> tuple[float, float, float]:
    """Kaporta (s_yangın perdesi, s_arka yüz = lüle halkası önü, s_çene kapanışı başı). Çene kapanışı: arka yüzden
    önceki son gövde istasyonu (dik kapanış s 2,178 → 2,19)."""
    s0 = float(P.SPEC["propulsion"]["cowl"]["s_from_m"])
    s_end = float(P.SPEC["propulsion"]["exhaust_ring"]["s_from_m"])
    st = sorted(float(r[0]) for r in P.SPEC["fuselage"]["stations"])
    s_close = max(s for s in st if s < s_end - 1e-6)
    return s0, s_end, s_close


COWL_CHEEK_FRAME_W = 0.006                          # yanak/kaporta eklerindeki iç flanş eni
COWL_CUT_BAND = 0.0024                              # eğik ek flanşının düzleme dik kalınlığı (1,6 mm + normal sapma payı)


def _cowl_skin_cross(s: float, z: float, side: float) -> tuple[np.ndarray, np.ndarray] | None:
    """Kaporta dış yüzünün ``s`` istasyonunda ``z`` yüksekliğini ``side`` tarafında kestiği nokta (y, z) ve oradaki dışa
    birim normal (y, z) — spec takımı; kesmiyorsa None."""
    from .. import shapes as S
    ss, O = S._cowl_grid()
    ring = O[int(np.argmin(np.abs(ss - s)))]
    c = ring[:, 1:].mean(0)
    n = len(ring)
    for i in range(n):
        a, b = ring[i], ring[(i + 1) % n]
        if (a[2] - z) * (b[2] - z) < 0:
            t = (z - a[2]) / (b[2] - a[2])
            q = (a + t * (b - a))[1:]
            if q[0] * side <= 0:
                continue
            tg = (b - a)[1:]
            nr = _unit(np.array([tg[1], -tg[0]]))
            if float(nr @ (q - c)) < 0:
                nr = -nr
            return q, nr
    return None


def cowl_cheek_cuts() -> dict:
    """Yanak baskı bölgeleri ``{"L"/"R": {"planes": [(p, n, tür)], "s": (s_a, s_b), "center", "angles_deg"}}``
    (Blender; n bölgenin DIŞINA bakar; tür "frame" = iki yanda 6 mm iç flanş, "open" = düz ek). Spec yanak
    pencerelerinden türetilir, iki düzeltmeyle (R14, kıymık):

    * Yatay z = sabit kesimler yüzeye 30–60° yatık yerlerde (yanağın üst rampası, alt köşe) deriyi verev keser, kama
      kıymığı (< 0,4 mm) ve sıfır kalınlıklı flanş bırakıyordu. Üst/alt ekler artık DERİYE DİK eğik düzlemlerdir:
      düzlem, pencere sınırının deriyi kestiği noktaların ortalamasından geçer ve oradaki ortalama deri normalini içerir
      (istasyonlar boyunca sapma ≤ 8°; flanş bandı bu sapma için 2,4 mm).
    * Kesim düzlemi dik çene kapanışına / arka flanşa düşerse (s_b > s_kapanış − 2 mm) yüzey s düzlemine neredeyse
      paraleldir: sol yanak arka yüzün ÖTESİNE uzatılır (sol alt-arka bölge, flanş yarısı dahil, tek parça; ekler
      y = 0 ve eğik düzlemde, ikisi de flanşa dik), sağ yanak kapanıştan 2 mm önce biter."""
    cw = P.SPEC["propulsion"]["cowl"]
    _, s_end, s_close = cowl_s_range()
    out = {}
    for sd, sg in (("L", 1.0), ("R", -1.0)):
        c = cw["cheek_left" if sd == "L" else "cheek_right"]
        s_a, s_b, z_a, z_b = float(c["s_from_m"]), float(c["s_to_m"]), float(c["z_from_m"]), float(c["z_to_m"])
        if s_b > s_close - COWL_CUT_CLEAR:
            s_b = s_end + 0.004 if sd == "L" else s_close - COWL_CUT_CLEAR
        st = np.arange(s_a + 0.004, min(s_b, s_close - COWL_CUT_CLEAR) - 0.002, 0.006)
        planes = [(Bp(s_a, 0, 0), np.array([1.0, 0, 0]), "frame")]
        if s_b < s_end:
            planes.append((Bp(s_b, 0, 0), np.array([-1.0, 0, 0]), "frame"))
        angs, mids = [], []
        for z_cut in (z_b, z_a):
            hits = [h for h in (_cowl_skin_cross(float(x), z_cut, sg) for x in st) if h is not None]
            if len(hits) < max(2, len(st) // 2):          # pencere kaporta dışında (sol yanak alt kenarı) → y = 0 eki
                continue
            q = np.mean([h[0] for h in hits], axis=0)
            nr = _unit(np.mean([h[1] for h in hits], axis=0))
            m = np.array([-nr[1], nr[0]])               # düzlem normali (y, z): deri normaline dik
            inside = np.array([sg * 0.06, 0.5 * (z_a + z_b)])
            if float(m @ (inside - q)) > 0:
                m = -m
            angs.append(math.degrees(math.atan2(nr[1], nr[0] * sg)))
            mids.append(q)
            planes.append((Bp(0.0, float(q[0]), float(q[1])), np.array([0.0, m[0], m[1]]), "frame"))
        planes.append((Bp(0.0, 0.0, 0.0), np.array([0.0, -sg, 0.0]), "open"))
        zc = float(np.mean([q[1] for q in mids])) if len(mids) > 1 else 0.5 * (z_b + 0.04)
        center = Bp(0.5 * (s_a + min(s_b, s_end)), sg * 0.05, zc)
        out[sd] = {"planes": planes, "s": (s_a, s_b), "center": center, "angles_deg": [round(a, 1) for a in angs]}
    return out


COWL_FLOW_CLEAR_M = 0.026                           # arka açıklık kesicisinin flanş önüne uzaması (akış silindiri)


def cowl_cutters(part: str) -> list[Solid]:
    """Kaporta parçalarının R01 delikleri ve kırpmaları: arka Ø92 açıklık (``shapes.cowl_aft_opening`` — lüle halkası
    iç çapı; soğutma havası + rulman burnu), çene soğutma çıkış yarığı (``shapes.cooling_exit_cutter``, kabuğu
    boydan boya), sol yanakta susturucu borusu deliği (``shapes.muffler_pipe_hole``) ve motor yasak bölgeleri
    (``shapes.engine_keepouts``: DLE-20 zarfları + 5 mm, susturucu + 15,5 mm hava boşluğu) — flanş, çerçeve ve iç
    yapı bunlara giremez (UP_cowl_* ↔ U_Env_* çakışması 0). Bölge dışındaki kesiciler parçayı etkilemez."""
    from .. import shapes as S
    # Arka açıklık kesicisi flanştan COWL_FLOW_CLEAR_M öne uzar: Ø92 akış silindirinin içine giren ek flanşları ve
    # yapıştırma dillerini de kırpar (entegrasyon R3: üst/yanak ek flanşları halka kesitinin %5'ini kapatıyordu,
    # r 33–45 mm). Kabuk bu silindirin dışındadır (s < 2,184'te iç yüz r ≥ 53 mm), motor zarfları r ≤ 33 mm.
    out = [md_solid(S.cowl_aft_opening(fwd=COWL_FLOW_CLEAR_M), "aft_opening"),
           md_solid(S.cooling_exit_cutter(), "chin_exit")]
    if part == "cowl_cheek_L":
        out.append(md_solid(S.muffler_pipe_hole(), "muffler_pipe"))
    out += [md_solid(md, f"keepout_{i}") for i, md in enumerate(S.engine_keepouts())]
    return out


def cowl_features(ctx: Ctx, seg: SegSpec) -> Feature:
    """Kaporta üst parçası: R01 delikleri/kırpmaları (``cowl_cutters``) + yangın perdesi 6×M3."""
    f = Feature(holes=cowl_cutters(seg.key), notes=["arka Ø92 açıklık + çene soğutma yarığı (açık)",
                                                    "motor zarfı yasak bölgeleri kırpıldı (5 / 15,5 mm)"])
    s_fw = float(P.SPEC["fuselage"]["modules"]["firewall_s_m"])
    for y, z in _ring_points(s_fw + 0.001, _WALLS["cowl_pa_cf"] + 0.004, [20, 90, 160, 200, 270, 340], [], 0.02):
        f.holes.append(prim_hole(Bp(s_fw - 0.002, y, z), Bp(s_fw + 0.008, y, z), M3_HOLE_D, name="fw_screw"))
    return f


def cheek_features(ctx: Ctx, seg: SegSpec) -> Feature:
    """Kaporta yanakları: R01 delikleri/kırpmaları; sağ yanakta 6 açık panjur yarığı (3,5 mm dolu panel), sol yanakta
    susturucu borusu deliği."""
    f = Feature(holes=cowl_cutters(seg.key), notes=["motor zarfı yasak bölgeleri kırpıldı (5 / 15,5 mm)"])
    if seg.key.endswith("_R"):
        f.structure += louver_panel_fill()
        f.holes += louver_slot_holes()
        f.notes.append("6 panjur yarığı açık (3 × 24 mm), panel 3,5 mm dolu")
    else:
        f.notes.append("susturucu borusu deliği (Ø12,4 + 3 mm) + çene soğutma yarığı yarısı")
    return f


def _ring_prism(ring_a: np.ndarray, ring_b: np.ndarray, name: str) -> Solid:
    """Eş indisli iki kapalı halka (dünya) arası prizma; yön işaretli hacimle düzeltilir."""
    n = len(ring_a)
    V = np.vstack([ring_a, ring_b])
    F = [tuple(range(n - 1, -1, -1)), tuple(range(n, 2 * n))]
    for i in range(n):
        j = (i + 1) % n
        F.append((i, j, n + j, n + i))
    sol = Solid(V, F, name)
    if _signed_volume(sol.V, sol.F) < 0:
        sol = Solid(V, [tuple(reversed(f)) for f in F], name)
    return sol


LOUVER_LIP_W = 0.0012                               # panjur dudağı eni (yarık çevresine DİK)
LOUVER_HOLE_GROW = 0.0001                           # yarık deliği dudak iç duvarından 0,1 mm büyük (çakışan yüz yok)


def _louver_frame(pts: np.ndarray, nrm: np.ndarray) -> np.ndarray:
    """Yarık çevresi noktalarında deri düzleminde DIŞA birim normal (çevreye dik). ``shapes.cowl_louvers`` dudak
    yönünü yarık merkezinden ışınsal alıyordu: uzun kenarlarda yön yarık eksenine yakın → dudak eni 1,2 mm yerine
    ≈ 0,17 mm (R14 kıymık); baskı dudağı burada çevreye dik kurulur."""
    t = np.roll(pts, -1, axis=0) - np.roll(pts, 1, axis=0)
    r = np.cross(t, nrm)
    r = r / np.maximum(np.linalg.norm(r, axis=1, keepdims=True), 1e-12)
    flip = np.einsum("ij,ij->i", r, pts - pts.mean(0)) < 0
    r[flip] *= -1.0
    return r


def louver_lip_solids() -> list[Solid]:
    """Panjur dudak çerçeveleri (baskı): yarık çevresinden dışa ``LOUVER_LIP_W`` (çevreye dik), deri normali boyunca
    −0,8…+``louver_lip_m`` mm — sahnedeki ``U_Cowl_Louvers`` yerine (bkz. ``_louver_frame``)."""
    from .. import shapes as S
    c = P.SPEC["propulsion"]["cowl"]["cheek_right"]
    lip = float(c.get("louver_lip_m", 0.0008))
    out = []
    for pts, nrm, _ in S._louver_slots():
        r = _louver_frame(pts, nrm)
        k = len(pts)
        rings = [pts + r * LOUVER_LIP_W - nrm * 0.0008, pts + r * LOUVER_LIP_W + nrm * lip, pts + nrm * lip,
                 pts - nrm * 0.0008]
        V = P.points_to_blender(np.vstack(rings))
        F = []
        for a in range(4):
            b = (a + 1) % 4
            for j in range(k):
                j2 = (j + 1) % k
                F.append((a * k + j, a * k + j2, b * k + j2, b * k + j))
        sol = Solid(np.asarray(V, float), F, "louver_lip")
        if _signed_volume(sol.V, sol.F) < 0:
            sol = Solid(sol.V, [tuple(reversed(f)) for f in F], "louver_lip")
        out.append(sol)
    return out


def louver_slot_holes() -> list[Solid]:
    """Sağ yanak panjur yarıkları (``shapes._louver_slots``): 1,6 mm deriyi ve 3,5 mm paneli delip geçen açık yarık
    (soğutma havası çıkışı); dudak iç duvarından ``LOUVER_HOLE_GROW`` büyük."""
    from .. import shapes as S
    out = []
    for pts, nrm, _ in S._louver_slots():
        q = pts + _louver_frame(pts, nrm) * LOUVER_HOLE_GROW
        Wp = np.column_stack([-q[:, 0], q[:, 1], q[:, 2]])
        Wn = np.column_stack([-nrm[:, 0], nrm[:, 1], nrm[:, 2]])
        out.append(_ring_prism(Wp - Wn * 0.007, Wp + Wn * 0.004, "louver_slot"))
    return out


def exhaust_ring_solid() -> Solid:
    """Lüle halkası baskı katısı (``shapes.exhaust_ring`` ile aynı istasyonlar): sahnedeki arka dudak, dış halkanın
    süperelips kesitinden ortalama yarıçaplı DAİREYE atlıyordu → 45° konumlarında 1–2 mm sivri sırt ve içte kıymık
    (R14). Burada dudak her açıda kendi dış yarıçapından iç yarıçapa yarım elipsle iner (basamaksız)."""
    from .. import shapes as S
    er = P.SPEC["propulsion"]["exhaust_ring"]
    s0, s1 = float(er["s_from_m"]), float(er["s_to_m"])
    ri = 0.5 * float(er["id_m"])
    zc = float(P.PROP.hub[2])
    # ön yüz kaporta arka flanşına alın alına yapışır (s0 + yapıştırma aralığı): sahnedeki 2 mm'lik geçme dili
    # kaporta flanşıyla 4,3 cm³ çakışıyordu (montaj denetimi)
    outer_s = [s0 + GLUE_GAP, s0 + 0.0012, s0 + 0.004, s1 - 0.006, s1 - 0.0025]
    shrink = [0.985, 0.992, 1.0, 1.0, 1.0]
    rings = []
    for s, f in zip(outer_s, shrink):
        yz = S.fuselage_ring_yz(min(max(s, s0), s1))
        rings.append(np.column_stack([np.full(len(yz), s), yz[:, 0] * f, zc + (yz[:, 1] - zc) * f]))
    last = rings[-1]
    ang = np.arctan2(last[:, 2] - zc, last[:, 1])
    ro = np.hypot(last[:, 1], last[:, 2] - zc)
    for a in np.linspace(0.0, math.pi, 9)[1:]:
        s = s1 - 0.0025 + 0.0025 * math.sin(a)
        r = 0.5 * (ro + ri) + 0.5 * (ro - ri) * math.cos(a)
        rings.append(np.column_stack([np.full(len(ang), s), r * np.cos(ang), zc + r * np.sin(ang)]))
    rings.append(np.column_stack([np.full(len(ang), s0 + GLUE_GAP), ri * np.cos(ang), zc + ri * np.sin(ang)]))
    m = len(ang)
    V = P.points_to_blender(np.vstack(rings))
    F = []
    nr = len(rings)
    for k in range(nr):
        k2 = (k + 1) % nr
        for j in range(m):
            j2 = (j + 1) % m
            F.append((k * m + j, k * m + j2, k2 * m + j2, k2 * m + j))
    sol = Solid(np.asarray(V, float), F, "exhaust_ring")
    if _signed_volume(sol.V, sol.F) < 0:
        sol = Solid(sol.V, [tuple(reversed(f)) for f in F], "exhaust_ring")
    return sol


def louver_panel_fill(t: float = 0.0035, margin: float = 0.005) -> list[Solid]:
    """Panjur paneli: yarıkların çevresinde deri ``t`` kalınlıkta dolu (sahnede 2 mm derin cepler 1,6 mm eti aşar;
    kaydırılmış boşluk yarık aralarında kıymık bırakmasın, yarıklar sağlam bir panelden geçsin)."""
    from .. import shapes as S
    slots = getattr(S, "_louver_slots", None)
    rings = [p for p, _, _ in slots()] if slots else []
    if not rings:
        return []
    pts = np.vstack(rings)
    s0, s1 = float(pts[:, 0].min()) - margin, float(pts[:, 0].max()) + margin
    z0, z1 = float(pts[:, 2].min()) - margin, float(pts[:, 2].max()) + margin
    y_out = float(pts[:, 1].min()) - 0.004                 # sağ yanak: y < 0, dış yüzün dışından
    y_in = float(pts[:, 1].max()) + t
    lo, hi = Bp(s1, y_out, z0), Bp(s0, y_in, z1)
    lo2, hi2 = np.minimum(lo, hi), np.maximum(lo, hi)
    return [prim_box(0.5 * (lo2 + hi2), np.eye(3), 0.5 * (hi2 - lo2), name="louver_panel")]


def well_roof_fill(gap_max: float = 0.004) -> list[Solid]:
    """Ana kuyu tavanı kanat üst derisine ≤ ``gap_max`` yaklaştığı bant (incelen arka-iç köşe; tavan = üst deri −
    1,5 mm): iki 0,8 mm et burada kesişir ve kaydırılmış boşluk ince kıymık bırakır → bant dolu basılır (yapı)."""
    from .. import shapes as S
    w = next((g for g in P.gear_wells() if g.name == SIDE), None)
    if w is None:
        return []
    poly = np.asarray(w.outline, float)
    a0, a1 = float(poly[:3, 0].min()), float(poly[:3, 0].max())
    b0, b1 = sorted((abs(float(poly[0, 1])), abs(float(poly[2, 1]))))
    roof = S.well_roof_z(SIDE)
    tab = S.wing_table(SIDE)
    ss = np.arange(a0, a1 + 1e-9, 0.001)
    s_x = None
    for s in ss:
        gaps = [tab.z(s, y, "upper") - roof(s, y) for y in np.linspace(b0, b1, 7)]
        gaps = [g for g in gaps if np.isfinite(g)]
        if gaps and min(gaps) < gap_max:
            s_x = float(s)
            break
    if s_x is None:
        return []
    grid = [(s, y) for s in np.linspace(s_x, a1, 12) for y in np.linspace(b0, b1, 12)]   # köşeler değil, ızgara:
    zs = [roof(s, y) for s, y in grid]                       # tavan iç bölgede (y 0,08–0,10) köşelerden 3 mm alçak
    zu = [tab.z(s, y, "upper") for s, y in grid]
    zlo, zhi = min(zs) - 0.003, max(z for z in zu if np.isfinite(z)) + 0.002
    lo = Bp(a1 + 0.004, b0 - 0.003, zlo)
    hi = Bp(s_x - 0.003, b1 + 0.003, zhi)
    c, h = 0.5 * (lo + hi), 0.5 * np.abs(hi - lo)
    return [prim_box(c, np.eye(3), h, name="solid_well_roof")]      # "solid*": takım yatağı keepout'u boşaltmaz


# =====================================================================================================
# Yük yolları (P4): kanat kutusu çerçeveleri + G10 köprü yuvaları, ana/burun takım yatakları, motor halkası,
# longeron kırık soketleri. Hepsi numpy ilkellerinden + boolean; komşu parçalarda karşılık gelen yuva/koruma
# hacimleri (``keepout``) aynı fonksiyonlardan türetilir → montajda tam oturma.
# =====================================================================================================
G10_T = float(_PR["g10_bridge"]["t_mm"]) / 1000.0          # 3 mm
G10_L = float(_PR["g10_bridge"]["length_mm"]) / 1000.0     # 300 mm (y ±0,15)
G10_H = float(_PR["g10_bridge"]["h_mm"]) / 1000.0         # 40 mm: kök bloğu profili (iç ≈ 42 mm) + depo tabanı
G10_CLEAR = 0.00015                                        # yuva boşluğu her yanda (3,3 mm yuva; epoksi dolgulu)
TANK_FLOOR_Z = -0.0165                                     # depo zarfı tabanı (params.tank_envelopes z0 −0,016) − 0,5 mm
TANK_HALF_Y = 0.0725                                       # depo zarfı dış yanı (0,072) + 0,5 mm
TANK_TOP_Z = 0.0665
WB_FRAME_T = 0.003                                         # kanat kutusu çerçevesi (PA-CF)
BRIDGE_BOLT_Y = (0.020, 0.050)                             # boru başına 2×M4: plaka–soket–plaka (|y|; panel borusu y ≥ 0,06)
GEAR_WALL = 0.0022                                         # takım beşiği duvarı (PA-CF)
GEAR_CLEAR = 0.0003                                        # ER-150 gövdesi ile beşik arası
ENGINE_RING_S = (2.0125, 2.0425)                           # motor halkası (yangın perdesi önü; halka 9 içinde 30 mm bindirme)
ENGINE_BOLT_SQ = float(P.SPEC["propulsion"]["engine"].get("mount_hole_spacing_m", 0.060))
LONGERON_SOCKET_END_S = 2.0275                             # longeron arka ucu (motor halkası soketi; M4 dişlileriyle eksenel ayrık)
LJ_LEN = 0.018                                             # longeron kırık soketi: kırığın her yanında 18 mm


def _fus_ring_yz(s: float, inset: float, nq: int = 24) -> np.ndarray:
    """Temiz gövde kesiti (y, z), içe ``inset`` kaydırılmış; sabit indis düzeni (``fuselage_outline``)."""
    yz = np.asarray(P.fuselage_outline(s, nq), float)
    c = yz.mean(0)
    n = len(yz)
    out = []
    for i in range(n):
        t = _unit(yz[(i + 1) % n] - yz[i - 1])
        nr = np.array([t[1], -t[0]])
        if (c - yz[i]) @ nr < 0:
            nr = -nr
        out.append(yz[i] + nr * inset)
    return np.asarray(out)


def fus_loft(s0: float, s1: float, inset_out: float, inset_in: float | None = None, n_st: int = 3,
             nq: int = 24, name: str = "fus_loft") -> Solid:
    """Gövde kesitleri boyunca loft: ``inset_out`` içe kaydırılmış dış yüz; ``inset_in`` verilirse halka (tüp),
    yoksa dolu katı. Blender ekseni (X = −s)."""
    ss = np.linspace(s0, s1, n_st)
    outer = [_fus_ring_yz(s, inset_out, nq) for s in ss]
    inner = [_fus_ring_yz(s, inset_in, nq) for s in ss] if inset_in is not None else None
    n = len(outer[0])
    V, F = [], []

    def ring_idx(k, which):
        base = (k * (2 if inner is not None else 1) + which) * n
        return [base + j for j in range(n)]

    for k, s in enumerate(ss):
        V += [(-s, y, z) for y, z in outer[k]]
        if inner is not None:
            V += [(-s, y, z) for y, z in inner[k]]
    m = len(ss)
    for k in range(m - 1):
        a, b = ring_idx(k, 0), ring_idx(k + 1, 0)
        for j in range(n):
            j2 = (j + 1) % n
            F.append((a[j], a[j2], b[j2], b[j]))
        if inner is not None:
            a, b = ring_idx(k, 1), ring_idx(k + 1, 1)
            for j in range(n):
                j2 = (j + 1) % n
                F.append((a[j], b[j], b[j2], a[j2]))
    if inner is None:
        F.append(tuple(reversed(ring_idx(0, 0))))
        F.append(tuple(ring_idx(m - 1, 0)))
    else:
        for k, sgn in ((0, -1), (m - 1, 1)):
            o, i_ = ring_idx(k, 0), ring_idx(k, 1)
            for j in range(n):
                j2 = (j + 1) % n
                q = (o[j], o[j2], i_[j2], i_[j])
                F.append(q if sgn > 0 else tuple(reversed(q)))
    sol = Solid(np.asarray(V, float), F, name)
    if _signed_volume(sol.V, sol.F) < 0:
        sol = Solid(sol.V, [tuple(reversed(f)) for f in sol.F], name)
    return sol


def socket_axis_z(y: float) -> float:
    """Merkez soket ekseni z(y) (spec), 4° dihedral V."""
    t = {x.name: x for x in P.spar_tubes(SIDE)}["centre_socket"]
    z0, z1, y1 = float(t.p0[2]), float(t.p1[2]), float(t.p1[1])
    return z0 + (z1 - z0) * abs(y) / y1


def g10_plate_planes() -> list[tuple[str, float, float]]:
    """İki G10 köprü plakası: (ad, ön yüz s, arka yüz s). Soketin (Ø30) önüne ve arkasına teğet."""
    r = 0.5 * float({x.name: x for x in P.spar_tubes(SIDE)}["centre_socket"].od)
    s = P.WING_SPAR_S
    return [("ön", s - r - G10_T, s - r), ("arka", s + r, s + r + G10_T)]


def g10_plate_z(y: float, grow: float = 0.0) -> tuple[float, float]:
    """Köprü plakasının y'deki alt/üst z'si (40 mm, soket eksenine ortalı; gövde içinde depo tabanında kesik)."""
    zc = socket_axis_z(y)
    lo, hi = zc - 0.5 * G10_H, zc + 0.5 * G10_H
    hi = min(hi, TANK_FLOOR_Z)
    return lo - grow, hi + grow


def g10_plate_solid(which: int, grow_t: float = 0.0, grow_h: float = 0.0, y0: float = -0.5 * G10_L,
                    y1: float = 0.5 * G10_L, name: str = "g10") -> Solid:
    """Köprü plakası katısı (``which`` 0 ön / 1 arka), kalınlıkta ``grow_t``, yükseklikte ``grow_h`` büyütülmüş."""
    _, sa, sb = g10_plate_planes()[which]
    ys = np.unique(np.r_[np.linspace(y0, y1, 13), [v for v in (0.0,) if y0 < v < y1]])
    lo = [g10_plate_z(y, grow_h)[0] for y in ys]
    hi = [g10_plate_z(y, grow_h)[1] for y in ys]
    poly = np.array([(y, z) for y, z in zip(ys, lo)] + [(y, z) for y, z in zip(ys[::-1], hi[::-1])])
    return prim_prism(poly, Bp(sa - grow_t, 0, 0), np.array([0, 1.0, 0]), np.array([0, 0, 1.0]),
                      np.array([-1.0, 0, 0]), 0.0, (sb - sa) + 2 * grow_t, name=name)


def wing_frame_parts(which: int) -> dict:
    """Kanat kutusu çerçevesi (PA-CF, 3 mm): köprü plakasının dış yüzüne yaslanan sabit-s perde; depo zarfının
    çevresinde halka (alt bant + yan dikmeler + üst bant), 4 longeron soketi (yaka), 4 köprü cıvatası (M4).
    Dönüş: {"base", "adds", "holes", "s"}."""
    _, sa, sb = g10_plate_planes()[which]
    if which == 0:
        s1 = sa - G10_CLEAR
        s0 = s1 - WB_FRAME_T
    else:
        s0 = sb + G10_CLEAR
        s1 = s0 + WB_FRAME_T
    base = fus_loft(s0, s1, _wall_of("fuselage_mid") + 0.0002, n_st=2, name="wf_base")
    holes = []
    tank = prim_box(Bp(0.5 * (s0 + s1), 0.0, 0.5 * (TANK_FLOOR_Z + TANK_TOP_Z)), np.eye(3),
                    (0.5 * (s1 - s0) + 0.01, TANK_HALF_Y, 0.5 * (TANK_TOP_Z - TANK_FLOOR_Z)), name="tank_ko")
    holes.append(tank)
    yb = 0.040                                                       # plakanın altında hafifletme deliği
    z_hi = g10_plate_z(yb)[0] - 0.005
    sec = P.fuselage_section(0.5 * (s0 + s1))
    z_lo = sec.z_bottom + _wall_of("fuselage_mid") + 0.007
    if z_hi - z_lo > 0.006:
        holes.append(prim_box(Bp(0.5 * (s0 + s1), 0.0, 0.5 * (z_lo + z_hi)), np.eye(3),
                              (0.5 * (s1 - s0) + 0.01, yb, 0.5 * (z_hi - z_lo)), name="wf_light"))
    sm = 0.5 * (s0 + s1)                                             # R04: depo üstü bantta hafifletme deliği
    sec_m = P.fuselage_section(sm)
    z_t0 = TANK_TOP_Z + 0.006
    z_t1 = sec_m.z_top - _wall_of("fuselage_mid") - 0.012
    y_sh = min(abs(_longeron_at("shoulder", sd, sm)[1]) for sd in ("L", "R"))
    y_t = min(y_sh - 0.0045 - 0.0035 - 0.008, (P.fuselage_half_width_at(sec_m, z_t1) or 0.0) - 0.008)
    if z_t1 - z_t0 > 0.008 and y_t > 0.010:
        holes.append(prim_box(Bp(sm, 0.0, 0.5 * (z_t0 + z_t1)), np.eye(3), (0.5 * (s1 - s0) + 0.01, y_t,
                                                                           0.5 * (z_t1 - z_t0)), name="wf_light_top"))
    for y in BRIDGE_BOLT_Y:                                          # köprü cıvataları: plaka–soket–plaka (s boyunca)
        for sg in (1.0, -1.0):
            z = socket_axis_z(y)
            holes.append(prim_hole(Bp(s0 - 0.004, sg * y, z), Bp(s1 + 0.004, sg * y, z),
                                   hole_d(0.004, "bolt", "PA-CF"), name="wf_bolt"))
    adds = []
    for kind in ("chine", "shoulder"):
        for sd in ("L", "R"):
            sa_, sb_ = (s0 - 0.009, s1) if which == 0 else (s0, s1 + 0.009)   # yaka yalnız dış yüzde (iç yüz düz)
            pa, pb = _longeron_at(kind, sd, sa_), _longeron_at(kind, sd, sb_)
            d = _unit(pb - pa)
            r = 0.5 * hole_d(0.008, "tube", "PA-CF")
            adds.append(prim_frustum(pa, pb, r + 0.002, n=32, name="wf_collar"))
            holes.append(prim_hole(pa - d * 0.01, pb + d * 0.01, 2 * r, name="wf_long"))
    ca, cb = (s0 - 0.012, s1) if which == 0 else (s0, s1 + 0.012)   # düz yüz (köprüye yaslanan) tam düzlem: tabla yüzü
    clip = fus_loft(ca, cb, _wall_of("fuselage_mid") + 0.0002, n_st=3, name="wf_clip")
    return {"base": base, "adds": adds, "holes": holes, "clip": [clip], "s": (s0, s1)}


def wing_frame_keepout(which: int) -> Solid:
    """Çerçeve + yakalar + 0,3 mm (gövde halkası iç yapısı girmez)."""
    p = wing_frame_parts(which)
    s0, s1 = p["s"]
    return fus_loft(s0 - 0.0063, s1 + 0.0063, _wall_of("fuselage_mid") + 0.0001, n_st=2, name="wf_ko")


def g10_slot_features(y_from: float, y_to: float = 0.5 * G10_L + 0.0015) -> Feature:
    """Kök bloğu 1'de iki G10 köprü yuvası (3,3 mm × 40,4 mm, kör uç) + yuva duvarları (1,2 mm iç yapı)."""
    f = Feature()
    for which in (0, 1):
        f.holes.append(g10_plate_solid(which, G10_CLEAR, 0.0002, y_from, y_to, name="g10_slot"))
        f.structure.append(g10_plate_solid(which, G10_CLEAR + 0.0012, 0.0014, y_from, y_to + 0.0012, name="g10_wall"))
    f.notes.append("G10 köprü yuvaları 2 × 3,3 × 40,4 mm")
    return f


# ---------------------------------------------------------------- ana takım yatağı (ER-150)
def _unit_frame() -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    env, _ = S_main_unit_box()
    A = np.asarray(env.axes, float)                 # satırlar: s, açıklık (dihedral), düşey
    return np.asarray(env.center, float), A, np.asarray(env.half, float)


def S_main_unit_box():
    from .. import shapes as S
    return S.main_unit_box(SIDE)


def _local_box(c, A, lo, hi, name="lbox") -> Solid:
    """Ünite yerel çerçevesinde (A satırları) [lo, hi] kutusu → Blender katısı."""
    lo, hi = np.asarray(lo, float), np.asarray(hi, float)
    cl = c + A.T @ (0.5 * (lo + hi))
    axes_b = np.column_stack([Bv(A[0]), Bv(A[1]), Bv(A[2])])
    return prim_box(Bp(*cl), axes_b, 0.5 * (hi - lo), name=name)


def _ceiling_solid(s0, s1, y0, y1, z_fn, z_bot, n=9, name="ceil", ns: int | None = None) -> Solid:
    """Yükseklik alanı katısı: üst yüz ``z_fn(s, y)``, alt düz ``z_bot`` (spec), kenarlar düşey. ``ns`` s yönünde
    örnek sayısı (varsayılan ``n``)."""
    ns = ns or n
    ss, ys = np.linspace(s0, s1, ns), np.linspace(y0, y1, n)
    m = n
    V, F = [], []
    for s in ss:
        for y in ys:
            V.append(Bp(s, y, z_fn(s, y)))
    for s in ss:
        for y in ys:
            V.append(Bp(s, y, z_bot))
    top = lambda i, j: i * m + j
    bot = lambda i, j: ns * m + i * m + j
    for i in range(ns - 1):
        for j in range(m - 1):
            F.append((top(i, j), top(i + 1, j), top(i + 1, j + 1), top(i, j + 1)))
            F.append((bot(i, j), bot(i, j + 1), bot(i + 1, j + 1), bot(i + 1, j)))
    edges = [[(i, 0) for i in range(ns)], [(ns - 1, j) for j in range(m)], [(i, m - 1) for i in range(ns - 1, -1, -1)],
             [(0, j) for j in range(m - 1, -1, -1)]]
    for e in edges:
        for (i, j), (i2, j2) in zip(e[:-1], e[1:]):
            F.append((top(i, j), bot(i, j), bot(i2, j2), top(i2, j2)))
    sol = Solid(np.asarray(V, float), F, name)
    if _signed_volume(sol.V, sol.F) < 0:
        sol = Solid(sol.V, [tuple(reversed(f)) for f in sol.F], name)
    return sol


def _slot_keepout(grow: float = 0.0015) -> Solid:
    """Ana bacak yuvası (eğik açıklık, ``params._leg_slot``) + astar duvarı + pay: ağzın altından tavanın üstüne."""
    sl = np.asarray(P._leg_slot(SIDE), float)       # (s, y) dörtgen
    c = sl.mean(0)
    out = []
    for i in range(len(sl)):
        a, b, cc = sl[i - 1], sl[i], sl[(i + 1) % len(sl)]
        e1, e2 = _unit(b - a), _unit(cc - b)
        n1, n2 = np.array([-e1[1], e1[0]]), np.array([-e2[1], e2[0]])
        if (c - b) @ n1 > 0:
            n1, n2 = -n1, -n2
        bis = _unit(n1 + n2)
        out.append(b + bis * grow / max(0.3, float(bis @ n1)))
    out = np.asarray(out)
    V2 = np.column_stack([-out[:, 0], out[:, 1]])
    if 0.5 * np.sum(V2[:, 0] * np.roll(V2[:, 1], -1) - np.roll(V2[:, 0], -1) * V2[:, 1]) < 0:
        V2 = V2[::-1]
    roof = -0.021 + 0.0008 + 0.0003
    return prim_prism(V2, np.zeros(3), np.array([1.0, 0, 0]), np.array([0, 1.0, 0]), np.array([0, 0, 1.0]),
                      -0.09, roof, name="slot_ko")


GEAR_ARM_Y = (0.2410, 0.2670)                    # yuva tavanı üstünden sokete uzanan kol (kök bloğu 2 flanş bandının dışında)
GEAR_CRADLE_Y = (0.2325, 0.3175)                 # beşik (kök bloğu 2 içinde; iç uç açık, dış uç G10 plakada)


def gear_mount_main_parts() -> dict:
    """``gear_mount_L`` (PA-CF): ER-150 gövdesini önden/arkadan/alttan saran beşik (0,3 mm boşluk), yuva tavanının
    üstünden Ø30 merkez sokete uzanan eyerli kol, ön duvarda 2 × M3 ısıl gömme dişli (ünite cıvataları). Spec
    koordinatı → Blender. Dönüş: {"base", "adds", "holes", "clip", "keepout"}."""
    c, A, h = _unit_frame()
    w, cl = GEAR_WALL, GEAR_CLEAR
    span_of = lambda y: (y - c[1]) / A[1, 1]
    sp0, sp1 = span_of(GEAR_CRADLE_Y[0]), span_of(GEAR_CRADLE_Y[1])
    base = _local_box(c, A, (-h[0] - cl - w, sp0, -h[2] - cl - w), (h[0] + cl + w, sp1, h[2] - 0.0025), "cradle")
    holes = [_local_box(c, A, (-h[0] - cl, sp0 - 0.01, -h[2] - cl), (h[0] + cl, sp1 + 0.01, h[2] + 0.02), "unit_pocket")]
    up_in = lambda s, y: P.wing_surface_z(s, y, "upper") - _wall_of("root_block") - 0.0021   # flanş bandının altı
    s_sock = P.WING_SPAR_S
    s_front = c[0] - h[0] - cl - w
    arm_bot = -0.021 + 0.0008 + 0.0003
    arm = _ceiling_solid(s_sock, s_front + 0.0015, GEAR_ARM_Y[0], GEAR_ARM_Y[1], up_in, arm_bot, n=7, name="arm")
    adds = [arm, _local_box(c, A, (-h[0] - cl - w, span_of(GEAR_ARM_Y[0]), -h[2] - cl - w),
                            (-h[0] - cl, span_of(GEAR_ARM_Y[1]), h[2] + 0.02), "arm_post")]   # kol–beşik bağlantısı
    t = {x.name: x for x in P.spar_tubes(SIDE)}["centre_socket"]
    sock = prim_hole(Bp(*t.p0) + np.array([0.0, -0.01, 0.0]), Bp(*t.p1), hole_d(t.od, "tube", "PA-CF"), name="sock")
    holes.append(sock)
    for y in (0.280, 0.310):                         # ön duvar: 2 × M3 ısıl gömme dişli (eksen s), dış yüzde göbek
        sp = span_of(y)
        p_in = c + A.T @ np.array([-h[0] - cl, sp, 0.0])
        p_out = c + A.T @ np.array([-h[0] - cl - w - 0.0050, sp, 0.0])
        adds.append(prim_frustum(Bp(*p_in), Bp(*p_out), 0.0036, n=32, name="m3_boss"))
        di, dd = INSERT["M3"]
        p_deep = c + A.T @ np.array([-h[0] - cl - w - 0.0050 + dd, sp, 0.0])
        holes.append(prim_hole(Bp(*p_out) + Bv(A[0]) * -0.0005, Bp(*p_deep), di, name="m3_insert"))
    s_aft = c[0] + h[0] + cl + w + 0.0015
    ceiling = _ceiling_solid(P.WING_SPAR_S - 0.02, s_aft, GEAR_CRADLE_Y[0] - 0.01, GEAR_CRADLE_Y[1] + 0.01,
                             up_in, -0.09, n=11, ns=33, name="ceil")
    from .. import shapes as S
    blister_in = lambda s, y: S.blister_bottom_z(s, y, SIDE) + _wall_of("root_fairing") + 0.0003
    floor = _ceiling_solid(P.WING_SPAR_S - 0.02, s_aft, GEAR_CRADLE_Y[0] - 0.01, GEAR_CRADLE_Y[1] + 0.01,
                           blister_in, 0.05, n=11, ns=33, name="floor")
    holes += conduit_holes(root_conduit_polyline(), CONDUIT_D["root"])          # P9: kablo kanalı koldan geçer
    clips = [ceiling, floor]
    aft = _wing_aft_clip(GEAR_CRADLE_Y[0] - 0.01, GEAR_CRADLE_Y[1] + 0.01, float(c[2]), float(c[0]))
    if aft is not None:                                  # kanat firar bölgesi / flap oyuğu − 0,2 mm
        clips.append(aft)
    hp = _HINGE_PLANS.get("FlapIn")                      # montaj denetimi: beşik arka duvarı kök bloğu 2'deki iç flap
    if hp is not None:                                   # menteşe diline 0,8 mm giriyordu (y 0,309–0,318) → dil +
        for u in hp.lugs:                                # 0,2 mm beşikten çıkarılır
            if GEAR_CRADLE_Y[0] - 0.02 < float(hp.point(u)[1]) < GEAR_CRADLE_Y[1] + 0.02:
                for sl in hinge_lug_prims(hp, u):
                    V_, T_ = solid_triangles(sl)
                    holes.append(Solid(offset_vertices(V_, T_, -GLUE_GAP), [tuple(x) for x in T_], "lug_clear"))
    return {"base": base, "adds": adds, "holes": holes + [_slot_keepout(0.0015)], "clip": clips}


_HINGE_PLANS: dict = {}                             # make_recipes'in son menteşe planları (ad → HingePlan)


def _wing_aft_clip(y0: float, y1: float, z: float, s_in: float, gap: float = GLUE_GAP) -> Solid | None:
    """Kanat merkez kaynağının (``U_WingCenter_<SIDE>``) ``s_in``'den arkaya ilk yüzeyi (flap oyuğu / firar
    bölgesi) y boyunca ışınla bulunur; bu çizginin ``gap`` önünde biten dik prizma (kırpma katısı, Blender ekseni)."""
    nm = f"U_WingCenter_{SIDE}"
    if nm not in bpy.data.objects:
        return None
    src = source_solid(nm)
    T = [list(f) for f in src.F]
    bvh = BVHTree.FromPolygons(np.asarray(src.V, float).tolist(), T, epsilon=0.0)
    pts = []
    for y in np.linspace(y0, y1, 9):
        best = None
        for dz in (-0.006, 0.0, 0.006):
            h = bvh.ray_cast(Vector(tuple(Bp(s_in, float(y), z + dz))), Vector((-1.0, 0.0, 0.0)), 0.2)
            if h[0] is not None:
                sh = -float(h[0][0])
                best = sh if best is None else min(best, sh)
        if best is None:
            return None
        pts.append((best - gap, float(y)))
    poly = [(s_in - 0.2, y0 - 0.001)] + [(s, y) for s, y in pts] + [(s_in - 0.2, y1 + 0.001)]
    poly2 = np.array([(-s, y) for s, y in poly])            # Blender x = −s
    if _poly_area2(poly2) < 0:
        poly2 = poly2[::-1]
    return prim_prism(poly2, np.zeros(3), np.array([1.0, 0, 0]), np.array([0, 1.0, 0]), np.array([0, 0, 1.0]),
                      z - 0.2, z + 0.2, name="aft_clip")


def gear_mount_main_keepout() -> list:
    """Kök bloklarında iç yapının girmeyeceği hacimler: beşik + kol (+0,3 mm, kök bloğu 2'nin açık iç ucuna dek
    uzatılmış: montajda beşik y yönünde kaydırılarak yerleşir) ve ER-150 gövdesi (+0,5 mm)."""
    c, A, h = _unit_frame()
    w, cl = GEAR_WALL, GEAR_CLEAR
    span_of = lambda y: (y - c[1]) / A[1, 1]
    rb_mid = float(P.print_cuts()["root_block_y"][1])
    ko = [_local_box(c, A, (-h[0] - cl - w - 0.0055, span_of(rb_mid - 0.006), -h[2] - cl - w - 0.0003),
                     (h[0] + cl + w + 0.0003, span_of(GEAR_CRADLE_Y[1]) + 0.0003, h[2] + 0.0005), "cradle_ko"),
          _local_box(c, A, (-h[0] - 0.0005, -h[1] - 0.0005, -h[2] - 0.0005), (h[0] + 0.0005, h[1] + 0.0005,
                                                                              h[2] + 0.0005), "unit_ko")]
    up_in = lambda s, y: P.wing_surface_z(s, y, "upper") - _wall_of("root_block") - 0.0018
    ko.append(_ceiling_solid(P.WING_SPAR_S - 0.016, c[0] - h[0], rb_mid - 0.006, GEAR_ARM_Y[1] + 0.0003, up_in,
                             -0.021 + 0.0008, n=7, name="arm_ko"))
    return ko


def gear_unit_opening() -> Solid:
    """Kök bloklarının alt derisinde ER-150 + beşik açıklığı (kabartma kaportası altını kapatır); iç uca dek
    (beşik/ünite y yönünde takılır)."""
    c, A, h = _unit_frame()
    w, cl = GEAR_WALL, GEAR_CLEAR
    span_of = lambda y: (y - c[1]) / A[1, 1]
    return _local_box(c, A, (-h[0] - cl - w - 0.0003, span_of(0.2130), -h[2] - 0.03),
                      (h[0] + cl + w + 0.0003, span_of(GEAR_CRADLE_Y[1]) + 0.0003, -h[2] + 0.008), "unit_open")


# ---------------------------------------------------------------- burun takım yatağı
NOSE_MOUNT_S = (0.4080, 0.4860)


def gear_mount_nose_parts() -> dict:
    """``gear_mount_N`` (PA-CF, 4,3 mm): burun ER-150'nin G10 plakası üstünde, iki chine longeronuna eyerle yaslanan
    yatak plakası (akü kızağı tabanı); 4 × M3 ısıl gömme dişli (alttan, plaka cıvataları)."""
    z0, z1 = -0.0138, -0.0095
    s0, s1 = NOSE_MOUNT_S
    yh = 0.0905
    base = prim_box(Bp(0.5 * (s0 + s1), 0.0, 0.5 * (z0 + z1)), np.eye(3), (0.5 * (s1 - s0), yh, 0.5 * (z1 - z0)),
                    name="nose_plate")
    adds, holes = [], []
    for sd in ("L", "R"):
        pa, pb = _longeron_at("chine", sd, s0), _longeron_at("chine", sd, s1)
        d = _unit(pb - pa)
        r = 0.5 * hole_d(0.008, "tube", "PA-CF")
        adds.append(prim_frustum(pa, pb, r + 0.0020, n=32, name="nm_collar"))
        holes.append(prim_hole(pa - d * 0.01, pb + d * 0.01, 2 * r, name="nm_long"))
    for yc in (-0.0475, 0.0475):                                    # hafifletme (akü bu tabana oturur)
        holes.append(prim_box(Bp(0.4470, yc, 0.5 * (z0 + z1)), np.eye(3), (0.0215, 0.0215, 0.01), name="nm_light"))
    di, dd = INSERT["M3S"]
    for s in (0.416, 0.478):
        for y in (-0.015, 0.015):
            holes.append(prim_hole(Bp(s, y, z0 - 0.001), Bp(s, y, z0 + dd), di, name="nm_insert"))
    clip = fus_loft(s0 - 0.01, s1 + 0.01, _wall_of("fuselage_nose") + 0.0003, n_st=3, name="nm_clip")
    above = prim_box(Bp(0.5 * (s0 + s1), 0.0, z0 + 0.03), np.eye(3), (0.5 * (s1 - s0) + 0.02, 0.15, 0.03), name="nm_up")
    return {"base": base, "adds": adds, "holes": holes, "clip": [clip, above]}


def gear_mount_nose_keepout() -> Solid:
    """Yatak + yakalar + 0,3 mm; ön uca (s 0,40, halka 1 çerçeve açıklığı) dek: plaka önden kaydırılarak takılır."""
    s0, s1 = NOSE_MOUNT_S
    sa = float(P.SPEC["fuselage"]["modules"]["nose_module_s_m"][1]) - 0.005
    return prim_box(Bp(0.5 * (sa + s1 + 0.0003), 0.0, -0.0102), np.eye(3), (0.5 * (s1 + 0.0003 - sa), 0.105, 0.0068),
                    name="nm_ko")


# ---------------------------------------------------------------- motor halkası
def engine_bolt_points() -> list[tuple[float, float]]:
    """DLE-20 montaj deliklerinin yangın perdesindeki (y, z) konumları: krank eksenine ortalı kare."""
    hub, a = P.thrust_axis()
    s_fw = float(P.SPEC["fuselage"]["modules"]["firewall_s_m"])
    zc = float(hub[2] + (s_fw - hub[0]) * a[2] / a[0])
    h = 0.5 * ENGINE_BOLT_SQ
    return [(sy * h, zc + sz * h) for sy in (-1.0, 1.0) for sz in (-1.0, 1.0)]


def engine_ring_parts() -> dict:
    """``engine_ring`` (PA-CF): halka 9'un içine 30 mm bindiren 3 mm bilezik + krank eksenine ortalı 60 mm kare motor
    çerçevesi (4 × M4 ısıl gömme dişli, arka yüzden), göbeklerden bileziğe yatay kollar, 4 longeron soketi (ön yüz,
    15 mm kör) ve kaporta için 6 × M3 dişli; NACA kanal açıklığı kesik."""
    s0, s1 = ENGINE_RING_S
    w_skin = _wall_of("tail_cone")
    band = fus_loft(s0, s1, w_skin + 0.0002, w_skin + 0.0032, n_st=3, name="er_band")
    adds, holes = [], []
    di, dd = INSERT["M4"]
    sf = s1 - 0.010                                                  # çerçeve: arka 10 mm
    pts = engine_bolt_points()
    for y, z in pts:
        adds.append(prim_frustum(Bp(sf, y, z), Bp(s1, y, z), 0.0058, n=32, name="er_boss"))
        holes.append(prim_hole(Bp(s1 + 0.001, y, z), Bp(s1 - dd, y, z), di, name="er_m4"))
    ys = sorted({p[0] for p in pts})
    zs = sorted({p[1] for p in pts})
    sc = 0.5 * (sf + s1)
    for z in zs:                                                     # yatay kollar (bileziğe dek) ve kare kenarları
        adds.append(prim_box(Bp(sc, 0.0, z), np.eye(3), (0.5 * (s1 - sf), 0.066, 0.0022), name="er_rib_h"))
    for y in ys:
        adds.append(prim_box(Bp(sc, y, 0.5 * (zs[0] + zs[1])), np.eye(3), (0.5 * (s1 - sf), 0.0022,
                                                                          0.5 * (zs[1] - zs[0])), name="er_rib_v"))
    r = 0.5 * hole_d(0.008, "tube", "PA-CF")
    for kind in ("chine", "shoulder"):                              # longeron soketleri (ön yüzden kör)
        for sd in ("L", "R"):
            pe = _longeron_at(kind, sd, LONGERON_SOCKET_END_S)
            pa = _longeron_at(kind, sd, s0)
            d = _unit(pe - pa)
            adds.append(prim_frustum(pa, pe + d * 0.003, r + 0.0022, n=32, name="er_sock"))
            holes.append(prim_hole(pa - d * 0.003, pe, 2 * r, name="er_long"))
            q = pe + d * 0.0015
            yy, zz = float(q[1]), float(q[2])                        # sokete bilezik bağlantısı (dışa kol)
            sec_hw = P.fuselage_half_width_at(0.5 * (s0 + s1), zz) or 0.06
            ym = np.sign(yy) * (sec_hw - w_skin)
            adds.append(prim_box(Bp(0.5 * (s0 + LONGERON_SOCKET_END_S), 0.5 * (yy + ym), zz), np.eye(3),
                                 (0.5 * (LONGERON_SOCKET_END_S - s0), 0.5 * abs(ym - yy) + 0.001, 0.0022),
                                 name="er_sock_web"))
    s_fw = float(P.SPEC["fuselage"]["modules"]["firewall_s_m"])
    di3, dd3 = INSERT["M3"]
    for y, z in _ring_points(s_fw + 0.001, _WALLS["cowl_pa_cf"] + 0.004, [20, 90, 160, 200, 270, 340], [], 0.02):
        adds.append(prim_frustum(Bp(s1 - 0.008, y, z), Bp(s1, y, z), 0.0038, n=24, name="er_m3_boss"))
        holes.append(prim_hole(Bp(s1 + 0.001, y, z), Bp(s1 - dd3, y, z), di3, name="er_m3"))
    from .. import shapes as S
    cut = S.intake_cutter()
    holes.append(Solid(np.asarray(cut.verts_blender(), float), [tuple(f) for f in cut.faces], "er_naca"))
    holes.append(intake_duct_cut())
    clip = fus_loft(s0 - 0.004, s1 + 0.004, w_skin + 0.0002, n_st=4, name="er_clip")
    return {"base": band, "adds": adds, "holes": holes, "clip": [clip],
            "subtract_sources": [("intake", GLUE_GAP)]}             # NACA dudak çerçevesi (1,1 cm³ çakışma)


def engine_ring_band_clear() -> Solid:
    """Halka 9'un arka yapıştırma flanşı (``LAND_T`` 2 mm) motor halkası bileziğiyle (deriden 1,2–4,2 mm) 0,8 mm
    çakışıyordu ve NACA kanal köşesinde 0,04–0,15 mm'lik kıymık bırakıyordu (R14). Bindirme boyunca halka 9 yalnız
    derisini (+0,1 mm) tutar, bilezik bölgesi (+ yapıştırma aralığı) çıkarılır: flanş görevini bilezik görür."""
    s0, s1 = ENGINE_RING_S
    w_skin = _wall_of("tail_cone")
    return fus_loft(s0 - GLUE_GAP, s1 + 0.003, w_skin + 0.0001, w_skin + 0.0032 + GLUE_GAP, n_st=5, nq=48,
                    name="er_band_clear")


def engine_ring_keepout() -> Solid:
    s0, s1 = ENGINE_RING_S
    return fus_loft(s0 - 0.0005, s1 + 0.003, 0.0001, n_st=3, name="er_ko")


# ---------------------------------------------------------------- longeron kırık soketleri (s 1,167)
def longeron_break_s() -> float:
    return float(P.SPEC["fuselage"]["longerons"]["breaks_s_m"][0])


def longeron_joint_parts(kind: str) -> dict:
    """``longeron_joint_<tür>`` (PETG): s = 1,167 kırığında iki düz longeron parçasını kırık açısıyla buluşturan
    soket bloğu: iki Ø8,6 kör yuva (her biri 18 mm), Ø12,6 gövde, gövde iç yüzüne kırpılmış."""
    sb = longeron_break_s()
    pieces = P.longeron_pieces(kind, SIDE)
    (a1, b1), (a2, b2) = pieces[0], pieces[1]
    B = Bp(*b1)
    d1 = _unit(Bp(*b1) - Bp(*a1))
    d2 = _unit(Bp(*b2) - Bp(*a2))
    r = 0.5 * hole_d(0.008, "tube", "PETG")
    adds = [prim_frustum(B - d1 * LJ_LEN, B + d1 * 0.002, r + 0.0022, n=32, name="lj_a"),
            prim_frustum(B - d2 * 0.002, B + d2 * LJ_LEN, r + 0.0022, n=32, name="lj_b")]
    holes = [prim_hole(B - d1 * (LJ_LEN + 0.002), B - d1 * 0.0008, 2 * r, name="lj_ha"),
             prim_hole(B + d2 * 0.0008, B + d2 * (LJ_LEN + 0.002), 2 * r, name="lj_hb"),
             prim_frustum(B - d1 * 0.0009, B + d2 * 0.0009, 0.002, n=16, name="lj_vent")]
    clip = fus_loft(sb - 0.03, sb + 0.03, _wall_of("fuselage_mid") + 0.0004, n_st=3, name="lj_clip")
    return {"base": adds[0], "adds": adds[1:], "holes": holes, "clip": [clip]}


def longeron_joint_pocket_parts(kind: str, side: str) -> dict:
    """Halka 4/5'te soket bloğu cebi: blok + 0,3 mm, deri iç yüzünden 0,1 mm uzak (gövde içine kırpılır)."""
    sb = longeron_break_s()
    pieces = P.longeron_pieces(kind, side)
    (a1, b1), (a2, b2) = pieces[0], pieces[1]
    B = Bp(*b1)
    d1, d2 = _unit(Bp(*b1) - Bp(*a1)), _unit(Bp(*b2) - Bp(*a2))
    r = 0.5 * hole_d(0.008, "tube", "PETG") + 0.0022 + 0.0003
    a = prim_frustum(B - d1 * (LJ_LEN + 0.0003), B + d1 * 0.0023, r, n=32, name="ljp_a")
    b = prim_frustum(B - d2 * 0.0023, B + d2 * (LJ_LEN + 0.0003), r, n=32, name="ljp_b")
    clip = fus_loft(sb - 0.03, sb + 0.03, _wall_of("fuselage_mid") + 0.0001, n_st=3, name="ljp_clip")
    return {"base": a, "adds": [b], "holes": [], "clip": [clip]}


# =====================================================================================================
# Sökülebilir dış panel tutma, kablo kanalları, MPX cepleri, iç flap bağlayıcı (P9)
# =====================================================================================================
LOCK_TAB = {"s": 1.200, "w": 0.012, "h": 0.008, "y0": 0.342, "y1": 0.378, "y_bolt": 0.351, "clear": 0.0002}
MPX = {"s": 1.2900, "w": 0.026, "h": 0.013, "depth": 0.015, "clear": 0.0003}      # MPX 6 pin gövdesi + pay. varsayım
CONDUIT_D = {"wing": 0.008, "root": 0.006, "tail": 0.006}
FLAP_JOINER = {"r_off": 0.012, "d": 0.002, "socket": 0.015, "slot_w": 0.0028, "deg": (-3.0, 33.0)}   # varsayım


def lock_tab_frame() -> tuple[np.ndarray, float]:
    """Tutma dilinin merkezi (Blender) ve z merkezi: s = 1,200 (D-kutu bölgesi), y = 0,36 ekinin iki yanında."""
    T = LOCK_TAB
    zc = P.wing_mid_z(T["s"], 0.36)
    zc = float(zc if zc is not None else -0.015)
    return Bp(T["s"], 0.5 * (T["y0"] + T["y1"]), zc), zc


def lock_tab_parts() -> dict:
    """``panel_lock_tab`` (PETG): 12 × 8 × 36 mm dil; dış panel 1'in kök kaburgasına yapıştırılır, kök bloğu 2'nin
    cebine girer; kök bloğu tarafında alttan M4 naylon cıvata için ısıl gömme dişli (dikey, boydan boya)."""
    T = LOCK_TAB
    c, zc = lock_tab_frame()
    base = prim_box(c, np.eye(3), (0.5 * T["w"], 0.5 * (T["y1"] - T["y0"]), 0.5 * T["h"]), name="tab")
    di, _ = INSERT["M4"]
    pb = Bp(T["s"], T["y_bolt"], zc)
    holes = [prim_hole(pb - np.array([0, 0, 0.006]), pb + np.array([0, 0, 0.006]), di, name="tab_insert")]
    return {"base": base, "adds": [], "holes": holes, "clip": []}


def lock_tab_features(part: str) -> Feature:
    """Dil cebi (+0,2 mm) ve cep duvarları (1,2 mm iç yapı); kök bloğu 2'de alttan Ø4,5 cıvata deliği + göbek."""
    T = LOCK_TAB
    c, zc = lock_tab_frame()
    f = Feature()
    cl = T["clear"]
    hy = 0.5 * (T["y1"] - T["y0"]) + 0.0005
    f.holes.append(prim_box(c, np.eye(3), (0.5 * T["w"] + cl, hy, 0.5 * T["h"] + cl), name="tab_pocket"))
    f.structure.append(prim_box(c, np.eye(3), (0.5 * T["w"] + cl + 0.0012, hy + 0.0012, 0.5 * T["h"] + cl + 0.0012),
                                name="tab_wall"))
    if part == "root2":
        pb = Bp(T["s"], T["y_bolt"], zc)
        z_lo = (P.wing_surface_z(T["s"], T["y_bolt"], "lower") or (zc - 0.02)) - 0.004
        f.holes.append(prim_hole(Bp(T["s"], T["y_bolt"], z_lo), pb, hole_d(0.004, "bolt"), name="tab_bolt"))
        f.structure.append(prim_frustum(Bp(T["s"], T["y_bolt"], z_lo), pb, 0.0052, n=32, name="tab_boss"))
        f.notes.append("dış panel tutma dili cebi + alttan M4 naylon cıvata")
    else:
        f.notes.append("dış panel tutma dili cebi (dil yapıştırılır)")
    return f


def wing_servo_center(name: str) -> np.ndarray:
    """Kanat servo yuvası merkezi (Blender): ana kiriş kovanı ile arka kiriş arası, orta kalınlık."""
    ys = SERVO_STATIONS[name]
    m0, m1 = _spar_pieces()[1 if ys < 1.1 else 2][3:5]
    r0, r1 = _rear_line()
    sm = -_line_at(m0, m1, ys)[0] + 0.5 * (0.027 if ys < 1.1 else 0.016) + SLEEVE_T + 0.003
    sr = -_line_at(r0, r1, ys)[0] - 0.004 - 0.003
    sc = 0.5 * (sm + sr)
    zc = P.wing_mid_z(sc, ys) or 0.0
    return Bp(sc, ys, zc)


def mpx_center() -> np.ndarray:
    zc = P.wing_mid_z(MPX["s"], 0.355)
    return Bp(MPX["s"], 0.36, float(zc if zc is not None else -0.0165))


def wing_conduit_polyline() -> list[np.ndarray]:
    """Dış panel kablo kanalı: kanatçık servo yuvası → dış flap servo yuvası → kök kaburgasındaki MPX cebi."""
    a = wing_servo_center("Aileron")
    b = wing_servo_center("FlapOut")
    m = mpx_center()
    return [a, b, m + np.array([0.0, 0.002, 0.0])]


def root_conduit_polyline() -> list[np.ndarray]:
    """Kök blokları kanalı (Ø6): MPX cebinin arkası → yuva tavanı üstünden (takım yatağı kolunun deliğinden) →
    kuyu önünden gövde yan duvarına; gövde içine açılır."""
    m = mpx_center()
    zr = -0.0130
    z_in = P.wing_mid_z(1.268, 0.12) or -0.031
    return [m + np.array([0.0, -0.003, 0.0]), Bp(1.290, 0.345, zr), Bp(1.276, 0.300, zr), Bp(1.268, 0.235, zr),
            Bp(1.268, 0.105, float(z_in)), Bp(1.268, 0.060, float(z_in))]


def conduit_holes(points: Sequence[np.ndarray], d: float, name: str = "conduit") -> list:
    """Kanal: ardışık noktalar arası silindirler (kıvrımlarda küre yerine 1 mm bindirme)."""
    out = []
    for p0, p1 in zip(points[:-1], points[1:]):
        dd = _unit(p1 - p0)
        out.append(prim_frustum(p0 - dd * 0.001, p1 + dd * 0.001, 0.5 * d, n=20, name=name))
    return out


def mpx_pocket(part: str) -> Feature:
    """MPX 6 pin cebi (26 × 13 mm, 15 mm derin): dış panel 1 kök kaburgasında ve kök bloğu 2'de karşılıklı."""
    c = mpx_center()
    f = Feature()
    sgn = 1.0 if part == "panel" else -1.0
    cc = c + np.array([0.0, sgn * 0.5 * MPX["depth"], 0.0])
    cl = MPX["clear"]
    half = (0.5 * MPX["w"] + cl, 0.5 * MPX["depth"] + 0.0006, 0.5 * MPX["h"] + cl)
    f.holes.append(prim_box(cc, np.eye(3), half, name="mpx"))
    f.structure.append(prim_box(cc, np.eye(3), (half[0] + 0.0012, half[1] + 0.0012, half[2] + 0.0012), name="mpx_wall"))
    f.notes.append("MPX 6 pin cebi")
    return f


def tail_conduit_polyline() -> list[np.ndarray]:
    """Kuyruk kablo kanalı (Ø6): dikey 1 dümen servosu → dikey kökü / stabilize ucu birleşimi → stabilize boyunca
    (kiriş ile arka çubuk arasında, elevatör servo yuvasından geçerek) → koni içi."""
    def s_mid(y):
        st = P.stab_station(y)
        s_sp = st.le_s + float(P.SPEC["tail"]["stab"]["spar"]["offset_from_le_m"]) + 0.006 + SLEEVE_T + 0.002
        s_rr = st.le_s + 0.5 * st.chord - 0.003 - SLEEVE_T - 0.002
        return 0.5 * (s_sp + s_rr), st.z
    s_r, z_r = s_mid(0.030)
    s_t, z_t = s_mid(0.515)
    hv = SERVO_STATIONS["Rudder"]
    st = P.fin_station(hv, SIDE)
    hp = P.hinge_line("Rudder", SIDE)
    fsp = float(P.SPEC["tail"]["fin"]["spar"]["offset_from_le_m"])
    s_a = st.le[0] + fsp + 0.004 + SLEEVE_T + 0.003
    s_b = st.le[0] + 0.65 * st.chord - (hp.nose_radius_in + 0.001 + 0.0025 + 0.004)
    rud = Bp(0.5 * (s_a + s_b), st.le[1], st.le[2])
    return [rud, Bp(s_t, 0.515, z_t), Bp(s_r, 0.030, z_r)]


def flap_joiner_geom() -> dict:
    """İç flap ↔ dış flap bağlayıcı tel: menteşe ekseninden ``r_off`` geride, eksene paralel; y = 0,36 ekinin iki
    yanındaki sabit kaburgalarda yay yarıkları (−3…+33°)."""
    hi = P.hinge_line("FlapIn", SIDE)
    ho = P.hinge_line("FlapOut", SIDE)
    X = _unit(np.asarray(hi.axis_outboard_b))
    F = hi.frame_b()
    Y = np.asarray(F[:, 1])                          # veterde geriye (gövdeye göre)
    Y = _unit(Y - X * float(Y @ X))
    Z = np.cross(X, Y)
    pe = np.asarray(hi.p_out_b)                      # iç flap dış ucu (eksen)
    return {"X": X, "Y": Y, "Z": Z, "p": pe, "L_in": hi.length, "ho": ho}


def flap_joiner_slot() -> Solid:
    """Sabit kaburgalardaki yay yarığı: menteşe ekseni çevresinde r_off ± yarık/2, −3…33° (flap aşağı +)."""
    g = flap_joiner_geom()
    FJ = FLAP_JOINER
    r0, r1 = FJ["r_off"] - 0.5 * FJ["slot_w"], FJ["r_off"] + 0.5 * FJ["slot_w"]
    a0, a1 = (math.radians(v) for v in FJ["deg"])
    n = 14
    angs = np.linspace(a0, a1, n)
    # açı + = firar kenarı aşağı: Y (geriye) → −Z_dünya yönüne döner; yerel (Y, Z) düzleminde (cos, −sin)
    sgn = -1.0 if float(g["Z"][2]) > 0 else 1.0
    pts = [(r1 * math.cos(a), sgn * r1 * math.sin(a)) for a in angs] + \
          [(r0 * math.cos(a), sgn * r0 * math.sin(a)) for a in angs[::-1]]
    poly = np.asarray(pts)
    if 0.5 * np.sum(poly[:, 0] * np.roll(poly[:, 1], -1) - np.roll(poly[:, 0], -1) * poly[:, 1]) < 0:
        poly = poly[::-1]
    return prim_prism(poly, g["p"], g["Y"], g["Z"], g["X"], -0.004, 0.0145, name="fj_slot")


def flap_joiner_socket(which: str) -> Solid:
    """Flap ucunda bağlayıcı tel yuvası (Ø2,3 × 15 mm): ``in`` iç flap dış ucu, ``out`` dış flap iç ucu."""
    g = flap_joiner_geom()
    FJ = FLAP_JOINER
    q = g["p"] + g["Y"] * FJ["r_off"]
    d = hole_d(FJ["d"], "pin")
    if which == "in":
        return prim_hole(q - g["X"] * FJ["socket"], q + g["X"] * 0.003, d, name="fj_sock")
    qo = q + g["X"] * (np.linalg.norm(np.asarray(g["ho"].p_in_b) - g["p"]))
    return prim_hole(qo - g["X"] * 0.003, qo + g["X"] * FJ["socket"], d, name="fj_sock")


# =====================================================================================================
# Aviyonik kapağı (P10): 1,0 mm füme PETG levha kalıp üstünde ısıl biçimlendirilir; basılı PETG çerçeve
# (düz alt yüz, 6 mıknatıs cebi) levhanın altına yapıştırılır; gövdede gömme oturma basamağı + karşı mıknatıslar
# =====================================================================================================
HATCH_SHEET_T = 0.0010                       # füme PETG levha. varsayım
HATCH_FRAME_W = 0.008                        # çerçeve bandı: Ø6,4 mıknatıs cebi + 2 × 0,8 mm duvar
HATCH_OPEN_INSET = 0.0084                    # gövde erişim açıklığı kapak çevresinin bu kadar içinde (basamak 9,9 mm)
HATCH_FRAME_TMIN = 0.0044                    # çerçeve en ince yer (mıknatıs cebi 3,2 + 1,2 mm)
HATCH_SPLIT_S = 0.680                        # kalıp ve çerçeve iki parça (tabla)
HATCH_TRIM = 0.008                           # kalıpta kesim payı
HATCH_DRAFT_DEG = 2.0
HATCH_BUCK_DEPTH = 0.015                     # kalıbın üst yüzün en altından tabana derinliği
MAGNET = {"d": 0.006, "t": 0.003, "pocket_d": 0.0064, "pocket_t": 0.0032}
_HATCH_HW = float(np.abs(np.asarray(P.hatch_outline(), float)[:, 1]).max())      # kapak yarı genişliği (0,055)
HATCH_MAGNETS = [(s, sg * round(_HATCH_HW - 0.0002 - 0.5 * HATCH_FRAME_W, 4))      # çerçeve bandı ortası
                 for s in (0.560, 0.665, 0.800) for sg in (1.0, -1.0)]
LEDGE_T = 0.0020                             # gövde oturma basamağı kalınlığı


def hatch_outline_sy(d: float = 0.0, step: float = 0.002) -> np.ndarray:
    """Kapak plan çokgeni (s, y), ``d`` > 0 içe / < 0 dışa ötelenmiş, ``step`` ile yeniden örneklenmiş."""
    from .. import shapes as S
    base = S.poly_resample(np.asarray(P.hatch_outline(), float), step)
    return S.poly_offset(base, d) if abs(d) > 0 else base


def hatch_outline_inset(d: float, step: float = 0.002) -> np.ndarray:
    """İçe ``d`` ötelenmiş kapak çokgeni; arka testere ucundaki dar açılı köşelerde köşe kaydırmasının ürettiği
    kırlangıç kuyruğu (özgün kenara ``d``'den yakın noktalar) atılır → kendini kesmeyen çokgen (P13)."""
    if d <= 0:
        return hatch_outline_sy(d, step)
    base = hatch_outline_sy(0.0, step)
    Q = hatch_outline_sy(d, step)
    A, AB = base, np.roll(base, -1, axis=0) - base
    L2 = np.maximum((AB ** 2).sum(1), 1e-18)
    QA = Q[:, None, :] - A[None, :, :]
    t = np.clip((QA * AB[None]).sum(2) / L2[None], 0.0, 1.0)
    D = np.linalg.norm(QA - t[..., None] * AB[None], axis=2).min(1)
    return Q[D >= d - 0.0003]


_HATCH_BVH = [None]


def hatch_top_z(s: float, y: float) -> float:
    """Kapak (U_Hatch) dış yüzü z; plan dışında gövde derisi + kenar basamağı (0,6 mm)."""
    from .. import shapes as S
    if _HATCH_BVH[0] is None:
        ob = bpy.data.objects.get("U_Hatch")
        if ob is not None:
            V, F = _mesh_np(ob.data, rest_matrix(ob))
            _HATCH_BVH[0] = BVHTree.FromPolygons(V.tolist(), [list(f) for f in F], epsilon=0.0)
        else:
            _HATCH_BVH[0] = False
    rim = float(S.HATCH_FACET["rim"])
    if _HATCH_BVH[0]:
        hit = _HATCH_BVH[0].ray_cast(Vector(tuple(Bp(s, y, 0.30))), Vector((0.0, 0.0, -1.0)), 1.0)
        if hit[0] is not None:
            return float(hit[0][2])
    return float(S._skin_top(s, y)) + rim


def hatch_frame_plane(half: int) -> tuple[float, float, float]:
    """Çerçeve yarısının düz alt yüzü (tabla yüzü) z = a + b·s + c·y: üst yüze (levha iç yüzü − 0,1 mm) en küçük
    kareler düzlemi, en ince yer ``HATCH_FRAME_TMIN`` olacak kadar aşağı kaydırılmış."""
    outer = hatch_outline_sy(0.0002)
    inner = hatch_outline_inset(HATCH_FRAME_W)
    pts = np.vstack([outer, inner])
    m = pts[:, 0] <= HATCH_SPLIT_S if half == 0 else pts[:, 0] >= HATCH_SPLIT_S
    pts = pts[m]
    zt = np.array([hatch_top_z(s, y) - HATCH_SHEET_T - 0.0001 for s, y in pts])
    A = np.column_stack([np.ones(len(pts)), pts[:, 0], pts[:, 1]])
    coef, *_ = np.linalg.lstsq(A, zt, rcond=None)
    gap = zt - A @ coef
    coef[0] += float(gap.min()) - HATCH_FRAME_TMIN
    return float(coef[0]), float(coef[1]), float(coef[2])


def _plane_z(coef, s, y) -> float:
    return coef[0] + coef[1] * s + coef[2] * y


def _poly_prism_sy(poly_sy: np.ndarray, z_bot: Callable, z_top: Callable, name: str) -> Solid:
    """(s, y) çokgeni boyunca dik prizma; köşe başına alt/üst z (alt ve üst n-gen düzlemsel olmalı ya da yakın)."""
    poly = np.asarray(poly_sy, float)
    if 0.5 * np.sum(poly[:, 0] * np.roll(poly[:, 1], -1) - np.roll(poly[:, 0], -1) * poly[:, 1]) > 0:
        poly = poly[::-1]                            # Blender XY'de (X = −s) saat yönü tersi olsun
    n = len(poly)
    V = [Bp(s, y, z_bot(s, y)) for s, y in poly] + [Bp(s, y, z_top(s, y)) for s, y in poly]
    F = [tuple(range(n - 1, -1, -1)), tuple(range(n, 2 * n))]
    for i in range(n):
        j = (i + 1) % n
        F.append((i, j, n + j, n + i))
    sol = Solid(np.asarray(V, float), F, name)
    if _signed_volume(sol.V, sol.F) < 0:
        sol = Solid(sol.V, [tuple(reversed(f)) for f in sol.F], name)
    return sol


def _band_strip(outer_sy, inner_sy, z_bot: Callable, z_top: Callable, name: str) -> Solid:
    """İki eş indisli halka (dış/iç, (s, y)) arası bant katısı; üst ve alt yüz köşe başına z."""
    o, i_ = np.asarray(outer_sy, float), np.asarray(inner_sy, float)
    n = len(o)
    V = [Bp(s, y, z_top(s, y)) for s, y in o] + [Bp(s, y, z_top(s, y)) for s, y in i_] + \
        [Bp(s, y, z_bot(s, y)) for s, y in o] + [Bp(s, y, z_bot(s, y)) for s, y in i_]
    OT, IT, OB, IB = 0, n, 2 * n, 3 * n
    F = []
    for k in range(n):
        j = (k + 1) % n
        F.append((OT + k, OT + j, IT + j, IT + k))          # üst
        F.append((OB + k, IB + k, IB + j, OB + j))          # alt
        F.append((OB + k, OB + j, OT + j, OT + k))          # dış duvar
        F.append((IB + k, IT + k, IT + j, IB + j))          # iç duvar
    sol = Solid(np.asarray(V, float), F, name)
    if _signed_volume(sol.V, sol.F) < 0:
        sol = Solid(sol.V, [tuple(reversed(f)) for f in sol.F], name)
    return sol


def _half_box(half: int, gap: float = 0.0002, name: str = "half") -> Solid:
    """Kapak ikiye bölme yarı-uzayı (s ≤ / ≥ ``HATCH_SPLIT_S``)."""
    s0, s1 = (0.40, HATCH_SPLIT_S - gap) if half == 0 else (HATCH_SPLIT_S + gap, 1.00)
    return prim_box(Bp(0.5 * (s0 + s1), 0.0, 0.10), np.eye(3), (0.5 * (s1 - s0), 0.12, 0.15), name=name)


def hatch_frame_parts(half: int) -> dict:
    """``hatch_frame_a/b`` (PETG): levhanın altına yapıştırılan 8 mm bant; üst yüz levha iç yüzünü izler, alt yüz
    düzlem (tabla yüzü = gövde oturma basamağına oturur); 3 × Ø6,4 × 3,2 mıknatıs cebi (alttan).
    Kurulum boolean ile: yükseklik alanı (üst yüz) ∩ dış çevre prizması ∩ alt düzlem yarı-uzayı − iç çevre (temiz
    iç öteleme; testere uçlarında kendini kesen bant şeridi yok)."""
    outer = hatch_outline_sy(0.0002)
    inner = hatch_outline_inset(HATCH_FRAME_W)
    coef = hatch_frame_plane(half)
    s0, s1 = float(outer[:, 0].min()) - 0.003, float(outer[:, 0].max()) + 0.003
    y0, y1 = float(outer[:, 1].min()) - 0.003, float(outer[:, 1].max()) + 0.003
    zb = min(_plane_z(coef, s, y) for s in (s0, s1) for y in (y0, y1)) - 0.004
    base = _ceiling_solid(s0, s1, y0, y1, lambda s, y: hatch_top_z(s, y) - HATCH_SHEET_T - 0.0001, zb,
                          n=45, ns=181, name="hframe_hf")
    side = _poly_prism_sy(outer, lambda s, y: zb - 0.004, lambda s, y: 0.25, "hframe_out")
    a, b, c = coef
    plane = region_solid([(np.array([0.0, 0.0, a]), _unit(np.array([-b, c, -1.0])))], Bp(0.68, 0.0, 0.20))
    holes = [_poly_prism_sy(inner, lambda s, y: zb - 0.01, lambda s, y: 0.26, "hframe_in")]
    for s, y in HATCH_MAGNETS:
        if (s <= HATCH_SPLIT_S) == (half == 0):
            zm = _plane_z(coef, s, y)
            holes.append(prim_hole(Bp(s, y, zm - 0.001), Bp(s, y, zm + MAGNET["pocket_t"]), MAGNET["pocket_d"],
                                   name="magnet"))
    return {"base": base, "adds": [], "holes": holes, "clip": [side, plane, _half_box(half)]}


def hatch_buck_parts(half: int) -> dict:
    """``hatch_buck_a/b`` (PLA kalıp): üst yüz = kapak dış yüzü − levha kalınlığı (kesim payı 8 mm), 2° eğimli yanlar,
    düz taban; ikiye bölünmüş, Ø3 CF hizalama pimi delikleri (bölme yüzünde)."""
    poly_top = hatch_outline_sy(-HATCH_TRIM, 0.004)
    z_top_fn = lambda s, y: hatch_top_z(s, y) - HATCH_SHEET_T
    zs = [z_top_fn(s, y) for s, y in poly_top]
    z_base = min(zs) - HATCH_BUCK_DEPTH
    dd = HATCH_BUCK_DEPTH * math.tan(math.radians(HATCH_DRAFT_DEG)) + 0.0006
    poly_bot = hatch_outline_sy(-HATCH_TRIM - dd, 0.004)
    # yükseklik alanı (üst yüz) × eğimli çevre prizması
    s0, s1 = float(poly_bot[:, 0].min()) - 0.002, float(poly_bot[:, 0].max()) + 0.002
    y0, y1 = float(poly_bot[:, 1].min()) - 0.002, float(poly_bot[:, 1].max()) + 0.002
    hf = _ceiling_solid(s0, s1, y0, y1, z_top_fn, z_base, n=25, ns=61, name="buck_hf")
    side = _band_like_frustum(poly_bot, poly_top, z_base - 0.001, max(zs) + 0.01)
    holes = []
    for y in (-0.030, 0.030):
        zc = z_base + 0.008
        for sg in (-1.0, 1.0):
            holes.append(prim_hole(Bp(HATCH_SPLIT_S - sg * 0.0005, y, zc), Bp(HATCH_SPLIT_S + sg * 0.011, y, zc),
                                   hole_d(PIN_D, "pin", "PETG"), name="buck_pin"))
    return {"base": hf, "adds": [], "holes": holes, "clip": [side, _half_box(half, 0.0)]}


def _band_like_frustum(poly_bot, poly_top, z0, z1) -> Solid:
    """Alt çokgen (z0) → üst çokgen (z1) eğimli prizma (eş indisli çokgenler)."""
    a, b = np.asarray(poly_bot, float), np.asarray(poly_top, float)
    if 0.5 * np.sum(a[:, 0] * np.roll(a[:, 1], -1) - np.roll(a[:, 0], -1) * a[:, 1]) > 0:
        a, b = a[::-1], b[::-1]
    n = len(a)
    V = [Bp(s, y, z0) for s, y in a] + [Bp(s, y, z1) for s, y in b]
    F = [tuple(range(n - 1, -1, -1)), tuple(range(n, 2 * n))]
    for i in range(n):
        j = (i + 1) % n
        F.append((i, j, n + j, n + i))
    sol = Solid(np.asarray(V, float), F, "draft")
    if _signed_volume(sol.V, sol.F) < 0:
        sol = Solid(sol.V, [tuple(reversed(f)) for f in sol.F], "draft")
    return sol


def hatch_seat_ledge_parts(half: int) -> dict:
    """Gövdede oturma basamağı (yarım): kapak çevresinin 1,5 mm dışından 8,4 mm içine, çerçeve alt düzleminin
    0,2 mm altından ``LEDGE_T`` kalınlıkta + basamak duvarı (gövde dış yüzüne kırpılır)."""
    coef = hatch_frame_plane(half)
    seat = lambda s, y: _plane_z(coef, s, y) - 0.0002
    poly = hatch_outline_sy(-0.0015)
    blk = _poly_prism_sy(poly, lambda s, y: seat(s, y) - LEDGE_T, lambda s, y: 0.25, "ledge")
    adds = []
    for s, y in HATCH_MAGNETS:                          # mıknatıs göbekleri (basamağın altında)
        if (s <= HATCH_SPLIT_S) == (half == 0):
            z = seat(s, y)
            adds.append(prim_frustum(Bp(s, y, z - MAGNET["pocket_t"] - 0.0012), Bp(s, y, z - 0.0005), 0.0052, n=32,
                                     name="mag_boss"))
    from .. import shapes as S
    fus = S.fuselage()
    clip_fus = Solid(np.asarray(fus.verts_blender(), float), [tuple(f) for f in fus.faces], "fus")
    return {"base": blk, "adds": adds, "holes": [], "clip": [clip_fus, _half_box(half, 0.0)]}


def hatch_seat_features(ctx: Ctx, s0: float, s1: float) -> Feature:
    """Gövde halkasında kapak oturma yeri: basamak (eklenir, bölgeye kırpılır), basamak üstü boşaltma (kapak
    çevresi + 0,3 mm, çerçeve alt düzleminden yukarı), erişim açıklığı (çevrenin 8,4 mm içi), mıknatıs cepleri."""
    f = Feature()
    for half in (0, 1):
        hs0, hs1 = (0.49, HATCH_SPLIT_S) if half == 0 else (HATCH_SPLIT_S, 0.87)
        if s1 < hs0 or s0 > hs1:
            continue
        led = ctx.custom_solid(f"hatch_ledge_{half}", lambda half=half: hatch_seat_ledge_parts(half))
        if led is not None:
            f.adds.append(led)
        coef = hatch_frame_plane(half)
        seat = lambda s, y, coef=coef: _plane_z(coef, s, y) - 0.0002
        rec = ctx.custom_solid(f"hatch_recess_{half}", lambda half=half, seat=seat: {
            "base": _poly_prism_sy(hatch_outline_sy(-0.0003), seat, lambda s, y: 0.25, "recess"),
            "adds": [], "holes": [], "clip": [_half_box(half, 0.0)]})
        if rec is not None:
            f.holes.append(rec)
        for s, y in HATCH_MAGNETS:
            if (s <= HATCH_SPLIT_S) == (half == 0) and s0 - 0.01 < s < s1 + 0.01:
                z = seat(s, y)
                f.holes.append(prim_hole(Bp(s, y, z - MAGNET["pocket_t"]), Bp(s, y, z + 0.001), MAGNET["pocket_d"],
                                         name="magnet"))
    f.holes.append(_poly_prism_sy(hatch_outline_inset(HATCH_OPEN_INSET), lambda s, y: 0.0, lambda s, y: 0.25,
                                  "hatch_open"))
    f.notes.append("kapak oturma basamağı + 3 mıknatıs cebi (yarım başına)")
    return f


def build_custom(ctx: Ctx, seg: SegSpec, out_col: bpy.types.Collection, parts: dict) -> Built | None:
    """Özel parça: base ∪ adds, ∩ clip(ler), − holes (MANIFOLD) → kapalı katı; ``UP_<anahtar>``."""
    B = ctx.B
    A = ctx.obj("cu", parts["base"])
    col = B.group(ctx.name("cadd"), parts.get("adds", []))
    if col is not None:
        B.op(A, col, "UNION")
        Booler.drop(col)
    for cl in parts.get("clip", []) or []:
        co = ctx.obj("cclip", cl)
        B.op(A, co, "INTERSECT")
        _delete(co)
    col = B.group(ctx.name("chole"), parts.get("holes", []))
    if col is not None:
        B.op(A, col, "DIFFERENCE")
        Booler.drop(col)
    for k, pad in parts.get("subtract_sources", []) or []:          # komşu basılı parça (+ yapıştırma aralığı)
        if ctx.has(k):
            B.safe(A, ctx.grown(k, pad), "DIFFERENCE", ctx.cut_solver, B.solver, min_volume=1e-9)
    cl = finalize_mesh(A)
    if len(A.data.polygons) == 0:
        _delete(A)
        return None
    ctx.work.objects.unlink(A)
    out_col.objects.link(A)
    A.name = f"UP_{seg.key}"
    A.data.name = A.name
    chk = mesh_check(A.data)
    chk["removed_islands"] = cl.get("removed_islands", 0)
    return Built(seg, A, chk, {"notes": list(parts.get("notes", []))})


# =====================================================================================================
# Özel parçalar
# =====================================================================================================
NOSE_FLANGE_T = 0.003                               # PETG flanş kalınlığı (s yönünde)


def nose_flange_insets() -> tuple[float, float]:
    """PETG flanşın dış/iç yüzlerinin gövde dış yüzünden içe uzaklığı (m): dış = max(et + 0,6, LAND_T) + 0,2 mm,
    iç = et + 10 mm (et: burun bölgesi derisi)."""
    w = _wall_of("fuselage_nose")
    return max(w + 0.0006, LAND_T) + 0.0002, w + 0.010


def nose_flange_pocket() -> Solid:
    """Modül halkasının arka (s = 0,40) çerçevesinde flanş cebi: flanş hacmi + yapıştırma aralığı (R14 montaj
    denetimi: flanş çerçeveyle 3,7 cm³ çakışıyordu). Flanş çerçevenin arka 3 mm'sinin yerini alır."""
    s = float(P.SPEC["fuselage"]["modules"]["nose_module_s_m"][1])
    a, b = nose_flange_insets()
    return fus_loft(s - NOSE_FLANGE_T - GLUE_GAP, s + 0.001, a - GLUE_GAP, b + GLUE_GAP, n_st=3, nq=48,
                    name="nose_flange_pocket")


def build_nose_flange(ctx: Ctx, seg: SegSpec, out_col: bpy.types.Collection) -> Built | None:
    """Burun modülü PETG flanşı: modül halkasının iç flanşına (land) yapışan 3 mm halka; 4×M4 + 2×Ø4 pim."""
    s = float(P.SPEC["fuselage"]["modules"]["nose_module_s_m"][1])
    t = seg.wall or NOSE_FLANGE_T
    B = ctx.B
    slab_planes = [(Bp(s, 0, 0), np.array([-1.0, 0, 0])), (Bp(s - t, 0, 0), np.array([1.0, 0, 0]))]
    base = ctx.copy(ctx.outer("fus_clean"), "nf")
    reg = ctx.obj("regx", region_solid([(p + n * LOCAL_MARGIN, n) for p, n in slab_planes], Bp(s, 0, 0)))
    B.op(base, reg, "INTERSECT", solver=ctx.cut_solver)
    _delete(reg)
    V, Tt = triangulate(base.data)
    a_out, a_in = nose_flange_insets()
    outer_in = ctx.obj("nf_o", Solid(offset_vertices(V, Tt, np.full(len(V), a_out)), [tuple(x) for x in Tt]))
    # iç kenar: kesit çokgeninden (fus_loft) — ağ köşe kaydırması chine sırtında (eğrilik yarıçapı < 10 mm) kendini
    # kesen kırlangıç kuyruğu bırakıyordu (y ±88 mm'de 0,04–0,24 mm kıymık, R14)
    inner_in = ctx.obj("nf_i", fus_loft(s - t - 0.002, s + 0.002, a_in, n_st=3, nq=24, name="nf_i"))
    _delete(base)
    reg = ctx.obj("reg", region_solid(slab_planes, Bp(s, 0, 0)))
    B.op(outer_in, reg, "INTERSECT")
    B.op(inner_in, reg, "INTERSECT")
    _delete(reg)
    B.op(outer_in, inner_in, "DIFFERENCE")
    _delete(inner_in)
    bolts, pins = nose_flange_bolts()
    holes = [prim_hole(Bp(s - t - 0.003, y, z), Bp(s + 0.003, y, z), M4_HOLE_D) for y, z in bolts]
    holes += [prim_hole(Bp(s - t - 0.003, y, z), Bp(s + 0.003, y, z), hole_d(NOSE_PIN_D, "pin", "PETG"))
              for y, z in pins]
    col = B.group(ctx.name("hole"), holes)
    B.op(outer_in, col, "DIFFERENCE")
    Booler.drop(col)
    finalize_mesh(outer_in)
    ctx.work.objects.unlink(outer_in)
    out_col.objects.link(outer_in)
    outer_in.name = f"UP_{seg.key}"
    outer_in.data.name = outer_in.name
    return Built(seg, outer_in, mesh_check(outer_in.data))


def build_scuff_pad(ctx: Ctx, seg: SegSpec, out_col: bpy.types.Collection) -> Built | None:
    """Kaporta altı sürtünme pabucu (montaj denetimi): sahnedeki pabuç kaporta derisinin içine ≈ 1 mm gömülüdür
    (kaporta ile 0,7 cm³ çakışma; kaportayı çıkarınca 0,1–0,3 mm kama kalıyordu). Baskı pabucu: sahne pabucunun plan
    izi (dışbükey zarf) ∩ (kaporta + aralık + t) − (kaporta + aralık) → kaporta dış yüzünü izleyen ``t`` kalınlıkta
    katman; üst yüz kaportaya yapışır, alt yüz aşınır."""
    B = ctx.B
    t = float(seg.wall or 0.002)
    src = ctx.solid(seg.sources[0])
    hull = _hull2(np.asarray(src.V, float)[:, :2])
    if _poly_area2(hull) < 0:
        hull = hull[::-1]
    z0, z1 = float(np.min(src.V[:, 2])) - 0.03, float(np.max(src.V[:, 2])) + 0.03
    A = ctx.obj("scuff", prim_prism(hull, np.zeros(3), np.array([1.0, 0, 0]), np.array([0, 1.0, 0]),
                                    np.array([0, 0, 1.0]), z0, z1, name="scuff_fp"))
    B.op(A, ctx.grown("cowl", GLUE_GAP + t), "INTERSECT")
    B.op(A, ctx.grown("cowl", GLUE_GAP), "DIFFERENCE")
    cl = finalize_mesh(A)
    if len(A.data.polygons) == 0:
        _delete(A)
        return None
    ctx.work.objects.unlink(A)
    out_col.objects.link(A)
    A.name = f"UP_{seg.key}"
    A.data.name = A.name
    chk = mesh_check(A.data)
    chk["removed_islands"] = cl.get("removed_islands", 0)
    return Built(seg, A, chk, {"notes": [f"kaporta yüzünü izleyen {t * 1000:.0f} mm katman"]})


def build_plain(ctx: Ctx, seg: SegSpec, out_col: bpy.types.Collection) -> Built | None:
    """Kabuksuz parça (kapak, ince panel, sürtünme pabucu): kaynak ∩ bölge − kesikler."""
    A = ctx.copy(ctx.outer(seg.sources[0]), "A")
    if seg.ends:
        center = seg.center if seg.center is not None else np.mean([e.p for e in seg.ends], axis=0)
        reg = ctx.obj("reg", region_solid(_planes(seg.ends), center))
        ctx.B.op(A, reg, "INTERSECT", solver=ctx.cut_solver)
        _delete(reg)
    for cut in seg.cutouts:                             # R14: kıymık şeritleri (kapak çentik köşesi) vb.
        co = ctx.obj("cut", cut)
        ctx.B.safe(A, co, "DIFFERENCE", ctx.cut_solver, "MANIFOLD", min_volume=1e-9)
        _delete(co)
    for k, pad, _ in seg.subtract:                      # komşu parça (+ yapıştırma aralığı): sürtünme pabucu ↔ kaporta
        if ctx.has(k):
            ctx.B.safe(A, ctx.grown(k, pad), "DIFFERENCE", ctx.cut_solver, ctx.B.solver, min_volume=1e-9)
    if len(A.data.polygons) == 0:
        _delete(A)
        return None
    cl = finalize_mesh(A)
    ctx.work.objects.unlink(A)
    out_col.objects.link(A)
    A.name = f"UP_{seg.key}"
    A.data.name = A.name
    chk = mesh_check(A.data)
    chk["removed_islands"] = cl["removed_islands"]
    notes = []
    drop = getattr(ctx, "merge_notes", {}).get(seg.sources[0])
    if drop:
        notes.append(f"gövdeye değmeyen {len(drop)} donanım bileşeni basılmadı ({', '.join(f'{v:.2f}' for v in drop)} "
                     "cm³) — ayrı takılır")
    return Built(seg, A, chk, {"notes": notes})


# =====================================================================================================
# Değerlendirme, bölme, ayna, dışa aktarım
# =====================================================================================================
def _axis_words(u: np.ndarray) -> str:
    """Blender dünya yönünü uçak eksenleriyle anlatır (ör. "+y (sol/dış)")."""
    names = [("+x (burun)", "−x (kuyruk)"), ("+y (sol/iskele)", "−y (sağ/sancak)"), ("+z (yukarı)", "−z (aşağı)")]
    i = int(np.argmax(np.abs(u)))
    tilt = math.degrees(math.acos(min(1.0, abs(float(u[i])))))
    w = names[i][0 if u[i] > 0 else 1]
    return w if tilt < 1.0 else f"{w} ({tilt:.0f}° eğik)"


GROOVE = tuple(float(v) / 1000.0 for v in _PR.get("joint_groove_mm", [0.4, 0.3]))   # (eksen boyunca, içe) m
GROOVE_MIN_T = 0.0012                               # pah yalnız içe et ≥ 1,2 mm olan köşede (ince firar kenarı atlanır)


def joint_grooves(ctx: "Ctx", seg: SegSpec, ob: bpy.types.Object) -> int:
    """P13: segment uçlarında (plate/land/frame) dış deri kenarına ``GROOVE`` pahı — iki komşu parça birleşince
    V-oluk (panel çizgisi). bmesh: uç düzleminden ``w`` içeride kesit halkası eklenir; düzlemdeki dış deri köşeleri
    (kaynağın dış yüzeyinde ≤ 10 µm) düzlem içinde ``d`` kadar içe kaydırılır (topoloji değişmez). İçe et
    ``GROOVE_MIN_T``'den azsa (firar kenarı, deliğe yakın) köşe atlanır. Ağ denetimi geçmezse geri alınır → 0."""
    ends = [e for e in seg.ends if e.kind in ("plate", "land", "frame")]
    if not ends or not seg.sources:
        return 0
    w, d = GROOVE
    cache = ctx.__dict__.setdefault("_src_bvh", {})
    key = seg.sources[0]
    if key not in cache:
        Ss = ctx.solid(key)
        cache[key] = BVHTree.FromPolygons(Ss.V.tolist(), [list(f) for f in Ss.F], epsilon=0.0)
    bvh_src = cache[key]
    me = ob.data
    backup = me.copy()
    v0 = mesh_check(me)["volume_m3"]
    bm = bmesh.new()
    bm.from_mesh(me)
    M = ob.matrix_world
    Mi = M.inverted()
    moved = 0
    for e in ends:
        pw, nw = Vector(tuple(e.p)), Vector(tuple(e.n))          # dünya; n segment dışına
        p = Mi @ pw
        n = (Mi.to_3x3() @ nw).normalized()
        bmesh.ops.bisect_plane(bm, geom=bm.verts[:] + bm.edges[:] + bm.faces[:], plane_co=p - n * w, plane_no=n,
                               dist=1e-7)
        bm.verts.ensure_lookup_table()
        bm.faces.ensure_lookup_table()
        bvh_part = BVHTree.FromBMesh(bm)
        cand = []
        for v in bm.verts:
            if abs((v.co - p).dot(n)) > 2e-6:
                continue
            q = bvh_src.find_nearest(M @ v.co)
            if q[0] is None or q[3] > 1e-5:
                continue
            m = (Mi.to_3x3() @ q[1])
            m = m - n * m.dot(n)
            if m.length < 0.3:                                    # yüzey uç düzlemine neredeyse paralel
                continue
            inward = -m.normalized()
            h = bvh_part.ray_cast(v.co + inward * 2e-6 - n * 1e-5, inward, 0.01)
            if h[0] is None or h[3] < GROOVE_MIN_T:
                continue
            cand.append((v, inward))
        for v, inward in cand:
            v.co = v.co + inward * d
        moved += len(cand)
    bm.to_mesh(me)
    bm.free()
    me.update()
    chk = mesh_check(me)
    if not moved or not chk["ok"] or chk["volume_m3"] > v0 + 1e-10 or chk["volume_m3"] < 0.98 * v0:
        bad = ob.data
        ob.data = backup
        bpy.data.meshes.remove(bad)
        if moved:
            ctx.log(f"{seg.key}: ek pahı geri alındı (ağ denetimi)")
        return 0
    bpy.data.meshes.remove(backup)
    return moved


FOOT_T = 0.0010                                     # kırılabilir baskı ayağı kalınlığı (tabla teması yetmezse)
FOOT_GROW = 0.005                                   # ayak, alt izin 5 mm dışına taşar
TRUNC_MAX = 0.0004                                  # eğri alt yüzde düzleme kesimi en çok 0,4 mm


def _bed_mesh_op(ctx: "Ctx", W: np.ndarray, T: np.ndarray, tool: Solid, op: str) -> tuple[np.ndarray, np.ndarray] | None:
    """Tabla koordinatında (W, T) ağına MANIFOLD boolean uygular → (W, T) ya da None (kapalı değilse)."""
    ob = ctx.obj("bed", Solid(W, [tuple(t) for t in T]))
    to = ctx.obj("bedt", tool)
    try:
        ctx.B.op(ob, to, op)
        finalize_mesh(ob)
        if not mesh_check(ob.data)["ok"]:
            return None
        V2, T2 = triangulate(ob.data)
        return V2, T2
    finally:
        _delete(to)
        _delete(ob)


def bed_fix(ctx: "Ctx", V: np.ndarray, T: np.ndarray, pl: dict) -> dict | None:
    """Tabla teması yetersizse (P6) yalnız STL için: (1) eğri tabanı ≤ 0,4 mm düzleme keser (düz temas yaması),
    (2) olmazsa altına 1 mm kırılabilir ayak ekler (alt iz + 5 mm). Dönüş: {"W", "T", "contact_cm2", "note"}."""
    M = pl["M"]
    W = V @ M[:3, :3].T + M[:3, 3]
    need = pl["contact_need_cm2"]
    span = W.max(0) - W.min(0)
    c = 0.5 * (W.max(0) + W.min(0))
    for d in (0.0001, 0.0002, 0.0003, TRUNC_MAX):
        box = prim_box((c[0], c[1], d - 0.5), np.eye(3), (span[0] + 0.05, span[1] + 0.05, 0.5), name="trunc")
        r = _bed_mesh_op(ctx, W, T, box, "DIFFERENCE")
        if r is None:
            continue
        W2, T2 = r
        W2 = W2 - np.array([0.0, 0.0, W2[:, 2].min()])
        pc = placement(W2, T2, (0.0, 0.0, 1.0), ctx.bed)
        if pc["bed_contact_cm2"] >= need:
            return {"W": W2, "T": T2, "contact_cm2": pc["bed_contact_cm2"],
                    "note": f"tabla teması için alt yüz {d * 1000:.1f} mm düzlendi"}
    low = W[W[:, 2] < W[:, 2].min() + 0.0015][:, :2]
    if len(low) < 3:
        low = W[np.argsort(W[:, 2])[:16], :2]
    hull = _hull2(low)
    if len(hull) < 3:
        cxy = low.mean(0)
        hull = np.array([cxy + [-0.002, -0.002], cxy + [0.002, -0.002], cxy + [0.002, 0.002], cxy + [-0.002, 0.002]])
    ring = []
    cen = hull.mean(0)
    for k in range(len(hull)):                          # dışbükey izi FOOT_GROW kadar büyüt (köşe açıortayı)
        a_, b_, c_ = hull[k - 1], hull[k], hull[(k + 1) % len(hull)]
        e1, e2 = _unit(b_ - a_), _unit(c_ - b_)
        n1, n2 = np.array([e1[1], -e1[0]]), np.array([e2[1], -e2[0]])
        if (b_ - cen) @ n1 < 0:
            n1, n2 = -n1, -n2
        bis = _unit(n1 + n2)
        ring.append(b_ + bis * FOOT_GROW / max(0.3, float(bis @ n1)))
    ring = np.asarray(ring)
    if 0.5 * np.sum(ring[:, 0] * np.roll(ring[:, 1], -1) - np.roll(ring[:, 0], -1) * ring[:, 1]) < 0:
        ring = ring[::-1]
    lift = FOOT_T - 0.0002
    W1 = W + np.array([0.0, 0.0, lift])
    foot = prim_prism(ring, np.zeros(3), np.array([1.0, 0, 0]), np.array([0, 1.0, 0]), np.array([0, 0, 1.0]),
                      0.0, FOOT_T, name="foot")
    r = _bed_mesh_op(ctx, W1, T, foot, "UNION")
    if r is None:
        return None
    W2, T2 = r
    W2 = W2 - np.array([0.0, 0.0, W2[:, 2].min()])
    pc = placement(W2, T2, (0.0, 0.0, 1.0), ctx.bed)
    return {"W": W2, "T": T2, "contact_cm2": pc["bed_contact_cm2"],
            "note": f"{FOOT_T * 1000:.0f} mm kırılabilir baskı ayağı (baskıdan sonra kesilir)"}


# Kütle modeli (R04): kalın PA-CF / PETG bölgeleri dilimleyici tablosundaki gibi basılır — yüzeyden "çevre kabuğu"
# derinliğine kadar dolu (PA-CF 3 × 0,53 ≈ 1,6 mm; PETG 3 × 0,4 = 1,2 mm), daha derini %40 gyroid. Kabuğun altındaki
# hacim oranı (``core``) iç noktalardan yüzeye uzaklıkla ölçülür; kütle = ρ·V·(1 − core·(1 − dolgu)). LW-PLA/LW-ASA
# ince duvarlı tasarlandığı için dolu sayılır (tutucu).
SLICER_FILL = {"PA-CF": (0.0016, 0.40), "PETG": (0.0012, 0.40)}


def _fill_key(material: str) -> str:
    return "PETG" if material.upper().startswith("PETG") else material.upper()


def fill_factor(material: str, core: float) -> float:
    """Katı hacme göre basılan malzeme oranı (``SLICER_FILL``; tablo dışı malzemeler 1)."""
    k = _fill_key(material)
    if k not in SLICER_FILL:
        return 1.0
    return float(1.0 - core * (1.0 - SLICER_FILL[k][1]))


def core_fraction(V: np.ndarray, T: np.ndarray, shell: float, n: int = 1500, seed: int = 11) -> float:
    """Katının yüzeyden ``shell``'den derin hacim oranı: kutu içinde rastgele noktalar, +Z ışın paritesiyle içeride
    olanlar (``n`` adede dek), en yakın yüzey uzaklığı > ``shell`` oranı."""
    if len(T) == 0:
        return 0.0
    a_, b_, c_ = V[T[:, 0]], V[T[:, 1]], V[T[:, 2]]
    area = 0.5 * float(np.linalg.norm(np.cross(b_ - a_, c_ - a_), axis=1).sum())
    vol = abs(float(np.einsum("ij,ij->i", a_, np.cross(b_, c_)).sum()) / 6.0)
    if 2.0 * vol / max(area, 1e-12) <= 1.1 * shell:     # ortalama et ≤ kabuk: çekirdek yok (ince duvarlı parça)
        return 0.0
    bvh = BVHTree.FromPolygons(V.tolist(), T.tolist(), epsilon=0.0)
    lo, hi = V.min(0), V.max(0)
    rng = np.random.default_rng(seed)
    dz = Vector((0.0, 0.0, 1.0))
    inside, tries = [], 0
    while len(inside) < n and tries < 40 * n:
        q = lo + rng.random(3) * (hi - lo)
        tries += 1
        o, cnt = Vector(tuple(q)), 0
        for _ in range(64):
            h = bvh.ray_cast(o, dz, 10.0)
            if h[0] is None:
                break
            cnt += 1
            o = h[0] + dz * 1e-7
        if cnt % 2 == 1:
            inside.append(q)
    if not inside:
        return 0.0
    d = np.array([bvh.find_nearest(Vector(tuple(q)))[3] for q in inside])
    return float((d > shell).mean())


def evaluate(b: Built, bed, ctx: "Ctx | None" = None) -> dict:
    """Baskı yönü (P6: tabla teması sert koşul), tabla sığması, 45° çıkıntı ve destek sınıfı (aşağı ışın), hacim /
    kütle / süre, et ölçümü. Tabla teması yetmiyorsa STL'ye düzleme ya da kırılabilir ayak (``bed_fix``)."""
    seg = b.seg
    V, T = triangulate(b.ob.data)
    pref = []
    if seg.bed_end is not None and seg.bed_end < len(seg.ends):
        pref.append(-seg.ends[seg.bed_end].n)
    if seg.up_hint is not None:
        pref.append(np.asarray(seg.up_hint, float))
    pl = choose_orientation(V, T, bed, pref, force=seg.force_up)
    if seg.force_up and pref:                           # dik baskı zorunlu: tercih edilen yön sığmıyorsa bölünecek
        pp = placement(V, T, pref[0], bed)
        if not (pp["fits"] or pp["fits_tight"]):
            pp["fits"] = pp["fits_tight"] = False
            pl = pp
    mat = seg.material or _zone(seg.zone)[0]
    wall = seg.wall if seg.wall is not None else _zone(seg.zone)[1]
    vol_cm3 = b.check["volume_m3"] * 1e6
    core = core_fraction(V, T, SLICER_FILL[_fill_key(mat)][0]) if _fill_key(mat) in SLICER_FILL else 0.0
    fill = TOOLING_INFILL if seg.tooling else fill_factor(mat, core)
    rate = PRINT_RATE["PETG" if mat.startswith("PETG") else mat]
    layers = pl["size_mm"][2] / rate["layer_mm"]
    hours = fill * vol_cm3 / rate["cm3_h"] + layers * LAYER_OVERHEAD_S / 3600.0 + PLATE_SETUP_H
    out = {"placement": pl, "V": V, "T": T, "material": mat, "wall_mm": wall * 1000.0, "volume_cm3": vol_cm3,
           "mass_g": fill * vol_cm3 * density(mat), "hours": hours, "fill": fill, "core": core}
    if (pl["fits"] or pl["fits_tight"]) and not pl["contact_ok"] and ctx is not None:
        fx = bed_fix(ctx, V, T, pl)
        if fx is not None:
            out["bed_fix"] = fx
            pl["bed_contact_cm2"] = fx["contact_cm2"]
            pl["contact_ok"] = fx["contact_cm2"] >= pl["contact_need_cm2"]
            pl["size_mm"] = (pl["size_mm"][0], pl["size_mm"][1], float((fx["W"][:, 2].max()) * 1000.0))
    if "bed_fix" in out:
        Wb, Tb = out["bed_fix"]["W"], out["bed_fix"]["T"]
        out.update(support_class(Wb, Tb, np.eye(4)))
    else:
        out.update(support_class(V, T, pl["M"]))
    out["min_wall"] = min_wall_mm(b.ob, n_samples=1200)
    return out


# ---------------------------------------------------------------- ısı kuralı (AERO-09)
HEAT_RULE = dict(_PR.get("heat_rule", {"radius_m": 0.150, "no_material": "LW-PLA", "around": ["cylinder", "muffler"]}))
HEAT_SAFE_MATERIAL = "LW-ASA"                       # Tg 95 °C; LW-PLA (Tg 55 °C) yerine ısı bölgesinde (öneri)
# R05 incelemesi: Tg < 120 °C olan hiçbir parça susturucu/silindir zarfına (ve susturucu çıkış borusu/kalkanına)
# 50 mm'den yakın olamaz. Spec'te ``print.heat_rule.tg_min_c`` / ``tg_radius_m`` verilirse onlar kullanılır.
HEAT_TG_RULE = {"tg_min_c": float(HEAT_RULE.get("tg_min_c", 120.0)),
                "radius_m": float(HEAT_RULE.get("tg_radius_m", 0.050))}
# Kural 3 (son inceleme): hiçbir basılı parça susturucu çıkış borusuna ``pipe_clear_m``'den (5 mm) yakın değil;
# boru yanaktan ısıya dayanıklı geçiş halkasıyla geçer (BOM).
HEAT_PIPE_CLEAR_M = float(HEAT_RULE.get("pipe_clear_m", 0.005))
# spec ``print.zones[].parts``: adıyla listelenen parçalar o bölgenin filamentiyle basılır (ör. stab_root_heat →
# stab_1, elevator_1 LW-ASA). Isı kuralı artık malzemeyi kendisi değiştirmez: ihlal = çalıştırma hatası.
_PART_ZONES = {str(pk): str(z["zone"]) for z in _PR["zones"] for pk in (z.get("parts") or [])}


def tg_c(material: str) -> float:
    """Filament ısıl sınırı (°C; spec ``print.filaments.*.tg_c``): amorf filamentlerde (LW-PLA, PETG, LW-ASA) camsı
    geçiş Tg; yarı kristal PA-CF'de ısıl eğilme sıcaklığı HDT (0,45 MPa) — PA-CF'nin Tg'si ≈ 60 °C'dir ama karbon
    elyaflı kristal yapı onu HDT'ye kadar taşır."""
    key = "PETG" if material.startswith("PETG") else material
    f = _PR["filaments"].get(key) or _EXTRA_FIL.get(key) or {}
    return float(f.get("tg_c", 0.0))


def part_zone(key: str) -> str | None:
    """Spec ``zones[].parts`` ataması (bölünmüş parça ``stab_1a`` → ``stab_1``)."""
    if key in _PART_ZONES:
        return _PART_ZONES[key]
    if key[-1:] in ("a", "b") and key[-2:-1].isdigit():
        return _PART_ZONES.get(key[:-1])
    return None


def _env_distance(e: "P.EnvPart", Q: np.ndarray) -> np.ndarray:
    """Noktalar (spec, (n, 3)) ile paketleme zarfı (kutu ya da silindir) arası en kısa mesafe (m)."""
    A = np.asarray(e.axes, float)
    if A.shape == (3, 3) and not np.allclose(A @ A.T, np.eye(3), atol=1e-6):
        A = A.T
    c = np.asarray(e.center, float)
    q = (Q - c) @ np.column_stack([A[0], A[1], A[2]])        # yerel (axes satır vektörleri)
    h = np.asarray(e.half, float)
    if e.kind == "cyl":
        dx = np.maximum(np.abs(q[:, 0]) - h[0], 0.0)
        dr = np.maximum(np.hypot(q[:, 1], q[:, 2]) - h[1], 0.0)
        return np.hypot(dx, dr)
    return np.linalg.norm(np.maximum(np.abs(q) - h, 0.0), axis=1)


HEAT_PIPE = "susturucu çıkış borusu"
HEAT_SHIELD = "ısı kalkanı"
HEAT_SRC_TR = {"cylinder": "silindir", "spark_cap": "buji başlığı", "muffler": "susturucu", "crankcase": "karter",
               "carb": "karbüratör", "front_bearing": "rulman burnu"}      # rapor için Türkçe kaynak adları


def heat_sources() -> tuple[list, np.ndarray, np.ndarray]:
    """Isı kaynakları: motor zarfından ``print.heat_rule.around`` (silindir + buji başlığı, susturucu) ve adlı nokta
    kaynaklar — susturucu çıkış borusu (``shapes.muffler_pipe_tube`` + çıkış noktası) ve borunun çıktığı yerde
    yanağa 0,5 mm aralıkla oturan Al ısı kalkanı (``shapes.muffler_shield``); sahnedeki ``U_Exhaust_Muffler`` bu
    ikisinin birleşimidir. Döndürür: (zarflar, noktalar (n, 3) spec, nokta etiketleri (n,))."""
    from .. import shapes as S
    around = set(HEAT_RULE.get("around", ["cylinder", "muffler"]))
    env = [e for e in P.engine_envelope() if e.name in around or (e.name == "spark_cap" and "cylinder" in around)]
    pipe = np.vstack([np.asarray(P.muffler_outlet().pos, float)[None], _surface_samples(S.muffler_pipe_tube())])
    shield = _surface_samples(S.muffler_shield())
    labels = np.array([HEAT_PIPE] * len(pipe) + [HEAT_SHIELD] * len(shield), dtype=object)
    return env, np.vstack([pipe, shield]), labels


def _surface_samples(md, step: float = 0.001) -> np.ndarray:
    """Ağ yüzeyinden ≈ ``step`` aralıklı nokta örnekleri (spec): her üçgen kenar boyuna göre barisentrik ızgarayla
    bölünür. Uzun boru gibi az köşeli kaynakların yüzey ortası da örneklenir (yalnız köşe noktaları boru yanağın
    deliğinden geçtiği yerdeki en kısa uzaklığı ıskalıyordu)."""
    V = np.asarray(md.verts, float)
    out = [V]
    for f in md.faces:
        f = list(f)
        for i in range(1, len(f) - 1):
            a, b, c = V[f[0]], V[f[i]], V[f[i + 1]]
            n = int(min(40, max(1, math.ceil(max(np.linalg.norm(b - a), np.linalg.norm(c - b),
                                                  np.linalg.norm(a - c)) / step))))
            if n <= 1:
                continue
            u, v = np.meshgrid(np.arange(n + 1), np.arange(n + 1))
            m = (u + v) <= n
            u, v = u[m] / n, v[m] / n
            out.append(a + np.outer(u, b - a) + np.outer(v, c - a))
    return np.unique(np.round(np.vstack(out), 6), axis=0)


def heat_distance(V_world: np.ndarray, T: np.ndarray, mirror: bool, sources=None) -> tuple[float, str, dict]:
    """Parçanın (ve ayna eşinin) ısı kaynaklarına en kısa mesafesi (m), en yakın kaynağın adı ve kaynak başına en
    kısa mesafeler (m; ``{ad: d}``)."""
    env, pts, labels = sources or heat_sources()
    per: dict[str, float] = {}
    variants = [V_world] + ([V_world * np.array([1.0, -1.0, 1.0])] if mirror else [])
    for Vw in variants:
        Q = np.column_stack([-Vw[:, 0], Vw[:, 1], Vw[:, 2]])
        for e in env:
            d = float(_env_distance(e, Q).min())
            per[e.name] = min(per.get(e.name, math.inf), d)
        gap = np.maximum(np.maximum(Q.min(0) - pts.max(0), pts.min(0) - Q.max(0)), 0.0) if len(pts) else None
        if len(pts) and float(np.linalg.norm(gap)) < 0.30:     # kutular 0,3 m'den uzaksa nokta kaynakları atlanır
            Tm = T if Vw is V_world else T[:, ::-1]
            bvh = BVHTree.FromPolygons(Q.tolist(), Tm.tolist(), epsilon=0.0)
            for p, lab in zip(pts, labels):
                hit = bvh.find_nearest(Vector(tuple(p)))
                if hit[0] is not None and hit[3] < per.get(lab, math.inf):
                    per[lab] = float(hit[3])
    if not per:
        return math.inf, "", {}
    name = min(per, key=per.get)
    return per[name], name, per


def heat_rule_eval(rows: Sequence[dict]) -> dict:
    """Isı kuralı kararı (AERO-09, R05) — saf fonksiyon (birim testli). ``rows``: ``{"key", "material", "min_m",
    "source", "zone"?, "by_source"?}``. Kural 1: kaynaklara ``radius_m`` (150 mm) içinde ``no_material`` (LW-PLA)
    yok; kural 2: ısıl sınırı (``tg_c``: amorf filamentte Tg, PA-CF'de HDT) ``tg_min_c``'nin (120 °C) altındaki
    filament ``tg_radius_m``'den (50 mm) yakın değil; kural 3: hiçbir parça susturucu çıkış borusuna
    ``pipe_clear_m``'den (5 mm) yakın değil (``by_source``). Malzeme kendiliğinden değiştirilmez: ihlal raporlanır,
    çalıştırma 1 ile biter."""
    heat = {"radius_mm": float(HEAT_RULE["radius_m"]) * 1000.0, "no_material": HEAT_RULE["no_material"],
            "around": list(HEAT_RULE.get("around", [])) + [HEAT_PIPE, HEAT_SHIELD],
            "tg_rule": {"tg_min_c": HEAT_TG_RULE["tg_min_c"], "radius_mm": HEAT_TG_RULE["radius_m"] * 1000.0},
            "pipe_rule": {"clear_mm": HEAT_PIPE_CLEAR_M * 1000.0},
            "assigned": [], "violations": [], "near": []}
    for r in rows:
        hd, mat, key, src = float(r["min_m"]), str(r["material"]), str(r["key"]), str(r.get("source", ""))
        mm = round(hd * 1000.0, 1)
        if r.get("zone") is not None:
            heat["assigned"].append({"key": key, "zone": r["zone"], "material": mat, "min_mm": mm})
        if hd < float(HEAT_RULE["radius_m"]) and mat == HEAT_RULE["no_material"]:
            heat["violations"].append({"key": key, "material": mat, "min_mm": mm, "source": src,
                                       "rule": f"{mat} ≤ {heat['radius_mm']:.0f} mm",
                                       "fix": f"spec print.zones[].parts → {HEAT_SAFE_MATERIAL} ya da PA-CF"})
        elif hd < HEAT_TG_RULE["radius_m"] and tg_c(mat) < HEAT_TG_RULE["tg_min_c"]:
            heat["violations"].append({"key": key, "material": mat, "min_mm": mm, "source": src,
                                       "rule": f"ısıl sınır {tg_c(mat):.0f} °C < {HEAT_TG_RULE['tg_min_c']:.0f} °C, "
                                               f"≤ {HEAT_TG_RULE['radius_m'] * 1000:.0f} mm",
                                       "fix": "PA-CF (HDT ≈ 150 °C) ya da parçayı uzaklaştırın"})
        pipe = (r.get("by_source") or {}).get(HEAT_PIPE)
        if pipe is not None and float(pipe) < HEAT_PIPE_CLEAR_M:
            heat["violations"].append({"key": key, "material": mat, "min_mm": round(float(pipe) * 1000.0, 1),
                                       "source": HEAT_PIPE,
                                       "rule": f"çıkış borusuna ≤ {HEAT_PIPE_CLEAR_M * 1000:.0f} mm",
                                       "fix": "deliği büyütün (spec outlet.hole_clear_m) ve geçiş halkası kullanın"})
        if hd < 0.25:
            row = {"key": key, "material": mat, "tg_c": tg_c(mat), "min_mm": mm, "source": src}
            if r.get("by_source"):
                row["by_source_mm"] = {k: round(v * 1000.0, 1) for k, v in sorted(r["by_source"].items(),
                                                                                  key=lambda kv: kv[1])[:3]}
            heat["near"].append(row)
    heat["near"].sort(key=lambda x: x["min_mm"])
    heat["ok"] = not heat["violations"]
    return heat


def _split_recipe(r: Recipe, ev: dict, bed) -> list[Recipe] | None:
    """Tablaya sığmayan segmenti ikiye böler (yükseklik fazlaysa yukarı ekseni boyunca, iz büyükse uzun iz
    ekseni boyunca; kanatta veter yönündeki bölme ana kirişin arkasına konur). Yeni yüzler: kabuklu parçada
    çift kaburga (plate/plate), diğerlerinde açık."""
    seg = r.seg
    pl = ev["placement"]
    V = ev["V"]
    M = pl["M"]
    R = M[:3, :3]
    if pl["size_mm"][2] > bed[2] - Z_MARGIN:
        axis = np.asarray(pl["up"], float)
    else:
        W = V @ R.T
        ext = W[:, :2].max(0) - W[:, :2].min(0)
        e = np.array([1.0, 0, 0]) if ext[0] >= ext[1] else np.array([0, 1.0, 0])
        axis = _unit(R.T @ e)
    proj = V @ axis
    pos = 0.5 * (proj.min() + proj.max())
    if seg.group == "Kanat" and abs(axis[0]) > 0.8:          # veter yönü: ana kiriş kovanının arkası
        s_split = P.WING_SPAR_S + 0.034
        pos = float(Bp(s_split, 0, 0) @ axis)
    p = axis * pos
    shelled = seg.shell and not seg.cover
    up = np.asarray(pl["up"], float)
    along_up = abs(float(axis @ up)) > 0.9
    has_land = any(e.kind == "land" for e in seg.ends)
    bed_e = seg.ends[seg.bed_end] if seg.bed_end is not None and seg.bed_end < len(seg.ends) else None
    kids = []
    for tag, nvec in (("a", axis), ("b", -axis)):
        kind, bed_end = ("plate" if shelled else "open"), seg.bed_end
        if shelled and along_up and has_land and bed_e is not None:
            holds_bed = float((bed_e.p - p) @ nvec) < 0      # bu çocuk yatak ucunu içeriyor mu
            if holds_bed:
                kind = "land"                               # yeni yüz baskının üstü → yapıştırma flanşı
            else:
                kind = bed_e.kind                           # yeni yüz bu çocuğun tablası (kaburga/çerçeve)
                bed_end = len(seg.ends)
        ends = list(seg.ends) + [End(p.copy(), nvec, kind, label="tabla bölmesi", holes=False)]
        sep = "" if seg.key[-1].isdigit() else "_"           # stab_2 → stab_2a, rudder → rudder_a
        s2 = SegSpec(**{**seg.__dict__, "key": f"{seg.key}{sep}{tag}", "ends": ends, "bed_end": bed_end,
                        "name_tr": f"{seg.name_tr} — parça {tag.upper()} (tablaya sığması için bölündü)"})
        kids.append(Recipe(s2, r.feat_fn))
    return kids


def _mirror_object(ob: bpy.types.Object, name: str, col: bpy.types.Collection) -> bpy.types.Object:
    V, F = _mesh_np(ob.data)
    V = V * np.array([1.0, -1.0, 1.0])
    me = _new_mesh(name, V, [tuple(reversed(f)) for f in F])
    for m in ob.data.materials:
        me.materials.append(m)
    o2 = bpy.data.objects.new(name, me)
    col.objects.link(o2)
    for k in ob.keys():
        o2[k] = ob[k]
    return o2


def _clear_print_collection(col: bpy.types.Collection) -> None:
    for ob in list(col.objects):
        if ob.name.startswith("UP_"):
            _delete(ob)


def _separate_coincident(V: np.ndarray, T: np.ndarray, eps: float = 5e-7, push: float = 2e-6) -> tuple[np.ndarray, int]:
    """Topolojik olarak ayrı ama aynı yerdeki köşeler (kendine değen ağ: iki yüzey bir çizgide/noktada temas eder)
    STL okunurken birleşir ve manifold olmayan kenar üretir. Bu köşeler kendi malzemelerine doğru (köşe normalinin
    tersi) ``push`` kadar itilir → temas µm boşluğa döner, ağ topolojisi değişmez. Dönüş: (V, itilen köşe sayısı)."""
    from mathutils.kdtree import KDTree
    kd = KDTree(len(V))
    for i, v in enumerate(V):
        kd.insert(Vector(v), i)
    kd.balance()
    rank: dict[int, int] = {}
    for i, v in enumerate(V):
        if i in rank:
            continue
        r = sorted(j for _, j, _ in kd.find_range(Vector(v), eps))
        if len(r) > 1:
            for k, j in enumerate(r):                   # aynı normalli üst üste yüzeyler de ayrılsın: 1×, 2×, 3× itme
                rank.setdefault(j, k)
    if not rank:
        return V, 0
    a, b, c = V[T[:, 0]], V[T[:, 1]], V[T[:, 2]]
    fn = np.cross(b - a, c - a)
    N = np.zeros_like(V)
    for k in range(3):
        np.add.at(N, T[:, k], fn)
    ln = np.linalg.norm(N, axis=1)
    N /= np.maximum(ln, 1e-30)[:, None]
    N[ln < 1e-14] = np.array([0.57735, 0.57735, 0.57735])   # yalnız sıfır alanlı üçgenlerin köşesi: sabit yön
    idx = np.array(sorted(rank))
    mult = np.array([1.0 + rank[i] for i in idx])
    V2 = V.copy()
    V2[idx] -= N[idx] * (push * mult)[:, None]
    return V2, len(idx)


def _welded_dups(V: np.ndarray, T: np.ndarray) -> int:
    """STL okumasında çift yönlü kenar sayısı: (1) mm float32 tam eşitlik, (2) 0,1 µm ızgara kaynağı (denetçi /
    dilimleyici okuması). Büyük olanı döner."""
    worst = 0
    for q in ((V * 1000.0).astype(np.float32), np.round(V * 1e7).astype(np.int64)):
        _, inv = np.unique(q, axis=0, return_inverse=True)
        Tw = inv.reshape(-1)[T]                         # dejenere üçgen de sayılır (okuyucu atmaz)
        ec = stl_edge_check(Tw)
        worst = max(worst, int(ec["dup_directed"]) + int(ec["unpaired"]))
    return worst


def _fix_pinches(b: "Built") -> None:
    """Kendine değen ağ (STL kaynağında manifold olmayan kenar) varsa eski tam temizlik (5 µm kaynak + sıkışma
    kenarı çökertme) bir KOPYADA denenir; kapalı, çift kenarsız ve hacim farkı ≤ %0,5 ise kabul edilir."""
    V, T = triangulate(b.ob.data)
    if _welded_dups(V, T) == 0:
        return
    keep = b.ob.data.copy()
    v0 = mesh_check(keep)["volume_m3"]
    clean_mesh(b.ob, triangles=True)
    c = mesh_check(b.ob.data)
    V2, T2 = triangulate(b.ob.data)
    if c["ok"] and _welded_dups(V2, T2) == 0 and abs(c["volume_m3"] - v0) <= 0.005 * abs(v0):
        bpy.data.meshes.remove(keep)
        b.check = {**c, "removed_islands": b.check.get("removed_islands", 0)}
        return
    bad = b.ob.data
    b.ob.data = keep
    bpy.data.meshes.remove(bad)


ZERO_AREA_MM2 = 1e-9                                    # |AB × AC| bundan küçükse üçgen sıfır alanlı sayılır (mm²)
SLIVER_H_MM = 1e-5      # uzun kenarına yüksekliği bundan küçük üçgen float32 mm'de doğrusaldır (250 mm'de float32 adımı 1,5e-5)


def _mm32(V: np.ndarray) -> np.ndarray:
    """STL'ye yazılacak köşeler: metre → mm, float32'ye yuvarlanmış (hesap float64)."""
    return (np.asarray(V, np.float64) * 1000.0).astype(np.float32).astype(np.float64)


def _tri_cross_len(W: np.ndarray, T: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """Üçgen başına |AB × AC| (mm²) ve en uzun kenar (mm)."""
    W = np.asarray(W, np.float64)
    a, b, c = W[T[:, 0]], W[T[:, 1]], W[T[:, 2]]
    cr = np.linalg.norm(np.cross(b - a, c - a), axis=1)
    L = np.maximum(np.maximum(np.linalg.norm(b - a, axis=1), np.linalg.norm(c - b, axis=1)), np.linalg.norm(a - c, axis=1))
    return cr, L


def _zero_area(W: np.ndarray, T: np.ndarray, tol_mm2: float = ZERO_AREA_MM2) -> np.ndarray:
    """Sıfır alanlı üçgenlerin indisleri (``W`` mm)."""
    cr, _ = _tri_cross_len(W, T)
    return np.nonzero(cr < tol_mm2)[0]


def _slivers(W: np.ndarray, T: np.ndarray, h_mm: float = SLIVER_H_MM) -> np.ndarray:
    """Sıfır alanlı ya da uzun kenarına yüksekliği ``h_mm``'den küçük (float32 mm çözünürlüğünde doğrusal) üçgenler."""
    cr, L = _tri_cross_len(W, T)
    return np.nonzero((cr < ZERO_AREA_MM2) | (cr < h_mm * L))[0]


def _flip_degenerate(V: np.ndarray, T: np.ndarray, h_mm: float = SLIVER_H_MM) -> tuple[np.ndarray, int]:
    """STL'ye sıfır alanlı ya da doğrusal (yüksekliği ``h_mm``'den küçük) üçgen yazılmasın: sınama, yazılacak float32
    mm köşelerle yapılır. Orta köşe ``m`` uzun kenar ``p→q`` üzerindeyse o kenarı paylaşan komşu ``(q, p, x)``
    ``m``'de bölünür: ``(q, m, x)`` + ``(m, p, x)``. Üçgen sayısı ve kenar eşleşmesi korunur, yüzey en çok ``h_mm``
    kadar oynar; yeni üçgenler de ince ya da ters yönlü çıkacaksa veya ``m–x`` kenarı zaten varsa o üçgene dokunulmaz
    (art arda doğrusal üçgenler kalabilir). Döndürür: (yeni T, giderilen sayısı)."""
    W = _mm32(V)

    def cross(tris: np.ndarray) -> np.ndarray:
        a, b, c = W[tris[:, 0]], W[tris[:, 1]], W[tris[:, 2]]
        return np.cross(b - a, c - a)

    T = np.array(T, copy=True)
    n = 0
    for _ in range(6):                                  # bir bölme yeni ince üçgen bırakırsa birkaç tur
        bad = _slivers(W, T, h_mm)
        if not len(bad):
            break
        emap = {}
        for i, (p0, p1, p2) in enumerate(T.tolist()):
            emap[(p0, p1)] = i
            emap[(p1, p2)] = i
            emap[(p2, p0)] = i
        n_round = 0
        for i in bad.tolist():
            if not len(_slivers(W, T[i:i + 1], h_mm)):  # bu turda komşu olarak zaten bölündü
                continue
            tri = T[i].tolist()
            L = [float(np.linalg.norm(W[tri[(k + 1) % 3]] - W[tri[k]])) for k in range(3)]
            if min(L) <= 0.0:                           # çakışık köşe (iğne): bu yöntemin konusu değil
                continue
            k = int(np.argmax(L))
            p_, q_, m_ = tri[k], tri[(k + 1) % 3], tri[(k + 2) % 3]
            j = emap.get((q_, p_))
            if j is None or j == i:
                continue
            x = [v for v in T[j].tolist() if v not in (p_, q_)]
            if len(x) != 1 or (m_, x[0]) in emap or (x[0], m_) in emap:
                continue
            x = x[0]
            new = np.array([(q_, m_, x), (m_, p_, x)])
            nn, nj = cross(new), cross(T[j:j + 1])[0]
            if len(_slivers(W, new, h_mm)) or (nn @ nj <= 0.0).any():
                continue
            T[i], T[j] = new
            del emap[(p_, q_)], emap[(q_, p_)]
            emap.update({(q_, m_): i, (m_, x): i, (x, q_): i, (m_, p_): j, (p_, x): j, (x, m_): j})
            n_round += 1
        n += n_round
        if not n_round:
            break
    return T, n


def export_part(b: Built, ev: dict, stl_dir: Path) -> dict:
    """Baskı yönünde, tabla merkezli, mm STL (tabla teması düzeltmesi varsa onunla). Kendine değen köşeler µm
    ayrılır (``_separate_coincident``). Dönüş: dosya bilgisi (yazılan ağın hacmi dahil)."""
    if "bed_fix" in ev:
        V, T = ev["bed_fix"]["W"], ev["bed_fix"]["T"]
    else:
        M = ev["placement"]["M"]
        V = ev["V"] @ M[:3, :3].T + M[:3, 3]
        T = ev["T"]
    n_sep = 0
    for push in (2e-6, 5e-6, 1.2e-5):
        if _welded_dups(V, T) == 0:
            break
        V, n = _separate_coincident(V, T, push=push)
        n_sep += n
    if n_sep:
        V = V - np.array([0.0, 0.0, V[:, 2].min()])     # itme tabla altına taşıdıysa z_min = 0
    T2, n_flip = _flip_degenerate(V, T)
    if n_flip and stl_edge_check(T2)["ok"]:             # kapalılık bozulursa eski üçgenlerle kalınır
        T = T2
    else:
        n_flip = 0
    W = _mm32(V)
    n_zero, n_sliver = int(len(_zero_area(W, T))), int(len(_slivers(W, T)))
    path = stl_dir / f"{b.seg.key}.stl"
    size = write_stl(path, V, T, b.seg.key)
    a, bb, c = V[T[:, 0]], V[T[:, 1]], V[T[:, 2]]
    vol = float(np.einsum("ij,ij->i", a, np.cross(bb, c)).sum() / 6.0) * 1e6
    return {"file": f"{stl_dir.name}/{path.name}", "bytes": size, "triangles": int(len(T)),   # çıktı köküne göre
            "volume_cm3": round(vol, 3), "separated_verts": n_sep, "degenerate_split": n_flip, "zero_area": n_zero,
            "slivers": n_sliver}


def verify_stl(path: Path, expect_volume_cm3: float) -> dict:
    """STL'yi yeniden okur: kenar eşleşmesi (kapalı, tutarlı yön), hacim karşılaştırması; ayrıca Blender STL
    içe aktarıcısıyla (``wm.stl_import``) yükleyip bmesh manifold denetimi yapar."""
    V, T = read_stl(path)
    ec = stl_edge_check(T)
    q = np.round(V / 1e-4).astype(np.int64)            # 0,1 µm kaynakla da (dilimleyici / denetçi okuması) kapalı mı
    _, inv = np.unique(q, axis=0, return_inverse=True)
    Tw = inv.reshape(-1)[T]
    ec["welded"] = stl_edge_check(Tw)
    ec["ok"] = ec["ok"] and ec["welded"]["ok"]
    ec["zero_area"] = int(len(_zero_area(V, T)))
    ec["slivers"] = int(len(_slivers(V, T)))
    a, b, c = V[T[:, 0]], V[T[:, 1]], V[T[:, 2]]
    vol = float(np.einsum("ij,ij->i", a, np.cross(b, c)).sum() / 6.0) / 1000.0
    out = {"edges": ec, "volume_cm3": vol, "volume_err_pct": 100.0 * (vol - expect_volume_cm3) /
           max(abs(expect_volume_cm3), 1e-9)}
    try:
        before = set(bpy.data.objects.keys())
        bpy.ops.wm.stl_import(filepath=str(path))
        new = [bpy.data.objects[n] for n in set(bpy.data.objects.keys()) - before]
        if new:
            ob = new[0]
            bm = bmesh.new()
            bm.from_mesh(ob.data)
            bmesh.ops.remove_doubles(bm, verts=bm.verts[:], dist=1e-5)
            nm = sum(1 for e in bm.edges if not e.is_manifold)
            out["blender_import"] = {"verts": len(bm.verts), "faces": len(bm.faces), "nonmanifold": nm,
                                     "volume_cm3": bm.calc_volume(signed=True) / 1000.0}
            bm.free()
            for o in new:
                _delete(o)
    except Exception as exc:                            # pragma: no cover - içe aktarıcı yoksa
        out["blender_import"] = {"error": str(exc)}
    return out


# =====================================================================================================
# Ana giriş
# =====================================================================================================
def _jsonable(o):
    """numpy skalerleri/dizileri → JSON."""
    if isinstance(o, np.generic):
        return o.item()
    if isinstance(o, np.ndarray):
        return o.tolist()
    return str(o)


def _ensure_airframe(scene: bpy.types.Scene, verbose: bool = False) -> None:
    if "U_Fuselage" not in bpy.data.objects:
        from . import airframe
        airframe.build(scene, verbose=verbose)


def _explode_vector(ob: bpy.types.Object, seg: SegSpec, factor: float = 0.28) -> np.ndarray:
    V, _ = _mesh_np(ob.data)
    c = V.mean(0)
    cg = np.asarray(P.U_ROOT_B)
    d = (c - cg) * factor
    if seg.group == "Kumanda yüzeyleri":
        d += np.array([-0.06, 0.0, -0.02])
    if seg.group == "Kaplamalar" or seg.group == "Takım kapakları":
        d += np.array([0.0, 0.0, -0.08 if c[2] < 0.03 else 0.08])
    return d


def build_print_parts(bed=BED_DEFAULT, out_dir=None, *, scene: bpy.types.Scene | None = None,
                      only: Sequence[str] | None = None, export: bool = True, write_report: bool = True,
                      solver: str = "MANIFOLD", cut_solver: str = "EXACT", mirror_objects: bool = True,
                      verify: int = -1, verbose: bool = False) -> dict:
    """Baskı parçalarını kurar, ``UCAV_Print``'e koyar, STL (mm) ve rapor yazar.

    ``bed``: tabla (mm) — varsayılan 256×256×256; 220×220×250 de desteklenir (sığmayan segment otomatik bölünür).
    ``out_dir``: çıktı kökü (varsayılan ``ucav/out``; varsayılan dışı tabla için ``ucav/out/print_<X>x<Y>x<Z>``).
    STL'ler ``<out_dir>/stl``, rapor ``<out_dir>/print_report.md`` ve ``.json``. ``only``: anahtar önekleri
    (ör. ``["wing_panel", "fus_ring_1"]``) — kısmi kurulum. ``verify``: yeniden okunup doğrulanacak STL sayısı
    (−1 → hepsi).
    Dönüş: özet sözlüğü (parçalar, toplamlar, uyarılar, dosyalar).
    """
    t_start = time.time()
    scene = scene or bpy.context.scene
    bed = tuple(float(b) for b in bed)
    _ensure_airframe(scene, verbose)
    if out_dir is None:
        out_dir = P.OUT_DIR if tuple(int(b) for b in bed) == BED_DEFAULT else \
            P.OUT_DIR / f"print_{int(bed[0])}x{int(bed[1])}x{int(bed[2])}"
    out_dir = Path(out_dir)
    stl_dir = out_dir / "stl"
    cols = U.ensure_collections(scene)
    pcol = cols["UCAV_Print"]
    _clear_print_collection(pcol)
    ctx = Ctx(scene, bed, solver=solver, cut_solver=cut_solver, verbose=verbose)
    recipes, plans = make_recipes(ctx, only)
    ctx.log(f"{len(recipes)} tarif")
    queue = list(recipes)
    done: list[tuple[Built, dict]] = []
    depth: dict[str, int] = {}
    while queue:
        r = queue.pop(0)
        seg = r.seg
        t0 = time.time()
        faulthandler.dump_traceback_later(PART_TIMEOUT_S, exit=True)     # boolean takılırsa sessizce beklemesin
        try:
            if seg.builder is not None:
                _CUR_MAT[0] = seg.material or _zone(seg.zone)[0]
                b = seg.builder(ctx, seg, pcol)
            elif seg.key == "fus_nose_flange":
                b = build_nose_flange(ctx, seg, pcol)
            elif not seg.shell:
                b = build_plain(ctx, seg, pcol)
            else:
                b = None

                def rank(bk):                           # kapalı > boşluğu çıkmış > az sınır kenarlı
                    c = bk.check
                    return (0 if c["ok"] else 1, 1 if c.get("cavity_failed") else 0, c["boundary"] + c["nonmanifold"])

                tries = [(k, None) for k in range(N_PERTURB)]
                for k, solver_k in tries:               # sıkışma kalırsa küçük kaydırmayla, sonra EXACT ile yeniden dene
                    s_try = seg if k == 0 else SegSpec(**{**seg.__dict__, "perturb": k})
                    _CUR_MAT[0] = seg.material or _zone(seg.zone)[0]       # delik boşluğu malzemeye göre (P7)
                    feat = r.feat_fn(s_try) if r.feat_fn else Feature()
                    old_solver = ctx.B.solver
                    if solver_k:
                        ctx.B.solver = solver_k
                    try:
                        bk = build_segment(ctx, s_try, feat, pcol)
                    finally:
                        ctx.B.solver = old_solver
                    if bk is None:
                        break
                    if b is None or rank(bk) < rank(b):
                        if b is not None:
                            _delete(b.ob)
                        b = bk
                        b.seg = seg
                    else:
                        _delete(bk.ob)
                    if b.check["ok"]:
                        if k:
                            ctx.log(f"{seg.key}: {k}. denemede manifold" + (f" ({solver_k})" if solver_k else ""))
                        break
                if b is not None:
                    b.ob.name = f"UP_{seg.key}"
                    b.ob.data.name = b.ob.name
        except Exception as exc:                        # bir parçanın hatası diğerlerini durdurmasın
            ctx.warnings.append(f"{seg.key}: kurulamadı ({type(exc).__name__}: {exc})")
            ctx.log(f"HATA {seg.key}: {exc}")
            continue
        finally:
            faulthandler.cancel_dump_traceback_later()
        if b is None or len(b.ob.data.polygons) == 0:
            ctx.warnings.append(f"{seg.key}: boş sonuç (kaynak bölgede yok)")
            if b is not None:
                _delete(b.ob)
            continue
        if (not b.check["ok"] and not b.check.get("cavity_failed") and 0 < b.check["volume_m3"] < VOXEL_REPAIR_MAX_M3
                and len(b.ob.data.polygons) < VOXEL_REPAIR_MAX_FACES):
            v0 = b.check["volume_m3"]
            keep = b.ob.data.copy()
            info = voxel_repair(b.ob)
            chk = {**mesh_check(b.ob.data), "removed_islands": b.check.get("removed_islands", 0)}
            if chk["ok"] and abs(chk["volume_m3"] - v0) <= 0.05 * v0:      # boşluk dolmadıysa kabul
                b.check = chk
                msg = (f"{seg.key}: voksel onarım ({info['voxel_mm']:.2f} mm), hacim {v0 * 1e6:.1f} → "
                       f"{b.check['volume_m3'] * 1e6:.1f} cm³")
                b.info.setdefault("notes", []).append("voksel onarım")
                bpy.data.meshes.remove(keep)
            else:                                       # onarım boşluğu doldurdu / kapanmadı: özgün ağ kalır
                bad_me = b.ob.data
                b.ob.data = keep
                bpy.data.meshes.remove(bad_me)
                msg = f"{seg.key}: voksel onarım reddedildi (hacim {v0 * 1e6:.1f} → {chk['volume_m3'] * 1e6:.1f} cm³)"
            ctx.warnings.append(msg)
            ctx.log(msg)
        if not seg.split_loose:
            n_fr, v_fr = drop_fragments(b.ob)
            if n_fr:
                b.check = {**mesh_check(b.ob.data), "removed_islands": b.check.get("removed_islands", 0) + n_fr}
                b.info.setdefault("notes", []).append(f"{n_fr} kopuk kıymık atıldı ({v_fr * 1e6:.2f} cm³)")
                ctx.log(f"{seg.key}: {n_fr} kopuk kıymık atıldı ({v_fr * 1e6:.2f} cm³)")
        if b.check.get("ok") and b.seg.ends and b.seg.shell and not b.seg.builder:
            ng = joint_grooves(ctx, b.seg, b.ob)                # P13: ek kenarı pahı (V-oluk = panel çizgisi)
            if ng:
                b.check = {**mesh_check(b.ob.data), "removed_islands": b.check.get("removed_islands", 0)}
                b.info.setdefault("notes", []).append(
                    f"ek pahı {_fmt_g(GROOVE[0] * 1000)} × {_fmt_g(GROOVE[1] * 1000)} mm")
        if b.check.get("ok"):                                   # 0,1 µm kaynakta çift kenar (kendine değen ağ) kalmasın
            _fix_pinches(b)
        builts = [b]
        if seg.split_loose:
            shells = split_shells(b.ob)
            if len(shells) > 1:
                builts = []
                mats = list(b.ob.data.materials)
                base_key = seg.key
                _delete(b.ob)
                kept = []
                for i, (Vs, Fs) in enumerate(shells):
                    c = Vs.mean(0)
                    if c[1] < -1e-4 and any(abs(c2[0] - c[0]) < 2e-3 and abs(c2[1] + c[1]) < 2e-3 and
                                            abs(c2[2] - c[2]) < 2e-3 for c2 in kept):
                        continue                         # sağ ayna eşi (sol eşiyle birlikte sayılır)
                    kept.append(c)
                    tag = "abcdefgh"[len(builts)]
                    s2 = SegSpec(**{**seg.__dict__, "key": f"{base_key}_{tag}", "split_loose": False,
                                    "mirror": c[1] > 1e-4,
                                    "name_tr": f"{seg.name_tr} ({'üst' if c[2] > 0.066 else 'alt'})"})
                    ob = _obj(f"UP_{s2.key}", Solid(Vs, Fs), pcol)
                    finalize_mesh(ob)
                    for m in mats:
                        ob.data.materials.append(m)
                    builts.append(Built(s2, ob, mesh_check(ob.data)))
        for bb in builts:
            ev = evaluate(bb, bed, ctx)
            pl = ev["placement"]
            if not pl["fits"] and not pl["fits_tight"] and depth.get(bb.seg.key, 0) < 3:
                kids = _split_recipe(Recipe(bb.seg, r.feat_fn), ev, bed)
                if kids:
                    ctx.log(f"{bb.seg.key}: tablaya sığmadı {np.round(pl['size_mm'], 0)} → ikiye bölünüyor")
                    for kd in kids:
                        depth[kd.seg.key] = depth.get(bb.seg.key, 0) + 1
                    queue = kids + queue
                    _delete(bb.ob)
                    continue
            done.append((bb, ev))
            ctx.log(f"{bb.seg.key}: {'OK' if bb.check['ok'] else 'SORUN'} {ev['volume_cm3']:.1f} cm³ "
                    f"{ev['mass_g']:.0f} g {np.round(pl['size_mm'], 0)} mm ({time.time() - t0:.1f} s)")
    # ısı kuralı (AERO-09, R05): (1) susturucu/silindir zarfları ve susturucu çıkışının ``radius_m`` (150 mm)
    # yakınında LW-PLA yok; (2) ısıl sınırı < 120 °C olan parça bunlara 50 mm'den yakın değil; (3) hiçbir parça
    # çıkış borusuna 5 mm'den yakın değil. Malzeme kendiliğinden
    # değiştirilmez: ihlal raporlanır ve çalıştırma 1 ile biter (build.py --print, printprep CLI). Isı bölgesindeki
    # parçaların malzemesi spec'te adıyla atanır (``print.zones[].parts``).
    heat_src = heat_sources()
    rows = []
    for b, ev in done:
        hd, hsrc, per = heat_distance(ev["V"], ev["T"], b.seg.mirror, heat_src)
        ev["heat"] = {"min_mm": round(hd * 1000.0, 1), "source": hsrc}
        rows.append({"key": b.seg.key, "material": ev["material"], "min_m": hd, "source": hsrc,
                     "zone": part_zone(b.seg.key), "by_source": per})
    heat = heat_rule_eval(rows)
    for v in heat["violations"]:
        ctx.warnings.append(f"ısı kuralı ihlali: {v['key']} ({v['material']}), en yakın kaynak {v['source']} "
                            f"{v['min_mm']:.0f} mm — {v['rule']}")
    ctx.heat = heat
    # malzeme, özellikler, ayna eşleri
    results = []
    for b, ev in done:
        seg = b.seg
        mat = ev["material"]
        b.ob.data.materials.clear()
        b.ob.data.materials.append(U.get_material(FILAMENT_MATERIAL.get(mat, "UM_SkinBottom")))
        qty = seg.qty * (2 if seg.mirror else 1)
        b.ob["ucav_print_key"] = seg.key
        b.ob["ucav_print_material"] = mat
        b.ob["ucav_print_mass_g"] = round(ev["mass_g"], 1)
        b.ob["ucav_print_qty"] = qty
        b.ob["ucav_explode"] = list(map(float, _explode_vector(b.ob, seg)))
        b.ob["ucav_print_tooling"] = bool(seg.tooling)
        b.ob.hide_render = True
        if seg.mirror and mirror_objects:
            name_r = b.ob.name + "_R"
            o2 = _mirror_object(b.ob, name_r, pcol)
            ex = np.asarray(b.ob["ucav_explode"])
            o2["ucav_explode"] = [float(ex[0]), float(-ex[1]), float(ex[2])]
            o2["ucav_print_mirror_of"] = b.ob.name
            o2.hide_render = True
        info = {"key": seg.key, "name": seg.name_tr, "group": seg.group, "material": mat, "zone": seg.zone,
                "wall_mm": round(ev["wall_mm"], 2), "qty": qty, "mirror": seg.mirror,
                "check": {k: v for k, v in b.check.items() if k != "volume_m3"},
                "volume_cm3": round(ev["volume_cm3"], 2), "mass_g": round(ev["mass_g"], 1),
                "fill": round(float(ev.get("fill", 1.0)), 3), "mass_solid_g": round(ev["volume_cm3"] * density(mat), 1),
                "mass_total_g": round(ev["mass_g"] * qty, 1), "print_h": round(ev["hours"], 2),
                "print_h_total": round(ev["hours"] * qty, 2),
                "size_mm": [round(x, 1) for x in ev["placement"]["size_mm"]],
                "orientation": {"up_world": [round(x, 3) for x in ev["placement"]["up"]],
                                "up_words": _axis_words(np.asarray(ev["placement"]["up"])),
                                "z_rot_deg": ev["placement"]["angle_deg"],
                                "bed_end": (seg.ends[seg.bed_end].label if seg.bed_end is not None and
                                            seg.bed_end < len(seg.ends) else "")},
                "bed_margin_mm": round(ev["placement"]["margin_mm"], 1), "fits": ev["placement"]["fits"],
                "fits_tight": ev["placement"]["fits_tight"], "overhang_cm2": round(ev["placement"]["overhang_cm2"], 1),
                "bed_contact_cm2": round(ev["placement"]["bed_contact_cm2"], 2),
                "contact_need_cm2": round(ev["placement"]["contact_need_cm2"], 2),
                "contact_ok": bool(ev["placement"]["contact_ok"]),
                "oh45_to_bed_cm2": ev["oh45_to_bed_cm2"], "oh45_over_model_cm2": ev["oh45_over_model_cm2"],
                "oh45_over_model_gt5mm_cm2": ev["oh45_over_model_gt5mm_cm2"],
                "bed_fix": ev["bed_fix"]["note"] if "bed_fix" in ev else "",
                "support": ev["support"], "min_wall": {k: round(v, 3) for k, v in ev["min_wall"].items()},
                "note": seg.note, "pin_holes": int(b.info.get("pin_holes", 0)), "heat": ev.get("heat", {}),
                "tooling": bool(seg.tooling),
                "features": sorted(set(b.info.get("notes", [])))}
        if export:
            info["stl"] = export_part(b, ev, stl_dir)
        results.append(info)
    # doğrulama: birkaç STL'yi yeniden oku
    verified = []
    if export and verify:
        pick = sorted(results, key=lambda r: -r["volume_cm3"])
        verify = len(pick) if verify < 0 else verify
        step = max(1, len(pick) // max(1, verify))
        for r in pick[::step][:verify]:
            verified.append({"key": r["key"], **verify_stl(out_dir / r["stl"]["file"],
                                                            r["stl"].get("volume_cm3", r["volume_cm3"]))})
    summary = summarize(results, bed, plans, ctx, verified, time.time() - t_start)
    if write_report:
        out_dir.mkdir(parents=True, exist_ok=True)
        (out_dir / "print_report.json").write_text(json.dumps(summary, ensure_ascii=False, indent=1, default=_jsonable),
                                                   encoding="utf-8")
        (out_dir / "print_report.md").write_text(report_md(summary), encoding="utf-8")
    ctx.cleanup()
    ctx.log(f"bitti: {len(results)} benzersiz parça, {time.time() - t_start:.0f} s, boolean {ctx.B.n_ops} "
            f"({ctx.B.t_ops:.0f} s)")
    return summary


# =====================================================================================================
# Malzeme listesi (basılmayanlar) ve rapor
# =====================================================================================================
def _longeron_rows() -> list[dict]:
    """Longeron düz parça boyları (P3: kırıklarda PETG soket blokları)."""
    rows = []
    for kind, nm in (("chine", "chine"), ("shoulder", "omuz")):
        try:
            pcs = P.longeron_pieces(kind, "L")
        except Exception:                               # pragma: no cover - eski params
            continue
        Ls = [float(np.linalg.norm(np.asarray(b) - np.asarray(a))) for a, b in pcs]
        rows.append({"group": "CF boru/çubuk", "item": f"Gövde longeronu — {nm} (düz parçalar, sol + sağ)",
                     "spec": "Ø8/6 mm; parça boyları " + " + ".join(f"{L * 1000:.0f}" for L in Ls) +
                             " mm (uçlar soket bloklarına 18 mm girer)", "qty": 2 * len(Ls)})
    return rows


def bom(plans: dict, n_pin_joints: int) -> list[dict]:
    """Basılmayan parçalar: CF borular, G10, bağlantı elemanları, servolar, takım, motor, aviyonik (spec'ten)."""
    out = []
    for t in _PR["spar_tubes"]:
        if t["name"] == "longeron":
            out += _longeron_rows()
            continue
        L = float(t["length_m"]) if "length_m" in t else 0.0
        out.append({"group": "CF boru/çubuk", "item": t["name_tr"],
                    "spec": f"Ø{t['od_mm']}/{t['id_mm']} mm × {L * 1000:.0f} mm" if t["id_mm"] else
                    f"Ø{t['od_mm']} mm dolu × {L * 1000:.0f} mm", "qty": int(t["qty"])})
    pins = []
    for nm, hp in plans.items():
        u0, u1 = PIN_BORE_EXT.get(nm, (-0.006, 0.006))
        pins.append(f"{nm} {(hp.L + u1 - u0) * 1000:.0f}")
    out.append({"group": "CF boru/çubuk", "item": "Menteşe pimi — Ø1,75 mm PETG filament (P1; çelik tel yerine)",
                "spec": f"delik Ø{_fmt_g(HINGE_PIN_HOLE_D * 1000)} mm çevrel; boylar (mm, yüzey başına): " + ", ".join(pins),
                "qty": 2 * len(plans)})
    out.append({"group": "CF boru/çubuk", "item": "Segment hizalama pimi (CF çubuk)",
                "spec": f"Ø{PIN_D * 1000:.0f} mm × {2 * PIN_DEPTH * 1000 - 2:.0f} mm", "qty": int(n_pin_joints)})
    out.append({"group": "CF boru/çubuk", "item": "İç flap bağlayıcı teli (P9: iç flap dış flaba bağlı, ayrı servo yok)",
                "spec": f"Ø{FLAP_JOINER['d'] * 1000:.0f} mm yay teli, 60 mm, iki ucu 90° kıvrık + Ø3/2,1 pirinç boru "
                        "soket (flap içine epoksi)", "qty": 2})
    g = _PR["g10_bridge"]
    out += [
        {"group": "G10 / kontrplak", "item": "Dihedral köprüsü (P4: kök bloğu ve kanat kutusu çerçevelerindeki "
                                             "3,3 mm yuvalara epoksi)",
         "spec": f"{g['plates']}× {g['t_mm']}×{G10_H * 1000:.0f}×{g['length_mm']} mm G10 (panelin 45 mm'si kök profiline "
                 f"ve depo tabanına sığmaz), {g['bolts']}", "qty": 1},
        {"group": "G10 / kontrplak", "item": "Yangın perdesi (s = 2,045)",
         "spec": f"G10 {float(P.SPEC['fuselage']['modules']['firewall_t_m']) * 1000:.0f} mm, gövde konturu; NACA kanal "
                 "açıklığı", "qty": 1},
        {"group": "G10 / kontrplak", "item": "Ön gövde / kuyruk modülü flanşı (s = 1,55)", "spec": "G10 2 mm, 4×M4",
         "qty": 1},
        {"group": "G10 / kontrplak", "item": "ER-150 takım montaj plakası (ana yatağa 2 × M3, burun yatağına 4 × M3)",
         "spec": "G10 3 mm", "qty": 3},
        {"group": "G10 / kontrplak", "item": "Kumanda hornu", "spec": "G10 1,6 mm, yarığa yapıştırma",
         "qty": 2 * (len(plans) - 1)},
        {"group": "G10 / kontrplak", "item": "Kuyruk eyeri (stabilize boruları V birleşimi)",
         "spec": "G10 2 mm + epoksi, koni içinde", "qty": 1},
        {"group": "Dolgu", "item": "Kanat üstü fileto (basılmaz: sıfıra inen kama)",
         "spec": "epoksi + mikrobalon, kanat kökü–chine arası, ≈ 2 × 17 cm³; şablon: U_Fairing_Fillet_L/R", "qty": 2},
        {"group": "Dolgu", "item": "Stabilize kök filetosu (basılmaz: ≤ 3 mm kama)",
         "spec": "epoksi + mikrobalon (ısı bölgesi; LW-PLA yok), ≈ 2 × 4 cm³", "qty": 2},
        {"group": "Dolgu", "item": "Dikey kök mermisi (basılmaz: dikey + stabilizeye teğet, yandan kırpmalı tarif yok)",
         "spec": "epoksi + mikrobalon ya da 0,5 mm ısıl biçimlendirilmiş levha; dışarıda ≈ 2 × 10 cm³; şablon: "
                 "U_Fairing_FinRoot_L/R", "qty": 2},
        {"group": "Dolgu", "item": "Kaplama kenarları (kök kaportası, ER-150 kabartması, taret yakası: ≥ 0,8 mm'de "
         "kırpılmış)", "spec": "epoksi + mikrobalon ile deriye sıfırlanır", "qty": 3},
        {"group": "Dolgu", "item": "Dümen servo kaportası (dikeyin iç yüzü; segment planında değil)",
         "spec": "30 × 10 × 5 mm kabarcık, ≈ 0,4 g: PETG (şablon U_Fairing_Servo_Rudder_L/R ağı, taban deriye "
                 "zımparalanır) ya da 0,5 mm ısıl biçimlendirilmiş levha; dikey rengiyle boyanır, arka ağız açık",
         "qty": 2},
        {"group": "Bağlantı", "item": "Egzoz çıkış borusu geçiş halkası (sol yanak)",
         "spec": str(P.SPEC["propulsion"]["muffler"]["outlet"].get("grommet", "ısıya dayanıklı silikon/seramik keçe")),
         "qty": 1},
        {"group": "Bağlantı", "item": "M4 cıvata + kelebek somun (burun modülü)", "spec": "A2 paslanmaz", "qty": 4},
        {"group": "Bağlantı", "item": "M4 cıvata + somun (G10 köprü–soket–çerçeve, s = 1,55 flanşı)",
         "spec": "12.9; köprüde boru başına 2 adet", "qty": 12},
        {"group": "Bağlantı", "item": "Motor bağlantısı: M4 × 30 + titreşim takozu (kauçuk, Ø10 × 8, 40 Shore A)",
         "spec": f"motor halkasındaki 4 × M4 ısıl gömme dişliye, kare {ENGINE_BOLT_SQ * 1000:.0f} mm", "qty": 4},
        {"group": "Bağlantı", "item": "M4 naylon cıvata (dış panel tutma dili; P9)",
         "spec": "M4 × 16, alttan, kök bloğu 2'deki ısıl gömme dişliye", "qty": 2},
        {"group": "Bağlantı", "item": "Isıl gömme dişli M4",
         "spec": f"Ø{_fmt_g(INSERT['M4'][0] * 1000)} delik × {_fmt_g(INSERT['M4'][1] * 1000)}: motor halkası 4 + tutma "
                 "dili 2",
         "qty": 6},
        {"group": "Bağlantı", "item": "Isıl gömme dişli M3 + M3 × 8 cıvata",
         "spec": f"Ø{_fmt_g(INSERT['M3'][0] * 1000)} delik: ana yatak 2 × 2, burun yatağı 4 (kısa), kaporta/yanak 6",
         "qty": 14},
        {"group": "Bağlantı", "item": "Hizalama pimi (burun modülü)", "spec": f"çelik Ø{NOSE_PIN_D * 1000:.0f}×20 mm",
         "qty": 2},
        {"group": "Bağlantı", "item": "Neodim mıknatıs (aviyonik kapağı; çerçeve 6 + gövde basamağı 6, eş kutup)",
         "spec": f"Ø{MAGNET['d'] * 1000:.0f}×{MAGNET['t'] * 1000:.0f} mm N52, cep Ø{_fmt_g(MAGNET['pocket_d'] * 1000)}"
                 f"×{_fmt_g(MAGNET['pocket_t'] * 1000)}", "qty": 12},
        {"group": "Bağlantı", "item": "MPX konnektör", "spec": "6 pin: kanat paneli ↔ kök bloğu 2 (2), kuyruk "
                                                               "modülü s = 1,55 (1); 8 pin: burun modülü (1)", "qty": 4},
        {"group": "Kapak", "item": "Füme PETG levha (aviyonik kapağı, ısıl biçimlendirme; P10)",
         "spec": f"{_fmt_g(HATCH_SHEET_T * 1000)} mm, ≥ 300 × 450 mm (1 yedek)", "qty": 2},
        {"group": "Servolar", "item": SERVO["name"], "spec": "kanatçık, dış flap (iç flap bağlı), elevatör, dümen — "
                                                            "yan başına 4", "qty": 8},
        {"group": "Servolar", "item": "İtme çubuğu Ø1,6 çelik + çatal (clevis) + kilit", "spec": "her kumanda servosuna 1",
         "qty": 8},
        {"group": "Servolar", "item": "Gaz, jikle, burun yönlendirme servosu", "spec": "mini", "qty": 3},
        {"group": "Servolar", "item": "Kapak servoları + sıralayıcı + fren", "spec": "mikro", "qty": 1},
        {"group": "İniş takımı", "item": P.SPEC["landing_gear"]["product"], "spec": "", "qty": 1},
        {"group": "İniş takımı", "item": "Ana tekerlek",
         "spec": f"Ø{_fmt_g(float(P.SPEC['landing_gear']['main']['wheel_d_m']) * 1000)} mm, frenli", "qty": 2},
        {"group": "İniş takımı", "item": "Burun tekerleği",
         "spec": f"Ø{float(P.SPEC['landing_gear']['nose']['wheel_d_m']) * 1000:.0f} mm", "qty": 1},
        {"group": "İtki", "item": P.SPEC["propulsion"]["engine"]["model"], "spec": "susturucu + CDI dahil", "qty": 1},
        {"group": "İtki", "item": P.SPEC["propulsion"]["prop"]["model"], "spec": "", "qty": 1},
        {"group": "İtki", "item": "Spinner", "spec": f"Al Ø{float(P.SPEC['propulsion']['spinner']['d_m']) * 1000:.0f} mm",
         "qty": 1},
        {"group": "İtki", "item": "Yakıt deposu",
         "spec": f"2 × {_fmt_g(float(P.SPEC['fuselage']['internals']['fuel_tanks']['volume_l_each']))} L",
         "qty": 2},
        {"group": "İtki", "item": "Isı kalkanı", "spec": P.SPEC["propulsion"]["muffler"]["shield"], "qty": 1},
        {"group": "Faydalı yük / aviyonik", "item": P.SPEC["payload"]["turret"]["model"] if "payload" in P.SPEC
         else "EO/IR taret", "spec": "", "qty": 1},
        {"group": "Faydalı yük / aviyonik", "item": "Ana akü 4S2P Molicel P45B", "spec": "130 Wh", "qty": 1},
        {"group": "Faydalı yük / aviyonik", "item": "Uçuş kontrolcüsü, 2× GNSS, telemetri, RC alıcı, LED'ler",
         "spec": "", "qty": 1},
    ]
    return out


def summarize(results: list, bed, plans: dict, ctx: Ctx, verified: list, elapsed: float) -> dict:
    tools = [r for r in results if r.get("tooling")]
    results_all = results
    results = [r for r in results if not r.get("tooling")]          # alet (kalıp) uçak toplamına girmez
    tot_mass = sum(r["mass_total_g"] for r in results)
    tot_h = sum(r["print_h_total"] for r in results)
    n_pieces = sum(r["qty"] for r in results)
    by_mat: dict[str, dict] = {}
    for r in results:
        d = by_mat.setdefault(r["material"], {"pieces": 0, "mass_g": 0.0, "hours": 0.0, "volume_cm3": 0.0})
        d["pieces"] += r["qty"]
        d["mass_g"] += r["mass_total_g"]
        d["hours"] += r["print_h_total"]
        d["volume_cm3"] += r["volume_cm3"] * r["qty"]
    by_group: dict[str, dict] = {}
    for r in results:
        d = by_group.setdefault(r["group"], {"pieces": 0, "mass_g": 0.0, "hours": 0.0})
        d["pieces"] += r["qty"]
        d["mass_g"] += r["mass_total_g"]
        d["hours"] += r["print_h_total"]
    stl_bytes = sum(r.get("stl", {}).get("bytes", 0) for r in results_all)
    n_pins = int(math.ceil(sum(r.get("pin_holes", 0) * r["qty"] for r in results) / 2))
    est = _PR["estimates"]
    return {
        "model": str(P.SPEC["meta"]["name"]), "date": time.strftime("%Y-%m-%d"),
        "bed_mm": list(bed), "bed_margin_mm": BED_MARGIN,
        "solver": {"segment_cut": ctx.cut_solver, "shell_features": ctx.B.solver, "boolean_ops": ctx.B.n_ops,
                   "boolean_s": round(ctx.B.t_ops, 1), "elapsed_s": round(elapsed, 1)},
        "totals": {"unique_parts": len(results), "pieces": n_pieces, "mass_g": round(tot_mass, 0),
                   "print_h": round(tot_h, 1), "stl_mb": round(stl_bytes / 1e6, 2),
                   "stl_zero_area": sum(r.get("stl", {}).get("zero_area", 0) for r in results_all),
                   "stl_degenerate_split": sum(r.get("stl", {}).get("degenerate_split", 0) for r in results_all),
                   "stl_slivers": sum(r.get("stl", {}).get("slivers", 0) for r in results_all),
                   "spec_estimate": {"print_mass_kg": est["print_mass_kg"], "print_hours": est["print_hours"],
                                     "segment_count": est["segment_count"], "panel": dict(est.get("panel", {}))}},
        "by_material": {k: {kk: round(vv, 1) for kk, vv in v.items()} for k, v in by_mat.items()},
        "by_group": {k: {kk: round(vv, 1) for kk, vv in v.items()} for k, v in by_group.items()},
        "all_manifold": all(r["check"]["ok"] for r in results_all),
        "all_fit": all(r["fits"] or r["fits_tight"] for r in results_all),
        "tooling": {"parts": [r["key"] for r in tools], "pieces": sum(r["qty"] for r in tools),
                    "mass_g": round(sum(r["mass_total_g"] for r in tools), 0),
                    "print_h": round(sum(r["print_h_total"] for r in tools), 1)},
        "heat_rule": getattr(ctx, "heat", {"ok": True}),
        "parts": results_all, "verified_stl": verified, "warnings": list(ctx.warnings),
        "hinges": {k: {"lugs": len(v.lugs), "length_mm": round(v.L * 1000, 1),
                       "bore_mm": round(HINGE_PIN_HOLE_D * 1000, 2), "pin": f"Ø{HINGE_PIN_D * 1000:.2f} mm PETG filament",
                       "pin_length_mm": round((v.L + PIN_BORE_EXT.get(k, (-0.006, 0.006))[1]
                                               - PIN_BORE_EXT.get(k, (-0.006, 0.006))[0]) * 1000, 0),
                       "insert_side": PIN_INSERT.get(k, ("", "", ""))[0], "route": PIN_INSERT.get(k, ("", "", ""))[1],
                       "retention": PIN_INSERT.get(k, ("", "", ""))[2]} for k, v in plans.items()},
        "load_paths": LOAD_PATHS,
        "print_cuts": getattr(ctx, "print_cuts", {}),
        "tolerance_coupon": {"stl": "tolerance_coupon.stl",
                             "holes": [{"label": a, "nominal_mm": round(b * 1000, 2), "hole_mm": round(c * 1000, 2)}
                                       for a, b, c in coupon_holes()],
                             "note": "önce kupon basın, gerekirse TUBE_CLEAR ayarlayın"},
        "thermoform": THERMOFORM_STEPS,
        "slicer_groups": SLICER_GROUPS,
        "bom": bom(plans, n_pins),
        "settings": {"rib_mm": RIB_T * 1000, "frame_mm": [FRAME_T * 1000, FRAME_W * 1000],
                     "land_mm": [LAND_T * 1000, LAND_H * 1000], "sleeve_mm": SLEEVE_T * 1000,
                     "web_mm": WEB_T * 1000, "lattice_mm": LATTICE_T * 1000, "tube_clear_mm": TUBE_CLEAR * 1000,
                     "pin_mm": PIN_D * 1000, "hinge_pin_mm": HINGE_PIN_D * 1000},
    }


def _fmt(x, nd=0):
    s = f"{x:,.{nd}f}".replace(",", " ")
    return s.replace(".", ",")


def _fmt_g(x: float) -> str:
    """En çok 2 ondalık, sondaki sıfırlar atılır (0.45 → "0,45", 1.6 → "1,6", 3.0 → "3")."""
    return f"{x:.2f}".rstrip("0").rstrip(".").replace(".", ",")


def report_md(S: dict) -> str:
    """Türkçe baskı raporu (Markdown)."""
    bed = S["bed_mm"]
    T = S["totals"]
    L = []
    L.append(f"# {S['model']} — 3B baskı raporu")
    L.append("")
    L.append(f"Üretim: `ucav/blender/printprep.py` · tarih {S['date']} · tabla {bed[0]:.0f}×{bed[1]:.0f}×{bed[2]:.0f} mm "
             f"(kenar payı {S['bed_margin_mm']:.0f} mm) · boolean: segment kesimi {S['solver']['segment_cut']}, "
             f"kabuk/özellikler {S['solver']['shell_features']} ({S['solver']['boolean_ops']} işlem).")
    L.append("")
    L.append("## Özet")
    L.append("")
    L.append("| | Bu model | Spec planı (kütle bütçesi) | Panel tahmini |")
    L.append("|---|---:|---:|---:|")
    est = T["spec_estimate"]
    pan = est.get("panel") or {}

    def _pn(k, nd, unit=""):
        return f"{_fmt(pan[k], nd)}{unit}" if k in pan else "—"
    L.append(f"| Benzersiz STL | {T['unique_parts']} | — | — |")
    L.append(f"| Basılacak parça (ayna dahil) | {T['pieces']} | {est['segment_count']} | {_pn('segment_count', 0)} |")
    L.append(f"| Basılı kütle | {_fmt(T['mass_g'] / 1000, 2)} kg | {_fmt(est['print_mass_kg'], 2)} kg | "
             f"{_pn('print_mass_kg', 2, ' kg')} |")
    L.append(f"| Baskı süresi (kaba) | {_fmt(T['print_h'], 0)} h | {_fmt(est['print_hours'], 0)} h | "
             f"{_pn('print_hours', 0, ' h')} |")
    stl_txt = f"{_fmt(T['stl_mb'], 1)} MB" if any("stl" in p for p in S["parts"]) else "yazılmadı (--no-stl)"
    L.append(f"| STL toplamı | {stl_txt} | ≤ 40 MB hedef | |")
    L.append(f"| Manifold / kapalı | {'evet, hepsi' if S['all_manifold'] else 'HAYIR — uyarılara bakın'} | | |")
    n_tight = sum(1 for p in S["parts"] if not p["fits"] and p["fits_tight"])
    fit_txt = ("hepsi" + (f" ({n_tight} benzersiz parça dar payla, ≥ {BED_MARGIN_MIN:.0f} mm)" if n_tight else "")
               if S["all_fit"] else "HAYIR — uyarılara bakın")
    L.append(f"| Tablaya sığma | {fit_txt} | | |")
    L.append("")
    L.append("Kütle, filament etkin yoğunluğuyla (LW-PLA köpürmüş 0,65 g/cm³) parça hacminden hesaplanır; boya, "
             "yapıştırıcı ve basılmayan parçalar dahil değildir. Süre: hacimsel hız + katman başına "
             f"{_fmt_g(LAYER_OVERHEAD_S)} s + tabla başına {PLATE_SETUP_H * 60:.0f} dk (kaba tahmin).")
    L.append("")
    L.append("| Malzeme | Parça | Hacim (cm³) | Kütle (g) | Süre (h) |")
    L.append("|---|---:|---:|---:|---:|")
    for k, v in sorted(S["by_material"].items()):
        L.append(f"| {k} | {v['pieces']:.0f} | {_fmt(v['volume_cm3'], 0)} | {_fmt(v['mass_g'], 0)} | {_fmt(v['hours'], 1)} |")
    L.append("")
    L += _mass_table(S)
    # parça tablosu
    L.append("## Parça tablosu")
    L.append("")
    L.append("Ölçüler baskı yönündedir (X × Y tabla izi, Z yükseklik). **Yön**: tabladan yukarı bakan uçak ekseni; "
             "**Z açısı**: tablada izin döndürülmesi (köşegen yerleşim). Ayna (L/R) parçalar tek STL olarak verilir: "
             "sağ eş için dilimleyicide X ya da Y'de aynalayın.")
    groups = []
    for r in S["parts"]:
        if r["group"] not in groups:
            groups.append(r["group"])
    for g in groups:
        L.append("")
        L.append(f"### {g}")
        L.append("")
        L.append("| STL | Parça | Malz. | Et (mm) | Adet | Ölçü (mm) | Yön / Z açısı | Pay (mm) | Kütle (g) | Süre (h) | Destek | Not |")
        L.append("|---|---|---|---:|---:|---|---|---:|---:|---:|---|---|")
        for r in S["parts"]:
            if r["group"] != g:
                continue
            sz = r["size_mm"]
            ori = r["orientation"]
            ori_s = f"{ori['up_words']} / {ori['z_rot_deg']:.0f}°"
            if ori["bed_end"]:
                ori_s += f"; tablada: {ori['bed_end']}"
            note = r["note"]
            if r["mirror"]:
                note = ("sağ = ayna; " + note).strip("; ")
            if not r["fits"] and r["fits_tight"]:
                note = (note + "; dar pay").strip("; ")
            stl = Path(r["stl"]["file"]).name if "stl" in r else f"{r['key']}.stl"
            L.append(f"| `{stl}` | {r['name']} | {r['material']} | {_fmt(r['wall_mm'], 1)} | {r['qty']} | "
                     f"{sz[0]:.0f}×{sz[1]:.0f}×{sz[2]:.0f} | {ori_s} | {r['bed_margin_mm']:.0f} | "
                     f"{_fmt(r['mass_g'], 0)} | {_fmt(r['print_h'], 1)} | {r['support']} | {note} |")
    L.append("")
    # doğrulama
    L.append("## Doğrulama")
    L.append("")
    L.append("Her parça bmesh ile denetlenir: sınır kenar = 0, manifold olmayan kenar = 0, tel kenar = 0, yönü "
             "tutarsız kenar = 0, işaretli hacim > 0. Et ölçümü: yüzeyden içe ışın (alan ağırlıklı 1200 örnek); "
             "%5'lik değer nominal et ya da en ince iç yapıdır (kanat panellerinde "
             f"{_fmt_g(LATTICE_T * 1000)} mm geodezik kafes, gövde halkalarında 0,7 mm deri); en küçük değer köşe/pah örneklerini ve kaplamaların "
             "sıfıra incelen yapışma kenarlarını içerir.")
    L.append("")
    L.append("| STL | Kenar denetimi (yeniden okuma) | Hacim farkı | Blender içe aktarma |")
    L.append("|---|---|---:|---|")
    for v in S["verified_stl"]:
        bi = v.get("blender_import", {})
        bi_s = (f"{bi.get('faces', 0)} yüz, manifold olmayan {bi.get('nonmanifold', '?')}" if "error" not in bi
                else f"hata: {bi['error']}")
        ec = v["edges"]
        ew = ec.get("welded", {})
        L.append(f"| `{v['key']}.stl` | {ec['triangles']} üçgen, eşsiz {ec['unpaired']}, çift {ec['dup_directed']}; "
                 f"0,1 µm kaynakla eşsiz {ew.get('unpaired', '?')}, çift {ew.get('dup_directed', '?')}; "
                 f"sıfır alanlı {ec.get('zero_area', '?')} "
                 f"{'✓' if ec['ok'] else '✗'} | {v['volume_err_pct']:+.3f} % | {bi_s} |")
    L.append("")
    tt = S["totals"]
    if "stl_zero_area" in tt:
        L.append(f"Yazılan tüm STL'lerde sıfır alanlı üçgen (float32 mm, |AB×AC| < {ZERO_AREA_MM2:g} mm²): "
                 f"{tt['stl_zero_area']}. Float32 çözünürlüğünde doğrusal ince üçgen (uzun kenarına yükseklik "
                 f"< {SLIVER_H_MM * 1e6:.0f} nm): {tt.get('stl_slivers', '?')} kaldı; dışa aktarımda komşu üçgeni bölerek "
                 f"giderilen: {tt['stl_degenerate_split']} (yüzey ≤ {SLIVER_H_MM * 1e6:.0f} nm oynar, hacim ve kapalılık "
                 "korunur). Kalanlar art arda doğrusal üçgenlerdir; dilimleyici için etkisizdir.")
        L.append("")
    bad = [r for r in S["parts"] if not r["check"]["ok"]]
    L.append(f"Manifold olmayan parça: {len(bad)}. " + ("" if not bad else ", ".join(r["key"] for r in bad)))
    L.append("")
    L.append("| Parça | Et %5 (mm) | Et medyan (mm) | Çıkıntı (cm²) |")
    L.append("|---|---:|---:|---:|")
    for r in S["parts"]:
        mw = r["min_wall"]
        L.append(f"| {r['key']} | {_fmt(mw.get('p05_mm', 0), 2)} | {_fmt(mw.get('median_mm', 0), 2)} | "
                 f"{_fmt(r['overhang_cm2'], 1)} |")
    if S["warnings"]:
        L.append("")
        L.append("### Uyarılar")
        L.append("")
        for w in S["warnings"]:
            L.append(f"- {w}")
    L.append("")
    L += _report_sections(S)
    L.append("")
    # BOM
    L.append("## Basılmayan parçalar (BOM)")
    L.append("")
    L.append("| Grup | Kalem | Özellik | Adet |")
    L.append("|---|---|---|---:|")
    for b in S["bom"]:
        L.append(f"| {b['group']} | {b['item']} | {b['spec']} | {b['qty']} |")
    L.append("")
    L.append(_ASSEMBLY_MD)
    L.append("")
    L.append(_settings_md(S))
    L.append("")
    L.append("## Kapsam")
    L.append("")
    L.append("Sivil gözetleme/araştırma platformu. Faydalı yük yalnız EO/IR taret; silah, mühimmat, dış yük askısı "
             "ya da bırakma mekanizması yoktur ve eklenmez.")
    L.append("")
    return "\n".join(L)


LOAD_PATHS = [
    {"load": "Kanat kaldırması (yarım kanat ≈ 11,7 kg × n)",
     "path": "dış panel derisi + Ø27 panel borusu → Ø30 merkez soket → G10 köprü (2 × 3 mm, soketin iki yanında "
             "3,3 mm yuvalarda, boru başına 2 × M4) → kanat kutusu çerçeveleri (s 1,20 / 1,29) → 4 longeron + halka 5",
     "parts": "wing_panel_*, root_block_1/2, wing_frame_fwd, wing_frame_aft, fus_ring_5"},
    {"load": "Kanat burulması / hücum açısı",
     "path": "Ø6 dolu CF açı pimi (y 0,334–0,434, %60 veter) + Ø8 arka kiriş → kök bloğu arka ağı → çerçeveler",
     "parts": "wing_panel_1, root_block_2, root_block_1"},
    {"load": "Dış panel eki (eksenel / sökülebilir)",
     "path": "PETG tutma dili (panel 1 kök kaburgası) → M4 naylon cıvata → kök bloğu 2'deki M4 ısıl gömme dişli",
     "parts": "panel_lock_tab, wing_panel_1, root_block_2"},
    {"load": "Ana takım (iniş darbesi, fren)",
     "path": "ER-150 ünitesi → G10 3 mm plaka → ana takım yatağı (PA-CF beşik, 2 × M3) → sokete eyerli kol → Ø30 "
             "soket → G10 köprü → çerçeveler / longeronlar",
     "parts": "gear_mount_L (+ ayna), root_block_2, wing_frame_*"},
    {"load": "Burun takımı",
     "path": "burun ünitesi → G10 plaka (4 × M3 kısa dişli) → burun yatağı (PA-CF, iki chine longeronuna eyer) → "
             "longeronlar → halka 1 çerçevesi",
     "parts": "gear_mount_N, fus_ring_1"},
    {"load": "İtki + titreşim (DLE-20, 1,86 kW, tek silindir)",
     "path": "motor → 4 × M4 + kauçuk takoz → motor halkası (PA-CF, halka 9 içinde 30 mm bindirme, 4 longeron "
             "soketi) → longeronlar; G10 yangın perdesi halkaya yaslanır",
     "parts": "engine_ring, fus_ring_9"},
    {"load": "Kuyruk (stabilize + dikey yükleri)",
     "path": "Ø12 stabilize kirişi + Ø6 arka çubuk → koni içi G10 eyer → kuyruk konisi (LW-ASA) → s = 1,55 G10 "
             "flanşı (4 × M4) → longeronlar",
     "parts": "stab_1/2, fin_1/2, fus_ring_7–9"},
    {"load": "Longeron sürekliliği",
     "path": "düz CF parçalar s = 1,167 ve 1,55 kırıklarında PETG soket bloklarına 18 mm girer (epoksi)",
     "parts": "longeron_joint_chine, longeron_joint_shoulder, fus_ring_4/5"},
]

THERMOFORM_STEPS = [
    "Kalıp: hatch_buck_a + hatch_buck_b (PLA, 0,3 mm katman, 3 çevre, %15 dolgu) Ø3 CF pimlerle birleştirilip "
    "yapıştırılır; üst yüz 240 → 400 kum zımpara, dolgu astarı, 800 kum; kalıp ayırıcı (PVA / mum). Yanlar 2° eğimli, "
    "8 mm kesim payı kalıba dahil.",
    "Levha: 1,0 mm füme PETG 65 °C'de 2 saat kurutulur (kabarcık olmasın), alüminyum çerçeveye sıkıştırılır.",
    "Isıtma: fırında / ısıtıcı altında 140–160 °C, levha 10–20 mm sarkana kadar (≈ 1–2 dk).",
    "Biçimlendirme: kalıp vakum masasına (≥ 350 × 200 mm) konur, levha hızla indirilir, vakum (≥ 0,6 bar) açılır; "
    "soğuyunca (≈ 1 dk) çıkarılır.",
    "Kesim: kalıp kenar çizgisinden makas / Dremel ile kesilir, kenar 400 kumla düzeltilir.",
    "Çerçeve: hatch_frame_a/b (PETG, düz alt yüz tablada) cepleri ile 6 mıknatıs CA ile yapıştırılır (kutup gövde "
    "basamağındaki eşleriyle denenir); çerçeve levhanın iç yüzüne şeffaf epoksi / UV yapıştırıcı ile yapıştırılır "
    "(bölme s = 0,680).",
    "Kontrol: kapak gövde basamağına oturur, dış yüz deriden en çok 1,6 mm yüksek (flush); mıknatıs eşleri 0,3 mm "
    "içinde eş eksenli (aynı HATCH_MAGNETS listesinden).",
]

# R04: önceki yayımlanan baskı raporunun (R2 sonu, 4,06 kg) grup kütleleri ve bu turdaki hafifletme kalemleri —
# raporun "Kütle tablosu" bölümü karşılaştırma için kullanır.
R04_TARGET_KG = 3.5
R04_BASELINE_G = {"Kanat": 1443.6, "Kuyruk": 305.1, "Kumanda yüzeyleri": 365.0, "Gövde": 911.2, "Yük yolları": 355.5,
                  "İtki": 276.8, "Kaplamalar": 246.9, "Faydalı yük": 79.6, "Takım kapakları": 74.9}
R04_LEVERS = {
    "Kanat": "kafes yalnız panel 1–4 (y < 1,19 m), 0,6 mm üye × 90 mm aralık; ana/arka kiriş kovanı yerine 16 mm "
             "bilezikler (≤ 55 mm aralık; boru yükü bileziklerden ve kaburgalardan deriye)",
    "Kuyruk": "dikey kiriş kesme ağı dikey normaline döndürüldü (orta düzlem zarı kalktı); kiriş kovanı → bilezik",
    "Kumanda yüzeyleri": "orta düzlem levhası yerine 0,5 mm dikey menteşe ağı; menteşe dili R 4,5 mm, çentik yalnız ön "
                         "yarıda",
    "Gövde": "halka/modül derisi 0,8 → 0,7 mm + 4 iç stringer (0,8 × 5 mm); çerçeve 8 → 6 mm, flanş 2,4 → 2,0 mm; "
             "longeron kovanı → bilezik; keepout içindeki iç yapı artık gerçekten çıkarılıyor (halka 5); halka 9 "
             "arka flanşı motor halkası bindirmesinde yalnız deri; modülde PETG flanş cebi",
    "Yük yolları": "kanat kutusu çerçevesinde depo üstü hafifletme deliği; kalın PA-CF/PETG parçalar dilimleyici "
                   "dolgusuyla (kabuk + %40 gyroid) hesaplanır; motor halkası NACA çerçevesinden, ana takım beşiği "
                   "iç flap menteşe dilinden arındırıldı",
    "İtki": "kaporta print_solid kaynaktan (Ø92 arka açıklık, çene yarığı, motor zarfı boşlukları, panjur dudakları); "
            "lüle halkası kaportaya alın alına (geçme dili yok); pabuç PA-CF 2 mm, kaporta yüzünü izler",
    "Kaplamalar": "kök kaportası ve ER-150 kabartması 1,6 → 1,2 mm; tüy kenar ve iç ince kuşaklar kırpılır",
    "Faydalı yük": "taret yakası gövde dış yüzünde kesilir: fileto tabanı karın derisine bindirmeli yapışır, tüy "
                   "kenar 0,8 mm'de kırpılır (gömülü üst kutu ve iç dudak basılmaz; modül derisiyle çakışıyordu)",
    "Takım kapakları": "burun kapağı çentik köşesi kıymığı kesildi (fark geometri turundaki kapak değişikliğinden)",
}


SLICER_GROUPS = [
    {"group": "Kanat panelleri, uç kapağı", "parts": "wing_panel_*, wing_tip", "material": "LW-PLA", "nozzle_mm": 0.4,
     "line_mm": "0,6 (kafes 1 × 0,6 — yalnız panel 1–4; deri 0,6 = 1 × 0,6; D-kutu 1,2 = 2 × 0,6)",
     "layer_mm": "0,20 (kafes)–0,25", "walls": "1 (D-kutu 2)", "infill": "%0", "arachne_mm": "min 0,50 / maks 0,70"},
    {"group": "Kök blokları, strake, gövde halkaları", "parts": "root_block_*, glove_strake, fus_*", "material": "LW-PLA",
     "nozzle_mm": 0.6, "line_mm": "0,7 (gövde derisi 0,7 = 1 çizgi + 0,8 × 5 mm stringer; kök bloğu 0,8; flanş 2,0 = "
     "3 × 0,67)", "layer_mm": "0,25–0,30", "walls": "1", "infill": "%0", "arachne_mm": "min 0,55 / maks 0,95"},
    {"group": "Kuyruk konisi ve ısı bölgesi", "parts": "fus_ring_7–9, stab_1, elevator_1 (spec print.zones."
     "stab_root_heat.parts)", "material": "LW-ASA", "nozzle_mm": 0.4,
     "line_mm": "0,5 (1,0 = 2 × 0,5; stabilize/elevatör derisi 0,5 = 1 çizgi)", "layer_mm": "0,20–0,25", "walls": "2",
     "infill": "%0", "arachne_mm": "min 0,40 / maks 0,60"},
    {"group": "Kuyruk yüzeyleri, kumanda yüzeyleri", "parts": "stab_*, fin_*, aileron_*, flap*, elevator_*, rudder_*",
     "material": "LW-PLA", "nozzle_mm": 0.4, "line_mm": "0,5 (deri 0,5 = 1 çizgi; menteşe ağı 0,5)",
     "layer_mm": "0,20", "walls": "1", "infill": "%0", "arachne_mm": "min 0,40 / maks 0,60"},
    {"group": "PA-CF kaplama ve yük yolu parçaları", "parts": "cowl_* (panjur dudakları sağ yanakta), exhaust_ring, scuff_pad, intake, "
     "root_fairing, gear_blister, wing_frame_*, gear_mount_*, engine_ring", "material": "PA-CF", "nozzle_mm": 0.4,
     "line_mm": "0,53 (1,6 = 3 × 0,53; kök kaportası 1,2 = 2 × 0,6; 3 mm çerçeve 6 çizgi)", "layer_mm": "0,20",
     "walls": "3", "infill": "%100 (≤ 3,2 mm), %40 gyroid (daha kalın; kütle modeli buna göre)",
     "arachne_mm": "min 0,40 / maks 0,65"},
    {"group": "PETG parçalar", "parts": "door_*, hatch_frame_*, fus_nose_flange, turret_collar, panel_lock_tab, "
     "longeron_joint_*", "material": "PETG", "nozzle_mm": 0.4, "line_mm": "0,40 (1,2 = 3 × 0,4; 1,6 = 4 × 0,4)",
     "layer_mm": "0,20", "walls": "3–4", "infill": "%100 (ince), %40 gyroid (blok)", "arachne_mm": "min 0,34 / maks 0,50"},
    {"group": "Alet: kalıp + kupon", "parts": "hatch_buck_*, tolerance_coupon", "material": "PLA / LW-PLA",
     "nozzle_mm": 0.6, "line_mm": "0,65", "layer_mm": "0,30", "walls": "3", "infill": "%15", "arachne_mm": "—"},
]


def _mass_table(S: dict) -> list[str]:
    """R04 kütle tablosu: grup başına önceki rapor (R2 sonu) ↔ bu model, hafifletme kalemleri ve dolgu modeli payı."""
    T = S["totals"]
    bg = S.get("by_group", {})
    solid = sum(float(p.get("mass_solid_g", p["mass_g"])) * int(p["qty"]) for p in S["parts"] if p["group"] in bg)
    fill_gain = solid - sum(float(p["mass_g"]) * int(p["qty"]) for p in S["parts"] if p["group"] in bg)
    L = [f"## Kütle tablosu (R04: hedef ≤ {_fmt(R04_TARGET_KG, 1)} kg)", "",
         f"Basılı gövde kütlesi **{_fmt(T['mass_g'] / 1000, 2)} kg** "
         f"({'hedefin altında' if T['mass_g'] <= R04_TARGET_KG * 1000 else 'HEDEFİN ÜSTÜNDE'}; önceki rapor "
         f"{_fmt(sum(R04_BASELINE_G.values()) / 1000, 2)} kg). Kalın PA-CF/PETG parçalar (ortalama et > kabuk) "
         f"dilimleyicideki gibi kabuk + %40 gyroid dolgu ile hesaplanır (PA-CF 1,6 mm, PETG 1,2 mm kabuk); dolu "
         f"basılsalardı +{_fmt(fill_gain, 0)} g. Kanat, gövde ve kuyruk dayanımı: kiriş/longeron yükü bilezik ve "
         "kaburgalarla deriye geçer, ince deriler stringer ve kafesle bölünür; kök blokları, yük yolu çerçeveleri ve "
         "menteşe hatları inceltilmedi. Spec kütle bütçesinin \"Basılı:\" kalemleri (`mass.breakdown`) bu tablodaki "
         "grup kütleleridir; `sizing.py --check` toplamı ve her grubu bu raporla karşılaştırır (≤ 5 g).", "",
         "| Grup | Önceki rapor (g) | Bu model (g) | Fark (g) | Ne değişti |", "|---|---:|---:|---:|---|"]
    tot0 = tot1 = 0.0
    for g in list(R04_BASELINE_G) + [g for g in bg if g not in R04_BASELINE_G]:
        m0 = R04_BASELINE_G.get(g)
        m1 = float(bg.get(g, {}).get("mass_g", 0.0))
        tot0 += m0 or 0.0
        tot1 += m1
        L.append(f"| {g} | {_fmt(m0, 0) if m0 is not None else '—'} | {_fmt(m1, 0)} | "
                 f"{_fmt(m1 - m0, 0) if m0 is not None else '—'} | {R04_LEVERS.get(g, '')} |")
    L.append(f"| **Toplam** | **{_fmt(tot0, 0)}** | **{_fmt(tot1, 0)}** | **{_fmt(tot1 - tot0, 0)}** | |")
    L.append("")
    return L


def _fmt_st(v: float) -> str:
    """İstasyon (m): 3–4 ondalık, sondaki sıfırlar atılır (0.2325 → "0,2325", 0.4 → "0,4")."""
    return f"{float(v):.4f}".rstrip("0").rstrip(".").replace(".", ",")


def _report_sections(S: dict) -> list[str]:
    """Yük yolları, ısı kuralı, menteşe pimleri, tolerans kuponu, ısıl biçimlendirme, kesim istasyonları, aletler."""
    L = ["## Yük yolları (P4)", "", "| Yük | Yol | Basılı parçalar |", "|---|---|---|"]
    for r in S.get("load_paths", []):
        L.append(f"| {r['load']} | {r['path']} | {r['parts']} |")
    hr = S.get("heat_rule", {})
    tr = hr.get("tg_rule", {"tg_min_c": 120, "radius_mm": 50})
    L += ["", "## Isı kuralı (AERO-09, R05)", ""]
    L.append(f"Kural 1: silindir (+ buji başlığı) ve susturucu zarflarının ve susturucu çıkış borusu/ısı kalkanının "
             f"{_fmt(hr.get('radius_mm', 150), 0)} mm yakınında {hr.get('no_material', 'LW-PLA')} yok. Kural 2: ısıl "
             f"sınırı {_fmt(tr['tg_min_c'], 0)} °C'nin altındaki hiçbir filament (LW-ASA 95, PETG 80, LW-PLA 55 °C) "
             f"bu kaynaklara {_fmt(tr['radius_mm'], 0)} mm'den yakın değil (zarflar `params.engine_envelope`, çıkış "
             "`U_Exhaust_Muffler`). Isıl sınır amorf filamentlerde camsı geçiş (Tg), yarı kristal PA-CF'de ısıl eğilme "
             "sıcaklığıdır (HDT, 0,45 MPa; PA-CF'nin Tg'si ≈ 60 °C). Kural 3: hiçbir parça susturucu çıkış borusuna "
             f"{_fmt(hr.get('pipe_rule', {}).get('clear_mm', 5), 0)} mm'den yakın değil. "
             "Printprep malzemeyi kendiliğinden değiştirmez: ihlal varsa `build.py --print` ve "
             "`printprep` 1 ile çıkar. Isı bölgesindeki parçaların filamenti spec'te adıyla atanır "
             f"(`print.zones[].parts`). Sonuç: **{'uygun' if hr.get('ok', True) else 'İHLAL'}**.")
    L.append("")
    outlet = P.muffler_outlet().params
    L.append("Kaynaklar ayrı adlandırılır: motor zarfı parçaları, susturucu çıkış borusu ve borunun çıktığı yerde sol "
             "yanak dış yüzüne (0,1–0,5 mm aralıkla) oturan 0,5 mm Al ısı kalkanı (kalkan yanağı borudan korur; yanak "
             f"PA-CF, HDT ≈ 150 °C). Boru yanaktaki deliğinden {_fmt_g(float(outlet.get('hole_clear_m', 0.0015)) * 1000)} "
             "mm radyal boşlukla geçer; delikteki geçiş halkası boruyu ortalar ve sıcak gazı yanağa değdirmez (BOM: "
             f"{outlet.get('grommet') or 'ısıya dayanıklı silikon/seramik keçe halka'}). Boru ve kalkan uzaklıkları kaynak "
             "yüzeyinin yoğun örneklerinden parça yüzeyine, zarf uzaklıkları parça köşelerinden ölçülür; 10 mm'nin "
             "altındakiler 0,1 mm çözünürlükle verilir; son sütun parçanın en yakın üç kaynağıdır.")
    if hr.get("assigned"):
        L.append("")
        L.append("Spec ataması: " + "; ".join(
            f"`{x['key']}` → {x['material']} ({x['zone']}; kaynağa {_fmt(x['min_mm'], 0)} mm)" for x in hr["assigned"]))
    if hr.get("violations"):
        L.append("")
        L.append("**İhlaller:** " + "; ".join(
            f"`{x['key']}` {x['material']} {_fmt(x['min_mm'], 0)} mm ({HEAT_SRC_TR.get(x['source'], x['source'])}; "
            f"{x.get('rule', '')}; çözüm: "
            f"{x.get('fix', '')})" for x in hr["violations"]))
    if hr.get("near"):
        def _mm(v):
            return _fmt(v, 1 if float(v) < 10.0 else 0)
        L += ["", "| Parça (250 mm içinde) | Malzeme | Isıl sınır (°C) | En yakın (mm) | Kaynak | Kaynaklara uzaklık (mm) |",
              "|---|---|---:|---:|---|---|"]
        for x in hr["near"][:16]:
            per = "; ".join(f"{HEAT_SRC_TR.get(k, k)} {_mm(v)}" for k, v in (x.get("by_source_mm") or {}).items())
            L.append(f"| {x['key']} | {x['material']} | {_fmt(x.get('tg_c', tg_c(x['material'])), 0)} | "
                     f"{_mm(x['min_mm'])} | {HEAT_SRC_TR.get(x['source'], x['source'])} | {per} |")
    L += ["", "## Menteşe pimleri (P1)", "",
          f"Pim: Ø{_fmt_g(HINGE_PIN_D * 1000)} mm PETG filament (çelik tel yerine; esnek, paslanmaz, kesilip ısıyla "
          f"ucu mantarlanabilir). Delik Ø{_fmt_g(HINGE_PIN_HOLE_D * 1000)} mm çevrel çokgen (iç yarıçap "
          f"{_fmt_g(HINGE_PIN_HOLE_D * 500)} mm). Her yüzeyin tek bir düz takma yolu vardır; delik takma tarafında "
          "komşu parçadan dışarı uzatılmıştır.", "",
          "| Yüzey | Dil | Pim boyu (mm) | Takma ucu | Yol | Tutma |", "|---|---:|---:|---|---|---|"]
    for k, h in S.get("hinges", {}).items():
        L.append(f"| {k} | {h['lugs']} | {_fmt(h.get('pin_length_mm', h['length_mm']), 0)} | {h.get('insert_side', '')} | "
                 f"{h.get('route', '')} | {h.get('retention', '')} |")
    tc = S.get("tolerance_coupon")
    if tc:
        L += ["", "## Tolerans kuponu (P7)", "",
              f"**Önce kupon basın, gerekirse TUBE_CLEAR ayarlayın.** `{tc['stl']}`: 20 mm LW-PLA blok, delikler "
              "segmentlerdeki gibi dik basılır ve parçalarla aynı payı taşır (çevrel çokgen, kenar ≈ 0,6 mm). Boru "
              "elle zorlanmadan geçmeli, 0,3 mm'den fazla boşluk kalmamalı; geçmiyorsa `HOLE_CLEAR` (LW-PLA boru "
              f"{_fmt_g(TUBE_CLEAR * 1000)} mm) artırılıp yeniden üretin.", "",
              "| Delik | Nominal (mm) | Basılan (mm) |", "|---|---:|---:|"]
        for hh in tc["holes"]:
            L.append(f"| {hh['label']} | {_fmt_g(hh['nominal_mm'])} | {_fmt_g(hh['hole_mm'])} |")
    if S.get("thermoform"):
        L += ["", "## Aviyonik kapağı — ısıl biçimlendirme (P10)", "",
              "FDM PETG füme cam görünümü vermez (çok çizgili, buğulu); kapak 1,0 mm füme PETG levhadan basılı "
              "kalıp üstünde vakumla biçimlendirilir (RC kanopi yöntemi). Kalıp uçak kütlesine girmez.", ""]
        L += [f"{i + 1}. {t}" for i, t in enumerate(S["thermoform"])]
    pc = S.get("print_cuts")
    if pc:
        L += ["", "## Baskı kesim istasyonları = panel çizgileri (P13)", "",
              f"Segment eklerinde dış deri kenarına {_fmt_g(pc['groove_mm'][0])} × {_fmt_g(pc['groove_mm'][1])} mm pah: "
              "komşu iki parça birleşince V-oluk (panel çizgisi) oluşur. Render panel çizgileri bu istasyonlarda olmalı "
              "(JSON `print_cuts`).", "",
              "- kanat y (m): " + ", ".join(_fmt_st(v) for v in pc["wing_y_m"]) + f"; strake ayrımı s = {_fmt_st(pc['strake_s_m'])}",
              "- gövde s (m): " + ", ".join(_fmt_st(v) for v in pc["fuselage_s_m"]),
              f"- stabilize eki y = {_fmt_st(pc['stab_y_m'])} m (ok ekseni boyunca), dikey eki h = {_fmt_st(pc['fin_h_m'])} m",
              "- kumanda yüzeyi ekleri y (m): " + "; ".join(f"{k} " + ", ".join(_fmt_st(v) for v in vv)
                                                           for k, vv in pc["control_surface_y_m"].items() if vv)]
    tl = S.get("tooling", {})
    if tl.get("parts"):
        L += ["", "## Alet ve test parçaları", "",
              f"Uçak toplamına girmez: {', '.join('`' + k + '.stl`' for k in tl['parts'])} — {tl['pieces']} parça, "
              f"≈ {_fmt(tl['mass_g'], 0)} g, ≈ {_fmt(tl['print_h'], 1)} h (kalıp %{TOOLING_INFILL * 100:.0f} dolgu "
              "eşdeğeri)."]
    return L


_ASSEMBLY_MD = """## Montaj sırası

0. **Tolerans kuponu**: `tolerance_coupon.stl` ilk basılır; CF borular, pimler ve Ø1,75 PETG menteşe pimi deliklerde
   denenir (bkz. Tolerans kuponu). Gerekirse `HOLE_CLEAR` ayarlanıp STL'ler yeniden üretilir.
1. **Kanat dış paneli** (her yan): segmentleri kökten uca dizin; her ekte alttaki segmentin kaburgası üstteki
   segmentin 2,4 mm flanşına oturur. Ø3 CF hizalama pimlerini hücum kenarı göbeklerine yapıştırın, 27/25 panel
   borusunu ve 8/6 arka kirişi kılavuz kovanlarından geçirerek kuru montajla hizayı kontrol edin; sonra ekleri
   ince CA / 5 dk epoksi ile sırayla yapıştırın. Uç borusu (16/14) ve kademeli burç 1,00–1,10'da panel borusuna
   girer. Servo kablolarını Ø8 kablo kanalından (ana kirişin arkasında; kaburga ve kafes delikleri hizalı) panel 1
   kök kaburgasındaki MPX 6 pin cebine çekin. PETG tutma dilini panel 1 kök kaburgasına (açı piminin arkası) yapıştırın.
2. **Dış flap, uç kapağı, kanatçık** (menteşe pimleri, P1):
   a. Dış flap segmentlerini çift kaburga yüzlerinden (pim delikleri hizalı) yapıştırın; flabı oyuğa yerleştirip
      Ø1,75 PETG pimi **iç uçtan**, panel 1'in y = 0,36 kök kaburgasındaki delikten sürün (dış uçta kör delikte durur).
   b. Uç kapağını seyrüsefer LED'i ve kablosuyla eğik uç düzlemine yapıştırın.
   c. Kanatçığı oyuğa yerleştirip pimi **dış uçtan**, uç kapağının dış yüzündeki Ø2,1 delikten sürün (iç uçta kör
      delikte durur); dış deliği CA + mikrobalonla tıkayın (sökmek için tıkaç delinir).
   G10 hornları yarıklarına epoksiyle yapıştırın; servoları yuvalarına takıp itme çubuğu + çatalla bağlayın.
3. **Kök blokları, iç flap, ana takım**: kök bloğu 1 ile glove strake'i pimlerle birleştirin, kök bloğu 2'yi
   flanşından yapıştırın; Ø30 soketi, iç takviyeyi ve açı pimi kovanını epoksiyle sabitleyin. İç flap: Ø3/2,1 pirinç
   bağlayıcı soketini flap içine epoksileyin; iç flabı oyuğa yerleştirip pimi **dış uçtan**, kök bloğu 2'nin
   y = 0,36 kaburgasındaki delikten sürün (panel sökülüyken). İç flabın servosu yoktur: Ø2 bağlayıcı tel panel
   takılırken dış flabın soketine girer (P9). Ana takım yatağını (gear_mount_L/R) kök bloğu 2'nin açık iç ucundan
   kaydırıp sokete eyerleyin; ER-150 ünitesini G10 plakasıyla yatağa 2 × M3 (ısıl gömme dişli) bağlayın.
4. **Gövde ve kanat kutusu** (P4): halkaları çerçeve (tabla tarafı) → flanş (üst) sırasıyla dizin. Longeronlar 3'er
   düz CF parçadır: chine ve omuz kovanlarından geçirip s = 1,167 ve 1,55 kırıklarında PETG soket bloklarına
   (longeron_joint_*) epoksiyle birleştirin. Kanat kutusu çerçevelerini (wing_frame_fwd/aft, PA-CF) halka 5'teki
   yuvalarına yerleştirin; G10 köprü plakalarını (2 × 3 × 40 × 300 mm) çerçeve ve kök bloğu yuvalarına epoksiyle
   oturtun, boru başına 2 × M4 ile plaka–soket–plaka sıkın. s = 1,55'e G10 flanşı yapıştırın. Burun takım yatağını
   (gear_mount_N) halka 1'in ön ucundan kaydırıp iki chine longeronuna eyerleyin; burun ünitesi 4 × M3.
5. **Kaplamalar**: kök kaportasını (kano) ve ER-150 kabartmasını yapıştırın; ≥ 0,8 mm'de kırpılmış kenarları ve
   kanat üstü filetoyu epoksi + mikrobalonla deriye sıfırlayın.
6. **Burun modülü**: burun konisi + modül halkasını yapıştırın; PETG flanşı modül halkasının flanşına yapıştırın;
   taret yakasını takın. Modül 2 × Ø4 pim + 4 × M4 kelebek somunla halka 1'e bağlanır (sökülebilir; MPX 8 pin).
7. **Kuyruk**: stabilize yarılarını 12/10 kiriş ve 6/4 arka çubukla koni içindeki G10 eyerde birleştirin; kök
   filetosunu epoksi + mikrobalonla doldurun. **Dikeyler takılmadan önce** elevatörü oyuğa yerleştirip pimi
   **dış uçtan**, stabilize ucundaki delikten sürün (dikey takılınca elevatör pimi sökülemez). Dikeyleri stabilize
   ucuna ve 8/6 dikey kirişine geçirin. Dümeni oyuğa yerleştirip pimi **üst uçtan**, dikey 2 tepesindeki Ø2,1
   delikten sürün; üst deliği CA ile tıkayın. Kuyruk servo kabloları Ø6 kanaldan (stabilize ağları → kök duvarı →
   koni) s = 1,55 MPX 6 pine.
8. **Motor bölümü**: motor halkasını (engine_ring) halka 9 içine 30 mm bindirerek yapıştırın (4 longeron ucu
   soketlere girer); G10 yangın perdesini halkaya yaslayın. DLE-20'yi 4 × M4 + kauçuk takozla halkadaki ısıl gömme
   dişlilere bağlayın, ısı kalkanını takın; PA-CF kaportayı 6 × M3 ile, yanakları flanşlarından vidalayın; lüle
   halkasını kaporta önüne, NACA dudak çerçevesini halka 9'daki cebe, PA-CF sürtünme pabucunu kaporta altına
   yüksek sıcaklık epoksisiyle yapıştırın (aşınınca sökülüp yenilenir).
9. **Kapaklar**: takım kapaklarını (PETG; menteşe dilleri ve horn baskıda dahil) menteşelerine takın, burun tapa
   kapağını (door_N_3) bacak dirseğine bağlayın. Aviyonik kapağını (ısıl biçimlendirilmiş levha + PETG çerçeve)
   mıknatıslarla oturtun.
10. **Saha montajı**: dış paneli boruya geçirip MPX'i takın, tutma dilini alttan M4 naylon cıvatayla kök bloğu 2'ye
   bağlayın. Ağırlık merkezini s = 1,232 m'de (aralık 1,219–1,245) doğrulayın."""


def _settings_md(S: dict) -> str:
    """Yazıcı ayarları (P14): grup başına dilimleyici tablosu + malzeme süreçleri + destek + ısı/boya kuralı."""
    st = S["settings"]
    L = ["## Yazıcı ayarları", "",
         "Çizgi genişlikleri modeldeki etlerle (spec `print.walls_mm`) uyumludur: her et, tablodaki çizgi genişliğinin "
         "tam katıdır ya da Arachne aralığına tek çizgi olarak sığar.", "",
         "| Grup | Parçalar | Malzeme | Nozul (mm) | Çizgi (mm) | Katman (mm) | Çevre | Dolgu | Arachne duvar genişliği |",
         "|---|---|---|---:|---|---|---|---|---|"]
    for g in S.get("slicer_groups", SLICER_GROUPS):
        L.append(f"| {g['group']} | {g['parts']} | {g['material']} | {_fmt_g(g['nozzle_mm'])} | {g['line_mm']} | "
                 f"{g['layer_mm']} | {g['walls']} | {g['infill']} | {g['arachne_mm']} |")
    L += ["",
          "**LW-PLA** (colorFabb, köpüren): 230–245 °C, akış %55–60, 40–60 mm/s, fan %30–50, tabla 50–60 °C, geri çekme "
          "0,5–1 mm (\"geri çekmede sil\" açık), seyir \"çevreleri geçme\", dikiş firar kenarında hizalı. İnce duvar "
          "algılama (Arachne / \"Print Thin Walls\") açık, boşluk dolgusu kapalı; kaburga "
          f"{_fmt_g(st['rib_mm'])} mm ve çerçeveler dolu basılır. Segmentler dik (açıklık / gövde ekseni Z'de) basılır: "
          "kaburga ya da çerçeve tablada, flanş üstte.",
          "**LW-ASA**: 250–265 °C, akış %55–65, kapalı kabin, tabla 95–105 °C, fan %0–20.",
          "**PA-CF**: sertleştirilmiş nozul, 270–290 °C, kurutulmuş filament (80 °C 6 h), kapalı kabin, fan %0–20.",
          "**PETG**: 235–245 °C, fan %30–50, tabla 70–80 °C.",
          "**PLA kalıp**: 0,6 mm nozul, 0,3 mm katman, 3 çevre, %15 dolgu; üst yüz zımpara + astar (ısıl "
          "biçimlendirme bölümü).", "",
          "**Destek** (P6): parça tablosundaki \"Destek\" sütunu bağlayıcıdır — \"tabla desteği\": seçilen yönde "
          f"45°'den dik, altında model olmayan çıkıntı > {_fmt_g(SUPPORT_MIN_CM2)} cm²; \"model üstü destek\": 5 mm'den "
          f"yüksekten modele inen çıkıntı > {_fmt_g(MODEL_SUPPORT_MIN_CM2)} cm²; \"yok\". Ağaç (tree) destek, arayüz "
          "boşluğu 0,2 mm önerilir. Tabla teması 3 cm²'den ya da izin %3'ünden azsa STL'ye 1 mm kırılabilir baskı ayağı "
          "eklenmiştir (Not sütunu); baskıdan sonra kesilir. Kanat ve gövde segmentlerindeki çıkıntılar: menteşe dili "
          "kamaları, kuyu tavanları, kapak/kuyu açıklığı kenarları ve kapalı uç kapakları.", "",
          "**Isı / boya kuralı** (P14): LW-PLA Tg ≈ 55 °C; güneşte koyu üst deri 65–75 °C'ye ısınır. Üst yüz boyasının "
          "güneş yansıtması ≥ 0,5 olmalı (açık gri ya da IR-yansıtıcı \"cool\" pigmentli gri). **\"taktik\" (koyu) "
          "livery yalnız render içindir**: gerçek uçakta koyu üst yüz kullanılacaksa üst deriler (kanat, gövde üstü, "
          "kuyruk) LW-ASA'ya (Tg 95 °C) geçirilmelidir. Isı kuralı bölgesindeki parçalar zaten LW-ASA / PA-CF.", "",
          f"**Montaj payları** (P7): boru deliği Ø + {_fmt_g(HOLE_CLEAR['lw']['tube'] * 1000)} mm (LW-PLA/LW-ASA; "
          f"PETG/PA-CF + {_fmt_g(HOLE_CLEAR['hard']['tube'] * 1000)} mm), pim deliği Ø + "
          f"{_fmt_g(HOLE_CLEAR['lw']['pin'] * 1000)} mm, M4 Ø{_fmt_g(M4_HOLE_D * 1000)}, M3 Ø{_fmt_g(M3_HOLE_D * 1000)}, "
          f"ısıl gömme dişli M3 Ø{_fmt_g(INSERT['M3'][0] * 1000)} / M4 Ø{_fmt_g(INSERT['M4'][0] * 1000)} mm; bütün "
          "delikler çevrel çokgen (iç yarıçap = delik yarıçapı, kenar ≈ 0,6 mm). Kovan eti "
          f"{_fmt_g(st['sleeve_mm'])} mm, kesme ağı {_fmt_g(st['web_mm'])} mm, geodezik kafes "
          f"{_fmt_g(st['lattice_mm'])} mm (±{LATTICE_DEG:.0f}°, {LATTICE_PITCH * 1000:.0f} mm aralık, yalnız iç dört "
          f"panelde; dik baskıda yatayla {90 - LATTICE_DEG:.0f}°), hizalama pimi Ø{_fmt_g(st['pin_mm'])} mm, menteşe pimi Ø{_fmt_g(st['hinge_pin_mm'])} "
          f"mm, çerçeve {_fmt(st['frame_mm'][0], 1)} × {_fmt(st['frame_mm'][1], 0)} mm, flanş "
          f"{_fmt(st['land_mm'][0], 1)} × {_fmt(st['land_mm'][1], 0)} mm."]
    return "\n".join(L)


# =====================================================================================================
# Önizleme (patlatılmış) — düşük çözünürlük Cycles
# =====================================================================================================
PREVIEW_COLOR = {"LW-PLA": (0.80, 0.80, 0.77), "LW-ASA": (0.62, 0.58, 0.50), "PA-CF": (0.09, 0.09, 0.10),
                 "PETG": (0.85, 0.32, 0.06), "PETG-füme": (0.12, 0.16, 0.20), "TPU": (0.03, 0.03, 0.03),
                 "PLA": (0.20, 0.45, 0.75)}


def render_exploded_preview(filepath: str | Path, res=(640, 400), samples: int = 16, explode: float = 1.0,
                            view: str = "iso") -> str:
    """``UCAV_Print`` parçalarını patlatılmış hâlde ayrı geçici sahnede render eder (Cycles CPU, düşük örnek).
    Renkler filamente göre (nesne düzeyinde geçici malzeme; ``UM_*`` malzemelerine dokunulmaz)."""
    src_col = bpy.data.collections.get(PRINT_COL)
    if src_col is None:
        raise RuntimeError("UCAV_Print yok — önce build_print_parts()")
    sc = bpy.data.scenes.new("UCAV_PrintPreview")
    sc.render.engine = "CYCLES"
    sc.cycles.device = "CPU"
    sc.cycles.samples = samples
    sc.cycles.use_denoising = True
    sc.render.resolution_x, sc.render.resolution_y = res
    sc.view_settings.view_transform = "AgX"
    world = bpy.data.worlds.new("PP_World")
    world.use_nodes = True
    bg = world.node_tree.nodes.get("Background")
    bg.inputs[0].default_value = (0.035, 0.04, 0.05, 1.0)
    bg.inputs[1].default_value = 1.0
    sc.world = world
    col = bpy.data.collections.new("PP_PreviewCol")
    sc.collection.children.link(col)
    mats = {}
    for k, rgb in PREVIEW_COLOR.items():
        m = bpy.data.materials.new(f"PP_{k}")
        m.use_nodes = True
        bsdf = m.node_tree.nodes.get("Principled BSDF")
        bsdf.inputs["Base Color"].default_value = (*rgb, 1.0)
        bsdf.inputs["Roughness"].default_value = 0.55 if k != "PETG-füme" else 0.15
        mats[k] = m
    tmp = []
    for ob in src_col.objects:
        if not ob.name.startswith("UP_") or ob.get("ucav_print_tooling"):    # kalıp / kupon uçak görünümünde değil
            continue
        o2 = bpy.data.objects.new(ob.name + "_pv", ob.data)
        ex = np.asarray(ob.get("ucav_explode", (0, 0, 0)), float) * explode
        o2.location = Vector(tuple(ex))
        col.objects.link(o2)
        if o2.material_slots:
            o2.material_slots[0].link = "OBJECT"
            o2.material_slots[0].material = mats.get(str(ob.get("ucav_print_material", "LW-PLA")), mats["LW-PLA"])
        tmp.append(o2)
    pts = []
    for o in tmp:
        Vo, _ = _mesh_np(o.data)
        pts.append(Vo[::25] + np.array(o.location))
    V = np.vstack(pts)
    c = 0.5 * (V.min(0) + V.max(0))
    ext = float(np.linalg.norm(V.max(0) - V.min(0)))
    cam_data = bpy.data.cameras.new("PP_Cam")
    cam_data.lens = 50
    cam = bpy.data.objects.new("PP_Cam", cam_data)
    d = {"iso": np.array([0.62, 0.78, 0.62]), "top": np.array([0.0, 0.05, 1.0]),
         "side": np.array([0.05, 1.0, 0.15])}.get(view, np.array([0.62, 0.78, 0.62]))
    d = _unit(d)
    cam.location = Vector(tuple(c + d * ext * 1.25))
    cam.rotation_euler = (Vector(tuple(c)) - cam.location).to_track_quat("-Z", "Y").to_euler()
    sc.collection.objects.link(cam)
    sc.camera = cam
    for i, (dd, en, ang) in enumerate((((0.35, -0.45, 1.0), 3.2, 3.0), ((-0.7, 0.6, 0.35), 1.0, 10.0),
                                       ((0.2, 0.9, -0.5), 0.5, 20.0))):
        ld = bpy.data.lights.new(f"PP_Sun{i}", "SUN")
        ld.energy = en
        ld.angle = math.radians(ang)
        lo = bpy.data.objects.new(f"PP_Sun{i}", ld)
        lo.rotation_euler = Vector(tuple(-np.asarray(dd))).to_track_quat("-Z", "Y").to_euler()
        sc.collection.objects.link(lo)
    sc.render.filepath = str(filepath)
    sc.render.image_settings.file_format = "PNG"
    bpy.ops.render.render(write_still=True, scene=sc.name)
    for o in list(sc.objects):
        data = o.data
        bpy.data.objects.remove(o, do_unlink=True)
        if isinstance(data, bpy.types.Camera):
            bpy.data.cameras.remove(data)
        elif isinstance(data, bpy.types.Light):
            bpy.data.lights.remove(data)
    for m in mats.values():
        bpy.data.materials.remove(m)
    bpy.data.collections.remove(col)
    bpy.data.worlds.remove(world)
    bpy.data.scenes.remove(sc)
    return str(filepath)


# =====================================================================================================
# Komut satırı
# =====================================================================================================
def main(argv: Sequence[str] | None = None) -> int:
    """``python3 -m ucav.blender.printprep [--bed X Y Z] [--out DİZİN] [--only ÖNEK…] [--no-stl] [--preview PNG]``"""
    ap = argparse.ArgumentParser(prog="printprep", description="YELKOVAN YK-38 baskı hazırlığı: segmentler, STL (mm), "
                                 "baskı raporu (Türkçe).")
    ap.add_argument("--bed", nargs=3, type=float, default=list(BED_DEFAULT), metavar=("X", "Y", "Z"),
                    help="tabla ölçüsü mm (varsayılan 256 256 256; 220 220 250 de desteklenir)")
    ap.add_argument("--out", default=None, help="çıktı kökü (stl/, print_report.md/.json)")
    ap.add_argument("--only", nargs="*", default=None, help="yalnız bu anahtar önekleri (ör. wing_panel fus_ring)")
    ap.add_argument("--no-stl", action="store_true", help="STL yazma (yalnız rapor)")
    ap.add_argument("--no-report", action="store_true", help="rapor yazma")
    ap.add_argument("--preview", default=None, help="patlatılmış önizleme PNG yolu")
    ap.add_argument("--blend", default=None, help="sonuç sahnesini .blend olarak kaydet")
    ap.add_argument("--solver", default="MANIFOLD", choices=["MANIFOLD", "EXACT"],
                    help="kabuk/özellik boolean çözücüsü (segment kesimi her zaman EXACT)")
    ap.add_argument("-q", "--quiet", action="store_true")
    a = ap.parse_args(argv)
    S = build_print_parts(tuple(a.bed), a.out, only=a.only, export=not a.no_stl, write_report=not a.no_report,
                          solver=a.solver, verbose=not a.quiet)
    T = S["totals"]
    print(f"{T['unique_parts']} benzersiz STL, {T['pieces']} parça, {T['mass_g'] / 1000:.2f} kg, ≈{T['print_h']:.0f} h, "
          f"{T['stl_mb']:.1f} MB; manifold: {'evet' if S['all_manifold'] else 'HAYIR'}; "
          f"sığma: {'evet' if S['all_fit'] else 'HAYIR'}")
    for w in S["warnings"]:
        print("UYARI:", w)
    if a.preview:
        render_exploded_preview(a.preview)
        print("önizleme:", a.preview)
    if a.blend:
        bpy.ops.wm.save_as_mainfile(filepath=str(Path(a.blend).resolve()))
    heat_ok = S.get("heat_rule", {}).get("ok", True)
    if not heat_ok:
        print("HATA: ısı kuralı ihlali (çıkış 1):", ", ".join(f"{v['key']} [{v['rule']}]"
                                                         for v in S["heat_rule"].get("violations", [])))
    return 0 if (S["all_manifold"] and S["all_fit"] and heat_ok) else 1


if __name__ == "__main__":                                      # pragma: no cover
    sys.exit(main())
