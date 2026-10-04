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
   ``boşluk −= iç yapılar`` (kılavuz kovanları, kesme ağları, ±45° geodezik kafes, pim göbekleri, flanşlar),
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
yakası (PETG), TPU sürtünme pabucu. Kanat üstü ve stabilize kök filetoları sıfıra inen kama olduğundan basılmaz
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
FRAME_W = 0.008                                     # entegre halka çerçeve genişliği (deriden içe). varsayım
LAND_T = 0.0024                                     # üst uç yapıştırma flanşı toplam et (deri dahil). varsayım
LAND_H = 0.006                                      # flanş yüksekliği. varsayım
SLEEVE_T = 0.0008                                   # boru kılavuz kovanı eti (2 çizgi). varsayım
SHOULDER = 0.002                                    # kovan kademesi / kör uç kapağı kalınlığı. varsayım
WEB_T = 0.0008                                      # kesme ağı (kiriş gövdesi) eti. varsayım
LATTICE_T = _WALLS["lattice"]                       # ±45° geodezik iç kafes (0,45 mm)
LATTICE_PITCH = 0.070                               # kafes aralığı (spec yorumu "≈ 70 mm")
TUBE_CLEAR = 0.0004                                 # CF boru deliği çap boşluğu (Ø + 0,4 mm). varsayım
PIN_D = 0.003                                       # hizalama pimi Ø3 CF çubuk. varsayım
PIN_HOLE_D = PIN_D + 0.0002
PIN_DEPTH = 0.010                                   # pim deliği derinliği (her iki yanda). varsayım
BOSS_D = 0.0075                                     # pim göbeği çapı. varsayım
HINGE_PIN_D = float(P.SPEC["wing"]["control_surfaces"]["hinge_pin_d_m"])
HINGE_PIN_HOLE_D = HINGE_PIN_D + 0.0001
HINGE_SLEEVE_R = 0.5 * HINGE_PIN_HOLE_D + _WALLS["hinge_knuckles"]
LUG_W = 0.005                                       # basılı menteşe dili genişliği (eksen boyunca). varsayım
LUG_CLEAR = 0.0005                                  # dil–çentik eksenel boşluk (her yanda). varsayım
LUG_PITCH = 0.090                                   # dil aralığı (en fazla). varsayım
HINGE_ZONE_EXTRA = 0.0012                           # menteşe oyuğu dolu bölgesi: r_oyuk + et + 1,2 mm (dudak kaması). varsayım
HORN_SLOT = (0.0018, 0.014)                         # G10 horn yarığı (kalınlık, veter boyu). varsayım
SERVO = {"name": "10 mm ince kanat servosu (KST X10 sınıfı)", "open": (0.036, 0.024), "box_t": 0.0012}   # varsayım
LIGHT_MARGIN = 0.0028                               # hafifletme deliği ile kenar/engel arası en az et. varsayım
LIGHT_R = (0.003, 0.022)                            # hafifletme deliği yarıçap aralığı. varsayım
M4_HOLE_D = 0.0043
M3_HOLE_D = 0.0033
NOSE_PIN_D = 0.004                                  # burun modülü hizalama pimi (çelik Ø4). varsayım
CYL_SEG = 28                                        # silindir kenar sayısı (STL boyutu / pürüzsüzlük dengesi)
SMALL_ISLAND_M3 = 2e-9                              # 2 mm³'ten küçük boolean kırıntıları atılır
FRAGMENT_REL = 0.06                                 # ana kabuğun %6'sından küçük kopuk dolu kabuk atılır
THIN_COVER_FACTOR = 2.5                             # kaplama ortalama kalınlığı ≤ 2,5 × et ise dolu basılır. varsayım
GLUE_GAP = 0.0002                                   # kaplama/fileto ile gövde/kanat arası yapıştırma boşluğu. varsayım
VOXEL_REPAIR_MAX_M3 = 80e-6                         # manifold olmayan ≤ 80 cm³ parçaya son çare voksel onarım
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
}
LAYER_OVERHEAD_S = 2.5                              # katman başına seyir/geri çekme/Z hareketi (s). varsayım
PLATE_SETUP_H = 0.10                                # tabla başına ısınma/kalibrasyon (h). varsayım

# Basılı parçaların sahne malzemesi (sözleşme: params.MATERIALS adları; UCAV_Print render dışı)
FILAMENT_MATERIAL = {"LW-PLA": "UM_SkinBottom", "LW-ASA": "UM_SkinTop", "PA-CF": "UM_PACF", "PETG": "UM_Accent",
                     "TPU": "UM_TPU", "PETG-füme": "UM_SmokeHatch"}


def _zone(name: str) -> tuple[str, float]:
    """Spec bölgesi → (filament, et m)."""
    z = _ZONES[name]
    return str(z["material"]), float(z["wall_mm"]) / 1000.0


def density(material: str) -> float:
    """Filament etkin yoğunluğu (g/cm³; LW-PLA köpürmüş etkin değer)."""
    key = "PETG" if material.startswith("PETG") else material
    return float(_FIL[key]["density_g_cm3"])


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

    def op(self, target: bpy.types.Object, operand, operation: str, self_: bool = False,
           solver: str | None = None) -> bpy.types.Object:
        t0 = time.time()
        mod = target.modifiers.new("pp_bool", "BOOLEAN")
        mod.operation = operation
        mod.solver = solver or self.solver
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
        return target

    def safe(self, target: bpy.types.Object, operand, operation: str, first: str, second: str,
             min_volume: float = 0.0) -> bpy.types.Object:
        """Önce ``first`` çözücüyle dener; sonuç boş/manifold değil/beklenenden küçükse ``second`` ile yeniden."""
        backup = target.data.copy()
        self.op(target, operand, operation, solver=first)
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
    drop = [(v, sh) for v, sh in shells if 0.0 < v < rel * vmax]
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


def triangulate(me: bpy.types.Mesh) -> tuple[np.ndarray, np.ndarray]:
    """Doğru (içbükey n-gen güvenli) üçgenleme: bmesh ``triangulate`` (ear-clip) → (V, T)."""
    bm = bmesh.new()
    bm.from_mesh(me)
    bmesh.ops.triangulate(bm, faces=bm.faces[:], quad_method="BEAUTY", ngon_method="EAR_CLIP")
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
    return {"min_mm": float(d.min()), "p05_mm": float(np.percentile(d, 5)), "median_mm": float(np.median(d))}


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
    adds: list = field(default_factory=list)        # parçaya eklenir (menteşe dilleri)
    notes: list = field(default_factory=list)

    def extend(self, other: "Feature") -> "Feature":
        self.structure += other.structure
        self.holes += other.holes
        self.adds += other.adds
        self.notes += other.notes
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


def placement(V: np.ndarray, T: np.ndarray, up, bed: tuple[float, float, float], margin: float = BED_MARGIN) -> dict:
    """Parçayı ``up`` yukarı olacak biçimde çevirir, tabanı Z = 0'a, izi tabla merkezine koyar.
    Dönüş: 4×4 dönüşüm (m → m, tabla merkezli), ölçüler (mm), çıkıntı alanı (cm²), tabla teması (cm²)."""
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
    on_bed = (nz < -0.999) & (zc < 2e-5)
    over = (nz < -math.sin(math.radians(OVERHANG_DEG))) & ~on_bed
    fits_z = height <= bed[2] - Z_MARGIN
    return {"M": M, "up": tuple(float(x) for x in _unit(up)), "angle_deg": fit["angle_deg"],
            "size_mm": (fit["w_mm"], fit["h_mm"], height), "margin_mm": fit["margin_mm"],
            "fits": bool(fit["fits"] and fits_z), "fits_tight": bool(fit["fits_tight"] and fits_z),
            "overhang_cm2": float(ar[over].sum() * 1e4), "bed_contact_cm2": float(ar[on_bed].sum() * 1e4)}


def choose_orientation(V: np.ndarray, T: np.ndarray, bed, preferred: Sequence = (), margin: float = BED_MARGIN,
                       prefer_slack_cm2: float = 3.0) -> dict:
    """Aday yönler: tercih edilenler (ör. tablaya oturan kaburga), OBB eksenleri (±). Önce sığanlar; sonra en az
    çıkıntı; tercih edilen yön, en iyiden en çok ``prefer_slack_cm2`` kötüyse seçilir."""
    C = V - V.mean(0)
    _, _, Vt = np.linalg.svd(C[:: max(1, len(C) // 4000)], full_matrices=False)
    cands = [(_unit(u), True) for u in preferred if u is not None]
    for ax in Vt:
        cands += [(_unit(ax), False), (_unit(-ax), False)]
    for ax in np.eye(3):                                # dünya eksenleri (PCA eksenleri kabuk kırpılınca kayar)
        cands += [(ax.copy(), False), (-ax, False)]
    res = []
    for u, pref in cands:
        pl = placement(V, T, u, bed, margin)
        pl["preferred"] = pref
        res.append(pl)

    def key(p):                                         # sığma → tabla teması (≥ 1 cm²) → çıkıntı → yükseklik
        return (0 if p["fits"] else (1 if p["fits_tight"] else 2), 0 if p["bed_contact_cm2"] >= 1.0 else 1,
                p["overhang_cm2"] - (0.5 if p["preferred"] else 0), p["size_mm"][2])

    res.sort(key=key)
    best = res[0]
    for p in res:
        if p["preferred"] and (p["fits"] or not best["fits"]) and (p["fits_tight"] or not best["fits_tight"]):
            if p["overhang_cm2"] <= best["overhang_cm2"] + prefer_slack_cm2:
                return p
            break
    return best


# =====================================================================================================
# STL (ikili, mm)
# =====================================================================================================
def write_stl(path: Path, V: np.ndarray, T: np.ndarray, name: str = "") -> int:
    """İkili STL yazar (V metre → mm). Dosya boyutunu (bayt) döndürür."""
    Vm = np.asarray(V, np.float64) * 1000.0
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
    """Gövde: s < 1,55 LW-PLA 0,8 mm; kuyruk konisi (s ≥ 1,55) LW-ASA 1,0 mm (geçiş flanş düzleminde)."""
    s = -V[:, 0]
    w0 = _zone("fuselage_mid")[1]
    w1 = _zone("tail_cone")[1]
    s_f = float(P.SPEC["fuselage"]["modules"]["flange_s_m"])
    return w0 + (w1 - w0) * _smooth((s - (s_f - 0.002)) / 0.004)


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
    def add_source(self, key: str, names: Sequence[str], wall_fn: Callable, solid_fn: Callable | None = None) -> bool:
        missing = [n for n in names if n not in bpy.data.objects]
        if missing and solid_fn is None:
            self.warnings.append(f"kaynak eksik: {', '.join(missing)} ({key} atlandı)")
            return False
        self.sources[key] = Source(key, list(names), wall_fn, solid_fn)
        return True

    def has(self, key: str) -> bool:
        return key in self.sources

    def outer(self, key: str) -> bpy.types.Object:
        """Kaynağın dış katısı (dünya, dinlenme pozu; birden çok nesne → EXACT birleşim)."""
        if key in self._outer:
            return self._outer[key]
        src = self.sources[key]
        solids = [src.solid_fn()] if src.solid_fn else [source_solid(n) for n in src.names]
        ob = self.obj(f"out_{key}", solids[0])
        for s in solids[1:]:
            o2 = self.obj(f"outx_{key}", s)
            self.B.op(ob, o2, "UNION", solver=self.cut_solver)
            _delete(o2)
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
                       [x.transformed(M) for x in feat.adds], list(feat.notes))
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
    for k, pad, _ in walled:
        B.op(base, ctx.grown(k, pad), "DIFFERENCE")
    A = ctx.copy(base, "A")
    if seg.ends:
        reg = ctx.obj("reg", region_solid(_planes(seg.ends), center))
        B.op(A, reg, "INTERSECT", solver=ctx.cut_solver)
        _delete(reg)
    for cut in seg.cutouts:                             # açıklıklar yalnız parçadan (kenar flanşı et bölgesinden gelir)
        co = ctx.obj("cut", cut)
        B.op(A, co, "DIFFERENCE", solver=ctx.cut_solver)
        _delete(co)
    for k, pad, _ in plain:                             # temiz sahne katıları: EXACT, olmazsa MANIFOLD (doğrulamalı)
        B.safe(A, ctx.grown(k, pad), "DIFFERENCE", ctx.cut_solver, B.solver, min_volume=1e-9)
    if len(A.data.polygons) == 0:
        _delete(A)
        _delete(base)
        return None
    inner = None
    shell = seg.shell
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
        for e in seg.ends:
            if e.kind == "plate" and e.holes:
                feat.holes += rib_holes(ctx, seg, e, feat, inner)
        col = B.group(ctx.name("st"), feat.structure)
        if col is not None:
            B.op(cav, col, "DIFFERENCE")
            Booler.drop(col)
        B.op(A, cav, "DIFFERENCE")
        _delete(cav)
    _delete(base)
    for k in seg.union_extra:
        if ctx.has(k):
            B.op(A, ctx.outer(k), "UNION")
    col = B.group(ctx.name("add"), feat.adds)
    if col is not None:
        B.op(A, col, "UNION")
        Booler.drop(col)
    col = B.group(ctx.name("hole"), feat.holes)
    if col is not None:
        B.op(A, col, "DIFFERENCE")
        Booler.drop(col)
    cl = clean_mesh(A, triangles=True)
    me = A.data
    ctx.work.objects.unlink(A)
    out_col.objects.link(A)
    A.name = f"UP_{seg.key}"
    me.name = A.name
    chk = mesh_check(me)
    chk["removed_islands"] = cl["removed_islands"]
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


def tube_prims(p0, p1, od: float, *, sleeve: bool = True, clear: float = TUBE_CLEAR, ext: float = 0.0005,
               web_up=None, web_h: float = 0.08, name: str = "tube") -> Feature:
    """CF boru kanalı: delik (Ø od + pay) + kılavuz kovanı (iç yapı) + isteğe bağlı kesme ağı (``web_up`` yönünde)."""
    p0, p1 = np.asarray(p0, float), np.asarray(p1, float)
    d = _unit(p1 - p0)
    a, b = p0 - d * ext, p1 + d * ext
    r = 0.5 * (od + clear)
    f = Feature()
    f.holes.append(prim_frustum(a, b, r, name=f"{name}_hole"))
    if sleeve:                                          # kovan uçları deliği 2 mm aşar: kademe omzu / kör uç kapağı
        f.structure.append(prim_frustum(a - d * SHOULDER, b + d * SHOULDER, r + SLEEVE_T, name=f"{name}_sleeve"))
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
        f.structure.append(prim_frustum(q - inward * 0.003, q + inward * (PIN_DEPTH + 0.006), 0.5 * d, n=20,
                                        name="boss"))
        f.holes.append(prim_frustum(q + e.n * 0.002, q - e.n * PIN_DEPTH, 0.5 * PIN_HOLE_D, n=16, name="pin"))
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


def hinge_lug_prims(hp: HingePlan, u: float) -> list:
    """Sabit parçaya eklenen dil: eksen çevresinde disk (r_burun − 0,6) + öne köprü + baskı yönünde 45° destek kaması."""
    h = hp.h
    rn = hp.r_nose(u)
    rl = rn - 0.0006
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
    """Yüzeyde dil çentiği: eksen çevresinde silindir (r_burun + 0,8) — sabit tarafın kama yönünde uzatılmış —
    ve yüzeyin baskı yukarı ucunda 45° koni tavan."""
    h = hp.h
    rn = hp.r_nose(u)
    rl = rn - 0.0006
    rc = rn + h.gap_m
    rnotch = rn + 0.0008
    c = hp.point(u)
    X = hp.X
    sgn = hp.lug_up.get(u, 1.0)
    run = rl + rc + 0.004
    lo = -0.5 * LUG_W - LUG_CLEAR - (run if sgn > 0 else 0.0)
    hi = 0.5 * LUG_W + LUG_CLEAR + (run if sgn < 0 else 0.0)
    out = [prim_frustum(c + X * lo, c + X * hi, rnotch, n=24, name="notch")]
    top = (hi if surf_up_sign > 0 else lo) - surf_up_sign * 0.0003
    out.append(prim_frustum(c + X * top, c + X * (top + surf_up_sign * (rnotch + 0.0003)), rnotch + 0.0001, 0.0,
                            n=23, name="notch_cone"))
    return out


def hinge_pin_bore(hp: HingePlan) -> Solid:
    return prim_frustum(hp.a - hp.X * 0.006, hp.a + hp.X * (hp.L + 0.006), 0.5 * HINGE_PIN_HOLE_D, n=12, name="pin_bore")


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
    """Kanat segmenti (sol): boru kanalları + kovanlar + ağlar, arka kiriş, açı pimi, ±45° kafes, pim göbekleri,
    servo yuvası, menteşe bölgeleri/dilleri/pim deliği, firar kenarı dolu şeridi."""
    f = Feature()
    lo, hi = y0 - 0.001, y1 + 0.001
    Zb = np.array([0.0, 0.0, 1.0])
    # ana kiriş
    for name, a, b, p0, p1, od in _spar_pieces():
        ya, yb = max(a, lo), min(b, hi)
        if yb <= ya:
            continue
        f.extend(tube_prims(_line_at(p0, p1, ya), _line_at(p0, p1, yb), od, name=name))
    s0, s1 = _spar_line()
    f.structure.append(web_prim(_line_at(s0, s1, y0 - 0.03), _line_at(s0, s1, y1 + 0.03), Zb))
    # arka kiriş / kök arka ağı
    r0, r1 = _rear_line()
    if part == "outer":
        ya, yb = max(0.38, lo), min(1.82, hi)
        if yb > ya:
            f.extend(tube_prims(_line_at(r0, r1, ya), _line_at(r0, r1, yb), 0.008, name="rear"))
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
        f.extend(tube_prims(_line_at(a, b, ya), _line_at(a, b, yb), ip.od, clear=0.0002, name="inc_pin"))
    # ±45° geodezik kafes (dış panel): ana ağ ile arka ağ arasında
    if part == "outer" and y1 <= P.wing_parts()["U_WingOuter"][1] + 0.01:
        for k in range(-3, 40):
            yk = 0.36 + k * LATTICE_PITCH
            for sg in (1.0, -1.0):
                sa = float(-s0[0]) + 0.004
                sb = float(-_line_at(r0, r1, yk)[0]) - 0.004
                dy = sg * (sb - sa)
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
        f.holes.append(hinge_pin_bore(hp))
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
    ctx.add_source("cowl", ["U_Cowl"], _wall_const(_WALLS["cowl_pa_cf"]))
    ctx.add_source("exhaust_ring", ["U_ExhaustRing"], _wall_const(_WALLS["cowl_pa_cf"]))
    ctx.add_source("louvers", ["U_Cowl_Louvers"], _wall_const(_WALLS["cowl_pa_cf"]))
    ctx.add_source("intake", ["U_Intake"], _wall_const(_wall_of("intake")))
    ctx.add_source("root_fairing", ["U_Fairing_Root_L", "U_Fairing_Blister_L"], _wall_const(_wall_of("root_fairing")))
    ctx.add_source("collar", ["U_Turret_Mount"], _wall_const(_wall_of("turret_collar")))
    ctx.add_source("hatch", ["U_Hatch"], _wall_const(_WALLS["hatch_petg"]))
    ctx.add_source("scuff", ["U_ScuffPad"], _wall_const(_WALLS["tpu_pad"]))
    for d in ("N_1", "L_1", "L_2"):
        ctx.add_source(f"door_{d}", [f"U_Door_{d}"], _wall_const(_wall_of("gear_doors")))
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
        f.holes.append(hinge_pin_bore(hp))
        f.structure.append(_te_band_stab(0.0, P.STAB_HALF_SPAN + 0.006, _WALLS["tail_skin"]))
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
                          subtract=(fus_sub if idx == 1 else []), center=Bp(2.05, 0.13 if idx == 1 else 0.39, 0.065),
                          note=("kök yüzü kuyruk konisi konturunu izler; borular koni içindeki eyerde birleşir"
                                if idx == 1 else "uç dikey içine 5 mm gömülür"))
            recipes.append(Recipe(seg, lambda seg, idx=idx: stab_ff(seg, idx)))

    def fin_ff(seg, idx):
        f = Feature()
        T = {t.name: t for t in P.spar_tubes(SIDE)}
        fs = T["fin_spar"]
        a, b = Bp(*fs.p0), Bp(*fs.p1)
        d = _unit(b - a)
        f.extend(tube_prims(a - d * 0.018, b, 0.008, web_up=np.array([1.0, 0, 0]), web_h=0.05, name="fin_spar"))
        hp = plans["Rudder"]
        f.structure.append(hinge_zone(hp.h, _WALLS["tail_skin"]))
        for u in hp.lugs:
            q = hp.point(u)
            if all(float((q - e.p) @ e.n) < -0.004 for e in seg.ends):
                f.adds += hinge_lug_prims(hp, u)
        f.holes.append(hinge_pin_bore(hp))
        f.structure.append(_te_band_fin(0.0, 0.32, _WALLS["tail_skin"]))
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
        f.structure.append(prim_frustum(a, b, HINGE_SLEEVE_R, n=16, name="pin_sleeve"))
        cm = 0.4 * max(h.chord_in, h.chord_out)        # orta ağ veterin ön %40'ı (pim kovanını deriye bağlar)
        f.structure.append(prim_box(0.5 * (a + b) + hp.Y * 0.5 * cm, np.column_stack([hp.X, hp.Y, hp.Z]),
                                    (0.5 * float(np.linalg.norm(b - a)), 0.5 * cm, 0.5 * WEB_T), name="mid_web"))
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
        st = SERVO_STATIONS.get(nm)
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
            seg = SegSpec(key, f"{label} {k + 1}/{nseg}" if nseg > 1 else label, group, [f"surf_{nm}"],
                          "control_surface", ends, mirror=True, bed_end=(0 if k > 0 else None),
                          up_hint=hp.X, center=hp.point(0.5 * (bounds[k] + bounds[k + 1])), force_up=True,
                          note="menteşe ekseni Z'de; Ø1,5 çelik pim, basılı dil çentikleri")
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
        seg = SegSpec(key, name_tr, "Gövde", ["fus"], zone, ends, bed_end=bed, center=Bp(0.5 * (s0 + s1), 0.0, 0.05),
                      force_up=True,
                      note="")
        recipes.append(Recipe(seg, lambda seg, s0=s0, s1=s1: fuselage_features(ctx, seg, s0, s1)))

    if want("fus_nose_flange") or want("fus"):
        seg = SegSpec("fus_nose_flange", "Burun modülü PETG bağlantı flanşı (4×M4, 2 pim)", "Gövde", ["fus_clean"],
                      "frames", [], material="PETG", wall=0.003, shell=False, bed_end=None,
                      up_hint=np.array([-1.0, 0, 0]))
        recipes.append(Recipe(seg, None))

    # ------------------------------------------------------------------ kaporta, lüle, hava alığı, kapaklar
    cw = P.SPEC["propulsion"]["cowl"]
    boxes = {}
    for sd, sg in (("L", 1.0), ("R", -1.0)):
        c = cw["cheek_left" if sd == "L" else "cheek_right"]
        s_a, s_b, z_a, z_b = float(c["s_from_m"]), float(c["s_to_m"]), float(c["z_from_m"]), float(c["z_to_m"])
        boxes[sd] = (s_a, s_b, z_a, z_b, sg)

    def cheek_box(sd, grow=0.0):
        s_a, s_b, z_a, z_b, sg = boxes[sd]
        lo = Bp(s_b + grow, 0.0 if sg > 0 else -0.2, z_a - grow)
        hi = Bp(s_a - grow, 0.2 if sg > 0 else 0.0, z_b + grow)
        lo2, hi2 = np.minimum(lo, hi), np.maximum(lo, hi)
        return prim_box(0.5 * (lo2 + hi2), np.eye(3), 0.5 * (hi2 - lo2), name="cheek_box")

    def cheek_zones(sd):
        s_a, s_b, z_a, z_b, sg = boxes[sd]
        lim = (Bp(s_b + 0.012, -0.2, z_a - 0.012) * 0 + np.array([-(s_b + 0.012), -0.2 if sg < 0 else -0.001, z_a - 0.012]),
               np.array([-(s_a - 0.012), 0.001 if sg < 0 else 0.2, z_b + 0.012]))
        fw = 0.006
        return [WallZone(Bp(s_a, 0, 0), np.array([1.0, 0, 0]), FRAME_T, 0.0002, extra=fw, box=lim),
                WallZone(Bp(s_b, 0, 0), np.array([-1.0, 0, 0]), FRAME_T, 0.0002, extra=fw, box=lim),
                WallZone(Bp(0, 0, z_a), np.array([0, 0, -1.0]), FRAME_T, 0.0002, extra=fw, box=lim),
                WallZone(Bp(0, 0, z_b), np.array([0, 0, 1.0]), FRAME_T, 0.0002, extra=fw, box=lim)]

    if want("cowl"):
        front = End(*_splane(s_fw + CAP_INSET, 1.0), "frame", width=0.008, label="yangın perdesi")
        seg = SegSpec("cowl_top", "Motor kaportası (PA-CF; yanak açıklıkları flanşlı)", "İtki", ["cowl"], "cowl",
                      [front], bed_end=0, cutouts=[cheek_box("L"), cheek_box("R")],
                      wall_zones=cheek_zones("L") + cheek_zones("R"), center=Bp(2.13, 0, 0.13))
        recipes.append(Recipe(seg, lambda seg: cowl_features(ctx, seg)))
        for sd, nm in (("L", "Sol yanak (susturucu tarafı)"), ("R", "Sağ yanak (panjurlu)")):
            s_a, s_b, z_a, z_b, sg = boxes[sd]
            ends = [End(*_splane(s_a, 1.0), "frame", width=0.006), End(*_splane(s_b, -1.0), "frame", width=0.006),
                    End(Bp(0, 0, z_a), np.array([0, 0, -1.0]), "frame", width=0.006),
                    End(Bp(0, 0, z_b), np.array([0, 0, 1.0]), "frame", width=0.006),
                    End(Bp(0, 0, 0), np.array([0, -sg, 0.0]), "open")]
            seg = SegSpec(f"cowl_cheek_{sd}", f"Kaporta {nm.lower()}", "İtki", ["cowl"], "cowl", ends,
                          union_extra=(["louvers"] if sd == "R" else []), bed_end=None,
                          up_hint=np.array([0, sg, 0.0]), center=Bp(0.5 * (s_a + s_b), sg * 0.05, 0.5 * (z_a + z_b)))
            recipes.append(Recipe(seg, None))
        seg = SegSpec("exhaust_ring", "Lüle halkası (PA-CF)", "İtki", ["exhaust_ring"], "exhaust_ring", [],
                      bed_end=None, up_hint=np.array([1.0, 0, 0]))
        recipes.append(Recipe(seg, None))
        seg = SegSpec("scuff_pad", "Kaporta altı sürtünme pabucu (TPU)", "İtki", ["scuff"], "cowl", [], material="TPU",
                      wall=_WALLS["tpu_pad"], shell=False, up_hint=np.array([0, 0, -1.0]))
        recipes.append(Recipe(seg, None))
    if want("intake"):
        seg = SegSpec("intake", "Sırt hava alığı (PA-CF)", "İtki", ["intake"], "intake", [],
                      subtract=[("fus_clean", GLUE_GAP, 0.0)], cover=True, up_hint=np.array([0, 0, 1.0]))
        recipes.append(Recipe(seg, lambda seg: Feature(holes=[intake_duct_cut()])))
    # kök kaportası, filetolar (açık kapak kabukları)
    if want("root_fairing"):
        seg = SegSpec("root_fairing", "Kök kaportası + ER-150 kabartması (PA-CF)", "Kaplamalar", ["root_fairing"],
                      "root_fairing", [], mirror=True, subtract=[("wing_center_clean", GLUE_GAP, 0.0), ("fus_clean", GLUE_GAP, 0.0)],
                      cover=True, up_hint=np.array([0, 0, -1.0]))
        recipes.append(Recipe(seg, None))
    # Kanat üstü fileto basılmaz: kanat–gövde arasında sıfıra inen kama (%5 et ≈ 0,06 mm) → BOM: dolgu fileto.
    # Stabilize kök filetosu basılmaz: stabilize/koni arasında 0,1–3 mm kama (baskıya uygun değil) → BOM'da
    # epoksi + mikrobalon dolgu (ısı bölgesinde LW-PLA yok kuralı korunur).
    if want("turret_collar"):
        seg = SegSpec("turret_collar", "Taret yakası (8 faset, PETG)", "Faydalı yük", ["collar"], "turret_collar", [],
                      up_hint=np.array([0, 0, 1.0]))
        recipes.append(Recipe(seg, None))
    if want("hatch"):
        seg = SegSpec("hatch", "Aviyonik kapağı (füme PETG, mıknatıslı)", "Kaplamalar", ["hatch"], "hatch", [],
                      material="PETG-füme", shell=False, up_hint=np.array([0, 0, 1.0]))
        recipes.append(Recipe(seg, None))
    for d, nm, mir in (("N_1", "Burun takımı kapağı", True), ("L_1", "Ana takım kuyu kapağı", True),
                       ("L_2", "Ana takım bacak kapağı", True)):
        key = f"door_{d}"
        if want(key) or want("door"):
            seg = SegSpec(key, f"{nm} (PETG 1,2 mm)", "Takım kapakları", [key], "gear_doors", [], shell=False,
                          mirror=mir, up_hint=np.array([0, 0, -1.0]))
            recipes.append(Recipe(seg, None))
    return recipes, plans


def intake_duct_cut() -> Solid:
    """Hava alığı kanal tabanından kuyruk konisine açılan hava geçişi (alık ve gövde halkası birlikte delinir)."""
    I = P.intake_spec()
    s0 = float(I["s_from_m"])
    w = 0.5 * float(I["mouth_w_m"]) - 0.006
    zt = float(I["mouth_center"][2]) - 0.004
    zb = float(I["skin_z_at_mouth"]) - 0.030
    lo, hi = Bp(s0 + 0.030, -w, zb), Bp(s0 + 0.006, w, zt)
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
                f.extend(tube_prims(p0, p1, float(L["od_m"]), name=f"longeron_{kind}_{sd}"))
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
    # kanat kutusu (G10 köprü + Ø30 soket) geçişi
    if s0 < P.WING_SPAR_S < s1:
        T = {t.name: t for t in P.spar_tubes(SIDE)}
        zc = Bp(*T["centre_socket"].p0)[2]
        g10 = P.SPEC["print"]["g10_bridge"]
        hw = 0.5 * (0.030 + 2 * float(g10["t_mm"]) / 1000.0) + 0.0015
        hh = 0.5 * float(g10["h_mm"]) / 1000.0 + 0.0015
        f.holes.append(prim_box(Bp(P.WING_SPAR_S, 0, zc + 0.003), np.eye(3), (hw, 0.25, hh + 0.003), name="wing_box"))
        f.notes.append("kanat kutusu geçişi (G10 köprü + Ø30 soket)")
    # aviyonik kapak erişim açıklığı
    hs = P.SPEC["details"]["hatch"]
    if s0 < float(hs["s_to_m"]) and s1 > float(hs["s_from_m"]):
        f.holes.append(_hatch_opening())
    # burun modülü bağlantısı (ring 1 ön çerçevesi)
    s_nm = float(P.SPEC["fuselage"]["modules"]["nose_module_s_m"][1])
    if abs(s0 - s_nm) < 1e-6 or abs(s1 - s_nm) < 1e-6:
        bolts, pins = nose_flange_bolts()
        for (y, z), d in [(b, M4_HOLE_D) for b in bolts] + [(p, NOSE_PIN_D + 0.0002) for p in pins]:
            f.holes.append(prim_frustum(Bp(s_nm - 0.01, y, z), Bp(s_nm + 0.012, y, z), 0.5 * d, n=16, name="nose_bolt"))
        f.notes.append("burun modülü: 4×M4 kelebek + 2×Ø4 pim")
    # G10 flanş (s = 1,55): kuyruk konisi ön çerçevesinde 4×M4
    s_fl = float(P.SPEC["fuselage"]["modules"]["flange_s_m"])
    if abs(s0 - s_fl) < 1e-6:
        avoid = [(_longeron_at(k, sd, s_fl)[1], _longeron_at(k, sd, s_fl)[2]) for k in ("chine", "shoulder")
                 for sd in ("L", "R")]
        for y, z in _ring_points(s_fl, _wall_of("tail_cone") + 0.005, [40, 140, 220, 320], avoid, 0.012):
            f.holes.append(prim_frustum(Bp(s_fl - 0.01, y, z), Bp(s_fl + 0.012, y, z), 0.5 * M4_HOLE_D, n=16,
                                        name="g10_bolt"))
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
            f.holes.append(prim_frustum(a - d * 0.01, a + d * 0.12, 0.5 * (od + TUBE_CLEAR), n=20, name="stab_tube"))
            hit = None
            bvh2 = BVHTree.FromPolygons(host.V.tolist(), [list(f_) for f_ in host.F], epsilon=0.0)
            r = bvh2.ray_cast(Vector(a + d * 0.001), Vector(d), 0.2)
            if r[0] is not None:
                hit = np.array(r[0])
                f.structure.append(prim_frustum(hit - d * 0.008, hit + d * 0.002, 0.5 * (od + TUBE_CLEAR) + 0.003,
                                                n=20, name="stab_boss"))
    # hava alığı geçişi
    I = P.intake_spec()
    if s0 < float(I["s_from_m"]) + 0.03 and s1 > float(I["s_from_m"]):
        f.holes.append(intake_duct_cut())
    return f


def cowl_features(ctx: Ctx, seg: SegSpec) -> Feature:
    """Kaporta: göbek/krank geçişi Ø40 + 8×Ø12 soğutma çıkışı (lüle boşluğu tabanı), yangın perdesi 6×M3."""
    f = Feature()
    pr = P.PROP
    hub = np.asarray(pr.hub_b)
    ax = np.asarray(pr.axis_aft_b)
    u, v, _ = _perp_frame(ax)
    a, b = hub - ax * 0.10, hub + ax * 0.01
    f.holes.append(prim_frustum(a, b, 0.020, n=36, name="hub_hole"))
    for k in range(8):
        t = 2 * math.pi * (k + 0.5) / 8
        off = (math.cos(t) * u + math.sin(t) * v) * 0.031
        f.holes.append(prim_frustum(a + off, b + off, 0.006, n=16, name="cool_hole"))
    s_fw = float(P.SPEC["fuselage"]["modules"]["firewall_s_m"])
    for y, z in _ring_points(s_fw + 0.001, _WALLS["cowl_pa_cf"] + 0.004, [20, 90, 160, 200, 270, 340], [], 0.02):
        f.holes.append(prim_frustum(Bp(s_fw - 0.002, y, z), Bp(s_fw + 0.008, y, z), 0.5 * M3_HOLE_D, n=12,
                                    name="fw_screw"))
    return f


# =====================================================================================================
# Özel parçalar
# =====================================================================================================
def build_nose_flange(ctx: Ctx, seg: SegSpec, out_col: bpy.types.Collection) -> Built | None:
    """Burun modülü PETG flanşı: modül halkasının iç flanşına (land) yapışan 3 mm halka; 4×M4 + 2×Ø4 pim."""
    s = float(P.SPEC["fuselage"]["modules"]["nose_module_s_m"][1])
    t = seg.wall or 0.003
    B = ctx.B
    slab_planes = [(Bp(s, 0, 0), np.array([-1.0, 0, 0])), (Bp(s - t, 0, 0), np.array([1.0, 0, 0]))]
    base = ctx.copy(ctx.outer("fus_clean"), "nf")
    reg = ctx.obj("regx", region_solid([(p + n * LOCAL_MARGIN, n) for p, n in slab_planes], Bp(s, 0, 0)))
    B.op(base, reg, "INTERSECT", solver=ctx.cut_solver)
    _delete(reg)
    V, Tt = triangulate(base.data)
    w = ctx.sources["fus"].wall_fn(V)
    outer_in = ctx.obj("nf_o", Solid(offset_vertices(V, Tt, np.maximum(w + 0.0006, LAND_T) + 0.0002),
                                     [tuple(x) for x in Tt]))
    inner_in = ctx.obj("nf_i", Solid(offset_vertices(V, Tt, w + 0.010), [tuple(x) for x in Tt]))
    _delete(base)
    reg = ctx.obj("reg", region_solid(slab_planes, Bp(s, 0, 0)))
    B.op(outer_in, reg, "INTERSECT")
    B.op(inner_in, reg, "INTERSECT")
    _delete(reg)
    B.op(outer_in, inner_in, "DIFFERENCE")
    _delete(inner_in)
    bolts, pins = nose_flange_bolts()
    holes = [prim_frustum(Bp(s - t - 0.003, y, z), Bp(s + 0.003, y, z), 0.5 * M4_HOLE_D, n=16) for y, z in bolts]
    holes += [prim_frustum(Bp(s - t - 0.003, y, z), Bp(s + 0.003, y, z), 0.5 * (NOSE_PIN_D + 0.0002), n=16)
              for y, z in pins]
    col = B.group(ctx.name("hole"), holes)
    B.op(outer_in, col, "DIFFERENCE")
    Booler.drop(col)
    clean_mesh(outer_in, triangles=True)
    ctx.work.objects.unlink(outer_in)
    out_col.objects.link(outer_in)
    outer_in.name = f"UP_{seg.key}"
    outer_in.data.name = outer_in.name
    return Built(seg, outer_in, mesh_check(outer_in.data))


def build_plain(ctx: Ctx, seg: SegSpec, out_col: bpy.types.Collection) -> Built | None:
    """Kabuksuz parça (kapak, ince panel, TPU pabuç): kaynak ∩ bölge."""
    A = ctx.copy(ctx.outer(seg.sources[0]), "A")
    if seg.ends:
        center = seg.center if seg.center is not None else np.mean([e.p for e in seg.ends], axis=0)
        reg = ctx.obj("reg", region_solid(_planes(seg.ends), center))
        ctx.B.op(A, reg, "INTERSECT", solver=ctx.cut_solver)
        _delete(reg)
    if len(A.data.polygons) == 0:
        _delete(A)
        return None
    cl = clean_mesh(A, triangles=True)
    ctx.work.objects.unlink(A)
    out_col.objects.link(A)
    A.name = f"UP_{seg.key}"
    A.data.name = A.name
    chk = mesh_check(A.data)
    chk["removed_islands"] = cl["removed_islands"]
    return Built(seg, A, chk)


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


def evaluate(b: Built, bed) -> dict:
    """Baskı yönü, tabla sığması, çıkıntı, hacim/kütle/süre ve et ölçümü."""
    seg = b.seg
    V, T = triangulate(b.ob.data)
    pref = []
    if seg.bed_end is not None and seg.bed_end < len(seg.ends):
        pref.append(-seg.ends[seg.bed_end].n)
    if seg.up_hint is not None:
        pref.append(np.asarray(seg.up_hint, float))
    pl = choose_orientation(V, T, bed, pref)
    if seg.force_up and pref:                           # dik baskı zorunlu: tercih edilen yön sığmıyorsa bölünecek
        pp = placement(V, T, pref[0], bed)
        if not (pp["fits"] or pp["fits_tight"]):
            pp["fits"] = pp["fits_tight"] = False
            pl = pp
        elif not (pl["fits"] and np.allclose(pl["up"], _unit(pref[0]), atol=1e-6)):
            pl = pp if (pp["fits"] or not pl["fits"]) else pl
    mat = seg.material or _zone(seg.zone)[0]
    wall = seg.wall if seg.wall is not None else _zone(seg.zone)[1]
    vol_cm3 = b.check["volume_m3"] * 1e6
    rate = PRINT_RATE["PETG" if mat.startswith("PETG") else mat]
    layers = pl["size_mm"][2] / rate["layer_mm"]
    hours = vol_cm3 / rate["cm3_h"] + layers * LAYER_OVERHEAD_S / 3600.0 + PLATE_SETUP_H
    over = pl["overhang_cm2"]
    support = "yok" if over < 3.0 else ("kısa köprü (iç)" if over < 30.0 else "destek önerilir")
    mw = min_wall_mm(b.ob, n_samples=1200)
    return {"placement": pl, "V": V, "T": T, "material": mat, "wall_mm": wall * 1000.0, "volume_cm3": vol_cm3,
            "mass_g": vol_cm3 * density(mat), "hours": hours, "support": support, "min_wall": mw}


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


def export_part(b: Built, ev: dict, stl_dir: Path) -> dict:
    """Baskı yönünde, tabla merkezli, mm STL. Dönüş: dosya bilgisi."""
    M = ev["placement"]["M"]
    V = ev["V"] @ M[:3, :3].T + M[:3, 3]
    T = ev["T"]
    path = stl_dir / f"{b.seg.key}.stl"
    size = write_stl(path, V, T, b.seg.key)
    return {"file": f"{stl_dir.name}/{path.name}", "bytes": size, "triangles": int(len(T))}   # çıktı köküne göre


def verify_stl(path: Path, expect_volume_cm3: float) -> dict:
    """STL'yi yeniden okur: kenar eşleşmesi (kapalı, tutarlı yön), hacim karşılaştırması; ayrıca Blender STL
    içe aktarıcısıyla (``wm.stl_import``) yükleyip bmesh manifold denetimi yapar."""
    V, T = read_stl(path)
    ec = stl_edge_check(T)
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
                      verify: int = 6, verbose: bool = False) -> dict:
    """Baskı parçalarını kurar, ``UCAV_Print``'e koyar, STL (mm) ve rapor yazar.

    ``bed``: tabla (mm) — varsayılan 256×256×256; 220×220×250 de desteklenir (sığmayan segment otomatik bölünür).
    ``out_dir``: çıktı kökü (varsayılan ``ucav/out``; varsayılan dışı tabla için ``ucav/out/print_<X>x<Y>x<Z>``).
    STL'ler ``<out_dir>/stl``, rapor ``<out_dir>/print_report.md`` ve ``.json``. ``only``: anahtar önekleri
    (ör. ``["wing_panel", "fus_ring_1"]``) — kısmi kurulum. ``verify``: yeniden okunup doğrulanacak STL sayısı.
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
            if seg.key == "fus_nose_flange":
                b = build_nose_flange(ctx, seg, pcol)
            elif not seg.shell:
                b = build_plain(ctx, seg, pcol)
            else:
                b = None
                for k in range(4):                      # sıkışma kalırsa küçük kaydırmayla yeniden dene
                    s_try = seg if k == 0 else SegSpec(**{**seg.__dict__, "perturb": k})
                    feat = r.feat_fn(s_try) if r.feat_fn else Feature()
                    bk = build_segment(ctx, s_try, feat, pcol)
                    if bk is None:
                        break
                    bad = bk.check["boundary"] + bk.check["nonmanifold"]
                    if b is None or bad < b.check["boundary"] + b.check["nonmanifold"]:
                        if b is not None:
                            _delete(b.ob)
                        b = bk
                        b.seg = seg
                    else:
                        _delete(bk.ob)
                    if b.check["ok"]:
                        if k:
                            ctx.log(f"{seg.key}: {k}. kaydırmalı denemede manifold")
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
        if b is None:
            ctx.warnings.append(f"{seg.key}: boş sonuç (kaynak bölgede yok)")
            continue
        if not b.check["ok"] and 0 < b.check["volume_m3"] < VOXEL_REPAIR_MAX_M3:
            v0 = b.check["volume_m3"]
            info = voxel_repair(b.ob)
            b.check = {**mesh_check(b.ob.data), "removed_islands": b.check.get("removed_islands", 0)}
            msg = (f"{seg.key}: voksel onarım ({info['voxel_mm']:.2f} mm), hacim {v0 * 1e6:.1f} → "
                   f"{b.check['volume_m3'] * 1e6:.1f} cm³, manifold {'evet' if b.check['ok'] else 'HAYIR'}")
            ctx.warnings.append(msg)
            b.info.setdefault("notes", []).append("voksel onarım")
            ctx.log(msg)
        if not seg.split_loose:
            n_fr, v_fr = drop_fragments(b.ob)
            if n_fr:
                b.check = {**mesh_check(b.ob.data), "removed_islands": b.check.get("removed_islands", 0) + n_fr}
                b.info.setdefault("notes", []).append(f"{n_fr} kopuk kıymık atıldı ({v_fr * 1e6:.2f} cm³)")
                ctx.log(f"{seg.key}: {n_fr} kopuk kıymık atıldı ({v_fr * 1e6:.2f} cm³)")
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
                    clean_mesh(ob, triangles=True)
                    for m in mats:
                        ob.data.materials.append(m)
                    builts.append(Built(s2, ob, mesh_check(ob.data)))
        for bb in builts:
            ev = evaluate(bb, bed)
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
                "support": ev["support"], "min_wall": {k: round(v, 3) for k, v in ev["min_wall"].items()},
                "note": seg.note, "pin_holes": int(b.info.get("pin_holes", 0)),
                "features": sorted(set(b.info.get("notes", [])))}
        if export:
            info["stl"] = export_part(b, ev, stl_dir)
        results.append(info)
    # doğrulama: birkaç STL'yi yeniden oku
    verified = []
    if export and verify:
        pick = sorted(results, key=lambda r: -r["volume_cm3"])
        step = max(1, len(pick) // max(1, verify))
        for r in pick[::step][:verify]:
            verified.append({"key": r["key"], **verify_stl(out_dir / r["stl"]["file"], r["volume_cm3"])})
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
def bom(plans: dict, n_pin_joints: int) -> list[dict]:
    """Basılmayan parçalar: CF borular, G10, bağlantı elemanları, servolar, takım, motor, aviyonik (spec'ten)."""
    out = []
    for t in _PR["spar_tubes"]:
        L = float(t["length_m"]) if "length_m" in t else 0.0
        out.append({"group": "CF boru/çubuk", "item": t["name_tr"],
                    "spec": f"Ø{t['od_mm']}/{t['id_mm']} mm × {L * 1000:.0f} mm" if t["id_mm"] else
                    f"Ø{t['od_mm']} mm dolu × {L * 1000:.0f} mm", "qty": int(t["qty"])})
    pin_len = sum(hp.L + 0.010 for hp in plans.values()) * 2
    out.append({"group": "CF boru/çubuk", "item": "Menteşe pimi (çelik yay teli)",
                "spec": f"Ø{HINGE_PIN_D * 1000:.1f} mm, toplam ≈ {pin_len:.2f} m (yüzey başına tek parça)",
                "qty": 2 * len(plans)})
    out.append({"group": "CF boru/çubuk", "item": "Segment hizalama pimi (CF çubuk)",
                "spec": f"Ø{PIN_D * 1000:.0f} mm × {2 * PIN_DEPTH * 1000 - 2:.0f} mm", "qty": int(n_pin_joints)})
    g = _PR["g10_bridge"]
    out += [
        {"group": "G10 / kontrplak", "item": "Dihedral köprüsü (epoksi dolgulu yuvalar)",
         "spec": f"{g['plates']}× {g['t_mm']}×{g['h_mm']}×{g['length_mm']} mm G10, {g['bolts']}", "qty": 1},
        {"group": "G10 / kontrplak", "item": "Yangın perdesi (s = 2,06)",
         "spec": f"G10 {float(P.SPEC['fuselage']['modules']['firewall_t_m']) * 1000:.0f} mm, gövde konturu", "qty": 1},
        {"group": "G10 / kontrplak", "item": "Ön gövde / kuyruk modülü flanşı (s = 1,55)", "spec": "G10 2 mm, 4×M4",
         "qty": 1},
        {"group": "G10 / kontrplak", "item": "ER-150 takım montaj plakası", "spec": "G10 3 mm (kök bloğu)",
         "qty": 3},
        {"group": "G10 / kontrplak", "item": "Kumanda hornu", "spec": "G10 1,6 mm, yarığa yapıştırma",
         "qty": 2 * len(plans)},
        {"group": "G10 / kontrplak", "item": "Kuyruk eyeri (stabilize boruları V birleşimi)",
         "spec": "G10 2 mm + epoksi, koni içinde", "qty": 1},
        {"group": "Dolgu", "item": "Kanat üstü fileto (basılmaz: sıfıra inen kama)",
         "spec": "epoksi + mikrobalon, kanat kökü–chine arası, ≈ 2 × 17 cm³; şablon: U_Fairing_Fillet_L/R", "qty": 2},
        {"group": "Dolgu", "item": "Stabilize kök filetosu (basılmaz: 0,1–3 mm kama)",
         "spec": "epoksi + mikrobalon (ısı bölgesi; LW-PLA yok), ≈ 2 × 4 cm³", "qty": 2},
        {"group": "Bağlantı", "item": "M4 cıvata + kelebek somun (burun modülü)", "spec": "A2 paslanmaz", "qty": 4},
        {"group": "Bağlantı", "item": "M4 cıvata + somun (G10 köprü, flanş, motor)", "spec": "12.9", "qty": 12},
        {"group": "Bağlantı", "item": "M3 cıvata + ısıl gömme dişli (kaporta, yanaklar)", "spec": "M3×8", "qty": 14},
        {"group": "Bağlantı", "item": "Hizalama pimi (burun modülü)", "spec": f"çelik Ø{NOSE_PIN_D * 1000:.0f}×20 mm",
         "qty": 2},
        {"group": "Bağlantı", "item": "Neodim mıknatıs (aviyonik kapağı)", "spec": "Ø6×3 mm", "qty": 6},
        {"group": "Bağlantı", "item": "MPX 6 pin konnektör (kanat paneli), 8 pin (burun modülü)", "spec": "",
         "qty": 3},
        {"group": "Servolar", "item": SERVO["name"], "spec": "kanatçık, dış/iç flap, elevatör, dümen", "qty": 10},
        {"group": "Servolar", "item": "Gaz, jikle, burun yönlendirme servosu", "spec": "mini", "qty": 3},
        {"group": "Servolar", "item": "Kapak servoları + sıralayıcı + fren", "spec": "mikro", "qty": 1},
        {"group": "İniş takımı", "item": P.SPEC["landing_gear"]["product"], "spec": "", "qty": 1},
        {"group": "İniş takımı", "item": "Ana tekerlek",
         "spec": f"Ø{float(P.SPEC['landing_gear']['main']['wheel_d_m']) * 1000:.1f} mm, frenli", "qty": 2},
        {"group": "İniş takımı", "item": "Burun tekerleği",
         "spec": f"Ø{float(P.SPEC['landing_gear']['nose']['wheel_d_m']) * 1000:.0f} mm", "qty": 1},
        {"group": "İtki", "item": P.SPEC["propulsion"]["engine"]["model"], "spec": "susturucu + CDI dahil", "qty": 1},
        {"group": "İtki", "item": P.SPEC["propulsion"]["prop"]["model"], "spec": "", "qty": 1},
        {"group": "İtki", "item": "Spinner", "spec": f"Al Ø{float(P.SPEC['propulsion']['spinner']['d_m']) * 1000:.0f} mm",
         "qty": 1},
        {"group": "İtki", "item": "Yakıt deposu", "spec": f"2 × {P.SPEC['fuselage']['internals']['fuel_tanks']['volume_l_each']} L",
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
    stl_bytes = sum(r.get("stl", {}).get("bytes", 0) for r in results)
    n_pins = int(math.ceil(sum(r.get("pin_holes", 0) * r["qty"] for r in results) / 2))
    est = _PR["estimates"]
    return {
        "model": str(P.SPEC["meta"]["name"]), "date": time.strftime("%Y-%m-%d"),
        "bed_mm": list(bed), "bed_margin_mm": BED_MARGIN,
        "solver": {"segment_cut": ctx.cut_solver, "shell_features": ctx.B.solver, "boolean_ops": ctx.B.n_ops,
                   "boolean_s": round(ctx.B.t_ops, 1), "elapsed_s": round(elapsed, 1)},
        "totals": {"unique_parts": len(results), "pieces": n_pieces, "mass_g": round(tot_mass, 0),
                   "print_h": round(tot_h, 1), "stl_mb": round(stl_bytes / 1e6, 2),
                   "spec_estimate": {"print_mass_kg": est["print_mass_kg"], "print_hours": est["print_hours"],
                                     "segment_count": est["segment_count"]}},
        "by_material": {k: {kk: round(vv, 1) for kk, vv in v.items()} for k, v in by_mat.items()},
        "by_group": {k: {kk: round(vv, 1) for kk, vv in v.items()} for k, v in by_group.items()},
        "all_manifold": all(r["check"]["ok"] for r in results),
        "all_fit": all(r["fits"] or r["fits_tight"] for r in results),
        "parts": results, "verified_stl": verified, "warnings": list(ctx.warnings),
        "hinges": {k: {"lugs": len(v.lugs), "length_mm": round(v.L * 1000, 1)} for k, v in plans.items()},
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
    L.append("| | Bu model | Spec tahmini |")
    L.append("|---|---:|---:|")
    est = T["spec_estimate"]
    L.append(f"| Benzersiz STL | {T['unique_parts']} | — |")
    L.append(f"| Basılacak parça (ayna dahil) | {T['pieces']} | {est['segment_count']} |")
    L.append(f"| Basılı kütle | {_fmt(T['mass_g'] / 1000, 2)} kg | {_fmt(est['print_mass_kg'], 2)} kg |")
    L.append(f"| Baskı süresi (kaba) | {_fmt(T['print_h'], 0)} h | {_fmt(est['print_hours'], 0)} h |")
    stl_txt = f"{_fmt(T['stl_mb'], 1)} MB" if any("stl" in p for p in S["parts"]) else "yazılmadı (--no-stl)"
    L.append(f"| STL toplamı | {stl_txt} | ≤ 40 MB hedef |")
    L.append(f"| Manifold / kapalı | {'evet, hepsi' if S['all_manifold'] else 'HAYIR — uyarılara bakın'} | |")
    n_tight = sum(1 for p in S["parts"] if not p["fits"] and p["fits_tight"])
    fit_txt = ("hepsi" + (f" ({n_tight} benzersiz parça dar payla, ≥ {BED_MARGIN_MIN:.0f} mm)" if n_tight else "")
               if S["all_fit"] else "HAYIR — uyarılara bakın")
    L.append(f"| Tablaya sığma | {fit_txt} | |")
    L.append("")
    L.append("Kütle, filament etkin yoğunluğuyla (LW-PLA köpürmüş 0,65 g/cm³) parça hacminden hesaplanır; boya, "
             "yapıştırıcı ve basılmayan parçalar dahil değildir. Süre: hacimsel hız + katman başına "
             f"{LAYER_OVERHEAD_S:.1f} s + tabla başına {PLATE_SETUP_H * 60:.0f} dk (kaba tahmin).")
    L.append("")
    L.append("| Malzeme | Parça | Hacim (cm³) | Kütle (g) | Süre (h) |")
    L.append("|---|---:|---:|---:|---:|")
    for k, v in sorted(S["by_material"].items()):
        L.append(f"| {k} | {v['pieces']:.0f} | {_fmt(v['volume_cm3'], 0)} | {_fmt(v['mass_g'], 0)} | {_fmt(v['hours'], 1)} |")
    L.append("")
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
             f"{_fmt_g(LATTICE_T * 1000)} mm geodezik kafes); en küçük değer köşe/pah örneklerini ve kaplamaların "
             "sıfıra incelen yapışma kenarlarını içerir.")
    L.append("")
    L.append("| STL | Kenar denetimi (yeniden okuma) | Hacim farkı | Blender içe aktarma |")
    L.append("|---|---|---:|---|")
    for v in S["verified_stl"]:
        bi = v.get("blender_import", {})
        bi_s = (f"{bi.get('faces', 0)} yüz, manifold olmayan {bi.get('nonmanifold', '?')}" if "error" not in bi
                else f"hata: {bi['error']}")
        ec = v["edges"]
        L.append(f"| `{v['key']}.stl` | {ec['triangles']} üçgen, eşsiz {ec['unpaired']}, çift {ec['dup_directed']} "
                 f"{'✓' if ec['ok'] else '✗'} | {v['volume_err_pct']:+.3f} % | {bi_s} |")
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
    L.append(_SETTINGS_MD.format(**{k: _fmt_g(v) if isinstance(v, float) else v for k, v in S["settings"].items()
                                   if not isinstance(v, list)},
                                 land=f"{_fmt(S['settings']['land_mm'][0], 1)} mm × {_fmt(S['settings']['land_mm'][1], 0)} mm",
                                 frame=f"{_fmt(S['settings']['frame_mm'][0], 1)} mm × {_fmt(S['settings']['frame_mm'][1], 0)} mm"))
    L.append("")
    L.append("## Kapsam")
    L.append("")
    L.append("Sivil gözetleme/araştırma platformu. Faydalı yük yalnız EO/IR taret; silah, mühimmat, dış yük askısı "
             "ya da bırakma mekanizması yoktur ve eklenmez.")
    L.append("")
    return "\n".join(L)


_ASSEMBLY_MD = """## Montaj sırası

1. **Kanat dış paneli** (her yan): segmentleri kökten uca dizin; her ekte alttaki segmentin kaburgası üstteki
   segmentin 2,4 mm flanşına oturur. Ø3 CF hizalama pimlerini hücum kenarı göbeklerine yapıştırın, 27/25 panel
   borusunu ve 8/6 arka kirişi kılavuz kovanlarından geçirerek kuru montajla hizayı kontrol edin; sonra ekleri
   ince CA / 5 dk epoksi ile sırayla yapıştırın. Uç borusu (16/14) ve kademeli burç 1,00–1,10'da panel borusuna girer.
2. **Uç kapağı**: seyrüsefer LED'ini yuvasına koyup kabloyu kaburga deliklerinden geçirin; eğik uç düzlemine yapıştırın.
3. **Kumanda yüzeyleri**: segmentleri çift kaburga yüzlerinden yapıştırın (pim deliği hizalı). Yüzeyi oyuğa
   yerleştirip Ø1,5 çelik pimi dış uçtan dil ve kovanlardan geçirin; pim ucunu oyuk duvarında CA ile sabitleyin.
   G10 hornu yarığa epoksiyle yapıştırın; servo yuvasına servoyu takıp itme çubuğunu bağlayın.
4. **Kök blokları ve strake**: kök bloğu 1 ile glove strake'i pimlerle birleştirin, kök bloğu 2'yi flanşından
   yapıştırın. Ø30 soketi ve açı pimi kovanını epoksiyle sabitleyin; ER-150 ünitesini G10 plakasıyla kuyu
   üstüne bağlayın.
5. **Gövde**: halkaları çerçeve (tabla tarafı) → flanş (üst) sırasıyla dizin; 4 adet 8/6 CF longeronu chine ve omuz
   kovanlarından geçirerek hizalayın ve yapıştırın. s = 1,55'e G10 flanşı, s = 2,06'ya G10 yangın perdesini
   yapıştırın (kuyruk konisi LW-ASA). G10 dihedral köprüsünü kanat kutusu açıklığından geçirip kök bloklarına
   M4 ile bağlayın; kök kaportasını yapıştırın, kanat üstü filetoyu epoksi + mikrobalonla şekillendirin.
6. **Burun modülü**: burun konisi + modül halkasını yapıştırın; PETG flanşı modül halkasının flanşına yapıştırın;
   taret yakasını yuvasına takın. Modül 2×Ø4 pim + 4×M4 kelebek somunla halka 1'e bağlanır (sökülebilir).
7. **Kuyruk**: stabilize yarılarını 12/10 kiriş ve 6/4 arka çubukla koni içindeki G10 eyerde birleştirin; kök
   yüzleri koni konturuna 0,3 mm boşlukla oturur. Kök filetosunu epoksi + mikrobalonla doldurun. Dikeyleri stabilize ucuna
   (iç yüz yuvası) ve 8/6 dikey kirişine geçirin.
8. **Motor bölümü**: DLE-20'yi yangın perdesine bağlayın, ısı kalkanını takın; PA-CF kaportayı 6×M3 ile perdeye,
   yanakları M3 ısıl gömme dişlilerle flanşlarına vidalayın; lüle halkasını kaporta arkasına yapıştırın; TPU pabucu
   kaporta altına yapıştırın.
9. **Kapaklar**: takım kapaklarını (PETG) menteşelerine, aviyonik kapağını mıknatıslarına yerleştirin. Ağırlık
   merkezini s = 1,232 m'de (aralık 1,219–1,245) doğrulayın."""


_SETTINGS_MD = """## Yazıcı ayarları

**LW-PLA tek duvar deriler** (kanat, gövde, kuyruk; colorFabb LW-PLA): nozul 0,4 mm, 230–245 °C (köpürme), akış
%55–60, çizgi genişliği 0,6 mm (köpürmüş), katman 0,25–0,30 mm, çevre sayısı 1 (0,6 mm deri tek çizgi;
1,2 mm D-kutu ve flanşlar 2 çizgi), dolgu %0, üst/alt katman 3 (kaburga {rib_mm} mm ve çerçeveler dolu basılır),
hız 40–60 mm/s, fan %30–50, tabla 50–60 °C, geri çekme 0,5–1 mm (LW-PLA sızar; "geri çekmede sil" açık),
"çevreleri geçme" seyir açık, dikiş firar kenarında hizalı. İnce duvar algılama (Arachne / "Print Thin Walls")
açık olmalı: {lattice_mm} mm geodezik kafes tek ince çizgiyle basılır; boşluk dolgusu kapalı.
Segmentler dik (açıklık Z'de) basılır; kaburga tablada, {land} flanş üstte. 45°'den dik çıkıntı yalnız
menteşe dili kamalarında ve kapalı kuyruk uçlarında (kısa köprü) vardır; destek gerekmez.

**LW-ASA** (kuyruk konisi, 1,0 mm): 250–265 °C, akış %55–65, kapalı kabin, tabla 95–105 °C, fan %0–20.
**PA-CF** (kaporta, lüle, kök kaportası, alık; 1,6 mm): sertleştirilmiş nozul, 270–290 °C,
kurutulmuş filament (80 °C 6 h), 3 çevre, %100 dolgu (ince duvar), kapalı kabin. Kaplamaların gövdeye/kanada
yapışan kenarı sıfıra incelir (alanın %2–7'si < 0,5 mm); dilimleyici 0,4 mm altını basmaz → montajda epoksi
macunla kenar düzeltilir.
**PETG** (kapaklar, taret yakası, burun flanşı; 1,2–3 mm): 235–245 °C, %100 dolgu, katman 0,2 mm; aviyonik
kapağı füme PETG, parlak yüz yukarı.
**TPU 95A** (sürtünme pabucu): 220–230 °C, yavaş (20 mm/s), geri çekme kapalı.

Montaj payları: boru deliği Ø + {tube_clear_mm} mm, kovan eti {sleeve_mm} mm, kesme ağı {web_mm} mm,
geodezik kafes {lattice_mm} mm (±45°, 70 mm aralık), hizalama pimi Ø{pin_mm} mm, menteşe pimi Ø{hinge_pin_mm} mm,
çerçeve {frame}."""


# =====================================================================================================
# Önizleme (patlatılmış) — düşük çözünürlük Cycles
# =====================================================================================================
PREVIEW_COLOR = {"LW-PLA": (0.80, 0.80, 0.77), "LW-ASA": (0.62, 0.58, 0.50), "PA-CF": (0.09, 0.09, 0.10),
                 "PETG": (0.85, 0.32, 0.06), "PETG-füme": (0.12, 0.16, 0.20), "TPU": (0.03, 0.03, 0.03)}


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
        if not ob.name.startswith("UP_"):
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
    return 0 if (S["all_manifold"] and S["all_fit"]) else 1


if __name__ == "__main__":                                      # pragma: no cover
    sys.exit(main())
