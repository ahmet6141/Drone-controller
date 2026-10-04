"""YELKOVAN YK-38 — iniş takımı (bpy 4.5): ``build(scene=None)``.

JP Hobby ER-150 sınıfı elektrikli toplama üniteleri, yağlı amortisörlü (oleo) burun ve ana bacaklar, makaslı
tork bağlantıları, çatal ve aks, lastik (sırt dişi + yanak), jant, ana tekerlerde jant içi kampanalı fren, burun
yönlendirme. Bütün sayılar ``params``'tan (pivot, bacak boyu, yatıklık, teker ölçüleri, toplama ekseni, statik
çökme) gelir; burada yalnızca görünmeyen tasarım ayrıntıları (cidar, pim çapı, bağlantı boyu…) ``# varsayım``
olarak tanımlıdır.

Hiyerarşi (dinlenme pozu = takım AÇIK, amortisör yüksüz; sürücüler ``rig.setup`` ile)
-------------------------------------------------------------------------------------
Her bacak ``X`` ∈ {N, L, R} için (koleksiyon ``UCAV_Gear``)::

    U_Root
    ├── U_GearUnit_X        sabit: ER-150 gövdesi, G10 plaka, trunnion pimi (+ ana takımda trunnion kaportası)
    └── U_GearPivot_X       empty, trunnion; yerel X = GearLeg.retract_axis_b → rotation_euler.x = +retract_deg TOPLAR
        ├── U_GearAxleRef_X empty, yüksüz aks merkezi (amortisör sürücüsü bunun dünya Z'sini okur)
        ├── U_GearStrut_L/R  dönen trunnion bloğu (kaporta kovanı) + üst amortisör silindiri + kapak braketleri
        │   ├── U_Door_L/R_2 bacak kapağı (airframe; bu modül bacağa bağlar)
        │   ├── U_GearLinkU_X üst tork bağlantısı (yerel Y etrafında döner)
        │   └── U_GearSlider_X krom kayar boru + çatal + aks (+ fren tablası); location.z = z0 + sıkışma
        │       ├── U_GearLinkL_X alt tork bağlantısı
        │       └── U_GearWheel_X lastik + jant (+ fren kampanası); yerel Y = aks → rotation_euler.y = dönüş
        └── (burun) U_GearKnuckle_N trunnion bloğu → U_GearSteer_N (empty, yerel Z = bacak ekseni, yönlendirme)
                    → U_GearStrut_N (silindir + yönlendirme kolu) → U_GearSlider_N → U_GearWheel_N

Bacak çerçevesi (dünya, dinlenme): X = bacağa dik ileri, Y = teker aksı (+Y = iskele), Z = bacak boyunca yukarı
(aks → pivot). Alt nesnelerin ebeveyn ters matrisi birimdir: ``location`` bacak çerçevesinde yerel konumdur
(amortisör sürücüsü tek kanal: ``location[2]``). Kapak ``U_Door_<leg>_2`` airframe'de toplu konumda
modellenmiştir; burada açık poza (−retract_deg) çevrilip ``U_GearStrut``'a bağlanır (yeniden çağrıda params'tan
hesaplanır, birikmez). Deri kapakları (``U_Door_N_1/2``, ``U_Door_L/R_1``) ``U_Root``'ta kalır; yalnızca
``rig`` sürer.

Paketleme kararları (``build`` sonrası ``report()`` sayısal özet verir)
---------------------------------------------------------------------
* Ana bacakta amortisör ekseni pivot hattından ``MAIN_STRUT_OFFSET`` (8,2 mm) İÇE kaydırılmıştır: içe 90°
  toplanınca boru kuyu tavanına doğru (yukarı) kalkar, bacak kapağına 1 mm'den fazla pay kalır. Teker merkezi
  yine params aks noktasındadır (çatal tacı farkı kapatır) → iz genişliği ve yer teması değişmez.
* Ana teker kuyuda yatık yatar (36 mm derinlik): çatal kolları 1,6 mm, lastiğe 1 mm pay, aks uçları gömme. Lastik
  kesiti yuvarlatılmış (balon) profildir. Kuyu tavanı arka-iç köşede kanat üst derisiyle sınırlandığından
  (``shapes.well_roof_z``) lastik eni ``tyre_fit_width()`` ile tavana ``TYRE_CLEAR`` pay kalacak şekilde
  sınırlanır (bugün 22,5 mm; params 26 mm). Tavan yükseltilirse en kendiliğinden params değerine döner.
* Ana tork bağlantıları ileri-içe (32°) bakar ve yüksüzken neredeyse düzdür (diz 1,8 mm dışarıda): toplu konumda
  26 mm'lik bacak yuvasında kalırlar; yerde statik çökmeyle ~29° bükülürler.
* Doğrulama: ``clearance_report()`` takım çevrimi boyunca BVH kesişimlerini verir; ``new`` ve ``known`` boş
  olmalıdır. (Eski airframe temasları giderildi: kuyu duvarları artık dişsiz zarfı izler — testere dişi yalnız deri
  dudağındadır, ``shapes.gear_cutters``; bacak kapağı pivotun 7,5 mm içinde biter — ``params.LEG_DOOR_OUT``.
  ``_known_airframe_contact`` bu temaslar geri gelirse etiketlemek için durur.)
* Ana trunnion pimi (z −0,042) kabartmanın dış kenarında deri yüzeyinin 1,7 mm altında kalır (params/airframe);
  pim yatakları bu yüzden pim ekseni etrafında simetrik bir trunnion kaportasıyla (önde/arkada sabit, ortada dönen
  kovan) örtülür — takım her konumda aynı kesintisiz kabarıklık olarak görünür.

Kullanım::

    from ucav.blender import airframe, gear, rig
    airframe.build(); gear.build(); rig.setup()
"""
from __future__ import annotations

import math
import re
import time
from dataclasses import dataclass, field

import bpy
import numpy as np
from mathutils import Matrix, Vector

from .. import params as P
from .. import shapes as S
from . import util as U

LEGS = ("N", "L", "R")
COLLECTION = "UCAV_Gear"
_rad = math.radians

M_GEAR, M_STEEL, M_HUB, M_TIRE = P.MATERIAL_ROLES["gear"], P.MATERIAL_ROLES["steel"], P.MATERIAL_ROLES["hub"], P.MATERIAL_ROLES["tire"]
M_DARK, M_SKIN, M_PLATE = P.MATERIAL_ROLES["engine"], P.MATERIAL_ROLES["skin_bottom"], P.MATERIAL_ROLES["pacf"]

# ----------------------------------------------------------------------------------------------- ER-150 gövdesi
_ER = re.search(r"(\d+)\s*×\s*(\d+)\s*×\s*(\d+)\s*mm", str(P.SPEC["landing_gear"]["product"]))
ER150_BODY = tuple(float(v) / 1000.0 for v in _ER.groups()) if _ER else (0.026, 0.102, 0.032)   # G × U × Y (spec ürün metni)

# ----------------------------------------------------------------------------------------------- görünmeyen ayrıntılar (varsayım)
PIN_D = 0.004                     # trunnion pimi (çelik). varsayım
MAIN_STRUT_OFFSET = 0.0082        # ana amortisör ekseni pivot hattından içe (toplu konumda yukarı). varsayım
MAIN_KNUCKLE_R = 0.0065           # dönen trunnion kovanı / kaporta yarıçapı. varsayım
SLOT_CLEAR = 0.0005               # dönen parça ile kuyu astarı arası en az pay. varsayım
FORK_GAP = 0.0010                 # lastik ile çatal kolu arası. varsayım
FORK_T = 0.0016                   # çatal kolu kalınlığı (7075 levha). varsayım
CROWN_T = 0.0050                  # çatal tacı kalınlığı. varsayım
CROWN_GAP = 0.0028                # lastik sırtı ile taç arası. varsayım
STROKE_DEFAULT = float(P.SPEC["landing_gear"]["main"].get("stroke_m", 0.015))
NOSE_STROKE = float(P.SPEC["landing_gear"]["nose"].get("stroke_m", STROKE_DEFAULT))   # burunda spec yok → ana. varsayım
LINK_ANGLE_MAIN = 32.0            # ana tork bağlantısı: ileriden içe doğru açı (°); toplu konumda yuvaya sığar. varsayım
LINK_T = 0.0030                   # bağlantı plakası kalınlığı (pim ekseni boyunca). varsayım
LINK_MAIN = {"ell": 0.032, "r_lug": 0.0080, "knee_off": 0.0018, "w_pin": 0.0022, "w_knee": 0.0020}   # varsayım
LINK_NOSE = {"ell": 0.037, "r_lug": 0.0080, "z_u": -0.094, "w_pin": 0.0030, "w_knee": 0.0024}        # varsayım
TYRE = {"bead": 0.70, "rm": 0.32, "p_out": 1.85, "groove_a": 0.30, "groove_w": 0.0012, "groove_d": 0.0009}
                                  # lastik kesiti: topuk eni oranı, en geniş yerin kesit yüksekliğindeki yeri, sırt
                                  # süperelips üssü, oluk konumu (yarı ene oran)/eni/derinliği. varsayım
TYRE_CLEAR = 0.0006               # toplu tekerin kuyu tavanına en az payı (lastik eni buna göre sınırlanır). varsayım
DOOR_BRACKET_Z = (0.035, 0.085)   # bacak kapağı braketleri (pivottan bacak boyunca). varsayım


# =====================================================================================================
# Geometri yardımcıları (saf numpy; ``shapes.MeshBuilder`` / ``shapes.lathe``; space="local")
# =====================================================================================================
def _unit(v) -> np.ndarray:
    v = np.asarray(v, float)
    return v / np.linalg.norm(v)


def _basis_for(axis, ref=None) -> np.ndarray:
    """Sütunları (u, v, w = axis) olan sağ el 3×3; u, ``ref``'e (yoksa en dik eksene) en yakın."""
    w = _unit(axis)
    if ref is None:
        ref = np.eye(3)[int(np.argmin(np.abs(w)))]
    u = np.asarray(ref, float) - w * float(np.dot(ref, w))
    if np.linalg.norm(u) < 1e-9:
        u = np.eye(3)[int(np.argmin(np.abs(w)))] - w * w[int(np.argmin(np.abs(w)))]
    u = _unit(u)
    v = np.cross(w, u)
    return np.column_stack([u, v, w])


def _xf(md: S.MeshData, R=None, t=None) -> S.MeshData:
    """Yerel ağı döndürür/taşır (yerinde): v' = R v + t."""
    V = np.asarray(md.verts, float)
    if R is not None:
        V = V @ np.asarray(R, float).T
    if t is not None:
        V = V + np.asarray(t, float)
    md.verts = V
    return md


def lathe_along(name: str, profile, origin, axis, mat, n: int = 32, ref=None, closed_profile: bool = False,
                smooth: float | None = 35.0, radius_fn=None) -> S.MeshData:
    """``profile`` (a, r) dönel yüzeyi; ``a`` = ``origin``'den ``axis`` boyunca."""
    md = S.lathe(np.asarray(profile, float), nseg=n, name=name, mat=mat, axis="z", closed_profile=closed_profile,
                 space="local", radius_fn=radius_fn, smooth_angle=smooth)
    return _xf(md, _basis_for(axis, ref), origin)


def cyl(name: str, p0, p1, r: float, mat, n: int = 28, r1: float | None = None, chamfer: float = 0.0,
        smooth: float | None = 35.0) -> S.MeshData:
    """``p0``→``p1`` silindir/kesik koni (uçlar kapalı, isteğe bağlı pah)."""
    p0, p1 = np.asarray(p0, float), np.asarray(p1, float)
    L = float(np.linalg.norm(p1 - p0))
    r1 = r if r1 is None else r1
    c = min(chamfer, 0.45 * L, 0.45 * min(r, r1))
    prof = [(0.0, 0.0)]
    prof += [(0.0, r - c), (c, r)] if c > 0 else [(0.0, r)]
    prof += [(L - c, r1), (L, r1 - c)] if c > 0 else [(L, r1)]
    prof += [(L, 0.0)]
    return lathe_along(name, prof, p0, p1 - p0, mat, n, smooth=smooth)


def rrect(hu: float, hv: float, r: float, n_c: int = 5) -> np.ndarray:
    """Köşe yarıçapı ``r`` olan dikdörtgen çevre (u, v), saat yönü tersine."""
    r = min(r, 0.999 * hu, 0.999 * hv)
    if r <= 1e-9:
        return np.array([(hu, hv), (-hu, hv), (-hu, -hv), (hu, -hv)])
    pts = []
    for cu, cv, a0 in ((hu - r, hv - r, 0.0), (-hu + r, hv - r, 90.0), (-hu + r, -hv + r, 180.0), (hu - r, -hv + r, 270.0)):
        for k in range(n_c + 1):
            a = _rad(a0 + 90.0 * k / n_c)
            pts.append((cu + r * math.cos(a), cv + r * math.sin(a)))
    return np.asarray(pts)


def stadium(p_a, p_b, r_a: float, r_b: float, n: int = 10) -> np.ndarray:
    """İki dairesel uçlu (``r_a``, ``r_b``) teğet çevre (2D), saat yönü tersine."""
    p_a, p_b = np.asarray(p_a, float), np.asarray(p_b, float)
    d = p_b - p_a
    L = float(np.linalg.norm(d))
    ang = math.atan2(d[1], d[0])
    phi = math.asin(max(-0.99, min(0.99, (r_a - r_b) / L)))
    pts = []
    for k in range(n + 1):                      # b ucu: −(90°−φ) … +(90°−φ)
        a = ang - (math.pi / 2 - phi) + k * (math.pi - 2 * phi) / n
        pts.append(p_b + r_b * np.array([math.cos(a), math.sin(a)]))
    for k in range(n + 1):                      # a ucu
        a = ang + (math.pi / 2 - phi) + k * (math.pi + 2 * phi) / n
        pts.append(p_a + r_a * np.array([math.cos(a), math.sin(a)]))
    return np.asarray(pts)


def prism(name: str, poly_uv, origin, u, v, w0: float, w1: float, mat, smooth: float | None = 40.0,
          mat_side=None) -> S.MeshData:
    """2D çevre (``u``, ``v`` düzleminde) ``w = u × v`` boyunca ``w0``…``w1`` çekilmiş kapalı katı."""
    u, v = _unit(u), _unit(v)
    w = np.cross(u, v)
    poly = np.asarray(poly_uv, float)
    base = np.asarray(origin, float) + poly[:, :1] * u + poly[:, 1:2] * v
    mb = S.MeshBuilder(name, "local")
    a = mb.add(base + w0 * w)
    b = mb.add(base + w1 * w)
    mb.strip(a, b, mat_side or mat)
    mb.cap(a, mat, start=True)
    mb.cap(b, mat, start=False)
    for i in range(len(poly)):                 # kapak kenarları keskin
        mb.sharp += [(int(a[i]), int(a[(i + 1) % len(poly)])), (int(b[i]), int(b[(i + 1) % len(poly)]))]
    return mb.build(smooth)


def box(name: str, center, half, R=None, mat=M_GEAR, r: float = 0.0, smooth: float | None = 40.0) -> S.MeshData:
    """Yuvarlatılmış kenarlı kutu: ``half`` (hx, hy, hz), çerçeve ``R`` (sütunlar), yuvarlama XY köşelerinde."""
    R = np.eye(3) if R is None else np.asarray(R, float)
    hx, hy, hz = half
    return prism(name, rrect(hx, hy, r), center, R[:, 0], R[:, 1], -hz, hz, mat, smooth)


def merge(name: str, parts, smooth: float | None = 35.0) -> S.MeshData:
    md = S.merge(name, [p for p in parts if p is not None], smooth)
    md.space = "local"
    return md


# =====================================================================================================
# Bacak çerçevesi ve ölçüler (params'tan)
# =====================================================================================================
@dataclass
class LegGeom:
    """Bir bacağın çerçeveleri ve türetilmiş ölçüleri (bacak çerçevesi: X ileri ⊥, Y aks, Z bacak boyunca yukarı)."""

    name: str
    leg: P.GearLeg
    sg: float                         # +1: N ve L (iskele), −1: R
    pivot_b: np.ndarray
    F_pivot: np.ndarray               # 3×3, X = toplama ekseni
    F_leg: np.ndarray                 # 3×3
    L: float                          # pivot → aks (yüksüz)
    y_e: float                        # amortisör ekseninin bacak Y'si (ana: içe kaydırma)
    R_w: float
    W_w: float
    r_rim: float
    strut_r: float
    slider_r: float
    stroke: float
    z: dict = field(default_factory=dict)      # istasyonlar (bacak Z'si, pivot = 0)
    link: dict = field(default_factory=dict)   # tork bağlantısı

    @property
    def is_nose(self) -> bool:
        return self.name == "N"

    @property
    def inboard_y(self) -> float:
        """Bacak Y ekseninde gövdeye doğru yön (+1/−1); burunda 0."""
        return 0.0 if self.is_nose else -self.sg

    def leg_to_world(self, q, retract_deg: float = 0.0) -> np.ndarray:
        """Bacak çerçevesindeki nokta(lar) → dünya; ``retract_deg`` kadar toplanmış pozda."""
        q0 = np.asarray(q, float)
        W = np.atleast_2d(q0) @ self.F_leg.T
        if retract_deg:
            W = P.rotate_about_axis(W, np.zeros(3), self.leg.retract_axis_b, retract_deg)
        return (W + self.pivot_b).reshape(q0.shape)

    @property
    def axle_l(self) -> np.ndarray:
        return np.array([0.0, 0.0, -self.L])


def tyre_half_width_factor(r, R: float, r_rim: float) -> np.ndarray:
    """Lastik kesitinin ``r`` yarıçapındaki yarı eninin en büyük yarı ene oranı (``tyre_profile`` ile aynı eğri)."""
    r = np.asarray(r, float)
    Ab = TYRE["bead"]
    r_m = r_rim + TYRE["rm"] * (R - r_rim)
    B, p = R - r_m, TYRE["p_out"]
    lo = Ab + (1 - Ab) * np.sqrt(np.clip(1 - ((r_m - r) / (r_m - r_rim)) ** 2, 0, 1))
    hi = np.clip(1 - np.clip((r - r_m) / B, 0, 1) ** p, 0, 1) ** (1 / p)
    return np.where(r < r_m, lo, hi)


@__import__("functools").lru_cache(maxsize=None)
def tyre_fit_width(name: str) -> float:
    """Ana tekerin toplu konumda kuyuya sığan en büyük lastik eni (m): kuyu tavanı (``shapes.well_roof_z`` − astar)
    tekerin orta düzleminden ``r`` yarıçapında her yönde ``hw(r) + TYRE_CLEAR``'dan yüksek olmalı. Tavan kanat üst
    derisiyle sınırlandığı arka-iç köşede eni daraltır; airframe tavanı yükseltirse params değerine döner."""
    g = P.gear_leg(name)
    c = np.asarray(g.axle_retracted, float)
    roof = S.well_roof_z(name)
    liner = S.BAY_INSET + S.BAY_WALL
    rr = np.linspace(0.5 * g.hub_d, g.wheel_r, 24)
    th = np.linspace(0.0, 2 * math.pi, 96, endpoint=False)
    best = g.wheel_w
    for r in rr:
        avail = min(roof(c[0] + r * math.cos(t), c[1] + r * math.sin(t)) - liner - c[2] for t in th)
        f = float(tyre_half_width_factor(r, g.wheel_r, 0.5 * g.hub_d))
        if f > 1e-6:
            best = min(best, 2.0 * (avail - TYRE_CLEAR) / f)
    return float(math.floor(best * 2000.0) / 2000.0)          # 0,5 mm'ye aşağı yuvarla


def leg_geom(name: str) -> LegGeom:
    """``params.gear_leg`` → çerçeveler ve parça istasyonları."""
    g = P.gear_leg(name)
    sg = -1.0 if name == "R" else 1.0
    pivot_b = np.asarray(g.pivot_b, float)
    X_r = _unit(g.retract_axis_b)
    Fp = S._frame_from_axes(X_r, [0.0, 0.0, 1.0])
    a_b = _unit(P.vec_to_blender(g.leg_dir))                # pivottan aksa (aşağı)
    Y = np.array([0.0, 1.0, 0.0])                            # teker aksı (wheel_axis_b)
    Zl = -a_b
    Xl = _unit(np.cross(Y, Zl))
    Fl = np.column_stack([Xl, Y, Zl])
    nose = name == "N"
    W = g.wheel_w if nose else min(g.wheel_w, tyre_fit_width(name))
    lg = LegGeom(name, g, sg, pivot_b, Fp, Fl, g.leg_length, 0.0 if nose else -sg * MAIN_STRUT_OFFSET, g.wheel_r,
                 W, 0.5 * g.hub_d, 0.5 * g.strut_d, 0.5 * g.slider_d, NOSE_STROKE if nose else STROKE_DEFAULT)
    R_w, L = lg.R_w, lg.L
    crown_bot = -(L - R_w) + CROWN_GAP
    if nose:
        z = {"knuckle": 0.0, "house_top": -0.003, "house_bot": -0.022, "horn": -0.0240, "cyl_top": -0.016,
             "gland": -0.098, "slider_top": -0.060}
        lk = dict(LINK_NOSE)
        n_dir = np.array([1.0, 0.0, 0.0])
    else:
        z = {"knuckle": 0.0, "sock_top": -0.0035, "sock_bot": -0.026, "cyl_top": -0.020, "gland": -0.106,
             "slider_top": -0.060}
        lk = dict(LINK_MAIN)
        th = _rad(LINK_ANGLE_MAIN)
        n_dir = np.array([math.cos(th), lg.inboard_y * math.sin(th), 0.0])
    z["crown_bot"] = crown_bot
    z["crown_top"] = crown_bot + CROWN_T
    z["axle"] = -L
    ell = lk["ell"]
    z_l = z["crown_top"] + 0.0042                              # alt kulak tacın üstünde
    if "knee_off" in lk:                                       # dizin yüksüz konumda dışa çıkıntısı sabit
        alpha0 = math.asin(lk["knee_off"] / ell)
        z_u = z_l + 2 * ell * math.cos(alpha0)
    else:
        z_u = lk["z_u"]
        alpha0 = math.acos(min(1.0, (z_u - z_l) / (2 * ell)))
    D0 = z_u - z_l
    lg.z = z
    lg.link = {"n": n_dir, "r_lug": lk["r_lug"], "z_u": z_u, "z_l": z_l, "ell": ell, "D0": D0, "alpha0": alpha0,
               "w_pin": lk["w_pin"], "w_knee": lk["w_knee"]}
    return lg


def leg_geoms() -> dict[str, LegGeom]:
    return {n: leg_geom(n) for n in LEGS}


# =====================================================================================================
# Teker: lastik, jant, fren
# =====================================================================================================
def tyre_profile(R: float, r_rim: float, W: float) -> np.ndarray:
    """Lastik kesiti (a, r) kapalı döngü: topuk → alt yanak → omuz → sırt (iki çevresel oluk, dik duvarlı) →
    karşı omuz/yanak → topuk → jant oturma yüzeyi. Üst kesit süperelipstir (balon lastik)."""
    A = 0.5 * W
    Ab = TYRE["bead"] * A
    r_m = r_rim + TYRE["rm"] * (R - r_rim)
    B = R - r_m
    p = TYRE["p_out"]
    ga, gw, gd = TYRE["groove_a"] * A, TYRE["groove_w"], TYRE["groove_d"]

    def r_of_a(a):
        return r_m + B * max(0.0, 1.0 - abs(a / A) ** p) ** (1.0 / p)

    a_c = 0.62 * A
    phi_c = math.acos((a_c / A) ** (p / 2.0))
    out = []
    for psi in np.linspace(math.pi / 2, 0.0, 9)[:-1]:                     # alt yanak (−a)
        out.append((-(Ab + (A - Ab) * math.cos(psi)), r_m - (r_m - r_rim) * math.sin(psi)))
    for ph in np.linspace(math.pi, math.pi - phi_c, 16)[:-1]:            # omuz (−a)
        c, sn = math.cos(ph), math.sin(ph)
        out.append((-A * abs(c) ** (2 / p), r_m + B * abs(sn) ** (2 / p)))
    edges = sorted([(-ga - gw / 2, "L"), (-ga + gw / 2, "R"), (ga - gw / 2, "L"), (ga + gw / 2, "R")])
    xs = sorted(set(np.round(np.linspace(-a_c, a_c, 23), 7)) | {round(e, 7) for e, _ in edges})
    kind = {round(e, 7): k for e, k in edges}

    def in_groove(a):
        return any(abs(a - gc) < gw / 2 - 1e-9 for gc in (-ga, ga))

    for a in xs:                                                           # sırt
        r = r_of_a(a)
        k = kind.get(round(a, 7))
        if k == "L":
            out += [(a, r), (a, r - gd)]
        elif k == "R":
            out += [(a, r - gd), (a, r)]
        else:
            out.append((a, r - gd if in_groove(a) else r))
    for ph in np.linspace(phi_c, 0.0, 16)[1:]:                            # omuz (+a)
        c, sn = math.cos(ph), math.sin(ph)
        out.append((A * abs(c) ** (2 / p), r_m + B * abs(sn) ** (2 / p)))
    for psi in np.linspace(0.0, math.pi / 2, 9)[1:]:                      # alt yanak (+a)
        out.append((Ab + (A - Ab) * math.cos(psi), r_m - (r_m - r_rim) * math.sin(psi)))
    for a in np.linspace(Ab, -Ab, 7)[1:-1]:                               # jant oturma yüzeyi
        out.append((float(a), r_rim))
    return np.asarray(out)


def tyre(name: str, lg: LegGeom, n: int = 72) -> S.MeshData:
    """Lastik (yerel: aks merkezi, aks = Y)."""
    prof = tyre_profile(lg.R_w, lg.r_rim, lg.W_w)
    md = S.lathe(prof, nseg=n, name=name, mat=M_TIRE, axis="y", closed_profile=True, space="local", smooth_angle=50.0)
    return md


def hub(name: str, lg: LegGeom, brake_side: float = 0.0, n: int = 48) -> S.MeshData:
    """Jant: iki flanşlı jant gövdesi, çanak göbek, rulman kapağı, 5 kollu kabartma, cıvatalar; ``brake_side``
    (±1) tarafında kampana (dönen)."""
    rr = lg.r_rim
    A = 0.5 * lg.W_w
    Ab = TYRE["bead"] * A
    fl = Ab + 0.0012                               # flanş dış yüzü
    boss_a = A + FORK_GAP - 0.0002                 # göbek, çatal kolunun 0,2 mm içinde biter
    parts = []
    # jant gövdesi (kapalı döngü profil, a ekseni = Y)
    prof = [(-fl, rr - 0.0040), (-fl, rr + 0.0011), (-Ab + 0.0002, rr + 0.0011), (-Ab + 0.0002, rr - 0.0001),
            (Ab - 0.0002, rr - 0.0001), (Ab - 0.0002, rr + 0.0011), (fl, rr + 0.0011), (fl, rr - 0.0040)]
    parts.append(S.lathe(np.asarray(prof), nseg=n, name="rim", mat=M_HUB, axis="y", closed_profile=True, space="local",
                         smooth_angle=40.0))
    # çanak göbek: dış yüz (−brake_side) çukur, iç yüz (brake_side) kampana için boş
    ds = -brake_side if brake_side else 1.0
    web = [(0.0, 0.0), (-0.0028, 0.0), (-0.0028, 0.0060), (-0.0012, 0.0085), (-0.0012, rr - 0.0045),
           (0.0010, rr - 0.0035), (0.0018, rr - 0.0045), (0.0018, 0.0085), (0.0035, 0.0060), (0.0035, 0.0)]
    web = [(ds * a, r) for a, r in web]
    if ds < 0:
        web = web[::-1]
    w = S.lathe(np.asarray(web), nseg=n, name="web", mat=M_HUB, axis="y", space="local", smooth_angle=40.0)
    parts.append(w)
    # rulman göbeği
    parts.append(cyl("boss", (0, -boss_a, 0), (0, boss_a, 0), 0.0052, M_HUB, 24, chamfer=0.0006))
    parts.append(cyl("cap_o", (0, ds * boss_a, 0), (0, ds * (boss_a + 0.00015), 0), 0.0034, M_STEEL, 20))
    # 5 kol (dış yüzde kabartma)
    for k in range(5):
        th = 2 * math.pi * k / 5 + 0.3
        e_r = np.array([math.cos(th), 0.0, math.sin(th)])
        e_t = np.array([-math.sin(th), 0.0, math.cos(th)])
        c = 0.5 * (0.0075 + rr - 0.0050) * e_r + np.array([0.0, ds * 0.0020, 0.0])
        R = np.column_stack([e_r, e_t, np.array([0.0, 1.0, 0.0])])
        parts.append(box("spoke", c, (0.5 * (rr - 0.0125), 0.0016, 0.0009), R, M_HUB, r=0.0007))
        bc = 0.0072 * e_r + np.array([0.0, ds * 0.0030, 0.0])
        parts.append(cyl("bolt", bc - np.array([0, ds * 0.0005, 0]), bc + np.array([0, ds * 0.0008, 0]), 0.0011, M_STEEL, 6))
    if brake_side:
        # kampana: jant içinde, fren tarafında
        drum = [(0.0, 0.0), (brake_side * 0.0005, 0.0), (brake_side * 0.0005, 0.0148), (brake_side * (Ab - 0.0010), 0.0148),
                (brake_side * (Ab - 0.0010), 0.0130), (brake_side * 0.0040, 0.0130), (brake_side * 0.0040, 0.0)]
        if brake_side < 0:
            drum = drum[::-1]
        parts.append(S.lathe(np.asarray(drum), nseg=n, name="drum", mat=M_DARK, axis="y", space="local", smooth_angle=40.0))
    return merge(name, parts, 40.0)


def wheel(name: str, lg: LegGeom, brake_side: float = 0.0) -> S.MeshData:
    return merge(name, [tyre("tyre", lg, 72 if not lg.is_nose else 64), hub("hub", lg, brake_side)], 40.0)


# =====================================================================================================
# Bacak parçaları (bacak çerçevesinde)
# =====================================================================================================
def _fork_arm(lg: LegGeom, side: float, origin_z: float) -> S.MeshData:
    """Çatal kolu: taçtan aksa daralan levha (bacak XZ düzleminde), ``side`` (±1) tarafta."""
    zc, za = lg.z["crown_top"] - 0.0010, lg.z["axle"]
    poly = stadium((0.0, zc), (0.0, za), 0.0062, 0.0048, 8)
    y0 = side * (0.5 * lg.W_w + FORK_GAP)
    y1 = y0 + side * FORK_T
    md = prism("arm", poly, (0, 0, -origin_z), (1, 0, 0), (0, 0, 1), min(-y0, -y1), max(-y0, -y1), M_GEAR, 40.0)
    return md


def slider_parts(lg: LegGeom) -> S.MeshData:
    """``U_GearSlider``: krom kayar boru, çatal tacı, kollar, aks, alt tork bağlantısı kulağı (+ fren tablası).
    Yerel: orijin = kayar boru ekseninde ``z["slider_top"]`` (bacak çerçevesi yönleri)."""
    o = np.array([0.0, lg.y_e, lg.z["slider_top"]])
    zt, zb = lg.z["crown_top"], lg.z["crown_bot"]
    hw = 0.5 * lg.W_w + FORK_GAP + FORK_T
    parts = [cyl("slider", np.array([0, lg.y_e, lg.z["slider_top"]]) - o, np.array([0, lg.y_e, zt + 0.0005]) - o,
                 lg.slider_r, M_STEEL, 28, chamfer=0.0005)]
    # taç: kollar arası köprü (yuvarlatılmış), kayar boru tabanı (manşon)
    parts.append(prism("crown", rrect(0.0065, hw, 0.0030), np.array([0, 0, 0]) - o, (1, 0, 0), (0, 1, 0), zb, zt, M_GEAR))
    parts.append(cyl("collar", np.array([0, lg.y_e, zt - 0.0010]) - o, np.array([0, lg.y_e, zt + 0.0045]) - o,
                     lg.slider_r + 0.0018, M_GEAR, 28, chamfer=0.0005))
    for sd in (-1.0, 1.0):
        md = _fork_arm(lg, sd, 0.0)
        parts.append(_xf(md, None, -o))
    # aks (gömme uçlu)
    ya = 0.5 * lg.W_w + FORK_GAP + FORK_T
    ax_r = 0.0025 if not lg.is_nose else 0.0020
    parts.append(cyl("axle", np.array([0, -ya + 0.0002, lg.z["axle"]]) - o, np.array([0, ya - 0.0002, lg.z["axle"]]) - o,
                     ax_r, M_STEEL, 16))
    for sd in (-1.0, 1.0):
        parts.append(cyl("axcap", np.array([0, sd * (ya - 0.0001), lg.z["axle"]]) - o,
                         np.array([0, sd * (ya + 0.00015), lg.z["axle"]]) - o, ax_r + 0.0008, M_STEEL, 16))
    # alt tork bağlantısı kulağı (tacın üstünde, n yönünde)
    lk = lg.link
    n = lk["n"]
    t = np.cross([0.0, 0.0, 1.0], n)
    pin = np.array([0.0, lg.y_e, 0.0]) + n * lk["r_lug"] + np.array([0.0, 0.0, lk["z_l"]])
    for sd in (-1.0, 1.0):
        c = pin + t * sd * (0.5 * LINK_T + 0.0011) + n * (-0.0020) + np.array([0, 0, -0.0012])
        R = np.column_stack([n, t, [0, 0, 1.0]])
        parts.append(_xf(box("lug", np.zeros(3), (0.0040, 0.0008, 0.0032), None, M_GEAR, r=0.0018), R, c - o))
    parts.append(cyl("lpin", pin - t * (0.5 * LINK_T + 0.0018) - o, pin + t * (0.5 * LINK_T + 0.0018) - o, 0.0011, M_STEEL, 12))
    # fren tablası (sabit, aks üzerinde) + tork kolu
    if not lg.is_nose:
        bs = lg.inboard_y
        A = 0.5 * lg.W_w
        Ab = TYRE["bead"] * A
        a0, a1 = bs * (Ab - 0.0006), bs * (Ab + 0.0006)
        parts.append(cyl("bplate", np.array([0, min(a0, a1), lg.z["axle"]]) - o, np.array([0, max(a0, a1), lg.z["axle"]]) - o,
                         0.0125, M_GEAR, 36, chamfer=0.0003))
        arm_c = np.array([-0.0065, bs * (Ab + 0.0002), lg.z["axle"] + 0.0080])
        parts.append(_xf(box("barm", np.zeros(3), (0.0020, 0.0006, 0.0085), None, M_GEAR, r=0.0015),
                         np.column_stack([[math.cos(0.6), 0, math.sin(0.6)], [0, 1, 0], [-math.sin(0.6), 0, math.cos(0.6)]]),
                         arm_c - o))
        # fren kablosu (taca kadar)
        c0 = np.array([-0.0035, bs * (Ab + 0.0002), lg.z["axle"] + 0.0115])
        c1 = np.array([-0.0050, bs * (hw - 0.0030), zb - 0.0005])
        parts.append(cyl("bcable", c0 - o, c1 - o, 0.0006, M_DARK, 8))
    return merge("slider", parts)


def strut_parts(lg: LegGeom) -> S.MeshData:
    """``U_GearStrut``: (ana) dönen trunnion kovanı + soket bloğu; üst silindir, salmastra somunu, üst tork kulağı,
    (ana) kapak braketleri; (burun) yönlendirme kolu. Yerel: orijin = pivot, bacak çerçevesi."""
    ye = lg.y_e
    parts = []
    z = lg.z
    if not lg.is_nose:
        pin_l = lg.F_leg.T @ _unit(lg.leg.retract_axis_b)
        half = _knuckle_half_len(lg)
        parts.append(cyl("knuckle", -pin_l * half, pin_l * half, MAIN_KNUCKLE_R, M_SKIN, 40, chamfer=0.0006))
        # soket bloğu: X'te geniş, Y'de boru kadar (toplu konumda tavan/kapak arası)
        parts.append(prism("socket", rrect(0.0092, lg.strut_r + 0.0003, 0.0040), (0, ye, 0), (1, 0, 0), (0, 1, 0),
                           z["sock_bot"], z["sock_top"], M_GEAR))
        for zz in (-0.010, -0.019):            # sıkma cıvataları (arka yüz)
            parts.append(cyl("bolt", (-0.0090, ye, zz), (-0.0102, ye, zz), 0.0014, M_STEEL, 6))
        # kapak braketleri (dış yana)
        for d in DOOR_BRACKET_Z:
            L_tab = _door_tab_len(lg, -d)
            if L_tab > 0.0008:
                y0 = ye + lg.sg * (lg.strut_r - 0.0010)
                y1 = ye + lg.sg * (lg.strut_r + L_tab)
                parts.append(prism("bracket", rrect(0.0028, 0.0032, 0.0010), (0, 0.5 * (y0 + y1), -d), (0, 0, 1), (1, 0, 0),
                                   -0.5 * abs(y1 - y0), 0.5 * abs(y1 - y0), M_GEAR))
                parts.append(cyl("band", (0, ye, -d - 0.0035), (0, ye, -d + 0.0035), lg.strut_r + 0.0004, M_GEAR, 28))
    else:
        parts.append(cyl("journal", (0, 0, z["house_top"] - 0.001), (0, 0, z["house_bot"] - 0.0005), 0.0072, M_STEEL, 24))
        # yönlendirme kolu (yatağın altında, geriye)
        hc = rrect(0.0085, 0.0085, 0.0084, 6)
        parts.append(prism("horn_hub", hc, (0, 0, 0), (1, 0, 0), (0, 1, 0), z["horn"] - 0.0035, z["horn"], M_GEAR))
        arm = stadium((0.0, 0.0), (-0.0165, 0.0), 0.0060, 0.0030, 6)
        parts.append(prism("horn_arm", arm, (0, 0, 0), (1, 0, 0), (0, 1, 0), z["horn"] - 0.0028, z["horn"] - 0.0006, M_GEAR))
        parts.append(cyl("ball", (-0.0165, 0, z["horn"] - 0.0006), (-0.0165, 0, z["horn"] + 0.0030), 0.0013, M_STEEL, 10))
        parts.append(lathe_along("balltop", [(0, 0), (0, 0.0016), (0.0010, 0.0019), (0.0022, 0.0012), (0.0026, 0)],
                                 (-0.0165, 0, z["horn"] + 0.0030), (0, 0, 1), M_STEEL, 12))
    # üst silindir ve salmastra somunu
    top = z["cyl_top"]
    parts.append(cyl("cyl", (0, ye, top), (0, ye, z["gland"] + 0.004), lg.strut_r, M_GEAR, 32, chamfer=0.0004))
    gl_r = lg.strut_r if not lg.is_nose else lg.strut_r + 0.0006
    parts.append(lathe_along("gland", [(0, 0), (0, gl_r - 0.0003), (0.0003, gl_r), (0.0042, gl_r), (0.0045, gl_r - 0.0006),
                                       (0.0058, lg.slider_r + 0.0012), (0.0062, lg.slider_r + 0.0004), (0.0062, 0)],
                             (0, ye, z["gland"] + 0.0040), (0, 0, -1), M_GEAR, 32))
    parts.append(lathe_along("wiper", [(0, 0), (0, lg.slider_r + 0.0010), (0.0012, lg.slider_r + 0.0005), (0.0012, 0)],
                             (0, ye, z["gland"] - 0.0022), (0, 0, -1), M_DARK, 24))
    # üst tork kulağı
    lk = lg.link
    n = lk["n"]
    t = np.cross([0.0, 0.0, 1.0], n)
    pin = np.array([0.0, ye, lk["z_u"]]) + n * lk["r_lug"]
    R = np.column_stack([n, t, [0, 0, 1.0]])
    for sd in (-1.0, 1.0):
        c = pin + t * sd * (0.5 * LINK_T + 0.0011) + n * (-0.0020) + np.array([0, 0, 0.0010])
        parts.append(_xf(box("lug", np.zeros(3), (0.0040, 0.0008, 0.0032), None, M_GEAR, r=0.0018), R, c))
    parts.append(cyl("upin", pin - t * (0.5 * LINK_T + 0.0018), pin + t * (0.5 * LINK_T + 0.0018), 0.0011, M_STEEL, 12))
    # hidrolik/fren hattı (ana): silindir boyunca ince kablo
    if not lg.is_nose:
        off = np.array([-(lg.strut_r + 0.0006), ye, 0.0])          # arka yüz (toplu konumda yuva eni yönünde)
        parts.append(cyl("line", off + [0, 0, z["sock_bot"] - 0.002], off + [0, 0, z["gland"] + 0.010], 0.0006, M_DARK, 8))
    return merge("strut", parts)


def knuckle_nose(lg: LegGeom) -> S.MeshData:
    """``U_GearKnuckle_N``: pimi taşıyan dönen blok + yönlendirme yatağı gövdesi (yerel: pivot, bacak çerçevesi)."""
    z = lg.z
    parts = [cyl("block", (0, -0.0085, 0), (0, 0.0085, 0), 0.0075, M_GEAR, 32, chamfer=0.0007),
             cyl("house", (0, 0, z["house_top"]), (0, 0, z["house_bot"]), 0.0102, M_GEAR, 32, chamfer=0.0008),
             box("web", (0, 0, -0.0060), (0.0060, 0.0085, 0.0045), None, M_GEAR, r=0.0020)]
    # servo yuvası (blok yan yüzünde, küçük)
    parts.append(box("servo", (0.0040, 0.0, -0.0145), (0.0055, 0.0098, 0.0045), None, M_DARK, r=0.0012))
    return merge("knuckle", parts)


def link_part(lg: LegGeom, upper: bool) -> S.MeshData:
    """Tork bağlantısı (yerel: kendi pimi; X = n (dışa), Z = bacak ekseni yukarı, Y = pim ekseni)."""
    lk = lg.link
    ell, a0 = lk["ell"], lk["alpha0"]
    if upper:
        knee = (ell * math.sin(a0), -ell * math.cos(a0))
    else:
        knee = (ell * math.sin(a0), ell * math.cos(a0))
    poly = stadium((0.0, 0.0), knee, lk["w_pin"] if upper else 0.92 * lk["w_pin"], lk["w_knee"], 8)
    parts = [prism("plate", poly, (0, 0, 0), (1, 0, 0), (0, 0, 1), -0.5 * LINK_T, 0.5 * LINK_T, M_GEAR, 40.0)]
    # gövde yivi (hafifletme) yerine kenar kalınlaştırma: dirsek pimi
    if upper:
        parts.append(cyl("kpin", (knee[0], -0.5 * LINK_T - 0.0005, knee[1]), (knee[0], 0.5 * LINK_T + 0.0005, knee[1]),
                         0.0010, M_STEEL, 10))
    return merge("link", parts)


# =====================================================================================================
# Sabit üniteler
# =====================================================================================================
def _slot_edges_s(y: float) -> tuple[float, float]:
    """Ana bacak yuvası açıklığının ``y``'deki ön/arka ``s`` kenarları (params._leg_slot doğrusu)."""
    sl = P._leg_slot("L")
    (s0a, y0a), (s0b, y0b) = sl[0], sl[1]
    (s1a, _), (s1b, _) = sl[3], sl[2]
    t = (abs(y) - abs(y0a)) / (abs(y0b) - abs(y0a))
    return s0a + t * (s0b - s0a), s1a + t * (s1b - s1a)


def _knuckle_half_len(lg: LegGeom) -> float:
    """Dönen kovanın yarı boyu: yuva astarının (eğik) iç yüzleri arasında, kovan yarıçapı genişliğinde."""
    py = abs(lg.leg.pivot[1])
    ys = np.linspace(py - MAIN_KNUCKLE_R, py + MAIN_KNUCKLE_R, 9)
    inset = S.BAY_INSET + S.BAY_WALL
    lo = max(_slot_edges_s(y)[0] for y in ys) + inset + SLOT_CLEAR
    hi = min(_slot_edges_s(y)[1] for y in ys) - inset - SLOT_CLEAR
    c = lg.leg.pivot[0]
    return float(min(c - lo, hi - c))


def _door_tab_len(lg: LegGeom, z_l: float) -> float:
    """Bacak kapağı braketi boyu: amortisör yüzeyinden kapak iç yüzüne (toplu konumda, params kapak yüzeyi)."""
    out_y = lg.sg
    lo, hi = 0.0, 0.03
    for _ in range(40):
        mid = 0.5 * (lo + hi)
        q = np.array([0.0, lg.y_e + out_y * (lg.strut_r + mid), z_l])
        w = lg.leg_to_world(q, lg.leg.retract_deg)
        s, y, zz = P.from_blender(*w)
        z_in = S.belly_z(s, y) + S.DOOR_T
        if zz - 0.0004 > z_in:                  # hâlâ kapağın üstünde → uzat
            lo = mid
        else:
            hi = mid
    return lo


def _spec_part(md: S.MeshData, lg: LegGeom, mirror: bool = True) -> S.MeshData:
    """Spec ekseninde (s, y, z; sol taraf için) kurulmuş ağı bacağın yerel Blender eksenine (orijin pivot, dünya
    yönleri) çevirir; sağ bacakta y aynalanır. Yüz yönleri hacim işaretiyle düzeltilir."""
    V = np.asarray(md.verts, float).copy()
    if mirror:
        V[:, 1] *= lg.sg
    md.verts = np.asarray(P.points_to_blender(V)) - lg.pivot_b
    md.space = "local"
    return md.oriented()


def main_unit(lg: LegGeom) -> S.MeshData:
    """``U_GearUnit_L/R``: ER-150 gövdesi (yuvanın önünde, kanat içinde, dihedral boyunca eğik), uç kapağı, düşey
    braket, trunnion pimi ve pim etrafında ön/arka trunnion kaportası (deri rengi). Yerel: orijin pivot, dünya eksenleri."""
    g = lg.leg
    py = abs(g.pivot[1])
    bw, bl, bh = ER150_BODY
    y0, y1 = (float(v) for v in P.SPEC["landing_gear"]["main"]["unit_span_y_m"])
    y1 = max(y1, py + 0.002)
    s1 = min(_slot_edges_s(y)[0] for y in np.linspace(y0, py + MAIN_KNUCKLE_R, 9)) - 0.0025
    s0 = s1 - bw
    dih = math.tan(_rad(P.WING_DIHEDRAL))
    ss, ys = np.linspace(s0, s1, 5), np.linspace(y0, y1, 9)
    tab = S.wing_table("L")
    base0 = max(S.belly_z(s, y) + 0.0021 - dih * (y - y0) for s in ss for y in ys)     # deri + 1,6 mm et + pay
    top0 = min(tab.z(s, y, "upper") - 0.0013 - dih * (y - y0) for s in ss for y in ys)
    h = min(bh, top0 - base0)
    zc0 = base0 + 0.5 * h
    ang = math.atan(dih)
    Rb = np.array([[1.0, 0.0, 0.0], [0.0, math.cos(ang), -math.sin(ang)], [0.0, math.sin(ang), math.cos(ang)]])
    yc = 0.5 * (y0 + y1)
    c_spec = np.array([0.5 * (s0 + s1), yc, zc0 + dih * (yc - y0)])
    parts = []
    body = box("er150", np.zeros(3), (0.5 * bw, 0.5 * (y1 - y0), 0.5 * h), None, M_DARK, r=0.0025)
    body.verts = body.verts @ Rb.T + c_spec
    parts.append(_spec_part(body, lg))
    cap = box("cap", np.zeros(3), (0.5 * bw + 0.0004, 0.010, 0.5 * h + 0.0004), None, M_GEAR, r=0.0025)
    cap.verts = cap.verts @ Rb.T + c_spec + Rb @ np.array([0.0, -0.5 * (y1 - y0) + 0.012, 0.0])
    parts.append(_spec_part(cap, lg))
    # düşey braket: gövde dış ucundan pime (yuvanın önünde, ön kaportanın içinde)
    z_bot = zc0 + dih * (py - y0) - 0.5 * h
    br = box("drop", np.zeros(3), (0.0035, 0.0040, 0.5 * (z_bot + 0.002 - g.pivot[2])), None, M_GEAR, r=0.0015)
    br.verts = br.verts + np.array([s1 - 0.0045, py, 0.5 * (z_bot + 0.002 + g.pivot[2])])
    parts.append(_spec_part(br, lg))
    # pim (kaportaların içinden geçer)
    sa, sb = _slot_edges_s(py)[0] - 0.010, _slot_edges_s(py)[1] + 0.010
    pa = np.asarray(P.to_blender(sa, g.pivot[1], g.pivot[2])) - lg.pivot_b
    pb = np.asarray(P.to_blender(sb, g.pivot[1], g.pivot[2])) - lg.pivot_b
    parts.append(cyl("pin", pa, pb, 0.5 * PIN_D, M_STEEL, 16))
    parts += trunnion_fairings(lg)
    return merge("unit", parts, 35.0)


TRUNNION_FAIRING_LEN = (0.034, 0.030)      # ön / arka sabit kaporta boyu. varsayım


def trunnion_fairings(lg: LegGeom) -> list[S.MeshData]:
    """Pim ekseni etrafında ön/arka sabit trunnion kaportaları (deri rengi). Yuvaya bakan uç yüzleri yuva
    kenarına paralel eğiktir (açıklığa girmez); serbest uçlar sivrilir."""
    g = lg.leg
    py = abs(g.pivot[1])
    r = MAIN_KNUCKLE_R
    out = []
    nseg = 40
    th = np.linspace(0, 2 * math.pi, nseg, endpoint=False)
    for which, Lf in (("front", TRUNNION_FAIRING_LEN[0]), ("aft", TRUNNION_FAIRING_LEN[1])):
        k = np.linspace(0.0, 1.0, 14)
        if which == "front":
            prof_r = r * np.sqrt(np.clip(1 - (1 - k) ** 2.2, 0, 1)) ** 0.9       # sivri ön uç → tam yarıçap
        else:
            prof_r = r * np.sqrt(np.clip(1 - k ** 2.6, 0, 1)) ** 0.9            # tam yarıçap → sivri arka uç
        mb = S.MeshBuilder(which, "local")
        rings = []
        for kk, rr in zip(k, prof_r):
            yy = py + rr * np.cos(th)                       # spec y (sol için)
            zz = g.pivot[2] + rr * np.sin(th)
            if which == "front":
                s_face = np.array([_slot_edges_s(v)[0] for v in yy]) - SLOT_CLEAR - 0.0004
                s_tip = s_face.min() - Lf
                ss = s_tip + kk * (s_face - s_tip)
            else:
                s_face = np.array([_slot_edges_s(v)[1] for v in yy]) + SLOT_CLEAR + 0.0004
                s_tip = s_face.max() + Lf
                ss = s_face + kk * (s_tip - s_face)
            pts = np.column_stack([ss, lg.sg * yy, zz])            # spec, tarafı belli
            if rr < 1e-6:
                rings.append(int(mb.add(pts[:1])[0]))
            else:
                rings.append(mb.add(pts))
        # loft
        for a, b in zip(rings[:-1], rings[1:]):
            if isinstance(a, int):
                mb.fan(a, b, M_SKIN, start=True)
            elif isinstance(b, int):
                mb.fan(b, a, M_SKIN, start=False)
            else:
                mb.strip(a, b, M_SKIN)
        if not isinstance(rings[0], int):
            mb.cap(rings[0], M_SKIN, start=True)
        if not isinstance(rings[-1], int):
            mb.cap(rings[-1], M_DARK, start=False)
        out.append(_spec_part(mb.build(35.0), lg, mirror=False))
    return out


def nose_unit(lg: LegGeom) -> S.MeshData:
    """``U_GearUnit_N``: ER-150 gövdesi kuyunun önünde (gövde içinde), boyuna; G10 plaka; pimi taşıyan iki yanak
    (kuyu ön duvarından geçer), çelik pim ve somunlar. Yerel: orijin pivot, dünya eksenleri."""
    g = lg.leg
    bw, bl, bh = ER150_BODY
    s1 = float(P.SPEC["landing_gear"]["nose"]["well"]["s_m"][0]) - 0.0015
    s0 = s1 - bl
    zc = g.pivot[2] - 0.0020
    parts = []

    def B(c_spec, half, mat, r=0.0020):
        md = box("b", np.zeros(3), half, None, mat, r=r)
        md.verts = md.verts + np.asarray(c_spec)
        return _spec_part(md, lg, mirror=False)

    parts.append(B((0.5 * (s0 + s1), 0.0, zc), (0.5 * bl, 0.5 * bw, 0.5 * bh), M_DARK, 0.0025))
    parts.append(B((s0 + 0.012, 0.0, zc), (0.010, 0.5 * bw + 0.0004, 0.5 * bh + 0.0004), M_GEAR, 0.0025))
    parts.append(B((0.5 * (s0 + s1), 0.0, zc + 0.5 * bh + 0.0015), (0.5 * bl + 0.006, 0.5 * bw + 0.006, 0.0015), M_PLATE, 0.002))
    for sd in (-1.0, 1.0):                                    # yanaklar: gövde ucundan pime
        poly = stadium((s1 - 0.004, zc), (g.pivot[0], g.pivot[2]), 0.0085, 0.0070, 8)
        md = prism("cheek", poly, (0, 0, 0), (1, 0, 0), (0, 0, 1), min(sd * 0.0095, sd * 0.0125),
                   max(sd * 0.0095, sd * 0.0125), M_GEAR, 40.0)
        # prism düzlemi (u=x→s, v=z) ve w = u×v = −y → y'yi düzelt
        md.verts[:, 1] *= -1.0
        parts.append(_spec_part(md, lg, mirror=False))
    pa = np.asarray(P.to_blender(g.pivot[0], -0.0135, g.pivot[2])) - lg.pivot_b
    pb = np.asarray(P.to_blender(g.pivot[0], 0.0135, g.pivot[2])) - lg.pivot_b
    parts.append(cyl("pin", pa, pb, 0.5 * PIN_D, M_STEEL, 16))
    for sd in (-1.0, 1.0):
        c = np.asarray(P.to_blender(g.pivot[0], sd * 0.0126, g.pivot[2])) - lg.pivot_b
        parts.append(cyl("nut", c, c + np.array([0, sd * 0.0012, 0]), 0.0030, M_STEEL, 6))
    return merge("unit", parts, 35.0)


# =====================================================================================================
# Blender nesneleri
# =====================================================================================================
def _mesh_obj(md: S.MeshData, name: str, col) -> bpy.types.Object:
    md.name = name
    md.space = "local"
    U.remove_object(name)
    me = U.mesh_from_data(md, name)
    ob = bpy.data.objects.new(name, me)
    col.objects.link(ob)
    ob.rotation_mode = "XYZ"
    ob["ucav_closed"] = bool(md.closed)
    return ob


def _empty(name: str, col, display: str = "PLAIN_AXES", size: float = 0.02) -> bpy.types.Object:
    U.remove_object(name)
    ob = bpy.data.objects.new(name, None)
    col.objects.link(ob)
    ob.empty_display_type = display
    ob.empty_display_size = size
    ob.rotation_mode = "XYZ"
    return ob


def _attach_local(child: bpy.types.Object, parent: bpy.types.Object, loc=(0.0, 0.0, 0.0), R_rel=None) -> None:
    """Birim ebeveyn ters matrisiyle bağlar: ``location`` ebeveyn yerel ekseninde, yön ``delta_rotation_euler``'de."""
    child.parent = parent
    child.matrix_parent_inverse = Matrix.Identity(4)
    child.location = Vector(tuple(map(float, loc)))
    child.rotation_euler = (0.0, 0.0, 0.0)
    child.delta_location = (0.0, 0.0, 0.0)
    child.delta_rotation_euler = Matrix(np.eye(3).tolist() if R_rel is None else np.asarray(R_rel, float).tolist()).to_euler("XYZ")


def _set_world_root(ob: bpy.types.Object, root: bpy.types.Object, origin_b, frame_b=None) -> None:
    """``U_Root``'a dünya konumu korunarak bağlar (airframe ile aynı kural)."""
    ob.location = Vector(tuple(map(float, origin_b)))
    ob.rotation_euler = (0.0, 0.0, 0.0)
    ob.delta_rotation_euler = Matrix((np.eye(3) if frame_b is None else np.asarray(frame_b, float)).tolist()).to_euler("XYZ")
    U.parent_keep_world(ob, root)


def door_down_matrix(name: str) -> Matrix:
    """``U_Door_<leg>_2``'nin takım açık pozundaki dünya matrisi (airframe toplu pozundan −retract_deg)."""
    d = next(x for x in P.gear_doors() if x.name == name)
    g = P.gear_leg(d.leg)
    F = S._frame_from_axes(g.retract_axis_b, [0.0, 0.0, 1.0])
    return (Matrix.Translation(Vector(g.pivot_b)) @ Matrix(F.tolist()).to_4x4()
            @ Matrix.Rotation(_rad(-g.retract_deg), 4, "X"))


def build(scene: bpy.types.Scene | None = None, *, verbose: bool = False) -> dict[str, bpy.types.Object]:
    """İniş takımını kurar (``airframe.build`` sonrası); ``{ad: nesne}`` döndürür. Tekrar çağrılabilir.

    Dinlenme pozu takım açık + amortisör yüksüzdür; statik çökme, kapak/bacak sıralaması, teker dönüşü ve burun
    yönlendirmesi ``rig.setup()`` sürücüleriyle gelir.
    """
    scene = scene or bpy.context.scene
    t0 = time.time()
    cols = U.ensure_collections(scene)
    col = cols[COLLECTION]
    root = bpy.data.objects.get("U_Root")
    if root is None:
        raise RuntimeError("U_Root yok: önce airframe.build() çağrılmalı")
    objs: dict[str, bpy.types.Object] = {}
    for name in LEGS:
        lg = leg_geom(name)
        X = name
        # ---------------------------------------------------------------- sabit ünite
        unit = _mesh_obj(main_unit(lg) if not lg.is_nose else nose_unit(lg), f"U_GearUnit_{X}", col)
        _set_world_root(unit, root, lg.pivot_b)
        unit["ucav_role"] = "gear_unit"
        objs[unit.name] = unit
        # ---------------------------------------------------------------- pivot
        piv = _empty(f"U_GearPivot_{X}", col, "ARROWS", 0.03)
        _set_world_root(piv, root, lg.pivot_b, lg.F_pivot)
        piv["ucav_role"] = "gear_pivot"
        piv["gear_retract_deg"] = float(lg.leg.retract_deg)
        piv["gear_leg"] = X
        objs[piv.name] = piv
        R_rel = lg.F_pivot.T @ lg.F_leg                       # pivot → bacak çerçevesi
        # ---------------------------------------------------------------- bacak çerçevesi
        if lg.is_nose:
            kn = _mesh_obj(knuckle_nose(lg), f"U_GearKnuckle_{X}", col)
            _attach_local(kn, piv, (0, 0, 0), R_rel)
            objs[kn.name] = kn
            steer = _empty(f"U_GearSteer_{X}", col, "SINGLE_ARROW", 0.04)
            _attach_local(steer, kn, (0, 0, 0))
            steer["ucav_role"] = "gear_steer"
            steer["gear_steer_max_deg"] = float(P.SPEC["landing_gear"]["nose"].get("steer_deg", 30.0))
            objs[steer.name] = steer
            frame_parent = kn
            strut = _mesh_obj(strut_parts(lg), f"U_GearStrut_{X}", col)
            _attach_local(strut, steer, (0, 0, 0))
        else:
            strut = _mesh_obj(strut_parts(lg), f"U_GearStrut_{X}", col)
            _attach_local(strut, piv, (0, 0, 0), R_rel)
            frame_parent = strut
        strut["ucav_role"] = "gear_strut"
        objs[strut.name] = strut
        ref = _empty(f"U_GearAxleRef_{X}", col, "SPHERE", 0.006)
        _attach_local(ref, frame_parent, lg.axle_l)
        ref.hide_render = True
        objs[ref.name] = ref
        # ---------------------------------------------------------------- kayar boru, teker, bağlantılar
        z0 = lg.z["slider_top"]
        sl = _mesh_obj(slider_parts(lg), f"U_GearSlider_{X}", col)
        _attach_local(sl, strut, (0.0, lg.y_e, z0))
        sl["ucav_role"] = "gear_slider"
        sl["gear_z0"] = z0
        sl["gear_stroke_m"] = lg.stroke
        sl["gear_static_compression_m"] = float(lg.leg.static_compression)
        objs[sl.name] = sl
        wh = _mesh_obj(wheel(f"U_GearWheel_{X}", lg, 0.0 if lg.is_nose else lg.inboard_y), f"U_GearWheel_{X}", col)
        _attach_local(wh, sl, np.array([0.0, -lg.y_e, -lg.L - z0]))
        wh["ucav_role"] = "gear_wheel"
        wh["gear_wheel_r_m"] = float(lg.R_w)
        objs[wh.name] = wh
        lk = lg.link
        n = lk["n"]
        Rl = np.column_stack([n, np.cross([0.0, 0.0, 1.0], n), [0.0, 0.0, 1.0]])
        pu = np.array([0.0, lg.y_e, lk["z_u"]]) + n * lk["r_lug"]
        lu = _mesh_obj(link_part(lg, True), f"U_GearLinkU_{X}", col)
        _attach_local(lu, strut, pu, Rl)
        pl = np.array([0.0, lg.y_e, lk["z_l"]]) + n * lk["r_lug"] - np.array([0.0, lg.y_e, z0])
        ll = _mesh_obj(link_part(lg, False), f"U_GearLinkL_{X}", col)
        _attach_local(ll, sl, pl, Rl)
        for ob in (lu, ll):
            ob["ucav_role"] = "gear_link"
            ob["gear_link_len_m"] = float(lk["ell"])
            ob["gear_link_d0_m"] = float(lk["D0"])
            ob["gear_link_alpha0_rad"] = float(lk["alpha0"])
            objs[ob.name] = ob
        # ---------------------------------------------------------------- bacak kapağı
        if not lg.is_nose:
            door = bpy.data.objects.get(f"U_Door_{X}_2")
            if door is not None:
                Wd = door_down_matrix(door.name)
                Ws = Matrix.Translation(Vector(lg.pivot_b)) @ Matrix(lg.F_leg.tolist()).to_4x4()
                rel = Ws.inverted() @ Wd
                door.parent = strut
                door.matrix_parent_inverse = Matrix.Identity(4)
                door.location = rel.to_translation()
                door.rotation_euler = (0.0, 0.0, 0.0)
                door.delta_location = (0.0, 0.0, 0.0)
                door.delta_rotation_euler = rel.to_3x3().to_euler("XYZ")
                door["ucav_gear_rigged"] = True
                objs[door.name] = door
        if verbose:
            print(f"[gear {time.time() - t0:5.1f} s] {X}")
    bpy.context.view_layer.update()
    return objs


# =====================================================================================================
# Raporlar ve kontroller (doğrulama, testler)
# =====================================================================================================
def compression(name: str, axle_world_z: float) -> float:
    """Amortisör sıkışması (bacak boyunca): teker tabanı ``GROUND_Z`` altına inmesin; [0, strok]."""
    lg = leg_geom(name)
    c = (P.GROUND_Z - (axle_world_z - lg.R_w)) / math.cos(_rad(lg.leg.rake_deg))
    return float(min(max(c, 0.0), lg.stroke))


def link_angle(name: str, comp: float) -> float:
    """Tork bağlantısı açısı (bacak ekseninden, rad) — ``rig`` sürücüsünün Python karşılığı."""
    lk = leg_geom(name).link
    return math.acos(max(-1.0, min(1.0, (lk["D0"] - comp) / (2 * lk["ell"]))))


MOVING_PREFIX = ("U_GearStrut_", "U_GearSlider_", "U_GearWheel_", "U_GearLinkU_", "U_GearLinkL_", "U_GearKnuckle_")


def _known_airframe_contact(part: str, other: str, s: float) -> str | None:
    """Airframe kaynaklı bilinen temaslar (bkz. modül raporu): kuyu arka duvarındaki testere dişi uçları ve bacak
    kapağının pivot ötesine taşan dış ucu."""
    if part.startswith("U_GearWheel_N") and other.startswith(("U_Bay_N", "U_Fuselage")) and s > 0.655:
        return "nose_well_aft_teeth"
    if part.startswith("U_GearWheel_") and other.startswith(("U_Bay_", "U_WingCenter_", "U_Fairing_Root_")) and s > 1.364:
        return "main_well_aft_teeth"
    if part.startswith("U_Door_") or other.startswith("U_Door_L_2") or other.startswith("U_Door_R_2"):
        return "strut_door_outline"
    return None


def clearance_report(gear_values=(1.0, 0.9, 0.75, 0.5, 0.25, 0.15, 0.0), lift: float = 0.3) -> dict:
    """Dünya uzayında BVH üçgen kesişimleri: hareketli takım parçaları (+ bacak kapakları) ↔ diğer bütün ``U_*``
    ağları, ``gear`` değerleri boyunca (``rig.setup`` gerekli). ``U_Root`` geçici olarak ``lift`` kadar kaldırılır
    (amortisör yüksüz), sonra eski hâline döner. Dönüş: ``{"new": [...], "known": [...]}``; her satır
    ``(gear, parça, diğer, üçgen_çifti, s_merkez, etiket)``. ``new`` boş olmalıdır."""
    from mathutils.bvhtree import BVHTree
    root = bpy.data.objects["U_Root"]
    saved = (root.location.copy(), float(root.get("gear", 1.0)))
    anim = root.animation_data.action if root.animation_data else None
    if anim is not None:
        root.animation_data.action = None
    root.location.z += lift

    def bvh(ob):
        dg = bpy.context.evaluated_depsgraph_get()
        ev = ob.evaluated_get(dg)
        me = ev.to_mesh()
        mw = ev.matrix_world
        V = [mw @ v.co for v in me.vertices]
        F = [tuple(p.vertices) for p in me.polygons]
        ev.to_mesh_clear()
        return BVHTree.FromPolygons(V, F), V

    moving = [o.name for o in bpy.data.objects if o.type == "MESH" and o.name.startswith(MOVING_PREFIX)]
    moving += [n for n in ("U_Door_L_2", "U_Door_R_2") if n in bpy.data.objects]
    others = [o.name for o in bpy.data.objects if o.type == "MESH" and o.name.startswith("U_") and o.name not in moving
              and not o.name.startswith("U_Stand")]
    excl = {("U_GearStrut_L", "U_GearUnit_L"), ("U_GearStrut_R", "U_GearUnit_R"), ("U_GearKnuckle_N", "U_GearUnit_N")}
    out = {"new": [], "known": []}
    try:
        for gv in gear_values:
            root["gear"] = float(gv)
            root.update_tag()
            bpy.context.view_layer.update()
            Rinv = root.matrix_world.inverted()
            trees = {n: bvh(bpy.data.objects[n]) for n in others}
            for pn in moving:
                tp, Vp = bvh(bpy.data.objects[pn])
                for on, (to, _) in trees.items():
                    if (pn, on) in excl:
                        continue
                    pairs = tp.overlap(to)
                    if not pairs:
                        continue
                    c = np.mean([tuple(Rinv @ Vp[i]) for i, _ in pairs[:64]], axis=0) + np.asarray(P.U_ROOT_B)
                    s = float(-c[0])
                    tag = _known_airframe_contact(pn, on, s)
                    out["known" if tag else "new"].append((gv, pn, on, len(pairs), round(s, 4), tag))
    finally:
        root.location = saved[0]
        root["gear"] = saved[1]
        if anim is not None:
            root.animation_data.action = anim
        root.update_tag()
        bpy.context.view_layer.update()
    return out


def report() -> dict:
    """Bacak başına türetilmiş ölçüler (rapor/test): ofset, istasyonlar, bağlantı, ünite boyutu."""
    out = {}
    for n in LEGS:
        lg = leg_geom(n)
        lk = lg.link
        out[n] = {"strut_offset_m": lg.y_e, "leg_length_m": lg.L, "stroke_m": lg.stroke, "tyre_width_m": lg.W_w,
                  "tyre_width_params_m": lg.leg.wheel_w,
                  "static_compression_m": lg.leg.static_compression, "crown_gap_m": CROWN_GAP,
                  "link": {"len_m": lk["ell"], "d0_m": lk["D0"], "alpha0_deg": math.degrees(lk["alpha0"]),
                           "knee_r_m": lk["r_lug"] + lk["ell"] * math.sin(lk["alpha0"])},
                  "fork_half_width_m": 0.5 * lg.W_w + FORK_GAP + FORK_T}
        if not lg.is_nose:
            out[n]["knuckle_half_len_m"] = _knuckle_half_len(lg)
            out[n]["door_tabs_m"] = [_door_tab_len(lg, -d) for d in DOOR_BRACKET_Z]
    return out
