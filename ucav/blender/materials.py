"""YELKOVAN YK-38 — malzemeler, boya şemaları, panel çizgileri ve işaretler (bpy 4.5): ``build(livery="standart")``.

``params.MATERIALS`` içindeki her ``UM_*`` adı için Cycles PBR düğüm ağacı kurulur. Gövde/takım modülleri
yüzlere malzeme yuvalarını önceden atar (yoksa ``util.get_material`` basit Principled yer tutucu oluşturur);
bu modül **aynı adlı veri bloğunu** bulur ve düğüm ağacını yeniden yazar — eksik (magenta) malzeme kalmaz.
Bilinmeyen yeni bir ``UM_*`` adı spec'e eklenirse genel Principled tarifiyle kurulur.

Renk, pürüzlülük ve metaliklik değerlerinin TEK kaynağı ``spec.yaml → materials``'tır (``params.MATERIALS``); bu modül
yalnız tarifi (düğüm ağacını) seçer. Yüzlere hangi malzemenin gideceğine gövde/takım modülleri rol tablosuyla
(``params.MATERIAL_ROLES``) karar verir: kuyu ve kapak iç yüzleri ``UM_Liner`` (ışımasız açık gri astar), kapak
kenarları ``UM_Seal`` (koyu conta: kapalı kapak çevresi ince koyu çizgi okunur), taret pencere çerçeveleri
``UM_Bezel``, taret yakası ``UM_SkinBottom``, susturucu ``UM_Exhaust``; ``UM_Orange`` yalnız uyarı rolündedir.

Malzeme tarifleri (renkler ``spec.yaml → materials``; aşağıdaki ince ayarlar "varsayım"dır)
------------------------------------------------------------------------------------------
* **Boya** (``UM_SkinTop`` RAL 7035, ``UM_SkinBottom`` RAL 9003, ``UM_Accent`` RAL 7016, ``UM_Turquoise``
  RAL 5018, ``UM_Antenna``): saten boya — nesne koordinatında çok hafif ton (±%2,5) ve pürüzlülük (±0,04)
  dalgalanması, ince vernik (üst 0,25; alt yüzey 0,10 — düşük parlaklık), portakal kabuğu mikro tümseği. Doku
  nesne koordinatındadır: kumanda yüzeyi döndüğünde ya da uçak uçtuğunda boya "kaymaz".
* **Panel çizgileri** (``UCAV_PanelLines`` düğüm grubu; ``UM_SkinTop``/``UM_SkinBottom``/``UM_Accent``): baskı
  segment eklerindeki V-oluklar (``panel_stations``: gövde ``s`` = ``printprep.fuselage_cuts()``, kanat ``|y|``,
  stabilize ``|y|``, dikey ``h``) 0,5 mm yarı genişlik + 0,4 mm geçişle koyulaşan oluk ve tümsek; gerçek servo yuvalarının (``printprep.
  SERVO_STATIONS``) alt yüzde 46 × 30 mm R4 servis kapakları ve köşelerde Ø2 vida başları. Hangi düzlemin hangi
  nesneye uygulandığı nesne özelliği ``ucav_panel``'dan okunur (1 gövde/kaporta, 2 kanat, 3 stabilize, 4 dikey;
  ``build`` yazar). Hareketli yüzeyler, kapaklar ve aviyonik kapağı çizgisizdir.
* ``UM_Orange`` (RAL 2004 saf turuncu, floresan değil, ışımasız): yalnız pala uçları, dikeylerdeki pervane
  düzlemi bandı ve bakım sehpası (uyarı rolü).
* ``UM_Liner`` (kuyu/kapak iç astarı, ipek mat, vernik 0,15), ``UM_Seal`` (kapak kenarı/conta, mat koyu).
* ``UM_PropDisc``: dönen pervanenin zaman ortalamalı diski (``U_PropDisc``, rig kurar): Transparent + koyu
  Principled karışımı; örtme = nesne özelliği ``ucav_disc`` (rig devirle sürer, 0…0,35) × yarıçap profili (pala
  bölgesi soluk, uçtaki ``UM_Orange`` bandı daha belirgin halka). Dururken nesne render dışıdır.
* ``UM_SmokeHatch``: füme parlak PETG — iki arayüzlü geçirgenlik (toplam ≈ %10), cam gibi yansıma.
* ``UM_PACF`` ve ``UM_Nozzle`` (lüle halkası, soğutma havası çıkışı): boyasız PA-CF — mat, ince baskı dokusu,
  hafif "sheen"; ısı tonu YOK (egzoz sol yanaktaki susturucudan çıkar).
* ``UM_Exhaust``: susturucu — paslanmaz çelik, gürültüyle karışan bronz/mavi ısı tonu (spec ``heat_tint``).
* ``UM_Prop``: kayın pala, mat siyah boyalı (kalın vernik, damar yok); uçlar ``UM_Orange``.
* ``UM_Spinner``: boyalı alüminyum, saten antrasit (ayna yansıması yok).
* ``UM_Gear`` (anodize Al), ``UM_Hub``, ``UM_Steel`` (oleo krom / pitot — pürüzlülük ≤ 0,14), ``UM_Engine``.
* ``UM_Tire``: kauçuk — mat, tozlu "sheen", ince tümsek. ``UM_TPU``: mat siyah TPU.
* ``UM_TurretBody``: saten koyu gri (#2A2D30) — top formu okunur, "delik" gibi görünmez. ``UM_Bezel``: siyah eloksal
  pencere çerçevesi. ``UM_SensorGlass``: pencere nesnesinin yerel +Y yarısı EO (çok koyu cam, vernik 0,6, ince AR
  filmi → soluk mor/yeşil yansıma), −Y yarısı IR (germanyum: koyu gri, yarı metalik, ince film → altın/mor ışıltı).
  Kendi ışıması yoktur; yalnız yansır. ``UM_Carbon``: 2×2 dimi örgü karbon + vernik.
* **Işıklar** (``UM_NavRed``, ``UM_NavGreen``, ``UM_Strobe``, ``UM_StatusLED``): parlak lens + ışıma.
  Işıma şiddeti = spec ``emission`` × nesne özelliği ``ucav_emission`` (Attribute düğümü, nesne türü). ``rig``
  her ışık nesnesinin ``ucav_emission`` değerini sürer (seyrüsefer: ``nav_lights``, çakarlar: ``strobe``);
  böylece aynı malzemeyi paylaşan iniş ışığı çakmaz. Özellik yoksa ``build`` 1,0 yazar. ``UM_StatusLED`` ayrıca
  ``ucav_status_gate`` ile çarpılır: ``U_Root``'ta ayrı bir ``status_led`` özelliği yoksa (halka hâlâ
  ``nav_lights``'a bağlıysa) 0 → taret halkası sönük koyu conta gibi görünür; gerçek EO/IR taretler ışımaz.

Boya şemaları (``LIVERIES``)
----------------------------
* ``"standart"`` (``"standard"`` da kabul edilir): spec renkleri — üst RAL 7035, alt RAL 9003.
* ``"taktik"``: yalnız render için koyu düşük görünürlüklü gri (spec ``materials.liveries.taktik``: üst #4A5055
  ≈ RAL 7015, alt #6F777C, vurgu/anten #2A2E32, ince çizgi #4E5B60, spinner #3A3F44 …); mat boya (pürüzlülük 0,62,
  vernik 0,08); işaretler zeminden AÇIK (#8C9499). Spec'te eksik ad varsa ``TAKTIK_DEFAULT`` kullanılır. Uyarı: koyu
  boya güneşte LW-PLA'yı ısıtır (Tg ≈ 55 °C) — uçacak gövde için önerilmez (baskı raporundaki ısı/boya kuralı).

İşaretler (``build_decals``)
----------------------------
Yazılar Blender metin eğrisinden ağa çevrilir (Liberation Sans Bold, %86 dar; yoksa DejaVu/FreeSans/yerleşik
yazı tipi), ≤ 2,5 mm kenarlı üçgenlere bölünür ve BVH ışın izdüşümüyle ev sahibi yüzeye oturtulur (eğriliğe
uyar). Her işaret 0,15 mm kalınlığında kapalı ince katıdır (üst yüzü deriden 0,35 mm yukarıda; manifold,
gölge düşürmez). Yazı malzemesi ``UM_Accent`` (nesne özelliği ``ucav_decal_mix`` ile hedef renge açılır →
standartta düşük kontrastlı gri, taktikte zeminden açık gri).

* ``U_Decal_YK38_L/R``: dikeylerin dış yüzünde "YK-38" (yükseklik 26 mm), sabit dikey kısmında.
* ``U_Decal_Name_L/R``: kuyruk konisi yanında, chine çizgisine paralel "YELKOVAN" (22 mm).
* ``U_Decal_PropBand_L/R``: dikeylerin iç yüzünde pervane düzlemi uyarı bandı (``UM_Orange``, 12 mm), bant
  ``PROP.plane_s_at_z`` ile 5° aşağı itki eğimini izler.
* ``U_Decal_CG_L/R``: gövdenin iki yanında CG istasyonunda (``params.CG.s``; kot CG + 20 mm, kök filetosunun üstü)
  Ø18 mm siyah/beyaz çeyrekli ağırlık merkezi işareti (koyu çeyrekler ``UM_Accent``, açık çeyrekler ``UM_SkinBottom``).

Yazılı işlevsel şablonlar (ADIM ATMA, YAKIT, PERVANE, STATİK PORT …) gövde modülünün ``U_Stencil_*`` nesneleridir
(spec ``details.markings.stencils``); ``build`` yalnız ``UM_Accent`` kullananlara şema kontrastını (``ucav_decal_mix``)
yazar. Tüm deri ince LW-PLA olduğundan yürüme yolu çizgisi yoktur.

Hepsi ``UCAV_Details`` koleksiyonunda, ev sahibine dünya dönüşümü korunarak bağlıdır (``build`` tekrar
çağrılabilir; eski işaretler silinip yeniden kurulur). Baskı modülü bu nesneleri kullanmaz.

Sıra: ``airframe.build()`` → ``gear.build()`` → ``rig.setup()`` → ``materials.build()`` (sıra değişse de çalışır).
"""
from __future__ import annotations

import math
import os
from dataclasses import dataclass

import bpy
import bmesh
import numpy as np
from mathutils import Vector
from mathutils.bvhtree import BVHTree

from .. import params as P
from . import util as U

LIGHT_MATERIALS = ("UM_NavRed", "UM_NavGreen", "UM_Strobe", "UM_StatusLED")
EMISSION_PROP = "ucav_emission"           # ışık nesnesi çarpanı (rig sürer)
STATUS_GATE_PROP = "ucav_status_gate"     # taret durum LED'i kapısı (0 = sönük; bkz. modül belgesi)
DECAL_MIX_PROP = "ucav_decal_mix"         # yazı rengini hedef renge açma oranı (0 = saf antrasit)
PANEL_PROP = "ucav_panel"                 # panel çizgisi ailesi (1 gövde, 2 kanat, 3 stabilize, 4 dikey)

# İnce ayarlar (varsayım) — boya katmanı
PAINT_COAT = 0.25                         # vernik ağırlığı (saten)
PAINT_COAT_BOTTOM = 0.10                  # alt yüzey: düşük parlaklık (gölgede gökyüzü yansıması az)
PAINT_COAT_ROUGH = 0.22
PAINT_TONE_VAR = 0.025                    # ± ton dalgalanması
PAINT_ROUGH_VAR = 0.04                    # ± pürüzlülük dalgalanması
ORANGE_GLOW = 0.0                         # eski floresans taklidi kaldırıldı (gerçek boya ışımaz)
SMOKE_TOTAL_T = 0.10                      # füme kapak toplam geçirgenliği (iki arayüz)
TAKTIK_PAINT = {"roughness": 0.62, "coat": 0.08}   # taktik: mat düşük görünürlük

# ---- Taktik boya şeması: spec ``materials.liveries.taktik`` geçerlidir; spec'te eksik ad için yedek değerler ----
TAKTIK_DEFAULT: dict[str, str] = {
    "UM_SkinTop": "#4A5055", "UM_SkinBottom": "#6F777C", "UM_Accent": "#2A2E32", "UM_Antenna": "#2A2E32",
    "UM_Turquoise": "#4E5B60", "UM_Spinner": "#3A3F44", "UM_StatusLED": "#2E8C93",
}
TAKTIK_DECAL = "#8C9499"                  # taktik işaret rengi: zeminden açık gri
# Pervane diski (``UM_PropDisc``; varsayım): pala bölgesi ve uç bandı örtme çarpanları (× ``ucav_disc`` ≤ 0,35), kenar geçişi
DISC_PROFILE = {"blade": 0.38, "tip": 0.65, "fade_m": 0.004}
DECAL_MIX = {"standart": 0.42, "taktik": 1.0}      # yazı → hedef renk oranı
EO_TINT = "#08090A"                      # EO penceresi: çok koyu, nötr cam (camgöbeği ton yok)
EO_AR_NM = 105.0                         # MgF₂ AR filmi (λ₀ ≈ 580 nm) → soluk mor/yeşil artık yansıma
IR_TINT = "#2A2C30"                      # germanyum penceresi
IR_FILM_NM = 270.0                       # DLC/AR filmi → altın/mor ışıltı

# Panel çizgileri (varsayım; oluk spec ``print.joint_groove_mm`` 0,4 mm — render'da biraz geniş tutulur)
PANEL_HALF_W = 0.0005                     # çizgi yarı genişliği (m)
PANEL_FALLOFF = 0.0004                    # kenar geçişi (m)
PANEL_DARK = 0.45                         # çizgi ortasında koyulaşma ağırlığı (MULTIPLY → ≈ %29 koyu)
PANEL_DARK_COLOR = (0.355, 0.36, 0.37)
PANEL_BUMP = (0.6, 0.0003)                # oluk tümseği (güç, mesafe)
PANEL_OBJECTS = {"U_Fuselage": 1, "U_Cowl": 1, "U_WingCenter_L": 2, "U_WingCenter_R": 2, "U_WingOuter_L": 2,
                 "U_WingOuter_R": 2, "U_Tip_L": 2, "U_Tip_R": 2, "U_Stab": 3, "U_Fin_L": 4, "U_Fin_R": 4}
SERVO_HATCH = (0.046, 0.030, 0.004)       # servis kapağı (açıklık yönü, veter yönü, köşe yarıçapı) m
SERVO_HATCH_X_C = 0.465                   # kapak merkezi veter oranı (ana kiriş %28 ile arka kiriş %65 arası)
FASTENER = (0.004, 0.0010)                # vida başı: köşeden içeri, yarıçap (m)

FONT_CANDIDATES = (
    "/usr/share/fonts/truetype/liberation/LiberationSans-Bold.ttf",
    "/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf",
    "/usr/share/fonts/truetype/freefont/FreeSansBold.ttf",
    "C:/Windows/Fonts/arialbd.ttf",
    "/Library/Fonts/Arial Bold.ttf",
)
TEXT_CONDENSE = 0.86                      # yazı yatay daraltma
DECAL_OFFSET = 0.00035                    # yüzeyden kaldırma (m) — işaretin üst yüzü
DECAL_THICK = 0.00015                     # işaret kalınlığı (m): kapalı ince katı (manifold)
DECAL_MAX_EDGE = 0.0025                   # izdüşümden önce üçgen kenarı üst sınırı (m)


# =====================================================================================================
# Yardımcılar: renk, düğüm ağacı
# =====================================================================================================
def hex_to_linear(hx: str) -> tuple[float, float, float]:
    """``#RRGGBB`` (sRGB) → doğrusal RGB."""
    hx = hx.lstrip("#")
    c = [int(hx[i:i + 2], 16) / 255.0 for i in (0, 2, 4)]
    return tuple(x / 12.92 if x <= 0.04045 else ((x + 0.055) / 1.055) ** 2.4 for x in c)


def _rgba(c) -> tuple[float, float, float, float]:
    return (float(c[0]), float(c[1]), float(c[2]), 1.0)


def _scale(c, k: float):
    return tuple(min(1.0, max(0.0, float(x) * k)) for x in c[:3])


class _Tree:
    """Küçük düğüm ağacı kurucusu: düğüm ekle, giriş yaz, bağla. ``nodes.clear()`` ile başlar."""

    def __init__(self, nt: bpy.types.NodeTree):
        self.nt = nt
        nt.nodes.clear()

    def node(self, idname: str, x: float = 0.0, y: float = 0.0, label: str | None = None, ins: dict | None = None,
             **props):
        n = self.nt.nodes.new(idname)
        n.location = (x, y)
        if label:
            n.label = label
        for k, v in props.items():
            setattr(n, k, v)
        for k, v in (ins or {}).items():
            n.inputs[k].default_value = v
        return n

    def link(self, out_socket, in_socket) -> None:
        self.nt.links.new(out_socket, in_socket)

    # ---- sık kullanılan parçalar
    def coords(self, x: float = -1400.0, y: float = 0.0):
        return self.node("ShaderNodeTexCoord", x, y)

    def noise(self, vec, scale: float, detail: float = 2.0, rough: float = 0.5, x: float = -1100.0, y: float = 0.0,
              dims: str = "3D"):
        n = self.node("ShaderNodeTexNoise", x, y, noise_dimensions=dims,
                      ins={"Scale": scale, "Detail": detail, "Roughness": rough})
        self.link(vec, n.inputs["Vector"])
        return n

    def map_range(self, value, lo: float, hi: float, a: float = 0.0, b: float = 1.0, x: float = -800.0,
                  y: float = 0.0, clamp: bool = True):
        n = self.node("ShaderNodeMapRange", x, y, clamp=clamp,
                      ins={"From Min": a, "From Max": b, "To Min": lo, "To Max": hi})
        self.link(value, n.inputs["Value"])
        return n.outputs["Result"]

    def mix_rgb(self, fac, a, b, x: float = -500.0, y: float = 0.0, blend: str = "MIX"):
        """``ShaderNodeMix`` (RGBA). ``a``/``b`` renk ya da soket."""
        n = self.node("ShaderNodeMix", x, y, data_type="RGBA", blend_type=blend)
        if isinstance(fac, (int, float)):
            n.inputs[0].default_value = float(fac)
        else:
            self.link(fac, n.inputs[0])
        for idx, v in ((6, a), (7, b)):
            if isinstance(v, (tuple, list)):
                n.inputs[idx].default_value = _rgba(v) if len(v) == 3 else tuple(v)
            else:
                self.link(v, n.inputs[idx])
        return n.outputs[2]

    def math(self, op: str, a, b=None, x: float = -600.0, y: float = 0.0, clamp: bool = False):
        n = self.node("ShaderNodeMath", x, y, operation=op, use_clamp=clamp)
        for i, v in enumerate((a, b)):
            if v is None:
                continue
            if isinstance(v, (int, float)):
                n.inputs[i].default_value = float(v)
            else:
                self.link(v, n.inputs[i])
        return n.outputs[0]

    def bump(self, height, strength: float, distance: float, x: float = -400.0, y: float = -500.0):
        n = self.node("ShaderNodeBump", x, y, ins={"Strength": strength, "Distance": distance})
        self.link(height, n.inputs["Height"])
        return n.outputs["Normal"]

    def principled(self, x: float = 0.0, y: float = 0.0, **ins):
        n = self.node("ShaderNodeBsdfPrincipled", x, y)
        n.name = "Principled BSDF"
        for k, v in ins.items():
            key = k.replace("_", " ")
            sock = n.inputs[key]
            if isinstance(v, (int, float)):
                sock.default_value = float(v)
            elif isinstance(v, (tuple, list)):
                sock.default_value = _rgba(v) if len(v) == 3 and sock.type == "RGBA" else tuple(v)
            else:
                self.link(v, sock)
        return n

    def output(self, shader, x: float = 300.0, y: float = 0.0):
        o = self.node("ShaderNodeOutputMaterial", x, y, target="ALL")
        self.link(shader, o.inputs["Surface"])
        return o


# =====================================================================================================
# Panel çizgileri (düğüm grubu)
# =====================================================================================================
PANEL_GROUP = "UCAV_PanelLines"


def panel_stations() -> dict[str, list[float]]:
    """Panel çizgisi düzlemleri = baskı segment ekleri (fiziksel V-oluklar; baskı raporundaki ``print_cuts`` ile
    aynı): gövde ``s`` ``printprep.fuselage_cuts()``'tan (burun konisi ayrımı 0,148 ve kuyu/eyer/hava alığından
    kaydırılmış ekler dahil), kanat ``|y|`` (kök bloğu iç ucu ve uç kapağının dış ucu hariç), stabilize ``|y|``
    (orta ek ve uç hariç), dikey ``h`` (kök ve uç hariç) ``params.print_cuts()``'tan."""
    c = P.print_cuts()
    try:
        from .printprep import fuselage_cuts
        fus = [float(v) for v in fuselage_cuts()]
    except Exception:                                         # pragma: no cover — baskı modülü yoksa plan değerleri
        fus = [float(v) for v in c["fuselage_s"][1:-1]]
    wing = sorted({round(float(v), 6) for v in c["root_block_y"][1:] + c["wing_panel_y"][:-1]})
    return {"fuselage_s": fus, "wing_y": wing, "stab_y": [float(v) for v in c["stab_y"][1:-1]],
            "fin_h": [float(v) for v in c["fin_h"][1:-1]]}


def _servo_stations() -> dict[str, float]:
    try:
        from .printprep import SERVO_STATIONS                 # gerçek servo yuvaları (baskı modülü)
        return dict(SERVO_STATIONS)
    except Exception:                                         # pragma: no cover — baskı modülü yoksa
        return {"Aileron": 1.30, "FlapOut": 0.68, "Elevator": 0.17}


def access_panels() -> list[dict]:
    """Servis kapakları (plan koordinatı, Blender X ve |Y|): kanat altı (dış flap ve kanatçık servoları) ve
    stabilize altı (irtifa servosu). Her biri: aile (2 kanat, 3 stabilize), merkez, yarı boyutlar, köşe yarıçapı."""
    ss = _servo_stations()
    hx, hy, r = SERVO_HATCH
    out = []
    for key in ("FlapOut", "Aileron"):
        if key in ss:
            y = float(ss[key])
            st = P.wing_station(y)
            s_c = st.le_s + SERVO_HATCH_X_C * (st.te_s - st.le_s)
            out.append({"name": key, "panel": 2, "x": -s_c, "y": y, "hs": 0.5 * hy, "hy": 0.5 * hx, "r": r})
    if "Elevator" in ss:
        y = float(ss["Elevator"])
        st = P.stab_station(y)
        s_c = st.le_s + 0.36 * st.chord
        hs = min(0.5 * hy, 0.16 * st.chord)
        out.append({"name": "Elevator", "panel": 3, "x": -s_c, "y": y, "hs": hs, "hy": 0.5 * hx, "r": r})
    return out


def _dist_to_stations(T, coord, stations, x: float, y: float):
    """min_k |coord − v_k| (düğümlerle)."""
    out = None
    for k, v in enumerate(stations):
        d = T.math("ABSOLUTE", T.math("SUBTRACT", coord, float(v), x=x, y=y - 60 * k), x=x + 160, y=y - 60 * k)
        out = d if out is None else T.math("MINIMUM", out, d, x=x + 320, y=y - 60 * k)
    return out


def _groove(T, d, x: float, y: float, half: float = PANEL_HALF_W):
    """Uzaklık → oluk maskesi (çizgi ortasında 1, ``half`` + geçişten sonra 0)."""
    return T.map_range(d, 1.0, 0.0, half, half + PANEL_FALLOFF, x=x, y=y)


def _rounded_box(T, X, AY, hp: dict, x: float, y: float):
    """Yuvarlatılmış dikdörtgen kenar çizgisi + köşelerde vida başları (plan koordinatında maske)."""
    px = T.math("ABSOLUTE", T.math("SUBTRACT", X, hp["x"], x=x, y=y), x=x + 150, y=y)
    py = T.math("ABSOLUTE", T.math("SUBTRACT", AY, hp["y"], x=x, y=y - 80), x=x + 150, y=y - 80)
    r = hp["r"]
    qx = T.math("SUBTRACT", px, hp["hs"] - r, x=x + 300, y=y)
    qy = T.math("SUBTRACT", py, hp["hy"] - r, x=x + 300, y=y - 80)
    v = T.node("ShaderNodeCombineXYZ", x + 450, y - 40)
    T.link(T.math("MAXIMUM", qx, 0.0, x=x + 380, y=y + 60), v.inputs["X"])
    T.link(T.math("MAXIMUM", qy, 0.0, x=x + 380, y=y - 140), v.inputs["Y"])
    ln = T.node("ShaderNodeVectorMath", x + 600, y - 40, operation="LENGTH")
    T.link(v.outputs[0], ln.inputs[0])
    inside = T.math("MINIMUM", T.math("MAXIMUM", qx, qy, x=x + 450, y=y - 200), 0.0, x=x + 600, y=y - 200)
    sd = T.math("SUBTRACT", T.math("ADD", ln.outputs["Value"], inside, x=x + 750, y=y - 100), r, x=x + 900, y=y - 100)
    edge = _groove(T, T.math("ABSOLUTE", sd, None, x=x + 1050, y=y - 100), x + 1200, y - 100)
    inset, fr = FASTENER
    cv = T.node("ShaderNodeCombineXYZ", x + 450, y - 300)
    T.link(T.math("SUBTRACT", px, hp["hs"] - inset, x=x + 300, y=y - 260), cv.inputs["X"])
    T.link(T.math("SUBTRACT", py, hp["hy"] - inset, x=x + 300, y=y - 340), cv.inputs["Y"])
    cl = T.node("ShaderNodeVectorMath", x + 600, y - 300, operation="LENGTH")
    T.link(cv.outputs[0], cl.inputs[0])
    dot = T.map_range(cl.outputs["Value"], 0.85, 0.0, fr * 0.7, fr, x=x + 1050, y=y - 300)
    return T.math("MAXIMUM", edge, dot, x=x + 1350, y=y - 150)


def panel_group(force: bool = False) -> bpy.types.NodeTree:
    """``UCAV_PanelLines`` düğüm grubu → çıkış ``Mask`` (0…1). Nesne koordinatı = spec koordinatı (statik deri
    parçaları ``U_Root`` altında birim dönüşümlüdür: Blender X = −s, Y = y, Z = z); aile nesne özelliği
    ``ucav_panel``'dan seçilir (yoksa 0 → çizgi yok). Düzlemler değişmediyse var olan grup döner; yeniden kurulumda
    arayüz soketi korunur (grubu kullanan malzemelerin bağlantıları kopmaz)."""
    st = panel_stations()
    hatches = access_panels()
    sig = repr((st, hatches, PANEL_HALF_W, PANEL_FALLOFF, FASTENER))
    ng = bpy.data.node_groups.get(PANEL_GROUP)
    if ng is not None and not force and ng.get("ucav_sig") == sig and ng.nodes.get("UCAV_Out") is not None:
        return ng
    if ng is None:
        ng = bpy.data.node_groups.new(PANEL_GROUP, "ShaderNodeTree")
    if not any(it.item_type == "SOCKET" and it.in_out == "OUTPUT" and it.name == "Mask" for it in ng.interface.items_tree):
        ng.interface.new_socket("Mask", in_out="OUTPUT", socket_type="NodeSocketFloat")
    T = _Tree(ng)
    tc = T.node("ShaderNodeTexCoord", -2600, 0)
    sep = T.node("ShaderNodeSeparateXYZ", -2400, 0)
    T.link(tc.outputs["Object"], sep.inputs[0])
    nsep = T.node("ShaderNodeSeparateXYZ", -2400, -400)
    T.link(tc.outputs["Normal"], nsep.inputs[0])
    X, Y, Z = sep.outputs["X"], sep.outputs["Y"], sep.outputs["Z"]
    s_c = T.math("MULTIPLY", X, -1.0, x=-2200, y=200)
    ay = T.math("ABSOLUTE", Y, None, x=-2200, y=0)
    att = T.node("ShaderNodeAttribute", -2400, 600, attribute_type="OBJECT", attribute_name=PANEL_PROP)

    def sel(k: int, yy: float):
        o = T.math("COMPARE", att.outputs["Fac"], float(k), x=-600, y=yy)
        o.node.inputs[2].default_value = 0.25
        return o

    lower = T.math("LESS_THAN", nsep.outputs["Z"], -0.3, x=-2200, y=-400)
    # gövde: istasyon düzlemleri (yakıt kapağı, statik portlar vb. gövde modülünün ``U_Stencil_*`` nesneleridir)
    m1 = _groove(T, _dist_to_stations(T, s_c, st["fuselage_s"], -2000, 1600), -1200, 1600)
    # kanat ve stabilize: |y| düzlemleri + alt yüzde servis kapakları
    m2 = _groove(T, _dist_to_stations(T, ay, st["wing_y"], -2000, 1000), -1200, 1000)
    m3 = _groove(T, _dist_to_stations(T, ay, st["stab_y"], -2000, 300), -1200, 300) if st["stab_y"] else None
    yy = -1000
    for hp in hatches:
        hm = T.math("MULTIPLY", _rounded_box(T, X, ay, hp, -3000, yy), lower, x=-1100, y=yy)
        if hp["panel"] == 2:
            m2 = T.math("MAXIMUM", m2, hm, x=-900, y=yy)
        elif m3 is not None:
            m3 = T.math("MAXIMUM", m3, hm, x=-900, y=yy)
        else:
            m3 = hm
        yy -= 500
    # dikey: h = (|y| − y_kök)·sin(eğim) + (z − z_kök)·cos(eğim)
    fn = P.SPEC["tail"]["fin"]
    cant = math.radians(float(fn["cant_deg"]))
    h = T.math("ADD", T.math("MULTIPLY", T.math("SUBTRACT", ay, float(fn["y_root_m"]), x=-2100, y=-200), math.sin(cant),
                             x=-1950, y=-200),
               T.math("MULTIPLY", T.math("SUBTRACT", Z, float(fn["z_root_m"]), x=-2100, y=-300), math.cos(cant),
                      x=-1950, y=-300), x=-1800, y=-250)
    m4 = _groove(T, _dist_to_stations(T, h, st["fin_h"], -1650, -250), -1200, -250) if st["fin_h"] else None
    total = None
    for k, m, yy in ((1, m1, 1600), (2, m2, 1000), (3, m3, 300), (4, m4, -250)):
        if m is None:
            continue
        t = T.math("MULTIPLY", m, sel(k, yy), x=-400, y=yy)
        total = t if total is None else T.math("MAXIMUM", total, t, x=-200, y=yy)
    go = T.node("NodeGroupOutput", 100, 0)
    go.name = "UCAV_Out"
    T.link(total, go.inputs["Mask"])
    ng["ucav_sig"] = sig
    return ng


# =====================================================================================================
# Malzeme tarifleri
# =====================================================================================================
def _paint(T: _Tree, color, rough: float, *, coat: float = PAINT_COAT, coat_rough: float = PAINT_COAT_ROUGH,
           glow: float = 0.0, decal_mix=None, metallic: float = 0.0, panels: bool = False, tone: bool = True):
    """Saten boya: ton + pürüzlülük dalgalanması, ince vernik, portakal kabuğu tümseği; ``panels`` → panel çizgileri."""
    tc = T.coords()
    obj = tc.outputs["Object"]
    if tone:
        n_tone = T.noise(obj, 2.2, 3.0, 0.55, y=300)
        base = T.mix_rgb(n_tone.outputs["Fac"], _scale(color, 1.0 - PAINT_TONE_VAR),
                         _scale(color, 1.0 + PAINT_TONE_VAR), y=300)
    else:
        base = T.mix_rgb(0.0, color, color, y=300)
    if decal_mix is not None:                       # işaret nesneleri: hedef renge doğru aç
        at = T.node("ShaderNodeAttribute", -800, 600, attribute_type="OBJECT", attribute_name=DECAL_MIX_PROP)
        base = T.mix_rgb(at.outputs["Fac"], base, decal_mix, x=-300, y=400)
    n_r = T.noise(obj, 7.0, 2.0, 0.5, y=0)
    rgh = T.map_range(n_r.outputs["Fac"], rough - PAINT_ROUGH_VAR, rough + PAINT_ROUGH_VAR, 0.3, 0.7, y=0)
    n_peel = T.noise(obj, 420.0, 1.0, 0.4, y=-500)
    nrm = T.bump(n_peel.outputs["Fac"], 0.035, 0.0002)
    if panels:
        g = T.node("ShaderNodeGroup", -900, 800, label="Panel çizgileri")
        g.node_tree = panel_group()
        mask = g.outputs["Mask"]
        base = T.mix_rgb(T.math("MULTIPLY", mask, PANEL_DARK, x=-650, y=800), base, PANEL_DARK_COLOR, x=-200, y=600,
                         blend="MULTIPLY")
        b2 = T.node("ShaderNodeBump", -200, -700, ins={"Strength": PANEL_BUMP[0], "Distance": PANEL_BUMP[1]})
        T.link(T.math("MULTIPLY", mask, -1.0, x=-650, y=-700), b2.inputs["Height"])
        T.link(nrm, b2.inputs["Normal"])
        nrm = b2.outputs["Normal"]
    ins = dict(Base_Color=base, Roughness=rgh, Metallic=metallic, Coat_Weight=coat, Coat_Roughness=coat_rough,
               Coat_IOR=1.5, Normal=nrm, Coat_Normal=nrm)
    if glow > 0:
        ins.update(Emission_Color=color, Emission_Strength=glow)
    bsdf = T.principled(**ins)
    T.output(bsdf.outputs[0])


def _generic(T: _Tree, spec: P.MaterialSpec, color):
    """Spec değerlerinden düz Principled (bilinmeyen yeni ad için)."""
    ins = dict(Base_Color=color, Roughness=spec.roughness, Metallic=spec.metallic, IOR=spec.ior)
    if spec.transmission > 0:
        ins["Transmission_Weight"] = spec.transmission
    if spec.emission > 0:
        ins.update(Emission_Color=color, Emission_Strength=spec.emission)
    T.output(T.principled(**ins).outputs[0])


def _smoke(T: _Tree, spec, color):
    """Füme PETG: arayüz başına renk = √(toplam geçirgenlik) ölçekli spec tonu."""
    c = np.asarray(color, float)
    per_iface = math.sqrt(SMOKE_TOTAL_T)
    tint = tuple(c / max(c.max(), 1e-6) * per_iface)
    bsdf = T.principled(Base_Color=tint, Roughness=min(spec.roughness, 0.05), IOR=spec.ior,
                        Transmission_Weight=1.0, Coat_Weight=0.0)
    T.output(bsdf.outputs[0])


def _pacf(T: _Tree, spec, color):
    """Boyasız PA-CF: mat, ince baskı dokusu (tümsek), hafif "sheen"."""
    tc = T.coords()
    obj = tc.outputs["Object"]
    n1 = T.noise(obj, 900.0, 2.0, 0.6, y=-400)
    nrm = T.bump(n1.outputs["Fac"], 0.12, 0.0003)
    n2 = T.noise(obj, 9.0, 2.0, 0.5, y=0)
    rgh = T.map_range(n2.outputs["Fac"], spec.roughness - 0.06, spec.roughness + 0.04, 0.3, 0.7)
    bsdf = T.principled(Base_Color=color, Roughness=rgh, Metallic=0.0, Sheen_Weight=0.25, Sheen_Roughness=0.4,
                        Sheen_Tint=(0.6, 0.6, 0.62), Normal=nrm)
    T.output(bsdf.outputs[0])


def _exhaust(T: _Tree, spec, color):
    """Paslanmaz çelik susturucu: düşük frekanslı gürültüyle bronz/mavi ısı tonu lekeleri (spec ``heat_tint``),
    ince fırçalama pürüzlülüğü."""
    tc = T.coords()
    obj = tc.outputs["Object"]
    nz = T.noise(obj, 38.0, 3.0, 0.6, y=200)
    f = T.map_range(nz.outputs["Fac"], 0.0, 1.0, 0.42, 0.68, y=200)
    ramp = T.node("ShaderNodeValToRGB", -500, 200)
    tints = [hex_to_linear(h) for h in spec.extra.get("heat_tint", ["#6A4A2E", "#3B4F7A"])]
    els = ramp.color_ramp.elements
    els[0].position, els[0].color = 0.0, _rgba(color)
    els[1].position, els[1].color = 0.55, _rgba(tuple(0.5 * (a + b) for a, b in zip(color, tints[0])))
    e = els.new(0.82)
    e.color = _rgba(tints[0])
    e = els.new(1.0)
    e.color = _rgba(tints[1] if len(tints) > 1 else tints[0])
    T.link(f, ramp.inputs["Fac"])
    n2 = T.noise(obj, 160.0, 2.0, 0.5, y=-200)
    rgh = T.map_range(n2.outputs["Fac"], spec.roughness - 0.05, spec.roughness + 0.08, 0.3, 0.7, y=-200)
    bsdf = T.principled(Base_Color=ramp.outputs["Color"], Metallic=spec.metallic, Roughness=rgh, Anisotropic=0.3)
    T.output(bsdf.outputs[0])


def _metal(T: _Tree, spec, color, rough: float | None = None, var: float = 0.03, aniso: float = 0.0,
           metallic: float | None = None):
    tc = T.coords()
    n = T.noise(tc.outputs["Object"], 40.0, 3.0, 0.55)
    r = spec.roughness if rough is None else rough
    rgh = T.map_range(n.outputs["Fac"], max(0.02, r - var), r + var, 0.3, 0.7)
    ins = dict(Base_Color=color, Metallic=spec.metallic if metallic is None else metallic, Roughness=rgh)
    if aniso:
        ins["Anisotropic"] = aniso
    T.output(T.principled(**ins).outputs[0])


def _rubber(T: _Tree, spec, color):
    tc = T.coords()
    obj = tc.outputs["Object"]
    n = T.noise(obj, 1500.0, 2.0, 0.6, y=-400)
    nrm = T.bump(n.outputs["Fac"], 0.15, 0.0002)
    n2 = T.noise(obj, 30.0, 2.0, 0.5)
    rgh = T.map_range(n2.outputs["Fac"], spec.roughness - 0.12, spec.roughness - 0.02, 0.3, 0.7)
    bsdf = T.principled(Base_Color=color, Roughness=rgh, Sheen_Weight=0.35, Sheen_Roughness=0.5,
                        Sheen_Tint=(0.55, 0.55, 0.55), Normal=nrm)
    T.output(bsdf.outputs[0])


def _turret(T: _Tree, spec, color):
    """Taret gövdesi: saten koyu gri boya, ince vernik (form okunur), hafif "sheen"."""
    bsdf = T.principled(Base_Color=color, Roughness=spec.roughness, Coat_Weight=0.10, Coat_Roughness=0.30,
                        Sheen_Weight=0.10, Sheen_Roughness=0.5)
    T.output(bsdf.outputs[0])


def _sensor_glass(T: _Tree, spec, color):
    """EO (yerel +Y, iskele) ve IR (yerel −Y, sancak) pencereleri aynı malzemede; ayrım nesne koordinatıyla.
    İkisi de ışımasızdır: EO çok koyu, vernikli (AR filmi → soluk mor/yeşil), IR germanyum (yarı metalik,
    ince film → altın/mor ışıltı)."""
    tc = T.coords()
    sep = T.node("ShaderNodeSeparateXYZ", -1100, 0)
    T.link(tc.outputs["Object"], sep.inputs[0])
    is_eo = T.math("GREATER_THAN", sep.outputs["Y"], 0.0, x=-900, y=0)
    ir = T.principled(-300, -350, Base_Color=hex_to_linear(IR_TINT), Metallic=0.3, Roughness=0.08,
                      Thin_Film_Thickness=IR_FILM_NM, Thin_Film_IOR=2.0)
    eo = T.principled(-300, 350, Base_Color=hex_to_linear(EO_TINT), Roughness=max(0.02, spec.roughness), IOR=spec.ior,
                      Coat_Weight=0.6, Coat_Roughness=0.02, Thin_Film_Thickness=EO_AR_NM, Thin_Film_IOR=1.38)
    mix = T.node("ShaderNodeMixShader", 100, 0)
    T.link(is_eo, mix.inputs["Fac"])
    T.link(ir.outputs[0], mix.inputs[1])
    T.link(eo.outputs[0], mix.inputs[2])
    T.output(mix.outputs[0], 350, 0)


def _carbon(T: _Tree, spec, color):
    """2×2 dimi karbon örgüsü (≈ 3 mm demet) + vernik: iki yönlü dalga bantlarının farkı."""
    tc = T.coords()
    obj = tc.outputs["Object"]
    w1 = T.node("ShaderNodeTexWave", -900, 200, wave_type="BANDS", bands_direction="DIAGONAL",
                ins={"Scale": 330.0, "Distortion": 0.0})
    T.link(obj, w1.inputs["Vector"])
    ck = T.node("ShaderNodeTexChecker", -900, -100, ins={"Scale": 330.0})
    T.link(obj, ck.inputs["Vector"])
    f = T.math("MULTIPLY", w1.outputs["Fac"], ck.outputs["Fac"], x=-650, y=100)
    base = T.mix_rgb(f, _scale(color, 0.8), _scale(color, 1.6), x=-450, y=100)
    bsdf = T.principled(Base_Color=base, Roughness=0.3, Anisotropic=0.5, Coat_Weight=1.0, Coat_Roughness=0.05)
    T.output(bsdf.outputs[0])


def _light_lens(T: _Tree, spec, color):
    """Işık lensi: parlak kubbe + ışıma (spec şiddeti × nesne ``ucav_emission``; durum LED'inde × kapı)."""
    at = T.node("ShaderNodeAttribute", -700, -300, attribute_type="OBJECT", attribute_name=EMISSION_PROP)
    strength = T.math("MULTIPLY", at.outputs["Fac"], float(spec.emission), x=-450, y=-300)
    if spec.name == "UM_StatusLED":
        gate = T.node("ShaderNodeAttribute", -700, -500, attribute_type="OBJECT", attribute_name=STATUS_GATE_PROP)
        strength = T.math("MULTIPLY", strength, gate.outputs["Fac"], x=-300, y=-400)
        seal = P.MATERIALS.get("UM_Seal")
        lens = tuple(seal.rgb_linear) if seal is not None else hex_to_linear("#1C1D1E")   # sönükken koyu conta
    elif spec.name == "UM_Strobe":
        lens = tuple(0.35 + 0.45 * float(c) for c in color)
    else:
        lens = _scale(color, 0.55)
    bsdf = T.principled(Base_Color=lens, Roughness=0.12, Coat_Weight=1.0, Coat_Roughness=0.03,
                        Emission_Color=color, Emission_Strength=strength)
    T.output(bsdf.outputs[0])


def _prop_disc(T: _Tree, spec, color):
    """Dönen pervane diski: Transparent ↔ Principled karışımı. Örtme = nesne ``ucav_disc`` (rig) × yarıçap profili
    r = √(y² + z²) (nesne koordinatı; orijin göbekte, yerel X = mil): pala bölgesi ``DISC_PROFILE["blade"]``, uçtaki
    turuncu bant (``P.PROP``: yarıçap − ``tip_band_m`` … yarıçap) tam; iki kenarda ``fade_m`` yumuşak geçiş. Renk:
    pala bölgesi koyu duman (spec rengi), uç halkası ``UM_Orange``."""
    pr = P.PROP
    r1 = float(pr.radius)
    band = float(P.SPEC["propulsion"]["prop"].get("tip_band_m", 0.02))
    r_in, r_tip, fade = 0.035, r1 - band, float(DISC_PROFILE["fade_m"])
    tc = T.coords()
    sep = T.node("ShaderNodeSeparateXYZ", -1300, 0)
    T.link(tc.outputs["Object"], sep.inputs[0])
    yz = T.node("ShaderNodeCombineXYZ", -1100, 0)
    T.link(sep.outputs["Y"], yz.inputs["Y"])
    T.link(sep.outputs["Z"], yz.inputs["Z"])
    ln = T.node("ShaderNodeVectorMath", -950, 0, operation="LENGTH")
    T.link(yz.outputs[0], ln.inputs[0])
    r = ln.outputs["Value"]
    # profil: iç kenarda 0 → pala bölgesi → uç bandında 1 → dış kenarda 0
    inner = T.map_range(r, 0.0, 1.0, r_in, r_in + fade, x=-750, y=200)
    outer = T.map_range(r, 1.0, 0.0, r1 - fade, r1, x=-750, y=0)
    tip = T.map_range(r, 0.0, 1.0, r_tip - 0.5 * fade, r_tip + 0.5 * fade, x=-750, y=-200)
    lvl = T.math("ADD", float(DISC_PROFILE["blade"]),
                 T.math("MULTIPLY", tip, float(DISC_PROFILE["tip"]) - float(DISC_PROFILE["blade"]), x=-550, y=-200),
                 x=-400, y=-100)
    prof = T.math("MULTIPLY", T.math("MULTIPLY", inner, outer, x=-550, y=100), lvl, x=-250, y=0)
    at = T.node("ShaderNodeAttribute", -550, 400, attribute_type="OBJECT", attribute_name="ucav_disc")
    fac = T.math("MULTIPLY", prof, at.outputs["Fac"], x=-100, y=200, clamp=True)
    orange = P.MATERIALS.get("UM_Orange")
    tip_col = tuple(orange.rgb_linear) if orange is not None else (0.8, 0.1, 0.01)
    col = T.mix_rgb(tip, color, tip_col, x=-250, y=-350)
    bsdf = T.principled(-50, -300, Base_Color=col, Roughness=spec.roughness, Metallic=0.0)
    tr = T.node("ShaderNodeBsdfTransparent", -50, -50)
    mix = T.node("ShaderNodeMixShader", 150, 0)
    T.link(fac, mix.inputs["Fac"])
    T.link(tr.outputs[0], mix.inputs[1])
    T.link(bsdf.outputs[0], mix.inputs[2])
    T.output(mix.outputs[0], 350, 0)


RECIPES = {
    "UM_SkinTop": "paint_panels", "UM_SkinBottom": "paint_panels", "UM_Accent": "paint_decal",
    "UM_Turquoise": "gloss", "UM_Antenna": "paint", "UM_Orange": "warning", "UM_SmokeHatch": "smoke",
    "UM_PACF": "pacf", "UM_Nozzle": "pacf", "UM_Prop": "prop", "UM_Spinner": "spinner", "UM_Gear": "anod",
    "UM_Hub": "metal", "UM_Steel": "chrome", "UM_Engine": "metal", "UM_Tire": "rubber", "UM_TPU": "rubber",
    "UM_TurretBody": "turret", "UM_SensorGlass": "glass", "UM_Carbon": "carbon", "UM_Liner": "liner",
    "UM_Seal": "seal", "UM_Bezel": "bezel", "UM_Exhaust": "exhaust", "UM_Erosion": "erosion",
    "UM_Hazard": "warning", "UM_PropDisc": "propdisc",
    "UM_NavRed": "light", "UM_NavGreen": "light", "UM_Strobe": "light", "UM_StatusLED": "light",
}


def _recipe(name: str) -> str:
    """Ad → tarif. Bilinmeyen ad → genel Principled."""
    return RECIPES.get(name, "generic")


# =====================================================================================================
# Boya şemaları
# =====================================================================================================
def _livery_key(livery: str) -> str:
    key = (livery or "standart").strip().lower()
    if key in ("standart", "standard", "std"):
        return "standart"
    if key in ("taktik", "tactical", "tac"):
        return "taktik"
    raise KeyError(f"bilinmeyen boya şeması: {livery!r} (standart | taktik)")


def taktik_hex() -> dict[str, str]:
    """Taktik şema renkleri: spec ``materials.liveries.taktik``'teki ``UM_*`` değerleri; spec'te olmayan adlar için
    ``TAKTIK_DEFAULT``."""
    tk = {k: str(v) for k, v in (P.LIVERIES.get("taktik", {}) or {}).items() if str(k).startswith("UM_")}
    return {**TAKTIK_DEFAULT, **tk}


def livery_colors(livery: str = "standart") -> dict[str, tuple[float, float, float]]:
    """Boya şemasına göre her ``UM_*`` için doğrusal RGB."""
    key = _livery_key(livery)
    out = {n: tuple(s.rgb_linear) for n, s in P.MATERIALS.items()}
    if key == "taktik":
        for n, hx in taktik_hex().items():
            if n in out:
                out[n] = hex_to_linear(hx)
    return out


LIVERIES = ("standart", "taktik")


# =====================================================================================================
# Kurulum
# =====================================================================================================
def decal_target_color(colors: dict, livery: str = "standart") -> tuple[float, float, float]:
    """Yazı rengi hedefi. Standart: antrasitten üst yüzey rengine doğru %62 (orta gri; ``DECAL_MIX`` 0,42 ile
    düşük kontrast). Taktik: zeminden açık gri ``TAKTIK_DECAL`` (düşük görünürlüklü açık işaret)."""
    if _livery_key(livery) == "taktik":
        return hex_to_linear(TAKTIK_DECAL)
    top = np.asarray(colors["UM_SkinTop"], float)
    acc = np.asarray(colors["UM_Accent"], float)
    return tuple(acc + 0.62 * (top - acc))


def build_material(name: str, color=None, decal_target=None, livery: str = "standart") -> bpy.types.Material:
    """Tek bir ``UM_*`` malzemesini (veri bloğu varsa onu) kurar ve döndürür. ``decal_target``: yalnız
    ``UM_Accent`` için işaret nesnelerinin açılacağı renk (None → standart boya şemasından)."""
    spec = P.MATERIALS[name]
    color = tuple(color if color is not None else spec.rgb_linear)
    taktik = _livery_key(livery) == "taktik"
    mat = bpy.data.materials.get(name) or bpy.data.materials.new(name)
    mat.use_nodes = True
    if mat.node_tree.animation_data is not None:      # eski düğümlere bağlı sürücüler (yer tutucudan kalma)
        mat.node_tree.animation_data_clear()
    T = _Tree(mat.node_tree)
    kind = _recipe(name)
    paint_r = TAKTIK_PAINT["roughness"] if taktik else spec.roughness
    if kind in ("paint", "paint_panels", "paint_decal"):
        coat = TAKTIK_PAINT["coat"] if taktik else (PAINT_COAT_BOTTOM if name == "UM_SkinBottom" else PAINT_COAT)
        dm = (decal_target or decal_target_color(livery_colors("standart"))) if kind == "paint_decal" else None
        _paint(T, color, paint_r, coat=coat, decal_mix=dm, panels=kind != "paint")
    elif kind == "gloss":
        _paint(T, color, min(spec.roughness, 0.32), coat=0.6, coat_rough=0.08)
    elif kind == "warning":
        _paint(T, color, spec.roughness, glow=ORANGE_GLOW)
    elif kind == "liner":
        _paint(T, color, spec.roughness, coat=0.15, coat_rough=0.3)
    elif kind == "erosion":                          # hücum kenarı aşınma şeridi: daha parlak, sert boya/film
        _paint(T, color, spec.roughness, coat=0.5, coat_rough=0.15)
    elif kind == "seal":
        _paint(T, color, spec.roughness, coat=0.05, coat_rough=0.4, tone=False)
    elif kind == "prop":
        _paint(T, color, spec.roughness, coat=0.6, coat_rough=0.2, tone=False)
    elif kind == "spinner":
        _paint(T, color, spec.roughness, coat=0.6, coat_rough=0.12, metallic=spec.metallic)
    elif kind == "smoke":
        _smoke(T, spec, color)
    elif kind == "pacf":
        _pacf(T, spec, color)
    elif kind == "exhaust":
        _exhaust(T, spec, color)
    elif kind == "anod":
        _metal(T, spec, color, aniso=0.35)
    elif kind == "chrome":
        _metal(T, spec, color, rough=min(spec.roughness, 0.14), var=0.02)
    elif kind == "metal":
        _metal(T, spec, color)
    elif kind == "bezel":
        _metal(T, spec, color, var=0.05)
    elif kind == "rubber":
        _rubber(T, spec, color)
    elif kind == "turret":
        _turret(T, spec, color)
    elif kind == "glass":
        _sensor_glass(T, spec, color)
    elif kind == "carbon":
        _carbon(T, spec, color)
    elif kind == "light":
        _light_lens(T, spec, color)
    elif kind == "propdisc":
        _prop_disc(T, spec, color)
    else:
        _generic(T, spec, color)
    # görünüm penceresi (Solid) rengi
    mat.diffuse_color = _rgba(color if kind != "smoke" else _scale(color, 2.0))
    mat.roughness = float(paint_r if kind.startswith("paint") else spec.roughness)
    mat.metallic = float(spec.metallic)
    mat["ucav_recipe"] = kind
    return mat


def ensure_light_props() -> list[str]:
    """Işık malzemesi kullanan nesnelerde ``ucav_emission`` yoksa 1,0 yazar (rig sonradan sürer). Durum LED'i
    nesnelerine ``ucav_status_gate`` yazar: ``U_Root``'ta ``status_led`` özelliği varsa 1 (rig ayrı sürer), yoksa 0."""
    root = bpy.data.objects.get("U_Root")
    gate = 1.0 if root is not None and "status_led" in root.keys() else 0.0
    out = []
    for ob in bpy.data.objects:
        if ob.type != "MESH" or ob.data is None:
            continue
        names = [m.name for m in ob.data.materials if m is not None]
        if any(n in LIGHT_MATERIALS for n in names):
            if EMISSION_PROP not in ob.keys():
                ob[EMISSION_PROP] = 1.0
                ob.id_properties_ui(EMISSION_PROP).update(min=0.0, max=1.0,
                                                          description="Işık şiddeti çarpanı (rig sürer)")
            if "UM_StatusLED" in names:
                ob[STATUS_GATE_PROP] = gate
                ob.id_properties_ui(STATUS_GATE_PROP).update(
                    min=0.0, max=1.0, description="Taret durum LED'i kapısı: 0 = sönük (nav_lights'a bağlı değil)")
            out.append(ob.name)
    return out


def tag_stencils(livery: str = "standart") -> list[str]:
    """Gövde modülünün ``U_Stencil_*`` şablonlarından ``UM_Accent`` kullananlara yazı kontrastı (``ucav_decal_mix``)
    yazar: standartta düşük kontrastlı gri, taktikte zeminden açık gri."""
    key = _livery_key(livery)
    out = []
    for ob in bpy.data.objects:
        if ob.type == "MESH" and ob.name.startswith("U_Stencil_") and ob.data is not None and \
                any(m is not None and m.name == "UM_Accent" for m in ob.data.materials):
            ob[DECAL_MIX_PROP] = float(DECAL_MIX[key])
            out.append(ob.name)
    return out


def tag_panels() -> list[str]:
    """Panel çizgisi ailesini (``ucav_panel``) statik deri nesnelerine yazar (bkz. ``PANEL_OBJECTS``)."""
    out = []
    for name, k in PANEL_OBJECTS.items():
        ob = bpy.data.objects.get(name)
        if ob is not None:
            ob[PANEL_PROP] = float(k)
            out.append(name)
    return out


def build(livery: str = "standart", *, decals: bool = True, scene: bpy.types.Scene | None = None) -> dict:
    """Bütün ``UM_*`` malzemelerini ``livery`` boya şemasıyla kurar; ``decals`` True ise işaretleri ekler.
    Tekrar çağrılabilir (ör. ``build("taktik")`` yalnız renkleri ve işaret kontrastını değiştirir).
    Döndürür: {"livery", "materials", "lights", "decals", "missing", "panels", "stencils"} — ``missing``:
    sahnede malzemesi olmayan ya da ``UM_*`` dışı yuva taşıyan ağ nesneleri (boş olmalı)."""
    scene = scene or bpy.context.scene
    key = _livery_key(livery)
    cols = livery_colors(key)
    target = decal_target_color(cols, key)
    panel_group(force=True)                          # bir kez; malzemeler aynı grubu paylaşır
    built = [build_material(n, cols[n], target, key).name for n in P.MATERIALS]
    lights = ensure_light_props()
    panels = tag_panels()
    stencils = tag_stencils(key)
    dec = build_decals(key, scene) if decals else []
    scene["ucav_livery"] = key
    missing = []
    for ob in bpy.data.objects:
        if ob.type == "MESH" and ob.name.startswith("U_"):
            ms = list(ob.data.materials)
            if not ms or any(m is None or m.name not in P.MATERIALS for m in ms):
                missing.append(ob.name)
    return {"livery": key, "materials": built, "lights": lights, "decals": dec, "missing": missing,
            "panels": panels, "stencils": stencils}


# =====================================================================================================
# İşaretler (metin ağı + BVH izdüşümü)
# =====================================================================================================
@dataclass
class DecalSpec:
    """Bir işaretin yerleşimi (Blender dünya koordinatı)."""
    name: str
    host: str
    text: str | None            # None → şerit (bant) ya da 2B şekil (``shape``)
    material: str
    height: float               # yazı büyük harf yüksekliği / şekil ölçeği (m)
    center: tuple               # yaklaşık yüzey noktası
    u: tuple                    # okuma yönü (taban çizgisi)
    n: tuple                    # yüzey dışa normali (izdüşüm yönü −n)
    strip: np.ndarray | None = None     # bant için (k, m, 3) ızgara noktaları (satır: yükseklik, sütun: genişlik)
    shape: tuple | None = None          # 2B şekil (V (birim: height), F, yüz başına malzeme indisi)
    materials: tuple = ()               # ek malzeme yuvaları (``shape`` yüz indisleri 1, 2 … bunlara)
    mix: float | None = None            # ``ucav_decal_mix`` (None → yazı: şema değeri, şerit/şekil: 0)


def _disc_mesh(quartered: bool = True, ring: float = 0.10, n: int = 48) -> tuple[np.ndarray, list, list]:
    """Birim çaplı (yarıçap 0,5) disk: dış halka (kalınlık ``ring``·çap) + iç çeyrekler. ``quartered``: karşılıklı
    iki çeyrek koyu (indis 0), diğer ikisi açık (indis 1) — ağırlık merkezi işareti; değilse yalnız halka."""
    r_out, r_in = 0.5, 0.5 - ring
    th = np.linspace(0.0, 2 * np.pi, n, endpoint=False)
    V = [(0.0, 0.0)] + [(r_in * math.cos(t), r_in * math.sin(t)) for t in th] + \
        [(r_out * math.cos(t), r_out * math.sin(t)) for t in th]
    F, M = [], []
    for i in range(n):
        j = (i + 1) % n
        F.append([1 + i, 1 + n + i, 1 + n + j, 1 + j])                    # halka
        M.append(0)
        if quartered:
            F.append([0, 1 + i, 1 + j])
            M.append(0 if (i * 4 // n) % 2 == 0 else 1)
    if not quartered:                                                   # yalnız halka: merkez noktası kullanılmaz
        V = V[1:]
        F = [[k - 1 for k in f] for f in F]
    return np.asarray(V, float), F, M


def _font():
    for p in FONT_CANDIDATES:
        if os.path.isfile(p):
            f = bpy.data.fonts.get(os.path.basename(p))
            if f is None:
                try:
                    f = bpy.data.fonts.load(p, check_existing=True)
                except RuntimeError:
                    continue
            return f
    return bpy.data.fonts.load("<builtin>", check_existing=True)


def text_outline_mesh(text: str, max_edge: float = 0.03) -> tuple[np.ndarray, list[list[int]]]:
    """Metni 2B üçgen ağa çevirir (birim: büyük harf yüksekliği = 1, merkez 0). ``max_edge``: aynı birimde
    üçgen kenarı üst sınırı (izdüşümde eğriliğe uyum için)."""
    cu = bpy.data.curves.new("UCAV_tmp_text", "FONT")
    cu.body = text
    cu.font = _font()
    cu.size = 1.0
    cu.resolution_u = 4
    cu.fill_mode = "BOTH"
    cu.align_x = "CENTER"
    cu.align_y = "CENTER"
    cu.space_character = 1.04
    tmp = bpy.data.objects.new("UCAV_tmp_text", cu)
    bpy.context.scene.collection.objects.link(tmp)
    try:
        dg = bpy.context.evaluated_depsgraph_get()
        dg.update()
        me = bpy.data.meshes.new_from_object(tmp.evaluated_get(dg), depsgraph=dg)
    finally:
        bpy.data.objects.remove(tmp, do_unlink=True)
        bpy.data.curves.remove(cu)
    bm = bmesh.new()
    bm.from_mesh(me)
    bpy.data.meshes.remove(me)
    bmesh.ops.remove_doubles(bm, verts=bm.verts, dist=1e-6)
    co = np.array([v.co[:2] for v in bm.verts])
    lo, hi = co.min(0), co.max(0)
    h = hi[1] - lo[1]
    c = 0.5 * (lo + hi)
    for v in bm.verts:
        v.co.x = (v.co.x - c[0]) / h * TEXT_CONDENSE
        v.co.y = (v.co.y - c[1]) / h
        v.co.z = 0.0
    bmesh.ops.triangulate(bm, faces=bm.faces)
    for _ in range(8):
        long_edges = [e for e in bm.edges if e.calc_length() > max_edge]
        if not long_edges:
            break
        bmesh.ops.subdivide_edges(bm, edges=long_edges, cuts=1, use_grid_fill=False)
        bmesh.ops.triangulate(bm, faces=bm.faces, quad_method="BEAUTY", ngon_method="BEAUTY")
    V = np.array([v.co[:2] for v in bm.verts])
    bm.verts.index_update()
    F = [[v.index for v in f.verts] for f in bm.faces]
    bm.free()
    return V, F


def _host_bvh(ob: bpy.types.Object) -> BVHTree:
    """Ev sahibi nesnenin değerlendirilmiş ağından dünya koordinatlı BVH."""
    dg = bpy.context.evaluated_depsgraph_get()
    ev = ob.evaluated_get(dg)
    me = ev.to_mesh()
    try:
        me.calc_loop_triangles()
        M = ob.matrix_world
        verts = [M @ v.co for v in me.vertices]
        tris = [tuple(t.vertices) for t in me.loop_triangles]
    finally:
        ev.to_mesh_clear()
    return BVHTree.FromPolygons(verts, tris)


def _project(bvh: BVHTree, pts: np.ndarray, n: Vector, lift: float = 0.02,
             offset: float = DECAL_OFFSET) -> tuple[np.ndarray, np.ndarray, int]:
    """Noktaları −n yönünde yüzeye izdüşürür; yüzey normali boyunca ``offset`` kadar kaldırır.
    Döndürür: (noktalar, nokta başına yüzey normali, ıskalanan sayısı)."""
    out = np.array(pts, float)
    nrms = np.tile(np.asarray(n, float), (len(out), 1))
    miss = 0
    for i, p in enumerate(pts):
        start = Vector(p) + n * lift
        loc, nrm, _, _ = bvh.ray_cast(start, -n, 3 * lift)
        if loc is None:
            miss += 1
            continue
        if nrm.dot(n) < 0:
            nrm = -nrm
        out[i] = np.asarray(loc + nrm * offset)
        nrms[i] = np.asarray(nrm)
    return out, nrms, miss


def _solidify(V: np.ndarray, N: np.ndarray, F: list, n_ref, t: float = DECAL_THICK) -> tuple[np.ndarray, list]:
    """Açık yüzey işaretini ``t`` kalınlığında kapalı katıya çevirir (alt yüz −N yönünde; yan duvarlar
    sınır kenarlarından). Üst yüz normali ``n_ref`` yönüne çevrilir → dışa bakan, manifold kabuk."""
    V = np.asarray(V, float)
    F = [list(f) for f in F]
    if F:                                                       # yön: üst yüz normali n_ref ile aynı
        a, b, c = (V[i] for i in F[0][:3])
        if np.dot(np.cross(b - a, c - a), np.asarray(n_ref, float)) < 0:
            F = [f[::-1] for f in F]
    nv = len(V)
    Vb = V - np.asarray(N, float) * t
    out_f = [tuple(f) for f in F] + [tuple(nv + i for i in f[::-1]) for f in F]
    count: dict[tuple[int, int], int] = {}
    directed = []
    for f in F:
        for k in range(len(f)):
            a, b = f[k], f[(k + 1) % len(f)]
            key = (min(a, b), max(a, b))
            count[key] = count.get(key, 0) + 1
            directed.append((a, b))
    for a, b in directed:
        if count[(min(a, b), max(a, b))] == 1:
            out_f.append((a, nv + a, nv + b, b))
    return np.vstack([V, Vb]), out_f


def _surface_point(bvh: BVHTree, p, n) -> tuple[Vector, Vector] | None:
    n = Vector(n).normalized()
    loc, nrm, _, _ = bvh.ray_cast(Vector(p) + n * 0.08, -n, 0.3)
    if loc is None:
        return None
    if nrm.dot(n) < 0:
        nrm = -nrm
    return loc, nrm


def decal_specs() -> list[DecalSpec]:
    """İşaretlerin yerleşimi (params geometrisinden); metinler ve ölçüler ``spec.yaml → details.markings.decals``."""
    dk = (P.SPEC.get("details", {}).get("markings", {}) or {}).get("decals", {}) or {}
    fin_text, fin_h = str(dk.get("fin_text", "YK-38")), float(dk.get("fin_text_h_m", 0.026))
    name_text, name_h = str(dk.get("name_text", "YELKOVAN")), float(dk.get("name_text_h_m", 0.022))
    out: list[DecalSpec] = []
    tr = P.hinge_line("Rudder", "L").chord_fraction                         # dümen veter oranı (0,35)
    for side in ("L", "R"):
        sgn = 1.0 if side == "L" else -1.0
        aft_reading = (-1.0, 0.0, 0.0) if side == "L" else (1.0, 0.0, 0.0)     # soldan sağa okunur
        # --- dikey dış yüzü: "YK-38"
        h = 0.20
        fs = P.fin_station(h, side)
        nrm = np.asarray(P.vec_to_blender(fs.normal), float)
        if nrm[1] * sgn < 0:
            nrm = -nrm
        frac = 0.5 * (1.0 - tr)                                                # sabit kısmın ortası
        c_spec = np.asarray(fs.le, float) + np.array([frac * fs.chord, 0.0, 0.0])
        out.append(DecalSpec(f"U_Decal_YK38_{side}", f"U_Fin_{side}", fin_text, "UM_Accent", fin_h,
                             tuple(P.to_blender(*c_spec)), aft_reading, tuple(nrm)))
        # --- kuyruk konisi yanı: "YELKOVAN" (chine çizgisine paralel)
        s0, s1 = 1.56, 1.80
        z0 = P.fuselage_section(s0).z_chine + 0.033
        z1 = P.fuselage_section(s1).z_chine + 0.033
        sc, zc = 0.5 * (s0 + s1), 0.5 * (z0 + z1)
        u = np.array([-(s1 - s0), 0.0, z1 - z0]) * (1.0 if side == "L" else -1.0)
        out.append(DecalSpec(f"U_Decal_Name_{side}", "U_Fuselage", name_text, "UM_Accent", name_h,
                             tuple(P.to_blender(sc, sgn * 0.2, zc)), tuple(u / np.linalg.norm(u)),
                             (0.0, sgn, 0.0)))
        # --- dikey iç yüzü: pervane düzlemi uyarı bandı
        w = float(dk.get("prop_band_w_m", 0.012))
        inner = -nrm
        rows = []
        for hh in np.linspace(0.006, 0.20, 60):
            f2 = P.fin_station(float(hh), side)
            z = float(f2.le[2])
            s_c = float(P.PROP.plane_s_at_z(z))
            if f2.le[0] > s_c - 0.5 * w - 0.007:                               # hücum kenarına 7 mm kala dur
                break
            yz = np.asarray(f2.le, float)
            rows.append([P.to_blender(sv, yz[1], yz[2]) for sv in np.linspace(s_c - 0.5 * w, s_c + 0.5 * w, 7)])
        if len(rows) >= 2:
            strip = np.asarray(rows, float)
            out.append(DecalSpec(f"U_Decal_PropBand_{side}", f"U_Fin_{side}", None, "UM_Orange", w,
                                 tuple(strip.mean((0, 1))), (0.0, 0.0, 1.0), tuple(inner), strip))
    return out + stencil_specs()


# Ağırlık merkezi işareti (varsayım: çap, CG kotunun üstüne kaydırma). Yazılı şablonlar (ADIM ATMA, YAKIT, PERVANE,
# STATİK PORT …) gövde modülünün ``U_Stencil_*`` nesneleridir (spec ``details.markings.stencils``); burada yinelenmez.
CG_MARK_D = 0.018
CG_MARK_DZ = 0.020        # disk CG kotunun 20 mm üstünde: alt kenarı kök filetosunun (s 1,232'de z ≈ 0,002) üstünde kalır


def stencil_specs() -> list[DecalSpec]:
    """Ağırlık merkezi işareti: gövdenin iki yanında ``params.CG.s`` istasyonunda (kot CG + ``CG_MARK_DZ``, kanat kök
    filetosunun üstü) Ø18 mm siyah/beyaz çeyrekli disk (koyu çeyrekler + dış halka ``UM_Accent``, açık çeyrekler
    ``UM_SkinBottom``). İşaret boyuna CG istasyonunu gösterir."""
    out: list[DecalSpec] = []
    cg_s, cg_z = float(P.CG.s), float(P.CG.z)
    disc = _disc_mesh(quartered=True, ring=0.09)
    for side in ("L", "R"):
        sgn = 1.0 if side == "L" else -1.0
        aft_reading = (-1.0, 0.0, 0.0) if side == "L" else (1.0, 0.0, 0.0)
        out.append(DecalSpec(f"U_Decal_CG_{side}", "U_Fuselage", None, "UM_Accent", CG_MARK_D,
                             P.to_blender(cg_s, sgn * 0.2, cg_z + CG_MARK_DZ), aft_reading, (0.0, sgn, 0.0), shape=disc,
                             materials=("UM_SkinBottom",), mix=0.0))
    return out


def _decal_mesh(ds: DecalSpec, bvh: BVHTree) -> tuple[np.ndarray, list, int, list | None]:
    """İşaret ağı → (köşeler, yüzler, ıskalanan nokta, yüz başına malzeme indisi ya da None)."""
    n = Vector(ds.n).normalized()
    if ds.strip is not None:
        k, m = ds.strip.shape[:2]
        pts = ds.strip.reshape(-1, 3)
        V, N, miss = _project(bvh, pts, n, lift=0.03)               # ızgarayı yüzeye −n yönünde izdüşür
        F = [[i * m + j, i * m + j + 1, (i + 1) * m + j + 1, (i + 1) * m + j]
             for i in range(k - 1) for j in range(m - 1)]
        V, F = _solidify(V, N, F, n)
        return V, F, miss, None
    hit = _surface_point(bvh, ds.center, n)
    if hit is None:
        raise RuntimeError(f"{ds.name}: yüzey bulunamadı ({ds.host})")
    c, n_s = hit
    n = n_s.normalized()
    u = Vector(ds.u)
    u = (u - n * u.dot(n)).normalized()
    v = n.cross(u)
    fm = None
    if ds.shape is not None:
        V2, F, fm = ds.shape
        V2 = np.asarray(V2, float)
    else:
        V2, F = text_outline_mesh(ds.text, DECAL_MAX_EDGE / ds.height)
    pts = np.array([c + u * (x * ds.height) + v * (y * ds.height) for x, y in V2])
    V, N, miss = _project(bvh, pts, n)
    nf = len(F)
    V, F = _solidify(V, N, F, n)
    if fm is not None:
        fm = list(fm) + list(fm) + [0] * (len(F) - 2 * nf)        # üst, alt, yan duvarlar
    return V, F, miss, fm


def build_decals(livery: str = "standart", scene: bpy.types.Scene | None = None) -> list[str]:
    """İşaret nesnelerini kurar (eskileri silinir). Ev sahibi yoksa o işaret atlanır."""
    scene = scene or bpy.context.scene
    key = _livery_key(livery)
    cols = U.ensure_collections(scene)
    col = cols["UCAV_Details"]
    for ob in [o for o in bpy.data.objects if o.name.startswith("U_Decal_")]:
        U.remove_object(ob.name)
    made = []
    bvhs: dict[str, BVHTree] = {}
    for ds in decal_specs():
        host = bpy.data.objects.get(ds.host)
        if host is None:
            continue
        if ds.host not in bvhs:
            bvhs[ds.host] = _host_bvh(host)
        V, F, miss, fm = _decal_mesh(ds, bvhs[ds.host])
        if miss > 0.02 * len(V):
            print(f"[materials] uyarı: {ds.name} için {miss}/{len(V)} nokta yüzeye düşmedi")
        me = bpy.data.meshes.new(ds.name)
        me.from_pydata([tuple(p) for p in V], [], F)
        for mname in (ds.material, *ds.materials):
            me.materials.append(U.get_material(mname))
        if fm is not None:
            me.polygons.foreach_set("material_index", [int(i) for i in fm])
        me.validate()
        me.shade_smooth()
        ob = bpy.data.objects.new(ds.name, me)
        col.objects.link(ob)
        U.parent_keep_world(ob, host)                  # köşeler dünyada; nesne dönüşümü kimlik
        mix = ds.mix if ds.mix is not None else (float(DECAL_MIX[key]) if ds.text else 0.0)
        ob[DECAL_MIX_PROP] = float(mix)
        ob["ucav_decal"] = ds.text or ("shape" if ds.shape is not None else "prop_band")
        ob["ucav_closed"] = True
        ob.visible_shadow = False
        made.append(ob.name)
    return made
