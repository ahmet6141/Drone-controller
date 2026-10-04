"""YELKOVAN YK-38 — malzemeler, boya şemaları ve işaretler (bpy 4.5): ``build(livery="standart")``.

``params.MATERIALS`` içindeki her ``UM_*`` adı için Cycles PBR düğüm ağacı kurulur. Gövde/takım modülleri
yüzlere malzeme yuvalarını önceden atar (yoksa ``util.get_material`` basit Principled yer tutucu oluşturur);
bu modül **aynı adlı veri bloğunu** bulur ve düğüm ağacını yeniden yazar — yuvalar değişmez, eksik (magenta)
malzeme kalmaz. Bilinmeyen yeni bir ``UM_*`` adı spec'e eklenirse genel Principled tarifiyle kurulur.

Malzeme tarifleri (renkler ``spec.yaml → materials``; aşağıdaki ince ayarlar "varsayım"dır)
------------------------------------------------------------------------------------------
* **Boya** (``UM_SkinTop`` RAL 7035, ``UM_SkinBottom`` RAL 9003, ``UM_Accent`` RAL 7016, ``UM_Turquoise``
  RAL 5018, ``UM_Antenna``): saten boya — nesne koordinatında çok hafif ton (±%2,5) ve pürüzlülük (±0,04)
  dalgalanması, ince vernik katmanı (coat 0,25), portakal kabuğu mikro tümseği (yalnız yakın yansımada görünür).
  Doku nesne koordinatındadır: kumanda yüzeyi döndüğünde ya da uçak uçtuğunda boya "kaymaz".
* ``UM_Orange`` RAL 2005 (parlak/fosforlu turuncu): boya + çok zayıf ışıma (gün ışığı floresansı taklidi) —
  kuyu ve kapak içleri gölgede de turuncu okunur.
* ``UM_SmokeHatch``: füme parlak PETG — iki arayüzlü geçirgenlik (toplam ≈ %10), cam gibi yansıma; kapağın
  altındaki GNSS cebi ve antenler loş görünür.
* ``UM_PACF``: boyasız PA-CF — mat, ince baskı dokusu (tümsek), hafif "sheen".
* ``UM_Nozzle``: antrasit metal lüle halkası; iç kenardan dışa ısı tonu (mavi → bronz, spec ``heat_tint``),
  pervane ekseninden radyal uzaklıkla ve gürültüyle karışır.
* ``UM_Prop``: vernikli kayın — pala boyunca uzanan düz damar (dalga + gürültü), kalın vernik katmanı.
* ``UM_Spinner``: parlatılmış alüminyum, torna izi — eksen etrafında radyal teğetli anizotropi.
* ``UM_Gear`` (anodize Al), ``UM_Hub``, ``UM_Steel`` (oleo krom / pitot — pürüzlülük ≤ 0,14), ``UM_Engine``.
* ``UM_Tire``: kauçuk — mat, tozlu "sheen", ince tümsek. ``UM_TPU``: mat siyah TPU.
* ``UM_TurretBody``: mat siyah "soft-touch" boya. ``UM_SensorGlass``: pencere nesnesinin yerel +Y yarısı EO
  (koyu lacivert cam, MgF₂ yansıma önleyici ince film → mavi-mor yansıma), −Y yarısı IR (germanyum: gri, yarı metalik,
  DLC ince film). ``UM_Carbon``: 2×2 dimi örgü karbon + vernik.
* **Işıklar** (``UM_NavRed``, ``UM_NavGreen``, ``UM_Strobe``, ``UM_StatusLED``): parlak lens + ışıma.
  Işıma şiddeti = spec ``emission`` × nesne özelliği ``ucav_emission`` (Attribute düğümü, nesne türü). ``rig``
  her ışık nesnesinin ``ucav_emission`` değerini sürer (seyrüsefer: ``nav_lights``, çakarlar: ``strobe``);
  böylece aynı malzemeyi paylaşan iniş ışığı çakmaz, seyrüsefer gibi davranır. Özellik yoksa ``build``
  1,0 yazar (ışıklar açık). Not: Emission Strength soketi bağlı olduğundan ``rig.drive_light_materials``
  malzeme düzeyinde sürücü kurmaz — tasarım gereği.

Boya şemaları (``LIVERIES``)
----------------------------
* ``"standart"`` (``"standard"`` da kabul edilir): spec renkleri — üst RAL 7035, alt RAL 9003.
* ``"taktik"``: yalnız render için düşük görünürlüklü koyu gri (spec ``materials.liveries.taktik``: üst
  RAL 7046, alt RAL 7035); turkuaz çizgi ve lens halkası koyu gri-petrole çekilir (varsayım). Uyarı: koyu boya
  güneşte LW-PLA'yı ısıtır (Tg ≈ 55 °C) — uçacak gövde için önerilmez.

İşaretler (``build_decals``)
----------------------------
Yazılar Blender metin eğrisinden ağa çevrilir (Liberation Sans Bold, %86 dar; yoksa DejaVu/FreeSans/yerleşik
yazı tipi), ≤ 2,5 mm kenarlı üçgenlere bölünür ve BVH ışın izdüşümüyle ev sahibi yüzeye oturtulur (eğriliğe
uyar). Her işaret 0,15 mm kalınlığında kapalı ince katıdır (üst yüzü deriden 0,35 mm yukarıda; manifold,
gölge düşürmez). Malzeme ``UM_Accent`` (antrasit "şablon" yazı; nesne özelliği ``ucav_decal_mix`` ile üst
yüzey rengine doğru açılır → spec'teki "düşük kontrastlı gri"; taktik şemada saf antrasit).

* ``U_Decal_YK38_L/R``: dikeylerin dış yüzünde "YK-38" (yükseklik 26 mm), sabit dikey kısmında.
* ``U_Decal_Name_L/R``: kuyruk konisi yanında, chine çizgisine paralel "YELKOVAN" (22 mm).
* ``U_Decal_PropBand_L/R``: dikeylerin iç yüzünde pervane düzlemi uyarı bandı (RAL 2005, 12 mm), bant
  ``PROP.plane_s_at_z`` ile 5° aşağı itki eğimini izler.

Hepsi ``UCAV_Details`` koleksiyonunda, ev sahibine dünya dönüşümü korunarak bağlıdır (``build`` tekrar
çağrılabilir; eski işaretler silinip yeniden kurulur). Baskı modülü bu nesneleri kullanmaz.

Sıra: ``airframe.build()`` → ``gear.build()`` → ``materials.build()`` → ``rig.setup()`` (sıra değişse de çalışır).
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
DECAL_MIX_PROP = "ucav_decal_mix"         # yazı rengini yüzey rengine açma oranı (0 = saf antrasit)

# İnce ayarlar (varsayım) — boya katmanı
PAINT_COAT = 0.25                         # vernik ağırlığı (saten)
PAINT_COAT_ROUGH = 0.22
PAINT_TONE_VAR = 0.025                    # ± ton dalgalanması
PAINT_ROUGH_VAR = 0.04                    # ± pürüzlülük dalgalanması
ORANGE_GLOW = 0.10                        # RAL 2005 floresans taklidi (ışıma şiddeti)
SMOKE_TOTAL_T = 0.10                      # füme kapak toplam geçirgenliği (iki arayüz)

# Taktik boya şemasında spec dışında kalan renkler (varsayım: düşük görünürlük)
TAKTIK_EXTRA = {"UM_Turquoise": "#3F5A5E", "UM_StatusLED": "#2E8C93"}
DECAL_MIX = {"standart": 0.42, "taktik": 0.0}      # yazı kontrastı (0 = antrasit)
EO_BLUE = (0.004, 0.010, 0.040)          # EO penceresi koyu lacivert ton (doğrusal)
EO_AR_NM = 135.0                         # MgF₂ yansıma önleyici ince film (≈ ¼λ) → mavi-mor yansıma

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
# Malzeme tarifleri
# =====================================================================================================
def _paint(T: _Tree, color, rough: float, *, coat: float = PAINT_COAT, coat_rough: float = PAINT_COAT_ROUGH,
           glow: float = 0.0, decal_mix=None, metallic: float = 0.0):
    """Saten boya: ton + pürüzlülük dalgalanması, ince vernik, portakal kabuğu tümseği."""
    tc = T.coords()
    obj = tc.outputs["Object"]
    n_tone = T.noise(obj, 2.2, 3.0, 0.55, y=300)
    base = T.mix_rgb(n_tone.outputs["Fac"], _scale(color, 1.0 - PAINT_TONE_VAR), _scale(color, 1.0 + PAINT_TONE_VAR),
                     y=300)
    if decal_mix is not None:                       # işaret nesneleri: yüzey rengine doğru aç (düşük kontrast)
        at = T.node("ShaderNodeAttribute", -800, 600, attribute_type="OBJECT", attribute_name=DECAL_MIX_PROP)
        base = T.mix_rgb(at.outputs["Fac"], base, decal_mix, x=-300, y=400)
    n_r = T.noise(obj, 7.0, 2.0, 0.5, y=0)
    rgh = T.map_range(n_r.outputs["Fac"], rough - PAINT_ROUGH_VAR, rough + PAINT_ROUGH_VAR, 0.3, 0.7, y=0)
    n_peel = T.noise(obj, 420.0, 1.0, 0.4, y=-500)
    nrm = T.bump(n_peel.outputs["Fac"], 0.035, 0.0002)
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
    tc = T.coords()
    obj = tc.outputs["Object"]
    n1 = T.noise(obj, 900.0, 2.0, 0.6, y=-400)
    nrm = T.bump(n1.outputs["Fac"], 0.12, 0.0003)
    n2 = T.noise(obj, 9.0, 2.0, 0.5, y=0)
    rgh = T.map_range(n2.outputs["Fac"], spec.roughness - 0.06, spec.roughness + 0.04, 0.3, 0.7)
    bsdf = T.principled(Base_Color=color, Roughness=rgh, Metallic=0.0, Sheen_Weight=0.25, Sheen_Roughness=0.4,
                        Sheen_Tint=(0.6, 0.6, 0.62), Normal=nrm)
    T.output(bsdf.outputs[0])


def _nozzle(T: _Tree, spec, color):
    """Isı tonu: pervane ekseninden radyal uzaklık (iç kenar sıcak) + gürültü → antrasit / bronz / mavi."""
    ring = P.SPEC["propulsion"]["exhaust_ring"]
    r_in, r_out = 0.5 * float(ring["id_m"]), 0.5 * float(ring["od_m"])
    hub = Vector(P.PROP.hub_b)
    ax = Vector(P.PROP.axis_aft_b).normalized()
    tc = T.coords()
    obj = tc.outputs["Object"]                                      # lüle nesnelerinin orijini dünya (0,0,0)
    d = T.node("ShaderNodeVectorMath", -1150, 200, operation="SUBTRACT")
    T.link(obj, d.inputs[0])
    d.inputs[1].default_value = hub
    cr = T.node("ShaderNodeVectorMath", -950, 200, operation="CROSS_PRODUCT")
    T.link(d.outputs[0], cr.inputs[0])
    cr.inputs[1].default_value = ax
    ln = T.node("ShaderNodeVectorMath", -750, 200, operation="LENGTH")
    T.link(cr.outputs[0], ln.inputs[0])
    radial = T.map_range(ln.outputs["Value"], 1.0, 0.0, r_in, r_out + 0.25 * (r_out - r_in), y=200)
    nz = T.noise(obj, 160.0, 3.0, 0.6, y=-100)
    f = T.math("MULTIPLY", radial, T.map_range(nz.outputs["Fac"], 0.55, 1.15, 0.3, 0.7, y=-100), x=-450, y=100,
               clamp=True)
    ramp = T.node("ShaderNodeValToRGB", -250, 100)
    tints = [hex_to_linear(h) for h in spec.extra.get("heat_tint", ["#6A4A2E", "#3B4F7A"])]
    els = ramp.color_ramp.elements
    els[0].position, els[0].color = 0.0, _rgba(color)
    els[1].position, els[1].color = 0.45, _rgba(tints[0])
    e = els.new(0.85)
    e.color = _rgba(tints[1] if len(tints) > 1 else tints[0])
    T.link(f, ramp.inputs["Fac"])
    bsdf = T.principled(Base_Color=ramp.outputs["Color"], Metallic=spec.metallic, Roughness=spec.roughness)
    T.output(bsdf.outputs[0])


def _prop_wood(T: _Tree, spec, color):
    """Vernikli kayın: pala (yerel Z) boyunca düz damar; damar şeritleri veter (Y) yönünde değişir."""
    tc = T.coords()
    obj = tc.outputs["Object"]
    mp = T.node("ShaderNodeMapping", -1200, 0, ins={"Scale": (1.0, 1.0, 0.08)})
    T.link(obj, mp.inputs["Vector"])
    wave = T.node("ShaderNodeTexWave", -950, 0, wave_type="BANDS", bands_direction="Y", wave_profile="SIN",
                  ins={"Scale": 160.0, "Distortion": 6.0, "Detail": 3.0, "Detail Scale": 1.5})
    T.link(mp.outputs["Vector"], wave.inputs["Vector"])
    nz = T.noise(obj, 60.0, 4.0, 0.6, y=-300)
    f = T.math("MULTIPLY", wave.outputs["Fac"], T.map_range(nz.outputs["Fac"], 0.6, 1.2, 0.3, 0.7, y=-300),
               x=-700, y=-100, clamp=True)
    light = _scale(color, 1.08)
    dark = tuple(np.asarray(color) * np.array([0.72, 0.62, 0.55]))
    base = T.mix_rgb(f, light, dark, x=-450, y=0)
    nrm = T.bump(wave.outputs["Fac"], 0.04, 0.0001)
    bsdf = T.principled(Base_Color=base, Roughness=0.42, Coat_Weight=1.0, Coat_Roughness=max(0.04, spec.roughness / 4),
                        Coat_IOR=1.52, Coat_Tint=(1.0, 0.93, 0.80), Normal=nrm)
    T.output(bsdf.outputs[0])


def _spun_metal(T: _Tree, spec, color, axis: str = "X"):
    """Parlak alüminyum, torna izi: yerel eksen etrafında radyal teğetli anizotropi."""
    tg = T.node("ShaderNodeTangent", -400, -300, direction_type="RADIAL", axis=axis)
    bsdf = T.principled(Base_Color=color, Metallic=spec.metallic, Roughness=spec.roughness + 0.04, Anisotropic=0.75,
                        Tangent=tg.outputs["Tangent"])
    T.output(bsdf.outputs[0])


def _metal(T: _Tree, spec, color, rough: float | None = None, var: float = 0.03, aniso: float = 0.0):
    tc = T.coords()
    n = T.noise(tc.outputs["Object"], 40.0, 3.0, 0.55)
    r = spec.roughness if rough is None else rough
    rgh = T.map_range(n.outputs["Fac"], max(0.02, r - var), r + var, 0.3, 0.7)
    ins = dict(Base_Color=color, Metallic=spec.metallic, Roughness=rgh)
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


def _soft_black(T: _Tree, spec, color):
    bsdf = T.principled(Base_Color=color, Roughness=spec.roughness, Coat_Weight=0.12, Coat_Roughness=0.35,
                        Sheen_Weight=0.15, Sheen_Roughness=0.5)
    T.output(bsdf.outputs[0])


def _sensor_glass(T: _Tree, spec, color):
    """EO (yerel +Y, iskele) ve IR (yerel −Y, sancak) pencereleri aynı malzemede; ayrım nesne koordinatıyla."""
    tc = T.coords()
    sep = T.node("ShaderNodeSeparateXYZ", -1100, 0)
    T.link(tc.outputs["Object"], sep.inputs[0])
    is_eo = T.math("GREATER_THAN", sep.outputs["Y"], 0.0, x=-900, y=0)
    ir = T.principled(-300, -350, Base_Color=(0.20, 0.205, 0.215), Metallic=0.8, Roughness=0.035,
                      Thin_Film_Thickness=270.0, Thin_Film_IOR=2.0)
    eo_tint = tuple(0.5 * float(c) + 0.5 * b for c, b in zip(color, EO_BLUE))         # koyu lacivert cam
    eo = T.principled(-300, 350, Base_Color=eo_tint, Roughness=max(0.015, spec.roughness), IOR=spec.ior,
                      Specular_IOR_Level=0.6, Thin_Film_Thickness=EO_AR_NM, Thin_Film_IOR=1.38)
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
    """Işık lensi: parlak kubbe + ışıma (spec şiddeti × nesne ``ucav_emission``)."""
    at = T.node("ShaderNodeAttribute", -700, -300, attribute_type="OBJECT", attribute_name=EMISSION_PROP)
    strength = T.math("MULTIPLY", at.outputs["Fac"], float(spec.emission), x=-450, y=-300)
    lens = tuple(0.35 + 0.45 * float(c) for c in color) if spec.name == "UM_Strobe" else _scale(color, 0.55)
    bsdf = T.principled(Base_Color=lens, Roughness=0.12, Coat_Weight=1.0, Coat_Roughness=0.03,
                        Emission_Color=color, Emission_Strength=strength)
    T.output(bsdf.outputs[0])


def _recipe(name: str):
    """Ad → tarif. Bilinmeyen ad → genel Principled."""
    return {
        "UM_SkinTop": "paint", "UM_SkinBottom": "paint", "UM_Accent": "paint_decal", "UM_Turquoise": "gloss",
        "UM_Antenna": "paint", "UM_Orange": "orange", "UM_SmokeHatch": "smoke", "UM_PACF": "pacf",
        "UM_Nozzle": "nozzle", "UM_Prop": "wood", "UM_Spinner": "spun", "UM_Gear": "anod", "UM_Hub": "metal",
        "UM_Steel": "chrome", "UM_Engine": "metal", "UM_Tire": "rubber", "UM_TPU": "rubber",
        "UM_TurretBody": "soft_black", "UM_SensorGlass": "glass", "UM_Carbon": "carbon",
        "UM_NavRed": "light", "UM_NavGreen": "light", "UM_Strobe": "light", "UM_StatusLED": "light",
    }.get(name, "generic")


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


def livery_colors(livery: str = "standart") -> dict[str, tuple[float, float, float]]:
    """Boya şemasına göre her ``UM_*`` için doğrusal RGB."""
    key = _livery_key(livery)
    out = {n: tuple(s.rgb_linear) for n, s in P.MATERIALS.items()}
    if key == "taktik":
        tk = P.LIVERIES.get("taktik", {}) or {}
        for n, hx in {**TAKTIK_EXTRA, **{k: v for k, v in tk.items() if k.startswith("UM_")}}.items():
            if n in out:
                out[n] = hex_to_linear(hx)
    return out


LIVERIES = ("standart", "taktik")


# =====================================================================================================
# Kurulum
# =====================================================================================================
def decal_target_color(colors: dict) -> tuple[float, float, float]:
    """Düşük kontrastlı yazı rengi hedefi: antrasitten üst yüzey rengine doğru %62 (orta gri)."""
    top = np.asarray(colors["UM_SkinTop"], float)
    acc = np.asarray(colors["UM_Accent"], float)
    return tuple(acc + 0.62 * (top - acc))


def build_material(name: str, color=None, decal_target=None) -> bpy.types.Material:
    """Tek bir ``UM_*`` malzemesini (veri bloğu varsa onu) kurar ve döndürür. ``decal_target``: yalnız
    ``UM_Accent`` için işaret nesnelerinin açılacağı renk (None → standart boya şemasından)."""
    spec = P.MATERIALS[name]
    color = tuple(color if color is not None else spec.rgb_linear)
    mat = bpy.data.materials.get(name) or bpy.data.materials.new(name)
    mat.use_nodes = True
    if mat.node_tree.animation_data is not None:      # eski düğümlere bağlı sürücüler (yer tutucudan kalma)
        mat.node_tree.animation_data_clear()
    T = _Tree(mat.node_tree)
    kind = _recipe(name)
    if kind == "paint":
        _paint(T, color, spec.roughness)
    elif kind == "paint_decal":
        _paint(T, color, spec.roughness,
               decal_mix=decal_target or decal_target_color(livery_colors("standart")))
    elif kind == "gloss":
        _paint(T, color, min(spec.roughness, 0.32), coat=0.6, coat_rough=0.08)
    elif kind == "orange":
        _paint(T, color, spec.roughness, glow=ORANGE_GLOW)
    elif kind == "smoke":
        _smoke(T, spec, color)
    elif kind == "pacf":
        _pacf(T, spec, color)
    elif kind == "nozzle":
        _nozzle(T, spec, color)
    elif kind == "wood":
        _prop_wood(T, spec, color)
    elif kind == "spun":
        _spun_metal(T, spec, color)
    elif kind == "anod":
        _metal(T, spec, color, aniso=0.35)
    elif kind == "chrome":
        _metal(T, spec, color, rough=min(spec.roughness, 0.14), var=0.02)
    elif kind == "metal":
        _metal(T, spec, color)
    elif kind == "rubber":
        _rubber(T, spec, color)
    elif kind == "soft_black":
        _soft_black(T, spec, color)
    elif kind == "glass":
        _sensor_glass(T, spec, color)
    elif kind == "carbon":
        _carbon(T, spec, color)
    elif kind == "light":
        _light_lens(T, spec, color)
    else:
        _generic(T, spec, color)
    # görünüm penceresi (Solid) rengi
    mat.diffuse_color = _rgba(color if kind != "smoke" else _scale(color, 2.0))
    mat.roughness = float(spec.roughness)
    mat.metallic = float(spec.metallic)
    mat["ucav_recipe"] = kind
    return mat


def ensure_light_props() -> list[str]:
    """Işık malzemesi kullanan nesnelerde ``ucav_emission`` yoksa 1,0 yazar (rig sonradan sürer)."""
    out = []
    for ob in bpy.data.objects:
        if ob.type != "MESH" or ob.data is None:
            continue
        if any(m is not None and m.name in LIGHT_MATERIALS for m in ob.data.materials):
            if EMISSION_PROP not in ob.keys():
                ob[EMISSION_PROP] = 1.0
                ob.id_properties_ui(EMISSION_PROP).update(min=0.0, max=1.0,
                                                          description="Işık şiddeti çarpanı (rig sürer)")
            out.append(ob.name)
    return out


def build(livery: str = "standart", *, decals: bool = True, scene: bpy.types.Scene | None = None) -> dict:
    """Bütün ``UM_*`` malzemelerini ``livery`` boya şemasıyla kurar; ``decals`` True ise işaretleri ekler.
    Tekrar çağrılabilir (ör. ``build("taktik")`` yalnız renkleri ve işaret kontrastını değiştirir).
    Döndürür: {"livery", "materials", "lights", "decals", "missing"} — ``missing``: sahnede malzemesi olmayan
    ya da ``UM_*`` dışı yuva taşıyan ağ nesneleri (boş olmalı)."""
    scene = scene or bpy.context.scene
    key = _livery_key(livery)
    cols = livery_colors(key)
    target = decal_target_color(cols)
    built = [build_material(n, cols[n], target).name for n in P.MATERIALS]
    lights = ensure_light_props()
    dec = build_decals(key, scene) if decals else []
    scene["ucav_livery"] = key
    missing = []
    for ob in bpy.data.objects:
        if ob.type == "MESH" and ob.name.startswith("U_"):
            ms = list(ob.data.materials)
            if not ms or any(m is None or m.name not in P.MATERIALS for m in ms):
                missing.append(ob.name)
    return {"livery": key, "materials": built, "lights": lights, "decals": dec, "missing": missing}


# =====================================================================================================
# İşaretler (metin ağı + BVH izdüşümü)
# =====================================================================================================
@dataclass
class DecalSpec:
    """Bir işaretin yerleşimi (Blender dünya koordinatı)."""
    name: str
    host: str
    text: str | None            # None → şerit (bant)
    material: str
    height: float               # yazı büyük harf yüksekliği (m)
    center: tuple               # yaklaşık yüzey noktası
    u: tuple                    # okuma yönü (taban çizgisi)
    n: tuple                    # yüzey dışa normali (izdüşüm yönü −n)
    strip: np.ndarray | None = None     # bant için (k, m, 3) ızgara noktaları (satır: yükseklik, sütun: genişlik)


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
    return out


def _decal_mesh(ds: DecalSpec, bvh: BVHTree) -> tuple[np.ndarray, list, int]:
    n = Vector(ds.n).normalized()
    if ds.strip is not None:
        k, m = ds.strip.shape[:2]
        pts = ds.strip.reshape(-1, 3)
        V, N, miss = _project(bvh, pts, n, lift=0.03)               # ızgarayı yüzeye −n yönünde izdüşür
        F = [[i * m + j, i * m + j + 1, (i + 1) * m + j + 1, (i + 1) * m + j]
             for i in range(k - 1) for j in range(m - 1)]
        V, F = _solidify(V, N, F, n)
        return V, F, miss
    hit = _surface_point(bvh, ds.center, n)
    if hit is None:
        raise RuntimeError(f"{ds.name}: yüzey bulunamadı ({ds.host})")
    c, n_s = hit
    n = n_s.normalized()
    u = Vector(ds.u)
    u = (u - n * u.dot(n)).normalized()
    v = n.cross(u)
    V2, F = text_outline_mesh(ds.text, DECAL_MAX_EDGE / ds.height)
    pts = np.array([c + u * (x * ds.height) + v * (y * ds.height) for x, y in V2])
    V, N, miss = _project(bvh, pts, n)
    V, F = _solidify(V, N, F, n)
    return V, F, miss


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
        V, F, miss = _decal_mesh(ds, bvhs[ds.host])
        if miss > 0.02 * len(V):
            print(f"[materials] uyarı: {ds.name} için {miss}/{len(V)} nokta yüzeye düşmedi")
        me = bpy.data.meshes.new(ds.name)
        me.from_pydata([tuple(p) for p in V], [], F)
        me.validate()
        me.materials.append(U.get_material(ds.material))
        me.shade_smooth()
        ob = bpy.data.objects.new(ds.name, me)
        col.objects.link(ob)
        U.parent_keep_world(ob, host)                  # köşeler dünyada; nesne dönüşümü kimlik
        ob[DECAL_MIX_PROP] = float(DECAL_MIX[key]) if ds.text else 0.0
        ob["ucav_decal"] = ds.text or "prop_band"
        ob["ucav_closed"] = True
        ob.visible_shadow = False
        made.append(ob.name)
    return made
