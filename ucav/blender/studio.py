"""YELKOVAN YK-38 — stüdyo (bpy 4.5): Cycles ayarları, dünya, zemin, ışıklar ve sabit kameralar — ``setup()``.

İki ortam
---------
* ``"pist"`` (varsayılan): Nishita fiziksel gökyüzü (güneş diski kapalı; yerine aynı yönde 0,8° açılı Güneş
  lambası → net ama yumuşak kenarlı gölge), 8 m × 230 m asfalt pist (eşik çizgisi, "piyano" tuşları, "09"/"27"
  numaraları, kesikli eksen çizgisi, kenar çizgileri, hedef noktası blokları; aşınmış boya, lastik izleri),
  biçilmiş çim (6 m biçme şeritleri), 4,3 km'de alçak tepe/ağaç sırası silueti ve kamera ışınlarına uygulanan
  hava perspektifi (zemin uzaklıkla ufkun ~3° üstündeki gök rengine karışır → ufukta dikiş yok). Gökyüzünün
  ufuk altı da ufuk rengiyle doldurulur (alttan bakışta siyah yarım küre yok). Pist ``+X`` yönündedir; uçak
  dinlenme konumunda eşiğin hemen ilerisinde, eksen çizgisi üzerindedir (``animation`` gösterim klibinin
  kalkış koşusu ve tırmanışı bu pistte/alanda geçer).
* ``"studyo"``: ufuksuz koyu "sonsuz" stüdyo — antrasit fon (beyaz uçakla kontrast), uçağın altında ışık
  havuzu olan yarı parlak zemin (3 m → 12 m arasında fona karışır, her bakış açısında dikişsiz); ızgaralı
  büyük anahtar softbox, dolgu, iki kontur ışığı ve tepe şerit softbox (boyada uzun yansıma çizgisi).
  Ürün kataloğu görünümü; ``mechanisms`` klibinin varsayılan ortamı.

Işık yönü ``set_sun(azimut, yükseklik)`` ile değişir (azimut uçak eksenine göre: 0° = burun yönü, +90° =
iskele/sol). Pistte Güneş lambası, gökyüzü ve zemindeki pus rengi birlikte döner; stüdyoda ışık düzeneği döner.

Pozlama ve yer dolgusu
----------------------
* Pozlama ortama göredir (``EXPOSURE``): pistte −0,4 EV (beyaz boya AgX omzunda sıkışmaz, iki boya şeması ayrı
  okunur), stüdyoda 0. ``configure_cycles`` sahnenin ``ucav_env`` değerinden okur.
* ``S_UnderFill``: zeminin 3 cm üstünde yukarı bakan, kameraya/yansımaya görünmez alan ışığı — koyu asfaltın
  vermediği yer yansıması (beyaz alt yüzeyler ve kanat kökü gölgede beyaz okunur). ``U_Root``'un X/Y'sini izler;
  gücü ``ucav_fill_w`` × irtifa sönümüdür (basit ifade sürücüsü: dinlenme yüksekliğinin 1,5 m üstünde sıfır →
  gösterim klibinde kalkıştan ≈ 1 s sonra söner). Pistte varsayılan 35 W, stüdyoda kapalı; görünüm başına
  ``under_fill`` (``under`` 60 W, ``top`` 0).

Kameralar (``VIEWS``)
---------------------
``U_Cam_<görünüm>``: hero (ön-sol 3/4, 70 mm, alçak, f/5.6, güneş 24° — uzun gölge), rear34 (arka-sağ 3/4),
side (iskele yanı, ORTOGRAFİK, yükseklik = kanat dihedrali 4° → yakın kanat ince çizgi; alçak güneş), front
(135 mm), top, under (alttan; takım toplu, zemin kamera ışınına görünmez, alttan dolgu ışığı), nose (taret yakın
plan, 85 mm, f/4), tail (U-kuyruk ve itici pervane, f/5.6), gearbay (sol ana takım ve açık kuyu, 28 mm, f/4).
Uzaklık, uçağın gerçek köşe noktalarından kadraja sığdırılarak hesaplanır (``fit=None``, ``margin`` > 1 kanat
uçlarını kırpar) ya da hedef çevresinde verilen yarıçapa göre (``fit``); ortografik görünümde (``ortho``) ölçek ve
kadraj ortası aynı noktalardan bulunur. Uçak yoksa params zarfı kullanılır.
Kameralar ``U_CamRig_Stills`` boşluğuna bağlıdır; bu boşluk ``U_Root`` konumunu ve baş açısını izler →
animasyonun herhangi bir karesinde de sabit kameralar uçağı kadrajlar.

Görünüm başına ``controls`` (``U_Root`` kontrol paneli değerleri), ``hide_ground``, ``sun``, ``under_fill`` ve
alan derinliği (``fstop``, odak = uzaklık × ``focus``) ``apply_view`` ile uygulanır, ``restore_view`` ile geri
alınır (``render.render_stills`` bunu her görünümde yapar; animasyonlu kontrol eğrileri o süre için susturulur).

Nesne adları: stüdyo ağları/ışıkları ``S_*``, malzemeleri ``SM_*``, dünyalar ``SW_*`` (``UM_*`` uçak
malzemelerinden ayrı). Koleksiyonlar (``UCAV_Studio`` altında, renk etiketli): ``UCAV_Env`` (zemin, gökyüzü/stüdyo
ışıkları, dolgu), ``UCAV_Cameras_Stills`` (``U_Cam_<görünüm>`` + ``U_CamRig_Stills``; ebeveyn tersi birim),
``animation`` modülünün ``UCAV_Cameras_Anim`` ve ``UCAV_Stand``'i. ``setup`` tekrar çağrılabilir (eski stüdyo
nesneleri silinir; animasyon kameraları ``U_Cam_Showcase_*``/``U_Cam_Mechanisms`` dokunulmaz).
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field

import bpy
import numpy as np
from mathutils import Matrix, Vector

from .. import params as P
from . import util as U

ENVS = ("pist", "studyo")
STUDIO = "UCAV_Studio"
STILLS_COL = "UCAV_Cameras_Stills"         # sabit görüntü kameraları + U_CamRig_Stills (UCAV_Studio altında)
ENV_COL = "UCAV_Env"                       # zemin, gökyüzü ışıkları, stüdyo ışıkları, dolgu (S_*; UCAV_Studio altında)
SUB_COLORS = {STILLS_COL: "COLOR_05", ENV_COL: "COLOR_07"}
RIG = "U_CamRig_Stills"
LIGHT_RIG = "S_LightRig"
SUN = "S_Sun"

# Varsayılan güneş (uçak eksenine göre; pist +X yönünde) — varsayım
SUN_AZ_DEG = 58.0
SUN_EL_DEG = 33.0
SUN_STRENGTH = 4.2                 # W/m² (Cycles birimi; AgX ile doğal pozlama)
SUN_ANGLE_DEG = 0.8                # yumuşak gölge kenarı
SKY_STRENGTH = 0.18                # Nishita parlaklığı (güneş diski kapalı)
SKY_DUST = 0.4                     # pus (ufuk yumuşaklığı)
HAZE_DIST_M = 6000.0               # hava perspektifi e-katlama uzaklığı (≈ 20 km görüş)
HAZE_SKY_Z = 0.05                  # pus rengi: ufkun ~3° üstündeki gökyüzü (biraz mavimsi)
SKY_OZONE = 3.0                    # daha derin mavi gök

# Pist (varsayım: küçük İHA test pisti)
RUNWAY_W = 8.0
RUNWAY_X0 = -22.0                  # eşik tarafı (uçak X≈0'da burun)
RUNWAY_X1 = 208.0
RUNWAY_NUMBER = "09"
GRASS_DROP = 0.03                  # çim pist yüzeyinden 3 cm aşağıda
GROUND_HALF = 6000.0               # zemin düzlemi yarı boyu (m)

UNDER_FILL = "S_UnderFill"
UNDER_FILL_W = 60.0                # alt görünüş dolgu ışığı (zeminden yansıyan gün ışığı taklidi)
GROUND_FILL_W = 35.0               # diğer pist görünüşleri ve animasyon: koyu asfaltın vermediği yer yansıması
FILL_PROP = "ucav_fill_w"          # S_UnderFill nesne özelliği: temel güç (W); sürücü irtifayla söndürür
FILL_FADE_M = 1.5                  # U_Root dinlenme yüksekliğinin bu kadar üstünde dolgu sıfır (kalkıştan ≈ 1 s sonra)
ENV_FILL_W = {"pist": GROUND_FILL_W, "studyo": 0.0}

# Pozlama (EV): pistte iki boya şeması da AgX omzunun altında kalsın (standart boya ≤ 240, taktik koyu okunur)
EXPOSURE = {"pist": -0.4, "studyo": 0.0}

# Stüdyo
STUDIO_FLOOR_R = 40.0
STUDIO_BG = (0.040, 0.042, 0.046)  # doğrusal fon: koyu antrasit (beyaz uçakla kontrast)


# =====================================================================================================
# Cycles ve renk yönetimi
# =====================================================================================================
def configure_cycles(scene: bpy.types.Scene | None = None, samples: int = 96, *, adaptive: float = 0.015,
                     look: str = "AgX - Medium High Contrast", exposure: float | None = None) -> None:
    """Cycles CPU, uyarlamalı örnekleme, OIDN gürültü giderme (albedo+normal), ışık ağacı, sınırlı sekmeler,
    AgX görünüm dönüşümü. 4 çekirdekli CPU için dengeli varsayılanlar. ``exposure`` None → ortamın değeri
    (``EXPOSURE``: pist −0,4 EV, stüdyo 0)."""
    scene = scene or bpy.context.scene
    r = scene.render
    r.engine = "CYCLES"
    c = scene.cycles
    c.device = "CPU"
    c.feature_set = "SUPPORTED"
    c.samples = int(samples)
    c.preview_samples = 16
    c.use_adaptive_sampling = True
    c.adaptive_threshold = adaptive
    c.adaptive_min_samples = 0
    c.use_denoising = True
    c.denoiser = "OPENIMAGEDENOISE"
    c.denoising_input_passes = "RGB_ALBEDO_NORMAL"
    c.denoising_prefilter = "ACCURATE"
    if hasattr(c, "denoising_quality"):
        c.denoising_quality = "HIGH"
    c.use_light_tree = True
    c.max_bounces = 12
    c.diffuse_bounces = 4
    c.glossy_bounces = 4
    c.transmission_bounces = 8
    c.transparent_max_bounces = 8
    c.volume_bounces = 0
    c.caustics_reflective = False
    c.caustics_refractive = False
    c.sample_clamp_direct = 0.0
    c.sample_clamp_indirect = 8.0
    c.blur_glossy = 0.5
    c.pixel_filter_type = "BLACKMAN_HARRIS"
    c.filter_width = 1.5
    c.use_animated_seed = True
    r.film_transparent = False
    r.use_persistent_data = True
    r.threads_mode = "AUTO"
    r.use_border = False
    vs = scene.view_settings
    scene.display_settings.display_device = "sRGB"
    vs.view_transform = "AgX"
    try:
        vs.look = look
    except TypeError:
        vs.look = "None"
    vs.exposure = float(EXPOSURE.get(scene.get("ucav_env", "pist"), 0.0) if exposure is None else exposure)
    vs.gamma = 1.0


# =====================================================================================================
# Yardımcılar
# =====================================================================================================
def _studio_col(scene, sub: str | None = None) -> bpy.types.Collection:
    """``UCAV_Studio`` ya da onun alt koleksiyonu (``STILLS_COL``, ``ENV_COL``; renk etiketli)."""
    cols = U.ensure_collections(scene)
    if sub is None:
        return cols[STUDIO]
    col = U.ensure_collection(sub, cols[STUDIO], scene)
    if hasattr(col, "color_tag"):
        col.color_tag = SUB_COLORS.get(sub, "NONE")
    return col


def _remove(prefixes: tuple[str, ...]) -> None:
    for ob in [o for o in bpy.data.objects if o.name.startswith(prefixes)]:
        data = ob.data
        bpy.data.objects.remove(ob, do_unlink=True)
        if data is not None and data.users == 0:
            if isinstance(data, bpy.types.Mesh):
                bpy.data.meshes.remove(data)
            elif isinstance(data, bpy.types.Light):
                bpy.data.lights.remove(data)
            elif isinstance(data, bpy.types.Camera):
                bpy.data.cameras.remove(data)


def _mesh_object(name: str, verts, faces, mat: bpy.types.Material | None, col, smooth: bool = False):
    me = bpy.data.meshes.get(name)
    if me is not None and me.users == 0:
        bpy.data.meshes.remove(me)
    me = bpy.data.meshes.new(name)
    me.from_pydata([tuple(map(float, v)) for v in verts], [], [list(map(int, f)) for f in faces])
    me.validate()
    if mat is not None:
        me.materials.append(mat)
    if smooth:
        me.shade_smooth()
    ob = bpy.data.objects.new(name, me)
    col.objects.link(ob)
    return ob


def _sun_vector(az_deg: float, el_deg: float) -> Vector:
    a, e = math.radians(az_deg), math.radians(el_deg)
    return Vector((math.cos(e) * math.cos(a), math.cos(e) * math.sin(a), math.sin(e)))


def _new_material(name: str) -> tuple[bpy.types.Material, "object"]:
    from .materials import _Tree
    mat = bpy.data.materials.get(name) or bpy.data.materials.new(name)
    mat.use_nodes = True
    return mat, _Tree(mat.node_tree)


# =====================================================================================================
# Dünya
# =====================================================================================================
def _sky_node(nt, az_deg: float, el_deg: float, x: float = -300.0, y: float = 0.0):
    sky = nt.nodes.new("ShaderNodeTexSky")
    sky.location = (x, y)
    sky.name = "UCAV_Sky"
    sky.sky_type = "NISHITA"
    sky.sun_disc = False
    sky.sun_elevation = math.radians(el_deg)
    sky.sun_rotation = math.radians(90.0 - az_deg)        # Nishita: yatay yön = (sin rot, cos rot)
    sky.altitude = 0.0
    sky.air_density = 1.0
    sky.dust_density = SKY_DUST
    sky.ozone_density = SKY_OZONE
    return sky


def _horizon_clamped_dir(T, vec_socket, zmin: float = 0.015, x: float = -900.0, y: float = 0.0):
    """Yönün z bileşenini ``zmin``'e kırpıp normalize eder (ufuk altı → ufuk rengi)."""
    sep = T.node("ShaderNodeSeparateXYZ", x, y)
    T.link(vec_socket, sep.inputs[0])
    z = T.math("MAXIMUM", sep.outputs["Z"], zmin, x=x + 180, y=y - 120)
    comb = T.node("ShaderNodeCombineXYZ", x + 360, y)
    T.link(sep.outputs["X"], comb.inputs["X"])
    T.link(sep.outputs["Y"], comb.inputs["Y"])
    T.link(z, comb.inputs["Z"])
    nrm = T.node("ShaderNodeVectorMath", x + 540, y, operation="NORMALIZE")
    T.link(comb.outputs[0], nrm.inputs[0])
    return nrm.outputs["Vector"]


def build_world_sky(scene, az_deg: float = SUN_AZ_DEG, el_deg: float = SUN_EL_DEG) -> bpy.types.World:
    from .materials import _Tree
    w = bpy.data.worlds.get("SW_Pist") or bpy.data.worlds.new("SW_Pist")
    w.use_nodes = True
    T = _Tree(w.node_tree)
    tc = T.node("ShaderNodeTexCoord", -1300, 0)
    d = _horizon_clamped_dir(T, tc.outputs["Generated"])
    sky = _sky_node(w.node_tree, az_deg, el_deg)
    T.link(d, sky.inputs["Vector"])
    bg = T.node("ShaderNodeBackground", 0, 0, ins={"Strength": SKY_STRENGTH})
    T.link(sky.outputs["Color"], bg.inputs["Color"])
    out = T.node("ShaderNodeOutputWorld", 250, 0)
    T.link(bg.outputs[0], out.inputs["Surface"])
    scene.world = w
    return w


def build_world_studio(scene) -> bpy.types.World:
    """Stüdyo dünyası: kamera ışınlarına düz fon rengi; aydınlatmaya üstte parlak, altta koyu yumuşak kubbe."""
    from .materials import _Tree
    w = bpy.data.worlds.get("SW_Studyo") or bpy.data.worlds.new("SW_Studyo")
    w.use_nodes = True
    T = _Tree(w.node_tree)
    tc = T.node("ShaderNodeTexCoord", -1100, 0)
    sep = T.node("ShaderNodeSeparateXYZ", -900, 0)
    T.link(tc.outputs["Generated"], sep.inputs[0])
    f = T.map_range(sep.outputs["Z"], 0.0, 1.0, -0.2, 0.9, x=-700, y=0)
    ramp = T.node("ShaderNodeValToRGB", -500, 0)
    ramp.color_ramp.elements[0].color = (0.02, 0.021, 0.023, 1.0)
    ramp.color_ramp.elements[1].color = (0.60, 0.62, 0.65, 1.0)
    T.link(f, ramp.inputs["Fac"])
    bg_light = T.node("ShaderNodeBackground", -200, -100, ins={"Strength": 0.22})
    T.link(ramp.outputs["Color"], bg_light.inputs["Color"])
    bg_cam = T.node("ShaderNodeBackground", -200, 150, ins={"Color": (*STUDIO_BG, 1.0), "Strength": 1.0})
    lp = T.node("ShaderNodeLightPath", -500, 300)
    mix = T.node("ShaderNodeMixShader", 50, 0)
    T.link(lp.outputs["Is Camera Ray"], mix.inputs["Fac"])
    T.link(bg_light.outputs[0], mix.inputs[1])
    T.link(bg_cam.outputs[0], mix.inputs[2])
    out = T.node("ShaderNodeOutputWorld", 250, 0)
    T.link(mix.outputs[0], out.inputs["Surface"])
    scene.world = w
    return w


# =====================================================================================================
# Zemin malzemeleri (SM_*)
# =====================================================================================================
def _aerial(T, shader_socket, scene, az_deg, el_deg, x: float = 400.0):
    """Kamera ışınlarında yüzeyi uzaklıkla ufuk gökyüzü rengine karıştırır (hava perspektifi)."""
    cd = T.node("ShaderNodeCameraData", x - 900, -500)
    k = T.math("DIVIDE", cd.outputs["View Distance"], -HAZE_DIST_M, x=x - 700, y=-500)
    e = T.math("EXPONENT", k, None, x=x - 550, y=-500)
    fac = T.math("SUBTRACT", 1.0, e, x=x - 400, y=-500)
    lp = T.node("ShaderNodeLightPath", x - 550, -750)
    fac = T.math("MULTIPLY", fac, lp.outputs["Is Camera Ray"], x=x - 250, y=-600)
    geo = T.node("ShaderNodeNewGeometry", x - 1300, -900)
    neg = T.node("ShaderNodeVectorMath", x - 1100, -900, operation="SCALE", ins={"Scale": -1.0})
    T.link(geo.outputs["Incoming"], neg.inputs[0])
    d = _horizon_clamped_dir(T, neg.outputs["Vector"], HAZE_SKY_Z, x=x - 950, y=-900)
    sky = _sky_node(T.nt, az_deg, el_deg, x=x - 300, y=-900)
    T.link(d, sky.inputs["Vector"])
    em = T.node("ShaderNodeEmission", x - 100, -800, ins={"Strength": SKY_STRENGTH})
    T.link(sky.outputs["Color"], em.inputs["Color"])
    mix = T.node("ShaderNodeMixShader", x, 0)
    T.link(fac, mix.inputs["Fac"])
    T.link(shader_socket, mix.inputs[1])
    T.link(em.outputs[0], mix.inputs[2])
    return mix.outputs[0]


def _grass_material(scene, az, el) -> bpy.types.Material:
    """Biçilmiş çim: iki ölçekli yeşil/saman karışımı, Voronoi öbekleri, 6 m biçme şeritleri, tümsek."""
    mat, T = _new_material("SM_Grass")
    tc = T.coords(-1800, 0)
    obj = tc.outputs["Object"]
    n1 = T.noise(obj, 0.08, 4.0, 0.6, x=-1500, y=300)
    n2 = T.noise(obj, 1.6, 3.0, 0.6, x=-1500, y=0)
    dark, light, straw = (0.018, 0.042, 0.008), (0.052, 0.092, 0.016), (0.11, 0.10, 0.040)
    c1 = T.mix_rgb(n1.outputs["Fac"], dark, light, x=-1200, y=300)
    straw_f = T.map_range(n2.outputs["Fac"], 0.0, 0.55, 0.5, 0.78, x=-1200, y=0)
    c2 = T.mix_rgb(straw_f, c1, straw, x=-1000, y=200)
    vor = T.node("ShaderNodeTexVoronoi", -1500, -300, ins={"Scale": 9.0, "Randomness": 1.0})
    T.link(obj, vor.inputs["Vector"])
    clump = T.map_range(vor.outputs["Distance"], 0.86, 1.10, 0.0, 1.0, x=-1200, y=-300)
    wave = T.node("ShaderNodeTexWave", -1500, 600, wave_type="BANDS", bands_direction="Y", wave_profile="SIN",
                  ins={"Scale": 0.1047, "Distortion": 0.6, "Detail": 1.0})        # pist boyunca 6 m periyot
    T.link(obj, wave.inputs["Vector"])
    stripe = T.map_range(wave.outputs["Fac"], 0.86, 1.14, 0.35, 0.65, x=-1200, y=600)
    val = T.math("MULTIPLY", clump, stripe, x=-900, y=-100)
    hsv = T.node("ShaderNodeHueSaturation", -700, 100)
    T.link(c2, hsv.inputs["Color"])
    T.link(val, hsv.inputs["Value"])
    nb = T.noise(obj, 260.0, 3.0, 0.7, x=-900, y=-600)
    nrm = T.bump(nb.outputs["Fac"], 0.5, 0.004, x=-600, y=-600)
    bsdf = T.principled(-300, 0, Base_Color=hsv.outputs["Color"], Roughness=0.95, Specular_IOR_Level=0.3,
                        Normal=nrm)
    out = _aerial(T, bsdf.outputs[0], scene, az, el)
    T.output(out, 600, 0)
    return mat


def _asphalt_material(scene, az, el) -> bpy.types.Material:
    mat, T = _new_material("SM_Asphalt")
    tc = T.coords(-1800, 0)
    obj = tc.outputs["Object"]
    vor = T.node("ShaderNodeTexVoronoi", -1500, 300, ins={"Scale": 140.0, "Randomness": 1.0})
    T.link(obj, vor.inputs["Vector"])
    agg = T.map_range(vor.outputs["Distance"], 1.0, 0.0, 0.0, 0.32, x=-1250, y=300)
    n_patch = T.noise(obj, 0.25, 4.0, 0.6, x=-1500, y=0)
    mp = T.node("ShaderNodeMapping", -1650, -300, ins={"Scale": (0.03, 3.0, 1.0)})
    T.link(obj, mp.inputs["Vector"])
    n_tire = T.noise(mp.outputs["Vector"], 3.0, 3.0, 0.6, x=-1450, y=-300)
    sep = T.node("ShaderNodeSeparateXYZ", -1450, -550)
    T.link(obj, sep.inputs[0])
    ay = T.math("ABSOLUTE", sep.outputs["Y"], None, x=-1250, y=-550)
    lane = T.map_range(ay, 1.0, 0.0, 0.3, 1.8, x=-1100, y=-550)               # lastik izleri eksen çevresinde
    tire = T.math("MULTIPLY", lane, T.map_range(n_tire.outputs["Fac"], 0.0, 1.0, 0.5, 0.7, x=-1100, y=-300),
                  x=-900, y=-400, clamp=True)
    base_a, base_b = (0.050, 0.050, 0.052), (0.085, 0.084, 0.082)
    c = T.mix_rgb(n_patch.outputs["Fac"], base_a, base_b, x=-1000, y=0)
    c = T.mix_rgb(agg, c, (0.20, 0.20, 0.195), x=-800, y=100)
    c = T.mix_rgb(T.math("MULTIPLY", tire, 0.35, x=-750, y=-300), c, (0.022, 0.022, 0.023), x=-600, y=0)
    rough = T.map_range(agg, 0.82, 0.62, 0.0, 1.0, x=-600, y=-150)
    nrm = T.bump(vor.outputs["Distance"], 0.35, 0.0015, x=-600, y=-650)
    bsdf = T.principled(-300, 0, Base_Color=c, Roughness=rough, Normal=nrm)
    out = _aerial(T, bsdf.outputs[0], scene, az, el)
    T.output(out, 600, 0)
    return mat


def _paint_material(scene, az, el) -> bpy.types.Material:
    """Pist boyası: beyaz, aşınmış (gürültüyle asfalta açılan delikler), retro-yansıtıcı boncuk yok."""
    mat, T = _new_material("SM_RunwayPaint")
    tc = T.coords(-1500, 0)
    obj = tc.outputs["Object"]
    n = T.noise(obj, 3.5, 5.0, 0.65, x=-1200, y=0)
    wear = T.map_range(n.outputs["Fac"], 1.0, 0.0, 0.30, 0.36, x=-950, y=0)
    vor = T.node("ShaderNodeTexVoronoi", -1200, -300, ins={"Scale": 140.0})
    T.link(obj, vor.inputs["Vector"])
    c = T.mix_rgb(wear, (0.66, 0.66, 0.64), (0.07, 0.07, 0.072), x=-700, y=0)
    nrm = T.bump(vor.outputs["Distance"], 0.15, 0.001, x=-600, y=-400)
    bsdf = T.principled(-300, 0, Base_Color=c, Roughness=0.62, Normal=nrm)
    out = _aerial(T, bsdf.outputs[0], scene, az, el)
    T.output(out, 600, 0)
    return mat


def _studio_floor_material() -> bpy.types.Material:
    mat, T = _new_material("SM_StudioFloor")
    tc = T.coords(-1200, 0)
    r = T.node("ShaderNodeVectorMath", -1000, -250, operation="LENGTH")
    T.link(tc.outputs["Object"], r.inputs[0])
    fade = T.map_range(r.outputs["Value"], 0.0, 1.0, 3.0, 12.0, x=-800, y=-250)     # ışık havuzu → fon
    bsdf = T.principled(-400, 100, Base_Color=(0.050, 0.052, 0.056), Roughness=0.42, Specular_IOR_Level=0.18)
    em = T.node("ShaderNodeEmission", -400, -200, ins={"Color": (*STUDIO_BG, 1.0), "Strength": 1.0})
    lp = T.node("ShaderNodeLightPath", -600, -450)
    f2 = T.math("MULTIPLY", fade, lp.outputs["Is Camera Ray"], x=-300, y=-350)
    mix = T.node("ShaderNodeMixShader", 0, 0)
    T.link(f2, mix.inputs["Fac"])
    T.link(bsdf.outputs[0], mix.inputs[1])
    T.link(em.outputs[0], mix.inputs[2])
    T.output(mix.outputs[0], 250, 0)
    return mat


# =====================================================================================================
# Zemin geometrisi
# =====================================================================================================
def _box(x0, x1, y0, y1, z0, z1):
    v = [(x0, y0, z0), (x1, y0, z0), (x1, y1, z0), (x0, y1, z0), (x0, y0, z1), (x1, y0, z1), (x1, y1, z1), (x0, y1, z1)]
    f = [(0, 3, 2, 1), (4, 5, 6, 7), (0, 1, 5, 4), (1, 2, 6, 5), (2, 3, 7, 6), (3, 0, 4, 7)]
    return v, f


def runway_marks() -> tuple[list, list]:
    """Pist işaretleri (dünya koordinatı, z = zemin + 0,4 mm): eşik çizgisi, piyano tuşları, numara, eksen
    çizgisi, kenar çizgileri, hedef noktası blokları — iki uçta simetrik."""
    from .materials import text_outline_mesh
    zg = P.GROUND_Z + 0.0004
    V: list = []
    F: list = []

    def quad(x0, x1, y0, y1):
        b = len(V)
        V.extend([(x0, y0, zg), (x1, y0, zg), (x1, y1, zg), (x0, y1, zg)])
        F.append((b, b + 1, b + 2, b + 3))

    hw = RUNWAY_W / 2
    L = RUNWAY_X1 - RUNWAY_X0
    for end in (0, 1):
        sgn = 1.0 if end == 0 else -1.0
        x_th = RUNWAY_X0 + 0.6 if end == 0 else RUNWAY_X1 - 0.6

        def X(d):                                   # eşikten piste doğru uzaklık → dünya X
            return x_th + sgn * d
        quad(*sorted((X(0.0), X(0.45))), -hw + 0.2, hw - 0.2)                          # eşik çizgisi
        for k in range(3):                                                              # piyano tuşları
            yc = 0.95 + k * 0.95
            for s in (1, -1):
                quad(*sorted((X(1.2), X(6.2))), *sorted((s * (yc - 0.24), s * (yc + 0.24))))
        # numara: tabanı yaklaşmaya bakar (yukarı = pist yönü), okunuş sağa (−Y) — uçta 180° döner
        txt = RUNWAY_NUMBER if end == 0 else f"{(int(RUNWAY_NUMBER) + 18 - 1) % 36 + 1:02d}"
        V2, F2 = text_outline_mesh(txt, 0.5)
        h_num = 3.0
        b = len(V)
        xc = X(8.6)
        for x2, y2 in V2:
            # metin x → −Y (sgn), metin y → +X (sgn)
            V.append((xc + sgn * y2 * h_num, -sgn * x2 * h_num * 1.25, zg))
        F.extend([tuple(b + i for i in f) for f in F2])
        for s in (1, -1):                                                               # hedef noktası blokları
            quad(*sorted((X(34.0), X(40.0))), *sorted((s * 1.2, s * 2.0)))
    # eksen çizgisi: 3 m çizgi + 2 m boşluk (eşik işaretlerinin arasından)
    x = RUNWAY_X0 + 18.6
    while x + 3.0 < RUNWAY_X1 - 18.6:
        quad(x, x + 3.0, -0.09, 0.09)
        x += 5.0
    # kenar çizgileri
    for s in (1, -1):
        quad(RUNWAY_X0 + 0.6, RUNWAY_X1 - 0.6, *sorted((s * (hw - 0.30), s * (hw - 0.15))))
    return V, F


def _hills(n: int = 2880, r: float = 4300.0, seed: int = 38) -> tuple[list, list]:
    """Uzak alçak tepe halkası (hava perspektifiyle siluet olur): birkaç uzun dalga + ağaç sırası pürüzü."""
    rng = np.random.default_rng(seed)
    t = np.linspace(0, 2 * np.pi, n, endpoint=False)
    h = np.zeros(n)
    for k, amp in ((2, 14.0), (5, 11.0), (11, 7.0), (23, 4.0), (47, 2.5), (97, 1.4)):
        h += amp * np.sin(k * t + rng.uniform(0, 2 * np.pi))
    tree = np.convolve(rng.normal(0.0, 1.0, n), np.ones(5) / 5, mode="same")         # ağaç sırası pürüzü
    h = np.clip(16.0 + h + 2.2 * tree, 3.0, None)
    zg = P.GROUND_Z - GRASS_DROP
    V, F = [], []
    for i in range(n):
        c, s = math.cos(t[i]), math.sin(t[i])
        V.append((r * c, r * s, zg - 2.0))
        V.append(((r + 120) * c, (r + 120) * s, zg + h[i]))
        V.append(((r + 900) * c, (r + 900) * s, zg + 0.7 * h[i]))
    for i in range(n):
        j = (i + 1) % n
        a, b = 3 * i, 3 * j
        F.append((a, b, b + 1, a + 1))
        F.append((a + 1, b + 1, b + 2, a + 2))
    return V, F


def build_ground_pist(scene, az=SUN_AZ_DEG, el=SUN_EL_DEG) -> list[str]:
    col = _studio_col(scene, ENV_COL)
    grass = _grass_material(scene, az, el)
    asph = _asphalt_material(scene, az, el)
    paint = _paint_material(scene, az, el)
    zg = P.GROUND_Z
    G = GROUND_HALF
    made = []
    ob = _mesh_object("S_Ground", [(-G, -G, zg - GRASS_DROP), (G, -G, zg - GRASS_DROP), (G, G, zg - GRASS_DROP),
                                   (-G, G, zg - GRASS_DROP)], [(0, 1, 2, 3)], grass, col)
    made.append(ob.name)
    v, f = _box(RUNWAY_X0, RUNWAY_X1, -RUNWAY_W / 2, RUNWAY_W / 2, zg - 0.08, zg)
    made.append(_mesh_object("S_Runway", v, f, asph, col).name)
    V, F = runway_marks()
    ob = _mesh_object("S_RunwayMarks", V, F, paint, col)
    ob.visible_shadow = False
    made.append(ob.name)
    V, F = _hills()
    ob = _mesh_object("S_Hills", V, F, grass, col, smooth=True)
    ob.visible_shadow = False
    made.append(ob.name)
    return made


def build_ground_studio(scene) -> list[str]:
    col = _studio_col(scene, ENV_COL)
    mat = _studio_floor_material()
    n = 96
    ang = 2 * math.pi * np.arange(n) / n
    V = [(0.0, 0.0, 0.0)] + [(STUDIO_FLOOR_R * math.cos(a), STUDIO_FLOOR_R * math.sin(a), 0.0) for a in ang]
    F = [(0, 1 + i, 1 + (i + 1) % n) for i in range(n)]
    ob = _mesh_object("S_StudioFloor", V, F, mat, col)
    ob.location = (P.U_ROOT_B[0], 0.0, P.GROUND_Z)            # nesne koordinatı merkezi = uçak ortası
    return [ob.name]


# =====================================================================================================
# Işıklar
# =====================================================================================================
def _light(name: str, kind: str, col, energy: float, color=(1.0, 1.0, 1.0), **props) -> bpy.types.Object:
    li = bpy.data.lights.get(name)
    if li is not None and li.users == 0:
        bpy.data.lights.remove(li)
    li = bpy.data.lights.new(name, kind)
    li.energy = energy
    li.color = color
    for k, v in props.items():
        setattr(li, k, v)
    ob = bpy.data.objects.new(name, li)
    col.objects.link(ob)
    return ob


def _aim(ob, target: Vector) -> None:
    d = (target - ob.location)
    ob.rotation_euler = d.to_track_quat("-Z", "Y").to_euler()


def build_lights_pist(scene, az=SUN_AZ_DEG, el=SUN_EL_DEG) -> list[str]:
    col = _studio_col(scene, ENV_COL)
    sun = _light(SUN, "SUN", col, SUN_STRENGTH, (1.0, 0.955, 0.90), angle=math.radians(SUN_ANGLE_DEG))
    _orient_sun(sun, az, el)
    return [sun.name, _under_fill(col, ENV_FILL_W["pist"]).name]


def _under_fill(col, watts: float = 0.0) -> bpy.types.Object:
    """Zeminin 3 cm üstünde yukarı bakan geniş, kameraya ve yansımalara görünmez dolgu ışığı: koyu asfaltın
    vermediği yer yansımasını (beyaz alt yüzeyler gölgede beyaz okunsun) taklit eder. ``U_Root``'un X/Y'sini
    izler (Z zeminde kilitli); güç = ``ucav_fill_w`` × irtifa sönümü (basit ifade sürücüsü: dinlenme
    yüksekliğinden ``FILL_FADE_M`` yukarıda sıfır → kalkıştan sonra kendiliğinden söner)."""
    ob = _light(UNDER_FILL, "AREA", col, 0.0, (1.0, 0.98, 0.95), shape="RECTANGLE", size=5.0, size_y=3.5)
    ob.location = (P.U_ROOT_B[0], 0.0, P.GROUND_Z + 0.03)
    ob.rotation_euler = (math.pi, 0.0, 0.0)                         # ışık +Z yönünde
    ob.visible_camera = False
    ob.visible_glossy = False
    ob.visible_transmission = False
    ob[FILL_PROP] = float(watts)
    ob.id_properties_ui(FILL_PROP).update(min=0.0, max=200.0, description="Yer yansıması dolgu gücü (W)")
    ob.hide_render = watts <= 0
    root = bpy.data.objects.get("U_Root")
    if root is not None:
        cl = ob.constraints.new("COPY_LOCATION")
        cl.target = root
        cl.use_z = False
        cl.use_offset = False
    li = ob.data
    fc = li.driver_add("energy")
    d = fc.driver
    d.type = "SCRIPTED"
    v = d.variables.new()
    v.name, v.type = "w", "SINGLE_PROP"
    v.targets[0].id_type = "OBJECT"
    v.targets[0].id = ob
    v.targets[0].data_path = f'["{FILL_PROP}"]'
    if root is not None:
        z = d.variables.new()
        z.name, z.type = "z", "TRANSFORMS"
        z.targets[0].id = root
        z.targets[0].transform_type = "LOC_Z"
        z.targets[0].transform_space = "WORLD_SPACE"
        d.expression = f"w * clamp(1 - (z - {P.U_ROOT_B[2]:.6g}) / {FILL_FADE_M:.6g}, 0, 1)"
    else:
        d.expression = "w"
    return ob


def set_fill(watts: float) -> None:
    """Yer yansıması dolgu gücü (W; 0 = kapalı)."""
    ob = bpy.data.objects.get(UNDER_FILL)
    if ob is None:
        return
    ob[FILL_PROP] = float(watts)
    ob.hide_render = watts <= 0


def _orient_sun(sun, az, el) -> None:
    d = _sun_vector(az, el)
    sun.location = Vector(P.U_ROOT_B) + d * 20.0
    sun.rotation_euler = d.to_track_quat("Z", "Y").to_euler()       # lamba −Z yönünde ışık verir


def build_lights_studio(scene) -> list[str]:
    """Anahtar (ön-sol üst, büyük softbox), dolgu (sağ), iki kontur (arka), tepe şerit softbox."""
    col = _studio_col(scene, ENV_COL)
    rig = bpy.data.objects.get(LIGHT_RIG) or bpy.data.objects.new(LIGHT_RIG, None)
    if rig.name not in col.objects:
        col.objects.link(rig)
    rig.empty_display_type = "CIRCLE"
    rig.location = Vector(P.U_ROOT_B)
    rig.rotation_euler = (0.0, 0.0, 0.0)
    tgt = Vector(P.U_ROOT_B) + Vector((0.2, 0.0, -0.05))
    specs = [
        # ad, azimut, yükseklik, uzaklık, boyut (x, y), güç (W), renk, yayılma (°; ızgaralı softbox)
        ("S_Key", 50.0, 42.0, 6.5, (3.2, 2.2), 85.0, (1.0, 0.97, 0.93), 55.0),
        ("S_Fill", -70.0, 18.0, 7.5, (4.0, 3.0), 22.0, (0.92, 0.96, 1.0), 70.0),
        ("S_Rim_L", 150.0, 30.0, 6.0, (1.0, 3.0), 45.0, (1.0, 1.0, 1.0), 45.0),
        ("S_Rim_R", -145.0, 34.0, 6.0, (1.0, 3.0), 40.0, (1.0, 1.0, 1.0), 45.0),
        ("S_Strip", 0.0, 89.0, 4.0, (5.0, 0.45), 18.0, (1.0, 1.0, 1.0), 60.0),
    ]
    made = [rig.name]
    for name, az, el, dist, size, w, colr, spread in specs:
        ob = _light(name, "AREA", col, w, colr, shape="RECTANGLE", size=size[0], size_y=size[1],
                    spread=math.radians(spread))
        ob.location = tgt + _sun_vector(az, el) * dist
        _aim(ob, tgt)
        if name == "S_Strip":
            ob.rotation_euler = (0.0, 0.0, 0.0)
        U.parent_keep_world(ob, rig)
        made.append(ob.name)
    return made


def set_sun(az_deg: float | None = None, el_deg: float | None = None, scene=None) -> tuple[float, float]:
    """Güneş/ışık düzeneği yönü (uçak eksenine göre azimut, derece). ``None`` → varsayılan.
    Pistte Güneş lambası, dünya gökyüzü ve zemin malzemelerindeki ufuk gökyüzü birlikte güncellenir."""
    scene = scene or bpy.context.scene
    az = SUN_AZ_DEG if az_deg is None else float(az_deg)
    el = SUN_EL_DEG if el_deg is None else float(el_deg)
    heading = _rig_heading()
    az_w = az + math.degrees(heading)
    sun = bpy.data.objects.get(SUN)
    if sun is not None:
        _orient_sun(sun, az_w, el)
    trees = []
    if scene.world is not None and scene.world.node_tree is not None:
        trees.append(scene.world.node_tree)
    trees += [bpy.data.materials[n].node_tree for n in ("SM_Grass", "SM_Asphalt", "SM_RunwayPaint")
              if n in bpy.data.materials]
    for nt in trees:
        for node in nt.nodes:
            if node.type == "TEX_SKY":
                node.sun_elevation = math.radians(el)
                node.sun_rotation = math.radians(90.0 - az_w)
    rig = bpy.data.objects.get(LIGHT_RIG)
    if rig is not None:
        rig.rotation_euler = (0.0, 0.0, math.radians(az_w - 50.0))     # anahtar ışık azimutu 50°
    scene["ucav_sun"] = (az, el)
    return az, el


def _rig_heading() -> float:
    root = bpy.data.objects.get("U_Root")
    if root is None:
        return 0.0
    return float(root.matrix_world.to_euler("XYZ").z)


# =====================================================================================================
# Kameralar
# =====================================================================================================
@dataclass
class ViewSpec:
    """Sabit kamera tanımı. Açılar uçak eksenine göre: azimut 0° = burun yönü, +90° = iskele (sol);
    yükseklik + = yukarıdan. ``target`` spec koordinatı (s, y, z). ``fit`` None → bütün uçak kadraja sığar
    (``margin`` kenar payı); sayı → hedef çevresinde bu yarıçap (m) kadraja sığar."""
    name: str
    title: str
    az: float
    el: float
    target: tuple = (1.15, 0.0, 0.0)
    lens: float = 50.0
    fit: float | None = None
    margin: float = 0.90
    aspect: float = 1.6
    fstop: float | None = None
    controls: dict = field(default_factory=dict)
    hide_ground: bool = False
    sun: tuple | None = None
    up: tuple | None = None          # tepe/alt görünüşte ekran yukarısı (dünya)
    shift: tuple = (0.0, 0.0)
    under_fill: float = 0.0          # alttan dolgu ışığı gücü (W); 0 = kapalı
    ortho: bool = False              # ortografik kamera (ölçek ``margin`` ile uçağa sığdırılır)
    focus: float = 1.0               # alan derinliği odak uzaklığı = kamera uzaklığı × bu


GF = GROUND_FILL_W
VIEWS: dict[str, ViewSpec] = {v.name: v for v in [
    ViewSpec("hero", "Kahraman — ön-sol 3/4", 33.0, 6.0, (1.05, 0.0, 0.0), 70.0, margin=1.42, sun=(118.0, 24.0),
             fstop=5.6, focus=0.92, under_fill=GF),
    ViewSpec("rear34", "Arka-sağ 3/4", -138.0, 15.0, (1.25, 0.0, 0.03), 50.0, margin=1.12, sun=(-80.0, 34.0),
             under_fill=GF),
    # yan: ortografik, yükseklik = kanat dihedrali → yakın kanat ince bir çizgi (alt yüzü "koyu çokgen" olmaz);
    # alçak, kamera tarafından güneş + biraz güçlü yer dolgusu → beyaz alt yüz (RAL 9003) üstten açık okunur
    ViewSpec("side", "Yan (iskele, ortografik)", 90.0, P.WING_DIHEDRAL, (1.24, 0.0, 0.02), 135.0, margin=0.93,
             sun=(70.0, 16.0), under_fill=42.0, ortho=True),
    ViewSpec("front", "Ön", 0.0, 3.0, (1.20, 0.0, 0.0), 135.0, margin=0.93, sun=(40.0, 36.0), under_fill=GF),
    ViewSpec("top", "Üst", 0.0, 89.9, (1.24, 0.0, 0.0), 85.0, margin=0.92, aspect=1.6, up=(1.0, 0.0, 0.0),
             sun=(70.0, 55.0)),
    ViewSpec("under", "Alt (takım toplu)", 0.0, -89.9, (1.24, 0.0, 0.0), 85.0, margin=0.92, up=(1.0, 0.0, 0.0),
             hide_ground=True, controls={"gear": 0.0}, sun=(70.0, 55.0), under_fill=UNDER_FILL_W),
    ViewSpec("nose", "Burun ve EO/IR taret", 32.0, -4.0, (0.25, 0.0, -0.075), 85.0, fit=0.20, fstop=4.0,
             controls={"turret_pan_deg": 18.0, "turret_tilt_deg": -12.0}, sun=(70.0, 24.0), under_fill=GF),
    ViewSpec("tail", "U-kuyruk ve itici pervane", -152.0, 9.0, (2.21, 0.0, 0.12), 70.0, fit=0.42, fstop=5.6,
             sun=(-95.0, 30.0), under_fill=GF),
    ViewSpec("gearbay", "Sol ana takım ve kuyu", 38.0, -10.0, (1.31, 0.20, -0.10), 28.0, fit=0.24, fstop=4.0,
             controls={"gear_doors": 1.0}, sun=(80.0, 28.0), under_fill=GF),
]}


def aircraft_points(scene=None, per_object: int = 1500) -> np.ndarray:
    """Uçak nesnelerinin (``UCAV`` koleksiyonları, render'da görünür ağlar) dünya köşe noktaları; nesne başına
    en çok ``per_object`` nokta (seyreltilmiş) — kadraja sığdırma için sıkı dış zarf."""
    pts = []
    root_col = bpy.data.collections.get("UCAV")
    cols = [root_col] + list(root_col.children_recursive) if root_col else []
    seen = set()
    dg = bpy.context.evaluated_depsgraph_get()
    for c in cols:
        for ob in c.objects:
            if ob.type != "MESH" or ob.name in seen or ob.hide_render or ob.name.startswith("U_Stand_"):
                continue
            seen.add(ob.name)
            ev = ob.evaluated_get(dg)
            me = ev.data
            n = len(me.vertices)
            if n == 0:
                continue
            co = np.empty(n * 3, np.float32)
            me.vertices.foreach_get("co", co)
            co = co.reshape(-1, 3)[:: max(1, n // per_object)]
            M = np.asarray(ev.matrix_world, float)
            pts.append(co @ M[:3, :3].T + M[:3, 3])
    return np.vstack(pts) if pts else np.zeros((0, 3))


def _param_envelope() -> np.ndarray:
    """Uçak kurulmadan kamera kadrajı için kaba zarf (params): burun, kuyruk, kanat uçları, teker teması."""
    span = P.WING_SEMI_SPAN
    fin = P.fin_station(float(P.SPEC["tail"]["fin"]["height_m"]), "L")
    pts = [(0.0, 0.0, 0.0), (fin.te_s, 0.0, 0.0), (fin.te_s, fin.le[1], fin.le[2]), (fin.te_s, -fin.le[1], fin.le[2]),
           (P.wing_station(span).le_s, span, P.wing_reference()["tip_z"]),
           (P.wing_station(span).le_s, -span, P.wing_reference()["tip_z"])]
    out = [P.to_blender(*p) for p in pts]
    out += [(x, y, P.GROUND_Z) for x, y, _ in out]
    return np.asarray(out, float)


def _look_rotation(cam_pos: Vector, target: Vector, up: Vector | None):
    fwd = (target - cam_pos).normalized()
    up = Vector((0.0, 0.0, 1.0)) if up is None else Vector(up)
    right = fwd.cross(up)
    if right.length < 1e-6:
        right = fwd.cross(Vector((1.0, 0.0, 0.0)))
    right.normalize()
    cam_up = right.cross(fwd).normalized()
    R = Matrix((right, cam_up, -fwd)).transposed()     # sütunlar: kamera X, Y, Z
    return R


def _fit_distance(view: ViewSpec, target: Vector, pts: np.ndarray, sensor: float = 36.0) -> float:
    """Bütün noktalar kadrajın ``margin`` oranına sığacak en küçük uzaklık (ikiye bölme)."""
    tan_h = (sensor / 2) / view.lens
    tan_v = tan_h / view.aspect
    d = _view_dir(view)
    R = None

    def ok(dist):
        nonlocal R
        cam = target + d * dist
        R = _look_rotation(cam, target, Vector(view.up) if view.up else None)
        Q = (np.asarray(pts) - np.asarray(cam)) @ np.asarray(R)          # kamera eksenleri
        z = -Q[:, 2]
        if (z <= 0.05).any():
            return False
        return (np.abs(Q[:, 0] / z).max() <= tan_h * view.margin) and (np.abs(Q[:, 1] / z).max() <= tan_v * view.margin)
    lo, hi = 0.3, 200.0
    for _ in range(60):
        mid = 0.5 * (lo + hi)
        if ok(mid):
            hi = mid
        else:
            lo = mid
    return hi


def _fit_ortho(view: ViewSpec, target: Vector, pts: np.ndarray, dist: float = 30.0) -> tuple[Vector, float]:
    """Ortografik kadraj: kamera ekseni ``target``'tan geçen bakış yönünde; uçak zarfının ortası kadraj ortasına
    kaydırılır, ölçek (geniş kenar) zarf + ``margin`` payına göre. Döndürür: (kamera konumu, ortho_scale)."""
    d = _view_dir(view)
    cam = target + d * dist
    R = _look_rotation(cam, target, Vector(view.up) if view.up else None)
    Q = (np.asarray(pts) - np.asarray(cam)) @ np.asarray(R)
    lo, hi = Q[:, :2].min(0), Q[:, :2].max(0)
    c = 0.5 * (lo + hi)
    w, h = hi - lo
    scale = max(w, h * view.aspect) / view.margin
    Rm = np.asarray(R)
    cam = cam + Vector(Rm[:, 0] * c[0] + Rm[:, 1] * c[1])
    return cam, float(scale)


def _view_dir(view: ViewSpec) -> Vector:
    return _sun_vector(view.az, view.el)


def _camera_obj(name: str, col) -> bpy.types.Object:
    U.remove_object(name)
    cam = bpy.data.cameras.get(name)
    if cam is None or cam.users > 0:
        cam = bpy.data.cameras.new(name)
    ob = bpy.data.objects.new(name, cam)
    col.objects.link(ob)
    return ob


def build_cameras(scene=None) -> list[str]:
    """``VIEWS`` kameralarını kurar; ``U_CamRig_Stills`` (``U_Root`` konum + baş açısı izler) altına bağlar.
    Uzaklıklar dinlenme pozundaki uçağa göre hesaplanır (takım açık)."""
    scene = scene or bpy.context.scene
    col = _studio_col(scene, STILLS_COL)
    root = bpy.data.objects.get("U_Root")
    rest = Matrix.Translation(Vector(P.U_ROOT_B))
    # dinlenme pozu noktaları: kök şu an başka yerdeyse geri taşı
    pts = aircraft_points(scene)
    if root is not None and len(pts):
        M = np.asarray(rest @ root.matrix_world.inverted(), float)
        pts = pts @ M[:3, :3].T + M[:3, 3]
    if len(pts) == 0:                                           # uçak henüz kurulmadı: params zarfı
        pts = _param_envelope()
    rig = bpy.data.objects.get(RIG)
    if rig is None:
        rig = bpy.data.objects.new(RIG, None)
    if rig.name not in col.objects:                               # eski dosyada UCAV_Studio'daysa taşı
        for c in list(rig.users_collection):
            c.objects.unlink(rig)
        col.objects.link(rig)
    rig.empty_display_type = "CUBE"
    rig.empty_display_size = 0.2
    rig.hide_render = True
    rig.location = Vector(P.U_ROOT_B)
    for c in list(rig.constraints):
        rig.constraints.remove(c)
    if root is not None:
        cl = rig.constraints.new("COPY_LOCATION")
        cl.target = root
        cr = rig.constraints.new("COPY_ROTATION")
        cr.target = root
        cr.use_x = cr.use_y = False
    made = []
    for v in VIEWS.values():
        ob = _camera_obj(f"U_Cam_{v.name}", col)
        cam = ob.data
        cam.lens = v.lens
        cam.sensor_fit = "HORIZONTAL"
        cam.sensor_width = 36.0
        cam.clip_start = 0.01
        cam.clip_end = 12000.0
        cam.shift_x, cam.shift_y = v.shift
        tgt = Vector(P.to_blender(*v.target))
        if v.ortho:
            pos, scale = _fit_ortho(v, tgt, pts)
            cam.type = "ORTHO"
            cam.ortho_scale = scale
            dist = (pos - tgt).length
            ob["ucav_ortho_scale"] = round(scale, 4)
        else:
            if v.fit is None:
                dist = _fit_distance(v, tgt, pts)
            else:                                               # yarıçap kısa kenara (düşey) sığar
                dist = v.fit / ((36.0 / 2) / v.lens / v.aspect)
            pos = tgt + _view_dir(v) * dist
        R = _look_rotation(pos, pos - _view_dir(v), Vector(v.up) if v.up else None)
        ob.parent = rig                                         # dünya = rig · (rest⁻¹ · taban) → dinlenmede taban
        ob.matrix_parent_inverse = Matrix.Identity(4)           # birim ebeveyn tersi (rig modülüyle aynı kural)
        ob.matrix_basis = rest.inverted() @ Matrix.Translation(pos) @ R.to_4x4()
        cam.dof.use_dof = v.fstop is not None and not v.ortho
        if cam.dof.use_dof:
            cam.dof.aperture_fstop = v.fstop
            cam.dof.focus_distance = dist * v.focus
            cam.dof.aperture_blades = 7
        ob["ucav_view"] = v.title
        ob["ucav_distance_m"] = round(dist, 3)
        made.append(ob.name)
    return made


# =====================================================================================================
# Görünüm uygulama (render modülü kullanır)
# =====================================================================================================
_GROUND = ("S_Ground", "S_Runway", "S_RunwayMarks", "S_StudioFloor")


def apply_view(name: str, scene=None) -> dict:
    """Görünümü etkinleştirir: etkin kamera, ışık yönü, zemin görünürlüğü, kontrol değerleri. Geri alma için
    eski durumu döndürür (``restore_view``). Kontrol özelliği animasyonluysa o eğri geçici susturulur; zaman
    çizelgesindeki kamera işaretleri (``animation`` klipleri) render'da etkin kamerayı ezdiği için geçici çözülür."""
    scene = scene or bpy.context.scene
    v = VIEWS[name]
    cam = bpy.data.objects.get(f"U_Cam_{name}")
    if cam is None:
        raise KeyError(f"kamera yok: U_Cam_{name} (önce studio.setup())")
    state = {"camera": scene.camera.name if scene.camera else None,
             "sun": tuple(scene.get("ucav_sun", (SUN_AZ_DEG, SUN_EL_DEG))),
             "ground": {}, "controls": {}, "muted": [], "markers": []}
    for m in scene.timeline_markers:                # kamera işaretleri render'da etkin kamerayı ezer → geçici çöz
        if m.camera is not None:
            state["markers"].append((m.name, m.frame, m.camera.name))
            m.camera = None
    scene.camera = cam
    if v.sun is not None:
        set_sun(v.sun[0], v.sun[1], scene)
    fill = bpy.data.objects.get(UNDER_FILL)
    if fill is not None:
        state["fill"] = (fill.hide_render, float(fill.get(FILL_PROP, 0.0)))
        set_fill(v.under_fill)
    for g in _GROUND:
        ob = bpy.data.objects.get(g)
        if ob is not None:
            state["ground"][g] = ob.visible_camera
            ob.visible_camera = not v.hide_ground
    root = bpy.data.objects.get("U_Root")
    if root is not None and v.controls:
        ad = root.animation_data
        for k, val in v.controls.items():
            if k not in root.keys():
                continue
            state["controls"][k] = float(root[k])
            if ad is not None and ad.action is not None:
                for fc in _fcurves(ad.action):
                    if fc.data_path == f'["{k}"]' and not fc.mute:
                        fc.mute = True
                        state["muted"].append(fc.data_path)
            root[k] = float(val)
        _refresh()
    else:
        bpy.context.view_layer.update()
    return state


def restore_view(state: dict, scene=None) -> None:
    scene = scene or bpy.context.scene
    for name, frame, cam_name in state.get("markers", []):
        for m in scene.timeline_markers:
            if m.name == name and m.frame == frame:
                m.camera = bpy.data.objects.get(cam_name)
    if state.get("camera"):
        scene.camera = bpy.data.objects.get(state["camera"])
    set_sun(*state["sun"], scene=scene)
    fill = bpy.data.objects.get(UNDER_FILL)
    if fill is not None and "fill" in state:
        fill[FILL_PROP] = state["fill"][1]
        fill.hide_render = state["fill"][0]
    for g, vis in state["ground"].items():
        ob = bpy.data.objects.get(g)
        if ob is not None:
            ob.visible_camera = vis
    root = bpy.data.objects.get("U_Root")
    if root is not None:
        for k, val in state["controls"].items():
            root[k] = val
        ad = root.animation_data
        if ad is not None and ad.action is not None:
            for fc in _fcurves(ad.action):
                if fc.data_path in state["muted"]:
                    fc.mute = False
        _refresh()


def _fcurves(action):
    if hasattr(action, "fcurves") and len(getattr(action, "fcurves", [])):
        return list(action.fcurves)
    out = []
    for layer in getattr(action, "layers", []):                 # katmanlı aksiyonlar (4.4+)
        for strip in layer.strips:
            for bag in getattr(strip, "channelbags", []):
                out.extend(bag.fcurves)
    return out


def _refresh() -> None:
    try:
        from . import rig as RIG_MOD
        RIG_MOD.refresh()
    except Exception:                                           # rig yoksa yalnız görünüm katmanını güncelle
        bpy.context.view_layer.update()


# =====================================================================================================
# Kurulum
# =====================================================================================================
def current_env(scene=None) -> str | None:
    scene = scene or bpy.context.scene
    return scene.get("ucav_env")


def setup(scene: bpy.types.Scene | None = None, *, env: str = "pist", samples: int = 96, cameras: bool = True,
          sun: tuple | None = None) -> dict:
    """Stüdyoyu kurar: Cycles/AgX ayarları, dünya, zemin, ışıklar, sabit kameralar. ``env`` ∈ {"pist", "studyo"}.
    Tekrar çağrılabilir; ortam değiştirilebilir. Döndürür: {"env", "objects", "cameras", "world"}."""
    scene = scene or bpy.context.scene
    if env not in ENVS:
        raise KeyError(f"bilinmeyen ortam: {env!r} ({', '.join(ENVS)})")
    scene["ucav_env"] = env
    configure_cycles(scene, samples)
    _remove(("S_",))
    az, el = sun if sun is not None else (SUN_AZ_DEG, SUN_EL_DEG)
    if env == "pist":
        world = build_world_sky(scene, az, el)
        objs = build_ground_pist(scene, az, el) + build_lights_pist(scene, az, el)
    else:
        world = build_world_studio(scene)
        objs = build_ground_studio(scene) + build_lights_studio(scene) + \
            [_under_fill(_studio_col(scene, ENV_COL), ENV_FILL_W["studyo"]).name]
    set_sun(az, el, scene)
    cams = build_cameras(scene) if cameras else []
    if cams and (scene.camera is None or not scene.camera.name.startswith("U_Cam_")):
        scene.camera = bpy.data.objects[cams[0]]
    return {"env": env, "objects": objs, "cameras": cams, "world": world.name}
