"""YELKOVAN YK-38 — kontrol paneli ve sürücüler (bpy 4.5): ``setup(scene=None)``.

``U_Root`` üzerinde sözleşmedeki özel özellikler (Türkçe açıklama, min/maks) kurulur; hareketli bütün parçalar
bunlara Blender **basit ifade** sürücüleriyle bağlanır (Python betikleri kapalıyken de çalışır). Animasyon yalnız
bu özelliklere ve ``U_Root`` dönüşümüne anahtar kare koyar (pişirilmiş pervane turu ``U_Prop["ucav_turns"]``
dışında). Sürücüler sahneye (``SCENE``) bağlanmaz: ``UCAV`` koleksiyonu başka dosyaya eklenince (append/link)
yalnız uçak gelir; bütün kontrol özellikleri kütüphane geçersiz kılmasına (library override) açıktır.

Kontrol paneli (``U_Root``)
---------------------------
=================  ==========  ===================================================================================
özellik            aralık      anlam
=================  ==========  ===================================================================================
``gear``           0…1         0 = toplu, 1 = açık/kilitli. Tek değer sırayı sürer: kapaklar 0–0,15 açılır,
                               bacaklar 0,15–0,85, kapaklar 0,85–1,0 kapanır (``params.gear_phase`` ile aynı)
``gear_doors``     0…1         deri kapaklarını el ile açar (otomatik sıranın üstüne, büyük olan geçerli)
``aileron_deg``    −25…25      + = sağa yatış; sol firar kenarı aşağı. Diferansiyel 1,67:1 → yukarı ≤ 20°, aşağı ≤ 12°
``flap_deg``       0…35        iç + dış flap birlikte, + = aşağı; mekanik sınır 30°
``elevator_deg``   −25…25      + = firar kenarı aşağı (burun aşağı); sınır −25 / +20
``rudder_deg``     −25…25      + = firar kenarı sancağa (burun sağa); sınır ±22; burun tekeri ∓30° (yalnız takım açık)
``prop_rpm``       0…9000      pervane devri (dev/dk)
``prop_auto``      0/1         pervane açısı kipi: 1 = kare·rpm/1440 tur (sabit devirde tam, 24 fps); 0 = pişirilmiş
                               tur ``U_Prop["ucav_turns"]`` (devir değişirken doğru; ``YK38_pervane_pisir.py``)
``turret_pan_deg`` −180…180    + = iskeleye bakar
``turret_tilt_deg`` −90…20     + = yukarı, −90 = nadir
``nav_lights``     0/1         seyrüsefer (kırmızı/yeşil) ve iniş ışığı
``strobe``         0/1         beyaz çakarlar: 30 karede bir çift çakış (kare 0–1 ve 5–6)
``status_led``     0/1         taret durum LED'i (bakım/test); uçuşta ve kliplerde 0 — gerçek EO/IR taretler ışımaz
``ground_z``       m           zemin yüksekliği (dünya Z): amortisörler bu düzleme oturur (varsayılan
                               ``params.GROUND_Z``). Uçağı kendi zemininize koyunca burayı o zeminin Z'sine eşitleyin
``wheel_auto``     0/1         1 = tekerler ``U_Root``'un baş yönündeki yer ilerlemesiyle kaymadan döner
``wheel_roll_m``   m           ek yuvarlanma yolu: açı = (wheel_roll_m + wheel_auto·ilerleme)/R (klipler pişirir:
                               kalkıştan sonra yavaşlama, takım toplanırken fren)
=================  ==========  ===================================================================================

Sürülen kanallar (``driver_table()`` tam listeyi verir)
------------------------------------------------------
* ``U_GearPivot_N/L/R``.rotation_euler.x = radians(retract)·(1 − smoothstep(0,15; 0,85; gear))
* Deri kapakları ``U_Door_N_1/2``, ``U_Door_L/R_1``.rotation_euler.x = radians(open)·max(sıra, gear_doors)
* ``U_GearSlider_X``.location.z = z0 + clamp((ground_z − (aks_z − R))/cos(yatıklık), 0, strok) — yer teması
  (``U_GearAxleRef_X`` dünya Z'si); havada amortisör uzar, yerde statik çöker.
* ``U_GearLinkU/L_X``.rotation_euler.y — makas açısı (acos), amortisörle birlikte.
* ``U_GearWheel_X``.rotation_euler.y = (wheel_roll_m + wheel_auto·((X − X0)·cos ψ + Y·sin ψ))/R — ``U_Root`` dünya
  konumunun baş yönüne izdüşümü (ψ = dünya Z açısı): her başta kaymadan yuvarlanma.
* ``U_GearSteer_N``.rotation_euler.z = −radians(rudder·30/22)·smoothstep(0,85; 1; gear).
* Kumanda yüzeyleri ``rotation_euler.x`` (yerel X = ``HingeLine.axis_positive_b``: + = firar kenarı aşağı/sancağa).
* ``U_Prop``.rotation_euler.x = (U_Prop["ucav_turns"] + prop_auto·kare·rpm/1440)·2π.
  ``prop_auto`` = 1 iken sabit devirde açı tamdır; devir anahtarlanırsa açı ∫rpm·dt değildir (açısal hız
  rpm + kare·d(rpm)/dt olur → pervane geri dönüyormuş gibi görünür). Bu yüzden devri değişen animasyonda
  ``YK38_pervane_pisir.py`` metin bloğu (``.blend`` içinde) ``ucav_turns``'ü ∫rpm/60·dt olarak pişirir ve
  ``prop_auto`` = 0 yapar; klipler de böyle pişirilmiştir.
* ``U_Turret_Pan``.rotation_euler.z, ``U_Turret_Tilt``.rotation_euler.y = −tilt.
* Işıklar: ışık nesnelerinde ``ucav_emission`` (0…1) sürülür (seyrüsefer/iniş: ``nav_lights``, çakarlar:
  ``strobe``, taret halkası: ``status_led``); ``materials`` modülünün ışık malzemeleri yayım şiddetini bu nesne
  özelliğiyle (Attribute düğümü, Object) çarpar. ``drive_light_materials()`` yalnız bağlı olmayan Principled
  "Emission Strength" / Emission "Strength" soketlerini sürer (yer tutucu malzemeler için yedek yol).
* ``U_PropDisc`` (yalnız ``params.MATERIALS``'ta ``UM_PropDisc`` varsa): ``ucav_disc`` = clamp((rpm − 900)/2600,
  0, 1)·0,35 — hızlı dönen pervanenin soluk diski (malzeme bu özellikle karışır); dururken render dışı.
* Kumanda bağlantıları (``ensure_linkages``; kanatçık, dış flap, irtifa, istikamet × 2): ``U_Horn_*`` boynuz
  (yüzeye bağlı, deliği menteşe hattının dik altında), ``U_ServoArm_*`` servo kolu (servis kapağından çıkar,
  ``rotation_euler.x`` = yüzeyle aynı ifade), ``U_Pushrod_*`` Ø1,6 çelik itme çubuğu + çatallar (servo koluna bağlı,
  ``rotation_euler.x`` = −aynı ifade → dönmeden öteler). Paralelkenar bağlantı: kol = boynuz vektörü.

Ebeveyn kuralı (``normalize_parenting``): ``UCAV`` ağacındaki bütün çocukların ebeveyn ters matrisi birimdir;
konum ebeveyne göre yereldir, yerel eksenler ``delta_rotation_euler``'dedir (dünya dönüşümü korunur; "Clear Parent
Inverse" hiçbir şeyi oynatmaz). ``materials`` çıkartmaları rig'den sonra kurulduğundan ``animation.build`` de çağırır.

Görünüm penceresi kolaylıkları: ``U_Root`` büyük, önde çizilen ve adı görünen bir okla seçilir; sürülen parçaların
ve boynuzların konum/dönüş/ölçeği kilitlidir (G/R/S ile menteşeden kaçmaz); koleksiyonlar renk etiketlidir; ``UCAV``
koleksiyonu varlık (asset) olarak işaretlidir (Asset Browser'dan sürüklenebilir).

Sıra: ``airframe.build()`` → ``gear.build()`` → ``rig.setup()`` → ``materials.build()`` (ikisi yer değiştirebilir).
Aralıklar ``spec.yaml → rig.ranges_deg``'dendir.
"""
from __future__ import annotations

import math

import bpy

from .. import params as P

ROOT = "U_Root"
_G = P.SPEC["landing_gear"]
_SEQ = _G["sequence"]
STEER_MAX = float(_G["nose"].get("steer_deg", 30.0))
STROBE_PERIOD = 30                    # kare (24 fps'de 1,25 s). varsayım
STROBE_FLASHES = ((0, 2), (5, 7))     # [başlangıç, bitiş) kare — çift çakış. varsayım
PROP_FPS = 24.0                       # prop_auto = 1 formülünün varsaydığı kare hızı (1440 = 24·60)
PROP_TURNS = "ucav_turns"             # U_Prop: pişirilmiş tur sayısı (∫rpm/60·dt)
DISC_RPM = (900.0, 2600.0, 0.35)      # pervane diski: eşik dev/dk, tam görünüm aralığı, en büyük örtme. varsayım
BAKE_TEXT = "YK38_pervane_pisir.py"
_RANGE = {k: tuple(float(x) for x in v) for k, v in P.SPEC.get("rig", {}).get("ranges_deg", {}).items()}


def _rng(name: str, default: tuple[float, float]) -> tuple[float, float]:
    """Kontrol paneli aralığı: ``spec.yaml → rig.ranges_deg`` (yoksa sözleşme varsayılanı)."""
    return _RANGE.get(name, default)

# (ad, varsayılan, min, maks, açıklama)
PROPS: list[tuple[str, float, float, float, str]] = [
    ("gear", 1.0, 0.0, 1.0,
     "İniş takımı: 0 = toplu, 1 = açık ve kilitli. Tek değer sırayı sürer: kapaklar 0–0,15 açılır, bacaklar "
     "0,15–0,85, kapaklar 0,85–1,0 kapanır"),
    ("gear_doors", 0.0, 0.0, 1.0,
     "Takım kapakları el ile: 0 = otomatik sıra, 1 = bütün deri kapakları açık (sıradakiyle büyük olan geçerli)"),
    ("aileron_deg", 0.0, *_rng("aileron", (-25.0, 25.0)),
     "Kanatçık (°): + = sağa yatış (sol firar kenarı aşağı, sağ yukarı). Diferansiyel 1,67:1 — yukarı en çok 20°, "
     "aşağı en çok 12°"),
    ("flap_deg", 0.0, *_rng("flap", (0.0, 35.0)),
     "Flap (°): iç ve dış flaplar birlikte, + = firar kenarı aşağı. Mekanik sınır 30°"),
    ("elevator_deg", 0.0, *_rng("elevator", (-25.0, 25.0)),
     "İrtifa dümeni (°): + = firar kenarı aşağı (burun aşağı). Mekanik sınır −25° yukarı / +20° aşağı"),
    ("rudder_deg", 0.0, *_rng("rudder", (-25.0, 25.0)),
     "İstikamet dümeni (°): + = firar kenarı sancağa (burun sağa). Sınır ±22°. Takım açıkken burun tekeri de döner"),
    ("prop_rpm", 0.0, 0.0, 9000.0,
     "Pervane devri (dev/dk); arkadan bakınca saat yönü tersine. Devri anahtarladıysanız 'YK38_pervane_pisir.py' "
     "betiğini çalıştırın (açı = ∫rpm·dt)"),
    ("prop_auto", 1.0, 0.0, 1.0,
     "Pervane açısı kipi: 1 = kare·prop_rpm/1440 tur (sabit devirde tam, 24 fps); 0 = pişirilmiş tur "
     "U_Prop['ucav_turns'] (devir değişen animasyonda; 'YK38_pervane_pisir.py' yazar)"),
    ("turret_pan_deg", 0.0, -180.0, 180.0, "Taret yatay açısı (°): + = iskeleye (sola) bakar"),
    ("turret_tilt_deg", 0.0, -90.0, 20.0, "Taret düşey açısı (°): + = yukarı, −90 = tam aşağı (nadir)"),
    ("nav_lights", 1.0, 0.0, 1.0,
     "Seyrüsefer ışıkları (iskele kırmızı, sancak yeşil) ve iniş ışığı: 0 = kapalı, 1 = açık"),
    ("strobe", 1.0, 0.0, 1.0, "Beyaz çakar ışıklar (dikey uçlarında): 0 = kapalı, 1 = açık (kareye bağlı çift çakış)"),
    ("status_led", 0.0, 0.0, 1.0,
     "Taret durum LED'i (bakım/test): 0 = kapalı, 1 = yanık. Gerçek EO/IR taretler ışımaz; uçuşta ve kliplerde 0"),
    ("ground_z", float(P.GROUND_Z), -1000.0, 1000.0,
     "Zemin yüksekliği (dünya Z, m): amortisörler ve tekerler bu düzleme oturur. Uçağı kendi sahnenizde başka bir "
     "zemine koyunca burayı o zeminin Z'sine eşitleyin (U_Root zeminin 0,2964 m üstünde)"),
    ("wheel_auto", 1.0, 0.0, 1.0,
     "Tekerler kendiliğinden dönsün: 1 = U_Root'un baş yönündeki yer ilerlemesiyle kaymadan, 0 = yalnız wheel_roll_m"),
    ("wheel_roll_m", 0.0, -1.0e5, 1.0e5,
     "Teker yuvarlanma yolu (m), açıya eklenir: açı = (wheel_roll_m + wheel_auto·ilerleme)/R. Klipler pişirir "
     "(kalkıştan sonra yavaşlama, takım toplanırken fren)"),
]
PROP_NAMES = tuple(p[0] for p in PROPS)
_PRECISION = {"ground_z": 4, "wheel_roll_m": 3}
_SOFT = {"ground_z": (-10.0, 10.0), "wheel_roll_m": (-1000.0, 1000.0)}

LIGHT_MATERIALS = {"UM_NavRed": "nav", "UM_NavGreen": "nav", "UM_StatusLED": "status", "UM_Strobe": "strobe"}
LIGHT_OBJECTS = {"U_Light_Nav_L": "nav", "U_Light_Nav_R": "nav", "U_Light_Turret_Ring": "status",
                 "U_Light_Strobe_L": "strobe", "U_Light_Strobe_R": "strobe", "U_Light_Landing": "nav"}
COLLECTION_COLORS = {"UCAV": "COLOR_04", "UCAV_Airframe": "COLOR_07", "UCAV_Surfaces": "COLOR_05",
                     "UCAV_Gear": "COLOR_02", "UCAV_Propulsion": "COLOR_01", "UCAV_Payload": "COLOR_06",
                     "UCAV_Details": "COLOR_03", "UCAV_Print": "COLOR_08", "UCAV_Studio": "NONE"}


# =====================================================================================================
# İfadeler (Python karşılıkları testlerde kullanılır)
# =====================================================================================================
def _f(x: float) -> str:
    return f"{x:.9g}"


def strobe_expr(var: str = "s") -> str:
    """Kareye bağlı çift çakış (basit ifade)."""
    terms = " or ".join(f"(fmod(frame, {STROBE_PERIOD}) >= {a} and fmod(frame, {STROBE_PERIOD}) < {b})"
                        for a, b in STROBE_FLASHES)
    return f"{var} * ({terms})"


def strobe_on(frame: float) -> float:
    m = math.fmod(frame, STROBE_PERIOD)
    return float(any(a <= m < b for a, b in STROBE_FLASHES))


def aileron_split(cmd: float) -> tuple[float, float]:
    """Kanatçık komutu → (sol, sağ) sapma (°, + = firar kenarı aşağı): yukarı giden tam, aşağı giden ×k
    (k = aşağı/yukarı sınır oranı), sonra mekanik sınırlar."""
    h = P.hinge_line("Aileron", "L")
    k = h.pos_max_deg / h.neg_max_deg
    a, b = 0.5 * (1 + k), 0.5 * (1 - k)

    def one(raw):
        return min(max(a * raw - b * abs(raw), -h.neg_max_deg), h.pos_max_deg)
    return one(cmd), one(-cmd)


def surface_expr(name: str, side: str) -> str:
    """Kumanda yüzeyi sürücü ifadesi (radyan). Değişkenler: a, f, e, r."""
    h = P.hinge_line(name, side)
    lo, hi = -h.neg_max_deg, h.pos_max_deg
    if name == "Aileron":
        k = h.pos_max_deg / h.neg_max_deg
        a, b = 0.5 * (1 + k), 0.5 * (1 - k)
        sgn = "" if side == "L" else "-"
        return f"radians(clamp({_f(a)} * {sgn}a - {_f(b)} * abs(a), {_f(lo)}, {_f(hi)}))"
    var = {"FlapIn": "f", "FlapOut": "f", "Elevator": "e", "Rudder": "r"}[name]
    return f"radians(clamp({var}, {_f(lo)}, {_f(hi)}))"


def gear_legs_expr(retract_deg: float) -> str:
    l0, l1 = _SEQ["legs"]
    return f"radians({_f(retract_deg)}) * (1 - smoothstep({_f(l0)}, {_f(l1)}, g))"


def gear_doors_expr(open_deg: float) -> str:
    a0, a1 = _SEQ["doors_open"]
    c0, c1 = _SEQ["doors_close"]
    return (f"radians({_f(open_deg)}) * max(smoothstep({_f(a0)}, {_f(a1)}, g) - smoothstep({_f(c0)}, {_f(c1)}, g), "
            f"clamp(d, 0, 1))")


def steer_expr() -> str:
    h = P.hinge_line("Rudder", "L")
    c0, c1 = _SEQ["doors_close"]
    k = STEER_MAX / h.pos_max_deg
    return (f"-radians(clamp(r, {_f(-h.neg_max_deg)}, {_f(h.pos_max_deg)}) * {_f(k)}) * "
            f"smoothstep({_f(c0)}, {_f(c1)}, g)")


def oleo_expr(leg: str) -> str:
    """Amortisör: ``z0 + clamp((gz − (z − R))/cos(yatıklık), 0, strok)``; ``z`` = aks referansı dünya Z,
    ``gz`` = ``U_Root["ground_z"]`` (zemin düzlemi)."""
    from . import gear as GR
    lg = GR.leg_geom(leg)
    c = math.cos(math.radians(lg.leg.rake_deg))
    return f"{_f(lg.z['slider_top'])} + clamp((gz - z + {_f(lg.R_w)}) / {_f(c)}, 0, {_f(lg.stroke)})"


def link_expr(leg: str, upper: bool) -> str:
    """Tork bağlantısı açısı; değişken ``L`` = kayar borunun ``location.z``'si."""
    from . import gear as GR
    lk = GR.leg_geom(leg).link
    z0 = GR.leg_geom(leg).z["slider_top"]
    alpha = f"acos(clamp(({_f(lk['D0'])} - (L - {_f(z0)})) / {_f(2 * lk['ell'])}, -1, 1))"
    a0 = _f(lk["alpha0"])
    return f"{a0} - {alpha}" if upper else f"{alpha} - {a0}"


def wheel_expr(leg: str) -> str:
    """Teker açısı: ``(roll + w·((x − X0)·cos h + y·sin h))/R`` — ``U_Root`` dünya konumunun baş yönüne (h = dünya
    Z açısı) izdüşümü; ``roll`` = ``wheel_roll_m``, ``w`` = ``wheel_auto``."""
    from . import gear as GR
    return (f"(roll + w * ((x - {_f(P.U_ROOT_B[0])}) * cos(h) + (y - {_f(P.U_ROOT_B[1])}) * sin(h))) / "
            f"{_f(GR.leg_geom(leg).R_w)}")


def prop_expr() -> str:
    """Pervane açısı (radyan): ``(k + m·kare·rpm/1440)·2π`` — ``k`` = ``U_Prop["ucav_turns"]``,
    ``m`` = ``prop_auto``."""
    return f"(k + m * frame * rpm / {_f(PROP_FPS * 60.0)}) * 2 * pi"


def prop_angle(frame: float, rpm: float, turns: float = 0.0, auto: float = 1.0) -> float:
    """``prop_expr`` Python karşılığı (radyan)."""
    return (turns + auto * frame * rpm / (PROP_FPS * 60.0)) * 2.0 * math.pi


def disc_expr() -> str:
    a, b, mx = DISC_RPM
    return f"clamp((rpm - {_f(a)}) / {_f(b)}, 0, 1) * {_f(mx)}"


# =====================================================================================================
# Sürücü kurulumu
# =====================================================================================================
def _var_prop(d, name: str, id_obj, path: str, id_type: str = "OBJECT") -> None:
    v = d.variables.new()
    v.name = name
    v.type = "SINGLE_PROP"
    v.targets[0].id_type = id_type
    v.targets[0].id = id_obj
    v.targets[0].data_path = path


def _var_transform(d, name: str, ob, ttype: str) -> None:
    v = d.variables.new()
    v.name = name
    v.type = "TRANSFORMS"
    v.targets[0].id = ob
    v.targets[0].transform_type = ttype
    v.targets[0].transform_space = "WORLD_SPACE"


def _driver(owner, path: str, index: int, expr: str):
    """Var olan sürücüyü kaldırıp yenisini kurar (tekrar çağrılabilir)."""
    try:
        owner.driver_remove(path, index)
    except TypeError:
        owner.driver_remove(path)
    fc = owner.driver_add(path, index) if index >= 0 else owner.driver_add(path)
    for m in list(fc.modifiers):
        fc.modifiers.remove(m)
    d = fc.driver
    d.type = "SCRIPTED"
    for v in list(d.variables):
        d.variables.remove(v)
    d.expression = expr
    return d


_ALIAS = {"g": "gear", "d": "gear_doors", "a": "aileron_deg", "f": "flap_deg", "e": "elevator_deg", "r": "rudder_deg",
          "rpm": "prop_rpm", "m": "prop_auto", "pan": "turret_pan_deg", "tilt": "turret_tilt_deg", "n": "nav_lights",
          "s": "strobe", "t": "status_led", "gz": "ground_z", "w": "wheel_auto", "roll": "wheel_roll_m"}


def _root_vars(d, root, names) -> None:
    for v in names:
        _var_prop(d, v, root, f'["{_ALIAS[v]}"]')


def _overridable(ob, key: str) -> None:
    """Özel özelliği kütüphane geçersiz kılmasına açar (link + Library Override ile anahtarlanabilsin)."""
    try:
        ob.property_overridable_library_set(f'["{key}"]', True)
    except (AttributeError, TypeError, ValueError):
        pass


def ensure_props(root: bpy.types.Object, reset: bool = False) -> None:
    """Sözleşmedeki özel özellikleri (float, min/maks, Türkçe açıklama) kurar; var olan değerler korunur.
    Hepsi kütüphane geçersiz kılmasına açıktır."""
    for name, default, lo, hi, desc in PROPS:
        if reset or name not in root.keys():
            root[name] = float(default)
        else:
            root[name] = float(min(max(float(root[name]), lo), hi))
        ui = root.id_properties_ui(name)
        slo, shi = _SOFT.get(name, (lo, hi))
        ui.update(min=lo, max=hi, soft_min=slo, soft_max=shi, default=float(default), description=desc,
                  step=100 if hi - lo > 2 else 10, precision=_PRECISION.get(name, 2 if hi - lo <= 2 else 1))
        _overridable(root, name)
    root["ucav_rig"] = "YK-38 kontrol paneli (rig.setup)"


def ensure_prop_turns(prop: bpy.types.Object | None) -> None:
    """``U_Prop["ucav_turns"]`` (pişirilmiş tur) kurulur; eski ``ucav_turns_offset`` kaldırılır."""
    if prop is None:
        return
    if "ucav_turns_offset" in prop.keys():
        del prop["ucav_turns_offset"]
    if PROP_TURNS not in prop.keys():
        prop[PROP_TURNS] = 0.0
    prop.id_properties_ui(PROP_TURNS).update(
        description="Pişirilmiş pervane turu (∫rpm/60·dt). U_Root['prop_auto'] = 0 iken açı = ucav_turns·2π; "
                    "'YK38_pervane_pisir.py' yazar. prop_auto = 1 iken yalnız faz ofsetidir")
    _overridable(prop, PROP_TURNS)


def drive_light_materials(scene: bpy.types.Scene | None = None) -> list[str]:
    """Işık malzemelerinin yayım şiddetini sürer (Principled "Emission Strength" / Emission "Strength");
    temel şiddet ``params.MATERIALS[...].emission``. Sürülen soketlerin listesini döndürür."""
    root = bpy.data.objects[ROOT]
    done = []
    var = {"nav": "n", "strobe": "s", "status": "t"}
    for mname, kind in LIGHT_MATERIALS.items():
        mat = bpy.data.materials.get(mname)
        if mat is None or not mat.use_nodes or mat.node_tree is None:
            continue
        base = float(P.MATERIALS[mname].emission) if mname in P.MATERIALS else 1.0
        nt = mat.node_tree
        for node in nt.nodes:
            sock = None
            if node.type == "BSDF_PRINCIPLED":
                sock = node.inputs.get("Emission Strength")
            elif node.type == "EMISSION":
                sock = node.inputs.get("Strength")
            if sock is None or sock.is_linked:
                continue
            path = sock.path_from_id("default_value")
            v = var[kind]
            expr = (f"{_f(base)} * " + strobe_expr("clamp(s, 0, 1)")) if kind == "strobe" else \
                f"{_f(base)} * clamp({v}, 0, 1)"
            d = _driver(nt, path, -1, expr)
            _root_vars(d, root, [v])
            done.append(f"{mname}:{node.name}")
    return done


def driver_table() -> list[dict]:
    """Kurulacak sürücülerin tablosu: nesne, kanal, indis, ifade, değişkenler (belgeleme ve test)."""
    rows = []
    for leg in ("N", "L", "R"):
        g = P.gear_leg(leg)
        rows.append({"obj": f"U_GearPivot_{leg}", "path": "rotation_euler", "index": 0,
                     "expr": gear_legs_expr(g.retract_deg), "vars": ["g"]})
        rows.append({"obj": f"U_GearSlider_{leg}", "path": "location", "index": 2, "expr": oleo_expr(leg),
                     "vars": ["gz", ("z", f"U_GearAxleRef_{leg}", "LOC_Z")]})
        for up, nm in ((True, "U"), (False, "L")):
            rows.append({"obj": f"U_GearLink{nm}_{leg}", "path": "rotation_euler", "index": 1,
                         "expr": link_expr(leg, up), "vars": [("L", f"U_GearSlider_{leg}", "location[2]")]})
        rows.append({"obj": f"U_GearWheel_{leg}", "path": "rotation_euler", "index": 1, "expr": wheel_expr(leg),
                     "vars": ["roll", "w", ("x", ROOT, "LOC_X"), ("y", ROOT, "LOC_Y"), ("h", ROOT, "ROT_Z")]})
    rows.append({"obj": "U_GearSteer_N", "path": "rotation_euler", "index": 2, "expr": steer_expr(),
                 "vars": ["r", "g"]})
    for dd in P.gear_doors():
        if dd.attach == "skin":
            rows.append({"obj": dd.name, "path": "rotation_euler", "index": 0, "expr": gear_doors_expr(dd.open_deg),
                         "vars": ["g", "d"]})
    for h in P.hinge_lines():
        var = {"Aileron": "a", "FlapIn": "f", "FlapOut": "f", "Elevator": "e", "Rudder": "r"}[h.name]
        rows.append({"obj": h.obj_name, "path": "rotation_euler", "index": 0, "expr": surface_expr(h.name, h.side),
                     "vars": [var]})
    rows.append({"obj": "U_Prop", "path": "rotation_euler", "index": 0, "expr": prop_expr(),
                 "vars": ["rpm", "m", ("k", "U_Prop", f'["{PROP_TURNS}"]')]})
    rows.append({"obj": "U_Turret_Pan", "path": "rotation_euler", "index": 2, "expr": "radians(clamp(pan, -180, 180))",
                 "vars": ["pan"]})
    rows.append({"obj": "U_Turret_Tilt", "path": "rotation_euler", "index": 1, "expr": "-radians(clamp(tilt, -90, 20))",
                 "vars": ["tilt"]})
    for ob, kind in LIGHT_OBJECTS.items():
        v = {"nav": "n", "strobe": "s", "status": "t"}[kind]
        rows.append({"obj": ob, "path": '["ucav_emission"]', "index": -1,
                     "expr": strobe_expr("clamp(s, 0, 1)") if kind == "strobe" else f"clamp({v}, 0, 1)",
                     "vars": [v]})
    if bpy.data.objects.get(DISC_NAME) is not None:
        rows.append({"obj": DISC_NAME, "path": '["ucav_disc"]', "index": -1, "expr": disc_expr(), "vars": ["rpm"]})
        rows.append({"obj": DISC_NAME, "path": "hide_render", "index": -1, "expr": f"rpm < {_f(DISC_RPM[0])}",
                     "vars": ["rpm"]})
    for name in LINKAGES:                    # servo kolu yüzeyle aynı açı; çubuk ters dönüşle yalnız öteler
        var = {"Aileron": "a", "FlapIn": "f", "FlapOut": "f", "Elevator": "e", "Rudder": "r"}[name]
        for side in ("L", "R"):
            _, n_arm, n_rod = linkage_names(name, side)
            if bpy.data.objects.get(n_arm) is None or bpy.data.objects.get(n_rod) is None:
                continue
            ex = surface_expr(name, side)
            rows.append({"obj": n_arm, "path": "rotation_euler", "index": 0, "expr": ex, "vars": [var]})
            rows.append({"obj": n_rod, "path": "rotation_euler", "index": 0, "expr": f"-({ex})", "vars": [var]})
    return rows


# =====================================================================================================
# Pervane diski (isteğe bağlı: malzemesi ``UM_PropDisc`` spec'te tanımlıysa)
# =====================================================================================================
DISC_NAME = "U_PropDisc"
DISC_MAT = "UM_PropDisc"


def ensure_prop_disc() -> bpy.types.Object | None:
    """Pervane düzleminde ince kapalı halka (r 0,035 … pala ucu, 96 dilim, 1 mm): hızlı dönen pervanenin zaman
    ortalamalı diski. Orijin göbekte, yerel X = mil ekseni (geriye; ``U_Prop`` ile aynı çerçeve) → malzeme yarıçapı
    Object koordinatından √(y² + z²) diye okur. ``U_Root``'a bağlıdır (pervaneyle dönmez), seçilemez, gölge vermez.
    Malzeme ``UM_PropDisc`` ``params.MATERIALS``'ta yoksa kurulmaz (``None``)."""
    if DISC_MAT not in P.MATERIALS:
        return None
    from . import util as U
    import numpy as np
    from mathutils import Matrix
    root = bpy.data.objects[ROOT]
    pr = P.PROP
    hub = np.asarray(P.to_blender(*pr.hub), float)
    ax = np.asarray(pr.axis_aft_b, float)
    ax /= np.linalg.norm(ax)
    e1 = np.cross(ax, [0.0, 0.0, 1.0])
    e1 /= np.linalg.norm(e1)
    e2 = np.cross(ax, e1)
    Fr = np.column_stack([ax, e1, e2])                       # yerel X = mil, Y/Z = disk düzlemi
    r0, r1, th, n = 0.035, float(pr.radius), 0.001, 96
    V, F = [], []
    for k in range(n):
        a = 2 * math.pi * k / n
        d = np.array([0.0, math.cos(a), math.sin(a)])
        for r in (r0, r1):
            for s in (-0.5, 0.5):
                V.append(tuple(d * r + np.array([s * th, 0.0, 0.0])))
    for k in range(n):
        a, b = 4 * k, 4 * ((k + 1) % n)
        # dörtlü: (iç-ön, iç-arka, dış-ön, dış-arka) = (0, 1, 2, 3); yüzler dışa dönük (ön −mil, arka +mil)
        F += [(a + 0, b + 0, b + 2, a + 2), (a + 1, a + 3, b + 3, b + 1),
              (a + 2, b + 2, b + 3, a + 3), (a + 0, a + 1, b + 1, b + 0)]
    old = bpy.data.objects.get(DISC_NAME)
    if old is not None:
        bpy.data.objects.remove(old, do_unlink=True)
    me = bpy.data.meshes.get(DISC_NAME)
    if me is not None:
        bpy.data.meshes.remove(me)
    me = bpy.data.meshes.new(DISC_NAME)
    me.from_pydata(V, [], F)
    me.validate(verbose=False)
    me.update()
    me.materials.append(U.get_material(DISC_MAT))
    ob = bpy.data.objects.new(DISC_NAME, me)
    col = bpy.data.collections.get("UCAV_Propulsion") or bpy.data.collections.get("UCAV")
    col.objects.link(ob)
    ob.parent = root
    ob.matrix_parent_inverse.identity()
    ob.location = tuple(map(float, hub - np.asarray(P.U_ROOT_B, float)))
    ob.rotation_mode = "XYZ"
    ob.delta_rotation_euler = Matrix(Fr.tolist()).to_euler("XYZ")
    ob.hide_select = True
    ob.visible_shadow = False
    ob["ucav_disc"] = 0.0
    ob.id_properties_ui("ucav_disc").update(min=0.0, max=1.0, description="Pervane diski örtmesi (rig sürer)")
    return ob


# =====================================================================================================
# Kumanda bağlantıları: boynuz, servo kolu, itme çubuğu (GEO-01)
# =====================================================================================================
LINKAGES = ("Aileron", "FlapOut", "Elevator", "Rudder")    # iç flap: servo yeri baskı planında (P9) açık
LINK_COL = "UCAV_Surfaces"
LINK_T = 0.0016                     # boynuz ve servo kolu plaka kalınlığı (G10 / naylon). varsayım
HORN_OUT = {"wing": 0.011, "stab": 0.009, "fin": 0.009}     # boynuz deliği, yüzey derisinin dışında (m). varsayım
ARM_OUT = 0.009                     # servo kolu ucu, servo kapağının (ev sahibi deri) en az bu kadar dışında. varsayım
ROD_R = 0.0008                      # Ø1,6 mm çelik itme çubuğu
SERVO_X_C = {"wing": 0.465, "stab": 0.36, "fin": 0.40}      # servo (servis kapağı) merkezi, veter oranı. varsayım
_SERVO_Y_DEFAULT = {"Aileron": 1.30, "FlapOut": 0.68, "Elevator": 0.17, "Rudder": 0.10, "FlapIn": 0.20}
_KIND = {"Aileron": "wing", "FlapOut": "wing", "FlapIn": "wing", "Elevator": "stab", "Rudder": "fin"}


def _servo_stations() -> dict[str, float]:
    """Servo açıklık istasyonları (kanat/stabilize ``|y|``, dikey ``h``): baskı planı (``printprep.SERVO_STATIONS``)
    ile aynı — servis kapakları (``materials``) da buradan çizilir."""
    try:
        from .printprep import SERVO_STATIONS
        return {**_SERVO_Y_DEFAULT, **{k: float(v) for k, v in SERVO_STATIONS.items()}}
    except Exception:                                       # pragma: no cover — baskı modülü yoksa
        return dict(_SERVO_Y_DEFAULT)


def _servo_point_b(name: str, side: str):
    """Servo çıkış mili noktası (Blender, dinlenme): servis kapağı merkezi, kalınlık ortası."""
    import numpy as np
    st_v = _servo_stations()[name]
    sg = 1.0 if side == "L" else -1.0
    kind = _KIND[name]
    if kind == "wing":
        y = sg * st_v
        st = P.wing_station(y)
        try:
            from .materials import SERVO_HATCH_X_C as xc
        except Exception:                                   # pragma: no cover
            xc = SERVO_X_C["wing"]
        s = st.le_s + float(xc) * (st.te_s - st.le_s)
        z = P.wing_mid_z(s, y)
        return np.asarray(P.to_blender(s, y, st.z_ref if z is None else z), float)
    if kind == "stab":
        st = P.stab_station(sg * st_v)
        return np.asarray(P.to_blender(st.le_s + SERVO_X_C["stab"] * st.chord, sg * st_v, st.z), float)
    fs = P.fin_station(st_v, side)
    return np.asarray(P.to_blender(fs.le[0] + SERVO_X_C["fin"] * fs.chord, fs.le[1], fs.le[2]), float)


def _rest_matrix(ob):
    """Dinlenme pozu dünya matrisi: sürülen ``rotation_euler`` yok sayılır, ``U_Root`` tasarım CG'sinde."""
    from mathutils import Matrix, Vector
    if ob.name == ROOT:
        return Matrix.Translation(Vector(P.U_ROOT_B))
    R = ob.delta_rotation_euler.to_matrix().to_4x4()
    S = Matrix.Diagonal((*[a * b for a, b in zip(ob.scale, ob.delta_scale)], 1.0))
    B = Matrix.Translation(ob.location + ob.delta_location) @ R @ S
    return _rest_matrix(ob.parent) @ ob.matrix_parent_inverse @ B if ob.parent is not None else B


def _skin_w(targets, M_f, x: float, u: float, s_z: float, far: float = 0.3) -> float | None:
    """``M_f`` çerçevesinde (x, u) doğrusunda ``s_z``·Z yönündeki en dış deri (dışarıdan içe ışın): w = s_z·z."""
    from mathutils import Vector
    Mfi = M_f.inverted()
    p_w = M_f @ Vector((x, u, s_z * far))
    d_w = (M_f.to_3x3() @ Vector((0.0, 0.0, -s_z))).normalized()
    best = None
    for bvh, M in targets:
        Mi = M.inverted()
        hit = bvh.ray_cast(Mi @ p_w, (Mi.to_3x3() @ d_w).normalized(), 2.0 * far)[0]
        if hit is None:
            continue
        w = s_z * (Mfi @ (M @ hit)).z
        best = w if best is None else max(best, w)
    return best


def _convex_hull(pts):
    """2B dışbükey zarf (Andrew), saat yönü tersine."""
    pts = sorted(set((round(float(a), 7), round(float(b), 7)) for a, b in pts))

    def cross(o, a, b):
        return (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0])
    lo, hi = [], []
    for p in pts:
        while len(lo) >= 2 and cross(lo[-2], lo[-1], p) <= 0:
            lo.pop()
        lo.append(p)
    for p in reversed(pts):
        while len(hi) >= 2 and cross(hi[-2], hi[-1], p) <= 0:
            hi.pop()
        hi.append(p)
    return lo[:-1] + hi[:-1]


def linkage_names(name: str, side: str) -> tuple[str, str, str]:
    return f"U_Horn_{name}_{side}", f"U_ServoArm_{name}_{side}", f"U_Pushrod_{name}_{side}"


def linkage_geometry(name: str, side: str) -> dict | None:
    """Paralelkenar bağlantı ölçüleri (kumanda yüzeyi çerçevesinde, orijin menteşe ortası, X menteşe, Y geriye):
    boynuz deliği H = (x, 0, s·d), servo mili S = (x, −L, 0), servo kolu = boynuz vektörü (0, 0, s·d) → çubuk
    dönmeden öteler (kol ve yüzey aynı açıyla döner). ``s`` = dışa yön (+1 kanat/stabilize altı, dikeyde içe)."""
    import numpy as np
    from mathutils import Matrix, Vector
    from mathutils.bvhtree import BVHTree
    h = P.hinge_line(name, side)
    surf = bpy.data.objects.get(h.obj_name)
    if surf is None or surf.parent is None:
        return None
    kind = _KIND[name]
    F = np.asarray(h.frame_b(), float)
    O = np.asarray(h.mid_b, float)
    M_f = Matrix.Translation(Vector(O)) @ Matrix([list(F[i]) for i in range(3)]).to_4x4()
    S_w = _servo_point_b(name, side)
    x = float(np.dot(S_w - O, F[:, 0]))
    x = float(np.clip(x, -0.5 * h.length + 0.02, 0.5 * h.length - 0.02))
    L = float(-np.dot(S_w - O, F[:, 1]))
    if kind == "fin":                                        # dikey: boynuz ve servo kolu içe (pervane tarafı)
        fin = np.asarray(P.to_blender(*P.fin_station(0.1, side).le), float)
        s_z = 1.0 if float(np.dot(F[:, 2], [0.0, -np.sign(fin[1]), 0.0])) > 0 else -1.0
    else:                                                    # yatay yüzeyler: aşağı
        s_z = 1.0 if F[2, 2] < 0 else -1.0
    dg = bpy.context.evaluated_depsgraph_get()
    tg_s = [(BVHTree.FromObject(surf, dg), _rest_matrix(surf))]
    host = surf.parent
    hosts = [host] + [bpy.data.objects.get(n.format(s=side)) for n in
                      (("U_WingCenter_{s}", "U_WingOuter_{s}", "U_Tip_{s}") if kind == "wing" else ())]
    tg_h = [(BVHTree.FromObject(o, dg), _rest_matrix(o)) for o in dict.fromkeys(o for o in hosts if o is not None)]
    w_hinge = _skin_w(tg_s, M_f, x, 0.004, s_z)
    w_servo = _skin_w(tg_h, M_f, x, -L, s_z)
    if w_hinge is None or w_servo is None:
        return None
    d = max(w_hinge + HORN_OUT[kind], w_servo + ARM_OUT)
    base = [(u, (_skin_w(tg_s, M_f, x, u, s_z) or w_hinge) - 0.0012) for u in (0.0025, 0.011, 0.020)]
    return {"name": name, "side": side, "kind": kind, "x": x, "L": L, "d": d, "s": s_z, "frame": F, "origin": O,
            "M_f": M_f, "host": host.name, "surface": surf.name, "w_hinge": w_hinge, "w_servo": w_servo,
            "horn_base": base}


def _link_mesh_parts(g: dict):
    """(boynuz, servo kolu, çubuk) ağları — her biri kendi nesne orijinine göre, yüzey çerçevesi eksenlerinde."""
    import numpy as np
    from . import gear as GR
    d, L, s = g["d"], g["L"], g["s"]
    ey, ez = np.array([0.0, 1.0, 0.0]), np.array([0.0, 0.0, s])
    t2 = 0.5 * LINK_T
    # boynuz: deri tabanı (1,2 mm gömülü) + delik çevresinde Ø5,2 mm uç → dışbükey levha; orijin delik
    pts = [(u, w - d) for u, w in g["horn_base"]]
    pts += [(0.0026 * math.cos(a), 0.0026 * math.sin(a)) for a in np.linspace(0.0, 2 * math.pi, 16, endpoint=False)]
    horn = GR.prism("horn", _convex_hull(pts), np.zeros(3), ey, ez, -t2, t2, "UM_Accent", 40.0)
    # servo kolu: mil göbeği (r 3 mm) → uç (r 2,1 mm), orijin mil
    arm = GR.merge("arm", [GR.prism("arm", GR.stadium((0.0, 0.0), (0.0, d), 0.0030, 0.0021, 8), np.zeros(3), ey,
                                    ez, -t2 - 0.0002, t2 + 0.0002, "UM_Carbon", 40.0),
                           GR.cyl("spline", (-0.0030, 0.0, 0.0), (0.0030, 0.0, 0.0), 0.0019, "UM_Steel", 16)], 40.0)
    # çubuk: Ø1,6 çelik + iki uçta çatal (klevis); orijin servo kolu ucu, çubuk +Y (geriye) boyunca L
    rod = GR.merge("rod", [GR.cyl("rod", (0.0, 0.0028, 0.0), (0.0, L - 0.0028, 0.0), ROD_R, "UM_Steel", 10),
                           GR.box("clevis_a", (0.0, 0.0, 0.0), (0.0018, 0.0030, 0.0019), None, "UM_Accent", 0.0006),
                           GR.box("clevis_b", (0.0, L, 0.0), (0.0018, 0.0030, 0.0019), None, "UM_Accent", 0.0006)],
                   40.0)
    return horn, arm, rod


def _link_object(name: str, md, col, parent, location, delta_euler=(0.0, 0.0, 0.0)):
    from mathutils import Matrix, Vector
    from . import util as U
    U.remove_object(name)
    md.name = name
    me = U.mesh_from_data(md, name)
    ob = bpy.data.objects.new(name, me)
    col.objects.link(ob)
    ob.parent = parent
    ob.matrix_parent_inverse = Matrix.Identity(4)
    ob.location = Vector(tuple(map(float, location)))
    ob.rotation_mode = "XYZ"
    ob.rotation_euler = (0.0, 0.0, 0.0)
    ob.delta_rotation_euler = tuple(map(float, delta_euler))
    ob["ucav_role"] = "linkage"
    return ob


def ensure_linkages() -> list[dict]:
    """Her kumanda yüzeyi (``LINKAGES``, iki yan) için boynuz (yüzeye bağlı), servo kolu (ev sahibi deriye bağlı,
    yüzeyle aynı ifadeyle döner) ve itme çubuğu (servo koluna bağlı, ters dönüşle öteler) kurar. Tekrar
    çağrılabilir. Ölçüleri döndürür."""
    from mathutils import Matrix, Vector
    col = bpy.data.collections.get(LINK_COL) or bpy.data.collections.get("UCAV")
    out = []
    for name in LINKAGES:
        for side in ("L", "R"):
            g = linkage_geometry(name, side)
            if g is None:
                continue
            horn_md, arm_md, rod_md = _link_mesh_parts(g)
            n_horn, n_arm, n_rod = linkage_names(name, side)
            surf, host = bpy.data.objects[g["surface"]], bpy.data.objects[g["host"]]
            _link_object(n_horn, horn_md, col, surf, (g["x"], 0.0, g["s"] * g["d"]))
            Mb = _rest_matrix(host).inverted() @ g["M_f"] @ Matrix.Translation(Vector((g["x"], -g["L"], 0.0)))
            arm = _link_object(n_arm, arm_md, col, host, Mb.to_translation(), Mb.to_3x3().to_euler("XYZ"))
            _link_object(n_rod, rod_md, col, arm, (0.0, 0.0, g["s"] * g["d"]))
            out.append({k: (round(v, 4) if isinstance(v, float) else v) for k, v in g.items()
                        if k in ("name", "side", "x", "L", "d", "w_hinge", "w_servo", "host")})
    return out


def _action_fcurves(action) -> list:
    """Aksiyonun F-eğrileri (eski API ya da 4.4+ katmanlı aksiyon)."""
    if len(getattr(action, "fcurves", ())):
        return list(action.fcurves)
    out = []
    for layer in getattr(action, "layers", ()):
        for strip in layer.strips:
            for bag in getattr(strip, "channelbags", ()):
                out.extend(bag.fcurves)
    return out


def normalize_parenting(objects=None) -> list[str]:
    """Ebeveyn ters matrisini birim yapar (tek kural: yerel konum + ``delta_rotation_euler``), dünya dönüşümü
    korunur: B' = Pinv·B → konum = R·(konum + delta) + t − delta, delta dönüş = R·delta dönüş. Anahtarlı, konumu
    sürülen ya da ölçekli ebeveyn tersine sahip nesneler atlanır. Varsayılan: ``UCAV`` koleksiyon ağacı."""
    from mathutils import Matrix
    if objects is None:
        col = bpy.data.collections.get("UCAV")
        objects = list(col.all_objects) if col is not None else []
    I4 = Matrix.Identity(4)
    done = []
    for ob in objects:
        Pm = ob.matrix_parent_inverse
        if ob.parent is None or all(abs(Pm[i][j] - I4[i][j]) < 1e-9 for i in range(4) for j in range(4)):
            continue
        ad = ob.animation_data
        moving = ("location", "delta_location", "delta_rotation_euler")
        if ad is not None and (any(d.data_path in moving for d in ad.drivers) or (
                ad.action is not None and any(fc.data_path in moving for fc in _action_fcurves(ad.action)))):
            continue
        if ob.rotation_mode not in ("XYZ", "XZY", "YXZ", "YZX", "ZXY", "ZYX"):
            continue
        R3 = Pm.to_3x3()
        RRt = R3 @ R3.transposed()
        if max(abs(RRt[i][j] - (1.0 if i == j else 0.0)) for i in range(3) for j in range(3)) > 1e-6:
            continue                                         # ölçekli ebeveyn tersi: dokunma
        dloc = ob.delta_location.copy()
        loc = R3 @ (ob.location + dloc) + Pm.to_translation() - dloc
        drot = (R3 @ ob.delta_rotation_euler.to_matrix()).to_euler(ob.rotation_mode, ob.delta_rotation_euler)
        ob.matrix_parent_inverse = I4.copy()
        ob.location = loc
        ob.delta_rotation_euler = drot
        done.append(ob.name)
    if done:
        bpy.context.view_layer.update()
    return done


# =====================================================================================================
# Görünüm penceresi kolaylıkları, varlık, metin bloğu
# =====================================================================================================
def viewport_setup(driven: set[str]) -> None:
    """``U_Root`` büyük, önde çizilen, adı görünen ok; sürülen nesnelerde konum/dönüş/ölçek kilidi (yalnız
    etkileşimli dönüşümü kilitler; sürücüler ve Python etkilenmez); koleksiyon renk etiketleri."""
    root = bpy.data.objects[ROOT]
    root.empty_display_type = "ARROWS"
    root.empty_display_size = 1.2
    root.show_in_front = True
    root.show_name = True
    for name in sorted(driven):
        ob = bpy.data.objects.get(name)
        if ob is None or ob is root:
            continue
        ob.lock_location = (True, True, True)
        ob.lock_rotation = (True, True, True)
        ob.lock_scale = (True, True, True)
    for name, tag in COLLECTION_COLORS.items():
        col = bpy.data.collections.get(name)
        if col is not None and hasattr(col, "color_tag"):
            col.color_tag = tag


def mark_asset() -> None:
    """``UCAV`` koleksiyonunu varlık (asset) olarak işaretler (Asset Browser'dan sürüklenebilir)."""
    col = bpy.data.collections.get("UCAV")
    if col is None or not hasattr(col, "asset_mark"):
        return
    try:
        if col.asset_data is None:
            col.asset_mark()
        ad = col.asset_data
        ad.description = ("YELKOVAN YK-38 — sivil EO/IR gözetleme İHA'sı (3,80 m). Kontrol paneli U_Root özel "
                          "özelliklerinde; zemin yüksekliği U_Root['ground_z']")
        ad.author = "ucav/ (YK-38 depo)"
        for tag in ("YK-38", "İHA", "uçak", "rig"):
            if tag not in ad.tags:
                ad.tags.new(tag)
    except (AttributeError, RuntimeError, TypeError):
        pass


_BAKE_SCRIPT = '''# YELKOVAN YK-38 — pervane açısını pişir (depo gerekmez; Text Editor'da ▶ Run Script)
#
# Neden: U_Root["prop_auto"] = 1 iken pervane açısı = kare·prop_rpm/1440 turdur. Devir sabitken bu tamdır; ama
# prop_rpm'e farklı değerli anahtar kareler koyarsanız açısal hız rpm + kare·d(rpm)/dt olur ve pervane geri
# dönüyormuş gibi görünür. Bu betik açıyı gerçek integral olarak pişirir:
#   U_Prop["ucav_turns"](kare) = ∫ rpm/60 dt   (kare başına 4 alt örnek, trapez; her kareye LINEAR anahtar)
#   U_Root["prop_auto"] = 0                     (açı = ucav_turns·2π)
# prop_rpm eğrisini her düzenleyişinizden sonra yeniden çalıştırın. Geri almak için: U_Prop'taki ucav_turns
# eğrisini silin ve U_Root["prop_auto"] = 1 yapın. Hareket bulanıklığı alt kareleri doğrusal aradeğerlenir.
import bpy

ALT_ORNEK = 4                     # kare başına alt örnek

sc = bpy.context.scene
root = bpy.data.objects["U_Root"]
prop = bpy.data.objects["U_Prop"]
fps = sc.render.fps / sc.render.fps_base


def fcurve(ob, path):
    ad = ob.animation_data
    act = ad.action if ad is not None else None
    return act.fcurves.find(path) if act is not None else None


fc_rpm = fcurve(root, '["prop_rpm"]')


def rpm(f):
    return max(0.0, fc_rpm.evaluate(f)) if fc_rpm is not None else max(0.0, float(root["prop_rpm"]))


f0, f1 = sc.frame_start, sc.frame_end
frames = list(range(f0, f1 + 2))
turns = [0.0]
for f in frames[:-1]:
    h = 1.0 / ALT_ORNEK
    s = 0.0
    for k in range(ALT_ORNEK):
        a = f + k * h
        s += 0.5 * (rpm(a) + rpm(a + h)) * h
    turns.append(turns[-1] + s / 60.0 / fps)
if prop.animation_data is None:
    prop.animation_data_create()
act = prop.animation_data.action
if act is None:
    act = bpy.data.actions.new("YK38_el_Prop")
    act.use_fake_user = True
    prop.animation_data.action = act
old = act.fcurves.find('["ucav_turns"]')
if old is not None:
    act.fcurves.remove(old)
fc = act.fcurves.new('["ucav_turns"]')
fc.keyframe_points.add(len(frames))
co = []
for f, v in zip(frames, turns):
    co += [float(f), float(v)]
fc.keyframe_points.foreach_set("co", co)
fc.keyframe_points.foreach_set("interpolation", [1] * len(frames))     # LINEAR
fc.extrapolation = "LINEAR"
fc.update()
ad = prop.animation_data
if getattr(ad, "action_slot", True) is None:      # Blender 4.4+: yeni aksiyonun yuvası atanınca etkin olur
    ad.action = None
    ad.action = act
ra = root.animation_data.action if root.animation_data is not None else None
old = ra.fcurves.find('["prop_auto"]') if ra is not None else None
if old is not None:
    ra.fcurves.remove(old)
root["prop_auto"] = 0.0
root.update_tag()
sc.frame_set(sc.frame_current)
print("YK-38 pervane pişirildi: kare %d–%d, %.1f tur" % (f0, f1 + 1, turns[-1]))
'''


def embed_bake_text() -> None:
    """``.blend`` içine depo gerektirmeyen pervane pişirme metin bloğu (``YK38_pervane_pisir.py``)."""
    txt = bpy.data.texts.get(BAKE_TEXT) or bpy.data.texts.new(BAKE_TEXT)
    txt.from_string(_BAKE_SCRIPT)


def setup(scene: bpy.types.Scene | None = None, *, reset: bool = False) -> dict:
    """Özel özellikleri ve bütün sürücüleri kurar (``airframe.build`` + ``gear.build`` sonrası). Tekrar
    çağrılabilir. ``reset`` True ise özellikler varsayılana döner. Kurulan sürücü sayısı ve eksik nesneleri döndürür."""
    scene = scene or bpy.context.scene
    root = bpy.data.objects.get(ROOT)
    if root is None:
        raise RuntimeError("U_Root yok: önce airframe.build() ve gear.build()")
    ensure_props(root, reset)
    ensure_prop_turns(bpy.data.objects.get("U_Prop"))
    ensure_prop_disc()
    normalized = normalize_parenting()
    links = ensure_linkages()
    n_ok, missing, not_simple = 0, [], []
    driven: set[str] = set()
    for row in driver_table():
        ob = bpy.data.objects.get(row["obj"])
        if ob is None:
            missing.append(row["obj"])
            continue
        if row["path"].startswith('["'):
            key = row["path"][2:-2]
            if key not in ob.keys():
                ob[key] = 1.0
                ob.id_properties_ui(key).update(min=0.0, max=1.0, description="Işık şiddeti çarpanı (rig sürer)")
            _overridable(ob, key)
            d = _driver(ob, row["path"], -1, row["expr"])
        elif row["index"] < 0:
            d = _driver(ob, row["path"], -1, row["expr"])
        else:
            d = _driver(ob, row["path"], row["index"], row["expr"])
        for v in row["vars"]:
            if isinstance(v, str):
                _root_vars(d, root, [v])
            elif v[2].startswith(("LOC_", "ROT_")):
                _var_transform(d, v[0], bpy.data.objects[v[1]], v[2])
            else:
                _var_prop(d, v[0], bpy.data.objects[v[1]], v[2])
        if not d.is_simple_expression:
            not_simple.append(f"{row['obj']}.{row['path']}[{row['index']}]")
        driven.add(row["obj"])
        n_ok += 1
    mats = drive_light_materials(scene)
    viewport_setup(driven | {linkage_names(g["name"], g["side"])[0] for g in links})
    mark_asset()
    embed_bake_text()
    refresh()
    return {"drivers": n_ok, "material_drivers": mats, "missing": missing, "not_simple": not_simple,
            "linkages": links, "normalized": len(normalized)}


def set_controls(**values) -> None:
    """Kolaylık: ``set_controls(gear=0.5, aileron_deg=10)`` → ``U_Root`` özelliklerini sınırlar içinde yazar."""
    root = bpy.data.objects[ROOT]
    lim = {p[0]: (p[2], p[3]) for p in PROPS}
    for k, v in values.items():
        if k not in lim:
            raise KeyError(k)
        root[k] = float(min(max(v, lim[k][0]), lim[k][1]))
    refresh()


def refresh() -> None:
    """Python'dan özellik yazıldıktan sonra sürücüleri yeniden değerlendirir (``U_Root`` etiketlenir)."""
    root = bpy.data.objects.get(ROOT)
    if root is not None:
        root.update_tag()
    bpy.context.view_layer.update()
