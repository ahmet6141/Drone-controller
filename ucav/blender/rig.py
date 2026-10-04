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

Görünüm penceresi kolaylıkları: ``U_Root`` büyük, önde çizilen ve adı görünen bir okla seçilir; sürülen parçaların
konum/dönüş/ölçeği kilitlidir (G/R/S ile menteşeden kaçmaz); koleksiyonlar renk etiketlidir; ``UCAV`` koleksiyonu
varlık (asset) olarak işaretlidir (Asset Browser'dan sürüklenebilir).

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
    ("flap_deg", 0.0, *_rng("flap", (0.0, 35.0)), "Flap (°): iç ve dış flaplar birlikte, + = firar kenarı aşağı. Mekanik sınır 30°"),
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
    """Pervane açısı (radyan): ``(k + m·kare·rpm/1440)·2π`` — ``k`` = ``U_Prop["ucav_turns"]``, ``m`` = ``prop_auto``."""
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
    rows.append({"obj": "U_GearSteer_N", "path": "rotation_euler", "index": 2, "expr": steer_expr(), "vars": ["r", "g"]})
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
    return rows


# =====================================================================================================
# Pervane diski (isteğe bağlı: malzemesi ``UM_PropDisc`` spec'te tanımlıysa)
# =====================================================================================================
DISC_NAME = "U_PropDisc"
DISC_MAT = "UM_PropDisc"


def ensure_prop_disc() -> bpy.types.Object | None:
    """Pervane düzleminde ince kapalı halka (r 0,035 … pala ucu, 96 dilim, 1 mm): hızlı dönen pervanenin zaman
    ortalamalı diski. ``U_Root``'a bağlıdır (pervaneyle dönmez), seçilemez, gölge vermez. Malzeme ``UM_PropDisc``
    ``params.MATERIALS``'ta yoksa kurulmaz (``None``)."""
    if DISC_MAT not in P.MATERIALS:
        return None
    from . import util as U
    import numpy as np
    root = bpy.data.objects[ROOT]
    pr = P.PROP
    hub = np.asarray(P.to_blender(*pr.hub), float)
    ax = np.asarray(pr.axis_aft_b, float)
    ax /= np.linalg.norm(ax)
    e1 = np.cross(ax, [0.0, 0.0, 1.0])
    e1 /= np.linalg.norm(e1)
    e2 = np.cross(ax, e1)
    r0, r1, th, n = 0.035, float(pr.radius), 0.001, 96
    V, F = [], []
    for k in range(n):
        a = 2 * math.pi * k / n
        d = math.cos(a) * e1 + math.sin(a) * e2
        for r in (r0, r1):
            for s in (-0.5, 0.5):
                V.append(tuple(hub - np.asarray(P.U_ROOT_B) + d * r + ax * s * th))
    for k in range(n):
        a, b = 4 * k, 4 * ((k + 1) % n)
        # dörtlü: (iç-ön, iç-arka, dış-ön, dış-arka) = (0, 1, 2, 3)
        F += [(a + 0, a + 2, b + 2, b + 0), (a + 1, b + 1, b + 3, a + 3),
              (a + 2, a + 3, b + 3, b + 2), (a + 0, b + 0, b + 1, a + 1)]
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
    ob.location = (0.0, 0.0, 0.0)
    ob.hide_select = True
    ob.visible_shadow = False
    ob["ucav_disc"] = 0.0
    ob.id_properties_ui("ucav_disc").update(min=0.0, max=1.0, description="Pervane diski örtmesi (rig sürer)")
    return ob


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
    viewport_setup(driven)
    mark_asset()
    embed_bake_text()
    refresh()
    return {"drivers": n_ok, "material_drivers": mats, "missing": missing, "not_simple": not_simple}


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
