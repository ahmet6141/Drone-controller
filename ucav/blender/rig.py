"""YELKOVAN YK-38 — kontrol paneli ve sürücüler (bpy 4.5): ``setup(scene=None)``.

``U_Root`` üzerinde sözleşmedeki özel özellikler (Türkçe açıklama, min/maks) kurulur; hareketli bütün parçalar
bunlara Blender **basit ifade** sürücüleriyle bağlanır (Python betikleri kapalıyken de çalışır). Animasyon yalnız
bu özelliklere ve ``U_Root`` dönüşümüne anahtar kare koyar.

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
``prop_rpm``       0…9000      pervane devri; açı = kare/fps·rpm/60·2π (+ ``U_Prop["ucav_turns_offset"]``·2π)
``turret_pan_deg`` −180…180    + = iskeleye bakar
``turret_tilt_deg`` −90…20     + = yukarı, −90 = nadir
``nav_lights``     0/1         seyrüsefer (kırmızı/yeşil) ve taret durum halkası
``strobe``         0/1         beyaz çakarlar: 30 karede bir çift çakış (kare 0–1 ve 5–6)
=================  ==========  ===================================================================================

Sürülen kanallar (``driver_table()`` tam listeyi verir)
------------------------------------------------------
* ``U_GearPivot_N/L/R``.rotation_euler.x = radians(retract)·(1 − smoothstep(0,15; 0,85; gear))
* Deri kapakları ``U_Door_N_1/2``, ``U_Door_L/R_1``.rotation_euler.x = radians(open)·max(sıra, gear_doors)
* ``U_GearSlider_X``.location.z = z0 + clamp((zemin − (aks_z − R))/cos(yatıklık), 0, strok) — yer teması
  (``U_GearAxleRef_X`` dünya Z'si; zemin = ``params.GROUND_Z``); havada amortisör uzar, yerde statik çöker.
* ``U_GearLinkU/L_X``.rotation_euler.y — makas açısı (acos), amortisörle birlikte.
* ``U_GearWheel_X``.rotation_euler.y = (U_Root dünya X − X0)/R — kaymadan yuvarlanma.
* ``U_GearSteer_N``.rotation_euler.z = −radians(rudder·30/22)·smoothstep(0,85; 1; gear).
* Kumanda yüzeyleri ``rotation_euler.x`` (yerel X = ``HingeLine.axis_positive_b``: + = firar kenarı aşağı/sancağa).
* ``U_Prop``.rotation_euler.x, ``U_Turret_Pan``.rotation_euler.z, ``U_Turret_Tilt``.rotation_euler.y = −tilt.
* Işıklar: ışık nesnelerinde ``ucav_emission`` (0…1) sürülür; ``materials`` modülünün ışık malzemeleri yayım
  şiddetini bu nesne özelliğiyle (Attribute düğümü, Object) çarpar — iniş lambası ``UM_Strobe``'u paylaşsa da
  yanıp sönmez. ``drive_light_materials()`` yalnız bağlı olmayan Principled "Emission Strength" / Emission
  "Strength" soketlerini sürer (``materials`` kurulduysa bilerek boş döner; yer tutucu malzemeler için yedek yol).

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
     "Pervane devri (dev/dk). Açı = kare/fps·rpm/60·2π (+ U_Prop['ucav_turns_offset']·2π); arkadan bakınca saat "
     "yönü tersine"),
    ("turret_pan_deg", 0.0, -180.0, 180.0, "Taret yatay açısı (°): + = iskeleye (sola) bakar"),
    ("turret_tilt_deg", 0.0, -90.0, 20.0, "Taret düşey açısı (°): + = yukarı, −90 = tam aşağı (nadir)"),
    ("nav_lights", 1.0, 0.0, 1.0,
     "Seyrüsefer ışıkları (iskele kırmızı, sancak yeşil) ve taret durum halkası: 0 = kapalı, 1 = açık"),
    ("strobe", 1.0, 0.0, 1.0, "Beyaz çakar ışıklar (dikey uçlarında): 0 = kapalı, 1 = açık (kareye bağlı çift çakış)"),
]
PROP_NAMES = tuple(p[0] for p in PROPS)

LIGHT_MATERIALS = {"UM_NavRed": "nav", "UM_NavGreen": "nav", "UM_StatusLED": "nav", "UM_Strobe": "strobe"}
LIGHT_OBJECTS = {"U_Light_Nav_L": "nav", "U_Light_Nav_R": "nav", "U_Light_Turret_Ring": "nav",
                 "U_Light_Strobe_L": "strobe", "U_Light_Strobe_R": "strobe", "U_Light_Landing": "nav"}


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
    """Amortisör: ``z0 + clamp((zemin − (z − R))/cos(yatıklık), 0, strok)``; değişken ``z`` = aks referansı dünya Z."""
    from . import gear as GR
    lg = GR.leg_geom(leg)
    c = math.cos(math.radians(lg.leg.rake_deg))
    return (f"{_f(lg.z['slider_top'])} + clamp(({_f(P.GROUND_Z)} - z + {_f(lg.R_w)}) / {_f(c)}, 0, {_f(lg.stroke)})")


def link_expr(leg: str, upper: bool) -> str:
    """Tork bağlantısı açısı; değişken ``L`` = kayar borunun ``location.z``'si."""
    from . import gear as GR
    lk = GR.leg_geom(leg).link
    z0 = GR.leg_geom(leg).z["slider_top"]
    alpha = f"acos(clamp(({_f(lk['D0'])} - (L - {_f(z0)})) / {_f(2 * lk['ell'])}, -1, 1))"
    a0 = _f(lk["alpha0"])
    return f"{a0} - {alpha}" if upper else f"{alpha} - {a0}"


def wheel_expr(leg: str) -> str:
    from . import gear as GR
    return f"(x - {_f(P.U_ROOT_B[0])}) / {_f(GR.leg_geom(leg).R_w)}"


def prop_expr() -> str:
    return "(frame / (fps / fb) * rpm / 60 + k) * 2 * pi"


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


def _root_vars(d, root, names) -> None:
    alias = {"g": "gear", "d": "gear_doors", "a": "aileron_deg", "f": "flap_deg", "e": "elevator_deg", "r": "rudder_deg",
             "rpm": "prop_rpm", "pan": "turret_pan_deg", "tilt": "turret_tilt_deg", "n": "nav_lights", "s": "strobe"}
    for v in names:
        _var_prop(d, v, root, f'["{alias[v]}"]')


def ensure_props(root: bpy.types.Object, reset: bool = False) -> None:
    """Sözleşmedeki özel özellikleri (float, min/maks, Türkçe açıklama) kurar; var olan değerler korunur."""
    for name, default, lo, hi, desc in PROPS:
        if reset or name not in root.keys():
            root[name] = float(default)
        else:
            root[name] = float(min(max(float(root[name]), lo), hi))
        ui = root.id_properties_ui(name)
        ui.update(min=lo, max=hi, soft_min=lo, soft_max=hi, default=float(default), description=desc,
                  step=100 if hi - lo > 2 else 10, precision=2 if hi - lo <= 2 else 1)
    root["ucav_rig"] = "YK-38 kontrol paneli (rig.setup)"


def drive_light_materials(scene: bpy.types.Scene | None = None) -> list[str]:
    """Işık malzemelerinin yayım şiddetini sürer (Principled "Emission Strength" / Emission "Strength");
    temel şiddet ``params.MATERIALS[...].emission``. Sürülen soketlerin listesini döndürür."""
    root = bpy.data.objects[ROOT]
    done = []
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
            expr = f"{_f(base)} * clamp(n, 0, 1)" if kind == "nav" else f"{_f(base)} * " + strobe_expr("clamp(s, 0, 1)")
            d = _driver(nt, path, -1, expr)
            _root_vars(d, root, ["n"] if kind == "nav" else ["s"])
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
                     "vars": [("z", f"U_GearAxleRef_{leg}", "LOC_Z")]})
        for up, nm in ((True, "U"), (False, "L")):
            rows.append({"obj": f"U_GearLink{nm}_{leg}", "path": "rotation_euler", "index": 1,
                         "expr": link_expr(leg, up), "vars": [("L", f"U_GearSlider_{leg}", "location[2]")]})
        rows.append({"obj": f"U_GearWheel_{leg}", "path": "rotation_euler", "index": 1, "expr": wheel_expr(leg),
                     "vars": [("x", ROOT, "LOC_X")]})
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
                 "vars": ["rpm", ("fps", "SCENE", "render.fps"), ("fb", "SCENE", "render.fps_base"),
                          ("k", "U_Prop", '["ucav_turns_offset"]')]})
    rows.append({"obj": "U_Turret_Pan", "path": "rotation_euler", "index": 2, "expr": "radians(clamp(pan, -180, 180))",
                 "vars": ["pan"]})
    rows.append({"obj": "U_Turret_Tilt", "path": "rotation_euler", "index": 1, "expr": "-radians(clamp(tilt, -90, 20))",
                 "vars": ["tilt"]})
    for ob, kind in LIGHT_OBJECTS.items():
        rows.append({"obj": ob, "path": '["ucav_emission"]', "index": -1,
                     "expr": "clamp(n, 0, 1)" if kind == "nav" else strobe_expr("clamp(s, 0, 1)"),
                     "vars": ["n"] if kind == "nav" else ["s"]})
    return rows


def setup(scene: bpy.types.Scene | None = None, *, reset: bool = False) -> dict:
    """Özel özellikleri ve bütün sürücüleri kurar (``airframe.build`` + ``gear.build`` sonrası). Tekrar
    çağrılabilir. ``reset`` True ise özellikler varsayılana döner. Kurulan sürücü sayısı ve eksik nesneleri döndürür."""
    scene = scene or bpy.context.scene
    root = bpy.data.objects.get(ROOT)
    if root is None:
        raise RuntimeError("U_Root yok: önce airframe.build() ve gear.build()")
    ensure_props(root, reset)
    prop = bpy.data.objects.get("U_Prop")
    if prop is not None and "ucav_turns_offset" not in prop.keys():
        prop["ucav_turns_offset"] = 0.0
        prop.id_properties_ui("ucav_turns_offset").update(
            description="Tur ofseti (animasyon): devir değişirken açıyı ∫rpm·dt'ye eşitler; el ile kullanımda 0")
    n_ok, missing, not_simple = 0, [], []
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
            d = _driver(ob, row["path"], -1, row["expr"])
        else:
            d = _driver(ob, row["path"], row["index"], row["expr"])
        for v in row["vars"]:
            if isinstance(v, str):
                _root_vars(d, root, [v])
            elif v[1] == "SCENE":
                _var_prop(d, v[0], scene, v[2], "SCENE")
            elif v[2].startswith(("LOC_", "ROT_")):
                _var_transform(d, v[0], bpy.data.objects[v[1]], v[2])
            else:
                _var_prop(d, v[0], bpy.data.objects[v[1]], v[2])
        if not d.is_simple_expression:
            not_simple.append(f"{row['obj']}.{row['path']}[{row['index']}]")
        n_ok += 1
    mats = drive_light_materials(scene)
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
