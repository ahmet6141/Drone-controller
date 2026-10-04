#!/usr/bin/env python3
"""YELKOVAN YK-38 — tek komutla Blender sahnesi ve bütün çıktılar (bpy 4.5, Cycles CPU).

Sıra: ``airframe`` → ``gear`` → ``rig`` → ``materials`` → ``studio`` → ``animation`` → çıktılar
(``--print`` baskı parçaları/STL, ``--glb``, ``--blend``, ``--stills`` sabit görüntüler, ``--anim`` animasyonlar).
Her modül tekrar çağrılabilir; bu betik onları sözleşmedeki sırayla çağırır, süreleri ölçer ve sonda çıktı
dosyalarını boyutlarıyla listeler.

Kullanım (depo kökünden)
------------------------
::

    python3 ucav/blender/build.py --blend                         # ucav/out/yk38.blend (sıkıştırılmış, rig hazır)
    python3 ucav/blender/build.py --blend --glb --print --stills    # sahne + GLB + STL/rapor + 9 sabit görüntü
    python3 ucav/blender/build.py --stills hero side --samples 32 --res 960x600
    python3 ucav/blender/build.py --stills hero rear34 --livery taktik      # → yk38_hero_taktik.jpg …
    python3 ucav/blender/build.py --stills hero side --env studyo           # koyu stüdyo
    python3 ucav/blender/build.py --anim mechanisms --res 960x540 --samples 16
    python3 ucav/blender/build.py --anim showcase --frames 1-120 --step 2   # hızlı önizleme (süre korunur)
    python3 ucav/blender/build.py --print --bed 220                         # 220×220×250 tabla raporu
    blender -b -P ucav/blender/build.py -- --blend --stills                 # Blender uygulamasıyla (arka plan)

Blender arayüzünde (Scripting sekmesi): *Text → Open* ile bu dosyayı açıp *Run Script* (▶). Argüman yoksa
sahne mevcut dosyaya kurulur, çıktı yazılmaz; argüman vermek için aşağıdaki ``SCRIPTING_ARGS`` satırını
düzenleyin (ör. ``"--blend --stills hero"``) ya da Blender'ı ``UCAV_BUILD_ARGS`` ortam değişkeniyle başlatın.
Terminalde çıktı bayrağı verilmezse ``--blend`` varsayılır.

Blender uygulamasının kendi Python'unda PyYAML yoksa (``params`` spec.yaml'ı onunla okur) sistem Python'undaki
saf-Python ``yaml`` paketi geçici bir yol üzerinden ödünç alınır; o da yoksa kurulum komutu yazdırılır.

Süreler (4 çekirdekli CPU, ölçülen): sahne kurulumu ≈ 25 s; baskı (``--print``) ≈ 1,5 dk (+ sahne); 9 sabit
görüntü 1600×1000 / 128 örnek ≈ 33 dk; animasyonlar için ``render.py`` belgesine ve ``ucav/README.md`` §7'ye bakın.
"""
from __future__ import annotations

import argparse
import contextlib
import os
import shutil
import subprocess
import sys
import tempfile
import time
from pathlib import Path

# Blender arayüzünden (Scripting sekmesi) çalıştırırken argümanlar: ör. "--blend --stills hero side --samples 32"
SCRIPTING_ARGS = ""

VIEWS_ALL = ("hero", "rear34", "side", "front", "top", "under", "nose", "tail", "gearbay")
CLIPS = ("showcase", "mechanisms")
BEDS = {"256": (256, 256, 256), "220": (220, 220, 250)}
BLEND_NAME = "yk38.blend"
GLB_NAME = "yk38.glb"
STILL_DEFAULT = {"res": (1600, 1000), "samples": 64}
ANIM_DEFAULT = {"res": (1280, 720), "samples": 24}

_T0 = time.time()


def log(msg: str) -> None:
    print(f"[build {time.time() - _T0:7.1f} s] {msg}", flush=True)


# =====================================================================================================
# Ortam: depo kökü, sys.path, PyYAML
# =====================================================================================================
def _repo_root() -> Path:
    """Depo kökünü bulur (``ucav/params.py`` içeren dizin): ``__file__``, Blender metin blokları, açık .blend,
    ``UCAV_REPO`` ortam değişkeni ve çalışma dizini sırayla denenir."""
    cands: list[Path] = []
    env = os.environ.get("UCAV_REPO")
    if env:
        cands.append(Path(env) / "ucav")
    f = globals().get("__file__")
    if f:
        cands.append(Path(f))
    try:
        import bpy
        for t in bpy.data.texts:
            if t.filepath:
                cands.append(Path(bpy.path.abspath(t.filepath)))
        if bpy.data.filepath:
            cands.append(Path(bpy.data.filepath))
    except Exception:                                   # bpy yok ya da kısıtlı bağlam
        pass
    cands.append(Path.cwd() / "_")
    for c in cands:
        try:
            c = c.resolve()
        except OSError:
            continue
        for p in (c, *c.parents):
            if (p / "ucav" / "params.py").is_file() and (p / "ucav" / "blender" / "airframe.py").is_file():
                return p
    raise RuntimeError("depo kökü bulunamadı: UCAV_REPO=/yol/Drone-controller ortam değişkenini ayarlayın")


def _ensure_yaml() -> None:
    """PyYAML yoksa (Blender'ın paketli Python'u) sistem Python'unun ``yaml`` paketini geçici dizine bağlar."""
    try:
        import yaml  # noqa: F401
        return
    except ImportError:
        pass
    for exe in ("python3", "python"):
        path = shutil.which(exe)
        if not path or Path(path).resolve() == Path(sys.executable).resolve():
            continue
        try:
            res = subprocess.run([path, "-c", "import os, yaml; print(os.path.dirname(yaml.__file__))"],
                                 capture_output=True, text=True, timeout=30)
        except (OSError, subprocess.SubprocessError):
            continue
        src = Path(res.stdout.strip()) if res.returncode == 0 else None
        if not src or not src.is_dir():
            continue
        shim = Path(tempfile.gettempdir()) / "ucav_yaml_shim"
        dst = shim / "yaml"
        shim.mkdir(exist_ok=True)
        if not dst.exists():
            try:
                dst.symlink_to(src, target_is_directory=True)
            except OSError:
                shutil.copytree(src, dst)
        sys.path.append(str(shim))
        try:
            import yaml  # noqa: F401,F811  (C hızlandırıcısı uyumsuzsa saf Python sürümü yüklenir)
            print(f"[build] PyYAML sistem Python'undan ödünç alındı: {src}")
            return
        except ImportError:
            sys.path.remove(str(shim))
    raise SystemExit(f"PyYAML bulunamadı. Blender'ın Python'una kurun:\n  \"{sys.executable}\" -m pip install pyyaml")


def _setup_path() -> Path:
    root = _repo_root()
    if str(root) not in sys.path:
        sys.path.insert(0, str(root))
    _ensure_yaml()
    return root


# =====================================================================================================
# Argümanlar
# =====================================================================================================
def _res(text: str) -> tuple[int, int] | int:
    t = text.lower().replace("×", "x")
    if "x" in t:
        w, h = t.split("x", 1)
        return int(w), int(h)
    return int(t)


def _frames(text: str) -> tuple[int, int]:
    a, _, b = text.partition("-")
    a, b = int(a), int(b or a)
    if b < a:
        raise argparse.ArgumentTypeError("kare aralığı a-b, a ≤ b olmalı")
    return a, b


class _TrHelp(argparse.HelpFormatter):
    def add_usage(self, usage, actions, groups, prefix=None):
        return super().add_usage(usage, actions, groups, "kullanım: " if prefix is None else prefix)


class _TrParser(argparse.ArgumentParser):
    def error(self, message):
        self.print_usage(sys.stderr)
        self.exit(2, f"{self.prog}: hata: {message}\n")


def parser() -> argparse.ArgumentParser:
    ap = _TrParser(
        prog="build.py", add_help=False, formatter_class=_TrHelp,
        description="YELKOVAN YK-38 sahnesini kurar (gövde → takım → rig → malzemeler → stüdyo → animasyon) ve "
                    "istenen çıktıları üretir. Terminalde çıktı bayrağı verilmezse --blend varsayılır.",
        epilog="Örnek: python3 ucav/blender/build.py --blend --glb --print --stills  |  "
               "blender -b -P ucav/blender/build.py -- --anim mechanisms --res 960x540 --samples 16")
    ap._optionals.title = "genel"
    ap.add_argument("-h", "--help", action="help", help="bu yardımı göster ve çık")
    g = ap.add_argument_group("çıktılar")
    g.add_argument("--blend", action="store_true", help=f"sahneyi ucav/out/{BLEND_NAME} olarak kaydet (sıkıştırılmış; "
                   "kontrol paneli, sürücüler ve iki animasyon klibi hazır)")
    g.add_argument("--glb", action="store_true", help=f"uçağı ucav/out/{GLB_NAME} olarak dışa aktar (glTF 2.0, Y yukarı; "
                   "boya renkleri basit PBR'ye çevrilir; 'mechanisms' döngüsü — takım, yüzeyler, taret, pervane — "
                   "kare kare pişirilmiş tek animasyon olarak eklenir)")
    g.add_argument("--glb-static", action="store_true", help="GLB'yi animasyonsuz, dinlenme pozunda yaz")
    g.add_argument("--stills", nargs="*", metavar="GÖRÜNÜM", default=None,
                   help=f"sabit görüntüler → ucav/out/render/yk38_<görünüm>.jpg; görünüm verilmezse hepsi: "
                        f"{' '.join(VIEWS_ALL)}")
    g.add_argument("--anim", choices=(*CLIPS, "all"), default=None,
                   help="animasyon → ucav/out/anim/<klip>.mp4 (H.264): showcase (21 s, pist), mechanisms "
                        "(16 s döngü, stüdyo) ya da all")
    g.add_argument("--print", dest="do_print", action="store_true",
                   help="3B baskı parçaları: segmentler, STL (mm) ve Türkçe baskı raporu (ucav/out/stl, print_report.md)")
    o = ap.add_argument_group("ayarlar")
    o.add_argument("--bed", choices=sorted(BEDS), default="256",
                   help="baskı tablası: 256 = 256×256×256 mm (STL'ler ucav/out/stl), 220 = 220×220×250 mm "
                        "(ucav/out/print_220x220x250)")
    o.add_argument("--no-stl", action="store_true", help="--print ile yalnız rapor yaz (STL yok)")
    o.add_argument("--livery", choices=("standart", "taktik"), default="standart",
                   help="boya şeması (taktik: koyu gri, yalnız render; LW-PLA güneşte ısınır)")
    o.add_argument("--env", choices=("pist", "studyo"), default=None,
                   help="sabit görüntü ortamı (varsayılan pist); animasyonda klibin kendi ortamı kullanılır")
    o.add_argument("--samples", type=int, default=None,
                   help=f"Cycles örnek sayısı (varsayılan: görüntü {STILL_DEFAULT['samples']}, "
                        f"animasyon {ANIM_DEFAULT['samples']})")
    o.add_argument("--res", type=_res, default=None, metavar="GxY",
                   help="çözünürlük, ör. 1600x1000 (tek sayı = uzun kenar). Varsayılan: görüntü 1600x1000, "
                        "animasyon 1280x720")
    o.add_argument("--frames", type=_frames, default=None, metavar="a-b",
                   help="animasyonda yalnız bu kare aralığı (ör. 1-120)")
    o.add_argument("--step", type=int, default=1, help="animasyonda her N. kare (önizleme; video süresi korunur)")
    o.add_argument("--no-motion-blur", action="store_true", help="animasyonda hareket bulanıklığını kapat")
    o.add_argument("--gpu", action="store_true",
                   help="Cycles'ı GPU'da çalıştır (OptiX → CUDA → HIP → Metal → oneAPI sırayla denenir; "
                        "GPU bulunamazsa CPU). Kendi bilgisayarında render için önerilir")
    o.add_argument("--keep-scene", action="store_true",
                   help="mevcut sahneyi sıfırlama (varsayılan: arka planda boş fabrika sahnesiyle başlanır)")
    o.add_argument("-q", "--quiet", action="store_true", help="modüllerin ayrıntılı günlüğünü kapat")
    return ap


def _argv(argv: list[str] | None) -> list[str]:
    if argv is not None:
        return list(argv)
    if "--" in sys.argv:
        return sys.argv[sys.argv.index("--") + 1:]
    import bpy
    if not bpy.app.background:                          # Blender arayüzü: Scripting sekmesi
        return (os.environ.get("UCAV_BUILD_ARGS") or SCRIPTING_ARGS).split()
    if Path(sys.argv[0]).name.lower().startswith("blender"):     # blender -b -P build.py (-- yok)
        return (os.environ.get("UCAV_BUILD_ARGS") or "").split()
    return sys.argv[1:]


# =====================================================================================================
# Sahne
# =====================================================================================================
_DEFAULT_OBJECTS = {"Cube": "MESH", "Light": "LIGHT", "Camera": "CAMERA"}


def reset_scene(keep: bool = False) -> None:
    """Arka planda boş fabrika sahnesi; arayüzde yalnız Blender'ın varsayılan küp/ışık/kamerası silinir
    (açık metin düzenleyicisi ve kullanıcının diğer verisi korunur)."""
    import bpy
    if bpy.app.background and not keep:
        bpy.ops.wm.read_factory_settings(use_empty=True)
        return
    for name, kind in _DEFAULT_OBJECTS.items():
        ob = bpy.data.objects.get(name)
        if ob is not None and ob.type == kind and not ob.name.startswith(("U_", "S_")):
            bpy.data.objects.remove(ob, do_unlink=True)


def build_scene(livery: str = "standart", verbose: bool = True) -> dict:
    """Sözleşme sırasıyla bütün modülleri kurar; her adımın süresini ve özetini döndürür."""
    import bpy
    from ucav.blender import airframe, animation, gear, materials, rig, studio
    scene = bpy.context.scene
    info: dict = {"timings": {}}

    def step(name, fn):
        t0 = time.time()
        res = fn()
        info["timings"][name] = round(time.time() - t0, 2)
        log(f"{name}: {info['timings'][name]:.1f} s")
        return res

    objs = step("airframe", lambda: airframe.build(scene, verbose=verbose))
    info["airframe_objects"] = len(objs)
    step("gear", lambda: gear.build(scene))
    r = step("rig", lambda: rig.setup(scene))
    info["rig"] = {k: r[k] for k in ("drivers", "missing", "not_simple")}
    if r["missing"] or r["not_simple"]:
        log(f"UYARI rig: eksik {r['missing']}, basit olmayan {r['not_simple']}")
    m = step("materials", lambda: materials.build(livery, scene=scene))
    info["materials"] = {"livery": m["livery"], "count": len(m["materials"]), "decals": len(m["decals"]),
                         "missing": m["missing"]}
    if m["missing"]:
        log(f"UYARI malzeme: yuvası eksik nesneler {m['missing']}")
    st = step("studio", lambda: studio.setup(scene, env="pist"))
    info["studio"] = {"env": st["env"], "cameras": len(st["cameras"])}
    a = step("animation", lambda: animation.build(scene, select="showcase"))
    info["animation"] = {"actions": len(a["actions"]), "cameras": a["cameras"], "showcase": a["showcase"]}
    finish_scene(scene)
    return info


CLIP_TEXT = "YK38_klip_sec.py"
_CLIP_SWITCHER = '''# YELKOVAN YK-38 — klip seçici ve kontrol paneli notu (depo gerekmez; Text Editor'da ▶ Run Script)
#
# KLIP = "showcase"   : 21 s gösterim (kuruluş planı, kumanda yakın planları, kalkış, takım toplama, dönüş);
#                       kameralar zaman çizelgesi işaretleriyle değişir (YK38_ev_* olay işaretleri kamerasızdır)
# KLIP = "mechanisms" : 16 s kesintisiz döngü (bakım sehpası, yakın planlar: kumandalar, burun tekeri, taret, takım)
# KLIP = "yok"        : animasyon kaldırılır, dinlenme pozu — kendi animasyonunuz için U_Root özelliklerine
#                       anahtar kare koyun (özellik üstünde I tuşu)
#
# Kontrol paneli: U_Root seçin → Object Properties → Custom Properties (ya da N paneli → Item):
#   gear 0…1 (0 = toplu, 1 = açık; kapak-bacak-kapak sırası tek değerle), gear_doors 0…1 (el ile kapak),
#   aileron_deg ±25 (+ = sağa yatış), flap_deg 0…35, elevator_deg ±25 (+ = firar kenarı aşağı),
#   rudder_deg ±25 (+ = firar kenarı sancağa, burun tekeri birlikte), prop_rpm 0…9000,
#   prop_auto 0/1 (1 = açı kare·rpm/1440 tur; 0 = pişirilmiş U_Prop["ucav_turns"], bkz. YK38_pervane_pisir.py),
#   turret_pan_deg ±180 (+ = iskele), turret_tilt_deg −90…20 (+ = yukarı), nav_lights 0/1, strobe 0/1,
#   status_led 0/1 (taret durum LED'i, yalnız bakım/test), ground_z (zemin Z'si: uçağı kendi zemininize koyunca
#   eşitleyin), wheel_auto 0/1 (tekerler U_Root'un yer ilerlemesiyle döner), wheel_roll_m (ek yuvarlanma, m).
# Bütün hareketli parçalar bu özelliklere "basit ifade" sürücüleriyle bağlıdır (Python betiği izni gerekmez).
# Devri değişen kendi animasyonunuzda pervane açısı için YK38_pervane_pisir.py metin bloğunu çalıştırın.
#
# Render (Ctrl+F12): betik klibin ortamını da seçer (showcase → pist, mechanisms → stüdyo: koleksiyon, dünya,
# pozlama, yer dolgusu) ve çıktıyı //anim/yk38_<klip>_ önekine yönlendirir (H.264 MP4). Dosyada hareket
# bulanıklığı, 1280×720 %100 ve 24 örnek hazırdır (komut satırındaki --anim ile aynı). GPU için:
# Edit → Preferences → System → Cycles Render Devices, sonra Render Properties → Device: GPU Compute.
import bpy

KLIP = "mechanisms"

sc = bpy.context.scene
info = sc["ucav_clips"]
root = bpy.data.objects["U_Root"]
prop = bpy.data.objects.get("U_Prop")


def assign(ob, act):
    if ob is None:
        return
    if ob.animation_data is None:
        ob.animation_data_create()
    ob.animation_data.action = act


def layer_col(lc, name):
    if lc.name == name:
        return lc
    for ch in lc.children:
        found = layer_col(ch, name)
        if found is not None:
            return found
    return None


def set_env(name):
    envs = info.get("envs", {})
    if name not in envs:
        return
    for key, e in envs.items():
        lc = layer_col(bpy.context.view_layer.layer_collection, e["collection"])
        if lc is not None:
            lc.exclude = key != name
    e = envs[name]
    w = bpy.data.worlds.get(e["world"])
    if w is not None:
        sc.world = w
    sc.view_settings.exposure = float(e["exposure"])
    fill = bpy.data.objects.get(e.get("fill", ""))
    if fill is not None:
        fill[e["fill_prop"]] = float(e["fill_w"])
        fill.hide_render = float(e["fill_w"]) <= 0.0


for m in list(sc.timeline_markers):
    if m.name.startswith("YK38_"):
        sc.timeline_markers.remove(m)
if KLIP == "yok":
    assign(root, None)
    assign(prop, None)
    root.location = tuple(info["rest_location"])
    root.rotation_euler = (0.0, 0.0, 0.0)
    for k, v in info["defaults"].items():
        root[k] = v
    if prop is not None:
        prop["ucav_turns"] = 0.0
    sc.camera = bpy.data.objects.get("U_Cam_hero") or sc.camera
    sc.frame_start, sc.frame_end = 1, 250
    set_env("pist")
    sc.render.filepath = "//anim/yk38_"
else:
    c = info[KLIP]
    assign(root, bpy.data.actions.get("YK38_%s_Root" % KLIP))
    assign(prop, bpy.data.actions.get("YK38_%s_Prop" % KLIP))
    sc.frame_start, sc.frame_end = 1, int(c["frames"])
    for m in c["markers"]:
        mk = sc.timeline_markers.new("YK38_" + m["camera"][6:], frame=int(m["frame"]))
        mk.camera = bpy.data.objects.get(m["camera"])
    for e in c.get("events", []):                       # kamerasız olay işaretleri (kamera geçişini etkilemez)
        sc.timeline_markers.new("YK38_" + e["name"], frame=int(e["frame"]))
    sc.camera = bpy.data.objects.get(c["markers"][0]["camera"])
    set_env(c.get("env", "pist"))
    sc.render.filepath = "//anim/yk38_%s_" % KLIP
for ob in bpy.data.objects:
    if ob.name.startswith("U_Stand_"):
        ob.hide_render = ob.hide_viewport = KLIP != "mechanisms"
root.update_tag()
sc.frame_set(1)
print("YK-38 klip:", KLIP)
'''


def embed_clip_switcher(scene) -> None:
    """.blend içine depo gerektirmeyen klip seçici metin bloğu ve klip bilgisini (kare sayısı, kamera işaretleri,
    varsayılan kontrol değerleri) koyar."""
    import bpy
    from ucav import params as P
    from ucav.blender import animation, rig
    from ucav.blender import render as R
    from ucav.blender import studio as S
    clips = {}
    for name, plan in (("showcase", animation.showcase_plan), ("mechanisms", animation.mechanisms_plan)):
        pl = plan()
        clips[name] = {"frames": animation.nframes(name), "env": R.CLIP_ENV.get(name, "pist"),
                       "markers": [{"frame": int(f), "camera": str(c)} for f, c in pl["markers"]],
                       "events": [{"frame": int(f), "name": str(n)} for f, n in pl["events"]]}
    clips["envs"] = {env: {"collection": S.ENV_COL if env == "pist" else GUI_STUDIO_COL,
                           "world": "SW_Pist" if env == "pist" else "SW_Studyo",
                           "exposure": float(S.EXPOSURE[env]), "fill": S.UNDER_FILL, "fill_prop": S.FILL_PROP,
                           "fill_w": float(S.ENV_FILL_W[env])} for env in S.ENVS}
    clips["rest_location"] = list(P.U_ROOT_B)
    clips["defaults"] = {p[0]: float(p[1]) for p in rig.PROPS}
    scene["ucav_clips"] = clips
    txt = bpy.data.texts.get(CLIP_TEXT) or bpy.data.texts.new(CLIP_TEXT)
    txt.from_string(_CLIP_SWITCHER)


def finish_scene(scene) -> None:
    """Kullanıcıya hazır sahne: gösterim klibi etkin, kare 1, F12 için makul render ayarları, görünüm
    penceresi malzeme önizlemesi, klip seçici metin bloğu."""
    import bpy
    from ucav.blender import animation, rig
    embed_clip_switcher(scene)
    animation.set_scene_range("showcase", scene)
    r = scene.render
    r.resolution_x, r.resolution_y, r.resolution_percentage = 1920, 1080, 50
    r.filepath = "//anim/yk38_"
    scene.cycles.samples = 64
    scene.frame_set(1)
    rig.refresh()
    root = bpy.data.objects.get("U_Root")
    if root is not None and bpy.context.view_layer.objects.get(root.name) is not None:
        for ob in bpy.context.view_layer.objects:
            ob.select_set(False)
        root.select_set(True)
        bpy.context.view_layer.objects.active = root
    for scr in bpy.data.screens:
        for area in scr.areas:
            if area.type != "VIEW_3D":
                continue
            for sp in area.spaces:
                if sp.type == "VIEW_3D":
                    sp.clip_start, sp.clip_end = 0.01, 2000.0
                    sp.shading.type = "MATERIAL"


def _layer_collection(lc, name):
    if lc.name == name:
        return lc
    for ch in lc.children:
        found = _layer_collection(ch, name)
        if found is not None:
            return found
    return None


# =====================================================================================================
# Çıktılar
# =====================================================================================================
def run_print(bed_key: str, export: bool, verbose: bool) -> dict:
    from ucav.blender import printprep
    bed = BEDS[bed_key]
    t0 = time.time()
    S = printprep.build_print_parts(bed=bed, export=export, verbose=verbose)
    T = S["totals"]
    heat_ok = bool((S.get("heat_rule") or {}).get("ok", True))
    log(f"baskı {bed[0]}×{bed[1]}×{bed[2]}: {T['unique_parts']} benzersiz parça, {T['pieces']} adet, "
        f"{T['mass_g'] / 1000:.2f} kg, ≈{T['print_h']:.0f} h, manifold {'evet' if S['all_manifold'] else 'HAYIR'}, "
        f"sığma {'evet' if S['all_fit'] else 'HAYIR'}, ısı kuralı {'evet' if heat_ok else 'HAYIR'} "
        f"({time.time() - t0:.0f} s)")
    for w in S["warnings"]:
        log(f"  baskı uyarısı: {w}")
    return {"seconds": round(time.time() - t0, 1), "all_manifold": S["all_manifold"], "all_fit": S["all_fit"],
            "heat_ok": heat_ok, "unique_parts": T["unique_parts"], "pieces": T["pieces"],
            "mass_kg": round(T["mass_g"] / 1000, 2), "print_h": round(T["print_h"], 0)}


@contextlib.contextmanager
def _gltf_materials(livery: str):
    """GLB için geçici basit PBR malzemeleri: düğüm ağaçlı boyalar glTF'e renksiz çıkar; aynı ``UM_*`` adıyla
    düz Principled (temel renk = boya şeması rengi) konur, dışa aktarımdan sonra özgün malzemeler geri döner."""
    import bpy
    from ucav import params as P
    from ucav.blender import materials as M
    cols = M.livery_colors(livery)
    root = bpy.data.objects.get("U_Root")
    led = float(root.get("status_led", 0.0)) if root is not None else 0.0      # taret LED'i: dinlenmede sönük
    originals: dict[str, bpy.types.Material] = {}
    temps: dict[str, bpy.types.Material] = {}
    for name, spec in P.MATERIALS.items():
        mat = bpy.data.materials.get(name)
        if mat is None:
            continue
        mat.name = name + "__cycles"
        originals[name] = mat
        tm = bpy.data.materials.new(name)
        tm.use_nodes = True
        b = tm.node_tree.nodes.get("Principled BSDF")
        rgba = (*cols.get(name, spec.rgb_linear), 1.0)
        b.inputs["Base Color"].default_value = rgba
        b.inputs["Roughness"].default_value = float(spec.roughness)
        b.inputs["Metallic"].default_value = float(spec.metallic)
        if spec.transmission > 0:
            b.inputs["Transmission Weight"].default_value = float(spec.transmission)
            b.inputs["IOR"].default_value = float(spec.ior)
        emission = float(spec.emission) * (led if name == "UM_StatusLED" else 1.0)
        if emission > 0:
            b.inputs["Emission Color"].default_value = rgba
            b.inputs["Emission Strength"].default_value = emission
        tm.diffuse_color = rgba
        temps[name] = tm
    by_orig = {m: temps[n] for n, m in originals.items()}
    swapped: list[tuple] = []
    for me in bpy.data.meshes:
        for i, m in enumerate(me.materials):
            if m in by_orig:
                me.materials[i] = by_orig[m]
                swapped.append((me, i, m))
    try:
        yield
    finally:
        for me, i, m in swapped:
            me.materials[i] = m
        for name, tm in temps.items():
            bpy.data.materials.remove(tm)
        for name, mat in originals.items():
            mat.name = name


GLB_PROP_TURNS = 64          # pişirilen döngüde pervane turu: mechanisms 240 dev/dk (384 kare → 60°/kare, ileri yönde)
GLB_EXCLUDE = ("U_PropDisc", "U_Cowl_Cavity")   # glTF'e girmeyenler: pervane diski (karışım malzemesi düz PBR'de
                                                 # opak disk olur) ve yalnız render kaporta boşluğu (R01). Ayrıca
                                                 # ``ucav_render_only`` işaretli her nesne (``_glb_objects``)


@contextlib.contextmanager
def _gltf_prop_spin(active: bool):
    """Pişirme süresince pervane sürücüsü susturulur ve döngüye tam sayıda tur atan (``GLB_PROP_TURNS``) doğrusal,
    doğru yönlü bir dönüş konur: kare başına örneklenen glTF'te hızlı devir geriye dönüyormuş gibi örtüşür
    (stroboskop), döngü dikişinde de açı kesintisiz kalır."""
    import math

    import bpy
    prop = bpy.data.objects.get("U_Prop")
    if not active or prop is None or prop.animation_data is None:
        yield
        return
    ad = prop.animation_data
    drv = [d for d in ad.drivers if d.data_path == "rotation_euler" and d.array_index == 0]
    old_action = ad.action
    scene = bpy.context.scene
    act = bpy.data.actions.new("YK38_glb_PropSpin")
    fc = act.fcurves.new("rotation_euler", index=0) if hasattr(act, "fcurves") else None
    if fc is None:                                       # katmanlı aksiyon API'si (ileri sürümler)
        ad.action = act
        prop.keyframe_insert("rotation_euler", index=0, frame=scene.frame_start)
        fc = ad.action.fcurves[0]
    f0, f1 = scene.frame_start, scene.frame_end + 1
    fc.keyframe_points.add(2)
    fc.keyframe_points[0].co = (f0, 0.0)
    fc.keyframe_points[1].co = (f1, GLB_PROP_TURNS * 2 * math.pi)
    for k in fc.keyframe_points:
        k.interpolation = "LINEAR"
    for d in drv:
        d.mute = True
    ad.action = act
    try:
        yield
    finally:
        for d in drv:
            d.mute = False
        ad.action = old_action
        bpy.data.actions.remove(act)
        prop.rotation_euler[0] = 0.0


def _glb_objects(view_layer) -> list:
    """GLB'ye giden nesneler: ``UCAV`` ağacında görünüm katmanındakiler; ``GLB_EXCLUDE`` ve ``ucav_render_only``
    işaretliler (yalnız render yardımcıları: kaporta boşluğu) hariç."""
    import bpy
    return [o for o in bpy.data.collections["UCAV"].all_objects
            if view_layer.objects.get(o.name) is not None and o.name not in GLB_EXCLUDE
            and not o.get("ucav_render_only")]


def export_glb(path: Path, livery: str, with_anim: bool = False) -> dict:
    """Uçağı (``UCAV`` koleksiyon ağacı; stüdyo, kameralar ve baskı parçaları hariç) GLB olarak yazar. Dinlenme
    pozu (takım açık, kumandalar nötr). ``with_anim``: ``mechanisms`` döngüsü kare kare pişirilir."""
    import bpy
    from ucav.blender import animation, rig
    scene = bpy.context.scene
    t0 = time.time()
    if with_anim:
        animation.set_scene_range("mechanisms", scene)
    else:
        animation.clear(scene)
    vl = bpy.context.view_layer
    for ob in vl.objects:
        ob.select_set(False)
    objs = _glb_objects(vl)
    for o in objs:
        o.select_set(True)
    path.parent.mkdir(parents=True, exist_ok=True)
    kw = dict(filepath=str(path), export_format="GLB", use_selection=True, export_apply=True, export_yup=True,
              export_extras=True, export_cameras=False, export_lights=False, export_animations=with_anim)
    if with_anim:
        kw.update(export_animation_mode="SCENE", export_anim_scene_split_object=False, export_bake_animation=True,
                  export_frame_range=True, export_force_sampling=True, export_optimize_animation_size=True)
    try:
        with _gltf_materials(livery), _gltf_prop_spin(with_anim):
            bpy.ops.export_scene.gltf(**kw)
    finally:
        for o in objs:
            o.select_set(False)
        animation.set_scene_range("showcase", scene)
        scene.frame_set(1)
        rig.refresh()
    sec = round(time.time() - t0, 1)
    log(f"GLB: {path} ({path.stat().st_size / 1e6:.1f} MB, {len(objs)} nesne, {sec} s)")
    return {"file": str(path), "mb": round(path.stat().st_size / 1e6, 2), "objects": len(objs), "seconds": sec}


GUI_STUDIO_COL = "UCAV_Env_Studyo"


def prepare_gui_render(scene) -> dict:
    """Arayüzden render (Ctrl+F12) komut satırındaki ``--anim`` ile aynı sonucu versin: stüdyo ortamı (zemin,
    ışık düzeneği, ``SW_Studyo`` dünyası) pist ortamının yanına ayrı, görünüm katmanından çıkarılmış bir
    koleksiyona (``UCAV_Env_Studyo``) kurulur — klip seçici ``mechanisms``'te onu açar, pisti kapatır; hareket
    bulanıklığı (pervane 64, tekerler 8 alt adım), 1280×720 %100, 24 örnek ve H.264 MP4 çıktısı ayarlanır.
    Sonraki ``--stills``/``--anim`` adımları kendi ayarlarını yeniden kurduğundan etkilenmez."""
    import bpy
    from ucav.blender import render as R
    from ucav.blender import studio as S
    w_pist = scene.world
    old = {o.name for o in bpy.data.objects}
    if bpy.data.objects.get("S_StudioFloor") is None:
        S.build_ground_studio(scene)
        S.build_lights_studio(scene)
    col = S._studio_col(scene, GUI_STUDIO_COL)
    for ob in [o for o in bpy.data.objects if o.name not in old]:
        for c in list(ob.users_collection):
            if c is not col:
                c.objects.unlink(ob)
        if ob.name not in col.objects:
            col.objects.link(ob)
    w_studio = S.build_world_studio(scene)
    scene.world = w_pist if w_pist is not None else bpy.data.worlds.get("SW_Pist")
    for w in (w_pist, w_studio):
        if w is not None:
            w.use_fake_user = True                      # klip seçici dünyayı değiştirince kaybolmasın
    lc = _layer_collection(bpy.context.view_layer.layer_collection, GUI_STUDIO_COL)
    if lc is not None:
        lc.exclude = True                               # varsayılan klip showcase → pist
    R.setup_motion_blur(scene, True)
    r = scene.render
    r.resolution_x, r.resolution_y = ANIM_DEFAULT["res"]
    r.resolution_percentage = 100
    scene.cycles.samples = ANIM_DEFAULT["samples"]
    im = r.image_settings
    im.file_format = "FFMPEG"
    r.ffmpeg.format = "MPEG4"
    r.ffmpeg.codec = "H264"
    r.ffmpeg.constant_rate_factor = "HIGH"
    r.ffmpeg.ffmpeg_preset = "GOOD"
    r.filepath = "//anim/yk38_showcase_"
    return {"studio_objects": len(col.objects), "worlds": [w.name for w in (w_pist, w_studio) if w is not None]}


def save_blend(path: Path) -> dict:
    """Sıkıştırılmış .blend. ``UCAV_Print`` (varsa) görünüm katmanından çıkarılır (dosyada durur; Outliner'da
    işaretlenince görünür) — görünüm penceresi yalnız uçak ve stüdyoyla açılır. Önce ``prepare_gui_render``:
    dosyadan arayüzle alınan render komut satırıyla aynı ayarlarda olur."""
    import bpy
    t0 = time.time()
    path.parent.mkdir(parents=True, exist_ok=True)
    prepare_gui_render(bpy.context.scene)
    lc = _layer_collection(bpy.context.view_layer.layer_collection, "UCAV_Print")
    if lc is not None:
        lc.exclude = True
    bpy.ops.wm.save_as_mainfile(filepath=str(path), compress=True, check_existing=False)
    for bak in (path.with_suffix(".blend1"),):
        if bak.exists():
            bak.unlink()
    sec = round(time.time() - t0, 1)
    log(f"blend: {path} ({path.stat().st_size / 1e6:.1f} MB, {sec} s)")
    return {"file": str(path), "mb": round(path.stat().st_size / 1e6, 2), "seconds": sec}


def render_stills(views, res, samples: int, livery: str, env: str | None) -> dict:
    from ucav.blender import render
    t0 = time.time()
    r = render.render_stills(views or None, res=res, samples=samples, env=env,
                             livery=livery if livery != "standart" else None)
    log(f"sabit görüntüler: {len(r['files'])} dosya, {time.time() - t0:.0f} s — {r['timings']}")
    return {"files": r["files"], "timings": r["timings"], "seconds": round(time.time() - t0, 1)}


def render_anims(clips, res, samples: int, frames, step: int, motion_blur: bool, livery: str) -> dict:
    from ucav.blender import render
    out = {}
    for c in clips:
        t0 = time.time()
        r = render.render_animation(c, res=res, samples=samples, frames=frames, frame_step=step,
                                    motion_blur=motion_blur, livery=livery if livery != "standart" else None)
        log(f"animasyon {c}: {r['file']} — {r['frames']} kare, {r['s_per_frame']} s/kare, {time.time() - t0:.0f} s")
        out[c] = r
    return out


def _size(p: Path) -> str:
    n = p.stat().st_size
    return f"{n / 1e6:.1f} MB" if n >= 1e5 else f"{n / 1e3:.0f} kB"


def outputs_summary(root: Path) -> list[str]:
    """``ucav/out`` altındaki çıktıların dosya boyutları (rapor)."""
    out = root / "ucav" / "out"
    lines = []
    for name in (BLEND_NAME, GLB_NAME, "print_report.md", "sizing.md"):
        p = out / name
        if p.exists():
            lines.append(f"  {p.relative_to(root)}: {_size(p)}")
    stl = sorted((out / "stl").glob("*.stl"))
    if stl:
        tot = sum(p.stat().st_size for p in stl)
        lines.append(f"  ucav/out/stl/: {len(stl)} STL, {tot / 1e6:.1f} MB")
    for sub, pat in (("render", "*.jpg"), ("anim", "*.mp4")):
        for p in sorted((out / sub).glob(pat)):
            lines.append(f"  {p.relative_to(root)}: {_size(p)}")
    return lines


# =====================================================================================================
# Ana akış
# =====================================================================================================
def main(argv: list[str] | None = None) -> int:
    root = _setup_path()
    import bpy
    raw = _argv(argv)
    try:
        args = parser().parse_args(raw)
    except SystemExit as exc:                           # --help ya da hatalı argüman: Blender arayüzünü kapatma
        if bpy.app.background:
            raise
        return int(exc.code or 0)
    wants = args.blend or args.glb or args.glb_static or args.do_print or args.stills is not None or args.anim
    if not wants and bpy.app.background and argv is None:
        args.blend = True
        log("çıktı bayrağı yok → --blend (yardım: --help)")
    out = root / "ucav" / "out"
    log(f"depo {root}; Blender {bpy.app.version_string}; argümanlar: {' '.join(raw) or '(yok)'}")
    reset_scene(args.keep_scene)
    info = build_scene(args.livery, verbose=not args.quiet)
    summary: dict = {"scene": info}
    if args.gpu:
        from ucav.blender import studio
        backend = studio.enable_gpu(bpy.context.scene)
        summary["device"] = backend
        log(f"render aygıtı: {backend}" + (" (GPU bulunamadı, CPU kullanılıyor)" if backend == "CPU" else ""))
    ok = not info["rig"]["missing"] and not info["materials"]["missing"]
    if args.do_print:
        summary["print"] = run_print(args.bed, export=not args.no_stl, verbose=not args.quiet)
        ok = ok and summary["print"]["all_manifold"] and summary["print"]["all_fit"] and summary["print"]["heat_ok"]
    if args.glb or args.glb_static:
        summary["glb"] = export_glb(out / GLB_NAME, args.livery, with_anim=not args.glb_static)
    if args.blend:
        summary["blend"] = save_blend(out / BLEND_NAME)
    if args.stills is not None:
        summary["stills"] = render_stills(args.stills, args.res or STILL_DEFAULT["res"],
                                          args.samples or STILL_DEFAULT["samples"], args.livery, args.env)
    if args.anim:
        clips = CLIPS if args.anim == "all" else (args.anim,)
        summary["anim"] = render_anims(clips, args.res or ANIM_DEFAULT["res"], args.samples or ANIM_DEFAULT["samples"],
                                       args.frames, args.step, not args.no_motion_blur, args.livery)
    log("sahne adımları: " + ", ".join(f"{k} {v:.1f} s" for k, v in info["timings"].items()))
    lines = outputs_summary(root)
    if lines:
        log("çıktılar:\n" + "\n".join(lines))
    log(f"bitti ({'sorunsuz' if ok else 'UYARILAR VAR'}), toplam {time.time() - _T0:.0f} s")
    return 0 if ok else 1


if __name__ == "__main__":
    try:
        import bpy as _bpy
        _ui = not _bpy.app.background
    except ImportError:                                  # bpy yoksa (ör. düz Python, bpy kurulu değil)
        sys.exit("bpy bulunamadı: 'pip install bpy==4.5.*' ya da 'blender -b -P ucav/blender/build.py -- ...'")
    if _ui:
        main()                                           # Scripting sekmesi: Blender'ı kapatma
    else:
        sys.exit(main())
