"""YELKOVAN YK-38 — render (bpy 4.5, Cycles CPU): sabit görüntüler (JPG) ve animasyon (MP4, H.264).

* ``render_stills(views, res, samples, out_dir)`` — ``studio.VIEWS`` kameralarından JPG (``ucav/out/render/``).
  Her görünümün kontrol değerleri (ör. ``under``: takım toplu; ``gearbay``: kapaklar açık), güneş yönü ve zemin
  görünürlüğü geçici uygulanır, render sonrası geri alınır. Kare oranı görünümden gelir (``res`` uzun kenar
  ya da (g, y) çifti).
* ``render_animation(name, res, samples, out)`` — ``animation`` klibi (``showcase`` | ``mechanisms``) kare kare
  JPG olarak işlenir (yarıda kalırsa ``resume=True`` var olan kareleri atlar), sonra Blender'ın kendi
  FFMPEG'iyle (VSE) MP4/H.264'e kodlanır — harici ``ffmpeg`` gerekmez. ``frame_step`` > 1 hızlı önizleme
  verir; video kare hızı ``fps / frame_step`` olur (süre korunur).
* Hareket bulanıklığı (animasyonda açık): obtüratör 0,5 kare (180°). Pervane ``U_Prop`` için Cycles hareket
  adımı 2⁶ = 64 → 8600 dev/dk'da obtüratör içindeki ≈ 3 tur doğru yaylanır; tekerler 2³. Palaların izi bu hızda
  çok soluk kaldığından 900 dev/dk üstünde ``U_PropDisc`` (yarı saydam açık "pus" diski, uçta yumuşak kenarlı soluk
  turuncu halka; rig sürer) görünür.
* Gürültü giderme: OpenImageDenoise (albedo + normal). Görünüm dönüşümü: AgX, "Medium High Contrast"; pozlama
  ortama göre (``studio.EXPOSURE``: pist −0,4 EV, stüdyo 0) — her render çağrısında ``studio.configure_cycles``
  yeniden uygular. Yer yansıması dolgusu (``S_UnderFill``) animasyonda uçağı izler ve kalkıştan sonra söner.

Süreler (4 çekirdekli CPU)
--------------------------
* Önizleme 640×400, 16 örnek: ``pist`` görünüm başına 6–17 s, ``studyo`` 2–5 s.
* 960×600, 24 örnek: ``pist`` görünüm başına ≈ 20–26 s.
* **Depodaki sabit görüntüler 1600×1000, 128 örnek** (makine boşken): ``pist`` görünüm başına 80–370 s (``under``
  en hızlı, ``gearbay`` en yavaş), 9 görünüm ≈ 33 dk; taktik şema 178–233 s; ``studyo`` 137–149 s. Varsayılan
  64 örnek kabaca yarı süredir.
* Depodaki animasyon önizlemeleri 960×540, 16 örnek, hareket bulanıklığı (ölçüldü): ``showcase`` (504 kare)
  10,1 s/kare ≈ 85 dk (yakın planlar 16–22 s, havada 6–7 s), ``mechanisms`` (384 kare, stüdyo) 11,4 s/kare ≈ 73 dk.
* Animasyon varsayılanı 1280×720, 24 örnek (tahmin ≈ 2,7 × önizleme): ``showcase`` ≈ 3,8 saat, ``mechanisms``
  ≈ 3,3 saat.
* Hızlı önizleme ``res=(640, 360), samples=12, frame_step=2`` (12 fps, süre korunur): kare sayısı yarıya, kare
  süresi ≈ 1/3'e iner (tahmin: ``showcase`` ≈ 15 dk, ``mechanisms`` ≈ 12 dk).
Süreler ``render_stills(...)["timings"]`` ve ``render_animation(...)["s_per_frame"]`` ile döner.

Kullanım (``build.py`` bu fonksiyonları çağırır)::

    from ucav.blender import airframe, gear, materials, rig, animation, studio, render
    airframe.build(); gear.build(); materials.build(); rig.setup(); animation.build(); studio.setup()
    render.render_stills(["hero", "side"], res=1600, samples=64)
    render.render_stills(["hero"], livery="taktik")              # ikinci boya şeması (yalnız render)
    render.render_stills(["hero", "side"], env="studyo")          # koyu stüdyo
    render.render_animation("mechanisms", res=(1280, 720), samples=24)
"""
from __future__ import annotations

import os
import shutil
import time
from pathlib import Path

import bpy

from . import studio as S

ROOT_DIR = Path(__file__).resolve().parents[1]                  # ucav/
OUT_RENDER = ROOT_DIR / "out" / "render"
OUT_ANIM = ROOT_DIR / "out" / "anim"
DEFAULT_VIEWS = tuple(S.VIEWS)
JPEG_QUALITY = 92
CLIP_ENV = {"showcase": "pist", "mechanisms": "studyo"}       # env=None iken klibin ortamı
PROP_MOTION_STEPS = 7                                            # 2^(7−1) = 64 alt adım
WHEEL_MOTION_STEPS = 4


def _resolution(res, aspect: float) -> tuple[int, int]:
    """``res``: int → uzun kenar; (g, y) → aynen. Görünüm oranı ``aspect`` (g/y)."""
    if isinstance(res, (tuple, list)):
        return int(res[0]), int(res[1])
    long = int(res)
    if aspect >= 1.0:
        return long, int(round(long / aspect / 2) * 2)
    return int(round(long * aspect / 2) * 2), long


def _image_settings(scene, fmt: str = "JPEG", quality: int = JPEG_QUALITY) -> None:
    im = scene.render.image_settings
    im.file_format = fmt
    im.color_mode = "RGB"
    if fmt == "JPEG":
        im.quality = quality
    elif fmt == "PNG":
        im.color_depth = "8"
        im.compression = 15


def _ensure_scene(scene, env: str | None, livery: str | None) -> None:
    if livery is not None:
        from . import materials as M
        M.build(livery)
    if env is not None and S.current_env(scene) != env:
        S.setup(scene, env=env)
    elif S.current_env(scene) is None or bpy.data.objects.get("U_Cam_hero") is None:
        S.setup(scene, env=env or "pist")


def render_stills(views=None, res=1600, samples: int = 64, out_dir=None, *, env: str | None = None,
                  livery: str | None = None, frame: int | None = None, prefix: str = "yk38_",
                  quality: int = JPEG_QUALITY, motion_blur: bool | None = None,
                  scene: bpy.types.Scene | None = None) -> dict:
    """Seçili görünümleri JPG olarak işler. ``views`` None → hepsi (``studio.VIEWS``). ``res`` uzun kenar (px)
    ya da (g, y). ``env``/``livery`` verilirse önce stüdyo/malzemeler kurulur (sahnede kalıcıdır; geri dönmek için
    ``livery="standart"``/``env="pist"`` verin). ``frame`` verilirse o kare
    (ör. animasyon içinden uçuş anı; kameralar ``U_CamRig_Stills`` ile uçağı izler). ``motion_blur`` None →
    pervane dönüyorsa (``U_Root["prop_rpm"]`` > 0) açık, duruyorsa kapalı.
    Dosya adı: ``<prefix><görünüm>[_<boya>][_<ortam≠pist>].jpg``. Döndürür: {"files": [...], "timings": {...}}."""
    scene = scene or bpy.context.scene
    _ensure_scene(scene, env, livery)
    views = list(views or DEFAULT_VIEWS)
    unknown = [v for v in views if v not in S.VIEWS]
    if unknown:
        raise KeyError(f"bilinmeyen görünüm(ler): {unknown} (mevcut: {', '.join(S.VIEWS)})")
    out = Path(out_dir) if out_dir else OUT_RENDER
    out.mkdir(parents=True, exist_ok=True)
    S.configure_cycles(scene, samples)
    _image_settings(scene, "JPEG", quality)
    r = scene.render
    old = (r.resolution_x, r.resolution_y, r.resolution_percentage, r.filepath, r.use_motion_blur, scene.frame_current)
    if frame is not None:
        scene.frame_set(int(frame))
    if motion_blur is None:
        root = bpy.data.objects.get("U_Root")
        motion_blur = bool(root is not None and float(root.get("prop_rpm", 0.0)) > 0.0)
    setup_motion_blur(scene, motion_blur)
    liv = scene.get("ucav_livery", "standart")
    env_now = S.current_env(scene) or "pist"
    files, timings = [], {}
    try:
        for v in views:
            spec = S.VIEWS[v]
            r.resolution_x, r.resolution_y = _resolution(res, spec.aspect)
            r.resolution_percentage = 100
            suffix = ("" if liv == "standart" else f"_{liv}") + ("" if env_now == "pist" else f"_{env_now}")
            path = out / f"{prefix}{v}{suffix}.jpg"
            state = S.apply_view(v, scene)
            try:
                r.filepath = str(path)
                t0 = time.time()
                bpy.ops.render.render(write_still=True, scene=scene.name)
                timings[v] = round(time.time() - t0, 1)
            finally:
                S.restore_view(state, scene)
            files.append(str(path))
            print(f"[render] {v}: {path.name} {r.resolution_x}×{r.resolution_y}, {samples} örnek, {timings[v]} s")
    finally:
        r.resolution_x, r.resolution_y, r.resolution_percentage, r.filepath, r.use_motion_blur = old[:5]
        if frame is not None:
            scene.frame_set(old[5])
    return {"files": files, "timings": timings}


# =====================================================================================================
# Animasyon
# =====================================================================================================
def setup_motion_blur(scene, enable: bool = True, shutter: float = 0.5) -> None:
    """Hareket bulanıklığı: obtüratör ``shutter`` kare (merkezli); pervane ve tekerlere ek alt adım."""
    r = scene.render
    r.use_motion_blur = enable
    r.motion_blur_shutter = shutter
    if hasattr(r, "motion_blur_position"):
        r.motion_blur_position = "CENTER"
    for name, steps in (("U_Prop", PROP_MOTION_STEPS), ("U_Spinner", 1)):
        ob = bpy.data.objects.get(name)
        if ob is not None:
            ob.cycles.use_motion_blur = True
            ob.cycles.motion_steps = steps
    for ob in bpy.data.objects:
        if ob.name.startswith("U_GearWheel_"):
            ob.cycles.motion_steps = WHEEL_MOTION_STEPS


def _ensure_clip(name: str, scene) -> None:
    from . import animation as A
    if bpy.data.actions.get(f"YK38_{name}_Root") is None:
        A.build(scene, select=name)
    else:
        A.set_scene_range(name, scene)


def render_animation(name: str = "showcase", res=(1280, 720), samples: int = 24, out=None, *,
                     frame_step: int = 1, frames: tuple[int, int] | None = None, motion_blur: bool = True,
                     shutter: float = 0.5, env: str | None = None, livery: str | None = None,
                     resume: bool = True, keep_frames: bool = False, crf: str = "HIGH",
                     scene: bpy.types.Scene | None = None) -> dict:
    """Klibi işler ve MP4 (H.264) yazar. ``frames`` = (ilk, son) alt aralık; ``frame_step`` önizleme seyreltmesi.
    ``env`` None → klibin ortamı (``CLIP_ENV``: gösterim → pist, mekanizmalar → stüdyo).
    Kareler ``<out>_frames/`` altına JPG yazılır (``resume``: var olanlar atlanır), sonra kodlanır.
    Döndürür: {"file", "frames", "seconds", "s_per_frame"}."""
    scene = scene or bpy.context.scene
    _ensure_scene(scene, env or CLIP_ENV.get(name), livery)
    _ensure_clip(name, scene)
    out = Path(out) if out else OUT_ANIM / f"{name}.mp4"
    out.parent.mkdir(parents=True, exist_ok=True)
    fdir = out.with_name(out.stem + "_frames")
    fdir.mkdir(parents=True, exist_ok=True)
    f0, f1 = frames if frames else (scene.frame_start, scene.frame_end)
    step = max(1, int(frame_step))
    S.configure_cycles(scene, samples)
    setup_motion_blur(scene, motion_blur, shutter)
    r = scene.render
    r.resolution_x, r.resolution_y = _resolution(res, 16 / 9)
    r.resolution_percentage = 100
    _image_settings(scene, "JPEG", 95)
    seq = list(range(int(f0), int(f1) + 1, step))
    t_all = time.time()
    rendered = 0
    for k, f in enumerate(seq):
        path = fdir / f"f_{k + 1:05d}.jpg"
        if resume and path.exists() and path.stat().st_size > 0:
            continue
        scene.frame_set(f)
        r.filepath = str(path)
        t0 = time.time()
        bpy.ops.render.render(write_still=True, scene=scene.name)
        rendered += 1
        print(f"[render] {name} kare {f} ({k + 1}/{len(seq)}) {time.time() - t0:.1f} s")
    secs = time.time() - t_all
    fps = scene.render.fps / scene.render.fps_base / step
    encode_mp4(sorted(fdir.glob("f_*.jpg")), out, fps, (r.resolution_x, r.resolution_y), crf=crf)
    if not keep_frames:
        shutil.rmtree(fdir, ignore_errors=True)
    return {"file": str(out), "frames": len(seq), "seconds": round(secs, 1),
            "s_per_frame": round(secs / max(rendered, 1), 1), "fps": fps}


def encode_mp4(images, out, fps: float, size: tuple[int, int], crf: str = "HIGH") -> str:
    """Görüntü dizisini Blender VSE + FFMPEG ile MP4/H.264'e kodlar (görüntüler zaten AgX'li → "Standard")."""
    images = [Path(p) for p in images]
    if not images:
        raise RuntimeError("kodlanacak kare yok")
    out = Path(out)
    enc = bpy.data.scenes.new("UCAV_Encode")
    try:
        ed = enc.sequence_editor_create()
        coll = ed.strips if hasattr(ed, "strips") else ed.sequences
        strip = coll.new_image("frames", str(images[0]), channel=1, frame_start=1)
        for p in images[1:]:
            strip.elements.append(p.name)
        strip.colorspace_settings.name = "sRGB"
        n = len(images)
        enc.frame_start, enc.frame_end = 1, n
        fps_i = max(1, int(round(fps)))
        enc.render.fps = fps_i
        enc.render.fps_base = fps_i / fps if fps > 0 else 1.0
        enc.render.resolution_x, enc.render.resolution_y = size
        enc.render.resolution_percentage = 100
        enc.view_settings.view_transform = "Standard"
        enc.view_settings.look = "None"
        enc.render.use_sequencer = True
        enc.render.use_compositing = False
        im = enc.render.image_settings
        im.file_format = "FFMPEG"
        ff = enc.render.ffmpeg
        ff.format = "MPEG4"
        ff.codec = "H264"
        ff.constant_rate_factor = crf
        ff.ffmpeg_preset = "GOOD"
        ff.gopsize = max(1, int(round(fps)))
        ff.audio_codec = "NONE"
        enc.render.use_file_extension = False
        enc.render.filepath = str(out)
        if out.exists():
            out.unlink()
        bpy.ops.render.render(animation=True, scene=enc.name)
    finally:
        bpy.data.scenes.remove(enc)
    if not out.exists():
        # bazı sürümler kare aralığını ada ekler: bul ve yeniden adlandır
        cands = sorted(out.parent.glob(out.stem + "*.mp4"), key=os.path.getmtime)
        if cands:
            cands[-1].rename(out)
    print(f"[render] MP4: {out} ({len(images)} kare, {fps:.2f} fps)")
    return str(out)
