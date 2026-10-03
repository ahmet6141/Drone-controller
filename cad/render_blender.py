"""DC7 fotogerçekçi render (Blender / Cycles): GLB montajı → render/*.png

Geometri CadQuery'den gelir (ölçü doğruluğu); Blender yalnızca malzeme, ışık, kamera ve render
yapar. Malzemeler cad/visual.py'deki CMF tanımlarından üretilir.

Çalıştırma (ikisinden biri):
    pip install bpy==4.5.3      # Python 3.11; Blender'ı Python modülü olarak kurar
    python cad/render_blender.py --model v2 [--samples 128] [--views hero,front] [--blend]
    blender -b -P cad/render_blender.py -- --model v1 [--samples 128] [--views hero,grip]
Önce: python3 cad/build.py (v1: cad/out/dc7_assembly.glb) veya python3 cad/v2/build_v2.py
(v2: cad/v2/out/dc7_v2.glb).
"""
from __future__ import annotations

import argparse
import math
import sys
import time
from pathlib import Path

import bpy
from mathutils import Vector

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
sys.path.insert(0, str(HERE / "v2"))
import params as P  # noqa: E402
import v2_params as V  # noqa: E402
import visual  # noqa: E402

BACKGROUND = (0.80, 0.82, 0.84)          # doğrusal; AgX sonrası ≈ açık gri stüdyo fonu
V2_GIMBAL = V.GIMBAL_POS                 # kamera merkezi (mm)
# Görünümler: (azimut°, yükseklik°, hedef (m), çerçeve yarıçapı (m) | None = tüm drone, çözünürlük, f-stop | None)
# azimut 0° = burnun önü, −90° = sağ yan
MODELS = {
    "v1": {
        "glb": HERE / "out" / "dc7_assembly.glb", "out": HERE / "out" / "render",
        "views": {
            "hero": (-38.0, 22.0, None, None, (1920, 1200), None),
            "side": (-90.0, 3.0, None, None, (1920, 1000), None),
            "top": (0.0, 90.0, None, None, (1500, 1500), None),
            "grip": (-30.0, -16.0, (0.0, 0.0, -0.055), 0.085, (1500, 1100), 4.0),
            "gimbal": (-22.0, 8.0, (P.GIMBAL_POS[0] / 1000 - 0.012, 0.010, P.GIMBAL_POS[2] / 1000 + 0.012), 0.062,
                       (1500, 1100), 4.0),
        },
    },
    "v2": {
        "glb": HERE / "v2" / "out" / "dc7_v2.glb", "out": HERE / "v2" / "out" / "render",
        "views": {
            "hero": (-36.0, 20.0, None, None, (1920, 1200), None),
            "front": (-14.0, 7.0, None, None, (1920, 1100), None),
            "rear": (-148.0, 24.0, None, None, (1920, 1200), None),
            "side": (-90.0, 2.0, None, None, (1920, 900), None),
            "top": (0.0, 90.0, None, None, (1500, 1500), None),
            "under": (-28.0, -24.0, (0.010, 0.0, -0.045), 0.15, (1600, 1100), 5.6),
            "nose": (-28.0, 10.0, (V2_GIMBAL[0] / 1000 - 0.014, 0.0, V2_GIMBAL[2] / 1000 + 0.004), 0.072,
                     (1600, 1100), 4.0),
        },
    },
}


def parse_args() -> argparse.Namespace:
    argv = sys.argv[sys.argv.index("--") + 1:] if "--" in sys.argv else sys.argv[1:]
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--model", choices=sorted(MODELS), default="v1")
    ap.add_argument("--glb", type=Path, help="varsayılan: modelin GLB dosyası")
    ap.add_argument("--out", type=Path, help="varsayılan: modelin render klasörü")
    ap.add_argument("--samples", type=int, default=128)
    ap.add_argument("--scale", type=float, default=1.0, help="çözünürlük çarpanı (deneme için 0.4)")
    ap.add_argument("--views", help="virgülle ayrılmış; varsayılan: modelin tüm görünümleri")
    ap.add_argument("--format", choices=("jpg", "png"), default="jpg", help="jpg (depo için küçük) veya png")
    ap.add_argument("--blend", action="store_true", help="sahneyi render klasörüne .blend olarak da kaydet")
    args = ap.parse_args(argv)
    model = MODELS[args.model]
    args.glb = args.glb or model["glb"]
    args.out = args.out or model["out"]
    args.view_table = model["views"]
    args.views = args.views or ",".join(model["views"])
    return args


# --- sahne ------------------------------------------------------------------------------------
def import_model(path: Path) -> list[bpy.types.Object]:
    bpy.ops.wm.read_factory_settings(use_empty=True)
    bpy.ops.import_scene.gltf(filepath=str(path))
    scene = bpy.context.scene
    pivot = bpy.data.objects.new("DC7", None)
    scene.collection.objects.link(pivot)
    for obj in list(scene.objects):
        if obj.parent is None and obj is not pivot:
            obj.parent = pivot
    # CadQuery GLB'yi mm yazar (glTF metre bekler) → 0,001 ölçek. Eksen: içe aktarıcı glTF'nin Y-yukarı
    # eksenini Z-yukarıya çevirir; yine de drone'un en ince ekseni Z değilse (eski/başka dışa aktarıcı) düzelt.
    pivot.scale = (0.001, 0.001, 0.001)
    bpy.context.view_layer.update()
    meshes = [o for o in scene.objects if o.type == "MESH"]
    lo, hi = world_bounds(meshes)
    d = hi - lo
    if d.y < d.z and d.y < d.x:
        pivot.rotation_euler = (math.radians(90.0), 0.0, 0.0)
        bpy.context.view_layer.update()
    return meshes


def world_bounds(objs) -> tuple[Vector, Vector]:
    lo = Vector((1e9, 1e9, 1e9))
    hi = Vector((-1e9, -1e9, -1e9))
    for o in objs:
        for corner in o.bound_box:
            w = o.matrix_world @ Vector(corner)
            lo = Vector(map(min, lo, w))
            hi = Vector(map(max, hi, w))
    return lo, hi


def _principled(mat: bpy.types.Material):
    return next(n for n in mat.node_tree.nodes if n.type == "BSDF_PRINCIPLED")


def make_material(key: str, spec: dict) -> bpy.types.Material:
    mat = bpy.data.materials.new(f"DC7_{key}")
    mat.use_nodes = True
    nodes, links = mat.node_tree.nodes, mat.node_tree.links
    bsdf = _principled(mat)
    color = (*visual.linear_rgb(spec["color"]), 1.0)
    bsdf.inputs["Base Color"].default_value = color
    bsdf.inputs["Metallic"].default_value = spec["metallic"]
    bsdf.inputs["Roughness"].default_value = spec["roughness"]
    bsdf.inputs["Coat Weight"].default_value = spec["coat"]
    bsdf.inputs["Coat Roughness"].default_value = 0.06
    if spec.get("emission"):                             # seyir lambaları
        bsdf.inputs["Emission Color"].default_value = color
        bsdf.inputs["Emission Strength"].default_value = spec["emission"]
    coords = nodes.new("ShaderNodeTexCoord")
    if spec["pattern"] == "weave":                       # karbon örgü: nesne uzayında ≈ 3 mm kareler (mm)
        checker = nodes.new("ShaderNodeTexChecker")
        checker.inputs["Scale"].default_value = 0.17
        checker.inputs["Color1"].default_value = color
        checker.inputs["Color2"].default_value = tuple(min(1.0, c * 2.4) for c in color[:3]) + (1.0,)
        links.new(coords.outputs["Object"], checker.inputs["Vector"])
        links.new(checker.outputs["Color"], bsdf.inputs["Base Color"])
        bump = nodes.new("ShaderNodeBump")
        bump.inputs["Strength"].default_value = 0.06
        links.new(checker.outputs["Fac"], bump.inputs["Height"])
        links.new(bump.outputs["Normal"], bsdf.inputs["Normal"])
    elif spec["pattern"] == "grain":                     # baskı yüzeyi: ince mat doku
        noise = nodes.new("ShaderNodeTexNoise")
        noise.inputs["Scale"].default_value = 1.6
        noise.inputs["Detail"].default_value = 6.0
        links.new(coords.outputs["Object"], noise.inputs["Vector"])
        bump = nodes.new("ShaderNodeBump")
        bump.inputs["Strength"].default_value = 0.10
        links.new(noise.outputs["Fac"], bump.inputs["Height"])
        links.new(bump.outputs["Normal"], bsdf.inputs["Normal"])
    return mat


def assign_materials(objs) -> dict[str, int]:
    mats = {key: make_material(key, spec) for key, spec in visual.MATERIALS.items()}
    used: dict[str, int] = {}
    for o in objs:
        key = visual.material_for(o.name)
        o.data.materials.clear()
        o.data.materials.append(mats[key])
        used[key] = used.get(key, 0) + 1
    return used


def studio(lo: Vector, hi: Vector) -> None:
    scene = bpy.context.scene
    world = bpy.data.worlds.new("studio")
    world.use_nodes = True
    bg = world.node_tree.nodes["Background"]
    bg.inputs["Color"].default_value = (*BACKGROUND, 1.0)
    bg.inputs["Strength"].default_value = 0.22
    scene.world = world
    # Gölge yakalayıcı zemin: drone zeminin 12 cm üstünde süzülüyor gibi
    z_floor = lo.z - 0.12
    bpy.ops.mesh.primitive_plane_add(size=30.0, location=(0.0, 0.0, z_floor))
    floor = bpy.context.active_object
    floor.name = "zemin"
    floor.is_shadow_catcher = True
    size = max(hi.x - lo.x, hi.y - lo.y)
    center = (lo + hi) / 2
    # Alan ışığı (tek yüzlü Lambert): E = P / (π d²). Anahtar ışık ≈ 3 W/m² hedefler → beyaz yüzey ≈ 0,6
    # doğrusal değer, AgX'te patlamaz. Tepe ışığı küçük ve zayıf tutulur: büyük bir tepe ışığı yukarı bakan
    # tüm yüzeylerde yansır ve koyu plastikleri griye/beyaza çeker ("soluk" görünüm).
    for name, energy, offset, radius in (("ana", 36.0, (1.1, -1.2, 1.2), 1.0),
                                         ("dolgu", 13.0, (-1.4, -0.7, 0.6), 1.4),
                                         ("kontur", 30.0, (-0.5, 1.4, 1.0), 0.8),
                                         ("tepe", 6.0, (-0.6, 0.3, 1.8), 0.7)):
        light = bpy.data.lights.new(name, type="AREA")
        light.energy = energy * (size / 0.45) ** 2
        light.size = radius * size / 0.45
        obj = bpy.data.objects.new(name, light)
        obj.location = center + Vector(offset) * (size / 0.45)
        direction = center - obj.location
        obj.rotation_euler = direction.to_track_quat("-Z", "Y").to_euler()
        scene.collection.objects.link(obj)


def setup_render(samples: int, fmt: str = "jpg") -> None:
    scene = bpy.context.scene
    scene.render.engine = "CYCLES"
    scene.cycles.device = "CPU"
    scene.cycles.samples = samples
    scene.cycles.use_adaptive_sampling = True
    scene.cycles.use_denoising = True
    scene.cycles.denoiser = "OPENIMAGEDENOISE"
    scene.render.film_transparent = True
    scene.render.image_settings.file_format = "JPEG" if fmt == "jpg" else "PNG"
    scene.render.image_settings.color_mode = "RGB"
    if fmt == "jpg":
        scene.render.image_settings.quality = 90
    scene.view_settings.view_transform = "AgX"
    for look in ("AgX - Medium High Contrast", "Medium High Contrast"):
        try:
            scene.view_settings.look = look
            break
        except TypeError:
            continue
    # Saydam film + gölge yakalayıcı → stüdyo arka planı üzerine birleştir
    scene.use_nodes = True
    tree = scene.node_tree
    tree.nodes.clear()
    layers = tree.nodes.new("CompositorNodeRLayers")
    rgb = tree.nodes.new("CompositorNodeRGB")
    rgb.outputs[0].default_value = (*BACKGROUND, 1.0)
    over = tree.nodes.new("CompositorNodeAlphaOver")
    out = tree.nodes.new("CompositorNodeComposite")
    tree.links.new(rgb.outputs[0], over.inputs[1])
    tree.links.new(layers.outputs["Image"], over.inputs[2])
    tree.links.new(over.outputs["Image"], out.inputs["Image"])


def camera_for(view: str, table: dict, lo: Vector, hi: Vector, scale: float) -> bpy.types.Object:
    az, el, target, radius, (w, h), fstop = table[view]
    scene = bpy.context.scene
    scene.render.resolution_x, scene.render.resolution_y = int(w * scale), int(h * scale)
    cam_data = bpy.data.cameras.new(f"kamera_{view}")
    cam_data.lens = 70.0
    cam_data.sensor_width = 36.0
    cam = bpy.data.objects.new(f"kamera_{view}", cam_data)
    scene.collection.objects.link(cam)
    center = Vector(target) if target else (lo + hi) / 2
    t_h = cam_data.sensor_width / 2 / cam_data.lens                     # yatay yarı görüş açısının tanjantı
    t_w, t_v = (t_h, t_h * h / w) if w >= h else (t_h * w / h, t_h)
    if el >= 89.0:
        fwd, right, up = Vector((0, 0, -1)), Vector((0, -1, 0)), Vector((1, 0, 0))     # burun görüntünün üstünde
    else:
        a, e = math.radians(az), math.radians(el)
        fwd = -Vector((math.cos(e) * math.cos(a), math.cos(e) * math.sin(a), math.sin(e)))
        right = fwd.cross(Vector((0, 0, 1))).normalized()
        up = right.cross(fwd)
    if radius:                                                         # yakın plan: hedef çevresinde küre
        dist = radius / math.atan(min(t_w, t_v)) * 1.04
    else:                                                              # tüm drone: kutu köşeleri kadraja sığsın
        margin = 1.08
        dist = 0.0
        for corner in [Vector((x, y, z)) for x in (lo.x, hi.x) for y in (lo.y, hi.y) for z in (lo.z, hi.z)]:
            d = corner - center
            depth = d.dot(fwd)
            dist = max(dist, abs(d.dot(right)) * margin / t_w - depth, abs(d.dot(up)) * margin / t_v - depth)
    cam.location = center - fwd * dist
    if el >= 89.0:
        cam.rotation_euler = (0.0, 0.0, math.radians(-90.0))
    else:
        cam.rotation_euler = fwd.to_track_quat("-Z", "Y").to_euler()
    if fstop:
        cam_data.dof.use_dof = True
        cam_data.dof.focus_distance = (center - cam.location).length
        cam_data.dof.aperture_fstop = fstop
    scene.camera = cam
    return cam


def main() -> int:
    args = parse_args()
    if not args.glb.exists():
        print(f"GLB yok: {args.glb} — önce python3 cad/build.py veya python3 cad/v2/build_v2.py", file=sys.stderr)
        return 1
    args.out.mkdir(parents=True, exist_ok=True)
    objs = import_model(args.glb)
    lo, hi = world_bounds(objs)
    dims = hi - lo
    print(f"Model: {len(objs)} nesne, {dims.x * 1000:.0f} × {dims.y * 1000:.0f} × {dims.z * 1000:.0f} mm")
    used = assign_materials(objs)
    print("Malzemeler:", ", ".join(f"{k} {n}" for k, n in sorted(used.items())))
    studio(lo, hi)
    setup_render(args.samples, args.format)
    floor = bpy.data.objects.get("zemin")
    for view in [v.strip() for v in args.views.split(",") if v.strip()]:
        camera_for(view, args.view_table, lo, hi, args.scale)
        if floor is not None:                                    # alttan bakışta zemin kamerayı örter
            floor.hide_render = args.view_table[view][1] < 0
        path = args.out / f"{view}.{args.format}"
        bpy.context.scene.render.filepath = str(path)
        t0 = time.time()
        bpy.ops.render.render(write_still=True)
        print(f"  {view}: {path} ({time.time() - t0:.0f} s)")
    if args.blend:
        bpy.ops.wm.save_as_mainfile(filepath=str(args.out / "dc7_studio.blend"))
    return 0


if __name__ == "__main__":
    sys.exit(main())
