"""Build the YK-250 Blender scene from the part registry (ARCHITECTURE.md §7).

* One collection per group under ``YK250``; ``YK250_Rig`` holds the control object and joint empties.
* Object name = part id. Custom properties: name_tr, material, process, thickness_mm, mass_kg, step, vendor.
* Joints: ``J_<name>`` (fixed frame at the joint origin, local X = joint axis) and ``J_<name>_m`` (moving child,
  driven: rotation X for revolute, location X for prismatic). Moving parts are children of ``J_<name>_m``.
* Control object ``YK250_Root`` carries one custom property per joint ``prop`` plus ``explode`` (0..1) and
  ``step`` (assembly step, parts with ``Part.step > step`` are hidden). All drivers are simple expressions.
* Previews: Workbench renders (needs Mesa EGL headless: ``bash ucav250/setup_env.sh``).

CLI:  python3 -m ucav250.blender.build [--previews] [--no-blend] [--glb]
"""
from __future__ import annotations

import argparse
import math
import sys
import time
from pathlib import Path

import bpy
import numpy as np
from mathutils import Matrix, Vector

from ..core.parts import GROUPS, Registry, layup_props

ROOT_NAME = "YK250_Root"
DEFAULT_COLORS = {  # group -> viewport colour (sRGB hex) if spec.display.colors has no entry for the part colour
    "chassis": "#8A9099", "shell": "#D8DCDF", "wing": "#D8DCDF", "tail": "#D8DCDF", "controls": "#B9C2C9",
    "propulsion": "#3B3F44", "fuel": "#C9A227", "gear": "#5C6670", "systems": "#3E7CB1", "payload": "#2B2F33",
    "hardware": "#A7ABAE",
}


def _hex(h: str) -> tuple[float, float, float, float]:
    h = h.lstrip("#")
    r, g, b = (int(h[i:i + 2], 16) / 255.0 for i in (0, 2, 4))
    lin = [c / 12.92 if c <= 0.04045 else ((c + 0.055) / 1.055) ** 2.4 for c in (r, g, b)]
    return (*lin, 1.0)


def reset_scene() -> None:
    bpy.ops.wm.read_factory_settings(use_empty=True)


def _collection(name: str, parent=None):
    col = bpy.data.collections.get(name) or bpy.data.collections.new(name)
    par = parent or bpy.context.scene.collection
    if col.name not in par.children:
        par.children.link(col)
    return col


def _material(key: str, hexcol: str):
    name = f"YK_{key}"
    mat = bpy.data.materials.get(name)
    if mat is None:
        mat = bpy.data.materials.new(name)
        mat.diffuse_color = _hex(hexcol)
        mat.roughness = 0.6
        mat.use_nodes = True
        bsdf = mat.node_tree.nodes.get("Principled BSDF")
        if bsdf is not None:
            bsdf.inputs["Base Color"].default_value = _hex(hexcol)
            bsdf.inputs["Roughness"].default_value = 0.55
    return mat


def _mesh_object(name: str, V: np.ndarray, F: np.ndarray, col, mat, smooth_deg: float = 32.0):
    me = bpy.data.meshes.new(name)
    me.from_pydata(V.tolist(), [], F.tolist())
    me.validate(clean_customdata=False)
    me.update()
    if smooth_deg > 0:
        me.shade_smooth()
        try:
            me.set_sharp_from_angle(angle=math.radians(smooth_deg))
        except Exception:  # pragma: no cover - older Blender
            pass
    me.materials.append(mat)
    ob = bpy.data.objects.new(name, me)
    col.objects.link(ob)
    return ob


def _add_prop(ob, name: str, value, lo=None, hi=None, desc: str = ""):
    ob[name] = value
    ui = ob.id_properties_ui(name)
    kw = {"description": desc}
    if lo is not None:
        kw.update(min=lo, max=hi, soft_min=lo, soft_max=hi)
    try:
        ui.update(**kw)
    except TypeError:  # pragma: no cover
        pass


def _driver(target, path: str, index: int, expr: str, variables: dict[str, tuple]):
    """variables: name -> (id_object, data_path)."""
    fc = target.driver_add(path, index) if index >= 0 else target.driver_add(path)
    drv = fc.driver
    drv.type = "SCRIPTED"
    for vn, (idb, dp) in variables.items():
        v = drv.variables.new()
        v.name = vn
        v.type = "SINGLE_PROP"
        v.targets[0].id = idb
        v.targets[0].data_path = dp
    drv.expression = expr
    return fc


def _frame_matrix(origin, axis) -> Matrix:
    a = np.asarray(axis, float)
    a = a / np.linalg.norm(a)
    ref = np.array([0.0, 0.0, 1.0]) if abs(a[2]) < 0.9 else np.array([0.0, 1.0, 0.0])
    y = np.cross(ref, a)
    y /= np.linalg.norm(y)
    z = np.cross(a, y)
    M = Matrix.Identity(4)
    for i in range(3):
        M[i][0], M[i][1], M[i][2], M[i][3] = a[i], y[i], z[i], float(origin[i])
    return M


def build_scene(reg: Registry, verbose: bool = True) -> dict:
    t0 = time.time()
    spec = reg.spec
    scene = bpy.context.scene
    scene.unit_settings.system = "METRIC"
    scene.unit_settings.length_unit = "MILLIMETERS"
    root_col = _collection("YK250")
    cols = {g: _collection(f"YK250_{g.capitalize()}", root_col) for g in GROUPS}
    rig_col = _collection("YK250_Rig", root_col)
    colors = (spec.get("display", {}) or {}).get("colors", {}) or {}

    # control object at the design CG (or origin)
    root = bpy.data.objects.new(ROOT_NAME, None)
    root.empty_display_type = "ARROWS"
    root.empty_display_size = 0.3
    cg = (spec.get("stability", {}) or {}).get("cg_design")
    if cg:
        root.location = Vector(cg)
    rig_col.objects.link(root)
    props_done = set()
    for j in reg.joints.values():
        if j.prop and j.prop not in props_done:
            props_done.add(j.prop)
            lo, hi = sorted((j.lo / j.scale, j.hi / j.scale)) if j.scale else (0.0, 1.0)
            _add_prop(root, j.prop, float(j.rest / j.scale) if j.scale else 0.0, lo, hi, f"joint {j.name}")
    max_step = max([p.step for p in reg.parts.values()] + [0])
    _add_prop(root, "explode", 0.0, 0.0, 1.0, "exploded view factor")
    _add_prop(root, "step", int(max_step), 0, int(max_step), "assembly step: parts with a higher step are hidden")

    # joints
    jobj = {}
    for j in reg.joints.values():                       # registry preserves insertion order (parents first)
        fixed = bpy.data.objects.new(f"J_{j.name}", None)
        fixed.empty_display_type = "SINGLE_ARROW"
        fixed.empty_display_size = 0.08
        rig_col.objects.link(fixed)
        Mw = _frame_matrix(j.origin, j.axis)
        if j.parent:
            par = jobj[j.parent]
            bpy.context.view_layer.update()
            fixed.parent = par
            fixed.matrix_parent_inverse = Matrix.Identity(4)
        fixed.matrix_world = Mw
        mov = bpy.data.objects.new(f"J_{j.name}_m", None)
        mov.empty_display_type = "PLAIN_AXES"
        mov.empty_display_size = 0.04
        rig_col.objects.link(mov)
        mov.parent = fixed
        mov.matrix_parent_inverse = Matrix.Identity(4)
        bpy.context.view_layer.update()
        if j.prop or j.expr:
            expr = j.expr or f"{j.scale!r}*p-({j.rest!r})"
            variables = {"p": (root, f'["{j.prop}"]')} if j.prop else {}
            for name in sorted({jj.prop for jj in reg.joints.values() if jj.prop}):
                if j.expr and name in j.expr:
                    variables[name] = (root, f'["{name}"]')
            if j.kind == "revolute":
                _driver(mov, "rotation_euler", 0, expr, variables)
            else:
                _driver(mov, "location", 0, expr, variables)
        jobj[j.name] = mov
    bpy.context.view_layer.update()

    # parts
    n_obj = 0
    n_tri = 0
    for p in reg.parts.values():
        if p.process == "consumable":
            continue
        mesh = p.mesh
        key = p.color or p.group
        mat = _material(key, colors.get(key, DEFAULT_COLORS.get(p.group, "#C0C0C0")))
        ob = _mesh_object(p.id, mesh.V, mesh.F, cols[p.group], mat, smooth_deg=0.0 if p.group == "hardware" else 32)
        n_obj += 1
        n_tri += len(mesh.F)
        t = p.thickness if p.thickness is not None else (layup_props(spec, p.layup)["thickness"] if p.layup else None)
        for k, v in (("name_tr", p.name_tr), ("material", p.material), ("process", p.process),
                     ("thickness_mm", -1.0 if t is None else round(t * 1000, 3)), ("mass_kg", round(reg.mass(p), 4)),
                     ("step", int(p.step)), ("vendor", p.vendor)):
            ob[k] = v
        if p.joint:
            mov = jobj[p.joint]
            ob.parent = mov
            # parent inverse = rest pose of the joint frame -> the object's basis stays world-aligned at rest, so the
            # exploded offset below is a world vector (it then follows the joint when the joint moves)
            ob.matrix_parent_inverse = mov.matrix_world.inverted()
        ex = Vector(p.explode)
        if ex.length > 0:
            exl = ex
            for i in range(3):
                if abs(exl[i]) > 1e-9:
                    _driver(ob, "delta_location", i, f"e*{exl[i]!r}", {"e": (root, '["explode"]')})
        if p.step > 0:
            for path in ("hide_viewport", "hide_render"):
                _driver(ob, path, -1, f"s<{int(p.step)}", {"s": (root, '["step"]')})
    bpy.context.view_layer.update()
    info = {"objects": n_obj, "triangles": n_tri, "joints": len(reg.joints), "seconds": round(time.time() - t0, 1)}
    if verbose:
        print(f"[yk250 build] {info}")
    return info


def set_controls(**values) -> None:
    """Set control properties on YK250_Root and re-evaluate drivers (ID-property writes do not tag the depsgraph)."""
    root = bpy.data.objects[ROOT_NAME]
    for k, v in values.items():
        root[k] = v
    root.update_tag()
    sc = bpy.context.scene
    sc.frame_set(sc.frame_current)
    bpy.context.view_layer.update()


# =====================================================================================================================
# previews (Workbench)
# =====================================================================================================================
VIEWS = {  # name -> (direction from target to camera, up, ortho)
    "front": ((-1, 0, 0), (0, 0, 1), True),
    "side": ((0, -1, 0), (0, 0, 1), True),
    "top": ((0, 0, 1), (-1, 0, 0), True),
    "iso": ((-1.0, -1.15, 0.75), (0, 0, 1), False),
    "iso_rear": ((1.1, 1.0, 0.6), (0, 0, 1), False),
    "under": ((-0.3, -0.5, -1.0), (0, 0, 1), False),
}


def _scene_points(max_per_object: int = 3000) -> np.ndarray:
    """World-space vertex sample of every visible mesh object (evaluated, so drivers/explode are applied)."""
    dg = bpy.context.evaluated_depsgraph_get()
    pts = []
    for ob in bpy.context.scene.objects:
        if ob.type != "MESH" or ob.hide_render or ob.hide_get():
            continue
        ev = ob.evaluated_get(dg)
        me = ev.to_mesh()
        n = len(me.vertices)
        if n:
            co = np.empty(n * 3)
            me.vertices.foreach_get("co", co)
            co = co.reshape(-1, 3)
            if n > max_per_object:
                co = co[:: int(math.ceil(n / max_per_object))]
            M = np.array(ev.matrix_world)
            pts.append(co @ M[:3, :3].T + M[:3, 3])
        ev.to_mesh_clear()
    return np.vstack(pts) if pts else np.zeros((1, 3))


def setup_workbench(res=(1600, 1000)):
    sc = bpy.context.scene
    sc.render.engine = "BLENDER_WORKBENCH"
    sc.render.resolution_x, sc.render.resolution_y = res
    sc.render.resolution_percentage = 100
    sh = sc.display.shading
    sh.light = "STUDIO"
    sh.color_type = "MATERIAL"
    sh.show_cavity = True
    sh.cavity_type = "BOTH"
    sh.show_object_outline = True
    sh.show_shadows = False
    sc.display.render_aa = "8"
    sc.render.film_transparent = False
    sc.world = sc.world or bpy.data.worlds.new("YK_World")
    sc.world.color = (0.82, 0.84, 0.86)
    sc.render.image_settings.file_format = "PNG"


def render_view(name: str, out_path: Path, direction, up, ortho: bool, margin: float = 1.06):
    """Aim a camera along ``-direction`` and fit the projected vertex cloud (shift + scale/distance) with ``margin``."""
    P = _scene_points()
    c = 0.5 * (P.min(0) + P.max(0))
    R = 0.5 * float(np.linalg.norm(P.max(0) - P.min(0))) + 1e-6
    cam_data = bpy.data.cameras.get("YK_Cam") or bpy.data.cameras.new("YK_Cam")
    cam = bpy.data.objects.get("YK_Cam") or bpy.data.objects.new("YK_Cam", cam_data)
    if cam.name not in bpy.context.scene.collection.objects:
        bpy.context.scene.collection.objects.link(cam)
    d = np.asarray(direction, float)
    d /= np.linalg.norm(d)
    x = np.cross(np.asarray(up, float), d)
    x /= np.linalg.norm(x)
    y = np.cross(d, x)
    sc = bpy.context.scene
    aspect = sc.render.resolution_x / sc.render.resolution_y
    cam_data.shift_x = cam_data.shift_y = 0.0
    if ortho:
        cam_data.type = "ORTHO"
        dist = 4.0 * R + 1.0
        loc = c + d * dist
        u, w = (P - loc) @ x, (P - loc) @ y
        wid, hei = u.max() - u.min(), w.max() - w.min()
        scale = margin * max(wid, hei * aspect)
        cam_data.ortho_scale = scale
        cam_data.shift_x = 0.5 * (u.max() + u.min()) / scale
        cam_data.shift_y = 0.5 * (w.max() + w.min()) / scale
    else:
        cam_data.type = "PERSP"
        cam_data.lens, cam_data.sensor_width, cam_data.sensor_fit = 50.0, 36.0, "HORIZONTAL"
        tan_h = 18.0 / 50.0
        dist = R / tan_h * 1.2
        for _ in range(4):
            loc = c + d * dist
            rel = P - loc
            zc = -(rel @ d)                         # depth in front of the camera (> 0)
            u = (rel @ x) / zc
            w = (rel @ y) / zc
            half = max(u.max() - u.min(), (w.max() - w.min()) * aspect) / 2
            dist *= margin * half / tan_h
        loc = c + d * dist
        rel = P - loc
        zc = -(rel @ d)
        u, w = (rel @ x) / zc, (rel @ y) / zc
        cam_data.shift_x = 0.5 * (u.max() + u.min()) / (2 * tan_h)
        cam_data.shift_y = 0.5 * (w.max() + w.min()) / (2 * tan_h)
    cam.matrix_world = Matrix(((x[0], y[0], d[0], loc[0]), (x[1], y[1], d[1], loc[1]),
                               (x[2], y[2], d[2], loc[2]), (0, 0, 0, 1)))
    cam_data.clip_start = 0.01
    cam_data.clip_end = dist + 4 * R
    sc.camera = cam
    sc.render.filepath = str(out_path)
    bpy.ops.render.render(write_still=True)


def render_previews(spec: dict, out_dir: Path, res=(1600, 1000)) -> list[str]:
    """Static views at rest, an exploded iso view and every state in spec.display.preview_states:
    ``[{name, view, set: {prop: value}}]``."""
    out_dir.mkdir(parents=True, exist_ok=True)
    setup_workbench(res)
    root = bpy.data.objects[ROOT_NAME]
    files = []
    for name, (d, up, ortho) in VIEWS.items():
        p = out_dir / f"{name}.png"
        render_view(name, p, d, up, ortho)
        files.append(str(p))
    set_controls(explode=1.0)
    p = out_dir / "exploded_iso.png"
    render_view("exploded", p, VIEWS["iso"][0], VIEWS["iso"][1], False, margin=1.04)
    files.append(str(p))
    set_controls(explode=0.0)
    for st in (spec.get("display", {}) or {}).get("preview_states", []) or []:
        if not isinstance(st, dict):            # descriptive entry only (no {name, view, set}): nothing to pose
            continue
        saved = {k: root[k] for k in st.get("set", {})}
        set_controls(**st.get("set", {}))
        d, up, ortho = VIEWS[st.get("view", "iso")]
        p = out_dir / f"{st['name']}.png"
        render_view(st["name"], p, d, up, ortho)
        files.append(str(p))
        set_controls(**saved)
    return files


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description="YK-250 Blender scene")
    ap.add_argument("--previews", action="store_true")
    ap.add_argument("--no-blend", action="store_true")
    ap.add_argument("--glb", action="store_true")
    ap.add_argument("--modules", default="", help="build only these producer modules (comma list; hardware appended)")
    ap.add_argument("--out", default="", help="output folder (default ucav250/out); previews go to <out>/previews")
    a = ap.parse_args(argv)
    from ..core.assemble import build_registry
    from ..core.spec import OUT_DIR
    out_dir = Path(a.out) if a.out else OUT_DIR
    out_dir.mkdir(parents=True, exist_ok=True)
    mods = [m for m in a.modules.split(",") if m]
    if mods and "hardware" not in mods:
        mods.append("hardware")
    reset_scene()
    reg = build_registry(modules=mods or None, strict=not mods)
    info = build_scene(reg)
    if a.previews:
        info["previews"] = render_previews(reg.spec, out_dir / "previews")
    if not a.no_blend:
        bpy.ops.wm.save_as_mainfile(filepath=str(out_dir / "yk250.blend"), compress=True)
    if a.glb:
        bpy.ops.export_scene.gltf(filepath=str(out_dir / "yk250.glb"), export_format="GLB", use_selection=False)
    print(info)
    return 0


if __name__ == "__main__":
    sys.exit(main())
