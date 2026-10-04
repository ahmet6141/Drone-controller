"""YELKOVAN YK-38 — Blender yardımcıları (bpy 4.5).

``ucav.shapes`` ağlarını (``MeshData``) Blender nesnesine çevirir ve sözleşmedeki ortak kuralları uygular:

* **Ağ**: köşeler ``foreach_set`` ile hızlı yüklenir; yüz başına malzeme yuvası (``UM_*``; yoksa ``params``
  renginden basit Principled malzeme oluşturulur — materials modülü düğüm ağacını sonra doldurur), yumuşak
  gölgeleme + açıya göre keskin kenar (``Mesh.set_sharp_from_angle``) + açıkça işaretli keskin kenarlar.
* **Pivot ve yerel eksenler**: nesne orijini ``origin_b`` (Blender dünya), yerel eksenler ``frame_b`` (3×3,
  sütunlar X, Y, Z). Eksen takımı ``delta_rotation_euler``'e yazılır; ``rotation_euler`` dinlenmede (0, 0, 0)
  kalır → sürücü yalnızca ``rotation_euler[i]``'yi sürer ve dönüş nesnenin yerel ekseni etrafında olur.
* **Ebeveyn**: ``parent_keep_world`` — ebeveyn ters matrisi ``parent.matrix_world⁻¹`` (Ctrl+P "Object"
  davranışı); çocuğun dünya dönüşümü değişmez, ``location`` dünya konumunu gösterir.
* **Koleksiyonlar**: ``UCAV`` → ``UCAV_Airframe``, ``UCAV_Surfaces``, ``UCAV_Gear``, ``UCAV_Propulsion``,
  ``UCAV_Payload``, ``UCAV_Details``; ``UCAV_Print`` (render dışı), ``UCAV_Studio``, ``UCAV_Envelopes`` (render ve
  GLB dışı paketleme zarfları, tel kafes).
* **Boolean**: ``boolean_difference`` (EXACT çözücü, malzeme aktarımı, değiştirici uygulanır).
"""
from __future__ import annotations

import math
from typing import Iterable

import bpy
import numpy as np
from mathutils import Matrix, Vector

from .. import params as P
from ..shapes import MeshData

# Koleksiyon ağacı (sözleşme)
COLLECTIONS: dict[str, str | None] = {
    "UCAV": None,
    "UCAV_Airframe": "UCAV",
    "UCAV_Surfaces": "UCAV",
    "UCAV_Gear": "UCAV",
    "UCAV_Propulsion": "UCAV",
    "UCAV_Payload": "UCAV",
    "UCAV_Details": "UCAV",
    "UCAV_Print": None,
    "UCAV_Studio": None,
    "UCAV_Envelopes": None,      # paketleme zarfları (motor, susturucu, depo, akü): render ve GLB dışı, tel kafes
}
RENDER_EXCLUDED = ("UCAV_Print", "UCAV_Envelopes")


# =====================================================================================================
# Koleksiyonlar
# =====================================================================================================
def ensure_collection(name: str, parent: bpy.types.Collection | None = None,
                      scene: bpy.types.Scene | None = None) -> bpy.types.Collection:
    """Koleksiyonu döndürür; yoksa oluşturur ve ``parent``'a (yoksa sahne köküne) bağlar."""
    scene = scene or bpy.context.scene
    col = bpy.data.collections.get(name)
    if col is None:
        col = bpy.data.collections.new(name)
    parent = parent or scene.collection
    if col.name not in parent.children.keys():
        parent.children.link(col)
    return col


def ensure_collections(scene: bpy.types.Scene | None = None) -> dict[str, bpy.types.Collection]:
    """Sözleşmedeki bütün koleksiyonları kurar; ``UCAV_Print`` render'dan hariç tutulur."""
    scene = scene or bpy.context.scene
    out: dict[str, bpy.types.Collection] = {}
    for name, par in COLLECTIONS.items():
        out[name] = ensure_collection(name, out[par] if par else None, scene)
    for name in RENDER_EXCLUDED:
        out[name].hide_render = True
    return out


# =====================================================================================================
# Malzemeler
# =====================================================================================================
def get_material(name: str) -> bpy.types.Material:
    """``UM_*`` malzemesini döndürür; yoksa ``params.MATERIALS`` değerlerinden basit Principled oluşturur
    (materials modülü aynı adlı malzemeyi düğüm ağacıyla doldurur). Bilinmeyen ad → ``KeyError``."""
    mat = bpy.data.materials.get(name)
    if mat is not None:
        return mat
    spec = P.MATERIALS[name]
    mat = bpy.data.materials.new(name)
    mat.use_nodes = True
    mat.diffuse_color = spec.rgba_linear
    bsdf = mat.node_tree.nodes.get("Principled BSDF")
    if bsdf is not None:
        bsdf.inputs["Base Color"].default_value = spec.rgba_linear
        bsdf.inputs["Roughness"].default_value = spec.roughness
        bsdf.inputs["Metallic"].default_value = spec.metallic
        if spec.transmission > 0:
            bsdf.inputs["Transmission Weight"].default_value = spec.transmission
            bsdf.inputs["IOR"].default_value = spec.ior
        if spec.emission > 0:
            bsdf.inputs["Emission Color"].default_value = spec.rgba_linear
            bsdf.inputs["Emission Strength"].default_value = spec.emission
    return mat


# =====================================================================================================
# Ağ ve nesne
# =====================================================================================================
def to_local(verts_b: np.ndarray, origin_b=None, frame_b=None) -> np.ndarray:
    """Dünya (Blender) köşelerini nesne yerel koordinatına çevirir: ``Fᵀ (v − o)``."""
    V = np.asarray(verts_b, float)
    if origin_b is not None:
        V = V - np.asarray(origin_b, float)
    if frame_b is not None:
        V = V @ np.asarray(frame_b, float)
    return V


def mesh_from_data(md: MeshData, name: str | None = None, origin_b=None, frame_b=None) -> bpy.types.Mesh:
    """``MeshData`` → ``bpy.types.Mesh``. ``space="spec"`` ağlar önce Blender'a, sonra yerel eksene çevrilir;
    ``space="local"`` ağlar olduğu gibi kullanılır (zaten nesne yerel ekseninde)."""
    V = md.verts_blender()
    if md.space == "spec":
        V = to_local(V, origin_b, frame_b)
    name = name or md.name
    old = bpy.data.meshes.get(name)
    if old is not None and old.users == 0:
        bpy.data.meshes.remove(old)
    me = bpy.data.meshes.new(name)
    faces = md.faces
    totals = np.fromiter((len(f) for f in faces), dtype=np.int32, count=len(faces))
    starts = np.r_[0, np.cumsum(totals)[:-1]].astype(np.int32)
    loops = np.fromiter((i for f in faces for i in f), dtype=np.int32, count=int(totals.sum()))
    me.vertices.add(len(V))
    me.vertices.foreach_set("co", V.astype(np.float32).ravel())
    me.loops.add(len(loops))
    me.loops.foreach_set("vertex_index", loops)
    me.polygons.add(len(faces))
    me.polygons.foreach_set("loop_start", starts)
    me.update(calc_edges=True)
    me.validate(clean_customdata=False)
    for m in md.mats:
        me.materials.append(get_material(m))
    if len(md.face_mat) == len(me.polygons):
        me.polygons.foreach_set("material_index", np.asarray(md.face_mat, np.int32))
    apply_shading(me, md.smooth_angle, md.sharp_edges)
    return me


def apply_shading(me: bpy.types.Mesh, smooth_angle: float | None, sharp_edges: Iterable = ()) -> None:
    """Yumuşak gölgeleme + açıya göre keskin kenar (+ verilen kenarlar keskin); ``None`` → düz gölgeleme."""
    if smooth_angle is None:
        me.shade_flat()
        return
    me.shade_smooth()
    me.set_sharp_from_angle(angle=math.radians(smooth_angle))       # "sharp_edge" sıfırlanır ve doldurulur
    sharp_edges = list(sharp_edges)
    if sharp_edges:
        ev = np.zeros(len(me.edges) * 2, np.int32)
        me.edges.foreach_get("vertices", ev)
        ev = ev.reshape(-1, 2)
        key = {(int(min(a, b)), int(max(a, b))): i for i, (a, b) in enumerate(ev)}
        attr = me.attributes.get("sharp_edge") or me.attributes.new("sharp_edge", "BOOLEAN", "EDGE")
        vals = np.zeros(len(me.edges), bool)
        attr.data.foreach_get("value", vals)
        for a, b in sharp_edges:
            i = key.get((min(a, b), max(a, b)))
            if i is not None:
                vals[i] = True
        attr.data.foreach_set("value", vals)


def remove_object(name: str) -> None:
    """Varsa nesneyi (ve kullanılmayan ağını) siler — ``build`` tekrar çağrılabilsin diye."""
    ob = bpy.data.objects.get(name)
    if ob is None:
        return
    data = ob.data
    for ch in list(ob.children):
        ch.parent = None
    bpy.data.objects.remove(ob, do_unlink=True)
    if isinstance(data, bpy.types.Mesh) and data.users == 0:
        bpy.data.meshes.remove(data)


def new_object(name: str, data, collection: bpy.types.Collection, origin_b=(0.0, 0.0, 0.0),
               frame_b=None) -> bpy.types.Object:
    """Nesne oluşturur (aynı adlı eskisini siler). ``frame_b`` → ``delta_rotation_euler`` (yerel eksenler),
    ``rotation_euler`` = 0 (dinlenme)."""
    remove_object(name)
    ob = bpy.data.objects.new(name, data)
    collection.objects.link(ob)
    ob.location = Vector(tuple(map(float, origin_b)))
    ob.rotation_mode = "XYZ"
    ob.rotation_euler = (0.0, 0.0, 0.0)
    if frame_b is not None:
        ob.delta_rotation_euler = Matrix(np.asarray(frame_b, float).tolist()).to_euler("XYZ")
    return ob


def object_from_mesh(md: MeshData, collection: bpy.types.Collection, name: str | None = None, origin_b=(0.0, 0.0, 0.0),
                     frame_b=None) -> bpy.types.Object:
    """``MeshData``'dan nesne: köşeler ``origin_b``/``frame_b`` yerel eksenine çevrilir (dünya konumu korunur)."""
    name = name or md.name
    remove_object(name)
    me = mesh_from_data(md, name, origin_b, frame_b)
    ob = new_object(name, me, collection, origin_b, frame_b)
    ob["ucav_closed"] = bool(md.closed)
    return ob


def new_empty(name: str, collection: bpy.types.Collection, origin_b=(0.0, 0.0, 0.0), frame_b=None,
              display: str = "PLAIN_AXES", size: float = 0.05) -> bpy.types.Object:
    ob = new_object(name, None, collection, origin_b, frame_b)
    ob.empty_display_type = display
    ob.empty_display_size = size
    return ob


def world_matrix(ob: bpy.types.Object) -> Matrix:
    """Depsgraph güncellemesi beklemeden nesnenin dünya matrisi (konum, delta + dönüş, ölçek, ebeveyn)."""
    rot = ob.delta_rotation_euler.to_matrix() @ ob.rotation_euler.to_matrix()
    basis = Matrix.Translation(ob.location + ob.delta_location) @ rot.to_4x4()
    sc = Matrix.Diagonal((*[a * b for a, b in zip(ob.scale, ob.delta_scale)], 1.0))
    basis = basis @ sc
    if ob.parent is not None:
        return world_matrix(ob.parent) @ ob.matrix_parent_inverse @ basis
    return basis


def parent_keep_world(child: bpy.types.Object, parent: bpy.types.Object) -> None:
    """Çocuğu dünya dönüşümünü koruyarak bağlar: ``matrix_parent_inverse = parent_world⁻¹``."""
    child.parent = parent
    child.matrix_parent_inverse = world_matrix(parent).inverted()


# =====================================================================================================
# Boolean
# =====================================================================================================
def boolean_difference(target: bpy.types.Object, cutter: bpy.types.Object, apply: bool = True,
                       name: str = "UCAV_Cut") -> bpy.types.Modifier | None:
    """EXACT boolean farkı; kesicinin malzemesi yeni yüzlere aktarılır (kuyu duvarları turuncu). ``apply``
    True ise değiştirici değerlendirilip ağa yazılır (render hızlı, ağ kapalı kalır)."""
    mod = target.modifiers.new(name, "BOOLEAN")
    mod.operation = "DIFFERENCE"
    mod.solver = "EXACT"
    mod.object = cutter
    mod.material_mode = "TRANSFER"
    if not apply:
        return mod
    dg = bpy.context.evaluated_depsgraph_get()
    dg.update()
    ev = target.evaluated_get(dg)
    new = bpy.data.meshes.new_from_object(ev, preserve_all_data_layers=True, depsgraph=dg)
    old = target.data
    target.modifiers.remove(mod)
    new.name = old.name
    clean_degenerate(new)
    target.data = new
    if old.users == 0:
        bpy.data.meshes.remove(old)
    target.data.name = target.name
    return None


BOOL_JITTER = ((0.0, 0.0, 0.0), (0.0, 0.0, 2.1e-5), (1.3e-5, 0.0, -1.7e-5), (-1.1e-5, 0.9e-5, 2.9e-5),
               (2.3e-5, -1.2e-5, -3.1e-5))                  # sayısal çakışmada kesiciye uygulanan küçük kaydırmalar (m)


def boolean_difference_safe(target: bpy.types.Object, cutter: bpy.types.Object) -> int:
    """``boolean_difference`` + doğrulama: sonuç kapalı (sınır/manifold dışı kenar yok) değilse eski ağ geri yüklenir ve
    kesici mikrometre ölçeğinde kaydırılarak (``BOOL_JITTER``) yeniden denenir — EXACT çözücünün eş düzlemli /
    çakışık yüzlerde nadiren ürettiği bozuk sonuçlara karşı. Dönüş: kullanılan deneme indisi (bozuk kalırsa −1)."""
    base = cutter.location.copy()
    keep = target.data.copy()
    try:
        for k, d in enumerate(BOOL_JITTER):
            cutter.location = base + Vector(d)
            bpy.context.view_layer.update()
            boolean_difference(target, cutter)
            r = mesh_report(target)
            if not (r["boundary"] or r["nonmanifold"]):
                return k
            bad = target.data
            target.data = keep.copy()
            target.data.name = target.name
            if bad.users == 0:
                bpy.data.meshes.remove(bad)
        print(f"[util] uyarı: {target.name} ← {cutter.name} boolean sonucu kapalı değil")
        return -1
    finally:
        cutter.location = base
        if keep.users == 0:
            bpy.data.meshes.remove(keep)


def clean_degenerate(me: bpy.types.Mesh, dist: float = 1e-7) -> dict:
    """EXACT boolean artıklarını temizler: çakışık köşeler (``dist`` ≤ 0,1 µm) birleştirilir (bu köşelerle sıfır
    alanlı kalan yüzler çöker), aynı köşeleri paylaşan yüz çiftleri (sıfır hacimli "yastık") ve kopuk köşeler silinir.
    Kenar sayımı bunları göstermez ama ``Mesh.validate()`` (ör. glTF dışa aktarıcısı) onları silip delik bırakır."""
    import bmesh
    bm = bmesh.new()
    bm.from_mesh(me)
    n_v, n_f = len(bm.verts), len(bm.faces)
    bmesh.ops.remove_doubles(bm, verts=bm.verts, dist=dist)     # çakışık köşeli (sıfır alanlı) yüzler de çöker
    seen: dict = {}
    dup = set()
    for f in bm.faces:
        k = tuple(sorted(v.index for v in f.verts))
        if k in seen:
            dup.update((f, seen[k]))
        else:
            seen[k] = f
    if dup:
        bmesh.ops.delete(bm, geom=list(dup), context="FACES_ONLY")
    loose_e = [e for e in bm.edges if not e.link_faces]
    if loose_e:
        bmesh.ops.delete(bm, geom=loose_e, context="EDGES")
    loose_v = [v for v in bm.verts if not v.link_faces]
    if loose_v:
        bmesh.ops.delete(bm, geom=loose_v, context="VERTS")
    out = {"verts_removed": n_v - len(bm.verts), "faces_removed": n_f - len(bm.faces), "dup_pairs": len(dup) // 2}
    bm.to_mesh(me)
    bm.free()
    me.update()
    return out


def mesh_report(ob: bpy.types.Object) -> dict:
    """Blender ağının kenar raporu: sınır (1 yüz) ve manifold olmayan (>2 yüz) kenar sayısı."""
    me = ob.data
    ev = np.zeros(len(me.edges) * 2, np.int32)
    me.edges.foreach_get("vertices", ev)
    cnt = np.zeros(len(me.edges), np.int32)
    key = {}
    pairs = ev.reshape(-1, 2)
    for i, (a, b) in enumerate(pairs):
        key[(min(a, b), max(a, b))] = i
    for p in me.polygons:
        vs = list(p.vertices)
        for k in range(len(vs)):
            a, b = vs[k], vs[(k + 1) % len(vs)]
            cnt[key[(min(a, b), max(a, b))]] += 1
    return {"name": ob.name, "verts": len(me.vertices), "faces": len(me.polygons),
            "boundary": int((cnt == 1).sum()), "nonmanifold": int((cnt > 2).sum()),
            "materials": [m.name if m else None for m in me.materials]}


# =====================================================================================================
# Şablon yazılar ve boya işaretleri (ince kapalı katılar, ev sahibi yüzeye BVH izdüşümü)
# =====================================================================================================
FONT_CANDIDATES = (
    "/usr/share/fonts/truetype/liberation/LiberationSans-Bold.ttf",
    "/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf",
    "/usr/share/fonts/truetype/freefont/FreeSansBold.ttf",
    "C:/Windows/Fonts/arialbd.ttf",
    "/Library/Fonts/Arial Bold.ttf",
)
STENCIL_CONDENSE = 0.86                   # şablon yazı yatay daraltma
STENCIL_LIFT = 0.00035                    # üst yüzün deriden yüksekliği (m)
STENCIL_THICK = 0.00015                   # ince katı kalınlığı (m)
STENCIL_MAX_EDGE = 0.004                  # izdüşümden önce üçgen kenarı üst sınırı (m)


def _font():
    import os
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


def text_mesh_2d(text: str, height: float) -> tuple[np.ndarray, list]:
    """Metin → 2B üçgen ağ (m): büyük harf yüksekliği ``height``, merkez (0, 0), x okuma yönü."""
    import bmesh
    cu = bpy.data.curves.new("UCAV_tmp_stencil", "FONT")
    cu.body = text
    cu.font = _font()
    cu.size = 1.0
    cu.resolution_u = 4
    cu.fill_mode = "BOTH"
    cu.align_x = "CENTER"
    cu.align_y = "CENTER"
    cu.space_character = 1.06
    tmp = bpy.data.objects.new("UCAV_tmp_stencil", cu)
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
    caps = [c for c in text if c.isalpha() or c.isdigit()]
    lo, hi = co.min(0), co.max(0)
    h_ref = (hi[1] - lo[1]) if caps else 1.0
    c = 0.5 * (lo + hi)
    for v in bm.verts:
        v.co.x = (v.co.x - c[0]) / h_ref * STENCIL_CONDENSE * height
        v.co.y = (v.co.y - c[1]) / h_ref * height
        v.co.z = 0.0
    bmesh.ops.dissolve_degenerate(bm, dist=1e-7, edges=bm.edges[:])
    bmesh.ops.triangulate(bm, faces=bm.faces[:])
    for _ in range(4):
        long_edges = [e for e in bm.edges if e.calc_length() > STENCIL_MAX_EDGE]
        if not long_edges:
            break
        bmesh.ops.subdivide_edges(bm, edges=long_edges, cuts=1, use_grid_fill=False)
        bmesh.ops.triangulate(bm, faces=bm.faces[:], quad_method="BEAUTY", ngon_method="BEAUTY")
        bmesh.ops.dissolve_degenerate(bm, dist=1e-7, edges=bm.edges[:])
        bmesh.ops.triangulate(bm, faces=bm.faces[:])
    bm.verts.index_update()
    V = np.array([v.co[:2] for v in bm.verts])
    F = [[v.index for v in f.verts] for f in bm.faces]
    bm.free()
    return V, F


def _host_bvh_world(ob: bpy.types.Object):
    from mathutils.bvhtree import BVHTree
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


def stencil_object(name: str, host: bpy.types.Object, V2: np.ndarray, F: list, center_b, u_b, n_b, material: str,
                   collection: bpy.types.Collection, bvh=None, lift: float = STENCIL_LIFT,
                   thick: float = STENCIL_THICK) -> bpy.types.Object | None:
    """2B işaret ağını (m; x = ``u_b``, y = ``n_b × u_b``) ``center_b`` çevresinde ev sahibi yüzeye ``−n_b`` yönünde
    izdüşürür, ``lift`` kadar kaldırır, ``thick`` kalınlığında kapalı katıya çevirir ve ev sahibine dünya korunarak
    bağlar. Yüzeyi ıskalayan nokta %2'yi aşarsa None (işaret kurulmaz)."""
    n = Vector(tuple(map(float, n_b))).normalized()
    u = Vector(tuple(map(float, u_b)))
    u = (u - n * u.dot(n)).normalized()
    v = n.cross(u)
    c = Vector(tuple(map(float, center_b)))
    bvh = bvh or _host_bvh_world(host)
    hit = bvh.ray_cast(c + n * 0.05, -n, 0.2)
    if hit[0] is None:
        return None
    c = hit[0]
    pts, nrm, miss = [], [], 0
    for x, y in np.asarray(V2, float):
        p = c + u * float(x) + v * float(y)
        h = bvh.ray_cast(p + n * 0.02, -n, 0.06)
        if h[0] is None:
            miss += 1
            pts.append(p)
            nrm.append(n)
            continue
        nn = h[1] if h[1].dot(n) >= 0 else -h[1]
        pts.append(h[0] + nn * lift)
        nrm.append(nn)
    if miss > 0.02 * len(pts):
        return None
    P3 = np.array([tuple(p) for p in pts])
    N3 = np.array([tuple(q) for q in nrm])
    F = [list(f) for f in F]
    if F:
        a, b, cc = (P3[i] for i in F[0][:3])
        if np.dot(np.cross(b - a, cc - a), np.asarray(n)) < 0:
            F = [f[::-1] for f in F]
    nv = len(P3)
    Vb = P3 - N3 * thick
    out_f = [tuple(f) for f in F] + [tuple(nv + i for i in f[::-1]) for f in F]
    cnt: dict = {}
    directed = []
    for f in F:
        for k in range(len(f)):
            e0, e1 = f[k], f[(k + 1) % len(f)]
            cnt[(min(e0, e1), max(e0, e1))] = cnt.get((min(e0, e1), max(e0, e1)), 0) + 1
            directed.append((e0, e1))
    for e0, e1 in directed:
        if cnt[(min(e0, e1), max(e0, e1))] == 1:
            out_f.append((e0, nv + e0, nv + e1, e1))
    remove_object(name)
    me = bpy.data.meshes.new(name)
    me.from_pydata([tuple(p) for p in np.vstack([P3, Vb])], [], out_f)
    me.validate()
    me.materials.append(get_material(material))
    me.shade_smooth()
    ob = bpy.data.objects.new(name, me)
    collection.objects.link(ob)
    parent_keep_world(ob, host)
    ob["ucav_closed"] = True
    ob.visible_shadow = False
    return ob
