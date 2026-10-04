"""YELKOVAN YK-38 — Blender sahnesinin uçtan uca testleri (``bpy`` yoksa atlanır).

Sahne bir kez kurulur (``ucav.blender.build.build_scene``: gövde → takım → rig → malzemeler → stüdyo →
animasyon, ≈ 25 s). Denetlenenler: sözleşmedeki nesneler ve koleksiyonlar, kontrol paneli özellikleri, bütün
sürücülerin geçerli ve "basit ifade" olması, her ağın UM_* malzemeli ve kapalı (manifold) olması, takım açıkken
tekerlerin zemine değmesi, takım çevriminde çakışma olmaması, kumanda yüzeyi işaret kuralı, ana ölçülerin
spec'e %1 içinde uyması, şablon yazılar, görünüm kuralları (yan görünüş göz hizasında perspektif, kadrajlar,
malzeme değerleri, panel çizgileri = baskı ekleri, kuyu/kapaklarda turuncu yok, CG işareti, dolgu ışığı sürücüsü,
pozlama), pervane diski, kumanda bağlantıları, pervane açısı = ∫rpm·dt, döngü dikişi, taret LED'i, ebeveyn kuralı,
takım/teker zamanlaması, başka dosyaya ekleme (append), animasyon klipleri, GLB ve küçük bir render. Baskı: küçük
bir segment alt kümesi geçici dizine kurulur (manifold + tablaya sığma) ve depodaki STL'ler/rapor numpy ile yeniden
denetlenir (tek kabuk, tabla teması, destek sınıfı, ısı kuralı, yük yolları, menteşe pimleri, tolerans kuponu, kalıp
parçaları).

Çalıştırma: ``python3 -m unittest tests.test_ucav_blender -v`` (≈ 2 dk).
"""
from __future__ import annotations

import importlib.util
import json
import math
import struct
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
HAVE_BPY = importlib.util.find_spec("bpy") is not None
OUT = ROOT / "ucav" / "out"

CONTRACT_OBJECTS = [
    "U_Root", "U_Fuselage", "U_WingCenter_L", "U_WingCenter_R", "U_WingOuter_L", "U_WingOuter_R", "U_Tip_L", "U_Tip_R",
    "U_Stab", "U_Fin_L", "U_Fin_R", "U_Cowl", "U_Intake", "U_ExhaustRing", "U_Hatch",
    "U_Aileron_L", "U_Aileron_R", "U_FlapIn_L", "U_FlapIn_R", "U_FlapOut_L", "U_FlapOut_R",
    "U_Elevator_L", "U_Elevator_R", "U_Rudder_L", "U_Rudder_R",
    "U_Prop", "U_Spinner", "U_Turret_Mount", "U_Turret_Pan", "U_Turret_Tilt", "U_Turret_Window",
    "U_Bay_N", "U_Bay_L", "U_Bay_R", "U_Door_N_1", "U_Door_N_2", "U_Door_L_1", "U_Door_R_1", "U_Door_L_2", "U_Door_R_2",
    "U_Pitot", "U_Light_Nav_L", "U_Light_Nav_R", "U_Light_Strobe_L", "U_Light_Strobe_R", "U_Light_Landing",
    "U_Light_Turret_Ring", "U_Door_N_3", "U_Cowl_Louvers", "U_Exhaust_Muffler", "U_Fairing_FinRoot_L",
    "U_Fairing_FinRoot_R", "U_Hatch_Frame", "U_PropDisc", "U_Fairing_Servo_Rudder_L", "U_Fairing_Servo_Rudder_R",
    "U_Cowl_Cavity", "U_ScuffPad",
] + [f"U_{k}_{leg}" for leg in ("N", "L", "R") for k in ("GearPivot", "GearStrut", "GearWheel", "GearSlider", "GearUnit")] \
  + ["U_GearSteer_N"] \
  + [f"U_{k}_{s}_{side}" for k in ("Horn", "ServoArm", "Pushrod") for s in ("Aileron", "FlapOut", "Elevator", "Rudder")
     for side in ("L", "R")]
COLLECTIONS = ["UCAV", "UCAV_Airframe", "UCAV_Surfaces", "UCAV_Gear", "UCAV_Propulsion", "UCAV_Payload", "UCAV_Details",
               "UCAV_Print", "UCAV_Studio"]
VIEWS = ["hero", "rear34", "side", "front", "top", "under", "nose", "tail", "gearbay"]

# Başka bir dosyaya ``UCAV`` koleksiyonunu ekleme (append) denetimi — ayrı Python sürecinde çalışır
# (argümanlar: kaynak .blend, U_Root X, U_Root'un zeminden yüksekliği).
_APPEND_CHECK = r'''
import sys, bpy
src, x0, h0 = sys.argv[-3], float(sys.argv[-2]), float(sys.argv[-1])
bpy.ops.wm.read_factory_settings(use_empty=True)
with bpy.data.libraries.load(src, link=False) as (df, dt):
    dt.collections = ["UCAV"]
bpy.context.scene.collection.children.link(dt.collections[0])
assert len(bpy.data.scenes) == 1, [s.name for s in bpy.data.scenes]
assert not any(c.name in ("UCAV_Studio", "UCAV_Print") for c in bpy.data.collections)
bad = [o.name for o in bpy.data.objects if o.animation_data for fc in o.animation_data.drivers
       if not (fc.is_valid and fc.driver.is_valid)]
assert not bad, bad
root = bpy.data.objects["U_Root"]
if root.animation_data is not None:
    root.animation_data.action = None
root["gear"] = 1.0
root["ground_z"] = 0.0                                   # kendi zemininiz: Z = 0
root.location = (x0, 0.0, h0)
root.rotation_euler = (0.0, 0.0, 0.0)
root.update_tag()
bpy.context.view_layer.update()
dg = bpy.context.evaluated_depsgraph_get()
for leg in "NLR":
    ev = bpy.data.objects["U_GearWheel_" + leg].evaluated_get(dg)
    me = ev.to_mesh()
    z = min((ev.matrix_world @ v.co).z for v in me.vertices)
    ev.to_mesh_clear()
    assert abs(z) < 5e-4, (leg, z)
print("APPEND_OK")
'''


def _read_stl(path: Path):
    data = path.read_bytes()
    n = struct.unpack("<I", data[80:84])[0]
    rec = np.frombuffer(data[84:84 + 50 * n], dtype=[("n", "<f4", 3), ("v", "<f4", (3, 3)), ("a", "<u2")])
    P3 = rec["v"].reshape(-1, 3)
    V, inv = np.unique(P3, axis=0, return_inverse=True)
    return V.astype(float), inv.reshape(-1, 3)


def _closed(T: np.ndarray) -> bool:
    """Her yönlü kenar tam bir kez ve ters eşiyle (kapalı, tutarlı yönlü yüzey)."""
    E = np.concatenate([T[:, [0, 1]], T[:, [1, 2]], T[:, [2, 0]]]).astype(np.int64)
    key = E[:, 0] * (1 << 32) + E[:, 1]
    rkey = E[:, 1] * (1 << 32) + E[:, 0]
    u, cnt = np.unique(key, return_counts=True)
    return bool((cnt == 1).all() and np.isin(rkey, key).all())


def _volume(V: np.ndarray, T: np.ndarray) -> float:
    a, b, c = V[T[:, 0]], V[T[:, 1]], V[T[:, 2]]
    return float(np.einsum("ij,ij->i", a, np.cross(b, c)).sum() / 6.0)


def _shell_labels(T: np.ndarray, nv: int) -> np.ndarray:
    """Üçgen başına bağlı bileşen (kabuk) etiketi: köşe paylaşımıyla etiket yayma + işaretçi atlama (numpy)."""
    lab = np.arange(nv)
    while True:
        m = np.minimum(np.minimum(lab[T[:, 0]], lab[T[:, 1]]), lab[T[:, 2]])
        new = lab.copy()
        for k in range(3):
            np.minimum.at(new, T[:, k], m)
        np.minimum.at(new, lab, new)
        new = new[new]
        if np.array_equal(new, lab):
            return lab[T[:, 0]]
        lab = new


def _bed_contact_cm2(V: np.ndarray, T: np.ndarray, tol: float = 0.05) -> float:
    """Tablaya (z_min) ``tol`` mm içinde yatan, aşağı bakan (n_z < −0,999) üçgenlerin alanı (cm²)."""
    z0 = V[:, 2].min()
    a, b, c = V[T[:, 0]], V[T[:, 1]], V[T[:, 2]]
    n = np.cross(b - a, c - a)
    area = 0.5 * np.linalg.norm(n, axis=1)
    nz = n[:, 2] / np.maximum(2 * area, 1e-30)
    low = (np.abs(a[:, 2] - z0) < tol) & (np.abs(b[:, 2] - z0) < tol) & (np.abs(c[:, 2] - z0) < tol)
    return float(area[low & (nz < -0.999)].sum() / 100.0)


@unittest.skipUnless(HAVE_BPY, "bpy kurulu değil")
class TestYK38Scene(unittest.TestCase):
    """Tam sahne (bir kez kurulur)."""

    @classmethod
    def setUpClass(cls):
        import bpy

        from ucav.blender import animation, build
        bpy.ops.wm.read_factory_settings(use_empty=True)
        cls.info = build.build_scene(verbose=False)
        animation.clear()                                      # ölçümler dinlenme pozunda

    # ------------------------------------------------------------------ yardımcılar
    @staticmethod
    def _world_verts(ob) -> np.ndarray:
        import bpy
        dg = bpy.context.evaluated_depsgraph_get()
        ev = ob.evaluated_get(dg)
        me = ev.to_mesh()
        V = np.empty(len(me.vertices) * 3)
        me.vertices.foreach_get("co", V)
        V = V.reshape(-1, 3)
        M = np.array(ev.matrix_world)
        ev.to_mesh_clear()
        return V @ M[:3, :3].T + M[:3, 3]

    def _aircraft_meshes(self):
        import bpy
        return [o for o in bpy.data.collections["UCAV"].all_objects if o.type == "MESH" and not o.hide_render]

    def _set(self, **kw):
        from ucav.blender import rig
        rig.set_controls(**kw)

    def tearDown(self):
        from ucav.blender import rig
        rig.set_controls(gear=1.0, gear_doors=0.0, aileron_deg=0.0, flap_deg=0.0, elevator_deg=0.0, rudder_deg=0.0)

    # ------------------------------------------------------------------ sözleşme
    def test_contract_objects_and_collections(self):
        import bpy
        missing = [n for n in CONTRACT_OBJECTS if bpy.data.objects.get(n) is None]
        self.assertEqual(missing, [])
        for c in COLLECTIONS:
            self.assertIsNotNone(bpy.data.collections.get(c), c)
        self.assertTrue(bpy.data.collections["UCAV_Print"].hide_render)
        dup = [o.name for o in bpy.data.objects if o.name.startswith("U_") and ".0" in o.name]
        self.assertEqual(dup, [], "yinelenen (.001) nesne")

    def test_everything_hangs_from_root(self):
        import bpy
        root = bpy.data.objects["U_Root"]
        for ob in bpy.data.collections["UCAV"].all_objects:
            if ob is root:
                continue
            p = ob
            while p.parent is not None:
                p = p.parent
            self.assertIs(p, root, ob.name)
        from ucav import params as P
        np.testing.assert_allclose(tuple(root.matrix_world.translation), P.U_ROOT_B, atol=1e-9)

    def test_control_panel_properties(self):
        import bpy

        from ucav.blender import rig
        root = bpy.data.objects["U_Root"]
        for name, default, lo, hi, desc in rig.PROPS:
            self.assertIn(name, root.keys())
            ui = root.id_properties_ui(name).as_dict()
            self.assertAlmostEqual(ui["min"], lo)
            self.assertAlmostEqual(ui["max"], hi)
            self.assertTrue(ui["description"])
        for name in ("gear", "gear_doors", "aileron_deg", "flap_deg", "elevator_deg", "rudder_deg", "prop_rpm",
                     "turret_pan_deg", "turret_tilt_deg", "nav_lights", "strobe"):
            self.assertIn(name, rig.PROP_NAMES)

    def test_drivers_valid_and_simple(self):
        import bpy
        n = 0
        bad = []
        for idb in list(bpy.data.objects) + list(bpy.data.materials):
            ad = getattr(idb, "animation_data", None)
            if ad is None:
                continue
            for fc in ad.drivers:
                d = fc.driver
                n += 1
                if not (fc.is_valid and d.is_valid and d.is_simple_expression):
                    bad.append(f"{idb.name}.{fc.data_path}[{fc.array_index}]")
                for v in d.variables:
                    if not v.is_name_valid or v.targets[0].id is None:
                        bad.append(f"{idb.name}.{fc.data_path}: değişken {v.name}")
        self.assertGreaterEqual(n, 39)
        self.assertEqual(bad, [])
        self.assertEqual(self.info["rig"]["missing"], [])
        self.assertEqual(self.info["rig"]["not_simple"], [])

    # ------------------------------------------------------------------ ağlar ve malzemeler
    def test_all_meshes_have_um_materials(self):
        import bpy

        from ucav import params as P
        bad = []
        for ob in bpy.data.objects:
            if ob.type != "MESH" or not ob.name.startswith("U_"):
                continue
            mats = list(ob.data.materials)
            if not mats or any(m is None or m.name not in P.MATERIALS for m in mats):
                bad.append(ob.name)
        self.assertEqual(bad, [])
        self.assertEqual(self.info["materials"]["missing"], [])
        for name in P.MATERIALS:
            mat = bpy.data.materials[name]
            self.assertTrue(mat.use_nodes and mat.node_tree is not None, name)
            self.assertIn("ucav_recipe", mat.keys(), name)

    def test_meshes_manifold_and_valid(self):
        """Kapalı (sınır/manifold dışı kenar yok) ve ``Mesh.validate()`` hiçbir şeyi değiştirmiyor (glTF dışa
        aktarıcısı geçersiz yüzleri silip delik bırakır — boolean artıkları ``util.clean_degenerate`` ile temizlenir)."""
        import bpy

        from ucav.blender import util as U
        bad, invalid = {}, []
        for ob in bpy.data.objects:
            if ob.type == "MESH" and ob.name.startswith("U_") and not ob.name.startswith("U_Stand"):
                r = U.mesh_report(ob)
                if r["boundary"] or r["nonmanifold"]:
                    bad[ob.name] = (r["boundary"], r["nonmanifold"])
                me = ob.data.copy()
                try:
                    if me.validate(verbose=False):
                        invalid.append(ob.name)
                finally:
                    bpy.data.meshes.remove(me)
        self.assertEqual(bad, {})
        self.assertEqual(invalid, [])

    # ------------------------------------------------------------------ ölçüler
    def test_overall_dimensions_within_1pct(self):
        from ucav import params as P
        ov = P.SPEC["overall"]
        V = np.vstack([self._world_verts(o) for o in self._aircraft_meshes()])
        span = V[:, 1].max() - V[:, 1].min()
        length = V[:, 0].max() - V[:, 0].min()                 # burun ucu → dikey firar kenarı
        height = V[:, 2].max() - P.GROUND_Z                     # zemin → en yüksek nokta (takım açık)
        self.assertAlmostEqual(V[:, 0].max(), 0.0, delta=0.002)   # burun ucu X = 0
        for got, ref, nm in ((span, ov["span_m"], "açıklık"), (length, ov["length_m"], "boy"),
                             (height, ov["height_m"], "yükseklik")):
            self.assertLess(abs(got / float(ref) - 1.0), 0.01, f"{nm}: {got:.4f} / spec {ref}")

    def test_gear_down_wheels_touch_ground(self):
        from ucav import params as P
        import bpy
        self._set(gear=1.0)
        for leg in ("N", "L", "R"):
            zmin = self._world_verts(bpy.data.objects[f"U_GearWheel_{leg}"])[:, 2].min()
            self.assertAlmostEqual(zmin, P.GROUND_Z, delta=0.0005, msg=leg)
        # aircraft above ground everywhere else (wheels are the lowest points)
        others = [o for o in self._aircraft_meshes() if not o.name.startswith("U_GearWheel_")]
        zlow = min(self._world_verts(o)[:, 2].min() for o in others)
        self.assertGreater(zlow, P.GROUND_Z + 0.005)

    def test_gear_retracts_into_wells(self):
        import bpy

        from ucav import params as P
        self._set(gear=0.0)
        for leg in ("N", "L", "R"):
            g = P.gear_leg(leg)
            ref = bpy.data.objects[f"U_GearAxleRef_{leg}"].matrix_world.translation
            np.testing.assert_allclose(tuple(ref), P.to_blender(*g.axle_retracted), atol=2e-4, err_msg=leg)
        zlow = min(self._world_verts(bpy.data.objects[f"U_GearWheel_{leg}"])[:, 2].min() for leg in ("N", "L", "R"))
        self.assertGreater(zlow, -0.10)                         # tekerler gövde/kanat zarfının içinde

    def test_gear_cycle_no_collisions(self):
        from ucav.blender import gear
        rep = gear.clearance_report(gear_values=(1.0, 0.85, 0.6, 0.4, 0.15, 0.0))
        self.assertEqual(rep["new"], [])
        self.assertEqual(rep["known"], [])

    @staticmethod
    def _bvh(ob):
        """Değerlendirilmiş ağın dünya uzayı BVH ağacı ve köşeleri."""
        import bpy
        from mathutils.bvhtree import BVHTree
        dg = bpy.context.evaluated_depsgraph_get()
        ev = ob.evaluated_get(dg)
        me = ev.to_mesh()
        mw = ev.matrix_world
        V = [mw @ v.co for v in me.vertices]
        F = [tuple(p.vertices) for p in me.polygons]
        ev.to_mesh_clear()
        return BVHTree.FromPolygons(V, F), np.array([tuple(v) for v in V])

    def test_cowl_parts_clear_engine_envelopes(self):
        """R01: kaporta kabuğu, lüle halkası, panjur dudakları, sürtünme pabucu ve NACA dudağı motor ve susturucu
        zarflarına (``U_Env_Engine``, ``U_Env_Muffler``) GİRMEZ: BVH üçgen çakışması 0 (eski kaporta fincanı 166 çift
        veriyordu). ``U_Exhaust_Muffler`` borusu tasarım gereği susturucu kutusuna 2 mm gömülüdür (hariç). Koyu boşluk
        diski yalnız render içindir (``ucav_render_only``; GLB/baskı/çakışma dışı)."""
        import bpy
        envs = {n: self._bvh(bpy.data.objects[n])[0] for n in ("U_Env_Engine", "U_Env_Muffler")}
        hits = {}
        for n in ("U_Cowl", "U_ExhaustRing", "U_Cowl_Louvers", "U_ScuffPad", "U_Intake"):
            t = self._bvh(bpy.data.objects[n])[0]
            for en, et in envs.items():
                k = len(t.overlap(et))
                if k:
                    hits[f"{n}~{en}"] = k
        self.assertEqual(hits, {})
        cav = bpy.data.objects["U_Cowl_Cavity"]
        self.assertTrue(cav.get("ucav_render_only"))
        self.assertEqual(cav.parent.name, "U_Cowl")

    def test_cooling_exit_open_in_scene(self):
        """R01: sahnedeki kaportada soğutma çıkışı gerçekten AÇIK — kaporta boşluğundan geriye (s ekseni) atılan
        ışınlar lüle halkası halkasından (spinner ile halka iç çapı arası) ve çene yarığından kaporta parçalarına
        (kabuk, lüle, panjur, pabuç, susturucu borusu) çarpmadan çıkar. Açık alan = isabetsiz ışın oranı × geometrik
        alan; çıkış (halka + çene) ≥ 1,3 × NACA giriş alanı (≥ 28 cm²)."""
        import bpy
        from mathutils import Vector

        from ucav import params as P
        from ucav import shapes as S
        trees = [self._bvh(bpy.data.objects[n])[0] for n in
                 ("U_Cowl", "U_ExhaustRing", "U_Cowl_Louvers", "U_ScuffPad", "U_Exhaust_Muffler")]
        d = Vector(P.vec_to_blender((1.0, 0.0, 0.0)))

        def open_frac(pts_spec, length):
            free = 0
            for p in pts_spec:
                o = Vector(P.to_blender(*p))
                if all(t.ray_cast(o, d, length)[0] is None for t in trees):
                    free += 1
            return free / len(pts_spec)
        er = P.SPEC["propulsion"]["exhaust_ring"]
        r0, r1 = 0.5 * P.PROP.spinner_d + 0.001, 0.5 * float(er["id_m"]) - 0.001
        zc = P.PROP.hub[2]
        ring_pts = [(2.172, r * math.cos(a), zc + r * math.sin(a))
                    for r in np.sqrt(np.linspace(r0 ** 2, r1 ** 2, 6)) for a in np.radians(np.arange(0, 360, 12))]
        f_ring = open_frac(ring_pts, 0.05)
        a_ring = f_ring * math.pi * ((0.5 * float(er["id_m"])) ** 2 - (0.5 * P.PROP.spinner_d) ** 2)
        V = np.asarray(S.cooling_exit_cutter().verts)
        w = float(np.abs(V[:, 1]).max()) - 0.004
        z0, z1 = float(V[:, 2].min()), float(V[:, 2].max())
        chin_pts = [(2.170, y, z) for y in np.linspace(-0.9 * w, 0.9 * w, 9) for z in np.linspace(z0 + 0.001, z1 - 0.001, 4)]
        f_chin = open_frac(chin_pts, 0.035)
        a_chin = f_chin * 2 * w * (z1 - z0)
        inlet = S.cooling_flow_areas()["inlet"]
        self.assertGreater(f_ring, 0.95, f_ring)
        self.assertGreater(f_chin, 0.95, f_chin)
        self.assertGreaterEqual(a_ring + a_chin, 1.3 * inlet, (a_ring * 1e4, a_chin * 1e4, inlet * 1e4))

    def test_gear_doors_sweep_no_collisions(self):
        """Deri kapakları (``gear_doors`` 0,3 / 0,6 / 1,0, takım açık) gövde, kanat, kaplamalar ve takımla çakışmaz
        (BVH; kapak donanımı ``U_DoorHw_*`` kapağa bağlıdır, hariç). R06 dudağı menteşe kenarından 6 mm uzak."""
        import bpy
        from mathutils import Vector
        doors = ["U_Door_N_1", "U_Door_N_2", "U_Door_L_1", "U_Door_R_1"]
        skip = ("U_DoorHw_", "U_Env_", "U_Stand", "UP_")
        others = [o for o in bpy.data.objects if o.type == "MESH" and o.name.startswith("U_")
                  and not o.name.startswith(skip) and o.name not in doors and not o.hide_render]
        hits = {}
        try:
            for gd in (0.3, 0.6, 1.0):
                self._set(gear=1.0, gear_doors=gd)
                for dn in doors:
                    td, Vd = self._bvh(bpy.data.objects[dn])
                    lo, hi = Vd.min(0) - 0.01, Vd.max(0) + 0.01
                    for ob in others:
                        bb = np.array([tuple(ob.matrix_world @ Vector(c)) for c in ob.bound_box])
                        if (bb.max(0) < lo).any() or (bb.min(0) > hi).any():
                            continue
                        k = len(td.overlap(self._bvh(ob)[0]))
                        if k:
                            hits[f"{dn}~{ob.name}@{gd}"] = k
        finally:
            self._set(gear=1.0, gear_doors=0.0)
        self.assertEqual(hits, {})

    def test_linkage_sweep_no_collisions(self):
        """Kumanda bağlantıları tam aralıkta çarpışmasız: yalnız açık izin listesindeki (``rig.LINK_ALLOW``) tasarım
        gereği gömülü temaslar; en küçük açıklık ≥ 0,3 mm; dümen bağlantısı dikeyin iç yüzünde, kaporta altında."""
        import bpy

        from ucav.blender import rig
        rep = rig.linkage_clearance_report()
        self.assertEqual(rep["new"], [], rep["new_detail"][:6])
        self.assertTrue(rep["known"])                                   # tarama gerçekten temas buldu (izinli)
        self.assertEqual(len(rep["faces"]), 2 * len(rig.LINKAGES))
        self.assertGreaterEqual(rep["min_gap_m"], 0.0003, rep["gaps"])
        for side in ("L", "R"):
            self.assertEqual(rep["faces"][f"Rudder_{side}"], "inboard")
            self.assertEqual(rep["faces"][f"Aileron_{side}"], "lower")
            fair = bpy.data.objects[rig.fairing_name("Rudder", side)]
            self.assertEqual([m.name for m in fair.data.materials], list(rig.FAIRING_MATS))
        for ctl, vals in rep["values"].items():                        # tam kontrol paneli aralığı tarandı
            lo, hi = next((p[2], p[3]) for p in rig.PROPS if p[0] == ctl)
            self.assertEqual((min(vals), max(vals)), (lo, hi), ctl)

    def test_rudder_fairing_covers_servo_arm(self):
        """Dümen servo kolu kaportanın altında: kol ucu (± kumanda) kaporta tepesinin altında, kaporta 30 × 10 mm
        ve deriden ≤ 5,5 mm kabarık; boynuz ve çubuk dikeyin iç yüzünde (dışa normal boyunca < 0)."""
        import bpy
        from mathutils import Vector

        from ucav import params as P
        from ucav.blender import rig
        for side in ("L", "R"):
            g = rig.linkage_geometry("Rudder", side)
            fr = g["fairing"]
            self.assertIsNotNone(fr)
            fair = bpy.data.objects[fr["name"]]
            Mi = g["M_f"].inverted()
            V = np.array([tuple(Mi @ (fair.matrix_world @ v.co)) for v in fair.data.vertices])
            self.assertAlmostEqual(V[:, 1].max() - V[:, 1].min(), 0.030, delta=0.001)
            self.assertAlmostEqual(V[:, 0].max() - V[:, 0].min(), 0.010, delta=0.001)
            top = float((g["s"] * V[:, 2]).max())
            self.assertLessEqual(top - g["w_servo"], 0.0055)
            self.assertGreaterEqual(top, g["d"] + rig.ARM_TIP_R)       # kol ucu kaporta tepesinin altında
            fs = P.fin_station(0.1, side)
            nrm, le = np.asarray(P.vec_to_blender(fs.normal), float), np.asarray(P.to_blender(*fs.le), float)
            for n in (rig.linkage_names("Rudder", side)[0], rig.linkage_names("Rudder", side)[2], fr["name"]):
                ob = bpy.data.objects[n]
                W = np.array([tuple(ob.matrix_world @ Vector(v.co)) for v in ob.data.vertices])
                self.assertLess(float(((W - le) @ nrm).max()), 0.0, n)

    def test_control_surface_sign_convention(self):
        """+ derece: kanat/stabilize firar kenarı aşağı (iki yanda), dümen firar kenarı sancağa (−Y)."""
        import bpy

        from ucav import params as P
        for h in P.hinge_lines():
            ob = bpy.data.objects[h.obj_name]
            drv = [d for d in ob.animation_data.drivers if d.data_path == "rotation_euler"]
            V0 = self._world_verts(ob)
            for d in drv:
                d.mute = True                                   # nesne çerçevesini doğrudan dene
            try:
                ob.rotation_euler[0] = math.radians(10.0)
                bpy.context.view_layer.update()
                V1 = self._world_verts(ob)
            finally:
                ob.rotation_euler[0] = 0.0
                for d in drv:
                    d.mute = False
                bpy.context.view_layer.update()
            far = np.argsort(np.linalg.norm(V0 - np.asarray(h.mid_b), axis=1))[-20:]      # firar kenarına yakın
            d = (V1[far] - V0[far]).mean(0)
            if h.name == "Rudder":
                self.assertLess(d[1], -0.002, h.obj_name)
            else:
                self.assertLess(d[2], -0.002, h.obj_name)

    def test_driven_controls_move_parts(self):
        import bpy
        self._set(aileron_deg=15.0, flap_deg=20.0, elevator_deg=10.0, rudder_deg=-12.0)
        rx = lambda n: math.degrees(bpy.data.objects[n].rotation_euler[0])     # noqa: E731
        self.assertGreater(rx("U_Aileron_L"), 5.0)              # sağa yatış: sol firar kenarı aşağı
        self.assertLess(rx("U_Aileron_R"), -10.0)
        self.assertAlmostEqual(rx("U_FlapIn_L"), 20.0, delta=0.01)
        self.assertAlmostEqual(rx("U_FlapOut_R"), 20.0, delta=0.01)
        self.assertAlmostEqual(rx("U_Elevator_L"), 10.0, delta=0.01)
        self.assertAlmostEqual(rx("U_Rudder_R"), -12.0, delta=0.01)
        self._set(gear=0.0)
        self.assertGreater(math.degrees(bpy.data.objects["U_GearPivot_L"].rotation_euler[0]), 89.0)


    # ------------------------------------------------------------------ işaretler ve görünüm kuralları
    def test_stencils_hosted_and_closed(self):
        """Şablon yazılar (``U_Stencil_*``): en az 20, her biri spec'teki ev sahibine bağlı, UM_* malzemeli, kapalı."""
        import bpy

        from ucav import params as P
        from ucav import shapes as S
        from ucav.blender import util as U
        specs = {x.name: x.host for x in S.stencil_specs()}
        objs = [o for o in bpy.data.objects if o.name.startswith("U_Stencil_")]
        self.assertGreaterEqual(len(objs), 20)
        self.assertEqual(sorted(o.name for o in objs), sorted(specs))
        for ob in objs:
            self.assertIsNotNone(ob.parent, ob.name)
            self.assertEqual(ob.parent.name, specs[ob.name], ob.name)
            self.assertTrue(all(m is not None and m.name in P.MATERIALS for m in ob.data.materials), ob.name)
            r = U.mesh_report(ob)
            self.assertEqual((r["boundary"], r["nonmanifold"]), (0, 0), ob.name)

    def test_look_rules(self):
        """Yan görünüş göz hizasında perspektif (≤ 1° yukarıdan, seyrüsefer ışıkları kapalı); pist pozlaması −0,4 EV;
        panel çizgileri baskı ekleriyle aynı; kuyu ve kapaklarda UM_Orange yok; CG işareti CG istasyonunda ve kök
        filetosunun üstünde; yer dolgu ışığının sürücüsü basit ifade."""
        import bpy

        from ucav import params as P
        from ucav.blender import animation, studio
        from ucav.blender import materials as M
        animation.clear()                                       # ölçümler dinlenme pozunda
        side = studio.VIEWS["side"]
        self.assertLessEqual(side.el, 1.0)
        self.assertGreater(side.el, 0.0)
        self.assertFalse(side.ortho)
        self.assertEqual(bpy.data.objects["U_Cam_side"].data.type, "PERSP")
        self.assertEqual(side.controls.get("nav_lights"), 0.0)
        self.assertAlmostEqual(studio.EXPOSURE["pist"], -0.4)
        self.assertAlmostEqual(bpy.context.scene.view_settings.exposure, -0.4, places=3)
        st = M.panel_stations()
        self.assertGreaterEqual(len(st["fuselage_s"]), 10)
        self.assertEqual(len(st["wing_y"]), 9)
        rep = OUT / "print_report.json"
        if rep.exists():                                        # render çizgileri = gerçek baskı ekleri
            pc = json.loads(rep.read_text(encoding="utf-8")).get("print_cuts", {})
            if pc.get("fuselage_s_m"):
                np.testing.assert_allclose(st["fuselage_s"], pc["fuselage_s_m"], atol=6e-4)
            if pc.get("wing_y_m"):
                np.testing.assert_allclose(st["wing_y"], [y for y in pc["wing_y_m"] if y < P.WING_SEMI_SPAN - 1e-6],
                                           atol=6e-4)
        orange = [o.name for o in bpy.data.objects if o.type == "MESH" and o.name.startswith(("U_Bay_", "U_Door_"))
                  and any(m is not None and m.name == "UM_Orange" for m in o.data.materials)]
        self.assertEqual(orange, [])
        for side in ("L", "R"):
            V = self._world_verts(bpy.data.objects[f"U_Decal_CG_{side}"])
            self.assertAlmostEqual(-0.5 * (V[:, 0].min() + V[:, 0].max()), P.CG.s, delta=0.005, msg=side)
            F = self._world_verts(bpy.data.objects[f"U_Fairing_Fillet_{side}"])
            near = F[np.abs(F[:, 0] - V[:, 0].mean()) < 0.012]
            if len(near):
                self.assertGreater(V[:, 2].min(), near[:, 2].max() + 0.002, side)
        fill = bpy.data.objects.get(studio.UNDER_FILL)
        self.assertIsNotNone(fill)
        drv = fill.data.animation_data.drivers
        self.assertTrue(len(drv))
        for fc in drv:
            self.assertTrue(fc.is_valid and fc.driver.is_valid and fc.driver.is_simple_expression, fc.data_path)

    def _project(self, cam_name: str, pts: np.ndarray, res=(1600, 1000)) -> np.ndarray:
        """Dünya noktaları → piksel (x sağa, y aşağı) ``cam_name`` kamerasında (objektif kaydırması dahil)."""
        import bpy
        from bpy_extras.object_utils import world_to_camera_view
        from mathutils import Vector
        sc = bpy.context.scene
        old = (sc.render.resolution_x, sc.render.resolution_y, sc.render.resolution_percentage)
        sc.render.resolution_x, sc.render.resolution_y, sc.render.resolution_percentage = res[0], res[1], 100
        try:
            cam = bpy.data.objects[cam_name]
            q = np.array([tuple(world_to_camera_view(sc, cam, Vector(p))) for p in pts])
        finally:
            sc.render.resolution_x, sc.render.resolution_y, sc.render.resolution_percentage = old
        return np.column_stack([q[:, 0] * res[0], (1.0 - q[:, 1]) * res[1], q[:, 2]])

    def test_still_framing(self):
        """R02/R09: kahraman ve arka-sağ 3/4 görünüşlerinde iki kanat ucu da kadrajın ≥ %3 içinde; yan görünüşte uzak
        (sancak) kanat sırt çizgisinin üstüne çıkmaz (≤ 3 px @ 1600 px) ve ufuk kadrajın üst %25'inin altında
        (gökyüzü ≥ %25)."""
        import bpy
        from mathutils import Vector

        from ucav.blender import animation, studio
        animation.clear()
        bpy.context.view_layer.update()
        pts = studio.aircraft_points(bpy.context.scene, per_object=800)
        for view in ("hero", "rear34"):
            self.assertTrue(studio.VIEWS[view].center, view)
            q = self._project(f"U_Cam_{view}", pts)
            self.assertGreaterEqual(q[:, 0].min() / 1600.0, 0.025, view)
            self.assertLessEqual(q[:, 0].max() / 1600.0, 0.975, view)
            self.assertGreaterEqual(q[:, 1].min() / 1000.0, 0.0, view)
            self.assertLessEqual(q[:, 1].max() / 1000.0, 1.0, view)
        # yan: sütun başına uzak kanadın tepesi gövde tepesinin altında
        far = np.vstack([self._world_verts(bpy.data.objects[n]) for n in
                         ("U_WingCenter_R", "U_WingOuter_R", "U_Tip_R", "U_FlapIn_R", "U_FlapOut_R", "U_Aileron_R")])
        fus = self._world_verts(bpy.data.objects["U_Fuselage"])
        qf, qw = self._project("U_Cam_side", fus), self._project("U_Cam_side", far)
        cols = np.floor(qf[:, 0] / 8.0).astype(int)
        top = {}
        for c, y in zip(cols, qf[:, 1]):
            top[c] = min(top.get(c, 1e9), y)
        above = [top[c] - y for c, y in zip(np.floor(qw[:, 0] / 8.0).astype(int), qw[:, 1]) if c in top]
        self.assertTrue(above)
        self.assertLessEqual(max(above), 3.0)
        cam = bpy.data.objects["U_Cam_side"]
        fwd = cam.matrix_world.to_3x3() @ Vector((0.0, 0.0, -1.0))
        h = Vector((fwd.x, fwd.y, 0.0)).normalized() * 3000.0 + cam.matrix_world.translation
        h.z = cam.matrix_world.translation.z
        y_hor = self._project("U_Cam_side", np.array([tuple(h)]))[0, 1] / 1000.0
        self.assertGreater(y_hor, 0.25)
        self.assertLess(y_hor, 0.6)

    def test_material_look_values(self):
        """R03/R08/R10/R12: pervane diski açık pus (yumuşak, soluk uç halkası); taret pencerelerinde tam parlak
        kaplama ve ``nose`` görünümünde ufku yansıtan taret açısı; füme kapakta sert vernik; standart şemada üst/alt
        parlaklık oranı ≥ 1,5 (karşı gölge), taktik üst boya koyu (L* < 30); çim şeritleri düşük genlikli."""
        import bpy

        from ucav import params as P
        from ucav.blender import studio
        from ucav.blender import materials as M

        def lum(rgb):
            return 0.2126 * rgb[0] + 0.7152 * rgb[1] + 0.0722 * rgb[2]
        prof = M.DISC_PROFILE
        self.assertLessEqual(prof["tip"] / prof["blade"], 1.25)
        self.assertGreaterEqual(prof["fade_m"], 0.010)
        self.assertGreaterEqual(M.DISC_TIP_MIX, 0.5)
        self.assertGreater(lum(P.MATERIALS["UM_PropDisc"].rgb_linear), 0.25)          # açık pus, koyu duman değil
        glass = bpy.data.materials["UM_SensorGlass"].node_tree.nodes
        coats = [n.inputs["Coat Weight"].default_value for n in glass if n.type == "BSDF_PRINCIPLED"]
        self.assertEqual(len(coats), 2)
        self.assertTrue(all(c >= 0.99 for c in coats))
        self.assertGreaterEqual(studio.VIEWS["nose"].controls["turret_tilt_deg"], -5.0)
        hatch = [n for n in bpy.data.materials["UM_SmokeHatch"].node_tree.nodes if n.type == "BSDF_PRINCIPLED"]
        self.assertEqual(len(hatch), 1)
        self.assertGreaterEqual(hatch[0].inputs["Coat Weight"].default_value, 0.99)
        self.assertLessEqual(hatch[0].inputs["Coat Roughness"].default_value, 0.04)
        self.assertGreater(M.SMOKE_TOTAL_T, 0.10)
        cols = M.livery_colors("standart")
        self.assertGreaterEqual(lum(cols["UM_SkinBottom"]) / lum(cols["UM_SkinTop"]), 1.5)
        tk = M.livery_colors("taktik")["UM_SkinTop"]
        y = lum(tk)
        self.assertLess(116.0 * y ** (1.0 / 3.0) - 16.0, 30.0)
        self.assertLessEqual(studio.GRASS_STRIPE, 0.08)

    def test_turret_led_independent_of_nav_lights(self):
        """Taret durum halkası ``status_led``'e bağlı: seyrüsefer ışıkları açıkken bile sönük (gerçek EO/IR taretler
        ışımaz); ``status_led`` = 1 iken yanar."""
        import bpy

        from ucav.blender import rig
        ring = bpy.data.objects["U_Light_Turret_Ring"]
        try:
            rig.set_controls(nav_lights=1.0, status_led=0.0)
            self.assertEqual(float(ring["ucav_emission"]), 0.0)
            self.assertEqual(float(bpy.data.objects["U_Light_Nav_L"]["ucav_emission"]), 1.0)
            rig.set_controls(status_led=1.0)
            self.assertEqual(float(ring["ucav_emission"]), 1.0)
        finally:
            rig.set_controls(nav_lights=1.0, status_led=0.0)

    def test_prop_disc(self):
        """Pervane diski: ``UM_PropDisc`` malzemeli; pervane dururken render dışı, 3000 dev/dk'da görünür."""
        import bpy

        from ucav.blender import rig
        disc = bpy.data.objects["U_PropDisc"]
        self.assertEqual([m.name for m in disc.data.materials], ["UM_PropDisc"])
        self.assertEqual(bpy.data.materials["UM_PropDisc"]["ucav_recipe"], "propdisc")
        try:
            rig.set_controls(prop_rpm=0.0)
            self.assertTrue(disc.hide_render)
            rig.set_controls(prop_rpm=3000.0)
            self.assertFalse(disc.hide_render)
            self.assertGreater(float(disc["ucav_disc"]), 0.2)
        finally:
            rig.set_controls(prop_rpm=0.0)

    # ------------------------------------------------------------------ rig: bağlantılar, ebeveyn kuralı, pervane
    def test_control_linkages_follow_surfaces(self):
        """Servo kolu yüzeyle aynı açıyla döner; itme çubuğu dönmez (paralelkenar), ucu boynuz deliğinde."""
        import bpy
        from mathutils import Vector

        from ucav.blender import rig
        try:
            for name, ctl in (("Aileron", "aileron_deg"), ("FlapOut", "flap_deg"), ("Elevator", "elevator_deg"),
                              ("Rudder", "rudder_deg")):
                for side in ("L", "R"):
                    horn, arm, rod = (bpy.data.objects[n] for n in rig.linkage_names(name, side))
                    surf = bpy.data.objects[f"U_{name}_{side}"]
                    R0 = np.array(rod.matrix_world.to_3x3())
                    rig.set_controls(**{ctl: 15.0})
                    self.assertAlmostEqual(arm.rotation_euler.x, surf.rotation_euler.x, places=6)
                    np.testing.assert_allclose(np.array(rod.matrix_world.to_3x3()), R0, atol=1e-6)
                    L = rig.linkage_geometry(name, side)["L"]
                    tip = rod.matrix_world @ Vector((0.0, L, 0.0))
                    np.testing.assert_allclose(tuple(tip), tuple(horn.matrix_world.translation), atol=2e-4,
                                               err_msg=f"{name}_{side}")
                    rig.set_controls(**{ctl: 0.0})
        finally:
            rig.set_controls(aileron_deg=0.0, flap_deg=0.0, elevator_deg=0.0, rudder_deg=0.0)

    def test_viewport_locks_and_parenting(self):
        """Sürülen nesneler kilitli; ``UCAV`` ağacında ebeveyn ters matrisleri birim; ``U_Root`` önde çizilir;
        sabit görüntü kameraları da birim ebeveyn tersiyle ``UCAV_Cameras_Stills``'te."""
        import bpy
        root = bpy.data.objects["U_Root"]
        self.assertTrue(root.show_in_front)
        self.assertGreaterEqual(root.empty_display_size, 1.0)
        unit = lambda M: all(abs(M[i][j] - (i == j)) < 1e-9 for i in range(4) for j in range(4))   # noqa: E731
        for ob in bpy.data.collections["UCAV"].all_objects:
            if ob.animation_data is not None and len(ob.animation_data.drivers) and ob is not root:
                self.assertTrue(all(ob.lock_location) and all(ob.lock_scale), ob.name)
            if ob.parent is not None:
                self.assertTrue(unit(ob.matrix_parent_inverse), ob.name)
        for v in VIEWS:
            cam = bpy.data.objects[f"U_Cam_{v}"]
            self.assertTrue(unit(cam.matrix_parent_inverse), cam.name)
            self.assertIn("UCAV_Cameras_Stills", [c.name for c in cam.users_collection])

    def test_prop_angle_is_rpm_integral(self):
        """Pervane açısı ∫rpm·dt (pişirilmiş ``ucav_turns``): gösterimde etkin devir plana %1 içinde; mekanizma
        döngüsünde dikişte açı aynı (mod 2π)."""
        import bpy

        from ucav.blender import animation, rig
        sc = bpy.context.scene
        prop, root = bpy.data.objects["U_Prop"], bpy.data.objects["U_Root"]

        def ang(f, sub=0.0):
            sc.frame_set(f, subframe=sub)
            return prop.rotation_euler.x
        try:
            animation.set_scene_range("showcase")
            fc = next(f for f in rig._action_fcurves(root.animation_data.action) if f.data_path == '["prop_rpm"]')
            for f in (100, 300):
                eff = (ang(f, 0.02) - ang(f)) / 0.02 / (2 * math.pi) * sc.render.fps * 60.0
                self.assertAlmostEqual(eff / fc.evaluate(f), 1.0, delta=0.01, msg=f)
            n = animation.set_scene_range("mechanisms")["frames"]
            d = (ang(n + 1) - ang(1)) % (2 * math.pi)
            self.assertLess(min(d, 2 * math.pi - d), 1e-3)
        finally:
            animation.clear()

    def test_mechanisms_loop_seam(self):
        """Döngü dikişi: kare N+1'de ``U_Root`` özellikleri, dönüşümü ve etkin kamera kare 1 ile aynı."""
        import bpy

        from ucav.blender import animation
        sc = bpy.context.scene
        root = bpy.data.objects["U_Root"]

        def state(f):
            sc.frame_set(f)
            cam = max((m for m in sc.timeline_markers if m.camera is not None and m.frame <= f),
                      key=lambda m: m.frame).camera
            props = [float(root[k]) for k in sorted(root.keys()) if isinstance(root[k], float)]
            return np.array(props), np.array(root.matrix_world), np.array(cam.matrix_world)
        try:
            n = animation.set_scene_range("mechanisms")["frames"]
            for x, y in zip(state(1), state(n + 1)):
                np.testing.assert_allclose(x, y, atol=1e-6)
        finally:
            animation.clear()

    def test_showcase_gear_and_wheels(self):
        """Takım ≥ 2,5 m AGL kazançta toplanır; teker kesmeden 1,5 s sonra teker dönüşü < %10, takım toplanırken 0."""
        import bpy

        from ucav import params as P
        from ucav.blender import animation
        sc = bpy.context.scene
        sp = animation.showcase_plan()
        g_up = animation.CONTROL_KEYS_SHOWCASE["gear"][1][0]
        self.assertGreaterEqual(sp["loc"][animation._fr(g_up) - 1, 2] - P.U_ROOT_B[2], 2.5)
        w = bpy.data.objects["U_GearWheel_L"]

        def rate(f):
            sc.frame_set(f)
            a = w.rotation_euler.y
            sc.frame_set(f, subframe=0.05)
            return (w.rotation_euler.y - a) / 0.05
        try:
            animation.set_scene_range("showcase")
            F = sp["i_lof"] + 1
            r0 = rate(F - 1)
            self.assertGreater(abs(r0), 100.0 / sc.render.fps)
            self.assertLess(abs(rate(F + 36)), 0.10 * abs(r0))
            self.assertEqual(rate(animation._fr(g_up + 1.2)), 0.0)
        finally:
            animation.clear()

    def test_asset_append_single_scene(self):
        """``UCAV`` başka dosyaya eklenince tek sahne, stüdyo/baskı yok, sürücüler geçerli; zemin
        ``U_Root['ground_z']`` ile taşınır (tekerler Z = 0'a oturur). Ayrı süreçte."""
        import bpy

        from ucav import params as P
        with tempfile.TemporaryDirectory() as td:
            src = str(Path(td) / "a.blend")
            bpy.ops.wm.save_as_mainfile(filepath=src, copy=True)
            r = subprocess.run([sys.executable, "-c", _APPEND_CHECK, src, repr(P.U_ROOT_B[0]),
                                repr(P.U_ROOT_B[2] - P.GROUND_Z)], capture_output=True, text=True, timeout=600)
            self.assertIn("APPEND_OK", r.stdout, r.stdout[-1500:] + r.stderr[-1500:])

    # ------------------------------------------------------------------ animasyon, kameralar, GLB, render
    def test_animation_clips(self):
        import bpy

        from ucav.blender import animation
        try:
            for name, frames in (("showcase", 504), ("mechanisms", 384)):
                r = animation.set_scene_range(name)
                self.assertEqual(r["frames"], frames)
                self.assertIsNotNone(bpy.data.actions.get(f"YK38_{name}_Root"))
                self.assertTrue(bpy.context.scene.timeline_markers)
                self.assertIsNotNone(bpy.context.scene.camera)
        finally:
            animation.clear()
        root = bpy.data.objects["U_Root"]
        self.assertIsNone(root.animation_data.action)
        self.assertEqual(float(root["gear"]), 1.0)
        self.assertIn("YK38_klip_sec.py", bpy.data.texts)

    def test_still_cameras(self):
        import bpy
        for v in VIEWS:
            self.assertEqual(bpy.data.objects[f"U_Cam_{v}"].type, "CAMERA")

    def test_view_apply_restore_roundtrip(self):
        """Görünüm ayarı (kamera, kontrol değerleri) uygulanıp geri alınınca sahne eski hâline döner."""
        import bpy

        from ucav.blender import animation, studio
        sc = bpy.context.scene
        animation.set_scene_range("showcase")
        try:
            cam0, n_mk = sc.camera, len(sc.timeline_markers)
            root = bpy.data.objects["U_Root"]
            g0 = float(root["gear"])
            st = studio.apply_view("under")
            self.assertEqual(sc.camera.name, "U_Cam_under")
            self.assertAlmostEqual(float(root["gear"]), 0.0)
            studio.restore_view(st)
            self.assertIs(sc.camera, cam0)
            self.assertEqual(len(sc.timeline_markers), n_mk)
            self.assertAlmostEqual(float(root["gear"]), g0)
        finally:
            animation.clear()

    def test_encode_mp4(self):
        import bpy

        from ucav.blender import render
        with tempfile.TemporaryDirectory() as td:
            imgs = []
            for k in range(2):
                im = bpy.data.images.new(f"t{k}", 64, 36)
                im.pixels[:] = [0.2 * k, 0.4, 0.6, 1.0] * (64 * 36)
                im.filepath_raw = str(Path(td) / f"f_{k + 1:05d}.png")
                im.file_format = "PNG"
                im.save()
                bpy.data.images.remove(im)
                imgs.append(Path(td) / f"f_{k + 1:05d}.png")
            out = Path(td) / "t.mp4"
            render.encode_mp4(imgs, out, 12.0, (64, 36))
            self.assertTrue(out.exists() and out.stat().st_size > 200)

    def test_glb_export(self):
        from ucav.blender import build
        with tempfile.TemporaryDirectory() as td:
            p = Path(td) / "t.glb"
            r = build.export_glb(p, "standart", with_anim=True)
            data = p.read_bytes()
            self.assertEqual(data[:4], b"glTF")
            L = struct.unpack("<I", data[12:16])[0]
            J = json.loads(data[20:20 + L])
            mats = {m["name"]: m for m in J["materials"]}
            top = mats["UM_SkinTop"]["pbrMetallicRoughness"]["baseColorFactor"]
            self.assertLess(top[0], 0.8)                        # RAL 7035 (beyaz varsayılan değil)
            self.assertEqual(len(J.get("animations", [])), 1)
            self.assertGreater(r["objects"], 80)
            nodes = [n.get("name", "") for n in J["nodes"]]
            self.assertNotIn("U_PropDisc", nodes)               # karışım malzemeli disk glTF'te opak olurdu
            self.assertNotIn("U_Cowl_Cavity", nodes)            # yalnız render boşluk diski (R01)
            self.assertFalse([n for n in nodes if n.startswith(("U_Env_", "UP_", "S_"))])
            led = mats.get("UM_StatusLED", {})
            self.assertFalse(any(led.get("emissiveFactor", [0.0])))   # taret LED'i dinlenmede sönük
        import bpy
        self.assertIsNotNone(bpy.data.materials["UM_SkinTop"].node_tree.nodes.get("Material Output"))
        self.assertNotIn("UM_SkinTop__cycles", bpy.data.materials)

    def test_tiny_render(self):
        from ucav.blender import render
        with tempfile.TemporaryDirectory() as td:
            r = render.render_stills(["hero"], res=(160, 100), samples=1, out_dir=td)
            p = Path(r["files"][0])
            self.assertTrue(p.exists() and p.stat().st_size > 1000)
            self.assertEqual(p.read_bytes()[:2], b"\xff\xd8")


@unittest.skipUnless(HAVE_BPY, "bpy kurulu değil")
class TestYK38PrintSubset(unittest.TestCase):
    """Küçük bir baskı alt kümesi (gövde gerekirse kurulur): manifold ve tablaya sığma."""

    def test_print_subset_manifold_and_fits(self):
        import bpy

        from ucav.blender import printprep
        if "U_Fuselage" not in bpy.data.objects:
            from ucav.blender import airframe
            airframe.build()
        bed = (256, 256, 256)
        with tempfile.TemporaryDirectory() as td:
            S = printprep.build_print_parts(bed=bed, out_dir=td, only=["aileron_1", "door_L_2", "fin_1"], verify=0)
            self.assertTrue(S["parts"], "parça kurulmadı")
            self.assertTrue(S["all_manifold"])
            self.assertTrue(S["all_fit"])
            for part in S["parts"]:
                V, T = _read_stl(Path(td) / part["stl"]["file"])
                self.assertTrue(_closed(T), part["key"])
                self.assertGreater(_volume(V, T), 0.0, part["key"])
                ext = V.max(0) - V.min(0)
                self.assertTrue((ext <= np.array(bed) + 1e-6).all(), part["key"])


    def test_cowl_print_parts_clear_engine_and_open(self):
        """R01, baskı: kaporta parçaları (``print_solid`` kaynağı, motor yasak bölgeleri, Ø92 arka açıklık, çene
        yarığı) motor ve susturucu zarflarına girmez (BVH çakışması 0, köşe uzaklığı ≥ 4,5 mm), arka açıklık ve çene
        yarığı boydan boya açık (ışınlar hiçbir UP_ kaporta parçasına çarpmaz)."""
        import bpy
        from mathutils import Vector
        from mathutils.bvhtree import BVHTree

        from ucav import params as P
        from ucav import shapes as S
        from ucav.blender import printprep
        if "U_Fuselage" not in bpy.data.objects:
            from ucav.blender import airframe
            airframe.build()
        keys = ["cowl_top", "cowl_cheek_L", "cowl_cheek_R", "exhaust_ring", "scuff_pad"]
        with tempfile.TemporaryDirectory() as td:                # "cowl" tarifi beş itki parçasını birlikte kurar
            R = printprep.build_print_parts(bed=(256, 256, 256), out_dir=td, only=["cowl"], verify=0)
        self.assertTrue(R["all_manifold"])
        self.assertTrue(set(keys) <= {p["key"] for p in R["parts"]}, [p["key"] for p in R["parts"]])

        def tree(ob):
            dg = bpy.context.evaluated_depsgraph_get()
            ev = ob.evaluated_get(dg)
            me = ev.to_mesh()
            V = [ev.matrix_world @ v.co for v in me.vertices]
            F = [tuple(p.vertices) for p in me.polygons]
            ev.to_mesh_clear()
            return BVHTree.FromPolygons(V, F), np.array([tuple(v) for v in V])
        envs = {n: tree(bpy.data.objects[n])[0] for n in ("U_Env_Engine", "U_Env_Muffler")}
        parts = {k: tree(bpy.data.objects[f"UP_{k}"]) for k in keys}
        hits = {f"{k}~{n}": len(t.overlap(et)) for k, (t, _) in parts.items() for n, et in envs.items()
                if len(t.overlap(et))}
        self.assertEqual(hits, {})
        for k, (_, V) in parts.items():
            Q = np.column_stack([-V[:, 0], V[:, 1], V[:, 2]])
            for e in P.engine_envelope():
                lim = 0.015 if e.name == "muffler" else 0.0045
                self.assertGreaterEqual(float(printprep._env_distance(e, Q).min()), lim, f"{k}↔{e.name}")
        d = Vector(P.vec_to_blender((1.0, 0.0, 0.0)))
        er = P.SPEC["propulsion"]["exhaust_ring"]
        zc = P.PROP.hub[2]
        r0, r1 = 0.5 * P.PROP.spinner_d + 0.001, 0.5 * float(er["id_m"]) - 0.001
        rays = [(2.172, r * math.cos(a), zc + r * math.sin(a)) for r in (r0, 0.5 * (r0 + r1), r1)
                for a in np.radians(np.arange(0, 360, 15))]
        V = np.asarray(S.cooling_exit_cutter().verts)
        w = float(np.abs(V[:, 1]).max()) - 0.004
        zz = 0.5 * (float(V[:, 2].min()) + float(V[:, 2].max()))
        rays += [(2.170, y, zz) for y in np.linspace(-0.85 * w, 0.85 * w, 7)]
        blocked = [p for p in rays if any(t.ray_cast(Vector(P.to_blender(*p)), d, 0.05)[0] is not None
                                          for t, _ in parts.values())]
        self.assertEqual(blocked, [])


class TestHeatRuleLogic(unittest.TestCase):
    """Isı kuralı kararı (``printprep.heat_rule_eval``): kural 1 (150 mm içinde LW-PLA yok) ve kural 2 (50 mm içinde
    Tg < 120 °C yok) ihlali raporlar ve ``ok`` False olur; malzeme kendiliğinden değişmez; spec ataması listelenir."""

    @unittest.skipUnless(HAVE_BPY, "bpy kurulu değil")
    def test_rules_and_failure(self):
        from ucav.blender import printprep as PP
        ok_rows = [{"key": "cowl_cheek_L", "material": "PA-CF", "min_m": 0.0002, "source": "ısı kalkanı"},
                   {"key": "elevator_1", "material": "LW-ASA", "min_m": 0.0566, "source": "muffler",
                    "zone": "stab_root_heat"},
                   {"key": "stab_2a", "material": "LW-PLA", "min_m": 0.164, "source": "susturucu çıkış borusu"}]
        h = PP.heat_rule_eval(ok_rows)
        self.assertTrue(h["ok"])
        self.assertEqual(h["violations"], [])
        self.assertEqual([a["key"] for a in h["assigned"]], ["elevator_1"])
        self.assertEqual([x["key"] for x in h["near"]], ["cowl_cheek_L", "elevator_1", "stab_2a"])
        bad = [{"key": "stab_2a", "material": "LW-PLA", "min_m": 0.120, "source": "muffler"},       # kural 1
               {"key": "elevator_1", "material": "LW-ASA", "min_m": 0.040, "source": "muffler"},    # kural 2 (95 °C)
               {"key": "door_L_1", "material": "PETG", "min_m": 0.049, "source": "cylinder"},       # kural 2 (80 °C)
               {"key": "cowl_top", "material": "PA-CF", "min_m": 0.003, "source": "cylinder"}]      # PA-CF 150 °C: uygun
        h = PP.heat_rule_eval(bad)
        self.assertFalse(h["ok"])
        self.assertEqual(sorted(v["key"] for v in h["violations"]), ["door_L_1", "elevator_1", "stab_2a"])
        self.assertEqual(next(v for v in h["violations"] if v["key"] == "stab_2a")["material"], "LW-PLA")
        self.assertGreaterEqual(PP.HEAT_TG_RULE["tg_min_c"], 120.0)
        self.assertLessEqual(PP.HEAT_TG_RULE["radius_m"], 0.050 + 1e-9)


class TestRenderDevice(unittest.TestCase):
    """Render aygıtı: ``studio.enable_gpu`` GPU arka ucu bulursa sahneyi GPU'ya işaretler, bulamazsa CPU'da kalır;
    ``configure_cycles`` işarete uyar (yerel bilgisayarda ``build.py --gpu``)."""

    @unittest.skipUnless(HAVE_BPY, "bpy kurulu değil")
    def test_enable_gpu_and_configure(self):
        import bpy
        from ucav.blender import studio
        sc = bpy.data.scenes.new("YK38_device_test")
        try:
            backend = studio.enable_gpu(sc)
            self.assertIn(backend, ("CPU",) + studio.GPU_BACKENDS)
            studio.configure_cycles(sc, samples=8)
            self.assertEqual(sc.cycles.device, "CPU" if backend == "CPU" else "GPU")
            sc["ucav_device"] = "GPU"
            studio.configure_cycles(sc, samples=8)
            self.assertEqual(sc.cycles.device, "GPU")
            sc["ucav_device"] = "CPU"
            studio.configure_cycles(sc, samples=8)
            self.assertEqual(sc.cycles.device, "CPU")
        finally:
            bpy.data.scenes.remove(sc)

    @unittest.skipUnless(HAVE_BPY, "bpy kurulu değil")
    def test_cli_has_gpu_flag(self):
        from ucav.blender import build
        self.assertTrue(build.parser().parse_args(["--gpu", "--anim", "mechanisms"]).gpu)
        self.assertFalse(build.parser().parse_args([]).gpu)


class TestYK38PrintOutputs(unittest.TestCase):
    """Depodaki baskı çıktıları (``build.py --print``): rapor tutarlı, her STL kapalı ve tablaya sığar (numpy)."""

    def setUp(self):
        self.report = OUT / "print_report.json"
        if not self.report.exists():
            self.skipTest("ucav/out/print_report.json yok (python3 ucav/blender/build.py --print)")

    def test_report_all_manifold_and_fit(self):
        S = json.loads(self.report.read_text(encoding="utf-8"))
        self.assertTrue(S["all_manifold"])
        self.assertTrue(S["all_fit"])
        bed = np.array(S["bed_mm"], float)
        for part in S["parts"]:
            self.assertTrue(part["check"]["ok"], part["key"])
            self.assertTrue(part["fits"] or part["fits_tight"], part["key"])
            self.assertTrue((np.array(part["size_mm"]) <= bed + 1e-6).all(), part["key"])

    def test_stl_files_closed_and_within_bed(self):
        """Her STL kapalı, tek pozitif kabuk (iç boşluk kabukları negatif hacimlidir), Z_min = 0, tabla teması
        ≥ 3 cm² (z_min'in 0,05 mm içinde aşağı bakan yüzler) ve tablaya sığar; klasörde rapor dışı STL yok."""
        S = json.loads(self.report.read_text(encoding="utf-8"))
        bed = np.array(S["bed_mm"], float)
        files = [OUT / p["stl"]["file"] for p in S["parts"] if p.get("stl")]
        self.assertTrue(files)
        for f in files:
            self.assertTrue(f.exists(), f)
            V, T = _read_stl(f)
            self.assertTrue(_closed(T), f.name)
            self.assertGreater(_volume(V, T), 0.0, f.name)
            self.assertAlmostEqual(V[:, 2].min(), 0.0, delta=1e-3, msg=f.name)      # tablada (Z_min = 0)
            self.assertTrue((V.max(0) - V.min(0) <= bed + 1e-6).all(), f.name)
            lab = _shell_labels(T, len(V))
            pos = [k for k in np.unique(lab) if _volume(V, T[lab == k]) > 1e-9]
            self.assertEqual(len(pos), 1, f"{f.name}: {len(pos)} pozitif kabuk")
            self.assertGreaterEqual(_bed_contact_cm2(V, T), 3.0, f.name)
        if S["bed_mm"] == [256, 256, 256] or tuple(S["bed_mm"]) == (256.0, 256.0, 256.0):
            on_disk = {p.name for p in (OUT / "stl").glob("*.stl")}
            self.assertEqual(on_disk, {Path(f).name for f in files})

    def test_report_supports_heat_and_assembly(self):
        """Destek sınıfı (tablaya bakan 45° sarkma > 2 cm² → "tabla desteği"), ısı kuralı (150 mm içinde LW-PLA
        yok), yük yolları, menteşe pimi takma yönleri, tolerans kuponu, ısıl biçimlendirme adımları ve beklenen
        P4/P10 parçaları raporda; eski kenar üstü kapak parçaları (hatch_a/b) yok."""
        S = json.loads(self.report.read_text(encoding="utf-8"))
        for part in S["parts"]:
            if float(part.get("oh45_to_bed_cm2", 0.0)) > 2.0:
                self.assertEqual(part.get("support"), "tabla desteği", part["key"])
            if part.get("material") == "LW-PLA" and part.get("heat"):
                self.assertGreaterEqual(float(part["heat"]["min_mm"]), 150.0, part["key"])
        self.assertTrue(S["heat_rule"]["ok"])
        self.assertEqual(S["heat_rule"].get("violations", []), [])
        hr = S["heat_rule"]                                     # R05: Tg kuralı — 50 mm içinde Tg ≥ 120 °C
        for x in hr["near"]:
            if float(x["min_mm"]) < float(hr["tg_rule"]["radius_mm"]):
                self.assertGreaterEqual(float(x["tg_c"]), float(hr["tg_rule"]["tg_min_c"]), x["key"])
        self.assertTrue({"stab_1", "elevator_1"} <= {a["key"] for a in hr["assigned"]})
        near = {x["key"]: x for x in hr["near"]}                # R01: kaporta parçaları motora/susturucuya değmez
        for k in ("cowl_top", "cowl_cheek_L", "cowl_cheek_R", "exhaust_ring", "scuff_pad"):
            self.assertIn(k, near)
            self.assertGreater(float(near[k]["min_mm"]), 0.0, k)
            for src, mm in (near[k].get("by_source_mm") or {}).items():
                if src in ("cylinder", "spark_cap", "crankcase", "carb", "front_bearing"):
                    self.assertGreaterEqual(float(mm), 4.5, f"{k}↔{src}")
                if src == "muffler":
                    self.assertGreaterEqual(float(mm), 15.0, f"{k}↔{src}")
        self.assertTrue(S.get("load_paths"))
        self.assertTrue(S.get("hinges"))
        for name, h in S["hinges"].items():
            self.assertTrue(h.get("insert_side"), name)
        self.assertTrue(S.get("tolerance_coupon"))
        self.assertTrue(S.get("thermoform"))
        keys = {p["key"] for p in S["parts"]}
        need = {"hatch_buck_a", "hatch_buck_b", "hatch_frame_a", "hatch_frame_b", "tolerance_coupon", "wing_frame_fwd",
                "wing_frame_aft", "gear_mount_L", "gear_mount_N", "engine_ring", "door_N_3"}
        self.assertEqual(need - keys, set())
        self.assertFalse(keys & {"hatch_a", "hatch_b"})
        self.assertFalse((OUT / "stl" / "hatch_a.stl").exists() or (OUT / "stl" / "hatch_b.stl").exists())


if __name__ == "__main__":
    unittest.main()
