"""YELKOVAN YK-38 — Blender sahnesinin uçtan uca testleri (``bpy`` yoksa atlanır).

Sahne bir kez kurulur (``ucav.blender.build.build_scene``: gövde → takım → rig → malzemeler → stüdyo →
animasyon, ≈ 20 s). Denetlenenler: sözleşmedeki nesneler ve koleksiyonlar, kontrol paneli özellikleri, bütün
sürücülerin geçerli ve "basit ifade" olması, her ağın UM_* malzemeli ve kapalı (manifold) olması, takım açıkken
tekerlerin zemine değmesi, takım çevriminde çakışma olmaması, kumanda yüzeyi işaret kuralı, ana ölçülerin
spec'e %1 içinde uyması, animasyon klipleri, GLB ve küçük bir render. Baskı: küçük bir segment alt kümesi geçici
dizine kurulur (manifold + tablaya sığma) ve depodaki STL'ler/rapor numpy ile yeniden denetlenir.

Çalıştırma: ``python3 -m unittest tests.test_ucav_blender -v`` (≈ 1–2 dk).
"""
from __future__ import annotations

import importlib.util
import json
import math
import struct
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
    "U_Light_Turret_Ring",
] + [f"U_{k}_{leg}" for leg in ("N", "L", "R") for k in ("GearPivot", "GearStrut", "GearWheel", "GearSlider", "GearUnit")] \
  + ["U_GearSteer_N"]
COLLECTIONS = ["UCAV", "UCAV_Airframe", "UCAV_Surfaces", "UCAV_Gear", "UCAV_Propulsion", "UCAV_Payload", "UCAV_Details",
               "UCAV_Print", "UCAV_Studio"]
VIEWS = ["hero", "rear34", "side", "front", "top", "under", "nose", "tail", "gearbay"]


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

    # ------------------------------------------------------------------ animasyon, kameralar, GLB, render
    def test_animation_clips(self):
        import bpy

        from ucav.blender import animation
        try:
            for name, frames in (("showcase", 456), ("mechanisms", 240)):
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
        S = json.loads(self.report.read_text(encoding="utf-8"))
        bed = np.array(S["bed_mm"], float)
        files = [OUT / p["stl"]["file"] for p in S["parts"] if p.get("stl")]
        self.assertTrue(files)
        for f in files:
            self.assertTrue(f.exists(), f)
            V, T = _read_stl(f)
            self.assertTrue(_closed(T), f.name)
            self.assertGreater(_volume(V, T), 0.0, f.name)
            self.assertGreaterEqual(V[:, 2].min(), -1e-3, f.name)       # tablada (Z ≥ 0)
            self.assertTrue((V.max(0) - V.min(0) <= bed + 1e-6).all(), f.name)


if __name__ == "__main__":
    unittest.main()
