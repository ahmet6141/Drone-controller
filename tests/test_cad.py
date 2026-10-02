"""cad/ — yerleşim (her zaman) ve parametrik parça (CadQuery kuruluysa) testleri.

CadQuery testleri: pip install cadquery  (yoksa atlanır; CI yalnızca yerleşim testlerini çalıştırır).
"""
from __future__ import annotations

import importlib.util
import math
import sys
import tempfile
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "cad"))
sys.path.insert(0, str(ROOT / "tools"))

import budget_calc  # noqa: E402
import layout  # noqa: E402
import params as P  # noqa: E402
import validate_params  # noqa: E402

HAS_CADQUERY = importlib.util.find_spec("cadquery") is not None


class TestLayout(unittest.TestCase):
    def test_design_rules(self):
        for name, ok, detail in layout.design_rules():
            self.assertTrue(ok, f"{name}: {detail}")

    def test_every_component_is_placed(self):
        profile = budget_calc.load(layout.PROFILE)
        self.assertEqual(len(layout.masses(profile)), len(profile["components"]) + 9)

    def test_battery_position_centers_cg(self):
        cg, total = layout.center_of_gravity()
        self.assertAlmostEqual(cg[0], 0.0, delta=0.5)
        self.assertAlmostEqual(cg[1], 0.0, delta=layout.CG_TOLERANCE_MM)
        auw = budget_calc.compute(budget_calc.load(layout.PROFILE)).auw_g
        self.assertAlmostEqual(total, auw, delta=0.5, msg="yerleşim kütlesi bütçe AUW'si ile aynı olmalı")

    def test_rotor_geometry_matches_px4(self):
        params = {n: float(v) for _, n, v, _ in validate_params.parse_px4(
            ROOT / "config" / "px4" / "base" / "10-airframe-dc7.params")}
        self.assertAlmostEqual(params["CA_ROTOR0_PX"], P.MOTOR_XY / 1000.0, delta=0.001)
        self.assertAlmostEqual(params["CA_ROTOR0_PY"], P.MOTOR_XY / 1000.0, delta=0.001)

    def test_gimbal_software_limit_matches_behavior(self):
        behavior = budget_calc.load(ROOT / "config" / "mission" / "behavior.yaml")
        lo, hi = behavior["gimbal"]["pitch_limits_deg"]
        self.assertEqual((lo, hi), (P.PITCH_RANGE[0], P.PITCH_SOFT_MAX))
        view = layout.gimbal_view_report()
        self.assertTrue(all(p > hi for p in view["blocked"]), view["blocked"])

    def test_camera_model(self):
        cam = (0.0, 0.0, 0.0)
        self.assertTrue(layout.in_camera_view((100, 0, 0), cam, 0, 66, 41))
        self.assertFalse(layout.in_camera_view((-100, 0, 0), cam, 0, 66, 41))
        self.assertTrue(layout.in_camera_view((0, 0, -100), cam, -90, 66, 41))
        self.assertFalse(layout.in_camera_view((100, 100, 0), cam, 0, 66, 41))      # 45° > 33°
        self.assertTrue(layout.in_cone((0, 0, -50), cam, -1.0, 21))
        self.assertFalse(layout.in_cone((30, 0, -50), cam, -1.0, 21))

    def test_down_camera_sees_through_bumper(self):
        self.assertGreaterEqual(layout.down_camera_clear_half_angle(), P.WIDE_HFOV / 2)


@unittest.skipUnless(HAS_CADQUERY, "CadQuery kurulu değil")
class TestCadParts(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        import build
        cls.build = build
        cls.parts, cls.y_roll = build.build_parts()
        cls.rows = {r["part"]: r for r in build.mass_table(cls.parts)}

    def test_single_solids(self):
        for name, (wp, _, _) in self.parts.items():
            self.assertEqual(wp.solids().size(), 1, name)

    def test_budgets(self):
        limits = {"Tam pervane koruması": 1.00, "Avuç iniş tutamağı": 1.15, "2 eksen fırçasız gimbal": 1.10,
                  "Üst plaka, bağlantılar": 1.00}
        for row in self.build.budget_comparison(list(self.rows.values())):
            key = next(k for k in limits if k in row["component"])
            self.assertLessEqual(row["model_g"], row["budget_g"] * limits[key], row["component"])

    def test_guard_dimensions_and_mesh_opening(self):
        import cadquery as cq
        import prop_guard
        ring = self.parts["guard_ring"][0]
        bb = ring.val().BoundingBox()
        self.assertAlmostEqual(bb.xlen, 2 * (P.guard_outer_r() + P.GUARD_LIP), delta=0.2)
        pitch = prop_guard.mesh_pitch()
        center = (4.5 * pitch, 2.5 * pitch)            # ağ gözü merkezi: çubuklar k·pitch'te, kollardan uzak

        def probe(d: float) -> float:
            pin = cq.Workplane("XY").center(*center).circle(d / 2).extrude(P.MESH_T)
            return sum(s.Volume() for s in ring.intersect(pin).solids().vals())
        self.assertEqual(probe(P.MESH_OPENING - 0.4), 0.0, "göz bu çaptan küçük olmamalı")
        self.assertGreater(probe(P.MESH_OPENING + 0.4), 0.0, "≥ 10,4 mm parmak geçmemeli")

    def test_grip_geometry(self):
        import palm_grip
        tube, mount, bumper = (self.parts[n][0] for n in ("grip_tube", "grip_sensor_mount", "grip_bumper"))
        self.assertAlmostEqual(tube.val().BoundingBox().xlen, P.GRIP_D, delta=0.1)
        placed = palm_grip.placed(tube, mount, bumper)
        bottom = min(s.val().BoundingBox().zmin for s in placed)
        top = max(s.val().BoundingBox().zmax for s in placed)
        self.assertAlmostEqual(bottom, P.GRIP_BOTTOM_Z, delta=0.01)
        self.assertAlmostEqual(top, P.FRAME_BOTTOM_Z, delta=0.01)
        self.assertGreaterEqual(P.PROP_PLANE_Z - bottom, P.GRIP_MIN_DROP)
        self.assertAlmostEqual(placed[1].val().BoundingBox().zmin, P.GRIP_BOTTOM_Z + P.SENSOR_RECESS, delta=0.01)

    def test_gimbal_motion_and_balance(self):
        import gimbal_2axis
        self.assertEqual(gimbal_2axis.interference(self.y_roll), {"pitch": 0.0, "roll": 0.0})
        self.assertTrue(gimbal_2axis.stack_fits(self.y_roll)[0])
        bx, bz = gimbal_2axis.pitch_balance()
        self.assertLessEqual(math.hypot(bx, bz), P.CRADLE_SLOT)

    def test_step_export_roundtrip(self):
        import cadquery as cq
        wp = self.parts["grip_bumper"][0]
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / "bumper.step"
            cq.exporters.export(wp, str(path))
            back = cq.importers.importStep(str(path))
            self.assertAlmostEqual(back.val().Volume(), wp.val().Volume(), delta=1.0)


if __name__ == "__main__":
    unittest.main()
