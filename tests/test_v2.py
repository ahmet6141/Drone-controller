"""cad/v2/ — bütünleşik gövde: yerleşim ve titreşim (her zaman), parametrik parçalar (CadQuery kuruluysa).

CadQuery testleri ≈ 2 dk sürer; CI yalnızca CadQuery gerektirmeyen testleri çalıştırır.
"""
from __future__ import annotations

import importlib.util
import math
import sys
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "cad" / "v2"))
sys.path.insert(0, str(ROOT / "cad"))
sys.path.insert(0, str(ROOT / "tools"))

import analysis_v2  # noqa: E402
import budget_calc  # noqa: E402
import layout_v2  # noqa: E402
import params as P  # noqa: E402
import v2_params as V  # noqa: E402

HAS_CADQUERY = importlib.util.find_spec("cadquery") is not None


class TestLayoutV2(unittest.TestCase):
    def test_packaging_checks(self):
        for name, ok, detail in layout_v2.checks():
            self.assertTrue(ok, f"{name}: {detail}")

    def test_battery_solves_cg_and_masses_match_budget(self):
        profile = layout_v2.load_variant()
        bx = layout_v2.solve_battery_x(profile)
        cg, total = layout_v2.cg_of(layout_v2.components(profile, bx))
        self.assertAlmostEqual(cg[0], 0.0, delta=0.5)
        self.assertAlmostEqual(total, budget_calc.compute(profile).auw_g, delta=0.5,
                               msg="yerleşim kütlesi bütçe AUW'si ile aynı olmalı")

    def test_variant_within_limits(self):
        self.assertTrue(budget_calc.compute(layout_v2.load_variant()).ok)

    def test_gimbal_view_and_palm_stability(self):
        view = layout_v2.gimbal_view()
        self.assertLessEqual(view["clear_min"], -90.0)
        self.assertGreaterEqual(view["clear_max"], P.PITCH_SOFT_MAX)
        self.assertGreater(layout_v2.tip_angle(), 18.0, "v2 avuçta v1'den (≈ 15°) daha kararlı olmalı")

    def test_enclosed_props_allow_short_pod(self):
        """Üst ızgara + alt ızgara + motor eteği varsa 100 mm, yoksa v1'in 120 mm kuralı geçerli."""
        self.assertTrue(V.TOP_GRILLE and V.BELL_SKIRT, "kısa ayak yalnızca tam kapalı pervaneyle")
        drop = P.PROP_PLANE_Z - V.POD_BOTTOM_Z
        self.assertGreaterEqual(drop, V.POD_MIN_DROP_ENCLOSED)
        self.assertLess(drop, P.GRIP_MIN_DROP, "v2 ayağı v1 kuralından kısa (amaç bu)")
        self.assertGreaterEqual(V.GIMBAL_POS[2] - V.GIMBAL_BELOW_CENTER - V.POD_BOTTOM_Z, 10.0,
                                "gimbal avuç düzleminin ≥ 10 mm üstünde")

    def test_pod_sensors_see_through_opening(self):
        for name, (clear, need) in layout_v2.pod_sensor_clearance().items():
            self.assertGreaterEqual(clear, need, name)
        self.assertGreaterEqual(V.SENSOR_RECESS, 20.0, "ToF ölü bölgesi ≈ 2 cm: temas anında avuç ölçülebilmeli")

    def test_sections_are_consistent(self):
        for st in V.STATIONS:
            x, z0 = st[0], st[1]
            pts = V.section_points(x)
            n = len(pts)
            self.assertEqual(n, V.SECTION_POINTS, "tüm kesitler aynı sayıda nokta (loft bükülmesin)")
            self.assertEqual(pts[0], (0.0, z0), "kesit alt ortadan başlamalı")
            mirror = max(abs(pts[k][0] + pts[-k][0]) + abs(pts[k][1] - pts[-k][1]) for k in range(1, n))
            self.assertLess(mirror, 1e-9, "k. ve (n−k). noktalar ayna görüntüsü olmalı (simetrik gövde)")
            area = sum(a[0] * b[1] - b[0] * a[1] for a, b in zip(pts, pts[1:] + pts[:1])) / 2
            self.assertGreater(area, 0.0, "saat yönünün tersine")
            self.assertAlmostEqual(V.half_width(x, (z0 + st[2]) / 2), st[5] / 2, delta=0.01)
        self.assertLess(V.half_width(0.0, 0.0, V.WALL), V.half_width(0.0, 0.0))


class TestAnalysisV2(unittest.TestCase):
    def test_engineering_checks(self):
        for name, ok, detail in analysis_v2.checks():
            self.assertTrue(ok, f"{name}: {detail}")

    def test_selected_section_and_sweep(self):
        sweep = {h: ok for h, _, _, ok in analysis_v2.section_sweep()}
        self.assertTrue(sweep[V.ARM_ROOT[1]], "seçili kol kesiti güvenli pencerede olmalı")
        self.assertFalse(sweep[24.0], "ilk tasarım (24 mm kök) 1× bandına giriyordu")

    def test_more_mass_lowers_frequency(self):
        self.assertGreater(analysis_v2.frequency(0.0), analysis_v2.frequency(1.0))
        self.assertGreater(analysis_v2.frequency(0.5, 1.0), analysis_v2.frequency(0.5, analysis_v2.E_CONDITIONED))

    def test_uniform_beam_between_closed_form_bounds(self):
        """Sabit kesit + uç kütlesi: f = √(3EI / ((M + 0,236 m) L³)) / 2π. Rijit motor yuvası ucu nedeniyle
        sonuç, tüm boyun esnek olduğu (alt sınır) ile kütlenin kol ucunda olduğu (üst sınır) durum arasında."""
        sec = analysis_v2.section(0.0)
        f_num = analysis_v2.frequency(0.0, 1.0, "Iy", V.ARM_ROOT, V.ARM_ROOT)
        e = V.E_MOLDED[analysis_v2.MATERIAL]
        lf = analysis_v2.free_length() / 1000.0
        lt = lf + (V.MOTOR_POD[0] - 3.0) / 1000.0
        m_beam = V.DENSITY[analysis_v2.MATERIAL] * 1000 * sec["A"] * 1e-6 * lf * 1.06
        m_eff = analysis_v2.tip_mass_g(0.0) / 1000 + 0.236 * m_beam

        def f(length: float) -> float:
            return math.sqrt(3 * e * sec["Iy"] * 1e-12 / length ** 3 / m_eff) / (2 * math.pi)
        self.assertLess(f(lt), f_num)
        self.assertLess(f_num, f(lf))


@unittest.skipUnless(HAS_CADQUERY, "CadQuery kurulu değil")
class TestCadV2(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        import airframe
        import build_v2
        cls.A = airframe
        cls.B = build_v2
        cls.parts = build_v2.build_parts()
        cls.rows = {r["part"]: r for r in build_v2.mass_rows(cls.parts)}
        cls.arms = airframe.arm_duct_placed(cls.parts["arm_duct"])

    def _overlap(self, a, b) -> float:
        return self.B._overlap(a, b)

    def test_single_solids(self):
        for name in self.B.PRODUCTION:
            self.assertEqual(self.parts[name].solids().size(), 1, name)

    def test_arm_roots_and_battery_fit(self):
        body = self.B._union([self.parts[n] for n in ("top_shell", "bottom_tub", "nose_cover")])
        battery = self.parts["battery_shell"]
        for arm in self.arms:
            self.assertLess(self._overlap(arm, body), 1.0)
            self.assertLess(self._overlap(arm, battery), 1.0)
        self.assertLess(self._overlap(battery, body), 1.0)

    def test_grille_blocks_fingers(self):
        import cadquery as cq
        module = self.parts["arm_duct"]
        pitch = V.GRILLE["cell"] + V.GRILLE["rib"]
        # Hücre merkezi: sütun j = 3 (x = 3·pitch·sin60), satır i = 2 (+ yarım adım tek sütunda)
        center = (3 * pitch * math.sin(math.radians(60)), (2 + 0.5) * pitch)

        def probe(d: float) -> float:
            pin = cq.Workplane("XY").workplane(offset=V.DUCT_Z[0]).center(*center).circle(d / 2).extrude(V.GRILLE["t"])
            return self._overlap(module, pin)
        self.assertEqual(probe(V.GRILLE["cell"] - 0.4), 0.0, "hücre bu çaptan küçük olmamalı")
        self.assertGreater(probe(V.GRILLE["cell"] + 0.4), 0.0, "≥ 10,4 mm parmak geçmemeli")

    def test_top_grille_blocks_fingers(self):
        import cadquery as cq
        g = V.TOP_GRILLE
        top = self.parts["top_grille"]
        pitch = g["cell"] + g["rib"]
        center = (3 * pitch * math.sin(math.radians(60)), (2 + 0.5) * pitch)

        def probe(d: float) -> float:
            pin = cq.Workplane("XY").workplane(offset=g["z"]).center(*center).circle(d / 2).extrude(g["t"])
            return self._overlap(top, pin)
        self.assertEqual(probe(g["cell"] - 0.4), 0.0)
        self.assertGreater(probe(g["cell"] + 0.4), 0.0, "≥ 10,4 mm parmak üstten de geçmemeli")

    def test_analysis_inputs_match_cad(self):
        d = analysis_v2.duct_masses_g()
        top = self.B.volume(self.parts["top_grille"]) / 1000 * V.DENSITY["PC"]
        self.assertAlmostEqual(analysis_v2.TOP_GRILLE_G, top, delta=0.10 * top)
        rho = V.DENSITY[analysis_v2.MATERIAL]
        ring = self.B.volume(self.A.duct_ring()) / 1000 * rho
        self.assertAlmostEqual(d["ring"], ring, delta=0.10 * ring)
        pod = self.B.volume(self.A.motor_pod()) / 1000 * rho
        self.assertAlmostEqual(analysis_v2.POD_MASS_G, pod, delta=0.10 * pod)
        grille = self.B.volume(self.A.grille()) / 1000 * rho
        self.assertAlmostEqual(d["grille"] + d["hub"], grille, delta=0.10 * grille)

    def test_layout_uses_cad_centers_of_mass(self):
        rows = self.rows
        for name in ("top_shell", "bottom_tub", "nose_cover"):
            for a, b in zip(rows[name]["cg"], V.PART_CG[name]):
                self.assertAlmostEqual(a, b, delta=3.0, msg=name)
        group = [rows[n] for n in ("pod", "pod_tip", "sensor_window")]
        z = sum(r["mass_g"] * r["cg"][2] for r in group) / sum(r["mass_g"] for r in group)
        self.assertAlmostEqual(z, V.PART_CG["pod"][2], delta=3.0)
        arm = rows["arm_duct"]["cg"]
        self.assertAlmostEqual(arm[0], V.ARM_DUCT_CG[0], delta=1.5)
        self.assertAlmostEqual(arm[2], V.ARM_DUCT_CG[1], delta=1.5)

    def test_profile_masses_match_cad(self):
        for row in self.B.variant_comparison(list(self.rows.values()), self.parts):
            self.assertLessEqual(abs(row["diff_pct"]), 5.0, row["component"])


if __name__ == "__main__":
    unittest.main()
