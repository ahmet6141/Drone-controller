"""ucav/ çekirdeği (bpy gerektirmez): profil verisi, yeniden örnekleme ve karıştırma, spec/params enterpolasyonu,
planform integralleri, menteşe ve takım eksenleri, `sizing.py --check`.

Çalıştırma: python3 -m unittest discover -s tests -q
"""
from __future__ import annotations

import math
import sys
import tempfile
import unittest
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from ucav import airfoils as AF  # noqa: E402
from ucav import params as P  # noqa: E402
from ucav import sizing  # noqa: E402


def _seg_dist(points: np.ndarray, poly: np.ndarray) -> np.ndarray:
    """Her noktanın çoklu doğruya en kısa uzaklığı."""
    a, b = poly[:-1], poly[1:]
    ab = b - a
    L2 = np.maximum((ab ** 2).sum(1), 1e-30)
    out = []
    for p in points:
        t = np.clip(((p - a) * ab).sum(1) / L2, 0, 1)
        q = a + t[:, None] * ab
        out.append(np.sqrt(((q - p) ** 2).sum(1)).min())
    return np.array(out)


class TestAirfoilData(unittest.TestCase):
    def test_uiuc_files_keep_header_and_source(self):
        for name, header in (("sd7062", "SD7062 (14%)"), ("sd7032", "SD7032-099-88")):
            a = AF.load(name)
            self.assertEqual(a.header, header)
            self.assertIn(f"m-selig.ae.illinois.edu/ads/coord/{name}.dat", a.source)
            self.assertEqual(len(a.coords), 61)
            self.assertAlmostEqual(a.coords[0, 0], 1.0, places=3)      # Selig: TE üst → LE → TE alt
            self.assertAlmostEqual(a.coords[-1, 0], 1.0, places=3)
            self.assertLess(a.coords[:, 0].min(), 0.001)

    def test_thickness_and_camber_match_published(self):
        t, xt = AF.max_thickness(AF.section("sd7062"))
        self.assertAlmostEqual(t, 0.140, delta=0.002)
        self.assertTrue(0.24 < xt < 0.32)
        f, _ = AF.max_camber(AF.section("sd7062"))
        self.assertAlmostEqual(f, 0.040, delta=0.003)
        self.assertAlmostEqual(AF.max_thickness(AF.section("sd7032"))[0], 0.0995, delta=0.002)
        t10, x10 = AF.max_thickness(AF.section("naca0010"))
        self.assertAlmostEqual(t10, 0.100, delta=0.0005)
        self.assertAlmostEqual(x10, 0.30, delta=0.01)

    def test_naca_symmetric_closed(self):
        c = AF.naca4("0010", 41)
        x, yu, yl = AF.surfaces(c)
        np.testing.assert_allclose(yu, -yl, atol=1e-12)
        self.assertAlmostEqual(AF.te_thickness(c), 0.0, places=9)
        self.assertGreater(AF.te_thickness(AF.naca4("0010", 41, closed_te=False)), 0.001)
        with self.assertRaises(ValueError):
            AF.naca4("010")

    def test_read_lednicer(self):
        up = AF.surfaces(AF.section("naca0010", 21))
        with tempfile.TemporaryDirectory() as d:
            p = Path(d) / "led.dat"
            rows = ["LEDNICER TEST", "21. 21.", ""] + [f"{x} {y}" for x, y in zip(up[0], up[1])] + [""] + \
                   [f"{x} {y}" for x, y in zip(up[0], up[2])]
            p.write_text("\n".join(rows))
            a = AF.read_dat(p)
        self.assertEqual(a.header, "LEDNICER TEST")
        self.assertEqual(len(a.coords), 41)
        self.assertAlmostEqual(AF.max_thickness(AF.resample(a.coords, 41))[0], 0.10, delta=0.002)


class TestAirfoilOps(unittest.TestCase):
    def test_common_parameterisation(self):
        for n in (41, 81, 121):
            a, b = AF.section("sd7062", n), AF.section("sd7032", n)
            self.assertEqual(a.shape, (2 * n - 1, 2))
            self.assertEqual(a.shape, b.shape)
            np.testing.assert_allclose(a[n - 1], (0.0, 0.0), atol=1e-12)          # LE indisi n − 1
            np.testing.assert_allclose(a[:, 0], b[:, 0], atol=1e-12)              # aynı x istasyonları
            xa, _, _ = AF.surfaces(a)
            np.testing.assert_allclose(xa, AF.x_stations(n), atol=1e-12)
            self.assertAlmostEqual(a[0, 0], 1.0)

    def test_resample_fidelity(self):
        """Yeniden örneklenmiş kesit, özgün noktalara (normalize edilmiş çerçevede) 0,05 mm/veter·0,29 m içinde."""
        raw = AF.load("sd7062").coords
        dense = AF.normalize(raw)
        res = AF.section("sd7062", 101)
        d = _seg_dist(dense[::7], res)
        self.assertLess(d.max(), 3e-4)

    def test_blend(self):
        a, b = AF.section("sd7062"), AF.section("sd7032")
        np.testing.assert_allclose(AF.blend(a, b, 0.0), a)
        np.testing.assert_allclose(AF.blend(a, b, 1.0), b)
        tm = AF.max_thickness(AF.blend(a, b, 0.5))[0]
        self.assertTrue(AF.max_thickness(b)[0] < tm < AF.max_thickness(a)[0])
        with self.assertRaises(ValueError):
            AF.blend(a, AF.section("sd7032", 41), 0.5)

    def test_scale_thickness_keeps_camber(self):
        a = AF.section("sd7062")
        b = AF.scale_thickness(a, 0.155 / AF.max_thickness(a)[0])
        self.assertAlmostEqual(AF.max_thickness(b)[0], 0.155, delta=5e-4)
        x = np.linspace(0.05, 0.95, 19)
        np.testing.assert_allclose(AF.camber_at(a, x), AF.camber_at(b, x), atol=1e-12)
        self.assertAlmostEqual(AF.max_thickness(AF.set_thickness(a, 0.12))[0], 0.12, delta=5e-4)

    def test_blunt_te(self):
        a = AF.section("sd7032")
        te = 0.0015 / 0.15
        b = AF.blunt_te(a, te, x_start=0.55)
        self.assertAlmostEqual(AF.te_thickness(b), te, places=9)
        xa, _, _ = AF.surfaces(a)
        front = xa <= 0.55                                                  # x_start'ın önü değişmez
        np.testing.assert_allclose(a[:len(xa)][::-1][front], b[:len(xa)][::-1][front], atol=1e-12)
        np.testing.assert_allclose(a[len(xa) - 1:][front], b[len(xa) - 1:][front], atol=1e-12)
        x = np.linspace(0.6, 1.0, 9)
        np.testing.assert_allclose(AF.camber_at(a, x), AF.camber_at(b, x), atol=1e-12)
        np.testing.assert_allclose(AF.blunt_te(b, te / 2), b)                  # zaten yeterince kalın


class TestParamsBasics(unittest.TestCase):
    def test_scope_no_weapon_keys(self):
        bad = ("weapon", "hardpoint", "munition", "pylon", "store", "release", "bomb", "missile")

        def walk(node, path=""):
            if isinstance(node, dict):
                for k, v in node.items():
                    for w in bad:
                        self.assertNotIn(w, str(k).lower(), f"kapsam dışı anahtar: {path}.{k}")
                    walk(v, f"{path}.{k}")
            elif isinstance(node, list):
                for i, v in enumerate(node):
                    walk(v, f"{path}[{i}]")

        walk(P.SPEC)
        self.assertIn("yalnız EO/IR", P.SPEC["meta"]["scope"])

    def test_coordinates(self):
        self.assertEqual(P.to_blender(1.2, 0.3, -0.04), (-1.2, 0.3, -0.04))
        self.assertEqual(P.from_blender(*P.to_blender(1.2, -0.3, 0.1)), (1.2, -0.3, 0.1))
        pts = np.array([[1.0, 2.0, 3.0], [0.5, -1.0, 0.0]])
        np.testing.assert_allclose(P.points_to_blender(pts), [[-1, 2, 3], [-0.5, -1, 0]])
        np.testing.assert_allclose(P.vec_to_blender((1, 0, 0)), (-1, 0, 0))
        self.assertEqual(P.U_ROOT_B, (-1.232, 0.0, 0.006))
        self.assertEqual(P.spec("wing.dihedral_deg"), 4.0)
        with self.assertRaises(KeyError):
            P.spec("wing.no_such_key")

    def test_pchip_shape_preserving(self):
        x = np.array([0.0, 1.0, 2.0, 3.0, 4.0])
        y = np.array([0.0, 0.1, 1.0, 1.05, 1.05])
        xq = np.linspace(0, 4, 401)
        f = P.Pchip(x, y)
        v = f(xq)
        np.testing.assert_allclose(f(x), y, atol=1e-12)
        self.assertTrue(np.all(np.diff(v) >= -1e-12), "monoton veride aşma olmamalı")
        self.assertLessEqual(v.max(), 1.05 + 1e-12)

    def test_materials_complete(self):
        need = {"UM_SkinTop", "UM_SkinBottom", "UM_Accent", "UM_Turquoise", "UM_Orange", "UM_SmokeHatch", "UM_PACF",
                "UM_Nozzle", "UM_Prop", "UM_Gear", "UM_Tire", "UM_SensorGlass", "UM_NavRed", "UM_NavGreen", "UM_Strobe"}
        self.assertTrue(need <= set(P.MATERIALS))
        self.assertTrue(set(P.MATERIAL_ROLES.values()) <= set(P.MATERIALS))
        for m in P.MATERIALS.values():
            self.assertTrue(all(0.0 <= c <= 1.0 for c in m.rgb_linear), m.name)
        self.assertEqual(P.MATERIALS["UM_SkinTop"].ral, "RAL 7035")
        self.assertAlmostEqual(P.srgb_to_linear(1.0), 1.0)
        self.assertAlmostEqual(P.hex_to_linear("#808080")[0], 0.2158605, places=5)


class TestFuselage(unittest.TestCase):
    def test_stations_reproduced(self):
        for s, w, h, zc in P.fuselage_stations():
            sec = P.fuselage_section(s)
            self.assertAlmostEqual(sec.width, w, places=9)
            self.assertAlmostEqual(sec.height, h, places=9)
            self.assertAlmostEqual(sec.z_center, zc, places=9)

    def test_smooth_monotone_and_maxima(self):
        ss = np.linspace(0, P.FUSELAGE_LENGTH, 2206)
        w = np.array([P.fuselage_section(s).width for s in ss])
        self.assertAlmostEqual(w.max(), 0.205, places=6)                  # PCHIP aşma yapmaz
        self.assertAlmostEqual(ss[int(np.argmax(w))], 0.85, delta=0.002)
        st = P.fuselage_stations()
        for (s0, w0, *_), (s1, w1, *_) in zip(st[:-1], st[1:]):
            m = (ss >= s0) & (ss <= s1)
            self.assertTrue(np.all(w[m] >= min(w0, w1) - 1e-9) and np.all(w[m] <= max(w0, w1) + 1e-9))

    def test_nose_droop_and_tip_radius(self):
        self.assertAlmostEqual(P.fuselage_section(0.0).z_center, -0.014)
        s = 0.001
        r = P.SPEC["fuselage"]["nose"]["tip_radius_plan_m"]
        self.assertAlmostEqual(P.fuselage_section(s).half_width, math.sqrt(2 * r * s), delta=0.1 * math.sqrt(2 * r * s))

    def test_outline_topology(self):
        nq = 20
        for s in (0.17, 0.85, 1.7, 2.205):
            o = P.fuselage_outline(s, nq)
            sec = P.fuselage_section(s)
            self.assertEqual(o.shape, (4 * nq - 4, 2))
            np.testing.assert_allclose(o[0], (0.0, sec.z_top), atol=1e-9)
            np.testing.assert_allclose(o[nq - 1], (sec.half_width, sec.z_chine), atol=1e-9)
            np.testing.assert_allclose(o[2 * nq - 2], (0.0, sec.z_bottom), atol=1e-9)
            np.testing.assert_allclose(o[3 * nq - 3], (-sec.half_width, sec.z_chine), atol=1e-9)
            for k in range(1, 2 * nq - 2):                                  # sağ yarı sol yarının aynası
                np.testing.assert_allclose(o[4 * nq - 4 - k], (-o[k, 0], o[k, 1]), atol=1e-12)
        # yarı genişlik fonksiyonu dış çizgiyle tutarlı
        o = P.fuselage_outline(0.85, 40)
        for y, z in o[1:38]:
            self.assertAlmostEqual(P.fuselage_half_width_at(0.85, z), y, delta=2e-4)

    def test_fuselage_z_lookup(self):
        sec = P.fuselage_section(1.0)
        self.assertAlmostEqual(P.fuselage_z_at(1.0, 0.0, "bottom"), sec.z_bottom, delta=1e-6)
        z = P.fuselage_z_at(1.0, 0.05, "bottom")
        self.assertAlmostEqual(P.fuselage_half_width_at(1.0, z), 0.05, delta=1e-6)
        self.assertIsNone(P.fuselage_z_at(1.0, 0.2))


class TestWing(unittest.TestCase):
    def test_reference_planform_matches_spec(self):
        r, ref = P.wing_reference(), P.SPEC["wing"]["reference"]
        self.assertAlmostEqual(r["S"], ref["area_m2"], delta=0.0005)
        self.assertAlmostEqual(r["AR"], ref["aspect_ratio"], delta=0.01)
        self.assertAlmostEqual(r["MAC"], ref["mac_m"], delta=0.0002)
        self.assertAlmostEqual(r["y_mac"], ref["mac_y_m"], delta=0.0005)
        self.assertAlmostEqual(r["le_s_mac"], ref["mac_le_s_m"], delta=0.0005)
        self.assertAlmostEqual(2 * P.WING_SEMI_SPAN, P.SPEC["overall"]["span_m"], places=9)

    def test_planform_breakpoints(self):
        self.assertAlmostEqual(P.wing_chord_ref(0.5), 0.29)
        self.assertAlmostEqual(P.wing_chord_ref(1.0), 0.29)
        self.assertAlmostEqual(P.wing_chord_ref(1.9), 0.15)
        self.assertAlmostEqual(P.wing_station(0.1025).le_s, 1.061, delta=0.0005)      # glove gövde yanında
        self.assertAlmostEqual(P.wing_station(0.24).le_s, P.wing_le_ref(0.24), places=9)
        a, b = P.wing_station(1.83), P.wing_station(1.90)
        self.assertAlmostEqual(math.degrees(math.atan((b.le_s - a.le_s) / 0.07)), 36.0, delta=0.01)   # raked uç
        self.assertAlmostEqual(b.te_s, P.wing_te_s(1.9), places=9)
        parts = P.wing_parts()
        self.assertEqual(parts["U_WingOuter"], (0.36, 1.83))
        self.assertAlmostEqual(parts["U_WingOuter"][1] - parts["U_WingOuter"][0] + 0.07, 1.54)   # dış panel 1,54 m
        bp = P.wing_breakpoints()
        for y in (0.0, 0.1025, 0.24, 0.36, 1.0, 1.83, 1.9):
            self.assertIn(y, bp)

    def test_section_twist_dihedral_thickness(self):
        n = P.N_AIRFOIL
        for y, inc in ((0.5, 2.5), (1.0, 2.5), (1.45, 1.0), (1.9, -0.5)):
            sec = P.wing_section(y, te_thickness=0.0)
            self.assertEqual(sec.shape, (2 * n - 1, 3))
            le, te = sec[n - 1], 0.5 * (sec[0] + sec[-1])
            ang = math.degrees(math.atan2(le[2] - te[2], te[0] - le[0]))
            self.assertAlmostEqual(ang, inc, delta=0.02, msg=f"y={y}")
            self.assertTrue(np.allclose(sec[:, 1], y))
        self.assertAlmostEqual(P.wing_station(1.9).z_ref, 0.0829, delta=0.0002)
        self.assertAlmostEqual(P.wing_station(0.05).t_c, 0.155, delta=0.001)          # kök kalınlaşması
        self.assertAlmostEqual(P.wing_station(0.5).t_c, 0.140, delta=0.001)
        self.assertAlmostEqual(P.wing_station(1.9 - 0.007).t_c, AF.max_thickness(AF.section("sd7032"))[0], delta=0.001)
        tip = P.wing_section(1.9)
        np.testing.assert_allclose(tip[:n][::-1], tip[n - 1:], atol=1e-9)              # uç kapalı: kalınlık 0 (kamber kalır)
        sec = P.wing_section(0.6)
        self.assertAlmostEqual(sec[0, 2] - sec[-1, 2], 0.0015 * math.cos(math.radians(2.5)), delta=1e-4)   # 1,5 mm TE
        mirror = P.wing_section(-0.6)
        np.testing.assert_allclose(mirror[:, [0, 2]], sec[:, [0, 2]])
        np.testing.assert_allclose(mirror[:, 1], -sec[:, 1])

    def test_surface_queries(self):
        u, l = P.wing_surface_z(1.30, 0.5, "upper"), P.wing_surface_z(1.30, 0.5, "lower")
        self.assertGreater(u, l)
        self.assertAlmostEqual(P.wing_thickness_at(1.30, 0.5), u - l)
        self.assertIsNone(P.wing_surface_z(0.9, 0.5))

    def test_spar_tubes_inside_profile(self):
        for t in P.spar_tubes("L"):
            if t.name.startswith(("stab", "fin")):
                continue
            for p in (t.p0, t.p1):
                lo, up = P.wing_surface_z(p[0], p[1], "lower"), P.wing_surface_z(p[0], p[1], "upper")
                self.assertTrue(lo + t.od / 2 <= p[2] + 1e-9 <= up - t.od / 2 + 1e-9 or t.od <= up - lo, t.name)
        names = {t.name for t in P.spar_tubes("R")}
        self.assertTrue({"centre_socket", "panel_tube", "tip_tube", "rear_spar", "stab_spar", "fin_spar"} <= names)
        rear = [t for t in P.SPEC["print"]["spar_tubes"] if t["name"] == "rear_spar"][0]
        self.assertEqual(rear["chord_fraction"], P.SPEC["wing"]["rear_spar_chord_fraction"])
        cs = [t for t in P.SPEC["print"]["spar_tubes"] if t["name"] == "centre_socket"][0]
        self.assertEqual(cs["chord_fraction"], P.SPEC["wing"]["spar_chord_fraction"])


class TestTailAndSurfaces(unittest.TestCase):
    def test_tail_geometry(self):
        r = P.stab_station(0.52)
        self.assertAlmostEqual(r.le_s, 2.150, delta=0.0005)
        self.assertAlmostEqual(r.te_s, 2.269, delta=0.0005)
        f = P.fin_station(0.32)
        self.assertAlmostEqual(f.le[1], 0.587, delta=0.001)
        self.assertAlmostEqual(f.le[2], 0.378, delta=0.001)
        self.assertAlmostEqual(f.te_s, 2.475, delta=0.001)
        self.assertAlmostEqual(P.fin_station(0.32, "R").le[1], -f.le[1])
        sec = P.fin_section(0.1, "L")
        nrm = np.asarray(P.fin_station(0.1).normal)
        self.assertAlmostEqual(float(np.ptp(sec @ nrm)), 0.10 * P.fin_station(0.1).chord, delta=0.002)

    def test_hinge_lines_and_axes(self):
        hl = P.hinge_lines()
        self.assertEqual(len(hl), 10)
        self.assertEqual({h.obj_name for h in hl},
                         {f"U_{n}_{s}" for n in P.CONTROL_SURFACES for s in "LR"})
        for h in hl:
            mid_b = np.asarray(h.mid_b)
            te_b = mid_b + np.array([-h.chord_in, 0, 0])            # menteşenin arkasında bir nokta
            moved = P.rotate_about_axis(te_b, mid_b, h.axis_positive_b, 10.0)
            if h.kind == "fin":
                self.assertLess(moved[1], te_b[1], f"{h.obj_name}: + dönüş FK'yi sancağa götürmeli")
            else:
                self.assertLess(moved[2], te_b[2], f"{h.obj_name}: + dönüş FK'yi aşağı götürmeli")
            self.assertAlmostEqual(np.linalg.norm(h.axis_positive_b), 1.0)
            F = h.frame_b()
            np.testing.assert_allclose(F.T @ F, np.eye(3), atol=1e-9)
            self.assertAlmostEqual(np.linalg.det(F), 1.0, places=9)
            self.assertAlmostEqual(h.gap_m, 0.001)
        a = P.hinge_line("Aileron", "L")
        self.assertEqual((a.pos_max_deg, a.neg_max_deg), (12.0, 20.0))     # +12 aşağı / −20 yukarı
        self.assertAlmostEqual(a.chord_in, 0.28 * P.wing_station(1.10).chord, delta=1e-6)
        e = P.hinge_line("Elevator", "R")
        self.assertEqual((e.pos_max_deg, e.neg_max_deg), (20.0, 25.0))
        with self.assertRaises(KeyError):
            P.hinge_line("Spoiler")

    def test_control_surface_areas(self):
        cs = P.SPEC["wing"]["control_surfaces"]
        self.assertAlmostEqual(P.control_surface_area("Aileron"), cs["aileron"]["area_total_m2"], delta=0.001)
        self.assertAlmostEqual(P.control_surface_area("FlapIn") + P.control_surface_area("FlapOut"),
                               cs["flap_area_total_m2"], delta=0.002)


class TestGearPropTurret(unittest.TestCase):
    def test_static_contact_on_ground(self):
        for g in P.gear_legs():
            self.assertAlmostEqual(g.contact_static[2], P.GROUND_Z, places=9)
            self.assertGreater(g.static_sag, 0.0)
        self.assertAlmostEqual(P.gear_leg("L").static_sag, P.SPEC["landing_gear"]["main"]["static_sag_m"], delta=5e-4)

    def test_retract_axes(self):
        for g in P.gear_legs():
            ax = np.asarray(P.to_blender(*g.axle_unloaded))
            moved = P.rotate_about_axis(ax, g.pivot_b, g.retract_axis_b, g.retract_deg)
            np.testing.assert_allclose(P.from_blender(*moved), g.axle_retracted, atol=1e-12)
        n, l, r = P.gear_leg("N"), P.gear_leg("L"), P.gear_leg("R")
        self.assertGreater(n.axle_retracted[0], n.pivot[0] + 0.2)                  # burun geriye
        self.assertAlmostEqual(l.axle_retracted[1], 0.115, delta=0.001)          # ana içe
        self.assertAlmostEqual(r.axle_retracted[1], -0.115, delta=0.001)
        self.assertAlmostEqual(l.axle_retracted[2], -0.042, delta=1e-9)

    def test_wheels_fit_wells(self):
        res = sizing.gear_results(sizing.wing_results())
        self.assertGreater(min(res["main_well_margins"].values()), 0.002)
        self.assertGreater(min(res["nose_well_margins"].values()), 0.002)

    def test_doors_open_downward(self):
        doors = P.gear_doors()
        self.assertEqual({d.name for d in doors},
                         {"U_Door_N_1", "U_Door_N_2", "U_Door_L_1", "U_Door_R_1", "U_Door_L_2", "U_Door_R_2"})
        for d in doors:
            if d.attach != "skin":
                continue
            h0, h1 = np.asarray(P.to_blender(*d.hinge_p0)), np.asarray(P.to_blender(*d.hinge_p1))
            cen = np.mean(np.asarray(d.outline), axis=0)
            mid = 0.5 * (h0 + h1)
            pt = np.array([mid[0], cen[1], mid[2]])
            moved = P.rotate_about_axis(pt, mid, d.axis_open_b, 30.0)
            self.assertLess(moved[2], pt[2] - 1e-4, d.name)

    def test_gear_phase(self):
        self.assertEqual(P.gear_phase(0.0), {"legs": 0.0, "doors": 0.0})
        self.assertEqual(P.gear_phase(1.0), {"legs": 1.0, "doors": 0.0})
        self.assertEqual(P.gear_phase(0.5)["doors"], 1.0)
        self.assertEqual(P.gear_phase(0.15)["legs"], 0.0)

    def test_prop_and_turret(self):
        pr = P.PROP
        self.assertAlmostEqual(pr.diameter, 0.4064)
        self.assertLess(pr.axis_aft_b[0], 0)                                      # geriye
        self.assertAlmostEqual(math.degrees(math.asin(pr.axis_aft_b[2])), 5.0, places=9)
        bottom = pr.disk_point(0.0)
        self.assertGreater(bottom[0], pr.hub[0])                                  # disk tabanı geride
        self.assertAlmostEqual(P.thrust_line_z(P.CG.s), 0.044, delta=0.001)
        t = P.TURRET
        self.assertAlmostEqual(t.ground_clearance, 0.156, delta=0.0005)
        self.assertLess(t.ball_center[2] + t.ball_d / 2, t.belly_z + 0.03)       # yarı gömülü
        self.assertGreater(t.ball_center[2] + t.ball_d / 2, t.belly_z)

    def test_details(self):
        names = {f.obj_name for f in P.lights()}
        self.assertTrue({"U_Light_Nav_L", "U_Light_Nav_R", "U_Light_Strobe_L", "U_Light_Strobe_R"} <= names)
        nav = {f.name: f for f in P.lights()}
        self.assertGreater(nav["Nav_L"].pos[1], 1.83)
        self.assertEqual(nav["Nav_L"].params["color"], "red")
        self.assertEqual(P.pitot().obj_name, "U_Pitot")
        self.assertTrue(all(a.obj_name.startswith("U_Antenna_") for a in P.antennas()))
        self.assertEqual(P.hatch_outline().shape, (6, 2))
        cuts = P.print_cuts()
        self.assertAlmostEqual(cuts["wing_panel_y"][-2], 1.83)
        self.assertAlmostEqual(cuts["wing_panel_y"][-1], 1.90)
        self.assertIn(1.55, cuts["fuselage_s"])
        self.assertLessEqual(max(np.diff(cuts["fuselage_s"][1:-1])), 0.215 + 1e-9)


class TestSizing(unittest.TestCase):
    def test_sizing_check_passes(self):
        with tempfile.TemporaryDirectory() as d:
            out = Path(d) / "sizing.md"
            rc = sizing.main(["--check", "--quiet", "--out", str(out)])
            self.assertEqual(rc, 0, "sizing.py --check tolerans dışı değer buldu")
            text = out.read_text(encoding="utf-8")
        self.assertIn("boyutlandırma", text)
        self.assertIn("Statik marj", text)

    def test_key_numbers(self):
        R = sizing.compute()
        self.assertAlmostEqual(R["wing"]["S"], 0.976, delta=0.001)
        self.assertAlmostEqual(R["wing"]["AR"], 14.8, delta=0.02)
        self.assertAlmostEqual(R["wing"]["MAC"], 0.2646, delta=0.0005)
        self.assertAlmostEqual(R["stab"]["SM"], 10.0, delta=0.5)
        self.assertGreaterEqual(R["gear"]["prop_strike"], 13.0)
        self.assertAlmostEqual(R["tail"]["V_h"], 0.475, delta=0.005)
        checks = sizing.build_checks(R)
        self.assertTrue(all(c.ok for c in checks), [c.label for c in checks if not c.ok])


if __name__ == "__main__":
    unittest.main()
