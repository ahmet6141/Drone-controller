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
        self.assertGreater(n.axle_retracted[0], n.pivot[0] + 0.18)                 # burun geriye
        self.assertAlmostEqual(l.axle_retracted[1], 0.115, delta=0.001)          # ana içe
        self.assertAlmostEqual(r.axle_retracted[1], -0.115, delta=0.001)
        ref = P.SPEC["landing_gear"]["reference"]["retracted_main_wheel"]
        self.assertAlmostEqual(l.axle_retracted[2], float(ref[2]), delta=1e-9)      # R14: aks 3 mm dışa → 3 mm alçak

    def test_wheels_fit_wells(self):
        res = sizing.gear_results(sizing.wing_results())
        self.assertGreater(min(res["main_well_margins"].values()), 0.002)
        self.assertGreater(min(res["nose_well_margins"].values()), 0.002)

    def test_doors_open_downward(self):
        doors = P.gear_doors()
        self.assertEqual({d.name for d in doors},
                         {"U_Door_N_1", "U_Door_N_2", "U_Door_N_3", "U_Door_L_1", "U_Door_R_1", "U_Door_L_2",
                          "U_Door_R_2"})
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
        self.assertAlmostEqual(P.thrust_line_z(P.CG.s),
                               P.SPEC["propulsion"]["performance"]["thrust_line_z_at_cg_m"], delta=0.001)
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


class TestGeometryFixes(unittest.TestCase):
    """Tasarım incelemesi düzeltmelerinin (geometri) gerilemeye karşı kilitleri: motor bölmesi zarfları, sırt çizgisi,
    stabilize kirişi, düz longeronlar, ana takım ünitesi, NACA dudağı, kapak, flap aralığı, işaretler."""

    @classmethod
    def setUpClass(cls):
        from ucav import shapes as S
        cls.S = S

    def test_engine_envelopes_inside_cowl(self):
        """Motor zarflarının kaporta KABUĞUNA 3B payı (R01: dik çene kapanışı, ön dudak ve arka flanş dahil; eski
        kesit-içi 2B ölçüm arka fincanı ve çeneyi görmüyordu): her motor parçası ≥ 5 mm, susturucu ≥ 15,5 mm."""
        cl = P.SPEC["propulsion"]["engine"]["clearance_m"]
        r = self.S.engine_bay_clearance()
        for part in ("front_bearing", "crankcase", "carb", "cylinder", "spark_cap"):
            self.assertGreaterEqual(r[part], float(cl["cowl"]) - 1e-4, part)
        self.assertGreaterEqual(r["min_value"], float(cl["cowl"]) - 1e-4)
        self.assertGreaterEqual(r["muffler"], float(cl["muffler_air_gap"]) - 1e-4)
        self.assertGreaterEqual(r["carb_to_firewall"], float(cl["carb_to_firewall"]) - 1e-4)
        self.assertGreaterEqual(r["spinner_to_ring"], 0.005 - 1e-4)

    def test_cowl_shell_no_envelope_overlap(self):
        """R01: sahne kaportası gerçek kabuktur (kapalı, manifold); hiçbir kabuk üçgeni (köşe + kenar ortası + ağırlık
        merkezi örnekleri) motor/susturucu zarfının içine girmez (Blender BVH çakışmasının numpy karşılığı); baskı
        katısı düz kapaklı kapalı katıdır (fincan/taban YOK); koyu boşluk diski yalnız render içindir ve s ≥ 2,19."""
        S = self.S
        shell = S.cowl()
        chk = shell.check()
        self.assertEqual((chk["boundary"], chk["nonmanifold"], chk["flipped"]), (0, 0, 0))
        tris = S._tri_array(shell)
        samples = np.vstack([tris.reshape(-1, 3), tris.mean(1), 0.5 * (tris[:, 0] + tris[:, 1]),
                             0.5 * (tris[:, 1] + tris[:, 2]), 0.5 * (tris[:, 2] + tris[:, 0])])
        for part in P.engine_envelope():
            self.assertFalse(S._inside_env(part, samples).any(), f"kaporta kabuğu {part.name} zarfına giriyor")
        solid = S.cowl(print_solid=True)
        self.assertEqual(solid.check()["boundary"], 0)
        ss, O = S._cowl_grid()
        self.assertEqual(len(solid.verts), O.shape[0] * O.shape[1])          # yalnız dış yüz halkaları: iç fincan yok
        s0, s_end = S._cowl_s_range()
        self.assertAlmostEqual(float(np.asarray(solid.verts)[:, 0].max()), s_end, places=9)
        cav = np.asarray(S.cowl_cavity().verts)
        self.assertGreaterEqual(float(cav[:, 0].min()), s_end)
        for part in P.engine_envelope():
            self.assertFalse(S._inside_env(part, cav).any(), part.name)

    def test_engine_keepouts_for_print(self):
        """R01 (baskı): yuvarlak motor yasak bölgeleri kapalı; yüzleri zarfa ≥ pay (5 mm; susturucu 15,5 mm), sahne
        kabuğuna değmez → printprep bunları UP_cowl_* parçalarından çıkarınca çakışma 0, kabuk kesilmez."""
        S = self.S
        cl = P.SPEC["propulsion"]["engine"]["clearance_m"]
        tris = S._tri_array(S.cowl())
        shell = np.vstack([tris.reshape(-1, 3), tris.mean(1)])
        for md, part in zip(S.engine_keepouts(), P.engine_envelope()):
            c = md.check()
            self.assertEqual((c["boundary"], c["nonmanifold"], c["flipped"]), (0, 0, 0), md.name)
            A, h = np.asarray(part.axes, float), np.asarray(part.half, float)

            def dist(Q):
                q = (Q - np.asarray(part.center)) @ A.T
                if part.kind == "cyl":
                    return np.hypot(np.maximum(np.abs(q[:, 0]) - h[0], 0.0),
                                    np.maximum(np.hypot(q[:, 1], q[:, 2]) - h[1], 0.0))
                return np.linalg.norm(np.maximum(np.abs(q) - h, 0.0), axis=1)
            kt = S._tri_array(md)
            dk = dist(np.vstack([kt.reshape(-1, 3), kt.mean(1)]))
            need = float(cl["muffler_air_gap"] if part.name == "muffler" else cl["cowl"])
            self.assertGreaterEqual(float(dk.min()), need, md.name)
            self.assertGreater(float(dist(shell).min()), float(dk.max()), md.name)

    def test_cooling_exit_open_and_sized(self):
        """R01: soğutma çıkışı gerçekten açık — arka flanş Ø92 açık, çene yarığı kabuğu boydan boya keser; çıkış alanı
        (min(lüle, flanş açıklığı) + çene) NACA ağzının ≥ 1,5 katı; ağız ≈ 22 cm² (spec)."""
        S = self.S
        ex = S.cooling_exit_check()
        self.assertTrue(ex["through"], ex)
        self.assertGreater(ex["min_overlap_m"], 0.002)
        a = S.cooling_flow_areas()
        self.assertAlmostEqual(a["inlet"] * 1e4, float(P.SPEC["propulsion"]["intake"]["area_cm2"]), delta=1.5)
        self.assertGreaterEqual(a["exit"], 1.5 * a["inlet"])
        self.assertGreaterEqual(a["aft_opening"], a["ring"])                   # dar kesit lüle halkasıdır, motor değil
        self.assertAlmostEqual(S.cowl_open_r(), 0.5 * float(P.SPEC["propulsion"]["exhaust_ring"]["id_m"]), places=9)
        # baskı açıklık katısı flanşı boydan boya keser
        s_end = S._cowl_s_range()[1]
        op = np.asarray(S.cowl_aft_opening().verts)
        self.assertLess(float(op[:, 0].min()), s_end - S.COWL_WALL)
        self.assertGreater(float(op[:, 0].max()), s_end)

    def test_muffler_clear_of_stab(self):
        """R05: susturucu zarfı stabilize + elevatöre (±25° sapma dahil) ≥ spec ``body_min``; çıkış borusu ve ısı
        kalkanı ≥ ``outlet_min`` (60 mm); boru kaporta yanağından dışarı çıkar ve aşağı-dışa-geriye bakar."""
        S = self.S
        lim = P.SPEC["propulsion"]["muffler"]["stab_clearance_m"]
        mf = next(p for p in P.engine_envelope() if p.name == "muffler")
        stab = S.stab_parts()
        h = P.hinge_line("Elevator", "L")
        p0, ax = np.asarray(h.p_in), np.asarray(h.p_out) - np.asarray(h.p_in)
        ax = ax / np.linalg.norm(ax)

        def rot(V, ang):
            a, r = math.radians(ang), V - p0
            return p0 + r * math.cos(a) + np.cross(ax, r) * math.sin(a) + np.outer(r @ ax, ax) * (1 - math.cos(a))

        EV = np.asarray(stab["U_Elevator_L"].verts)
        VS = np.asarray(stab["U_Stab"].verts)
        V = np.vstack([VS[VS[:, 1] > 0], EV, rot(EV, 25.0), rot(EV, -25.0)])
        A = np.asarray(mf.axes, float)
        q = (V - np.asarray(mf.center)) @ A.T
        d_body = float(np.linalg.norm(np.maximum(np.abs(q) - np.asarray(mf.half), 0.0), axis=1).min())
        self.assertGreaterEqual(d_body, float(lim["body_min"]) - 1e-4)
        d_out = float(S.points_tris_distance(V, S._tri_array(S.muffler_pipe())).min())
        self.assertGreaterEqual(d_out, float(lim["outlet_min"]))
        p_start, d, t_exit, L = S.muffler_pipe_axis()
        self.assertGreater(t_exit, 0.005)
        self.assertGreater(L, t_exit + 0.010)                                 # boru yanaktan ≥ 10 mm dışarı taşar
        self.assertLess(d[2], -0.5)
        self.assertGreater(d[0], 0.2)
        self.assertGreater(d[1], 0.2)

    def test_gear_doors_lip_and_fine_teeth(self):
        """R06: testere dişi derinliği ≤ 3 mm (36° yanlar); ana ve burun kapaklarının menteşe dışı kenarlarında 2,5 mm
        dönüş dudağı (kenar kalınlığı okunur)."""
        S = self.S
        self.assertLessEqual(P.SAWTOOTH["depth_m"], 0.003 + 1e-9)
        for name in ("U_Door_L_1", "U_Door_N_1"):
            d = next(x for x in P.gear_doors() if x.name == name)
            md = S.gear_door(name).mesh
            self.assertEqual(md.check()["boundary"], 0, name)
            V = np.asarray(md.verts)
            zf = S._skin_z_for_leg(d.leg)
            t = np.array([z - zf(s, y) for s, y, z in V])
            self.assertGreater(float(np.sum(np.abs(t - S.DOOR_LIP["t"]) < 2e-4)), 40, name)

    def test_turret_collar_round_with_fillet(self):
        """R07: yaka ≥ 24 faset; karına iç bükey fileto ile bağlanır (yaka yüzü karın derisine yaklaşırken yarıçap
        açılır, kenar basamağı yok)."""
        S = self.S
        t = P.TURRET
        self.assertGreaterEqual(int(t.collar_facets), 24)
        md = S.turret_mount()
        self.assertEqual(md.check()["boundary"], 0)
        V = np.asarray(md.verts)
        r = np.hypot(V[:, 0], V[:, 1])
        Rb = 0.5 * t.collar_d
        for a in np.linspace(0, 2 * math.pi, 12, endpoint=False):
            c, s_ = math.cos(a), math.sin(a)
            sel = (np.abs(np.arctan2(V[:, 1], V[:, 0]) - math.atan2(s_, c) + math.pi) % (2 * math.pi) - math.pi) < 0.02
            zb = S._belly_local(t, c, s_, Rb + 0.004)
            near = sel & (V[:, 2] < zb + 0.0005) & (V[:, 2] > zb - 0.0015)
            if near.any():
                self.assertGreater(float(r[near].max()), Rb + 0.002)

    def test_hatch_frame_width(self):
        """R12: aviyonik kapağı çerçeve bandı 6 mm."""
        self.assertGreaterEqual(self.S.HATCH_FRAME["w"], 0.006 - 1e-9)

    def test_main_tyre_full_width(self):
        """R14: spec lastik eni (26 mm) toplu konumda kuyuya sığar — tavana (kanat üst derisiyle sınırlı) ≥ 2,5 mm ve
        kapalı kapağın iç yüzüne (tava, boyuna boncuk, göbek boşluğu; taşan jant göbeği dahil) ≥ 2,5 mm (teker 3 mm
        dışa kaçık aksla 3 mm alçak, kanoe tabanı 3 mm derin)."""
        S = self.S
        for side in ("L", "R"):
            g = P.gear_leg(side)
            self.assertGreaterEqual(S.tyre_fit_width(side), g.wheel_w - 1e-9, side)
            self.assertGreaterEqual(S.stowed_tyre_door_gap(side), S.TYRE_CLEAR - 1e-4, side)
            self.assertAlmostEqual(abs(g.contact_static[1]), float(P.SPEC["overall"]["wheel_track_m"]) / 2, delta=1e-6)

    def test_fin_root_bullet_proud(self):
        """Dikey kökü mermisi dikey kök kesiti ve stabilize uç kesitinden her istasyonda ≥ 1,5 mm taşar (basılabilir)."""
        S = self.S
        md = S.fin_root_fairing("L")
        self.assertEqual(md.check()["boundary"], 0)
        V = np.asarray(md.verts)
        y0, z0 = float(P.SPEC["tail"]["fin"]["y_root_m"]), float(P.SPEC["tail"]["fin"]["z_root_m"])
        fin, stab = P.fin_section(0.0, "L"), P.stab_section(y0)
        ss = np.linspace(2.16, float(fin[:, 0].max()) - 0.01, 25)
        need = np.maximum(S._section_half_thickness(fin, ss), S._section_half_thickness(stab, ss))
        for s, n in zip(ss, need):
            ring = V[np.abs(V[:, 0] - s) < 0.003]
            if len(ring):
                self.assertGreaterEqual(float(np.hypot(ring[:, 1] - y0, ring[:, 2] - z0).max()), n + 0.0015 - 2e-4, s)

    def test_scuff_pad_first_contact(self):
        """Kaporta altı pabucu (PA-CF, ısıya dayanıklı) kuyruk çarpmasında GERÇEK kaporta ağından (yanak kabartısı
        dahil), lüle halkası, susturucu borusu ve gövde/NACA dudağından önce değer; dış yüzü kaporta alt yüzünden
        ``t_m`` kabarık (eski pabuç gövde kesitine göre konduğu için kabartılı kaporta yüzünün içinde kalıyordu)."""
        S = self.S
        sp = P.SPEC["propulsion"]["cowl"]["scuff_pad"]
        self.assertNotEqual(str(sp["material"]).upper(), "TPU")
        sm = P.gear_leg("L").contact_static[0]

        def ang(md):
            V = np.asarray(md.verts, float)
            return float(np.degrees(np.arctan2(V[:, 2] - P.GROUND_Z, V[:, 0] - sm)).min())
        pad = ang(S.scuff_pad())
        for nm, md in (("kaporta", S.cowl()), ("lüle", S.exhaust_ring()), ("susturucu borusu", S.muffler_pipe()),
                       ("gövde", S.fuselage()), ("NACA", S.intake()), ("panjur", S.cowl_louvers())):
            self.assertLess(pad, ang(md) - 0.05, nm)
        V = np.asarray(S.scuff_pad().verts, float)
        t = float(sp["t_m"])
        low = [z - S.cowl_bottom_z(s, y) for s, y, z in V[::7] if S.cowl_bottom_z(s, y) is not None]
        self.assertAlmostEqual(min(low), -t, delta=2e-4)                  # aşınma yüzü kaporta yüzünden t aşağıda
        self.assertTrue(S.scuff_pad().check()["boundary"] == 0)

    def test_spine_taut_no_hump(self):
        """Kanattan lüle halkasına sırt çizgisi tek yönlü yükselir (eski 'deve hörgücü' yok), eğim ≤ 8°."""
        S = self.S
        s_cowl = float(P.SPEC["propulsion"]["cowl"]["s_from_m"])
        s_ring = float(P.SPEC["propulsion"]["exhaust_ring"]["s_from_m"])
        ss = np.linspace(1.0, s_ring - 0.002, 80)
        z = np.array([P.fuselage_section(s).z_top if s < s_cowl else S.cowl_section_yz(s)[:, 1].max() for s in ss])
        self.assertTrue(np.all(np.diff(z) >= -2e-4), "sırt çizgisinde yerel tepe")
        self.assertLess(math.degrees(math.atan(np.max(np.diff(z) / np.diff(ss)))), 8.0)

    def test_stab_spar_fit(self):
        f = P.stab_spar_fit()
        self.assertGreaterEqual(f["margin_min"], 0.0008)
        self.assertGreaterEqual(f["skin_min"], 0.0005)

    def test_longerons_straight_and_inside(self):
        L = P.SPEC["fuselage"]["longerons"]
        br = [float(L["s_from_m"])] + [float(b) for b in L["breaks_s_m"]] + [float(L["s_to_m"])]
        r = 0.5 * float(L["od_m"])
        for kind in ("chine", "shoulder"):
            pcs = P.longeron_pieces(kind, "L")
            self.assertEqual([round(float(p0[0]), 6) for p0, _ in pcs] + [round(float(pcs[-1][1][0]), 6)], br)
            for (a0, a1), (b0, _) in zip(pcs[:-1], pcs[1:]):
                np.testing.assert_allclose(a1, b0, atol=1e-12)                       # kırıkta süreklilik
            path = P.longeron_path(kind, "L", n=60)
            for p in path:                                                          # yol = parçaların doğruları
                seg = next((p0, p1) for p0, p1 in pcs if p0[0] - 1e-9 <= p[0] <= p1[0] + 1e-9)
                d = np.linalg.norm(np.cross(seg[1] - seg[0], p - seg[0])) / np.linalg.norm(seg[1] - seg[0])
                self.assertLess(d, 1e-9)
                self.assertGreater(P.fuselage_half_width_at(float(p[0]), float(p[2])) - p[1], r, kind)

    def test_main_unit_clears_spar_tubes(self):
        S = self.S
        for side in ("L", "R"):
            box, top_margin = S.main_unit_box(side)
            self.assertGreaterEqual(top_margin, 0.0)
            pts = box.surface_points(16)
            for t in P.spar_tubes(side):
                if t.name not in ("centre_socket", "panel_tube", "inner_reinforce", "rear_spar", "incidence_pin"):
                    continue
                d = _seg_dist(pts, np.array([t.p0, t.p1])).min() - 0.5 * t.od
                self.assertGreater(d, 0.002, f"{side} {t.name}")
            self.assertGreater(box.center[0] - box.half[0], P.gear_leg(side).pivot[0])   # ünite bacak yuvasının arkasında

    def test_nose_gear_trail_and_flap_gap(self):
        self.assertGreaterEqual(P.gear_leg("N").trail, 0.010)
        self.assertLessEqual(P.gear_leg("N").trail, 0.015)
        cs = P.SPEC["wing"]["control_surfaces"]
        gap = float(cs["flap_out"]["y_from_m"]) - float(cs["flap_in"]["y_to_m"])
        self.assertGreaterEqual(gap, 0.008)
        self.assertLessEqual(gap, 0.012)

    def test_hatch_flush(self):
        h = P.SPEC["details"]["hatch"]
        self.assertLessEqual(float(h["bulge_m"]) + self.S.HATCH_FACET["rim"], 0.002)
        o = P.hatch_outline()
        area = 0.5 * abs(np.dot(o[:, 0], np.roll(o[:, 1], -1)) - np.dot(o[:, 1], np.roll(o[:, 0], -1)))
        self.assertLessEqual(area, 0.040)

    def test_naca_lip_follows_belly(self):
        """NACA boğaz çerçevesi karın derisinin en çok 1 mm altına iner (dikdörtgen köşeler dışarı taşmaz)."""
        S = self.S
        m = S.intake()
        self.assertEqual(m.check()["boundary"], 0)
        V = np.asarray(m.verts)
        for s, y, z in V:
            self.assertGreater(z, S._skin_bottom(float(s), float(y)) - 0.001)

    def test_prop_swept_gap_and_exit_area(self):
        """Elevatör firar kenarı ↔ pala süpürme hacmi (pervane yerel ağı döndürülerek, (eksenel, yarıçap) düzleminde)
        ≥ 0,25 D ve spec ``gap_swept_min_m`` ile tutarlı; lüle akış alanı = π/4·(ID² − spinner²)."""
        pr = P.PROP
        hub, ax = np.asarray(pr.hub), np.asarray(pr.axis_aft)
        V = np.asarray(self.S.prop().verts)
        bx, br = V[:, 0], np.hypot(V[:, 1], V[:, 2])
        z = float(P.SPEC["tail"]["stab"]["z_m"])
        best = 1.0
        for y in np.linspace(0.0, 0.6, 121):
            d = np.array([P.stab_station(float(y)).te_s, y, z]) - hub
            a = float(d @ ax)
            r = float(np.linalg.norm(d - a * ax))
            best = min(best, float(np.min(np.hypot(bx - a, br - r))))
        self.assertGreaterEqual(best, 0.25 * pr.diameter)
        self.assertAlmostEqual(best, float(P.SPEC["tail"]["reference"]["gap_swept_min_m"]), delta=0.002)
        er = P.SPEC["propulsion"]["exhaust_ring"]
        flow = math.pi / 4 * (float(er["id_m"]) ** 2 - pr.spinner_d ** 2) * 1e4
        self.assertAlmostEqual(flow, float(er["area_cm2"]), delta=0.01 * flow)
        ann = math.pi / 4 * (float(er["od_m"]) ** 2 - float(er["id_m"]) ** 2) * 1e4
        self.assertAlmostEqual(ann, float(er["annulus_cm2"]), delta=0.01 * ann)

    def test_stencils_named_and_hosted(self):
        st = self.S.stencil_specs()
        names = [x.name for x in st]
        self.assertEqual(len(names), len(set(names)))
        self.assertGreaterEqual(len(st), 20)
        self.assertTrue(all(n.startswith("U_Stencil_") and x.host.startswith("U_") for n, x in zip(names, st)))
        for side in ("L", "R"):
            for host in ("Aileron", "Elevator", "FlapIn", "FlapOut"):
                self.assertIn(f"U_Stencil_AdimAtma_{host}_{side}", names)


class TestReadme(unittest.TestCase):
    """``ucav/README.md`` güncel: kapsam notu ve bağlantılar, giriş tipi, kontrol sayısı, klip süreleri spec/koda uyar."""

    @classmethod
    def setUpClass(cls):
        cls.path = ROOT / "ucav" / "README.md"
        cls.text = cls.path.read_text(encoding="utf-8")

    def _has(self, needle: str) -> None:
        self.assertTrue(needle in self.text, f"README'de yok: {needle!r}")

    def test_scope_note_and_links(self):
        for w in ("Kapsam", "sivil", "Silah", "mühimmat", "askı noktası", "pilon", "bırakma mekanizması"):
            self._has(w)
        import re
        links = re.findall(r"\]\(([^)#\s]+)\)", self.text)
        self.assertTrue(links)
        for rel in links:
            if rel.startswith(("http://", "https://")):
                continue
            self.assertTrue((self.path.parent / rel).exists(), f"README bağlantısı yok: {rel}")
        for need in ("spec.yaml", "out/sizing.md", "out/print_report.md", "out/anim/showcase.mp4",
                     "out/anim/mechanisms.mp4", "../docs/08-guvenlik-ve-mevzuat.md"):
            self.assertIn(need, links)

    def test_matches_spec_and_sizing(self):
        if P.SPEC["propulsion"]["intake"]["type"] == "naca_flush_ventral":
            self._has("NACA")
            self.assertFalse("Sırtta NACA" in self.text, "README eski sırt girişini anlatıyor")
        C = sizing.build_checks(sizing.compute())
        n = sum(c.is_check for c in C)
        self._has(f"{n}/{n} kontrol")
        self.assertFalse("⟪" in self.text, "README'de doldurulmamış yer tutucu var")
        for frames, secs in ((504, 21), (384, 16)):              # animation.CLIPS × 24 fps
            self._has(f"{secs} s, {frames} kare")


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
