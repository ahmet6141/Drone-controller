"""YK-250 HANCER sizing: spec.yaml v1 schema, derived-value check, requirements, OML meshes, civil scope.

Runs ``python3 -m ucav250.analysis.sizing --check --no-figures --out <scratch dir>`` once (about 70 s; the scratch
directory keeps the repository outputs untouched) and inspects the spec and the written sizing.json."""
from __future__ import annotations

import importlib.util
import json
import math
import re
import sys
import tempfile
import unittest
from pathlib import Path

import numpy as np

REPO = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO))
try:
    import yaml
    from ucav250.analysis import sizing as Z
    from ucav250.core import spec as SPEC
    from ucav250.design import oml
    HAVE = True
except Exception:  # pragma: no cover
    HAVE = False

SPEC_FILE = REPO / "ucav250" / "spec.yaml"
BANNED = ("weapon", "hardpoint", "hard point", "hard_point", "hard-point", "pylon", "munition", "release", "bomb",
          "missile", "warhead", "silah", "mühimmat", "muhimmat")


@unittest.skipUnless(HAVE, "ucav250 dependencies missing")
class TestSizingCheck(unittest.TestCase):
    """The sizing CLI in --check mode: every derived spec value within tolerance and every requirement met."""
    rc = None

    @classmethod
    def setUpClass(cls):
        SPEC.load.cache_clear()
        cls.tmp = tempfile.TemporaryDirectory()
        cls.out_dir = Path(cls.tmp.name)
        cls.rc = Z.main(["--check", "--no-figures", "--out", str(cls.out_dir)])
        cls.out = json.loads((cls.out_dir / "sizing.json").read_text(encoding="utf-8"))
        cls.S = SPEC.load()

    @classmethod
    def tearDownClass(cls):
        cls.tmp.cleanup()

    def test_outputs_in_scratch_dir_and_reproducible(self):
        """--out keeps the repository outputs untouched; the written files carry no run time (repeatable)."""
        self.assertTrue((self.out_dir / "sizing.md").exists())
        self.assertNotIn("runtime_s", self.out["meta"])
        self.assertNotIn("Çalışma süresi", (self.out_dir / "sizing.md").read_text(encoding="utf-8"))

    def test_check_exit_code_zero(self):
        self.assertEqual(self.rc, 0)

    def test_derived_values_within_tolerance(self):
        chk = self.out["derived_check"]
        self.assertGreater(chk["n_compared"], 500)
        self.assertEqual(chk["n_bad"], 0, chk["bad"][:10])

    def test_all_requirements_pass(self):
        reqs = self.out["requirements"]
        self.assertEqual(len(reqs), len(self.S["requirements"]))
        failed = [r for r in reqs if not r["pass"]]
        self.assertEqual(failed, [])

    def test_requirement_metrics_exist(self):
        M = self.out["metrics"]
        for r in self.S["requirements"]:
            self.assertIn(r["metric"], M, r["id"])
            self.assertIn(r["op"], Z.OPS)
            self.assertTrue(r.get("source"), r["id"])
            self.assertTrue(r.get("text_tr"), r["id"])

    def test_mtom_below_150_kg(self):
        self.assertLess(self.S["mass"]["mtow_kg"], 150.0)
        self.assertLess(self.out["metrics"]["mtow_kg"], 150.0)
        self.assertLessEqual(self.S["mass"]["mtow_kg"], self.S["mass"]["mtow_cap_kg"])

    def test_endurance_and_payload_requirement(self):
        M = self.out["metrics"]
        self.assertGreaterEqual(M["endurance_h"], 10.0)
        self.assertGreaterEqual(M["payload_kg"], 20.0)

    def test_new_checks_reported(self):
        """Verification-finding checks (take-off physics, 925(a)/(c) clearances, stabilator hinge and roots, fin roots,
        spar depth, gust matrix, electrical budget) are in the outputs."""
        M = self.out["metrics"]
        for k in ("takeoff_main_gear_load_at_rotation_N", "prop_clear_min_925a_m", "prop_clear_radial_m",
                  "prop_clear_longitudinal_m", "stab_trim_cl_local_max", "stab_hinge_peak_margin", "stab_root_gap_m",
                  "stab_root_body_clearance_m", "fin_root_max_gap_m", "spar_depth_main_ratio", "spar_depth_rear_ratio",
                  "wing_root_chine_step_m", "flap_inboard_end_outboard_of_joint_m", "generator_margin_loiter"):
            self.assertIn(k, M, k)
        to = self.out["performance"]["takeoff_sl_mtow"]
        self.assertEqual(len(to["cases"]), sum(1 for c in self.out["mass"]["ground_cases_gear_down"]
                                               if abs(c["m"] - self.out["mass"]["mtow_kg"]) < 0.5))
        self.assertGreaterEqual(to["V_lof_m_s"], 1.1 * to["VS_TO_m_s"] - 1e-6)
        self.assertGreater(to["V_lof_m_s"], to["V_R_m_s"])
        gm = self.out["loads"]["gust_matrix"]
        self.assertGreaterEqual(len(gm), 6)
        self.assertGreater(self.out["loads"]["cl_alpha_used"], self.out["aero"]["CL_alpha_vlm"])
        g = self.out["ground"]
        self.assertAlmostEqual(g["prop_clear_min_925a"], min(g["prop_clear_static"], g["prop_clear_liftoff"],
                                                             g["prop_clear_touchdown_unloaded"]), places=9)

    def test_report_written_in_turkish(self):
        md = (self.out_dir / "sizing.md").read_text(encoding="utf-8")
        for w in ("Gereksinim uyumu", "Kütle", "Kararlılık", "Performans", "İniş takımı"):
            self.assertIn(w, md)


@unittest.skipUnless(HAVE, "ucav250 dependencies missing")
class TestSpecSchema(unittest.TestCase):
    """ARCHITECTURE.md §9 keys and the internal consistency of the derived blocks."""

    @classmethod
    def setUpClass(cls):
        with open(SPEC_FILE, encoding="utf-8") as fh:
            cls.S = yaml.safe_load(fh)

    def test_top_level_keys(self):
        for k in ("meta", "requirements", "mission", "engine", "propeller", "configuration", "wing", "tail", "fuselage",
                  "landing_gear", "payload", "aero", "mass", "stability", "performance", "structures", "materials",
                  "layups", "processes", "display", "layout", "assembly"):
            self.assertIn(k, self.S, k)
        for k in ("name", "project", "revision", "date", "description_tr", "scope_tr"):
            self.assertIn(k, self.S["meta"])
        self.assertEqual(self.S["meta"]["name"], "YK-250 HANÇER")

    def test_block_keys(self):
        W = self.S["wing"]
        for k in ("span", "area", "aspect_ratio", "taper", "sweep_c4_deg", "dihedral_deg", "incidence_deg", "washout_deg",
                  "airfoils", "sections", "mac", "mac_le_x", "mac_y", "controls"):
            self.assertIn(k, W, k)
        for c in ("aileron", "flap"):
            for k in ("eta0", "eta1", "xc_hinge", "range_deg"):
                self.assertIn(k, W["controls"][c])
        T = self.S["tail"]
        for k in ("type", "surfaces", "volume_h", "volume_v", "arm_h", "arm_v"):
            self.assertIn(k, T, k)
        for s in ("stabilator", "stabilator_stub", "fin", "ventral"):
            for k in ("sections", "mirror", "area", "span"):
                self.assertIn(k, T["surfaces"][s], (s, k))
        sc = T["surfaces"]["stabilator"]["controls"]                     # ARCHITECTURE.md §9 tail controls (F15)
        for k in ("range_deg", "linkage_ratio", "actuator", "ac_mac_fraction_range", "checks"):
            self.assertIn(k, sc, k)
        self.assertIn("rudder", T["surfaces"]["fin"]["controls"])
        for k in ("xc_hinge", "range_deg", "chord_fraction"):
            self.assertIn(k, T["surfaces"]["fin"]["controls"]["rudder"], k)
        A = self.S["aero"]                                                # ARCHITECTURE.md §9 aero keys (F15)
        for k in ("cd0", "e", "k", "clmax_clean", "clmax_to", "clmax_ld", "ld_max", "polars"):
            self.assertIn(k, A, k)
        for c in self.S["mass"]["cases"]:
            self.assertIn("payload", c, c["name"])
        self.assertIn("gust_matrix", self.S["structures"]["derived"])
        self.assertIn("clearance_checks", self.S["propeller"])
        F = self.S["fuselage"]
        for k in ("length", "width_max", "height_max", "stations"):
            self.assertIn(k, F)
        st = np.asarray(F["stations"], float)
        self.assertEqual(st.shape[1], 7)
        self.assertTrue(np.all(np.diff(st[:, 0]) > 0))
        M = self.S["mass"]
        for k in ("mtow_kg", "empty_kg", "fuel_kg", "payload_kg", "budget", "cases", "items"):
            self.assertIn(k, M)
        for g, b in M["budget"].items():
            self.assertIn("target_kg", b)
            self.assertIn("tol_kg", b)
        SB = self.S["stability"]
        for k in ("mac", "mac_le_x", "np_x", "cg_design", "cg_range_x", "static_margin_range"):
            self.assertIn(k, SB)
        self.assertEqual(len(SB["cg_design"]), 3)
        LG = self.S["landing_gear"]
        for k in ("type", "tyre", "main", "nose", "ground_z", "wheelbase", "checks"):
            self.assertIn(k, LG)
        for k in ("n_limit_pos", "n_limit_neg", "factor_of_safety", "fitting_factor", "VC_eas", "VD_eas", "gust"):
            self.assertIn(k, self.S["structures"])
        self.assertAlmostEqual(self.S["structures"]["n_limit_pos"], 3.8)
        self.assertAlmostEqual(self.S["structures"]["factor_of_safety"], 1.5)
        for k in ("propulsion", "tail", "wing_position", "gear", "fuselage_style", "transport_breakdown", "reasons_tr"):
            self.assertIn(k, self.S["configuration"])

    def test_wing_sections_reproduce_planform(self):
        """Outer-panel sections lie on the reference trapezoid; the spec area/span/MAC are the trapezoid values."""
        W = self.S["wing"]
        P = W["planform"]
        T = Z.trapezoid(P)
        self.assertAlmostEqual(W["area"], P["area"], places=6)
        self.assertAlmostEqual(W["span"], P["span"], places=6)
        self.assertAlmostEqual(W["aspect_ratio"], P["span"] ** 2 / P["area"], places=4)
        self.assertAlmostEqual(W["mac"], T["mac"], places=4)
        self.assertAlmostEqual(W["mac_le_x"], T["x_le_mac"], places=4)
        self.assertAlmostEqual(W["mac_y"], T["y_mac"], places=4)
        secs = W["sections"]
        self.assertAlmostEqual(secs[-1]["y"], W["span"] / 2, places=5)
        outer = [s for s in secs if s["y"] >= P["y_junction"] - 1e-9]
        self.assertGreaterEqual(len(outer), 3)
        for s in outer:
            self.assertAlmostEqual(s["chord"], float(T["c"](s["y"])), delta=1e-3)
            self.assertAlmostEqual(s["x_le"], float(T["xle"](s["y"])), delta=1e-3)
        # trapezoid area from the outer chords extended to the centre line
        ys = np.linspace(0.0, W["span"] / 2, 200)
        self.assertAlmostEqual(2 * np.trapz(T["c"](ys), ys), W["area"], delta=1e-4)
        # LERX: monotonic leading edge in span, root at the chine, actual planform larger than the trapezoid
        y = np.array([s["y"] for s in secs])
        self.assertTrue(np.all(np.diff(y) > 0))
        sa = oml.mean_aerodynamic_chord(secs)
        self.assertGreater(2 * sa["half_area"] + 2 * secs[0]["y"] * secs[0]["chord"], W["area"])

    def test_tail_sections_reproduce_areas(self):
        for k, v in self.S["tail"]["surfaces"].items():
            pp = Z.panel_planform(v["sections"])
            n = 2 if v.get("mirror", True) else 1
            self.assertAlmostEqual(n * pp["area"], v["area"], delta=2e-4, msg=k)
            self.assertAlmostEqual(pp["span"], v["span"], delta=2e-4, msg=k)

    def test_mass_bookkeeping(self):
        M = self.S["mass"]
        empty = sum(i["mass_kg"] for i in M["items"])
        self.assertAlmostEqual(empty, M["empty_kg"], delta=0.01)
        self.assertAlmostEqual(M["mtow_kg"] - M["empty_kg"] - M["payload_kg"], M["fuel_kg"], delta=0.01)
        for i in M["items"]:
            self.assertTrue(i.get("basis"), i["name"])
            self.assertIn(i["group"], M["budget"], i["name"])
        groups = {}
        for i in M["items"]:
            groups[i["group"]] = groups.get(i["group"], 0.0) + i["mass_kg"]
        for g, b in M["budget"].items():
            self.assertLessEqual(abs(groups[g] - b["target_kg"]), b["tol_kg"], g)

    def test_landing_gear_geometry(self):
        LG = self.S["landing_gear"]
        mg, ng = LG["main"]["axle_static"], LG["nose"]["axle_static"]
        self.assertAlmostEqual(LG["wheelbase"], mg[0] - ng[0], places=4)
        self.assertAlmostEqual(LG["main"]["track"], 2 * mg[1], places=4)
        r = LG["tyre"]["diameter"] / 2 - LG["tyre"]["static_deflection"]
        self.assertAlmostEqual(LG["ground_z"], mg[2] - r, places=4)
        self.assertEqual(LG["type"], "retractable tricycle")
        # nose-up static attitude: the nose-wheel contact lies (x_mg - x_ng) tan(theta) BELOW the main contact
        th = math.radians(LG["rules"]["static_attitude_deg"])
        z_nose_contact = ng[2] - r
        self.assertAlmostEqual(z_nose_contact, LG["ground_z"] - (mg[0] - ng[0]) * math.tan(th), places=3)
        self.assertAlmostEqual(LG["checks"]["static_attitude_deg"], LG["rules"]["static_attitude_deg"], places=2)

    def test_turret_retractable(self):
        T = self.S["payload"]["turret"]
        self.assertGreater(T["ball_center_retracted_z"], T["ball_center_extended_z"])
        self.assertAlmostEqual(T["ball_center_retracted_z"] - T["ball_center_extended_z"], T["stroke"], places=4)

    def test_spec_writer_round_trip(self):
        """``--update-spec`` writes through write_spec: the spec must survive a write/read cycle unchanged and the
        file must carry the header and one banner per block (the committed spec is a write_spec output)."""
        import tempfile
        with tempfile.TemporaryDirectory() as td:
            p = Path(td) / "spec.yaml"
            Z.write_spec(self.S, p)
            text = p.read_text(encoding="utf-8")
            with open(p, encoding="utf-8") as fh:
                self.assertEqual(yaml.safe_load(fh), self.S)
        self.assertTrue(text.startswith(Z.SPEC_HEAD))
        for k in self.S:
            self.assertIn(f"\n# {k}: ", text, k)
        self.assertEqual(text, SPEC_FILE.read_text(encoding="utf-8"))


@unittest.skipUnless(HAVE, "ucav250 dependencies missing")
class TestOML(unittest.TestCase):
    """Fuselage, wing and tail OML meshes from the spec: closed, outward, no self-intersections."""

    @classmethod
    def setUpClass(cls):
        with open(SPEC_FILE, encoding="utf-8") as fh:
            cls.af = Z.Airframe(yaml.safe_load(fh))

    def test_meshes_closed_and_clean(self):
        meshes = [("fuselage", self.af.body_mesh), ("wing", self.af.wing_mesh)] + list(self.af.tail_meshes.items())
        for name, m in meshes:
            ck = m.check(self_intersect=True)
            self.assertTrue(ck["ok"], (name, ck))
            self.assertEqual(ck["self_intersections"], 0, name)

    def test_fuselage_dimensions(self):
        with open(SPEC_FILE, encoding="utf-8") as fh:
            S = yaml.safe_load(fh)
        lo, hi = self.af.body_mesh.bounds()
        self.assertAlmostEqual(hi[0] - lo[0], S["fuselage"]["length"], delta=2e-3)
        self.assertAlmostEqual(hi[1] - lo[1], S["fuselage"]["width_max"], delta=5e-3)


@unittest.skipUnless(HAVE, "ucav250 dependencies missing")
class TestCivilScope(unittest.TestCase):
    """No weapon-related terms anywhere in spec.yaml except the negative scope statement (meta.scope_tr)."""

    def test_scan(self):
        text = SPEC_FILE.read_text(encoding="utf-8")
        lines = text.splitlines()
        # drop the scope_tr scalar (it may wrap over several lines)
        keep, skip = [], False
        for ln in lines:
            if re.match(r"^\s+scope_tr:", ln):
                skip = True
                continue
            if skip and re.match(r"^\s{4,}\S", ln):
                continue
            skip = False
            keep.append(ln)
        body = "\n".join(keep).lower()
        for w in BANNED:
            self.assertNotIn(w, body, w)
        scope = yaml.safe_load(text)["meta"]["scope_tr"].lower()
        for w in ("weapon", "hardpoint", "pylon", "munition", "release"):
            self.assertIn(w, scope)


@unittest.skipUnless(HAVE, "ucav250 dependencies missing")
class TestReproduction(unittest.TestCase):
    """sizing.py reproduces the propulsion model of the reviewed endurance concept study exactly."""

    def test_propulsion_matches_endurance_study(self):
        """Same equations as the endurance study: compared on the study's propeller (Mejzlik 32x18 2B, sheet 0161; the
        HANCER primary is the 31x12 3B) and the study's electrical load (HANCER adds a research-payload allowance)."""
        path = REPO / "ucav250" / "data" / "concepts" / "endurance" / "calc.py"
        spec = importlib.util.spec_from_file_location("endurance_calc_ref", path)
        E = importlib.util.module_from_spec(spec)
        sys.modules["endurance_calc_ref"] = E
        spec.loader.exec_module(E)
        S = SPEC.load()
        key = "aero.yaml#propeller_tables_mejzlik.0161"
        D = S["propeller"]["diameter"] if S["propeller"]["table_ref"] == key else \
            S["propeller"]["alternatives"][key]["diameter"]
        eng = Z.Engine(S["engine"], E.P_ELEC, float(S["engine"]["generator"]["efficiency"]))
        prop = Z.Prop(Z.ref_get(key)["rows_rpm_thrust_N_torque_Nm_power_W"], float(D), float(S["propeller"]["k_wot"]),
                      float(S["propeller"]["k_inst"]), eng)
        for V, h in ((0.0, 0.0), (30.0, 0.0), (33.0, 3000.0)):
            a, b = E.PROP.wot(V, h), prop.wot(V, h)
            self.assertAlmostEqual(a["T"], b["T"], places=6)
            self.assertAlmostEqual(a["rpm"], b["rpm"], places=4)
        for T, V, h in ((100.0, 33.0, 3000.0), (120.0, 30.0, 0.0)):
            self.assertAlmostEqual(E.PROP.cruise(T, V, h)["P_shaft"], prop.cruise(T, V, h)["P_shaft"], places=4)
        for P in (3000.0, 4500.0, 9000.0):
            self.assertAlmostEqual(E.bsfc_g_kwh(P), eng.bsfc(P), places=6)
        self.assertAlmostEqual(E.P_GEN_SHAFT, eng.p_gen_shaft, places=6)

    def test_vlm_elliptic_reference(self):
        """Vortex lattice of an elliptic planform: CL_alpha ~ 2 pi A/(A + 2) and Trefftz span efficiency ~ 1."""
        b, c0, n = 8.0, 1.0, 40
        th = np.linspace(0, math.pi, n + 1)
        y = -0.5 * b * np.cos(th)
        c = c0 * np.sqrt(np.clip(1 - (2 * y / b) ** 2, 1e-4, None))
        strips = [(np.array([-0.25 * c[i], y[i], 0.0]), np.array([0.75 * c[i], y[i], 0.0]),
                   np.array([-0.25 * c[i + 1], y[i + 1], 0.0]), np.array([0.75 * c[i + 1], y[i + 1], 0.0]))
                  for i in range(n)]
        S = math.pi * b * c0 / 4
        sol = Z.vlm_solve([Z.vlm_panels(strips, 6)], S, 0.85 * c0, 0.0)
        AR = b * b / S
        self.assertAlmostEqual(sol["CL_alpha"], 2 * math.pi * AR / (AR + 2), delta=0.06 * sol["CL_alpha"])


if __name__ == "__main__":
    unittest.main()
