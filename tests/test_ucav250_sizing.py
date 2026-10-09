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
G = 9.80665
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


def tracked_outputs() -> list:
    """Repository files the sizing CLI could write: the spec, ucav250/out/*, ucav250/docs/fig/*."""
    root = REPO / "ucav250"
    return [SPEC_FILE] + sorted((root / "out").glob("*")) + sorted((root / "docs" / "fig").glob("*.png"))
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
        cls.tracked = {p: (p.stat().st_mtime_ns, p.read_bytes()) for p in tracked_outputs()}
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

    def test_tracked_files_untouched(self):
        """F18: the test run (and --check) does not modify tracked repository files (spec, out/, docs/fig)."""
        for p, (mt, data) in self.tracked.items():
            self.assertEqual(p.read_bytes(), data, str(p))
            self.assertEqual(p.stat().st_mtime_ns, mt, str(p))

    def test_check_is_read_only(self):
        """F18: --check alone writes nothing; the explicit output command (no flag) and --out write."""
        P = Z.build_parser()
        self.assertFalse(Z.outputs_written(P.parse_args(["--check"])))
        self.assertTrue(Z.outputs_written(P.parse_args(["--check", "--out", "/tmp/x"])))
        self.assertTrue(Z.outputs_written(P.parse_args([])))

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
                  "wing_root_chine_step_m", "flap_inboard_end_outboard_of_joint_m", "generator_margin_loiter",
                  "tail_root_max_gap_m", "stab_hinge_rated_margin_max", "prop_guard_ventral_margin_m",
                  "prop_plane_behind_cowl_over_D", "prop_clear_wingtip_on_ground_m", "generator_peak_margin_loiter",
                  "tail_root_interferences", "fcs_speed_limit_margin_m_s", "landing_ground_roll_mtow_m",
                  "mass_budget_margin_kg"):
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

    def test_propeller_guards_and_spindle(self):
        """F10: fins and the ventral fin cross the inclined disc plane outside the tip circle; F2/F3: the stabilator
        spindle sits at the forward end of the vortex-lattice AC band, ahead of the cylinders, inside the stub."""
        pc = self.out["propeller"]["clearances"]
        parts = {g["part"] for g in pc["guard_map"]["guards"]}
        self.assertTrue({"fin", "ventral"} <= parts)
        for g in pc["guard_map"]["guards"]:
            if g["part"] in ("fin", "ventral"):
                self.assertGreaterEqual(g["margin_over_tip_m"], 0.026, g)
        hm = self.out["stabilator_hinge"]
        self.assertGreaterEqual(hm["ac_offset_min_m"], -1e-9)                  # never unstable about the spindle
        self.assertTrue(hm["statically_stable_surface"])
        st = self.S["tail"]["surfaces"]["stabilator"]
        vlm = st["controls"]["ac_mac_fraction_vlm"]
        self.assertAlmostEqual(st["params"]["pivot_mac_fraction"],
                               min(vlm.values()) - st["controls"]["ac_band_fwd"], delta=2e-3)
        sc = self.out["packaging"]["stabilator_spindle"]
        self.assertTrue(sc["ok"], sc)

    def test_fix_round2_checks(self):
        """Fix round 2: converged closure, generator DC output with the power-electronics efficiency, per-configuration
        peak-load check (HD59 design mission, E180 growth mission), operating limits, MTOM landing, budget ceiling."""
        hist = self.out["derived_check"]["closure_history"]
        self.assertTrue(hist[-1].get("converged"), hist[-3:])
        gen = self.S["engine"]["generator"]
        eta_pe = gen["power_electronics_efficiency"]
        self.assertGreater(eta_pe, 0.5)
        self.assertLess(eta_pe, 1.0)
        for k in ("start", "end"):
            p = self.out["performance"]["mission_loiter_points"][k]
            self.assertAlmostEqual(p["gen_W"], gen["power_continuous_W"] * p["rpm"] / gen["rated_rpm"] * eta_pe,
                                   delta=1e-6 * p["gen_W"])
        el = self.out["electrical"]
        # fix round 3 (V2-01): R-52 is the v1.2 capability again (E180 peak available throughout the design-mission
        # loiter), supplied by the generator plus the battery peak-support share; the v1.3 "installed turret only" rule
        # is withdrawn
        self.assertEqual(self.S["mission"]["loiter_rpm_floor"], "e180_peak_battery_support")
        draws = [el["e180_peak_battery_draw_design_mission_Wh"], el["e180_peak_battery_draw_e180_mission_Wh"]]
        margin = el["battery_peak_share_Wh"] / max(max(draws), 1e-3)          # outputs carry 6 significant digits
        self.assertAlmostEqual(self.out["metrics"]["e180_peak_support_margin"], margin, delta=1e-5 * margin)
        self.assertGreaterEqual(self.out["metrics"]["e180_peak_support_margin"], 1.0)
        self.assertAlmostEqual(el["battery_peak_share_Wh"], el["battery_usable_Wh"] - el["battery_reserve_Wh"],
                               delta=1e-5 * el["battery_usable_Wh"])
        EB = self.S["engine"]["electrical_budget"]
        reserve = EB["battery_reserve_power_W"] * EB["battery_reserve_time_min"] / 60.0
        self.assertAlmostEqual(el["battery_reserve_Wh"], reserve, delta=1e-5 * reserve)       # 6 significant digits
        cap = float(self.S["mission"]["peak_support_deficit_cap_W"])
        self.assertLessEqual(el["e180_peak_deficit_max_design_mission_W"], cap + 0.05)
        self.assertLessEqual(el["e180_peak_deficit_max_e180_mission_W"], cap + 0.05)
        self.assertAlmostEqual(self.out["metrics"]["generator_peak_margin_loiter"],
                               el["margin_peak_e180_at_design_mission_rpm"], places=9)
        # the battery energy the E180 peak would draw = integral of (peak - generator output) over the loiter steps
        lo = [r for r in self.out["performance"]["mission_log"] if r["kind"] == "loiter"]
        draw = sum(max(el["peak_e180_W"] - r["gen_W"], 0.0) * r["dt_s"] / 3600.0 for r in lo)
        self.assertAlmostEqual(draw, el["e180_peak_battery_draw_design_mission_Wh"], delta=1e-5 * draw)  # 6 digits
        e180 = self.out["performance"]["e180_growth_mission"]
        self.assertGreaterEqual(e180["loiter_points"]["end"]["rpm"], el["generator_rpm_floor_e180_mission"] - 1e-6)
        for k in ("start", "end"):
            self.assertGreaterEqual(self.out["performance"]["mission_loiter_points"][k]["rpm"],
                                    el["generator_rpm_floor"] - 1e-6)
        ol = self.out["loads"]["operating_limits"]
        self.assertAlmostEqual(ol["VNE_eas"], 0.9 * self.out["loads"]["VD_m_s"], places=9)
        self.assertLessEqual(ol["VNO_eas"], min(self.out["loads"]["VC_m_s"], 0.89 * ol["VNE_eas"]) + 1e-9)
        self.assertLessEqual(ol["fcs_speed_limit_eas"], ol["VNO_eas"] + 1e-9)
        self.assertIn("landing_sl_mtow_takeoff_flap", self.out["performance"])
        bc = self.out["mass"]["budget_check"]
        self.assertEqual(bc["groups_over_ceiling"], [])
        # the fuel for the 10 h mission is solved on the air time itself (return transit at the post-loiter weight)
        req_h = float(self.S["mission"]["endurance_requirement_h"])
        self.assertLessEqual(bc["air_time_h_at_fuel_for_10h"], req_h + 1e-9)
        self.assertGreater(bc["air_time_h_at_fuel_for_10h"], req_h - 1e-3)
        hm = self.out["stabilator_hinge"]
        self.assertNotAlmostEqual(hm["CN_max_panel"], hm["trim_rule_clmax_not_used"], places=3)
        self.assertGreater(hm["CN_max_panel"], 1.0)
        self.assertGreaterEqual(hm["surface_travel_deg"], 20.0)
        self.assertEqual(self.out["tail_root_interference"]["n_conflicts"], 0)
        # R-56 (last: it fails while R-02 is not met, see doc 02 sec. 14)
        self.assertLessEqual(bc["budget_sum_kg"], bc["empty_kg_at_R02_limit"] - bc["reserve_kg"] + 1e-9)

    def test_fix_round3_checks(self):
        """Fix round 3: integrated descent at the generator floor (V2-07), consistent lift-off (V2-05), control-surface
        hinge moments through four-bar linkages (V2-03/V2-06), installation factor at full throttle (V2-04), polars
        clipped at the trimmed CLmax (V2-08)."""
        S, out = self.S, self.out
        log = out["performance"]["mission_log"]
        de = [r for r in log if r["kind"] == "descent"]
        self.assertGreater(len(de), 1)
        self.assertAlmostEqual(sum(r["h_from_m"] - r["h_to_m"] for r in de), S["mission"]["loiter_altitude"], places=6)
        eb = S["engine"]["electrical_budget"]
        load = eb["continuous_base_W"] + eb["research_payload_allowance_W"]
        for r in de:
            self.assertAlmostEqual(r["gen_W"], load, delta=1e-6 * load)        # the generator carries the load
            self.assertAlmostEqual(r["dt_s"], (r["h_from_m"] - r["h_to_m"]) / r["sink_m_s"], delta=1e-6 * r["dt_s"])
            self.assertAlmostEqual(r["fraction"], 1.0 - r["fuel_kg"] * G / r["W_start_N"], delta=1e-6)  # 6 digits
            if r.get("speed_band_end") is None:
                self.assertAlmostEqual(r["sink_m_s"], S["mission"]["descent_rate"], places=3)
        to = out["performance"]["takeoff_sl_mtow"]
        self.assertGreaterEqual(to["V_lof_m_s"], to["V_R_m_s"])
        self.assertGreaterEqual(to["V_lof_m_s"], 1.1 * to["VS_TO_m_s"] - 1e-6)
        self.assertGreaterEqual(to["theta_lof_deg"], to["theta_ground_deg"] - 1e-9)   # no "lift-off below the ground attitude"
        self.assertGreaterEqual(to["CL_available_lof"], to["CL_wb_lof"] - 1e-3)      # L >= W + T sin(eps) + F_t at lift-off
        self.assertLessEqual(to["theta_lof_deg"], to["theta_1p1VS_trimmed_deg"] + 1e-6)  # R-16 attitude is the bound
        sch = to["fcs_stabilator_schedule"]
        self.assertLessEqual(sch["download_start_m_s"], to["V_R_m_s"])
        self.assertGreater(to["main_gear_load_at_VR_N"], 0.0)
        ch = out["control_hinges"]
        for k in ("aileron", "flap", "rudder", "stabilator"):
            self.assertTrue(ch[k]["reachable"], k)
            self.assertGreaterEqual(ch[k]["travel_margin_deg"], 0.0, k)
            self.assertGreaterEqual(ch[k]["rated_margin_min"], 1.1, k)
            self.assertGreaterEqual(ch[k]["peak_margin_min"], 1.0, k)
            self.assertGreaterEqual(ch[k]["ratio_min"], ch[k]["arm_ratio_neutral"] * 0.99, k)
        self.assertEqual(out["metrics"]["control_actuator_rated_margin_min"],
                         min(ch[k]["rated_margin_min"] for k in ("aileron", "flap", "rudder")))
        self.assertGreaterEqual(out["metrics"]["stab_hinge_rated_margin_max"], 1.1)
        for p in out["aero"]["polars"].values():
            self.assertLessEqual(p["fit"]["CL_endurance"], p["fit"]["CL_max_trimmed"] + 1e-9)
            self.assertLessEqual(p["fit"]["CL_LDmax"], p["fit"]["CL_max_trimmed"] + 1e-9)

    def test_turret_field_of_regard_includes_wing(self):
        """F13: the ray test obstacles include the wing (both sides) as well as the tail surfaces."""
        with open(SPEC_FILE, encoding="utf-8") as fh:
            af = Z.Airframe(yaml.safe_load(fh))
        n_all = len(Z.structure_triangles(af))
        n_tail = len(Z.structure_triangles(af, include_wing=False))
        self.assertGreater(n_all, n_tail)

    def test_report_written_in_turkish(self):
        md = (self.out_dir / "sizing.md").read_text(encoding="utf-8")
        for w in ("Gereksinim uyumu", "Kütle", "Kararlılık", "Performans", "İniş takımı"):
            self.assertIn(w, md)


@unittest.skipUnless(HAVE, "ucav250 dependencies missing")
class TestLinkagesAndInstallation(unittest.TestCase):
    """V2-06: four-bar kinematics (a symmetric N:1 crank-rocker cannot pass asin(1/N); the torque ratio grows toward
    the ends); V2-04: the pusher installation factor at full throttle ramps from 1.0 static to k_inst."""

    def test_fourbar_limits_and_symmetry(self):
        fb = Z.FourBar(0.015, 3.0, 2.0)                      # long pushrod: close to the r_s sin(t) = r_h sin(p) limit
        for th in (0.3, 0.8, 1.2):
            self.assertAlmostEqual(fb.phi(th), math.asin(math.sin(th) / 3.0), delta=2e-3)
            # symmetric to second order: with a finite pushrod l = d the push and pull sides differ by about
            # Y^2 / (d r_h cos phi), Y = r_s (cos th - 1) - r_h (cos phi - 1) (lateral offset of the rod ends);
            # 10 % allowance for the higher-order terms
            ph = fb.phi(th)
            Y = fb.r_s * (math.cos(th) - 1.0) - fb.r_h * (math.cos(ph) - 1.0)
            self.assertLessEqual(abs(fb.phi(-th) + ph), 1.1 * Y * Y / (fb.d * fb.r_h * math.cos(ph)))
        # v1.3 "3:1 bellcrank, +-28.3 deg": not achievable with a 3:1 four-bar; -20 deg is out of reach in the
        # long-pushrod limit (asin(1/3) = 19.47 deg) and, with the 100 mm pushrod, reached only at the toggle
        fb3 = Z.FourBar(0.015, 3.0, 0.1)
        for d in (-28.3, 28.3):
            self.assertTrue(math.isnan(fb3.theta(math.radians(d))))
        self.assertTrue(math.isnan(fb.theta(math.radians(-20.0))))
        t3 = fb3.theta(math.radians(-20.0))
        self.assertGreater(abs(math.degrees(t3)), 80.0)
        self.assertGreater(fb3.ratio(t3), 10.0)
        fb2 = Z.FourBar(0.015, 2.5, 0.1)
        t20 = fb2.theta(math.radians(-20.0))
        self.assertTrue(math.isfinite(t20))
        self.assertGreater(fb2.ratio(t20), fb2.ratio(0.0))
        self.assertAlmostEqual(fb2.ratio(0.0), 2.5, delta=0.01)
        self.assertAlmostEqual(math.degrees(fb2.phi(t20)), -20.0, places=6)

    def test_wot_installation_ramp(self):
        S = SPEC.load()
        eng, prop = Z.make_propulsion(S)
        ki = float(S["propeller"]["k_inst"])
        self.assertEqual(prop.k_wot_installed(0.0), 1.0)
        self.assertAlmostEqual(prop.k_wot_installed(float(S["propeller"]["k_inst_wot_ramp_speed"])), ki, places=12)
        self.assertAlmostEqual(prop.k_wot_installed(40.0), ki, places=12)
        p0 = Z.Prop(Z.ref_get(S["propeller"]["table_ref"])["rows_rpm_thrust_N_torque_Nm_power_W"],
                    float(S["propeller"]["diameter"]), float(S["propeller"]["k_wot"]), ki, eng)
        self.assertAlmostEqual(prop.wot(0.0, 0.0)["T"], p0.wot(0.0, 0.0)["T"], places=9)
        self.assertAlmostEqual(prop.wot(30.0, 0.0)["T"], ki * p0.wot(30.0, 0.0)["T"], places=9)


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
            self.assertLessEqual(groups[g], b["target_kg"] + 1e-6, g)                # V1-03: one-sided ceiling

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
class TestGeometryFixes(unittest.TestCase):
    """Fix round 2 geometry: LERX/glove morph without airfoil-family switches (V1-06), tail roots trimmed at the OML
    with an interference check that would catch the buried v1.2 fin root (V1-04)."""

    @classmethod
    def setUpClass(cls):
        with open(SPEC_FILE, encoding="utf-8") as fh:
            cls.S = yaml.safe_load(fh)
        cls.af = Z.Airframe(cls.S)

    def test_glove_airfoil_files_reproducible(self):
        self.assertEqual(Z.write_glove_airfoils(self.S, check_only=True), [])

    def test_glove_morph_no_family_switch(self):
        P = self.S["wing"]["planform"]
        L = Z.lerx_section(P)
        names = [Z.glove_airfoil_name(w, L) for w in Z.GLOVE_BLEND_STEPS] + ["nlf416"]
        secs = [s_ for s_ in self.S["wing"]["sections"] if s_["y"] <= P["y_junction"] + 1e-9]
        idx = [names.index(s_["airfoil"]) for s_ in secs]
        self.assertEqual(idx, sorted(idx))
        self.assertTrue(all(b - a <= 1 for a, b in zip(idx, idx[1:])), idx)      # one blend step at a time
        self.assertEqual(idx[0], 0)
        self.assertEqual(idx[-1], len(names) - 1)

    def test_naca4_modified_reference(self):
        """NACA 0012-63 (four-digit modified, I = 6, m = 0.3) is close to the standard NACA 0012."""
        A = Z.naca4_modified(0.12, 6.0, 0.3)
        B = oml._naca4("0012", 161)
        np.testing.assert_allclose(A[:, 1], B[:, 1], atol=2.5e-3)
        self.assertAlmostEqual(float(np.max(A[:, 1]) * 2), 0.12, places=4)

    def test_tail_root_interference(self):
        r = Z.tail_root_interference(self.S, self.af)
        self.assertEqual(r["n_conflicts"], 0, r["conflicts"])
        S2 = json.loads(json.dumps(self.S))
        S2["tail"]["root_structure"]["fitting_band_depth"] = 0.40          # untrimmed planar roots: must conflict
        S2["tail"]["root_structure"]["band_depth_aft_of_firewall"] = 0.40
        self.assertGreater(Z.tail_root_interference(S2, self.af)["n_conflicts"], 0)


@unittest.skipUnless(HAVE, "ucav250 dependencies missing")
class TestMissionIntegration(unittest.TestCase):
    """V1-01: the mission fuel of every flown segment equals the time integral of the point fuel flow (bsfc x shaft
    power incl. the generator at the trimmed point, T = D / cos(eps)), checked independently with fine steps."""

    @classmethod
    def setUpClass(cls):
        with open(SPEC_FILE, encoding="utf-8") as fh:
            cls.S = yaml.safe_load(fh)
        cls.R = Z.evaluate(cls.S, sens=False, light=True)
        cls.fl = cls.R["_flight"]
        cls.log = cls.R["performance"]["mission_log"]

    def _fine(self, kind, W0, length, n=24, h=None):
        fl = self.fl
        h = fl.h_loiter if h is None else h
        W, fuel, t = W0, 0.0, 0.0
        for _ in range(n):
            if kind == "loiter":
                pt = lambda W_: fl.best_loiter(W_, h, "loiter", rpm_floor=True)        # noqa: E731
                d = length / n
                p0 = pt(W)
                pm = pt(W - 0.5 * p0["ff_kg_s"] * d * G)
                f_ = pm["ff_kg_s"] * d
            else:
                pt = lambda W_: fl.best_range(W_, h)                                    # noqa: E731
                d = length / n
                p0 = pt(W)
                pm = pt(W - 0.5 * p0["ff_kg_s"] * d / p0["V"] * G)
                f_ = pm["ff_kg_s"] * d / pm["V"]
            fuel += f_
            W -= f_ * G
        return fuel

    def test_loiter_and_cruise_steps_match_fine_integration(self):
        lo = next(r for r in self.log if r["kind"] == "loiter")
        self.assertAlmostEqual(self._fine("loiter", lo["W_start_N"], lo["dt_s"]), lo["fuel_kg"],
                               delta=0.002 * lo["fuel_kg"])
        cr = next(r for r in self.log if r["kind"] == "cruise")
        dist = Z.Flight.STEP_CRUISE_M
        self.assertAlmostEqual(self._fine("cruise", cr["W_start_N"], dist), cr["fuel_kg"], delta=0.002 * cr["fuel_kg"])

    def test_climb_matches_fine_integration(self):
        fl = self.fl
        rows = [r for r in self.log if r["kind"] == "climb"]
        W, fuel, Vp = rows[0]["W_start_N"], 0.0, None
        h_top = rows[-1]["h_to_m"]
        n = 30
        dh = h_top / n
        for i in range(n):
            p = fl.climb_point(W, (i + 0.5) * dh)
            dhe = dh + (0.0 if Vp is None else (p["V"] ** 2 - Vp ** 2) / (2 * G))
            f_ = p["ff_kg_h"] / 3600.0 * dhe / p["roc"]
            fuel += f_
            W -= f_ * G
            Vp = p["V"]
        self.assertAlmostEqual(fuel, sum(r["fuel_kg"] for r in rows), delta=0.005 * fuel)

    def test_step_fuel_consistent_with_point_fuel_flows(self):
        """Every integrated cruise/loiter/reserve step: fuel = dt x (ff_start + 4 ff_mid + ff_end)/6 within 0.2 %, and
        the point fuel flow is bsfc x total shaft power (the v1.2 Breguet L/D error was ~1 %)."""
        for r in self.log:
            if r["kind"] not in ("cruise", "loiter", "reserve") or r.get("partial_step"):
                continue
            simpson = r["dt_s"] / 3600.0 * (r["ff_start_kg_h"] + 4 * r["ff_mid_kg_h"] + r["ff_end_kg_h"]) / 6.0
            self.assertAlmostEqual(r["fuel_kg"], simpson, delta=0.002 * r["fuel_kg"], msg=r["name"])
        p = self.fl.best_loiter(self.log[0]["W_start_N"] * 0.97, self.fl.h_loiter, "loiter")
        self.assertAlmostEqual(p["ff_kg_s"], p["bsfc_g_kWh"] * p["P_total"] / 3.6e9, places=12)
        lp = self.fl.level_point(1400.0, 33.0, self.fl.h_loiter, "loiter")
        e = self.fl.eps
        self.assertAlmostEqual(lp["CL"] * 0.5 * Z.AL.isa(self.fl.h_loiter)["rho"] * 33.0 ** 2 * self.fl.Sw,
                               1400.0 + lp["D"] * math.tan(e), delta=1e-4 * 1400.0)   # 2-pass fixed point

    def test_mission_fuel_bookkeeping(self):
        mis = self.R["performance"]
        frac = 1.0
        for r in self.log:
            frac *= r["fraction"]
        m0 = self.S["mass"]["mtow_kg"]
        trapped = self.S["mission"]["trapped_fuel_fraction"]
        self.assertAlmostEqual((1 - frac) * (1 + trapped), mis["mission_fuel_fraction"], places=9)
        self.assertLessEqual(mis["mission_fuel_fraction"] * m0, self.S["mass"]["fuel_kg"] + 1e-6)


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
class TestWording(unittest.TestCase):
    """F19: no 'hard point' wording in the sizing code, the sizing report, the research hand-over or the reference
    concepts (negative scope statements such as 'no hardpoints' excepted)."""

    def test_no_hard_point_items(self):
        root = REPO / "ucav250"
        files = [root / "analysis" / "sizing.py", root / "docs" / "02_konsept_ve_boyutlandirma.md"] + \
            sorted((root / "data" / "research").glob("*.yaml")) + sorted((root / "data" / "concepts").rglob("*.py")) + \
            sorted((root / "data" / "concepts").rglob("*.yaml"))
        for p in files:
            text = p.read_text(encoding="utf-8").lower()
            for w in ("hard_point", "hard point", "hard-point"):
                self.assertNotIn(w, text, f"{p}: {w}")


@unittest.skipUnless(HAVE, "ucav250 dependencies missing")
class TestDoc02(unittest.TestCase):
    """F14: the key numbers of doc 02 are those of the spec (no stale figures after a spec update)."""

    def test_key_numbers_match_spec(self):
        with open(SPEC_FILE, encoding="utf-8") as fh:
            S = yaml.safe_load(fh)
        doc = (REPO / "ucav250" / "docs" / "02_konsept_ve_boyutlandirma.md").read_text(encoding="utf-8")

        def tr(x, n):
            return f"{x:,.{n}f}".replace(",", "X").replace(".", ",").replace("X", ".")
        self.assertIn(f"rev. {S['meta']['revision']}", doc.splitlines()[0])
        n_req = len(S["requirements"])
        self.assertIn(f"{n_req} gereksinimin", doc)
        self.assertIn(f"**{tr(S['performance']['reference']['endurance_h'], 2)} h**", doc)
        self.assertIn(f"{tr(S['mass']['empty_kg'], 1)} kg", doc)
        self.assertIn(f"**{tr(S['mass']['mtow_kg'], 1)} kg**", doc)
        self.assertIn(f"{tr(S['wing']['area'], 3)} m²", doc)
        self.assertIn(f"{tr(S['tail']['surfaces']['stabilator']['area'], 3)} m²", doc)
        # V1-08: the design mission's own loiter start/end points are reported; the MTOM point at 3000 m carries its
        # own label (it is not the mission loiter start)
        ref = S["performance"]["reference"]
        self.assertIn(f"{tr(ref['mission_loiter_start_tas_m_s'], 2)} m/s TAS", doc)
        self.assertIn(f"{tr(ref['mission_loiter_start_eas_m_s'], 2)} m/s EAS", doc)
        self.assertIn(f"{tr(ref['mission_loiter_start_rpm'], 0)} rpm", doc)
        self.assertIn(f"{tr(ref['mission_loiter_start_generator_W'], 0)} W", doc)
        self.assertIn(f"{tr(ref['mission_loiter_end_rpm'], 0)} rpm", doc)
        self.assertIn(f"{tr(ref['mission_loiter_end_generator_W'], 0)} W", doc)
        self.assertIn(f"{tr(ref['loiter_rpm'], 0)} rpm", doc)
        self.assertIn("MTOM'da 3000 m bekleme noktası", doc)
        self.assertNotIn("Bekleme 3000 m (başlangıç)", doc)


@unittest.skipUnless(HAVE, "ucav250 dependencies missing")
class TestAirfoils(unittest.TestCase):
    """F17: the ventral fin uses the analytic NACA 0010 (the coarse UIUC naca0010.dat is not read)."""

    def test_ventral_naca0010_analytic(self):
        with open(SPEC_FILE, encoding="utf-8") as fh:
            S = yaml.safe_load(fh)
        name = S["tail"]["surfaces"]["ventral"]["params"]["airfoil"]
        self.assertEqual(name, "NACA-0010")
        np.testing.assert_allclose(oml.airfoil_coords(name), oml._naca4("0010"))


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
        # same equations with the study's own factors (HANCER uses k_inst 0.93 for its bluff cowl base and the
        # full-throttle installation ramp, fix round 3; both are checked in TestLinkagesAndInstallation)
        self.assertAlmostEqual(float(S["propeller"]["k_wot"]), E.K_WOT, places=12)
        prop = Z.Prop(Z.ref_get(key)["rows_rpm_thrust_N_torque_Nm_power_W"], float(D), E.K_WOT, E.K_INST, eng)
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
