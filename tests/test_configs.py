"""Konfigürasyon tutarlılık testleri.

Çalıştırma (ek bağımlılık gerekmez, yalnızca PyYAML):
    python3 -m unittest discover -s tests -v
"""
from __future__ import annotations

import math
import os
import re
import sys
import tempfile
import unittest
from pathlib import Path

import yaml

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "tools"))

import budget_calc  # noqa: E402
import validate_params  # noqa: E402

CONFIG = ROOT / "config"


def load_yaml(path: Path) -> dict:
    with open(path, encoding="utf-8") as fh:
        return yaml.safe_load(fh)


def px4_params(path: Path) -> dict[str, float]:
    return {name: float(value) for _, name, value, _ in validate_params.parse_px4(path)}


def merged(paths) -> dict[str, float]:
    out: dict[str, float] = {}
    for path in paths:
        for name, value in px4_params(path).items():
            if name in out and out[name] != value:
                raise AssertionError(f"{name} çelişkili tanım ({path.name})")
            out[name] = value
    return out


PX4_BASE = sorted((CONFIG / "px4" / "base").glob("*.params"))
PX4_PROFILES = sorted((CONFIG / "px4" / "profiles").glob("*.params"))
AP_FILES = sorted((CONFIG / "ardupilot").glob("*.param"))


def ap_params(path: Path) -> dict[str, float]:
    return {name: float(value) for _, name, value in validate_params.parse_ardupilot(path)}


class TestBudgetMath(unittest.TestCase):
    CURVE = [(100.0, 10.0), (500.0, 80.0), (1000.0, 250.0)]
    K1 = math.log(80.0 / 10.0) / math.log(500.0 / 100.0)  # ilk segmentin güç yasası üssü

    def test_power_law_interpolation(self):
        self.assertAlmostEqual(budget_calc.power_at_thrust(self.CURVE, 500.0), 80.0)
        self.assertAlmostEqual(budget_calc.power_at_thrust(self.CURVE, 300.0),
                               10.0 * 3.0 ** self.K1)
        # ilk noktanın altı: momentum teorisi P ∝ T^1.5
        self.assertAlmostEqual(budget_calc.power_at_thrust(self.CURVE, 50.0), 10.0 * 0.5 ** 1.5)

    def test_inverse_is_consistent(self):
        for t in (40.0, 100.0, 300.0, 750.0, 1000.0):
            p = budget_calc.power_at_thrust(self.CURVE, t)
            self.assertAlmostEqual(budget_calc.thrust_at_power(self.CURVE, p), t)
        self.assertEqual(budget_calc.thrust_at_power(self.CURVE, 999.0), 1000.0)

    def test_out_of_curve_raises(self):
        with self.assertRaises(ValueError):
            budget_calc.power_at_thrust(self.CURVE, 1200.0)

    def test_thr_mdl_fac_fit_recovers_model(self):
        for fac in (0.0, 0.35, 0.7, 1.0):
            pts = [[u, 1500 * (fac * u * u + (1 - fac) * u)] for u in (0.2, 0.4, 0.6, 0.8, 1.0)]
            self.assertAlmostEqual(budget_calc.fit_thr_mdl_fac(pts), fac, places=6)
        with self.assertRaises(ValueError):
            budget_calc.fit_thr_mdl_fac([[0.5, 700.0]])  # %100 noktası yok

    def _profile(self, installation_factor=1.0):
        return {
            "profile": {"id": "t", "name": "test"},
            "limits": {"max_auw_g": 1000, "min_tw_ratio": 2.0, "min_hover_time_min": 1},
            "components": [{"name": "gövde", "category": "gövde", "mass_g": 200, "power_w": 10}],
            "propulsion": {
                "motor_count": 4,
                "installation_factor": installation_factor,
                "motor": {"mass_g": 50}, "prop": {"mass_g": 5}, "esc": {"mass_g": 20},
                "thrust_curve": {"points": [list(p) for p in self.CURVE]},
            },
            "battery": {"cells_series": 6, "nominal_cell_v": 3.7, "capacity_mah": 1000,
                        "usable_fraction": 0.8, "mass_g": 200, "max_continuous_a": 100},
        }

    def test_synthetic_profile(self):
        b = budget_calc.compute(self._profile())
        self.assertAlmostEqual(b.auw_g, 200 + 4 * 55 + 20 + 200)          # 640 g
        self.assertAlmostEqual(b.hover_thrust_per_motor_g, 160.0)
        hover_w = 4 * 10.0 * 1.6 ** self.K1 + 10.0
        self.assertAlmostEqual(b.total_hover_w, hover_w)
        self.assertAlmostEqual(b.hover_time_min, 6 * 3.7 * 0.8 / hover_w * 60)
        self.assertAlmostEqual(b.hover_throttle, 640 / 4000)
        self.assertTrue(b.ok)

    def test_installation_factor_costs_power_and_thrust(self):
        ideal = budget_calc.compute(self._profile(1.0))
        guarded = budget_calc.compute(self._profile(0.85))
        self.assertGreater(guarded.total_hover_w, ideal.total_hover_w)
        self.assertAlmostEqual(guarded.max_thrust_g, ideal.max_thrust_g * 0.85)
        self.assertAlmostEqual(guarded.hover_throttle, ideal.hover_throttle / 0.85)
        with self.assertRaises(ValueError):
            budget_calc.compute(self._profile(0.3))


class TestHardwareProfiles(unittest.TestCase):
    REQUIRED = ("profile", "limits", "components", "propulsion", "battery")

    def profiles(self):
        paths = budget_calc.all_profiles()
        self.assertGreaterEqual(len(paths), 3, "en az 3 donanım seviyesi bekleniyor")
        return paths

    def test_profiles_meet_their_limits(self):
        for path in self.profiles():
            with self.subTest(profile=path.name):
                data = load_yaml(path)
                for key in self.REQUIRED:
                    self.assertIn(key, data)
                b = budget_calc.compute(data)
                failed = [f"{c.name}: {c.detail}" for c in b.checks if not c.ok]
                self.assertFalse(failed, f"{path.name} limitleri sağlamıyor: {failed}")

    def test_component_fields(self):
        for path in self.profiles():
            for comp in load_yaml(path)["components"]:
                with self.subTest(profile=path.name, component=comp.get("name")):
                    self.assertGreater(comp["mass_g"], 0)
                    self.assertGreaterEqual(comp.get("power_w", 0), 0)
                    self.assertIn("category", comp)


class TestMissionConfig(unittest.TestCase):
    def setUp(self):
        self.behavior = load_yaml(CONFIG / "mission" / "behavior.yaml")
        self.safety = load_yaml(CONFIG / "mission" / "safety.yaml")

    def test_palm_landing_invariants(self):
        p = self.behavior["palm_landing"]
        d = p["descend"]
        self.assertTrue(p["require_guards"], "avuca iniş pervane koruması olmadan açılamaz")
        self.assertFalse(p["ready"]["approach_person"], "drone kişiye kendisi yaklaşmamalı")
        self.assertLessEqual(d["speed_far_mps"], 0.30)
        self.assertLess(d["speed_near_mps"], d["speed_far_mps"])
        self.assertLessEqual(d["palm_lost_timeout_s"], 0.5)
        self.assertLess(p["contact"]["tof_contact_m"], p["detection"]["tof_valid_range_m"][0])
        self.assertLess(d["abort_lateral_error_m"], p["align"]["abort_lateral_error_m"])
        self.assertLessEqual(p["contact"]["confirm_s"], 0.15)  # F-14: ≤ 150 ms
        self.assertGreaterEqual(p["contact"]["min_extra_cues"], 1)
        self.assertIn(p["target"], ("marker_pad", "bare_palm"))
        self.assertTrue(p["require_operator_confirm"])
        self.assertGreaterEqual(p["detection"]["hand_still_s"], 1.0)
        self.assertLessEqual(p["geofence_radius_m"], 15.0)

    def test_follow_respects_people_distance(self):
        f = self.behavior["follow"]
        self.assertGreaterEqual(f["min_distance_m"],
                                self.safety["people"]["min_horizontal_distance_m"])
        self.assertLessEqual(f["max_speed_mps"],
                             self.safety["setpoint_limits"]["max_horizontal_speed_mps"])
        self.assertIn(f["default_mode"], f["allowed_modes"])

    def test_gesture_confirmation(self):
        g = self.behavior["gestures"]
        self.assertLessEqual(g["confirm_min_frames"], g["confirm_window_frames"])
        self.assertGreaterEqual(g["hold_time_s"], 1.0)  # F-11
        self.assertEqual(g["bindings"]["fist"], "stop_hover")


class TestFlightStackParams(unittest.TestCase):
    def setUp(self):
        self.base = merged(PX4_BASE)
        self.safety = load_yaml(CONFIG / "mission" / "safety.yaml")
        self.tier_b = budget_calc.compute(load_yaml(CONFIG / "hardware" / "tier-b-pro.yaml"))

    def test_params_match_official_reference(self):
        """Ad, tip, aralık ve enum: PX4 v1.17 ve ArduPilot Copter 4.7.1 referansları."""
        errors = []
        for path in PX4_BASE + PX4_PROFILES:
            errors += validate_params.validate_px4(path)
        for path in AP_FILES:
            errors += validate_params.validate_ardupilot(path)
        self.assertFalse(errors, "\n".join(errors))

    def test_px4_structure(self):
        self.assertGreaterEqual(len(PX4_BASE), 4)
        self.assertGreater(len(self.base), 60)
        profiles = {p.name: set(px4_params(p)) for p in PX4_PROFILES}
        self.assertGreaterEqual(len(profiles), 2)
        keysets = list(profiles.values())
        for name, keys in profiles.items():
            with self.subTest(profile=name):
                self.assertFalse(keys & set(self.base), "profil, base parametresini ezmemeli")
                self.assertEqual(keys, keysets[0], "profiller aynı parametre kümesini tanımlamalı")

    def test_px4_matches_safety_config(self):
        p, s = self.base, self.safety
        self.assertEqual(p["GF_MAX_HOR_DIST"], s["geofence"]["max_horizontal_distance_m"])
        self.assertEqual(p["GF_MAX_VER_DIST"], s["geofence"]["max_altitude_m"])
        self.assertAlmostEqual(p["BAT_LOW_THR"], s["battery"]["low_fraction"])
        self.assertAlmostEqual(p["BAT_CRIT_THR"], s["battery"]["critical_fraction"])
        self.assertAlmostEqual(p["BAT_EMERGEN_THR"], s["battery"]["emergency_fraction"])
        lim = s["setpoint_limits"]
        self.assertEqual(p["MPC_XY_VEL_MAX"], lim["max_horizontal_speed_mps"])
        self.assertEqual(p["MPC_Z_VEL_MAX_UP"], lim["max_up_speed_mps"])
        self.assertEqual(p["MPC_Z_VEL_MAX_DN"], lim["max_down_speed_mps"])

    def test_px4_safety_invariants(self):
        p = self.base
        self.assertEqual(p["COM_RCL_EXCEPT"], 0, "harici modlar RC kaybından muaf olmamalı")
        self.assertLess(p["LNDMC_Z_VEL_MAX"], p["MPC_LAND_CRWL"])
        self.assertEqual(p["MNT_MODE_IN"], -1, "gimbal Jetson'dan sürülür")
        self.assertNotEqual(p["UXRCE_DDS_CFG"], p["MAV_0_CONFIG"], "DDS ve MAVLink ayrı portlarda")

    def test_hover_thrust_matches_budget(self):
        """Başlangıç hover itkisi ve itki modeli, önerilen seviyenin bütçe hesabıyla uyumlu."""
        self.assertAlmostEqual(self.base["MPC_THR_HOVER"], self.tier_b.hover_throttle, delta=0.05)
        self.assertAlmostEqual(self.base["THR_MDL_FAC"], self.tier_b.thr_mdl_fac, delta=0.10)
        ap = ap_params(AP_FILES[0])
        self.assertAlmostEqual(ap["MOT_THST_HOVER"], self.tier_b.hover_throttle, delta=0.05)

    def test_ardupilot_matches_safety_config(self):
        ap = ap_params(AP_FILES[0])
        self.assertEqual(ap["FENCE_RADIUS"], self.safety["geofence"]["max_horizontal_distance_m"])
        self.assertEqual(ap["FENCE_ALT_MAX"], self.safety["geofence"]["max_altitude_m"])
        self.assertEqual(ap["EK3_RNG_USE_HGT"], -1, "lidar yükseklik kaynağı olmamalı (avuca iniş)")


class TestValidatorCatchesErrors(unittest.TestCase):
    """Doğrulayıcı boş geçmemeli: bilinen hataları yakalamalı."""

    def _write(self, text: str, suffix: str) -> Path:
        fd, name = tempfile.mkstemp(suffix=suffix)
        os.close(fd)
        path = Path(name)
        path.write_text(text, encoding="utf-8")
        self.addCleanup(path.unlink)
        return path

    def test_px4_errors(self):
        bad = self._write("\n".join([
            "1\t1\tNOT_A_PARAM\t1\t6",        # yok
            "1\t1\tMPC_THR_HOVER\t0.2\t6",    # FLOAT ama INT32 tipi
            "1\t1\tMPC_THR_HOVER\t0.9\t9",    # tekrar + maks 0.8 üstü
            "1\t1\tNAV_DLL_ACT\t4\t6",        # enum'da 4 yok
            "1\t1\tEKF2_EV_CTRL\t16\t6",      # bitmask taşması
        ]) + "\n", ".params")
        errors = validate_params.validate_px4(bad)
        joined = "\n".join(errors)
        for needle in ("NOT_A_PARAM", "tipi", "iki kez", "maks", "izinli", "maks 15"):
            self.assertIn(needle, joined)

    def test_ardupilot_errors(self):
        bad = self._write("ANGLE_MAX,3000\nLAND_SPD_MS,0.1\nMOT_PWM_TYPE,42\n", ".param")
        joined = "\n".join(validate_params.validate_ardupilot(bad))
        self.assertIn("ANGLE_MAX", joined)          # 4.7'de ATC_ANGLE_MAX oldu
        self.assertIn("min 0.3", joined)            # LAND_SPD_MS alt sınırı
        self.assertIn("MOT_PWM_TYPE", joined)       # geçersiz enum


class TestCompanionConfig(unittest.TestCase):
    ENV_FILES = {"dc7.env": "jetson", "dc7-rpi.env": "raspberry_pi"}

    @staticmethod
    def read_env(path: Path) -> dict[str, str]:
        env = {}
        for line in path.read_text(encoding="utf-8").splitlines():
            if line and not line.startswith("#") and "=" in line:
                k, v = line.split("=", 1)
                env[k.strip()] = v.strip()
        return env

    def test_env_files_match_px4_and_tiers(self):
        px4 = merged(PX4_BASE)
        tiers = {load_yaml(p)["profile"]["id"]: load_yaml(p)["profile"]["platform"]
                 for p in budget_calc.all_profiles()}
        for name, platform in self.ENV_FILES.items():
            with self.subTest(env=name):
                env = self.read_env(CONFIG / "companion" / name)
                self.assertEqual(env["DC7_PLATFORM"], platform)
                self.assertEqual(tiers[env["DC7_TIER"]], platform, "seviye ile platform uyumsuz")
                self.assertEqual(int(env["ROS_DOMAIN_ID"]), px4["UXRCE_DDS_DOM_ID"])
                self.assertEqual(int(env["DC7_DDS_BAUD"]), px4["SER_TEL2_BAUD"])
                self.assertNotEqual(env["DC7_DDS_SERIAL"], env["DC7_GIMBAL_SERIAL"])

    def test_mavlink_router_baud_matches_px4(self):
        px4 = merged(PX4_BASE)
        router = (CONFIG / "companion" / "mavlink-router" / "main.conf").read_text(encoding="utf-8")
        baud = int(re.search(r"^Baud\s*=\s*(\d+)", router, re.M).group(1))
        self.assertEqual(baud, px4["SER_TEL1_BAUD"])


class TestAIConfig(unittest.TestCase):
    AGPL_PREFIXES = ("yolo", "bytetrack", "botsort")

    def setUp(self):
        self.ai = load_yaml(CONFIG / "ai" / "perception.yaml")
        self.behavior = load_yaml(CONFIG / "mission" / "behavior.yaml")

    def test_thresholds_match_behavior(self):
        b = self.behavior
        self.assertEqual(self.ai["gesture"]["min_confidence"], b["gestures"]["min_confidence"])
        self.assertEqual(self.ai["palm"]["min_confidence"],
                         b["palm_landing"]["detection"]["min_palm_confidence"])
        self.assertEqual(self.ai["tracking"]["reid"]["min_similarity"],
                         b["follow"]["lost"]["reid_min_similarity"])
        gestures = set(self.ai["gesture"]["classes"]) - {"none"}
        self.assertEqual(gestures, set(b["gestures"]["bindings"]))

    def test_tiers_match_hardware_profiles(self):
        ids = {load_yaml(p)["profile"]["id"] for p in budget_calc.all_profiles()}
        self.assertEqual(set(self.ai["tiers"]), ids)

    def test_commercial_substitutes_cover_agpl_models(self):
        used = set()
        for tier in self.ai["tiers"].values():
            used.add(tier["detector"]["arch"])
            used.add(tier["mot"]["type"])
            used.add(tier["hand"]["detector"])
        agpl = {m for m in used if m.startswith(self.AGPL_PREFIXES)}
        missing = agpl - set(self.ai["commercial_substitutes"])
        self.assertFalse(missing, f"ticari profil için muadili tanımsız: {missing}")
        self.assertIn(self.ai["license_profile"], ("research", "commercial"))

    def test_platforms_match_hardware(self):
        for path in budget_calc.all_profiles():
            prof = load_yaml(path)["profile"]
            with self.subTest(tier=prof["id"]):
                self.assertIn(prof["platform"], self.ai["platforms"])
                self.assertEqual(self.ai["tiers"][prof["id"]]["platform"], prof["platform"])
        for cam in self.ai["cameras"].values():
            self.assertEqual(set(cam["options"]), set(self.ai["platforms"]),
                             "her kamera rolü için her platformda bir seçenek olmalı")

    def test_gimbal_limits(self):
        g = self.behavior["gimbal"]
        self.assertEqual(g["type"], "diy_2axis")
        self.assertLessEqual(g["pitch_limits_deg"][0], -90, "avuç/nadir görüşü için tam aşağı bakabilmeli")
        self.assertEqual(g["roll_limits_deg"][0], -g["roll_limits_deg"][1])
        self.assertGreaterEqual(g["control_rate_hz"], 30)
        self.assertGreater(g["body_yaw_follow_gain"], 0, "2 eksen gimbalda yaw gövdeyle yapılır")

    def test_vlm_never_in_control_loop(self):
        self.assertEqual(self.ai["vlm"]["role"], "target_selector_only")
        self.assertLessEqual(self.ai["vlm"]["max_rate_hz"], 1.0)


class TestAllYamlParses(unittest.TestCase):
    def test_every_yaml_file_parses(self):
        for path in sorted(CONFIG.rglob("*.y*ml")):
            with self.subTest(file=str(path.relative_to(ROOT))):
                self.assertIsNotNone(load_yaml(path))


if __name__ == "__main__":
    unittest.main()
