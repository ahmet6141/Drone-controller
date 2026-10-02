"""Konfigürasyon tutarlılık testleri.

Çalıştırma (ek bağımlılık gerekmez, yalnızca PyYAML):
    python3 -m unittest discover -s tests -v
"""
from __future__ import annotations

import re
import sys
import unittest
from pathlib import Path

import yaml

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "tools"))

import budget_calc  # noqa: E402

CONFIG = ROOT / "config"
PARAM_NAME = re.compile(r"^[A-Z][A-Z0-9_]{0,15}$")  # PX4 ve ArduPilot: en fazla 16 karakter


def load_yaml(path: Path) -> dict:
    with open(path, encoding="utf-8") as fh:
        return yaml.safe_load(fh)


def parse_px4_params(path: Path) -> dict[str, float]:
    """QGroundControl parametre dosyası: 'vehicle<TAB>comp<TAB>NAME<TAB>value<TAB>type'."""
    params: dict[str, float] = {}
    for lineno, raw in enumerate(path.read_text(encoding="utf-8").splitlines(), 1):
        line = raw.strip()
        if not line or line.startswith("#"):
            continue
        fields = raw.split("\t")
        if len(fields) != 5:
            raise AssertionError(f"{path.name}:{lineno}: 5 TAB ayrılmış alan bekleniyordu")
        _, _, name, value, ptype = fields
        if not PARAM_NAME.match(name):
            raise AssertionError(f"{path.name}:{lineno}: geçersiz parametre adı {name!r}")
        if ptype not in ("6", "9"):  # MAV_PARAM_TYPE_INT32 / MAV_PARAM_TYPE_REAL32
            raise AssertionError(f"{path.name}:{lineno}: tip 6 (INT32) veya 9 (REAL32) olmalı")
        if ptype == "6" and not re.fullmatch(r"-?\d+", value):
            raise AssertionError(f"{path.name}:{lineno}: INT32 parametrede tam sayı olmalı")
        if name in params:
            raise AssertionError(f"{path.name}:{lineno}: {name} dosyada iki kez tanımlı")
        params[name] = float(value)
    return params


def parse_ardupilot_params(path: Path) -> dict[str, float]:
    """Mission Planner / MAVProxy formatı: 'NAME,value' (# yorum satırları serbest)."""
    params: dict[str, float] = {}
    for lineno, raw in enumerate(path.read_text(encoding="utf-8").splitlines(), 1):
        line = raw.split("#", 1)[0].strip()
        if not line:
            continue
        name, sep, value = line.partition(",")
        if not sep:
            raise AssertionError(f"{path.name}:{lineno}: 'NAME,value' bekleniyordu")
        name, value = name.strip(), value.strip()
        if not PARAM_NAME.match(name):
            raise AssertionError(f"{path.name}:{lineno}: geçersiz parametre adı {name!r}")
        if name in params:
            raise AssertionError(f"{path.name}:{lineno}: {name} dosyada iki kez tanımlı")
        params[name] = float(value)
    return params


def all_px4_params() -> dict[str, tuple[float, str]]:
    merged: dict[str, tuple[float, str]] = {}
    for path in sorted((CONFIG / "px4").glob("*.params")):
        for name, value in parse_px4_params(path).items():
            if name in merged and merged[name][0] != value:
                raise AssertionError(
                    f"{name} çelişkili: {merged[name][1]}={merged[name][0]} / {path.name}={value}")
            merged[name] = (value, path.name)
    return merged


class TestBudgetMath(unittest.TestCase):
    CURVE = [(100.0, 10.0), (500.0, 80.0), (1000.0, 250.0)]

    def test_interpolation(self):
        self.assertAlmostEqual(budget_calc.power_at_thrust(self.CURVE, 300.0), 45.0)
        self.assertAlmostEqual(budget_calc.power_at_thrust(self.CURVE, 50.0), 5.0)
        self.assertAlmostEqual(budget_calc.thrust_at_power(self.CURVE, 45.0), 300.0)
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

    def test_synthetic_profile(self):
        profile = {
            "profile": {"id": "t", "name": "test"},
            "limits": {"max_auw_g": 1000, "min_tw_ratio": 2.0, "min_hover_time_min": 1},
            "components": [{"name": "gövde", "category": "gövde", "mass_g": 200, "power_w": 10}],
            "propulsion": {
                "motor_count": 4,
                "motor": {"mass_g": 50}, "prop": {"mass_g": 5}, "esc": {"mass_g": 20},
                "thrust_curve": {"points": [list(p) for p in self.CURVE]},
            },
            "battery": {"cells_series": 6, "nominal_cell_v": 3.7, "capacity_mah": 1000,
                        "usable_fraction": 0.8, "mass_g": 200, "max_continuous_a": 100},
        }
        b = budget_calc.compute(profile)
        self.assertAlmostEqual(b.auw_g, 200 + 4 * 55 + 20 + 200)          # 640 g
        self.assertAlmostEqual(b.hover_thrust_per_motor_g, 160.0)
        hover_w = 4 * (10 + 70 * 60 / 400) + 10                           # 92 W
        self.assertAlmostEqual(b.total_hover_w, hover_w)
        self.assertAlmostEqual(b.hover_time_min, 6 * 3.7 * 0.8 / hover_w * 60)
        self.assertAlmostEqual(b.hover_throttle, 640 / 4000)
        self.assertTrue(b.ok)


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
    def test_px4_files_parse_without_conflicts(self):
        params = all_px4_params()
        self.assertGreater(len(params), 30)

    def test_px4_matches_safety_config(self):
        params = {k: v for k, (v, _) in all_px4_params().items()}
        s = load_yaml(CONFIG / "mission" / "safety.yaml")
        self.assertEqual(params["GF_MAX_HOR_DIST"], s["geofence"]["max_horizontal_distance_m"])
        self.assertEqual(params["GF_MAX_VER_DIST"], s["geofence"]["max_altitude_m"])
        self.assertAlmostEqual(params["BAT_LOW_THR"], s["battery"]["low_fraction"])
        self.assertAlmostEqual(params["BAT_CRIT_THR"], s["battery"]["critical_fraction"])
        self.assertAlmostEqual(params["BAT_EMERGEN_THR"], s["battery"]["emergency_fraction"])

    def test_px4_hover_thrust_matches_budget(self):
        """MPC_THR_HOVER başlangıcı, önerilen seviyenin bütçe hesabıyla uyumlu olmalı."""
        params = {k: v for k, (v, _) in all_px4_params().items()}
        b = budget_calc.compute(load_yaml(CONFIG / "hardware" / "tier-b-pro.yaml"))
        self.assertAlmostEqual(params["MPC_THR_HOVER"], b.hover_throttle, delta=0.05)

    def test_ardupilot_files_parse(self):
        for path in sorted((CONFIG / "ardupilot").glob("*.param")):
            with self.subTest(file=path.name):
                self.assertGreater(len(parse_ardupilot_params(path)), 30)


class TestAllYamlParses(unittest.TestCase):
    def test_every_yaml_file_parses(self):
        for path in sorted(CONFIG.rglob("*.y*ml")):
            with self.subTest(file=str(path.relative_to(ROOT))):
                self.assertIsNotNone(load_yaml(path))


if __name__ == "__main__":
    unittest.main()
