"""config/sensors/catalog.yaml + tools/sensor_matrix.py — sensör kataloğu testleri."""
from __future__ import annotations

import contextlib
import copy
import io
import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "tools"))

import sensor_matrix  # noqa: E402

CATALOG = sensor_matrix.load_catalog()
REFS = sensor_matrix.load_refs()
# Kullanıcıya alternatif sunulması gereken ana görevler
MAIN_ROLES = ("optical_flow", "rangefinder_down", "overhead", "palm_tof", "obstacle", "gnss", "remote_id")


class TestCatalog(unittest.TestCase):
    def test_catalog_is_valid(self):
        self.assertEqual(sensor_matrix.validate(CATALOG, REFS), [])

    def test_each_main_role_has_alternatives(self):
        for role in MAIN_ROLES:
            opts = [o for o in CATALOG["options"] if role in o["roles"]]
            self.assertGreaterEqual(len(opts), 2, role)

    def test_every_tier_sensor_is_cataloged(self):
        """Donanım profillerindeki her 'sensör' bileşeni bir seviye seçimiyle karşılanmalı."""
        for tier, picks in CATALOG["selection"].items():
            profile = sensor_matrix.yaml.safe_load(
                (sensor_matrix.HW_DIR / f"{tier}.yaml").read_text(encoding="utf-8"))
            matches = {CATALOG_BY_ID[oid]["match"] for oid in picks.values()}
            for comp in profile["components"]:
                if comp["category"] == "sensör":
                    self.assertTrue(any(m in comp["name"] for m in matches), f"{tier}: {comp['name']}")

    def test_selected_prices_match_hardware_profiles(self):
        for tier, picks in CATALOG["selection"].items():
            profile = sensor_matrix.yaml.safe_load(
                (sensor_matrix.HW_DIR / f"{tier}.yaml").read_text(encoding="utf-8"))
            for oid in set(picks.values()):
                opt = CATALOG_BY_ID[oid]
                comp = next(c for c in profile["components"] if opt["match"] in c["name"])
                self.assertAlmostEqual(opt["price_usd"], comp["price_usd"], delta=0.5, msg=f"{tier}: {oid}")

    def test_companion_bridge_covers_px4_roles(self):
        topics = CATALOG["bridges"]["px4_dds"]["topics"]
        self.assertEqual(set(topics), {"distance_sensor", "obstacle_distance", "sensor_optical_flow",
                                       "vehicle_visual_odometry"})
        self.assertTrue(all(t.startswith("/fmu/in/") for t in topics.values()))

    def test_unverified_prices_are_flagged(self):
        for o in CATALOG["options"]:
            if o.get("price_usd") is None:
                self.assertFalse(o["price_verified"], o["id"])


class TestBreadth(unittest.TestCase):
    def test_counts_from_reference_data(self):
        counts = sensor_matrix.breadth(CATALOG, REFS)
        self.assertEqual(counts["px4"], {"rangefinder": 15, "optical_flow": 5, "obstacle": 1, "gnss": 6,
                                         "dronecan": 6})
        self.assertEqual(counts["ardupilot"], {"rangefinder": 46, "optical_flow": 8, "obstacle": 12, "gnss": 22,
                                               "gimbal": 13})


class TestValidatorCatchesErrors(unittest.TestCase):
    def _errors_with(self, mutate) -> list[str]:
        cat = copy.deepcopy(CATALOG)
        mutate(cat)
        return sensor_matrix.validate(cat, REFS)

    def _opt(self, cat, oid):
        return next(o for o in cat["options"] if o["id"] == oid)

    def test_unknown_param(self):
        errs = self._errors_with(lambda c: self._opt(c, "tfmini_s")["px4"]["params"].update(SENS_EN_TFMINI=1))
        self.assertTrue(any("SENS_EN_TFMINI" in e for e in errs))

    def test_bad_enum_value(self):
        errs = self._errors_with(lambda c: self._opt(c, "mtf01")["ardupilot"]["params"].update(FLOW_TYPE=9))
        self.assertTrue(any("FLOW_TYPE" in e for e in errs))

    def test_port_on_non_port_param(self):
        errs = self._errors_with(lambda c: self._opt(c, "vl53l1x")["px4"]["params"].update(SENS_EN_VL53L1X="port"))
        self.assertTrue(any("port" in e for e in errs))

    def test_bad_serial_protocol(self):
        errs = self._errors_with(lambda c: self._opt(c, "ld06")["ardupilot"].update(serial_protocol=6))
        self.assertTrue(any("serial_protocol" in e for e in errs))

    def test_selection_must_exist_in_hardware_profile(self):
        errs = self._errors_with(lambda c: c["selection"]["tier-a-ekonomik"].update(optical_flow="h_flow"))
        self.assertTrue(any("donanım profilinde" in e for e in errs))

    def test_companion_role_without_bridge(self):
        errs = self._errors_with(lambda c: c["roles"]["obstacle"].pop("px4_bridge"))
        self.assertTrue(any("köprüsü yok" in e for e in errs))


class TestRendering(unittest.TestCase):
    def test_markdown_has_a_table_per_role(self):
        md = sensor_matrix.render_markdown(CATALOG)
        for role in CATALOG["roles"].values():
            self.assertIn(f"### {role['title']}", md)
        self.assertIn("`/fmu/in/obstacle_distance`", md)

    def test_cli_exit_code(self):
        out = io.StringIO()
        with contextlib.redirect_stdout(out):
            self.assertEqual(sensor_matrix.main([]), 0)
        self.assertIn("Doğrulama: 0 hata", out.getvalue())


CATALOG_BY_ID = {o["id"]: o for o in CATALOG["options"]}

if __name__ == "__main__":
    unittest.main()
