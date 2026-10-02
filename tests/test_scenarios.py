"""config/budget/scenarios.yaml + tools/scenarios.py — bütçe kesinti senaryoları testleri."""
from __future__ import annotations

import contextlib
import copy
import io
import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "tools"))

import scenarios  # noqa: E402

SPEC = scenarios.load()
BASE, SINGLES, PACKAGES = scenarios.run(SPEC)
BY_ID = {s["id"]: s for s in SPEC["scenarios"]}
# Hiçbir kesinti paketi bunları kaldıramaz: güvenlik ve mevzuat için zorunlu bileşenler
MUST_KEEP = ("pervane koruması", "tutamağı", "Remote ID", "ELRS alıcı", "GNSS", "MTF-01", "VL53L8CX")


class TestScenarios(unittest.TestCase):
    def test_every_scenario_saves_money(self):
        for r in SINGLES:
            self.assertLess(r.d_total_usd, 0, r.id)

    def test_known_deltas(self):
        r = {x.id: x for x in SINGLES}
        self.assertAlmostEqual(r["pi5-4gb"].d_airframe_usd, -65)
        self.assertAlmostEqual(r["pi5-4gb"].d_auw_g, 0)
        self.assertAlmostEqual(r["ai-hat-26t"].d_airframe_usd, -80.05)
        self.assertAlmostEqual(r["gimbal-servo-tilt"].d_auw_g, -60)
        self.assertAlmostEqual(r["fc-pixhawk-6c-mini"].d_auw_g, 34.9)
        self.assertAlmostEqual(r["link-wifi-dev"].d_ground_usd, -80)
        self.assertGreater(r["gimbal-servo-tilt"].d_hover_min, 0)

    def test_metadata(self):
        for s in SPEC["scenarios"]:
            self.assertIn(s["risk"], ("düşük", "orta", "yüksek"), s["id"])
            self.assertIn(s["scope"], ("kalıcı", "Ar-Ge"), s["id"])
            self.assertTrue(s["effect"] and s["source"], s["id"])
            for other in s.get("excludes", []):
                self.assertIn(s["id"], BY_ID[other].get("excludes", []), f"{s['id']} ↔ {other} simetrik değil")


class TestPackages(unittest.TestCase):
    def test_packages_stay_within_limits(self):
        for r in PACKAGES:
            self.assertTrue(r.budget.ok, f"{r.id}: {[c.detail for c in r.budget.checks if not c.ok]}")

    def test_packages_are_ordered_by_savings(self):
        savings = [-r.d_total_usd for r in PACKAGES]
        self.assertEqual(savings, sorted(savings))

    def test_recommended_package_is_low_risk_and_permanent(self):
        pkg = next(p for p in SPEC["packages"] if p["id"] == "onerilen")
        for sid in pkg["scenarios"]:
            self.assertEqual((BY_ID[sid]["risk"], BY_ID[sid]["scope"]), ("düşük", "kalıcı"), sid)

    def test_safety_components_survive_every_package(self):
        for pkg in SPEC["packages"]:
            names = [c["name"] for c in scenarios.package_profile(SPEC, pkg)["components"]]
            for must in MUST_KEEP:
                self.assertTrue(any(must in n for n in names), f"{pkg['id']}: {must!r} kalmadı")


class TestErrors(unittest.TestCase):
    def setUp(self):
        self.base = scenarios.base_profile(SPEC)

    def test_ambiguous_match(self):
        with self.assertRaises(scenarios.ScenarioError):
            scenarios.apply(self.base, {"id": "x", "ops": [{"op": "adjust", "match": "Raspberry Pi", "price_usd": -1}]})

    def test_missing_match(self):
        with self.assertRaises(scenarios.ScenarioError):
            scenarios.apply(self.base, {"id": "x", "ops": [{"op": "remove", "match": "olmayan parça"}]})

    def test_negative_price(self):
        with self.assertRaises(scenarios.ScenarioError):
            scenarios.apply(self.base, {"id": "x", "ops": [{"op": "adjust", "match": "VL53L1X", "price_usd": -100}]})

    def test_exclusive_scenarios(self):
        spec = copy.deepcopy(SPEC)
        spec["packages"] = [{"id": "p", "title": "p", "scenarios": ["ai-hat-26t", "ai-hat-13t"]}]
        with self.assertRaises(scenarios.ScenarioError):
            scenarios.run(spec)

    def test_replaced_item_cannot_be_patched_again(self):
        spec = copy.deepcopy(SPEC)
        spec["scenarios"].append({"id": "ark-ucuz", "title": "t", "risk": "düşük", "scope": "kalıcı", "effect": "e",
                                  "source": "s", "ops": [{"op": "adjust", "match": "ARK FPV", "price_usd": -10}]})
        pkg = {"id": "p", "title": "p", "scenarios": ["fc-pixhawk-6c-mini", "ark-ucuz"]}
        with self.assertRaises(scenarios.ScenarioError):
            scenarios.package_profile(spec, pkg)

    def test_cli(self):
        out = io.StringIO()
        with contextlib.redirect_stdout(out):
            self.assertEqual(scenarios.main(["--markdown"]), 0)
        self.assertIn("| Paket |", out.getvalue())


if __name__ == "__main__":
    unittest.main()
