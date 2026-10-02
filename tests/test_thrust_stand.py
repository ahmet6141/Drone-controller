"""tools/thrust_stand.py — itki standı CSV → itki eğrisi testleri."""
from __future__ import annotations

import contextlib
import io
import sys
import tempfile
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "tools"))

import budget_calc  # noqa: E402
import thrust_stand  # noqa: E402

TEMPLATE = ROOT / "config" / "hardware" / "thrust" / "ornek-f90-hq7x4x3.csv"
HEADER = "throttle_pct,thrust_g,voltage_v,current_a\n"


def _write(tmp: str, name: str, body: str) -> Path:
    path = Path(tmp) / name
    path.write_text(body, encoding="utf-8")
    return path


class TestThrustStand(unittest.TestCase):
    def test_template_reproduces_profile_curve(self):
        rows = thrust_stand.read_csv(TEMPLATE)
        curve = thrust_stand.thrust_curve(rows, "test")
        tier_a = budget_calc.load(ROOT / "config" / "hardware" / "tier-a-ekonomik.yaml")
        ref = tier_a["propulsion"]["thrust_curve"]
        for got, want in zip(curve["points"], ref["points"]):
            self.assertAlmostEqual(got[0], want[0], delta=0.5)
            self.assertAlmostEqual(got[1], want[1], delta=0.5)
        self.assertEqual([p[0] for p in curve["throttle_points"]], [p[0] for p in ref["throttle_points"]])
        self.assertAlmostEqual(budget_calc.fit_thr_mdl_fac(curve["throttle_points"]),
                               budget_calc.fit_thr_mdl_fac(ref["throttle_points"]), places=3)

    def test_installation_factor(self):
        free = thrust_stand.read_csv(TEMPLATE)
        guarded = [thrust_stand.Row(r.throttle, 0.9 * r.thrust_g, r.voltage_v, r.current_a) for r in free]
        self.assertAlmostEqual(thrust_stand.installation_factor(free, guarded), 0.90, places=3)

    def test_rejects_bad_input(self):
        with tempfile.TemporaryDirectory() as tmp:
            with self.assertRaises(ValueError):
                thrust_stand.read_csv(_write(tmp, "a.csv", "throttle_pct,thrust_g\n50,900\n"))
            with self.assertRaises(ValueError):
                thrust_stand.read_csv(_write(tmp, "b.csv", HEADER + "50,900,24,8\n60,850,24,10\n"))
            with self.assertRaises(ValueError):
                thrust_stand.read_csv(_write(tmp, "c.csv", HEADER + "150,900,24,8\n"))

    def test_cli_with_profile(self):
        out = io.StringIO()
        with contextlib.redirect_stdout(out):
            code = thrust_stand.main([str(TEMPLATE), "--profile",
                                      str(ROOT / "config" / "hardware" / "tier-a-ekonomik.yaml")])
        self.assertEqual(code, 0)
        self.assertIn("THR_MDL_FAC", out.getvalue())
        self.assertIn("thrust_curve:", out.getvalue())


if __name__ == "__main__":
    unittest.main()
