"""ucav250 lifting line and section-polar plumbing."""
from __future__ import annotations

import math
import sys
import unittest
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from ucav250.analysis import aero as AE  # noqa: E402


class TestLiftingLine(unittest.TestCase):
    def test_elliptic_wing(self):
        s = 5.0
        y = np.linspace(0, s, 400)
        c = 0.6 * np.sqrt(np.clip(1 - (y / s) ** 2, 1e-8, None))
        LL = AE.lifting_line(y, c, np.zeros_like(y), np.full_like(y, 2 * math.pi), np.zeros_like(y), s)
        self.assertAlmostEqual(LL["e_inviscid"], 1.0, places=3)
        self.assertAlmostEqual(LL["CL_alpha"], 2 * math.pi / (1 + 2 / LL["AR"]), delta=0.01)

    def test_rectangular_wing(self):
        y = np.linspace(0, 4.0, 200)
        LL = AE.lifting_line(y, np.ones_like(y), np.zeros_like(y), np.full_like(y, 2 * math.pi), np.zeros_like(y), 4.0)
        self.assertTrue(0.92 < LL["e_inviscid"] < 0.95, LL["e_inviscid"])       # Glauert δ ≈ 0.06-0.07 at AR 8
        self.assertTrue(4.75 < LL["CL_alpha"] < 4.95, LL["CL_alpha"])
        # washout lowers the tip loading: zero-lift angle of the wing shifts positive
        LL2 = AE.lifting_line(y, np.ones_like(y), -np.radians(0) - 3.0 * y / 4.0, np.full_like(y, 2 * math.pi),
                              np.zeros_like(y), 4.0)
        self.assertGreater(LL2["alpha_zero_lift_deg"], 0.0)

    def test_surface_analysis_smoke(self):
        secs = [dict(y=0.3, x_le=1.0, z_le=0.0, chord=0.55, twist_deg=0.0, airfoil="naca4415"),
                dict(y=3.0, x_le=1.1, z_le=0.1, chord=0.30, twist_deg=-2.0, airfoil="naca4412")]
        r = AE.surface_analysis(secs, 30.0, 0.0)
        self.assertTrue(1.1 < r["CLmax"] < 1.7, r["CLmax"])
        self.assertTrue(0.0 < r["stall_onset_eta"] < 0.8)                     # tapered + washout: not tip first
        self.assertLess(r["cd_profile"](0.4), r["cd_profile"](1.1))
        self.assertTrue(0.004 < r["cd_profile"](0.5) < 0.015)


if __name__ == "__main__":
    unittest.main()
