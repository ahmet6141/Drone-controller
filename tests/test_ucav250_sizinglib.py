"""ucav250 sizing library: constraint curves, mission fractions, mass iteration."""
from __future__ import annotations

import math
import sys
import unittest
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from ucav250.analysis import aerolib as AL  # noqa: E402
from ucav250.analysis import sizinglib as SZ  # noqa: E402

A = SZ.Aero(cd0=0.032, k=AL.k_induced(16, 0.78), clmax=1.35, clmax_to=1.6, clmax_ld=1.8)


class TestSizingLib(unittest.TestCase):
    def test_constraints_monotonic(self):
        ws = np.linspace(300, 900, 13)
        cr = SZ.pw_cruise(ws, 45.0, 3000.0, A)
        self.assertTrue(np.all(cr > 0))
        to = SZ.pw_takeoff(ws, 250.0, A)
        self.assertTrue(np.all(np.diff(to) > 0))                    # heavier loading -> more power for the same run
        self.assertGreater(SZ.pw_ceiling(600.0, 5000.0, A), SZ.pw_ceiling(600.0, 2000.0, A))
        self.assertGreater(SZ.pw_climb(600.0, 4.0, 0.0, A), SZ.pw_climb(600.0, 2.0, 0.0, A))
        self.assertAlmostEqual(SZ.ws_stall(20.0, 1.5), 0.5 * 1.225 * 400 * 1.5, delta=0.01)
        ws_l = SZ.ws_landing(150.0, 1.8)
        vtd = 1.15 * math.sqrt(2 * ws_l / (1.225 * 1.8))
        self.assertAlmostEqual(vtd ** 2 / (2 * SZ.G0 * 0.3) + vtd, 150.0, delta=0.5)

    def test_design_point(self):
        ws = np.linspace(200, 1000, 81)
        curves = {"cruise": SZ.pw_cruise(ws, 45.0, 3000.0, A), "to": SZ.pw_takeoff(ws, 250.0, A)}
        dp = SZ.design_point(ws, curves, 700.0)
        self.assertLessEqual(dp["ws"], 700.0)
        self.assertIn(dp["active"], curves)

    def test_mission_and_mass(self):
        bsfc = 500e-3 / 3.6e6                                        # 500 g/kWh in kg/J
        segs = [SZ.Segment("warmup"), SZ.Segment("taxi"), SZ.Segment("takeoff"),
                SZ.Segment("climb", 3000.0, V=30.0, LD=14.0, eta=0.72, bsfc=bsfc),
                SZ.Segment("loiter", 12 * 3600.0, V=33.0, LD=17.0, eta=0.78, bsfc=bsfc),
                SZ.Segment("descent"), SZ.Segment("landing"),
                SZ.Segment("reserve", 1800.0, V=33.0, LD=17.0, eta=0.78, bsfc=bsfc)]
        ff, fr = SZ.mission_fuel_fraction(segs)
        self.assertTrue(0.15 < ff < 0.40, ff)
        self.assertTrue(all(0 < f < 1 for f in fr))
        a, c = SZ.fit_empty_fraction(np.array([50.0, 100.0, 200.0]), np.array([30.0, 55.0, 100.0]))
        self.assertAlmostEqual(a * 100.0 ** c, 0.55, delta=0.03)
        mm = SZ.MassModel(payload=20.0, fixed=25.0, fuel_fraction=ff, airframe_fraction_fn=lambda m0: 0.30)
        r = mm.solve()
        self.assertAlmostEqual(r["mtow"], 45.0 / (1 - ff - 0.30), places=5)
        self.assertAlmostEqual(r["mtow"], r["fuel"] + r["airframe"] + r["payload"] + r["fixed"], places=6)


if __name__ == "__main__":
    unittest.main()
