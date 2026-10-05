"""ucav250 aero/performance library against textbook values."""
from __future__ import annotations

import math
import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from ucav250.analysis import aerolib as A  # noqa: E402


class TestAeroLib(unittest.TestCase):
    def test_isa(self):
        a0 = A.isa(0.0)
        self.assertAlmostEqual(a0["rho"], 1.225, places=3)
        self.assertAlmostEqual(A.isa(3000.0)["rho"], 0.9093, places=3)

    def test_friction_and_form(self):
        self.assertAlmostEqual(A.cf_flat(1e7), 0.455 / 7 ** 2.58, places=9)
        self.assertGreater(A.cf_flat(1e6, 0.0), A.cf_flat(1e6, 0.5))
        self.assertGreater(A.ff_wing(0.15), A.ff_wing(0.10))
        self.assertGreater(A.ff_body(4), A.ff_body(8))

    def test_lift_and_oswald(self):
        a = A.lift_slope(8.0)
        self.assertTrue(4.4 < a < 5.0, a)
        self.assertTrue(0.6 < A.oswald_straight(15.0) < 0.7)            # Raymer fit: low at high AR
        e = A.oswald_nita_scholz(15.0, 0.5, 0.0, 0.08)
        self.assertTrue(0.74 < e < 0.82, e)
        self.assertGreater(A.oswald_nita_scholz(10.0, 0.5), A.oswald_nita_scholz(20.0, 0.5))

    def test_speeds_and_breguet(self):
        W, S, cd0, AR, e = 120 * A.G0, 2.4, 0.03, 15.0, 0.8
        k = A.k_induced(AR, e)
        sp = A.speeds(W, S, cd0, k, 1.225)
        self.assertAlmostEqual(sp["LD_max"], 0.5 / math.sqrt(cd0 * k), places=9)
        self.assertLess(sp["V_min_power"], sp["V_ld_max"])
        # power at V_min_power is the minimum of P_required
        p0 = A.power_required(sp["V_min_power"], W, S, cd0, k, 1.225)
        self.assertLess(p0, A.power_required(sp["V_min_power"] * 1.1, W, S, cd0, k, 1.225))
        self.assertLess(p0, A.power_required(sp["V_min_power"] * 0.9, W, S, cd0, k, 1.225))
        E = A.breguet_endurance_prop(0.75, 1e-7, 1.0, 0.05, 1.225, S, W, 0.85 * W)
        self.assertGreater(E, 0)

    def test_propulsion_and_field(self):
        T0 = A.static_thrust(18000.0, 0.8)
        self.assertTrue(350 < T0 < 700, T0)
        eta = A.prop_efficiency(35.0, 12000.0, 0.8, 1.225)
        self.assertTrue(0.6 < eta <= 0.82, eta)
        self.assertAlmostEqual(A.power_lapse(1.0), 1.0, places=9)
        to = A.takeoff_ground_roll(120 * A.G0, 2.4, 1.6, 0.04, A.k_induced(15, 0.8), 450.0, 300.0)
        self.assertTrue(50 < to["ground_roll"] < 400, to)
        ld = A.landing_roll(110 * A.G0, 2.4, 1.8, 0.05, A.k_induced(15, 0.8))
        self.assertTrue(30 < ld["ground_roll"] < 400, ld)


if __name__ == "__main__":
    unittest.main()
