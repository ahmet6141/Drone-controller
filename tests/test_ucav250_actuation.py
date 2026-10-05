"""ucav250 shared hinge/actuator generators: closed meshes, no interference through the hinge range, four-bar."""
from __future__ import annotations

import math
import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
try:
    from ucav250.core import geom as G
    from ucav250.design import actuation as A
    HAVE = True
except Exception:  # pragma: no cover
    HAVE = False


@unittest.skipUnless(HAVE, "ucav250 dependencies missing")
class TestHinge(unittest.TestCase):
    def test_clevis_tongue_pin_clear_through_range(self):
        hs = A.HingeSpec()
        p, a = (1.0, 0.5, 0.0), (0, 1, 0)
        fx = A.hinge_bracket_fixed(p, a, (1, 0, 0), hs, reach=0.03)
        mv = A.hinge_bracket_moving(p, a, (-1, 0, 0), hs, reach=0.025)
        pin = A.hinge_pin(p, a, hs)
        for m in (fx["mesh"], mv["mesh"], pin):
            self.assertTrue(m.check(self_intersect=True)["ok"])
        self.assertLess(G.intersection_volume(pin, fx["mesh"]), 1e-12)
        self.assertLess(G.intersection_volume(pin, mv["mesh"]), 1e-12)
        for ang in (-0.6, -0.3, 0.0, 0.3, 0.6):
            mvr = mv["mesh"].rotated_about(p, a, ang)
            self.assertLess(G.intersection_volume(fx["mesh"], mvr), 1e-12, ang)
            self.assertGreaterEqual(G.min_gap(fx["mesh"], mvr, 0.01), hs.gap - 1e-6)


@unittest.skipUnless(HAVE, "ucav250 dependencies missing")
class TestActuators(unittest.TestCase):
    def test_servo_arm_horn_rod_closed(self):
        sv = A.servo_body((0.8, 0.5, -0.02), (1, 0, 0), (0, 0, 1), A.ServoSpec())
        arm = A.servo_arm(sv["shaft_p"], (0, 0, 1), (0, 1, 0), 0.02)
        horn = A.control_horn((1.05, 0.52, -0.01), (0, 0, -1), (0, 1, 0), 0.03)
        rod = A.pushrod(arm["tip"], horn["hole"])
        for m in (sv["mesh"], arm["mesh"], horn["mesh"], rod["mesh"]):
            self.assertTrue(m.check(self_intersect=True)["ok"])

    def test_four_bar(self):
        k = A.linkage_kinematics((1.0, 0.5, 0.0), (0, 1, 0), (1.01, 0.5, -0.03), (0.85, 0.5, -0.03), (0, 1, 0), 0.02,
                                 math.pi / 2, (math.radians(-25), math.radians(25)))
        self.assertAlmostEqual(k["rod_length"], math.hypot(0.16, 0.02), places=6)
        self.assertGreater(k["min_transmission_deg"], 40.0)
        self.assertLess(k["servo_travel_deg"], 120.0)


if __name__ == "__main__":
    unittest.main()
