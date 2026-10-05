"""ucav250 stability library sanity checks."""
from __future__ import annotations

import math
import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from ucav250.analysis import stablib as S  # noqa: E402


class TestStabLib(unittest.TestCase):
    def test_volumes_and_vtail(self):
        v = S.tail_volumes(2.0, 0.4, 5.0, 0.4, 1.8, 0.25, 1.8)
        self.assertAlmostEqual(v["V_H"], 0.9, places=9)
        self.assertAlmostEqual(v["V_V"], 0.045, places=9)
        p = S.v_tail_projection(0.6, 45.0)
        self.assertAlmostEqual(p["S_h_eff"] + p["S_v_eff"], 0.6, places=9)
        self.assertAlmostEqual(p["S_h_eff"], 0.3, places=9)

    def test_downwash(self):
        d_hi = S.downwash_gradient(15, 0.5, 2.0, 0.3, 5.0)
        d_lo = S.downwash_gradient(6, 0.5, 2.0, 0.3, 5.0)
        self.assertTrue(0.1 < d_hi < 0.3, d_hi)
        self.assertGreater(d_lo, d_hi)                       # low AR -> more downwash

    def test_neutral_point(self):
        # no tail, no fuselage: NP at the wing aerodynamic centre
        self.assertAlmostEqual(S.neutral_point(5.0, 1.0, 4.0, 0.0, 3.0, 2.0, 0.4, 0.3), 1.0, places=12)
        # tail moves NP aft, fuselage moves it forward
        np_t = S.neutral_point(5.0, 1.0, 4.0, 0.4, 3.0, 2.0, 0.4, 0.3)
        np_tf = S.neutral_point(5.0, 1.0, 4.0, 0.4, 3.0, 2.0, 0.4, 0.3, cm_alpha_fus=0.2)
        self.assertGreater(np_t, 1.0)
        self.assertLess(np_tf, np_t)
        self.assertAlmostEqual(S.static_margin(1.2, 1.1, 0.4), 0.25, places=12)

    def test_directional(self):
        self.assertGreater(S.cn_beta_vertical(3.0, 0.25, 1.8, 2.0, 5.0), 0)
        self.assertLess(S.cn_beta_fuselage(0.001, 1.6, 0.6, 2.5, 2.0, 5.0), 0)
        self.assertTrue(0.05 < S.kf_fuselage(0.3) < 0.4)


if __name__ == "__main__":
    unittest.main()
