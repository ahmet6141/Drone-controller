"""ucav250 hand-calculation library against closed-form results."""
from __future__ import annotations

import math
import sys
import unittest
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from ucav250.analysis import structlib as S  # noqa: E402


class TestStructLib(unittest.TestCase):
    def test_uniform_cantilever(self):
        y = np.linspace(0, 2.0, 401)
        V, M = S.beam_loads(y, np.full_like(y, 100.0))
        self.assertAlmostEqual(V[0], 200.0, places=6)
        self.assertAlmostEqual(M[0], 100.0 * 2.0 ** 2 / 2, places=3)

    def test_schrenk_normalised_and_root_heavy(self):
        y = np.linspace(0, 2.5, 501)
        c = np.interp(y, [0, 2.5], [0.6, 0.3])
        l = S.schrenk(y, c, 2.5)
        self.assertAlmostEqual(np.trapz(l, y), 1.0, places=9)
        self.assertGreater(l[0], l[-1])

    def test_gust_and_vn(self):
        ws, a, mac = 120 * 9.80665, 5.5, 0.45
        kg = S.gust_factor(ws, a, mac)
        self.assertTrue(0.5 < kg < 0.9)
        up, dn = S.gust_n(40.0, 15.24, ws, a, mac)
        self.assertAlmostEqual(up - 1.0, 1.0 - dn, places=12)
        vn = S.vn_diagram(ws, 1.4, -0.8, 3.8, -1.5, 40.0, 56.0, a, mac)
        self.assertAlmostEqual(vn["VA"], vn["VS"] * math.sqrt(3.8), places=9)
        self.assertGreaterEqual(vn["n_limit_pos"], 3.8)

    def test_sections_and_joints(self):
        t = S.tube(0.02, 0.018)
        self.assertAlmostEqual(t["I"], math.pi / 4 * (0.02 ** 4 - 0.018 ** 4), places=15)
        self.assertAlmostEqual(S.bolt_shear(0.006, 400e6, 2), 2 * math.pi / 4 * 0.006 ** 2 * 400e6, places=3)
        lug = S.lug_axial(500e6, 800e6, w=0.02, D=0.008, t=0.005, e=0.012)
        self.assertGreater(lug["P_allow"], 0)
        self.assertLessEqual(lug["P_allow"], lug["P_net"])
        self.assertAlmostEqual(S.ms(1500.0, 1000.0, 1.5), 0.0, places=12)
        self.assertAlmostEqual(S.euler_buckling(70e9, 1e-9, 1.0, 1.0), math.pi ** 2 * 70.0, places=6)

    def test_finite_plate_shear_buckling(self):
        """Fix round 3: finite orthotropic plate in shear - isotropic limit k_s = 5.35 + 4 (s/l)^2 on the short side,
        symmetric in a <-> b, and the long-plate value for l >> s."""
        Dv = 10.0
        D = np.array([[Dv, 0.3 * Dv, 0.0], [0.3 * Dv, Dv, 0.0], [0.0, 0.0, 0.35 * Dv]])      # D12 + 2 D66 = D
        b = 0.08
        long_ = S.orthotropic_shear_buckling_long(D, b)
        self.assertAlmostEqual(long_ / (math.pi ** 2 * Dv / b ** 2), 5.34, delta=0.02)
        self.assertAlmostEqual(S.orthotropic_shear_buckling_finite(D, 100.0, b) / long_, 1.0, places=5)
        sq = S.orthotropic_shear_buckling_finite(D, b, b)
        self.assertAlmostEqual(sq / long_, 9.35 / 5.35, places=9)
        self.assertAlmostEqual(S.orthotropic_shear_buckling_finite(D, 0.06, b),
                               S.orthotropic_shear_buckling_finite(D, b, 0.06), places=6)
        self.assertGreater(S.orthotropic_shear_buckling_finite(D, 0.06, b), long_)


if __name__ == "__main__":
    unittest.main()
