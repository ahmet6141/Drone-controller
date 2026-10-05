"""ucav250 OML evaluators (fuselage superellipse stations, lifting surfaces, airfoils)."""
from __future__ import annotations

import math
import sys
import unittest
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
try:
    from ucav250.core.geom import shell_from_grid
    from ucav250.design import oml as O
    HAVE = True
except Exception:  # pragma: no cover
    HAVE = False

ST = [[0.0, 0.0, 0.0, 0.0, 2.0, 2.0], [0.3, 0.30, 0.28, 0.0, 2.2, 2.5], [1.2, 0.42, 0.40, 0.02, 2.5, 3.0],
      [2.6, 0.30, 0.30, 0.05, 2.4, 2.4], [3.2, 0.10, 0.10, 0.08, 2.0, 2.0]]


@unittest.skipUnless(HAVE, "ucav250 dependencies missing")
class TestFuselage(unittest.TestCase):
    def setUp(self):
        self.F = O.Fuselage(np.array(ST))

    def test_closed_mesh_round_nose(self):
        m = self.F.mesh(100, 72)
        self.assertTrue(m.check(self_intersect=True)["ok"])
        lo, hi = m.bounds()
        self.assertAlmostEqual(lo[0], 0.0, places=9)
        self.assertAlmostEqual(hi[0], 3.2, places=9)
        self.assertAlmostEqual(hi[1], 0.21, delta=1e-3)          # half max width

    def test_outward_normal_and_inward_panel(self):
        n = self.F.normal(1.2, math.pi / 2)
        np.testing.assert_allclose(n, [0, 1, 0], atol=1e-3)
        g = self.F.grid(1.0, 1.5, 1.2, 1.9, 12, 10)
        sh = shell_from_grid(g, 0.003)
        self.assertTrue(sh.check(self_intersect=True)["ok"])
        r_out = np.hypot(g[..., 1], g[..., 2] - 0.02).mean()
        inner = sh.V[len(sh.V) // 2:]
        self.assertLess(np.hypot(inner[:, 1], inner[:, 2] - 0.02).mean(), r_out)


@unittest.skipUnless(HAVE, "ucav250 dependencies missing")
class TestSurfaces(unittest.TestCase):
    def test_wing_mac_and_mesh(self):
        secs = [dict(y=0.0, x_le=1.0, z_le=0.0, chord=0.6, twist_deg=2, airfoil="naca4415"),
                dict(y=2.0, x_le=1.15, z_le=0.1, chord=0.3, twist_deg=-1, airfoil="naca4412")]
        W = O.LiftingSurface(secs, n_chord=40)
        m = W.mesh(refine=3)
        self.assertTrue(m.check(self_intersect=True)["ok"])
        mac = O.mean_aerodynamic_chord(secs)
        lam = 0.5
        self.assertAlmostEqual(mac["mac"], 2 / 3 * 0.6 * (1 + lam + lam ** 2) / (1 + lam), places=4)
        self.assertAlmostEqual(mac["half_area"], 0.9, places=6)

    def test_inverted_v_and_fin_orientation(self):
        d = math.radians(-35)
        sd = (0, math.cos(d), math.sin(d))
        secs = [dict(y=0.05, x_le=2.7, z_le=0.0, chord=0.35, airfoil="naca0012", span_dir=sd),
                dict(y=0.05 + 0.7 * math.cos(d), x_le=2.95, z_le=0.7 * math.sin(d), chord=0.2, airfoil="naca0012",
                     span_dir=sd)]
        m = O.LiftingSurface(secs).mesh(2)
        self.assertTrue(m.check(self_intersect=True)["ok"])
        self.assertLess(m.bounds()[0][2], -0.35)                  # tips below the root (anhedral)
        fin = [dict(y=0.0, x_le=2.6, z_le=0.1, chord=0.4, airfoil="naca0010", span_dir=(0, 0, 1)),
               dict(y=0.0, x_le=2.85, z_le=0.6, chord=0.2, airfoil="naca0010", span_dir=(0, 0, 1))]
        f = O.LiftingSurface(fin).mesh(2)
        self.assertTrue(f.check()["ok"])
        self.assertLess(f.bounds()[1][1] - f.bounds()[0][1], 0.06)  # thin in Y: vertical fin

    def test_airfoil_te_and_thickness(self):
        x, yu, yl = O.resampled("naca0012", 80, 0.005)
        self.assertAlmostEqual(yu[-1] - yl[-1], 0.005, places=6)
        t, xt = O.max_thickness("naca0012")
        self.assertAlmostEqual(t, 0.12, delta=0.002)
        self.assertAlmostEqual(xt, 0.30, delta=0.03)


if __name__ == "__main__":
    unittest.main()
