"""ucav250 shared structural generators: closed meshes that fit the OML; control surface clears its range."""
from __future__ import annotations

import math
import sys
import unittest
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from ucav250.core.geom import Mesh, intersection_volume, min_gap  # noqa: E402
from ucav250.core.parts import Joint  # noqa: E402
from ucav250.design import structgen as SG  # noqa: E402
from ucav250.design.oml import Fuselage, LiftingSurface  # noqa: E402

WING = LiftingSurface([
    dict(y=0.3, x_le=1.0, z_le=0.0, chord=0.55, twist_deg=1.0, airfoil="naca4415"),
    dict(y=1.5, x_le=1.03, z_le=0.03, chord=0.45, twist_deg=0.0, airfoil="naca4415"),
    dict(y=3.0, x_le=1.1, z_le=0.1, chord=0.30, twist_deg=-2.0, airfoil="naca4412")])
FUS = Fuselage(np.array([[0, 0, 0, 0, 2, 2, 0.4], [0.4, 0.35, 0.3, 0, 1.3, 1.6, 0.4],
                         [2.0, 0.5, 0.42, 0.0, 1.3, 1.6, 0.4], [3.4, 0.12, 0.12, 0.05, 2, 2, 0.5]]))


class TestStructGen(unittest.TestCase):
    def test_section_exact_on_loft(self):
        poly, fr = SG.section2d(WING, 1.5)
        self.assertAlmostEqual(poly.bounds[2], 0.45, delta=2e-3)        # chord at a defining section
        e = WING.span_coords()
        mid = 0.5 * (e[1] + e[2])
        self.assertAlmostEqual(WING.chord_at(mid), 0.375, places=9)

    def test_rib_inside_wing(self):
        r = SG.rib(WING, 1.0, 0.003, 0.0015, cutouts=[(0.25, 0.03, None)], holes=[(0.45, 0.04)])
        self.assertTrue(r.check(self_intersect=True)["ok"])
        w = WING.mesh()
        self.assertAlmostEqual(intersection_volume(r, w), r.volume(), delta=0.02 * r.volume())

    def test_control_surface_sweep(self):
        fx, mv, hf = SG.control_surface_regions(0.72, 0.0015, 0.5)
        fixed = SG.loft_region(WING, 1.6, 2.8, fx)
        mov = SG.loft_region(WING, 1.6, 2.8, mv)
        self.assertTrue(fixed.check(self_intersect=True)["ok"] and mov.check(self_intersect=True)["ok"])
        a, b = SG.hinge_axis(WING, 1.6, 2.8, hf)
        j = Joint("ail", "revolute", a, b - a, -math.radians(30), math.radians(30))
        for d in np.radians([-30, -20, -10, 0, 10, 20, 30]):
            m = Mesh(j.apply(mov.V, d), mov.F)
            self.assertLess(intersection_volume(fixed, m), 1e-9)
            self.assertGreater(min_gap(fixed, m, 0.01), 0.001)

    def test_fuselage_parts(self):
        bh = SG.bulkhead(FUS, 1.0, 0.004, 0.002)
        rf = SG.ring_frame(FUS, 1.5, 0.003, 0.002, 0.03)
        pn = SG.fuselage_panel(FUS, 0.8, 1.8, 0.3, 1.2, 0.0012)
        fm = FUS.mesh(200, 192)
        for m in (bh, rf, pn):
            self.assertTrue(m.check(self_intersect=True)["ok"])
            # outer faces lie on the exact OML; the faceted reference mesh is slightly inside it
            self.assertAlmostEqual(intersection_volume(m, fm), m.volume(), delta=0.03 * m.volume())


if __name__ == "__main__":
    unittest.main()
