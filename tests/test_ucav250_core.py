"""ucav250 core: geometry kernel and part registry (no aircraft data needed)."""
from __future__ import annotations

import math
import sys
import unittest
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

try:
    from shapely.geometry import Point, Polygon

    from ucav250.core import geom as G
    from ucav250.core.parts import Fastener, Joint, Part, Registry, layup_props, mirror_part, part_number
    HAVE = True
except Exception:  # pragma: no cover - optional deps missing
    HAVE = False


@unittest.skipUnless(HAVE, "ucav250 dependencies missing (bash ucav250/setup_env.sh)")
class TestGeom(unittest.TestCase):
    def test_primitives_closed_and_exact(self):
        b = G.box((1, 2, 3))
        self.assertTrue(b.check(self_intersect=True)["ok"])
        self.assertAlmostEqual(b.volume(), 6.0, places=9)
        c = G.cylinder(0.1, (0, 0, 0), (0, 0, 1), n=96)
        self.assertTrue(c.check()["ok"])
        self.assertAlmostEqual(c.volume() / (math.pi * 0.01), 1.0, delta=0.002)
        t = G.tube(0.05, 0.04, (0, 0, 0), (1, 0, 0), n=96)
        self.assertTrue(t.check(self_intersect=True)["ok"])
        self.assertAlmostEqual(t.volume() / (math.pi * (0.05 ** 2 - 0.04 ** 2)), 1.0, delta=0.003)

    def test_extrude_with_hole(self):
        poly = Polygon([(0, 0), (0.3, 0), (0.3, 0.2), (0, 0.2)]).difference(Point(0.15, 0.1).buffer(0.04, 32))
        e = G.extrude(poly, 0.005, origin=(1, 0, 0), u=(0, 1, 0), v=(0, 0, 1))
        self.assertTrue(e.check(self_intersect=True)["ok"])
        self.assertAlmostEqual(e.volume(), poly.area * 0.005, places=12)

    def test_mass_props_box(self):
        mp = G.box((1, 2, 3), center=(1, 2, 3)).mass_props(1000.0)
        self.assertAlmostEqual(mp["mass"], 6000.0, places=6)
        np.testing.assert_allclose(mp["cg"], [1, 2, 3], atol=1e-12)
        np.testing.assert_allclose(np.diag(mp["inertia"]), [6500, 5000, 2500], rtol=1e-9)

    def test_booleans_and_gap(self):
        a = G.box((1, 1, 1))
        b = G.box((1, 1, 1), center=(0.5, 0, 0))
        self.assertAlmostEqual(G.intersection_volume(a, b), 0.5, places=9)
        self.assertAlmostEqual(G.union([a, b]).volume(), 1.5, places=9)
        self.assertAlmostEqual(G.min_gap(a, G.box((1, 1, 1), center=(1.01, 0, 0))), 0.01, places=6)
        touching = G.box((1, 1, 1), center=(1.0, 0, 0))
        self.assertLess(G.intersection_volume(a, touching), 1e-9)

    def test_shell_and_thickness_probe(self):
        th = np.linspace(0, np.pi, 40)
        xs = np.linspace(0, 1, 20)
        P = np.stack([np.stack([np.full_like(th, x), 0.3 * np.cos(th), 0.3 * np.sin(th)], -1) for x in xs], 1)
        sh = G.shell_from_grid(P, 0.003)
        self.assertTrue(sh.check(self_intersect=True)["ok"])
        w = G.wall_thickness_samples(sh, n=200)
        self.assertAlmostEqual(float(np.nanmedian(w)), 0.003, delta=1e-4)

    def test_mirror_keeps_outward(self):
        m = G.box((1, 1, 1), center=(0, 2, 0)).mirrored_y()
        self.assertTrue(m.check()["ok"])
        self.assertLess(m.bounds()[1][1], 0)


@unittest.skipUnless(HAVE, "ucav250 dependencies missing (bash ucav250/setup_env.sh)")
class TestRegistry(unittest.TestCase):
    SPEC = {"materials": {"al": {"density": 2700.0}, "cf": {"ply_t": 0.00025, "density": 1550.0},
                          "foam": {"density": 75.0}},
            "layups": {"sandwich": {"plies": [["cf", 0, 2], ["cf", 45, 2]], "core": "foam", "core_t": 0.003}}}

    def test_layup_and_mass(self):
        lp = layup_props(self.SPEC, "sandwich")
        self.assertAlmostEqual(lp["thickness"], 0.004, places=12)
        self.assertAlmostEqual(lp["areal_mass"], 4 * 0.00025 * 1550 + 0.003 * 75, places=12)
        reg = Registry(self.SPEC)
        p = reg.add(Part(id=part_number("chassis", 1), name="plate", name_tr="levha", group="chassis",
                         material="al", process="cnc", mesh_fn=lambda: G.box((0.1, 0.1, 0.01))))
        self.assertAlmostEqual(reg.mass(p), 0.1 * 0.1 * 0.01 * 2700, places=9)

    def test_joint_chain_and_mirror(self):
        reg = Registry(self.SPEC)
        reg.add_joint(Joint("leg_R", "revolute", (0, 0.5, 0), (1, 0, 0), 0.0, math.pi / 2))
        reg.add_joint(Joint("door_R", "revolute", (0, 0.6, 0), (1, 0, 0), 0.0, 1.0, parent="leg_R"))
        p = reg.add(Part(id="YK250-LG-001-R", name="door starboard", name_tr="sağ kapak", group="gear",
                         material="al", process="cnc", side="R", joint="door_R",
                         mesh_fn=lambda: G.box((0.1, 0.1, 0.01), center=(0, 0.7, 0)),
                         fasteners=[Fastener("F1", "ISO 4762 M5x12", "screw", 0.005, 0.012, (0, 0.7, 0.005),
                                             (0, 0, -1), ("YK250-LG-001-R",))]))
        self.assertEqual([j.name for j in reg.joint_chain("door_R")], ["door_R", "leg_R"])
        self.assertEqual([q.id for q in reg.moving_parts("leg_R")], [p.id])
        V = reg.posed_vertices(p, {"leg_R": math.pi / 2})
        self.assertGreater(V[:, 2].mean(), 0.15)            # rotated up about the leg axis
        m = mirror_part(p, "YK250-LG-001-L")
        self.assertTrue(m.mesh.check()["ok"])
        self.assertLess(m.mesh.bounds()[1][1], 0)
        self.assertEqual(m.fasteners[0].joins, ("YK250-LG-001-L",))
        self.assertEqual(m.side, "L")


@unittest.skipUnless(HAVE, "ucav250 dependencies missing (bash ucav250/setup_env.sh)")
class TestChecks(unittest.TestCase):
    """Toy assembly: plate A bolted to plate B, a hinged flap that hits a stop at its upper extreme, an unattached
    part, and a bolt too close to an edge — each check must catch exactly its own defect."""

    def _reg(self):
        from shapely.geometry import box as sbox
        spec = {"materials": {"al": {"density": 2700.0, "kind": "metal"}},
                "processes": {"cnc": {"min_thickness": 0.001}},
                "layout": {"root_part": "A", "clearances": [{"name": "flap-stop", "a": "joint:flap",
                                                             "b": ["STOP"], "min_mm": 2.0,
                                                             "joints": ["flap"]}]}}
        reg = Registry(spec)
        hole = Point(0.05, 0.05).buffer(0.0028, 32)
        A = sbox(0, 0, 0.1, 0.1).difference(hole)
        outline = [np.array([[0, 0, 0], [0.1, 0, 0], [0.1, 0.1, 0], [0, 0.1, 0], [0, 0, 0]], float)]
        f_ok = Fastener("F-ok", "ISO 4762 M5x12", "screw", 0.005, 0.012, (0.05, 0.05, 0.004), (0, 0, -1), ("A", "B"))
        f_edge = Fastener("F-edge", "ISO 4762 M5x12", "screw", 0.005, 0.012, (0.006, 0.05, 0.004), (0, 0, -1),
                          ("A", "B"))
        reg.add(Part(id="A", name="a", name_tr="a", group="chassis", material="al", process="cnc", thickness=0.002,
                     mesh_fn=lambda: G.extrude(A, 0.002, origin=(0, 0, 0.002)), outline=outline,
                     fasteners=[f_ok, f_edge]))
        reg.add(Part(id="B", name="b", name_tr="b", group="chassis", material="al", process="cnc", thickness=0.002,
                     mesh_fn=lambda: G.extrude(A, 0.002), outline=outline, contacts=("A",)))
        reg.add_joint(Joint("flap", "revolute", (0.1, 0, 0.004), (0, 1, 0), -0.6, 0.6))
        reg.add(Part(id="FLAP", name="flap", name_tr="flap", group="controls", material="al", process="cnc",
                     thickness=0.002, joint="flap", parent="A",
                     mesh_fn=lambda: G.box((0.05, 0.1, 0.002), center=(0.127, 0.05, 0.005))))
        reg.add(Part(id="STOP", name="stop", name_tr="stop", group="chassis", material="al", process="cnc",
                     thickness=0.01, parent="A", mesh_fn=lambda: G.box((0.01, 0.1, 0.01), center=(0.14, 0.05, 0.03))))
        reg.add(Part(id="FLOAT", name="float", name_tr="float", group="systems", material="al", process="cnc",
                     thickness=0.01, mesh_fn=lambda: G.box((0.01, 0.01, 0.01), center=(1, 1, 1))))
        return reg

    def test_each_check_catches_its_defect(self):
        from ucav250.analysis import checks as C
        reg = self._reg()
        self.assertEqual(C.check_meshes(reg), [])
        self.assertEqual(C.check_static(reg), [])            # bolted plates only touch
        sw = C.check_swept(reg, n=9)
        self.assertTrue(sw and all(set(v["parts"]) == {"FLAP", "STOP"} for v in sw), sw)
        cl = C.check_clearances(reg)
        self.assertTrue(any(set(v["parts"]) == {"FLAP", "STOP"} for v in cl), cl)
        fe = C.check_fasteners(reg)
        self.assertEqual({v["parts"][0] for v in fe if v["check"] == "edge_distance"}, {"F-edge"})
        at = C.check_attachment(reg)
        self.assertEqual([v["parts"] for v in at], [["FLOAT"]])
        self.assertEqual(C.check_thickness(reg), [])


if __name__ == "__main__":
    unittest.main()
