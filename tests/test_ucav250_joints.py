"""ucav250 joint helpers: holes cut through every clamped part, standard lengths, hardware fits without
interference, geometry-based edge distance and pierce checks, mirrored holes."""
from __future__ import annotations

import sys
import unittest
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from ucav250.analysis import checks as C  # noqa: E402
from ucav250.core import geom as G  # noqa: E402
from ucav250.core.parts import Part, Registry, mirror_part  # noqa: E402
from ucav250.design import hardware as H  # noqa: E402
from ucav250.design import joints as J  # noqa: E402

SPEC = {"materials": {"al": {"density": 2700.0, "kind": "metal"}, "cfrp": {"density": 1550.0, "kind": "composite"},
                      "fastener_steel": {"density": 7900.0}},
        "processes": {"cnc": {"min_thickness": 0.001}}, "layout": {"root_part": "A"}}


def plates(reg, ta=0.004, tb=0.005, mat_b="al"):
    reg.add(Part(id="A", name="a", name_tr="a", group="chassis", material="al", process="cnc", thickness=ta,
                 mesh_fn=lambda: G.box((0.10, 0.06, ta), (0.05, 0.0, tb + ta / 2))))
    reg.add(Part(id="B", name="b", name_tr="b", group="chassis", material=mat_b, process="cnc", thickness=tb,
                 mesh_fn=lambda: G.box((0.10, 0.06, tb), (0.05, 0.0, tb / 2)), contacts=("A",)))


class TestJoints(unittest.TestCase):
    def test_lengths(self):
        self.assertAlmostEqual(J.std_length(0.0151), 0.016)
        self.assertAlmostEqual(J.std_length_floor(0.0151), 0.014)

    def test_bolt_through_nyloc(self):
        reg = Registry(SPEC)
        plates(reg)
        f = J.bolt_through(reg, "F1", 5, (0.03, 0.0, 0.0), (0, 0, -1), ["A", "B"], washer_head=True)
        self.assertAlmostEqual(f.grip, 0.009, delta=1e-5)
        self.assertAlmostEqual(f.position[2], 0.009, delta=1e-5)          # head on top face of A
        self.assertTrue(f.spec.startswith("ISO 4762 M5x"))
        nominal = float(f.spec.split("x")[1].split("-")[0]) / 1000
        self.assertGreaterEqual(nominal, 0.009 + 0.001 + 0.001 + 0.005 + 0.0008 - 1e-9)
        J.bolt_through(reg, "F2", 5, (0.07, 0.0, 0.0), (0, 0, -1), ["A", "B"])
        self.assertEqual(len(reg.parts["A"].holes), 2)
        H.register(reg, SPEC)
        self.assertEqual(C.check_meshes(reg), [])
        self.assertEqual(C.check_static(reg), [])
        self.assertEqual(C.check_attachment(reg), [])
        self.assertEqual(C.check_fasteners(reg), [])

    def test_edge_distance_from_mesh(self):
        reg = Registry(SPEC)
        plates(reg, mat_b="al")
        J.bolt_through(reg, "F1", 5, (0.006, 0.0, 0.0), (0, 0, -1), ["A", "B"])      # 6 mm from the edge < 2 D
        v = [x for x in C.check_fasteners(reg) if x["check"] == "edge_distance"]
        self.assertEqual(len(v), 2)
        self.assertAlmostEqual(v[0]["value"], 6.0, delta=0.3)

    def test_composite_rule_and_miss(self):
        reg = Registry(SPEC)
        plates(reg, mat_b="cfrp")
        reg.parts["B"].layup = None
        J.bolt_through(reg, "F1", 5, (0.011, 0.0, 0.0), (0, 0, -1), ["A", "B"])      # 11 mm: ok metal, < 2.5 D CFRP
        v = [x for x in C.check_fasteners(reg) if x["check"] == "edge_distance"]
        self.assertEqual([x["parts"][1] for x in v], ["B"])
        # a fastener record whose line misses a joined part is reported
        reg.add(Part(id="X", name="x", name_tr="x", group="chassis", material="al", process="cnc", thickness=0.003,
                     mesh_fn=lambda: G.box((0.02, 0.02, 0.003), (0.5, 0.5, 0.5))))
        reg.parts["A"].fasteners[0].joins = ("A", "B", "X")
        self.assertTrue(any(x["check"] == "fastener_miss" for x in C.check_fasteners(reg)))

    def test_insert_and_nutplate(self):
        reg = Registry(SPEC)
        reg.add(Part(id="A", name="a", name_tr="a", group="chassis", material="al", process="cnc", thickness=0.002,
                     mesh_fn=lambda: G.box((0.06, 0.06, 0.002), (0.0, 0.0, 0.021))))
        reg.add(Part(id="P", name="p", name_tr="p", group="chassis", material="al", process="cnc", thickness=0.020,
                     mesh_fn=lambda: G.box((0.08, 0.08, 0.020), (0.0, 0.0, 0.010)), contacts=("A",)))
        f = J.bolt(reg, "F1", 5, (0.0, 0.0, 0.022), (0, 0, -1), [("A", 0.002)], nut="insert", insert_part="P")
        self.assertIn("P", f.joins)
        reg.add(Part(id="S", name="s", name_tr="s", group="shell", material="al", process="cnc", thickness=0.0015,
                     mesh_fn=lambda: G.box((0.05, 0.05, 0.0015), (0.2, 0.0, 0.00075))))
        reg.add(Part(id="T", name="t", name_tr="t", group="chassis", material="al", process="cnc", thickness=0.002,
                     mesh_fn=lambda: G.box((0.05, 0.05, 0.002), (0.2, 0.0, -0.001)), contacts=("S", "P")))
        J.bolt(reg, "F2", 4, (0.2, 0.0, 0.0015), (0, 0, -1), [("S", 0.0015), ("T", 0.002)], nut="nutplate",
               head="ISO 7380", grade="A2-70")
        J.quarter_turn(reg, "F3", (0.215, 0.015, 0.0015), (0, 0, -1), "S", 0.0015, "T", 0.002)
        H.register(reg, SPEC)
        self.assertEqual(C.check_meshes(reg), [])
        self.assertEqual(C.check_static(reg), [])

    def test_mirrored_holes(self):
        reg = Registry(SPEC)
        reg.add(Part(id="R", name="starboard bracket", name_tr="sağ braket", group="chassis", material="al",
                     process="cnc", thickness=0.004, side="R",
                     mesh_fn=lambda: G.box((0.04, 0.04, 0.004), (0.0, 0.3, 0.002))))
        J.bolt(reg, "F1", 5, (0.0, 0.3, 0.004), (0, 0, -1), [("R", 0.004)])
        L = mirror_part(reg.parts["R"], "L")
        self.assertEqual(len(L.holes), 1)
        self.assertAlmostEqual(L.holes[0][0][1], -0.3)
        self.assertAlmostEqual(L.mesh.volume(), reg.parts["R"].mesh.volume(), places=10)
        self.assertLess(L.mesh.volume(), L.base_mesh.volume())
        self.assertEqual(L.fasteners[0].grip, reg.parts["R"].fasteners[0].grip)


if __name__ == "__main__":
    unittest.main()
