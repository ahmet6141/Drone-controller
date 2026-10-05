"""ucav250 hardware producer: bolted plates through ISO 273 clearance holes -> no interference, fully attached."""
from __future__ import annotations

import sys
import unittest
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
try:
    from shapely.geometry import Point
    from shapely.geometry import box as sbox

    from ucav250.analysis import checks as C
    from ucav250.core import geom as G
    from ucav250.core.parts import Fastener, Part, Registry
    from ucav250.design import fastener_catalog as FC
    from ucav250.design import hardware as H
    HAVE = True
except Exception:  # pragma: no cover
    HAVE = False


@unittest.skipUnless(HAVE, "ucav250 dependencies missing")
class TestHardware(unittest.TestCase):
    def test_bolted_joint_clean(self):
        spec = {"materials": {"al": {"density": 2700.0, "kind": "metal"}, "fastener_steel": {"density": 7900.0}},
                "processes": {"cnc": {"min_thickness": 0.001}}, "layout": {"root_part": "A"}}
        reg = Registry(spec)
        holes = [Point(x, 0.025).buffer(FC.clearance(6) / 2, 48) for x in (0.02, 0.06)]
        plate = sbox(0, 0, 0.08, 0.05).difference(holes[0]).difference(holes[1])
        outline = [np.array([[0, 0, 0], [0.08, 0, 0], [0.08, 0.05, 0], [0, 0.05, 0], [0, 0, 0]], float)]
        fs = [Fastener(f"F{i}", "ISO 4762 M6x20-8.8", "bolt", 0.006, 0.020, (x, 0.025, 0.010), (0, 0, -1), ("A", "B"),
                       nut="ISO 7040 M6", grip=0.010, torque_nm=10.0, step=2) for i, x in enumerate((0.02, 0.06))]
        reg.add(Part(id="A", name="a", name_tr="a", group="chassis", material="al", process="cnc", thickness=0.005,
                     mesh_fn=lambda: G.extrude(plate, 0.005, origin=(0, 0, 0.005)), outline=outline, fasteners=fs))
        reg.add(Part(id="B", name="b", name_tr="b", group="chassis", material="al", process="cnc", thickness=0.005,
                     mesh_fn=lambda: G.extrude(plate, 0.005), outline=outline, contacts=("A",)))
        H.register(reg, spec)
        self.assertEqual(len(reg.by_group("hardware")), 2)
        self.assertEqual(C.check_meshes(reg), [])
        self.assertEqual(C.check_static(reg), [])
        self.assertEqual(C.check_attachment(reg), [])
        self.assertEqual([v for v in C.check_fasteners(reg) if v["check"] == "edge_distance"], [])
        hw = reg.parts["YK250-HW-F0"]
        self.assertAlmostEqual(reg.mass(hw), 0.0, delta=0.02)           # a few grams
        self.assertGreater(reg.mass(hw), 0.003)


if __name__ == "__main__":
    unittest.main()
