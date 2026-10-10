"""ucav250 chassis producer (design/chassis.py): the built chassis is clean under every design-rule check, every part
honours the producer contract (ids, material / process / thickness, parent, step, explode, contacts), and the
framework details the chassis relies on (fused box unions, path extension, mid-plane of the clamped material in the
edge-distance check, insert bores, nominal tapped holes, fastener materials with a density) behave as specified.
Nothing here writes to the repository (checks run with write=False)."""
from __future__ import annotations

import math
import re
import sys
import unittest
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
try:
    from ucav250.analysis import checks as K
    from ucav250.core import geom as G
    from ucav250.core.assemble import build_registry
    from ucav250.core.parts import Fastener, Part, Registry, layup_props
    from ucav250.design import chassis as CH
    from ucav250.design import fastener_catalog as FC
    from ucav250.design import hardware as HW
    from ucav250.design import joints as J
    HAVE = True
except Exception:  # pragma: no cover
    HAVE = False

ID_RE = re.compile(r"^YK250-CH-(\d{3})(-[LR])?$")


@unittest.skipUnless(HAVE, "ucav250 dependencies missing")
class TestChassisBuild(unittest.TestCase):
    """One registry for the whole class: chassis registered twice (the second call must be ignored) + hardware."""

    @classmethod
    def setUpClass(cls):
        cls.reg = build_registry(modules=["chassis", "chassis", "hardware"], strict=False)
        cls.spec = cls.reg.spec
        cls.ch = {pid: p for pid, p in cls.reg.parts.items() if pid.startswith("YK250-CH-")}
        focus = set(cls.ch)
        cls.summary = K.run_all(cls.reg, quick=False, write=False, focus=focus)

    def test_design_rule_checks_clean(self):
        bad = {k: v[:3] for k, v in self.summary["details"].items() if v}
        self.assertTrue(self.summary["ok"], f"chassis violations: {bad}")

    def test_registered_once_and_no_fastener_problems(self):
        notes = " ".join(str(n) for n in self.reg.log)
        self.assertIn("second call ignored", notes)
        self.assertNotIn("fastener problems", notes)

    def test_root_and_ids(self):
        root = self.spec["layout"]["root_part"]
        self.assertEqual(root, "YK250-CH-001")
        self.assertIn(root, self.reg.parts)
        self.assertGreater(len(self.ch), 100)
        for pid in self.ch:
            m = ID_RE.match(pid)
            self.assertIsNotNone(m, pid)
            self.assertTrue(1 <= int(m.group(1)) <= 149, pid)

    def test_part_contract(self):
        mats, procs = self.spec["materials"], self.spec["processes"]
        for pid, p in self.ch.items():
            self.assertTrue(p.layup or p.material in mats, f"{pid}: material {p.material}")
            self.assertIn(p.process, list(procs) + ["purchased"], pid)
            if not p.purchased:
                self.assertTrue(p.thickness is not None or p.layup, f"{pid}: no thickness / layup")
            self.assertTrue(p.name and p.name_tr, pid)
            self.assertIsInstance(p.step, int)
            self.assertGreaterEqual(p.step, 1, pid)
            self.assertEqual(len(tuple(p.explode)), 3, pid)
            if pid != "YK250-CH-001":
                self.assertIn(p.parent, self.reg.parts, f"{pid}: parent {p.parent}")
            for c in p.contacts:
                self.assertIn(c, self.reg.parts, f"{pid}: contact {c}")

    def test_mirror_pairs(self):
        for pid, p in self.ch.items():
            if not pid.endswith("-R"):
                continue
            q = self.reg.parts[pid[:-1] + "L"]
            lo_r, hi_r = p.base_mesh.bounds()
            lo_l, hi_l = q.base_mesh.bounds()
            np.testing.assert_allclose([lo_r[0], hi_r[0], lo_r[2], hi_r[2]], [lo_l[0], hi_l[0], lo_l[2], hi_l[2]],
                                       atol=1e-6, err_msg=pid)
            np.testing.assert_allclose([lo_r[1], hi_r[1]], [-hi_l[1], -lo_l[1]], atol=1e-6, err_msg=pid)
            self.assertAlmostEqual(p.base_mesh.volume(), q.base_mesh.volume(), delta=1e-9)

    def test_fasteners_join_existing_parts(self):
        fs = [f for f in self.reg.fasteners() if any(j.startswith("YK250-CH-") for j in f.joins)]
        self.assertGreater(len(fs), 150)
        for f in fs:
            self.assertTrue(all(j in self.reg.parts for j in f.joins), f.id)
            self.assertIsNotNone(f.grip, f.id)
            self.assertGreater(f.length, 0.0, f.id)
            self.assertIn(f"YK250-HW-{f.id}", self.reg.parts)

    def test_mass_is_computable(self):
        m = sum(self.reg.mass(p) for p in self.reg.parts.values() if p.group == "chassis")
        self.assertTrue(math.isfinite(m))
        self.assertGreater(m, 5.0)
        self.assertIsNotNone(self.reg.parts["YK250-CH-001"].mass_kg)        # CT box: bottom-up sub-volume mass
        hw = sum(self.reg.mass(p) for p in self.reg.parts.values() if p.group == "hardware")
        self.assertGreater(hw, 0.0)


@unittest.skipUnless(HAVE, "ucav250 dependencies missing")
class TestChassisHelpers(unittest.TestCase):
    def test_fuse_boxes_single_shell(self):
        boxes = [((0.0, 0.0, 0.0), (0.02, 0.01, 0.01)), ((0.02, 0.0, 0.0), (0.04, 0.01, 0.005))]
        m = CH.fuse_boxes(boxes)
        v_exp = 0.02 * 0.01 * 0.01 + 0.02 * 0.01 * 0.005
        self.assertAlmostEqual(m.volume(), v_exp, delta=1e-9)
        self.assertAlmostEqual(m.to_manifold().volume(), m.volume(), delta=1e-12)   # no touching shells
        self.assertEqual(len(m.to_manifold().decompose()), 1)

    def test_extend_path_is_linear(self):
        P = np.array([[0.0, 0.0, 0.0], [0.1, 0.02, -0.01], [0.3, 0.02, 0.0]])
        Q = CH.extend_path(P, 0.01)
        np.testing.assert_allclose(Q[0], [-0.01, -0.002, 0.001], atol=1e-12)
        np.testing.assert_allclose(Q[-1], [0.31, 0.02, 0.0005], atol=1e-12)
        np.testing.assert_allclose(Q[1], P[1])

    def test_fastener_material_has_density(self):
        mats = {"steel_4130_n": {"density": 7833.0}, "ss_304_annealed": {"density": 7916.0},
                "ti_6al_4v_annealed_sheet": {"density": 4429.0}}
        self.assertEqual(HW.fastener_material("ISO 4762 M6x14-12.9", mats), "steel_4130_n")
        self.assertEqual(HW.fastener_material("ISO 4762 M4x8-A2-70", mats), "ss_304_annealed")
        self.assertEqual(HW.fastener_material("ISO 4762 M4x18-Ti-6Al-4V", mats), "ti_6al_4v_annealed_sheet")
        self.assertEqual(HW.fastener_material("ISO 4762 M6x14-12.9", {"fastener_steel": {"density": 7900.0}}),
                         "fastener_steel")


@unittest.skipUnless(HAVE, "ucav250 dependencies missing")
class TestFrameworkDetails(unittest.TestCase):
    def test_edge_distance_uses_clamped_material(self):
        """A part with a second wall beyond the nut (J longeron, U channel): the edge distance is measured at the
        mid-plane of the clamped wall, not in the cavity between the walls."""
        plate = G.box((0.06, 0.06, 0.005), center=(0.0, 0.0, 0.0025))
        side = G.box((0.005, 0.06, 0.03), center=(0.0125, 0.0, 0.015))
        top = G.box((0.045, 0.06, 0.005), center=(-0.0075, 0.0, 0.0275))
        u = G.union([plate, side, top])
        f = Fastener("T1", "ISO 4762 M6x16-12.9", "bolt", 0.006, 0.016, (0.0, 0.0, 0.0), (0.0, 0.0, 1.0),
                     ("U", "V"), nut="ISO 7040 M6", grip=0.005)
        ed, pierced = K._edge_distance_mesh(f, u.to_manifold(), 0.2)
        self.assertTrue(pierced)
        self.assertAlmostEqual(ed, 0.030, delta=1e-4)          # plate edge, not the side wall 10 mm away

    def test_layup_props_counts_both_sandwich_faces(self):
        spec = {"materials": {"ply": {"ply_t": 0.0002, "density": 1500.0}, "core": {"density": 50.0}},
                "layups": {"s": {"plies": [["ply", 0, 2]], "inner_plies": [["ply", 45, 3]], "core": "core",
                                 "core_t": 0.005},
                           "bad": {"plies": [["ply", 0, "sized"]]}}}
        lp = layup_props(spec, "s")
        self.assertAlmostEqual(lp["thickness"], 5 * 0.0002 + 0.005, delta=1e-12)
        self.assertAlmostEqual(lp["areal_mass"], 5 * 0.0002 * 1500.0 + 0.005 * 50.0, delta=1e-9)
        with self.assertRaises(ValueError):
            layup_props(spec, "bad")

    def test_insert_bore_radius(self):
        f = Fastener("T2", "ISO 4762 M6x14-12.9", "bolt", 0.006, 0.014, (0.0, 0.0, 0.0), (0.0, 1.0, 0.0),
                     ("A", "B"), nut="potted insert M6", grip=0.0065)
        r = K._insert_bore_radius(f, "B")
        self.assertAlmostEqual(r, 0.5 * FC.INSERT[6][1] + 0.0001, delta=1e-12)
        self.assertIsNone(K._insert_bore_radius(f, "A"))

    def test_tapped_hole_is_nominal_diameter(self):
        spec = {"materials": {"al": {"density": 2800.0, "kind": "metal"}},
                "processes": {"cnc": {"min_thickness": 0.001}}, "layout": {"root_part": "A"}}
        reg = Registry(spec)
        reg.add(Part(id="A", name="a", name_tr="a", group="chassis", material="al", process="cnc", thickness=0.004,
                     mesh_fn=lambda: G.box((0.04, 0.04, 0.004), center=(0.0, 0.0, 0.002))))
        reg.add(Part(id="B", name="b", name_tr="b", group="chassis", material="al", process="cnc", thickness=0.02,
                     mesh_fn=lambda: G.box((0.04, 0.04, 0.02), center=(0.0, 0.0, -0.01)), contacts=("A",)))
        J.bolt(reg, "T3", 4, (0.0, 0.0, 0.004), (0.0, 0.0, -1.0), [("A", 0.004)], nut="tapped", tapped_part="B",
               tapped_depth=0.01, step=1)
        p0, p1, r = reg.parts["B"].holes[-1]
        self.assertAlmostEqual(r, 0.002, delta=1e-12)           # 0.5 d: the shank does not overlap the thread


if __name__ == "__main__":
    unittest.main()
