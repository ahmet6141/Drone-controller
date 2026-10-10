"""ucav250 chassis producer (design/chassis.py): the built chassis is clean under every design-rule check, every part
honours the producer contract (ids, material / process / thickness, parent, step, explode, contacts), and the
framework details the chassis relies on (fused box unions, path extension, mid-plane of the clamped material in the
edge-distance check, insert bores, nominal tapped holes, fastener materials with a density, blind rivets) behave as
specified. The chassis-fix round (CH-V01..V15) adds the physical checks: every part reaches the root through fasteners
and touching bonded faces, one solid per part (declared bush / spacer sets excepted), a buildable assembly order,
straight bolt insertion, the gear / turret / assembly-path / cooling-duct envelopes, positive detail-joint margins and
the mass reconciliation. Nothing here writes to the repository (checks run with write=False)."""
from __future__ import annotations

import copy
import math
import re
import sys
import unittest
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
try:
    from ucav250.analysis import checks as K
    from ucav250.analysis import layout_check as LC
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


    # ------------------------------------------------------------------ chassis-fix round (CH-V01..V15)
    def _cache(self):
        if not hasattr(self.__class__, "_mc"):
            self.__class__._mc = K._ManCache(self.reg)
        return self.__class__._mc

    def _produced(self):
        """Parts produced by the chassis module (CH-* and the firewall-shield PR-* parts it registers)."""
        return [pid for pid, p in self.reg.parts.items() if pid.startswith(("YK250-CH-", "YK250-PR-08", "YK250-PR-09"))]

    def test_attachment_through_fasteners_and_bonded_faces(self):
        """V01 / V10: every chassis part reaches the root through fastener stacks or faces that really touch (bond line
        <= 0.3 mm); declared contacts all touch."""
        cache = self._cache()
        root = self.spec["layout"]["root_part"]
        adj = {pid: set() for pid in self.reg.parts}
        for f in self.reg.fasteners():
            for a, b in zip(f.joins, f.joins[1:]):
                adj[a].add(b)
                adj[b].add(a)
        for pid in self._produced():
            p = self.reg.parts[pid]
            for c in set(p.contacts) | ({p.parent} if p.parent else set()):
                if c in self.reg.parts and K._touching(cache, pid, c):
                    adj[pid].add(c)
                    adj[c].add(pid)
            for c in p.contacts:
                self.assertTrue(K._touching(cache, pid, c), f"{pid}: declared contact {c} does not touch")
        seen, todo = {root}, [root]
        while todo:
            x = todo.pop()
            for y in adj[x] - seen:
                seen.add(y)
                todo.append(y)
        self.assertEqual(sorted(set(self._produced()) - seen), [])

    def test_parents_carry_their_parts(self):
        """V10: a part's parent touches it or is fastened to it (frames excepted: they are located on the assembly jig,
        whose datum is the CT box), and the parent is installed no later than the part."""
        fastened = {pid: set() for pid in self.reg.parts}
        for f in self.reg.fasteners():
            for a in f.joins:
                fastened[a].update(f.joins)
        frames = {st["part"] for st in self.spec["layout"]["stations"]}
        cache = self._cache()
        for pid in self._produced():
            p = self.reg.parts[pid]
            if not p.parent:
                continue
            self.assertGreaterEqual(p.step, self.reg.parts[p.parent].step, f"{pid} before its parent {p.parent}")
            if pid in frames:
                continue
            self.assertTrue(p.parent in fastened[pid] or K._touching(cache, pid, p.parent),
                            f"{pid}: parent {p.parent} neither touches nor is fastened to it")

    def test_one_solid_per_part(self):
        """V12 / V14: each produced part is one solid (internal voids allowed: tube bores, hollow nodes), except the
        declared sets of identical loose items."""
        sets = {"YK250-CH-053-R": 4, "YK250-CH-053-L": 4, "YK250-CH-074-R": 2, "YK250-CH-074-L": 2}
        multi_ok = ("YK250-CH-090-", "YK250-PR-094")        # spacer tubes / stand-offs (one per bolt)
        cache = self._cache()
        for pid in self._produced():
            if pid.startswith(multi_ok):
                continue
            vols = [m.volume() for m in cache.man(pid).decompose()]
            n = sum(1 for v in vols if v > 1e-9)
            self.assertEqual(n, sets.get(pid, 1), f"{pid}: {n} solids {sorted(vols, reverse=True)[:5]}")

    def test_assembly_order(self):
        """V06: a fastener is installed with (not before) the last of the parts it joins."""
        for f in self.reg.fasteners():
            if not any(j.startswith(("YK250-CH-", "YK250-PR-")) for j in f.joins):
                continue
            last = max(self.reg.parts[j].step for j in f.joins)
            self.assertGreaterEqual(f.step, last, f"{f.id}: step {f.step} < {last}")

    def test_bolts_insert_straight(self):
        """V07: each chassis bolt (head d_k, washer, shank length) comes in along its axis from the head side without
        crossing any part (0.1 mm clearance round the head)."""
        dk = {3: (0.0055, 0.003), 4: (0.007, 0.004), 5: (0.0085, 0.005), 6: (0.010, 0.006), 8: (0.013, 0.008)}
        cache = self._cache()
        solid = [pid for pid in self.reg.parts if not pid.startswith("YK250-HW-")]
        bad = []
        for f in self.reg.fasteners():
            if f.kind != "bolt" or not any(j.startswith(("YK250-CH-", "YK250-PR-")) for j in f.joins):
                continue
            d_k, k = dk[round(f.d * 1000)]
            a, p = np.asarray(f.axis, float), np.asarray(f.position, float)
            w = 0.0016 if f.washer_head else 0.0
            p0, p1 = p - a * (w + 0.0003), p - a * (w + k + f.length + 0.002)
            cyl = G.cylinder(0.5 * d_k + 0.0001, p1, p0, n=24)
            lo, hi = cyl.bounds()
            man = cyl.to_manifold()
            for q in solid:
                if K._boxes_overlap((lo, hi), cache.box(q)) and (man ^ cache.man(q)).volume() > 1e-9:
                    bad.append((f.id, q))
        self.assertEqual(bad, [])

    def test_mechanism_and_path_envelopes(self):
        """V04 / V05 / V13: main / nose tyre >= tyre_to_well and legs >= harness_to_moving_parts over the retraction
        (except the parts the legs pivot in), turret growth envelope >= turret_to_bay_wall over its travel, every
        assembly path free of chassis parts except the ones it engages, the cooling-duct corridor free."""
        ctx = LC.Ctx(copy.deepcopy(self.spec))
        L = self.spec["layout"]
        cv = L["clearance_values"]
        cache = self._cache()
        produced = [pid for pid in self._produced() if pid in self.reg.parts]
        C = CH.Ctx(Registry(copy.deepcopy(self.spec)), copy.deepcopy(self.spec))

        def mesh_of(pr):
            if isinstance(pr, LC.Sphere):
                return G.sphere(pr.r, pr.c, n=32)
            if isinstance(pr, LC.Cyl):
                return G.cylinder(pr.r, pr.c - pr.h * pr.a, pr.c + pr.h * pr.a, n=40)
            if isinstance(pr, LC.Capsule):
                return G.union([G.cylinder(pr.r, pr.p0, pr.p1, n=32), G.sphere(pr.r, pr.p0, n=24),
                                G.sphere(pr.r, pr.p1, n=24)])
            if isinstance(pr, LC.Torus):
                prof = [(pr.R0 + pr.rt * math.cos(t), pr.rt * math.sin(t))
                        for t in np.linspace(0, 2 * math.pi, 25)[:-1]]
                return G.revolve(prof, n=64, axis_origin=pr.c, axis=pr.a)
            if isinstance(pr, LC.OBB):
                return G.box(2 * pr.h, pr.c, R=pr.R)
            raise TypeError(pr)

        def gaps(prims, need, allow=()):
            out = []
            for pr in prims:
                m = mesh_of(pr)
                lo, hi = m.bounds()
                man = m.to_manifold()
                for q in produced:
                    if q in allow or not K._boxes_overlap((lo, hi), cache.box(q), pad=need):
                        continue
                    g = float(man.min_gap(cache.man(q), need + 0.002))
                    if (need > 0 and g < need - 1e-6) or (need == 0 and (man ^ cache.man(q)).volume() > 1e-9):
                        out.append((q, round(g * 1000, 2)))
            return out

        J_ = {j["name"]: j for j in L["mechanisms"]["joints"]}
        bad = []
        for ang in np.linspace(0.0, float(J_["main_gear_R"]["hi"]), 13):
            g = LC.gear_prims(ctx, "main", ang, "R")
            bad += [("main tyre", ang, x) for x in gaps([g["tyre"]], float(cv["tyre_to_well"]))]
            bad += [("main leg", ang, x) for x in gaps([g["leg"]], float(cv["harness_to_moving_parts"]),
                                                      ("YK250-CH-070-R", "YK250-CH-074-R"))]
        for ang in np.linspace(0.0, float(J_["nose_gear"]["hi"]), 10):
            g = LC.gear_prims(ctx, "nose", ang)
            bad += [("nose tyre", ang, x) for x in gaps([g["tyre"]], float(cv["tyre_to_well"]))]
            bad += [("nose leg", ang, x) for x in gaps([g["leg"]], float(cv["harness_to_moving_parts"]),
                                                      tuple(f"YK250-CH-{n}-{s}" for n in ("071", "073")
                                                            for s in "RL"))]
        for st in L["mechanisms"]["sequences"]["turret_extension"]["states"]:
            bad += [("turret", st["turret_elevator"], x)
                    for x in gaps(LC.turret_prims(ctx, float(st["turret_elevator"])),
                                  float(cv["turret_to_bay_wall"]))]
        objs = LC.layout_objects(ctx)
        movable = {"mission_tray_removal": ("YK250-CH-120",), "ecu_removal": ("YK250-CH-120",)}   # the tray itself
        for pth in L["mechanisms"]["assembly_paths"]:
            prims = LC._path_prims(ctx, pth, objs)
            for side, pr in (("R", prims), ("L", [LC.mirror_prim(q) for q in prims])):
                if side == "L" and not pth.get("mirror"):
                    continue
                allow = set(movable.get(pth["name"], ()))
                for e in pth.get("engages", []):
                    try:
                        allow.add(C.ref(e, side))
                    except KeyError:
                        pass
                    if e == "ENGINE-MOUNT":
                        allow.add(L["chassis"]["engine_mount"]["part"])
                    if e == "F-FORK":                   # the fork prongs are integral with the CT box CH-001
                        allow.add(L["root_part"])
                bad += [(pth["name"], side, x) for x in gaps(pr, 0.0, allow)]
        ko = next(k for k in L["keep_outs"] if k["id"] == "KO-COOLING-DUCT")
        P = np.asarray(ko["path"], float)
        for off in ko["lateral_offsets"]:
            bad += [("cooling duct", off, x) for x in gaps(LC.polyline(P + np.array([0.0, off, 0.0]),
                                                                       float(ko["radius"])), 0.0)]
        self.assertEqual(bad, [])

    def test_detail_joint_margins(self):
        """V08: the detail fasteners that differ from the layout bolt groups have positive margins on the structures
        loads."""
        rows = CH.detail_joint_margins(self.spec)
        ids = {r["id"] for r in rows}
        for need in ("SPL-CH-AFT-SH", "SPL-CH-AFT-BR-RIB", "SPL-CH-AFT-BR-LEG", "VENTRAL-KEEL-BOLTS",
                     "KEEL-FOOT-BOLTS", "KEEL-FOOT-BR", "NODE-FOOT-BOLTS", "NODE-FOOT-BR", "UPLOCK-INSERT-M4",
                     "DORSAL-SPLICE-M4"):
            self.assertIn(need, ids)
        for r in rows:
            self.assertGreaterEqual(r["ms"], 0.0, f"{r['id']}: MS {r['ms']:.3f}")

    def test_mass_reconciliation_covers_every_part(self):
        """V03: every chassis part is booked in one spec.mass item; the rows add up to the chassis group mass."""
        rows = CH.mass_reconciliation(self.reg, self.spec)
        self.assertNotIn("unassigned", [r["item"] for r in rows])
        tot = sum(self.reg.mass(p) for p in self.reg.parts.values() if p.group == "chassis")
        self.assertAlmostEqual(sum(r["model_kg"] for r in rows), tot, delta=1e-6)
        items = {it["name"] for it in self.spec["mass"]["items"]}
        for r in rows:
            if r["budget_kg"] is not None:
                self.assertIn(r["item"], items)


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
        # V15: the alloy is read from the grade token only ("estimate", "captive", "anti-rotation" are not "Ti")
        self.assertEqual(HW.fastener_material("blind rivet, A286 / CherryMAX class (estimate) 3.2x3.5", mats),
                         "ss_304_annealed")
        self.assertEqual(HW.fastener_material("captive screw M4 (anti-rotation, estimate)", mats), "steel_4130_n")
        self.assertEqual(HW.fastener_material("NAS1956 Ti 6-4 bolt", mats), "ti_6al_4v_annealed_sheet")


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

    def test_blind_rivet(self):
        """joints.rivet: kind 'rivet', hole d + 0.1 mm in every stack part, length = grip + 0.6 d rounded up to 0.5 mm,
        blind (self-locking stem) far side; hardware.py classes an A286 rivet as stainless."""
        spec = {"materials": {"ss": {"density": 7900.0, "kind": "metal"}},
                "processes": {"sm": {"min_thickness": 0.0003}}, "layout": {"root_part": "A"}}
        reg = Registry(spec)
        for pid, z0, t in (("A", 0.0, 0.0008), ("B", -0.0004, 0.0004)):
            reg.add(Part(id=pid, name=pid, name_tr=pid, group="propulsion", material="ss", process="sm",
                         thickness=t, mesh_fn=(lambda z0=z0, t=t: G.box((0.03, 0.03, t), center=(0.0, 0.0, z0 + t / 2)))))
        f = J.rivet(reg, "R1", 0.0032, (0.0, 0.0, 0.0008), (0.0, 0.0, -1.0), [("A", 0.0008), ("B", 0.0004)],
                    spec="blind rivet, A286 class (estimate)", step=12)
        self.assertEqual(f.kind, "rivet")
        self.assertEqual(f.joins, ("A", "B"))
        self.assertAlmostEqual(f.length, math.ceil((0.0012 + 0.6 * 0.0032) * 2000) / 2000, delta=1e-12)
        self.assertIn("blind", f.nut)
        for pid in ("A", "B"):
            self.assertAlmostEqual(reg.parts[pid].holes[-1][2], 0.5 * 0.0032 + 0.5 * J.RIVET_HOLE_CLEARANCE, delta=1e-12)


if __name__ == "__main__":
    unittest.main()
