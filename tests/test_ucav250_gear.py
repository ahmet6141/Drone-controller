"""ucav250 landing-gear producer (design/gear.py): retractable tricycle gear, wheels, brakes, retraction EMAs, locks and
the gear doors with their hinges.

The gear parts are clean under every design-rule check (mesh, static, swept through the gear_retraction sequence,
layout clearances incl. tyre-to-well 12 mm and doors-to-legs 10 mm, thickness, fastener edge distance / pierce /
pitch, declared contacts, attachment graph), every part honours the producer contract, the joints are the layout's
(spec.layout.mechanisms) and the sequence is the layout's gear_retraction, the interfaces come from spec.layout and
the group mass is reported against spec.mass.budget.gear. Nothing here writes to the repository (checks run with
write=False)."""
from __future__ import annotations

import re
import sys
import unittest
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
try:
    from ucav250.analysis import checks as K
    from ucav250.core.assemble import build_registry
    from ucav250.design import gear as GR
    HAVE = True
except Exception:  # pragma: no cover
    HAVE = False

ID_RE = re.compile(r"^YK250-(LG|FC)-(\d{3})(-[LR])?$")
FORBIDDEN = ("hardpoint", "hard-point", "hard point", "pylon", "weapon", "munition", "release mechanism")
GEAR_STEPS = (20, 28, 29, 30)
GEAR_JOINTS = ("main_gear_R", "main_gear_L", "main_inner_door_R", "main_inner_door_L", "main_trunnion_door_R",
               "main_trunnion_door_L", "nose_gear", "nose_steer", "nose_door_R", "nose_door_L")


def _gear_part(pid: str) -> bool:
    m = ID_RE.match(pid)
    return bool(m) and 620 <= int(m.group(2)) <= 719


@unittest.skipUnless(HAVE, "ucav250 dependencies missing")
class TestGearBuild(unittest.TestCase):
    """One registry for the class: chassis, shell, gear registered twice (the second call is ignored), hardware."""

    @classmethod
    def setUpClass(cls):
        cls.reg = build_registry(modules=["chassis", "shell", "gear", "gear", "hardware"], strict=False)
        from ucav250.core import spec as S
        cls.B = GR.Build(build_registry(modules=["chassis"], strict=False), S.load())   # mechanism objects (math only)
        cls.B.joints()
        cls.B.main_gear()
        cls.B.main_doors()
        cls.B.nose_gear()
        cls.B.drives()
        cls.spec = cls.reg.spec
        cls.gear = {pid: p for pid, p in cls.reg.parts.items() if _gear_part(pid)}
        cls.summary = K.run_all(cls.reg, quick=False, write=False, focus=set(cls.gear))

    # ------------------------------------------------------------------ checks
    def test_design_rule_checks_clean(self):
        bad = {k: v[:3] for k, v in self.summary["details"].items() if v}
        self.assertTrue(self.summary["ok"], f"gear violations: {bad}")
        for k in ("mesh", "static", "swept", "clearance", "thickness", "fastener", "contact", "attachment"):
            self.assertIn(k, self.summary["violations"], k)

    def test_registered_once(self):
        self.assertIn("gear: already registered, second call ignored", self.reg.log)

    # ------------------------------------------------------------------ contract
    def test_ids_in_gear_range_and_mirrored(self):
        lo, hi = self.spec["layout"]["part_numbers"]["gear"]
        self.assertGreater(len(self.gear), 60)
        for pid, p in self.gear.items():
            m = ID_RE.match(pid)
            self.assertTrue(lo <= int(m.group(2)) <= hi, pid)
            if m.group(3):
                other = pid[:-1] + ("L" if pid.endswith("R") else "R")
                self.assertIn(other, self.gear, f"{pid} has no mirror")
                self.assertEqual(p.side, m.group(3)[1], pid)
            else:
                self.assertEqual(p.side, "C", pid)
            if m.group(1) == "LG":
                self.assertEqual(p.group, "gear", pid)

    def test_layout_fixed_ids(self):
        """Interface part ids fixed by spec.layout (mechanisms, actuators, equipment) exist with the right role."""
        L = self.spec["layout"]
        P = self.reg.parts
        for s in ("R", "L"):
            self.assertEqual(P[f"YK250-LG-620-{s}"].joint, f"main_gear_{s}")
            self.assertEqual(P[f"YK250-LG-622-{s}"].joint, f"main_gear_{s}")
            self.assertEqual(P[f"YK250-LG-672-{s}"].joint, f"main_gear_{s}")
            self.assertEqual(P[f"YK250-LG-670-{s}"].joint, f"main_inner_door_{s}")
            self.assertEqual(P[f"YK250-LG-673-{s}"].joint, f"main_trunnion_door_{s}")
            self.assertEqual(P[f"YK250-LG-674-{s}"].joint, f"nose_door_{s}")
            self.assertIn("DA 30", P[f"YK250-FC-676-{s}"].vendor)     # IR-02: inner-door actuator DA 22 -> DA 30
            self.assertIn(f"YK250-LG-630-{s}", P)
        self.assertEqual(P["YK250-LG-624"].joint, "nose_gear")
        self.assertEqual(P["YK250-LG-626"].joint, "nose_steer")
        self.assertEqual(P["YK250-FC-628"].joint, "nose_gear")
        self.assertEqual(P["YK250-FC-628"].group, "controls")
        self.assertIn("DA 26", P["YK250-FC-628"].vendor)
        self.assertIn("DA 22", P["YK250-LG-678"].vendor)
        acts = {a["id"]: a for a in L["systems"]["actuators"]}
        self.assertEqual(acts["ACT-NLG-EMA"]["part"], "YK250-LG-632")
        self.assertIn("YK250-LG-632", P)
        # TOST wheels: datasheet masses (components.yaml)
        self.assertAlmostEqual(P["YK250-LG-622-R"].mass_kg, 0.860 + 0.450 + 0.080, places=3)
        self.assertAlmostEqual(P["YK250-LG-626"].mass_kg, 0.365 + 0.450 + 0.080, places=3)

    def test_part_contract(self):
        mats, procs = self.spec["materials"], self.spec["processes"]
        steps = {int(s["step"]) for s in self.spec["assembly"]["steps"]}
        for pid, p in self.gear.items():
            self.assertTrue(p.name and p.name_tr, pid)
            if p.purchased:
                self.assertTrue(p.vendor, pid)
                self.assertIsNotNone(p.mass_kg, pid)
            else:
                self.assertIn(p.material, mats, pid)
                self.assertIn(p.process, procs, pid)
                self.assertTrue(p.thickness is not None or p.layup, pid)
                self.assertGreaterEqual(p.thickness, float(procs[p.process]["min_thickness"]) - 1e-9, pid)
            self.assertIn(p.step, steps, pid)
            self.assertIn(p.step, GEAR_STEPS, pid)
            self.assertEqual(len(tuple(p.explode)), 3, pid)
            self.assertGreater(float(np.linalg.norm(p.explode)), 0.0, pid)
            self.assertIn(p.parent, self.reg.parts, f"{pid}: parent {p.parent}")
            for c in p.contacts:
                self.assertIn(c, self.reg.parts, f"{pid}: contact {c}")
            txt = " ".join((p.name, p.name_tr, p.notes, p.vendor)).lower()
            for w in FORBIDDEN:
                self.assertNotIn(w, txt, pid)

    def test_fasteners_on_gear_parts(self):
        fs = [f for p in self.gear.values() for f in p.fasteners]
        self.assertGreater(len(fs), 40)
        for f in fs:
            self.assertIn(f"YK250-HW-{f.id}", self.reg.parts, f.id)
            for j in f.joins:
                self.assertIn(j, self.reg.parts, f.id)

    # ------------------------------------------------------------------ kinematics
    def test_joints_are_the_layout_joints(self):
        lj = {j["name"]: j for j in self.spec["layout"]["mechanisms"]["joints"]}
        for name in GEAR_JOINTS:
            j, s = self.reg.joints[name], lj[name]
            np.testing.assert_allclose(j.origin, s["origin"], atol=1e-9, err_msg=name)
            np.testing.assert_allclose(j.axis, np.asarray(s["axis"], float) / np.linalg.norm(s["axis"]), atol=1e-9)
            self.assertAlmostEqual(j.lo, float(s["lo"]), places=9, msg=name)
            self.assertAlmostEqual(j.hi, float(s["hi"]), places=9, msg=name)
            self.assertAlmostEqual(j.rest, float(s["rest"]), places=9, msg=name)
            self.assertEqual(j.expr, s.get("expr", ""), name)
        self.assertEqual(self.reg.joints["nose_steer"].parent, "nose_gear")

    def test_retraction_sequence(self):
        seq = self.reg.sequences["gear_retraction"]
        ls = self.spec["layout"]["mechanisms"]["sequences"]["gear_retraction"]["states"]
        self.assertEqual(len(seq), len(ls))
        for st, ref in zip(seq, ls):
            for k, v in ref.items():
                self.assertAlmostEqual(st[k], float(v), places=6)
        for k, v in seq[0].items():                   # control 0 = rest pose (gear down)
            self.assertAlmostEqual(v, self.reg.joints[k].rest, places=4, msg=k)
        up = seq[-1]
        self.assertAlmostEqual(up["main_gear_R"], self.reg.joints["main_gear_R"].hi, places=5)
        self.assertAlmostEqual(up["nose_gear"], self.reg.joints["nose_gear"].hi, places=5)

    def test_stowed_wheels_at_layout_positions(self):
        """The wheel centres reach landing_gear.*.retraction.stowed_wheel_center when the gear is up."""
        LG = self.spec["landing_gear"]
        up = self.reg.sequences["gear_retraction"][-1]
        for pid, key in (("YK250-LG-622-R", "main"), ("YK250-LG-626", "nose")):
            p = self.reg.parts[pid]
            c = np.asarray(LG[key]["axle_static"], float)[None]
            for j in self.reg.joint_chain(p.joint):
                c = j.apply(c, up.get(j.name, j.rest))
            np.testing.assert_allclose(c[0], LG[key]["retraction"]["stowed_wheel_center"], atol=0.001, err_msg=pid)

    # ------------------------------------------------------------------ gear detail round mechanisms (GD-01..12)
    MECHS = (  # crank joint, link joint, output joint, link-end pin part, output pin part, toggle at gear_up
        ("main_idoor_crank_R", "main_idoor_link_R", "main_inner_door_R", 0.0),
        ("main_idoor_crank_L", "main_idoor_link_L", "main_inner_door_L", 0.0),
        ("main_tdoor_crank_R", "main_tdoor_link_R", "main_trunnion_door_R", 1.0),
        ("main_tdoor_crank_L", "main_tdoor_link_L", "main_trunnion_door_L", 1.0),
        ("nose_drag_crank", "nose_drag_link", "nose_gear", 0.0),
        ("nose_ddrive_crank", "nose_ddrive_link", "nose_door_L", 1.0))

    def _pose(self, P, joint, st):
        V = np.asarray(P, float)[None]
        for j in self.reg.joint_chain(joint):
            V = j.apply(V, st.get(j.name, j.rest))
        return V[0]

    def _pin(self, link_joint):
        """Rest-pose output pin of a crank / link chain: the link-end eye of the link part."""
        lj = self.reg.joints[link_joint]
        part = next(p for p in self.gear.values() if p.joint == link_joint)
        f = next(f for f in part.fasteners if f.id.endswith("-P2"))
        return f.position + 0.5 * f.length * f.axis, lj

    def test_mechanism_joints_follow_the_sequence(self):
        """Every detail-round joint has an expression equal to its gear_retraction values at every sample and is set
        in every state (coupled: swept only along the sequence); all are 0 in the rest pose."""
        seq = self.reg.sequences["gear_retraction"]
        gs = [float(v) for v in self.spec["layout"]["mechanisms"]["sequences"]["gear_retraction"]["values"]]
        names = [n for n in self.reg.joints if n.startswith(("main_idoor_", "main_tdoor_", "main_downlock_",
                                                              "nose_drag_", "nose_ddrive_", "nose_dbar", "nose_dlink_"))]
        self.assertEqual(len(names), 17)
        for n in names:
            j = self.reg.joints[n]
            self.assertTrue(j.expr, n)
            self.assertEqual(j.rest, 0.0, n)
            for g, st in zip(gs, seq):
                self.assertIn(n, st, n)
                self.assertAlmostEqual(GR.eval_pl_expr(j.expr, g), st[n], places=5, msg=f"{n} at {g}")
                self.assertTrue(j.lo - 1e-9 <= st[n] <= j.hi + 1e-9, n)
            moving = [p.id for p in self.reg.moving_parts(n)]
            self.assertTrue(moving, n)

    def test_linkages_close_and_lock_over_centre(self):
        """Closure: the link-end pin posed through crank + link coincides with the horn pin posed by the output joint
        in every sequence state (0.05 mm); travel and transmission angle; crank and link collinear (toggle =
        over-centre lock) in the locked pose."""
        seq = self.reg.sequences["gear_retraction"]
        gs = [float(v) for v in self.spec["layout"]["mechanisms"]["sequences"]["gear_retraction"]["values"]]
        for cj, lj, oj, g_lock in self.MECHS:
            q, _j = self._pin(lj)
            C = self.reg.joints[cj].origin
            K = self.reg.joints[lj].origin
            for st in seq:
                a, b = self._pose(q, lj, st), self._pose(q, oj, st)
                self.assertLess(float(np.linalg.norm(a - b)), 5e-5, f"{cj}: chain does not close")
            k = gs.index(g_lock)
            st = seq[k]
            Cp, Kp, qp = C, self._pose(K, cj, st), self._pose(q, lj, st)
            u = self.reg.joints[cj].axis
            c = np.cross(Kp - Cp, qp - Kp) @ u / (np.linalg.norm(Kp - Cp) * np.linalg.norm(qp - Kp))
            self.assertLess(abs(c), 1e-3, f"{cj}: not in toggle in the locked pose")
            travel = max(abs(s[cj]) for s in seq)
            self.assertTrue(np.radians(100) < travel < np.radians(170), f"{cj}: crank travel {np.degrees(travel):.0f}")
        for mech in (self.B.ID.m, self.B.TD.m, self.B.NG.m, self.B.NDD.m):
            self.assertGreater(mech.mu_min(), 30.0)

    def test_inner_door_drive_travel_matches_layout(self):
        """Integration round (IR-02): the door-work check (structures G-DOOR-DRIVE: DA 30 rated torque x crank travel)
        uses the layout crank travel; it must be the travel of the built mechanism (within 1 deg), and the actuator is
        the DA 30."""
        seq = self.reg.sequences["gear_retraction"]
        travel = max(abs(s["main_idoor_crank_R"]) for s in seq)
        eq = next(e for e in self.spec["layout"]["systems"]["equipment"] if e["id"] == "EQ-DOORACT")
        self.assertAlmostEqual(float(np.degrees(travel)), float(eq["shaft"]["crank_travel_deg"]), delta=1.0)
        self.assertIn("DA 30", self.reg.parts["YK250-FC-676-R"].vendor)

    def test_nose_door_coupling_mirrors_the_doors(self):
        """Both coupling links meet at the slide-bar pin in every state; the doors move as mirror images."""
        seq = self.reg.sequences["gear_retraction"]
        D = self.B.NDD
        for st in seq:
            self.assertAlmostEqual(st["nose_door_R"], st["nose_door_L"], places=9)
            r = D.cpl(st["nose_door_R"])
            self.assertAlmostEqual(float(np.linalg.norm(r["qR"] - r["B"])), D.lc, places=9)
            self.assertAlmostEqual(float(np.linalg.norm(r["qL"] - r["B"])), D.lc, places=9)
            self.assertGreater(r["mu"], 35.0)
        for sd in ("R", "L"):
            hp = next(f for f in self.reg.parts[f"YK250-LG-699-{sd}"].fasteners if f.id.endswith("-P1"))
            q = hp.position + 0.5 * hp.length * hp.axis
            for st in seq:
                a = self._pose(q, f"nose_dlink_{sd}", st)
                b = self._pose(q, f"nose_door_{sd}", st)
                self.assertLess(float(np.linalg.norm(a - b)), 5e-5, sd)

    def test_main_downlock_withdrawn_before_the_leg_moves(self):
        """The lock bolt is engaged (11.5 mm in the aft yoke arm) with the gear down and clear of the arm in every
        state in which the leg has left the down position."""
        M = self.B.M
        seq = self.reg.sequences["gear_retraction"]
        bolt = self.reg.parts["YK250-LG-641-R"].mesh.V
        x_tip = float(bolt[:, 0].min())
        self.assertGreater(M.arm_a[1] - x_tip, 0.010)
        for st in seq:
            if st["main_gear_R"] > 1e-9:
                self.assertGreater(x_tip + st["main_downlock_R"] - M.arm_a[1], 0.002)
        self.assertEqual(seq[0]["main_downlock_R"], 0.0)

    # ------------------------------------------------------------------ mass
    def test_group_mass_reported(self):
        m = sum(self.reg.mass(p) for p in self.gear.values() if p.group == "gear")
        b = self.spec["mass"]["budget"]["gear"]
        self.assertGreater(m, 0.5 * float(b["target_kg"]))
        self.assertLess(m, float(b["target_kg"]) + float(b["tol_kg"]))
        print(f"\ngear group mass {m:.2f} kg vs budget {b['target_kg']:.2f} +- {b['tol_kg']:.2f} kg")


@unittest.skipUnless(HAVE, "ucav250 dependencies missing")
class TestGearHelpers(unittest.TestCase):
    def test_phi_lower_round_trip(self):
        from ucav250.core.spec import load
        from ucav250.design import oml as O
        fus = O.fuselage_from_spec(load())
        for x, y in ((2.9759, 0.2), (0.8, 0.02), (2.9, 0.39)):
            ph = GR.phi_lower(fus, x, y)
            p = fus.point(x, ph)
            self.assertAlmostEqual(float(p[1]), y, places=6)
            self.assertLess(float(p[2]), 0.0)


if __name__ == "__main__":
    unittest.main()
