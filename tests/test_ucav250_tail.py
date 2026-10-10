"""ucav250 tail producer (design/tail.py): twin canted fins with rudders, fixed stabilator root stubs, all-moving
stabilators on Ti stub spindles, the ventral fin with its replaceable bumper skid and the rudder / stabilator drives.

The tail parts are clean under every design-rule check (mesh, static, swept, clearance, thickness, fastener edge
distance / pierce / pitch, declared contacts, attachment graph), every part honours the producer contract, the
interfaces come from spec.layout (chassis fittings, mechanism joints, actuators), the four-bar linkages close in every
registered sequence state within the datasheet servo travel, the stabilator root gap holds over -20..+15 deg, and the
group masses are reported against spec.mass.budget. Nothing here writes to the repository (checks run with
write=False)."""
from __future__ import annotations

import math
import re
import sys
import unittest
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
try:
    import yaml

    from ucav250.analysis import checks as K
    from ucav250.core import geom as G
    from ucav250.core.assemble import build_registry
    from ucav250.design import tail as T
    HAVE = True
except Exception:  # pragma: no cover
    HAVE = False

ID_RE = re.compile(r"^YK250-(TL|FC)-(\d{3})(-[LR])?$")
FORBIDDEN = ("hardpoint", "hard-point", "hard point", "pylon", "weapon", "munition", "release mechanism")
TAIL_STEPS = (31, 32, 33, 34)


def _tail_part(pid: str) -> bool:
    m = ID_RE.match(pid)
    return bool(m) and 250 <= int(m.group(2)) <= 349


@unittest.skipUnless(HAVE, "ucav250 dependencies missing")
class TestTailBuild(unittest.TestCase):
    """One registry for the class: chassis, tail registered twice (the second call is ignored), hardware."""

    @classmethod
    def setUpClass(cls):
        cls.reg = build_registry(modules=["chassis", "tail", "tail", "hardware"], strict=False)
        cls.spec = cls.reg.spec
        cls.tail = {pid: p for pid, p in cls.reg.parts.items() if _tail_part(pid)}
        cls.summary = K.run_all(cls.reg, quick=False, write=False, focus=set(cls.tail))
        cls.C = T.Ctx(cls.reg, cls.spec)          # geometry helpers only (no registration)

    # ------------------------------------------------------------------ checks
    def test_design_rule_checks_clean(self):
        bad = {k: v[:3] for k, v in self.summary["details"].items() if v}
        self.assertTrue(self.summary["ok"], f"tail violations: {bad}")
        for k in ("mesh", "static", "swept", "clearance", "thickness", "fastener", "contact", "attachment"):
            self.assertIn(k, self.summary["violations"], k)

    def test_registered_once(self):
        self.assertIn("tail: already registered, second call ignored", self.reg.log)

    # ------------------------------------------------------------------ contract
    def test_ids_in_tail_range_and_mirrored(self):
        lo, hi = self.spec["layout"]["part_numbers"]["tail"]
        self.assertGreater(len(self.tail), 110)
        for pid, p in self.tail.items():
            m = ID_RE.match(pid)
            n = int(m.group(2))
            self.assertTrue(lo <= n <= hi, pid)
            self.assertEqual(p.group, "tail" if m.group(1) == "TL" else "controls", pid)
            if m.group(1) == "FC":
                self.assertGreaterEqual(n, 300, pid)            # FC sub-range of the tail module
            if m.group(3):
                other = pid[:-1] + ("L" if pid.endswith("R") else "R")
                self.assertIn(other, self.tail, f"{pid} has no mirror")
                self.assertEqual(p.side, m.group(3)[1], pid)
            else:                                               # centre line: ventral fin and its skid only
                self.assertEqual(p.side, "C", pid)
                self.assertTrue(253 < n < 290, pid)

    def test_layout_fixed_ids(self):
        """Interface part ids fixed by spec.layout (mechanisms, actuators) exist with the right role."""
        L = self.spec["layout"]
        for j in L["mechanisms"]["joints"]:
            if j["name"] in ("rudder_R", "rudder_L"):
                pid = j["moves"]
                self.assertIn(pid, self.tail)
                self.assertEqual(self.tail[pid].joint, j["name"])
        acts = {a["id"]: a for a in L["systems"]["actuators"]}
        for s in ("R", "L"):
            self.assertIn("DA 26", self.tail[acts["ACT-RUDDER"]["part"] + f"-{s}"].vendor)
        eq = {e["id"]: e for e in L["systems"]["equipment"]}
        for s in ("R", "L"):
            self.assertIn("DA 30", self.tail[eq["EQ-STABACT"]["part"] + f"-{s}"].vendor)
        for pid in ("YK250-TL-250-R", "YK250-TL-252-R", "YK250-TL-254", "YK250-TL-256-R", "YK250-TL-258-R",
                    "YK250-TL-260", "YK250-TL-301-R", "YK250-FC-300-R"):
            self.assertIn(pid, self.tail)

    def test_part_contract(self):
        mats, procs = self.spec["materials"], self.spec["processes"]
        steps = {int(s["step"]) for s in self.spec["assembly"]["steps"]}
        for pid, p in self.tail.items():
            self.assertTrue(p.name and p.name_tr, pid)
            if p.purchased:
                self.assertTrue(p.vendor, pid)
                self.assertTrue(p.mass_kg or p.material in mats, pid)
            else:
                self.assertIn(p.material, mats, pid)
                self.assertIn(p.process, procs, pid)
                self.assertTrue(p.thickness is not None or p.layup, pid)
                t = p.thickness if p.thickness is not None else 1.0
                self.assertGreaterEqual(t, float(procs[p.process]["min_thickness"]) - 1e-9, pid)
            self.assertIn(p.step, steps, pid)
            self.assertIn(p.step, TAIL_STEPS, pid)
            self.assertEqual(len(tuple(p.explode)), 3, pid)
            self.assertGreater(float(np.linalg.norm(p.explode)), 0.0, pid)
            self.assertIn(p.parent, self.reg.parts, f"{pid}: parent {p.parent}")
            for c in p.contacts:
                self.assertIn(c, self.reg.parts, f"{pid}: contact {c}")

    def test_no_weapon_wording(self):
        for pid, p in self.tail.items():
            text = " ".join([p.name, p.name_tr, p.notes, p.vendor]).lower()
            for w in FORBIDDEN:
                self.assertNotIn(w, text, pid)
        src = Path(T.__file__).read_text(encoding="utf-8").lower()
        for w in FORBIDDEN:
            self.assertNotIn(w, src)

    def test_meshes_closed_and_mirror_equal(self):
        for pid, p in self.tail.items():
            self.assertGreater(p.mesh.volume(), 0.0, pid)
            if pid.endswith("-R"):
                vr, vl = p.mesh.volume(), self.tail[pid[:-1] + "L"].mesh.volume()
                self.assertAlmostEqual(vr, vl, delta=1e-3 * vr + 1e-12, msg=pid)

    # ------------------------------------------------------------------ interfaces from spec.layout
    def test_root_fittings_on_layout_chassis_fittings(self):
        fit = {f["id"]: f for f in self.spec["layout"]["chassis"]["fittings"]}
        pairs = (("YK250-TL-266-R", "F-FIN-FRONT"), ("YK250-TL-267-R", "F-FW-CORNER"),
                 ("YK250-TL-269-R", "F-SPINDLE-NODE"), ("YK250-TL-265-R", "F-STUB-FRONT"),
                 ("YK250-TL-284", "F-VENTRAL-1"), ("YK250-TL-285", "F-VENTRAL-2"), ("YK250-TL-286", "F-VENTRAL-3"))
        fs = self.reg.fasteners()
        for pid, fid in pairs:
            ch = fit[fid]["part"] + ("" if pid[-2] != "-" else pid[-2:])
            self.assertEqual(self.tail[pid].parent, ch, pid)
            joined = [f for f in fs if pid in f.joins and ch in f.joins]
            self.assertGreaterEqual(len(joined), 1 if fid.startswith("F-VENTRAL") else 2, f"{pid} -> {ch}")
            self.assertTrue(all("12.9" in f.spec for f in joined), pid)

    def test_stub_front_lug_in_clevis_slot(self):
        """The stub front-spar fitting lug lies between the CH-098 ears (0.1 mm each side) and seats on the slot
        floor; its two bolts keep 2 D to the slot floor and to the ear edge (layout F-STUB-FRONT box)."""
        f = self.spec["layout"]["chassis"]["fittings"]
        box = np.asarray([x for x in f if x["id"] == "F-STUB-FRONT"][0]["box"], float)
        lo, hi = self.tail["YK250-TL-265-R"].mesh.bounds()
        self.assertAlmostEqual(lo[0], box[0][0] + T.SF_EAR + T.BOND, delta=1e-6)
        self.assertAlmostEqual(hi[0], box[1][0] - T.SF_EAR - T.BOND, delta=1e-6)
        bolts = [x for x in self.reg.fasteners() if x.id.startswith("YK250-TL-265-R-B") and "YK250-CH-098-R" in x.joins]
        self.assertEqual(len(bolts), 2)
        for b in bolts:
            self.assertGreaterEqual(b.position[1] - lo[1], 2 * b.d - 1e-6)
            self.assertGreaterEqual(box[1][1] - b.position[1], 2 * b.d - 1e-6)

    def test_mechanism_joints_match_layout(self):
        lj = {j["name"]: j for j in self.spec["layout"]["mechanisms"]["joints"]}
        for name in ("rudder_R", "rudder_L", "stabilator_R", "stabilator_L"):
            j, l = self.reg.joints[name], lj[name]
            self.assertTrue(np.allclose(j.origin, l["origin"]))
            self.assertTrue(np.allclose(j.axis, np.asarray(l["axis"]) / np.linalg.norm(l["axis"])))
            self.assertAlmostEqual(j.lo, l["lo"])
            self.assertAlmostEqual(j.hi, l["hi"])
            self.assertAlmostEqual(j.rest, 0.0)
            self.assertEqual(j.prop, l["prop"])
        rng = self.spec["tail"]["surfaces"]["stabilator"]["controls"]["range_deg"]
        self.assertAlmostEqual(math.degrees(self.reg.joints["stabilator_R"].lo), rng[0], places=3)
        self.assertAlmostEqual(math.degrees(self.reg.joints["stabilator_R"].hi), rng[1], places=3)

    def test_purchased_data_from_components(self):
        root = Path(T.__file__).resolve().parents[1]
        comp = yaml.safe_load((root / "data" / "research" / "components.yaml").read_text(encoding="utf-8"))
        items = {}

        def walk(o):
            if isinstance(o, dict):
                if "id" in o and "mass_kg" in o:
                    items[o["id"]] = o
                for v in o.values():
                    walk(v)
            elif isinstance(o, list):
                for v in o:
                    walk(v)
        walk(comp)
        d26, d30 = items["volz_da26"], items["volz_da30"]
        self.assertAlmostEqual(T.DA26["mass"], d26["mass_kg"]["value"])
        self.assertTrue(np.allclose([T.DA26["L"], T.DA26["H"], T.DA26["W"]], d26["case_dimensions_m"]["value"]))
        self.assertTrue(np.allclose(T.DA26["holes"], d26["mounting"]["flange_hole_pattern_m"]["value"]))
        self.assertAlmostEqual(T.DA26["shaft_edge"], d26["mounting"]["output_axis_from_case_edge_m"]["value"])
        self.assertAlmostEqual(T.DA30["mass"], d30["mass_kg"]["value"])
        self.assertTrue(np.allclose([T.DA30["L"], T.DA30["H"], T.DA30["W"]], d30["envelope_dimensions_m"]["value"]))
        self.assertTrue(np.allclose(T.DA30["holes"], d30["mounting"]["flange_hole_pattern_m"]["value"]))
        self.assertAlmostEqual(T.DA30["axis_from_holes"], d30["mounting"]["output_axis_offset_m"]["value"])
        for s in ("R", "L"):
            self.assertAlmostEqual(self.tail[f"YK250-FC-303-{s}"].mass_kg, d26["mass_kg"]["value"])
            self.assertAlmostEqual(self.tail[f"YK250-FC-302-{s}"].mass_kg, d30["mass_kg"]["value"])

    # ------------------------------------------------------------------ mechanisms
    def _ball_centres(self):
        f = T.FinGeo(self.C)
        sd = T.StabDrive(self.C, T.StabGeo(self.C))
        return {"rudder": (f.linkage().B0, "rudder_{s}", "rudder_rod2_{s}"),
                "stabilator": (sd.B0, "stabilator_{s}", "stab_rod_{s}")}

    def test_linkages_close_in_every_sequence_state(self):
        """In every registered state the pushrod's horn-end socket (posed through servo arm -> rod joints) stays on the
        horn ball (posed through the surface joint): the four-bar closes."""
        for w, (B0, js, jr) in self._ball_centres().items():
            for s, M in (("R", np.eye(3)), ("L", np.diag([1.0, -1.0, 1.0]))):
                seq = self.reg.sequences[f"{w}_{s}"]
                self.assertGreaterEqual(len(seq), 9)
                b = (M @ B0)[None]
                for st in seq:
                    vh = b.copy()
                    for j in self.reg.joint_chain(js.format(s=s)):
                        vh = j.apply(vh, st.get(j.name, j.rest))
                    vr = b.copy()
                    for j in self.reg.joint_chain(jr.format(s=s)):
                        vr = j.apply(vr, st.get(j.name, j.rest))
                    self.assertLess(float(np.linalg.norm(vh - vr)), 5e-5, f"{w}_{s} {st}")

    def test_sequences_cover_the_layout_range(self):
        for w in ("rudder", "stabilator"):
            for s in ("R", "L"):
                j = self.reg.joints[f"{w}_{s}"]
                vals = [st[f"{w}_{s}"] for st in self.reg.sequences[f"{w}_{s}"]]
                self.assertAlmostEqual(min(vals), j.lo, places=6)
                self.assertAlmostEqual(max(vals), j.hi, places=6)
                self.assertIn(0.0, [round(v, 12) for v in vals])

    def test_servo_travel_and_ratio(self):
        """Servo arm angles stay inside the ordered +-85 deg travel; neutral linkage ratios as specified."""
        T_ = self.spec["tail"]["surfaces"]
        f = T.FinGeo(self.C)
        lk = f.linkage()
        th = [st["rudder_arm_R"] for st in self.reg.sequences["rudder_R"]]
        self.assertLessEqual(math.degrees(max(abs(min(th)), abs(max(th)))), 85.0)
        r = abs(lk.ratio(0.0, 0.0))
        self.assertAlmostEqual(r, T_["fin"]["controls"]["rudder"]["linkage"]["arm_ratio"], delta=0.1)
        sd = T.StabDrive(self.C, T.StabGeo(self.C))
        th = [st["stab_arm_R"] for st in self.reg.sequences["stabilator_R"]]
        self.assertLessEqual(math.degrees(max(abs(min(th)), abs(max(th)))), 85.0)
        r = abs(sd.lk.ratio(0.0, 0.0))
        self.assertAlmostEqual(r, T_["stabilator"]["controls"]["linkage_ratio"], delta=0.05)

    def test_coupled_joint_expressions_follow_the_tables(self):
        """The Blender expressions (polynomials in the control property) reproduce the exact four-bar values."""
        for w, coupled in (("rudder", ("rudder_arm", "rudder_rod1", "rudder_rod2")), ("stabilator", ("stab_arm",
                                                                                                 "stab_rod"))):
            for s in ("R", "L"):
                surf = self.reg.joints[f"{w}_{s}"]
                for st in self.reg.sequences[f"{w}_{s}"]:
                    deg = st[f"{w}_{s}"] / surf.scale
                    for c in coupled:
                        j = self.reg.joints[f"{c}_{s}"]
                        val = eval(j.expr, {"__builtins__": {}}, {surf.prop: deg})
                        self.assertAlmostEqual(val, st[j.name], delta=2e-3, msg=j.name)

    def test_stabilator_root_gap_over_range(self):
        """R-38: the moving panel keeps >= 6 mm (layout.clearances) from the fixed root stub over -20..+15 deg; the
        nominal spanwise gap is tail.surfaces.stabilator.params.y_root_gap."""
        prm = self.spec["tail"]["surfaces"]["stabilator"]["params"]
        y_stub = self.spec["tail"]["surfaces"]["stabilator_stub"]["sections"][-1]["y"]
        self.assertAlmostEqual(prm["y_root"] - y_stub, prm["y_root_gap"], places=6)
        need = [r["min_mm"] for r in self.spec["layout"]["clearances"] if r.get("name") ==
                "stabilator vs fixed root stub"][0] * 1e-3
        stub = self.reg.parts["YK250-TL-252-R"].mesh.to_manifold()
        moving = [p for p in self.reg.moving_parts("stabilator_R") if p.id.startswith("YK250-TL-2")]
        for st in self.reg.sequences["stabilator_R"]:
            for p in moving:
                g = float(self.reg.posed_mesh(p, st).to_manifold().min_gap(stub, 0.05))
                self.assertGreaterEqual(g, need - 1e-6, f"{p.id} {st}")

    def test_bumper_skid_under_propeller_disc(self):
        """The replaceable shoe is the lowest tail point and spans the layout bumper contact point (R-48)."""
        cp = np.asarray(self.spec["tail"]["surfaces"]["ventral"]["bumper"]["contact_point"], float)
        lo, hi = self.tail["YK250-TL-261"].mesh.bounds()
        self.assertTrue(lo[0] <= cp[0] <= hi[0])
        z_low = min(p.mesh.bounds()[0][2] for pid, p in self.tail.items() if pid != "YK250-TL-261")
        self.assertLess(lo[2], z_low)
        self.assertLessEqual(lo[2], cp[2] + 0.002)

    # ------------------------------------------------------------------ fasteners
    def test_fasteners_join_tail_parts(self):
        fs = [f for f in self.reg.fasteners() if any(pid in self.tail for pid in f.joins)]
        self.assertGreater(len(fs), 100)
        for f in fs:
            for pid in f.joins:
                self.assertIn(pid, self.reg.parts, f.id)
            self.assertRegex(f.spec, r"M\d|ISO 2341", f.id)
            self.assertIn(f.step, TAIL_STEPS, f.id)
        pins = [f for f in fs if f.kind == "clevis_pin"]
        self.assertEqual(len(pins), 2 * len(T.HINGE_S))
        for f in pins:
            self.assertAlmostEqual(f.d, T.HINGE.pin_d)

    # ------------------------------------------------------------------ mass
    def test_masses_against_budget(self):
        mt = sum(self.reg.mass(p) for p in self.tail.values() if p.group == "tail")
        mc = sum(self.reg.mass(p) for p in self.tail.values() if p.group == "controls")
        b = self.spec["mass"]["budget"]
        self.assertLessEqual(mt, b["tail"]["target_kg"])                 # ceiling (mass.budget_rules)
        items = {i["name"]: i for i in self.spec["mass"]["items"]}
        share = sum(items[k]["mass_kg"] for k in ("actuators_rudders_2x_DA26", "actuators_stabilators_2x_DA30",
                                                    "control_surfaces_rudders_pair"))
        self.assertLessEqual(mc, share + 0.05)
        self.assertGreater(mt, 5.0)
        self.assertGreater(mc, 2.5)


if __name__ == "__main__":
    unittest.main()
