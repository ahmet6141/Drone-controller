"""ucav250 wing producer (design/wing.py): the outer panels, ailerons, flaps and their drives are clean under every
design-rule check, every part honours the producer contract (ids in the wing range, material / process / thickness,
parent, assembly step, explode vector), the interfaces are taken from spec.layout (wing joint pins, hinge joints,
actuator models), the four-bar linkages close exactly in every registered sequence state and stay inside the actuator
datasheet travel, and the group masses are reported against spec.mass.budget. Nothing here writes to the repository
(checks run with write=False)."""
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
    from ucav250.core.assemble import build_registry
    from ucav250.design import wing as WG
    HAVE = True
except Exception:  # pragma: no cover
    HAVE = False

ID_RE = re.compile(r"^YK250-(WG|FC)-(\d{3})-([LR])$")
FORBIDDEN = ("hardpoint", "hard-point", "hard point", "pylon", "weapon", "munition", "release mechanism")


@unittest.skipUnless(HAVE, "ucav250 dependencies missing")
class TestWingBuild(unittest.TestCase):
    """One registry for the class: chassis, wing registered twice (the second call is ignored), hardware."""

    @classmethod
    def setUpClass(cls):
        cls.reg = build_registry(modules=["chassis", "wing", "wing", "hardware"], strict=False)
        cls.spec = cls.reg.spec
        cls.wing = {pid: p for pid, p in cls.reg.parts.items() if pid.startswith(("YK250-WG-", "YK250-FC-"))}
        focus = set(cls.wing)
        cls.summary = K.run_all(cls.reg, quick=False, write=False, focus=focus)

    # ------------------------------------------------------------------ checks
    def test_design_rule_checks_clean(self):
        bad = {k: v[:3] for k, v in self.summary["details"].items() if v}
        self.assertTrue(self.summary["ok"], f"wing violations: {bad}")

    def test_registered_once(self):
        self.assertIn("wing: register() second call ignored", self.reg.log)

    # ------------------------------------------------------------------ contract
    def test_ids_in_wing_range_and_mirrored(self):
        lo, hi = self.spec["layout"]["part_numbers"]["wing"]
        self.assertGreater(len(self.wing), 120)
        for pid, p in self.wing.items():
            m = ID_RE.match(pid)
            self.assertIsNotNone(m, pid)
            n = int(m.group(2))
            self.assertTrue(lo <= n <= hi, pid)
            self.assertEqual(m.group(1), "WG" if n < 200 else "FC", pid)
            self.assertEqual(p.group, "wing" if m.group(1) == "WG" else "controls", pid)
            other = pid[:-1] + ("L" if pid.endswith("R") else "R")
            if pid != "YK250-WG-192-L":                 # port-only pitot mast (layout AD-PITOT-WING)
                self.assertIn(other, self.wing, f"{pid} has no mirror")
            self.assertEqual(p.side, m.group(3), pid)

    def test_part_contract(self):
        mats, procs = self.spec["materials"], self.spec["processes"]
        steps = {int(s["step"]) for s in self.spec["assembly"]["steps"]}
        for pid, p in self.wing.items():
            self.assertTrue(p.name and p.name_tr, pid)
            if p.purchased:
                self.assertTrue(p.vendor and p.mass_kg, pid)
            else:
                self.assertIn(p.material, mats, pid)
                self.assertIn(p.process, procs, pid)
                self.assertTrue(p.thickness is not None or p.layup, pid)
                self.assertGreaterEqual(p.thickness or 1.0, float(procs[p.process]["min_thickness"]) - 1e-9, pid)
            self.assertIn(p.step, steps, pid)
            self.assertIn(p.step, (WG.STEP, WG.STEP_RIG), pid)
            self.assertEqual(len(tuple(p.explode)), 3, pid)
            self.assertTrue(p.parent is None or p.parent in self.reg.parts, f"{pid}: parent {p.parent}")
            for c in p.contacts:
                self.assertIn(c, self.reg.parts, f"{pid}: contact {c}")

    def test_no_weapon_wording(self):
        for pid, p in self.wing.items():
            text = " ".join([p.name, p.name_tr, p.notes]).lower()
            for w in FORBIDDEN:
                self.assertNotIn(w, text, pid)
        src = Path(WG.__file__).read_text(encoding="utf-8").lower()
        for w in FORBIDDEN:
            self.assertNotIn(w, src)

    def test_meshes_closed_and_mirror_equal(self):
        for pid, p in self.wing.items():
            if not pid.endswith("-R"):
                self.assertGreater(p.mesh.volume(), 0.0, pid)
                continue
            q = self.wing[pid[:-1] + "L"]
            vr, vl = p.mesh.volume(), q.mesh.volume()
            self.assertGreater(vr, 0.0, pid)
            self.assertAlmostEqual(vr, vl, delta=1e-3 * vr + 1e-12, msg=pid)

    # ------------------------------------------------------------------ interfaces from spec.layout
    def test_joint_hardware_on_layout_interfaces(self):
        wj = self.spec["layout"]["chassis"]["wing_joint"]
        self.assertEqual(wj["main_spar"]["tongue"]["part"] + "-R", "YK250-WG-151-R")
        self.assertEqual(wj["rear_spar"]["lug"]["part"] + "-R", "YK250-WG-152-R")
        for i, pin in enumerate(wj["main_spar"]["pins"]):
            m = self.reg.parts[f"YK250-WG-{158 + i}-R"].mesh
            c, a = np.asarray(pin["position"], float), np.asarray(pin["axis"], float)
            V = m.V - c
            radial = np.linalg.norm(V - np.outer(V @ a, a), axis=1)
            shank = radial[V @ a > 0.0]                 # far end of the shank (head is on the -axis side)
            self.assertAlmostEqual(float(shank.max()), 0.5 * pin["diameter"], delta=2e-4)
        rp = wj["rear_spar"]["pin"]
        m = self.reg.parts["YK250-WG-160-R"].mesh
        c = np.asarray(rp["position"], float)
        lo, hi = m.bounds()
        self.assertTrue(np.allclose(0.5 * (lo + hi)[:2], c[:2], atol=1e-4))

    def test_hinge_joints_match_layout(self):
        lj = {j["name"]: j for j in self.spec["layout"]["mechanisms"]["joints"]}
        for name in ("aileron_R", "aileron_L", "flap_R", "flap_L"):
            j, l = self.reg.joints[name], lj[name]
            self.assertTrue(np.allclose(j.origin, l["origin"]))
            self.assertTrue(np.allclose(j.axis, np.asarray(l["axis"]) / np.linalg.norm(l["axis"])))
            self.assertAlmostEqual(j.lo, l["lo"])
            self.assertAlmostEqual(j.hi, l["hi"])
            self.assertEqual(j.prop, l["prop"])
            self.assertTrue(j.expr, f"{name}: surface joint is coupled to its linkage")

    def test_systems_mounts_from_layout(self):
        """Port pitot mast clamps the AD-PITOT-WING probe envelope; the tip fairing leaves the LT-WING box free."""
        from ucav250.core import geom as G
        items = {i["id"]: i for i in self.spec["layout"]["systems"]["air_data_lights"]}
        it = items["AD-PITOT-WING"]
        probe = G.cylinder(0.5 * it["diameter"], it["p0"], it["p1"], n=32)
        mast = self.reg.parts["YK250-WG-192-L"].mesh
        self.assertLess(G.intersection_volume(mast, probe), 1e-12)
        self.assertLess(G.min_gap(mast, probe, 0.01), 3e-4)
        b = np.asarray(items["LT-WING"]["box"], float)
        box = G.box(b[1] - b[0], 0.5 * (b[0] + b[1]))
        self.assertLess(G.intersection_volume(self.reg.parts["YK250-WG-186-R"].mesh, box), 1e-12)

    def test_actuators_are_the_layout_models(self):
        acts = {a["id"]: a for a in self.spec["layout"]["systems"]["actuators"]}
        for w, aid, pnum in (("aileron", "ACT-AILERON", 204), ("flap", "ACT-FLAP", 206)):
            p = self.reg.parts[f"YK250-FC-{pnum}-R"]
            self.assertEqual(acts[aid]["part"], f"YK250-FC-{pnum}")
            d = WG.actuator_data(WG.ACT_KEY[w])
            self.assertTrue(p.purchased)
            self.assertAlmostEqual(p.mass_kg, d["mass_kg"])
            model = "DA 26" if w == "aileron" else "DA 30"
            self.assertIn(model, p.vendor)

    # ------------------------------------------------------------------ mechanisms
    def test_linkages_close_in_every_sequence_state(self):
        """In every registered state the rod's horn end (posed through servo -> rod joints) meets the horn eye (posed
        through the surface joint): the four-bar closes."""
        op = WG.OP(self.spec)
        for w in ("flap", "aileron"):
            dg = WG.drive_geo(op, w)
            B0 = dg["B0"]
            for s, M in (("R", np.eye(3)), ("L", np.diag([1.0, -1.0, 1.0]))):
                seq = self.reg.sequences[f"{w}_{s}_linkage"]
                self.assertGreaterEqual(len(seq), 9)
                b = (M @ B0)[None]
                for st in seq:
                    vh = b.copy()
                    for j in self.reg.joint_chain(f"{w}_{s}"):
                        vh = j.apply(vh, st.get(j.name, j.rest))
                    vr = b.copy()
                    for j in self.reg.joint_chain(f"{w}_rod_{s}"):
                        vr = j.apply(vr, st.get(j.name, j.rest))
                    self.assertLess(float(np.linalg.norm(vh - vr)), 5e-5, f"{w}_{s} {st}")

    def test_servo_travel_and_transmission(self):
        op = WG.OP(self.spec)
        for w, lim in (("aileron", "travel_max_standard_deg"), ("flap", "travel_max_deg")):
            dg = WG.drive_geo(op, w)
            d = WG.actuator_data(WG.ACT_KEY[w])
            lo, hi = d[lim]
            k = dg["kin"]
            th = np.degrees(np.asarray(k["servo_angles"]) - dg["th0"])
            # neutral (surface at the linkage neutral) is the middle of the servo stroke
            mid = 0.5 * (th.min() + th.max())
            self.assertLessEqual(th.max() - mid, hi + 1e-6, w)
            self.assertGreaterEqual(th.min() - mid, lo - 1e-6, w)
            self.assertGreater(k["min_transmission_deg"], 60.0, w)
            self.assertGreaterEqual(min(dg["margins"]), 0.003, w)

    def test_pushrod_column_and_rod_end_on_built_geometry(self):
        """The structures checks C-PUSHROD / C-RODEND repeated on the as-built linkage (rod length, horn radius and
        rod section of this module): actuator peak torque x largest linkage ratio at the horn, FoS 1.5 column
        (Johnson-Euler, pinned ends), rod-end bolt bearing in the 3 mm 7075 horn with the 3.33 push-pull factor."""
        from ucav250.analysis import structlib as ST
        op = WG.OP(self.spec)
        al = self.spec["materials"]["al_7075_t651_plate"]
        for w in ("flap", "aileron"):
            dg = WG.drive_geo(op, w)
            ctl = self.spec["wing"]["controls"][w]
            F = float(ctl["actuator"]["torque_peak_Nm"]) * float(ctl["checks"]["ratio_max"]) / dg["r_h"]
            A = math.pi / 4 * (WG.ROD_OD ** 2 - WG.ROD_ID ** 2)
            I = math.pi / 64 * (WG.ROD_OD ** 4 - WG.ROD_ID ** 4)
            je = ST.johnson_euler(float(al["E"]), float(al["Fcy"]), A, I, dg["L_rod"])
            self.assertGreaterEqual(je["P_cr"] / (1.5 * F) - 1.0, 0.0, w)
            self.assertGreaterEqual(WG.ARM_T * 0.003 * float(al["Fbru"]) / (3.33 * F) - 1.0, 0.0, w)

    def test_root_bay_skin_doubler(self):
        """structures.sizing.wing_joint.transition: one extra PW ply per box skin between the spars over the first
        0.12 m of the panel (OML on the loft, inner face stepped), none outboard of the drop-off."""
        op = WG.OP(self.spec)
        tr = self.spec["structures"]["sizing"]["wing_joint"]["transition"]
        t_ply = float(self.spec["materials"]["cfrp_pw_mtm45_as4"]["ply_t"])
        self.assertAlmostEqual(op.t_dbl, int(tr["root_bay_skin_doubler_plies_per_face"]) * t_ply)
        for y, full in ((0.75, True), (0.80, True), (0.90, False)):
            e = op.eta(y)
            for side in ("up", "lo"):
                k = op.knots(e, side)
                s = 0.5 * (k["b6"] + k["b7"])
                t = float(op.t_skin(e, side, [s])[0])
                self.assertAlmostEqual(t, op.t_box(e, side) + (op.t_dbl if full else 0.0), delta=1e-9, msg=(y, side))
        self.assertAlmostEqual(op.y_dbl_end - (WG.ROOT_RIB_Y + WG.T_RIB), float(tr["root_bay_doubler_length_m"]))

    def test_coupled_joint_expressions_follow_the_tables(self):
        """The Blender expressions (polynomials in the control property) reproduce the exact four-bar values."""
        for w in ("flap", "aileron"):
            surf = self.reg.joints[f"{w}_R"]
            for jn in (f"{w}_servo_R", f"{w}_rod_R"):
                j = self.reg.joints[jn]
                for st in self.reg.sequences[f"{w}_R_linkage"]:
                    deg = st[f"{w}_R"] / surf.scale
                    val = eval(j.expr, {"__builtins__": {}}, {surf.prop: deg})
                    self.assertAlmostEqual(val, st[jn], delta=2e-3, msg=jn)

    # ------------------------------------------------------------------ fasteners
    def test_fasteners_join_wing_parts(self):
        fs = [f for f in self.reg.fasteners() if any(pid in self.wing for pid in f.joins)]
        self.assertGreater(len(fs), 90)
        for f in fs:
            for pid in f.joins:
                self.assertIn(pid, self.reg.parts, f.id)
            self.assertRegex(f.spec, r"M\d|ISO 2341", f.id)
        pins = [f for f in fs if f.kind == "clevis_pin"]
        self.assertEqual(len(pins), 2 * (len(WG.HINGE_Y["flap"]) + len(WG.HINGE_Y["aileron"])))
        keepers = [f for f in fs if "YK250-CH-001" in f.joins]
        self.assertEqual(len(keepers), 8)

    # ------------------------------------------------------------------ mass
    def test_masses_against_budget(self):
        mw = sum(self.reg.mass(p) for p in self.wing.values() if p.group == "wing")
        mc = sum(self.reg.mass(p) for p in self.wing.values() if p.group == "controls")
        b = self.spec["mass"]["budget"]
        self.assertLessEqual(mw, b["wing"]["target_kg"] + b["wing"]["tol_kg"])
        items = {i["name"]: i for i in self.spec["mass"]["items"]}
        share = sum(items[k]["mass_kg"] for k in ("actuators_ailerons_2x_DA26", "actuators_flaps_2x_DA30",
                                                    "control_surfaces_ailerons_pair", "control_surfaces_flaps_pair"))
        self.assertLessEqual(mc, share + 0.05)
        self.assertGreater(mw, 8.0)
        self.assertGreater(mc, 3.0)


if __name__ == "__main__":
    unittest.main()
