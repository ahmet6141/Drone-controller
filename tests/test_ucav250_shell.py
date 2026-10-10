"""ucav250 shell producer (design/shell.py): every body skin, hatch, access door, fairing, RF window, the nose cone, the
cowl pieces, the LERX/glove skins and the wing-root fairing are clean under every design-rule check against the
chassis, every part honours the producer contract (ids in the shell range, material / process / thickness or layup,
parent, assembly step, explode vector), the layout panels are all built, the moving parts follow spec.layout
(parachute hatch lift-off, refuel door hinge), the fastener rows follow the layout fastening types, and the group mass
is reported against spec.mass.budget. Nothing here writes to the repository (checks run with write=False)."""
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
    from ucav250.design import shell as SH
    HAVE = True
except Exception:  # pragma: no cover
    HAVE = False

ID_RE = re.compile(r"^YK250-SH-(\d{3})(-[LR])?$")
FORBIDDEN = ("hardpoint", "hard-point", "hard point", "pylon", "weapon", "munition", "release mechanism")


@unittest.skipUnless(HAVE, "ucav250 dependencies missing")
class TestShellBuild(unittest.TestCase):
    """One registry for the class: chassis, shell registered twice (the second call is ignored), hardware."""

    @classmethod
    def setUpClass(cls):
        cls.reg = build_registry(modules=["chassis", "shell", "shell", "hardware"], strict=False)
        cls.spec = cls.reg.spec
        cls.shell = {pid: p for pid, p in cls.reg.parts.items() if pid.startswith("YK250-SH-")}
        cls.fast = [f for f in cls.reg.fasteners() if f.id.startswith("YK250-SH-")]
        cls.summary = K.run_all(cls.reg, quick=False, write=False, focus=set(cls.shell))

    # ------------------------------------------------------------------ checks
    def test_design_rule_checks_clean(self):
        bad = {k: v[:3] for k, v in self.summary["details"].items() if v}
        self.assertTrue(self.summary["ok"], f"shell violations: {bad}")

    def test_registered_once(self):
        self.assertIn("shell: register() second call ignored", self.reg.log)

    # ------------------------------------------------------------------ contract
    def test_ids_in_shell_range_and_layout_panels_built(self):
        lo, hi = self.spec["layout"]["part_numbers"]["shell"]
        self.assertGreater(len(self.shell), 50)
        for pid, p in self.shell.items():
            m = ID_RE.match(pid)
            self.assertIsNotNone(m, pid)
            self.assertTrue(lo <= int(m.group(1)) <= hi, pid)
            self.assertEqual(p.group, "shell", pid)
            if m.group(2):
                self.assertIn(pid[:-1] + ("L" if pid.endswith("R") else "R"), self.shell, f"{pid} has no mirror")
                self.assertEqual(p.side, m.group(2)[1], pid)
        for pan in self.spec["layout"]["shell"]["panels"]:
            ids = [pan["part"] + "-R", pan["part"] + "-L"] if pan.get("mirror") else [pan["part"]]
            for pid in ids:
                self.assertIn(pid, self.shell, pan["id"])
        wrf = self.spec["layout"]["shell"]["wing_root_fairing"]["part"]
        self.assertIn(wrf + "-R", self.shell)
        self.assertIn(wrf + "-L", self.shell)

    def test_part_contract(self):
        mats, procs = self.spec["materials"], self.spec["processes"]
        steps = {int(s["step"]) for s in self.spec["assembly"]["steps"]}
        for pid, p in self.shell.items():
            self.assertTrue(p.name and p.name_tr, pid)
            self.assertIn(p.material, mats, pid)
            if p.purchased:
                self.assertTrue(p.vendor, pid)
            else:
                self.assertIn(p.process, procs, pid)
                self.assertTrue(p.thickness is not None or p.layup, pid)
                self.assertGreaterEqual(p.thickness or 1.0, float(procs[p.process]["min_thickness"]) - 1e-9, pid)
            self.assertIn(p.step, steps, pid)
            self.assertIn(p.step, set(SH.STEP.values()), pid)
            self.assertEqual(len(tuple(p.explode)), 3, pid)
            self.assertTrue(p.parent in self.reg.parts, f"{pid}: parent {p.parent}")
            for c in p.contacts:
                self.assertIn(c, self.reg.parts, f"{pid}: contact {c}")

    def test_rf_windows_are_glass(self):
        """layout.shell.rules.layups: RF windows GFRP, no carbon within the window."""
        for pan in self.spec["layout"]["shell"]["panels"]:
            if pan.get("material") == "gfrp_7781_mtm45":
                ids = [pan["part"] + "-R", pan["part"] + "-L"] if pan.get("mirror") else [pan["part"]]
                for pid in ids:
                    self.assertEqual(self.shell[pid].material, "gfrp_7781_mtm45", pid)

    def test_no_weapon_wording(self):
        for pid, p in self.shell.items():
            text = " ".join([p.name, p.name_tr, p.notes]).lower()
            for w in FORBIDDEN:
                self.assertNotIn(w, text, pid)
        src = Path(SH.__file__).read_text(encoding="utf-8").lower()
        for w in FORBIDDEN:
            self.assertNotIn(w, src)

    def test_meshes_closed_and_mirror_equal(self):
        for pid, p in self.shell.items():
            self.assertTrue(p.mesh.check()["ok"], pid)
            if not pid.endswith("-R"):
                continue
            q = self.shell[pid[:-1] + "L"]
            vr, vl = p.mesh.volume(), q.mesh.volume()
            self.assertAlmostEqual(vr, vl, delta=2e-3 * vr + 1e-12, msg=pid)

    # ------------------------------------------------------------------ interfaces from spec.layout
    def test_joints_follow_the_layout(self):
        lj = {j["name"]: j for j in self.spec["layout"]["mechanisms"]["joints"]}["para_hatch"]
        j = self.reg.joints["para_hatch"]
        self.assertEqual(j.kind, lj["kind"])
        self.assertTrue(np.allclose(j.origin, lj["origin"]))
        self.assertTrue(np.allclose(j.axis, np.asarray(lj["axis"]) / np.linalg.norm(lj["axis"])))
        self.assertAlmostEqual(j.hi, lj["hi"])
        self.assertEqual(self.shell["YK250-SH-367"].joint, "para_hatch")
        h = {p["id"]: p for p in self.spec["layout"]["shell"]["panels"]}["P-REFUEL"]["hinge"]
        r = self.reg.joints["refuel_door"]
        self.assertEqual(r.kind, "revolute")
        self.assertAlmostEqual(r.lo, math.radians(h["range_deg"][0]))
        self.assertAlmostEqual(r.hi, math.radians(h["range_deg"][1]))
        self.assertAlmostEqual(float(r.origin[0]), float(h["axis_point"][0]), delta=1e-6)
        self.assertEqual(self.shell["YK250-SH-385"].joint, "refuel_door")

    def test_turret_ring_opening(self):
        """HD59 opening radius = ball / 2 + radial clearance (spec.payload.turret)."""
        T = self.spec["payload"]["turret"]
        r = 0.5 * float(T["ball_diameter"]) + float(T["bay"]["aperture_ring"]["radial_clearance"])
        V = self.shell["YK250-SH-363"].mesh.V
        d = np.hypot(V[:, 0] - float(T["bay_center_x"]), V[:, 1])
        self.assertAlmostEqual(float(d.min()), r, delta=0.0015)

    def test_stub_band_trimmed_to_the_layout_band(self):
        sb = {p["id"]: p for p in self.spec["layout"]["shell"]["panels"]}["P-STUBROOT"]
        z0, z1 = sb["z_band"]
        lo, hi = self.shell["YK250-SH-393-R"].mesh.bounds()
        self.assertGreater(float(lo[2]), z0 - 0.012)
        self.assertLess(float(hi[2]), z1 + 0.012)
        self.assertGreater(float(lo[1]), sb["y"][0] - 0.012)

    # ------------------------------------------------------------------ fasteners
    def test_fastener_rows_follow_the_layout_types(self):
        self.assertGreater(len(self.fast), 600)
        pans = {p["part"]: p for p in self.spec["layout"]["shell"]["panels"]}
        by_owner: dict[str, list] = {}
        for f in self.fast:
            for pid in f.joins:
                self.assertIn(pid, self.reg.parts, f.id)
            self.assertTrue(f.id.startswith(f.joins[0] + "-"), f.id)
            by_owner.setdefault(f.joins[0], []).append(f)
        for pid, fs in by_owner.items():
            pan = pans.get(re.sub(r"-[LR]$", "", pid))
            typ = (pan or {}).get("fastening", {}).get("type")
            for f in fs:
                if typ == "camloc":
                    self.assertEqual(f.kind, "camloc", f.id)
                elif typ == "nutplate+screw":
                    self.assertEqual(f.kind, "bolt", f.id)
                    self.assertIn("nutplate", f.nut, f.id)
                    self.assertRegex(f.spec, r"ISO 7380 M[34]", f.id)
                elif typ == "bonded":
                    self.assertEqual(f.kind, "rivet", f.id)
            P = np.array([f.position for f in fs])
            if len(P) > 1:
                D = np.linalg.norm(P[:, None] - P[None], axis=2) + np.eye(len(P))
                d = max(f.d for f in fs)
                self.assertGreaterEqual(float(D.min()), 3.0 * d - 1e-6, pid)
        # every structural skin is screwed to the chassis lands
        for pan in self.spec["layout"]["shell"]["panels"]:
            if pan.get("attach") == "fixed" and pan["fastening"]["type"] == "nutplate+screw":
                self.assertGreaterEqual(len(by_owner.get(pan["part"], [])), 4, pan["id"])

    # ------------------------------------------------------------------ mass
    def test_group_mass_against_budget(self):
        m = sum(self.reg.mass(p) for p in self.shell.values())
        b = self.spec["mass"]["budget"]["shell"]
        self.assertLessEqual(abs(m - float(b["target_kg"])), float(b["tol_kg"]), f"shell {m:.3f} kg")


if __name__ == "__main__":
    unittest.main()
