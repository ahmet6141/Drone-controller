"""ucav250 systems producer (design/systems.py): avionics and power (PDU, contactor / fuse block, DC-DC, buffer battery,
autopilot, datalinks, transponder, Remote ID, FTS unit) with their mounts on the chassis trays and the avionics deck,
mated connectors, the harness trunks of layout.systems.harness with their frame / member grommets, the centre-line
conduit and the wing / tail connectors, the antennas (GNSS, nose-cone dipoles, transponder blade), the nose pitot and
the parachute installation (container, pads, girth strap, bridle legs on their spools, hatch latch).

The systems parts are clean under every design-rule check (mesh, static, swept, clearance, thickness, fastener edge
distance / pierce / pitch, declared contacts, attachment graph) against the chassis and the shell; every part honours
the producer contract; the interfaces come from spec.layout (equipment boxes, antenna points, harness paths and
penetrations, frame cut-outs); and the group mass is compared with spec.mass.budget.systems. Nothing here writes to
the repository (checks run with write=False)."""
from __future__ import annotations

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
    from ucav250.design import systems as SY
    HAVE = True
except Exception:  # pragma: no cover
    HAVE = False

ID_RE = re.compile(r"^YK250-SY-(\d{3})(-[LR])?$")
FORBIDDEN = ("hardpoint", "hard-point", "hard point", "pylon", "weapon", "munition", "release mechanism")
STEPS = (19, 20, 21, 22)


@unittest.skipUnless(HAVE, "ucav250 dependencies missing")
class TestSystemsBuild(unittest.TestCase):
    """One registry for the class: chassis, shell, systems registered twice (the second call is ignored), hardware."""

    @classmethod
    def setUpClass(cls):
        cls.reg = build_registry(modules=["chassis", "shell", "systems", "systems", "hardware"], strict=False)
        cls.spec = cls.reg.spec
        cls.L = cls.spec["layout"]
        cls.sy = {pid: p for pid, p in cls.reg.parts.items() if ID_RE.match(pid)}
        cls.summary = K.run_all(cls.reg, quick=False, write=False, focus=set(cls.sy))

    # ------------------------------------------------------------------ checks
    def test_design_rule_checks_clean(self):
        bad = {k: v[:3] for k, v in self.summary["details"].items() if v}
        self.assertTrue(self.summary["ok"], f"systems violations: {bad}")
        for k in ("mesh", "static", "swept", "clearance", "thickness", "fastener", "contact", "attachment"):
            self.assertIn(k, self.summary["violations"], k)
            self.assertEqual(self.summary["violations"][k], 0, k)

    def test_registered_once(self):
        self.assertIn("systems: already registered, second call ignored", self.reg.log)

    # ------------------------------------------------------------------ contract
    def test_ids_in_systems_range_and_mirrored(self):
        lo, hi = self.L["part_numbers"]["systems"]
        self.assertGreaterEqual(len(self.sy), 80)
        for pid, p in self.sy.items():
            m = ID_RE.match(pid)
            self.assertTrue(lo <= int(m.group(1)) <= hi, pid)
            self.assertEqual(p.group, "systems", pid)
            if m.group(2):
                other = pid[:-1] + ("L" if pid.endswith("R") else "R")
                self.assertIn(other, self.sy, f"{pid} has no mirror")
                self.assertEqual(p.side, m.group(2)[1], pid)
            else:
                self.assertEqual(p.side, "C", pid)

    def test_layout_fixed_ids(self):
        """Every systems-group part the layout fixes on the body exists with its layout name and mass."""
        sy = self.L["systems"]
        for e in sy["equipment"]:
            if e.get("group") != "systems" or not e["part"].startswith("YK250-SY-"):
                continue
            self.assertIn(e["part"], self.sy, e["id"])
            self.assertEqual(self.sy[e["part"]].name, e["name"], e["id"])
            self.assertAlmostEqual(self.reg.mass(self.sy[e["part"]]), float(e["mass_kg"]) - (
                0.030 * sum(float(self.L["chassis"]["parachute"]["bridle"][k]["length"])
                            for k in ("forward_leg", "aft_leg")) if e["id"] == "EQ-PARACHUTE" else 0.0), places=3,
                msg=e["id"])
        for key in ("antennas", "air_data_lights"):
            for a in sy[key]:
                body = ("YK250-SH-350", "YK250-SH-354", "YK250-SH-372")
                if a.get("window", "") in body or str(a.get("window", "")).startswith("external") or \
                        a["id"] == "AD-PITOT-NOSE":
                    self.assertIn(a["part"], self.sy, a["id"])
        for t in sy["harness"]["trunks"]:
            ids = [t["part"] + "-R", t["part"] + "-L"] if t.get("mirror") else [t["part"]]
            for pid in ids:
                self.assertIn(pid, self.sy, t["id"])

    def test_part_contract(self):
        mats, procs = self.spec["materials"], self.spec["processes"]
        steps = {int(s["step"]) for s in self.spec["assembly"]["steps"]}
        for pid, p in self.sy.items():
            self.assertTrue(p.name and p.name_tr, pid)
            self.assertIn(p.step, steps, pid)
            self.assertIn(p.step, STEPS, pid)
            if p.purchased:
                self.assertTrue(p.vendor, pid)
                self.assertIsNotNone(p.mass_kg, pid)
                self.assertGreater(p.mass_kg, 0.0, pid)
            else:
                self.assertIn(p.material, mats, pid)
                self.assertIn(p.process, procs, pid)
                self.assertIsNotNone(p.thickness, pid)
                self.assertGreaterEqual(p.thickness, float(procs[p.process]["min_thickness"]) - 1e-9, pid)
            self.assertEqual(len(tuple(p.explode)), 3, pid)
            self.assertGreater(float(np.linalg.norm(p.explode)), 0.0, pid)
            self.assertIn(p.parent, self.reg.parts, f"{pid}: parent {p.parent}")
            self.assertTrue(p.contacts, pid)
            for c in p.contacts:
                self.assertIn(c, self.reg.parts, f"{pid}: contact {c}")

    def test_no_weapon_wording(self):
        for pid, p in self.sy.items():
            text = " ".join([p.name, p.name_tr, p.notes, p.vendor]).lower()
            for w in FORBIDDEN:
                self.assertNotIn(w, text, pid)
        src = Path(SY.__file__).read_text(encoding="utf-8").lower()
        for w in FORBIDDEN:
            self.assertNotIn(w, src)

    def test_meshes_closed_and_mirror_equal(self):
        for pid, p in self.sy.items():
            self.assertGreater(p.mesh.volume(), 0.0, pid)
            if pid.endswith("-R"):
                vr, vl = p.mesh.volume(), self.sy[pid[:-1] + "L"].mesh.volume()
                self.assertAlmostEqual(vr, vl, delta=2e-3 * vr + 1e-12, msg=pid)

    # ------------------------------------------------------------------ interfaces from spec.layout
    def test_units_inside_their_layout_boxes(self):
        n = 0
        for e in self.L["systems"]["equipment"]:
            if e["part"] not in self.sy:
                continue
            lo, hi = self.sy[e["part"]].mesh.bounds()
            b = np.asarray(e["box"], float)
            self.assertTrue(np.all(lo >= b[0] - 1e-6) and np.all(hi <= b[1] + 1e-6), e["id"])
            n += 1
        self.assertGreaterEqual(n, 12)

    def test_connectors_inside_the_connector_allowance(self):
        """Each mated plug sits against its unit face and inside the layout envelope_with_connector."""
        n = 0
        for e in self.L["systems"]["equipment"]:
            if "envelope_with_connector" not in e or e["part"] not in self.sy:
                continue
            plugs = [p for p in self.sy.values() if p.parent == e["part"] and "mated connector" in p.name]
            for p in plugs:
                lo, hi = p.mesh.bounds()
                b = np.asarray(e["envelope_with_connector"], float)
                self.assertTrue(np.all(lo >= b[0] - 1e-6) and np.all(hi <= b[1] + 1e-6), p.id)
                n += 1
        self.assertGreaterEqual(n, 9)

    def test_antennas_on_their_layout_points(self):
        for a in self.L["systems"]["antennas"]:
            if a["id"] not in ("ANT-GNSS1", "ANT-GNSS2", "ANT-C2A", "ANT-FTS"):
                continue
            lo, hi = self.sy[a["part"]].mesh.bounds()
            self.assertLess(float(np.linalg.norm(0.5 * (lo + hi) - np.asarray(a["point"], float))), 0.001, a["id"])
            self.assertTrue(np.allclose(hi - lo, a["size"], atol=0.001), a["id"])
        x = next(a for a in self.L["systems"]["antennas"] if a["id"] == "ANT-XPDR")
        lo, hi = self.sy[x["part"]].mesh.bounds()
        p, s = np.asarray(x["point"], float), np.asarray(x["size"], float)
        self.assertLessEqual(lo[2], p[2] - 0.5 * s[2] + 0.001)      # blade reaches the layout tip
        self.assertTrue(lo[0] >= p[0] - 0.5 * s[0] - 1e-6 and hi[0] <= p[0] + 0.5 * s[0] + 1e-6)

    def test_harness_passes_the_declared_penetrations(self):
        """The bundles fill the declared member penetrations (centre box cover, side-of-body / glove ribs, well wall):
        each layout penetration point lies inside the bundle that the layout routes through it."""
        n = 0
        for t in self.L["systems"]["harness"]["trunks"]:
            pid = t["part"] + ("-R" if t.get("mirror") else "")
            m = self.sy[pid].mesh
            for pen in t.get("penetrations") or []:
                q = np.asarray(pen["point"], float)
                ball = G.sphere(0.0015, q, n=12)
                self.assertGreater(G.intersection_volume(ball, m), 0.0, f"{t['id']} {pen['member']}")
                n += 1
        self.assertGreaterEqual(n, 5)

    def test_every_harness_cutout_crossing_has_a_grommet(self):
        """Every frame the bundles cross through a harness cut-out carries a grommet set touching that frame."""
        sets = {p.parent for p in self.sy.values() if "grommet" in p.name}
        for st in self.L["stations"]:
            if any(c["id"].startswith(("C-HARN", "C-COAX", "C-FW-HARN")) for c in st.get("cutouts", [])):
                self.assertIn(st["part"], sets, st["id"])

    # ------------------------------------------------------------------ fasteners
    def test_fasteners_metric_and_hardware_built(self):
        fs = [f for f in self.reg.fasteners() if any(pid in self.sy for pid in f.joins)]
        self.assertGreaterEqual(len(fs), 70)
        for f in fs:
            for pid in f.joins:
                self.assertIn(pid, self.reg.parts, f.id)
            self.assertRegex(f.spec, r"^ISO (4762|7380) M[345]x\d+-A2-70$", f.id)
            self.assertIn(f.step, STEPS, f.id)
            self.assertIn(f"YK250-HW-{f.id}", self.reg.parts)

    # ------------------------------------------------------------------ mass
    def test_mass_against_budget(self):
        """Group mass at or below the spec.mass.budget.systems ceiling; the harness parts carry exactly the layout
        harness item (cables 70 % + connectors / grommets 30 %)."""
        b = self.spec["mass"]["budget"]["systems"]
        m = sum(self.reg.mass(p) for p in self.reg.parts.values() if p.group == "systems")
        self.assertLessEqual(m, float(b["target_kg"]), f"systems group {m:.3f} kg")
        harness = [p for p in self.sy.values() if p.vendor.startswith(("harness assembly", "moulded", "fire-resistant",
                                                                         "split nylon", "MIL-DTL-38999"))]
        self.assertAlmostEqual(sum(self.reg.mass(p) for p in harness), SY.HARNESS_BASE_KG, delta=0.002)


if __name__ == "__main__":
    unittest.main()
