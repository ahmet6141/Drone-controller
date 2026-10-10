"""ucav250 fuel producer (design/fuel.py): three bladder cells with their vulcanised fitting flanges, the plumbing of
layout.fuel_lines (refuel, fill, bottom interconnects, saddle drain, feed, return, vents with the anti-siphon loop,
sump drains), check / float valves, the collector (header tank), level probes, the dry-break coupling, the frame
pass-through seals, the firewall bulkhead block with the shut-off valve, the pump unit with its gascolator, the aft-bay
line support and the skin-end fittings, plus the fuel contents (consumables).

The fuel parts are clean under every design-rule check (mesh, static, swept, clearance, thickness, fastener edge
distance / pierce / pitch, declared contacts, attachment graph); every part honours the producer contract; the
interfaces come from spec.layout (fuel supports, fuel lines with their penetrations, frame cut-outs, equipment); the
contents carry the design fuel load; and the group mass is compared with spec.mass.budget. Nothing here writes to the
repository (checks run with write=False)."""
from __future__ import annotations

import math
import re
import sys
import unittest
from collections import Counter
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
try:
    import yaml

    from ucav250.analysis import checks as K
    from ucav250.core.assemble import build_registry
    from ucav250.design import fuel as FU
    HAVE = True
except Exception:  # pragma: no cover
    HAVE = False

ID_RE = re.compile(r"^YK250-FU-(\d{3})(-[LR])?$")
FORBIDDEN = ("hardpoint", "hard-point", "hard point", "pylon", "weapon", "munition", "release mechanism")
STEP, STEP_LOAD = 18, 38


def _seg_dist(P: np.ndarray, q: np.ndarray) -> float:
    best = math.inf
    for a, b in zip(P[:-1], P[1:]):
        d = b - a
        t = float(np.clip((q - a) @ d / max(float(d @ d), 1e-18), 0.0, 1.0))
        best = min(best, float(np.linalg.norm(a + t * d - q)))
    return best


@unittest.skipUnless(HAVE, "ucav250 dependencies missing")
class TestFuelBuild(unittest.TestCase):
    """One registry for the class: chassis, fuel registered twice (the second call is ignored), hardware."""

    @classmethod
    def setUpClass(cls):
        cls.reg = build_registry(modules=["chassis", "fuel", "fuel", "hardware"], strict=False)
        cls.spec = cls.reg.spec
        cls.L = cls.spec["layout"]
        cls.fuel = {pid: p for pid, p in cls.reg.parts.items() if ID_RE.match(pid)}
        cls.hard = {pid: p for pid, p in cls.fuel.items() if p.process != "consumable"}
        cls.contents = {pid: p for pid, p in cls.fuel.items() if p.process == "consumable"}
        cls.summary = K.run_all(cls.reg, quick=False, write=False, focus=set(cls.fuel))
        cls.F = FU.Fuel(cls.reg, cls.spec)          # geometry helpers only (no registration)
        cls.F.plan_lines()

    # ------------------------------------------------------------------ checks
    def test_design_rule_checks_clean(self):
        bad = {k: v[:3] for k, v in self.summary["details"].items() if v}
        self.assertTrue(self.summary["ok"], f"fuel violations: {bad}")
        for k in ("mesh", "static", "swept", "clearance", "thickness", "fastener", "contact", "attachment"):
            self.assertIn(k, self.summary["violations"], k)
            self.assertEqual(self.summary["violations"][k], 0, k)

    def test_registered_once(self):
        self.assertIn("fuel: already registered, second call ignored", self.reg.log)

    # ------------------------------------------------------------------ contract
    def test_ids_in_fuel_range_and_mirrored(self):
        lo, hi = self.L["part_numbers"]["fuel"]
        self.assertGreaterEqual(len(self.fuel), 49)
        self.assertEqual({p.group for p in self.reg.parts.values() if p.id.startswith("YK250-FU-")}, {"fuel"})
        for pid, p in self.fuel.items():
            m = ID_RE.match(pid)
            self.assertTrue(lo <= int(m.group(1)) <= hi, pid)
            self.assertEqual(p.group, "fuel", pid)
            if m.group(2):
                other = pid[:-1] + ("L" if pid.endswith("R") else "R")
                self.assertIn(other, self.fuel, f"{pid} has no mirror")
                self.assertEqual(p.side, m.group(2)[1], pid)
            else:
                self.assertEqual(p.side, "C", pid)

    def test_layout_fixed_ids(self):
        """Every fuel part id the layout fixes (cells, lines, pump, shut-off valve) exists, once, with its role."""
        fixed = []
        for fs in self.L["chassis"]["fuel_supports"]:
            fixed.append(fs["parts"]["cell"])
            self.assertIn("cell", self.fuel[fs["parts"]["cell"]].name)
            self.assertEqual(self.fuel[fs["parts"]["cell"]].parent, fs["parts"]["liner"])
        for ln in self.L["fuel_lines"]:
            fixed.append(ln["part"])
            ids = [ln["part"] + "-R", ln["part"] + "-L"] if ln.get("mirror") else [ln["part"]]
            for pid in ids:
                self.assertIn(pid, self.fuel, ln["id"])
                self.assertEqual(self.fuel[pid].name, ln["name"], pid)
        for e in self.L["systems"]["equipment"]:
            if e.get("group") == "fuel":
                fixed.append(e["part"])
                self.assertIn(e["part"], self.fuel, e["id"])
                self.assertAlmostEqual(self.fuel[e["part"]].mass_kg, float(e["mass_kg"]), places=6)
        dup = [k for k, n in Counter(fixed).items() if n > 1]
        self.assertEqual(dup, [], "layout fuel part ids must be unique")

    def test_part_contract(self):
        mats, procs = self.spec["materials"], self.spec["processes"]
        steps = {int(s["step"]) for s in self.spec["assembly"]["steps"]}
        for pid, p in self.fuel.items():
            self.assertTrue(p.name and p.name_tr, pid)
            if p.process == "consumable":
                self.assertEqual(p.step, STEP_LOAD, pid)
                self.assertIsNotNone(p.mass_kg, pid)
            elif p.purchased:
                self.assertTrue(p.vendor, pid)
                self.assertTrue(p.mass_kg, pid)
                self.assertEqual(p.step, STEP, pid)
            else:
                self.assertIn(p.material, mats, pid)
                self.assertIn(p.process, procs, pid)
                self.assertIsNotNone(p.thickness, pid)
                self.assertGreaterEqual(p.thickness, float(procs[p.process]["min_thickness"]) - 1e-9, pid)
                self.assertEqual(p.step, STEP, pid)
            self.assertIn(p.step, steps, pid)
            self.assertEqual(len(tuple(p.explode)), 3, pid)
            self.assertGreater(float(np.linalg.norm(p.explode)), 0.0, pid)
            self.assertIn(p.parent, self.reg.parts, f"{pid}: parent {p.parent}")
            self.assertTrue(p.contacts, pid)
            for c in p.contacts:
                self.assertIn(c, self.reg.parts, f"{pid}: contact {c}")

    def test_no_weapon_wording(self):
        for pid, p in self.fuel.items():
            text = " ".join([p.name, p.name_tr, p.notes, p.vendor]).lower()
            for w in FORBIDDEN:
                self.assertNotIn(w, text, pid)
        src = Path(FU.__file__).read_text(encoding="utf-8").lower()
        for w in FORBIDDEN:
            self.assertNotIn(w, src)

    def test_meshes_closed_and_mirror_equal(self):
        for pid, p in self.fuel.items():
            self.assertGreater(p.mesh.volume(), 0.0, pid)
            if pid.endswith("-R"):
                vr, vl = p.mesh.volume(), self.fuel[pid[:-1] + "L"].mesh.volume()
                self.assertAlmostEqual(vr, vl, delta=1e-3 * vr + 1e-12, msg=pid)

    # ------------------------------------------------------------------ interfaces from spec.layout
    def test_lines_pass_the_declared_penetrations(self):
        """Each line passes through its declared deck / well-roof penetration points (sealed bulkhead unions)."""
        n = 0
        for ln in self.L["fuel_lines"]:
            Q = self.F.lines[ln["id"]].centerline()
            for pen in ln.get("penetrations") or []:
                self.assertLess(_seg_dist(Q, np.asarray(pen["point"], float)), 0.0005, f"{ln['id']} {pen['member']}")
                n += 1
        self.assertGreaterEqual(n, 4)

    def test_lines_cross_frames_inside_declared_cutouts(self):
        """Wherever a line crosses the mid-plane of a frame / bulkhead web, it lies inside one of that station's
        declared cut-outs with at least 2 mm of seal material round it (ring frames have an open centre)."""
        n = 0
        for ln in self.L["fuel_lines"]:
            line = self.F.lines[ln["id"]]
            Q = line.centerline()
            for st in self.L["stations"]:
                if st.get("type") == "ring":
                    continue
                xf = st.get("x_faces") or [st["x"] - 0.5 * st["t"], st["x"] + 0.5 * st["t"]]
                k = math.tan(math.radians(float(st.get("sweep_deg", 0.0))))
                xm = 0.5 * (float(xf[0]) + float(xf[1]))
                p, _i = FU.plane_cross(Q, lambda q: q[0] - (xm + abs(q[1]) * k))
                if p is None:
                    continue
                margins = []
                for c in st.get("cutouts") or []:
                    for mir in ((False, True) if c.get("mirror") else (False,)):
                        y0, y1 = c["y"]
                        if mir:
                            y0, y1 = -y1, -y0
                        margins.append(min(p[1] - y0, y1 - p[1], p[2] - c["z"][0], c["z"][1] - p[2]) - line.ro)
                self.assertTrue(margins and max(margins) >= 0.002, f"{ln['id']} crosses {st['id']} at {p}")
                n += 1
        self.assertGreaterEqual(n, 12)

    def test_equipment_in_layout_boxes(self):
        for eid in ("EQ-FUELPUMP", "EQ-SHUTOFF"):
            e = next(x for x in self.L["systems"]["equipment"] if x["id"] == eid)
            lo, hi = self.fuel[e["part"]].mesh.bounds()
            b = np.asarray(e["box"], float)
            self.assertTrue(np.all(lo >= b[0] - 1e-6) and np.all(hi <= b[1] + 1e-6), eid)

    def test_vent_loop_above_every_cell(self):
        """Anti-siphon loop: the vent line's high point is above the top of every bladder (the highest fuel level)."""
        z_vent = float(self.F.lines["FL-VENT"].centerline()[:, 2].max())
        for key in FU.CELL_KEYS:
            self.assertGreater(z_vent, float(self.F.cell[key].inner_solid().bounds()[1][2]) + 0.02, key)

    def test_aft_bay_hose_spans_supported(self):
        """FL-FEED-2 and FL-RETURN are held by the support bracket bushes: no unsupported span longer than 0.30 m
        between the pump / FS-GEAR seal, the bracket and the firewall block / shut-off valve."""
        x_web = FU.SUPPORT["x_web"] + 0.5 * FU.BRACKET_T
        x_fw = float(next(s for s in self.L["stations"] if s["id"] == "FS3670")["x_faces"][0])
        x_gear = float(next(s for s in self.L["stations"] if s["id"] == "FS-GEAR")["x"])
        f2 = self.F.lines["FL-FEED-2"].path
        for lid, xs in (("FL-FEED-2", (f2[0][0], x_web, f2[-1][0])), ("FL-RETURN", (x_fw, x_web, x_gear))):
            Q = self.F.lines[lid].centerline()
            s = np.r_[0.0, np.cumsum(np.linalg.norm(np.diff(Q, axis=0), axis=1))]
            arc = []
            for x in xs:
                p, i = FU.plane_cross(Q, lambda q, x=x: q[0] - x)
                if p is None:                             # station at a line end (pump outlet / valve inlet)
                    p, i = (Q[0], 0) if abs(Q[0][0] - x) < abs(Q[-1][0] - x) else (Q[-1], len(Q) - 2)
                arc.append(s[i] + float(np.linalg.norm(p - Q[i])))
            spans = np.abs(np.diff(arc))
            self.assertLessEqual(float(spans.max()), 0.30, f"{lid}: spans {spans}")
            self.assertIn(self.F.lines[lid].pid, self.fuel["YK250-FU-616"].contacts)
        self.assertIn("YK250-FU-615", self.fuel["YK250-FU-616"].contacts)

    # ------------------------------------------------------------------ contents
    def test_contents_carry_the_design_fuel_load(self):
        rho = float(self.spec["engine"]["fuel"]["density_kg_per_m3"])
        fuel_kg = float(self.spec["mass"]["fuel_kg"])
        self.assertEqual(len(self.contents), 3)
        self.assertAlmostEqual(sum(p.mass_kg for p in self.contents.values()), fuel_kg, delta=5e-4)
        cells = {fs["parts"]["cell"] for fs in self.L["chassis"]["fuel_supports"]}
        for pid, p in self.contents.items():
            self.assertAlmostEqual(p.mass_kg, p.mesh.volume() * rho, delta=1e-3, msg=pid)
            self.assertIn(p.parent, cells, pid)
        # the heaviest loading case still fits the bladders
        ff = max(float(c.get("fuel_fraction", 1.0)) for c in self.spec["mass"]["cases"])
        v_in = sum(c.inner_solid().volume() for c in self.F.cell.values())
        self.assertGreaterEqual(v_in, ff * fuel_kg / rho)

    def test_fuel_cg_on_the_layout_station(self):
        """Fuel CG of the contents (equal fill fraction) on the layout fuel CG station and on the centre line."""
        M = sum(p.mass_kg for p in self.contents.values())
        cg = sum(p.mass_kg * p.mesh.mass_props(1.0)["cg"] for p in self.contents.values()) / M
        ref = np.asarray(self.spec["mass"]["fuel_cg"], float)
        self.assertLess(abs(cg[0] - ref[0]), 0.01)
        self.assertLess(abs(cg[1]), 0.001)
        self.assertLessEqual(cg[2], ref[2])            # fuel settles below the bay centroid

    # ------------------------------------------------------------------ purchased data
    def test_purchased_data_from_components(self):
        root = Path(FU.__file__).resolve().parents[1]
        comp = yaml.safe_load((root / "data" / "research" / "components.yaml").read_text(encoding="utf-8"))
        items = {}

        def walk(o):
            if isinstance(o, dict):
                if "id" in o:
                    items[o["id"]] = o
                for v in o.values():
                    walk(v)
            elif isinstance(o, list):
                for v in o:
                    walk(v)
        walk(comp)
        self.assertAlmostEqual(FU.T_BLAD, items["atl_ultralight_bladder"]["wall_thickness_m"]["value"]["ultralight"])
        self.assertAlmostEqual(FU.COUPLING["mass"], items["obp_dry_break_an6"]["mass_kg"]["value"])
        self.assertAlmostEqual(FU.FV["mass"], items["rollover_vent_valve"]["mass_kg"]["value"])
        self.assertAlmostEqual(FU.PROBE["mass"], items["gill_micro_level_sensor"]["mass_kg"]["value"])
        self.assertAlmostEqual(FU.GASCOLATOR["mass"], items["andair_gas375"]["mass_kg"]["value"])
        self.assertAlmostEqual(FU.PUMP["mass"], items["limbach_efi_supply_kit"]["mass_kg"]["value"])
        self.assertAlmostEqual(self.fuel["YK250-FU-574"].mass_kg, FU.COUPLING["mass"])
        self.assertAlmostEqual(self.fuel["YK250-FU-617"].mass_kg, FU.GASCOLATOR["mass"])
        for pid in ("YK250-FU-578", "YK250-FU-579", "YK250-FU-595"):
            self.assertAlmostEqual(self.fuel[pid].mass_kg, FU.PROBE["mass"], msg=pid)

    # ------------------------------------------------------------------ fasteners
    def test_fasteners_join_fuel_parts(self):
        fs = [f for f in self.reg.fasteners() if any(pid in self.fuel for pid in f.joins)]
        self.assertGreaterEqual(len(fs), 40)
        for f in fs:
            for pid in f.joins:
                self.assertIn(pid, self.reg.parts, f.id)
            self.assertRegex(f.spec, r"^ISO (4762|7380) M[34]x\d+-A2-70$", f.id)
            self.assertEqual(f.step, STEP, f.id)
            self.assertIn(f"YK250-HW-{f.id}", self.reg.parts)

    # ------------------------------------------------------------------ mass
    def test_mass_against_budget(self):
        """Group mass against spec.mass.budget.fuel (ceiling, tol_kg): the EFI pump unit (layout mass_item
        engine_group_installed) is booked in the propulsion budget and is compared separately."""
        b = self.spec["mass"]["budget"]["fuel"]
        pump = next(e for e in self.L["systems"]["equipment"] if e["id"] == "EQ-FUELPUMP")
        self.assertEqual(pump["mass_item"], "engine_group_installed")
        m = sum(self.reg.mass(p) for p in self.hard.values() if p.id != pump["part"])
        self.assertLessEqual(abs(m - float(b["target_kg"])), float(b["tol_kg"]), f"fuel group {m:.3f} kg")
        self.assertAlmostEqual(self.reg.mass(self.fuel[pump["part"]]), float(pump["mass_kg"]))


if __name__ == "__main__":
    unittest.main()
