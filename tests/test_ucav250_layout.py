"""YK-250 HANCER layout phase: spec.layout / spec.assembly interface definition and ucav250.analysis.layout_check.

The full check set runs once (about 7 s). The CLI test writes the Turkish report and the figures into a scratch
directory and verifies that no tracked file changed. The negative tests corrupt a deep copy of the spec and confirm
that the corresponding check detects the fault (the checks are not vacuous)."""
from __future__ import annotations

import copy
import re
import sys
import tempfile
import unittest
from pathlib import Path

import numpy as np

REPO = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO))
try:
    import yaml
    from ucav250.analysis import layout_check as LC
    from ucav250.core import spec as SPEC
    HAVE = True
except Exception:  # pragma: no cover
    HAVE = False

SPEC_FILE = REPO / "ucav250" / "spec.yaml"
DOC03 = REPO / "ucav250" / "docs" / "03_yerlesim_ve_yapi_konsepti.md"
BANNED = ("weapon", "hardpoint", "hard point", "hard_point", "hard-point", "pylon", "munition", "release", "bomb",
          "missile", "warhead", "silah", "mühimmat", "muhimmat", "sert nokta")
TR_CHARS = set("çğıöşüÇĞİÖŞÜ")


def tracked_outputs() -> list:
    root = REPO / "ucav250"
    return [SPEC_FILE] + sorted((root / "out").glob("layout.*")) + sorted((root / "docs" / "fig").glob("yk250_layout_*"))


def load_spec() -> dict:
    with open(SPEC_FILE, encoding="utf-8") as fh:
        return yaml.safe_load(fh)


@unittest.skipUnless(HAVE, "ucav250 dependencies missing")
class TestLayoutChecks(unittest.TestCase):
    """Every layout check passes on the committed spec."""

    @classmethod
    def setUpClass(cls):
        SPEC.load.cache_clear()
        cls.ctx = LC.Ctx()
        cls.res = LC.run_checks(cls.ctx)

    def test_all_checks_pass(self):
        bad = [f"{r['check']} {r['item']}: {r['value']} / {r['limit']} {r['detail']}" for r in self.res["rows"]
               if not r["ok"]]
        self.assertEqual(bad, [])

    def test_every_check_group_present(self):
        groups = {r["check"] for r in self.res["rows"]}
        self.assertEqual(groups, {f"C{i:02d}" for i in range(1, 14)})
        self.assertGreaterEqual(self.res["n"], 60)

    def test_cg_layout_matches_spec_items(self):
        cg = self.res["cg"]
        self.assertLessEqual(float(np.abs(np.subtract(cg["cg_spec"], cg["cg_layout"])).max()), LC.CG_TOL_TOTAL)

    def test_presizing_margins(self):
        ms = [r["MS"] for r in self.res["presizing"] if r["MS"] is not None]
        self.assertGreaterEqual(len(ms), 8)
        self.assertGreaterEqual(min(ms), 0.0)
        for r in self.res["presizing"]:
            self.assertIn("item_tr", r)
            self.assertIn("basis_tr", r)


@unittest.skipUnless(HAVE, "ucav250 dependencies missing")
class TestLayoutContract(unittest.TestCase):
    """Schema-level properties of spec.layout / spec.assembly that the detail modules rely on."""

    @classmethod
    def setUpClass(cls):
        cls.S = load_spec()
        cls.L = cls.S["layout"]
        cls.A = cls.S["assembly"]

    def test_part_number_ranges_disjoint_and_root(self):
        rng = sorted(tuple(v) for v in self.L["part_numbers"].values())
        for (a0, a1), (b0, b1) in zip(rng, rng[1:]):
            self.assertLess(a1, b0)
        self.assertEqual(self.L["root_part"], "YK250-CH-001")
        ctbox = [m for m in self.L["chassis"]["members"] if m["part"] == self.L["root_part"]]
        self.assertEqual(len(ctbox), 1)

    def test_part_ids_unique_and_well_formed(self):
        parts = [p for _, p, _m in LC._all_parts(self.L)]
        self.assertEqual(len(parts), len(set(parts)))
        for p in parts:
            self.assertRegex(p, r"^YK250-[A-Z]{2}-\d{3}$")

    def test_stations(self):
        st = self.L["stations"]
        xs = [float(s_["x"]) for s_ in st]
        self.assertEqual(xs, sorted(xs))
        fw = [s_ for s_ in st if s_.get("subtype") == "firewall"]
        self.assertEqual(len(fw), 1)
        self.assertAlmostEqual(float(fw[0]["x_faces"][1]), float(self.L["firewall_x"]), places=6)
        for s_ in st:
            self.assertLess(s_["x_faces"][0], s_["x_faces"][1])
            for c in s_.get("cutouts", []):
                self.assertLess(c["y"][0], c["y"][1])
                self.assertLess(c["z"][0], c["z"][1])

    def test_mirrored_objects_defined_once(self):
        for group in (self.L["chassis"]["members"], self.L["chassis"]["fittings"], self.L["shell"]["panels"],
                      self.L["systems"]["equipment"]):
            for o in group:
                if o.get("mirror"):
                    self.assertFalse(re.search(r"-[RL]$", o["part"]), o["id"])

    def test_sequences_reference_registered_joints(self):
        J = {j["name"]: j for j in self.L["mechanisms"]["joints"]}
        for name, sq in self.L["mechanisms"]["sequences"].items():
            self.assertEqual(len(sq["values"]), len(sq["states"]), name)
            self.assertEqual(sq["values"], sorted(sq["values"]), name)
            for st in sq["states"]:
                for k, v in st.items():
                    self.assertIn(k, J, f"{name}: {k}")
                    self.assertGreaterEqual(v, float(J[k]["lo"]) - 1e-9)
                    self.assertLessEqual(v, float(J[k]["hi"]) + 1e-9)

    def test_clearances_list_format(self):
        cl = self.L["clearances"]
        self.assertIsInstance(cl, list)
        J = {j["name"] for j in self.L["mechanisms"]["joints"]}
        for r in cl:
            self.assertTrue({"name", "a", "b", "min_mm"} <= set(r), r)
            self.assertGreater(float(r["min_mm"]), 0.0)
            for jn in r.get("joints", []) or []:
                self.assertIn(jn, J, r["name"])

    def test_removable_panels_carry_no_primary_load(self):
        for p in self.L["shell"]["panels"]:
            if p["attach"] in ("removable", "hinged"):
                self.assertNotEqual(p.get("layup"), "wing_skin_primary", p["id"])

    def test_maintenance_without_primary_structure(self):
        rows = self.A["maintenance_access"]
        self.assertGreaterEqual(len(rows), 20)
        panels = {p["id"]: p for p in self.L["shell"]["panels"]}
        for r in rows:
            self.assertFalse(r["primary_structure_removed"], r["item"])
            for a in r["access"]:
                if a in panels:
                    self.assertIn(panels[a]["attach"], ("removable", "hinged"), a)
            self.assertTrue(TR_CHARS & set(r["item_tr"]) or r["item_tr"] != r["item"], r["item"])

    def test_assembly_steps_and_transport(self):
        steps = self.A["steps"]
        self.assertEqual([s_["step"] for s_ in steps], list(range(1, len(steps) + 1)))
        for s_ in steps:
            for k in ("title_tr", "subassembly", "text", "tools", "checks"):
                self.assertTrue(s_.get(k), f"step {s_['step']}: {k}")
        text = " ".join(s_["text"] + s_["title_tr"] for s_ in steps)
        self.assertTrue(TR_CHARS & set(text))
        R = {r["id"]: r for r in self.S["requirements"]}
        outer = next(u for u in self.A["transport"]["units"] if u["unit"].startswith("outer wing panel"))
        centre = next(u for u in self.A["transport"]["units"] if u["unit"].startswith("centre body"))
        self.assertLessEqual(outer["size_m"][0], 3.4)
        self.assertLessEqual(centre["size_m"][1], 2.0)
        self.assertIn("R-29", R)
        self.assertIn("R-30", R)

    def test_mass_placement_applied(self):
        items = {i["name"]: i for i in self.S["mass"]["items"]}
        for k, v in self.L["mass_placement"].items():
            self.assertIn(k, items)
            p = np.asarray(v["position"], float)
            q = np.array([items[k]["x"], items[k]["y"], items[k]["z"]], float)
            self.assertLessEqual(float(np.abs(p - q).max()), LC.CG_TOL_ITEM, k)
            self.assertIn("position_sizing_phase", v)

    def test_civil_scope_wording(self):
        files = [REPO / "ucav250" / "analysis" / "layout_check.py", DOC03]
        texts = [yaml.safe_dump(self.L, allow_unicode=True), yaml.safe_dump(self.A, allow_unicode=True)]
        texts += [p.read_text(encoding="utf-8") for p in files if p.exists()]
        for t in texts:
            low = t.lower()
            for w in BANNED:
                self.assertNotIn(w, low, w)


@unittest.skipUnless(HAVE, "ucav250 dependencies missing")
class TestLayoutCLI(unittest.TestCase):
    """CLI in --check mode with outputs in a scratch directory: exit 0, Turkish report, five figures, no tracked
    file modified."""

    def test_cli_scratch_outputs(self):
        before = {p: p.read_bytes() for p in tracked_outputs()}
        with tempfile.TemporaryDirectory() as td:
            out, fig = Path(td) / "out", Path(td) / "fig"
            rc = LC.main(["--check", "--out", str(out), "--fig-dir", str(fig)])
            self.assertEqual(rc, 0)
            md = (out / "layout.md").read_text(encoding="utf-8")
            self.assertTrue(md.startswith("# YK-250 HANÇER"))
            self.assertIn("kontrol geçti", md)
            self.assertNotIn("**KALDI**", md)
            for k in ("side", "top", "structure", "shell", "sections"):
                p = fig / f"yk250_layout_{k}.png"
                self.assertTrue(p.exists(), p)
                self.assertGreater(p.stat().st_size, 50_000)
            self.assertTrue((out / "layout.json").exists())
        after = {p: p.read_bytes() for p in tracked_outputs()}
        self.assertEqual(set(before), set(after))
        for p in before:
            self.assertEqual(before[p], after[p], p)


@unittest.skipUnless(HAVE, "ucav250 dependencies missing")
class TestChecksDetectFaults(unittest.TestCase):
    """Each check flags a deliberately corrupted copy of the spec."""

    @classmethod
    def setUpClass(cls):
        SPEC.load.cache_clear()
        cls.S0 = copy.deepcopy(SPEC.load())

    def _ctx(self, edit):
        S = copy.deepcopy(self.S0)
        edit(S)
        return LC.Ctx(S)

    @staticmethod
    def _failed(rows, prefix):
        return [r for r in rows if r["check"] == prefix and not r["ok"]]

    def test_missing_cutout_detected(self):
        def edit(S):
            st = next(s_ for s_ in S["layout"]["stations"] if s_["id"] == "FS-MS")
            st["cutouts"] = [c for c in st["cutouts"] if c["id"] != "C-HARN-MS"]
        rows = LC.check_stations(self._ctx(edit))
        self.assertTrue(any("crossings" in r["item"] for r in self._failed(rows, "C02")))

    def test_battery_near_fuel_detected(self):
        def edit(S):
            eq = next(e for e in S["layout"]["systems"]["equipment"] if e["id"] == "EQ-BUFFER_BATTERY")
            for k in ("box", "envelope_with_connector"):
                if k in eq:
                    eq[k] = [[c[0] + 1.0, c[1], c[2]] for c in eq[k]]
        ctx = self._ctx(edit)
        rows = LC.check_keepouts(ctx, LC.layout_objects(ctx))
        self.assertTrue(any("battery" in r["item"] for r in self._failed(rows, "C08")))

    def test_overlap_detected(self):
        def edit(S):
            E = {e["id"]: e for e in S["layout"]["systems"]["equipment"]}
            E["EQ-DATALINK_BACKUP"]["box"] = copy.deepcopy(E["EQ-AUTOPILOT"]["box"])
            E["EQ-DATALINK_BACKUP"].pop("envelope_with_connector", None)
        ctx = self._ctx(edit)
        rows, _ = LC.check_overlaps(ctx, LC.layout_objects(ctx))
        self.assertTrue(self._failed(rows, "C04"))

    def test_stale_mass_item_detected(self):
        def edit(S):
            it = next(i for i in S["mass"]["items"] if i["name"] == "frames_bulkheads")
            it["x"] = float(it["x"]) + 0.05
        rows, _ = LC.check_cg(self._ctx(edit))
        self.assertTrue(self._failed(rows, "C06"))

    def test_unknown_sequence_joint_detected(self):
        def edit(S):
            S["layout"]["mechanisms"]["sequences"]["gear_retraction"]["states"][3]["no_such_joint"] = 0.1
        rows = LC.check_mech_defs(self._ctx(edit))
        self.assertTrue(self._failed(rows, "C12"))


@unittest.skipUnless(HAVE, "ucav250 dependencies missing")
class TestDoc03(unittest.TestCase):
    """docs/03 exists, is Turkish and quotes the spec (stations, root part, check count)."""

    def test_doc03(self):
        self.assertTrue(DOC03.exists())
        doc = DOC03.read_text(encoding="utf-8")
        self.assertTrue(TR_CHARS & set(doc))
        S = load_spec()
        for s_ in S["layout"]["stations"]:
            self.assertIn(s_["id"], doc)
        self.assertIn(S["layout"]["root_part"], doc)
        for k in ("side", "top", "structure", "shell", "sections"):
            self.assertIn(f"fig/yk250_layout_{k}.png", doc)


if __name__ == "__main__":
    unittest.main()
