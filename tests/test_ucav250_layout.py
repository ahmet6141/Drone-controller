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
        self.assertGreaterEqual(self.res["n"], 62)

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

    def test_assembly_fasteners_match_the_layout(self):
        """Fix round 3 (PK3-11): fastener sizes and counts quoted in the assembly steps equal the layout - the bridle
        riser fittings (base / frame bolt groups of F-RISER-FWD / -AFT), the chine-longeron splices (M-CHINE.splices)
        and the firewall shield thickness (layout.stations FS3670 shield_t)."""
        fit = {f["id"]: f for f in self.L["chassis"]["fittings"]}
        steps = {s_["step"]: s_["text"] + " " + " ".join(map(str, s_["checks"])) for s_ in self.A["steps"]}
        for fid in ("F-RISER-FWD", "F-RISER-AFT"):
            groups = {}
            for b_ in fit[fid]["bolts"]:
                groups.setdefault(b_.get("group"), []).append(round(float(b_["d"]) * 1000))
            for g, ds in groups.items():
                txt = f"{len(ds)} x M{max(ds)}"
                self.assertTrue(any(txt in t for t in steps.values()), f"{fid} {g}: '{txt}' not in the assembly steps")
        bad = [m.group(0) for t in steps.values() for m in re.finditer(r"(\d+) x M(\d+) 12[,.]9", t)
               if (int(m.group(1)), int(m.group(2))) not in
               {(len([b_ for b_ in fit[f]["bolts"] if b_.get("group") == g]), d)
                for f in ("F-RISER-FWD", "F-RISER-AFT") for g in {b_.get("group") for b_ in fit[f]["bolts"]}
                for d in {round(float(b_["d"]) * 1000) for b_ in fit[f]["bolts"] if b_.get("group") == g}}]
        self.assertEqual(bad, [], "12.9 bolt groups quoted in the steps that no riser fitting has")
        chine = next(m for m in self.L["chassis"]["members"] if m["id"] == "M-CHINE")
        for sp in chine.get("splices", []):
            txt = f"{len(sp['bolts'])} x M{round(float(sp['d']) * 1000)}"
            self.assertTrue(any(txt in t for t in steps.values()), f"{sp['id']}: '{txt}' not in the assembly steps")
        fw = next(s_ for s_ in self.L["stations"] if s_["id"] == "FS3670")
        sh = f"{float(fw['shield_t']) * 1000:.1f}".replace(".", ",") + " mm"
        self.assertTrue(any(sh in t for t in steps.values()), f"shield thickness {sh} not in the assembly steps")
        self.assertFalse(any("0,5 mm AISI" in t for t in steps.values()))

    def test_one_nose_door_drive(self):
        """Fix round 2 (PK2-13 + mass closure): one DA 22 drives both nose clamshell doors through a centre-line
        bellcrank whose swept envelope is a layout object; the door-drive mass rule counts 3 actuators + the bellcrank."""
        eq = [e for e in self.L["systems"]["equipment"] if e["id"].startswith("EQ-NDOORACT")]
        self.assertEqual([e["id"] for e in eq], ["EQ-NDOORACT"])
        self.assertIn("drive_envelope", eq[0])
        gd = self.S["mass"]["rules"]["gear_doors"]
        self.assertEqual(int(gd["inner_door_actuator"]["count"]), 3)
        self.assertGreater(float(gd["nose_door_drive"]["bellcrank_links_kg"]), 0.0)

    def test_heat_protection_booked_in_mass(self):
        """Mass closure of PK2-09: every heat-protection part carries its area and net mass, and the engine cooling /
        fire-protection item is the bottom-up sum: firewall layer + composite allowance (baseline 1 m2 minus the cowl
        skin booked in the shell) + heat protection (sizing.cooling_installation_mass)."""
        from ucav250.analysis import sizing as Z
        hp = self.L["heat_protection"]
        self.assertGreater(float(hp["mass"]["total_net_kg"]), 0.1)
        for h in hp["inserts"] + hp["shields"]:
            self.assertGreater(float(h["area_m2"]), 0.0, h["id"])
        m, basis = Z.cooling_installation_mass(self.S)
        fw = float(self.L["chassis"]["engine_mount"]["firewall_stackup"]["mass"]["total_kg"])
        cs = self.S["mass"]["rules"]["cooling_split"]
        self.assertAlmostEqual(cs["baffles_plenum_ducts_lip_kg"],
                               cs["baseline_areal_kg_m2"] * (cs["baseline_composite_area_m2"] - cs["cowl_skin_area_m2"]),
                               places=3)
        self.assertAlmostEqual(m, fw + float(cs["baffles_plenum_ducts_lip_kg"]) + float(hp["mass"]["total_net_kg"]),
                               places=6)
        it = next(i for i in self.S["mass"]["items"] if i["name"] == "cooling_baffles_firewall_cowl_flap")
        self.assertAlmostEqual(float(it["mass_base_kg"]), m, delta=1e-4)
        self.assertIn("heat protection", it["basis"])

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
class TestLayoutCheckReadOnly(unittest.TestCase):
    """--check without --out never writes the tracked outputs (fix round 1, S1-10 / VPK-14) and the written report
    carries no run time."""

    def test_check_mode_writes_nothing(self):
        before = {p: p.read_bytes() for p in tracked_outputs()}
        self.assertEqual(LC.main(["--check"]), 0)
        self.assertEqual({p: p.read_bytes() for p in tracked_outputs()}, before)
        md = (REPO / "ucav250" / "out" / "layout.md").read_text(encoding="utf-8")
        self.assertNotIn("süre", md.split("## 1.")[0])
        self.assertNotIn('"elapsed_s"', (REPO / "ucav250" / "out" / "layout.json").read_text(encoding="utf-8"))


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

    # ---------------------------------------------------------------- fix round 3 (PK3-*): new checks are not vacuous
    def test_harness_corridor_violation_detected(self):
        """PK3-05: the well keel webs back at y +-0.0105 leave the main trunk 0.5 mm per side (< the 5 mm corridor)."""
        def edit(S):
            m = next(m for m in S["layout"]["chassis"]["members"] if m["id"] == "M-WELLKEEL")
            m["box"][0][1], m["box"][1][1] = 0.0105, 0.0115
        ctx = self._ctx(edit)
        rows, _ = LC.check_overlaps(ctx, LC.layout_objects(ctx))
        self.assertTrue(any("harness corridors" in r["item"] for r in self._failed(rows, "C04")))

    def test_cell_without_top_vent_detected(self):
        """PK3-01: the forward cell without its float vent valve fails the plumbing row."""
        def edit(S):
            for fl in S["layout"]["fuel_lines"]:
                fl["valves"] = [v for v in fl.get("valves", []) if v["id"] != "FV-F"]
        rows = LC.check_stations(self._ctx(edit))
        self.assertTrue(any("fuel system plumbing" in r["item"] for r in self._failed(rows, "C02")))

    def test_removable_panel_overlap_detected(self):
        """PK3-02: the parachute hatch run 25 mm aft over the spine strip is flagged although both use FS1810."""
        def edit(S):
            p = next(p for p in S["layout"]["shell"]["panels"] if p["id"] == "P-PARAHATCH")
            p["x"][1] = round(p["x"][1] + 0.025, 4)
        rows = LC.panel_overlap_rows(self._ctx(edit))
        self.assertFalse(rows[0]["ok"])

    def test_cradle_pad_on_hatch_detected(self):
        """PK3-06: a cradle pad moved inboard onto the aft equipment hatch is flagged."""
        def edit(S):
            pd = next(p for p in S["layout"]["chassis"]["ground_handling"]["cradle_pads"] if p["id"] == "CR-FSGEAR")
            pd["y"] = [0.0, 0.10]
        self.assertFalse(LC.cradle_pad_row(self._ctx(edit))["ok"])

    def test_material_process_mismatch_detected(self):
        """PK3-08: a CFRP ring insert with the sheet-metal process (the round-2 state) is flagged."""
        def edit(S):
            p = next(p for p in S["layout"]["shell"]["panels"] if p["id"] == "P-TURRETRING")
            p["process"] = "sheet_metal_aluminium"
        self.assertFalse(LC.material_process_row(self._ctx(edit))["ok"])

    def test_plume_through_propeller_detected(self):
        """PK3-10: the round-2 exhaust aim (aft-outboard-down) puts the plume cone through the blade tips."""
        def edit(S):
            for k in S["layout"]["keep_outs"]:
                if k["id"].startswith("KO-EXHAUST"):
                    sg = 1.0 if k["exit"]["point"][1] > 0 else -1.0
                    k["exit"]["direction"] = [0.9082, sg * 0.3179, -0.2724]
        ctx = self._ctx(edit)
        kx = [k for k in ctx.L["keep_outs"] if k["id"].startswith("KO-EXHAUST")]
        self.assertFalse(LC.prop_plume_row(ctx, kx)["ok"])

    def test_missing_longeron_notch_detected(self):
        """PK3-04: a frame the chine longeron crosses without its notch is flagged."""
        def edit(S):
            st = next(s_ for s_ in S["layout"]["stations"] if s_["id"] == "FS1490")
            st["cutouts"] = [c for c in st["cutouts"] if c["id"] != "C-CHINE"]
        self.assertFalse(LC.longeron_notch_row(self._ctx(edit))["ok"])

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

    def test_access_panel_off_its_surface_detected(self):
        """C09: a glove access panel whose outline reaches ahead of the LERX leading edge is flagged."""
        def edit(S):
            p = next(q for q in S["layout"]["shell"]["panels"] if q["id"] == "P-JOINTACCESS")
            p["outline"] = [[c[0] - 0.08, c[1]] for c in p["outline"]]
        rows = LC.check_shell(self._ctx(edit))
        self.assertTrue(any("on their surface" in r["item"] for r in self._failed(rows, "C09")))

    def test_side_bay_panel_on_the_chine_detected(self):
        """C09: a body access panel reaching the chine (no land on the chine longeron) is flagged."""
        def edit(S):
            p = next(q for q in S["layout"]["shell"]["panels"] if q["id"] == "P-SIDEBAY-R")
            p["y"] = [0.05, 0.235]
        rows = LC.check_shell(self._ctx(edit))
        self.assertTrue(any("on their surface" in r["item"] for r in self._failed(rows, "C09")))

    def test_sweep_keep_out_range_mismatch_detected(self):
        """C12: a swept-volume keep-out whose range differs from its joint is flagged."""
        def edit(S):
            k = next(q for q in S["layout"]["keep_outs"] if q["id"] == "KO-SWEEP-MAINGEAR")
            k["joints"]["main_gear_R"] = [0.0, 90.0]
        rows = LC.check_mech_defs(self._ctx(edit))
        self.assertTrue(any("keep-outs" in r["item"] for r in self._failed(rows, "C12")))

    def test_transport_length_over_r29_detected(self):
        """C11: an outer-panel transport length (with the tongue) above R-29 is flagged."""
        def edit(S):
            u = next(q for q in S["assembly"]["transport"]["units"] if q["unit"].startswith("outer wing panel"))
            u["size_m"] = [3.6] + list(u["size_m"][1:])
        rows = LC.check_assembly(self._ctx(edit))
        self.assertTrue(any("transport" in r["item"] for r in self._failed(rows, "C11")))

    def test_fitting_bolt_edge_detected(self):
        """C04: a fitting envelope too small for its declared bolt pattern (edge < 2 D) is flagged (VPK-07)."""
        def edit(S):
            f = next(q for q in S["layout"]["chassis"]["fittings"] if q["id"] == "F-RISER-FWD")
            b = f["bolts"][0]
            f["box"] = [list(f["box"][0]), [f["box"][1][0], float(b["point"][1]) + 0.003, f["box"][1][2]]]
            f.pop("boxes", None)
        self.assertFalse(LC.fitting_bolt_row(self._ctx(edit))["ok"])

    def test_structure_structure_overlap_detected(self):
        """C04: two structural fittings occupying the same space are flagged (structure pairs, VPK-06)."""
        def edit(S):
            F = {q["id"]: q for q in S["layout"]["chassis"]["fittings"]}
            F["F-UPLOCK"]["box"] = copy.deepcopy(F["F-TRUNNION"]["box"])
            F["F-UPLOCK"].pop("boxes", None)
        ctx = self._ctx(edit)
        rows, _ = LC.check_overlaps(ctx, LC.layout_objects(ctx))
        self.assertTrue(self._failed(rows, "C04"))

    def test_panel_edge_without_land_detected(self):
        """C09: a removable panel edge moved off its frame land is flagged (VPK-04 / VPK-12)."""
        def edit(S):
            p = next(q for q in S["layout"]["shell"]["panels"] if q["id"] == "P-MBHATCH")
            p["x"] = [float(p["x"][0]) + 0.05, float(p["x"][1])]
        rows = LC.check_lands(self._ctx(edit))
        self.assertTrue(self._failed(rows, "C09"))

    def test_fixed_surface_through_removable_panel_detected(self):
        """C09: an upper cowl without the cut-outs around the fin roots is crossed by the root cut lines (VPK-05)."""
        def edit(S):
            p = next(q for q in S["layout"]["shell"]["panels"] if q["id"] == "P-COWL-UP")
            x0, x1 = (float(v) for v in p["x"])
            y0, y1 = (float(v) for v in p["y"])
            p["outline"] = [[x0, y0], [x1, y0], [x1, y1], [x0, y1]]
        rows = LC.check_root_lines(self._ctx(edit))
        self.assertTrue(self._failed(rows, "C09"))

    def test_access_opening_too_small_detected(self):
        """C10: a parachute hatch narrower than the container is flagged (clear opening = panel - 2 x land, VPK-03)."""
        def edit(S):
            p = next(q for q in S["layout"]["shell"]["panels"] if q["id"] == "P-PARAHATCH")
            p["y"] = [-0.08, 0.08]
        rows = LC.check_access(self._ctx(edit))
        self.assertTrue(self._failed(rows, "C10"))

    # ---------------------------------------------------------------- fix round 2 (PK2-*): the new checks are not vacuous
    def test_antenna_in_the_skin_detected(self):
        """C03 (PK2-01): an antenna raised 4 mm toward the nose-cone skin is caught by the true OML distance."""
        def edit(S):
            a = next(q for q in S["layout"]["systems"]["antennas"] if q["id"] == "ANT-FTS")
            a["point"] = [a["point"][0], a["point"][1], float(a["point"][2]) + 0.004]
        rows = LC.check_inside(self._ctx(edit))
        self.assertTrue(any("antenna" in r["item"] for r in self._failed(rows, "C03")))

    def test_cutout_corner_in_the_edge_band_detected(self):
        """C02 (PK2-02): a cut-out whose corner reaches into the 20 mm frame edge band is flagged (not only its centre)."""
        def edit(S):
            st = next(s_ for s_ in S["layout"]["stations"] if s_["id"] == "FS0600")
            c = next(q for q in st["cutouts"] if q["id"] == "C-HARN-FWD")
            c["y"] = [float(c["y"][0]), float(c["y"][1]) + 0.06]
        rows = LC.check_stations(self._ctx(edit))
        self.assertTrue(any("corners" in r["item"] for r in self._failed(rows, "C02")))

    def test_gear_leg_against_the_beam_detected(self):
        """C05 (PK2-03): the main leg is swept against every static object - a gear beam moved inboard is hit."""
        def edit(S):
            m = next(q for q in S["layout"]["chassis"]["members"] if q["id"] == "M-GEARBEAM")
            for k in ("box", "boxes"):
                if k in m:
                    m[k] = [[[c[0], c[1] - 0.030, c[2]] for c in b] for b in m[k]] if k == "boxes" else \
                        [[c[0], c[1] - 0.030, c[2]] for c in m[k]]
        ctx = self._ctx(edit)
        rows = LC.check_mechanisms(ctx, LC.layout_objects(ctx))
        self.assertTrue(any("main leg" in r["item"] or "main tyre" in r["item"] for r in self._failed(rows, "C05")))

    def test_unplaced_mass_item_detected(self):
        """C06 (PK2-07): a mass item whose hardware the layout places must appear in layout.mass_placement."""
        def edit(S):
            S["layout"]["mass_placement"].pop("actuators_ailerons_2x_DA26")
        rows, _ = LC.check_cg(self._ctx(edit))
        self.assertTrue(any("mass_item" in r["item"] for r in self._failed(rows, "C06")))

    def test_composite_cowl_piece_in_the_hot_zone_detected(self):
        """C08 (PK2-09): the upper cowl side piece in CFRP is within 25 mm of the cylinder heads."""
        def edit(S):
            p = next(q for q in S["layout"]["shell"]["panels"] if q["id"] == "P-COWL-UPS")
            p["material"], p["layup"] = "cfrp_pw_mtm45_as4", "shell_secondary"
        ctx = self._ctx(edit)
        rows = LC.check_heat(ctx, LC.layout_objects(ctx), *self._hot(ctx))
        self.assertTrue(any("cylinder-head" in r["item"] for r in self._failed(rows, "C08")))

    def test_unshielded_stub_detected(self):
        """C08 (PK2-09): without its heat shield the CFRP stabilator stub is inside the 50 mm exhaust margin."""
        def edit(S):
            hp = S["layout"]["heat_protection"]
            hp["shields"] = [h for h in hp["shields"] if h["id"] != "HS-STUB"]
        ctx = self._ctx(edit)
        rows = LC.check_heat(ctx, LC.layout_objects(ctx), *self._hot(ctx))
        self.assertTrue(any("exhaust" in r["item"] for r in self._failed(rows, "C08")))

    def test_nose_tyre_against_the_full_deck_detected(self):
        """C05 (fix round 2 re-closure): with the full 6.8 mm sandwich deck over the keel slot (no solid strip) the
        stowed nose tyre is closer to the avionics deck than the 12 mm tyre-to-well clearance."""
        def edit(S):
            m = next(q for q in S["layout"]["chassis"]["members"] if q["id"] == "M-DECK-NOSE")
            m.pop("boxes", None)
        ctx = self._ctx(edit)
        rows = LC.check_mechanisms(ctx, LC.layout_objects(ctx))
        self.assertTrue(any("nose tyre" in r["item"] for r in self._failed(rows, "C05")))

    def test_nose_door_linkage_in_the_tyre_path_detected(self):
        """C05 (PK2-13): the nose-door bellcrank / link envelope moved 30 mm forward into the stowed tyre is flagged."""
        def edit(S):
            e = next(q for q in S["layout"]["systems"]["equipment"] if q["id"] == "EQ-NDOORACT")
            b = e["drive_envelope"]["box"]
            e["drive_envelope"]["box"] = [[b[0][0] - 0.03, b[0][1], b[0][2]], [b[1][0] - 0.03, b[1][1], b[1][2]]]
        ctx = self._ctx(edit)
        rows = LC.check_mechanisms(ctx, LC.layout_objects(ctx))
        self.assertTrue(any("nose tyre" in r["item"] for r in self._failed(rows, "C05")))

    @staticmethod
    def _hot(ctx):
        objs = LC.layout_objects(ctx)
        eng = next(o for o in objs if o.id == "ENGINE")
        return LC.OBB(eng.prims[1].c, eng.prims[1].R, eng.prims[1].h), [o for o in objs if o.kind == "exhaust"]

    def test_outer_panel_transport_envelope_detected(self):
        """C11 (PK2-12): an outer-panel transport chord below the loft's chord-wise extent is flagged."""
        def edit(S):
            u = next(q for q in S["assembly"]["transport"]["units"] if q["unit"].startswith("outer wing panel"))
            u["size_m"] = [u["size_m"][0], 0.58, 0.09]
        rows = LC.check_assembly(self._ctx(edit))
        self.assertTrue(any("loft extents" in r["item"] for r in self._failed(rows, "C11")))

    def test_unknown_sequence_joint_detected(self):
        def edit(S):
            S["layout"]["mechanisms"]["sequences"]["gear_retraction"]["states"][3]["no_such_joint"] = 0.1
        rows = LC.check_mech_defs(self._ctx(edit))
        self.assertTrue(self._failed(rows, "C12"))


@unittest.skipUnless(HAVE, "ucav250 dependencies missing")
class TestLayoutBuild(unittest.TestCase):
    """The layout generator package reproduces spec.layout / spec.assembly byte for byte (nothing written), has no
    scratch paths or probe scripts, and its interface decisions are consistent."""

    def test_regenerated_spec_identical(self):
        from ucav250.layout_build import build as B
        new = B.render(B.generated_spec())
        self.assertEqual(new, SPEC_FILE.read_text(encoding="utf-8"),
                         "spec.yaml differs from `python3 -m ucav250.layout_build.build --write`")

    def test_package_clean(self):
        root = REPO / "ucav250" / "layout_build"
        mods = sorted(p.name for p in root.glob("*.py"))
        self.assertEqual(mods, ["__init__.py", "b_assembly.py", "b_chassis.py", "b_common.py", "b_loads.py",
                                "b_mech.py", "b_shell.py", "b_stations.py", "b_systems.py", "build.py"])
        for p in root.glob("*.py"):
            text = p.read_text(encoding="utf-8")
            self.assertNotIn("/tmp/", text, p.name)
            self.assertNotIn("sys.path.insert", text, p.name)
            self.assertIsNone(re.search(r"^from b_\w+ import|^import b_\w+", text, re.M), p.name)

    def test_turret_door_drive_consistent(self):
        S = load_spec()
        eq = {e["id"]: e for e in S["layout"]["systems"]["equipment"]}
        self.assertIn("EQ-TDOORACT", eq)
        texts = [S["payload"]["turret"]["bay"]["doors"], S["layout"]["chassis"]["turret_elevator"]["doors"]]
        texts += [j.get("notes", "") for j in S["layout"]["mechanisms"]["joints"] if j["name"].startswith("turret_door")]
        for tx in texts:
            self.assertIn("DA 22", tx)
            self.assertNotIn("cam-slot", tx)
        acc = next(m for m in S["assembly"]["maintenance_access"] if m["item"].startswith("turret elevator"))
        self.assertIn("P-TDOORACC", acc["access"])

    def test_turret_stroke_keeps_field_of_regard(self):
        """The extended ball centre stays at least as low as in the sizing phase (z -0.226 m) so R-25 holds."""
        S = load_spec()
        T = S["payload"]["turret"]
        self.assertLessEqual(float(T["ball_center_extended_z"]), -0.226 + 1e-6)
        self.assertAlmostEqual(float(T["ball_center_retracted_z"]) - float(T["stroke"]),
                               float(T["ball_center_extended_z"]), delta=0.003)
        el = S["layout"]["chassis"]["turret_elevator"]
        self.assertAlmostEqual(float(el["stroke"]), float(T["stroke"]), places=9)
        j = next(q for q in S["layout"]["mechanisms"]["joints"] if q["name"] == "turret_elevator")
        self.assertAlmostEqual(float(j["hi"]), float(T["stroke"]), places=6)


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

    def test_doc03_tables_match_spec(self):
        """Fix round 1, VPK-14: the station and panel tables of doc 03 carry the spec x values (to 0.5 mm), so the
        document cannot silently fall behind a re-closure of the layout."""
        doc = DOC03.read_text(encoding="utf-8")
        S = load_spec()
        rows = {}
        for line in doc.splitlines():
            if line.startswith("| "):
                cells = [c.strip() for c in line.strip().strip("|").split("|")]
                rows.setdefault(cells[0].replace(" (L/R)", ""), cells)

        def num(s_):
            return float(s_.replace(",", "."))
        for s_ in S["layout"]["stations"]:
            self.assertIn(s_["id"], rows, s_["id"])
            m = re.match(r"\d+,\d+", rows[s_["id"]][1])
            self.assertIsNotNone(m, s_["id"])
            self.assertAlmostEqual(num(m.group(0)), float(s_["x"]), delta=5e-4, msg=s_["id"])
        for p in S["layout"]["shell"]["panels"]:
            self.assertIn(p["id"], rows, p["id"])
            m = re.match(r"(\d+,\d+)–(\d+,\d+)$", rows[p["id"]][3])
            self.assertIsNotNone(m, p["id"])
            self.assertAlmostEqual(num(m.group(1)), float(p["x"][0]), delta=5e-4, msg=p["id"])
            self.assertAlmostEqual(num(m.group(2)), float(p["x"][1]), delta=5e-4, msg=p["id"])


if __name__ == "__main__":
    unittest.main()
