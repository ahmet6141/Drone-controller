"""ucav250 propulsion producer (design/propulsion.py): Limbach L 275 EF + SG750 on four isolators in the chassis mount
cups, Mejzlik 31x12 3B pusher propeller with hub spacer, backplate, crush plate and spinner on ``prop_spin``, the two
exhausts inside the KO-EXHAUST routing envelopes, the cooling path (S-duct, firewall transition duct, coupling boot,
engine plenum and cylinder baffles), the throttle actuator, the engine-side fuel hoses, the ECU and the generator power
electronics.

The propulsion parts are clean under every design-rule check (mesh, static, swept, clearance, thickness, fastener edge
distance / pierce / pitch, declared contacts, attachment graph), honour the producer contract, take their interfaces
from spec.layout (mount bolts / isolators, keep-outs, exit point, cooling corridor, equipment boxes, prop joint), keep
the 10 mm engine dynamic margin to the airframe, and the group mass is reported against spec.mass.budget. Nothing here
writes to the repository (checks run with write=False)."""
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
    from ucav250.design import propulsion as P
    HAVE = True
except Exception:  # pragma: no cover
    HAVE = False

ID_RE = re.compile(r"^YK250-PR-(\d{3})(-[LR])?$")
FORBIDDEN = ("hardpoint", "hard-point", "hard point", "pylon", "weapon", "munition", "release mechanism")
PR_STEPS = (20, 25, 26, 27)


def _mine(pid: str) -> bool:
    m = ID_RE.match(pid)
    return bool(m) and 500 <= int(m.group(1)) <= 569


@unittest.skipUnless(HAVE, "ucav250 dependencies missing")
class TestPropulsionBuild(unittest.TestCase):
    """One registry for the class: chassis, propulsion registered twice (the second call is ignored), hardware."""

    @classmethod
    def setUpClass(cls):
        cls.reg = build_registry(modules=["chassis", "propulsion", "propulsion", "hardware"], strict=False)
        cls.spec = cls.reg.spec
        cls.L = cls.spec["layout"]
        cls.pr = {pid: p for pid, p in cls.reg.parts.items() if _mine(pid)}
        focus = {pid for pid in cls.reg.parts if pid.startswith("YK250-PR")}
        cls.summary = K.run_all(cls.reg, quick=False, write=False, focus=focus)
        cls.C = P.Ctx(cls.reg, cls.spec)          # geometry helpers only (no registration)

    # ------------------------------------------------------------------ checks
    def test_design_rule_checks_clean(self):
        bad = {k: v[:3] for k, v in self.summary["details"].items() if v}
        self.assertTrue(self.summary["ok"], f"propulsion violations: {bad}")
        for k in ("mesh", "static", "swept", "clearance", "thickness", "fastener", "contact", "attachment"):
            self.assertIn(k, self.summary["violations"], k)

    def test_registered_once(self):
        self.assertIn("propulsion: already registered, second call ignored", self.reg.log)

    # ------------------------------------------------------------------ contract
    def test_ids_in_range_and_mirrored(self):
        lo, hi = self.L["part_numbers"]["propulsion"]
        self.assertGreater(len(self.pr), 25)
        for pid, p in self.pr.items():
            m = ID_RE.match(pid)
            self.assertTrue(lo <= int(m.group(1)) <= hi, pid)
            self.assertEqual(p.group, "propulsion", pid)
            if m.group(2):
                other = pid[:-1] + ("L" if pid.endswith("R") else "R")
                self.assertIn(other, self.pr, f"{pid} has no mirror")
                self.assertEqual(p.side, m.group(2)[1], pid)
            else:
                self.assertEqual(p.side, "C", pid)

    def test_layout_fixed_ids(self):
        """Interface part ids fixed by spec.layout exist with the right role."""
        for pid in ("YK250-PR-500", "YK250-PR-504-R", "YK250-PR-504-L", "YK250-PR-510", "YK250-PR-512",
                    "YK250-PR-540", "YK250-PR-542", "YK250-PR-546"):
            self.assertIn(pid, self.pr)
        lj = next(j for j in self.L["mechanisms"]["joints"] if j["name"] == "prop_spin")
        for pid in re.findall(r"YK250-PR-\d{3}", lj["moves"]):
            self.assertEqual(self.pr[pid].joint, "prop_spin", pid)
        eq = {e["id"]: e for e in self.L["systems"]["equipment"]}
        for k in ("EQ-ECU", "EQ-GENERATOR_PE"):
            p = self.pr[eq[k]["part"]]
            self.assertAlmostEqual(p.mass_kg, eq[k]["mass_kg"])
            self.assertEqual(p.parent, eq[k]["mount"]["tray"].split()[0])
            lo, hi = p.mesh.bounds()
            blo, bhi = np.asarray(eq[k]["box"][0]), np.asarray(eq[k]["box"][1])
            self.assertTrue(np.all(lo[:2] >= blo[:2] - 1e-6) and np.all(hi <= bhi + 1e-6), k)

    def test_part_contract(self):
        mats, procs = self.spec["materials"], self.spec["processes"]
        steps = {int(s["step"]) for s in self.spec["assembly"]["steps"]}
        for pid, p in self.pr.items():
            self.assertTrue(p.name and p.name_tr, pid)
            if p.purchased:
                self.assertTrue(p.vendor, pid)
                self.assertTrue(p.mass_kg is not None and p.mass_kg > 0, pid)
            else:
                self.assertIn(p.material, mats, pid)
                self.assertIn(p.process, procs, pid)
                self.assertTrue(p.thickness is not None or p.layup, pid)
                t = p.thickness if p.thickness is not None else 1.0
                self.assertGreaterEqual(t, float(procs[p.process]["min_thickness"]) - 1e-9, pid)
            self.assertIn(p.step, steps, pid)
            self.assertIn(p.step, PR_STEPS, pid)
            self.assertEqual(len(tuple(p.explode)), 3, pid)
            self.assertGreater(float(np.linalg.norm(p.explode)), 0.0, pid)
            self.assertIn(p.parent, self.reg.parts, f"{pid}: parent {p.parent}")
            for c in p.contacts:
                self.assertIn(c, self.reg.parts, f"{pid}: contact {c}")

    def test_no_weapon_wording(self):
        for pid, p in self.pr.items():
            text = " ".join([p.name, p.name_tr, p.notes, p.vendor]).lower()
            for w in FORBIDDEN:
                self.assertNotIn(w, text, pid)
        src = Path(P.__file__).read_text(encoding="utf-8").lower()
        for w in FORBIDDEN:
            self.assertNotIn(w, src)

    def test_meshes_closed_and_mirror_equal(self):
        for pid, p in self.pr.items():
            self.assertGreater(p.mesh.volume(), 0.0, pid)
            if pid.endswith("-R"):
                vr, vl = p.base_mesh.volume(), self.pr[pid[:-1] + "L"].base_mesh.volume()
                self.assertAlmostEqual(vr, vl, delta=1e-3 * vr + 1e-12, msg=pid)

    # ------------------------------------------------------------------ engine and mount interface
    def test_engine_on_layout_mount_bolts(self):
        """Four M8 12.9 bolts on the layout bolt points: cup bottom -> isolator -> crankcase boss (12 mm class
        thread), the isolators on the layout centres inside the chassis cups."""
        em = self.L["chassis"]["engine_mount"]
        bolts = [f for f in self.reg.fasteners() if f.id.startswith("YK250-PR-50") and f.id.endswith("-B1")
                 and em["part"] in f.joins]
        self.assertEqual(len(bolts), 4)
        pts = np.asarray(em["bolts"]["points"], float)
        d = np.asarray(em["thrust_axis"]["direction_aft"], float)
        d /= np.linalg.norm(d)
        for f in bolts:
            self.assertIn("M8", f.spec)
            self.assertIn("12.9", f.spec)
            self.assertEqual(f.joins[0], em["part"])
            self.assertEqual(f.joins[-1], "YK250-PR-500")
            self.assertAlmostEqual(abs(float(np.dot(f.axis, d))), 1.0, places=6)
            off = pts - f.position                            # bolt line through a layout bolt point
            perp = np.linalg.norm(off - np.outer(off @ f.axis, f.axis), axis=1)
            self.assertLess(float(perp.min()), 1e-6)
            self.assertGreaterEqual(f.length - f.grip, 1.2 * f.d)      # thread engagement in the boss
        for k, c in enumerate(em["isolators"]["centres"]):
            m = self.pr[f"YK250-PR-{506 + k}"].mesh
            ax = np.asarray(c, float)
            V = m.V - ax
            rad = np.linalg.norm(V - np.outer(V @ d, d), axis=1)
            self.assertLessEqual(float(rad.max()), 0.0205 + 1e-6)      # inside the cup ID 41

    def test_engine_inside_dynamic_envelope(self):
        """Every engine vertex lies inside the KO-ENGINE boxes grown by the layout margin (isolator travel)."""
        ko = next(k for k in self.L["keep_outs"] if k["id"] == "KO-ENGINE")
        m = float(ko["margin"])
        C = self.C
        L = np.array([C.loc(p) for p in self.pr["YK250-PR-500"].mesh.V])
        inside = np.zeros(len(L), bool)
        for b in ko["boxes"]:
            lo = np.array([b["u"][0], b["v"][0], b["w"][0]]) - m
            hi = np.array([b["u"][1], b["v"][1], b["w"][1]]) + m
            inside |= np.all((L >= lo - 1e-9) & (L <= hi + 1e-9), axis=1)
        self.assertTrue(inside.all(), f"{(~inside).sum()} engine vertices outside KO-ENGINE + margin")

    def _engine_mounted(self, p) -> bool:
        k = 0
        while p is not None and k < 20:
            if p.id == "YK250-PR-500":
                return True
            p = self.reg.parts.get(p.parent) if p.parent else None
            k += 1
        return False

    def test_engine_dynamic_margin(self):
        """Parts carried by the engine keep the layout engine_keep_out margin (10 mm) to the airframe structure;
        the flexible links (hoses, coupling boot) and the isolators are exempt."""
        need = float(self.L["clearance_values"]["engine_keep_out"])
        eng = [p for p in self.pr.values() if self._engine_mounted(p) and p.id not in ("YK250-PR-516",
                                                                                        "YK250-PR-517")]
        others = [p for p in self.reg.parts.values() if p.group == "chassis"]
        self.assertGreater(len(eng), 8)
        mans = {}
        for a in eng:
            ma = a.mesh.to_manifold()
            la, ha = a.mesh.bounds()
            for b in others:
                if b.id in a.contacts or a.id in b.contacts:
                    continue
                lb, hb = b.mesh.bounds()
                if np.any(ha + need < lb) or np.any(hb + need < la):
                    continue
                mans.setdefault(b.id, b.mesh.to_manifold())
                g = float(ma.min_gap(mans[b.id], need * 1.5))
                self.assertGreaterEqual(g, need - 1e-4, f"{a.id} vs {b.id}: {g * 1000:.2f} mm")

    # ------------------------------------------------------------------ exhaust
    def test_exhaust_in_routing_envelope_and_exit(self):
        """Exhaust surface inside KO-EXHAUST-R (+ the port flange inside the cylinder envelope), tail pipe ending on
        the layout exit point along the layout exit direction."""
        ko = next(k for k in self.L["keep_outs"] if k["id"] == "KO-EXHAUST-R")
        boxes = [(np.asarray(a, float), np.asarray(b, float)) for a, b in ko["boxes"]]
        C = self.C
        cyl = next(b for b in next(k for k in self.L["keep_outs"] if k["id"] == "KO-ENGINE")["boxes"]
                   if b["id"] == "cylinders_heads")
        V = self.pr["YK250-PR-504-R"].mesh.V
        for p in V:
            in_box = any(np.all(p >= a - 1e-4) and np.all(p <= b + 1e-4) for a, b in boxes)
            q = C.loc(p)
            in_cyl = (cyl["u"][0] - 1e-4 <= q[0] <= cyl["u"][1] + 1e-4 and abs(q[1]) <= cyl["v"][1] + 1e-4 and
                      cyl["w"][0] - 0.02 <= q[2] <= cyl["w"][1])
            self.assertTrue(in_box or in_cyl, f"exhaust vertex {np.round(p, 4)} outside the routing envelope")
        pa = P.exhaust_paths(C)
        E, e = np.asarray(ko["exit"]["point"], float), np.asarray(ko["exit"]["direction"], float)
        e /= np.linalg.norm(e)
        self.assertLess(float(np.linalg.norm(pa["tail"][-1] - E)), 1e-6)
        t = pa["tail"][-1] - pa["tail"][-2]
        self.assertGreater(float(np.dot(t / np.linalg.norm(t), e)), 0.995)

    def test_exhaust_clear_of_composites(self):
        """The 50 mm unshielded rule against every composite part present (chassis CFRP)."""
        mats = self.spec["materials"]
        comps = [p for p in self.reg.parts.values() if p.group == "chassis" and
                 (p.layup or mats.get(p.material, {}).get("kind") == "composite")]
        for side in ("R", "L"):
            m = self.pr[f"YK250-PR-504-{side}"].mesh.to_manifold()
            for b in comps:
                g = float(m.min_gap(b.mesh.to_manifold(), 0.06))
                self.assertGreaterEqual(g, 0.05 - 1e-4, f"{side} vs {b.id}")

    # ------------------------------------------------------------------ propeller
    def test_propeller_joint_and_planform(self):
        lj = next(j for j in self.L["mechanisms"]["joints"] if j["name"] == "prop_spin")
        j = self.reg.joints["prop_spin"]
        self.assertTrue(np.allclose(j.origin, lj["origin"]))
        self.assertTrue(np.allclose(j.axis, np.asarray(lj["axis"]) / np.linalg.norm(lj["axis"])))
        self.assertAlmostEqual(j.lo, lj["lo"])
        self.assertAlmostEqual(j.hi, lj["hi"])
        self.assertEqual(j.prop, lj["prop"])
        pr = self.spec["propeller"]
        V = self.pr["YK250-PR-540"].mesh.V - j.origin
        ax = V @ j.axis
        rad = np.linalg.norm(V - np.outer(ax, j.axis), axis=1)
        self.assertAlmostEqual(float(rad.max()), 0.5 * pr["diameter"], delta=0.001)
        tip = rad > 0.9 * 0.5 * pr["diameter"]
        self.assertLess(float(np.abs(ax[tip]).max()), pr["clearance_checks"]["blade_tip_axial_half_extent_m"] + 0.002)
        self.assertAlmostEqual(self.pr["YK250-PR-540"].mass_kg, pr["mass_kg"])
        sp = self.pr["YK250-PR-542"].mesh.V - j.origin
        srad = np.linalg.norm(sp - np.outer(sp @ j.axis, j.axis), axis=1)
        self.assertLessEqual(float(srad.max()), 0.5 * pr["spinner"]["diameter"] + 1e-6)

    def test_static_parts_outside_prop_disc(self):
        """KO-PROP: only the rotating group (hub / spinner) enters the propeller disc keep-out."""
        ko = next(k for k in self.L["keep_outs"] if k["id"] == "KO-PROP")
        c, a = np.asarray(ko["centre"], float), np.asarray(ko["axis"], float)
        a /= np.linalg.norm(a)
        for pid, p in self.reg.parts.items():
            if p.joint == "prop_spin":
                continue
            V = p.mesh.V - c
            ax = V @ a
            rad = np.linalg.norm(V - np.outer(ax, a), axis=1)
            inside = (np.abs(ax) <= ko["half_thickness"]) & (rad <= ko["radius"])
            self.assertFalse(inside.any(), pid)

    # ------------------------------------------------------------------ cooling
    def test_sduct_in_corridor_and_through_cutout(self):
        """The S-duct stays inside KO-COOLING-DUCT (three 80 mm tubes, stadium envelope) and exits through C-DUCT;
        only its inlet mouth (first corridor segment, open roof) rises above the corridor, to the inlet lip land
        under the OML (never into the skin)."""
        ko = next(k for k in self.L["keep_outs"] if k["id"] == "KO-COOLING-DUCT")
        Pth = np.asarray(ko["path"], float)
        hy = max(abs(o) for o in ko["lateral_offsets"]) + ko["radius"]
        ex = ko["exit_section"]
        V = self.pr["YK250-PR-546"].mesh.V
        x0 = Pth[0, 0]
        self.assertGreaterEqual(float(V[:, 0].min()), x0 - 1e-6)
        self.assertLessEqual(float(V[:, 0].max()), ex["x"][1] + 1e-6)
        dep = P.inlet_bond_depth()
        for p in V:
            if p[0] <= Pth[1, 0] + 1e-9:                 # inlet mouth
                zc = np.interp(p[0], Pth[:, 0], Pth[:, 2])
                self.assertLessEqual(abs(p[1]), hy + 1e-6)
                self.assertGreaterEqual(p[2], zc - ko["radius"] - 1e-6)
                self.assertLessEqual(p[2], self.C.oml_z_top(p[0], p[1]) - dep + 2e-4)
            elif p[0] <= Pth[-1, 0]:
                zc = np.interp(p[0], Pth[:, 0], Pth[:, 2])
                self.assertLessEqual(abs(p[1]), hy + 1e-6)
                self.assertLessEqual(abs(p[2] - zc), ko["radius"] + 1e-6)
            else:                                     # flattening to the exit section (last 55 mm)
                self.assertLessEqual(abs(p[1]), hy + 1e-6)
                self.assertTrue(min(ex["z"]) - 1e-6 <= p[2] <= Pth[-1, 2] + ko["radius"] + 1e-6)
                if p[0] >= P.DUCT_FLAT_X:
                    self.assertTrue(ex["y"][0] - 1e-6 <= p[1] <= ex["y"][1] + 1e-6)
                    self.assertTrue(min(ex["z"]) - 1e-6 <= p[2] <= max(ex["z"]) + 1e-6)
        # the mouth roof is open: a vertical line from the OML down to the corridor centre meets no duct material
        man = self.pr["YK250-PR-546"].mesh.to_manifold()
        xm = 0.5 * (Pth[0, 0] + Pth[1, 0])
        zt = self.C.oml_z_top(xm, 0.0)
        hits = G.ray_hits(man, np.array([xm, 0.0, zt]), np.array([xm, 0.0, np.interp(xm, Pth[:, 0], Pth[:, 2])]))
        self.assertEqual(len(hits), 0)
        cut = next(c for c in next(s for s in self.L["stations"] if s["id"] == "FS3670")["cutouts"]
                   if c["id"] == "C-DUCT")
        T = self.pr["YK250-PR-548"].mesh
        sl = G.intersection(T, G.box([0.004, 1.0, 1.0], center=(3.660, 0, 0.2)))
        lo, hi = sl.bounds()
        self.assertTrue(cut["y"][0] < lo[1] and hi[1] < cut["y"][1] and cut["z"][0] < lo[2] and hi[2] < cut["z"][1])

    def test_cooling_passage_areas(self):
        """Report the free areas along the cooling path; the narrowest (boot over the mount truss) is >= 60 % of the
        duct exit section (layout: no area requirement, cooling-flow check in this phase -> doc open item)."""
        zc, hy, hz, r = P.inlet_section(self.C)
        boot = 4 * hy * hz - (4 - math.pi) * r * r
        ex = next(k for k in self.L["keep_outs"] if k["id"] == "KO-COOLING-DUCT")["exit_section"]
        self.assertGreater(boot, 0.6 * ex["area_m2"])

    # ------------------------------------------------------------------ fasteners
    def test_fasteners_join_propulsion_parts(self):
        fs = [f for f in self.reg.fasteners() if any(pid in self.pr for pid in f.joins)]
        self.assertGreater(len(fs), 50)
        for f in fs:
            for pid in f.joins:
                self.assertIn(pid, self.reg.parts, f.id)
            self.assertIn(f.step, PR_STEPS, f.id)
        prop = [f for f in fs if f.id.startswith("YK250-PR-544-B")]
        self.assertEqual(len(prop), 6)
        for f in prop:
            self.assertEqual(f.joins[:3], ("YK250-PR-544", "YK250-PR-540", "YK250-PR-543"))

    # ------------------------------------------------------------------ mass
    def test_masses_against_budget(self):
        b = self.spec["mass"]["budget"]["propulsion"]
        m = sum(self.reg.mass(p) for p in self.reg.parts.values() if p.group == "propulsion")
        self.assertLessEqual(m, b["target_kg"])                         # ceiling (mass.budget_rules)
        self.assertGreater(m, 10.0)
        items = self.spec["engine"]["installed_items_kg"]
        eng = self.reg.mass(self.pr["YK250-PR-500"]) + self.reg.mass(self.pr["YK250-PR-501"])
        self.assertAlmostEqual(eng, items["engine_bare"], places=6)
        self.assertAlmostEqual(self.pr["YK250-PR-502"].mass_kg, items["starter_generator_sg750"])

    def test_purchased_data_from_components(self):
        root = Path(P.__file__).resolve().parents[1]
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
        d22 = items["volz_da22_28v"]
        self.assertAlmostEqual(P.DA22["mass"], d22["mass_kg"]["value"])
        self.assertTrue(np.allclose([P.DA22["L"], P.DA22["H"], P.DA22["W"]], d22["case_dimensions_m"]["value"]))
        self.assertAlmostEqual(self.pr["YK250-PR-514"].mass_kg, d22["mass_kg"]["value"])
        sg = items["epropelled_sg750"]
        self.assertAlmostEqual(self.pr["YK250-PR-502"].mass_kg, sg["mass_kg"]["value"])


def _cowl_stand_in(C):
    """Stand-in lower cowl half from spec.layout only (P-COWL-LO extents, shell_secondary 5.8 mm sandwich, 1.6 mm solid
    edge band within 30 mm of the cut-out, the HS-COWL-EXIT insert regions cut out with a 0.5 mm gap) until the shell
    producer registers the real halves."""
    hp = P._hp(C, "inserts", "HS-COWL-EXIT")["regions"]
    pan = P._cowl_lo(C)
    clip = P.box3((pan["x"][0] + 0.0005, pan["y"][0] + 0.0005, -0.3), (4.1, 0.5, pan["z_band"][1] - 0.0005))
    full = P.inter(P._oml_band(C, 0.0, P.COWL_SKIN), clip)
    band = P.inter(P._oml_band(C, 0.0, P.HS["band_t"]), clip)
    near = P._box_union(hp, 0.030)
    m = P.diff(P.union([P.diff(full, [near]), P.inter(band, near)]), [P._box_union(hp, 0.0005)])
    return P.finish(P.largest_piece(m))


@unittest.skipUnless(HAVE, "ucav250 dependencies missing")
class TestHeatProtection(unittest.TestCase):
    """layout.heat_protection HS-COWL-EXIT (PR-520) / HS-COWL-SHIELD (PR-523) ride on the lower cowl halves: they are
    registered only when the halves exist. Verified here on a layout-derived stand-in of YK250-SH-451-R/L (a fixture
    marked consumable so that it hangs on the root part without its own Camloc row)."""

    @classmethod
    def setUpClass(cls):
        from ucav250.core.parts import Part, mirror_part
        from ucav250.design import hardware as HW
        cls.reg = build_registry(modules=["chassis"], strict=False)
        cls.spec = cls.reg.spec
        P._spinner_dims(cls.spec)
        cls.C = P.Ctx(cls.reg, cls.spec)
        hid = P._cowl_lo(cls.C)["part"]
        host = Part(id=hid + "-R", name="lower cowl stand-in, starboard", name_tr="alt kaporta vekili, sağ",
                    group="shell", material="cfrp_pw_mtm45_as4", process="consumable",
                    mesh_fn=lambda C=cls.C: _cowl_stand_in(C), side="R", thickness=P.HS["band_t"],
                    parent=cls.L_root(cls.spec), step=36)
        cls.reg.add(host)
        cls.reg.add(mirror_part(host, hid + "-L"))
        P.register(cls.reg, cls.spec)
        HW.register(cls.reg, cls.spec)
        cls.hosts = {"R": hid + "-R", "L": hid + "-L"}
        focus = {pid for pid in cls.reg.parts if pid.startswith("YK250-PR")}
        cls.summary = K.run_all(cls.reg, quick=False, write=False, focus=focus)

    @staticmethod
    def L_root(spec):
        return spec["layout"]["root_part"]

    def test_registered_on_hosts(self):
        for side in ("R", "L"):
            ins, shd = self.reg.parts[f"YK250-PR-520-{side}"], self.reg.parts[f"YK250-PR-523-{side}"]
            self.assertEqual(ins.parent, self.hosts[side])
            self.assertEqual(shd.parent, ins.id)
            self.assertEqual(ins.material, "ss_304_annealed")
            self.assertGreaterEqual(ins.thickness, float(self.spec["processes"][ins.process]["min_thickness"]))
            self.assertIn(ins.step, {int(s["step"]) for s in self.spec["assembly"]["steps"]})
        self.assertNotIn("YK250-PR-521-R", self.reg.parts)          # not required (doc: >= 50 mm margins)
        self.assertNotIn("YK250-PR-522-R", self.reg.parts)

    def test_design_rule_checks_clean(self):
        bad = {k: v[:3] for k, v in self.summary["details"].items() if v}
        self.assertTrue(self.summary["ok"], f"heat-protection violations: {bad}")

    def test_rivets_pierce_and_join(self):
        man = {pid: self.reg.parts[pid].mesh.to_manifold() for pid in
               ("YK250-PR-520-R", "YK250-PR-523-R", self.hosts["R"])}
        fs = [f for f in self.reg.fasteners() if f.joins and f.joins[0] == "YK250-PR-520-R"]
        lap = [f for f in fs if self.hosts["R"] in f.joins]
        sh = [f for f in fs if "YK250-PR-523-R" in f.joins]
        self.assertGreaterEqual(len(lap), 15)
        self.assertGreaterEqual(len(sh), 10)
        for f in lap + sh:
            for pid in f.joins:              # every joined sheet is met by a probe line next to the hole
                e1 = np.cross(f.axis, [0.0, 0.0, 1.0] if abs(f.axis[2]) < 0.9 else [1.0, 0.0, 0.0])
                o = f.position + 1.6 * 0.5 * f.d * e1 / np.linalg.norm(e1)
                h = G.ray_hits(man[pid], o - 0.003 * f.axis, o + (f.grip or f.length) * f.axis + 0.003 * f.axis)
                self.assertGreaterEqual(len(h), 2, f"{f.id} misses {pid}")

    def test_air_gaps_and_pipe_clearance(self):
        host = self.reg.parts[self.hosts["R"]].mesh.to_manifold()
        shd = self.reg.parts["YK250-PR-523-R"].mesh.to_manifold()
        ins = self.reg.parts["YK250-PR-520-R"].mesh.to_manifold()
        exh = self.reg.parts["YK250-PR-504-R"].mesh.to_manifold()
        self.assertGreaterEqual(shd.min_gap(host, 0.02), P.HS["standoff"] - 3e-4)   # 5 mm air gap (layout)
        self.assertGreaterEqual(ins.min_gap(exh, 0.02), P.HS["hole"] - 1e-3)        # engine dynamic margin
        self.assertGreaterEqual(exh.min_gap(host, 0.05), 0.025)                    # shielded cowl rule

    def test_masses(self):
        m = sum(self.reg.mass(self.reg.parts[f"YK250-PR-{n}-{s}"]) for n in (520, 523) for s in "RL")
        self.assertLess(m, 1.0)
        self.assertGreater(m, 0.3)


if __name__ == "__main__":
    unittest.main()
