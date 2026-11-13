"""ucav250 payload producer (design/payload.py): the retractable Trillium HD59 EO/IR turret, its isolated mount, the
ball-screw elevator (rails, carriage, drive, end stops), the two sliding bay doors with their Volz DA 22 rack drives,
the aperture lip seal and the harness service loop.

The payload parts are clean under every design-rule check (mesh, static, swept over the coupled sequence
``turret_extension``, clearance, thickness, fastener, contact, attachment) against the chassis and the shell; every
part honours the producer contract; the turret is flush (fully inside the closed doors) when retracted and its field
of regard is free of payload and chassis parts when extended; the E180 growth envelope fits the fixed mechanism; and
the group mass is compared with the payload mass items. Nothing here writes to the repository (checks run with
write=False)."""
from __future__ import annotations

import re
import sys
import unittest
from pathlib import Path

import numpy as np
from shapely.geometry import Point, Polygon, box as sbox
from shapely.ops import unary_union

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
try:
    from ucav250.analysis import checks as K
    from ucav250.core import geom as G
    from ucav250.core.assemble import build_registry
    from ucav250.design import payload as PL
    HAVE = True
except Exception:  # pragma: no cover
    HAVE = False

ID_RE = re.compile(r"^YK250-PL-(\d{3})(-[LR])?$")
FORBIDDEN = ("hardpoint", "hard-point", "hard point", "pylon", "weapon", "munition", "release mechanism")
TURRET, CARRIAGE, DOOR_R, DOOR_L = "YK250-PL-820", "YK250-PL-822", "YK250-PL-826-R", "YK250-PL-826-L"
RING, LOWER_SKIN = "YK250-SH-363", "YK250-SH-361"


@unittest.skipUnless(HAVE, "ucav250 dependencies missing")
class TestPayloadBuild(unittest.TestCase):
    """One registry for the class: chassis, shell, payload registered twice (the second call is ignored), hardware."""

    @classmethod
    def setUpClass(cls):
        cls.reg = build_registry(modules=["chassis", "shell", "payload", "payload", "hardware"], strict=False)
        cls.spec = cls.reg.spec
        cls.L = cls.spec["layout"]
        cls.pl = {pid: p for pid, p in cls.reg.parts.items() if ID_RE.match(pid)}
        cls.summary = K.run_all(cls.reg, quick=False, write=False, focus=set(cls.pl))
        cls.seq = cls.reg.sequences["turret_extension"]
        cls.ext = cls.seq[-1]

    def posed(self, pid, state):
        return self.reg.posed_mesh(self.reg.parts[pid], state)

    # ------------------------------------------------------------------ checks
    def test_design_rule_checks_clean(self):
        bad = {k: v[:3] for k, v in self.summary["details"].items() if v}
        self.assertTrue(self.summary["ok"], f"payload violations: {bad}")
        for k in ("mesh", "static", "swept", "clearance", "thickness", "fastener", "contact", "attachment"):
            self.assertIn(k, self.summary["violations"], k)
            self.assertEqual(self.summary["violations"][k], 0, k)

    def test_registered_once(self):
        self.assertIn("payload: already registered, second call ignored", self.reg.log)

    # ------------------------------------------------------------------ contract
    def test_ids_in_payload_range_and_mirrored(self):
        lo, hi = self.L["part_numbers"]["payload"]
        self.assertGreaterEqual(len(self.pl), 35)
        for pid, p in self.pl.items():
            m = ID_RE.match(pid)
            self.assertTrue(lo <= int(m.group(1)) <= hi, pid)
            self.assertEqual(p.group, "payload", pid)
            if m.group(2):
                other = pid[:-1] + ("L" if pid.endswith("R") else "R")
                self.assertIn(other, self.pl, f"{pid} has no mirror")
                self.assertEqual(p.side, m.group(2)[1], pid)
            else:
                self.assertEqual(p.side, "C", pid)

    def test_layout_fixed_ids(self):
        """The ids the layout fixes for the turret bay exist: turret, carriage, elevator drive, the two rails, the
        door actuators and the sliding doors."""
        el = self.L["chassis"]["turret_elevator"]
        ids = [el["part"]] + [r["part"] for r in el["rails"]]
        eq = {e["id"]: e for e in self.L["systems"]["equipment"]}
        ids += [eq["EQ-TURRET"]["part"], eq["EQ-TDOORACT"]["part"] + "-R", eq["EQ-TDOORACT"]["part"] + "-L"]
        ids += [CARRIAGE, DOOR_R, DOOR_L]
        for pid in ids:
            self.assertIn(pid, self.pl, pid)
        self.assertAlmostEqual(self.reg.mass(self.pl[eq["EQ-TDOORACT"]["part"] + "-R"]),
                               float(eq["EQ-TDOORACT"]["mass_kg"]), places=4)

    def test_part_contract(self):
        mats, procs = self.spec["materials"], self.spec["processes"]
        steps = {int(s["step"]) for s in self.spec["assembly"]["steps"]}
        for pid, p in self.pl.items():
            self.assertTrue(p.name and p.name_tr, pid)
            self.assertIn(p.step, steps, pid)
            self.assertEqual(p.step, PL.STEP, pid)
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
        for pid, p in self.pl.items():
            text = " ".join([p.name, p.name_tr, p.notes, p.vendor]).lower()
            for w in FORBIDDEN:
                self.assertNotIn(w, text, pid)
        src = Path(PL.__file__).read_text(encoding="utf-8").lower()
        for w in FORBIDDEN:
            self.assertNotIn(w, src)

    def test_meshes_closed_and_mirror_equal(self):
        for pid, p in self.pl.items():
            self.assertGreater(p.mesh.volume(), 0.0, pid)
            if pid.endswith("-R"):
                vr, vl = p.mesh.volume(), self.pl[pid[:-1] + "L"].mesh.volume()
                self.assertAlmostEqual(vr, vl, delta=2e-3 * vr + 1e-12, msg=pid)

    # ------------------------------------------------------------------ interfaces from spec.layout
    def test_turret_envelope_and_layout_boxes(self):
        """HD59 ball / height from payload.turret, ball centre on the layout point, the turret, door actuator and
        elevator drive inside their equipment boxes."""
        T = self.spec["payload"]["turret"]
        lo, hi = self.pl[TURRET].mesh.bounds()
        self.assertAlmostEqual(hi[2] - lo[2], float(T["height"]), delta=0.001)
        self.assertAlmostEqual(hi[0] - lo[0], float(T["ball_diameter"]), delta=0.001)
        c = np.asarray(self.L["chassis"]["turret_elevator"]["ball_center_retracted"], float)
        self.assertAlmostEqual(lo[2] + 0.5 * float(T["ball_diameter"]), c[2], delta=0.001)
        for e in self.L["systems"]["equipment"]:
            if e["id"] not in ("EQ-TURRET", "EQ-TDOORACT", "EQ-ELEVATOR"):
                continue
            b = np.asarray(e["box"], float)
            for pid in ([e["part"] + "-R"] if e.get("mirror") else [e["part"]]):
                lo, hi = self.pl[pid].mesh.bounds()
                self.assertTrue(np.all(lo >= b[0] - 1e-6) and np.all(hi <= b[1] + 1e-6), f"{e['id']} {pid}")

    def test_joints_and_sequence_from_layout(self):
        jl = {j["name"]: j for j in self.L["mechanisms"]["joints"]}
        for name in ("turret_elevator", "turret_door_R", "turret_door_L"):
            j = self.reg.joints[name]
            self.assertEqual(j.kind, jl[name]["kind"])
            self.assertAlmostEqual(j.hi, float(jl[name]["hi"]))
            self.assertTrue(np.allclose(j.axis, jl[name]["axis"]))
        moving = {p.id for p in self.reg.moving_parts("turret_elevator")}
        self.assertTrue({TURRET, CARRIAGE} <= moving)
        self.assertIn(DOOR_R, {p.id for p in self.reg.moving_parts("turret_door_R")})
        states = self.L["mechanisms"]["sequences"]["turret_extension"]["states"]
        self.assertEqual(len(self.seq), len(states))
        # doors fully open before the elevator moves, elevator at full stroke at the end
        for st in self.seq:
            if st["turret_elevator"] > 0.0:
                self.assertAlmostEqual(st["turret_door_R"], jl["turret_door_R"]["hi"])
                self.assertAlmostEqual(st["turret_door_L"], jl["turret_door_L"]["hi"])
        self.assertAlmostEqual(self.ext["turret_elevator"], float(self.L["chassis"]["turret_elevator"]["stroke"]))

    # ------------------------------------------------------------------ motion
    def test_sequence_sweep_without_collision(self):
        """Every state of turret_extension (and mid-points between them): the moving payload parts do not overlap the
        chassis, the shell or the fixed payload parts, and the doors keep >= 5 mm to the turret and carriage."""
        moving = {p.id for j in ("turret_elevator", "turret_door_R", "turret_door_L")
                  for p in self.reg.moving_parts(j)}
        fixed = [pid for pid, p in self.reg.parts.items() if pid not in moving and p.group in
                 ("chassis", "shell", "payload")]
        states = list(self.seq)
        states += [{k: 0.5 * (a[k] + b[k]) for k in a} for a, b in zip(self.seq[:-1], self.seq[1:])]
        cache = {}
        for st in states:
            posed = {pid: self.posed(pid, st) for pid in moving if pid.startswith("YK250-PL")}
            for a, ma in posed.items():
                for b in fixed:
                    if b in self.reg.parts[a].contacts or a in self.reg.parts[b].contacts:
                        continue
                    if b not in cache:
                        cache[b] = self.reg.parts[b].mesh
                    if not G.aabb_overlap(ma, cache[b]):
                        continue
                    self.assertLess(G.intersection_volume(ma, cache[b]), 1e-9, f"{a} x {b} at {st}")
            for d in (DOOR_R, DOOR_L):
                for t in (TURRET, CARRIAGE):
                    self.assertGreaterEqual(G.min_gap(posed[d], posed[t], 0.02), 0.005 - 1e-6, f"{d}-{t} {st}")

    def test_flush_when_retracted(self):
        """Retracted with the doors closed: the doors close the aperture (they cover the ring opening in plan) without
        reaching below the OML, the turret is fully above the closed doors with >= door_clearance to their inner
        face, and nothing of the payload protrudes below the OML."""
        T = self.spec["payload"]["turret"]
        C = PL.Ctx(self.reg, self.spec)
        need = float(T["bay"]["door_clearance"])
        for d in (DOOR_R, DOOR_L):
            self.assertGreaterEqual(G.min_gap(self.pl[d].mesh, self.pl[TURRET].mesh, 0.05), need - 1e-6, d)
        # doors cover the ring opening in plan, apart from the 1 mm butt gap of their edges on the centre line
        shadow = []
        for d in (DOOR_R, DOOR_L):
            m = self.pl[d].mesh
            tri = m.V[m.F]
            down = m.face_normals()[:, 2] < -0.2
            shadow += [Polygon(t[:, :2]).buffer(1e-6) for t in tri[down]]
        cover = unary_union(shadow)
        opening = Point(C.xc, 0.0).buffer(C.r_open, 128).difference(
            sbox(C.xc - 0.1, -PL.DOOR_GAP_CL - 1e-4, C.xc + 0.1, PL.DOOR_GAP_CL + 1e-4))
        self.assertLess(opening.difference(cover).area, 1e-7)
        # no payload vertex below the lower OML in the rest pose (doors on the inner face of the skin)
        for pid, p in self.pl.items():
            V = p.mesh.V
            sel = (V[:, 0] > C.xf1) & (V[:, 0] < C.xf3)
            if not np.any(sel):
                continue
            V = V[sel]
            z_oml = np.array([float(C.z_in(x, y, 0.0)) for x, y in V[::max(1, len(V) // 400)][:, :2]])
            self.assertTrue(np.all(V[::max(1, len(V) // 400)][:, 2] >= z_oml - 1e-4), pid)

    def test_fov_clear_when_extended(self):
        """Extended (doors open): no payload or chassis part reaches below the KO-TURRET-FOV cone, the ball centre is on
        the layout point and the ball clears the ring by >= 5 mm."""
        ids = [pid for pid, p in self.reg.parts.items() if p.group in ("payload", "chassis") and pid != TURRET]
        self.assertEqual(PL.fov_obstructions(self.reg, ids, self.ext), [])
        T = self.spec["payload"]["turret"]
        lo, hi = self.posed(TURRET, self.ext).bounds()
        c = np.asarray(self.L["chassis"]["turret_elevator"]["ball_center_extended"], float)
        self.assertAlmostEqual(lo[2], c[2] - 0.5 * float(T["ball_diameter"]), delta=0.001)
        self.assertGreaterEqual(G.min_gap(self.posed(TURRET, self.ext), self.reg.parts[RING].mesh, 0.02), 0.005 - 1e-6)

    def test_e180_growth_envelope(self):
        """The E180 growth envelope (sizing.turret_checks) on the HD59 ball centre clears every door, rail, drive and
        harness part at every state of turret_extension, and the closed doors when retracted (the elevator-borne
        HD59 kit - turret, adapter, isolators, carriage - the seal and the 157 mm ring are exchanged for the E180)."""
        kit = {p.id for p in self.pl.values() if p.joint == "turret_elevator"} | {"YK250-PL-847"}
        others = [pid for pid in self.pl if pid not in kit]
        for st in self.seq:
            dz = float(st["turret_elevator"])
            env = PL.e180_envelope(self.spec, dz)
            for pid in others:
                m = self.posed(pid, st)
                if G.aabb_overlap(env, m):
                    self.assertLess(G.intersection_volume(env, m), 1e-9, f"{pid} at {st}")
        env = PL.e180_envelope(self.spec, 0.0)
        for d in (DOOR_R, DOOR_L):
            self.assertGreater(G.min_gap(env, self.pl[d].mesh, 0.05), 0.0, d)

    def test_end_stops_bracket_the_stroke(self):
        """Hard stops 0.5 mm beyond the travel: at full stroke the carriage plate stands 0.5 mm above the lower journal
        bracket boss, retracted the ball-nut bumper stands 0.5 mm below the drive face."""
        car_ext = self.posed(CARRIAGE, self.ext)
        self.assertAlmostEqual(G.min_gap(car_ext, self.pl["YK250-PL-832"].mesh, 0.01), PL.STOP_GAP, delta=1e-4)
        self.assertAlmostEqual(G.min_gap(self.pl["YK250-PL-823"].mesh, self.pl["YK250-PL-824"].mesh, 0.01),
                               PL.STOP_GAP, delta=1e-4)

    # ------------------------------------------------------------------ fasteners
    def test_fasteners_metric_and_hardware_built(self):
        fs = [f for f in self.reg.fasteners() if any(pid in self.pl for pid in f.joins)]
        self.assertGreaterEqual(len(fs), 60)
        for f in fs:
            for pid in f.joins:
                self.assertIn(pid, self.reg.parts, f.id)
            self.assertRegex(f.spec, r"^ISO (4762|7380) M[345]x\d+-A2-70$", f.id)
            self.assertEqual(f.step, PL.STEP, f.id)
            self.assertIn(f"YK250-HW-{f.id}", self.reg.parts)

    # ------------------------------------------------------------------ mass
    def test_mass_against_budget(self):
        """HD59 + isolated mount = payload.turret_mass_kg; the mechanism (everything else of the payload group) at or
        below its mass item turret_lift_mechanism_doors (mass.rules.turret_mechanism_kg with the
        mass.rules.growth_allowance; the item also books the aperture ring and the bay liner, built by shell / chassis:
        see docs/detail/payload.md section 9)."""
        mount = {TURRET} | {p.id for p in self.pl.values() if "isolator" in p.name or "adapter" in p.name}
        m_mount = sum(self.reg.mass(self.pl[pid]) for pid in mount)
        m_mech = sum(self.reg.mass(p) for pid, p in self.pl.items() if pid not in mount)
        self.assertAlmostEqual(m_mount, float(self.spec["payload"]["turret_mass_kg"]), delta=0.02)
        item = next(i for i in self.spec["mass"]["items"] if i["name"] == "turret_lift_mechanism_doors")
        self.assertAlmostEqual(float(item["mass_base_kg"]), float(self.spec["mass"]["rules"]["turret_mechanism_kg"]))
        self.assertLessEqual(m_mech, float(item["mass_kg"]) + 1e-6, f"turret mechanism {m_mech:.3f} kg")


if __name__ == "__main__":
    unittest.main()
