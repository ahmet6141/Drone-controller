"""YK-250 HANCER structures phase: ucav250.analysis.structures (hand calculations, margins of safety) and the
structlib helpers it added (laminate theory, plate / sandwich stability, columns, joints, truss, landing).

The structlib tests compare every helper with a closed-form or textbook value. The structures tests run the full
analysis on the committed spec (check mode: the sized dimensions are read from spec.structures.sizing), require every
margin >= 0, a spec block identical to the regenerated one and consistent layout interfaces, and confirm with
corrupted designs that the checks are not vacuous. The CLI test writes into a scratch directory and verifies that no
tracked file changed."""
from __future__ import annotations

import copy
import math
import sys
import tempfile
import unittest
from pathlib import Path

import numpy as np

REPO = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO))
try:
    import yaml
    from ucav250.analysis import structlib as ST
    from ucav250.analysis import structures as SS
    from ucav250.core import spec as SPEC
    HAVE = True
except Exception:  # pragma: no cover
    HAVE = False

SPEC_FILE = REPO / "ucav250" / "spec.yaml"
DOC04 = REPO / "ucav250" / "docs" / "04_yapi_hesaplari.md"
OUT = REPO / "ucav250" / "out"
BANNED = ("weapon", "hardpoint", "hard point", "hard_point", "hard-point", "pylon", "munition", "release", "bomb",
          "missile", "warhead", "silah", "mühimmat", "muhimmat", "sert nokta")
TR_CHARS = set("çğıöşüÇĞİÖŞÜ")


def iso_ply(E=70e9, nu=0.3, t=1e-3, F=300e6):
    return ({"E1": E, "E2": E, "G12": E / (2 * (1 + nu)), "nu12": nu, "Ftu": F, "Fcu": F, "Fsu": F / 2}, 0.0, t)


def ud_ply(theta=0.0, t=0.14e-3):
    p = {"E1": 130e9, "E2": 8e9, "G12": 3e9, "nu12": 0.3, "Ftu": 1500e6, "Fcu": 800e6, "F2tu": 25e6, "F2cu": 100e6,
         "Fsu": 30e6}
    return (p, theta, t)


# =====================================================================================================================
# structlib helpers
# =====================================================================================================================
@unittest.skipUnless(HAVE, "ucav250 dependencies missing")
class TestLaminateTheory(unittest.TestCase):
    def test_isotropic_plate_abd(self):
        E, nu, t = 70e9, 0.3, 2e-3
        lam = ST.laminate_abd([iso_ply(E, nu, t / 2), iso_ply(E, nu, t / 2)])
        Q11 = E / (1 - nu ** 2)
        self.assertAlmostEqual(lam["A"][0, 0] / (Q11 * t), 1.0, places=9)
        self.assertAlmostEqual(lam["D"][0, 0] / (Q11 * t ** 3 / 12), 1.0, places=9)
        self.assertLess(np.abs(lam["B"]).max(), 1e-6)
        eng = ST.laminate_engineering(lam)
        self.assertAlmostEqual(eng["Ex"] / E, 1.0, places=9)
        self.assertAlmostEqual(eng["nuxy"], nu, places=9)

    def test_qbar_rotation(self):
        Q = ST.ply_Q(130e9, 8e9, 3e9, 0.3)
        np.testing.assert_allclose(ST.Qbar(Q, 0.0), Q, rtol=1e-12, atol=1e-3)
        Q90 = ST.Qbar(Q, 90.0)
        self.assertAlmostEqual(Q90[0, 0] / Q[1, 1], 1.0, places=9)
        Q45 = ST.Qbar(Q, 45.0)
        self.assertAlmostEqual(Q45[0, 0] / ((Q[0, 0] + Q[1, 1] + 2 * Q[0, 1] + 4 * Q[2, 2]) / 4), 1.0, places=9)

    def test_first_ply_failure_uniaxial(self):
        lam = ST.laminate_abd([ud_ply(0.0, 1e-3)])
        N = 1000.0 * 1e3                                    # 1000 N/mm on 1 mm -> 1000 MPa in the fibre
        f = ST.first_ply_failure(lam, (N, 0.0, 0.0))
        self.assertAlmostEqual(f["R_max_stress"], 1.5, places=2)       # Xt / sigma, small Poisson coupling
        fc = ST.first_ply_failure(lam, (-N, 0.0, 0.0))
        self.assertAlmostEqual(fc["R_max_stress"], 0.8, places=2)
        self.assertLessEqual(f["R"], f["R_max_stress"] + 1e-12)

    def test_restrained_unsymmetric_face(self):
        lam = ST.laminate_abd([ud_ply(0.0), ud_ply(90.0)])            # unsymmetric: B != 0
        self.assertGreater(np.abs(lam["B"]).max(), 1.0)
        N = (50e3, 0.0, 0.0)
        e_r = ST.ply_stresses(lam, N, restrained=True)
        e0 = np.linalg.solve(lam["A"], np.asarray(N))
        for r in e_r:                                        # membrane strain only: top = bottom of every ply
            if r["ply"] == 0:
                np.testing.assert_allclose(r["eps"][0], e0[0], rtol=1e-9)
        free = ST.first_ply_failure(lam, N)["R"]
        rest = ST.first_ply_failure(lam, N, restrained=True)["R"]
        self.assertGreater(rest, free)                       # the free warping adds bending stresses


@unittest.skipUnless(HAVE, "ucav250 dependencies missing")
class TestStability(unittest.TestCase):
    def setUp(self):
        self.E, self.nu, self.t = 70e9, 0.3, 2e-3
        self.lam = ST.laminate_abd([iso_ply(self.E, self.nu, self.t)])
        self.Dp = self.E * self.t ** 3 / (12 * (1 - self.nu ** 2))

    def test_orthotropic_compression_isotropic_limit(self):
        b = 0.2
        for a in (0.2, 1.0, 3.0):
            N = ST.orthotropic_compression_buckling(self.lam["D"], a, b)
            self.assertAlmostEqual(N / (4 * math.pi ** 2 * self.Dp / b ** 2), 1.0, delta=0.02)
        N_short = ST.orthotropic_compression_buckling(self.lam["D"], 0.1, b)      # a/b = 0.5: k = 6.25
        self.assertAlmostEqual(N_short / (6.25 * math.pi ** 2 * self.Dp / b ** 2), 1.0, places=6)
        self.assertAlmostEqual(N / self.t, ST.plate_compression_buckling(self.E, self.t, b, self.nu), delta=0.02 * N / self.t)

    def test_orthotropic_shear_long_isotropic_limit(self):
        b = 0.15
        Ns = ST.orthotropic_shear_buckling_long(self.lam["D"], b)
        self.assertAlmostEqual(Ns / (5.35 * math.pi ** 2 * self.Dp / b ** 2), 1.0, delta=0.03)

    def test_sandwich_formulas(self):
        Gc, d, c = 14e6, 5.5e-3, 5e-3
        S_ = ST.sandwich_shear_stiffness(Gc, d, c)
        self.assertAlmostEqual(S_, Gc * d ** 2 / c)
        self.assertAlmostEqual(ST.sandwich_buckling(1e5, Gc, d, c), 1e5 / (1 + 1e5 / S_))
        self.assertLess(ST.sandwich_buckling(1e9, Gc, d, c), S_)           # bounded by shear crimping
        self.assertAlmostEqual(ST.shear_crimping_stress(Gc, d, c, 0.6e-3, 0.4e-3), S_ / 1e-3)
        self.assertAlmostEqual(ST.face_wrinkling(50e9, 45e6, 14e6), 0.5 * (50e9 * 45e6 * 14e6) ** (1 / 3))
        face = ST.laminate_abd([iso_ply(70e9, 0.3, 0.5e-3)])
        np.testing.assert_allclose(ST.sandwich_D(face, d), face["A"] * d ** 2 / 2)
        self.assertAlmostEqual(ST.intracell_dimpling(70e9, 0.3, 0.5e-3, 3.2e-3),
                               2 * 70e9 / (1 - 0.09) * (0.5 / 3.2) ** 2)
        st = ST.sandwich_strip_pressure(20e3, 0.3, 6e-3, 0.6e-3)
        self.assertAlmostEqual(st["M"], 20e3 * 0.09 / 8)
        self.assertAlmostEqual(st["tau_core"], 20e3 * 0.15 / 6e-3)
        self.assertAlmostEqual(ST.sandwich_strip_pressure(20e3, 0.3, 6e-3, 0.6e-3, fixed=True)["M"], 20e3 * 0.09 / 12)

    def test_johnson_euler(self):
        E, Fcy = 200e9, 400e6
        A, I = 1e-4, 1e-9
        rho = math.sqrt(I / A)
        sl_t = math.pi * math.sqrt(2 * E / Fcy)
        long_ = ST.johnson_euler(E, Fcy, A, I, 2 * sl_t * rho)
        self.assertEqual(long_["mode"], "Euler")
        self.assertAlmostEqual(long_["P_cr"], math.pi ** 2 * E * I / (2 * sl_t * rho) ** 2)
        at_t = ST.johnson_euler(E, Fcy, A, I, 0.999999 * sl_t * rho)
        self.assertAlmostEqual(at_t["sigma_cr"] / (Fcy / 2), 1.0, places=4)       # continuous at the transition
        short = ST.johnson_euler(E, Fcy, A, I, 1e-3)
        self.assertEqual(short["mode"], "Johnson")
        self.assertLess(short["sigma_cr"], Fcy)

    def test_tubes_and_cells(self):
        T, r = 50.0, 0.01
        self.assertAlmostEqual(ST.tube_torsion_stress(T, r, 0.0), 2 * T / (math.pi * r ** 3))
        self.assertAlmostEqual(ST.bredt_shear_flow(100.0, 0.02), 2500.0)


@unittest.skipUnless(HAVE, "ucav250 dependencies missing")
class TestJointsTrussLanding(unittest.TestCase):
    def test_bolt_group_inplane(self):
        pts = [[-1, -1], [1, -1], [1, 1], [-1, 1]]
        np.testing.assert_allclose(ST.bolt_group_inplane(pts, (400.0, 0.0)), [100.0] * 4)
        M = 800.0
        np.testing.assert_allclose(ST.bolt_group_inplane(pts, (0.0, 0.0), M), [M / (4 * math.sqrt(2))] * 4)
        f = ST.bolt_group_inplane(pts, (400.0, 0.0), M)
        self.assertAlmostEqual(float(f.max()), math.hypot(100.0 + 100.0, 100.0), places=6)

    def test_bolt_group_tension(self):
        pts = np.array([[0.0, -0.01], [0.0, 0.01], [0.02, -0.01], [0.02, 0.01]])
        T = ST.bolt_group_tension(pts, 1000.0, Mx=20.0)
        self.assertAlmostEqual(T.sum(), 1000.0)
        r = pts[:, 1] - pts[:, 1].mean()
        self.assertAlmostEqual(float((T * r).sum()), 20.0)

    def test_bolt_group_6dof(self):
        """VS2-02: rigid fitting on elastic bolts - axial share of a tension, couple of a moment, equal shear."""
        pts = [[-0.01, -0.01, 0], [0.01, -0.01, 0], [-0.01, 0.01, 0], [0.01, 0.01, 0]]
        ax = [[0, 0, 1]] * 4
        r = ST.bolt_group_6dof(pts, ax, [0, 0, 400.0], [0, 0, 0], [0, 0, 0])
        np.testing.assert_allclose(r["axial"], [100.0] * 4, atol=1e-9)
        r = ST.bolt_group_6dof(pts, ax, [0, 0, 0], [2.0, 0, 0], [0, 0, 0])
        np.testing.assert_allclose(r["axial"], [-50.0, -50.0, 50.0, 50.0], atol=1e-9)
        r = ST.bolt_group_6dof(pts, ax, [100.0, 0, 0], [0, 0, 0], [0, 0, 0])
        np.testing.assert_allclose(r["shear"], [25.0] * 4, atol=1e-9)
        np.testing.assert_allclose(r["forces"].sum(axis=0), [100.0, 0.0, 0.0], atol=1e-9)

    def test_pin_bending(self):
        M = ST.pin_bending_moment(10e3, 0.006, 0.030, 0.0002)
        self.assertAlmostEqual(M, 5e3 * (0.003 + 0.0002 + 0.0075))
        self.assertAlmostEqual(ST.pin_bending_stress(M, 0.014), 32 * M / (math.pi * 0.014 ** 3))

    def test_truss_two_bar(self):
        th = math.radians(30.0)
        nodes = [[-math.cos(th), 0, 0], [math.cos(th), 0, 0], [0, 0, -math.sin(th)], [0, 1, -math.sin(th)]]
        # two-bar planar truss loaded at the apex + a stabilising out-of-plane member
        members = [(0, 2), (1, 2), (2, 3)]
        P = 1000.0
        tr = ST.truss3d(nodes, members, [0, 1, 3], {2: (0.0, 0.0, -P)}, EA=1e7)
        self.assertAlmostEqual(tr["N"][0], P / (2 * math.sin(th)), places=6)     # both bars in tension
        self.assertAlmostEqual(tr["N"][1], P / (2 * math.sin(th)), places=6)
        self.assertAlmostEqual(abs(tr["N"][2]), 0.0, places=6)
        R = sum(tr["reactions"].values())
        np.testing.assert_allclose(R, [0.0, 0.0, P], atol=1e-6)

    def test_landing(self):
        r = ST.landing_nj(0.3, 0.15, 0.6, 2 / 3)
        self.assertAlmostEqual(r["nj"], (0.3 + 0.15 / 3) / (0.6 * 0.15))
        self.assertAlmostEqual(r["n_inertia"], r["nj"] + 2 / 3)
        self.assertAlmostEqual(ST.sink_speed(300.0), 2.13)
        self.assertAlmostEqual(ST.sink_speed(1e5), 3.05)


# =====================================================================================================================
# structures analysis on the committed spec
# =====================================================================================================================
@unittest.skipUnless(HAVE, "ucav250 dependencies missing")
class TestStructures(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        SPEC.load.cache_clear()
        cls.S = copy.deepcopy(SPEC.load())
        cls.c, cls.res, cls.block, cls.inter = SS.compute(copy.deepcopy(cls.S), authoring=False)
        cls.rows = cls.res["rows"]

    def test_every_margin_non_negative(self):
        neg = [f"{r['id']} {r['ms']:.3f}" for r in self.rows if r["ms"] is not None and r["ms"] < 0.0]
        self.assertEqual(neg, [])
        self.assertGreater(len([r for r in self.rows if r["ms"] is not None]), 150)

    def test_spec_block_current_and_interfaces(self):
        c2, res2, block2, inter2 = SS.compute(copy.deepcopy(self.S), authoring=True)
        self.assertEqual(SS.stale_items(self.S, block2), [])
        self.assertEqual([i["id"] for i in self.inter if not i["ok"]], [])

    def test_reproducible(self):
        c2, res2, block2, _ = SS.compute(copy.deepcopy(self.S), authoring=False)
        self.assertEqual([(r["id"], r["ms"], r["applied"], r["allowable"]) for r in res2["rows"]],
                         [(r["id"], r["ms"], r["applied"], r["allowable"]) for r in self.rows])
        self.assertEqual(block2, self.block)

    def test_coverage_of_the_load_cases_and_members(self):
        groups = {r["group"] for r in self.rows}
        for g in ("wing", "wing joint", "CT box", "tail", "controls", "gear", "engine mount", "parachute", "fuel bays",
                  "turret", "transport", "body", "frames", "equipment"):
            self.assertIn(g, groups)
        ids = {r["id"] for r in self.rows}
        for i in ("W-CAP-01", "W-WEB-01", "W-RWEB", "W-GLOVEWEB", "W-SKINBUCK-OP-UP-IN", "W-SKINCRIMP-CT",
                  "W-RIBCRUSH", "J-PIN-SHEAR", "J-PIN-BEND", "J-PIN-BUSH", "J-TONGUE-BUSH", "J-PRONG-BUSH",
                  "J-TONGUE-FLANGE-TT", "J-TONGUE-FLANGE-DT", "J-TONGUE-FLANGE", "J-TONGUE-WEB", "J-PRONG-WEB",
                  "J-REAR-PIN-SHEAR", "J-REAR-PIN-BEND", "J-REAR-LUG", "J-REAR-BR", "J-REAR-PLATE",
                  "J-REAR-BOLTS-RIB", "J-REAR-BOLTS-WEB", "J-REAR-BR-WEB", "J-REAR-LUGFIT", "CT-WEB-MS",
                  "CT-ATT-BR-MS", "CT-KINK", "T-SPINDLE", "T-BRG-IN", "T-NODE-STUB-BOLTS", "T-NODE-CHEEK",
                  "T-NODE-INBOARD", "T-NODE-BOSS", "T-NODE-BASE-BOLTS", "T-NODE-FW-BR", "T-NODE-FW-CORE",
                  "T-SOCKET-BR", "T-SOCKET-TUBE", "T-SOCKET-OD", "T-FINSPAR", "T-FINSKIN", "T-FINFIT-FR-REAR",
                  "T-AFTKEEL", "C-HINGE-AILERON", "C-HINGE-RUDDER", "C-PUSHROD-FLAP", "C-GGUST-AILERON", "G-MLEG",
                  "G-TRUN-BR", "G-TRUN-NET", "G-TRUN-SO", "G-DOWNLOCK", "G-NLEG", "G-NPIVOT-BR", "G-BEAM-CAP",
                  "G-EMA-MAIN", "G-DOOR-HINGE", "G-UPLOCK", "E-STRUT_C", "E-BOLT", "E-FOOT", "E-FOOT-BR",
                  "E-ISOLATOR", "P-SHACKLE", "P-LUG", "P-LUG-BR", "P-FRAME", "P-FRAME-BOLTS", "P-FRAME-BR",
                  "P-SPINE-BOLTS", "P-SPINE-BR", "P-SPINE-AX", "P-SPINE-COL", "P-SPINE-LOCAL", "P-SPINE-SCREWS",
                  "P-SPINE-SKIN", "F-FACE-030", "F-CORE-030", "F-WELLWEB", "F-PAYRAIL", "TU-ROOF", "TU-DOORDRIVE",
                  "TR-PAD", "B-DORSAL", "B-MIDFLOOR", "B-MIDFLOOR-BUCK", "B-SKIN-SHEAR", "FR-GEAR-WEB", "FR-GEAR-BR",
                  "FR-RS-WEB", "FR-ROOF-SIDE", "FR-MS-RING", "FR-RS-RING", "FW-FOOT-CORE", "FW-FOOT-LAND",
                  "FW-FOOT-FACE", "FW-UPPER-SPLICE", "FR-3738-KEEL", "EQ-PAYTRAY"):
            self.assertIn(i, ids, i)
        # fix round 1: the metallic tongue / fork, the bonded cap transfer and the old drag pin are gone (S1-02/S1-03)
        for gone in ("J-BOND-TONGUE", "J-BOND-FORK", "J-FS-BRG-TONGUE", "J-FS-BRG-FORK", "J-DRAG-SHEAR",
                     "T-HOUSING-BOLTS", "T-STUBROOT-LAND", "P-BOLTS"):
            self.assertNotIn(gone, ids, gone)

    def test_rows_well_formed(self):
        for r in self.rows:
            self.assertTrue(r["member_tr"] and r["case_tr"] and r["ref"] and r["unit"], r["id"])
            self.assertTrue(math.isfinite(r["applied"]), r["id"])
            if r["ms"] is not None:
                self.assertTrue(math.isfinite(r["allowable"]) and r["allowable"] > 0, r["id"])
                self.assertGreaterEqual(r["factor_total"], 1.0 - 1e-12, r["id"])
                self.assertAlmostEqual(r["ms"], r["allowable"] / (r["applied"] * r["factor_total"]) - 1.0, places=9)

    def test_factor_combination(self):
        f = SS.total_factor(self.c, fa=True, bear=True)
        self.assertAlmostEqual(f["total"], 1.5 * 2.0)                 # the largest special factor only
        self.assertAlmostEqual(SS.total_factor(self.c, fit=True, comp=True)["total"], 1.5 * 1.2)
        self.assertAlmostEqual(SS.total_factor(self.c, ult_only=True, fit=True)["total"], 1.15)
        hinge = [r for r in self.rows if r["id"].startswith("C-HINGE-")]
        self.assertTrue(hinge and all(abs(r["factor_total"] - 6.67) < 1e-9 for r in hinge))
        pins = [r for r in self.rows if r["id"] in ("J-PIN-SHEAR", "J-PIN-BEND", "T-SPLINE")]
        self.assertTrue(all(abs(r["factor_total"] - 2.25) < 1e-9 for r in pins))

    def test_wing_inertia_places_the_terms(self):
        """S1-01: the wing-set mass is distributed where it is: the carry-through caps inside the body (no relief
        outboard), the joint parts between the tongue tip and the end of the cap build-up; the distribution integrates
        to the wing-set mass of one side."""
        y = np.linspace(0.0, self.c.T["b2"], 2001)
        w = SS.wing_inertia(self.c, y)
        t = SS.wing_mass_terms(self.c)
        # fix round 2 (VS2-05): the relief uses the minimum credible wing mass - item base masses (no growth allowance)
        # without the 1.05 model factor; the actuators at their base masses
        W_min = ((SS.mass_item_base(self.c, "wing_structure_pair") +
                  SS.mass_item_base(self.c, "control_surfaces_ailerons_pair") +
                  SS.mass_item_base(self.c, "control_surfaces_flaps_pair")) / 1.05 +
                 SS.mass_item_base(self.c, "actuators_ailerons_2x_DA26") +
                 SS.mass_item_base(self.c, "actuators_flaps_2x_DA30"))
        self.assertAlmostEqual(float(np.trapz(w, y)) / (W_min / 2), 1.0, delta=0.01)
        W_grown = (SS.mass_item(self.c, "wing_structure_pair") + SS.mass_item(self.c, "control_surfaces_ailerons_pair") +
                   SS.mass_item(self.c, "control_surfaces_flaps_pair") + SS.mass_item(self.c, "actuators_ailerons_2x_DA26") +
                   SS.mass_item(self.c, "actuators_flaps_2x_DA30"))
        self.assertLess(float(np.trapz(w, y)), 0.96 * W_grown / 2)
        self.assertGreater(float(np.trapz(w[y < SS.Y_SOB], y[y < SS.Y_SOB])), 0.25 * t["caps"])

    def test_glove_caps_carry_the_glove_box_moment(self):
        """The tongue carries the joint moment to the pins: the glove caps between the pins see only the remainder
        (zero tongue moment at and inboard of the inner pin)."""
        L = SS.wing_loads(self.c)
        g = SS.joint_geometry(self.c)
        self.assertEqual(SS.tongue_moment(self.c, L, g["y_in"] - 0.01), 0.0)
        y = 0.5 * (g["y_in"] + g["y_out"])
        M, _ = SS.cap_moments(self.c, L, y)
        self.assertLess(M, SS.at(L, "M", y))
        self.assertGreater(M, 0.0)
        self.assertEqual(SS.cap_moments(self.c, L, 0.2)[0], SS.at(L, "M", 0.2))

    def test_joint_drag_moment_path(self):
        """S1-02: the chordwise cases carry a drag moment Mz = M tan(alpha) as a spanwise couple on the rear pin; a
        too small rear pin fails."""
        cs = self.res["wing_joint"]["cases"]
        for k in ("PHAA", "NHAA", "VD-DRAG"):
            self.assertIn(k, cs)
        self.assertGreater(cs["PHAA"]["Mz"], 0.2 * cs["PHAA"]["M"])
        self.assertAlmostEqual(cs["PHAA"]["F_y"], cs["PHAA"]["Mz"] / self.res["wing_joint"]["dx"], places=6)
        S = copy.deepcopy(self.S)
        S["layout"]["chassis"]["wing_joint"]["rear_spar"]["pin"]["diameter"] = 0.002
        c = SS.Ctx(S)
        R = SS.Rows()
        SS.check_wing_joint(c, R, self.res["sized"])
        self.assertLess({r["id"]: r["ms"] for r in R.rows}["J-REAR-PIN-SHEAR"], 0.0)

    def test_parachute_spine_path(self):
        """S1-04: the bridle x-component is carried by the spine channel; a too thin spine fails."""
        self.assertGreater(self.res["parachute"]["Fx_max"], 5000.0)
        S = copy.deepcopy(self.S)
        m = next(q for q in S["layout"]["chassis"]["members"] if q["id"] == "M-SPINE")
        m["section"]["t"] = 0.0002
        c = SS.Ctx(S)
        R = SS.Rows()
        SS.check_parachute(c, R)
        ms = {r["id"]: r["ms"] for r in R.rows}
        self.assertLess(min(ms["P-SPINE-AX"], ms["P-SPINE-LOCAL"]), 0.0)

    def test_firewall_foot_land_needed(self):
        """S1-06 / VS2-06: without the 85 mm land and the denser core insert the firewall peak core shear at a lower
        engine foot fails; with the 66 mm land of fix round 1 the peak shear (x 1.23) fails as well."""
        D = copy.deepcopy(self.S["structures"]["sizing"])
        D["firewall"]["foot_land"].update(land_r_m=0.035, core="core_rohacell_51wf")
        c = SS.Ctx(copy.deepcopy(self.S), design=D)
        R = SS.Rows()
        ct = SS.check_ct_box(c, R, self.res["sized"])
        em = SS.check_engine_mount(c, R)
        SS.check_frames(c, R, ct, em)
        self.assertLess({r["id"]: r["ms"] for r in R.rows}["FW-FOOT-CORE"], 0.0)
        D = copy.deepcopy(self.S["structures"]["sizing"])
        D["firewall"]["foot_land"].update(land_r_m=0.066)
        c = SS.Ctx(copy.deepcopy(self.S), design=D)
        R = SS.Rows()
        SS.check_frames(c, R, SS.check_ct_box(c, R, self.res["sized"]), SS.check_engine_mount(c, R))
        self.assertLess({r["id"]: r["ms"] for r in R.rows}["FW-FOOT-CORE"], 0.0)

    def test_fix_round_2_rows_present(self):
        ids = {r["id"] for r in self.rows}
        for i in ("P-SHACKLE-BEND", "P-SPOOL-BR", "P-SPINE-PULL", "G-DOWNLOCK-BEND", "G-FIT-BOLTS", "G-FIT-BR",
                  "G-FIT-PULL-BEAM", "G-FIT-PULL-ROOF", "G-BEAM-WEB", "G-BEAM-NOTCH", "G-BEAM-END", "G-BEAM-TORSION",
                  "G-ROOF-TORSION", "FR-3738-SEG", "FR-3738-SEG-SH", "FR-3738-KEEL-BOLTS", "FR-3738-END-BOLTS",
                  "CT-KINK-BOLTS", "CT-KINK-BR", "CT-KINK-RIB", "CT-KINK-PLATE", "J-TRANS-WEB", "J-TRANS-ILSS",
                  "J-TRANS-RIB", "J-TRANS-CAP", "J-TRANS-SKIN", "T-SPINDLE-SPL-MT", "T-NODE-TEMP", "G-TOW-NLEG",
                  "G-TOW-PIVOT", "DT-COND"):
            self.assertIn(i, ids, i)
        dt = next(r for r in self.rows if r["id"] == "DT-COND")
        self.assertLess(dt["applied"], 0.50)

    def test_shackle_pin_bending_detected(self):
        """VS2-01: the former d 6 4130 shackle pin fails in bending (Melcon-Hoblit) although it passes in shear."""
        S = copy.deepcopy(self.S)
        f = next(q for q in S["layout"]["chassis"]["fittings"] if q["id"] == "F-RISER-FWD")
        f["shackle_pin"] = {"d": 0.006, "material": "steel_4130_n"}
        f["lug"]["bore"] = 0.006
        R = SS.Rows()
        SS.check_parachute(SS.Ctx(S), R)
        ms = {r["id"]: r["ms"] for r in R.rows}
        self.assertGreater(ms["P-SHACKLE"], 0.0)
        self.assertLess(ms["P-SHACKLE-BEND"], 0.0)

    def test_downlock_bending_detected(self):
        """VS2-07: a d 8 4130 lock bolt passes double shear but fails in bending in the clevis."""
        D = copy.deepcopy(self.S["structures"]["sizing"])
        D["gear"]["main_downlock_pin_d_m"] = 0.008
        R = SS.Rows()
        SS.check_gear(SS.Ctx(copy.deepcopy(self.S), design=D), R)
        ms = {r["id"]: r["ms"] for r in R.rows}
        self.assertGreater(ms["G-DOWNLOCK"], 0.0)
        self.assertLess(ms["G-DOWNLOCK-BEND"], 0.0)

    def test_lug_shear_bearing_uses_ftu(self):
        """VS2-09: the shear-bearing allowable of structlib.lug_axial is K_br F_tu D t (F_bru would be ~1.9 x higher)."""
        al = self.S["materials"]["al_7075_t651_plate"]
        a = ST.lug_axial(al["Ftu"], al["Ftu"], w=0.032, D=0.008, t=0.008, e=0.016)
        b = ST.lug_axial(al["Ftu"], al["Fbru"], w=0.032, D=0.008, t=0.008, e=0.016)
        self.assertLess(a["P_shear_bearing"], b["P_shear_bearing"])
        for rid in ("J-REAR-LUG", "P-LUG"):
            self.assertIn("F_tu", next(r for r in self.rows if r["id"] == rid)["ref"])

    def test_ud_bearing_and_lug_edges(self):
        """S1-05 / S1-09: the UD tape carries no bearing allowable; the trunnion lug is checked at its actual e/D."""
        self.assertNotIn("Fbru", self.S["materials"]["cfrp_ud_mtm45_as4"])
        self.assertIn("no fastener", self.S["structures"]["sizing"]["wing"]["main_cap"]["interleaf"])
        tr = next(r for r in self.rows if r["id"] == "G-TRUN-BR")
        lug = next(f for f in self.S["layout"]["chassis"]["fittings"] if f["id"] == "F-TRUNNION")["lug"]
        self.assertIn(f"e/D {float(lug['e_m']) / float(lug['bore']):.2f}", tr["member"])
        self.assertIn("Bruhn", tr["ref"])

    def test_open_items_from_computed_values(self):
        """S1-07: the kink force in the open items is the CT-KINK value; no run time in the outputs (S1-10)."""
        kink = next(r for r in self.rows if r["id"] == "CT-KINK")["applied"] / 1e3
        items = " ".join(a for a, _ in SS.open_items(self.res))
        self.assertIn(f"{kink:.1f} kN", items)
        self.assertNotIn("runtime_s", self.res["summary"])
        js = (OUT / "structures.json").read_text(encoding="utf-8")
        self.assertNotIn("runtime", js)

    def test_negative_aft_keel_detected(self):
        D = copy.deepcopy(self.S["structures"]["sizing"])
        D["body"]["aft_keel"].update(material="al_2024_t3_sheet", t_m=0.0016)
        c = SS.Ctx(copy.deepcopy(self.S), design=D)
        R = SS.Rows()
        SS.check_tail(c, R)
        ms = {r["id"]: r["ms"] for r in R.rows}
        self.assertLess(ms["T-AFTKEEL"], 0.0)

    def test_negative_skin_orientation_detected(self):
        """The layout-phase wing skin (0/90-dominated faces, 5 mm ROHACELL 51 WF over the box) crimps and buckles at
        the cap design strain; the structures-phase skins pass."""
        S = copy.deepcopy(self.S)
        for key in ("wing_skin_primary", "wing_box_skin_upper"):
            S["layups"][key]["plies"] = [["cfrp_pw_mtm45_as4", "0/90,+-45,0/90", 3]]
            S["layups"][key]["inner_plies"] = [["cfrp_pw_mtm45_as4", "+-45,0/90", 2]]
            S["layups"][key]["core_t"] = 0.005
        f = SS.total_factor(self.c, comp=True)["total"]
        zones = self.block["wing"]["main_cap"]["zones"]
        old = {r["id"]: r for r in SS.wing_skin_panels(SS.Ctx(S), SS.wing_loads(SS.Ctx(S)), zones)}
        new = {r["id"]: r for r in SS.wing_skin_panels(self.c, SS.wing_loads(self.c), zones)}
        for k in ("W-SKINBUCK-OP-UP-IN", "W-SKINBUCK-GLOVE-UP"):
            self.assertLess(old[k]["N_crimp"] / (old[k]["N_combined"] * f) - 1.0, 0.0, k)      # crimping fails
            self.assertLess(old[k]["k"] / f - 1.0, 0.0, k)                                     # buckling fails
            self.assertGreaterEqual(new[k]["N_crimp"] / (new[k]["N_combined"] * f) - 1.0, 0.0, k)
            self.assertGreaterEqual(new[k]["k"] / f - 1.0, 0.0, k)

    def test_mass_block_consistent_with_sizing_items(self):
        mb = self.S["structures"]["sizing"]["mass"]
        items = {i["name"]: i for i in self.S["mass"]["items"]}
        self.assertAlmostEqual(items["wing_carry_through_box_fittings"]["mass_base_kg"],
                               mb["chassis"]["carry_through_kg"], places=3)
        self.assertAlmostEqual(items["stabilator_spindle_bearing_housings"]["mass_base_kg"],
                               mb["chassis"]["stabilator_spindle_bearing_housings_kg"], places=3)
        self.assertAlmostEqual(items["nose_gear_trunnion_fitting"]["mass_base_kg"], mb["chassis"]["nose_pivot_fitting_kg"],
                               places=3)
        self.assertAlmostEqual(items["parachute_attach_fitting"]["mass_base_kg"],
                               mb["chassis"]["parachute_spine_fittings_kg"], places=3)
        self.assertIn("structures", items["wing_carry_through_box_fittings"]["basis"])
        self.assertEqual(self.block["mass"], mb)
        # group estimates within the mass.budget ceilings
        est = {}
        for it in self.S["mass"]["items"]:
            est[it["group"]] = est.get(it["group"], 0.0) + float(it["mass_kg"])
        for g, b in self.S["mass"]["budget"].items():
            self.assertLessEqual(est.get(g, 0.0), float(b["target_kg"]) + 1e-6, g)

    def test_text_rules(self):
        texts = [yaml.safe_dump(self.S["structures"]["sizing"], allow_unicode=True),
                 yaml.safe_dump({k: self.S["layups"][k] for k in SS.LAYUPS}, allow_unicode=True)]
        for p in (DOC04, OUT / "structures.md"):
            self.assertTrue(p.exists(), p)
            texts.append(p.read_text(encoding="utf-8"))
        for t in texts:
            low = t.lower()
            for w in BANNED:
                self.assertNotIn(w, low, w)
        doc = DOC04.read_text(encoding="utf-8")
        self.assertTrue(TR_CHARS & set(doc))
        for key in ("MS", "CS-LUAS", "STANAG", "çırpınma", "ana kiriş başlığı"):
            self.assertIn(key, doc)
        md = (OUT / "structures.md").read_text(encoding="utf-8")
        self.assertIn("| No | Eleman / kontrol | Yük durumu | Uygulanan | İzin verilen | Birim | Katsayılar | MS | "
                      "Referans |", md)

    def test_cli_check_and_scratch_outputs(self):
        tracked = [SPEC_FILE, OUT / "structures.md", OUT / "structures.json"]
        before = {p: p.read_bytes() for p in tracked}
        self.assertEqual(SS.main(["--check"]), 0)
        with tempfile.TemporaryDirectory() as td:
            self.assertEqual(SS.main(["--out", td]), 0)
            md = (Path(td) / "structures.md").read_text(encoding="utf-8")
            self.assertEqual(md, (OUT / "structures.md").read_text(encoding="utf-8"))
        self.assertEqual({p: p.read_bytes() for p in tracked}, before)


if __name__ == "__main__":
    unittest.main()
