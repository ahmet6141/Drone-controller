"""Landing-gear producer (YK-250 HANCER): retractable tricycle gear, wheels, brakes, retraction drives, locks and every
gear door with its hinges and drive linkage (``spec.landing_gear``, ``spec.layout.mechanisms``).

What it builds (part numbers from ``spec.layout.part_numbers.gear``: 620-669 legs, wheels, brakes, actuators; 670-719
doors, hinges, locks, door drives; the ids fixed by the layout are kept, the detail parts use free numbers of the
range, see ``docs/detail/gear.md``):

* main legs (starboard modelled, port = ``mirror_part``): machined 7075 yoke + outer cylinder of the telescopic
  spring / oil damper leg (``structures.sizing.gear.main_leg`` OD 42 x 3.5) on the 18 deg splay line from the trunnion
  to the axle, flanged 4130 stub axles (d 20 x 30) in the 20 H7 bushings of F-TRUNNION, sliding tube with the axle
  carrier, scissor torque links, the cantilever 4130 axle with its nut, the TOST SB Max II 3.5 in disc-brake wheel
  with the TOST 200x50 6 PR tyre and the BZT M caliper on a keyed torque plate; damper cartridge (envelope);
* main retraction: rotary EMA on the trunnion axis (layout ACT-MLG-EMA envelope, output spline into the forward stub
  axle), its reaction flange bolted through the main-well forward wall; up-lock: catch on the F-UPLOCK pivot boss and a
  twin roller on the leg; down-lock: lock bolt of the EMA unit (gear-unit item, see the open items);
* main doors: sequenced inner door (DA 22 actuator on FS-GEAR, crank + pushrod through C-DOORLINK to the door horn),
  leg door on two standoff brackets of the leg, trunnion door on the gear-beam hinge line driven by the leg through a
  slotted (lost-motion) link with an opening spring;
* nose leg: 7075 yoke + outer cylinder (OD 36 x 3), flanged stub axles d 16 in the F-NG-PIVOT bushings, steering
  sleeve + trailing fork (trail 30 mm) with the TOST LR Max II wheel, steering DA 26 on the aft face of the leg (belt
  drive), friction shimmy damper on the steering collar, retraction EMA (ACT-NLG-EMA) with crank and over-centre drag
  link (down-lock), up-lock hook under the keel wall engaging a roller on the fork crown;
* nose clamshell doors with their hinge pins, bearing blocks, horns, centre-line bellcrank and links (NDOOR-LINKAGE)
  driven by one DA 22 (EQ-NDOORACT); brake actuator + master cylinder (EQ-BRAKE_UNIT).

Interfaces read (never another module's geometry for placement): ``spec.landing_gear`` (trunnion, axle, splay,
tyre, nose pivot / axle / trail, door scheme), ``spec.layout`` (part_numbers, mechanisms joints / sequences /
door_outlines / assembly_paths, chassis fittings F-TRUNNION / F-UPLOCK / F-NG-PIVOT, members M-WELLKEEL /
M-WELLWALL-FWD / M-KEELWALL / M-GEARBEAM / M-DECK-NOSE, stations FS0600 / FS1110 / FS-GEAR, systems equipment and
actuators, shell panel ids, clearances), ``spec.structures.sizing.gear``, ``spec.fuselage`` (OML through ``oml``),
``spec.materials`` / ``layups`` / ``processes`` / ``assembly`` / ``mass``. Purchased items carry their datasheet
masses (``data/research/components.yaml``); estimates are labelled.

Module-private detailing constants are listed below and in ``docs/detail/gear.md``.
"""
from __future__ import annotations

import math

import numpy as np
from shapely.geometry import Point, Polygon
from shapely.ops import unary_union

from ..core import geom as G
from ..core.parts import Fastener, Joint, Part, Registry, mirror_part
from . import joints as J
from . import oml as O

GROUP = "gear"

# ---------------------------------------------------------------------------------------------------------------------
# module-private detailing constants (docs/detail/gear.md)
# ---------------------------------------------------------------------------------------------------------------------
M7075, M4130, MPW = "al_7075_t651_plate", "steel_4130_n", "cfrp_pw_mtm45_as4"
P_CNC, P_PREG, P_BUY = "cnc_milling_metal", "prepreg_ooa_vacbag", "purchased"
STEP_MAIN, STEP_NOSE, STEP_DOOR, STEP_BRAKE = 28, 29, 30, 20

YOKE_GAP = 0.001            # yoke arm to fitting lug face (each side)
ARM_T = 0.013               # yoke arm thickness = layout pivot_scheme stub_axle arm_m
ARM_R = 0.025               # yoke arm boss radius about the trunnion axis
STUB_FL_R, STUB_FL_T = 0.0225, 0.004   # stub-axle flange (3 x M3 at r 16.2 mm into the arm)
STUB_BOLT_R = 0.0162
CYL_RI = 0.0175             # outer cylinder bore (OD 42 x 3.5, structures.sizing.gear.main_leg)
CYL_S = (0.030, 0.105)      # outer cylinder along the leg axis (s from the trunnion axis)
CROWN_S = (0.0235, 0.038)   # yoke crown between the arms (1 mm clear of the stub-axle flanges)
GLAND = (0.096, 0.0235)     # gland nut collar start s / radius
TUBE_RO, TUBE_RI = 0.01725, 0.0145     # sliding tube (0.25 mm radial bearing clearance in the cylinder)
TUBE_S = (0.058, 0.122)
CARR_Y = (-0.046, -0.034)   # axle carrier plate, offset from the wheel centre along the axle (inboard)
CARR_TOP = 0.085            # carrier plate top above the axle centre
AXLE_R = 0.00995            # axle d 20 (TOST bearing bore 20 mm, g6), bores modelled at 20.02 mm
HUB_HW = 0.025              # TOST SB Max II installation width 50 mm
DISC = (0.0445, 0.0480, 0.050)   # brake disc y0 / y1 (from the wheel centre, outboard) and radius (estimate)
TP_Y = (0.0485, 0.0535)     # caliper torque plate
CAL = (0.040, 0.072, 0.019, 0.037)  # caliper box: radial x0, x1, half height z, inboard face y
NUT_Y = (0.0535, 0.0615)    # axle nut
ROLLER_E = -0.029           # up-lock roller axis offset from the leg axis (stowed: above the leg)
ROLLER_R = 0.0072
ROLLER_PIN_D = 0.005
HOOK_Y = (0.2330, 0.2380, 0.2620, 0.2670)   # up-lock catch plates (beside the F-UPLOCK boss, clear of its bolt heads)
HOOK_PIN_D = 0.005
DOOR_GAP = 0.0005           # half of the 1.0 mm panel gap (layout.shell.rules.joggles)
BOND_TB = 0.0003            # bond line of the trunnion-door hinge blocks on the skin inner face


def _u(v) -> np.ndarray:
    v = np.asarray(v, float)
    return v / np.linalg.norm(v)


def _rot(V, origin, axis, ang):
    return G.rotate_about(np.asarray(V, float), origin, axis, ang)


def _box(lo, hi) -> G.Mesh:
    lo, hi = np.asarray(lo, float), np.asarray(hi, float)
    return G.box(hi - lo, 0.5 * (lo + hi))


def _obox(center, R, size) -> G.Mesh:
    """Box of ``size`` along the columns of ``R`` centred at ``center``."""
    return G.box(size, center=np.asarray(center, float), R=np.asarray(R, float))


def _plate(poly, origin, u, v, t0, t1) -> G.Mesh:
    """Shapely polygon drawn in the (u, v) plane through ``origin``, extruded along u x v from t0 to t1."""
    w = np.cross(_u(u), _u(v))
    return G.extrude(poly, t1 - t0, origin=np.asarray(origin, float) + t0 * w, u=u, v=v)


def _clean(m: G.Mesh, tol: float = 2e-6) -> G.Mesh:
    """Collapse sliver triangles left by the booleans (manifold simplify within ``tol``)."""
    return G.Mesh.from_manifold(m.to_manifold().simplify(tol))


# =====================================================================================================================
# context: interfaces from the spec
# =====================================================================================================================
class Ctx:
    def __init__(self, reg: Registry, spec: dict):
        self.reg = reg
        self.S = spec
        self.L = spec["layout"]
        self.LG = spec["landing_gear"]
        self.n0 = int(self.L["part_numbers"]["gear"][0])
        self.fit = {f["id"]: f for f in self.L["chassis"]["fittings"]}
        self.mem = {m["id"]: m for m in self.L["chassis"]["members"]}
        self.st = {s["id"]: s for s in self.L["stations"]}
        self.jl = {j["name"]: j for j in self.L["mechanisms"]["joints"]}
        self.door = self.L["mechanisms"]["door_outlines"]
        self.eq = {e["id"]: e for e in self.L["systems"]["equipment"]}
        self.act = {a["id"]: a for a in self.L["systems"]["actuators"]}
        self.pan = {p["id"]: p for p in self.L["shell"]["panels"]}
        self.fus = O.fuselage_from_spec(spec)
        sz = spec["structures"]["sizing"]["gear"]
        self.sz = sz
        m = self.LG["main"]
        self.T = np.asarray(m["trunnion"], float)
        self.A = np.asarray(m["axle_static"], float)
        self.d = _u(self.A - self.T)                       # leg axis (down, outboard: 18 deg splay)
        self.ex = np.array([1.0, 0.0, 0.0])
        self.e = _u(np.cross(self.ex, self.d))              # outboard-up, normal to the leg in the y-z plane
        if self.e[1] < 0:
            self.e = -self.e
        self.leg_len = float(np.linalg.norm(self.A - self.T))
        tyre = self.LG["tyre"]
        self.R_t, self.W_t, self.r_rim = 0.5 * float(tyre["diameter"]), float(tyre["width"]), float(tyre["rim_radius"])
        self.cyl_ro = 0.5 * float(sz["main_leg"]["od_m"])
        self.cyl_t = float(sz["main_leg"]["wall_m"])
        tf = self.fit["F-TRUNNION"]
        self.lug_x = sorted(float(b["x"]) for b in tf["bearings"])
        bx = [np.asarray(b, float) for b in tf["boxes"]]
        # inner (yoke-side) faces of the two bearing lugs: the lug boxes that contain the bearing x positions
        self.lug_in = (max(b[1][0] for b in bx if b[0][0] <= self.lug_x[0] <= b[1][0]),
                       min(b[0][0] for b in bx if b[0][0] <= self.lug_x[1] <= b[1][0]))
        sa = tf["pivot_scheme"]["stub_axle"]
        self.stub_d, self.stub_l = float(sa["d"]), float(sa["length"])
        jm = self.jl["main_gear_R"]
        self.j_main = jm
        lo_ = self.door["main_leg_door_R"]["outline"]
        # leg slot of the skin (x range of the leg / trunnion door outlines) less 1 mm margin each side
        self.slot_x = (min(p[0] for p in lo_) + 0.001, max(p[0] for p in lo_) - 0.001)
        self.ret = float(jm["hi"])

    # ---------------------------------------------------------------- helpers
    def pid(self, num: int, side: str = "C", code: str = "LG") -> str:
        return f"YK250-{code}-{num:03d}" + ("" if side == "C" else f"-{side}")

    def P(self, s, e_=0.0, x_=0.0) -> np.ndarray:
        """Point in the main-leg frame: s along the leg axis from the trunnion axis, e_ along e, x_ along x."""
        return self.T + s * self.d + e_ * self.e + x_ * self.ex

    def Rleg(self) -> np.ndarray:
        return np.column_stack([self.ex, self.e, self.d])

    def stow(self, V):
        """Main-gear starboard rest -> stowed (main_gear_R at hi)."""
        jm = self.j_main
        return _rot(V, jm["origin"], jm["axis"], self.ret)

    def unstow(self, V):
        jm = self.j_main
        return _rot(V, jm["origin"], jm["axis"], -self.ret)


# =====================================================================================================================
# main leg geometry (starboard, rest pose = gear down, static compression)
# =====================================================================================================================
class MainLeg:
    def __init__(self, C: Ctx):
        self.C = C
        T = C.T
        g = YOKE_GAP
        self.arm_f = (C.lug_in[0] + g, C.lug_in[0] + g + ARM_T)     # forward yoke arm x range
        self.arm_a = (C.lug_in[1] - g - ARM_T, C.lug_in[1] - g)     # aft yoke arm x range
        self.xm = float(T[0])
        # stub axles d 20 x 30: from the inner face of each arm outward into the lug bushing
        self.stub_f = (self.arm_f[1] - C.stub_l, self.arm_f[1])
        self.stub_a = (self.arm_a[0], self.arm_a[0] + C.stub_l)
        self.roller_s = (C.T[1] - HOOK_Y[3] - 0.0005, C.T[1] - HOOK_Y[0] + 0.0005)   # stowed roller y -> leg s
        self.lug_s = (self.roller_s[0] + 0.0075, self.roller_s[1] - 0.0075)

    # ---------------------------------------------------------------- 620: yoke + outer cylinder
    def leg(self) -> G.Mesh:
        C = self.C
        R = C.Rleg()
        T = C.T
        parts = []
        # the arms lie outside the leg slot of the skin (x): below the trunnion they stay on the inboard half of
        # the crown (e <= 0) so that they never reach the curved chine skin; the crown itself is slot-wide
        e_in = -(C.cyl_ro - 0.002)
        for xa, xb in (self.arm_f, self.arm_a):
            disc = G.cylinder(ARM_R, (xa, T[1], T[2]), (xb, T[1], T[2]), n=48)
            blk = _obox(C.P(0.5 * (CROWN_S[0] + CROWN_S[1]), 0.5 * e_in, 0.5 * (xa + xb) - T[0]), R,
                        (xb - xa, -e_in, CROWN_S[1] - CROWN_S[0]))
            parts.append(G.hull(np.vstack([disc.V, blk.V])))
        xs0, xs1 = C.slot_x
        crown = _obox(C.P(0.5 * (CROWN_S[0] + CROWN_S[1]), 0.0, 0.5 * (xs0 + xs1) - T[0]), R,
                      (xs1 - xs0, 2 * C.cyl_ro - 0.004, CROWN_S[1] - CROWN_S[0]))
        parts.append(crown)
        parts.append(G.tube(C.cyl_ro, CYL_RI, C.P(CYL_S[0]), C.P(CYL_S[1]), n=48))
        parts.append(G.tube(GLAND[1], CYL_RI, C.P(GLAND[0]), C.P(CYL_S[1]), n=48))
        # up-lock roller lug (rest: inboard-down side of the cylinder; stowed: on top of the leg)
        s0, s1 = self.lug_s
        rl = 2.1 * ROLLER_PIN_D                      # 2 D edge distance round the roller pin (+5 %)
        lug = G.hull(np.vstack([
            _obox(C.P(0.5 * (s0 + s1), 0.5 * (ROLLER_E - C.cyl_ro + 0.003), 0.0), R,
                  (2 * rl, abs(ROLLER_E) - C.cyl_ro + 0.003, s1 - s0)).V,
            G.cylinder(rl, C.P(s0, ROLLER_E), C.P(s1, ROLLER_E), n=32).V]))
        parts.append(lug)
        # leg-door bracket lugs (rest: outboard-up side of the cylinder; stowed: under the leg)
        for (x0, x1) in self.door_lug_x():
            for s_ in self.door_brk_s():
                parts.append(_obox(C.P(s_, C.cyl_ro + 0.001, 0.5 * (x0 + x1) - T[0]), R, (x1 - x0, 0.028, 0.018)))
        # torque-link upper lug (forward side of the cylinder)
        m = G.union(parts)
        cuts = [G.cylinder(0.5 * C.stub_d + 0.00002, (xa - 0.002, T[1], T[2]), (xb + 0.002, T[1], T[2]), n=48)
                for xa, xb in (self.arm_f, self.arm_a)]
        cuts.append(G.cylinder(CYL_RI, C.P(CROWN_S[1] - 0.0001), C.P(CYL_S[1] + 0.01), n=48))
        return G.difference(m, cuts)

    def door_lug_x(self):
        T = self.C.T
        return ((T[0] - 0.0195, T[0] - 0.0135), (T[0] + 0.0135, T[0] + 0.0195))

    def door_brk_s(self):
        return (0.056, 0.084)

    # ---------------------------------------------------------------- stub axles (636)
    def stubs(self) -> G.Mesh:
        C = self.C
        T = C.T
        r = 0.5 * C.stub_d - 0.00005
        ms = []
        for (xa, xb), fl in ((self.stub_f, (self.arm_f[1], self.arm_f[1] + STUB_FL_T)),
                             (self.stub_a, (self.arm_a[0] - STUB_FL_T, self.arm_a[0]))):
            shaft = G.cylinder(r, (xa, T[1], T[2]), (xb, T[1], T[2]), n=48)
            if fl[0] < xa:
                shaft = G.cylinder(r, (xa, T[1], T[2]), (xb, T[1], T[2]), n=48)
            flange = G.cylinder(STUB_FL_R, (fl[0], T[1], T[2]), (fl[1], T[1], T[2]), n=48)
            ms.append(G.union([shaft, flange]))
        return G.merge(ms)

    def stub_bolts(self):
        """(position on the flange face, axis) of the 3 x M3 flange bolts per stub axle."""
        C = self.C
        T = C.T
        out = []
        for face, ax in ((self.arm_f[1] + STUB_FL_T, -1.0), (self.arm_a[0] - STUB_FL_T, 1.0)):
            for k in range(3):
                a = math.radians(90.0 + 120.0 * k)
                p = np.array([face, T[1] + STUB_BOLT_R * math.cos(a), T[2] + STUB_BOLT_R * math.sin(a)])
                out.append((p, np.array([ax, 0.0, 0.0])))
        return out

    # ---------------------------------------------------------------- 621: sliding tube + axle carrier
    def carrier(self) -> G.Mesh:
        C = self.C
        R = C.Rleg()
        A = C.A
        y0, y1 = A[1] + CARR_Y[0], A[1] + CARR_Y[1]
        tube = G.tube(TUBE_RO, TUBE_RI, C.P(TUBE_S[0]), C.P(TUBE_S[1]), n=48)
        end = G.cylinder(TUBE_RO, C.P(TUBE_S[1] - 0.006), C.P(TUBE_S[1] + 0.002), n=48)
        z_top = A[2] + CARR_TOP
        strip = _box((A[0] - 0.016, y0, z_top - 0.004), (A[0] + 0.016, y1, z_top))
        neck = G.hull(np.vstack([end.V, strip.V]))
        poly = unary_union([Point(0.0, 0.0).buffer(0.022, 48),
                            Polygon([(-0.016, 0.0), (0.016, 0.0), (0.016, CARR_TOP), (-0.016, CARR_TOP)])])
        plate = _plate(poly, (A[0], y0, A[2]), (1, 0, 0), (0, 0, 1), -(y1 - y0), 0.0)
        m = G.union([tube, neck, plate])
        cut = [G.cylinder(0.01001, (A[0], y0 - 0.01, A[2]), (A[0], y1 + 0.01, A[2]), n=48)]
        return G.difference(m, cut)

    # ---------------------------------------------------------------- 634: axle + nut
    def axle(self) -> G.Mesh:
        A = self.C.A
        y = A[1]
        head = G.cylinder(0.016, (A[0], y + CARR_Y[0] - 0.004, A[2]), (A[0], y + CARR_Y[0], A[2]), n=48)
        shaft = G.cylinder(AXLE_R, (A[0], y + CARR_Y[0] - 0.0001, A[2]), (A[0], y + NUT_Y[1], A[2]), n=48)
        collar = G.tube(0.0145, AXLE_R - 0.0002, (A[0], y + CARR_Y[1], A[2]), (A[0], y - HUB_HW, A[2]), n=48)
        nut = G.tube(0.0145, AXLE_R - 0.0002, (A[0], y + NUT_Y[0], A[2]), (A[0], y + NUT_Y[1], A[2]), n=6)
        return G.union([head, shaft, collar, nut])

    # ---------------------------------------------------------------- 622: wheel + tyre + brake disc (TOST)
    def wheel(self) -> G.Mesh:
        return wheel_mesh(self.C, self.C.A, np.array([0.0, 1.0, 0.0]), disc=False)

    def disc(self) -> G.Mesh:
        y0, y1, rd = DISC
        return G.revolve([(0.014, HUB_HW), (0.028, HUB_HW), (0.028, y0 + 0.0002), (rd, y0), (rd, y1), (0.022, y1),
                          (0.022, y0 + 0.002), (0.014, y0 + 0.002)], n=64, axis_origin=self.C.A, axis=(0.0, 1.0, 0.0))

    # ---------------------------------------------------------------- 635: torque plate, 623: caliper
    def torque_plate(self) -> G.Mesh:
        A = self.C.A
        poly = unary_union([Point(0.0, 0.0).buffer(0.017, 48),
                            Polygon([(0.0, -0.014), (0.035, -0.0195), (CAL[1] + 0.0015, -0.0195),
                                     (CAL[1] + 0.0015, 0.0195), (0.035, 0.0195), (0.0, 0.014)])])
        poly = poly.difference(Point(0.0, 0.0).buffer(0.01001, 48))
        return _plate(poly, (A[0], A[1] + TP_Y[0], A[2]), (1, 0, 0), (0, 0, 1), -(TP_Y[1] - TP_Y[0]), 0.0)

    def caliper(self) -> G.Mesh:
        A = self.C.A
        y_in, y_out = A[1] + CAL[3], A[1] + TP_Y[0] - 0.0001
        body = _box((A[0] + CAL[0], y_in, A[2] - CAL[2]), (A[0] + CAL[1], y_out, A[2] + CAL[2]))
        slot = _box((A[0] + CAL[0] - 0.01, A[1] + DISC[0] - 0.0005, A[2] - CAL[2] - 0.01),
                    (A[0] + DISC[2] + 0.0005, A[1] + DISC[1] + 0.0005, A[2] + CAL[2] + 0.01))
        port = G.cylinder(0.006, (A[0] + 0.056, y_in + 0.003, A[2] + CAL[2] - 0.0001),
                          (A[0] + 0.056, y_in + 0.003, A[2] + CAL[2] + 0.008), n=6)
        return G.union([G.difference(body, [slot]), port])

    def caliper_bolts(self):
        A = self.C.A
        return [(np.array([A[0] + 0.061, A[1] + TP_Y[1], A[2] + s * 0.0085]), np.array([0.0, -1.0, 0.0]))
                for s in (1, -1)]

    # ---------------------------------------------------------------- up-lock catch on the F-UPLOCK boss (642)
    def hook_pivot(self) -> np.ndarray:
        f = self.C.fit["F-UPLOCK"]
        p = np.asarray(f["point"], float)
        # pivot bore of the F-UPLOCK hook boss: 6 mm above the layout hook point (chassis detail, d 5.2 bore)
        return p + np.array([0.0, 0.0, 0.006])

    def hook(self) -> G.Mesh:
        C = self.C
        pv = self.hook_pivot()
        zr = C.T[2] - ROLLER_E                       # stowed roller centre (above the stowed leg)
        r_p = 2.05 * HOOK_PIN_D
        prof = unary_union([Point(0.0, 0.0).buffer(r_p, 48),
                            Polygon([(-r_p, 0.0), (r_p, 0.0), (r_p, zr - pv[2] - 0.0065), (-r_p, zr - pv[2] - 0.0065)])])
        notch = unary_union([Point(0.0, zr - pv[2]).buffer(ROLLER_R + 0.0005, 48),
                             Polygon([(-(ROLLER_R + 0.0005), zr - pv[2]), (ROLLER_R + 0.0005, zr - pv[2]),
                                      (ROLLER_R + 0.0005, zr - pv[2] - 0.03), (-(ROLLER_R + 0.0005), zr - pv[2] - 0.03)])])
        tab = Polygon([(-0.020, -0.019), (-r_p + 0.001, -0.019), (-r_p + 0.001, -0.007), (-0.020, -0.007)])
        prof = unary_union([prof, tab]).difference(notch)
        plates = [_plate(prof, (pv[0], y0, pv[2]), (1, 0, 0), (0, 0, 1), -(y1 - y0), 0.0)
                  for y0, y1 in ((HOOK_Y[0], HOOK_Y[1]), (HOOK_Y[2], HOOK_Y[3]))]
        # bridge forward of the boss joins the plates (clear of the stowed roller lug and cylinder)
        br = _box((pv[0] - 0.020, HOOK_Y[1] - 0.0005, pv[2] - 0.019), (pv[0] - 0.012, HOOK_Y[2] + 0.0005, pv[2] - 0.007))
        return G.union(plates + [br])


def wheel_mesh(C: Ctx, c, ax, disc: bool) -> G.Mesh:
    """TOST 3.5 in wheel with the 200x50 tyre (and the flanged brake disc on the +ax side) revolved about ``ax``
    through ``c``: hub bore 20 mm, hub width 50 mm, rim r 44.5 mm, tyre OD 190 x 54 mm (layout tyre)."""
    R, W, rr = C.R_t, C.W_t, C.r_rim
    hc, a, b = 0.5 * (R + rr), 0.5 * (R - rr), 0.5 * W
    th = np.linspace(0.0, 2 * math.pi, 96, endpoint=False)
    sec = Polygon([(hc + a * math.cos(t), b * math.sin(t)) for t in th])     # elliptic section 50.5 x 54
    bead = Polygon([(rr - 0.002, -0.4 * W), (rr + 0.010, -0.4 * W), (rr + 0.010, 0.4 * W), (rr - 0.002, 0.4 * W)])
    tsec = unary_union([sec, bead]).simplify(0.0001)
    tyre = G.revolve(np.asarray(tsec.exterior.coords)[:-1], n=96, axis_origin=c, axis=ax)
    hub = G.revolve([(0.01001, -HUB_HW), (0.020, -HUB_HW), (0.020, -0.012), (rr + 0.001, -0.020),
                     (rr + 0.001, 0.020), (0.020, 0.012), (0.020, HUB_HW), (0.01001, HUB_HW)],
                    n=64, axis_origin=c, axis=ax)
    ms = [tyre, hub]
    if disc:
        y0, y1, rd = DISC
        ms.append(G.revolve([(0.014, HUB_HW - 0.0005), (0.028, HUB_HW - 0.0005), (0.028, y0 + 0.0002),
                             (rd, y0), (rd, y1), (0.022, y1), (0.022, y0 + 0.002), (0.014, y0 + 0.002)],
                            n=64, axis_origin=c, axis=ax))
    return G.union(ms)


# =====================================================================================================================
# doors: OML panels, swept reliefs, hinges
# =====================================================================================================================
def phi_lower(fus, x, y):
    """Polar angle of the lower OML point at (x, |y|) (vectorised bisection on the lower half)."""
    x = np.asarray(x, float)
    y = np.abs(np.asarray(y, float))
    lo = np.full(np.broadcast(x, y).shape, 0.5 * math.pi)
    hi = np.full_like(lo, math.pi)
    for _ in range(48):
        mid = 0.5 * (lo + hi)
        yy = fus.point(x, mid)[..., 1]
        big = yy > y
        lo = np.where(big, mid, lo)
        hi = np.where(big, hi, mid)
    return 0.5 * (lo + hi)


def oml_panel(C: Ctx, poly: Polygon, t: float, off: float = 0.0, nx: int = 40, ny: int = 30, pad: float = 0.004,
              side: float = 1.0) -> G.Mesh:
    """Solid between the lower OML (moved inward by ``off``) and ``t`` further inward over the plan polygon ``poly``
    (x, y), with vertical edge walls (like the shell cut-outs). ``side`` -1 for port (y < 0) polygons."""
    x0, y0, x1, y1 = poly.bounds
    xs = np.linspace(x0 - pad, x1 + pad, nx)
    ya = max(y0 - pad, 1e-4) if y0 >= 0.0 else y0 - pad          # never fold the grid over the centre line
    yb = min(y1 + pad, -1e-4) if y1 <= 0.0 else y1 + pad
    ys = np.linspace(ya, yb, ny)
    X, Y = np.meshgrid(xs, ys, indexing="ij")
    PH = phi_lower(C.fus, X, Y)
    if side < 0:
        PH = 2 * math.pi - PH
    P = C.fus.point(X, PH)
    N = C.fus.normal(X, PH)
    P = P - off * N
    pan = G.shell_from_grid(P, t, inward=-N)
    clip = G.extrude(poly, 0.8, origin=(0.0, 0.0, -0.6), u=(1, 0, 0), v=(0, 1, 0))
    return G.intersection(pan, clip)


def skin_section(C: "Ctx", x: float, ya: float, yb: float, t: float, n: int = 17, m: float = 0.0005) -> Polygon:
    """(y, z) section of a lower-skin band between ya and yb (outer face = OML, thickness t along the normal), grown
    by ``m`` all round."""
    ys = np.linspace(ya - m, yb + m, n)
    ph = phi_lower(C.fus, np.full_like(ys, x), ys)
    ph = np.where(ys < 0, 2 * math.pi - ph, ph)
    P = C.fus.point(np.full_like(ys, x), ph)
    N = C.fus.normal(np.full_like(ys, x), ph)
    out = P + m * N
    inn = P - (t + m) * N
    pts = np.vstack([out[:, 1:3], inn[::-1, 1:3]])
    return Polygon(pts).buffer(0)


def rect(x0, x1, y0, y1) -> Polygon:
    return Polygon([(x0, y0), (x1, y0), (x1, y1), (x0, y1)])


def relief_cutter(xs, section_fn, origin, axis, v0: float, v1: float, nth: int = 31) -> G.Mesh:
    """Cutter swept by fixed obstacles relative to a part turning about an (almost) x-parallel axis: at each station x
    the obstacle section (shapely polygon in (y, z), ``section_fn(x)``) is turned by -v for v in v0..v1 and united;
    between two stations the cutter is the prism of the union of both station regions (conservative, robust)."""
    regs = []
    for x in xs:
        sec = section_fn(x)
        geoms = list(sec.geoms) if hasattr(sec, "geoms") else [sec]
        polys = []
        for v in np.linspace(v0, v1, nth):
            for gpoly in geoms:
                pts = np.array([[x, py, pz] for py, pz in np.asarray(gpoly.exterior.coords)])
                q = _rot(pts, origin, axis, -v)
                polys.append(Polygon(q[:, 1:3]).buffer(0))
        regs.append(unary_union(polys).buffer(0.0002, join_style=2))
    prisms = []
    for k in range(len(xs) - 1):
        u = unary_union([regs[k], regs[k + 1]]).simplify(0.00005)
        prisms.append(_plate(u, (float(xs[k]), 0.0, 0.0), (0, 1, 0), (0, 0, 1), 0.0, float(xs[k + 1] - xs[k])))
    return G.union(prisms)


def skin_section(C: "Ctx", x: float, ya: float, yb: float, t: float, n: int = 17, m: float = 0.0005) -> Polygon:
    """(y, z) section of a lower-skin band between ya and yb (outer face = OML, thickness t along the normal), grown
    by ``m`` all round."""
    ys = np.linspace(ya - m, yb + m, n)
    ph = phi_lower(C.fus, np.full_like(ys, x), ys)
    ph = np.where(ys < 0, 2 * math.pi - ph, ph)
    P = C.fus.point(np.full_like(ys, x), ph)
    N = C.fus.normal(np.full_like(ys, x), ph)
    out = P + m * N
    inn = P - (t + m) * N
    pts = np.vstack([out[:, 1:3], inn[::-1, 1:3]])
    return Polygon(pts).buffer(0)


def rect(x0, x1, y0, y1) -> Polygon:
    return Polygon([(x0, y0), (x1, y0), (x1, y1), (x0, y1)])


def relief_cutter(xs, section_fn, origin, axis, v0: float, v1: float, nth: int = 31, npts: int = 160) -> G.Mesh:
    """Cutter swept by fixed obstacles relative to a part turning about an x-parallel axis: at each station x the
    obstacle section (shapely polygon in (y, z), ``section_fn(x)``) is turned by -v for v in v0..v1 and united; the
    stations are lofted into one closed solid (subtract it from the part in its closed pose)."""
    from shapely.geometry.polygon import orient
    rings = []
    for x in xs:
        sec = section_fn(x)
        polys = []
        geoms = list(sec.geoms) if hasattr(sec, "geoms") else [sec]
        for v in np.linspace(v0, v1, nth):
            for gpoly in geoms:
                pts = np.array([[x, py, pz] for py, pz in np.asarray(gpoly.exterior.coords)])
                q = _rot(pts, origin, axis, -v)
                polys.append(Polygon(q[:, 1:3]).buffer(0))
        u = unary_union(polys).buffer(0.0002, join_style=2)
        if u.geom_type != "Polygon":
            u = max(u.geoms, key=lambda g: g.area)
        u = orient(Polygon(u.exterior), 1.0)
        ring = u.exterior
        L = ring.length
        ds = np.linspace(0, L, 1440, endpoint=False)
        d0 = max(ds, key=lambda d: ring.interpolate(d).x + 0.3 * ring.interpolate(d).y)   # consistent start point
        pts = [ring.interpolate((d0 + L * k / npts) % L) for k in range(npts)]
        rings.append(np.array([[x, p.x, p.y] for p in pts]))
    return G.fix_orientation(G.loft(rings))


class Doors:
    """Main-gear doors (starboard): inner door 670, leg door 672 (+ brackets), trunnion door 673; hinges."""

    def __init__(self, C: Ctx, M: "MainLeg"):
        self.C, self.M = C, M
        D = C.door
        self.o_in = Polygon(D["main_inner_door_R"]["outline"])
        self.o_leg = Polygon(D["main_leg_door_R"]["outline"])
        self.o_tr = Polygon(D["main_trunnion_door_R"]["outline"])
        self.t = float(D["main_inner_door_R"]["thickness"])
        from ..core.parts import layup_props
        self.t_skin = layup_props(C.S, "shell_secondary")["thickness"]
        ji = C.jl["main_inner_door_R"]
        self.ji = ji
        self.jt = C.jl["main_trunnion_door_R"]
        # inner-door hinge zone: where the layout hinge line lies inside the door laminate (x 2.914..2.950)
        self.h_in = (2.914, 2.950)
        self.ears = ((2.9140, 2.9180), (2.9300, 2.9340), (2.9460, 2.9500))
        self.knuck = ((2.9185, 2.9295), (2.9345, 2.9455))

    def door_poly(self, o):
        return o.buffer(-DOOR_GAP, join_style=2)

    def area(self, o) -> float:
        return float(self.door_poly(o).area)

    # ---------------------------------------------------------------- inner door 670
    def cove(self) -> G.Mesh:
        o = np.asarray(self.ji["origin"], float)
        return G.cylinder(0.0042, (self.h_in[0] - 0.0015, o[1], o[2]), (self.h_in[1] + 0.0015, o[1], o[2]), n=32)

    def ear_polys(self):
        o = np.asarray(self.ji["origin"], float)
        wk = self.C.mem["M-WELLKEEL"]["box"]
        yw0, yw1, zb = float(wk[0][1]), float(wk[1][1]), float(wk[0][2])
        ring = Point(o[1], o[2]).buffer(0.0035, 32)
        col = Polygon([(o[1] - 0.0025, o[2]), (o[1] + 0.0025, o[2]), (o[1] + 0.0025, zb - 0.0035),
                       (yw1 + 0.003, zb - 0.0005), (yw1 + 0.003, zb + 0.022), (yw1, zb + 0.022), (yw1, zb - 0.0005),
                       (o[1] - 0.0025, zb - 0.004)])
        return unary_union([ring, col]).difference(Point(o[1], o[2]).buffer(0.0015 + 0.00002, 24))

    def fixed_hinge(self) -> G.Mesh:
        """671: inner-door fixed hinge bracket (7075): base plate on the outboard face of the well keel web, three
        ears down to the hinge line (pin d 3)."""
        wk = self.C.mem["M-WELLKEEL"]["box"]
        yw1, zb = float(wk[1][1]), float(wk[0][2])
        prof = self.ear_polys()
        ears = [_plate(prof, (xa, 0.0, 0.0), (0, 1, 0), (0, 0, 1), 0.0, xb - xa) for xa, xb in self.ears]
        base = _box((self.ears[0][0], yw1, zb + 0.004), (self.ears[-1][1], yw1 + 0.003, zb + 0.026))
        return G.union(ears + [base])

    def inner_door(self) -> G.Mesh:
        C = self.C
        o = np.asarray(self.ji["origin"], float)
        a = np.asarray(self.ji["axis"], float)
        pan = oml_panel(C, self.door_poly(self.o_in), self.t)
        kn = [G.cylinder(0.0035, (xa, o[1], o[2]), (xb, o[1], o[2]), n=32) for xa, xb in self.knuck]
        m = G.union([pan] + kn)
        x0, y0, x1, y1 = self.o_in.bounds
        ys = y0 - DOOR_GAP
        cove = Point(o[1], o[2]).buffer(0.0042, 32)

        def strip(x, with_cove):
            sec = skin_section(C, x, -ys, ys, self.t_skin)
            return sec.difference(cove) if with_cove else sec
        v0, v1 = float(self.ji["lo"]), float(self.ji["hi"])
        h0, h1 = self.h_in
        cuts = [relief_cutter(np.linspace(x0 - 0.002, h0 + 0.0005, 8), lambda x: strip(x, False), o, a, v0, v1),
                relief_cutter(np.linspace(h0 - 0.0005, h1 + 0.0005, 6), lambda x: strip(x, True), o, a, v0, v1),
                relief_cutter(np.linspace(h1 - 0.0005, x1 + 0.002, 20), lambda x: strip(x, False), o, a, v0, v1)]
        ear = self.ear_polys().buffer(0.0007)
        for xa, xb in self.ears:
            cuts.append(relief_cutter(np.array([xa - 0.0007, xb + 0.0007]), lambda x: ear, o, a, v0, v1))
        m = G.difference(m, cuts)
        bore = G.cylinder(0.0015 + 0.00002, (self.h_in[0] - 0.001, o[1], o[2]), (self.h_in[1] + 0.001, o[1], o[2]), n=24)
        return _clean(G.difference(m, [bore]))

    def inner_pin(self) -> G.Mesh:
        o = np.asarray(self.ji["origin"], float)
        return G.cylinder(0.0015 - 0.00001, (self.ears[0][0] - 0.0005, o[1], o[2]), (self.ears[-1][1] + 0.0005, o[1], o[2]),
                          n=24)

    # ---------------------------------------------------------------- leg door 672 (closed at main_gear hi)
    def leg_door_closed(self) -> G.Mesh:
        return oml_panel(self.C, self.door_poly(self.o_leg), self.t)

    def leg_door(self) -> G.Mesh:
        m = self.leg_door_closed()
        return G.Mesh(self.C.unstow(m.V), m.F)

    def leg_brackets(self) -> G.Mesh:
        """680: two standoff brackets (7075, 4 mm plate + 8 mm bonded foot) from the leg lugs to the leg-door inner
        face, built in the stowed pose (door flush in the skin) and turned back to the rest pose."""
        C, M = self.C, self.M
        ms = []
        for (x0, x1) in M.door_lug_x():
            fwd = x0 < C.T[0]
            xb = (x0 - 0.004, x0) if fwd else (x1, x1 + 0.004)
            lugc = C.stow(np.array([C.P(s_, C.cyl_ro + 0.001, 0.0) for s_ in M.door_brk_s()]))
            ya, yb = min(p[1] for p in lugc) - 0.009, max(p[1] for p in lugc) + 0.009
            zt = max(p[2] for p in lugc) + 0.010
            blk = _box((xb[0], ya, -0.222), (xb[1], yb, zt))
            foot = _box((xb[0], ya, -0.222), (xb[1] + 0.010, yb, -0.185)) if fwd else \
                _box((xb[0] - 0.010, ya, -0.222), (xb[1], yb, -0.185))
            ms.append(G.union([blk, foot]))
        m = G.union(ms)
        big = self.o_leg.buffer(0.03, join_style=2)
        m = G.difference(m, [oml_panel(C, big, 0.03 + self.t, off=-0.03)])
        return G.Mesh(C.unstow(m.V), m.F)

    def bracket_bolts(self):
        """(head point, axis) of the 2 x M4 per bracket through bracket + leg lug (rest pose), head outside."""
        C, M = self.C, self.M
        out = []
        for (x0, x1) in M.door_lug_x():
            fwd = x0 < C.T[0]
            for s_ in M.door_brk_s():
                p = C.P(s_, C.cyl_ro + 0.001, 0.0)
                p[0] = x0 - 0.004 if fwd else x1 + 0.004
                out.append((p, np.array([1.0 if fwd else -1.0, 0.0, 0.0])))
        return out

    # ---------------------------------------------------------------- trunnion door 673 (closed at lo, rest = hi)
    def tr_door_closed(self) -> G.Mesh:
        C = self.C
        jt = self.jt
        o = np.asarray(jt["origin"], float)
        a = np.asarray(jt["axis"], float)
        x0, y0, x1, y1 = self.door_poly(self.o_tr).bounds
        pan = oml_panel(C, self.door_poly(self.o_tr), self.t)
        kn = G.cylinder(0.003, (x0, o[1], o[2]), (x1, o[1], o[2]), n=32)
        m = G.union([pan, kn])
        yo = self.o_tr.bounds[3] + DOOR_GAP
        beam = C.mem["M-GEARBEAM"]
        nb = beam["notch"]
        yb0, yb1 = float(beam["box"][0][1]), float(beam["box"][1][1])
        b_sec = Polygon([(yb0 - 0.0005, float(nb["z_top"]) - 0.0005), (yb1 + 0.0005, float(nb["z_top"]) - 0.0005),
                         (yb1 + 0.0005, -0.06), (yb0 - 0.0005, -0.06)])

        cove = Point(o[1], o[2]).buffer(0.0042, 32)

        def obst(x):
            return unary_union([skin_section(C, x, yo, yo + 0.03, self.t_skin).difference(cove), b_sec])
        cut = relief_cutter(np.linspace(x0 - 0.002, x1 + 0.002, 8), obst, o, a, float(jt["lo"]), float(jt["hi"]))
        m = G.difference(m, [cut])
        return _clean(G.difference(m, [G.cylinder(0.0015 + 0.00002, (x0 - 0.001, o[1], o[2]), (x1 + 0.001, o[1], o[2]),
                                                  n=24)]))

    def tr_blocks(self):
        """681: trunnion-door hinge end blocks (7075): either side of the leg slot, bonded on the inner face of the
        lower skin round the hinge line (the beam is notched there), knuckle d 7 on the pin d 3."""
        C = self.C
        o = np.asarray(self.jt["origin"], float)
        xs0, xs1 = self.o_tr.bounds[0], self.o_tr.bounds[2]
        nb = C.mem["M-GEARBEAM"]["notch"]
        z_top = float(nb["z_top"]) - 0.0015
        rng = ((xs0 - DOOR_GAP - 0.0095, xs0 - DOOR_GAP - 0.0005), (xs1 + DOOR_GAP + 0.0005, xs1 + DOOR_GAP + 0.0095))
        y_in = 0.3725
        ms = []
        for xa, xb in rng:
            blk = _box((xa, y_in, -0.17), (xb, o[1] + 0.0035, z_top))
            kn = G.cylinder(0.0035, (xa, o[1], o[2]), (xb, o[1], o[2]), n=32)
            m = G.union([blk, kn])
            big = rect(xa - 0.01, xb + 0.01, 0.30, 0.42)
            m = G.difference(m, [oml_panel(C, big, 0.03 + self.t_skin + BOND_TB, off=-0.03, nx=10, ny=60)])
            ms.append(G.union([m, kn]))
        bore = G.cylinder(0.0015 + 0.00002, (rng[0][0] - 0.001, o[1], o[2]), (rng[1][1] + 0.001, o[1], o[2]), n=24)
        return G.difference(G.union(ms), [bore]), rng

    def tr_notch(self):
        o = np.asarray(self.jt["origin"], float)
        _m, rng = self.tr_blocks_rng()
        return [G.cylinder(0.0042, (rng[0][0] - 0.0005, o[1], o[2]), (rng[1][1] + 0.0005, o[1], o[2]), n=32)]

    def tr_blocks_rng(self):
        xs0, xs1 = self.o_tr.bounds[0], self.o_tr.bounds[2]
        return None, ((xs0 - DOOR_GAP - 0.0095, xs0 - DOOR_GAP - 0.0005), (xs1 + DOOR_GAP + 0.0005, xs1 + DOOR_GAP + 0.0095))

    def tr_pin(self) -> G.Mesh:
        o = np.asarray(self.jt["origin"], float)
        _m, rng = self.tr_blocks_rng()
        return G.cylinder(0.0015 - 0.00001, (rng[0][0] + 0.0005, o[1], o[2]), (rng[1][1] - 0.0005, o[1], o[2]), n=24)

    def tr_door(self) -> G.Mesh:
        jt = self.jt
        m = self.tr_door_closed()
        return G.Mesh(_rot(m.V, jt["origin"], jt["axis"], float(jt["rest"])), m.F)


# =====================================================================================================================
# nose gear (centre line; rest = gear down, static compression)
# =====================================================================================================================
N_ARM = (0.0255, 0.0345)    # yoke arm |y| range (1 mm inside the F-NG-PIVOT blocks), layout arm_m 9 mm
N_ARM_IN = 0.0290           # inner layer of the arm (to the crown, flange screws) / outer layer (boss only)
N_ARM_R = 0.0175            # outer-layer boss radius: inside the heads of the F-NG-PIVOT bolts and 10 mm from the
#                             open nose doors (layout clearance)
N_ARM_RI = 0.0190           # inner-layer boss radius
N_STUB_FL = (0.0215, 0.0255, 0.0190)   # stub flange |y| range and radius (inside the yoke)
N_STUB_BOLT_R = 0.0137
N_CROWN = (-0.0200, -0.0360)           # crown z below the pivot (clear of the stub flanges)
N_CROWN_HW = 0.0285                    # crown half width (y): inside the pivot-block bolt heads
N_CYL = (-0.030, -0.168)               # outer cylinder z (from the pivot)
N_TUBE = (-0.110, -0.212)              # steering / sliding tube z (from the pivot)
N_FORK_Z = (-0.207, -0.222)            # fork crown z (from the pivot); tyre top 7 mm below
N_FORK_Y = (0.0285, 0.0345)            # fork arm |y|
N_UPL_Y = (0.0394, 0.0411)             # nose up-lock roller on the axle ends (|y|), catch plate beside the keel wall


def _ccw(v, a):  # noqa: D103 - rotate a 2-D (x, z) vector counter-clockwise
    c, s_ = math.cos(a), math.sin(a)
    return np.array([v[0] * c - v[1] * s_, v[0] * s_ + v[1] * c])


class NoseLeg:
    def __init__(self, C: Ctx):
        self.C = C
        n = C.LG["nose"]
        self.Pv = np.asarray(n["pivot"], float)
        self.An = np.asarray(n["axle_static"], float)
        sz = C.sz["nose_leg"]
        self.ro = 0.5 * float(sz["od_m"])
        self.ri = self.ro - float(sz["wall_m"])
        f = C.fit["F-NG-PIVOT"]
        self.stub_d = float(f["pivot_scheme"]["stub_axle"]["d"])
        self.stub_l = float(f["pivot_scheme"]["stub_axle"]["length"])
        self.ret = float(C.jl["nose_gear"]["hi"])

    def X(self, x_, y_, z_):
        return np.array([self.Pv[0] + x_, y_, self.Pv[2] + z_])

    # ---------------------------------------------------------------- 624: yoke + outer cylinder + horn
    def leg(self) -> G.Mesh:
        Pv = self.Pv
        parts = []
        for sg in (1, -1):
            ya, yb = sg * N_ARM[0], sg * N_ARM_IN
            disc = G.cylinder(N_ARM_RI, (Pv[0], ya, Pv[2]), (Pv[0], yb, Pv[2]), n=48)
            blk = _box(self.X(-self.ro + 0.002, min(ya, yb), N_CROWN[1]), self.X(self.ro - 0.002, max(ya, yb),
                                                                                 N_CROWN[0]))
            parts.append(G.hull(np.vstack([disc.V, blk.V])))
            parts.append(G.cylinder(N_ARM_R, (Pv[0], sg * (N_ARM_IN - 0.0002), Pv[2]), (Pv[0], sg * N_ARM[1], Pv[2]), n=48))
        parts.append(_box(self.X(-self.ro, -N_CROWN_HW, N_CROWN[1]), self.X(self.ro, N_CROWN_HW, N_CROWN[0])))
        parts.append(G.tube(self.ro, self.ri, self.X(0, 0, N_CROWN[1] + 0.006), self.X(0, 0, N_CYL[1]), n=48))
        parts.append(G.tube(self.ro + 0.003, self.ri, self.X(0, 0, N_CYL[1] + 0.008), self.X(0, 0, N_CYL[1]), n=48))
        # steering-actuator mounting lugs (beside the cylinder, aft), trimmed to the case face
        act = self.C.act["ACT-STEER"]
        c = np.asarray(act["obb"]["center"], float)
        lugs = [_box((Pv[0] + 0.012, sg * 0.0165 - 0.0075, c[2] + dz - 0.0095), (c[0], sg * 0.0165 + 0.0075, c[2] + dz + 0.0095))
                for sg in (1, -1) for dz in (0.034, -0.034)]
        parts.append(G.difference(G.union(lugs), [self.steer_case()]))
        m = G.union(parts)
        cuts = [G.cylinder(0.5 * self.stub_d + 0.00002, (Pv[0], -0.05, Pv[2]), (Pv[0], 0.05, Pv[2]), n=48),
                G.cylinder(self.ri, self.X(0, 0, N_CROWN[1] - 0.0001), self.X(0, 0, N_CYL[1] - 0.01), n=48)]
        return G.difference(m, cuts)

    def stubs(self) -> G.Mesh:
        Pv = self.Pv
        r = 0.5 * self.stub_d - 0.00005
        ms = []
        y_bush = float(self.C.mem["M-KEELWALL"]["box"][0][1]) - 0.0005       # 0.5 mm short of the bushing end
        for sg in (1, -1):
            y_in = sg * N_ARM[0]
            y_out = sg * y_bush
            sh = G.cylinder(r, (Pv[0], y_in, Pv[2]), (Pv[0], y_out, Pv[2]), n=48)
            fl = G.cylinder(N_STUB_FL[2], (Pv[0], sg * N_STUB_FL[0], Pv[2]), (Pv[0], sg * N_STUB_FL[1], Pv[2]), n=48)
            ms.append(G.union([sh, fl]))
        return G.merge(ms)

    def stub_bolts(self):
        Pv = self.Pv
        out = []
        for sg in (1, -1):
            for ang in (90.0, 210.0, 330.0):
                a = math.radians(ang)
                p = np.array([Pv[0] + N_STUB_BOLT_R * math.cos(a), sg * N_STUB_FL[0], Pv[2] + N_STUB_BOLT_R * math.sin(a)])
                out.append((p, np.array([0.0, float(sg), 0.0])))
        return out

    # ---------------------------------------------------------------- 631: steering tube + trailing fork
    def fork(self) -> G.Mesh:
        Pv, An = self.Pv, self.An
        tro = self.ri - 0.00025
        tube = G.tube(tro, tro - 0.0025, self.X(0, 0, N_TUBE[0]), self.X(0, 0, N_TUBE[1]), n=48)
        crown = _box(self.X(-0.018, -N_FORK_Y[1], N_FORK_Z[1]), (An[0] + 0.016, N_FORK_Y[1], Pv[2] + N_FORK_Z[0]))
        boss = G.cylinder(tro, self.X(0, 0, N_TUBE[1] + 0.0005), self.X(0, 0, N_FORK_Z[0] - 0.0005), n=48)
        arms = []
        for sg in (1, -1):
            ya, yb = sg * N_FORK_Y[0], sg * N_FORK_Y[1]
            eye = G.cylinder(0.016, (An[0], ya, An[2]), (An[0], yb, An[2]), n=48)
            top = _box((An[0] - 0.014, min(ya, yb), Pv[2] + N_FORK_Z[1] - 0.001), (An[0] + 0.014, max(ya, yb),
                                                                                    Pv[2] + N_FORK_Z[1] + 0.001))
            arms.append(G.hull(np.vstack([eye.V, top.V])))
        m = G.union([tube, crown, boss] + arms)
        return G.difference(m, [G.cylinder(0.01001, (An[0], -0.05, An[2]), (An[0], 0.05, An[2]), n=48)])

    def axle(self) -> G.Mesh:
        An = self.An
        y1 = N_FORK_Y[1] + 0.0045
        pins = [G.cylinder(0.004, (An[0], sg * (y1 - 0.001), An[2]), (An[0], sg * N_UPL_Y[1], An[2]), n=32) for sg in (1, -1)]
        return G.union(pins + [G.tube(AXLE_R, 0.0035, (An[0], -y1, An[2]), (An[0], y1, An[2]), n=48),
                        G.tube(0.0145, 0.006, (An[0], y1 - 0.0045, An[2]), (An[0], y1, An[2]), n=6),
                        G.tube(0.0145, 0.006, (An[0], -y1, An[2]), (An[0], -y1 + 0.0045, An[2]), n=6)])

    def spacers(self) -> G.Mesh:
        An = self.An
        return G.merge([G.tube(0.0145, 0.01001, (An[0], sg * HUB_HW, An[2]), (An[0], sg * N_FORK_Y[0], An[2]), n=48)
                        for sg in (1, -1)])

    def wheel(self) -> G.Mesh:
        return wheel_mesh(self.C, self.An, np.array([0.0, 1.0, 0.0]), disc=False)

    def rollers(self) -> G.Mesh:
        An = self.An
        return G.merge([G.tube(0.006, 0.00402, (An[0], sg * N_UPL_Y[0], An[2]), (An[0], sg * (N_UPL_Y[1] - 0.0002), An[2]),
                               n=32) for sg in (1, -1)])

    def stowed_axle(self) -> np.ndarray:
        th = self.ret
        v = _ccw(self.An[[0, 2]] - self.Pv[[0, 2]], th)
        return np.array([self.Pv[0] + v[0], 0.0, self.Pv[2] + v[1]])

    def catch(self) -> G.Mesh:
        """650: nose up-lock catch (4130 plate, starboard; port = mirror) on the keel-wall inner face: the axle-end
        roller enters its notch from below at the end of the retraction (latch pawl + solenoid: gear-unit item)."""
        a = self.stowed_axle()
        wall = float(self.C.mem["M-KEELWALL"]["box"][0][1])
        prof = Polygon([(-0.020, -0.016), (0.020, -0.016), (0.020, 0.020), (-0.020, 0.020)])
        notch = unary_union([Point(0.0, 0.0).buffer(0.0065, 48),
                             Polygon([(-0.0065, 0.0), (0.0065, 0.0), (0.0065, -0.03), (-0.0065, -0.03)])])
        return _plate(prof.difference(notch), (a[0], wall - 0.0001, a[2]), (1, 0, 0), (0, 0, 1), 0.0,
                      wall - 0.0001 - N_UPL_Y[0] + 0.0004)

    def collar(self) -> G.Mesh:
        """644: steering collar + toothed-belt pulley (7075) clamped on the steering tube below the cylinder."""
        z0, z1 = N_CYL[1] - 0.0005, N_CYL[1] - 0.026
        tro = self.ri - 0.00025
        return G.union([G.tube(0.024, tro + 0.00002, self.X(0, 0, z0), self.X(0, 0, z0 - 0.006), n=60),
                        G.tube(0.022, tro + 0.00002, self.X(0, 0, z0 - 0.0059), self.X(0, 0, z1), n=60)])

    def shimmy(self) -> G.Mesh:
        """646: friction shimmy-damper ring pack (bronze / steel discs, spring-loaded) between the cylinder end and
        the collar (gear-unit item, mass estimate)."""
        z0 = N_CYL[1]
        return G.tube(self.ro + 0.003, self.ri + 0.0002, self.X(0, 0, z0), self.X(0, 0, z0 - 0.0005), n=60)

    def damper(self) -> G.Mesh:
        return G.cylinder(self.ri - 0.0003, self.X(0, 0, N_CROWN[1] - 0.0002), self.X(0, 0, N_TUBE[0] + 0.0002), n=40)

    # ---------------------------------------------------------------- steering actuator (FC-628) + belt
    def steer_case(self) -> G.Mesh:
        ob = self.C.act["ACT-STEER"]["obb"]
        return G.box(2 * np.asarray(ob["half"], float), center=np.asarray(ob["center"], float),
                     R=np.asarray(ob["axes"], float).T)

    def steer_bolts(self):
        ob = self.C.act["ACT-STEER"]["obb"]
        c = np.asarray(ob["center"], float)
        a0 = np.asarray(ob["axes"][0], float)
        h0 = float(ob["half"][0])
        out = []
        for sg in (1, -1):
            for dz in (0.034, -0.034):
                p = c - h0 * a0 + np.array([0.0, sg * 0.0165, 0.0]) + dz * np.asarray(ob["axes"][2], float) * -1.0
                out.append((p, a0))
        return out

    def steer_act(self) -> G.Mesh:
        ob = self.C.act["ACT-STEER"]["obb"]
        c = np.asarray(ob["center"], float)
        h = np.asarray(ob["half"], float)
        case = self.steer_case()
        # right-angle gearhead + belt pulley under the case (output on the steering axis' parallel)
        zb = c[2] - h[2]
        gh = _box((c[0] - 0.012, -0.012, zb - 0.012), (c[0] + 0.012, 0.012, zb + 0.002))
        pul = G.cylinder(0.009, (c[0], 0.0, zb - 0.0119), (c[0], 0.0, N_CYL[1] + self.Pv[2] - 0.0115), n=40)
        return G.union([case, gh, pul])

    def belt(self) -> G.Mesh:
        ob = self.C.act["ACT-STEER"]["obb"]
        c = np.asarray(ob["center"], float)
        zc0 = self.Pv[2] + N_CYL[1] - 0.0115
        z0, z1 = zc0 - 0.007, zc0 + 0.004
        x0, x1 = self.Pv[0], c[0]
        outer = unary_union([Point(x0, 0).buffer(0.0235, 64), Point(x1, 0).buffer(0.0105, 48),
                             Polygon([(x0, -0.0235), (x1, -0.0105), (x1, 0.0105), (x0, 0.0235)])])
        inner = unary_union([Point(x0, 0).buffer(0.0222, 64), Point(x1, 0).buffer(0.0092, 48),
                             Polygon([(x0, -0.0222), (x1, -0.0092), (x1, 0.0092), (x0, 0.0222)])])
        return G.extrude(outer.difference(inner), z1 - z0, origin=(0, 0, z0), u=(1, 0, 0), v=(0, 1, 0))

    def ema(self) -> G.Mesh:
        a = self.C.act["ACT-NLG-EMA"]
        lo, hi = np.asarray(a["box"][0], float), np.asarray(a["box"][1], float)
        body = _box(lo + np.array([0, 0.0005, 0.0005]), hi - np.array([0, 0.0005, 0.0005]))
        xc, zc = 0.5 * (lo[0] + hi[0]), lo[2] + 0.030      # output shaft (port face): crank + drag link, see the open items
        shaft = G.cylinder(0.006, (xc, lo[1] + 0.001, zc), (xc, lo[1] - 0.003, zc), n=32)
        wall_y = float(self.C.mem["M-KEELWALL"]["box"][0][1])
        flange = _box((lo[0], hi[1] - 0.001, hi[2] - 0.022), (hi[0], wall_y, hi[2] - 0.0005))
        return G.union([body, shaft, flange])


class NoseDoors:
    """Nose clamshell doors 674-R/L (starboard modelled): OML panel between the keel-slot edge and the centre line,
    knuckle tube on the hinge line, pin d 3, three hinge blocks bonded to the keel-wall inner face."""

    def __init__(self, C: Ctx):
        self.C = C
        self.j = C.jl["nose_door_R"]
        self.o = Polygon(C.door["nose_door_R"]["outline"])
        self.t = float(C.door["nose_door_R"]["thickness"])
        from ..core.parts import layup_props
        self.t_skin = layup_props(C.S, "shell_secondary")["thickness"]
        self.O = np.asarray(self.j["origin"], float)
        self.a = _u(self.j["axis"])
        x0, x1 = self.o.bounds[0], self.o.bounds[2]
        # hinge stations only where neither the leg yoke (pivot) nor the tyre passes near the hinge line
        # (layout clearances: doors vs legs 10 mm, nose tyre vs doors 12 mm): forward of the stowed tyre and aft of it
        self.blocks = ((0.700, 0.710), (0.736, 0.746), (x1 - 0.031, x1 - 0.021))
        self.knuck = ((0.7105, 0.7355), (x1 - 0.0205, x1 - 0.0085))
        self.wall_y = float(C.mem["M-KEELWALL"]["box"][0][1])

    def on_axis(self, x: float) -> np.ndarray:
        return self.O + (x - self.O[0]) / self.a[0] * self.a

    def stations(self):
        """(x0, x1) of the two hinge stations (blocks + door knuckles)."""
        return ((self.blocks[0][0], self.blocks[1][1]), (self.blocks[2][0], self.knuck[1][1]))

    def cove(self) -> list:
        return [G.cylinder(0.0042, self.on_axis(xa - 0.0015), self.on_axis(xb + 0.0015), n=32) for xa, xb in self.stations()]

    def block_mesh(self) -> G.Mesh:
        ms = []
        for xa, xb in self.blocks:
            pa, pb = self.on_axis(xa), self.on_axis(xb)
            kn = G.cylinder(0.0035, pa, pb, n=32)
            zt = max(pa[2], pb[2]) + 0.014
            blk = _box((xa, self.O[1] - 0.0015, min(pa[2], pb[2]) - 0.001), (xb, self.wall_y, zt))
            big = rect(xa - 0.01, xb + 0.01, 0.0, 0.09)
            blk = G.difference(blk, [oml_panel(self.C, big, 0.03 + self.t_skin + 0.003, off=-0.03, nx=8, ny=40)])
            ms.append(G.union([blk, kn]))
        bores = [G.cylinder(0.0015 + 0.00002, self.on_axis(xa - 0.001), self.on_axis(xb + 0.001), n=24)
                 for xa, xb in self.stations()]
        return G.difference(G.union(ms), bores)

    def pin(self) -> G.Mesh:
        return G.merge([G.cylinder(0.0015 - 0.00001, self.on_axis(xa + 0.0005), self.on_axis(xb - 0.0005), n=24)
                        for xa, xb in self.stations()])

    def door_closed(self) -> G.Mesh:
        C = self.C
        poly = self.o.buffer(-DOOR_GAP, join_style=2)
        x0, x1 = poly.bounds[0], poly.bounds[2]
        pan = oml_panel(C, poly, self.t, nx=60, ny=16)
        kn = [G.cylinder(0.003, self.on_axis(xa), self.on_axis(xb), n=32) for xa, xb in self.knuck]
        m = G.union([pan] + kn)
        cove = Point(self.O[1], 0.0).buffer(0.0042, 32)
        wall = self.wall_y
        bl = [(xa, xb) for xa, xb in self.blocks]

        st = self.stations()

        def obst(x):
            z_ax = float(self.on_axis(x)[2])
            sk = skin_section(C, x, self.o.bounds[3] + DOOR_GAP, 0.09, self.t_skin)
            if any(xa - 0.0015 <= x <= xb + 0.0015 for xa, xb in st):
                sk = sk.difference(Point(self.O[1], z_ax).buffer(0.0042, 32))
            wl = Polygon([(wall - 0.0005, z_ax + 0.0025), (wall + 0.008, z_ax + 0.0025), (wall + 0.008, 0.0),
                          (wall - 0.0005, 0.0)])
            return unary_union([sk, wl]).buffer(0.0005)
        lo_, hi_ = float(self.j["lo"]), float(self.j["hi"])
        cuts = []
        edges = [x0 - 0.002] + [v for xa, xb in st for v in (xa - 0.0015, xb + 0.0015)] + [x1 + 0.002]
        for k in range(len(edges) - 1):
            xa_, xb_ = edges[k], edges[k + 1]
            if xb_ - xa_ < 1e-4:
                continue
            nn = max(2, int((xb_ - xa_) / 0.03) + 2)
            cuts.append(relief_cutter(np.linspace(xa_ + 1e-5, xb_ - 1e-5, nn), obst, self.O, self.a, lo_, hi_))
        for xa, xb in self.blocks:
            pa = self.on_axis(xa)
            z_b = float(min(pa[2], self.on_axis(xb)[2])) - 0.0015
            sec = unary_union([Polygon([(self.O[1] - 0.0022, z_b), (wall, z_b), (wall, z_b + 0.03),
                                        (self.O[1] - 0.0022, z_b + 0.03)]),
                               Point(self.O[1], float(pa[2])).buffer(0.0042, 32)])
            cuts.append(relief_cutter(np.array([xa - 0.001, xb + 0.001]), lambda x, sec=sec: sec, self.O, self.a,
                                      float(self.j["lo"]), float(self.j["hi"])))
        m = G.difference(m, cuts)
        return _clean(G.difference(m, [G.cylinder(0.0015 + 0.00002, self.on_axis(xa - 0.001), self.on_axis(xb + 0.001),
                                                  n=24) for xa, xb in st]))

    def door(self) -> G.Mesh:
        m = self.door_closed()
        return G.Mesh(_rot(m.V, self.O, self.a, float(self.j["rest"])), m.F)


# =====================================================================================================================
# registration
# =====================================================================================================================
class Build:
    def __init__(self, reg: Registry, spec: dict):
        self.reg = reg
        self.C = Ctx(reg, spec)
        self.mirror_ids: list[str] = []

    def add(self, pid, name, name_tr, material, process, fn, *, group=GROUP, thickness=None, layup=None, parent=None,
            step=STEP_MAIN, explode=(0.0, 0.0, -0.3), contacts=(), joint=None, purchased=False, vendor="",
            mass_kg=None, notes="", mirror=True) -> Part:
        side = "R" if pid.endswith("-R") else ("L" if pid.endswith("-L") else "C")
        p = Part(id=pid, name=name, name_tr=name_tr, group=group, material=material, process=process, mesh_fn=fn,
                 thickness=thickness, layup=layup, parent=parent, step=step, explode=tuple(explode),
                 contacts=tuple(contacts), joint=joint, purchased=purchased, vendor=vendor, mass_kg=mass_kg,
                 side=side, notes=notes)
        self.reg.add(p)
        if mirror and side == "R":
            self.mirror_ids.append(pid)
        return p

    # ---------------------------------------------------------------- joints and the retraction sequence
    def joints(self):
        reg = self.reg
        for j in self.C.L["mechanisms"]["joints"]:
            if j["name"] not in ("main_gear_R", "main_gear_L", "main_inner_door_R", "main_inner_door_L",
                                 "main_trunnion_door_R", "main_trunnion_door_L", "nose_gear", "nose_steer",
                                 "nose_door_R", "nose_door_L"):
                continue
            if j["name"] in reg.joints:
                continue
            reg.add_joint(Joint(name=j["name"], kind=j["kind"], origin=j["origin"], axis=j["axis"],
                                lo=float(j["lo"]), hi=float(j["hi"]), rest=float(j["rest"]), prop=j.get("prop", ""),
                                scale=float(j.get("scale", 1.0)), parent=j.get("parent"), expr=j.get("expr", ""),
                                notes=str(j.get("notes", ""))[:300]))

    def sequence(self, extra=None):
        seq = self.C.L["mechanisms"]["sequences"]["gear_retraction"]
        states = []
        for k, st in enumerate(seq["states"]):
            s2 = {kk: float(v) for kk, v in st.items()}
            if extra:
                s2.update(extra(float(seq["values"][k]), s2))
            states.append(s2)
        self.reg.add_sequence("gear_retraction", states)

    # ---------------------------------------------------------------- main gear (starboard)
    def main_gear(self):
        C = self.C
        M = MainLeg(C)
        self.M = M
        pid = lambda n: C.pid(n, "R")
        jR = "main_gear_R"
        ch_trun, ch_bush, ch_upl = "YK250-CH-070-R", "YK250-CH-074-R", "YK250-CH-072-R"
        wall = self.C.mem["M-WELLWALL-FWD"]["part"]
        ema = C.act["ACT-MLG-EMA"]
        self.add(pid(620), "main leg: yoke + outer cylinder (7075-T651 forging, machined), starboard",
                 "ana bacak: çatal + dış silindir (7075-T651 dövme, işlenmiş), sağ", M7075, P_CNC, M.leg,
                 thickness=C.cyl_t, joint=jR, parent=pid(636), contacts=(pid(636),),
                 notes="telescopic spring/oil damper leg (SAGITTA architecture, components.yaml); OD 42 x 3.5 "
                       "(structures.sizing.gear.main_leg); yoke arms 13 mm (layout F-TRUNNION pivot_scheme arm_m) "
                       "1 mm inside the lug faces; integral up-lock roller lug, leg-door bracket lugs and torque-link "
                       "lug; 3D-forging + 5-axis machining, ISO 2768-m, bores H7, hard anodised")
        self.add(pid(636), "main-leg flanged stub axle pair (4130 N, d 20 x 30), starboard",
                 "ana bacak flanşlı mil çifti (4130 N, d 20 x 30), sağ", M4130, P_CNC, M.stubs,
                 thickness=0.004, joint=jR, parent=ch_bush, contacts=(ch_bush, pid(620)),
                 notes="layout F-TRUNNION pivot_scheme: inserted from inside the yoke outward through the arms "
                       "into the 20 H7 bushings CH-074 (assembly_paths main_stub_axle_*), 3 x M3 12.9 per flange "
                       "into the arm; forward stub carries the EMA output spline (internal)")
        self.add(pid(621), "main-leg sliding tube + axle carrier (7075-T651), starboard",
                 "ana bacak kayar boru + aks taşıyıcı (7075-T651), sağ", M7075, P_CNC, M.carrier,
                 thickness=TUBE_RO - TUBE_RI, joint=jR, parent=pid(620), contacts=(pid(620), pid(634)),
                 notes="piston tube d 34.5 x 2.75 in the 35 bore (0.25 mm radial running clearance on the gland "
                       "bushes), static compression 30 mm of 80 (landing_gear.main), cantilever axle carrier plate "
                       "12 mm inboard of the hub (7 mm to the tyre)")
        self.add(pid(625), "main-leg spring/oil damper cartridge (envelope), starboard",
                 "ana bacak yay/yağ amortisör kartuşu (zarf), sağ", P_BUY, P_BUY, lambda: G.union([
                     G.cylinder(CYL_RI - 0.0002, C.P(CROWN_S[1]), C.P(TUBE_S[0] - 0.0005), n=40),
                     G.cylinder(TUBE_RI - 0.0002, C.P(TUBE_S[0] - 0.0006), C.P(TUBE_S[1] - 0.007), n=40)]),
                 purchased=True, vendor="industrial hydraulic damper + coil spring cartridge (Zimmer Group class, "
                 "SAGITTA practice, tuned to the landing curve; mass estimate)", mass_kg=0.35, joint=jR,
                 parent=pid(620), contacts=(pid(620),),
                 notes="envelope only: stroke 80 mm (landing_gear.main.stroke), mass estimate 0.35 kg")
        self.add(pid(634), "main axle d 20 with nut (4130 N), starboard", "ana aks d 20, somunlu (4130 N), sağ",
                 M4130, P_CNC, M.axle, thickness=0.0045, joint=jR, parent=pid(621), contacts=(pid(621), pid(622),
                                                                                             pid(635)),
                 notes="cantilever axle pressed in the carrier bore (head inboard), keyed torque plate and "
                       "M20x1.5 castellated nut outboard; wheel bearings 20 mm (TOST)")
        self.add(pid(622), "main wheel TOST SB Max II 70-50-20 + tyre 200x50 6 PR + tube, starboard",
                 "ana tekerlek TOST SB Max II 70-50-20 + 200x50 6 PR lastik + iç lastik, sağ", P_BUY, P_BUY, M.wheel,
                 purchased=True, vendor="TOST GmbH: wheel SB Max II 70-50-20 art. 053030 (0.860 kg incl. brake "
                 "disc), tyre 200x50 6 PR art. 063191 (0.450 kg), tube 063192 (0.080 kg)", mass_kg=1.39, joint=jR,
                 parent=pid(634), contacts=(pid(634),),
                 notes="components.yaml tost_sb_max2_70_50_20; static 625 N vs 1960 N tyre capacity")
        self.add(pid(637), "brake disc with hub flange (TOST, part of the SB Max II wheel), starboard",
                 "fren diski, göbek flanşlı (TOST, SB Max II tekerleğin parçası), sağ", P_BUY, P_BUY, M.disc,
                 purchased=True, vendor="TOST SB Max II flanged brake disc 3.5 mm (in the 0.860 kg wheel mass)",
                 mass_kg=0.0, joint=jR, parent=pid(622), contacts=(pid(622),),
                 notes="components.yaml: disc thickness 3.5 mm; the disc mass is in the wheel datasheet mass")
        self.add(pid(635), "brake torque plate (7075-T651), starboard", "fren tork plakası (7075-T651), sağ", M7075,
                 P_CNC, M.torque_plate, thickness=TP_Y[1] - TP_Y[0], joint=jR, parent=pid(634),
                 contacts=(pid(634), pid(623)),
                 notes="keyed (double-D) on the axle end, clamped by the axle nut; carries the caliper (2 x M5)")
        self.add(pid(623), "brake caliper TOST BZT M, starboard", "fren kaliperi TOST BZT M, sağ", P_BUY, P_BUY,
                 M.caliper, purchased=True, vendor="TOST BZT M 3-piston floating caliper, M10x1 port, DOT 4 "
                 "(080110), 0.400 kg, 100 N m", mass_kg=0.40, joint=jR, parent=pid(635), contacts=(pid(635),),
                 notes="components.yaml tost_sb_max2_70_50_20.caliper; hydraulic line from the master cylinder "
                       "EQ-BRAKE_UNIT along the leg (flexible hose, open item)")
        # up-lock roller pair on its pin
        s0, s1 = M.roller_s
        def rollers():
            r1 = G.tube(ROLLER_R, 0.5 * ROLLER_PIN_D + 0.00003, C.P(s0, ROLLER_E), C.P(M.lug_s[0], ROLLER_E), n=32)
            r2 = G.tube(ROLLER_R, 0.5 * ROLLER_PIN_D + 0.00003, C.P(M.lug_s[1], ROLLER_E), C.P(s1, ROLLER_E), n=32)
            return G.merge([r1, r2])
        self.add(pid(639), "up-lock roller pair (needle-bearing track rollers d 14), starboard",
                 "yukarı kilit makara çifti (d 14 iğneli makara), sağ", P_BUY, P_BUY, rollers, purchased=True,
                 vendor="track roller d 14 x 7.5 on a 5 mm pin (catalogue class, mass estimate)", mass_kg=0.012,
                 joint=jR, parent=pid(620), contacts=(pid(620),))
        upl = C.fit["F-UPLOCK"]["part"] + "-R"
        self.add(pid(642), "main up-lock catch (4130 N plates, spring latch + release solenoid of the gear unit), "
                 "starboard", "ana yukarı kilit kancası (4130 N plakalar, yaylı mandal + çözme solenoidi), sağ", M4130,
                 P_CNC, M.hook, thickness=HOOK_Y[1] - HOOK_Y[0], parent=upl, step=STEP_MAIN, explode=(0.0, 0.0, -0.2),
                 contacts=(upl,),
                 notes="layout F-UPLOCK: catch plates either side of the hook boss on its pivot pin; the leg roller "
                       "pair enters the notches from below at the end of the retraction; latch pawl + solenoid "
                       "release are gear-unit items (envelope not modelled, open item)")
        # EMA (layout ACT-MLG-EMA envelope) + reaction flange bolted through the well forward wall
        cyl = ema["cylinder"]
        c0 = np.asarray(cyl["center"], float)
        hl = float(cyl["half_length"])
        rr = float(cyl["radius"]) - 0.0005
        x_wall = float(self.C.mem["M-WELLWALL-FWD"]["box"][1][0])
        self.ema_bolts = []
        for ang in (0.0, 90.0, 135.0):             # up, inboard, inboard-down (clear of the skin and the beam)
            a = math.radians(ang)
            self.ema_bolts.append(np.array([x_wall + 0.006, C.T[1] - 0.038 * math.sin(a), C.T[2] + 0.038 * math.cos(a)]))

        def ema_mesh():
            body = G.cylinder(rr, (c0[0] - hl + 0.0006, c0[1], c0[2]), (C.lug_x[0] - 0.013 - 0.0005, c0[1], c0[2]),
                              n=64)
            spline = G.cylinder(0.0095, (C.lug_x[0] - 0.0135, c0[1], c0[2]), (M.stub_f[0] - 0.0005, c0[1], c0[2]),
                                n=32)
            pts = [G.cylinder(0.031, (x_wall, c0[1], c0[2]), (x_wall + 0.006, c0[1], c0[2]), n=48).V]
            for b in self.ema_bolts:
                pts.append(G.cylinder(0.0105, (x_wall, b[1], b[2]), (x_wall + 0.006, b[1], b[2]), n=32).V)
            fl = G.hull(np.vstack(pts))
            # flat on the outboard-upper side of the housing: clears the heads of the F-TRUNNION beam bolts
            # (layout bolts B1 / B2, heads in the well)
            b2 = next(b for b in C.fit["F-TRUNNION"]["bolts"] if b["id"] == "B2")
            flat = _box((c0[0] - 0.03, float(b2["point"][1]) - 0.015, float(b2["point"][2]) - 0.0115),
                        (c0[0] + 0.03, 0.5, float(b2["point"][2]) + 0.012))
            return G.difference(G.union([body, spline, fl]), [flat])
        self.add(pid(630), "main-gear retraction EMA (BLDC + planetary, integral lock bolt; allocation envelope), "
                 "starboard", "ana takım toplama EMA'sı (BLDC + planet dişli, dahili kilit pimi; yer ayırma zarfı), "
                 "sağ", P_BUY, P_BUY, ema_mesh, purchased=True,
                 vendor="DA-26-class BLDC EMA + planetary stage (layout ACT-MLG-EMA, SAGITTA practice; mass "
                 "estimate 0.27 kg per layout.mass_placement)", mass_kg=0.27, parent=wall, contacts=(wall,),
                 notes="layout ACT-MLG-EMA: d 60 x 40 envelope between the well forward wall and the forward lug; "
                       "output spline into the forward stub axle; reaction flange 6 mm bolted through the 1.6 mm "
                       "well forward wall (3 x M5 12.9); torque requirement structures G-EMA-MAIN")

    def main_doors(self):
        C, M = self.C, self.M
        Dr = Doors(C, M)
        self.Dr = Dr
        pid = lambda n: C.pid(n, "R")
        from ..core.parts import layup_props
        areal = layup_props(C.S, "shell_secondary")["areal_mass"]
        wk = C.mem["M-WELLKEEL"]["part"] + "-R"
        door_note = ("CFRP sandwich door (layout.shell.rules layups: shell_secondary skins), modelled 4 mm thick "
                     "(layout door_outlines thickness); mass = area x the shell_secondary areal mass; perimeter seal "
                     "on the opening land (layout.shell.rules.seals)")
        self.add(pid(671), "main inner-door fixed hinge bracket (7075-T651, 3 ears), starboard",
                 "ana iç kapak sabit menteşe braketi (7075-T651, 3 kulak), sağ", M7075, P_CNC, Dr.fixed_hinge,
                 thickness=0.003, parent=wk, contacts=(wk,), step=STEP_DOOR, explode=(0.0, 0.0, -0.15),
                 notes="layout M-WELLKEEL: bolted to the outboard face of the well keel web (2 x M4, nuts in the "
                       "harness channel); ears 4 mm round the layout hinge line where it lies in the door laminate")
        self.add(pid(670), "main inner door (CFRP sandwich), starboard", "ana iç kapak (CFRP sandviç), sağ", MPW,
                 P_PREG, Dr.inner_door, thickness=Dr.t, joint="main_inner_door_R", parent=pid(671),
                 contacts=(pid(677),), step=STEP_DOOR, explode=(0.0, 0.05, -0.25),
                 mass_kg=Dr.area(Dr.o_in) * areal, notes=door_note + "; two bonded 7075 knuckles on the hinge line, "
                 "hinge edge relieved where it would sweep into the centre strip")
        self.add(pid(677), "main inner-door hinge pin d 3 (ISO 2338 h8, 4130) + retaining rings, starboard",
                 "ana iç kapak menteşe pimi d 3 (ISO 2338 h8, 4130) + segmanlar, sağ", P_BUY, P_BUY, Dr.inner_pin,
                 purchased=True, vendor="ISO 2338 h8 dowel d 3 x 37 + 2 x DIN 6799 retaining rings (catalogue)",
                 mass_kg=0.003, parent=pid(671), contacts=(pid(671), pid(670)), step=STEP_DOOR)
        self.add(pid(680), "leg-door standoff brackets (7075-T651, pair), starboard",
                 "bacak kapağı mesafe braketleri (7075-T651, çift), sağ", M7075, P_CNC, Dr.leg_brackets,
                 thickness=0.004, joint="main_gear_R", parent=pid(620), contacts=(pid(620), pid(672)),
                 step=STEP_MAIN, notes="mass.rules.gear_doors.leg_door_brackets; bolted 2 x M4 each to the leg lugs, "
                                       "foot bonded to the door")
        self.add(pid(672), "main leg door (CFRP sandwich, carried by the leg), starboard",
                 "ana bacak kapağı (CFRP sandviç, bacakla hareket eder), sağ", MPW, P_PREG, Dr.leg_door,
                 thickness=Dr.t, joint="main_gear_R", parent=pid(680), contacts=(pid(680),), step=STEP_MAIN,
                 mass_kg=Dr.area(Dr.o_leg) * areal, notes=door_note + "; closed (flush) when the gear is up "
                 "(layout door_outlines main_leg_door_R closed_at hi)")
        self.add(pid(673), "main trunnion door (CFRP sandwich), starboard", "ana mafsal kapağı (CFRP sandviç), sağ",
                 MPW, P_PREG, Dr.tr_door, thickness=Dr.t, joint="main_trunnion_door_R",
                 parent=C.pan["P-CENTRE-LOWER"]["part"], step=STEP_DOOR, explode=(0.0, 0.1, -0.2),
                 mass_kg=Dr.area(Dr.o_tr) * areal, notes=door_note + "; opening spring on the hinge pin (torsion "
                 "spring, gear-unit kit)")
        skin = C.pan["P-CENTRE-LOWER"]["part"]
        self.add(pid(681), "trunnion-door hinge end blocks (7075-T651, pair), starboard",
                 "mafsal kapağı menteşe uç blokları (7075-T651, çift), sağ", M7075, P_CNC, lambda: Dr.tr_blocks()[0],
                 thickness=0.003, parent=skin, contacts=(skin,), step=STEP_DOOR, explode=(0.0, 0.05, -0.15),
                 notes="bonded (EA 9394, 0.1 mm) on the inner face of the lower skin either side of the leg slot, "
                       "inside the notch of the gear beam; knuckles in a 4.2 mm cove of the skin edge")
        self.add(pid(682), "trunnion-door hinge pin d 3 (ISO 2338 h8, 4130) + retaining rings, starboard",
                 "mafsal kapağı menteşe pimi d 3 (ISO 2338 h8, 4130) + segmanlar, sağ", P_BUY, P_BUY, Dr.tr_pin,
                 purchased=True, vendor="ISO 2338 h8 dowel d 3 x 82 + 2 x DIN 6799 retaining rings (catalogue)",
                 mass_kg=0.005, parent=pid(681), contacts=(pid(681), pid(673)), step=STEP_DOOR)
        self.reg.parts[pid(673)].parent = pid(682)
        self.reg.parts[pid(673)].contacts = (pid(682),)

    # ---------------------------------------------------------------- nose gear (centre line + R/L pairs)
    def nose_gear(self):
        C = self.C
        N = NoseLeg(C)
        self.N = N
        ND = NoseDoors(C)
        self.ND = ND
        jn, js = "nose_gear", "nose_steer"
        lg = lambda n, side="C": C.pid(n, side)
        bush_r, bush_l = "YK250-CH-073-R", "YK250-CH-073-L"
        kw = C.mem["M-KEELWALL"]["part"]
        S = STEP_NOSE
        ex = (0.0, 0.0, -0.35)
        self.add(lg(624), "nose leg: yoke + outer cylinder + retraction horn (7075-T651 forging, machined)",
                 "burun bacağı: çatal + dış silindir + toplama kolu (7075-T651 dövme, işlenmiş)", M7075, P_CNC, N.leg,
                 thickness=N.ro - N.ri, joint=jn, parent=lg(640), contacts=(lg(640),), step=S, explode=ex,
                 notes="OD 36 x 3 (structures.sizing.gear.nose_leg); yoke arms 9 mm (layout F-NG-PIVOT arm_m); "
                       "horn of the over-centre drag link on the port arm; steering-actuator lugs aft")
        self.add(lg(640), "nose-leg flanged stub axle pair (4130 N, d 16 x 22)", "burun bacağı flanşlı mil çifti (4130 N, "
                 "d 16 x 22)", M4130, P_CNC, N.stubs, thickness=0.004, joint=jn, parent=bush_r,
                 contacts=(bush_r, bush_l, lg(624)), step=S, explode=ex,
                 notes="layout F-NG-PIVOT pivot_scheme: inserted outward from inside the yoke into the 16 H7 bushings "
                       "CH-073 (assembly_paths nose_stub_axle_R/L); the layout's 3 x M4 flange screws do not fit the "
                       "9 mm arm inside the heads of the F-NG-PIVOT bolts: flange keyed to the arm by a 3 mm dowel, "
                       "axial retention by the yoke (open item for the gear-unit supplier)")
        self.add(lg(633), "nose-leg spring/oil damper cartridge (envelope)", "burun bacağı yay/yağ amortisör kartuşu "
                 "(zarf)", P_BUY, P_BUY, N.damper, purchased=True, vendor="industrial hydraulic damper + coil spring "
                 "cartridge (SAGITTA practice, spring rate 3.3 kN/m; mass estimate)", mass_kg=0.25, joint=jn,
                 parent=lg(624), contacts=(lg(624),), step=S, explode=ex, notes="stroke 70 mm (landing_gear.nose.stroke)")
        self.add(lg(631), "nose steering tube + trailing fork (7075-T651)", "burun yönlendirme borusu + çatal "
                 "(7075-T651)", M7075, P_CNC, N.fork, thickness=0.0025, joint=js, parent=lg(624),
                 contacts=(lg(624), lg(648)), step=S, explode=ex,
                 notes="trail 30 mm (landing_gear.nose.trail); turns +-20 deg in the cylinder bore on two "
                       "bushes; fork arms 6 mm beside the 50 mm hub")
        self.add(lg(648), "nose axle d 20 with nuts (4130 N)", "burun aksı d 20, somunlu (4130 N)", M4130, P_CNC, N.axle,
                 thickness=AXLE_R - 0.006, joint=js, parent=lg(631), contacts=(lg(631), lg(649)), step=S, explode=ex)
        self.add(lg(649), "nose-wheel hub spacers (7075-T651, pair)", "burun tekerlek göbek araları (7075-T651, çift)",
                 M7075, P_CNC, N.spacers, thickness=0.0045, joint=js, parent=lg(648), contacts=(lg(648), lg(626), lg(631)),
                 step=S, explode=ex)
        self.add(lg(626), "nose wheel TOST LR Max II + tyre 200x50 6 PR + tube", "burun tekerleği TOST LR Max II + "
                 "200x50 6 PR lastik + iç lastik", P_BUY, P_BUY, N.wheel, purchased=True,
                 vendor="TOST GmbH: plain wheel LR Max II art. 033120 (0.365 kg), tyre 200x50 6 PR art. 063191 "
                        "(0.450 kg), tube (0.080 kg)", mass_kg=0.895, joint=js, parent=lg(648), contacts=(lg(648),),
                 step=S, explode=ex, notes="components.yaml tost_lr_max2_nose")
        self.add(lg(644), "nose steering collar + belt pulley (7075-T651)", "burun yönlendirme bileziği + kayış kasnağı "
                 "(7075-T651)", M7075, P_CNC, N.collar, thickness=0.0045, joint=js, parent=lg(631),
                 contacts=(lg(631),), step=S, explode=ex, notes="clamped and keyed on the steering tube")
        self.add(lg(646), "nose shimmy damper (friction disc pack, spring-loaded)", "burun shimmy sönümleyicisi (yaylı "
                 "sürtünme disk paketi)", P_BUY, P_BUY, N.shimmy, purchased=True,
                 vendor="friction shimmy damper disc pack (sailplane / LSA practice; mass estimate)", mass_kg=0.02,
                 joint=jn, parent=lg(624), contacts=(lg(624), lg(644)), step=S, explode=ex)
        self.add(C.pid(628, "C", "FC"), "nose-wheel steering actuator Volz DA 26 + right-angle gearhead and pulley",
                 "burun tekerleği yönlendirme eyleyicisi Volz DA 26 + açılı dişli kutusu ve kasnak", P_BUY, P_BUY,
                 N.steer_act, group="controls", purchased=True,
                 vendor="Volz DA 26-30-5024 (0.270 kg datasheet) + 90 deg gearhead and HTD pulley (estimate)",
                 mass_kg=0.27, joint=jn, parent=lg(624), contacts=(lg(624),), step=S, explode=ex,
                 notes="layout ACT-STEER: case on the aft face of the leg; toothed belt to the steering collar "
                       "(SAGITTA practice); centred before retraction")
        self.add(lg(645), "nose steering toothed belt (HTD 3M, 9 mm)", "burun yönlendirme dişli kayışı (HTD 3M, 9 mm)",
                 P_BUY, P_BUY, N.belt, purchased=True, vendor="HTD 3M-9 closed toothed belt (catalogue)", mass_kg=0.01,
                 joint=jn, parent=C.pid(628, "C", "FC"), contacts=(C.pid(628, "C", "FC"), lg(644)), step=S,
                 explode=ex)
        self.add(lg(632), "nose-gear retraction EMA (BLDC + planetary; allocation envelope)",
                 "burun takımı toplama EMA'sı (BLDC + planet dişli; yer ayırma zarfı)", P_BUY, P_BUY, N.ema,
                 purchased=True, vendor="DA-26-class BLDC EMA + planetary stage (layout ACT-NLG-EMA, mass estimate "
                 "0.27 kg per layout.mass_placement)", mass_kg=0.27, parent=kw + "-R", contacts=(kw + "-R",), step=S,
                 explode=(0.0, 0.0, 0.2), notes="layout ACT-NLG-EMA: between the keel walls above the pivot, flange "
                 "bolted to the starboard keel-wall land (2 x M5); output shaft to the port side crank")
        self.add(lg(647), "nose up-lock rollers on the axle ends (pair)", "burun yukarı kilit makaraları (aks uçlarında, "
                 "çift)", P_BUY, P_BUY, N.rollers, purchased=True, vendor="plain bearing rollers d 12 x 1.5 (catalogue "
                 "class, mass estimate)", mass_kg=0.004, joint=js, parent=lg(648), contacts=(lg(648),), step=S,
                 explode=ex)
        self.add(lg(650, "R"), "nose up-lock catch plate (4130 N), starboard", "burun yukarı kilit mandal plakası "
                 "(4130 N), sağ", M4130, P_CNC, N.catch, thickness=0.0023, parent=kw + "-R", contacts=(kw + "-R",),
                 step=S, explode=(0.0, 0.05, 0.0),
                 notes="layout F-NG-PIVOT locks: up-lock engaging the axle roller (here on the keel-wall face beside "
                       "the stowed axle; bonded + latch pawl and release solenoid of the gear unit, open item)")
        # nose doors
        from ..core.parts import layup_props
        areal = layup_props(C.S, "shell_secondary")["areal_mass"]
        self.add(lg(683, "R"), "nose-door hinge blocks (7075-T651, 3 off), starboard",
                 "burun kapağı menteşe blokları (7075-T651, 3 adet), sağ", M7075, P_CNC, ND.block_mesh, thickness=0.003,
                 parent=kw + "-R", contacts=(kw + "-R",), step=STEP_DOOR, explode=(0.0, 0.05, -0.15),
                 notes="bonded (EA 9394) to the inner face of the keel wall at the slot edge; knuckles in a 4.2 mm "
                       "cove of the lower nose skin edge")
        self.add(lg(684, "R"), "nose-door hinge pin d 3 (ISO 2338 h8, 4130), starboard",
                 "burun kapağı menteşe pimi d 3 (ISO 2338 h8, 4130), sağ", P_BUY, P_BUY, ND.pin, purchased=True,
                 vendor="ISO 2338 h8 dowel d 3 x 450 + retaining rings (catalogue)", mass_kg=0.025,
                 parent=lg(683, "R"), contacts=(lg(683, "R"), lg(674, "R")), step=STEP_DOOR)
        self.add(lg(674, "R"), "nose clamshell door (CFRP sandwich), starboard", "burun istiridye kapağı (CFRP sandviç), "
                 "sağ", MPW, P_PREG, ND.door, thickness=ND.t, joint="nose_door_R", parent=lg(684, "R"),
                 contacts=(lg(684, "R"),), step=STEP_DOOR, explode=(0.0, 0.08, -0.2),
                 mass_kg=float(ND.o.buffer(-DOOR_GAP, join_style=2).area) * areal,
                 notes="layout door_outlines nose_door_R; modelled open (rest = gear down); knuckle tube on the hinge "
                       "line; drive horn on the aft pin extension (NDOOR-LINKAGE, see open items)")

    def nose_fasteners(self):
        C, N = self.C, self.N
        reg = self.reg
        lg = lambda n, side="C": C.pid(n, side)
        fc = C.pid(628, "C", "FC")
        for k, (p, ax) in enumerate(N.steer_bolts()):
            J.bolt(reg, f"{fc}-B{k + 1}", 3, p, ax, [(fc, 0.026)], nut="tapped", tapped_part=lg(624),
                   tapped_depth=0.010, grade="12.9", step=STEP_NOSE)
        kw = C.mem["M-KEELWALL"]["part"] + "-R"
        a = C.act["ACT-NLG-EMA"]
        hi = np.asarray(a["box"][1], float)
        lo = np.asarray(a["box"][0], float)
        for k, xb in enumerate((lo[0] + 0.012, hi[0] - 0.012)):
            J.bolt_through(reg, f"{lg(632)}-B{k + 1}", 5, (xb, hi[1], hi[2] - 0.0115), (0.0, 1.0, 0.0), [lg(632), kw],
                           grade="12.9", step=STEP_NOSE, notes="EMA flange through the keel-wall land, nut in the "
                                                               "starboard side bay")
        sk = reg.parts[C.pan["P-NOSE-LOWER"]["part"]]
        for cv in self.ND.cove():
            sk.holes += [cv, cv.mirrored_y()]

    # ---------------------------------------------------------------- door actuators (DA 22) on their frames
    def door_actuators(self):
        C = self.C
        eq = C.eq["EQ-DOORACT"]
        lo, hi = np.asarray(eq["box"][0], float), np.asarray(eq["box"][1], float)
        fr = C.st["FS-GEAR"]
        xf = float(fr["x_faces"][1])
        yc = 0.5 * (lo[1] + hi[1])
        ears = ((lo[2] - 0.0115, lo[2] + 0.0015), (hi[2] - 0.0015, hi[2] + 0.0115))
        self.da_main = dict(lo=lo, hi=hi, xf=xf, yc=yc, ears=ears)

        def da22():
            case = _box(lo, hi)
            e = [_box((lo[0], lo[1], za), (lo[0] + 0.003, hi[1], zb)) for za, zb in ears]
            shaft = G.cylinder(0.004, (hi[0] - 0.012, lo[1] + 0.0005, lo[2] + 0.012), (hi[0] - 0.012, lo[1] - 0.004,
                                                                                       lo[2] + 0.012), n=24)
            return G.union([case, shaft] + e)

        def plate():                        # top edge below the TR-AFTBAY tray flange on the same frame face
            return _box((xf, lo[1] - 0.002, ears[0][0] - 0.002), (lo[0], hi[1] + 0.002, ears[1][1]))
        frame = fr["part"]
        self.add(C.pid(685, "R"), "inner-door actuator mounting plate (7075-T651), starboard",
                 "iç kapak eyleyicisi bağlantı plakası (7075-T651), sağ", M7075, P_CNC, plate, thickness=lo[0] - xf,
                 parent=frame, contacts=(frame,), step=STEP_DOOR, explode=(0.1, 0.0, 0.0),
                 notes="layout EQ-DOORACT: on the aft face of FS-GEAR (potted inserts, the frame is the vapour-tight "
                       "aft fuel-bay bulkhead: no through-bolts)")
        self.add(C.pid(676, "R", "FC"), "main inner-door actuator Volz DA 22 (28 V), starboard",
                 "ana iç kapak eyleyicisi Volz DA 22 (28 V), sağ", P_BUY, P_BUY, da22, purchased=True,
                 vendor="Volz DA 22-30-4128 (28 V, long case) 41.5 x 65.9 x 22 mm, 0.132 kg (datasheet)",
                 mass_kg=0.132, parent=C.pid(685, "R"), contacts=(C.pid(685, "R"),), step=STEP_DOOR,
                 explode=(0.12, 0.0, 0.0),
                 notes="layout EQ-DOORACT; crank + pushrod through C-DOORLINK to the inner-door horn: see the open "
                       "items (the 35 x 18 mm frame window does not pass a pushrod for the 90 deg door travel)")
        eqn = C.eq["EQ-NDOORACT"]
        lo2, hi2 = np.asarray(eqn["box"][0], float), np.asarray(eqn["box"][1], float)
        tray = eqn["mount"]["tray"]
        # the case passes through the cut-out of the side-bay tray; two ears rest on the tray top (z -0.089)
        z_tr = -0.089
        ears2 = ((lo2[1] - 0.0245, lo2[1] + 0.0015), (hi2[1] - 0.0015, hi2[1] + 0.0245))
        bolt_y = (lo2[1] - 0.0135, hi2[1] + 0.0135)
        xm2 = 0.5 * (lo2[0] + hi2[0])
        self.da_nose = dict(lo=lo2, hi=hi2, xm=xm2, ears=ears2, tray=tray, z_tr=z_tr, bolt_y=bolt_y)

        def da22n():
            case = _box(lo2, hi2)
            e = [_box((xm2 - 0.0105, ya, z_tr), (xm2 + 0.0105, yb, z_tr + 0.003)) for ya, yb in ears2]
            return G.union([case] + e)
        self.add(C.pid(678), "nose-door actuator Volz DA 22 (28 V), drives both clamshell doors",
                 "burun kapağı eyleyicisi Volz DA 22 (28 V), iki kapağı tahrik eder", P_BUY, P_BUY, da22n, purchased=True,
                 vendor="Volz DA 22-30-4128 (28 V, long case), 0.132 kg (datasheet)", mass_kg=0.132,
                 parent=tray, contacts=(tray,), step=STEP_DOOR, explode=(-0.12, 0.0, 0.1),
                 notes="layout EQ-NDOORACT (port side bay at FS1110), through the cut-out of the side-bay tray "
                       "TR-SIDEBAY-L, two ears on the tray (2 x M4 + nutplates); crank, pushrod, centre-line bellcrank and door links (NDOOR-LINKAGE): see "
                       "the open items")

    def door_actuator_fasteners(self, side: str):
        C, reg = self.C, self.reg
        d = self.da_main
        s = 1.0 if side == "R" else -1.0
        fr = C.st["FS-GEAR"]["part"]
        for k, (za, zb) in enumerate(d["ears"]):
            zc = 0.5 * (za + zb) + (-0.0005 if k == 0 else 0.0005)
            J.bolt(reg, f"{C.pid(676, side, 'FC')}-B{k + 1}", 3, (d["lo"][0] + 0.003, s * d["yc"], zc), (-1.0, 0.0, 0.0),
                   [(C.pid(676, side, "FC"), 0.003), (C.pid(685, side), d["lo"][0] - d["xf"])], nut="insert",
                   insert_part=fr, insert_depth=0.0068, grade="12.9", step=STEP_DOOR)
        if side == "R":
            n = self.da_nose
            for k, yb_ in enumerate(n["bolt_y"]):
                J.bolt_through(reg, f"{C.pid(678)}-B{k + 1}", 4, (n["xm"], yb_, n["z_tr"] + 0.003),
                               (0.0, 0.0, -1.0), [C.pid(678), n["tray"]], nut="nutplate", grade="12.9", step=STEP_DOOR,
                               notes="nutplate under the side-bay tray")

    def main_fasteners(self, side: str):
        """Fasteners of one main gear (built on each side's own parts so every hole is drilled)."""
        C, M = self.C, self.M
        s = 1.0 if side == "R" else -1.0
        mv = lambda v: np.asarray(v, float) * np.array([1.0, s, 1.0])
        pid = lambda n: C.pid(n, side)
        reg = self.reg
        wall = C.mem["M-WELLWALL-FWD"]["part"]
        for k, (p, ax) in enumerate(M.stub_bolts()):
            J.bolt(reg, f"{pid(636)}-B{k + 1}", 3, mv(p), mv(ax), [(pid(636), STUB_FL_T)], nut="tapped",
                   tapped_part=pid(620), tapped_depth=0.010, grade="12.9", head="ISO 4762", step=STEP_MAIN,
                   notes="stub-axle flange to the yoke arm, Loctite 243")
        for k, b in enumerate(self.ema_bolts):
            J.bolt_through(reg, f"{pid(630)}-B{k + 1}", 5, mv(b), (-1.0, 0.0, 0.0), [pid(630), wall], grade="12.9",
                           washer_head=True, step=STEP_MAIN, notes="EMA reaction flange through the well forward wall")
        for k, (p, ax) in enumerate(M.caliper_bolts()):
            J.bolt(reg, f"{pid(635)}-B{k + 1}", 5, mv(p), mv(ax), [(pid(635), TP_Y[1] - TP_Y[0])], nut="tapped",
                   tapped_part=pid(623), tapped_depth=0.011, grade="12.9", step=STEP_MAIN)
        for k, (p, ax) in enumerate(self.Dr.bracket_bolts()):
            J.rivet(reg, f"{pid(680)}-R{k + 1}", 0.0032, mv(p), mv(ax), [(pid(680), 0.004), (pid(620), 0.006)],
                    spec="blind rivet NAS9301B-4 (CherryMAX, A286 stem)", step=STEP_MAIN,
                    notes="leg-door bracket to the leg lug (heads inside the leg-slot width)")
        wk = C.mem["M-WELLKEEL"]["part"] + f"-{side}"
        zb = float(C.mem["M-WELLKEEL"]["box"][0][2])
        yw1 = float(C.mem["M-WELLKEEL"]["box"][1][1])
        for k, xb in enumerate((self.Dr.ears[0][0] + 0.008, self.Dr.ears[-1][1] - 0.008)):
            J.bolt_through(reg, f"{pid(671)}-B{k + 1}", 4, mv((xb, yw1 + 0.002, zb + 0.016)), mv((0.0, -1.0, 0.0)),
                           [pid(671), wk], grade="12.9", step=STEP_DOOR, notes="nut in the harness channel")
        sk = reg.parts[C.pan["P-CENTRE-LOWER"]["part"]]
        for cv in [self.Dr.cove()] + self.Dr.tr_notch():
            sk.holes.append(cv if side == "R" else cv.mirrored_y())
        s0, s1 = M.roller_s
        pin_through(reg, f"{pid(620)}-P9", ROLLER_PIN_D, mv(C.P(s0 - 0.001, ROLLER_E)),
                    mv(C.P(s1 + 0.001, ROLLER_E)), (pid(620),), step=STEP_MAIN,
                    notes="roller pin through the leg lug; rollers ride on it against the lug faces")
        pv = M.hook_pivot()
        pin_through(reg, f"{pid(642)}-P1", HOOK_PIN_D, mv((pv[0], HOOK_Y[0] - 0.001, pv[2])),
                    mv((pv[0], HOOK_Y[3] + 0.001, pv[2])), (pid(642),), step=STEP_MAIN,
                    notes="catch pivot pin through the F-UPLOCK boss bore (d 5.2); the boss gives 6 mm edge "
                          "distance (1.2 D): chassis open item, the catch plates bear on the boss faces")

    # ---------------------------------------------------------------- mirror
    def mirror(self):
        reg = self.reg
        ids = list(self.mirror_ids)
        id_map = {p: p[:-1] + "L" for p in ids}
        for pid in ids:                          # starboard neighbours of other modules -> their port twins
            p = reg.parts[pid]
            for q in list(p.contacts) + [p.parent]:
                if q and q.endswith("-R") and q[:-1] + "L" in reg.parts:
                    id_map.setdefault(q, q[:-1] + "L")
        for pid in ids:
            p = reg.parts[pid]
            jn = p.joint
            jl = None if jn is None else (jn[:-1] + "L" if jn.endswith("_R") else jn)
            q = mirror_part(p, id_map[pid], joint=jl, id_map=id_map)
            q.fasteners = []
            q.holes = []
            reg.add(q)


def pin_through(reg: Registry, fid: str, d: float, p0, p1, part_ids, *, step: int = 0, fit: float = 0.00005,
                spec: str = "ISO 2341-B", notes: str = "") -> Fastener:
    """Clevis pin from p0 to p1 (head at p0) through every part in ``part_ids`` (bores d + fit), for pin joints whose
    parts are not a solid stack (rollers, link eyes side by side)."""
    p0, p1 = np.asarray(p0, float), np.asarray(p1, float)
    a = _u(p1 - p0)
    L = float(np.linalg.norm(p1 - p0))
    for pid in part_ids:
        reg.parts[pid].add_hole(p0 - 0.001 * a, p1 + 0.001 * a, 0.5 * d + 0.5 * fit)
    f = Fastener(id=fid, spec=f"{spec} {d * 1000:g}x{J.std_length(L, J.PIN_LENGTH_MM) * 1000:.0f}", kind="clevis_pin",
                 d=d, length=L, position=p0, axis=a, joins=tuple(part_ids), nut="ISO 1234 split pin + ISO 7089 washer",
                 step=step, notes=notes, grip=L)
    reg.parts[part_ids[0]].fasteners.append(f)
    return f


def register(reg: Registry, spec: dict) -> None:
    if any(p.startswith("YK250-LG-620") for p in reg.parts):
        reg.note("gear: already registered, second call ignored")
        return
    B = Build(reg, spec)
    B.joints()
    B.main_gear()
    B.main_doors()
    B.nose_gear()
    B.door_actuators()
    B.mirror()
    for side in ("R", "L"):
        B.main_fasteners(side)
        B.door_actuator_fasteners(side)
    B.nose_fasteners()
    B.sequence()
