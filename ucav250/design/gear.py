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
TL_PIN_D = 0.004            # torque-link pins
TL_UP = (0.096, -0.032)     # torque-link pins in the leg plane (s, x): upper (cylinder lug), lower (carrier lug), apex
TL_LO = (0.150, -0.026)
TL_AP = (0.123, -0.050)
TL_E = ((-0.016, -0.010), (-0.009, -0.003))  # e ranges: upper link / lower lug, lower link / upper lug
LINK_T = 0.006              # torque links
NOSE_ARM_T = 0.009          # nose yoke arm = layout F-NG-PIVOT stub_axle arm_m
DOOR_T = 0.004              # gear doors (layout door_outlines thickness)
DOOR_GAP = 0.0005           # half of the 1.0 mm panel gap (layout.shell.rules.joggles)


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


def _pieces_largest(m: G.Mesh) -> G.Mesh:
    return m


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

    def oml_z(self, x: float, y: float) -> float:
        """Lower OML z at (x, y) (bisection on the section's lower half)."""
        lo, hi = math.pi / 2, math.pi
        for _ in range(60):
            mid = 0.5 * (lo + hi)
            p = self.fus.point(x, mid)
            if abs(float(p[1])) > abs(y):
                lo = mid
            else:
                hi = mid
        return float(self.fus.point(x, 0.5 * (lo + hi))[2])

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
        return ((T[0] - 0.0255, T[0] - 0.0195), (T[0] + 0.0195, T[0] + 0.0255))

    def door_brk_s(self):
        return (0.060, 0.090)

    def tl_up_s(self) -> float:
        return 0.096

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
        return wheel_mesh(self.C, self.C.A, np.array([0.0, 1.0, 0.0]), disc=True)

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

    # ---------------------------------------------------------------- torque links (637 upper, 638 lower)
    def tl_points(self):
        C = self.C
        return C.P(TL_UP[0], 0.0, TL_UP[1]), C.P(TL_LO[0], 0.0, TL_LO[1]), C.P(TL_AP[0], 0.0, TL_AP[1])

    def tlinks(self):
        C = self.C
        up, lo, apex = self.tl_points()
        r_e = 2.1 * TL_PIN_D

        def link(p, q, e_rng):
            u = _u(q - p)
            v = np.cross(C.e, u)
            L = float(np.linalg.norm(q - p))
            poly = unary_union([Point(0, 0).buffer(r_e, 32), Point(L, 0).buffer(r_e, 32),
                                Polygon([(0, -0.006), (L, -0.006), (L, 0.006), (0, 0.006)])])
            return _plate(poly, p, u, v, e_rng[0], e_rng[1])
        return link(up, apex, TL_E[0]), link(lo, apex, TL_E[1])

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
    ys = np.linspace(y0 - pad, y1 + pad, ny)
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


def rect(x0, x1, y0, y1) -> Polygon:
    return Polygon([(x0, y0), (x1, y0), (x1, y1), (x0, y1)])


def swept_relief(m: G.Mesh, origin, axis, v0: float, v1: float, obstacles, n: int = 19) -> G.Mesh:
    """``m`` (closed pose) minus every region that would meet an obstacle while the part turns about the axis by
    v0..v1 (relative to the closed pose)."""
    cuts = []
    for v in np.linspace(v0, v1, n):
        for o in obstacles:
            cuts.append(G.Mesh(_rot(o.V, origin, axis, -v), o.F))
    return G.difference(m, cuts)


class Doors:
    """Main-gear doors (starboard): inner door 670, leg door 672 (+ brackets), trunnion door 673; hinges."""

    def __init__(self, C: Ctx, M: "MainLeg"):
        self.C, self.M = C, M
        D = C.door
        self.o_in = Polygon(D["main_inner_door_R"]["outline"])
        self.o_leg = Polygon(D["main_leg_door_R"]["outline"])
        self.o_tr = Polygon(D["main_trunnion_door_R"]["outline"])
        self.t = float(D["main_inner_door_R"]["thickness"])
        lay = C.S["layups"]["shell_secondary"]
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
    def strip_obstacle(self):
        """Centre strip of the lower skin between the two inner doors (layout outline edge y 0.01 - panel gap),
        layup shell_secondary thick, with the hinge coves cut: obstacle of the swept relief."""
        C = self.C
        x0, y0, x1, y1 = self.o_in.bounds
        band = oml_panel(C, rect(x0 - 0.01, x1 + 0.01, -(y0 - DOOR_GAP), y0 - DOOR_GAP), self.t_skin + 0.0005,
                         off=-0.0005, nx=30, ny=8)
        return G.difference(band, [self.cove()])

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
        base = _box((self.ears[0][0], yw1, zb + 0.006), (self.ears[-1][1], yw1 + 0.003, zb + 0.022))
        return G.union(ears + [base])

    def inner_door(self) -> G.Mesh:
        C = self.C
        o = np.asarray(self.ji["origin"], float)
        a = np.asarray(self.ji["axis"], float)
        pan = oml_panel(C, self.door_poly(self.o_in), self.t)
        kn = [G.cylinder(0.0035, (xa, o[1], o[2]), (xb, o[1], o[2]), n=32) for xa, xb in self.knuck]
        m = G.union([pan] + kn)
        obst = [self.strip_obstacle()]
        ear_env = [_plate(self.ear_polys().buffer(0.0007), (xa - 0.0007, 0.0, 0.0), (0, 1, 0), (0, 0, 1), 0.0,
                          xb - xa + 0.0014) for xa, xb in self.ears]
        m = swept_relief(m, o, a, float(self.ji["lo"]), float(self.ji["hi"]), obst + ear_env)
        bore = G.cylinder(0.0015 + 0.00002, (self.h_in[0] - 0.001, o[1], o[2]), (self.h_in[1] + 0.001, o[1], o[2]), n=24)
        return G.difference(m, [bore])

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
            lugc = C.stow(np.array([C.P(s_, C.cyl_ro + 0.005, 0.0) for s_ in M.door_brk_s()]))
            ya, yb = min(p[1] for p in lugc) - 0.009, max(p[1] for p in lugc) + 0.009
            zt = max(p[2] for p in lugc) + 0.010
            blk = _box((xb[0], ya, -0.25), (xb[1], yb, zt))
            foot = _box((xb[0], ya, -0.25), (xb[1] + 0.008, yb, -0.185)) if fwd else \
                _box((xb[0] - 0.008, ya, -0.25), (xb[1], yb, -0.185))
            ms.append(G.union([blk, foot]))
        m = G.union(ms)
        big = self.o_leg.buffer(0.03, join_style=2)
        m = G.difference(m, [oml_panel(C, big, self.t, off=0.0), oml_panel(C, big, 0.2, off=-0.2)])
        return G.Mesh(C.unstow(m.V), m.F)

    def bracket_bolts(self):
        """(head point, axis) of the 2 x M4 per bracket through bracket + leg lug (rest pose), head outside."""
        C, M = self.C, self.M
        out = []
        for (x0, x1) in M.door_lug_x():
            fwd = x0 < C.T[0]
            for s_ in M.door_brk_s():
                p = C.P(s_, C.cyl_ro + 0.005, 0.0)
                p[0] = x0 - 0.004 if fwd else x1 + 0.004
                out.append((p, np.array([1.0 if fwd else -1.0, 0.0, 0.0])))
        return out

    # ---------------------------------------------------------------- trunnion door 673 (closed at lo, rest = hi)
    def tr_obstacles(self):
        C = self.C
        x0, y0, x1, y1 = self.o_tr.bounds
        skin = oml_panel(C, rect(x0 - 0.012, x1 + 0.012, y1 + DOOR_GAP, y1 + 0.03), self.t_skin + 0.0005, off=-0.0005,
                         nx=24, ny=12)
        beam = [_box(np.asarray(b[0], float) - 0.0005, np.asarray(b[1], float) + 0.0005)
                for b in C.mem["M-GEARBEAM"]["boxes"]]
        return [skin] + beam

    def tr_door_closed(self) -> G.Mesh:
        C = self.C
        jt = self.jt
        o = np.asarray(jt["origin"], float)
        a = np.asarray(jt["axis"], float)
        x0, y0, x1, y1 = self.door_poly(self.o_tr).bounds
        pan = oml_panel(C, self.door_poly(self.o_tr), self.t)
        kn = G.cylinder(0.003, (x0, o[1], o[2]), (x1, o[1], o[2]), n=32)
        m = G.union([pan, kn])
        m = swept_relief(m, o, a, float(jt["lo"]), float(jt["hi"]), self.tr_obstacles())
        return G.difference(m, [G.cylinder(0.0015 + 0.00002, (x0 - 0.001, o[1], o[2]), (x1 + 0.001, o[1], o[2]), n=24)])

    def tr_door(self) -> G.Mesh:
        jt = self.jt
        m = self.tr_door_closed()
        return G.Mesh(_rot(m.V, jt["origin"], jt["axis"], float(jt["rest"])), m.F)


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
                 mass_kg=Dr.area(Dr.o_tr) * areal, notes=door_note)

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
            J.bolt_through(reg, f"{pid(680)}-B{k + 1}", 4, mv(p), mv(ax), [pid(680), pid(620)], grade="12.9",
                           step=STEP_MAIN, notes="leg-door bracket to the leg lug")
        wk = C.mem["M-WELLKEEL"]["part"] + f"-{side}"
        zb = float(C.mem["M-WELLKEEL"]["box"][0][2])
        yw1 = float(C.mem["M-WELLKEEL"]["box"][1][1])
        for k, xb in enumerate((self.Dr.ears[0][0] + 0.008, self.Dr.ears[-1][1] - 0.008)):
            J.bolt_through(reg, f"{pid(671)}-B{k + 1}", 4, mv((xb, yw1 + 0.002, zb + 0.016)), mv((0.0, -1.0, 0.0)),
                           [pid(671), wk], grade="12.9", step=STEP_DOOR, notes="nut in the harness channel")
        cove = self.Dr.cove()
        sk = reg.parts[C.pan["P-CENTRE-LOWER"]["part"]]
        sk.holes.append(cove if side == "R" else cove.mirrored_y())
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
    B.mirror()
    for side in ("R", "L"):
        B.main_fasteners(side)
    B.sequence()
