"""Landing-gear producer (YK-250 HANCER): retractable tricycle gear, wheels, brakes, retraction drives, locks and every
gear door with its hinges (``spec.landing_gear``, ``spec.layout.mechanisms``).

What it builds (part numbers from ``spec.layout.part_numbers.gear``: 620-669 legs, wheels, brakes, actuators; 670-719
doors, hinges, locks, door drives; the ids fixed by the layout are kept, the detail parts use free numbers of the
range, see ``docs/detail/gear.md``):

* main legs (starboard modelled, port = ``mirror_part``): machined 7075 yoke + outer cylinder of the telescopic
  spring / oil damper leg (``structures.sizing.gear.main_leg`` OD 42 x 3.5) on the 18 deg splay line from the trunnion
  to the axle, flanged 4130 stub axles (d 20 x 30) in the 20 H7 bushings of F-TRUNNION, sliding tube with the axle
  carrier, the cantilever 4130 axle with its nut, the TOST SB Max II 3.5 in disc-brake wheel with the TOST 200x50 6 PR
  tyre and the BZT M caliper on a keyed torque plate; damper cartridge (envelope);
* main retraction: rotary EMA on the trunnion axis (layout ACT-MLG-EMA envelope, output spline into the forward stub
  axle, integral lock bolt), its reaction flange bolted through the main-well forward wall; up-lock: catch on the
  F-UPLOCK pivot boss and a twin roller on the leg;
* main doors: inner door on three hinge stations of the well keel web along the inclined hinge line, leg door on two
  standoff brackets of the leg, trunnion door on hinge blocks either side of the leg slot (bolted to the beam web);
* nose leg: 7075 yoke + outer cylinder (OD 36 x 3), flanged stub axles d 16 in the F-NG-PIVOT bushings, steering tube
  + trailing fork (trail 30 mm) with the TOST LR Max II wheel, steering DA 26 on the aft face of the leg (belt drive),
  friction shimmy damper, retraction EMA (ACT-NLG-EMA), up-lock rollers on the axle ends and catches on the keel walls;
* nose clamshell doors on hinge blocks of the keel walls, nose-door DA 22 (EQ-NDOORACT) on the side-bay tray;
* gear detail round (GD-01..12, ``docs/detail/gear.md`` section 7.1): inner-door drive (DA 22 crank + link + horn,
  bearing housing in C-DOORSHAFT), trunnion-door drive (own micro gearmotor, crank + link + horn), nose drag link
  (EMA crank + link to the yoke horn, over-centre down-lock), nose-door drive + symmetric slide-bar coupling, main
  down-lock bolt + solenoid, anti-rotation key, nose stub spreader, brake unit hanger and brake swivel. Each planar
  linkage is solved exactly (``FourBar``) and posed by a crank joint and a child link joint whose values are added to
  gear_retraction and written as piecewise-linear expressions of gear_up.

Door hinge edges are shaped by swept reliefs against the fixed skin / structure (``relief_cutter``); the skin coves at
the hinge stations are cut by the shell module (layout ``door_outlines.*.hinge``).

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
STEP_MAIN, STEP_NOSE, STEP_DOOR = 28, 29, 30

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
HOOK_PIN_D = 0.004
DOOR_GAP = 0.0005           # half of the 1.0 mm panel gap (layout.shell.rules.joggles)
BOND_TB = 0.0003            # bond line of the trunnion-door hinge blocks on the skin inner face
IN_EAR_T = 0.003             # inner-door fixed hinge ear thickness (GD-06)
TR_FOOT = (0.0179, 0.0221)  # trunnion-door hinge block feet on the skin: forward / aft extension along x (GD-08)
TR_FOOT_Y = (0.3625, 0.3845)  # foot y range (inboard of the gear-beam web)
IN_BASE_L = 0.020           # inner-door hinge bracket base plate length on the keel web (one M4, 2.5 D edges)
DL_ARM_R = 0.0145           # aft yoke-arm boss round the down-lock bolt bore (GD-03)
DL_FIT = 0.00005            # radial clearance of the lock bolt in its bores
DL_STROKE = 0.015           # lock-bolt withdrawal (gear_up 0.02 .. 0.10, before the leg moves at 0.15)
DL_TIP = 0.0015             # bolt tip behind the forward face of the aft yoke arm (engaged: 11.5 mm in the arm)
DL_TAIL = 0.0050            # bolt tail inside the solenoid (engaged)
SOL_R, SOL_L = 0.012, 0.0272  # lock solenoid OD 24 x 27 mm (estimate, push-pull class)
SWV_R = 0.0165               # brake swivel M2.5 screws on the bearing boss of the aft lug (r 11.5 bore, r 21.8 boss)
SOL_NECK = (0.0094, 0.006)   # threaded neck M19 x 1 of the solenoid in a counterbore of the aft lug (+ jam nut)
KEY = (0.003, 0.0025, 0.096, 0.105, 0.0163)  # anti-rotation key: width, radial height, s range, inner radius (GD-12)


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
# planar linkages (gear detail round: door drives, nose drag link): every crank / link / output axis of one mechanism is
# parallel, so the mechanism is solved exactly in the plane normal to that axis and posed with chained revolute joints
# =====================================================================================================================
def plane_frame(u) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """(e1, e2, u) right-handed frame of the plane normal to ``u`` (e1 = y projected, or x for a y axis)."""
    u = _u(u)
    e1 = np.array([0.0, 1.0, 0.0]) if abs(u[1]) < 0.9 else np.array([1.0, 0.0, 0.0])
    e1 = _u(e1 - (e1 @ u) * u)
    return e1, np.cross(u, e1), u


def _r2(a: float) -> np.ndarray:
    c, s_ = math.cos(a), math.sin(a)
    return np.array([[c, -s_], [s_, c]])


def _ang(v) -> float:
    return math.atan2(float(v[1]), float(v[0]))


class FourBar:
    """Planar four-bar: a driven crank (pivot ``C``, radius ``rc``) and an output member (pivot ``O``) rotating about
    parallel axes ``u``; the link joins the crank pin K to the output pin q. ``q_tog`` is the output pin in the TOGGLE
    pose (crank and link collinear = over-centre lock: the gear-down pose of a down-lock, the closed pose of a door);
    output changes are measured from that pose. ``s_out`` is the sign of the output joint axis against ``u``.
    K_tog = C + rc (q_tog - C)/|q_tog - C| (``fold`` = False, extended toggle) or the opposite point (``fold`` = True).
    ``drive`` (+1 / -1) is the crank rotation sense about u when the output leaves the toggle pose (it selects the branch
    through the toggle)."""

    def __init__(self, u, plane_pt, C, O, rc: float, q_tog, s_out: float, fold: bool, drive: float):
        self.e1, self.e2, self.u = plane_frame(u)
        self.p0 = np.asarray(plane_pt, float)
        self.C3 = self.on_plane(C)
        self.O3 = self.on_plane(O)
        self.c, self.o = self.to2(self.C3), self.to2(self.O3)
        self.q_rest = self.to2(q_tog)
        self.rc, self.s_out, self.fold, self.drive = float(rc), float(s_out), bool(fold), float(drive)
        w = _u(self.q_rest - self.c)
        self.k_rest = self.c + (-1.0 if fold else 1.0) * self.rc * w
        self.l = float(np.linalg.norm(self.q_rest - self.k_rest))
        self.psi0 = _ang(self.k_rest - self.c)
        self.lam0 = _ang(self.q_rest - self.k_rest)

    def on_plane(self, P) -> np.ndarray:
        P = np.asarray(P, float)
        return P - ((P - self.p0) @ self.u) * self.u

    def to2(self, P) -> np.ndarray:
        d = np.asarray(P, float) - self.p0
        return np.array([d @ self.e1, d @ self.e2])

    def to3(self, q2) -> np.ndarray:
        return self.p0 + q2[0] * self.e1 + q2[1] * self.e2

    def q_at(self, d_out: float) -> np.ndarray:
        """Output pin for an output-joint change ``d_out`` (value - rest)."""
        return self.o + _r2(self.s_out * d_out) @ (self.q_rest - self.o)

    def solve(self, d_outs) -> list[dict]:
        """Crank angle, link angle, crank pin, transmission angle for each output change (ordered from the rest pose
        outward: the branch is followed continuously)."""
        out, prev = [], self.psi0
        for k, d in enumerate(d_outs):
            q = self.q_at(d)
            dd = float(np.linalg.norm(q - self.c))
            if abs(d) < 1e-12:
                psi = self.psi0
            else:
                if dd > self.rc + self.l + 1e-9 or dd < abs(self.l - self.rc) - 1e-9:
                    raise ValueError(f"four-bar: output change {d:.4f} out of reach")
                base = _ang(q - self.c)
                g = math.acos(max(-1.0, min(1.0, (self.rc ** 2 + dd ** 2 - self.l ** 2) / (2 * self.rc * dd))))
                cands = [base + g, base - g]
                if len(out) == 0 or (len(out) == 1 and abs(out[0]["psi"] - self.psi0) < 1e-12):
                    psi = min(cands, key=lambda v: -self.drive * (((v - self.psi0 + math.pi) % (2 * math.pi)) - math.pi))
                else:
                    psi = min(cands, key=lambda v: abs(((v - prev + math.pi) % (2 * math.pi)) - math.pi))
                psi = prev + (((psi - prev + math.pi) % (2 * math.pi)) - math.pi)
            K = self.c + self.rc * np.array([math.cos(psi), math.sin(psi)])
            lam = _ang(q - K)
            lam = self.lam0 + (((lam - self.lam0 + math.pi) % (2 * math.pi)) - math.pi) if not out else \
                out[-1]["lam"] + (((lam - out[-1]["lam"] + math.pi) % (2 * math.pi)) - math.pi)
            v1, v2 = q - K, q - self.o
            mu = math.degrees(math.acos(min(1.0, abs(v1 @ v2) / (np.linalg.norm(v1) * np.linalg.norm(v2) + 1e-15))))
            out.append({"psi": psi, "lam": lam, "K": K, "q": q, "mu": mu, "dpsi": psi - self.psi0,
                        "dlam": lam - self.lam0})
            prev = psi
        return out


def pl_expr(gs, vals, prop: str = "gear_up") -> str:
    """Blender simple expression (clamp only) of the piecewise-linear function through (gs, vals): equal to the
    sequence values at every sample."""
    terms = [f"{vals[0]:.6f}"] if abs(vals[0]) > 1e-9 else []
    for k in range(len(gs) - 1):
        dv = vals[k + 1] - vals[k]
        if abs(dv) < 1e-9:
            continue
        dg = gs[k + 1] - gs[k]
        terms.append(f"{dv:+.6f}*clamp(({prop}-{gs[k]:.4f})/{dg:.4f},0,1)")
    return "".join(terms).lstrip("+") or "0.0"


def eval_pl_expr(expr: str, g: float) -> float:
    """Evaluate a pl_expr / layout expression (clamp, min, max, abs) at gear_up = g (tests and checks)."""
    def clamp(v, a, b):
        return min(max(v, a), b)
    return float(eval(expr, {"__builtins__": {}}, {"clamp": clamp, "min": min, "max": max, "abs": abs, "gear_up": g}))


def bar_mesh(fb: FourBar, P0, P1, r0: float, r1: float, t0: float, t1: float, hole0: float = 0.0,
             hole1: float = 0.0, extra=(), waist: float = 0.0) -> G.Mesh:
    """Flat bar (hull of two eye circles) between the 3-D points P0, P1 in the plane of ``fb`` (2-D coordinates), from
    t0 to t1 along the mechanism axis (measured from the plane point), with pin bores; ``extra`` = shapely polygons
    added to the outline."""
    a, b = fb.to2(P0), fb.to2(P1)
    w0, w1 = (waist, waist) if waist else (r0, r1)        # half width of the bar between the eyes
    poly = unary_union([Point(*a).buffer(r0, 48), Point(*b).buffer(r1, 48),
                        Polygon([tuple(a + w0 * _r2(math.pi / 2) @ _u(b - a)), tuple(b + w1 * _r2(math.pi / 2) @ _u(b - a)),
                                 tuple(b - w1 * _r2(math.pi / 2) @ _u(b - a)), tuple(a - w0 * _r2(math.pi / 2) @ _u(b - a))])]
                       + list(extra))
    if hole0:
        poly = poly.difference(Point(*a).buffer(hole0, 32))
    if hole1:
        poly = poly.difference(Point(*b).buffer(hole1, 32))
    return _plate(poly, fb.p0, fb.e1, fb.e2, t0, t1)


def fb_at(fb: FourBar, d: float, step_deg: float = 1.0) -> dict:
    """Four-bar state for the output change ``d`` from the toggle pose (branch followed continuously from the toggle)."""
    n = max(3, int(abs(math.degrees(d)) / step_deg) + 2)
    return fb.solve(np.linspace(0.0, d, n))[-1]


def best_fourbar(u, plane_pt, C, O, rc, q_tog, s_out, fold, d_max) -> FourBar:
    """FourBar with the branch (drive sense) that moves the crank monotonically over the output travel 0..d_max with
    the larger minimum transmission angle."""
    best = None
    for dr in (1.0, -1.0):
        fb = FourBar(u, plane_pt, C, O, rc, q_tog, s_out, fold, dr)
        try:
            sol = fb.solve(np.linspace(0.0, d_max, 46))
        except ValueError:
            continue
        ps = [s_["dpsi"] for s_ in sol]
        if not all((b - a) * math.copysign(1.0, ps[-1]) > -1e-7 for a, b in zip(ps, ps[1:])):
            continue
        mu = min(s_["mu"] for s_ in sol[1:])
        if best is None or mu > best[0]:
            best = (mu, fb)
    if best is None:
        raise ValueError("four-bar: no monotonic branch over the travel")
    return best[1]


class Mech:
    """A planar drive (crank on a fixed actuator, link, horn on the output member) posed with two chained revolute
    joints: crank (origin C, axis u) and link (child of the crank, origin = crank pin K in the rest pose). ``d_of``
    maps a gear_retraction state to the output change from the toggle pose; all joint values are 0 in the rest pose
    (gear down)."""

    def __init__(self, fb: FourBar, d_of, d_rest: float, d_max: float):
        self.fb, self.d_of, self.d_rest, self.d_max = fb, d_of, float(d_rest), float(d_max)
        self.rest = fb_at(fb, self.d_rest)

    def values(self, st: dict) -> tuple[float, float]:
        s_ = fb_at(self.fb, float(self.d_of(st)))
        dc = s_["dpsi"] - self.rest["dpsi"]
        return dc, (s_["dlam"] - self.rest["dlam"]) - dc

    def K3(self) -> np.ndarray:
        return self.fb.to3(self.rest["K"])

    def q3(self) -> np.ndarray:
        return self.fb.to3(self.rest["q"])

    def mu_min(self) -> float:
        return min(s_["mu"] for s_ in self.fb.solve(np.linspace(0.0, self.d_max, 46))[1:])

    def travel(self) -> float:
        return abs(self.fb.solve(np.linspace(0.0, self.d_max, 46))[-1]["dpsi"])


def eye_poly(fb: FourBar, P, r: float) -> Polygon:
    return Point(*fb.to2(P)).buffer(r, 48)


def inner_face_2d(C: "Ctx", fb: FourBar, tc: float, ys, t: float, side: float = 1.0, gap: float = BOND_TB) -> list:
    """2-D points (plane of ``fb``) of the inner face of a lower-skin door panel (OML - t along the normal) at the
    plane offset ``tc`` for the y stations ``ys``, moved ``gap`` inward (bond line of a horn foot)."""
    out = []
    for y in ys:
        x = float(fb.p0[0])
        for _ in range(3):
            ph = phi_lower(C.fus, np.array([x]), np.array([y]))
            if side < 0:
                ph = 2 * math.pi - ph
            P = C.fus.point(np.array([x]), ph)[0]
            N = C.fus.normal(np.array([x]), ph)[0]
            Q = P - (t + gap) * N
            x = float(fb.p0[0] + (tc - (Q[1] - fb.p0[1]) * fb.u[1] - (Q[2] - fb.p0[2]) * fb.u[2]) / fb.u[0])
        out.append(tuple(fb.to2(Q)))
    return out


def horn_poly(fb: FourBar, q2, r_eye: float, foot) -> Polygon:
    """Horn outline: hull of the pin eye and the foot points (on the output member's face)."""
    return unary_union([Point(*q2).buffer(r_eye, 48), Polygon(list(foot) + [tuple(q2)]).buffer(0)]).convex_hull


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
        # down-lock boss on the aft arm (GD-03): the lock bolt of the aft bearing lug engages it with the gear down
        dl = self.downlock()
        xa, xb = self.arm_a
        parts.append(G.hull(np.vstack([
            G.cylinder(ARM_R, (xa, T[1], T[2]), (xb, T[1], T[2]), n=48).V,
            G.cylinder(DL_ARM_R, (xa, dl["p"][1], dl["p"][2]), (xb, dl["p"][1], dl["p"][2]), n=48).V])))
        m = G.union(parts)
        cuts = [G.cylinder(0.5 * C.stub_d + 0.00002, (xa - 0.002, T[1], T[2]), (xb + 0.002, T[1], T[2]), n=48)
                for xa, xb in (self.arm_f, self.arm_a)]
        cuts.append(G.cylinder(CYL_RI, C.P(CROWN_S[1] - 0.0001), C.P(CYL_S[1] + 0.01), n=48))
        cuts.append(G.cylinder(dl["r"] + DL_FIT, (xa - 0.001, dl["p"][1], dl["p"][2]), (xb + 0.001, dl["p"][1], dl["p"][2]),
                               n=48))
        cuts.append(self.key_box(0.00003, (KEY[1] + 0.0003, 0.0)))            # keyway of the anti-rotation key
        return G.difference(m, cuts)

    def downlock(self) -> dict:
        d = self.C.fit["F-TRUNNION"]["downlock"]
        return {"p": np.asarray(d["point"], float), "r": 0.5 * float(d["d"]), "lug_bottom_z": float(d["lug_bottom_z"])}

    def key_box(self, grow: float, radial=(KEY[1], 0.0), s_rng=None) -> G.Mesh:
        """Anti-rotation key (GD-12) on the +x side of the leg (leg frame): ``radial`` = (height, inner offset)."""
        C = self.C
        s0, s1 = s_rng if s_rng else (KEY[2], KEY[3])
        h, r0 = radial
        rc = KEY[4] + r0 + 0.5 * h
        return _obox(C.P(0.5 * (s0 + s1), 0.0, rc), C.Rleg(), (h + 2 * grow, KEY[0] + 2 * grow, s1 - s0 + 2 * grow))

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
            bore = G.cylinder(0.005, (min(xa, fl[0]) - 0.001, T[1], T[2]), (max(xb, fl[1]) + 0.001, T[1], T[2]), n=32)
            ms.append(G.difference(G.union([shaft, flange]), [bore]))      # gun-drilled d 10 (lightening)
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
        cut = [G.cylinder(0.01001, (A[0], y0 - 0.01, A[2]), (A[0], y1 + 0.01, A[2]), n=48),
               # key slot (GD-12): open at the top of the tube, 3.05 wide, 1.2 mm deep, down to the end plug
               self.key_box(0.000025, (0.0012, -0.0001), (TUBE_S[0] - 0.002, TUBE_S[1] - 0.0065))]
        return G.difference(m, cut)

    # ---------------------------------------------------------------- 634: axle + nut
    def axle(self) -> G.Mesh:
        A = self.C.A
        y = A[1]
        head = G.tube(0.016, 0.006, (A[0], y + CARR_Y[0] - 0.004, A[2]), (A[0], y + CARR_Y[0], A[2]), n=48)
        shaft = G.tube(AXLE_R, 0.006, (A[0], y + CARR_Y[0] - 0.0001, A[2]), (A[0], y + NUT_Y[1], A[2]), n=48)
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
        if "pin" in f:                               # gear detail round GD-10: pivot pin bore of the layout
            return np.asarray(f["pin"]["point"], float)
        return np.asarray(f["point"], float) + np.array([0.0, 0.0, 0.006])

    def hook(self) -> G.Mesh:
        C = self.C
        pv = self.hook_pivot()
        zr = C.T[2] - ROLLER_E                       # stowed roller centre (above the stowed leg)
        r_p = 2.05 * HOOK_PIN_D
        w = max(r_p, ROLLER_R + 0.0005 + 0.0028)          # side walls of the roller notch >= 2.8 mm
        prof = unary_union([Point(0.0, 0.0).buffer(r_p, 48),
                            Polygon([(-w, 0.0), (w, 0.0), (w, zr - pv[2] - 0.0065), (-w, zr - pv[2] - 0.0065)])])
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
        self.o_i = np.asarray(ji["origin"], float)
        self.a_i = _u(ji["axis"])
        # gear detail round (GD-06): three hinge stations along the inclined hinge line (layout door_outlines hinge):
        # fixed bracket = two ears 3 mm, door knuckle 5.5 mm between them (0.5 mm gaps), pin d 3
        self.stations = [tuple(float(v) for v in st) for st in D["main_inner_door_R"]["hinge"]["stations"]]
        self.ears = [((x0, x0 + IN_EAR_T), (x1 - IN_EAR_T, x1)) for x0, x1 in self.stations]
        self.knuck = [(x0 + IN_EAR_T + 0.0005, x1 - IN_EAR_T - 0.0005) for x0, x1 in self.stations]
        self.cove_r = float(D["main_inner_door_R"]["hinge"]["cove_r"])

    def door_poly(self, o):
        return o.buffer(-DOOR_GAP, join_style=2)

    def area(self, o) -> float:
        return float(self.door_poly(o).area)

    # ---------------------------------------------------------------- inner door 670
    def on_axis(self, x: float) -> np.ndarray:
        """Point of the inner-door hinge line at station x."""
        return self.o_i + (x - self.o_i[0]) / self.a_i[0] * self.a_i

    def ear_polys(self, x: float, hole: bool = True):
        """(y, z) profile of a fixed hinge ear at station x: ring r 3.5 round the pin, column up to the lower edge of
        the well keel web and a 3 mm foot on its outboard face (bolted)."""
        o = self.on_axis(x)
        wk = self.C.mem["M-WELLKEEL"]["box"]
        yw1, zb = float(wk[1][1]), float(wk[0][2])
        ring = Point(o[1], o[2]).buffer(0.0035, 32)
        col = Polygon([(o[1] - 0.0025, o[2]), (o[1] + 0.0025, o[2]), (o[1] + 0.0025, zb - 0.0035),
                       (yw1 + 0.003, zb - 0.0005), (yw1 + 0.003, zb + 0.026), (yw1, zb + 0.026), (yw1, zb - 0.0005),
                       (o[1] - 0.0025, zb - 0.004)])
        u = unary_union([ring, col])
        return u.difference(Point(o[1], o[2]).buffer(0.0015 + 0.00002, 24)) if hole else u

    def fixed_hinge(self) -> G.Mesh:
        """671: inner-door fixed hinge brackets (7075, three stations): two ears round the hinge line (pin d 3) and a
        base plate 20 mm long on the outboard face of the well keel web (one M4 each)."""
        wk = self.C.mem["M-WELLKEEL"]["box"]
        yw1, zb = float(wk[1][1]), float(wk[0][2])
        ms = []
        for (x0, x1), ears in zip(self.stations, self.ears):
            for xa, xb in ears:
                ms.append(_plate(self.ear_polys(0.5 * (xa + xb), hole=False), (xa, 0.0, 0.0), (0, 1, 0), (0, 0, 1), 0.0,
                                 xb - xa))
            xc = 0.5 * (x0 + x1)
            ms.append(_box((xc - IN_BASE_L / 2, yw1, zb + 0.004), (xc + IN_BASE_L / 2, yw1 + 0.003, zb + 0.026)))
        bores = [G.cylinder(0.0015 + 0.00002, self.on_axis(x0 - 0.001), self.on_axis(x1 + 0.001), n=24)
                 for x0, x1 in self.stations]
        return G.difference(G.union(ms), bores)

    def cove(self) -> list:
        return [G.cylinder(self.cove_r, self.on_axis(x0 - 0.0015), self.on_axis(x1 + 0.0015), n=32)
                for x0, x1 in self.stations]

    def inner_door(self) -> G.Mesh:
        C = self.C
        o, a = self.o_i, self.a_i
        pan = oml_panel(C, self.door_poly(self.o_in), self.t)
        kn = [G.cylinder(0.0035, self.on_axis(xa), self.on_axis(xb), n=32) for xa, xb in self.knuck]
        m = G.union([pan] + kn)
        x0, y0, x1, y1 = self.o_in.bounds
        ys = y0 - DOOR_GAP

        def strip(x, with_cove):
            sec = skin_section(C, x, -ys, ys, self.t_skin)
            if with_cove:
                q = self.on_axis(x)
                sec = sec.difference(Point(q[1], q[2]).buffer(self.cove_r, 32))
            return sec
        v0, v1 = float(self.ji["lo"]), float(self.ji["hi"])
        edges = [x0 - 0.002] + [v for xa, xb in self.stations for v in (xa - 0.0015, xb + 0.0015)] + [x1 + 0.002]
        cuts = []
        for k in range(len(edges) - 1):
            xa_, xb_ = edges[k], edges[k + 1]
            wc = (k % 2 == 1)                                  # inside a station: the strip has its cove
            nn = max(3, int((xb_ - xa_) / 0.012) + 2)
            cuts.append(relief_cutter(np.linspace(xa_ - 0.0005, xb_ + 0.0005, nn), lambda x, wc=wc: strip(x, wc), o, a,
                                      v0, v1))
        for ears in self.ears:
            for xa, xb in ears:
                ear = self.ear_polys(0.5 * (xa + xb)).buffer(0.0007)
                cuts.append(relief_cutter(np.array([xa - 0.0007, xb + 0.0007]), lambda x, ear=ear: ear, o, a, v0, v1))
        m = G.difference(m, cuts)
        bores = [G.cylinder(0.0015 + 0.00002, self.on_axis(xa - 0.001), self.on_axis(xb + 0.001), n=24)
                 for xa, xb in self.knuck]
        return _clean(G.difference(m, bores))

    def inner_pin(self) -> G.Mesh:
        return G.merge([G.cylinder(0.0015 - 0.00001, self.on_axis(x0 + 0.0003), self.on_axis(x1 - 0.0003), n=24)
                        for x0, x1 in self.stations])

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

    def tr_blocks_rng(self):
        """x ranges of the two hinge end blocks (layout hinge station = the beam notch): the aft block starts behind
        the aft yoke arm, whose down-lock boss sweeps a 42.5 mm radius round the trunnion axis (GD-03)."""
        xs0, xs1 = self.o_tr.bounds[0], self.o_tr.bounds[2]
        st = self.C.door["main_trunnion_door_R"]["hinge"]["stations"][0]
        return None, ((float(st[0]), xs0 - DOOR_GAP - 0.0005), (max(xs1 + DOOR_GAP + 0.0005, self.M.arm_a[1] + 0.0006),
                                                               float(st[1])))

    def tr_feet(self):
        """x ranges of the two feet of the hinge blocks on the inner face of the lower skin (GD-08)."""
        _m, rng = self.tr_blocks_rng()
        return ((rng[0][0] - TR_FOOT[0], rng[0][1]), (rng[1][0], rng[1][1] + TR_FOOT[1]))

    def tr_blocks(self):
        """681: trunnion-door hinge end blocks (7075, GD-08): either side of the leg slot inside the notch of the gear
        beam, knuckle d 7 on the pin d 3; each with a 3 mm foot along the inner face of the lower skin (0.3 mm bond
        line) riveted through the skin (2 x d 2.4 blind rivets per block)."""
        C = self.C
        o = np.asarray(self.jt["origin"], float)
        nb = C.mem["M-GEARBEAM"]["notch"]
        z_top = float(nb["z_top"]) - 0.0015
        _m, rng = self.tr_blocks_rng()
        y_in = 0.3725
        ms = []
        for (xa, xb), (fa, fb_) in zip(rng, self.tr_feet()):
            blk = _box((xa, y_in, -0.17), (xb, o[1] + 0.0035, z_top))
            kn = G.cylinder(0.0035, (xa, o[1], o[2]), (xb, o[1], o[2]), n=32)
            m = G.union([blk, kn])
            big = rect(min(xa, fa) - 0.01, max(xb, fb_) + 0.01, 0.30, 0.42)
            m = G.difference(m, [oml_panel(C, big, 0.03 + self.t_skin + BOND_TB, off=-0.03, nx=14, ny=60)])
            foot = oml_panel(C, rect(fa, fb_, TR_FOOT_Y[0], TR_FOOT_Y[1]), 0.003, off=self.t_skin + BOND_TB, nx=12, ny=8)
            ms.append(G.union([m, kn, foot]))
        bore = G.cylinder(0.0015 + 0.00002, (rng[0][0] - 0.001, o[1], o[2]), (rng[1][1] + 0.001, o[1], o[2]), n=24)
        return _clean(G.difference(G.union(ms), [bore])), rng

    def tr_rivets(self):
        """(point on the OML, inward axis) of the foot rivets."""
        C = self.C
        (f0, f1), (a0, a1) = self.tr_feet()
        y = 0.5 * (TR_FOOT_Y[0] + TR_FOOT_Y[1])
        out = []
        for x in (f0 + 0.005, f0 + 0.015, a1 - 0.013, a1 - 0.005):
            ph = phi_lower(C.fus, np.array([x]), np.array([y]))
            P = C.fus.point(np.array([x]), ph)[0]
            N = C.fus.normal(np.array([x]), ph)[0]
            out.append((P, -N))
        return out

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
        self.extra = []                  # integral features added by the gear detail round (drag-link horn)

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
        m = G.union(parts + list(self.extra))
        cuts = [G.cylinder(0.5 * self.stub_d + 0.00002, (Pv[0], -0.05, Pv[2]), (Pv[0], 0.05, Pv[2]), n=48),
                G.cylinder(self.ri, self.X(0, 0, N_CROWN[1] - 0.0001), self.X(0, 0, N_CYL[1] - 0.01), n=48)]
        return _clean(G.difference(m, cuts))

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
        """632: retraction EMA envelope (layout ACT-NLG-EMA) with the spline bore of the crank on its port face and a
        flange on the starboard keel-wall land (2 x M5)."""
        a = self.C.act["ACT-NLG-EMA"]
        lo, hi = np.asarray(a["box"][0], float), np.asarray(a["box"][1], float)
        sp = np.asarray(a["shaft"]["point"], float)
        body = _box(lo + np.array([0, 0.0, 0.0005]), hi - np.array([0, 0.0005, 0.0005]))
        wall_y = float(self.C.mem["M-KEELWALL"]["box"][0][1])
        tabs = [_box((x0, wall_y - EMA_TAB[1], hi[2] - 0.022), (x1, wall_y, hi[2] - 0.0005))
                for x0, x1 in ((lo[0] - EMA_TAB[0], lo[0] + 0.001), (hi[0] - 0.001, hi[0] + EMA_TAB[0]))]
        bore = G.cylinder(0.00605, (sp[0], lo[1] - 0.001, sp[2]), (sp[0], lo[1] + 0.0125, sp[2]), n=32)
        return _clean(G.difference(G.union([body] + tabs), [bore]))

    def ema_bolts(self):
        a = self.C.act["ACT-NLG-EMA"]
        lo, hi = np.asarray(a["box"][0], float), np.asarray(a["box"][1], float)
        wall_y = float(self.C.mem["M-KEELWALL"]["box"][0][1])
        return [np.array([x, wall_y - EMA_TAB[1], hi[2] - 0.0115]) for x in (lo[0] - 0.5 * EMA_TAB[0], hi[0] + 0.5 * EMA_TAB[0])]

    def spreader(self) -> G.Mesh:
        """656 (GD-11): stub-axle spreader sleeve between the two stub flanges (jack-screw pair, 7075 / 4130): it
        holds both flanges against the yoke arms so neither stub can walk inward out of its bushing."""
        Pv = self.Pv
        y = N_STUB_FL[0] - 0.0001
        tube = G.tube(0.0065, 0.0030, (Pv[0], -y, Pv[2]), (Pv[0], y, Pv[2]), n=40)
        hexn = G.tube(0.0080, 0.0030, (Pv[0], -0.004, Pv[2]), (Pv[0], 0.004, Pv[2]), n=6)
        return G.union([tube, hexn])

    def catch(self) -> G.Mesh:
        """650: nose up-lock catch (4130 plate, starboard; port = mirror) on the keel-wall inner face: the axle-end
        roller enters its notch from below at the end of the retraction (latch pawl + solenoid: gear-unit item); two
        tapped bosses above the notch take 2 x M4 from the side bay (GD-08)."""
        a = self.stowed_axle()
        wall = float(self.C.mem["M-KEELWALL"]["box"][0][1])
        prof = Polygon([(-0.020, -0.016), (0.020, -0.016), (0.020, CATCH_TOP), (-0.020, CATCH_TOP)])
        notch = unary_union([Point(0.0, 0.0).buffer(0.0065, 48),
                             Polygon([(-0.0065, 0.0), (0.0065, 0.0), (0.0065, -0.03), (-0.0065, -0.03)])])
        pl = _plate(prof.difference(notch), (a[0], wall - 0.0001, a[2]), (1, 0, 0), (0, 0, 1), 0.0,
                    wall - 0.0001 - N_UPL_Y[0] + 0.0004)
        boss = _box((a[0] - 0.020, CATCH_BOSS_Y, a[2] + CATCH_BOSS_Z), (a[0] + 0.020, wall - 0.0001, a[2] + CATCH_TOP))
        return G.union([pl, boss])

    def catch_bolts(self):
        a = self.stowed_axle()
        wall_o = float(self.C.mem["M-KEELWALL"]["box"][1][1])
        return [(np.array([a[0] + dx, wall_o, a[2] + CATCH_BOLT_Z]), np.array([0.0, -1.0, 0.0])) for dx in (-0.010, 0.010)]


EMA_TAB = (0.016, 0.004)            # nose EMA mounting tabs fore / aft of the body on the keel-wall face (x length, y)
SIDEBAY_TRAY = {"R": "YK250-CH-122", "L": "YK250-CH-118"}   # side-bay trays (keel-wall flange under the catch bolts)
CATCH_TOP, CATCH_BOSS_Y, CATCH_BOSS_Z, CATCH_BOLT_Z = 0.0355, 0.0346, 0.0152, 0.0232   # catch bosses above the notch
#                                                     (clear of the axle nut r 14.5), M4 tapped 8 mm


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
        self.blocks = ((0.700, 0.710), (0.736, 0.746), (x1 - 0.031, x1 - 0.026))   # the last ends ahead of the coupling horns
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
        """683: hinge blocks (7075) on the keel-wall inner face, each with a flange above the hinge line (out of the
        door's swept space) bolted through the wall (one M4, nut in the side bay; GD-08)."""
        ms = []
        for xa, xb in self.blocks:
            pa, pb = self.on_axis(xa), self.on_axis(xb)
            kn = G.cylinder(0.0035, pa, pb, n=32)
            zt = max(pa[2], pb[2]) + 0.014
            (f0, f1), _xb, zc = self.block_flange(xa, xb)
            ms.append(_box((f0, self.O[1] - 0.0015, zc + 0.010), (f1, self.wall_y, zc + 0.032)))
            blk = _box((xa, self.O[1] - 0.0015, min(pa[2], pb[2]) - 0.001), (xb, self.wall_y, zt))
            big = rect(xa - 0.01, xb + 0.01, 0.0, 0.09)
            blk = G.difference(blk, [oml_panel(self.C, big, 0.03 + self.t_skin + 0.003, off=-0.03, nx=8, ny=40)])
            ms.append(G.union([blk, kn]))
        bores = [G.cylinder(0.0015 + 0.00002, self.on_axis(xa - 0.001), self.on_axis(xb + 0.001), n=24)
                 for xa, xb in self.stations()]
        return G.difference(G.union(ms), bores)

    def block_flange(self, xa, xb):
        """Flange x range above the hinge line, bolt x and hinge z: centred on the block, the aft block's flange
        ahead of it (the nose-door coupling horns and drive link work aft of x 1.068)."""
        xc = 0.5 * (xa + xb)
        f = (xc - 0.010, xc + 0.010) if xa < 0.9 else (xa - 0.012, xa + 0.004)
        return f, 0.5 * (f[0] + f[1]), float(self.on_axis(xc)[2])

    def block_bolts(self):
        out = []
        for xa, xb in self.blocks:
            _f, xbolt, zc = self.block_flange(xa, xb)
            out.append((np.array([xbolt, self.O[1] - 0.0015, zc + 0.021]), np.array([0.0, 1.0, 0.0])))
        return out

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
# gear detail round: door drives, nose drag link, main down-lock, brake installation (docs/detail/gear.md section 7)
# =====================================================================================================================
ID_RC, ID_FOLD = 0.034, True                          # inner-door crank radius, folded toggle with the door closed
ID_Q = (0.041, -0.180)                                # inner-door horn pin, door closed (y, z in the drive plane)
ID_T = {"horn": (-0.01075, -0.00775), "link": (-0.0075, -0.0045), "crank": (-0.00425, -0.00125)}  # along the shaft
#                                                     axis from the layout shaft point (planes 6 mm ahead of it)
#                                                     (0.25 mm plane gaps: shim washers on the pins)
ID_PIN, ID_EYE = 0.003, 0.0062                         # pins d 3, eyes r 6 (2 D edge distance)
TD_X, TD_OM, TD_RC = 2.9845, (0.3657, -0.1166), 0.016   # trunnion-door drive plane (link), gearmotor axis, crank
TD_Q = (0.3761, -0.1360)                              # trunnion-door horn pin, door closed (y, z)
TD_T = {"crank": (-0.00425, -0.00125), "link": (-0.0010, 0.0020), "horn": (0.00225, 0.00525)}
TD_MOT = (0.008, 2.9620, 2.9800)                      # gearmotor envelope r 8 x 18 mm (estimate), x range
TD_PIN, TD_EYE = 0.003, 0.0062
NG_RH, NG_AL, NG_RC = 0.033, math.radians(116.0), 0.0341   # nose drag-link horn pin (radius, angle about the pivot in
#                                                            (x, z)), crank radius; folded toggle with the gear down
NG_Y = {"horn": (-0.01025, -0.00525), "link": (-0.0050, 0.0000), "crank": (0.00025, 0.00475)}
NG_PIN, NG_EYE = 0.006, 0.0122                         # pins d 6 (heat-treated, gear-unit requirement G-NLOCK)
ND_XP, ND_RC, ND_QREL = 1.07275, 0.0169, (0.0062, 0.0171)  # nose-door drive: link plane (port hinge point), crank
#                                     radius, drive pin on the port coupling horn rel. the port hinge (inboard, up; closed)
NC_XP, NC_B, NC_BETA, NC_ZC0 = 1.073, 0.028, math.radians(160.0), -0.130   # coupling plane x, horn radius / angle
#                                                                            (starboard), slide-bar pin z (closed)
NC_X = {"horn_L": (1.0680, 1.0710), "horn_R": (1.0680, 1.0710), "crank": (1.0680, 1.0710),
        "link_R": (1.07125, 1.07425), "dlink": (1.07125, 1.07425), "link_L": (1.0745, 1.0775),
        "bar": (1.07775, 1.08175), "guide": (1.0820, 1.0860)}   # all door-level parts aft of the tyre swing (x 1.0713)
NC_PIN, NC_EYE, NC_GP = 0.003, 0.0062, (0.010, 0.020)  # coupling pins d 3, eyes r 6, bar guide pins d 3 above the bar pin
ND_PIN, ND_EYE = 0.003, 0.0062


def _seq_bounds(vals, pad=1e-6):
    return min(0.0, min(vals)) - pad, max(0.0, max(vals)) + pad


class InnerDoorDrive:
    """GD-05: DA 22 on the aft face of FS-GEAR, shaft parallel to the inner-door hinge line through a bearing housing
    in C-DOORSHAFT; crank 686 + link 687 to the horn 688 bonded at the aft end of the door; folded toggle with the
    door closed (over-centre lock)."""

    def __init__(self, C: Ctx, Dr: "Doors"):
        self.C, self.Dr = C, Dr
        eq = C.eq["EQ-DOORACT"]
        self.Cp = np.asarray(eq["shaft"]["point"], float)
        u = _u(eq["shaft"]["axis"])
        self.u = u
        d_max = float(Dr.ji["hi"])
        fb = best_fourbar(u, self.Cp, self.Cp, Dr.on_axis(self.Cp[0]), ID_RC, (self.Cp[0], ID_Q[0], ID_Q[1]),
                          math.copysign(1.0, float(Dr.a_i @ u)), ID_FOLD, d_max)
        self.m = Mech(fb, lambda st: st["main_inner_door_R"], 0.0, d_max)
        lo, hi = np.asarray(eq["box"][0], float), np.asarray(eq["box"][1], float)
        self.t_case = (lo[0] - self.Cp[0] + 0.02075 * abs(u[2])) / u[0]   # DA 22 case forward face along u
        self.e1 = np.array([0.0, 1.0, 0.0])
        self.e2 = np.cross(u, self.e1)
        self.R = np.column_stack([u, self.e1, self.e2])
        st = C.st["FS-GEAR"]
        self.xf0, self.xf1 = float(st["x_faces"][0]), float(st["x_faces"][1])

    def at(self, t, b=0.0, c=0.0):
        return self.Cp + t * self.u + b * self.e1 + c * self.e2

    def crank(self) -> G.Mesh:
        m, fb = self.m, self.m.fb
        t0, t1 = ID_T["crank"]
        plate = bar_mesh(fb, self.Cp, m.K3(), 0.009, ID_EYE, t0, t1, hole1=0.5 * ID_PIN + 0.000025)
        shaft = G.cylinder(0.004, self.at(t1 - 0.0005), self.at(self.t_case + 0.010), n=32)
        return _clean(G.union([plate, shaft]))

    def link(self) -> G.Mesh:
        t0, t1 = ID_T["link"]
        h = 0.5 * ID_PIN + 0.000025
        return bar_mesh(self.m.fb, self.m.K3(), self.m.q3(), ID_EYE, ID_EYE, t0, t1, hole0=h, hole1=h)

    def horn(self) -> G.Mesh:
        fb = self.m.fb
        t0, t1 = ID_T["horn"]
        q2 = fb.to2(self.m.q3())
        foot = inner_face_2d(self.C, fb, 0.5 * (t0 + t1), np.linspace(ID_Q[0] - 0.012, ID_Q[0] + 0.012, 7), self.Dr.t)
        poly = horn_poly(fb, q2, ID_EYE, foot).difference(Point(*q2).buffer(0.5 * ID_PIN + 0.000025, 32))
        return _clean(_plate(poly, fb.p0, fb.e1, fb.e2, t0, t1))

    def case(self) -> G.Mesh:
        """676: DA 22 case (41.5 x 65.9 x 22, datasheet) on its tilted axes, two top ears, spline bore."""
        tc = self.t_case
        body = G.box((0.022, 0.0659, 0.0415), center=self.at(tc + 0.011, 0.5 * 0.0659 - 0.014, 0.0), R=self.R)
        ears = [G.box((0.003, 0.0135, 0.0145), center=self.at(tc + 0.0015, b, 0.02075 + 0.00625), R=self.R)
                for b in DA_EAR_B]
        bore = G.cylinder(0.00405, self.at(tc - 0.001), self.at(tc + 0.0105), n=32)
        return _clean(G.difference(G.union([body] + ears), [bore]))

    def plate(self) -> G.Mesh:
        """685: wedge plate between the aft face of FS-GEAR and the tilted case face (3.6 mm at the shaft)."""
        tc = self.t_case
        pts = [self.at(tc, b, c) for b in (-0.016, 0.0539) for c in (-0.02075, 0.02075 + 0.0135)]
        P = np.array(pts)
        Pf = P.copy()
        Pf[:, 0] = self.xf1
        m = G.hull(np.vstack([P, Pf]))
        return G.difference(m, [G.cylinder(0.0045, self.at(tc - 0.02), self.at(tc + 0.001), n=32)])

    def ear_bolts(self):
        return [self.at(self.t_case + 0.003, b, 0.02075 + 0.006) for b in DA_EAR_B]

    def housing(self) -> G.Mesh:
        """689: flanged bearing housing (7075) on the forward face of FS-GEAR, spigot in C-DOORSHAFT, two sealed
        deep-groove bearings 8 x 12 (catalogue class) for the d 8 shaft."""
        y, z = HS_YZ
        fl = _box((self.xf0 - 0.003, y - 0.025, z - 0.012), (self.xf0, y + 0.025, z + 0.012))
        sp = G.cylinder(0.0085, (self.xf0 - 0.0001, y, z), (self.xf1 - 0.0003, y, z), n=40)
        t0 = (self.xf0 - 0.004 - self.Cp[0]) / self.u[0]
        t1 = (self.xf1 + 0.001 - self.Cp[0]) / self.u[0]
        return G.difference(G.union([fl, sp]), [G.cylinder(0.00405, self.at(t0), self.at(t1), n=32)])

    def housing_bolts(self):
        y, z = HS_YZ
        return [np.array([self.xf0 - 0.003, y + dy, z]) for dy in (-0.0175, 0.0175)]


DA_EAR_B = (-0.006, 0.044)          # DA 22 top ears along the case (e1), M3 into the frame inserts
HS_YZ = (0.044, -0.135)             # bearing housing centre on the forward face of FS-GEAR (C-DOORSHAFT centre)


class TrunnionDoorDrive:
    """GD-07: micro gearmotor 691 (estimate) in the yoke pocket above the beam notch, clamped to the inboard face of the
    F-TRUNNION flange (bracket 695); crank 692 + link 693 to the horn 694 bonded on the trunnion door; folded toggle
    with the door closed. The door closes at gear_up 0.85..1.0 (sequence = joint expression)."""

    def __init__(self, C: Ctx, Dr: "Doors"):
        self.C, self.Dr = C, Dr
        jt = Dr.jt
        self.jt = jt
        u = np.array([1.0, 0.0, 0.0])
        self.Cp = np.array([TD_X, TD_OM[0], TD_OM[1]])
        d_max = float(jt["hi"])
        fb = best_fourbar(u, self.Cp, self.Cp, jt["origin"], TD_RC, (TD_X, TD_Q[0], TD_Q[1]),
                          math.copysign(1.0, float(_u(jt["axis"]) @ u)), True, d_max)
        self.m = Mech(fb, lambda st: st["main_trunnion_door_R"], float(jt["rest"]), d_max)

    def x(self, t):
        return TD_X + t

    def motor(self) -> G.Mesh:
        r, x0, x1 = TD_MOT
        y, z = TD_OM
        return G.difference(G.cylinder(r, (x0, y, z), (x1, y, z), n=40),
                            [G.cylinder(0.00155, (x1 - 0.0035, y, z), (x1 + 0.001, y, z), n=24)])

    def crank(self) -> G.Mesh:
        m, fb = self.m, self.m.fb
        t0, t1 = TD_T["crank"]
        plate = bar_mesh(fb, self.Cp, m.K3(), 0.0045, TD_EYE, t0, t1, hole1=0.5 * TD_PIN + 0.000025)
        y, z = TD_OM
        stub = G.cylinder(0.0015, (TD_MOT[2] - 0.003, y, z), (self.x(t0) + 0.0005, y, z), n=24)
        return _clean(G.union([plate, stub]))

    def link(self) -> G.Mesh:
        t0, t1 = TD_T["link"]
        h = 0.5 * TD_PIN + 0.000025
        return bar_mesh(self.m.fb, self.m.K3(), self.m.q3(), TD_EYE, TD_EYE, t0, t1, hole0=h, hole1=h, waist=0.0035)

    def horn(self) -> G.Mesh:
        """Built with the door closed (toggle pose) and turned to the rest (open) pose with the door."""
        fb, jt = self.m.fb, self.jt
        t0, t1 = TD_T["horn"]
        q2 = fb.q_rest
        ymax = float(self.Dr.door_poly(self.Dr.o_tr).bounds[3]) - 0.0015
        foot = inner_face_2d(self.C, fb, 0.5 * (t0 + t1), np.linspace(TD_Q[0] - 0.008, ymax, 6), self.Dr.t)
        poly = horn_poly(fb, q2, TD_EYE, foot).difference(Point(*q2).buffer(0.5 * TD_PIN + 0.000025, 32))
        m = _plate(poly, fb.p0, fb.e1, fb.e2, t0, t1)
        return G.Mesh(_rot(m.V, jt["origin"], jt["axis"], float(jt["rest"])), m.F)

    def bracket(self) -> G.Mesh:
        """695: half-clamp over the gearmotor (bonded) with a web to a tab on the inboard face of the gear-beam web
        below the F-TRUNNION flange (one M3 into a potted insert of the beam)."""
        y, z = TD_OM
        r = TD_MOT[0]
        ang = np.linspace(0.0, math.pi, 25)
        ring = Polygon([(y + (r + 0.0020) * math.cos(a), z + (r + 0.0020) * math.sin(a)) for a in ang] +
                       [(y + (r + BOND_TB) * math.cos(a), z + (r + BOND_TB) * math.sin(a)) for a in ang[::-1]])
        yb = float(self.C.mem["M-GEARBEAM"]["box"][0][1])
        web = rect(y + r + BOND_TB, yb - 0.003, z - 0.0015, z + 0.0015)
        tab = rect(yb - 0.003, yb, TD_TAB_Z[0], TD_TAB_Z[1])
        poly = unary_union([ring.buffer(0), web, tab])
        return _clean(_plate(poly, (TD_MOT[1], 0.0, 0.0), (0, 1, 0), (0, 0, 1), 0.0, TD_MOT[2] - TD_MOT[1]))

    def bracket_bolts(self):
        yb = float(self.C.mem["M-GEARBEAM"]["box"][0][1])
        return [np.array([0.5 * (TD_MOT[1] + TD_MOT[2]), yb - 0.003, TD_BOLT_Z])]


TD_TAB_Z, TD_BOLT_Z = (-0.1335, -0.1105), -0.1220   # bracket tab on the beam web (between the notch top and the
#                                                     F-TRUNNION flange) and its M3 insert


class NoseDrag:
    """GD-04: nose retraction EMA crank 655 + drag link 653 in the centre plane to the horn of the yoke (integral,
    r 31 about the pivot: the stub-axle insertion path r < 19.5 stays free); folded toggle with the gear down =
    over-centre down-lock (stop on the EMA housing)."""

    def __init__(self, C: Ctx, N: "NoseLeg"):
        self.C, self.N = C, N
        a = C.act["ACT-NLG-EMA"]
        sp = np.asarray(a["shaft"]["point"], float)
        self.Cp = np.array([sp[0], 0.0, sp[2]])
        self.h0 = np.array([N.Pv[0] + NG_RH * math.cos(NG_AL), 0.0, N.Pv[2] + NG_RH * math.sin(NG_AL)])
        u = np.array([0.0, 1.0, 0.0])
        jn = C.jl["nose_gear"]
        d_max = float(jn["hi"])
        fb = best_fourbar(u, self.Cp, self.Cp, N.Pv, NG_RC, self.h0, math.copysign(1.0, float(_u(jn["axis"]) @ u)),
                          True, d_max)
        self.m = Mech(fb, lambda st: st["nose_gear"], 0.0, d_max)
        self.y_ema = float(sp[1])

    def crank(self) -> G.Mesh:
        fb = self.m.fb
        t0, t1 = NG_Y["crank"]
        plate = bar_mesh(fb, self.Cp, self.m.K3(), 0.013, NG_EYE, t0, t1, hole1=0.5 * NG_PIN + 0.000025)
        spl = G.cylinder(0.006, (self.Cp[0], t1 - 0.0005, self.Cp[2]), (self.Cp[0], self.y_ema + 0.012, self.Cp[2]), n=32)
        return _clean(G.union([plate, spl]))

    def link(self) -> G.Mesh:
        t0, t1 = NG_Y["link"]
        h = 0.5 * NG_PIN + 0.000025
        return bar_mesh(self.m.fb, self.m.K3(), self.m.q3(), NG_EYE, NG_EYE, t0, t1, hole0=h, hole1=h)

    def horn_poly_xz(self) -> Polygon:
        """Yoke horn outline in (x, z) about the pivot: pin eye r 10.5 at r 31 / 116 deg and a web down the forward
        side of the stub-insertion path (r >= 20.5) into the crown."""
        P = self.N.Pv
        h = self.h0[[0, 2]] - P[[0, 2]]
        ang = np.linspace(NG_AL, math.radians(235.0), 30)
        sector = Polygon([(0.0365 * math.cos(a), 0.0365 * math.sin(a)) for a in ang] +
                         [(0.0205 * math.cos(a), 0.0205 * math.sin(a)) for a in ang[::-1]])
        eye = Point(*h).buffer(NG_EYE, 48)
        crown = rect(-self.N.ro, 0.0, N_CROWN[1], N_CROWN[0] + 0.004)
        poly = unary_union([sector, eye, crown]).difference(Point(0.0, 0.0).buffer(0.0205, 64))
        return poly.difference(Point(*h).buffer(0.5 * NG_PIN + 0.000025, 32))

    def horn(self) -> G.Mesh:
        P = self.N.Pv
        y0, y1 = NG_Y["horn"]
        return _plate(self.horn_poly_xz().simplify(0.00003).buffer(0), (P[0], y0, P[2]), (1, 0, 0), (0, 0, 1),
                      -(y1 - y0) - 0.0002, 0.0)


class NoseDoorDrive:
    """GD-08: DA 22 678 in the port side bay, output parallel to the door hinge axes, case ahead of its crank 679; drive
    link 696 through the port keel-wall window W-NDOOR-LINK to the drive arm of the port coupling horn 698-L (folded
    toggle with the doors closed = over-centre lock). Coupling: horns 698-R/L, links 699-R/L to the slide bar 700
    guided by two pins in the slot of the guide plate 701 (bolted to both keel walls): the starboard door follows the
    port door as its exact mirror. Every door-level part lies aft of the nose-tyre swing (x >= 1.068) and ahead of the
    slot-end sill (x < 1.0806)."""

    def __init__(self, C: Ctx, ND: "NoseDoors"):
        self.C, self.ND = C, ND
        eq = C.eq["EQ-NDOORACT"]
        self.eq = eq
        self.Cp = np.asarray(eq["shaft"]["point"], float)
        u = _u(eq["shaft"]["axis"])
        self.u = u
        jl, jr = C.jl["nose_door_L"], C.jl["nose_door_R"]
        self.jl, self.jr = jl, jr
        oL, aL = np.asarray(jl["origin"], float), _u(jl["axis"])
        d_max = float(jl["hi"])
        pL = oL + (ND_XP - oL[0]) / aL[0] * aL
        self.pL = pL
        e1, e2, _u3 = plane_frame(u)
        q = pL + ND_QREL[0] * e1 + ND_QREL[1] * e2
        fb = best_fourbar(u, pL, self.Cp, pL, ND_RC, q, math.copysign(1.0, float(aL @ u)), True, d_max)
        self.m = Mech(fb, lambda st: st["nose_door_L"], float(jl["rest"]), d_max)
        e1_ = np.array([0.0, 1.0, 0.0])
        e2_ = np.cross(u, e1_)
        ph = math.radians(float(eq["shaft"].get("roll_deg", 0.0)))
        self.e1 = math.cos(ph) * e1_ + math.sin(ph) * e2_
        self.e2 = -math.sin(ph) * e1_ + math.cos(ph) * e2_
        self.R = np.column_stack([u, self.e1, self.e2])
        self.t_case = (-0.0230, -0.0010)               # layout EQ-NDOORACT: case ahead of the crank plane
        # coupling (planar in the plane normal to u; both door axes are parallel to u)
        oR = np.asarray(jr["origin"], float)
        zax = oR[2] + (NC_XP - oR[0]) / u[0] * u[2]
        self.cp = FourBar(u, (NC_XP, 0.0, zax), (NC_XP, 0.0, zax), (NC_XP, 0.0, zax), 0.01, (NC_XP, 0.0, zax + 0.02),
                          1.0, True, 1.0)                  # plane frame helper only (to2 / to3)
        self.HR = self.cp.to2(oR + (NC_XP - oR[0]) / u[0] * u)
        self.HL = np.array([-self.HR[0], self.HR[1]])
        self.qR0 = self.HR + NC_B * np.array([math.cos(NC_BETA), math.sin(NC_BETA)])
        self.B0 = np.array([0.0, NC_ZC0 - zax])
        self.lc = float(np.linalg.norm(self.B0 - self.qR0))
        self.sR = math.copysign(1.0, float(_u(jr["axis"]) @ u))
        self.v_rest = float(jr["rest"])

    def at(self, t, b=0.0, c=0.0):
        return self.Cp + t * self.u + b * self.e1 + c * self.e2

    # ---------------- coupling kinematics (v = door opening, both doors)
    def cpl(self, v: float) -> dict:
        qR = self.HR + _r2(self.sR * v) @ (self.qR0 - self.HR)
        qL = np.array([-qR[0], qR[1]])
        B = np.array([0.0, qR[1] + math.sqrt(self.lc ** 2 - qR[0] ** 2)])
        return {"qR": qR, "qL": qL, "B": B, "lamR": _ang(qR - B), "lamL": _ang(qL - B),
                "mu": math.degrees(math.acos(min(1.0, abs((qR - B) @ (qR - self.HR)) /
                                                 (np.linalg.norm(qR - B) * np.linalg.norm(qR - self.HR)))))}

    def cpl_values(self, st: dict) -> tuple[float, float, float]:
        a, r = self.cpl(st["nose_door_R"]), self.cpl(self.v_rest)
        return a["B"][1] - r["B"][1], a["lamR"] - r["lamR"], a["lamL"] - r["lamL"]

    def tx(self, key, ref=None):
        """Plane offsets along u of the plane ``key`` (x at the hinge level) from the coupling plane point."""
        x0 = NC_XP if ref is None else ref
        return tuple((x - x0) / self.u[0] for x in NC_X[key])

    def td(self, key):
        """Plane offsets of ``key`` from the drive plane point (port hinge at ND_XP)."""
        return self.tx(key, ND_XP)

    # ---------------- drive parts
    def case(self) -> G.Mesh:
        """678: DA 22 case on its tilted axes ahead of the crank, outboard ear on the side-bay tray (2 x M3 +
        nutplates), spline bore on the aft face."""
        t0, t1 = self.t_case
        body = G.box((t1 - t0, 0.0415, 0.0659), center=self.at(0.5 * (t0 + t1), 0.0, 0.5 * 0.0659 - 0.014), R=self.R)
        y_o = self.ear_face_y()
        lo = np.asarray(self.eq["box"][0], float)
        ear = _box((NDA_EAR_X[0], lo[1] - 0.003 - NDA_EAR_OUT, NDA_TRAY_Z), (NDA_EAR_X[1], y_o + 0.002, NDA_TRAY_Z + 0.003))
        bore = G.cylinder(0.00405, self.at(t1 - 0.0105), self.at(t1 + 0.001), n=32)
        return _clean(G.difference(G.union([body, ear]), [bore]))

    def ear_face_y(self) -> float:
        """y of the outboard case face at the tray top (the case is rolled about its output axis)."""
        c = (NDA_TRAY_Z + 0.0015 - self.Cp[2] + 0.02075 * self.e1[2]) / self.e2[2]
        return float(self.Cp[1] - 0.02075 * self.e1[1] + c * self.e2[1])

    def case_bolts(self):
        lo = np.asarray(self.eq["box"][0], float)
        y = lo[1] - 0.003 - NDA_EAR_OUT + 0.0062
        return [np.array([x, y, NDA_TRAY_Z + 0.003]) for x in (NDA_EAR_X[0] + 0.006, NDA_EAR_X[1] - 0.006)]

    def crank(self) -> G.Mesh:
        fb = self.m.fb
        t0, t1 = self.td("crank")
        plate = bar_mesh(fb, fb.C3, self.m.K3(), 0.0085, ND_EYE, t0, t1, hole1=0.5 * ND_PIN + 0.000025)
        tc = (fb.C3 - self.Cp) @ self.u                    # crank plane offset from the layout shaft point
        shaft = G.cylinder(0.004, self.at(self.t_case[1] - 0.010), self.at(tc + t0 + 0.0005), n=32)
        return _clean(G.union([plate, shaft]))

    def link(self) -> G.Mesh:
        t0, t1 = self.td("dlink")
        h = 0.5 * ND_PIN + 0.000025
        return _clean(bar_mesh(self.m.fb, self.m.K3(), self.m.q3(), ND_EYE, ND_EYE, t0, t1, hole0=h, hole1=h))

    def _door_horn(self, fb, pins2, t0, t1, side, j, eye) -> G.Mesh:
        """Horn bonded on a nose door: built with the doors closed (pins in the frame of ``fb``), turned to the rest
        (open) pose."""
        q2 = pins2[0]
        qy = float(fb.to3(q2)[1])
        ys = np.linspace(qy - 0.009, qy + 0.009, 7)
        ys = np.clip(ys, -0.0345, 0.0345)
        ys = ys[np.abs(ys) > 0.0012]
        foot = inner_face_2d(self.C, fb, 0.5 * (t0 + t1), ys, self.ND.t, side=side)
        poly = unary_union([Point(*p).buffer(eye, 48) for p in pins2] + [Polygon(list(foot) + [tuple(p) for p in pins2])
                                                                        .buffer(0)]).convex_hull
        for p in pins2:
            poly = poly.difference(Point(*p).buffer(0.5 * ND_PIN + 0.000025, 32))
        poly = poly.simplify(0.00003).buffer(0)
        m = _plate(poly, fb.p0, fb.e1, fb.e2, t0, t1)
        return _clean(G.Mesh(_rot(m.V, j["origin"], j["axis"], float(j["rest"])), m.F))

    def drive_pin2(self):
        """Drive pin of the port horn in the coupling frame, doors closed."""
        return self.cp.to2(self.m.fb.to3(self.m.fb.q_rest))

    # ---------------- coupling parts (rest pose: doors open)
    def c_horn(self, side: str) -> G.Mesh:
        q2 = self.qR0 if side == "R" else np.array([-self.qR0[0], self.qR0[1]])
        pins = [q2] if side == "R" else [q2, self.drive_pin2()]
        t0, t1 = self.tx("horn_" + side)
        return self._door_horn(self.cp, pins, t0, t1, 1.0 if side == "R" else -1.0,
                               self.jr if side == "R" else self.jl, NC_EYE)

    def c_link(self, side: str) -> G.Mesh:
        r = self.cpl(self.v_rest)
        t0, t1 = self.tx("link_" + side)
        h = 0.5 * NC_PIN + 0.000025
        return _clean(bar_mesh(self.cp, self.cp.to3(r["B"]), self.cp.to3(r["q" + side]), NC_EYE, NC_EYE, t0, t1,
                               hole0=h, hole1=h))

    def bar(self) -> G.Mesh:
        """700: slide bar 13 x 4 (7075) with the coupling pin at its lower end and two guide pins above it."""
        B = self.cpl(self.v_rest)["B"]
        t0, t1 = self.tx("bar")
        hw = 0.0065
        poly = unary_union([Point(*B).buffer(hw, 40), rect(B[0] - hw, B[0] + hw, B[1], B[1] + NC_GP[1] + hw)])
        for d in (0.0,) + NC_GP:
            poly = poly.difference(Point(B[0], B[1] + d).buffer(0.5 * NC_PIN + 0.000025, 32))
        return _clean(_plate(poly, self.cp.p0, self.cp.e1, self.cp.e2, t0, t1))

    def guide_slot(self):
        r0, r1 = self.cpl(self.v_rest)["B"][1], self.cpl(0.0)["B"][1]
        return min(r0, r1) + NC_GP[0] - 0.0016, max(r0, r1) + NC_GP[1] + 0.0016

    def guide(self) -> G.Mesh:
        """701: guide plate (7075, 4 mm) across the keel slot with the vertical guide slot, lightening windows and two
        flanges forward along the keel walls (one M4 each through the wall)."""
        t0, t1 = self.tx("guide")
        s0, s1 = self.guide_slot()
        wy = float(self.C.mem["M-KEELWALL"]["box"][0][1])
        zb = NC_GUIDE_ZB - self.cp.p0[2]
        plate = rect(-wy + 0.0005, wy - 0.0005, zb, s1 + 0.016).difference(rect(-0.0016, 0.0016, s0, s1))
        for sg in (1.0, -1.0):                    # lightening windows (8 mm webs)
            plate = plate.difference(rect(sg * 0.0096, sg * (wy - 0.0115), zb + 0.008, s1 - 0.024))
        m = _plate(plate, self.cp.p0, self.cp.e1, self.cp.e2, t0, t1)
        fls = [_plate(rect(min(sg * wy, sg * (wy - 0.003)), max(sg * wy, sg * (wy - 0.003)), s1 - 0.012, s1 + 0.016),
                      self.cp.p0, self.cp.e1, self.cp.e2, t0 - NC_FL_L, t0 + 0.0005) for sg in (1.0, -1.0)]
        return _clean(G.union([m] + fls))

    def guide_bolts(self):
        """(head point on the flange inner face, axis) of the guide-plate bolts (through the keel walls)."""
        t0, _t1 = self.tx("guide")
        s0, s1 = self.guide_slot()
        wy = float(self.C.mem["M-KEELWALL"]["box"][0][1])
        return [(self.cp.to3(np.array([sg * (wy - 0.003), s1 + 0.006])) + (t0 - 0.5 * NC_FL_L) * self.cp.u,
                 np.array([0.0, sg, 0.0])) for sg in (1.0, -1.0)]


NDA_TRAY_Z = -0.089                 # side-bay tray top face (TR-SIDEBAY-L)
NDA_EAR_X = (1.0480, 1.0720)        # nose-door DA 22 outboard ear on the side-bay tray (outboard of its cut-out)
NDA_EAR_OUT = 0.0140                # ear length beyond the tray cut-out (M3 nutplates, 7.5 mm composite edge distance)
NC_GUIDE_ZB = -0.1700               # guide plate lower edge (above the slot-end sill)
SIDE_EN, SIDE_TR = {"R": "starboard", "L": "port"}, {"R": "sağ", "L": "sol"}
PIN_X = 0.0003                      # pin ends beyond the outer plates
NC_FL_L = 0.016                     # guide-plate flange length on the keel walls


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
                 M4130, P_CNC, M.axle, thickness=AXLE_R - 0.006, joint=jR, parent=pid(621), contacts=(pid(621), pid(622),
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
                 notes="layout M-WELLKEEL: three stations on the inclined hinge line (GD-06), each bolted to the "
                       "outboard face of the well keel web (1 x M4, nut in the harness channel); ears 3 mm")
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
                 notes="either side of the leg slot inside the notch of the gear beam, each with a flange on the beam "
                       "web outside the notch (2 x M4 into potted inserts, GD-08); 0.3 mm bond line to the skin; "
                       "knuckles in the 4.2 mm cove of the skin edge (shell)")
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
        self.NG = NoseDrag(C, N)
        N.extra.append(self.NG.horn())
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
                       "integral horn of the drag link (pin d 10 at r 31 mm, centre plane y -10.5..-5.5 mm, GD-04); "
                       "steering-actuator lugs aft")
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
                 explode=(0.0, 0.0, 0.2), notes="layout ACT-NLG-EMA (GD-04): starboard half of the slot above the pivot, "
                 "two tabs bolted to the starboard keel-wall land (2 x M4); spline output on the port face drives the "
                 "crank 655")
        self.add(lg(647), "nose up-lock rollers on the axle ends (pair)", "burun yukarı kilit makaraları (aks uçlarında, "
                 "çift)", P_BUY, P_BUY, N.rollers, purchased=True, vendor="plain bearing rollers d 12 x 1.5 (catalogue "
                 "class, mass estimate)", mass_kg=0.004, joint=js, parent=lg(648), contacts=(lg(648),), step=S,
                 explode=ex)
        self.add(lg(650, "R"), "nose up-lock catch plate (4130 N), starboard", "burun yukarı kilit mandal plakası "
                 "(4130 N), sağ", M4130, P_CNC, N.catch, thickness=0.0023, parent=kw + "-R", contacts=(kw + "-R",),
                 step=S, explode=(0.0, 0.05, 0.0),
                 notes="layout F-NG-PIVOT locks: up-lock engaging the axle roller on the keel-wall face beside the "
                       "stowed axle; 2 x M4 from the side bay into tapped bosses (GD-08); latch pawl and release "
                       "solenoid of the gear unit")
        # nose doors
        from ..core.parts import layup_props
        areal = layup_props(C.S, "shell_secondary")["areal_mass"]
        self.add(lg(683, "R"), "nose-door hinge blocks (7075-T651, 3 off), starboard",
                 "burun kapağı menteşe blokları (7075-T651, 3 adet), sağ", M7075, P_CNC, ND.block_mesh, thickness=0.003,
                 parent=kw + "-R", contacts=(kw + "-R",), step=STEP_DOOR, explode=(0.0, 0.05, -0.15),
                 notes="one M4 per block through the keel wall (nut in the side bay, GD-08) above the hinge line; "
                       "knuckles in the 4.2 mm coves of the lower nose skin edge (shell)")
        self.add(lg(684, "R"), "nose-door hinge pin d 3 (ISO 2338 h8, 4130), starboard",
                 "burun kapağı menteşe pimi d 3 (ISO 2338 h8, 4130), sağ", P_BUY, P_BUY, ND.pin, purchased=True,
                 vendor="ISO 2338 h8 dowel d 3 x 450 + retaining rings (catalogue)", mass_kg=0.025,
                 parent=lg(683, "R"), contacts=(lg(683, "R"), lg(674, "R")), step=STEP_DOOR)
        self.add(lg(674, "R"), "nose clamshell door (CFRP sandwich), starboard", "burun istiridye kapağı (CFRP sandviç), "
                 "sağ", MPW, P_PREG, ND.door, thickness=ND.t, joint="nose_door_R", parent=lg(684, "R"),
                 contacts=(lg(684, "R"),), step=STEP_DOOR, explode=(0.0, 0.08, -0.2),
                 mass_kg=float(ND.o.buffer(-DOOR_GAP, join_style=2).area) * areal,
                 notes="layout door_outlines nose_door_R; modelled open (rest = gear down); knuckle tube on the hinge "
                       "line; coupling horn 698 bonded on the door (on the port door with the drive arm, GD-08)")

    def nose_fasteners(self):
        C, N = self.C, self.N
        reg = self.reg
        lg = lambda n, side="C": C.pid(n, side)
        fc = C.pid(628, "C", "FC")
        for k, (p, ax) in enumerate(N.steer_bolts()):
            J.bolt(reg, f"{fc}-B{k + 1}", 3, p, ax, [(fc, 0.026)], nut="tapped", tapped_part=lg(624),
                   tapped_depth=0.010, grade="12.9", step=STEP_NOSE)
        kw = C.mem["M-KEELWALL"]["part"] + "-R"
        wt_ = float(C.mem["M-KEELWALL"]["box"][1][1] - C.mem["M-KEELWALL"]["box"][0][1])
        for k, p in enumerate(N.ema_bolts()):
            J.bolt(reg, f"{lg(632)}-B{k + 1}", 4, p, (0.0, 1.0, 0.0), [(lg(632), EMA_TAB[1]), (kw, wt_)], grade="12.9",
                   step=STEP_NOSE, notes="EMA tab through the keel-wall land, nut in the starboard side bay")
        for sd in ("R", "L"):
            kws = C.mem["M-KEELWALL"]["part"] + f"-{sd}"
            sg = 1.0 if sd == "R" else -1.0
            wt = float(C.mem["M-KEELWALL"]["box"][1][1] - C.mem["M-KEELWALL"]["box"][0][1])
            tray = SIDEBAY_TRAY[sd]
            for k, (p, ax) in enumerate(N.catch_bolts()):
                J.bolt_through(reg, f"{lg(650, sd)}-B{k + 1}", 3, p * np.array([1.0, sg, 1.0]), ax * np.array([1.0, sg, 1.0]),
                               [tray, kws], owner=lg(650, sd), nut="tapped", tapped_part=lg(650, sd),
                               tapped_depth=0.0068, grade="12.9", step=STEP_NOSE,
                               notes="from the side bay through the tray flange and the wall into the tapped catch boss")
            for k, (p, ax) in enumerate(self.ND.block_bolts()):
                J.bolt_through(reg, f"{lg(683, sd)}-B{k + 1}", 4, p * np.array([1.0, sg, 1.0]),
                               ax * np.array([1.0, sg, 1.0]), [lg(683, sd), kws], grade="12.9", step=STEP_DOOR,
                               notes="hinge block through the keel wall, nut in the side bay")

    # ---------------------------------------------------------------- door drives, locks, brake installation
    def drives(self):
        """Gear detail round parts (GD-01..GD-12): door drives, nose drag link, main down-lock, brake installation."""
        C, M, Dr, N, ND = self.C, self.M, self.Dr, self.N, self.ND
        pid = lambda n: C.pid(n, "R")
        lg = lambda n, side="C": C.pid(n, side)
        fr = C.st["FS-GEAR"]["part"]
        # ---- main inner-door drive (GD-05)
        I = InnerDoorDrive(C, Dr)
        self.ID = I
        self.add(pid(685), "inner-door actuator wedge plate (7075-T651), starboard",
                 "iç kapak eyleyicisi kama plakası (7075-T651), sağ", M7075, P_CNC, I.plate, thickness=0.0036,
                 parent=fr, contacts=(fr,), step=STEP_DOOR, explode=(0.1, 0.0, 0.0),
                 notes="on the aft face of FS-GEAR (potted inserts): tilts the DA 22 so that its output is parallel to "
                       "the inner-door hinge line (3.9 deg)")
        self.add(C.pid(676, "R", "FC"), "main inner-door actuator Volz DA 22 (28 V), starboard",
                 "ana iç kapak eyleyicisi Volz DA 22 (28 V), sağ", P_BUY, P_BUY, I.case, purchased=True,
                 vendor="Volz DA 22-30-4128 (28 V, long case) 41.5 x 65.9 x 22 mm, 0.132 kg (datasheet)",
                 mass_kg=0.132, parent=pid(685), contacts=(pid(685),), step=STEP_DOOR, explode=(0.12, 0.0, 0.0),
                 notes="layout EQ-DOORACT (GD-05): 2 x M3 through the top ears and the wedge plate into the frame "
                       "inserts; output spline drives the crank shaft through the bearing housing in C-DOORSHAFT")
        self.add(pid(689), "inner-door drive bearing housing (7075-T651, 2 x 688-2RS bearings), starboard",
                 "iç kapak tahrik yatak yuvası (7075-T651, 2 x 688-2RS rulman), sağ", M7075, P_CNC, I.housing,
                 thickness=0.003, parent=fr, contacts=(fr,), step=STEP_DOOR, explode=(-0.08, 0.0, 0.0),
                 notes="flange on the forward face of FS-GEAR (2 x M3 into potted inserts), spigot in C-DOORSHAFT, "
                       "lip seal on the d 8 shaft (vapour-tight bulkhead)")
        self.add(pid(686), "inner-door drive crank with d 8 shaft (4130 N), starboard",
                 "iç kapak tahrik krankı, d 8 milli (4130 N), sağ", M4130, P_CNC, I.crank, thickness=0.003,
                 joint="main_idoor_crank_R", parent=pid(689), contacts=(pid(689), C.pid(676, "R", "FC")),
                 step=STEP_DOOR, explode=(-0.08, 0.0, 0.0),
                 notes=f"crank r {ID_RC * 1000:.0f} mm on the DA 22 spline; folded toggle with the door closed "
                       "(over-centre lock against the link stop)")
        self.add(pid(687), "inner-door drive link (7075-T651), starboard", "iç kapak tahrik biyeli (7075-T651), sağ",
                 M7075, P_CNC, I.link, thickness=0.003, joint="main_idoor_link_R", parent=pid(686),
                 contacts=(pid(686), pid(688)), step=STEP_DOOR, explode=(-0.08, 0.0, -0.05),
                 notes=f"link {I.m.fb.l * 1000:.1f} mm, pins d 4 (ISO 2341-B)")
        self.add(pid(688), "inner-door drive horn (7075-T651, bonded on the door), starboard",
                 "iç kapak tahrik kolu (7075-T651, kapağa yapıştırılmış), sağ", M7075, P_CNC, I.horn,
                 thickness=0.003, joint="main_inner_door_R", parent=pid(670), contacts=(pid(670), pid(687)),
                 step=STEP_DOOR, explode=(0.0, 0.0, -0.1),
                 notes="foot bonded (EA 9394, 0.3 mm) on the door inner skin at the aft end of the door")
        # ---- trunnion-door drive (GD-07)
        T = TrunnionDoorDrive(C, Dr)
        self.TD = T
        trun = "YK250-CH-070-R"
        beamR = C.mem["M-GEARBEAM"]["part"] + "-R"
        self.add(pid(695), "trunnion-door gearmotor bracket (7075-T651), starboard",
                 "mafsal kapağı dişli motor braketi (7075-T651), sağ", M7075, P_CNC, T.bracket, thickness=0.0018,
                 parent=beamR, contacts=(beamR, pid(691)), step=STEP_DOOR, explode=(0.0, -0.05, 0.0),
                 notes="half-clamp over the gearmotor (bonded, Loctite 638) with a tab on the inboard face of the "
                       "gear-beam web between the notch and the F-TRUNNION flange, 1 x M3 into a potted insert")
        self.add(pid(691), "trunnion-door gearmotor (d 16 x 21 mm micro DC gearmotor + encoder; estimate), starboard",
                 "mafsal kapağı dişli motoru (d 16 x 21 mm mikro DC dişli motor + enkoder; tahmin), sağ", P_BUY, P_BUY,
                 T.motor, purchased=True, vendor="micro DC gearmotor class d 16 mm, ~0.3 N m output (Faulhaber / "
                 "maxon class; envelope and mass are estimates)", mass_kg=0.045, parent=pid(695),
                 contacts=(pid(695),), step=STEP_DOOR, explode=(0.0, -0.05, 0.0),
                 notes="structures G-TDOOR-LINK: 0.257 N m limit at the crank; holding torque not needed in the "
                       "closed toggle (over-centre)")
        self.add(pid(692), "trunnion-door crank (4130 N), starboard", "mafsal kapağı krankı (4130 N), sağ",
                 M4130, P_CNC, T.crank, thickness=0.003, joint="main_tdoor_crank_R", parent=pid(691),
                 contacts=(pid(691),), step=STEP_DOOR, explode=(0.0, -0.05, 0.0),
                 notes=f"crank r {TD_RC * 1000:.0f} mm, folded toggle with the door closed")
        self.add(pid(693), "trunnion-door link (7075-T651), starboard", "mafsal kapağı biyeli (7075-T651), sağ",
                 M7075, P_CNC, T.link, thickness=0.003, joint="main_tdoor_link_R", parent=pid(692),
                 contacts=(pid(692), pid(694)), step=STEP_DOOR, explode=(0.0, -0.05, -0.03),
                 notes=f"link {T.m.fb.l * 1000:.1f} mm, pins d 3")
        self.add(pid(694), "trunnion-door horn (7075-T651, bonded on the door), starboard",
                 "mafsal kapağı kolu (7075-T651, kapağa yapıştırılmış), sağ", M7075, P_CNC, T.horn, thickness=0.003,
                 joint="main_trunnion_door_R", parent=pid(673), contacts=(pid(673), pid(693)), step=STEP_DOOR,
                 explode=(0.0, 0.05, -0.1))
        # ---- main down-lock (GD-03), anti-rotation key (GD-12), brake swivel (GD-09)
        dl = M.downlock()
        x_face = float(next(b for b in C.fit["F-TRUNNION"]["boxes"] if abs(b[0][2] - dl["lug_bottom_z"]) < 1e-6)[1][0])
        self.dl = dict(dl, x_face=x_face)

        def lock_bolt():
            p = dl["p"]
            return G.cylinder(dl["r"] - 0.00001, (M.arm_a[0] + DL_TIP, p[1], p[2]), (x_face + DL_TAIL, p[1], p[2]), n=48)

        def solenoid():
            p = dl["p"]
            neck = G.cylinder(SOL_NECK[0], (x_face - SOL_NECK[1], p[1], p[2]), (x_face + 0.0005, p[1], p[2]), n=48)
            body = G.cylinder(SOL_R, (x_face + 0.0003, p[1], p[2]), (x_face + SOL_L, p[1], p[2]), n=48)
            bore = G.cylinder(dl["r"] + 0.0002, (x_face - SOL_NECK[1] - 0.001, p[1], p[2]),
                              (x_face + DL_TAIL + DL_STROKE + 0.003, p[1], p[2]), n=40)
            return _clean(G.difference(G.union([neck, body]), [bore]))

        self.add(pid(641), "main down-lock bolt d 14 (4130 N, 36-40 HRC), starboard",
                 "ana aşağı kilit pimi d 14 (4130 N, 36-40 HRC), sağ", M4130, P_CNC, lock_bolt, thickness=0.007,
                 joint="main_downlock_R", parent=trun, contacts=(trun, pid(620)), step=STEP_MAIN,
                 notes="GD-03: spring-engaged in the aft yoke-arm boss with the gear down (single shear across the "
                       "1 mm lug-to-arm gap), withdrawn 15 mm aft by the solenoid before the leg moves; bores "
                       "bushed (layout F-TRUNNION downlock)")
        self.add(pid(638), "main down-lock solenoid (push-pull, 28 V, spring return; estimate), starboard",
                 "ana aşağı kilit solenoidi (it-çek, 28 V, yay dönüşlü; tahmin), sağ", P_BUY, P_BUY, solenoid,
                 purchased=True, vendor="tubular push-pull solenoid d 26 x 27 mm, 15 mm stroke, 28 V (catalogue "
                 "class; envelope and mass are estimates)", mass_kg=0.060, parent=trun, contacts=(trun,),
                 step=STEP_MAIN, notes="threaded neck in a counterbore of the aft lug face, jam nut + lock wire; "
                                       "lock-bolt spring inside; proximity sensor of the bolt position (gear-unit item)")
        self.add(pid(643), "main-leg anti-rotation key 3 x 2.5 x 9 (4130 N), starboard",
                 "ana bacak dönme önleyici kama 3 x 2,5 x 9 (4130 N), sağ", M4130, P_CNC,
                 lambda: M.key_box(0.0, (KEY[1], 0.0)), thickness=KEY[1], joint="main_gear_R", parent=pid(620),
                 contacts=(pid(620), pid(621)), step=STEP_MAIN,
                 notes="GD-12: the sliding tube is keyed to the gland instead of a scissor torque link (the 64 mm "
                       "leg slot leaves no room for a scissor beside the OD 42 cylinder); key in the gland keyway, "
                       "slot in the tube over the stroke")
        # ---- brake master unit (GD-09)
        eqb = C.eq["EQ-BRAKE_UNIT"]
        blo, bhi = np.asarray(eqb["box"][0], float), np.asarray(eqb["box"][1], float)
        self.brk = (blo, bhi)
        deck = eqb["mount"]["tray"]
        z_deck = float(self.reg.parts[deck].mesh.V[:, 2].min()) if deck in self.reg.parts else 0.014

        def hanger():
            xc = 0.5 * (blo[0] + bhi[0])
            top = G.difference(_box((blo[0] + 0.002, blo[1] - 0.003, z_deck - 0.003), (bhi[0] - 0.002, bhi[1] + 0.003, z_deck)),
                               [_box((blo[0] + 0.018, blo[1] + 0.013, z_deck - 0.004), (bhi[0] - 0.018, bhi[1] - 0.013,
                                                                                       z_deck + 0.001))])
            st = []
            for xs in (xc - 0.030, xc + 0.030):           # two U-straps round the unit (rubber-lined, clamped)
                x0, x1 = xs - 0.006, xs + 0.006
                st += [_box((x0, y0, blo[2] - 0.003), (x1, y1, z_deck - 0.0029))
                       for y0, y1 in ((blo[1] - 0.003, blo[1]), (bhi[1], bhi[1] + 0.003))]
                st.append(_box((x0, blo[1] - 0.003, blo[2] - 0.003), (x1, bhi[1] + 0.003, blo[2])))
            return _clean(G.union([top] + st))
        self.z_deck = z_deck
        self.add(C.pid(629, "C", "FC"), "brake actuator Volz DA 26 + master cylinder (brake-by-wire unit)",
                 "fren eyleyicisi Volz DA 26 + ana silindir (elektrikli fren birimi)", P_BUY, P_BUY,
                 lambda: _box(blo, bhi), group="controls", purchased=True,
                 vendor="Volz DA 26 (0.270 kg datasheet, case 102.8 x 54 x 26 mm) + master cylinder on its output "
                 "(0.065 kg estimate; layout.rules.bay_contents brake_unit 0.335 kg)", mass_kg=0.335,
                 parent=lg(658), contacts=(lg(658),), step=STEP_MAIN, explode=(0.0, -0.1, -0.1),
                 notes="layout EQ-BRAKE_UNIT (GD-09): hung from the avionics deck; two brake lines (Nylon 1/8 in) "
                       "along H-MAIN to the main-gear swivels")
        self.add(lg(658), "brake-unit hanger (7075-T651)", "fren birimi askısı (7075-T651)", M7075, P_CNC, hanger,
                 thickness=0.003, parent=deck, contacts=(deck,), step=STEP_MAIN, explode=(0.0, 0.0, -0.08),
                 notes="4 x M4 into potted inserts of the avionics deck; the unit sits in two rubber-lined U-straps "
                       "(no fasteners into the purchased case)")
        # ---- nose drag link (GD-04), spreader (GD-11)
        G4 = self.NG
        self.add(lg(655), "nose retraction crank (4130 N)", "burun toplama krankı (4130 N)", M4130, P_CNC,
                 G4.crank, thickness=0.005, joint="nose_drag_crank", parent=lg(632), contacts=(lg(632),),
                 step=STEP_NOSE, explode=(0.0, -0.05, 0.1),
                 notes=f"crank r {NG_RC * 1000:.1f} on the EMA spline; folded toggle with the gear down = down-lock")
        self.add(lg(653), "nose drag link (4130 N)", "burun sürükleme biyeli (4130 N)", M4130, P_CNC,
                 G4.link, thickness=0.005, joint="nose_drag_link", parent=lg(655), contacts=(lg(655), lg(624)),
                 step=STEP_NOSE, explode=(0.0, -0.05, 0.1),
                 notes=f"link {G4.m.fb.l * 1000:.1f} mm, pins d 10 (G-NLOCK)")
        self.add(lg(656), "nose stub-axle spreader sleeve with jack nut (7075-T651 / 4130)",
                 "burun mil aralık burcu, kriko somunlu (7075-T651 / 4130)", M7075, P_CNC, N.spreader, thickness=0.0035,
                 joint="nose_gear", parent=lg(624), contacts=(lg(640),), step=STEP_NOSE, explode=(0.0, 0.0, -0.3),
                 notes="GD-11: jacked against both stub flanges after insertion and lock-wired; the flange dowel "
                       "keys rotation")
        # ---- nose-door drive and coupling (GD-08)
        ND2 = NoseDoorDrive(C, ND)
        self.NDD = ND2
        tray = C.eq["EQ-NDOORACT"]["mount"]["tray"]
        self.add(C.pid(678), "nose-door actuator Volz DA 22 (28 V), drives both clamshell doors",
                 "burun kapağı eyleyicisi Volz DA 22 (28 V), iki kapağı tahrik eder", P_BUY, P_BUY, ND2.case,
                 purchased=True, vendor="Volz DA 22-30-4128 (28 V, long case), 0.132 kg (datasheet)", mass_kg=0.132,
                 parent=tray, contacts=(tray,), step=STEP_DOOR, explode=(-0.12, 0.0, 0.1),
                 notes="layout EQ-NDOORACT (GD-08): output parallel to the door hinge axes; forward ear on the "
                       "side-bay tray (2 x M4 + nutplates)")
        kwL = C.mem["M-KEELWALL"]["part"] + "-L"
        self.add(lg(679), "nose-door drive crank with shaft (4130 N)", "burun kapağı tahrik krankı, milli "
                 "(4130 N)", M4130, P_CNC, ND2.crank, thickness=0.003, joint="nose_ddrive_crank",
                 parent=C.pid(678), contacts=(C.pid(678),), step=STEP_DOOR, explode=(-0.05, -0.05, 0.0))
        self.add(lg(696), "nose-door drive link (7075-T651)", "burun kapağı tahrik biyeli (7075-T651)", M7075, P_CNC,
                 ND2.link, thickness=0.003, joint="nose_ddrive_link", parent=lg(679), contacts=(lg(679), lg(698, "L")),
                 step=STEP_DOOR, explode=(-0.05, -0.05, 0.0),
                 notes=f"link {ND2.m.fb.l * 1000:.1f} mm through the port keel-wall window W-NDOOR-LINK to the drive "
                       "arm of the port coupling horn")
        for sd in ("R", "L"):
            self.add(lg(698, sd), f"nose-door coupling horn (7075-T651, bonded on the door{', with the drive arm' if sd == 'L' else ''}), {SIDE_EN[sd]}",
                     f"burun kapağı bağlantı kolu (7075-T651, kapağa yapıştırılmış), {SIDE_TR[sd]}", M7075, P_CNC,
                     lambda sd=sd: ND2.c_horn(sd), thickness=0.003, joint=f"nose_door_{sd}", parent=lg(674, sd),
                     contacts=(lg(674, sd),) + ((lg(696),) if sd == "L" else (lg(699, sd),)), step=STEP_DOOR,
                     explode=(0.0, 0.0, -0.1), mirror=False)
            self.add(lg(699, sd), f"nose-door coupling link (7075-T651), {SIDE_EN[sd]}",
                     f"burun kapağı bağlantı biyeli (7075-T651), {SIDE_TR[sd]}", M7075, P_CNC,
                     lambda sd=sd: ND2.c_link(sd), thickness=0.003, joint=f"nose_dlink_{sd}", parent=lg(700),
                     contacts=(lg(700), lg(699, "R")) if sd == "L" else (lg(698, sd), lg(699, "L")), step=STEP_DOOR,
                     explode=(0.0, 0.0, -0.1), mirror=False,
                     notes=f"link {ND2.lc * 1000:.1f} mm, pins d 4")
        self.add(lg(700), "nose-door coupling slide bar (7075-T651) with two guide pins", "burun kapağı bağlantı "
                 "kızağı (7075-T651), iki kılavuz pimli", M7075, P_CNC, ND2.bar, thickness=0.004, joint="nose_dbar",
                 parent=lg(701), contacts=(lg(701),), step=STEP_DOOR, explode=(0.0, 0.0, 0.05))
        kwR = C.mem["M-KEELWALL"]["part"] + "-R"
        self.add(lg(701), "nose-door coupling guide plate (7075-T651)", "burun kapağı bağlantı kılavuz plakası "
                 "(7075-T651)", M7075, P_CNC, ND2.guide, thickness=0.004, parent=kwR, contacts=(kwR, kwL),
                 step=STEP_DOOR, explode=(0.0, 0.0, 0.05),
                 notes="across the keel slot, flanges bolted through both keel walls (M4, nuts in the side bays)")

    def mech_joints(self):
        """Joints of the gear detail round: crank / link chains posed by exact planar kinematics, the joint values on
        the gear_retraction sequence and the same values as piecewise-linear expressions of gear_up."""
        reg = self.reg
        seq = self.C.L["mechanisms"]["sequences"]["gear_retraction"]
        gs = [float(v) for v in seq["values"]]
        base = [{k: float(v) for k, v in st.items()} for st in seq["states"]]
        tables: dict[str, list[float]] = {}
        defs = []                                 # (name, kind, origin, axis, parent, mirror)

        def mech(prefix, m: Mech, mirror):
            vals = [m.values(st) for st in base]
            c, k = prefix + "crank" + ("_" if mirror else ""), prefix + "link" + ("_" if mirror else "")
            tables[c] = [v[0] for v in vals]
            tables[k] = [v[1] for v in vals]
            defs.append((c, "revolute", m.fb.C3, m.fb.u, None, mirror))
            defs.append((k, "revolute", m.K3(), m.fb.u, c, mirror))
        mech("main_idoor_", self.ID.m, True)
        mech("main_tdoor_", self.TD.m, True)
        mech("nose_drag_", self.NG.m, False)
        mech("nose_ddrive_", self.NDD.m, False)
        tables["main_downlock_"] = [DL_STROKE * min(max((g - 0.02) / 0.08, 0.0), 1.0) for g in gs]
        defs.append(("main_downlock_", "prismatic", self.dl["p"], np.array([1.0, 0.0, 0.0]), None, True))
        cv = [self.NDD.cpl_values(st) for st in base]
        r = self.NDD.cpl(self.NDD.v_rest)
        tables["nose_dbar"] = [v[0] for v in cv]
        tables["nose_dlink_R"] = [v[1] for v in cv]
        tables["nose_dlink_L"] = [v[2] for v in cv]
        cp = self.NDD.cp
        defs.append(("nose_dbar", "prismatic", cp.to3(r["B"]), cp.e2, None, False))
        defs.append(("nose_dlink_R", "revolute", cp.to3(r["B"]), cp.u, "nose_dbar", False))
        defs.append(("nose_dlink_L", "revolute", cp.to3(r["B"]), cp.u, "nose_dbar", False))
        self.seq_tables = {}
        for name, kind, o, a, par, mir in defs:
            vals = tables[name]
            lo, hi = _seq_bounds(vals)
            expr = pl_expr(gs, vals)
            names = [(name + "R" if mir else name, o, a, (par + "R") if (mir and par) else par)]
            if mir:
                a_l = np.array([a[0], -a[1], a[2]]) if kind == "prismatic" else -np.array([a[0], -a[1], a[2]])
                names.append((name + "L", np.array([o[0], -o[1], o[2]]), a_l, (par + "L") if par else None))
            for nm, o_, a_, p_ in names:
                reg.add_joint(Joint(name=nm, kind=kind, origin=np.asarray(o_, float), axis=np.asarray(a_, float), lo=lo,
                                    hi=hi, rest=0.0, parent=p_, expr=expr,
                                    notes="gear detail round: exact planar kinematics sampled on gear_retraction "
                                          "(docs/detail/gear.md section 7)"))
                self.seq_tables[nm] = vals

    def seq_extra(self, g: float, st: dict) -> dict:
        gs = [float(v) for v in self.C.L["mechanisms"]["sequences"]["gear_retraction"]["values"]]
        k = min(range(len(gs)), key=lambda i: abs(gs[i] - g))
        return {nm: v[k] for nm, v in self.seq_tables.items()}

    def drive_fasteners(self, side: str):
        """Pins and bolts of the gear detail round (main gear: both sides on their own parts; nose: once)."""
        C, reg = self.C, self.reg
        s = 1.0 if side == "R" else -1.0
        mv = lambda v: np.asarray(v, float) * np.array([1.0, s, 1.0])
        pid = lambda n: C.pid(n, side)
        fr = C.st["FS-GEAR"]["part"]
        I, T = self.ID, self.TD
        # inner-door drive
        for nm, P, ids, t in (("P1", I.m.K3(), (pid(686), pid(687)), (ID_T["crank"][1], ID_T["link"][0])),
                              ("P2", I.m.q3(), (pid(687), pid(688)), (ID_T["link"][1], ID_T["horn"][0]))):
            a = I.m.fb.u
            p0 = P + (max(t) + 0.0005) * a
            p1 = P + (min(t) - 0.0005) * a
            pin_through(reg, f"{pid(687)}-{nm}", ID_PIN, mv(p0), mv(p1), ids, step=STEP_DOOR,
                        notes="ISO 2341-B d 4 + split pin; shim washers set the 0.5 mm plane gaps")
        for k, p in enumerate(I.ear_bolts()):
            te = 0.003 / I.u[0]                                  # ear along x (bolt normal to the frame face)
            J.bolt(reg, f"{C.pid(676, side, 'FC')}-B{k + 1}", 3, mv(p), (-1.0, 0.0, 0.0), [(C.pid(676, side, "FC"), te),
                   (pid(685), p[0] - I.xf1 - te)], nut="insert", insert_part=fr, insert_depth=0.0068, grade="12.9", step=STEP_DOOR,
                   notes="potted insert of the FS-GEAR sandwich (no through-bolts in the fuel-bay bulkhead)")
        for k, p in enumerate(I.housing_bolts()):
            J.bolt(reg, f"{pid(689)}-B{k + 1}", 3, mv(p), (1.0, 0.0, 0.0), [(pid(689), 0.003)], nut="insert",
                   insert_part=fr, insert_depth=0.0068, grade="12.9", step=STEP_DOOR)
        # trunnion-door drive
        trun = f"YK250-CH-070-{side}"
        for nm, P, ids, t in (("P1", T.m.K3(), (pid(692), pid(693)), (TD_T["crank"][0], TD_T["link"][1])),
                              ("P2", T.m.q3(), (pid(693), pid(694)), (TD_T["link"][0], TD_T["horn"][1]))):
            p0 = P + (min(t) - 0.0005) * T.m.fb.u
            p1 = P + (max(t) + 0.0005) * T.m.fb.u
            pin_through(reg, f"{pid(693)}-{nm}", TD_PIN, mv(p0), mv(p1), ids, step=STEP_DOOR)
        beam = C.mem["M-GEARBEAM"]["part"] + f"-{side}"
        beam = beam if beam in reg.parts else C.mem["M-GEARBEAM"]["part"]
        for k, p in enumerate(T.bracket_bolts()):
            J.bolt(reg, f"{pid(695)}-B{k + 1}", 3, mv(p), (0.0, s, 0.0), [(pid(695), 0.003)], nut="insert",
                   insert_part=beam, insert_depth=0.0068, grade="12.9", step=STEP_DOOR,
                   notes="potted insert in the gear-beam web below the F-TRUNNION flange")
        # trunnion-door hinge-block feet riveted through the skin (GD-08)
        skin = C.pan["P-CENTRE-LOWER"]["part"]
        for k, (p, ax) in enumerate(self.Dr.tr_rivets()):
            stk = ([(skin, self.Dr.t_skin + BOND_TB)] if skin in reg.parts else []) + [(pid(681), 0.003)]
            J.rivet(reg, f"{pid(681)}-R{k + 1}", 0.0024, mv(p), mv(ax), stk,
                    owner=pid(681), spec="blind rivet NAS9301B-3 (CherryMAX, 100 deg countersunk head, A286 stem)",
                    step=STEP_DOOR, notes="flush head on the OML; the foot is bonded as well (EA 9394)")
        # down-lock solenoid, brake swivel, up-lock catch pin

    def nose_drive_fasteners(self):
        C, reg = self.C, self.reg
        lg = lambda n, side="C": C.pid(n, side)
        G4, D2 = self.NG, self.NDD
        u = G4.m.fb.u
        pin_through(reg, f"{lg(653)}-P1", NG_PIN, G4.m.K3() + (NG_Y["link"][0] - PIN_X) * u,
                    G4.m.K3() + (NG_Y["crank"][1] + PIN_X) * u, (lg(655), lg(653)), step=STEP_NOSE,
                    notes="clevis bolt d 6, heat-treated alloy steel (gear-unit requirement G-NLOCK), castellated nut")
        pin_through(reg, f"{lg(653)}-P2", NG_PIN, G4.m.q3() + (NG_Y["horn"][0] - PIN_X) * u,
                    G4.m.q3() + (NG_Y["link"][1] + PIN_X) * u, (lg(653), lg(624)), step=STEP_NOSE)
        fb = D2.m.fb
        for nm, P, ids, keys in (("P1", D2.m.K3(), (lg(679), lg(696)), ("crank", "dlink")),
                                 ("P2", D2.m.q3(), (lg(696), lg(698, "L")), ("horn_L", "dlink"))):
            t = D2.td(keys[0]) + D2.td(keys[1])
            pin_through(reg, f"{lg(696)}-{nm}", ND_PIN, P + (min(t) - PIN_X) * fb.u, P + (max(t) + PIN_X) * fb.u, ids,
                        step=STEP_DOOR)
        r = D2.cpl(D2.v_rest)
        cp = D2.cp
        tt = D2.tx
        on = lambda P, t: P + (t - (P - cp.p0) @ cp.u) * cp.u
        for sd in ("R", "L"):
            P = cp.to3(r["q" + sd])
            t = tt("horn_" + sd) + tt("link_" + sd)
            pin_through(reg, f"{lg(699, sd)}-P1", NC_PIN, on(P, min(t) - PIN_X), on(P, max(t) + PIN_X),
                        (lg(699, sd), lg(698, sd)), step=STEP_DOOR)
        Bp = cp.to3(r["B"])
        pin_through(reg, f"{lg(700)}-P1", NC_PIN, on(Bp, tt("link_R")[0] - PIN_X), on(Bp, tt("bar")[1] + PIN_X),
                    (lg(699, "L"), lg(699, "R"), lg(700)), step=STEP_DOOR)
        for k, d in enumerate(NC_GP):
            Pg = cp.to3(r["B"] + np.array([0.0, d]))
            pin_through(reg, f"{lg(700)}-P{k + 2}", NC_PIN, on(Pg, tt("bar")[0] + PIN_X), on(Pg, tt("guide")[1] + 0.0012),
                        (lg(700),), step=STEP_DOOR,
                        notes="shouldered guide pin pressed in the bar, running in the slot of the guide plate, "
                              "washer + retaining ring behind the plate")
        tray = C.eq["EQ-NDOORACT"]["mount"]["tray"]
        for k, p in enumerate(D2.case_bolts()):
            J.bolt_through(reg, f"{C.pid(678)}-B{k + 1}", 3, p, (0.0, 0.0, -1.0), [C.pid(678), tray], nut="nutplate",
                           grade="12.9", step=STEP_DOOR, notes="nutplate under the side-bay tray")
        kw = C.mem["M-KEELWALL"]["part"]
        wt = float(C.mem["M-KEELWALL"]["box"][1][1] - C.mem["M-KEELWALL"]["box"][0][1])
        for k, (p, ax) in enumerate(D2.guide_bolts()):
            sd = "R" if ax[1] > 0 else "L"
            J.bolt(reg, f"{lg(701)}-B{k + 1}", 4, p, ax, [(lg(701), 0.003), (f"{kw}-{sd}", wt)], grade="12.9",
                   step=STEP_DOOR, notes="nut in the side bay")
        # brake hanger
        blo, bhi = self.brk
        deck = C.eq["EQ-BRAKE_UNIT"]["mount"]["tray"]
        k = 0
        for x in (blo[0] + 0.010, bhi[0] - 0.010):
            for y in (blo[1] + 0.008, bhi[1] - 0.008):
                k += 1
                J.bolt(reg, f"{lg(658)}-B{k}", 4, (x, y, self.z_deck - 0.003), (0.0, 0.0, 1.0), [(lg(658), 0.003)],
                       nut="insert", insert_part=deck, insert_depth=0.0068, grade="12.9", step=STEP_MAIN)


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
        for k, (x0, x1) in enumerate(self.Dr.stations):
            J.bolt_through(reg, f"{pid(671)}-B{k + 1}", 4, mv((0.5 * (x0 + x1), yw1 + 0.002, zb + 0.015)),
                           mv((0.0, -1.0, 0.0)), [pid(671), wk], grade="12.9", step=STEP_DOOR,
                           notes="one M4 per hinge station, nut in the harness channel")
        s0, s1 = M.roller_s
        pin_through(reg, f"{pid(620)}-P9", ROLLER_PIN_D, mv(C.P(s0 - 0.001, ROLLER_E)),
                    mv(C.P(s1 + 0.001, ROLLER_E)), (pid(620),), step=STEP_MAIN,
                    notes="roller pin through the leg lug; rollers ride on it against the lug faces")
        pv = M.hook_pivot()
        upl = C.fit["F-UPLOCK"]["part"] + f"-{side}"
        pin_through(reg, f"{pid(642)}-P1", HOOK_PIN_D, mv((pv[0], HOOK_Y[0] - 0.001, pv[2])),
                    mv((pv[0], HOOK_Y[3] + 0.001, pv[2])), (pid(642), upl), step=STEP_MAIN,
                    notes="catch pivot pin through the F-UPLOCK boss bore (layout pin, GD-10: 10 mm = 2 D edge "
                          "distance to the boss bottom, 10.5 mm to its faces)")

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
    B.drives()
    B.mech_joints()
    B.mirror()
    for side in ("R", "L"):
        B.main_fasteners(side)
        B.drive_fasteners(side)
    B.nose_fasteners()
    B.nose_drive_fasteners()
    B.sequence(extra=B.seq_extra)
