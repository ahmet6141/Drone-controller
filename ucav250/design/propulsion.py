"""Propulsion producer (YK-250 HANCER): the Limbach L 275 EF installation behind the firewall and its pusher propeller.

What it builds (part numbers from ``spec.layout.part_numbers.propulsion`` 500-569; ids fixed by the layout are kept:
engine PR-500, exhaust PR-504-R/L, ECU PR-510, generator power electronics PR-512, heat protection PR-520..523,
propeller PR-540, spinner PR-542, S-duct PR-546):

* engine (purchased, envelope model inside ``layout.keep_outs[KO-ENGINE]``): crankcase with the four rear M8 mount
  bosses on the layout bolt pattern, twin finned cylinders + heads with the bolted heat sinks, spark plugs and caps,
  exhaust port bosses, piston-port intake manifolds, throttle bodies on a common cross shaft, injectors + fuel rail,
  hemispherical mesh air filters below the crankcase, the rear output (generator coupling) housing; the propeller
  output spool (crank-nose flange) is a separate rotating part (``prop_spin``);
* ePropelled SG750 starter-generator on a machined 7075 adapter on the coupling flange (counter-bored M4 into the
  rear housing, M4 through-bolts from the generator face);
* four conical elastomer isolators in the chassis mount cups (``layout.chassis.engine_mount.isolators``), each with
  its M8 12.9 through-bolt (head + washer on the cup bottom, 12.5 mm thread in the crankcase boss);
* propeller group on ``layout.mechanisms.joints[prop_spin]``: 110 mm 7075 hub spacer (forward flange 6 x M6 into the
  output spool, aft flange tapped for the 6 x M6 propeller bolts), spinner backplate clamped under the propeller hub,
  Mejzlik 31x12 3B (one-piece carbon, blade planform from ``spec.propeller.blade_model``), 7075 crush plate with the
  integral spinner stand-off, CFRP spinner cone with blade slots, centre M5 screw at the tip;
* exhaust (each side): stainless 304 port flange + header, silencer can and tail pipe inside the routing envelope
  ``KO-EXHAUST-R/L`` ending at the layout exit point / direction, 2 x M6 into the cylinder port boss;
* cooling: CFRP S-duct from the dorsal inlet throat to the firewall (``KO-COOLING-DUCT`` corridor, exit section through
  ``C-DUCT``), stainless firewall transition duct (fireproof spigot through C-DUCT, rising over the mount truss), silicone
  -glass coupling boot, engine-mounted 5052 plenum with the cylinder baffle skirts (air down through the fin packs),
  stainless node heat baffles (``layout.heat_protection.hardware[F-SPINDLE-NODE]``);
* engine accessories: throttle actuator Volz DA 22 on a 5052 bracket under the crankcase (direct drive of the throttle
  cross shaft), engine ECU on the mission tray, generator power electronics on the port side-bay tray (layout boxes);
* heat protection (``layout.heat_protection``): stainless exit insert and foil shields - registered only when their host
  parts (lower cowl halves, stub root strips, stabilator stubs) are in the registry.

Interfaces read (never another module's geometry): ``spec.layout`` (part numbers, chassis.engine_mount: thrust axis,
mount face, bolt pattern, isolator centres, cups; stations FS3480 / FS3670 cut-outs; keep_outs KO-ENGINE /
KO-EXHAUST-R/L / KO-COOLING-DUCT / KO-PROP; mechanisms prop_spin; systems equipment EQ-ECU / EQ-GENERATOR_PE with their
trays; heat_protection; clearances), ``spec.propeller`` (hub, spacer, spinner, blade model, mass), ``spec.engine``
(envelope, installed item masses), ``spec.fuselage`` (OML, for the cowl-following plenum roof and the heat-shield
panels), ``spec.materials`` / ``spec.processes``. Engine-internal dimensions not in the spec are module-private estimates
listed below and in ``ucav250/docs/detail/propulsion.md``.
"""
from __future__ import annotations

import math

import numpy as np
from shapely.geometry import Point, Polygon
from shapely.geometry import box as sbox
from shapely.ops import unary_union

from ..core import geom as G
from ..core.parts import Joint, Part, Registry, mirror_part, part_number
from . import fastener_catalog as FC
from . import joints as J
from . import oml as O

GROUP = "propulsion"

# =====================================================================================================================
# module-private detailing constants (estimates unless a source is named; docs/detail/propulsion.md)
# =====================================================================================================================
OV = 2e-4                   # boolean overlap of fused features (ARCHITECTURE section 5)
GAP = 5e-4                  # running / assembly gap between neighbouring parts that do not touch
# --- engine (engine frame: u ahead of the propeller plane along the crank axis, v = BL, w = engine up) ------------
U_COUPLING_FROM_MOUNT = 0.040   # engine.yaml installation.datum_distances crankcase_mount_face_to_generator_coupling
SPOOL_R, SPOOL_T = 0.0365, 0.010    # output spool flange OD 73 (engine.yaml prop_flange_outer_diameter_estimate 72 +-5)
SPOOL_HUB_R = 0.016         # spool stub in the crank-nose bore
SPOOL_RECESS_R = 0.011      # crank-nose bolt recess
NOSE_R = 0.032              # front bearing housing (twin 20 mm ball bearings, engine.yaml)
CASE_V = 0.058              # crankcase half width to the cylinder base faces
CASE_W = (-0.072, 0.046)    # crankcase lower / upper face (w)
CASE_RC = 0.020             # crankcase corner radius
BOSS_R = 0.0165             # M8 mount boss radius (>= 2 D edge distance for the tapped M8)
REAR_R = 0.026              # rear bearing / generator coupling housing radius (>= 10 mm to the chassis cups)
REAR_BOLT_R = 0.017         # 4 x M4 adapter screws (2 D edge distance to the housing rim and the coupling recess)
COUPLING_RECESS_R = 0.007
CYL_U = 0.245               # cylinder axis station (centre of layout cylinder slab 0.18..0.31; the real axial
#                             stagger of the boxer cylinders is not modelled)
CYL_R = 0.040               # barrel core (bore 66 mm + liner / wall)
CYL_V = (0.057, 0.176)      # base flange face .. barrel end
FIN = dict(hu=0.049, hw=0.057, rc=0.012, t=0.0015, v0=0.070, pitch=0.0065, n=16)
HEAD_V = (0.170, 0.184)     # head + bolted heat sink
HEAD_HU, HEAD_HW = 0.046, 0.054
PLUG = (0.184, 0.1980, 0.0095)  # plug + cap to the KO-ENGINE width (engine.envelope.width 0.3968)
EXH_PORT_V = 0.163          # exhaust port centre on the barrel underside (inside the KO-EXHAUST routing boxes)
EXH_BOSS = dict(hu=0.033, v0=0.151, v1=0.175, w_face=-0.062)
EXH_BOLT_DU = 0.021         # 2 x M6 port bolts (heads 5 mm clear of the header pipe)
INTAKE_V = 0.085            # intake manifold station (v), below the crankcase into the KO intake box
THROTTLE_W = (-0.128, -0.168)
FILTER_R, FILTER_W = 0.044, -0.190
CROSS_SHAFT_W = -0.148
# --- SG750 + adapter ---------------------------------------------------------------------------------------------
SG750_R, SG750_T = 0.0505, 0.0286   # engine.yaml installation_items.sg750 (101 mm dia x 28.6 mm)
ADAPTER_T, ADAPTER_R = 0.010, 0.040
SG_BOLT_R = 0.032           # 4 x M4 through the generator into the adapter
# --- isolators (layout: conical elastomer d 40 x 25 envelope, estimate) ------------------------------------------
ISO_R_BASE, ISO_R_TOP, ISO_SLEEVE_R = 0.020, 0.015, 0.007
ELASTOMER_DENSITY = 1200.0  # natural-rubber class (estimate; spec.materials has no elastomer)
# --- propeller group ----------------------------------------------------------------------------------------------
SPACER = dict(r_fl_f=0.0365, t_fl_f=0.008, r_o=0.035, r_i=0.0325, r_fl_a=0.044, t_fl_a=0.010, bore=0.011,
              pcd_f=0.024, pcd_a=0.031, access_r=0.003)
BACKPLATE_T = 0.002
CRUSH_T, CRUSH_R = 0.005, 0.042
STANDOFF_RO, STANDOFF_RI, STANDOFF_SOLID = 0.008, 0.0050, 0.012
HUB_R, HUB_BORE = 0.050, 0.011
SPINNER_T, SPINNER_TIP_T, SPINNER_TIP_R = 0.0008, 0.004, 0.016
BLADE_CLEAR = 0.003         # spinner slot clearance round the blade
BLADE_AIRFOIL = "naca4410"  # blade section (estimate: t/c from spec.propeller.blade_model, 4 % camber)
BLADE_STACK = 0.35          # blade sections stacked on 35 % chord
BLADE_ROOT_R = 0.045        # blade root buried in the hub disc
# --- exhaust ------------------------------------------------------------------------------------------------------
EXH_T = 0.0008              # tube / can wall (stainless 304 sheet / tube)
HEADER_R = 0.011            # header OD 22
TAIL_R = 0.010              # tail pipe OD 20 (fits the KO-EXHAUST exit box with the exit ring)
EXH_FLANGE_T = 0.006
CAN = dict(r=0.0215, x0=3.803, x1=3.893, y=0.1785, z=0.100)   # silencer can (axis along x)
# --- cooling ------------------------------------------------------------------------------------------------------
DUCT_T = 0.0008             # S-duct CFRP 4 plies PW
SPIGOT_T = 0.0005           # stainless transition duct
BAFFLE_T = 0.0008           # 5052-H32 plenum / baffle sheet
COWL_SKIN = 0.0058          # shell_secondary laminate (layups) - the plenum roof stays >= 10 mm inside it
DYN = 0.010                 # engine dynamic margin (layout.clearance_values.engine_keep_out)

MAT_SS, MAT_7075, MAT_6061, MAT_5052 = "ss_304_annealed", "al_7075_t651_plate", "al_6061_t6_sheet", "al_5052_h32_sheet"
MAT_PW, MAT_STEEL = "cfrp_pw_mtm45_as4", "steel_4130_n"
P_CNC, P_SHEET_AL, P_SHEET_ST, P_PREG, P_BUY = ("cnc_milling_metal", "sheet_metal_aluminium", "sheet_metal_steel",
                                                "prepreg_ooa_vacbag", "purchased")
STEP_FW, STEP_AVI, STEP_ENGINE, STEP_ACC, STEP_PROP, STEP_STUB, STEP_CLOSE = 12, 20, 25, 26, 27, 31, 36
EX_ENG = 0.30               # exploded-view offset of the engine group along the thrust axis (aft)


# =====================================================================================================================
# geometry helpers
# =====================================================================================================================
def rrect(a0, a1, b0, b1, r) -> Polygon:
    """Rounded rectangle [a0, a1] x [b0, b1] with corner radius r."""
    r = min(r, 0.499 * (a1 - a0), 0.499 * (b1 - b0))
    return sbox(a0 + r, b0 + r, a1 - r, b1 - r).buffer(r, 16)


def box3(lo, hi) -> G.Mesh:
    lo, hi = np.asarray(lo, float), np.asarray(hi, float)
    return G.box(hi - lo, center=0.5 * (lo + hi))


def union(ms) -> G.Mesh:
    ms = [m for m in ms if m is not None]
    return ms[0] if len(ms) == 1 else G.union(ms)


def diff(a: G.Mesh, cutters) -> G.Mesh:
    cutters = [c for c in cutters if c is not None]
    return G.difference(a, cutters) if cutters else a


def inter(a: G.Mesh, b: G.Mesh) -> G.Mesh:
    m = G.intersection(a, b)
    if m is None:
        raise ValueError("empty intersection")
    return m


def largest_piece(m: G.Mesh) -> G.Mesh:
    import manifold3d as m3
    parts = m.to_manifold().decompose()
    if len(parts) <= 1:
        return m
    best = max(parts, key=lambda p: p.volume())
    return G.Mesh.from_manifold(best)


def finish(m: G.Mesh) -> G.Mesh:
    """Round-trip through manifold3d (merges duplicate vertices, drops slivers) and verify closedness."""
    m = G.Mesh.from_manifold(m.to_manifold())
    if not m.check()["ok"]:
        raise ValueError("propulsion: open or inverted mesh")
    return m


def prism_axis(poly2d, a0: float, a1: float, axis: int) -> G.Mesh:
    """Prism of a shapely polygon drawn in the plane normal to local ``axis`` (0: (y, z), 1: (x, z), 2: (x, y)),
    extruded from a0 to a1 along that axis (local coordinates)."""
    if axis == 0:
        return G.extrude(poly2d, a1 - a0, origin=(a0, 0, 0), u=(0, 1, 0), v=(0, 0, 1))
    if axis == 1:                           # (x, z) plane, extrusion along +y: u = z, v = x gives z x x = +y
        P = Polygon([(q[1], q[0]) for q in np.asarray(poly2d.exterior.coords)])
        holes = [[(q[1], q[0]) for q in np.asarray(h.coords)] for h in poly2d.interiors]
        P = Polygon(P.exterior.coords, holes)
        return G.extrude(P, a1 - a0, origin=(0, a0, 0), u=(0, 0, 1), v=(1, 0, 0))
    return G.extrude(poly2d, a1 - a0, origin=(0, 0, a0), u=(1, 0, 0), v=(0, 1, 0))


def tube_along(path, r_o, r_i, n=20, ext=0.002) -> tuple[G.Mesh, G.Mesh]:
    """Solid round rod (outer) and bore (inner, extended ``ext`` beyond both ends) along a polyline."""
    P = np.asarray(path, float)
    t0 = G.unit(P[1] - P[0])
    t1 = G.unit(P[-1] - P[-2])
    Pi = np.vstack([P[0] - ext * t0, P, P[-1] + ext * t1])
    return G.sweep_circle(P, r_o, n=n), G.sweep_circle(Pi, r_i, n=n)


def arc_path(a, ta, b, tb, n=12, r_min=0.0):
    """Smooth path from a (tangent ta) to b (tangent tb): cubic Hermite with tangent length 0.5 x chord."""
    a, b = np.asarray(a, float), np.asarray(b, float)
    L = float(np.linalg.norm(b - a))
    ta, tb = G.unit(ta) * 0.6 * L, G.unit(tb) * 0.6 * L
    s = np.linspace(0, 1, n)[:, None]
    h00, h10, h01, h11 = 2 * s ** 3 - 3 * s ** 2 + 1, s ** 3 - 2 * s ** 2 + s, -2 * s ** 3 + 3 * s ** 2, s ** 3 - s ** 2
    return h00 * a + h10 * ta + h01 * b + h11 * tb


# =====================================================================================================================
# context: engine frame and layout lookups
# =====================================================================================================================
class Ctx:
    """Spec access and the engine frame. Engine frame (local x, y, z) = (u, v, w): u ahead of the propeller plane
    along the crank axis (from ``propeller.hub``), v = BL, w = engine up (normal to the axis in the symmetry plane).
    The frame is left-handed (det -1); :meth:`to_air` maps a local mesh with the winding fixed."""

    def __init__(self, reg: Registry, spec: dict):
        self.reg, self.S = reg, spec
        self.L = L = spec["layout"]
        self.n0 = int(L["part_numbers"]["propulsion"][0])
        self.n1 = int(L["part_numbers"]["propulsion"][1])
        pr = spec["propeller"]
        self.H = np.asarray(pr["hub"], float)
        eng = L["chassis"]["engine_mount"]
        d = G.unit(eng["thrust_axis"]["direction_aft"])
        self.d = d
        self.eu, self.ev = -d, np.array([0.0, 1.0, 0.0])
        self.ew = G.unit(np.array([-d[2], 0.0, d[0]]))  # (-sin, 0, cos): engine up
        self.R = np.column_stack([self.eu, self.ev, self.ew])
        self.eng = eng
        self.ko = {k["id"]: k for k in L["keep_outs"]}
        self.st = {s["id"]: s for s in L["stations"]}
        self.eq = {e["id"]: e for e in L["systems"]["equipment"]}
        self.fus = O.fuselage_from_spec(spec)
        # datums along the crank axis (u)
        self.u_flange = float(pr["hub_half_thickness"]) + float(pr["hub_spacer"])
        self.u_mount = float(eng["mount_face"]["distance_ahead_of_prop_plane"])
        self.u_coupling = self.u_mount + U_COUPLING_FROM_MOUNT
        ko = {b["id"]: b for b in self.ko["KO-ENGINE"]["boxes"]}
        self.ko_eng = ko
        self.u_sg_front = float(ko["crankcase_sg750"]["u"][1])
        self.v_max = float(ko["cylinders_heads"]["v"][1])
        self.bolts_vw = [tuple(self.loc(p)[1:]) for p in eng["bolts"]["points"]]
        self.iso_u = [float(self.loc(c)[0]) for c in eng["isolators"]["centres"]]
        self.thread_in_boss = 0.0125 + 2 * 0.00125    # 12 mm engagement in the bosses (layout bolts spec)

    # ------------------------------------------------------------------ frames
    def P(self, u, v, w) -> np.ndarray:
        return self.H + u * self.eu + v * self.ev + w * self.ew

    def loc(self, p) -> np.ndarray:
        return self.R.T @ (np.asarray(p, float) - self.H)

    def to_air(self, m: G.Mesh) -> G.Mesh:
        return m.transformed(self.R, self.H)

    def vec(self, du, dv, dw) -> np.ndarray:
        return du * self.eu + dv * self.ev + dw * self.ew

    def pid(self, num: int, side: str = "C") -> str:
        if not self.n0 <= num <= self.n1:
            raise ValueError(f"propulsion part number {num} outside {self.n0}..{self.n1}")
        return part_number(GROUP, num, side)

    def oml_y(self, x: float, z: float) -> float:
        """Half width of the fuselage OML at station x and height z (0 outside the section)."""
        w, h, zc, nt, nb = (float(a) for a in self.fus.section(x))
        tf = float(self.fus.top_frac(x))
        hh = tf * h if z >= zc else (1.0 - tf) * h
        n = nt if z >= zc else nb
        q = abs(z - zc) / hh
        return 0.0 if q >= 1.0 else 0.5 * w * (1.0 - q ** n) ** (1.0 / n)

    def oml_z_top(self, x: float, y: float) -> float:
        w, h, zc, nt, nb = (float(a) for a in self.fus.section(x))
        tf = float(self.fus.top_frac(x))
        q = abs(2 * y / w)
        return zc if q >= 1 else zc + tf * h * (1.0 - q ** nt) ** (1.0 / nt)


# =====================================================================================================================
# engine (local engine frame, x = u, y = v, z = w)
# =====================================================================================================================
THROTTLE = dict(r=0.021, w0=-0.124, w1=-0.164, shaft_w=-0.144, shaft_r=0.003)
PLENUM_IN = dict(r=0.024, w0=-0.126, w1=-0.104)      # intake plenum block on the throttle body
BRANCH_R = 0.014
FILTER = dict(r=0.045, w_top=-0.168, w_c=-0.188)       # hemispherical mesh filter (bottom -0.233 > KO -0.2346)
DA22 = dict(L=0.0415, H=0.0659, W=0.022, mass=0.132, shaft_from_top=0.018)   # components.yaml volz_da22_28v (case
#                                                                              L x H x W, mass); shaft station est.
SERVO_U0 = CYL_U + THROTTLE["r"] + 0.004              # servo case aft face (coaxial drive of the throttle shaft)
SERVO_BRKT = dict(t=0.0015, u0=0.268, u1=0.292, hv=0.0237, w_top=-0.072, w_bot=-0.152)
RAIL = dict(u=CYL_U + 0.030, w=-0.096, r=0.004, v=0.075)   # fuel rail (u, w), radius, half length
REG = dict(r=0.011, len=0.030)                        # pressure regulator at the starboard rail end


def _fins(u_c, side: int) -> list[G.Mesh]:
    out = []
    poly = rrect(u_c - FIN["hu"], u_c + FIN["hu"], -FIN["hw"], FIN["hw"], FIN["rc"])
    for k in range(FIN["n"]):
        v0 = FIN["v0"] + k * FIN["pitch"]
        a0, a1 = (v0, v0 + FIN["t"]) if side > 0 else (-v0 - FIN["t"], -v0)
        out.append(prism_axis(poly, a0, a1, 1))
    return out


def engine_cylinder(C: Ctx, side: int) -> G.Mesh:
    """Cylinder + head of one side (local frame): base flange, barrel core, cooling fins, head with the bolted heat
    sink fins, spark plug + cap, exhaust port boss (flange face at EXH_BOSS w_face) and the intake port branch."""
    s = float(side)
    u = CYL_U
    ms = []
    v0, v1 = CYL_V
    fl = rrect(u - 0.040, u + 0.040, -0.040, 0.040, 0.008)
    ms.append(prism_axis(fl, *(sorted([s * (v0 - 0.002), s * (v0 + 0.007)])), 1))
    ms.append(G.cylinder(CYL_R, (u, s * (v0 + 0.004), 0.0), (u, s * (v1 + OV), 0.0), n=48))
    ms += _fins(u, side)
    hv0, hv1 = HEAD_V
    head = rrect(u - HEAD_HU, u + HEAD_HU, -HEAD_HW, HEAD_HW, 0.012)
    ms.append(prism_axis(head, *(sorted([s * (hv0 - OV), s * hv1])), 1))
    for du in (-0.043, -0.031, -0.019, 0.019, 0.031, 0.043):          # bolted heat-sink fins (plates normal to u)
        ms.append(box3((u + du - 0.00075, min(s * (hv1 - 0.002), s * 0.192), -0.045),
                       (u + du + 0.00075, max(s * (hv1 - 0.002), s * 0.192), 0.045)))
    p0, p1, pr = PLUG
    ms.append(G.cylinder(0.0105, (u, s * (p0 - OV), 0.0), (u, s * (p0 + 0.006), 0.0), n=6))
    ms.append(G.cylinder(pr, (u, s * (p0 + 0.0055), 0.0), (u, s * p1, 0.0), n=24))
    b = EXH_BOSS
    ms.append(box3((u - b["hu"], min(s * b["v0"], s * b["v1"]), b["w_face"]),
                   (u + b["hu"], max(s * b["v0"], s * b["v1"]), -0.030)))
    # intake branch from the plenum block to the barrel underside (piston-port inlet)
    path = arc_path((u, s * 0.010, PLENUM_IN["w1"] - 0.004), (0, s, 0.35), (u, s * INTAKE_V, -0.030), (0, 0, 1), n=14)
    ms.append(G.sweep_circle(path, BRANCH_R, n=20))
    # injector boss on the branch (forward side) toward the rail
    q = path[8]
    ms.append(G.cylinder(0.0055, q + np.array([0.004, 0, 0]), (RAIL["u"], s * float(abs(q[1])), RAIL["w"]), n=16))
    m = union(ms)
    port = G.cylinder(0.0095, (u, s * EXH_PORT_V, b["w_face"] - 0.001), (u, s * EXH_PORT_V, -0.040), n=32)
    return diff(m, [port])


def engine_mesh(C: Ctx) -> G.Mesh:
    """L 275 EF envelope model (local frame), static part (the output spool is :func:`spool_mesh`)."""
    uf, um, uc = C.u_flange, C.u_mount, C.u_coupling
    ms = []
    # front bearing nose (bore round the rotating spool stub, 0.5 mm radial / 1.5 mm axial gap)
    nose = [(0.0, uf + SPOOL_T + 0.0215), (SPOOL_HUB_R + GAP, uf + SPOOL_T + 0.0215),
            (SPOOL_HUB_R + GAP, uf + SPOOL_T + GAP), (NOSE_R - 0.002, uf + SPOOL_T + GAP),
            (NOSE_R, uf + SPOOL_T + GAP + 0.002), (NOSE_R, 0.182), (0.0, 0.182)]
    ms.append(G.revolve(nose, n=48, axis_origin=(0, 0, 0), axis=(1, 0, 0), ref=(0, 0, 1)))
    # crankcase (rear cover 2 mm short of the mount face: only the bosses seat on the isolators)
    sec = rrect(-CASE_V, CASE_V, CASE_W[0], CASE_W[1], CASE_RC)
    ms.append(prism_axis(sec, 0.176, um - 0.002, 0))
    for v, w in C.bolts_vw:
        ms.append(G.cylinder(BOSS_R, (0.286, v, w), (um, v, w), n=48))
    # rear bearing / generator coupling housing
    ms.append(G.cylinder(REAR_R, (um - 0.006, 0, 0), (uc, 0, 0), n=48))
    # baffle mounting bosses on the crankcase top (M5 tapped), servo bracket bosses underneath
    for u in BAFFLE_BOSS_U:
        ms.append(G.cylinder(0.008, (u, 0.0, CASE_W[1] - 0.004), (u, 0.0, BAFFLE_BOSS_W), n=24))
    for v in (-0.012, 0.012):
        ms.append(G.cylinder(0.0065, ((SERVO_BRKT["u0"] + SERVO_BRKT["u1"]) / 2, v, CASE_W[0] + 0.004),
                             ((SERVO_BRKT["u0"] + SERVO_BRKT["u1"]) / 2, v, CASE_W[0] - 0.0001), n=20))
    for side in (1, -1):
        ms.append(engine_cylinder(C, side))
    # intake: plenum block, single throttle body (butterfly shaft along u, coaxial servo), hemispherical mesh filter
    u = CYL_U
    ms.append(G.cylinder(PLENUM_IN["r"], (u, 0, PLENUM_IN["w0"]), (u, 0, PLENUM_IN["w1"]), n=36))
    ms.append(G.cylinder(THROTTLE["r"], (u, 0, THROTTLE["w0"] + OV), (u, 0, THROTTLE["w1"]), n=36))
    ms.append(G.cylinder(THROTTLE["shaft_r"], (u - THROTTLE["r"] - 0.004, 0, THROTTLE["shaft_w"]),
                         (SERVO_U0, 0, THROTTLE["shaft_w"]), n=12))
    f = FILTER
    ms.append(G.cylinder(f["r"] * 0.55, (u, 0, THROTTLE["w1"] + OV), (u, 0, f["w_top"] - OV), n=36))
    ms.append(G.cylinder(f["r"], (u, 0, f["w_top"]), (u, 0, f["w_c"]), n=40))
    hemi = G.sphere(f["r"], (u, 0, f["w_c"]), n=40)
    ms.append(inter(hemi, box3((u - 0.1, -0.1, f["w_c"] - 0.1), (u + 0.1, 0.1, f["w_c"] + OV))))
    # fuel rail with the two injector bosses, inlet fitting (port end), pressure regulator (starboard end)
    ms.append(G.cylinder(RAIL["r"], (RAIL["u"], -RAIL["v"], RAIL["w"]), (RAIL["u"], RAIL["v"], RAIL["w"]), n=16))
    ms.append(G.cylinder(REG["r"], (RAIL["u"], RAIL["v"] - 0.002, RAIL["w"]),
                         (RAIL["u"], RAIL["v"] + REG["len"], RAIL["w"]), n=24))
    ms.append(G.cylinder(0.006, (RAIL["u"], -RAIL["v"] + 0.002, RAIL["w"]), (RAIL["u"], -RAIL["v"] - 0.016, RAIL["w"]),
                         n=6))
    m = union(ms)
    cut = [G.cylinder(COUPLING_RECESS_R, (uc - 0.003, 0, 0), (uc + 0.001, 0, 0), n=24)]
    return finish(C.to_air(diff(m, cut)))


BAFFLE_BOSS_U = (0.205, 0.285)      # crankcase-top plenum mounting bosses (M5 tapped), clear of the cylinder flanges
BAFFLE_BOSS_W = CASE_W[1] + 0.012   # boss top (plenum floor seat)


def spool_mesh(C: Ctx) -> G.Mesh:
    """Propeller output spool (crank-nose flange, part of the engine; rotates with ``prop_spin``)."""
    uf = C.u_flange
    prof = [(SPOOL_RECESS_R, uf), (SPOOL_R, uf), (SPOOL_R, uf + SPOOL_T), (SPOOL_HUB_R, uf + SPOOL_T),
            (SPOOL_HUB_R, uf + SPOOL_T + 0.020), (0.0, uf + SPOOL_T + 0.020), (0.0, uf + 0.006),
            (SPOOL_RECESS_R, uf + 0.006)]
    return finish(C.to_air(G.revolve(prof, n=48, axis_origin=(0, 0, 0), axis=(1, 0, 0), ref=(0, 0, 1))))


def sg750_mesh(C: Ctx) -> G.Mesh:
    """ePropelled SG750 starter-generator envelope (101 x 28.6 mm) with a cable boss, ahead of the adapter."""
    u0 = C.u_coupling + ADAPTER_T
    u1 = u0 + SG750_T
    body = G.revolve([(0.0, u0), (SG750_R - 0.003, u0), (SG750_R, u0 + 0.003), (SG750_R, u1 - 0.003),
                      (SG750_R - 0.003, u1), (0.0, u1)], n=64, axis_origin=(0, 0, 0), axis=(1, 0, 0), ref=(0, 0, 1))
    boss = box3((u0 + 0.006, -0.012, -SG750_R - 0.008), (u1 - 0.006, 0.012, -SG750_R + 0.004))
    return finish(C.to_air(union([body, boss])))


def adapter_mesh(C: Ctx) -> G.Mesh:
    """Machined 7075 SG750 adapter on the coupling flange; counterbores for the 4 x M4 housing screws."""
    u0 = C.u_coupling
    m = G.cylinder(ADAPTER_R, (u0, 0, 0), (u0 + ADAPTER_T, 0, 0), n=64)
    cut = [G.cylinder(COUPLING_RECESS_R + 0.002, (u0 - 0.001, 0, 0), (u0 + ADAPTER_T + 0.001, 0, 0), n=24)]
    for k in range(4):
        a = math.radians(45 + 90 * k)
        cut.append(G.cylinder(0.0042, (u0 + ADAPTER_T - 0.0045, REAR_BOLT_R * math.cos(a), REAR_BOLT_R * math.sin(a)),
                              (u0 + ADAPTER_T + 0.001, REAR_BOLT_R * math.cos(a), REAR_BOLT_R * math.sin(a)), n=24))
    return finish(C.to_air(diff(m, cut)))


def isolator_mesh(C: Ctx, k: int) -> G.Mesh:
    """Conical elastomer isolator (envelope d 40 x 25 per layout, estimate): base ring in the chassis cup, elastomer
    cone, steel top plate on the crankcase boss, inner sleeve. Height = mount face to the cup-bottom face."""
    v, w = C.bolts_vw[k]
    h = ISO_H
    um = C.u_mount
    prof = [(0.0046, um), (ISO_R_TOP, um), (ISO_R_TOP, um + 0.002), (ISO_R_TOP - 0.001, um + 0.0025),
            (ISO_R_BASE - 0.001, um + h - 0.004), (ISO_R_BASE, um + h - 0.0035), (ISO_R_BASE, um + h),
            (0.0046, um + h)]
    m = G.revolve(prof, n=48, axis_origin=(0, v, w), axis=(1, 0, 0), ref=(0, 0, 1))
    return finish(C.to_air(m))


ISO_H = 0.0245              # isolator installed height: layout envelope 25 mm (centre = mount face + 12.5 mm) less a
#                             0.5 mm seating allowance, so the base seats on the cup floor without volume overlap
