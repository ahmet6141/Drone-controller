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
* cooling: CFRP S-duct from the dorsal inlet to the firewall (``KO-COOLING-DUCT`` corridor, open-roof mouth under the
  inlet lip over the first corridor segment, exit section through ``C-DUCT``), stainless firewall transition duct (fireproof spigot through C-DUCT, rising over the mount truss), silicone
  -glass coupling boot, engine-mounted 5052 plenum with the cylinder baffle skirts (air down through the fin packs),
  stainless node heat baffles (``layout.heat_protection.hardware[F-SPINDLE-NODE]``);
* engine accessories: throttle actuator Volz DA 22 on a 5052 bracket under the crankcase (direct drive of the throttle
  cross shaft), engine ECU on the mission tray, generator power electronics on the port side-bay tray (layout boxes);
* heat protection (``layout.heat_protection``): HS-COWL-EXIT stainless exit insert (PR-520-R/L, external riveted lap over
  the cowl cut-out, tail-pipe slot) and HS-COWL-SHIELD stainless shield (PR-523-R/L, hung from the insert with a 5 mm
  air gap under the cowl) - registered only when the lower cowl halves (YK250-SH-451-R/L) are in the registry;
  HS-STUBROOT / HS-STUB (PR-521 / PR-522) are not needed (the stub-root strip and the stub keep >= 50 mm from the
  exhaust).

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
SPOOL_R, SPOOL_T = 0.0365, 0.011    # output spool flange OD 73 (engine.yaml prop_flange_outer_diameter_estimate 72 +-5)
SPOOL_HUB_R = 0.016         # spool stub in the crank-nose bore
SPOOL_FIT = 0.0001          # radial running fit of the spool stub in the nose bore (bearing seat: the stub bears)
SPOOL_RECESS_R = 0.011      # crank-nose bolt recess
NOSE_R = 0.032              # front bearing housing (twin 20 mm ball bearings, engine.yaml)
CASE_V = 0.058              # crankcase half width to the cylinder base faces
CASE_W = (-0.072, 0.046)    # crankcase lower / upper face (w)
CASE_RC = 0.020             # crankcase corner radius
CASE_WALL = 0.006           # crankcase wall (casting)
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
EXH_BOSS = dict(hu=0.034, v0=0.151, v1=0.175, w_face=-0.062)
EXH_SPIGOT_R, EXH_EAR_R = 0.0145, 0.0122   # port spigot round the 19 mm port, bolt ears (2 D edge distance)
EXH_BOLT_DU = 0.022         # 2 x M6 port bolts (heads 6 mm clear of the header pipe, 2 D to the port bore)
INTAKE_V = 0.085            # intake manifold station (v), below the crankcase into the KO intake box
THROTTLE_W = (-0.128, -0.168)
FILTER_R, FILTER_W = 0.044, -0.190
CROSS_SHAFT_W = -0.148
# --- SG750 + adapter ---------------------------------------------------------------------------------------------
SG750_R, SG750_T = 0.0505, 0.0286   # engine.yaml installation_items.sg750 (101 mm dia x 28.6 mm)
ADAPTER_T, ADAPTER_R = 0.010, 0.0295   # adapter stays >= 13 mm off the mount ring (dynamic margin 10 mm)
SG_BOLT_R = 0.021           # 4 x M4 through the generator hub into the adapter (between the housing screws)
# --- isolators (layout: conical elastomer d 40 x 25 envelope, estimate) ------------------------------------------
ISO_R_BASE, ISO_R_TOP, ISO_SLEEVE_R, ISO_BORE = 0.020, 0.015, 0.007, 0.0040
ELASTOMER_DENSITY = 1200.0  # natural-rubber class (estimate; spec.materials has no elastomer)
# --- propeller group ----------------------------------------------------------------------------------------------
SPACER = dict(r_fl_f=0.0365, t_fl_f=0.008, r_o=0.035, r_i=0.0325, r_fl_a=0.044, t_fl_a=0.010, bore=0.011,
              pcd_f=0.024, pcd_a=0.031, access_r=0.003)
BACKPLATE_T = 0.002
CRUSH_T, CRUSH_R = 0.005, 0.044
STANDOFF_RO, STANDOFF_RI, STANDOFF_SOLID = 0.0105, 0.0060, 0.012   # end boss OD 21 (M5 2 D), bore 12
STANDOFF_TUBE = 0.0080      # stand-off tube OD 16 x 2 mm
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
CAN = dict(r=0.019, x0=3.803, x1=3.897, y=0.190, z=0.100, end_r=0.006)   # silencer can (axis along x): inside
#                             KO-EXHAUST-R box 1, inboard edge clear of the port-bolt key path, >= 10 mm + skin to the OML
# --- cooling ------------------------------------------------------------------------------------------------------
DUCT_T = 0.0008             # S-duct CFRP 4 plies PW
SPIGOT_T = 0.0005           # stainless transition duct
BAFFLE_T = 0.0008           # 5052-H32 plenum / baffle sheet
COWL_SKIN = 0.0058          # shell_secondary laminate (layups) - the plenum roof stays >= 10 mm inside it
DYN = 0.010                 # engine dynamic margin (layout.clearance_values.engine_keep_out)
FINE = {3: 0.0032, 4: 0.0043, 5: 0.0053, 6: 0.0064, 8: 0.0084}   # ISO 273 fine-series holes (multi-part stacks)

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


def finish_clean(m: G.Mesh) -> G.Mesh:
    """``finish`` for thin formed sheets cut by many booleans: merge the sub-10-um sliver edges the cuts leave where a
    trim plane grazes the curved sheet (manifold simplify at growing tolerance until the mesh check incl. the
    triangle self-intersection test passes)."""
    man = m.to_manifold()
    for tol in (0.0, 1e-7, 1e-6, 4e-6, 1e-5):
        out = G.Mesh.from_manifold(man.simplify(tol) if tol else man)
        try:
            ok = out.check(self_intersect=True)["ok"]
        except ImportError:                       # no triangle-test backend: closedness only
            ok = out.check()["ok"]
        if ok:
            return out
    raise ValueError("propulsion: sheet mesh not clean after simplify")


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

    def body_env(self, inset: float, x0: float, x1: float, dx: float = 0.01, n: int = 192) -> G.Mesh:
        """Closed solid of the fuselage OML inset by ``inset`` between x0 and x1 (loft of inset sections)."""
        from .structgen import fuselage_section2d, resample_ring
        k = max(2, int(math.ceil((x1 - x0) / dx)) + 1)
        rings = []
        for x in np.linspace(x0, x1, k):
            P2 = resample_ring(fuselage_section2d(self.fus, float(x), inset, n=256), n, start_dir=(0.0, 1.0))
            rings.append(np.column_stack([np.full(len(P2), x), P2[:, 0], P2[:, 1]]))
        return G.loft(rings)

    def oml_z_top(self, x: float, y: float) -> float:
        w, h, zc, nt, nb = (float(a) for a in self.fus.section(x))
        tf = float(self.fus.top_frac(x))
        q = abs(2 * y / w)
        return zc if q >= 1 else zc + tf * h * (1.0 - q ** nt) ** (1.0 / nt)


# =====================================================================================================================
# engine (local engine frame, x = u, y = v, z = w)
# =====================================================================================================================
THROTTLE = dict(r=0.021, w0=-0.124, w1=-0.164, shaft_w=-0.144, shaft_r=0.003)
PLENUM_IN = dict(r=0.022, w0=-0.126, w1=-0.104)      # intake plenum block on the throttle body
BRANCH_R = 0.014
FILTER = dict(r=0.045, w_top=-0.168, w_c=-0.188)       # hemispherical mesh filter (bottom -0.233 > KO -0.2346)
DA22 = dict(L=0.0415, H=0.0659, W=0.022, mass=0.132, shaft_from_bottom=0.018)   # components.yaml volz_da22_28v
#                                                                    (case L x H x W, mass); shaft station: estimate
SERVO_U0 = CYL_U + THROTTLE["r"] + 0.004              # servo case aft face (coaxial drive of the throttle shaft)
SERVO_BRKT = dict(t=0.0015, u0=0.270, u1=0.294, w_top=-0.0745, w_bot=-0.150, leg_v=DA22["L"] / 2 + 0.0001)
SERVO_BOLT_U, SERVO_BOLT_V = 0.282, (-0.0120, 0.0120)   # 2 x M5 bracket bolts into the crankcase-bottom bosses
RAIL = dict(u=0.301, w=-0.096, r=0.004, v=0.075)   # fuel rail (u, w), radius, half length (ahead of the servo)
BOSS_M5_R = 0.0102          # M5 tapped bosses (2 D edge distance)
NIPPLE_V, NIPPLE_U = 0.090, 0.326   # hose nipples (rail inlet port end, regulator outlet starboard) pointing +u
HEAD_BOSS_W = 0.035         # plenum stand-off boss on each head (above the spark plug)
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
    b = EXH_BOSS                                      # exhaust port spigot + two bolt ears (flange face b["w_face"])
    ms.append(G.cylinder(EXH_SPIGOT_R, (u, s * EXH_PORT_V, b["w_face"]), (u, s * EXH_PORT_V, -0.030), n=40))
    for du in (-EXH_BOLT_DU, EXH_BOLT_DU):
        ms.append(G.cylinder(EXH_EAR_R, (u + du, s * EXH_PORT_V, b["w_face"]), (u + du, s * EXH_PORT_V, -0.044), n=48))
        ms.append(box3((u + min(du, 0.0), s * EXH_PORT_V - 0.004, b["w_face"]),       # web rib ear - spigot
                       (u + max(du, 0.0), s * EXH_PORT_V + 0.004, -0.050)))
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
    nose = [(0.0, uf + SPOOL_T + 0.0215), (SPOOL_HUB_R + SPOOL_FIT, uf + SPOOL_T + 0.0215),
            (SPOOL_HUB_R + SPOOL_FIT, uf + SPOOL_T + GAP), (NOSE_R - 0.002, uf + SPOOL_T + GAP),
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
        ms.append(G.cylinder(BOSS_M5_R, (u, 0.0, CASE_W[1] - 0.004), (u, 0.0, BAFFLE_BOSS_W), n=32))
    for sv in (1.0, -1.0):                              # plenum stand-off bosses on the heads (between heat-sink fins)
        ms.append(G.cylinder(BOSS_M5_R, (CYL_U, sv * (HEAD_V[1] - 0.002), HEAD_BOSS_W),
                             (CYL_U, sv * PLEN["v_out"], HEAD_BOSS_W), n=32))
    for v in SERVO_BOLT_V:
        ms.append(G.cylinder(BOSS_M5_R, (SERVO_BOLT_U, v, CASE_W[0] + 0.004), (SERVO_BOLT_U, v, SERVO_BRKT["w_top"]),
                             n=32))
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
    ms.append(G.cylinder(0.006, (RAIL["u"], -RAIL["v"] + 0.002, RAIL["w"]), (RAIL["u"], -NIPPLE_V, RAIL["w"]), n=6))
    for sv in (-1.0, 1.0):                             # inlet elbow (port end) / regulator outlet (starboard): +u
        u0 = RAIL["u"] - (0.0 if sv < 0 else -REG["r"] + 0.001)
        ms.append(G.cylinder(0.0042, (RAIL["u"] - 0.004, sv * NIPPLE_V, RAIL["w"]), (NIPPLE_U, sv * NIPPLE_V, RAIL["w"]),
                             n=16))
    m = union(ms)
    cut = [G.cylinder(COUPLING_RECESS_R, (uc - 0.003, 0, 0), (uc + 0.001, 0, 0), n=24)]
    # crank chamber (6 mm walls), rear bearing-housing annulus and the cylinder bores (66 mm) open into it; the mount
    # bosses stand inside the chamber (12 mm threads end in the cavity)
    cav = rrect(-CASE_V + CASE_WALL, CASE_V - CASE_WALL, CASE_W[0] + CASE_WALL, CASE_W[1] - CASE_WALL,
                CASE_RC - CASE_WALL)
    chamber = prism_axis(cav, 0.184, um - 0.002 - CASE_WALL, 0)
    bosses = [G.cylinder(BOSS_R, (0.280, v, w), (um + 0.001, v, w), n=48) for v, w in C.bolts_vw]
    cut.append(diff(chamber, bosses))
    cut.append(G.tube(REAR_R - 0.006, COUPLING_RECESS_R + 0.002, (um - CASE_WALL - 0.004, 0, 0), (uc - 0.0115, 0, 0),
                      n=48))
    for side in (1, -1):
        cut.append(G.cylinder(0.033, (CYL_U, side * (CASE_V - CASE_WALL - 0.002), 0.0), (CYL_U, side * HEAD_V[0], 0.0),
                              n=48))
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
    """ePropelled SG750 starter-generator envelope (101 x 28.6 mm) with a cable boss (starboard, clear of the lower
    mount struts), ahead of the adapter."""
    u0 = C.u_coupling + ADAPTER_T
    u1 = u0 + SG750_T
    body = G.revolve([(0.0, u0), (SG750_R - 0.003, u0), (SG750_R, u0 + 0.003), (SG750_R, u1 - 0.003),
                      (SG750_R - 0.003, u1), (0.0, u1)], n=64, axis_origin=(0, 0, 0), axis=(1, 0, 0), ref=(0, 0, 1))
    boss = box3((u0 + 0.006, SG750_R - 0.004, -0.012), (u1 - 0.006, SG750_R + 0.008, 0.012))   # cable boss (+v)
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
    prof = [(ISO_BORE, um), (ISO_R_TOP, um), (ISO_R_TOP, um + 0.002), (ISO_R_TOP - 0.001, um + 0.0025),
            (ISO_R_BASE - 0.001, um + h - 0.004), (ISO_R_BASE, um + h - 0.0035), (ISO_R_BASE, um + h),
            (ISO_BORE, um + h)]
    m = G.revolve(prof, n=48, axis_origin=(0, v, w), axis=(1, 0, 0), ref=(0, 0, 1))
    return finish(C.to_air(m))


ISO_H = 0.0245              # isolator installed height: layout envelope 25 mm (centre = mount face + 12.5 mm) less a
#                             0.5 mm seating allowance, so the base seats on the cup floor without volume overlap


# =====================================================================================================================
# propeller group (rotates with prop_spin)
# =====================================================================================================================
SPINNER_BASE_U = 0.020          # spinner base / backplate aft face = propeller hub forward face (hub half thickness)
SHANK_C, SHANK_R1, BLEND_R1 = 0.044, 0.074, 0.098   # root shank chord inside the spinner, blend to the planform


def _bolt_angles(n: int, offset_deg: float) -> list[float]:
    return [math.radians(offset_deg + 360.0 * k / n) for k in range(n)]


def spacer_mesh(C: Ctx) -> G.Mesh:
    """110 mm hub spacer, machined 7075-T651: forward flange on the output spool (6 x M6 from inside the bore, heads
    reached with a hex key through the access holes of the aft flange), tube, aft flange tapped for the propeller
    bolts, centring spigot for the backplate and the propeller hub bore."""
    S_ = SPACER
    uf = C.u_flange
    ua0 = SPINNER_BASE_U + BACKPLATE_T                 # aft flange aft face (backplate between it and the hub)
    ua1 = ua0 + S_["t_fl_a"]
    uf0 = uf - S_["t_fl_f"]
    prof = [(S_["bore"], uf), (S_["r_fl_f"], uf), (S_["r_fl_f"], uf0), (S_["r_o"], uf0), (S_["r_o"], ua1),
            (S_["r_fl_a"], ua1), (S_["r_fl_a"], ua0), (HUB_BORE - 0.0002, ua0), (HUB_BORE - 0.0002, ua0 - 0.014),
            (0.0, ua0 - 0.014), (0.0, ua1 - OV), (S_["r_i"], ua1 - OV), (S_["r_i"], uf0), (S_["bore"], uf0)]
    m = G.revolve(prof, n=64, axis_origin=(0, 0, 0), axis=(1, 0, 0), ref=(0, 0, 1))
    cut = []
    for a in _bolt_angles(6, 30.0):                    # hex-key access holes in line with the forward-flange bolts
        y, z = S_["pcd_f"] * math.cos(a), S_["pcd_f"] * math.sin(a)
        cut.append(G.cylinder(S_["access_r"], (ua0 - 0.001, y, z), (ua1 + 0.001, y, z), n=20))
    return finish(C.to_air(diff(m, cut)))


def backplate_mesh(C: Ctx) -> G.Mesh:
    """Spinner backplate, 7075 disc 2 mm clamped between the spacer aft flange and the propeller hub."""
    u0 = SPINNER_BASE_U
    m = G.revolve([(HUB_BORE, u0), (SPINNER_R, u0), (SPINNER_R, u0 + BACKPLATE_T), (HUB_BORE, u0 + BACKPLATE_T)],
                  n=96, axis_origin=(0, 0, 0), axis=(1, 0, 0), ref=(0, 0, 1))
    return finish(C.to_air(m))


def crush_mesh(C: Ctx) -> G.Mesh:
    """7075 crush plate behind the propeller hub with the integral spinner stand-off (tapped M5 end)."""
    u1 = -C.S["propeller"]["hub_half_thickness"]
    u0 = u1 - CRUSH_T
    tip_in = SPINNER_BASE_U - SPINNER_L + SPINNER_TIP_T
    e0 = tip_in + STANDOFF_SOLID + 0.004               # end boss (M5 edge distance) -> slim tube
    prof = [(STANDOFF_RI, u1), (CRUSH_R, u1), (CRUSH_R, u0), (STANDOFF_TUBE, u0), (STANDOFF_TUBE, e0 + 0.003),
            (STANDOFF_RO, e0), (STANDOFF_RO, tip_in), (0.0, tip_in), (0.0, tip_in + STANDOFF_SOLID),
            (STANDOFF_RI, tip_in + STANDOFF_SOLID)]
    return finish(C.to_air(G.revolve(prof, n=48, axis_origin=(0, 0, 0), axis=(1, 0, 0), ref=(0, 0, 1))))


SPINNER_R = None   # set from spec.propeller.spinner in Ctx-dependent helpers (see _spinner_dims)
SPINNER_L = None


def _spinner_dims(spec: dict) -> None:
    global SPINNER_R, SPINNER_L
    sp = spec["propeller"]["spinner"]
    SPINNER_R = 0.5 * float(sp["diameter"])
    SPINNER_L = float(sp["length"])


def blade_rings(C: Ctx, k: int, rs, grow: float = 0.0, n: int = 40) -> list[np.ndarray]:
    """World section rings of blade ``k`` (azimuth 90 + 120 k deg from +v toward +w) at radii ``rs``; ``grow`` offsets
    the section outline (spinner slot clearance). Planform: spec.propeller.blade_model (chord, pitch angle, t/c)
    from r_root outboard, a narrower root shank inside the spinner (axial extent inside the hub / backplate gap)."""
    from shapely.geometry.polygon import orient
    from .structgen import resample_ring
    bm = C.S["propeller"]["blade_model"]
    R = 0.5 * float(C.S["propeller"]["diameter"])
    r0, c0, c1 = float(bm["r_root"]), float(bm["chord_root"]), float(bm["chord_tip"])
    b0, b1, tc = float(bm["beta_root_deg"]), float(bm["beta_tip_deg"]), float(bm["thickness_ratio"])
    th = math.radians(90.0 + 120.0 * k)
    er = math.cos(th) * C.ev + math.sin(th) * C.ew
    et = np.cross(C.d, er)                               # positive prop_spin rotation moves the blade along +et
    rings = []
    for r in rs:
        cs = c0 + (c1 - c0) * (r - r0) / (R - r0)
        if r <= SHANK_R1:
            c = SHANK_C
        elif r < BLEND_R1:
            t = (r - SHANK_R1) / (BLEND_R1 - SHANK_R1)
            c = SHANK_C + (c0 + (c1 - c0) * (BLEND_R1 - r0) / (R - r0) - SHANK_C) * (3 * t * t - 2 * t ** 3)
        else:
            c = cs
        beta = math.radians(b0 + (b1 - b0) * (r - r0) / (R - r0))
        scale = 1.0 + 0.6 * max(0.0, (0.12 - r) / (0.12 - 0.05))   # thicker root (t/c 16 % at the hub)
        x, yu, yl = O.resampled(BLADE_AIRFOIL, 40, 0.012, scale * tc / 0.10)
        P2 = np.vstack([np.column_stack([x, yu])[::-1], np.column_stack([x, yl])[1:-1]])
        P2[:, 0] -= 0.5                                  # stacked on the half-chord point
        poly = Polygon(P2 * c).buffer(0)
        if grow > 0:
            poly = poly.buffer(grow, 8)
        ring2 = resample_ring(orient(poly, 1.0), n, start_dir=(1.0, 0.0))
        chord = -math.cos(beta) * et + math.sin(beta) * C.d          # LE -> TE
        nup = -(math.sin(beta) * et + math.cos(beta) * C.d)          # suction side faces forward
        o = C.H + r * er
        rings.append(o + ring2[:, :1] * chord + ring2[:, 1:2] * nup)
    return rings


def prop_mesh(C: Ctx) -> G.Mesh:
    """Mejzlik 31x12 3B envelope: hub disc (centre bore on the spacer spigot) + three lofted blades."""
    h = float(C.S["propeller"]["hub_half_thickness"])
    R = 0.5 * float(C.S["propeller"]["diameter"])
    hub = C.to_air(G.revolve([(HUB_BORE, -h), (HUB_R, -h), (HUB_R, h), (HUB_BORE, h)], n=96, axis_origin=(0, 0, 0),
                             axis=(1, 0, 0), ref=(0, 0, 1)))
    rs = np.r_[np.linspace(HUB_R - 0.0015, SHANK_R1, 4)[:-1], np.linspace(SHANK_R1, BLEND_R1, 6)[:-1],
               np.linspace(BLEND_R1, R - 0.03, 14), R - 0.015, R - 0.006, R]
    blades = []
    for k in range(3):
        rings = blade_rings(C, k, rs)
        tip = rings[-1]                                  # rounded tip: shrink the last ring about its centroid
        c = tip.mean(0)
        rings[-1] = c + 0.55 * (tip - c)
        blades.append(G.loft(rings))
    return finish(union([hub] + blades))


def spinner_mesh(C: Ctx) -> G.Mesh:
    """CFRP spinner cone (0.8 mm, 4 mm tip boss for the centre screw), base face on the backplate, blade slots."""
    u0 = SPINNER_BASE_U
    L = SPINNER_L
    ss = np.linspace(0.0, 1.0, 40)
    r_out = SPINNER_TIP_R + (SPINNER_R - SPINNER_TIP_R) * np.sqrt(np.clip(1.0 - ss ** 2, 0.0, 1.0))
    us = u0 - ss * L
    outer = Polygon(list(zip(r_out, us)) + list(zip(-r_out[::-1], us[::-1])))
    inner = outer.buffer(-SPINNER_T, join_style=1)
    inner = inner.intersection(sbox(-1, u0 - L + SPINNER_TIP_T, 1, u0 - 0.006)).union(
        sbox(-(SPINNER_R - SPINNER_T), u0 - 0.012, SPINNER_R - SPINNER_T, u0 + 0.001))
    shell = outer.difference(inner).intersection(sbox(0.0, -1.0, 1.0, 1.0))
    from shapely.geometry.polygon import orient
    shell = orient(shell, 1.0)
    P = np.asarray(shell.exterior.coords)[:-1]
    m = C.to_air(G.revolve(P, n=96, axis_origin=(0, 0, 0), axis=(1, 0, 0), ref=(0, 0, 1)))
    rs = np.linspace(0.055, 0.095, 5)
    slots = [G.loft(blade_rings(C, k, rs, grow=BLADE_CLEAR, n=48)) for k in range(3)]
    return finish(largest_piece(diff(m, slots)))


# =====================================================================================================================
# exhaust (world frame, starboard; the port side is the mirror image)
# =====================================================================================================================
def exhaust_paths(C: Ctx) -> dict:
    """Centre lines of the neck (port flange -> can) and the tail pipe (can -> layout exit point / direction)."""
    ko = C.ko["KO-EXHAUST-R"]
    E = np.asarray(ko["exit"]["point"], float)
    e = G.unit(ko["exit"]["direction"])
    b = EXH_BOSS
    f0 = C.P(CYL_U, EXH_PORT_V, b["w_face"] - EXH_FLANGE_T)          # flange lower face, port centre
    n_end = np.array([3.842, CAN["y"] - 0.002, CAN["z"] + 0.004])
    neck = arc_path(f0 + 0.002 * C.ew, -C.ew, n_end, (0.0, 0.55, -0.83), n=10)
    # tail pipe: down from the can bottom inside routing box 1, 90 deg bend (R 2 D) into box 2 below its top face,
    # aft, last 90 deg bend (R 1.5 D) onto the layout exit direction ending at the exit point
    xa = TAIL_X
    zs = TAIL_Z_TURN
    R1, R2 = 2 * TAIL_R, 1.5 * TAIL_R
    xh = np.array([1.0, 0.0, 0.0])
    a2 = E - R2 * (e + xh)                               # start of the last bend (tangent +x)
    pts = [np.array([xa, CAN["y"], CAN["z"] + 0.004]), np.array([xa, CAN["y"], zs])]
    th = np.linspace(0.0, 0.5 * math.pi, 9)[1:]
    c1 = np.array([xa + R1, CAN["y"], zs])
    pts += [c1 + R1 * np.array([-math.cos(t), 0.0, -math.sin(t)]) for t in th]
    b_end = pts[-1]
    for f in np.linspace(0.0, 1.0, 6)[1:-1]:             # straight run, y drifting outboard to the bend start
        pts.append(b_end + f * (a2 - b_end))
    c2 = a2 + R2 * e
    pts += [c2 - R2 * math.cos(t) * e + R2 * math.sin(t) * xh for t in np.linspace(0.0, 0.5 * math.pi, 9)]
    tail = np.vstack(pts)
    return {"neck": neck, "tail": tail, "exit": E, "exit_dir": e, "flange_face": f0}


TAIL_X, TAIL_Z_TURN = 3.875, 0.026   # tail-pipe drop station (x) and first-bend start (z) inside KO-EXHAUST box 1


def exhaust_mesh(C: Ctx) -> G.Mesh:
    """Starboard exhaust weldment, stainless 304: port flange 6 mm, neck OD 22, silencer can (OD 38 x 94, 0.8 mm),
    tail pipe OD 20 to the layout exit (outboard / 60 deg down, KO-EXHAUST-R exit)."""
    pa = exhaust_paths(C)
    b = EXH_BOSS
    u = CYL_U
    fl_loc = box3((u - b["hu"], b["v0"], b["w_face"] - EXH_FLANGE_T), (u + b["hu"], b["v1"], b["w_face"]))
    fl_loc = G.Mesh(fl_loc.V, fl_loc.F)
    flange = C.to_air(G.extrude(rrect(u - b["hu"], u + b["hu"], b["v0"], b["v1"], 0.004), EXH_FLANGE_T,
                                origin=(0, 0, b["w_face"] - EXH_FLANGE_T), u=(1, 0, 0), v=(0, 1, 0)))
    neck_o, neck_i = tube_along(pa["neck"], HEADER_R, HEADER_R - EXH_T, n=24, ext=0.004)
    x0, x1, r, er = CAN["x0"], CAN["x1"], CAN["r"], CAN["end_r"]
    ax0 = np.array([0.0, CAN["y"], CAN["z"]])
    prof = [(0.0, x0), (r - er, x0), (r - 0.3 * er, x0 + 0.3 * er), (r, x0 + er), (r, x1 - er),
            (r - 0.3 * er, x1 - 0.3 * er), (r - er, x1), (0.0, x1)]
    can_o = G.revolve(prof, n=48, axis_origin=ax0, axis=(1, 0, 0), ref=(0, 0, 1))
    t = EXH_T
    prof_i = [(0.0, x0 + t), (r - er, x0 + t), (r - 0.3 * er - 0.7 * t, x0 + 0.3 * er + 0.3 * t), (r - t, x0 + er),
              (r - t, x1 - er), (r - 0.3 * er - 0.7 * t, x1 - 0.3 * er - 0.3 * t), (r - er, x1 - t), (0.0, x1 - t)]
    can_i = G.revolve(prof_i, n=48, axis_origin=ax0, axis=(1, 0, 0), ref=(0, 0, 1))
    tail_o, tail_i = tube_along(pa["tail"], TAIL_R, TAIL_R - EXH_T, n=24, ext=0.004)
    # bores stop inside the can (the can interior joins them); the neck bore runs through the flange
    port = C.to_air(G.cylinder(0.0095, (u, EXH_PORT_V, b["w_face"] - EXH_FLANGE_T - 0.002),
                               (u, EXH_PORT_V, b["w_face"] + 0.001), n=32))
    outer = union([flange, neck_o, can_o, tail_o])
    neck_i = diff(neck_i, [C.to_air(box3((u - 0.05, 0.10, b["w_face"] - EXH_FLANGE_T + OV), (u + 0.05, 0.25, 0.05)))])
    m = diff(outer, [neck_i, can_i, tail_i, port])
    return finish(largest_piece(m))


# =====================================================================================================================
# cooling: S-duct, firewall transition duct, coupling boot, engine plenum with cylinder baffles
# =====================================================================================================================
def rr_ring(c, e1, e2, ha: float, hb: float, r: float, n: int = 64) -> np.ndarray:
    """Rounded rectangle (half sizes ha along e1, hb along e2, corner radius r) as n points, CCW about e1 x e2,
    starting on +e1 (consistent ring for lofts)."""
    r = min(r, 0.999 * ha, 0.999 * hb)
    poly = sbox(-ha + r, -hb + r, ha - r, hb - r).buffer(r, 32)
    from .structgen import resample_ring
    from shapely.geometry.polygon import orient
    P = resample_ring(orient(poly, 1.0), n, start_dir=(1.0, 0.0))
    return np.asarray(c, float) + P[:, :1] * np.asarray(e1, float) + P[:, 1:2] * np.asarray(e2, float)


def _sec_ring(x, yc, zc, hy, hz, r, n=64, grow=0.0):
    return rr_ring((x, yc, zc), (0, 1, 0), (0, 0, 1), hy + grow, hz + grow, r + grow, n)


def duct_stations(C: Ctx) -> list[tuple]:
    """S-duct outer sections (x, z_centre, half width, half height, corner radius) along KO-COOLING-DUCT: the
    corridor of three 80 mm tubes (1 mm inside it) from the inlet throat, flattening over the last 55 mm to the
    exit section through C-DUCT."""
    ko = C.ko["KO-COOLING-DUCT"]
    P = np.asarray(ko["path"], float)
    r = float(ko["radius"]) - 0.001
    hy = max(abs(float(o)) for o in ko["lateral_offsets"]) + r
    ex = ko["exit_section"]
    out = [(float(p[0]), float(p[2]), hy, r, r) for p in P]
    zx = 0.5 * (ex["z"][0] + ex["z"][1])
    exs = (zx, 0.5 * (ex["y"][1] - ex["y"][0]), 0.5 * (ex["z"][1] - ex["z"][0]), 0.020)
    out.append((DUCT_FLAT_X,) + exs)                # constant exit section over the slip joint on the spigot
    out.append((float(ex["x"][1]),) + exs)
    return out


def sduct_mesh(C: Ctx) -> G.Mesh:
    """CFRP S-duct (4 plies PW, 0.8 mm): inlet throat (bonded into the inlet lip socket) -> firewall exit section,
    the last 35 mm slip over the stainless spigot (PR-548)."""
    st = duct_stations(C)
    rings_o, rings_i = [], []
    X = np.array([s_[0] for s_ in st])
    xs = np.unique(np.r_[np.linspace(st[0][0], st[-3][0], 21), np.linspace(st[-3][0], st[-2][0], 5), st[-1][0]])
    for x in xs:
        vals = [np.interp(x, X, [s_[k] for s_ in st]) for k in (1, 2, 3, 4)]
        zc, hy, hz, r = vals
        rings_o.append(_sec_ring(x, 0.0, zc, hy, hz, r))
        rings_i.append(_sec_ring(x, 0.0, zc, hy - DUCT_T, hz - DUCT_T, max(r - DUCT_T, 0.002)))
    xi = [xs[0] - 0.002] + list(xs[1:-1]) + [xs[-1] + 0.002]
    rings_i = [ri + np.array([xx - x0, 0, 0]) for ri, xx, x0 in zip(rings_i, xi, xs)]
    # inlet mouth: between the first two corridor stations the roof is open to the dorsal inlet (P-INLET); the side
    # walls rise to INLET_BOND_DEPTH under the OML with 12 mm inward bonding flanges on the lip's joggled land, the
    # roof starts at the throat station with a bulkhead up to the same depth
    x0, x1 = st[0][0], st[1][0]
    t = DUCT_T
    dep = inlet_bond_depth()
    mo, mi = [], []
    for x in np.linspace(x0, x1, 11):
        zc, hy, hz = (float(np.interp(x, X, [s_[k] for s_ in st])) for k in (1, 2, 3))
        zb, zt = zc + MOUTH_ZB * hz, C.oml_z_top(x, 0.0) + 0.02
        mo.append(_sec_ring(x, 0.0, 0.5 * (zb + zt), hy, 0.5 * (zt - zb), 0.002))
    for x in np.linspace(x0 - 0.002, x1 - t, 11):
        xc = min(max(x, x0), x1)
        zc, hy, hz = (float(np.interp(xc, X, [s_[k] for s_ in st])) for k in (1, 2, 3))
        zb, zt = zc + MOUTH_ZB * hz + t, C.oml_z_top(xc, 0.0) + 0.03
        mi.append(_sec_ring(x, 0.0, 0.5 * (zb + zt), hy - t, 0.5 * (zt - zb), 0.002))
    env = C.body_env(dep, x0 - 0.01, x1 + 0.01, dx=0.005)
    mouth_o = inter(G.loft(mo), env)
    duct = diff(union([G.loft(rings_o), mouth_o]), [G.loft(rings_i), G.loft(mi)])
    hy0 = float(st[0][2])
    band = diff(env, [C.body_env(dep + t, x0 - 0.02, x1 + 0.02, dx=0.005)])
    fl = [inter(band, box3((x0, sv * (hy0 - t - MOUTH_FL) if sv > 0 else -(hy0 - 0.5 * t), 0.2),
                           (x1 - 0.5 * t, hy0 - 0.5 * t if sv > 0 else -(hy0 - t - MOUTH_FL), 0.4)))
          for sv in (1.0, -1.0)]
    return finish(largest_piece(union([duct] + fl)))


MOUTH_ZB = 0.3              # mouth side walls start 0.3 half-heights above the duct centre line (crossing, not tangent)
MOUTH_FL = 0.012            # inward bonding flanges of the mouth walls on the inlet lip land


def inlet_bond_depth() -> float:
    """Depth below the OML of the inlet-lip land's inner face + 0.2 mm bond line: P-INLET (shell_secondary, COWL_SKIN)
    sits in a joggled recess of P-AFT-UPPER (layout.shell.rules.joggles: depth = panel t + 0.3 mm) on a 1.6 mm solid
    land (sandwich_edges edge band)."""
    return COWL_SKIN + 0.0003 + 0.0016 + 0.0002


SPIGOT_X0 = 3.621           # transition duct forward end inside the S-duct (32 mm slip joint)
DUCT_FLAT_X = 3.616         # S-duct constant exit section from here to the exit (layout: flattens over the last 55 mm)
EDGE_ANGLE_LEG = 0.013      # firewall shield edge angle leg (layout firewall_stackup: angle 2 x 13 mm)
SDUCT_SCREW_X = 3.636       # 4 x M4 of the slip joint (>= 2.5 D to the duct end, >= 2 D to the spigot end)
TRANS = dict(x_flange=0.0005, fl_out=0.012, fl_top=0.009, x_rise0=3.676, x_rise1=3.703, x_wide=3.712, x_end=3.745,
             z_lo=0.294, z_hi=0.328, z_hi_fw=0.312, r=0.016, hy=0.120)


def spigot_section(C: Ctx):
    """(z_centre, half width, half height, r) of the spigot: the S-duct exit inner section less a 0.1 mm fit."""
    ex = C.ko["KO-COOLING-DUCT"]["exit_section"]
    zc = 0.5 * (ex["z"][0] + ex["z"][1])
    hy = 0.5 * (ex["y"][1] - ex["y"][0]) - DUCT_T - 0.0001
    hz = 0.5 * (ex["z"][1] - ex["z"][0]) - DUCT_T - 0.0001
    return zc, hy, hz, 0.020 - DUCT_T - 0.0001


def transition_sections(C: Ctx) -> list[tuple]:
    """(x, z_lo, z_hi, half width, corner r) of the transition duct: spigot section through C-DUCT, floor rising
    over the engine-mount truss (>= 5 mm above it), widened to the boot section once past the port upper strut."""
    x_fw = float(C.st["FS3670"]["x"])
    zc, hy, hz, r = spigot_section(C)
    T = TRANS
    return [(SPIGOT_X0, zc - hz, zc + hz, hy, r), (x_fw + 0.0015, zc - hz, zc + hz, hy, r),
            (T["x_rise0"], zc - hz, T["z_hi_fw"], hy, r), (T["x_rise1"], T["z_lo"], T["z_hi"], hy, T["r"]),
            (T["x_wide"], T["z_lo"], T["z_hi"], T["hy"], T["r"]), (T["x_end"], T["z_lo"], T["z_hi"], T["hy"], T["r"])]


def transition_mesh(C: Ctx) -> G.Mesh:
    """Stainless 304 (0.5 mm) firewall spigot + transition duct: slip spigot inside the S-duct end, through the
    C-DUCT cut-out (fireproof seal), flange riveted on the shield aft face, rising over the engine-mount truss (5 mm
    above it) to the coupling boot. The inner surface is offset normal to the sloped floor / roof (true 0.5 mm)."""
    x_fw = float(C.st["FS3670"]["x"])
    sec = transition_sections(C)
    T = TRANS
    X = np.array([s_[0] for s_ in sec])
    xs = np.unique(np.r_[np.linspace(SPIGOT_X0, x_fw + 0.0015, 4), np.linspace(x_fw + 0.0015, T["x_end"], 22), X])
    col = lambda k: np.interp(xs, X, [s_[k] for s_ in sec])
    zl, zh, hy, rr = col(1), col(2), col(3), col(4)
    sl_lo, sl_hi = np.gradient(zl, xs), np.gradient(zh, xs)
    rings_o, rings_i = [], []
    t = SPIGOT_T
    for k, x in enumerate(xs):
        rings_o.append(_sec_ring(x, 0.0, 0.5 * (zl[k] + zh[k]), hy[k], 0.5 * (zh[k] - zl[k]), rr[k]))
        zli = zl[k] + t * math.sqrt(1.0 + sl_lo[k] ** 2)
        zhi = zh[k] - t * math.sqrt(1.0 + sl_hi[k] ** 2)
        rings_i.append(_sec_ring(x, 0.0, 0.5 * (zli + zhi), hy[k] - t, 0.5 * (zhi - zli), rr[k] - t))
    rings_i[0] = rings_i[0] - np.array([0.002, 0, 0])
    rings_i[-1] = rings_i[-1] + np.array([0.002, 0, 0])
    tube = diff(G.loft(rings_o), [G.loft(rings_i)])
    zc, hy0, hz0, r = spigot_section(C)
    # flange on the shield aft face (x_fw .. x_fw + 0.5 mm): 15 mm round the cut-out at top / bottom, narrow at the
    # sides (upper engine-mount feet / corner fittings at |y| >= 0.098)
    cut = next(c for c in C.st["FS3670"]["cutouts"] if c["id"] == "C-DUCT")
    y0, y1 = cut["y"]
    z0, z1 = cut["z"]
    fl = sbox(y0 - 0.002, z0 - T["fl_out"], y1 + 0.002, z1 + T["fl_top"]).buffer(0.003, 8).difference(
        rr_poly(hy0 - OV, hz0 - OV, r - OV, zc))
    from .structgen import fuselage_section2d
    st = C.st["FS3670"]
    allowed = fuselage_section2d(C.fus, x_fw, float(st["inset"]) + EDGE_ANGLE_LEG + 0.0035)
    fl = fl.intersection(allowed)
    flange = G.extrude(fl, SPIGOT_T, origin=(x_fw, 0, 0), u=(0, 1, 0), v=(0, 0, 1))
    return finish(union([tube, flange]))


def rr_poly(hy, hz, r, zc) -> Polygon:
    return sbox(-hy + r, zc - hz + r, hy - r, zc + hz - r).buffer(r, 32)


def flange_rivet_points(C: Ctx) -> list[np.ndarray]:
    """Blind rivets d 3.2 of the transition-duct flange on the firewall shield (top and bottom rows)."""
    x_fw = float(C.st["FS3670"]["x"])
    cut = next(c for c in C.st["FS3670"]["cutouts"] if c["id"] == "C-DUCT")
    z0, z1 = cut["z"]
    zr = (z0 - 0.5 * (TRANS["fl_out"] + 0.003), z1 + 0.5 * (TRANS["fl_top"] + 0.003))
    return [np.array([x_fw, y, z]) for z in zr for y in (-0.055, 0.0, 0.055)]


# engine plenum + cylinder baffles (local frame u-planes; roof follows the cowl inset by skin + dynamic margin)
PLEN = dict(u_front=CYL_U + FIN["hu"] + 0.0015, u_aft=CYL_U - FIN["hu"] - 0.0015, v_out=0.199, w_floor=0.0,
            w_skirt=-0.030, v_floor=0.062, inlet_hy=0.0855, collar=0.012)
SEAL = 0.001                # 1 mm free gap of the baffle edges to the fins (silicone seal strips not modelled)


def plenum_mesh(C: Ctx) -> G.Mesh:
    """Engine-mounted cooling plenum + cylinder baffles, 5052-H32 0.8 mm (formed and riveted): roof 15.8 mm inside
    the cowl OML (skin + 10 mm dynamic margin), front wall with the inlet collar, aft and outboard walls, floor over
    the crankcase on the two M5 bosses, skirts down the fore / aft / outboard faces of each fin pack (air leaves
    through the fins to the lower cowl and the annular exit)."""
    P_ = PLEN
    t = BAFFLE_T
    wf = BAFFLE_BOSS_W
    # roof envelope (world): OML inset by skin + dynamic margin, over x 3.70..3.95
    roof_in = C.body_env(COWL_SKIN + DYN + t, 3.70, 3.95)
    roof_out = C.body_env(COWL_SKIN + DYN, 3.70, 3.95)

    def slab(u0, u1, v0, v1, w0, w1):
        return C.to_air(box3((u0, v0, w0), (u1, v1, w1)))
    uf, ua, vo = P_["u_front"], P_["u_aft"], P_["v_out"]
    outer = inter(roof_out, slab(ua - t, uf + t, -vo - t, vo + t, wf, 0.20))
    inner = inter(roof_in, slab(ua, uf, -vo, vo, wf + t, 0.20))
    box = diff(outer, [inner])
    # open bottom over the fin packs (floor only over the crankcase |v| < v_floor)
    for sv in (1, -1):
        box = diff(box, [slab(ua + OV, uf - OV, min(sv * P_["v_floor"], sv * (vo - OV)),
                              max(sv * P_["v_floor"], sv * (vo - OV)), wf - 0.01, wf + t + OV)])
    # skirts: fore / aft / outboard faces of each fin pack down to w_skirt
    sk = []
    for sv in (1, -1):
        v0, v1 = sorted((sv * (P_["v_floor"] - 0.004), sv * (vo + t)))
        sk.append(slab(uf, uf + t, v0, v1, P_["w_skirt"], wf + OV))
        sk.append(slab(ua - t, ua, v0, v1, P_["w_skirt"], wf + OV))
        o0, o1 = sorted((sv * vo, sv * (vo + t)))
        sk.append(slab(ua - t, uf + t, o0, o1, P_["w_skirt"], wf + OV))
    m = union([box] + sk)
    # inlet: window in the front wall + collar toward the coupling boot
    m = diff(m, [inlet_cutter(C, grow=-t)])
    m = union([m, inlet_collar(C)])
    # clearance notches round the four rear mount bosses (3 mm)
    m = diff(m, [C.to_air(G.cylinder(BOSS_R + 0.003, (0.27, v, w), (0.33, v, w), n=32)) for v, w in C.bolts_vw])
    return finish(largest_piece(m))


def inlet_section(C: Ctx):
    """Inlet collar section (world x-plane): rounded rectangle above the floor / below the roof."""
    T = TRANS
    return 0.5 * (T["z_lo"] + T["z_hi"]), T["hy"], 0.5 * (T["z_hi"] - T["z_lo"]), T["r"]


def _collar_x(C: Ctx) -> tuple[float, float]:
    """x span of the inlet collar: from the plenum front wall (at the collar height) forward by PLEN collar."""
    zc = inlet_section(C)[0]
    p = C.P(PLEN["u_front"], 0.0, 0.0)
    # front wall plane: points q with (q - p) . eu = 0 -> x at height zc
    x_wall = p[0] - (zc - p[2]) * C.eu[2] / C.eu[0]
    return x_wall - PLEN["collar"], x_wall + 0.004


def inlet_collar(C: Ctx) -> G.Mesh:
    zc, hy, hz, r = inlet_section(C)
    x0, x1 = _collar_x(C)
    o = G.extrude(rr_poly(hy, hz, r, zc), x1 - x0, origin=(x0, 0, 0), u=(0, 1, 0), v=(0, 0, 1))
    i = G.extrude(rr_poly(hy - BAFFLE_T, hz - BAFFLE_T, r - BAFFLE_T, zc), x1 - x0 + 0.004, origin=(x0 - 0.002, 0, 0),
                  u=(0, 1, 0), v=(0, 0, 1))
    return diff(o, [i])


def inlet_cutter(C: Ctx, grow: float = 0.0) -> G.Mesh:
    zc, hy, hz, r = inlet_section(C)
    x0, x1 = _collar_x(C)
    return G.extrude(rr_poly(hy + grow, hz + grow, r + grow, zc), x1 - x0 + 0.006, origin=(x0 - 0.002, 0, 0),
                     u=(0, 1, 0), v=(0, 0, 1))


def boot_mesh(C: Ctx) -> G.Mesh:
    """Silicone-coated glass-cloth coupling boot (2 mm wall) over the transition-duct end and the plenum collar,
    clamped with two band clamps; 25 mm free length takes the engine motion on its isolators."""
    T = TRANS
    zc, hy, hz, r = inlet_section(C)
    x0 = T["x_end"] - 0.010
    x1 = _collar_x(C)[0] + 0.006
    tw = 0.002
    o = G.extrude(rr_poly(hy + tw, hz + tw, r + tw, zc), x1 - x0, origin=(x0, 0, 0), u=(0, 1, 0), v=(0, 0, 1))
    i = G.extrude(rr_poly(hy + 0.0001, hz + 0.0001, r + 0.0001, zc), x1 - x0 + 0.004, origin=(x0 - 0.002, 0, 0),
                  u=(0, 1, 0), v=(0, 0, 1))
    return finish(diff(o, [i]))


# =====================================================================================================================
# accessories: throttle actuator + bracket, engine-side fuel hoses, ECU, generator power electronics, node baffles
# =====================================================================================================================
SERVO_PADS = [(0.281, -0.110), (0.281, -0.140)]      # (u, w) of the 2 x M3 per side into tapped case pads


def servo_box(C: Ctx) -> tuple:
    w0 = THROTTLE["shaft_w"] - DA22["shaft_from_bottom"]
    return (SERVO_U0, -DA22["L"] / 2, w0), (SERVO_U0 + DA22["W"], DA22["L"] / 2, w0 + DA22["H"])


def servo_mesh(C: Ctx) -> G.Mesh:
    """Volz DA 22 throttle actuator envelope (case 41.5 x 65.9 x 22 mm), output on the aft face coaxial with the
    throttle shaft; hollow case with 6 mm tapped pads on both sides for the bracket screws (estimate)."""
    lo, hi = servo_box(C)
    lo, hi = np.asarray(lo), np.asarray(hi)
    m = diff(box3(lo, hi), [box3(lo + 0.002, hi - 0.002)])
    pads = []
    for u, w in SERVO_PADS:
        for sv in (1.0, -1.0):
            pads.append(G.cylinder(0.0064, (u, sv * (hi[1] - 0.001), w), (u, sv * (hi[1] - 0.006), w), n=32))
    boss = G.cylinder(0.006, (lo[0] + OV, 0, THROTTLE["shaft_w"]), (lo[0] - 0.0005, 0, THROTTLE["shaft_w"]), n=24)
    return finish(C.to_air(union([m] + pads)))


def servo_bracket_mesh(C: Ctx) -> G.Mesh:
    """5052-H32 1.5 mm U-bracket: top web on the two crankcase-bottom bosses, legs either side of the DA 22."""
    B = SERVO_BRKT
    t, lv = B["t"], B["leg_v"]
    top = box3((B["u0"], -lv - t, B["w_top"] - t), (B["u1"], lv + t, B["w_top"]))
    legs = [box3((B["u0"], sv * lv if sv > 0 else -lv - t, B["w_bot"]), (B["u1"], lv + t if sv > 0 else -lv,
                                                                       B["w_top"] - t + OV)) for sv in (1, -1)]
    return finish(C.to_air(union([top] + legs)))


HOSE_RO, HOSE_RI = 0.007, 0.0044  # -4 fuel hose in fire sleeve (OD 14); end sockets slide 8 mm over the nipples
HOSE_KG_PER_M = 0.15        # hose + fire sleeve + crimped ends (estimate)


def hose_paths(C: Ctx) -> dict:
    """Engine-side flexible hoses from the fuel-module line ends (layout.fuel_lines FL-FEED-3 end, FL-RETURN start)
    to the rail inlet (port end) and the regulator outlet (starboard end)."""
    fl = {f["id"]: f for f in C.L["fuel_lines"]}
    feed_end = np.asarray(fl["FL-FEED-3"]["path"][-1], float)
    feed_dir = G.unit(np.asarray(fl["FL-FEED-3"]["path"][-1]) - np.asarray(fl["FL-FEED-3"]["path"][-2]))
    ret_start = np.asarray(fl["FL-RETURN"]["path"][0], float)
    ret_dir = G.unit(np.asarray(fl["FL-RETURN"]["path"][0]) - np.asarray(fl["FL-RETURN"]["path"][1]))
    inlet = C.P(NIPPLE_U - 0.008, -NIPPLE_V, RAIL["w"])
    reg_out = C.P(NIPPLE_U - 0.008, NIPPLE_V, RAIL["w"])
    lead = 0.010
    feed = np.vstack([feed_end, arc_path(feed_end + lead * feed_dir, feed_dir, inlet + lead * C.eu, -C.eu, n=16),
                      inlet])
    ret = np.vstack([reg_out, arc_path(reg_out + lead * C.eu, C.eu, ret_start + lead * ret_dir, -ret_dir, n=16),
                     ret_start])
    return {"feed": feed, "return": ret}


def hose_mesh(C: Ctx, which: str) -> G.Mesh:
    P = hose_paths(C)[which]
    o, i = tube_along(P, HOSE_RO, HOSE_RI, n=16, ext=0.002)
    return finish(diff(o, [i]))


def unit_box(C: Ctx, key: str):
    """(lo, hi, base thickness) of an equipment unit: layout box; the generator PE adds its heat-sink base plate down
    to the side-bay tray face (layout.rules.boxes.avionics_side_bays z[0])."""
    e = C.eq["EQ-ECU" if key == "ecu" else "EQ-GENERATOR_PE"]
    lo, hi = np.asarray(e["box"][0], float), np.asarray(e["box"][1], float)
    if key == "pe":
        floor = float(C.L["rules"]["boxes"]["avionics_side_bays"]["z"][0])
        base_t = PE_BASE_T if abs(lo[2] - floor) < 1e-6 else lo[2] - floor
        lo = np.array([lo[0], lo[1], floor])
        return lo, hi, base_t
    return lo, hi, ECU_BASE_T


def flanged_box(lo, hi, base_t: float, inset_x: float) -> G.Mesh:
    """Avionics unit envelope: base flange plate over the layout footprint + body inset at the x ends."""
    lo, hi = np.asarray(lo, float), np.asarray(hi, float)
    base = box3(lo, (hi[0], hi[1], lo[2] + base_t))
    body = box3((lo[0] + inset_x, lo[1], lo[2] + base_t - OV), (hi[0] - inset_x, hi[1], hi[2]))
    con = box3((hi[0] - inset_x - OV, 0.5 * (lo[1] + hi[1]) - 0.015, lo[2] + base_t + 0.010),
               (hi[0] - inset_x + 0.008, 0.5 * (lo[1] + hi[1]) + 0.015, lo[2] + base_t + 0.030))
    return finish(union([base, body, con]))


ECU_BASE_T, ECU_INSET, ECU_BOLT = 0.003, 0.0165, (0.008, 0.012, 0.010)   # base, body inset, bolt inset x0/x1/y
PE_BASE_T, PE_INSET, PE_BOLT = 0.004, 0.0145, (0.008, 0.008, 0.008)


def node_baffle_mesh(C: Ctx) -> G.Mesh:
    """Stainless 0.4 mm heat baffle 70 x 60 mm on the plenum front skirt facing the stabilator node boss
    (layout.heat_protection.hardware F-SPINDLE-NODE baffle), starboard."""
    u0 = PLEN["u_front"] + BAFFLE_T
    return finish(C.to_air(box3((u0, NODE_BAFFLE["v"][0], NODE_BAFFLE["w"][0]),
                                (u0 + NODE_BAFFLE["t"], NODE_BAFFLE["v"][1], NODE_BAFFLE["w"][1]))))


NODE_BAFFLE = dict(v=(0.128, 0.198), w=(-0.025, 0.035), t=0.0004)


# =====================================================================================================================
# heat protection in the lower cowl (layout.heat_protection: HS-COWL-EXIT insert PR-520, HS-COWL-SHIELD PR-523)
# =====================================================================================================================
HS = dict(lift=0.0001, lap=0.020, hole=0.010, band_t=0.0016, standoff=0.005, wall_in=0.0055, fl_w=0.014,
          rivet_d=0.0032, rivet_out=0.010, rivet_pitch=0.025, fl_rivet_d=0.0024, fl_rivet_in=0.0125, fl_pitch=0.035,
          edge=0.0005, x0=3.70, phi0=0.42 * math.pi, nx=150, nphi=110)
#   lift: the external insert lies 0.1 mm off the cowl OML (bond / sealant line); lap: riveted lap round the cowl
#   cut-out (layout 20 mm); hole: radial clearance of the tail pipe in the insert (layout 5 mm stand-off ring raised to
#   the 10 mm engine dynamic margin, layout.clearance_values.engine_keep_out: the pipe moves with the engine);
#   band_t: cowl solid edge band at the riveted cut-out edge (layout.shell.rules.sandwich_edges 1.6 mm);
#   standoff: shield air gap below the thickest cowl laminate (layout 5 mm); wall_in / fl_w: the shield's return wall
#   5.5 mm inside the insert region boundary (>= 5 mm air gap to the cowl cut-out edge, which lies on the boundary + the
#   panel gap) and its 14 mm flange riveted to the insert; edge: margin to the P-COWL-LO panel edges


def _boxes(regions, grow: float):
    return [(np.asarray(a, float) - grow, np.asarray(b, float) + grow) for a, b in regions]


def _box_union(regions, grow: float) -> G.Mesh:
    return union([box3(lo, hi) for lo, hi in _boxes(regions, grow)])


def box_sdist(p, regions, grow: float = 0.0) -> float:
    """Signed distance (outside > 0) of point p to the union of the (grown) region boxes."""
    best = math.inf
    for lo, hi in _boxes(regions, grow):
        out = np.maximum(np.maximum(lo - p, 0.0), p - hi)
        d = float(np.linalg.norm(out)) if np.any(out > 0) else -float(min((p - lo).min(), (hi - p).min()))
        best = min(best, d)
    return best


def _hp(C: Ctx, kind: str, hid: str) -> dict:
    return next(i for i in C.L["heat_protection"][kind] if i["id"] == hid)


def _cowl_lo(C: Ctx) -> dict:
    return next(p for p in C.L["shell"]["panels"] if p["id"] == "P-COWL-LO")


def _oml_band(C: Ctx, d0: float, d1: float) -> G.Mesh:
    """Solid between depths d0 < d1 below the OML (negative = outside) over the lower starboard aft body (one grid
    for every heat-protection part, so their faces coincide where they touch)."""
    fus = C.fus
    x1 = fus.x1 - 0.001
    xs = np.linspace(HS["x0"], x1, HS["nx"])
    ph = np.linspace(HS["phi0"], math.pi, HS["nphi"])
    X, PH = np.meshgrid(xs, ph, indexing="ij")
    P = fus.point(X, PH)
    N = fus.normal(X, PH)
    return G.shell_from_grid(P - d0 * N, d1 - d0, inward=-N)


def _cowl_clip(C: Ctx, extra: float = 0.0) -> G.Mesh:
    """P-COWL-LO extents (layout x, z_band, inner y edge), less the edge margin (+ extra)."""
    pan = _cowl_lo(C)
    x0, x1 = pan["x"]
    z0, z1 = pan["z_band"]
    e = HS["edge"] + extra
    return box3((x0 + e, float(pan["y"][0]) + e, z0 - 0.1), (x1 + 0.1, 0.5, z1 - e))


def _pipe_cutter(C: Ctx, clear: float) -> G.Mesh:
    """Clearance volume round the tail pipe: union of capsules (hulls of two spheres) along its centre line - a swept
    circle of this radius would fold inside the 1.5 D bend."""
    pa = exhaust_paths(C)
    path = np.vstack([pa["tail"][:-1], pa["tail"][-1] + np.outer([0.0, 0.03, 0.06], pa["exit_dir"])])
    sph = G.sphere(TAIL_R + clear, n=20).V
    caps = [G.hull(np.vstack([sph + a, sph + b])) for a, b in zip(path[:-1], path[1:])
            if np.linalg.norm(b - a) > 1e-6]
    return union(caps)


def cowl_insert_mesh(C: Ctx) -> G.Mesh:
    """HS-COWL-EXIT (PR-520-R): AISI 304 0.4 mm external insert over the cowl cut-out (the layout insert regions, cut
    from the lower cowl by the shell) and its 20 mm riveted lap, 0.1 mm off the OML; tail-pipe exit slot with 10 mm
    radial clearance."""
    hp = _hp(C, "inserts", "HS-COWL-EXIT")
    t = float(hp["t"])
    m = inter(inter(_oml_band(C, -HS["lift"] - t, -HS["lift"]), _box_union(hp["regions"], HS["lap"])), _cowl_clip(C))
    return finish_clean(largest_piece(diff(m, [_pipe_cutter(C, HS["hole"])])))


def _surface_samples(C: Ctx, nx: int = 300, nphi: int = 240):
    xs = np.linspace(HS["x0"], C.fus.x1 - 0.002, nx)
    ph = np.linspace(HS["phi0"], math.pi, nphi)
    X, PH = np.meshgrid(xs, ph, indexing="ij")
    return C.fus.point(X, PH).reshape(-1, 3), X.ravel(), PH.ravel()


def _greedy(points, pitch: float):
    out = []
    for p, x, ph in points:
        if all(np.linalg.norm(p - q) >= pitch for q, _x, _ph in out):
            out.append((p, x, ph))
    return out


def _pipe_dist(C: Ctx, p) -> float:
    pa = exhaust_paths(C)
    path = np.vstack([pa["tail"], pa["tail"][-1] + np.outer([0.03, 0.06], pa["exit_dir"])])
    a, b = path[:-1], path[1:]
    ab = b - a
    tt = np.clip(np.einsum("ij,ij->i", p - a, ab) / np.maximum(np.einsum("ij,ij->i", ab, ab), 1e-12), 0.0, 1.0)
    return float(np.min(np.linalg.norm(a + tt[:, None] * ab - p, axis=1))) - TAIL_R


def cowl_insert_rivets(C: Ctx) -> list[tuple]:
    """Lap rivets insert -> cowl (OML point, inward axis): 10 mm outside the cut-out edge (2.5 D in the cowl band,
    10 mm to the insert edge), ~25 mm pitch, >= 2.5 D + margin inside the cowl panel edges."""
    hp = _hp(C, "inserts", "HS-COWL-EXIT")
    pan = _cowl_lo(C)
    P, X, PH = _surface_samples(C)
    e = 2.5 * HS["rivet_d"] + 0.004
    cand = []
    for p, x, ph in zip(P, X, PH):
        if abs(box_sdist(p, hp["regions"]) - HS["rivet_out"]) > 0.0006:
            continue
        if p[2] > pan["z_band"][1] - e or p[0] > pan["x"][1] - e or p[1] < pan["y"][0] + e:
            continue
        cand.append((p, x, ph))
    return [(p, -C.fus.normal(x, ph)) for p, x, ph in _greedy(cand, HS["rivet_pitch"])]


def cowl_shield_mesh(C: Ctx) -> G.Mesh:
    """HS-COWL-SHIELD (PR-523-R): AISI 304 sheet (0.4 mm, process minimum; layout foil 0.1 mm) 5 mm inside the
    thickest cowl laminate over the shield band (25..50 mm from the stack envelope), with a return wall 5.5 mm inside
    the cut-out edge and a 14 mm flange riveted under the insert: the shield hangs from the insert, the cowl inner face
    and its cut edge keep their 5 mm air gap whatever the edge build-up."""
    sh = _hp(C, "shields", "HS-COWL-SHIELD")
    reg = _hp(C, "inserts", "HS-COWL-EXIT")["regions"]
    t = float(_hp(C, "inserts", "HS-COWL-EXIT")["t"])
    d0 = COWL_SKIN + HS["standoff"]
    w0 = HS["wall_in"]
    clip = _cowl_clip(C, 0.002)
    # the three pieces overlap by half a sheet thickness (no coplanar faces in the union)
    pan_ = inter(_oml_band(C, d0, d0 + t), diff(_box_union(sh["regions"], 0.0), [_box_union(reg, -w0 - 0.5 * t)]))
    wall = inter(_oml_band(C, -HS["lift"] + 0.5 * t, d0 + 0.5 * t),
                 diff(_box_union(reg, -w0), [_box_union(reg, -w0 - t)]))
    fl = inter(_oml_band(C, -HS["lift"], -HS["lift"] + t),
               diff(_box_union(reg, -w0 - 0.5 * t), [_box_union(reg, -w0 - HS["fl_w"])]))
    m = inter(union([pan_, wall, fl]), clip)
    return finish_clean(largest_piece(diff(m, [_pipe_cutter(C, HS["hole"] + 0.003)])))


def sheet_outline(m: G.Mesh, C: Ctx, cos_min: float = 0.7, bend=None) -> list[np.ndarray]:
    """Cut-edge polylines of a formed sheet on the aft body (boundary of its outward-facing face) -> ``Part.outline``:
    the edge-distance check then measures to the real trimmed edges; its radial mid-plane rays would leave a 0.4 mm
    sheet curved to the boat-tail within a few millimetres. ``bend(points) -> bool mask`` marks boundary points that
    lie on a bend line (the root of a formed wall), which is not a free edge: those segments are dropped."""
    V, F = m.V, m.F
    a, b, c = V[F[:, 0]], V[F[:, 1]], V[F[:, 2]]
    fn = np.cross(b - a, c - a)
    fn /= np.maximum(np.linalg.norm(fn, axis=1, keepdims=True), 1e-18)
    cen = (a + b + c) / 3.0
    w, h, zc, _nt, _nb = (np.asarray(q, float) for q in C.fus.section(cen[:, 0]))
    tf = np.asarray(C.fus.top_frac(cen[:, 0]), float)
    hh = np.where(cen[:, 2] >= zc, tf * h, (1.0 - tf) * h)
    phi = np.arctan2(2.0 * cen[:, 1] / w, (cen[:, 2] - zc) / hh)          # exact for the n = 2 aft sections
    sel = np.einsum("ij,ij->i", fn, C.fus.normal(cen[:, 0], phi)) > cos_min
    E = np.sort(np.vstack([F[sel][:, [0, 1]], F[sel][:, [1, 2]], F[sel][:, [2, 0]]]), axis=1)
    uq, cnt = np.unique(E, axis=0, return_counts=True)
    adj: dict[int, list[int]] = {}
    for i, j in uq[cnt == 1]:
        adj.setdefault(int(i), []).append(int(j))
        adj.setdefault(int(j), []).append(int(i))
    lines, used = [], set()
    for start in adj:
        for nxt in adj[start]:
            if (min(start, nxt), max(start, nxt)) in used:
                continue
            chain, prev, cur = [start], start, nxt
            used.add((min(start, nxt), max(start, nxt)))
            while True:
                chain.append(cur)
                cand = [k for k in adj[cur] if (min(cur, k), max(cur, k)) not in used]
                if not cand:
                    break
                used.add((min(cur, cand[0]), max(cur, cand[0])))
                prev, cur = cur, cand[0]
            lines.append(V[chain].copy())
    if bend is None:
        return lines
    out = []
    for L in lines:
        keep = ~np.asarray(bend(L), bool)
        seg = []
        for k in range(len(L)):
            if keep[k]:
                seg.append(L[k])
            else:
                if len(seg) > 1:
                    out.append(np.array(seg))
                seg = []
        if len(seg) > 1:
            out.append(np.array(seg))
    return out


def shield_bend_mask(C: Ctx):
    """Boundary points on the shield's wall roots (the band of the return wall, wall_in .. wall_in + t inside the
    cut-out edge)."""
    reg = _hp(C, "inserts", "HS-COWL-EXIT")["regions"]
    lo, hi = -HS["wall_in"] - 0.0004 - 0.0006, -HS["wall_in"] + 0.0006

    def mask(P):
        return np.array([lo <= box_sdist(p, reg) <= hi for p in P])
    return mask


def shield_rivets(C: Ctx) -> list[tuple]:
    """Shield flange rivets (through the insert, set from outside): 12.5 mm inside the cut-out edge (mid-flange),
    ~35 mm pitch, clear of the pipe slot."""
    reg = _hp(C, "inserts", "HS-COWL-EXIT")["regions"]
    pan = _cowl_lo(C)
    P, X, PH = _surface_samples(C)
    e = 2.0 * HS["fl_rivet_d"] + 0.003
    cand = []
    for p, x, ph in zip(P, X, PH):
        if abs(box_sdist(p, reg) + HS["fl_rivet_in"]) > 0.0006:
            continue
        if p[2] > pan["z_band"][1] - e or p[0] > pan["x"][1] - e:
            continue
        if _pipe_dist(C, p) < HS["hole"] + 0.003 + e:
            continue
        cand.append((p, x, ph))
    return [(p, -C.fus.normal(x, ph)) for p, x, ph in _greedy(cand, HS["fl_pitch"])]


# =====================================================================================================================
# registration
# =====================================================================================================================
class _Reg:
    def __init__(self, C: Ctx):
        self.C = C
        self.reg = C.reg

    def add(self, num, side, name, name_tr, material, process, fn, **kw) -> Part:
        C = self.C
        pid = C.pid(num, side)
        memo = {}

        def mesh_fn(fn=fn, memo=memo):
            if "m" not in memo:
                memo["m"] = fn()
            return memo["m"]
        kw.setdefault("color", "")
        return self.reg.add(Part(id=pid, name=name, name_tr=name_tr, group=GROUP, material=material, process=process,
                                 mesh_fn=mesh_fn, side=side, **kw))

    def ex(self, *v) -> tuple:
        return tuple(float(a) for a in np.asarray(v, float))

    # ------------------------------------------------------------------ engine
    def engine(self):
        C, A = self.C, self.add
        eng = C.S["engine"]
        items = eng["installed_items_kg"]
        mount = C.eng["part"]
        exd = EX_ENG * C.d
        spool_m = spool_mesh(C).volume() * float(C.S["materials"][MAT_STEEL]["density"])
        self.ids = ids = {}
        ids["engine"] = A(500, "C", "engine Limbach L 275 EF (static)", "motor Limbach L 275 EF", P_BUY, P_BUY,
                          lambda: engine_mesh(C), purchased=True,
                          vendor="Limbach Flugmotoren L 275 EF (EFI, 274 cc boxer twin); intake, throttle body, injectors "
                                 "and rail, plugs and heat sinks included",
                          mass_kg=float(items["engine_bare"]) - spool_m, parent=mount, step=STEP_ENGINE,
                          explode=self.ex(*exd), contacts=(),
                          notes=f"dry mass {items['engine_bare']} kg (engine.installed_items_kg.engine_bare) less the "
                                f"output spool ({spool_m:.3f} kg) modelled as PR-501; envelope inside KO-ENGINE; port "
                                "locations per the Limbach installation drawing are an open item").id
        ids["spool"] = A(501, "C", "engine output spool (propeller flange)", "motor çıkış makarası (pervane flanşı)",
                         MAT_STEEL, P_BUY, lambda: spool_mesh(C), purchased=True,
                         vendor="Limbach L 275 EF crank-nose spool (part of the engine)", mass_kg=spool_m,
                         parent=ids["engine"], step=STEP_ENGINE, explode=self.ex(*exd), joint="prop_spin",
                         notes="rotating part of the engine; mass = geometry x steel density (estimate), deducted from "
                               "the engine dry mass").id
        ids["sg750"] = A(502, "C", "starter-generator ePropelled SG750", "marş-jeneratör ePropelled SG750", P_BUY, P_BUY,
                         lambda: sg750_mesh(C), purchased=True, vendor="ePropelled SG750 (800 W cont., 50 V AC 3-ph)",
                         mass_kg=float(items["starter_generator_sg750"]), parent=ids["engine"], step=STEP_ENGINE,
                         explode=self.ex(*(exd - 0.08 * C.d)), notes="engine.installed_items_kg.starter_generator_sg750")
        ids["sg750"] = ids["sg750"].id
        ids["adapter"] = A(503, "C", "SG750 adapter flange", "SG750 adaptör flanşı", MAT_7075, P_CNC,
                           lambda: adapter_mesh(C), thickness=ADAPTER_T - 0.0045, parent=ids["engine"],
                           step=STEP_ENGINE, explode=self.ex(*(exd - 0.05 * C.d)),
                           contacts=(ids["engine"], ids["sg750"]),
                           notes=f"machined 7075-T651; budget item generator_adapter_coupling "
                                 f"{items['generator_adapter_coupling']} kg").id
        self.reg.parts[ids["sg750"]].contacts = (ids["adapter"],)
        for k in range(4):
            ids[f"iso{k}"] = A(506 + k, "C", f"engine isolator {k + 1}", f"motor titreşim takozu {k + 1}", P_BUY, P_BUY,
                               (lambda k=k: isolator_mesh(C, k)), purchased=True,
                               vendor="Limbach 'Damper Shock Mount' (conical elastomer, fail-safe through-bolt)",
                               mass_kg=self._iso_mass(k), parent=mount, step=STEP_ENGINE,
                               explode=self.ex(*(0.5 * exd)), contacts=(mount, ids["engine"]),
                               notes="layout envelope d 40 x 25 (estimate); mass = steel + elastomer volumes "
                                     "(estimate); dimensions and stiffness not published").id

    def _iso_mass(self, k):
        m = isolator_mesh(self.C, k)
        v = m.volume()
        steel = math.pi * (ISO_R_TOP ** 2) * 0.002 + math.pi * (ISO_R_BASE ** 2 - ISO_SLEEVE_R ** 2) * 0.0035 + \
            math.pi * (ISO_SLEEVE_R ** 2 - ISO_BORE ** 2) * ISO_H
        return round(steel * 7833.0 + max(v - steel, 0.0) * ELASTOMER_DENSITY, 4)

    # ------------------------------------------------------------------ propeller group
    def propeller(self):
        C, A, ids = self.C, self.add, self.ids
        pr = C.S["propeller"]
        lj = next(j for j in C.L["mechanisms"]["joints"] if j["name"] == "prop_spin")
        self.reg.add_joint(Joint(name="prop_spin", kind=lj["kind"], origin=np.asarray(lj["origin"], float),
                                 axis=np.asarray(lj["axis"], float), lo=float(lj["lo"]), hi=float(lj["hi"]),
                                 rest=float(lj["rest"]), prop=lj["prop"], scale=float(lj["scale"]),
                                 notes=lj.get("notes", "")))
        ex = 0.45 * C.d
        ids["prop"] = A(540, "C", f"propeller {pr['model']}", "pervane Mejzlik 31x12 3B", P_BUY, P_BUY,
                        lambda: prop_mesh(C), purchased=True, vendor=f"Mejzlik Propellers {pr['model']} (carbon, "
                        "pusher hand per the engine rotation)", mass_kg=float(pr["mass_kg"]), joint="prop_spin",
                        step=STEP_PROP, explode=self.ex(*ex), color="propulsion",
                        notes="blade planform spec.propeller.blade_model (estimate) with a narrower root shank inside "
                              "the spinner; drilling to the hub pattern by the supplier").id
        ids["spacer"] = A(541, "C", "propeller hub spacer 110 mm", "pervane göbek ara parçası 110 mm", MAT_7075, P_CNC,
                          lambda: spacer_mesh(C), thickness=SPACER["r_o"] - SPACER["r_i"], joint="prop_spin",
                          parent=ids["spool"], step=STEP_PROP, explode=self.ex(*(0.25 * C.d)),
                          contacts=(ids["spool"],), notes="propeller.hub_spacer (design choice); Limbach approval "
                                                          "open").id
        ids["spinner"] = A(542, "C", "spinner cone", "pervane konisi (spinner)", MAT_PW, P_PREG,
                           lambda: spinner_mesh(C), thickness=SPINNER_T, joint="prop_spin", parent=ids["prop"],
                           step=STEP_PROP, explode=self.ex(*(0.65 * C.d)), color="shell",
                           notes="CFRP PW 4 plies (0.8 mm), 4 mm tip boss; propeller.spinner (estimate)").id
        ids["backplate"] = A(543, "C", "spinner backplate", "spinner arka plakası", MAT_7075, P_CNC,
                             lambda: backplate_mesh(C), thickness=BACKPLATE_T, joint="prop_spin", parent=ids["spacer"],
                             step=STEP_PROP, explode=self.ex(*(0.35 * C.d)),
                             contacts=(ids["spacer"], ids["prop"], ids["spinner"])).id
        ids["crush"] = A(544, "C", "propeller crush plate with spinner stand-off", "pervane baskı plakası ve spinner "
                         "dayanağı", MAT_7075, P_CNC, lambda: crush_mesh(C), thickness=CRUSH_T, joint="prop_spin",
                         parent=ids["prop"], step=STEP_PROP, explode=self.ex(*(0.55 * C.d)),
                         contacts=(ids["prop"], ids["spinner"])).id
        self.reg.parts[ids["prop"]].parent = ids["spacer"]
        self.reg.parts[ids["prop"]].contacts = (ids["backplate"], ids["crush"])
        self.reg.parts[ids["spinner"]].contacts = (ids["backplate"], ids["crush"])

    # ------------------------------------------------------------------ exhaust
    def exhaust(self):
        C, A, ids = self.C, self.add, self.ids
        items = C.S["engine"]["installed_items_kg"]
        exr = A(504, "R", "exhaust, starboard (port flange, silencer, tail pipe)", "egzoz, sağ (flanş, susturucu, "
                "çıkış borusu)", MAT_SS, P_SHEET_ST, lambda: exhaust_mesh(C), thickness=EXH_T, parent=ids["engine"],
                step=STEP_ACC, explode=self.ex(*(EX_ENG * C.d + np.array([0.0, 0.10, -0.12]))),
                contacts=(ids["engine"],), color="hardware",
                notes=f"stainless 304 tube / rolled can 0.8 mm, flange 6 mm (laser cut), TIG welded; inside "
                      f"KO-EXHAUST-R, exit point and direction per layout; allowance exhaust_with_silencers "
                      f"{items['exhaust_with_silencers']} kg (both sides)")
        ids["exh_R"] = exr.id
        ids["exh_L"] = self.reg.add(mirror_part(exr, C.pid(504, "L"))).id

    # ------------------------------------------------------------------ cooling
    def cooling(self):
        C, A, ids = self.C, self.add, self.ids
        shield = part_number(GROUP, 88)                 # firewall shield (chassis-made, propulsion group)
        ex = EX_ENG * C.d
        ids["trans"] = A(548, "C", "firewall cooling spigot / transition duct", "yangın perdesi soğutma geçiş kanalı",
                         MAT_SS, P_SHEET_ST, lambda: transition_mesh(C), thickness=SPIGOT_T, parent=shield,
                         step=STEP_ACC, explode=self.ex(0.08, 0.0, 0.10), contacts=(shield,), color="hardware",
                         notes="AISI 304 0.5 mm, brake-formed and seam-welded; spigot through C-DUCT with a fireproof "
                               "silicone-glass seal; flange riveted to the shield (6 x d 2.4); rises 5 mm over the "
                               "engine-mount truss").id
        ids["sduct"] = A(546, "C", "cooling-air S-duct", "soğutma havası S-kanalı", MAT_PW, P_PREG,
                         lambda: sduct_mesh(C), thickness=DUCT_T, parent=ids["trans"], step=STEP_ACC,
                         explode=self.ex(-0.12, 0.0, 0.12), contacts=(ids["trans"],), color="shell",
                         notes="CFRP PW 4 plies 0.8 mm on a split male mould; inside KO-COOLING-DUCT; forward end "
                               "bonded into the inlet lip socket (shell P-INLET), aft end slips 35 mm over the spigot "
                               "(4 x M4 + nutplates)").id
        ids["plenum"] = A(552, "C", "cooling plenum and cylinder baffles", "soğutma plenumu ve silindir perdeleri",
                          MAT_5052, P_SHEET_AL, lambda: plenum_mesh(C), thickness=BAFFLE_T, parent=ids["engine"],
                          step=STEP_ACC, explode=self.ex(*(ex + np.array([0.0, 0.0, 0.16]))),
                          contacts=(ids["engine"],), color="chassis",
                          notes="5052-H32 0.8 mm, formed (bend radius >= 1.5 t) and riveted; roof 15.8 mm inside the "
                                "cowl OML; skirts 1 mm off the fin packs (silicone seal strips); 4 x M5 to the "
                                "crankcase and head bosses").id
        vb = boot_mesh(C).volume()
        ids["boot"] = A(550, "C", "plenum coupling boot", "plenum bağlantı körüğü", P_BUY, P_BUY,
                        lambda: boot_mesh(C), purchased=True,
                        vendor="silicone-coated glass-cloth duct coupling, 2 mm, two band clamps (fireproof class)",
                        mass_kg=round(vb * 1500.0, 3), parent=ids["trans"], step=STEP_ACC,
                        explode=self.ex(0.04, 0.0, 0.14), contacts=(ids["trans"], ids["plenum"]),
                        notes="mass = volume x 1500 kg/m3 (estimate); 25 mm free length for the isolator travel").id
        nb = A(524, "R", "node heat baffle, starboard", "düğüm ısı perdesi, sağ", MAT_SS, P_SHEET_ST,
               lambda: node_baffle_mesh(C), thickness=NODE_BAFFLE["t"], parent=ids["plenum"], step=STEP_ACC,
               explode=self.ex(*(ex + np.array([0.0, 0.05, 0.16]))), contacts=(ids["plenum"],), color="hardware",
               notes="layout.heat_protection.hardware F-SPINDLE-NODE baffle: AISI 304 0.4 mm 70 x 60 mm between the "
                     "cylinder head and the stabilator node boss, riveted to the plenum front skirt")
        ids["nb_R"] = nb.id
        ids["nb_L"] = self.reg.add(mirror_part(nb, C.pid(524, "L"))).id

    # ------------------------------------------------------------------ heat protection (lower cowl)
    def heat_protection(self) -> bool:
        """HS-COWL-EXIT (PR-520-R/L) and HS-COWL-SHIELD (PR-523-R/L) ride on the lower cowl halves: registered only
        when the shell has registered them (YK250-SH-451-R/L). PR-521 / PR-522 are not required (the stub-root
        strip and the stabilator stub keep >= 50 mm from the exhaust, docs/detail/propulsion.md)."""
        C, A, ids, reg = self.C, self.add, self.ids, self.reg
        hosts = {side: _cowl_lo(C)["part"] + "-" + side for side in ("R", "L")}
        if not all(h in reg.parts for h in hosts.values()):
            reg.note("propulsion: lower cowl halves not registered - heat-protection parts PR-520 / PR-523 skipped")
            return False
        hp, sh = _hp(C, "inserts", "HS-COWL-EXIT"), _hp(C, "shields", "HS-COWL-SHIELD")
        ins = A(520, "R", "exhaust exit insert, starboard lower cowl", "egzoz çıkış levhası, sağ alt kaporta", MAT_SS,
                P_SHEET_ST, lambda: cowl_insert_mesh(C), thickness=float(hp["t"]), parent=hosts["R"],
                step=STEP_CLOSE, explode=(0.0, 0.10, -0.25), contacts=(hosts["R"],), color="hardware",
                notes="layout HS-COWL-EXIT: AISI 304 0.4 mm (FIRE-001 fireproof without test), formed to the OML, "
                      "external over the cowl cut-out (layout insert regions) with a 20 mm lap, blind rivets d 3.2 "
                      "at ~25 mm; tail-pipe exit slot with 10 mm radial clearance")
        shd = A(523, "R", "cowl heat shield, starboard", "kaporta ısı kalkanı, sağ", MAT_SS, P_SHEET_ST,
                lambda: cowl_shield_mesh(C), thickness=float(hp["t"]), parent=ins.id, step=STEP_CLOSE,
                explode=(0.0, 0.05, -0.20), contacts=(ins.id,), color="hardware",
                notes=f"layout {sh['id']}: AISI 304 0.4 mm (layout foil 0.1 mm is below the sheet_metal_steel "
                      "minimum), 5 mm air gap below the thickest cowl laminate over the 25..50 mm band, return wall "
                      "5.5 mm inside the cut-out edge, flange riveted under the insert (d 2.4 at ~35 mm)")
        ins.outline = sheet_outline(ins.base_mesh, C)
        shd.outline = sheet_outline(shd.base_mesh, C, bend=shield_bend_mask(C))
        ids["ins_R"], ids["shd_R"] = ins.id, shd.id
        ids["ins_L"] = reg.add(mirror_part(ins, C.pid(520, "L"), id_map={hosts["R"]: hosts["L"]})).id
        ids["shd_L"] = reg.add(mirror_part(shd, C.pid(523, "L"), id_map={ins.id: ids["ins_L"]})).id
        ids["cowl_R"], ids["cowl_L"] = hosts["R"], hosts["L"]
        return True

    def fasteners_heat(self):
        C, ids, reg = self.C, self.ids, self.reg
        t = float(_hp(C, "inserts", "HS-COWL-EXIT")["t"])
        lap = cowl_insert_rivets(C)
        fl = shield_rivets(C)
        for side, sv in (("R", 1.0), ("L", -1.0)):
            M = np.array([1.0, sv, 1.0])
            for i, (p, a) in enumerate(lap):
                n = -a
                J.rivet(reg, f"{ids['ins_' + side]}-R{i + 1}", HS["rivet_d"], M * (p + (HS["lift"] + t) * n), M * a,
                        [(ids["ins_" + side], t + HS["lift"]), (ids["cowl_" + side], HS["band_t"])],
                        spec="blind rivet stainless (Monel) d 3.2", step=STEP_CLOSE,
                        notes="insert lap -> lower cowl solid edge band (sandwich_edges 1.6 mm)")
            for i, (p, a) in enumerate(fl):
                n = -a
                J.rivet(reg, f"{ids['shd_' + side]}-R{i + 1}", HS["fl_rivet_d"], M * (p + (HS["lift"] + t) * n), M * a,
                        [(ids["ins_" + side], t), (ids["shd_" + side], t + HS["lift"])],
                        spec="blind rivet stainless (Monel) d 2.4", step=STEP_CLOSE,
                        notes="shield flange under the insert, set from outside")

    # ------------------------------------------------------------------ accessories and equipment
    def accessories(self):
        C, A, ids = self.C, self.add, self.ids
        ex = EX_ENG * C.d
        ids["brkt"] = A(515, "C", "throttle actuator bracket", "gaz eyleyicisi braketi", MAT_5052, P_SHEET_AL,
                        lambda: servo_bracket_mesh(C), thickness=SERVO_BRKT["t"], parent=ids["engine"],
                        step=STEP_ACC, explode=self.ex(*(ex + np.array([0.0, 0.0, -0.10]))),
                        contacts=(ids["engine"],), color="chassis").id
        ids["servo"] = A(514, "C", "throttle actuator Volz DA 22", "gaz eyleyicisi Volz DA 22", P_BUY, P_BUY,
                         lambda: servo_mesh(C), purchased=True, vendor="Volz DA 22-30-4128 (28 V)",
                         mass_kg=DA22["mass"], parent=ids["brkt"], step=STEP_ACC,
                         explode=self.ex(*(ex + np.array([0.0, 0.0, -0.16]))), contacts=(ids["brkt"], ids["engine"]),
                         notes="components.yaml volz_da22_28v (case 41.5 x 65.9 x 22 mm, 0.132 kg); output coupled "
                               "to the throttle cross shaft; allowance throttle_servo_cht_egt_sensors").id
        fl = {f["id"]: f for f in C.L["fuel_lines"]}
        for num, which, line in ((516, "feed", "FL-FEED-3"), (517, "return", "FL-RETURN")):
            P_ = hose_paths(C)[which]
            length = float(np.linalg.norm(np.diff(P_, axis=0), axis=1).sum())
            fid = fl[line]["part"]
            cont = (ids["engine"],) + ((fid,) if fid in self.reg.parts else ())
            ids["hose_" + which] = A(num, "C", f"engine fuel hose ({which})", "motor yakıt hortumu (" +
                                     ("besleme" if which == "feed" else "dönüş") + ")", P_BUY, P_BUY,
                                     (lambda w=which: hose_mesh(C, w)), purchased=True,
                                     vendor="-4 PTFE-lined fuel hose in fire sleeve, crimped sockets",
                                     mass_kg=round(length * HOSE_KG_PER_M, 3), parent=ids["engine"], step=STEP_ACC,
                                     explode=self.ex(*(0.5 * ex + np.array([0.0, 0.0, -0.12]))), contacts=cont,
                                     notes=f"flexible link from the {line} end (layout.fuel_lines, fuel module) to the "
                                           "engine rail / regulator: takes the isolator motion").id
        eq = C.eq
        e, pe = eq["EQ-ECU"], eq["EQ-GENERATOR_PE"]
        ids["ecu"] = A(510, "C", e["name"], e["name_tr"], P_BUY, P_BUY,
                       lambda: flanged_box(*unit_box(C, "ecu"), ECU_INSET), purchased=True,
                       vendor="Limbach engine ECU (CAN, 12/24 V) incl. ignition coils allowance", mass_kg=float(e["mass_kg"]),
                       parent=e["mount"]["tray"].split()[0], step=STEP_AVI, explode=(0.0, 0.0, -0.15),
                       contacts=(e["mount"]["tray"].split()[0],), color="payload",
                       notes=f"layout {e['id']} box and mass ({e['source']})").id
        ids["pe"] = A(512, "C", pe["name"], pe["name_tr"], P_BUY, P_BUY,
                      lambda: flanged_box(*unit_box(C, "pe"), PE_INSET), purchased=True,
                      vendor="ePropelled iPS750-class rectifier / regulator with heat-sink base plate",
                      mass_kg=float(pe["mass_kg"]), parent=pe["mount"]["tray"], step=STEP_AVI,
                      explode=(0.0, -0.12, 0.05), contacts=(pe["mount"]["tray"],), color="payload",
                      notes=f"layout {pe['id']} box and mass ({pe['source']})").id

    def fasteners_cooling(self):
        C, ids, reg = self.C, self.ids, self.reg
        shield = part_number(GROUP, 88)
        for i, p in enumerate(flange_rivet_points(C)):
            J.rivet(reg, f"{ids['trans']}-R{i + 1}", 0.0024, p + np.array([SPIGOT_T, 0, 0]), (-1.0, 0.0, 0.0),
                    [(ids["trans"], SPIGOT_T), (shield, float(C.st["FS3670"]["shield_t"]))], spec="blind rivet "
                    "stainless (Monel) d 2.4", step=STEP_ACC)
        # S-duct on the spigot: 4 x M4 button heads through the duct and spigot walls into nutplates (inside)
        zc, hy, hz, r = spigot_section(C)
        for i, (y, sz) in enumerate(((-0.05, 1), (0.05, 1), (-0.05, -1), (0.05, -1))):
            p = np.array([SDUCT_SCREW_X, y, zc + sz * (hz + DUCT_T + 0.0001)])
            J.bolt(reg, f"{ids['sduct']}-B{i + 1}", 4, p, (0.0, 0.0, -sz), [(ids["sduct"], DUCT_T + 0.0001),
                                                                           (ids["trans"], SPIGOT_T)],
                   head="ISO 7380", grade="A2-70", nut="nutplate", step=STEP_ACC, hole_d=FINE[4],
                   orient=(1.0, 0.0, 0.0))
        # plenum: 2 x M5 floor -> crankcase bosses, 2 x M5 outboard walls -> head bosses
        t = BAFFLE_T
        for i, u in enumerate(BAFFLE_BOSS_U):
            J.bolt(reg, f"{ids['plenum']}-B{i + 1}", 5, C.P(u, 0.0, BAFFLE_BOSS_W + t), -C.ew, [(ids["plenum"], t)],
                   grade="A2-70", nut="tapped", tapped_part=ids["engine"], tapped_depth=0.010, step=STEP_ACC,
                   washer_head=True)
        for i, sv in enumerate((1.0, -1.0)):
            J.bolt(reg, f"{ids['plenum']}-B{i + 3}", 5, C.P(CYL_U, sv * (PLEN["v_out"] + t), HEAD_BOSS_W), -sv * C.ev,
                   [(ids["plenum"], t)], grade="A2-70", nut="tapped", tapped_part=ids["engine"],
                   tapped_depth=0.010, step=STEP_ACC, washer_head=True)
        # node heat baffles: 4 blind rivets d 2.4 each into the front skirt
        u0 = PLEN["u_front"] + BAFFLE_T + NODE_BAFFLE["t"]
        for side, sv in (("R", 1.0), ("L", -1.0)):
            k = 0
            for v in (NODE_BAFFLE["v"][0] + 0.006, NODE_BAFFLE["v"][1] - 0.006):
                for w in (NODE_BAFFLE["w"][0] + 0.006, NODE_BAFFLE["w"][1] - 0.006):
                    k += 1
                    J.rivet(reg, f"{ids['nb_' + side]}-R{k}", 0.0024, C.P(u0, sv * v, w), C.d,
                            [(ids["nb_" + side], NODE_BAFFLE["t"]), (ids["plenum"], BAFFLE_T)],
                            spec="blind rivet aluminium d 2.4", step=STEP_ACC)

    def fasteners_accessories(self):
        C, ids, reg = self.C, self.ids, self.reg
        B = SERVO_BRKT
        for i, v in enumerate(SERVO_BOLT_V):
            J.bolt(reg, f"{ids['brkt']}-B{i + 1}", 5, C.P(SERVO_BOLT_U, v, B["w_top"] - B["t"]), C.ew,
                   [(ids["brkt"], B["t"])], grade="A2-70", nut="tapped", tapped_part=ids["engine"],
                   tapped_depth=0.010, step=STEP_ACC, washer_head=True)
        k = 0
        for u, w in SERVO_PADS:
            for sv in (1.0, -1.0):
                k += 1
                J.bolt(reg, f"{ids['servo']}-B{k}", 3, C.P(u, sv * (B["leg_v"] + B["t"]), w), -sv * C.ev,
                       [(ids["brkt"], B["t"] + 0.0001)], grade="A2-70", nut="tapped", tapped_part=ids["servo"],
                       tapped_depth=0.006, step=STEP_ACC, owner=ids["brkt"])
        for key, bolt in (("ecu", ECU_BOLT), ("pe", PE_BOLT)):
            lo, hi, base_t = unit_box(C, key)
            tray = reg.parts[ids[key]].parent
            k = 0
            for x in (lo[0] + bolt[0], hi[0] - bolt[1]):
                for y in (lo[1] + bolt[2], hi[1] - bolt[2]):
                    k += 1
                    J.bolt_through(reg, f"{ids[key]}-B{k}", 4, (x, y, lo[2] + 0.5 * base_t), (0.0, 0.0, -1.0),
                                   [ids[key], tray], grade="A2-70", step=STEP_AVI, hole_d=FINE[4],
                                   notes="ISO 4762 + ISO 7040 nyloc under the tray; elastomer washers (isolated "
                                         "mounting, layout TR-MISSION) not modelled")

    # ------------------------------------------------------------------ fasteners
    def fasteners_core(self):
        C, ids, reg = self.C, self.ids, self.reg
        mount = C.eng["part"]
        # engine isolator bolts: head + washer on the cup bottom, through the isolator, into the crankcase boss
        for k, p in enumerate(C.eng["bolts"]["points"]):
            pt = np.asarray(p, float)
            J.bolt_through(reg, f"{ids[f'iso{k}']}-B1", 8, pt, C.d, [mount, ids[f"iso{k}"]], owner=ids[f"iso{k}"],
                           grade="12.9", nut="tapped", tapped_part=ids["engine"], tapped_depth=0.0145,
                           washer_head=True, step=STEP_ENGINE, max_gap=0.0006, hole_d=FINE[8],
                           notes="ISO 4762 M8 12.9 + NORD-LOCK pair (modelled as ISO 7089), safety-wired in pairs; "
                                 "fail-safe: the head on the cup floor keeps the engine captive")
        # SG750 adapter: 4 x M4 counter-bored into the rear housing; SG750: 4 x M4 through the unit into the adapter
        uc = C.u_coupling
        for i, a in enumerate(_bolt_angles(4, 45.0)):
            v, w = REAR_BOLT_R * math.cos(a), REAR_BOLT_R * math.sin(a)
            J.bolt(reg, f"{ids['adapter']}-B{i + 1}", 4, C.P(uc + ADAPTER_T - 0.0045, v, w), C.d,
                   [(ids["adapter"], ADAPTER_T - 0.0045)], grade="12.9", nut="tapped", tapped_part=ids["engine"],
                   tapped_depth=0.008, step=STEP_ENGINE)
        for i, a in enumerate(_bolt_angles(4, 0.0)):
            v, w = SG_BOLT_R * math.cos(a), SG_BOLT_R * math.sin(a)
            J.bolt(reg, f"{ids['sg750']}-B{i + 1}", 4, C.P(uc + ADAPTER_T + SG750_T, v, w), C.d,
                   [(ids["sg750"], SG750_T)], grade="12.9", nut="tapped", tapped_part=ids["adapter"],
                   tapped_depth=ADAPTER_T - 0.0005, step=STEP_ENGINE)
        # spacer forward flange -> output spool (6 x M6 12.9, heads inside the spacer bore)
        for i, a in enumerate(_bolt_angles(6, 30.0)):
            v, w = SPACER["pcd_f"] * math.cos(a), SPACER["pcd_f"] * math.sin(a)
            J.bolt(reg, f"{ids['spacer']}-B{i + 1}", 6, C.P(C.u_flange - SPACER["t_fl_f"], v, w), C.eu,
                   [(ids["spacer"], SPACER["t_fl_f"])], grade="12.9", nut="tapped", tapped_part=ids["spool"],
                   tapped_depth=SPOOL_T, step=STEP_PROP, notes="Loctite 243 + safety wire")
        # propeller bolts: crush plate + hub + backplate into the spacer aft flange (6 x M6 12.9)
        h = float(C.S["propeller"]["hub_half_thickness"])
        for i, a in enumerate(_bolt_angles(6, 0.0)):
            v, w = SPACER["pcd_a"] * math.cos(a), SPACER["pcd_a"] * math.sin(a)
            J.bolt(reg, f"{ids['crush']}-B{i + 1}", 6, C.P(-h - CRUSH_T, v, w), C.eu,
                   [(ids["crush"], CRUSH_T), (ids["prop"], 2 * h), (ids["backplate"], BACKPLATE_T)], grade="12.9",
                   nut="tapped", tapped_part=ids["spacer"], tapped_depth=SPACER["t_fl_a"], step=STEP_PROP, hole_d=FINE[6],
                   torque_nm=9.0, notes="torque to the propeller maker's value (composite hub), re-torque after the "
                                        "first runs; safety wire")
        # exhaust port flanges: 2 x M6 each into the cylinder port ears (heat-resistant A4 / Inconel locking per
        # Limbach; modelled as 12.9 socket head)
        b = EXH_BOSS
        for side, sv in (("R", 1.0), ("L", -1.0)):
            for i, du in enumerate((-EXH_BOLT_DU, EXH_BOLT_DU)):
                J.bolt(reg, f"{ids['exh_' + side]}-B{i + 1}", 6, C.P(CYL_U + du, sv * EXH_PORT_V,
                                                                     b["w_face"] - EXH_FLANGE_T), C.ew,
                       [(ids["exh_" + side], EXH_FLANGE_T)], grade="12.9", nut="tapped", tapped_part=ids["engine"],
                       tapped_depth=0.012, step=STEP_ACC, notes="high-temperature anti-seize; exhaust gasket")
        # spinner centre screw at the tip into the stand-off
        tip = SPINNER_BASE_U - SPINNER_L
        J.bolt(reg, f"{ids['spinner']}-B1", 5, C.P(tip, 0, 0), C.eu, [(ids["spinner"], SPINNER_TIP_T)],
               head="ISO 7380", grade="A2-70", nut="tapped", tapped_part=ids["crush"], tapped_depth=STANDOFF_SOLID,
               step=STEP_PROP)


def register(reg: Registry, spec: dict) -> None:
    """Register the propulsion parts, the propeller joint and the fasteners (ARCHITECTURE.md producer contract)."""
    if part_number(GROUP, 500) in reg.parts:
        reg.note("propulsion: already registered, second call ignored")
        return
    _spinner_dims(spec)
    C = Ctx(reg, spec)
    R = _Reg(C)
    R.engine()
    R.propeller()
    R.exhaust()
    R.cooling()
    R.accessories()
    R.fasteners_core()
    R.fasteners_cooling()
    R.fasteners_accessories()
    if R.heat_protection():
        R.fasteners_heat()
