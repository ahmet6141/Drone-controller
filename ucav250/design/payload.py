"""Payload producer (``layout.part_numbers.payload``, YK250-PL-820..849): the retractable Trillium HD59 EO/IR turret of
the YK-250 HANCER civil surveillance UAV, its vibration-isolated mount, the ball-screw elevator, the two sliding
turret-bay doors with their Volz DA 22 rack drives, the aperture seal and the turret harness service loop.

What is built (every part with material + process + thickness, a parent, an assembly step, an explode vector and
contacts that really touch; fasteners through ``design/joints.py``):

* turret: Trillium HD59-LLVV envelope (datasheet ball d 147 mm, height 202 mm, 1.55 kg; components.yaml
  trillium_hd59_llvv): ball, pan / yoke stem (``payload.turret.stem_diameter``) and the top mounting base with four M4
  tapped holes (the vendor ICD is not public: bolt circle is an estimate); its interface cable to the carriage;
* isolated mount: 7075 adapter plate on the HD59 top, four cylindrical elastomer isolators (M4 female both ends) to
  the carriage plate (``payload.turret_mass_kg`` = HD59 1.55 + mount 0.20 kg);
* elevator (``layout.chassis.turret_elevator``): 7075 carriage (4 mm ring + isolator pads + arms, two corner tabs),
  four MGN12-class carriage blocks on the two MGN12-class profile rails (front-starboard and aft-port corners, 4 x M3
  per rail end into floating anchor nutplates on the chassis corner-angle seats), d 8 x 2 ball screw in the front-port
  corner turned by the BLDC worm-gear drive with spring brake and absolute encoder under the roof (6061 bearing
  housing on four potted M4 inserts of the roof sandwich, ``top_anchor``), lower journal bracket on the FS1110 web
  that is also the lower hard stop; the upper hard stop is the drive face (PU bumper on the nut); joint
  ``turret_elevator`` (rest = retracted);
* sliding doors (``layout.mechanisms`` turret_door_R/L, prismatic on rails inclined 32 deg): 2 mm CFRP doors shaped
  so that they close the 157 mm aperture behind the ring insert and stay inside the lower skin, the frame caps and
  under the free lower edge of the bay walls over the whole travel; straight grooved 6061 rails (6 x 6 mm, 2.6 mm
  groove on the door side) on integral stand-off lugs (ISO 7380 M3 into potted inserts of the FS1110 / FS1330 webs,
  spacing <= 60 mm, none under the wall corner / elevator-rail seat), the bonded slider tongue (PTFE-lined) running
  in the groove keeps the slider no higher than the rail so that the door set passes under the elevator rail seats
  and the free wall edge; the aft slider carries the rack; the inboard rail ends are relieved round the E180 growth
  envelope at full extension; Volz DA 22 (EQ-TDOORACT) on a 6061 bracket on the FS1330 aft face, 7075 pinion shaft
  through C-TDOOR-SHAFT; coupled sequence ``turret_extension`` (doors open before the turret moves);
* sealing: moulded silicone lip seal bonded inside the aperture ring insert (0.5 mm off the closed doors);
* harness: SLS PA12 guide trough on the starboard bay wall (4 x M3 into potted inserts) holding the rolling flat-cable
  service loop, ending in a MIL-DTL-38999 jam-nut receptacle through the wall.

Interfaces (``spec.layout`` only, never another module's geometry): ``payload.turret`` (ball, height, stem, stroke,
ball centres, door thickness / clearance, aperture ring), ``layout.chassis.turret_elevator`` (rails, screw axis, top
anchor), ``structures.sizing.turret`` (rail screws), ``layout.mechanisms`` (joints, sequence ``turret_extension``,
``door_outlines``), ``layout.stations`` (FS1110 / FS1330 faces, C-TDOOR-SHAFT cut-out), ``layout.chassis.members``
(M-TURRETWALL / M-TURRETROOF boxes), ``layout.shell.panels`` (P-TURRETRING thickness and outline, lower-skin layup),
``layout.systems.equipment`` (EQ-TURRET / EQ-TDOORACT boxes), ``layout.part_numbers`` and the OML
(``oml.fuselage_from_spec``). The chassis corner-angle seats of the rails carry no layout id: they are found by their
rail id in the registry name.

Manufacturing: machined parts CNC 7075-T651 / 6061-T6 (ISO 2768-mK; pinions hard-anodised 7075), doors OOA prepreg
(10 plies PW, 2.0 mm), trough SLS PA12; purchased units carry datasheet masses or labelled estimates; fasteners metric
(ISO 4762 / ISO 7380, nyloc / floating nutplate / potted insert), holes ISO 273 medium, edge distance >= 2 D metal /
polymer and >= 2.5 D composite. Module notes: ``docs/detail/payload.md``.
"""
from __future__ import annotations

import math

import numpy as np
from shapely.geometry import LineString, Point, Polygon, box as sbox
from shapely.ops import unary_union

from ..core import geom as G
from ..core.parts import Joint, Part, Registry, layup_props, mirror_part, part_number
from . import joints as J
from . import oml as O

GROUP = "payload"
MASS_ITEM_MOUNT = "eo_ir_turret_hd59_mount"          # spec.mass.payload_items (HD59 + isolated mount)
MASS_ITEM_MECH = "turret_lift_mechanism_doors"       # spec.mass.items, group systems (elevator, doors, drives)

# =====================================================================================================================
# module constants (detailing choices; interface values come from spec.layout / spec.payload)
# =====================================================================================================================
M7075, M6061, MPW, MPA12 = "al_7075_t651_plate", "al_6061_t6_sheet", "cfrp_pw_mtm45_as4", "pa12_sls"
P_CNC, P_PREG, P_SLS, P_BUY = "cnc_milling_metal", "prepreg_ooa_vacbag", "sls_pa12", "purchased"
STEP = 24                    # spec.assembly step 24: turret elevator, sliding doors and HD59
OV = 1e-4                    # boolean overlap of fused features
CT = 5e-5                    # declared contact faces 0.05 mm apart (no volume overlap, inside the 0.3 mm contact test)

# HD59 envelope (datasheet ball / height, payload.turret); base and bolt circle estimates (vendor ICD not public)
HD59_MASS = 1.55             # components.yaml trillium_hd59_llvv mass_kg (datasheet)
HD59_BASE_R, HD59_BASE_H = 0.060, 0.012
HD59_BOLT_R = 0.042
MOUNT_MASS = 0.20            # payload.sources.turret_mass_kg: isolator / mount 0.20 kg (adapter + isolators)
# isolated mount
ADAPT_T, ADAPT_HALF, ADAPT_HOLE_R = 0.004, 0.062, 0.030
ISO_R, ISO_H, ISO_XY = 0.010, 0.016, 0.048
# carriage
CARR_T = 0.004
CARR_ARM = 0.010              # half width of the carriage arms to the rail tabs and the ball nut
CARR_HOLE_R = 0.050           # central lightening / harness hole (>= 17 mm to the isolator screws)
TAB_T = 0.004
# MGN12-class rail / block (estimate geometry; layout rail_spec)
RAIL_W, RAIL_H = 0.012, 0.008
RAIL_CB_R, RAIL_CB_D = 0.003, 0.004     # counterbore for the ISO 4762 M3 heads
RAIL_PITCH, RAIL_END = 0.025, 0.0125
BLK_W, BLK_L, BLK_H, BLK_LIFT = 0.027, 0.0347, 0.014, 0.003
BLK_GAP_TOP, BLK_GAP_SIDE = 0.0002, 0.0005
BLK_TOP_Z = 0.0932           # retracted: upper block top (rail top end 0.095)
BLK_SCREW_DZ = (-0.0085, 0.0065)                 # tab screws about the block centre (head clear of the carriage plate)
RAIL_MASS_PER_M, BLK_MASS = 0.65, 0.034          # estimates (MGN12 class)
# ball screw / nut / drive
SCREW_R, JOURNAL_R, BORE_FIT = 0.004, 0.003, 0.00005
NUT_FL_R, NUT_FL_T, NUT_BODY_R, NUT_BORE_R, NUT_BOLT_R = 0.017, 0.004, 0.0075, 0.0042, 0.0105
SCREW_HOLE_R = 0.0045
GBX = (0.013, 0.022, 0.013, 0.020)               # gearbox: -dx, +dx, -dy(outboard), +dy(inboard) about the screw
GBX_Z = (0.076, 0.098)
MOTOR_R, MOTOR_DY, MOTOR_Z, MOTOR_X1 = 0.011, 0.010, 0.0869, 1.212
HOUS_T = 0.004
HOUS_SEAT_R = 0.0135        # bearing seat boss (626-class angular-contact pair, D 19 mm)
STOP_GAP = 0.0005            # hard stops 0.5 mm beyond the electrical limits
LB_BOSS_R = 0.0065
LB_DROP = 0.009              # bracket web / flange top below the boss top (stop face)
# doors
DOOR_T = 0.002               # 10 plies PW solid laminate (door_outlines thickness 4 mm = door + sliders envelope)
DOOR_GAP_CL = 0.0005         # half of the 1 mm gap between the two doors on the centre line
DOOR_FRAME_GAP = 0.020       # door edges 20 mm off the frame faces (rails and sliders between)
T_RING_CLR = 0.0001          # seal / door reference above the ring inner face
T_CAP = 0.0082               # frame-cap top above the OML (skin inset 6.5 mm + 1.6 mm cap laminate + 0.1)
Y_LAND = 0.0965              # skin land / frame caps start outboard of this |y| (ring alone inboard)
DOOR_CLR = 0.0010            # door outer face to the ring / skin / caps
# door rails and sliders
DR_W, DR_H = 0.006, 0.006    # rail bar: x width, height normal to the travel
GR_H, GR_D = 0.0026, 0.003   # guide groove in the door-side face of the rail (the slider tongue runs in it)
CB_D = 0.0018                # counterbore depth for the ISO 7380 M3 head in the stand-off lug (above the groove)
# door-rail stand-off lug: bolt line, top and bottom relative to the rail centre line (6.5 mm edge distance round the
# M3 above and below, the ISO 7380 head clear above the groove)
SO_BOLT, SO_TOP, SO_BOT = 0.0045, 0.011, -0.002
DR_X_GAP = 0.0011            # rail to bay-wall end post
SL_PAD = 0.002               # slider bonding pad thickness over the door top
SL_T, SL_CLR = 0.0022, 0.0002                    # slider tongue thickness, running clearance
SO_LEN = 0.012
SO_SPAN = 0.060              # largest stand-off spacing along a door rail (rail deflection under 9 g + suction)
# rack / pinion (module 0.5, teeth envelope only: pitch line / pitch cylinder, 0.2 mm apart)
PIN_R, PIN_W, RACK_W, RACK_D = 0.0195, 0.006, 0.006, 0.006
PIN_WEB = 0.002              # pinion web (recessed both sides of the hub, 3.5 mm tooth rim)
PIN_SHAFT_R = 0.004
# DA 22 bracket
BR_T = 0.0025                # DA 22 side flange (the frame flange is the 3 mm gap frame face - DA 22 case)
# seal ring
SEAL_RI_ADD, SEAL_W = 0.0015, 0.006
# harness
TR_X, TR_W, TR_T = 1.270, 0.040, 0.0015
TR_DEPTH, TR_SLOT = 0.012, 0.024
TR_Z = (-0.088, 0.065)
BAND_W, BAND_T = 0.016, 0.0013
RCPT_Z = 0.088


def _u(v) -> np.ndarray:
    v = np.asarray(v, float)
    return v / np.linalg.norm(v)


def box3(lo, hi) -> G.Mesh:
    lo, hi = np.asarray(lo, float), np.asarray(hi, float)
    return G.box(hi - lo, 0.5 * (lo + hi))


def prism_z(poly, z0, z1) -> G.Mesh:
    """Plan polygon (x, y) extruded from z0 to z1."""
    return G.extrude(poly, z1 - z0, origin=(0.0, 0.0, z0), u=(1, 0, 0), v=(0, 1, 0))


def prism_y(poly_zx, y0, y1) -> G.Mesh:
    """Polygon in (z, x) extruded from y0 to y1 (z x x = +y)."""
    return G.extrude(poly_zx, y1 - y0, origin=(0.0, y0, 0.0), u=(0, 0, 1), v=(1, 0, 0))


def prism_x(poly_yz, x0, x1) -> G.Mesh:
    """Polygon in (y, z) extruded from x0 to x1."""
    return G.extrude(poly_yz, x1 - x0, origin=(x0, 0.0, 0.0), u=(0, 1, 0), v=(0, 0, 1))


def clean(m: G.Mesh, tol: float = 1e-6) -> G.Mesh:
    return G.Mesh.from_manifold(m.to_manifold().simplify(tol))


def clip_cl(m: G.Mesh, y0: float = DOOR_GAP_CL) -> G.Mesh:
    """Starboard part trimmed at |y| = y0 (1 mm gap to its port mirror on the centre line)."""
    out = G.intersection(m, box3((-5.0, y0, -5.0), (5.0, 5.0, 5.0)))
    if out is None:
        raise ValueError("payload: centre-line trim removed the whole part")
    return out


def rrect(x0, x1, y0, y1, r):
    return sbox(x0 + r, y0 + r, x1 - r, y1 - r).buffer(r, 16)


# =====================================================================================================================
# context: interfaces from the spec
# =====================================================================================================================
class Ctx:
    def __init__(self, reg: Registry, spec: dict):
        self.reg, self.S = reg, spec
        L = self.L = spec["layout"]
        T = self.T = spec["payload"]["turret"]
        self.n0 = int(L["part_numbers"]["payload"][0])
        self.st = {s["id"]: s for s in L["stations"]}
        self.mem = {m["id"]: m for m in L["chassis"]["members"]}
        self.pan = {p["id"]: p for p in L["shell"]["panels"]}
        self.eq = {e["id"]: e for e in L["systems"]["equipment"]}
        self.jl = {j["name"]: j for j in L["mechanisms"]["joints"]}
        self.el = L["chassis"]["turret_elevator"]
        self.fus = O.fuselage_from_spec(spec)
        # bay
        self.xc = float(T["bay_center_x"])
        self.r_ball = 0.5 * float(T["ball_diameter"])
        self.h_tur = float(T["height"])
        self.r_stem = 0.5 * float(T["stem_diameter"])
        self.zc0 = float(self.el["ball_center_retracted"][2])
        self.stroke = float(self.el["stroke"])
        self.z_top = self.zc0 - self.r_ball + self.h_tur                 # HD59 mounting face, retracted
        self.r_open = self.r_ball + float(T["bay"]["aperture_ring"]["radial_clearance"])
        f1, f3 = self.st["FS1110"], self.st["FS1330"]
        self.xf1 = float(f1["x_faces"][1])                               # FS1110 aft face
        self.xf3 = float(f3["x_faces"][0])                               # FS1330 forward face
        self.xf3a = float(f3["x_faces"][1])                              # FS1330 aft face
        self.t_frame = float(f1["t"])
        self.cut_shaft = next(c for c in f3["cutouts"] if c["id"] == "C-TDOOR-SHAFT")
        wall = self.mem["M-TURRETWALL"]
        self.yw_in, self.yw_out = float(wall["box"][0][1]), float(wall["box"][1][1])
        self.t_wall = float(wall["section"]["t"])
        roof = self.mem["M-TURRETROOF"]
        self.z_roof = float(roof["box"][0][2])
        self.t_roof = float(roof["section"]["t"])
        ring = self.pan["P-TURRETRING"]
        self.t_ring = float(ring["thickness_m"])
        self.t_skin = layup_props(spec, self.pan["P-MID-LOWER"]["layup"])["thickness"]
        band = L["mechanisms"]["door_outlines"]["turret_door_R"]["band"]
        self.band_x = (float(band[0][0]), float(band[1][0]))
        # part ids of the interfaces
        self.p_fs1110, self.p_fs1330 = f1["part"], f3["part"]
        self.p_wall = {s: f"{wall['part']}-{s}" for s in "RL"}
        self.p_roof = roof["part"]
        self.p_ring = ring["part"]
        self.rails = {r["corner"]: r for r in self.el["rails"]}
        self.screw_xy = np.asarray(self.el["drive"]["screw_axis"][0][:2], float)
        self.z_rail = (float(self.rails["FR"]["line"][0][2]), float(self.rails["FR"]["line"][1][2]))
        jd = self.jl["turret_door_R"]
        self.d_o = np.asarray(jd["origin"], float)
        self.d_ax = _u(jd["axis"])                                       # (0, cos a, sin a)
        self.d_travel = float(jd["hi"])
        self.d_n = np.array([0.0, -self.d_ax[2], self.d_ax[1]])          # normal of the travel in the y-z plane (up)
        self.rail_screws = int(spec["structures"]["sizing"]["turret"]["rail_screws_per_rail_end"])
        self.da22 = self.eq["EQ-TDOORACT"]
        self._curves = {}

    # ------------------------------------------------------------------ ids
    def pid(self, k: int, side: str = "C") -> str:
        return part_number(GROUP, self.n0 + k, side)

    def anchor(self, corner: str) -> str:
        rid = self.rails[corner]["id"]
        hits = [p.id for p in self.reg.parts.values() if p.group == "chassis" and f"({rid})" in p.name]
        if len(hits) != 1:
            raise ValueError(f"payload: chassis corner-angle seat of {rid} not found in the registry")
        return hits[0]

    # ------------------------------------------------------------------ OML (lower starboard quadrant)
    def _curve(self, x: float, t: float):
        key = (round(float(x), 6), round(float(t), 7))
        if key not in self._curves:
            phi = np.linspace(0.5 * math.pi + 1e-3, math.pi, 2400)
            P = self.fus.point(np.full_like(phi, x), phi)
            y, z = P[::-1, 1], P[::-1, 2]                                # increasing y (keel -> chine)
            dy, dz = np.gradient(y), np.gradient(z)
            nn = np.hypot(dy, dz)
            ny, nz = -dz / nn, dy / nn                                   # inward (up, toward the centre line)
            yq, zq = y + t * ny, z + t * nz
            o = np.argsort(yq)
            self._curves[key] = (yq[o], zq[o])
        return self._curves[key]

    def z_in(self, x: float, y, t: float):
        """z of the lower surface offset inward by ``t`` (normal, in the x section) at |y|."""
        yq, zq = self._curve(x, t)
        return np.interp(np.abs(np.asarray(y, float)), yq, zq)

    def z_obst(self, x: float, y):
        """Lowest free z above the ring insert (inboard of Y_LAND) or the skin / frame caps (outboard)."""
        y = np.abs(np.asarray(y, float))
        return np.where(y < Y_LAND, self.z_in(x, y, self.t_ring + T_RING_CLR), self.z_in(x, y, T_CAP))

    # ------------------------------------------------------------------ travel frame of the doors (y-z plane)
    def uw(self, y, z):
        p = np.stack([np.asarray(y, float) - self.d_o[1], np.asarray(z, float) - self.d_o[2]], axis=-1)
        return p @ self.d_ax[1:], p @ self.d_n[1:]

    def yz(self, u, w):
        u, w = np.asarray(u, float), np.asarray(w, float)
        return (self.d_o[1] + u * self.d_ax[1] + w * self.d_n[1], self.d_o[2] + u * self.d_ax[2] + w * self.d_n[2])

    def prism_travel(self, poly_wx, u0, u1) -> G.Mesh:
        """Polygon in (w, x) extruded along the door travel from u0 to u1 (n x x = travel direction)."""
        y0, z0 = self.yz(u0, 0.0)
        return G.extrude(poly_wx, u1 - u0, origin=(0.0, float(y0), float(z0)), u=self.d_n, v=(1.0, 0.0, 0.0))


# =====================================================================================================================
# geometry: turret, mount, elevator
# =====================================================================================================================
def hd59_mesh(C: Ctx) -> G.Mesh:
    c = np.array([C.xc, 0.0, C.zc0])
    zt = C.z_top
    ball = G.sphere(C.r_ball, c, n=96)       # equator vertices on the ring-opening vertices
    z_stem = C.zc0 + math.sqrt(C.r_ball ** 2 - C.r_stem ** 2) - 0.004
    stem = G.cylinder(C.r_stem, (C.xc, 0.0, z_stem), (C.xc, 0.0, zt - HD59_BASE_H + OV), n=64)
    base = G.cylinder(HD59_BASE_R, (C.xc, 0.0, zt - HD59_BASE_H), (C.xc, 0.0, zt), n=72)
    # sensor window bezel (flat ring on the forward face of the ball, cosmetic within the ball envelope)
    return G.union([ball, stem, base])


def adapter_mesh(C: Ctx) -> G.Mesh:
    h = ADAPT_HALF
    poly = rrect(C.xc - h, C.xc + h, -h, h, 0.008).difference(Point(C.xc, 0.0).buffer(ADAPT_HOLE_R, 48))
    return prism_z(poly, C.z_top, C.z_top + ADAPT_T)


def iso_points(C: Ctx):
    return [(C.xc + sx * ISO_XY, sy * ISO_XY) for sx in (1, -1) for sy in (1, -1)]


def isolator_mesh(C: Ctx, x, y) -> G.Mesh:
    z0 = C.z_top + ADAPT_T
    return G.cylinder(ISO_R, (x, y, z0 + CT), (x, y, z0 + ISO_H - CT), n=40)


class Elev:
    """Elevator geometry in the retracted pose (rest)."""

    def __init__(self, C: Ctx):
        self.C = C
        self.z_c0 = C.z_top + ADAPT_T + ISO_H                            # carriage plate bottom
        self.z_c1 = self.z_c0 + CARR_T
        self.blocks = []                                                  # (corner, k, z0, z1)
        for corner in ("FR", "AL"):
            z1 = BLK_TOP_Z
            for k in range(2):
                self.blocks.append((corner, k, z1 - BLK_L, z1))
                z1 -= BLK_L + 0.001
        self.tab_z = (min(b[2] for b in self.blocks), BLK_TOP_Z)
        self.sx, self.sy = float(C.screw_xy[0]), float(C.screw_xy[1])
        self.nut_z0 = self.z_c1                                           # flange bottom on the plate top
        self.gbx_z0 = GBX_Z[0]
        self.nut_top = self.gbx_z0 - STOP_GAP                             # PU bumper top = upper hard stop
        self.trough_tab = (TR_X, C.yw_in - 0.0075)

    def rail_geom(self, corner):
        rl = self.C.rails[corner]
        x_r, y_r = float(rl["line"][0][0]), float(rl["line"][0][1])
        sg = 1.0 if y_r > 0 else -1.0
        seat = abs(y_r) + 0.00325                                         # chassis seat plane (layout rail axis + 3.25)
        return x_r, sg, seat

    def plate_poly(self):
        C = self.C
        xmin = C.xf1 + float(C.T["bay"]["wall_margin"]) + 0.0021
        xmax = C.xf3 - float(C.T["bay"]["wall_margin"]) - 0.0021
        ylim = C.yw_in - 0.0075
        # ring round the turret harness hole + one pad per isolator screw (8.8 mm edge distance for the M4)
        ring = Point(C.xc, 0.0).buffer(CARR_HOLE_R + 0.010, 96)
        parts = [ring] + [Point(x, y).buffer(0.011, 32) for x, y in iso_points(C)]
        for corner in ("FR", "AL"):
            x_r, sg, seat = self.rail_geom(corner)
            y_face = seat - BLK_H                                         # block top face |y| (tab outer face)
            parts.append(LineString([(C.xc, 0.0), (x_r, sg * (y_face - 0.012))]).buffer(CARR_ARM, 16))
            parts.append(sbox(x_r - 0.5 * BLK_W, min(sg * (y_face - 0.016), sg * y_face),
                              x_r + 0.5 * BLK_W, max(sg * (y_face - 0.016), sg * y_face)))
        parts.append(LineString([(C.xc, 0.0), (self.sx, self.sy)]).buffer(CARR_ARM, 16))
        parts.append(Point(self.sx, self.sy).buffer(NUT_FL_R, 64))
        tx, ty = self.trough_tab
        parts.append(LineString([(C.xc + 0.03, 0.04), (tx, ty - 0.006)]).buffer(0.006, 16))
        parts.append(sbox(tx - 0.006, ty - 0.012, tx + 0.006, ty))
        poly = unary_union(parts).intersection(sbox(xmin, -ylim, xmax, ylim))
        poly = poly.difference(Point(C.xc, 0.0).buffer(CARR_HOLE_R, 48))
        poly = poly.difference(Point(self.sx, self.sy).buffer(SCREW_HOLE_R, 32))      # ball-screw clearance
        return poly

    def carriage_mesh(self):
        C = self.C
        parts = [prism_z(self.plate_poly(), self.z_c0, self.z_c1)]
        for corner in ("FR", "AL"):
            x_r, sg, seat = self.rail_geom(corner)
            yb = seat - BLK_H - CT                                        # tab outer face |y| (block top - CT)
            ya = yb + CT - TAB_T
            y0, y1 = sorted((sg * ya, sg * yb))
            parts.append(box3((x_r - 0.5 * BLK_W, y0, self.tab_z[0]), (x_r + 0.5 * BLK_W, y1, self.tab_z[1])))
        return clean(G.union(parts))

    def block_mesh(self, corner, z0, z1):
        x_r, sg, seat = self.rail_geom(corner)
        y_top = seat - BLK_H
        y_bot = seat - BLK_LIFT
        body = box3((x_r - 0.5 * BLK_W, y_top, z0), (x_r + 0.5 * BLK_W, y_bot, z1))
        slot = box3((x_r - 0.5 * RAIL_W - BLK_GAP_SIDE, seat - RAIL_H - BLK_GAP_TOP, z0 - 0.001),
                    (x_r + 0.5 * RAIL_W + BLK_GAP_SIDE, seat + 0.001, z1 + 0.001))
        m = G.difference(body, [slot])
        return m if sg > 0 else m.mirrored_y()

    def rail_mesh(self, corner):
        x_r, sg, seat = self.rail_geom(corner)
        z0, z1 = self.C.z_rail
        m = box3((x_r - 0.5 * RAIL_W, seat - RAIL_H, z0), (x_r + 0.5 * RAIL_W, seat - CT, z1))
        cbs = [G.cylinder(RAIL_CB_R, (x_r, seat - RAIL_H - 0.001, z), (x_r, seat - RAIL_H + RAIL_CB_D, z), n=32)
               for z in self.rail_screw_z()]
        m = G.difference(m, cbs)
        return m if sg > 0 else m.mirrored_y()

    def rail_screw_z(self):
        z0, z1 = self.C.z_rail
        n = self.C.rail_screws
        return [z0 + RAIL_END + k * RAIL_PITCH for k in range(n)] + [z1 - RAIL_END - k * RAIL_PITCH for k in range(n)]

    # ---------------------------------------------------------------- screw, nut, drive, housing, lower bracket
    def screw_mesh(self):
        x, y = self.sx, self.sy
        zl = self.lb_z()[0]
        parts = [G.cylinder(JOURNAL_R, (x, y, zl + 0.001), (x, y, self.lb_z()[1] + 0.002), n=24),
                 G.cylinder(SCREW_R, (x, y, self.lb_z()[1] + 0.001), (x, y, GBX_Z[1] - 0.002), n=32),
                 G.cylinder(JOURNAL_R, (x, y, GBX_Z[1] - 0.003), (x, y, self.C.z_roof - 0.0006), n=24)]
        return G.union(parts)

    def nut_mesh(self):
        x, y = self.sx, self.sy
        fl = G.cylinder(NUT_FL_R, (x, y, self.nut_z0 + CT), (x, y, self.nut_z0 + NUT_FL_T), n=64)
        body = G.cylinder(NUT_BODY_R, (x, y, self.nut_z0 + NUT_FL_T - OV), (x, y, self.nut_top), n=48)
        bore = G.cylinder(NUT_BORE_R, (x, y, self.nut_z0 - 0.001), (x, y, self.nut_top + 0.001), n=32)
        return G.difference(G.union([fl, body]), [bore])

    def gbx_box(self):
        x, y = self.sx, self.sy
        return (x - GBX[0], y - GBX[2], GBX_Z[0]), (x + GBX[1], y + GBX[3], GBX_Z[1])

    def drive_mesh(self):
        x, y = self.sx, self.sy
        lo, hi = self.gbx_box()
        gbx = box3(lo, (hi[0], hi[1], hi[2] - CT))
        ym = y + MOTOR_DY
        motor = G.cylinder(MOTOR_R, (hi[0] - OV, ym, MOTOR_Z), (MOTOR_X1 - 0.012, ym, MOTOR_Z), n=48)
        brake = G.cylinder(MOTOR_R - 0.0005, (MOTOR_X1 - 0.012 - OV, ym, MOTOR_Z), (MOTOR_X1, ym, MOTOR_Z), n=48)
        bore = G.cylinder(SCREW_R + 0.0002, (x, y, lo[2] - 0.001), (x, y, hi[2] + 0.001), n=32)
        return G.difference(G.union([gbx, motor, brake]), [bore])

    def housing_geom(self):
        x, y = self.sx, self.sy
        lo, hi = self.gbx_box()
        zr = self.C.z_roof
        fl_lo = (x - 0.020, y - 0.018, zr - HOUS_T)
        fl_hi = (x + 0.0345, y + 0.040, zr - CT)
        bolts = [(x - 0.0115, y + 0.0315), (x + 0.0165, y + 0.0315), (x + 0.026, y + 0.0075),
                 (x + 0.026, y - 0.0095)]
        return fl_lo, fl_hi, bolts

    def housing_mesh(self):
        x, y = self.sx, self.sy
        lo, hi = self.gbx_box()
        fl_lo, fl_hi, _b = self.housing_geom()
        flange = box3(fl_lo, fl_hi)
        z0, z1 = hi[2], fl_lo[2] + OV
        seat = G.cylinder(HOUS_SEAT_R, (x, y, z0), (x, y, z1), n=48)                  # bearing seat (626 pair)
        lug = box3((x - 0.0135, hi[1] - 0.0135, z0), (x + 0.0135, hi[1] + 0.0005, z1))  # gearbox screw lugs
        bore = G.cylinder(JOURNAL_R + BORE_FIT, (x, y, hi[2] - 0.001), (x, y, fl_hi[2] + 0.001), n=32)
        return G.difference(G.union([flange, seat, lug]), [bore])

    def lb_z(self):
        """Lower journal bracket: boss z range (bottom, top = lower hard stop under the carriage plate)."""
        z_top = self.z_c0 - self.C.stroke - STOP_GAP
        return z_top - 0.034, z_top

    def lb_mesh(self):
        C = self.C
        x, y = self.sx, self.sy
        zb, zt = self.lb_z()
        xf = C.xf1
        zw = zt - LB_DROP                                                 # web / flange below the nut-flange nylocs
        flange = box3((xf + CT, y - 0.020, zb - 0.020), (xf + 0.004, y + 0.020, zw))
        web = box3((xf + 0.004 - OV, y - 0.005, zb), (x, y + 0.005, zw))
        boss = G.cylinder(LB_BOSS_R, (x, y, zb), (x, y, zt), n=40)
        bore = G.cylinder(JOURNAL_R + BORE_FIT, (x, y, zb - 0.001), (x, y, zt + 0.001), n=24)
        return G.difference(G.union([flange, web, boss]), [bore])

    def lb_bolts(self):
        zb, zt = self.lb_z()
        y = self.sy
        return [(y - 0.0115, zb - 0.0115), (y + 0.0115, zt - LB_DROP - 0.0085)]


# =====================================================================================================================
# geometry: doors, rails, sliders, rack, drive
# =====================================================================================================================
class Doors:
    """Starboard door set in the closed pose (rest); port = mirror."""

    def __init__(self, C: Ctx):
        self.C = C
        self.x0 = C.xf1 + DOOR_FRAME_GAP
        self.x1 = C.xf3 - DOOR_FRAME_GAP
        do = C.L["mechanisms"]["door_outlines"]["turret_door_R"]["outline_closed"]
        arc = float(do[2][1]) - float(do[0][1])                           # closed width along the skin
        self.y1 = self._y_of_arc(C.xc, arc)
        self.y0 = DOOR_GAP_CL
        self.xs = np.linspace(self.x0, self.x1, 34)
        self.ys = np.linspace(self.y0, self.y1, 40)
        self.Z = np.array([self.z_outer_row(x, self.ys) for x in self.xs])

    def _y_of_arc(self, x, arc):
        C = self.C
        ys = np.linspace(0.0, 0.2, 2001)
        zs = C.z_in(x, ys, 0.0)
        s = np.r_[0.0, np.cumsum(np.hypot(np.diff(ys), np.diff(zs)))]
        return float(np.interp(arc, s, ys))

    def z_outer_row(self, x, ys):
        """Door outer face: above the ring at rest and above ring / skin / caps over the whole travel."""
        C = self.C
        ca, sa = C.d_ax[1], C.d_ax[2]
        z = C.z_obst(x, ys) + DOOR_CLR
        for s in np.linspace(0.0, C.d_travel, 97):
            z = np.maximum(z, C.z_obst(x, ys + s * ca) - s * sa + DOOR_CLR)
        return z

    def z_under_min(self, x, y):
        """Lowest door outer-face z over (x, y) during the whole travel (closed door at s = 0, then slid by s)."""
        C = self.C
        ca, sa = C.d_ax[1], C.d_ax[2]
        row = np.array([float(np.interp(x, self.xs, self.Z[:, k])) for k in range(len(self.ys))])
        y = max(float(y), self.y0)
        z = np.inf
        for s in np.linspace(0.0, C.d_travel, 121):
            yq = y - s * ca
            if self.y0 <= yq <= self.y1:
                z = min(z, float(np.interp(yq, self.ys, row)) + s * sa)
        return z

    def grids(self, xs=None, ys=None, Z=None):
        xs, ys, Z = (self.xs, self.ys, self.Z) if xs is None else (xs, ys, Z)
        X, Y = np.meshgrid(xs, ys, indexing="ij")
        P = np.stack([X, Y, Z], axis=-1)
        dPx = np.gradient(P, axis=0)
        dPy = np.gradient(P, axis=1)
        N = np.cross(dPx, dPy)
        N /= np.linalg.norm(N, axis=2, keepdims=True)
        return P, N

    def door_mesh(self):
        P, N = self.grids()
        return clip_cl(G.shell_from_grid(P, DOOR_T, inward=N))

    def top_slab(self):
        """Solid from the door top face down 60 mm, over the door plan grown by 10 mm (so that its side faces, which
        follow the surface normal, stay outside the slider columns): cutter for the bonded sliders / rack feet."""
        xs = np.r_[self.x0 - 0.010, self.xs, self.x1 + 0.010]
        ys = np.r_[self.ys, self.y1 + 0.010]
        Z = np.array([self.z_outer_row(x, ys) for x in xs])
        P, N = self.grids(xs, ys, Z)
        Ptop = P + DOOR_T * N
        Pd, Nd = self.grids()
        Ptop[1:-1, :-1] = Pd + DOOR_T * Nd                               # identical to the door top inside its plan
        return G.shell_from_grid(Ptop, 0.06, inward=-N)

    def column(self, x0, x1):
        return box3((max(x0, self.x0) + OV, self.y0 + OV, -0.4), (min(x1, self.x1) - OV, self.y1 - OV, 0.0))

    def z_top_at(self, x, y):
        i = int(np.argmin(np.abs(self.xs - x)))
        return float(np.interp(y, self.ys, self.Z[i])) + DOOR_T / math.cos(math.atan(0.6))

    # ---------------------------------------------------------------- door rails
    def rail_x(self, which):
        C = self.C
        if which == "F":                                                  # aft of the wall end post (door band)
            xa = C.band_x[0] + DR_X_GAP
            return xa, xa + DR_W
        xb = C.band_x[1] - DR_X_GAP
        return xb - DR_W, xb

    def slider_u(self):
        """u range of the sliders on the door (closed): inside the door plan with 6 mm margin."""
        C = self.C
        x = self.x0 + 0.003
        ya, yb = self.y0 + 0.006, self.y1 - 0.024
        ua = float(C.uw(ya, self.z_top_at(x, ya))[0])
        ub = float(C.uw(yb, self.z_top_at(x, yb))[0])
        return ua, ub

    def rail_w(self, which):
        """Rail centre line (w): lowest line whose slider lower lip clears the ring / skin / caps over the travel."""
        C = self.C
        xa, xb = self.rail_x(which)
        ua, ub = self.slider_u()
        need = -1.0
        lip = 0.5 * DR_H + 0.0005                                         # rail bottom (tongue is higher)
        for x in np.linspace(xa - 0.002, xb + 0.003, 5):
            for u in np.linspace(ua - 0.004, ub + C.d_travel + 0.004, 160):
                # point on the lip bottom for a trial w: y, z linear in w -> solve z(w) >= z_obst(y(w)) by iteration
                w = -0.01
                for _ in range(200):
                    y, z = C.yz(u, w - lip)
                    zo = float(C.z_obst(x, y))
                    if z >= zo:
                        break
                    w += 0.0005
                need = max(need, w)
        # stand-offs bottom too (13 mm high)
        return need + 0.0003

    def rail_span(self, which):
        ua, ub = self.slider_u()
        return ua - 0.006, ub + self.C.d_travel + 0.006

    def standoff_u(self, which, w_r):
        """Stand-off stations: avoid the bay-wall line (end post), the rail-anchor corner, and keep the M3 >= 2.5 D
        from the frame web edge (OML inset 6.5 mm + 7.5 mm + margin)."""
        C = self.C
        u0, u1 = self.rail_span(which)
        x = C.xf1 - 0.5 * C.t_frame if which == "F" else C.xf3 + 0.5 * C.t_frame
        bkt = self.bracket_geom()[2]
        ok = []
        for u in np.linspace(u0 + 0.008, u1 - 0.008, 400):
            y, z = C.yz(u, w_r + SO_BOLT)
            if 0.075 < y < 0.13:                     # under the elevator rail / its corner seat and the bay-wall edge
                continue
            if z < float(C.z_in(x, y, 0.0065)) + 0.0075 + 0.003:
                continue
            if which == "A" and min(math.hypot(y - yb, z - zb) for yb, zb in bkt) < 0.016:
                continue                                     # potted inserts of the DA 22 bracket in the same web
            ok.append(u)
        if len(ok) < 2:
            raise ValueError("payload: no stand-off station for the door rail")
        ok = np.array(ok)
        # both ends of every free segment (the crossing band under the wall corner splits the rail), plus mid-points
        # so that no span exceeds SO_SPAN
        cuts = np.where(np.diff(ok) > 0.005)[0]
        segs = np.split(ok, cuts + 1)
        picks = []
        for sg_ in segs:
            a, b = float(sg_[0]), float(sg_[-1])
            k = max(1, int(math.ceil((b - a) / SO_SPAN)))
            picks += [a + (b - a) * i / k for i in range(k + 1)] if b - a > SO_LEN else [0.5 * (a + b)]
        return picks

    def rail_mesh(self, which, w_r):
        C = self.C
        xa, xb = self.rail_x(which)
        u0, u1 = self.rail_span(which)
        if which == "F":
            gr = sbox(w_r - 0.5 * GR_H, xb - GR_D, w_r + 0.5 * GR_H, xb)
        else:
            gr = sbox(w_r - 0.5 * GR_H, xa, w_r + 0.5 * GR_H, xa + GR_D)
        bar = C.prism_travel(sbox(w_r - 0.5 * DR_H, xa, w_r + 0.5 * DR_H, xb).difference(gr), u0, u1)
        parts = [bar]
        xf = C.xf1 if which == "F" else C.xf3
        for u in self.standoff_u(which, w_r):
            lug = sbox(w_r + 0.5 * DR_H - OV, xa, w_r + SO_TOP, xb)          # above the bar, over the groove
            if which == "F":
                pol = sbox(w_r + SO_BOT, xf + CT, w_r + SO_TOP, xa + OV).union(lug)
            else:
                pol = sbox(w_r + SO_BOT, xb - OV, w_r + SO_TOP, xf - CT).union(lug)
            parts.append(C.prism_travel(pol, u - 0.5 * SO_LEN, u + 0.5 * SO_LEN))
        m = G.union(parts)
        # counterbores for the ISO 7380 M3 heads under the groove bottom
        cbs = []
        for u in self.standoff_u(which, w_r):
            y, z = C.yz(u, w_r + SO_BOLT)
            if which == "F":
                cbs.append(G.cylinder(0.0031, (xb - CB_D, y, z), (xb + 0.001, y, z), n=32))
            else:
                cbs.append(G.cylinder(0.0031, (xa - 0.001, y, z), (xa + CB_D, y, z), n=32))
        # inboard rail ends relieved for the E180 growth ball at full extension (0.5 mm round the envelope)
        relief = e180_envelope(C.S, C.stroke, grow=0.0005)
        return clip_cl(G.difference(m, cbs + [relief]))

    # ---------------------------------------------------------------- sliders and rack
    def slider_mesh(self, which, w_r, slab, rack=None):
        C = self.C
        xa, xb = self.rail_x(which)
        ua, ub = self.slider_u()
        t0, t1 = w_r - 0.5 * GR_H + SL_CLR, w_r + 0.5 * GR_H - SL_CLR      # tongue in the rail groove
        if which == "F":
            tongue = sbox(t0, xb - GR_D + SL_CLR, t1, self.x0 + 0.002)
            riser_x = (self.x0 + 0.0005, self.x0 + 0.014)
        else:
            tongue = sbox(t0, self.x1 - 0.002, t1, xa + GR_D - SL_CLR)
            riser_x = (self.x1 - 0.014, self.x1 - 0.0005)
        c_part = G.difference(C.prism_travel(tongue, ua, ub), [slab])
        riser = C.prism_travel(sbox(w_r - 0.045, riser_x[0], t1, riser_x[1]), ua, ub)
        riser = G.intersection(G.difference(riser, [slab]), self.column(*riser_x))
        if riser is None:
            raise ValueError("payload: door slider riser does not reach the door")
        # bonding pad: at least 2 mm of slider over the door top under the whole riser (no feathered edge where the
        # door top approaches the tongue)
        P, N = self.grids()
        pad = G.shell_from_grid(P + DOOR_T * N, SL_PAD, inward=N)
        pad = G.intersection(G.intersection(pad, self.column(*riser_x)),
                             C.prism_travel(sbox(-0.2, riser_x[0], 0.2, riser_x[1]), ua, ub))
        if pad is not None:
            riser = G.union([riser, pad])
        parts = [c_part, riser]
        if rack is not None:
            parts += rack
        return clip_cl(clean(G.union(parts)))

    def rack_geom(self):
        """Pinion pitch circle (y, z) centre on the C-TDOOR-SHAFT cut-out centre; rack top w (0.2 mm below the pitch
        line) and the rack u range (closed) for the 0.12 m travel."""
        C = self.C
        cs = C.cut_shaft
        yc, zc = 0.5 * sum(cs["y"]), 0.5 * sum(cs["z"])
        uc, wc = (float(v) for v in C.uw(yc, zc))
        w_top = wc - PIN_R - 0.0002
        return (yc, zc), uc, w_top, (uc - C.d_travel - 0.006, uc + 0.006)

    def rack_parts(self, slab):
        C = self.C
        (yc, zc), uc, w_top, (u0, u1) = self.rack_geom()
        xa, xb = self.pinion_x()
        beam = G.difference(C.prism_travel(sbox(w_top - RACK_D, xa, w_top, xb), u0, u1), [slab])
        ua, ub = self.slider_u()
        feet = C.prism_travel(sbox(w_top - 0.045, xa, w_top - RACK_D + OV, xb), max(u0, ua), min(u1, ub))
        feet = G.intersection(G.difference(feet, [slab]), self.column(xa, xb))
        return [beam] + ([feet] if feet is not None else [])

    def pinion_x(self):
        xa_rail, _ = self.rail_x("A")
        xb = xa_rail - SL_CLR - 0.0025 - 0.0045
        return xb - RACK_W, xb

    def pinion_mesh(self):
        C = self.C
        (yc, zc), *_ = self.rack_geom()
        xa, xb = self.pinion_x()
        x_end = float(C.da22["box"][0][0])
        pin = G.cylinder(PIN_R, (xa, yc, zc), (xb, yc, zc), n=96)
        rec = [G.cylinder(PIN_R - 0.0035, (xa - 0.001, yc, zc), (0.5 * (xa + xb) - 0.5 * PIN_WEB, yc, zc), n=64),
               G.cylinder(PIN_R - 0.0035, (0.5 * (xa + xb) + 0.5 * PIN_WEB, yc, zc), (xb + 0.001, yc, zc), n=64)]
        pin = G.union([G.difference(pin, rec), G.cylinder(0.008, (xa, yc, zc), (xb, yc, zc), n=48)])
        hub = G.cylinder(0.008, (xb - OV, yc, zc), (xb + 0.004, yc, zc), n=48)
        shaft = G.cylinder(PIN_SHAFT_R, (xb + 0.004 - OV, yc, zc), (x_end - CT, yc, zc), n=32)
        return G.union([pin, hub, shaft])

    # ---------------------------------------------------------------- DA 22 and bracket
    def da22_mesh(self):
        C = self.C
        lo, hi = (np.asarray(v, float) for v in C.da22["box"])
        return box3(lo, hi)

    def bracket_geom(self):
        C = self.C
        lo, hi = (np.asarray(v, float) for v in C.da22["box"])
        xa = C.xf3a
        (yc, zc), *_ = self.rack_geom()
        fl = ((xa + CT, yc - 0.023, zc - 0.039), (lo[0] - CT, yc + 0.024, zc + 0.038))
        zm = 0.5 * (lo[2] + hi[2])
        side = ((xa + CT, lo[1] - BR_T, zm - 0.012), (lo[0] + 0.062, lo[1] - CT, zm + 0.012))
        frame_bolts = [(yc - 0.010, zc - 0.0305), (yc, zc + 0.0295), (yc + 0.006, zc - 0.028),
                       (yc + 0.015, zc + 0.0295)]
        da_bolts = [(lo[0] + 0.0156, 0.5 * (lo[2] + hi[2])), (lo[0] + 0.0516, 0.5 * (lo[2] + hi[2]))]
        return fl, side, frame_bolts, da_bolts

    def bracket_mesh(self):
        (yc, zc), *_ = self.rack_geom()
        fl, side, _fb, _db = self.bracket_geom()
        m = G.union([box3(*fl), box3(side[0], side[1])])
        bore = G.cylinder(PIN_SHAFT_R + BORE_FIT, (fl[0][0] - 0.001, yc, zc), (fl[1][0] + 0.001, yc, zc), n=32)
        # lower edge trimmed parallel to the skin, 1 mm above the FS1330 aft cap (the frame cap runs aft on the skin)
        C = self.C
        x0, x1 = fl[0][0] - 0.001, side[1][0] + 0.001
        ys = np.linspace(fl[0][1] - 0.002, fl[1][1] + 0.002, 24)
        zc_ = np.maximum(C.z_in(x0, ys, T_CAP + 0.001), C.z_in(x1, ys, T_CAP + 0.001))
        keep = prism_x(Polygon([(float(y), float(z)) for y, z in zip(ys, zc_)]
                               + [(float(ys[-1]), 0.0), (float(ys[0]), 0.0)]), x0, x1)
        return G.intersection(G.difference(m, [bore]), keep)


# =====================================================================================================================
# geometry: seal and harness
# =====================================================================================================================
def seal_mesh(C: Ctx, D: Doors) -> G.Mesh:
    ri, ro = C.r_open + SEAL_RI_ADD, C.r_open + SEAL_RI_ADD + SEAL_W
    th = np.linspace(0.0, 2 * math.pi, 160, endpoint=False)
    rs = np.linspace(ri, ro, 4)
    P = np.zeros((len(th), len(rs), 3))
    Tt = np.zeros((len(th), len(rs)))
    for i, t in enumerate(th):
        for j, r in enumerate(rs):
            x, y = C.xc + r * math.cos(t), r * math.sin(t)
            zb = float(C.z_in(x, y, C.t_ring + T_RING_CLR))
            zd = D.z_under_min(x, abs(y))
            P[i, j] = (x, y, zb)
            Tt[i, j] = max(zd - 0.0005 - zb, 0.0003)
    N = np.zeros_like(P)
    N[..., 2] = 1.0
    return G.shell_from_grid(P, Tt, inward=N, close_u=True)


def trough_mesh(C: Ctx) -> G.Mesh:
    x0, x1 = TR_X - 0.5 * TR_W, TR_X + 0.5 * TR_W
    yb = C.yw_in - CT
    yf = yb - TR_DEPTH
    z0, z1 = TR_Z
    s0, s1 = TR_X - 0.5 * TR_SLOT, TR_X + 0.5 * TR_SLOT
    parts = [box3((x0, yb - TR_T, z0), (x1, yb, z1)),                    # back (on the wall)
             box3((x0, yf, z0), (x0 + TR_T, yb - TR_T + OV, z1)),        # side walls
             box3((x1 - TR_T, yf, z0), (x1, yb - TR_T + OV, z1)),
             box3((x0 + TR_T - OV, yf, z0), (s0, yf + TR_T, z1)),        # front lips
             box3((s1, yf, z0), (x1 - TR_T + OV, yf + TR_T, z1)),
             box3((x0 + TR_T - OV, yf + TR_T - OV, z0), (x1 - TR_T + OV, yb - TR_T + OV, z0 + TR_T))]   # bottom cap
    return clean(G.union(parts))


def trough_bolts(C: Ctx):
    z0, z1 = TR_Z
    return [(TR_X + sx * (0.5 * TR_W - 0.0065), z) for sx in (-1, 1) for z in (z0 + 0.018, z1 - 0.015)]


def loop_mesh(C: Ctx) -> G.Mesh:
    """Flat flexible cable (fixed leg of the rolling loop) + 90 deg backshell + MIL-DTL-38999 plug / jam-nut receptacle
    through the starboard wall."""
    yb = C.yw_in - CT - TR_T                                              # trough back inner face
    z0, z1 = TR_Z
    band = box3((TR_X - 0.5 * BAND_W, yb - CT - BAND_T, z0 + TR_T + 0.0005), (TR_X + 0.5 * BAND_W, yb - CT, 0.066))
    shell = box3((TR_X - 0.5 * BAND_W, C.yw_in - 0.013, 0.0645), (TR_X + 0.5 * BAND_W, yb - CT - OV, 0.080))
    coup = G.cylinder(0.0115, (TR_X, C.yw_in - 0.014, RCPT_Z), (TR_X, C.yw_in - 0.002 - CT, RCPT_Z), n=48)
    flange = box3((TR_X - 0.013, C.yw_in - 0.002 - OV, RCPT_Z - 0.013), (TR_X + 0.013, C.yw_in - CT, RCPT_Z + 0.013))
    barrel = G.cylinder(0.0090, (TR_X, C.yw_in - 0.001, RCPT_Z), (TR_X, C.yw_out + 0.0004, RCPT_Z), n=48)
    jam = G.cylinder(0.0125, (TR_X, C.yw_out + CT, RCPT_Z), (TR_X, C.yw_out + 0.003, RCPT_Z), n=6)
    return G.union([band, shell, coup, flange, barrel, jam])


def pigtail_mesh(C: Ctx, E: Elev) -> G.Mesh:
    r = 0.004
    zc = E.z_c1 + r + CT
    tx, ty = E.trough_tab
    conn = G.cylinder(0.007, (C.xc, 0.0, C.z_top + CT), (C.xc, 0.0, C.z_top + 0.010), n=40)
    pts = [(C.xc, 0.0, C.z_top + 0.008), (C.xc, 0.0, zc - 0.008), (C.xc + 0.004, 0.003, zc),
           (C.xc + 0.015, 0.025, zc), (tx, 0.072, zc), (tx, ty - 0.001, zc)]
    path = _fillet_path(np.asarray(pts, float), 0.006, 8)
    cable = G.sweep_circle(path, r, n=20)
    return G.union([conn, cable])


def _fillet_path(P, rad, k):
    out = [P[0]]
    for i in range(1, len(P) - 1):
        a, b, c = P[i - 1], P[i], P[i + 1]
        da, dc = _u(a - b), _u(c - b)
        la, lc = np.linalg.norm(a - b), np.linalg.norm(c - b)
        r_ = min(rad, 0.45 * la, 0.45 * lc)
        p0, p1 = b + r_ * da, b + r_ * dc
        for t in np.linspace(0.0, 1.0, k):
            out.append((1 - t) ** 2 * p0 + 2 * (1 - t) * t * b + t ** 2 * p1)
    out.append(P[-1])
    Q = [out[0]]
    for q in out[1:]:
        if np.linalg.norm(q - Q[-1]) > 1e-5:
            Q.append(q)
    return np.asarray(Q)


# =====================================================================================================================
# registration
# =====================================================================================================================
class Build:
    def __init__(self, reg: Registry, spec: dict):
        self.reg, self.spec = reg, spec
        self.C = C = Ctx(reg, spec)
        self.E = Elev(C)
        self.D = Doors(C)
        self.mirror_ids: list[str] = []
        self._mem = {}

    def M(self, key, fn):
        if key not in self._mem:
            self._mem[key] = fn()
        return self._mem[key]

    def add(self, k, side, name, name_tr, material, process, mesh_fn, *, thickness=None, parent, contacts=(),
            joint=None, explode=(0.0, 0.0, -0.2), purchased=False, vendor="", mass_kg=None, color="chassis",
            notes="", mirror=False, mass_item=MASS_ITEM_MECH):
        """Register a payload part. ``mass_item``: the spec.mass item it is booked to (analysis/mass.py budget group):
        the HD59 and its isolated mount are the payload item eo_ir_turret_hd59_mount, everything else (elevator,
        doors, drives, seal, service loop) is the systems item turret_lift_mechanism_doors (layout.mass_placement)."""
        pid = self.C.pid(k, side)
        p = Part(id=pid, name=name, name_tr=name_tr, group=GROUP, material=material, process=process,
                 mesh_fn=mesh_fn, thickness=thickness, purchased=purchased, vendor=vendor, mass_kg=mass_kg,
                 side=side, joint=joint, parent=parent, step=STEP, explode=tuple(explode), contacts=tuple(contacts),
                 color=color, notes=notes, mass_item=mass_item)
        self.reg.add(p)
        if mirror:
            self.mirror_ids.append(pid)
        return pid

    # ---------------------------------------------------------------- joints + sequence
    def joints(self):
        for name in ("turret_elevator", "turret_door_R", "turret_door_L"):
            j = self.C.jl[name]
            if name in self.reg.joints:
                continue
            self.reg.add_joint(Joint(name=name, kind=j["kind"], origin=j["origin"], axis=j["axis"], lo=float(j["lo"]),
                                     hi=float(j["hi"]), rest=float(j["rest"]), prop=j.get("prop", ""),
                                     scale=float(j.get("scale", 1.0)), parent=j.get("parent"),
                                     expr=j.get("expr", ""), notes=str(j.get("notes", ""))[:300]))

    def sequence(self):
        seq = self.C.L["mechanisms"]["sequences"]["turret_extension"]
        self.reg.add_sequence("turret_extension", [{k: float(v) for k, v in st.items()} for st in seq["states"]])

    # ---------------------------------------------------------------- turret, mount, elevator
    def turret(self):
        C, E, M = self.C, self.E, self.M
        jt = "turret_elevator"
        tur, adp, car = C.pid(0), C.pid(1), C.pid(2)
        self.add(0, "C", "EO/IR turret Trillium HD59-LLVV", "EO/IR taret Trillium HD59-LLVV", "purchased", P_BUY,
                 lambda: M("hd59", lambda: hd59_mesh(C)), parent=adp, contacts=(adp,), joint=jt, purchased=True,
                 vendor="Trillium Engineering HD59-LLVV (datasheet: ball d 147 mm, height 202 mm, 1.55 kg; mounting "
                        "ICD not public, base / bolt circle estimate)", mass_kg=HD59_MASS, color="payload",
                 explode=(0.0, 0.0, -0.45), mass_item=MASS_ITEM_MOUNT)
        m_iso = 0.012
        self.add(1, "C", "turret adapter (isolator) plate", "taret adaptör (izolatör) plakası", M7075, P_CNC,
                 lambda: M("adp", lambda: adapter_mesh(C)), thickness=ADAPT_T, parent=car, joint=jt,
                 contacts=(tur,), explode=(0.0, 0.0, -0.38), mass_item=MASS_ITEM_MOUNT,
                 notes="7075-T651 4 mm, 4 x M4 to the HD59 base (tapped), 4 x M4 to the isolators")
        iso_ids = []
        for i, (x, y) in enumerate(iso_points(C)):
            iso_ids.append(self.add(23 + i, "C", f"elastomer vibration isolator {i + 1}", f"elastomer titreşim "
                                    f"izolatörü {i + 1}", "purchased", P_BUY,
                                    (lambda x=x, y=y: isolator_mesh(C, x, y)), parent=car, joint=jt,
                                    contacts=(adp, car), purchased=True, mass_kg=m_iso,
                                    vendor="cylindrical elastomer isolator d 20 x 16 mm, M4 female both ends, ~40 Shore "
                                           "A (estimate, tuned to the HD59 mass; mount total 0.20 kg)",
                                    explode=(0.0, 0.0, -0.30), color="payload", mass_item=MASS_ITEM_MOUNT))
        self.iso_ids = iso_ids
        self.add(2, "C", "turret carriage (elevator)", "taret taşıyıcısı (asansör)", M7075, P_CNC,
                 lambda: M("car", lambda: E.carriage_mesh()), thickness=TAB_T, parent=C.pid(3), joint=jt,
                 contacts=(C.pid(3),), explode=(0.0, 0.0, -0.25),
                 notes="7075-T651 machined from plate: 5 mm plate + two 4 mm corner tabs to the rail blocks, ball-nut "
                       "seat, harness tab into the trough slot")
        self.add(3, "C", "ball nut 8 x 2 with PU bumper (upper hard stop)", "bilyalı somun 8 x 2, PU tampon (üst "
                 "sert durdurucu)", "purchased", P_BUY, lambda: M("nut", lambda: E.nut_mesh()), parent=C.pid(5),
                 joint=jt, contacts=(C.pid(5), car), purchased=True, mass_kg=0.040,
                 vendor="miniature flanged ball nut d 8 x 2 (SFK0802 class, estimate geometry and mass)",
                 explode=(0.0, 0.0, -0.20))
        # blocks
        blk = {"FR": [], "AL": []}
        k = 19
        for corner, i, z0, z1 in E.blocks:
            rail = C.rails[corner]["part"]
            pid = self.add(k, "C", f"rail carriage block {corner} {'upper' if i == 0 else 'lower'}",
                           f"ray taşıyıcı bloğu {corner} {'üst' if i == 0 else 'alt'}", "purchased", P_BUY,
                           (lambda corner=corner, z0=z0, z1=z1: E.block_mesh(corner, z0, z1)), parent=car, joint=jt,
                           contacts=(car, rail), purchased=True, mass_kg=BLK_MASS,
                           vendor="miniature profile-rail block MGN12C class (estimate; 2 x M3 mounting on the "
                                  "centre line)", explode=(0.0, 0.0, -0.22))
            blk[corner].append(pid)
            k += 1
        self.blk = blk

    def elevator_fixed(self):
        C, E, M = self.C, self.E, self.M
        for corner, num in (("FR", 8), ("AL", 9)):
            anc = C.anchor(corner)
            L = C.z_rail[1] - C.z_rail[0]
            self.add(num, "C", f"profile rail {C.rails[corner]['id']} (MGN12 class)",
                     f"profil ray {C.rails[corner]['id']} (MGN12 sınıfı)", "purchased", P_BUY,
                     (lambda corner=corner: M(("rail", corner), lambda: E.rail_mesh(corner))), parent=anc,
                     contacts=(anc,), purchased=True, mass_kg=round(RAIL_MASS_PER_M * L, 4),
                     vendor=f"miniature profile rail 12 mm (MGN12 class), L {L * 1000:.0f} mm, counterbored M3 at 25 mm "
                            "(estimate)", explode=(0.0, 0.0, -0.12))
        self.add(5, "C", "ball screw d 8 x 2 with journals", "bilyalı vida d 8 x 2, muylulu", "purchased", P_BUY,
                 lambda: M("screw", lambda: E.screw_mesh()), parent=C.pid(7), contacts=(C.pid(7), C.pid(12), C.pid(4)),
                 purchased=True, mass_kg=0.085,
                 vendor="rolled ball screw d 8 x 2 mm, journals d 6, L 0.24 m (estimate)", explode=(0.0, 0.0, -0.10))
        self.add(4, "C", "elevator drive: BLDC worm gear motor, spring brake, absolute encoder",
                 "asansör tahriki: BLDC sonsuz vida dişli motor, yay frenli, mutlak kodlayıcı", "purchased", P_BUY,
                 lambda: M("drive", lambda: E.drive_mesh()), parent=C.pid(7), contacts=(C.pid(7),), purchased=True,
                 mass_kg=0.200, vendor="BLDC gear motor d 22 mm + worm stage on a hollow wheel, spring-applied brake, "
                                       "absolute encoder, 2 limit switches (estimate)", explode=(0.06, 0.0, 0.05))
        self.add(7, "C", "ball-screw bearing housing (roof)", "bilyalı vida yatak yuvası (tavan)", M6061, P_CNC,
                 lambda: M("hous", lambda: E.housing_mesh()), thickness=HOUS_T, parent=C.p_roof,
                 contacts=(C.p_roof,), explode=(0.0, 0.0, 0.08),
                 notes="6061-T6 machined, angular-contact bearing pair 626 class for the upper journal, 4 x M4 into "
                       "potted inserts of the roof sandwich (layout top_anchor)")
        self.add(12, "C", "ball-screw lower journal bracket and lower hard stop", "bilyalı vida alt muylu braketi ve "
                 "alt sert durdurucu", M6061, P_CNC, lambda: M("lb", lambda: E.lb_mesh()), thickness=0.004,
                 parent=C.p_fs1110, contacts=(C.p_fs1110,), explode=(0.05, 0.0, -0.05),
                 notes="6061-T6 machined; PTFE-lined bush for the lower journal; boss top 0.5 mm under the carriage "
                       "plate at full extension = lower hard stop; 2 x M4 into potted inserts of the FS1110 web")

    # ---------------------------------------------------------------- doors (starboard; port mirrored)
    def doors(self):
        C, D, M = self.C, self.D, self.M
        jd = "turret_door_R"
        door = C.pid(6, "R")
        wF, wA = D.rail_w("F"), D.rail_w("A")
        self.w_r = {"F": wF, "A": wA}
        slab = lambda: M("slab", lambda: D.top_slab())
        self.add(6, "R", "sliding turret-bay door, starboard", "kayar taret bölmesi kapağı, sağ", MPW, P_PREG,
                 lambda: M("door", lambda: D.door_mesh()), thickness=DOOR_T, parent=C.pid(14, "R"),
                 contacts=(C.pid(14, "R"), C.pid(16, "R")), joint=jd, explode=(0.0, 0.25, -0.20), color="shell",
                 mirror=True, notes="OOA prepreg 10 plies PW (2.0 mm), moulded to the travel envelope (closed behind "
                                    "the aperture ring, inside the skin / caps when open); edge sliders bonded")
        self.add(14, "R", "door slider, front (PTFE-lined), starboard", "kapak kızağı, ön (PTFE kaplı), sağ", M6061,
                 P_CNC, lambda: M("slF", lambda: D.slider_mesh("F", wF, slab())), thickness=SL_T, parent=C.pid(17, "R"),
                 contacts=(door, C.pid(17, "R")), joint=jd, explode=(0.0, 0.22, -0.16), mirror=True,
                 notes="6061-T6 machined slider: PTFE-lined tongue in the rail groove + riser / 2 mm pad bonded "
                       "(EA 9394) to the door top face")
        self.add(16, "R", "door slider with rack, aft (PTFE-lined), starboard", "kapak kızağı ve kremayer, arka (PTFE "
                 "kaplı), sağ", M7075, P_CNC,
                 lambda: M("slA", lambda: D.slider_mesh("A", wA, slab(), rack=D.rack_parts(slab()))), thickness=SL_T,
                 parent=C.pid(18, "R"), contacts=(door, C.pid(18, "R"), C.pid(15, "R")), joint=jd,
                 explode=(0.0, 0.22, -0.16), mirror=True,
                 notes="7075-T651 machined slider (tongue + riser) + module 0.5 rack (teeth envelope = pitch line - 0.2 "
                       "mm), bonded to "
                       "the door; the rack cantilevers 56 mm outboard of the door edge under the bay wall")
        self.add(17, "R", "door rail, front (on FS1110), starboard", "kapak rayı, ön (FS1110 üzerinde), sağ", M6061,
                 P_CNC, lambda: M("rF", lambda: D.rail_mesh("F", wF)), thickness=DR_W, parent=C.p_fs1110,
                 contacts=(C.p_fs1110,), explode=(0.0, 0.15, -0.10), mirror=True,
                 notes="6061-T6 bar 6 x 6 mm with a 2.6 mm guide groove, inclined 32 deg, integral stand-off lugs "
                       "(<= 60 mm apart), ISO 7380 M3 into potted inserts of the web")
        self.add(18, "R", "door rail, aft (on FS1330), starboard", "kapak rayı, arka (FS1330 üzerinde), sağ", M6061,
                 P_CNC, lambda: M("rA", lambda: D.rail_mesh("A", wA)), thickness=DR_W, parent=C.p_fs1330,
                 contacts=(C.p_fs1330,), explode=(0.0, 0.15, -0.10), mirror=True,
                 notes="6061-T6 bar 6 x 6 mm with a 2.6 mm guide groove, inclined 32 deg, integral stand-off lugs "
                       "(<= 60 mm apart), ISO 7380 M3 into potted inserts of the web")
        self.add(15, "R", "door pinion and drive shaft, starboard", "kapak pinyonu ve tahrik mili, sağ", M7075, P_CNC,
                 lambda: M("pin", lambda: D.pinion_mesh()), thickness=PIN_SHAFT_R, parent=C.pid(10, "R"),
                 contacts=(C.pid(10, "R"), C.pid(11, "R"), C.pid(16, "R")), explode=(0.08, 0.0, 0.0), mirror=True,
                 notes="7075-T651 hard-anodised spur pinion module 0.5, 78 teeth (pitch d 39 mm, teeth envelope = pitch "
                       "cylinder, 2 mm web, 3.5 mm rim), splined "
                       "to the DA 22 output, carried in the bracket bush; shaft through C-TDOOR-SHAFT")
        self.add(10, "R", "turret door actuator Volz DA 22 (EQ-TDOORACT), starboard",
                 "taret kapağı eyleyicisi Volz DA 22 (EQ-TDOORACT), sağ", "purchased", P_BUY,
                 lambda: M("da22", lambda: D.da22_mesh()), parent=C.pid(11, "R"), contacts=(C.pid(11, "R"),),
                 purchased=True, mass_kg=float(C.da22["mass_kg"]), mirror=True, color="controls",
                 vendor="Volz DA 22-30-4128 (28 V), case 41.5 x 65.9 x 22 mm, 0.132 kg, 1.8 N m rated "
                        "(components.yaml volz_da22_28v)", explode=(0.15, 0.0, -0.05))
        self.add(11, "R", "DA 22 bracket on FS1330 aft face, starboard", "DA 22 braketi (FS1330 arka yüzü), sağ",
                 M6061, P_CNC, lambda: M("br", lambda: D.bracket_mesh()), thickness=BR_T, parent=C.p_fs1330,
                 contacts=(C.p_fs1330, C.pid(10, "R")), explode=(0.12, 0.0, -0.05), mirror=True,
                 notes="6061-T6 machined L-bracket 3 mm: frame flange (4 x M4 into potted inserts of FS1330) + side "
                       "flange (2 x M3 into the DA 22 case), bush for the pinion shaft")

    # ---------------------------------------------------------------- seal, harness
    def seal_harness(self):
        C, D, E, M = self.C, self.D, self.E, self.M
        self.add(27, "C", "aperture lip seal (silicone, bonded in the ring insert)", "açıklık dudak contası (silikon, "
                 "halka içine yapıştırılmış)", "purchased", P_BUY, lambda: M("seal", lambda: seal_mesh(C, D)),
                 parent=C.p_ring, contacts=(C.p_ring,), purchased=True, mass_kg=0.012, color="payload",
                 vendor="moulded silicone lip seal, cast to the door gap (0.5 mm off the closed doors), bonded with "
                        "RTV primer (estimate)", explode=(0.0, 0.0, -0.55))
        wall = C.p_wall["R"]
        self.add(28, "C", "harness service-loop trough (starboard bay wall)", "kablo servis döngüsü kanalı (sağ bölme "
                 "duvarı)", MPA12, P_SLS, lambda: M("tr", lambda: trough_mesh(C)), thickness=TR_T, parent=wall,
                 contacts=(wall,), explode=(0.0, -0.10, 0.0),
                 notes="SLS PA12 C-channel 40 x 12 mm, 24 mm slot for the carriage harness tab; the flat cable rolls "
                       "inside it (rolling-loop U-bend not modelled: deformable), 4 x M3 into potted wall inserts")
        self.add(29, "C", "turret harness service loop: flat cable + MIL-DTL-38999 receptacle",
                 "taret kablo servis döngüsü: yassı kablo + MIL-DTL-38999 soket", "purchased", P_BUY,
                 lambda: M("loop", lambda: loop_mesh(C)), parent=wall, contacts=(wall, C.pid(28)), purchased=True,
                 mass_kg=0.060, color="systems",
                 vendor="shielded flat flexible harness 16 mm (power 24 V + Ethernet), 90 deg backshell, MIL-DTL-38999 "
                        "series III jam-nut receptacle shell 13 through the wall (estimate)", explode=(0.0, -0.08, 0.05))
        C.reg.parts[wall].add_hole((TR_X, C.yw_in - 0.002, RCPT_Z), (TR_X, C.yw_out + 0.002, RCPT_Z), 0.0095)
        self.add(13, "C", "HD59 interface cable to the carriage tab", "HD59 arayüz kablosu (taşıyıcı tırnağına)",
                 "purchased", P_BUY, lambda: M("pig", lambda: pigtail_mesh(C, E)), parent=C.pid(0),
                 contacts=(C.pid(0), C.pid(2)), joint="turret_elevator", purchased=True, mass_kg=0.030,
                 color="systems", vendor="HD59 interface cable d 8 mm (vendor pigtail, power + Ethernet), clamped on "
                                         "the carriage (estimate)", explode=(0.0, 0.0, -0.30))

    # ---------------------------------------------------------------- mirror
    def mirror(self):
        reg = self.reg
        ids = list(self.mirror_ids)
        id_map = {p: p[:-1] + "L" for p in ids}
        for pid in ids:
            p = reg.parts[pid]
            jn = p.joint
            jl = None if jn is None else (jn[:-1] + "L" if jn.endswith("_R") else jn)
            q = mirror_part(p, id_map[pid], joint=jl, id_map=id_map)
            q.fasteners = []
            q.holes = []
            reg.add(q)

    # ---------------------------------------------------------------- fasteners
    def fasteners_elevator(self):
        C, E, reg = self.C, self.E, self.reg
        tur, adp, car = C.pid(0), C.pid(1), C.pid(2)
        z_a1 = C.z_top + ADAPT_T
        r = HD59_BOLT_R
        for i, (dx, dy) in enumerate(((r, 0.0), (0.0, r), (-r, 0.0), (0.0, -r))):
            J.bolt(reg, f"{adp}-B{i + 1}", 4, (C.xc + dx, dy, z_a1), (0, 0, -1), [(adp, ADAPT_T)], owner=adp,
                   nut="tapped", tapped_part=tur, tapped_depth=0.008, step=STEP,
                   notes="HD59 mounting screw (vendor ICD estimate), thread locker")
        for i, ((x, y), iso) in enumerate(zip(iso_points(C), self.iso_ids)):
            J.bolt(reg, f"{car}-B{i + 1}", 4, (x, y, E.z_c1), (0, 0, -1), [(car, CARR_T)], owner=car, nut="tapped",
                   tapped_part=iso, tapped_depth=0.0075, step=STEP, notes="carriage to isolator (top insert)")
            J.bolt(reg, f"{adp}-B{i + 5}", 4, (x, y, C.z_top), (0, 0, 1), [(adp, ADAPT_T)], owner=adp, nut="tapped",
                   tapped_part=iso, tapped_depth=0.0075, step=STEP, notes="adapter to isolator (bottom insert)")
        nb = 9
        for corner in ("FR", "AL"):
            x_r, sg, seat = E.rail_geom(corner)
            y_head = seat - BLK_H - TAB_T
            for bid, (cc, i, z0, z1) in zip(self.blk[corner], [b for b in E.blocks if b[0] == corner]):
                for dz in BLK_SCREW_DZ:
                    J.bolt(reg, f"{car}-B{nb}", 3, (x_r, sg * y_head, 0.5 * (z0 + z1) + dz), (0, sg, 0),
                           [(car, TAB_T)], owner=car, nut="tapped", tapped_part=bid, tapped_depth=0.0055, step=STEP,
                           notes="carriage tab to rail block")
                    nb += 1
        nut = C.pid(3)
        for i in range(4):
            a = math.radians(45 + 90 * i)
            p = (E.sx + NUT_BOLT_R * math.cos(a), E.sy + NUT_BOLT_R * math.sin(a), E.nut_z0 + NUT_FL_T)
            J.bolt(reg, f"{nut}-B{i + 1}", 3, p, (0, 0, -1), [(nut, NUT_FL_T), (car, CARR_T)], owner=nut, step=STEP,
                   notes="ball-nut flange to the carriage, nyloc under the plate")
        # rails to the chassis corner-angle seats (floating nutplates on the seat back)
        for corner, num in (("FR", 8), ("AL", 9)):
            rail, anc = C.pid(num), C.anchor(corner)
            x_r, sg, seat = E.rail_geom(corner)
            for i, z in enumerate(E.rail_screw_z()):
                J.bolt(reg, f"{rail}-B{i + 1}", 3, (x_r, sg * (seat - RAIL_H + RAIL_CB_D), z), (0, sg, 0),
                       [(rail, RAIL_H - RAIL_CB_D), (anc, 0.003)], owner=rail, nut="nutplate", step=STEP,
                       orient=(0, 0, 1), notes="rail screw into a floating anchor nutplate riveted on the seat back "
                                               "(layout.chassis.turret rail_screws_per_rail_end)")
        # bearing housing into the roof inserts, gearbox into the housing boss, lower bracket into FS1110
        hous = C.pid(7)
        fl_lo, fl_hi, bolts = E.housing_geom()
        for i, (x, y) in enumerate(bolts):
            J.bolt(reg, f"{hous}-B{i + 1}", 4, (x, y, fl_lo[2]), (0, 0, 1), [(hous, HOUS_T)], owner=hous,
                   nut="insert", insert_part=C.p_roof, insert_depth=C.t_roof, step=STEP,
                   notes="potted M4 insert through the roof sandwich (layout top_anchor)")
        drv = C.pid(4)
        lo, hi = E.gbx_box()
        for i, dx in enumerate((-0.007, 0.007)):
            J.bolt(reg, f"{drv}-B{i + 1}", 3, (E.sx + dx, hi[1] - 0.0065, lo[2]), (0, 0, 1),
                   [(drv, GBX_Z[1] - GBX_Z[0])], owner=drv, nut="tapped", tapped_part=hous, tapped_depth=0.008,
                   step=STEP, notes="gearbox to the housing boss")
        lbk = C.pid(12)
        for i, (y, z) in enumerate(E.lb_bolts()):
            J.bolt(reg, f"{lbk}-B{i + 1}", 4, (C.xf1 + 0.004, y, z), (-1, 0, 0), [(lbk, 0.004 - CT)], owner=lbk,
                   nut="insert", insert_part=C.p_fs1110, insert_depth=C.t_frame, step=STEP,
                   notes="potted M4 insert in the FS1110 web")

    def fasteners_doors(self, side):
        C, D, reg = self.C, self.D, self.reg
        sg = 1.0 if side == "R" else -1.0
        for which, num, frame, xf in (("F", 17, C.p_fs1110, C.xf1), ("A", 18, C.p_fs1330, C.xf3)):
            rail = C.pid(num, side)
            xa, xb = D.rail_x(which)
            w_r = self.w_r[which]
            for i, u in enumerate(D.standoff_u(which, w_r)):
                y, z = C.yz(u, w_r + SO_BOLT)
                if which == "F":
                    p, ax, grip = (xb - CB_D, sg * y, z), (-1, 0, 0), xb - CB_D - xf - CT
                else:
                    p, ax, grip = (xa + CB_D, sg * y, z), (1, 0, 0), xf - CT - xa - CB_D
                J.bolt(reg, f"{rail}-B{i + 1}", 3, p, ax, [(rail, grip)], owner=rail, head="ISO 7380", nut="insert",
                       insert_part=frame,
                       insert_depth=C.t_frame, step=STEP,
                       notes="door rail stand-off, potted M3 insert in the frame web")
        br, da = C.pid(11, side), C.pid(10, side)
        fl, sidef, fbolts, dbolts = D.bracket_geom()
        for i, (y, z) in enumerate(fbolts):
            J.bolt(reg, f"{br}-B{i + 1}", 4, (fl[1][0], sg * y, z), (-1, 0, 0), [(br, fl[1][0] - fl[0][0] + CT)],
                   owner=br, nut="insert", insert_part=C.p_fs1330, insert_depth=C.t_frame, step=STEP,
                   notes="DA 22 bracket, potted M4 insert in the FS1330 web")
        for i, (x, z) in enumerate(dbolts):
            J.bolt(reg, f"{br}-B{i + 5}", 3, (x, sg * sidef[0][1], z), (0, sg, 0), [(br, BR_T - CT)], owner=br,
                   nut="tapped", tapped_part=da, tapped_depth=0.008, step=STEP, notes="DA 22 case screw")

    def fasteners_harness(self):
        C, reg = self.C, self.reg
        tr, wall = C.pid(28), C.p_wall["R"]
        y_head = C.yw_in - CT - TR_T
        for i, (x, z) in enumerate(trough_bolts(C)):
            J.bolt(reg, f"{tr}-B{i + 1}", 3, (x, y_head, z), (0, 1, 0), [(tr, TR_T + CT)], owner=tr, head="ISO 7380",
                   nut="insert", insert_part=wall, insert_depth=C.t_wall, step=STEP,
                   notes="potted M3 insert through the wall sandwich")


# =====================================================================================================================
# verification helpers (used by the tests and the module notes)
# =====================================================================================================================
def e180_envelope(spec: dict, stroke: float = 0.0, grow: float = 0.0) -> G.Mesh:
    """E180 growth envelope as ``sizing.turret_checks`` defines it (``payload.turret.growth_envelope``): sphere d 0.18 m
    on the HD59 ball centre plus a d 0.18 m cylinder up to the envelope top (ball centre + height - radius), moved
    ``stroke`` down along the elevator, radius grown by ``grow``."""
    T = spec["payload"]["turret"]
    el = spec["layout"]["chassis"]["turret_elevator"]
    xc, zc = float(el["ball_center_retracted"][0]), float(el["ball_center_retracted"][2]) - float(stroke)
    g = T["growth_envelope"]
    top = zc + float(g["height"]) - 0.5 * float(g["diameter"])
    r = 0.5 * float(g["diameter"]) + grow
    return G.union([G.sphere(r, (xc, 0.0, zc), n=64), G.cylinder(r, (xc, 0.0, zc), (xc, 0.0, top), n=64)])


def fov_obstructions(reg: Registry, part_ids, state: dict | None = None) -> list[tuple[str, float, float]]:
    """Parts reaching below the field-of-regard cone of the extended turret (``layout.keep_outs`` KO-TURRET-FOV: apex on
    the extended ball centre, every protrusion above ``min_clear_elevation_deg``). Each vertex outside the ball is
    tested (posed with ``state``). Returns (part id, azimuth deg with 0 = forward, elevation deg) of the lowest vertex
    of every obstructing part."""
    ko = next(k for k in reg.spec["layout"]["keep_outs"] if k["id"] == "KO-TURRET-FOV")
    c = np.asarray(ko["apex"], float)
    e_min = float(ko["min_clear_elevation_deg"])
    r_b = 0.5 * float(reg.spec["payload"]["turret"]["ball_diameter"])
    out = []
    for pid in part_ids:
        p = reg.parts[pid]
        V = reg.posed_vertices(p, state) if state else p.mesh.V
        d = V - c
        elev = np.degrees(np.arctan2(d[:, 2], np.hypot(d[:, 0], d[:, 1])))
        bad = (np.linalg.norm(d, axis=1) > r_b + 1e-4) & (elev < e_min)
        if np.any(bad):
            k = int(np.argmin(np.where(bad, elev, np.inf)))
            out.append((pid, float(np.degrees(np.arctan2(d[k, 1], -d[k, 0])) % 360.0), float(elev[k])))
    return out


def register(reg: Registry, spec: dict) -> None:
    if any(p.startswith("YK250-PL-820") for p in reg.parts):
        reg.note("payload: already registered, second call ignored")
        return
    B = Build(reg, spec)
    B.joints()
    B.turret()
    B.elevator_fixed()
    B.doors()
    B.seal_harness()
    B.mirror()
    B.fasteners_elevator()
    for side in ("R", "L"):
        B.fasteners_doors(side)
    B.fasteners_harness()
    B.sequence()
