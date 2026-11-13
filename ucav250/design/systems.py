"""Systems producer (``layout.part_numbers.systems``, YK250-SY-720..819): avionics, electrical power, harness, antennas,
air data, lights and the recovery parachute installation of the YK-250 HANCER (civil EO/IR surveillance UAV).

What is built (every part with material + process + thickness, a parent, an assembly step, an explode vector and
contacts that really touch; fasteners through ``design/joints.py``):

* power: VISIONAIRtronics 1000 W PDU on six machined spacers (through-bolted from the side bays into the PDU base),
  main contactor / fuse block and 28-12 V DC-DC on a pocketed 6061 power-switching plate (potted M3 inserts in the
  avionics deck), 12S2P Li-ion buffer battery in a 6061 cradle frame with two stainless hold-down straps on TR-FWDBAY,
  independent FTS unit hung under the tray on two formed 6061 angles;
* avionics: Veronte 1x autopilot, Silvus SC4200EP and Microhard pDDL2450 datalinks and the ping200X transponder on a
  machined 6061 rack (base, inboard web and shelf with screw bosses) through-bolted to TR-SIDEBAY-R, the Dronetag
  Remote ID beacon on an SLS PA12 cradle on the FS0300 forward face (inside the GFRP nose cone);
* connectors: mated MIL-DTL-38999 plug + backshell on every unit connector face (inside the layout connector
  allowance), the blind-mate wing receptacles CN-WING on float-mount plates on the joint ribs, the fireproof firewall
  harness grommet (C-FW-HARN);
* harness: the trunks of ``layout.systems.harness`` (H-MAIN, H-COAX, H-WING, H-TAIL, H-ENGINE, H-NOSE) as round
  bundles on their declared paths (filleted corners), moulded split grommets in every frame cut-out they cross and in
  the declared member penetrations (centre box lower cover, side-of-body and glove ribs, main-well forward wall), and
  the CFRP centre-line harness conduit between the main wells;
* antennas / air data / lights: GNSS 1 and 2 (Calian HC977EXF) on formed 6061 frame brackets under their GFRP RF
  windows, the 2.4 GHz C2 dipole and the FTS antenna on SLS PA12 pedestals bonded into the GFRP nose cone, the
  transponder blade on the belly with a bonded copper-mesh ground-plane doubler, the heated nose pitot-static probe
  in the cone tip (nose-cone / belly items only when the shell is registered); the wing items (HASA pitot in the
  wing module's mast, wing-tip lights on the tip ribs) only when the wing module is registered; the fin-tip dipoles
  and the tail light are not created (no fit / provision in the tail module's tip caps: open item);
* recovery: UAVOS 200 container on four pads and a girth strap through the four strap brackets, the stowed Y-bridle
  legs with their spools on the two shackle pins (F-RISER-FWD / F-RISER-AFT), the pin-puller hatch latch on its
  bracket on FS1810.

Interfaces (``spec.layout`` only, never another module's geometry): ``systems.equipment`` / ``antennas`` /
``air_data_lights`` / ``harness`` (boxes, points, paths, penetrations, connectors), ``rules.boxes`` (bay boxes),
``chassis.members`` (deck, walls, floors, keel webs, ribs, spine), ``chassis.trays``, ``chassis.fittings`` (F-RISER-*
with their bridle spools), ``chassis.parachute``, ``stations`` (faces, cut-outs, sweep), ``shell.panels`` (RF
windows), ``mechanisms.joints`` (para_hatch) and the OML (``oml.fuselage_from_spec``: GFRP cone inner surface = OML
inset 6.0 mm, layout antenna notes).

Manufacturing: brackets and plates are CNC-milled 6061-T6 (ISO 2768-mK) or press-brake 6061-T6 sheet (inner bend
radius 3 t), straps are 304 sheet (0.5 t), pedestals / cradles SLS PA12, the conduit OOA prepreg (4 plies PW);
purchased units carry vendor datasheet masses (components.yaml) or labelled estimates; fasteners are metric
(ISO 4762 / ISO 7380, ISO 7040 nyloc nuts, potted inserts), clearance holes ISO 273 medium, edge distance >= 2 D
(metal / polymer) and >= 2.5 D (composite).
"""
from __future__ import annotations

import math

import numpy as np
from shapely.geometry import LineString, Point, Polygon, box as sbox
from shapely.ops import unary_union

from ..core import geom as G
from ..core.parts import Part, Registry, layup_props, mirror_part, part_number
from . import fastener_catalog as FC
from . import joints as J
from . import oml as O
from . import structgen as SG

# =====================================================================================================================
# module constants (detailing choices; interface values come from spec.layout)
# =====================================================================================================================
OV = 1e-4                    # boolean overlap of fused features
TOUCH = 5e-5                 # declared contact faces: 0.05 mm apart (no volume overlap, within the 0.3 mm contact test)
STEP_HARNESS, STEP_AVIONICS, STEP_ANTENNA, STEP_PARA = 19, 20, 21, 22      # spec.assembly steps

MAT_AL, MAT_SS, MAT_PA12, MAT_PW = "al_6061_t6_sheet", "ss_304_annealed", "pa12_sls", "cfrp_pw_mtm45_as4"
P_CNC, P_SHEET, P_SHEET_SS, P_SLS, P_PREG, P_BUY = ("cnc_milling_metal", "sheet_metal_aluminium", "sheet_metal_steel",
                                                    "sls_pa12", "prepreg_ooa_vacbag", "purchased")

SHEET_T = 0.002              # formed 6061-T6 brackets
R_AL = 3.0 * SHEET_T         # 6061-T6 inner bend radius 3 t (processes.sheet_metal_aluminium)
ANGLE_T = 0.0016             # FTS hanger angles
STRAP_T = 0.0006             # battery hold-down straps (304, inner bend radius 0.5 t)
STRAP_W = 0.016
PLATE_T = 0.0016             # web of the pocketed plates / rack base
BOSS_R = 0.0065              # screw bosses (>= 2 D edge distance for M3)
CB_DEPTH = FC.ISO7380[3][1] + 0.0001     # counterbore for an ISO 7380 M3 button head (flush under the plate)
CB_R = 0.5 * FC.ISO7380[3][0] + 0.0004
TAP_DEPTH = 0.007            # threaded holes in the purchased unit bases (vendor M3 mounting threads)
SPACER_R = 0.0065            # PDU spacers d 13

CONN_LEN_MARGIN = 0.002      # mated plug + backshell stops 2 mm short of the layout connector allowance
PLUG_MASS = 0.025            # mated MIL-DTL-38999 plug + backshell (estimate, from the harness connector share)
RECEPT_MASS = 0.035          # wing receptacle + float plate (estimate)
GROM_RHO = 700.0             # moulded silicone grommet with a cored plug: effective density (estimate)
GROM_CLR, GROM_RIM, GROM_FL = 0.0002, 0.004, 0.0015
HARNESS_BASE_KG = 2.70       # mass.items wiring_harness_connectors_coax base mass (cables + connectors + coax)
CABLE_SHARE = 0.70           # cables 70 % (layout.mass_placement basis), connectors / grommets 30 %

CONE_T = 0.006               # GFRP nose cone wall (layout antenna notes: "6.0 mm GFRP cone")

# part numbers (layout.part_numbers.systems = 720..819); layout-fixed ids are read from the layout
N = dict(pdu_spacers=721, sw_plate=723, batt_cradle=725, strap_f=727, strap_a=729, rack=731, rid_cradle=737,
         fts_angle=739, gnss1_brkt=740, gnss2_brkt=741, grom0=756, grom_ct=766, grom_sob=767, grom_glove=768,
         grom_well=769, fw_grommet=770, conduit=771, cn_wing=772, cn_plate=773, xpdr_doubler=785, ped_c2a=792,
         ped_fts=793, bridle_fwd=801, bridle_aft=802, spool_f=803, spool_a=805, latch_brkt=806, para_strap=807,
         para_pad=808, para_pad2=809)
PLUG_NUMS = (742, 743, 744, 745, 746, 747, 748, 749, 733)     # mated connectors, in the order of Systems.connectors


# =====================================================================================================================
# small geometry helpers
# =====================================================================================================================
def box3(lo, hi) -> G.Mesh:
    lo, hi = np.asarray(lo, float), np.asarray(hi, float)
    return G.box(hi - lo, center=0.5 * (lo + hi))


def finish(m: G.Mesh) -> G.Mesh:
    """Round trip through manifold (1 micrometre simplification): removes boolean slivers."""
    return G.Mesh.from_manifold(m.to_manifold().simplify(1e-6))


def union(ms) -> G.Mesh:
    ms = [m for m in ms if m is not None]
    return ms[0] if len(ms) == 1 else G.union(ms)


def diff(a: G.Mesh, cutters) -> G.Mesh:
    cutters = [c for c in cutters if c is not None]
    return G.difference(a, cutters) if cutters else a


def inter(a: G.Mesh, b: G.Mesh) -> G.Mesh:
    r = G.intersection(a, b)
    if r is None:
        raise ValueError("empty intersection")
    return r


def prism_z(poly, z0: float, z1: float) -> G.Mesh:
    return G.extrude(poly, z1 - z0, origin=(0.0, 0.0, z0), u=(1.0, 0.0, 0.0), v=(0.0, 1.0, 0.0))


def prism_x(poly, x0: float, x1: float) -> G.Mesh:
    """(y, z) polygon extruded along +x."""
    return G.extrude(poly, x1 - x0, origin=(x0, 0.0, 0.0), u=(0.0, 1.0, 0.0), v=(0.0, 0.0, 1.0))


def prism_y(poly, y0: float, y1: float) -> G.Mesh:
    """(x, z) polygon extruded along +y."""
    m = G.extrude(poly, y1 - y0, origin=(0.0, y1, 0.0), u=(1.0, 0.0, 0.0), v=(0.0, 0.0, 1.0))
    return m                         # u x v = -y: the extrusion runs from y1 towards y0


def cyl(p0, p1, r: float, n: int = 32) -> G.Mesh:
    return G.cylinder(r, np.asarray(p0, float), np.asarray(p1, float), n=n)


def seg_len(P) -> np.ndarray:
    return np.linalg.norm(np.diff(np.asarray(P, float), axis=0), axis=1)


def fillet(P, r: float, n_arc: int = 10, share: float = 0.48, radius_of=None) -> np.ndarray:
    """Polyline with every corner replaced by a circular arc of radius r (reduced to fit ``share`` of each shared leg
    and 0.9 of an end leg); ``radius_of(vertex)`` may return a different radius for a corner."""
    P = np.asarray(P, float)
    P = P[np.r_[True, seg_len(P) > 1e-9]]
    if len(P) < 3:
        return P
    L = seg_len(P)
    out = [P[0]]
    for i in range(1, len(P) - 1):
        a, b, c = P[i - 1], P[i], P[i + 1]
        u, v = G.unit(a - b), G.unit(c - b)
        phi = math.acos(float(np.clip(np.dot(u, v), -1.0, 1.0)))
        if math.pi - phi < 1e-3:
            out.append(b)
            continue
        rr = radius_of(b) if radius_of is not None else None
        rr = r if rr is None else rr
        t = min(rr / math.tan(phi / 2), L[i - 1] * (share if i - 1 > 0 else 0.9), L[i] * (share if i + 1 < len(P) - 1
                                                                                         else 0.9))
        R = t * math.tan(phi / 2)
        cen = b + G.unit(u + v) * (R / math.sin(phi / 2))
        d1, d2 = b + u * t - cen, b + v * t - cen
        ang = math.acos(float(np.clip(np.dot(G.unit(d1), G.unit(d2)), -1, 1)))
        for s in np.linspace(0.0, 1.0, n_arc):
            w1, w2 = math.sin((1 - s) * ang) / math.sin(ang), math.sin(s * ang) / math.sin(ang)
            out.append(cen + w1 * d1 + w2 * d2)
    out.append(P[-1])
    Q = np.asarray(out)
    return Q[np.r_[True, seg_len(Q) > 1e-7]]


def densify(Q, step: float = 0.01) -> np.ndarray:
    """Extra points on long straight legs (smooth parallel-transport frames for the sweep)."""
    out = [Q[0]]
    for a, b in zip(Q[:-1], Q[1:]):
        k = max(1, int(math.ceil(np.linalg.norm(b - a) / step)))
        for s in np.linspace(0.0, 1.0, k + 1)[1:]:
            out.append(a + s * (b - a))
    return np.asarray(out)


def bent_profile(points, t: float, r_in: float) -> Polygon:
    """Cross-section of a formed sheet part: the mid-thickness polyline ``points`` (2-D) filleted with the centre-line
    radius r_in + t/2 and thickened by t/2 each side (flat ends): inner bend radius r_in, outer r_in + t."""
    P = np.asarray(points, float)
    P3 = np.column_stack([P, np.zeros(len(P))])
    Q = fillet(P3, r_in + 0.5 * t, n_arc=12, share=0.5)[:, :2]
    return LineString(Q).buffer(0.5 * t, cap_style=2, join_style=2, mitre_limit=1.5)


WALLS = dict(xm=0.002, xp=0.002, ym=0.002, yp=0.002, zm=0.008, zp=0.002)   # enclosure walls (base 8 mm: threads)


def enclosure(lo, hi, w: dict) -> G.Mesh:
    """Purchased unit envelope as a closed enclosure (outer box minus the cavity): the M3 mounting threads sit in the
    8 mm base wall (vendor drawing to be confirmed); mass from the datasheet, CG from this envelope."""
    lo, hi = np.asarray(lo, float), np.asarray(hi, float)
    ilo = lo + np.array([w["xm"], w["ym"], w["zm"]])
    ihi = hi - np.array([w["xp"], w["yp"], w["zp"]])
    return diff(box3(lo, hi), [box3(ilo, ihi)])


def can(p0, p1, r: float, base: float, wall: float = 0.0015, n: int = 48) -> G.Mesh:
    """Closed cylindrical housing (antennas): base wall ``base`` at p0 for the mounting threads."""
    p0, p1 = np.asarray(p0, float), np.asarray(p1, float)
    a = G.unit(p1 - p0)
    return diff(G.cylinder(r, p0, p1, n=n), [G.cylinder(r - wall, p0 + base * a, p1 - wall * a, n=n)])


def counterbores(pts, z_bottom: float):
    """Counterbores from below for ISO 7380 M3 button heads (flush under a plate whose bottom face is z_bottom)."""
    return [cyl((x, y, z_bottom - 0.001), (x, y, z_bottom + CB_DEPTH), CB_R) for x, y in pts]


# =====================================================================================================================
# context
# =====================================================================================================================
class Ctx:
    def __init__(self, reg: Registry, spec: dict):
        self.reg, self.S = reg, spec
        L = self.L = spec["layout"]
        self.fus = O.fuselage_from_spec(spec)
        self.st = {s["id"]: s for s in L["stations"]}
        self.st_by_part = {s["part"]: s for s in L["stations"]}
        self.mem = {m["id"]: m for m in L["chassis"]["members"]}
        self.fit = {f["id"]: f for f in L["chassis"]["fittings"]}
        self.trays = {t["id"]: t for t in L["chassis"]["trays"]}
        self.boxes = L["rules"]["boxes"]
        sy = L["systems"]
        self.eq = {e["id"]: e for e in sy["equipment"]}
        self.ant = {a["id"]: a for a in sy["antennas"]}
        self.adl = {a["id"]: a for a in sy["air_data_lights"]}
        self.trunk = {t["id"]: t for t in sy["harness"]["trunks"]}
        self.conn = {c["id"]: c for c in sy["harness"]["connectors"]}
        self.panels = {p["id"]: p for p in L["shell"]["panels"]}
        self.mech = {j["name"]: j for j in L["mechanisms"]["joints"]}
        self.skin_t = layup_props(spec, "shell_secondary")["thickness"]

    def eqbox(self, eid: str):
        b = self.eq[eid]["box"]
        return np.asarray(b[0], float), np.asarray(b[1], float)

    def member_box(self, mid: str):
        b = self.mem[mid]["box"]
        return np.asarray(b[0], float), np.asarray(b[1], float)

    def stp(self, sid: str) -> str:
        return self.st[sid]["part"]

    def face(self, sid: str, side: int):
        """(x at the centre line, sweep tangent) of the forward (-1) / aft (+1) web face of a frame."""
        st = self.st[sid]
        xf = st.get("x_faces") or [st["x"] - 0.5 * st["t"], st["x"] + 0.5 * st["t"]]
        return float(xf[0] if side < 0 else xf[1]), math.tan(math.radians(float(st.get("sweep_deg", 0.0))))

    def cutout(self, sid: str, cid: str, mirror: bool = False):
        c = next(c for c in self.st[sid]["cutouts"] if c["id"] == cid)
        y0, y1 = (float(v) for v in c["y"])
        if mirror:
            y0, y1 = -y1, -y0
        return y0, y1, float(c["z"][0]), float(c["z"][1])

    def cone_cavity(self, x0: float, x1: float, inset: float, n: int = 6, ring_n: int = 96) -> G.Mesh:
        """Solid of the nose-cone cavity between x0 and x1: OML section inset by ``inset`` (the GFRP cone inner
        surface is the OML inset by CONE_T), lofted through n stations."""
        rings = []
        for x in np.linspace(x0, x1, n):
            poly = SG.fuselage_section2d(self.fus, float(x), inset, n=256)
            R = SG.resample_ring(poly, ring_n, start_dir=(0.0, 1.0))
            rings.append(np.column_stack([np.full(len(R), x), R[:, 0], R[:, 1]]))
        return G.loft(rings)

    def body_band(self, x0: float, x1: float, inset0: float, inset1: float, n: int = 6, ring_n: int = 160) -> G.Mesh:
        """Shell of the body between OML insets inset0 < inset1 (negative = outside the OML)."""
        def solid(ins):
            rings = []
            for x in np.linspace(x0, x1, n):
                poly = Polygon(self.fus.point(np.full(256, x), np.linspace(0, 2 * math.pi, 256, endpoint=False))
                               [:, 1:3]).buffer(0)
                poly = poly.buffer(-ins, join_style=2) if ins != 0 else poly
                poly = SG.largest(poly)
                R = SG.resample_ring(poly, ring_n, start_dir=(0.0, 1.0))
                rings.append(np.column_stack([np.full(len(R), x), R[:, 0], R[:, 1]]))
            return G.loft(rings)
        return diff(solid(inset0), [solid(inset1)])


# =====================================================================================================================
# the producer
# =====================================================================================================================
class Systems:
    def __init__(self, reg: Registry, spec: dict):
        self.reg, self.spec = reg, spec
        self.C = Ctx(reg, spec)
        self.n0, self.n1 = self.C.L["part_numbers"]["systems"]
        self.plugs = []              # (pid, unit pid) mated connectors
        self.harness_parts = []      # trunk part ids (cable mass share)
        self.harness_fixed = {}      # connector / grommet part id -> mass (connector share)

    # ------------------------------------------------------------------ helpers
    def pid(self, key_or_num, side: str = "C") -> str:
        n = N[key_or_num] if isinstance(key_or_num, str) else int(key_or_num)
        if not self.n0 <= n <= self.n1:
            raise ValueError(f"systems part number {n} outside {self.n0}..{self.n1}")
        return part_number("systems", n, side=side)

    def add(self, pid, name, name_tr, material, process, mesh_fn, *, thickness=None, purchased=False, vendor="",
            mass_kg=None, parent=None, step=STEP_AVIONICS, explode=(0.0, 0.0, 0.1), contacts=(), notes="") -> Part:
        side = "R" if pid.endswith("-R") else ("L" if pid.endswith("-L") else "C")
        p = Part(id=pid, name=name, name_tr=name_tr, group="systems", material=material, process=process,
                 mesh_fn=mesh_fn, thickness=thickness, purchased=purchased, vendor=vendor, mass_kg=mass_kg,
                 side=side, parent=parent, step=step, explode=tuple(float(v) for v in explode),
                 contacts=tuple(dict.fromkeys(c for c in contacts if c)), notes=notes, color="systems")
        return self.reg.add(p)

    def has(self, pid: str) -> bool:
        return pid in self.reg.parts

    def find_part(self, prefix: str, words) -> str | None:
        """Parent lookup by id prefix and name words (wing / tail parts whose numbers the layout does not fix)."""
        for pid, p in self.reg.parts.items():
            if pid.startswith(prefix) and all(w in p.name for w in words):
                return pid
        return None

    # ------------------------------------------------------------------ main
    def build(self):
        self.power_switching()
        self.pdu()
        self.battery_and_fts()
        self.sidebay_rack()
        self.remote_id()
        self.connectors()
        self.gnss()
        if self.has(self.C.panels["P-NOSECONE"]["part"]):
            self.nose_antennas()
            self.pitot_nose()
        else:
            self.reg.note("systems: shell not registered - nose-cone antennas, pedestals and the nose pitot not created")
        if self.has(self.C.panels["P-FWDSKIN"]["part"]):
            self.transponder_blade()
        else:
            self.reg.note("systems: shell not registered - transponder blade and its doubler not created")
        self.harness()
        self.parachute()
        self.wing_tail_items()
        self.harness_masses()

    # =================================================================================================================
    # purchased units (layout boxes; M3 vendor threads in the base)
    # =================================================================================================================
    def unit(self, eid: str, vendor: str, mass: float, *, parent: str, contacts=(), notes: str = "",
             step: int = STEP_AVIONICS, explode=(0.0, 0.0, 0.12), walls=None) -> str:
        e = self.C.eq[eid]
        lo, hi = self.C.eqbox(eid)
        pid = e["part"]
        w = dict(WALLS, **(walls or {}))
        self.add(pid, e["name"], e["name_tr"], P_BUY, P_BUY, (lambda lo=lo, hi=hi, w=w: enclosure(lo, hi, w)),
                 purchased=True,
                 vendor=vendor, mass_kg=mass, parent=parent, contacts=contacts, step=step, explode=explode,
                 notes=(notes + "; " if notes else "") + f"envelope = layout box ({e['source'][:120]})")
        return pid

    def tapped(self, fid, unit: str, plate: str, x: float, y: float, z_plate_bottom: float, z_unit: float,
               step: int = STEP_AVIONICS):
        """ISO 7380 M3 button head from below, head in a counterbore of ``plate`` (bottom face z_plate_bottom), thread
        in the vendor M3 mounting hole of ``unit`` (base face z_unit)."""
        p = (x, y, z_plate_bottom + CB_DEPTH)
        J.bolt(self.reg, fid, 3, p, (0.0, 0.0, 1.0), [(plate, z_unit - p[2])], head="ISO 7380", nut="tapped",
               tapped_part=unit, tapped_depth=TAP_DEPTH, owner=plate, step=step,
               notes=f"{unit} on its mount (vendor M3 base threads, thread-locking compound)")

    def insert_bolt(self, fid, size, point, axis, stack, insert_part, depth, step=STEP_AVIONICS, notes=""):
        J.bolt(self.reg, fid, size, point, axis, stack, nut="insert", insert_part=insert_part, insert_depth=depth,
               owner=stack[0][0], step=step, notes=notes)

    def nyloc(self, fid, size, point, axis, stack, step=STEP_AVIONICS, notes=""):
        J.bolt(self.reg, fid, size, point, axis, stack, nut="ISO 7040", owner=stack[0][0], step=step, notes=notes)

    # =================================================================================================================
    # power switching: contactor / fuse block + DC-DC on a pocketed plate on the avionics deck
    # =================================================================================================================
    def power_switching(self):
        C = self.C
        deck = C.mem["M-DECK-NOSE"]["part"]
        z_deck = float(C.member_box("M-DECK-NOSE")[1][2])
        t_deck = float(C.mem["M-DECK-NOSE"]["section"]["t"])
        bb = C.boxes["power_switching_bay"]
        clo, chi = C.eqbox("EQ-CONTACTOR_FUSES")
        dlo, dhi = C.eqbox("EQ-DCDC_28_12")
        z_top = float(min(clo[2], dlo[2]))
        assert abs(clo[2] - dlo[2]) < 1e-9, "contactor and DC-DC share one plate level"
        x0, x1 = float(bb["x"][0]) + 0.005, float(bb["x"][1]) + 0.0025
        hw = float(bb["half_width"]) - 0.003
        inset = 0.0065
        u_pts = {"c": [(clo[0] + inset, clo[1] + inset), (clo[0] + inset, chi[1] - inset), (chi[0] - inset, clo[1] + inset),
                       (chi[0] - inset, chi[1] - inset)],
                 "d": [(dlo[0] + inset, dlo[1] + inset), (dlo[0] + inset, dhi[1] - inset), (dhi[0] - inset, dlo[1] + inset),
                       (dhi[0] - inset, dhi[1] - inset)]}
        d_pts = [(x0 + 0.006, s * (hw - 0.006)) for s in (1, -1)] + [(x1 - 0.006, s * (hw - 0.006)) for s in (1, -1)]
        self.sw_geom = dict(x0=x0, x1=x1, hw=hw, z0=z_deck, z1=z_top, u_pts=u_pts, d_pts=d_pts)

        def mesh():
            body = box3((x0, -hw, z_deck + TOUCH), (x1, hw, z_top))
            # pockets from below: 1.6 mm web under the units, bosses round every screw, 6 mm rims
            pk = sbox(x0 + 0.006, -hw + 0.006, x1 - 0.006, hw - 0.006)
            keep = unary_union([Point(p).buffer(BOSS_R, 24) for p in u_pts["c"] + u_pts["d"] + d_pts])
            pocket = pk.difference(keep)
            cuts = [prism_z(g, z_deck - 0.001, z_top - PLATE_T) for g in getattr(pocket, "geoms", [pocket])]
            cuts += counterbores(u_pts["c"] + u_pts["d"], z_deck + TOUCH)
            return finish(diff(body, cuts))
        plate = self.pid("sw_plate")
        self.add(plate, "power-switching mounting plate", "güç anahtarlama montaj plakası", MAT_AL, P_CNC, mesh,
                 thickness=PLATE_T, parent=deck, contacts=(deck,), explode=(0.0, 0.0, 0.10),
                 notes="6061-T6 plate milled from 6 mm stock, pocketed from below (1.6 mm web, screw bosses d 13); "
                       "heat spreader of the contactor / DC-DC bases (thermal pad); 4 x M3 into potted "
                       "through-inserts of the avionics deck (outboard of the keel slot)")
        c = self.unit("EQ-CONTACTOR_FUSES", "main contactor + fuse block, 28 V (allocation, components.yaml electrical)",
                      float(C.eq["EQ-CONTACTOR_FUSES"]["mass_kg"]), parent=plate, contacts=(plate,))
        d = self.unit("EQ-DCDC_28_12", "RECOM 28-to-12 V DC-DC module (components.yaml electrical.recom)",
                      float(C.eq["EQ-DCDC_28_12"]["mass_kg"]), parent=plate, contacts=(plate,))
        for k, (x, y) in enumerate(u_pts["c"]):
            self.tapped(f"{c}-B{k + 1}", c, plate, x, y, z_deck + TOUCH, z_top)
        for k, (x, y) in enumerate(u_pts["d"]):
            self.tapped(f"{d}-B{k + 1}", d, plate, x, y, z_deck + TOUCH, z_top)
        for k, (x, y) in enumerate(d_pts):
            self.insert_bolt(f"{plate}-B{k + 1}", 3, (x, y, z_top), (0.0, 0.0, -1.0),
                             [(plate, z_top - z_deck)], deck, t_deck,
                             notes="power-switching plate rim into a potted M3 insert of the avionics deck")

    # =================================================================================================================
    # PDU on six spacers, bolted from below through the deck (heads in the side bays)
    # =================================================================================================================
    def pdu(self):
        C = self.C
        deck = C.mem["M-DECK-NOSE"]["part"]
        lo_d, hi_d = C.member_box("M-DECK-NOSE")
        z_deck, z_bot = float(hi_d[2]), float(lo_d[2])
        lo, hi = C.eqbox("EQ-PDU")
        # middle pair aft of the brake-unit box under the deck (its hanger lies against the deck underside there):
        # bolt heads >= 3 mm from the EQ-BRAKE_UNIT box in x
        _, hi_b = C.eqbox("EQ-BRAKE_UNIT")
        x_mid = max(0.5 * float(lo[0] + hi[0]), float(hi_b[0]) + 0.003 + 0.5 * FC.ISO4762[3][0])
        xs = (float(lo[0]) + 0.017, x_mid, float(hi[0]) - 0.012)
        ys = (float(hi[1]) - 0.0125, float(lo[1]) + 0.0125)
        pts = [(x, y) for x in xs for y in ys]
        self.pdu_pts = pts
        sp = self.pid("pdu_spacers")

        def mesh():
            ms = [G.tube(SPACER_R, 0.5 * FC.clearance(3), (x, y, z_deck + TOUCH), (x, y, float(lo[2])), n=40)
                  for x, y in pts]
            return G.merge(ms)
        self.add(sp, "PDU mounting spacers (6x)", "PDU montaj burçları (6 adet)", MAT_AL, P_CNC, mesh,
                 thickness=SPACER_R - 0.5 * FC.clearance(3), parent=deck, contacts=(deck,), explode=(0.0, 0.0, 0.10),
                 notes="6061-T6 turned spacers d 13 x 3.4 mm (deck top to the PDU base, thermal pad between)")
        pdu = self.unit("EQ-PDU", "VISIONAIRtronics 1000 W PDU (components.yaml vat_1000w_pdu, IP67, MIL-DTL-38999)",
                        float(C.eq["EQ-PDU"]["mass_kg"]), parent=sp, contacts=(sp,),
                        notes="conduction-cooled through its base; the unit's thermostatic fan outputs are wired for an "
                              "optional cooling fan (open item, thermal test)")
        t_deck = z_deck - z_bot
        for k, (x, y) in enumerate(pts):
            J.bolt(self.reg, f"{pdu}-B{k + 1}", 3, (x, y, z_bot), (0.0, 0.0, 1.0),
                   [(deck, t_deck), (sp, float(lo[2]) - z_deck)], nut="tapped", tapped_part=pdu,
                   tapped_depth=TAP_DEPTH, owner=sp, step=STEP_AVIONICS,
                   notes="from below through a potted bushing of the deck (access through the side-bay panels)")

    # =================================================================================================================
    # buffer battery (cradle + straps) and the FTS unit (two hanger angles under TR-FWDBAY)
    # =================================================================================================================
    def battery_and_fts(self):
        C = self.C
        tray = C.trays["TR-FWDBAY"]["part"]
        z_tray = float(C.boxes["forward_bay"]["z"][0])
        t_tray = 0.003
        lo, hi = C.eqbox("EQ-BUFFER_BATTERY")
        z_c0, z_c1 = z_tray + TOUCH, float(lo[2])
        xa, xb = float(lo[0]) - 0.002, float(hi[0]) + 0.002
        ysd = float(hi[1])
        hw = 0.054
        x_straps = (0.470, 0.555)
        yb = 0.048
        x_hang = 0.582
        flo, fhi = C.eqbox("EQ-FTS_UNIT")
        # cradle outline: follows the tray taper (tray half-width 41.5 mm at x 0.39 growing aft)
        outline = Polygon([(xa, -0.040), (0.410, -0.0455), (0.440, -0.0510), (0.462, -hw), (xb, -hw), (xb, hw),
                           (0.462, hw), (0.440, 0.0510), (0.410, 0.0455), (xa, 0.040)])
        bolt_pts = [(x, s * yb) for x in x_straps + (x_hang,) for s in (1, -1)]

        outline = unary_union([sbox(xa, -ysd - 0.004, xb, ysd + 0.004)] +
                              [sbox(x - 0.008, -0.056, x + 0.008, 0.056) for x in x_straps]).intersection(outline)

        def cradle():
            body = prism_z(outline, z_c0, z_c1)
            inner = sbox(xa + 0.007, -ysd + 0.006, xb - 0.007, ysd - 0.006)
            bars = unary_union([sbox(x - 0.008, -1, x + 0.008, 1) for x in x_straps])
            pocket = inner.difference(bars)
            cuts = [prism_z(g, z_c0 - 0.001, z_c1 + 0.001) for g in getattr(pocket, "geoms", [pocket])]
            lips = [box3((xa, -0.028, z_c1 - OV), (float(lo[0]) - TOUCH, 0.028, z_c1 + 0.006)),
                    box3((float(hi[0]) + TOUCH, -0.028, z_c1 - OV), (xb, 0.028, z_c1 + 0.006))]
            return finish(union([diff(body, cuts)] + lips))
        cr = self.pid("batt_cradle")
        self.add(cr, "buffer-battery cradle", "tampon batarya beşiği", MAT_AL, P_CNC, cradle, thickness=0.002,
                 parent=tray, contacts=(tray,), explode=(0.0, 0.0, 0.14),
                 notes="6061-T6 frame 3.4 mm (rim 6 mm, two cross bars under the straps), fore / aft lips 6 mm "
                       "(9 g forward crash restraint of the 1.98 kg pack)")
        bat = self.unit("EQ-BUFFER_BATTERY", "12S2P Molicel INR-21700-P45B Li-ion pack, cell-fused vented box "
                        "(components.yaml liion_12s2p_molicel_p45b)", float(C.eq["EQ-BUFFER_BATTERY"]["mass_kg"]),
                        parent=cr, contacts=(cr,), explode=(0.0, 0.0, 0.20),
                        notes="vent outlet ducted to the flame-arresting grille of the forward-bay hatch P-FWDHATCH")
        # hold-down straps (304, 0.8 mm, inner bend radius 0.4 mm), 0.2 mm off the box sides and top
        g = 0.0002
        yl = ysd + g + 0.5 * STRAP_T
        zt = float(hi[2]) + g + 0.5 * STRAP_T
        zf = z_c1 + 0.5 * STRAP_T + TOUCH
        prof = bent_profile([(-0.060, zf), (-yl, zf), (-yl, zt), (yl, zt), (yl, zf), (0.060, zf)], STRAP_T,
                            0.5 * STRAP_T)
        straps = []
        for key, x in zip(("strap_f", "strap_a"), x_straps):
            sid = self.pid(key)
            straps.append(sid)
            self.add(sid, f"battery hold-down strap ({'fwd' if key == 'strap_f' else 'aft'})",
                     f"batarya bağlama kayışı ({'ön' if key == 'strap_f' else 'arka'})", MAT_SS, P_SHEET_SS,
                     (lambda x=x: prism_x(prof, x - 0.5 * STRAP_W, x + 0.5 * STRAP_W)), thickness=STRAP_T,
                     parent=cr, contacts=(cr, bat), explode=(0.0, 0.0, 0.26),
                     notes="304 sheet 0.6 x 16 mm formed over the pack (inner radius 0.5 t), feet on the cradle bars")
        # FTS hanger angles (6061-T6 1.6 mm, inner radius 3 t), legs 0.05 mm off the unit sides
        yleg = float(fhi[1]) + TOUCH + 0.5 * ANGLE_T
        zfl = z_tray - t_tray - 0.5 * ANGLE_T - TOUCH
        z_leg_bot = float(flo[2]) + 0.0035
        prof_a = bent_profile([(0.0545, zfl), (yleg, zfl), (yleg, z_leg_bot)], ANGLE_T, 3.0 * ANGLE_T)
        hx0, hx1 = 0.512, 0.5905
        ang = self.pid("fts_angle", "R")

        def angle():
            return prism_x(prof_a, hx0, hx1)
        fts = self.unit("EQ-FTS_UNIT", "independent flight-termination unit (components.yaml recovery_and_safety."
                        "flight_termination, receiver + controller + backup cell)", float(C.eq["EQ-FTS_UNIT"]["mass_kg"]),
                        parent=ang, contacts=(ang, ang[:-1] + "L"), explode=(0.0, 0.0, -0.12),
                        walls=dict(ym=0.008, yp=0.008, zm=0.002),
                        notes="hung between the two angles (2 x M3 each side into the vendor side threads)")
        self.add(ang, "FTS hanger angle, starboard", "FTS askı köşebendi, sağ", MAT_AL, P_SHEET, angle,
                 thickness=ANGLE_T, parent=tray, contacts=(tray, fts), explode=(0.0, 0.05, -0.08),
                 notes="6061-T6 1.6 mm angle, flange under TR-FWDBAY clamped by the cradle / strap bolts")
        self.reg.add(mirror_part(self.reg.parts[ang], ang[:-1] + "L", id_map={ang: ang[:-1] + "L"}))
        angL = ang[:-1] + "L"
        # bolts: strap feet + cradle + tray (+ hanger flange) with nyloc nuts under the tray / hanger
        k = 0
        for (x, y) in bolt_pts:
            k += 1
            stack = []
            if x in x_straps:
                stack.append((straps[x_straps.index(x)], STRAP_T + TOUCH))
            if x <= xb - 0.006:
                stack.append((cr, z_c1 - z_c0))
            stack += [(tray, t_tray + TOUCH)]
            if hx0 + 0.006 <= x <= hx1 - 0.006:
                stack.append((ang if y > 0 else angL, ANGLE_T + TOUCH))
            top = z_c1 + (STRAP_T + TOUCH if x in x_straps else 0.0) if x <= xb - 0.006 else z_tray
            self.nyloc(f"{cr}-B{k}", 3, (x, y, top), (0.0, 0.0, -1.0), stack,
                       notes="battery strap / cradle / tray / FTS hanger stack, nyloc under the tray")
        for s, a_id in ((1, ang), (-1, angL)):
            for j, x in enumerate((0.530, 0.575)):
                yf = s * (float(fhi[1]) + TOUCH + ANGLE_T)
                J.bolt(self.reg, f"{a_id}-B{j + 1}", 3, (x, yf, -0.095), (0.0, -s, 0.0), [(a_id, ANGLE_T + TOUCH)],
                       nut="tapped", tapped_part=fts, tapped_depth=TAP_DEPTH, owner=a_id, step=STEP_AVIONICS,
                       notes="FTS unit side thread (vendor)")

    # =================================================================================================================
    # starboard side bay: avionics rack on TR-SIDEBAY-R
    # =================================================================================================================
    def sidebay_rack(self):
        C = self.C
        tray = C.trays["TR-SIDEBAY-R"]["part"]
        z0 = float(C.boxes["avionics_side_bays"]["z"][0])
        a_lo, a_hi = C.eqbox("EQ-AUTOPILOT")
        s_lo, s_hi = C.eqbox("EQ-DATALINK_PRIMARY")
        m_lo, m_hi = C.eqbox("EQ-DATALINK_BACKUP")
        t_lo, t_hi = C.eqbox("EQ-TRANSPONDER")
        yw0, yw1 = 0.0565, 0.0605                       # inboard web (clear of the tray keel-flange bolt heads)
        xr0, xr1 = 0.858, 1.092
        x_sh1 = 1.018
        z_b0 = z0 + TOUCH
        z_b1 = z_b0 + PLATE_T
        z_s0, z_s1 = float(m_lo[2]) - 0.0035, float(m_lo[2]) - 0.0015        # shelf 2 mm, bosses up to the units
        base = Polygon([(xr0, yw0), (xr1, yw0), (xr1, 0.150), (0.955, 0.150), (xr0, 0.1345)])
        ins = 0.0065

        def corners(lo, hi, i=ins):
            return [(float(lo[0]) + i, float(lo[1]) + i), (float(lo[0]) + i, float(hi[1]) - i),
                    (float(hi[0]) - i, float(lo[1]) + i), (float(hi[0]) - i, float(hi[1]) - i)]
        pa, ps, pm, pt = corners(a_lo, a_hi), corners(s_lo, s_hi), corners(m_lo, m_hi), corners(t_lo, t_hi, 0.006)
        tray_pts = [(0.968, 0.075), (0.968, 0.125), (1.086, 0.080), (1.086, 0.125), (0.990, 0.1435), (1.050, 0.1435)]
        self.rack_geom = dict(base=base, z0=z_b0)

        def rack():
            ms = [prism_z(base, z_b0, z_b1)]
            for pts, zt in ((pa, float(a_lo[2])), (ps, float(s_lo[2]))):
                ms += [G.cylinder(BOSS_R, (x, y, z_b1 - OV), (x, y, zt), n=32) for x, y in pts]
            ms.append(box3((xr0, yw0, z_b1 - OV), (x_sh1, yw1, z_s1)))                         # inboard web
            ms.append(prism_z(Polygon([(xr0, yw0), (x_sh1, yw0), (x_sh1, 0.1345), (xr0, 0.1345)]), z_s0, z_s1))
            for pts, zt in ((pm, float(m_lo[2])), (pt, float(t_lo[2]))):
                ms += [G.cylinder(BOSS_R, (x, y, z_s1 - OV), (x, y, zt), n=32) for x, y in pts]
            m = union(ms)
            cuts = counterbores(pa + ps, z_b0) + counterbores(pm + pt, z_s0)
            # lightening windows under the units (they sit on their bosses) and in the web
            for (lo_, hi_, pts, z0_, z1_) in ((a_lo, a_hi, pa, z_b0, z_b1), (s_lo, s_hi, ps, z_b0, z_b1),
                                              (m_lo, m_hi, pm, z_s0, z_s1), (t_lo, t_hi, pt, z_s0, z_s1)):
                win = sbox(float(lo_[0]) + 0.003, max(float(lo_[1]) + 0.003, yw1 + 0.004), float(hi_[0]) - 0.003,
                           min(float(hi_[1]) - 0.003, 0.1305)).difference(
                    unary_union([Point(q).buffer(BOSS_R + 0.0025, 24) for q in pts]))
                cuts += [prism_z(g, z0_ - 0.001, z1_ + 0.001) for g in getattr(win, "geoms", [win]) if g.area > 2e-5]
            cuts += [G.cylinder(0.011, (x, yw0 - 0.001, -0.061), (x, yw1 + 0.001, -0.061), n=32)
                     for x in (0.895, 0.950, 1.000)]
            return finish(diff(m, cuts))
        rk = self.pid("rack")
        self.add(rk, "avionics rack, starboard side bay", "aviyonik rafı, sağ yan bölme", MAT_AL, P_CNC, rack,
                 thickness=PLATE_T, parent=tray, contacts=(tray,), explode=(0.0, 0.10, -0.12),
                 notes="one-piece 6061-T6 rack: 1.6 mm base with screw bosses (autopilot, C2 datalink), 4 mm inboard "
                       "web, 2 mm shelf with bosses (backup link, transponder); 6 x M3 through TR-SIDEBAY-R, nylocs below")
        units = [("EQ-AUTOPILOT", "Embention Veronte Autopilot 1x, ADS-B variant (components.yaml veronte_autopilot_1x, "
                  "M3 mounting screws)", 0.209, pa, z_b0),
                 ("EQ-DATALINK_PRIMARY", "Silvus StreamCaster SC4200EP, module variant (components.yaml silvus_sc4200ep)",
                  float(C.eq["EQ-DATALINK_PRIMARY"]["mass_kg"]), ps, z_b0),
                 ("EQ-DATALINK_BACKUP", "Microhard pDDL2450, enclosed (components.yaml microhard_pddl2450)",
                  float(C.eq["EQ-DATALINK_BACKUP"]["mass_kg"]), pm, z_s0),
                 ("EQ-TRANSPONDER", "uAvionix ping200X Mode S ES (components.yaml uavionix_ping200x)",
                  float(C.eq["EQ-TRANSPONDER"]["mass_kg"]), pt, z_s0)]
        for eid, vendor, mass, pts, zb in units:
            u = self.unit(eid, vendor, mass, parent=rk, contacts=(rk,), explode=(0.0, 0.14, -0.16))
            zu = float(C.eqbox(eid)[0][2])
            for k, (x, y) in enumerate(pts):
                self.tapped(f"{u}-B{k + 1}", u, rk, x, y, zb, zu)
        t_tray = 0.003
        for k, (x, y) in enumerate(tray_pts):
            self.nyloc(f"{rk}-B{k + 1}", 3, (x, y, z_b1), (0.0, 0.0, -1.0), [(rk, PLATE_T), (tray, t_tray + TOUCH)],
                       notes="rack base through TR-SIDEBAY-R, nyloc under the tray (side-bay panel access)")

    # =================================================================================================================
    # Remote ID beacon on an SLS cradle on the FS0300 forward face
    # =================================================================================================================
    def remote_id(self):
        C = self.C
        fr = C.stp("FS0300")
        xf, _ = C.face("FS0300", -1)
        t_fr = float(C.st["FS0300"]["t"])
        lo, hi = C.eqbox("EQ-REMOTE_ID")
        tp = 0.0026
        x_p0 = xf - tp
        z_l1 = float(lo[2]) - TOUCH
        z_l0 = z_l1 - 0.0024
        ya, yb = -0.050, -0.016

        def cradle():
            plate = box3((x_p0, ya, -0.001), (xf - 0.0002, yb, z_l1))
            ledge = box3((float(lo[0]) - 0.001, float(lo[1]) - 0.003, z_l0), (x_p0 + OV, float(hi[1]) + 0.001, z_l1))
            cheek = box3((float(lo[0]) - 0.001, float(lo[1]) - 0.003, z_l1 - OV),
                         (float(lo[0]) + 0.012, float(lo[1]) - TOUCH, float(lo[2]) + 0.008))
            return finish(union([plate, ledge, cheek]))
        cid = self.pid("rid_cradle")
        self.add(cid, "Remote ID beacon cradle", "uzaktan kimlik vericisi beşiği", MAT_PA12, P_SLS, cradle,
                 thickness=0.0024, parent=fr, contacts=(fr,), explode=(-0.10, 0.0, 0.0), step=STEP_ANTENNA,
                 notes="SLS PA12 (RF-transparent) back plate 2.6 mm + ledge 2.4 mm + side cheek; beacon held with a "
                       "hook-and-loop pad and a cable tie through the ledge slot")
        self.unit("EQ-REMOTE_ID", "Dronetag Beacon (components.yaml dronetag_beacon, integral antennas ANT-RID)",
                  float(C.eq["EQ-REMOTE_ID"]["mass_kg"]), parent=cid, contacts=(cid,), step=STEP_ANTENNA,
                  explode=(-0.14, 0.0, 0.0))
        for k, y in enumerate((-0.0225, -0.044)):
            self.insert_bolt(f"{cid}-B{k + 1}", 3, (x_p0, y, 0.005), (1.0, 0.0, 0.0), [(cid, tp)], fr, t_fr,
                             step=STEP_ANTENNA, notes="cradle into a potted M3 insert of FS0300")

    # =================================================================================================================
    # mated connectors (plug + backshell) on every unit connector face
    # =================================================================================================================
    def connectors(self):
        C = self.C
        nums = iter(PLUG_NUMS)
        spec = [("EQ-PDU", (0.050, -0.050)), ("EQ-CONTACTOR_FUSES", (None,)), ("EQ-DCDC_28_12", (None,)),
                ("EQ-BUFFER_BATTERY", (None,)), ("EQ-AUTOPILOT", (None,)), ("EQ-DATALINK_PRIMARY", (None,)),
                ("EQ-DATALINK_BACKUP", (None,)), ("EQ-TRANSPONDER", (None,)), ("EQ-FTS_UNIT", (None,))]
        for eid, ys in spec:
            e = C.eq[eid]
            lo, hi = C.eqbox(eid)
            ew = np.asarray(e["envelope_with_connector"], float)
            face = e["connector"].split()[0]                     # "+x" / "-x"
            sgn = 1.0 if face.startswith("+") else -1.0
            xf = float(hi[0] if sgn > 0 else lo[0])
            allow = abs(float(ew[1][0] if sgn > 0 else ew[0][0]) - xf)
            L = allow - CONN_LEN_MARGIN
            h = float(hi[2] - lo[2])
            w = float(hi[1] - lo[1])
            zc = 0.5 * float(lo[2] + hi[2])
            r = 0.5 * min(0.75 * h, 0.75 * w / len(ys), 0.026)
            cs = [(0.5 * float(lo[1] + hi[1]) if y is None else y) for y in ys]
            pid = self.pid(next(nums))
            unit = e["part"]

            def mesh(xf=xf, sgn=sgn, L=L, r=r, cs=cs, zc=zc):
                ms = []
                for yc in cs:
                    p0 = np.array([xf + sgn * TOUCH, yc, zc])
                    ms.append(G.cylinder(r, p0, p0 + sgn * 0.6 * L * np.array([1.0, 0, 0]), n=32))
                    ms.append(G.cylinder(0.8 * r, p0 + sgn * (0.6 * L - OV) * np.array([1.0, 0, 0]),
                                         p0 + sgn * L * np.array([1.0, 0, 0]), n=32))
                return union(ms)
            self.add(pid, f"mated connector + backshell, {e['id']}", f"bağlı konnektör + arka kılıf, {e['id']}",
                     P_BUY, P_BUY, mesh, purchased=True, vendor="MIL-DTL-38999 series III plug + backshell (harness "
                     "side)", mass_kg=PLUG_MASS * len(cs), parent=unit, contacts=(unit,), step=STEP_HARNESS,
                     explode=(sgn * 0.06, 0.0, 0.0),
                     notes=f"{len(cs)} plug(s) on the {face} face inside the layout connector allowance "
                           f"({allow * 1000:.0f} mm); breakout cables to the trunk dressed at installation")
            self.harness_fixed[pid] = PLUG_MASS * len(cs)

    # =================================================================================================================
    # GNSS antennas on formed frame brackets
    # =================================================================================================================
    def gnss_bracket(self, key: str, aid: str, sid: str, z_screw: float, x_arm0: float):
        C = self.C
        a = C.ant[aid]
        p = np.asarray(a["point"], float)
        d, h = float(a["size"][0]), float(a["size"][2])
        z_ab = float(p[2]) - 0.5 * h                         # antenna base
        xf, _ = C.face(sid, -1)
        fr = C.stp(sid)
        t = 0.0016
        zc_arm = z_ab - TOUCH - 0.5 * t
        xw = xf - TOUCH - 0.5 * t
        z_fl0 = z_screw - 0.010
        prof = bent_profile([(x_arm0, zc_arm), (xw, zc_arm), (xw, z_fl0)], t, 3.0 * t)
        yc = float(p[1])
        wy = 0.016
        bid = self.pid(key)

        def mesh():
            m = prism_y(prof, yc - wy, yc + wy)
            return m
        self.add(bid, f"GNSS antenna bracket ({aid})", f"GNSS anten braketi ({aid})", MAT_AL, P_SHEET, mesh,
                 thickness=t, parent=fr, contacts=(fr,), step=STEP_ANTENNA, explode=(-0.06, 0.0, 0.05),
                 notes="6061-T6 1.6 mm, one 90 deg bend (inner radius 3 t); flange on the frame forward face with "
                       "2 x M3 into potted inserts, arm under the antenna (no carbon above it: RF window "
                       f"{a['window']})")
        ant = a["part"]
        self.add(ant, a["name"], a["name_tr"], P_BUY, P_BUY,
                 (lambda p=p, d=d, h=h: can(p - [0, 0, 0.5 * h], p + [0, 0, 0.5 * h], 0.5 * d, 0.007)),
                 purchased=True, vendor="Calian (Tallysman) HC977EXF embedded helical (components.yaml calian_hc977exf)",
                 mass_kg=0.008, parent=bid, contacts=(bid,), step=STEP_ANTENNA, explode=(0.0, 0.0, 0.08),
                 notes=f"{a['link']}; SSMA pigtail to the autopilot")
        for k, s in enumerate((1, -1)):
            self.insert_bolt(f"{bid}-B{k + 1}", 3, (xf - TOUCH - t, yc + s * (wy - 0.008), z_screw), (1.0, 0.0, 0.0),
                             [(bid, t + TOUCH)], fr, float(C.st[sid]["t"]), step=STEP_ANTENNA,
                             notes="GNSS bracket flange into a potted M3 insert of the frame")
        for k, s in enumerate((1, -1)):
            J.bolt(self.reg, f"{ant}-B{k + 1}", 3, (float(p[0]), yc + s * 0.0095, zc_arm - 0.5 * t), (0.0, 0.0, 1.0),
                   [(bid, t + TOUCH)], head="ISO 7380", nut="tapped", tapped_part=ant, tapped_depth=TAP_DEPTH, owner=bid,
                   step=STEP_ANTENNA, notes="antenna base thread (vendor)")

    def gnss(self):
        self.gnss_bracket("gnss1_brkt", "ANT-GNSS1", "FS1110", 0.080, 0.944)
        self.gnss_bracket("gnss2_brkt", "ANT-GNSS2", "FS-FUEL", 0.044, 2.064)

    # =================================================================================================================
    # nose-cone antennas on SLS pedestals bonded to the GFRP cone inner surface
    # =================================================================================================================
    def nose_antennas(self):
        C = self.C
        cone = C.panels["P-NOSECONE"]["part"]
        for aid, key, r_ped, vendor, mass in (
                ("ANT-C2A", "ped_c2a", 0.010, "Southwest Antennas 1001-202 omni dipole 2.1-2.5 GHz, TNC (components.yaml "
                 "southwest_antennas_1001_202)", 0.0227),
                ("ANT-FTS", "ped_fts", 0.008, "FTS command antenna, band per the selected FTS (estimate)", 0.010)):
            a = C.ant[aid]
            p = np.asarray(a["point"], float)
            d, h = float(a["size"][0]), float(a["size"][2])
            z_b = float(p[2]) - 0.5 * h
            ped = self.pid(key)

            def ped_mesh(p=p, r=r_ped, z_b=z_b):
                cav = C.cone_cavity(float(p[0]) - r - 0.004, float(p[0]) + r + 0.004, CONE_T + 0.0001)
                c = G.cylinder(r, (p[0], p[1], -0.2), (p[0], p[1], z_b - TOUCH), n=48)
                return finish(inter(c, cav))
            self.add(ped, f"antenna pedestal ({aid})", f"anten kaidesi ({aid})", MAT_PA12, P_SLS, ped_mesh,
                     thickness=2 * r_ped, parent=cone, contacts=(cone,), step=STEP_ANTENNA, explode=(-0.08, 0.0, -0.04),
                     notes="SLS PA12 (RF transparent), foot follows the cone inner surface, bonded (epoxy paste); "
                           "the antenna base nut clamps on its top face")
            self.add(a["part"], a["name"], a["name_tr"], P_BUY, P_BUY,
                     (lambda p=p, d=d, h=h: G.cylinder(0.5 * d, p - [0, 0, 0.5 * h], p + [0, 0, 0.5 * h], n=40)),
                     purchased=True, vendor=vendor, mass_kg=mass, parent=ped, contacts=(ped,), step=STEP_ANTENNA,
                     explode=(-0.10, 0.0, 0.0), notes=f"{a['link']}; coax pigtail through C-COAX-NC (service loop "
                                                       "for cone removal)")

    # =================================================================================================================
    # transponder blade on the belly with its bonded ground-plane doubler
    # =================================================================================================================
    def lower_point(self, x: float, y: float):
        """OML point and outward unit normal on the lower body surface at (x, y)."""
        fus = self.C.fus
        lo_, hi_ = 0.5 * math.pi + 1e-4, math.pi if y >= 0 else 1.5 * math.pi - 1e-4
        if y < 0:
            lo_, hi_ = math.pi, 1.5 * math.pi - 1e-4
        for _ in range(60):
            m = 0.5 * (lo_ + hi_)
            yy = float(fus.point(x, m)[1])
            if yy > y:
                lo_ = m
            else:
                hi_ = m
        ph = 0.5 * (lo_ + hi_)
        return np.asarray(fus.point(x, ph), float), np.asarray(fus.normal(x, ph), float)

    def transponder_blade(self):
        C = self.C
        a = C.ant["ANT-XPDR"]
        p = np.asarray(a["point"], float)
        L, W, H = (float(v) for v in a["size"])
        x0, x1 = float(p[0]) - 0.5 * L, float(p[0]) + 0.5 * L
        skin = C.panels["P-FWDSKIN"]["part"]
        z_b = float(p[2]) - 0.5 * H
        t_base, gap = 0.003, 0.00015
        fp = sbox(x0, -0.030, x1, 0.030)

        def blade():
            band = C.body_band(x0 - 0.002, x1 + 0.002, -t_base - gap, -gap, n=7)
            base = inter(band, prism_z(fp, -0.4, 0.0))
            zr = float(p[2]) + 0.5 * H
            fin = G.hull(np.array([[x0 + 0.003, -0.5 * W, zr], [x1 - 0.002, -0.5 * W, zr], [x0 + 0.003, 0.5 * W, zr],
                                   [x1 - 0.002, 0.5 * W, zr], [x1 - 0.002, -0.5 * W, z_b], [x1 - 0.032, -0.5 * W, z_b],
                                   [x1 - 0.002, 0.5 * W, z_b], [x1 - 0.032, 0.5 * W, z_b]]))
            fin = diff(fin, [self._oml_solid(x0 - 0.01, x1 + 0.01, -t_base - gap + OV)])
            return finish(union([base, fin]))
        bid = a["part"]
        self.add(bid, a["name"], a["name_tr"], P_BUY, P_BUY, blade, purchased=True,
                 vendor="L-band 1090 MHz blade antenna (allocation 60 x 8 x 70 mm, estimate 0.03 kg per components.yaml "
                        "avionics mass basis)", mass_kg=0.030, parent=skin, contacts=(skin,), step=STEP_ANTENNA,
                 explode=(0.0, 0.0, -0.10), notes=f"{a['notes'][:200]}")
        dbl = self.pid("xpdr_doubler")
        t_d = 0.0012

        def doubler():
            band = C.body_band(x0 - 0.014, x1 + 0.014, C.skin_t + 0.0001, C.skin_t + 0.0001 + t_d, n=7)
            return finish(inter(band, prism_z(sbox(x0 - 0.014, -0.044, x1 + 0.014, 0.044), -0.4, 0.0)))
        self.add(dbl, "transponder ground-plane doubler", "transponder zemin düzlemi takviyesi", MAT_PW, P_PREG,
                 doubler, thickness=t_d, parent=skin, contacts=(skin,), step=STEP_ANTENNA, explode=(0.0, 0.0, 0.06),
                 notes="bonded copper-mesh ground plane in a 4-ply PW doubler (1.2 mm) on the skin inner face, "
                       "bonded to the airframe earth")
        k = 0
        for dx in (-0.019, 0.019):
            for yb in (0.020, -0.020):
                k += 1
                q, nrm = self.lower_point(float(p[0]) + dx, yb)
                ax = -nrm
                # stack measured locally (the line would cross the opposite V face of the keel further in)
                iv = J.measure_stack(self.reg, [bid, skin, dbl], q, ax, 0.5 * FC.clearance(4) * 1.6, search=0.015)
                head = q + iv[0][1] * ax
                stack = [(pid_, e - s_) for pid_, s_, e in iv]
                stack[-1] = (stack[-1][0], stack[-1][1] + (iv[-1][2] - iv[0][1] - sum(t for _p, t in stack)))
                J.bolt(self.reg, f"{bid}-B{k}", 4, head, ax, stack, owner=bid, step=STEP_ANTENNA,
                       notes="blade base through the skin and the ground-plane doubler (normal to the "
                                     "skin), nyloc inside")

    def _oml_solid(self, x0, x1, inset):
        rings = []
        for x in np.linspace(x0, x1, 7):
            poly = Polygon(self.C.fus.point(np.full(256, x), np.linspace(0, 2 * math.pi, 256, endpoint=False))
                           [:, 1:3]).buffer(0)
            poly = SG.largest(poly.buffer(-inset, join_style=2)) if inset else poly
            R = SG.resample_ring(poly, 160, start_dir=(0.0, 1.0))
            rings.append(np.column_stack([np.full(len(R), x), R[:, 0], R[:, 1]]))
        return G.loft(rings)

    # =================================================================================================================
    # heated nose pitot-static probe in the bored GFRP cone tip
    # =================================================================================================================
    def pitot_nose(self):
        C = self.C
        it = C.adl["AD-PITOT-NOSE"]
        p0, p1 = np.asarray(it["p0"], float), np.asarray(it["p1"], float)
        r = 0.5 * float(it["diameter"])
        cone = C.panels["P-NOSECONE"]["part"]
        ax = G.unit(p1 - p0)
        self.reg.parts[cone].add_hole(p1 - 0.05 * ax, p1 + 0.002 * ax, r + TOUCH)
        self.add(it["part"], it["name"], it["name_tr"], P_BUY, P_BUY,
                 (lambda: G.cylinder(r, p0, p1, n=32)), purchased=True,
                 vendor="Aeroprobe heated pitot-static probe, 28 V / 42 W (components.yaml aeroprobe_heated_pitot_static)",
                 mass_kg=0.020, parent=cone, contacts=(cone,), step=STEP_ANTENNA, explode=(-0.15, 0.0, 0.0),
                 notes=f"{it['mount']}; bonded into the bore of the solid GFRP cone tip")

    # =================================================================================================================
    # harness: trunks, frame grommets, member penetration grommets, centre-line conduit, wing receptacles
    # =================================================================================================================
    def trunk_path(self, tid: str) -> np.ndarray:
        """Centre line of a trunk (starboard copy for mirrored branches): the layout path with detail changes inside
        the declared corridors (bundles are solid round sections here, the layout corridors are centre lines):

        * H-MAIN / H-COAX turn into the centre-line channel inside the 30 mm lateral duct (centre lines 4.6 / 6.2 mm
          aft of the layout x, bend radius 20 mm) and end at the aft-bay junction bracket CN-TAIL;
        * H-WING starts beside the trunk it branches from, passes the main-spar frame straight, crosses over the trunk
          before turning inboard, runs straight through the centre-box cover and ends at the CN-WING backshell;
        * H-TAIL / H-ENGINE start at CN-TAIL; H-TAIL passes FS3480 inside the ring opening (the fin-root lands of the
          ring start at |y| 0.110 above z 0.21);
        * H-NOSE runs through the FS0300 cut-out to 3 mm short of the forward face."""
        C = self.C
        P = np.asarray(C.trunk[tid]["path"], float).copy()
        J_ = self.junction
        if tid in ("H-MAIN", "H-COAX"):
            r = 0.5 * float(C.trunk[tid]["diameter"])
            xd = float(C.member_box("M-PAYWALL-AFT")[1][0]) + r + 0.0005       # duct: payload-bay aft wall + 0.5 mm
            x_duct = float(P[-3][0]) if tid == "H-MAIN" else float(P[-4][0])
            P[np.abs(P[:, 0] - x_duct) < 1e-6, 0] = xd
            i_g = int(np.argmin(np.abs(P[:, 0] - self.x_gear_vertex(tid))))
            yc, zc = J_["conn"][tid]
            P = np.vstack([P[:i_g + 1], [J_["x_f"] - 0.050, yc, zc], [J_["x_f"] - TOUCH, yc, zc]])
        elif tid == "H-WING":
            r_main = 0.5 * float(C.trunk["H-MAIN"]["diameter"])
            r_w = 0.5 * float(C.trunk[tid]["diameter"])
            y0 = abs(float(C.trunk["H-MAIN"]["path"][6][1])) + r_main + r_w + 0.0006
            cn = self.cn_wing_end
            P = np.array([[P[0][0], y0, P[0][2]], [P[1][0], y0 + 0.001, P[1][2]], P[2], [2.545, 0.3000, -0.097],
                          [2.570, 0.291, -0.078], [2.600, 0.250, -0.075], [2.620, 0.250, -0.052], [2.6357, 0.250, -0.010],
                          [2.670, 0.300, -0.013], P[8], P[9], [cn[0], cn[1] - 0.030, cn[2]], cn])
        elif tid == "H-TAIL":
            yc, zc = J_["conn"]["H-TAIL"]
            P = np.array([[J_["x_a"] + TOUCH, yc, zc], [J_["x_a"] + 0.025, yc, zc], [3.440, 0.082, 0.170],
                          [3.500, 0.092, 0.215], P[-1]])
        elif tid == "H-ENGINE":
            yc, zc = J_["conn"]["H-ENGINE"]
            P = np.array([[J_["x_a"] + TOUCH, yc, zc], [J_["x_a"] + 0.025, yc, zc], P[1], P[2]])
        elif tid == "H-NOSE":
            xf, _ = C.face("FS0300", -1)
            u = G.unit(P[-1] - P[-2])
            P[-1] = P[-2] + (xf - 0.003 - P[-2][0]) / u[0] * u
        return P

    def x_gear_vertex(self, tid: str) -> float:
        """x of the layout vertex just aft of FS-GEAR (the climb into the aft bay starts there)."""
        P = np.asarray(self.C.trunk[tid]["path"], float)
        xg = self.C.face("FS-GEAR", 1)[0]
        return float(min(P[P[:, 0] > xg][:, 0]))

    def corner_radius(self, tid: str):
        """Corner radii differing from 4 D: the turn into the centre-line channel (20 mm, inside the lateral duct) and
        the climb into the aft bay right behind FS-GEAR (40 mm)."""
        C = self.C
        if tid not in ("H-MAIN", "H-COAX"):
            return None
        x_gear = self.x_gear_vertex(tid)

        def f(b):
            if abs(b[1]) < 1e-9 and b[0] < 2.9:
                return 0.020
            if abs(b[0] - x_gear) < 1e-6:
                return 0.040
            return None
        return f

    def bundle(self, tid: str, grow: float = 0.0) -> G.Mesh:
        d = float(self.C.trunk[tid]["diameter"])
        Q = self.bundle_line(tid)
        return G.sweep_circle(Q, 0.5 * d + grow, n=24)

    def straight_through_frames(self, P: np.ndarray) -> np.ndarray:
        """A layout path vertex on a frame plane becomes a pass-through point: the bundle continues straight for 14 mm
        (frame + grommet flange) and bends after it, so that the bend stays clear of the cut-out edges."""
        C = self.C
        out = [P[0]]
        for i in range(1, len(P) - 1):
            V = P[i]
            out.append(V)
            for s_ in C.L["stations"]:
                if not s_.get("cutouts"):
                    continue
                xc = 0.5 * (C.face(s_["id"], -1)[0] + C.face(s_["id"], 1)[0]) + abs(V[1]) * C.face(s_["id"], -1)[1]
                if abs(V[0] - xc) < 0.015:
                    u = G.unit(V - P[i - 1])
                    out.append(V + 0.014 * u)
                    break
        out.append(P[-1])
        return np.asarray(out)

    def bundle_line(self, tid: str) -> np.ndarray:
        if tid not in self._lines:
            d = float(self.C.trunk[tid]["diameter"])
            P = self.straight_through_frames(self.trunk_path(tid))
            self._lines[tid] = densify(fillet(P, 4.0 * d, radius_of=self.corner_radius(tid)), 0.008)
        return self._lines[tid]

    def link(self, pid: str, other: str):
        """Declare a contact on ``pid`` (now, or when the part is created later: port copies)."""
        if pid in self.reg.parts:
            self.reg.parts[pid].contacts = tuple(dict.fromkeys(self.reg.parts[pid].contacts + (other,)))
        else:
            self.pending.setdefault(pid, []).append(other)

    def harness(self):
        C = self.C
        self._lines = {}
        self.pending = {}
        # wing receptacle (CN-WING) geometry first: the H-WING bundle ends at its backshell
        jr = C.member_box("M-JOINTRIB")
        y_rib = float(jr[0][1])
        cn = np.asarray(C.conn["CN-WING"]["point"], float)
        RC = dict(plate_t=0.002, fl=0.033, fl_t=0.002, r_body=0.0125, r_bs=0.011, bs_len=0.016, y_front=y_rib + 0.004)
        RC["y_p1"] = y_rib - 0.0001
        RC["y_p0"] = RC["y_p1"] - RC["plate_t"]
        RC["y_f1"] = RC["y_p0"] - TOUCH
        RC["y_f0"] = RC["y_f1"] - RC["fl_t"]
        RC["y_bs"] = RC["y_f0"] - RC["bs_len"]
        self.cn_wing_end = np.array([cn[0], RC["y_bs"] - TOUCH, cn[2]])
        # aft-bay junction bracket CN-TAIL (on TR-AFTBAY): H-MAIN / H-COAX end at its forward connectors, H-TAIL R/L
        # and H-ENGINE start at its aft connectors
        self.junction = dict(x_w0=3.2835, x_w1=3.2851, x_f=3.2735, x_a=3.2955,
                             conn={"H-MAIN": (0.0, 0.030), "H-COAX": (0.0, -0.025), "H-TAIL": (0.030, 0.045),
                                   "H-ENGINE": (-0.050, 0.010)})
        parents = {"H-MAIN": C.mem["M-MIDFLOOR"]["part"], "H-COAX": C.mem["M-MIDFLOOR"]["part"],
                   "H-WING": C.mem["M-CTBOX"]["part"], "H-TAIL": C.stp("FS3480"), "H-ENGINE": C.stp("FS3670"),
                   "H-NOSE": C.mem["M-DECK-NOSE"]["part"]}
        # integration round (IR-07): fuel level probe branches (layout H-FUEL in the spine channel, H-FUEL-A aft)
        fin_rr = "YK250-TL-255-R"                       # fin root rib (tail module; H-FIN, IR-06)
        for tid, par in (("H-FUEL", C.mem["M-SPINE"]["part"]), ("H-FUEL-A", C.stp("FS-GEAR")),
                         ("H-FIN", fin_rr if fin_rr in self.reg.parts else C.stp("FS3480"))):
            if tid in C.trunk:
                parents[tid] = par
        self.trunk_ids = {}
        for tid in parents:
            t = C.trunk[tid]
            pid = t["part"] + ("-R" if t.get("mirror") else "")
            self.trunk_ids[tid] = [pid] + ([t["part"] + "-L"] if t.get("mirror") else [])
            d = float(t["diameter"])
            self.add(pid, t["name"] + (", starboard" if t.get("mirror") else ""),
                     "kablo demeti " + tid + (", sağ" if t.get("mirror") else ""), P_BUY, P_BUY,
                     (lambda tid=tid: self.bundle(tid)), purchased=True,
                     vendor="harness assembly to the YK-250 wiring drawing (EN 2266 / AS50881 practice): SPEC 55 "
                            "wires, shielded twisted pairs, RG-316 / LMR-100 coax, PTFE sleeving, lacing",
                     mass_kg=0.0, parent=parents[tid], contacts=(), step=STEP_HARNESS,
                     explode=(0.0, 0.0, 0.12 if tid != "H-ENGINE" else -0.12),
                     notes=f"d {d * 1000:.0f} mm bundle on layout.systems.harness {tid}; " + t["text"][:240])
            self.harness_parts.append(pid)
        self.frame_grommets()
        self.member_grommets()
        self.spine_grommet()
        self.conduit()
        self.junction_bracket()
        self.aft_tray_passage()
        self.wing_receptacle(RC, cn, 1)
        self.wing_receptacle(RC, cn, -1)
        # port copies of the mirrored branches and of the H-WING penetration grommets
        self.fin_bores()
        for tid in [t for t in ("H-WING", "H-TAIL", "H-FIN") if t in self.trunk_ids]:
            pr, pl = self.trunk_ids[tid]
            q = mirror_part(self.reg.parts[pr], pl)
            q.contacts = ()
            self.reg.add(q)
            self.harness_parts.append(pl)
        if "H-FIN" in self.trunk_ids and "H-TAIL" in self.trunk_ids:     # IR-06: breakout of H-TAIL at its end
            for fi, ti in zip(self.trunk_ids["H-FIN"], self.trunk_ids["H-TAIL"]):
                self.link(fi, ti)
                self.link(ti, fi)
        self.dorsal_grommets()
        for g in self.member_grommets_R:
            q = mirror_part(self.reg.parts[g], g[:-1] + "L")
            q.parent = q.parent[:-1] + "L" if q.parent.endswith("-R") else q.parent
            q.contacts = (q.parent, self.trunk_ids["H-WING"][-1])
            self.reg.add(q)
            self.harness_fixed[q.id] = None
            self.link(self.trunk_ids["H-WING"][-1], q.id)
        for pid, others in self.pending.items():
            self.reg.parts[pid].contacts = tuple(dict.fromkeys(self.reg.parts[pid].contacts + tuple(others)))
        self.pending = {}

    def crossings(self, sid: str):
        """[(trunk id, side, point)] where a bundle centre line crosses the mid plane of frame ``sid``."""
        C = self.C
        xc = 0.5 * (C.face(sid, -1)[0] + C.face(sid, 1)[0])
        k = C.face(sid, -1)[1]
        out = []
        for tid in self.trunk_ids:
            Q = self.bundle_line(tid)
            for side in ((1, -1) if C.trunk[tid].get("mirror") else (1,)):
                Qs = Q * np.array([1.0, side, 1.0])
                f = Qs[:, 0] - (xc + np.abs(Qs[:, 1]) * k)
                for i in range(len(Qs) - 1):
                    if f[i] == 0 or f[i] * f[i + 1] < 0:
                        tt = f[i] / (f[i] - f[i + 1]) if f[i] != f[i + 1] else 0.0
                        out.append((tid, side, Qs[i] + tt * (Qs[i + 1] - Qs[i])))
        return out

    def chevron(self, poly, sid: str, t0: float, t1: float) -> G.Mesh:
        """Prism over the (y, z) polygon between the planes x = x_fwd_face(y) + t0 and + t1 (swept frames: sheared
        per half; the polygons here lie on one side of y = 0)."""
        C = self.C
        xf, k = C.face(sid, -1)
        m = prism_x(poly, xf + t0, xf + t1)
        if k:
            V = m.V.copy()
            V[:, 0] += np.abs(V[:, 1]) * k
            m = G.Mesh(V, m.F)
        return m

    def bundle_env(self, tid: str, side: int, grow: float = TOUCH) -> G.Mesh:
        key = (tid, side, grow)
        if key not in self._env:
            env = self.bundle(tid, grow)
            self._env[key] = env if side > 0 else env.mirrored_y()
        return self._env[key]

    def frame_grommets(self):
        """One grommet set per frame: a moulded split grommet in every harness cut-out a bundle crosses."""
        C = self.C
        n = N["grom0"]
        self._env = {}
        member_cut = []
        for m in C.L["chassis"]["members"]:
            if "box" not in m:
                continue
            lo, hi = np.asarray(m["box"][0], float), np.asarray(m["box"][1], float)
            member_cut.append(box3(lo - 0.0002, hi + 0.0002))
            if m.get("mirror"):
                member_cut.append(box3(np.array([lo[0], -hi[1], lo[2]]) - 0.0002,
                                       np.array([hi[0], -lo[1], hi[2]]) + 0.0002))
        xg = C.face("FS-GEAR", 1)[0]
        extra = {"FS-GEAR": [box3((xg, -0.11, -0.100), (xg + 0.010, 0.11, -0.050))]}   # TR-AFTBAY flange (notched)
        da_lo, da_hi = C.eqbox("EQ-DOORACT")          # inner-door actuators (+ mounting plates) on the aft face
        for s_ in (1, -1):
            y0_, y1_ = (float(da_lo[1]), float(da_hi[1])) if s_ > 0 else (-float(da_hi[1]), -float(da_lo[1]))
            extra["FS-GEAR"].append(box3((xg, y0_ - 0.004, float(da_lo[2]) - 0.004),
                                         (xg + 0.010, y1_ + 0.004, -0.050)))
        # inner-door drive bearing housings on the forward face, flanged round the C-DOORSHAFT cut-outs: the grommet
        # flange keeps 4 mm above the cut-out and 20 mm to the sides of it free on that face
        xgf = C.face("FS-GEAR", -1)[0]
        for mir in (False, True):
            y0_, y1_, z0_, z1_ = C.cutout("FS-GEAR", "C-DOORSHAFT", mir)
            extra["FS-GEAR"].append(box3((xgf - 0.010, y0_ - 0.020, z0_ - 0.010), (xgf, y1_ + 0.020, z1_ + 0.004)))
        # IR-07: C-HARN-FUELA opens into the aft fuel cell: the probe lead's bulkhead connector seals on the aft face,
        # no grommet flange / plug end on the forward (fuel-bay liner) face
        fa = [c for c in C.st["FS-GEAR"].get("cutouts", []) if c["id"] == "C-HARN-FUELA"]
        if fa:
            y0_, y1_, z0_, z1_ = C.cutout("FS-GEAR", "C-HARN-FUELA", False)
            extra["FS-GEAR"].append(box3((xgf - 0.010, y0_ - 0.010, z0_ - 0.010), (xgf, y1_ + 0.010, z1_ + 0.010)))
        for sid in [s_["id"] for s_ in C.L["stations"]]:
            st = C.st[sid]
            cuts = [c for c in st.get("cutouts", []) if c["id"].startswith(("C-HARN", "C-COAX", "C-FW-HARN"))]
            if not cuts:
                continue
            xs = self.crossings(sid)
            w = C.face(sid, 1)[0] - C.face(sid, -1)[0]
            items = []
            for c in cuts:
                for mir in ((False, True) if c.get("mirror") else (False,)):
                    y0, y1, z0, z1 = C.cutout(sid, c["id"], mir)
                    inside = [(tid, side, p) for tid, side, p in xs if y0 < p[1] < y1 and z0 < p[2] < z1]
                    if not inside:
                        continue
                    for tid, side, p in inside:
                        r = 0.5 * float(C.trunk[tid]["diameter"])
                        if not (y0 + r + 0.001 <= p[1] <= y1 - r - 0.001 and z0 + r + 0.001 <= p[2] <= z1 - r - 0.001):
                            raise ValueError(f"{tid} does not clear cut-out {sid}/{c['id']}: {p}")
                    items.append((c["id"] + ("-L" if mir else ""), (y0, y1, z0, z1), [(t_, s_) for t_, s_, _ in inside]))
            if not items:
                continue
            is_fw = sid == "FS3670"
            pid = self.pid("fw_grommet") if is_fw else self.pid(n)
            if not is_fw:
                n += 1
                if n > N["grom_ct"]:
                    raise ValueError("grommet part numbers overrun")

            def mesh(items=items, sid=sid, w=w):
                bodies = []
                for _cid, (y0, y1, z0, z1), bundles in items:
                    plug = sbox(y0 + GROM_CLR, z0 + GROM_CLR, y1 - GROM_CLR, z1 - GROM_CLR)
                    rim = sbox(y0 - GROM_RIM, z0 - GROM_RIM, y1 + GROM_RIM, z1 + GROM_RIM)
                    g = union([self.chevron(plug, sid, -TOUCH - GROM_FL, w + TOUCH + GROM_FL),
                               self.chevron(rim, sid, -TOUCH - GROM_FL, -TOUCH),
                               self.chevron(rim, sid, w + TOUCH, w + TOUCH + GROM_FL)])
                    cut = [self.bundle_env(t_, s_) for t_, s_ in bundles] + extra.get(sid, [])
                    lo, hi = g.bounds()
                    for mc in member_cut:
                        mlo, mhi = mc.bounds()
                        if np.all(mhi >= lo) and np.all(mlo <= hi):
                            cut.append(mc)
                    bodies.append(finish(diff(g, cut)))
                return G.merge(bodies) if len(bodies) > 1 else bodies[0]
            bids = []
            for _cid, _r, bundles in items:
                for t_, s_ in bundles:
                    ids = self.trunk_ids[t_]
                    bids.append(ids[0] if s_ > 0 else ids[-1])
            bids = list(dict.fromkeys(bids))
            fr = st["part"]
            cl = ", ".join(i[0] for i in items)
            nm = "fireproof harness grommet, firewall" if is_fw else f"harness grommet set {sid}"
            nm_tr = ("yangın perdesi yangına dayanıklı kablo geçiş rondelası" if is_fw
                     else f"kablo geçiş rondela takımı {sid}")
            self.add(pid, nm, nm_tr, P_BUY, P_BUY, mesh, purchased=True,
                     vendor=("fire-resistant silicone grommet with stainless retainer, harness in fire sleeve (ISO 2685 "
                             "fireproof pass-through)") if is_fw else
                     "moulded split silicone grommets (VMQ 60 Shore A), bonded, harness tied to them",
                     mass_kg=0.0, parent=fr, contacts=(fr,), step=STEP_HARNESS, explode=(0.0, 0.0, 0.10),
                     notes=f"cut-outs {cl} of {sid}: plug 0.2 mm inside the cut-out, {GROM_FL * 1000:.1f} mm flanges on "
                           "both web faces (trimmed at the members meeting the frame), holes for " + ", ".join(bids))
            self.harness_fixed[pid] = None
            for b in bids:
                self.link(pid, b)
                self.link(b, pid)

    def ring_grommet(self, pid, name, name_tr, member_part, point, axis, t_wall, bore_r, bundles, rim=0.0035,
                     trim=()):
        """Split ring grommet in a round bore of a panel (panel from ``point`` to point + t_wall * axis): plug 0.2 mm
        inside the bore, 1.5 mm flanges 3.5 mm beyond it on both faces, bundle holes."""
        a = G.unit(axis)
        p = np.asarray(point, float)

        def mesh():
            g = union([G.cylinder(bore_r - GROM_CLR, p - (TOUCH + GROM_FL) * a, p + (t_wall + TOUCH + GROM_FL) * a, n=40),
                       G.cylinder(bore_r + rim, p - (TOUCH + GROM_FL) * a, p - TOUCH * a, n=40),
                       G.cylinder(bore_r + rim, p + (t_wall + TOUCH) * a, p + (t_wall + TOUCH + GROM_FL) * a, n=40)])
            return finish(diff(g, [self.bundle_env(t_, s_) for t_, s_ in bundles] + list(trim)))
        bids = [self.trunk_ids[t_][0] if s_ > 0 else self.trunk_ids[t_][-1] for t_, s_ in bundles]
        self.add(pid, name, name_tr, P_BUY, P_BUY, mesh, purchased=True,
                 vendor="moulded split silicone grommet (VMQ 60 Shore A), sealed with polysulphide", mass_kg=0.0,
                 parent=member_part, contacts=(member_part,) + tuple(bids), step=STEP_HARNESS,
                 explode=(0.0, 0.0, 0.10), notes="layout.systems.harness penetration; holes for " + ", ".join(bids))
        self.harness_fixed[pid] = None
        for b in bids:
            self.link(b, pid)
        return pid

    def dorsal_grommets(self):
        """IR-06: H-FIN enters the fin root between the webs of the dorsal longeron's hat section through a bore in
        its crown (layout penetration on M-DORSAL, both sides): bore cut here, sealed split ring grommet (SY-795-R/L;
        flanges 1 mm, clear of the crown-web fillets; the bore lies 9 mm ahead of the web bolts of CH-092)."""
        C = self.C
        if "H-FIN" not in self.trunk_ids:
            return
        pen = next((p_ for p_ in C.trunk["H-FIN"].get("penetrations", []) if p_["member"] == "M-DORSAL"), None)
        if pen is None:
            return
        base = C.mem["M-DORSAL"]["part"]
        a = np.array([0.0, 0.0, 1.0])
        r_b = 0.5 * float(C.trunk["H-FIN"]["diameter"]) + 0.0012
        for k, (sd, fy) in enumerate((("R", 1.0), ("L", -1.0))):
            part = f"{base}-{sd}"
            if part not in self.reg.parts:
                continue
            q = np.asarray(pen["point"], float) * np.array([1.0, fy, 1.0])
            o = q - 0.03 * a
            (_r, r0, r1), = J.measure_stack(self.reg, [part], o, a, 0.0035, search=0.03)
            p0 = o + r0 * a
            t_w = r1 - r0
            self.reg.parts[part].add_hole(p0 - 0.003 * a, p0 + (t_w + 0.003) * a, r_b)
            self.ring_grommet(self.pid(795, sd), f"harness grommet, dorsal longeron crown (fin branch), "
                              f"{'starboard' if sd == 'R' else 'port'}",
                              f"kablo geçiş rondelası, sırt uzun kirişi tabanı (dikey kolu), {'sağ' if sd == 'R' else 'sol'}",
                              part, p0 - 0.0003 * a, a, t_w + 0.0006, r_b, [("H-FIN", 1 if sd == "R" else -1)],
                              rim=0.001)          # flanges 0.35 mm off the crown faces (crown slopes ~2 deg along x)
            self.harness_fixed[self.pid(795, sd)] = None

    def fin_bores(self):
        """IR-06: H-FIN runs up the fin box through bores in the tail ribs it crosses (bundle + 1.5 mm, moulded split
        grommets, drawing note) - cut here in the starboard parts and mirrored into the port ones."""
        if "H-FIN" not in self.trunk_ids:
            return
        bore = self.bundle("H-FIN", 0.0015)
        lo, hi = bore.bounds()
        man = bore.to_manifold()
        for pid, p in list(self.reg.parts.items()):
            if not pid.startswith("YK250-TL-") or not pid.endswith("-R"):
                continue
            plo, phi = p.mesh.bounds()
            if np.any(phi < lo) or np.any(plo > hi):
                continue
            v = man ^ p.mesh.to_manifold()
            if v.is_empty() or v.volume() < 1e-9:
                continue
            p.holes.append(bore)
            q = pid[:-1] + "L"
            if q in self.reg.parts:
                self.reg.parts[q].holes.append(bore.mirrored_y())

    def spine_grommet(self):
        """Integration round (IR-07): H-FUEL enters the dorsal spine channel through its floor in the mission bay
        (layout penetration on M-SPINE): bore in the channel floor (cut here) with a sealed split ring grommet; the
        branch starts as a breakout of H-COAX (declared contact)."""
        C = self.C
        if "H-FUEL" not in C.trunk:
            return
        t = C.trunk["H-FUEL"]
        pen = next(p_ for p_ in t["penetrations"] if p_["member"] == "M-SPINE")
        q = np.asarray(pen["point"], float)
        spine = C.mem["M-SPINE"]["part"]
        z0 = float(C.mem["M-SPINE"]["box"][0][2])
        fb = C.mem["M-SPINE"].get("boxes", [C.mem["M-SPINE"]["box"]])[0]
        t_fl = float(fb[1][2]) - float(fb[0][2])                   # channel floor band of the layout member
        r_b = 0.5 * float(t["diameter"]) + 0.001            # 0.8 mm plug wall
        p0 = np.array([q[0], q[1], z0 - 0.0002])
        # D-shaped flanges: trimmed 0.3 mm inboard of the channel's starboard wall (layout M-SPINE wall box)
        y_wall = min(float(b_[0][1]) for b_ in C.mem["M-SPINE"].get("boxes", []) if float(b_[0][1]) > 0.0)
        trim = [box3((q[0] - 0.02, y_wall - 0.0003, z0 - 0.02), (q[0] + 0.02, y_wall + 0.03, z0 + 0.04))]
        self.reg.parts[spine].add_hole(p0 - np.array([0.0, 0.0, 0.003]), p0 + np.array([0.0, 0.0, t_fl + 0.003]), r_b)
        self.ring_grommet(self.pid(779), "harness grommet, spine-channel floor (fuel probe branch)",
                          "kablo geçiş rondelası, sırt kanalı tabanı (yakıt algılayıcı kolu)", spine, p0, (0, 0, 1),
                          t_fl + 0.0004, r_b, [("H-FUEL", 1)], rim=0.002, trim=trim)
        self.harness_fixed[self.pid(779)] = None
        hf, hc = self.trunk_ids["H-FUEL"][0], self.trunk_ids["H-COAX"][0]
        self.link(hf, hc)
        self.link(hc, hf)

    def member_grommets(self):
        C = self.C
        pen = {p_["member"]: np.asarray(p_["point"], float) for p_ in C.trunk["H-WING"]["penetrations"]}
        box_lo, _ = C.member_box("M-CTBOX")
        t_cov = layup_props(self.spec, "ct_box_cover")["thickness"]
        p = pen["M-CTBOX"].copy()
        p[2] = float(box_lo[2])
        g1 = self.ring_grommet(self.pid("grom_ct", "R"), "harness grommet, centre-box lower cover, starboard",
                               "kablo geçiş rondelası, orta kutu alt kapağı, sağ", C.mem["M-CTBOX"]["part"], p,
                               (0, 0, 1), t_cov, 0.010, [("H-WING", 1)])
        sob = C.member_box("M-SOB")
        p = pen["M-SOB"].copy()
        p[1] = float(sob[0][1])
        g2 = self.ring_grommet(self.pid("grom_sob", "R"), "harness grommet, side-of-body rib, starboard",
                               "kablo geçiş rondelası, gövde yanı kaburgası, sağ", C.mem["M-SOB"]["part"] + "-R", p,
                               (0, 1, 0), float(sob[1][1] - sob[0][1]), 0.010, [("H-WING", 1)], rim=0.0015)
        gl = C.member_box("M-GLOVERIB")
        p = pen["M-GLOVERIB"].copy()
        p[1] = float(gl[0][1])
        glove = self.find_part("YK250-CH-", ("glove rib", "box piece", "starboard")) or C.mem["M-GLOVERIB"]["part"] + "-R"
        g3 = self.ring_grommet(self.pid("grom_glove", "R"), "harness grommet, glove rib, starboard",
                               "kablo geçiş rondelası, eldiven kaburgası, sağ", glove, p,
                               (0, 1, 0), float(gl[1][1] - gl[0][1]), 0.010, [("H-WING", 1)])
        self.member_grommets_R = (g1, g2, g3)
        # main-well forward wall: H-MAIN over H-COAX in one sealed two-hole grommet (aft flange inside the channel)
        ww = C.member_box("M-WELLWALL-FWD")
        x0w, x1w = float(ww[0][0]), float(ww[1][0])
        pts, rs = [], []
        for t in ("H-MAIN", "H-COAX"):
            pp = next(p_ for p_ in C.trunk[t]["penetrations"] if p_["member"] == "M-WELLWALL-FWD")
            pts.append(np.asarray(pp["point"], float))
            rs.append(0.5 * float(C.trunk[t]["diameter"]) + 0.001)          # wall bore: d + 2 mm (sealed entry)
        wid = self.pid("grom_well")
        y_in = float(C.member_box("M-WELLKEEL")[0][1]) - 0.0004

        def wmesh():
            g = []
            for q, r in zip(pts, rs):
                g.append(G.cylinder(r - GROM_CLR, (x0w - TOUCH - GROM_FL, q[1], q[2]), (x1w + TOUCH + GROM_FL, q[1], q[2]),
                                    n=40))
                g.append(G.cylinder(r + 0.0035, (x0w - TOUCH - GROM_FL, q[1], q[2]), (x0w - TOUCH, q[1], q[2]), n=40))
                g.append(G.cylinder(r + 0.0035, (x1w + TOUCH, q[1], q[2]), (x1w + TOUCH + GROM_FL, q[1], q[2]), n=40))
            m = union(g)
            cut = [self.bundle_env("H-MAIN", 1), self.bundle_env("H-COAX", 1),
                   box3((x1w, y_in, -0.3), (x1w + 0.01, 0.1, 0.0)), box3((x1w, -0.1, -0.3), (x1w + 0.01, -y_in, 0.0))]
            return finish(diff(m, cut))
        wall = C.mem["M-WELLWALL-FWD"]["part"]
        hm, hc = self.trunk_ids["H-MAIN"][0], self.trunk_ids["H-COAX"][0]
        self.add(wid, "sealed harness entry, main-well forward wall", "sızdırmaz kablo girişi, ana kuyu ön duvarı",
                 P_BUY, P_BUY, wmesh, purchased=True, vendor="moulded two-hole silicone grommet, sealed (polysulphide)",
                 mass_kg=0.0, parent=wall, contacts=(wall, hm, hc), step=STEP_HARNESS, explode=(0.0, 0.0, -0.10),
                 notes="H-MAIN over H-COAX through M-WELLWALL-FWD into the centre-line conduit")
        self.harness_fixed[wid] = None
        self.link(hm, wid)
        self.link(hc, wid)

    def conduit(self):
        """CFRP U-conduit (4 plies PW, 0.8 mm) closing the centre-line harness channel between the main wells:
        side walls bonded to the keel webs M-WELLKEEL, flanges under the well roof, floor under the coax tier."""
        C = self.C
        kw_lo, _ = C.member_box("M-WELLKEEL")
        roof_lo, _ = C.member_box("M-WELLROOF")
        ww = C.member_box("M-WELLWALL-FWD")
        t = 0.0008
        yo = float(kw_lo[1]) - 0.0001
        yi = yo - t
        zt = float(roof_lo[2]) - 0.0001
        rc = 0.5 * float(C.trunk["H-COAX"]["diameter"])
        zc = float(C.trunk["H-COAX"]["path"][-3][2])
        zb1 = zc - rc - 0.0015
        zb0 = zb1 - t
        x0 = float(ww[1][0]) + TOUCH + GROM_FL + 0.0005
        x1 = C.face("FS-GEAR", -1)[0] - TOUCH - GROM_FL - 0.0005
        cid = self.pid("conduit")
        prof = Polygon([(-yo, zt), (-yi, zt), (-yi, zb1), (yi, zb1), (yi, zt), (yo, zt), (yo, zb0), (-yo, zb0)])
        kw = C.mem["M-WELLKEEL"]["part"]
        self.add(cid, "centre-line harness conduit (between the main wells)", "orta hat kablo kanalı (ana kuyular arası)",
                 MAT_PW, P_PREG, (lambda: prism_x(prof, x0, x1)), thickness=t, parent=kw + "-R",
                 contacts=(kw + "-R", kw + "-L", C.mem["M-WELLROOF"]["part"]), step=STEP_HARNESS,
                 explode=(0.0, 0.0, -0.15),
                 notes="OOA prepreg U, 4 plies PW (0.8 mm), bonded to the keel webs (closes the channel towards the "
                       "wells: layout H-MAIN 'sealed CFRP conduit'); H-MAIN upper tier, H-COAX lower tier")

    def junction_bracket(self):
        """CN-TAIL: formed 6061 bracket on TR-AFTBAY (layout chassis.trays TR-AFTBAY 'tail connector bracket') with
        the bulkhead connectors that split H-MAIN / H-COAX into the tail branches and the engine harness."""
        C = self.C
        Jn = self.junction
        tray = C.trays["TR-AFTBAY"]["part"]
        z_tray = float(C.boxes["equipment_bay_aft"]["z"][0])
        t = 0.0016
        xm = Jn["x_w0"] + 0.5 * t
        zf = z_tray + TOUCH + 0.5 * t
        x_end = 3.308
        prof = bent_profile([(xm, 0.062), (xm, zf), (x_end, zf)], t, 3.0 * t)
        foot = Polygon([(Jn["x_w0"] + t + 3.0 * t, zf - 0.5 * t), (x_end, zf - 0.5 * t), (x_end, zf + 0.5 * t),
                        (Jn["x_w0"] + t + 3.0 * t, zf + 0.5 * t)])
        conns = [(tid, 1, Jn["conn"][tid]) for tid in ("H-MAIN", "H-COAX", "H-ENGINE")] + \
            [("H-TAIL", s_, (s_ * Jn["conn"]["H-TAIL"][0], Jn["conn"]["H-TAIL"][1])) for s_ in (1, -1)]
        rc = {tid: 0.5 * float(C.trunk[tid]["diameter"]) + 0.0025 for tid in Jn["conn"]}
        bid = self.pid(774)

        def bmesh():
            m = union([prism_y(prof, -0.0685, 0.045), prism_y(foot, -0.098, 0.098)])
            cuts = [cyl((Jn["x_w0"] - 0.001, y, z), (Jn["x_w1"] + 0.001, y, z), rc[tid] + 0.0003, 40)
                    for tid, _s, (y, z) in conns]
            return finish(diff(m, cuts))
        self.add(bid, "aft-bay connector bracket CN-TAIL", "arka bölme konnektör braketi CN-TAIL", MAT_AL, P_SHEET,
                 bmesh, thickness=t, parent=tray, contacts=(tray,), step=STEP_HARNESS, explode=(0.0, 0.0, 0.10),
                 notes="6061-T6 1.6 mm, web with five bulkhead connector holes, foot on TR-AFTBAY (3 x M4, nylocs)")
        for k, y in enumerate((0.0, 0.090, -0.090)):
            self.nyloc(f"{bid}-B{k + 1}", 4, (3.2995, y, zf + 0.5 * t), (0.0, 0.0, -1.0), [(bid, t + TOUCH), (tray, 0.002)],
                       step=STEP_HARNESS, notes="connector bracket foot through TR-AFTBAY")
        cid = self.pid(775)
        hids = []
        for tid, side, _yz in conns:
            ids = self.trunk_ids[tid]
            hids.append(ids[0] if side > 0 else ids[-1])

        def cmesh():
            ms = []
            for tid, _s, (y, z) in conns:
                ms.append(cyl((Jn["x_f"], y, z), (Jn["x_a"], y, z), rc[tid], 40))
                ms.append(cyl((Jn["x_w1"] + TOUCH, y, z), (Jn["x_w1"] + TOUCH + 0.0015, y, z), rc[tid] + 0.003, 40))
            return union(ms)
        self.add(cid, "connector set CN-TAIL (5 bulkhead receptacles + plugs)",
                 "konnektör takımı CN-TAIL (5 adet geçiş konnektörü)", P_BUY, P_BUY, cmesh, purchased=True,
                 vendor="MIL-DTL-38999 series III jam-nut receptacles with mated plugs and backshells (layout CN-TAIL)",
                 mass_kg=5 * PLUG_MASS, parent=bid, contacts=(bid,) + tuple(dict.fromkeys(hids)), step=STEP_HARNESS,
                 explode=(0.0, 0.0, 0.14), notes=C.conn["CN-TAIL"]["location"])
        self.harness_fixed[cid] = 5 * PLUG_MASS
        for h in dict.fromkeys(hids):
            self.link(h, cid)

    def aft_tray_passage(self):
        """H-MAIN and H-COAX climb from the centre-line channel through TR-AFTBAY into the aft bay: clearance notch in
        the tray's FS-GEAR flange and slots in the tray floor (cut here, holes of a part registered earlier) with
        split edge-protection sleeves in the floor slots."""
        C = self.C
        tray = C.trays["TR-AFTBAY"]["part"]
        z_tray = float(C.boxes["equipment_bay_aft"]["z"][0])
        xg = C.face("FS-GEAR", 1)[0]
        region = box3((xg - 0.0005, -0.04, -0.105), (3.25, 0.04, -0.045))
        for tid in ("H-MAIN", "H-COAX"):
            self.reg.parts[tray].holes.append(inter(self.bundle(tid, 0.0035), region))
        slab = box3((xg + 0.008, -0.04, z_tray - 0.002 - 0.0015), (3.25, 0.04, z_tray + 0.0015))
        sid = self.pid(776)

        def smesh():
            ms = []
            for tid in ("H-MAIN", "H-COAX"):
                ring_ = diff(self.bundle(tid, 0.0033), [self.bundle(tid, TOUCH)])
                ms.append(finish(inter(ring_, slab)))
            return G.merge(ms)
        hm, hc = self.trunk_ids["H-MAIN"][0], self.trunk_ids["H-COAX"][0]
        self.add(sid, "harness edge sleeves, TR-AFTBAY floor slots", "kablo kenar koruma kılıfları, TR-AFTBAY taban",
                 P_BUY, P_BUY, smesh, purchased=True, vendor="split nylon edge-protection grommet strip (caterpillar "
                 "grommet), bonded", mass_kg=0.0, parent=tray, contacts=(tray, hm, hc), step=STEP_HARNESS,
                 explode=(0.0, 0.0, 0.10), notes="lines the two floor slots cut in TR-AFTBAY for the climbing trunks")
        self.harness_fixed[sid] = None
        self.link(hm, sid)
        self.link(hc, sid)

    def wing_receptacle(self, RC, cn, side: int):
        C = self.C
        sfx = "R" if side > 0 else "L"
        rib = C.mem["M-JOINTRIB"]["part"] + "-" + sfx
        x0p, x1p, z0p, z1p = 2.6935, 2.7565, -0.034, 0.014
        cx, cz = float(cn[0]), float(cn[2])
        plate, rec = self.pid("cn_plate", sfx), self.pid("cn_wing", sfx)
        M = np.array([1.0, side, 1.0])

        def mir(m):
            return m if side > 0 else m.mirrored_y()

        def pmesh():
            m = box3((x0p, RC["y_p0"], z0p), (x1p, RC["y_p1"], z1p))
            return mir(diff(m, [cyl((cx, RC["y_p0"] - 0.001, cz), (cx, RC["y_p1"] + 0.001, cz), RC["r_body"] + 0.0005,
                                    48)]))
        self.add(plate, f"wing connector float plate, {'starboard' if side > 0 else 'port'}",
                 f"kanat konnektörü yüzer plakası, {'sağ' if side > 0 else 'sol'}", MAT_AL, P_CNC,
                 pmesh, thickness=RC["plate_t"], parent=rib, contacts=(rib,), step=STEP_HARNESS,
                 explode=(0.0, -0.06 * side, 0.0),
                 notes="6061-T6 2 mm on the inboard face of the joint rib (2 x M3 into potted inserts); the receptacle "
                       "floats +-0.5 mm in its bore (blind mate in the last 8 mm of the panel stroke)")

        def rmesh():
            fl = box3((cx - 0.5 * RC["fl"], RC["y_f0"], cz - 0.5 * RC["fl"]),
                      (cx + 0.5 * RC["fl"], RC["y_f1"], cz + 0.5 * RC["fl"]))
            body = cyl((cx, RC["y_f1"] - OV, cz), (cx, RC["y_front"], cz), RC["r_body"], 48)
            bs = cyl((cx, RC["y_bs"], cz), (cx, RC["y_f0"] + OV, cz), RC["r_bs"], 48)
            return mir(union([fl, body, bs]))
        hw = self.trunk_ids["H-WING"][0 if side > 0 else -1]
        self.add(rec, f"wing connector CN-WING (receptacle), {'starboard' if side > 0 else 'port'}",
                 f"kanat konnektörü CN-WING (dişi), {'sağ' if side > 0 else 'sol'}", P_BUY, P_BUY, rmesh,
                 purchased=True, vendor="MIL-DTL-38999 series III receptacle, 19 pins + 2 coax inserts, jam-nut, "
                 "with backshell (layout CN-WING type, estimate)", mass_kg=RECEPT_MASS, parent=plate,
                 contacts=(plate, hw), step=STEP_HARNESS, explode=(0.0, -0.08 * side, 0.0),
                 notes=C.conn["CN-WING"]["location"])
        self.harness_fixed[rec] = RECEPT_MASS
        self.link(hw, rec)
        t_rib = float(C.member_box("M-JOINTRIB")[1][1] - C.member_box("M-JOINTRIB")[0][1])
        for k, x in enumerate((cx - 0.024, cx + 0.024)):
            self.insert_bolt(f"{plate}-B{k + 1}", 3, M * (x, RC["y_p0"], cz), (0.0, float(side), 0.0),
                             [(plate, RC["y_p1"] - RC["y_p0"] + 0.0001)], rib, t_rib, step=STEP_HARNESS,
                             notes="float plate into a potted M3 insert of the joint rib")

    # =================================================================================================================
    # recovery: UAVOS 200 container, pads, girth strap, bridle legs + spools on the shackle pins, hatch latch
    # =================================================================================================================
    def parachute(self):
        C = self.C
        e = C.eq["EQ-PARACHUTE"]
        lo, hi = C.eqbox("EQ-PARACHUTE")
        floor = C.mem["M-PARAFLOOR"]["part"]
        z_fl = float(C.member_box("M-PARAFLOOR")[1][2])
        br = C.L["chassis"]["parachute"]["bridle"]
        leg_mass = {"forward_leg": 0.030 * float(br["forward_leg"]["length"]),
                    "aft_leg": 0.030 * float(br["aft_leg"]["length"])}      # aramid webbing ~30 g/m (estimate)
        # base pads (bonded): container rests on four 6061 pads with neoprene facing
        pad = self.pid("para_pad")
        pad_pts = [(float(lo[0]) + 0.030, float(lo[1]) + 0.030), (float(lo[0]) + 0.030, float(hi[1]) - 0.030),
                   (float(hi[0]) - 0.030, float(lo[1]) + 0.030), (float(hi[0]) - 0.030, float(hi[1]) - 0.030)]

        def pad_mesh():
            return G.merge([box3((x - 0.0125, y - 0.0125, z_fl + TOUCH), (x + 0.0125, y + 0.0125, float(lo[2]) - TOUCH))
                            for x, y in pad_pts])
        self.add(pad, "parachute container base pads (4x)", "paraşüt konteyneri taban pedleri (4 adet)", MAT_AL, P_CNC,
                 pad_mesh, thickness=float(lo[2]) - z_fl - 2 * TOUCH, parent=floor, contacts=(floor,),
                 step=STEP_PARA, explode=(0.0, 0.0, 0.25),
                 notes="6061-T6 25 x 25 x 5.9 mm pads bonded to M-PARAFLOOR, 1 mm neoprene facing (not drawn)")
        cont_mass = float(e["mass_kg"]) - sum(leg_mass.values())
        def cont_mesh():
            # compartment box with R 14 mm rounded fore / aft top edges (moulded container; keeps 10 mm to the hatch
            # locating tongues)
            R = 0.014
            prof = Polygon([(lo[0], lo[2]), (hi[0], lo[2]), (hi[0], hi[2] - R), (hi[0] - R, hi[2]), (lo[0] + R, hi[2]),
                            (lo[0], hi[2] - R)])
            arcs = [Point(lo[0] + R, hi[2] - R).buffer(R, 32), Point(hi[0] - R, hi[2] - R).buffer(R, 32)]
            prof = unary_union([prof] + arcs).intersection(sbox(lo[0], lo[2], hi[0], hi[2]))
            return prism_y(prof, float(lo[1]), float(hi[1]))
        cont = self.add(e["part"], e["name"], e["name_tr"], P_BUY, P_BUY, cont_mesh, purchased=True,
                        vendor="UAVOS 200 Parachute System (components.yaml uavos_200_parachute_system: 0.300 x 0.300 x "
                               "0.275 m, 4.5 kg incl. the Y-bridle, max load 220 kg, deploy <= 97 m/s)",
                        mass_kg=round(cont_mass, 4), parent=pad, contacts=(pad,), step=STEP_PARA,
                        explode=(0.0, 0.0, 0.45), notes="mouth under the dorsal hatch P-PARAHATCH; container mass = "
                                                        "system mass less the two modelled bridle legs").id
        # girth strap through the CH-114 strap brackets (webbing between bracket bridge and bay wall)
        st = self.pid("para_strap")
        wall_y = float(C.member_box("M-PARAWALL")[0][1])
        t_s = 0.0015
        yc_side = float(hi[1]) + TOUCH + 0.5 * t_s
        yc_brk = wall_y - 0.0005 - 0.5 * t_s
        jogs = ((1.545, 1.575), (1.725, 1.755))       # strap brackets CH-114 (x 1.56) and CH-123 (x 1.74), 24 mm + 3
        xc0, xc1 = float(lo[0]) - TOUCH - 0.5 * t_s, float(hi[0]) + TOUCH + 0.5 * t_s
        side = []
        for xb0, xb1 in jogs:
            side += [(xb0 - 0.015, yc_side), (xb0, yc_brk), (xb1, yc_brk), (xb1 + 0.015, yc_side)]
        pts = [(xc0, -yc_side)] + [(x, -y) for x, y in side] + [(xc1, -yc_side), (xc1, yc_side)] + \
            [(x, y) for x, y in side[::-1]] + [(xc0, yc_side), (xc0, -yc_side)]
        ring_ = LineString(pts).buffer(0.5 * t_s, cap_style=2, join_style=2, mitre_limit=2.0)
        zs0, zs1 = -0.090, -0.070
        self.add(st, "parachute container girth strap", "paraşüt konteyneri kuşak kayışı", P_BUY, P_BUY,
                 (lambda: prism_z(ring_, zs0, zs1)), purchased=True,
                 vendor="polyester webbing 20 mm with stainless cam buckle (strap kit, estimate 0.04 kg)", mass_kg=0.04,
                 parent=cont, contacts=(cont,), step=STEP_PARA, explode=(0.0, 0.0, 0.35),
                 notes="passes between the bridge and the wall of the four strap brackets (layout chassis.parachute "
                       "restraint); cam buckle on the forward face")
        # spools on the shackle pins and the stowed bridle legs
        fits = {"forward_leg": C.fit["F-RISER-FWD"], "aft_leg": C.fit["F-RISER-AFT"]}
        z_sp = 0.165                                                        # spine-floor top (slot between the strips)
        sp_ids = {}
        for leg, key, num in (("forward_leg", "spool_f", "spool_f"), ("aft_leg", "spool_a", "spool_a")):
            f = fits[leg]
            bs = f["bridle_spool"]
            q = np.asarray(f["point"], float)
            sid = self.pid(num)
            sp_ids[leg] = sid
            ro, ri, Ls = 0.5 * float(bs["od_m"]), 0.5 * float(bs["id_m"]), float(bs["length_m"])
            ears = [b_ for b_ in f["boxes"] if abs(float(b_[0][1])) < 0.01 and float(b_[1][2]) > 0.18]
            ye_out = max(max(abs(float(b_[0][1])), abs(float(b_[1][1]))) for b_ in ears)
            pin = self.pid(810 if leg == "forward_leg" else 811)
            self.add(pin, f"shackle pin ({f['id']})", f"kelepçe pimi ({f['id']})", "ti_6al_4v_annealed_sheet", P_CNC,
                     (lambda q=q, ye=ye_out: G.cylinder(0.004 - 0.00002, q - [0, ye - 0.0001, 0], q + [0, ye - 0.0001, 0],
                                                        n=40)),
                     thickness=0.008, parent=f["part"], contacts=(f["part"],), step=STEP_PARA, explode=(0.0, 0.08, 0.20),
                     notes="Ti-6Al-4V shackle pin d 8 f7, flush with the ear faces, retained by two internal circlips in "
                           "the ear bores (the fitting's bolt heads leave no room for a pin head / split pin)")
            self.add(sid, f"bridle spool ({f['id']})", f"kayış makarası ({f['id']})", "steel_4130_n", P_CNC,
                     (lambda q=q, ro=ro, ri=ri, Ls=Ls: G.tube(ro, ri, q - [0, 0.5 * Ls, 0], q + [0, 0.5 * Ls, 0], n=40)),
                     thickness=ro - ri, parent=pin, contacts=(pin,), step=STEP_PARA,
                     explode=(0.0, 0.0, 0.20), notes=bs["text"][:220])
        for leg, num, nm, nm_tr in (("forward_leg", "bridle_fwd", "parachute bridle forward leg (stowed)",
                                     "paraşüt Y-kayışı ön ayak (istiflenmiş)"),
                                    ("aft_leg", "bridle_aft", "parachute bridle aft leg (stowed in the spine channel)",
                                     "paraşüt Y-kayışı arka ayak (sırt kanalında)")):
            f = fits[leg]
            q = np.asarray(f["point"], float)
            r_in = 0.5 * float(f["bridle_spool"]["od_m"]) + TOUCH
            w = 0.0045
            lid = self.pid(num)
            fwd_pt = np.asarray(br["forward_leg"]["point"], float)
            aft_pt = np.asarray(br["aft_leg"]["point"], float)

            def leg_mesh(q=q, leg=leg, r_in=r_in, fwd_pt=fwd_pt, aft_pt=aft_pt):
                loop = G.tube(r_in + 0.0015, r_in, q - [0, 0.5 * w, 0], q + [0, 0.5 * w, 0], n=40)
                if leg == "forward_leg":         # flat fold resting in the slot between the base strips
                    tail = box3((q[0] + r_in, -0.5 * w, z_sp + 0.0005), (q[0] + 0.022, 0.5 * w, z_sp + 0.0045))
                    return union([loop, tail])
                x0 = fwd_pt[0] + 0.030
                x1 = aft_pt[0] - 0.030
                zc = z_sp + 0.008 + 0.0001
                stow = union([G.cylinder(0.008, (x0, 0, zc), (x1, 0, zc), n=32), G.sphere(0.008, (x0, 0, zc), n=24)])
                tail = box3((x1 - 0.004, -0.5 * w, z_sp + 0.0005), (q[0] - r_in + OV, 0.5 * w, z_sp + 0.0045))
                return union([loop, tail, stow])
            self.add(lid, nm, nm_tr, P_BUY, P_BUY, leg_mesh, purchased=True,
                     vendor="UAVOS Y-bridle leg, aramid webbing with sewn end loop (part of the UAVOS 200 system)",
                     mass_kg=round(leg_mass[leg], 4), parent=sp_ids[leg], contacts=(sp_ids[leg],), step=STEP_PARA,
                     explode=(0.0, 0.0, 0.30),
                     notes=f"layout chassis.parachute.bridle.{leg} (length {float(br[leg]['length']):.3f} m): sewn loop on "
                           "the spool; the free run to the confluence ring in the container mouth crosses FS1810 at "
                           "C-BRIDLE-F (open item: the notch is only a cap relief in the chassis frame)")
        # pin-puller hatch latch on a formed bracket on the FS1810 forward face
        fr = C.stp("FS1810")
        xf, _ = C.face("FS1810", -1)
        llo, lhi = C.eqbox("EQ-PARALATCH")
        t = SHEET_T
        za = float(llo[2]) - TOUCH - 0.5 * t
        xw = xf - TOUCH - 0.5 * t
        prof = bent_profile([(float(llo[0]) - 0.0005, za), (xw, za), (xw, za - 0.032)], t, R_AL)
        lb = self.pid("latch_brkt")
        self.add(lb, "parachute hatch latch bracket", "paraşüt kapağı mandal braketi", MAT_AL, P_SHEET,
                 (lambda: prism_y(prof, float(llo[1]), float(lhi[1]))), thickness=t, parent=fr, contacts=(fr,),
                 step=STEP_PARA, explode=(-0.08, 0.0, 0.10),
                 notes="6061-T6 2 mm, flange on the FS1810 forward face (2 x M4 into potted inserts), arm under the "
                       "latch (CH-006 in the layout mount field: the latch sits at the aft hatch edge on FS1810)")
        la = C.eq["EQ-PARALATCH"]["part"]
        llo_u = np.array([llo[0], -0.015, llo[2]])
        lhi_u = np.array([lhi[0], 0.015, llo[2] + 0.022])
        self.add(la, C.eq["EQ-PARALATCH"]["name"], C.eq["EQ-PARALATCH"]["name_tr"], P_BUY, P_BUY,
                 (lambda: enclosure(llo_u, lhi_u, WALLS)), purchased=True,
                 vendor="pin-puller hatch latch with cable-linked corner pins (estimate class, fired by the parachute "
                        "controller / FTS)", mass_kg=float(C.eq["EQ-PARALATCH"]["mass_kg"]), parent=lb, contacts=(lb,),
                 step=STEP_PARA, explode=(-0.10, 0.0, 0.14),
                 notes="30 x 30 x 22 mm unit inside the layout allocation box (10 mm to the hatch in every pose)")
        for k, y in enumerate((-0.010, 0.010)):
            J.bolt(self.reg, f"{la}-B{k + 1}", 3, (0.5 * float(llo[0] + lhi[0]), 0.8 * y, za - 0.5 * t), (0.0, 0.0, 1.0),
                   [(lb, t + TOUCH)], head="ISO 7380", nut="tapped", tapped_part=la, tapped_depth=TAP_DEPTH, owner=lb,
                   step=STEP_PARA, notes="latch base thread (vendor)")
            self.insert_bolt(f"{lb}-B{k + 1}", 4, (xf - TOUCH - t, y, za - 0.022), (1.0, 0.0, 0.0), [(lb, t + TOUCH)],
                             fr, float(C.st["FS1810"]["t"]), step=STEP_PARA,
                             notes="latch bracket into a potted M4 insert of FS1810")

    # =================================================================================================================
    # wing / tail mounted items (registered when their parents are in the registry)
    # =================================================================================================================
    def wing_tail_items(self):
        C = self.C
        # HASA pitot in the clamp ring of the wing module's pitot mast (port outer panel)
        it = C.adl["AD-PITOT-WING"]
        mast = self.find_part("YK250-WG-", ("pitot mast",))
        if mast is None:
            self.reg.note("systems: wing not registered - AD-PITOT-WING (YK250-SY-789) not created")
        else:
            p0, p1 = np.asarray(it["p0"], float), np.asarray(it["p1"], float)
            r = 0.5 * float(it["diameter"])
            self.add(it["part"], it["name"], it["name_tr"], P_BUY, P_BUY, (lambda: G.cylinder(r, p0, p1, n=32)),
                     purchased=True, vendor="PowerBox Pitot HASA, DroneCAN (components.yaml powerbox_pitot_hasa)",
                     mass_kg=0.048, parent=mast, contacts=(mast,), step=STEP_ANTENNA, explode=(-0.10, 0.0, -0.05),
                     notes=f"{it['mount']}; bonded in the mast clamp ring, cable through the mast (wing open item)")
        # wing-tip position / strobe lights against the tip-rib face, 2 x M5 into potted inserts
        lt = C.adl["LT-WING"]
        for side, sfx in ((1, "R"), (-1, "L")):
            rib = self.find_part("YK250-WG-", ("tip rib", "starboard" if side > 0 else "port"))
            fair = self.find_part("YK250-WG-", ("tip fairing", "starboard" if side > 0 else "port"))
            if rib is None or fair is None:
                if side > 0:
                    self.reg.note("systems: wing not registered - LT-WING lights (YK250-SY-790-R/L) not created")
                continue
            lo, hi = np.asarray(lt["box"][0], float), np.asarray(lt["box"][1], float)
            if side < 0:
                lo, hi = np.array([lo[0], -hi[1], lo[2]]), np.array([hi[0], -lo[1], hi[2]])
            pid = self.pid(int(lt["part"].split("-")[2]), sfx)
            colour = "green" if side > 0 else "red"
            self.add(pid, f"wing-tip position / strobe light ({colour}), {'starboard' if side > 0 else 'port'}",
                     f"kanat ucu seyir / çakar ışığı ({'yeşil' if side > 0 else 'kırmızı'}), "
                     f"{'sağ' if side > 0 else 'sol'}", P_BUY, P_BUY,
                     (lambda lo=lo, hi=hi: enclosure(lo, hi, dict(WALLS, zm=0.002, ym=0.004, yp=0.004))),
                     purchased=True, vendor="AveoFlashLP LSA 3-in-1 (components.yaml aveoflash_lp_lsa, stainless "
                     "screws)", mass_kg=0.083, parent=rib, contacts=(rib,), step=STEP_ANTENNA,
                     explode=(0.0, 0.10 * side, 0.0), notes=f"{lt['mount']}; in the tip-fairing pocket")
            # screws 38 mm apart (hole pattern: estimate) centred on the thickest part of the tip rib web (x/c ~ 0.25,
            # section >= 23 mm): >= 2.5 D edge distance in the composite rib (IR-09)
            for k, dx in enumerate((-0.006, 0.032)):
                q = np.array([0.5 * float(lo[0] + hi[0]) + dx, 0.5 * float(lo[1] + hi[1]), 0.5 * float(lo[2] + hi[2])])
                ax = np.array([0.0, -float(side), 0.0])
                (_r, r0, r1), = J.measure_stack(self.reg, [rib], q, ax, 0.0036, search=0.08)
                J.bolt_through(self.reg, f"{pid}-B{k + 1}", 4, q, ax, [pid], owner=pid, nut="insert", insert_part=rib,
                               insert_depth=r1 - r0, step=STEP_ANTENNA,
                               notes="light through its mounting holes (M4 per the installation drawing, stainless) "
                                     "into through-thickness potted inserts of the tip rib")
        self.fin_items()

    def fin_items(self):
        """Integration round (IR-06): ANT-BKP (starboard) / ANT-C2B (port), Southwest Antennas 1001-202 dipoles along
        the fin span on the tip rib inside the GFRP tip caps (TNC bulkhead jack through the rib, jam nut under it), and
        LT-TAIL, the AveoFlash LSA tail light on the tip face of the starboard cap (2 x M4 into the potted inserts of
        the tail module's GFRP mounting block TL-292); coax / supply in H-FIN."""
        C = self.C
        if not any(pid.startswith("YK250-TL-258") for pid in self.reg.parts):
            return
        for aid, sd in (("ANT-BKP", "R"), ("ANT-C2B", "L")):
            a = C.ant.get(aid)
            if a is None or "axis" not in a:
                continue
            rib, cap = f"YK250-TL-264-{sd}", f"YK250-TL-258-{sd}"
            ax = G.unit(np.asarray(a["axis"], float))
            base = np.asarray(a["base"], float)
            L, d = float(a["length"]), float(a["diameter"])
            (_r, r0, r1), = J.measure_stack(self.reg, [rib], base, -ax, 0.009, search=0.03)
            top, bot = base - r0 * ax, base - r1 * ax                 # rib faces on the antenna axis
            r_jack = 0.006

            def amesh(top=top, bot=bot, ax=ax, L=L, d=d):
                body = G.cylinder(0.5 * d, top + 0.0001 * ax, top + L * ax, n=40)
                stub = G.cylinder(r_jack, bot - 0.004 * ax, top + 0.0002 * ax, n=32)
                nut = G.cylinder(0.009, bot - 0.004 * ax, bot - 0.0001 * ax, n=6)
                return G.union([body, stub, nut])
            self.reg.parts[rib].add_hole(top + 0.002 * ax, bot - 0.002 * ax, r_jack + 0.0005)
            jid = getattr(self, "trunk_ids", {}).get("H-FIN", [None, None])[0 if sd == "R" else -1]
            self.add(a["part"], a["name"], a["name_tr"], P_BUY, P_BUY, amesh, purchased=True,
                     vendor="Southwest Antennas 1001-202 omni half-wave dipole 2.1-2.5 GHz, TNC(m) "
                            "(components.yaml southwest_antennas_1001_202) + TNC bulkhead jack and jam nut",
                     mass_kg=0.0227 + 0.012, parent=rib, contacts=(rib,), step=STEP_ANTENNA,
                     explode=tuple((0.12 * ax).tolist()),
                     notes=f"{a['link']}; {a.get('mount', '')}; RF window: GFRP tip cap {cap}; coax in H-FIN"
                           + (f" ({jid})" if jid else "") + "; jack mass 0.012 kg (estimate)")
        lt = next((q for q in C.L["systems"]["air_data_lights"] if q["id"] == "LT-TAIL"), None)
        blk = "YK250-TL-292-R"
        if lt is None or "frame" not in lt or blk not in self.reg.parts:
            return
        fr = lt["frame"]
        Rm = np.asarray(fr["axes"], float).T                         # columns: chord, thickness normal, span
        hc = np.asarray(fr["half"], float)
        pc = np.asarray(fr["center"], float)
        lid = self.pid(int(lt["part"].split("-")[2]))
        self.add(lid, "tail position / strobe light (white, aft-facing)", "kuyruk seyir / çakar ışığı (beyaz)",
                 P_BUY, P_BUY, (lambda: G.box(tuple((2 * hc).tolist()), center=pc, R=Rm)), purchased=True,
                 vendor="AveoFlashLP LSA 3-in-1 (components.yaml aveoflash_lp_lsa, stainless screws)", mass_kg=0.083,
                 parent="YK250-TL-258-R", contacts=("YK250-TL-258-R",), step=STEP_ANTENNA,
                 explode=tuple((0.10 * Rm[:, 2]).tolist()), notes=lt["mount"] + "; supply in H-FIN (starboard)")
        w = Rm[:, 2]
        for k, dc in enumerate((-0.019, 0.019)):
            q = pc + dc * Rm[:, 0] + hc[2] * w
            (_r, r0, r1), = J.measure_stack(self.reg, [blk], q, -w, 0.0036, search=0.08)
            J.bolt_through(self.reg, f"{lid}-B{k + 1}", 4, q, -w, [lid, "YK250-TL-258-R"], owner=lid, nut="insert",
                           insert_part=blk, insert_depth=r1 - r0, step=STEP_ANTENNA,
                           notes="light through its mounting holes and the cap tip face into potted inserts of the "
                                 "mounting block (M4 as the wing-tip units; hole spacing 38 mm along the chord: "
                                 "estimate, the datasheet gives no pattern)")

    # =================================================================================================================
    # harness mass: cables 70 % by length x d^2, connectors / grommets / breakouts 30 % of the layout item
    # =================================================================================================================
    def harness_masses(self):
        C = self.C
        total = HARNESS_BASE_KG
        cables = CABLE_SHARE * total
        w = {}
        for pid in self.harness_parts:
            tid = next(t for t, ids in self.trunk_ids.items() if pid in ids)
            Q = self.bundle_line(tid)
            w[pid] = float(np.sum(seg_len(Q))) * float(C.trunk[tid]["diameter"]) ** 2
        b2 = 0.5 * float(self.spec["wing"]["span"])
        yj = float(self.spec["wing"]["planform"]["y_junction"])
        for pid in self.trunk_ids["H-WING"]:
            w[pid] += (b2 - yj) * 0.012 ** 2                  # outer-panel harness (to the tip) carried in the branch
        W = sum(w.values())
        fixed = 0.0
        for pid, m in self.harness_fixed.items():
            if m is None:
                m = round(self.reg.parts[pid].mesh.volume() * GROM_RHO, 4)
                self.reg.parts[pid].mass_kg = m
            fixed += m
        rest = (1.0 - CABLE_SHARE) * total - fixed
        if rest < 0:
            raise ValueError(f"harness connector share exceeded by {-rest:.3f} kg")
        Ltot = sum(w.values())
        for pid in self.harness_parts:
            m = cables * w[pid] / W + rest * w[pid] / Ltot
            self.reg.parts[pid].mass_kg = round(m, 4)
            self.reg.parts[pid].notes += (f"; mass {m:.3f} kg = cables (70 % of {total:.2f} kg by length x d^2, incl. the "
                                          "outer-panel harness for H-WING) + breakout connectors / ties (rest of the 30 %)")


def register(reg: Registry, spec: dict) -> None:
    if any(p.group == "systems" for p in reg.parts.values()):
        reg.note("systems: already registered, second call ignored")
        return
    Systems(reg, spec).build()
