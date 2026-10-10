"""Wing producer (YK-250 HANÇER): the two removable outer wing panels (y 0.70 .. 3.60 m each side), their ailerons and
flaps with hinges and actuation, and the outer-panel half of the wing joint.

What it builds (part numbers from ``spec.layout.part_numbers.wing``: WG 150-199 structure, FC 200-249 controls; the
starboard panel is modelled in +y and mirrored with ``core.parts.mirror_part``):

* the CFRP main spar with its integral joint tongue (YK250-WG-151, layout ``chassis.wing_joint.main_spar.tongue``): UD
  caps 30 mm wide sized by ``structures.sizing.wing.main_cap.zones`` (cap outer face 1 mm under the OML), +-45 PW web
  (``main_web.zones``), the 0.10 m root-bay depth transition to the 30 x 61 mm tongue (UD flanges 10 -> 3 mm, 25-ply
  web, [+-45/0/90] boss blocks) and its bonded 4130 bush pair; the two Ti main pins P-MAIN1/2 with their keeper
  plates (2 x M4 into potted inserts of the fork front pad), the rear-spar root fitting with the 8 mm 7075 lug
  (YK250-WG-152) and the Ti ball-lock rear pin P-REAR;
* the CFRP C-section rear spar (web aft face on a straight line just forward of the control-surface coves), the
  flanged sandwich ribs (root, bay, servo-bay and tip ribs; nose and box pieces), the sandwich skins split into a
  leading-edge (D-nose) skin and upper / lower box skins (core ramped to 1 mm solid laminate over the spar caps, at
  the nose and at the cove lips), the fixed trailing-edge closures between the moving surfaces, the GFRP tip fairing,
  the servo hatch frames and covers (with the linkage fairings), the harness conduit and the drain holes;
* the aileron and the flap (YK250-FC-200 / 202: round nose in a cove, ``structgen.control_surface_regions``; foam-core
  CFRP shells with end faces normal to the hinge axis), their 7075 clevis hinge brackets on the rear-spar web, bonded
  hinge tongues and pins on the hinge lines of ``layout.mechanisms.joints``; the Volz DA 26 / DA 30 actuators
  (YK250-FC-204 / 206) on their hatch covers, the servo arms, push-rods and horns (four-bar linkages of
  ``wing.controls.*.linkage``, solved with ``actuation.linkage_kinematics``) and the coupled joints / sequences that
  move the linkage with the surface.

Interfaces read (never another module's geometry): ``spec.layout`` (part_numbers, chassis.wing_joint: plane, insertion
axis, pins, fork slot, rear lug / pin; mechanisms.joints aileron_* / flap_*; systems.actuators ACT-AILERON / ACT-FLAP;
systems.harness CN-WING; systems.air_data_lights LT-WING; keep_outs KO-WINGJOINT-PATH), ``spec.wing`` (OML sections,
planform, controls), ``spec.structures.sizing.wing`` / ``wing_joint`` (sized plies and joint dimensions),
``spec.layups`` / ``materials`` / ``processes``, ``data/research/components.yaml`` (actuator dimensions).

Module-private detailing constants are listed below and in ``docs/detail/wing.md``.
"""
from __future__ import annotations

import functools
import math
from pathlib import Path

import numpy as np
import yaml
from shapely.geometry import LineString, MultiPolygon, Point, Polygon
from shapely.geometry import box as sbox
from shapely.ops import unary_union

from ..core import geom as G
from ..core.parts import Joint, Part, Registry, layup_props, mirror_part, part_number
from . import joints as J
from . import oml as O
from . import structgen as SG

# ---------------------------------------------------------------------------------------------------------------------
# module-private detailing constants (docs/detail/wing.md)
# ---------------------------------------------------------------------------------------------------------------------
BOND = 1.0e-4               # modelled bond line between bonded parts (EA 9394; real 0.1-0.2 mm)
OV = 2.0e-4                 # boolean overlap of fused features (ARCHITECTURE §5: >= 0.1 mm)
T_SOLID = 0.0010            # solid skin laminate (5 plies PW) over spar caps / flanges, at the nose and the cove lips
RAMP = 0.015                # core ramp 1:3 (5 mm over 15 mm)
W_CAP = 0.030               # main-spar cap width (structures.sizing.wing.main_cap.width_outer_m)
W_RFL = 0.025               # rear-spar flange width = rear cap width (structures.sizing.wing.rear_cap.width_m)
T_RIB = 0.0068              # rib_panel sandwich
RIB_FL_W = 0.015            # rib T-flange width each side of the web
RIB_FL_T = 0.0016           # rib flange (solid edge band, 8 plies)
COVE_GAP = 0.0035           # cove gap: >= 3 mm moving-surface clearance (layout.clearances) + 0.5 mm
END_GAP = 0.0035            # control-surface end gaps (planes normal to the hinge axis)
WEB_TO_COVE = 0.0005        # rear-spar web aft face set back from the cove circle
NOSE_SOLID = 0.012          # chord fraction of the solid nose laminate
CS_SKIN_T = 0.0006          # control-surface skins: 3 plies PW over a ROHACELL 51 WF core (full depth)
ROOT_RIB_Y = 0.7049         # root rib inboard face = joint plane 0.70 + chassis joint-rib half thickness + 1.5 mm seal
TIP_RIB_Y1 = 3.5399         # tip rib outboard face (0.1 mm inboard of the LT-WING light box, layout.systems)
STEP = 35                   # assembly step: outer wing panels (spec.assembly)
STEP_RIG = 37               # assembly step: control rigging (spec.assembly)

GROUP_W, GROUP_C = "wing", "controls"
MAT_UD, MAT_PW, MAT_GF = "cfrp_ud_mtm45_as4", "cfrp_pw_mtm45_as4", "gfrp_7781_mtm45"
MAT_7075, MAT_4130, MAT_TI = "al_7075_t651_plate", "steel_4130_n", "ti_6al_4v_annealed_sheet"
CORE51, CORE71 = "core_rohacell_51wf", "core_rohacell_71wf"
P_PREG, P_CNC = "prepreg_ooa_vacbag", "cnc_milling_metal"


def _unit(v):
    return G.unit(v)


def _components() -> dict:
    p = Path(__file__).resolve().parents[1] / "data" / "research" / "components.yaml"
    with open(p, encoding="utf-8") as fh:
        return yaml.safe_load(fh)


@functools.lru_cache(maxsize=None)
def actuator_data(key: str) -> dict:
    """Datasheet record of a Volz actuator from data/research/components.yaml (value fields flattened)."""
    def walk(o):
        if isinstance(o, dict):
            if o.get("id") == key and "model" in o:
                return o
            for v in o.values():
                r = walk(v)
                if r is not None:
                    return r
        elif isinstance(o, list):
            for v in o:
                r = walk(v)
                if r is not None:
                    return r
        return None
    rec = walk(_components())
    if rec is None:
        raise KeyError(key)

    def val(x):
        return x["value"] if isinstance(x, dict) and "value" in x else x
    out = {k: val(v) for k, v in rec.items() if not isinstance(v, dict) or "value" in v}
    out["mounting"] = {k: val(v) for k, v in (rec.get("mounting") or {}).items()}
    return out


# =====================================================================================================================
# outer-panel geometry
# =====================================================================================================================
class OP:
    """Starboard outer-panel geometry: span coordinate (eta along the dihedral line), exact OML section curves in each
    section frame (s along the chord from the leading edge, t along the local up), skin thickness profiles, spar lines,
    hinge lines and coves. Everything is derived from spec.wing / spec.layout / spec.structures."""

    def __init__(self, spec: dict):
        self.S = spec
        W = spec["wing"]
        self.W = O.LiftingSurface(W["sections"], n_chord=120, name="wing")
        self._e = self.W.span_coords()
        self._y = np.array([float(s["y"]) for s in W["sections"]])
        self.y_j = float(W["planform"]["y_junction"])
        self.eta_end = float(self._e[-1])
        L = spec["layout"]
        self.L = L
        self.wj = L["chassis"]["wing_joint"]
        self.jn = {j["name"]: j for j in L["mechanisms"]["joints"]}
        self.ctl = W["controls"]
        sz = spec["structures"]["sizing"]
        self.sz_w, self.sz_j = sz["wing"], sz["wing_joint"]
        mats = spec["materials"]
        self.t_ud = float(mats[MAT_UD]["ply_t"])
        self.t_pw = float(mats[MAT_PW]["ply_t"])
        self.t_skin_primary = float(layup_props(spec, "wing_skin_primary")["thickness"])
        self.t_skin_box_up = float(layup_props(spec, "wing_box_skin_upper")["thickness"])
        self.y_box_up_end = float(self.sz_w["box_skin_upper_y_end_m"])
        self._cv = {}
        self._rear_line()

    # ------------------------------------------------------------------ span coordinate
    def eta(self, y: float) -> float:
        return float(np.interp(y, self._y, self._e))

    def y_of(self, eta: float) -> float:
        return float(np.interp(eta, self._e, self._y))

    def chord(self, eta: float) -> float:
        return float(self.W.chord_at(eta))

    def frame(self, eta: float):
        o, c, u, w = self.W.frame_at(eta)
        return o, c, u, w

    # ------------------------------------------------------------------ OML curves
    def curve(self, eta: float, side: str) -> dict:
        """Dense OML curve of the section at eta, LE -> TE on the ``side`` ('up' / 'lo'): arc length sig, s, t and the
        unit inward normal (n_s, n_t) in the section frame."""
        key = (round(eta, 7), side)
        if key not in self._cv:
            o, c, u, w = self.frame(eta)
            n = 240
            P = self.W.loop_at(eta, n)
            d = P - o
            st = np.column_stack([d @ c, d @ u])
            up = st[:n][::-1]
            lo = st[n - 1:]
            pts = up if side == "up" else lo
            seg = np.linalg.norm(np.diff(pts, axis=0), axis=1)
            keep = np.r_[True, seg > 1e-9]
            pts = pts[keep]
            sig = np.r_[0.0, np.cumsum(np.linalg.norm(np.diff(pts, axis=0), axis=1))]
            T = np.gradient(pts, sig, axis=0)
            T /= np.linalg.norm(T, axis=1, keepdims=True)
            if side == "up":
                N = np.column_stack([T[:, 1], -T[:, 0]])
            else:
                N = np.column_stack([-T[:, 1], T[:, 0]])
            self._cv[key] = {"sig": sig, "s": pts[:, 0], "t": pts[:, 1], "ns": N[:, 0], "nt": N[:, 1],
                             "frame": (o, c, u, w)}
        return self._cv[key]

    def sig_of_s(self, eta: float, side: str, s) -> np.ndarray:
        cv = self.curve(eta, side)
        return np.interp(s, cv["s"], cv["sig"])

    def at_sig(self, eta: float, side: str, sig):
        """(s, t, n_s, n_t) on the OML curve at arc lengths ``sig``."""
        cv = self.curve(eta, side)
        sig = np.asarray(sig, float)
        s = np.interp(sig, cv["sig"], cv["s"])
        t = np.interp(sig, cv["sig"], cv["t"])
        ns = np.interp(sig, cv["sig"], cv["ns"])
        nt = np.interp(sig, cv["sig"], cv["nt"])
        nn = np.hypot(ns, nt)
        return s, t, ns / nn, nt / nn

    def t_oml(self, eta: float, side: str, s):
        cv = self.curve(eta, side)
        return np.interp(s, cv["s"], cv["t"])

    def to3d(self, eta: float, s, t, proj_y: float | None = None) -> np.ndarray:
        o, c, u, _w = self.frame(eta)
        s = np.atleast_1d(np.asarray(s, float))
        t = np.atleast_1d(np.asarray(t, float))
        P = o + s[:, None] * c + t[:, None] * u
        if proj_y is not None:
            P[:, 1] = proj_y
        return P

    def vec3d(self, eta: float, vs, vt) -> np.ndarray:
        _o, c, u, _w = self.frame(eta)
        vs = np.atleast_1d(np.asarray(vs, float))
        vt = np.atleast_1d(np.asarray(vt, float))
        return vs[:, None] * c + vt[:, None] * u

    def s_of_x(self, eta: float, x: float) -> float:
        """Chord coordinate s at which the section chord line of ``eta`` reaches plan x."""
        o, c, _u, _w = self.frame(eta)
        return float((x - o[0]) / c[0])

    # ------------------------------------------------------------------ spar lines
    def s_main(self, eta: float) -> float:
        return 0.25 * self.chord(eta)

    def hinge_cs(self, eta: float) -> tuple[float, float, float]:
        """(s_H, t_H, r) of the 0.75-chord mid-thickness hinge line at eta (structgen.control_surface_regions with
        hinge_frac 0.5; r = half thickness = nose radius)."""
        ch = self.chord(eta)
        s = 0.75 * ch
        tu = float(self.t_oml(eta, "up", s))
        tl = float(self.t_oml(eta, "lo", s))
        th = 0.5 * (tu + tl)
        return s, th, 0.5 * (tu - tl)

    def _rear_line(self):
        """Rear-spar web aft face: a straight plan line x = a + b y that keeps the web WEB_TO_COVE forward of every
        cove circle (radius r + COVE_GAP about the hinge line) from the root to the tip rib."""
        ys = np.linspace(self.y_j, TIP_RIB_Y1, 60)
        lim = []
        for y in ys:
            e = self.eta(y)
            sH, _tH, r = self.hinge_cs(e)
            s_lim = sH - r - COVE_GAP - WEB_TO_COVE
            p = self.to3d(e, [s_lim], [0.0])[0]
            lim.append(p[0])
        lim = np.asarray(lim)
        # straight line through the two end limits, then lowered to the tightest intermediate limit
        b = (lim[-1] - lim[0]) / (ys[-1] - ys[0])
        a = float(np.min(lim - b * ys))
        self.rw_a, self.rw_b = a, float(b)

    def x_rear(self, y: float) -> float:
        return self.rw_a + self.rw_b * y

    def s_rear(self, eta: float) -> float:
        """Chord coordinate of the rear-spar web aft face (on the chord line)."""
        return self.s_of_x(eta, self.x_rear(self.y_of(eta)))


    # ------------------------------------------------------------------ hinge lines and coves (layout.mechanisms.joints)
    def hinge(self, which: str) -> dict:
        """Hinge line of 'flap' / 'aileron' (starboard joint of the layout): origin, unit axis, span ends (eta) of
        the moving surface, chord-frame helpers."""
        jn = self.jn[f"{which}_R"]
        c = self.ctl[which]
        b2 = 0.5 * float(self.S["wing"]["span"])
        y0, y1 = float(c["eta0"]) * b2, float(c["eta1"]) * b2
        a = _unit(jn["axis"])
        o = np.asarray(jn["origin"], float)
        return {"origin": o, "axis": a, "y0": y0, "y1": y1, "e0": self.eta(y0), "e1": self.eta(y1),
                "lo": float(jn["lo"]), "hi": float(jn["hi"]), "joint": jn}

    def hinge_point_at(self, which: str, y: float) -> np.ndarray:
        """Point of the layout hinge line at plan station y."""
        h = self.hinge(which)
        o, a = h["origin"], h["axis"]
        return o + (y - o[1]) / a[1] * a

    def lip_s(self, eta: float, side: str) -> float:
        """Chord position of the skin's aft (cove-lip) end: the inner face of the T_SOLID lip lies on the cove circle
        (radius r + COVE_GAP + 0.1 mm about the 0.75-chord mid-thickness hinge point)."""
        sH, tH, r = self.hinge_cs(eta)
        R = r + COVE_GAP + 1e-4
        lo, hi = sH - R - 0.02, sH
        for _ in range(60):
            m = 0.5 * (lo + hi)
            sg = self.sig_of_s(eta, side, m)
            s, t, ns, nt = self.at_sig(eta, side, sg)
            pin = np.array([s + T_SOLID * ns, t + T_SOLID * nt])
            d = math.hypot(pin[0] - sH, pin[1] - tH)
            if d > R:
                lo = m
            else:
                hi = m
        return lo

    # ------------------------------------------------------------------ skin thickness
    def t_box(self, eta: float, side: str) -> float:
        if side == "lo":
            return self.t_skin_primary
        y = self.y_of(eta)
        y0 = self.y_box_up_end
        f = float(np.clip((y - y0) / 0.03, 0.0, 1.0))
        return (1 - f) * self.t_skin_box_up + f * self.t_skin_primary

    def knots(self, eta: float, side: str) -> dict:
        ch = self.chord(eta)
        sm = self.s_main(eta)
        sr = self.s_rear(eta)
        srf = sr - self.t_rweb() - W_RFL
        b1 = NOSE_SOLID * ch
        return {"b0": 0.0, "b1": b1, "b2": b1 + RAMP, "b3": sm - 0.5 * W_CAP - RAMP, "b4": sm - 0.5 * W_CAP,
                "sm": sm, "b5": sm + 0.5 * W_CAP, "b6": sm + 0.5 * W_CAP + RAMP, "b7": srf, "b8": srf + RAMP,
                "sr": sr, "lip": self.lip_s(eta, side), "srf": srf}

    def t_skin(self, eta: float, side: str, s) -> np.ndarray:
        k = self.knots(eta, side)
        tf, tb = self.t_skin_primary, self.t_box(eta, side)
        xs = [k["b0"], k["b1"], k["b2"], k["b3"], k["b4"], k["b5"], k["b6"], k["b7"], k["b8"], k["lip"] + 0.01]
        ts = [T_SOLID, T_SOLID, tf, tf, T_SOLID, T_SOLID, tb, tb, T_SOLID, T_SOLID]
        return np.interp(s, xs, ts)

    # ------------------------------------------------------------------ spar sizes
    def t_rweb(self) -> float:
        return int(self.sz_w["rear_web"]["plies"]) * self.t_pw

    def t_rflange(self) -> float:
        return self.t_rweb() + int(self.sz_w["rear_cap"]["plies"]) * self.t_ud

    def cap_plies(self, y: float) -> int:
        for a, b, n in self.sz_w["main_cap"]["zones"]:
            if a - 1e-9 <= y <= b + 1e-9:
                return int(n)
        return int(self.sz_w["main_cap"]["min_plies"])

    def t_cap(self, y: float) -> float:
        return self.cap_plies(max(y, self.y_j + 1e-6)) * self.t_ud

    def t_mweb(self, y: float) -> float:
        for a, b, n in self.sz_w["main_web"]["zones"]:
            if a - 1e-9 <= y <= b + 1e-9:
                return int(n) * self.t_pw
        return int(self.sz_w["main_web"]["min_plies"]) * self.t_pw

    # ------------------------------------------------------------------ stations (eta, projected y or None)
    def pt(self, st, s, t) -> np.ndarray:
        eta, py = st
        return self.to3d(eta, s, t, py)

    def vec(self, st, vs, vt) -> np.ndarray:
        eta, py = st
        V = self.vec3d(eta, vs, vt)
        if py is not None:
            V[:, 1] = 0.0
            V /= np.linalg.norm(V, axis=1, keepdims=True)
        return V

    def stp(self, y: float) -> tuple:
        """Projected station: the section at plan y forced into the plane y = const (root / tip planes)."""
        return (self.eta(y), float(y))

    @staticmethod
    def stn(eta: float) -> tuple:
        return (float(eta), None)

    def inner(self, st, side: str, s, extra: float = 0.0):
        """Points (3-D) and 2-D (s, t) of the skin inner face (+ ``extra`` further inward) at chord positions s."""
        eta = st[0]
        s = np.atleast_1d(np.asarray(s, float))
        sg = self.sig_of_s(eta, side, s)
        s0, t0, ns, nt = self.at_sig(eta, side, sg)
        th = self.t_skin(eta, side, s0) + extra
        s2, t2 = s0 + th * ns, t0 + th * nt
        return self.pt(st, s2, t2), np.column_stack([s2, t2]), np.column_stack([ns, nt])

    # ------------------------------------------------------------------ chord nodes
    def nodes(self, eta: float, side: str, seq) -> np.ndarray:
        """Chord positions s along ``side`` for a list of segments [(s_from, s_to, n)], arc-length uniform inside each
        segment (n nodes per segment, the segment end is the next segment's start; the last end is appended)."""
        out = []
        for a, b, n in seq:
            ga, gb = self.sig_of_s(eta, side, [a, b])
            g = np.linspace(ga, gb, int(n) + 1)[:-1]
            out.append(np.interp(g, self.curve(eta, side)["sig"], self.curve(eta, side)["s"]))
        out.append(np.array([seq[-1][1]]))
        return np.concatenate(out)


# =====================================================================================================================
# panel layout (spanwise stations of ribs, hinges, servo bays) - module detailing, from the layout interfaces
# =====================================================================================================================
RIB_Y = (0.775, 1.005, 1.33, 1.66, 1.95, 2.03, 2.26, 2.55, 2.88, 3.20)    # native rib planes (plan y of the LE)
HINGE_Y = {"flap": (0.826, 1.219, 1.599, 1.993), "aileron": (2.30, 2.70, 3.10)}
PIN_D = {"flap": 0.004, "aileron": 0.003}      # structures C-HPIN (4 mm; aileron 3 mm, see docs/detail/wing.md)
BRKT_BOLT = {"flap": 4, "aileron": 3}
LEVER_K = {"flap": 1.2, "aileron": 1.25}       # arm/horn lengths x k at the same 2:1 ratio (rod below the skin)


def skin_stations(op: OP) -> list:
    """Spanwise stations of every skin / spar / rib loft: projected root and tip planes, every rib plane with its
    flange edges, every OML section inside the panel, the box-skin core change, filled to <= 60 mm spacing."""
    e_lo, e_hi = op.eta(ROOT_RIB_Y + T_RIB + RIB_FL_W), op.eta(TIP_RIB_Y1 - T_RIB - RIB_FL_W)
    native = set()
    for y in RIB_Y:
        e = op.eta(y)
        for d in (-(0.5 * T_RIB + RIB_FL_W), 0.0, 0.5 * T_RIB + RIB_FL_W):
            native.add(round(e + d, 7))
    for e in op._e:
        if e_lo + 1e-4 < e < e_hi - 1e-4:
            native.add(round(float(e), 7))
    yb = op.y_box_up_end
    native.update({round(op.eta(yb), 7), round(op.eta(yb + 0.03), 7), round(op.eta(ROOT_TRANS_Y1), 7)})
    native = sorted(e for e in native if e_lo - 1e-9 <= e <= e_hi + 1e-9)
    if native[0] > e_lo + 1e-6:
        native = [e_lo] + native
    if native[-1] < e_hi - 1e-6:
        native.append(e_hi)
    fill = []
    for a, b in zip(native, native[1:]):
        n = int(math.ceil((b - a) / 0.06))
        fill += list(np.linspace(a, b, n + 1)[:-1])
    fill.append(native[-1])
    sts = [op.stp(ROOT_RIB_Y), op.stp(ROOT_RIB_Y + T_RIB), op.stp(ROOT_RIB_Y + T_RIB + RIB_FL_W)]
    sts += [op.stn(e) for e in fill]
    sts += [op.stp(TIP_RIB_Y1 - T_RIB - RIB_FL_W), op.stp(TIP_RIB_Y1 - T_RIB), op.stp(TIP_RIB_Y1)]
    # drop native stations that fall inside the projected root / tip bands
    out = []
    for st in sts:
        if st[1] is None and (op.y_of(st[0]) < ROOT_RIB_Y + T_RIB + RIB_FL_W + 1e-3 or
                              op.y_of(st[0]) > TIP_RIB_Y1 - T_RIB - RIB_FL_W - 1e-3):
            continue
        out.append(st)
    return out


ROOT_TRANS_Y1 = ROOT_RIB_Y + T_RIB + 0.100     # end of the 0.10 m cap build-up / depth transition (VS2-04)


def _slab(Pout: np.ndarray, Pin: np.ndarray) -> G.Mesh:
    """Closed solid between an outer and an inner point grid (nu, nv, 3) of the same topology."""
    D = Pin - Pout
    T = np.linalg.norm(D, axis=2)
    inward = D / np.maximum(T, 1e-12)[..., None]
    return G.shell_from_grid(Pout, T, inward=inward)


def skin_seq(op: OP, eta: float, side: str, part: str) -> list:
    k = op.knots(eta, side)
    if part == "le":
        seq = [(k["sm"], k["b4"], 3), (k["b4"], k["b3"], 5), (k["b3"], k["b2"], 16), (k["b2"], k["b1"], 5),
               (k["b1"], 0.0, 6)]
        return seq if side == "up" else [(b, a, n) for a, b, n in seq[::-1]]
    return [(k["sm"], k["b5"], 3), (k["b5"], k["b6"], 5), (k["b6"], k["b7"], 34), (k["b7"], k["b8"], 5),
            (k["b8"], k["sr"], 3), (k["sr"], k["lip"], 2)]


def skin_grids(op: OP, sts: list, part: str):
    """Outer / inner point grids of the LE skin ('le': upper cap centre -> nose -> lower cap centre) or of a box skin
    ('up' / 'lo': cap centre -> cove lip)."""
    Po, Pi = [], []
    for st in sts:
        eta = st[0]
        if part == "le":
            su = op.nodes(eta, "up", skin_seq(op, eta, "up", "le"))
            sl = op.nodes(eta, "lo", skin_seq(op, eta, "lo", "le"))[1:]
            ou = op.pt(st, su, op.t_oml(eta, "up", su))
            ol = op.pt(st, sl, op.t_oml(eta, "lo", sl))
            iu = op.inner(st, "up", su)[0]
            il = op.inner(st, "lo", sl)[0]
            Po.append(np.vstack([ou, ol]))
            Pi.append(np.vstack([iu, il]))
        else:
            s = op.nodes(eta, part, skin_seq(op, eta, part, "box"))
            Po.append(op.pt(st, s, op.t_oml(eta, part, s)))
            Pi.append(op.inner(st, part, s)[0])
    return np.stack(Po, axis=1), np.stack(Pi, axis=1)      # (n_chord, n_span, 3)


def _strip_ring(top: np.ndarray, bot: np.ndarray) -> np.ndarray:
    return np.vstack([top, bot[::-1]])


def _loft(rings) -> G.Mesh:
    return G.loft([np.asarray(r, float) for r in rings])


def _union(ms) -> G.Mesh:
    ms = [m for m in ms if m is not None]
    return ms[0] if len(ms) == 1 else G.union(ms)


def _diff(a: G.Mesh, cutters) -> G.Mesh:
    cutters = [c for c in cutters if c is not None]
    return a if not cutters else G.difference(a, cutters)


def _inter(a: G.Mesh, b: G.Mesh) -> G.Mesh:
    r = G.intersection(a, b)
    if r is None:
        raise ValueError("empty intersection")
    return r


def finish(m: G.Mesh) -> G.Mesh:
    """Boolean clean-up: merge sub-micron slivers so the mesh passes the self-intersection test (as chassis.finish)."""
    out = G.Mesh.from_manifold(m.to_manifold().simplify(1e-7))
    if out.check(self_intersect=True)["ok"]:
        return out
    out2 = G.Mesh.from_manifold(m.to_manifold().simplify(1e-6))
    return out2 if out2.check(self_intersect=True)["ok"] else out


def largest_piece(m: G.Mesh) -> G.Mesh:
    parts = m.to_manifold().decompose()
    if len(parts) <= 1:
        return m
    return G.Mesh.from_manifold(max(parts, key=lambda p: p.volume()))


def obox(center, axes, half) -> G.Mesh:
    """Oriented box: ``axes`` rows are unit local axes, ``half`` the half extents."""
    return G.box(2 * np.asarray(half, float), center, R=np.asarray(axes, float).T)


def prism(poly, origin, u, v, t0: float, t1: float) -> G.Mesh:
    """shapely polygon in the (u, v) plane at ``origin`` extruded along u x v from t0 to t1."""
    w = np.cross(_unit(u), _unit(v))
    return G.extrude(poly, t1 - t0, origin=np.asarray(origin, float) + t0 * w, u=u, v=v)


# =====================================================================================================================
# main spar (caps + web), root-bay transition, tongue
# =====================================================================================================================
N_CAP = 9


class Tongue:
    """Tongue frame (layout.chassis.wing_joint): straight beam along the insertion axis through the plan positions of
    the main pins, centred on z = 0 (the fork slot / KO-WINGJOINT-PATH centre); n = pin axis (aft), u = +z."""

    def __init__(self, op: OP):
        wj = op.wj
        self.d = _unit(-np.asarray(wj["insertion"]["axis_inboard"], float))
        self.n = np.array([self.d[1], -self.d[0], 0.0])
        self.u = np.array([0.0, 0.0, 1.0])
        tg = op.sz_j["tongue"]
        self.W = float(tg["width_m"])
        self.H = float(tg["height_m"])
        self.tf = float(tg["flange_t_m"])
        self.tf_min = float(tg["flange_taper"]["t_min_m"])
        self.tw = int(tg["web_plies"]) * op.t_pw
        self.boss_L = float(tg["boss_length_m"])
        self.pins = wj["main_spar"]["pins"]
        p2 = np.asarray(self.pins[1]["position"], float)
        self.p0 = np.array([p2[0], p2[1], 0.0])            # on the centre line (z = 0)
        bush = op.sz_j["bush"]
        self.bush_od = float(bush["od_m"])
        self.bush_id = float(bush["id_m"])
        self.y_tip = self.y_pin(0) - float(wj["main_spar"]["tongue"]["edge_distance_bush"]["tongue_tip_m"]) * self.d[1]

    def y_pin(self, i: int) -> float:
        return float(self.pins[i]["position"][1])

    def point(self, y: float, n: float = 0.0, u: float = 0.0) -> np.ndarray:
        """Point on the tongue centre line at plan y, offset by (n, u)."""
        s = (y - self.p0[1]) / self.d[1]
        return self.p0 + s * self.d + n * self.n + u * self.u

    def flange_t(self, y: float) -> float:
        y1, y2 = self.y_pin(0), self.y_pin(1)
        if y >= y2:
            return self.tf
        if y <= y1:
            return self.tf_min
        return self.tf_min + (self.tf - self.tf_min) * (y - y1) / (y2 - y1)

    def pin_center(self, i: int) -> np.ndarray:
        return np.asarray(self.pins[i]["position"], float)

    def section_at_y(self, y: float, n0, n1, u0, u1) -> np.ndarray:
        """Rectangle n0..n1 x u0..u1 of the tongue cross-section cut by the plane y = const (CCW in (x, z))."""
        out = []
        for n, u in ((n0, u0), (n1, u0), (n1, u1), (n0, u1)):
            q = self.p0 + n * self.n + u * self.u
            s = (y - q[1]) / self.d[1]
            out.append(q + s * self.d)
        return np.asarray(out)


def tongue_mesh(op: OP, tg: Tongue, y_out: float) -> G.Mesh:
    """Tongue from its tip (chamfered 3 x 30 deg) to the plane y = y_out: I-section (UD flanges tapered 10 -> 3 mm
    between the pins, 25-ply web) with solid boss blocks round the two pin bores, bores OD 22 for the bonded bushes."""
    W, H = tg.W, tg.H
    s_tip = (tg.y_tip - tg.p0[1]) / tg.d[1]
    s_out = (y_out - tg.p0[1]) / tg.d[1] + 0.012          # beyond the plane, trimmed below

    def rect(s, n0, n1, u0, u1):
        c = tg.p0 + s * tg.d
        return np.array([c + n0 * tg.n + u0 * tg.u, c + n1 * tg.n + u0 * tg.u, c + n1 * tg.n + u1 * tg.u,
                         c + n0 * tg.n + u1 * tg.u])
    env = _loft([rect(s_tip, -W / 2, W / 2, -H / 2, H / 2), rect(s_out, -W / 2, W / 2, -H / 2, H / 2)])
    # side channels between the flanges (the flange inner face follows the taper), interrupted by the boss blocks
    ss = sorted({s_tip - 0.001, s_out + 0.001} | {(tg.y_pin(i) - tg.p0[1]) / tg.d[1] + k * 0.5 * tg.boss_L
                                                   for i in (0, 1) for k in (-1, 1)})
    bosses = [((tg.y_pin(i) - tg.p0[1]) / tg.d[1] - 0.5 * tg.boss_L, (tg.y_pin(i) - tg.p0[1]) / tg.d[1] +
               0.5 * tg.boss_L) for i in (0, 1)]
    cuts = []
    for a, b in zip(ss, ss[1:]):
        mid = 0.5 * (a + b)
        if any(lo - 1e-9 <= mid <= hi + 1e-9 for lo, hi in bosses):
            continue
        rings_p, rings_m = [], []
        for s in np.linspace(a, b, 5):
            y = float((tg.p0 + s * tg.d)[1])
            tf = tg.flange_t(y)
            rings_p.append(rect(s, 0.5 * tg.tw, W / 2 + 0.002, -H / 2 + tf, H / 2 - tf))
            rings_m.append(rect(s, -W / 2 - 0.002, -0.5 * tg.tw, -H / 2 + tf, H / 2 - tf))
        cuts += [_loft(rings_p), _loft(rings_m)]
    # tip chamfers (3 mm x 30 deg on the four tip edges)
    c3, h3 = 0.003, 0.003 * math.tan(math.radians(30.0))
    ct = tg.p0 + s_tip * tg.d
    for e1, h1 in ((tg.u, H / 2), (-tg.u, H / 2), (tg.n, W / 2), (-tg.n, W / 2)):
        e2 = tg.n if abs(np.dot(e1, tg.u)) > 0.5 else tg.u
        w2 = W if abs(np.dot(e1, tg.u)) > 0.5 else H
        pts = []
        for k in (-1, 1):
            q = ct + k * (0.5 * w2 + 0.002) * e2
            pts += [q + (h1 + 0.002) * e1 - 0.002 * tg.d, q + (h1 - h3) * e1 - 0.002 * tg.d,
                    q + (h1 + 0.002) * e1 + (c3 + 0.0005) * tg.d]
        cuts.append(G.hull(np.asarray(pts)))
    for i in (0, 1):
        c = tg.pin_center(i)
        cuts.append(G.cylinder(0.5 * tg.bush_od + BOND * 0.5, c - 0.03 * tg.n, c + 0.03 * tg.n, n=48))
    m = _diff(env, cuts)
    # trim at the plane y = y_out
    big = G.box((1.0, 1.0, 1.0), (tg.p0[0], y_out - 0.5, 0.0))
    return _inter(m, big)


def cap_rings(op: OP, st, side: str, extra_w: float = 0.0):
    """Main-cap cross-section at station st: top (bonded to the skin) and bottom polylines (3-D)."""
    eta = st[0]
    sm = op.s_main(eta)
    s = np.linspace(sm - 0.5 * W_CAP - extra_w, sm + 0.5 * W_CAP + extra_w, N_CAP)
    P, st2, nn = op.inner(st, side, s, extra=BOND)
    tc = op.t_cap(op.y_of(eta) if st[1] is None else st[1])
    s2 = st2[:, 0] + tc * nn[:, 0]
    t2 = st2[:, 1] + tc * nn[:, 1]
    return P, op.pt(st, s2, t2), st2, np.column_stack([s2, t2])


def spar_outboard(op: OP, sts: list) -> dict:
    """Main-spar caps and web from the end of the root-bay transition to the tip rib (loft through the skin stations;
    caps 1 mm + bond under the OML, zone ply counts; web zone plies)."""
    caps = {"up": [], "lo": []}
    web = []
    for st in sts:
        y = op.y_of(st[0]) if st[1] is None else st[1]
        if y < ROOT_TRANS_Y1 - 1e-6 or y > TIP_RIB_Y1 - T_RIB - BOND + 1e-6:
            continue
        rr = {}
        for side in ("up", "lo"):
            top, bot, top2, bot2 = cap_rings(op, st, side)
            caps[side].append(_strip_ring(top, bot))
            rr[side] = bot2
        eta = st[0]
        sm = op.s_main(eta)
        tw = op.t_mweb(y)
        tu = float(np.interp(sm, rr["up"][:, 0], rr["up"][:, 1])) + OV
        tl = float(np.interp(sm, rr["lo"][:, 0], rr["lo"][:, 1])) - OV
        web.append(op.pt(st, [sm - tw / 2, sm + tw / 2, sm + tw / 2, sm - tw / 2], [tl, tl, tu, tu]))
    # last ring at the tip-rib inboard face (projected)
    st = op.stp(TIP_RIB_Y1 - T_RIB - BOND)
    rr = {}
    for side in ("up", "lo"):
        top, bot, top2, bot2 = cap_rings(op, st, side)
        caps[side].append(_strip_ring(top, bot))
        rr[side] = bot2
    sm = op.s_main(st[0])
    tw = op.t_mweb(st[1])
    tu = float(np.interp(sm, rr["up"][:, 0], rr["up"][:, 1])) + OV
    tl = float(np.interp(sm, rr["lo"][:, 0], rr["lo"][:, 1])) - OV
    web.append(op.pt(st, [sm - tw / 2, sm + tw / 2, sm + tw / 2, sm - tw / 2], [tl, tl, tu, tu]))
    return {"cap_up": _loft(caps["up"]), "cap_lo": _loft(caps["lo"]), "web": _loft(web)}


def spar_transition(op: OP, tg: Tongue) -> dict:
    """Root-bay depth transition (structures.sizing.wing_joint.transition): caps ramp linearly from the tongue flanges
    (at the root-rib outboard face) to their loft position at ROOT_TRANS_Y1, thickness 10 mm -> zone plies; web
    25 -> zone plies; tapered ROHACELL 71 WF filler between the skins and the ramped caps."""
    ya, yb = ROOT_RIB_Y + T_RIB - OV, ROOT_TRANS_Y1
    stb = op.stn(op.eta(yb))
    W2, H2, tf = tg.W / 2, tg.H / 2, tg.tf
    n_lin = np.linspace(-W2, W2, N_CAP)
    out = {}
    for side, sg in (("up", 1.0), ("lo", -1.0)):
        topb, botb, top2, bot2 = cap_rings(op, stb, side)
        ta, ba = [], []
        for n in n_lin[::-1] if False else n_lin:
            q_top = tg.section_at_y(ya, n, n, sg * H2, sg * H2)[0]
            q_bot = tg.section_at_y(ya, n, n, sg * (H2 - tf), sg * (H2 - tf))[0]
            ta.append(q_top)
            ba.append(q_bot)
        ta, ba = np.asarray(ta), np.asarray(ba)
        # orient the tongue strip like the loft strip (s increasing = x increasing)
        if (ta[-1, 0] - ta[0, 0]) * (topb[-1, 0] - topb[0, 0]) < 0:
            ta, ba = ta[::-1], ba[::-1]
        out[f"cap_{side}"] = _loft([_strip_ring(ta, ba), _strip_ring(topb, botb)])
        # filler: from the skin inner face (+bond) down into the cap top (0.2 mm overlap)
        sta = op.stp(ya)
        sm_a = float(np.dot(np.array([np.mean(ta[:, 0]) - op.frame(sta[0])[0][0], 0, 0]), [1, 0, 0]))
        sa = np.linspace(op.s_of_x(sta[0], ta[0, 0]), op.s_of_x(sta[0], ta[-1, 0]), N_CAP)
        Pa = op.inner(sta, side, sa, extra=BOND)[0]
        fa_bot = ta - sg * OV * np.array([0, 0, 1.0])
        Pb = topb
        fb_bot = topb + (botb - topb) * (0.0003 / max(np.linalg.norm(botb[0] - topb[0]), 1e-9))
        out[f"fill_{side}"] = _loft([_strip_ring(Pa, fa_bot), _strip_ring(Pb, fb_bot)])
        out[f"_bot_{side}"] = (ba, botb)
    # web: tongue web (25 plies) -> zone web
    tw_a, tw_b = tg.tw, op.t_mweb(yb)
    ba_u, bb_u = out["_bot_up"]
    ba_l, bb_l = out["_bot_lo"]
    sm_b = op.s_main(stb[0])
    zu_a = tg.section_at_y(ya, 0, 0, H2 - tf + OV, H2 - tf + OV)[0]
    zl_a = tg.section_at_y(ya, 0, 0, -(H2 - tf + OV), -(H2 - tf + OV))[0]
    ring_a = np.vstack([tg.section_at_y(ya, -tw_a / 2, tw_a / 2, -(H2 - tf) - OV, (H2 - tf) + OV)])
    tu = float(np.interp(sm_b, bot2[:, 0], bot2[:, 1]))     # last computed side is 'lo'; recompute both
    _t, _b, _t2, bu2 = cap_rings(op, stb, "up")
    _t, _b, _t2, bl2 = cap_rings(op, stb, "lo")
    tu = float(np.interp(sm_b, bu2[:, 0], bu2[:, 1])) + OV
    tl = float(np.interp(sm_b, bl2[:, 0], bl2[:, 1])) - OV
    ring_b = op.pt(stb, [sm_b - tw_b / 2, sm_b + tw_b / 2, sm_b + tw_b / 2, sm_b - tw_b / 2], [tl, tl, tu, tu])
    out["web"] = _loft([ring_a, ring_b])
    for k in [k for k in out if k.startswith("_")]:
        out.pop(k)
    return out
