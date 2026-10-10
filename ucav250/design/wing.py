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
RIB_Y = (0.83, 1.15, 1.40, 1.70, 1.99, 2.26, 2.55, 2.88, 3.20)    # native rib planes (plan y of the LE)
HINGE_Y = {"flap": (0.78, 1.25, 1.55, 1.88), "aileron": (2.31, 2.70, 3.10)}
PIN_D = {"flap": 0.004, "aileron": 0.003}      # structures C-HPIN (4 mm; aileron 3 mm, see docs/detail/wing.md)
BRKT_BOLT = {"flap": 3, "aileron": 3}
LEVER_K = {"flap": 1.2, "aileron": 1.4}       # arm/horn lengths x k at the same 2:1 ratio (rod below the skin)


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
    stb = op.stn(op.eta(yb) + 0.001)                        # 1 mm into the outboard loft (fused union)
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
        sa = np.linspace(op.s_of_x(sta[0], ta[0, 0]), op.s_of_x(sta[0], ta[-1, 0]), N_CAP)
        Pa = op.inner(sta, side, sa, extra=BOND)[0]
        Pa[:, 1] = ROOT_RIB_Y + T_RIB + BOND                 # filler bonded to the root-rib outboard face
        fa_bot = ta - sg * OV * np.array([0, 0, 1.0])
        fa_bot[:, 1] = ROOT_RIB_Y + T_RIB + BOND
        Pb = topb
        fb_bot = topb + (botb - topb) * (0.0003 / max(np.linalg.norm(botb[0] - topb[0]), 1e-9))
        out[f"fill_{side}"] = _loft([_strip_ring(Pa, fa_bot), _strip_ring(Pb, fb_bot)])
        out[f"_bot_{side}"] = (ba, botb)
    # web: tongue web (25 plies) -> zone web
    tw_a, tw_b = tg.tw, op.t_mweb(yb)
    sm_b = op.s_main(stb[0])
    ring_a = np.vstack([tg.section_at_y(ya, -tw_a / 2, tw_a / 2, -(H2 - tf) - OV, (H2 - tf) + OV)])
    _t, _b, _t2, bu2 = cap_rings(op, stb, "up")
    _t, _b, _t2, bl2 = cap_rings(op, stb, "lo")
    tu = float(np.interp(sm_b, bu2[:, 0], bu2[:, 1])) + OV
    tl = float(np.interp(sm_b, bl2[:, 0], bl2[:, 1])) - OV
    ring_b = op.pt(stb, [sm_b - tw_b / 2, sm_b + tw_b / 2, sm_b + tw_b / 2, sm_b - tw_b / 2], [tl, tl, tu, tu])
    out["web"] = _loft([ring_a, ring_b])
    for k in [k for k in out if k.startswith("_")]:
        out.pop(k)
    return out


# =====================================================================================================================
# rear spar (C-section, flanges forward)
# =====================================================================================================================
N_FL = 6
RS_Y0 = ROOT_RIB_Y + T_RIB + BOND + 0.0038 + BOND  # rear-spar root: outboard of the root fitting's rib flange (FIT_T)


def rear_poly2(op: OP, st) -> np.ndarray:
    """(s, t) C-polygon of the rear spar at station st: upper flange top (fwd -> aft), web aft face, lower flange
    bottom (aft -> fwd), lower flange inner face, web forward face, upper flange inner face."""
    eta = st[0]
    sr = op.s_rear(eta)
    tw, tf = op.t_rweb(), op.t_rflange()
    srf = sr - tw - W_RFL
    s = np.linspace(srf, sr, N_FL)
    _P, u2, nu = op.inner(st, "up", s, extra=BOND)
    _P, l2, nl = op.inner(st, "lo", s, extra=BOND)
    ub = u2 + tf * nu
    lb = l2 + tf * nl
    tu_i = float(np.interp(sr - tw, ub[:, 0], ub[:, 1]))
    tl_i = float(np.interp(sr - tw, lb[:, 0], lb[:, 1]))
    ub[-1] = (sr - tw, tu_i)
    lb[-1] = (sr - tw, tl_i)
    return np.vstack([u2, l2[::-1], lb, ub[::-1]])


def rear_ring(op: OP, st) -> np.ndarray:
    P2 = rear_poly2(op, st)
    return op.pt(st, P2[:, 0], P2[:, 1])


def rear_spar_mesh(op: OP, sts: list) -> G.Mesh:
    rings = [rear_ring(op, op.stp(RS_Y0))]
    for st in sts:
        y = op.y_of(st[0]) if st[1] is None else st[1]
        if RS_Y0 + 0.002 < y < TIP_RIB_Y1 - T_RIB - BOND - 0.002 and st[1] is None:
            rings.append(rear_ring(op, st))
    rings.append(rear_ring(op, op.stp(TIP_RIB_Y1 - T_RIB - BOND)))
    return _loft(rings)


# =====================================================================================================================
# ribs
# =====================================================================================================================
def knot_seq(k: dict) -> list:
    """Chord segments LE -> cove lip with every thickness knot as a node (polygons must not cut ramp corners)."""
    pts = [0.0, k["b1"], k["b2"], k["b3"], k["b4"], k["sm"], k["b5"], k["b6"], k["b7"], k["b8"], k["sr"], k["lip"]]
    ns = [6, 5, 14, 5, 3, 3, 5, 30, 5, 3, 2]
    return [(a, b, n) for a, b, n in zip(pts, pts[1:], ns)]


def interior_poly(op: OP, st, extra: float = BOND, to_te: bool = False) -> Polygon:
    """(s, t) polygon of the section interior inside the skins (+ ``extra``), LE to the cove lips; with ``to_te`` the
    region aft of the lips (no skin there) is bounded by the OML up to the trailing edge."""
    eta = st[0]
    su = op.nodes(eta, "up", knot_seq(op.knots(eta, "up")))
    sl = op.nodes(eta, "lo", knot_seq(op.knots(eta, "lo")))
    _P, iu, _n = op.inner(st, "up", su, extra=extra)
    _P, il, _n = op.inner(st, "lo", sl, extra=extra)
    if not to_te:
        return SG.largest(Polygon(np.vstack([iu[::-1], il[1:]])).buffer(0))
    cu, cl = op.curve(eta, "up"), op.curve(eta, "lo")
    mu = cu["s"] > iu[-1, 0] + 1e-4
    ml = cl["s"] > il[-1, 0] + 1e-4
    ring = np.vstack([iu[::-1], il[1:], np.column_stack([cl["s"][ml], cl["t"][ml]]),
                      np.column_stack([cu["s"][mu], cu["t"][mu]])[::-1]])
    return SG.largest(Polygon(ring).buffer(0))


def cap_polys(op: OP, st, grow: float = BOND) -> list:
    out = []
    for side in ("up", "lo"):
        _t, _b, top2, bot2 = cap_rings(op, st, side)
        out.append(Polygon(np.vstack([top2, bot2[::-1]])).buffer(grow, join_style=2))
    return out


def rflange_polys(op: OP, st, grow: float = BOND) -> list:
    return [Polygon(rear_poly2(op, st)).buffer(0).buffer(grow, join_style=2)]


R_CONDUIT = 0.007           # harness conduit OD 14 mm (GFRP tube, 1 mm wall)


CONDUIT_Y0 = ROOT_RIB_Y + T_RIB + 0.040     # conduit inboard end (40 mm for the CN-WING plug and backshell)
CONDUIT_RISE_Y = 0.89                        # conduit reaches its upper route (over the flap servo) here
CONDUIT_MERGE = (1.30, 1.70)                 # blend from the upper route onto the main-spar web line
CONDUIT_UP = 0.014                           # upper route: centre this far under the upper skin inner face
CONDUIT_Y1 = TIP_RIB_Y1 - T_RIB - 0.020      # conduit outboard end


def _smooth(x: float) -> float:
    x = min(1.0, max(0.0, x))
    return x * x * (3.0 - 2.0 * x)


def _web_st(op: OP, eta: float, y: float) -> tuple[float, float]:
    sm = op.s_main(eta)
    s = sm + 0.5 * op.t_mweb(max(y, ROOT_TRANS_Y1)) + BOND + R_CONDUIT
    tu = float(op.t_oml(eta, "up", s))
    tl = float(op.t_oml(eta, "lo", s))
    return s, 0.5 * (tu + tl)


def conduit_st(op: OP, st) -> tuple[float, float]:
    """Harness conduit centre (s, t) at a station. Outboard of y 1.70 it is bonded along the aft face of the main-spar
    web at mid-depth; inboard it leaves the CN-WING plug (layout.systems.harness, x 2.725 / z -0.01) spanwise, rises
    over the flap servo to 14 mm under the upper skin inner face and blends onto the web line between y 1.30 and
    1.70 (smooth-step blends, no kinks)."""
    eta = st[0]
    y = op.y_of(eta) if st[1] is None else st[1]
    s_w, t_w = _web_st(op, eta, y)
    if y >= CONDUIT_MERGE[1]:
        return s_w, t_w
    cn = next(c for c in op.L["systems"]["harness"]["connectors"] if c["id"] == "CN-WING")["point"]
    ym = CONDUIT_MERGE[1]
    em = op.eta(ym)
    x_m = op.pt(op.stn(em), [_web_st(op, em, ym)[0]], [0.0])[0][0]
    x_u = float(cn[0]) + (x_m - float(cn[0])) * (y - CONDUIT_Y0) / (ym - CONDUIT_Y0)
    s_u = op.s_of_x(eta, x_u)
    t_u = float(op.inner(st, "up", [s_u])[1][0][1]) - CONDUIT_UP
    o, c, u, _w = op.frame(eta)
    t_cn = (float(cn[2]) - o[2] - s_u * c[2]) / u[2]
    f = _smooth((y - CONDUIT_Y0) / (CONDUIT_RISE_Y - CONDUIT_Y0))
    s_r, t_r = s_u, (1 - f) * t_cn + f * t_u
    g = _smooth((y - CONDUIT_MERGE[0]) / (CONDUIT_MERGE[1] - CONDUIT_MERGE[0]))
    return (1 - g) * s_r + g * s_w, (1 - g) * t_r + g * t_w


def conduit_mesh(op: OP) -> G.Mesh:
    """GFRP harness conduit OD 14 x 1 mm (wing branch of H-WING) from the CN-WING plug backshell to the tip bay."""
    ys = np.r_[np.arange(CONDUIT_Y0, CONDUIT_Y1, 0.010), CONDUIT_Y1]
    path = []
    for y in ys:
        st = op.stn(op.eta(y))
        sc, tc = conduit_st(op, st)
        path.append(op.pt(st, [sc], [tc])[0])
    path = np.asarray(path)
    outer = G.sweep_circle(path, R_CONDUIT, n=20)
    d0 = _unit(path[1] - path[0])
    d1 = _unit(path[-1] - path[-2])
    ext = np.vstack([path[0] - 0.002 * d0, path, path[-1] + 0.002 * d1])
    inner = G.sweep_circle(ext, R_CONDUIT - 0.001, n=20)
    return finish(_diff(outer, [inner]))


def strip_nodes(op: OP, st, side: str, s0: float, s1: float, n: int = 6) -> np.ndarray:
    """Chord nodes s0..s1 with every thickness knot inside the range as a node (same count at every station when the
    range is defined relative to the knots)."""
    k = op.knots(st[0], side)
    inside = [v for kk, v in k.items() if kk in ("b1", "b2", "b3", "b4", "b5", "b6", "b7", "b8", "sr")
              and s0 + 1e-6 < v < s1 - 1e-6]
    pts = [s0] + sorted(inside) + [s1]
    return op.nodes(st[0], side, [(a, b, n) for a, b in zip(pts, pts[1:])])


def flange_range(op: OP, st, kind: str) -> tuple[float, float]:
    """Chord range of a rib T-flange along the skins at station st (clear of the spar caps / rear flange)."""
    eta = st[0]
    sm = op.s_main(eta)
    srf = op.s_rear(eta) - op.t_rweb() - W_RFL
    if kind == "nose":
        return op.knots(eta, "up")["b2"] + 0.005, sm - 0.5 * W_CAP - 2 * BOND
    return sm + 0.5 * W_CAP + 2 * BOND, srf - 2 * BOND


def strip_ring(op: OP, st, side: str, s0, s1=None, t: float = RIB_FL_T) -> np.ndarray:
    """Flange strip along the skin inner face at st; ``s0`` is either a chord value (with s1) or a rib kind."""
    if isinstance(s0, str):
        s0, s1 = flange_range(op, st, s0)
    s = strip_nodes(op, st, side, s0, s1)
    P = op.inner(st, side, s, extra=BOND)[0]
    Q = op.inner(st, side, s, extra=BOND + t)[0]
    return np.vstack([P, Q[::-1]])


def map2(op: OP, st_from, st_to, P2) -> np.ndarray:
    """(s, t) points of station st_from expressed in the section frame of st_to (via 3-D)."""
    P2 = np.asarray(P2, float)
    P = op.pt(st_from, P2[:, 0], P2[:, 1])
    o, c, u, _w = op.frame(st_to[0])
    if st_to[1] is not None:
        o = np.array([o[0], st_to[1], o[2]])
    d = P - o
    return np.column_stack([d @ c, d @ u])


def map_poly(op: OP, st_from, st_to, poly):
    def one(p):
        ext = map2(op, st_from, st_to, np.asarray(p.exterior.coords))
        holes = [map2(op, st_from, st_to, np.asarray(h.coords)) for h in p.interiors]
        return Polygon(ext, holes)
    ps = list(poly.geoms) if isinstance(poly, MultiPolygon) else [poly]
    return unary_union([one(p) for p in ps]).buffer(0)


def plate_mesh(op: OP, st, poly, t: float = T_RIB) -> G.Mesh:
    """Planar plate centred on the station plane (native: the section plane; projected: the plane y = st[1])."""
    eta, py = st
    o, c, u, _w = op.frame(eta)
    if py is None:
        return G.extrude(poly, t, origin=o, u=c, v=u, centered=True)
    # projected: map the (s, t) outline into (x, z) and extrude along y
    def mp(p):
        P = np.asarray(p.exterior.coords)
        Q = op.pt(st, P[:, 0], P[:, 1])
        holes = []
        for h in p.interiors:
            H = np.asarray(h.coords)
            HQ = op.pt(st, H[:, 0], H[:, 1])
            holes.append(HQ[:, [0, 2]])
        return Polygon(Q[:, [0, 2]], holes).buffer(0)
    polys = [mp(p) for p in (poly.geoms if isinstance(poly, MultiPolygon) else [poly])]
    xz = unary_union(polys)
    return prism(xz, (0.0, py - 0.5 * t, 0.0), (1.0, 0.0, 0.0), (0.0, 0.0, 1.0), 0.0, t).transformed(
        np.diag([1.0, -1.0, 1.0]), (0.0, 2 * py, 0.0))


def rib_region(op: OP, st, kind: str) -> Polygon:
    """Rib piece region at one station (its own frame): inside the skins, between the spar webs, minus caps / flanges."""
    eta = st[0]
    y = op.y_of(eta) if st[1] is None else st[1]
    sm = op.s_main(eta)
    tw = op.t_mweb(max(y, ROOT_TRANS_Y1))
    sr = op.s_rear(eta)
    poly = interior_poly(op, st)
    if kind == "nose":
        s0, s1 = -1.0, sm - 0.5 * tw - BOND
    else:
        s0, s1 = sm + 0.5 * tw + BOND, sr - op.t_rweb() - BOND
    reg = poly.intersection(sbox(s0, -1.0, s1, 1.0))
    for c in cap_polys(op, st) + (rflange_polys(op, st) if kind == "box" else []):
        reg = reg.difference(c)
    return SG.largest(reg)


def rib_piece(op: OP, eta: float, kind: str, holes=True) -> G.Mesh:
    """Flanged sandwich rib (rib_panel) piece at a native station: 'nose' (LE skin -> main-spar web) or 'box'
    (main-spar web -> rear-spar web), notched round the spar caps / flanges, T-flanges 15 mm each side bonded to the
    skins, lightening holes, conduit hole (box). The plate outline is the intersection of the free regions at its two
    faces and its mid-plane."""
    st = op.stn(eta)
    sm = op.s_main(eta)
    sr = op.s_rear(eta)
    srf = sr - op.t_rweb() - W_RFL
    reg = rib_region(op, st, kind)
    for dl in (-0.5 * T_RIB - OV, 0.5 * T_RIB + OV):
        so = op.stn(eta + dl)
        reg = reg.intersection(map_poly(op, so, st, rib_region(op, so, kind)))
    reg = SG.largest(reg)
    if holes:
        cut = []
        s0, s1 = reg.bounds[0], reg.bounds[2]
        if kind == "box":
            for dl in (-0.5 * T_RIB - OV, 0.0, 0.5 * T_RIB + OV):
                so = op.stn(eta + dl)
                sc, tc = conduit_st(op, so)
                q = map2(op, so, st, [[sc, tc]])[0]
                cut.append(Point(q[0], q[1]).buffer(R_CONDUIT + 0.0015, 32))
            c_hole = unary_union(cut)
            a, b = s0 + 0.035, s1 - 0.02
            n_h = max(1, int((b - a) // 0.08))
            for k in range(n_h):
                sx = a + (k + 0.5) * (b - a) / n_h
                tu, tl = float(op.t_oml(eta, "up", sx)), float(op.t_oml(eta, "lo", sx))
                d = min(0.42 * (tu - tl - 0.016), 0.6 * (b - a) / n_h)
                hole = Point(sx, 0.5 * (tu + tl)).buffer(0.5 * d, 40)
                if d > 0.012 and hole.distance(c_hole) > 0.008:
                    cut.append(hole)
        else:
            sx = 0.55 * s1
            tu, tl = float(op.t_oml(eta, "up", sx)), float(op.t_oml(eta, "lo", sx))
            d = min(0.40 * (tu - tl - 0.014), 0.5 * s1)
            if d > 0.012:
                cut.append(Point(sx, 0.5 * (tu + tl)).buffer(0.5 * d, 40))
        for cc in cut:
            reg = reg.difference(cc)
        reg = SG.largest(reg)
    plate = plate_mesh(op, st, reg)
    fls = []
    for side in ("up", "lo"):
        for e0, e1 in ((eta - 0.5 * T_RIB - RIB_FL_W, eta - 0.5 * T_RIB + OV),
                       (eta + 0.5 * T_RIB - OV, eta + 0.5 * T_RIB + RIB_FL_W)):
            fls.append(_loft([strip_ring(op, op.stn(e0), side, kind), strip_ring(op, op.stn(e1), side, kind)]))
    return finish(_union([plate] + fls))


# =====================================================================================================================
# root rib, tip rib, joint hardware
# =====================================================================================================================
FIT_T = 0.0038              # rear root fitting: rib flange thickness (M5 into 6.8 mm through-inserts: 6 mm engagement)
FIT_X = (2.776, 2.897)      # rear root fitting: chordwise extent of the rib flange (box bay -> lug)
FIT_WEB_Y1 = 0.765          # rear root fitting: web flange end (bonded to the rear-spar web root)
LUG_SLOT = 0.001            # clearance round the lug plate in the root rib


def end_rib_region(op: OP, st_a, st_b, kind: str) -> Polygon:
    """(x, z) region of a projected end rib between the planes y = st_a[1] .. st_b[1]: inside the skins up to the
    cove lips, the OML aft of them (the rib is the panel's root / tip closure)."""
    regs = []
    for st in (st_a, st_b):
        P2 = np.asarray(interior_poly(op, st, to_te=True).exterior.coords)
        Q = op.pt(st, P2[:, 0], P2[:, 1])
        regs.append(Polygon(Q[:, [0, 2]]).buffer(0))
    reg = regs[0].intersection(regs[1]).buffer(-2e-5, join_style=2).buffer(2e-5, join_style=2)
    return SG.largest(reg.simplify(2e-6))


def end_rib_flange(op: OP, y0: float, y1: float, side: str, excl) -> list:
    """Flange strip lofts of a projected end rib along the skin (LE 3 % chord -> lip), split round excluded chord
    ranges ``excl`` [(s0, s1)] given as functions of the station."""
    out = []
    sts = [op.stp(y0), op.stp(y1)]
    rngs = []
    for st in sts:
        k = op.knots(st[0], side)
        cuts = sorted(excl(st))
        a = k["b2"] + 0.005
        pieces = []
        for c0, c1 in cuts:
            if c0 > a + 0.004:
                pieces.append((a, c0))
            a = max(a, c1)
        if k["lip"] - 0.002 > a + 0.004:
            pieces.append((a, k["lip"] - 0.002))
        rngs.append(pieces)
    for (a0, a1), (b0, b1) in zip(rngs[0], rngs[1]):
        out.append(_loft([strip_ring(op, sts[0], side, a0, a1), strip_ring(op, sts[1], side, b0, b1)]))
    return out


def root_rib_mesh(op: OP, tg: Tongue, lug_poly_xz: Polygon) -> G.Mesh:
    """Root rib (YK250-WG-161): flat rib_panel plate in the plane y 0.7049 .. 0.7117 (inboard face on the 1.5 mm seal
    gap to the centre-section joint rib), full section to the trailing edge, 20-ply solid land round the tongue,
    openings for the tongue, the rear-spar lug and the blind-mate wing connector plug (CN-WING); T-flanges outboard."""
    ya, yb = ROOT_RIB_Y, ROOT_RIB_Y + T_RIB
    reg = end_rib_region(op, op.stp(ya), op.stp(yb), "root")
    # tongue passage: union of the tongue sections at both faces + bond
    W2, H2 = 0.5 * tg.W, 0.5 * tg.H
    t_sec = unary_union([Polygon(tg.section_at_y(y, -W2, W2, -H2, H2)[:, [0, 2]]) for y in (ya - 0.001, yb + 0.001)])
    reg = reg.difference(t_sec.convex_hull.buffer(BOND, join_style=2))
    reg = reg.difference(lug_poly_xz.buffer(LUG_SLOT, join_style=2))
    cn = next(c for c in op.L["systems"]["harness"]["connectors"] if c["id"] == "CN-WING")["point"]
    reg = reg.difference(Point(cn[0], cn[2]).buffer(0.0155, 48))
    reg = SG.largest(reg)
    plate = prism(reg, (0.0, ya, 0.0), (1.0, 0.0, 0.0), (0.0, 0.0, 1.0), -T_RIB, 0.0)   # extrudes along -y
    plate = plate.translated((0.0, T_RIB, 0.0)) if plate.bounds()[0][1] < ya - 1e-6 else plate

    def excl(st):
        eta = st[0]
        sm = op.s_main(eta)
        srf = op.s_rear(eta) - op.t_rweb() - W_RFL
        return [(sm - 0.5 * W_CAP - 0.004, sm + 0.5 * W_CAP + 0.004), (srf - 0.004, 10.0)]
    fls = []
    for side in ("up", "lo"):
        fls += end_rib_flange(op, yb - OV, yb + RIB_FL_W, side, excl)
    return finish(_union([plate] + fls))


def tip_rib_mesh(op: OP) -> G.Mesh:
    """Tip rib (YK250-WG-178): flat rib_panel plate y 3.5331 .. 3.5399, full section; its outboard face carries the
    tip fairing and the LT-WING light (layout.systems.air_data_lights); T-flanges inboard."""
    ya, yb = TIP_RIB_Y1 - T_RIB, TIP_RIB_Y1
    reg = end_rib_region(op, op.stp(ya), op.stp(yb), "tip")
    plate = prism(reg, (0.0, ya, 0.0), (1.0, 0.0, 0.0), (0.0, 0.0, 1.0), -T_RIB, 0.0)
    if plate.bounds()[0][1] < ya - 1e-6:
        plate = plate.translated((0.0, T_RIB, 0.0))

    def excl(st):
        eta = st[0]
        sm = op.s_main(eta)
        srf = op.s_rear(eta) - op.t_rweb() - W_RFL
        return [(sm - 0.5 * W_CAP - 0.004, sm + 0.5 * W_CAP + 0.004), (srf - 0.004, 10.0)]
    fls = []
    for side in ("up", "lo"):
        fls += end_rib_flange(op, ya - RIB_FL_W, ya + OV, side, excl)
    return finish(_union([plate] + fls))


def tongue_bushes(op: OP, tg: Tongue) -> G.Mesh:
    """Bonded 4130 bush pair of the tongue (structures.sizing.wing_joint.bush: 16 H8 x OD 22 x 30)."""
    ms = []
    for i in (0, 1):
        c = tg.pin_center(i)
        ri = 0.5 * tg.bush_id + 0.0000135
        ms.append(G.tube(0.5 * tg.bush_od, ri, c - 0.5 * tg.W * tg.n, c + 0.5 * tg.W * tg.n, n=48))
    return G.union(ms)


PIN_HEAD = (0.024, 0.006)    # main pin head d x h (layout pins)
FORK_PAD_N = 0.0152 + 0.010  # fork pad outer face from the slot centre line (layout slot width / 2 + 10 mm pad)


def main_pin(op: OP, tg: Tongue, i: int) -> G.Mesh:
    """Headed Ti-6Al-4V main pin d 16 h8 x 62 (layout pins P-MAIN1/2): head on the fork front pad, shank through the
    four fork bushes and the two tongue bushes."""
    p = tg.pins[i]
    c = np.asarray(p["position"], float)
    a = _unit(p["axis"])
    d = float(p["diameter"])
    L = float(p["length"])
    r = 0.5 * d - 0.0000135
    n0 = -FORK_PAD_N - BOND
    shank = G.cylinder(r, c + (n0 - OV) * a, c + (n0 + L) * a, n=40)
    hd, hh = PIN_HEAD
    head = G.cylinder(0.5 * hd, c + (n0 - hh) * a, c + n0 * a, n=40)
    return G.union([shank, head])


KEEPER = {"t": 0.0025, "half_d": 0.0105, "boss_r": 0.0085, "bolt_u": 0.022, "half_u": 0.031}


def keeper_mesh(op: OP, tg: Tongue, i: int) -> tuple[G.Mesh, list]:
    """7075 keeper plate over a main-pin head: plate 2.5 mm, two bosses down to the fork front pad (2 x M4 into
    potted inserts of the pad, layout pins.spec). Returns (mesh, [(bolt head point, axis)])."""
    p = tg.pins[i]
    c = np.asarray(p["position"], float)
    a = _unit(p["axis"])
    hd, hh = PIN_HEAD
    n_head = -FORK_PAD_N - BOND - hh                      # front face of the pin head
    n_back = n_head - BOND
    n_front = n_back - KEEPER["t"]
    n_pad = -FORK_PAD_N - BOND
    dd = tg.d
    u = tg.u
    plate = obox(c + 0.5 * (n_back + n_front) * a, [a, dd, u], [0.5 * KEEPER["t"], KEEPER["half_d"], KEEPER["half_u"]])
    ms = [plate]
    bolts = []
    for sg in (-1, 1):
        q = c + sg * KEEPER["bolt_u"] * u
        ms.append(G.cylinder(KEEPER["boss_r"], q + (n_back + OV) * a, q + n_pad * a, n=40))
        bolts.append((q + n_front * a, a))
    return G.union(ms), bolts


def lug_plan(op: OP) -> Polygon:
    """Plan outline (x, y) of the rear-spar lug plate: bore 8 H8 on P-REAR, e 16 mm semicircular tip, 32 mm wide along
    the insertion axis (layout rear_spar.lug), forward edge trimmed parallel to the centre-section rear spar
    (M-CTBOX rear_spar_line + 0.3 web + 3.2 pad + 4 slot-fitting web plate + 1 mm) inside the centre section."""
    rp = op.wj["rear_spar"]
    lug = rp["lug"]
    pr = np.asarray(rp["pin"]["position"], float)
    d = _unit(-np.asarray(op.wj["insertion"]["axis_inboard"], float))[:2]
    n = np.array([d[1], -d[0]])
    w2 = 0.5 * float(lug["width"])
    y_end = ROOT_RIB_Y + T_RIB + BOND + FIT_T - 0.001
    s_end = (y_end - pr[1]) / d[1]
    tip = Point(pr[0], pr[1]).buffer(w2, 64)
    body = Polygon([pr[:2] - w2 * n, pr[:2] + w2 * n, pr[:2] + w2 * n + s_end * d, pr[:2] - w2 * n + s_end * d])
    reg = unary_union([tip, body])
    rs = np.asarray({m["id"]: m for m in op.L["chassis"]["members"]}["M-CTBOX"]["rear_spar_line"], float)
    ys = np.linspace(0.6, op.y_j - 0.0034, 20)
    xt = np.interp(ys, rs[:, 1], rs[:, 0]) + 0.0003 + 0.0032 + 0.004 + 0.001
    trim = Polygon(np.vstack([np.column_stack([xt, ys]), [[xt[-1], op.y_j - 0.0034], [2.5, op.y_j - 0.0034],
                                                           [2.5, 0.6]]]))
    reg = reg.difference(trim)
    return SG.largest(reg.difference(Point(pr[0], pr[1]).buffer(0.5 * float(lug["bore"]) + 0.0000135, 48)))


def fitting_mesh(op: OP) -> tuple[G.Mesh, list]:
    """Rear-spar root fitting with the lug (YK250-WG-152, machined 7075-T651): lug plate 8 mm on the rear-pin plane,
    rib flange FIT_T on the root-rib outboard face (4 x M5 into through-thickness inserts of the root rib), web
    flange 4 mm bonded to the forward face of the rear-spar web root. Returns (mesh, [(bolt head point, axis)])."""
    rp = op.wj["rear_spar"]
    pz = float(rp["pin"]["position"][2])
    t2 = 0.5 * float(rp["lug"]["thickness"])
    lug = prism(lug_plan(op), (0.0, 0.0, pz - t2), (1.0, 0.0, 0.0), (0.0, 1.0, 0.0), 0.0, 2 * t2)
    y0, y1 = ROOT_RIB_Y + T_RIB + BOND, ROOT_RIB_Y + T_RIB + BOND + FIT_T
    stm = op.stp(0.5 * (y0 + y1))
    P2 = np.asarray(interior_poly(op, stm).exterior.coords)
    Q = op.pt(stm, P2[:, 0], P2[:, 1])
    inner = Polygon(Q[:, [0, 2]]).buffer(0).buffer(-(RIB_FL_T + 0.0012), join_style=2)
    fl_xz = SG.largest(inner.intersection(sbox(FIT_X[0], -1.0, FIT_X[1], 1.0)))
    flange = prism(fl_xz, (0.0, y0, 0.0), (1.0, 0.0, 0.0), (0.0, 0.0, 1.0), -FIT_T, 0.0)
    if flange.bounds()[0][1] < y0 - 1e-6:
        flange = flange.translated((0.0, FIT_T, 0.0))
    # web flange inside the C-section, on the forward face of the rear-spar web
    rings = []
    sts = [op.stp(y0)] + [op.stn(e) for e in np.linspace(op.eta(y0 + 0.004), op.eta(FIT_WEB_Y1), 6)]
    for st in sts:
        C = rear_poly2(op, st)
        sr = op.s_rear(st[0])
        tw = op.t_rweb()
        ub = C[3 * N_FL:][::-1]       # upper flange inner face (fwd -> web)
        lb = C[2 * N_FL:3 * N_FL]     # lower flange inner face (fwd -> web)
        s0, s1 = sr - tw - 0.004, sr - tw - 2 * BOND
        tu = float(np.interp(s0, ub[:, 0], ub[:, 1])) - 0.0008
        tl = float(np.interp(s0, lb[:, 0], lb[:, 1])) + 0.0008
        rings.append(op.pt(st, [s0, s1, s1, s0], [tl, tl, tu, tu]))
    web = _loft(rings)
    m = finish(_union([lug, flange, web]))
    zs = 0.5 * (fl_xz.bounds[1] + fl_xz.bounds[3])
    bolts = []
    for x in (FIT_X[0] + 0.012, FIT_X[0] + 0.044):
        for dz in (-0.009, 0.009):
            bolts.append((np.array([x, y1, zs + dz]), np.array([0.0, -1.0, 0.0])))
    return m, bolts


def rear_pin_mesh(op: OP) -> G.Mesh:
    """Ti-6Al-4V ball-lock rear pin d 8 f7 (layout P-REAR), inserted upward through P-REARACCESS: push-button head
    d 14 x 4 under the lower slot plate, shank through the slot plates and the lug, 2.5 mm ball-lock end above."""
    rp = op.wj["rear_spar"]
    sf = rp["slot_fitting"]
    c = np.asarray(rp["pin"]["position"], float)
    z_lo = c[2] - 0.5 * float(sf["slot"]) - float(sf["plate_t"]) - BOND
    z_hi = c[2] + 0.5 * float(sf["slot"]) + float(sf["plate_t"]) + 0.0025
    r = 0.5 * float(rp["pin"]["diameter"]) - 0.00002
    shank = G.cylinder(r, (c[0], c[1], z_lo - OV), (c[0], c[1], z_hi), n=32)
    head = G.cylinder(0.007, (c[0], c[1], z_lo - 0.004), (c[0], c[1], z_lo), n=32)
    return G.union([shank, head])


# =====================================================================================================================
# region lofts: trailing-edge closures, control surfaces, tip fairing
# =====================================================================================================================
def region_loft(op: OP, sts: list, region_fn, n_ring: int = 140, start_dir=(1.0, 0.0)) -> G.Mesh:
    rings = []
    for st in sts:
        reg = SG.largest(region_fn(st))
        P2 = SG.resample_ring(reg, n_ring, start_dir)
        rings.append(op.pt(st, P2[:, 0], P2[:, 1]))
    return _loft(rings)


def halfspace(point, normal, size: float = 2.0) -> G.Mesh:
    """Large box filling the half-space (p - point) . normal <= 0."""
    nrm = _unit(normal)
    ref = np.array([0.0, 0.0, 1.0]) if abs(nrm[2]) < 0.9 else np.array([1.0, 0.0, 0.0])
    e1 = _unit(np.cross(nrm, ref))
    e2 = np.cross(nrm, e1)
    c = np.asarray(point, float) - 0.5 * size * nrm
    return obox(c, [nrm, e1, e2], [0.5 * size, 0.5 * size, 0.5 * size])


def native_between(op: OP, y0: float, y1: float, step: float = 0.04) -> list:
    sts = skin_stations(op)
    e0, e1 = op.eta(y0), op.eta(y1)
    es = sorted({round(e0, 7), round(e1, 7)} | {round(st[0], 7) for st in sts if st[1] is None and e0 < st[0] < e1})
    out = []
    for a, b in zip(es, es[1:]):
        n = max(1, int(math.ceil((b - a) / step)))
        out += list(np.linspace(a, b, n + 1)[:-1])
    out.append(es[-1])
    return [op.stn(e) for e in out]


def te_region(op: OP, st) -> Polygon:
    reg = interior_poly(op, st, extra=2 * BOND, to_te=True)
    sr = op.s_rear(st[0])
    return SG.largest(reg.intersection(sbox(sr + 2 * BOND, -1.0, 2.0, 1.0)))


def te_closure(op: OP, kind: str, cutters=()) -> G.Mesh:
    """Fixed trailing-edge closure (foam core + CFRP skin) aft of the rear-spar web between the moving surfaces:
    'root' (root rib -> flap), 'mid' (flap -> aileron), 'tip' (aileron -> tip rib); end faces normal to the hinge
    axes (END_GAP to the moving surfaces)."""
    hf, ha = op.hinge("flap"), op.hinge("aileron")
    if kind == "root":
        y0, y1 = ROOT_RIB_Y + T_RIB + BOND, hf["y0"] + 0.03
        sts = [op.stp(y0)] + native_between(op, y0 + 0.004, y1)
        cut = [(op.hinge_point_at("flap", hf["y0"]), hf["axis"], -END_GAP)]
    elif kind == "mid":
        y0, y1 = hf["y1"] - 0.03, ha["y0"] + 0.03
        sts = native_between(op, y0, y1)
        cut = [(op.hinge_point_at("flap", hf["y1"]), -hf["axis"], -END_GAP),
               (op.hinge_point_at("aileron", ha["y0"]), ha["axis"], -END_GAP)]
    else:
        y0, y1 = ha["y1"] - 0.03, TIP_RIB_Y1 - T_RIB - BOND
        sts = native_between(op, y0, y1 - 0.004) + [op.stp(y1)]
        cut = [(op.hinge_point_at("aileron", ha["y1"]), -ha["axis"], -END_GAP)]
    m = region_loft(op, sts, lambda st: te_region(op, st))
    for p, a, off in cut:
        m = _inter(m, halfspace(np.asarray(p) + off * _unit(a), a))
    m = _diff(m, list(cutters))
    return finish(largest_piece(m))


def moving_region(op: OP, which: str, st) -> Polygon:
    poly, _fr = SG.section2d(op.W, st[0], n=240)
    _fixed, moving, _h = SG.control_surface_regions(0.75, gap=COVE_GAP)
    return moving(poly, op.chord(st[0]), st[0])


def surface_raw(op: OP, which: str) -> G.Mesh:
    """Moving surface (foam core + CFRP skin): structgen round-nose region lofted between the hinge-line end planes
    (normal to the layout hinge axis)."""
    h = op.hinge(which)
    sts = native_between(op, h["y0"] - 0.02, h["y1"] + 0.02, step=0.03)
    m = region_loft(op, sts, lambda st: moving_region(op, which, st), n_ring=160)
    m = _inter(m, halfspace(op.hinge_point_at(which, h["y1"]), h["axis"]))
    m = _inter(m, halfspace(op.hinge_point_at(which, h["y0"]), -h["axis"]))
    return m


def tip_fairing_mesh(op: OP, cutters=()) -> G.Mesh:
    """GFRP tip fairing (4 plies 7781, 1.0 mm): OML cap from the tip-rib outboard face to the tip section, pocket for
    the LT-WING position light (layout box + 0.5 mm)."""
    y0 = TIP_RIB_Y1 + BOND
    sts = [op.stp(y0), op.stp(y0 + 0.01), op.stp(y0 + 0.02)] + [op.stn(e) for e in
                                                               np.linspace(op.eta(y0 + 0.03), op.eta_end, 4)]

    def full(st):
        poly, _ = SG.section2d(op.W, st[0], n=200)
        return poly
    outer = region_loft(op, sts, full, n_ring=160)
    t = 0.0010
    inner_sts = [op.stp(y0 - 0.001)] + sts[1:-1]

    def shrunk(st):
        poly, _ = SG.section2d(op.W, st[0], n=200)
        return poly.buffer(-t, join_style=2)
    inner = region_loft(op, inner_sts, shrunk, n_ring=160)
    # inner cavity ends t before the tip cap
    end = op.stn(op.eta_end)
    o, c, u, w = op.frame(end[0])
    inner = _inter(inner, halfspace(o - t * w, w))
    m = _diff(outer, [inner] + list(cutters))
    return finish(largest_piece(m))


# =====================================================================================================================
# hinges (7075 clevis bracket on the rear-spar web, bonded 7075 tongue in the surface, stainless pin)
# =====================================================================================================================
HINGE_LUG_T = 0.003          # structures C-HINGE: 3 mm 7075 lugs
HINGE_GAP = 0.0005           # axial gap tongue / clevis lug
HINGE_BASE_T = 0.003
TONGUE_EMBED = 0.020         # tongue length bonded into the surface core
WEB_PAD_T = 0.0020           # 10-ply pad on the rear-spar web forward face under each bracket


def hframe(axis) -> tuple:
    a = _unit(axis)
    x = np.array([1.0, 0.0, 0.0])
    c = _unit(x - np.dot(x, a) * a)
    n = np.cross(c, a)
    return a, c, n


def hinge_geo(op: OP, which: str, y: float) -> dict:
    """Geometry of one hinge station: hinge point H, frame, web position, lug radius, bracket base size, bolt points
    (head on the base, -c), notch band for the surface."""
    h = op.hinge(which)
    H = op.hinge_point_at(which, y)
    a, c, n = hframe(h["axis"])
    st = op.stn(op.eta(y))
    sr = op.s_rear(st[0])
    _o, cs, us, _w = op.frame(st[0])
    sH, tH, r = op.hinge_cs(st[0])
    # web aft face and the free height between the skin lips just aft of it, in the hinge frame
    Pw = op.pt(st, [sr], [tH])[0]
    c_w = float(np.dot(Pw - H, c))
    s_b = sr + HINGE_BASE_T + 0.001
    tu = float(op.inner(st, "up", [s_b], extra=BOND)[1][0, 1])
    tl = float(op.inner(st, "lo", [s_b], extra=BOND)[1][0, 1])
    n_mid = float(np.dot(op.pt(st, [sr], [0.5 * (tu + tl)])[0] - H, n))
    h_free = tu - tl
    d = PIN_D[which]
    r_lug = 2.0 * d + 0.0005
    bd = BRKT_BOLT[which]
    D = bd * 1e-3
    from . import fastener_catalog as FCAT
    spacing = max(3 * D, FCAT.NUTPLATE[bd][0] + 0.002)
    base_w = spacing + 2 * 2.3 * D
    base_h = min(h_free - 0.002, 2 * r_lug + 0.006)
    half_clevis = 0.5 * HINGE_LUG_T + HINGE_GAP + HINGE_LUG_T
    band = 0.5 * base_w + 0.003
    bolts = [(H + c_w * c + 0.0002 * c + HINGE_BASE_T * c + k * 0.5 * spacing * a + n_mid * n, -c)
             for k in (-1, 1)]
    return {"H": H, "a": a, "c": c, "n": n, "c_w": c_w, "n_mid": n_mid, "r": r, "r_lug": r_lug, "d": d,
            "base_w": base_w, "base_h": base_h, "half_clevis": half_clevis, "band": band, "bolts": bolts,
            "bolt_d": bd, "st": st, "y": y}


def _prism_frame(poly, H, e_u, e_v, e_w, w0: float, w1: float) -> G.Mesh:
    """shapely polygon in (u, v) about H, extruded along e_w from w0 to w1 (e_u x e_v = +/- e_w handled)."""
    ww = np.cross(e_u, e_v)
    if np.dot(ww, e_w) > 0:
        return G.extrude(poly, w1 - w0, origin=H + w0 * e_w, u=e_u, v=e_v)
    m = G.extrude(poly, w1 - w0, origin=H + w1 * e_w, u=e_u, v=e_v)
    return m


def clevis_profile(g: dict) -> Polygon:
    """Lug profile in the hinge frame (c, n): disc r_lug about the axis + stem into the base plate."""
    hb = g["base_h"]
    c_in = g["c_w"] + 2 * BOND + 0.5 * HINGE_BASE_T          # stem starts inside the base thickness (fused)
    return unary_union([Point(0.0, 0.0).buffer(g["r_lug"], 48),
                        Polygon([(c_in, g["n_mid"] - 0.5 * hb + 0.001), (0.0, -g["r_lug"]), (0.0, g["r_lug"]),
                                 (c_in, g["n_mid"] + 0.5 * hb - 0.001)])])


def base_plate(op: OP, g: dict) -> G.Mesh:
    """Bracket base plate parallel to the rear-spar web aft face (lofted in the section frames like the web)."""
    rings = []
    half = 0.5 * g["base_w"]
    for dy in (-half, 0.0, half):
        st = op.stn(op.eta(g["y"] + dy * g["a"][1]))
        sr = op.s_rear(st[0])
        _sH, tH, _r = op.hinge_cs(st[0])
        tm = tH + g["n_mid"]
        s0, s1 = sr + 2 * BOND, sr + 2 * BOND + HINGE_BASE_T
        h2 = 0.5 * g["base_h"]
        rings.append(op.pt(st, [s0, s1, s1, s0], [tm - h2, tm - h2, tm + h2, tm + h2]))
    return _loft(rings)


def clevis_mesh(op: OP, g: dict) -> G.Mesh:
    """Fixed hinge bracket (machined 7075): base plate on the rear-spar web aft face + two lugs about the hinge axis."""
    H, a, c, n = g["H"], g["a"], g["c"], g["n"]
    prof = clevis_profile(g).difference(Point(0.0, 0.0).buffer(0.5 * g["d"] + 0.000025, 32))
    lugs = []
    t0 = 0.5 * HINGE_LUG_T + HINGE_GAP
    for sg in (-1, 1):
        w0, w1 = (t0, t0 + HINGE_LUG_T) if sg > 0 else (-t0 - HINGE_LUG_T, -t0)
        lugs.append(_prism_frame(prof, H, c, n, a, w0, w1))
    return finish(_union([base_plate(op, g)] + lugs))


def tongue_profile(g: dict, region_cn: Polygon | None) -> Polygon:
    rl = g["r_lug"]
    c_end = rl + 0.003 + TONGUE_EMBED
    trap = Polygon([(0.0, -rl), (c_end, -0.5 * g["h_t"]), (c_end, 0.5 * g["h_t"]), (0.0, rl)])
    if region_cn is not None:
        trap = trap.intersection(region_cn)
    prof = unary_union([Point(0.0, 0.0).buffer(rl, 48), trap])
    return SG.largest(prof).difference(Point(0.0, 0.0).buffer(0.5 * g["d"] + 0.000025, 32))


def section_cn(op: OP, which: str, g: dict, shrink: float) -> Polygon:
    """Moving-surface section at the hinge station in the hinge frame (c, n), shrunk by ``shrink``."""
    st = g["st"]
    reg = moving_region(op, which, st)
    P = np.asarray(reg.exterior.coords)
    Q = op.pt(st, P[:, 0], P[:, 1]) - g["H"]
    return Polygon(np.column_stack([Q @ g["c"], Q @ g["n"]])).buffer(0).buffer(-shrink, join_style=2)


def tongue_mesh_h(op: OP, which: str, g: dict) -> G.Mesh:
    reg = section_cn(op, which, g, CS_SKIN_T + 0.0008)
    th = reg.bounds
    g["h_t"] = min(2 * g["r_lug"], 0.010)
    prof = tongue_profile(g, reg)
    return _prism_frame(prof, g["H"], g["c"], g["n"], g["a"], -0.5 * HINGE_LUG_T, 0.5 * HINGE_LUG_T)


def notch_cutter(g: dict, lo: float, hi: float) -> G.Mesh:
    """Surface cut-out at a hinge: nose forward of the axis, a disc r_lug + 3 mm, and the bracket (+3 mm) as seen
    from the surface over its whole deflection range [lo, hi], across the bracket width + 3 mm each side."""
    from shapely import affinity
    br = clevis_profile(g).union(sbox(g["c_w"], g["n_mid"] - 0.5 * g["base_h"], g["c_w"] + 0.004,
                                      g["n_mid"] + 0.5 * g["base_h"])).buffer(0.003)
    shapes = [sbox(-0.08, -0.06, 0.0, 0.06), Point(0.0, 0.0).buffer(g["r_lug"] + 0.003, 48)]
    for dl in np.linspace(lo, hi, 17):
        # the surface frame rotates by +dl about a: in it the bracket appears rotated by -dl; (c, n) rotation sense
        # from e_c -> -e_n for +dl
        shapes.append(affinity.rotate(br, -math.degrees(-dl), origin=(0.0, 0.0)))
    prof = unary_union(shapes)
    return _prism_frame(prof, g["H"], g["c"], g["n"], g["a"], -g["band"], g["band"])


def slot_cutter(op: OP, which: str, g: dict) -> G.Mesh:
    reg = section_cn(op, which, g, CS_SKIN_T + 0.0008)
    prof = tongue_profile(g, reg).buffer(BOND, join_style=2)
    return _prism_frame(prof, g["H"], g["c"], g["n"], g["a"], -0.5 * HINGE_LUG_T - BOND, 0.5 * HINGE_LUG_T + BOND)


def web_pad(op: OP, g: dict) -> G.Mesh:
    """10-ply pad on the forward face of the rear-spar web under a hinge bracket (between the flanges)."""
    rings = []
    half = 0.5 * g["base_w"] + 0.008
    for dy in (-half, 0.0, half):
        st = op.stn(op.eta(g["y"] + dy / max(g["a"][1], 0.5)))
        C = rear_poly2(op, st)
        sr = op.s_rear(st[0])
        tw = op.t_rweb()
        ub = C[3 * N_FL:][::-1]
        lb = C[2 * N_FL:3 * N_FL]
        s0, s1 = sr - tw - WEB_PAD_T, sr - tw + OV
        tu = float(np.interp(s0, ub[:, 0], ub[:, 1])) + OV
        tl = float(np.interp(s0, lb[:, 0], lb[:, 1])) - OV
        rings.append(op.pt(st, [s0, s1, s1, s0], [tl, tl, tu, tu]))
    return _loft(rings)


# =====================================================================================================================
# servo bays, four-bar linkages, servo / arm / push-rod / horn, hatch frame and cover with the linkage fairing
# =====================================================================================================================
BAY_RIBS = {"flap": (1.15, 0.83), "aileron": (2.26, 1.99)}    # (outboard, inboard) rib y of each servo bay
FRAME_T = 0.0020             # hatch frame (doubler ring, 10 plies PW) bonded on the skin inner face
FRAME_BORDER = 0.010         # frame border on the skin round the opening
LAND_SPAN, LAND_CHORD = 0.018, 0.007     # frame land inside the opening (spanwise / chordwise edges)
COVER_GAP = 0.001            # cover to opening gap
COVER_RIM_T = 0.0058         # cover rim = shell_secondary sandwich (flush 0.1 mm under the OML)
COVER_MID_T = 0.0020         # cover middle (solid laminate, servo cradle bonded on it)
CRADLE_WALL = 0.003
CRADLE_BASE_MIN = 0.002
SERVO_BOLT = 4
ARM_T = 0.003                # servo arm / horn plate (7075, structures C-RODEND: 3 mm horn)
EYE_R, EYE_T = 0.0065, 0.005 # rod-end eye (>= 2 D edge distance for the M3 rod-end bolt)
ROD_OD, ROD_ID = 0.006, 0.004            # push-rod 7075 tube 6 x 1 (structures C-PUSHROD)
ACT_KEY = {"flap": "volz_da30", "aileron": "volz_da26"}
ACT_PART = {"flap": "ACT-FLAP", "aileron": "ACT-AILERON"}


def servo_dims(which: str) -> dict:
    """Case L (chordwise) x W (vertical) x H (along the shaft = hinge axis), shaft offset from the aft edge, hole
    pattern along H, mass (components.yaml datasheet values; DA 30: envelope incl. D-Sub)."""
    d = actuator_data(ACT_KEY[which])
    if which == "aileron":
        L, H, W = d["case_dimensions_m"]
        off = float(d["mounting"]["output_axis_from_case_edge_m"])
        hp = float(d["mounting"]["flange_hole_pattern_m"][0])
    else:
        H, L, W = d["envelope_dimensions_m"]
        off = 0.010                                   # output shaft 10 mm from the aft face (layout ACT-FLAP)
        hp = float(d["mounting"]["flange_hole_pattern_m"][0])
    return {"L": float(L), "W": float(W), "H": float(H), "off": off, "hole_pitch": hp, "mass": float(d["mass_kg"]),
            "model": d["model"]}


def eta_of_point(op: OP, P) -> float:
    """Native station (eta) whose section plane contains the point P."""
    P = np.asarray(P, float)
    e = op.eta(float(P[1]))
    for _ in range(8):
        o, _c, _u, w = op.frame(e)
        f = float(np.dot(w, P - o))
        if abs(f) < 1e-9:
            break
        e += f
    return e


def opening_s(op: OP, eta: float) -> tuple[float, float]:
    """Chord range of a hatch opening at a native station: FRAME_BORDER + 2 mm inside the full-depth sandwich of the
    lower box skin (between the main-spar cap ramp b6 and the rear-flange ramp b7), i.e. parallel to the spars."""
    k = op.knots(eta, "lo")
    return k["b6"] + FRAME_BORDER + 0.002, k["b7"] - FRAME_BORDER - 0.002


def _drive_at(op: OP, which: str, yd: float, c_S: float | None = None) -> dict:
    """Four-bar and servo pose for the rod plane at hinge station yd (chordwise shaft position c_S, or the most
    forward position the cover land allows)."""
    h = op.hinge(which)
    a, c, n = hframe(h["axis"])
    sd = servo_dims(which)
    lk = op.ctl[which]["linkage"]
    k = LEVER_K[which]
    r_s = float(lk["servo_arm_m"]) * k
    r_h = r_s * float(lk["arm_ratio"])
    d_n = math.radians(float(lk.get("neutral_deg", 0.0)))
    Hd = op.hinge_point_at(which, yd)
    a1 = -0.0106
    a0 = a1 - sd["H"]

    def case_pts(cS, nS):
        S_ = Hd + cS * c + nS * n
        cf = sd["off"] - sd["L"] - BOND - CRADLE_WALL - 0.0025            # incl. the M4 low-head screw heads
        ca = sd["off"] + BOND + CRADLE_WALL + 0.0025
        return [S_ + cc * c + nn * n + aa * a for cc in (cf, ca) for nn in (-0.5 * sd["W"] - 0.003, 0.5 * sd["W"])
                for aa in (a0, a1)]
    # chordwise: forward bolt heads behind the forward land (+3 mm), checked at both case ends
    if c_S is None:
        c_S = -0.5 * op.chord(op.eta(yd))
        for _ in range(4):
            worst = 1.0
            for P in case_pts(c_S, -0.01):
                e = eta_of_point(op, P)
                s_lim = opening_s(op, e)[0] + COVER_GAP + LAND_CHORD + 0.003
                worst = min(worst, op.s_of_x(e, P[0]) - s_lim)
            c_S -= worst
    # vertical: case bottom on the cradle base over the cover middle (OML + 0.1 + COVER_MID_T + bond)
    bott = []
    for P in case_pts(c_S, 0.0):
        e = eta_of_point(op, P)
        s0 = op.s_of_x(e, P[0])
        for ss in (s0 - 0.01, s0, s0 + 0.01):
            sg = op.sig_of_s(e, "lo", [ss])
            s_, t_, ns_, nt_ = op.at_sig(e, "lo", sg)
            off = 0.0001 + COVER_MID_T + 1.5 * BOND
            Q = op.to3d(e, s_ + off * ns_, t_ + off * nt_)[0]
            bott.append(float(np.dot(Q - Hd, n)))
    n_bottom = max(bott) + CRADLE_BASE_MIN
    n_S = n_bottom + 0.5 * sd["W"]
    S = Hd + c_S * c + n_S * n
    A_n = S - r_s * n
    B_n = Hd - r_h * n
    for _ in range(40):
        rod = _unit(B_n - A_n)
        hd = np.cross(a, rod)
        if np.dot(hd, n) > 0:
            hd = -hd
        A_n = S + r_s * hd
        B_n = Hd + r_h * hd
    from . import actuation as A
    B0 = A.rotate_about(B_n[None], Hd, a, -d_n)[0] if d_n else B_n.copy()
    ref = np.array([0, 0, 1.0]) if abs(a[2]) < 0.9 else np.array([1.0, 0, 0])
    e1 = _unit(np.cross(a, ref))
    e2 = np.cross(a, e1)
    th_n = math.atan2(np.dot(A_n - S, e2), np.dot(A_n - S, e1))
    L_rod = float(np.linalg.norm(B_n - A_n))
    th = th_n
    for _ in range(80):
        def f(t):
            return np.linalg.norm(S + r_s * (math.cos(t) * e1 + math.sin(t) * e2) - B0) - L_rod
        df = (f(th + 1e-7) - f(th - 1e-7)) / 2e-7
        th -= f(th) / df
    A0 = S + r_s * (math.cos(th) * e1 + math.sin(th) * e2)
    kin = A.linkage_kinematics(Hd, a, B0, S, a, r_s, th, (h["lo"], h["hi"]), n=41)
    return {"which": which, "a": a, "c": c, "n": n, "Hd": Hd, "yd": yd, "std": op.stn(op.eta(yd)), "S": S,
            "A0": A0, "B0": B0, "A_n": A_n, "B_n": B_n, "th0": th, "e1": e1, "e2": e2, "r_s": r_s, "r_h": r_h,
            "L_rod": L_rod, "kin": kin, "sd": sd, "c_S": c_S, "n_bottom": n_bottom, "lo": h["lo"], "hi": h["hi"],
            "d_n": d_n, "case_pts": case_pts(c_S, n_S)}


def drive_geo(op: OP, which: str) -> dict:
    """Servo bay, hatch opening, servo position and the four-bar of one surface (starboard). The rod plane station
    is placed so the servo case (+ cradle and screw heads) clears the inboard cover land and the linkage blister clears
    the outboard land by the same margin (both edges are native stations; the drive is aligned with the swept hinge
    axis, so its plan footprint is skewed against them)."""
    cache = op.__dict__.setdefault("_dg", {})
    if which in cache:
        return cache[which]
    y_out, y_in = BAY_RIBS[which]
    yo0 = y_in + 0.5 * T_RIB + RIB_FL_W + FRAME_BORDER + 0.002
    yo1 = y_out - 0.5 * T_RIB - RIB_FL_W - FRAME_BORDER - 0.002
    e_in = op.eta(yo0 + LAND_SPAN)
    e_out = op.eta(yo1 - LAND_SPAN)
    yd = 0.5 * (yo0 + yo1)
    for _ in range(6):
        dg = _drive_at(op, which, yd)
        m_in = min(eta_of_point(op, P) for P in dg["case_pts"]) - e_in
        env = linkage_envelope(dg).buffer(CAV_CLR + BLISTER_T)
        a1 = -0.0106
        a_hi = a1 + 0.0106 + 0.5 * ROD_OD + CAV_CLR + BLISTER_T
        pts = [dg["Hd"] + x * dg["c"] + z * dg["n"] + a_hi * dg["a"] for x, z in np.asarray(env.exterior.coords)
               if z < dg["n_bottom"] - CRADLE_BASE_MIN]
        m_out = e_out - max(eta_of_point(op, P) for P in pts)
        if abs(m_out - m_in) < 2e-4:
            break
        yd += 0.5 * (m_out - m_in) / dg["a"][1]
    if min(m_in, m_out) < 0.003:
        raise ValueError(f"{which} drive does not fit its servo bay (margins {m_in:.4f} / {m_out:.4f} m)")
    dg["opening"] = {"y": (yo0, yo1)}
    dg["margins"] = (m_in, m_out)
    cache[which] = dg
    return dg


def linkage_states(dg: dict, n: int = 13) -> list:
    """Exact coupled joint values over the deflection range: (delta, arm angle from rest, rod angle from rest)."""
    from . import actuation as A
    S, a, e1, e2 = dg["S"], dg["a"], dg["e1"], dg["e2"]
    out = []
    for dl in np.linspace(dg["lo"], dg["hi"], n):
        k = A.linkage_kinematics(dg["Hd"], a, dg["B0"], S, a, dg["r_s"], dg["th0"], (0.0, dl) if dl != 0 else
                                 (0.0, 1e-9), n=max(3, int(abs(dl) / 0.02) + 3))
        th = float(k["servo_angles"][-1])
        A1 = S + dg["r_s"] * (math.cos(th) * e1 + math.sin(th) * e2)
        B1 = A.rotate_about(dg["B0"][None], dg["Hd"], a, dl)[0]
        # rod angle: rotation about a taking (B0 - A0) rotated by the arm angle to (B1 - A1)
        v0 = A.rotate_about((dg["B0"] - dg["A0"])[None], np.zeros(3), a, th - dg["th0"])[0]
        v1 = B1 - A1
        x0 = np.dot(v0, e1), np.dot(v0, e2)
        x1 = np.dot(v1, e1), np.dot(v1, e2)
        ph = math.atan2(x1[1], x1[0]) - math.atan2(x0[1], x0[0])
        ph = (ph + math.pi) % (2 * math.pi) - math.pi
        out.append((float(dl), float(th - dg["th0"]), float(ph)))
    return out


def _local(dg: dict, c: float, n: float, a: float = 0.0, ref=None) -> np.ndarray:
    """Point at (c, n, a) of the drive frame about the hinge point of the rod plane (or about ``ref``)."""
    o = dg["Hd"] if ref is None else ref
    return o + c * dg["c"] + n * dg["n"] + a * dg["a"]


def a_case_end(dg: dict) -> float:
    """Spanwise coordinate (along the hinge axis, from the rod plane) of the servo case outboard end face."""
    return -0.0106


def servo_mesh(dg: dict) -> G.Mesh:
    """Actuator envelope (datasheet dimensions): case L x W x H, output shaft d 6 x 8 mm on the outboard end face."""
    sd = dg["sd"]
    S = dg["S"]
    a, c, n = dg["a"], dg["c"], dg["n"]
    a1 = a_case_end(dg)
    a0 = a1 - sd["H"]
    c_aft = sd["off"]
    centre = S + (c_aft - 0.5 * sd["L"]) * c + 0.5 * (a0 + a1) * a
    case = obox(centre, [c, n, a], [0.5 * sd["L"], 0.5 * sd["W"], 0.5 * sd["H"]])
    shaft = G.cylinder(0.003, S + (a1 - OV) * a, S + (a1 + 0.008) * a, n=24)
    return G.union([case, shaft])


def cradle_mesh(op: OP, dg: dict) -> tuple[G.Mesh, list]:
    """7075 servo cradle bonded on the hatch cover: base (follows the cover inner face) and two walls along the
    case's forward / aft faces with 2 + 2 M4 screws into the case sides (tapped). Returns (mesh, bolts)."""
    sd = dg["sd"]
    S = dg["S"]
    a, c, n = dg["a"], dg["c"], dg["n"]
    a1 = a_case_end(dg) - 0.004
    a0 = a_case_end(dg) - sd["H"] + 0.004
    cf = sd["off"] - sd["L"] - BOND - CRADLE_WALL       # forward wall outer face (c from the shaft)
    ca = sd["off"] + BOND + CRADLE_WALL
    n_b = -0.5 * sd["W"] - BOND                         # base top = case bottom - bond (n from the shaft)
    # base: loft along a of sections (c, n): bottom follows the cover middle inner face
    rings = []
    for aa in np.linspace(a0, a1, 7):
        P0 = S + aa * a
        cs = np.linspace(cf, ca, 24)
        top = [P0 + cc * c + n_b * n for cc in cs]
        bot = []
        for p in top:
            e = eta_of_point(op, p)
            s_ = op.s_of_x(e, p[0])
            sg = op.sig_of_s(e, "lo", [s_])
            s2, t2, ns2, nt2 = op.at_sig(e, "lo", sg)
            off = 0.0001 + COVER_MID_T + 1.5 * BOND
            q = op.to3d(e, s2 + off * ns2, t2 + off * nt2)[0]
            # bottom point: on the line through p along -n, at the cover face height
            bot.append(p + (np.dot(q - p, n)) * n)
        rings.append(np.vstack([np.asarray(top), np.asarray(bot)[::-1]]))
    base = _loft(rings)
    hw = 0.85 * sd["W"]
    walls = []
    for c0, c1 in ((cf, cf + CRADLE_WALL), (ca - CRADLE_WALL, ca)):
        ctr = S + 0.5 * (c0 + c1) * c + (n_b - 0.0015 + 0.5 * (hw + 0.0015)) * n + 0.5 * (a0 + a1) * a
        walls.append(obox(ctr, [c, n, a], [0.5 * (c1 - c0), 0.5 * (hw + 0.0015), 0.5 * (a1 - a0)]))
    m = finish(_union([base] + walls))
    am = 0.5 * (a_case_end(dg) - sd["H"] + a_case_end(dg))
    bolts = []
    for aa in (am - 0.5 * sd["hole_pitch"], am + 0.5 * sd["hole_pitch"]):
        bolts.append((S + cf * c + aa * a, c))                 # forward wall: head forward, into the case (+c)
        bolts.append((S + ca * c + aa * a, -c))                # aft wall: head aft
    return m, bolts


def arm_mesh(dg: dict) -> G.Mesh:
    """7075 servo arm on the output spline (hub d 12, plate 3 mm), eye at the rod-end bolt."""
    S, A0, a = dg["S"], dg["A0"], dg["a"]
    e1, e2 = dg["e1"], dg["e2"]
    a1 = a_case_end(dg)

    def uv(p):
        d = p - S
        return (float(np.dot(d, e1)), float(np.dot(d, e2)))
    pa = uv(A0)
    prof = unary_union([Point(0, 0).buffer(0.006, 40), Point(*pa).buffer(EYE_R, 40),
                        LineString([(0, 0), pa]).buffer(0.0045)])
    prof = prof.difference(Point(0, 0).buffer(0.003 + BOND, 32))
    plate = G.extrude(prof, ARM_T, origin=S + (a1 + 0.005) * a, u=e1, v=e2)
    if np.dot(np.cross(e1, e2), a) < 0:
        plate = plate.translated(ARM_T * a)
    hub = G.tube(0.006, 0.003 + BOND, S + (a1 + 0.0012) * a, S + (a1 + 0.005 + OV) * a, n=32)
    return finish(_union([plate, hub]))


def rod_mesh(dg: dict) -> G.Mesh:
    """Push-rod: 7075 tube 6 x 1 with two rod-end eyes (r 6.5, 5 mm) in the rod plane."""
    A0, B0, a = dg["A0"], dg["B0"], dg["a"]
    a_e0 = a_case_end(dg) + 0.005 + ARM_T + BOND
    ctrA = A0 + (a_e0 - np.dot(A0 - dg["Hd"], a)) * a
    ctrB = B0 + (a_e0 - np.dot(B0 - dg["Hd"], a)) * a
    d = _unit(ctrB - ctrA)
    am = a_e0 + 0.5 * EYE_T
    pA = ctrA + 0.5 * EYE_T * a
    pB = ctrB + 0.5 * EYE_T * a
    eyes = [G.cylinder(EYE_R, c0, c0 + EYE_T * a, n=36) for c0 in (ctrA, ctrB)]
    necks = [G.cylinder(0.0024, pA + (EYE_R - 0.001) * d, pA + 0.020 * d, n=20),
             G.cylinder(0.0024, pB - (EYE_R - 0.001) * d, pB - 0.020 * d, n=20)]
    tube = G.tube(0.5 * ROD_OD, 0.5 * ROD_ID, pA + 0.018 * d, pB - 0.018 * d, n=20)
    return finish(_union(eyes + necks + [tube]))


def horn_mesh(op: OP, dg: dict) -> tuple[G.Mesh, list, list]:
    """7075 horn on the surface's lower face: L-flange (3 mm, follows the OML, 2 x M3 into potted inserts of the
    surface) and the 3 mm horn plate in the arm plane down to the rod-end hole at B. Returns (mesh, flange bolts,
    rod-end bolt point)."""
    B0, Hd, a, c, n = dg["B0"], dg["Hd"], dg["a"], dg["c"], dg["n"]
    a_p0 = a_case_end(dg) + 0.005                      # horn plate a-range (same plane as the arm plate)
    a_p1 = a_p0 + ARM_T
    a_f0 = a_p0 - 0.014                                 # flange extends inboard of the plate
    # flange grid on the lower OML (outward offset BOND .. BOND + 3 mm), chord from H + 4 .. H + 34 mm
    Po, Pi = [], []
    for aa in np.linspace(a_f0, a_p1, 5):
        P0 = Hd + aa * a
        st = op.stn(op.eta(P0[1]))
        sH, tH, r = op.hinge_cs(st[0])
        ss = np.linspace(sH + 0.004, sH + 0.034, 12)
        sg = op.sig_of_s(st[0], "lo", ss)
        s2, t2, ns2, nt2 = op.at_sig(st[0], "lo", sg)
        Po.append(op.pt(st, s2 - (BOND + 0.003) * ns2, t2 - (BOND + 0.003) * nt2))
        Pi.append(op.pt(st, s2 - BOND * ns2, t2 - BOND * nt2))
    flange = _slab(np.stack(Po, 0), np.stack(Pi, 0))
    # plate: hull of the flange's mid line (in the plate plane) and the eye at B
    mid = 0.5 * (np.stack(Po, 0) + np.stack(Pi, 0))[-2]           # line near the plate
    pts = [(float(np.dot(p - Hd, c)), float(np.dot(p - Hd, n))) for p in mid]
    pb = (float(np.dot(B0 - Hd, c)), float(np.dot(B0 - Hd, n)))
    root = LineString(pts).buffer(0.0012)
    prof = unary_union([root, Point(*pb).buffer(EYE_R, 40)]).convex_hull
    plate = _prism_frame(prof, Hd, c, n, a, a_p0, a_p1)
    m = finish(_union([flange, plate]))
    fl_bolts = []
    for k_ in (0.25, 0.75):
        i = int(round(k_ * 11))
        P_out = np.stack(Po, 0)[1][i]                        # a row inboard of the plate
        P_in = np.stack(Pi, 0)[1][i]
        fl_bolts.append((P_out, _unit(P_in - P_out)))
    rod_bolt = (B0 + (a_p0 - np.dot(B0 - Hd, a)) * a, a)
    return m, fl_bolts, rod_bolt


# ---------------------------------------------------------------------------------------------------------------------
# hatch: skin opening, frame (doubler ring with lands), cover with the linkage fairing (blister)
# ---------------------------------------------------------------------------------------------------------------------
def lower_slab(op: OP, y0: float, y1: float, s0: float, s1: float, d0, d1, n_s: int = 28,
               step: float = 0.02) -> G.Mesh:
    """Solid between two offsets of the lower OML (d0 < d1 inward along the local normal; callables of (eta, s)
    allowed) over native stations y0..y1 and chord s0..s1 (same s range at every station)."""
    sts = native_between(op, y0, y1, step)
    Po, Pi = [], []
    for st in sts:
        eta = st[0]
        ss = np.linspace(s0(eta) if callable(s0) else s0, s1(eta) if callable(s1) else s1, n_s)
        sg = op.sig_of_s(eta, "lo", ss)
        s_, t_, ns_, nt_ = op.at_sig(eta, "lo", sg)
        a = d0(eta, s_) if callable(d0) else np.full_like(s_, d0)
        b = d1(eta, s_) if callable(d1) else np.full_like(s_, d1)
        Po.append(op.pt(st, s_ + a * ns_, t_ + a * nt_))
        Pi.append(op.pt(st, s_ + b * ns_, t_ + b * nt_))
    return _slab(np.stack(Po, axis=1), np.stack(Pi, axis=1))


def hatch_geo(op: OP, which: str) -> dict:
    dg = drive_geo(op, which)

    def sf(k: int, d: float):
        return lambda e: opening_s(op, e)[k] + d
    return {"dg": dg, "y": dg["opening"]["y"], "s": sf}


def opening_cutter(op: OP, hg: dict) -> G.Mesh:
    (y0, y1), sf = hg["y"], hg["s"]
    return lower_slab(op, y0, y1, sf(0, 0.0), sf(1, 0.0), -0.004, lambda e, s: op.t_skin(e, "lo", s) + 0.0005)


def frame_mesh(op: OP, hg: dict) -> G.Mesh:
    """Hatch frame: 10-ply PW ring bonded on the lower skin inner face, FRAME_BORDER round the opening, lands
    LAND_CHORD / LAND_SPAN inside the opening carrying the cover rim and the four nutplates."""
    (y0, y1), sf = hg["y"], hg["s"]

    def d0(e, s):
        return op.t_skin(e, "lo", s) + 1.5 * BOND

    def d1(e, s):
        return op.t_skin(e, "lo", s) + 1.5 * BOND + FRAME_T
    outer = lower_slab(op, y0 - FRAME_BORDER, y1 + FRAME_BORDER, sf(0, -FRAME_BORDER), sf(1, FRAME_BORDER), d0, d1)
    hole = lower_slab(op, y0 + LAND_SPAN, y1 - LAND_SPAN, sf(0, LAND_CHORD), sf(1, -LAND_CHORD),
                      lambda e, s: d0(e, s) - 0.002, lambda e, s: d1(e, s) + 0.002)
    return finish(_diff(outer, [hole]))


CAV_CLR = 0.0025             # linkage cavity: clearance round the swept arm / rod envelope
BLISTER_T = 0.0020           # blister wall (= cover middle laminate)


def linkage_envelope(dg: dict, n: int = 17) -> Polygon:
    """Swept envelope (drive-plane c, n about the hinge point) of the servo arm and the push-rod over the full surface
    range (exact four-bar states)."""
    from . import actuation as A
    Hd, a, c, nn = dg["Hd"], dg["a"], dg["c"], dg["n"]

    def cn(p):
        d = p - Hd
        return (float(np.dot(d, c)), float(np.dot(d, nn)))
    S = cn(dg["S"])
    shapes = []
    for dl, dth, _ph in linkage_states(dg, n):
        th = dg["th0"] + dth
        A1 = dg["S"] + dg["r_s"] * (math.cos(th) * dg["e1"] + math.sin(th) * dg["e2"])
        B1 = A.rotate_about(dg["B0"][None], Hd, a, dl)[0]
        pa, pb = cn(A1), cn(B1)
        shapes += [Point(*S).buffer(0.006, 32), LineString([S, pa]).buffer(0.0045), Point(*pa).buffer(EYE_R, 32),
                   LineString([pa, pb]).buffer(0.5 * ROD_OD), Point(*pb).buffer(EYE_R, 32)]
    return unary_union(shapes)


def cover_mesh(op: OP, hg: dict) -> tuple[G.Mesh, G.Mesh]:
    """Hatch cover: 2 mm PW laminate flush 0.1 mm under the OML, rim built up to the skin inner face over the frame
    lands, with the integral linkage blister (2 mm wall round the swept arm / rod envelope + 2.5 mm, open aft where
    the rod leaves). Returns (cover, cavity cutter)."""
    dg = hg["dg"]
    (y0, y1), sf = hg["y"], hg["s"]
    g = COVER_GAP

    def rim_in(e, s):
        return op.t_skin(e, "lo", s) - 0.0001
    mid = lower_slab(op, y0 + g, y1 - g, sf(0, g), sf(1, -g), 0.0001, 0.0001 + COVER_MID_T)
    rim = lower_slab(op, y0 + g, y1 - g, sf(0, g), sf(1, -g), 0.0001 + COVER_MID_T - OV, rim_in)
    rim_hole = lower_slab(op, y0 + LAND_SPAN, y1 - LAND_SPAN, sf(0, LAND_CHORD), sf(1, -LAND_CHORD), 0.0, 0.010)
    rim = _diff(rim, [rim_hole])
    env = linkage_envelope(dg)
    a1 = a_case_end(dg)
    a_lo, a_hi = a1 + 0.005 - CAV_CLR, a1 + 0.0106 + 0.5 * ROD_OD + CAV_CLR
    cav_poly = env.buffer(CAV_CLR, 24)
    cavity = _prism_frame(cav_poly, dg["Hd"], dg["c"], dg["n"], dg["a"], a_lo, a_hi)
    outer_poly = cav_poly.buffer(BLISTER_T, 24)
    blister = _prism_frame(outer_poly, dg["Hd"], dg["c"], dg["n"], dg["a"], a_lo - BLISTER_T, a_hi + BLISTER_T)
    outside = lower_slab(op, y0 + g, y1 - g, sf(0, g), sf(1, -g), -0.06, 0.0001 + OV)
    blister = _inter(blister, outside)
    cover = _diff(_union([mid, rim, blister]), [cavity])
    return finish(largest_piece(cover)), cavity
