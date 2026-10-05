"""Shared actuation and hinge hardware generators — shared math, registers no parts.

Used by wing.py and tail.py (and gear.py for door links) so every control surface uses the same, consistent
hardware family:

* :func:`hinge_bracket_fixed`  CNC aluminium clevis bracket (base flange + two lugs), bolted to a spar web.
* :func:`hinge_bracket_moving` single-lug tongue bracket on the control-surface spar, rotating inside the clevis.
* :func:`hinge_pin`            shoulder bolt / pin along the hinge axis (clearance fit in both brackets).
* :func:`servo_body`           actuator envelope with mounting flanges and output shaft (from datasheet dims).
* :func:`servo_arm`, :func:`control_horn`, :func:`pushrod` (rod + two rod-end bearings).
* :func:`linkage_kinematics`   four-bar check: rod length constant, transmission angle limits, servo travel needed.

Every generator returns closed :class:`Mesh` objects in the aircraft frame plus the fastener/attachment points the
producer needs to create :class:`Fastener` records. Dimensions are SI (m).
"""
from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np
from shapely.geometry import LineString, Point, Polygon
from shapely.ops import unary_union

from ..core.geom import Mesh, box, cylinder, extrude, frame, rot_axis_angle, rotate_about, union, unit

# ISO 273 medium clearance holes (m) for metric screws
CLEARANCE = {2.5: 0.0029, 3: 0.0034, 4: 0.0045, 5: 0.0055, 6: 0.0066, 8: 0.009, 10: 0.011, 12: 0.0135}


def clearance_hole(d_nominal_mm: float) -> float:
    return CLEARANCE.get(d_nominal_mm, d_nominal_mm * 1.1e-3)


# =====================================================================================================================
# hinges
# =====================================================================================================================
@dataclass
class HingeSpec:
    pin_d: float = 0.005            # hinge bolt shank diameter (shoulder bolt), H7/g6 clearance in lugs
    lug_t: float = 0.004            # lug thickness
    lug_r: float = 0.008            # lug outer radius around the pin (>= 1.5 x pin_d edge margin)
    gap: float = 0.0005             # axial gap between moving tongue and each clevis lug
    base_t: float = 0.003           # base flange thickness
    base_w: float = 0.030           # base flange width (along the hinge line)
    base_h: float = 0.024           # base flange height (on the spar web)
    bolt_d_mm: float = 4.0          # flange bolts (2 per bracket)
    pin_clear: float = 0.00002      # radial clearance pin/lug bore (H7/g6 ~ 10-30 um)


def _lug_profile(r_out: float, hole_r: float, reach: float, base_w: float) -> Polygon:
    """2-D lug in its own plane: rounded end of radius r_out centred at (0, 0), tapering to a base of width base_w at
    distance ``reach`` along -v (toward the base flange), with the pin hole."""
    end = Point(0.0, 0.0).buffer(r_out, 48)
    stem = Polygon([(-base_w / 2, -reach), (base_w / 2, -reach), (r_out, 0.0), (-r_out, 0.0)])
    return unary_union([end, stem]).difference(Point(0, 0).buffer(hole_r, 48))


def hinge_bracket_fixed(axis_p, axis_dir, mount_normal, hs: HingeSpec, reach: float) -> dict:
    """Clevis bracket for a hinge at ``axis_p`` (point on the hinge line) with hinge direction ``axis_dir``.
    ``mount_normal`` is the unit normal of the spar web it bolts to, pointing FROM the web TOWARD the hinge line;
    ``reach`` is the distance from the web face to the hinge axis. Returns {"mesh", "bolts": [(pos, axis)],
    "web_point"}."""
    a = unit(axis_dir)
    n = unit(np.asarray(mount_normal, float) - np.dot(mount_normal, a) * a)
    Fr = frame(n, a)                                        # e1 = n (toward hinge), e2 = a, e3 = n x a
    p = np.asarray(axis_p, float)
    web_pt = p - reach * n
    tongue_t = hs.lug_t                                     # moving tongue thickness equals lug thickness
    span = tongue_t + 2 * hs.gap + 2 * hs.lug_t            # clevis outer width along the axis
    parts = []
    hole_r = hs.pin_d / 2 + hs.pin_clear
    for side in (-1, 1):
        off = side * (tongue_t / 2 + hs.gap + hs.lug_t / 2)
        prof = _lug_profile(hs.lug_r, hole_r, reach - 0.5 * hs.base_t, min(hs.base_h, 2.2 * hs.lug_r))  # 0.5 t into the base: fused union
        # lug plane spanned by n (toward hinge = -v in profile coords flipped) and e3; thickness along a
        u_dir = Fr[:, 2]
        v_dir = n                                           # profile +v points toward the hinge from the stem side
        origin = p + off * a
        lug = extrude(prof, hs.lug_t, origin=origin - 0.5 * hs.lug_t * a, u=u_dir, v=v_dir)
        parts.append(lug)
    base = box((hs.base_t, max(span, hs.base_w), hs.base_h), center=web_pt + 0.5 * hs.base_t * n, R=Fr)
    parts.append(base)
    mesh = union(parts)
    bolts = []
    for side in (-1, 1):
        bp = web_pt + side * (max(span, hs.base_w) / 2 - 2.0 * hs.bolt_d_mm * 1e-3) * a
        bolts.append((bp + hs.base_t * n, -n))
    return {"mesh": mesh, "bolts": bolts, "web_point": web_pt, "span": max(span, hs.base_w)}


def hinge_bracket_moving(axis_p, axis_dir, mount_normal, hs: HingeSpec, reach: float) -> dict:
    """Single-lug tongue on the control-surface side; ``mount_normal`` points from the surface spar web toward the
    hinge line, ``reach`` = web face to axis distance."""
    a = unit(axis_dir)
    n = unit(np.asarray(mount_normal, float) - np.dot(mount_normal, a) * a)
    Fr = frame(n, a)
    p = np.asarray(axis_p, float)
    web_pt = p - reach * n
    hole_r = hs.pin_d / 2 + hs.pin_clear
    prof = _lug_profile(hs.lug_r, hole_r, reach - 0.5 * hs.base_t, min(hs.base_h, 2.2 * hs.lug_r))  # 0.5 t into the base: fused union
    lug = extrude(prof, hs.lug_t, origin=p - 0.5 * hs.lug_t * a, u=Fr[:, 2], v=n)
    base = box((hs.base_t, hs.base_w, hs.base_h), center=web_pt + 0.5 * hs.base_t * n, R=Fr)
    mesh = union([lug, base])
    bolts = [(web_pt + side * (hs.base_w / 2 - 2.0 * hs.bolt_d_mm * 1e-3) * a + hs.base_t * n, -n)
             for side in (-1, 1)]
    return {"mesh": mesh, "bolts": bolts, "web_point": web_pt}


def hinge_pin(axis_p, axis_dir, hs: HingeSpec, length: float | None = None, head: bool = True) -> Mesh:
    """Shoulder-bolt hinge pin centred on ``axis_p`` along ``axis_dir`` (head on the negative side)."""
    a = unit(axis_dir)
    L = length or (hs.lug_t * 3 + 2 * hs.gap + 0.006)
    p = np.asarray(axis_p, float)
    shank = cylinder(hs.pin_d / 2, p - 0.5 * L * a, p + 0.5 * L * a, n=24)
    if not head:
        return shank
    h = cylinder(hs.pin_d * 0.85, p - (0.5 * L + 0.0025) * a, p - (0.5 * L - 1e-4) * a, n=24)
    return union([shank, h])


# =====================================================================================================================
# actuators and linkage
# =====================================================================================================================
@dataclass
class ServoSpec:
    body: tuple = (0.040, 0.020, 0.037)     # L (along output-shaft offset dir), W, H (along shaft) of the case
    flange: tuple = (0.054, 0.020, 0.0025)  # mounting flange length, width, thickness
    flange_z: float = 0.026                 # flange position from case bottom along the shaft axis
    shaft_offset: float = 0.010             # output shaft offset from case centre along L
    shaft_r: float = 0.003
    shaft_len: float = 0.006
    hole_d_mm: float = 3.0                  # flange screws
    hole_pitch: float = 0.048               # flange hole spacing along L
    mass_kg: float = 0.07
    model: str = ""


def servo_body(origin, L_dir, shaft_dir, ss: ServoSpec) -> dict:
    """Actuator envelope: case bottom centre at ``origin``, case length along ``L_dir``, output shaft along
    ``shaft_dir``. Returns {"mesh", "shaft_p" (top of shaft), "flange_holes": [(pos, axis)]}."""
    l = unit(L_dir)
    s = unit(np.asarray(shaft_dir, float) - np.dot(shaft_dir, l) * l)
    Fr = np.column_stack([l, np.cross(s, l), s])
    o = np.asarray(origin, float)
    Lb, Wb, Hb = ss.body
    case = box((Lb, Wb, Hb), center=o + 0.5 * Hb * s, R=Fr)
    fl = box(ss.flange, center=o + (ss.flange_z + 0.5 * ss.flange[2]) * s, R=Fr)
    sp0 = o + Hb * s + ss.shaft_offset * l
    shaft = cylinder(ss.shaft_r, sp0 - 1e-4 * s, sp0 + ss.shaft_len * s, n=20)
    mesh = union([case, fl, shaft])
    holes = [(o + (ss.flange_z + ss.flange[2]) * s + side * 0.5 * ss.hole_pitch * l, -s) for side in (-1, 1)]
    return {"mesh": mesh, "shaft_p": sp0 + ss.shaft_len * s, "shaft_base": sp0, "shaft_dir": s,
            "flange_holes": holes}


def servo_arm(shaft_top, shaft_dir, arm_dir, length: float, t: float = 0.003, w: float = 0.008) -> dict:
    """Flat arm from the output shaft along ``arm_dir`` (perpendicular to the shaft); ball-link hole at the tip."""
    s = unit(shaft_dir)
    d = unit(np.asarray(arm_dir, float) - np.dot(arm_dir, s) * s)
    prof = unary_union([Point(0, 0).buffer(w * 0.7, 32), Point(length, 0).buffer(w * 0.5, 32),
                        Polygon([(0, -w / 2), (length, -w * 0.4), (length, w * 0.4), (0, w / 2)])])
    prof = prof.difference(Point(length, 0).buffer(0.0016, 24))
    m = extrude(prof, t, origin=np.asarray(shaft_top, float) - 1e-4 * s, u=d, v=np.cross(s, d))
    return {"mesh": m, "tip": np.asarray(shaft_top, float) + length * d + 0.5 * t * s}


def control_horn(base_p, base_normal, hinge_dir, height: float, t: float = 0.003, base_l: float = 0.030,
                 hole_d: float = 0.0032) -> dict:
    """Plate horn standing on a surface at ``base_p`` (outward ``base_normal``), in the plane perpendicular to the
    hinge axis. Returns {"mesh", "hole": linkage hole centre, "bolts": [(pos, axis)]}."""
    nrm = unit(base_normal)
    a = unit(hinge_dir)
    c = unit(np.cross(a, nrm))                              # chordwise direction in the horn plane
    prof = Polygon([(-base_l / 2, 0), (base_l / 2, 0), (0.006, height), (-0.006, height)])
    prof = unary_union([prof, Point(0, height).buffer(0.006, 32)]).difference(Point(0, height).buffer(hole_d / 2, 24))
    m = extrude(prof, t, origin=np.asarray(base_p, float) - 0.5 * t * a, u=c, v=nrm)
    hole = np.asarray(base_p, float) + height * nrm
    bolts = [(np.asarray(base_p, float) + side * base_l * 0.3 * c, -nrm) for side in (-1, 1)]
    return {"mesh": m, "hole": hole, "bolts": bolts}


def pushrod(p0, p1, rod_d: float = 0.004, end_r: float = 0.005, end_len: float = 0.018) -> dict:
    """Rod with two rod-end bearings; ball centres at p0 and p1 (ball pivots on bolts through horn/arm holes)."""
    p0, p1 = np.asarray(p0, float), np.asarray(p1, float)
    d = unit(p1 - p0)
    L = float(np.linalg.norm(p1 - p0))
    if L < 2 * end_len + 0.005:
        raise ValueError(f"pushrod too short ({L:.3f} m)")
    rod = cylinder(rod_d / 2, p0 + (end_len - 0.002) * d, p1 - (end_len - 0.002) * d, n=16)
    ref = np.array([0, 0, 1.0]) if abs(d[2]) < 0.9 else np.array([1.0, 0, 0])
    side = unit(np.cross(d, ref))
    ends = []
    for c, sgn in ((p0, 1), (p1, -1)):
        eye = cylinder(end_r, c - 0.0025 * side, c + 0.0025 * side, n=24)
        neck = cylinder(rod_d * 0.6, c + sgn * end_r * 0.6 * d, c + sgn * end_len * d, n=16)
        ends.append(union([eye, neck]))
    return {"mesh": union([rod] + ends), "length": L, "pivot_axis": side}


def linkage_kinematics(hinge_p, hinge_dir, horn_hole, arm_center, arm_axis, arm_len: float, arm_angle0: float,
                       defl_range: tuple, n: int = 41) -> dict:
    """Planar-ish four-bar: control surface rotates about the hinge, the servo arm about its shaft; rod length fixed
    at the rest geometry. Returns servo angles (rad) needed over the deflection range and the minimum transmission
    angle (deg) between rod and horn arm; raises if the linkage toggles (no solution)."""
    hp, ha = np.asarray(hinge_p, float), unit(hinge_dir)
    sc, sa = np.asarray(arm_center, float), unit(arm_axis)
    ref = np.array([0, 0, 1.0]) if abs(sa[2]) < 0.9 else np.array([1.0, 0, 0])
    e1 = unit(np.cross(sa, ref))
    e2 = np.cross(sa, e1)

    def arm_tip(th):
        return sc + arm_len * (math.cos(th) * e1 + math.sin(th) * e2)

    rod_L = float(np.linalg.norm(arm_tip(arm_angle0) - np.asarray(horn_hole, float)))
    out_th, trans = [], []
    th = arm_angle0
    for dlt in np.linspace(defl_range[0], defl_range[1], n):
        hole = rotate_about(np.asarray(horn_hole, float)[None], hp, ha, dlt)[0]
        # solve |arm_tip(th) - hole| = rod_L by Newton from the previous angle
        for _ in range(60):
            f = np.linalg.norm(arm_tip(th) - hole) - rod_L
            df = (np.linalg.norm(arm_tip(th + 1e-6) - hole) - np.linalg.norm(arm_tip(th - 1e-6) - hole)) / 2e-6
            if abs(df) < 1e-12:
                raise ValueError("linkage toggles (zero derivative)")
            th -= f / df
            if abs(f) < 1e-9:
                break
        if abs(np.linalg.norm(arm_tip(th) - hole) - rod_L) > 1e-6:
            raise ValueError("linkage has no solution over the deflection range")
        rod = unit(hole - arm_tip(th))
        horn_arm = unit(hole - (hp + np.dot(hole - hp, ha) * ha))
        ang = math.degrees(math.acos(min(1.0, abs(float(np.dot(rod, horn_arm))))))
        trans.append(ang)                       # 90 deg ideal; acos(|rod . horn arm|) in [0, 90]
        out_th.append(th)
    out_th = np.array(out_th)
    return {"rod_length": rod_L, "servo_angles": out_th, "servo_travel_deg": float(np.degrees(np.ptp(out_th))),
            "min_transmission_deg": float(min(trans))}
