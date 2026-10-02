"""Kendi tasarımımız 2 eksen fırçasız gimbal (docs/10 §3): pitch + roll, yaw gövdeyle.

Basılan parçalar:
  * gimbal_cradle   — kamera beşiği: CM3 kartı M2 ara parçalarla; yan plakada kızaklı delikler
                      (kamera ± CRADLE_SLOT kaydırılarak pitch ekseninde dengelenir).
  * gimbal_roll_arm — L kol: pitch motoru yan plakada, roll motoru arka plakada.
  * gimbal_top      — roll motoru taşıyıcı dik plaka + sönümleyici üst plakası.
  * gimbal_boom     — gövde alt plakasından öne uzanan kaburgalı taşıyıcı (gövde koordinatında).
Motorlar ve kamera yalnızca montaj, denge ve çarpışma kontrolü için temsilidir (basılmaz).

Gimbal koordinatı: orijin kamera merkezi (pitch ekseni ∩ optik eksen), x ileri, y sol, z yukarı.
Pitch ekseni y yönünde (x = z = 0); roll ekseni x yönünde, y = roll_axis_y() noktasından geçer:
dönen grubun (kamera + beşik + pitch motoru + kol) ağırlık merkezine yerleştirilir.
"""
from __future__ import annotations

import math

import cadquery as cq

import params as P

T = P.GIMBAL_PART_T
CRADLE_BACK_X = -6.0            # beşik arka plakası x ∈ [−6, −3]; kart M2 ara parçalarla x = −1'de
CRADLE_HALF = 14.0              # beşik yarı yüksekliği (z) ve kamera tarafı yarı genişliği (y)
CAM_CG_X = 1.5                  # kamera modülünün ağırlık merkezi (mercek öne kaydırır; tahmini)
ARM_HALF = 15.0
MOTOR_GAP = 0.5


def _box(x0, x1, y0, y1, z0, z1) -> cq.Workplane:
    return (cq.Workplane("XY").box(x1 - x0, y1 - y0, z1 - z0, centered=False).translate((x0, y0, z0)))


def _cyl_y(cx, cz, y0, y1, r) -> cq.Workplane:
    return cq.Workplane("XZ").workplane(offset=-y0).center(cx, cz).circle(r).extrude(-(y1 - y0))


def _cyl_x(cy, cz, x0, x1, r) -> cq.Workplane:
    return cq.Workplane("YZ").workplane(offset=x0).center(cy, cz).circle(r).extrude(x1 - x0)


def _circle_pts(d: float, offset_deg: float = 45.0) -> list[tuple[float, float]]:
    r = d / 2
    return [(r * math.cos(math.radians(offset_deg + 90 * k)), r * math.sin(math.radians(offset_deg + 90 * k)))
            for k in range(4)]


# --- y konumları (kamera tarafından dışa doğru)
def cradle_side_y() -> tuple[float, float]:
    y0 = P.CAM_BOARD[0] / 2 + 3.5
    return y0, y0 + T


def pitch_motor_y() -> tuple[float, float]:
    y0 = cradle_side_y()[1] + MOTOR_GAP
    return y0, y0 + P.GIMBAL_MOTOR_H


def arm_side_y() -> tuple[float, float]:
    y0 = pitch_motor_y()[1] + MOTOR_GAP
    return y0, y0 + T


# --- parçalar ----------------------------------------------------------------------------------
def camera() -> cq.Workplane:
    w, h, t = P.CAM_BOARD
    lw, lh, ld = P.CAM_LENS
    board = _box(-t, 0.0, -w / 2, w / 2, -h / 2, h / 2)
    lens = _box(0.0, ld, -lw / 2, lw / 2, -lh / 2, lh / 2)
    return board.union(lens)


def gimbal_cradle() -> cq.Workplane:
    w, h, t = P.CAM_BOARD
    ys0, ys1 = cradle_side_y()
    back = _box(CRADLE_BACK_X, CRADLE_BACK_X + T, -CRADLE_HALF, ys1, -CRADLE_HALF, CRADLE_HALF)
    side = _box(CRADLE_BACK_X, 12.0, ys0, ys1, -CRADLE_HALF, CRADLE_HALF)
    part = back.union(side)
    # Kart delikleri (y, z) — Pi kamera deseni, üst kenardan CAM_HOLE_OFFSET
    hw, hh = P.CAM_HOLES
    top = h / 2 - P.CAM_HOLE_OFFSET
    pts = [(hw / 2, top), (-hw / 2, top), (hw / 2, top - hh), (-hw / 2, top - hh)]
    standoff_x0 = CRADLE_BACK_X + T
    for y, z in pts:
        part = part.union(_cyl_x(y, z, standoff_x0, -t, 2.2))
        part = part.cut(_cyl_x(y, z, CRADLE_BACK_X - 1, 0.0, P.M2 / 2))
    # Kızaklı delikler: pitch motoru rotor deseni, x yönünde ± CRADLE_SLOT
    for x, z in _circle_pts(P.GIMBAL_MOTOR_HOLE_CIRCLE):
        slot = (cq.Workplane("XZ").workplane(offset=-ys0).center(x, z)
                .slot2D(2 * P.CRADLE_SLOT + P.M2, P.M2).extrude(-T))
        part = part.cut(slot)
    return part


def swept_radius(shape: cq.Workplane) -> float:
    """Pitch ekseni (y doğrusu) etrafında şeklin en uzak noktası (köşe yaklaşımı)."""
    bb = shape.val().BoundingBox()
    return max(math.hypot(x, z) for x in (bb.xmin, bb.xmax) for z in (bb.zmin, bb.zmax))


def arm_back_x() -> float:
    r = max(swept_radius(gimbal_cradle()), swept_radius(camera()))
    return -(r + P.GIMBAL_CLEARANCE)


def pitch_motor() -> cq.Workplane:
    y0, y1 = pitch_motor_y()
    return _cyl_y(0.0, 0.0, y0, y1, P.GIMBAL_MOTOR_D / 2)


def gimbal_roll_arm(y_roll: float) -> cq.Workplane:
    xb = arm_back_x()
    ya0, ya1 = arm_side_y()
    side = _box(xb - T, 14.0, ya0, ya1, -ARM_HALF, ARM_HALF)
    back = _box(xb - T, xb, y_roll - ARM_HALF - 2.0, ya1, -ARM_HALF, ARM_HALF)
    part = side.union(back)
    for x, z in _circle_pts(P.GIMBAL_MOTOR_HOLE_CIRCLE):          # pitch motoru statoru
        part = part.cut(cq.Workplane("XZ").workplane(offset=-ya0).center(x, z).circle(P.M2 / 2).extrude(-T))
    part = part.cut(cq.Workplane("XZ").workplane(offset=-ya0).circle(4.0).extrude(-T))
    for y, z in _circle_pts(P.GIMBAL_MOTOR_HOLE_CIRCLE):          # roll motoru rotoru
        part = part.cut(_cyl_x(y_roll + y, z, xb - T - 1, xb + 1, P.M2 / 2))
    return part.cut(_cyl_x(y_roll, 0.0, xb - T - 1, xb + 1, 4.0))


def roll_motor(y_roll: float) -> cq.Workplane:
    x1 = arm_back_x() - T - MOTOR_GAP
    return _cyl_x(y_roll, 0.0, x1 - P.GIMBAL_MOTOR_H, x1, P.GIMBAL_MOTOR_D / 2)


def roll_group(y_roll: float) -> cq.Workplane:
    return gimbal_roll_arm(y_roll).union(pitch_motor()).union(gimbal_cradle()).union(camera())


def roll_clearance_z(y_roll: float) -> float:
    """Roll ± sınırda dönen grubun ulaştığı en yüksek z (üst plaka bunun üstünde olmalı)."""
    group = roll_group(y_roll)
    top = -1e9
    for ang in (P.ROLL_RANGE[0], 0.0, P.ROLL_RANGE[1]):
        bb = group.rotate((0, y_roll, 0), (1, y_roll, 0), ang).val().BoundingBox()
        top = max(top, bb.zmax)
    return top


def top_plate_z(y_roll: float) -> float:
    """Üst plakanın alt yüzeyi."""
    return roll_clearance_z(y_roll) + P.GIMBAL_CLEARANCE


def roll_stator_x() -> float:
    """Roll motoru statorunu taşıyan dik plakanın ön yüzü."""
    return arm_back_x() - T - MOTOR_GAP - P.GIMBAL_MOTOR_H - MOTOR_GAP


def gimbal_top(y_roll: float) -> cq.Workplane:
    xm = roll_stator_x()
    z_top = top_plate_z(y_roll)
    half = P.GIMBAL_MOTOR_D / 2 + 1.5
    upright = _box(xm - T, xm, y_roll - half, y_roll + half, -half, z_top + T)
    cx, cy = damper_center(y_roll)
    dx, dy = P.DAMPER_SPACING
    plate = _box(xm - T, cx + dx / 2 + 5.0, cy - dy / 2 - 5.0, cy + dy / 2 + 5.0, z_top, z_top + T)
    part = upright.union(plate)
    for y, z in _circle_pts(P.GIMBAL_MOTOR_HOLE_CIRCLE):
        part = part.cut(_cyl_x(y_roll + y, z, xm - T - 1, xm + 1, P.M2 / 2))
    part = part.cut(_cyl_x(y_roll, 0.0, xm - T - 1, xm + 1, 4.0))
    holes = (cq.Workplane("XY").workplane(offset=z_top)
             .pushPoints([(cx + sx * dx / 2, cy + sy * dy / 2) for sx in (-1, 1) for sy in (-1, 1)])
             .circle(P.M3 / 2).extrude(T))
    window = cq.Workplane("XY").workplane(offset=z_top).center(cx, cy).circle(8.0).extrude(T)
    return part.cut(holes).cut(window)


def damper_center(y_roll: float) -> tuple[float, float]:
    """Sönümleyici deseninin merkezi: üst plaka, roll statoru plakasından öne uzanır."""
    return (roll_stator_x() - T + P.DAMPER_SPACING[0] / 2 + 5.0, y_roll)


def top_plate_top_z(y_roll: float) -> float:
    return top_plate_z(y_roll) + T


def gimbal_boom(y_roll: float) -> cq.Workplane:
    """Gövde koordinatında: alt plakaya 4 × M3, öne uzanan kaburgalı kol, ucunda sönümleyici deseni."""
    cx, cy = damper_center(y_roll)
    gx, gy, _ = P.GIMBAL_POS
    dcx, dcy = gx + cx, gy + cy
    dx, dy = P.DAMPER_SPACING
    x0, x1 = P.BOOM_X[0], dcx + dx / 2 + 6.0
    z1 = P.FRAME_BOTTOM_Z
    z0 = z1 - T
    width = dy + 12.0
    plate = _box(x0, x1, dcy - width / 2, dcy + width / 2, z0, z1)
    for sy in (-1, 1):
        yc = dcy + sy * (width / 2 - 2.5)
        plate = plate.union(_box(x0 + 15.0, x1 - 3.0, yc - 1.5, yc + 1.5, z1, z1 + 6.0))
    frame_holes = [(x0 + 5.0, dcy + sy * 10.0) for sy in (-1, 1)] + [(x0 + 12.0, dcy + sy * 10.0) for sy in (-1, 1)]
    damper_holes = [(dcx + sx * dx / 2, dcy + sy * dy / 2) for sx in (-1, 1) for sy in (-1, 1)]
    holes = (cq.Workplane("XY").workplane(offset=z0).pushPoints(frame_holes + damper_holes)
             .circle(P.M3 / 2).extrude(T))
    plate = plate.cut(holes)
    for wx in (dcx, x0 + 22.0):                                     # hafifletme + kablo geçişi
        plate = plate.cut(cq.Workplane("XY").workplane(offset=z0).center(wx, dcy).circle(7.0).extrude(T))
    return plate


# --- denge ve kontroller ---------------------------------------------------------------------
def _mass(shape: cq.Workplane, material: str) -> tuple[float, tuple[float, float, float]]:
    solid = shape.val()
    c = cq.Shape.centerOfMass(solid)
    return solid.Volume() / 1000.0 * P.DENSITY[material], (c.x, c.y, c.z)


def roll_axis_y(material: str = "PA-CF", iterations: int = 4) -> float:
    """Roll eksenini dönen grubun ağırlık merkezine koyar (y), birkaç yinelemeyle."""
    y = 10.0
    for _ in range(iterations):
        items = [(P.CAM_MASS_G, (CAM_CG_X, 0.0, 0.0)), _mass(gimbal_cradle(), material),
                 (P.GIMBAL_MOTOR_MASS_G, (0.0, sum(pitch_motor_y()) / 2, 0.0)),
                 _mass(gimbal_roll_arm(y), material)]
        y = sum(m * c[1] for m, c in items) / sum(m for m, _ in items)
    return y


def pitch_balance(material: str = "PA-CF") -> tuple[float, float]:
    """Pitch ekseni etrafında dengesizlik: (kamera + beşik) ağırlık merkezinin x, z kayması (mm)."""
    items = [(P.CAM_MASS_G, (CAM_CG_X, 0.0, 0.0)), _mass(gimbal_cradle(), material)]
    m = sum(mi for mi, _ in items)
    return (sum(mi * c[0] for mi, c in items) / m, sum(mi * c[2] for mi, c in items) / m)


def interference(y_roll: float, step: float = 10.0) -> dict[str, float]:
    """Pitch ve roll hareket aralığında çakışan hacim (mm³); 0 olmalı."""
    def overlap(a: cq.Workplane, b: cq.Workplane) -> float:
        try:
            return sum(s.Volume() for s in a.intersect(b).solids().vals())
        except Exception:                                          # boş kesişim
            return 0.0

    arm, top = gimbal_roll_arm(y_roll), gimbal_top(y_roll)
    moving = gimbal_cradle().union(camera())
    worst_pitch = 0.0
    n = int(round((P.PITCH_RANGE[1] - P.PITCH_RANGE[0]) / step))
    for i in range(n + 1):
        pitch = P.PITCH_RANGE[0] + i * step
        rotated = moving.rotate((0, 0, 0), (0, 1, 0), -pitch)    # +pitch = burun yukarı
        worst_pitch = max(worst_pitch, overlap(rotated, arm), overlap(rotated, top))
    group = roll_group(y_roll)
    worst_roll = 0.0
    for ang in (P.ROLL_RANGE[0], P.ROLL_RANGE[0] / 2, P.ROLL_RANGE[1] / 2, P.ROLL_RANGE[1]):
        worst_roll = max(worst_roll, overlap(group.rotate((0, y_roll, 0), (1, y_roll, 0), ang), top))
    return {"pitch": worst_pitch, "roll": worst_roll}


def stack_fits(y_roll: float) -> tuple[bool, float]:
    """Kamera merkezi → üst plaka + sönümleyici + taşıyıcı kol, gövde alt plakasına sığıyor mu?"""
    needed = top_plate_top_z(y_roll) + P.DAMPER_H + T          # gimbal koordinatında
    available = P.FRAME_BOTTOM_Z - P.GIMBAL_POS[2]
    return needed <= available + 1e-6, available - needed


def parts(y_roll: float | None = None) -> dict[str, tuple[cq.Workplane, str, int]]:
    y = roll_axis_y() if y_roll is None else y_roll
    return {"gimbal_cradle": (gimbal_cradle(), "PA-CF", 1),
            "gimbal_roll_arm": (gimbal_roll_arm(y), "PA-CF", 1),
            "gimbal_top": (gimbal_top(y), "PA-CF", 1),
            "gimbal_boom": (gimbal_boom(y), "PA-CF", 1)}


def placed(y_roll: float) -> list[tuple[str, cq.Workplane]]:
    """Gövde koordinatında montaj (temsili motorlar ve kamera dahil); boom zaten gövde koordinatında."""
    g = P.GIMBAL_POS
    local = [("gimbal_cradle", gimbal_cradle()), ("camera", camera()), ("pitch_motor", pitch_motor()),
             ("gimbal_roll_arm", gimbal_roll_arm(y_roll)), ("roll_motor", roll_motor(y_roll)),
             ("gimbal_top", gimbal_top(y_roll))]
    return [(n, s.translate(g)) for n, s in local] + [("gimbal_boom", gimbal_boom(y_roll))]
