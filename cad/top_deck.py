"""Üst katlar: companion tepsisi (Pi 5 + AI HAT+) ve batarya plakası.

  * companion_tray — uçuş yığınının üstüne 30,5 mm M3 deseniyle; Pi 5 M2.5 delikleri (USB/Ethernet
                     arkaya bakar), FFC geçişleri, köşelerde batarya plakası ara parçaları.
  * battery_plate  — ara parçalarla tepsinin üstünde; batarya kayış yuvaları, ağırlık merkezini
                     ortalayan batarya konumuna göre (cad/layout.py), önde yukarı bakan VL53L1X.
Parça koordinatı: z = 0 plakanın alt yüzeyi; x ileri, y sol (gövde ile aynı yön).
"""
from __future__ import annotations

import math

import cadquery as cq

import layout
import params as P

KEEP_OUT = 3.5           # hafifletme delikleri ile diğer delikler/yuvalar arasındaki en az et kalınlığı


def _plate(size: tuple[float, float, float]) -> cq.Workplane:
    lx, ly, t = size
    return cq.Workplane("XY").box(lx, ly, t, centered=(True, True, False)).edges("|Z").fillet(6.0)


def _posts() -> list[tuple[float, float]]:
    px, py = P.TRAY_POSTS
    return [(sx * px / 2, sy * py / 2) for sx in (-1, 1) for sy in (-1, 1)]


def pi5_holes() -> list[tuple[float, float]]:
    hx, hy = P.PI5_HOLES
    cx = P.PI5_X + P.PI5_HOLE_OFFSET            # port olmayan uç önde → desen merkezi öne kayar
    return [(cx + sx * hx / 2, sy * hy / 2) for sx in (-1, 1) for sy in (-1, 1)]


def _lighten(part: cq.Workplane, size, keep: list[tuple[float, float, float]]) -> cq.Workplane:
    """Izgara hafifletme delikleri; keep = [(x, y, yarıçap)] korunacak bölgeler."""
    lx, ly, t = size
    r = P.LIGHTEN_HOLE / 2
    pts = []
    nx, ny = int(lx // P.LIGHTEN_PITCH), int(ly // P.LIGHTEN_PITCH)
    for i in range(-nx, nx + 1):
        for j in range(-ny, ny + 1):
            x, y = i * P.LIGHTEN_PITCH, j * P.LIGHTEN_PITCH + (P.LIGHTEN_PITCH / 2 if i % 2 else 0.0)
            if abs(x) + r + 5.0 > lx / 2 or abs(y) + r + 5.0 > ly / 2:
                continue
            if any(math.hypot(x - kx, y - ky) < r + kr + KEEP_OUT for kx, ky, kr in keep):
                continue
            pts.append((x, y))
    if pts:
        part = part.cut(cq.Workplane("XY").pushPoints(pts).circle(r).extrude(t))
    return part


def companion_tray() -> cq.Workplane:
    size = P.TRAY
    t = size[2]
    part = _plate(size)
    s = P.STACK / 2
    stack = [(sx * s, sy * s) for sx in (-1, 1) for sy in (-1, 1)]
    slots = [(P.TRAY[0] / 2 - 10.0, 0.0), (0.0, P.TRAY[1] / 2 - 9.0)]   # gimbal FFC (ön), aşağı kamera FFC (yan)
    part = part.cut(cq.Workplane("XY").pushPoints(stack + _posts()).circle(P.M3 / 2).extrude(t))
    part = part.cut(cq.Workplane("XY").pushPoints(pi5_holes()).circle(P.M2_5 / 2).extrude(t))
    part = part.cut(cq.Workplane("XY").center(*slots[0]).rect(*P.FFC_SLOT[::-1]).extrude(t))
    part = part.cut(cq.Workplane("XY").center(*slots[1]).rect(*P.FFC_SLOT).extrude(t))
    keep = ([(x, y, P.M3) for x, y in stack + _posts()] + [(x, y, P.M2_5) for x, y in pi5_holes()]
            + [(x, y, P.FFC_SLOT[0] / 2) for x, y in slots])
    return _lighten(part, size, keep)


def strap_slots(battery_x: float) -> list[tuple[float, float]]:
    y = P.BATTERY_SIZE[1] / 2 + P.STRAP_SLOT[1] / 2 + 1.0
    return [(battery_x + sx * P.STRAP_SPACING / 2, sy * y) for sx in (-1, 1) for sy in (-1, 1)]


def battery_plate(battery_x: float | None = None) -> cq.Workplane:
    bx = layout.battery_position()[0] if battery_x is None else battery_x
    size = P.BATTERY_PLATE
    t = size[2]
    part = _plate(size)
    part = part.cut(cq.Workplane("XY").pushPoints(_posts()).circle(P.M3 / 2).extrude(t))
    for x, y in strap_slots(bx):
        part = part.cut(cq.Workplane("XY").center(x, y).slot2D(P.STRAP_SLOT[0], P.STRAP_SLOT[1]).extrude(t))
    tx, ty = P.OVERHEAD_TOF_POS
    tof = [(tx, ty + 6.35), (tx, ty - 6.35)]                       # VL53L1X kartı (tahmini desen)
    part = part.cut(cq.Workplane("XY").pushPoints(tof).circle(P.M2 / 2).extrude(t))
    keep = ([(x, y, P.M3) for x, y in _posts()] + [(x, y, P.STRAP_SLOT[0] / 2 + 1.0) for x, y in strap_slots(bx)]
            + [(x, y, P.M2) for x, y in tof])
    return _lighten(part, size, keep)


def shell_height() -> float:
    return P.BATTERY_PLATE_Z - (P.TRAY_Z + P.TRAY[2])


def companion_shell() -> cq.Workplane:
    """İsteğe bağlı kabuk: tepsi ile batarya plakası arasında yuvarlak köşeli bant.

    Elektroniği pervane akışından, tozdan ve parmaklardan korur; yanlarda havalandırma yarıkları
    (Active Cooler), arkada USB/Ethernet, önde FFC açıklığı. Dik duvarlar → desteksiz basılır.
    """
    h = shell_height()
    lx, ly = P.TRAY[0] - 2 * P.SHELL_INSET, P.TRAY[1] - 2 * P.SHELL_INSET
    w = P.SHELL_WALL
    outer = cq.Sketch().rect(lx, ly).vertices().fillet(6.0 - P.SHELL_INSET)
    inner = cq.Sketch().rect(lx - 2 * w, ly - 2 * w).vertices().fillet(6.0 - P.SHELL_INSET - w)
    shell = (cq.Workplane("XY").placeSketch(outer).extrude(h)
             .cut(cq.Workplane("XY").placeSketch(inner).extrude(h)))
    # Arka: Pi 5 USB/Ethernet erişimi; ön: gimbal FFC ve sıcak hava çıkışı
    shell = shell.cut(cq.Workplane("YZ").workplane(offset=-lx / 2 - 1).center(0, h / 2)
                      .placeSketch(cq.Sketch().rect(48.0, h - 10.0).vertices().fillet(4.0)).extrude(w + 2))
    shell = shell.cut(cq.Workplane("YZ").workplane(offset=lx / 2 - w - 1).center(0, h / 2)
                      .placeSketch(cq.Sketch().rect(30.0, 12.0).vertices().fillet(3.0)).extrude(w + 2))
    # Yan havalandırma yarıkları (Pi üzerinde)
    vents = [(P.PI5_X - 24.0 + 8.0 * k, h / 2) for k in range(7)]
    for side in (-1, 1):
        y0 = side * (ly / 2 + 1) if side < 0 else ly / 2 - w - 1
        for vx, vz in vents:
            shell = shell.cut(cq.Workplane("XZ").workplane(offset=-y0).center(vx, vz)
                              .slot2D(h - 14.0, 3.0, angle=90).extrude(-(w + 2)))
    return shell


def parts() -> dict[str, tuple[cq.Workplane, str, int]]:
    return {"companion_tray": (companion_tray(), "PA-CF", 1), "battery_plate": (battery_plate(), "PA-CF", 1),
            "companion_shell": (companion_shell(), "PA-CF", 1)}


def placed(tray: cq.Workplane, plate: cq.Workplane, shell: cq.Workplane | None = None) -> list[cq.Workplane]:
    out = [tray.translate((0, 0, P.TRAY_Z)), plate.translate((0, 0, P.BATTERY_PLATE_Z))]
    if shell is not None:
        out.append(shell.translate((0, 0, P.TRAY_Z + P.TRAY[2])))
    return out
