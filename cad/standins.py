"""Satın alınan parçaların görsel temsilleri (basılmaz): render ve montaj kontrolü için.

Gövde (MOZ7 benzeri), 2807 motor, 7 inç 3 kanatlı pervane, Pi 5 + AI HAT+, 6S1P batarya ve kayışları,
GNSS direği, MTF-01, kameralar, ELRS antenleri, gimbal sönümleyicileri. Ölçüler yaklaşıktır; gerçek
parçaların STEP modelleri (üretici siteleri) bulunursa bunların yerine konabilir.
Tüm fonksiyonlar gövde koordinatında (x ileri, y sol, z yukarı; mm) [(ad, katı)] döndürür.
"""
from __future__ import annotations

import math

import cadquery as cq

import gimbal_2axis as G
import layout
import palm_grip
import params as P

Item = tuple[str, cq.Workplane]


def _box(x0, x1, y0, y1, z0, z1) -> cq.Workplane:
    return cq.Workplane("XY").box(x1 - x0, y1 - y0, z1 - z0, centered=False).translate((x0, y0, z0))


def _cyl(x, y, z0, z1, r) -> cq.Workplane:
    return cq.Workplane("XY").workplane(offset=z0).center(x, y).circle(r).extrude(z1 - z0)


def _revolve(points: list[tuple[float, float]]) -> cq.Workplane:
    return cq.Workplane("XZ").polyline(points).close().revolve(360.0, (0, 0, 0), (0, 1, 0))


def _rounded(lx: float, ly: float, r: float) -> cq.Sketch:
    return cq.Sketch().rect(lx, ly).vertices().fillet(r)


# --- gövde ------------------------------------------------------------------------------------
def frame() -> list[Item]:
    items: list[Item] = []
    zb = P.FRAME_BOTTOM_Z
    bottom = (cq.Workplane("XY").workplane(offset=zb).center(0, 0)
              .placeSketch(_rounded(2 * P.FRAME_FRONT_X, 60.0, 10.0)).extrude(2.0))
    bottom = bottom.cut(_cyl(0, 0, zb, zb + 2, 12.0))
    for sy in (-1, 1):
        bottom = bottom.cut(cq.Workplane("XY").workplane(offset=zb).center(32.0, sy * 17.0)
                            .slot2D(26.0, 9.0).extrude(2.0))
        bottom = bottom.cut(cq.Workplane("XY").workplane(offset=zb).center(-32.0, sy * 17.0)
                            .slot2D(26.0, 9.0).extrude(2.0))
    items.append(("frame_bottom", bottom))
    zt = P.FRAME_TOP_Z - 2.0
    top = (cq.Workplane("XY").workplane(offset=zt).placeSketch(_rounded(100.0, 54.0, 8.0)).extrude(2.0))
    for sx in (-1, 1):
        top = top.cut(cq.Workplane("XY").workplane(offset=zt).center(sx * 28.0, 0).slot2D(30.0, 12.0).extrude(2.0))
    items.append(("frame_top", top))
    for x, y in P.motor_positions():
        heading = math.degrees(math.atan2(y, x))
        root, tip = 18.0, P.MOTOR_R
        arm = (cq.Workplane("XY").workplane(offset=-P.ARM_T)
               .polyline([(root, -11.0), (tip, -8.0), (tip, 8.0), (root, 11.0)]).close().extrude(P.ARM_T)
               .union(cq.Workplane("XY").workplane(offset=-P.ARM_T).center(tip, 0).circle(12.0).extrude(P.ARM_T))
               .cut(cq.Workplane("XY").workplane(offset=-P.ARM_T).center(92.0, 0).slot2D(56.0, 7.0).extrude(P.ARM_T)))
        items.append(("frame_arm", arm.rotate((0, 0, 0), (0, 0, 1), heading)))
    for sx in (-1, 1):
        for sy in (-1, 1):
            items.append(("frame_standoff", _cyl(sx * 44.0, sy * 21.0, 0.0, zt, 2.5)))
    for z, name in ((4.0, "frame_stack_pcb"), (12.0, "frame_stack_pcb")):
        items.append((name, _box(-18, 18, -18, 18, z, z + 1.6)))
    items.append(("frame_stack", _box(-6, 6, -6, 6, 13.6, 15.6)))
    return items


# --- itki -------------------------------------------------------------------------------------
def motor(x: float, y: float) -> list[Item]:
    z0 = P.HUB_T
    r = P.MOTOR_BELL_D / 2
    base = _revolve([(0, 0), (r, 0), (r, 2.5), (r - 1.5, 3.2), (0, 3.2)]).translate((x, y, z0))
    winding = _revolve([(0, 3.2), (r - 1.2, 3.2), (r - 1.2, 4.6), (0, 4.6)]).translate((x, y, z0))
    bell = _revolve([(4.0, 4.6), (r, 4.6), (r, 20.5), (r - 2.0, 22.0), (4.0, 22.0)]).translate((x, y, z0))
    for k in range(6):                                                   # çan üstü havalandırma
        a = 60.0 * k + 30.0
        slot = (cq.Workplane("XY").workplane(offset=z0 + 20.0).center(9.5 * math.cos(math.radians(a)),
                                                                        9.5 * math.sin(math.radians(a)))
                .slot2D(7.0, 3.0, angle=a + 90.0).extrude(2.5).translate((x, y, 0)))
        bell = bell.cut(slot)
    hub_top = P.PROP_PLANE_Z + P.PROP_HUB[1] / 2
    shaft = _cyl(x, y, z0 + 22.0, hub_top + 5.5, 2.5)
    nut = (cq.Workplane("XY").workplane(offset=hub_top).center(x, y).polygon(6, 9.2).extrude(5.0)
           .cut(_cyl(x, y, hub_top, hub_top + 5.0, 2.6)))
    return [("motor_base", base), ("motor_winding", winding), ("motor_bell", bell), ("motor_shaft", shaft),
            ("motor_nut", nut)]


def _blade(mirror: bool) -> cq.Workplane:
    wp = cq.Workplane("YZ")
    prev = 0.0
    for i, (r, chord, th) in enumerate(P.PROP_STATIONS):
        ang = math.degrees(math.atan(P.PROP_PITCH / (2 * math.pi * r)))
        wp = wp.workplane(offset=r - prev) if i else wp.workplane(offset=r - 1.5)
        wp = wp.ellipse(chord / 2, th / 2, rotation_angle=ang)
        prev = r if i else r - 1.5
    blade = wp.loft(ruled=False, combine=True)
    return blade.mirror("XZ") if mirror else blade


def prop(x: float, y: float, phase: float, ccw: bool) -> list[Item]:
    hub_r, hub_h = P.PROP_HUB
    body = (cq.Workplane("XY").workplane(offset=-hub_h / 2).circle(hub_r).circle(2.6).extrude(hub_h))
    blade = _blade(ccw)
    for k in range(3):
        body = body.union(blade.rotate((0, 0, 0), (0, 0, 1), phase + 120.0 * k))
    return [("prop", body.translate((x, y, P.PROP_PLANE_Z)))]


def propulsion() -> list[Item]:
    items: list[Item] = []
    for i, (x, y) in enumerate(P.motor_positions()):
        items += motor(x, y)
        items += prop(x, y, phase=17.0 + 41.0 * i, ccw=bool(i % 2))
    return items


# --- elektronik ---------------------------------------------------------------------------------
def companion() -> list[Item]:
    """Raspberry Pi 5 + Active Cooler + AI HAT+ (v1: tepsi üstünde, USB/Ethernet arkaya bakar)."""
    return companion_at(P.PI5_X, P.TRAY_Z + P.TRAY[2] + 6.0)


def companion_at(cx: float, z0: float, yaw_deg: float = 0.0, pivot: tuple[float, float] | None = None) -> list[Item]:
    """Pi 5 yığını: kart merkezi x = cx, kart alt yüzeyi z0; yaw_deg ile (pivot etrafında) döndürülür."""
    items = _companion_items(cx, z0)
    if yaw_deg:
        px, py = pivot if pivot else (cx, 0.0)
        items = [(n, s.rotate((px, py, 0), (px, py, 1), yaw_deg)) for n, s in items]
    return items


def _companion_items(cx: float, z0: float) -> list[Item]:
    lx, ly = P.PI5_BOARD
    x_rear, x_front = cx - lx / 2, cx + lx / 2
    items: list[Item] = [("pi_pcb", _box(x_rear, x_front, -ly / 2, ly / 2, z0, z0 + 1.6))]
    zt = z0 + 1.6
    items += [("pi_port", _box(x_rear - 2.0, x_rear + 17.0, -26.0, -8.5, zt, zt + 13.5)),      # Ethernet
              ("pi_port", _box(x_rear - 2.0, x_rear + 15.5, -6.0, 7.5, zt, zt + 16.0)),       # USB 3
              ("pi_port", _box(x_rear - 2.0, x_rear + 15.5, 10.0, 23.5, zt, zt + 16.0))]      # USB 2
    hx = cx + P.PI5_HOLE_OFFSET
    items.append(("pi_header", _box(hx - 25.5, hx + 25.5, ly / 2 - 6.0, ly / 2 - 1.0, zt, zt + 8.5)))
    items.append(("pi_cooler", _box(cx - 6.0, cx + 26.0, -20.0, 12.0, zt, zt + 7.0)))
    items.append(("pi_chip", _cyl(cx + 10.0, -4.0, zt + 7.0, zt + 8.0, 13.0)))
    z_hat = zt + 16.0
    for x, y in [(hx + sx * P.PI5_HOLES[0] / 2, sy * P.PI5_HOLES[1] / 2) for sx in (-1, 1) for sy in (-1, 1)]:
        items.append(("pi_standoff", _cyl(x, y, zt, z_hat, 2.2)))
    items.append(("hat_pcb", _box(hx - 32.5, hx + 32.5, -28.25, 28.25, z_hat, z_hat + 1.6)))
    items.append(("pi_chip", _box(hx - 7.5, hx + 7.5, -7.5, 7.5, z_hat + 1.6, z_hat + 3.4)))
    return items


def battery() -> list[Item]:
    bx, by, bz = layout.battery_position()
    sx, sy, sz = P.BATTERY_SIZE
    body = cq.Workplane("XY").box(sx, sy, sz).edges().fillet(5.0).translate((bx, by, bz))
    items: list[Item] = [("battery", body)]
    zb = P.BATTERY_PLATE_Z
    for k in (-1, 1):                                                    # kayışlar plakadaki yuvalardan geçer
        xs = bx + k * P.STRAP_SPACING / 2
        loop = (cq.Workplane("YZ").workplane(offset=xs - 10.0)
                .center(by, (zb + bz + sz / 2) / 2 + 0.6)
                .placeSketch(_rounded(sy + 2.4, bz + sz / 2 - zb + 1.2, 3.0)).extrude(20.0)
                .cut(cq.Workplane("YZ").workplane(offset=xs - 10.0).center(by, (zb + bz + sz / 2) / 2)
                     .placeSketch(_rounded(sy, bz + sz / 2 - zb, 2.0)).extrude(20.0)))
        items.append(("battery_strap", loop))
    xr = bx - sx / 2
    items.append(("xt60", _box(xr - 16.0, xr - 8.0, -8.0, 8.0, bz + sz / 2 - 12.0, bz + sz / 2 - 4.0)))
    for name, dy in (("wire_red", 3.5), ("wire_black", -3.5)):
        wire = (cq.Workplane("YZ").workplane(offset=xr - 8.0).center(by + dy, bz + sz / 2 - 8.0)
                .circle(1.8).extrude(9.0))
        items.append((name, wire))
    return items


def gnss() -> list[Item]:
    gx, gy, gz = P.GNSS_POS
    mast = _cyl(gx, gy, P.FRAME_TOP_Z, gz - 7.0, 4.0)
    puck = _revolve([(0, 0), (23.0, 0), (23.0, 6.0), (19.0, 11.0), (10.0, 14.0), (0, 14.0)]).translate((gx, gy, gz - 7.0))
    return [("gnss_mast", mast), ("gnss", puck)]


def flow_sensor() -> list[Item]:
    fx, fy, fz = layout.PLACEMENT["MTF-01"]
    return [("flow_pcb", _box(fx - 15.0, fx + 15.0, fy - 7.0, fy + 7.0, fz, fz + 1.2)),
            ("flow_sensor", _cyl(fx - 5.0, fy, fz - 2.0, fz, 3.0)),
            ("flow_sensor", _box(fx + 4.0, fx + 8.0, fy - 2.0, fy + 2.0, fz - 1.5, fz))]


def antennas() -> list[Item]:
    items: list[Item] = []
    for sy in (-1, 1):
        rod = (cq.Workplane("XY").circle(1.3).extrude(45.0)
               .rotate((0, 0, 0), (1, 0, 0), sy * 135.0)
               .translate((-48.0, sy * 22.0, P.FRAME_BOTTOM_Z)))
        items.append(("antenna", rod))
    return items


def down_camera() -> list[Item]:
    """Tutamak içindeki aşağı kamera (CM3 Wide) ve VL53L8CX: sensör tablasının üstünde, mercek aşağı."""
    z = P.FRAME_BOTTOM_Z + palm_grip.mount_bottom_z() + palm_grip.MOUNT_T
    w, h, t = P.CAM_BOARD
    return [("camera_pcb", _box(-h / 2, h / 2, -w / 2, w / 2, z, z + t)),
            ("camera_lens", _cyl(0, 0, z - palm_grip.MOUNT_T + 0.2, z, 6.0)),
            ("camera_glass", _cyl(0, 0, z - palm_grip.MOUNT_T, z - palm_grip.MOUNT_T + 0.2, 4.0)),
            ("tof_pcb", _box(-6.5, 6.5, palm_grip.TOF_Y - 6.5, palm_grip.TOF_Y + 6.5, z, z + 1.0)),
            ("tof_sensor", _box(-3.2, 3.2, palm_grip.TOF_Y - 1.5, palm_grip.TOF_Y + 1.5, z - 1.0, z))]


def gimbal_details(y_roll: float) -> list[Item]:
    """Gimbal motorları, kamera ayrıntısı ve sönümleyiciler (basılan parçalar build.py'de)."""
    g = P.GIMBAL_POS
    w, h, t = P.CAM_BOARD
    lw, lh, ld = P.CAM_LENS
    cam = [("camera_pcb", G._box(-t, 0.0, -w / 2, w / 2, -h / 2, h / 2)),
           ("camera_lens", G._box(0.0, ld - 2.0, -lw / 2, lw / 2, -lh / 2, lh / 2)),
           ("camera_lens", G._cyl_x(0.0, 0.0, ld - 2.0, ld, 3.6)),
           ("camera_glass", G._cyl_x(0.0, 0.0, ld, ld + 0.3, 2.6))]
    motors = [("gimbal_motor", G.pitch_motor()), ("gimbal_motor", G.roll_motor(y_roll))]
    items = [(n, s.translate(g)) for n, s in cam + motors]
    cx, cy = G.damper_center(y_roll)
    dx, dy = P.DAMPER_SPACING
    z_top = g[2] + G.top_plate_top_z(y_roll)
    z_boom = P.FRAME_BOTTOM_Z - P.GIMBAL_PART_T
    zc = (z_top + z_boom) / 2
    rad = (z_boom - z_top) / 2 + 0.3
    for sx in (-1, 1):
        for sy in (-1, 1):
            ball = cq.Workplane("XY").sphere(rad).translate((g[0] + cx + sx * dx / 2, g[1] + cy + sy * dy / 2, zc))
            items.append(("damper", ball))
    return items


def all_items(y_roll: float) -> list[Item]:
    return (frame() + propulsion() + companion() + battery() + gnss() + flow_sensor() + antennas()
            + down_camera() + gimbal_details(y_roll))
