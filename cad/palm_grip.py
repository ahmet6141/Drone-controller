"""Avuç iniş tutamağı (docs/05 §3.1): tüp + sensör tablası + TPU tampon.

  * grip_tube         — üstte iç flanş (uçuş yığını M3 deseni), baklava hafifletme pencereleri
                        (45° kenarlı → desteksiz basılır), sensör tablası için 45° pahlı iç basamak.
  * grip_sensor_mount — aşağı kamera (CM3 Wide) ve VL53L8CX tablası; alt yüzeyi tutamak
                        tabanından SENSOR_RECESS (25 mm) içeride. Tüpe 3 radyal M2 ile bağlanır.
  * grip_bumper       — TPU tampon halkası; köpük ped bunun içine yapıştırılır.
Parça koordinatı: z = 0 tutamağın üst yüzeyi (gövde alt plakası), aşağı negatif.
"""
from __future__ import annotations

import math

import cadquery as cq

import params as P

LEDGE_W = 3.0
MOUNT_T = 2.0
SKIRT_H = 5.0
SKIRT_W = 1.2
BOSS = 6.0
LIP_H = 4.0
FIT = 0.15                       # geçme boşluğu (yarıçapta)
FLANGE_T = 2.5
FLANGE_RING_W = 7.0              # flanş: dış bant + 4 kol (M3 göbekleri)
LUG_W = 9.0


def radii() -> tuple[float, float]:
    r_out = P.GRIP_D / 2
    return r_out, r_out - P.GRIP_WALL


def total_drop() -> float:
    """Tutamağın gövde altından tabanına boyu."""
    return P.FRAME_BOTTOM_Z - P.GRIP_BOTTOM_Z


def tube_length() -> float:
    return total_drop() - P.BUMPER_H


def mount_bottom_z() -> float:
    """Sensör tablası alt yüzeyi (parça koordinatı)."""
    return -total_drop() + P.SENSOR_RECESS


def ledge_z() -> float:
    return mount_bottom_z() + MOUNT_T + SKIRT_H


def _revolve(points: list[tuple[float, float]]) -> cq.Workplane:
    return cq.Workplane("XZ").polyline(points).close().revolve(360.0, (0, 0, 0), (0, 1, 0))


def waist_z() -> tuple[float, float, float]:
    """Bel bölgesi (parça koordinatı): başlangıç, en ince nokta, bitiş — pencerelerin olduğu kısım."""
    return -8.0, -29.0, -50.0


def grip_tube() -> cq.Workplane:
    r_out, r_in = radii()
    L, zl = tube_length(), ledge_z()
    za, zm, zb = waist_z()
    w = P.GRIP_WAIST
    # Kesit: düz üst → içe kavisli bel (ele oturur) → düz alt; iç yüzey aynı kavisle (sabit duvar)
    tube = (cq.Workplane("XZ").moveTo(r_in, 0.0).lineTo(r_out, 0.0).lineTo(r_out, za)
            .threePointArc((r_out - w, zm), (r_out, zb)).lineTo(r_out, -L).lineTo(r_in, -L)
            .lineTo(r_in, zl).lineTo(r_in - LEDGE_W, zl).lineTo(r_in, zl + LEDGE_W)       # basamak + 45° pah
            .lineTo(r_in, zb).threePointArc((r_in - w, zm), (r_in, za)).close()
            .revolve(360.0, (0, 0, 0), (0, 1, 0)))
    # Üst flanş: iç bant + 4 kol; ortası açık (FFC ve ToF kabloları gövdeye çıkar)
    s = P.STACK / 2
    band_in = r_in - FLANGE_RING_W
    flange = (cq.Workplane("XY").workplane(offset=-FLANGE_T).circle(r_in + 0.1).circle(band_in)
              .extrude(FLANGE_T))
    lug_r0 = math.hypot(s, s) - 5.0
    for a in (45.0, 135.0, 225.0, 315.0):
        lug = (cq.Workplane("XY").workplane(offset=-FLANGE_T).center((lug_r0 + band_in + 1.0) / 2, 0)
               .rect(band_in + 1.0 - lug_r0, LUG_W).extrude(FLANGE_T).rotate((0, 0, 0), (0, 0, 1), a))
        boss = (cq.Workplane("XY").workplane(offset=-FLANGE_T).center(math.hypot(s, s), 0)
                .circle(LUG_W / 2).extrude(FLANGE_T).rotate((0, 0, 0), (0, 0, 1), a))
        flange = flange.union(lug).union(boss)
    holes = (cq.Workplane("XY").workplane(offset=-FLANGE_T)
             .pushPoints([(s, s), (s, -s), (-s, s), (-s, -s)]).circle(P.M3 / 2).extrude(FLANGE_T))
    tube = tube.union(flange.cut(holes))
    # Baklava pencereler: köşeleri yuvarlatılmış kare, 45° döndürülmüş → üst kenarlar 45° eğimli
    # (desteksiz basılır), yuvarlak köşeler çentik etkisini azaltır.
    side = P.GRIP_WINDOW / math.sqrt(2)
    window = cq.Sketch().rect(side, side).vertices().fillet(P.WINDOW_FILLET)
    depth = P.GRIP_WALL + P.GRIP_WAIST + 3.0
    rows = (-15.0, -28.0, -41.0)
    for i, zc in enumerate(rows):
        for k in range(10):
            ang = 36.0 * k + (18.0 if i % 2 else 0.0)
            cutter = (cq.Workplane("YZ").workplane(offset=r_in - P.GRIP_WAIST - 1.5).center(0, zc)
                      .placeSketch(window).extrude(depth)
                      .rotate((0, 0, zc), (1, 0, zc), 45.0)
                      .rotate((0, 0, 0), (0, 0, 1), ang))
            tube = tube.cut(cutter)
    # Sensör tablası vidaları: 3 radyal M2 (etek göbeklerine)
    z_screw = mount_bottom_z() + MOUNT_T + SKIRT_H / 2
    for a in (0.0, 120.0, 240.0):
        pin = (cq.Workplane("YZ").workplane(offset=r_in - 1.0).center(0, z_screw).circle(P.M2 / 2)
               .extrude(P.GRIP_WALL + 2.0).rotate((0, 0, 0), (0, 0, 1), a + 60.0))
        tube = tube.cut(pin)
    return tube


def camera_hole_points() -> list[tuple[float, float]]:
    """Kamera kartı M2 delikleri (mercek merkezine göre; Pi kamera deseni, kartla doğrulayın)."""
    w, h = P.CAM_HOLES
    top = P.CAM_BOARD[1] / 2 - P.CAM_HOLE_OFFSET
    return [(top, w / 2), (top, -w / 2), (top - h, w / 2), (top - h, -w / 2)]


TOF_Y = 19.0


def grip_sensor_mount() -> cq.Workplane:
    _, r_in = radii()
    r = r_in - FIT
    disc = cq.Workplane("XY").circle(r).extrude(MOUNT_T)
    skirt = cq.Workplane("XY").workplane(offset=MOUNT_T).circle(r).circle(r - SKIRT_W).extrude(SKIRT_H)
    part = disc.union(skirt)
    z_screw = MOUNT_T + SKIRT_H / 2
    for a in (60.0, 180.0, 300.0):
        boss = (cq.Workplane("XY").workplane(offset=MOUNT_T).center(r - BOSS / 2 - 0.5, 0).rect(BOSS, BOSS)
                .extrude(SKIRT_H).rotate((0, 0, 0), (0, 0, 1), a))
        pilot = (cq.Workplane("YZ").workplane(offset=r - BOSS - 1.0).center(0, z_screw).circle(P.M2_PILOT / 2)
                 .extrude(BOSS + 2.0).rotate((0, 0, 0), (0, 0, 1), a))
        part = part.union(boss).cut(pilot)
    light_r = (r + 17.5) / 2                      # kamera kartı (≤ 17 mm) ile etek arası
    lights = [(light_r * math.cos(math.radians(a)), light_r * math.sin(math.radians(a)))
              for a in (0, 30, 120, 150, 210, 240, 270, 330)]
    for cutter in (cq.Workplane("XY").circle(6.5).extrude(MOUNT_T),                  # mercek
                   cq.Workplane("XY").pushPoints(camera_hole_points()).circle(P.M2 / 2).extrude(MOUNT_T),
                   cq.Workplane("XY").center(0, TOF_Y).rect(*P.TOF_WINDOW).extrude(MOUNT_T),
                   cq.Workplane("XY").pushPoints(lights).circle(4.5).extrude(MOUNT_T)):
        part = part.cut(cutter)
    return part


def grip_bumper() -> cq.Workplane:
    """TPU tampon: avuca değen alt kenar tam yuvarlak (keskin kenar yok), üstte tüpe geçen dudak."""
    r_out, r_in = radii()
    r_mouth = r_out - P.BUMPER_WALL
    rho = P.BUMPER_ROUND
    return (cq.Workplane("XZ").moveTo(r_mouth, rho)
            .threePointArc(((r_mouth + r_out) / 2, rho - (r_out - r_mouth) / 2), (r_out, rho))
            .lineTo(r_out, P.BUMPER_H).lineTo(r_in - FIT, P.BUMPER_H).lineTo(r_in - FIT, P.BUMPER_H + LIP_H)
            .lineTo(r_mouth, P.BUMPER_H + LIP_H).close()
            .revolve(360.0, (0, 0, 0), (0, 1, 0)))


def parts() -> dict[str, tuple[cq.Workplane, str, int]]:
    return {"grip_tube": (grip_tube(), "PETG", 1),
            "grip_sensor_mount": (grip_sensor_mount(), "PETG", 1),
            "grip_bumper": (grip_bumper(), "TPU", 1)}


def placed(tube: cq.Workplane, mount: cq.Workplane, bumper: cq.Workplane) -> list[cq.Workplane]:
    z0 = P.FRAME_BOTTOM_Z
    return [tube.translate((0, 0, z0)),
            mount.translate((0, 0, z0 + mount_bottom_z())),
            bumper.translate((0, 0, P.GRIP_BOTTOM_Z))]
