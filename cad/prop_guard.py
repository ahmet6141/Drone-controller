"""Pervane koruması (motor başına 1 takım, ×4 aynı parça).

İki parça, ikisi de desteksiz basılır:
  * guard_ring  — halka + alt ağ + göbek + takviye kolları; ağ yüzeyi baskı tablasında.
  * guard_mount — motorla kol arasına giren plaka + 4 direk; direklerin ucuna halka göbeği M2 ile vidalanır.
Parça koordinatları: guard_ring'de z = 0 ağın alt yüzeyi; guard_mount'ta z = 0 kolun üst yüzeyi.
"""
from __future__ import annotations

import math

import cadquery as cq

import params as P


def collar_radii() -> tuple[float, float]:
    r_in = P.MOTOR_BELL_D / 2 + P.COLLAR_GAP
    return r_in, r_in + P.COLLAR_W


def post_radius() -> float:
    return sum(collar_radii()) / 2


def post_points() -> list[tuple[float, float]]:
    """Direkler yerel 45/135/225/315°'de; yerel −x kol yönüdür → motor kabloları direklerin arasından geçer."""
    r = post_radius()
    return [(r * math.cos(math.radians(45 + 90 * k)), r * math.sin(math.radians(45 + 90 * k)))
            for k in range(P.POSTS)]


def mesh_pitch() -> float:
    return P.MESH_OPENING + P.MESH_RIB


def ring_profile() -> cq.Workplane:
    """Halka kesiti (r, z): düz duvar + üstte dışa açılan çan ağzı + yuvarlatılmış kenar.

    Çan ağzı halkanın üst kenarını flanş gibi rijitleştirir, parmağa keskin kenar bırakmaz ve
    40° altı eğimle desteksiz basılır.
    """
    r_in, r_out = P.guard_inner_r(), P.guard_outer_r()
    h = P.GUARD_BELOW + P.GUARD_ABOVE
    f, d, t = P.GUARD_FLARE_H, P.GUARD_FLARE_OUT, P.GUARD_WALL
    return (cq.Workplane("XZ").moveTo(r_in, 0.0).lineTo(r_out, 0.0).lineTo(r_out, h - f)
            .lineTo(r_out + d, h).threePointArc((r_in + d + t / 2, h + t / 2), (r_in + d, h))
            .lineTo(r_in, h - f).close())


def guard_ring() -> cq.Workplane:
    r_in, r_out = P.guard_inner_r(), P.guard_outer_r()
    c_in, c_out = collar_radii()
    height = P.GUARD_BELOW + P.GUARD_ABOVE
    ring = ring_profile().revolve(360.0, (0, 0, 0), (0, 1, 0))

    pitch = mesh_pitch()
    n = int(r_out // pitch) + 1
    offsets = [k * pitch for k in range(-n, n + 1)]
    bars_x = cq.Workplane("XY").pushPoints([(0, o) for o in offsets]).rect(2 * r_out, P.MESH_RIB).extrude(P.MESH_T)
    bars_y = cq.Workplane("XY").pushPoints([(o, 0) for o in offsets]).rect(P.MESH_RIB, 2 * r_out).extrude(P.MESH_T)
    annulus = cq.Workplane("XY").circle(r_in + 0.5).circle(c_out - 0.5).extrude(P.MESH_T)
    if P.MESH_SECTOR_DEG < 360.0:
        # Yalnızca gövdeye bakan sektör: montajda −x yönü gövde merkezine çevrilir (placed()).
        half = math.radians(P.MESH_SECTOR_DEG / 2)
        pts = [(0.0, 0.0)] + [(-2 * r_out * math.cos(a), 2 * r_out * math.sin(a))
                              for a in (-half + k * half / 8 for k in range(17))]
        annulus = annulus.intersect(cq.Workplane("XY").polyline(pts).close().extrude(P.MESH_T))
    mesh = bars_x.union(bars_y).intersect(annulus)

    collar = cq.Workplane("XY").circle(c_out).circle(c_in).extrude(P.SPOKE_T)
    part = ring.union(mesh).union(collar)
    span = r_in - c_out + 1.0
    for k in range(P.SPOKES):
        spoke = (cq.Workplane("XY").center(c_out + span / 2 - 0.5, 0).rect(span, P.SPOKE_W).extrude(P.SPOKE_T)
                 .rotate((0, 0, 0), (0, 0, 1), 360.0 / P.SPOKES * k))
        part = part.union(spoke)
    holes = cq.Workplane("XY").pushPoints(post_points()).circle(P.M2 / 2).extrude(P.SPOKE_T)
    return part.cut(holes)


def guard_mount() -> cq.Workplane:
    """X biçimli plaka: motor delikleri merkez diskte, 4 kol direklere uzanır."""
    s = P.MOTOR_HOLE_SPACING / 2
    disc_r = math.hypot(s, s) + P.M3 / 2 + 2.0
    post_h = P.PROP_PLANE_Z - P.GUARD_BELOW - P.HUB_T
    plate = cq.Workplane("XY").circle(disc_r).extrude(P.HUB_T)
    for k in range(P.POSTS):
        lug = (cq.Workplane("XY").center((disc_r + post_radius()) / 2, 0)
               .rect(post_radius() - disc_r + P.POST_D / 2, P.LUG_W).extrude(P.HUB_T)
               .rotate((0, 0, 0), (0, 0, 1), 45.0 + 90.0 * k))
        plate = plate.union(lug)
    plate = plate.union(cq.Workplane("XY").pushPoints(post_points()).circle(P.POST_D / 2 + 1.0).extrude(P.HUB_T))
    holes = (cq.Workplane("XY").pushPoints([(s, s), (s, -s), (-s, s), (-s, -s)]).circle(P.M3 / 2).extrude(P.HUB_T)
             .union(cq.Workplane("XY").circle(4.5).extrude(P.HUB_T)))          # mil segmanı
    plate = plate.cut(holes)
    rp = P.POST_D / 2
    foot_r, foot_h = P.POST_FOOT
    for x, y in post_points():
        # Konik ayaklı direk (döndürülmüş kesit): tabanda kesit büyür → kırılma noktası olmaz
        post = (cq.Workplane("XZ").moveTo(0.0, 0.0).lineTo(rp + foot_r, 0.0).lineTo(rp, foot_h)
                .lineTo(rp, post_h - 0.5).lineTo(rp - 0.5, post_h).lineTo(0.0, post_h).close()
                .revolve(360.0, (0, 0, 0), (0, 1, 0)).translate((x, y, P.HUB_T)))
        plate = plate.union(post)
    pilot = (cq.Workplane("XY").workplane(offset=P.HUB_T + post_h - 8.0).pushPoints(post_points())
             .circle(P.M2_PILOT / 2).extrude(8.0))
    return plate.cut(pilot)


def blocked_fraction(ring: cq.Workplane | None = None) -> float:
    """Ağ + göbek + kolların pervane diski altında kapattığı alan oranı (itki kaybı göstergesi)."""
    ring = ring if ring is not None else guard_ring()
    slab = cq.Workplane("XY").circle(P.PROP_R).extrude(P.MESH_T)
    covered = ring.intersect(slab)
    area = sum(s.Volume() for s in covered.solids().vals()) / P.MESH_T
    return area / (math.pi * P.PROP_R ** 2)


def parts() -> dict[str, tuple[cq.Workplane, str, int]]:
    return {"guard_ring": (guard_ring(), "PA-CF", 4), "guard_mount": (guard_mount(), "PA-CF", 4)}


def placed(ring: cq.Workplane, mount: cq.Workplane) -> list[cq.Workplane]:
    """Montaj konumları (gövde koordinatı): yerel +x merkezden motora bakar, yerel −x kol boyunca merkeze."""
    out = []
    for x, y in P.motor_positions():
        heading = math.degrees(math.atan2(y, x))
        z_axis = ((0, 0, 0), (0, 0, 1))
        out.append(ring.rotate(*z_axis, heading).translate((x, y, P.PROP_PLANE_Z - P.GUARD_BELOW)))
        out.append(mount.rotate(*z_axis, heading).translate((x, y, 0.0)))
    return out
