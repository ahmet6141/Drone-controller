"""Yerleşim kontrolleri (CadQuery gerektirmez): ağırlık merkezi, batarya konumu, görüş alanları.

Kütleler config/hardware/tier-a-ekonomik.yaml'dan, konumlar bu dosyadan gelir (öneri / tahmini).
Bataryanın x konumu, ağırlık merkezini (CG) gövde merkezine getirecek şekilde hesaplanır.

Kullanım:
    python3 cad/layout.py
"""
from __future__ import annotations

import math
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "tools"))
sys.path.insert(0, str(Path(__file__).resolve().parent))

import budget_calc  # noqa: E402
import params as P  # noqa: E402

PROFILE = budget_calc.HW_DIR / "tier-a-ekonomik.yaml"
Point = tuple[float, float, float]

# Bileşen adı parçası → konum (mm). Her bileşen tam olarak bir girdiyle eşleşmelidir.
PLACEMENT: dict[str, Point] = {
    "GEPRC MOZ7": (0.0, 0.0, 0.0),
    "Tam pervane koruması": (0.0, 0.0, P.PROP_PLANE_Z - P.GUARD_BELOW),        # 4 simetrik koruma
    "Avuç iniş tutamağı": (0.0, 0.0, (P.FRAME_BOTTOM_Z + P.GRIP_BOTTOM_Z) / 2),
    "Üst plaka, bağlantılar": (0.0, 0.0, P.TRAY_Z),
    "ARK FPV": (0.0, 0.0, 10.0),
    "Raspberry Pi 5": (P.PI5_X, 0.0, P.TRAY_Z + P.COMPANION_STACK_H / 2),
    "Camera Module 3 (IMX708": P.GIMBAL_POS,
    "2 eksen fırçasız gimbal": (P.GIMBAL_POS[0] - 12.0, 0.0, P.GIMBAL_POS[2] + 14.0),
    "Camera Module 3 Wide": (0.0, 0.0, P.GRIP_BOTTOM_Z + P.SENSOR_RECESS),
    "VL53L8CX": (0.0, 0.0, P.GRIP_BOTTOM_Z + P.SENSOR_RECESS),
    "VL53L1X": (P.OVERHEAD_TOF_POS[0], P.OVERHEAD_TOF_POS[1], P.BATTERY_PLATE_Z + 3.0),
    "MTF-01": (P.FLOW_X, 0.0, P.FRAME_BOTTOM_Z - 3.0),
    "M10 GNSS": P.GNSS_POS,                                                   # arka direk
    "XR4": (-40.0, 25.0, 12.0),
    "WFB-ng": (-10.0, -28.0, P.TRAY_Z + 5.0),
    "Remote ID": (-55.0, -20.0, 12.0),
    "BEC": (0.0, 28.0, 12.0),
}
MOTOR_Z = P.HUB_T + 12.0
ESC_POS: Point = (0.0, 0.0, 6.0)
CG_TOLERANCE_MM = 5.0


def _match(name: str) -> Point:
    hits = [pos for key, pos in PLACEMENT.items() if key in name]
    if len(hits) != 1:
        raise KeyError(f"{name!r} için {len(hits)} yerleşim girdisi var (1 olmalı)")
    return hits[0]


def masses(profile: dict | None = None) -> list[tuple[str, float, Point]]:
    """(ad, kütle_g, konum) listesi — batarya hariç."""
    profile = profile or budget_calc.load(PROFILE)
    out = [(c["name"], c["mass_g"] * c.get("qty", 1), _match(c["name"])) for c in profile["components"]]
    prop = profile["propulsion"]
    for x, y in P.motor_positions():
        out.append(("motor", prop["motor"]["mass_g"], (x, y, MOTOR_Z)))
        out.append(("pervane", prop["prop"]["mass_g"], (x, y, P.PROP_PLANE_Z)))
    out.append(("ESC", prop["esc"]["mass_g"], ESC_POS))
    return out


def battery_position(profile: dict | None = None) -> Point:
    """CG'yi x = 0'a getiren batarya konumu (batarya plakası üstünde)."""
    profile = profile or budget_calc.load(PROFILE)
    items = masses(profile)
    moment_x = sum(m * pos[0] for _, m, pos in items)
    x = -moment_x / profile["battery"]["mass_g"]
    z = P.BATTERY_PLATE_Z + P.BATTERY_PLATE[2] + P.BATTERY_SIZE[2] / 2
    return (round(x, 1), 0.0, z)


def center_of_gravity(profile: dict | None = None, battery: Point | None = None) -> tuple[Point, float]:
    profile = profile or budget_calc.load(PROFILE)
    items = masses(profile) + [("batarya", profile["battery"]["mass_g"], battery or battery_position(profile))]
    total = sum(m for _, m, _ in items)
    cg = tuple(sum(m * pos[i] for _, m, pos in items) / total for i in range(3))
    return cg, total


def inertia(profile: dict | None = None) -> tuple[float, float, float]:
    """Ağırlık merkezine göre Ixx, Iyy, Izz (kg·m²) — SITL (Gazebo) modeli için başlangıç (≈ ±%30).

    Nokta kütle yaklaşımı; korumalar motor konumlarına (halka öz ataleti dahil), gövdenin %60'ı
    kol ortalarına dağıtılır.
    """
    profile = profile or budget_calc.load(PROFILE)
    cg, _ = center_of_gravity(profile)
    pts: list[tuple[float, Point]] = []
    self_i = [0.0, 0.0, 0.0]
    for name, m, pos in masses(profile) + [("batarya", profile["battery"]["mass_g"], battery_position(profile))]:
        if "pervane koruması" in name:
            r = P.guard_outer_r() + P.GUARD_FLARE_OUT / 2
            for x, y in P.motor_positions():
                pts.append((m / 4, (x, y, pos[2])))
                self_i[0] += m / 4 * r * r / 2
                self_i[1] += m / 4 * r * r / 2
                self_i[2] += m / 4 * r * r
        elif "MOZ7" in name:
            pts.append((0.4 * m, pos))
            pts += [(0.15 * m, (x / 2, y / 2, -P.ARM_T / 2)) for x, y in P.motor_positions()]
        else:
            pts.append((m, pos))
    ixx = sum(m * ((p[1] - cg[1]) ** 2 + (p[2] - cg[2]) ** 2) for m, p in pts) + self_i[0]
    iyy = sum(m * ((p[0] - cg[0]) ** 2 + (p[2] - cg[2]) ** 2) for m, p in pts) + self_i[1]
    izz = sum(m * ((p[0] - cg[0]) ** 2 + (p[1] - cg[1]) ** 2) for m, p in pts) + self_i[2]
    return tuple(round(v * 1e-9, 5) for v in (ixx, iyy, izz))       # g·mm² → kg·m²


# --- Görüş alanı kontrolleri -----------------------------------------------------------------

def obstacle_points() -> dict[str, list[Point]]:
    """Görüşe girmemesi gereken yapılar: koruma halkaları, pervane uçları, tutamak, kollar, alt plaka."""
    ring_r = P.guard_max_r()
    zs_ring = (P.PROP_PLANE_Z - P.GUARD_BELOW, P.PROP_PLANE_Z, P.PROP_PLANE_Z + P.GUARD_ABOVE)
    pts: dict[str, list[Point]] = {"koruma": [], "pervane": [], "tutamak": [], "kol": [], "alt plaka": []}
    for mx, my in P.motor_positions():
        for k in range(72):
            a = math.radians(5 * k)
            for z in zs_ring:
                pts["koruma"].append((mx + ring_r * math.cos(a), my + ring_r * math.sin(a), z))
            pts["pervane"].append((mx + P.PROP_R * math.cos(a), my + P.PROP_R * math.sin(a), P.PROP_PLANE_Z))
        for t in range(21):
            f = t / 20
            for z in (0.0, -P.ARM_T):
                pts["kol"].append((mx * f, my * f, z))
    r = P.GRIP_D / 2
    for k in range(36):
        a = math.radians(10 * k)
        for t in range(11):
            z = P.FRAME_BOTTOM_Z + (P.GRIP_BOTTOM_Z - P.FRAME_BOTTOM_Z) * t / 10
            pts["tutamak"].append((r * math.cos(a), r * math.sin(a), z))
    for x in range(-50, 51, 10):
        for y in (-30.0, 30.0):
            pts["alt plaka"].append((float(x), y, P.FRAME_BOTTOM_Z))
    return pts


def in_camera_view(p: Point, cam: Point, pitch_deg: float, hfov: float, vfov: float) -> bool:
    """İğne deliği kamera: +x'e bakar, pitch (yukarı +) kadar döndürülmüş; dikdörtgen görüş alanı."""
    vx, vy, vz = (p[0] - cam[0], p[1] - cam[1], p[2] - cam[2])
    th = math.radians(pitch_deg)
    fwd = vx * math.cos(th) + vz * math.sin(th)
    up = -vx * math.sin(th) + vz * math.cos(th)
    if fwd <= 1e-6:
        return False
    return (abs(math.degrees(math.atan2(vy, fwd))) <= hfov / 2
            and abs(math.degrees(math.atan2(up, fwd))) <= vfov / 2)


def in_cone(p: Point, apex: Point, axis_z: float, half_angle_deg: float) -> bool:
    """Dikey eksenli koni (axis_z = −1 aşağı, +1 yukarı) içinde mi?"""
    vx, vy, vz = (p[0] - apex[0], p[1] - apex[1], p[2] - apex[2])
    along = vz * axis_z
    if along <= 1e-6:
        return False
    return math.degrees(math.atan2(math.hypot(vx, vy), along)) <= half_angle_deg


def gimbal_view_report(step: float = 5.0) -> dict:
    """Pitch aralığında gimbal kamerasının görüşüne giren yapılar."""
    pts = obstacle_points()
    blocked: dict[float, list[str]] = {}
    pitch = P.PITCH_RANGE[0]
    while pitch <= P.PITCH_RANGE[1] + 1e-9:
        hits = [name for name, group in pts.items()
                if any(in_camera_view(p, P.GIMBAL_POS, pitch, P.CAM_HFOV, P.CAM_VFOV) for p in group)]
        if hits:
            blocked[pitch] = hits
        pitch += step
    clear = [p for p in _frange(P.PITCH_RANGE[0], P.PITCH_RANGE[1], step) if p not in blocked]
    return {"blocked": blocked, "clear_min": min(clear) if clear else None,
            "clear_max": max(clear) if clear else None}


def _frange(a: float, b: float, step: float) -> list[float]:
    n = int(round((b - a) / step))
    return [a + i * step for i in range(n + 1)]


def flow_view_clear() -> list[str]:
    pos = PLACEMENT["MTF-01"]
    return [name for name, group in obstacle_points().items()
            if any(in_cone(p, pos, -1.0, P.FLOW_FOV / 2) for p in group)]


def overhead_view_clear(profile: dict | None = None, half_angle: float = 13.5) -> list[str]:
    """Yukarı bakan VL53L1X (27° FOV): batarya ve GNSS direği görüşe girmemeli."""
    bx, by, bz = battery_position(profile)
    sx, sy, sz = P.BATTERY_SIZE
    battery = [(bx + dx * sx / 2, by + dy * sy / 2, bz + sz / 2) for dx in (-1, 0, 1) for dy in (-1, 0, 1)]
    gx, gy, gz = PLACEMENT["M10 GNSS"]
    mast = [(gx, gy, P.FRAME_TOP_Z + (gz - P.FRAME_TOP_Z) * k / 10) for k in range(11)]
    pos = PLACEMENT["VL53L1X"]
    hits = []
    if any(in_cone(p, pos, 1.0, half_angle) for p in battery):
        hits.append("batarya")
    if any(in_cone(p, pos, 1.0, half_angle) for p in mast):
        hits.append("GNSS direği")
    return hits


def down_camera_clear_half_angle() -> float:
    """Tutamak içine gömülü aşağı kameranın tampon ağzından görebildiği yarım açı (°)."""
    r_open = P.GRIP_D / 2 - P.BUMPER_WALL          # tampon ağzı yarıçapı (palm_grip.py ile aynı)
    return math.degrees(math.atan2(r_open, P.SENSOR_RECESS))


def design_rules() -> list[tuple[str, bool, str]]:
    """CAD'den bağımsız tasarım kuralları (docs/05 §3, docs/10 §3)."""
    m = P.MOTOR_XY
    side_gap = 2 * m - 2 * P.guard_max_r()
    tray_corner = (P.TRAY[0] / 2, P.TRAY[1] / 2)
    tray_to_ring = math.hypot(m - tray_corner[0], m - tray_corner[1]) - P.guard_max_r()
    battery = battery_position()
    fits = abs(battery[0]) + P.BATTERY_SIZE[0] / 2 <= P.BATTERY_PLATE[0] / 2
    mast_gap = abs(P.GNSS_POS[0] - battery[0]) - P.BATTERY_SIZE[0] / 2
    cg, _ = center_of_gravity()
    drop = P.PROP_PLANE_Z - P.GRIP_BOTTOM_Z
    view = gimbal_view_report()
    collar_in = P.MOTOR_BELL_D / 2 + P.COLLAR_GAP
    collar_out = collar_in + P.COLLAR_W
    mesh_z = P.PROP_PLANE_Z - P.GUARD_BELOW
    blade_gap = min(
        min(P.PROP_PLANE_Z - P.prop_half_height(r) - (mesh_z + P.SPOKE_T)
            for r in (collar_in + 0.5 * k for k in range(int(2 * (collar_out - collar_in)) + 1))),
        min(P.PROP_PLANE_Z - P.prop_half_height(r) - (mesh_z + P.MESH_T)
            for r in range(int(collar_out), int(P.PROP_R))))
    return [
        ("pervane ↔ halka boşluğu", P.GUARD_CLEARANCE >= 6.0, f"{P.GUARD_CLEARANCE:.1f} mm ≥ 6"),
        ("kanat ↔ ağ / göbek düşey boşluğu", blade_gap >= 2.0, f"{blade_gap:.1f} mm ≥ 2 (temsili kanat kesitleri)"),
        ("ağ gözü", P.MESH_OPENING <= 10.0, f"{P.MESH_OPENING:.1f} mm ≤ 10"),
        ("tutamak tabanı pervane düzleminin altında", drop >= P.GRIP_MIN_DROP, f"{drop:.0f} mm ≥ {P.GRIP_MIN_DROP:.0f}"),
        ("tutamak çapı", 65.0 <= P.GRIP_D <= 75.0, f"Ø {P.GRIP_D:.0f} mm (65–75)"),
        ("komşu korumalar arası boşluk", side_gap >= 20.0, f"{side_gap:.1f} mm ≥ 20"),
        ("tepsi köşesi ↔ koruma halkası", tray_to_ring >= 3.0, f"{tray_to_ring:.1f} mm ≥ 3"),
        ("batarya plakaya sığıyor", fits, f"batarya x = {battery[0]:+.1f} mm"),
        ("GNSS direği ↔ batarya", mast_gap >= 5.0, f"{mast_gap:.1f} mm ≥ 5"),
        ("CG yatay ofset", math.hypot(cg[0], cg[1]) <= CG_TOLERANCE_MM,
         f"({cg[0]:+.1f}, {cg[1]:+.1f}) mm ≤ {CG_TOLERANCE_MM:.0f}"),
        ("akış sensörü görüşü temiz", not flow_view_clear(), ", ".join(flow_view_clear()) or "temiz"),
        ("yukarı ToF görüşü temiz", not overhead_view_clear(), ", ".join(overhead_view_clear()) or "temiz"),
        ("gimbal yazılım aralığında görüş temiz", all(p > P.PITCH_SOFT_MAX for p in view["blocked"]),
         f"temiz {view['clear_min']:+.0f}° … {view['clear_max']:+.0f}°, yazılım sınırı ≤ {P.PITCH_SOFT_MAX:+.0f}°"),
    ]


def tip_angles(profile: dict | None = None) -> dict[str, float]:
    """Motorlar kapalıyken düz yüzeyde devrilme açısı (°): tutamak tabanı ve geniş ayak ile."""
    cg, _ = center_of_gravity(profile)
    h = cg[2] - P.GRIP_BOTTOM_Z
    return {"tutamak": math.degrees(math.atan2(P.GRIP_D / 2, h)),
            "geniş ayak": math.degrees(math.atan2(P.GROUND_FOOT_D / 2, h))}


def main() -> int:
    profile = budget_calc.load(PROFILE)
    cg, total = center_of_gravity(profile)
    bx, _, bz = battery_position(profile)
    print(f"Toplam kütle {total:.0f} g; batarya x = {bx:+.1f} mm (plaka üstü, z = {bz:.0f} mm)")
    dz = cg[2] - P.PROP_PLANE_Z
    print(f"Ağırlık merkezi: x {cg[0]:+.1f}, y {cg[1]:+.1f}, z {cg[2]:+.1f} mm"
          f" (pervane düzleminin {abs(dz):.0f} mm {'üstünde' if dz > 0 else 'altında'})")
    tips = tip_angles(profile)
    print(f"Devrilme açısı (motorlar kapalı): tutamak tabanında {tips['tutamak']:.1f}°,"
          f" Ø{P.GROUND_FOOT_D:.0f} mm ayakla {tips['geniş ayak']:.1f}°")
    ixx, iyy, izz = inertia(profile)
    print(f"Atalet (CG'ye göre, SITL başlangıcı): Ixx {ixx:.4f}, Iyy {iyy:.4f}, Izz {izz:.4f} kg·m²")
    view = gimbal_view_report()
    for pitch, hits in sorted(view["blocked"].items()):
        print(f"  gimbal pitch {pitch:+.0f}°: görüşte {', '.join(hits)}")
    print(f"Aşağı kamera tampon ağzı yarım açısı: {down_camera_clear_half_angle():.1f}°"
          f" (CM3 Wide yatay {P.WIDE_HFOV / 2:.0f}°, dikey {P.WIDE_VFOV / 2:.1f}°)")
    ok = True
    for name, passed, detail in design_rules():
        ok &= passed
        print(f"  [{'OK ' if passed else 'HATA'}] {name}: {detail}")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
