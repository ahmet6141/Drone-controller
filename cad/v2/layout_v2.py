"""DC7 v2 paketleme çalışması (CadQuery gerektirmez).

Her bileşen bir kutu (veya nokta) olarak yerleştirilir; kütleler config/hardware/tier-a-ekonomik.yaml'dan
(gövdeye ait v2 parçaları cad/v2 CAD raporundan) gelir. Kontroller:
  * her iç bileşen gövde iç yüzeyinin içinde (et + boşluk payıyla)
  * bileşenler çakışmıyor; kanallar (pervane + halka) ile en az 3 mm boşluk
  * ağırlık merkezi: batarya x konumu CG'yi motor merkezine getirecek şekilde çözülür
  * IMU ofseti (EKF2_IMU_POS), GNSS–akım/gürültü kaynakları mesafesi, görüş alanları

Kullanım: python3 cad/v2/layout_v2.py
"""
from __future__ import annotations

import math
import sys
from dataclasses import dataclass
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
sys.path.insert(0, str(HERE.parent))
sys.path.insert(0, str(HERE.parents[1] / "tools"))

import budget_calc  # noqa: E402
import layout  # noqa: E402
import params as P  # noqa: E402
import v2_params as V  # noqa: E402

VARIANT = budget_calc.HW_DIR / "variants" / "tier-a-entegre.yaml"
CLEARANCE = 1.5               # bileşen ↔ kabuk iç yüzeyi
DUCT_CLEARANCE = 3.0
GNSS_MIN_DIST = 70.0          # GNSS/pusula ↔ ESC, batarya akım yolu, Pi (USB3/Wi-Fi) en az mesafe
Box = tuple[float, float, float, float, float, float]
# Yerleşim kararları (x konumları, mm); batarya x'i CG için çözülür
LAYOUT = {"esc_x": 26.0, "pi_x": 36.0, "gnss_x": -66.0}


@dataclass
class Comp:
    name: str
    mass_g: float
    box: Box | None            # None: nokta kütle (cg kullanılır)
    cg: tuple[float, float, float] | None = None
    inside: bool = True        # gövde içinde olmalı mı

    @property
    def center(self) -> tuple[float, float, float]:
        if self.cg is not None:
            return self.cg
        x0, x1, y0, y1, z0, z1 = self.box
        return ((x0 + x1) / 2, (y0 + y1) / 2, (z0 + z1) / 2)


def _box(cx: float, cy: float, cz: float, lx: float, ly: float, lz: float) -> Box:
    return (cx - lx / 2, cx + lx / 2, cy - ly / 2, cy + ly / 2, cz - lz / 2, cz + lz / 2)


def load_variant() -> dict:
    return budget_calc.load(VARIANT)


def _mass(profile: dict, key: str) -> float:
    hits = [c for c in profile["components"] if key in c["name"]]
    if len(hits) != 1:
        raise KeyError(f"{key!r}: {len(hits)} bileşen eşleşti")
    return hits[0]["mass_g"] * hits[0].get("qty", 1)


def battery_front(battery_x: float) -> float:
    """Batarya paketinin (hücreler + ön konnektör) ön ucu."""
    return battery_x + V.PACK[0] / 2


def components(profile: dict | None = None, battery_x: float = 0.0) -> list[Comp]:
    """v2 yerleşimi. battery_x: hücre paketinin merkezi. Akıllı batarya kabuğu üç kütleye ayrılır:
    kuyruk kapağı (gövdenin kuyruğunda, sabit), paket kabuğu (kapaktan ön uca, düzgün) ve ön uçtaki
    konnektör + BMS kartı."""
    pr = profile or load_variant()
    m = lambda key: _mass(pr, key)                                           # noqa: E731
    pz0, pz1 = V.PACK_Z
    px, py, pzh = V.PACK
    zc = (pz0 + pz1) / 2
    x_tail, x_front = V.STATIONS[0][0], battery_front(battery_x)
    shell = m("Akıllı batarya kabuğu")
    comps = [
        Comp("batarya", pr["battery"]["mass_g"], _box(battery_x, 0, zc, px, py, pzh)),
        Comp("batarya kuyruk kapağı", V.BATTERY_CAP_G, None, cg=(x_tail + V.TAIL_CAP / 2 - 1.0, 0.0, zc), inside=False),
        Comp("batarya paket kabuğu", shell - V.BATTERY_CAP_G - V.BATTERY_ELEC_G, None,
             cg=((x_tail + V.CAP_WALL + x_front) / 2, 0.0, zc), inside=False),
        Comp("batarya konnektörü + BMS", V.BATTERY_ELEC_G, None, cg=(x_front - 4.0, 0.0, zc), inside=False),
        Comp("FC (IMU)", m("ARK FPV"), _box(0.0, 0, 20.0, 30, 30, 8)),
        Comp("ESC (4'ü 1 arada)", pr["propulsion"]["esc"]["mass_g"], _box(LAYOUT["esc_x"], 0, -1.0, 36, 36, 10)),
        Comp("companion (Pi 5 + AI HAT+)", m("Raspberry Pi 5"), _box(LAYOUT["pi_x"], 0, -24.0, 56, 85, 32)),
        Comp("BEC", m("BEC"), _box(LAYOUT["esc_x"] + 30.0, 0, 0.0, 24, 18, 8)),
        Comp("WFB-ng adaptör", m("WFB-ng"), _box(LAYOUT["esc_x"] + 32.0, 0, 10.0, 30, 22, 9)),
        Comp("GNSS + pusula", m("GNSS"), _box(LAYOUT["gnss_x"], 0, 18.5, 26, 26, 9)),
        Comp("ELRS alıcı", m("XR4"), _box(-36.0, 0, 20.0, 20, 12, 6)),
        Comp("Remote ID", m("Remote ID"), _box(28.0, 0, 20.0, 26, 18, 6)),
        Comp("VL53L1X (yukarı)", m("VL53L1X"), _box(50.0, 0, 33.0, 13, 13, 2)),
        Comp("gimbal (motorlar, taşıyıcı, kapsül)", m("fırçasız gimbal") - V.GIMBAL_CTRL["g"], None, cg=V.GIMBAL_CG,
             inside=False),
        Comp("gimbal kontrolcüsü", V.GIMBAL_CTRL["g"], _box(*V.GIMBAL_CTRL["center"], *V.GIMBAL_CTRL["size"])),
        Comp("gimbal kamerası (CM3)", m("Camera Module 3 (IMX708"), None, cg=V.GIMBAL_POS, inside=False),
        Comp("avuç ayağı sensörleri (CM3 Wide + 8×8 ToF + MTF-01)",
             m("Camera Module 3 Wide") + m("VL53L8CX") + m("MTF-01"), None,
             cg=(V.POD_X, 0.0, V.POD_BOTTOM_Z + V.SENSOR_RECESS), inside=False),
        Comp("kablolama ve bağlantılar", m("Kablolama"), None, cg=(0.0, 0.0, -4.0), inside=False),
    ]
    # Gövde parçaları: CAD ağırlık merkezleri (v2_params.PART_CG)
    for key, part in (("Üst kabuk", "top_shell"), ("Alt kabuk", "bottom_tub"), ("Burun kapağı", "nose_cover"),
                      ("Avuç ayağı", "pod")):
        comps.append(Comp(key, m(key), None, cg=V.PART_CG[part], inside=False))
    arm_mass = m("Kol–kanal modülü")
    top_mass = m("Üst ızgara")
    prop = pr["propulsion"]
    f = 1.0 + V.ARM_DUCT_CG[0] / P.MOTOR_R
    for x, y in P.motor_positions():
        comps.append(Comp("kol–kanal modülü", arm_mass / 4, None, cg=(x * f, y * f, V.ARM_DUCT_CG[1]), inside=False))
        comps.append(Comp("üst ızgara", top_mass / 4, None, cg=(x, y, V.TOP_GRILLE["z"] + 1.0), inside=False))
        comps.append(Comp("motor", prop["motor"]["mass_g"], None, cg=(x, y, P.HUB_T + 12.0), inside=False))
        comps.append(Comp("pervane", prop["prop"]["mass_g"], None, cg=(x, y, P.PROP_PLANE_Z), inside=False))
    return comps


def cg_of(comps: list[Comp]) -> tuple[tuple[float, float, float], float]:
    total = sum(c.mass_g for c in comps)
    return tuple(sum(c.mass_g * c.center[i] for c in comps) / total for i in range(3)), total


def solve_battery_x(profile: dict | None = None) -> float:
    """CG_x = 0 olacak hücre paketi konumu. Moment batarya konumuna doğrusal bağlıdır (hücreler ve konnektör
    tam, paket kabuğu yarım hızla kayar; kapak gövdenin kuyruğunda sabit)."""
    pr = profile or load_variant()
    def moment(x: float) -> float:
        return sum(c.mass_g * c.center[0] for c in components(pr, x))
    m0 = moment(0.0)
    return round(-m0 / (moment(1.0) - m0), 1)


def _corners(b: Box):
    x0, x1, y0, y1, z0, z1 = b
    return [(x, y, z) for x in (x0, x1) for y in (y0, y1) for z in (z0, z1)]


def inside_body(b: Box, inset: float) -> bool:
    """Kutunun köşeleri ve kenar orta noktaları gövde iç yüzeyinin içinde mi?"""
    x0, x1, y0, y1, z0, z1 = b
    xs = (x0, (x0 + x1) / 2, x1)
    zs = (z0, (z0 + z1) / 2, z1)
    for x in xs:
        for z in zs:
            hw = V.half_width(x, z, inset)
            if hw < 0 or max(abs(y0), abs(y1)) > hw:
                return False
    return True


def _overlap(a: Box, b: Box) -> bool:
    return a[0] < b[1] and b[0] < a[1] and a[2] < b[3] and b[2] < a[3] and a[4] < b[5] and b[4] < a[5]


def duct_gap(b: Box) -> float:
    """Kutunun en yakın kanala (halka dış yüzeyi + pervane diski) yatay mesafesi, kanal yüksekliğinde."""
    z_lo, z_hi = V.DUCT_Z[0] - 1.0, V.DUCT_Z[1] + V.DUCT_FLARE[0]
    if b[5] < z_lo or b[4] > z_hi:
        return math.inf
    r_out = V.DUCT_IN_R + V.DUCT_WALL + V.DUCT_FLARE[1]
    best = math.inf
    for mx, my in P.motor_positions():
        dx = max(b[0] - mx, 0.0, mx - b[1])
        dy = max(b[2] - my, 0.0, my - b[3])
        best = min(best, math.hypot(dx, dy) - r_out)
    return best


def body_duct_gap() -> float:
    """Gövdenin kanal yüksekliğindeki kesitleri ile kanallar arası en küçük yatay boşluk."""
    r_out = V.DUCT_IN_R + V.DUCT_WALL + V.DUCT_FLARE[1]
    best = math.inf
    x_lo, x_hi = V.body_x_range()
    for i in range(int(x_hi - x_lo) + 1):
        x = x_lo + i
        for z in (V.DUCT_Z[0], (V.DUCT_Z[0] + V.DUCT_Z[1]) / 2, V.DUCT_Z[1] + V.DUCT_FLARE[0]):
            hw = V.half_width(x, z)
            if hw < 0:
                continue
            for mx, my in P.motor_positions():
                d = math.hypot(x - mx, max(abs(my) - hw, 0.0) if abs(my) > hw else 0.0) - r_out
                best = min(best, d)
    return best


def checks(profile: dict | None = None) -> list[tuple[str, bool, str]]:
    pr = profile or load_variant()
    bx = solve_battery_x(pr)
    comps = components(pr, bx)
    cg, total = cg_of(comps)
    out: list[tuple[str, bool, str]] = []
    boxed = [c for c in comps if c.box is not None and c.inside]
    bad_inside = [c.name for c in boxed if not inside_body(c.box, V.WALL + CLEARANCE)]
    out.append(("iç bileşenler gövdenin içinde", not bad_inside, ", ".join(bad_inside) or f"{len(boxed)} bileşen"))
    clashes = [f"{a.name} ↔ {b.name}" for i, a in enumerate(boxed) for b in boxed[i + 1:] if _overlap(a.box, b.box)]
    out.append(("bileşenler çakışmıyor", not clashes, ", ".join(clashes) or "temiz"))
    gaps = {c.name: duct_gap(c.box) for c in boxed}
    worst = min(gaps.items(), key=lambda kv: kv[1])
    out.append(("bileşen ↔ kanal boşluğu", worst[1] >= DUCT_CLEARANCE,
                f"en yakın: {worst[0]} {worst[1]:.1f} mm" if math.isfinite(worst[1]) else "kanal yüksekliğinde bileşen yok"))
    bd = body_duct_gap()
    out.append(("gövde ↔ kanal boşluğu", bd >= DUCT_CLEARANCE, f"{bd:.1f} mm ≥ {DUCT_CLEARANCE:.0f}"))
    pack = next(c for c in comps if c.name == "batarya").box
    length = battery_front(bx) - V.STATIONS[0][0]
    need = V.TAIL_CAP + V.PACK[0]
    out.append(("batarya kuyruktan takılıyor (kapak = kuyruk)", need - 1.0 <= length <= need + 8.0,
                f"hücre merkezi x = {bx:+.1f} mm; kapaktan ön uca {length:.1f} mm "
                f"(en az {need:.1f}; BMS boşluğu {length - need:+.1f} mm, en çok +8)"))
    out.append(("CG yatay ofset", math.hypot(cg[0], cg[1]) <= 3.0, f"({cg[0]:+.1f}, {cg[1]:+.1f}) mm"))
    stack = next(c for c in comps if c.name == "FC (IMU)").center
    imu = (stack[0] - cg[0], stack[1] - cg[1], stack[2] - cg[2])
    out.append(("IMU ↔ CG (EKF2_IMU_POS)", math.dist(stack, cg) <= 40.0,
                f"({imu[0]:+.0f}, {imu[1]:+.0f}, {imu[2]:+.0f}) mm"))
    gnss = next(c for c in comps if c.name == "GNSS + pusula").center
    noisy = {n: next(c for c in comps if c.name == n).center
             for n in ("ESC (4'ü 1 arada)", "companion (Pi 5 + AI HAT+)", "WFB-ng adaptör", "gimbal kontrolcüsü")}
    connector = (pack[1], 0.0, (pack[4] + pack[5]) / 2)                      # konnektör ve akım yolu
    dists = {n: math.dist(gnss, p) for n, p in noisy.items()} | {"batarya konnektörü": math.dist(gnss, connector)}
    near = min(dists.items(), key=lambda kv: kv[1])
    out.append(("GNSS/pusula ↔ gürültü kaynakları", near[1] >= GNSS_MIN_DIST,
                f"en yakın {near[0]} {near[1]:.0f} mm ≥ {GNSS_MIN_DIST:.0f}"))
    drop = P.PROP_PLANE_Z - V.POD_BOTTOM_Z
    out.append(("avuç ayağı pervane düzleminin altında (tam kapalı pervane)", drop >= V.POD_MIN_DROP_ENCLOSED,
                f"{drop:.0f} mm ≥ {V.POD_MIN_DROP_ENCLOSED:.0f} (üst + alt ızgara + motor eteği; açık üstte 120)"))
    low, nose_low, mid = gimbal_heights()
    out.append(("gimbal sarkmıyor (burun alt çizgisinde, ayağın üstünde)",
                low >= nose_low - 3.0 and low >= V.POD_BOTTOM_Z + 10.0,
                f"en alçak {low:.0f} mm (−90…+30° pitch); burun altı {nose_low:.0f}, ayak {V.POD_BOTTOM_Z:.0f} mm"))
    out.append(("kamera orta hatta, gövde orta yüksekliğinde", V.GIMBAL_POS[1] == 0.0 and abs(V.GIMBAL_POS[2] - mid) <= 8.0,
                f"y = 0; kamera z {V.GIMBAL_POS[2]:.0f}, ağırlık merkezi {cg[2]:.0f}, burun ortası {mid:.0f} mm"))
    clear = pod_sensor_clearance()
    out.append(("ayak sensörleri ağızdan kırpılmadan görür", all(c >= need for c, need in clear.values()),
                ", ".join(f"{n} {c:.0f}° ≥ {need:.0f}°" for n, (c, need) in clear.items())))
    return out


def gimbal_heights() -> tuple[float, float, float]:
    """(kapsülün en alçak noktası, pitch eksenindeki burun alt yüzü, kamera kesitinde burnun orta yüksekliği)."""
    low = V.PIVOT_Z + V.GIMBAL_SWEEP_Z[0]
    nose_low = V.section_at(V.GIMBAL_PIVOT[0])[0]
    z0, *_, z1 = V.section_at(V.GIMBAL_POS[0])[:4]
    return low, nose_low, (z0 + z1) / 2


def lens_at(pitch: float) -> tuple[float, float, float]:
    """Mercek giriş göz bebeği (gövde koordinatı): pitch ekseni etrafında döner (+ = burun yukarı)."""
    px, _, pz = V.GIMBAL_PIVOT
    d = V.CAM_PUPIL_X - V.PITCH_AXIS_DX
    th = math.radians(pitch)
    return (px + d * math.cos(th), 0.0, pz + d * math.sin(th))


def view_obstacles() -> dict[str, list[tuple[float, float, float]]]:
    """Gimbal kamerasının görüşüne girmemesi gereken v2 yapıları (nokta bulutu): kanal halkaları ve çan ağızları,
    üst ızgaralar, kollar, avuç ayağı ve burnun ağız çevresi."""
    pts: dict[str, list[tuple[float, float, float]]] = {"kanal": [], "üst ızgara": [], "kol": [], "ayak": [], "burun": []}
    r_ring = V.DUCT_IN_R + V.DUCT_WALL + V.DUCT_FLARE[1]
    g = V.TOP_GRILLE
    for mx, my in P.motor_positions():
        for k in range(120):
            a = math.radians(3 * k)
            for r, z in ((V.DUCT_IN_R, V.DUCT_Z[0]), (r_ring, V.DUCT_Z[1]), (r_ring, V.DUCT_Z[1] + V.DUCT_FLARE[0])):
                pts["kanal"].append((mx + r * math.cos(a), my + r * math.sin(a), z))
            for r in (g["r_out"], g["r_out"] * 0.7, g["r_out"] * 0.4):
                pts["üst ızgara"].append((mx + r * math.cos(a), my + r * math.sin(a), g["z"]))
        for t in range(31):
            f = 0.25 + 0.75 * t / 30
            for z in (-17.0, 0.0, 18.0):
                pts["kol"].append((mx * f, my * f, z * (1.0 - 0.5 * f)))
    for k in range(72):
        a = math.radians(5 * k)
        for z in (V.POD_BOTTOM_Z, V.POD_BOTTOM_Z + 15.0, belly_z_estimate()):
            pts["ayak"].append((V.POD_X + V.POD_TOP_R * math.cos(a), V.POD_TOP_R * math.sin(a), z))
    hy, top = V.MOUTH_HALF_Y, V.PIVOT_Z + V.MOUTH_TOP
    x_tip = V.STATIONS[-1][0]
    for i in range(41):
        x = V.GIMBAL_PIVOT[0] - 10.0 + (x_tip - V.GIMBAL_PIVOT[0] + 10.0) * i / 40
        for y, z in V.section_points(x):
            if abs(y) <= hy and z <= top:                       # ağzın içi: kabuk yok
                continue
            pts["burun"].append((x, y, z))
    return pts


def belly_z_estimate() -> float:
    return V.section_at(V.POD_X)[0]


def gimbal_view(step: float = 5.0) -> dict:
    """Pitch aralığında (mekanik −90…+30°) kamera görüşüne giren yapılar; kamera her açıda pitch ekseni etrafında
    döner (mercek konumu lens_at)."""
    pts = view_obstacles()
    blocked: dict[float, list[str]] = {}
    n = int(round((P.PITCH_RANGE[1] - P.PITCH_RANGE[0]) / step))
    pitches = [P.PITCH_RANGE[0] + i * step for i in range(n + 1)]
    for pitch in pitches:
        cam = lens_at(pitch)
        hits = [name for name, group in pts.items()
                if any(layout.in_camera_view(p, cam, pitch, P.CAM_HFOV, P.CAM_VFOV) for p in group)]
        if hits:
            blocked[pitch] = hits
    clear = [p for p in pitches if p not in blocked]
    return {"blocked": blocked, "clear_min": min(clear) if clear else None,
            "clear_max": max(clear) if clear else None}


def tip_angle(profile: dict | None = None) -> float:
    pr = profile or load_variant()
    cg, _ = cg_of(components(pr, solve_battery_x(pr)))
    return math.degrees(math.atan2(V.POD_BOTTOM_R, cg[2] - V.POD_BOTTOM_Z))


def pod_sensor_clearance() -> dict[str, tuple[float, float]]:
    """Her ayak sensörünün TPU uç ağzından (iç yarıçap) kırpılmadan görebildiği yarım açı (°) ve gereken açı.
    Mercek sensör camına değer → derinlik = SENSOR_RECESS; en kötü yön: ağız kenarına en yakın taraf."""
    r_open = V.POD_BOTTOM_R - P.BUMPER_WALL
    out = {}
    for name, ((x, y), need) in V.POD_SENSORS.items():
        out[name] = (math.degrees(math.atan2(r_open - math.hypot(x, y), V.SENSOR_RECESS)), need)
    return out


def main() -> int:
    pr = load_variant()
    bx = solve_battery_x(pr)
    comps = components(pr, bx)
    cg, total = cg_of(comps)
    print(f"v2 toplam kütle {total:.0f} g; batarya paketi merkezi x = {bx:+.1f} mm")
    print(f"Ağırlık merkezi ({cg[0]:+.1f}, {cg[1]:+.1f}, {cg[2]:+.1f}) mm — pervane düzleminin "
          f"{P.PROP_PLANE_Z - cg[2]:.0f} mm altında; avuçta devrilme açısı {tip_angle(pr):.1f}°")
    view = gimbal_view()
    print(f"Gimbal görüşü temiz: {view['clear_min']:+.0f}° … {view['clear_max']:+.0f}° (yazılım sınırı +{P.PITCH_SOFT_MAX:.0f}°)")
    ok = True
    for name, passed, detail in checks(pr):
        ok &= passed
        print(f"  [{'OK ' if passed else 'HATA'}] {name}: {detail}")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
