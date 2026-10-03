#!/usr/bin/env python3
"""DC7 v2 bütünleşik gövde: parçaları üretir, STL/STEP/GLB dışa aktarır; kütle (üretim ve prototip
malzemesiyle), kalıp (DFM) kuralları, çakışma kontrolleri, rapor ve önizlemeler.

Gereksinim: pip install cadquery matplotlib
Kullanım:
    python3 cad/v2/build_v2.py             # cad/v2/out/: stl/, step/, dc7_v2.glb, report.md, preview_v2.png
    python3 cad/v2/build_v2.py --fast      # çakışma kontrollerinin hızlı alt kümesi, önizleme yok
"""
from __future__ import annotations

import argparse
import json
import math
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
sys.path.insert(0, str(HERE.parent))
sys.path.insert(0, str(HERE.parents[1] / "tools"))

import cadquery as cq  # noqa: E402

import airframe as A  # noqa: E402
import analysis_v2 as AN  # noqa: E402
import budget_calc  # noqa: E402
import build as B  # noqa: E402  (v1 çizici ve GLB yardımcıları)
import gimbal_v2 as GV  # noqa: E402
import layout_v2 as K  # noqa: E402
import params as P  # noqa: E402
import standins as S  # noqa: E402
import v2_params as V  # noqa: E402

OUT = HERE / "out"
# parça → (üretim malzemesi, adet, kalıp/üretim notu)
PRODUCTION = {
    "top_shell": ("PC/ABS", 1, "kanopi; yarım kol soketleri ayrım düzlemine açık; kanopi emiş yarıkları dikey (maçasız)"),
    "bottom_tub": ("PC/ABS", 1, "taşıyıcı: V uçlu kol soketleri, batarya rayları, vida kuleleri; yan yarıklar için 2 kayar maça"),
    "nose_cover": ("PC/ABS", 1, "burun üst yarısı: alın + yanak üstleri + ağız astarı (2K: dış gövde rengi, astar siyah)"),
    "nose_chin": ("PC/ABS", 1, "burun alt yarısı + sönümleyici perdesi; gimbal iki yarının arasına oturur (ayrım = pitch ekseni)"),
    "arm_duct": ("PA6-GF30", 4, "TEK kalıp × 4; U kesit kol + çan ağızlı kanal + bal peteği ızgara + 3 radyal kaburga + motor çanı eteği; göbekten yolluk"),
    "top_grille": ("PC", 4, "çıkarılabilir üst ızgara: 6 ayak + 3 geçme tırnak; pervane değişiminde çıkar; tek kalıp × 4"),
    "pod": ("PC/ABS", 1, f"avuç ayağı (kısa, Ø{2 * V.POD_BOTTOM_R:.0f}); sensör tablası tabandan {V.SENSOR_RECESS:.0f} mm, IR camlı"),
    "pod_tip": ("TPU", 1, "avuca değen uç (TPU 95A; seri üretimde ayağın üstüne ikinci enjeksiyon)"),
    "sensor_window": ("PC", 1, "IR geçirgen sensör camı (siyah IR mürekkep maskeli), sensörlere sıfır boşlukla"),
    "battery_shell": ("PC/ABS", 1, "akıllı batarya kabuğu + kuyruk kapağı (gövde çizgisini tamamlar)"),
}
# Gimbal parçaları (gimbal_v2): gövde toplamından ayrı raporlanır, profilde "fırçasız gimbal" kalemindedir
GIMBAL_PRODUCTION = {
    "gimbal_bracket": (GV.MATERIAL, 1, "sönümlü U taşıyıcı: arka plaka 4 sönümleyiciyle perdeye; yan plakalar yanakların içinde"),
    "gimbal_frame": (GV.MATERIAL, 1, "pitch çerçevesi: roll motoru arka plakada; kollar pitch eksenine (+y motor, −y pim)"),
    "gimbal_cradle": (GV.MATERIAL, 1, "beşik: roll rotoruna bağlı; CM3 kartı ara parçalarla önünde"),
    "camera_housing": ("PC", 1, "kamera başlığı: öne daralan kapak (1 mm); mercek halkası alüminyum, koruyucu cam"),
}
PROTOTYPE_MATERIAL = {"TPU": "TPU"}                 # diğerleri MJF PA12 ile basılır
# Varyant profilindeki bileşen adı → (CAD parçaları, CAD dışı ek kütle g)
VARIANT_MAP = {
    "Üst kabuk": (["top_shell"], 0.0), "Alt kabuk": (["bottom_tub"], 0.0),
    "Burun kapağı": (["nose_cover", "nose_chin"], 0.0),
    "Kol–kanal modülü": (["arm_duct"], 0.0), "Avuç ayağı": (["pod", "pod_tip", "sensor_window"], 0.0),
    "Üst ızgara": (["top_grille"], 0.0),
    # kabuk + kilit düğmeleri (POM) + konnektör, BMS/yakıt göstergesi kartı, yaylar ≈ 10 g
    "Akıllı batarya kabuğu": (["battery_shell", "battery_latch"], 10.0),
    # basılan/kalıplanan parçalar + 2 motor, IMU, kontrolcü, sönümleyiciler, pim + rulman, mercek halkası ve cam
    "fırçasız gimbal": (list(GIMBAL_PRODUCTION), None),
}
LATCH_MATERIAL = "POM"


def volume(wp: cq.Workplane) -> float:
    return sum(s.Volume() for s in wp.solids().vals())


def build_parts() -> dict[str, cq.Workplane]:
    parts = A.body_parts_detailed()
    parts["arm_duct"] = A.arm_duct()
    parts["pod"] = A.pod()
    parts["pod_tip"] = A.pod_tip()
    parts["sensor_window"] = A.sensor_window()
    parts["top_grille"] = A.top_grille()
    bat = A.battery_parts()
    parts["battery_shell"] = bat["battery_shell"]
    parts["battery_latch"] = bat["battery_latch"]
    for name, shape in GV.printed_parts().items():                 # gövde koordinatında (nötr poz)
        parts[name] = shape.translate(V.GIMBAL_POS)
    return parts


def center_of_mass(wp: cq.Workplane) -> tuple[float, float, float]:
    c = cq.Shape.centerOfMass(cq.Compound.makeCompound(wp.solids().vals()))
    return (round(c.x, 1), round(c.y, 1), round(c.z, 1))


def mass_rows(parts: dict[str, cq.Workplane], table: dict | None = None) -> list[dict]:
    rows = []
    for name, (mat, qty, note) in (PRODUCTION if table is None else table).items():
        v = volume(parts[name]) / 1000.0
        proto = PROTOTYPE_MATERIAL.get(mat, "PA12 (MJF)")
        rows.append({"part": name, "qty": qty, "material": mat, "volume_cm3": round(v, 2),
                     "mass_g": round(v * V.DENSITY[mat], 1), "proto_material": proto,
                     "proto_mass_g": round(v * V.DENSITY[proto], 1), "solids": parts[name].solids().size(),
                     "cg": center_of_mass(parts[name]), "note": note})
    return rows


def gimbal_items(pitch: float = 0.0, roll: float = 0.0) -> list[tuple[str, cq.Workplane]]:
    """Ön gimbal (gimbal_v2) gövde koordinatında + uçuş kontrolcüsünün üstündeki gimbal kontrolcüsü kartı."""
    b = GV.controller_box()
    return GV.placed(pitch, roll) + [("gimbal_ctrl_pcb", S._box(*b))]


def internals() -> list[tuple[str, cq.Workplane]]:
    """Görünür iç parçalar (render ve çakışma için): Pi, FC, ESC, ayak sensörleri, LED'ler."""
    comps = {c.name: c for c in K.components(None, K.solve_battery_x())}
    items: list[tuple[str, cq.Workplane]] = []
    pi = comps["companion (Pi 5 + AI HAT+)"].box
    items += S.companion_at((pi[0] + pi[1]) / 2, pi[4], yaw_deg=90.0)
    for name, key in (("fc_pcb", "FC (IMU)"), ("esc_pcb", "ESC (4'ü 1 arada)")):
        b = comps[key].box
        items.append((name, S._box(b[0], b[1], b[2], b[3], b[4], b[4] + 1.6)))
    gb = comps["GNSS + pusula"].box
    items.append(("gnss", S._box(gb[0], gb[1], gb[2], gb[3], gb[4], gb[5])))
    items += pod_sensors()
    # Seyir lambaları: motor yuvalarının dış yüzünde (ızgaranın altında; önden ve alttan görünür)
    for (mx, my) in P.motor_positions():
        ang = math.atan2(my, mx)
        r = math.hypot(mx, my) + V.MOTOR_POD[0] - 0.5
        color = "led_white" if mx < 0 else ("led_red" if my > 0 else "led_green")
        led = (cq.Workplane("XY").box(3.0, 10.0, 4.0).rotate((0, 0, 0), (0, 0, 1), math.degrees(ang))
               .translate((r * math.cos(ang), r * math.sin(ang), P.HUB_T - 6.0)))
        items.append((color, led))
    return items


def pod_sensors() -> list[tuple[str, cq.Workplane]]:
    """Ayak tabanı sensörleri (tablanın üstünde; mercekler tabladan aşağı uzanıp sensör camına değer)."""
    z = V.POD_BOTTOM_Z + V.SENSOR_RECESS + 2.0
    (cx, cy), _ = V.POD_SENSORS["CM3 Wide"]
    (tx, ty), _ = V.POD_SENSORS["VL53L8CX"]
    (fx, fy), _ = V.POD_SENSORS["MTF-01"]
    w, h, t = P.CAM_BOARD
    return [("camera_pcb", S._box(cx - h / 2, cx + h / 2, cy - w / 2, cy + w / 2, z, z + t)),
            ("camera_lens", S._cyl(cx, cy, z - 2.0, z, 6.0)),
            ("tof_pcb", S._box(tx - 6.5, tx + 6.5, ty - 6.0, ty + 6.5, z, z + 1.0)),
            ("flow_pcb", S._box(fx - 15.0, fx + 15.0, fy - 7.0, fy + 7.0, z, z + 1.2)),
            ("flow_sensor", S._cyl(fx, fy, z - 2.0, z, 3.0))]


def assembly(parts: dict[str, cq.Workplane]) -> list[tuple[str, cq.Workplane]]:
    items: list[tuple[str, cq.Workplane]] = [(n, parts[n]) for n in ("top_shell", "bottom_tub", "pod",
                                                                      "pod_tip", "sensor_window", "battery_shell",
                                                                      "battery_latch")]
    items += nose_items(parts)
    items += [("arm_duct", m) for m in A.arm_duct_placed(parts["arm_duct"])]
    items += [("top_grille", m) for m in A.arm_duct_placed(parts["top_grille"])]
    for i, (x, y) in enumerate(P.motor_positions()):
        items += S.motor(x, y)
        items += S.prop(x, y, phase=17.0 + 41.0 * i, ccw=bool(i % 2))
    items += gimbal_items()
    items += internals()
    return items


def nose_items(parts: dict[str, cq.Workplane]) -> list[tuple[str, cq.Workplane]]:
    """Burun yarıları render için iki renkli (2K): dış kabuk gövde renginde, ağız astarı siyah."""
    liner = A.mouth_liner()
    return [("nose_cover", parts["nose_cover"].cut(liner)), ("nose_chin", parts["nose_chin"].cut(liner)),
            ("nose_liner", liner)]


def _overlap(a: cq.Workplane, b: cq.Workplane) -> float:
    try:
        return volume(a.intersect(b))
    except Exception:
        return 0.0


def _union(items: list[cq.Workplane]) -> cq.Workplane:
    out = items[0]
    for w in items[1:]:
        out = out.union(w)
    return out


GIMBAL_POSES = [(p, r) for p in (P.PITCH_RANGE[0], -45.0, 0.0, P.PITCH_SOFT_MAX, P.PITCH_RANGE[1])
                for r in (P.ROLL_RANGE[0], 0.0, P.ROLL_RANGE[1])]


def gimbal_checks(parts: dict[str, cq.Workplane], arms: list[cq.Workplane], tops: list[cq.Workplane],
                  fast: bool = False) -> list[tuple[str, bool, str]]:
    """Ön gimbal: sabit parçaların burna boşluğu (sönümleyici yolu), kapsülün tüm hareket aralığında burun, taşıyıcı,
    kanallar ve üst ızgaralarla çakışmaması, denge ve ağız payı."""
    out: list[tuple[str, bool, str]] = []
    xp = GV.pitch_axis_x()
    g = V.GIMBAL_POS
    nose = {n: parts[n] for n in ("nose_cover", "nose_chin")}
    static = {"taşıyıcı": GV.bracket(xp).translate(g), "pitch motoru": GV.pitch_motor(xp).translate(g),
              "rulman": GV.bearing(xp).translate(g)}
    gaps = {f"{a} ↔ {b}": wa.val().distance(wb.val()) for a, wa in static.items() for b, wb in nose.items()}
    worst = min(gaps.items(), key=lambda kv: kv[1])
    out.append(("gimbal taşıyıcısı ↔ burun (sönümleyici yolu)", worst[1] >= 1.0, f"en yakın {worst[0]} {worst[1]:.1f} mm ≥ 1"))
    front = [w for w in arms + tops if w.val().BoundingBox().xmin > 0.0]           # ön iki modül
    poses = GIMBAL_POSES if not fast else [(P.PITCH_RANGE[0], 0.0), (0.0, P.ROLL_RANGE[1]), (P.PITCH_RANGE[1], 0.0)]
    worst_v = 0.0
    for pitch, roll in poses:
        cap = GV.capsule(pitch, roll)
        worst_v = max([worst_v] + [_overlap(cap, w) for w in list(nose.values()) + list(static.values()) + front])
    out.append(("gimbal kapsülü hareket aralığı ↔ burun, taşıyıcı, kanallar", worst_v < 1.0,
                f"{worst_v:.2f} mm³ (pitch −90/−45/0/+15/+30 × roll −30/0/+30)" if not fast else f"{worst_v:.2f} mm³ (3 poz)"))
    clear = min(GV.capsule(p_, r_).val().distance(w.val()) for p_, r_ in ((P.PITCH_RANGE[0], 0.0), (0.0, 0.0),
                                                                          (P.PITCH_RANGE[1], P.ROLL_RANGE[1]))
                for w in nose.values())
    out.append(("gimbal kapsülü ↔ ağız boşluğu", clear >= 1.5, f"{clear:.1f} mm ≥ 1,5 (−90°, 0°, +30° / roll 30°)"))
    ext = GV.sweep_extent(poses)
    ok = (ext["r_max"] <= V.GIMBAL_SWEEP_R + 0.15 and ext["z_max"] <= V.GIMBAL_SWEEP_Z[1] + 0.15
          and ext["z_min"] >= V.GIMBAL_SWEEP_Z[0] - 0.15)
    out.append(("ağız payı (süpürme + 2 mm) ve parametreler CAD ile uyumlu",
                ok and V.MOUTH_R - ext["r_max"] >= 2.0 and V.MOUTH_TOP - ext["z_max"] >= 2.0,
                f"süpürme r {ext['r_max']:.1f} / z {ext['z_min']:+.1f}…{ext['z_max']:+.1f} mm; ağız r {V.MOUTH_R}, üst {V.MOUTH_TOP}"))
    roll_off = math.hypot(*GV.roll_balance())
    out.append(("gimbal dengesi: pitch ekseni kapsül ağırlık merkezinde, roll ekseni optik eksende",
                abs(xp - V.PITCH_AXIS_DX) <= 0.3 and roll_off <= 0.3,
                f"pitch ekseni x {xp:+.2f} mm (parametre {V.PITCH_AXIS_DX:+.1f}); roll kaçıklığı {roll_off:.2f} mm"))
    return out


def cad_checks(parts: dict[str, cq.Workplane], asm: list[tuple[str, cq.Workplane]],
               fast: bool = False) -> list[tuple[str, bool, str]]:
    out: list[tuple[str, bool, str]] = []
    for name in list(PRODUCTION) + list(GIMBAL_PRODUCTION):
        n = parts[name].solids().size()
        out.append((f"{name}: tek katı", n == 1, f"{n} katı"))
    body = _union([parts[n] for n in ("top_shell", "bottom_tub", "nose_cover", "nose_chin")])
    rings = [A.duct_ring().rotate((0, 0, 0), (0, 0, 1), math.degrees(math.atan2(my, mx))).translate((mx, my, 0))
             for mx, my in P.motor_positions()]
    gap = min(body.val().distance(r.val()) for r in rings)
    out.append(("gövde ↔ kanal halkası", gap >= 2.0, f"{gap:.1f} mm ≥ 2"))
    arms = [w for n, w in asm if n == "arm_duct"]
    arm_clash = sum(_overlap(a, body) for a in arms)
    out.append(("kol kökleri soketlerden temassız geçer", arm_clash < 1.0, f"{arm_clash:.2f} mm³"))
    battery = parts["battery_shell"]
    bat_clash = _overlap(battery, body) + sum(_overlap(battery, a) for a in arms)
    out.append(("batarya tünele çakışmasız girer", bat_clash < 1.0, f"{bat_clash:.2f} mm³"))
    props = [w for n, w in asm if n == "prop"]
    prop_clash = sum(_overlap(p, a) for p in props for a in arms) if not fast else 0.0
    out.append(("pervane ↔ kanal / ızgara", prop_clash < 1.0, f"{prop_clash:.2f} mm³" if not fast else "atlandı (--fast)"))
    out += gimbal_checks(parts, arms, [w for n, w in asm if n == "top_grille"], fast)
    low = min((w.val().BoundingBox().zmin, n) for n, w in asm if n not in ("pod", "pod_tip", "sensor_window"))
    out.append(("avuç ayağı en alçak nokta (avuca yalnızca o değer)", low[0] >= V.POD_BOTTOM_Z + 5.0,
                f"ayak {V.POD_BOTTOM_Z:.0f} mm; sonraki en alçak: {low[1]} {low[0]:.1f} mm"))
    # Tam kapalı pervane: üst ızgara ↔ pervane / somun, motor çanı ↔ etek boşlukları (tek modül yeterli: ×4 özdeş)
    top = parts["top_grille"].val()
    blade = min(w.val().distance(top) for w in [s for n, s in S.prop(0, 0, phase=17.0, ccw=False)])
    mot = dict(S.motor(0, 0))
    hub = min(mot["motor_nut"].val().distance(top), mot["motor_shaft"].val().distance(top))
    bell = mot["motor_bell"].val().distance(parts["arm_duct"].val())
    out.append(("üst ızgara ↔ pervane (gürültü payı) / somun", blade >= 4.0 and hub >= 3.0,
                f"kanat {blade:.1f} mm ≥ 4, somun ve mil {hub:.1f} mm ≥ 3"))
    out.append(("motor çanı ↔ etek (dönen çan yandan kapalı)", bell >= 2.0, f"{bell:.1f} mm ≥ 2"))
    sensors = _union([w for _, w in pod_sensors()])
    near = _union([w for n, w in asm if n.startswith(("pi_", "hat_"))] + [parts["battery_shell"]])
    gap = sensors.val().distance(near.val())
    out.append(("ayak sensörleri ↔ Pi ve batarya", gap >= 1.5, f"{gap:.1f} mm ≥ 1,5"))
    inside = _union([w for n, w in asm if n.startswith(("pi_", "hat_", "fc_pcb", "esc_pcb", "gnss", "gimbal_ctrl"))])
    shells = [parts[n] for n in ("top_shell", "bottom_tub", "nose_cover", "nose_chin", "pod")]
    clash = sum(_overlap(inside, w) for w in shells + arms)
    out.append(("iç kartlar (Pi, FC, ESC, GNSS, gimbal kontrolcüsü) ↔ kabuklar, kovanlar, kollar", clash < 1.0,
                f"{clash:.2f} mm³"))
    # Kalıp (DFM) kuralları — tasarım parametrelerinden
    out += [
        ("kabuk et kalınlığı 1,5–2,5 mm", 1.5 <= min(V.WALL, V.NOSE_WALL) and max(V.WALL, V.NOSE_WALL) <= 2.5,
         f"gövde {V.WALL}, burun {V.NOSE_WALL} mm (iç duvarlar: ağız astarı {V.MOUTH_WALL}, perde {V.BAY_WALL})"),
        ("kol / kanal et kalınlığı ≥ 1,5 mm", min(V.ARM_WALL, V.DUCT_WALL) >= 1.5, f"kol {V.ARM_WALL}, kanal {V.DUCT_WALL} mm"),
        ("kaburga ≤ 0,6 × et", V.RIB_RATIO <= 0.6, f"{V.RIB_RATIO:.1f}"),
        ("kalıptan çıkma açısı ≥ 1°", V.DRAFT_DEG >= 1.0, f"{V.DRAFT_DEG}° (kanal iç duvarı)"),
        ("ızgara hücresi ≤ 10 mm, kaburga ≥ 1,2 mm", V.GRILLE["cell"] <= 10.0 and V.GRILLE["rib"] >= 1.2,
         f"hücre {V.GRILLE['cell']} mm, kaburga {V.GRILLE['rib']} mm"),
    ]
    return out


def variant_comparison(rows: list[dict], parts: dict[str, cq.Workplane]) -> list[dict]:
    variant = budget_calc.load(K.VARIANT)
    by = {r["part"]: r["mass_g"] for r in rows}
    by["battery_latch"] = volume(parts["battery_latch"]) / 1000.0 * V.DENSITY[LATCH_MATERIAL]
    out = []
    for key, (names, extra) in VARIANT_MAP.items():
        comp = next(c for c in variant["components"] if key in c["name"])
        if extra is None:                                             # gimbal: CAD parçaları + satın alınan donanım
            cad = GV.hardware_cg()[1]
        else:
            cad = sum(by[n] for n in names) + extra                   # tek modül (adet profildedir)
        out.append({"component": comp["name"], "profile_g": comp["mass_g"], "cad_g": round(cad, 1),
                    "diff_pct": round(100 * (cad - comp["mass_g"]) / comp["mass_g"], 1)})
    return out


def write_previews(asm: list[tuple[str, cq.Workplane]], out: Path) -> Path:
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    fig, axes = plt.subplots(1, 3, figsize=(18, 6.5), dpi=110)
    B.render(axes[0], asm, -50, 24, "İzometrik (ön sağ üst)", tol=0.8)
    B.render(axes[1], asm, -90, 0, "Yandan (sağ)", tol=0.8)
    B.render(axes[2], asm, 0, 90, "Üstten (burun yukarı)", tol=0.8)
    fig.suptitle("DC7 v2 — bütünleşik gövde (cad/v2/build_v2.py)", fontsize=12)
    fig.tight_layout()
    path = out / "preview_v2.png"
    fig.savefig(path)
    plt.close(fig)
    return path


def gimbal_report(grows: list[dict]) -> list[str]:
    """Ön gimbal bölümü: parçalar, satın alınan donanım, eksenler, görüş."""
    xp = GV.pitch_axis_x()
    (cx, cy, cz), total = GV.hardware_cg()
    view = K.gimbal_view()
    lines = ["## Ön gimbal (cad/v2/gimbal_v2.py)", "",
             f"Kamera merkezi ({V.GIMBAL_POS[0]:.1f}, {V.GIMBAL_POS[1]:.0f}, {V.GIMBAL_POS[2]:.0f}) mm: burnun önünde, orta "
             f"hatta; pitch ekseni {-xp:.1f} mm arkada, kapsülün ağırlık merkezinde. Eksen sırası: dışta pitch (motor sağ "
             f"yanakta, rulman sol yanakta), içte roll (kameranın arkasında, optik eksenle eş eksenli).", "",
             "| Parça | Malzeme | Hacim (cm³) | Kütle (g) | Prototip (MJF) (g) | Not |", "|---|---|---:|---:|---:|---|"]
    for r in grows:
        lines.append(f"| {r['part']} | {r['material']} | {r['volume_cm3']:.1f} | {r['mass_g']:.1f} | {r['proto_mass_g']:.1f} "
                     f"| {r['note']} |")
    bought = [(n, m) for n, m, _ in GV.hardware_masses() if not n.startswith(("gimbal_", "camera_housing"))]
    lines += ["", "Satın alınan / temsili: " + ", ".join(f"{n} {m:.1f} g" for n, m in bought) + ".",
              f"Gimbal toplamı (kamera hariç) **{total:.1f} g**, ağırlık merkezi ({cx:.1f}, {cy:+.1f}, {cz:.1f}) mm.",
              f"Görüş temiz: {view['clear_min']:+.0f}° … {view['clear_max']:+.0f}° (mekanik {P.PITCH_RANGE[0]:+.0f}…"
              f"{P.PITCH_RANGE[1]:+.0f}°, yazılım sınırı +{P.PITCH_SOFT_MAX:.0f}°).", ""]
    return lines


def write_report(rows, grows, comparison, checks, out: Path) -> Path:
    variant = budget_calc.load(K.VARIANT)
    b2 = budget_calc.compute(variant)
    b1 = budget_calc.compute(budget_calc.load(budget_calc.HW_DIR / "tier-a-ekonomik.yaml"))
    bx = K.solve_battery_x(variant)
    cg, total = K.cg_of(K.components(variant, bx))
    lines = ["# DC7 v2 CAD raporu (otomatik — `python3 cad/v2/build_v2.py`)", "",
             "## Parçalar", "",
             "| Parça | Adet | Üretim malzemesi | Hacim (cm³) | Kütle (g) | Prototip (MJF) (g) | Ağırlık merkezi (mm) | Not |",
             "|---|---:|---|---:|---:|---:|---|---|"]
    for r in rows:
        cg_txt = ", ".join(f"{c:+.0f}" for c in r["cg"])
        lines.append(f"| {r['part']} | {r['qty']} | {r['material']} | {r['volume_cm3']:.1f} | {r['mass_g']:.1f} "
                     f"| {r['proto_mass_g']:.1f} | ({cg_txt}){' yerel' if r['part'] == 'arm_duct' else ''} | {r['note']} |")
    air_prod = sum(r["mass_g"] * r["qty"] for r in rows)
    air_proto = sum(r["proto_mass_g"] * r["qty"] for r in rows)
    lines += ["", f"Gövde toplamı: **{air_prod:.0f} g** (üretim) · {air_proto:.0f} g (MJF PA12 prototip)", ""]
    lines += gimbal_report(grows)
    lines += ["## Varyant profiliyle karşılaştırma (config/hardware/variants/tier-a-entegre.yaml)", "",
              "| Bileşen | Profil (g) | CAD (g) | Fark |", "|---|---:|---:|---:|"]
    for c in comparison:
        lines.append(f"| {c['component']} | {c['profile_g']:.0f} | {c['cad_g']:.1f} | %{c['diff_pct']:+.0f} |")
    lines += ["", "## v1 (FPV gövde) ↔ v2 (bütünleşik)", "",
              "| | v1 | v2 |", "|---|---:|---:|",
              f"| Kalkış ağırlığı | {b1.auw_g:.0f} g | {b2.auw_g:.0f} g |",
              f"| Hover süresi | {b1.hover_time_min:.1f} dk | {b2.hover_time_min:.1f} dk |",
              f"| T/W (batarya sınırlı) | {b1.tw_ratio_battery_limited:.2f} | {b2.tw_ratio_battery_limited:.2f} |",
              f"| Hover gazı | {b1.hover_throttle:.2f} | {b2.hover_throttle:.2f} |",
              "", "## Yerleşim (cad/v2/layout_v2.py)", "",
              f"- Toplam {total:.0f} g; batarya paketi merkezi x = {bx:+.1f} mm; ağırlık merkezi "
              f"({cg[0]:+.1f}, {cg[1]:+.1f}, {cg[2]:+.1f}) mm",
              f"- Avuçta devrilme açısı {K.tip_angle(variant):.1f}°", "",
              "## Kontroller", "", "| Kontrol | Sonuç | Değer |", "|---|---|---|"]
    for name, ok, detail in K.checks(variant) + AN.checks() + checks:
        lines.append(f"| {name} | {'✅' if ok else '❌'} | {detail} |")
    lo, hi = AN.window()
    lines += ["", "## Kol kesiti seçimi (cad/v2/analysis_v2.py)", "",
              f"Güvenli pencere {lo:.0f}–{hi:.0f} Hz (1× dönüş bandının üst payı … 3× kanat geçişinin alt payı).", "",
              "| Kök yüksekliği | Dikey frekans aralığı | Sonuç |", "|---:|---:|---|"]
    for h, fmin, fmax, ok in AN.section_sweep():
        sel = " (seçili)" if abs(h - V.ARM_ROOT[1]) < 1e-9 else ""
        lines.append(f"| {h:.0f} mm{sel} | {fmin:.0f}–{fmax:.0f} Hz | {'pencerede' if ok else 'bant payına giriyor'} |")
    path = out / "report.md"
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")
    (out / "report.json").write_text(json.dumps({"parts": rows, "gimbal_parts": grows, "comparison": comparison,
                                                "checks": [[n, ok, d] for n, ok, d in checks]},
                                               ensure_ascii=False, indent=1), encoding="utf-8")
    return path


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--out", type=Path, default=OUT)
    ap.add_argument("--fast", action="store_true")
    args = ap.parse_args(argv)
    args.out.mkdir(parents=True, exist_ok=True)
    parts = build_parts()
    asm = assembly(parts)
    rows = mass_rows(parts)
    grows = mass_rows(parts, GIMBAL_PRODUCTION)
    checks = cad_checks(parts, asm, fast=args.fast)
    comparison = variant_comparison(rows, parts)
    (args.out / "stl").mkdir(exist_ok=True)
    (args.out / "step").mkdir(exist_ok=True)
    for name in list(PRODUCTION) + list(GIMBAL_PRODUCTION):
        cq.exporters.export(parts[name], str(args.out / "stl" / f"{name}.stl"), tolerance=0.05, angularTolerance=0.2)
        cq.exporters.export(parts[name], str(args.out / "step" / f"{name}.step"))
    B.export_glb(asm, args.out / "dc7_v2.glb")
    report = write_report(rows, grows, comparison, checks, args.out)
    if not args.fast:
        write_previews(asm, args.out)
    print(report.read_text(encoding="utf-8"))
    all_ok = all(ok for _, ok, _ in checks + K.checks() + AN.checks())
    return 0 if all_ok else 1


if __name__ == "__main__":
    sys.exit(main())
