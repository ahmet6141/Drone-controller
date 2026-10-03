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
import gimbal_2axis as G  # noqa: E402
import layout_v2 as K  # noqa: E402
import params as P  # noqa: E402
import standins as S  # noqa: E402
import v2_params as V  # noqa: E402

OUT = HERE / "out"
# parça → (üretim malzemesi, adet, kalıp/üretim notu)
PRODUCTION = {
    "top_shell": ("PC/ABS", 1, "kanopi; yarım kol soketleri ayrım düzlemine açık; kanopi emiş yarıkları dikey (maçasız)"),
    "bottom_tub": ("PC/ABS", 1, "taşıyıcı: V uçlu kol soketleri, batarya rayları, vida kuleleri; yan yarıklar için 2 kayar maça"),
    "nose_cover": ("PC/ABS", 1, "saten-mat siyah; gimbal başlığı (tavan) + arka perde, yanaksız: gimbal altta açıkta"),
    "arm_duct": ("PA6-GF30", 4, "TEK kalıp × 4; U kesit kol + çan ağızlı kanal + bal peteği ızgara + 3 radyal kaburga; göbekten yolluk"),
    "pod": ("PC/ABS", 1, "avuç ayağı; sensör tablası (tabandan 25 mm)"),
    "pod_tip": ("TPU", 1, "avuca değen uç (TPU 95A; seri üretimde ayağın üstüne ikinci enjeksiyon)"),
    "battery_shell": ("PC/ABS", 1, "akıllı batarya kabuğu + kuyruk kapağı (gövde çizgisini tamamlar)"),
}
PROTOTYPE_MATERIAL = {"TPU": "TPU"}                 # diğerleri MJF PA12 ile basılır
# Varyant profilindeki bileşen adı → (CAD parçaları, CAD dışı ek kütle g)
VARIANT_MAP = {
    "Üst kabuk": (["top_shell"], 0.0), "Alt kabuk": (["bottom_tub"], 0.0), "Burun kapağı": (["nose_cover"], 0.0),
    "Kol–kanal modülü": (["arm_duct"], 0.0), "Avuç ayağı": (["pod", "pod_tip"], 0.0),
    # kabuk + kilit düğmeleri (POM) + konnektör, BMS/yakıt göstergesi kartı, yaylar ≈ 10 g
    "Akıllı batarya kabuğu": (["battery_shell", "battery_latch"], 10.0),
}
LATCH_MATERIAL = "POM"


def volume(wp: cq.Workplane) -> float:
    return sum(s.Volume() for s in wp.solids().vals())


def build_parts() -> dict[str, cq.Workplane]:
    parts = A.body_parts_detailed()
    parts["arm_duct"] = A.arm_duct()
    parts["pod"] = A.pod()
    parts["pod_tip"] = A.pod_tip()
    bat = A.battery_parts()
    parts["battery_shell"] = bat["battery_shell"]
    parts["battery_latch"] = bat["battery_latch"]
    return parts


def center_of_mass(wp: cq.Workplane) -> tuple[float, float, float]:
    c = cq.Shape.centerOfMass(cq.Compound.makeCompound(wp.solids().vals()))
    return (round(c.x, 1), round(c.y, 1), round(c.z, 1))


def mass_rows(parts: dict[str, cq.Workplane]) -> list[dict]:
    rows = []
    for name, (mat, qty, note) in PRODUCTION.items():
        v = volume(parts[name]) / 1000.0
        proto = PROTOTYPE_MATERIAL.get(mat, "PA12 (MJF)")
        rows.append({"part": name, "qty": qty, "material": mat, "volume_cm3": round(v, 2),
                     "mass_g": round(v * V.DENSITY[mat], 1), "proto_material": proto,
                     "proto_mass_g": round(v * V.DENSITY[proto], 1), "solids": parts[name].solids().size(),
                     "cg": center_of_mass(parts[name]), "note": note})
    return rows


def _with_gimbal_pos(fn):
    old = P.GIMBAL_POS
    P.GIMBAL_POS = V.GIMBAL_POS
    try:
        return fn()
    finally:
        P.GIMBAL_POS = old


def gimbal_items(y_roll: float) -> list[tuple[str, cq.Workplane]]:
    """v1 gimbal parçaları burun bölmesinde (taşıyıcı kol yok; sönümleyiciler bölme tavanına)."""
    items = _with_gimbal_pos(lambda: [(n, s) for n, s in G.placed(y_roll) if n.startswith("gimbal_")
                                      and n != "gimbal_boom"])
    details = _with_gimbal_pos(lambda: S.gimbal_details(y_roll))
    items += [(n, s) for n, s in details if n != "damper"]
    items += [(n, w.translate(V.GIMBAL_POS)) for n, w in A.camera_head().items()]
    g = V.GIMBAL_POS
    cx, cy = G.damper_center(y_roll)
    dx, dy = P.DAMPER_SPACING
    z_top = g[2] + G.top_plate_top_z(y_roll)
    zc = (z_top + V.GIMBAL_MOUNT_Z) / 2
    rad = (V.GIMBAL_MOUNT_Z - z_top) / 2 + 0.3
    for sx in (-1, 1):
        for sy in (-1, 1):
            items.append(("damper", cq.Workplane("XY").sphere(rad)
                          .translate((g[0] + cx + sx * dx / 2, g[1] + cy + sy * dy / 2, zc))))
    return items


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
    # Ayak tabanı: CM3 Wide (ortada), VL53L8CX (+y), MTF-01 (−y) — sensör tablasının üstünde
    z = P.GRIP_BOTTOM_Z + P.SENSOR_RECESS + 2.0
    w, h, t = P.CAM_BOARD
    items += [("camera_pcb", S._box(-h / 2, h / 2, -w / 2, w / 2, z, z + t)),
              ("camera_lens", S._cyl(0, 0, z - 2.0, z, 6.0)),
              ("camera_glass", S._cyl(0, 0, z - 2.2, z - 2.0, 4.0)),
              ("tof_pcb", S._box(-6.5, 6.5, 11.5, 24.5, z, z + 1.0)),
              ("flow_pcb", S._box(-20.0, 10.0, -26.0, -12.0, z, z + 1.2)),
              ("flow_sensor", S._cyl(-5.0, -19.0, z - 2.0, z, 3.0))]
    # Seyir lambaları: motor yuvalarının dış yüzünde (ızgaranın altında; önden ve alttan görünür)
    for (mx, my) in P.motor_positions():
        ang = math.atan2(my, mx)
        r = math.hypot(mx, my) + V.MOTOR_POD[0] - 0.5
        color = "led_white" if mx < 0 else ("led_red" if my > 0 else "led_green")
        led = (cq.Workplane("XY").box(3.0, 10.0, 4.0).rotate((0, 0, 0), (0, 0, 1), math.degrees(ang))
               .translate((r * math.cos(ang), r * math.sin(ang), P.HUB_T - 6.0)))
        items.append((color, led))
    return items


def assembly(parts: dict[str, cq.Workplane], y_roll: float) -> list[tuple[str, cq.Workplane]]:
    items: list[tuple[str, cq.Workplane]] = [(n, parts[n]) for n in ("top_shell", "bottom_tub", "nose_cover", "pod",
                                                                      "pod_tip", "battery_shell", "battery_latch")]
    items += [("arm_duct", m) for m in A.arm_duct_placed(parts["arm_duct"])]
    for i, (x, y) in enumerate(P.motor_positions()):
        items += S.motor(x, y)
        items += S.prop(x, y, phase=17.0 + 41.0 * i, ccw=bool(i % 2))
    items += gimbal_items(y_roll)
    items += internals()
    return items


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


def cad_checks(parts: dict[str, cq.Workplane], asm: list[tuple[str, cq.Workplane]], y_roll: float,
               fast: bool = False) -> list[tuple[str, bool, str]]:
    out: list[tuple[str, bool, str]] = []
    for name in PRODUCTION:
        n = parts[name].solids().size()
        out.append((f"{name}: tek katı", n == 1, f"{n} katı"))
    body = _union([parts["top_shell"], parts["bottom_tub"], parts["nose_cover"]])
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
    gim = [w for n, w in asm if n.startswith(("gimbal_", "camera_housing", "camera_bezel"))
           or n in ("camera_pcb", "camera_lens", "gimbal_motor")]
    gim = [w for w in gim if w.val().BoundingBox().xmin > V.NOSE_SPLIT_X - 60]
    nose_clash = sum(_overlap(g, parts["nose_cover"]) for g in gim)
    out.append(("gimbal ↔ burun kapağı (nötr)", nose_clash < 1.0, f"{nose_clash:.2f} mm³"))
    if not fast:
        g = V.GIMBAL_POS
        worst = 0.0
        head = _union(list(A.camera_head().values()))
        moving = G.gimbal_cradle().union(G.camera()).union(head)
        group = G.roll_group(y_roll).union(head)
        static = G.gimbal_top(y_roll).translate(g)
        arm = G.gimbal_roll_arm(y_roll).translate(g)
        for pitch in (P.PITCH_RANGE[0], -45.0, 0.0, P.PITCH_RANGE[1]):
            rot = moving.rotate((0, 0, 0), (0, 1, 0), -pitch).translate(g)
            worst = max(worst, _overlap(rot, parts["nose_cover"]), _overlap(rot, parts["bottom_tub"]),
                        _overlap(rot, static), _overlap(rot, arm))
        for roll in P.ROLL_RANGE:
            rot = group.rotate((0, y_roll, 0), (1, y_roll, 0), roll).translate(g)
            worst = max(worst, _overlap(rot, parts["nose_cover"]), _overlap(rot, static))
        out.append(("gimbal + kamera başlığı hareket aralığı ↔ burun ve gimbal gövdesi", worst < 1.0,
                    f"{worst:.2f} mm³ (pitch −90/−45/0/+30, roll ±30)"))
    low = min((w.val().BoundingBox().zmin, n) for n, w in asm if n not in ("pod", "pod_tip"))
    out.append(("avuç ayağı en alçak nokta (avuca yalnızca o değer)", low[0] >= P.GRIP_BOTTOM_Z + 5.0,
                f"ayak {P.GRIP_BOTTOM_Z:.0f} mm; sonraki en alçak: {low[1]} {low[0]:.1f} mm"))
    inside = _union([w for n, w in asm if n.startswith(("pi_", "hat_", "fc_pcb", "esc_pcb", "gnss"))])
    shells = [parts[n] for n in ("top_shell", "bottom_tub", "nose_cover", "pod")]
    clash = sum(_overlap(inside, w) for w in shells + arms)
    out.append(("iç kartlar (Pi, FC, ESC, GNSS) ↔ kabuklar, kovanlar, kollar", clash < 1.0, f"{clash:.2f} mm³"))
    # Kalıp (DFM) kuralları — tasarım parametrelerinden
    out += [
        ("kabuk et kalınlığı 1,5–2,5 mm", 1.5 <= V.WALL <= 2.5, f"{V.WALL} mm"),
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
        cad = sum(by[n] for n in names) + extra                       # tek modül (adet profildedir)
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


def write_report(rows, comparison, checks, out: Path) -> Path:
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
    lines += ["", f"Gövde toplamı: **{air_prod:.0f} g** (üretim) · {air_proto:.0f} g (MJF PA12 prototip)", "",
              "## Varyant profiliyle karşılaştırma (config/hardware/variants/tier-a-entegre.yaml)", "",
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
    (out / "report.json").write_text(json.dumps({"parts": rows, "comparison": comparison,
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
    y_roll = G.roll_axis_y()
    asm = assembly(parts, y_roll)
    rows = mass_rows(parts)
    checks = cad_checks(parts, asm, y_roll, fast=args.fast)
    comparison = variant_comparison(rows, parts)
    (args.out / "stl").mkdir(exist_ok=True)
    (args.out / "step").mkdir(exist_ok=True)
    for name in PRODUCTION:
        cq.exporters.export(parts[name], str(args.out / "stl" / f"{name}.stl"), tolerance=0.05, angularTolerance=0.2)
        cq.exporters.export(parts[name], str(args.out / "step" / f"{name}.step"))
    B.export_glb(asm, args.out / "dc7_v2.glb")
    report = write_report(rows, comparison, checks, args.out)
    if not args.fast:
        write_previews(asm, args.out)
    print(report.read_text(encoding="utf-8"))
    all_ok = all(ok for _, ok, _ in checks + K.checks() + AN.checks())
    return 0 if all_ok else 1


if __name__ == "__main__":
    sys.exit(main())
