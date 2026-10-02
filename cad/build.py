#!/usr/bin/env python3
"""DC7 parametrik CAD: tüm basılacak parçaları üretir, STEP/STL dışa aktarır, kütle ve tasarım
raporu yazar, montaj ve parça önizlemeleri çizer.

Gereksinim: pip install cadquery matplotlib   (CadQuery 2.x)
Kullanım:
    python3 cad/build.py                 # cad/out/ altına STEP, STL, PNG ve report.md
    python3 cad/build.py --no-render     # yalnızca dışa aktarım + rapor
"""
from __future__ import annotations

import argparse
import json
import math
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
sys.path.insert(0, str(HERE.parent / "tools"))

import cadquery as cq  # noqa: E402

import budget_calc  # noqa: E402
import gimbal_2axis  # noqa: E402
import layout  # noqa: E402
import palm_grip  # noqa: E402
import params as P  # noqa: E402
import prop_guard  # noqa: E402
import standins  # noqa: E402
import top_deck  # noqa: E402
import visual  # noqa: E402

OUT = HERE / "out"
PRINT_NOTES = {
    "guard_ring": "ağ yüzeyi tablada; destek yok",
    "guard_mount": "plaka tablada; direkler yukarı",
    "grip_tube": "ters: flanş tablada; pencereler ve basamak 45° → destek yok",
    "grip_sensor_mount": "disk tablada",
    "grip_bumper": "TPU 95A; ağız tablada",
    "gimbal_cradle": "arka plaka tablada; ara parçalar yukarı",
    "gimbal_roll_arm": "yan plaka tablada",
    "gimbal_top": "ters: üst plaka tablada",
    "gimbal_boom": "plaka tablada; kaburgalar yukarı",
    "companion_tray": "düz",
    "battery_plate": "düz",
    "companion_shell": "dik bant; yarıklar dikey → destek yok (isteğe bağlı)",
}
# Bütçe karşılaştırması: tier-a-ekonomik.yaml bileşeni → (parçalar, basılmayan ek kütle g, açıklama)
BUDGET_GROUPS = {
    "Tam pervane koruması": (["guard_ring", "guard_mount"], 0.0, "4 takım"),
    "Avuç iniş tutamağı": (["grip_tube", "grip_sensor_mount", "grip_bumper"], 0.0, "köpük ped hariç"),
    "2 eksen fırçasız gimbal": (["gimbal_cradle", "gimbal_roll_arm", "gimbal_top", "gimbal_boom"],
                                2 * P.GIMBAL_MOTOR_MASS_G + 10.0 + 2.0 + 5.0,
                                "+ 2 motor, STorM32, IMU, sönümleyiciler (docs/10 §3.2)"),
    "Üst plaka, bağlantılar": (["companion_tray", "battery_plate", "companion_shell"], 0.0,
                               "kabuk dahil; kalan pay kablolama ve bağlantılar için"),
}
COLORS = {
    "guard_ring": "#3b6ea5", "guard_mount": "#2f4f6f", "grip_tube": "#d98c2b", "grip_sensor_mount": "#9a5b13",
    "grip_bumper": "#444444", "gimbal_cradle": "#6aa84f", "gimbal_roll_arm": "#38761d", "gimbal_top": "#274e13",
    "gimbal_boom": "#7f6000", "companion_tray": "#8e7cc3", "battery_plate": "#674ea7",
    "companion_shell": "#b4a7d6",
}


def preview_color(name: str) -> str:
    """Önizleme rengi: basılan parçalar ayırt edici renklerde; temsili parçalar malzeme renginin açığı."""
    if name in COLORS:
        return COLORS[name]
    r, g, b = visual.rgb(visual.MATERIALS[visual.material_for(name)]["color"])
    lift = lambda c: min(1.0, 0.35 + 0.75 * c)                       # noqa: E731
    return "#%02x%02x%02x" % tuple(int(255 * lift(c)) for c in (r, g, b))


def volume(wp: cq.Workplane) -> float:
    return sum(s.Volume() for s in wp.solids().vals())


def build_parts() -> tuple[dict[str, tuple[cq.Workplane, str, int]], float]:
    y_roll = gimbal_2axis.roll_axis_y()
    parts: dict[str, tuple[cq.Workplane, str, int]] = {}
    for module in (prop_guard, palm_grip, top_deck):
        parts.update(module.parts())
    parts.update(gimbal_2axis.parts(y_roll))
    return parts, y_roll


def export(parts: dict, out: Path) -> list[Path]:
    (out / "step").mkdir(parents=True, exist_ok=True)
    (out / "stl").mkdir(parents=True, exist_ok=True)
    files = []
    for name, (wp, _, _) in parts.items():
        step, stl = out / "step" / f"{name}.step", out / "stl" / f"{name}.stl"
        cq.exporters.export(wp, str(step))
        cq.exporters.export(wp, str(stl), tolerance=0.05, angularTolerance=0.2)
        files += [step, stl]
    return files


def mass_table(parts: dict) -> list[dict]:
    rows = []
    for name, (wp, mat, qty) in parts.items():
        v = volume(wp) / 1000.0
        rows.append({"part": name, "material": mat, "qty": qty, "volume_cm3": round(v, 2),
                     "mass_g": round(v * P.DENSITY[mat], 1),
                     "mass_petg_g": round(v * P.DENSITY["PETG"], 1),
                     "solids": len(wp.solids().vals()), "print": PRINT_NOTES.get(name, "")})
    return rows


def budget_comparison(rows: list[dict]) -> list[dict]:
    profile = budget_calc.load(layout.PROFILE)
    by_part = {r["part"]: r for r in rows}
    out = []
    for key, (names, extra, note) in BUDGET_GROUPS.items():
        comp = next(c for c in profile["components"] if key in c["name"])
        model = sum(by_part[n]["mass_g"] * by_part[n]["qty"] for n in names) + extra
        out.append({"component": comp["name"], "budget_g": comp["mass_g"], "model_g": round(model, 1),
                    "diff_pct": round(100.0 * (model - comp["mass_g"]) / comp["mass_g"], 1), "note": note})
    return out


def cad_checks(parts: dict, y_roll: float) -> list[tuple[str, bool, str]]:
    checks = [(f"{n}: tek katı", wp_.solids().size() == 1, f"{wp_.solids().size()} katı")
              for n, (wp_, _, _) in parts.items()]
    inter = gimbal_2axis.interference(y_roll)
    fits, margin = gimbal_2axis.stack_fits(y_roll)
    bx, bz = gimbal_2axis.pitch_balance()
    blocked = prop_guard.blocked_fraction(parts["guard_ring"][0])
    checks += [
        ("gimbal pitch −90…+30° çakışma", inter["pitch"] < 1e-3, f"{inter['pitch']:.3f} mm³"),
        ("gimbal roll ±30° çakışma", inter["roll"] < 1e-3, f"{inter['roll']:.3f} mm³"),
        ("gimbal dikey yığın gövdeye sığıyor", fits, f"pay {margin:.1f} mm"),
        ("pitch dengesi kızak aralığında", math.hypot(bx, bz) <= P.CRADLE_SLOT,
         f"kayma ({bx:+.2f}, {bz:+.2f}) mm ≤ {P.CRADLE_SLOT:.0f}"),
        ("ağın kapattığı pervane alanı", blocked <= 0.25, f"%{100 * blocked:.0f} → installation_factor itki standında ölçülmeli"),
    ]
    return checks


# --- basit 2B ressam (painter) çizici ---------------------------------------------------------
def _triangles(wp: cq.Workplane, tol: float) -> list[tuple]:
    tris = []
    for solid in wp.solids().vals():
        verts, faces = solid.tessellate(tol, 0.5)
        pts = [(v.x, v.y, v.z) for v in verts]
        tris += [(pts[a], pts[b], pts[c]) for a, b, c in faces]
    return tris


def _view(az: float, el: float):
    if el >= 89.9:
        return (0.0, 0.0, 1.0), (0.0, -1.0, 0.0), (1.0, 0.0, 0.0)
    a, e = math.radians(az), math.radians(el)
    cam = (math.cos(e) * math.cos(a), math.cos(e) * math.sin(a), math.sin(e))
    d = tuple(-c for c in cam)
    right = (d[1], -d[0], 0.0)
    n = math.hypot(*right)
    right = tuple(c / n for c in right)
    up = (right[1] * d[2] - right[2] * d[1], right[2] * d[0] - right[0] * d[2], right[0] * d[1] - right[1] * d[0])
    return cam, right, up


def render(ax, items: list[tuple[str, cq.Workplane]], az: float, el: float, title: str, tol: float = 0.6):
    from matplotlib.collections import PolyCollection
    from matplotlib.colors import to_rgb

    cam, right, up = _view(az, el)
    light = (cam[0] + 0.3, cam[1] + 0.2, cam[2] + 0.6)
    ln = math.sqrt(sum(c * c for c in light))
    light = tuple(c / ln for c in light)
    polys, depth, colors = [], [], []
    for name, wp in items:
        base = to_rgb(preview_color(name))
        for a, b, c in _triangles(wp, tol):
            u = (b[0] - a[0], b[1] - a[1], b[2] - a[2])
            v = (c[0] - a[0], c[1] - a[1], c[2] - a[2])
            nrm = (u[1] * v[2] - u[2] * v[1], u[2] * v[0] - u[0] * v[2], u[0] * v[1] - u[1] * v[0])
            nn = math.sqrt(sum(x * x for x in nrm)) or 1.0
            shade = 0.35 + 0.65 * abs(sum(nrm[i] * light[i] for i in range(3)) / nn)
            pts2 = [(sum(p[i] * right[i] for i in range(3)), sum(p[i] * up[i] for i in range(3))) for p in (a, b, c)]
            polys.append(pts2)
            depth.append(sum(sum(p[i] * cam[i] for i in range(3)) for p in (a, b, c)) / 3)
            colors.append(tuple(min(1.0, ch * shade) for ch in base))
    order = sorted(range(len(polys)), key=lambda i: depth[i])
    ax.add_collection(PolyCollection([polys[i] for i in order], facecolors=[colors[i] for i in order],
                                     edgecolors="none", antialiaseds=False))
    xs = [p[0] for poly in polys for p in poly]
    ys = [p[1] for poly in polys for p in poly]
    if xs:
        ax.set_xlim(min(xs) - 5, max(xs) + 5)
        ax.set_ylim(min(ys) - 5, max(ys) + 5)
    ax.set_aspect("equal")
    ax.set_title(title, fontsize=9)
    ax.axis("off")


def assembly(parts: dict, y_roll: float) -> list[tuple[str, cq.Workplane]]:
    """Gövde koordinatında tam montaj: basılan parçalar + satın alınan parçaların temsilleri."""
    items: list[tuple[str, cq.Workplane]] = []
    ring, mount = parts["guard_ring"][0], parts["guard_mount"][0]
    placed = prop_guard.placed(ring, mount)
    items += [("guard_ring" if i % 2 == 0 else "guard_mount", s) for i, s in enumerate(placed)]
    tube, smount, bumper = (parts[n][0] for n in ("grip_tube", "grip_sensor_mount", "grip_bumper"))
    items += list(zip(("grip_tube", "grip_sensor_mount", "grip_bumper"), palm_grip.placed(tube, smount, bumper)))
    items += [(n, s) for n, s in gimbal_2axis.placed(y_roll) if n.startswith("gimbal_")]
    items += list(zip(("companion_tray", "battery_plate", "companion_shell"),
                      top_deck.placed(parts["companion_tray"][0], parts["battery_plate"][0],
                                      parts["companion_shell"][0])))
    items += standins.all_items(y_roll)
    return items


def export_glb(items: list[tuple[str, cq.Workplane]], path: Path) -> Path:
    """Blender / glTF görüntüleyiciler için tek dosya montaj (mm; Blender betiği ölçeği düzeltir)."""
    assy = cq.Assembly(name="dc7")
    counts: dict[str, int] = {}
    for name, wp in items:
        counts[name] = counts.get(name, 0) + 1
        r, g, b = visual.rgb(visual.MATERIALS[visual.material_for(name)]["color"])
        assy.add(wp, name=f"{name}.{counts[name]:03d}", color=cq.Color(r, g, b, 1.0))
    assy.export(str(path), tolerance=0.03, angularTolerance=0.1)
    return path


def write_previews(parts: dict, y_roll: float, out: Path) -> list[Path]:
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    items = assembly(parts, y_roll)
    fig, axes = plt.subplots(1, 3, figsize=(18, 6.5), dpi=110)
    render(axes[0], items, -50, 28, "İzometrik (ön sağ üst)")
    render(axes[1], items, -90, 0, "Yandan (sağ)")
    render(axes[2], items, 0, 90, "Üstten (burun yukarı)")
    fig.suptitle("DC7 — 7 inç, Seviye A yerleşimi (cad/build.py)", fontsize=12)
    fig.tight_layout()
    p1 = out / "preview_assembly.png"
    fig.savefig(p1)
    plt.close(fig)

    names = list(parts)
    cols = 4
    rows_n = math.ceil(len(names) / cols)
    fig, axes = plt.subplots(rows_n, cols, figsize=(16, 4.2 * rows_n), dpi=100)
    rows = {r["part"]: r for r in mass_table(parts)}
    for ax, name in zip(axes.flat, names):
        wp, mat, qty = parts[name]
        r = rows[name]
        render(ax, [(name, wp)], -50, 30, f"{name} ×{qty}\n{mat} {r['mass_g']} g", tol=0.3)
    for ax in list(axes.flat)[len(names):]:
        ax.axis("off")
    fig.tight_layout()
    p2 = out / "preview_parts.png"
    fig.savefig(p2)
    plt.close(fig)
    return [p1, p2]


def write_report(rows: list[dict], budget: list[dict], checks: list, rules: list, y_roll: float, out: Path) -> Path:
    cg, total = layout.center_of_gravity()
    bx, _, _ = layout.battery_position()
    tips = layout.tip_angles()
    lines = ["# DC7 CAD raporu (otomatik — `python3 cad/build.py`)", "",
             "## Parçalar", "",
             "| Parça | Adet | Malzeme | Hacim (cm³) | Kütle (g) | PETG ile (g) | Baskı yönü |",
             "|---|---:|---|---:|---:|---:|---|"]
    for r in rows:
        lines.append(f"| {r['part']} | {r['qty']} | {r['material']} | {r['volume_cm3']:.2f} | {r['mass_g']:.1f} "
                     f"| {r['mass_petg_g']:.1f} | {r['print']} |")
    lines += ["", "## Bütçeyle karşılaştırma (config/hardware/tier-a-ekonomik.yaml)", "",
              "| Bileşen | Bütçe (g) | Model (g) | Fark | Not |", "|---|---:|---:|---:|---|"]
    for b in budget:
        lines.append(f"| {b['component']} | {b['budget_g']:.0f} | {b['model_g']:.1f} | %{b['diff_pct']:+.0f} | {b['note']} |")
    lines += ["", "## Yerleşim", "",
              f"- Toplam kütle {total:.0f} g; ağırlık merkezi ({cg[0]:+.1f}, {cg[1]:+.1f}, {cg[2]:+.1f}) mm",
              f"- Batarya konumu: x = {bx:+.1f} mm (ağırlık merkezini ortalar)",
              f"- Gimbal roll ekseni kamera merkezinden y = {y_roll:+.1f} mm (dönen grubun ağırlık merkezi)",
              f"- Devrilme açısı (motorlar kapalı): tutamakla {tips['tutamak']:.1f}°, "
              f"Ø{P.GROUND_FOOT_D:.0f} mm ayakla {tips['geniş ayak']:.1f}°",
              "- Atalet (CG'ye göre, SITL başlangıcı): Ixx {:.4f}, Iyy {:.4f}, Izz {:.4f} kg·m²".format(*layout.inertia()),
              "", "## Kontroller", "", "| Kontrol | Sonuç | Değer |", "|---|---|---|"]
    for name, ok, detail in rules + checks:
        lines.append(f"| {name} | {'✅' if ok else '❌'} | {detail} |")
    path = out / "report.md"
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")
    (out / "report.json").write_text(json.dumps({"parts": rows, "budget": budget,
                                                 "checks": [[n, ok, d] for n, ok, d in rules + checks]},
                                                ensure_ascii=False, indent=1), encoding="utf-8")
    return path


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--out", type=Path, default=OUT)
    ap.add_argument("--no-render", action="store_true", help="matplotlib önizlemelerini atla")
    ap.add_argument("--no-glb", action="store_true", help="Blender için GLB montajını atla")
    args = ap.parse_args(argv)
    args.out.mkdir(parents=True, exist_ok=True)

    parts, y_roll = build_parts()
    export(parts, args.out)
    if not args.no_glb:
        export_glb(assembly(parts, y_roll), args.out / "dc7_assembly.glb")
    rows = mass_table(parts)
    budget = budget_comparison(rows)
    checks = cad_checks(parts, y_roll)
    rules = layout.design_rules()
    report = write_report(rows, budget, checks, rules, y_roll, args.out)
    if not args.no_render:
        write_previews(parts, y_roll, args.out)
    print(report.read_text(encoding="utf-8"))
    return 0 if all(ok for _, ok, _ in rules + checks) else 1


if __name__ == "__main__":
    sys.exit(main())
