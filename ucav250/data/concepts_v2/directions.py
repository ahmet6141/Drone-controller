"""YK-250 v2 — design-direction study.

Four distinct, modern configurations at the same scale (Limbach L 275 EF 18 kW pusher, MTOM 145 kg, 20 kg EO/IR
payload, ~33 kg fuel), modelled as outer-mould-line (OML) surfaces with ucav250.design.oml and rendered as shaded
Workbench views for a design review. The numbers are PRELIMINARY (about +/-20 %): they only rank the directions; the
chosen direction is sized in detail afterwards with the full chain (analysis/aero.py, sizinglib, stablib).

Run:  PYTHONPATH=. python3 ucav250/data/concepts_v2/directions.py [--only manta,ok,...] [--res 1400x900]
Out:  ucav250/data/concepts_v2/<key>/{iso,rear,top,side,front}.png, sheet.png; ucav250/data/concepts_v2/compare.png,
      directions.yaml
"""
from __future__ import annotations

import argparse
import math
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT))

from ucav250.analysis import aerolib as AL  # noqa: E402
from ucav250.core.geom import Mesh, cylinder, revolve, sphere  # noqa: E402
from ucav250.design.oml import Fuselage, LiftingSurface  # noqa: E402

OUT = Path(__file__).resolve().parent
G0 = 9.80665

# ---------------------------------------------------------------------------------------------------------------------
# common mission/scale (from research baseline + the reviewed 'endurance' reference concept)
# ---------------------------------------------------------------------------------------------------------------------
MTOM = 145.0
FUEL = 33.0
PROP_D = 0.813                     # Mejzlik 32x18 2B (research aero.yaml)
BSFC_LOITER = 0.56                 # kg/kWh at ~25-30 % power (engine.yaml part-load estimate)
ETA_LOITER = 0.78                  # propeller efficiency at loiter (aero.yaml Mejzlik table)
H_LOITER = 3000.0

COL = {"skin": "#D3D7DB", "dark": "#2E3338", "accent": "#646C76", "glass": "#1E2228", "prop": "#1F2226",
       "gear": "#3F454C", "tyre": "#17191B", "turret": "#23272B", "metal": "#8C939B", "ground": "#F3F5F7"}


# =====================================================================================================================
# geometry helpers
# =====================================================================================================================
def smooth(y, pts):
    """Monotone PCHIP through control points [(y, value), ...] evaluated at y."""
    from scipy.interpolate import PchipInterpolator
    P = np.array(pts, float)
    return PchipInterpolator(P[:, 0], P[:, 1])(y)


def surface(sections, refine=1, n_chord=70, te=0.0015) -> Mesh:
    return LiftingSurface(sections, n_chord=n_chord, te_thickness=te).mesh(refine)


def mirror(m: Mesh) -> Mesh:
    return m.mirrored_y()


def wheel(center, r=0.10, w=0.05, axis=(0, 1, 0), n=40) -> Mesh:
    """Tyre as a rounded torus-like solid of revolution about ``axis``."""
    prof = []
    k = 14
    for i in range(k + 1):
        a = -math.pi / 2 + math.pi * i / k
        prof.append((r - 0.5 * w + 0.5 * w * math.cos(a) + 0.0, 0.5 * w * math.sin(a)))
    prof = [(r * 0.45, -0.5 * w)] + [(p[0] + 0.0, p[1]) for p in prof] + [(r * 0.45, 0.5 * w)]
    return revolve(prof, n=n, axis_origin=center, axis=axis)


def ellipsoid(center, a, b, c, n=28) -> Mesh:
    s = sphere(1.0, (0, 0, 0), n)
    return s.transformed(np.diag([a, b, c]), center)


def strut(p0, p1, r=0.012) -> Mesh:
    return cylinder(r, p0, p1, n=16)


def prop(hub, D=PROP_D, blades=2, beta_root=48.0, beta_tip=18.0, phase_deg=20.0) -> list[Mesh]:
    """Pusher propeller: tapered twisted blades in the YZ plane at x = hub[0] plus a spinner (pointing aft)."""
    hub = np.asarray(hub, float)
    R = 0.5 * D
    rs = np.linspace(0.075, R, 7)
    out = []
    for k in range(blades):
        secs = []
        for r in rs:
            f = (r - rs[0]) / (R - rs[0])
            beta = beta_root + (beta_tip - beta_root) * f
            c = 0.075 * (1 - 0.55 * f) + 0.012
            secs.append({"y": 0.0, "x_le": hub[0] - 0.5 * c * math.sin(math.radians(beta)), "z_le": r,
                         "chord": c, "twist_deg": 90.0 - beta, "airfoil": "naca4412",
                         "thickness_scale": 0.9 - 0.5 * f, "span_dir": (0.0, 0.0, 1.0)})
        m = LiftingSurface(secs, n_chord=24, te_thickness=0.0008).mesh()
        ang = math.radians(phase_deg + 360.0 * k / blades)
        m = m.rotated_about((0, 0, 0), (1, 0, 0), ang).translated((0, hub[1], hub[2]))
        out.append(m)
    sp = revolve([(0.0, 0.0), (0.075, 0.0), (0.072, 0.05), (0.055, 0.11), (0.03, 0.16), (0.0, 0.185)], n=40,
                 axis_origin=hub - np.array([0.02, 0, 0]), axis=(1, 0, 0))
    out.append(sp)
    return out


def duct(x0, x1, center_yz, r_in, t_max=0.035, n=64) -> Mesh:
    """Annular propeller duct (shroud) with a cambered section, from x0 to x1."""
    L = x1 - x0
    xs = np.linspace(0, 1, 24)
    thick = t_max * (np.sqrt(np.clip(xs, 0, 1)) * (1 - xs) * 2.6 + 0.05)
    flare = 0.020 * (1 - xs) ** 2                      # bell-mouth inlet lip
    inner = [(r_in - flare[i] * 0.4, x0 + L * xs[i]) for i in range(len(xs))]
    outer = [(r_in + thick[i] + flare[i] * 0.3, x0 + L * xs[i]) for i in range(len(xs))][::-1]
    prof = inner + outer
    return revolve(prof, n=n, axis_origin=(0.0, center_yz[0], center_yz[1]), axis=(1, 0, 0))


def turret(center, r=0.085) -> list[Mesh]:
    c = np.asarray(center, float)
    return [cylinder(0.072, c + (0, 0, 0.02), c + (0, 0, 0.085), n=32), sphere(r, c, 36),
            ellipsoid(c + (-0.06, 0, -0.005), 0.012, 0.035, 0.03)]


# =====================================================================================================================
# direction A — MANTA: blended wing-body, high-AR outer panels, twin canted fins, pusher in the spine
# =====================================================================================================================
def manta() -> dict:
    ys = np.r_[np.linspace(0.0, 0.80, 11), np.linspace(0.92, 3.35, 9)]
    xle = smooth(ys, [(0.0, 0.0), (0.20, 0.30), (0.45, 0.70), (0.65, 1.00), (0.80, 1.14), (1.00, 1.19),
                      (3.35, 1.44)])
    te = smooth(ys, [(0.0, 2.30), (0.25, 2.25), (0.50, 2.05), (0.68, 1.86), (0.80, 1.82), (1.00, 1.82),
                     (3.35, 1.74)])
    zle = np.where(ys < 0.80, 0.0, (ys - 0.80) * math.tan(math.radians(3.5)))
    tw = smooth(ys, [(0.0, 2.0), (0.80, 1.5), (3.35, -1.5)])
    secs = []
    for y, a, b, z, t in zip(ys, xle, te, zle, tw):
        c = b - a
        if y < 0.45:
            af, ts = "naca1418", 1.0
        elif y < 0.78:
            af, ts = "naca2415", 1.0
        else:
            af, ts = "nlf416", 1.0 - 0.19 * (y - 0.78) / 2.57
        secs.append({"y": float(y), "x_le": float(a), "z_le": float(z), "chord": float(c), "twist_deg": float(t),
                     "airfoil": af, "thickness_scale": float(ts)})
    wing_r = surface(secs, refine=2)
    # dorsal spine: blends the engine bay into the centre body and ends in the tail cone + spinner
    spine = Fuselage(np.array([
        [0.05, 0.0, 0.0, 0.08, 2.0, 2.0, 0.5],
        [0.38, 0.30, 0.20, 0.11, 1.5, 1.8, 0.55],
        [1.00, 0.40, 0.30, 0.15, 1.6, 1.8, 0.6],
        [1.85, 0.44, 0.34, 0.14, 1.6, 1.8, 0.6],
        [2.45, 0.34, 0.27, 0.09, 1.8, 1.8, 0.55],
        [2.86, 0.17, 0.16, 0.07, 2.0, 2.0, 0.5],
    ]))
    spine_m = spine.mesh(90, 64)
    # twin outward-canted fins on the centre-body trailing edge (ruddervators)
    cant = math.radians(32.0)
    y0, z0, h = 0.50, 0.05, 0.62
    fin = []
    for s in np.linspace(0, 1, 4):
        fin.append({"y": y0 + s * h * math.sin(cant), "x_le": 1.45 + s * 0.42, "z_le": z0 + s * h * math.cos(cant),
                    "chord": 0.60 - 0.30 * s, "twist_deg": 0.0, "airfoil": "naca0010"})
    fin_r = surface(fin)
    parts = [("wing_r", wing_r, "skin"), ("wing_l", mirror(wing_r), "skin"), ("spine", spine_m, "skin"),
             ("fin_r", fin_r, "accent"), ("fin_l", mirror(fin_r), "accent")]
    for i, m in enumerate(prop((2.95, 0.0, 0.07))):
        parts.append((f"prop{i}", m, "prop"))
    for i, m in enumerate(turret((0.52, 0.0, -0.27))):
        parts.append((f"turret{i}", m, "turret"))
    # retractable tricycle gear (shown extended): nose leg + two mains in the centre body
    gz = -0.80
    parts += [("ng_strut", strut((0.62, 0, -0.15), (0.62, 0, gz + 0.09), 0.016), "gear"),
              ("ng_wheel", wheel((0.62, 0, gz + 0.09), 0.09, 0.045), "tyre")]
    for sgn in (1, -1):
        parts += [(f"mg_strut{sgn}", strut((1.60, sgn * 0.50, -0.08), (1.64, sgn * 0.58, gz + 0.10), 0.02), "gear"),
                  (f"mg_wheel{sgn}", wheel((1.64, sgn * 0.62, gz + 0.10), 0.10, 0.05), "tyre")]
    parts += [("sat_bulge", ellipsoid((0.85, 0.0, 0.29), 0.26, 0.16, 0.05), "accent"),
              ("intake", ellipsoid((1.62, 0.0, 0.31), 0.22, 0.10, 0.035), "dark")]
    S = 2 * np.trapz([s["chord"] for s in secs], [s["y"] for s in secs])
    return {"key": "manta", "name": "MANTA", "name_tr": "Kanat-gövde birleşik (BWB)", "parts": parts,
            "S_ref": S, "span": 2 * ys[-1], "length": 3.07, "ground_z": gz,
            "e": 0.86, "cd_misc": 0.0040, "interf": 1.02, "gear_retract": True,
            "pitch": "Merkez gövde ve dış kanat arka kenarındaki eleron/elevonlar + eğik ikiz dümen-yükseliş (ruddervator)",
            "summary_tr": ("Taşıyıcı merkez gövde (BWB) yüksek açıklıklı dış kanatlara kesintisiz geçer; motor sırttaki "
                           "omurga kaportasında, itici pervane iki dışa eğik dikey kuyruk arasında korunur. Geniş iç hacim: "
                           "yakıt, faydalı yük ve aviyonik ağırlık merkezinin üzerinde; içeri katlanır iniş takımı.")}


# =====================================================================================================================
# direction B — OK (arrow): canard pusher, swept main wing with blended winglet-fins, chined body
# =====================================================================================================================
def ok() -> dict:
    fus = Fuselage(np.array([
        [0.00, 0.0, 0.0, -0.02, 2.0, 2.0, 0.5],
        [0.30, 0.26, 0.22, -0.01, 1.35, 1.25, 0.55],
        [0.90, 0.42, 0.38, 0.00, 1.35, 1.25, 0.58],
        [1.90, 0.46, 0.42, 0.00, 1.35, 1.25, 0.58],
        [2.80, 0.42, 0.38, 0.02, 1.35, 1.30, 0.58],
        [3.45, 0.26, 0.24, 0.05, 1.6, 1.6, 0.55],
        [3.66, 0.14, 0.13, 0.06, 2.0, 2.0, 0.5],
    ]))
    fus_m = fus.mesh(110, 72)
    can = []
    for y, xl, c, z in ((0.15, 0.62, 0.30, 0.13), (0.55, 0.70, 0.25, 0.115), (0.92, 0.79, 0.17, 0.10)):
        can.append({"y": y, "x_le": xl, "z_le": z, "chord": c, "twist_deg": 1.0, "airfoil": "naca2412"})
    can_r = surface(can)
    ys = np.linspace(0.20, 3.10, 9)
    sweep = math.radians(24.0)
    wing = []
    for y in ys:
        f = (y - 0.20) / 2.90
        wing.append({"y": float(y), "x_le": float(2.05 + (y - 0.20) * math.tan(sweep)),
                     "z_le": float(-0.07 + (y - 0.20) * math.tan(math.radians(2.0))),
                     "chord": float(0.70 - 0.38 * f), "twist_deg": float(1.0 - 2.5 * f), "airfoil": "nlf416",
                     "thickness_scale": float(1.0 - 0.19 * f)})
    wing_r = surface(wing)
    # blended winglet-fin (rudder) at the tip: short transition then a near-vertical fin canted 8 deg outboard
    tip = wing[-1]
    wl = []
    for s in np.linspace(0, 1, 5):
        ang = math.radians(90.0 * min(1.0, s * 1.6))
        r = 0.10
        yy = tip["y"] + (r * math.sin(ang) if s < 0.62 else r + (s - 0.62) * 0.55 * math.sin(math.radians(8)))
        zz = tip["z_le"] + (r * (1 - math.cos(ang)) if s < 0.62 else r + (s - 0.62) * 0.55 * math.cos(math.radians(8)))
        wl.append({"y": float(yy), "x_le": float(tip["x_le"] + 0.05 + s * 0.30), "z_le": float(zz),
                   "chord": float(tip["chord"] - 0.10 * s), "twist_deg": 0.0, "airfoil": "naca0010"})
    wl_r = LiftingSurface(wl, n_chord=60).mesh()
    parts = [("fus", fus_m, "skin"), ("canard_r", can_r, "accent"), ("canard_l", mirror(can_r), "accent"),
             ("wing_r", wing_r, "skin"), ("wing_l", mirror(wing_r), "skin"), ("wl_r", wl_r, "accent"),
             ("wl_l", mirror(wl_r), "accent"),
             ("ventral", surface([{"y": 0.0, "x_le": 3.05, "z_le": -0.10, "chord": 0.42, "twist_deg": 0,
                                   "airfoil": "naca0010", "span_dir": (0, 0, -1)},
                                  {"y": 0.0, "x_le": 3.30, "z_le": -0.36, "chord": 0.22, "twist_deg": 0,
                                   "airfoil": "naca0010", "span_dir": (0, 0, -1)}]), "accent")]
    for i, m in enumerate(prop((3.80, 0.0, 0.06))):
        parts.append((f"prop{i}", m, "prop"))
    for i, m in enumerate(turret((0.34, 0.0, -0.22))):
        parts.append((f"turret{i}", m, "turret"))
    gz = -0.66
    parts += [("ng_strut", strut((0.62, 0, -0.15), (0.64, 0, gz + 0.085), 0.015), "gear"),
              ("ng_wheel", wheel((0.64, 0, gz + 0.085), 0.085, 0.045), "tyre"),
              ("ng_pant", ellipsoid((0.64, 0, gz + 0.10), 0.13, 0.04, 0.07), "skin")]
    for sgn in (1, -1):
        parts += [(f"mg_bow{sgn}", strut((2.45, sgn * 0.18, -0.16), (2.50, sgn * 0.62, gz + 0.10), 0.018), "gear"),
                  (f"mg_wheel{sgn}", wheel((2.50, sgn * 0.64, gz + 0.10), 0.10, 0.05), "tyre"),
                  (f"mg_pant{sgn}", ellipsoid((2.50, sgn * 0.64, gz + 0.12), 0.16, 0.045, 0.085), "skin")]
    parts += [("intake", ellipsoid((2.75, 0.0, 0.21), 0.20, 0.09, 0.03), "dark")]
    S = 2 * np.trapz([s["chord"] for s in wing], [s["y"] for s in wing]) + 2 * 0.20 * 0.70
    S_c = 2 * np.trapz([s["chord"] for s in can], [s["y"] for s in can])
    return {"key": "ok", "name": "OK", "name_tr": "Kanard-itici, ok kanat + kanatucu dümen", "parts": parts,
            "S_ref": S, "S_extra": S_c, "span": 2 * (ys[-1] + 0.12), "length": 3.97, "ground_z": gz,
            "e": 0.80, "cd_misc": 0.0040, "interf": 1.06, "gear_retract": False,
            "pitch": "Kanard üzerinde yükseliş dümeni; ana kanatta kanatçık, kanat ucu dikeylerinde dümen",
            "summary_tr": ("Burunda kanard, arkada geriye ok açılı ana kanat ve kanat uçlarından kıvrılarak yükselen "
                           "dikey dümenler: kuyruk yok. Kanard önce perdövites yapar, ana kanat stall'a girmez "
                           "(doğal stall koruması). İtici pervane gövde sonunda, altında ventral koruma kanadı.")}


# =====================================================================================================================
# direction C — HALKA (ring): box / joined wing, rear wing on a dorsal fin, tip fins
# =====================================================================================================================
def halka() -> dict:
    fus = Fuselage(np.array([
        [0.00, 0.0, 0.0, -0.01, 2.0, 2.0, 0.5],
        [0.30, 0.26, 0.22, 0.0, 1.4, 1.3, 0.55],
        [0.90, 0.42, 0.38, 0.0, 1.4, 1.3, 0.56],
        [2.00, 0.44, 0.40, 0.0, 1.4, 1.3, 0.56],
        [2.95, 0.38, 0.34, 0.03, 1.5, 1.4, 0.56],
        [3.50, 0.22, 0.20, 0.05, 1.8, 1.8, 0.5],
        [3.64, 0.13, 0.12, 0.06, 2.0, 2.0, 0.5],
    ]))
    fus_m = fus.mesh(110, 72)
    b2 = 2.75
    fw = []
    for y in np.linspace(0.18, b2, 8):
        f = (y - 0.18) / (b2 - 0.18)
        fw.append({"y": float(y), "x_le": float(1.00 + (y - 0.18) * math.tan(math.radians(27))),
                   "z_le": float(-0.16 + (y - 0.18) * math.tan(math.radians(4.5))), "chord": float(0.58 - 0.24 * f),
                   "twist_deg": float(1.5 - 2.5 * f), "airfoil": "nlf416", "thickness_scale": float(1 - 0.19 * f)})
    fw_r = surface(fw)
    tipF = fw[-1]
    rw = []
    z_root, z_tip = 0.80, 0.56
    for y in np.linspace(0.0, b2, 8):
        f = y / b2
        rw.append({"y": float(y), "x_le": float(3.18 - y * math.tan(math.radians(14.5))),
                   "z_le": float(z_root + (z_tip - z_root) * f), "chord": float(0.46 - 0.12 * f),
                   "twist_deg": float(-1.0 + 1.0 * f), "airfoil": "nlf416", "thickness_scale": 0.85})
    rw_r = surface(rw)
    tipR = rw[-1]
    tf = []
    for s in np.linspace(0, 1, 4):
        tf.append({"y": float(b2 + 0.004), "x_le": float(tipF["x_le"] + s * (tipR["x_le"] - tipF["x_le"])),
                   "z_le": float(tipF["z_le"] + s * (tipR["z_le"] - tipF["z_le"])),
                   "chord": float(tipF["chord"] + s * (tipR["chord"] - tipF["chord"])), "twist_deg": 0.0,
                   "airfoil": "naca0010", "span_dir": (0.0, 0.0, 1.0)})
    tf_r = surface(tf)
    vf = []
    for s in np.linspace(0, 1, 4):
        vf.append({"y": 0.0, "x_le": float(2.82 + s * 0.40), "z_le": float(0.14 + s * (z_root - 0.12)),
                   "chord": float(0.62 - 0.14 * s), "twist_deg": 0.0, "airfoil": "naca0012", "span_dir": (0, 0, 1)})
    vf_m = surface(vf)
    parts = [("fus", fus_m, "skin"), ("fw_r", fw_r, "skin"), ("fw_l", mirror(fw_r), "skin"),
             ("rw_r", rw_r, "skin"), ("rw_l", mirror(rw_r), "skin"), ("tf_r", tf_r, "accent"),
             ("tf_l", mirror(tf_r), "accent"), ("vfin", vf_m, "accent")]
    for i, m in enumerate(prop((3.78, 0.0, 0.04))):
        parts.append((f"prop{i}", m, "prop"))
    for i, m in enumerate(turret((0.36, 0.0, -0.22))):
        parts.append((f"turret{i}", m, "turret"))
    gz = -0.66
    parts += [("ng_strut", strut((0.62, 0, -0.15), (0.64, 0, gz + 0.085), 0.015), "gear"),
              ("ng_wheel", wheel((0.64, 0, gz + 0.085), 0.085, 0.045), "tyre"),
              ("ng_pant", ellipsoid((0.64, 0, gz + 0.10), 0.13, 0.04, 0.07), "skin")]
    for sgn in (1, -1):
        parts += [(f"mg_bow{sgn}", strut((1.92, sgn * 0.18, -0.17), (1.97, sgn * 0.62, gz + 0.10), 0.018), "gear"),
                  (f"mg_wheel{sgn}", wheel((1.97, sgn * 0.64, gz + 0.10), 0.10, 0.05), "tyre"),
                  (f"mg_pant{sgn}", ellipsoid((1.97, sgn * 0.64, gz + 0.12), 0.16, 0.045, 0.085), "skin")]
    S_f = 2 * np.trapz([s["chord"] for s in fw], [s["y"] for s in fw]) + 2 * 0.18 * 0.58
    S_r = 2 * np.trapz([s["chord"] for s in rw], [s["y"] for s in rw])
    return {"key": "halka", "name": "HALKA", "name_tr": "Kutu / birleşik kanat", "parts": parts,
            "S_ref": S_f + S_r, "span": 2 * b2, "length": 3.82, "ground_z": gz,
            "e": 1.00, "cd_misc": 0.0040, "interf": 1.10, "gear_retract": False,
            "pitch": "Arka kanatta yükseliş dümenleri, ön kanatta kanatçık/flap, dikey kuyrukta dümen",
            "summary_tr": ("Alçak, geriye ok açılı ön kanat; sırttaki dikey kuyruğun tepesinden çıkan ileri ok açılı arka "
                           "kanat; kanat uçlarında ikisini birleştiren dikey plakalar (kutu kanat). Aynı açıklıkta indüklenmiş "
                           "sürükleme ~%20 daha düşük; 5,5 m açıklıkla tek parça taşınabilir kanat yarıları.")}


# =====================================================================================================================
# direction D — HANÇER (dagger): chined lifting body with strakes, canted twin tails, ducted pusher
# =====================================================================================================================
def hancer() -> dict:
    fus = Fuselage(np.array([
        [0.00, 0.0, 0.0, -0.02, 2.0, 2.0, 0.5],
        [0.35, 0.34, 0.20, -0.02, 1.25, 1.15, 0.52],
        [1.10, 0.58, 0.34, 0.00, 1.25, 1.15, 0.55],
        [2.20, 0.62, 0.38, 0.00, 1.25, 1.15, 0.55],
        [3.10, 0.52, 0.34, 0.03, 1.35, 1.25, 0.55],
        [3.62, 0.46, 0.30, 0.05, 1.6, 1.6, 0.5],
    ]))
    fus_m = fus.mesh(110, 72)
    ys = np.r_[np.linspace(0.20, 0.62, 5), np.linspace(0.78, 3.15, 8)]
    xle = smooth(ys, [(0.20, 0.95), (0.40, 1.42), (0.62, 1.86), (0.78, 1.95), (3.15, 2.13)])
    te = smooth(ys, [(0.20, 2.62), (0.62, 2.55), (0.78, 2.52), (3.15, 2.42)])
    wing = []
    for y, a, b in zip(ys, xle, te):
        f = max(0.0, (y - 0.78) / 2.37)
        wing.append({"y": float(y), "x_le": float(a), "z_le": float(0.02 + max(0.0, y - 0.62) * 0.04),
                     "chord": float(b - a), "twist_deg": float(1.5 - 3.0 * f),
                     "airfoil": "nlf416" if y >= 0.62 else "naca2412",
                     "thickness_scale": float(1 - 0.19 * f) if y >= 0.62 else 0.85})
    wing_r = surface(wing, refine=2)
    cant = math.radians(28)
    fin = []
    for s in np.linspace(0, 1, 4):
        fin.append({"y": float(0.24 + s * 0.58 * math.sin(cant)), "x_le": float(2.78 + s * 0.38),
                    "z_le": float(0.14 + s * 0.58 * math.cos(cant)), "chord": float(0.62 - 0.30 * s),
                    "twist_deg": 0.0, "airfoil": "naca0010"})
    fin_r = surface(fin)
    hs = []
    for s in np.linspace(0, 1, 3):
        hs.append({"y": float(0.24 + s * 0.62), "x_le": float(3.05 + s * 0.28), "z_le": float(0.02),
                   "chord": float(0.42 - 0.18 * s), "twist_deg": 0.0, "airfoil": "naca0010"})
    hs_r = surface(hs)
    parts = [("fus", fus_m, "skin"), ("wing_r", wing_r, "skin"), ("wing_l", mirror(wing_r), "skin"),
             ("fin_r", fin_r, "accent"), ("fin_l", mirror(fin_r), "accent"), ("hs_r", hs_r, "accent"),
             ("hs_l", mirror(hs_r), "accent"),
             ("duct", duct(3.66, 4.02, (0.0, 0.05), 0.5 * PROP_D + 0.012), "dark")]
    for i, m in enumerate(prop((3.84, 0.0, 0.05))):
        parts.append((f"prop{i}", m, "prop"))
    for i in range(3):                                   # duct stators
        a = math.radians(90 + 120 * i)
        parts.append((f"stator{i}", strut((3.95, 0.07 * math.cos(a), 0.05 + 0.07 * math.sin(a)),
                                          (3.95, 0.42 * math.cos(a), 0.05 + 0.42 * math.sin(a)), 0.009), "dark"))
    for i, m in enumerate(turret((0.40, 0.0, -0.20))):
        parts.append((f"turret{i}", m, "turret"))
    gz = -0.68
    parts += [("ng_strut", strut((0.66, 0, -0.14), (0.68, 0, gz + 0.085), 0.015), "gear"),
              ("ng_wheel", wheel((0.68, 0, gz + 0.085), 0.085, 0.045), "tyre")]
    for sgn in (1, -1):
        parts += [(f"mg_strut{sgn}", strut((2.30, sgn * 0.26, -0.15), (2.36, sgn * 0.62, gz + 0.10), 0.02), "gear"),
                  (f"mg_wheel{sgn}", wheel((2.36, sgn * 0.66, gz + 0.10), 0.10, 0.05), "tyre")]
    parts += [("intake_r", ellipsoid((2.55, 0.25, 0.14), 0.22, 0.03, 0.06), "dark"),
              ("intake_l", ellipsoid((2.55, -0.25, 0.14), 0.22, 0.03, 0.06), "dark")]
    S = 2 * np.trapz([s["chord"] for s in wing], [s["y"] for s in wing]) + 2 * 0.20 * 1.67
    return {"key": "hancer", "name": "HANÇER", "name_tr": "Köşeli taşıyıcı gövde + kanalli itici fan", "parts": parts,
            "S_ref": S, "span": 2 * 3.15, "length": 4.02, "ground_z": gz,
            "e": 0.80, "cd_misc": 0.0050, "interf": 1.04, "gear_retract": True,
            "pitch": "Hareketli yatay stabilizatör (stabilator) + dışa eğik ikiz dümen, kanatçık/flap",
            "summary_tr": ("Elmas kesitli, keskin kenar çizgili (chine) taşıyıcı gövde ön kenar uzantılarıyla (LERX) "
                           "kanada kaynaşır; dışa eğik ikiz dikey kuyruk + hareketli yatay kuyruk; pervane bir kanal (duct) "
                           "içinde: daha sessiz, yerde güvenli, jet görünümlü arka bölüm.")}


DIRECTIONS = {"manta": manta, "ok": ok, "halka": halka, "hancer": hancer}


# =====================================================================================================================
# preliminary numbers (same equations for every direction)
# =====================================================================================================================
def numbers(d: dict) -> dict:
    """Component build-up with one flat-plate skin-friction value and per-type form factors (same for all four):
    CD0 = interference x sum(cf * FF * S_wet / S_ref) + misc (cooling, turret, antennas) + fixed-gear increment."""
    cf = AL.cf_flat(AL.reynolds(25.0, 0.6, H_LOITER))                     # turbulent (tripped) flat plate
    S = d["S_ref"] + d.get("S_extra", 0.0)
    wet, cdf = 0.0, 0.0
    for n, m, _c in d["parts"]:
        if n.startswith(("prop", "turret", "ng_", "mg_", "stator", "sat_", "intake")):
            continue
        a = m.area()
        ff = 1.20 if n.startswith(("fus", "spine", "duct")) else (1.45 if (d["key"] == "manta" and
                                                                           n.startswith("wing")) else 1.35)
        wet += a
        cdf += cf * ff * a / S
    span = d["span"]
    AR = span ** 2 / S
    cd0 = d.get("interf", 1.05) * cdf + d["cd_misc"] + (0.0 if d["gear_retract"] else 0.0040)
    e = d["e"]
    k = 1 / (math.pi * AR * e)
    ld = 0.5 / math.sqrt(cd0 * k)
    atm = AL.isa(H_LOITER)
    W0 = MTOM * G0
    W1 = (MTOM - 0.75 * FUEL) * G0                     # 25 % of the fuel: warm-up, climb, 2x100 km transit, reserve
    cl_e = min(math.sqrt(3 * cd0 / k), 1.05)                          # cap at a practical loiter CL
    cd_e = cd0 + k * cl_e ** 2
    E = AL.breguet_endurance_prop(ETA_LOITER, BSFC_LOITER / 3.6e6, cl_e, cd_e, atm["rho"], S, W0, W1) / 3600
    v_e = math.sqrt(2 * W0 / (atm["rho"] * S * cl_e))
    out = {"S_ref_m2": S, "span_m": span, "AR": AR, "S_wet_m2": wet, "cd0": cd0, "e": e, "LD_max": ld,
           "endurance_h": E, "v_loiter_tas_m_s": v_e, "ws_kg_m2": MTOM / S, "length_m": d["length"]}
    nd = {"cd0": 4, "S_ref_m2": 2, "span_m": 2, "length_m": 2, "e": 2}
    return {k: round(float(v), nd.get(k, 1)) for k, v in out.items()}


# =====================================================================================================================
# rendering (Workbench)
# =====================================================================================================================
VIEWS = {"iso": ((-1.0, -0.95, 0.62), (0, 0, 1), False), "rear": ((1.0, -0.85, 0.50), (0, 0, 1), False),
         "top": ((0, 0, 1), (-1, 0, 0), True), "side": ((0, -1, 0), (0, 0, 1), True),
         "front": ((-1, 0, 0), (0, 0, 1), True)}


def render(d: dict, res=(1400, 900)) -> dict:
    """Workbench views: outdoor studio light, specular, cavity, outline; a light ground plane under the perspective and
    side views (it is excluded from camera framing)."""
    import bpy
    from ucav250.blender import build as B
    B.reset_scene()
    col = B._collection("YK250_concept")
    mats = {k: B._material(k, v) for k, v in COL.items()}
    for name, m, ck in d["parts"]:
        if name.startswith(("sat_", "intake")):          # flush NACA inlets: not modelled at concept level
            continue
        B._mesh_object(name, m.V, m.F, col, mats[ck], smooth_deg=40.0)
    B.setup_workbench(res)
    sc = bpy.context.scene
    sc.view_settings.view_transform = "Standard"
    sc.world.color = (0.93, 0.94, 0.95)
    sh = sc.display.shading
    sh.light = "STUDIO"
    sh.studio_light = "outdoor.sl"
    sh.show_specular_highlight = True
    sh.show_shadows = True
    sh.shadow_intensity = 0.3
    sc.display.light_direction = (-0.45, -0.35, 0.82)
    sh.cavity_ridge_factor = sh.cavity_valley_factor = 1.0
    bpy.ops.mesh.primitive_plane_add(size=60, location=(1.5, 0.0, d["ground_z"]))
    ground = bpy.context.active_object
    ground.data.materials.append(mats["ground"])
    out = {}
    folder = OUT / d["key"]
    folder.mkdir(parents=True, exist_ok=True)
    for v, (dv, up, ortho) in VIEWS.items():
        p = folder / f"{v}.png"
        ground.hide_render = True
        ground.hide_set(True)
        B.render_view(v, p, dv, up, ortho, margin=1.05)          # frames the aircraft only
        show_ground = v in ("iso", "rear", "side")
        ground.hide_render = not show_ground
        ground.hide_set(not show_ground)
        sc.render.filepath = str(p)
        bpy.ops.render.render(write_still=True)
        out[v] = str(p)
    return out


def sheet(d: dict, num: dict, imgs: dict) -> str:
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.image as mpimg
    import matplotlib.pyplot as plt
    fig = plt.figure(figsize=(16, 10), dpi=110)
    fig.patch.set_facecolor("white")
    ax = fig.add_axes([0.0, 0.33, 0.62, 0.60])
    ax.imshow(mpimg.imread(imgs["iso"]))
    ax.axis("off")
    for (k, rect) in (("rear", [0.62, 0.55, 0.38, 0.38]), ("top", [0.0, 0.0, 0.34, 0.33]),
                      ("side", [0.33, 0.04, 0.34, 0.27]), ("front", [0.66, 0.04, 0.34, 0.27])):
        a = fig.add_axes(rect)
        a.imshow(mpimg.imread(imgs[k]))
        a.axis("off")
    fig.text(0.015, 0.955, f"YK-250 · {d['name']} — {d['name_tr']}", fontsize=20, weight="bold", color="#1E2228")
    fig.text(0.015, 0.93, "Tasarım yönü çalışması · Limbach L 275 EF 18 kW itici · MTOM 145 kg · 20 kg EO/IR faydalı yük · "
             "ön tahmin (±%20)", fontsize=10.5, color="#4A5058")
    tx = (f"Açıklık {num['span_m']} m · boy {num['length_m']} m · S {num['S_ref_m2']} m² · AR {num['AR']} · W/S "
          f"{num['ws_kg_m2']} kg/m²\nCD0 ≈ {num['cd0']} · e ≈ {num['e']} · (L/D)maks ≈ {num['LD_max']} · "
          f"dayanım ≈ {num['endurance_h']} h (3000 m)")
    fig.text(0.64, 0.50, tx, fontsize=11, color="#1E2228", va="top", linespacing=1.6)
    import textwrap
    fig.text(0.64, 0.41, "\n".join(textwrap.wrap(d["summary_tr"], 62)), fontsize=10, color="#30353B", va="top",
             linespacing=1.45)
    p = OUT / d["key"] / "sheet.png"
    fig.savefig(p, dpi=110)
    plt.close(fig)
    return str(p)


def compare(rows: list[tuple[dict, dict, dict]]) -> str:
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.image as mpimg
    import matplotlib.pyplot as plt
    fig = plt.figure(figsize=(18, 11), dpi=100)
    for i, (d, num, imgs) in enumerate(rows):
        r, c = divmod(i, 2)
        a = fig.add_axes([0.01 + 0.5 * c, 0.53 - 0.48 * r, 0.48, 0.40])
        a.imshow(mpimg.imread(imgs["iso"]))
        a.axis("off")
        fig.text(0.02 + 0.5 * c, 0.935 - 0.48 * r, f"{chr(65 + i)} · {d['name']} — {d['name_tr']}", fontsize=15,
                 weight="bold", color="#1E2228")
        fig.text(0.02 + 0.5 * c, 0.915 - 0.48 * r,
                 f"açıklık {num['span_m']} m · AR {num['AR']} · (L/D)maks ≈ {num['LD_max']} · dayanım ≈ "
                 f"{num['endurance_h']} h · {'içeri katlanır' if d['gear_retract'] else 'sabit kaportalı'} takım",
                 fontsize=10.5, color="#4A5058")
    fig.text(0.5, 0.012, "YK-250 v2 tasarım yönleri — aynı motor/görev ölçeği, ön tahmin (±%20). Seçilen yön ayrıntılı "
             "boyutlandırılır.", ha="center", fontsize=10, color="#4A5058")
    p = OUT / "compare.png"
    fig.savefig(p, dpi=100)
    plt.close(fig)
    return str(p)


def main(argv=None) -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--only", default="")
    ap.add_argument("--res", default="1400x900")
    ap.add_argument("--no-render", action="store_true")
    a = ap.parse_args(argv)
    keys = [k for k in a.only.split(",") if k] or list(DIRECTIONS)
    res = tuple(int(v) for v in a.res.split("x"))
    rows = []
    import yaml
    table = {}
    for k in keys:
        d = DIRECTIONS[k]()
        num = numbers(d)
        table[k] = {"name": d["name"], "name_tr": d["name_tr"], "summary_tr": d["summary_tr"], "pitch_tr": d["pitch"],
                    "gear_retractable": d["gear_retract"], **num}
        print(k, num)
        if not a.no_render:
            imgs = render(d, res)
            table[k]["sheet"] = sheet(d, num, imgs)
            rows.append((d, num, imgs))
    if rows and len(rows) == len(DIRECTIONS):
        table["compare"] = compare(rows)
    (OUT / "directions.yaml").write_text(yaml.safe_dump(table, allow_unicode=True, sort_keys=False), encoding="utf-8")
    return 0


if __name__ == "__main__":
    sys.exit(main())
