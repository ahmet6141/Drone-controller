#!/usr/bin/env python3
"""YELKOVAN YK-38 — boyutlandırma ve geometri kontrolü (saf Python; bpy gerektirmez).

Bütün büyüklükler ``ucav/params.py`` geometrisinden YENİDEN hesaplanır ve ``spec.yaml``'daki panel
referanslarıyla karşılaştırılır:

* Kanat: referans alan, AR, MAC ve yeri (planform integrali, merkez/konik kırığı dahil), gerçek planform alanı
  (glove + raked uç), kumanda yüzeyi alanları.
* Kuyruk: alanlar, MAC'ler, a.c. konumları, kollar, V_h / V_v, pervane–stabilize aralığı (5° disk eğimiyle).
* Kararlılık: nötr nokta ve statik marj — spec ``stability.method`` (a_wb, Helmbold a_h + uç plakası,
  DATCOM dε/dα, Raymer K_f gövde terimi, glove terimi).
* Performans: kanat yüklemesi, stall hızları; itki hattı yüksekliği ve momenti.
* İniş takımı: gerçek temas noktalarından iz, dingil açıklığı, tip-back, devrilme, burun yükü, pervane çarpma,
  kaporta/stabilize/kanat ucu temas açıları, açıklıklar; katlanmış teker konumları ve kuyu payları.
* Gövde, kütle toplamı ve yapı sığma kontrolleri (CF boruların profil içinde kalması, menteşe oyuğu payı).

Çıktı: ``ucav/out/sizing.md`` (Türkçe tablolar).
Kullanım:
    python3 ucav/sizing.py            # raporu yazar, özeti basar
    python3 ucav/sizing.py --check    # tolerans dışı değer varsa çıkış kodu 1
"""
from __future__ import annotations

import argparse
import math
import sys
from dataclasses import dataclass
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from ucav import airfoils as AF  # noqa: E402
from ucav import params as P  # noqa: E402

RAD, DEG = math.radians, math.degrees
REPORT_PATH = P.OUT_DIR / "sizing.md"


# =====================================================================================================
# Kontrol kaydı
# =====================================================================================================
@dataclass
class Check:
    """Bir büyüklüğün hesap–spec karşılaştırması. ``ref`` None ise yalnız bilgi (ya da ``limit`` ile sınır)."""

    group: str
    label: str
    value: float
    ref: float | None
    unit: str
    kind: str                       # tolerans sınıfı: rel, rel_loose, length, clearance, angle, pct, speed, mass, info, min, max
    fmt: str = ".4g"
    note: str = ""
    limit: float | None = None      # kind "min"/"max" için eşik

    def tol(self) -> float | None:
        c = P.SPEC["checks"]
        if self.kind == "rel":
            return abs(self.ref) * float(c["rel_default"]) if self.ref is not None else None
        if self.kind == "rel_loose":
            return abs(self.ref) * 0.05 if self.ref is not None else None
        absmap = {"length": c["abs"]["length_m"], "clearance": c["abs"]["clearance_mm"], "angle": c["abs"]["angle_deg"],
                  "pct": c["abs"]["pct_mac"], "speed": c["abs"]["speed_ms"], "mass": c["abs"]["mass_kg"]}
        return float(absmap[self.kind]) if self.kind in absmap else None

    @property
    def ok(self) -> bool:
        if self.kind == "info":
            return True
        if self.kind == "min":
            return self.value >= float(self.limit)
        if self.kind == "max":
            return self.value <= float(self.limit)
        return abs(self.value - float(self.ref)) <= float(self.tol()) + 1e-12

    @property
    def is_check(self) -> bool:
        return self.kind != "info"


# =====================================================================================================
# Hesaplar
# =====================================================================================================
def _fmt(v: float | None, fmt: str = ".4g") -> str:
    """Türkçe ondalık virgüllü sayı."""
    if v is None:
        return "—"
    return format(v, fmt).replace(".", ",")


def _trapz(f, x) -> float:
    return float(np.trapezoid(f, x)) if hasattr(np, "trapezoid") else float(np.trapz(f, x))


def wing_results() -> dict:
    """Kanat referans integralleri + gerçek planform alanı + kumanda yüzeyleri."""
    r = dict(P.wing_reference())
    pf = P.wing_planform(4000)
    r["S_planform"] = 2 * _trapz(pf["te"] - pf["le"], pf["y"])           # glove + raked uç dahil, gövde içi dahil
    g = P.SPEC["wing"]["glove"]
    yg = np.linspace(float(g["root_y_m"]), float(g["end_y_m"]), 2001)
    r["S_glove"] = 2 * _trapz(np.array([P.glove_extension(v) for v in yg]), yg)
    rt = P.SPEC["wing"]["raked_tip"]
    yr = np.linspace(float(rt["y_from_m"]), P.WING_SEMI_SPAN, 801)
    r["S_rake_loss"] = 2 * _trapz(np.array([P.wing_station(v).le_s - P.wing_le_ref(v) for v in yr]), yr)
    r["tip_chord_actual"] = P.wing_station(P.WING_SEMI_SPAN).chord
    ys = P.SPEC["wing"]["fuselage_side_y_m"]
    ye = np.linspace(ys, P.WING_SEMI_SPAN, 4001)
    r["S_exposed_ref"] = 2 * _trapz(np.asarray(P.wing_chord_ref(ye)), ye)
    cs = P.SPEC["wing"]["control_surfaces"]
    r["A_aileron"] = P.control_surface_area("Aileron")
    r["A_flap"] = P.control_surface_area("FlapIn") + P.control_surface_area("FlapOut")
    flapped = 0.0
    for k in ("flap_in", "flap_out"):
        yy = np.linspace(cs[k]["y_from_m"], cs[k]["y_to_m"], 2001)
        flapped += 2 * _trapz(np.asarray(P.wing_chord_ref(yy)), yy)
    r["flapped_ratio"] = flapped / r["S"]
    a = cs["aileron"]
    r["ail_chord_in"] = a["chord_fraction"] * P.wing_chord_ref(a["y_from_m"])
    r["ail_chord_out"] = a["chord_fraction"] * P.wing_chord_ref(a["y_to_m"])
    yq = np.linspace(P.WING_Y_KINK, P.WING_SEMI_SPAN, 3)
    q = np.asarray(P.wing_le_ref(yq)) + 0.25 * np.asarray(P.wing_chord_ref(yq))
    r["sweep_c4_deg"] = DEG(math.atan((q[-1] - q[0]) / (yq[-1] - yq[0])))
    return r


def tail_results(wing: dict) -> dict:
    """Stabilize ve dikeyler: alanlar, MAC, a.c., kollar, hacim katsayıları, pervane aralığı."""
    T = P.SPEC["tail"]
    st, fn = T["stab"], T["fin"]
    b2 = P.STAB_HALF_SPAN
    y = np.linspace(0.0, b2, 4001)
    c = np.array([P.stab_station(v).chord for v in y])
    le = np.array([P.stab_station(v).le_s for v in y])
    Sh = 2 * _trapz(c, y)
    mach = 2 / Sh * _trapz(c ** 2, y)
    ymh = 2 / Sh * _trapz(c * y, y)
    xc4 = 2 / Sh * _trapz(c * (le + 0.25 * c), y)
    x_ach = xc4                                  # a.c. = MAC'in %25'i
    # açıkta kalan alan: stabilize kökünde (c/4) stabilize yüksekliğinde gövde yarı genişliği
    s_root_c4 = float(st["root_le_s_m"]) + 0.25 * float(st["root_chord_m"])
    y_exp = P.fuselage_half_width_at(s_root_c4, float(st["z_m"]))
    m = y >= y_exp
    Sh_exp = 2 * _trapz(c[m], y[m])
    root, tip = P.stab_station(0.0), P.stab_station(b2)
    sweep_te = DEG(math.atan((tip.te_s - root.te_s) / b2))
    sweep_c2 = DEG(math.atan(((tip.le_s + tip.chord / 2) - (root.le_s + root.chord / 2)) / b2))
    ARh = (2 * b2) ** 2 / Sh
    k = float(T["aero"]["endplate_k"])
    ARe = ARh * (1 + 1.9 * k * float(fn["height_m"]) / (2 * b2))
    # dikeyler
    H = float(fn["height_m"])
    h = np.linspace(0.0, H, 4001)
    cv = np.array([P.fin_station(v).chord for v in h])
    lev = np.array([P.fin_station(v).le[0] for v in h])
    Sv1 = _trapz(cv, h)
    macv = _trapz(cv ** 2, h) / Sv1
    hmv = _trapz(cv * h, h) / Sv1
    x_acv = _trapz(cv * (lev + 0.25 * cv), h) / Sv1
    Sv = 2 * Sv1
    Sv_proj = Sv * math.cos(RAD(float(fn["cant_deg"])))
    ftip = P.fin_station(H)
    cg = P.CG.s
    lh = x_ach - wing["ac_s"]
    Vh = Sh * lh / (wing["S"] * wing["MAC"])
    lv = x_acv - cg
    Vv = Sv_proj * lv / (wing["S"] * 2 * P.WING_SEMI_SPAN)
    # pervane – stabilize firar kenarı aralığı (disk 5° eğik: alçaldıkça geride)
    pr = P.PROP
    dz = float(st["z_m"]) - pr.hub[2]
    y_disc = math.sqrt(max(pr.radius ** 2 - dz ** 2, 0.0))
    s_plane = pr.plane_s_at_z(float(st["z_m"]))
    gap_tip = s_plane - P.stab_station(y_disc).te_s
    gap_root = s_plane - root.te_s
    return {"Sh": Sh, "Sh_exp": Sh_exp, "y_exp": y_exp, "MAC_h": mach, "y_mac_h": ymh, "x_ac_h": x_ach, "AR_h": ARh,
            "AR_h_eff": ARe, "sweep_te_h": sweep_te, "sweep_c2_h": sweep_c2, "tip_le": tip.le_s, "tip_te": tip.te_s,
            "Sv": Sv, "Sv_proj": Sv_proj, "MAC_v": macv, "h_mac_v": hmv, "x_ac_v": x_acv, "fin_tip_y": ftip.le[1],
            "fin_tip_z": ftip.le[2], "fin_tip_le": ftip.le[0], "fin_tip_te": ftip.te_s,
            "fin_root_te": P.fin_station(0.0).te_s, "tip_to_tip": 2 * ftip.le[1], "l_h": lh, "V_h": Vh, "l_v": lv,
            "V_v": Vv, "tail_ratio": (Sh + Sv) / wing["S"], "S_tail": Sh + Sv, "gap_tip": gap_tip, "gap_root": gap_root,
            "gap_over_D": gap_tip / pr.diameter, "y_disc": y_disc,
            "A_elevator": P.control_surface_area("Elevator"), "A_rudder": P.control_surface_area("Rudder")}


def helmbold(A: float, sweep_c2_deg: float, kappa: float = 1.0) -> float:
    """Helmbold/DATCOM kaldırma eğimi (1/rad): 2πA / (2 + √(4 + A²/κ²·(1 + tan²Λ_c/2)))."""
    return 2 * math.pi * A / (2 + math.sqrt(4 + (A ** 2 / kappa ** 2) * (1 + math.tan(RAD(sweep_c2_deg)) ** 2)))


def datcom_downwash(AR: float, taper: float, l_H: float, h_H: float, b: float, sweep_c4_deg: float = 0.0) -> float:
    """DATCOM dε/dα: 4,44·[K_A·K_λ·K_H·√cosΛ_c/4]^1,19."""
    KA = 1 / AR - 1 / (1 + AR ** 1.7)
    Kl = (10 - 3 * taper) / 7
    KH = (1 - abs(h_H / b)) / (2 * l_H / b) ** (1 / 3)
    return 4.44 * (KA * Kl * KH * math.sqrt(math.cos(RAD(sweep_c4_deg)))) ** 1.19


def stability_results(wing: dict, tail: dict) -> dict:
    """Nötr nokta ve statik marj (spec ``stability.method``)."""
    S = P.SPEC
    inp = S["stability"]["inputs"]
    a_wb = float(inp["a_wb_per_rad"])
    eta = float(inp["eta_h"])
    a_h = helmbold(tail["AR_h_eff"], tail["sweep_c2_h"], float(S["tail"]["aero"]["kappa"]))
    l_H = tail["x_ac_h"] - (float(S["wing"]["le_root_s_m"]) + 0.25 * P.WING_ROOT_CHORD)
    h_H = float(S["tail"]["stab"]["z_m"]) - P.WING_Z_ROOT
    de = datcom_downwash(wing["AR"], wing["taper"], l_H, h_H, 2 * P.WING_SEMI_SPAN, wing["sweep_c4_deg"])
    F_h = eta * a_h * tail["Sh_exp"] / wing["S"] * (1 - de)
    w_f = max(P.fuselage_section(s).width for s in np.linspace(0, P.FUSELAGE_LENGTH, 441))
    Cmaf = float(inp["K_f_per_deg"]) * w_f ** 2 * P.FUSELAGE_LENGTH / (wing["MAC"] * wing["S"]) * 180 / math.pi
    F_g, x_g = float(inp["F_g"]), float(inp["x_g_s_m"])
    x_np = (a_wb * wing["ac_s"] + F_h * tail["x_ac_h"] + F_g * x_g - Cmaf * wing["MAC"]) / (a_wb + F_h + F_g)
    c = wing["MAC"]
    cg = P.CG.s
    out = {"a_h": a_h, "de": de, "F_h": F_h, "Cmaf": Cmaf, "x_np": x_np, "np_pct": 100 * (x_np - wing["le_s_mac"]) / c,
           "cg_pct": 100 * (cg - wing["le_s_mac"]) / c, "SM": 100 * (x_np - cg) / c,
           "cg_fwd": x_np - 0.15 * c, "cg_aft": x_np - 0.05 * c, "l_H": l_H, "h_H": h_H}
    for e in (0.85, 1.0):
        Fh_e = e * a_h * tail["Sh_exp"] / wing["S"] * (1 - de)
        xn = (a_wb * wing["ac_s"] + Fh_e * tail["x_ac_h"] + F_g * x_g - Cmaf * c) / (a_wb + Fh_e + F_g)
        out[f"SM_eta_{e:.2f}"] = 100 * (xn - cg) / c
    return out


def performance_results(wing: dict) -> dict:
    """Kanat yüklemesi, stall hızları, itki hattı."""
    S = P.SPEC
    rho, g = float(S["performance"]["rho_kg_m3"]), float(S["performance"]["g_m_s2"])
    m, m_land = float(S["mass"]["mtow_kg"]), float(S["mass"]["landing_mass_kg"])
    cl = S["wing"]["aero"]["CLmax"]

    def vs(mass, CL):
        return math.sqrt(2 * mass * g / (rho * wing["S"] * CL))

    T = float(S["propulsion"]["performance"]["static_thrust_kgf"]) * g
    z_t = P.thrust_line_z(P.CG.s)
    arm = z_t - P.CG.z
    return {"wing_loading": m / wing["S"], "vs_clean": vs(m, cl["clean"]), "vs_f20": vs(m, cl["flaps_20"]),
            "vs_f20_land": vs(m_land, cl["flaps_20"]), "vs_f30": vs(m, cl["flaps_30"]),
            "thrust_z_cg": z_t, "thrust_arm": arm, "thrust_moment": T * math.cos(RAD(P.PROP.downthrust_deg)) * arm,
            "T_W": T / (m * g)}


def _pitch_angle(point: tuple[float, float, float], pivot_s: float) -> float:
    """Ana teker temas çizgisi etrafında burun yukarı dönüşte ``point``'in zemine değdiği açı (derece)."""
    return DEG(math.atan2(point[2] - P.GROUND_Z, point[0] - pivot_s))


def gear_results(wing: dict) -> dict:
    """Temas noktalarından yer geometrisi ve açılar."""
    S = P.SPEC
    gN, gL, gR = P.gear_leg("N"), P.gear_leg("L"), P.gear_leg("R")
    cN, cL, cR = gN.contact_static, gL.contact_static, gR.contact_static
    sm = cL[0]
    cg_s, cg_z = P.CG.s, P.CG.z
    h = cg_z - P.GROUND_Z
    track = cL[1] - cR[1]
    wb = sm - cN[0]
    aft = S["stability"]["cg_range_s_m"][1]
    phi = math.atan((track / 2) / wb)
    overturn = DEG(math.atan(h / ((cg_s - cN[0]) * math.sin(phi))))
    W = float(S["mass"]["mtow_kg"]) * float(S["performance"]["g_m_s2"])
    nose_frac = (sm - cg_s) / wb
    pr = P.PROP
    pb = pr.disk_point(0.0)
    th = RAD(6.0)
    dx, dz = pb[0] - sm, pb[2] - P.GROUND_Z
    # kaporta / gövde alt hattı: ana tekerin arkasındaki en kritik nokta
    ss = np.linspace(1.45, P.FUSELAGE_LENGTH, 300)
    ang = [(_pitch_angle((s, 0.0, P.fuselage_section(s).z_bottom), sm), s) for s in ss]
    cowl_ang, cowl_s = min(ang)
    # stabilize ucu (uç kesitinin en alçak noktaları) ve dikey kökü firar kenarı
    tip_sec = P.stab_section(P.STAB_HALF_SPAN)
    stab_ang = min(_pitch_angle(tuple(p), sm) for p in tip_sec)
    stab_te = P.stab_station(P.STAB_HALF_SPAN)
    stab_te_ang = _pitch_angle((stab_te.te_s, 0.0, stab_te.z), sm)
    fin_root = P.fin_section(0.0)
    fin_ang = min(_pitch_angle(tuple(p), sm) for p in fin_root)
    # kanat ucu: dış panel ucu (1,83) ve raked uç kesitlerinin en alçak noktası
    tip_z = min(P.wing_section(v)[:, 2].min() for v in np.linspace(1.80, P.WING_SEMI_SPAN - 0.006, 7))
    wingtip_clear = tip_z - P.GROUND_Z
    bank = DEG(math.atan(wingtip_clear / (P.WING_SEMI_SPAN - cL[1])))
    belly = min(P.fuselage_section(s).z_bottom for s in np.linspace(0.5, 0.85, 71)) - P.GROUND_Z
    # katlanmış tekerler ve kuyu payları
    mw = S["landing_gear"]["main"]["well"]
    a = gL.axle_retracted
    r = gL.wheel_r
    main_margins = {"ön": a[0] - r - mw["s_m"][0], "arka": mw["s_m"][1] - (a[0] + r),
                    "iç": a[1] - r - mw["y_m"][0], "dış": mw["y_m"][1] - (a[1] + r)}
    fz = float(S["wing"]["root_fairing"]["bottom_z_m"])
    main_margins["ağız"] = (a[2] - gL.wheel_w / 2) - fz
    main_margins["tavan"] = (fz + mw["depth_m"]) - (a[2] + gL.wheel_w / 2)
    nw = S["landing_gear"]["nose"]["well"]
    an = gN.axle_retracted
    nose_margins = {"ön": an[0] - gN.wheel_r - nw["s_m"][0], "arka": nw["s_m"][1] - (an[0] + gN.wheel_r),
                    "yan": nw["half_width_m"] - gN.wheel_w / 2, "tavan": nw["roof_z_m"] - (an[2] + gN.wheel_r),
                    "karın": (an[2] - gN.wheel_r) - P.fuselage_section(an[0]).z_bottom}
    lower = P.wing_surface_z(gL.pivot[0], gL.pivot[1], "lower")
    return {"track": track, "wheelbase": wb, "cg_height": h, "main_axle_s": gL.axle_static[0],
            "tip_back": DEG(math.atan((sm - cg_s) / h)), "tip_back_aft": DEG(math.atan((sm - aft) / h)),
            "overturn": overturn, "nose_load_pct": 100 * nose_frac, "load_main_each": W * (1 - nose_frac) / 2,
            "load_nose": W * nose_frac, "prop_clear_mm": 1000 * dz, "prop_strike": _pitch_angle(pb, sm),
            "prop_clear_6deg_mm": 1000 * (dz * math.cos(th) - dx * math.sin(th)), "cowl_contact": cowl_ang,
            "cowl_contact_s": cowl_s, "stab_tip_contact": stab_ang, "stab_tip_te_contact": stab_te_ang,
            "fin_root_contact": fin_ang, "wingtip_clear_mm": 1000 * wingtip_clear, "bank_to_tip": bank,
            "turret_clear_mm": 1000 * P.TURRET.ground_clearance, "belly_clear_mm": 1000 * belly,
            "main_retracted": a, "nose_retracted": an, "main_well_margins": main_margins,
            "nose_well_margins": nose_margins, "main_sag": gL.static_sag, "nose_sag": gN.static_sag,
            "pivot_below_wing_mm": 1000 * (lower - gL.pivot[2]) if lower is not None else float("nan"),
            "height": P.fin_station(float(S["tail"]["fin"]["height_m"])).le[2] - P.GROUND_Z}


def fuselage_results() -> dict:
    """Gövde azami ölçüleri, ıslak alan (çevre integrali), incelik oranı, kuyruk konisi kalkışı."""
    ss = np.linspace(0.0, P.FUSELAGE_LENGTH, 1103)
    secs = [P.fuselage_section(s) for s in ss]
    per, area = [], []
    for s in ss:
        o = P.fuselage_outline(s, 48)
        c = np.vstack([o, o[:1]])
        per.append(float(np.sum(np.linalg.norm(np.diff(c, axis=0), axis=1))))
        area.append(0.5 * abs(float(np.dot(o[:, 0], np.roll(o[:, 1], -1)) - np.dot(o[:, 1], np.roll(o[:, 0], -1)))))
    i = int(np.argmax(area))
    d_eq = math.sqrt(4 * area[i] / math.pi)

    def zb(s):
        return P.fuselage_section(s).z_bottom

    return {"max_w": max(c.width for c in secs), "max_h": max(c.height for c in secs),
            "max_h_s": ss[int(np.argmax([c.height for c in secs]))], "wetted": _trapz(np.array(per), ss),
            "A_max": area[i], "A_max_s": ss[i], "d_eq": d_eq,
            "fineness": P.FUSELAGE_LENGTH / (0.5 * (max(c.width for c in secs) + max(c.height for c in secs))),
            "upsweep_110": DEG(math.atan((zb(1.95) - zb(1.10)) / 0.85)),
            "upsweep_140": DEG(math.atan((zb(1.95) - zb(1.40)) / 0.55)),
            "nose_droop": -P.fuselage_section(0.0).z_center}


def structure_results() -> dict:
    """CF boru sığma ve menteşe oyuğu payları (baskı/yapı ön kontrolü)."""
    W = P.SPEC["wing"]
    skin = float(P.SPEC["print"]["walls_mm"]["wing_skin"]) / 1000
    out = {}
    # uç borusu y = 1,70'te: OD + 2 kabuk + 2 mm ≤ yerel kalınlık (panel ölçütü)
    for t in P.spar_tubes("L"):
        if t.name in ("centre_socket", "panel_tube", "tip_tube", "rear_spar"):
            yy = abs(t.p1[1])
            thick = P.wing_thickness_at(t.p1[0], yy)
            out[f"fit_{t.name}"] = (thick - (t.od + 2 * skin + 0.002)) * 1000 if t.name == "tip_tube" else (thick - (t.od + 2 * skin)) * 1000
            out[f"thick_{t.name}"] = thick * 1000
            out[f"y_{t.name}"] = yy
    # arka kiriş ↔ menteşe oyuğu (yuvarlak burun yarıçapı + aralık) en dar pay
    rs = [t for t in P.spar_tubes("L") if t.name == "rear_spar"][0]
    margins = []
    for yy in np.linspace(abs(rs.p0[1]), abs(rs.p1[1]), 60):
        f = (yy - abs(rs.p0[1])) / (abs(rs.p1[1]) - abs(rs.p0[1]))
        s_tube = rs.p0[0] + f * (rs.p1[0] - rs.p0[0])
        st = P.wing_station(yy)
        r = 0.5 * float(AF.thickness_at(P.wing_airfoil(yy), 0.72)) * st.chord
        cove = (st.te_s - 0.28 * st.chord) - r - float(W["control_surfaces"]["hinge_gap_m"])
        margins.append((cove - (s_tube + rs.od / 2)) * 1000)
    out["rear_spar_cove_mm"] = min(margins)
    # stabilize ve dikey kirişleri: dış uçta kalınlık − (OD + 2·0,5 mm)
    tskin = float(P.SPEC["print"]["walls_mm"]["tail_skin"]) / 1000
    for t in P.spar_tubes("L"):
        if t.name == "stab_spar":
            st = P.stab_station(t.p1[1])
            xc = (t.p1[0] - st.le_s) / st.chord
            th = float(AF.thickness_at(P.tail_airfoil(), xc)) * st.chord
            out["fit_stab_spar"] = (th - (t.od + 2 * tskin)) * 1000
            # borunun sığdığı en dış y
            for yy in np.linspace(abs(t.p1[1]), 0.0, 400):
                st2 = P.stab_station(yy)
                s_t = t.p0[0] + (yy - abs(t.p0[1])) * math.tan(RAD(30.0))
                th2 = float(AF.thickness_at(P.tail_airfoil(), (s_t - st2.le_s) / st2.chord)) * st2.chord
                if th2 >= t.od + 2 * tskin:
                    out["stab_spar_fit_to_y"] = yy
                    break
        if t.name == "fin_spar":
            hh = P.SPEC["tail"]["fin"]["height_m"] * 0.98
            st = P.fin_station(hh)
            out["fit_fin_spar"] = (0.10 * st.chord * 0.98 - (t.od + 2 * tskin)) * 1000
    return out


def mass_results() -> dict:
    """Kütle kalemleri toplamı ve boş kütle."""
    M = P.SPEC["mass"]
    total = sum(float(v) for _, v in M["breakdown"])
    return {"sum": total, "empty": float(M["mtow_kg"]) - float(M["fuel_kg"]) - float(M["payload_kg"]),
            "fuel_from_volume": float(P.SPEC["propulsion"]["fuel"]["tank_l"]) * float(P.SPEC["propulsion"]["fuel"]["density_kg_l"])}


def compute() -> dict:
    """Bütün hesaplar: ``{"wing", "tail", "stab", "perf", "gear", "fus", "struct", "mass"}``."""
    w = wing_results()
    t = tail_results(w)
    return {"wing": w, "tail": t, "stab": stability_results(w, t), "perf": performance_results(w),
            "gear": gear_results(w), "fus": fuselage_results(), "struct": structure_results(), "mass": mass_results()}


# =====================================================================================================
# Karşılaştırma tablosu
# =====================================================================================================
def build_checks(R: dict) -> list[Check]:
    """Hesap ↔ spec karşılaştırmaları (grup sırası raporla aynı)."""
    S = P.SPEC
    w, t, st, pf, g, fu, sr, ms = (R[k] for k in ("wing", "tail", "stab", "perf", "gear", "fus", "struct", "mass"))
    wr, tr, gr = S["wing"]["reference"], S["tail"]["reference"], S["landing_gear"]["reference"]
    C: list[Check] = []

    def add(group, label, value, ref, unit, kind, fmt=".4g", note="", limit=None):
        C.append(Check(group, label, float(value), None if ref is None else float(ref), unit, kind, fmt, note, limit))

    G = "Genel ölçüler"
    add(G, "Açıklık", 2 * P.WING_SEMI_SPAN, S["overall"]["span_m"], "m", "length", ".3f")
    add(G, "Toplam boy (dikey ucu firar kenarı)", t["fin_tip_te"], S["overall"]["length_m"], "m", "length", ".3f")
    add(G, "Spinner dahil gövde boyu", P.PROP.spinner_tip_s, S["overall"]["length_with_spinner_m"], "m", "length", ".3f")
    add(G, "Yükseklik (takım açık)", g["height"], S["overall"]["height_m"], "m", "length", ".3f")
    add(G, "Teker izi", g["track"], S["overall"]["wheel_track_m"], "m", "length", ".3f")
    add(G, "Dingil açıklığı", g["wheelbase"], S["overall"]["wheelbase_m"], "m", "length", ".3f",
        "statik temas noktaları arası (ana bacak 12° yatık; sıkışma aksı 1,7 mm öne alır)")

    G = "Kanat"
    add(G, "Referans alan S", w["S"], wr["area_m2"], "m²", "rel", ".4f", "trapez, gövde içi dahil")
    add(G, "Açıklık oranı AR", w["AR"], wr["aspect_ratio"], "", "rel", ".2f")
    add(G, "Sivrilme λ", w["taper"], wr["taper"], "", "rel", ".3f")
    add(G, "MAC", w["MAC"], wr["mac_m"], "m", "length", ".4f")
    add(G, "MAC y konumu", w["y_mac"], wr["mac_y_m"], "m", "length", ".3f")
    add(G, "MAC hücum kenarı s", w["le_s_mac"], wr["mac_le_s_m"], "m", "length", ".4f")
    add(G, "Kanat a.c. s (MAC %25)", w["ac_s"], wr["ac_s_m"], "m", "length", ".4f")
    add(G, "Ana kiriş s (%28)", w["spar_s"], wr["spar_s_m"], "m", "length", ".4f")
    add(G, "HK oku (dış panel)", w["sweep_le_deg"], wr["sweep_le_deg"], "°", "angle", ".2f")
    add(G, "FK oku (dış panel)", w["sweep_te_deg"], wr["sweep_te_deg"], "°", "angle", ".2f")
    add(G, "Uç veter düzlemi z", w["tip_z"], wr["tip_chord_plane_z_m"], "m", "length", ".4f")
    add(G, "Glove alanı (iki yan)", w["S_glove"], S["wing"]["glove"]["area_both_m2"], "m²", "rel", ".4f")
    add(G, "Gerçek planform alanı (glove + raked uç)", w["S_planform"], None, "m²", "info", ".4f",
        f"glove +{_fmt(w['S_glove'], '.4f')}, raked uç −{_fmt(w['S_rake_loss'], '.4f')}")
    add(G, "Açıkta kalan referans alan (y ≥ 0,1025)", w["S_exposed_ref"], None, "m²", "info", ".4f")
    add(G, "Raked uç veteri (y = 1,90)", w["tip_chord_actual"], None, "m", "info", ".3f")

    G = "Kanat planform tablosu"
    for y, le, te, cref, zr, inc in wr.get("planform_table", []):
        ws = P.wing_station(y)
        yl = _fmt(y, ".4g")
        add(G, f"y = {yl}: hücum kenarı s", ws.le_s, le, "m", "length", ".4f")
        add(G, f"y = {yl}: firar kenarı s", ws.te_s, te, "m", "length", ".4f")
        add(G, f"y = {yl}: referans veter", ws.chord_ref, cref, "m", "length", ".4f")
        add(G, f"y = {yl}: veter düzlemi z (kiriş hattı)", ws.z_ref, zr, "m", "length", ".4f")
        add(G, f"y = {yl}: yerel açı (kök + burulma)", ws.incidence_deg, inc, "°", "angle", ".3f")

    G = "Kumanda yüzeyleri"
    cs = S["wing"]["control_surfaces"]
    add(G, "Kanatçık alanı (iki yan)", w["A_aileron"], cs["aileron"]["area_total_m2"], "m²", "rel", ".4f")
    add(G, "Kanatçık veteri iç uç", w["ail_chord_in"], cs["aileron"]["chord_root_tip_m"][0], "m", "length", ".4f")
    add(G, "Kanatçık veteri dış uç", w["ail_chord_out"], cs["aileron"]["chord_root_tip_m"][1], "m", "length", ".4f")
    add(G, "Flap alanı (iki yan)", w["A_flap"], cs["flap_area_total_m2"], "m²", "rel_loose", ".4f")
    add(G, "Flaplı alan oranı", w["flapped_ratio"], cs["flapped_area_ratio"], "", "rel", ".3f")
    add(G, "İrtifa dümeni alanı (iki yan)", t["A_elevator"], S["tail"]["control_surfaces"]["elevator"]["area_total_m2"],
        "m²", "rel_loose", ".4f")
    add(G, "İstikamet dümeni alanı (iki dümen)", t["A_rudder"], S["tail"]["control_surfaces"]["rudder"]["area_total_m2"],
        "m²", "rel_loose", ".4f")

    G = "Kuyruk"
    add(G, "Yatay alan S_h", t["Sh"], tr["area_h_m2"], "m²", "rel", ".4f")
    add(G, "Açıkta yatay alan S_h,açık", t["Sh_exp"], tr["area_h_exposed_m2"], "m²", "rel_loose", ".4f",
        f"gövde yarı genişliği stabilize kökünde {_fmt(t['y_exp'], '.4f')} m")
    add(G, "Dikey alan (gerçek, iki)", t["Sv"], tr["area_v_true_m2"], "m²", "rel", ".4f")
    add(G, "Dikey alan (izdüşüm)", t["Sv_proj"], tr["area_v_projected_m2"], "m²", "rel", ".4f")
    add(G, "Toplam kuyruk alanı", t["S_tail"], tr["area_m2"], "m²", "rel", ".4f")
    add(G, "Kuyruk/kanat alan oranı", t["tail_ratio"], tr["tail_to_wing_area_ratio"], "", "rel", ".3f")
    add(G, "Stabilize MAC", t["MAC_h"], tr["stab_mac_m"], "m", "length", ".4f")
    add(G, "Dikey MAC", t["MAC_v"], tr["fin_mac_m"], "m", "length", ".4f")
    add(G, "Stabilize AR", t["AR_h"], tr["stab_AR"], "", "rel", ".2f")
    add(G, "Stabilize etkin AR (uç plakası)", t["AR_h_eff"], tr["stab_AR_effective"], "", "rel", ".2f")
    add(G, "Stabilize a.c. s", t["x_ac_h"], tr["stab_ac_s_m"], "m", "length", ".4f")
    add(G, "Dikey a.c. s", t["x_ac_v"], tr["fin_ac_s_m"], "m", "length", ".4f")
    add(G, "Kuyruk kolu l_h (kanat a.c. → stabilize a.c.)", t["l_h"], tr["arm_m"], "m", "length", ".4f")
    add(G, "Dikey kolu l_v (CG → dikey a.c.)", t["l_v"], tr["l_v_m"], "m", "length", ".4f")
    add(G, "Yatay hacim katsayısı V_h", t["V_h"], tr["volume_h"], "", "rel", ".4f")
    add(G, "Dikey hacim katsayısı V_v", t["V_v"], tr["volume_v"], "", "rel", ".4f")
    add(G, "Stabilize uç HK s", t["tip_le"], tr["stab_tip_le_s_m"], "m", "length", ".4f")
    add(G, "Stabilize uç FK s", t["tip_te"], tr["stab_tip_te_s_m"], "m", "length", ".4f")
    add(G, "Stabilize FK oku", t["sweep_te_h"], tr["stab_sweep_te_deg"], "°", "angle", ".2f")
    add(G, "Dikey uç HK s", t["fin_tip_le"], tr["fin_tip_le_s_m"], "m", "length", ".4f")
    add(G, "Dikey uç y", t["fin_tip_y"], tr["fin_tip_y_m"], "m", "length", ".4f")
    add(G, "Dikey uç z", t["fin_tip_z"], tr["fin_tip_z_m"], "m", "length", ".4f")
    add(G, "Dikey uçları arası", t["tip_to_tip"], tr["tip_to_tip_m"], "m", "length", ".3f")
    add(G, "Pervane – stabilize FK aralığı (pala ucunda)", t["gap_tip"], tr["gap_te_to_prop_m"], "m", "length", ".4f",
        f"disk yarı genişliği stabilize düzleminde {_fmt(t['y_disc'], '.4f')} m, 5° eğim dahil")
    add(G, "Pervane – stabilize FK aralığı (kökte)", t["gap_root"], tr["gap_root_m"], "m", "length", ".4f")
    add(G, "Aralık / pervane çapı", t["gap_over_D"], tr["gap_over_D"], "", "rel_loose", ".3f")

    G = "Kararlılık"
    sref = S["stability"]
    add(G, "Stabilize kaldırma eğimi a_h", st["a_h"], sref["reference"]["a_h_per_rad"], "1/rad", "rel", ".3f",
        "Helmbold, AR_etkin")
    add(G, "Sapma gradyanı dε/dα", st["de"], sref["reference"]["deps_dalpha"], "", "rel", ".3f",
        f"DATCOM, l_H {_fmt(st['l_H'], '.3f')} m, h_H {_fmt(st['h_H'], '.3f')} m")
    add(G, "Kuyruk terimi F_h", st["F_h"], sref["reference"]["F_h"], "", "rel", ".3f")
    add(G, "Gövde Cmα_f", st["Cmaf"], sref["reference"]["Cm_alpha_f_per_rad"], "1/rad", "rel", ".3f")
    add(G, "Nötr nokta s", st["x_np"], sref["np_s_m"], "m", "length", ".4f")
    add(G, "Nötr nokta (%MAC)", st["np_pct"], sref["np_pct_mac"], "%", "pct", ".1f")
    add(G, "CG (%MAC)", st["cg_pct"], sref["cg_pct_mac"], "%", "pct", ".1f")
    add(G, "Statik marj (η_h 0,90)", st["SM"], sref["static_margin_pct_mac"], "%MAC", "pct", ".1f")
    add(G, "Statik marj (η_h 0,85 / 1,00)", st["SM_eta_0.85"], None, "%MAC", "info", ".1f",
        f"η_h 1,00: {_fmt(st['SM_eta_1.00'], '.1f')}")
    add(G, "Ön CG sınırı (SM %15)", st["cg_fwd"], sref["cg_range_s_m"][0], "m", "length", ".4f")
    add(G, "Arka CG sınırı (SM %5)", st["cg_aft"], sref["cg_range_s_m"][1], "m", "length", ".4f")

    G = "Performans ve itki hattı"
    pr = S["performance"]
    add(G, "Kanat yüklemesi", pf["wing_loading"], pr["wing_loading_kg_m2"], "kg/m²", "rel", ".2f")
    add(G, "Stall hızı, temiz (MTOW)", pf["vs_clean"], pr["stall_speed_ms"]["clean"], "m/s", "speed", ".2f")
    add(G, "Stall hızı, 20° flap (MTOW)", pf["vs_f20"], pr["stall_speed_ms"]["flaps_20_mtow"], "m/s", "speed", ".2f")
    add(G, "Stall hızı, 20° flap (iniş kütlesi)", pf["vs_f20_land"], pr["stall_speed_ms"]["flaps_20_landing"], "m/s",
        "speed", ".2f")
    add(G, "Stall hızı, 30° flap (MTOW)", pf["vs_f30"], pr["stall_speed_ms"]["flaps_30_mtow"], "m/s", "speed", ".2f")
    pp = S["propulsion"]["performance"]
    add(G, "İtki hattı z (CG istasyonunda)", pf["thrust_z_cg"], pp["thrust_line_z_at_cg_m"], "m", "length", ".4f")
    add(G, "İtki momenti (tam güç, burun aşağı)", pf["thrust_moment"], pp["thrust_moment_nm"], "N·m", "rel_loose", ".2f")
    add(G, "İtki/ağırlık", pf["T_W"], pp["thrust_to_weight"], "", "rel", ".3f")

    G = "İniş takımı ve yer açıları"
    add(G, "Ana aks s (statik)", g["main_axle_s"], gr["main_axle_s_m"], "m", "length", ".4f")
    add(G, "CG yüksekliği (zeminden)", g["cg_height"], gr["cg_height_m"], "m", "length", ".4f")
    add(G, "Tip-back açısı (nominal CG)", g["tip_back"], gr["tip_back_deg"], "°", "angle", ".2f")
    add(G, "Tip-back açısı (arka CG)", g["tip_back_aft"], gr["tip_back_aft_cg_deg"], "°", "angle", ".2f")
    add(G, "Devrilme açısı", g["overturn"], gr["overturn_deg"], "°", "angle", ".2f")
    add(G, "Burun yükü", g["nose_load_pct"], gr["nose_load_pct"], "%", "pct", ".1f")
    add(G, "Statik yük, ana teker başına", g["load_main_each"], gr["static_loads_n"]["main_each"], "N", "rel", ".1f")
    add(G, "Statik yük, burun", g["load_nose"], gr["static_loads_n"]["nose"], "N", "rel_loose", ".1f")
    add(G, "Pervane yer açıklığı", g["prop_clear_mm"], gr["prop_clearance_mm"], "mm", "clearance", ".0f")
    add(G, "Pervane çarpma açısı", g["prop_strike"], gr["prop_strike_deg"], "°", "angle", ".2f", "≥ 13° şartı")
    add(G, "Pervane açıklığı 6° burun yukarıda", g["prop_clear_6deg_mm"], gr["prop_clearance_at_6deg_mm"], "mm",
        "clearance", ".0f")
    add(G, "Kaporta/gövde temas açısı", g["cowl_contact"], gr["cowl_contact_deg"], "°", "angle", ".2f",
        f"kritik nokta s = {_fmt(g['cowl_contact_s'], '.3f')}")
    add(G, "Stabilize ucu FK temas açısı", g["stab_tip_te_contact"], gr["stab_tip_contact_deg"], "°", "angle", ".2f")
    add(G, "Stabilize ucu kesiti (en kritik nokta)", g["stab_tip_contact"], None, "°", "info", ".2f")
    add(G, "Dikey kökü FK temas açısı", g["fin_root_contact"], None, "°", "info", ".2f",
        "dikey kökü stabilize ucunun 0,12 m arkasına uzanır; pervane yine önce değer")
    add(G, "Kanat ucu yer açıklığı", g["wingtip_clear_mm"], gr["wingtip_clearance_mm"], "mm", "clearance", ".0f")
    add(G, "Kanat ucuna yatış açısı", g["bank_to_tip"], gr["bank_to_wingtip_deg"], "°", "angle", ".2f")
    add(G, "Taret yer açıklığı", g["turret_clear_mm"], gr["turret_clearance_mm"], "mm", "clearance", ".0f")
    add(G, "Karın açıklığı (s 0,5–0,85)", g["belly_clear_mm"], gr["belly_clearance_mm"], "mm", "clearance", ".0f")
    rm = gr["retracted_main_wheel"]
    add(G, "Katlanmış ana teker s", g["main_retracted"][0], rm[0], "m", "length", ".4f")
    add(G, "Katlanmış ana teker y", g["main_retracted"][1], rm[1], "m", "length", ".4f")
    add(G, "Katlanmış ana teker z", g["main_retracted"][2], rm[2], "m", "length", ".4f")
    add(G, "Katlanmış burun tekeri s", g["nose_retracted"][0], gr["retracted_nose_wheel_s_m"], "m", "rel", ".3f",
        "yüksüz bacak (uçuşta); statik boyla 0,620")
    add(G, "Ana bacak statik çökmesi (dikey)", g["main_sag"], S["landing_gear"]["main"]["static_sag_m"], "m", "length",
        ".4f", "zemin z'den türetilir")
    add(G, "Burun bacağı statik çökmesi (dikey)", g["nose_sag"], None, "m", "info", ".4f")
    add(G, "Ana kuyu payı (en dar)", min(g["main_well_margins"].values()) * 1000, None, "mm", "min", ".1f",
        ", ".join(f"{k} {_fmt(v * 1000, '.1f')}" for k, v in g["main_well_margins"].items()), limit=2.0)
    add(G, "Burun kuyusu payı (en dar)", min(g["nose_well_margins"].values()) * 1000, None, "mm", "min", ".1f",
        ", ".join(f"{k} {_fmt(v * 1000, '.1f')}" for k, v in g["nose_well_margins"].items()), limit=2.0)
    add(G, "Ana pivot kanat altının altında", g["pivot_below_wing_mm"], None, "mm", "max", ".1f",
        "ünite kabartması (unit_blister) bunu ve kabuk etini örtmeli",
        limit=1000 * float(S["wing"]["root_fairing"]["unit_blister"]["depth_m"]) - 1.6)

    G = "Gövde"
    fr = S["fuselage"]["reference"]
    add(G, "Azami genişlik", fu["max_w"], fr["max_width_m"], "m", "length", ".4f")
    add(G, "Azami yükseklik", fu["max_h"], fr["max_height_m"], "m", "length", ".4f",
        f"s = {_fmt(fu['max_h_s'], '.2f')}'de; istasyon tablosunun tepesi 0,212 (panel 0,214)")
    add(G, "Islak alan", fu["wetted"], fr["wetted_area_m2"], "m²", "rel_loose", ".3f", "çevre integrali")
    add(G, "İncelik oranı L/((w+h)/2)", fu["fineness"], fr["fineness_ratio"], "", "rel_loose", ".2f",
        f"eşdeğer çapla L/d_eş = {_fmt(P.FUSELAGE_LENGTH / fu['d_eq'], '.2f')} "
        f"(A_max {_fmt(fu['A_max'], '.4f')} m², s = {_fmt(fu['A_max_s'], '.2f')})")
    tc = S["fuselage"]["tail_cone"]["belly_upsweep_deg"]
    add(G, "Karın kalkışı s 1,10→1,95", fu["upsweep_110"], tc["s_1p10_1p95"], "°", "angle", ".2f")
    add(G, "Karın kalkışı s 1,40→1,95", fu["upsweep_140"], tc["s_1p40_1p95"], "°", "angle", ".2f")
    add(G, "Burun sarkması", fu["nose_droop"], S["fuselage"]["nose"]["droop_m"], "m", "length", ".4f")

    G = "Kütle"
    M = S["mass"]
    add(G, "Kalemler toplamı", ms["sum"], M["mtow_kg"], "kg", "mass", ".3f")
    add(G, "Boş kütle (MTOW − yakıt − faydalı yük)", ms["empty"], M["empty_kg"], "kg", "mass", ".3f")
    add(G, "Yakıt kütlesi (1,40 L × 0,745)", ms["fuel_from_volume"], M["fuel_kg"], "kg", "mass", ".3f")

    G = "Yapı ve baskı sığma"
    add(G, "Merkez soket 30/27, y 0,40'ta kalınlık payı", sr["fit_centre_socket"], None, "mm", "min", ".1f",
        f"yerel kalınlık {_fmt(sr['thick_centre_socket'], '.1f')} mm, OD + 2 kabuk", limit=0.0)
    add(G, "Panel borusu 27/25, y 1,10'da kalınlık payı", sr["fit_panel_tube"], None, "mm", "min", ".1f",
        f"yerel kalınlık {_fmt(sr['thick_panel_tube'], '.1f')} mm", limit=0.0)
    add(G, "Uç borusu 16/14, y 1,70'te kalınlık payı", sr["fit_tip_tube"], None, "mm", "min", ".1f",
        f"yerel kalınlık {_fmt(sr['thick_tip_tube'], '.1f')} mm; ölçüt OD + 2 kabuk + 2 mm (panel: 19,2 ≤ 19,6)", limit=0.0)
    add(G, "Arka kiriş 8/6, y 1,82'de kalınlık payı", sr["fit_rear_spar"], None, "mm", "min", ".1f",
        f"yerel kalınlık {_fmt(sr['thick_rear_spar'], '.1f')} mm", limit=0.0)
    add(G, "Arka kiriş ↔ menteşe oyuğu (en dar)", sr["rear_spar_cove_mm"], None, "mm", "min", ".1f",
        "oyuk = menteşe − yuvarlak burun yarıçapı − 1 mm aralık", limit=1.0)
    add(G, "Stabilize kirişi 12/10, dış uçta kalınlık payı", sr["fit_stab_spar"], None, "mm", "info", ".1f",
        f"boru y ≤ {_fmt(sr.get('stab_spar_fit_to_y', float('nan')), '.3f')}'e kadar sığar; dışı uç mafsalında açılır")
    add(G, "Dikey kirişi 8/6, uçta kalınlık payı", sr["fit_fin_spar"], None, "mm", "min", ".1f", limit=0.0)
    return C


# =====================================================================================================
# Rapor
# =====================================================================================================


def report_md(R: dict, C: list[Check]) -> str:
    """Türkçe Markdown rapor (``ucav/out/sizing.md``)."""
    S = P.SPEC
    n_chk = sum(c.is_check for c in C)
    n_bad = sum(c.is_check and not c.ok for c in C)
    st = R["stab"]
    lines = [
        f"# {S['meta']['name']} — boyutlandırma ve geometri kontrolü",
        "",
        "Bu rapor `python3 ucav/sizing.py` ile üretilir. Bütün değerler `ucav/params.py` geometrisinden yeniden "
        "hesaplanır ve `ucav/spec.yaml`'daki panel referanslarıyla karşılaştırılır. Elle düzenlemeyin.",
        "",
        f"**Sonuç: {n_chk - n_bad}/{n_chk} kontrol tolerans içinde"
        + ("." if n_bad == 0 else f"; {n_bad} kontrol DIŞINDA.") + "**",
        "",
        "Toleranslar (`spec.yaml → checks`): oranlar ve alanlar %{} (ayrıntı tasarımına bağlı olanlar %5), "
        "konumlar {} mm, açıklıklar {} mm, açılar {}°, %MAC değerleri {} puan, hızlar {} m/s, kütle {} g. "
        "\"Sınır\" satırları bir eşiğe göre, \"bilgi\" satırları yalnız rapor içindir.".format(
            _fmt(100 * S["checks"]["rel_default"], "g"), _fmt(1000 * S["checks"]["abs"]["length_m"], "g"),
            _fmt(S["checks"]["abs"]["clearance_mm"], "g"), _fmt(S["checks"]["abs"]["angle_deg"], "g"),
            _fmt(S["checks"]["abs"]["pct_mac"], "g"), _fmt(S["checks"]["abs"]["speed_ms"], "g"),
            _fmt(1000 * S["checks"]["abs"]["mass_kg"], "g")),
        "",
        "Eksenler: `s` burun ucundan geriye, `y` sol +, `z` FRL'den yukarı (m). Blender: X = −s.",
        "",
    ]
    groups: list[str] = []
    for c in C:
        if c.group not in groups:
            groups.append(c.group)
    for gi, gname in enumerate(groups, start=1):
        lines += [f"## {gi}. {gname}", "", "| Büyüklük | Hesap | Spec | Fark | Tolerans / sınır | Durum | Not |",
                  "|---|---:|---:|---:|---:|:---:|---|"]
        for c in (c for c in C if c.group == gname):
            unit = f" {c.unit}" if c.unit else ""
            if c.kind == "info":
                diff, tol, status = "—", "—", "bilgi"
            elif c.kind in ("min", "max"):
                diff = "—"
                tol = ("≥ " if c.kind == "min" else "≤ ") + _fmt(c.limit, c.fmt) + unit
                status = "✓" if c.ok else "✗"
            else:
                d = c.value - c.ref
                diff = _fmt(d, "+" + c.fmt) if float(format(d, c.fmt)) != 0 else _fmt(0.0, c.fmt)
                tol = "± " + _fmt(c.tol(), c.fmt)
                status = "✓" if c.ok else "✗"
            lines.append(f"| {c.label} | {_fmt(c.value, c.fmt)}{unit} | {_fmt(c.ref, c.fmt)} | {diff} | {tol} | "
                         f"{status} | {c.note} |")
        lines.append("")
        if gname == "Kararlılık":
            lines += [
                "Yöntem (`spec.yaml → stability.method`): "
                "x_np = [a_wb·x_ac,w + F_h·x_ac,h + F_g·x_g − Cmα_f·c̄] / (a_wb + F_h + F_g). "
                f"a_wb = {_fmt(S['stability']['inputs']['a_wb_per_rad'])}/rad (kaldırma çizgisi, gövde etkisiyle); "
                f"a_h Helmbold ile AR_etkin = AR_h·(1 + 1,9·{_fmt(S['tail']['aero']['endplate_k'])}·h_dikey/b_h); "
                "dε/dα DATCOM (K_A·K_λ·K_H); F_h = η_h·a_h·S_h,açık/S·(1 − dε/dα); "
                f"Cmα_f = K_f·w_f²·L_f/(c̄·S)·180/π, K_f = {_fmt(S['stability']['inputs']['K_f_per_deg'])}/°; "
                f"glove terimi F_g = {_fmt(S['stability']['inputs']['F_g'])} (x_g = {_fmt(S['stability']['inputs']['x_g_s_m'])}). "
                f"Duyarlılık: η_h 0,85 → SM %{_fmt(st['SM_eta_0.85'], '.1f')}, η_h 1,00 → SM %{_fmt(st['SM_eta_1.00'], '.1f')}.",
                "",
            ]
        if gname == "İniş takımı ve yer açıları":
            lines += [
                "Temas noktaları gerçek bacak geometrisinden: ana bacak pivot (1,2825; ±0,33; −0,042), 0,22 m, 12° geriye "
                "yatık, statik sıkışma bacak boyunca; burun bacağı pivot (0,42; 0; −0,056), 0,208 m. Zemin z = "
                f"{_fmt(P.GROUND_Z, '.4f')}. Açılar ana teker temas çizgisi etrafında burun yukarı dönüşle hesaplanır; pervane "
                "diskinin en alçak noktası 5° eğim nedeniyle göbekten geridedir.",
                "",
            ]
    lines += [
        "## Notlar",
        "",
        "- Panelden sapan ya da panelde olmayan değerler `spec.yaml`'da `# varsayım` ile işaretlidir. Bu rapordaki "
        "\"bilgi\" satırları tasarım kararı için girdidir, tolerans kontrolü yoktur.",
        "- Arka kiriş %68 yerine %65'tedir: %72 menteşenin yuvarlak burun oyuğu %68'deki 8 mm boruyla 2,6 mm "
        "çakışıyordu.",
        "- Dikeylerin kökü (0,24 m veter) stabilize ucunun 0,12 m arkasına uzanır; bu nokta stabilize ucundan önce "
        "yere değer (yukarıdaki \"dikey kökü\" satırı). Pervane çarpma açısı yine en küçük açıdır.",
        "- Stabilize kirişi 12/10 boyunun tamamında NACA 0010 içinde kalmaz; dış ucu dikey mafsalına (G10) girer.",
        "",
    ]
    return "\n".join(lines)


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description="YK-38 boyutlandırma kontrolü: geometriden yeniden hesap ve spec ile "
                                             "karşılaştırma; ucav/out/sizing.md yazar.")
    ap.add_argument("--check", action="store_true", help="tolerans dışı değer varsa çıkış kodu 1 döndür")
    ap.add_argument("--out", type=Path, default=REPORT_PATH, help="rapor yolu (varsayılan ucav/out/sizing.md)")
    ap.add_argument("--quiet", action="store_true", help="yalnız hataları yaz")
    args = ap.parse_args(argv)
    R = compute()
    C = build_checks(R)
    args.out.parent.mkdir(parents=True, exist_ok=True)
    args.out.write_text(report_md(R, C), encoding="utf-8")
    bad = [c for c in C if c.is_check and not c.ok]
    if not args.quiet:
        n = sum(c.is_check for c in C)
        print(f"{P.SPEC['meta']['name']}: {n - len(bad)}/{n} kontrol tolerans içinde → {args.out}")
        w, s = R["wing"], R["stab"]
        print(f"  S {w['S']:.4f} m², AR {w['AR']:.2f}, MAC {w['MAC']:.4f} m @ y {w['y_mac']:.3f}, "
              f"NP {s['x_np']:.4f}, SM %{s['SM']:.1f}, pervane çarpma {R['gear']['prop_strike']:.2f}°")
    for c in bad:
        ref = f"spec {c.ref:{c.fmt}}" if c.ref is not None else f"sınır {c.limit}"
        print(f"  ✗ [{c.group}] {c.label}: {c.value:{c.fmt}} {c.unit} ({ref})", file=sys.stderr)
    return 1 if (args.check and bad) else 0


if __name__ == "__main__":
    sys.exit(main())
