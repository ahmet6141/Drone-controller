"""Layout builder: chassis (primary structure members, wing carry-through and joint, fittings, design loads)."""
from __future__ import annotations

from b_common import *  # noqa: F401,F403

T_SW = 0.0068          # sandwich panel thickness (layups.rib_panel incl. inner plies)
SKIN = 0.0058          # shell_secondary 0.4/5/0.4 mm


def chine_path(x0, x1, n=None, inset=0.020):
    xs = list(np.round(np.linspace(x0, x1, n or max(3, int(round((x1 - x0) / 0.15)) + 1)), 4))
    return [[float(x), r3(chine_halfwidth(x) - inset), r3(zc(x))] for x in xs]


def member(mid, part, name, name_tr, role, material, process, section, geometry, load_path, mirror=False,
           layup=None, thickness=None, notes="", touch=()):
    d = {"id": mid, "part": part, "name": name, "name_tr": name_tr, "role": role, "material": material,
         "process": process}
    if layup:
        d["layup"] = layup
    if thickness is not None:
        d["thickness"] = thickness
    d["section"] = section
    d.update(geometry)
    if mirror:
        d["mirror"] = True
    d["load_path"] = load_path
    if touch:
        d["touch"] = list(touch)
    if notes:
        d["notes"] = notes
    return d


def fin_spar_root(frac, depth=0.015):
    """Point on the fin mid-surface at chord fraction ``frac`` where the fin is ``depth`` below the OML (root band)."""
    F = af.tail["fin"]
    e = F.span_coords()
    for eta in np.linspace(e[0], e[0] + 0.30, 301):
        p = surf_point(F, eta, frac)
        if not af.inside(p[None], margin=depth)[0]:
            return r3(p)
    raise RuntimeError("fin spar root not found")


def members() -> list:
    zb0, zb1 = Z_BOX
    M = []
    nw = ZP["nose_gear_well"]["box"]
    # ---------------------------------------------------------------- longitudinal primary members
    x_ms38, x_rs38 = spar_x(0.38, 0.25), spar_x(0.38, 0.72)
    M.append(member("M-CHINE", "YK250-CH-020", "chine longeron", "kenar çizgisi uzun kirişi",
                    "primary longitudinal member on the chine line (z = chine plane): body bending (with the dorsal "
                    "ridge skin and the belly), upper/lower skin attachment land at the chine, LERX/glove root "
                    "attachment between FS1810 and the main-spar frame; forward piece FS0600 -> main-spar frame, "
                    "aft piece rear-spar frame -> FS3738, spliced to the side-of-body rib of the wing box with "
                    "2 x 4 M6 bolts",
                    "cfrp_ud_mtm45_as4", "prepreg_ooa_vacbag",
                    {"type": "J-section, UD caps + PW web/flanges (spar_cap_ud / spar_web)", "w": 0.035, "h": 0.030,
                     "t": 0.0024},
                    {"paths": [chine_path(0.6034, x_ms38 - 0.030), chine_path(x_rs38 + 0.030, X_RING - 0.0010)]},
                    "skins (shear) -> chine longeron (axial) -> frames / wing box side-of-body rib",
                    mirror=True, layup="spar_cap_ud", touch=["ST-*", "M-SOB", "M-DECK-NOSE", "M-WELLROOF",
                                                            "M-FWDDECK", "M-MIDFLOOR"]))
    z_nw_top = float(nw[1][2])
    M.append(member("M-KEELWALL", "YK250-CH-021", "nose-gear keel wall", "burun takımı omurga duvarı",
                    "vertical keel walls on both sides of the nose-gear slot: nose-gear pivot bushings and "
                    "retraction-actuator anchor, support of the avionics deck, inner walls of the avionics side bays",
                    "cfrp_pw_mtm45_as4", "prepreg_ooa_vacbag",
                    {"type": "flat sandwich wall (rib_panel) with solid laminate lands at the pivot and the "
                             "actuator anchor (7075-T651 doublers)", "t": T_SW},
                    {"box": r3([[0.6034, 0.0415, -0.215], [1.1066, 0.0415 + T_SW, 0.0140]]), "bottom": "skin"},
                    "nose-gear loads -> pivot bushings -> keel walls -> FS0600 / FS1110 + avionics deck -> chine "
                    "longerons", mirror=True, layup="rib_panel",
                    touch=["ST-FS0600", "ST-FS1110", "M-DECK-NOSE", "F-NG-PIVOT", "EQ-*"]))
    M.append(member("M-DECK-NOSE", "YK250-CH-022", "avionics deck", "aviyonik güverte",
                    "horizontal deck above the nose-gear well, chine to chine: PDU and power-switching equipment "
                    "tray, roof of the keel slot and of the side bays, upper chord of the nose box",
                    "cfrp_pw_mtm45_as4", "prepreg_ooa_vacbag", {"type": "flat sandwich deck (rib_panel)", "t": T_SW},
                    {"box": r3([[0.6034, -0.30, 0.0140], [1.1066, 0.30, 0.0140 + T_SW]]), "sides": "skin"},
                    "equipment inertia -> deck -> keel walls + chine longerons -> FS0600 / FS1110",
                    layup="rib_panel", touch=["ST-FS0600", "ST-FS1110", "M-KEELWALL", "M-CHINE"]))
    tb = ZP["turret_bay"]["box"]
    M.append(member("M-TURRETWALL", "YK250-CH-023", "turret-bay side wall", "taret bölmesi yan duvarı",
                    "side walls of the turret bay (box with FS1110 / FS1330 and the bay roof): frame the belly "
                    "cut-out, guide the sliding bay doors (slot 20 mm above the skin), corner rails of the elevator",
                    "cfrp_pw_mtm45_as4", "prepreg_ooa_vacbag", {"type": "flat sandwich wall (rib_panel)", "t": T_SW},
                    {"box": r3([[1.1134, 0.106, -0.165], [1.3266, 0.106 + T_SW, Z_TROOF]]), "bottom": "skin"},
                    "belly cut-out edge loads + turret inertia -> walls -> FS1110 / FS1330", mirror=True,
                    layup="rib_panel", touch=["ST-FS1110", "ST-FS1330", "M-TURRETROOF"]))
    M.append(member("M-TURRETROOF", "YK250-CH-024", "turret-bay roof", "taret bölmesi tavanı",
                    "roof of the turret bay: elevator top mount (BLDC + ball-screw bearing block)",
                    "cfrp_pw_mtm45_as4", "prepreg_ooa_vacbag", {"type": "flat sandwich (rib_panel)", "t": T_SW},
                    {"box": r3([[1.1134, -0.1128, Z_TROOF], [1.3266, 0.1128, Z_TROOF + T_SW]])},
                    "turret elevator reaction -> roof -> walls / frames", layup="rib_panel",
                    touch=["ST-FS1110", "ST-FS1330", "M-TURRETWALL"]))
    M.append(member("M-PARAWALL", "YK250-CH-025", "parachute-bay side wall", "paraşüt bölmesi yan duvarı",
                    "side walls of the parachute compartment, container restraint brackets",
                    "cfrp_pw_mtm45_as4", "prepreg_ooa_vacbag", {"type": "flat sandwich wall (rib_panel)", "t": T_SW},
                    {"box": r3([[X_PFF + 0.0034, 0.156, -0.1848], [X_PFR - 0.0034, 0.156 + T_SW, 0.165]]), "top": "skin"},
                    "container inertia -> walls/floor -> FS1490 / FS1810", mirror=True, layup="rib_panel",
                    touch=["ST-FS1490", "ST-FS1810", "M-PARAFLOOR"]))
    M.append(member("M-PARAFLOOR", "YK250-CH-026", "parachute-bay floor", "paraşüt bölmesi tabanı",
                    "floor of the parachute compartment", "cfrp_pw_mtm45_as4", "prepreg_ooa_vacbag",
                    {"type": "flat sandwich (rib_panel)", "t": T_SW},
                    {"box": r3([[X_PFF + 0.0034, -0.1628, -0.1848], [X_PFR - 0.0034, 0.1628, -0.1848 + T_SW]])},
                    "container inertia -> floor -> FS1490 / FS1810 / walls", layup="rib_panel",
                    touch=["ST-FS1490", "ST-FS1810", "M-PARAWALL"]))
    M.append(member("M-MIDFLOOR", "YK250-CH-027", "mission-bay floor", "görev bölmesi tabanı",
                    "equipment floor of the mission-computer bay (FS1810 -> FS-FUEL), chine to chine; harness "
                    "trunks below it", "cfrp_pw_mtm45_as4", "prepreg_ooa_vacbag",
                    {"type": "flat sandwich (rib_panel)", "t": T_SW},
                    {"box": r3([[X_PFR + 0.0034, -0.36, -0.1348], [X_FUELF - 0.0034, 0.36, -0.1348 + T_SW]]), "sides": "skin"},
                    "mission equipment inertia -> floor -> frames + lower skin", layup="rib_panel",
                    touch=["ST-FS1810", "ST-FS-FUEL", "M-CHINE"]))
    pay = ZP["payload_bay"]["box"]
    x_rs21 = spar_x(0.21, 0.72)
    M.append(member("M-KEEL", "YK250-CH-028", "keel beam (payload-bay side wall)", "omurga kirişi (yük bölmesi duvarı)",
                    "two vertical keel beams along the payload-bay edges (FS-FUEL -> rear-spar frame): belly bending "
                    "around the payload-bay cut-out, payload hatch land, outboard posts of the spar frames, "
                    "payload-tray rail supports", "cfrp_pw_mtm45_as4", "prepreg_ooa_vacbag",
                    {"type": "flat sandwich wall (rib_panel) with UD caps at the skin and at the deck", "t": T_SW},
                    {"box": r3([[X_FUELF + 0.0034, 0.2040, -0.215], [x_rs21 - 0.0034, 0.2040 + T_SW, -0.0532]]),
                     "bottom": "skin"},
                    "payload + belly loads -> keel beams -> FS-FUEL / spar frames -> wing box", mirror=True,
                    layup="rib_panel", touch=["ST-FS-FUEL", "ST-FS-MS", "ST-FS-RS", "M-FWDDECK", "M-CTBOX"]))
    M.append(member("M-FWDDECK", "YK250-CH-029", "forward fuel-bay floor / payload-bay roof",
                    "ön yakıt bölmesi tabanı / yük bölmesi tavanı",
                    "deck FS-FUEL -> main-spar frame, chine to chine: floor of the forward fuel cell, roof of the "
                    "forward part of the payload bay, payload-tray rails (underside)", "cfrp_pw_mtm45_as4",
                    "prepreg_ooa_vacbag", {"type": "flat sandwich (rib_panel), fuel-side face sealed", "t": T_SW},
                    {"box": r3([[X_FUELF + 0.0034, -0.40, -0.0600], [X_MS0 - 0.0034, 0.40, -0.0600 + T_SW]]), "sides": "skin"},
                    "fuel + payload inertia -> deck -> keel beams / FS-FUEL / main-spar frame", layup="rib_panel",
                    touch=["ST-FS-FUEL", "ST-FS-MS", "M-KEEL", "M-CHINE"]))
    M.append(member("M-WELLROOF", "YK250-CH-030", "main-gear well roof deck", "ana takım kuyusu tavanı",
                    "deck rear-spar frame -> FS-GEAR, chine to chine: roof of both main wells, floor of the aft fuel "
                    "cell, up-lock hooks; tension/compression chord of the belly across the wells",
                    "cfrp_pw_mtm45_as4", "prepreg_ooa_vacbag", {"type": "flat sandwich (rib_panel)", "t": T_SW},
                    {"box": r3([[X_RS0 + 0.0034, -0.40, -0.0668], [X_GEARF - 0.0034, 0.40, -0.0668 + T_SW]]), "sides": "skin"},
                    "gear + fuel loads -> well roof -> gear beams / rear-spar frame / FS-GEAR", layup="rib_panel",
                    touch=["ST-FS-RS", "ST-FS-GEAR", "M-GEARBEAM", "F-TRUNNION", "M-CHINE"]))
    M.append(member("M-GEARBEAM", "YK250-CH-031", "main-gear beam (outboard well wall)", "ana takım kirişi",
                    "outboard wall of each main well (rear-spar frame -> FS-GEAR): carries the trunnion fitting; "
                    "takes the gear loads to both frames and the well roof", "cfrp_pw_mtm45_as4",
                    "prepreg_ooa_vacbag", {"type": "sandwich wall (rib_panel) + UD caps, 7075 doubler at the "
                                                   "trunnion fitting", "t": T_SW},
                    {"box": r3([[spar_x(0.38, 0.72) + 0.0034, 0.3765, -0.180], [X_GEARF - 0.0034, 0.3765 + T_SW, -0.0668]]),
                     "bottom": "skin"},
                    "trunnion -> gear beam -> rear-spar frame + FS-GEAR + well roof -> wing box / chine longerons",
                    mirror=True, layup="rib_panel",
                    touch=["ST-FS-RS", "ST-FS-GEAR", "M-WELLROOF", "F-TRUNNION", "M-CHINE"]))
    xs = np.linspace(X_GEARF + 0.0034, X_RING - 0.0010, 6)
    M.append(member("M-DORSAL", "YK250-CH-032", "dorsal longeron", "sırt uzun kirişi",
                    "upper longerons under the dorsal skin (FS-GEAR -> FS3738) at the fin roots: tail bending "
                    "(fin and stub loads) forward into FS-GEAR / the aft fuel-bay frame", "cfrp_ud_mtm45_as4",
                    "prepreg_ooa_vacbag", {"type": "hat section 25 x 20 mm, UD cap + PW", "w": 0.025, "h": 0.020,
                                            "t": 0.0020},
                    {"paths": [[[r3(x), 0.150, r3(z_top(x, 0.150) - SKIN - 0.011)] for x in xs]]},
                    "fin/stub root fittings -> frames FS3480 / FS3670 -> dorsal longerons -> FS-GEAR", mirror=True,
                    layup="spar_cap_ud", touch=["ST-*", "F-FIN-*", "F-STUB-*"]))
    M.append(member("M-PAYWALL-AFT", "YK250-CH-034", "payload-bay aft wall", "faydalı yük bölmesi arka duvarı",
                    "aft wall of the payload bay (keel beam to keel beam); forward wall of the lateral harness duct",
                    "cfrp_pw_mtm45_as4", "prepreg_ooa_vacbag", {"type": "solid laminate wall 1.6 mm (8 plies PW)",
                                                               "t": 0.0016},
                    {"box": r3([[X_PB1, -0.2040, -0.215], [X_PB1 + 0.0016, 0.2040, -0.0532]]), "bottom": "skin"},
                    "secondary (closes the bay; payload loads go to the keel beams and the deck)",
                    layup="spar_web", thickness=0.0016, touch=["M-KEEL", "M-FWDDECK", "M-WELLROOF", "ST-FS-RS"]))
    M.append(member("M-WELLWALL-FWD", "YK250-CH-035", "main-well forward wall", "ana takım kuyusu ön duvarı",
                    "forward wall of both main wells (gear beam to gear beam, under the well roof); aft wall of the "
                    "lateral harness duct", "cfrp_pw_mtm45_as4", "prepreg_ooa_vacbag",
                    {"type": "solid laminate wall 1.6 mm (8 plies PW)", "t": 0.0016},
                    {"box": r3([[X_W0 - 0.0016, -0.3765, -0.215], [X_W0, 0.3765, -0.0668]]), "bottom": "skin"},
                    "secondary (well closure, harness duct wall)", layup="spar_web", thickness=0.0016,
                    touch=["M-GEARBEAM", "M-WELLROOF", "M-KEEL"]))
    M.append(member("M-AFTKEEL", "YK250-CH-033", "aft keel beam (engine bay, lower)", "arka omurga kirişi",
                    "centre-line channel under the engine (firewall -> x 3.935): ventral root fittings (3 points), "
                    "lower cowl land and lip-ring support; metallic (engine bay)", "al_2024_t3_sheet",
                    "sheet_metal_aluminium", {"type": "formed channel 24 x 40 mm, t 1.6 mm (bend radius >= 6 t)",
                                              "w": 0.024, "h": 0.040, "t": 0.0016},
                    {"box": r3([[X_FW + 0.0074, -0.012, -0.093], [3.905, 0.012, -0.055]])},
                    "ventral fin / bumper strike -> aft keel beam -> firewall + ring frame", thickness=0.0016,
                    touch=["ST-FS3670", "ST-FS3738", "F-VENTRAL-*"]))
    # ---------------------------------------------------------------- wing carry-through and glove ribs
    yj = YJ
    ms = [[r3(X_MS0), 0.0, 0.0], [r3(spar_x(Y_SOB, 0.25)), Y_SOB, 0.0], [r3(spar_x(yj, 0.25)), yj, 0.0]]
    rs = [[r3(X_RS0), 0.0, 0.0], [r3(spar_x(Y_SOB, 0.72)), Y_SOB, -0.006], [r3(spar_x(yj, 0.72)), yj, -0.012]]
    prof_m = Z.spar_depth_profile(S, af, float(P["main_spar_frac"]))["rows"]
    prof_r = Z.spar_depth_profile(S, af, float(P["rear_spar_frac"]))["rows"]

    def caps(prof, y, zmid):
        yy = np.array([r["y"] for r in prof])
        d = float(np.interp(y, yy, [r["depth"] for r in prof])) if y >= yy[0] else zb1 - zb0
        m = float(np.interp(y, yy, [r["mid"] for r in prof])) if y >= yy[0] else zmid
        d = min(d, zb1 - zb0)
        return r3([m - 0.5 * d + 0.004, m + 0.5 * d - 0.004])
    z_caps_m = [caps(prof_m, p[1], 0.0) for p in ms]
    z_caps_r = [caps(prof_r, p[1], 0.0) for p in rs]
    M.append(member("M-CTBOX", "YK250-CH-001", "centre wing box (carry-through)", "orta kanat kutusu (geçiş kutusu)",
                    "one-piece centre wing box y -0.70 .. +0.70: main and rear spars on the reference-trapezoid spar "
                    "lines (25 % / 72 % chord; chevron with a centre kink of 2 x 8.0 deg main and 2 x 4.8 deg rear), "
                    "continuous UD spar caps from fork to fork, +-45 webs, box covers inside the body, centre kink "
                    "fitting (7075-T651) at y = 0; carries the outer-panel moments, shears and torques across the "
                    "body; ROOT PART of the assembly", "cfrp_ud_mtm45_as4", "prepreg_ooa_vacbag",
                    {"type": "two-spar box; spar caps UD (spar_cap_ud, area from structures.derived: root "
                             f"{float(S['structures']['derived']['spar_cap_area_root_mm2']):.0f} mm2 per cap), webs "
                             "+-45 PW (spar_web, >= 0.6 mm), covers inside the body PW 2.0 mm quasi-isotropic",
                     "main_cap_width": 0.040, "rear_cap_width": 0.025, "depth_in_body": r3(zb1 - zb0)},
                    {"main_spar_line": ms, "rear_spar_line": rs, "z": r3([zb0, zb1]), "y_extent": [-yj, yj],
                     "main_spar_caps_z": z_caps_m, "rear_spar_caps_z": z_caps_r,
                     "box": r3([[X_MS0 - 0.021, -yj, zb0], [spar_x(yj, 0.72) + 0.013, yj, zb1]])},
                    "outer panel -> tongue + pins -> fork -> spar caps (bending couple) / webs (shear) -> box -> "
                    "spar frames + side-of-body ribs -> body; symmetric bending balanced through the box",
                    layup="spar_cap_ud", touch=["ST-FS-MS", "ST-FS-RS", "M-SOB", "M-GLOVERIB", "M-JOINTRIB",
                                                "F-FORK", "F-DRAGPIN", "M-KEEL", "M-CHINE", "M-WELLROOF",
                                                "M-FWDDECK"]))
    for mid, part, y, nm, ntr, role in (
            ("M-SOB", "YK250-CH-050", Y_SOB, "side-of-body rib", "gövde yanı kaburgası",
             "rib on the body side line between main and rear spar (and forward to the LERX nose): splices the chine "
             "longeron, closes the body-side of the glove box, kink rib of the chine/glove skins"),
            ("M-GLOVERIB", "YK250-CH-051", 0.550, "glove rib", "eldiven kaburgası",
             "intermediate glove rib (LE -> rear spar): fork fitting mid support, access-bay boundary"),
            ("M-JOINTRIB", "YK250-CH-052", yj, "joint rib (centre-section side)", "birleşim kaburgası (orta kesit)",
             "closing rib of the centre section at the outer-panel joint plane (streamwise, y = 0.70): fork mouth, "
             "drag-pin bushing fitting, harness pass-through, joint seal land")):
        x0 = float(af.wing.interpolate_section(y)["x_le"]) + 0.004
        x1 = spar_x(y, 0.72) + 0.013
        M.append(member(mid, part, nm, ntr, role, "cfrp_pw_mtm45_as4", "prepreg_ooa_vacbag",
                        {"type": "flanged sandwich rib (rib_panel), solid laminate at fittings", "t": T_SW},
                        {"box": r3([[x0, y - T_SW / 2, -0.050], [x1, y + T_SW / 2, 0.052]]), "contour": "wing_loft"},
                        "glove skins / fittings -> rib -> spars", mirror=True, layup="rib_panel",
                        touch=["M-CTBOX", "M-CHINE", "F-FORK", "F-DRAGPIN", "P-GLOVE-*"]))
    for i, y in enumerate((0.47, 0.53, 0.61)):
        sec = af.wing.interpolate_section(y)
        M.append(member(f"M-LERXRIB{i + 1}", f"YK250-CH-{56 + i:03d}", f"LERX nose rib {i + 1}",
                        f"LERX burun kaburgası {i + 1}",
                        "nose rib of the LERX/glove ahead of the main spar (supports the LERX skins, glove access "
                        "panel land)", "cfrp_pw_mtm45_as4", "prepreg_ooa_vacbag",
                        {"type": "flanged sandwich rib (rib_panel)", "t": T_SW},
                        {"box": r3([[float(sec["x_le"]) + 0.004, y - T_SW / 2, -0.05],
                                    [spar_x(y, 0.25) - 0.021, y + T_SW / 2, 0.05]]), "contour": "wing_loft"},
                        "LERX skins -> nose ribs -> main spar / side-of-body rib", mirror=True, layup="rib_panel",
                        touch=["M-CTBOX", "P-GLOVE-*", "M-SOB", "M-GLOVERIB", "M-JOINTRIB"]))
    return M


def wing_joint() -> dict:
    """Outer-panel joint at y = 0.70 (starboard; port mirrored)."""
    ys = (0.455, 0.680)
    prof = Z.spar_depth_profile(S, af, float(P["main_spar_frac"]))
    yy = np.array([r["y"] for r in prof["rows"]])
    mid = np.array([r["mid"] for r in prof["rows"]])
    dep = np.array([r["depth"] for r in prof["rows"]])
    d_s = np.array([spar_x(1.0, 0.25) - spar_x(0.0, 0.25), 1.0, 0.0])
    d_s /= np.linalg.norm(d_s)
    n_pin = np.array([d_s[1], -d_s[0], 0.0])                      # chordwise, perpendicular to the spar in plan
    pins = []
    for k, y in enumerate(ys):
        pins.append({"id": f"P-MAIN{k + 1}", "position": r3([spar_x(y, 0.25), y, float(np.interp(y, yy, mid))]),
                     "axis": r3(n_pin), "diameter": 0.014,
                     "spec": "headed pin, Ti-6Al-4V (ti_6al_4v_annealed_sheet allowables used as bar estimate), "
                             "d 14 h8 x 52 mm, keeper plate 2 x M4 on the fork front face",
                     "fit": "pin 14 h8 in 14 H8 reamed bores (fork prongs) and 14 H8 bush (tongue)",
                     "oml_depth_at_pin": r3(float(np.interp(y, yy, dep)))})
    xr = spar_x(YJ, 0.72)
    return {
        "plane": {"y": YJ, "kind": "streamwise joint rib faces (centre-section rib M-JOINTRIB / outer-panel root "
                                   "rib), 1.5 mm elastomer seal strip on the outer-panel face"},
        "insertion": {"axis_inboard": r3(-d_s), "stroke": 0.250,
                      "text": "outer panel slides inboard along the main-spar line (plan sweep 8.0 deg, no "
                              "dihedral in the tongue: the 3 deg dihedral kink is machined into the tongue root at "
                              "the outer-panel root rib); the tongue enters the fork and the drag pin its bushing in "
                              "the same stroke"},
        "main_spar": {
            "kind": "tongue-in-fork (sleeve) joint, bending reacted as a vertical couple on two chordwise shear pins "
                    "250 mm apart",
            "fork": {"part": "YK250-CH-053", "owner": "chassis", "material": "al_7075_t651_plate",
                     "process": "cnc_milling_metal",
                     "geometry": "two prongs (front/rear webs) 6 mm thick, 61 mm high, from y 0.43 to the joint "
                                 "rib, bonded (EA 9394) and bolted (2 x 6 M6, 12.9, nutplates) to the glove "
                                 "main-spar caps and web; mouth chamfer 3 x 30 deg",
                     "slot": {"width": 0.0304, "height": 0.0610, "y": [0.43, YJ]}},
            "tongue": {"part": "YK250-WG-151", "owner": "wing", "material": "al_7075_t651_plate",
                       "process": "cnc_milling_metal",
                       "geometry": "solid tongue 30 mm wide x 61 mm high, 250 mm engagement, bonded and bolted "
                                   "to the outer-panel spar caps (3 x 4 M6); 3 deg dihedral kink at the root rib"},
            "pins": pins,
            "access": "pins inserted from the front through the glove wing-joint access panel (SH-422-R/L), head "
                      "forward, keeper plate on the fork front face; 75 mm free space ahead of the fork for the pin "
                      "and the drift"},
        "rear_spar": {
            "kind": "drag / shear pin parallel to the insertion axis (no manual action at assembly)",
            "pin": {"part": "YK250-WG-152", "owner": "wing", "position": r3([xr, YJ, -0.012]),
                    "axis": r3(-d_s), "diameter": 0.010, "engagement": 0.030,
                    "spec": "17-4PH H1025 or Ti-6Al-4V pin d 10 f7, bonded + pinned into the outer-panel rear-spar "
                            "root fitting; tapered nose 15 mm"},
            "bushing_fitting": {"part": "YK250-CH-054", "owner": "chassis", "material": "al_7075_t651_plate",
                                "process": "cnc_milling_metal",
                                "geometry": "block fitting on the joint rib at the rear spar with a pressed "
                                            "bronze bush 10 H8, bolted 4 x M5 to the rib and the rear-spar web"}},
        "electrical": "wing harness (aileron + flap actuators RS-485/28 V, nav/strobe light, port: HASA pitot "
                      "DroneCAN) ends in a MIL-DTL-38999 III connector pair (plug on the outer-panel pigtail through "
                      "the root rib, receptacle on a bracket on the glove rib M-GLOVERIB) inside the joint access bay",
        "inspection": "joint access panel SH-422 (lower glove, ahead of the main spar, Camloc): visual check of pin "
                      "keepers, fork mouth, seal and connector at every assembly; fork bores eddy-current inspected "
                      "at the scheduled structural inspection (assembly.maintenance_access)",
        "pre_sizing": "layout_check.fitting_presizing(): pin double shear, pin bending, bearing in tongue and prongs "
                      "with limit x 1.5 x frequent-assembly factor 1.5 (bending, shear) and x 2.0 bearing factor "
                      "(structures, baseline.yaml#design_loads.factors)"}


def fittings() -> list:
    """Interface fittings (points/boxes), starboard where mirrored."""
    F = []
    mg = MG["trunnion"]
    F.append({"id": "F-TRUNNION", "part": "YK250-CH-070", "name": "main-gear trunnion fitting",
              "name_tr": "ana takım mafsal bağlantısı", "material": "al_7075_t651_plate", "process": "cnc_milling_metal",
              "mirror": True, "axis": [1.0, 0.0, 0.0], "pivot": r3(mg),
              "bearings": [{"x": r3(mg[0] - 0.049), "bore": 0.020, "fit": "20 H7 (sleeve bushing; match to the "
                            "selected gear unit)"}, {"x": r3(mg[0] + 0.049), "bore": 0.020, "fit": "20 H7"}],
              "box": r3([[mg[0] - 0.062, 0.326, mg[2] - 0.026], [mg[0] + 0.062, 0.3765, -0.0668]]),
              "attach": "bolted 6 x M6 12.9 to the gear beam (outboard) through a 7075 doubler and 4 x M6 to the well "
                        "roof deck (top), nutplates/insert sleeves in the sandwich",
              "actuator_anchor": {"kind": "rotary EMA of the gear unit on the trunnion axis, flange on the fitting "
                                          "front face", "point": r3([mg[0] - 0.075, mg[1], mg[2]])},
              "locks": "down-lock: spring-loaded lock pin in the fitting engaging the leg yoke (gear unit item); "
                       "up-lock: hook on the well roof deck engaging a roller on the leg"})
    nv = NG["pivot"]
    F.append({"id": "F-NG-PIVOT", "part": "YK250-CH-071", "name": "nose-gear pivot bushings",
              "name_tr": "burun takımı mafsal burçları", "material": "al_7075_t651_plate",
              "process": "cnc_milling_metal", "axis": [0.0, 1.0, 0.0], "pivot": r3(nv), "bore": 0.016,
              "box": r3([[nv[0] - 0.023, -0.0515, nv[2] - 0.0115], [nv[0] + 0.023, 0.0515, nv[2] + 0.0145]]),
              "attach": "two flanged bushings (16 H7) bonded into 7075 doubler blocks on the keel walls (3 mm "
                        "proud of the outboard faces, the V belly leaves no more room), 4 x M6 each, pivot pin through "
                        "both",
              "actuator_anchor": {"kind": "rotary EMA of the nose-gear unit on the pivot axis, flange on the "
                                          "starboard bushing block (outboard of the keel wall)",
                                  "point": r3([nv[0], 0.075, nv[2]])},
              "locks": "down-lock: over-centre lock link to the keel walls (gear unit); up-lock: hook under the "
                       "avionics deck engaging the axle roller"})
    pv = STAB["pivot"]
    rs = float(STAB["params"]["spindle_housing_radius"])
    F.append({"id": "F-SPINDLE-IN", "part": "YK250-CH-095", "name": "stabilator spindle inboard bearing housing",
              "name_tr": "stabilatör mili iç yatak yuvası", "material": "al_7075_t651_plate",
              "process": "cnc_milling_metal", "mirror": True, "axis": [0.0, 1.0, 0.0],
              "bearing": "61805-2RS thin-section deep-groove ball bearing 25 x 37 x 7 mm (tail.surfaces.stabilator."
                         "controls.sources.spindle, estimate)",
              "box": r3([[pv[0] - rs, 0.110, pv[2] - rs], [pv[0] + rs, 0.140, pv[2] + rs]]),
              "cylinder": {"center": r3([pv[0], 0.125, pv[2]]), "axis": [0.0, 1.0, 0.0], "radius": rs,
                           "half_length": 0.015},
              "attach": "riveted/bolted (6 x M5) to the engine-bay ring frame FS3738 web",
              "spindle": {"part": "YK250-TL-301", "owner": "tail", "diameter": 0.025,
                          "span_y": [0.110, round(float(STAB["params"]["y_root"]) + 0.10, 4)],
                          "outboard_bearing": "61805-2RS in the stub tip rib (tail module, y 0.31-0.33)",
                          "stabilator_interface": "spindle end engages 0.10 m into the stabilator root socket; "
                                                  "torque by a 25 mm involute spline, retained by one cross-bolt M6 "
                                                  "(access cover on the stabilator root rib)"},
              "horn": {"y": 0.165, "radius": float(STAB["controls"]["linkage"]["servo_arm_m"]) *
                       float(STAB["controls"]["linkage_ratio"])}})
    for fid, part, frac, frame, ntr in (("F-FIN-FRONT", "YK250-CH-096", None, "FS3480", "dikey ön kiriş bağlantısı"),
                                        ("F-FIN-REAR", "YK250-CH-097", None, "FS3670", "dikey arka kiriş bağlantısı")):
        x_fr = 3.480 if frame == "FS3480" else X_FW
        lo_, hi_ = 0.02, 0.98                                    # chord fraction whose root point lies on the frame
        for _ in range(40):
            mid_ = 0.5 * (lo_ + hi_)
            lo_, hi_ = (mid_, hi_) if fin_spar_root(mid_)[0] < x_fr else (lo_, mid_)
        frac = 0.5 * (lo_ + hi_)
        p = fin_spar_root(frac)
        F.append({"id": fid, "part": part, "name": f"fin {'front' if frame == 'FS3480' else 'rear'}-spar root "
                  "fitting", "name_tr": ntr, "material": "al_7075_t651_plate", "process": "cnc_milling_metal",
                  "mirror": True, "frame": frame, "fin_chord_fraction": r3(frac, 3), "point": p,
                  "box": r3([[x_fr - 0.025, p[1] - 0.015, p[2] - 0.030], [x_fr + 0.025, p[1] + 0.015, p[2] - 0.002]]),
                  "attach": "angle fitting bolted 4 x M6 to the frame and the dorsal longeron; fin spar root lug "
                            "(tail module) bolted to it with 2 x M6 12.9 in double shear (axis along x)"})
    for fid, part, x_fr, ntr in (("F-STUB-FRONT", "YK250-CH-098", 3.480, "sabit kök parçası ön bağlantısı"),):
        frac = (x_fr - float(STUB["params"]["x_le"])) / float(STUB["params"]["chord"])
        y_fit = round(half_width(x_fr, float(STUB["params"]["z"])) - 0.020, 4)
        F.append({"id": fid, "part": part, "name": "stabilator-stub front-spar fitting", "name_tr": ntr,
                  "material": "al_7075_t651_plate", "process": "cnc_milling_metal", "mirror": True, "frame": "FS3480",
                  "stub_chord_fraction": r3(frac, 3), "point": r3([x_fr, y_fit, float(STUB["params"]["z"])]),
                  "box": r3([[x_fr - 0.020, y_fit - 0.025, float(STUB["params"]["z"]) - 0.015],
                             [x_fr + 0.020, y_fit + 0.012, float(STUB["params"]["z"]) + 0.015]]),
                  "attach": "fitting on FS3480 at the body side; stub front spar lug 2 x M6 (axis y); the stub rear "
                            "attachment is the spindle housing (F-SPINDLE-IN) through the stub root rib"})
    zv = float(VEN["params"]["z_root"])
    for k, (x_v, frame) in enumerate(((X_FW + 0.012, "FS3670"), (X_RING + 0.010, "FS3738"), (3.890, "M-AFTKEEL"))):
        F.append({"id": f"F-VENTRAL-{k + 1}", "part": f"YK250-CH-{99 + k:03d}", "name": f"ventral fin root fitting "
                  f"{k + 1}", "name_tr": f"ventral kök bağlantısı {k + 1}", "material": "al_7075_t651_plate",
                  "process": "cnc_milling_metal", "frame": frame, "point": r3([x_v, 0.0, zv]),
                  "box": r3([[x_v - 0.012, -0.012, zv - 0.010], [x_v + 0.012, 0.012, -0.055]]),
                  "attach": "lug on the aft keel beam; ventral root lug 1 x M6 12.9 (double shear, axis y)"})
    d_s = np.array([spar_x(1.0, 0.25) - spar_x(0.0, 0.25), 1.0, 0.0])
    d_s /= np.linalg.norm(d_s)
    n_c = np.array([d_s[1], -d_s[0], 0.0])
    a_ = np.array([spar_x(0.43, 0.25), 0.43, 0.0])
    b_ = np.array([spar_x(YJ, 0.25), YJ, 0.0])
    zf = float(np.mean([r["mid"] for r in Z.spar_depth_profile(S, af, float(P["main_spar_frac"]))["rows"]]))
    cen = 0.5 * (a_ + b_) + np.array([0.0, 0.0, zf])
    half = [0.5 * float(np.linalg.norm(b_ - a_)), 0.5 * 0.0304 + 0.006, 0.5 * 0.0610 + 0.006]
    axes = np.column_stack([d_s, n_c, [0.0, 0.0, 1.0]])
    corners = [cen + sa * half[0] * axes[:, 0] + sb * half[1] * axes[:, 1] + sc * half[2] * axes[:, 2]
               for sa in (-1, 1) for sb in (-1, 1) for sc in (-1, 1)]
    F.append({"id": "F-FORK", "part": "YK250-CH-053", "name": "outer-panel joint fork (main spar)",
              "name_tr": "dış panel birleşim çatalı (ana kiriş)", "material": "al_7075_t651_plate",
              "process": "cnc_milling_metal", "mirror": True,
              "obb": {"center": r3(cen), "axes": r3(axes.T.tolist()), "half": r3(half)},
              "box": r3([np.min(corners, axis=0).tolist(), np.max(corners, axis=0).tolist()]),
              "attach": "layout.chassis.wing_joint.main_spar.fork (bonded + 2 x 6 M6 to the glove main-spar caps/web)"})
    xr = spar_x(YJ, 0.72)
    F.append({"id": "F-DRAGPIN", "part": "YK250-CH-054", "name": "drag-pin bushing fitting (rear spar)",
              "name_tr": "sürükleme pimi burç bağlantısı (arka kiriş)", "material": "al_7075_t651_plate",
              "process": "cnc_milling_metal", "mirror": True, "point": r3([xr, YJ, -0.012]),
              "box": r3([[xr - 0.012, YJ - 0.030, -0.026], [xr + 0.012, YJ - 0.002, 0.002]]),
              "attach": "layout.chassis.wing_joint.rear_spar.bushing_fitting (4 x M5 to the joint rib and rear-spar web)"})
    F.append({"id": "F-RISER-FWD", "part": "YK250-CH-112", "name": "parachute bridle forward attach fitting",
              "name_tr": "paraşüt ön kayış bağlantısı", "material": "al_7075_t651_plate", "process": "cnc_milling_metal",
              "frame": "FS1810", "point": r3([X_PFR, 0.0, 0.175]),
              "box": r3([[X_PFR - 0.015, -0.020, 0.152], [X_PFR + 0.015, 0.020, 0.190]]),
              "attach": "U-lug fitting through-bolted 4 x M6 12.9 to FS1810 (7075 doublers both faces), shackle pin "
                        "d 10 (steel), bridle forward leg"})
    F.append({"id": "F-RISER-AFT", "part": "YK250-CH-113", "name": "parachute bridle aft attach fitting",
              "name_tr": "paraşüt arka kayış bağlantısı", "material": "al_7075_t651_plate", "process": "cnc_milling_metal",
              "frame": "FS-RS", "point": r3([X_RS0, 0.0, 0.178]),
              "box": r3([[X_RS0 - 0.015, -0.020, 0.160], [X_RS0 + 0.015, 0.020, 0.192]]),
              "attach": "U-lug fitting through-bolted 4 x M6 12.9 to the rear-spar frame top (7075 doublers), "
                        "shackle pin d 10, bridle aft leg"})
    for k, (x_h, y_h, z_h) in enumerate(((3.670, 0.130, 0.290), (3.670, 0.110, 0.120))):
        F.append({"id": f"F-EMOUNT-{'UP' if k == 0 else 'LO'}", "part": f"YK250-CH-{86 + k:03d}",
                  "name": f"engine-mount firewall attach fitting ({'upper' if k == 0 else 'lower'})",
                  "name_tr": "motor bağlantı parçası (yangın perdesi)", "material": "al_7075_t651_plate",
                  "process": "cnc_milling_metal", "mirror": True, "frame": "FS3670",
                  "point": r3([x_h, y_h, z_h]),
                  "box": r3([[x_h - 0.0198, y_h - 0.020, z_h - 0.020], [x_h + 0.010, y_h + 0.020, z_h + 0.020]]),
                  "attach": "mount foot bolted through the firewall stack with 2 x M8 12.9, 7075 backing plate "
                            "40 x 40 x 5 mm on the forward face, stainless spacer tubes through the sandwich and air gap"})
    return F
