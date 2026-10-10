"""Layout builder: chassis (primary structure members, wing carry-through and joint, fittings, design loads)."""
from __future__ import annotations

from .b_common import *  # noqa: F401,F403

T_SW = 0.0068          # sandwich panel thickness (layups.rib_panel incl. inner plies)
SKIN = 0.0058          # shell_secondary 0.4/5/0.4 mm
# fix round 2 (PK2-03/05): inboard face of the main-gear beam (outboard well wall) moved from y 0.3765 into the lower
# chine corner, so that the splayed leg (r 22.5 mm, 18 deg) keeps >= 10 mm to the wall, its bottom edge on the skin
# and the trunnion-fitting flange over the gear-down position and the whole retraction (layout_check C05)
Y_GB = 0.385
X_NOTCH, Z_NOTCH = 0.042, -0.134    # reinforced notch in the beam's lower edge around the leg (half length, top z)
X0_T, Y0_T, Z0_T = (float(v) for v in MG["trunnion"])


def chine_path(x0, x1, n=None, inset=0.020):
    xs = list(np.round(np.linspace(x0, x1, n or max(3, int(round((x1 - x0) / 0.15)) + 1)), 4))
    return [[float(x), r3(chine_halfwidth(x) - inset), r3(zc(x))] for x in xs]


def _true_depth_caps(line, zc, tc) -> None:
    """Fix round 3 (VS3-01): the cap faces are drawn on the spar-depth profile of the loft; where the union OML of the
    layout check (body + wing, true 3-D distance) lies closer, the cap centroid is moved inward until the cap outer face
    keeps the solid skin over the caps (skin_solid_over_caps_m) as true distance. The shift is a fraction of a
    millimetre (interface check I-CAPZ of ucav250.analysis.structures allows 0.5 mm)."""
    from ..analysis.layout_check import Ctx
    ctx = _CTX.setdefault("lc", Ctx(S))
    t_s = float(_wd().get("skin_solid_over_caps_m", 0.001))
    for p_, z_, t in zip(line, zc, tc):
        for k, sg in ((1, 1.0), (0, -1.0)):
            for _ in range(3):
                d = float(ctx.depth(np.array([[p_[0], p_[1], z_[k] + sg * 0.5 * t]]))[0])
                if d >= t_s - 1e-6:
                    break
                z_[k] = round(z_[k] - sg * (t_s - d + 0.00002), 4)
    # between the points: the cap is straight, the loft is not; lower both ends of a segment by its worst shortfall
    from ..analysis.layout_check import Capsule
    for i in range(len(line) - 1):
        r_ = 0.5 * max(tc[i], tc[i + 1])
        for k, sg in ((1, 1.0), (0, -1.0)):
            for _ in range(3):
                a_ = np.array([line[i][0], line[i][1], zc[i][k]])
                b_ = np.array([line[i + 1][0], line[i + 1][1], zc[i + 1][k]])
                Q = Capsule(a_, b_, r_).samples(0.001)
                d = float(ctx.depth(Q).min())
                if d >= t_s + 0.00005:
                    break
                for j in (i, i + 1):
                    zc[j][k] = round(zc[j][k] - sg * (t_s - d + 0.0001), 4)


_CTX: dict = {}


def chine_paths() -> list:
    """The two chine-longeron pieces (forward: nose-gear bay -> 30 mm ahead of FS-MS; aft: 30 mm aft of FS-RS -> the
    firewall forward face); used for M-CHINE and for the longeron notches of the frames (b_stations.chine_notches)."""
    return [chine_path(0.6034, CHINE_FWD1), chine_path(CHINE_AFT0, X_FW_FWD - 0.0010)]


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


DUCT_SHIFT = 0.0035                            # payload-bay aft wall ahead of the zone end (PK3-05: duct >= 30 mm)
Y_WK = 0.015                                   # inner face of the well keel webs (PK3-05: 30 mm harness channel)
CHINE_FWD1 = spar_x(0.38, 0.25) - 0.030        # end of the forward chine piece (30 mm ahead of FS-MS)
CHINE_AFT0 = spar_x(0.38, 0.72) + 0.030        # start of the aft chine piece (30 mm aft of FS-RS)
SPL_PITCH, SPL_EDGE = 0.018, 0.015             # chine splices: 4 x M6 at 3 D pitch, 2.5 D composite edge (PK3-04)


def _frame_edge_x(x_web, sign):
    """x of a panel edge on a frame cap (b_shell._frame_edge)."""
    return round(float(x_web) + sign * 0.003, 4)


def _wd() -> dict:
    """structures.sizing.wing of the loaded spec (the ply schedule the cap geometry is drawn with)."""
    return ((S.get("structures") or {}).get("sizing") or {}).get("wing") or {}


def _prong_plies() -> int:
    """Fork prong ply count of structures.sizing.wing_joint.fork (8 since fix round 3, VS3-01)."""
    fk = (((S.get("structures") or {}).get("sizing") or {}).get("wing_joint") or {}).get("fork") or {}
    return int(fk.get("prong_plies", 8))


def plies_at(zones: list, y: float) -> int:
    """Ply count of the zone holding y (y0 <= y < y1; the last zone holds its end) - structures.cap_plies_at."""
    for y0, y1, n in zones:
        if y0 <= y < y1:
            return int(n)
    return int(zones[-1][2])


def cap_centroids(prof: list, y: float, which: str) -> tuple:
    """(lower, upper) cap centroid z and cap thickness at span station y (fix round 3, VS3-01): face = box-cover
    surface inside the body, glove loft outboard of the SOB ramp (linear between Y_SOB and Y_RAMP); centroid = face
    -+ (skin_solid_over_caps_m + t/2). The station y = YJ takes the glove zone (inboard side of the joint rib)."""
    wd = _wd()
    zb0, zb1 = Z_BOX
    t_s = float(wd.get("skin_solid_over_caps_m", 0.001))
    t_ply = float(S["materials"]["cfrp_ud_mtm45_as4"]["ply_t"])
    yq = y - 1e-6 if y >= YJ - 1e-9 else y
    if which == "main":
        zones = (wd.get("main_cap") or {}).get("zones") or [[0.0, 3.6, 58]]
        t = plies_at(zones, yq) * t_ply
    else:
        t = int((wd.get("rear_cap") or {}).get("plies", 2)) * t_ply
    if y < Y_SOB - 1e-9:
        fu, fl = zb1, zb0
    else:
        yy = np.array([r["y"] for r in prof])
        d = float(np.interp(y, yy, [r["depth"] for r in prof]))
        m = float(np.interp(y, yy, [r["mid"] for r in prof])) + float(P.get("z_root", 0.0))
        f = min(max((y - Y_SOB) / (Y_RAMP - Y_SOB), 0.0), 1.0)
        fu, fl = zb1 + f * (m + 0.5 * d - zb1), zb0 + f * (m - 0.5 * d - zb0)
    if which == "rear" and y <= Y_RAMP + 1e-9:
        # the body upper surface drops to the glove loft near the chine aft of mid-chord (|y| > 0.33 at the rear
        # spar): the rear caps (and the upper cover there) follow the OML under the solid skin over the caps
        yy = np.array([r["y"] for r in prof])
        d = float(np.interp(y, yy, [r["depth"] for r in prof]))
        m = float(np.interp(y, yy, [r["mid"] for r in prof])) + float(P.get("z_root", 0.0))
        x = spar_x(y, float(P["rear_spar_frac"]))
        top = max(float(z_top(x, y)) if y < Y_SOB else -1.0, m + 0.5 * d)
        bot = min(float(z_bot(x, y)) if y < Y_SOB else 1.0, m - 0.5 * d)
        fu, fl = min(fu, top), max(fl, bot)
    return r3([fl + t_s + 0.5 * t, fu - t_s - 0.5 * t]), t


def members() -> list:
    zb0, zb1 = Z_BOX
    M = []
    nw = ZP["nose_gear_well"]["box"]
    # ---------------------------------------------------------------- longitudinal primary members
    x_ms38, x_rs38 = spar_x(0.38, 0.25), spar_x(0.38, 0.72)
    spl = []
    for sid, xs_, sg, txt in (("SPL-CH-FWD", CHINE_FWD1, -1.0, "forward piece end"),
                              ("SPL-CH-AFT", CHINE_AFT0, 1.0, "aft piece start")):
        pts = [[r3(xs_ + sg * (SPL_EDGE + k * SPL_PITCH)), r3(chine_halfwidth(xs_) - 0.020 + 0.5 * 0.035), 0.0]
               for k in range(4)]
        spl.append({"id": sid, "with": "M-SOB", "piece": txt, "d": 0.006, "axis": [0.0, 1.0, 0.0],
                    "spec": "M6 Ti-6Al-4V (NAS1956 type) + self-locking nut, axis y, through the chine-longeron web "
                            "(J outboard leg, 2.4 mm) and the 16-ply solid land of the side-of-body rib",
                    "pitch_m": SPL_PITCH, "edge_m": SPL_EDGE, "bolts": pts})
    M.append(member("M-CHINE", "YK250-CH-020", "chine longeron", "kenar çizgisi uzun kirişi",
                    "primary longitudinal member on the chine line (z = chine plane): body bending (with the dorsal "
                    "ridge skin and the belly), upper/lower skin attachment land at the chine, LERX/glove root "
                    "attachment between FS1810 and the main-spar frame; forward piece FS0600 -> main-spar frame, "
                    "aft piece rear-spar frame -> forward face of the firewall FS3670; both pieces are spliced to the "
                    "side-of-body rib of the wing box with 4 x M6 Ti each (fix round 3, PK3-04: splices SPL-CH-FWD / "
                    "SPL-CH-AFT; the side-of-body rib extends aft of the rear spar as the aft splice land), so that "
                    "the axial longeron load passes through the side-of-body rib past the spar frames; the pieces "
                    "pass the frames they cross in longeron notches open to the frame edge (layout.stations C-CHINE: "
                    "placed laterally after the frames, 7075 shear clip, U-doubler); fix round 1 (VPK-01/VPK-06): no "
                    "carbon aft of the firewall - the aft end is spliced (2 x M5 Ti) to the forward tongue of the "
                    "metallic stabilator node fitting F-SPINDLE-NODE, which is through-bolted with the firewall stack",
                    "cfrp_ud_mtm45_as4", "prepreg_ooa_vacbag",
                    {"type": "J-section, UD caps + PW web/flanges (spar_cap_ud / spar_web)", "w": 0.035, "h": 0.030,
                     "t": 0.0024},
                    {"paths": chine_paths(),
                     "splices": spl},
                    "skins (shear) -> chine longeron (axial) -> frames / wing box side-of-body rib",
                    mirror=True, layup="spar_cap_ud", touch=["ST-*", "M-SOB", "M-DECK-NOSE", "M-WELLROOF",
                                                            "M-FWDDECK", "M-MIDFLOOR", "F-SPINDLE-NODE"]))
    # fix round 3 (PK3-02): slot-end sill between the keel walls = aft land of the nose clamshell doors, which end
    # 1 mm ahead of the turret ring insert (b_shell.RING_X: the ring owns the aft part of the FS1110 forward cap); the
    # sill starts >= 12 mm aft of the nose-tyre swing at the skin line (layout_check C05)
    xd1 = round(_frame_edge_x(1.1066, -1.0) - 0.009 - 0.001, 4)
    xs0_ = round(xd1 - 0.013, 4)
    M.append(member("M-NGSILL", "YK250-CH-038", "nose-gear slot-end land strip", "burun takımı yarık sonu kenar şeridi",
                    "fix round 3 (PK3-02): flat CFRP land strip 13 mm wide across the keel slot at its aft end, bonded "
                    "to the forward T-flange of FS1110 and to the two keel-wall lands at the skin line: aft land (12 mm "
                    "edge band) of the nose clamshell doors; flat (2.4 mm) so that the nose-door bellcrank / links "
                    "(NDOOR-LINKAGE, >= 10 mm above the skin) pass over it; the turret ring insert lands on the FS1110 "
                    "cap 1 mm aft of it; >= 12 mm aft of the nose-tyre swing (layout_check C05)",
                    "cfrp_pw_mtm45_as4", "prepreg_ooa_vacbag",
                    {"type": "flat strip 13 x 2.4 mm, 12 plies PW", "w": 0.013, "h": 0.0024, "t": 0.0024},
                    {"box": r3([[xs0_, -0.0415, z_bot(xs0_, 0.0415) + SKIN], [xd1, 0.0415,
                                                                             z_bot(xs0_, 0.0415) + SKIN + 0.0024]]),
                     "bottom": "skin"},
                    "door edge (air loads, seal pressure) -> land strip -> FS1110 forward flange / keel walls",
                    layup="spar_web", thickness=0.0024, touch=["M-KEELWALL", "ST-FS1110"]))
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
                    "cfrp_pw_mtm45_as4", "prepreg_ooa_vacbag",
                    {"type": "flat sandwich deck (rib_panel); over the keel slot (between the keel walls) a solid PW "
                             "strip of 6 plies (1.2 mm) at the upper-face level, core ramped out 1:3 over the keel walls "
                             "(fix round 2: 12 mm clearance to the stowed nose tyre; structures.sizing.body."
                             "deck_slot_strip)", "t": T_SW},
                    {"box": r3([[0.6034, -0.30, 0.0140], [1.1066, 0.30, 0.0140 + T_SW]]), "sides": "skin",
                     "boxes": r3([[[0.6034, -0.30, 0.0140], [1.1066, -0.0415, 0.0140 + T_SW]],
                                  [[0.6034, 0.0415, 0.0140], [1.1066, 0.30, 0.0140 + T_SW]],
                                  [[0.6034, -0.0415, 0.0140 + T_SW - 0.0012], [1.1066, 0.0415, 0.0140 + T_SW]]])},
                    "equipment inertia -> deck -> keel walls + chine longerons -> FS0600 / FS1110",
                    layup="rib_panel", touch=["ST-FS0600", "ST-FS1110", "M-KEELWALL", "M-CHINE"]))
    tb = ZP["turret_bay"]["box"]
    M.append(member("M-TURRETWALL", "YK250-CH-023", "turret-bay side wall", "taret bölmesi yan duvarı",
                    "side walls of the turret bay (box with FS1110 / FS1330 and the bay roof): corner rails of the "
                    "elevator; fix round 3 (PK3-09): the sliding bay doors cross the wall line in the door rail plane "
                    "(20 mm above the skin) along the whole door band, so the wall's lower edge is a FREE edge over the "
                    "band (10 mm end posts at FS1110 / FS1330 only, closed by a bonded edge cap) and the wall does not "
                    "frame the belly cut-out; the cut-out is framed by FS1110 / FS1330 (fore / aft) and by the bonded "
                    "CFRP skin doublers of the ring side lands (8 plies PW, 35 mm wide, |y| 0.100-0.135, under the door "
                    "rail plane, potted inserts for the ring side fasteners), part of the turret-bay frame allowance "
                    "(mass.rules.turret_bay_frame_kg)",
                    "cfrp_pw_mtm45_as4", "prepreg_ooa_vacbag", {"type": "flat sandwich wall (rib_panel)", "t": T_SW},
                    {"box": r3([[1.1134, 0.106, -0.165], [1.3266, 0.106 + T_SW, Z_TROOF]]), "bottom": "skin",
                     "lower_edge": "free over the door band x 1.123-1.317 (door rail plane 20 mm above the skin); end "
                                   "posts to the skin at FS1110 / FS1330",
                     "lands": [{"x": [1.1134, 1.3266], "y": [0.100, 0.135], "surface": "lower", "mirror": True,
                                "text": "side_land: bonded skin doubler (8 plies PW) under the door rail plane = side "
                                        "land of the turret ring insert (potted M4 inserts)"}]},
                    "turret inertia + elevator reactions -> walls -> FS1110 / FS1330 (belly cut-out edges: frames + "
                    "skin doublers, not the walls)", mirror=True,
                    layup="rib_panel", touch=["ST-FS1110", "ST-FS1330", "M-TURRETROOF"]))
    M.append(member("M-TURRETROOF", "YK250-CH-024", "turret-bay roof", "taret bölmesi tavanı",
                    "roof of the turret bay: elevator top mount (BLDC + ball-screw bearing block)",
                    "cfrp_pw_mtm45_as4", "prepreg_ooa_vacbag", {"type": "flat sandwich (rib_panel)", "t": T_SW},
                    {"boxes": r3([[[1.1134, -0.104, Z_TROOF], [1.3266, 0.104, Z_TROOF + T_SW]],
                                  [[1.1134, 0.104, Z_TROOF], [1.3266, 0.1128, Z_TROOF + 0.0016]],
                                  [[1.1134, -0.1128, Z_TROOF], [1.3266, -0.104, Z_TROOF + 0.0016]]]),
                     "box": r3([[1.1134, -0.1128, Z_TROOF], [1.3266, 0.1128, Z_TROOF + T_SW]])},
                    "turret elevator reaction -> roof -> walls / frames", layup="rib_panel",
                    notes="fix round 2 (PK2-10): the sandwich ends at y +-0.104; the outer 8.8 mm each side, bonded and "
                          "riveted on the wall tops, are the 1.6 mm solid edge band of the roof (the V roof leaves only "
                          "2 mm to the OML above the 6.8 mm sandwich at the front corners)",
                    touch=["ST-FS1110", "ST-FS1330", "M-TURRETWALL"]))
    M.append(member("M-PARAWALL", "YK250-CH-025", "parachute-bay side wall", "paraşüt bölmesi yan duvarı",
                    "side walls of the parachute compartment, container restraint brackets; fix round 1 (VPK-03): the top "
                    "flange is turned OUTBOARD (50 mm: inner 25 mm the hatch land, outer 25 mm the land of the fixed "
                    "surround skin) so that the clear opening is the 312 mm between the wall faces",
                    "cfrp_pw_mtm45_as4", "prepreg_ooa_vacbag", {"type": "flat sandwich wall (rib_panel)", "t": T_SW},
                    {"box": r3([[X_PFF + 0.0034, 0.156, -0.1848], [X_PFR - 0.0034, 0.156 + T_SW, 0.165]]), "top": "skin",
                     "lands": [{"x": r3([X_PFF + 0.0034, X_PFR - 0.0034]), "y": [0.156, 0.2128], "surface": "upper",
                                "mirror": True, "text": "outward top flange 50 mm (hatch land + surround-skin land)"}]},
                    "container inertia -> walls/floor -> FS1490 / FS1810", mirror=True, layup="rib_panel",
                    touch=["ST-FS1490", "ST-FS1810", "M-PARAFLOOR"]))
    M.append(member("M-PARAFLOOR", "YK250-CH-026", "parachute-bay floor", "paraşüt bölmesi tabanı",
                    "floor of the parachute compartment", "cfrp_pw_mtm45_as4", "prepreg_ooa_vacbag",
                    {"type": "flat sandwich (rib_panel)", "t": T_SW},
                    {"box": r3([[X_PFF + 0.0034, -0.1628, -0.1848], [X_PFR - 0.0034, 0.1628, -0.1848 + T_SW]])},
                    "container inertia -> floor -> FS1490 / FS1810 / walls", layup="rib_panel",
                    touch=["ST-FS1490", "ST-FS1810", "M-PARAWALL"]))
    M.append(member("M-MIDFLOOR", "YK250-CH-027", "mission-bay floor", "görev bölmesi tabanı",
                    "equipment floor of the mission-computer bay (FS1810 -> FS-FUEL), chine to chine; fix round 1 "
                    "(VPK-03): the centre (y +-0.125) is a framed cut-out closed by the removable equipment tray "
                    "TR-MISSION (mission computer + engine ECU), screwed from below to the cut-out edge stiffeners and "
                    "lowered out through the belly hatch P-MBHATCH; the two fixed outer strips (chine to y +-0.125) are "
                    "the lower chord of the forward body", "cfrp_pw_mtm45_as4", "prepreg_ooa_vacbag",
                    {"type": "flat sandwich (rib_panel) outer strips; two bonded longitudinal hat stiffeners (PW 4 plies, "
                             "20 x 15 mm) along the cut-out edges y +-0.125 carry the strip edges and the tray screws "
                             "(structures: floor-strip buckling as the lower chord of the forward body)", "t": T_SW},
                    {"box": r3([[X_PFR + 0.0034, -0.36, -0.1348], [X_FUELF - 0.0034, 0.36, -0.1348 + T_SW]]), "sides": "skin",
                     "cutout": {"x": r3([X_PFR + 0.0334, X_FUELF - 0.0334]), "y": [-0.125, 0.125],
                                "closed_by": "TR-MISSION (removable equipment tray)"}},
                    "mission equipment inertia -> tray / floor strips -> frames + chine longerons", layup="rib_panel",
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
                    layup="rib_panel", touch=["ST-FS-FUEL", "ST-FS-MS", "ST-FS-RS", "M-FWDDECK", "M-CTBOX",
                                              "M-WELLROOF"]))
    M.append(member("M-FWDDECK", "YK250-CH-029", "forward fuel-bay floor / payload-bay roof",
                    "ön yakıt bölmesi tabanı / yük bölmesi tavanı",
                    "deck FS-FUEL -> main-spar frame, chine to chine: floor of the forward fuel cell, roof of the "
                    "forward part of the payload bay, payload-tray rails (underside)", "cfrp_pw_mtm45_as4",
                    "prepreg_ooa_vacbag", {"type": "flat sandwich (rib_panel), fuel-side face sealed; the two payload-tray "
                                                   "rails (6061-T6 T 20 x 20 x 2.5 mm at y +-0.10) are bonded and screwed "
                                                   "(M4 at 50 mm) to the underside as deck stiffeners (structures phase)",
                                           "t": T_SW},
                    {"box": r3([[X_FUELF + 0.0034, -0.40, -0.0600], [X_MS0 - 0.0034, 0.40, -0.0600 + T_SW]]), "sides": "skin"},
                    "fuel + payload inertia -> deck -> keel beams / FS-FUEL / main-spar frame", layup="rib_panel",
                    touch=["ST-FS-FUEL", "ST-FS-MS", "M-KEEL", "M-CHINE"]))
    M.append(member("M-WELLROOF", "YK250-CH-030", "main-gear well roof deck", "ana takım kuyusu tavanı",
                    "deck rear-spar frame -> FS-GEAR, chine to chine: roof of both main wells, floor of the aft fuel "
                    "cell, up-lock hooks; tension/compression chord of the belly across the wells",
                    "cfrp_pw_mtm45_as4", "prepreg_ooa_vacbag",
                    {"type": "flat sandwich (layups.fuel_floor_wellroof: PW 3 plies / ROHACELL 71 WF 5.6 mm / 3 plies, "
                             "6.8 mm; structures phase: aft fuel-cell pressure), supported on the centre line by the "
                             "well keel web M-WELLKEEL", "t": T_SW},
                    {"box": r3([[X_RS0 + 0.0034, -0.40, -0.0668], [X_GEARF - 0.0034, 0.40, -0.0668 + T_SW]]), "sides": "skin"},
                    "gear + fuel loads -> well roof -> gear beams / well keel web / rear-spar frame / FS-GEAR",
                    layup="fuel_floor_wellroof",
                    touch=["ST-FS-RS", "ST-FS-GEAR", "M-GEARBEAM", "M-WELLKEEL", "F-TRUNNION", "M-CHINE"]))
    M.append(member("M-GEARBEAM", "YK250-CH-031", "main-gear beam (outboard well wall)", "ana takım kirişi",
                    "outboard wall of each main well (rear-spar frame -> FS-GEAR): carries the trunnion fitting; "
                    "takes the gear loads to both frames and the well roof; fix round 2 (PK2-03): inboard face at y "
                    f"{Y_GB} in the lower chine corner, bottom edge trimmed to the curved skin (the leg passes inboard of "
                    "it with >= 10 mm in every gear position); closed into a torsion box with the well roof by the "
                    "trunnion-fitting top plate and the end clips at FS-RS / FS-GEAR (structures G-BEAM-TORSION)",
                    "cfrp_pw_mtm45_as4",
                    "prepreg_ooa_vacbag", {"type": "sandwich wall (rib_panel) + UD caps, 7075 doubler at the "
                                                   "trunnion fitting", "t": T_SW},
                    {"boxes": r3([[[spar_x(Y_GB, 0.72) + 0.0034, Y_GB, z_bot(X0_T, Y_GB)],
                                   [X0_T - X_NOTCH, Y_GB + T_SW, -0.0668]],
                                  [[X0_T - X_NOTCH, Y_GB, Z_NOTCH], [X0_T + X_NOTCH, Y_GB + T_SW, -0.0668]],
                                  [[X0_T + X_NOTCH, Y_GB, z_bot(X0_T, Y_GB)], [X_GEARF - 0.0034, Y_GB + T_SW, -0.0668]]]),
                     "box": r3([[spar_x(Y_GB, 0.72) + 0.0034, Y_GB, z_bot(X0_T, Y_GB)],
                                [X_GEARF - 0.0034, Y_GB + T_SW, -0.0668]]),
                     "bottom": "skin",
                     "notch": {"x": r3([X0_T - X_NOTCH, X0_T + X_NOTCH]), "z_top": Z_NOTCH,
                               "text": "fix round 2 (PK2-03): reinforced notch in the lower edge round the leg (the "
                                       "splayed leg passes the wall at the skin): the lower UD cap runs up round the "
                                       "notch, the 7075 flange of F-TRUNNION (bolted fore and aft of it) bridges it; "
                                       "structures G-BEAM-* use the notch depth"}},
                    "trunnion -> gear beam -> rear-spar frame + FS-GEAR + well roof -> wing box / chine longerons",
                    mirror=True, layup="rib_panel",
                    touch=["ST-FS-RS", "ST-FS-GEAR", "M-WELLROOF", "F-TRUNNION", "M-CHINE"]))
    xs = np.linspace(X_GEARF + 0.0034, X_FW_FWD - 0.0010, 6)
    M.append(member("M-DORSAL", "YK250-CH-032", "dorsal longeron", "sırt uzun kirişi",
                    "upper longerons under the dorsal skin (FS-GEAR -> forward face of the firewall FS3670) at the fin "
                    "roots: tail bending (fin and stub loads) forward into FS-GEAR / the aft fuel-bay frame; fix round 1 "
                    "(VPK-06): the carbon longeron ends ahead of the firewall, spliced (3 x M5 Ti) to the forward tongue "
                    "of the firewall corner fitting F-FW-CORNER (no composite in the engine bay)", "cfrp_ud_mtm45_as4",
                    "prepreg_ooa_vacbag", {"type": "hat section 25 x 20 mm, UD cap + PW", "w": 0.025, "h": 0.020,
                                            "t": 0.0020},
                    {"paths": [[[r3(x), 0.150, r3(z_top(x, 0.150) - SKIN - 0.011)] for x in xs]]},
                    "fin root fittings -> FS3480 / firewall corner fitting -> dorsal longerons -> FS-GEAR", mirror=True,
                    layup="spar_cap_ud", touch=["ST-*", "F-FIN-*", "F-FW-CORNER"]))
    M.append(member("M-PAYWALL-AFT", "YK250-CH-034", "payload-bay aft wall", "faydalı yük bölmesi arka duvarı",
                    "aft wall of the payload bay (keel beam to keel beam); forward wall of the lateral harness duct",
                    "cfrp_pw_mtm45_as4", "prepreg_ooa_vacbag", {"type": "solid laminate wall 1.6 mm (8 plies PW)",
                                                               "t": 0.0016},
                    {"box": r3([[X_PB1 - DUCT_SHIFT, -0.2040, -0.215], [X_PB1 - DUCT_SHIFT + 0.0016, 0.2040, -0.0532]]),
                     "bottom": "skin",
                     "notes": "fix round 3 (PK3-05): 3.5 mm ahead of the payload-bay zone end so that the lateral harness "
                              "duct is >= 30 mm wide (H-MAIN 20 mm + the 10 mm corridor of layout.systems.harness.rules)"},
                    "secondary (closes the bay; payload loads go to the keel beams and the deck)",
                    layup="spar_web", thickness=0.0016, touch=["M-KEEL", "M-FWDDECK", "M-WELLROOF", "ST-FS-RS"]))
    M.append(member("M-WELLWALL-FWD", "YK250-CH-035", "main-well forward wall", "ana takım kuyusu ön duvarı",
                    "forward wall of both main wells (gear beam to gear beam, under the well roof); aft wall of the "
                    "lateral harness duct", "cfrp_pw_mtm45_as4", "prepreg_ooa_vacbag",
                    {"type": "solid laminate wall 1.6 mm (8 plies PW)", "t": 0.0016},
                    {"box": r3([[X_W0 - 0.0016, -Y_GB, -0.215], [X_W0, Y_GB, -0.0668]]), "bottom": "skin"},
                    "secondary (well closure, harness duct wall)", layup="spar_web", thickness=0.0016,
                    touch=["M-GEARBEAM", "M-WELLROOF", "M-KEEL"]))
    M.append(member("M-WELLKEEL", "YK250-CH-036", "main-well keel web", "ana takım kuyusu omurga gövdesi",
                    "two webs between the main wells (well forward wall -> FS-GEAR, from the well roof 0.12 m down), the "
                    "walls of the centre-line harness channel: halve the span of the well roof under the aft fuel cell; "
                    "the inner-door hinge brackets are bolted to their lower-edge flanges (structures phase)",
                    "cfrp_pw_mtm45_as4", "prepreg_ooa_vacbag",
                    {"type": "solid laminate web 1.0 mm (5 plies PW, 0/90 and +-45 alternating) with bonded edge "
                             "flanges, one per side of the harness channel", "t": 0.0010},
                    {"box": r3([[X_W0, Y_WK, -0.1868], [X_GEARF - 0.0034, Y_WK + 0.0010, -0.0668]]),
                     "notes": "fix round 3 (PK3-05): webs at y +-0.015..0.016 (was +-0.0105..0.0115): 30 mm clear channel "
                              "= H-MAIN 20 mm + the 10 mm harness corridor; the stowed tyre envelopes incl. their 12 mm "
                              "clearance start at |y| 0.0189, so each tyre keeps >= 14.9 mm to its web; the inner-door "
                              "hinge brackets on the lower-edge flanges reach 5 mm inboard to the hinge lines y +-0.010"},
                    "well-roof pressure / inner-door hinge loads -> keel webs -> well forward wall + FS-GEAR",
                    mirror=True, layup="spar_web", thickness=0.0010,
                    touch=["ST-FS-GEAR", "M-WELLROOF", "M-WELLWALL-FWD"]))
    M.append(member("M-AFTKEEL", "YK250-CH-033", "aft keel beam (engine bay, lower)", "arka omurga kirişi",
                    "centre-line channel under the engine (firewall -> x 3.905): ventral root fittings (3 points), "
                    "keel land of the split lower cowl halves and lip-ring support; metallic (engine bay)",
                    "al_7075_t651_plate",
                    "cnc_milling_metal", {"type": "machined channel 24 x 40 mm, t 2.5 mm (structures phase: the formed "
                                                  "2024-T3 1.6 mm channel fails the tail-bumper load) with two integral "
                                                  "lower land flanges 26 x 1.6 mm (fix round 1, VPK-05: keel land of the "
                                                  "L/R lower cowl halves, Camloc receptacles at y +-0.040)",
                                          "w": 0.024, "h": 0.040, "t": 0.0025, "land_flange_w": 0.026,
                                          "land_flange_t": 0.0016},
                    {"box": r3([[X_FW + 0.0074, -0.012, -0.093], [3.905, 0.012, -0.055]]), "bottom": "skin",
                     "lands": [{"x": r3([X_FW + 0.0074, 3.905]), "y": [-0.050, 0.050], "surface": "lower",
                                "text": "integral land flanges 26 x 1.6 mm (keel land of the split lower cowl)"}]},
                    "ventral fin / bumper strike -> aft keel beam -> firewall + ring frame", thickness=0.0025,
                    touch=["ST-FS3670", "ST-FS3738", "F-VENTRAL-*"]))
    # ---------------------------------------------------------------- dorsal spine (parachute bridle tie, S1-04)
    xs0, xs1 = X_PFR + 0.0034, X_RS0 - 0.0034
    zf0, zf1 = 0.161, 0.165                                     # channel floor (above the fuel-cell tops z 0.16)
    zw = round(z_top(2.0, 0.022) - 0.0063, 4)                  # wall top under the V-roof skin (5.8 mm normal)
    M.append(member("M-SPINE", "YK250-CH-037", "dorsal spine channel (parachute bridle tie)",
                    "sırt omurga kanalı (paraşüt kayış bağı)",
                    "fix round 1 (S1-04, VPK-04): structural U-channel on the centre line under the dorsal skin from "
                    "FS1810 to the rear-spar frame; carries both bridle fittings (F-RISER-FWD at its forward end, "
                    "F-RISER-AFT at its aft end) and closes the bridle triangle: the opposed x-components of the two "
                    "legs are a strut force in the channel, the net x-component is diffused by shear into the fixed "
                    "mission-bay upper skin P-MB-UPPER (screwed to the channel flanges, FS1810 -> FS-FUEL) and from it "
                    "into the chine longerons; the channel is a sealed trough through the vapour-tight fuel bays "
                    "(frame cut-outs C-SPINE-*); its two top flanges (y 0.020-0.080, under the skin) are the lands of "
                    "the tear-away bridle cover strip P-SPINE (y 0.020-0.045) and of the inboard edges of the fuel-bay "
                    "panels P-FUEL1-3 (y 0.055-0.080), with a bonded flush separator strip between them",
                    "cfrp_pw_mtm45_as4", "prepreg_ooa_vacbag",
                    {"type": "CFRP U-channel 44 x 24 mm, 4 plies PW (0.8 mm, [+-45/0/90]s, 50 % +-45), "
                             "flanges 60 mm each side following the V roof, local pad-ups to 16 plies (3.2 mm) under "
                             "the bridle fittings (structures P-SPINE-*)",
                     "w": 0.044, "h": 0.024, "t": 0.0008, "flange_w": 0.060, "pad_t": 0.0032},
                    {"boxes": r3([[[xs0, -0.022, zf0], [xs1, 0.022, zf1]],
                                  [[xs0, 0.020, zf1], [xs1, 0.022, zw]], [[xs0, -0.022, zf1], [xs1, -0.020, zw]]]),
                     "box": r3([[xs0, -0.022, zf0], [xs1, 0.022, zw]]), "top": "skin",
                     "lands": [{"x": r3([xs0, xs1]), "y": [0.020, 0.080], "surface": "upper", "mirror": True,
                                "text": "top flanges under the dorsal skin"}]},
                    "bridle legs -> F-RISER-FWD / F-RISER-AFT -> channel (axial) -> screws -> P-MB-UPPER (shear) -> "
                    "chine longerons; vertical components -> FS1810 / rear-spar frame",
                    layup="spar_web", thickness=0.0008,
                    touch=["ST-FS1810", "ST-FS-FUEL", "ST-FS-MS", "ST-FS-RS", "F-RISER-*"]))
    # ---------------------------------------------------------------- ventral keel strip ahead of the firewall (VPK-05)
    xv0, xv1 = 3.480 + 0.0034, X_FW_FWD - 0.0010
    vk = [[r3(x), 0.0, r3(z_bot(x) + SKIN + 0.010)] for x in np.linspace(xv0, xv1, 5)]
    M.append(member("M-VENTRALKEEL", "YK250-CH-039", "ventral keel strip (FS3480 -> firewall)",
                    "ventral omurga şeridi (FS3480 -> yangın perdesi)",
                    "fix round 1 (VPK-05): centre-line CFRP hat strip on the belly skin from FS3480 to the firewall "
                    "forward face: carries the forward root of the ventral fin (x 3.52-3.67, 2 x M5 positioning bolts "
                    "through its root rib; the three lugs F-VENTRAL-1..3 carry the loads) and is the keel land of the "
                    "split stabilator-actuator hatch P-STABACT-L/R; spliced through the firewall to the aft keel beam "
                    "M-AFTKEEL (2 x M5, metallic splice plate on the engine side)",
                    "cfrp_pw_mtm45_as4", "prepreg_ooa_vacbag",
                    {"type": "hat 24 x 20 mm, 8 plies PW (1.6 mm), land flanges 26 mm each side (y +-0.050)",
                     "w": 0.024, "h": 0.020, "t": 0.0016},
                    {"paths": [vk], "lands": [{"x": r3([xv0, xv1]), "y": [-0.050, 0.050], "surface": "lower",
                                              "text": "hat land flanges on the belly skin"}]},
                    "ventral forward root (side loads) -> keel strip -> FS3480 / firewall", layup="spar_web",
                    thickness=0.0016, touch=["ST-FS3480", "ST-FS3670"]))
    # ---------------------------------------------------------------- wing carry-through and glove ribs
    yj = YJ
    # fix round 3 (VS3-01): the glove caps follow the glove loft outboard of a defined cap ramp at the side-of-body rib
    # (the layout and the section model of ucav250.analysis.structures use the same cap geometry)
    ygl = [Y_SOB, Y_RAMP, 0.50, 0.55, 0.60, 0.65, yj]
    # main-spar points also at every cap ply-zone break between the SOB rib and the joint rib (the cap capsules of
    # layout_check C03 then carry the local cap thickness)
    zb_ = [float(v) for v in ((_wd().get("main_cap") or {}).get("zone_breaks_y_m") or [])]
    yms = sorted(set(ygl) | {round(v, 4) for v in zb_ if Y_SOB < v < yj})
    ms = [[r3(X_MS0), 0.0, 0.0]] + [[r3(spar_x(y_, 0.25)), y_, 0.0] for y_ in yms]
    rs = [[r3(X_RS0), 0.0, 0.0]] + [[r3(spar_x(y_, 0.72)), y_, r3(-0.006 * y_ / Y_SOB)] for y_ in Y_RS_BODY] + \
         [[r3(spar_x(y_, 0.72)), y_, r3(-0.006 - 0.006 * (y_ - Y_SOB) / (yj - Y_SOB))]
          for y_ in sorted(set(ygl) | set(Y_RS_GLOVE))]
    prof_m = Z.spar_depth_profile(S, af, float(P["main_spar_frac"]))["rows"]
    prof_r = Z.spar_depth_profile(S, af, float(P["rear_spar_frac"]))["rows"]
    z_caps_m, t_caps_m = zip(*[cap_centroids(prof_m, p[1], "main") for p in ms])
    z_caps_r, t_caps_r = zip(*[cap_centroids(prof_r, p[1], "rear") for p in rs])
    z_caps_m, z_caps_r = [list(v) for v in z_caps_m], [list(v) for v in z_caps_r]
    _true_depth_caps(ms, z_caps_m, t_caps_m)
    _true_depth_caps(rs, z_caps_r, t_caps_r)
    zbox0 = min([zb0] + [v[0] - 0.5 * t for v, t in zip(z_caps_m, t_caps_m)])
    zbox1 = max([zb1] + [v[1] + 0.5 * t for v, t in zip(z_caps_m, t_caps_m)])
    M.append(member("M-CTBOX", "YK250-CH-001", "centre wing box (carry-through)", "orta kanat kutusu (geçiş kutusu)",
                    "one-piece centre wing box y -0.70 .. +0.70: main and rear spars on the reference-trapezoid spar "
                    "lines (25 % / 72 % chord; chevron with a centre kink of 2 x 8.0 deg main and 2 x 4.8 deg rear), "
                    "continuous UD spar caps from fork to fork, +-45 webs, box covers inside the body, centre kink "
                    "fitting (7075-T651) at y = 0; carries the outer-panel moments, shears and torques across the "
                    "body; ROOT PART of the assembly", "cfrp_ud_mtm45_as4", "prepreg_ooa_vacbag",
                    {"type": "two-spar box; spar caps UD (spar_cap_ud, area from structures.derived: root "
                             f"{float(S['structures']['derived']['spar_cap_area_root_mm2']):.0f} mm2 per cap; ply "
                             "schedule structures.sizing.wing.main_cap.zones), webs +-45 PW (spar_web, >= 0.6 mm), "
                             "covers inside the body: sandwich layups.ct_box_cover 0.4/6/0.4 mm bonded to the caps, "
                             "centre-line rib with the kink fitting at y = 0",
                     "main_cap_width": 0.040, "rear_cap_width": 0.025, "depth_in_body": r3(zb1 - zb0)},
                    {"main_spar_line": ms, "rear_spar_line": rs, "z": r3([zb0, zb1]),
                     "z_note": "outer surfaces of the box covers inside the body (|y| < y_sob); outboard of the side-of-"
                               "body rib the caps follow the glove loft (sob_transition); near the chine aft of "
                               "mid-chord (|y| > 0.33 at the rear spar) the body upper surface drops to the glove "
                               "loft, and the rear caps and the aft part of the upper cover follow the OML under the "
                               "solid skin over the caps (rear_spar_caps_z)",
                     "y_extent": [-yj, yj],
                     "main_spar_caps_z": z_caps_m, "main_spar_caps_t": [r3(t, 5) for t in t_caps_m],
                     "rear_spar_caps_z": z_caps_r, "rear_spar_caps_t": [r3(t, 5) for t in t_caps_r],
                     "caps_basis": "fix round 3 (VS3-01): cap CENTROID z and cap thickness at each spar-line point (lower, "
                                   "upper), from the ply schedule structures.sizing.wing.main_cap.zones (rear caps: "
                                   "rear_cap.plies) x the UD ply thickness: centroid = face - skin_solid_over_caps_m - "
                                   "t/2, face = box cover surface (z) inside the body, the glove loft outboard of the "
                                   "ramp, linear in between; the section model of ucav250.analysis.structures "
                                   "(wing_section) uses the same geometry (interface check I-CAPZ)",
                     "sob_transition": {
                         "y": r3([Y_SOB, Y_RAMP]),
                         "text": "cap ramp from the box-cover level (z +-0.0391, under the saddle fuel cell / above the "
                                 "payload bay) at the side-of-body rib to the glove loft at the inner-pin station: "
                                 "each UD cap rises / drops linearly over the ramp (ply-drop free), kinks at both ends "
                                 "(SOB rib solid land inboard, padded fork prongs at the inner pin outboard); the glove "
                                 "skins stay on the loft and are bonded to the ramped caps through a tapered ROHACELL 71 "
                                 "WF filler; the glove box skins end on the SOB rib, whose web carries the offset to the "
                                 "box covers (structures W-SOB-*)"},
                     "box": r3([[X_MS0 - 0.021, -yj, zbox0], [spar_x(yj, 0.72) + 0.013, yj, zbox1]]),
                     "oml_clearance_m": float(_wd().get("skin_solid_over_caps_m", 0.001)),
                     "oml_clearance_basis": "fix round 3 (VS3-01): the cap outer face lies directly under the solid skin "
                                            "over the caps (structures.sizing.wing.skin_solid_over_caps_m; the sandwich "
                                            "core is ramped out over the caps); cap capsules = centroid +- t/2"},
                    "outer panel -> tongue + pins -> fork -> spar caps (bending couple) / webs (shear) -> box -> "
                    "spar frames + side-of-body ribs -> body; symmetric bending balanced through the box",
                    layup="spar_cap_ud", touch=["ST-FS-MS", "ST-FS-RS", "M-SOB", "M-GLOVERIB", "M-JOINTRIB",
                                                "F-FORK", "F-REARSLOT", "M-KEEL", "M-CHINE", "M-WELLROOF",
                                                "M-FWDDECK"]))
    # fix round 3 (VS3-03): centre-line rib of the CT box with the chevron kink fittings (F-KINK-UP / -LO)
    t_cov = 0.0068                                       # layups.ct_box_cover 0.4/6/0.4 mm
    xr0 = X_MS0 + 0.020                                  # aft edge of the 40 mm main cap
    xr1 = X_RS0 - 0.0125                                 # forward edge of the 25 mm rear cap
    M.append(member("M-CLRIB", "YK250-CH-055", "CT-box centre-line rib (chevron kink rib)",
                    "orta kutu orta hat kaburgası (ok kırığı kaburgası)",
                    "fix round 3 (VS3-03): rib in the plane y = 0 between the main and rear spar caps and between the "
                    "box covers; reacts the chordwise kink forces of the chevron main caps (via the 7075 kink fittings "
                    "F-KINK-UP / -LO, 5 x M6 Ti each through its solid lands) and of the box covers (bonded flanges) "
                    "as an in-plane couple carried to the spar frames FS-MS / FS-RS (bonded and riveted end clips); "
                    "laid up and bonded into the box in the CT-box jig (assembly step 1)", "cfrp_pw_mtm45_as4",
                    "prepreg_ooa_vacbag",
                    {"type": "flanged sandwich rib (rib_panel 0.4/6/0.4), 16-ply solid lands 100 x 30 mm under the two kink "
                             "fitting tabs, 20 mm flanges bonded to the box covers", "t": T_SW,
                     "land_plies": 16, "land_mm": [100, 30]},
                    {"box": r3([[xr0, -T_SW / 2, zb0 + t_cov], [xr1, T_SW / 2, zb1 - t_cov]]),
                     "fitting_lands": [{"x": r3([xr0, xr0 + 0.100]), "z": r3([zb1 - t_cov - 0.030, zb1 - t_cov]),
                                "text": "upper kink-fitting land"},
                               {"x": r3([xr0, xr0 + 0.100]), "z": r3([zb0 + t_cov, zb0 + t_cov + 0.030]),
                                "text": "lower kink-fitting land"}]},
                    "main-cap kink forces -> kink fittings -> bolts -> rib solid lands -> rib web (in-plane couple) -> "
                    "end clips -> FS-MS / FS-RS; cover kink forces -> flanges -> rib web",
                    layup="rib_panel", touch=["M-CTBOX", "ST-FS-MS", "ST-FS-RS", "F-KINK-UP", "F-KINK-LO"]))
    for mid, part, y, nm, ntr, role in (
            ("M-SOB", "YK250-CH-050", Y_SOB, "side-of-body rib", "gövde yanı kaburgası",
             "rib on the body side line between main and rear spar (and forward to the LERX nose): splices the chine "
             "longeron, closes the body-side of the glove box, kink rib of the chine/glove skins"),
            ("M-GLOVERIB", "YK250-CH-051", 0.550, "glove rib", "eldiven kaburgası",
             "intermediate glove rib (LE -> rear spar): mid support of the composite fork, LERX nose rib "
             "between the two pin bays (fix round 1, VPK-02: the three LERX nose ribs are deleted so that the pin, "
             "puller and reamer corridors ahead of the fork are free)"),
            ("M-JOINTRIB", "YK250-CH-052", yj, "joint rib (centre-section side)", "birleşim kaburgası (orta kesit)",
             "closing rib of the centre section at the outer-panel joint plane (streamwise, y = 0.70): fork mouth, "
             "rear-spar slot fitting, blind-mate wing connector CN-WING (between the spars), joint seal land")):
        x0 = float(af.wing.interpolate_section(y)["x_le"]) + 0.004
        x1 = spar_x(y, 0.72) + 0.013
        geo = {}
        if mid == "M-SOB":
            # fix round 3 (PK3-04): aft extension behind the rear spar (glove trailing-edge closing) = splice land of
            # the aft chine piece (4 x M6 Ti, M-CHINE.splices SPL-CH-AFT)
            x1 = CHINE_AFT0 + 0.090
            role = role + ("; fix round 3 (PK3-04): extended aft of the rear spar to x %.3f as the closing rib of the "
                           "glove trailing-edge bay and the splice land of the aft chine piece; 16-ply solid lands at "
                           "both chine splices (4 x M6 Ti each, M-CHINE.splices)" % x1)
            geo["fitting_lands"] = [{"x": r3([CHINE_FWD1 - 0.075, CHINE_FWD1]), "z": [-0.016, 0.016],
                             "text": "forward chine splice land (16 plies)"},
                            {"x": r3([CHINE_AFT0, x1]), "z": [-0.016, 0.016],
                             "text": "aft chine splice land (16 plies)"}]
        geo.update({"box": r3([[x0, y - T_SW / 2, -0.050], [x1, y + T_SW / 2, 0.052]]), "contour": "wing_loft"})
        M.append(member(mid, part, nm, ntr, role, "cfrp_pw_mtm45_as4", "prepreg_ooa_vacbag",
                        {"type": "flanged sandwich rib (rib_panel), solid laminate at fittings", "t": T_SW},
                        geo, "glove skins / fittings -> rib -> spars", mirror=True, layup="rib_panel",
                        touch=["M-CTBOX", "M-CHINE", "F-FORK", "F-REARSLOT", "P-GLOVE-*"]))
    return M


D_S = np.array([spar_x(1.0, 0.25) - spar_x(0.0, 0.25), 1.0, 0.0]) / np.linalg.norm(
    [spar_x(1.0, 0.25) - spar_x(0.0, 0.25), 1.0, 0.0])               # main-spar direction in plan (outboard)
N_PIN = np.array([D_S[1], -D_S[0], 0.0])                              # chordwise, perpendicular to the spar in plan
PIN_D = 0.016                                                         # main pins d 16 h8 (Ti-6Al-4V)
BUSH_OD = 0.022                                                       # bonded steel bushes 16 H8 / OD 22
PRONG_T = 0.010                                                       # fork prong thickness at the bushes
Y_TONGUE_TIP = 0.408                                                  # tongue tip (4.6 mm clear of the SOB rib face)
Y_PINS = (0.463, 0.645)                                               # >= 2.5 D_bush from the tongue tip / fork mouth
Y_RS_BODY = [0.30, 0.33, 0.345, 0.36, 0.38]                            # rear-spar points where the body top drops
Y_RS_GLOVE = [round(0.4 + 0.0125 * k, 4) for k in range(1, 24)]       # extra rear-spar points: the thin glove loft
Y_RAMP = Y_PINS[0]                                                    # end of the SOB cap ramp (fix round 3, VS3-01)
X_REARPIN_AFT = 0.0195                                                # rear pin 19.5 mm aft of the rear-spar line (fix round 2: slot plates >= 6 mm skin to the OML)
Y_REARPIN = 0.672                                                     # rear pin 28 mm inboard of the joint plane
REARPIN_D = 0.008


def pin_length() -> float:
    """Main pin length: two 10 mm prong bosses + 30.4 mm slot + 6 mm head + 5.6 mm keeper / chamfer allowance."""
    return round(2 * PRONG_T + 0.0304 + 0.006 + 0.0056, 4)


def wing_joint() -> dict:
    """Outer-panel joint at y = 0.70 (starboard; port mirrored). Fix round 1 (S1-02/S1-03/S1-05, VPK-02): composite
    spar-stub joint (sailplane practice) - the outer-panel CFRP tongue (continuation of the outer main caps) in the CFRP
    fork of the centre section (the glove main-spar box), two chordwise Ti pins in bonded steel bushes, and a rear-spar
    lug in a slot fitting with a vertical pin; no bonded or bolted spar-cap transfer to metal parts."""
    ys = Y_PINS
    prof = Z.spar_depth_profile(S, af, float(P["main_spar_frac"]))
    yy = np.array([r["y"] for r in prof["rows"]])
    mid = np.array([r["mid"] for r in prof["rows"]])
    dep = np.array([r["depth"] for r in prof["rows"]])
    d_s, n_pin = D_S, N_PIN
    L_pin = pin_length()
    pins = []
    for k, y in enumerate(ys):
        pins.append({"id": f"P-MAIN{k + 1}", "position": r3([spar_x(y, 0.25), y, float(np.interp(y, yy, mid))]),
                     "axis": r3(n_pin), "diameter": PIN_D, "length": L_pin, "head_diameter": 0.024,
                     "spec": f"headed pin, Ti-6Al-4V (ti_6al_4v_annealed_sheet allowables used as bar estimate), "
                             f"d 16 h8 x {L_pin * 1000:.0f} mm, head d 24 x 6 mm, keeper plate 2 x M4 on the fork front "
                             "boss (potted inserts)",
                     "fit": "pin 16 h8 in 16 H8 bores of bonded 4130 bushes (OD 22, fork prongs and tongue), bushes "
                            "line-reamed with the master tongue in the assembly jig",
                     "oml_depth_at_pin": r3(float(np.interp(y, yy, dep)))})
    xr = spar_x(Y_REARPIN, 0.72) + X_REARPIN_AFT
    xc_r = (xr - float(af.wing.interpolate_section(Y_REARPIN)["x_le"])) / float(
        af.wing.interpolate_section(Y_REARPIN)["chord"])
    zr = round(float(wing_pt(Y_REARPIN, xc_r, 0.5)[2]) - 0.001, 4)        # 1 mm below mid-depth (fix round 2: upper skin)
    e_tip = (ys[0] - Y_TONGUE_TIP) / d_s[1]
    e_mouth = (YJ - ys[1]) / d_s[1]
    return {
        "plane": {"y": YJ, "kind": "streamwise joint rib faces (centre-section rib M-JOINTRIB / outer-panel root "
                                   "rib), 1.5 mm elastomer seal strip on the outer-panel face"},
        "insertion": {"axis_inboard": r3(-d_s), "stroke": 0.300,
                      "text": "outer panel slides inboard along the main-spar line (plan sweep 8.0 deg; the 3 deg "
                              "dihedral kink lies in the outer-panel root bay, the tongue is straight): the tongue "
                              "enters the fork, the rear-spar lug its slot fitting and the blind-mate wing connector "
                              "its receptacle in the same stroke; then the two main pins and the rear pin are inserted "
                              "(layout.mechanisms.assembly_paths)"},
        "main_spar": {
            "kind": "composite spar-stub joint (sailplane practice): CFRP tongue of the outer panel in the CFRP fork "
                    "(glove main-spar box) of the centre section; bending reacted as a vertical couple on two "
                    f"chordwise pins {(ys[1] - ys[0]) * 1000:.0f} mm apart in y ({(ys[1] - ys[0]) / d_s[1] * 1000:.0f} "
                    "mm along the swept spar); the in-plane (drag) moment as a spanwise couple between the main pins "
                    "and the rear pin (0.27 m chordwise); no bonded or bolted spar-cap transfer to metal fittings",
            "fork": {"part": "YK250-CH-053", "owner": "chassis", "material": "cfrp_pw_mtm45_as4", "prong_t": PRONG_T,
                     "process": "prepreg_ooa_vacbag",
                     "geometry": "the glove main-spar box of the centre wing box (YK250-CH-001) from the side-of-body rib "
                                 "to the joint rib: UD caps 40 mm wide (structures.sizing.wing.main_cap) top and bottom, "
                                 "following the glove loft outboard of the SOB cap ramp (M-CTBOX.sob_transition), "
                                 f"two +-45 PW webs {_prong_plies() * 0.20066:.1f} mm (prongs, {_prong_plies()} plies, "
                                 "full depth between the cap faces) 30.4 mm apart, padded up to 10 mm (>= 40 % +-45, "
                                 "[+-45/0/90] blocks 50 mm long) around the pin bores, caps widened to 52 mm over the "
                                 "fork; four bonded 4130 bushes 16 H8 x OD 22 x 10 mm (YK250-CH-053 = bush set); mouth "
                                 "chamfer 3 x 30 deg on a bonded 1 mm GFRP wear strip (structures.sizing.wing_joint.fork)",
                     "slot": {"width": 0.0304, "height": 0.0610, "y": [round(Y_SOB + T_SW / 2, 4), YJ]},
                     "edge_distance_bush": {"fork_mouth_m": r3(e_mouth), "e_over_D": r3(e_mouth / BUSH_OD, 3)}},
            "tongue": {"part": "YK250-WG-151", "owner": "wing", "material": "cfrp_ud_mtm45_as4",
                       "process": "prepreg_ooa_vacbag",
                       "geometry": "CFRP spar stub 30 mm wide x 61 mm high from the outer-panel root rib to y "
                                   f"{Y_TONGUE_TIP} (engagement {YJ - Y_TONGUE_TIP:.3f} m): UD flanges 30 x 10 mm "
                                   "(continuation of the outer-panel main caps, built up 1:20 over 0.10 m from the root "
                                   "rib) to the outer pin, ply drops on the web side to 3 mm at the inner pin and 3 mm "
                                   "to the tip (outer faces stay on the 61 mm envelope; fix round 2), "
                                   "+-45 PW web 5 mm (25 plies), solid [+-45/0/90] boss blocks 30 mm wide x 50 mm long around "
                                   "the two pin bores with bonded 4130 bushes 16 H8 x OD 22 x 30 mm; tip closure 3 x 30 "
                                   "deg chamfer (structures.sizing.wing_joint.tongue)",
                       "edge_distance_bush": {"tongue_tip_m": r3(e_tip), "e_over_D": r3(e_tip / BUSH_OD, 3)}},
            "pins": pins,
            "access": "pins inserted from the front through the glove wing-joint access panel P-JOINTACCESS (lower "
                      "glove, side-of-body rib to joint rib), head forward, keeper plate on the fork front boss; the "
                      "LERX nose bays carry no rib between the side-of-body rib, the glove rib and the joint rib, so the "
                      "pin, puller and reamer corridors (layout.mechanisms.assembly_paths) are free"},
        "rear_spar": {
            "kind": "rear-spar lug in a slot fitting with a vertical pin: carries the chordwise force and the spanwise "
                    "force of the drag-moment couple (pin in double shear); the vertical torsion couple bears on the slot "
                    "plates (lug faces)",
            "lug": {"part": "YK250-WG-152", "owner": "wing", "material": "al_7075_t651_plate",
                    "process": "cnc_milling_metal", "thickness": 0.008, "width": 0.032, "bore": REARPIN_D,
                    "e_m": 0.016,
                    "geometry": "horizontal lug plate 8 mm, 32 mm wide, projecting 44 mm inboard of the joint plane from "
                                "the outer-panel rear-spar root fitting along the insertion axis; bore 8 H8, e 16 mm to "
                                "the lug tip (2 D); fitting bolted 4 x M5 to the outer-panel rear spar and root rib"},
            "slot_fitting": {"part": "YK250-CH-054", "owner": "chassis", "material": "al_7075_t651_plate",
                             "process": "cnc_milling_metal",
                             "geometry": "corner fitting at the joint rib / rear-spar web junction (aft face of the "
                                         "web, inboard face of the joint rib): two horizontal plates 4 mm, 8.2 mm apart "
                                         "(slot open outboard along the insertion axis), bores 8 H8; bolted 2 x M4 Ti to "
                                         "the joint rib (chordwise force in shear) and 2 x M5 Ti to the rear-spar web "
                                         "(spanwise force of the drag-moment couple in shear)",
                             "plate_t": 0.004, "slot": 0.0082},
            "pin": {"id": "P-REAR", "position": r3([xr, Y_REARPIN, zr]), "axis": [0.0, 0.0, 1.0],
                    "diameter": REARPIN_D, "length": 0.030,
                    "spec": "Ti-6Al-4V headed pin d 8 f7 x 30 mm with ball-lock retention (push-button ball-lock pin, lanyard), "
                            "inserted upward from below through P-REARACCESS"}},
        "electrical": "wing harness (aileron + flap actuators RS-485/28 V, nav/strobe light, port: HASA pitot "
                      "DroneCAN) ends in a blind-mate MIL-DTL-38999 III connector pair on the insertion axis (float-mounted "
                      "receptacle CN-WING on the joint rib between the spars, plug on the outer-panel root rib): it mates "
                      "in the last 8 mm of the insertion stroke; the centre-section branch runs in a bonded conduit inside "
                      "the centre wing box (layout.systems.harness H-WING)",
        "inspection": "joint access panel P-JOINTACCESS (lower glove, ahead of the main spar, Camloc) and P-REARACCESS "
                      "(lower glove, aft of the rear spar): visual check of the pin keepers, fork mouth, seal, rear pin "
                      "at every assembly; bushes and bores inspected (bore gauge, bush bond tap test) at the scheduled "
                      "structural inspection (assembly.maintenance_access)",
        "pre_sizing": "layout_check.fitting_presizing(): pin double shear, pin bending, bearing of the bushes in the "
                      "tongue and prong laminates with limit x 1.5 x frequent-assembly factor 1.5 (bending, shear) and "
                      "x 2.0 bearing factor (structures, baseline.yaml#design_loads.factors)"}


def bolt(bid, point, axis, d, spec, joins, group=""):
    """One fastener of a fitting bolt pattern: hole centre on the fitting face, axis, nominal diameter."""
    b = {"id": bid, "point": r3(point), "axis": r3(axis), "d": d, "spec": spec, "joins": list(joins)}
    if group:
        b["group"] = group
    return b


def _bbox(boxes):
    B = np.asarray(boxes, float)
    return r3([B[:, 0, :].min(axis=0).tolist(), B[:, 1, :].max(axis=0).tolist()])


EDGE_RULE = ("bolt patterns explicit (layout_check C04: every hole inside one envelope box with edge distance >= 2 D "
             "(metal, processes.cnc_milling_metal) / 2.5 D (composite) to the faces parallel to its axis, pitch >= 3 D "
             "within a group)")
M5, M6, M8 = "M5 12.9 (ISO 4762) + self-locking nut", "M6 12.9 (ISO 4762) + self-locking nut", \
    "M8 12.9 (ISO 4762) + NORD-LOCK washer"


def fittings() -> list:
    """Interface fittings, starboard where mirrored. Fix round 1 (VPK-01/06/07): every fitting carries its explicit bolt
    pattern ('bolts': point, axis, d, joined parts) and envelope boxes sized from it (edge >= 2 D, pitch >= 3 D); the
    stabilator node, the firewall corner, the composite fork and the rear-spar slot fitting are new."""
    F = []
    AX, AY, AZ = [1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]
    mg = MG["trunnion"]
    x0, y0, z0 = (float(v) for v in mg)
    yf = Y_GB - 0.008                                          # flange (doubler) inboard face on the gear beam
    fb = [[x0 - 0.085, yf, -0.128], [x0 - 0.036, Y_GB, -0.0668]]     # flange, fore of the forward lug
    fm = [[x0 - 0.036, yf, -0.110], [x0 + 0.036, Y_GB, -0.0668]]     # flange between the lugs (above the leg)
    fa = [[x0 + 0.036, yf, -0.128], [x0 + 0.085, Y_GB, -0.0668]]     # flange, aft of the aft lug
    # bearing lugs: lower part (round the bore) trimmed to y0 + e on its outboard side so that its lower outboard corner
    # keeps >= 10 mm from the closed trunnion door (fix round 2, PK2-03)
    lf = [[x0 - 0.062, 0.326, -0.130], [x0 - 0.036, 0.375, -0.0668]]       # forward bearing lug, upper part
    lfl = [[x0 - 0.062, 0.326, z0 - 0.023], [x0 - 0.036, round(y0 + 0.023, 4), -0.130]]   # forward lug round the bore
    la = [[x0 + 0.036, 0.326, -0.130], [x0 + 0.062, 0.375, -0.0668]]       # aft bearing lug, upper part
    lal = [[x0 + 0.036, 0.326, z0 - 0.023], [x0 + 0.062, round(y0 + 0.023, 4), -0.130]]   # aft lug round the bore
    wf = [[x0 - 0.062, 0.375, -0.130], [x0 - 0.036, yf, -0.0668]]         # lug-to-flange web (forward)
    wa = [[x0 + 0.036, 0.375, -0.130], [x0 + 0.062, yf, -0.0668]]         # lug-to-flange web (aft)
    tp = [[x0 - 0.036, 0.326, -0.0868], [x0 + 0.036, yf, -0.0668]]         # top plate (on the well roof)
    boxes = [fb, fm, fa, lf, lfl, la, lal, wf, wa, tp]
    BEAM = ("M6 12.9 (ISO 4762) from the well side (head and washer in the well), into a bonded through-thickness "
            "threaded insert M6 in the solid land of the gear beam (flush with its outboard face, installed in the beam "
            "subassembly; no nut on the outboard side)")
    ROOF = ("M6 12.9 (ISO 4762) from below (well side) into a sealed dome nutplate on the fuel side of the well roof "
            "(fuel-tight, installed and leak-tested in the chassis subassembly)")
    bl = [bolt(f"B{k + 1}", [x0 + dx, Y_GB, z], AY, 0.006, BEAM, ["M-GEARBEAM"], "gear beam")
          for k, (dx, z) in enumerate(((-0.070, -0.085), (-0.070, -0.115), (0.0, -0.088), (0.070, -0.085),
                                       (0.070, -0.115)))]
    bl += [bolt(f"B{6 + k}", [x0 + dx, y, -0.0668], AZ, 0.006, ROOF, ["M-WELLROOF"], "well roof")
           for k, (dx, y) in enumerate((dx, y) for dx in (-0.049, 0.049) for y in (0.340, 0.362))]
    F.append({"id": "F-TRUNNION", "part": "YK250-CH-070", "name": "main-gear trunnion fitting",
              "name_tr": "ana takım mafsal bağlantısı", "material": "al_7075_t651_plate", "process": "cnc_milling_metal",
              "mirror": True, "axis": [1.0, 0.0, 0.0], "pivot": r3(mg),
              "bearings": [{"x": r3(x0 - 0.049), "bore": 0.020, "fit": "20 H7 (sleeve bushing; match to the "
                            "selected gear unit)"}, {"x": r3(x0 + 0.049), "bore": 0.020, "fit": "20 H7"}],
              "boxes": r3(boxes), "box": _bbox(boxes), "bolts": bl,
              "lug": {"bore": 0.020, "e_m": r3(min(y0 - 0.326, z0 - lfl[0][2], lfl[1][1] - y0)), "w_m": 0.049, "t_m": 0.012,
                      "rule": "lug exception (fix round 1, S1-09): lug end distance below 2 D is a lug geometry, "
                              "checked with the Bruhn lug charts at the actual e/D (structures G-TRUN-*), not with the "
                              "fastener edge rule"},
              "pivot_scheme": {"kind": "flanged stub axles inserted from inside the leg yoke",
                               "text": "fix round 2 (PK2-04): no through-pin (a 118 mm pin cannot be inserted between "
                                       "the well forward wall and FS-GEAR): the gear-unit yoke carries two flanged 4130 "
                                       "stub axles d 20 x 30 mm, inserted from the inside of the yoke outward through "
                                       "the yoke arms into the two 20 H7 bushings of the fitting lugs and bolted to the "
                                       "yoke arms (3 x M5 each, gear-unit item); reached from the well with the gear "
                                       "down (inner door in maintenance mode); path "
                                       "layout.mechanisms.assembly_paths main_stub_axles_R",
                               "stub_axle": {"d": 0.020, "length": 0.030, "material": "steel_4130_n",
                                             "arm_m": 0.013}},
              "attach": "machined 7075-T651 fitting: flange (doubler) 8 mm on the inboard face of the gear beam, bolted "
                        "5 x M6 12.9 (axis y; 2 fore, 1 between and 2 aft of the lugs, so that every head is reached "
                        "from the well) into bonded through-thickness inserts of the beam land "
                        "(fix round 2, PK2-05: no nut between the beam and the curved skin); two bearing lugs 26 mm "
                        "(x) with 20 H7 bushings 98 mm apart; top plate 20 mm on the well roof, 4 x M6 (axis z, in the "
                        "lugs) from below into sealed dome nutplates on the fuel side; the flange between the lugs ends "
                        "at z -0.110, the outer flanges at z -0.128 and the lugs at y 0.375 below z -0.130 so that the leg (r 22.5 mm) keeps >= 10 mm "
                        "to the fitting in every gear position (layout_check C05); the flange bridges the notch in the "
                        "lower edge of the gear beam (reinforcement, structures G-BEAM-NOTCH)",
              "actuator_anchor": {"kind": "rotary EMA of the gear unit on the trunnion axis, flange on the fitting "
                                          "front face (layout.systems.actuators ACT-MLG-EMA)",
                                  "point": r3([x0 - 0.085, y0, z0])},
              "locks": "down-lock: 4130 lock bolt d 10 in double shear in a tight clevis of the fitting (ear gaps <= "
                       "0.5 mm) engaging the leg yoke (requirement for the gear-unit supplier; structures G-DOWNLOCK "
                       "shear and G-DOWNLOCK-BEND pin bending, fix round 2 VS2-07); up-lock: hook "
                       "fitting F-UPLOCK under the well roof deck engaging a roller on the leg",
              "inspection": "beam bolts and roof bolts from the well (gear down); the dome nutplates on the fuel side "
                            "are reached only with the aft cell removed (assembly.maintenance_access)",
              "touch": ["M-GEARBEAM", "M-WELLROOF", "M-CHINE"]})
    xu, yu = x0, 0.250
    ub = [[xu - 0.020, yu - 0.020, -0.090], [xu + 0.020, yu + 0.020, -0.0668]]
    F.append({"id": "F-UPLOCK", "part": "YK250-CH-072", "name": "main-gear up-lock hook fitting",
              "name_tr": "ana takım yukarı kilit kancası bağlantısı", "material": "al_7075_t651_plate",
              "process": "cnc_milling_metal", "mirror": True, "box": r3(ub),
              "point": r3([xu, yu, -0.090]),
              "bolts": [bolt(f"B{k + 1}", [xu + dx, yu + dy, -0.0668], AZ, 0.005, M5, ["M-WELLROOF"], "well roof")
                        for k, (dx, dy) in enumerate((dx, dy) for dx in (-0.010, 0.010) for dy in (-0.010, 0.010))],
              "attach": "machined bracket bolted 4 x M5 (axis z, 20 mm square) to the underside of the well roof deck "
                        "(blind potted inserts from the well side: the fuel-side facesheet of the roof is not pierced; "
                        "fix round 2, PK2-05); spring-loaded hook (gear unit item, solenoid unlock) engages the up-lock "
                        "roller on the stowed leg; structures F-UPLOCK-* (stowed leg at the manoeuvre load factor)",
              "touch": ["M-WELLROOF"]})
    nv = NG["pivot"]
    xn, zn = float(nv[0]), float(nv[2])
    # fix round 2 (PK2-10): the lower forward corner of each block is cut back (the V belly rises forward): aft part
    # full height, forward part from zn - 0.008 up, so that the block keeps the 5.8 mm skin to the OML (true distance)
    b_aft = [[xn - 0.004, 0.0355, zn - 0.020], [xn + 0.032, 0.0415, zn + 0.026]]
    b_fwd = [[xn - 0.032, 0.0355, zn - 0.008], [xn - 0.004, 0.0415, zn + 0.026]]
    blocks = [b_aft, b_fwd] + [[[b_[0][0], -b_[1][1], b_[0][2]], [b_[1][0], -b_[0][1], b_[1][2]]] for b_ in (b_aft, b_fwd)]
    bpts = ((-0.018, 0.009), (0.020, -0.008), (0.020, 0.014))         # (dx, dz) about the pivot, per side
    F.append({"id": "F-NG-PIVOT", "part": "YK250-CH-071", "name": "nose-gear pivot bushing blocks",
              "name_tr": "burun takımı mafsal burç blokları", "material": "al_7075_t651_plate",
              "process": "cnc_milling_metal", "axis": [0.0, 1.0, 0.0], "pivot": r3(nv), "bore": 0.016,
              "boxes": r3(blocks), "box": _bbox(blocks),
              "bolts": [bolt(f"B{k + 1}", [xn + dx, sy * 0.0385, zn + dz], AY, 0.006, M6 + ", through the keel wall",
                             ["M-KEELWALL"], "keel wall " + ("R" if sy > 0 else "L"))
                        for k, (sy, dx, dz) in enumerate((sy, dx, dz) for sy in (1, -1) for dx, dz in bpts)],
              "attach": "two 7075 doubler blocks 64 x 46 x 6 mm on the INBOARD faces of the keel walls (inside the keel "
                        "slot: the V belly leaves no room below the outboard faces, fix round 1 VPK-07), each with a "
                        "bonded flanged bushing 16 H7 (OD 22) and 3 x M6 through the wall (fix round 2: forward lower "
                        "corner cut back to the V belly, so one bolt forward at z +9 mm and two aft at z -8 / +14 mm "
                        "about the pivot)",
              "pivot_scheme": {"kind": "flanged stub axles inserted from inside the leg yoke",
                               "text": "fix round 2 (PK2-04): no through-pin (an 83 mm pin cannot pass the keel walls: "
                                       "the skin is 15-25 mm outboard of them): the nose-leg yoke carries two flanged "
                                       "4130 stub axles d 16 x 22 mm inserted from the inside of the yoke outward "
                                       "through the yoke arms into the bushings of the blocks and bolted to the arms "
                                       "(3 x M4 each, gear-unit item); reached through the keel slot with the gear down "
                                       "and the nose doors open; path layout.mechanisms.assembly_paths nose_stub_axles",
                               "stub_axle": {"d": 0.016, "length": 0.022, "material": "steel_4130_n",
                                             "arm_m": 0.009}},
              "actuator_anchor": {"kind": "rotary EMA of the nose-gear unit above the pivot between the keel walls, "
                                          "crank + drag link to the leg yoke (layout.systems.actuators ACT-NLG-EMA)",
                                  "point": r3([xn + 0.005, 0.0, -0.055])},
              "locks": "down-lock: over-centre drag link (gear unit, its pins in double shear); up-lock: hook under the "
                       "avionics deck engaging the axle roller",
              "touch": ["M-KEELWALL"]})
    # ------------------------------------------------------------------ stabilator node (VPK-01): firewall-mounted
    pv = STAB["pivot"]
    xs_, zs_ = float(pv[0]), float(pv[2])
    rs = float(STAB["params"]["spindle_housing_radius"])
    xa = X_FW + 0.008                                         # base flange 8 mm on the firewall aft face
    y_ci, y_co = (0.132, 0.160), (0.2125, 0.2195)              # inboard (bearing) / outboard (stub root) cheeks
    hb = [[X_FW, y_ci[0], 0.168], [xa, 0.165, 0.262]]          # base flange: inboard column
    ob = [[X_FW, 0.195, 0.168], [xa, 0.236, 0.240]]            # base flange: outboard column
    bb = [[X_FW, 0.165, 0.168], [xa, 0.195, 0.200]]            # base flange: bottom bar (pushrod window above)
    ci = [[xa, y_ci[0], zs_ - rs], [xs_, y_ci[1], zs_ + rs]]   # inboard cheek (arm to the bearing boss)
    co = [[xa, y_co[0], 0.176], [3.790, y_co[1], 0.262]]       # outboard cheek (stub rear-spar root plate)
    nbolts = [bolt(f"B{k + 1}", [X_FW, y, z], AX, 0.005, M5 + ", through the firewall stack (stainless spacer tubes) "
                   "and a 7075 backing plate on the forward face", ["ST-FS3670"], "firewall")
              for k, (y, z) in enumerate(((0.1485, 0.180), (0.1485, 0.250), (0.180, 0.184), (0.207, 0.182),
                                          (0.224, 0.182), (0.207, 0.226), (0.224, 0.226)))]
    nbolts += [bolt(f"B{8 + k}", [3.700 + dx, y_co[1], z], AY, 0.006, M6 + " (tension + shear)",
                    ["stub rear-spar root fitting (tail module)"], "stub root")
               for k, (dx, z) in enumerate((dx, z) for dx in (0.0, 0.076) for z in (0.188, 0.250))]
    F.append({"id": "F-SPINDLE-NODE", "part": "YK250-CH-095", "name": "stabilator node fitting (firewall-mounted)",
              "name_tr": "stabilatör düğüm bağlantısı (yangın perdesine bağlı)", "material": "al_7075_t651_plate",
              "process": "cnc_milling_metal", "mirror": True, "frame": "FS3670",
              "boxes": r3([hb, ob, bb, ci, co]), "box": _bbox([hb, ob, bb, ci, co]),
              "cylinder": {"center": r3([xs_, 0.145, zs_]), "axis": [0.0, 1.0, 0.0], "radius": rs,
                           "half_length": 0.015},
              "axis": [0.0, 1.0, 0.0],
              "bearing": "61805-ZZ thin-section deep-groove ball bearing 25 x 37 x 7 mm (steel shields, high-temperature grease; fix round 2, PK2-09: no elastomer seal in the cylinder hot zone) in the inboard boss (r "
                         f"{rs * 1000:.1f} mm, tail.surfaces.stabilator.params.spindle_housing_radius), retained by an "
                         "internal circlip and a 4 x M3 cover ring",
              "bolts": nbolts,
              "horn_space": {"y": [y_ci[1], y_co[0]], "text": "the spindle horn (y 0.180) swings between the inboard "
                             "and the outboard cheek (layout_check C04: horn sweep vs node, pushrod window)"},
              "spindle": {"part": "YK250-TL-301", "owner": "tail", "diameter": 0.025,
                          "span_y": [y_ci[0] - 0.002, round(float(STAB["params"]["y_root"]) + 0.10, 4)],
                          "inboard_bearing": f"61805-ZZ in the node boss (y {y_ci[0]}-{y_ci[1]})",
                          "outboard_bearing": "61805-ZZ in the stub tip rib (tail module, y 0.31-0.33)",
                          "outboard_bearing_y": 0.32,
                          "horn_y": 0.180,
                          "stabilator_interface": "spindle end engages 0.10 m into the stabilator root socket; "
                                                  "torque by a 25 mm involute spline, retained by one cross-bolt M6 "
                                                  "(access cover on the stabilator root rib)"},
              "spindle_hole_outboard_cheek": {"center": r3([xs_, y_co[0], zs_]), "diameter": 0.032,
                                              "text": "clearance hole for the 25 mm spindle (no bearing)"},
              "chine_splice": "M-CHINE ends at the firewall forward face in a 7075 end fitting bolted through the firewall "
                              "to the outboard column of the base flange (bolts B4-B7): the chine longeron axial load "
                              "enters the node on the firewall plane, no composite aft of the firewall",
              "attach": "machined 7075-T651 node per side, cantilevered 92 mm aft from the firewall aft face: base flange "
                        "8 mm (U-shaped around the stabilator pushrod window C-FW-PUSHROD) through-bolted with 7 x M5 "
                        "12.9 to the firewall stack (7075 backing plate forward); inboard cheek 28 mm (y 0.132-0.160, "
                        "clear of the engine-mount truss tubes) with the bearing boss; outboard cheek 7 mm (y "
                        "0.2125-0.2195; fix round 2: 10 -> 7 mm, structures T-NODE-CHEEK) carrying the stub rear-spar root fitting on 4 x "
                        "M6 12.9 (axis y, 76 x 62 mm rectangle, tension + shear) clear of the horn sweep; FS3738 is a "
                        "lower U-ring below the node (ring_z_max) and is riveted to the outboard cheek foot",
              "load_path": "stabilator -> spindle -> inboard bearing (boss) / stub tip bearing -> stub -> stub root fitting "
                           "-> outboard cheek (4 x M6) -> node -> 7 x M5 through the firewall -> firewall sandwich "
                           "(in-plane) + chine longeron splice -> body; structures T-NODE-*",
              "touch": ["ST-FS3670", "ST-FS3738", "M-CHINE"]})
    # ------------------------------------------------------------------ firewall corner fitting (VPK-06)
    cb = [[X_FW_FWD - 0.005, 0.098, 0.262], [X_FW, 0.156, 0.310]]      # forward backing plate (foot)
    ca = [[X_FW, 0.098, 0.262], [X_FW + 0.010, 0.156, 0.310]]         # aft base plate: engine foot pad
    ca2 = [[X_FW, 0.140, 0.258], [X_FW + 0.010, 0.172, 0.304]]        # aft base plate: clevis ear 1
    ce = [[X_FW + 0.018, 0.140, 0.258], [X_FW + 0.024, 0.172, 0.304]]  # aft clevis ear 2 (fin lug between)
    tb_ = [[X_FW_FWD - 0.035, 0.146, 0.290], [X_FW_FWD, 0.154, 0.310]]  # forward tab inside the dorsal-longeron hat
    fin_r = fin_spar_root(0.56)
    F.append({"id": "F-FW-CORNER", "part": "YK250-CH-086", "name": "firewall upper corner fitting (engine foot, fin "
              "rear spar, dorsal splice)", "name_tr": "yangın perdesi üst köşe bağlantısı (motor ayağı, dikey arka "
              "kirişi, sırt eki)", "material": "al_7075_t651_plate", "process": "cnc_milling_metal", "mirror": True,
              "frame": "FS3670", "boxes": r3([cb, ca, ca2, ce, tb_]), "box": _bbox([cb, ca, ca2, ce, tb_]),
              "engine_foot": r3([X_FW, 0.130, 0.290]), "fin_root_point": fin_r, "fin_chord_fraction": 0.56,
              "point": r3([X_FW, 0.160, 0.286]),
              "bolts": [bolt("B1", [X_FW, 0.114, 0.290], AX, 0.008, M8 + ", engine-mount foot through the stack",
                             ["ST-FS3670", "ENGINE-MOUNT foot"], "engine foot"),
                        bolt("B2", [X_FW, 0.138, 0.290], AX, 0.008, M8 + ", engine-mount foot through the stack",
                             ["ST-FS3670", "ENGINE-MOUNT foot"], "engine foot"),
                        bolt("B3", [X_FW + 0.021, 0.160, 0.290], AX, 0.006, M6 + ", fin rear-spar root lug, double "
                             "shear", ["fin rear-spar root lug (tail module)"], "fin clevis"),
                        bolt("B4", [X_FW + 0.021, 0.152, 0.272], AX, 0.006, M6 + ", fin rear-spar root lug, double "
                             "shear", ["fin rear-spar root lug (tail module)"], "fin clevis"),
                        bolt("B5", [X_FW_FWD - 0.0025, 0.110, 0.276], AX, 0.005, M5 + ", backing plate to the "
                             "firewall", ["ST-FS3670"], "splice"),
                        bolt("B6", [X_FW_FWD - 0.0025, 0.144, 0.276], AX, 0.005, M5 + ", backing plate to the "
                             "firewall", ["ST-FS3670"], "splice"),
                        bolt("B7", [X_FW_FWD - 0.025, 0.150, 0.300], AY, 0.005, M5 + " Ti, dorsal longeron end splice "
                             "(through the hat walls and the tab)", ["M-DORSAL"], "dorsal splice"),
                        bolt("B8", [X_FW_FWD - 0.010, 0.150, 0.300], AY, 0.005, M5 + " Ti, dorsal longeron end splice",
                             ["M-DORSAL"], "dorsal splice")],
              "attach": "one machined 7075-T651 corner fitting per side replaces the separate upper engine-mount foot "
                        "fitting, fin rear-spar fitting and dorsal-longeron end (fix round 1, VPK-06): aft base plate "
                        "10 mm (engine-mount upper foot, 2 x M8 12.9 through the firewall stack, pitch 24 mm) with an "
                        "aft clevis ear 6 mm (fin rear-spar root lug 8 mm between ear and base plate, 2 x M6 12.9 double "
                        "shear along x); forward splice plate 5 mm on the firewall forward face (2 x M5 to the firewall, "
                        "dorsal-longeron end fitting on M5 Ti): the carbon longeron ends ahead of the firewall",
              "touch": ["ST-FS3670", "M-DORSAL", "F-SPINDLE-NODE"]})
    eb = [[X_FW_FWD - 0.005, 0.082, 0.104], [X_FW + 0.010, 0.138, 0.136]]
    F.append({"id": "F-EMOUNT-LO", "part": "YK250-CH-087", "name": "engine-mount firewall attach fitting (lower)",
              "name_tr": "motor bağlantı parçası (yangın perdesi, alt)", "material": "al_7075_t651_plate",
              "process": "cnc_milling_metal", "mirror": True, "frame": "FS3670",
              "point": r3([X_FW, 0.110, 0.120]), "engine_foot": r3([X_FW, 0.110, 0.120]), "box": r3(eb),
              "bolts": [bolt(f"B{k + 1}", [X_FW, y, 0.120], AX, 0.008, M8 + ", through the firewall stack",
                             ["ST-FS3670", "ENGINE-MOUNT foot"], "engine foot") for k, y in enumerate((0.098, 0.122))],
              "attach": "mount foot pad 56 x 32 mm bolted through the firewall stack with 2 x M8 12.9 (pitch 24 mm, edge "
                        "16 mm), 7075 backing plate 5 mm on the forward face, stainless spacer tubes through the "
                        "sandwich and air gap",
              "touch": ["ST-FS3670"]})
    # ------------------------------------------------------------------ fin front-spar and stub front-spar fittings
    lo_, hi_ = 0.02, 0.98
    for _ in range(40):
        mid_ = 0.5 * (lo_ + hi_)
        lo_, hi_ = (mid_, hi_) if fin_spar_root(mid_)[0] < 3.480 else (lo_, mid_)
    frac = 0.5 * (lo_ + hi_)
    p = fin_spar_root(frac)
    xf = 3.480 + 0.0034
    fb_ = [[xf, p[1] - 0.021, p[2] - 0.064], [xf + 0.020, p[1] + 0.021, p[2] - 0.004]]
    F.append({"id": "F-FIN-FRONT", "part": "YK250-CH-096", "name": "fin front-spar root fitting",
              "name_tr": "dikey ön kiriş bağlantısı", "material": "al_7075_t651_plate", "process": "cnc_milling_metal",
              "mirror": True, "frame": "FS3480", "fin_chord_fraction": r3(frac, 3), "point": p, "box": r3(fb_),
              "bolts": [bolt("B1", [xf, p[1], p[2] - 0.016], AX, 0.006, M6 + ", fin front-spar root lug, double shear",
                             ["fin front-spar root lug (tail module)"], "fin clevis"),
                        bolt("B2", [xf, p[1], p[2] - 0.034], AX, 0.006, M6 + ", fin front-spar root lug, double shear",
                             ["fin front-spar root lug (tail module)"], "fin clevis"),
                        bolt("B3", [xf, p[1] - 0.009, p[2] - 0.052], AX, 0.006, M6 + ", to the frame land",
                             ["ST-FS3480"], "frame"),
                        bolt("B4", [xf, p[1] + 0.009, p[2] - 0.052], AX, 0.006, M6 + ", to the frame land",
                             ["ST-FS3480"], "frame")],
              "attach": "clevis fitting 42 x 60 mm on the aft face of FS3480: fin front-spar root lug in the clevis on 2 "
                        "x M6 12.9 (double shear along x), 2 x M6 to the frame land (and the dorsal longeron flange)",
              "touch": ["ST-FS3480", "M-DORSAL"]})
    zst = float(STUB["params"]["z"])
    sb_ = [[xf, 0.205, zst - 0.033], [xf + 0.016, 0.245, zst + 0.020]]
    F.append({"id": "F-STUB-FRONT", "part": "YK250-CH-098", "name": "stabilator-stub front-spar fitting",
              "name_tr": "sabit kök parçası ön bağlantısı", "material": "al_7075_t651_plate",
              "process": "cnc_milling_metal", "mirror": True, "frame": "FS3480",
              "stub_chord_fraction": r3((3.480 - float(STUB["params"]["x_le"])) / float(STUB["params"]["chord"]), 3),
              "point": r3([3.480, 0.2475, zst]), "box": r3(sb_),
              "bolts": [bolt("B1", [xf, 0.215, zst - 0.023], AX, 0.005, M5 + ", to the frame land", ["ST-FS3480"],
                             "frame"),
                        bolt("B2", [xf, 0.215, zst + 0.010], AX, 0.005, M5 + ", to the frame land", ["ST-FS3480"],
                             "frame"),
                        bolt("B3", [xf, 0.233, zst - 0.021], AX, 0.006, M6 + ", stub front-spar root lug, double shear",
                             ["stub front-spar root lug (tail module)"], "stub clevis"),
                        bolt("B4", [xf, 0.233, zst + 0.008], AX, 0.006, M6 + ", stub front-spar root lug, double shear",
                             ["stub front-spar root lug (tail module)"], "stub clevis")],
              "attach": "clevis fitting 40 x 53 mm on the aft face of FS3480 at the body side: stub front-spar root lug "
                        "on 2 x M6 12.9 (double shear along x), 2 x M5 to the frame land; the stub rear spar is carried "
                        "by the stabilator node F-SPINDLE-NODE",
              "touch": ["ST-FS3480"]})
    zv = float(VEN["params"]["z_root"])
    for k, (x_v, frame) in enumerate(((X_FW + 0.016, "FS3670"), (X_RING + 0.012, "FS3738"), (3.888, "M-AFTKEEL"))):
        vb_ = [[x_v - 0.016, -0.012, zv - 0.012], [x_v + 0.016, 0.012, -0.055]]
        F.append({"id": f"F-VENTRAL-{k + 1}", "part": f"YK250-CH-{99 + k:03d}", "name": f"ventral fin root fitting "
                  f"{k + 1}", "name_tr": f"ventral kök bağlantısı {k + 1}", "material": "al_7075_t651_plate",
                  "process": "cnc_milling_metal", "frame": frame, "point": r3([x_v, 0.0, zv]), "box": r3(vb_),
                  "bolts": [bolt("B1", [x_v, 0.012, zv], AY, 0.006, M6 + ", ventral root lug, double shear",
                                 ["ventral root lug (tail module)"], "ventral lug"),
                            bolt("B2", [x_v, 0.0, -0.055], AZ, 0.006, M6 + ", to the aft keel channel",
                                 ["M-AFTKEEL"], "keel")],
                  "lug": {"bore": 0.006, "e_m": 0.012, "rule": "clevis ears e = 2 D"},
                  "attach": "clevis fitting 32 x 24 mm on the aft keel beam (1 x M6 axis z): ventral root lug 1 x M6 "
                            "12.9 (double shear, axis y)",
                  "touch": ["M-AFTKEEL"]})
    # ------------------------------------------------------------------ wing joint fittings (S1-02/S1-03, VPK-02)
    prof = Z.spar_depth_profile(S, af, float(P["main_spar_frac"]))["rows"]
    yy = np.array([r["y"] for r in prof])
    dep = np.array([r["depth"] for r in prof])
    mid = np.array([r["mid"] for r in prof])
    wj = wing_joint()
    obbs, corners = [], []
    hn = 0.5 * 0.0304 + PRONG_T
    axes = np.column_stack([D_S, N_PIN, [0.0, 0.0, 1.0]])
    for ya, yb in ((Y_SOB + T_SW / 2, 0.520), (0.520, 0.600), (0.600, YJ - 0.004)):  # segments, own centre height
        a_ = np.array([spar_x(ya, 0.25), ya, 0.0])
        b_ = np.array([spar_x(yb, 0.25), yb, 0.0])
        ys9 = np.linspace(ya, yb, 9)
        for side in (-1.0, 1.0):                          # front / rear half: the caps follow the loft chordwise
            zs_u, zs_l = [], []
            for y_ in ys9:
                sec = af.wing.interpolate_section(y_)
                for dx in (0.0, side * hn):
                    xc = (spar_x(y_, 0.25) + dx / D_S[1] - float(sec["x_le"])) / float(sec["chord"])
                    zs_u.append(float(wing_pt(y_, xc, 1.0)[2]))
                    zs_l.append(float(wing_pt(y_, xc, 0.0)[2]))
            zu, zl = min(zs_u) - 0.0025, max(zs_l) + 0.0025
            cen = 0.5 * (a_ + b_) + side * 0.5 * hn * N_PIN + np.array([0.0, 0.0, 0.5 * (zu + zl)])
            half = [0.5 * float(np.linalg.norm(b_ - a_)), 0.5 * hn, 0.5 * (zu - zl)]
            obbs.append({"center": r3(cen), "axes": r3(axes.T.tolist()), "half": r3(half)})
            corners += [cen + sa * half[0] * axes[:, 0] + sb * half[1] * axes[:, 1] + sc * half[2] * axes[:, 2]
                        for sa in (-1, 1) for sb in (-1, 1) for sc in (-1, 1)]
    # fix round 3 (VS3-03): chevron kink fittings of the main caps at y = 0 (structures.sizing.wing.ct_box.kink_fitting)
    kf = (_wd().get("ct_box") or {}).get("kink_fitting") or {"plate_t_m": 0.002, "w_m": 0.040, "length_m": 0.10,
                                                              "tab_h_m": 0.030, "tab_t_m": 0.003, "bolts": 5,
                                                              "bolt_d_m": 0.006}
    zb0, zb1 = Z_BOX
    t_s = float(_wd().get("skin_solid_over_caps_m", 0.001))
    zc_lo, zc_up = (float(v) for v in cap_centroids([], 0.0, "main")[0])
    t_c = cap_centroids([], 0.0, "main")[1]
    xr0 = X_MS0 + 0.020
    tp, w_k, L_k = float(kf["plate_t_m"]), float(kf["w_m"]), float(kf["length_m"])
    h_t, t_t = float(kf["tab_h_m"]), float(kf["tab_t_m"])
    nb_k, d_k = int(kf["bolts"]), float(kf["bolt_d_m"])
    for fid, part, sg, nm, ntr in (("F-KINK-UP", "YK250-CH-056", 1.0, "CT-box kink fitting, upper main cap",
                                    "orta kutu kırık bağlantısı, üst ana başlık"),
                                   ("F-KINK-LO", "YK250-CH-057", -1.0, "CT-box kink fitting, lower main cap",
                                    "orta kutu kırık bağlantısı, alt ana başlık")):
        z_in = (zc_up - 0.5 * t_c) if sg > 0 else (zc_lo + 0.5 * t_c)       # inner face of the cap
        zp = sorted([z_in, z_in - sg * tp])
        zt_ = sorted([z_in, z_in - sg * h_t])
        ys = 1.0 if sg > 0 else -1.0                                       # tabs on opposite rib faces
        yt = sorted([ys * T_SW / 2, ys * (T_SW / 2 + t_t)])
        plate = [[X_MS0 - 0.5 * w_k, -0.5 * L_k, zp[0]], [X_MS0 + 0.5 * w_k, 0.5 * L_k, zp[1]]]
        tab = [[xr0, yt[0], zt_[0]], [xr0 + L_k, yt[1], zt_[1]]]
        pitch = 3.0 * d_k
        x_b0 = xr0 + 0.5 * (L_k - (nb_k - 1) * pitch)
        bl = [bolt(f"B{k + 1}", [x_b0 + k * pitch, ys * T_SW / 2, z_in - sg * 0.5 * h_t], AY, d_k,
                   "M6 Ti-6Al-4V (NAS1956 type) + self-locking nut, through the tab and the 16-ply solid land of the "
                   "centre-line rib", ["M-CLRIB"], "rib land") for k in range(nb_k)]
        F.append({"id": fid, "part": part, "name": nm, "name_tr": ntr, "material": "al_7075_t651_plate",
                  "process": "cnc_milling_metal", "boxes": r3([plate, tab]), "box": _bbox([plate, tab]), "bolts": bl,
                  "attach": f"fix round 3 (VS3-03): machined 7075-T651 L-fitting: plate {w_k * 1000:.0f} x "
                            f"{L_k * 1000:.0f} x {tp * 1000:.0f} mm bonded (EA 9394) to the box-side face of the main cap "
                            f"over the chevron kink (no fastener through the cap), tab {L_k * 1000:.0f} x "
                            f"{h_t * 1000:.0f} x {t_t * 1000:.0f} mm on the {'starboard' if sg > 0 else 'port'} face of "
                            f"the centre-line rib, {nb_k} x M6 Ti at {pitch * 1000:.0f} mm pitch (structures CT-KINK-*)",
                  "touch": ["M-CTBOX", "M-CLRIB"]})
    F.append({"id": "F-FORK", "part": "YK250-CH-053", "name": "outer-panel joint fork (glove main-spar box, CFRP)",
              "name_tr": "dış panel birleşim çatalı (eldiven ana kiriş kutusu, CFRP)", "material": "cfrp_pw_mtm45_as4",
              "process": "prepreg_ooa_vacbag", "mirror": True,
              "obbs": obbs,
              "box": r3([np.min(corners, axis=0).tolist(), np.max(corners, axis=0).tolist()]),
              "bolts": [bolt(pn["id"], pn["position"], pn["axis"], PIN_D, "pin d 16 in a bonded 4130 bush OD 22 "
                             "(edge rule on the pin diameter; vertical edges closed by the UD caps)",
                             ["tongue YK250-WG-151"], "main pins")
                        for pn in wj["main_spar"]["pins"]],
              "attach": "layout.chassis.wing_joint.main_spar.fork (the glove main-spar box of the centre wing box: UD caps, "
                        "two +-45 prongs padded to 10 mm at the bushes; no metal fitting)",
              "oml_clearance_m": 0.002,
              "oml_clearance_basis": "UD caps under the 1.0 mm solid skin over the caps (structures.sizing.wing."
                                     "skin_solid_over_caps_m) + 1 mm",
              "touch": ["M-CTBOX", "M-SOB", "M-GLOVERIB", "M-JOINTRIB"]})
    rp = wj["rear_spar"]["pin"]
    xr_, yr_, zr_ = rp["position"]
    xw = spar_x(yr_, 0.72) + 0.0034
    rb_s = [[xw, yr_ - 0.016, zr_ - 0.0081], [xr_ + 0.016, YJ - T_SW / 2, zr_ + 0.0081]]   # slot block (2 plates)
    rb_r = [[xw, YJ - T_SW / 2 - 0.004, zr_ - 0.008], [xw + 0.028, YJ - T_SW / 2, zr_ + 0.008]]  # rib flange
    rb_w = [[xw, 0.630, zr_ - 0.010], [xw + 0.005, YJ - T_SW / 2, zr_ + 0.010]]              # spar-web flange
    F.append({"id": "F-REARSLOT", "part": "YK250-CH-054", "name": "rear-spar slot fitting (outer-panel lug, vertical pin)",
              "name_tr": "arka kiriş yuva bağlantısı (dış panel kulağı, düşey pim)", "material": "al_7075_t651_plate",
              "process": "cnc_milling_metal", "mirror": True, "point": r3(rp["position"]),
              "boxes": r3([rb_s, rb_r, rb_w]), "box": _bbox([rb_s, rb_r, rb_w]),
              "bolts": [bolt("P-REAR", rp["position"], [0.0, 0.0, 1.0], rp["diameter"], rp["spec"],
                             ["rear-spar lug YK250-WG-152"], "rear pin"),
                        bolt("B1", [xw + 0.008, YJ - T_SW / 2, zr_], AY, 0.004, "M4 Ti (ISO 4762) + nutplate, to the "
                             "joint rib", ["M-JOINTRIB"], "joint rib"),
                        bolt("B2", [xw + 0.020, YJ - T_SW / 2, zr_], AY, 0.004, "M4 Ti (ISO 4762) + nutplate, to the "
                             "joint rib", ["M-JOINTRIB"], "joint rib"),
                        bolt("B3", [xw, 0.642, zr_], AX, 0.005, "M5 Ti (ISO 4762) + nutplate, to the rear-spar web",
                             ["M-CTBOX"], "rear spar web"),
                        bolt("B4", [xw, 0.660, zr_], AX, 0.005, "M5 Ti (ISO 4762) + nutplate, to the rear-spar web",
                             ["M-CTBOX"], "rear spar web")],
              "attach": "layout.chassis.wing_joint.rear_spar.slot_fitting",
              "touch": ["M-CTBOX", "M-JOINTRIB"]})
    # ------------------------------------------------------------------ parachute bridle fittings on the spine (S1-04)
    # fix round 2 (VS2-01 / PK2-10): Ti-6Al-4V shackle pin d 8 in a tight clevis (ears 6 mm, 0.5 mm gaps, 5 mm 4130
    # bridle spool), pin bending checked (structures P-SHACKLE-BEND); ears e = 15.5 mm (1.94 D, lug rule) so that the
    # ear tops keep the 6.0 mm GFRP cover strip P-SPINE to the OML; base strips beside the link slot, 4 x M4 floor bolts
    xsf, xsa = X_PFR + 0.0034, X_RS0 - 0.0034
    zf0, z_pin, e_ear = 0.165, 0.173, 0.0155
    for fid, part, frame, x_face, sg, ntr, nm in (
            ("F-RISER-FWD", "YK250-CH-112", "ST-FS1810", xsf, 1.0, "paraşüt ön kayış bağlantısı",
             "parachute bridle forward attach fitting"),
            ("F-RISER-AFT", "YK250-CH-113", "ST-FS-RS", xsa, -1.0, "paraşüt arka kayış bağlantısı",
             "parachute bridle aft attach fitting")):
        def bx(a0, a1, y0, y1, z0, z1):
            xa_, xb_ = sorted([x_face + sg * a0, x_face + sg * a1])
            return [[xa_, y0, z0], [xb_, y1, z1]]
        xp = x_face + sg * 0.026
        boxes = [bx(0.0, 0.006, -0.018, 0.018, zf0, 0.185),                     # flange on the frame face
                 bx(0.006, 0.050, 0.003, 0.021, zf0, 0.170), bx(0.006, 0.050, -0.021, -0.003, zf0, 0.170),  # base strips
                 bx(0.0105, 0.0415, 0.003, 0.009, 0.170, z_pin + e_ear),         # ears
                 bx(0.0105, 0.0415, -0.009, -0.003, 0.170, z_pin + e_ear)]
        bl = [bolt(f"B{k + 1}", [x_face + sg * dx, dy, zf0], AZ, 0.004, "M4 12.9 (ISO 4762) + self-locking nut, through "
                   "the spine channel floor (pad-up 3.2 mm, one 7075 washer plate 48 x 40 x 3 mm under the floor): "
                   "bridle x-component in shear, fitting moment in tension", ["M-SPINE"], "spine floor")
              for k, (dx, dy) in enumerate((dx, dy) for dx in (0.016, 0.040) for dy in (-0.013, 0.013))]
        bl += [bolt(f"B{5 + k}", [x_face, dy, 0.175], AX, 0.005, M5 + ", to the frame land (vertical component)",
                    [frame], "frame") for k, dy in enumerate((-0.008, 0.008))]
        F.append({"id": fid, "part": part, "name": nm, "name_tr": ntr, "material": "al_7075_t651_plate",
                  "process": "cnc_milling_metal", "frame": frame[3:],
                  "point": r3([xp, 0.0, z_pin]), "boxes": r3(boxes), "box": _bbox(boxes), "bolts": bl,
                  "lug": {"bore": 0.008, "e_m": e_ear, "t_m": 0.006, "w_m": 0.031, "ears": 2, "gap_m": 0.0005,
                          "rule": "U-lug ears 2 x 6 mm, e = 15.5 mm = 1.94 D: lug exception (S1-09), checked with the "
                                  "Bruhn lug relations at the actual e/D (structures P-LUG*)"},
                  "shackle_pin": {"d": 0.008, "material": "ti_6al_4v_annealed_sheet",
                                  "spec": "headed Ti-6Al-4V pin d 8 f7 x 24 mm, castellated nut + cotter pin (Ti sheet "
                                          "allowables used as bar estimate)"},
                  "bridle_spool": {"material": "steel_4130_n", "od_m": 0.012, "id_m": 0.0082, "length_m": 0.005,
                                   "text": "bridle termination (fix round 2, VS2-01): the sewn webbing loop of the "
                                           "bridle leg (UAVOS supply, loop strength = supplier rating) on a 4130 spool "
                                           "OD 12 x 5 mm on the shackle pin; 0.5 mm gap to each ear (tight clevis); the "
                                           "loop rests in the slot between the base strips when stowed"},
                  "attach": "U-lug fitting in the dorsal spine channel M-SPINE: flange 6 mm on the frame face with 2 x M5 "
                            "12.9 (axis x) into the frame land (vertical component), two base strips beside the link "
                            "slot with 4 x M4 12.9 (axis z) through the channel floor pad (x-component in shear, the "
                            "moment of the pin offset in tension; structures P-SPINE-BOLTS / P-FIT-MOMENT / "
                            "P-SPINE-PULL), two 6 mm ears with the Ti shackle pin d 8 at 26 mm from the frame face",
                  "touch": ["M-SPINE", frame]})
    return F
