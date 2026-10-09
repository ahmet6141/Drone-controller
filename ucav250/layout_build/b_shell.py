"""Layout builder: shell (skin panels, hatches, doors, fairings, RF windows) and the fastening rules."""
from __future__ import annotations

from .b_common import *  # noqa: F401,F403

CAM = {"type": "camloc", "spec": "Camloc 4002 stud + 2600 receptacle (quarter-turn, riveted receptacle)",
       "pitch": [0.075, 0.100], "edge_margin": 0.012}
NUT = {"type": "nutplate+screw", "spec": "ISO 7380 M4 A2-70 button head into floating anchor nutplate M4",
       "pitch": [0.025, 0.032], "edge_margin": 0.010}
NUT_SEAL = dict(NUT, spec="ISO 7380 M4 A2-70 into floating nutplates + fuel-resistant fluorosilicone gasket "
                          "(vapour-tight fuel-bay panel)", pitch=[0.025, 0.030])
INS = {"type": "insert+screw", "spec": "ISO 7380 M4 A2-70 into potted M4 inserts (fasteners_catalog.INSERT)",
       "pitch": [0.060, 0.100], "edge_margin": 0.012}
BOND = {"type": "bonded", "spec": "EA 9394 paste adhesive on the frame/longeron lands, 0.2 mm bondline; peel "
                                   "stoppers: M4 blind rivets every 150 mm at panel ends", "pitch": None,
        "edge_margin": None}

RULES = {
    "concept": "chassis + shell: the chassis (frames, longerons, keel members, decks, wing box) is closed and "
               "self-supporting for the jig assembly; the shell skins are the shear webs of the semi-monocoque. "
               "Structural skins ('fixed', semi-permanent) are fastened with M4 screws into floating nutplates at "
               "25-32 mm pitch (>= 3 D, <= 8 D) so every skin can be replaced; access panels and hatches are "
               "non-structural (framed cut-outs with doublers on frames/longerons) and use Camloc quarter-turn "
               "fasteners at 75-100 mm; fairings use M4 screws into potted inserts; the LERX/glove skins are primary "
               "wing skins and are bonded",
    "edge_distance": "fastener centre to panel edge and to land edge >= 2.5 D in composite (processes."
                     "prepreg_ooa_vacbag.edge_distance_D_min) -> 10 mm for M4, 12 mm for the Camloc stud; land "
                     "width >= 25 mm",
    "sandwich_edges": "core ramped out 1:3 to a 1.6 mm solid laminate edge band 20 mm wide at every fastener line, "
                      "joggle and hinge",
    "joggles": "removable panels sit flush in a joggled recess of the neighbouring fixed skin or on frame/longeron "
               "flanges: joggle depth = panel thickness + 0.3 mm, ramp 1:10, land 25 mm; panel gap 1.0 +- 0.3 mm, "
               "step <= 0.3 mm (aero surface)",
    "seals": "hatches and gear/turret doors carry perimeter seals (aero.drag_rules.gear_doors_sealed); fuel-bay "
             "panels a fuel-resistant gasket",
    "tolerances": "ISO 2768-mK for trimmed edges and holes (processes); hole positions drilled through jigs from "
                  "the master frame datums",
    "layups": "body skins and doors: layups.shell_secondary (0.4/5/0.4 mm); LERX/glove: layups.wing_skin_primary "
              "(upper skin between the spar caps: layups.wing_box_skin_upper); "
              "RF windows: gfrp_7781_mtm45 (2 plies 0/90 + 5 mm ROHACELL + 2 plies, no carbon within the window); "
              "cowl: layups.shell_secondary with stainless heat-shield patches at the exhaust exits"}


def panel(pid, part, name, name_tr, surface, x, y, attach, fastening, material="cfrp_pw_mtm45_as4",
          layup="shell_secondary", mirror=False, rf_window=False, hinge=None, notes="", lands=None, joint=None,
          outline=None, cutouts=None, free_edges=None, z_band=None, process=None, thickness=None):
    d = {"id": pid, "part": part, "name": name, "name_tr": name_tr, "surface": surface, "x": r3(x), "y": r3(y),
         "attach": attach, "fastening": fastening, "material": material,
         "process": process or ("prepreg_ooa_vacbag" if layup else "sheet_metal_aluminium"), "layup": layup}
    if not layup:
        # formed 6061-T6 cowl piece 0.8 mm (fix round 2); solid laminates give their thickness (fix round 3, PK3-08)
        d["thickness_m"] = thickness if thickness is not None else 0.0008
    if outline is not None:                 # plan polygon [[x, y], ...] of a non-rectangular panel (x / y = its box)
        d["outline"] = r3(outline)
    if mirror:
        d["mirror"] = True
    if rf_window:
        d["rf_window"] = True
    if hinge:
        d["hinge"] = hinge
    if joint:
        d["joint"] = joint
    if lands:
        d["lands"] = lands
    if free_edges:
        d["free_edges"] = list(free_edges)  # edges without a land (cowl exit lip ring)
    if z_band:
        d["z_band"] = r3(z_band)            # height band of a side panel (cowl pieces, stub-root strip)
    if cutouts:
        d["cutouts"] = cutouts
    if notes:
        d["notes"] = notes
    return d


TEAR = {"type": "tear-away", "spec": "hook-and-loop on the spine flanges + 4 nylon M3 shear screws (tear-away under the "
                                     "bridle pull)", "pitch": None, "edge_margin": 0.010}
FIN_LANDS = "integral flanges of the fin root rib (tail module)"


def _frame_edge(x_web, sign):
    """x of a panel edge sitting on a frame cap: 3 mm past the web centre towards the panel (edge band 25 mm on the web
    + T-flange 28 mm, layout_check C09 land rule)."""
    return round(float(x_web) + sign * 0.003, 4)


RING_X = (_frame_edge(1.1066, -1.0) - 0.022, _frame_edge(1.3334, 1.0) + 0.022)   # turret aperture ring insert x
RING_Y = 0.125
X_SB_NOTCH, Y_SB_NOTCH = 1.0706, 0.135      # side-bay / door-access panel notches round the ring (fix round 3, PK3-02)
X_PH1 = round(1.8134 + 0.010, 4)            # parachute hatch aft edge: FS1810 web aft face + 10 mm (PK3-02)


def panels() -> list:
    G = "gfrp_7781_mtm45"
    pay = ZP["payload_bay"]["box"]
    X = []
    X.append(panel("P-NOSECONE", "YK250-SH-350", "nose cone (radome)", "burun konisi (radom)", "body_full",
                   [0.000, 0.300], [-0.10, 0.10], "removable", dict(INS, spec="8 x M4 screws into nutplates on the "
                   "FS0300 flange (radial)", pitch=[0.040, 0.060]), material=G, rf_window=True,
                   lands=["ST-FS0300"], free_edges=["fore", "sides"], notes="GFRP: RF window for datalink antenna A, backup diversity, FTS antenna "
                                              "and the Remote ID beacon (on the FS0300 forward face, fix round 1 "
                                              "VPK-10); pitot boss (inserts) at the tip"))
    X.append(panel("P-FWDSKIN", "YK250-SH-351", "forward body skin", "ön gövde kaplaması", "body_full",
                   [0.300, 0.600], [-0.17, 0.17], "fixed", NUT, lands=["ST-FS0300", "ST-FS0600"],
                   notes="one-piece wrap (upper + lower halves co-cured at the chine edge band); cut-out: P-FWDHATCH; "
                         "transponder blade ANT-XPDR (x 0.45, belly) on a bonded copper-mesh ground-plane doubler "
                         "(fix round 3, PK3-11: the doubler lies in this skin, not in P-NOSE-LOWER)",
                   cutouts=[{"id": "P-FWDHATCH", "kind": "hatch"}]))
    xa, xb = _frame_edge(0.3034, 1.0) - 0.0003, 0.5996
    X.append(panel("P-FWDHATCH", "YK250-SH-353", "forward-bay hatch (buffer battery, FTS unit)",
                   "ön bölme kapağı (tampon batarya, FTS)", "body_upper", [xa, xb], [-0.130, 0.130], "removable",
                   CAM, lands=["ST-FS0300", "ST-FS0600", "P-FWDSKIN"],
                   outline=[[xa, -0.050], [xb, -0.130], [xb, 0.130], [xa, 0.050]],
                   notes="fix round 1 (VPK-03): trapezoid following the chine taper (>= 25 mm inboard of the chine), "
                         "fore and aft edges on the FS0300 / FS0600 caps: clear opening about 0.25 x 0.10-0.21 m passes "
                         "the 184 x 77 mm battery footprint; battery box vent outlet grille (flame-arresting mesh) in "
                         "this hatch; the FTS unit below the battery is reached with the battery lifted out (2 straps, "
                         "1 connector)"))
    X.append(panel("P-NOSE-LOWER", "YK250-SH-352", "lower nose skin", "alt burun kaplaması", "body_lower",
                   [0.600, 1.110], [-0.30, 0.30], "fixed", NUT, lands=["ST-FS0600", "ST-FS1110", "M-CHINE",
                                                                       "M-KEELWALL"],
                   notes="cut-outs: keel slot (nose-gear clamshell doors), side-bay access panels, turret ring "
                         "insert (forward edge)",
                   cutouts=[{"id": "NOSE-KEEL-SLOT", "kind": "gear door opening",
                             "outline": "layout.mechanisms.door_outlines.nose_door_R/L (closed)"},
                            {"id": "P-SIDEBAY-L", "kind": "hatch"}, {"id": "P-SIDEBAY-R", "kind": "hatch"},
                            {"id": "P-TURRETRING", "kind": "aperture insert (forward edge on the FS1110 cap)"}]))
    X.append(panel("P-AVHATCH", "YK250-SH-354", "avionics hatch (RF window GNSS 1)", "aviyonik kapağı (GNSS 1 RF penceresi)",
                   "body_upper", [_frame_edge(0.6, 1.0), _frame_edge(1.11, -1.0)], [-0.135, 0.135], "removable", CAM,
                   material=G, rf_window=True, lands=["ST-FS0600", "ST-FS1110", "P-NOSE-UPPER"],
                   notes="2 alignment dowels at the front, perimeter seal; fore and aft edges on the FS0600 / FS1110 "
                         "caps; access to PDU deck, power switching, nose harness"))
    X.append(panel("P-NOSE-UPPER", "YK250-SH-355", "upper nose skin (hatch frame)", "üst burun kaplaması",
                   "body_upper", [0.600, 1.110], [-0.30, 0.30], "fixed", NUT, lands=["ST-FS0600", "ST-FS1110",
                                                                                      "M-CHINE"],
                   cutouts=[{"id": "P-AVHATCH", "kind": "hatch"}]))
    for sd, ys, nm, ntr, mat_, rf, note in (
            ("L", [-0.200, -0.050], "port side-bay access panel (generator PE, brake unit)",
             "sol yan bölme kapağı (jeneratör güç elektroniği, fren birimi)", "cfrp_pw_mtm45_as4", False,
             "generator PE heat-sink plate on the panel inner face + flush louvre (about 48 W at 800 W, 94 % "
             "efficiency; thermal check open)"),
            ("R", [0.050, 0.200], "starboard side-bay access panel", "sağ yan bölme kapağı", G, True,
             "access to autopilot, datalinks, transponder (fix round 1: the Remote ID beacon moved to the nose cone)")):
        sg = -1.0 if sd == "L" else 1.0
        xb_ = _frame_edge(1.11, -1.0)
        ol = [[0.830, sg * 0.050], [X_SB_NOTCH, sg * 0.050], [X_SB_NOTCH, sg * Y_SB_NOTCH], [xb_, sg * Y_SB_NOTCH],
              [xb_, sg * 0.200], [0.830, sg * 0.200]]
        X.append(panel(f"P-SIDEBAY-{sd}", f"YK250-SH-{357 if sd == 'L' else 358}", nm, ntr, "body_lower",
                       [0.830, xb_], ys, "removable", CAM, material=mat_, rf_window=rf, outline=ol,
                       lands=["M-KEELWALL", "ST-FS1110", "P-NOSE-LOWER"],
                       notes=note + "; fix round 1 (VPK-03): forward edge moved to x 0.830 and aft edge onto the FS1110 "
                                    f"cap so that every side-bay unit lies under the clear opening; fix round 3 (PK3-02): "
                                    f"inboard of |y| {Y_SB_NOTCH} the aft edge ends at x {X_SB_NOTCH} on the joggled land "
                                    "of P-NOSE-LOWER (the turret ring insert P-TURRETRING owns the FS1110 cap there)"))
    X.append(panel("P-MID-UPPER", "YK250-SH-360", "upper mid skin", "üst orta kaplama", "body_upper", [1.110, X_PFF],
                   [-0.37, 0.37], "fixed", NUT, lands=["ST-FS1110", "ST-FS1330", "ST-FS1490", "M-CHINE"],
                   cutouts=[{"id": "P-PARAHATCH", "kind": "hatch edge (FS1490 cap, |y| <= 0.188)"}]))
    X.append(panel("P-MID-LOWER", "YK250-SH-361", "lower mid skin (turret bay cut-out)", "alt orta kaplama",
                   "body_lower", [1.110, X_PFF], [-0.37, 0.37], "fixed", NUT, lands=["ST-FS1110", "ST-FS1330",
                                                                                      "ST-FS1490", "M-TURRETWALL"],
                   cutouts=[{"id": "P-TURRETRING", "kind": "aperture insert"},
                            {"id": "P-TDOORACC", "kind": "hatch (L/R)"}]))
    xr0, xr1 = RING_X
    X.append(panel("P-TURRETRING", "YK250-SH-363", "turret aperture ring insert (HD59, 157 mm opening)",
                   "taret açıklık halkası (HD59)", "body_lower", [round(xr0, 4), round(xr1, 4)], [-RING_Y, RING_Y],
                   "removable", dict(INS, spec="14 x ISO 7380 M4 A2-70 into potted M4 inserts in the frame caps (fore/aft) "
                                               "and in the bonded side land doublers of the lower skin under the door "
                                               "rail plane (sides, fix round 3 PK3-09): pitch about 67 mm",
                                     pitch=[0.060, 0.080], edge_margin=0.012), layup=None,
                   process="prepreg_ooa_vacbag", thickness=0.0020,
                   lands=["ST-FS1110", "ST-FS1330", "M-TURRETWALL"],
                   notes=f"fix round 1 (VPK-08): flush skin insert {(xr1 - xr0) * 1000:.0f} x {2 * RING_Y * 1000:.0f} mm, "
                         "CFRP solid laminate 2.0 mm (10 plies PW, prepreg_ooa_vacbag; fix round 3 PK3-08), landing on the "
                         "outward caps of FS1110 / FS1330 and the side land doublers (M-TURRETWALL.side_land); the fastener rows "
                         "lie >= 17 mm from the interchangeable E180 ring's 0.19 m opening (YK250-SH-364) and >= 30 mm "
                         "from the HD59 opening (ball radius + payload.turret.bay.aperture_ring.radial_clearance); the "
                         "ring land (2.0 mm insert + 1.6 mm flange) stays within the 5.8 mm skin thickness and the "
                         "inserts are flush, so nothing protrudes into the band of the sliding doors "
                         "(layout.mechanisms.door_outlines.turret_door_R/L)"))
    xt0_, xt1_ = _frame_edge(1.33, 1.0) - 0.003, _frame_edge(1.49, -1.0) + 0.003
    X.append(panel("P-TDOORACC", "YK250-SH-365", "turret-door drive access panel (lower skin aft of FS1330)",
                   "taret kapağı tahrik erişim kapağı", "body_lower", [xt0_, xt1_],
                   [0.100, 0.220], "removable", CAM, mirror=True, lands=["ST-FS1330", "ST-FS1490", "P-MID-LOWER"],
                   outline=[[round(xr1 + 0.010, 4), 0.100], [xt1_, 0.100], [xt1_, 0.220], [xt0_, 0.220],
                            [xt0_, Y_SB_NOTCH], [round(xr1 + 0.010, 4), Y_SB_NOTCH]],
                   notes="fix round 3 (PK3-02): inboard of |y| 0.135 the forward edge starts 10 mm aft of the turret "
                         "ring insert (on the joggled land of P-MID-LOWER), so that the ring alone owns the FS1330 cap "
                         "there; fix round 1 (VPK-08): moved out of the band swept by the sliding doors: framed cut-out "
                         "between the caps of FS1330 and FS1490 under the door actuator EQ-TDOORACT (aft face of "
                         "FS1330, drive shaft through C-TDOOR-SHAFT to the pinion on the door rack); clear opening "
                         "about 110 x 70 mm; the racks and rails are inspected from inside the bay with the aperture "
                         "ring removed and the doors open"))
    X.append(panel("P-PARA-SURR", "YK250-SH-366", "upper skin around the parachute hatch", "paraşüt kapağı çevre kaplaması",
                   "body_upper", [X_PFF, X_PFR], [-0.40, 0.40], "fixed", NUT, lands=["ST-FS1490", "ST-FS1810",
                                                                                     "M-PARAWALL", "M-CHINE"],
                   cutouts=[{"id": "P-PARAHATCH", "kind": "hatch (tethered lift-off)"}]))
    xh0, xh1 = round(1.4866 - 0.0246, 4), X_PH1           # fix round 3 (PK3-02): aft edge 10 mm aft of the FS1810 web
    X.append(panel("P-PARAHATCH", "YK250-SH-367", "parachute hatch (dorsal, tethered lift-off)",
                   "paraşüt kapağı (sırt, bağlı fırlatmalı)", "body_upper", [xh0, xh1], [-0.188, 0.188], "removable",
                   {"type": "latch+tether", "spec": "no hinge (fix round 1, VPK-11): four locating tongues + one "
                    "pin-puller latch (YK250-SY-804) with cable-linked pins at the four corners; when the pins retract "
                    "the deploying canopy pack lifts the hatch straight off; 1.5 m aramid tether to FS1810 keeps it "
                    "attached", "pitch": None, "edge_margin": 0.012}, joint="para_hatch",
                   lands=["ST-FS1490", "ST-FS1810", "M-PARAWALL"],
                   notes=f"fix round 1 (VPK-03): {(xh1 - xh0) * 1000:.0f} x 376 mm hatch on the outward flanges of "
                         "FS1490 / FS1810 and of the bay walls (outside the 300 x 300 mm container footprint): clear "
                         "opening 313 x 312 mm between the frame and wall faces; the hatch follows the V roof (one-piece "
                         "GFRP/CFRP sandwich, no hinge line); fix round 3 (PK3-02): at FS1810 the hatch edge band lies on "
                         "the web and the first 10 mm of the aft flange, the tear-away strip P-SPINE starts 1 mm aft of it "
                         "on the rest of the flange (side by side, neither on top); P-MID-UPPER and P-MB-UPPER are cut "
                         "round the hatch over |y| <= 0.188"))
    X.append(panel("P-PARA-LOWER", "YK250-SH-368", "lower skin parachute bay", "paraşüt bölmesi alt kaplaması",
                   "body_lower", [X_PFF, X_PFR], [-0.40, 0.40], "fixed", NUT, lands=["ST-FS1490", "ST-FS1810"]))
    X.append(panel("P-MB-UPPER", "YK250-SH-370", "upper skin mission bay (structural)", "görev bölmesi üst kaplaması",
                   "body_upper", [X_PFR, X_FUELF], [-0.40, 0.40], "fixed", NUT,
                   lands=["ST-FS1810", "ST-FS-FUEL", "M-CHINE", "M-SPINE"],
                   notes="structural skin screwed (M4 nutplates, 25 mm pitch) to the spine-channel flanges, the frames "
                         "and the chine longerons: shear path of the net bridle x-component (structures P-SPINE-SKIN); "
                         "GNSS 2 window cut-out; fix round 3 (PK3-02): cut round the parachute hatch edge (FS1810 cap) "
                         "and the tear-away strip P-SPINE (over the spine channel)",
                   cutouts=[{"id": "P-GNSS2", "kind": "RF window insert"},
                            {"id": "P-PARAHATCH", "kind": "hatch edge (FS1810 cap, |y| <= 0.188)"},
                            {"id": "P-SPINE", "kind": "cover strip (spine channel)"}]))
    X.append(panel("P-GNSS2", "YK250-SH-372", "GNSS 2 RF window", "GNSS 2 RF penceresi", "body_upper",
                   r3([X_GNSS2 - 0.040, X_GNSS2 + 0.040]), [0.110, 0.190], "removable", INS, material=G, rf_window=True,
                   lands=["P-MB-UPPER"], notes="outboard of the spine flanges (no carbon above the antenna)"))
    X.append(panel("P-MBHATCH", "YK250-SH-373", "mission-bay hatch (belly)", "görev bölmesi kapağı", "body_lower",
                   [_frame_edge(X_PFR, 1.0), _frame_edge(X_FUELF, -1.0) + 0.003], [-0.155, 0.155], "removable", CAM,
                   lands=["ST-FS1810", "ST-FS-FUEL", "P-MB-LOWER"],
                   notes="fix round 1 (VPK-03/VPK-12): fore and aft edges on the FS1810 / FS-FUEL caps, sides in the "
                         "joggled land of the lower skin; clear opening about 0.345 x 0.26 m: the removable equipment "
                         "tray TR-MISSION (mission computer + engine ECU, closes the centre cut-out of the mission-bay "
                         "floor M-MIDFLOOR) is unscrewed from below and lowered out"))
    X.append(panel("P-MB-LOWER", "YK250-SH-374", "lower skin mission bay", "görev bölmesi alt kaplaması", "body_lower",
                   [X_PFR, X_FUELF], [-0.40, 0.40], "fixed", NUT, lands=["ST-FS1810", "ST-FS-FUEL", "M-CHINE"],
                   cutouts=[{"id": "P-MBHATCH", "kind": "hatch"}]))
    X.append(panel("P-SPINE", "YK250-SH-376", "tear-away bridle cover strip (over the spine channel)",
                   "yırtılır kayış örtüsü (sırt kanalı üstü)", "body_upper", [round(X_PH1 + 0.001, 4), X_RS0 + 0.016],
                   [-0.045, 0.045], "removable", TEAR, material=G,
                   lands=["M-SPINE", "ST-FS1810", "ST-FS-RS"],
                   notes="fix round 1 (VPK-04/S1-04): non-structural GFRP cover strip, lands on the inner 25 mm of the "
                         "spine-channel flanges; the fuel-bay panels land on the outer part of the same flanges (a bonded "
                         "flush separator strip y 0.045-0.055 between them)"))
    fs = {s_: 0.0 for s_ in ("FS-FUEL", "FS-MS", "FS-RS", "FS-GEAR")}
    swp = {"FS-FUEL": 0.0, "FS-MS": SW_MS, "FS-RS": SW_RS, "FS-GEAR": 0.0}
    xw = {"FS-FUEL": X_FUELF, "FS-MS": X_MS0, "FS-RS": X_RS0, "FS-GEAR": X_GEARF}

    def fx(fr, y, sign):
        return round(xw[fr] + abs(y) * math.tan(math.radians(swp[fr])) + sign * 0.003, 4)
    y_in, y_out = 0.055, 0.300
    for k, (fa, fb, nm, ntr) in enumerate((("FS-FUEL", "FS-MS", "forward", "ön"), ("FS-MS", "FS-RS", "saddle", "orta"),
                                           ("FS-RS", "FS-GEAR", "aft", "arka"))):
        ol = [[fx(fa, y_in, 1.0), y_in], [fx(fb, y_in, -1.0), y_in], [fx(fb, y_out, -1.0), y_out],
              [fx(fa, y_out, 1.0), y_out]]
        X.append(panel(f"P-FUEL{k + 1}", f"YK250-SH-{377 + k}", f"{nm} fuel-bay access panel",
                       f"{ntr} yakıt bölmesi kapağı", "body_upper",
                       [min(q[0] for q in ol), max(q[0] for q in ol)], [y_in, y_out], "removable", NUT_SEAL,
                       mirror=True, outline=ol,
                       lands=[f"ST-{fa}", f"ST-{fb}"] + (["M-SPINE"] if fa != "FS-RS" else []) + ["P-CENTRE-UPPER"],
                       notes="fix round 1 (VPK-04): fore and aft edges parallel to the (swept) frames on the frame-cap "
                             "T-flanges (25 mm edge band each, adjacent panels share a cap with a 6 mm gap over the "
                             "web, flush filler strip), inboard edge on the spine-channel flange (y >= 0.055), outboard "
                             "edge in the joggled land of the fixed centre skin; semi-permanent (bladder installation/"
                             "inspection), vapour-tight gasket"))
    X.append(panel("P-CENTRE-UPPER", "YK250-SH-380", "upper centre skin (fuel bays)", "üst orta gövde kaplaması",
                   "body_upper", [X_FUELF, X_GEARF], [-0.40, 0.40], "fixed", NUT,
                   lands=["ST-FS-FUEL", "ST-FS-MS", "ST-FS-RS", "ST-FS-GEAR", "M-CHINE"],
                   notes="outboard strips (fuel-panel outboard edges to the chine); the fuel panels sit on the frame "
                         "caps and the spine flanges",
                   cutouts=[{"id": f"P-FUEL{k}", "kind": "hatch (L/R)"} for k in (1, 2, 3)] +
                           [{"id": "P-SPINE", "kind": "cover strip"}]))
    X.append(panel("P-PAYHATCH", "YK250-SH-382", "payload-bay hatch (belly)", "faydalı yük bölmesi kapağı (karın)",
                   "body_lower", [_frame_edge(X_FUELF, 1.0), round(X_PB1 + 0.0016, 4)], [-0.222, 0.222], "removable",
                   CAM, lands=["M-KEEL", "ST-FS-FUEL", "M-PAYWALL-AFT"],
                   notes="fix round 1 (VPK-12): forward edge on the FS-FUEL cap (the keel beams start there), aft edge "
                         "on the payload-bay aft wall M-PAYWALL-AFT, sides on the keel-beam flanges; payload-specific "
                         "panels (sensor windows) replace it"))
    xrf = [2.300, 2.380]
    X.append(panel("P-CENTRE-LOWER", "YK250-SH-383", "lower centre skin", "alt orta gövde kaplaması", "body_lower",
                   [X_FUELF, X_GEARF], [-0.40, 0.40], "fixed", NUT,
                   lands=["ST-FS-FUEL", "ST-FS-MS", "ST-FS-RS", "ST-FS-GEAR", "M-KEEL", "M-GEARBEAM", "M-CHINE"],
                   notes="cut-outs with explicit outlines (fix round 1, VPK-09): payload hatch, main wells (inner doors "
                         "+ leg doors + trunnion doors: layout.mechanisms.door_outlines; fix round 2, PK2-03: the leg "
                         "slot runs out to the gear beam in the chine corner), refuel door; the centre strip between the "
                         "wells closes the harness channel",
                   cutouts=[{"id": "P-PAYHATCH", "kind": "hatch"},
                            {"id": "MAIN-WELL-R", "kind": "gear door opening",
                             "outline": "layout.mechanisms.door_outlines.main_inner_door_R + main_leg_door_R + "
                                        "main_trunnion_door_R"},
                            {"id": "MAIN-WELL-L", "kind": "gear door opening",
                             "outline": "layout.mechanisms.door_outlines.main_inner_door_L + main_leg_door_L + "
                                        "main_trunnion_door_L"},
                            {"id": "P-REFUEL", "kind": "door"}]))
    X.append(panel("P-REFUEL", "YK250-SH-385", "refuel access door (port)", "yakıt ikmal kapağı (sol)", "body_lower",
                   xrf, [-0.350, -0.290], "hinged", {"type": "hinge+latch", "spec": "flush piano hinge "
                   "(forward edge, straight: the lower skin is flat there) + push latch", "pitch": None,
                   "edge_margin": 0.010}, lands=["P-CENTRE-LOWER"],
                   hinge={"axis_point": [xrf[0], -0.320, round(z_bot(xrf[0], 0.320) + 0.002, 4)],
                          "axis": [0.0, 1.0, 0.0], "range_deg": [0, 100]},
                   notes="fix round 1 (VPK-09): moved ahead of the main spar under the forward fuel cell, outboard of "
                         "the port keel beam and of H-MAIN: OBP dry-break AN6 coupling (components.yaml "
                         "obp_dry_break_an6) on a bracket under the forward fuel deck, short vertical line through the "
                         "declared deck penetration into the forward cell (layout.fuel_lines FL-REFUEL); refuelling "
                         "bonding point (CS-LUAS.867(d))"))
    X.append(panel("P-AFTHATCH", "YK250-SH-387", "aft equipment hatch (belly)", "arka teçhizat kapağı (karın)",
                   "body_lower", [_frame_edge(X_GEARF, 1.0) - 0.003, _frame_edge(3.48, -1.0) + 0.003], [-0.120, 0.120],
                   "removable", CAM, lands=["ST-FS-GEAR", "ST-FS3480", "P-AFT-LOWER"],
                   notes="fore and aft edges on the FS-GEAR / FS3480 caps (fix round 1); access: fuel pump / regulator / "
                         "filter (drain), inner-door actuators, tail connector, engine harness junction (the ECU is "
                         "reached through P-MBHATCH, the generator PE and the brake unit through P-SIDEBAY-L)"))
    for sd, ys in (("R", [0.022, 0.180]),):
        X.append(panel("P-STABACT", "YK250-SH-389", "lower hatch FS3480-firewall, L/R halves (stabilator actuators, "
                       "pushrods, fuel shut-off valve)", "alt kapak FS3480-yangın perdesi, sağ/sol yarımlar "
                       "(stabilatör eyleyicileri)", "body_lower", [_frame_edge(3.48, 1.0) - 0.0024, X_FW_FWD], ys,
                       "removable", CAM, mirror=True, lands=["ST-FS3480", "ST-FS3670", "M-VENTRALKEEL", "P-AFT-LOWER"],
                       notes="fix round 1 (VPK-05/VPK-12): split into L/R halves meeting on the ventral keel strip "
                             "M-VENTRALKEEL (the ventral fin root passes between them in the fixed strip P-VENTRALROOT); "
                             "fore edge on the FS3480 cap, aft edge on the firewall forward cap; the DA 30 actuators, "
                             "their pushrods / fireproof boots and the shut-off valve on the forward face of the "
                             "firewall are reached from below without removing the cowl or any frame"))
    X.append(panel("P-VENTRALROOT", "YK250-SH-394", "ventral fin root strip (FS3480 -> cowl exit)",
                   "ventral kök şeridi", "body_lower", [_frame_edge(3.48, 1.0) - 0.0024, 4.000], [-0.022, 0.022],
                   "fairing", INS, lands=["M-VENTRALKEEL", "M-AFTKEEL"],
                   notes="fixed fairing strip around the ventral fin root (fin, ventral and stub root cut lines are "
                         "interfaces: layout.shell.root_cut_lines), carried by the ventral keel strip and the aft keel "
                         "beam land flanges"))
    X.append(panel("P-AFT-LOWER", "YK250-SH-388", "lower aft skin", "alt arka kaplama", "body_lower", [X_GEARF, 3.670],
                   [-0.40, 0.40], "fixed", NUT, lands=["ST-FS-GEAR", "ST-FS3480", "ST-FS3670", "M-CHINE"],
                   notes="fuel vent outlet (flush NACA vent) and sump drain fittings",
                   cutouts=[{"id": "P-AFTHATCH", "kind": "hatch"}, {"id": "P-STABACT", "kind": "hatch (L/R)"},
                            {"id": "P-VENTRALROOT", "kind": "fairing strip"}]))
    X.append(panel("P-AFT-UPPER", "YK250-SH-390", "upper aft skin", "üst arka kaplama", "body_upper", [X_GEARF, 3.670],
                   [-0.40, 0.40], "fixed", NUT, lands=["ST-FS-GEAR", "ST-FS3480", "ST-FS3670", "M-DORSAL", "M-CHINE"],
                   notes="fin roots pass through it (root-rib flange screwed on top with sealant fillet)"))
    X.append(panel("P-INLET", "YK250-SH-391", "dorsal cooling inlet lip (flush NACA-type)", "sırt soğutma girişi",
                   "body_upper", [3.180, 3.440], [-0.090, 0.090], "fixed", BOND, lands=["P-AFT-UPPER"],
                   notes="first-cut inlet area 0.031 m2 (engine.yaml cooling_airflow_estimate); S-duct (YK250-PR-546) "
                         "to the plenum over the cylinders through the firewall duct cut-out"))
    X.append(panel("P-FINROOT", "YK250-SH-392", "fin root cover strip", "dikey kök örtüsü", "body_upper",
                   [3.268, 3.670], [0.100, 0.220], "fairing", INS, mirror=True,
                   notes="covers the fin root-rib flange screws, sealant fillet"))
    X.append(panel("P-FINROOT-AFT", "YK250-SH-395", "fin root fairing over the engine bay (fixed)",
                   "motor bölmesi üstü dikey kök kaplaması (sabit)", "cowl_upper", [3.670, 3.965], [0.125, 0.197],
                   "fairing", INS, mirror=True,
                   notes="fix round 1 (VPK-05): the fin root runs over the engine bay to x 3.93 at y 0.14-0.19; this "
                         "fixed strip is the integral flange of the fin root rib (tail module), the upper cowl pieces "
                         "land on it"))
    X.append(panel("P-STUBROOT", "YK250-SH-393", "stabilator-stub root fairing", "kök parçası fileto kaplaması",
                   "body_side", [3.415, 3.962], [0.236, 0.300], "fairing", INS, mirror=True, z_band=[0.175, 0.262],
                   notes="fixed strip at the chine (cowl split line) around the stub root; removable for stub removal; "
                         "the cowl side pieces land on it"))
    cam_c = dict(CAM, pitch=[0.070, 0.090])
    X.append(panel("P-COWL-UP", "YK250-SH-450", "upper cowl, centre piece (between the fin roots)",
                   "üst motor kaportası, orta parça", "cowl_upper", [3.670, 4.000], [-0.236, 0.236], "removable", cam_c,
                   lands=["ST-FS3670", "P-FINROOT-AFT", "P-COWL-UPS", "P-COWL-LO"], free_edges=["aft"],
                   outline=[[3.670, -0.125], [3.965, -0.125], [3.965, -0.236], [4.000, -0.236], [4.000, 0.236],
                            [3.965, 0.236], [3.965, 0.125], [3.670, 0.125]],
                   notes="fix round 1 (VPK-05): the single upper cowl is split around the fin roots; seals to the "
                         "cooling plenum; aft lip ring part (annular cooling exit around the hub spacer); aft of the "
                         "fin roots (x > 3.935) its side edges lap over the outer pieces (removed first)"))
    X.append(panel("P-COWL-UPS", "YK250-SH-452", "upper cowl, outer piece (fin root -> stub root)",
                   "üst motor kaportası, yan parça", "cowl_upper", [3.670, 3.965], [0.197, 0.236], "removable", cam_c,
                   mirror=True, lands=["ST-FS3670", "P-FINROOT-AFT", "P-STUBROOT", "P-COWL-LO", "P-COWL-UP"],
                   z_band=[0.262, 0.400], material="al_6061_t6_sheet", layup=None,
                   notes="fix round 1 (VPK-05): from the fin-root fairing down to the stub-root strip at the chine "
                         "(cowl split line); laps on the lower cowl aft of the stub root; fix round 2 (PK2-09): "
                         "aluminium 6061-T6 sheet 0.8 mm (formed, removable): its lower edge comes within 25 mm of the "
                         "cylinder-head envelope, where KO-CYL-HOT admits no composite"))
    X.append(panel("P-COWL-LO", "YK250-SH-451", "lower cowl, L/R halves", "alt motor kaportası, sağ/sol yarımlar",
                   "cowl_lower", [3.670, 4.000], [0.022, 0.236], "removable", cam_c, mirror=True,
                   lands=["ST-FS3670", "M-AFTKEEL", "P-STUBROOT", "P-COWL-UPS", "P-COWL-UP", "P-VENTRALROOT"],
                   free_edges=["aft"],
                   z_band=[-0.200, 0.175],
                   notes="fix round 1 (VPK-05): split into L/R halves meeting on the aft keel beam land flanges (the "
                         "ventral fin root passes between them in the fixed strip P-VENTRALROOT); exhaust exit "
                         "cut-outs with stainless 304 heat-shield patches (0.5 mm on 5 mm stand-offs); aft lip ring "
                         "parts"))
    gs = [sec for sec in S["wing"]["sections"] if float(sec["y"]) <= YJ + 1e-6]
    gx = [round(min(float(sec["x_le"]) for sec in gs), 4), round(max(float(sec["x_le"]) + float(sec["chord"])
                                                                      for sec in gs), 4)]
    X.append(panel("P-GLOVE-UP", "YK250-SH-420", "LERX/glove upper skin", "LERX/eldiven üst kaplaması",
                   "glove_upper", gx, [0.400, 0.700], "fixed", BOND, layup="wing_skin_primary",
                   mirror=True, lands=["M-CHINE", "M-SOB", "M-GLOVERIB", "M-JOINTRIB", "M-CTBOX"],
                   notes="primary wing skin (wing_skin_primary 0.6/5/0.4; between the spar caps "
                         "layups.wing_box_skin_upper 0.6/6/0.4, structures phase), bonded; LERX leading edge solid "
                         "laminate (impact)"))
    X.append(panel("P-GLOVE-LO", "YK250-SH-421", "LERX/glove lower skin", "LERX/eldiven alt kaplaması",
                   "glove_lower", gx, [0.400, 0.700], "fixed", BOND, layup="wing_skin_primary",
                   mirror=True, lands=["M-CHINE", "M-SOB", "M-GLOVERIB", "M-JOINTRIB", "M-CTBOX"],
                   notes="framed cut-outs for the joint access panels", cutouts=[
                       {"id": "P-JOINTACCESS", "kind": "hatch"}, {"id": "P-REARACCESS", "kind": "hatch"}]))
    # plan outline: forward edge 50 mm behind the swept LERX/glove leading edge (20 mm solid LE band + 25 mm land +
    # 5 mm), aft edge on the main-spar (fork) cap flange 10 mm ahead of the spar line; spanwise from the side-of-body rib
    # cap to the joint rib cap (fix round 1, VPK-02: the LERX ribs are deleted, both pins, keepers, the reamer and the
    # puller are under the clear opening)
    y_a, y_b = round(Y_SOB + T_WALL / 2 + 0.003, 3), round(YJ - T_WALL / 2 - 0.003, 3)
    ys_ = [round(y_a + (y_b - y_a) * k / 48, 4) for k in range(49)]
    fwd = [[float(af.wing.interpolate_section(y_)["x_le"]) + 0.055, y_] for y_ in ys_]
    aft = [[spar_x(y_, 0.25) - 0.026, y_] for y_ in (y_b, y_a)]
    ol = fwd + aft
    X.append(panel("P-JOINTACCESS", "YK250-SH-422", "wing-joint access panel (lower glove, ahead of the main spar)",
                   "kanat birleşim erişim kapağı (alt eldiven)", "glove_lower",
                   [min(q[0] for q in ol), max(q[0] for q in ol)], [y_a, y_b], "removable", CAM,
                   layup="shell_secondary", mirror=True, lands=["M-SOB", "M-JOINTRIB", "F-FORK", "P-GLOVE-LO"],
                   outline=ol,
                   notes="fix round 1 (VPK-02): extended inboard to the side-of-body rib; opened at every wing assembly: "
                         "both main pins + keepers, pin puller and reamer corridors (layout.mechanisms.assembly_paths); "
                         "the cut-out in the primary glove skin is framed (doubler land on the ribs and the fork cap) "
                         "and carries the skin loads around it; the panel itself is a non-structural cover "
                         "(layups.shell_secondary, joggled land keeps it flush)"))
    from .b_chassis import wing_joint
    rp = wing_joint()["rear_spar"]["pin"]["position"]
    X.append(panel("P-REARACCESS", "YK250-SH-423", "rear-pin access port (lower glove, under the rear-spar slot fitting)",
                   "arka pim erişim deliği (alt eldiven)", "glove_lower", [round(rp[0] - 0.015, 4), round(rp[0] + 0.015, 4)],
                   [round(rp[1] - 0.015, 4), round(rp[1] + 0.015, 4)], "removable",
                   {"type": "bayonet", "spec": "flush bayonet cap d 30 mm over a d 18 mm hole in a bonded GFRP doubler "
                    "ring of the glove lower skin (no fasteners)", "pitch": None, "edge_margin": None},
                   layup="shell_secondary", mirror=True, lands=["P-GLOVE-LO"],
                   notes="fix round 1 (S1-02): directly under the vertical rear pin P-REAR of the rear-spar slot "
                         "fitting F-REARSLOT: the ball-lock pin (head d 14) is inserted upward through the d 18 hole"))
    return X


def root_cut_lines() -> dict:
    """Fix round 1 (VPK-05): the lines where the fixed tail surfaces (fins, stabilator stubs, ventral fin) leave the
    fuselage / cowl surface: interfaces between the tail module, the fixed root strips and the removable cowl / hatch
    pieces (layout_check C09: no root point inside a removable panel). Starboard points (port mirrored)."""
    out = {}
    for name, part in (("fin", "YK250-TL-250"), ("stabilator_stub", "YK250-TL-252"), ("ventral", "YK250-TL-254")):
        srf = af.tail[name]
        e = srf.span_coords()
        pts = []
        for xc in np.linspace(0.0, 1.0, 13):
            for fr in (0.0, 1.0):
                for eta in np.linspace(float(e[0]), float(e[-1]), 600):
                    q = surf_point(srf, eta, float(xc), fr)
                    if not af.inside(q[None], margin=0.0)[0]:
                        pts.append(r3(q))
                        break
        out[name] = {"part": part, "points": pts, "mirror": name != "ventral"}
    out["text"] = ("root intersection of each fixed tail surface with the fuselage / cowl outer surface (chord fractions "
                   "0..1, both faces); fixed strips P-FINROOT / P-FINROOT-AFT, P-STUBROOT and P-VENTRALROOT cover "
                   "them; the cowl and hatch pieces are split along them")
    return out


FAIRING_Y_INNER = 0.30


def wing_root_fairing() -> dict:
    """Fix round 2 (PK2-01/10): the wing-root junction fairing. The LERX/glove root section (y = wing.sections[0].y)
    is up to ~47 mm thick above and below the body chine (z = 0 at y 0.40), where the body V-roof ends in a sharp edge:
    the union of the body and the wing lofts has an open step there that the shell closes with a fairing. Its outer
    surface is the glove root profile extruded inboard (streamwise) to its intersection with the body; it is defined
    for |y| from FAIRING_Y_INNER to the wing root (layout_check uses it as part of the OML in C03 / C04). Planform
    areas (where the extruded profile lies outside the body) are computed here."""
    srf = af.wing
    e = srf.span_coords()
    y_root = float(S["wing"]["sections"][0]["y"])
    n = 121
    loop = srf.loop_at(float(e[0]), n)
    up, lo = loop[:n][::-1], loop[n - 1:]
    up, lo = up[np.argsort(up[:, 0])], lo[np.argsort(lo[:, 0])]
    dx, dy = 0.01, 0.004
    xs = np.arange(float(up[0, 0]) + 0.5 * dx, float(up[-1, 0]), dx)
    ys = np.arange(FAIRING_Y_INNER + 0.5 * dy, y_root, dy)
    a_up = a_lo = 0.0
    for x in xs:
        zu, zl = float(np.interp(x, up[:, 0], up[:, 2])), float(np.interp(x, lo[:, 0], lo[:, 2]))
        Pu = np.column_stack([np.full_like(ys, x), ys, np.full_like(ys, zu - 0.0005)])
        Pl = np.column_stack([np.full_like(ys, x), ys, np.full_like(ys, zl + 0.0005)])
        a_up += float(np.count_nonzero(~af.inside(Pu, 0.0))) * dx * dy
        a_lo += float(np.count_nonzero(~af.inside(Pl, 0.0))) * dx * dy
    return {"part": "YK250-SH-424", "mirror": True, "y_inner_m": FAIRING_Y_INNER, "y_outer_m": r3(y_root),
            "definition": "outer surface = the glove root profile (wing.sections[0]) extruded inboard streamwise to "
                          "its intersection with the fuselage loft; between the wing root and y_inner_m",
            "planform_area_upper_m2": round(2.0 * a_up, 4), "planform_area_lower_m2": round(2.0 * a_lo, 4),
            "construction": "part of the bonded LERX/glove upper and lower skins (layups.wing_skin_primary) closed to "
                            "the chine longeron flange; the centre wing box caps and the gear-beam corner run under it",
            "status": "fix round 2 (PK2-01 / PK2-10): defined here as an OML interface; the sizing drag model and the "
                      "renders do not yet include it (it replaces a strip of the body V-roof and the open step face "
                      "of the root section; its wetted-area and drag effect is an open item for the OML module)"}
