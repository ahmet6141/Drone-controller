"""Layout builder: shell (skin panels, hatches, doors, fairings, RF windows) and the fastening rules."""
from __future__ import annotations

from b_common import *  # noqa: F401,F403

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
    "layups": "body skins and doors: layups.shell_secondary (0.4/5/0.4 mm); LERX/glove: layups.wing_skin_primary; "
              "RF windows: gfrp_7781_mtm45 (2 plies 0/90 + 5 mm ROHACELL + 2 plies, no carbon within the window); "
              "cowl: layups.shell_secondary with stainless heat-shield patches at the exhaust exits"}


def panel(pid, part, name, name_tr, surface, x, y, attach, fastening, material="cfrp_pw_mtm45_as4",
          layup="shell_secondary", mirror=False, rf_window=False, hinge=None, notes="", lands=None, joint=None):
    d = {"id": pid, "part": part, "name": name, "name_tr": name_tr, "surface": surface, "x": r3(x), "y": r3(y),
         "attach": attach, "fastening": fastening, "material": material,
         "process": "prepreg_ooa_vacbag", "layup": layup}
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
    if notes:
        d["notes"] = notes
    return d


def panels() -> list:
    G = "gfrp_7781_mtm45"
    pay = ZP["payload_bay"]["box"]
    X = []
    X.append(panel("P-NOSECONE", "YK250-SH-350", "nose cone (radome)", "burun konisi (radom)", "body_full",
                   [0.000, 0.300], [-0.10, 0.10], "removable", dict(INS, spec="8 x M4 screws into nutplates on the "
                   "FS0300 flange (radial)", pitch=[0.040, 0.060]), material=G, rf_window=True,
                   lands=["ST-FS0300"], notes="GFRP: RF window for datalink antenna A, backup diversity, FTS antenna; "
                                              "pitot boss (inserts) at the tip"))
    X.append(panel("P-FWDSKIN", "YK250-SH-351", "forward body skin", "ön gövde kaplaması", "body_full",
                   [0.300, 0.600], [-0.17, 0.17], "fixed", NUT, lands=["ST-FS0300", "ST-FS0600"],
                   notes="one-piece wrap (upper + lower halves co-cured at the chine edge band)"))
    X.append(panel("P-FWDHATCH", "YK250-SH-353", "forward-bay hatch (buffer battery, FTS unit)",
                   "ön bölme kapağı (tampon batarya, FTS)", "body_upper", [0.390, 0.590], [-0.060, 0.060], "removable",
                   CAM, lands=["ST-FS0300", "ST-FS0600", "P-FWDSKIN"],
                   notes="battery box vent outlet grille (flame-arresting mesh) in this hatch; the FTS unit below the "
                         "battery is reached with the battery lifted out (2 straps, 1 connector)"))
    X.append(panel("P-NOSE-LOWER", "YK250-SH-352", "lower nose skin", "alt burun kaplaması", "body_lower",
                   [0.600, 1.110], [-0.30, 0.30], "fixed", NUT, lands=["ST-FS0600", "ST-FS1110", "M-CHINE",
                                                                       "M-KEELWALL"],
                   notes="cut-outs: keel slot (nose-gear doors), side-bay access panels; transponder blade on a "
                         "bonded copper-mesh doubler at x 0.45"))
    X.append(panel("P-AVHATCH", "YK250-SH-354", "avionics hatch (RF window GNSS 1)", "aviyonik kapağı (GNSS 1 RF penceresi)",
                   "body_upper", [0.620, 1.100], [-0.140, 0.140], "removable", CAM, material=G, rf_window=True,
                   lands=["ST-FS0600", "ST-FS1110", "P-NOSE-UPPER"],
                   notes="2 alignment dowels at the front, perimeter seal; access to PDU deck, power switching, nose "
                         "harness"))
    X.append(panel("P-NOSE-UPPER", "YK250-SH-355", "upper nose skin (hatch frame)", "üst burun kaplaması",
                   "body_upper", [0.600, 1.110], [-0.30, 0.30], "fixed", NUT, lands=["ST-FS0600", "ST-FS1110",
                                                                                      "M-CHINE"]))
    X.append(panel("P-SIDEBAY-L", "YK250-SH-357", "port side-bay access panel (generator PE, brake unit)",
                   "sol yan bölme kapağı (jeneratör güç elektroniği, fren birimi)",
                   "body_lower", [0.860, 1.100], [-0.235, -0.050], "removable", CAM,
                   lands=["M-KEELWALL", "P-NOSE-LOWER"], notes="generator PE heat-sink plate on the panel inner face + flush "
                                                              "louvre (about 48 W at 800 W, 94 % efficiency; thermal check "
                                                              "open)"))
    X.append(panel("P-SIDEBAY-R", "YK250-SH-358", "starboard side-bay access panel (RF window Remote ID)",
                   "sağ yan bölme kapağı (uzaktan kimlik RF penceresi)", "body_lower", [0.860, 1.100], [0.050, 0.235],
                   "removable", CAM, material=G, rf_window=True, lands=["M-KEELWALL", "P-NOSE-LOWER"],
                   notes="access to autopilot, datalinks, transponder, Remote ID"))
    X.append(panel("P-MID-UPPER", "YK250-SH-360", "upper mid skin", "üst orta kaplama", "body_upper", [1.110, X_PFF],
                   [-0.37, 0.37], "fixed", NUT, lands=["ST-FS1110", "ST-FS1330", "ST-FS1490", "M-CHINE"]))
    X.append(panel("P-MID-LOWER", "YK250-SH-361", "lower mid skin (turret bay cut-out)", "alt orta kaplama",
                   "body_lower", [1.110, X_PFF], [-0.37, 0.37], "fixed", NUT, lands=["ST-FS1110", "ST-FS1330",
                                                                                      "ST-FS1490", "M-TURRETWALL"]))
    X.append(panel("P-TURRETRING", "YK250-SH-363", "turret aperture ring (HD59, 157 mm opening)",
                   "taret açıklık halkası (HD59)", "body_lower", [1.114, 1.326], [-0.112, 0.112], "removable",
                   dict(NUT, spec="8 x ISO 7380 M4 into nutplates on the bay frames / walls", pitch=[0.060, 0.080]),
                   layup=None, lands=["ST-FS1110", "ST-FS1330", "M-TURRETWALL"],
                   notes="flush skin insert, CFRP solid laminate 2.0 mm; opening radius = HD59 ball radius + "
                         "payload.turret.bay.aperture_ring.radial_clearance; interchangeable E180 ring YK250-SH-364 "
                         "(0.19 m opening); sliding bay doors close behind it"))
    X.append(panel("P-PARA-SURR", "YK250-SH-366", "upper skin around the parachute hatch", "paraşüt kapağı çevre kaplaması",
                   "body_upper", [X_PFF, X_PFR], [-0.40, 0.40], "fixed", NUT, lands=["ST-FS1490", "ST-FS1810",
                                                                                     "M-PARAWALL", "M-CHINE"]))
    X.append(panel("P-PARAHATCH", "YK250-SH-367", "parachute hatch (dorsal)", "paraşüt kapağı", "body_upper",
                   [PARA_X[0], PARA_X[1]], [-0.150, 0.150], "hinged", {"type": "hinge+latch", "spec": "2 x piano-hinge "
                   "segments (stainless pin d 2.5) at the forward edge on the FS1490 land; pin-puller latch at the aft "
                   "edge; tether", "pitch": None, "edge_margin": 0.012}, joint="para_hatch",
                   hinge={"axis_point": [PARA_X[0], 0.0, round(z_top(PARA_X[0]) - 0.002, 4)], "axis": [0.0, -1.0, 0.0],
                          "range_deg": [0, 110]}))
    X.append(panel("P-PARA-LOWER", "YK250-SH-368", "lower skin parachute bay", "paraşüt bölmesi alt kaplaması",
                   "body_lower", [X_PFF, X_PFR], [-0.40, 0.40], "fixed", NUT, lands=["ST-FS1490", "ST-FS1810"]))
    X.append(panel("P-MB-UPPER", "YK250-SH-370", "upper skin mission bay", "görev bölmesi üst kaplaması", "body_upper",
                   [X_PFR, X_FUELF], [-0.40, 0.40], "fixed", NUT, lands=["ST-FS1810", "ST-FS-FUEL", "M-CHINE"],
                   notes="GNSS 2 window cut-out; start of the dorsal spine channel"))
    X.append(panel("P-GNSS2", "YK250-SH-372", "GNSS 2 RF window", "GNSS 2 RF penceresi", "body_upper",
                   [2.090, 2.170], [0.060, 0.140], "removable", INS, material=G, rf_window=True,
                   lands=["P-MB-UPPER"]))
    X.append(panel("P-MBHATCH", "YK250-SH-373", "mission-bay hatch", "görev bölmesi kapağı", "body_lower",
                   [X_PFR + 0.030, X_FUELF - 0.015], [-0.150, 0.150], "removable", CAM, lands=["ST-FS1810", "ST-FS-FUEL", "M-MIDFLOOR"]))
    X.append(panel("P-MB-LOWER", "YK250-SH-374", "lower skin mission bay", "görev bölmesi alt kaplaması", "body_lower",
                   [X_PFR, X_FUELF], [-0.40, 0.40], "fixed", NUT, lands=["ST-FS1810", "ST-FS-FUEL", "M-CHINE"]))
    X.append(panel("P-SPINE", "YK250-SH-376", "dorsal spine channel + tear-away bridle cover",
                   "sırt kanalı + yırtılır kayış kapağı", "body_upper", [X_PFR, X_RS0 + 0.016], [-0.030, 0.030], "fixed",
                   {"type": "nutplate+screw / tear-away", "spec": "channel: M4 nutplates on FS1810/FS-FUEL/main-spar/"
                    "rear-spar frame tops; cover strip: hook-and-loop + 4 nylon M3 shear screws (tear-away under "
                    "the bridle pull)", "pitch": [0.030, 0.150], "edge_margin": 0.010},
                   notes="bridle aft leg stowed in the channel from the parachute bay to the aft fitting"))
    cells = [FC[n]["x"] for n in ("forward_cell", "saddle_cell", "aft_cell")]
    for k, ((x0, x1), nm) in enumerate(zip([(c[0] - 0.0075, c[1] + 0.0075) for c in cells],
                                           ("forward", "saddle", "aft"))):
        x0, x1 = round(float(x0), 4), round(float(x1), 4)
        X.append(panel(f"P-FUEL{k + 1}", f"YK250-SH-{377 + k}", f"{nm} fuel-bay access panel",
                       f"{'ön' if k == 0 else 'orta' if k == 1 else 'arka'} yakıt bölmesi kapağı", "body_upper",
                       [x0, x1], [0.035, 0.300], "removable", NUT_SEAL, mirror=True,
                       lands=["P-CENTRE-UPPER", "P-SPINE"],
                       notes="semi-permanent (bladder installation/inspection), vapour-tight"))
    X.append(panel("P-CENTRE-UPPER", "YK250-SH-380", "upper centre skin (fuel bays)", "üst orta gövde kaplaması",
                   "body_upper", [X_FUELF, X_GEARF], [-0.40, 0.40], "fixed", NUT,
                   lands=["ST-FS-FUEL", "ST-FS-MS", "ST-FS-RS", "ST-FS-GEAR", "M-CHINE"]))
    X.append(panel("P-PAYHATCH", "YK250-SH-382", "payload-bay hatch (belly)", "faydalı yük bölmesi kapağı (karın)",
                   "body_lower", [float(pay[0][0]) + 0.004, float(pay[1][0]) - 0.004], [-0.222, 0.222], "removable",
                   CAM, lands=["M-KEEL", "ST-FS-FUEL"],
                   notes="payload-specific panels (sensor windows) replace it; land on the keel-beam flanges"))
    X.append(panel("P-CENTRE-LOWER", "YK250-SH-383", "lower centre skin", "alt orta gövde kaplaması", "body_lower",
                   [X_FUELF, X_GEARF], [-0.40, 0.40], "fixed", NUT,
                   lands=["ST-FS-FUEL", "ST-FS-MS", "ST-FS-RS", "ST-FS-GEAR", "M-KEEL", "M-GEARBEAM", "M-CHINE"],
                   notes="cut-outs: payload hatch, main wells (doors), refuel door; centre strip between the wells "
                         "closes the harness channel"))
    X.append(panel("P-REFUEL", "YK250-SH-385", "refuel access door (port)", "yakıt ikmal kapağı (sol)", "body_lower",
                   [X_RS0 - 0.165, X_RS0 - 0.085], [-0.330, -0.270], "hinged", {"type": "hinge+latch", "spec": "flush piano hinge "
                   "(forward edge) + push latch", "pitch": None, "edge_margin": 0.010},
                   notes="OBP dry-break AN6 coupling (components.yaml obp_dry_break_an6) + refuelling bonding point "
                         "(CS-LUAS.867(d)); outboard of the port keel beam, ahead of the main wells"))
    X.append(panel("P-AFTHATCH", "YK250-SH-387", "aft equipment hatch (belly)", "arka teçhizat kapağı (karın)",
                   "body_lower", [X_GEARF + 0.015, 3.470], [-0.120, 0.120], "removable", CAM, lands=["ST-FS-GEAR", "ST-FS3480",
                                                                                         "P-AFT-LOWER"],
                   notes="access: ECU, generator PE, fuel pump/filter (drain), brake actuator, inner-door actuators, "
                         "tail connector"))
    X.append(panel("P-STABACT", "YK250-SH-389", "lower hatch FS3480-firewall (stabilator actuators, pushrods, fuel "
                   "shut-off valve)", "alt kapak FS3480-yangın perdesi (stabilatör eyleyicileri)", "body_lower",
                   [3.495, X_FW - 0.0198 - 0.005], [-0.120, 0.120], "removable", CAM,
                   lands=["ST-FS3480", "ST-FS3670", "M-AFTKEEL", "P-AFT-LOWER"],
                   notes="the DA 30 actuators, their pushrods / fireproof boots and the shut-off valve on the forward "
                         "face of the firewall are reached from below without removing the cowl or any frame"))
    X.append(panel("P-AFT-LOWER", "YK250-SH-388", "lower aft skin", "alt arka kaplama", "body_lower", [X_GEARF, 3.670],
                   [-0.40, 0.40], "fixed", NUT, lands=["ST-FS-GEAR", "ST-FS3480", "ST-FS3670", "M-CHINE"],
                   notes="fuel vent outlet (flush NACA vent) and sump drain fittings"))
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
    X.append(panel("P-STUBROOT", "YK250-SH-393", "stabilator-stub root fairing", "kök parçası fileto kaplaması",
                   "body_side", [3.415, 3.962], [0.240, 0.300], "fairing", INS, mirror=True,
                   notes="removable for stub removal; stub root-rib flange screwed to the body side under it"))
    X.append(panel("P-COWL-UP", "YK250-SH-450", "upper cowl", "üst motor kaportası", "cowl_upper", [3.670, 4.000],
                   [-0.25, 0.25], "removable", dict(CAM, pitch=[0.070, 0.090]),
                   lands=["ST-FS3670", "ST-FS3738", "P-COWL-LO"],
                   notes="split line on the stabilator plane (z = tail.surfaces.stabilator_stub.params.z) with half "
                         "notches around the stub roots; seals to the cooling plenum; aft lip ring half (annular "
                         "cooling exit around the hub spacer)"))
    X.append(panel("P-COWL-LO", "YK250-SH-451", "lower cowl", "alt motor kaportası", "cowl_lower", [3.670, 4.000],
                   [-0.25, 0.25], "removable", dict(CAM, pitch=[0.070, 0.090]),
                   lands=["ST-FS3670", "ST-FS3738", "M-AFTKEEL", "P-COWL-UP"],
                   notes="exhaust exit cut-outs with stainless 304 heat-shield patches (0.5 mm on 5 mm stand-offs); "
                         "ventral-root slot; aft lip ring half"))
    gs = [sec for sec in S["wing"]["sections"] if float(sec["y"]) <= YJ + 1e-6]
    gx = [round(min(float(sec["x_le"]) for sec in gs), 4), round(max(float(sec["x_le"]) + float(sec["chord"])
                                                                      for sec in gs), 4)]
    for side in ("R",):
        X.append(panel("P-GLOVE-UP", "YK250-SH-420", "LERX/glove upper skin", "LERX/eldiven üst kaplaması",
                       "glove_upper", gx, [0.400, 0.700], "fixed", BOND, layup="wing_skin_primary",
                       mirror=True, lands=["M-CHINE", "M-SOB", "M-GLOVERIB", "M-JOINTRIB", "M-CTBOX", "M-LERXRIB1"],
                       notes="primary wing skin (wing_skin_primary 0.6/5/0.4), bonded; LERX leading edge solid "
                             "laminate (impact)"))
        X.append(panel("P-GLOVE-LO", "YK250-SH-421", "LERX/glove lower skin", "LERX/eldiven alt kaplaması",
                       "glove_lower", gx, [0.400, 0.700], "fixed", BOND, layup="wing_skin_primary",
                       mirror=True, lands=["M-CHINE", "M-SOB", "M-GLOVERIB", "M-JOINTRIB", "M-CTBOX"],
                       notes="framed cut-out for the joint access panel"))
        X.append(panel("P-JOINTACCESS", "YK250-SH-422", "wing-joint access panel (lower glove)",
                       "kanat birleşim erişim kapağı (alt eldiven)", "glove_lower",
                       [spar_x(0.455, 0.25) - 0.175, spar_x(0.455, 0.25) - 0.030], [0.440, 0.680], "removable", CAM,
                       layup="shell_secondary", mirror=True, lands=["M-GLOVERIB", "M-JOINTRIB", "M-LERXRIB1",
                                                                    "M-CTBOX"],
                       notes="opened at every wing assembly: main pins + keepers, wing connector; the cut-out in the "
                             "primary glove skin is framed (doubler land on the ribs and the box) and carries the skin "
                             "loads around it; the panel itself is a non-structural cover (layups.shell_secondary, "
                             "0.2 mm thinner than the glove skin: joggled land keeps it flush)"))
    return X
