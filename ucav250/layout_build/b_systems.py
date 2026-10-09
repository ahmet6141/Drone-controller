"""Layout builder: systems (equipment, antennas, air data, lights, actuators, harness trunks)."""
from __future__ import annotations

from b_common import *  # noqa: F401,F403
from b_mech import hinge, unit

RULE_EDITS = {   # layout.rules edits (deck sandwich between the side bays and the PDU / power-switching zones)
    ("boxes", "power_switching_bay", "z"): [0.021, 0.081],
    ("boxes", "avionics_power_deck", "z"): [0.021, 0.082],
    ("boxes", "avionics_side_bays", "z"): [-0.089, 0.0135],
    ("boxes", "avionics_side_bays", "content"): "pair of side bays beside the nose-gear keel slot: port = generator power "
                                                "electronics + brake actuator / master cylinder (layout phase: CG); "
                                                "starboard = autopilot (GNSS/IMU inside), C2/video datalink, backup C2 "
                                                "link, transponder, Remote ID",
}
NEW_BOXES = {"forward_bay": {"x": [0.3855, 0.5965], "half_width": 0.0415, "z": [-0.068, 0.035], "clearance": 0.008,
                             "content": "forward equipment bay FS0300-FS0600, upper (layout phase: CG): 12S2P Li-ion "
                                        "buffer battery in its vented, cell-fused box"},
             "forward_bay_lower": {"x": [0.490, 0.596], "half_width": 0.033, "z": [-0.113, -0.077], "clearance": 0.008,
                                   "content": "forward equipment bay FS0300-FS0600, lower: independent FTS unit"}}
NEW_CONTENT_Z = {"contactor_fuses": 0.051, "dcdc_28_12": 0.0385, "pdu": 0.0515}
MOVED_CONTENT = {"buffer_battery": {"zone": "forward_bay", "side": "centre", "center": [0.4810, 0.0, -0.0165],
                                    "connector_face": "+x"}}
ADDED_CONTENT = [
    {"name": "fts_unit", "zone": "forward_bay_lower", "side": "centre", "dims": [0.08, 0.06, 0.03],
     "center": [0.550, 0.0, -0.095], "connector_allowance": 0.015, "connector_face": "-x", "mass_kg": 0.15,
     "source": "components.yaml recovery_and_safety.flight_termination[independent_fts_channel] 0.15 kg; allocation box "
               "80 x 60 x 30 mm (estimate, no unit selected)"},
    {"name": "generator_pe", "zone": "avionics_side_bays", "side": "port", "dims": [0.12, 0.08, 0.05],
     "center": [0.925, -0.0965, -0.0600], "connector_allowance": 0.02, "connector_face": "+x", "mass_kg": 0.40,
     "source": "engine.installed_items_kg.generator_power_electronics 0.40 kg; allocation box 120 x 80 x 50 mm "
               "(estimate); layout phase: next to the PDU it feeds (28 V DC), 3-phase AC from the SG750 in H-MAIN; "
               "heat-sink plate on the side-bay hatch"},
    {"name": "brake_unit", "zone": "avionics_side_bays", "side": "port", "dims": [0.1028, 0.054, 0.026],
     "center": [0.920, -0.0965, -0.0090], "connector_allowance": 0.030, "connector_face": "+x", "mass_kg": 0.335,
     "source": "components.yaml volz_da26 case 102.8 x 54 x 26 mm, 0.27 kg + master cylinder on its output (in the "
               "30 mm allowance, estimate); layout phase: brake-by-wire master cylinder in the nose, two brake lines "
               "along H-MAIN to the main wheels"}]

BAY_PART = {"contactor_fuses": ("YK250-SY-722", "main contactor + fuse block", "ana kontaktör + sigorta bloğu"),
            "dcdc_28_12": ("YK250-SY-724", "28-to-12 V DC-DC module", "28-12 V DC-DC modülü"),
            "pdu": ("YK250-SY-720", "power distribution unit (VISIONAIRtronics 1000 W)", "güç dağıtım birimi"),
            "buffer_battery": ("YK250-SY-726", "12S2P Li-ion buffer battery in vented, cell-fused box",
                               "12S2P Li-ion tampon batarya (havalandırmalı kutu)"),
            "autopilot": ("YK250-SY-728", "autopilot Veronte 1x (GNSS/IMU, air data)", "otopilot Veronte 1x"),
            "datalink_primary": ("YK250-SY-730", "primary C2/video datalink Silvus SC4200EP",
                                 "birincil veri bağı Silvus SC4200EP"),
            "datalink_backup": ("YK250-SY-732", "backup C2 link Microhard pDDL2450", "yedek C2 bağı Microhard"),
            "transponder": ("YK250-SY-734", "Mode S ES transponder uAvionix ping200X", "transponder ping200X"),
            "remote_id": ("YK250-SY-736", "Remote ID beacon Dronetag", "uzaktan kimlik Dronetag"),
            "fts_unit": ("YK250-SY-738", "independent flight-termination unit", "bağımsız uçuş sonlandırma birimi"),
            "generator_pe": ("YK250-PR-512", "generator power electronics (SG750 rectifier/regulator)",
                             "jeneratör güç elektroniği"),
            "brake_unit": ("YK250-FC-629", "brake actuator Volz DA 26 + master cylinder",
                           "fren eyleyicisi DA 26 + ana silindir")}


def apply_rule_edits(L: dict) -> None:
    for (a, b, c), v in RULE_EDITS.items():
        L["rules"][a][b][c] = v
    for k, v in NEW_BOXES.items():
        L["rules"]["boxes"][k] = copy.deepcopy(v)
    items = L["rules"]["bay_contents"]["items"]
    for it in items:
        if it["name"] in NEW_CONTENT_Z:
            it["center"][2] = NEW_CONTENT_Z[it["name"]]
        if it["name"] in MOVED_CONTENT:
            it.update(copy.deepcopy(MOVED_CONTENT[it["name"]]))
    names = {it["name"] for it in items}
    for it in ADDED_CONTENT:
        if it["name"] not in names:
            items.append(copy.deepcopy(it))


def eq(eid, part, name, name_tr, group, box, mass_kg, mount, source, mirror=False, connector="", extra=None):
    d = {"id": eid, "part": part, "name": name, "name_tr": name_tr, "group": group, "box": r3(box)}
    if mirror:
        d["mirror"] = True
    d["mass_kg"] = mass_kg
    d["mount"] = mount
    if connector:
        d["connector"] = connector
    d["source"] = source
    if extra:
        d.update(extra)
    return d


def equipment(L: dict) -> list:
    E = []
    for it in L["rules"]["bay_contents"]["items"]:
        env = Z.content_envelope(it)
        part, nm, ntr = BAY_PART[it["name"]]
        box = env["body"].tolist()
        if it.get("side") == "port":
            pass                                            # center already carries the port y
        grp = {"generator_pe": "propulsion", "brake_unit": "controls"}.get(it["name"], "systems")
        tray = {"avionics_side_bays": "YK250-CH-118", "forward_bay": "YK250-CH-119", "forward_bay_lower": "YK250-CH-119"}.get(it["zone"], "YK250-CH-022")
        E.append(eq("EQ-" + it["name"].upper(), part, nm, ntr, grp, box, it["mass_kg"],
                    {"zone": it["zone"], "tray": tray},
                    f"layout.rules.bay_contents[{it['name']}] ({it.get('source', '')[:90]})",
                    connector=f"{it.get('connector_face', '+x')} face, {1000 * float(it.get('connector_allowance', 0)):.0f}"
                              " mm allowance", extra={"envelope_with_connector": r3(env["envelope"].tolist())}))
    E.append(eq("EQ-MC", "YK250-PL-850", "mission computer / recorder + video encoder",
                "görev bilgisayarı / kayıtçı + video kodlayıcı", "payload", [[1.860, -0.070, -0.120],
                                                                             [2.040, 0.070, -0.045]], 1.0,
                {"zone": "mission_computer", "tray": "YK250-CH-120 on the mission-bay floor"},
                "baseline.yaml#payload_set mission computer and recorder 1.0 kg; allocation box (estimate)"))
    E.append(eq("EQ-ECU", "YK250-PR-510", "engine ECU (Limbach)", "motor kontrol birimi", "propulsion",
                [[X_FUELF - 0.135, -0.200, -0.128], [X_FUELF - 0.015, -0.120, -0.078]], 0.8,
                {"zone": "mission bay floor, port aft corner (layout phase: CG; out of the engine-bay heat and "
                         "vibration)", "tray": "YK250-CH-120"},
                "engine.installed_items_kg.ecu_harness_ignition_coils 0.8 kg (coils on the engine); allocation box "
                "(estimate); engine lines (injectors, coils, sensors, throttle servo) run with H-MAIN to the aft bay "
                "and through the firewall (C-FW-HARN); ECU harness length / extension to be confirmed with Limbach"))
    E.append(eq("EQ-FUELPUMP", "YK250-FU-584", "EFI fuel pump, regulator, filter / gascolator",
                "yakıt pompası, regülatör, filtre", "fuel", [[X_GEARF + 0.010, 0.030, -0.060], [X_GEARF + 0.110, 0.090, 0.005]], 0.35,
                {"zone": "equipment_bay_aft", "tray": "YK250-CH-121"},
                "engine.installed_items_kg.fuel_pump_regulator_filter 0.35 kg (Limbach EFI supply kit); allocation box "
                "(estimate); filter accessible from the aft equipment hatch (CS-LUAS/STANAG drain + strainer rule)"))
    E.append(eq("EQ-SHUTOFF", "YK250-FU-576", "fuel shut-off valve (forward of the firewall)", "yakıt kesme valfi",
                "fuel", [[3.600, 0.020, 0.000], [3.640, 0.060, 0.040]], 0.08,
                {"zone": "forward face of FS3670", "tray": "YK250-CH-013"},
                "standards.yaml FUEL (CS-VLA 995: no valve on the engine side of the firewall); mass estimate"))
    E.append(eq("EQ-DOORACT", "YK250-FC-676", "main inner-door actuator Volz DA 22 (28 V)",
                "iç kapak eyleyicisi DA 22", "gear", [[X_GEARF + 0.0070, 0.030, -0.170], [X_GEARF + 0.0485, 0.052, -0.1041]],
                0.132, {"zone": "aft face of FS-GEAR", "tray": "YK250-CH-011"},
                "mass.rules.gear_doors.inner_door_actuator (components.yaml volz_da22_28v 41.5 x 65.9 x 22 mm, "
                "0.132 kg); crank + pushrod (C-DOORLINK) + over-centre lock to the door hinge horn", mirror=True))
    E.append(eq("EQ-STABACT", "YK250-FC-302", "stabilator actuator Volz DA 30", "stabilatör eyleyicisi DA 30",
                "controls", [[3.5650, 0.0060, 0.2025], [3.6500, 0.1645, 0.2325]], 0.63,
                {"zone": "forward face of the firewall (cool side)", "tray": "bracket on YK250-CH-013"},
                "tail.surfaces.stabilator.controls.actuator (components.yaml volz_da30 158.5 x 85 x 30 mm with the "
                "D-Sub connector); installed lying along y, output shaft (case height axis) at the outboard end "
                "(orientation inferred from the 77 x 18 mm lug pattern - confirm with the Volz drawing)", mirror=True,
                extra={"linkage": {"servo_axis": r3([3.6375, 0.1645, float(STAB["pivot"][2])]),
                                   "horn_axis": r3([STAB["pivot"][0], 0.1645, STAB["pivot"][2]]),
                                   "servo_arm_m": STAB["controls"]["linkage"]["servo_arm_m"],
                                   "horn_m": float(STAB["controls"]["linkage"]["servo_arm_m"]) *
                                   float(STAB["controls"]["linkage_ratio"]),
                                   "pushrod_base_m": STAB["controls"]["linkage"]["pushrod_base_m"],
                                   "path": "pushrod in the plane y = +-0.1645 through the firewall (fireproof "
                                           "bellows boot, cut-out C-FW-PUSHROD) to the spindle horn"}}))
    E.append(eq("EQ-TDOORACT", "YK250-PL-830", "turret bay door actuator Volz DA 22 (28 V) + rack drive",
                "taret kapağı eyleyicisi DA 22", "payload",
                [[X_TUR - 0.0659 / 2, 0.1180, -0.0400], [X_TUR + 0.0659 / 2, 0.1180 + 0.0220, 0.0015]], 0.132,
                {"zone": "outboard face of the turret-bay wall (above the door rail)", "tray": "YK250-CH-023"},
                "components.yaml volz_da22_28v 41.5 x 65.9 x 22 mm, 0.132 kg (within mass.rules.turret_mechanism_kg, "
                "estimate); pinion + rack on the sliding door; doors are opened before the elevator moves (sequence "
                "turret_extension)", mirror=True))
    xt, zr = X_TUR, float(TU["ball_center_retracted_z"])
    g = TU["growth_envelope"]
    E.append(eq("EQ-TURRET", "YK250-PL-820", "EO/IR turret Trillium HD59 (+ mount)", "EO/IR taret Trillium HD59",
                "payload", [[xt - 0.0735, -0.0735, zr - 0.0735], [xt + 0.0735, 0.0735, zr - 0.0735 + 0.202]], 1.75,
                {"zone": "turret_bay", "tray": "YK250-PL-822 carriage on the elevator"},
                "payload.turret (HD59 ball 0.147 m, height 0.202 m, 1.55 + 0.20 kg); E180 growth envelope "
                f"{g['diameter']} x {g['height']} m in the same bay", extra={"moves_with": "turret_elevator"}))
    E.append(eq("EQ-ELEVATOR", "YK250-PL-824", "turret elevator (ball screw, BLDC + brake, 2 rails, controller)",
                "taret asansörü (bilyalı vida, BLDC, 2 ray)", "systems",
                [[xt - 0.095, -0.095, float(ZP["turret_bay"]["box"][1][2]) - 0.070],
                 [xt + 0.095, 0.095, float(ZP["turret_bay"]["box"][1][2])]], 1.66,
                {"zone": "turret_bay (mechanism height payload.turret.bay.mechanism_height)", "tray": "YK250-CH-024"},
                "mass.rules.turret_mechanism_kg (estimate, no catalogue unit): rails in two diagonal bay corners "
                "(front-starboard, aft-port), ball screw in the front-port corner, motor on the bay roof; doors "
                "driven from the carriage (layout.mechanisms turret_door_*)"))
    pb = ZP["parachute_bay"]["box"]
    E.append(eq("EQ-PARACHUTE", "YK250-SY-800", "parachute system UAVOS 200 (container)", "paraşüt sistemi UAVOS 200",
                "systems", [[pb[0][0], -pb[1][1], pb[0][2]], [pb[1][0], pb[1][1], pb[1][2]]], 4.5,
                {"zone": "parachute_bay", "tray": "YK250-CH-026 floor + YK250-CH-114 restraint brackets"},
                "components.yaml uavos_200_parachute_system: compartment 0.300 x 0.300 x 0.275 m, 4.5 kg, max load "
                "220 kg, deploy <= 97 m/s, max load factor 5 g"))
    E.append(eq("EQ-PARALATCH", "YK250-SY-804", "parachute hatch latch (pin puller)", "paraşüt kapağı mandalı",
                "systems", [[PARA_X[1] - 0.045, -0.020, 0.150], [PARA_X[1] - 0.015, 0.020, 0.180]], 0.05,
                {"zone": "aft edge of the parachute hatch", "tray": "YK250-CH-006"},
                "estimate (pin-puller class, fired by the parachute controller / FTS)"))
    return E


def antennas() -> list:
    A = []
    F = af.tail["fin"]

    def ant(aid, part, name, name_tr, point, size, window, link, notes=""):
        d = {"id": aid, "part": part, "name": name, "name_tr": name_tr, "point": r3(point), "size": r3(size),
             "window": window, "link": link}
        if notes:
            d["notes"] = notes
        return d
    A.append(ant("ANT-GNSS1", "YK250-SY-780", "GNSS antenna 1 Calian HC977EXF (embedded helical)",
                 "GNSS anteni 1", [0.970, 0.0, z_top(0.970) - 0.0058 - 0.005 - 0.025], [0.0387, 0.0387, 0.0497],
                 "YK250-SH-354", "autopilot GNSS 1 (SSMA)", "under the GFRP avionics hatch; no ground plane needed "
                 "(components.yaml calian_hc977exf)"))
    A.append(ant("ANT-GNSS2", "YK250-SY-781", "GNSS antenna 2 Calian HC977EXF", "GNSS anteni 2",
                 [2.130, 0.100, z_top(2.130, 0.100) - 0.0058 - 0.005 - 0.025], [0.0387, 0.0387, 0.0497],
                 "YK250-SH-372", "autopilot GNSS 2 (dual-antenna heading, baseline 1.17 m)"))
    A.append(ant("ANT-C2A", "YK250-SY-782", "primary datalink antenna A (2.4 GHz omni dipole, SWA 1001-202)",
                 "birincil veri bağı anteni A", [0.240, 0.0, -0.030], [0.013, 0.013, 0.109], "YK250-SH-350",
                 "Silvus SC4200EP MIMO chain A", "vertical inside the GFRP nose cone (forward/lower coverage)"))
    pB = surf_point(F, 0.95, 0.35)
    A.append(ant("ANT-C2B", "YK250-SY-783", "primary datalink antenna B (2.4 GHz omni dipole, SWA 1001-202)",
                 "birincil veri bağı anteni B", pB * np.array([1, -1, 1]), [0.013, 0.013, 0.109], "YK250-TL-258-L",
                 "Silvus SC4200EP MIMO chain B", "along the span inside the GFRP port fin-tip cap (upper/lateral "
                                                  "coverage; research note: spaced fin/belly pair avoids shadowing)"))
    A.append(ant("ANT-BKP", "YK250-SY-784", "backup C2 antenna (2.4 GHz omni dipole)", "yedek C2 anteni",
                 pB, [0.013, 0.013, 0.109], "YK250-TL-258-R", "Microhard pDDL2450 main port",
                 "inside the GFRP starboard fin-tip cap; diversity port: small dipole in the nose cone ≥ 1 wavelength "
                 "from antenna A. Co-site with the 2.4 GHz Silvus band: frequency plan or the 900 MHz pDDL900 "
                 "alternative (components.yaml) - open item"))
    A.append(ant("ANT-XPDR", "YK250-SY-786", "transponder L-band blade antenna (1090 MHz)", "transponder anteni",
                 [0.450, 0.0, z_bot(0.450) - 0.035], [0.060, 0.008, 0.070], "external (belly, ahead of the nose gear)",
                 "uAvionix ping200X", "external blade on a bonded copper-mesh ground-plane doubler: a monopole "
                 "needs a ground plane and omni low-elevation coverage, which an internal antenna behind a small RF "
                 "window in a CFRP body would not give; in the misc protuberance drag item; above the turret -5 deg "
                 "cone (KO-TURRET-FOV)"))
    A.append(ant("ANT-RID", "YK250-SY-736", "Remote ID beacon internal antennas (BT 2.4 GHz)",
                 "uzaktan kimlik anteni (iç)", [1.0675, 0.100, -0.026], [0.037, 0.026, 0.016], "YK250-SH-358",
                 "Dronetag Beacon", "integral antenna of EQ-REMOTE_ID (same part); radiates through the GFRP starboard "
                                    "side-bay panel; the beacon's internal GNSS "
                                    "has a poor sky view there: position feed from the autopilot or the Veronte RID "
                                    "variant (components.yaml) - open item"))
    A[-1]["integral_to"] = "EQ-REMOTE_ID"
    A.append(ant("ANT-FTS", "YK250-SY-787", "FTS command antenna", "uçuş sonlandırma anteni", [0.150, 0.0, -0.035],
                 [0.010, 0.010, 0.060], "YK250-SH-350", "independent FTS receiver", "in the GFRP nose cone; band per "
                                                                                     "the selected FTS (open item)"))
    return A


def air_data_lights() -> list:
    out = []
    out.append({"id": "AD-PITOT-NOSE", "part": "YK250-SY-788", "name": "heated pitot-static probe (Aeroprobe)",
                "name_tr": "ısıtmalı pito-statik sonda (burun)", "p0": [-0.120, 0.0, -0.050], "p1": [0.020, 0.0, -0.050],
                "diameter": 0.010, "mount": "nose-cone tip boss (GFRP nose cone, M4 inserts), pneumatic lines to the "
                                            "autopilot ports (bulkhead unions in FS0300)",
                "source": "components.yaml aeroprobe_heated_pitot_static (28 V / 42 W heater); recommended set"})
    yp = -2.000
    sec = af.wing.interpolate_section(abs(yp))
    zl = float(wing_pt(abs(yp), 0.15, 0.0)[2])
    out.append({"id": "AD-PITOT-WING", "part": "YK250-SY-789", "name": "pitot HASA (DroneCAN, dissimilar airspeed)",
                "name_tr": "pito HASA (kanat altı)", "p0": r3([float(sec["x_le"]) - 0.100, yp, zl - 0.035]),
                "p1": r3([float(sec["x_le"]) + 0.063, yp, zl - 0.035]), "diameter": 0.010,
                "mount": "mast on the port outer-panel lower skin at 15 % chord (wing module), DroneCAN through the "
                         "wing-joint connector",
                "source": "components.yaml powerbox_pitot_hasa (0.010 x 0.163 m, 0.048 kg)"})
    b2 = 0.5 * float(S["wing"]["span"])
    tip = wing_pt(b2 - 0.02, 0.10, 0.5)
    out.append({"id": "LT-WING", "part": "YK250-SY-790", "name": "position/strobe light (green starboard, red port; "
                "AveoFlashLP LSA 3-in-1)", "name_tr": "seyir/çakar ışığı", "mirror": True,
                "box": r3([[tip[0] - 0.030, b2 - 0.060, tip[2] - 0.015], [tip[0] + 0.070, b2 + 0.004, tip[2] + 0.015]]),
                "mount": "wing-tip fairing (wing module), M5 screws (components.yaml mounting)",
                "source": "components.yaml aveoflash_lp_lsa 0.1 x 0.045 x 0.03 m, 0.083 kg"})
    pt = surf_point(af.tail["fin"], 0.97, 0.98)
    out.append({"id": "LT-TAIL", "part": "YK250-SY-791", "name": "tail position/strobe light (white, aft-facing)",
                "name_tr": "kuyruk seyir ışığı", "box": r3([[pt[0] - 0.050, pt[1] - 0.015, pt[2] - 0.020],
                                                         [pt[0] + 0.050, pt[1] + 0.015, pt[2] + 0.020]]),
                "mount": "trailing edge of the starboard fin-tip cap (GFRP), 2 x M5",
                "source": "components.yaml aveoflash_lp_lsa"})
    return out


def _fit_z(xs, ys, h, margin=0.004):
    """Centre z of a case of height h fitting in the outer-wing section over the x/y corners (margin each side)."""
    lo, hi = -1.0, 1.0
    for y in ys:
        for x in xs:
            sec = af.wing.interpolate_section(y)
            xc = (x - float(sec["x_le"])) / float(sec["chord"])
            lo = max(lo, float(wing_pt(y, xc, 0.0)[2]) + margin)
            hi = min(hi, float(wing_pt(y, xc, 1.0)[2]) - margin)
    if hi - lo < h:
        raise ValueError(f"actuator case {h} m does not fit (section {hi - lo:.4f} m)")
    return 0.5 * (lo + hi)


def wing_actuators() -> list:
    """Actuator installation interfaces in the outer panels and fins (starboard; port mirrored)."""
    W = S["wing"]["controls"]
    out = []
    for key, part, dims, y0, model, back in (("aileron", "YK250-FC-204", (0.054, 0.1028, 0.026), 2.300, "Volz DA 26",
                                              0.047),
                                             ("flap", "YK250-FC-206", (0.085, 0.1585, 0.030), 0.800, "Volz DA 30",
                                              0.075)):
        c = W[key]
        lk = c["linkage"]
        yh = y0 + 0.5 * dims[1]
        hp = wing_pt(yh, float(c["xc_hinge"]), 0.5)
        sx = float(hp[0]) - float(lk["pushrod_base_m"])
        x0, x1 = sx - back, sx - back + dims[0]
        zc_ = _fit_z((x0, x1), (y0, y0 + dims[1]), dims[2])
        box = [[x0, y0, zc_ - dims[2] / 2], [x1, y0 + dims[1], zc_ + dims[2] / 2]]
        out.append({"id": f"ACT-{key.upper()}", "part": part, "name": f"{key} actuator {model}", "mirror": True,
                    "box": r3(box), "servo_axis": r3([sx, y0 + dims[1], zc_]),
                    "hinge_point": r3(hp), "linkage": lk,
                    "install": "case lying spanwise, output shaft (case height axis) at the outboard end near the aft "
                               "edge of the case (case extends forward into the thicker section), mounted on a "
                               "lower-skin servo hatch frame between the spars; arm and horn in the x-z plane (planar "
                               "four-bar of wing.controls, R-57..R-59); pushrod through a reinforced slot in the "
                               "rear-spar web to the horn in the control-surface cove"})
    rc = FIN["controls"]["rudder"]
    F = af.tail["fin"]
    eta = 0.42
    hp = surf_point(F, eta, float(rc["xc_hinge"]))
    xs_ = float(rc["xc_hinge"]) - float(rc["linkage"]["pushrod_base_m"]) / F.chord_at(eta)
    sp = surf_point(F, eta, xs_)
    o, cdir, u, n_sp = F.frame_at(eta)
    span = -n_sp if n_sp[2] < 0 else n_sp
    cdir = cdir / np.linalg.norm(cdir)
    axes = np.column_stack([cdir, np.cross(span, cdir), span])
    centre = sp - 0.020 * cdir - 0.0514 * span
    out.append({"id": "ACT-RUDDER", "part": "YK250-FC-303", "name": "rudder actuator Volz DA 26", "mirror": True,
                "servo_axis": r3(sp), "hinge_point": r3(hp), "linkage": rc["linkage"],
                "obb": {"center": r3(centre), "axes": r3(axes.T.tolist()), "half": [0.027, 0.013, 0.0514]},
                "box": r3([np.min([centre + sa * 0.027 * axes[:, 0] + sb * 0.013 * axes[:, 1] + sc * 0.0514 * axes[:, 2]
                                   for sa in (-1, 1) for sb in (-1, 1) for sc in (-1, 1)], axis=0).tolist(),
                           np.max([centre + sa * 0.027 * axes[:, 0] + sb * 0.013 * axes[:, 1] + sc * 0.0514 * axes[:, 2]
                                   for sa in (-1, 1) for sb in (-1, 1) for sc in (-1, 1)], axis=0).tolist()]),
                "install": "case along the fin span in the fin box (extended-travel option; oriented box 'obb': axes "
                           "rows = chord, thickness, span), servo hatch on the inboard fin face; planar four-bar in "
                           "the fin chord plane"})
    return out


def harness() -> dict:
    def P(*pts):
        return [r3(p) for p in pts]
    trunks = [
        {"id": "H-MAIN", "part": "YK250-SY-750", "name": "main power/data trunk", "diameter": 0.020,
         "path": P([0.950, -0.110, 0.030], [1.110, -0.160, 0.060], [1.330, -0.160, 0.060], [X_PFF, -0.210, 0.050],
                   [X_PFR, -0.210, 0.050], [2.000, -0.270, -0.090], [X_FUELF, -0.270, -0.090],
                   [X_MS0 + 0.040, -0.280, -0.090], [X_DUCT, -0.290, -0.090], [X_DUCT, -0.150, -0.080],
                   [X_DUCT, 0.0, -0.080], [X_GEARF + 0.020, 0.0, -0.080], [X_GEARF + 0.180, 0.0, 0.030]),
         "penetrations": [{"member": "M-WELLWALL-FWD", "point": r3([X_W0 - 0.0008, 0.0, -0.080]),
                           "kind": "sealed conduit entry (centre-line channel)"}],
         "text": "PDU (+ generator AC lines from the SG750, ECU-engine lines from the mission bay, two brake lines) -> "
                 "port side of the turret and parachute bays -> mission bay -> outboard of the port keel beam under "
                 "the forward fuel deck -> lateral duct between the payload-bay aft wall and the main-well forward "
                 "wall (both 1.6 mm solid laminate) -> centre-line channel between the main wells (sealed CFRP conduit "
                 "through the well forward wall; the stowed tyre envelopes incl. their 12 mm clearance are 37.9 mm "
                 "apart) -> through FS-GEAR (C-HARN-CL) -> aft equipment bay; no route inside a fuel cell"},
        {"id": "H-COAX", "part": "YK250-SY-751", "name": "coax / data trunk (starboard)", "diameter": 0.016,
         "path": P([1.000, 0.150, -0.010], [1.100, 0.160, -0.005], [1.200, 0.160, 0.060], [1.330, 0.160, 0.060],
                   [X_PFF, 0.210, 0.050], [X_PFR, 0.210, 0.050], [2.000, 0.270, -0.090], [X_FUELF, 0.270, -0.090],
                   [X_DUCT, 0.290, -0.105], [X_DUCT, 0.150, -0.103], [X_DUCT, 0.0, -0.103],
                   [X_GEARF + 0.020, 0.0, -0.103], [X_GEARF + 0.100, 0.0, -0.050]),
         "penetrations": [{"member": "M-WELLWALL-FWD", "point": r3([X_W0 - 0.0008, 0.0, -0.103]),
                           "kind": "sealed conduit entry (centre-line channel, lower tier)"}],
         "text": "avionics side bay -> under the avionics deck to FS1110 -> starboard side of the turret and parachute "
                 "bays -> mirror of the main trunk -> shares the lateral duct and the centre-line channel (lower tier, "
                 "shielded coax) -> aft equipment bay -> fin-tip antennas (C2-B, backup) via the fin branches"},
        {"id": "H-WING", "part": "YK250-SY-752", "name": "wing branch (each side)", "diameter": 0.012, "mirror": True,
         "path": P([X_FUELF - 0.040, 0.270, -0.090], [X_FUELF - 0.040, 0.372, -0.050],
                   [X_FUELF - 0.012, 0.386, -0.030], [X_FUELF + 0.030, 0.410, -0.020], [2.400, 0.450, 0.000],
                   [spar_x(0.55, 0.25) - 0.070, 0.540, 0.000]),
         "penetrations": [{"member": "M-SOB", "point": r3([X_FUELF + 0.0125, Y_SOB, -0.0242]),
                           "kind": "grommet in the side-of-body rib"},
                          {"member": "M-LERXRIB1", "point": r3([2.400 + (0.47 - 0.45) / 0.09 * (spar_x(0.55, 0.25)
                                                                 - 0.070 - 2.400), 0.470, 0.000]),
                           "kind": "grommet in LERX rib 1"},
                          {"member": "M-LERXRIB2", "point": r3([2.400 + (0.53 - 0.45) / 0.09 * (spar_x(0.55, 0.25)
                                                                 - 0.070 - 2.400), 0.530, 0.000]),
                           "kind": "grommet in LERX rib 2"}],
         "text": "from the trunk in the mission bay up the body side to the chine, through FS-FUEL just below the chine "
                 "(C-HARN-WING, sealed grommet; it passes the 24 mm gap between FS-FUEL and the forward bladder, "
                 "outside the cell) and the side-of-body rib into the LERX bay, to the MIL-DTL-38999 receptacle on "
                 "the glove rib (joint access bay)"},
        {"id": "H-TAIL", "part": "YK250-SY-753", "name": "tail branch (each side)", "diameter": 0.016, "mirror": True,
         "path": P([X_GEARF + 0.200, 0.040, 0.040], [3.450, 0.100, 0.180], [3.520, 0.120, round(z_top(3.520, 0.120)
                                                                                                   - 0.025, 4)]),
         "text": "aft equipment bay -> FS3480 ring -> fin root (rudder actuator, fin-tip antenna coax, tail light); "
                 "stabilator actuator leads on the firewall forward face"},
        {"id": "H-ENGINE", "part": "YK250-SY-754", "name": "engine harness", "diameter": 0.020,
         "path": P([3.280, -0.050, 0.010], [3.600, -0.095, 0.040], [3.700, -0.095, 0.040]),
         "text": "ECU / generator PE -> firewall fireproof grommets (C-FW-HARN) -> injectors, coils, sensors, SG750 "
                 "(engine side in fire sleeve, >= 50 mm from the exhaust)"},
        {"id": "H-NOSE", "part": "YK250-SY-755", "name": "nose branch", "diameter": 0.012,
         "path": P([0.795, 0.070, 0.050], [0.600, 0.070, 0.045], [0.390, 0.055, 0.035], [0.320, 0.032, -0.010],
                   [0.300, 0.032, -0.018]),
         "text": "FTS, nose-cone antennas (coax), transponder blade feed, pitot heater; pitot lines alongside"}]
    return {"trunks": trunks,
            "rules": "trunks are keep-out corridors (diameter + 10 mm); grommets at every frame (cut-outs in "
                     "layout.stations); power and RF coax separated by >= 20 mm except in the shared centre-line "
                     "channel (shielded coax); >= 50 mm from the exhaust; no harness inside a fuel bay; wing and tail "
                     "branches separable at MIL-DTL-38999 connectors in the access bays",
            "connectors": [{"id": "CN-WING", "location": "glove rib y +-0.55, joint access bay", "type":
                            "MIL-DTL-38999 series III, 19-pin + 2 coax inserts (estimate)"},
                           {"id": "CN-TAIL", "location": "FS3480 ring (aft equipment bay side)", "type":
                            "MIL-DTL-38999 series III"},
                           {"id": "CN-ENGINE", "location": "forward face of the firewall", "type":
                            "fire-resistant circular connector (engine side) / grommet"}]}
