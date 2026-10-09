"""Layout builder: systems (equipment, antennas, air data, lights, actuators, harness trunks)."""
from __future__ import annotations

from .b_common import *  # noqa: F401,F403
from .b_mech import hinge, unit

RULE_EDITS = {   # layout.rules edits (deck sandwich between the side bays and the PDU / power-switching zones)
    ("boxes", "power_switching_bay", "z"): [0.021, 0.081],
    ("boxes", "avionics_power_deck", "z"): [0.021, 0.082],
    ("boxes", "avionics_side_bays", "z"): [-0.089, 0.0135],
    ("boxes", "equipment_bay_aft", "content"): "EFI fuel pump / regulator / filter (drain), inner-door actuators on the "
                                               "aft face of FS-GEAR, tail connector (layout phase: the ECU moved to "
                                               "the mission bay and the generator power electronics to the port side "
                                               "bay, for the CG)",
    ("boxes", "avionics_side_bays", "content"): "pair of side bays beside the nose-gear keel slot: port = generator power "
                                                "electronics + brake actuator / master cylinder (layout phase: CG); "
                                                "starboard = autopilot (GNSS/IMU inside), C2/video datalink, backup C2 "
                                                "link, transponder (fix round 1, VPK-10: the Remote ID beacon moved to "
                                                "the GFRP nose cone)",
}
NEW_BOXES = {"forward_bay": {"x": [0.3855, 0.5965], "half_width": 0.0415, "z": [-0.068, 0.035], "clearance": 0.008,
                             "content": "forward equipment bay FS0300-FS0600, upper (layout phase: CG): 12S2P Li-ion "
                                        "buffer battery in its vented, cell-fused box"},
             "forward_bay_lower": {"x": [0.490, 0.596], "half_width": 0.033, "z": [-0.113, -0.077], "clearance": 0.008,
                                   "content": "forward equipment bay FS0300-FS0600, lower: independent FTS unit"},
             "nose_cone_bay": {"x": [0.2530, 0.2966], "y_inner": 0.014, "half_width": 0.046, "z": [0.008, 0.031],
                               "clearance": 0.006,
                               "content": "inside the GFRP nose cone on the forward face of FS0300 (port): Remote ID "
                                          "beacon (fix round 1, VPK-10: unobstructed RF path through the radome, away "
                                          "from the datalink radio and the carbon deck)"}}
NEW_CONTENT_Z = {"contactor_fuses": 0.051, "dcdc_28_12": 0.0385, "pdu": 0.0515}
MOVED_CONTENT = {"buffer_battery": {"zone": "forward_bay", "side": "centre", "center": [0.4810, 0.0, -0.0165],
                                    "connector_face": "+x"},
                 "remote_id": {"zone": "nose_cone_bay", "side": "port", "center": [0.2751, -0.0300, 0.0200],
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
            "remote_id": ("YK250-SY-736", "Remote ID beacon Dronetag (nose cone)", "uzaktan kimlik Dronetag (burun "
                                                                                   "konisi)"),
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
        tray = {"avionics_side_bays": "YK250-CH-118" if it.get("side") == "port" else "YK250-CH-122",
                "forward_bay": "YK250-CH-119", "forward_bay_lower": "YK250-CH-119",
                "nose_cone_bay": "YK250-CH-002"}.get(it["zone"], "YK250-CH-022")
        E.append(eq("EQ-" + it["name"].upper(), part, nm, ntr, grp, box, it["mass_kg"],
                    {"zone": it["zone"], "tray": tray},
                    f"layout.rules.bay_contents[{it['name']}] ({it.get('source', '')[:90]})",
                    connector=f"{it.get('connector_face', '+x')} face, {1000 * float(it.get('connector_allowance', 0)):.0f}"
                              " mm allowance", extra={"envelope_with_connector": r3(env["envelope"].tolist())}))
    E.append(eq("EQ-MC", "YK250-PL-850", "mission computer / recorder + video encoder",
                "görev bilgisayarı / kayıtçı + video kodlayıcı", "payload", [[1.860, -0.070, -0.120],
                                                                             [2.040, 0.070, -0.045]], 1.0,
                {"zone": "mission_computer", "tray": "YK250-CH-120 removable tray in the floor cut-out (lowered out "
                                                     "through P-MBHATCH, fix round 1 VPK-03)"},
                "baseline.yaml#payload_set mission computer and recorder 1.0 kg; allocation box (estimate)"))
    E.append(eq("EQ-ECU", "YK250-PR-510", "engine ECU (Limbach)", "motor kontrol birimi", "propulsion",
                [[X_FUELF - 0.1555, -0.110, -0.128], [X_FUELF - 0.0355, -0.030, -0.078]], 0.8,
                {"zone": "mission bay, on the removable equipment tray (fix round 1, VPK-03: lowered out through "
                         "P-MBHATCH; out of the engine-bay heat and vibration)", "tray": "YK250-CH-120"},
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
                "0.132 kg); crank + pushrod (C-DOORLINK) + over-centre lock to the inner-door hinge horn, and a second "
                "crank + pushrod to the trunnion door (fix round 2, PK2-03: both doors open before and close after the "
                "leg moves)", mirror=True))
    nd_src = ("fix round 2 (PK2-13): the clamshell doors open before and close after the leg moves (sequence "
              "gear_retraction), which a leg-driven link cannot do; one DA 22 per door (components.yaml volz_da22_28v "
              "41.5 x 65.9 x 22 mm, 0.132 kg; mass.rules.gear_doors.inner_door_actuator count 4): shaft along x, crank "
              "+ pushrod in the plane x 1.095 (between the side-bay contents and FS1110) through a bushed hole in the "
              "keel-wall land to the horn at the aft end of the door hinge; over-centre lock in the closed position")
    E.append(eq("EQ-NDOORACT-R", "YK250-LG-678", "nose-door actuator Volz DA 22 (28 V), starboard",
                "burun kapağı eyleyicisi DA 22, sağ", "gear",
                [[1.0370, 0.1345, -0.0740], [1.0370 + 0.0659, 0.1345 + 0.0415, -0.0740 + 0.0220]], 0.132,
                {"zone": "aft outboard corner of the starboard avionics side bay on the forward face of FS1110, outboard "
                         "of EQ-DATALINK_PRIMARY (its removal path through P-SIDEBAY-R stays free)",
                 "tray": "bracket on FS1110 (off the isolated tray YK250-CH-122)"}, nd_src, extra={"mass_item": "gear_doors_wells_locks_sensors"}))
    E.append(eq("EQ-NDOORACT-L", "YK250-LG-679", "nose-door actuator Volz DA 22 (28 V), port",
                "burun kapağı eyleyicisi DA 22, sol", "gear",
                [[1.0370, -0.1200, -0.1100], [1.0370 + 0.0659, -0.1200 + 0.0415, -0.1100 + 0.0220]], 0.132,
                {"zone": "aft lower corner of the port avionics side bay on the forward face of FS1110 (under the "
                         "P-SIDEBAY-L opening)", "tray": "YK250-CH-118"}, nd_src,
                extra={"mass_item": "gear_doors_wells_locks_sensors"}))
    E.append(eq("EQ-STABACT", "YK250-FC-302", "stabilator actuator Volz DA 30", "stabilatör eyleyicisi DA 30",
                "controls", [[3.5650, 0.0215, 0.2025], [3.6500, 0.1800, 0.2325]], 0.63,
                {"zone": "forward face of the firewall (cool side)", "tray": "bracket on YK250-CH-013"},
                "tail.surfaces.stabilator.controls.actuator (components.yaml volz_da30 158.5 x 85 x 30 mm with the "
                "D-Sub connector); installed lying along y, output shaft (case height axis) at the outboard end "
                "(orientation inferred from the 77 x 18 mm lug pattern - confirm with the Volz drawing)", mirror=True,
                extra={"linkage": {"servo_axis": r3([3.6375, 0.1800, float(STAB["pivot"][2])]),
                                   "horn_axis": r3([STAB["pivot"][0], 0.1800, STAB["pivot"][2]]),
                                   "servo_arm_m": STAB["controls"]["linkage"]["servo_arm_m"],
                                   "horn_m": float(STAB["controls"]["linkage"]["servo_arm_m"]) *
                                   float(STAB["controls"]["linkage_ratio"]),
                                   "pushrod_base_m": STAB["controls"]["linkage"]["pushrod_base_m"],
                                   "path": "pushrod in the plane y = +-0.180 through the firewall (fireproof "
                                           "bellows boot, cut-out C-FW-PUSHROD) to the spindle horn"}}))
    E.append(eq("EQ-TDOORACT", "YK250-PL-830", "turret bay door actuator Volz DA 22 (28 V) + rack drive",
                "taret kapağı eyleyicisi DA 22", "payload",
                [[1.3364, 0.1240, -0.1135], [1.3364 + 0.0659, 0.1240 + 0.0220, -0.1135 + 0.0415]], 0.132,
                {"zone": "aft face of FS1330 (fix round 1, VPK-08: out of the band swept by the sliding doors); drive "
                         "shaft through C-TDOOR-SHAFT to the pinion on the door rack; reached through P-TDOORACC",
                 "tray": "YK250-CH-005"},
                "components.yaml volz_da22_28v 41.5 x 65.9 x 22 mm, 0.132 kg datasheet (part of mass.rules."
                "turret_mechanism_kg, door drives 0.304 kg = 2 x DA 22 + pinions, racks and door rails); pinion + rack "
                "on the sliding door; doors are opened before the elevator moves (sequence turret_extension)",
                mirror=True))
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
                 [xt + 0.095, 0.095, float(ZP["turret_bay"]["box"][1][2])]],
                round(float(S["mass"]["rules"]["turret_mechanism_kg"]) - 2 * 0.132, 3),
                {"zone": "turret_bay (mechanism height payload.turret.bay.mechanism_height)", "tray": "YK250-CH-024"},
                "mass.rules.turret_mechanism_kg less the two door actuators EQ-TDOORACT (estimate, no catalogue "
                "unit; includes the two sliding doors, their racks and rails, the bay liner and the HD59 aperture "
                "ring): rails in two diagonal bay corners (front-starboard, aft-port), ball screw in the front-port "
                "corner, motor on the bay roof; the doors are driven by EQ-TDOORACT (layout.mechanisms turret_door_*)"))
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
    for e in E:                       # fix round 2 (PK2-07): the spec mass item each placed object belongs to
        mi = MASS_ITEM_OF.get(e["id"])
        if mi and "mass_item" not in e:
            e["mass_item"] = mi
    return E


MASS_ITEM_OF = {"EQ-ECU": "engine_group_installed", "EQ-FUELPUMP": "engine_group_installed",
                "EQ-GENERATOR_PE": "engine_group_installed", "EQ-FTS_UNIT": "flight_termination_lights",
                "EQ-BRAKE_UNIT": "actuators_nose_steering_brake_2x_DA26", "EQ-STABACT": "actuators_stabilators_2x_DA30",
                "EQ-DOORACT": "gear_doors_wells_locks_sensors", "EQ-TDOORACT": "turret_lift_mechanism_doors",
                "EQ-ELEVATOR": "turret_lift_mechanism_doors", "EQ-PARACHUTE": "parachute_uavos_200",
                "EQ-SHUTOFF": "fuel_system_3_cells"}
ANT_MOUNT = 0.002     # antenna envelope to the INNER skin surface (fix round 2, PK2-01; was 5.8 mm skin + 2 mm on an
                      # approximate superellipse margin)
_CTX = {}


def _lc_ctx():
    """layout_check context on the loaded spec (true distance to the OML, local skin thickness of the shell panels)."""
    if "ctx" not in _CTX:
        from ..analysis import layout_check as LC
        _CTX["LC"], _CTX["ctx"] = LC, LC.Ctx(copy.deepcopy(S))
    return _CTX["LC"], _CTX["ctx"]


def _sink(point, size, z_hi=None, mount=ANT_MOUNT):
    """Highest centre z (<= z_hi, 0.5 mm steps) at which the whole antenna box lies ``mount`` inside the INNER skin
    surface: true distance to the OML >= local skin thickness (layout_check Ctx.depth / Ctx.skin_t) + mount."""
    LC, ctx = _lc_ctx()
    p = np.asarray(point, float)
    h = 0.5 * np.asarray(size, float)
    z0 = float(p[2]) if z_hi is None else float(z_hi)
    for k in range(400):
        z = z0 - 0.0005 * k
        P = LC.OBB.aabb([[p[0] - h[0], p[1] - h[1], z - h[2]], [p[0] + h[0], p[1] + h[1], z + h[2]]]).samples(0.004)
        if np.all(ctx.depth(P) >= ctx.skin_t(P) + mount):
            return r3([p[0], p[1], z])
    raise ValueError(f"antenna at {point} does not fit {mount} m inside the inner skin surface")


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
                 "GNSS anteni 1", _sink([0.970, 0.0, 0.0], [0.0387, 0.0387, 0.0497], z_top(0.970)),
                 [0.0387, 0.0387, 0.0497],
                 "YK250-SH-354", "autopilot GNSS 1 (SSMA)", "under the GFRP avionics hatch; no ground plane needed "
                 "(components.yaml calian_hc977exf)"))
    A.append(ant("ANT-GNSS2", "YK250-SY-781", "GNSS antenna 2 Calian HC977EXF", "GNSS anteni 2",
                 _sink([2.090, 0.150, 0.0], [0.0387, 0.0387, 0.0497], z_top(2.090, 0.150)), [0.0387, 0.0387, 0.0497],
                 "YK250-SH-372", "autopilot GNSS 2 (dual-antenna heading, baseline 1.17 m)"))
    A.append(ant("ANT-C2A", "YK250-SY-782", "primary datalink antenna A (2.4 GHz omni dipole, SWA 1001-202)",
                 "birincil veri bağı anteni A", _sink([0.250, 0.0, 0.0], [0.013, 0.013, 0.109], 0.0),
                 [0.013, 0.013, 0.109], "YK250-SH-350",
                 "Silvus SC4200EP MIMO chain A", "vertical inside the GFRP nose cone (forward/lower coverage); fix round "
                 "2 (PK2-01): x 0.25 and lowered so that the dipole envelope keeps 2 mm to the inner surface of the "
                 "6.0 mm GFRP cone (true distance)"))
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
                 "uzaktan kimlik anteni (iç)", [0.2751, -0.0300, 0.0200], [0.037, 0.026, 0.016], "YK250-SH-350",
                 "Dronetag Beacon", "integral antenna of EQ-REMOTE_ID (same part); fix round 1 (VPK-10): on the forward "
                                    "face of FS0300 inside the GFRP nose cone, unobstructed by the datalink radio and the "
                                    "carbon deck; the beacon's internal GNSS has a partial sky view through the radome: "
                                    "position feed from the autopilot or the Veronte RID variant (components.yaml) - "
                                    "open item"))
    A[-1]["integral_to"] = "EQ-REMOTE_ID"
    A.append(ant("ANT-FTS", "YK250-SY-787", "FTS command antenna", "uçuş sonlandırma anteni",
                 _sink([0.170, 0.0, 0.0], [0.010, 0.010, 0.060], 0.0), [0.010, 0.010, 0.060], "YK250-SH-350", "independent FTS receiver", "in the GFRP nose cone, lowered (fix round 2, PK2-01: "
                                                                     "2 mm to the inner surface of the 6.0 mm GFRP cone, "
                                                                     "true distance); band per the selected FTS (open item)"))
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


def _fit_z_true(x0, x1, y0, y1, h, mount=0.002):
    """Centre z of a case of height h between the x / y bounds with the largest smallest margin to the INNER skin
    surfaces (true distance to the wing loft - local skin thickness - mount; fix round 2, PK2-06); raises when the
    case does not fit."""
    LC, ctx = _lc_ctx()
    sec = af.wing.interpolate_section(0.5 * (y0 + y1))
    best = (-1.0, None)
    for zc_ in np.arange(float(sec["z_le"]) - 0.06, float(sec["z_le"]) + 0.10, 0.0005):
        P = LC.OBB.aabb([[x0, y0, zc_ - h / 2], [x1, y1, zc_ + h / 2]]).samples(0.006)
        d = ctx.depth(P)
        if d.min() <= 0.0:
            continue
        m = float((d - ctx.skin_t(P) - mount).min())
        if m > best[0]:
            best = (m, zc_)
    if best[1] is None or best[0] < 0.0:
        raise ValueError(f"actuator case {h} m does not fit at y {y0:.3f}-{y1:.3f} (margin {best[0]})")
    return round(best[1], 4), best[0]


AILERON_Y0 = 2.077          # inboard end of the aileron case: output arm 25 mm outboard of the aileron root (eta0 x b/2)
AILERON_PUSHROD_BASE = 0.16  # fix round 2 (PK2-06): servo axis 0.16 m ahead of the hinge, where the section is deep enough


def wing_actuators() -> list:
    """Actuator installation interfaces in the outer panels and fins (starboard; port mirrored). Fix round 2 (PK2-06):
    the cases are placed by the true distance to the inner skin surfaces (skin + 2 mm mount allowance); the aileron
    DA 26 (26 mm case) moved inboard to y 2.08-2.18 and forward (pushrod base 0.16 m) where the section is deep
    enough; the output arm is at the outboard end of the case and the horn in the same x-z plane."""
    W = S["wing"]["controls"]
    out = []
    for key, part, dims, y0, model, back in (("aileron", "YK250-FC-204", (0.054, 0.1028, 0.026), AILERON_Y0,
                                              "Volz DA 26", 0.047),
                                             ("flap", "YK250-FC-206", (0.085, 0.1585, 0.030), 0.800, "Volz DA 30",
                                              0.075)):
        c = W[key]
        lk = dict(c["linkage"])
        if key == "aileron":
            lk["pushrod_base_m"] = AILERON_PUSHROD_BASE
        ys = y0 + dims[1] + 0.005                         # arm plane: 5 mm outboard of the output end of the case
        hp = wing_pt(ys, float(c["xc_hinge"]), 0.5)
        sx = float(hp[0]) - float(lk["pushrod_base_m"])
        x0, x1 = sx - back, sx - back + dims[0]
        zc_, m_ = _fit_z_true(x0, x1, y0, y0 + dims[1], dims[2])
        box = [[x0, y0, zc_ - dims[2] / 2], [x1, y0 + dims[1], zc_ + dims[2] / 2]]
        mi = {"aileron": "actuators_ailerons_2x_DA26", "flap": "actuators_flaps_2x_DA30"}[key]
        out.append({"id": f"ACT-{key.upper()}", "part": part, "name": f"{key} actuator {model}", "mirror": True,
                    "box": r3(box), "servo_axis": r3([sx, ys, zc_]),
                    "hinge_point": r3([float(hp[0]), ys, float(hp[2])]), "linkage": lk, "mass_item": mi,
                    "clearance_to_inner_skin_m": r3(m_ + 0.002),
                    "install": "case lying spanwise, output shaft (case height axis) at the outboard end near the aft "
                               "edge of the case (case extends forward into the thicker section), mounted on a "
                               "lower-skin servo hatch frame between the spars; arm and horn in the x-z plane (planar "
                               "four-bar of wing.controls, R-57..R-59); pushrod through a reinforced slot in the "
                               "rear-spar web to the horn in the control-surface cove; fix round 2 (PK2-06): case "
                               ">= 2 mm inside the inner skin surfaces (true distance, value clearance_to_inner_skin_m)"})
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
                "mass_item": "actuators_rudders_2x_DA26", "in_tail": True,
                "servo_axis": r3(sp), "hinge_point": r3(hp), "linkage": rc["linkage"],
                "obb": {"center": r3(centre), "axes": r3(axes.T.tolist()), "half": [0.027, 0.013, 0.0514]},
                "box": r3([np.min([centre + sa * 0.027 * axes[:, 0] + sb * 0.013 * axes[:, 1] + sc * 0.0514 * axes[:, 2]
                                   for sa in (-1, 1) for sb in (-1, 1) for sc in (-1, 1)], axis=0).tolist(),
                           np.max([centre + sa * 0.027 * axes[:, 0] + sb * 0.013 * axes[:, 1] + sc * 0.0514 * axes[:, 2]
                                   for sa in (-1, 1) for sb in (-1, 1) for sc in (-1, 1)], axis=0).tolist()]),
                "install": "case along the fin span in the fin box (extended-travel option; oriented box 'obb': axes "
                           "rows = chord, thickness, span), servo hatch on the inboard fin face; planar four-bar in "
                           "the fin chord plane"})
    out += gear_actuators()
    return out


def gear_actuators() -> list:
    """Fix round 2 (PK2-13): landing-gear actuators as layout envelopes (the gear unit is not selected: allocation
    envelopes, estimates; SAGITTA practice: DA-26-class EMAs for retraction and steering, components.yaml):
    main-gear retraction EMA on the trunnion axis ahead of the forward lug, nose-gear retraction EMA above the pivot
    (crank + over-centre drag link), nose-wheel steering DA 26 on the nose leg (moves with the leg: rest pose = gear
    down, layout_check sweeps it with gear_retraction). The door drives are equipment (EQ-DOORACT, EQ-NDOORACT-R/-L)."""
    x0, y0, z0 = (float(v) for v in MG["trunnion"])
    xn, zn = float(NG["pivot"][0]), float(NG["pivot"][2])
    A = np.asarray(NG["axle_static"], float)
    Pn = np.asarray(NG["pivot"], float)
    u = (A - Pn) / np.linalg.norm(A - Pn)                    # down the nose leg
    nf = np.array([-u[2], 0.0, u[0]]) if u[2] < 0 else np.array([u[2], 0.0, -u[0]])
    if nf[0] > 0:
        nf = -nf                                             # forward-facing normal of the leg (x-z plane)
    r_leg = 0.5 * float(NG["leg_frontal_width"])
    c_st = Pn + 0.35 * (A - Pn) - (r_leg + 0.013 + 0.002) * nf     # aft face: on top of the leg when stowed
    axes = np.vstack([nf, [0.0, 1.0, 0.0], u])
    half = [0.013, 0.027, 0.0514]
    C = np.array([c_st + sa * half[0] * axes[0] + sb * half[1] * axes[1] + sc * half[2] * axes[2]
                  for sa in (-1, 1) for sb in (-1, 1) for sc in (-1, 1)])
    return [
        {"id": "ACT-MLG-EMA", "part": "YK250-LG-630", "name": "main-gear retraction EMA (gear unit, allocation envelope)",
         "mirror": True, "mass_item": "main_gear_legs_wheels_brakes_emas_pair",
         "cylinder": {"center": r3([x0 - 0.082, y0, z0]), "axis": [1.0, 0.0, 0.0], "radius": 0.030,
                      "half_length": 0.020},
         "box": r3([[x0 - 0.102, y0 - 0.030, z0 - 0.030], [x0 - 0.062, y0 + 0.030, z0 + 0.030]]),
         "drive": "output spline on the forward stub axle of the leg yoke (trunnion axis), flange on the front face of "
                  "the forward bearing lug of F-TRUNNION; torque requirement structures G-EMA-MAIN",
         "install": "allocation envelope d 60 x 40 mm (estimate: DA-26-class BLDC EMA + planetary stage, SAGITTA "
                    "practice; mass in the gear item), between the well forward wall and the forward lug"},
        {"id": "ACT-NLG-EMA", "part": "YK250-LG-632", "name": "nose-gear retraction EMA (gear unit, allocation envelope)",
         "mass_item": "nose_gear_leg_wheel_steering",
         "box": r3([[xn - 0.025, -0.030, -0.085], [xn + 0.035, 0.030, -0.025]]),
         "drive": "crank on the EMA output + over-centre drag link to the leg yoke (down-lock at the over-centre "
                  "position); torque requirement structures G-EMA-NOSE",
         "install": "allocation envelope 60 x 60 x 60 mm (estimate) between the keel walls above the pivot, bolted to "
                    "the starboard keel-wall land; >= 20 mm above the swept yoke"},
        {"id": "ACT-STEER", "part": "YK250-FC-628", "name": "nose-wheel steering actuator Volz DA 26",
         "mass_item": "actuators_nose_steering_brake_2x_DA26", "moves_with": "nose_gear",
         "obb": {"center": r3(c_st), "axes": r3(axes.tolist()), "half": half},
         "box": r3([C.min(axis=0).tolist(), C.max(axis=0).tolist()]),
         "drive": "toothed belt to the steering collar of the fork (SAGITTA practice); centred before retraction",
         "install": "case on the aft face of the nose leg at 35 % of its length (rest pose = gear down; components.yaml "
                    "volz_da26 102.8 x 54 x 26 mm), retracts with the leg (joint nose_gear) and lies on top of the "
                    "stowed leg (fix round 2, PK2-13: the forward face would put it between the stowed leg and the "
                    "lower skin)"}]


def harness() -> dict:
    def P(*pts):
        return [r3(p) for p in pts]
    trunks = [
        {"id": "H-MAIN", "part": "YK250-SY-750", "name": "main power/data trunk", "diameter": 0.020,
         "path": P([0.950, -0.110, 0.031], [1.110, -0.143, 0.035], [1.330, -0.1475, 0.049], [X_PFF, -0.195, 0.035],
                   [X_PFR, -0.195, 0.035], [2.000, -0.270, -0.090], [X_FUELF, -0.270, -0.090],
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
         "path": P([1.000, 0.150, -0.010], [1.110, 0.145, -0.005], [1.200, 0.1475, 0.049], [1.330, 0.1475, 0.049],
                   [X_PFF, 0.195, 0.035], [X_PFR, 0.195, 0.035], [2.000, 0.270, -0.090], [X_FUELF, 0.270, -0.090],
                   [X_DUCT, 0.290, -0.105], [X_DUCT, 0.150, -0.103], [X_DUCT, 0.0, -0.103],
                   [X_GEARF + 0.020, 0.0, -0.103], [X_GEARF + 0.100, 0.0, -0.050]),
         "penetrations": [{"member": "M-WELLWALL-FWD", "point": r3([X_W0 - 0.0008, 0.0, -0.103]),
                           "kind": "sealed conduit entry (centre-line channel, lower tier)"}],
         "text": "avionics side bay -> under the avionics deck to FS1110 -> starboard side of the turret and parachute "
                 "bays -> mirror of the main trunk -> shares the lateral duct and the centre-line channel (lower tier, "
                 "shielded coax) -> aft equipment bay -> fin-tip antennas (C2-B, backup) via the fin branches"},
        {"id": "H-WING", "part": "YK250-SY-752", "name": "wing branch (each side)", "diameter": 0.012, "mirror": True,
         "path": P([X_FUELF - 0.040, 0.270, -0.090], [X_FUELF + 0.040, 0.285, -0.090],
                   [spar_x(0.300, 0.25) - 0.030, 0.300, -0.090], [spar_x(0.300, 0.25) + 0.030, 0.300, -0.090],
                   [2.640, 0.340, -0.060], [2.680, 0.360, -0.046], [2.685, 0.370, -0.020], [2.700, 0.400, -0.015],
                   [2.715, 0.550, -0.012], [2.725, 0.690, -0.010]),
         "penetrations": [{"member": "M-CTBOX", "point": r3([2.682, 0.364, Z_BOX[0]]),
                           "kind": "sealed grommet in the lower box cover (inside the body, outboard of the payload "
                                   "bay)"},
                          {"member": "M-SOB", "point": r3([2.700, Y_SOB, -0.015]),
                           "kind": "grommet in the side-of-body rib between the spars"},
                          {"member": "M-GLOVERIB", "point": r3([2.715, 0.550, -0.012]),
                           "kind": "grommet in the glove rib between the spars"}],
         "text": "fix round 1 (VPK-13): from the trunk in the mission bay under the forward fuel deck (outside the "
                 "vapour-tight bay, C-HARN-FUEL / C-HARN-MS with the main/coax trunk) to below the centre wing box, "
                 "up through the lower box cover into a bonded conduit between the spars, through the side-of-body "
                 "and glove ribs to the blind-mate MIL-DTL-38999 receptacle CN-WING on the joint rib between the "
                 "spars; no harness inside a fuel bay, no LERX rib or pin corridor crossed"},
        {"id": "H-TAIL", "part": "YK250-SY-753", "name": "tail branch (each side)", "diameter": 0.016, "mirror": True,
         "path": P([X_GEARF + 0.200, 0.040, 0.040], [3.450, 0.100, 0.180], [3.520, 0.120, round(z_top(3.520, 0.120)
                                                                                                   - 0.025, 4)]),
         "text": "aft equipment bay -> FS3480 ring -> fin root (rudder actuator, fin-tip antenna coax, tail light); "
                 "stabilator actuator leads on the firewall forward face"},
        {"id": "H-ENGINE", "part": "YK250-SY-754", "name": "engine harness", "diameter": 0.020,
         "path": P([3.280, -0.050, 0.010], [3.600, -0.095, -0.0175], [3.700, -0.095, -0.0175]),
         "text": "ECU / generator PE -> firewall fireproof grommets (C-FW-HARN) -> injectors, coils, sensors, SG750 "
                 "(engine side in fire sleeve, >= 50 mm from the exhaust)"},
        {"id": "H-NOSE", "part": "YK250-SY-755", "name": "nose branch", "diameter": 0.012,
         "path": P([0.795, 0.082, 0.034], [0.700, 0.082, 0.034], [0.640, 0.0625, 0.030], [0.600, 0.0625, 0.030],
                   [0.395, 0.055, 0.030], [0.335, 0.0, 0.012], [0.300, 0.0, 0.010]),
         "text": "FTS, nose-cone antennas (coax), transponder blade feed, pitot heater; pitot lines alongside"}]
    return {"trunks": trunks,
            "rules": "trunks are keep-out corridors (diameter + 10 mm); grommets at every frame (cut-outs in "
                     "layout.stations); power and RF coax separated by >= 20 mm except in the shared centre-line "
                     "channel (shielded coax); >= 50 mm from the exhaust; no harness inside a fuel bay; wing and tail "
                     "branches separable at MIL-DTL-38999 connectors in the access bays",
            "connectors": [{"id": "CN-WING", "location": "joint rib y +-0.70 between the spars (inboard face, "
                                                      "float-mounted blind-mate receptacle, mates in the last 8 mm "
                                                      "of the outer-panel insertion stroke)", "point": r3([2.725, 0.690, -0.010]),
                            "type": "MIL-DTL-38999 series III, 19-pin + 2 coax inserts (estimate)"},
                           {"id": "CN-TAIL", "location": "FS3480 ring (aft equipment bay side)", "type":
                            "MIL-DTL-38999 series III"},
                           {"id": "CN-ENGINE", "location": "forward face of the firewall", "type":
                            "fire-resistant circular connector (engine side) / grommet"}]}


def fuel_lines() -> list:
    """Fuel line routes (fix round 1, VPK-09): refuel, transfer, feed / return, vent and drain, with diameters and the
    frame / deck penetrations they use (layout_check C02 / C04)."""
    fc = {c["name"]: c for c in S["layout"]["fuel_cells"]}
    xf, xs, xa = fc["forward_cell"]["x"], fc["saddle_cell"]["x"], fc["aft_cell"]["x"]
    zr = round(z_bot(2.340, 0.320) + 0.035, 4)

    def L(lid, part, name, name_tr, d, path, pens=(), text="", end_fitting=None):
        d_ = {"id": lid, "part": part, "name": name, "name_tr": name_tr, "diameter": d, "path": [r3(q) for q in path],
              "penetrations": [{"member": m, "point": r3(q), "kind": k} for m, q, k in pens], "text": text}
        if end_fitting:
            d_["end_fitting"] = end_fitting          # the line ends in a fitting in the skin (layout_check C03)
        return d_
    return [
        L("FL-REFUEL", "YK250-FU-580", "refuel line (OBP dry-break AN6 -> forward cell)", "yakıt ikmal hattı", 0.016,
          [[2.340, -0.320, zr], [2.340, -0.320, -0.064], [2.340, -0.320, -0.050]],
          [("M-FWDDECK", [2.340, -0.320, -0.0566], "sealed bulkhead union in the forward fuel deck")],
          "coupling on a bracket under the forward fuel deck (P-REFUEL door), straight up through the deck into the "
          "forward cell bottom fitting; outboard of the port keel beam and 32 mm from the main trunk H-MAIN"),
        L("FL-XFER-1", "YK250-FU-581", "transfer line forward -> saddle cell", "aktarma hattı ön -> eyer", 0.012,
          [[xf[1] - 0.010, 0.0, 0.095], [xs[0] + 0.010, 0.0, 0.095]], (), "through FS-MS C-FUEL-MS (sealed union)"),
        L("FL-XFER-2", "YK250-FU-582", "transfer line saddle -> aft cell", "aktarma hattı eyer -> arka", 0.012,
          [[xs[1] - 0.010, 0.0, 0.095], [xa[0] + 0.010, 0.0, 0.095]], (), "through FS-RS C-FUEL-RS (sealed union)"),
        L("FL-FEED-1", "YK250-FU-583", "feed line aft-cell sump -> pump", "besleme hattı (sump -> pompa)", 0.012,
          [[xa[1] - 0.010, 0.040, -0.040], [X_GEARF - 0.010, 0.040, -0.025], [X_GEARF + 0.0040, 0.040, -0.025]],
          (), "aft-cell sump fitting -> FS-GEAR C-FUEL-FEED (sealed union) -> EFI pump inlet"),
        L("FL-FEED-2", "YK250-FU-585", "feed line pump -> shut-off valve", "besleme hattı (pompa -> kesme vanası)",
          0.012, [[X_GEARF + 0.116, 0.060, -0.020], [3.300, 0.060, -0.010], [3.594, 0.040, 0.020]], (),
          "along the aft equipment bay floor angles, through the FS3480 ring opening"),
        L("FL-FEED-3", "YK250-FU-586", "feed line shut-off valve -> engine (fire sleeve)",
          "besleme hattı (kesme vanası -> motor)", 0.012,
          [[3.646, 0.040, 0.030], [X_FW + 0.035, 0.040, 0.030]], (),
          "through the firewall C-FW-FUEL (fireproof bulkhead union + fire sleeve)"),
        L("FL-RETURN", "YK250-FU-587", "return line engine regulator -> aft cell", "dönüş hattı", 0.010,
          [[X_FW + 0.030, 0.052, 0.042], [3.648, 0.052, 0.042], [3.630, 0.080, 0.050], [3.300, 0.105, 0.000],
           [X_GEARF + 0.030, 0.105, -0.017], [X_GEARF - 0.010, 0.105, -0.017], [xa[1] - 0.010, 0.090, 0.000]], (),
          "firewall C-FW-FUEL (above the feed line) -> over the shut-off valve -> aft equipment bay outboard of the "
          "pump -> FS-GEAR C-FUEL-RET (sealed union) -> aft-cell return fitting"),
        L("FL-VENT", "YK250-FU-588", "vent line (cells -> flush NACA vent)", "havalandırma hattı", 0.010,
          [[xa[1] - 0.015, 0.075, 0.140], [X_GEARF + 0.020, 0.075, 0.115], [3.235, 0.075, 0.000],
           [3.235, 0.150, round(z_bot(3.235, 0.150) + 0.012, 4)]], (),
          "from the aft-cell top (the cells are interconnected at the top by FL-XFER) through FS-GEAR C-FUEL-VENT "
          "(sealed union) down to the flush NACA vent in P-AFT-LOWER", end_fitting="skin"),
        L("FL-DRAIN", "YK250-FU-589", "gascolator / filter drain", "filtre boşaltma hattı", 0.008,
          [[X_GEARF + 0.060, 0.060, -0.066], [X_GEARF + 0.060, 0.060, round(z_bot(X_GEARF + 0.060, 0.060) + 0.010, 4)]],
          (), "drain valve flush with the aft equipment hatch P-AFTHATCH", end_fitting="skin")]
