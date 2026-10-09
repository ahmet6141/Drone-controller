"""Layout builder: engine mount + firewall, turret elevator, parachute bridle, fuel-cell supports, equipment trays and
the design-load cases of the interface fittings (values are computed by ucav250.analysis.layout_check)."""
from __future__ import annotations

from b_common import *  # noqa: F401,F403


def _mount_axes():
    up = np.array([-math.sin(EPS), 0.0, math.cos(EPS)])
    return D_THRUST, np.array([0.0, 1.0, 0.0]), up


def engine_mount(fittings: list) -> dict:
    """Welded 4130 bed mount: 4 firewall feet -> isolator ring in the crankcase mount-face plane."""
    ax, ey, ez = _mount_axes()
    mf = engine_point(MOUNT_FACE_AHEAD)
    holes = [r3(mf + float(h[0]) * ey + float(h[1]) * ez) for h in ENG["mount"]["bolt_pattern_yz_m"]]
    # isolator centres 30 mm aft of the bolt heads (isolator 25 mm + ring web)
    iso = [r3(np.asarray(h) - 0.0125 * ax) for h in holes]                 # isolator centre 12.5 mm ahead of the face
    ring_c = r3(mf - 0.030 * ax)                                             # ring plane 30 mm forward of the face
    feet = []
    for f in fittings:
        if f["id"].startswith("F-EMOUNT"):
            p = np.asarray(f["point"], float)
            feet.append(r3([X_FW, p[1], p[2]]))
            feet.append(r3([X_FW, -p[1], p[2]]))
    # ring nodes on the boss directions at r = 0.095 m (radial cup arms to the isolators at the bosses): the truss
    # stays outside the SG750 (d 101 mm) between the firewall and the mount face
    ring_nodes = []
    for h in ENG["mount"]["bolt_pattern_yz_m"]:
        d = np.array([float(h[0]), float(h[1])])
        d = d / np.linalg.norm(d)
        ring_nodes.append(r3(np.asarray(ring_c) + 0.095 * (d[0] * ey + d[1] * ez)))
    tubes = []
    axc = np.asarray(ring_c, float)

    def ang(p):
        q = np.asarray(p, float) - axc
        return math.atan2(q @ ez, q @ ey)
    for k, f in enumerate(feet):
        same = [j for j, n in enumerate(ring_nodes) if np.sign(n[1] - axc[1]) == np.sign(f[1])]
        for j in same:
            tubes.append({"from": f"foot{k + 1}", "to": f"ring{j + 1}", "a": f, "b": ring_nodes[j],
                          "length": r3(float(np.linalg.norm(np.asarray(f) - np.asarray(ring_nodes[j]))))})
    low_feet = [k for k, f in enumerate(feet) if f[2] < axc[2]]
    for k in low_feet:                                      # lower cross diagonals pass below the SG750
        f = feet[k]
        opp = [j for j, n in enumerate(ring_nodes) if np.sign(n[1] - axc[1]) != np.sign(f[1]) and
               n[2] < axc[2] - 0.06]                     # only nodes well below the crank axis
        for j in opp:
            tubes.append({"from": f"foot{k + 1}", "to": f"ring{j + 1}", "a": f, "b": ring_nodes[j],
                          "length": r3(float(np.linalg.norm(np.asarray(f) - np.asarray(ring_nodes[j])))),
                          "note": "lower cross diagonal (below the SG750)"})
    return {
        "part": "YK250-CH-085",
        "type": "welded 4130 N tube bed mount: 4 feet on the firewall attach fittings (F-EMOUNT-UP/LO, R/L) -> a "
                "welded ring (4130 tube 15.9 x 0.89 mm) in a plane 30 mm forward of the crankcase mount face; 4 "
                "elastomer isolators in the ring cups carry the engine on its 4 rear crankcase bosses",
        "material": "steel_4130_n", "process": "welded tube truss (TIG, normalised after welding), 12.7 x 0.89 mm "
                                               "struts (estimate, sized in the detail phase)",
        "thrust_axis": {"point": r3(HUB), "direction_aft": r3(D_THRUST),
                        "inclination_deg": float(PR["thrust_line_inclination_deg"]),
                        "text": "crank / thrust axis through propeller.hub, 5 deg aft-up (down-thrust in flight "
                                "direction); the mount ring is square to this axis (engine.yaml: thrust angle set by "
                                "the mount plate)"},
        "mount_face": {"center": r3(mf), "normal_aft": r3(D_THRUST),
                       "distance_ahead_of_prop_plane": r3(MOUNT_FACE_AHEAD),
                       "source": "engine.yaml installation.datums (181.4 mm flange-to-mount-face, read_from_drawing) "
                                 "+ propeller.hub_spacer + hub half thickness"},
        "bolts": {"count": 4, "size": "M8", "spec": "ISO 4762 M8 12.9 socket head cap screw + NORD-LOCK washer, "
                                                    "12 mm thread engagement in the crankcase bosses (engine.yaml "
                                                    "installation.mount.bolts), safety-wired in pairs",
                  "points": holes,
                  "pattern_source": "engine.mount.bolt_pattern_yz_m (estimate scaled from the L 275 E datasheet, "
                                    "+/-3 mm; for packaging only, NOT for drilling: the drilling template is taken "
                                    "from the Limbach drawing / the engine itself)"},
        "isolators": {"make_model": "Limbach optional 'Damper Shock Mount' (engine.yaml installation.mount.isolators;"
                                    " dimensions and stiffness not published)",
                      "packaging_envelope": "estimate: conical elastomer isolator d 40 x 25 mm per boss, fail-safe "
                                            "snubbing washer and through-bolt (engine stays captive if the elastomer "
                                            "fails)",
                      "target": "rigid-body mount modes <= engine.mount.isolator_rigid_body_modes_Hz_max (20 Hz; "
                                "engine.yaml vibration.isolator_target)",
                      "tag": "estimate", "centres": iso},
        "ring_center": ring_c, "ring_nodes": ring_nodes, "feet": feet, "tubes": tubes,
        "firewall_attach": "each foot: 4130 cup bolted with 2 x M8 12.9 through the firewall stack (F-EMOUNT "
                           "fittings: 7075 backing plate 40 x 40 x 5 mm on the forward face, stainless spacer tubes "
                           "through the sandwich and the air gap; stainless shield clamped under the foot)",
        "firewall_stackup": {
            "station": "FS3670", "aft_face_x": X_FW, "forward_face_x": r3(X_FW - 0.0148),
            "layers_fwd_to_aft": [
                {"layer": "CFRP sandwich bulkhead (layups.rib_panel)", "t": 0.0068},
                {"layer": "air gap on 12 stainless stand-offs (insulation blanket optional)", "t": 0.0075},
                {"layer": "AISI 304 stainless sheet, fireproof without test (>= 0.38 mm, standards.yaml FIRE-001)",
                 "t": 0.0005}],
            "text": "layout.firewall_x is the AFT face (stainless shield); the stack lies forward of it so the "
                    "firewall_gap of the sizing (SG750 front 20 mm aft of the shield) is kept; penetrations only "
                    "through fireproof unions, grommets and bellows (stations FS3670 cut-outs); fuel cells >= 13 mm "
                    "ahead of the forward face (CS-LUAS.967(c))"},
        "removal": "engine off the isolators to the aft after removing propeller, spinner, cowls, exhaust, harness "
                   "plugs, fuel quick-disconnects and 4 M8 bolts; the mount stays on the firewall",
        "design_load_case": "DL-ENGINE-MOUNT"}


def turret_elevator() -> dict:
    xt = X_TUR
    zr = float(TU["ball_center_retracted_z"])
    return {
        "joint": "turret_elevator",
        "part": "YK250-PL-824",
        "rails": [{"id": f"RAIL-{c}", "part": "YK250-PL-828", "corner": c,
                   "line": r3([[xt + sx * 0.085, sy * 0.094, -0.150], [xt + sx * 0.085, sy * 0.094, 0.095]])}
                  for c, sx, sy in (("FR", -1, 1), ("AL", 1, -1))],
        "rail_spec": "2 miniature profile rails 9 mm (MGN9 class, steel, estimate) in two diagonal bay corners "
                     "(front-starboard, aft-port), bonded and screwed (M3 into potted inserts) to the wall corner "
                     "angles; 2 carriages each on the turret carriage plate",
        "drive": {"spec": "ball screw d 8 x 2 mm in the front-port corner, BLDC gear motor with spring-applied brake "
                          "on the bay roof, 2 limit switches + absolute encoder (estimate)",
                  "screw_axis": r3([[xt - 0.085, -0.094, 0.095], [xt - 0.085, -0.094, -0.150]]),
                  "top_anchor": {"member": "M-TURRETROOF", "point": r3([xt - 0.085, -0.094, 0.106]),
                                 "attach": "bearing block 4 x M4 into potted inserts of the roof sandwich"},
                  "rail_anchors": "rail ends on FS1110 (front rail) and FS1330 (aft rail) through the wall corner "
                                  "angles"},
        "stroke": float(TU["stroke"]), "ball_center_retracted": r3([xt, 0.0, zr]),
        "ball_center_extended": r3([xt, 0.0, float(TU["ball_center_extended_z"])]),
        "doors": "two sliding doors (turret_door_R/L) on rails outboard of the bay walls, driven from the carriage by "
                 "cam-slot levers (mechanical sequence: no separate door actuator, doors cannot close on the ball)",
        "load_path": "turret inertia (emergency landing 9 g fwd / 6 g down ultimate, CS-VLA 561) -> carriage -> 2 "
                     "rails -> walls M-TURRETWALL -> FS1110 / FS1330; drive load -> ball screw -> roof",
        "design_load_case": "DL-TURRET"}


def parachute(fittings: list, cg: list) -> dict:
    """Y-bridle: forward leg to FS1810, aft leg to the rear-spar frame; confluence above the CG for a ~3 deg nose-down
    hang at the MTOW CG."""
    fa = np.asarray(next(f for f in fittings if f["id"] == "F-RISER-FWD")["point"], float)
    fb = np.asarray(next(f for f in fittings if f["id"] == "F-RISER-AFT")["point"], float)
    th = math.radians(3.0)
    u = np.array([math.sin(th), 0.0, math.cos(th)])          # world up in body axes for a nose-down pitch th
    H = 1.50
    C = np.asarray(cg, float) + H * u
    la, lb = float(np.linalg.norm(C - fa)), float(np.linalg.norm(C - fb))
    return {
        "unit": "UAVOS 200 Parachute System (components.yaml#recovery_and_safety.parachutes[uavos_200_parachute_"
                "system]: 4.5 kg, 0.300 x 0.300 x 0.275 m, max load 220 kg, max load factor 5 g, deploy <= 97 m/s)",
        "container": {"zone": "parachute_bay", "part": "YK250-SY-800",
                      "restraint": "4 brackets YK250-CH-114 on the bay walls/floor (2 x M5 each) + 2 straps; "
                                   "container mouth under the dorsal hatch"},
        "hatch": {"panel": "P-PARAHATCH", "joint": "para_hatch", "latch": "YK250-SY-804 pin-puller latch (FTS "
                                                                          "command), tether to the hatch"},
        "bridle": {"type": "Y-bridle (aramid webbing), confluence ring above the CG",
                   "forward_leg": {"fitting": "F-RISER-FWD", "point": r3(fa), "length": r3(la, 3),
                                   "route": "straight up out of the container mouth"},
                   "aft_leg": {"fitting": "F-RISER-AFT", "point": r3(fb), "length": r3(lb, 3),
                               "route": "stowed in the dorsal spine channel (P-SPINE) under a tear-away cover from "
                                        "the parachute bay to the rear-spar frame"},
                   "confluence_above_cg": H, "design_hang": "about 3 deg nose-down at the MTOW CG (verified by "
                                                           "layout_check for the CG range)",
                   "cg_used": r3(cg)},
        "load_path": "canopy -> riser -> confluence ring -> 2 legs -> U-lug fittings (4 x M6 12.9 each, 7075 "
                     "doublers both faces) -> FS1810 / rear-spar frame -> chine longerons, keel and the wing box",
        "design_load_case": "DL-PARACHUTE"}


def fuel_supports() -> list:
    out = []
    for c in S["layout"]["fuel_cells"]:
        out.append({"cell": c["name"], "x": r3(c["x"]), "z": r3(c["z"]),
                    "support": "bay liner (CFRP 2 plies, smooth, no sharp edges) bonded to the frames, keel beams "
                               "and deck; cell hung from 6 Velcro/loop tabs + 2 restraint straps; vertical and "
                               "lateral loads -> liner -> frames/decks; fore/aft (9 g ultimate emergency landing) -> "
                               "frames FS-FUEL / spar frames / FS-GEAR",
                    "access": {"forward_cell": "P-FUEL1", "saddle_cell": "P-FUEL2", "aft_cell": "P-FUEL3"}[c["name"]],
                    "parts": {"liner": "YK250-CH-116", "cell": {"forward_cell": "YK250-FU-570",
                                                                "saddle_cell": "YK250-FU-571",
                                                                "aft_cell": "YK250-FU-572"}[c["name"]]}})
    return out


def trays() -> list:
    return [
        {"id": "TR-SIDEBAY", "part": "YK250-CH-118", "mirror": True, "zone": "avionics_side_bays",
         "spec": "CFRP flat tray 3 mm with potted M4 inserts, on 4 elastomer isolators (autopilot side only)"},
        {"id": "TR-FWDBAY", "part": "YK250-CH-119", "zone": "forward bay FS0300-FS0600",
         "spec": "CFRP tray on FS0300/FS0600 angles (FTS unit, antennas splitters)"},
        {"id": "TR-MISSION", "part": "YK250-CH-120", "zone": "mission_computer",
         "spec": "tray with Dzus quarter-turn retainers on the mission-bay floor"},
        {"id": "TR-AFTBAY", "part": "YK250-CH-121", "zone": "equipment_bay_aft",
         "spec": "aluminium 6061-T6 tray 2 mm (ECU / PE heat sink), M4 nutplates on the floor angles"},
        {"id": "TR-PAYLOAD", "part": "YK250-PL-852", "zone": "payload_bay",
         "spec": "payload tray on 2 rails under the forward fuel deck (M-FWDDECK), 4 x M5 captive screws; research "
                 "payload <= mission.payload_max_kg retained for 9 g fwd / 6 g down ultimate"}]


def design_loads() -> dict:
    return {
        "basis": "structures (CS-LUAS tailored, STANAG 4703): limit n +3.8/-1.52 manoeuvre, gust envelope "
                 "(sizing loads.n_limit_wing_design), FoS 1.5, fitting factor 1.15, bearing factor 2.0 (pinned "
                 "joints), frequent-assembly factor 1.5; emergency landing ultimate inertia 9 fwd / 3 up / 1.5 side / "
                 "6 down (standards.yaml design_loads_recommended.emergency_landing_ultimate_inertia); emergency "
                 "parachute = ultimate-only case (CRASH-004). Combination: ultimate = limit x 1.5 x max(special "
                 "factor) (CS-LUAS.619).",
        "method": "first-cut pre-sizing of the interface fittings in ucav250.analysis.layout_check "
                  "(fitting_presizing): closed-form pin shear / bending / bearing and bolt-group shear against "
                  "the spec.materials allowables; reported as margins of safety MS = allowable / applied - 1. The "
                  "detail phase replaces these with the FE / test substantiation.",
        "cases": [
            {"id": "DL-WINGJOINT", "item": "outer-panel joint y 0.70 (P-MAIN1/2, fork, tongue, drag pin)",
             "condition": "symmetric flight at n = loads.n_limit_wing_design (gust envelope) x MTOM, Schrenk lift on "
                          "the reference trapezoid, no inertia relief (as sizing.wing_structure)",
             "factors": "1.5 x 1.5 (frequent assembly) on pin shear / bending; x 2.0 bearing factor on bearing"},
            {"id": "DL-PARACHUTE", "item": "bridle fittings F-RISER-FWD / F-RISER-AFT",
             "condition": "opening shock 13.1 kN (Galaxy GRS 4/240 published value at 240 kg / 240 km/h, larger "
                          "than UAVOS 200 5 g x MTOM) taken by EACH leg alone, direction from vertical to 60 deg "
                          "aft (snatch)", "factors": "ultimate-only (CRASH-004) x 1.15 fitting factor"},
            {"id": "DL-ENGINE-MOUNT", "item": "4 x M8 crankcase bolts, firewall feet",
             "condition": "(a) limit torque 154.7 N m (MCP x 6, standards.yaml engine_mount) + 3.8 g vertical; (b) "
                          "1.47 g side; (c) 15 g forward crash (ultimate, retention)",
             "factors": "1.5 on (a), (b); x 1.15 fitting; (c) ultimate as given x 1.15"},
            {"id": "DL-TURRET", "item": "elevator rails / carriage",
             "condition": "emergency landing 9 g fwd / 6 g down / 1.5 g side ultimate on the E180 growth turret mass",
             "factors": "ultimate as given x 1.15"},
            {"id": "DL-GEAR", "item": "trunnion and nose pivot fittings",
             "condition": "landing_gear loads (structures.landing: CS-LUAS App. H, STANAG Annex B nose multiples)",
             "factors": "1.5 x 1.15 (fittings), bearing x 2.0; numbers in the gear/chassis detail phase "
                        "(gear unit selection pending)"},
            {"id": "DL-TAIL", "item": "fin root, stub and ventral fittings, stabilator spindle bearings",
             "condition": "CS-LUAS.441/427 tail loads (detail phase; the spindle is pre-sized in sizing "
                          "stab_spindle_check)", "factors": "1.5 x 1.15, bearing x 2.0"}]}
