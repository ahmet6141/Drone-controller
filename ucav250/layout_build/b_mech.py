"""Layout builder: mechanisms (joints + sequences), keep-out envelopes, clearance values and clearance rules."""
from __future__ import annotations

from b_common import *  # noqa: F401,F403

DEG = math.pi / 180.0


def unit(v):
    v = np.asarray(v, float)
    return v / np.linalg.norm(v)


def mirror_axis_same_sense(a):
    """Port axis giving the SAME physical deflection sense as the starboard axis a (mirror + reversal)."""
    a = np.asarray(a, float)
    return -np.array([a[0], -a[1], a[2]])


def hinge(srf_name, eta0, eta1, xc, frac=0.5):
    srf = af.wing if srf_name == "wing" else af.tail[srf_name]
    p0 = surf_point(srf, eta0, xc, frac)
    p1 = surf_point(srf, eta1, xc, frac)
    return p0, p1


def joint(name, kind, origin, axis, lo, hi, rest=0.0, prop="", scale=1.0, parent=None, expr="", moves="",
          notes="", side="C", mirror_of=None):
    d = {"name": name, "kind": kind, "origin": r3(origin, 5), "axis": r3(unit(axis), 6), "lo": r3(lo, 6),
         "hi": r3(hi, 6), "rest": r3(rest, 6), "prop": prop, "scale": r3(scale, 7)}
    if parent:
        d["parent"] = parent
    if expr:
        d["expr"] = expr
    d["moves"] = moves
    d["side"] = side
    if mirror_of:
        d["mirror_of"] = mirror_of
    if notes:
        d["notes"] = notes
    return d


def mechanisms() -> dict:
    W = S["wing"]["controls"]
    b2 = 0.5 * float(S["wing"]["span"])
    J = []
    # ---------------------------------------------------------------- wing control surfaces
    for nm, key, fc, ctrl in (("aileron", "aileron", "YK250-FC-200", "aileron_deg"),
                              ("flap", "flap", "YK250-FC-202", "flap_deg")):
        c = W[key]
        y0, y1 = float(c["eta0"]) * b2, float(c["eta1"]) * b2
        p0, p1 = hinge("wing", wing_eta(y0), wing_eta(y1), float(c["xc_hinge"]))
        a = unit(p1 - p0)                                   # outboard along the hinge: + = trailing edge down
        lo, hi = (float(v) * DEG for v in c["range_deg"])
        sc = -DEG if key == "aileron" else DEG              # aileron_deg + = roll right (starboard TE up)
        J.append(joint(f"{nm}_R", "revolute", p0, a, lo, hi, prop=ctrl, scale=sc, side="R",
                       moves=f"{fc}-R (+ its horn; FC parts of the wing module)",
                       notes=f"hinge line at {c['xc_hinge']} chord, mid-thickness, y {y0:.3f}-{y1:.3f} m; + = TE down"))
        pm = p0 * np.array([1, -1, 1])
        sc_l = DEG if key == "aileron" else DEG
        J.append(joint(f"{nm}_L", "revolute", pm, mirror_axis_same_sense(a), lo, hi, prop=ctrl, scale=sc_l, side="L",
                       mirror_of=f"{nm}_R", moves=f"{fc}-L", notes="+ = TE down (port)"))
    # ---------------------------------------------------------------- rudders (fin hinge 70 % chord)
    rc = FIN["controls"]["rudder"]
    e0, e1 = 0.300, 0.880
    p0, p1 = hinge("fin", e0, e1, float(rc["xc_hinge"]))
    a = unit(p1 - p0)                                       # up the span: + = TE to starboard
    lo, hi = (float(v) * DEG for v in rc["range_deg"])
    J.append(joint("rudder_R", "revolute", p0, a, lo, hi, prop="rudder_deg", scale=-DEG, side="R",
                   moves="YK250-FC-300-R", notes=f"fin span coordinate {e0}-{e1} m (root clear of the cowl, tip cap "
                                                 "above 0.88 m); rudder_deg + = trailing edges to port"))
    pm = p0 * np.array([1, -1, 1])
    am = np.array([a[0], -a[1], a[2]])
    J.append(joint("rudder_L", "revolute", pm, am, lo, hi, prop="rudder_deg", scale=-DEG, side="L",
                   mirror_of="rudder_R", moves="YK250-FC-300-L",
                   notes="axis = mirrored up-span axis: + still moves the trailing edge to starboard"))
    # ---------------------------------------------------------------- stabilators (spindle axis y)
    pv = np.array(STAB["pivot"], float)
    lo, hi = (float(v) * DEG for v in STAB["controls"]["range_deg"])
    for sd, sgn in (("R", 1.0), ("L", -1.0)):
        J.append(joint(f"stabilator_{sd}", "revolute", pv * np.array([1, sgn, 1]), [0.0, 1.0, 0.0], lo, hi,
                       prop="elevator_deg", scale=DEG, side=sd, moves=f"YK250-TL-256-{sd} + spindle YK250-TL-301-{sd}"
                       f" + horn", notes="all-moving stabilator about the stub spindle; + = TE down (both sides, "
                                         "axis +y)"))
    # ---------------------------------------------------------------- main gear (+ inner doors, sequenced)
    T_ = np.array(MG["trunnion"], float)
    ang = float(MG["retraction"]["angle_deg"]) * DEG
    J.append(joint("main_gear_R", "revolute", T_, [-1.0, 0.0, 0.0], 0.0, ang, prop="gear_up", scale=ang,
                   expr=f"{ang:.5f}*clamp((gear_up-0.15)/0.7,0,1)", side="R",
                   moves="YK250-LG-620-R leg, YK250-LG-622-R wheel/tyre/brake, YK250-LG-672-R leg door",
                   notes="rest = gear down (modelled); + = inward retraction (108 deg) about the longitudinal "
                         "trunnion axis; gear_up 0 down, 1 up"))
    J.append(joint("main_gear_L", "revolute", T_ * np.array([1, -1, 1]), [1.0, 0.0, 0.0], 0.0, ang, prop="gear_up",
                   scale=ang, expr=f"{ang:.5f}*clamp((gear_up-0.15)/0.7,0,1)", side="L", mirror_of="main_gear_R",
                   moves="YK250-LG-620-L, YK250-LG-622-L, YK250-LG-672-L"))
    y_h = 0.5 * float(MG["well_gap"])
    x_w = 0.5 * (float(ZP["main_gear_wells"]["box"][0][0]) + float(ZP["main_gear_wells"]["box"][1][0]))
    o_d = np.array([x_w, y_h, z_bot(x_w, y_h) + 0.002])
    a_door = 95.0 * DEG
    J.append(joint("main_inner_door_R", "revolute", o_d, [-1.0, 0.0, 0.0], 0.0, a_door, prop="", scale=1.0,
                   expr=f"{a_door:.5f}*(clamp(gear_up/0.15,0,1)-clamp((gear_up-0.85)/0.15,0,1))", side="R",
                   moves="YK250-LG-670-R",
                   notes="hinged at the inboard well edge (y = well_gap/2); opens down 95 deg before the leg moves "
                         "and closes after the up-/down-lock (closed at both ends of the sequence)"))
    J.append(joint("main_inner_door_L", "revolute", o_d * np.array([1, -1, 1]), [1.0, 0.0, 0.0], 0.0, a_door,
                   prop="", scale=1.0, expr=f"{a_door:.5f}*(clamp(gear_up/0.15,0,1)-clamp((gear_up-0.85)/0.15,0,1))",
                   side="L", mirror_of="main_inner_door_R", moves="YK250-LG-670-L"))
    # ---------------------------------------------------------------- nose gear (+ steering, clamshell doors)
    Pn = np.array(NG["pivot"], float)
    an = float(NG["retraction"]["angle_deg"]) * DEG
    J.append(joint("nose_gear", "revolute", Pn, [0.0, -1.0, 0.0], 0.0, an, prop="", scale=1.0,
                   expr=f"{an:.5f}*clamp((gear_up-0.15)/0.7,0,1)", moves="YK250-LG-624 leg, YK250-LG-626 wheel/tyre, "
                   "YK250-FC-628 steering actuator", notes="+ = aft retraction 90 deg about the lateral pivot"))
    J.append(joint("nose_steer", "revolute", Pn, [0.0, 0.0, -1.0], -20.0 * DEG, 20.0 * DEG, prop="steer_deg",
                   scale=DEG, parent="nose_gear", moves="YK250-LG-626 wheel + fork",
                   notes="steering about the strut axis (vertical through the pivot in the down position; trail "
                         "landing_gear.nose.trail aft of it); +-20 deg design choice for taxi; centred (0) before "
                         "retraction by the steering actuator (sequence check at 0)"))
    xs0, xs1 = float(ZP["nose_gear_well"]["box"][0][0]), float(ZP["nose_gear_well"]["box"][1][0])
    yh = float(ZP["nose_gear_well"]["box"][1][1])
    q0 = np.array([xs0, yh, z_bot(xs0, yh) + 0.002])
    q1 = np.array([xs1, yh, z_bot(xs1, yh) + 0.002])
    for sd, sgn in (("R", 1.0), ("L", -1.0)):
        J.append(joint(f"nose_door_{sd}", "revolute", q0 * np.array([1, sgn, 1]),
                       sgn * unit(q1 - q0) * np.array([1, sgn, 1]), 0.0, 90.0 * DEG, prop="", scale=1.0,
                       expr=f"{90 * DEG:.5f}*(1-clamp((gear_up-0.85)/0.15,0,1))", side=sd,
                       moves=f"YK250-LG-674-{sd}",
                       notes="clamshell hinged at the keel-slot edge; open (90 deg, hanging) whenever the gear is not "
                             "up-locked, closed for gear_up > 0.85 + 0.15"))
    # ---------------------------------------------------------------- turret elevator + sliding bay doors
    xt = X_TUR
    zr = float(TU["ball_center_retracted_z"])
    stroke = float(TU["stroke"])
    J.append(joint("turret_elevator", "prismatic", [xt, 0.0, zr], [0.0, 0.0, -1.0], 0.0, stroke, prop="turret",
                   scale=stroke, expr=f"{stroke:.4f}*clamp((turret-0.2)/0.8,0,1)",
                   moves="YK250-PL-820 turret, YK250-PL-822 carriage",
                   notes="ball-screw elevator, rest = retracted; turret 0 = in, 1 = out; the elevator moves only after "
                         "the bay doors are fully open (turret 0.2 -> 1.0) and the doors close only after it is fully "
                         "retracted"))
    zs0 = z_bot(xt, 0.0) + 0.002
    zs1 = z_bot(xt, 0.100) + 0.002
    ad = unit([0.0, 0.100, zs1 - zs0])
    for sd, sgn in (("R", 1.0), ("L", -1.0)):
        J.append(joint(f"turret_door_{sd}", "prismatic", [xt, 0.0, zs0], ad * np.array([1, sgn, 1]), 0.0, 0.120,
                       prop="", scale=1.0, expr="0.12*clamp(turret/0.2,0,1)", side=sd, moves=f"YK250-PL-826-{sd}",
                       notes="sliding door on the inner face of the V belly, running in rails through a slot in the "
                             "lower edge of the bay wall to outboard of it; driven by its own DA 22 + rack "
                             "(EQ-TDOORACT): fully open (0.12 m along the skin each, clear of the E180 ring) before the elevator "
                             "starts, closed only after full retraction"))
    # ---------------------------------------------------------------- parachute hatch
    xh0 = PARA_X[0]
    J.append(joint("para_hatch", "revolute", [xh0, 0.0, z_top(xh0) - 0.002], [0.0, -1.0, 0.0], 0.0, 110.0 * DEG,
                   prop="para_hatch_deg", scale=DEG, moves="YK250-SH-367 hatch",
                   notes="dorsal hatch hinged at its forward edge (2 piano-hinge segments), opened by the deploying "
                         "canopy pack after the latch pin-puller (YK250-SY-804) retracts; tethered, never free"))
    # ---------------------------------------------------------------- propeller
    J.append(joint("prop_spin", "revolute", HUB, D_THRUST, 0.0, 2 * math.pi / 3, prop="prop_deg", scale=DEG,
                   moves="YK250-PR-540 propeller, YK250-PR-542 spinner",
                   notes="thrust axis (5 deg down-thrust: aft-up); one blade passage (3 blades) covers the swept "
                         "volume; rotation sense per the Limbach drawing (pusher propeller of matching hand)"))
    # ---------------------------------------------------------------- sequences (sampled joint states, rad / m)
    seq_g = []
    for k in range(21):
        g = k / 20.0
        cl = lambda v: min(max(v, 0.0), 1.0)                                    # noqa: E731
        leg = cl((g - 0.15) / 0.7)
        d_in = a_door * (cl(g / 0.15) - cl((g - 0.85) / 0.15))
        seq_g.append({"gear_up": r3(g, 3), "main_gear_R": r3(ang * leg, 5), "main_gear_L": r3(ang * leg, 5),
                      "main_inner_door_R": r3(d_in, 5), "main_inner_door_L": r3(d_in, 5),
                      "nose_gear": r3(an * leg, 5), "nose_steer": 0.0,
                      "nose_door_R": r3(90 * DEG * (1 - cl((g - 0.85) / 0.15)), 5),
                      "nose_door_L": r3(90 * DEG * (1 - cl((g - 0.85) / 0.15)), 5)})
    seq_t = []
    for k in range(13):
        t = k / 12.0
        dd = 0.12 * min(max(t / 0.2, 0.0), 1.0)
        ee = stroke * min(max((t - 0.2) / 0.8, 0.0), 1.0)
        seq_t.append({"turret": r3(t, 4), "turret_elevator": r3(ee, 5), "turret_door_R": r3(dd, 5),
                      "turret_door_L": r3(dd, 5)})
    props = {"gear_up": {"range": [0, 1], "unit": "-", "text": "0 = gear down and locked, 1 = up and locked; legs move "
                                                                "in 0.15-0.85, main inner doors open/close at the ends"},
             "turret": {"range": [0, 1], "unit": "-", "text": "0 = retracted (flush, doors closed); 0-0.2 doors open; "
                                                              "0.2-1 elevator extends 0.12 m"},
             "aileron_deg": {"range": [-20, 20], "unit": "deg", "text": "+ = roll right (starboard TE up)"},
             "flap_deg": {"range": [0, 40], "unit": "deg", "text": "take-off 35, landing 0"},
             "elevator_deg": {"range": [-20, 15], "unit": "deg", "text": "stabilator, + = TE down"},
             "rudder_deg": {"range": [-25, 25], "unit": "deg", "text": "+ = trailing edges to port"},
             "steer_deg": {"range": [-20, 20], "unit": "deg", "text": "nose-wheel steering, gear down only"},
             "para_hatch_deg": {"range": [0, 110], "unit": "deg", "text": "parachute hatch (deployment only)"},
             "prop_deg": {"range": [0, 120], "unit": "deg", "text": "propeller phase"}}
    return {"rules": "Joint = core.parts.Joint in the REST pose (all joints at rest = 0: gear down, turret in, "
                     "surfaces neutral, hatch closed). Producers register these names exactly (checks.py and "
                     "layout.clearances refer to them); a port joint is the mirrored starboard joint with the axis "
                     "chosen so that the same prop value gives the same physical motion. 'expr' is the Blender simple "
                     "expression of a coupled joint (control properties of YK250_Root); 'sequences' are the sampled "
                     "states the swept-interference check evaluates.",
            "controls": props, "joints": J,
            "sequences": {"gear_retraction": seq_g, "turret_extension": seq_t},
            "assembly_paths": [{"name": "outer_panel_insertion_R", "kind": "prismatic (assembly only, not a joint)",
                                "axis": None, "stroke": 0.25,
                                "text": "outer panel slides inboard along the main-spar line; tongue enters the fork "
                                        "and the drag pin its bushing (layout.chassis.wing_joint.insertion)"}]}


def engine_envelope() -> list:
    """Engine keep-out boxes in ENGINE axes (u = distance ahead of the propeller plane along the inclined thrust axis,
    v = lateral, w = engine vertical), from engine.envelope and the sizing zones; layout_check rotates them."""
    E = ENG["envelope"]
    h0 = HUB_FACE_AHEAD
    c0, c1 = (float(v) for v in E["cylinder_slab_from_flange"])
    cs = float(E["crank_axis_to_cylinder_side"])
    return [{"id": "crankcase_sg750", "u": r3([h0, SG750_FRONT_AHEAD]), "v": [-0.10, 0.10], "w": [-0.10, 0.05]},
            {"id": "cylinders_heads", "u": r3([h0 + c0, h0 + c1]), "v": r3([-E["width"] / 2, E["width"] / 2]),
             "w": [-cs, cs]},
            {"id": "intake_box", "u": r3([h0 + c0, h0 + c1]), "v": r3([-E["intake_box_width"] / 2,
                                                                    E["intake_box_width"] / 2]),
             "w": r3([cs - E["height"], -cs])},
            {"id": "prop_flange_hub_spacer", "u": [0.0, r3(h0)], "v": [-0.045, 0.045], "w": [-0.045, 0.045]}]


def keep_outs() -> list:
    R = 0.5 * float(PR["diameter"])
    ex_tip = float(PR["clearance_checks"]["blade_tip_axial_half_extent_m"])
    K = []
    K.append({"id": "KO-ENGINE", "kind": "engine dynamic envelope", "frame": "engine axes (u ahead of the propeller "
              "plane along the thrust axis, v lateral, w engine vertical); origin propeller.hub, axis inclined "
              "propeller.thrust_line_inclination_deg", "boxes": engine_envelope(),
              "margin": float(S["layout"]["clearances"]["engine_keep_out"])
              if isinstance(S["layout"]["clearances"], dict) else 0.010,
              "text": "L 275 EF + SG750 installed on isolators, crank axis = thrust axis (5 deg down-thrust, the "
                      "engine front sits ~34 mm lower than in the sizing boxes); 10 mm dynamic margin (isolator "
                      "travel); no part except the engine mount, isolators, exhaust, cooling baffles and the engine "
                      "harness enters it"})
    K.append({"id": "KO-PROP", "kind": "propeller disc + CS-VLA 925(c)", "centre": r3(HUB), "axis": r3(D_THRUST, 6),
              "radius": r3(R + float(PR["clearances"]["radial_tip_to_structure_min"])),
              "half_thickness": r3(ex_tip + float(PR["clearances"]["longitudinal_blade_to_structure_min"])),
              "hub_radius": r3(0.5 * float(PR["spinner"]["diameter"])),
              "text": "cylinder of radius R + 26 mm and axial half-thickness (blade tip half extent + 13 mm) about "
                      "the inclined disc; only the hub/spinner may enter; structure checked by sizing R-33/R-34 and "
                      "by checks.py (layout.clearances)"})
    K.append({"id": "KO-CYL-HOT", "kind": "hot zone (cylinder heads)", "of": "KO-ENGINE.cylinders_heads",
              "margin": 0.025, "text": "no composite or polymer part within 25 mm of the cylinder/head envelope "
                                       "(cooling baffles: high-temperature silicone/aluminium); CHT 240 degC design"})
    xe = 3.905
    for sd, sgn in (("R", 1), ("L", -1)):
        K.append({"id": f"KO-EXHAUST-{sd}", "kind": "exhaust hot zone", "mirror_of": "KO-EXHAUST-R" if sd == "L" else None,
                  "boxes": [r3([[3.800, sgn * 0.150, -0.030], [3.900, sgn * 0.215, 0.165]]) if sgn > 0 else
                            r3([[3.800, -0.215, -0.030], [3.900, -0.150, 0.165]]),
                            r3([[3.860, sgn * 0.140, -0.040], [3.960, sgn * 0.215, 0.030]]) if sgn > 0 else
                            r3([[3.860, -0.215, -0.040], [3.960, -0.140, 0.030]])],
                  "exit": {"point": r3([xe + 0.045, sgn * 0.205, -0.005]), "direction": r3(unit([1.0, sgn * 0.35, -0.30]))},
                  "plume_cone_half_angle_deg": 15.0, "plume_length": 0.30,
                  "margin_composite": 0.050, "margin_composite_shielded": 0.025,
                  "text": "routing envelope of the stack + silencer from the cylinder exhaust port (port location per "
                          "the Limbach installation drawing - open item) down along the outboard side of the "
                          "cylinder to the exit through the lower cowl side; 50 mm to composites, 25 mm behind a "
                          "stainless heat shield (layout.clearance_values.composite_to_exhaust); the plume cone "
                          "(15 deg, 0.30 m) must not touch the ventral fin, stabilators or the gear"})
    K.append({"id": "KO-TURRET-FOV", "kind": "turret field-of-regard cone", "apex": r3([X_TUR, 0.0,
              float(TU["ball_center_extended_z"])]), "min_clear_elevation_deg": -5.0,
              "text": "R-25: with the turret extended, every external protrusion (antennas, probes, lights, "
                      "fairings, the ventral fin) must lie above the -5 deg elevation cone from the ball centre at "
                      "every azimuth; checked by layout_check on the layout's external items (the OML, wing and "
                      "tail are checked by sizing.turret_checks)"})
    xd = [3.30, 3.40, 3.48, 3.52, 3.56, 3.60, X_FW - 0.0148 - 0.005]
    od = [0.060, 0.080, 0.094, 0.082, 0.070, 0.066, 0.066]   # under the FS3480 ring web, over the stabilator actuators
    K.append({"id": "KO-COOLING-DUCT", "kind": "cooling-air S-duct corridor",
              "path": [r3([x, 0.0, z_top(x) - o]) for x, o in zip(xd, od)], "radius": 0.040,
              "lateral_offsets": [-0.05, 0.0, 0.05],
              "text": "flush dorsal inlet (P-INLET) -> S-duct (YK250-PR-546) -> firewall duct cut-out C-DUCT -> plenum over "
                      "the cylinders; corridor of three 80 mm tubes side by side (about 0.18 x 0.08 m section); nothing "
                      "else may enter it"})
    K.append({"id": "KO-PARA-DEPLOY", "kind": "parachute deployment path", "box": r3([[PARA_X[0] - 0.02, -0.17, 0.15],
              [PARA_X[1] + 0.02, 0.17, 0.80]]), "text": "volume above the dorsal hatch: no antenna, light or probe; the bridle "
                                           "channel cover is the only item (tear-away)"})
    K.append({"id": "KO-WINGJOINT-PATH", "kind": "assembly path", "mirror": True,
              "box": r3([[spar_x(0.43, 0.25) - 0.022, 0.43, -0.031], [spar_x(0.96, 0.25) + 0.022, 0.96, 0.031]]),
              "text": "tongue insertion path (fork slot extended outboard over the stroke): nothing inside the fork "
                      "slot; the main pins are inserted after the panel is home"})
    K.append({"id": "KO-BATTERY-FUEL", "kind": "separation rule", "a": "EQ-BATTERY", "b": "fuel_cells",
              "min_distance": 1.0, "text": "Li-ion buffer battery >= 1 m from every fuel cell "
                                           "(engine.sources.buffer_battery)"})
    K.append({"id": "KO-FUEL-FIREWALL", "kind": "separation rule", "a": "fuel_cells", "b": "ST-FS3670",
              "min_distance": 0.013, "text": "CS-LUAS.967(c): fuel tanks >= 13 mm from the firewall, no tank on the "
                                            "engine side (standards.yaml fuel_system.firewall_clearance_m)"})
    return K


CLEARANCE_VALUES = {
    "prop_tip_to_structure_radial": 0.026, "prop_blade_to_structure_longitudinal": 0.013, "tyre_to_well": 0.012,
    "composite_to_exhaust": [0.025, 0.05], "engine_keep_out": 0.01, "composite_to_cylinder_heads": 0.025,
    "moving_surface_to_structure": 0.005, "rudder_root_to_cowl": 0.008, "turret_to_aperture_ring": 0.005,
    "turret_to_bay_wall": 0.006, "door_to_moving_gear": 0.010, "battery_to_fuel": 1.0, "fuel_to_firewall": 0.013,
    "harness_to_exhaust": 0.05, "harness_to_moving_parts": 0.010,
    "source": "standards.yaml#propeller_clearance (CS-VLA 925); baseline.yaml#engine.cooling; landing_gear.rules."
              "well_clearance; payload.turret.bay (aperture ring 5 mm, wall margin 6 mm); tail.surfaces.stabilator."
              "root_checks (R-38/R-39); CS-LUAS.967(c); engine.sources.buffer_battery; design rules (moving surfaces, "
              "doors, harness)"}

SEL_STRUCT = ["group:chassis", "group:shell", "group:fuel", "group:systems", "group:payload"]


def clearances() -> list:
    C = []

    def add(name, a, b, mm, joints=None, ref=""):
        d = {"name": name, "a": a, "b": b, "min_mm": mm}
        if joints:
            d["joints"] = joints
        if ref:
            d["ref"] = ref
        C.append(d)
    add("propeller blades vs fixed structure (CS-VLA 925(c)(2))", "joint:prop_spin",
        ["group:chassis", "group:shell", "group:tail", "group:wing", "group:gear"], 13.0, ["prop_spin"], "R-34")
    add("propeller tips vs fins and ventral (CS-VLA 925(c)(1))", "joint:prop_spin",
        ["YK250-TL-250-R", "YK250-TL-250-L", "YK250-TL-254", "YK250-TL-258-R", "YK250-TL-258-L", "YK250-TL-260"],
        26.0, ["prop_spin"], "R-33")
    add("main tyres vs well structure", ["YK250-LG-622-R", "YK250-LG-622-L"],
        ["group:chassis", "group:shell", "group:fuel", "YK250-LG-670-R", "YK250-LG-670-L"], 12.0,
        ["main_gear_R", "main_gear_L"], "landing_gear.rules.well_clearance")
    add("nose tyre vs keel slot", ["YK250-LG-626"], ["group:chassis", "group:shell", "group:systems",
                                                     "YK250-LG-674-R", "YK250-LG-674-L"], 12.0, ["nose_gear"])
    add("gear doors vs legs and wheels", ["YK250-LG-670-R", "YK250-LG-670-L", "YK250-LG-674-R", "YK250-LG-674-L"],
        ["YK250-LG-620-R", "YK250-LG-620-L", "YK250-LG-622-R", "YK250-LG-622-L", "YK250-LG-624", "YK250-LG-626"],
        10.0, ["main_inner_door_R", "main_inner_door_L", "nose_door_R", "nose_door_L"])
    add("engine (dynamic) vs airframe", ["YK250-PR-500"], SEL_STRUCT + ["group:tail"], 10.0, None,
        "layout.clearance_values.engine_keep_out")
    add("exhaust vs unshielded composites", ["YK250-PR-504-R", "YK250-PR-504-L"],
        ["group:tail", "group:wing", "group:fuel", "group:systems", "group:payload", "group:gear"], 50.0)
    add("exhaust vs shielded cowl and firewall", ["YK250-PR-504-R", "YK250-PR-504-L"],
        ["prefix:YK250-SH-45", "YK250-CH-013", "YK250-CH-014"], 25.0)
    add("cylinder heads vs composites", ["YK250-PR-500"], ["group:shell", "group:tail"], 25.0)
    add("stabilator vs fixed root stub", "joint:stabilator_R", ["YK250-TL-252-R"], 6.0, ["stabilator_R"], "R-38")
    add("stabilator vs fixed root stub (port)", "joint:stabilator_L", ["YK250-TL-252-L"], 6.0, ["stabilator_L"])
    add("stabilator vs body/cowl", "joint:stabilator_R", ["group:shell", "group:chassis"], 5.0, ["stabilator_R"],
        "R-39")
    add("stabilator vs body/cowl (port)", "joint:stabilator_L", ["group:shell", "group:chassis"], 5.0,
        ["stabilator_L"])
    add("rudder root vs cowl", "joint:rudder_R", ["group:shell", "group:chassis", "group:propulsion"], 8.0,
        ["rudder_R"])
    add("rudder root vs cowl (port)", "joint:rudder_L", ["group:shell", "group:chassis", "group:propulsion"], 8.0,
        ["rudder_L"])
    add("aileron vs wing structure", "joint:aileron_R", ["group:wing", "YK250-FC-202-R"], 3.0, ["aileron_R"])
    add("aileron vs wing structure (port)", "joint:aileron_L", ["group:wing", "YK250-FC-202-L"], 3.0, ["aileron_L"])
    add("flap vs wing structure and joint rib", "joint:flap_R", ["group:wing", "group:chassis", "group:shell"], 3.0,
        ["flap_R"])
    add("flap vs wing structure (port)", "joint:flap_L", ["group:wing", "group:chassis", "group:shell"], 3.0,
        ["flap_L"])
    add("turret vs aperture ring", "joint:turret_elevator", ["YK250-SH-363"], 5.0, ["turret_elevator"],
        "payload.turret.bay.aperture_ring")
    add("turret vs bay walls and frames", "joint:turret_elevator", ["YK250-CH-023-R", "YK250-CH-023-L",
                                                                    "YK250-CH-004", "YK250-CH-005"], 6.0,
        ["turret_elevator"], "payload.turret.bay.wall_margin")
    add("turret bay doors vs turret", ["YK250-PL-826-R", "YK250-PL-826-L"], ["YK250-PL-820", "YK250-PL-822"], 5.0,
        ["turret_door_R", "turret_door_L"])
    add("parachute hatch vs dorsal items", "joint:para_hatch", ["group:systems", "group:shell"], 10.0, ["para_hatch"])
    add("battery vs fuel cells", ["YK250-SY-726"], ["group:fuel"], 1000.0, None, "engine.sources.buffer_battery")
    add("fuel cells vs firewall (CS-LUAS.967(c))", ["YK250-FU-570", "YK250-FU-571", "YK250-FU-572"],
        ["YK250-CH-013"], 13.0)
    add("harness vs exhaust", ["prefix:YK250-SY-75"], ["YK250-PR-504-R", "YK250-PR-504-L"], 50.0)
    return C
