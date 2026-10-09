"""Layout builder: mechanisms (joints + sequences), keep-out envelopes, clearance values and clearance rules."""
from __future__ import annotations

from .b_common import *  # noqa: F401,F403

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
          notes="", side="C", mirror_of=None, hinge_parent=None, hinge_zone_m=None):
    d = {"name": name, "kind": kind, "origin": r3(origin, 5), "axis": r3(unit(axis), 6), "lo": r3(lo, 6),
         "hi": r3(hi, 6), "rest": r3(rest, 6), "prop": prop, "scale": r3(scale, 7)}
    if parent:
        d["parent"] = parent
    if hinge_parent:
        d["hinge_parent"] = hinge_parent          # the member carrying the hinge (no clearance within hinge_zone_m)
        d["hinge_zone_m"] = hinge_zone_m
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
    e0, e1 = rudder_span()
    p0, p1 = hinge("fin", e0, e1, float(rc["xc_hinge"]))
    a = unit(p1 - p0)                                       # up the span: + = TE to starboard
    lo, hi = (float(v) * DEG for v in rc["range_deg"])
    J.append(joint("rudder_R", "revolute", p0, a, lo, hi, prop="rudder_deg", scale=-DEG, side="R",
                   moves="YK250-FC-300-R", notes=f"fin span coordinate {e0:.3f}-{e1:.3f} m (tail.surfaces.fin."
                                                 "controls.rudder.eta0/eta1 x fin span: root clear of the cowl over "
                                                 "+-25 deg, layout_check C05; tip cap above); rudder_deg + = trailing "
                                                 "edges to port"))
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
    # fix round 2 (PK2-03): trunnion door per well, hinged on the lower edge of the gear beam (chine corner), closing the
    # leg-slot strip from the trimmed leg door (y 0.30) to the beam; opens outward-down before the leg moves and closes
    # after it (slaved to the inner-door DA 22 by a second crank / pushrod, EQ-DOORACT)
    from .b_chassis import Y_GB, T_SW as T_SW_GB
    y_th = round(Y_GB + T_SW_GB, 4)                    # outboard lower edge of the gear beam (skin corner)
    o_t = np.array([x_w, y_th, z_bot(float(MG["trunnion"][0]), y_th) + 0.002])
    a_tdoor = 125.0 * DEG
    J.append(joint("main_trunnion_door_R", "revolute", o_t, [1.0, 0.0, 0.0], 0.0, a_tdoor, prop="", scale=1.0,
                   expr=f"{a_tdoor:.5f}*(1-clamp((gear_up-0.85)/0.15,0,1))", side="R",
                   moves="YK250-LG-673-R", hinge_parent="M-GEARBEAM", hinge_zone_m=0.015,
                   notes="hinged on the lower edge of the gear beam (axis along x), + = free edge down and outboard; "
                         "open 125 deg whenever the gear is not up-locked (the down leg passes through its strip), "
                         "closed after the leg is up (gear_up 0.85 -> 1)"))
    J.append(joint("main_trunnion_door_L", "revolute", o_t * np.array([1, -1, 1]), [-1.0, 0.0, 0.0], 0.0, a_tdoor,
                   prop="", scale=1.0, expr=f"{a_tdoor:.5f}*(1-clamp((gear_up-0.85)/0.15,0,1))",
                   side="L", mirror_of="main_trunnion_door_R", moves="YK250-LG-673-L", hinge_parent="M-GEARBEAM",
                   hinge_zone_m=0.015))
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
                             "up-locked, closed for gear_up > 0.85 + 0.15; driven together with the other door by one "
                             "DA 22 (EQ-NDOORACT, centre-line bellcrank and links NDOOR-LINKAGE; fix round 2 PK2-13: a "
                             "leg-driven link cannot close the doors while the leg stands still)"))
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
    # ---------------------------------------------------------------- parachute hatch (tethered lift-off, VPK-11)
    xh = 0.5 * (PARA_X[0] + PARA_X[1])
    J.append(joint("para_hatch", "prismatic", [xh, 0.0, z_top(xh) - 0.002], [0.0, 0.0, 1.0], 0.0, 0.150,
                   prop="para_hatch", scale=0.150, moves="YK250-SH-367 hatch",
                   notes="fix round 1 (VPK-11): no hinge - the V-shaped hatch cannot carry a flush piano hinge on a "
                         "transverse axis; the pin-puller latch (YK250-SY-804) retracts the four corner pins, the "
                         "deploying canopy pack lifts the hatch straight off (0.15 m modelled lift, then free on its "
                         "1.5 m tether to FS1810); never free"))
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
                      "main_trunnion_door_R": r3(a_tdoor * (1 - cl((g - 0.85) / 0.15)), 5),
                      "main_trunnion_door_L": r3(a_tdoor * (1 - cl((g - 0.85) / 0.15)), 5),
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
                                                              f"0.2-1 elevator extends {stroke:.3f} m"},
             "aileron_deg": {"range": [-20, 20], "unit": "deg", "text": "+ = roll right (starboard TE up)"},
             "flap_deg": {"range": [0, 40], "unit": "deg", "text": "take-off 35, landing 0"},
             "elevator_deg": {"range": [-20, 15], "unit": "deg", "text": "stabilator, + = TE down"},
             "rudder_deg": {"range": [-25, 25], "unit": "deg", "text": "+ = trailing edges to port"},
             "steer_deg": {"range": [-20, 20], "unit": "deg", "text": "nose-wheel steering, gear down only"},
             "para_hatch": {"range": [0, 1], "unit": "-", "text": "parachute hatch lift-off (deployment only; 1 = "
                                                                  "0.15 m clear of the hatch seat)"},
             "prop_deg": {"range": [0, 120], "unit": "deg", "text": "propeller phase"}}
    return {"rules": "Joint = core.parts.Joint in the REST pose (all joints at rest = 0: gear down, turret in, "
                     "surfaces neutral, hatch closed). Producers register these names exactly (checks.py and "
                     "layout.clearances refer to them); a port joint is the mirrored starboard joint with the axis "
                     "chosen so that the same prop value gives the same physical motion. 'expr' is the Blender simple "
                     "expression of a coupled joint (control properties of YK250_Root); 'sequences' are the sampled "
                     "states the swept-interference check evaluates.",
            "controls": props, "joints": J,
            "sequences": {"gear_retraction": seq_g, "turret_extension": seq_t},
            "door_outlines": door_outlines(),
            "assembly_paths": assembly_paths()}


def rudder_span() -> tuple:
    """Rudder ends on the fin span coordinate (m): root reference + tail.surfaces.fin.controls.rudder.eta0/eta1 x fin
    span (the layout phase had the coordinates 0.30 / 0.88 = eta 0.200 / 0.844)."""
    rc = FIN["controls"]["rudder"]
    sp = float(FIN["params"]["span"])
    e0 = float(af.tail["fin"].span_coords()[0])
    return e0 + float(rc.get("eta0", (0.300 - e0) / sp)) * sp, e0 + float(rc.get("eta1", (0.880 - e0) / sp)) * sp


Y_LEGDOOR = 0.300          # outboard edge of the main leg door (fix round 2, PK2-03)


def door_outlines() -> dict:
    """Plan outlines (x, y; starboard, port mirrored) of the gear doors, the sliding turret doors and their rails (fix
    round 1, VPK-08/VPK-09): the shell cut-outs and layout_check use these, the gear / payload modules build the door
    panels from them."""
    from .b_chassis import Y_GB
    wb = ZP["main_gear_wells"]["box"]
    x0, x1 = float(wb[0][0]), float(wb[1][0])
    y_in = 0.5 * float(MG["well_gap"])
    y_out = float(LG["doors"]["main_inner_door_outer_edge_y"])
    xt0 = float(MG["trunnion"][0])
    slot = 0.5 * float(MG["leg_frontal_width"]) + 0.0225
    nw = ZP["nose_gear_well"]["box"]
    xs0, xs1, yh = float(nw[0][0]), float(nw[1][0]), float(nw[1][1])
    tb = ZP["turret_bay"]["box"]
    tx0, tx1 = float(tb[0][0]) + 0.003, float(tb[1][0]) - 0.003
    out = {
        "main_inner_door_R": {"joint": "main_inner_door_R", "thickness": 0.004,
                              "outline": r3([[x0, y_in], [x1, y_in], [x1, y_out], [x0, y_out]]),
                              "text": "inner door (well zone x range, from the inboard hinge edge to "
                                      "landing_gear.doors.main_inner_door_outer_edge_y)"},
        "main_leg_door_R": {"joint": "main_gear_R", "thickness": 0.004, "closed_at": "hi",
                            "outline": r3([[xt0 - 0.032, y_out], [xt0 + 0.032, y_out], [xt0 + 0.032, Y_LEGDOOR],
                                           [xt0 - 0.032, Y_LEGDOOR]]),
                            "text": "leg door carried by the leg on two standoff brackets (closed = gear up): closes the "
                                    "leg slot (64 mm, between the bearing lugs of F-TRUNNION) from the inner door edge to "
                                    f"y {Y_LEGDOOR} (fix round 2, PK2-03: trimmed so that its sweep stays outside the "
                                    "fitting and the beam)"},
        "main_trunnion_door_R": {"joint": "main_trunnion_door_R", "thickness": 0.004, "closed_at": "lo",
                                 "outline": r3([[xt0 - 0.032, Y_LEGDOOR], [xt0 + 0.032, Y_LEGDOOR],
                                                [xt0 + 0.032, Y_GB + 0.0068], [xt0 - 0.032, Y_GB + 0.0068]]),
                                 "text": "trunnion door hinged on the lower edge of the gear beam: closes the leg-slot "
                                         f"strip y {Y_LEGDOOR}-{Y_GB} when the gear is up (fix round 2, PK2-03)"},
        "nose_door_R": {"joint": "nose_door_R", "thickness": 0.004,
                        "outline": r3([[xs0, 0.0005], [xs1, 0.0005], [xs1, yh], [xs0, yh]]),
                        "text": "clamshell door hinged at the keel-slot edge (y = slot half width), meets its "
                                "partner on the centre line"},
        "turret_door_R": {"joint": "turret_door_R", "thickness": float(TU["bay"]["door_thickness"]),
                          "outline_closed": r3([[tx0, 0.0], [tx1, 0.0], [tx1, 0.105], [tx0, 0.105]]),
                          "travel_along_skin": 0.120,
                          "outline_open": r3([[tx0, 0.120], [tx1, 0.120], [tx1, 0.225], [tx0, 0.225]]),
                          "band": r3([[tx0, 0.0], [tx1, 0.0], [tx1, 0.235], [tx0, 0.235]]),
                          "rails": [{"id": f"RAIL-TD-{k}", "line": r3([[x_, 0.010, 0.0], [x_, 0.235, 0.0]]),
                                     "text": "door rail on the inner face of the belly skin (z follows the skin)"}
                                    for k, x_ in (("F", tx0 + 0.006), ("A", tx1 - 0.006))],
                          "text": "sliding door on the inner face of the V belly: closed y 0-0.105, open y 0.12-0.225 "
                                  "(arc length along the skin); the band it sweeps holds no removable cut-out other "
                                  "than the aperture ring P-TURRETRING, whose land and potted inserts stay within the "
                                  "skin thickness (layout_check C05)"}}
    return out


def assembly_paths() -> list:
    """Assembly / maintenance insertion paths with axis, stroke and envelope (fix round 1, VPK-02/VPK-09): layout_check
    C05 sweeps each envelope and checks it against structure and contents except the parts it engages ('engages')."""
    from .b_chassis import D_S, N_PIN, PIN_D, PRONG_T, Y_TONGUE_TIP, wing_joint
    wj = wing_joint()
    P_ = []

    def path(name, kind, start, axis, stroke, env, engages, text, mirror=False):
        d = {"name": name, "kind": kind, "start": r3(start), "axis": r3(unit(axis), 5), "stroke": r3(stroke),
             "envelope": env, "engages": list(engages), "text": text}
        if mirror:
            d["mirror"] = True
        P_.append(d)
    tip = np.array([spar_x(Y_TONGUE_TIP, 0.25), Y_TONGUE_TIP, float(wj["main_spar"]["pins"][0]["position"][2])])
    L_eng = (YJ - Y_TONGUE_TIP) / D_S[1]
    path("outer_panel_insertion_R", "prismatic (assembly only, not a joint)", tip + 0.012 * D_S, D_S,
         L_eng + float(wj["insertion"]["stroke"]) - 0.012, {"capsule_radius": 0.012},
         ["F-FORK", "M-CTBOX", "M-JOINTRIB", "M-GLOVERIB", "M-SOB"],
         "tongue core swept along the main-spar line from its seated position outboard over the engagement + the "
         "insertion stroke: inside the fork slot (KO-WINGJOINT-PATH); the outer panel slides inboard along -axis",
         mirror=True)
    rp = wj["rear_spar"]["pin"]
    path("rear_lug_insertion_R", "prismatic (with the outer panel)", np.asarray(rp["position"]) + 0.010 * D_S, D_S,
         0.050, {"capsule_radius": 0.006}, ["F-REARSLOT", "M-JOINTRIB"],
         "rear-spar lug bore centre swept outboard over the 44 mm lug engagement: inside the slot of F-REARSLOT",
         mirror=True)
    for pn in wj["main_spar"]["pins"]:
        c = np.asarray(pn["position"], float)
        L = float(pn["length"])
        grip = 2 * PRONG_T + 0.0304
        path(f"pin_{pn['id'][2:].lower()}_R", "pin insertion / withdrawal (by hand, ring handle)",
             c - (0.5 * grip + 0.003) * N_PIN, -N_PIN, grip + 0.006,
             {"capsule_radius": 0.5 * float(pn["head_diameter"]) + 0.002}, ["F-FORK"],
             "pin head corridor forward of the fork front face over the grip length + 6 mm (head d 24 + 2 mm): the "
             "withdrawn pin lies in the LERX bay and is taken out through P-JOINTACCESS", mirror=True)
        path(f"ream_{pn['id'][2:].lower()}_R", "line reaming (assembly jig)", c + 0.5 * L * N_PIN, -N_PIN,
             0.250, {"capsule_radius": 0.5 * PIN_D}, ["F-FORK"],
             "reamer shank through both prongs and the master tongue, withdrawn forward (step 5, before the glove "
             "skins)", mirror=True)
    path("rear_pin_R", "pin insertion (ball-lock, from below)", np.asarray(rp["position"]) - [0, 0, 0.015],
         [0.0, 0.0, -1.0], 0.025, {"capsule_radius": 0.007}, ["F-REARSLOT"],
         "ball-lock pin d 8 (head d 14) inserted upward through the access port P-REARACCESS", mirror=True)
    em = ENG["envelope"]
    path("engine_removal", "lift-off aft along the thrust axis (propeller, spinner, cowls off)", HUB, D_THRUST, 0.250,
         {"engine_envelope": "KO-ENGINE"}, ["ENGINE-MOUNT"],
         "engine off its 4 isolator bolts, moved aft along the crank axis by 0.25 m (mount truss stays on the "
         "firewall; fix round 2, PK2-11: 0.15 m left the cylinder heads under the fixed fin-root strips)")
    path("engine_lift", "lift up after the aft stroke (second leg of the engine removal)",
         HUB + 0.250 * D_THRUST, [0.0, 0.0, 1.0], 0.350, {"engine_envelope": "KO-ENGINE", "offset_axis": r3(D_THRUST),
                                                          "offset": 0.250}, ["ENGINE-MOUNT"],
         "after the 0.25 m aft stroke the engine is lifted 0.35 m clear of the fin-root strips and the fins")
    from .b_chassis import fittings as _fits
    fts = {f["id"]: f for f in _fits()}
    tr = fts["F-TRUNNION"]
    x0t, y0t, z0t = (float(v) for v in tr["pivot"])
    for sg, nm in ((-1.0, "fwd"), (1.0, "aft")):
        path(f"main_stub_axle_{nm}_R", "stub axle insertion from inside the leg yoke (gear down, maintenance mode)",
             [x0t + sg * 0.020, y0t, z0t], [sg, 0.0, 0.0], 0.030, {"capsule_radius": 0.010}, ["F-TRUNNION"],
             "flanged stub axle d 20 x 30 mm pushed outward from the yoke interior into the 20 H7 bushing of the "
             "lug (fix round 2, PK2-04: no through-pin)", mirror=True)
    npv = fts["F-NG-PIVOT"]
    xn, _, zn = (float(v) for v in npv["pivot"])
    for sg, nm in ((1.0, "R"), (-1.0, "L")):
        path(f"nose_stub_axle_{nm}", "stub axle insertion from inside the nose-leg yoke (gear down, doors open)",
             [xn, sg * 0.010, zn], [0.0, sg, 0.0], 0.0225, {"capsule_radius": 0.008}, ["F-NG-PIVOT"],
             "flanged stub axle d 16 x 22 mm pushed outward from the yoke interior into the bushing of the block (fix "
             "round 2, PK2-04: no through-pin)")
    path("turret_removal", "lowered through the bay opening (aperture ring off, doors open)",
         [X_TUR, 0.0, float(TU["ball_center_retracted_z"])], [0.0, 0.0, -1.0], 0.300,
         {"turret_envelope": "payload.turret.growth_envelope"}, ["TURRET", "EQ-TURRET", "EQ-ELEVATOR", "RAIL-FR",
                                                                 "RAIL-AL", "ELEV-SCREW"],
         "turret unbolted from its carriage (4 bolts) and lowered out of the bay")
    for nm, eid, ax_, stroke, eng, txt in (
            ("battery_removal", "EQ-BUFFER_BATTERY", [0, 0, 1], 0.150, ["EQ-FTS_UNIT"],
             "battery lifted out through P-FWDHATCH"),
            ("parachute_removal", "EQ-PARACHUTE", [0, 0, 1], 0.300, ["EQ-PARALATCH"],
             "container lifted out through the parachute hatch opening (313 x 312 mm between the frame and wall faces)"),
            ("mission_tray_removal", "EQ-MC", [0, 0, -1], 0.120, ["EQ-ECU"],
             "equipment tray TR-MISSION (mission computer + ECU) unscrewed and lowered through P-MBHATCH"),
            ("ecu_removal", "EQ-ECU", [0, 0, -1], 0.120, ["EQ-MC"], "with the tray, through P-MBHATCH")):
        path(nm, "item lift-out", [0.0, 0.0, 0.0], ax_, stroke, {"equipment": eid}, eng, txt)
    path("stabilator_removal_R", "slide outboard off the spindle (cross-bolt out)",
         [float(STAB["pivot"][0]), float(STAB["params"]["y_root"]), float(STAB["pivot"][2])], [0.0, 1.0, 0.0], 0.120,
         {"capsule_radius": 0.0125}, [], "root socket slides 0.10 m off the spline + 20 mm", mirror=True)
    path("propeller_removal", "off the hub along the thrust axis", HUB, D_THRUST, 0.100,
         {"capsule_radius": 0.06}, [], "propeller and spinner off the hub flange")
    return P_


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
              "margin": CLEARANCE_VALUES["engine_keep_out"],
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
    xd = [3.30, 3.40, 3.48, 3.52, 3.56, 3.60]
    od = [0.060, 0.080, 0.094, 0.082, 0.070, 0.066]          # under the FS3480 ring web, over the stabilator actuators
    K.append({"id": "KO-COOLING-DUCT", "kind": "cooling-air S-duct corridor",
              "path": [r3([x, 0.0, z_top(x) - o]) for x, o in zip(xd, od)], "radius": 0.040,
              "lateral_offsets": [-0.05, 0.0, 0.05],
              "exit_section": {"x": r3([3.600, X_FW - 0.0148 - 0.002]), "y": [-0.088, 0.088], "z": [0.240, 0.309],
                               "area_m2": 0.0118,
                               "text": "fix round 2 (PK2-02): the S-duct flattens over the last 55 mm to a rounded "
                                       "rectangle 176 x 69 mm (r 20 mm corners) through the firewall cut-out C-DUCT, "
                                       "keeping a continuous firewall rim above it; area 0.0118 m2 (three 80 mm tubes "
                                       "0.0151 m2; no duct-area requirement in the spec - cooling-flow check in the "
                                       "propulsion detail phase)"},
              "text": "flush dorsal inlet (P-INLET) -> S-duct (YK250-PR-546) -> firewall duct cut-out C-DUCT -> plenum over "
                      "the cylinders; corridor of three 80 mm tubes side by side (about 0.18 x 0.08 m section); nothing "
                      "else may enter it"})
    K.append({"id": "KO-PARA-DEPLOY", "kind": "parachute deployment path", "box": r3([[1.4616 - 0.02, -0.208, 0.10],
              [1.8384 + 0.02, 0.208, 0.80]]), "text": "volume above the 376 x 376 mm dorsal hatch (fix round 1): no "
                                                    "antenna, light or probe; the bridle cover strip is the only item "
                                                    "(tear-away)"})
    K.append({"id": "KO-WINGJOINT-PATH", "kind": "assembly path", "mirror": True,
              "box": r3([[spar_x(0.408, 0.25) - 0.023, 0.408, -0.031], [spar_x(1.00, 0.25) + 0.023, 1.00, 0.031]]),
              "text": "tongue insertion path (fork slot extended outboard over the 0.30 m stroke): nothing inside "
                      "the fork slot; the main pins and the rear pin are inserted after the panel is home "
                      "(layout.mechanisms.assembly_paths)"})
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
    add("parachute hatch (lift-off) vs dorsal items", "joint:para_hatch", ["group:systems", "group:shell"], 10.0,
        ["para_hatch"])
    add("battery vs fuel cells", ["YK250-SY-726"], ["group:fuel"], 1000.0, None, "engine.sources.buffer_battery")
    add("fuel cells vs firewall (CS-LUAS.967(c))", ["YK250-FU-570", "YK250-FU-571", "YK250-FU-572"],
        ["YK250-CH-013"], 13.0)
    add("harness vs exhaust", ["prefix:YK250-SY-75"], ["YK250-PR-504-R", "YK250-PR-504-L"], 50.0)
    return C


def sweep_keep_outs(L: dict) -> list:
    """Declarative swept volumes and corridors (gear, control surfaces, harness trunks, pushrods) for the detail
    modules; they reference the joints / trunks / equipment of the layout ``L`` and layout_check sweeps them (C05)
    and verifies the references and ranges (C12)."""
    J = {j["name"]: j for j in L["mechanisms"]["joints"]}
    cv = CLEARANCE_VALUES
    ty = LG["tyre"]

    def rng(name):
        j = J[name]
        if j["kind"] == "prismatic":
            return r3([float(j["lo"]), float(j["hi"])], 4)
        return r3([math.degrees(float(j["lo"])), math.degrees(float(j["hi"]))], 2)
    K = []
    K.append({"id": "KO-SWEEP-MAINGEAR", "kind": "swept volume (gear)", "mirror": True,
              "joints": {"main_gear_R": rng("main_gear_R"), "main_inner_door_R": rng("main_inner_door_R")},
              "sequence": "gear_retraction",
              "envelope": {"tyre": {"axle_static": r3(MG["axle_static"]), "diameter": float(ty["diameter"]),
                                    "width": float(ty["width"])},
                           "leg": {"from": r3(MG["trunnion"]), "to": r3(MG["axle_static"]),
                                   "radius": r3(0.5 * float(MG["leg_frontal_width"]))}},
              "margin": cv["tyre_to_well"],
              "text": "tyre + leg (+ the leg door carried by the leg) swept about the trunnion axis over the joint "
                      "range, and the inner door about its hinge (sequence gear_retraction); only the well walls, "
                      "the well roof and the gear beam bound it, nothing is mounted inside it; tyre to well "
                      ">= 12 mm"})
    K.append({"id": "KO-SWEEP-NOSEGEAR", "kind": "swept volume (gear)",
              "joints": {"nose_gear": rng("nose_gear"), "nose_steer": rng("nose_steer"),
                         "nose_door_R": rng("nose_door_R"), "nose_door_L": rng("nose_door_L")},
              "sequence": "gear_retraction",
              "envelope": {"tyre": {"axle_static": r3(NG["axle_static"]), "diameter": float(ty["diameter"]),
                                    "width": float(ty["width"]), "note": "nose tyre envelope taken as the main "
                                                                        "tyre (conservative until the unit is "
                                                                        "selected)"},
                           "leg": {"from": r3(NG["pivot"]), "to": r3(NG["axle_static"]),
                                   "radius": r3(0.5 * float(NG["leg_frontal_width"]))}},
              "margin": cv["tyre_to_well"],
              "text": "leg + wheel swept aft into the keel slot (steering centred before retraction; +-20 deg "
                      "steering only in the down position) and the clamshell doors about their keel-edge hinges; "
                      "the keel walls bound it"})
    surf = [n for n in ("aileron_R", "flap_R", "rudder_R", "stabilator_R") if n in J]
    K.append({"id": "KO-SWEEP-CONTROLS", "kind": "swept volume (control surfaces)", "mirror": True,
              "surfaces": [{"joint": n, "range_deg": rng(n), "moves": J[n].get("moves", "")} for n in surf],
              "margin": cv["moving_surface_to_structure"],
              "text": "each control surface (hinge line to trailing edge over its span; the stabilator as a whole "
                      "panel about the spindle axis) swept over its full joint range: only its hinge fittings, horn "
                      "and linkage enter the volume; 5 mm to every fixed part (rudder root vs cowl 8 mm, "
                      "stabilator root gap R-38)"})
    tr = [t["id"] for t in L["systems"]["harness"]["trunks"]]
    K.append({"id": "KO-CORRIDOR-HARNESS", "kind": "corridor (harness trunks)", "trunks": tr,
              "radius": "trunk diameter / 2 + radial_margin", "radial_margin": cv["harness_to_moving_parts"],
              "exhaust_margin": cv["harness_to_exhaust"],
              "text": "routing corridors of the harness trunks (layout.systems.harness.trunks: path + diameter): "
                      "the trunk passes every frame through its declared cut-out; no moving part within 10 mm, no "
                      "exhaust part within 50 mm; clamps at <= 150 mm on the frames / decks"})
    rods = []
    for e in L["systems"]["equipment"]:
        lk = e.get("linkage")
        if lk:
            rods.append({"equipment": e["id"], "from_axis": lk["servo_axis"], "to_axis": lk["horn_axis"],
                         "arm": float(lk["servo_arm_m"]), "horn": float(lk["horn_m"]), "through": "C-FW-PUSHROD",
                         "radius": 0.012})
    rods.append({"equipment": "EQ-DOORACT", "through": "C-DOORLINK", "radius": 0.010,
                 "text": "crank + pushrod from the DA 22 on the aft face of FS-GEAR to the inner-door hinge horn"})
    K.append({"id": "KO-CORRIDOR-PUSHRODS", "kind": "corridor (pushrods)", "mirror": True, "items": rods,
              "text": "pushrod corridors: rod + rod ends (d 8 mm rod, 12 mm swept radius over the actuator travel) "
                      "from the actuator arm to the horn; the frame crossings use the declared cut-outs (fireproof "
                      "bellows boots at the firewall); wing and tail actuator linkages stay inside the surfaces "
                      "(layout.systems.actuators)"})
    return K


HS_SHEET_T = 0.0004        # stainless 304 insert sheet (>= 0.38 mm fireproof without test, standards.yaml FIRE-001)
HS_FOIL_T = 0.0001         # stainless 304 heat-shield foil on stand-offs
HS_STANDOFF_KG_M2 = 0.30   # stand-offs + rivnuts of a foil shield (one per 100 x 100 mm, ~3 g each; estimate)
HS_LAP_W = 0.020           # riveted lap of an insert on the solid composite land
HS_RIVET = (0.025, 0.0005)  # rivet pitch (m) and mass (kg) of a 3.2 mm stainless blind rivet (estimate)
NODE_BAFFLE = (0.070, 0.060)  # stainless baffle between the cylinder heads and each node boss (m x m)


def heat_protection(K: list) -> dict:
    """Fix round 2 (PK2-09): heat protection of the engine bay (layout_check C08). Composite within the SHIELDED margin
    (25 mm) of an exhaust routing box is replaced by a riveted stainless insert; composite between the shielded and the
    unshielded margin (25-50 mm) carries a stainless foil heat shield on stand-offs; metal-only hardware inside the hot
    zones is listed with its temperature basis. Region boxes: each exhaust routing box grown by the margin (union of
    the boxes; a point outside a box grown by m is more than m away from it); starboard, port mirrored. The parts'
    areas and net masses are added by heat_protection_mass (fix round 2, mass closure)."""
    ex = next(k for k in K if k["id"] == "KO-EXHAUST-R")
    B = np.asarray(ex["boxes"], float)
    m_sh, m_un = float(ex["margin_composite_shielded"]), float(ex["margin_composite"])
    reg_in = [r3([(b[0] - m_sh).tolist(), (b[1] + m_sh).tolist()]) for b in B]
    reg_sh = [r3([(b[0] - m_un).tolist(), (b[1] + m_un).tolist()]) for b in B]
    t_in, t_f = HS_SHEET_T * 1000, HS_FOIL_T * 1000
    return {
        "inserts": [
            {"id": "HS-COWL-EXIT", "part": "YK250-PR-520", "panel": "P-COWL-LO", "mirror": True,
             "material": "ss_304_annealed", "t": HS_SHEET_T, "regions": reg_in,
             "text": f"stainless 304 exhaust-exit panel {t_in:.1f} mm (>= 0.38 mm, fireproof without test: standards.yaml "
                     "FIRE-001) riveted on a 20 mm lap into the lower cowl half: every part of the cowl within 25 mm of "
                     "the stack routing envelope (the boxes grown by 25 mm) is this insert; the stack passes through its "
                     "exit cut-out with a 5 mm stand-off ring"}],
        "shields": [
            {"id": "HS-COWL-SHIELD", "part": "YK250-PR-523", "panel": "P-COWL-LO", "mirror": True,
             "material": "ss_304_annealed", "t": HS_FOIL_T, "regions": reg_sh, "exclude_regions": reg_in,
             "text": f"stainless 304 foil {t_f:.1f} mm on 5 mm stand-offs (air gap) on the inner face of the CFRP lower "
                     "cowl between 25 and 50 mm from the stack routing envelope (around the insert): the composite "
                     "keeps the 25 mm shielded margin"},
            {"id": "HS-STUBROOT", "part": "YK250-PR-521", "panel": "P-STUBROOT", "mirror": True,
             "material": "ss_304_annealed", "t": HS_FOIL_T, "regions": reg_sh,
             "text": f"stainless 304 foil {t_f:.1f} mm on 5 mm stand-offs on the inner face of the lower edge of the "
                     "stub-root strip where it comes within 50 mm of the stack envelope (no part of the strip is "
                     "within 25 mm: shield, no insert)"},
            {"id": "HS-STUB", "part": "YK250-PR-522", "surface": "stabilator_stub", "mirror": True,
             "material": "ss_304_annealed", "t": HS_FOIL_T, "regions": reg_sh,
             "text": f"stainless 304 foil {t_f:.1f} mm on 5 mm stand-offs (air gap) over the lower / inboard stub skin "
                     "facing the stack: the CFRP stub keeps >= 25 mm (shielded margin) from the routing envelope"}],
        "hardware": [
            {"object": "F-SPINDLE-NODE", "basis": "machined 7075-T651 node (metal); inboard bearing 61805-ZZ (steel "
             f"shields, no elastomer seal) with high-temperature grease; a {t_in:.1f} mm stainless baffle "
             f"{NODE_BAFFLE[0] * 1000:.0f} x {NODE_BAFFLE[1] * 1000:.0f} mm between the cylinder heads and the node "
             "(cooling-baffle extension) faces the boss; the 7075 strength at the node temperature is an open item "
             "(structures T-NODE-* report the strength retention the margins need)",
             "baffle": {"material": "ss_304_annealed", "t": HS_SHEET_T, "size": list(NODE_BAFFLE), "mirror": True}},
            {"object": "STAB-HORN", "basis": "7075 horn on the Ti spindle, all-metal rod end (steel ball / steel race, "
             "no PTFE liner)"},
            {"object": "EQ-STABACT-PUSHROD", "basis": "7075 tube pushrod, all-metal rod ends; the firewall passage "
             "boot is a fireproof bellows (silicone-coated glass cloth, firewall side) - the boot lies forward of "
             "the 25 mm cylinder zone"}],
        "text": "C08 (fix round 2): composite shell panels and exposed tail lofts keep 25 mm from the cylinder-head "
                "envelope and 50 mm from the exhaust envelope (25 mm behind a heat shield); inserts replace the "
                "composite inside their regions; every other object inside the hot zones is listed under 'hardware'"}


def heat_protection_mass(hp: dict, areas: dict, L: dict) -> None:
    """Fix round 2 (mass closure): areas (one side, layout_check.heat_protection_areas) and NET masses (both sides, kg,
    before the growth allowance) of the heat-protection parts and of the metal cowl pieces, written into
    layout.heat_protection: insert = sheet (area + 20 mm lap on the perimeter 4 sqrt(A)) + rivets at 25 mm - the
    composite skin it replaces (sizing areal mass of the shell); shield = foil + stand-offs (on top of the composite);
    node baffles = sheet + 15 % fasteners; metal shell panels = sheet - the composite skin they replace. Read by
    sizing (item cooling_baffles_firewall_cowl_flap)."""
    rho = float(S["materials"]["ss_304_annealed"]["density"])
    am = Z.areal_masses(S)["shell"]
    tot, rows = 0.0, []
    for h in hp["inserts"]:
        A = float(areas.get(h["id"], 0.0))
        per = 4.0 * math.sqrt(max(A, 0.0))
        m1 = (A + per * HS_LAP_W) * float(h["t"]) * rho + per / HS_RIVET[0] * HS_RIVET[1] - A * am
        n = 2 if h.get("mirror") else 1
        h["area_m2"], h["mass_net_kg"] = r3(A, 5), r3(n * m1, 4)
        tot += n * m1
        rows.append(h["id"])
    for h in hp["shields"]:
        A = float(areas.get(h["id"], 0.0))
        m1 = A * (float(h["t"]) * rho + HS_STANDOFF_KG_M2)
        n = 2 if h.get("mirror") else 1
        h["area_m2"], h["mass_net_kg"] = r3(A, 5), r3(n * m1, 4)
        tot += n * m1
        rows.append(h["id"])
    for h in hp["hardware"]:
        bf = h.get("baffle")
        if bf:
            A = float(bf["size"][0]) * float(bf["size"][1])
            m1 = 1.15 * A * float(bf["t"]) * rho
            n = 2 if bf.get("mirror") else 1
            bf["mass_net_kg"] = r3(n * m1, 4)
            tot += n * m1
            rows.append(h["object"] + " baffle")
    metal = []
    for p in L["shell"]["panels"]:
        if p["id"] in areas and p.get("thickness_m"):
            A = float(areas[p["id"]])
            mat_ = S["materials"][p["material"]]
            n = 2 if p.get("mirror") else 1
            m1 = A * (float(p["thickness_m"]) * float(mat_["density"]) - am)
            metal.append({"panel": p["id"], "material": p["material"], "area_m2": r3(A, 5), "mass_net_kg": r3(n * m1, 4)})
            tot += n * m1
            rows.append(p["id"])
    hp["metal_panels"] = metal
    hp["mass"] = {"total_net_kg": r3(tot, 4), "items": rows,
                  "basis": "net masses of the parts above (both sides, before the growth allowance): inserts = stainless "
                           "sheet incl. a 20 mm riveted lap + blind rivets at 25 mm - the composite shell skin they "
                           f"replace ({am:.3f} kg/m2, sizing.areal_masses shell); foil shields + stand-offs "
                           f"{HS_STANDOFF_KG_M2:.2f} kg/m2 (estimate); node baffles + 15 % fasteners; metal cowl pieces - "
                           "the composite skin; added by sizing to cooling_baffles_firewall_cowl_flap (fix round 2)"}
