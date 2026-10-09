"""Layout builder: mass placement (layout-phase positions of the sizing mass items).

Each entry gives the components the layout places for a spec mass item (references to layout objects or explicit
points, with mass fractions) and the resulting centroid. ``mode: absolute`` -> sizing.mass_items uses the centroid;
``mode: offset`` (wing-/fin-attached items) -> sizing applies the offset to its own geometry-following position, so the
item keeps moving with the wing in the closure."""
from __future__ import annotations

from b_common import *  # noqa: F401,F403
from b_mech import engine_envelope


def _box_c(b, mirror=False):
    b = np.asarray(b, float)
    c = 0.5 * (b[0] + b[1])
    if mirror:
        c[1] = 0.0
    return c


def _path_len_c(path):
    P = np.asarray(path, float)
    seg = np.linalg.norm(np.diff(P, axis=0), axis=1)
    mid = 0.5 * (P[1:] + P[:-1])
    return float(seg.sum()), (seg[:, None] * mid).sum(0) / max(seg.sum(), 1e-12)


def _item(S_, name):
    return next(i for i in S_["mass"]["items"] if i["name"] == name)


def placements(L: dict, S_: dict) -> dict:
    ch = {m["id"]: m for m in L["chassis"]["members"]}
    eqs = {e["id"]: e for e in L["systems"]["equipment"]}
    fit = {f["id"]: f for f in L["chassis"]["fittings"]}
    out = {}

    def put(name, comps, mode="absolute", basis=""):
        m = np.array([c[1] for c in comps], float)
        P = np.array([c[2] for c in comps], float)
        cen = (m[:, None] * P).sum(0) / m.sum()
        d = {"mode": mode, "position": r3(cen), "basis": basis,
             "components": [{"what": c[0], "fraction": r3(c[1] / m.sum(), 4), "at": r3(c[2])} for c in comps]}
        out[name] = d
    # engine group (engine.installed_items_kg at their installed places; engine CG = crankcase centre, estimate)
    E = ENG["installed_items_kg"]
    env = {b["id"]: b for b in engine_envelope()}
    cc = env["crankcase_sg750"]
    u_eng = 0.5 * (HUB_FACE_AHEAD + float(cc["u"][1])) - 0.03
    put("engine_group_installed", [
        ("engine_bare (crankcase/cylinders centre, estimate)", E["engine_bare"], engine_point(u_eng)),
        ("ECU + harness + coils (aft equipment bay)", E["ecu_harness_ignition_coils"], _box_c(eqs["EQ-ECU"]["box"])),
        ("fuel pump/regulator/filter (aft equipment bay)", E["fuel_pump_regulator_filter"],
         _box_c(eqs["EQ-FUELPUMP"]["box"])),
        ("SG750 (front of the crankcase)", E["starter_generator_sg750"], engine_point(SG750_FRONT_AHEAD - 0.0143)),
        ("generator power electronics (port avionics side bay)", E["generator_power_electronics"],
         _box_c(eqs["EQ-GENERATOR_PE"]["box"])),
        ("generator adapter/coupling", E["generator_adapter_coupling"], engine_point(MOUNT_FACE_AHEAD + 0.02)),
        ("exhaust stacks + silencers (both sides)", E["exhaust_with_silencers"], [3.880, 0.0, 0.060]),
        ("mount isolators + bolts", E["engine_mount_isolators_bolts"], engine_point(MOUNT_FACE_AHEAD + 0.01)),
        ("throttle servo + CHT/EGT sensors", E["throttle_servo_cht_egt_sensors"], engine_point(0.24, 0.0, -0.10))],
        basis="engine.installed_items_kg at the layout positions (ECU and pump forward of the firewall in the aft "
              "equipment bay, PE in the port avionics side bay, zones_preliminary.equipment_bay_aft); crank axis inclined 5 deg")
    put("cooling_baffles_firewall_cowl_flap", [
        ("firewall stainless shield + edge angle (section centroid)", 0.8, [X_FW + 0.005, 0.0, 0.125]),
        ("cylinder baffles + plenum", 0.6, engine_point(HUB_FACE_AHEAD + 0.115, 0.0, 0.03)),
        ("cowl-flap / exit lip parts", 0.1, [3.98, 0.0, 0.22])],
        basis="split of the 1.5 kg allowance (estimate): firewall shield 0.5 mm 304 sheet ~0.8 kg, baffles/plenum "
              "0.6, exit lip 0.1")
    put("dorsal_cooling_inlet_s_duct", [("inlet lip", 0.3, [3.31, 0.0, 0.285]), ("S-duct", 0.7, [3.55, 0.0, 0.27])],
        basis="flush dorsal inlet x 3.18-3.44 + S-duct to the firewall duct cut-out")
    sa = eqs["EQ-STABACT-R"]["box"]
    put("actuators_stabilators_2x_DA30", [("2 x DA 30 on the firewall forward face", 1.26, _box_c(sa, True)),
                                          ("pushrods + spindle horns", 0.22, [3.69, 0.0, 0.218])],
        basis="DA 30 lying along y on the cool side of the firewall, pushrods through fireproof boots")
    nose_st = [float(NG["retraction"]["stowed_wheel_center"][0]) - 0.13, 0.0, -0.08]
    put("actuators_nose_steering_brake_2x_DA26", [("steering DA 26 on the nose leg (retracted)", 0.32, nose_st),
                                                  ("brake DA 26 + master cylinder (port avionics side bay)", 0.32,
                                                   _box_c(eqs["EQ-BRAKE_UNIT"]["box"]))],
        basis="steering actuator travels with the leg (flight CG = retracted); brake actuator in the aft bay")
    # harness: cables (share 0.70, estimate) by trunk length x diameter^2 + both outer-panel harnesses to the tips;
    # connectors / backshells / coax terminations (share 0.30, estimate) equally over the connection points
    cab = []
    for t in L["systems"]["harness"]["trunks"]:
        ln, c = _path_len_c(t["path"])
        w = ln * float(t["diameter"]) ** 2
        if t.get("mirror"):
            cab.append((t["id"] + " (x2)", 2 * w, [c[0], 0.0, c[2]]))
        else:
            cab.append((t["id"], w, c.tolist()))
    b2 = 0.5 * float(S_["wing"]["span"])
    w_wing = 2 * (b2 - YJ) * 0.012 ** 2
    pw = wing_pt(0.5 * (YJ + b2), 0.45)
    cab.append(("outer-panel harnesses (x2, to the tips)", w_wing, [float(pw[0]), 0.0, float(pw[2])]))
    pts = []
    for e in L["systems"]["equipment"]:
        pts += [_box_c(e["box"])] * (2 if e.get("mirror") else 1)
    for a in L["systems"]["actuators"]:
        pts += [_box_c(a["box"])] * (2 if a.get("mirror") else 1)
    for a in L["systems"]["antennas"]:
        pts.append(np.asarray(a["point"], float))
    for a in L["systems"]["air_data_lights"]:
        pts.append(_box_c(a["box"]) if "box" in a else 0.5 * (np.asarray(a["p0"], float) + np.asarray(a["p1"], float)))
    pts += [np.asarray(MG["trunnion"], float)] * 2 + [np.asarray(NG["pivot"], float)] * 2   # gear EMAs + steering
    wc = sum(c[1] for c in cab)
    comps = [(c[0], 0.70 * c[1] / wc, c[2]) for c in cab]
    comps.append((f"connectors at {len(pts)} connection points", 0.30,
                  np.mean([[p[0], 0.0, p[2]] for p in pts], axis=0).tolist()))
    put("wiring_harness_connectors_coax", comps, basis="cables 70 % (estimate) by trunk length x diameter^2 "
                                                       "(layout.systems.harness) + outer-panel harnesses; connectors, "
                                                       "backshells and coax terminations 30 % (estimate) equally over "
                                                       "the connection points of the layout (equipment, actuators, "
                                                       "antennas, lights, gear EMAs)")
    lt = {a["id"]: a for a in L["systems"]["air_data_lights"]}
    put("flight_termination_lights", [("FTS unit (forward bay)", 0.15, _box_c(eqs["EQ-FTS_UNIT"]["box"])),
                                      ("2 wing-tip lights", 0.166, [_box_c(lt["LT-WING-R"]["box"])[0], 0.0,
                                                                    _box_c(lt["LT-WING-R"]["box"])[2]]),
                                      ("tail light (fin tip)", 0.083, _box_c(lt["LT-TAIL"]["box"]))],
        basis="FTS 0.15 + 3 x AveoFlash 0.083 at their installed places")
    comps = []
    for mid in ("M-CHINE", "M-KEELWALL", "M-KEEL", "M-DORSAL", "M-AFTKEEL"):
        m = ch[mid]
        k = 2.0 if m.get("mirror") else 1.0
        if "paths" in m:
            for p in m["paths"]:
                ln, c = _path_len_c(p)
                comps.append((mid, k * ln, [c[0], 0.0, c[2]]))
        else:
            b = np.asarray(m["box"], float)
            comps.append((mid, k * (b[1][0] - b[0][0]), _box_c(b, True).tolist()))
    put("keel_beams_longerons", comps, basis="length-weighted centroid of the longitudinal members (chine longerons, "
                                             "keel walls, keel beams, dorsal longerons, aft keel)")
    comps = []
    for s_ in L["stations"]:
        x = float(s_["x"])
        yy, zz = np.meshgrid(np.arange(-0.45, 0.45, 0.005), np.arange(-0.25, 0.40, 0.005))
        P = np.column_stack([np.full(yy.size, x), yy.ravel(), zz.ravel()])
        ins = af.inside(P, margin=0.006)
        if s_["type"] == "ring":
            ins &= ~af.inside(P, margin=0.006 + float(s_.get("ring_depth", 0.04)))
        for c in s_.get("cutouts", []):
            for sg in ((1.0, -1.0) if c.get("mirror") else (1.0,)):
                y0, y1 = sorted([sg * c["y"][0], sg * c["y"][1]])
                ins &= ~((P[:, 1] >= y0) & (P[:, 1] <= y1) & (P[:, 2] >= c["z"][0]) & (P[:, 2] <= c["z"][1]))
        if s_.get("subtype") == "spar frame" or s_["id"] in ("FS-MS", "FS-RS"):
            ins &= ~((P[:, 2] >= Z_BOX[0]) & (P[:, 2] <= Z_BOX[1]))           # the box webs are YK250-CH-001
        a = float(ins.sum()) * 0.005 ** 2
        zc_ = float(P[ins, 2].mean()) if ins.any() else 0.0
        rho = 2700 if s_["material"].startswith("al_") else 400
        comps.append((s_["id"], max(a, 1e-4) * float(s_.get("t", 0.0068)) * rho, [x, 0.0, zc_]))
    put("frames_bulkheads", comps, basis="13 stations, mass ~ net web area (section inside the 6 mm skin inset minus "
                                         "the declared cut-outs and, for the spar frames, the box) x thickness x "
                                         "effective density (sandwich ~400 kg/m3, aluminium ring 2700 kg/m3; estimate)")
    put("engine_mount_4130", [("4130 truss + isolator ring", 1.0, [0.5 * (X_FW + 0.0074 + float(
        engine_point(MOUNT_FACE_AHEAD)[0])), 0.0, 0.21])], basis="truss between the firewall attach fittings and the "
                                                                  "crankcase mount face")
    put("parachute_attach_fitting", [("forward bridle fitting FS1810", 0.4, fit["F-RISER-FWD"]["point"]),
                                     ("aft bridle fitting rear-spar frame", 0.4, fit["F-RISER-AFT"]["point"]),
                                     ("container restraint brackets", 0.2, [0.5 * (PARA_X[0] + PARA_X[1]), 0.0, -0.03])],
        basis="two bridle fittings (Y-bridle) + container restraint")
    comps = []
    for mid in ("M-DECK-NOSE", "M-MIDFLOOR", "M-FWDDECK", "M-TURRETROOF", "M-PARAFLOOR"):
        b = np.asarray(ch[mid]["box"], float)
        comps.append((mid, (b[1][0] - b[0][0]) * (b[1][1] - b[0][1]), _box_c(b, True).tolist()))
    comps.append(("payload-tray rails", 0.06, [0.5 * (X_PB0 + X_PB1), 0.0, -0.065]))
    comps.append(("aft equipment tray", 0.06, [3.33, 0.0, -0.045]))
    comps.append(("side-bay trays", 0.04, [0.98, 0.0, -0.09]))
    put("floors_trays_rails", comps, basis="area-weighted decks/floors + trays")
    comps = []
    for p in L["shell"]["panels"]:
        if p["attach"] in ("removable", "hinged"):
            x0, x1 = p["x"]
            y0, y1 = p["y"]
            per = 2 * ((x1 - x0) + (y1 - y0)) * (2.0 if p.get("mirror") else 1.0)
            comps.append((p["id"], per, [0.5 * (x0 + x1), 0.0, 0.0]))
    put("hatch_frames_quick_access_fasteners", comps, basis="perimeter-weighted removable panels/hatches (frame lands, "
                                                           "Camlocs, nutplates)")
    comps = []
    for f in L["chassis"]["fittings"]:
        if f["id"].startswith(("F-FIN", "F-STUB", "F-VENTRAL")):
            p = np.asarray(f["point"], float)
            comps.append((f["id"], 2.0 if f.get("mirror") else 1.0, [p[0], 0.0, p[2]]))
    put("fin_ventral_root_fittings", comps, basis="fin, stub and ventral root fittings at the frames")
    return out
