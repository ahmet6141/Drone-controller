"""YK-250 HANCER structures: documented hand calculations on the spec geometry and layout (CS-LUAS / STANAG 4703).

    python3 -m ucav250.analysis.structures                 compute, write out/structures.md + .json
    python3 -m ucav250.analysis.structures --check         exit 1 if any margin of safety < 0, a sized dimension
                                                           is not in spec.yaml (structures.sizing) or is stale
    python3 -m ucav250.analysis.structures --update-spec   write the sized dimensions (structures.sizing) to spec.yaml

Method (docs/04_yapi_hesaplari.md): every check is a closed-form hand calculation (analysis/structlib.py) on the
spec geometry: wing sections from the loft (wing.sections), spar lines of the reference trapezoid, layout members,
fittings, stations and mechanisms (spec.layout), masses from spec.mass, materials / layups from spec.materials /
spec.layups, factors from spec.structures (baseline.yaml#design_loads, standards.yaml#design_loads_recommended).

Margin of safety: MS = allowable / (applied x total factor) - 1, where ``applied`` is the LIMIT load (or stress /
strain) of the load case and ``allowable`` the ULTIMATE design allowable; ultimate-only cases (emergency landing,
parachute opening, crash retention, reserve energy) carry the ultimate load as ``applied`` and no factor of safety.
Total factor (CS-LUAS.619, standards.yaml combination_rule): FoS 1.5 x max(applicable special factors): fitting 1.15
(CS-LUAS.625), bearing 2.0 for rotating / pinned joints (STANAG UL2.4), frequently-assembled attachments 1.5
(STANAG UL2.4), composite special factor 1.2 with B-basis ETW allowables (AMC LUAS.619; one of the special factors:
the composite factors themselves - hot/wet 1.0 x manufacturing 1.2 - are already multiplied into it); control-surface
hinge bearings 6.67 total (CS-LUAS.657), push-pull joints 3.33 total (CS-LUAS.693). Damage-tolerance strain limits
(STANAG UL13.1.2) are applied at ultimate load (FoS 1.5 only), as in sizing.wing_structure. Single-load-path
composite parts use 0.85 x the B-basis value as the A-basis estimate (CS-LUAS.613(b), baseline.yaml).

Ownership: this module owns ``spec.structures.sizing`` (the sized dimensions the detail modules build from); the
authored design values live in DESIGN below, the ply counts of the wing spar caps / webs are sized here from the
loads (zones and widths authored). Workflow after a change: ``structures --update-spec`` -> ``layout_build --check``
-> ``sizing --check`` (and ``--update-spec`` if a mass item changes) -> ``layout_check --check``."""
from __future__ import annotations

import argparse
import copy
import json
import math
import re
import sys
import time
from pathlib import Path

import numpy as np

from ..core import spec as SPEC
from ..design import oml
from . import aero as AE
from . import sizing as Z
from . import structlib as ST

G = 9.80665
RHO0 = 1.225
OUT_DIR = SPEC.OUT_DIR
FUEL_RHO = 740.0

# =====================================================================================================================
# authored design values (written to spec.structures.sizing by --update-spec; the analysis reads them from the spec)
# =====================================================================================================================
DESIGN = {
    "note": ("sized dimensions of the structures phase (ucav250.analysis.structures, docs/04_yapi_hesaplari.md); "
             "authored here and checked against the load cases; the wing spar-cap and web ply counts are sized from "
             "the loads per authored zone. Lengths in m, ply counts of the spec ply thickness."),
    "wing": {
        "inertia_relief": True,
        "inertia_relief_basis": "wing structure + control surfaces (distributed with the chord outboard of the SOB rib) "
                                "and the aileron / flap actuators (point masses) at the same load factor as the lift "
                                "(standard practice; the sizing-phase wing model omitted it); fix round 2 (VS2-05): "
                                "minimum credible wing mass = item base masses without the growth allowance and "
                                "without the 1.05 model factor",
        "skin_layup": "wing_skin_primary",
        "box_skin_upper_layup": "wing_box_skin_upper",
        "box_skin_upper_y_end_m": 2.2,
        "lerx_upper_first_bay_layup": "lerx_skin_upper_root",
        "lerx_upper_first_bay_text": "fix round 3 (VS3-04): the glove LERX upper skin between the side-of-body rib and the "
                                     "glove rib (no LERX nose ribs: pin, puller and reamer corridors) uses a 7 mm "
                                     "core (layups.lerx_skin_upper_root; panel buckling of the 0.15 m bay under the "
                                     "plane-section box strain, W-SKINBUCK-LERX-UP); OML unchanged, the inner face "
                                     "steps 2 mm",
        "skin_solid_over_caps_m": 0.001,
        "main_cap": {"material": "cfrp_ud_mtm45_as4", "width_body_glove_m": 0.040, "width_outer_m": 0.030,
                     # fix round 3 (VS3-01 + mass closure): breaks at the SOB cap ramp end (inner pin, y 0.463),
                     # 0.10 m ply-drop zones inside the CT box (outside the kink-fitting bond |y| <= 0.05), 0.05 m
                     # zones on the outer panel to y 2.10 (internal 1:20 ply drops, model factor 1.10 kept)
                     "zone_breaks_y_m": [0.0, 0.10, 0.20, 0.30, 0.40, 0.463, 0.50, 0.55, 0.625, 0.70, 0.75, 0.80,
                                         0.85, 0.90, 0.95, 1.00, 1.05, 1.10, 1.15, 1.20, 1.25, 1.30, 1.35, 1.40,
                                         1.45, 1.50, 1.55, 1.60, 1.65, 1.70, 1.75, 1.80, 1.85, 1.90, 1.95, 2.00,
                                         2.05, 2.10, 2.20, 3.60],
                     "min_plies": 10,
                     "interleaf": "none: no fastener passes through a main-spar cap (fix round 1, S1-05): the outer-panel "
                                  "joint is a pinned CFRP tongue / fork whose bores are in solid [+-45/0/90] boss blocks "
                                  "(>= 40 % +-45, baseline.yaml processes.laminate_stacking) between the UD flanges, and "
                                  "the CT box is attached through the frame lands; the caps are pure UD and the UD "
                                  "material carries no bearing allowable",
                     "fork_zone": "glove caps y 0.40-0.70 sized for the glove-box moment: the total moment minus the "
                                  "tongue moment (the tongue carries the joint moment to the pins; between the pins it "
                                  "falls linearly to zero at the inner pin)"},
        "sob_transition": {
            "rib_land_plies": 12, "land_length_m": 0.050, "intro_length_m": 0.030, "filler": "core_rohacell_71wf",
            "rib_face_doubler_plies": 1, "rib_doubler_band_m": 0.030,
            "text": "fix round 3 (VS3-01): the main and rear caps leave the box-cover level (z +-0.0391, layout M-CTBOX.z) "
                    "at the side-of-body rib and reach the glove loft at the inner-pin station (layout M-CTBOX."
                    "sob_transition, linear, no ply drop on the ramp); the reduced lever arm along the ramp sizes the "
                    "cap zone y 0.40-0.463; the kink forces F_cap sin(theta) go into a 12-ply solid land of the SOB rib "
                    "(50 mm long, under each cap) inboard and into the 10 mm bush pads of the prongs outboard; the glove "
                    "skins stay on the loft and are bonded to the ramped caps through a tapered ROHACELL 71 WF filler; "
                    "the glove box skins end on the SOB rib, whose sandwich web carries the skin-to-cover offset couple "
                    "(W-SOB-OFFSET) with one +-45 PW ply added on each face over a 30 mm band along its upper and "
                    "lower edges between the spars (the couple falls linearly to zero at mid-depth)"},
        "rear_cap": {"material": "cfrp_ud_mtm45_as4", "width_m": 0.025, "plies": 2,
                     "text": "UD strip of 2 plies under the rear-spar web flanges (bending is carried by the main caps "
                             "at the deeper main-spar station: lighter than thicker rear caps)"},
        "main_web": {"material": "cfrp_pw_mtm45_as4", "orientation_deg": 45.0,
                     "zone_breaks_y_m": [0.70, 0.85, 1.00, 1.15, 1.30, 1.45, 1.70, 1.95, 2.20, 2.45, 2.70, 2.95,
                                         3.60], "min_plies": 3,
                     "stiffening": "none between the ribs: the web panels are long plates with the web depth as the "
                                   "panel width (rib pitch > 3 x depth)"},
        "rear_web": {"material": "cfrp_pw_mtm45_as4", "orientation_deg": 45.0, "plies": 3},
        "ct_box": {"cover_layup": "ct_box_cover",
                   "covers": "sandwich covers layups.ct_box_cover (0.4/6/0.4 mm) continuing the glove skins "
                             "across the body between the spar frames, bonded to the caps (replaces the 2.0 mm "
                             "monolithic covers of the layout phase: a 0.31 m wide 2 mm cover buckles at about 7 MPa "
                             "and cannot carry the bending compression; the glove-skin load needs a continuous path "
                             "across the body); a centre-line rib (rib_panel, solid lands) at y = 0 with the kink "
                             "fitting reacts the chevron kink couple of the main caps into the two spar webs",
                   "web": "frames FS-MS / FS-RS are the box webs inside the body (rib_panel sandwich)",
                   "kink_fitting": {"material": "al_7075_t651_plate", "plate_t_m": 0.0020, "w_m": 0.040,
                                    "length_m": 0.10, "tab_h_m": 0.030, "tab_t_m": 0.003, "bolts": 5,
                                    "bolt_d_m": 0.006, "rib_land_plies": 16,
                                    "text": "fix round 2 (VS2-03): one machined 7075 chevron fitting per main cap "
                                            "(upper and lower) at y = 0: 2.0 mm plate 40 x 100 mm bonded over the cap "
                                            "kink (no fastener through the cap) with a 30 mm tab bolted 5 x M6 Ti "
                                            "through the 16-ply solid land of the centre-line rib; the opposite kink "
                                            "forces of the upper and lower caps form a couple carried by the rib in "
                                            "its plane to FS-MS / FS-RS"},
                   "web_doubler_plies_per_face": {"FS-MS": 1, "FS-RS": 0},
                   "web_doubler": "+-45 PW ply added on both faces of the main-spar frame web FS-MS between the caps "
                                  "(y -0.40..0.40); none on FS-RS"},
    },
    "wing_joint": {
        "concept": ("composite spar-stub joint (sailplane practice; fix round 1, S1-02 / S1-03 / S1-05): the CFRP tongue "
                    "of the outer panel (continuation of its main-spar caps) in the CFRP fork of the glove main-spar box "
                    "on two chordwise Ti pins in bonded 4130 bushes (vertical couple of the bending moment); rear-spar "
                    "7075 lug in a slot fitting on a vertical Ti pin (chordwise force + spanwise couple of the drag "
                    "moment with the main pins; vertical torsion couple on the slot plates); no bonded or bolted spar-cap "
                    "transfer, no fastener through a spar cap"),
        "tongue": {"flange_material": "cfrp_ud_mtm45_as4", "web_material": "cfrp_pw_mtm45_as4", "width_m": 0.030,
                   "height_m": 0.061, "flange_t_m": 0.010, "web_plies": 25, "boss_length_m": 0.050,
                   "buildup_length_m": 0.10,
                   "flange_taper": {"t_min_m": 0.003,
                                    "text": "fix round 2 (mass closure): the tongue moment falls linearly from the outer "
                                            "pin to zero at the inner pin, so each UD flange drops plies linearly from "
                                            "10 mm at the outer pin to 3 mm at the inner pin (internal ply drops on the "
                                            "web side, outer flange faces stay on the 61 mm envelope) and stays 3 mm "
                                            "to the tip; checked at the taper stations (J-TONGUE-TAPER) and against the "
                                            "1:20 ply-drop rule"},
                   "text": "CFRP spar stub 30 x 61 mm: UD flanges 30 x 10 mm from the root rib to the outer pin (the "
                           "outer-panel main caps built up 1:20 over 0.10 m from the root rib), tapered to 3 mm at the "
                           "inner pin, +-45 PW web 25 plies (5.0 mm), solid [+-45/0/90] boss blocks 30 x 41 x 50 mm "
                           "(>= 40 % +-45) around the two pin bores with bonded 4130 bushes"},
        "transition": {"length_m": 0.10, "intro_length_m": 0.030, "root_rib_land_plies": 20,
                       "root_bay_skin_doubler_plies_per_face": 1, "root_bay_doubler_faces": "outer",
                       "root_bay_doubler_length_m": 0.12,
                       "text": "fix round 2 (VS2-04): depth transition from the outer-panel main caps (loft-following, "
                               "under the skins) to the tongue flange line in the root bay of the outer panel: the caps "
                               "ramp linearly over the 0.10 m cap build-up zone from the joint rib (y_junction) outboard "
                               "(one slope per flange, kinks at both ends: at the joint rib into its 20-ply solid land, "
                               "at the outboard end into the 25-ply tongue web); the web depth follows the caps; the "
                               "skins stay on the loft (reduced cap lever arm checked with the skins in the section) and "
                               "get one extra 0/90 PW ply in the OUTER face between the spars over the first 0.12 m "
                               "of the outer panel (root-bay skin doubler: the 0.10 m ramp + a 20 mm ply drop-off)"},
        "fork": {"material": "cfrp_pw_mtm45_as4", "prong_plies": 8, "prong_pad_t_m": 0.010, "pad_length_m": 0.050,
                 "cap_width_m": 0.052,
                 "text": "glove main-spar box YK250-CH-001 from the side-of-body rib to the joint rib: two +-45 PW webs "
                         "8 plies (1.6 mm, the prongs) over the full depth between the UD cap faces (fix round 3, "
                         "VS3-01: the caps follow the glove loft outboard of the SOB cap ramp), padded to 10 mm "
                         "[+-45/0/90] blocks 50 mm long around the bushes, panels between the SOB rib, the bush pads, "
                         "the glove rib and the joint rib; UD caps widened to 52 mm over the fork"},
        "bush": {"material": "steel_4130_n", "od_m": 0.022, "id_m": 0.016, "length_tongue_m": 0.030,
                 "length_prong_m": 0.010,
                 "text": "bonded 4130 bushes 16 H8 x OD 22 (tongue 30 mm, prongs 10 mm), line-reamed in the jig; bearing "
                         "of the bush OD on the laminate checked against the QI open-hole compression strength (the "
                         "bore edge distance is below the e/D 3 of the QI bearing value; element test required)"},
        "rear": {"lug_material": "al_7075_t651_plate", "lug_fitting_bolts": 4, "lug_fitting_bolt_d_m": 0.005,
                 "slot_plate_arm_m": 0.016, "rib_bolts": 2, "rib_bolt_d_m": 0.004, "web_bolts": 2,
                 "web_bolt_d_m": 0.005, "web_pad_plies": 16,
                 "text": "rear-spar lug 7075-T651 (layout.chassis.wing_joint.rear_spar.lug) bolted 4 x M5 Ti to the "
                         "outer-panel rear-spar web pad (16 plies); slot fitting 2 x M4 Ti to the joint rib land and "
                         "2 x M5 Ti to the centre-section rear-spar web pad (16 plies)"},
        "drag_moment": ("in-plane (drag) moment about the vertical axis at the joint plane: a spanwise couple F_y = Mz / dx "
                        "between the main pins (shared equally) and the rear pin; chordwise force on the rear pin; cases "
                        "PHAA (C = L tan(alpha) at the wing CLmax), NHAA (C = L tan(alpha) at the negative CLmin), VD drag "
                        "(cd_min of the section, aft)"),
        "factors": "frequent-assembly factor 1.5 on the pins, bushes, bores and lugs (the means of attachment, STANAG "
                   "UL2.4), bearing factor 2.0 on bearing; composite special factor 1.2 (with A-basis 0.85 on the "
                   "single-load-path tongue flanges) on the laminates; fitting factor 1.15 on the bolted fittings",
    },
    "body": {
        "box_attachment_bolts_per_side_per_frame": 4, "box_attachment_bolt_d_m": 0.006,
        "frame_land_plies": 16,
        "shear_wall_doubler_plies_per_face": 0,
        "midfloor_stiffeners": 2,
        "deck_slot_strip": {"plies": 6,
                            "text": "fix round 2 (re-closure): over the nose-gear keel slot (between the keel walls, y "
                                    "+-0.0415) the avionics deck M-DECK-NOSE is a solid PW laminate of 6 plies (1.2 mm, the "
                                    "two sandwich faces together) "
                                    "at its upper-face level (core ramped out 1:3 over the keel walls): the stowed nose "
                                    "tyre keeps the 12 mm tyre-to-well clearance (layout_check C05); equipment on it is "
                                    "fixed with nutplates; span 83 mm between the keel walls"},
        "aft_keel": {"material": "al_7075_t651_plate", "process": "cnc_milling_metal", "w_m": 0.024, "h_m": 0.040,
                     "t_m": 0.0025,
                     "replaces": {"material": "al_2024_t3_sheet", "w_m": 0.024, "h_m": 0.040, "t_m": 0.0016},
                     "text": "M-AFTKEEL machined 7075-T651 channel 24 x 40 x 2.5 mm (was formed 2024-T3 "
                                            "1.6 mm: fails the bumper load; a formed 7075 / thicker 2024 channel cannot "
                                            "meet the bend-radius rule at this width)"},
        "fs3738_lower_segment": {"material": "al_7075_t651_plate", "depth_m": 0.060, "flange_w_m": 0.020,
                                 "flange_t_m": 0.0016, "web_t_m": 0.0020, "span_y_m": 0.30, "keel_bolts": 4,
                                 "keel_bolt_d_m": 0.005, "end_bolts": 3, "end_bolt_d_m": 0.005,
                                 "replaces": {"material": "al_2024_t3_sheet", "t_m": 0.002, "depth_m": 0.030,
                                              "flange_w_m": 0.020},
                                 "text": "fix round 2 (VS2-03): machined 7075-T651 lower segment of the FS3738 U-ring, "
                                         "I-section 60 mm deep (flanges 20 x 1.6 mm, web 2.0 mm; >= the 1.5 mm CNC minimum) between y +-0.15 (free "
                                         "height to the engine envelope + 10 mm is 150-180 mm there), the aft keel beam "
                                         "bolted to its web (4 x M5 12.9), spliced to the 2024 ring legs at both ends "
                                         "(3 x M5 12.9 each): reacts the tail-bumper keel reaction at FS3738"},
        "midfloor_stiffener": "M-MIDFLOOR: two bonded longitudinal hat stiffeners (PW 4 plies, 20 x 15 mm) under "
                              "the floor along the edges of the equipment-tray cut-out (y +-0.125), FS1810 to FS-FUEL; "
                              "the fixed outer strips (chine to y +-0.125) are the lower chord of the forward body, the "
                              "removable tray TR-MISSION is not credited",
        "frame_land": "solid laminate land (core ramped out) 16 plies PW (3.2 mm) at every fitting / box "
                      "attachment of the fitting frames",
    },
    "tail": {
        "stab_spar_cap_plies": 12, "stab_spar_cap_width_m": 0.025, "stab_spar_web_plies": 4,
        "spindle": {"material": "ti_6al_4v_annealed_sheet", "od_m": 0.025, "id_m": 0.0226,
                    "spline": {"length_m": 0.020, "tooth_depth_m": 0.0005,
                               "position": "outboard end of the spindle: the last 20 mm of the 0.10 m socket "
                                           "engagement (tip), where the socket's bending moment has fallen to ~10 % "
                                           "of its value at the socket mouth (fix round 2, VS2-08)"},
                    "text": "Ti-6Al-4V tube 25 x 1.2 mm (bearing seats ground; involute spline cut on the outboard "
                            "end, the last 20 mm of the socket engagement; Ti sheet allowables used as tube estimate)"},
        "socket": {"material": "al_7075_t651_plate", "od_m": 0.029, "length_m": 0.10,
                   "text": "stabilator root socket: 7075 sleeve 29 x 2 mm, 0.10 m, bonded and cross-bolted into the "
                           "stabilator root rib and spar (internal spline, cross-bolt M6)"},
        "node": {"material": "al_7075_t651_plate", "base_t_m": 0.008, "cheek_t_m": 0.007, "inboard_web_t_m": 0.003,
                 "bearing_od_m": 0.037, "fill": {"base": 0.55, "outboard": 0.70},
                 "text": "machined 7075-T651 node F-SPINDLE-NODE (layout boxes = envelopes): base flange 8 mm with "
                         "bolt bosses and edge rails (55 % of its envelope), inboard arm = 3 mm web carrying the bearing "
                         "boss (the layout cylinder: OD 42 = 2 x tail spindle_housing_radius, bore 37 for the 61805, "
                         "30 mm long), outboard cheek 7 mm with pockets outside the torsion band (70 %); the stub root "
                         "moment is carried by the outboard cheek in torsion (open section b t^3/3) into the base "
                         "flange, the base bolts take the in-plane couple"},
        "fin_spar_cap_plies": 12, "fin_spar_cap_width_m": 0.025, "fin_spar_web_plies": 4,
        "fin_root_bolt_spacing_m": 0.045,
    },
    "gear": {
        "main_leg": {"material": "al_7075_t6_sheet", "od_m": 0.042, "wall_m": 0.0035,
                     "text": "outer cylinder of the telescopic spring/oil damper leg (SAGITTA architecture); bar "
                             "allowables taken as the 7075-T6 sheet values (estimate)"},
        "main_trunnion_pin_d_m": 0.020, "main_trunnion_lug_t_m": 0.012,
        "main_downlock_pin_d_m": 0.010, "main_downlock_radius_m": 0.035,
        "main_downlock": {"material": "steel_4130_n", "ear_t_m": 0.006, "inner_t_m": 0.010, "gap_m": 0.0005,
                          "text": "fix round 2 (VS2-07): down-lock = 4130 lock bolt d 10 in double shear in a tight "
                                  "clevis of F-TRUNNION (ears 2 x 6 mm, gaps <= 0.5 mm) through a 10 mm lock lug of the "
                                  "leg yoke at r 35 mm from the retraction axis; requirement for the gear-unit "
                                  "supplier (lock bolt d >= 10 mm 4130 or equivalent, clevis gaps <= 0.5 mm)"},
        "trunnion_fitting": {"insert_flange_od_m": 0.014, "nutplate_base_m": 0.020, "k_axial_over_shear": 1.0,
                             "text": "fix round 2 (VS2-02): F-TRUNNION bolt group analysed as a rigid fitting on 9 "
                                     "elastic bolts (5 x M6 12.9 axis y into flanged through-thickness inserts of the "
                                     "16-ply gear-beam land, 4 x M6 12.9 axis z into sealed dome nutplates under the "
                                     "16-ply well-roof land; equal axial and shear bolt stiffness, estimate): force + "
                                     "three moments of the leg at the trunnion point; insert flange OD 14 mm, dome "
                                     "nutplate base 20 mm (estimates for the pull-through rows)"},
        "roof_fitting_doubler_plies_per_face": 1,
        "roof_fitting_doubler": "fix round 2 (VS2-02): one 0/90 PW ply added on each face of the well roof (fuel-bay "
                                "floor) over 150 x 100 mm under each trunnion fitting (roof-bolt couple of the leg "
                                "moments, G-ROOF-TORSION)",
        "beam_torsion": {"text": "fix round 2 (VS2-02): the roll moment M_x of the leg (down-lock reaction) leaves the "
                                 "fitting as a couple of y-forces on the gear-beam wall (beam-insert bolts) and z-forces "
                                 "on the well roof (roof bolts); the beam wall carries its share by plate bending between "
                                 "its top edge (well-roof deck, diaphragm) and its bottom edge (lower skin corner, "
                                 "diaphragm) over the flange length of the fitting; the diaphragms carry the edge forces "
                                 "in shear to FS-GEAR and the well forward wall"},
        "nose_leg": {"material": "al_7075_t6_sheet", "od_m": 0.036, "wall_m": 0.003},
        "nose_pivot_lug_t_m": 0.006,
        "nose_pivot_bush_od_m": 0.022,
        "uplock": {"insert": "potted M5 insert sleeves, potting diameter 24 mm (one potted block under the 20 mm "
                             "pattern), in the well-roof sandwich; hook load line within 6 mm of the pattern centre",
                   "b_p_m": 0.012, "hook_offset_m": 0.006},
        "beam_end_clip": {"bolts": 3, "bolt_d_m": 0.005,
                          "text": "gear-beam ends: bonded 7075 angle clips along the beam height, 3 x M5 Ti into the "
                                  "16-ply solid lands of FS-RS / FS-GEAR"},
        "gear_operating_speed_factor_vs": 1.6,
        "door_hinge_pin_d_m": 0.004, "door_hinge_count": 3,
    },
    "engine_mount": {"strut_od_m": 0.0127, "strut_wall_m": 0.00089, "ring_od_m": 0.0159, "ring_wall_m": 0.00089,
                     "foot_bolts": 2, "foot_bolt": "M8 12.9", "backing_plate_m": [0.056, 0.032, 0.005]},
    "firewall": {
        "core_insert": {"core": "core_rohacell_71wf", "area_m2": 0.12,
                        "text": "ROHACELL 71 WF core in the lower and outboard part of the firewall (y +-0.01..0.25, "
                                "z 0.02..0.27: lower engine feet and stabilator-node bases), core splice to the 51 WF"},
        "foot_land": {"core": "core_rohacell_71wf", "core_insert_m": 0.20, "land_plies": 22, "land_r_m": 0.082,
                      "core_shear_peak_factor": 1.23,
                      "peak_basis": "fix round 2 (VS2-06): the core shear round the land is not uniform (the nearest "
                                    "edge support draws more of the foot load); peak / mean = 1.23 from the review's "
                                    "plate-shear distribution (structures-2 VS2-06; no own FE), applied to the mean "
                                    "tau = P / (2 pi r d); land radius 66 -> 82 mm to restore the margin (with the "
                                    "engine-harness cut-out C-FW-HARN moved below the feet, fix round 2)",
                      "text": "lower engine-mount feet (F-EMOUNT-LO): the firewall sandwich gets a ROHACELL 71 WF core "
                              "insert 200 x 200 mm and a solid PW land of 22 plies (4.4 mm, core ramped out 1:3) of radius "
                              "82 mm around each foot; the out-of-plane (x) foot load is carried by plate bending and core "
                              "shear to the firewall perimeter (no longeron at the lower feet)"},
        "plate_support_m": "distance from the foot to the nearest firewall edge support (fuselage contour - inset)",
    },
    "parachute": {
        "shackle_pin_d_m": 0.008, "shackle_material": "ti_6al_4v_annealed_sheet", "lug_ear_t_m": 0.006,
        "lug_w_m": 0.031, "lug_e_m": 0.0155, "frame_bolts": 2, "frame_bolt_d_m": 0.005, "floor_bolts": 4,
        "floor_bolt_d_m": 0.004, "floor_bolt_rows_m": 0.024,
        "floor_washer_plate": {"material": "al_7075_t651_plate", "w_m": 0.048, "l_m": 0.040, "t_m": 0.003,
                               "text": "one 7075 washer plate 48 x 40 x 3 mm under the four floor bolts, below the spine "
                                       "floor pad (fix round 2, VS2-01: pull-through of the fitting moment)"},
        "frame_land_plies": 32, "spine_pad_plies": 16, "spine_plies": 4,
        "skin_screw": "ISO 7380 M4 A2-70 into nutplates, pitch 25 mm, both spine flanges, FS1810 -> FS-FUEL "
                      "(P-MB-UPPER)",
        "skin_screw_pitch_m": 0.025,
        "text": "bridle fittings in the dorsal spine channel M-SPINE: x-component and the fitting moment by 4 x M4 12.9 "
                "(axis z, nuts on a 48 x 40 mm washer plate under the 16-ply spine pad), z-component by 2 x M5 12.9 "
                "(axis x) into a 32-ply (6.4 mm) solid land of the frame; Ti-6Al-4V shackle pin d 8 in a tight clevis "
                "(ears 6 mm, gaps 0.5 mm) through a 4130 bridle spool (fix round 2, VS2-01) "
                "(FS1810 / FS-RS, core ramped out); spine 4 plies PW (0.8 mm) U 44 x 24 with 60 mm flanges, screwed to "
                "the fixed mission-bay upper skin P-MB-UPPER (net x-component in shear to the chine longerons)",
    },

    "fuel_bay": {
        "floor_layup": "fuel_floor_wellroof",
        "floors": ["M-WELLROOF"],
        "floor_note": "the aft-cell floor (well roof) keeps the 6.8 mm thickness of the layout (no interface change) "
                      "with 3-ply faces and the denser ROHACELL 71 WF core (core shear and face strain under the bay "
                      "pressure); the forward-cell floor M-FWDDECK keeps layups.rib_panel: the two payload-tray rails "
                      "are bonded and screwed to its underside at 50 mm pitch as stiffeners and halve its span",
        "payload_rails_y_m": 0.10,
        "payload_rail": {"material": "al_6061_t6_sheet", "flange_w_m": 0.020, "flange_t_m": 0.0025, "stem_h_m": 0.020,
                         "stem_t_m": 0.0025, "text": "payload-tray rails 6061-T6 T-section 20 x 20 x 2.5 mm at y +-0.10, "
                                                     "FS-FUEL to the main-spar frame, bonded + M4 screws at 50 mm to "
                                                     "the deck underside"},
        "well_web_t_m": 0.001, "well_web_y_m": 0.0105, "well_web_height_m": 0.12,
        "well_web": "M-WELLKEEL: two keel webs 1.0 mm solid PW laminate (5 plies, 0/90 and +-45 alternating) at "
                    "y +-0.0105..0.0115, the walls of the centre-line harness channel between the two main wells "
                    "(well forward wall to FS-GEAR, well roof to 0.12 m below it): halve the span of the well roof under "
                    "the aft fuel cell; the inner-door hinge brackets (hinge lines y +-0.01) are bolted to their "
                    "lower-edge flanges"},
    "turret": {"rail_screws_per_rail_end": 4, "screw": "M3 A2-70",
               "roof_block": "bearing block through-bolted 4 x M4 A2-70 with a 2 mm 7075 backing plate on the upper roof "
                             "face (no potted inserts in tension)",
               "door_area_each_m2": 0.0223, "door_rail": "6061-T6 rail with a 1 mm retaining lip, PTFE slider"},
    "ground_handling": {
        "tow_load_fraction_W": 0.30,
        "tow_load_basis": "0.3 W, the CS-23.509 / FAR 23.509 towing load for W <= 30 000 lb, used as a conservative "
                          "design choice: the CS-LUAS / CS-VLA towing clause text is not in the research files; tow bar "
                          "on the nose-fork axle (layout.chassis.ground_handling)",
        "tow_load_basis_tr": "W <= 30 000 lb için CS-23.509 / FAR 23.509 çekme yükü 0,3 W, muhafazakâr tasarım kararı "
                             "olarak: CS-LUAS / CS-VLA çekme maddesi metni araştırma dosyalarında yok; çekme demiri burun "
                             "çatalı dingilinde (layout.chassis.ground_handling)",
        "tie_down": "not designed for outdoor mooring: the operating concept keeps the aircraft in its transport cradle / "
                    "shelter when not flying (van transport, assembled on the field, assembly.transport); tie-down "
                    "points and their wind loads are an open item if outdoor parking is required",
        "jacking": "no jacking points: the aircraft is lifted by two persons or rests on the transport cradle saddles "
                   "(FS1810 / FS-GEAR lower lands, structures TR-PAD / TR-FRAME) for gear work"},
    "transport": {
        "factors_limit": {"vertical": 3.0, "fore_aft": 1.5, "lateral": 1.5},
        "basis": ("design choice (no transport load specification in the research files): road transport in the "
                  "cradle, limit load factors 3.0 g vertical, 1.5 g fore/aft, 1.5 g lateral (each alone), FoS 1.5; "
                  "to be replaced by the transport specification of the operator"),
    },
}

# layups owned / revised by the structures phase (written to spec.layups by --update-spec, checked by --check). Face
# codes are listed from the exposed surface toward the core; a '0/90' PW ply holds 0 and 90 deg fibres, a '+-45' ply
# +45 and -45 deg fibres (baseline.yaml#processes.laminate_stacking: +-45 outer plies, >= 10 % of the fibres in each of
# 0 / +45 / -45 / 90 over the sandwich).
LAYUPS = {
    "wing_skin_primary": {
        "plies": [["cfrp_pw_mtm45_as4", "+-45,0/90,+-45", 3]], "core": "core_rohacell_51wf", "core_t": 0.005,
        "inner_plies": [["cfrp_pw_mtm45_as4", "+-45,+-45", 2]], "adhesive_areal": 0.0,
        "use": ("wing and LERX skins (primary sandwich, 0.6/5/0.4 mm); structures phase: +-45-dominated faces (one 0/90 "
                "ply per sandwich = 10 % 0 and 90 deg, +-45 outer plies) so that the skin's low spanwise modulus leaves "
                "the bending to the spar caps and the skin stays below its shear-crimping load (the earlier "
                "'0/90,+-45,0/90 / +-45,0/90' faces crimp at the cap design strain; docs/04_yapi_hesaplari.md)"),
        "source": "endurance concept study areal-mass rule; baseline.yaml#processes min laminate; structures.py "
                  "(ply orientation, same ply count and thickness)"},
    "wing_box_skin_upper": {
        "plies": [["cfrp_pw_mtm45_as4", "+-45,0/90,+-45", 3]], "core": "core_rohacell_51wf", "core_t": 0.006,
        "inner_plies": [["cfrp_pw_mtm45_as4", "+-45,+-45", 2]], "adhesive_areal": 0.0,
        "use": ("upper wing skin between the main- and rear-spar caps from the side-of-body rib to "
                "structures.sizing.wing.box_skin_upper_y_end_m (0.6/6/0.4 mm): core 1 mm thicker than "
                "wing_skin_primary (buckling and shear crimping under the bending compression); OML unchanged, the "
                "inner surface steps 1 mm at the cap edges (core ramp)"),
        "source": "structures.py (check W-SKINBUCK / W-SKINCRIMP)"},
    "lerx_skin_upper_root": {
        "plies": [["cfrp_pw_mtm45_as4", "+-45,0/90,+-45", 3]], "core": "core_rohacell_51wf", "core_t": 0.007,
        "inner_plies": [["cfrp_pw_mtm45_as4", "+-45,+-45", 2]], "adhesive_areal": 0.0,
        "use": ("glove LERX upper skin of the first bay (side-of-body rib to glove rib, ahead of the main-spar cap; "
                "0.6/7/0.4 mm, fix round 3 VS3-04): the 0.15 m bay has no nose rib (pin, puller and reamer corridors) "
                "and carries the box bending strain; core 2 mm thicker than wing_skin_primary for panel buckling; OML "
                "unchanged, the inner face steps 2 mm (core ramp at the glove rib and the main-cap edge)"),
        "source": "structures.py (checks W-SKINBUCK-LERX-UP / W-SKINCRIMP-LERX-UP)"},
    "ct_box_cover": {
        "plies": [["cfrp_pw_mtm45_as4", "+-45,0/90", 2]], "core": "core_rohacell_51wf", "core_t": 0.006,
        "inner_plies": [["cfrp_pw_mtm45_as4", "+-45,+-45", 2]], "adhesive_areal": 0.0,
        "use": ("carry-through box covers (upper and lower) between the spar frames FS-MS / FS-RS, y -0.40..0.40 "
                "(0.4/6/0.4 mm), bonded to the CT-box caps and continuing the glove skins across the body"),
        "source": "structures.py (checks W-SKINBUCK-CT / W-SKINCRIMP-CT)"},
    "fuel_floor_wellroof": {
        "plies": [["cfrp_pw_mtm45_as4", "+-45,0/90,0/90", 3]], "core": "core_rohacell_71wf", "core_t": 0.0056,
        "inner_plies": [["cfrp_pw_mtm45_as4", "+-45,0/90,0/90", 3]], "adhesive_areal": 0.0,
        "use": ("aft fuel-cell floor = main-well roof M-WELLROOF (0.6/5.6/0.6 mm = 6.8 mm, the layout thickness): "
                "3-ply faces and the denser ROHACELL 71 WF core for the bay design pressure"),
        "source": "structures.py (checks F-*-030)"},
}

# =====================================================================================================================
# context
# =====================================================================================================================
COMPOSITE_KINDS = ("composite", "core")


class Ctx:
    """Spec, factors, masses and geometry handles shared by the load cases and the member checks."""

    def __init__(self, S: dict | None = None, design: dict | None = None):
        self.S = S if S is not None else SPEC.load()
        S = self.S
        self.from_spec = design is None and bool(S["structures"].get("sizing"))
        self.D = copy.deepcopy(design if design is not None else (S["structures"].get("sizing") or DESIGN))
        self.M = S["materials"]
        # layups: the spec's (check mode) or the spec's with the structures-owned entries of LAYUPS (authoring mode)
        self.lay = copy.deepcopy(S["layups"])
        if not self.from_spec:
            self.lay.update(copy.deepcopy(LAYUPS))
        st = S["structures"]
        self.f = {"fos": float(st["factor_of_safety"]), "fit": float(st["fitting_factor"]),
                  "bear": float(st["bearing_factor_pinned_joints"]), "fa": float(st["frequent_assembly_attachment_factor"]),
                  "comp": float(st["composite_special_factor"]), "a_basis": float(st["a_basis_factor_single_load_path"]),
                  "ctl": float(st["control_system_load_factor"]), "hinge_total": 6.67, "pushpull_total": 3.33}
        self.dt = {"cap_comp": float(st["damage_tolerance_strain_ue"]["compression_thick"]) * 1e-6,
                   "cap_tens": 5000e-6, "sand_comp": 2600e-6, "sand_tens": 5000e-6,
                   "shear_thin": float(st["damage_tolerance_strain_ue"]["shear_thick"]) * 1e-6}
        self.af = Z.Airframe(S)
        self.P = S["wing"]["planform"]
        self.T = Z.trapezoid(self.P)
        self.m0 = float(S["mass"]["mtow_kg"])
        der = st["derived"]
        gm = der["gust_matrix"]
        self.m_min = min(float(r["mass_kg"]) for r in gm)
        self.n_wing = float(der["n_limit_wing_design"])
        self.n_wing_neg = min(float(r["n_neg"]) * float(r["mass_kg"]) for r in gm) / self.m0
        self.n_eq_pos = max(float(r["n_pos"]) for r in gm + [{"n_pos": st["n_limit_pos"]}])
        self.n_eq_neg = min(float(r["n_neg"]) for r in gm + [{"n_neg": st["n_limit_neg"]}])
        self.VA = float(der["VA_eas"])
        self.VC = float(st["VC_eas"])
        self.VD = float(der["VD_eas_used"])
        self.q = lambda V: 0.5 * RHO0 * V * V
        self.ws = self.m0 * G / float(S["wing"]["area"])
        self.qcp = self.dt  # alias
        self.cache = {}


def mat(c: Ctx, key: str) -> dict:
    return c.M[key]


def total_factor(c: Ctx, ult_only: bool = False, fit: bool = False, bear: bool = False, fa: bool = False,
                 comp: bool = False, total: float | None = None, extra: float = 1.0) -> dict:
    """Total factor on a limit load: FoS x max(applicable special factors) (CS-LUAS.619); ``total`` overrides
    (hinge bearings 6.67, push-pull joints 3.33); ``extra`` multiplies (e.g. the A-basis estimate is applied to the
    allowable instead, so extra is normally 1)."""
    if total is not None:
        return {"fos": 1.0 if ult_only else c.f["fos"], "special": total / c.f["fos"], "total": total,
                "text": f"{total:g} total"}
    fos = 1.0 if ult_only else c.f["fos"]
    sp = {"fitting": c.f["fit"] if fit else 1.0, "bearing": c.f["bear"] if bear else 1.0,
          "frequent assembly": c.f["fa"] if fa else 1.0, "composite": c.f["comp"] if comp else 1.0}
    k = max(sp, key=sp.get)
    s = sp[k]
    txt = ("ultimate-only" if ult_only else f"FoS {fos:g}") + (f" x {k} {s:g}" if s > 1.0 else "")
    if extra != 1.0:
        txt += f" x {extra:g}"
    return {"fos": fos, "special": s, "total": fos * s * extra, "text": txt}


class Rows:
    """Collector of margin-of-safety rows."""

    def __init__(self):
        self.rows = []

    def add(self, id_: str, group: str, member: str, member_tr: str, case: str, case_tr: str, applied: float,
            allowable: float, unit: str, fac: dict, ref: str, part: str = "", note: str = "", kind: str = "strength"):
        app = float(applied)
        alw = float(allowable)
        ms = math.inf if app <= 0 else alw / (app * fac["total"]) - 1.0
        self.rows.append({"id": id_, "group": group, "member": member, "member_tr": member_tr, "part": part,
                          "case": case, "case_tr": case_tr, "applied": app, "allowable": alw, "unit": unit,
                          "factor_text": fac["text"], "factor_total": fac["total"], "ms": ms, "ref": ref,
                          "note": note, "kind": kind})
        return ms

    def info(self, id_: str, group: str, member: str, member_tr: str, case: str, case_tr: str, value: float,
             unit: str, ref: str, note: str = "", part: str = "", requirement: float | None = None):
        """Requirement / information row (purchased item rating to be confirmed, load handed to the detail design):
        no MS."""
        self.rows.append({"id": id_, "group": group, "member": member, "member_tr": member_tr, "part": part,
                          "case": case, "case_tr": case_tr, "applied": float(value), "allowable": requirement,
                          "unit": unit, "factor_text": "-", "factor_total": None, "ms": None, "ref": ref,
                          "note": note, "kind": "requirement"})


# =====================================================================================================================
# 1. load cases
# =====================================================================================================================
def flight_envelope(c: Ctx) -> dict:
    """V-n: manoeuvre (CS-LUAS.337) + gust (CS-LUAS.333(c), CS-VLA 341) for every mass x altitude of the sizing gust
    matrix (spec.structures.derived.gust_matrix, configuration lift slope). The wing is designed for the largest lift
    n m g (MTOM); equipment and attachments use the largest n (lightest case). The negative wing case is the largest
    |n m| over the matrix expressed at MTOM. The negative manoeuvre factor is kept constant to VD (conservative,
    standards.yaml note_existing_code)."""
    st = c.S["structures"]
    der = st["derived"]
    VS = math.sqrt(2 * c.ws / (RHO0 * float(c.S["aero"]["clmax_clean"])))
    lc = {"VS_eas": VS, "VA_eas": c.VA, "VC_eas": c.VC, "VD_eas": c.VD,
          "VF_eas": float(c.S["wing"]["controls"]["flap"]["checks"]["V_eas_m_s"]),
          "n_manoeuvre": [float(st["n_limit_pos"]), float(st["n_limit_neg"])],
          "gust_matrix": der["gust_matrix"], "n_wing_pos": c.n_wing, "n_wing_neg_equiv": c.n_wing_neg,
          "n_equipment": [c.n_eq_pos, c.n_eq_neg], "m_min_kg": c.m_min, "mtom_kg": c.m0,
          "n_flaps": 2.0, "q_VA": c.q(c.VA), "q_VC": c.q(c.VC), "q_VD": c.q(c.VD)}
    lc["V_LO_eas"] = float(c.D["gear"]["gear_operating_speed_factor_vs"]) * VS
    return lc


def wing_mass_terms(c: Ctx) -> dict:
    """Per-side masses (kg) of the terms of the wing set (wing_structure_pair + ailerons + flaps), each with the
    spanwise interval it physically occupies (fix round 1, S1-01), for the inertia relief. Fix round 2 (VS2-05): the
    relief is based on the minimum credible wing mass - the bottom-up masses WITHOUT the growth allowance and without
    the 1.05 model factor of sizing.wing_structure (mass_base_kg of the items / 1.05); a heavier wing only adds relief.
    The terms come from sizing.wing_structure with the structures-phase bottom-up masses (spec.structures.sizing.mass.
    wing) and are scaled by one factor so that their sum equals that minimum mass (closure rounding)."""
    if "wing_terms" in c.cache:
        return c.cache["wing_terms"]
    S = c.S
    ws = Z.wing_structure(S, c.af, c.m0, c.n_wing)
    names = ("caps", "webs", "rear_spar", "ribs", "skins", "joints")
    base = {k: float(ws[k]) for k in names}
    W_set = (mass_item_base(c, "wing_structure_pair") + mass_item_base(c, "control_surfaces_ailerons_pair") +
             mass_item_base(c, "control_surfaces_flaps_pair")) / 1.05
    s = W_set / sum(base.values())
    out = {k: v * s / 2 for k, v in base.items()}
    out["scale"] = s
    out["basis"] = "minimum credible: item base masses (no growth allowance) / 1.05 model factor"
    c.cache["wing_terms"] = out
    return out


def _shape(y: np.ndarray, w: np.ndarray, m: float) -> np.ndarray:
    """Distribution of the mass m (kg) over y with the shape w (normalised to integrate to m)."""
    a = float(np.trapz(w, y))
    return m * w / a if a > 0 else np.zeros_like(y)


def wing_inertia(c: Ctx, y: np.ndarray) -> np.ndarray:
    """Wing mass per unit span (kg/m, one half) for the inertia relief, every term of the wing set where it physically
    is (fix round 1, S1-01; the earlier model spread the whole set with the reference chord outboard of the side-of-body
    rib, which gave the carry-through caps and the joint hardware a relief they do not give):
    * main spar caps by the ply schedule (structures.sizing.wing.main_cap.zones x cap width), the carry-through part
      (y < 0.40, inside the body) where it is;
    * outer-panel main webs by the web ply zones x the chord (web depth ~ chord), y 0.70 - tip;
    * rear spar (caps + webs) in proportion to the reference chord from the side-of-body rib to the tip;
    * ribs in proportion to the square of the loft chord (rib area ~ depth x chord) from y_root_structure;
    * joint hardware of the outer panel (CFRP tongue, its cap build-up, rear-spar lug) uniformly from the tongue tip
      (fork slot inboard end) to the end of the cap build-up in the outer panel;
    * skins and the ailerons / flaps on the actual loft chord (wing.sections) from the body side to the tip;
    * the aileron and flap actuators as point masses at their layout positions (smeared over 0.1 m)."""
    S = c.S
    y = np.asarray(y, float)
    t = wing_mass_terms(c)
    b2 = c.T["b2"]
    yj = float(c.P["y_junction"])
    secs = S["wing"]["sections"]
    ys_ = np.array([s_["y"] for s_ in secs])
    c_act = np.interp(y, ys_, [s_["chord"] for s_ in secs])
    sz = S["structures"].get("sizing") or {}
    zones = ((sz.get("wing") or {}).get("main_cap") or {}).get("zones") or [[0.0, b2, 20]]
    wz = ((sz.get("wing") or {}).get("main_web") or {}).get("zones") or [[yj, b2, 3]]
    D = c.D["wing"]["main_cap"]
    n_cap = np.array([cap_plies_at(zones, v) for v in y], float)
    w_cap = np.where(y < yj, float(D["width_body_glove_m"]), float(D["width_outer_m"]))
    n_web = np.array([cap_plies_at(wz, v) for v in y], float)
    inside = y <= b2 + 1e-9
    w = _shape(y, n_cap * w_cap * inside, t["caps"])
    w = w + _shape(y, np.where(y >= yj, n_web * c.T["c"](y), 0.0), t["webs"])
    w = w + _shape(y, np.where(y >= Y_SOB, c.T["c"](y), 0.0), t["rear_spar"])
    y_rs = float(c.P["y_root_structure"])
    w = w + _shape(y, np.where(y >= y_rs, c_act ** 2, 0.0), t["ribs"])
    y_tt = float(c.S["layout"]["chassis"]["wing_joint"]["main_spar"]["fork"]["slot"]["y"][0])
    w = w + _shape(y, np.where((y >= y_tt) & (y <= yj + float(c.D["wing_joint"]["tongue"]["buildup_length_m"])), 1.0,
                               0.0), t["joints"])
    w = w + _shape(y, np.where(y >= ys_[0], c_act, 0.0), t["skins"])
    for act, item in (("ACT-AILERON", "actuators_ailerons_2x_DA26"), ("ACT-FLAP", "actuators_flaps_2x_DA30")):
        a = [a_ for a_ in S["layout"]["systems"]["actuators"] if a_["id"] == act][0]
        ya = float(a["servo_axis"][1])
        w = w + np.where(np.abs(y - ya) <= 0.05, mass_item_base(c, item) / 2 / 0.1, 0.0)
    return w


def wing_loads(c: Ctx, n: int = 361) -> dict:
    """Spanwise LIMIT loads of one half wing (root y = 0 at the centre line, reference trapezoid, Schrenk lift):
    shear V(y), bending M(y) for the positive design case n_wing x MTOM and the negative case, with the inertia relief
    of the wing structure and the wing-mounted equipment at the same n (wing_inertia; sizing.wing_structure omits the
    relief as a conservative simplification, structures.sizing.wing.inertia_relief); torsion about the main-spar line (25 % chord, the section aerodynamic
    centre lies on it): section pitching moment cm0 of the NLF(1)-0416 (NeuralFoil, aero.characteristics) at VD
    (the largest q), at VA with full aileron (CS-LUAS.349(b): dcm = -0.01 per degree over the aileron span) and at VF
    with 40 deg flap (thin-aerofoil plain-flap dcm = -1/2 sin(th) (1 - cos(th)) delta, cos(th) = 1 - 2 cf/c)."""
    if "wing_loads" in c.cache:
        return c.cache["wing_loads"]
    T, P = c.T, c.P
    b2 = T["b2"]
    y = np.linspace(0.0, b2, n)
    l = ST.schrenk(y, T["c"](y), b2)
    m_w = wing_inertia(c, y) if c.D["wing"].get("inertia_relief", False) else np.zeros_like(y)
    w = c.n_wing * G * (c.m0 / 2 * l - m_w)
    V, M = ST.beam_loads(y, w)
    w_n = c.n_wing_neg * G * (c.m0 / 2 * l - m_w)
    Vn, Mn = ST.beam_loads(y, w_n)
    cm0 = AE.characteristics("nlf416", Z.AL.reynolds(c.VC, float(T["c"](P["y_junction"])), 0.0))["cm0"]
    cc = T["c"](y)
    ctl = c.S["wing"]["controls"]
    ya0, ya1 = ctl["aileron"]["eta0"] * b2, ctl["aileron"]["eta1"] * b2
    yf0, yf1 = ctl["flap"]["eta0"] * b2, ctl["flap"]["eta1"] * b2
    d_ail = max(abs(v) for v in ctl["aileron"]["range_deg"])
    dcm_ail = -0.01 * d_ail
    E = float(ctl["flap"]["chord_fraction"])
    th = math.acos(1 - 2 * E)
    d_flap = math.radians(max(ctl["flap"]["range_deg"]))
    dcm_flap = -0.5 * math.sin(th) * (1 - math.cos(th)) * d_flap
    VF = float(ctl["flap"]["checks"]["V_eas_m_s"])

    def torque(qd, extra):
        m_ = qd * cc ** 2 * (abs(cm0) + extra)
        Tq = np.zeros_like(y)
        for i in range(len(y) - 2, -1, -1):
            Tq[i] = Tq[i + 1] + 0.5 * (m_[i] + m_[i + 1]) * (y[i + 1] - y[i])
        return Tq
    T_VD = torque(c.q(c.VD), 0.0)
    T_ail = torque(c.q(c.VA), np.where((y >= ya0) & (y <= ya1), abs(dcm_ail), 0.0))
    T_flap = torque(c.q(VF), np.where((y >= yf0) & (y <= yf1), abs(dcm_flap), 0.0))
    Tmax = np.maximum(np.maximum(T_VD, T_ail), T_flap)
    out = {"y": y, "V": V, "M": M, "V_neg": Vn, "M_neg": Mn, "inertia_relief": bool(c.D["wing"].get("inertia_relief")),
           "m_wing_half": float(np.trapz(m_w, y)),
           "T": Tmax, "T_VD": T_VD, "T_aileron": T_ail, "T_flap": T_flap, "cm0": cm0, "dcm_aileron": dcm_ail,
           "dcm_flap": dcm_flap, "n": c.n_wing, "n_neg": c.n_wing_neg}
    # chordwise (forward) load at high incidence (point A): C = L tan(alpha_section), no drag credit; section angle
    # at the wing CLmax from the configuration lift curve (aero.reference CL_0, CL_alpha) + wing incidence
    ar = c.S["aero"]["reference"]
    a_body = (float(ar["clmax_wing"]) - float(ar["CL_0"])) / float(ar["CL_alpha_per_rad"])
    a_sec = a_body + math.radians(float(P["incidence_deg"]))
    out["alpha_section_at_clmax_deg"] = math.degrees(a_sec)
    out["chord_fraction_of_lift"] = math.tan(a_sec)
    c.cache["wing_loads"] = out
    return out


def at(L: dict, key: str, y: float) -> float:
    return float(np.interp(y, L["y"], L[key]))


# =====================================================================================================================
# 2. wing section model (spar caps + box skins, plane sections)
# =====================================================================================================================
def ply_props(c: Ctx, key: str) -> dict:
    m = mat(c, key)
    return {"E1": float(m["E"]), "E2": float(m["E2"]), "G12": float(m["G12"]), "nu12": float(m["nu12"]),
            "Ftu": float(m["Ftu"]), "Fcu": float(m["Fcu"]), "Fsu": float(m["Fsu"]),
            "F2tu": m.get("F2tu"), "F2cu": m.get("F2cu"), "t": float(m["ply_t"])}


def face_laminate(c: Ctx, code: str, n: int = 1, key: str = "cfrp_pw_mtm45_as4") -> dict:
    """Laminate of PW fabric plies from a layup code ('0/90,+-45,0/90'): a '0/90' ply at 0 deg, a '+-45' ply at 45
    deg (fabric: one ply holds both directions); ``n`` repeats."""
    pp = ply_props(c, key)
    plies = []
    for _ in range(n):
        for tok in str(code).split(","):
            tok = tok.strip()
            th = 45.0 if "45" in tok else 0.0
            plies.append((pp, th, pp["t"]))
    return ST.laminate_abd(plies)


def skin_faces(c: Ctx, layup: str = "wing_skin_primary") -> dict:
    """Outer and inner face laminates, core thickness and core properties of a sandwich layup (key of the context
    layups = spec.layups + structures-owned LAYUPS). Face codes list the plies from the exposed surface toward the core
    (one entry = one PW ply, the count must match the code)."""
    L = c.lay[layup]
    (k_o, code_o, n_o), = L["plies"]
    (k_i, code_i, n_i), = L["inner_plies"]
    fo = face_laminate(c, code_o, 1 if len(str(code_o).split(",")) == n_o else n_o, k_o)
    fi = face_laminate(c, code_i, 1 if len(str(code_i).split(",")) == n_i else n_i, k_i)
    core = mat(c, L["core"])
    t_lab = f"{fo['h'] * 1000:.1f}/{float(L['core_t']) * 1000:g}/{fi['h'] * 1000:.1f}".replace(".", ",")
    return {"outer": fo, "inner": fi, "t_out": fo["h"], "t_in": fi["h"], "c": float(L["core_t"]),
            "E_out": ST.laminate_engineering(fo)["Ex"], "E_in": ST.laminate_engineering(fi)["Ex"],
            "G_out": ST.laminate_engineering(fo)["Gxy"], "G_in": ST.laminate_engineering(fi)["Gxy"],
            "core": core, "core_key": L["core"], "layup": layup, "label": f"{layup} {t_lab.replace(',', '.')} mm",
            "label_tr": f"{layup} {t_lab} mm", "codes": (code_o, code_i)}


def wing_contour(c: Ctx, y: float):
    """Upper / lower OML z(x) of the wing at span station y: linear loft between the bracketing spec sections, twist
    about the section leading edge (the construction of sizing.spar_depth_profile)."""
    secs = c.S["wing"]["sections"]
    ys = np.array([s_["y"] for s_ in secs])
    y = float(np.clip(y, ys[0], ys[-1]))
    j = int(np.clip(np.searchsorted(ys, y) - 1, 0, len(ys) - 2))
    t = (y - ys[j]) / max(ys[j + 1] - ys[j], 1e-9)
    cache = c.cache.setdefault("airfoils", {})

    def surf(i, x):
        s_ = secs[i]
        key = (s_["airfoil"], round(float(s_.get("thickness_scale", 1.0)), 4))
        if key not in cache:
            cache[key] = oml.resampled(s_["airfoil"], 201, 0.0015 / max(s_["chord"], 1e-6), key[1])
        xc, yu, yl = cache[key]
        u = np.clip((np.asarray(x, float) - s_["x_le"]) / s_["chord"], 0.0, 1.0)
        tw = math.radians(s_["twist_deg"])
        dz = -(np.asarray(x, float) - s_["x_le"]) * math.sin(tw)
        return (s_["z_le"] + s_["chord"] * np.interp(u, xc, yu) + dz, s_["z_le"] + s_["chord"] * np.interp(u, xc, yl) + dz)

    def f(x):
        a, b = surf(j, x), surf(j + 1, x)
        return (1 - t) * a[0] + t * b[0], (1 - t) * a[1] + t * b[1]
    return f


Y_SOB = 0.40          # side-of-body rib (layout M-SOB): CT box inboard, glove outboard


def ctbox_member(c: Ctx) -> dict:
    if "ctbox" not in c.cache:
        c.cache["ctbox"] = [m_ for m_ in c.S["layout"]["chassis"]["members"] if m_["id"] == "M-CTBOX"][0]
    return c.cache["ctbox"]


def sob_ramp(c: Ctx) -> tuple:
    """(y0, y1) of the cap ramp at the side-of-body rib (layout M-CTBOX.sob_transition, fix round 3 VS3-01): the caps
    leave the box-cover level at y0 and reach the glove loft at y1 (linear); (Y_SOB, Y_SOB) = step (layout phase)."""
    tr = ctbox_member(c).get("sob_transition") or {}
    y = tr.get("y") or [Y_SOB, Y_SOB]
    return float(y[0]), float(y[1])


def sob_fraction(c: Ctx, y: float) -> float:
    """Blend factor of the cap faces between the box-cover level (0) and the glove loft (1) at span station y."""
    y0, y1 = sob_ramp(c)
    if y < y0:
        return 0.0
    if y1 <= y0 + 1e-9:
        return 1.0
    return min(max((y - y0) / (y1 - y0), 0.0), 1.0)


def wing_section(c: Ctx, y: float, n_main: int, n_rear: int | None = None, nx: int = 24) -> dict:
    """Bending section of the wing box at span station y (plane sections, spanwise moduli): main and rear UD spar
    caps under the skins (skin over the caps solid laminate, core ramped out), the box skins between the caps (outer
    and inner faces of layups.wing_skin_primary; inside the body the CT-box sandwich covers, structures.sizing);
    D-nose and trailing-edge skins and the +-45 webs are not counted (conservative for strains). Returns EI, neutral
    axis, element strains per unit moment, cap lever arm, enclosed box area (torsion) and web heights."""
    D = c.D["wing"]
    T = c.T
    cT = float(T["c"](y))
    xle = float(T["xle"](y))
    xm, xr = xle + float(c.P["main_spar_frac"]) * cT, xle + float(c.P["rear_spar_frac"]) * cT
    ud = ply_props(c, D["main_cap"]["material"])
    t_ply = ud["t"]
    body = y < Y_SOB
    wm = D["main_cap"]["width_body_glove_m"] if y < float(c.P["y_junction"]) else D["main_cap"]["width_outer_m"]
    wr = D["rear_cap"]["width_m"]
    tm = n_main * t_ply
    tr = (n_rear if n_rear is not None else D["rear_cap"]["plies"]) * t_ply
    sk = skin_faces(c, D["ct_box"]["cover_layup"] if body else D["skin_layup"])
    els = []                                  # (E, A, z, kind)
    ct = ctbox_member(c)
    zt, zb = float(ct["z"][1]), float(ct["z"][0])
    if body:
        zu = lambda x: np.full_like(np.asarray(x, float), zt)           # noqa: E731
        zl = lambda x: np.full_like(np.asarray(x, float), zb)           # noqa: E731
    else:
        f = wing_contour(c, y)
        zu = lambda x: f(x)[0]                                           # noqa: E731
        zl = lambda x: f(x)[1]                                           # noqa: E731
    t_s = float(D["skin_solid_over_caps_m"])
    xs = np.linspace(xm + wm / 2, xr - wr / 2, nx + 1)
    xc_, dx = 0.5 * (xs[1:] + xs[:-1]), np.diff(xs)
    zus, zls = np.asarray(zu(xc_)), np.asarray(zl(xc_))
    skins_on = y >= float(D.get("box_skins_effective_from_y_m", 0.0))
    for k_ in range(len(xc_) if skins_on else 0):
        els.append((sk["E_out"], sk["t_out"] * dx[k_], zus[k_] - sk["t_out"] / 2, "skin_outer"))
        els.append((sk["E_in"], sk["t_in"] * dx[k_], zus[k_] - sk["t_out"] - sk["c"] - sk["t_in"] / 2, "skin_inner"))
        els.append((sk["E_out"], sk["t_out"] * dx[k_], zls[k_] + sk["t_out"] / 2, "skin_outer_lo"))
        els.append((sk["E_in"], sk["t_in"] * dx[k_], zls[k_] + sk["t_out"] + sk["c"] + sk["t_in"] / 2, "skin_inner_lo"))
    E_sover = 0.5 * (sk["E_out"] + sk["E_in"])
    zum, zlm = float(np.asarray(zu(xm))), float(np.asarray(zl(xm)))
    zur, zlr = float(np.asarray(zu(xr))), float(np.asarray(zl(xr)))
    # fix round 3 (VS3-01): cap faces = box-cover level inside the body, glove loft outboard of the SOB cap ramp,
    # linear over the ramp (the glove skins stay on the loft, tapered filler between skin and cap)
    fr = 0.0 if body else sob_fraction(c, y)
    fum, flm = (zum, zlm) if body else (zt + fr * (zum - zt), zb + fr * (zlm - zb))
    fur, flr = (zur, zlr) if body else (zt + fr * (zur - zt), zb + fr * (zlr - zb))
    z_cmu, z_cml = fum - t_s - tm / 2, flm + t_s + tm / 2
    z_cru, z_crl = fur - t_s - tr / 2, flr + t_s + tr / 2
    Eud = ud["E1"]
    els += [(Eud, wm * tm, z_cmu, "main_cap_up"), (Eud, wm * tm, z_cml, "main_cap_lo"),
            (Eud, wr * tr, z_cru, "rear_cap_up"), (Eud, wr * tr, z_crl, "rear_cap_lo"),
            (E_sover, wm * t_s, zum - t_s / 2, "skin_over_main"), (E_sover, wm * t_s, zlm + t_s / 2, "skin_over_main_lo"),
            (E_sover, wr * t_s, zur - t_s / 2, "skin_over_rear"), (E_sover, wr * t_s, zlr + t_s / 2, "skin_over_rear_lo")]
    E_ = np.array([e[0] for e in els])
    A_ = np.array([e[1] for e in els])
    z_ = np.array([e[2] for e in els])
    zna = float((E_ * A_ * z_).sum() / (E_ * A_).sum())
    EI = float((E_ * A_ * (z_ - zna) ** 2).sum()) + 2 * Eud * (wm * tm ** 3 + wr * tr ** 3) / 12
    # extreme fibres: outer surface of the box skin (largest z over the box chord) and of the caps
    xx = np.linspace(xm, xr, 41)
    z_top_max = float(np.max(np.asarray(zu(xx))))
    z_bot_min = float(np.min(np.asarray(zl(xx))))
    if Y_SOB <= y < float(c.P["y_junction"]):
        # fix round 3 (VS3-04): the glove LERX skins ahead of the main spar are bonded primary skins strained with the
        # box: their deepest point (from 20 mm aft of the leading edge, solid LE laminate) counts as an extreme fibre
        x_le_ = float(np.interp(y, [s_["y"] for s_ in c.S["wing"]["sections"]],
                                [s_["x_le"] for s_ in c.S["wing"]["sections"]]))
        xl_ = np.linspace(x_le_ + 0.020, xm, 41)
        z_top_max = max(z_top_max, float(np.max(np.asarray(zu(xl_)))))
        z_bot_min = min(z_bot_min, float(np.min(np.asarray(zl(xl_)))))
    h_main = z_cmu - z_cml
    h_rear = z_cru - z_crl
    # torsion cell: mid-skin contour between the spar webs
    tsk = sk["t_out"] + sk["c"] + sk["t_in"]
    depth = np.asarray(zu(xx)) - np.asarray(zl(xx)) - tsk
    A_box = float(np.trapz(depth, xx))
    # first moments (shear split between the webs): material attached to each web, upper half
    Q_main = Eud * wm * tm * abs(z_cmu - zna) + 0.5 * float(sum(e[0] * e[1] * abs(e[2] - zna) for e in els
                                                                 if e[3] in ("skin_outer", "skin_inner")))
    Q_rear = Eud * wr * tr * abs(z_cru - zna) + 0.5 * float(sum(e[0] * e[1] * abs(e[2] - zna) for e in els
                                                                 if e[3] in ("skin_outer", "skin_inner")))
    return {"y": y, "chord": cT, "x_main": xm, "x_rear": xr, "body": body, "EI": EI, "z_na": zna,
            "z_cap_main": (z_cmu, z_cml), "z_cap_rear": (z_cru, z_crl), "z_top_max": z_top_max,
            "z_bot_min": z_bot_min, "h_main": h_main, "h_rear": h_rear, "A_box": A_box,
            "h_web_main": h_main - tm, "sob_fraction": fr,
            "box_width": xr - xm, "t_main_cap": tm, "t_rear_cap": tr, "w_main": wm, "w_rear": wr,
            "E_ud": Eud, "skin": sk, "f_main": Q_main / (Q_main + Q_rear), "t_skin_cap": t_s,
            "depth_main": zum - zlm, "depth_rear": zur - zlr,
            "E_face_top": sk["E_out"],
            "els": els}


def section_strains(sec: dict, M: float) -> dict:
    """Strains (positive = tension) at the caps and at the skin outer fibres for a positive (upward) bending moment M
    (upper surface in compression)."""
    k = M / sec["EI"]
    zna = sec["z_na"]
    return {"cap_main_up": -k * (sec["z_cap_main"][0] - zna), "cap_main_lo": -k * (sec["z_cap_main"][1] - zna),
            "skin_top": -k * (sec["z_top_max"] - zna), "skin_bot": -k * (sec["z_bot_min"] - zna),
            "cap_rear_up": -k * (sec["z_cap_rear"][0] - zna)}


def prong_panels(c: Ctx) -> list:
    """(y0, y1, length along the spar) of the glove main-spar web panels (fork prongs) between their supports: the
    side-of-body rib, the bush pads (pad_length centred on each pin), the glove rib and the joint rib (layout members;
    fix round 3, VS3-01)."""
    if "prong_panels" in c.cache:
        return c.cache["prong_panels"]
    L_ = c.S["layout"]
    WJ = L_["chassis"]["wing_joint"]["main_spar"]
    mem = {m_["id"]: m_ for m_ in L_["chassis"]["members"]}
    sw = joint_geometry(c)["sw"]
    hp = 0.5 * float(c.D["wing_joint"]["fork"]["pad_length_m"]) * math.cos(sw)
    sup = []
    for mid in ("M-SOB", "M-GLOVERIB", "M-JOINTRIB"):
        if mid in mem:
            b_ = mem[mid]["box"]
            sup.append((float(b_[0][1]), float(b_[1][1])))
    for p_ in WJ["pins"]:
        yp = float(p_["position"][1])
        sup.append((yp - hp, yp + hp))
    sup.sort()
    out = []
    for (a0, a1), (b0, b1) in zip(sup[:-1], sup[1:]):
        if b0 > a1 + 1e-6:
            out.append((a1, b0, (b0 - a1) / math.cos(sw)))
    c.cache["prong_panels"] = out
    return out


def prong_panel_at(c: Ctx, y: float) -> float:
    """Length (along the spar) of the prong web panel holding span station y."""
    for y0, y1, a in prong_panels(c):
        if y0 - 1e-6 <= y <= y1 + 1e-6:
            return a
    return max(a for _, _, a in prong_panels(c))


def prong_height(c: Ctx, caps: list, y: float) -> float:
    """Clear height of the glove main-spar webs (fork prongs) between the cap inner faces at span station y (fix round
    3, VS3-01: full-depth webs, not the 61 mm tongue slot)."""
    return float(wing_section(c, y, cap_plies_at(caps, y))["h_web_main"])


def wing_cap_criteria(c: Ctx, sec: dict, M_lim: float, M_neg_lim: float) -> list:
    """(name, applied limit, allowable ultimate, factor dict, unit, ref) for the spar caps and skins at a station."""
    f15 = total_factor(c)
    fc = total_factor(c, comp=True)
    ud = ply_props(c, c.D["wing"]["main_cap"]["material"])
    out = []
    e_p = section_strains(sec, M_lim)
    e_n = section_strains(sec, M_neg_lim)
    e_cap_c = max(-e_p["cap_main_up"], -e_n["cap_main_lo"])
    e_cap_t = max(e_p["cap_main_lo"], e_n["cap_main_up"])
    e_sk_c = max(-e_p["skin_top"], -e_n["skin_bot"])
    E = sec["E_ud"]
    out.append(("cap compression strain (DT, laminate > 2 mm)", e_cap_c * 1e6, c.dt["cap_comp"] * 1e6, f15, "µε",
                "STANAG 4703 UL13.1.2 (3000 µε at ultimate)"))
    out.append(("cap compression stress (A-basis est. 0.85 Fcu)", e_cap_c * E / 1e6,
                c.f["a_basis"] * ud["Fcu"] / 1e6, fc, "MPa", "CS-LUAS.613(b), AMC LUAS.619; materials Fcu B-basis ETW"))
    out.append(("cap tension strain (DT)", e_cap_t * 1e6, c.dt["cap_tens"] * 1e6, f15, "µε", "STANAG 4703 UL13.1.2"))
    out.append(("cap tension stress (A-basis est. 0.85 Ftu)", e_cap_t * E / 1e6, c.f["a_basis"] * ud["Ftu"] / 1e6, fc,
                "MPa", "CS-LUAS.613(b)"))
    sk = sec["skin"]
    if True:
        out.append(("skin face compression strain (DT, sandwich)", e_sk_c * 1e6, c.dt["sand_comp"] * 1e6, f15, "µε",
                    "STANAG 4703 UL13.1.2 (2600 µε sandwich skins)"))
        out.append(("skin outer face OHC stress", e_sk_c * sk["E_out"] / 1e6, qi_design_values(c)["OHC_Pa"] / 1e6, fc,
                    "MPa", "materials.yaml design_values_for_code OHC (PW-QI ETW2 B-basis)"))
        core = sk["core"]
        wr = ST.face_wrinkling(sk["E_out"], float(core["E"]), float(core["G"]))
        out.append(("skin outer face wrinkling", e_sk_c * sk["E_out"] / 1e6, wr / 1e6, fc, "MPa",
                    "Zenkert 1995: 0.5 (Ef Ec Gc)^(1/3), ROHACELL 51 WF minimum values"))
    return out


def research_value(file: str, dotted: str):
    """Value from a research YAML (data/research/<file>.yaml) by dotted path."""
    d = Z.research(file if file.endswith(".yaml") else file + ".yaml")
    for k in dotted.split("."):
        d = d[k]
    return d


def qi_design_values(c: Ctx) -> dict:
    """Quasi-isotropic carbon laminate design values (OHT, OHC, bearing, E, G) of materials.yaml (B-basis ETW,
    notched / damage-tolerant checks)."""
    if "qi" not in c.cache:
        dv = research_value("materials", "composites.laminates.design_values_for_code")
        c.cache["qi"] = {k: float(dv[k]) for k in ("OHT_Pa", "OHC_Pa", "bearing_Pa", "E_qi_Pa", "G_qi_Pa")}
    return c.cache["qi"]


def web_laminate(c: Ctx, n: int, key: str = "cfrp_pw_mtm45_as4", theta: float = 45.0) -> dict:
    pp = ply_props(c, key)
    return ST.laminate_abd([(pp, theta, pp["t"])] * int(n))


def web_criteria(c: Ctx, q_lim: float, b: float, n: int, a: float | None = None) -> list:
    """Solid +-45 PW web of n plies under a limit shear flow q (N/m), panel width b (long plate, or a finite panel of
    length a between stiffening supports: structlib.orthotropic_shear_buckling_finite, fix round 3): strength
    (first-ply failure, B-basis ETW ply allowables), damage-tolerance shear strain at ultimate, shear buckling at
    ultimate."""
    lam = web_laminate(c, n)
    eng = ST.laminate_engineering(lam)
    fpf = ST.first_ply_failure(lam, (0.0, 0.0, q_lim))
    Ncr = ST.orthotropic_shear_buckling_long(lam["D"], b) if a is None else \
        ST.orthotropic_shear_buckling_finite(lam["D"], a, b)
    fc = total_factor(c, comp=True)
    f15 = total_factor(c)
    gam = q_lim / (eng["Gxy"] * lam["h"])
    return [("web shear, first-ply failure (max stress / Tsai-Wu)", q_lim / 1e3, fpf["R"] * q_lim / 1e3, fc, "N/mm",
             "CLT, ply allowables B-basis ETW (spec.materials)"),
            ("web shear strain (DT, thin laminate)", gam * 1e6, c.dt["shear_thin"] * 1e6, f15, "µε",
             "STANAG 4703 UL13.1.2 (5200 µε at ultimate)"),
            (("web shear buckling (long plate, SS)" if a is None else
              f"web shear buckling (panel {a * 1000:.0f} x {b * 1000:.0f} mm, SS)"), q_lim / 1e3, Ncr / 1e3, fc, "N/mm",
             "Kollár & Springer long-plate shear buckling, CLT bending stiffness" if a is None else
             "Kollár & Springer long-plate fit on the short side x isotropic finite-length factor (5.35 + 4 (s/l)^2) "
             "/ 5.35 (Timoshenko & Gere), CLT bending stiffness - estimate")]


def _min_ms(crit: list) -> float:
    return min(cr[2] / (cr[1] * cr[3]["total"]) - 1.0 if cr[1] > 0 else math.inf for cr in crit)


def wing_web_station(c: Ctx, L: dict, y: float, n_cap: int, n_web: int) -> tuple:
    sec = wing_section(c, y, n_cap)
    V = at(L, "V", y)
    Tq = at(L, "T", y)
    q_T = Tq / (2 * sec["A_box"])
    q_main = sec["f_main"] * V / sec["h_main"] + q_T
    b_main = sec["depth_main"] - 2 * sec["t_skin_cap"] - 2 * sec["t_main_cap"]
    q_rear = (1 - sec["f_main"]) * V / sec["h_rear"] + q_T
    b_rear = sec["depth_rear"] - 2 * sec["t_skin_cap"] - 2 * sec["t_rear_cap"]
    return sec, q_main, b_main, q_rear, b_rear, q_T


def size_wing(c: Ctx) -> dict:
    """Spar-cap ply counts per zone (smallest count meeting every cap / skin criterion at 6 stations of the zone) and
    main-web ply counts per outer-panel zone (strength, DT strain, no buckling at ultimate)."""
    L = wing_loads(c)
    D = c.D["wing"]
    zb = D["main_cap"]["zone_breaks_y_m"]
    b2 = c.T["b2"]
    caps = []
    for y0, y1 in zip(zb[:-1], zb[1:]):
        ys = np.linspace(y0 + 1e-4, min(y1, b2 - 0.02) - 1e-4, 6)
        n = int(D["main_cap"]["min_plies"])
        while n < 200:
            ok = all(_min_ms(wing_cap_criteria(c, wing_section(c, y, n), *cap_moments(c, L, y))) >= 0.0
                     for y in ys)
            if ok:
                break
            n += 1
        caps.append([y0, y1, n])
    # the caps run continuously fork to fork; each zone carries the count of its worst station (internal 1:20 ply
    # drops / build-ups at the zone breaks, fix round 3: also inside the CT box, outside the kink-fitting bond)
    webs = []
    wz = D["main_web"]["zone_breaks_y_m"]
    for y0, y1 in zip(wz[:-1], wz[1:]):
        ys = np.linspace(y0 + 1e-4, min(y1, b2 - 0.02) - 1e-4, 6)
        n = int(D["main_web"]["min_plies"])
        while n < 60:
            ok = True
            for y in ys:
                ncap = cap_plies_at(caps, y)
                sec, qm, bm, qr, br, qT = wing_web_station(c, L, y, ncap, n)
                if _min_ms(web_criteria(c, qm, bm, n)) < 0.0:
                    ok = False
                    break
            if ok:
                break
            n += 1
        webs.append([y0, y1, n])
    return {"main_cap_zones": caps, "main_web_zones": webs}


def cap_plies_at(zones: list, y: float) -> int:
    for y0, y1, n in zones:
        if y0 <= y < y1 or (y >= y1 and zones[-1][1] == y1 and y1 == zones[-1][1] and y <= y1 + 1e-9 and
                            [y0, y1, n] == zones[-1]):
            return int(n)
    return int(zones[-1][2])


# =====================================================================================================================
# Turkish labels of the recurring criterion names (report)
# =====================================================================================================================
TR = {
    "cap compression strain (DT, laminate > 2 mm)": "başlık bası birim şekil değiştirmesi (hasar toleransı, > 2 mm)",
    "cap compression stress (A-basis est. 0.85 Fcu)": "başlık bası gerilmesi (A-tabanı tahmini 0,85 Fcu)",
    "cap tension strain (DT)": "başlık çekme birim şekil değiştirmesi (hasar toleransı)",
    "cap tension stress (A-basis est. 0.85 Ftu)": "başlık çekme gerilmesi (A-tabanı tahmini 0,85 Ftu)",
    "skin face compression strain (DT, sandwich)": "kaplama yüzü bası birim şekil değiştirmesi (sandviç, hasar toleransı)",
    "skin outer face OHC stress": "kaplama dış yüzü delikli bası (OHC) gerilmesi",
    "skin outer face wrinkling": "kaplama dış yüzü buruşma",
    "web shear, first-ply failure (max stress / Tsai-Wu)": "gövde (web) kesmesi, ilk katman hasarı (en büyük gerilme / Tsai-Wu)",
    "web shear strain (DT, thin laminate)": "gövde kesme birim şekil değiştirmesi (ince lamine, hasar toleransı)",
    "web shear buckling (long plate, SS)": "gövde kesme burkulması (uzun levha, basit mesnet)",
}


def tr(s: str) -> str:
    return TR.get(s, s)


def dec(x: float, nd: int = 2) -> str:
    """Number with a decimal comma (Turkish text)."""
    return f"{x:.{nd}f}".replace(".", ",")


def add_crit(R: Rows, id_: str, group: str, member: str, member_tr: str, case: str, case_tr: str, cr: tuple,
             part: str = "", note: str = "") -> float:
    name, app, alw, fac, unit, ref = cr
    return R.add(id_, group, f"{member}: {name}", f"{member_tr}: {tr(name)}", case, case_tr, app, alw, unit, fac, ref,
                 part=part, note=note)


def worst(crits: list) -> tuple:
    return min(crits, key=lambda cr: cr[2] / (cr[1] * cr[3]["total"]) if cr[1] > 0 else math.inf)


worst_crit = worst


def sandwich_lam(c: Ctx, sk: dict) -> dict:
    """Full sandwich stack (inner face, core as a soft isotropic layer, outer face) for bending stiffness."""
    core = sk["core"]
    Ec, Gc = float(core["E"]), float(core.get("G", core.get("G_W", 1e7)))
    cp = {"E1": Ec, "E2": Ec, "G12": Gc, "nu12": 0.3}
    # bottom-to-top stack: inner face (listed from its exposed surface = bottom), core, outer face (listed from its
    # exposed surface = top, so reversed)
    plies = [(pl[0], pl[1], pl[2]) for pl in sk["inner"]["plies"]] + [(cp, 0.0, sk["c"])] + \
            [(pl[0], pl[1], pl[2]) for pl in reversed(sk["outer"]["plies"])]
    lam = ST.laminate_abd(plies)
    A, B, Dm = lam["A"], lam["B"], lam["D"]
    lam["Dstar"] = Dm - B @ np.linalg.solve(A, B)
    lam["Gc"] = Gc
    lam["d"] = sk["c"] + 0.5 * (sk["t_out"] + sk["t_in"])
    return lam


def panel_buckling_multiplier(c: Ctx, sk: dict, Nx: float, Nxy: float, a: float, b: float) -> dict:
    """Load multiplier k on the limit running loads (Nx compression, Nxy shear) at which the sandwich panel a x b
    (SS edges, a along Nx) reaches the interaction Rc + Rs^2 = 1; buckling loads from the reduced bending stiffness
    D* with the transverse-shear correction."""
    lam = sandwich_lam(c, sk)
    Dst = lam["Dstar"]
    Nc = ST.sandwich_buckling(ST.orthotropic_compression_buckling(Dst, a, b), lam["Gc"], lam["d"], sk["c"])
    Ns = ST.sandwich_buckling(ST.orthotropic_shear_buckling_long(Dst, min(a, b)), lam["Gc"], lam["d"], sk["c"])
    Rc, Rs = max(Nx, 0.0) / Nc, abs(Nxy) / Ns
    if Rs < 1e-12:
        k = 1.0 / Rc if Rc > 0 else math.inf
    else:
        k = (-Rc + math.sqrt(Rc * Rc + 4 * Rs * Rs)) / (2 * Rs * Rs)
    tc = sk["t_out"] + sk["t_in"]
    crimp = ST.shear_crimping_stress(lam["Gc"], lam["d"], sk["c"], sk["t_out"], sk["t_in"]) * tc
    return {"k": k, "Nc": Nc, "Ns": Ns, "Rc": Rc, "Rs": Rs, "N_crimp": crimp}


def wing_skin_regions(c: Ctx) -> list:
    """Skin panel regions of the wing box: (tag, layup, y0, y1, panel length a between ribs, surface, label_en,
    label_tr, part). Upper skins are compressed by the positive design case, lower skins by the negative case."""
    D = c.D["wing"]
    b2 = c.T["b2"]
    pitch = float(c.S["structures"]["rib_pitch_m"])
    y_x = float(D["box_skin_upper_y_end_m"])
    up, prim, ct = D["box_skin_upper_layup"], D["skin_layup"], D["ct_box"]["cover_layup"]
    return [("CT", ct, 0.02, Y_SOB - 0.02, Y_SOB, "both", "CT-box cover", "orta kutu kapağı", "YK250-CH-001"),
            ("GLOVE-UP", up, Y_SOB + 0.01, 0.69, 0.15, "upper", "glove upper box skin", "eldiven üst kutu kaplaması",
             "YK250-SH-420"),
            ("OP-UP-IN", up, 0.71, y_x, pitch, "upper", f"outer-panel upper box skin y 0.70-{y_x:.2f}",
             f"dış panel üst kutu kaplaması y 0,70-{y_x:.2f}".replace(".", ","), "wing module"),
            ("OP-UP-OUT", prim, y_x, b2 - 0.1, pitch, "upper", f"outer-panel upper box skin y {y_x:.2f}-tip",
             f"dış panel üst kutu kaplaması y {y_x:.2f}-uç".replace(".", ","), "wing module"),
            ("GLOVE-LO", prim, Y_SOB + 0.01, 0.69, 0.15, "lower", "glove lower box skin (negative case)",
             "eldiven alt kutu kaplaması (negatif durum)", "YK250-SH-421"),
            ("OP-LO", prim, 0.71, b2 - 0.1, pitch, "lower", "outer-panel lower box skin (negative case)",
             "dış panel alt kutu kaplaması (negatif durum)", "wing module")]


def wing_skin_panels(c: Ctx, L: dict, caps: list) -> list:
    """Sandwich skin panels a x b (b = box width between the cap edges) of every region: smallest buckling load
    multiplier over 12 stations (compression of the surface + torsion shear flow, SS edges, D* with the transverse-shear
    correction) and the combined running load against the shear-crimping load."""
    out = []
    for tag, lay, ya, yb, a, surf, lab, lab_tr, part in wing_skin_regions(c):
        sk = skin_faces(c, lay)
        best = None
        for y in np.linspace(ya, yb, 12):
            sec = wing_section(c, y, cap_plies_at(caps, y))
            Mp_, Mn_ = cap_moments(c, L, y)
            e_up = -section_strains(sec, Mp_)["skin_top"]
            e_lo = -section_strains(sec, Mn_)["skin_bot"]
            e = {"upper": e_up, "lower": e_lo, "both": max(e_up, e_lo)}[surf]
            Nx = max(e, 0.0) * (sk["E_out"] * sk["t_out"] + sk["E_in"] * sk["t_in"])
            Nxy = at(L, "T", y) / (2 * sec["A_box"])
            b = sec["box_width"] - 0.5 * (sec["w_main"] + sec["w_rear"])
            pb = panel_buckling_multiplier(c, sk, Nx, Nxy, a, b)
            pb.update(y=y, Nx=Nx, Nxy=Nxy, b=b, e=e)
            if best is None or pb["k"] < best["k"]:
                best = pb
            cr = math.hypot(Nx, Nxy)
            if "N_comb" not in best or cr > best.get("N_comb_max", 0.0):
                best["N_comb_max"] = max(best.get("N_comb_max", 0.0), cr)
        lbl, lbl_tr = sk["label"], sk["label_tr"]
        out.append({"id": f"W-SKINBUCK-{tag}", "k": best["k"], "y": best["y"], "a": a, "b": best["b"],
                    "Nx": best["Nx"], "Nxy": best["Nxy"], "N_combined": best["N_comb_max"], "N_crimp": best["N_crimp"],
                    "layup": lay, "part": part,
                    "member": f"{lab} panel {a:.2f} x {best['b']:.2f} m ({lbl}) buckling, compression + shear "
                              f"(governing y {best['y']:.2f})",
                    "member_tr": f"{lab_tr} paneli {dec(a)} x {dec(best['b'])} m ({lbl_tr}) burkulma, bası + kesme "
                                 f"(belirleyici y {dec(best['y'])})",
                    "crimp_member": f"{lab} ({lbl}): shear crimping (core shear instability)",
                    "crimp_member_tr": f"{lab_tr} ({lbl_tr}): kesme kıvrılması (çekirdek kesme kararsızlığı)",
                    "ref": f"Rc + Rs^2 = 1; N_c {best['Nc'] / 1e3:.0f} N/mm, N_s {best['Ns'] / 1e3:.0f} N/mm (CLT D*, "
                           "sandwich shear correction); Nx from the section strain of the surface"})
    return out


def check_wing(c: Ctx, R: Rows, sized: dict) -> dict:
    """Wing spar caps and skins (bending), main and rear spar webs (shear + torsion), box-skin panels between ribs
    (compression + shear interaction, shear crimping), ribs (crushing / Brazier load), glove webs (fork prongs)."""
    L = wing_loads(c)
    D = c.D["wing"]
    caps, webs = sized["main_cap_zones"], sized["main_web_zones"]
    b2 = c.T["b2"]
    ir = bool(D.get("inertia_relief", False))
    case = (f"symmetric gust/manoeuvre n = {c.n_wing:.2f} at MTOM (limit), negative {c.n_wing_neg:.2f} (largest |n m| "
            "of the gust matrix at MTOM); Schrenk lift on the reference trapezoid, " +
            ("inertia relief of the wing structure, control surfaces and actuators" if ir else "no inertia relief"))
    case_tr = (f"simetrik hamle/manevra n = {c.n_wing:.2f}, MTOM (limit), negatif {c.n_wing_neg:.2f} (hamle matrisinin "
               "en büyük |n m| değeri MTOM'a göre); referans yamuk üzerinde Schrenk taşıma, " +
               ("kanat yapısı, kumanda yüzeyleri ve eyleyicilerin atalet rahatlatmasıyla" if ir else
                "atalet rahatlatması yok"))
    out = {"stations": []}
    for k, (y0, y1, n) in enumerate(caps):
        ys = np.linspace(y0 + 1e-4, min(y1, b2 - 0.02) - 1e-4, 6)
        cands = []
        for y in ys:
            sec = wing_section(c, y, n)
            for cr in wing_cap_criteria(c, sec, *cap_moments(c, L, y)):
                cands.append((y, sec, cr))
        y, sec, cr = min(cands, key=lambda t: t[2][2] / (t[2][1] * t[2][3]["total"]) if t[2][1] > 0 else math.inf)
        w = sec["w_main"] * 1000
        fz = Y_SOB <= y0 < float(c.P["y_junction"])
        add_crit(R, f"W-CAP-{k + 1:02d}", "wing", f"main spar cap y {y0:.2f}-{y1:.2f} m ({n} plies x {w:.0f} mm, "
                 f"governing y {y:.2f}" + (", glove-box moment = total - tongue moment)" if fz else ")"),
                 f"ana kiriş başlığı y {y0:.2f}-{y1:.2f} m ({n} kat x {w:.0f} mm, belirleyici y {y:.2f}" +
                 (", eldiven kutusu momenti = toplam - dil momenti)" if fz else ")"), case, case_tr, cr,
                 part="YK250-CH-001 / wing module" if y0 < 0.7 else "wing module (outer panel)")
    # rolling condition (CS-LUAS.349(a)): 100 % of the semispan airload at n_A on one side, 2/3 of it on the other
    # (CS-VLA 349(a)), aileron torsion in the torque envelope; the per-side bending is below the design case
    nA = float(c.S["structures"]["n_limit_pos"])
    M_roll = at(L, "M", 0.0) * nA / c.n_wing
    R.info("W-ROLL", "wing", "rolling condition: root bending of the more loaded side (100 % at n_A) vs the design case",
           "yuvarlanma durumu: daha yüklü tarafın kök eğilme momenti (n_A'da %100) / tasarım durumu", 
           f"CS-LUAS.349(a): n_A {nA:g} on one side, 2/3 on the other, full aileron torsion at VA",
           f"CS-LUAS.349(a): bir tarafta n_A {nA:g}, diğerinde 2/3, VA'da tam kanatçık burulması", M_roll, "N m",
           f"covered: design-case root moment {at(L, 'M', 0.0):.0f} N m (n {c.n_wing:.2f}); the asymmetry is reacted "
           "by the roll inertia (the CT-box centre moment is the mean of both sides)", part="YK250-CH-001",
           requirement=at(L, "M", 0.0))
    # full criteria at the key stations (CT-box centre, glove root, outer-panel root)
    for tag, y in (("CT", 0.0), ("GLOVE", 0.41), ("OP", 0.71)):
        n = cap_plies_at(caps, y)
        sec = wing_section(c, y, n)
        out["stations"].append({"y": y, "plies": n, "EI": sec["EI"], "h_main": sec["h_main"], "A_box": sec["A_box"]})
        for j, cr in enumerate(wing_cap_criteria(c, sec, at(L, "M", y), at(L, "M_neg", y))):
            add_crit(R, f"W-SEC-{tag}-{j + 1}", "wing", f"wing section y {y:.2f} m", f"kanat kesiti y {y:.2f} m", case,
                     case_tr, cr)
    # main web (outer panel, solid +-45), rear web
    case_w = case + "; torsion: cm0 at VD, full aileron at VA (CS-LUAS.349(b)), 40 deg flap at VF (envelope)"
    case_wtr = case_tr + "; burulma: VD'de cm0, VA'da tam kanatçık (CS-LUAS.349(b)), VF'de 40° flap (zarf)"
    for k, (y0, y1, n) in enumerate(webs):
        ys = np.linspace(y0 + 1e-4, min(y1, b2 - 0.02) - 1e-4, 6)
        cands = []
        for y in ys:
            sec, qm, bm, qr, br, qT = wing_web_station(c, L, y, cap_plies_at(caps, y), n)
            for cr in web_criteria(c, qm, bm, n):
                cands.append((y, cr))
        y, cr = min(cands, key=lambda t: t[1][2] / (t[1][1] * t[1][3]["total"]))
        add_crit(R, f"W-WEB-{k + 1:02d}", "wing", f"main spar web y {y0:.2f}-{y1:.2f} m ({n} plies +-45 PW, governing "
                 f"y {y:.2f})", f"ana kiriş gövdesi y {y0:.2f}-{y1:.2f} m ({n} kat ±45 PW, belirleyici y {y:.2f})",
                 case_w, case_wtr, cr, part="wing module (outer panel)")
    nr = int(D["rear_web"]["plies"])
    cands = []
    for y in np.linspace(0.71, b2 - 0.05, 30):
        sec, qm, bm, qr, br, qT = wing_web_station(c, L, y, cap_plies_at(caps, y), 3)
        for cr in web_criteria(c, qr, br, nr):
            cands.append((y, cr))
    y, cr = min(cands, key=lambda t: t[1][2] / (t[1][1] * t[1][3]["total"]))
    add_crit(R, "W-RWEB", "wing", f"rear spar web outer panel ({nr} plies +-45, governing y {y:.2f})",
             f"arka kiriş gövdesi dış panel ({nr} kat ±45, belirleyici y {y:.2f})", case_w, case_wtr, cr)
    # glove main-spar webs (= fork prongs, CFRP +-45) inboard of the inner pin: wing shear (the pin couple between
    # the pins is J-PRONG-WEB)
    wj = c.S["layout"]["chassis"]["wing_joint"]["main_spar"]
    npr = int(c.D["wing_joint"]["fork"]["prong_plies"])
    # fix round 3 (VS3-01): the prongs are the full-depth glove main-spar webs between the cap inner faces (the cap
    # ramp shortens them over Y_SOB-Y_RAMP); shear and buckling at the stations inboard of the inner pin
    y_in = min(float(p_["position"][1]) for p_ in wj["pins"])
    cands = []
    for y in np.linspace(Y_SOB + 0.005, y_in, 6):
        h_pr = prong_height(c, caps, y)
        cands += web_criteria(c, at(L, "V", y) / (2 * h_pr), h_pr, npr, a=prong_panel_at(c, y))
    h_lo = prong_height(c, caps, Y_SOB + 0.005)
    add_crit(R, "W-GLOVEWEB", "wing", f"glove main-spar webs (2 x {npr} plies +-45 PW, the fork prongs, full depth "
             f"between the cap faces, h {h_lo * 1000:.0f} mm at the SOB) inboard of the inner pin, shear",
             f"iç pimin içinde eldiven ana kiriş gövdeleri (2 x {npr} kat ±45 PW, çatal kulakları, başlık yüzleri arasında "
             f"tam derinlik, SOB'da h {h_lo * 1000:.0f} mm), kesme", case, case_tr, worst(cands), part="YK250-CH-001")
    # skin panels between ribs: compression (from the section strain of the surface in compression) + torsion shear;
    # shear crimping of every skin region (incl. the D-nose / trailing-edge skins at the box-skin strain)
    for row in wing_skin_panels(c, L, caps):
        R.add(row["id"], "wing", row["member"], row["member_tr"], case_w, case_wtr, 1.0, row["k"], "load multiplier",
              total_factor(c, comp=True), row["ref"], part=row["part"])
        R.add(row["id"].replace("BUCK", "CRIMP"), "wing", row["crimp_member"], row["crimp_member_tr"], case_w, case_wtr,
              row["N_combined"] / 1e3, row["N_crimp"] / 1e3, "N/mm", total_factor(c, comp=True),
              "Zenkert 1995: N = G_c d^2 / c (core minimum G); N = hypot(Nx, Nxy)", part=row["part"])
    pitch = float(c.S["structures"]["rib_pitch_m"])
    # ribs: crushing (Brazier) load at the most loaded outer-panel rib, rib web as a sandwich column
    rib = skin_faces(c, "rib_panel")
    best = None
    for y in np.linspace(0.71, 2.0, 20):
        n = cap_plies_at(caps, y)
        sec = wing_section(c, y, n)
        kap = at(L, "M", y) / sec["EI"]
        P_up = sum(e[0] * e[1] * abs(e[2] - sec["z_na"]) for e in sec["els"] if e[2] > sec["z_na"])
        w = pitch * kap ** 2 * P_up / sec["box_width"]
        if best is None or w > best[1]:
            best = (y, w, sec)
    y, w, sec = best
    rl = sandwich_lam(c, rib)
    h_rib = sec["depth_main"]
    Ncol = ST.sandwich_buckling(math.pi ** 2 * rl["Dstar"][1, 1] / h_rib ** 2, rl["Gc"], rl["d"], rib["c"])
    R.add("W-RIBCRUSH", "wing", f"wing rib web (rib_panel sandwich) crushing load, column buckling (y {y:.2f}, pitch "
          f"{pitch:.2f} m)", f"kanat kaburgası gövdesi (rib_panel sandviç) ezilme yükü, kolon burkulması (y {y:.2f}, "
          f"aralık {pitch:.2f} m)", case, case_tr, w / 1e3, Ncol / 1e3, "N/mm", total_factor(c, comp=True),
          "Niu ch. 9 / Brazier: w = s kappa^2 sum(E A |z|) / b; Euler wide column with shear correction")
    out["L"] = L
    return out


def shear_out(P_t: float, e: float, D: float, Fsu: float) -> float:
    """Shear-out capacity of a lug / bore loaded toward an edge at distance e (centre to edge): two shear planes at
    40 deg, area 2 t (e - D/2 cos 40) (Bruhn D1); returns the allowable per unit thickness x t (P_t = thickness)."""
    return 2.0 * P_t * (e - 0.5 * D * math.cos(math.radians(40.0))) * Fsu


def joint_geometry(c: Ctx) -> dict:
    """Outer-panel joint geometry from the layout: main pins (inner / outer), sweep of the pin line, joint plane, arm
    of the outer pin to the joint plane along the spar, rear pin and its plan distance to the main-spar pin line."""
    if "joint_geom" in c.cache:
        return c.cache["joint_geom"]
    WJ = c.S["layout"]["chassis"]["wing_joint"]
    pins = sorted((np.asarray(p["position"], float) for p in WJ["main_spar"]["pins"]), key=lambda p: p[1])
    p1, p2 = pins[0], pins[-1]
    d = float(np.linalg.norm(p2 - p1))
    sw = math.atan2(abs(p2[0] - p1[0]), abs(p2[1] - p1[1]))
    yj = float(c.P["y_junction"])
    a = (yj - p2[1]) / math.cos(sw)
    pr = np.asarray(WJ["rear_spar"]["pin"]["position"], float)
    u = (p2 - p1)[:2] / np.linalg.norm((p2 - p1)[:2])
    w = (pr - p1)[:2]
    dx = abs(w[0] * u[1] - w[1] * u[0])
    out = {"p_in": p1, "p_out": p2, "y_in": float(p1[1]), "y_out": float(p2[1]), "d": d, "sw": sw, "yj": yj, "a": a,
           "p_rear": pr, "dx": dx, "D": float(WJ["main_spar"]["pins"][0]["diameter"])}
    c.cache["joint_geom"] = out
    return out


def tongue_moment(c: Ctx, L: dict, y: float, neg: bool = False) -> float:
    """Bending moment (x-axis frame of the wing loads, limit) carried by the CFRP tongue at span station y: the joint
    plane moment + shear x arm outboard of the outer pin, falling linearly to zero at the inner pin between the pins
    (moment of the inner-pin reaction), zero inboard of it."""
    g = joint_geometry(c)
    kV, kM = ("V_neg", "M_neg") if neg else ("V", "M")
    if y >= g["yj"] or y <= g["y_in"]:
        return 0.0
    Vj, Mj = at(L, kV, g["yj"]), at(L, kM, g["yj"])
    if y >= g["y_out"]:
        return Mj + Vj * (g["yj"] - y)
    return (Mj + Vj * (g["yj"] - g["y_out"])) * (y - g["y_in"]) / (g["y_out"] - g["y_in"])


def cap_moments(c: Ctx, L: dict, y: float) -> tuple:
    """(positive, negative) limit bending moment of the main-spar caps at y: the wing moment, minus the tongue moment
    in the fork zone of the glove box (structures.sizing.wing.main_cap.fork_zone)."""
    M, Mn = at(L, "M", y), at(L, "M_neg", y)
    if Y_SOB <= y < float(c.P["y_junction"]):
        M -= tongue_moment(c, L, y)
        Mn -= tongue_moment(c, L, y, neg=True)
    return M, Mn


def joint_cases(c: Ctx) -> list:
    """Load cases at the outer-panel joint plane (limit): vertical shear V, bending M, torsion T (envelope) and the
    in-plane loads: chordwise force C (+ forward) and the drag moment Mz about the vertical axis (fix round 1, S1-02):
    PHAA (C = L tan(alpha_section) at the wing CLmax, Mz = M tan(alpha)), NHAA (the same at the negative CLmin of
    structures.clmin_negative, mass MTOM, n_wing_neg), VD drag (section cd at the VD 1 g lift coefficient, aft)."""
    L = wing_loads(c)
    g = joint_geometry(c)
    yj = g["yj"]
    S = c.S
    V, M, Tq = at(L, "V", yj), at(L, "M", yj), at(L, "T", yj)
    Vn, Mn = at(L, "V_neg", yj), at(L, "M_neg", yj)
    ar = S["aero"]["reference"]
    inc = math.radians(float(c.P["incidence_deg"]))
    a_pos = math.radians(L["alpha_section_at_clmax_deg"])
    a_neg = (float(S["structures"]["clmin_negative"]) - float(ar["CL_0"])) / float(ar["CL_alpha_per_rad"]) + inc
    b2 = c.T["b2"]
    ys = np.linspace(yj, b2, 121)
    cc = c.T["c"](ys)
    Re = Z.AL.reynolds(c.VD, float(c.T["c"](yj)), 0.0)
    ch = AE.characteristics("nlf416", Re)
    cl_vd = c.m0 * G / (c.q(c.VD) * float(S["wing"]["area"]))
    cd = AE.section_cd(ch, cl_vd)
    D_vd = float(np.trapz(c.q(c.VD) * cc * cd, ys))
    Mz_vd = float(np.trapz(c.q(c.VD) * cc * cd * (ys - yj), ys))
    return [
        {"id": "PHAA", "V": V, "M": M, "T": Tq, "C": V * math.tan(a_pos), "Mz": M * math.tan(a_pos),
         "text": f"PHAA: n {c.n_wing:.2f}, alpha_section {math.degrees(a_pos):.1f} deg (wing CLmax)",
         "text_tr": f"PHAA: n {c.n_wing:.2f}, kesit açısı {dec(math.degrees(a_pos), 1)}° (kanat CLmaks)"},
        {"id": "NHAA", "V": abs(Vn), "M": abs(Mn), "T": Tq, "C": abs(Vn) * math.tan(abs(a_neg)),
         "Mz": abs(Mn) * math.tan(abs(a_neg)),
         "text": f"NHAA: n {c.n_wing_neg:.2f}, alpha_section {math.degrees(a_neg):.1f} deg (CLmin "
                 f"{float(S['structures']['clmin_negative']):g})",
         "text_tr": f"NHAA: n {dec(c.n_wing_neg)}, kesit açısı {dec(math.degrees(a_neg), 1)}° (CLmin "
                    f"{dec(float(S['structures']['clmin_negative']), 1)})"},
        {"id": "VD-DRAG", "V": c.m0 * G / 2 * (V / max(c.n_wing * c.m0 * G / 2, 1e-9)), "M": M / c.n_wing,
         "T": Tq, "C": -D_vd, "Mz": -Mz_vd,
         "text": f"VD {c.VD:.1f} m/s, 1 g, section cd {cd:.4f} on the outer panel (drag aft {D_vd:.0f} N)",
         "text_tr": f"VD {dec(c.VD, 1)} m/s, 1 g, dış panelde kesit cd {dec(cd, 4)} (geriye sürükleme {D_vd:.0f} N)"}]


def check_wing_joint(c: Ctx, R: Rows, sized: dict) -> dict:
    """Outer-panel joint y 0.70 (layout.chassis.wing_joint, structures.sizing.wing_joint; fix round 1, S1-02/S1-03/S1-05):
    CFRP tongue (outer-panel spar stub) in the CFRP fork of the glove main-spar box on two chordwise Ti pins (vertical
    couple R_in / R_out), rear-spar 7075 lug in a slot fitting on a vertical Ti pin. Vertical: R_in = (M/cos(sweep) +
    V a)/d, R_out = R_in + V, torsion couple F_T = T/dx (main pins / rear slot plates). In-plane: chordwise force C on
    the rear pin; drag moment Mz as a spanwise couple F_y = Mz/dx (main pins F_y/2 each, rear pin F_y). Pins, bushes,
    bores and lugs: frequent-assembly factor 1.5 (x FoS), bearing factor 2.0 on bearing; laminates: composite factor
    1.2, A-basis 0.85 on the single-load-path tongue flanges; bolted fittings: fitting factor 1.15."""
    L = wing_loads(c)
    S = c.S
    WJ = S["layout"]["chassis"]["wing_joint"]
    JD = c.D["wing_joint"]
    g = joint_geometry(c)
    d, a, sw, dx, D_ = g["d"], g["a"], g["sw"], g["dx"], g["D"]
    ti, al = mat(c, "ti_6al_4v_annealed_sheet"), mat(c, "al_7075_t651_plate")
    tg, fk, bu, rr = JD["tongue"], JD["fork"], JD["bush"], JD["rear"]
    st4 = mat(c, bu["material"])
    ud, pw = ply_props(c, tg["flange_material"]), ply_props(c, tg["web_material"])
    udm = mat(c, tg["flange_material"])
    qi = qi_design_values(c)
    fa = total_factor(c, fa=True)
    fb = total_factor(c, bear=True, fa=True)
    fit = total_factor(c, fit=True, comp=True)
    res = []
    for cs in joint_cases(c):
        Ms = cs["M"] / math.cos(sw)
        R_in = (Ms + cs["V"] * a) / d
        R_out = R_in + cs["V"]
        F_T = cs["T"] / dx
        F_y = cs["Mz"] / dx
        P_out = math.hypot(R_out + F_T, 0.5 * F_y)
        P_in = math.hypot(R_in + F_T, 0.5 * F_y)
        P_rear = math.hypot(cs["C"], F_y)
        res.append(dict(cs, R_in=R_in, R_out=R_out, F_T=F_T, F_y=F_y, P_out=P_out, P_in=P_in, P_rear=P_rear,
                        M_t=R_in * d))

    def case_txt(r):
        return (f"{r['text']}; joint plane y {g['yj']:.2f}: V {r['V']:.0f} N, M {r['M']:.0f} N m, T {r['T']:.0f} N m, "
                f"C {r['C']:.0f} N, Mz {r['Mz']:.0f} N m (limit); pins {d * 1000:.0f} mm apart: R_in {r['R_in']:.0f} N, "
                f"R_out {r['R_out']:.0f} N, torsion couple {r['F_T']:.0f} N, drag couple F_y {r['F_y']:.0f} N (dx "
                f"{dx * 1000:.0f} mm)",
                f"{r['text_tr']}; birleşim düzlemi y {dec(g['yj'])}: V {r['V']:.0f} N, M {r['M']:.0f} N m, T {r['T']:.0f} "
                f"N m, C {r['C']:.0f} N, Mz {r['Mz']:.0f} N m (limit); pimler {d * 1000:.0f} mm aralıklı: R_iç "
                f"{r['R_in']:.0f} N, R_dış {r['R_out']:.0f} N, burulma çifti {r['F_T']:.0f} N, sürükleme çifti F_y "
                f"{r['F_y']:.0f} N (dx {dx * 1000:.0f} mm)")

    def worst(key):
        return max(res, key=lambda r: abs(r[key]))
    # main pins
    r = worst("P_out")
    ct, ct_tr = case_txt(r)
    A = math.pi * D_ ** 2 / 4
    R.add("J-PIN-SHEAR", "wing joint", f"main pin d {D_ * 1000:.0f} Ti-6Al-4V (outer pin, resultant of the vertical "
          "couple, torsion couple and F_y/2), double shear", f"ana pim Ø{D_ * 1000:.0f} Ti-6Al-4V (dış pim; düşey çift, "
          "burulma çifti ve F_y/2 bileşkesi), çift kesme", ct, ct_tr, r["P_out"], 2 * A * float(ti["Fsu"]), "N", fa,
          "Fsu Ti-6Al-4V annealed (sheet values as bar estimate); STANAG UL2.4 frequent assembly 1.5", part="P-MAIN1/2")
    t_pr = float(fk["prong_pad_t_m"])
    t_tg = float(tg["width_m"])
    gap = 0.5 * (float(WJ["main_spar"]["fork"]["slot"]["width"]) - t_tg)
    Mp = ST.pin_bending_moment(r["P_out"], t_pr, t_tg, gap)
    R.add("J-PIN-BEND", "wing joint", "main pin bending (Melcon-Hoblit arm, no plastic credit)",
          "ana pim eğilmesi (Melcon-Hoblit kolu, plastik kazanç yok)", ct, ct_tr, ST.pin_bending_stress(Mp, D_) / 1e6,
          float(ti["Ftu"]) / 1e6, "MPa", fa, f"M = P/2 (t_prong {t_pr * 1000:.0f}/2 + gap {gap * 1000:.1f} + t_tongue "
          f"{t_tg * 1000:.0f}/4 mm); Ftu", part="P-MAIN1/2")
    Lp, Lt, OD = float(bu["length_prong_m"]), float(bu["length_tongue_m"]), float(bu["od_m"])
    R.add("J-PIN-BUSH", "wing joint", f"main pin bearing in the 4130 prong bushes (2 x {Lp * 1000:.0f} mm)",
          f"ana pimin 4130 kulak burçlarında ezilmesi (2 x {Lp * 1000:.0f} mm)", ct, ct_tr, r["P_out"],
          2 * D_ * Lp * float(st4["Fbru"]), "N", fb, "MMPDS 4130 Fbru (bush backed by the laminate); bearing factor 2.0",
          part="YK250-CH-053")
    R.add("J-TONGUE-BUSH", "wing joint", f"tongue boss: bush OD {OD * 1000:.0f} x {Lt * 1000:.0f} mm bearing on the "
          "[+-45/0/90] block", f"dil göbeği: burç dış çapı {OD * 1000:.0f} x {Lt * 1000:.0f} mm, [±45/0/90] blokta ezilme",
          ct, ct_tr, r["P_out"] / (OD * Lt) / 1e6, qi["OHC_Pa"] / 1e6, "MPa", fb,
          "QI open-hole compression ETW B-basis as a conservative bearing limit (bore e/D below the 3 of the QI bearing "
          "value; element test required); bearing factor 2.0", part="YK250-WG-151")
    R.add("J-PRONG-BUSH", "wing joint", f"fork prong pads ({t_pr * 1000:.0f} mm): bush OD {OD * 1000:.0f} x "
          f"{Lp * 1000:.0f} mm bearing, two prongs", f"çatal kulak takviyeleri ({t_pr * 1000:.0f} mm): burç dış çapı "
          f"{OD * 1000:.0f} x {Lp * 1000:.0f} mm ezilmesi, iki kulak", ct, ct_tr, r["P_out"] / (2 * OD * Lp) / 1e6,
          qi["OHC_Pa"] / 1e6, "MPa", fb, "QI OHC ETW as the bearing limit (as J-TONGUE-BUSH)", part="YK250-CH-001")
    bl = float(tg["boss_length_m"])
    R.add("J-TONGUE-FLANGE-TT", "wing joint", f"tongue UD flange: through-thickness compression under the boss block "
          f"({t_tg * 1000:.0f} x {bl * 1000:.0f} mm)", f"dil UD başlığı: göbek bloğu altında kalınlık yönünde bası "
          f"({t_tg * 1000:.0f} x {bl * 1000:.0f} mm)", ct, ct_tr, r["P_out"] / (t_tg * bl) / 1e6,
          float(udm["F2cu"]) / 1e6, "MPa", fa, "UD transverse compression F2cu (B-basis ETW); frequent assembly 1.5",
          part="YK250-WG-151")
    # tongue flanges at the outer pin (largest tongue moment), axial F_y/2 added
    r = max(res, key=lambda q: q["M_t"])
    ct, ct_tr = case_txt(r)
    h, tf = float(tg["height_m"]), float(tg["flange_t_m"])
    A_f = t_tg * tf
    F_f = r["M_t"] / (h - tf) + 0.5 * abs(r["F_y"])
    eps = F_f / (ud["E1"] * A_f)
    R.add("J-TONGUE-FLANGE-DT", "wing joint", f"tongue UD flange {t_tg * 1000:.0f} x {tf * 1000:.0f} mm at the outer pin "
          "(M = R_in d, + F_y/2 axial), compression strain", f"dış pimde dil UD başlığı {t_tg * 1000:.0f} x "
          f"{tf * 1000:.0f} mm (M = R_iç d, + F_y/2 eksenel), bası birim şekil değiştirmesi", ct, ct_tr, eps * 1e6,
          c.dt["cap_comp"] * 1e6, "µε", total_factor(c), "STANAG 4703 UL13.1.2 (3000 µε at ultimate); boss block not "
          "credited", part="YK250-WG-151")
    R.add("J-TONGUE-FLANGE", "wing joint", "tongue UD flange at the outer pin, compression stress (single load path)",
          "dış pimde dil UD başlığı, bası gerilmesi (tek yük yolu)", ct, ct_tr, eps * ud["E1"] / 1e6,
          c.f["a_basis"] * float(udm["Fcu"]) / 1e6, "MPa", total_factor(c, comp=True),
          "A-basis estimate 0.85 Fcu (CS-LUAS.613(b)); composite factor 1.2", part="YK250-WG-151")
    TP = tg.get("flange_taper")
    if TP:
        # flange ply drop between the pins: the tongue moment falls linearly from the outer pin (M_t) to zero at the
        # inner pin (tongue_moment), the flange thickness linearly from tf to t_min; strain at 21 stations + the
        # 1:20 ply-drop slope (materials processes.composite_moulding)
        t_min = float(TP["t_min_m"])
        L_tp = (g["y_out"] - g["y_in"]) / math.cos(sw)
        w_tp = (0.0, 0.0, 0.0)
        for f_ in np.linspace(0.0, 0.95, 20):
            t_q = tf - f_ * (tf - t_min)
            F_q = (1.0 - f_) * r["M_t"] / (h - t_q) + 0.5 * abs(r["F_y"])
            e_q = F_q / (ud["E1"] * t_tg * t_q)
            if e_q > w_tp[0]:
                w_tp = (e_q, f_, t_q)
        R.add("J-TONGUE-TAPER", "wing joint", f"tongue UD flange ply drop {tf * 1000:.0f} -> {t_min * 1000:.0f} mm between "
              f"the pins: compression strain at the taper stations (worst at {w_tp[1] * 100:.0f} % of the pin spacing, "
              f"flange {w_tp[2] * 1000:.1f} mm)", f"pimler arasında dil UD başlığı kat düşürme {tf * 1000:.0f} -> "
              f"{t_min * 1000:.0f} mm: inceltme istasyonlarında bası birim şekil değiştirmesi (en kötü istasyon dış pimden "
              f"x/s = {dec(w_tp[1])}, başlık {dec(w_tp[2] * 1000, 1)} mm)", ct, ct_tr, w_tp[0] * 1e6,
              c.dt["cap_comp"] * 1e6, "µε", total_factor(c), "M(y) linear from the outer pin to zero at the inner pin "
              "(tongue_moment) + F_y/2 axial; STANAG 4703 UL13.1.2 - fix round 2 (mass closure)", part="YK250-WG-151")
        R.add("J-TONGUE-TAPER-DROP", "wing joint", "tongue flange ply-drop slope (rise / run along the spar) vs the 1:20 "
              "rule", "dil başlığı kat düşürme eğimi (kiriş boyunca kalınlık değişimi / uzunluk), 1:20 kuralı", "geometry",
              "geometri", (tf - t_min) / L_tp, 1.0 / 20.0, "-", {"fos": 1.0, "special": 1.0, "total": 1.0,
                                                                  "text": "geometric rule"},
              "materials.yaml processes.composite_moulding taper_slope_main_load_direction (1:20 minimum)",
              part="YK250-WG-151", kind="geometry")
    hw = h - 2 * tf
    nw = int(tg["web_plies"])
    crit = web_criteria(c, r["R_in"] / hw, hw, nw)
    add_crit(R, "J-TONGUE-WEB", "wing joint", f"tongue +-45 web ({nw} plies) between the pins, shear R_in",
             f"pimler arasında dil ±45 gövdesi ({nw} kat), kesme R_iç", ct, ct_tr, worst_crit(crit), part="YK250-WG-151")
    r = worst("R_out")
    ct, ct_tr = case_txt(r)
    npr = int(fk["prong_plies"])
    crit = []                                       # fix round 3 (VS3-01): full-depth prongs between the cap faces
    for y_q in np.linspace(g["y_in"], g["y_out"], 9):
        hs = prong_height(c, sized["main_cap_zones"], y_q)
        crit += web_criteria(c, r["R_out"] / (2 * hs), hs, npr, a=prong_panel_at(c, y_q))
    add_crit(R, "J-PRONG-WEB", "wing joint", f"fork prongs (glove main-spar webs, 2 x {npr} plies +-45, full depth "
             "between the cap faces) between the pins, shear R_out", f"pimler arasında çatal kulakları (eldiven ana "
             f"kiriş gövdeleri, 2 x {npr} kat ±45, başlık yüzleri arasında tam derinlik), kesme R_dış", ct, ct_tr,
             worst_crit(crit), part="YK250-CH-001")
    # depth transition tongue -> outer-panel caps (fix round 2, VS2-04)
    TRN = JD.get("transition")
    if TRN:
        caps_z = sized["main_cap_zones"]
        yj_ = g["yj"]
        L_tr, L_i = float(TRN["length_m"]), float(TRN["intro_length_m"])
        y_e = yj_ + L_tr
        sec_e = wing_section(c, y_e + 0.002, cap_plies_at(caps_z, y_e + 0.002))
        h_t = h - tf
        step = 0.5 * (sec_e["h_main"] - h_t)
        th = math.atan2(abs(step), L_tr / math.cos(sw))
        rP = max(res, key=lambda q: q["M"])
        ctp, ctp_tr = case_txt(rP)
        F_e = at(L, "M", y_e) / sec_e["h_main"]
        Fk_e = F_e * math.sin(th)
        Fk_0 = (rP["M_t"] / h_t) * math.sin(th)
        t_wt = nw * pw["t"]
        R.add("J-TRANS-WEB", "wing joint", f"cap ramp (step {step * 1000:.1f} mm per flange over {L_tr * 1000:.0f} mm, "
              f"{math.degrees(th):.1f} deg): kink force at the outboard end into the {nw}-ply tongue web, web crushing "
              f"over {L_i * 1000:.0f} mm", f"başlık rampası (başlık başına {dec(step * 1000, 1)} mm basamak, "
              f"{L_tr * 1000:.0f} mm boyunca, {dec(math.degrees(th), 1)}°): dış uçta kırılma kuvveti {nw} katlı dil "
              f"gövdesine, {L_i * 1000:.0f} mm üzerinde gövde ezilmesi", ctp, ctp_tr, Fk_e / (t_wt * L_i) / 1e6,
              qi["OHC_Pa"] / 1e6, "MPa", total_factor(c, comp=True), "F_k = F_cap sin(theta); QI OHC ETW as the "
              "in-plane compression limit of the web - fix round 2, VS2-04", part="YK250-WG-151")
        ilss_ = float(research_value("materials", "composites.laminae.cfrp_pw_ooa_prepreg_as4_193.strength_Pa.ETW."
                                                  "ILSS.b_basis"))
        R.add("J-TRANS-ILSS", "wing joint", "cap ramp: flange-to-web interlaminar shear of the kink force (flange width "
              f"x {L_i * 1000:.0f} mm)", f"başlık rampası: kırılma kuvvetinin başlık-gövde ara yüzey kesmesi (başlık "
              f"genişliği x {L_i * 1000:.0f} mm)", ctp, ctp_tr, max(Fk_e, Fk_0) / (t_tg * L_i) / 1e6, ilss_ / 1e6, "MPa",
              total_factor(c, comp=True), "ILSS ETW B-basis (NCAMP PW) - fix round 2, VS2-04", part="YK250-WG-151")
        t_rl = int(TRN["root_rib_land_plies"]) * pw["t"]
        R.add("J-TRANS-RIB", "wing joint", f"cap ramp: kink force at the joint rib into its {int(TRN['root_rib_land_plies'])}"
              f"-ply solid land ({t_rl * 1000:.1f} x {t_tg * 1000:.0f} mm)", f"başlık rampası: birleşim kaburgasında "
              f"kırılma kuvveti {int(TRN['root_rib_land_plies'])} katlı dolu banda ({dec(t_rl * 1000, 1)} x "
              f"{t_tg * 1000:.0f} mm)", ctp, ctp_tr, Fk_0 / (t_rl * t_tg) / 1e6, qi["OHC_Pa"] / 1e6, "MPa",
              total_factor(c, comp=True), "QI OHC ETW - fix round 2, VS2-04", part="YK250-WG-150")
        A_t = t_tg * tf
        worst_e = (0.0, 0.0, 0.0)
        for fr_ in (0.0, 0.5, 1.0):
            y_q = yj_ + 0.002 + fr_ * (L_tr - 0.004)
            sq = wing_section(c, y_q, cap_plies_at(caps_z, y_q))
            A_o = sq["w_main"] * sq["t_main_cap"]
            dlt = (1.0 - fr_) * step
            A_q = A_t + fr_ * (A_o - A_t)
            els = []
            n_db = int(TRN.get("root_bay_skin_doubler_plies_per_face", 0))
            db_faces = (("skin_outer", "skin_outer_lo") if TRN.get("root_bay_doubler_faces", "both") == "outer"
                        else ("skin_outer", "skin_outer_lo", "skin_inner", "skin_inner_lo"))
            for e_ in sq["els"]:
                if e_[3] == "main_cap_up":
                    els.append((e_[0], A_q, e_[2] - dlt, e_[3]))
                elif e_[3] == "main_cap_lo":
                    els.append((e_[0], A_q, e_[2] + dlt, e_[3]))
                else:
                    els.append(e_)
                    if n_db and e_[3] in db_faces:
                        t_f = sq["skin"]["t_out"] if "outer" in e_[3] else sq["skin"]["t_in"]
                        els.append((pw["E1"], n_db * pw["t"] * e_[1] / t_f, e_[2], "root_bay_doubler"))
            E_ = np.array([e_[0] for e_ in els])
            A_ = np.array([e_[1] for e_ in els])
            z_ = np.array([e_[2] for e_ in els])
            zna = float((E_ * A_ * z_).sum() / (E_ * A_).sum())
            EI_ = float((E_ * A_ * (z_ - zna) ** 2).sum())
            sq2 = dict(sq, EI=EI_, z_na=zna, z_cap_main=(sq["z_cap_main"][0] - dlt, sq["z_cap_main"][1] + dlt))
            for M_ in (at(L, "M", y_q), at(L, "M_neg", y_q)):
                st_ = section_strains(sq2, M_)
                e_cap = max(abs(st_["cap_main_up"]), abs(st_["cap_main_lo"]))
                e_sk = max(abs(st_["skin_top"]), abs(st_["skin_bot"]))
                if e_cap > worst_e[0]:
                    worst_e = (e_cap, worst_e[1], y_q)
                if e_sk > worst_e[1]:
                    worst_e = (worst_e[0], e_sk, worst_e[2])
        R.add("J-TRANS-CAP", "wing joint", "cap ramp: cap strain along the reduced lever arm (caps offset toward the tongue "
              "line, built-up area; skins on the loft in the section)", "başlık rampası: azalan moment kolu boyunca "
              "başlık birim şekil değiştirmesi (başlıklar dil hattına kaydırılmış, takviyeli alan; kaplamalar kesitte)",
              ctp, ctp_tr, worst_e[0] * 1e6, c.dt["cap_comp"] * 1e6, "µε", total_factor(c),
              "plane sections, ramp start / middle / end; damage-tolerance strain - fix round 2, VS2-04",
              part="YK250-WG-151")
        n_dbt = int(TRN.get("root_bay_skin_doubler_plies_per_face", 0))
        f_db = "outer face" if TRN.get("root_bay_doubler_faces", "both") == "outer" else "face"
        f_db_tr = "dış yüze" if f_db == "outer face" else "yüz başına"
        R.add("J-TRANS-SKIN", "wing joint", f"cap ramp: skin strain with the reduced cap lever arm (root-bay skin "
              f"doubler +{n_dbt} 0/90 ply per {f_db}, {float(TRN.get('root_bay_doubler_length_m', 0.0)) * 1000:.0f} mm)",
              f"başlık rampası: azalan başlık kolu ile kaplama birim şekil değiştirmesi (kök bölmesi kaplama takviyesi "
              f"{f_db_tr} +{n_dbt} kat 0/90, {float(TRN.get('root_bay_doubler_length_m', 0.0)) * 1000:.0f} mm)", ctp, ctp_tr,
              worst_e[1] * 1e6,
              c.dt["sand_comp"] * 1e6, "µε", total_factor(c), "plane sections; sandwich damage-tolerance strain - fix "
              "round 2, VS2-04", part="outer panel skins")
    # rear pin, lug, slot fitting (in-plane loads)
    rp = WJ["rear_spar"]
    lug, slot = rp["lug"], rp["slot_fitting"]
    Dr = float(rp["pin"]["diameter"])
    tl, wl, el = float(lug["thickness"]), float(lug["width"]), float(lug["e_m"])
    tpl, gs = float(slot["plate_t"]), 0.5 * (float(slot["slot"]) - float(lug["thickness"]))
    r = worst("P_rear")
    ct, ct_tr = case_txt(r)
    R.add("J-REAR-PIN-SHEAR", "wing joint", f"rear pin d {Dr * 1000:.0f} Ti-6Al-4V, double shear (C and F_y)",
          f"arka pim Ø{Dr * 1000:.0f} Ti-6Al-4V, çift kesme (C ve F_y)", ct, ct_tr, r["P_rear"],
          2 * math.pi * Dr ** 2 / 4 * float(ti["Fsu"]), "N", fa, "Ti Fsu; frequent assembly 1.5", part="P-REAR")
    Mr = ST.pin_bending_moment(r["P_rear"], tpl, tl, gs)
    R.add("J-REAR-PIN-BEND", "wing joint", "rear pin bending (Melcon-Hoblit)", "arka pim eğilmesi (Melcon-Hoblit)", ct,
          ct_tr, ST.pin_bending_stress(Mr, Dr) / 1e6, float(ti["Ftu"]) / 1e6, "MPa", fa, "Ftu", part="P-REAR")
    lg = ST.lug_axial(float(al["Ftu"]), float(al["Ftu"]), w=wl, D=Dr, t=tl, e=el)   # VS2-09: Ftu in shear-bearing
    k_tr = 0.6
    Fa_, Ft_ = abs(r["F_y"]) * fa["total"], abs(r["C"]) * fa["total"]
    k_mult = 1.0 / (((Fa_ / lg["P_allow"]) ** 1.6 + (Ft_ / (k_tr * lg["P_allow"])) ** 1.6) ** (1 / 1.6))
    R.add("J-REAR-LUG", "wing joint", f"rear-spar lug 7075 (t {tl * 1000:.0f}, w {wl * 1000:.0f}, e/D {el / Dr:.1f}): "
          "axial F_y + transverse C, oblique interaction", f"arka kiriş kulağı 7075 (t {tl * 1000:.0f}, w {wl * 1000:.0f}, "
          f"e/D {dec(el / Dr, 1)}): eksenel F_y + enine C, eğik etkileşim", ct, ct_tr, 1.0, k_mult, "load multiplier",
          dict(total_factor(c, ult_only=True), text="FoS 1.5 x frequent assembly 1.5 (in the multiplier)"),
          "structlib.lug_axial (Bruhn D1, net section / shear-bearing K_br F_tu D t - fix round 2, VS2-09: F_tu, not "
          "F_bru); transverse allowable 0.6 x axial (conservative Bruhn trend); (Ra^1.6 + Rtr^1.6) = 1",
          part="YK250-WG-152")
    R.add("J-REAR-BR", "wing joint", f"rear pin bearing: lug {tl * 1000:.0f} mm and slot plates 2 x {tpl * 1000:.0f} mm "
          "(7075, e/D 2)", f"arka pim ezilmesi: kulak {tl * 1000:.0f} mm ve yuva levhaları 2 x {tpl * 1000:.0f} mm (7075, "
          "e/D 2)", ct, ct_tr, r["P_rear"], Dr * min(tl, 2 * tpl) * float(al["Fbru"]), "N", fb,
          "MMPDS Fbru e/D 2; bearing factor 2.0", part="YK250-WG-152 / YK250-CH-054")
    r = worst("F_T")
    ct, ct_tr = case_txt(r)
    arm = float(rr["slot_plate_arm_m"])
    sig_pl = 6 * abs(r["F_T"]) * arm / (wl * tpl ** 2)
    R.add("J-REAR-PLATE", "wing joint", f"slot plate {tpl * 1000:.0f} mm bending under the vertical torsion couple (lug "
          f"face bearing, arm {arm * 1000:.0f} mm)", f"yuva levhası {tpl * 1000:.0f} mm, düşey burulma çifti altında "
          f"eğilme (kulak yüzü teması, kol {arm * 1000:.0f} mm)", ct, ct_tr, sig_pl / 1e6, float(al["Fty"]) / 1e6, "MPa",
          fa, "elastic plate strip, Fty", part="YK250-CH-054")
    worst_b = max(res, key=lambda q: math.hypot(q["C"], q["F_T"]) + math.hypot(q["F_y"], q["F_T"]))
    ct, ct_tr = case_txt(worst_b)
    nb_r, db_r = int(rr["rib_bolts"]), float(rr["rib_bolt_d_m"])
    nb_w, db_w = int(rr["web_bolts"]), float(rr["web_bolt_d_m"])
    t_pad = int(rr["web_pad_plies"]) * pw["t"]
    P_rb = math.hypot(worst_b["C"], worst_b["F_T"]) / nb_r
    P_wb = math.hypot(worst_b["F_y"], worst_b["F_T"]) / nb_w
    R.add("J-REAR-BOLTS-RIB", "wing joint", f"slot fitting to the joint rib: {nb_r} x M{db_r * 1000:.0f} Ti, single shear "
          "(C, F_T)", f"yuva bağlantısı - birleşim kaburgası: {nb_r} x M{db_r * 1000:.0f} Ti, tek kesme (C, F_T)", ct, ct_tr,
          P_rb, math.pi * db_r ** 2 / 4 * float(ti["Fsu"]), "N", total_factor(c, fit=True), "Ti Fsu on the shank",
          part="YK250-CH-054")
    R.add("J-REAR-BR-RIB", "wing joint", f"slot fitting bolts M{db_r * 1000:.0f}: bearing in the joint-rib land "
          f"({t_pad * 1000:.1f} mm)", f"yuva bağlantısı cıvataları M{db_r * 1000:.0f}: birleşim kaburgası bandında ezilme "
          f"({dec(t_pad * 1000, 1)} mm)", ct, ct_tr, P_rb, db_r * t_pad * qi["bearing_Pa"], "N", fit,
          "QI bearing ETW (pad >= 40 % +-45, e/D >= 3)", part="M-JOINTRIB")
    R.add("J-REAR-BOLTS-WEB", "wing joint", f"slot fitting to the rear-spar web: {nb_w} x M{db_w * 1000:.0f} Ti, single "
          "shear (F_y, F_T)", f"yuva bağlantısı - arka kiriş gövdesi: {nb_w} x M{db_w * 1000:.0f} Ti, tek kesme (F_y, "
          "F_T)", ct, ct_tr, P_wb, math.pi * db_w ** 2 / 4 * float(ti["Fsu"]), "N", total_factor(c, fit=True),
          "Ti Fsu on the shank", part="YK250-CH-054")
    R.add("J-REAR-BR-WEB", "wing joint", f"slot fitting bolts M{db_w * 1000:.0f}: bearing in the rear-spar web pad "
          f"({int(rr['web_pad_plies'])} plies, {t_pad * 1000:.1f} mm)", f"yuva bağlantısı cıvataları M{db_w * 1000:.0f}: "
          f"arka kiriş gövde takviyesinde ezilme ({int(rr['web_pad_plies'])} kat, {dec(t_pad * 1000, 1)} mm)", ct, ct_tr,
          P_wb, db_w * t_pad * qi["bearing_Pa"], "N", fit, "QI bearing ETW", part="YK250-CH-001")
    nl_, dl_ = int(rr["lug_fitting_bolts"]), float(rr["lug_fitting_bolt_d_m"])
    P_lf = max(math.hypot(math.hypot(q["C"], q["F_y"]), q["F_T"]) for q in res) / nl_
    R.add("J-REAR-LUGFIT", "wing joint", f"rear lug fitting to the outer-panel rear spar: {nl_} x M{dl_ * 1000:.0f} Ti, "
          f"bearing in the web pad ({t_pad * 1000:.1f} mm)", f"arka kulak bağlantısı - dış panel arka kirişi: {nl_} x "
          f"M{dl_ * 1000:.0f} Ti, gövde takviyesinde ezilme ({dec(t_pad * 1000, 1)} mm)", ct, ct_tr, P_lf,
          dl_ * t_pad * qi["bearing_Pa"], "N", fit, "QI bearing ETW; resultant of C, F_y, F_T shared equally",
          part="YK250-WG-152")
    w0 = res[0]
    return {"V": w0["V"], "M": w0["M"], "T": w0["T"], "R_in": w0["R_in"], "R_out": w0["R_out"], "F_T": w0["F_T"],
            "C": w0["C"], "pin_spacing": d, "dx": dx,
            "cases": {q["id"]: {k: float(q[k]) for k in ("V", "M", "T", "C", "Mz", "R_in", "R_out", "F_T", "F_y",
                                                          "P_out", "P_rear")} for q in res}}


def sob_kinks(c: Ctx, sized: dict) -> dict:
    """Cap ramp at the side-of-body rib (fix round 3, VS3-01): ramp angles of the four caps and the limit cap forces at
    both ramp ends (positive and negative design moments)."""
    y0, y1 = sob_ramp(c)
    L = wing_loads(c)
    caps = sized["main_cap_zones"]
    sw = joint_geometry(c)["sw"]
    Lr = (y1 - y0) / math.cos(sw)
    ct = ctbox_member(c)
    zt, zb = float(ct["z"][1]), float(ct["z"][0])
    s1 = wing_section(c, y1, cap_plies_at(caps, y1 - 1e-6))
    f = wing_contour(c, y1)
    zum1, zlm1 = (float(v) for v in f(s1["x_main"]))
    th_u, th_l = math.atan2(abs(zum1 - zt), Lr), math.atan2(abs(zlm1 - zb), Lr)
    out = {"y0": y0, "y1": y1, "L": Lr, "th_up": th_u, "th_lo": th_l, "dz_up": zum1 - zt, "dz_lo": zlm1 - zb}
    for tag, y in (("0", y0), ("1", y1 - 1e-6)):
        sec = wing_section(c, y, cap_plies_at(caps, y))
        A = sec["w_main"] * sec["t_main_cap"]
        F = 0.0
        for M_ in (at(L, "M", y), at(L, "M_neg", y)):
            st_ = section_strains(sec, M_)
            F = max(F, abs(st_["cap_main_up"]) * sec["E_ud"] * A * math.sin(th_u),
                    abs(st_["cap_main_lo"]) * sec["E_ud"] * A * math.sin(th_l))
        out["K" + tag] = F
        out["sec" + tag] = sec
    return out


def check_sob_transition(c: Ctx, R: Rows, sized: dict) -> dict:
    """Cap ramp and box-cover / glove-skin offset at the side-of-body rib (fix round 3, VS3-01): kink forces of the caps
    at both ramp ends (SOB rib land inboard, padded prongs at the inner pin outboard), cap-to-web interlaminar shear,
    and the plate bending of the SOB rib web under the offset couple of the glove box skins (on the loft) and the box
    covers (on the box level). The cap and skin strains along the ramp are the W-CAP rows of the ramp zone."""
    y0, y1 = sob_ramp(c)
    if y1 <= y0 + 1e-9:
        return {}
    K = sob_kinks(c, sized)
    TD = c.D["wing"]["sob_transition"]
    pw = ply_props(c, "cfrp_pw_mtm45_as4")
    qi = qi_design_values(c)
    fk = c.D["wing_joint"]["fork"]
    w_cap = float(c.D["wing"]["main_cap"]["width_body_glove_m"])
    L_i = float(TD["intro_length_m"])
    t_l = int(TD["rib_land_plies"]) * pw["t"]
    case = (f"wing design case (limit, positive and negative): cap ramp y {y0:.3f}-{y1:.3f} over {K['L'] * 1000:.0f} mm "
            f"along the spar, upper cap {K['dz_up'] * 1000:+.1f} mm ({math.degrees(K['th_up']):.1f} deg), lower cap "
            f"{K['dz_lo'] * 1000:+.1f} mm ({math.degrees(K['th_lo']):.1f} deg); kink force F_cap sin(theta)")
    case_tr = (f"kanat tasarım durumu (limit, pozitif ve negatif): başlık rampası y {dec(y0, 3)}-{dec(y1, 3)}, kiriş "
               f"boyunca {K['L'] * 1000:.0f} mm, üst başlık {dec(K['dz_up'] * 1000, 1)} mm ({dec(math.degrees(K['th_up']), 1)}°), "
               f"alt başlık {dec(K['dz_lo'] * 1000, 1)} mm ({dec(math.degrees(K['th_lo']), 1)}°); kırılma kuvveti "
               "F_başlık sin(θ)")
    R.add("W-SOB-KINK-RIB", "wing", f"SOB cap ramp: kink force at the side-of-body rib into its {int(TD['rib_land_plies'])}-"
          f"ply solid land ({t_l * 1000:.1f} x {w_cap * 1000:.0f} mm)", f"SOB başlık rampası: gövde yanı kaburgasında "
          f"kırılma kuvveti {int(TD['rib_land_plies'])} katlı dolu banda ({dec(t_l * 1000, 1)} x {w_cap * 1000:.0f} mm)",
          case, case_tr, K["K0"] / (t_l * w_cap) / 1e6, qi["OHC_Pa"] / 1e6, "MPa", total_factor(c, comp=True),
          "QI OHC ETW as the in-plane compression limit of the land (as J-TRANS-RIB) - fix round 3, VS3-01",
          part="YK250-CH-050")
    t_pd = float(fk["prong_pad_t_m"])
    R.add("W-SOB-KINK-PRONG", "wing", f"SOB cap ramp: kink force at the inner-pin station into the two prong bush pads "
          f"(2 x {t_pd * 1000:.0f} mm over {L_i * 1000:.0f} mm)", f"SOB başlık rampası: iç pim istasyonunda kırılma "
          f"kuvveti iki kulak burç takviyesine (2 x {t_pd * 1000:.0f} mm, {L_i * 1000:.0f} mm boyunca)", case, case_tr,
          K["K1"] / (2 * t_pd * L_i) / 1e6, qi["OHC_Pa"] / 1e6, "MPa", total_factor(c, comp=True),
          "QI OHC ETW - fix round 3, VS3-01", part="YK250-CH-053")
    ilss_ = float(research_value("materials", "composites.laminae.cfrp_pw_ooa_prepreg_as4_193.strength_Pa.ETW."
                                              "ILSS.b_basis"))
    R.add("W-SOB-KINK-ILSS", "wing", f"SOB cap ramp: cap-to-web / land interlaminar shear of the larger kink force (cap "
          f"width x {L_i * 1000:.0f} mm)", f"SOB başlık rampası: büyük kırılma kuvvetinin başlık-gövde / bant ara yüzey "
          f"kesmesi (başlık genişliği x {L_i * 1000:.0f} mm)", case, case_tr,
          max(K["K0"], K["K1"]) / (w_cap * L_i) / 1e6, ilss_ / 1e6, "MPa", total_factor(c, comp=True),
          "ILSS ETW B-basis (NCAMP PW) - fix round 3, VS3-01", part="YK250-CH-001")
    # offset of the glove box skins (on the loft) to the box covers (box level) at the SOB rib: the rib web carries the
    # couple N_skin x e per unit chord as plate bending (sandwich rib_panel, faces at the core distance)
    sec0 = K["sec0"]
    L = wing_loads(c)
    sk = sec0["skin"]
    N_ = 0.0
    for M_ in (at(L, "M", y0), at(L, "M_neg", y0)):
        st_ = section_strains(sec0, M_)
        N_ = max(N_, max(abs(st_["skin_top"]), abs(st_["skin_bot"])) * (sk["E_out"] * sk["t_out"] + sk["E_in"] * sk["t_in"]))
    ct = ctbox_member(c)
    zt, zb = float(ct["z"][1]), float(ct["z"][0])
    f0 = wing_contour(c, y0)
    xx = np.linspace(sec0["x_main"], sec0["x_rear"], 21)
    e_ = max(float(np.max(np.asarray(f0(xx)[0]) - zt)), float(np.max(zb - np.asarray(f0(xx)[1]))))
    rib = skin_faces(c, "rib_panel")
    n_db = int(TD.get("rib_face_doubler_plies", 0))
    t_f = min(rib["t_out"], rib["t_in"]) + n_db * pw["t"]
    d_r = rib["c"] + t_f
    m_allow = qi["OHC_Pa"] * t_f * d_r
    R.add("W-SOB-OFFSET", "wing", f"SOB rib web (rib_panel sandwich + {n_db} +-45 ply per face over the "
          f"{float(TD.get('rib_doubler_band_m', 0.0)) * 1000:.0f} mm edge bands): plate bending of the glove-skin / "
          f"box-cover offset couple (offset {e_ * 1000:.1f} mm, skin running load from the section strain at the rib)",
          f"SOB kaburgası gövdesi (rib_panel sandviç + {float(TD.get('rib_doubler_band_m', 0.0)) * 1000:.0f} mm kenar "
          f"bantlarında yüz başına {n_db} kat ±45): eldiven kaplaması / kutu kapağı kaçıklık çiftinin plaka eğilmesi "
          f"(kaçıklık {dec(e_ * 1000, 1)} mm, kaplama yayılı yükü kaburgadaki kesit birim şekil değiştirmesinden)",
          f"wing design case (limit) at y {y0:.2f}: N_skin {N_ / 1e3:.1f} N/mm", f"kanat tasarım durumu (limit), y "
          f"{dec(y0)}: N_kaplama {dec(N_ / 1e3, 1)} N/mm", N_ * e_, m_allow, "N m/m", total_factor(c, comp=True),
          "m = N e per unit chord; M_allow = OHC (QI ETW) x t_face x face distance - fix round 3, VS3-01",
          part="YK250-CH-050")
    return {"ramp": {k: (float(v) if not isinstance(v, dict) else None) for k, v in K.items() if not k.startswith("sec")},
            "N_skin_sob": N_, "offset_m": e_}


def section_le(c: Ctx, y: float) -> tuple:
    """(x_le, chord) of the wing loft at span station y (linear between the spec sections, as wing_contour)."""
    secs = c.S["wing"]["sections"]
    ys = np.array([s_["y"] for s_ in secs])
    y = float(np.clip(y, ys[0], ys[-1]))
    return (float(np.interp(y, ys, [s_["x_le"] for s_ in secs])), float(np.interp(y, ys, [s_["chord"] for s_ in secs])))


def check_lerx_skins(c: Ctx, R: Rows, sized: dict) -> dict:
    """Glove LERX (D-nose) skins ahead of the main spar between the side-of-body rib, the glove rib and the joint rib
    (no LERX nose ribs since VPK-02) - fix round 3, VS3-04: bonded primary skins (P-GLOVE-UP / -LO, wing_skin_primary)
    strained with the box: plane sections at the box curvature (no shear-lag relief from the SOB free edge: upper
    bound of the strain), the deepest point of the LERX surface ahead of the main-cap edge; panel a = rib bay (span),
    b = LERX chord from 20 mm aft of the leading edge (solid LE laminate) to the main-cap edge; torsion shear flow of
    the box added (conservative). Upper skin under the positive case, lower skin under the negative case."""
    L = wing_loads(c)
    caps = sized["main_cap_zones"]
    D = c.D["wing"]
    mem = {m_["id"]: m_ for m_ in c.S["layout"]["chassis"]["members"]}
    ribs = sorted(0.5 * (float(mem[k]["box"][0][1]) + float(mem[k]["box"][1][1]))
                  for k in ("M-SOB", "M-GLOVERIB", "M-JOINTRIB") if k in mem)
    out = {}
    case = (f"wing design case n = {c.n_wing:.2f} (upper skin) / {c.n_wing_neg:.2f} (lower skin), limit; plane sections at "
            "the box curvature over the LERX chord")
    case_tr = (f"kanat tasarım durumu n = {c.n_wing:.2f} (üst kaplama) / {dec(c.n_wing_neg)} (alt kaplama), limit; LERX "
               "veteri boyunca kutu eğriliğinde düzlem kesitler")
    lay_bay1 = D.get("lerx_upper_first_bay_layup", D["skin_layup"])
    for surf in ("UP", "LO"):
        best = None
        e_max = (0.0, 0.0)
        for y in np.linspace(Y_SOB + 0.005, float(c.P["y_junction"]) - 0.005, 28):
            lay = lay_bay1 if (surf == "UP" and len(ribs) > 1 and y < ribs[1]) else D["skin_layup"]
            sk = skin_faces(c, lay)
            EA = sk["E_out"] * sk["t_out"] + sk["E_in"] * sk["t_in"]
            sec = wing_section(c, y, cap_plies_at(caps, y))
            xle, ch = section_le(c, y)
            x0, x1 = xle + 0.020, sec["x_main"] - 0.5 * sec["w_main"]
            if x1 <= x0 + 0.02:
                continue
            f = wing_contour(c, y)
            xx = np.linspace(x0, x1, 25)
            zu_, zl_ = (np.asarray(v) for v in f(xx))
            Mp_, Mn_ = cap_moments(c, L, y)          # glove-box moments (total - tongue moment in the fork)
            if surf == "UP":
                kap = Mp_ / sec["EI"]
                e = float(np.max(kap * (zu_ - sec["z_na"])))
            else:
                kap = Mn_ / sec["EI"]
                e = float(np.max(kap * (zl_ - sec["z_na"])))
            e = max(e, 0.0)
            ia = int(np.searchsorted(ribs, y))
            a = (ribs[min(ia, len(ribs) - 1)] - ribs[max(ia - 1, 0)]) if 0 < ia < len(ribs) else 0.15
            b = x1 - x0
            Nx = e * EA
            Nxy = at(L, "T", y) / (2 * sec["A_box"])
            pb = panel_buckling_multiplier(c, sk, Nx, Nxy, a, b)
            pb.update(y=y, Nx=Nx, Nxy=Nxy, a=a, b=b, e=e, sk=sk)
            if best is None or pb["k"] < best["k"]:
                best = dict(pb, N_comb_max=best.get("N_comb_max", 0.0) if best else 0.0)
            if e > e_max[0]:
                e_max = (e, y)
            best["N_comb_max"] = max(best.get("N_comb_max", 0.0), math.hypot(Nx, Nxy))
        sk = best["sk"]
        lab = {"UP": ("glove LERX upper skin (positive case)", "eldiven LERX üst kaplaması (pozitif durum)"),
               "LO": ("glove LERX lower skin (negative case)", "eldiven LERX alt kaplaması (negatif durum)")}[surf]
        part = "YK250-SH-420" if surf == "UP" else "YK250-SH-421"
        R.add(f"W-SKINBUCK-LERX-{surf}", "wing", f"{lab[0]} panel {best['a']:.2f} x {best['b']:.2f} m ({sk['label']}) "
              f"buckling, compression + shear (governing y {best['y']:.3f})", f"{lab[1]} paneli {dec(best['a'])} x "
              f"{dec(best['b'])} m ({sk['label_tr']}) burkulma, bası + kesme (belirleyici y {dec(best['y'], 3)})", case,
              case_tr, 1.0, best["k"], "load multiplier", total_factor(c, comp=True),
              f"Rc + Rs^2 = 1; N_c {best['Nc'] / 1e3:.0f} N/mm, N_s {best['Ns'] / 1e3:.0f} N/mm (CLT D*, sandwich shear "
              f"correction); Nx {best['Nx'] / 1e3:.1f} N/mm from the plane-section strain - fix round 3, VS3-04", part=part)
        R.add(f"W-SKINCRIMP-LERX-{surf}", "wing", f"{lab[0]} ({sk['label']}): shear crimping", f"{lab[1]} "
              f"({sk['label_tr']}): kesme kıvrılması", case, case_tr, best["N_comb_max"] / 1e3, best["N_crimp"] / 1e3,
              "N/mm", total_factor(c, comp=True), "Zenkert 1995: N = G_c d^2 / c; N = hypot(Nx, Nxy) - fix round 3, "
              "VS3-04", part=part)
        R.add(f"W-SKINDT-LERX-{surf}", "wing", f"{lab[0]}: face compression strain (DT, sandwich), deepest point of the "
              f"LERX surface (governing y {e_max[1]:.3f})", f"{lab[1]}: yüz bası birim şekil değiştirmesi (hasar toleransı, "
              f"sandviç), LERX yüzeyinin en derin noktası (belirleyici y {dec(e_max[1], 3)})", case, case_tr,
              e_max[0] * 1e6, c.dt["sand_comp"] * 1e6, "µε", total_factor(c), "STANAG 4703 UL13.1.2 (2600 µε sandwich "
              "skins) - fix round 3, VS3-04", part=part)
        out[surf] = {"k": best["k"], "y": best["y"], "Nx": best["Nx"], "e_max": e_max[0], "y_e": e_max[1]}
    return out


def wing_set_mass(c: Ctx) -> float:
    names = ("wing_structure_pair", "control_surfaces_ailerons_pair", "control_surfaces_flaps_pair",
             "actuators_ailerons_2x_DA26", "actuators_flaps_2x_DA30")
    return sum(float(i["mass_kg"]) for i in c.S["mass"]["items"] if i["name"] in names)


def design_case_items(c: Ctx, case_name: str = "mtow_design_payload_turret_retracted") -> list:
    """(mass, x, y, z, name) of every mass item, payload item and the fuel of a loading case (sizing.mass_cases)."""
    Ms = c.S["mass"]
    pay = {p["name"]: p for p in Ms["payload_items"]}
    cs = [k for k in Ms["cases"] if k["name"] == case_name][0]
    its = [(float(i["mass_kg"]), float(i["x"]), float(i["y"]), float(i["z"]), i["name"]) for i in Ms["items"]]
    for nm in cs.get("payload", []):
        p = pay[nm]
        z = p["z_extended"] if (cs.get("turret") == "extended" and "z_extended" in p) else p["z"]
        its.append((float(p["mass_kg"]), float(p["x"]), float(p.get("y", 0.0)), float(z), nm))
    fx, fy, fz = Ms["fuel_cg"]
    its.append((float(Ms["fuel_kg"]) * float(cs["fuel_fraction"]), float(fx), float(fy), float(fz), "fuel"))
    return its


def station_x(c: Ctx, sid: str) -> float:
    return float([s_ for s_ in c.S["layout"]["stations"] if s_["id"] == sid][0]["x"])


def check_ct_box(c: Ctx, R: Rows, sized: dict) -> dict:
    """Carry-through box inside the body: webs = spar frames FS-MS / FS-RS (rib_panel sandwich with +-45 doublers),
    box-to-frame attachments (body inertia at the wing design case + wing torsion couple at the SOB rib), centre kink
    load of the chevron caps."""
    L = wing_loads(c)
    caps = sized["main_cap_zones"]
    BD = c.D["body"]
    WD = c.D["wing"]["ct_box"]
    x_ms, x_rs = station_x(c, "FS-MS"), station_x(c, "FS-RS")
    # webs: shear at y 0.2 (inside the body), main / rear split from the section
    y = 0.20
    n = cap_plies_at(caps, y)
    sec = wing_section(c, y, n)
    V = at(L, "V", y)
    q_T = at(L, "T", y) / (2 * sec["A_box"])
    rib = skin_faces(c, "rib_panel")
    pp = ply_props(c, "cfrp_pw_mtm45_as4")
    case = f"wing design case, CT-box shear V {V:.0f} N at y {y:.2f} (limit) + torsion shear flow {q_T:.0f} N/m"
    case_tr = f"kanat tasarım durumu, orta kutu kesmesi V {V:.0f} N, y {y:.2f} (limit) + burulma kesme akışı {q_T:.0f} N/m"
    for tag, f_, h, sid in (("MS", sec["f_main"], sec["h_main"], "FS-MS"), ("RS", 1 - sec["f_main"], sec["h_rear"],
                                                                              "FS-RS")):
        q = f_ * V / h + q_T
        nd = int(WD["web_doubler_plies_per_face"][sid])
        face = ST.laminate_abd([(pl[0], pl[1], pl[2]) for pl in rib["outer"]["plies"]] + [(pp, 45.0, pp["t"])] * nd)
        fpf = ST.first_ply_failure(face, (0.0, 0.0, q / 2), restrained=True)
        R.add(f"CT-WEB-{tag}", "CT box", f"CT-box web = frame {sid} (rib_panel + {nd} +-45 plies per face), face shear "
              "first-ply failure", f"orta kutu gövdesi = {sid} çerçevesi (rib_panel + yüz başına {nd} kat ±45), yüz "
              "kesmesi ilk katman hasarı", case, case_tr, q / 1e3, fpf["R"] * q / 1e3, "N/mm", total_factor(c, comp=True),
              "CLT, ply allowables B-basis ETW", part=f"{sid}")
        eng = ST.laminate_engineering(face)
        R.add(f"CT-WEBDT-{tag}", "CT box", f"CT-box web {sid}: face shear strain (DT)", f"orta kutu gövdesi {sid}: yüz "
              "kesme birim şekil değiştirmesi (hasar toleransı)", case, case_tr,
              (q / 2) / (eng["Gxy"] * face["h"]) * 1e6, c.dt["shear_thin"] * 1e6, "µε", total_factor(c),
              "STANAG UL13.1.2 5200 µε", part=sid)
        d_ = rib["c"] + face["h"]
        Gc = float(rib["core"]["G"])
        R.add(f"CT-WEBCRIMP-{tag}", "CT box", f"CT-box web {sid}: shear crimping", f"orta kutu gövdesi {sid}: kesme "
              "kıvrılması", case, case_tr, q / 1e3, Gc * d_ ** 2 / rib["c"] / 1e3, "N/mm", total_factor(c, comp=True),
              "N = G_c d^2 / c", part=sid)
    # body inertia to the box through the spar frames (+ wing torsion couple at the SOB rib)
    m_b = c.m0 - wing_set_mass(c)
    its = [i for i in design_case_items(c) if i[4] not in ("wing_structure_pair", "control_surfaces_ailerons_pair",
                                                            "control_surfaces_flaps_pair", "actuators_ailerons_2x_DA26",
                                                            "actuators_flaps_2x_DA30")]
    xb = sum(i[0] * i[1] for i in its) / sum(i[0] for i in its)
    W = m_b * c.n_wing * G
    R_ms = W * (x_rs - xb) / (x_rs - x_ms)
    R_rs = W - R_ms
    Tsob = at(L, "T", Y_SOB)
    F_T = Tsob / (x_rs - x_ms)
    nb = int(BD["box_attachment_bolts_per_side_per_frame"])
    Db = float(BD["box_attachment_bolt_d_m"])
    t_land = int(BD["frame_land_plies"]) * pp["t"]
    qi = qi_design_values(c)
    ti = mat(c, "ti_6al_4v_annealed_sheet")
    case2 = (f"body inertia m {m_b:.1f} kg x n {c.n_wing:.2f} (limit) at body CG x {xb:.3f}: FS-MS {R_ms:.0f} N, "
             f"FS-RS {R_rs:.0f} N; wing torsion at the SOB {Tsob:.0f} N m as a frame couple {F_T:.0f} N per side")
    case2_tr = (f"gövde ataleti m {m_b:.1f} kg x n {c.n_wing:.2f} (limit), gövde AM x {xb:.3f}: FS-MS {R_ms:.0f} N, "
                f"FS-RS {R_rs:.0f} N; gövde yanı kaburgasında kanat burulması {Tsob:.0f} N m, taraf başına çerçeve "
                f"çifti {F_T:.0f} N")
    for tag, Rf in (("MS", R_ms), ("RS", R_rs)):
        P = abs(Rf) / 2 / nb + F_T / nb
        R.add(f"CT-ATT-BR-{tag}", "CT box", f"box-to-frame FS-{tag} attachment {nb} x M6 Ti per side: bearing in the "
              f"frame land ({t_land * 1000:.1f} mm solid laminate)", f"kutu - FS-{tag} çerçevesi bağlantısı taraf başına "
              f"{nb} x M6 Ti: çerçeve kenar bandında ezilme ({t_land * 1000:.1f} mm dolu lamine)", case2, case2_tr, P,
              Db * t_land * qi["bearing_Pa"], "N", total_factor(c, fit=True, comp=True),
              "QI bearing 2 % offset ETW (materials.yaml design_values_for_code)", part=f"FS-{tag}")
        R.add(f"CT-ATT-SH-{tag}", "CT box", f"box-to-frame FS-{tag} bolts M6 Ti, single shear", f"kutu - FS-{tag} "
              "cıvataları M6 Ti, tek kesme", case2, case2_tr, P, float(ti["Fsu"]) * math.pi * Db ** 2 / 4, "N",
              total_factor(c, fit=True), "Ti-6Al-4V Fsu", part=f"FS-{tag}")
    # centre kink of the chevron caps (2 x 8 deg): chordwise kink force per cap at y = 0
    sec0 = wing_section(c, 0.0, cap_plies_at(caps, 0.0))
    e0 = -section_strains(sec0, at(L, "M", 0.0))["cap_main_up"]
    F_cap0 = e0 * sec0["E_ud"] * sec0["w_main"] * sec0["t_main_cap"]
    k_ang = math.radians(float([s_ for s_ in c.S["layout"]["stations"] if s_["id"] == "FS-MS"][0]["sweep_deg"]))
    F_k = 2 * F_cap0 * math.sin(k_ang)
    R.info("CT-KINK", "CT box", "centre kink fitting (7075) at y = 0: chordwise kink force per main cap",
           "y = 0 orta kırık bağlantısı (7075): ana başlık başına veter yönü kırılma kuvveti",
           "wing design case (limit)", "kanat tasarım durumu (limit)", F_k, "N",
           "F = 2 F_cap sin(kink); taken by the kink fitting into the sandwich covers and the FS-MS web (detail design)",
           part="YK250-CH-001")
    KF = c.D["wing"]["ct_box"].get("kink_fitting")
    if KF:
        kfm = mat(c, KF["material"])
        nkb, dkb = int(KF["bolts"]), float(KF["bolt_d_m"])
        t_rl = int(KF["rib_land_plies"]) * pp["t"]
        ckt = (f"wing design case (limit): chevron kink force per main cap {F_k:.0f} N (F = 2 F_cap sin(kink), "
               f"F_cap {F_cap0:.0f} N at y = 0)")
        ckt_tr = (f"kanat tasarım durumu (limit): ana başlık başına ok kırılma kuvveti {F_k:.0f} N (F = 2 F_başlık "
                  f"sin(kırık), y = 0'da F_başlık {F_cap0:.0f} N)")
        R.add("CT-KINK-BOLTS", "CT box", f"kink fitting tab to the centre-line rib: {nkb} x M{dkb * 1000:.0f} Ti, single "
              "shear", f"kırık bağlantısı kulağı - orta hat kaburgası: {nkb} x M{dkb * 1000:.0f} Ti, tek kesme", ckt,
              ckt_tr, F_k / nkb, float(ti["Fsu"]) * math.pi * dkb ** 2 / 4, "N", total_factor(c, fit=True),
              "Ti-6Al-4V Fsu - fix round 2, VS2-03", part="YK250-CH-001")
        R.add("CT-KINK-BR", "CT box", f"kink fitting bolts: bearing in the {int(KF['rib_land_plies'])}-ply solid land of "
              f"the centre-line rib ({t_rl * 1000:.1f} mm)", f"kırık bağlantısı cıvataları: orta hat kaburgasının "
              f"{int(KF['rib_land_plies'])} katlı dolu bandında ezilme ({dec(t_rl * 1000, 1)} mm)", ckt, ckt_tr,
              F_k / nkb, dkb * t_rl * qi["bearing_Pa"], "N", total_factor(c, fit=True, comp=True), "QI bearing ETW",
              part="YK250-CH-001")
        land = face_laminate(c, "+-45,0/90", int(KF["rib_land_plies"]) // 2)
        q_k = F_k / float(KF["length_m"])
        fpk = ST.first_ply_failure(land, (0.0, 0.0, q_k), restrained=True)
        R.add("CT-KINK-RIB", "CT box", f"centre-line rib solid land: in-plane shear of the kink force over the fitting "
              f"length ({float(KF['length_m']) * 1000:.0f} mm), first-ply failure", f"orta hat kaburgası dolu bandı: "
              f"kırılma kuvvetinin bağlantı boyunca ({float(KF['length_m']) * 1000:.0f} mm) düzlem içi kesmesi, ilk "
              "katman hasarı", ckt, ckt_tr, q_k / 1e3, fpk["R"] * q_k / 1e3, "N/mm", total_factor(c, fit=True, comp=True),
              "CLT (50 % +-45 land)", part="YK250-CH-001")
        A_kp = float(KF["w_m"]) * float(KF["plate_t_m"])
        R.add("CT-KINK-PLATE", "CT box", f"kink fitting plate {float(KF['w_m']) * 1000:.0f} x "
              f"{float(KF['plate_t_m']) * 1000:.0f} mm 7075: kink force in tension / compression", f"kırık bağlantısı "
              f"levhası {float(KF['w_m']) * 1000:.0f} x {float(KF['plate_t_m']) * 1000:.0f} mm 7075: kırılma kuvveti "
              "çekme / bası", ckt, ckt_tr, F_k / A_kp / 1e6, float(kfm["Fty"]) / 1e6, "MPa", total_factor(c, fit=True),
              "Fty", part="YK250-CH-001")
    return {"m_body": m_b, "x_body": xb, "R_FS_MS": R_ms, "R_FS_RS": R_rs, "F_T": F_T, "F_cap0": F_cap0, "F_kink": F_k}


# =====================================================================================================================
# 3. tail: stabilator, spindle, stub, fins, ventral fin / bumper
# =====================================================================================================================
def mass_item(c: Ctx, name: str) -> float:
    return float([i for i in c.S["mass"]["items"] if i["name"] == name][0]["mass_kg"])


def mass_item_base(c: Ctx, name: str) -> float:
    """Item mass without the growth allowance (mass_base_kg)."""
    return float([i for i in c.S["mass"]["items"] if i["name"] == name][0]["mass_base_kg"])


def tail_loads(c: Ctx) -> dict:
    """Stabilator panel normal force (limit, one panel): full deflection at VA with the section CN_max (spec
    stabilator checks, as sizing.stab_hinge_moments) and the CS-VLA 425(d) gust at VC (largest gust alleviation
    factor of the gust matrix, configuration tail lift slope, downwash); unsymmetric 100 % / 72 % (CS-LUAS.427(b)).
    Fin side force (limit, one fin): sideslip 15 deg x overswing 1.5 at VA (CS-LUAS.441(a)), full rudder at VA and the
    lateral gust at VC (CS-VLA 443(b), Kgt = 0.88 upper bound) - each bounded by the section CN_max."""
    if "tail_loads" in c.cache:
        return c.cache["tail_loads"]
    S = c.S
    tp = Z.tail_props(S, c.VA, 0.0, c.af)
    st = S["tail"]["surfaces"]["stabilator"]
    chk = st["controls"]["checks"]
    eta = float(S["aero"]["stability_rules"]["eta_tail"])
    Sp = float(tp["stabilator"]["area"])
    cn = float(chk["CN_max_panel"])
    N_VA = eta * c.q(c.VA) * Sp * cn
    deps = float(S["stability"]["computed"]["downwash_gradient"])
    kg = max(ST.gust_factor(float(r["mass_kg"]) * G / float(S["wing"]["area"]),
                            float(S["structures"]["derived"]["cl_alpha_configuration_per_rad"]), float(S["wing"]["mac"]),
                            Z.AL.isa(float(r["altitude_m"]))["rho"]) for r in S["structures"]["derived"]["gust_matrix"])
    Ude = float(S["structures"]["gust"]["Ude_VC"])
    dL_gust = 0.5 * RHO0 * kg * Ude * c.VC * float(tp["stabilator"]["a"]) * 2 * Sp * (1 - deps) / 2
    N_s = max(N_VA, dL_gust)
    fin = tp["fin"]
    Sf = float(fin["area"])
    ch_f = AE.characteristics(S["tail"]["surfaces"]["fin"]["sections"][0]["airfoil"],
                              Z.AL.reynolds(c.VA, float(fin["mac"]), 0.0))
    cn_f = float(ch_f["clmax"])
    a_f = float(fin["a"])
    L_beta = c.q(c.VA) * Sf * min(1.5 * a_f * math.radians(15.0), cn_f)
    rud = S["tail"]["surfaces"]["fin"]["controls"]["rudder"]
    E = float(rud["chord_fraction"])
    th = math.acos(1 - 2 * E)
    tau = 1 - (th - math.sin(th)) / math.pi
    L_rud = c.q(c.VA) * Sf * min(a_f * tau * math.radians(max(rud["range_deg"])), cn_f)
    L_gust = 0.5 * RHO0 * 0.88 * Ude * c.VC * a_f * Sf
    N_f = max(L_beta, L_rud, L_gust)
    out = {"N_stab": N_s, "N_stab_VA": N_VA, "N_stab_gust": dL_gust, "kg_max": kg, "eta": eta, "S_panel": Sp,
           "y_cp_stab": float(tp["stabilator"]["y_mac"]), "N_fin": N_f, "L_beta": L_beta, "L_rudder": L_rud,
           "L_gust": L_gust, "S_fin": Sf, "cn_fin": cn_f, "tp": tp, "unsym_fraction": 0.72}
    c.cache["tail_loads"] = out
    return out


def check_tail(c: Ctx, R: Rows) -> dict:
    """Stabilator spindle (bending + actuator stall torque), bearings (required ratings), stub cantilever and its root
    fitting on the ring frame FS3738, socket / spline / cross-bolt (frequently assembled), stabilator panel spar;
    fin spars and root fittings; ventral fin and bumper load path."""
    S = c.S
    TL = tail_loads(c)
    TD = c.D["tail"]
    st4130, al = mat(c, "steel_4130_n"), mat(c, "al_7075_t651_plate")
    sp_m = mat(c, TD["spindle"]["material"])
    fits = {f["id"]: f for f in S["layout"]["chassis"]["fittings"]}
    node = fits["F-SPINDLE-NODE"]
    ND = TD["node"]
    y_bi = float(node["cylinder"]["center"][1])
    y_bo = float(node["spindle"]["outboard_bearing_y"])
    y_root = float(S["tail"]["surfaces"]["stabilator"]["params"]["y_root"])
    N = TL["N_stab"]
    M_bo = N * (TL["y_cp_stab"] - y_bo)
    R_in = M_bo / (y_bo - y_bi)
    R_out = N + R_in
    sp = TD["spindle"]
    ro, ri = sp["od_m"] / 2, sp["id_m"] / 2
    I = math.pi / 4 * (ro ** 4 - ri ** 4)
    shm = Z.stab_hinge_moments(S)
    act = S["tail"]["surfaces"]["stabilator"]["controls"]["actuator"]
    k_max = float(shm["linkage"]["ratio_max"])
    T_st = float(act["torque_peak_Nm"]) * k_max
    sig = M_bo * ro / I
    tau = ST.tube_torsion_stress(T_st, ro, ri)
    vm = math.sqrt(sig ** 2 + 3 * tau ** 2)
    case = (f"stabilator full deflection at VA, CN_max {S['tail']['surfaces']['stabilator']['controls']['checks']['CN_max_panel']:.2f}: "
            f"N {N:.0f} N per panel (limit; gust at VC {TL['N_stab_gust']:.0f} N) at y {TL['y_cp_stab']:.3f}; actuator "
            f"stall torque {T_st:.1f} N m (DA 30 peak x linkage ratio max {k_max:.2f}, CS-LUAS.395)")
    case_tr = (f"VA'da tam sapmış stabilatör, CN_maks: panel başına N {N:.0f} N (limit; VC hamlesi {TL['N_stab_gust']:.0f} N), "
               f"y {TL['y_cp_stab']:.3f}; eyleyici durma torku {T_st:.1f} N m (DA 30 tepe x en büyük bağlantı oranı "
               f"{k_max:.2f}, CS-LUAS.395)")
    wall = 0.5 * (sp["od_m"] - sp["id_m"])
    R.add("T-SPINDLE", "tail", f"stabilator spindle {sp['od_m'] * 1000:.0f} x {wall * 1000:.1f} mm "
          f"({sp['material']}) at the outboard bearing, bending + torsion (von Mises)", f"stabilatör mili "
          f"{sp['od_m'] * 1000:.0f} x {dec(wall * 1000, 1)} mm ({sp['material']}), dış yatakta eğilme + "
          "burulma (von Mises)", case, case_tr, vm / 1e6, float(sp_m["Ftu"]) / 1e6, "MPa", total_factor(c, fit=True),
          f"M {M_bo:.0f} N m, T {T_st:.1f} N m; MMPDS Ftu", part="YK250-TL-301")
    R.add("T-SPINDLE-SPL", "tail", "spindle spline section (tooth depth 0.5 mm off the wall), torsion alone (bending at "
          "the spline: T-SPINDLE-SPL-MT)", "mil kama kesiti (duvardan 0,5 mm diş derinliği), yalnız burulma (kamada eğilme: "
          "T-SPINDLE-SPL-MT)", case, case_tr,
          ST.tube_torsion_stress(T_st, ro - 0.0005, ri) / 1e6, float(sp_m["Fsu"]) / 1e6, "MPa",
          total_factor(c, fa=True), "Fsu; frequent assembly 1.5 (stabilator removed for transport)",
          part="YK250-TL-301")
    f_ult = c.f["fos"]
    R.info("T-BRG-IN", "tail", "spindle inboard bearing 61805-ZZ (node boss): required static rating C0 (ultimate "
           "radial load)", "mil iç yatağı 61805-ZZ (düğüm göbeği): gerekli statik yük sayısı C0 (nihai radyal yük)", case,
           case_tr, f_ult * R_in, "N", "procurement requirement: C0 >= this value (catalogue rating not in the research "
           "data)", part="F-SPINDLE-NODE")
    R.info("T-BRG-OUT", "tail", "spindle outboard bearing 61805-ZZ: required static rating C0 (ultimate radial load)",
           "mil dış yatağı 61805-ZZ: gerekli statik yük sayısı C0 (nihai radyal yük)", case, case_tr, f_ult * R_out, "N",
           "procurement requirement: C0 >= this value", part="stub tip rib")
    # node F-SPINDLE-NODE (fix round 1, VPK-01 / S1-06): stub root on the outboard cheek, inboard bearing boss, base
    # flange through-bolted to the firewall
    fit_n = total_factor(c, fit=True)
    Rm = float(research_value("materials", "fasteners.property_classes.steel_12_9.tensile_Rm_min_Pa"))
    As = research_value("materials", "fasteners.property_classes.stress_area.A_s_m2")
    bl = node["bolts"]
    sb = [b_ for b_ in bl if b_["group"] == "stub root"]
    fb_ = [b_ for b_ in bl if b_["group"] == "firewall"]
    ch_box = max(node["boxes"], key=lambda bx: bx[1][0] - bx[0][0])          # outboard cheek (longest in x)
    y_face = float(ch_box[1][1])
    x_base = float(min(bx[1][0] for bx in node["boxes"] if abs(bx[0][0] - station_x(c, "FS3670")) < 1e-6))
    M_sr = R_out * (y_bo - y_face)
    P_sb = np.array([[b_["point"][0], b_["point"][2]] for b_ in sb])
    T_sb = ST.bolt_group_tension(P_sb, 0.0, Mx=M_sr)
    S_sb = R_out / len(sb)
    d_sb = float(sb[0]["d"])
    As_sb = float(As[f"M{d_sb * 1000:.0f}"])
    r_int = min(1.0 / math.sqrt((S_sb * fit_n["total"] / (0.6 * Rm * As_sb)) ** 2 +
                                (max(t_, 0.0) * fit_n["total"] / (Rm * As_sb)) ** 2) for t_ in T_sb)
    cs_ = (f"; node: inboard bearing R_in {R_in:.0f} N at y {y_bi:.3f}, stub tip bearing R_out {R_out:.0f} N at y "
           f"{y_bo:.3f}, stub root moment at the cheek face (y {y_face:.4f}) {M_sr:.0f} N m")
    cs_tr = (f"; düğüm: iç yatak R_iç {R_in:.0f} N (y {y_bi:.3f}), kök parçası uç yatağı R_dış {R_out:.0f} N (y "
             f"{y_bo:.3f}), yanak yüzünde (y {y_face:.4f}) kök parçası kök momenti {M_sr:.0f} N m")
    R.add("T-NODE-STUB-BOLTS", "tail", f"stub rear-spar root fitting to the outboard cheek: {len(sb)} x "
          f"M{d_sb * 1000:.0f} 12.9 (axis y), tension from the root moment + shear R_out, interaction (most loaded bolt)",
          f"kök parçası arka kiriş kök bağlantısı - dış yanak: {len(sb)} x M{d_sb * 1000:.0f} 12.9 (y ekseni), kök momentinden "
          "çekme + R_dış kesmesi, etkileşim (en yüklü cıvata)", case + cs_, case_tr + cs_tr, 1.0, r_int, "load multiplier",
          dict(total_factor(c, ult_only=True), text="FoS 1.5 x fitting 1.15 (in the multiplier)"),
          "elastic bolt group (tension about the pattern centroid, contact not credited); ISO 898-1 12.9: tension Rm "
          "A_s, shear 0.6 Rm A_s; R_s^2 + R_t^2 = 1", part="YK250-CH-095")
    t_ch = float(ND["cheek_t_m"])
    R.add("T-NODE-STUB-BR", "tail", f"stub root bolts M{d_sb * 1000:.0f}: bearing in the {t_ch * 1000:.0f} mm 7075 cheek",
          f"kök parçası cıvataları M{d_sb * 1000:.0f}: {t_ch * 1000:.0f} mm 7075 yanakta ezilme", case + cs_,
          case_tr + cs_tr, S_sb, d_sb * t_ch * float(al["Fbru"]), "N", fit_n, "MMPDS Fbru (e/D >= 2)",
          part="YK250-CH-095")
    b_ch = float(ch_box[1][2] - ch_box[0][2])
    x_sb = float(P_sb[:, 0].mean())
    tau_ch = M_sr * t_ch / (b_ch * t_ch ** 3 / 3)
    sig_ch = 6 * R_out * (x_sb - x_base) / (t_ch * b_ch ** 2)
    R.add("T-NODE-CHEEK", "tail", f"node outboard cheek {t_ch * 1000:.0f} x {b_ch * 1000:.0f} mm: torsion from the stub root "
          "moment (open section b t^3/3) + in-plane bending from R_out, von Mises", f"düğüm dış yanağı "
          f"{t_ch * 1000:.0f} x {b_ch * 1000:.0f} mm: kök parçası kök momentinden burulma (açık kesit b t^3/3) + R_dış "
          "düzlem içi eğilmesi, von Mises", case + cs_, case_tr + cs_tr, math.sqrt(sig_ch ** 2 + 3 * tau_ch ** 2) / 1e6,
          float(al["Ftu"]) / 1e6, "MPa", fit_n, f"arm of R_out to the base {(x_sb - x_base) * 1000:.0f} mm; MMPDS Ftu",
          part="YK250-CH-095")
    cyl = node["cylinder"]
    x_brg = float(cyl["center"][0])
    in_box = [bx for bx in node["boxes"] if bx[0][1] <= y_bi <= bx[1][1] and bx[0][0] > x_base - 1e-6][0]
    h_in = float(in_box[1][2] - in_box[0][2])
    t_in = float(ND["inboard_web_t_m"])
    sig_in = 6 * R_in * (x_brg - x_base) / (t_in * h_in ** 2)
    R.add("T-NODE-INBOARD", "tail", f"node inboard arm (web {t_in * 1000:.0f} x {h_in * 1000:.0f} mm, cantilever "
          f"{(x_brg - x_base) * 1000:.0f} mm from the base) under the inboard bearing load, bending", f"düğüm iç kolu (gövde "
          f"{t_in * 1000:.0f} x {h_in * 1000:.0f} mm, tabandan {(x_brg - x_base) * 1000:.0f} mm konsol) iç yatak yükü altında, "
          "eğilme", case + cs_, case_tr + cs_tr, sig_in / 1e6, float(al["Ftu"]) / 1e6, "MPa", fit_n, "MMPDS Ftu",
          part="YK250-CH-095")
    r_o, r_b = float(cyl["radius"]), 0.5 * float(ND["bearing_od_m"])
    t_bw, L_b = r_o - r_b, 2 * float(cyl["half_length"])
    M_ring = 0.3183 * R_in * 0.5 * (r_o + r_b)
    R.add("T-NODE-BOSS", "tail", f"inboard bearing boss ring (OD {2 * r_o * 1000:.0f} / bore {2 * r_b * 1000:.0f} mm, wall "
          f"{t_bw * 1000:.1f} x {L_b * 1000:.0f} mm), ring bending under the bearing load", f"iç yatak göbeği halkası (dış "
          f"Ø{2 * r_o * 1000:.0f} / delik Ø{2 * r_b * 1000:.0f} mm, duvar {dec(t_bw * 1000, 1)} x {L_b * 1000:.0f} mm), yatak "
          "yükü altında halka eğilmesi", case + cs_, case_tr + cs_tr, 6 * M_ring / (L_b * t_bw ** 2) / 1e6,
          float(al["Ftu"]) / 1e6, "MPa", fit_n, "thin ring under diametral load M = P R / pi (Roark table 9.2 case 1, "
          "conservative: the arm web support not credited); MMPDS Ftu", part="YK250-CH-095")
    P_fb = np.array([[b_["point"][1], b_["point"][2]] for b_ in fb_])
    c_fb = P_fb.mean(axis=0)
    z_s = float(cyl["center"][2])
    y_r = 0.5 * (ch_box[0][1] + ch_box[1][1])
    Mx_n = M_sr + R_out * (y_r - c_fb[0]) - R_in * (y_bi - c_fb[0])
    shear_fb = ST.bolt_group_inplane(P_fb, (0.0, R_out - R_in), Mx_n)
    My_n = (R_out - R_in) * (x_brg - station_x(c, "FS3670"))
    ten_fb = ST.bolt_group_tension(P_fb, 0.0, Mx=My_n)
    d_fb = float(fb_[0]["d"])
    As_fb = float(As[f"M{d_fb * 1000:.0f}"])
    r_fb = min(1.0 / math.sqrt((s_ * fit_n["total"] / (0.6 * Rm * As_fb)) ** 2 +
                               (max(t_, 0.0) * fit_n["total"] / (Rm * As_fb)) ** 2) for s_, t_ in zip(shear_fb, ten_fb))
    cs2 = (f"; node base: in-plane moment about x {Mx_n:.0f} N m, net vertical {R_out - R_in:.0f} N, moment about y "
           f"{My_n:.0f} N m (limit)")
    cs2_tr = (f"; düğüm tabanı: x ekseni etrafında düzlem içi moment {Mx_n:.0f} N m, net düşey {R_out - R_in:.0f} N, y "
              f"ekseni etrafında moment {My_n:.0f} N m (limit)")
    R.add("T-NODE-BASE-BOLTS", "tail", f"node base flange to the firewall stack: {len(fb_)} x M{d_fb * 1000:.0f} 12.9 "
          "(axis x), shear + tension interaction (most loaded bolt)", f"düğüm taban flanşı - yangın perdesi katmanı: "
          f"{len(fb_)} x M{d_fb * 1000:.0f} 12.9 (x ekseni), kesme + çekme etkileşimi (en yüklü cıvata)", case + cs2,
          case_tr + cs2_tr, 1.0, r_fb, "load multiplier",
          dict(total_factor(c, ult_only=True), text="FoS 1.5 x fitting 1.15 (in the multiplier)"),
          "elastic bolt group (structlib.bolt_group_inplane / bolt_group_tension); ISO 898-1 12.9", part="YK250-CH-095")
    pp_ = ply_props(c, "cfrp_pw_mtm45_as4")
    t_lf = int(c.D["body"]["frame_land_plies"]) * pp_["t"]
    R.add("T-NODE-FW-BR", "tail", f"node base bolts M{d_fb * 1000:.0f}: bearing in the firewall land "
          f"({t_lf * 1000:.1f} mm solid laminate)", f"düğüm taban cıvataları M{d_fb * 1000:.0f}: yangın perdesi bandında "
          f"ezilme ({dec(t_lf * 1000, 1)} mm dolu lamine)", case + cs2, case_tr + cs2_tr, float(shear_fb.max()),
          d_fb * t_lf * qi_design_values(c)["bearing_Pa"], "N", total_factor(c, fit=True, comp=True),
          "QI bearing ETW; stainless spacer tubes not credited", part="FS3670")
    rib_ = skin_faces(c, "rib_panel")
    core_i = mat(c, c.D["firewall"]["core_insert"]["core"])
    d_fw = rib_["c"] + 0.5 * (rib_["t_out"] + rib_["t_in"])
    bx_b = [bx for bx in node["boxes"] if abs(bx[0][0] - station_x(c, "FS3670")) < 1e-6]
    perim_half = (max(b_[1][1] for b_ in bx_b) - min(b_[0][1] for b_ in bx_b)) + \
        (max(b_[1][2] for b_ in bx_b) - min(b_[0][2] for b_ in bx_b))
    P_t = float(np.clip(ten_fb, 0.0, None).sum())
    R.add("T-NODE-FW-CORE", "tail", "firewall sandwich under the node base: out-of-plane bolt tension of the base "
          "moment, core shear around the tension half of the backing-plate perimeter", "düğüm tabanı altında yangın "
          "perdesi sandviçi: taban momentinin düzlem dışı cıvata çekmesi, destek levhası çevresinin çekme yarısında "
          "çekirdek kesmesi", case + cs2, case_tr + cs2_tr, P_t / (perim_half * d_fw) / 1e6, float(core_i["Fsu"]) / 1e6,
          "MPa", total_factor(c, comp=True), f"tension half-perimeter {perim_half * 1000:.0f} mm, d {d_fw * 1000:.1f} mm; "
          f"{core_i['name']} core insert (structures.sizing.firewall.core_insert)", part="FS3670")
    # socket / spline / cross-bolt: removable stabilator (frequent assembly)
    m_panel = mass_item(c, "stabilators_pair") / 2
    F_ax = 12.0 * m_panel * G
    R.add("T-CROSSBOLT", "tail", "stabilator retention cross-bolt M6 12.9 (double shear), hinge-line inertia 12 g",
          "stabilatör tespit çapraz cıvatası M6 12.9 (çift kesme), menteşe ekseni ataleti 12 g",
          "inertia parallel to the hinge line 12 x panel weight (CS-LUAS.393(b))", "menteşe eksenine paralel atalet "
          "12 x panel ağırlığı (CS-LUAS.393(b))", F_ax, 2 * 0.6 * 1220e6 * 20.1e-6, "N", total_factor(c, fa=True),
          "ISO 898-1 12.9", part="YK250-TL-301")
    SK = TD["socket"]
    L_e = float(SK["length_m"])
    M_s = N * (TL["y_cp_stab"] - y_root)
    p_br = 6 * M_s / (sp["od_m"] * L_e ** 2) + N / (sp["od_m"] * L_e)
    R.add("T-SOCKET-BR", "tail", f"stabilator root socket (7075 sleeve) bearing on the spindle, {L_e:.2f} m engagement",
          f"stabilatör kök yuvası (7075 kovan) mil üzerinde ezilme, {dec(L_e)} m geçme", case, case_tr, p_br / 1e6,
          float(al["Fbru"]) / 1e6, "MPa", total_factor(c, bear=True, fa=True),
          "linear bearing pressure p = 6 M/(d L^2) + V/(d L); bearing factor 2.0", part="YK250-TL-256")
    msk = mat(c, SK["material"])
    ro_s, ri_s = 0.5 * float(SK["od_m"]), ro
    I_s = math.pi / 4 * (ro_s ** 4 - ri_s ** 4)
    R.add("T-SOCKET-TUBE", "tail", f"stabilator root socket sleeve {float(SK['od_m']) * 1000:.0f} x "
          f"{(ro_s - ri_s) * 1000:.1f} mm 7075, bending at the root", f"stabilatör kök yuvası kovanı "
          f"{float(SK['od_m']) * 1000:.0f} x {dec((ro_s - ri_s) * 1000, 1)} mm 7075, kökte eğilme", case, case_tr,
          M_s * ro_s / I_s / 1e6, float(msk["Ftu"]) / 1e6, "MPa", total_factor(c, fa=True),
          f"M {M_s:.0f} N m; MMPDS Ftu; frequent assembly 1.5", part="YK250-TL-256")
    D_o = float(SK["od_m"])
    p_o = 6 * M_s / (D_o * L_e ** 2) + N / (D_o * L_e)
    R.add("T-SOCKET-OD", "tail", "socket sleeve bearing on the stabilator root rib / spar laminate (ends of the "
          "engagement)", "kovanın stabilatör kök kaburgası / kiriş laminesi üzerinde ezilmesi (geçme uçları)", case, case_tr,
          p_o / 1e6, qi_design_values(c)["OHC_Pa"] / 1e6, "MPa", total_factor(c, bear=True, fa=True),
          "p = 6 M/(D L^2) + V/(D L); QI OHC ETW as the bearing limit; bearing factor 2.0", part="YK250-TL-256")
    tau_sp = 2 * T_st / (math.pi * (0.9 * sp["od_m"]) ** 2 * L_e / 2)
    spl = sp.get("spline") or {"length_m": 0.020, "tooth_depth_m": 0.0005}
    l_s, h_t = float(spl["length_m"]), float(spl["tooth_depth_m"])
    M_spl = 6 * M_s / L_e ** 2 * (l_s ** 2 / 2 - l_s ** 3 / (3 * L_e))      # socket moment at the spline root
    ro_r = sp["od_m"] / 2 - h_t
    I_r = math.pi / 4 * (ro_r ** 4 - ri ** 4)
    sig_r = M_spl * ro_r / I_r
    tau_r = ST.tube_torsion_stress(T_st, ro_r, ri)
    R.add("T-SPINDLE-SPL-MT", "tail", f"spindle at the spline root ({l_s * 1000:.0f} mm from the spindle tip, wall reduced "
          f"by the {h_t * 1000:.1f} mm teeth): bending (socket moment distribution) + torsion, von Mises",
          f"kama kökünde mil (mil ucundan {l_s * 1000:.0f} mm, {dec(h_t * 1000, 1)} mm dişlerle incelmiş duvar): eğilme "
          "(yuva moment dağılımı) + burulma, von Mises", case, case_tr, math.sqrt(sig_r ** 2 + 3 * tau_r ** 2) / 1e6,
          float(sp_m["Ftu"]) / 1e6, "MPa", total_factor(c, fa=True),
          f"M_spline = 6 M_s / L^2 (l^2/2 - l^3/(3L)) = {M_spl:.1f} N m (linear socket pressure, M_s {M_s:.0f} N m at the "
          "mouth); fix round 2, VS2-08", part="YK250-TL-301")
    R.add("T-SPLINE", "tail", "25 mm involute spline teeth (shear at 0.9 d, half the teeth effective)",
          "25 mm evolvent kama dişleri (0,9 d'de kesme, dişlerin yarısı etkin)", case, case_tr, tau_sp / 1e6,
          float(sp_m["Fsu"]) / 1e6, "MPa", total_factor(c, fa=True), "simplified spline shear; Fsu",
          part="YK250-TL-301")
    # stabilator panel spar at the end of the socket
    stp = S["tail"]["surfaces"]["stabilator"]["params"]
    y_e = y_root + L_e
    ch_e = float(np.interp(y_e, [stp["y_root"], stp["y_root"] + stp["span"]], [stp["root_chord"], stp["tip_chord"]]))
    xs_c = float(S["tail"]["surfaces"]["stabilator"]["controls"]["checks"]["spindle_x"])
    x_le_e = float(np.interp(y_e, [s_["y"] for s_ in S["tail"]["surfaces"]["stabilator"]["sections"]],
                             [s_["x_le"] for s_ in S["tail"]["surfaces"]["stabilator"]["sections"]]))
    xc = min(max((xs_c - x_le_e) / ch_e, 0.15), 0.6)
    depth = Z.section_depth_at("NACA-0014", 1.0, ch_e, xc)
    ud = ply_props(c, "cfrp_ud_mtm45_as4")
    tcap = int(TD["stab_spar_cap_plies"]) * ud["t"]
    h = depth - 2 * 0.001 - tcap
    Fc = N * (TL["y_cp_stab"] - y_e) / h
    eps = Fc / (ud["E1"] * TD["stab_spar_cap_width_m"] * tcap)
    R.add("T-STABSPAR", "tail", f"stabilator panel spar cap ({TD['stab_spar_cap_plies']} plies x "
          f"{TD['stab_spar_cap_width_m'] * 1000:.0f} mm UD) at the socket end, compression strain",
          f"stabilatör panel kirişi başlığı ({TD['stab_spar_cap_plies']} kat x {TD['stab_spar_cap_width_m'] * 1000:.0f} mm "
          "UD) yuva ucunda, bası birim şekil değiştirmesi", case, case_tr, eps * 1e6, c.dt["cap_comp"] * 1e6, "µε",
          total_factor(c), f"depth {depth * 1000:.0f} mm at x/c {xc:.2f}; STANAG UL13.1.2", part="YK250-TL-256")
    tsk = skin_faces(c, "tail_skin")
    lam_t = sandwich_lam(c, tsk)
    N_crimp_t = ST.shear_crimping_stress(lam_t["Gc"], lam_t["d"], tsk["c"], tsk["t_out"], tsk["t_in"]) * \
        (tsk["t_out"] + tsk["t_in"])
    EA_t = tsk["E_out"] * tsk["t_out"] + tsk["E_in"] * tsk["t_in"]
    e_sk = eps * depth / h
    R.add("T-STABSKIN", "tail", f"stabilator skin ({tsk['label']}) at the socket end: shear crimping under the spanwise "
          "compression (skin strain = cap strain x depth / cap lever arm)", f"stabilatör kaplaması ({tsk['label_tr']}) "
          "yuva ucunda: açıklık yönü basısında kesme kıvrılması", case, case_tr, e_sk * EA_t / 1e3, N_crimp_t / 1e3,
          "N/mm", total_factor(c, comp=True), "Zenkert 1995: N = G_c d^2 / c", part="YK250-TL-256")
    # fins
    fin_s = S["tail"]["surfaces"]["fin"]["sections"]
    p0 = np.array([fin_s[0]["y"], fin_s[0]["z_le"]])
    p1 = np.array([fin_s[-1]["y"], fin_s[-1]["z_le"]])
    u = (p1 - p0) / np.linalg.norm(p1 - p0)
    fr = [f for f in S["layout"]["chassis"]["fittings"] if f["id"] == "F-FIN-FRONT"][0]
    s_fit = float((np.array([fr["point"][1], fr["point"][2]]) - p0) @ u)
    tp = TL["tp"]["fin"]
    s_cp = float((np.array([tp["y_mac"], tp["z_mac"]]) - p0) @ u)
    Nf = TL["N_fin"]
    M_f = Nf * (s_cp - s_fit)
    fpar = S["tail"]["surfaces"]["fin"]["params"]
    c_root = fpar["root_chord"]
    h_fr = Z.section_depth_at("n0012", 1.0, c_root, float(fr["fin_chord_fraction"])) - 0.002
    rr = [f for f in S["layout"]["chassis"]["fittings"] if f["id"] == "F-FW-CORNER"][0]
    h_rr = Z.section_depth_at("n0012", 1.0, c_root, float(rr["fin_chord_fraction"])) - 0.002
    share_f = h_fr ** 2 / (h_fr ** 2 + h_rr ** 2)
    casef = (f"fin side force {Nf:.0f} N (limit, one fin; sideslip 15 deg x 1.5 {TL['L_beta']:.0f} N, full rudder "
             f"{TL['L_rudder']:.0f} N, lateral gust {TL['L_gust']:.0f} N, bounded by the section CN_max "
             f"{TL['cn_fin']:.2f}) at the exposed MAC, root moment {M_f:.0f} N m")
    casef_tr = (f"dikey yan kuvveti {Nf:.0f} N (limit, tek dikey; 15° x 1,5 yanal kayma {TL['L_beta']:.0f} N, tam dümen "
                f"{TL['L_rudder']:.0f} N, yanal hamle {TL['L_gust']:.0f} N, kesit CN_maks {TL['cn_fin']:.2f} ile "
                f"sınırlı), açıkta kalan OAV'de, kök momenti {M_f:.0f} N m")
    tcf = int(TD["fin_spar_cap_plies"]) * ud["t"]
    eps_f = share_f * M_f / (h_fr - tcf) / (ud["E1"] * TD["fin_spar_cap_width_m"] * tcf)
    R.add("T-FINSPAR", "tail", f"fin front spar cap ({TD['fin_spar_cap_plies']} plies x {TD['fin_spar_cap_width_m'] * 1000:.0f} "
          "mm UD) at the root fitting, compression strain", f"dikey ön kiriş başlığı ({TD['fin_spar_cap_plies']} kat x "
          f"{TD['fin_spar_cap_width_m'] * 1000:.0f} mm UD) kök bağlantısında, bası birim şekil değiştirmesi", casef,
          casef_tr, eps_f * 1e6, c.dt["cap_comp"] * 1e6, "µε", total_factor(c), "STANAG UL13.1.2",
          part="tail module (fin)")
    e_fsk = eps_f * h_fr / (h_fr - tcf)
    R.add("T-FINSKIN", "tail", f"fin skin ({tsk['label']}) at the root: shear crimping under the bending compression",
          f"dikey kaplaması ({tsk['label_tr']}) kökte: eğilme basısında kesme kıvrılması", casef, casef_tr,
          e_fsk * EA_t / 1e3, N_crimp_t / 1e3, "N/mm", total_factor(c, comp=True), "Zenkert 1995: N = G_c d^2 / c",
          part="tail module (fin)")
    sbf = float(TD["fin_root_bolt_spacing_m"])
    for tag, sh, fit in (("FRONT", share_f, fr), ("REAR", 1 - share_f, rr)):
        P = sh * M_f / sbf + 0.5 * sh * Nf
        R.add(f"T-FINROOT-{tag}", "tail", f"fin {tag.lower()}-spar root fitting {fit['part']}: 2 x M6 12.9 double shear "
              f"(cap bolts {sbf * 1000:.0f} mm apart)", f"dikey {('ön' if tag == 'FRONT' else 'arka')} kiriş kök "
              f"bağlantısı {fit['part']}: 2 x M6 12.9 çift kesme (başlık cıvataları {sbf * 1000:.0f} mm aralık)", casef,
              casef_tr, P, 2 * 0.6 * 1220e6 * 20.1e-6, "N", total_factor(c, fit=True), "ISO 898-1 12.9",
              part=fit["part"])
        R.add(f"T-FINROOT-BR-{tag}", "tail", f"fin {tag.lower()}-spar root fitting: 7075 angle lug bearing (t 5 mm)",
              f"dikey {('ön' if tag == 'FRONT' else 'arka')} kiriş kök bağlantısı: 7075 köşebent kulak ezilmesi (t 5 mm)",
              casef, casef_tr, P, 0.006 * 0.005 * float(al["Fbru"]), "N", total_factor(c, fit=True), "MMPDS Fbru",
              part=fit["part"])
        fb_s = [b_ for b_ in fit["bolts"] if b_["group"] in ("frame", "engine foot", "splice")]
        P_fr = 2 * P / len(fb_s)
        d_min = min(float(b_["d"]) for b_ in fb_s)
        lab_b = " + ".join(f"{sum(1 for q in fb_s if q['d'] == d_)} x M{d_ * 1000:.0f}" for d_ in sorted({q["d"] for q in fb_s}))
        R.add(f"T-FINFIT-FR-{tag}", "tail", f"fin fitting {fit['part']} to {fit['frame']}: {lab_b}, bearing in the "
              "frame land (3.2 mm, smallest bolt)", f"dikey bağlantısı {fit['part']} - {fit['frame']}: {lab_b}, çerçeve "
              "kenar bandında ezilme (3,2 mm, en küçük cıvata)", casef, casef_tr, P_fr,
              d_min * int(c.D["body"]["frame_land_plies"]) * ply_props(c, "cfrp_pw_mtm45_as4")["t"] *
              qi_design_values(c)["bearing_Pa"], "N", total_factor(c, fit=True, comp=True), "QI bearing ETW",
              part=fit["frame"])
    # ventral fin / bumper (design choice: 1.0 x MTOM weight at 45 deg up-aft through the skid, CS-LUAS H.9 direction)
    vb = S["tail"]["surfaces"]["ventral"]["bumper"]["contact_point"]
    P_b = 1.0 * c.m0 * G
    Fx, Fz = P_b / math.sqrt(2), P_b / math.sqrt(2)
    vf = [f for f in S["layout"]["chassis"]["fittings"] if f["id"].startswith("F-VENTRAL")]
    pts = np.array([[f["point"][0], f["point"][2]] for f in vf])
    cen = pts.mean(axis=0)
    Mz = Fz * (vb[0] - cen[0]) - Fx * (vb[2] - cen[1])
    bf = ST.bolt_group_inplane(pts, (Fx, Fz), Mz)
    caseb = ("tail-bumper strike (design choice): 1.0 x MTOM weight at 45 deg up and aft through the skid (CS-LUAS "
             "App. H H.9 direction; no tail-wheel load rule for a tricycle bumper)")
    caseb_tr = ("kuyruk tamponu çarpması (tasarım kararı): skidden 45° yukarı-geri 1,0 x MTOM ağırlığı (CS-LUAS Ek H H.9 "
                "yönü; burun tekerli uçak tamponu için kuyruk tekeri kuralı yok)")
    R.add("T-VENTRAL-BOLT", "tail", "ventral root fittings: most loaded lug bolt M6 12.9 double shear", "ventral kök "
          "bağlantıları: en yüklü kulak cıvatası M6 12.9 çift kesme", caseb, caseb_tr, float(bf.max()),
          2 * 0.6 * 1220e6 * 20.1e-6, "N", total_factor(c, fit=True), "elastic bolt group (3 lugs on the keel line)",
          part="YK250-CH-099..101")
    R.add("T-VENTRAL-LUG", "tail", "ventral root lug bearing (7075, t 6 mm)", "ventral kök kulağı ezilmesi (7075, t 6 mm)",
          caseb, caseb_tr, float(bf.max()), 0.006 * 0.006 * float(al["Fbru"]), "N", total_factor(c, fit=True),
          "MMPDS Fbru", part="YK250-CH-099..101")
    akd = c.D["body"]["aft_keel"]
    w_, h_, t_ = float(akd["w_m"]), float(akd["h_m"]), float(akd["t_m"])
    A_w, A_f = 2 * h_ * t_, (w_ - 2 * t_) * t_
    z_c = (A_w * h_ / 2 + A_f * (h_ - t_ / 2)) / (A_w + A_f)
    I_k = 2 * (t_ * h_ ** 3 / 12 + h_ * t_ * (h_ / 2 - z_c) ** 2) + A_f * (h_ - t_ / 2 - z_c) ** 2
    F3 = float(bf[np.argmax(pts[:, 0])])
    M_k = F3 * (pts[:, 0].max() - station_x(c, "FS3738"))
    mk = mat(c, akd["material"])
    R.add("T-AFTKEEL", "tail", f"aft keel beam M-AFTKEEL (machined 7075-T651 channel {w_ * 1000:.0f} x {h_ * 1000:.0f} x "
          f"{t_ * 1000:.1f}) cantilever aft of FS3738, bending", f"arka omurga kirişi M-AFTKEEL (talaşlı 7075-T651 U "
          f"{w_ * 1000:.0f} x {h_ * 1000:.0f} x {t_ * 1000:.1f}) FS3738 gerisinde konsol, eğilme", caseb, caseb_tr,
          M_k * max(z_c, h_ - z_c) / I_k / 1e6, float(mk["Fty"]) / 1e6, "MPa", total_factor(c),
          "M = most aft ventral lug load x arm to FS3738; Fty (local flange crippling not credited above yield)",
          part="YK250-CH-033")
    nrows = [r_ for r_ in R.rows if r_["id"] in ("T-NODE-CHEEK", "T-NODE-INBOARD", "T-NODE-BOSS") and
             r_["ms"] is not None]
    if nrows:
        wr_ = min(nrows, key=lambda r_: r_["ms"])
        R.info("T-NODE-TEMP", "tail", "stabilator node F-SPINDLE-NODE (7075-T651) in the cylinder-head zone: strength "
               "retention the metal rows need at the node design temperature", "silindir kafası bölgesinde stabilatör "
               "düğümü F-SPINDLE-NODE (7075-T651): düğüm tasarım sıcaklığında metal satırlarının gerektirdiği dayanım "
               "oranı", case, case_tr, 1.0 / (1.0 + wr_["ms"]), "F_tu(T) / F_tu(RT)",
               f"governing {wr_['id']} (MS {wr_['ms']:.2f} at room temperature): the 7075 strength at the node "
               "temperature must stay >= this fraction of RT; MMPDS elevated-temperature curves are not in the research "
               "data and the node temperature is not known (61805-ZZ bearing, stainless baffle between heads and node, "
               "layout.heat_protection): open item - measure in the engine run (fix round 2, PK2-09)",
               part="YK250-CH-095")
    return {"N_stab": N, "R_in": R_in, "R_out": R_out, "M_spindle": M_bo, "T_stall": T_st, "N_fin": Nf, "M_fin_root": M_f,
            "P_bumper": P_b, "tail_loads": {k: v for k, v in TL.items() if k != "tp"}}


# =====================================================================================================================
# 4. control surfaces: hinges, horns, push-rods, actuator mounts, ground gust
# =====================================================================================================================
HINGE_COEF = {2: 0.5, 3: 0.625, 4: 0.367, 5: 0.393}


def check_controls(c: Ctx, R: Rows) -> dict:
    """Aileron, flap, rudder: surface load = max(CS-VLA App. B average loading w = K(W/S) n/4.4 (>= 55 kg/m2,
    curve by deflection; CTL-007), the hinge moment the actuator can impose / (0.4 c_f) ); hinge reactions as a
    continuous beam on equally spaced hinges; hinge-pin bearing with the 6.67 total factor (CS-LUAS.657); hinge-line
    inertia 24 g (vertical) / 12 g (horizontal) (CS-LUAS.393(b)); horn / push-rod / rod-end with the actuator stall
    torque x the largest linkage ratio (CS-LUAS.395) and the 3.33 push-pull factor (CS-LUAS.693); ground gust
    hinge moment vs the actuator holding torque (CS-VLA 415)."""
    S = c.S
    areas = Z.control_surface_areas(S)
    ws_kg = c.m0 / float(S["wing"]["area"])

    def K_curve(defl):
        KA = 58.6 + 2.284 * max(ws_kg - 24.8, 0.0)
        KB = 58.6 + 1.972 * max(ws_kg - 28.8, 0.0)
        KC = 58.6 + 1.556 * max(ws_kg - 37.1, 0.0)
        if defl >= 30:
            return KA
        if defl >= 20:
            return KB + (KA - KB) * (defl - 20) / 10
        if defl >= 10:
            return KC + (KB - KC) * (defl - 10) / 10
        return KC
    n_m = float(S["structures"]["n_limit_pos"])
    W = S["wing"]
    fin = S["tail"]["surfaces"]["fin"]
    surf = {
        "aileron": {"ctl": W["controls"]["aileron"], "area": areas["aileron"]["area"] / 2, "hinges": 3, "vertical": False,
                    "cf": W["controls"]["aileron"]["chord_fraction"] * float(c.T["c"](areas["aileron"]["y"])),
                    "mass": mass_item(c, "control_surfaces_ailerons_pair") / 2, "defl": 20.0, "tr": "kanatçık"},
        "flap": {"ctl": W["controls"]["flap"], "area": areas["flap"]["area"] / 2, "hinges": 4, "vertical": False,
                 "cf": W["controls"]["flap"]["chord_fraction"] * float(c.T["c"](areas["flap"]["y"])),
                 "mass": mass_item(c, "control_surfaces_flaps_pair") / 2, "defl": 40.0, "tr": "flap"},
        "rudder": {"ctl": fin["controls"]["rudder"], "area": areas["rudder"]["area"] / 2, "hinges": 3, "vertical": True,
                   "cf": fin["params"]["rudder_chord_fraction"] * float(fin["mac"]),
                   "mass": mass_item(c, "control_surfaces_rudders_pair") / 2, "defl": 25.0, "tr": "dümen"}}
    al = mat(c, "al_7075_t6_sheet")
    ss = mat(c, "ss_304_annealed")
    GD = c.D["gear"]
    d_h = float(GD["door_hinge_pin_d_m"])
    out = {}
    for k, s_ in surf.items():
        act = s_["ctl"]["actuator"]
        chk = s_["ctl"]["checks"]
        link = s_["ctl"]["linkage"]
        k_max = float(chk["ratio_max"])
        H_st = float(act["torque_peak_Nm"]) * k_max
        if k == "flap":
            n_ = 2.0
        else:
            n_ = n_m
        w_bar = max(K_curve(s_["defl"]) * n_ / 4.4, 55.0) * G
        N_B = w_bar * s_["area"]
        N_act = H_st / (0.4 * s_["cf"])
        N = max(N_B, N_act)
        nh = s_["hinges"]
        Rh = HINGE_COEF.get(nh, 0.4) * N
        t_lug = 0.003
        case = (f"{k}: surface load {N:.0f} N (limit) = max(CS-VLA App. B w {w_bar:.0f} Pa x {s_['area']:.3f} m2, "
                f"stall hinge moment {H_st:.1f} N m / 0.4 c_f); {nh} hinges")
        case_tr = (f"{s_['tr']}: yüzey yükü {N:.0f} N (limit) = max(CS-VLA Ek B w {w_bar:.0f} Pa x {s_['area']:.3f} m2, "
                   f"durma menteşe momenti {H_st:.1f} N m / 0,4 c_f); {nh} menteşe")
        R.add(f"C-HINGE-{k.upper()}", "controls", f"{k} hinge: pin d {d_h * 1000:.0f} stainless in a 3 mm 7075 bracket lug, "
              "bearing (6.67 total)", f"{s_['tr']} menteşesi: Ø{d_h * 1000:.0f} paslanmaz pim, 3 mm 7075 dirsek kulağında "
              "ezilme (6,67 toplam)", case, case_tr, Rh, d_h * t_lug * float(al["Fbru"]), "N",
              total_factor(c, total=c.f["hinge_total"]), "CS-LUAS.657 hinge bearing factor 6.67; MMPDS Fbru",
              part=f"{k} hinge")
        R.add(f"C-HPIN-{k.upper()}", "controls", f"{k} hinge pin d {d_h * 1000:.0f} (304), double shear", f"{s_['tr']} "
              f"menteşe pimi Ø{d_h * 1000:.0f} (304), çift kesme", case, case_tr, Rh,
              2 * math.pi * d_h ** 2 / 4 * float(ss["Fsu"]), "N", total_factor(c, fit=True), "MMPDS 304 Fsu",
              part=f"{k} hinge")
        g_hl = 24.0 if s_["vertical"] else 12.0
        F_hl = g_hl * s_["mass"] * G
        R.add(f"C-HLINE-{k.upper()}", "controls", f"{k} hinge-line stop (one hinge takes the axial inertia), bracket lug "
              "bearing", f"{s_['tr']} menteşe ekseni dayanağı (eksenel ataleti tek menteşe taşır), dirsek kulağı ezilmesi",
              f"inertia parallel to the hinge line {g_hl:.0f} x surface weight (CS-LUAS.393(b))",
              f"menteşe eksenine paralel atalet {g_hl:.0f} x yüzey ağırlığı (CS-LUAS.393(b))", F_hl,
              d_h * t_lug * float(al["Fbru"]), "N", total_factor(c, total=c.f["hinge_total"]), "CS-LUAS.657",
              part=f"{k} hinge")
        r_s = float(link["servo_arm_m"])
        r_h = r_s * float(link["arm_ratio"])
        F_push = H_st / r_h
        casep = (f"{k}: actuator peak torque {act['torque_peak_Nm']} N m x largest linkage ratio {k_max:.2f} = "
                 f"{H_st:.1f} N m at the horn (CS-LUAS.395: the system limit load is the actuator output)")
        casep_tr = (f"{s_['tr']}: eyleyici tepe torku {act['torque_peak_Nm']} N m x en büyük bağlantı oranı {k_max:.2f} = "
                    f"boynuzda {H_st:.1f} N m (CS-LUAS.395: sistem limit yükü eyleyici çıktısıdır)")
        R.add(f"C-RODEND-{k.upper()}", "controls", f"{k} push-rod end bolt M3 A2-70 in the horn (t 3 mm 7075), bearing "
              "(3.33 total)", f"{s_['tr']} itme çubuğu ucu cıvatası M3 A2-70 boynuzda (t 3 mm 7075), ezilme (3,33 toplam)",
              casep, casep_tr, F_push, 0.003 * 0.003 * float(al["Fbru"]), "N",
              total_factor(c, total=c.f["pushpull_total"]), "CS-LUAS.693 push-pull joint factor 3.33",
              part=f"{k} horn")
        A_rod = math.pi / 4 * (0.006 ** 2 - 0.004 ** 2)
        I_rod = math.pi / 64 * (0.006 ** 4 - 0.004 ** 4)
        L_rod = float(link["pushrod_base_m"])
        je = ST.johnson_euler(float(al["E"]), float(al["Fcy"]), A_rod, I_rod, L_rod)
        R.add(f"C-PUSHROD-{k.upper()}", "controls", f"{k} push-rod 7075 tube 6 x 1 mm, L {L_rod * 1000:.0f} mm, column",
              f"{s_['tr']} itme çubuğu 7075 boru 6 x 1 mm, L {L_rod * 1000:.0f} mm, kolon", casep, casep_tr, F_push,
              je["P_cr"], "N", total_factor(c), f"Johnson-Euler ({je['mode']}), pinned ends", part=f"{k} push-rod")
        F_ins = F_push / 4 * 1.5
        b_p = 0.0045
        pp = ply_props(c, "cfrp_pw_mtm45_as4")
        R.add(f"C-ACTMOUNT-{k.upper()}", "controls", f"{k} actuator mount: 4 x M4 potted inserts in the servo frame "
              "(solid laminate 2.0 mm), face bearing per insert (moment share 1.5)", f"{s_['tr']} eyleyici bağlantısı: "
              "servo çerçevesinde 4 x M4 gömülü burç (2,0 mm dolu lamine), burç başına yüz ezilmesi (moment payı 1,5)",
              casep, casep_tr, F_ins, 2 * b_p * 10 * pp["t"] * qi_design_values(c)["bearing_Pa"], "N",
              total_factor(c, fit=True, comp=True), "QI bearing on the insert flange diameter 9 mm", part=f"{k} actuator")
        V_gg = min(2.01 * math.sqrt(ws_kg) + 4.45, 26.8)
        Kg_ = 0.75
        H_gg = Kg_ * s_["cf"] * s_["area"] * 0.5 * RHO0 * V_gg ** 2
        H_hold = float(act["torque_rated_Nm"]) * float(chk["ratio_min"])
        R.add(f"C-GGUST-{k.upper()}", "controls", f"{k} ground gust hinge moment vs actuator rated torque x minimum "
              "linkage ratio (powered, holding)", f"{s_['tr']} yer rüzgârı menteşe momenti / eyleyici anma torku x en "
              "küçük bağlantı oranı (enerjili tutma)", f"ground gust V {V_gg:.1f} m/s, K 0.75 (CS-VLA 415)",
              f"yer rüzgârı V {V_gg:.1f} m/s, K 0,75 (CS-VLA 415)", H_gg, H_hold, "N m",
              dict(total_factor(c, ult_only=True), text="limit vs rated (functional)"),
              "CS-VLA 415; gust lock not required if the margin holds", part=f"{k} actuator")
        out[k] = {"N": N, "N_appB": N_B, "N_actuator": N_act, "hinge_reaction": Rh, "H_stall": H_st, "F_push": F_push}
    return out


# =====================================================================================================================
# 5. ground loads and landing gear (CS-LUAS Appendix H, STANAG 4703 Annex B)
# =====================================================================================================================
def ground_loads(c: Ctx) -> dict:
    """Limit ground loads. Energy method: descent velocity CS-LUAS H.2(b) (spec.structures.derived.sink_speed_m_s),
    drop height h = V^2 / 2g, wing lift L = 2/3 W during the impact, available deflection = shock stroke (landing_gear.
    main.stroke, spring/oil damper, e_f 0.65) + tyre static deflection (lower bound of the tyre stroke, e_f 0.5):
    n_j = (h + (1 - L) d) / (e_f d) (STANAG UL.GL.1). Cases per landing gear: level landing on the mains (AMC VLA
    479(b) spin-up / spring-back / max-vertical combinations), three-point level landing (nose share by the static
    split at the forward CG), tail-down and one-wheel (same leg load), side load (H.7), braked roll (H.8), nose-wheel
    supplementary cases (H.10 with the STANAG forward case 3.2), reserve energy 1.2 V_sink, lift = W (H.12(b)).
    Ground contact points from landing_gear (axles, tyre radius, static attitude)."""
    S = c.S
    LG = S["landing_gear"]
    W = c.m0 * G
    V = float(S["structures"]["derived"]["sink_speed_m_s"])
    h = V ** 2 / (2 * G)
    d_s = float(LG["main"]["stroke"])
    d_t = float(LG["tyre"]["static_deflection"])
    d = d_s + d_t
    ef = (0.65 * d_s + 0.5 * d_t) / d
    Lf = float(S["structures"]["landing"]["wing_lift_fraction"])
    nl = ST.landing_nj(h, d, ef, Lf)
    h_res = (1.2 * V) ** 2 / (2 * G)
    nj_res = ST.landing_nj(h_res, d, ef, 1.0)["nj"]
    split_main = float(LG["checks"]["static_load_split_main_aft_cg"])
    nose_static = float(LG["checks"]["nose_load_fwd_cg"]) * W
    nj = nl["nj"]
    Pv_leg = nj * W / 2
    th_td = math.radians(float(LG["checks"]["bumper_contact_deg"]))
    cases = {
        "level_spin_up": (0.6 * Pv_leg, -0.5 * Pv_leg, 0.0),          # (Pz, Px aft-positive? x: + aft), Py
        "level_spring_back": (0.8 * Pv_leg, 0.5 * Pv_leg, 0.0),
        "level_max_vertical": (1.0 * Pv_leg, 0.25 * Pv_leg, 0.0),
        "side_inboard": (1.33 * W / 2, 0.0, -0.5 * W),
        "side_outboard": (1.33 * W / 2, 0.0, 0.33 * W),
        "braked_roll": (1.33 * W * split_main / 2, 0.8 * 1.33 * W * split_main / 2, 0.0),
        # tail-down landing (H.6): vertical reaction at the tail-bumper contact attitude, in body axes; one-wheel
        # landing (H.5): the level-landing leg load on one leg (leg / fitting loads as level, the unbalanced roll
        # moment is reacted by the roll inertia of the aircraft)
        "tail_down": (Pv_leg * math.cos(th_td), -Pv_leg * math.sin(th_td), 0.0),
        "one_wheel": (1.0 * Pv_leg, 0.25 * Pv_leg, 0.0)}
    nose = {"three_point_level": (nj * float(LG["checks"]["nose_load_fwd_cg"]) * W, 0.0, 0.0),
            "three_point_spin_up": (0.6 * nj * float(LG["checks"]["nose_load_fwd_cg"]) * W,
                                    0.5 * 0.6 * nj * float(LG["checks"]["nose_load_fwd_cg"]) * W, 0.0),
            "aft_load": (2.25 * nose_static, 1.8 * nose_static, 0.0),
            "forward_load": (3.2 * nose_static, -0.9 * nose_static, 0.0),
            "side_load": (2.25 * nose_static, 0.0, 1.575 * nose_static)}
    return {"V_sink": V, "h_drop": h, "d": d, "ef": ef, "L": Lf, "nj": nj, "n_inertia": nl["n_inertia"],
            "tail_down_attitude_deg": math.degrees(th_td),
            "nj_reserve_ultimate": nj_res, "Pv_leg": Pv_leg, "nose_static": nose_static, "main_cases": cases,
            "nose_cases": nose, "W": W, "split_main": split_main,
            "n_concentrated_mass": nl["n_inertia"] if nl["nj"] > float(S["structures"]["n_limit_pos"]) - 0.67 else None}


def doubled_face(c: Ctx, sk: dict, n_doubler: int) -> dict:
    """Outer face laminate of a sandwich layup with ``n_doubler`` added +-45 PW plies (shear walls)."""
    pp = ply_props(c, "cfrp_pw_mtm45_as4")
    return ST.laminate_abd([(pl[0], pl[1], pl[2]) for pl in sk["outer"]["plies"]] + [(pp, 45.0, pp["t"])] * n_doubler)


def check_gear(c: Ctx, R: Rows) -> dict:
    """Main gear: leg tube bending at the trunnion, trunnion bearings (two-bearing couple), down-lock pin (moment about
    the retraction axis), trunnion fitting bolts into the gear-beam land, gear beam (sandwich wall + caps) between the
    rear-spar frame and FS-GEAR. Nose gear: leg, pivot bushings in the keel walls, keel-wall shear. Retraction torque
    requirements (gear EMAs) at the gear operating speed (design choice 1.6 VS, n = 2.0 of CS-LUAS.345), door air loads.
    Directions: x aft, y outboard (starboard), z up; Px > 0 = drag (aft)."""
    S = c.S
    GL = ground_loads(c)
    GD = c.D["gear"]
    LG = S["landing_gear"]
    tr_ = np.asarray(LG["main"]["trunnion"], float)
    ax = np.asarray(LG["main"]["axle_static"], float)
    r_t = float(LG["tyre"]["diameter"]) / 2 - float(LG["tyre"]["static_deflection"])
    gnd = ax + np.array([0.0, 0.0, -r_t])
    r = gnd - tr_
    leg = GD["main_leg"]
    m7 = mat(c, leg["material"])
    ro = leg["od_m"] / 2
    ri = ro - leg["wall_m"]
    I = math.pi / 4 * (ro ** 4 - ri ** 4)
    A = math.pi * (ro ** 2 - ri ** 2)
    fit = [f for f in S["layout"]["chassis"]["fittings"] if f["id"] == "F-TRUNNION"][0]
    xb = [float(b["x"]) for b in fit["bearings"]]
    dxb = abs(xb[1] - xb[0])
    worst = {"leg": (0, ""), "brg": (0, ""), "lock": (0, ""), "fit": (0, "")}
    out = {"cases": {}}
    for name, (Pz, Px, Py) in GL["main_cases"].items():
        F = np.array([Px, Py, Pz])
        M = np.cross(r, F)                          # moment about the trunnion point
        u = (ax - tr_) / np.linalg.norm(ax - tr_)
        M_perp = M - (M @ u) * u
        sig = np.linalg.norm(M_perp) * ro / I + abs(F @ u) / A
        lock = abs(M[0]) / float(GD["main_downlock_radius_m"])
        # trunnion bearings: forces Fy, Fz + moments My, Mz about the leg point between the bearings
        Rb = math.hypot(abs(F[1]) / 2 + abs(M[2]) / dxb, abs(F[2]) / 2 + abs(M[1]) / dxb)
        out["cases"][name] = {"F": F.tolist(), "M_trunnion": M.tolist(), "sigma_leg": sig, "lock": lock, "bearing": Rb}
        for k_, v in (("leg", sig), ("brg", Rb), ("lock", lock)):
            if v > worst[k_][0]:
                worst[k_] = (v, name)
        Ffit = math.hypot(math.hypot(F[0], F[1]), F[2])
        if Ffit > worst["fit"][0]:
            worst["fit"] = (Ffit, name)
    gl_txt = (f"CS-LUAS App. H: V_sink {GL['V_sink']:.2f} m/s, d {GL['d'] * 1000:.0f} mm, e_f {GL['ef']:.2f}, "
              f"n_j {GL['nj']:.2f}, n {GL['n_inertia']:.2f}; per main leg P_v {GL['Pv_leg']:.0f} N (limit)")
    gl_tr = (f"CS-LUAS Ek H: V_çökme {GL['V_sink']:.2f} m/s, d {GL['d'] * 1000:.0f} mm, e_f {GL['ef']:.2f}, "
             f"n_j {GL['nj']:.2f}, n {GL['n_inertia']:.2f}; ana bacak başına P_v {GL['Pv_leg']:.0f} N (limit)")
    v, nm = worst["leg"]
    R.add("G-MLEG", "gear", f"main leg outer cylinder {leg['od_m'] * 1000:.0f} x {leg['wall_m'] * 1000:.1f} mm "
          f"7075 at the trunnion yoke, bending + axial (governing: {nm})", f"ana bacak dış silindiri "
          f"{leg['od_m'] * 1000:.0f} x {leg['wall_m'] * 1000:.1f} mm 7075, mafsal çatalında eğilme + eksenel "
          f"(belirleyici: {nm})", gl_txt, gl_tr, v / 1e6, float(m7["Ftu"]) / 1e6, "MPa", total_factor(c),
          "elastic, Ftu (7075-T6 sheet values as bar estimate)", part="YK250-LG-620")
    v, nm = worst["brg"]
    al = mat(c, "al_7075_t651_plate")
    lgd = fit["lug"]
    Dp, tl = float(lgd["bore"]), float(lgd["t_m"])
    el, wl = float(lgd["e_m"]), float(lgd["w_m"])
    lg = ST.lug_axial(float(al["Ftu"]), float(al["Ftu"]), w=wl, D=Dp, t=tl, e=el)
    R.add("G-TRUN-BR", "gear", f"trunnion bearing lug (7075, bore {Dp * 1000:.0f} mm, t {tl * 1000:.0f} mm, e/D "
          f"{el / Dp:.2f} < 2: lug exception), shear-bearing (governing: {nm})", f"mafsal yatak kulağı (7075, "
          f"Ø{Dp * 1000:.0f}, t {tl * 1000:.0f} mm, e/D {dec(el / Dp)} < 2: kulak istisnası), kesme-ezilme (belirleyici: "
          f"{nm})", gl_txt, gl_tr, v, lg["P_shear_bearing"], "N", total_factor(c, bear=True),
          f"Bruhn D1.5 lug shear-bearing at the actual e/D: P = K_br F_tu D t, K_br {lg['Kbr']:.2f} (structlib.lug_axial, "
          "conservative fit); bearing factor 2.0 (rotating joint) - fix round 1, S1-09", part="YK250-CH-070")
    R.add("G-TRUN-NET", "gear", f"trunnion lug net section (w {wl * 1000:.0f} mm, D {Dp * 1000:.0f} mm)",
          f"mafsal kulağı net kesiti (w {wl * 1000:.0f} mm, D {Dp * 1000:.0f} mm)", gl_txt, gl_tr, v, lg["P_net"], "N",
          total_factor(c, fit=True), f"Bruhn D1: K_t {lg['Kt']:.2f} F_tu (w - D) t", part="YK250-CH-070")
    st4 = mat(c, "steel_4130_n")
    R.add("G-TRUN-PIN", "gear", f"trunnion stub axle d {Dp * 1000:.0f} (4130), single shear",
          f"mafsal muylusu Ø{Dp * 1000:.0f} (4130), tek kesme", gl_txt, gl_tr, v, math.pi * Dp ** 2 / 4 * float(st4["Fsu"]),
          "N", total_factor(c, fit=True), "MMPDS 4130 Fsu", part="gear unit")
    sax_m = (fit.get("pivot_scheme") or {}).get("stub_axle")
    if sax_m:
        Dsa, arm_m = float(sax_m["d"]), float(sax_m["arm_m"])
        R.add("G-TRUN-AXLE-BEND", "gear", f"trunnion stub axle d {Dsa * 1000:.0f} bending: bearing load at the arm "
              f"{arm_m * 1000:.0f} mm from the yoke-arm face to the bush centre (cantilever), elastic",
              f"mafsal muylusu Ø{Dsa * 1000:.0f} eğilmesi: yatak yükü, çatal kolu yüzünden burç merkezine "
              f"{arm_m * 1000:.0f} mm kol (konsol), elastik", gl_txt, gl_tr, v * arm_m * 32 / (math.pi * Dsa ** 3) / 1e6,
              float(mat(c, sax_m.get("material", "steel_4130_n"))["Ftu"]) / 1e6, "MPa", total_factor(c, fit=True),
              "M = R_b arm, sigma = 32 M / (pi d^3); MMPDS 4130 Ftu (no plastic bending credit) - fix round 2, PK2-04 "
              "stub-axle scheme", part="gear unit")
    so = shear_out(tl, el, Dp, float(al["Fsu"]))
    R.add("G-TRUN-SO", "gear", f"trunnion lug shear-out (e {el * 1000:.1f} mm)", f"mafsal kulağı kesme yırtılması (e "
          f"{dec(el * 1000, 1)} mm)", gl_txt, gl_tr, v, so, "N", total_factor(c, fit=True), "Bruhn D1: 2 t (e - D/2 cos 40) "
          "Fsu at the actual edge distance", part="YK250-CH-070")
    v, nm = worst["lock"]
    Dl = float(GD["main_downlock_pin_d_m"])
    dl = GD.get("main_downlock") or {}
    stl = mat(c, dl.get("material", "steel_4130_n"))
    R.add("G-DOWNLOCK", "gear", f"main down-lock bolt d {Dl * 1000:.0f} (4130) at r {GD['main_downlock_radius_m'] * 1000:.0f} "
          f"mm from the retraction axis, double shear (governing: {nm})", f"ana takım aşağı kilit cıvatası Ø{Dl * 1000:.0f} "
          f"(4130), toplama ekseninden r {GD['main_downlock_radius_m'] * 1000:.0f} mm, çift kesme (belirleyici: {nm})",
          gl_txt, gl_tr, v, 2 * math.pi * Dl ** 2 / 4 * float(stl["Fsu"]), "N", total_factor(c, fit=True),
          "moment about the trunnion axis / lock radius", part="YK250-CH-070 / gear unit")
    if dl:
        Mlk = ST.pin_bending_moment(v, float(dl["ear_t_m"]), float(dl["inner_t_m"]), float(dl["gap_m"]))
        R.add("G-DOWNLOCK-BEND", "gear", f"main down-lock bolt d {Dl * 1000:.0f} bending in the clevis (Melcon-Hoblit: "
              f"ears {dl['ear_t_m'] * 1000:.0f} mm, lock lug {dl['inner_t_m'] * 1000:.0f} mm, gaps "
              f"{dl['gap_m'] * 1000:.1f} mm), elastic (governing: {nm})", f"ana takım aşağı kilit cıvatası "
              f"Ø{Dl * 1000:.0f} çatalda eğilme (Melcon-Hoblit: kulaklar {dl['ear_t_m'] * 1000:.0f} mm, kilit kulağı "
              f"{dl['inner_t_m'] * 1000:.0f} mm, boşluklar {dec(dl['gap_m'] * 1000, 1)} mm), elastik (belirleyici: {nm})",
              gl_txt, gl_tr, ST.pin_bending_stress(Mlk, Dl) / 1e6, float(stl["Ftu"]) / 1e6, "MPa",
              total_factor(c, fit=True), "structlib.pin_bending_moment / pin_bending_stress, Ftu (no plastic factor) - "
              "fix round 2, VS2-07; requirement for the gear-unit supplier", part="gear unit")
    # ---------------------------------------------------------------- F-TRUNNION bolt group (VS2-02): 6-DOF elastic
    pp = ply_props(c, "cfrp_pw_mtm45_as4")
    t_land = int(c.D["body"]["frame_land_plies"]) * pp["t"]
    qi = qi_design_values(c)
    Rm = float(research_value("materials", "fasteners.property_classes.steel_12_9.tensile_Rm_min_Pa"))
    As = research_value("materials", "fasteners.property_classes.stress_area.A_s_m2")
    ilss = float(research_value("materials", "composites.laminae.cfrp_pw_ooa_prepreg_as4_193.strength_Pa.ETW.ILSS."
                                             "b_basis"))
    TF = GD.get("trunnion_fitting") or {"insert_flange_od_m": 0.014, "nutplate_base_m": 0.020,
                                        "k_axial_over_shear": 1.0}
    bolts = fit["bolts"]
    Pb = np.array([b_["point"] for b_ in bolts], float)
    Ab = np.array([b_["axis"] for b_ in bolts], float)
    # outward (tension) direction of each bolt: away from the land it is anchored in (fitting side)
    out_dir = []
    for b_, pnt in zip(bolts, Pb):
        a_ = np.asarray(b_["axis"], float)
        out_dir.append(-a_ if (np.asarray(tr_, float) - pnt) @ a_ < 0 else a_)
    out_dir = np.array(out_dir)
    d_b = float(bolts[0]["d"])
    As_b = float(As[f"M{d_b * 1000:.0f}"])
    fu_f = total_factor(c, fit=True)
    fc_f = total_factor(c, fit=True, comp=True)
    wr = {"int": (9.0, ""), "br": (0.0, ""), "pull_beam": (0.0, ""), "pull_roof": (0.0, "")}
    is_roof = np.array([abs(a_[2]) > 0.9 for a_ in Ab])
    for name, (Pz_, Px_, Py_) in GL["main_cases"].items():
        F = np.array([Px_, Py_, Pz_])
        M = np.cross(r, F)
        g6 = ST.bolt_group_6dof(Pb, Ab, F, M, tr_, k_axial=float(TF["k_axial_over_shear"]), k_shear=1.0)
        T = np.maximum(np.einsum("ij,ij->i", g6["forces"], out_dir), 0.0)
        Sh = g6["shear"]
        rr_ = 1.0 / np.sqrt((Sh * fu_f["total"] / (0.6 * Rm * As_b)) ** 2 + (T * fu_f["total"] / (Rm * As_b)) ** 2 +
                            1e-30)
        k = int(np.argmin(rr_))
        if rr_[k] < wr["int"][0]:
            wr["int"] = (float(rr_[k]), f"{name}, bolt {bolts[k]['id']}")
        if float(Sh.max()) > wr["br"][0]:
            wr["br"] = (float(Sh.max()), f"{name}, bolt {bolts[int(np.argmax(Sh))]['id']}")
        Tb, Tr = float(T[~is_roof].max(initial=0.0)), float(T[is_roof].max(initial=0.0))
        if Tb > wr["pull_beam"][0]:
            wr["pull_beam"] = (Tb, name)
        if Tr > wr["pull_roof"][0]:
            wr["pull_roof"] = (Tr, name)
    v, nm = wr["int"]
    n_beam, n_roof = int((~is_roof).sum()), int(is_roof.sum())
    R.add("G-FIT-BOLTS", "gear", f"trunnion fitting bolt group ({n_beam} x M{d_b * 1000:.0f} 12.9 axis y into the gear "
          f"beam, {n_roof} x M{d_b * 1000:.0f} 12.9 axis z into the well roof): rigid fitting on elastic bolts, force + "
          f"three moments at the trunnion, shear / tension interaction (governing: {nm})", f"mafsal bağlantısı cıvata "
          f"grubu ({n_beam} x M{d_b * 1000:.0f} 12.9 y ekseni takım kirişine, {n_roof} x M{d_b * 1000:.0f} 12.9 z ekseni "
          f"kuyu tavanına): esnek cıvatalar üzerinde rijit bağlantı, mafsalda kuvvet + üç moment, kesme / çekme "
          f"etkileşimi (belirleyici: {nm})", gl_txt, gl_tr, 1.0, v, "load multiplier",
          dict(fu_f, text="FoS 1.5 x fitting 1.15 (in the multiplier)"),
          "structlib.bolt_group_6dof (equal bolt stiffness, estimate); ISO 898-1 12.9; R_s^2 + R_t^2 = 1 - fix round 2, "
          "VS2-02 (was: |F| / 10 x 1.5 bearing only)", part="YK250-CH-070")
    v, nm = wr["br"]
    R.add("G-FIT-BR", "gear", f"trunnion fitting bolts: bearing of the most loaded bolt in the 16-ply solid land "
          f"(t {t_land * 1000:.1f} mm) (governing: {nm})", f"mafsal bağlantısı cıvataları: en yüklü cıvatanın 16 katlı "
          f"dolu bantta ezilmesi (t {dec(t_land * 1000, 1)} mm) (belirleyici: {nm})", gl_txt, gl_tr, v,
          d_b * t_land * qi["bearing_Pa"], "N", fc_f, "QI bearing ETW (6-DOF bolt shear)", part="YK250-CH-031 / -030")
    v, nm = wr["pull_beam"]
    Dfl = float(TF["insert_flange_od_m"])
    R.add("G-FIT-PULL-BEAM", "gear", f"gear-beam inserts: pull-through of the insert flange (OD {Dfl * 1000:.0f}) through "
          f"the 16-ply land, most loaded bolt (governing: {nm})", f"takım kirişi ankrajları: ankraj flanşının (OD "
          f"{Dfl * 1000:.0f}) 16 katlı banttan sıyrılması, en yüklü cıvata (belirleyici: {nm})", gl_txt, gl_tr, v,
          math.pi * Dfl * t_land * ilss, "N", fc_f, "punching shear pi D t ILSS ETW B-basis (estimate until test)",
          part="YK250-CH-031")
    v, nm = wr["pull_roof"]
    Dnp = float(TF["nutplate_base_m"])
    R.add("G-FIT-PULL-ROOF", "gear", f"well-roof dome nutplates: pull-through of the nutplate base ({Dnp * 1000:.0f} mm) "
          f"through the 16-ply roof land, most loaded bolt (governing: {nm})", f"kuyu tavanı kubbe somun plakaları: "
          f"somun plakası tabanının ({Dnp * 1000:.0f} mm) 16 katlı tavan bandından sıyrılması, en yüklü cıvata "
          f"(belirleyici: {nm})", gl_txt, gl_tr, v, 4 * Dnp * t_land * ilss, "N", fc_f,
          "punching shear (square base perimeter) x t x ILSS ETW B-basis (estimate until test)", part="M-WELLROOF")
    # ---------------------------------------------------------------- gear beam (VS2-02 / PK2-03): vertical leg load +
    # drag couple M_y at the trunnion; notch depth at the leg; end reactions incl. the couple; M_x by wall bending
    gb = [m_ for m_ in S["layout"]["chassis"]["members"] if m_["id"] == "M-GEARBEAM"][0]
    x0, x1 = gb["box"][0][0], gb["box"][1][0]
    hb = gb["box"][1][2] - gb["box"][0][2]
    ntc = gb.get("notch")
    hn = (gb["box"][1][2] - float(ntc["z_top"])) if ntc else hb
    xt = float(tr_[0])
    Lb = x1 - x0
    a_, b_ = xt - x0, x1 - xt
    wb = {"M": (0.0, ""), "V": (0.0, ""), "R": (0.0, ""), "Mx": (0.0, "")}
    for name, (Pz_, Px_, Py_) in GL["main_cases"].items():
        F = np.array([Px_, Py_, Pz_])
        M = np.cross(r, F)
        Cy = float(M[1])
        Mm = max(abs(Pz_ * a_ * b_ / Lb - Cy * a_ / Lb), abs(Pz_ * a_ * b_ / Lb + Cy * b_ / Lb))
        Vm = max(abs(Pz_ * b_ / Lb - Cy / Lb), abs(Pz_ * a_ / Lb + Cy / Lb))
        for k_, v_ in (("M", Mm), ("V", Vm), ("R", Vm), ("Mx", abs(float(M[0])))):
            if v_ > wb[k_][0]:
                wb[k_] = (v_, name)
    ud = ply_props(c, "cfrp_ud_mtm45_as4")
    A_cap = 0.020 * 12 * ud["t"]
    Mb, nmM = wb["M"]
    eps = Mb / (hn - 0.004) / (ud["E1"] * A_cap)
    R.add("G-BEAM-CAP", "gear", f"main-gear beam UD caps (12 plies x 20 mm) bending at the leg (notch depth "
          f"{hn * 1000:.0f} mm), vertical load + drag couple M_y, compression strain (governing: {nmM})", "ana takım "
          f"kirişi UD başlıkları (12 kat x 20 mm) bacak hizasında eğilme (çentik derinliği {hn * 1000:.0f} mm), düşey yük "
          f"+ sürükleme çifti M_y, bası birim şekil değiştirmesi (belirleyici: {nmM})", gl_txt, gl_tr, eps * 1e6,
          c.dt["cap_comp"] * 1e6, "µε", total_factor(c), "simply supported between the well wall and FS-GEAR; the lower "
          "cap runs up round the notch (fix round 2, VS2-02 / PK2-03)", part="YK250-CH-031")
    rib = skin_faces(c, "rib_panel")
    nd = int(c.D["body"]["shear_wall_doubler_plies_per_face"])
    face = doubled_face(c, rib, nd)
    Vb, nmV = wb["V"]
    q = Vb / (hb - 0.004)              # the composite wall carries the shear outside the 7075 flange (full depth)
    fpf = ST.first_ply_failure(face, (0.0, 0.0, q / 2), restrained=True)
    dtx = f" + {nd} +-45 plies per face" if nd else ""
    dtx_tr = f" + yüz başına {nd} kat ±45" if nd else ""
    R.add("G-BEAM-WEB", "gear", f"main-gear beam sandwich wall (rib_panel{dtx}) face shear beside the fitting (full depth "
          f"{hb * 1000:.0f} mm), vertical load + drag couple, first-ply failure (governing: {nmV})", f"ana takım kirişi "
          f"sandviç duvarı (rib_panel{dtx_tr}) bağlantı yanında yüz kesmesi (tam derinlik {hb * 1000:.0f} mm), düşey "
          f"yük + sürükleme çifti, ilk katman hasarı (belirleyici: {nmV})", gl_txt, gl_tr, q / 1e3, fpf["R"] * q / 1e3,
          "N/mm", total_factor(c, comp=True), "CLT", part="YK250-CH-031")
    if ntc:
        fm_ = fit["boxes"][1]
        h_fm, t_fm = float(fm_[1][2] - fm_[0][2]), float(fm_[1][1] - fm_[0][1])
        tau_n = 1.5 * Vb / (h_fm * t_fm)
        R.add("G-BEAM-NOTCH", "gear", f"gear beam at the leg notch (x {ntc['x'][0]:.3f}-{ntc['x'][1]:.3f}, top z "
              f"{ntc['z_top']:.3f}): shear carried by the 7075 flange of F-TRUNNION bridging the notch "
              f"({h_fm * 1000:.0f} x {t_fm * 1000:.0f} mm; the notched composite wall not credited) (governing: {nmV})",
              f"bacak çentiğinde takım kirişi: kesme F-TRUNNION'ın çentiği köprüleyen 7075 flanşında ({h_fm * 1000:.0f} x "
              f"{t_fm * 1000:.0f} mm; çentikli kompozit duvar sayılmadı) (belirleyici: {nmV})", gl_txt, gl_tr,
              tau_n / 1e6, float(al["Fsu"]) / 1e6, "MPa", total_factor(c, fit=True),
              "parabolic shear 1.5 V / (h t), Fsu - fix round 2, PK2-03 / VS2-02", part="YK250-CH-070")
    bec = GD["beam_end_clip"]
    Rv, nmR = wb["R"]
    d_c = float(bec["bolt_d_m"])
    ti_ = mat(c, "ti_6al_4v_annealed_sheet")
    R.add("G-BEAM-END", "gear", f"gear-beam end clips: {bec['bolts']} x M{d_c * 1000:.0f} Ti per end, shear of the end "
          f"reaction incl. the drag couple M_y / L (governing: {nmR})", f"takım kirişi uç bağlantıları: uç başına "
          f"{bec['bolts']} x M{d_c * 1000:.0f} Ti, sürükleme çifti M_y / L dahil uç tepkisinin kesmesi (belirleyici: "
          f"{nmR})", gl_txt, gl_tr, Rv / int(bec["bolts"]), math.pi * d_c ** 2 / 4 * float(ti_["Fsu"]), "N",
          total_factor(c, fit=True), "Ti Fsu, single shear", part="YK250-CH-031")
    # roll moment M_x (VS2-02): the 6-DOF bolt forces load the gear-beam wall (y-forces of the beam bolts) and the well
    # roof (z-forces of the roof bolts); each is checked as a simply supported strip under these point loads, effective
    # width = the bolt-pattern length + 2 x 15 mm (the 8 mm 7075 flange / top plate spreads the loads along x)
    roof_f = skin_faces(c, c.D["fuel_bay"]["floor_layup"])
    wall_z0, wall_z1 = float(ntc["z_top"]) if ntc else float(gb["box"][0][2]), float(gb["box"][1][2])
    wk = [m_ for m_ in S["layout"]["chassis"]["members"] if m_["id"] == "M-WELLKEEL"]
    roof_y0 = float(wk[0]["box"][1][1]) if wk else 0.0115
    roof_y1 = float(gb["box"][0][1])
    L_b = float(np.ptp(Pb[~is_roof][:, 0])) + 0.030
    L_r = float(np.ptp(Pb[is_roof][:, 0])) + 0.030

    def strip_mmax(s0, s1, loads):
        """Max bending moment of a simply supported strip s0..s1 under point loads [(s, F)]."""
        Ls = s1 - s0
        R0 = sum(F_ * (s1 - s_) for s_, F_ in loads) / Ls
        best = 0.0
        for s_q in sorted([s0] + [s_ for s_, _ in loads] + [s1]):
            m_ = R0 * (s_q - s0) - sum(F_ * (s_q - s_) for s_, F_ in loads if s_ < s_q)
            best = max(best, abs(m_))
        return best
    wt = {"wall": (0.0, ""), "roof": (0.0, "")}
    for name, (Pz_, Px_, Py_) in GL["main_cases"].items():
        F = np.array([Px_, Py_, Pz_])
        g6 = ST.bolt_group_6dof(Pb, Ab, F, np.cross(r, F), tr_, k_axial=float(TF["k_axial_over_shear"]), k_shear=1.0)
        fw = [(float(p_[2]), float(f_[1])) for p_, f_, ro in zip(Pb, g6["forces"], is_roof) if not ro]
        fr_ = [(float(p_[1]), float(f_[2])) for p_, f_, ro in zip(Pb, g6["forces"], is_roof) if ro]
        mw = strip_mmax(wall_z0, wall_z1, fw) / L_b
        mr = strip_mmax(roof_y0, roof_y1, fr_) / L_r
        if mw > wt["wall"][0]:
            wt["wall"] = (mw, name)
        if mr > wt["roof"][0]:
            wt["roof"] = (mr, name)
    n_rd = int(GD.get("roof_fitting_doubler_plies_per_face", 0))
    pp_ = ply_props(c, "cfrp_pw_mtm45_as4")
    roof_f = dict(roof_f, outer=ST.laminate_abd([(pl[0], pl[1], pl[2]) for pl in roof_f["outer"]["plies"]] +
                                                [(pp_, 0.0, pp_["t"])] * n_rd),
                  t_out=roof_f["t_out"] + n_rd * pp_["t"], t_in=roof_f["t_in"] + n_rd * pp_["t"])
    for key, lay, rid, nm_en, nm_tr in (("wall", rib, "G-BEAM-TORSION", "gear-beam wall (rib_panel) between the "
                                          "well-roof and lower-skin edges", "kuyu tavanı ve alt kaplama kenarları "
                                          "arasında takım kirişi duvarı (rib_panel)"),
                                         ("roof", roof_f, "G-ROOF-TORSION", f"well roof (fuel-bay floor + {n_rd} 0/90 "
                                          "plies per face under the fitting) between the gear beam and the keel web",
                                          f"takım kirişi ile omurga perdesi arasında kuyu tavanı (yakıt bölmesi tabanı + "
                                          f"bağlantı altında yüz başına {n_rd} kat 0/90)")):
        m_, nm_ = wt[key]
        dd = lay["c"] + 0.5 * (lay["t_out"] + lay["t_in"])
        Nf = m_ / dd
        fp = ST.first_ply_failure(lay["outer"], (Nf, 0.0, 0.0), restrained=True)
        R.add(rid, "gear", f"leg roll moment M_x and drag couple through the F-TRUNNION bolts: {nm_en}, plate bending "
              f"under the bolt forces (6-DOF), face first-ply failure (governing: {nm_})", f"bacak yuvarlanma momenti M_x "
              f"ve sürükleme çifti F-TRUNNION cıvatalarından: {nm_tr}, cıvata kuvvetleri altında levha eğilmesi (6 "
              f"serbestlik), yüz ilk katman hasarı (belirleyici: {nm_})", gl_txt, gl_tr, Nf / 1e3, fp["R"] * Nf / 1e3,
              "N/mm", total_factor(c, fit=True, comp=True), "simply supported strip, effective width = bolt-pattern "
              "length + 30 mm; face force = m / d; CLT - fix round 2, VS2-02 (defined M_x path: beam wall and well "
              "roof as plates, their edges on the roof / skin / keel-web diaphragms)", part="YK250-CH-031 / M-WELLROOF")
    # nose gear
    pv = np.asarray(LG["nose"]["pivot"], float)
    axn = np.asarray(LG["nose"]["axle_static"], float)
    gn = axn + np.array([0.0, 0.0, -r_t])
    rn = gn - pv
    nl_ = GD["nose_leg"]
    mn = mat(c, nl_["material"])
    ron = nl_["od_m"] / 2
    rin = ron - nl_["wall_m"]
    In = math.pi / 4 * (ron ** 4 - rin ** 4)
    An = math.pi * (ron ** 2 - rin ** 2)
    un = (axn - pv) / np.linalg.norm(axn - pv)
    wn = {"leg": (0, ""), "pivot": (0, ""), "lockM": (0, "")}
    for name, (Pz_, Px_, Py_) in GL["nose_cases"].items():
        F = np.array([Px_, Py_, Pz_])
        rr = (axn - pv) if name in ("aft_load", "side_load") else rn
        M = np.cross(rr, F)
        Mp = M - (M @ un) * un
        sig = np.linalg.norm(Mp) * ron / In + abs(F @ un) / An
        for k_, v_ in (("leg", sig), ("pivot", float(np.linalg.norm(F))), ("lockM", abs(M[1]))):
            if v_ > wn[k_][0]:
                wn[k_] = (v_, name)
    ng_txt = (f"nose gear: static load at the forward CG {GL['nose_static']:.0f} N; level 3-point landing n_j share "
              "(CS-LUAS H.4), supplementary aft 2.25/1.8, forward 3.2/0.9, side 2.25/1.575 x static (H.10, STANAG)")
    ng_tr = (f"burun takımı: ön AM'de statik yük {GL['nose_static']:.0f} N; üç noktalı düz iniş n_j payı (CS-LUAS H.4), "
             "ek durumlar arka 2,25/1,8, ön 3,2/0,9, yan 2,25/1,575 x statik (H.10, STANAG)")
    v, nm = wn["leg"]
    R.add("G-NLEG", "gear", f"nose leg {nl_['od_m'] * 1000:.0f} x {nl_['wall_m'] * 1000:.1f} mm 7075 at the pivot yoke, "
          f"bending + axial (governing: {nm})", f"burun bacağı {nl_['od_m'] * 1000:.0f} x {nl_['wall_m'] * 1000:.1f} mm "
          f"7075, mafsal çatalında eğilme + eksenel (belirleyici: {nm})", ng_txt, ng_tr, v / 1e6, float(mn["Ftu"]) / 1e6,
          "MPa", total_factor(c), "elastic, Ftu", part="YK250-LG-624")
    v, nm = wn["pivot"]
    npv = [f for f in S["layout"]["chassis"]["fittings"] if f["id"] == "F-NG-PIVOT"][0]
    sax_n = npv["pivot_scheme"]["stub_axle"]
    Dn = float(sax_n["d"])
    tn = float(GD["nose_pivot_lug_t_m"])
    Db_n = float(GD["nose_pivot_bush_od_m"])
    blk = npv["boxes"][0]
    e_n = float(LG["nose"]["pivot"][2]) - float(blk[0][2])
    w_n = float(blk[1][0] - blk[0][0])
    lgn = ST.lug_axial(float(al["Ftu"]), float(al["Ftu"]), w=w_n, D=Db_n, t=tn, e=e_n)
    R.add("G-NPIVOT-BR", "gear", f"nose pivot blocks (7075, 2 x {tn * 1000:.0f} mm, flanged bushing OD "
          f"{Db_n * 1000:.0f}, e/D {e_n / Db_n:.2f}): shear-bearing of the bush in the blocks (governing: {nm})",
          f"burun mafsal blokları (7075, 2 x {tn * 1000:.0f} mm, flanşlı burç dış çapı {Db_n * 1000:.0f}, e/D "
          f"{dec(e_n / Db_n)}): burcun bloklarda kesme-ezilmesi (belirleyici: {nm})", ng_txt, ng_tr, v,
          2 * lgn["P_shear_bearing"], "N", total_factor(c, bear=True),
          f"Bruhn D1.5 lug at the actual e/D (K_br {lgn['Kbr']:.2f}, F_tu); bearing factor 2.0", part="YK250-CH-071")
    stn = mat(c, sax_n.get("material", "steel_4130_n"))
    R.add("G-NPIVOT-PIN", "gear", f"nose pivot stub axles 2 x d {Dn * 1000:.0f} (4130, solid, flanged; layout "
          "F-NG-PIVOT pivot_scheme), single shear each", f"burun mafsal muyluları 2 x Ø{Dn * 1000:.0f} (4130, dolu, "
          "flanşlı), her biri tek kesme", ng_txt, ng_tr, v, 2 * math.pi * Dn ** 2 / 4 * float(stn["Fsu"]), "N",
          total_factor(c, fit=True), "MMPDS 4130 Fsu; the pivot load shared by the two axles", part="YK250-LG-624")
    arm_n = float(sax_n["arm_m"])
    R.add("G-NPIVOT-AXLE-BEND", "gear", f"nose pivot stub axle d {Dn * 1000:.0f} bending: half the pivot load at the arm "
          f"{arm_n * 1000:.0f} mm from the yoke-arm face to the bush centre (cantilever), elastic",
          f"burun mafsal muylusu Ø{Dn * 1000:.0f} eğilmesi: mafsal yükünün yarısı, çatal kolu yüzünden burç merkezine "
          f"{arm_n * 1000:.0f} mm kol (konsol), elastik", ng_txt, ng_tr,
          0.5 * v * arm_n * 32 / (math.pi * Dn ** 3) / 1e6, float(stn["Ftu"]) / 1e6, "MPa", total_factor(c, fit=True),
          "M = (P/2) arm, sigma = 32 M / (pi d^3); MMPDS 4130 Ftu (no plastic bending credit) - fix round 2, PK2-04 "
          "stub-axle scheme", part="YK250-LG-624")
    # ground handling (fix round 2, VS2-12): towing at the nose leg with a tow bar on the fork axle
    GH = c.D.get("ground_handling") or {}
    if GH:
        k_tow = float(GH["tow_load_fraction_W"])
        F_tow = k_tow * c.m0 * G
        w_tow = (0.0, "")
        r_ax = axn - pv
        for ang in (0.0, 180.0, 30.0, -30.0, 150.0, -150.0):
            u_ = np.array([math.cos(math.radians(ang)), math.sin(math.radians(ang)), 0.0])
            F = F_tow * u_
            Mt = np.cross(r_ax, F)
            Mp = Mt - (Mt @ un) * un
            sig_t = np.linalg.norm(Mp) * ron / In + abs(F @ un) / An
            if sig_t > w_tow[0]:
                w_tow = (sig_t, f"{ang:+.0f} deg from the aft drag axis")
        tw_txt = (f"towing (design choice): tow load {k_tow:.2f} W = {F_tow:.0f} N at the nose-fork axle, aft / forward "
                  f"and 30 deg to either side ({GH['tow_load_basis']})")
        tw_tr = (f"çekme (tasarım kararı): burun çatalı dingilinde {dec(k_tow)} W = {F_tow:.0f} N çekme yükü, geri / ileri "
                 f"ve iki yana 30° ({GH['tow_load_basis_tr']})")
        R.add("G-TOW-NLEG", "gear", f"nose leg under the towing load at the fork axle (governing: {w_tow[1]})",
              f"çekme yükü altında burun bacağı (çatal dingili; belirleyici: {w_tow[1]})", tw_txt, tw_tr,
              w_tow[0] / 1e6, float(mn["Ftu"]) / 1e6, "MPa", total_factor(c), "elastic, Ftu - fix round 2, VS2-12",
              part="YK250-LG-624")
        R.add("G-TOW-PIVOT", "gear", "nose pivot blocks under the towing load (shear-bearing of the bushes)",
              "çekme yükü altında burun mafsal blokları (burçların kesme-ezilmesi)", tw_txt, tw_tr, F_tow,
              2 * lgn["P_shear_bearing"], "N", total_factor(c, bear=True), "Bruhn D1.5 lug (as G-NPIVOT-BR); bearing "
              "factor 2.0", part="YK250-CH-071")
    R.add("G-NPIVOT-WALL", "gear", "nose pivot block to keel wall: 4 x M6 Ti per side, bearing in the CFRP land "
          f"(t {t_land * 1000:.1f} mm)", "burun mafsal bloğu - omurga duvarı: taraf başına 4 x M6 Ti, CFRP bantta ezilme "
          f"(t {t_land * 1000:.1f} mm)", ng_txt, ng_tr, v / 2 / 4 * 1.5, 0.006 * t_land * qi_design_values(c)["bearing_Pa"],
          "N", total_factor(c, fit=True, comp=True), "peak bolt 1.5 x mean; QI bearing ETW", part="YK250-CH-021")
    v, nm = wn["lockM"]
    R.info("G-NLOCK", "gear", "nose down-lock (over-centre link to the keel walls): limit moment about the pivot axis",
           "burun aşağı kilidi (keel duvarlarına ölü nokta bağlantısı): mafsal ekseni etrafında limit moment", ng_txt,
           ng_tr, v, "N m", f"governing {nm}; the lock link (gear unit) is sized for this moment x 1.5 x 1.15",
           part="gear unit")
    kw = [m_ for m_ in S["layout"]["chassis"]["members"] if m_["id"] == "M-KEELWALL"][0]
    xa, xb_ = kw["box"][0][0], kw["box"][1][0]
    hk = kw["box"][1][2] - kw["box"][0][2]
    Vk = wn["pivot"][0] * (xb_ - pv[0]) / (xb_ - xa) / 2
    face = doubled_face(c, rib, nd)
    qk = Vk / hk
    fpf = ST.first_ply_failure(face, (0.0, 0.0, qk / 2), restrained=True)
    R.add("G-KEELWALL", "gear", "keel wall (rib_panel) shear between the nose pivot and FS0600, face first-ply failure",
          "omurga duvarı (rib_panel) burun mafsalı ile FS0600 arasında kesme, yüz ilk katman hasarı", ng_txt, ng_tr,
          qk / 1e3, fpf["R"] * qk / 1e3, "N/mm", total_factor(c, comp=True), "CLT", part="YK250-CH-021")
    # retraction torque requirements and door air loads (gear operating speed: design choice)
    VS = math.sqrt(2 * c.ws / (RHO0 * float(S["aero"]["clmax_clean"])))
    V_LO = float(GD["gear_operating_speed_factor_vs"]) * VS
    q_LO = c.q(V_LO)
    m_main = mass_item(c, "main_gear_legs_wheels_brakes_emas_pair") / 2
    m_nose = mass_item(c, "nose_gear_leg_wheel_steering")
    L_m = float(LG["main"]["leg_length"])
    L_n = float(LG["nose"]["leg_length"])
    tyre_A = float(LG["tyre"]["diameter"]) * float(LG["tyre"]["width"])
    T_main = 2.0 * m_main * G * 0.6 * L_m + 1.0 * q_LO * (tyre_A + float(LG["main"]["leg_frontal_width"]) * L_m) * 0.0
    T_nose = 2.0 * m_nose * G * 0.6 * L_n + 1.0 * q_LO * (tyre_A + float(LG["nose"]["leg_frontal_width"]) * L_n) * L_n
    rq = f"gear operation up to V_LO {V_LO:.1f} m/s EAS (design choice 1.6 VS, FCS-enforced), n = 2.0 (CS-LUAS.345)"
    rq_tr = f"takım işletimi V_LO {V_LO:.1f} m/s EAS'e kadar (tasarım kararı 1,6 VS, UKS ile sınırlı), n = 2,0 (CS-LUAS.345)"
    R.info("G-EMA-MAIN", "gear", "main-gear retraction EMA: required output torque on the trunnion axis (1.25 x weight "
           "moment at n 2; drag parallel to the axis)", "ana takım toplama EMA'sı: mafsal ekseninde gerekli çıkış torku "
           "(n 2'de ağırlık momenti x 1,25; sürükleme eksene paralel)", rq, rq_tr, 1.25 * T_main, "N m",
           "requirement for the gear unit EMA (CS-LUAS.395 analogue)", part="gear unit")
    R.info("G-EMA-NOSE", "gear", "nose-gear retraction EMA: required output torque on the pivot axis (1.25 x (weight "
           "moment at n 2 + wheel/leg drag x leg length, extension against the flow))", "burun takımı toplama EMA'sı: "
           "mafsal ekseninde gerekli çıkış torku (1,25 x (n 2'de ağırlık momenti + teker/bacak sürüklemesi x bacak "
           "boyu, akışa karşı açılma))", rq, rq_tr, 1.25 * T_nose, "N m", "requirement for the gear unit EMA",
           part="gear unit")
    gdg = Z.gear_door_geometry(S)
    A_in = gdg["area_main_inner_m2"] / 2
    w_in = float(LG["doors"]["main_inner_door_outer_edge_y"]) - float(LG["main"]["well_gap"]) / 2
    F_door = 1.0 * c.q(c.VD) * A_in
    H_door = F_door * w_in / 2
    nh = int(GD["door_hinge_count"])
    dh = float(GD["door_hinge_pin_d_m"])
    m75 = mat(c, "al_7075_t6_sheet")
    case_d = (f"closed main inner door, suction |Cp| 1.0 x q(VD) {c.q(c.VD):.0f} Pa (bound) on {A_in:.4f} m2: "
              f"{F_door:.0f} N, hinge moment {H_door:.1f} N m (limit)")
    case_dtr = (f"kapalı ana iç kapak, emme |Cp| 1,0 x q(VD) {c.q(c.VD):.0f} Pa (üst sınır), {A_in:.4f} m2: "
                f"{F_door:.0f} N, menteşe momenti {H_door:.1f} N m (limit)")
    R.add("G-DOOR-HINGE", "gear", f"main inner door hinges ({nh} x pin d {dh * 1000:.0f} in 3 mm 7075 lugs), bearing",
          f"ana iç kapak menteşeleri ({nh} x Ø{dh * 1000:.0f} pim, 3 mm 7075 kulak), ezilme", case_d, case_dtr,
          F_door * 0.625 * 1.0, dh * 0.003 * float(m75["Fbru"]), "N", total_factor(c, bear=True),
          "bearing factor 2.0 (rotating joint)", part="YK250-LG-670")
    da22 = float(research_value("components", "categories.control_surface_actuators.items")[[
        i["id"] for i in research_value("components", "categories.control_surface_actuators.items")].index(
        "volz_da22_28v")]["torque_rated_Nm"]["value"])
    R.info("G-DOOR-LOCK", "gear", "main inner door: closed-position holding moment (requirement: over-centre linkage or "
           f"door up-lock; DA 22 rated torque {da22} N m needs a linkage ratio >= the value / {da22})",
           "ana iç kapak: kapalı konumda tutma momenti (gereksinim: ölü nokta bağlantısı veya kapak kilidi; DA 22 anma "
           f"torku {da22} N m için bağlantı oranı >= değer / {da22})", case_d, case_dtr, 1.5 * H_door, "N m",
           "ultimate holding moment; the actuator is not relied upon to hold the closed door", part="EQ-DOORACT")
    # fix round 2 (PK2-13 / mass closure): ONE DA 22 drives both nose clamshell doors through the centre-line bellcrank
    nb_ = np.asarray(LG["nose"]["stowed_envelope"]["box"], float)
    A_nd = gdg["area_nose_m2"] / 2
    w_nd = 0.5 * (nb_[1][1] - nb_[0][1])
    H_nd = 1.0 * q_LO * A_nd * w_nd / 2
    cs_nd = (f"nose clamshell doors operated at V_LO {V_LO:.1f} m/s EAS: suction |Cp| 1.0 x q {q_LO:.0f} Pa (bound) on "
             f"{A_nd:.4f} m2 per door, hinge moment {H_nd:.2f} N m per door (limit); both doors on one actuator, "
             "bellcrank ratio 1.0 (no mechanical-advantage credit)")
    cs_nd_tr = (f"burun kapakları V_LO {dec(V_LO, 1)} m/s EAS'te işletilir: emme |Cp| 1,0 x q {q_LO:.0f} Pa (üst sınır), "
                f"kapak başına {dec(A_nd, 4)} m2, kapak başına menteşe momenti {dec(H_nd, 2)} N m (limit); iki kapak tek "
                "eyleyicide, kol oranı 1,0 (mekanik avantaj sayılmadı)")
    R.add("G-NDOOR-DRIVE", "gear", "nose-door drive: one Volz DA 22 for both clamshell doors (EQ-NDOORACT, bellcrank "
          "NDOOR-LINKAGE), required torque vs rated torque", "burun kapağı tahriki: iki kapak için tek Volz DA 22 "
          "(EQ-NDOORACT, kol NDOOR-LINKAGE), gerekli tork / anma torku", cs_nd, cs_nd_tr, 2 * H_nd, da22, "N m",
          dict(total_factor(c), text="FoS 1.5 (functional)"), "components.yaml volz_da22_28v rated torque; the closed "
          "doors are held by the over-centre lock, not by the actuator", part="EQ-NDOORACT")
    return {"ground": {k: v for k, v in GL.items() if k not in ("main_cases", "nose_cases")}, "V_LO": V_LO,
            "T_main_ema_req": 1.25 * T_main, "T_nose_ema_req": 1.25 * T_nose, "door_hinge_moment": H_door,
            "main_cases": out["cases"]}


# =====================================================================================================================
# 6. engine mount (CS-LUAS.361/363/371, CS-VLA 561(c))
# =====================================================================================================================
def engine_suspended_mass(c: Ctx) -> dict:
    """Mass carried by the isolators: bare engine, SG750, coupling, exhaust, throttle servo/sensors
    (layout.mass_placement.engine_group_installed component positions) + propeller and spinner/hub/spacer (mass
    items), x (1 + growth allowance); CG and propeller polar moment of inertia (uniform-rod blades over the radius:
    m R^2 / 3, an upper bound for tapered blades; spinner as a disc)."""
    S = c.S
    gr = 1.0 + float(S["mass"]["rules"]["growth_allowance"])
    E = S["engine"]["installed_items_kg"]
    comps = S["layout"]["mass_placement"]["engine_group_installed"]["components"]
    pos = {}
    for cmp in comps:
        w = cmp["what"]
        for key in ("engine_bare", "SG750", "adapter", "exhaust", "throttle"):
            if w.lower().startswith(key.lower()) or (key == "adapter" and "adapter" in w) or (key == "throttle" and
                                                                                             "throttle" in w):
                pos[key] = np.asarray(cmp["at"], float)
    items = [(float(E["engine_bare"]), pos["engine_bare"]), (float(E["starter_generator_sg750"]), pos["SG750"]),
             (float(E["generator_adapter_coupling"]), pos["adapter"]), (float(E["exhaust_with_silencers"]), pos["exhaust"]),
             (float(E["throttle_servo_cht_egt_sensors"]), pos["throttle"])]
    hub = np.asarray(S["propeller"]["hub"], float)
    m_p = float(S["propeller"]["mass_kg"])
    m_sp = float(S["mass"]["rules"]["spinner_hub_spacer_kg"])
    items += [(m_p, hub), (m_sp, hub - np.array([0.06, 0.0, 0.0]))]
    m = sum(i[0] for i in items) * gr
    cg = sum(i[0] * i[1] for i in items) * gr / m
    R_p = float(S["propeller"]["diameter"]) / 2
    Ip = m_p * R_p ** 2 / 3 + 0.5 * m_sp * (float(S["propeller"]["spinner"]["diameter"]) / 2) ** 2
    return {"m": m, "cg": cg, "Ip": Ip, "items": len(items)}


def rigid_distribution(points: np.ndarray, F: np.ndarray, M: np.ndarray, ref: np.ndarray) -> np.ndarray:
    """Forces at equal isotropic springs (isolators) of a rigid body loaded by F and M about ``ref``: f_i = k (u +
    theta x r_i) with (u, theta) from the 6 equilibrium equations."""
    r = points - ref
    A = np.zeros((6, 6))
    b = np.r_[F, M]
    for ri in r:
        Rx = np.array([[0, -ri[2], ri[1]], [ri[2], 0, -ri[0]], [-ri[1], ri[0], 0]])
        # f = u + theta x r = u - Rx theta
        A[:3, :3] += np.eye(3)
        A[:3, 3:] += -Rx
        A[3:, :3] += Rx
        A[3:, 3:] += -Rx @ Rx
    sol = np.linalg.solve(A, b)
    u, th = sol[:3], sol[3:]
    return np.array([u + np.cross(th, ri) for ri in r])


def engine_mount_cases(c: Ctx) -> dict:
    """Load cases on the engine group (forces / moments at its CG, aircraft axes, x aft, z up): inertia F = -n m g
    (z down for positive n), engine torque about the crank axis, thrust along the crank axis (forward, -x), gyroscopic
    couple. Limit cases x 1.5 (FoS) unless ultimate-only (crash / emergency / parachute)."""
    S = c.S
    es = engine_suspended_mass(c)
    m = es["m"]
    eng = S["engine"]
    em = S["layout"]["chassis"]["engine_mount"]
    u = np.asarray(em["thrust_axis"]["direction_aft"], float)
    u = u / np.linalg.norm(u)
    dl = research_value("standards", "design_loads_recommended.engine_mount")
    T_mcp = float(dl["limit_torque_MCP_Nm"]["value"])
    T_to = float(dl["limit_torque_takeoff_case_Nm"]["value"])
    n_side = float(dl["side_load_factor"]["value"])
    gy = dl["gyroscopic_case"]
    crash = float(dl["crash_forward_ultimate"]["value"])
    thrust = float(S["propeller"]["static_wot_reference"]["thrust_N"])
    rpm_mc = float(eng["power_max_continuous_W"]) / (float(dl["mean_torque_MCP_Nm"]["value"]) * 2 * math.pi / 60)
    Om = rpm_mc * 2 * math.pi / 60
    M_gyro_yaw = es["Ip"] * Om * float(gy["yaw_rate_rad_per_s"])
    M_gyro_pitch = es["Ip"] * Om * float(gy["pitch_rate_rad_per_s"])
    W = m * G
    GL = ground_loads(c)
    n_land = GL["n_inertia"] if GL["n_concentrated_mass"] is not None else 0.0
    para_n = 13100.0 / (c.m_min * G)
    T_fwd = -thrust * u
    zf = np.array([0.0, 0.0, -1.0])
    nA = float(S["structures"]["n_limit_pos"])
    k_t = float(dl["limit_torque_MCP_Nm"]["value"]) / float(dl["mean_torque_MCP_Nm"]["value"])
    gn = float(gy["normal_load_factor"])
    cases = {
        f"MCP torque x{k_t:g} + n_A {nA:g} + thrust": (nA * W * zf + T_fwd, T_mcp * u, False,
                                                       f"MCP torku x{k_t:g} + n_A {nA:g} + itki"),
        f"take-off torque + 0.75 n_A + thrust": (0.75 * nA * W * zf + T_fwd, T_to * u, False,
                                                 "kalkış torku + 0,75 n_A + itki"),
        f"gust n {c.n_eq_pos:.2f} (lightest case) + MCP torque": (c.n_eq_pos * W * zf + T_fwd, T_mcp * u, False,
                                                                  f"hamle n {c.n_eq_pos:.2f} (en hafif durum) + MCP torku"),
        f"negative n {c.n_eq_neg:.2f} + MCP torque": (c.n_eq_neg * W * zf + T_fwd, T_mcp * u, False,
                                                      f"negatif n {c.n_eq_neg:.2f} + MCP torku"),
        f"side load {n_side:g} g": (n_side * W * np.array([0.0, 1.0, 0.0]), np.zeros(3), False, f"yan yük {n_side:g} g"),
        (f"gyroscopic (yaw {float(gy['yaw_rate_rad_per_s']):g}, pitch {float(gy['pitch_rate_rad_per_s']):g} rad/s, "
         f"n {gn:g}) + thrust"): (
            gn * W * zf + T_fwd, np.array([0.0, M_gyro_yaw, M_gyro_pitch]), False,
            f"jiroskopik (sapma {float(gy['yaw_rate_rad_per_s']):g}, yunuslama {float(gy['pitch_rate_rad_per_s']):g} rad/s, "
            f"n {gn:g}) + itki"),
        "landing n_j + 0.67 (concentrated mass)": (n_land * W * zf, np.zeros(3), False,
                                                   "iniş n_j + 0,67 (yoğun kütle)"),
        f"crash {crash:g} g forward (ultimate)": (crash * W * np.array([-1.0, 0.0, 0.0]), np.zeros(3), True,
                                                  f"çarpma {crash:g} g ileri (nihai)"),
        "emergency landing 6 g down (ultimate)": (6.0 * W * zf, np.zeros(3), True, "acil iniş 6 g aşağı (nihai)"),
        "parachute opening 13.1 kN / lightest mass, down + 60 deg forward (ultimate)": (
            para_n * W * np.array([-math.sin(math.radians(60.0)), 0.0, -math.cos(math.radians(60.0))]), np.zeros(3),
            True, "paraşüt açılması 13,1 kN / en hafif kütle, aşağı + 60° ileri (nihai)")}
    return {"cases": cases, "es": es, "u": u, "T_mcp": T_mcp, "M_gyro_yaw": M_gyro_yaw, "M_gyro_pitch": M_gyro_pitch,
            "thrust": thrust, "rpm_mc": rpm_mc, "para_n": para_n}


def check_engine_mount(c: Ctx, R: Rows) -> dict:
    """Isolator loads (rigid engine on 4 equal isolators), crankcase bolts M8 12.9 (shear + tension interaction),
    welded 4130 truss (direct-stiffness pin-jointed space truss: 9 struts + 4 ring segments; isolator loads at the
    nearest ring node; struts: Johnson-Euler column K = 1, tension on the near-weld Ftu; ring: axial + local bending
    from the isolator-to-node offset), firewall feet (2 x M8 12.9 each: shear + tension; bearing in the firewall land)."""
    S = c.S
    EC = engine_mount_cases(c)
    es = EC["es"]
    em = S["layout"]["chassis"]["engine_mount"]
    ED = c.D["engine_mount"]
    iso = np.asarray(em["isolators"]["centres"], float)
    feet = np.asarray(em["feet"], float)
    ring = np.asarray(em["ring_nodes"], float)
    nodes = np.vstack([feet, ring])                       # 0..3 feet, 4..7 ring nodes
    name_idx = {f"foot{i + 1}": i for i in range(4)}
    name_idx.update({f"ring{i + 1}": 4 + i for i in range(4)})
    members = [(name_idx[t["from"]], name_idx[t["to"]]) for t in em["tubes"]]
    n_strut = len(members)
    members += [(4, 6), (6, 7), (7, 5), (5, 4)]           # welded ring r1-r3-r4-r2
    st = mat(c, "steel_4130_n")
    E_, Fcy = float(st["E"]), float(st["Fcy"])
    Ftu_w = float(st["Ftu_weld"])
    so, sw = ED["strut_od_m"], ED["strut_wall_m"]
    ro_, rw = ED["ring_od_m"], ED["ring_wall_m"]
    tube = lambda od, w: (math.pi / 4 * (od ** 2 - (od - 2 * w) ** 2), math.pi / 64 * (od ** 4 - (od - 2 * w) ** 4))  # noqa: E731
    A_s, I_s = tube(so, sw)
    A_r, I_r = tube(ro_, rw)
    EA = [E_ * A_s] * n_strut + [E_ * A_r] * 4
    near = [4 + int(np.argmin(np.linalg.norm(ring - p, axis=1))) for p in iso]
    worst = {}
    out = {}
    feet_ult = {}
    As8 = float(research_value("materials", "fasteners.property_classes.stress_area.A_s_m2")["M8"])
    Rm = float(research_value("materials", "fasteners.property_classes.steel_12_9.tensile_Rm_min_Pa"))
    for name, (F, M, ult, _tr) in EC["cases"].items():
        f_iso = rigid_distribution(iso, F, M, es["cg"])
        loads = {}
        for k_, f in zip(near, f_iso):
            loads[k_] = loads.get(k_, np.zeros(3)) + f
        tr_ = ST.truss3d(nodes, members, [0, 1, 2, 3], loads, EA)
        fac = 1.0 if ult else c.f["fos"]
        rec = {"N": tr_["N"].tolist(), "f_iso": f_iso.tolist(), "reactions": {int(k): v.tolist() for k, v in
                                                                           tr_["reactions"].items()}, "ult": ult}
        out[name] = rec
        # struts
        for j in range(len(members)):
            Nj = tr_["N"][j] * fac
            L_ = tr_["L"][j]
            A_, I_ = (A_s, I_s) if j < n_strut else (A_r, I_r)
            if Nj < 0:
                cap = ST.johnson_euler(E_, Fcy, A_, I_, L_, 1.0)["P_cr"]
                if j >= n_strut:
                    off = 0.035
                    f_loc = max(np.linalg.norm(f) for f in f_iso) * fac
                    sig_b = f_loc * off / 4 * ((ro_ / 2) / I_r)
                    cap = cap * max(1.0 - sig_b / Fcy, 0.05)
                key = "strut_c" if j < n_strut else "ring_c"
                r_ = cap / -Nj
            else:
                key = "strut_t" if j < n_strut else "ring_t"
                cap = Ftu_w * A_
                r_ = cap / max(Nj, 1e-9)
            if key not in worst or r_ < worst[key][0]:
                worst[key] = (r_, name, j, Nj, cap, ult)
        # crankcase bolts at the isolators (shear + tension along the crank axis)
        u = EC["u"]
        for f in f_iso:
            ft = abs(f @ u) * fac * c.f["fit"]
            fs = np.linalg.norm(f - (f @ u) * u) * fac * c.f["fit"]
            r_ = 1.0 / math.sqrt((ft / (Rm * As8)) ** 2 + (fs / (0.6 * Rm * As8)) ** 2)
            if "bolt" not in worst or r_ < worst["bolt"][0]:
                worst["bolt"] = (r_, name, None, math.hypot(ft, fs), None, ult)
        for k_, v in tr_["reactions"].items():
            Ru = -v * fac                                  # ultimate foot reaction on the firewall (aircraft axes)
            rec_f = feet_ult.setdefault(int(k_), {"rx": (0.0, ""), "ryz": (0.0, "")})
            if abs(Ru[0]) > rec_f["rx"][0]:
                rec_f["rx"] = (abs(float(Ru[0])), name)
            if math.hypot(Ru[1], Ru[2]) > rec_f["ryz"][0]:
                rec_f["ryz"] = (float(math.hypot(Ru[1], Ru[2])), name)
            Rv = -v * fac * c.f["fit"]
            ft = abs(Rv[0]) / 2
            fs = math.hypot(Rv[1], Rv[2]) / 2
            r_ = 1.0 / math.sqrt((ft / (Rm * As8)) ** 2 + (fs / (0.6 * Rm * As8)) ** 2)
            if "foot" not in worst or r_ < worst["foot"][0]:
                worst["foot"] = (r_, name, k_, math.hypot(ft, fs), None, ult)
            iso_max = max(np.linalg.norm(f) for f in f_iso) * fac
            if "iso" not in worst or iso_max > worst["iso"][0]:
                worst["iso"] = (iso_max, name, None, None, None, ult)
    txt = (f"engine group on the mount {es['m']:.2f} kg (incl. growth), CG x {es['cg'][0]:.3f} z {es['cg'][2]:.3f}; "
           f"I_p {es['Ip']:.4f} kg m2; torque MCP x 6 = {EC['T_mcp']:.1f} N m; static thrust {EC['thrust']:.0f} N")
    txt_tr = (f"bağlantı üzerindeki motor grubu {es['m']:.2f} kg (büyüme payı dahil), AM x {es['cg'][0]:.3f} z "
              f"{es['cg'][2]:.3f}; I_p {es['Ip']:.4f} kg m2; MCP torku x 6 = {EC['T_mcp']:.1f} N m; statik itki "
              f"{EC['thrust']:.0f} N")
    one = dict(total_factor(c, ult_only=True), text="load multiplier on the factored load")
    for key, lab, lab_tr in (("strut_c", f"engine-mount strut {so * 1000:.1f} x {sw * 1000:.2f} mm 4130, column (Johnson-Euler, K 1)",
                              f"motor bağlantı çubuğu {so * 1000:.1f} x {sw * 1000:.2f} mm 4130, kolon (Johnson-Euler, K 1)"),
                             ("strut_t", "engine-mount strut, tension on the near-weld Ftu",
                              "motor bağlantı çubuğu, kaynak yakını Ftu ile çekme"),
                             ("ring_c", f"isolator ring {ro_ * 1000:.1f} x {rw * 1000:.2f} mm 4130, column + local bending",
                              f"sönümleyici halkası {ro_ * 1000:.1f} x {rw * 1000:.2f} mm 4130, kolon + yerel eğilme"),
                             ("ring_t", "isolator ring, tension (near-weld Ftu)", "sönümleyici halkası, çekme (kaynak yakını Ftu)"),
                             ("bolt", "crankcase bolts M8 12.9 at the isolators, shear + tension interaction (x 1.15 fitting)",
                              "sönümleyicilerdeki karter cıvataları M8 12.9, kesme + çekme etkileşimi (x 1,15 bağlantı)"),
                             ("foot", "firewall feet 2 x M8 12.9 each, shear + tension interaction (x 1.15 fitting)",
                              "yangın perdesi ayakları, her biri 2 x M8 12.9, kesme + çekme etkileşimi (x 1,15 bağlantı)")):
        if key not in worst:
            continue
        r_, nm, j, Nj, cap, ult = worst[key]
        R.add(f"E-{key.upper()}", "engine mount", lab, lab_tr, f"{nm}; {txt}", f"{EC['cases'][nm][3]}; {txt_tr}", 1.0, r_,
              "load multiplier", one, "direct-stiffness space truss (structlib.truss3d); MMPDS 4130 N, near-weld Ftu "
              "80 ksi (AC 43.13-1B); ISO 898-1 12.9; loads already x FoS (limit cases) and x 1.15 (bolts)",
              part="YK250-CH-085" if key not in ("bolt",) else "engine")
    iso_max, nm, *_ = worst["iso"]
    R.info("E-ISOLATOR", "engine mount", "elastomer isolator: required ultimate load capacity (largest resultant per "
           "isolator, fail-safe snubber must retain it)", "elastomer sönümleyici: gerekli nihai yük kapasitesi "
           "(sönümleyici başına en büyük bileşke; emniyet tamponu tutmalı)", f"{nm}; {txt}",
           f"{EC['cases'][nm][3]}; {txt_tr}", iso_max,
           "N", "procurement requirement (Limbach damper shock mount data not published)", part="engine mount isolators")
    pp = ply_props(c, "cfrp_pw_mtm45_as4")
    t_land = int(c.D["body"]["frame_land_plies"]) * pp["t"]
    k_b = max(feet_ult, key=lambda k: feet_ult[k]["ryz"][0])
    P_b, nm_b = feet_ult[k_b]["ryz"]
    nfb = int(ED["foot_bolts"])
    R.add("E-FOOT-BR", "engine mount", f"firewall feet: {nfb} x M8 bearing in the firewall land ({t_land * 1000:.1f} mm "
          "solid laminate), in-plane foot reaction", f"yangın perdesi ayakları: {nfb} x M8, yangın perdesi bandında ezilme "
          f"({dec(t_land * 1000, 1)} mm dolu lamine), düzlem içi ayak tepkisi", f"{nm_b} (ultimate); {txt}",
          f"{EC['cases'][nm_b][3]} (nihai); {txt_tr}", P_b / nfb, 0.008 * t_land * qi_design_values(c)["bearing_Pa"], "N",
          total_factor(c, ult_only=True, fit=True, comp=True), "QI bearing ETW; stainless spacer tubes not credited",
          part="FS3670")
    r_, nm, k_, Pf, _, ult = worst["foot"]
    return {"cases": list(EC["cases"].keys()), "suspended": {"m": es["m"], "cg": es["cg"].tolist(), "Ip": es["Ip"]},
            "M_gyro_yaw": EC["M_gyro_yaw"], "para_n_ult": EC["para_n"], "worst": {k: [v[0], v[1]] for k, v in worst.items()},
            "t_land": t_land, "feet": feet.tolist(),
            "feet_ult": {int(k): {"rx": list(v["rx"]), "ryz": list(v["ryz"])} for k, v in feet_ult.items()},
            "case_tr": {k: v[3] for k, v in EC["cases"].items()}}


# =====================================================================================================================
# 6b. frames and bulkheads at concentrated loads (fix round 1, S1-06)
# =====================================================================================================================
def firewall_edge_distance(c: Ctx, y: float, z: float) -> float:
    """Distance from (y, z) on the firewall to its supported edge (fuselage contour at the firewall minus the frame
    inset): the smaller of the horizontal and the vertical distance to the superelliptic section."""
    x = station_x(c, "FS3670")
    a, bt, bb, zc, nt, nb = (float(v[0]) for v in c.af.sec(x))
    inset = float([s_ for s_ in c.S["layout"]["stations"] if s_["id"] == "FS3670"][0]["inset"])
    b, n = (bt, nt) if z >= zc else (bb, nb)
    t_ = min(abs(z - zc) / b, 1.0)
    y_e = a * (1.0 - t_ ** n) ** (1.0 / n)
    s_ = min(abs(y) / a, 1.0)
    z_e = (zc + bt * (1.0 - s_ ** nt) ** (1.0 / nt)) if z >= zc else (zc - bb * (1.0 - s_ ** nb) ** (1.0 / nb))
    return max(min(y_e - abs(y), abs(z_e - z)) - inset, 0.01)


def firewall_cut_fraction(c: Ctx, y0: float, z0: float, r: float) -> float:
    """Share of the circle of radius r about (y0, z0) on the firewall that lies inside a declared FS3670 cut-out
    (layout.stations FS3670 cutouts, mirrored ones on both sides): the interrupted part of a core-shear perimeter."""
    st = [s_ for s_ in c.S["layout"]["stations"] if s_["id"] == "FS3670"][0]
    th = np.linspace(0.0, 2.0 * math.pi, 1441)[:-1]
    Y, Zc = y0 + r * np.cos(th), z0 + r * np.sin(th)
    inside = np.zeros(len(th), bool)
    for cu in st.get("cutouts", []):
        for sg in ((1.0, -1.0) if cu.get("mirror") else (1.0,)):
            ya, yb = sorted((sg * float(cu["y"][0]), sg * float(cu["y"][1])))
            inside |= (Y >= ya) & (Y <= yb) & (Zc >= float(cu["z"][0])) & (Zc <= float(cu["z"][1]))
    return float(inside.mean())


def check_frames(c: Ctx, R: Rows, ct: dict, em: dict) -> dict:
    """Frames and bulkheads where concentrated loads enter (fix round 1, S1-06): FS-GEAR / FS-RS at the gear-beam ends
    (web shear at the end clips, clip bolts), the well-roof deck under the gear side load, FS-MS / FS-RS outside the CT
    box (body inertia + wing torsion couple at the side-of-body attachments: web shear), the firewall at the lower
    engine-mount feet (out-of-plane foot load: core shear at the land edge, land and face bending; Timoshenko simply
    supported circular plates, conservative), the main-gear up-lock fitting (potted inserts in the well roof) and the
    aft-keel reaction at FS3738 (requirement row)."""
    S = c.S
    out = {}
    GD = c.D["gear"]
    GL = ground_loads(c)
    LG = S["landing_gear"]
    qi = qi_design_values(c)
    pp = ply_props(c, "cfrp_pw_mtm45_as4")
    t_land = int(c.D["body"]["frame_land_plies"]) * pp["t"]
    rib = skin_faces(c, "rib_panel")
    fcomp = total_factor(c, comp=True)
    ffc = total_factor(c, fit=True, comp=True)
    gl_txt = (f"CS-LUAS App. H: n_j {GL['nj']:.2f}; per main leg P_v {GL['Pv_leg']:.0f} N (limit), all main-gear "
              "cases (governing shown)")
    gl_tr = f"CS-LUAS Ek H: n_j {GL['nj']:.2f}; ana bacak başına P_v {GL['Pv_leg']:.0f} N (limit), bütün ana takım durumları"
    # gear beam ends
    gb = [m_ for m_ in S["layout"]["chassis"]["members"] if m_["id"] == "M-GEARBEAM"][0]
    x0, x1 = gb["box"][0][0], gb["box"][1][0]
    hb = gb["box"][1][2] - gb["box"][0][2]
    xt = float(LG["main"]["trunnion"][0])
    Pz = max(abs(v[0]) for v in GL["main_cases"].values())
    Py = max(abs(v[2]) for v in GL["main_cases"].values())
    clip = GD["beam_end_clip"]
    nb, db = int(clip["bolts"]), float(clip["bolt_d_m"])
    h_cl = hb - 0.004
    tr_m = np.asarray(LG["main"]["trunnion"], float)
    ax_m = np.asarray(LG["main"]["axle_static"], float)
    r_m = ax_m + np.array([0.0, 0.0, -(float(LG["tyre"]["diameter"]) / 2 - float(LG["tyre"]["static_deflection"]))]) - tr_m
    for tag, sid, frac in (("GEAR", "FS-GEAR", (xt - x0) / (x1 - x0)), ("RS", "FS-RS", (x1 - xt) / (x1 - x0))):
        # fix round 2 (VS2-02): end reaction = vertical leg load share +- drag couple M_y / L, worst main-gear case
        Rg = max(abs(Pz_ * frac + (1.0 if tag == "GEAR" else -1.0) * float(np.cross(r_m, [Px_, Py_, Pz_])[1]) /
                     (x1 - x0)) for Pz_, Px_, Py_ in GL["main_cases"].values())
        q = Rg / h_cl
        fpf = ST.first_ply_failure(rib["outer"], (0.0, 0.0, q / 2), restrained=True)
        cs, cs_tr = (gl_txt + f"; beam-end reaction {Rg:.0f} N (vertical leg load share {frac * 100:.0f} % + drag "
                     "couple M_y / L)",
                     gl_tr + f"; kiriş ucu tepkisi {Rg:.0f} N (düşey bacak yükü payı % {frac * 100:.0f} + sürükleme "
                     "çifti M_y / L)")
        R.add(f"FR-{tag}-WEB", "frames", f"{sid} web at the gear-beam end clip (h {h_cl * 1000:.0f} mm): shear of the "
              "beam-end reaction, face first-ply failure", f"{sid} gövdesi takım kirişi uç bağlantısında (h "
              f"{h_cl * 1000:.0f} mm): kiriş ucu tepkisinin kesmesi, yüz ilk katman hasarı", cs, cs_tr, q / 1e3,
              fpf["R"] * q / 1e3, "N/mm", fcomp, "CLT (rib_panel faces)", part=sid)
        d_ = rib["c"] + rib["t_out"]
        R.add(f"FR-{tag}-CRIMP", "frames", f"{sid} web at the gear-beam end: shear crimping", f"{sid} gövdesi kiriş "
              "ucunda: kesme kıvrılması", cs, cs_tr, q / 1e3, float(rib["core"]["G"]) * d_ ** 2 / rib["c"] / 1e3,
              "N/mm", fcomp, "N = G_c d^2 / c", part=sid)
        R.add(f"FR-{tag}-BR", "frames", f"gear-beam end clip to {sid}: {nb} x M{db * 1000:.0f} Ti, bearing in the frame "
              f"land ({t_land * 1000:.1f} mm)", f"takım kirişi uç bağlantısı - {sid}: {nb} x M{db * 1000:.0f} Ti, "
              f"çerçeve bandında ezilme ({dec(t_land * 1000, 1)} mm)", cs, cs_tr, Rg / nb, db * t_land * qi["bearing_Pa"],
              "N", ffc, "QI bearing ETW", part=sid)
        out[f"R_{tag}"] = Rg
    # gear side load into the well-roof deck through the 4 trunnion-fitting roof bolts
    fit = [f for f in S["layout"]["chassis"]["fittings"] if f["id"] == "F-TRUNNION"][0]
    rb = [b_ for b_ in fit["bolts"] if b_["group"] == "well roof"]
    fl = skin_faces(c, c.D["fuel_bay"]["floor_layup"])
    t_rl = int(c.D["body"]["frame_land_plies"]) * pp["t"]
    R.add("FR-ROOF-SIDE", "frames", f"well-roof deck M-WELLROOF: gear side load through the {len(rb)} roof bolts "
          f"M{float(rb[0]['d']) * 1000:.0f} (axis z), bearing in the {t_rl * 1000:.1f} mm land", f"kuyu tavanı M-WELLROOF: "
          f"takım yan yükü {len(rb)} tavan cıvatasıyla M{float(rb[0]['d']) * 1000:.0f} (z ekseni), {dec(t_rl * 1000, 1)} mm "
          "bantta ezilme", gl_txt + f"; side load {Py:.0f} N", gl_tr + f"; yan yük {Py:.0f} N", Py / len(rb),
          float(rb[0]["d"]) * t_rl * qi["bearing_Pa"], "N", ffc, f"QI bearing ETW ({fl['label']} deck, solid land)",
          part="M-WELLROOF")
    # FS-MS / FS-RS outside the CT box: body inertia at the side-of-body attachments
    sec = wing_section(c, Y_SOB - 0.005, 20)
    for tag, Rf, h_, sid in (("MS", ct["R_FS_MS"], sec["h_main"], "FS-MS"), ("RS", ct["R_FS_RS"], sec["h_rear"], "FS-RS")):
        nd = int(c.D["wing"]["ct_box"]["web_doubler_plies_per_face"][sid])
        face = ST.laminate_abd([(pl[0], pl[1], pl[2]) for pl in rib["outer"]["plies"]] + [(pp, 45.0, pp["t"])] * nd)
        q = (abs(Rf) / 2 + ct["F_T"]) / h_
        fpf = ST.first_ply_failure(face, (0.0, 0.0, q / 2), restrained=True)
        cs = (f"wing design case: body inertia reaction {Rf:.0f} N on {sid} (limit, CT-ATT), half per side + wing torsion "
              f"couple {ct['F_T']:.0f} N, over the box depth {h_ * 1000:.0f} mm at the side-of-body rib")
        cs_tr = (f"kanat tasarım durumu: {sid} üzerinde gövde ataleti tepkisi {Rf:.0f} N (limit, CT-ATT), taraf başına "
                 f"yarısı + kanat burulma çifti {ct['F_T']:.0f} N, gövde yanı kaburgasında {h_ * 1000:.0f} mm kutu derinliği")
        R.add(f"FR-{tag}-RING", "frames", f"{sid} frame web outside the CT box at the side-of-body attachment (rib_panel + "
              f"{nd} +-45 per face): shear of the body inertia, face first-ply failure", f"{sid} çerçeve gövdesi orta "
              f"kutu dışında gövde yanı bağlantısında (rib_panel + yüz başına {nd} kat ±45): gövde ataleti kesmesi, yüz ilk "
              "katman hasarı", cs, cs_tr, q / 1e3, fpf["R"] * q / 1e3, "N/mm", fcomp,
              "CLT; FS-MS / FS-RS are full sandwich bulkheads above the box and 0.15 m wide posts beside the payload-bay "
              "cut-out below it, the body inertia enters at the box ends", part=sid)
    # firewall at the lower engine-mount feet (out-of-plane x reaction)
    FW = c.D["firewall"]
    fland = FW["foot_land"]
    feet = np.asarray(em["feet"], float)
    lower = [k for k in range(len(feet)) if feet[k][2] < 0.2]
    k_l = max(lower, key=lambda k: em["feet_ult"][k]["rx"][0])
    P, nm = em["feet_ult"][k_l]["rx"]
    yf, zf = float(feet[k_l][1]), float(feet[k_l][2])
    bp = c.D["engine_mount"]["backing_plate_m"]
    c_bp = math.sqrt(float(bp[0]) * float(bp[1]) / math.pi)
    r_l = float(fland["land_r_m"])
    t_l = int(fland["land_plies"]) * pp["t"]
    a_s = firewall_edge_distance(c, yf, zf)
    core = mat(c, fland["core"])
    d_s = rib["c"] + 0.5 * (rib["t_out"] + rib["t_in"])
    fu = total_factor(c, ult_only=True, comp=True)
    cs = (f"lower engine-mount foot (y {yf:.2f}, z {zf:.2f}): largest out-of-plane reaction {P:.0f} N (ultimate, case "
          f"'{nm}', E-FOOT), backing plate {float(bp[0]) * 1000:.0f} x {float(bp[1]) * 1000:.0f} mm (c "
          f"{c_bp * 1000:.0f} mm), land r {r_l * 1000:.0f} mm, edge support at {a_s * 1000:.0f} mm")
    cs_tr = (f"alt motor ayağı (y {dec(yf)}, z {dec(zf)}): en büyük düzlem dışı tepki {P:.0f} N (nihai, '{em['case_tr'][nm]}', "
             f"E-FOOT), destek levhası {float(bp[0]) * 1000:.0f} x {float(bp[1]) * 1000:.0f} mm (c {c_bp * 1000:.0f} mm), "
             f"bant r {r_l * 1000:.0f} mm, kenar desteği {a_s * 1000:.0f} mm")
    kpk = float(fland.get("core_shear_peak_factor", 1.0))
    # fix round 2: a firewall cut-out crossing the land-edge circle interrupts the core-shear perimeter
    f_cut = firewall_cut_fraction(c, yf, zf, r_l)
    R.add("FW-FOOT-CORE", "frames", f"firewall sandwich at the lower foot: peak core shear at the land edge "
          f"({core['name']} insert; peak / mean {kpk:.2f}; {f_cut * 100:.1f} % of the edge circle in cut-outs)",
          f"alt ayakta yangın perdesi sandviçi: bant kenarında en büyük çekirdek kesmesi ({core['name']} parçası; "
          f"tepe / ortalama {dec(kpk)}; kesiklerde kalan kenar çemberi payı %{dec(f_cut * 100, 1)})", cs,
          cs_tr, kpk * P / (2 * math.pi * r_l * (1.0 - f_cut) * d_s) / 1e6, float(core["Fsu"]) / 1e6, "MPa", fu,
          "tau_peak = k P / (2 pi r (1 - f_cut) d), k from the plate-shear distribution (fix round 2, VS2-06), f_cut = "
          "share of the land-edge circle inside the FS3670 cut-outs; core minimum value", part="FS3670")
    M_l = ST.plate_central_patch_moment(P, r_l, c_bp)
    R.add("FW-FOOT-LAND", "frames", f"firewall solid land ({int(fland['land_plies'])} plies, {t_l * 1000:.1f} mm) bending "
          "at the backing plate", f"yangın perdesi dolu bandı ({int(fland['land_plies'])} kat, {dec(t_l * 1000, 1)} mm) "
          "destek levhasında eğilme", cs, cs_tr, 6 * M_l / t_l ** 2 / 1e6, qi["OHC_Pa"] / 1e6, "MPa", fu,
          "simply supported circular plate, central patch (Timoshenko sec. 19); QI OHC ETW", part="FS3670")
    M_f = ST.plate_central_patch_moment(P, a_s, r_l)
    R.add("FW-FOOT-FACE", "frames", "firewall sandwich faces at the land edge: plate bending to the edge support",
          "bant kenarında yangın perdesi sandviç yüzleri: kenar desteğine plaka eğilmesi", cs, cs_tr,
          M_f / (rib["t_out"] * d_s) / 1e6, qi["OHC_Pa"] / 1e6, "MPa", fu,
          "simply supported circular plate a = edge distance (Timoshenko sec. 19); QI OHC ETW", part="FS3670")
    upper = [k for k in range(len(feet)) if feet[k][2] >= 0.2]
    k_u = max(upper, key=lambda k: em["feet_ult"][k]["rx"][0])
    Pu, nmu = em["feet_ult"][k_u]["rx"]
    cw = [f for f in S["layout"]["chassis"]["fittings"] if f["id"] == "F-FW-CORNER"][0]
    ds = [b_ for b_ in cw["bolts"] if b_["group"] == "dorsal splice"]
    do = [m_ for m_ in S["layout"]["chassis"]["members"] if m_["id"] == "M-DORSAL"][0]["section"]
    d_ds = float(ds[0]["d"])
    R.add("FW-UPPER-SPLICE", "frames", f"upper foot x-load into the dorsal longeron: corner-fitting splice {len(ds)} x "
          f"M{d_ds * 1000:.0f} Ti, bearing in the longeron ({float(do['t']) * 1000:.1f} mm)", f"üst ayak x yükü sırt uzun "
          f"kirişine: köşe bağlantısı eki {len(ds)} x M{d_ds * 1000:.0f} Ti, uzun kirişte ezilme "
          f"({dec(float(do['t']) * 1000, 1)} mm)", f"upper foot: largest x reaction {Pu:.0f} N (ultimate, '{nmu}')",
          f"üst ayak: en büyük x tepkisi {Pu:.0f} N (nihai, '{em['case_tr'][nmu]}')", Pu / len(ds),
          d_ds * float(do["t"]) * qi["bearing_Pa"], "N", total_factor(c, ult_only=True, fit=True, comp=True),
          "QI bearing ETW", part="YK250-CH-086")
    out.update({"foot_rx_lower": P, "foot_rx_upper": Pu, "firewall_edge_distance": a_s})
    # main-gear up-lock: stowed leg at the equipment load factor, potted inserts in the well roof
    up = [f for f in S["layout"]["chassis"]["fittings"] if f["id"] == "F-UPLOCK"][0]
    tr_ = np.asarray(LG["main"]["trunnion"], float)
    ax = np.asarray(LG["main"]["axle_static"], float)
    m_leg = mass_item(c, "main_gear_legs_wheels_brakes_emas_pair") / 2
    r_cg = 0.7 * float(np.linalg.norm(ax - tr_))
    pu = np.asarray(up["point"], float)
    r_lk = float(math.hypot(pu[1] - tr_[1], pu[2] - tr_[2]))
    n_up = max(abs(c.n_eq_pos), abs(c.n_eq_neg))
    F_lk = m_leg * G * n_up * r_cg / r_lk
    UL = GD["uplock"]
    ub = [b_ for b_ in up["bolts"]]
    Pu_b = np.array([[b_["point"][0], b_["point"][1]] for b_ in ub])
    T_b = ST.bolt_group_tension(Pu_b, F_lk, Mx=F_lk * float(UL["hook_offset_m"]))
    fl_core = mat(c, fl["core"]["key"]) if "key" in fl["core"] else fl["core"]
    P_ins = ST.insert_pullout(float(fl_core["Fsu"]), float(UL["b_p_m"]), fl["c"])
    cs = (f"stowed main leg {m_leg:.2f} kg at the equipment load factor {n_up:.2f} (gust matrix): CG radius "
          f"{r_cg:.3f} m (0.7 x trunnion-axle distance), up-lock radius {r_lk:.3f} m: hook load {F_lk:.0f} N (limit)")
    cs_tr = (f"toplanmış ana bacak {m_leg:.2f} kg, teçhizat yük katsayısı {dec(n_up)} (hamle matrisi): AM yarıçapı "
             f"{dec(r_cg, 3)} m (0,7 x mafsal-dingil mesafesi), yukarı kilit yarıçapı {dec(r_lk, 3)} m: kanca yükü "
             f"{F_lk:.0f} N (limit)")
    R.add("G-UPLOCK", "gear", f"main up-lock fitting F-UPLOCK: {len(ub)} x M{float(ub[0]['d']) * 1000:.0f} into potted "
          f"inserts of the well roof (potting r {float(UL['b_p_m']) * 1000:.0f} mm, core {fl['c'] * 1000:.1f} mm), "
          "insert pull-out (most loaded, hook offset)", f"ana yukarı kilit bağlantısı F-UPLOCK: kuyu tavanının dökme "
          f"insertlerine {len(ub)} x M{float(ub[0]['d']) * 1000:.0f} (dökme r {float(UL['b_p_m']) * 1000:.0f} mm, çekirdek "
          f"{dec(fl['c'] * 1000, 1)} mm), insert sökülmesi (en yüklü, kanca kaçıklığı)", cs, cs_tr, float(T_b.max()),
          P_ins, "N", ffc, "structlib.insert_pullout P = 2 pi b_p c tau_c (core minimum shear, no test correction)",
          part="YK250-CH-072")
    # aft keel support at FS3738 (requirement row: the lower U-ring is not sized for it; open item)
    vb = S["tail"]["surfaces"]["ventral"]["bumper"]["contact_point"]
    P_b = 1.0 * c.m0 * G
    Fx, Fz = P_b / math.sqrt(2), P_b / math.sqrt(2)
    ak = [m_ for m_ in S["layout"]["chassis"]["members"] if m_["id"] == "M-AFTKEEL"][0]
    x_fw = float(ak["box"][0][0])
    x38 = station_x(c, "FS3738")
    z_k = 0.5 * (ak["box"][0][2] + ak["box"][1][2])
    M_fw = Fz * (vb[0] - x_fw) + Fx * (z_k - vb[2])
    R38 = M_fw / (x38 - x_fw)
    cs38 = (f"tail-bumper strike 1.0 x MTOM weight at 45 deg (design choice, limit): keel pinned at the firewall end, "
            f"supported by FS3738: reaction {R38:.0f} N (strike moment {M_fw:.0f} N m / {x38 - x_fw:.4f} m)")
    cs38_tr = (f"kuyruk tamponu çarpması 45°'de 1,0 x MTOM ağırlığı (tasarım kararı, limit): omurga yangın perdesi ucunda "
               f"mafsallı, FS3738'de destekli: tepki {R38:.0f} N (çarpma momenti {M_fw:.0f} N m / {x38 - x_fw:.4f} m)")
    R.info("FR-3738-KEEL", "frames", "aft-keel support at FS3738 under the tail-bumper strike: vertical reaction (keel "
           "pinned at its firewall end, supported by FS3738), carried by the machined 7075 lower segment (FR-3738-*)",
           "kuyruk tamponu çarpmasında FS3738'de arka omurga desteği: düşey tepki (omurga yangın perdesi ucunda "
           "mafsallı, FS3738'de destekli), talaşlı 7075 alt parça taşır (FR-3738-*)", cs38, cs38_tr, R38, "N",
           f"moment of the strike about the keel firewall end {M_fw:.0f} N m / support spacing {x38 - x_fw:.4f} m (fix "
           "round 2, VS2-03: sized by FR-3738-SEG / -SEG-SH / -KEEL-* / -END-*)", part="FS3738")
    SG = c.D["body"]["fs3738_lower_segment"]
    msg = mat(c, SG["material"])
    hs_, bf_, tf_, tw_ = float(SG["depth_m"]), float(SG["flange_w_m"]), float(SG["flange_t_m"]), float(SG["web_t_m"])
    hw_ = hs_ - 2 * tf_
    I_sg = 2 * (bf_ * tf_ ** 3 / 12 + bf_ * tf_ * (hs_ / 2 - tf_ / 2) ** 2) + tw_ * hw_ ** 3 / 12
    Lsg = float(SG["span_y_m"])
    M_sg = R38 * Lsg / 4
    R.add("FR-3738-SEG", "frames", f"FS3738 lower segment (machined 7075 I {hs_ * 1000:.0f} x {bf_ * 1000:.0f} x "
          f"{tf_ * 1000:.1f} / {tw_ * 1000:.1f} mm, span {Lsg:.2f} m between the ring legs): bending under the keel "
          "reaction (simply supported, load at mid-span)", f"FS3738 alt parçası (talaşlı 7075 I {hs_ * 1000:.0f} x "
          f"{bf_ * 1000:.0f} x {dec(tf_ * 1000, 1)} / {dec(tw_ * 1000, 1)} mm, halka bacakları arası {dec(Lsg)} m): omurga "
          "tepkisi altında eğilme (basit mesnetli, yük ortada)", cs38, cs38_tr, M_sg * hs_ / 2 / I_sg / 1e6,
          float(msg["Fty"]) / 1e6, "MPa", total_factor(c), "M = R L / 4; Fty - fix round 2, VS2-03", part="FS3738")
    R.add("FR-3738-SEG-SH", "frames", "FS3738 lower segment web shear at the ring legs", "FS3738 alt parçası gövde "
          "kesmesi (halka bacaklarında)", cs38, cs38_tr, R38 / 2 / (hw_ * tw_) / 1e6, float(msg["Fsu"]) / 1e6, "MPa",
          total_factor(c), "average web shear, Fsu", part="FS3738")
    Rm_ = float(research_value("materials", "fasteners.property_classes.steel_12_9.tensile_Rm_min_Pa"))
    As_ = research_value("materials", "fasteners.property_classes.stress_area.A_s_m2")
    for tg_, nb_, db_, Pq in (("KEEL", int(SG["keel_bolts"]), float(SG["keel_bolt_d_m"]), R38),
                              ("END", int(SG["end_bolts"]), float(SG["end_bolt_d_m"]), R38 / 2)):
        Asb = float(As_[f"M{db_ * 1000:.0f}"])
        R.add(f"FR-3738-{tg_}-BOLTS", "frames", f"FS3738 lower segment: {'aft keel to the segment web' if tg_ == 'KEEL' else 'splice to each ring leg'} "
              f"{nb_} x M{db_ * 1000:.0f} 12.9, single shear", f"FS3738 alt parçası: "
              f"{'arka omurga - parça gövdesi' if tg_ == 'KEEL' else 'her halka bacağına ek'} {nb_} x M{db_ * 1000:.0f} "
              "12.9, tek kesme", cs38, cs38_tr, Pq / nb_, 0.6 * Rm_ * Asb, "N", total_factor(c, fit=True),
              "ISO 898-1 12.9, 0.6 Rm A_s", part="FS3738")
        R.add(f"FR-3738-{tg_}-BR", "frames", f"FS3738 lower segment: bearing of the {tg_.lower()} bolts in the 7075 web "
              f"({tw_ * 1000:.1f} mm)", f"FS3738 alt parçası: {'omurga' if tg_ == 'KEEL' else 'ek'} cıvatalarının 7075 "
              f"gövdede ezilmesi ({dec(tw_ * 1000, 1)} mm)", cs38, cs38_tr, Pq / nb_, db_ * tw_ * float(msg["Fbru"]), "N",
              total_factor(c, fit=True), "MMPDS Fbru (e/D 2)", part="FS3738")
    out.update({"R_FS3738_keel": R38, "uplock_load": F_lk})
    return out


# =====================================================================================================================
# 7. parachute recovery (CS-LUAS.380/382: emergency-only parachute = ultimate-only condition)
# =====================================================================================================================
PARA_SHOCK_N = 13100.0


def check_parachute(c: Ctx, R: Rows) -> dict:
    """Bridle U-lug fittings F-RISER-FWD (FS1810) / F-RISER-AFT (FS-RS) at the two ends of the dorsal spine channel
    M-SPINE (fix round 1, S1-04): 13.1 kN opening shock (GRS 4/240 published value, larger than UAVOS 200 5 g x MTOM)
    on EACH leg alone, directions a 30 deg cone about the static leg direction and vertical .. 60 deg aft (snatch);
    ultimate-only x 1.15 fitting factor (x 2.0 bearing on the rotating shackle, x 1.2 composite on laminates). Load
    split by design: the x-component through 4 x M5 (axis z) into the spine pad, a strut force in the spine and, net, by
    shear into the screwed mission-bay upper skin P-MB-UPPER and the chine longerons; the y/z components through
    2 x M5 (axis x) into a 32-ply solid land of the frame and the frame web."""
    S = c.S
    PD = c.D["parachute"]
    fu = total_factor(c, ult_only=True, fit=True)
    fb = total_factor(c, ult_only=True, bear=True)
    fl = total_factor(c, ult_only=True, fit=True, comp=True)
    al = mat(c, "al_7075_t651_plate")
    stp = mat(c, PD["shackle_material"])
    Rm = float(research_value("materials", "fasteners.property_classes.steel_12_9.tensile_Rm_min_Pa"))
    As = research_value("materials", "fasteners.property_classes.stress_area.A_s_m2")
    qi = qi_design_values(c)
    pp = ply_props(c, "cfrp_pw_mtm45_as4")
    br = S["layout"]["chassis"]["parachute"]["bridle"]
    fits = {f["id"]: f for f in S["layout"]["chassis"]["fittings"]}
    spine = [m_ for m_ in S["layout"]["chassis"]["members"] if m_["id"] == "M-SPINE"][0]
    cg = np.asarray(br["cg_used"], float)
    conf = cg + np.array([0.0, 0.0, float(br["confluence_above_cg"])])
    half = math.radians(30.0)
    dirs = []
    for leg in ("forward_leg", "aft_leg"):
        u0 = conf - np.asarray(br[leg]["point"], float)
        u0 /= np.linalg.norm(u0)
        e1 = np.cross(u0, [0.0, 1.0, 0.0])
        e1 /= np.linalg.norm(e1)
        e2 = np.cross(u0, e1)
        for th in np.linspace(0.0, half, 4):
            for ph in np.radians(np.arange(0.0, 360.0, 30.0)):
                dirs.append((leg, u0 * math.cos(th) + math.sin(th) * (math.cos(ph) * e1 + math.sin(ph) * e2)))
        for th in np.radians(np.linspace(0.0, 60.0, 13)):                 # snatch: vertical .. 60 deg aft
            dirs.append((leg, np.array([math.sin(th), 0.0, math.cos(th)])))
    F = np.array([PARA_SHOCK_N * u for _, u in dirs])
    i_x = int(np.argmax(np.abs(F[:, 0])))
    i_v = int(np.argmax(np.hypot(F[:, 1], F[:, 2])))

    def case(i):
        leg, u = dirs[i]
        ang = math.degrees(math.acos(max(min(u[2], 1.0), -1.0)))
        return (f"parachute opening shock 13.1 kN on one bridle leg alone (ultimate-only, CRASH-004); directions: 30 deg "
                f"cone about the static leg direction (confluence 1.5 m above the CG) and vertical .. 60 deg aft (snatch); "
                f"governing {leg.replace('_', ' ')}, {ang:.0f} deg from vertical: Fx {F[i, 0]:.0f}, Fy {F[i, 1]:.0f}, "
                f"Fz {F[i, 2]:.0f} N",
                f"paraşüt açılma şoku 13,1 kN tek kayış ayağında (yalnız nihai, CRASH-004); yönler: statik ayak yönü "
                f"etrafında 30° koni (birleşme halkası AM'nin 1,5 m üstünde) ve düşeyden 60° geriye (silkme); belirleyici "
                f"{'ön ayak' if leg == 'forward_leg' else 'arka ayak'}, düşeyden {ang:.0f}°: Fx {F[i, 0]:.0f}, "
                f"Fy {F[i, 1]:.0f}, Fz {F[i, 2]:.0f} N")
    ca, ca_tr = case(i_v)
    fr = fits["F-RISER-FWD"]
    lug = fr["lug"]
    Dp, te, el = float(lug["bore"]), float(lug["t_m"]), float(lug["e_m"])
    ne = int(lug.get("ears", 2))
    wl = float(lug.get("w_m", PD["lug_w_m"]))
    gap_e = float(lug.get("gap_m", 0.0))
    pin = fr.get("shackle_pin") or {"d": Dp, "material": PD["shackle_material"]}
    stp = mat(c, pin["material"])
    sp = fr.get("bridle_spool") or {}
    t_in = float(sp.get("length_m", te))
    R.add("P-SHACKLE", "parachute", f"shackle pin d {Dp * 1000:.0f} ({stp['name'].split(' ')[0]}), double shear",
          f"kilit pimi Ø{Dp * 1000:.0f} ({stp['name'].split(' ')[0]}), çift kesme", ca, ca_tr, PARA_SHOCK_N,
          2 * math.pi * Dp ** 2 / 4 * float(stp["Fsu"]), "N", fu, "Fsu of the pin material", part="YK250-CH-112 / -113")
    Mp = ST.pin_bending_moment(PARA_SHOCK_N, te, t_in, gap_e)
    R.add("P-SHACKLE-BEND", "parachute", f"shackle pin d {Dp * 1000:.0f} bending (Melcon-Hoblit: ears {te * 1000:.0f} mm, "
          f"bridle spool {t_in * 1000:.0f} mm, gaps {gap_e * 1000:.1f} mm), elastic", f"kilit pimi Ø{Dp * 1000:.0f} "
          f"eğilmesi (Melcon-Hoblit: kulaklar {te * 1000:.0f} mm, kayış makarası {t_in * 1000:.0f} mm, boşluklar "
          f"{dec(gap_e * 1000, 1)} mm), elastik", ca, ca_tr, ST.pin_bending_stress(Mp, Dp) / 1e6,
          float(stp["Ftu"]) / 1e6, "MPa", fu, "M = P/2 (t_o/2 + g + t_i/4) (structlib.pin_bending_moment), elastic "
          "stress against Ftu (no plastic bending factor credited) - fix round 2, VS2-01", part="YK250-CH-112 / -113")
    if sp:
        spm = mat(c, sp["material"])
        R.add("P-SPOOL-BR", "parachute", f"bridle spool {sp['material'].split('_')[0]} (OD {sp['od_m'] * 1000:.0f}, "
              f"{t_in * 1000:.0f} mm) bearing on the pin", f"kayış makarası (OD {sp['od_m'] * 1000:.0f}, "
              f"{t_in * 1000:.0f} mm) pim üzerinde ezilme", ca, ca_tr, PARA_SHOCK_N,
              Dp * t_in * float(spm["Fbru"]), "N", fb, "Fbru; bearing factor 2.0 (rotating)", part="bridle (UAVOS)")
    k_ed = min(1.0, el / Dp / 2.0)
    R.add("P-LUG-BR", "parachute", f"U-lug ears {ne} x {te * 1000:.0f} mm 7075 bearing (rotating shackle, e/D "
          f"{el / Dp:.2f})", f"U-kulak kulakları {ne} x {te * 1000:.0f} mm 7075 ezilme (dönen kilit, e/D {dec(el / Dp)})",
          ca, ca_tr, PARA_SHOCK_N, ne * Dp * te * float(al["Fbru"]) * k_ed, "N", fb,
          "MMPDS Fbru at e/D 2 x (e/D)/2 below e/D 2 (linear reduction, conservative); bearing factor 2.0",
          part="YK250-CH-112 / -113")
    lg = ST.lug_axial(float(al["Ftu"]), float(al["Ftu"]), w=wl, D=Dp, t=ne * te, e=el)
    R.add("P-LUG", "parachute", f"U-lug (w {wl * 1000:.0f}, e {el * 1000:.1f} = {el / Dp:.2f} D, {ne} x {te * 1000:.0f} "
          "mm) net section / shear-bearing", f"U-kulak (w {wl * 1000:.0f}, e {dec(el * 1000, 1)} = {dec(el / Dp)} D, "
          f"{ne} x {te * 1000:.0f} mm) net kesit / kesme-ezilme", ca, ca_tr, PARA_SHOCK_N, lg["P_allow"], "N", fu,
          "structlib.lug_axial (Bruhn D1 trend; shear-bearing K_br F_tu D t - fix round 2, VS2-09: F_tu, not F_bru)",
          part="YK250-CH-112 / -113")
    # frame path (y, z components)
    fr_b = [b_ for b_ in fr["bolts"] if b_["group"] == "frame"]
    d_f = float(fr_b[0]["d"])
    As_f = float(As[f"M{d_f * 1000:.0f}"])
    Pv = float(np.hypot(F[i_v, 1], F[i_v, 2])) / len(fr_b)
    R.add("P-FRAME-BOLTS", "parachute", f"fitting to the frame: {len(fr_b)} x M{d_f * 1000:.0f} 12.9 (axis x), single "
          "shear of the y/z component", f"bağlantı - çerçeve: {len(fr_b)} x M{d_f * 1000:.0f} 12.9 (x ekseni), y/z "
          "bileşeninin tek kesmesi", ca, ca_tr, Pv, 0.6 * Rm * As_f, "N", fu, "ISO 898-1 12.9, 0.6 Rm A_s",
          part="YK250-CH-112 / -113")
    t_fl = int(PD["frame_land_plies"]) * pp["t"]
    R.add("P-FRAME-BR", "parachute", f"frame land bearing under the {len(fr_b)} frame bolts (solid laminate "
          f"{int(PD['frame_land_plies'])} plies, {t_fl * 1000:.1f} mm)", f"{len(fr_b)} çerçeve cıvatası altında çerçeve "
          f"bandı ezilmesi (dolu lamine {int(PD['frame_land_plies'])} kat, {dec(t_fl * 1000, 1)} mm)", ca, ca_tr, Pv,
          d_f * t_fl * qi["bearing_Pa"], "N", fl, "QI bearing ETW", part="FS1810 / FS-RS")
    rib = skin_faces(c, "rib_panel")
    h_web = 0.15
    Fv = float(np.hypot(F[i_v, 1], F[i_v, 2]))
    q = Fv / 2 / h_web
    fpf = ST.first_ply_failure(rib["outer"], (0.0, 0.0, q / 2), restrained=True)
    R.add("P-FRAME", "parachute", "frame FS1810 / FS-RS web (rib_panel): shear of the y/z component into both frame "
          "sides, face first-ply failure", "FS1810 / FS-RS çerçeve gövdesi (rib_panel): y/z bileşeninin çerçevenin iki "
          "yanına kesmesi, yüz ilk katman hasarı", ca, ca_tr, q / 1e3, fpf["R"] * q / 1e3, "N/mm", fl,
          f"web depth {h_web:.2f} m to the chine longerons; CLT", part="FS1810 / FS-RS")
    # spine path (x component)
    cx, cx_tr = case(i_x)
    Fx = abs(float(F[i_x, 0]))
    fl_b = [b_ for b_ in fr["bolts"] if b_["group"] == "spine floor"]
    d_s = float(fl_b[0]["d"])
    As_s = float(As[f"M{d_s * 1000:.0f}"])
    # rigid fitting (fix round 2, VS2-01): the frame bolts (axis x) take the y / z components in shear; the moment about
    # y of the pin offset from the frame face (Fz a - Fx e_z) is reacted by the floor bolts in tension against the flange
    # heel on the frame face (T_i = M x_i / sum x_j^2), the x component by the floor bolts in shear
    x_face = float(fr_b[0]["point"][0])
    sgn = 1.0 if float(fr["point"][0]) > x_face else -1.0
    xi = np.array([sgn * (float(b_["point"][0]) - x_face) for b_ in fl_b])
    a_x = sgn * (float(fr["point"][0]) - x_face)
    e_z = float(fr["point"][2]) - float(fl_b[0]["point"][2])
    r_best, T_best, S_best, k_best = 1e9, 0.0, 0.0, 0
    for k_, Fk in enumerate(F):
        fx_, fz_ = sgn * float(Fk[0]), float(Fk[2])
        My = fz_ * a_x - fx_ * e_z                          # + lifts the bolts (heel at the frame face)
        T = max(My, 0.0) * xi.max() / float((xi ** 2).sum())
        S_ = abs(fx_) / len(fl_b)
        r_ = 1.0 / math.sqrt((S_ * fu["total"] / (0.6 * Rm * As_s)) ** 2 + (T * fu["total"] / (Rm * As_s)) ** 2)
        if r_ < r_best:
            r_best, T_best, S_best, k_best = r_, T, S_, k_
    cm, cm_tr = case(k_best)
    R.add("P-SPINE-BOLTS", "parachute", f"fitting to the spine floor: {len(fl_b)} x M{d_s * 1000:.0f} 12.9 (axis z): shear "
          f"of the x component + tension of the fitting moment (pin {a_x * 1000:.0f} mm from the frame face, "
          f"{e_z * 1000:.0f} mm above the floor; heel on the frame face), interaction", f"bağlantı - omurga tabanı: "
          f"{len(fl_b)} x M{d_s * 1000:.0f} 12.9 (z ekseni): x bileşeni kesmesi + bağlantı momentinin çekmesi (pim "
          f"çerçeve yüzünden {a_x * 1000:.0f} mm, tabandan {e_z * 1000:.0f} mm; topuk çerçeve yüzünde), etkileşim", cm,
          cm_tr, 1.0, r_best, "load multiplier",
          dict(total_factor(c, ult_only=True), text="ultimate-only x fitting 1.15 (in the multiplier)"),
          "ISO 898-1 12.9; R_s^2 + R_t^2 = 1 (fix round 2, VS2-01: Fz x arm now included)", part="YK250-CH-112 / -113")
    wp = PD["floor_washer_plate"]
    ilss = float(research_value("materials", "composites.laminae.cfrp_pw_ooa_prepreg_as4_193.strength_Pa.ETW.ILSS."
                                             "b_basis"))
    t_pad = int(PD["spine_pad_plies"]) * pp["t"]
    per = 2.0 * (float(wp["w_m"]) + float(wp["l_m"]))
    T_sum = T_best * float(xi.sum()) / float(xi.max())       # all four bolt tensions on the one washer plate
    R.add("P-SPINE-PULL", "parachute", f"floor-bolt pull-through of the spine pad ({int(PD['spine_pad_plies'])} plies, "
          f"{t_pad * 1000:.1f} mm): punching shear round the {wp['w_m'] * 1000:.0f} x {wp['l_m'] * 1000:.0f} mm washer "
          "plate under the four floor bolts", f"taban cıvatalarının omurga takviyesinden sıyrılması "
          f"({int(PD['spine_pad_plies'])} kat, {dec(t_pad * 1000, 1)} mm): dört taban cıvatasının altındaki "
          f"{wp['w_m'] * 1000:.0f} x {wp['l_m'] * 1000:.0f} mm pul levhası çevresinde zımbalama kesmesi", cm, cm_tr, T_sum,
          per * t_pad * ilss, "N", fl,
          "ILSS ETW B-basis (NCAMP PW) as the through-thickness shear estimate until a pull-through test exists - fix "
          "round 2, VS2-01", part="M-SPINE")
    Ss = Fx / len(fl_b)
    t_sp = int(PD["spine_pad_plies"]) * pp["t"]
    R.add("P-SPINE-BR", "parachute", f"spine floor pad ({int(PD['spine_pad_plies'])} plies, {t_sp * 1000:.1f} mm) bearing "
          f"under the {len(fl_b)} floor bolts", f"omurga taban takviyesi ({int(PD['spine_pad_plies'])} kat, "
          f"{dec(t_sp * 1000, 1)} mm) {len(fl_b)} taban cıvatası altında ezilme", cx, cx_tr, Ss,
          d_s * t_sp * qi["bearing_Pa"], "N", fl, "QI bearing ETW (pad >= 40 % +-45, edge 16 mm = 3.2 D)", part="M-SPINE")
    sec = spine["section"]
    w_, h_, t_, fw_ = float(sec["w"]), float(sec["h"]), float(sec["t"]), float(sec["flange_w"])
    els = [(w_ * t_, t_ / 2), (2 * h_ * t_, h_ / 2), (2 * fw_ * t_, h_ - t_ / 2)]   # floor, walls, flanges (z from floor)
    A_sp = sum(a_ for a_, _ in els)
    z_c = sum(a_ * z_ for a_, z_ in els) / A_sp
    I_sp = sum(a_ * (z_ - z_c) ** 2 for a_, z_ in els) + 2 * t_ * h_ ** 3 / 12
    E_q = qi["E_qi_Pa"]
    R.add("P-SPINE-AX", "parachute", f"spine channel M-SPINE ({w_ * 1000:.0f} x {h_ * 1000:.0f} x {t_ * 1000:.1f} mm, "
          f"flanges {fw_ * 1000:.0f} mm): axial stress of the full x component", f"omurga kanalı M-SPINE "
          f"({w_ * 1000:.0f} x {h_ * 1000:.0f} x {dec(t_ * 1000, 1)} mm, flanşlar {fw_ * 1000:.0f} mm): tam x bileşeninin "
          "eksenel gerilmesi", cx, cx_tr, Fx / A_sp / 1e6, qi["OHC_Pa"] / 1e6, "MPa", fl,
          "QI open-hole compression ETW (screw holes along the flanges)", part="M-SPINE")
    xs_ = sorted(station_x(c, sid) for sid in ("FS1810", "FS-FUEL", "FS-MS", "FS-RS"))
    L_col = max(b_ - a_ for a_, b_ in zip(xs_[:-1], xs_[1:]))
    P_cr = math.pi ** 2 * E_q * I_sp / L_col ** 2
    R.add("P-SPINE-COL", "parachute", f"spine channel as a column between frames (L {L_col:.3f} m, pinned, skin "
          "restraint not credited)", f"çerçeveler arasında kolon olarak omurga kanalı (L {dec(L_col, 3)} m, mafsallı, "
          "kaplama desteği sayılmadı)", cx, cx_tr, Fx, P_cr, "N", fl, "Euler, QI modulus", part="M-SPINE")
    sig_cr = 4.0 * math.pi ** 2 * E_q / (12 * (1 - 0.3 ** 2)) * (t_ / h_) ** 2
    R.add("P-SPINE-LOCAL", "parachute", f"spine channel walls ({h_ * 1000:.0f} mm, both edges supported) local buckling "
          "under the axial stress", f"omurga kanalı duvarları ({h_ * 1000:.0f} mm, iki kenar mesnetli) eksenel gerilme "
          "altında yerel burkulma", cx, cx_tr, Fx / A_sp / 1e6, sig_cr / 1e6, "MPa", fl,
          "SS plate k = 4 (Bruhn C5), QI modulus", part="M-SPINE")
    L_sk = station_x(c, "FS-FUEL") - station_x(c, "FS1810")
    pitch = float(PD["skin_screw_pitch_m"])
    n_sc = 2 * int(L_sk / pitch)
    As4 = float(As["M4"])
    R.add("P-SPINE-SCREWS", "parachute", f"spine flanges to P-MB-UPPER: {n_sc} x M4 A2-70 (pitch {pitch * 1000:.0f} mm, "
          f"FS1810 - FS-FUEL), shear of the x component (button head 0.8 x class)", f"omurga flanşları - P-MB-UPPER: "
          f"{n_sc} x M4 A2-70 ({pitch * 1000:.0f} mm aralık, FS1810 - FS-FUEL), x bileşeninin kesmesi (yuvarlak baş 0,8 x "
          "sınıf)", cx, cx_tr, Fx / n_sc, 0.8 * 0.6 * 700e6 * As4, "N", fu, "ISO 3506-1 A2-70", part="M-SPINE / YK250-SH-370")
    R.add("P-SPINE-SCREW-BR", "parachute", "spine flange screws M4: bearing in the 1.6 mm solid edge band of the skin",
          "omurga flanşı vidaları M4: kaplamanın 1,6 mm dolu kenar bandında ezilme", cx, cx_tr, Fx / n_sc,
          0.004 * 0.0016 * qi["bearing_Pa"], "N", fl, "QI bearing ETW", part="YK250-SH-370")
    sh = skin_faces(c, "shell_secondary")
    q_s = Fx / (2 * L_sk)
    fpf = ST.first_ply_failure(sh["outer"], (0.0, 0.0, q_s / 2), restrained=True)
    R.add("P-SPINE-SKIN", "parachute", "mission-bay upper skin P-MB-UPPER (shell_secondary): shear flow of the net x "
          "component from the spine flanges to the chine longerons, face first-ply failure", "görev bölmesi üst kaplaması "
          "P-MB-UPPER (shell_secondary): net x bileşeninin omurga flanşlarından kenar uzun kirişlerine kesme akışı, yüz "
          "ilk katman hasarı", cx, cx_tr, q_s / 1e3, fpf["R"] * q_s / 1e3, "N/mm", fl, "CLT", part="YK250-SH-370")
    return {"n_open_mtom": PARA_SHOCK_N / (c.m0 * G), "n_open_lightest": PARA_SHOCK_N / (c.m_min * G),
            "Fx_max": Fx, "F_vertical_max": Fv, "spine_area_m2": A_sp, "spine_I_m4": I_sp}


# =====================================================================================================================
# 8. fuel bays (CS-LUAS.965(b), 967), 9. turret elevator and doors, 10. transport and handling
# =====================================================================================================================
def fuel_pressures(c: Ctx) -> dict:
    """Design pressures on the structure-supported bladder bays (ultimate): 1.5 x the CS-LUAS.965(b) test pressure
    14 kPa (structure-supported non-metallic tank), 1.5 x the fuel head at the wing design n (full tank, cell height),
    emergency landing 9 g forward x cell length and 6 g down x cell height (ultimate, CS-VLA 561), parachute opening
    at MTOM x cell height (ultimate); FUEL-003 24 kPa (unsupported tanks) reported for information."""
    S = c.S
    std = research_value("standards", "requirements")
    p_test = [r for r in std if r["id"] == "FUEL-004"][0]["value"]["structure_supported_non_metallic_Pa"]
    cells = S["layout"]["fuel_cells"]
    h = max(cl["z"][1] - cl["z"][0] for cl in cells)
    Lc = max(cl["x"][1] - cl["x"][0] for cl in cells)
    rg = FUEL_RHO * G
    cand = {"1.5 x test pressure 14 kPa (CS-LUAS.965(b))": 1.5 * float(p_test),
            "1.5 x head at n_wing (full tank)": 1.5 * rg * c.n_wing * h,
            "emergency 9 g forward x cell length (ult)": rg * 9.0 * Lc,
            "emergency 6 g down x cell height (ult)": rg * 6.0 * h,
            "parachute opening at MTOM x cell height (ult)": rg * PARA_SHOCK_N / (c.m0 * G) * h}
    k = max(cand, key=cand.get)
    return {"p_ult": cand[k], "governing": k, "candidates": cand, "h_cell": h, "L_cell": Lc, "p_test": float(p_test)}


def check_fuel_bays(c: Ctx, R: Rows) -> dict:
    """Bay walls under the design pressure as sandwich strips (conservative long-panel limit, simply supported):
    face stress (first-ply failure of the face, DT strain 2600 µε for sandwich), core shear. Walls: forward fuel-bay
    bulkhead FS-FUEL (span = cell height), forward fuel-cell floor M-FWDDECK (span between the keel beams), aft-cell
    floor = main-well roof M-WELLROOF (span from the gear beam to the central well web M-WELLKEEL)."""
    S = c.S
    FP = fuel_pressures(c)
    p = FP["p_ult"]
    FD = c.D["fuel_bay"]
    mem = {m_["id"]: m_ for m_ in S["layout"]["chassis"]["members"]}
    kb = mem["M-KEEL"]["box"][0][1]
    gb = mem["M-GEARBEAM"]["box"][0][1]
    tw = float(FD["well_web_t_m"])
    y_web = float(FD["well_web_y_m"]) + tw                 # outer face of the keel web (roof support line)
    y_r = float(FD["payload_rails_y_m"])
    walls = [("FS-FUEL bulkhead", "FS-FUEL perdesi", "rib_panel", FP["h_cell"], "YK250-CH-008"),
             ("forward fuel-cell floor M-FWDDECK between the payload rails", "ön yakıt hücresi tabanı M-FWDDECK yük "
              "rayları arasında", "rib_panel", max(2 * y_r, kb - y_r), "YK250-CH-029"),
             ("aft-cell floor / well roof M-WELLROOF", "arka hücre tabanı / kuyu tavanı M-WELLROOF", FD["floor_layup"],
              gb - y_web, "YK250-CH-030")]
    case = f"fuel-bay design pressure {p / 1e3:.1f} kPa ultimate ({FP['governing']})"
    case_tr = f"yakıt bölmesi tasarım basıncı {p / 1e3:.1f} kPa nihai ({FP['governing']})"
    out = {}
    for name, name_tr, lay, b, part in walls:
        sk = skin_faces(c, lay)
        d = sk["c"] + 0.5 * (sk["t_out"] + sk["t_in"])
        st_ = ST.sandwich_strip_pressure(p, b, d, min(sk["t_out"], sk["t_in"]))
        face = sk["outer"] if sk["t_out"] <= sk["t_in"] else sk["inner"]
        tf = face["h"]
        N = st_["M"] / d
        fpf = ST.first_ply_failure(face, (N, 0.0, 0.0), restrained=True)
        one = dict(total_factor(c, ult_only=True, comp=True), text="ultimate x composite 1.2")
        R.add(f"F-FACE-{part[-3:]}", "fuel bays", f"{name} ({sk['label']}), span {b:.3f} m: face first-ply failure",
              f"{name_tr} ({sk['label']}), açıklık {b:.3f} m: yüz ilk katman hasarı", case, case_tr, N / 1e3,
              fpf["R"] * N / 1e3, "N/mm", one, "sandwich strip p b^2/8; CLT B-basis ETW", part=part)
        eps = N / (ST.laminate_engineering(face)["Ex"] * tf)
        R.add(f"F-DT-{part[-3:]}", "fuel bays", f"{name}: face strain (DT, sandwich)", f"{name_tr}: yüz birim şekil "
              "değiştirmesi (sandviç, hasar toleransı)", case, case_tr, eps * 1e6, c.dt["sand_comp"] * 1e6, "µε",
              dict(total_factor(c, ult_only=True), text="ultimate"), "STANAG UL13.1.2", part=part)
        Fs = float(sk["core"].get("Fsu", sk["core"].get("Fsu_W", 0.0)))
        R.add(f"F-CORE-{part[-3:]}", "fuel bays", f"{name}: core shear ({sk['core_key']})", f"{name_tr}: çekirdek kesmesi "
              f"({sk['core_key']})", case, case_tr, st_["tau_core"] / 1e6, Fs / 1e6, "MPa", one,
              "tau = (p b / 2) / d; ROHACELL minimum values", part=part)
        out[part] = {"span": b, "M": st_["M"], "tau": st_["tau_core"]}
    # payload rails as deck stiffeners: beams between FS-FUEL and the main-spar frame
    pr = FD["payload_rail"]
    fw, ft, sh, stt = pr["flange_w_m"], pr["flange_t_m"], pr["stem_h_m"], pr["stem_t_m"]
    A1, A2 = fw * ft, sh * stt
    z1, z2 = ft / 2, ft + sh / 2
    zc = (A1 * z1 + A2 * z2) / (A1 + A2)
    I_r = fw * ft ** 3 / 12 + A1 * (z1 - zc) ** 2 + stt * sh ** 3 / 12 + A2 * (z2 - zc) ** 2
    L_r = station_x(c, "FS-MS") - station_x(c, "FS-FUEL")
    w_r = p * 0.5 * (2 * y_r + (kb - y_r))
    M_r = w_r * L_r ** 2 / 8
    m6 = mat(c, pr["material"])
    R.add("F-PAYRAIL", "fuel bays", "payload-tray rail (6061-T6 T 20 x 20 x 2.5) as forward-deck stiffener, bending",
          "faydalı yük rayı (6061-T6 T 20 x 20 x 2,5) ön güverte takviyesi olarak, eğilme", case, case_tr,
          M_r * max(zc, ft + sh - zc) / I_r / 1e6, float(m6["Ftu"]) / 1e6, "MPa",
          dict(total_factor(c, ult_only=True, fit=True), text="ultimate x fitting 1.15"),
          f"simply supported over {L_r:.3f} m; MMPDS 6061-T6 Ftu", part="TR-PAYLOAD")
    # keel webs: reaction of the well-roof half (strip span b, half its load) as a beam between the well forward wall
    # and FS-GEAR (layout M-WELLKEEL length)
    span_w = mem["M-WELLKEEL"]["box"][1][0] - mem["M-WELLKEEL"]["box"][0][0]
    w_line = p * (gb - y_web) / 2
    Mw = w_line * span_w ** 2 / 8
    hw = float(FD["well_web_height_m"])
    sig = 6 * Mw / (tw * hw ** 2)
    one = dict(total_factor(c, ult_only=True, comp=True), text="ultimate x composite 1.2")
    R.add("F-WELLWEB", "fuel bays", f"well keel web M-WELLKEEL (each of two, solid PW {tw * 1000:.1f} mm, {hw * 1000:.0f} mm "
          "deep) bending under the well-roof reaction", f"kuyu omurga gövdesi M-WELLKEEL (iki gövdeden her biri, dolu PW "
          f"{dec(tw * 1000, 1)} mm, {hw * 1000:.0f} mm yükseklik) kuyu tavanı tepkisiyle eğilme", case, case_tr, sig / 1e6,
          qi_design_values(c)["OHC_Pa"] / 1e6, "MPa", one, f"beam over {span_w:.3f} m (well forward wall to FS-GEAR); "
          "OHC (hinge-bracket holes)",
          part="M-WELLKEEL")
    qi = qi_design_values(c)
    s_cr = ST.plate_compression_buckling(qi["E_qi_Pa"], tw, hw, 0.3, k=23.9)
    R.add("F-WELLWEB-BUCK", "fuel bays", "well keel web: plate buckling under in-plane bending (k 23.9, SS edges)",
          "kuyu omurga gövdesi: düzlem içi eğilmede levha burkulması (k 23,9, basit mesnet)", case, case_tr, sig / 1e6,
          s_cr / 1e6, "MPa", one, "Bruhn C5 / Timoshenko: k_b = 23.9 for pure bending; QI modulus (materials.yaml)",
          part="M-WELLKEEL")
    return {"pressure": FP, "walls": out, "well_web_moment": Mw}


def check_turret(c: Ctx, R: Rows) -> dict:
    """Turret elevator (E180 growth turret on the carriage): equipment retention load factors (ultimate) - down: max(1.5
    x flight n of the lightest case, 6 g emergency, parachute opening at the lightest mass), forward: max(9 g,
    parachute x sin 60), side 1.5 x 1.47; ball-screw axial load (requirement), roof bearing block through-bolts, rail
    screws; sliding bay doors under suction at VD with the rail friction drive force vs the DA 22."""
    S = c.S
    m_t = float(S["payload"]["growth_turret_mass_kg"]) + 0.35
    n_dn = max(1.5 * c.n_eq_pos, 6.0, PARA_SHOCK_N / (c.m_min * G))
    n_fw = max(9.0, PARA_SHOCK_N / (c.m_min * G) * math.sin(math.radians(60.0)))
    n_sd = max(1.5 * 1.47, 1.5)
    F_dn, F_fw, F_sd = m_t * G * n_dn, m_t * G * n_fw, m_t * G * n_sd
    case = (f"E180 growth turret + carriage {m_t:.2f} kg; ultimate n down {n_dn:.2f}, forward {n_fw:.2f}, side "
            f"{n_sd:.2f}")
    case_tr = (f"E180 büyüme tareti + taşıyıcı {m_t:.2f} kg; nihai n aşağı {n_dn:.2f}, ileri {n_fw:.2f}, yan {n_sd:.2f}")
    R.info("TU-SCREW", "turret", "ball screw d 8 x 2 and nut: required static axial rating (ultimate)",
           "bilyalı vida d 8 x 2 ve somun: gerekli statik eksenel yük sayısı (nihai)", case, case_tr, F_dn * c.f["fit"],
           "N", "procurement requirement (catalogue rating not in the research data)", part="EQ-ELEVATOR")
    R.info("TU-RAIL", "turret", "miniature profile rails (2 carriages each): required static rating per carriage "
           "(forward + side, ultimate)", "minyatür profil raylar (her birinde 2 taşıyıcı): taşıyıcı başına gerekli statik "
           "yük sayısı (ileri + yan, nihai)", case, case_tr, math.hypot(F_fw, F_sd) / 2 * c.f["fit"], "N",
           "procurement requirement", part="YK250-PL-828")
    a2 = 0.6 * 700e6 * float(research_value("materials", "fasteners.property_classes.stress_area.A_s_m2")["M4"])
    R.add("TU-ROOF", "turret", "roof bearing block: 4 x M4 A2-70 through-bolts with a 2 mm 7075 backing plate, tension "
          "(hanging load)", "tavan yatak bloğu: 4 x M4 A2-70 geçme cıvata, 2 mm 7075 karşı plaka, çekme (asılı yük)",
          case, case_tr, F_dn / 4, 700e6 * 8.78e-6, "N", total_factor(c, ult_only=True, fit=True), "ISO 3506-1 A2-70 Rm A_s",
          part="M-TURRETROOF")
    n_scr = int(c.D["turret"]["rail_screws_per_rail_end"])
    a3 = 0.6 * 700e6 * float(research_value("materials", "fasteners.property_classes.stress_area.A_s_m2")["M3"])
    R.add("TU-RAILSCR", "turret", f"rail end screws {n_scr} x M3 A2-70 per rail end (2 rails, 2 ends), shear",
          f"ray ucu vidaları ray ucu başına {n_scr} x M3 A2-70 (2 ray, 2 uç), kesme", case, case_tr,
          math.hypot(F_fw, F_sd) / (2 * 2 * n_scr), a3, "N", total_factor(c, ult_only=True, fit=True),
          "0.6 Rm A_s", part="YK250-PL-828")
    # sliding doors: suction at VD, rail friction (mu estimate) -> pinion torque vs DA 22 rated torque
    q_D = c.q(c.VD)
    A_door = float(c.D["turret"]["door_area_each_m2"])
    F_p = 1.0 * q_D * A_door
    mu = 0.3
    r_pin = 0.010
    T_req = mu * F_p * r_pin
    items = research_value("components", "categories.control_surface_actuators.items")
    da22 = [i for i in items if i["id"] == "volz_da22_28v"][0]
    R.add("TU-DOORDRIVE", "turret", "sliding bay door drive: pinion torque to move the door under suction (rail friction "
          f"mu {mu} estimate, pinion r {r_pin * 1000:.0f} mm) vs DA 22 rated torque", "kayar bölme kapağı tahriki: emme "
          f"altında kapağı yürütme pinyon torku (ray sürtünmesi mu {mu} tahmin, pinyon r {r_pin * 1000:.0f} mm) / DA 22 anma torku",
          f"closed door suction |Cp| 1.0 x q(VD) {q_D:.0f} Pa on {A_door:.4f} m2 (bound)",
          f"kapalı kapak emmesi |Cp| 1,0 x q(VD) {q_D:.0f} Pa, {A_door:.4f} m2 (üst sınır)", T_req,
          float(da22["torque_rated_Nm"]["value"]), "N m", dict(total_factor(c), text="FoS 1.5 (functional)"),
          "components.yaml volz_da22_28v rated torque", part="EQ-TDOORACT")
    R.add("TU-DOORRAIL", "turret", "sliding door rail lip (6061-T6 rail, 2 rails x 0.12 m), door retention under "
          "suction, shear of a 1 mm lip", "kayar kapak ray dudağı (6061-T6 ray, 2 ray x 0,12 m), emmede kapak tutma, 1 mm "
          "dudak kesmesi",
          f"closed door suction at VD (limit)", "VD'de kapalı kapak emmesi (limit)", F_p,
          2 * 0.12 * 0.001 * float(mat(c, "al_6061_t6_sheet")["Fsu"]), "N", total_factor(c, fit=True),
          "6061-T6 Fsu (rail lip, estimate geometry)", part="YK250-PL-826")
    return {"m": m_t, "n_down": n_dn, "n_fwd": n_fw, "F_down": F_dn, "door_drive_torque": T_req}


def check_transport(c: Ctx, R: Rows) -> dict:
    """Transport in the cradle (centre body empty, without the outer panels, stabilators and propeller) on padded
    saddles under the FS1810 and FS-GEAR lower frame lands; outer panel on its leading edge in a padded rack. Load
    factors: structures.sizing.transport (design choice, stated)."""
    S = c.S
    TD = c.D["transport"]
    nv = float(TD["factors_limit"]["vertical"])
    m_empty = float(S["mass"]["empty_kg"])
    m_wing = mass_item(c, "wing_structure_pair") + mass_item(c, "control_surfaces_ailerons_pair") + \
        mass_item(c, "control_surfaces_flaps_pair") + mass_item(c, "actuators_ailerons_2x_DA26") + \
        mass_item(c, "actuators_flaps_2x_DA30")
    m_op = 0.80 * m_wing / 2
    m_c = m_empty - 2 * m_op - mass_item(c, "stabilators_pair") - mass_item(c, "propeller")
    its = [i for i in design_case_items(c, "zero_fuel_design_payload") if i[4] not in (
        "propeller", "stabilators_pair") and not i[4].startswith("research") and i[4] != "fuel"]
    xcg = sum(i[0] * i[1] for i in its) / sum(i[0] for i in its)
    x1, x2 = station_x(c, "FS1810"), station_x(c, "FS-GEAR")
    W = m_c * G
    R2 = W * (xcg - x1) / (x2 - x1)
    R1 = W - R2
    Rmax = max(R1, R2)
    pad = (0.20, 0.05)
    p_pad = nv * Rmax / (pad[0] * pad[1])
    core = mat(c, "core_rohacell_51wf")
    case = (f"transport cradle, centre body {m_c:.1f} kg (empty, without outer panels, stabilators, propeller), CG x "
            f"{xcg:.2f}: saddle reactions {R1:.0f} / {R2:.0f} N at 1 g; {nv:g} g vertical (design choice)")
    case_tr = (f"taşıma kızağı, orta gövde {m_c:.1f} kg (boş, dış paneller, stabilatörler ve pervane yok), AM x {xcg:.2f}: "
               f"kızak tepkileri 1 g'de {R1:.0f} / {R2:.0f} N; {nv:g} g düşey (tasarım kararı)")
    R.add("TR-PAD", "transport", f"saddle pad {pad[0] * 1000:.0f} x {pad[1] * 1000:.0f} mm on the lower skin over the frame "
          "land: core crushing (ROHACELL 51 WF) where the pad overhangs the solid edge band",
          f"kızak yastığı {pad[0] * 1000:.0f} x {pad[1] * 1000:.0f} mm, çerçeve bandı üzerindeki alt kaplamada: çekirdek "
          "ezilmesi (ROHACELL 51 WF), yastığın dolu kenar bandı dışına taştığı yerde", case, case_tr, p_pad / 1e6,
          float(core["Fcu"]) / 1e6, "MPa", total_factor(c, comp=True), "ROHACELL 51 WF minimum Fcu", part="FS1810 / FS-GEAR")
    rib = skin_faces(c, "rib_panel")
    t_f = rib["t_out"] + rib["t_in"]
    sig = nv * Rmax / (pad[0] * t_f)
    R.add("TR-FRAME", "transport", "frame web edge compression over the saddle (rib_panel faces)", "kızak üzerinde çerçeve "
          "gövdesi kenar basısı (rib_panel yüzleri)", case, case_tr, sig / 1e6, qi_design_values(c)["OHC_Pa"] / 1e6, "MPa",
          total_factor(c, comp=True), "OHC (edge band with fastener holes)", part="FS1810 / FS-GEAR")
    F_le = nv * m_op * G / 2
    R.add("TR-PANEL-LE", "transport", "outer panel on its leading edge in the padded rack (2 pads 0.10 x 0.04 m): LE "
          "solid laminate bearing", "dış panel hücum kenarı üzerinde yastıklı rafta (2 yastık 0,10 x 0,04 m): hücum kenarı "
          "dolu lamine ezilmesi", f"outer panel {m_op:.1f} kg, {nv:g} g vertical (design choice)",
          f"dış panel {m_op:.1f} kg, {nv:g} g düşey (tasarım kararı)", F_le / (0.10 * 0.04) / 1e6,
          float(core["Fcu"]) / 1e6, "MPa", total_factor(c, comp=True),
          "conservative: pad pressure against the core crushing strength of the adjacent sandwich", part="wing module")
    return {"m_centre_body": m_c, "x_cg": xcg, "R_FS1810": R1, "R_FSGEAR": R2, "m_outer_panel": m_op}


# =====================================================================================================================
# 11. body (chassis longerons, torsion cell of the fixed skins, skin fasteners), equipment retention
# =====================================================================================================================
def body_section(c: Ctx, x: float) -> dict:
    """Body section at station x (superelliptic halves of the fuselage OML): enclosed area, perimeter, centroid z."""
    a, bt, bb, zc, nt, nb = (float(v[0]) for v in c.af.sec(x))
    th = np.linspace(0.0, math.pi / 2, 200)

    def quad(b, n):
        yy = a * np.cos(th) ** (2.0 / n)
        zz = b * np.sin(th) ** (2.0 / n)
        A = float(np.trapz(zz[::-1], yy[::-1]))
        P = float(np.sum(np.hypot(np.diff(yy), np.diff(zz))))
        return A, P, yy, zz
    At, Pt, _, _ = quad(bt, nt)
    Ab, Pb, _, _ = quad(bb, nb)
    return {"x": x, "half_width": a, "zc": zc, "A": 2 * (At + Ab), "P": 2 * (Pt + Pb), "top": zc + bt, "bot": zc - bb}


def body_mass_points(c: Ctx, case_name: str = "mtow_design_payload_turret_retracted") -> list:
    """(m, x, z, name) of the loading case with the distributed items split into their layout components
    (layout.mass_placement fractions) and the body skin split along x by the section perimeter."""
    S = c.S
    mp = S["layout"]["mass_placement"]
    pts = []
    for m, x, y, z, nm in design_case_items(c, case_name):
        if nm in mp and mp[nm].get("components"):
            comps = mp[nm]["components"]
            fs = sum(float(k["fraction"]) for k in comps)
            for k in comps:
                pts.append((m * float(k["fraction"]) / fs, float(k["at"][0]), float(k["at"][2]), nm))
        elif nm == "body_skin_sandwich":
            xs = np.linspace(0.02, float(S["fuselage"]["length"]) - 0.02, 60)
            P = np.array([body_section(c, xx)["P"] for xx in xs])
            for xx, w in zip(xs, P / P.sum()):
                pts.append((m * w, float(xx), 0.0, nm))
        else:
            pts.append((m, x, z, nm))
    return pts


def member_path_z(c: Ctx, mid: str, x: float) -> tuple:
    m_ = [m for m in c.S["layout"]["chassis"]["members"] if m["id"] == mid][0]
    for path in m_["paths"]:
        P = np.asarray(path, float)
        if P[0, 0] <= x <= P[-1, 0]:
            return float(np.interp(x, P[:, 0], P[:, 1])), float(np.interp(x, P[:, 0], P[:, 2]))
    return None


def check_body(c: Ctx, R: Rows, tail: dict) -> dict:
    """Body bending on the chassis longerons (the fixed skins are counted as shear webs only, layout.shell.rules):
    aft body at FS-GEAR (chine + dorsal longerons; inertia of the aft items at the equipment n + the full stabilator
    load of both panels, same sense), forward body at FS-FUEL (chine longerons + mission-bay floor; equipment n both
    signs); aft-body torsion (both fins at their side load + 28 % unsymmetric stabilator load) and vertical shear in the
    fixed skins (shell_secondary sandwich): skin first-ply failure, shear buckling, M4 screw rows (button head
    0.8 x class value) and bearing in the 1.6 mm edge band."""
    S = c.S
    pts = body_mass_points(c)
    ud = ply_props(c, "cfrp_ud_mtm45_as4")
    pw = ply_props(c, "cfrp_pw_mtm45_as4")
    mem = {m_["id"]: m_ for m_ in S["layout"]["chassis"]["members"]}
    ch = mem["M-CHINE"]["section"]
    EA_ch = ud["E1"] * ch["w"] * ch["t"] + pw["E1"] * ch["h"] * ch["t"]
    do = mem["M-DORSAL"]["section"]
    EA_do = ud["E1"] * do["w"] * do["t"] + pw["E1"] * 2 * do["h"] * do["t"]
    n = c.n_eq_pos
    out = {}
    # aft body at FS-GEAR (just aft of the frame)
    x_a = station_x(c, "FS-GEAR") + 0.005
    M_a = sum(m * n * G * (x - x_a) for m, x, z, nm in pts if x > x_a)
    V_a = sum(m * n * G for m, x, z, nm in pts if x > x_a)
    x_st = float(S["tail"]["surfaces"]["stabilator"]["pivot"][0])
    M_a += 2 * tail["N_stab"] * (x_st - x_a)
    V_a += 2 * tail["N_stab"]
    yc, zc_ = member_path_z(c, "M-CHINE", x_a)
    yd, zd = member_path_z(c, "M-DORSAL", x_a)
    zna = (2 * EA_ch * zc_ + 2 * EA_do * zd) / (2 * EA_ch + 2 * EA_do)
    EI = 2 * EA_ch * (zc_ - zna) ** 2 + 2 * EA_do * (zd - zna) ** 2
    e_do = M_a * (zd - zna) / EI
    e_ch = M_a * (zna - zc_) / EI
    case = (f"aft body at FS-GEAR: inertia of the aft items at n {n:.2f} (equipment n) + both stabilator panels at "
            f"{tail['N_stab']:.0f} N (same sense): M {M_a:.0f} N m, V {V_a:.0f} N (limit)")
    case_tr = (f"FS-GEAR'de arka gövde: arka kalemlerin n {n:.2f} (teçhizat n'i) ataleti + iki stabilatör paneli "
               f"{tail['N_stab']:.0f} N (aynı yön): M {M_a:.0f} N m, V {V_a:.0f} N (limit)")
    R.add("B-DORSAL", "body", f"dorsal longeron (hat {do['w'] * 1000:.0f} x {do['h'] * 1000:.0f} x {do['t'] * 1000:.1f}, "
          "UD cap + PW) at FS-GEAR, strain (DT, <= 2 mm)", f"sırt uzun kirişi (şapka {do['w'] * 1000:.0f} x "
          f"{do['h'] * 1000:.0f} x {do['t'] * 1000:.1f}, UD başlık + PW) FS-GEAR'de, birim şekil değiştirme (<= 2 mm)",
          case, case_tr, abs(e_do) * 1e6, c.dt["sand_comp"] * 1e6, "µε", total_factor(c),
          "STANAG UL13.1.2; longerons only (skins = shear webs)", part="YK250-CH-032")
    R.add("B-CHINE-AFT", "body", f"chine longeron (J {ch['w'] * 1000:.0f} x {ch['h'] * 1000:.0f} x {ch['t'] * 1000:.1f}) at "
          "FS-GEAR, strain (DT, > 2 mm)", f"kenar uzun kirişi (J {ch['w'] * 1000:.0f} x {ch['h'] * 1000:.0f} x "
          f"{ch['t'] * 1000:.1f}) FS-GEAR'de, birim şekil değiştirme (> 2 mm)", case, case_tr, abs(e_ch) * 1e6,
          c.dt["cap_comp"] * 1e6, "µε", total_factor(c), "STANAG UL13.1.2", part="YK250-CH-020")
    # dorsal hat column buckling between FS-GEAR and FS3480 (pinned at the frames, hat alone)
    t, w_, h_ = do["t"], do["w"], do["h"]
    els = [(w_ * t, h_ - t / 2), (2 * h_ * t, h_ / 2), (2 * 0.010 * t, t / 2)]
    A_h = sum(a for a, _ in els)
    zc_h = sum(a * z for a, z in els) / A_h
    I_h = sum(a * (z - zc_h) ** 2 for a, z in els) + 2 * t * h_ ** 3 / 12
    L_ = station_x(c, "FS3480") - station_x(c, "FS-GEAR")
    P_cr = math.pi ** 2 * pw["E1"] * I_h / L_ ** 2
    F_do = abs(e_do) * EA_do
    R.add("B-DORSAL-COL", "body", f"dorsal longeron column between FS-GEAR and FS3480 (L {L_:.3f} m, hat alone, PW "
          "modulus, pinned)", f"FS-GEAR ile FS3480 arasında sırt uzun kirişi kolonu (L {L_:.3f} m, yalnız şapka, PW "
          "modülü, mafsallı)", case, case_tr, F_do, P_cr, "N", total_factor(c, comp=True),
          "Euler; the fastened skin (nutplates 25-32 mm) is not credited", part="YK250-CH-032")
    # forward body at FS-FUEL: chine longerons + mission-bay floor, both signs
    x_f = station_x(c, "FS-FUEL") - 0.005
    M_f = sum(m * G * (x_f - x) for m, x, z, nm in pts if x < x_f)
    mf = mem["M-MIDFLOOR"]
    rib = skin_faces(c, "rib_panel")
    w_cut = (mf["cutout"]["y"][1] - mf["cutout"]["y"][0]) if mf.get("cutout") else 0.0
    w_fl = mf["box"][1][1] - mf["box"][0][1] - w_cut          # fixed outer strips only (the removable tray not credited)
    EA_fl = (rib["E_out"] * rib["t_out"] + rib["E_in"] * rib["t_in"]) * w_fl
    z_fl = 0.5 * (mf["box"][0][2] + mf["box"][1][2])
    _, zc_f = member_path_z(c, "M-CHINE", x_f)
    zna = (2 * EA_ch * zc_f + EA_fl * z_fl) / (2 * EA_ch + EA_fl)
    EI_f = 2 * EA_ch * (zc_f - zna) ** 2 + EA_fl * (z_fl - zna) ** 2
    worst_f = None
    for nn in (c.n_eq_pos, c.n_eq_neg):
        M = M_f * nn
        e_ch = abs(M * (zc_f - zna) / EI_f)
        e_fl = abs(M * (z_fl - zna) / EI_f)
        if worst_f is None or e_fl > worst_f[2]:
            worst_f = (nn, e_ch, e_fl, M)
    nn, e_ch, e_fl, M = worst_f
    case_f = (f"forward body at FS-FUEL: inertia of the forward items, n {nn:.2f} (equipment n, governing sign): "
              f"M {M:.0f} N m (limit)")
    case_ftr = (f"FS-FUEL'de ön gövde: ön kalemlerin ataleti, n {nn:.2f} (teçhizat n'i, belirleyici işaret): M {M:.0f} N m "
                "(limit)")
    R.add("B-CHINE-FWD", "body", "chine longeron at FS-FUEL, strain (DT, > 2 mm)", "FS-FUEL'de kenar uzun kirişi, birim "
          "şekil değiştirme (> 2 mm)", case_f, case_ftr, e_ch * 1e6, c.dt["cap_comp"] * 1e6, "µε", total_factor(c),
          "STANAG UL13.1.2", part="YK250-CH-020")
    R.add("B-MIDFLOOR", "body", f"mission-bay floor M-MIDFLOOR as the lower chord (fixed outer strips {w_fl:.3f} m net "
          "of the tray cut-out, rib_panel faces), strain (DT, sandwich)", "alt başlık olarak görev bölmesi tabanı "
          f"M-MIDFLOOR (tepsi kesiği dışında sabit dış şeritler net {dec(w_fl, 3)} m, rib_panel yüzleri), birim şekil "
          "değiştirme (sandviç)",
          case_f, case_ftr, e_fl * 1e6, c.dt["sand_comp"] * 1e6, "µε", total_factor(c), "STANAG UL13.1.2",
          part="YK250-CH-027")
    Nx = e_fl * (rib["E_out"] * rib["t_out"] + rib["E_in"] * rib["t_in"])
    a_ = station_x(c, "FS-FUEL") - station_x(c, "FS1810")
    n_st = int(c.D["body"]["midfloor_stiffeners"])
    b_p = w_fl / 2 if w_cut > 0 else w_fl / (n_st + 1)       # each strip between the chine and a cut-out stiffener
    pb = panel_buckling_multiplier(c, rib, Nx, 0.0, a_, b_p)
    R.add("B-MIDFLOOR-BUCK", "body", f"mission-bay floor strip {a_:.2f} x {b_p:.3f} m between the chine longeron and the "
          f"cut-out edge stiffener ({n_st} bonded hat stiffeners) buckling (compression)", f"görev bölmesi taban şeridi "
          f"{a_:.2f} x {b_p:.3f} m, kenar uzun kirişi ile kesik kenarı takviyesi arasında ({n_st} yapıştırılmış şapka "
          "takviye) burkulma (bası)", case_f, case_ftr, 1.0, pb["k"], "load multiplier", total_factor(c, comp=True),
          "CLT D*, sandwich shear correction; SS edges", part="YK250-CH-027")
    # aft-body torsion + vertical shear in the fixed skins
    tp = tail["tail_loads"]
    x_t = 0.5 * (station_x(c, "FS-GEAR") + station_x(c, "FS3480"))
    bs = body_section(c, x_t)
    fin_z = float(c.cache["tail_loads"]["tp"]["fin"]["z_mac"])
    T_b = 2 * tail["N_fin"] * (fin_z - bs["zc"]) + (1 - tp["unsym_fraction"]) * tail["N_stab"] * tp["y_cp_stab"]
    q_T = T_b / (2 * bs["A"])
    h_side = max(zd - zc_, 0.05)
    q_V = V_a / (2 * h_side)
    q = q_T + q_V
    sh = skin_faces(c, "shell_secondary")
    fpf = ST.first_ply_failure(sh["outer"], (0.0, 0.0, q / 2), restrained=True)
    case_t = (f"aft-body torsion {T_b:.0f} N m (both fins at {tail['N_fin']:.0f} N + 28 % unsymmetric stabilator) + "
              f"vertical shear {V_a:.0f} N on the side skins (limit): q = {q:.0f} N/m")
    case_ttr = (f"arka gövde burulması {T_b:.0f} N m (iki dikey {tail['N_fin']:.0f} N + %28 simetrik olmayan stabilatör) + "
                f"yan kaplamalarda düşey kesme {V_a:.0f} N (limit): q = {q:.0f} N/m")
    R.add("B-SKIN-SHEAR", "body", "fixed aft skins P-AFT-UPPER / -LOWER (shell_secondary 0.4/5/0.4): face shear, "
          "first-ply failure", "sabit arka kaplamalar P-AFT-UPPER / -LOWER (shell_secondary 0,4/5/0,4): yüz kesmesi, ilk "
          "katman hasarı", case_t, case_ttr, q / 1e3, fpf["R"] * q / 1e3, "N/mm", total_factor(c, comp=True), "CLT",
          part="YK250-SH-388 / -390")
    pb = panel_buckling_multiplier(c, sh, 0.0, q, station_x(c, "FS3480") - station_x(c, "FS-GEAR"), 0.25)
    R.add("B-SKIN-BUCK", "body", "fixed aft skin panel shear buckling (frame pitch x 0.25 m between longerons)",
          "sabit arka kaplama paneli kesme burkulması (çerçeve aralığı x uzun kirişler arası 0,25 m)", case_t, case_ttr,
          1.0, pb["k"], "load multiplier", total_factor(c, comp=True), "Kollár & Springer long plate, sandwich correction",
          part="YK250-SH-388 / -390")
    pitch = 0.032
    As4 = float(research_value("materials", "fasteners.property_classes.stress_area.A_s_m2")["M4"])
    R.add("B-SCREW-SH", "body", "skin screw row ISO 7380 M4 A2-70 at 32 mm pitch: shear (button head 0.8 x class)",
          "kaplama vida sırası ISO 7380 M4 A2-70, 32 mm aralık: kesme (yuvarlak baş 0,8 x sınıf)", case_t, case_ttr,
          q * pitch, 0.8 * 0.6 * 700e6 * As4, "N", total_factor(c, fit=True), "ISO 3506-1 A2-70; materials.yaml screw_types",
          part="shell fasteners")
    R.add("B-SCREW-BR", "body", "skin screw row M4: bearing in the 1.6 mm solid edge band", "kaplama vida sırası M4: 1,6 mm "
          "dolu kenar bandında ezilme", case_t, case_ttr, q * pitch, 0.004 * 0.0016 * qi_design_values(c)["bearing_Pa"],
          "N", total_factor(c, fit=True, comp=True), "QI bearing ETW", part="shell edge bands")
    out.update({"M_aft": M_a, "V_aft": V_a, "M_fwd_per_g": M_f, "T_aft": T_b, "q_aft": q, "cell_area": bs["A"]})
    return out


def check_equipment(c: Ctx, R: Rows) -> dict:
    """Equipment retention (ultimate load factors per direction): down max(1.5 n_equipment, 1.5 (n_j + 0.67) landing,
    6 g emergency, parachute opening at the lightest mass), up max(1.5 |n_neg|, 3 g), forward max(9 g, parachute x sin
    60), side max(1.5 x 1.47, 1.5 g). Required ultimate retention loads per item for the detail design of trays and
    brackets; checks where the layout defines the attachment (parachute container brackets, payload tray screws)."""
    GL = ground_loads(c)
    n_land = 1.5 * GL["n_inertia"] if GL["n_concentrated_mass"] is not None else 0.0
    p_l = PARA_SHOCK_N / (c.m_min * G)
    nd = max(1.5 * c.n_eq_pos, n_land, 6.0, p_l)
    nu = max(1.5 * abs(c.n_eq_neg), 3.0)
    nf = max(9.0, p_l * math.sin(math.radians(60.0)))
    ns = max(1.5 * 1.47, 1.5)
    case = (f"equipment retention, ultimate n: down {nd:.2f}, up {nu:.2f}, forward {nf:.2f}, side {ns:.2f}")
    case_tr = (f"teçhizat tutma, nihai n: aşağı {nd:.2f}, yukarı {nu:.2f}, ileri {nf:.2f}, yan {ns:.2f}")
    items = (("buffer_battery_12S2P_liion", "buffer battery (12S2P Li-ion box)", "tampon batarya (12S2P Li-ion kutu)"),
             ("pdu_dcdc_fuses", "PDU / DC-DC / fuses", "PDU / DC-DC / sigortalar"),
             ("avionics", "avionics (autopilot, datalinks)", "aviyonik (otopilot, veri bağları)"),
             ("parachute_uavos_200", "parachute container", "paraşüt kabı"),
             ("flight_termination_lights", "FTS + lights", "UST + ışıklar"))
    for nm, en, tr_ in items:
        m = mass_item(c, nm)
        R.info(f"EQ-{nm[:12].upper()}", "equipment", f"{en}: required ultimate retention load (down / forward)",
               f"{tr_}: gerekli nihai tutma yükü (aşağı / ileri)", case, case_tr, m * G * max(nd, nf), "N",
               f"mass {m:.2f} kg; tray / bracket / strap design load x 1.15 fitting", part=nm)
    # parachute container: 4 brackets x 2 M5 (shear), largest direction
    m_pc = mass_item(c, "parachute_uavos_200")
    F = m_pc * G * max(nd, nf)
    As5 = float(research_value("materials", "fasteners.property_classes.stress_area.A_s_m2")["M5"])
    R.add("EQ-PARA-BRK", "equipment", "parachute container brackets 4 x (2 x M5 A2-70), shear (one bracket pair carries "
          "half)", "paraşüt kabı dirsekleri 4 x (2 x M5 A2-70), kesme (bir dirsek çifti yarısını taşır)", case, case_tr,
          F / 4, 0.6 * 700e6 * As5, "N", total_factor(c, ult_only=True, fit=True), "0.6 Rm A_s", part="YK250-CH-114")
    # payload tray: 4 x M5 captive screws, research payload at MTOM (n of the MTOM case, no lightest-mass parachute)
    m_pay = sum(float(p["mass_kg"]) for p in c.S["mass"]["payload_items"] if p["name"] in (
        "research_payload_allowance", "research_payload_max_increment", "payload_tray_harness"))
    n_pay = max(1.5 * c.n_wing, n_land, 6.0, PARA_SHOCK_N / (c.m0 * G), 9.0)
    R.add("EQ-PAYTRAY", "equipment", f"payload tray (max payload {m_pay:.2f} kg) 4 x M5 A2-70 captive screws, shear",
          f"faydalı yük tepsisi (en büyük yük {m_pay:.2f} kg) 4 x M5 A2-70 tutsak vida, kesme",
          f"maximum payload at MTOM, ultimate n {n_pay:.2f} (largest of flight, landing, emergency, parachute)",
          f"MTOM'da en büyük faydalı yük, nihai n {n_pay:.2f} (uçuş, iniş, acil, paraşüt en büyüğü)",
          m_pay * G * n_pay / 4, 0.6 * 700e6 * As5, "N", total_factor(c, ult_only=True, fit=True), "0.6 Rm A_s",
          part="YK250-PL-852")
    return {"n_down": nd, "n_up": nu, "n_fwd": nf, "n_side": ns, "n_payload": n_pay}


# =====================================================================================================================
# 12. mass of the sized items (bottom-up) vs the allocations of the mass model
# =====================================================================================================================
def node_mass_each(c: Ctx) -> dict:
    """Machined 7075 stabilator node F-SPINDLE-NODE (one side): layout envelope volumes x the fill fractions of
    structures.sizing.tail.node (base flange with bolt bosses and rails, inboard arm + bearing boss, outboard cheek)."""
    S = c.S
    node = [f for f in S["layout"]["chassis"]["fittings"] if f["id"] == "F-SPINDLE-NODE"][0]
    ND = c.D["tail"]["node"]
    rho = float(mat(c, ND["material"])["density"])
    x_fw = station_x(c, "FS3670")
    y_bi = float(node["cylinder"]["center"][1])
    vol = {"base": 0.0, "outboard": 0.0}
    m = {}
    for bx in node["boxes"]:
        v = float(np.prod(np.subtract(bx[1], bx[0])))
        if abs(bx[0][0] - x_fw) < 1e-6 and bx[1][0] - bx[0][0] < 0.0101:
            vol["base"] += v
        elif bx[0][1] <= y_bi <= bx[1][1]:
            cy = node["cylinder"]
            h_in = float(bx[1][2] - bx[0][2])
            arm = float(cy["center"][0]) - float(cy["radius"]) - float(bx[0][0])
            v_ring = math.pi * (float(cy["radius"]) ** 2 - (0.5 * float(ND["bearing_od_m"])) ** 2) * 2 * \
                float(cy["half_length"])
            m["inboard"] = (v_ring + float(ND["inboard_web_t_m"]) * h_in * max(arm, 0.0)) * rho
        else:
            vol["outboard"] += v
    m.update({k: v * float(ND["fill"][k]) * rho for k, v in vol.items()})
    m["total"] = sum(m.values())
    return m


def mass_tally(c: Ctx, sized: dict) -> dict:
    """Bottom-up masses of the items sized in this phase against the allocations of the sizing mass model (both
    before the growth allowance): (A) wing primary structure - main / rear spar caps (ply schedule x 1.10 for ply
    drops and overlaps, as the model), webs (x 1.25, as the model), the outer-panel joint parts of the wing (CFRP
    tongues with their cap build-up, boss blocks and bushes; rear-spar lugs) - against the caps, webs, rear spar and
    joint terms of sizing.wing_structure; (B) carry-through box covers, web doublers, CFRP forks (glove webs, pads,
    cap widening, bushes), main and rear pins, rear slot fittings, kink fitting against mass.rules.carry_through_kg;
    (C) new / changed chassis members (incl. the firewall lands); (D) stabilator spindles, sockets, node fittings and
    bearings against the chassis item stabilator_spindle_bearing_housings + the stabilator fittings; (E) bottom-ups
    that replace concept allocations of the chassis group (nose-gear pivot fitting, parachute spine and fittings)."""
    S = c.S
    M_ = c.M
    ud, pw = M_["cfrp_ud_mtm45_as4"], M_["cfrp_pw_mtm45_as4"]
    rho_ud, rho_pw = float(ud["density"]), float(pw["density"])
    t_ud, t_pw = float(ud["ply_t"]), float(pw["ply_t"])
    rho_al = float(M_["al_7075_t651_plate"]["density"])
    rho_ti = float(M_["ti_6al_4v_annealed_sheet"]["density"])
    rho_st = float(M_["steel_4130_n"]["density"])
    D = c.D
    b2 = c.T["b2"]
    yj = float(c.P["y_junction"])
    WJ = S["layout"]["chassis"]["wing_joint"]
    out = {}
    # (A) wing primary
    caps = sized["main_cap_zones"]
    v_caps = 0.0
    for y0, y1, n in caps:
        w = D["wing"]["main_cap"]["width_body_glove_m"] if y0 < yj else D["wing"]["main_cap"]["width_outer_m"]
        v_caps += n * t_ud * w * (min(y1, b2) - y0)
    m_caps = 4 * v_caps * rho_ud * 1.10
    m_rcaps = 4 * D["wing"]["rear_cap"]["plies"] * t_ud * D["wing"]["rear_cap"]["width_m"] * b2 * rho_ud * 1.10
    ys = np.linspace(0.705, b2 - 0.01, 40)
    hw = []
    for y in ys:
        sec = wing_section(c, y, cap_plies_at(caps, y))
        hw.append(max(sec["depth_main"] - 2 * sec["t_skin_cap"] - 2 * sec["t_main_cap"], 0.0))
    nw = np.array([cap_plies_at(sized["main_web_zones"], y) for y in ys])
    m_web = 2 * float(np.trapz(nw * t_pw * np.array(hw), ys)) * rho_pw * 1.25
    ysr = np.linspace(Y_SOB + 0.005, b2 - 0.01, 40)
    hr2 = [max(wing_section(c, y, cap_plies_at(caps, y))["depth_rear"] - 0.002 - 0.0028, 0.0) for y in ysr]
    m_rweb = 2 * float(np.trapz(D["wing"]["rear_web"]["plies"] * t_pw * np.array(hr2), ysr)) * rho_pw * 1.25
    JD = D["wing_joint"]
    tg, fk, bu, rr = JD["tongue"], JD["fork"], JD["bush"], JD["rear"]
    wt, h, tf = float(tg["width_m"]), float(tg["height_m"]), float(tg["flange_t_m"])
    t_web = int(tg["web_plies"]) * t_pw
    y_tip = float(WJ["main_spar"]["fork"]["slot"]["y"][0])
    L_eng = yj - y_tip
    hw_ = h - 2 * tf
    bl = float(tg["boss_length_m"])
    OD, ID = float(bu["od_m"]), float(bu["id_m"])
    v_fl = 2 * wt * tf * L_eng
    TPm = tg.get("flange_taper")
    if TPm:                                     # flange ply drop between the pins (fix round 2, mass closure)
        py_ = sorted(float(p_["position"][1]) for p_ in WJ["main_spar"]["pins"])
        t_m = float(TPm["t_min_m"])
        v_fl = 2 * wt * (tf * (yj - py_[-1]) + 0.5 * (tf + t_m) * (py_[-1] - py_[0]) + t_m * (py_[0] - y_tip))
    v_web = hw_ * t_web * L_eng
    if TPm:                                     # the web fills the depth freed by the thinner flanges
        v_web += 2 * t_web * (tf - t_m) * (0.5 * (py_[-1] - py_[0]) + (py_[0] - y_tip))
    v_boss = 2 * (wt - t_web) * hw_ * bl - 2 * math.pi / 4 * OD ** 2 * wt
    t_cap0 = cap_plies_at(caps, yj + 0.001) * t_ud
    v_bu = 2 * wt * max(tf - t_cap0, 0.0) / 2 * float(tg["buildup_length_m"])
    m_bush_t = 2 * math.pi / 4 * (OD ** 2 - ID ** 2) * float(bu["length_tongue_m"]) * rho_st
    m_tongue = (v_fl + v_bu) * rho_ud + (v_web + v_boss) * rho_pw + m_bush_t
    lug = WJ["rear_spar"]["lug"]
    tl, wl = float(lug["thickness"]), float(lug["width"])
    m_lug = (tl * wl * 0.044 + 0.040 * 0.040 * 0.004) * rho_al * 0.85
    joint_outer = 2 * (m_tongue + m_lug)
    def areal(key):
        f = skin_faces(c, key)
        return (f["t_out"] + f["t_in"]) * rho_pw + f["c"] * float(f["core"]["density"])
    yb = np.linspace(Y_SOB, float(D["wing"]["box_skin_upper_y_end_m"]), 60)
    A_bx = 2 * float(np.trapz((float(c.P["rear_spar_frac"]) - float(c.P["main_spar_frac"])) * c.T["c"](yb), yb))
    m_bx = A_bx * (areal(D["wing"]["box_skin_upper_layup"]) - areal(D["wing"]["skin_layup"]))
    S_alloc = copy.deepcopy(S)                    # the concept mass model without the structures-phase values
    S_alloc["structures"].pop("sizing", None)
    ws = Z.wing_structure(S_alloc, c.af, c.m0, c.n_wing)
    alloc_A = ws["caps"] + ws["webs"] + ws["rear_spar"] + ws["joints"]
    TRN = JD.get("transition") or {}
    n_db = int(TRN.get("root_bay_skin_doubler_plies_per_face", 0))
    L_db = float(TRN.get("root_bay_doubler_length_m", 0.0))
    w_bx_j = (float(c.P["rear_spar_frac"]) - float(c.P["main_spar_frac"])) * float(c.T["c"](yj + 0.5 * L_db))
    n_fc = 1 if TRN.get("root_bay_doubler_faces", "both") == "outer" else 2
    m_rbd = 2 * 2 * n_fc * n_db * t_pw * w_bx_j * L_db * rho_pw       # 2 panels x 2 skins x faces
    bu_A = m_caps + m_rcaps + m_web + m_rweb + joint_outer + m_bx + m_rbd
    out["A_wing_primary"] = {"bottom_up_kg": bu_A, "allocation_kg": alloc_A, "delta_kg": bu_A - alloc_A,
                             "items": {"main_caps": m_caps, "rear_caps": m_rcaps, "main_webs": m_web, "rear_webs": m_rweb,
                                       "outer_joint_parts_pair": joint_outer, "tongue_each": m_tongue,
                                       "rear_lug_each": m_lug, "box_skin_upper_core_delta": m_bx,
                                       "root_bay_skin_doublers_pair": m_rbd},
                             "model_items": {"caps": ws["caps"], "webs": ws["webs"], "rear_spar": ws["rear_spar"],
                                             "joints": ws["joints"]}}
    # (B) carry-through box and CFRP forks
    sk = skin_faces(c, D["wing"]["ct_box"]["cover_layup"])
    ar = (sk["t_out"] + sk["t_in"]) * rho_pw + sk["c"] * float(sk["core"]["density"])
    x_ms, x_rs = station_x(c, "FS-MS"), station_x(c, "FS-RS")
    w_box = x_rs - x_ms - 0.5 * (D["wing"]["main_cap"]["width_body_glove_m"] + D["wing"]["rear_cap"]["width_m"])
    m_cov = 2 * w_box * 2 * Y_SOB * ar
    nd = sum(int(v) for v in D["wing"]["ct_box"]["web_doubler_plies_per_face"].values())
    m_dbl = 2 * 2 * Y_SOB * 0.070 * nd * t_pw * rho_pw          # both faces x 0.8 m x web depth, per frame ply
    hs = float(WJ["main_spar"]["fork"]["slot"]["height"])
    L_f = yj - Y_SOB
    t_pr = int(fk["prong_plies"]) * t_pw
    # fix round 3 (VS3-01): full-depth prongs (glove main-spar webs between the cap faces), mean height over the fork
    h_pr_m = float(np.mean([prong_height(c, caps, y) for y in np.linspace(Y_SOB + 0.005, yj - 0.005, 15)]))
    v_prong = 2 * t_pr * h_pr_m * L_f
    v_pad = 2 * 2 * (float(fk["prong_pad_t_m"]) - t_pr) * hs * float(fk["pad_length_m"])
    n_fz = float(np.mean([cap_plies_at(caps, y) for y in np.linspace(Y_SOB + 0.005, yj - 0.005, 20)]))
    v_widen = 2 * (float(fk["cap_width_m"]) - float(D["wing"]["main_cap"]["width_body_glove_m"])) * n_fz * t_ud * L_f
    m_bush_f = 4 * math.pi / 4 * (OD ** 2 - ID ** 2) * float(bu["length_prong_m"]) * rho_st
    m_fork = (v_prong + v_pad) * rho_pw + v_widen * rho_ud * 1.10 + m_bush_f + 0.010
    pins = WJ["main_spar"]["pins"]
    Dp = float(pins[0]["diameter"])
    Lp = float(pins[0]["length"])
    m_pin = math.pi / 4 * Dp ** 2 * Lp * rho_ti + 0.010
    rp = WJ["rear_spar"]
    Dr = float(rp["pin"]["diameter"])
    m_rpin = math.pi / 4 * Dr ** 2 * float(rp["pin"]["length"]) * rho_ti + 0.008
    tpl = float(rp["slot_fitting"]["plate_t"])
    m_slot = (2 * tpl * 0.036 * 0.040 + 0.035 * 0.020 * 0.004 * 2) * rho_al * 0.85
    KF = D["wing"]["ct_box"].get("kink_fitting")
    if KF:
        m_kink = 2 * (float(KF["w_m"]) * float(KF["plate_t_m"]) * float(KF["length_m"]) +
                      float(KF["length_m"]) * float(KF["tab_h_m"]) * float(KF["tab_t_m"])) * rho_al + \
            2 * int(KF["bolts"]) * (math.pi / 4 * float(KF["bolt_d_m"]) ** 2 * 0.030 * rho_ti + 0.002)
    else:
        m_kink = 0.10
    # fix round 3 (VS3-01): SOB ramp - solid lands of the SOB rib under the main caps (land - sandwich it replaces)
    # and the tapered filler wedges between the glove skins and the ramped caps (4 caps x 2 sides)
    TDs = D["wing"].get("sob_transition")
    m_sob = 0.0
    if TDs:
        rib_ = skin_faces(c, "rib_panel")
        a_rib = (rib_["t_out"] + rib_["t_in"]) * rho_pw + rib_["c"] * float(rib_["core"]["density"])
        w_c = float(D["wing"]["main_cap"]["width_body_glove_m"])
        m_land = 4 * float(TDs["land_length_m"]) * w_c * (int(TDs["rib_land_plies"]) * t_pw * rho_pw - a_rib)
        y0r, y1r = sob_ramp(c)
        Kq = sob_kinks(c, sized)
        rho_f = float(M_[TDs["filler"]]["density"])
        m_fill = 2 * w_c * (y1r - y0r) * 0.5 * (abs(Kq["dz_up"]) + abs(Kq["dz_lo"])) * rho_f
        s0_ = sob_kinks(c, sized)["sec0"]
        m_rdb = 2 * 2 * 2 * int(TDs.get("rib_face_doubler_plies", 0)) * t_pw * float(TDs.get("rib_doubler_band_m", 0.0)) * \
            (s0_["x_rear"] - s0_["x_main"]) * rho_pw                  # 2 sides x 2 edges x 2 faces
        m_sob = m_land + m_fill + m_rdb
    bu_B = m_cov + m_dbl + 2 * m_fork + 4 * m_pin + 2 * (m_rpin + m_slot) + m_kink + m_sob
    alloc_B = float(S["mass"]["rules"]["carry_through_kg"])
    out["B_carry_through"] = {"bottom_up_kg": bu_B, "allocation_kg": alloc_B, "delta_kg": bu_B - alloc_B,
                              "items": {"sandwich_covers": m_cov, "web_doublers": m_dbl, "cfrp_forks_pair": 2 * m_fork,
                                        "main_pins_4": 4 * m_pin, "rear_pins_slot_fittings_2": 2 * (m_rpin + m_slot),
                                        "kink_fittings_2": m_kink, "sob_ramp_lands_fillers": m_sob}}
    # (C) chassis members added / changed in this phase
    mem = {m_["id"]: m_ for m_ in S["layout"]["chassis"]["members"]}
    FD = D["fuel_bay"]
    fl = skin_faces(c, FD["floor_layup"])
    rib = skin_faces(c, "rib_panel")
    a_new = (fl["t_out"] + fl["t_in"]) * rho_pw + fl["c"] * float(fl["core"]["density"])
    a_old = (rib["t_out"] + rib["t_in"]) * rho_pw + rib["c"] * float(rib["core"]["density"])
    A_fl = 0.0
    for mid in FD["floors"]:
        bx = mem[mid]["box"]
        A_fl += (bx[1][0] - bx[0][0]) * (bx[1][1] - bx[0][1])
    m_floor = A_fl * (a_new - a_old)
    wk = mem["M-WELLKEEL"]["box"] if "M-WELLKEEL" in mem else mem["M-WELLROOF"]["box"]
    m_web_c = 2 * float(FD["well_web_t_m"]) * float(FD["well_web_height_m"]) * (wk[1][0] - wk[0][0]) * rho_pw * 1.15
    m_stf = int(D["body"]["midfloor_stiffeners"]) * 4 * t_pw * (0.020 + 2 * 0.015 + 2 * 0.008) * \
        (station_x(c, "FS-FUEL") - station_x(c, "FS1810")) * rho_pw
    ak = mem["M-AFTKEEL"]
    akd = D["body"]["aft_keel"]
    L_ak = ak["box"][1][0] - ak["box"][0][0]
    o_ = akd["replaces"]                                # the layout-phase channel (the concept allocation)
    v_old = (2 * o_["h_m"] + o_["w_m"] - 2 * o_["t_m"]) * o_["t_m"] * L_ak
    v_new = (2 * akd["h_m"] + akd["w_m"] - 2 * akd["t_m"]) * akd["t_m"] * L_ak
    m_ak = v_new * float(M_[akd["material"]]["density"]) - v_old * float(M_[o_["material"]]["density"])
    FW = D["firewall"]
    fland = FW["foot_land"]
    r_l = float(fland["land_r_m"])
    t_l = int(fland["land_plies"]) * t_pw
    a_sw = (rib["t_out"] + rib["t_in"]) * rho_pw + rib["c"] * float(M_[fland["core"]]["density"])
    m_fwland = 2 * math.pi * r_l ** 2 * (t_l * rho_pw - a_sw)
    m_fwcore = float(FW["core_insert"]["area_m2"]) * rib["c"] * (float(M_[FW["core_insert"]["core"]]["density"]) -
                                                               float(rib["core"]["density"]))
    SG = D["body"].get("fs3738_lower_segment")
    m_seg = 0.0
    if SG:
        A_new = 2 * float(SG["flange_w_m"]) * float(SG["flange_t_m"]) + \
            (float(SG["depth_m"]) - 2 * float(SG["flange_t_m"])) * float(SG["web_t_m"])
        o_s = SG["replaces"]
        A_old = (float(o_s["depth_m"]) + 2 * float(o_s["flange_w_m"])) * float(o_s["t_m"])
        m_seg = float(SG["span_y_m"]) * (A_new * float(M_[SG["material"]]["density"]) -
                                         A_old * float(M_[o_s["material"]]["density"])) + \
            (int(SG["keel_bolts"]) + 2 * int(SG["end_bolts"])) * 0.004
    n_rd = int(D["gear"].get("roof_fitting_doubler_plies_per_face", 0))
    m_rd = 2 * 2 * n_rd * t_pw * 0.15 * 0.10 * rho_pw                 # 2 sides x 2 faces
    DS = D["body"].get("deck_slot_strip")
    m_ds = 0.0
    if DS and "M-DECK-NOSE" in mem:
        db_ = mem["M-DECK-NOSE"]["box"]
        m_ds = (db_[1][0] - db_[0][0]) * 0.083 * (int(DS["plies"]) * t_pw * rho_pw - a_old)
    bu_C = m_floor + m_web_c + m_stf + m_ak + m_fwland + m_fwcore + m_seg + m_rd + m_ds
    out["C_chassis_changes"] = {"bottom_up_kg": bu_C, "allocation_kg": 0.0, "delta_kg": bu_C,
                                "items": {"fuel_floors_layup_delta": m_floor, "well_web_M-WELLKEEL": m_web_c,
                                          "midfloor_stiffeners": m_stf, "aft_keel_delta": m_ak,
                                          "firewall_foot_lands": m_fwland, "firewall_core_insert": m_fwcore,
                                          "fs3738_lower_segment_delta": m_seg, "well_roof_fitting_doublers": m_rd,
                                          "deck_slot_strip_delta": m_ds}}
    # (D) stabilator spindles, sockets, node fittings, bearings
    sp = D["tail"]["spindle"]
    node = [f for f in S["layout"]["chassis"]["fittings"] if f["id"] == "F-SPINDLE-NODE"][0]
    L_sp = float(node["spindle"]["span_y"][1]) - float(node["spindle"]["span_y"][0])
    m_sp = math.pi / 4 * (sp["od_m"] ** 2 - sp["id_m"] ** 2) * L_sp * float(M_[sp["material"]]["density"])
    nm_ = node_mass_each(c)
    m_brg = 0.020
    SK = D["tail"]["socket"]
    m_sock = math.pi / 4 * (float(SK["od_m"]) ** 2 - sp["od_m"] ** 2) * float(SK["length_m"]) * \
        float(M_[SK["material"]]["density"]) + 0.010
    bu_D = 2 * (m_sp + nm_["total"] + 2 * m_brg + m_sock)
    alloc_D = 0.40 + 2 * float(S["tail"]["surfaces"]["stabilator"]["fittings_kg"])
    out["D_spindles"] = {"bottom_up_kg": bu_D, "allocation_kg": alloc_D, "delta_kg": bu_D - alloc_D,
                         "items": {"spindle_each": m_sp, "node_each": nm_["total"], "node_base_each": nm_["base"],
                                   "node_inboard_each": nm_["inboard"], "node_outboard_each": nm_["outboard"],
                                   "bearing_each_estimate": m_brg, "root_socket_and_cross_bolt_each": m_sock},
                         "allocation_basis": "chassis item stabilator_spindle_bearing_housings 0.40 + "
                                             "tail.surfaces.stabilator.fittings_kg x 2 (spindle / root fittings)"}
    # (E) bottom-ups replacing concept allocations of the chassis group
    GD = D["gear"]
    npv = [f for f in S["layout"]["chassis"]["fittings"] if f["id"] == "F-NG-PIVOT"][0]
    v_blk = sum(float(np.prod(np.subtract(bx[1], bx[0]))) for bx in npv["boxes"])
    sax_n = npv["pivot_scheme"]["stub_axle"]
    Dn, Dno = float(sax_n["d"]), float(GD["nose_pivot_bush_od_m"])
    m_np = (v_blk - 2 * math.pi / 4 * Dno ** 2 * float(GD["nose_pivot_lug_t_m"])) * rho_al + \
        2 * (math.pi / 4 * (Dno ** 2 - Dn ** 2) * 0.012) * rho_st + \
        2 * (math.pi / 4 * Dn ** 2 * float(sax_n["length"]) * rho_st + 0.003)      # two solid stub axles + flanges
    sp_m = mem["M-SPINE"]
    sec = sp_m["section"]
    L_spn = float(sp_m["box"][1][0] - sp_m["box"][0][0])
    perim = float(sec["w"]) + 2 * float(sec["h"]) + 2 * float(sec["flange_w"])
    m_spine = perim * float(sec["t"]) * L_spn * rho_pw
    m_pads = 2 * 0.048 * float(sec["w"]) * (float(sec["pad_t"]) - float(sec["t"])) * rho_pw
    fits = {f["id"]: f for f in S["layout"]["chassis"]["fittings"]}
    m_rf = 0.0
    for fid in ("F-RISER-FWD", "F-RISER-AFT"):
        bx = fits[fid]["box"]
        m_rf += float(np.prod(np.subtract(bx[1], bx[0]))) * 0.35 * rho_al
    PD = D["parachute"]
    rho_pin = float(M_[PD["shackle_material"]]["density"])
    m_shk = 2 * math.pi / 4 * float(PD["shackle_pin_d_m"]) ** 2 * 0.040 * rho_pin
    wpl = PD.get("floor_washer_plate")
    if wpl:
        m_shk += 2 * float(wpl["w_m"]) * float(wpl["l_m"]) * float(wpl["t_m"]) * rho_al      # washer plates
    m_shk += 2 * math.pi / 4 * (0.012 ** 2 - 0.0082 ** 2) * 0.005 * rho_st                  # bridle spools
    t_pl = int(PD["frame_land_plies"]) * t_pw
    m_fland = 2 * 0.040 * 0.040 * (t_pl * rho_pw - a_old)
    m_para = m_spine + m_pads + m_rf + m_shk + m_fland
    alloc_E = {"nose_pivot_fitting": 0.30, "parachute_spine_fittings": 0.35}
    bu_E = {"nose_pivot_fitting": m_np, "parachute_spine_fittings": m_para}
    out["E_allocation_replacements"] = {"bottom_up_kg": sum(bu_E.values()), "allocation_kg": sum(alloc_E.values()),
                                        "delta_kg": sum(bu_E.values()) - sum(alloc_E.values()),
                                        "items": {"nose_pivot_fitting": m_np, "parachute_spine": m_spine,
                                                  "parachute_spine_pads": m_pads, "parachute_fittings_2": m_rf,
                                                  "parachute_shackle_pins_spools_washer_plates_2": m_shk,
                                                  "parachute_frame_lands_2": m_fland},
                                        "allocation_basis": "sizing.mass_items nose_gear_trunnion_fitting 0.30 and "
                                                            "parachute_attach_fitting 0.35 (endurance study)"}
    gr = 1.0 + float(S["mass"]["rules"]["growth_allowance"])
    tot = sum(v["delta_kg"] for v in out.values())
    out["total_delta_base_kg"] = tot
    out["total_delta_with_growth_kg"] = tot * gr
    out["note"] = ("bottom-up masses of the sized items vs the allocations of the sizing mass model, before the growth "
                   "allowance; A in the wing group (sizing.wing_structure terms x the model factor 1.05 on the wing item "
                   "are not applied here), B-E in the chassis group except the stabilator spindles and sockets (tail "
                   "group, x the tail model factor 1.05)")
    return out



# =====================================================================================================================
# 13. run, spec block, interfaces, outputs, CLI
# =====================================================================================================================
def open_items(res: dict) -> list:
    """(en, tr) open items of the structures phase; the numbers are formatted from the computed results (fix round 1,
    S1-07)."""
    F_k = res["ct_box"]["F_kink"] / 1e3
    R38 = res["frames"]["R_FS3738_keel"] / 1e3
    return [
        ("flutter / divergence / aileron reversal not analysed (no stiffness / mass model of the wing and tail yet): GVT "
         "and a flutter analysis (CS-LUAS.629) before the first flight; the +-45-dominated skins raise the torsional "
         "stiffness",
         "çırpınma / ıraksama / kanatçık tersinmesi analiz edilmedi (kanat ve kuyruğun rijitlik / kütle modeli henüz "
         "yok): ilk uçuştan önce yer titreşim testi (GVT) ve çırpınma analizi (CS-LUAS.629); ±45 ağırlıklı kaplamalar "
         "burulma rijitliğini artırır"),
        ("fatigue / damage tolerance of the metallic parts (pins, bushes, rear lug, node and gear fittings, engine-mount "
         "welds) and the composite BVID/CVID substantiation (CS-LUAS.572/573) by test; the hand calculations use the "
         "STANAG UL13.1.2 strain limits only",
         "metal parçaların (pimler, burçlar, arka kulak, düğüm ve takım bağlantıları, motor bağlantısı kaynakları) "
         "yorulması / hasar toleransı ve kompozit BVID/CVID kanıtı (CS-LUAS.572/573) testle; el hesapları yalnız STANAG "
         "UL13.1.2 birim şekil değiştirme sınırlarını kullanır"),
        ("composite outer-panel joint: the bush bearing on the tongue / prong laminates is checked against the QI "
         "open-hole compression strength as a conservative substitute (bore e/D below 3); an element test of the pinned "
         "CFRP tongue / fork (static, ETW, fatigue) is required before the design values are frozen",
         "kompozit dış panel birleşimi: burçların dil / kulak laminelerindeki ezilmesi, ihtiyatlı bir yerine koyma olarak "
         "QI delikli bası dayanımıyla kontrol edildi (delik e/D değeri 3'ün altında); tasarım değerleri dondurulmadan "
         "önce pimli CFRP dil / çatal eleman testi (statik, ETW, yorulma) gerekli"),
        ("procurement requirements (rows without MS): bearing static ratings (61805-ZZ), ball-screw / rail ratings of "
         "the turret elevator, engine isolator ultimate capacity, gear-unit EMA torques and the nose down-lock - "
         "catalogue values are not in the research data",
         "tedarik gereksinimleri (MS'siz satırlar): yatak statik yük sayıları (61805-ZZ), taret asansörü bilyalı vida / "
         "ray yük sayıları, motor sönümleyici nihai kapasitesi, takım ünitesi EMA torkları ve burun aşağı kilidi - "
         "katalog değerleri araştırma verisinde yok"),
        ("main inner gear doors: the DA 22 cannot hold the closed door against the VD suction; an over-centre linkage or "
         "a door up-lock is required (G-DOOR-LOCK)",
         "ana iç takım kapakları: DA 22 kapalı kapağı VD emmesine karşı tutamaz; ölü nokta bağlantısı veya kapak kilidi "
         "gerekli (G-DOOR-LOCK)"),
        (f"centre kink fitting (CT-KINK-*, {F_k:.1f} kN per cap, limit) and the FS3738 lower segment (FR-3738-*, keel "
         f"reaction {R38:.1f} kN, limit) are sized as hand-calculation concepts (fix round 2, VS2-03); their detail "
         "drawings and the load-limiting skid option remain detail design",
         f"orta kırık bağlantısı (CT-KINK-*, başlık başına {dec(F_k, 1)} kN, limit) ve FS3738 alt parçası (FR-3738-*, "
         f"omurga tepkisi {dec(R38, 1)} kN, limit) el hesabı konsepti olarak boyutlandırıldı (düzeltme turu 2, VS2-03); "
         "detay çizimleri ve yük sınırlayıcı kızak seçeneği detay tasarımda kalır"),
        ("hot zone: the 7075 node F-SPINDLE-NODE sits 10 mm from the cylinder-head envelope behind a stainless baffle; "
         "its design temperature is not known and the MMPDS elevated-temperature curves are not in the research data - "
         "T-NODE-TEMP gives the strength retention the margins need; measure the node temperature in the engine run",
         "sıcak bölge: 7075 düğüm F-SPINDLE-NODE paslanmaz bir perdenin arkasında silindir kafası zarfına 10 mm "
         "mesafede; tasarım sıcaklığı bilinmiyor ve MMPDS yüksek sıcaklık eğrileri araştırma verisinde yok - "
         "T-NODE-TEMP marjların gerektirdiği dayanım oranını verir; düğüm sıcaklığı motor çalıştırmasında ölçülmeli"),
        ("ground handling: towing at the nose fork with 0.3 W (CS-23.509 value, the CS-LUAS / CS-VLA text not in the "
         "research files) is checked (G-TOW-*); outdoor tie-down is excluded by the operating concept and jacking is "
         "not provided (cradle saddles) - to be confirmed by the operator",
         "yer işlemleri: burun çatalından 0,3 W ile çekme (CS-23.509 değeri; CS-LUAS / CS-VLA metni araştırma "
         "dosyalarında yok) kontrol edildi (G-TOW-*); açık havada bağlama işletme konseptiyle dışlandı, kriko noktası "
         "yok (beşik eyerleri) - işletmeci tarafından teyit edilmeli"),
        ("design choices without a specification in the research files: transport load factors (3.0 / 1.5 / 1.5 g), the "
         "tail-bumper strike load (1.0 x MTOM weight at 45 deg), the gear operating speed 1.6 VS; to be replaced by the "
         "operator / test data",
         "araştırma dosyalarında belirtimi olmayan tasarım kararları: taşıma yük katsayıları (3,0 / 1,5 / 1,5 g), kuyruk "
         "tamponu çarpma yükü (45°'de 1,0 x MTOM ağırlığı), takım işletme hızı 1,6 VS; işletmeci / test verisiyle "
         "değiştirilecek"),
        ("allowables: Ti-6Al-4V sheet values used for bar parts (pins, spindles), 7075-T6 sheet for the gear-leg tubes, "
         "core minimum values, MTM45-1 ETW B-basis; coupon / element tests of the sandwich skins (crimping, wrinkling) "
         "and of the potted inserts are required",
         "izin verilen değerler: çubuk parçalar (pimler, miller) için Ti-6Al-4V sac değerleri, takım bacağı boruları "
         "için 7075-T6 sac, çekirdek minimum değerleri, MTM45-1 ETW B-tabanı; sandviç kaplamaların (kıvrılma, buruşma) "
         "ve dökme insertlerin kupon / eleman testleri gerekli"),
    ]


def dt_applicability(c: Ctx, R: Rows) -> dict:
    """Fix round 2 (VS2-11): applicability of the STANAG 4703 UL13.1.2 damage-tolerance strain limits - the hot/wet
    (ETW) degradation of the matrix-sensitive properties must stay below 50 % of the room-temperature (RTD) value.
    Like-for-like NCAMP values (same direction; B-basis where both are published, else means)."""
    base = "composites.laminae."
    ud, pw = base + "cfrp_ud_ooa_prepreg_as4_145.strength_Pa.", base + "cfrp_pw_ooa_prepreg_as4_193.strength_Pa."
    out = {}
    for key, path, prop in (("UD Xc", ud, "Xc"), ("PW Xc (warp)", pw, "Xc"), ("PW Yc (fill)", pw, "Yc"),
                            ("PW S_0.2", pw, "S_0p2")):
        e_, r_ = research_value("materials", path + "ETW." + prop), research_value("materials", path + "RTD." + prop)
        basis = "b_basis" if e_.get("b_basis") and r_.get("b_basis") else "mean"
        out[key] = {"ETW": float(e_[basis]), "RTD": float(r_[basis]), "basis": basis,
                    "degradation": 1.0 - float(e_[basis]) / float(r_[basis])}
    worst = max(out.values(), key=lambda v: v["degradation"])
    wk = [k for k, v in out.items() if v is worst][0]
    txt = "; ".join(f"{k} {v['degradation'] * 100:.0f} % ({v['basis']})" for k, v in out.items())
    R.info("DT-COND", "wing", "applicability of the STANAG UL13.1.2 damage-tolerance strain limits: ETW degradation of "
           "the matrix-sensitive properties below 50 % of RTD", "STANAG UL13.1.2 hasar toleransı birim şekil "
           "değiştirme sınırlarının uygulanabilirliği: matrise duyarlı özelliklerin ETW kaybı RTD'nin % 50'sinin altında",
           "material qualification data (NCAMP MTM45-1/AS4, materials.yaml)", "malzeme yeterlilik verisi (NCAMP "
           "MTM45-1/AS4, materials.yaml)", worst["degradation"], "fraction", f"largest: {wk}; {txt}; condition "
           "< 0.50 met -> UL13.1.2 strain limits used (fix round 2, VS2-11)", requirement=0.50)
    return out


def run_all(c: Ctx, sized: dict | None = None) -> dict:
    """Every load case and member check. ``sized``: wing spar-cap / web ply zones (sized here unless given, e.g. read
    from the spec in check mode)."""
    R = Rows()
    if sized is None:
        sized = size_wing(c)
    res = {"sized": sized}
    res["flight_envelope"] = flight_envelope(c)
    res["dt_applicability"] = dt_applicability(c, R)
    res["wing"] = check_wing(c, R, sized)
    res["wing_joint"] = check_wing_joint(c, R, sized)
    res["sob_transition"] = check_sob_transition(c, R, sized)
    res["lerx_skins"] = check_lerx_skins(c, R, sized)
    res["ct_box"] = check_ct_box(c, R, sized)
    res["tail"] = check_tail(c, R)
    res["controls"] = check_controls(c, R)
    res["gear"] = check_gear(c, R)
    res["engine_mount"] = check_engine_mount(c, R)
    res["parachute"] = check_parachute(c, R)
    res["fuel_bays"] = check_fuel_bays(c, R)
    res["turret"] = check_turret(c, R)
    res["transport"] = check_transport(c, R)
    res["body"] = check_body(c, R, res["tail"])
    res["frames"] = check_frames(c, R, res["ct_box"], res["engine_mount"])
    res["equipment"] = check_equipment(c, R)
    res["mass"] = mass_tally(c, sized)
    res["rows"] = R.rows
    ms_rows = [r for r in R.rows if r["ms"] is not None]
    worst_row = min(ms_rows, key=lambda r: r["ms"])
    res["summary"] = {"n_rows": len(R.rows), "n_margins": len(ms_rows),
                      "n_requirements": len(R.rows) - len(ms_rows),
                      "n_negative": sum(1 for r in ms_rows if r["ms"] < 0.0),
                      "ms_min": worst_row["ms"], "ms_min_id": worst_row["id"]}
    return res


def mass_block(mt: dict) -> dict:
    """The structures-phase masses (kg, before the growth allowance) the sizing mass model consumes
    (sizing.wing_structure / mass_items / tail_structure)."""
    A, B = mt["A_wing_primary"]["items"], mt["B_carry_through"]
    C, Dd = mt["C_chassis_changes"]["items"], mt["D_spindles"]["items"]
    E = mt["E_allocation_replacements"]["items"]

    def r(v):
        return round(float(v), 3)
    return {
        "note": ("bottom-up masses of the sized structure (structures.mass_tally), before the growth allowance; read by "
                 "sizing.wing_structure (caps, webs, rear spar, joints, skins_add), sizing.mass_items (carry-through, "
                 "keel and floor additions, firewall lands, stabilator node fittings and bearings, nose-gear pivot "
                 "fitting, parachute spine and fittings) and sizing.tail_structure (stabilator spindle + socket)"),
        "wing": {"caps_kg": r(A["main_caps"]), "webs_kg": r(A["main_webs"]), "rear_spar_kg": r(A["rear_caps"] +
                                                                                               A["rear_webs"]),
                 "joints_kg": r(A["outer_joint_parts_pair"]),
                 "skins_add_kg": r(A["box_skin_upper_core_delta"] + A.get("root_bay_skin_doublers_pair", 0.0))},
        "chassis": {"carry_through_kg": r(B["bottom_up_kg"]),
                    "keel_beams_longerons_add_kg": r(C["well_web_M-WELLKEEL"] + C["aft_keel_delta"]),
                    "floors_trays_rails_add_kg": r(C["fuel_floors_layup_delta"] + C["midfloor_stiffeners"] +
                                                   C.get("well_roof_fitting_doublers", 0.0) +
                                                   C.get("deck_slot_strip_delta", 0.0)),
                    "frames_bulkheads_add_kg": r(C["firewall_foot_lands"] + C["firewall_core_insert"] +
                                                 C.get("fs3738_lower_segment_delta", 0.0)),
                    "stabilator_spindle_bearing_housings_kg": r(2 * (Dd["node_each"] + 2 * Dd["bearing_each_estimate"])),
                    "nose_pivot_fitting_kg": r(E["nose_pivot_fitting"]),
                    "parachute_spine_fittings_kg": r(E["parachute_spine"] + E["parachute_spine_pads"] +
                                                     E["parachute_fittings_2"] +
                                                     E["parachute_shackle_pins_spools_washer_plates_2"] +
                                                     E["parachute_frame_lands_2"])},
        "tail": {"stabilator_fittings_kg_each": r(Dd["spindle_each"] + Dd["root_socket_and_cross_bolt_each"])},
        "delta_vs_concept_allocation_kg": {k: r(v["delta_kg"]) for k, v in mt.items() if isinstance(v, dict)},
        "delta_total_kg": r(mt["total_delta_base_kg"])}


def sizing_block(c: Ctx, res: dict) -> dict:
    """spec.structures.sizing: the authored design values + the sized ply zones + the mass block + a summary."""
    D = copy.deepcopy(c.D)
    for k in ("mass", "summary"):
        D.pop(k, None)
    D["wing"]["main_cap"]["zones"] = [[round(float(a), 3), round(float(b), 3), int(n)]
                                      for a, b, n in res["sized"]["main_cap_zones"]]
    D["wing"]["main_web"]["zones"] = [[round(float(a), 3), round(float(b), 3), int(n)]
                                      for a, b, n in res["sized"]["main_web_zones"]]
    D["mass"] = mass_block(res["mass"])
    sm = res["summary"]
    D["summary"] = {"n_margins": sm["n_margins"], "n_requirements": sm["n_requirements"],
                    "ms_min": round(float(sm["ms_min"]), 3), "ms_min_id": sm["ms_min_id"],
                    "report": "out/structures.md, out/structures.json, docs/04_yapi_hesaplari.md"}
    return D


def layups_block() -> dict:
    return copy.deepcopy(LAYUPS)


def interface_checks(S: dict, D: dict) -> list:
    """The layout objects that carry structures-phase dimensions agree with structures.sizing / LAYUPS."""
    L = S["layout"]
    mem = {m_["id"]: m_ for m_ in L["chassis"]["members"]}
    fit = {f["id"]: f for f in L["chassis"]["fittings"]}
    out = []

    def chk(id_, text, ok):
        out.append({"id": id_, "text": text, "ok": bool(ok)})
    ak, akd = mem.get("M-AFTKEEL", {}), D["body"]["aft_keel"]
    sec = ak.get("section", {})
    chk("I-AFTKEEL", "layout M-AFTKEEL material / section = structures.sizing.body.aft_keel",
        ak.get("material") == akd["material"] and abs(float(sec.get("t", 0)) - akd["t_m"]) < 1e-9 and
        abs(float(sec.get("w", 0)) - akd["w_m"]) < 1e-9 and abs(float(sec.get("h", 0)) - akd["h_m"]) < 1e-9)
    wk = mem.get("M-WELLKEEL")
    chk("I-WELLKEEL", "layout M-WELLKEEL exists with the structures thickness",
        wk is not None and abs(float(wk.get("thickness", 0)) - D["fuel_bay"]["well_web_t_m"]) < 1e-9 and
        abs(float(wk["box"][0][1]) - D["fuel_bay"]["well_web_y_m"]) < 1e-6 and bool(wk.get("mirror")))
    chk("I-WELLROOF", "layout M-WELLROOF layup = structures.sizing.fuel_bay.floor_layup",
        mem.get("M-WELLROOF", {}).get("layup") == D["fuel_bay"]["floor_layup"])
    chk("I-CTBOX", "layout M-CTBOX section names the CT-box cover layup",
        D["wing"]["ct_box"]["cover_layup"] in str(mem.get("M-CTBOX", {}).get("section", {}).get("type", "")))
    mf = mem.get("M-MIDFLOOR", {})
    chk("I-MIDFLOOR", "layout M-MIDFLOOR names the two bonded cut-out edge stiffeners and has the tray cut-out",
        "two bonded" in str(mf.get("section", {}).get("type", "")) and bool(mf.get("cutout")) and
        int(D["body"]["midfloor_stiffeners"]) == 2)
    nd = fit.get("F-SPINDLE-NODE", {})
    chk("I-NODE", "layout F-SPINDLE-NODE: stub root bolts, firewall bolts, inboard bearing cylinder and outboard bearing "
        "station exist",
        {"stub root", "firewall"} <= {b_.get("group") for b_ in nd.get("bolts", [])} and "cylinder" in nd and
        "outboard_bearing_y" in nd.get("spindle", {}) and "F-SPINDLE-IN" not in fit)
    wj = L["chassis"]["wing_joint"]
    tg, fk = D["wing_joint"]["tongue"], D["wing_joint"]["fork"]
    tg_txt, fk_txt = str(wj["main_spar"]["tongue"].get("geometry", "")), str(wj["main_spar"]["fork"].get("geometry", ""))
    chk("I-TONGUE", "layout tongue: CFRP, flange and web thickness = structures.sizing.wing_joint.tongue",
        wj["main_spar"]["tongue"].get("material") == tg["flange_material"] and
        f"30 x {tg['flange_t_m'] * 1000:.0f} mm" in tg_txt and
        f"web {int(tg['web_plies']) * 0.20066:.0f} mm" in tg_txt and
        (not tg.get("flange_taper") or
         f"to {tg['flange_taper']['t_min_m'] * 1000:.0f} mm at the inner pin" in tg_txt))
    chk("I-FORK", "layout fork: CFRP prongs and pads = structures.sizing.wing_joint.fork",
        f"webs {int(fk['prong_plies']) * 0.20066:.1f} mm" in fk_txt and f"{int(fk['prong_plies'])} plies" in fk_txt and
        f"padded up to {fk['prong_pad_t_m'] * 1000:.0f} mm" in fk_txt and
        float(wj["main_spar"]["fork"].get("prong_t", 0)) == float(fk["prong_pad_t_m"]))
    sp_ = mem.get("M-SPINE", {}).get("section", {})
    chk("I-SPINE", "layout M-SPINE thickness = structures.sizing.parachute.spine_plies x ply",
        abs(float(sp_.get("t", 0)) - int(D["parachute"]["spine_plies"]) * 0.0002) < 2e-4 and
        f"{int(D['parachute']['spine_plies'])} plies" in str(sp_.get("type", "")))
    fr = fit.get("F-RISER-FWD", {}).get("lug", {})
    chk("I-RISER", "layout bridle U-lugs: shackle pin d and ear thickness = structures.sizing.parachute",
        abs(float(fr.get("bore", 0)) - D["parachute"]["shackle_pin_d_m"]) < 1e-9 and
        abs(float(fr.get("t_m", 0)) - D["parachute"]["lug_ear_t_m"]) < 1e-9 and
        abs(float(fr.get("e_m", 0)) - D["parachute"]["lug_e_m"]) < 1e-9)
    ud = S["materials"].get("cfrp_ud_mtm45_as4", {})
    chk("I-UD-BEARING", "materials.cfrp_ud_mtm45_as4 carries no bearing allowable (S1-05)", "Fbru" not in ud)
    for key, lay in LAYUPS.items():
        chk(f"I-LAYUP-{key}", f"spec.layups.{key} = structures LAYUPS[{key}]", S["layups"].get(key) == lay)
    return out


def fmt(x, nd: int = 3) -> str:
    """Compact number with a decimal comma (Turkish report)."""
    if x is None:
        return "–"
    if isinstance(x, str):
        return x
    x = float(x)
    if not math.isfinite(x):
        return "∞"
    a = abs(x)
    if a >= 1e4:
        t = f"{x:.0f}"
    elif a >= 100:
        t = f"{x:.1f}"
    elif a >= 10:
        t = f"{x:.2f}"
    else:
        t = f"{x:.{nd}g}" if a >= 1e-3 or a == 0 else f"{x:.2e}"
    return t.replace(".", ",")


GROUP_TR = {"wing": "Kanat", "wing joint": "Kanat dış panel birleşimi (y 0,70)", "CT box": "Orta kanat kutusu",
            "tail": "Kuyruk", "controls": "Kumanda yüzeyleri", "gear": "İniş takımı", "engine mount": "Motor bağlantısı",
            "parachute": "Paraşüt", "fuel bays": "Yakıt bölmeleri", "turret": "Taret asansörü ve kapakları",
            "transport": "Taşıma ve elleçleme", "body": "Gövde", "equipment": "Teçhizat tutma"}


REF_TR = [   # phrase translations of the reference column (longest first; formulas and citations are kept)
    ("the asymmetry is reacted by the roll inertia (the CT-box centre moment is the mean of both sides)",
     "asimetri yuvarlanma ataletiyle dengelenir (orta kutunun ortasındaki moment iki tarafın ortalamasıdır)"),
    ("covered: design-case root moment", "karşılanıyor: tasarım durumu kök momenti"),
    ("for pure bending; QI modulus", "saf eğilmede; QI modülü"),
    (" at x/c ", " x/c "),
    ("elastic bolt group (in-plane shear + torsion, tension + moment of the 15 mm shackle offset)",
     "elastik cıvata grubu (düzlem içi kesme + burulma, çekme + 15 mm kilit kaçıklığının momenti)"),
    ("taken by the kink fitting into the sandwich covers and the FS-MS web (detail design)",
     "kırık bağlantısıyla sandviç kapaklara ve FS-MS gövdesine aktarılır (detay tasarım)"),
    ("conservative: pad pressure against the core crushing strength of the adjacent sandwich",
     "muhafazakâr: yastık basıncı bitişik sandviçin çekirdek ezilme dayanımıyla karşılaştırıldı"),
    ("ultimate holding moment; the actuator is not relied upon to hold the closed door",
     "nihai tutma momenti; kapalı kapağı tutmak için eyleyiciye güvenilmez"),
    ("loads already x FoS (limit cases) and x 1.15 (bolts)", "yükler zaten x FoS (limit durumlar) ve x 1.15 (cıvatalar)"),
    ("the lock link (gear unit) is sized for this moment x 1.5 x 1.15",
     "kilit bağlantısı (takım ünitesi) bu moment x 1.5 x 1.15 için boyutlandırılır"),
    ("most aft ventral lug load x arm to FS3738", "en arka ventral kulak yükü x FS3738'e kol"),
    ("(local flange crippling not credited above yield)", "(akma üstünde yerel flanş buruşması hesaba katılmadı)"),
    ("the fastened skin (nutplates 25-32 mm) is not credited", "bağlı kaplama (25-32 mm somun plakaları) hesaba katılmadı"),
    ("fail-safe path at limit load (AC 20-107B practice)", "limit yükte emniyetli yedek yol (AC 20-107B uygulaması)"),
    ("(e/D >= 3, >= 40 % +-45 in the bolted zone)", "(e/D >= 3, cıvatalı bölgede >= %40 ±45)"),
    ("Nx from the section strain of the surface", "Nx yüzeyin kesit birim şekil değiştirmesinden"),
    ("fitting factor 1.15 (member away from the bore)", "bağlantı katsayısı 1.15 (delikten uzak eleman)"),
    ("local 7075 doubler at the fitting not credited", "bağlantıdaki yerel 7075 takviye hesaba katılmadı"),
    ("Euler wide column with shear correction", "kesme düzeltmeli geniş kolon (Euler)"),
    ("tray / bracket / strap design load x 1.15 fitting", "tepsi / dirsek / kayış tasarım yükü x 1.15 bağlantı"),
    ("(Limbach damper shock mount data not published)", "(Limbach sönümleyici verisi yayımlanmamış)"),
    ("(catalogue rating not in the research data)", "(katalog değeri araştırma verisinde yok)"),
    ("simply supported between the well wall and FS-GEAR", "kuyu duvarı ile FS-GEAR arasında basit mesnetli"),
    ("direct-stiffness space truss", "doğrudan rijitlik yöntemiyle uzay kafes"),
    ("moment about the trunnion axis / lock radius", "mafsal ekseni etrafında moment / kilit yarıçapı"),
    ("(7075-T6 sheet values as bar estimate)", "(çubuk için 7075-T6 sac değerleri, tahmin)"),
    ("(sheet values as bar estimate)", "(çubuk için sac değerleri, tahmin)"),
    ("(stabilator removed for transport)", "(stabilatör taşıma için sökülür)"),
    ("longerons only (skins = shear webs)", "yalnız uzun kirişler (kaplamalar = kesme gövdeleri)"),
    ("gust lock not required if the margin holds", "pay sağlanırsa rüzgâr kilidi gerekmez"),
    ("(well forward wall to FS-GEAR)", "(kuyu ön duvarından FS-GEAR'e)"),
    ("beam over", "kiriş, açıklık"),
    ("QI bearing on the insert flange diameter 9 mm", "burç flanşı çapında (9 mm) QI ezilme"),
    ("on the bush OD (bush wall 2 mm)", "burç dış çapında (burç duvarı 2 mm)"),
    ("linear bearing pressure", "doğrusal ezilme basıncı"),
    ("long-plate shear buckling", "uzun levha kesme burkulması"),
    ("CLT bending stiffness", "CLT eğilme rijitliği"),
    ("sandwich shear correction", "sandviç kesme düzeltmesi"),
    ("sandwich correction", "sandviç düzeltmesi"),
    ("ply allowables B-basis ETW", "kat izin verilen değerleri B-tabanı ETW"),
    ("materials Fcu B-basis ETW", "malzeme Fcu B-tabanı ETW"),
    ("hinge bearing factor", "menteşe ezilme katsayısı"),
    ("push-pull joint factor", "itme-çekme bağlantı katsayısı"),
    ("lap shear at 82 degC", "82 °C'de bindirme kesmesi"),
    ("no credit beyond 30 t", "30 t ötesi hesaba katılmaz"),
    ("(zero at the inner pin)", "(iç pimde sıfır)"),
    ("(rotating joint)", "(dönen bağlantı)"),
    ("(edge band with fastener holes)", "(bağlantı delikli kenar bandı)"),
    ("(hinge-bracket holes)", "(menteşe dirseği delikleri)"),
    ("(3 lugs on the keel line)", "(omurga hattında 3 kulak)"),
    ("(core minimum G)", "(çekirdek minimum G)"),
    ("(CS-LUAS.395 analogue)", "(CS-LUAS.395 benzeri)"),
    ("requirement for the gear unit EMA", "takım ünitesi EMA gereksinimi"),
    ("procurement requirement", "tedarik gereksinimi"),
    ("C0 >= this value", "C0 >= bu değer"),
    ("peak bolt 1.5 x mean", "en yüklü cıvata 1.5 x ortalama"),
    ("elastic bolt circle", "elastik cıvata dairesi"),
    ("elastic bolt group", "elastik cıvata grubu"),
    ("elastic bending", "elastik eğilme"),
    ("elastic,", "elastik,"),
    ("tension Rm A_s, shear 0.6 Rm A_s", "çekme Rm A_s, kesme 0.6 Rm A_s"),
    ("simplified spline shear", "basitleştirilmiş kama kesmesi"),
    ("simply supported over", "basit mesnetli, açıklık"),
    ("sandwich strip", "sandviç şerit"),
    ("Bruhn D1 trend", "Bruhn D1 eğilimi"),
    ("web depth", "gövde derinliği"),
    ("to the chine longerons", "kenar uzun kirişlerine kadar"),
    ("rail lip, estimate geometry", "ray dudağı, tahmini geometri"),
    ("sandwich skins", "sandviç kaplamalar"),
    ("at ultimate", "nihai yükte"),
    ("on the shank", "gövde (shank) kesitinde"),
    ("on A_s", "A_s üzerinde"),
    ("pinned ends", "mafsallı uçlar"),
    ("long plate", "uzun levha"),
    ("near-weld", "kaynak yakını"),
    ("frequent assembly", "sık sökme"),
    ("bearing factor", "ezilme katsayısı"),
    ("rated torque", "anma torku"),
    ("governing", "belirleyici"),
    ("annealed", "tavlanmış"),
    ("minimum values", "minimum değerler"),
    ("QI bearing 2 % offset ETW", "QI ezilme (%2 öteleme) ETW"),
    ("QI bearing ETW", "QI ezilme ETW"),
    ("bearing 2.0", "ezilme 2.0"),
    ("(A-basis)", "(A-tabanı)"),
    ("B-basis", "B-tabanı"),
    ("depth", "derinlik"),
    ("mass ", "kütle "),
]


def ref_tr(s: str) -> str:
    for a, b in REF_TR:
        s = s.replace(a, b)
    return s


def dec_text(s: str) -> str:
    """Decimal points -> commas in the numbers of a Turkish text (words that start with a letter, e.g. clause numbers
    'UL13.1.2', 'CS-LUAS.349', are kept)."""
    out = []
    for wd in s.split(" "):
        core = wd.lstrip("([")
        if core and (core[0].isalpha() or re.fullmatch(r"(12\.9|10\.9|8\.8)[,;:)]*", core) or
                     core.startswith("43.13")):          # clause numbers, bolt property classes, AC 43.13-1B
            out.append(wd)
        else:
            out.append(re.sub(r"(?<=\d)\.(?=\d)", ",", wd))
    return " ".join(out)


def to_json(res: dict, block: dict, inter: list) -> dict:
    def clean(o):
        if isinstance(o, dict):
            return {str(k): clean(v) for k, v in o.items() if k not in ("L", "els", "tp")}
        if isinstance(o, (list, tuple)):
            return [clean(v) for v in o]
        if isinstance(o, np.ndarray):
            return clean(o.tolist())
        if isinstance(o, (np.floating, float)):
            v = float(o)
            return v if math.isfinite(v) else None
        if isinstance(o, (np.integer,)):
            return int(o)
        return o
    keep = ("flight_envelope", "wing_joint", "ct_box", "tail", "controls", "gear", "engine_mount", "parachute",
            "fuel_bays", "turret", "transport", "body", "frames", "equipment")
    return clean({"generated_by": "python3 -m ucav250.analysis.structures", "summary": res["summary"],
                  "sized": res["sized"], "loads": {k: res[k] for k in keep}, "wing_stations": res["wing"]["stations"],
                  "rows": res["rows"], "mass_tally": res["mass"], "structures_sizing": block,
                  "interfaces": inter,
                  "open_items": [{"en": a, "tr": b} for a, b in open_items(res)]})


def to_markdown(c: Ctx, res: dict, block: dict, inter: list, budget: dict | None = None) -> str:
    rows = res["rows"]
    sm = res["summary"]
    w = []
    ms_min_txt = f"{sm['ms_min']:.3f}".replace(".", ",")
    w.append("# YK-250 HANÇER — yapı hesapları: emniyet payı tablosu\n")
    w.append("Üreten: `python3 -m ucav250.analysis.structures` (el hesapları, `analysis/structlib.py`); yöntem ve "
             "açıklamalar: `docs/04_yapi_hesaplari.md`. Uygulanan değer LİMİT yük/gerilme/birim şekil değiştirme "
             "(yalnız nihai durumlarda nihai), izin verilen değer NİHAİ tasarım değeridir; "
             "**MS = izin verilen / (uygulanan × toplam katsayı) − 1**.\n")
    w.append(f"- Satır sayısı: {sm['n_rows']} ({sm['n_margins']} emniyet payı, {sm['n_requirements']} gereksinim / "
             f"bilgi satırı)\n- En küçük MS: **{ms_min_txt}** ({sm['ms_min_id']}); negatif MS: "
             f"{sm['n_negative']}\n- Arayüz kontrolleri (yerleşim ↔ yapı boyutları): "
             f"{sum(1 for i in inter if i['ok'])}/{len(inter)} uygun\n")
    f = c.f
    w.append("## Katsayılar\n")
    w.append("| Katsayı | Değer | Kaynak |\n|---|---|---|")
    w.append(f"| Emniyet katsayısı (FoS) | {fmt(f['fos'])} | CS-LUAS.303, STANAG 4703 UL2.3 |")
    w.append(f"| Bağlantı (fitting) katsayısı | {fmt(f['fit'])} | CS-LUAS.625 |")
    w.append(f"| Mafsallı / dönen bağlantı ezilme katsayısı | {fmt(f['bear'])} | STANAG 4703 UL2.4 |")
    w.append(f"| Sık sökülüp takılan bağlantı katsayısı | {fmt(f['fa'])} | STANAG 4703 UL2.4 |")
    w.append(f"| Kompozit özel katsayısı (B-tabanı ETW ile) | {fmt(f['comp'])} | AMC LUAS.619 |")
    w.append(f"| Tek yük yollu kompozit A-tabanı tahmini | {fmt(f['a_basis'])} × B-tabanı | CS-LUAS.613(b) |")
    w.append(f"| Menteşe ezilmesi toplam | {fmt(f['hinge_total'])} | CS-LUAS.657 |")
    w.append(f"| İtme-çekme bağlantısı toplam | {fmt(f['pushpull_total'])} | CS-LUAS.693 |")
    w.append("| Birleştirme kuralı | FoS × en büyük özel katsayı | CS-LUAS.619 |")
    w.append("| Hasar toleransı sınırları (nihai yükte) | 3000 µε bası (> 2 mm), 2600 µε bası (sandviç / ≤ 2 mm), "
             "5000 µε çekme, 5200 µε kesme | STANAG 4703 UL13.1.2 |\n")
    # load cases
    cases = {}
    for r in rows:
        k = r["case_tr"]
        if k not in cases:
            cases[k] = f"YD-{len(cases) + 1:02d}"
    w.append("## Emniyet payı tablosu\n")
    w.append("Yük durumu kodları (YD-xx) tablonun altında açıklanmıştır. Birimler: N, N m, MPa, N/mm (kesme akışı / "
             "doğrusal yük), µε; 'yük çarpanı' satırlarında uygulanan = 1 (katsayılı yük) ve izin verilen = kritik yük "
             "çarpanıdır.\n")
    groups = []
    for r in rows:
        if r["group"] not in groups:
            groups.append(r["group"])
    for g in groups:
        w.append(f"### {GROUP_TR.get(g, g)}\n")
        w.append("| No | Eleman / kontrol | Yük durumu | Uygulanan | İzin verilen | Birim | Katsayılar | MS | Referans |")
        w.append("|---|---|---|---|---|---|---|---|---|")
        for r in rows:
            if r["group"] != g:
                continue
            if r["ms"] is None:
                ms_txt = "gereksinim"
            else:
                ms_txt = f"{r['ms']:.3f}".replace(".", ",") if r["ms"] < 100 else "> 100"
                if r["ms"] < 0:
                    ms_txt = f"**{ms_txt}**"
            ftxt = (r["factor_text"].replace("FoS", "FoS").replace("ultimate-only", "yalnız nihai")
                    .replace("fitting", "bağlantı").replace("bearing", "ezilme").replace("frequent assembly",
                                                                                      "sık sökme")
                    .replace("composite", "kompozit").replace("total", "toplam")
                    .replace("load multiplier on the factored load", "katsayılı yükte çarpan")
                    .replace("limit vs rated (functional)", "limit / anma (işlevsel)")
                    .replace("(in the multiplier)", "(çarpan içinde)").replace("ultimate", "nihai")
                    .replace("functional", "işlevsel"))
            unit = r["unit"].replace("load multiplier", "yük çarpanı")
            w.append(f"| {r['id']} | {dec_text(r['member_tr'])} | {cases[r['case_tr']]} | {fmt(r['applied'])} | "
                     f"{fmt(r['allowable'])} | {unit} | {dec_text(ftxt)} | {ms_txt} | {dec_text(ref_tr(r['ref']))} |")
        w.append("")
    w.append("## Yük durumları\n")
    w.append("| Kod | Yük durumu |\n|---|---|")
    for k, v in cases.items():
        w.append(f"| {v} | {dec_text(k)} |")
    w.append("")
    # sized dimensions
    D = block
    w.append("## Boyutlandırılan ölçüler (spec.structures.sizing)\n")
    w.append("Ana kiriş başlıkları (UD MTM45-1/AS4, kat kalınlığı "
             f"{fmt(ply_props(c, 'cfrp_ud_mtm45_as4')['t'] * 1000, 4)} mm; gövde/eldiven genişliği "
             f"{fmt(D['wing']['main_cap']['width_body_glove_m'] * 1000)} mm, dış panel "
             f"{fmt(D['wing']['main_cap']['width_outer_m'] * 1000)} mm):\n")
    w.append("| y aralığı (m) | Kat sayısı | Kalınlık (mm) |\n|---|---|---|")
    tud = ply_props(c, "cfrp_ud_mtm45_as4")["t"]
    for a, b, n in D["wing"]["main_cap"]["zones"]:
        w.append(f"| {fmt(a)} – {fmt(b)} | {n} | {fmt(n * tud * 1000)} |")
    w.append("\nAna kiriş gövdesi (±45 PW, dış panel):\n")
    w.append("| y aralığı (m) | Kat sayısı |\n|---|---|")
    for a, b, n in D["wing"]["main_web"]["zones"]:
        w.append(f"| {fmt(a)} – {fmt(b)} | {n} |")
    w.append("")
    w.append("Yapı aşamasının sahiplendiği / revize ettiği lamine dizilimleri (spec.layups):\n")
    w.append("| Anahtar | Dış yüz | Çekirdek | İç yüz | Kullanım |\n|---|---|---|---|---|")
    for k, v in LAYUPS.items():
        w.append(f"| {k} | {v['plies'][0][1]} | {v['core']} {fmt(v['core_t'] * 1000)} mm | {v['inner_plies'][0][1]} | "
                 f"{v['use'][:140]}… |")
    w.append("")
    # mass
    mt = res["mass"]
    w.append("## Kütle: aşağıdan yukarı / kavramsal model payı (büyüme payı öncesi)\n")
    w.append("| Grup | Aşağıdan yukarı (kg) | Model payı (kg) | Fark (kg) |\n|---|---|---|---|")
    names = {"A_wing_primary": "A kanat birincil yapısı (başlıklar, gövdeler, arka kiriş, dış birleşim, üst kutu "
                               "kaplama çekirdeği)",
             "B_carry_through": "B orta kutu + çatal + pimler + sürükleme pimi yuvaları",
             "C_chassis_changes": "C şasi eklemeleri (kuyu tavanı dizilimi, M-WELLKEEL, orta taban takviyesi, arka "
                                  "omurga)",
             "D_spindles": "D stabilatör milleri, yatak yuvaları, kök yuvaları"}
    for k, nm in names.items():
        v = mt[k]
        w.append(f"| {nm} | {fmt(v['bottom_up_kg'])} | {fmt(v['allocation_kg'])} | {fmt(v['delta_kg'])} |")
    w.append(f"| **Toplam** | | | **{fmt(mt['total_delta_base_kg'])}** (büyüme payıyla "
             f"{fmt(mt['total_delta_with_growth_kg'])}) |\n")
    if budget:
        w.append("Grup tavanları (mass.budget) ve sizing tahminleri (büyüme payı dahil):\n")
        w.append("| Grup | Tavan (kg) | Tahmin (kg) | Pay (kg) |\n|---|---|---|---|")
        for g, b in budget.items():
            w.append(f"| {g} | {fmt(b['target_kg'])} | {fmt(b.get('estimate_kg'))} | "
                     f"{fmt(None if b.get('estimate_kg') is None else b['target_kg'] - b['estimate_kg'])} |")
        w.append("")
    w.append("## Arayüz kontrolleri\n")
    w.append("| No | Kontrol | Sonuç |\n|---|---|---|")
    for i in inter:
        w.append(f"| {i['id']} | {i['text']} | {'uygun' if i['ok'] else '**UYGUN DEĞİL**'} |")
    w.append("\n## Açık konular\n")
    for a, b in open_items(res):
        w.append(f"- {b}")
    w.append("")
    return "\n".join(w)


def budget_table(S: dict) -> dict:
    """mass.budget ceilings with the group estimates of the spec mass items (incl. the growth allowance)."""
    est = {}
    for it in S["mass"]["items"]:
        est[it["group"]] = est.get(it["group"], 0.0) + float(it["mass_kg"])
    return {g: {"target_kg": float(b["target_kg"]), "estimate_kg": est.get(g)} for g, b in S["mass"]["budget"].items()}


def compute(S: dict | None = None, authoring: bool = True) -> tuple:
    """(context, results, spec block, interface checks). authoring=True: DESIGN + LAYUPS (the code), sized here;
    authoring=False: the spec's structures.sizing (sized zones read from it) and the spec's layups."""
    S = S if S is not None else SPEC.load()
    if authoring:
        c = Ctx(S, design=DESIGN)
        res = run_all(c)
    else:
        if not S["structures"].get("sizing"):
            raise RuntimeError("spec.structures.sizing missing: run --update-spec")
        c = Ctx(S)
        W = c.D["wing"]
        res = run_all(c, {"main_cap_zones": [list(z) for z in W["main_cap"]["zones"]],
                          "main_web_zones": [list(z) for z in W["main_web"]["zones"]]})
    block = sizing_block(c, res)
    inter = interface_checks(S, block)
    return c, res, block, inter


def write_outputs(c: Ctx, res: dict, block: dict, inter: list, out_dir: Path = OUT_DIR) -> None:
    out_dir.mkdir(parents=True, exist_ok=True)
    (out_dir / "structures.json").write_text(json.dumps(to_json(res, block, inter), indent=1, ensure_ascii=False) + "\n",
                                             encoding="utf-8")
    (out_dir / "structures.md").write_text(to_markdown(c, res, block, inter, budget_table(c.S)), encoding="utf-8")


def stale_items(S: dict, block: dict) -> list:
    """Differences between the spec (structures.sizing, layups) and the regenerated values."""
    bad = []
    cur = S["structures"].get("sizing")
    if cur is None:
        return ["structures.sizing missing"]
    for k in sorted(set(cur) | set(block)):
        if cur.get(k) != block.get(k):
            bad.append(f"structures.sizing.{k}")
    for k, v in LAYUPS.items():
        if S["layups"].get(k) != v:
            bad.append(f"layups.{k}")
    return bad


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--check", action="store_true", help="margins >= 0, spec block up to date, interfaces consistent")
    ap.add_argument("--update-spec", action="store_true", help="write structures.sizing and the owned layups to spec.yaml")
    ap.add_argument("--out", default=None, help="output directory (default ucav250/out)")
    a = ap.parse_args(argv)
    S = SPEC.load()
    if a.check:
        ok = True
        c, res, block, inter = compute(copy.deepcopy(S), authoring=False)
        neg = [r for r in res["rows"] if r["ms"] is not None and r["ms"] < 0.0]
        for r in neg:
            print(f"  [MS<0] {r['id']}: {r['member']} MS {r['ms']:.3f}")
        ok &= not neg
        c2, res2, block2, inter2 = compute(copy.deepcopy(S), authoring=True)
        stale = stale_items(S, block2)
        for b in stale:
            print(f"  [stale] {b} differs from the regenerated value (run --update-spec)")
        ok &= not stale
        badi = [i for i in inter if not i["ok"]]
        for i in badi:
            print(f"  [interface] {i['id']}: {i['text']}")
        ok &= not badi
        sm = res["summary"]
        print(f"[structures] {sm['n_margins']} margins (min {sm['ms_min']:.3f} {sm['ms_min_id']}), "
              f"{sm['n_requirements']} requirement rows, negative {len(neg)}, stale {len(stale)}, interfaces "
              f"{len(inter) - len(badi)}/{len(inter)} -> {'OK' if ok else 'FAIL'}")
        return 0 if ok else 1
    c, res, block, inter = compute(copy.deepcopy(S), authoring=True)
    if a.update_spec:
        S2 = copy.deepcopy(S)
        S2["structures"]["sizing"] = block
        S2["layups"].update(layups_block())
        Z.write_spec(S2, SPEC.SPEC_PATH)
        print(f"[structures] written spec.structures.sizing and layups {list(LAYUPS)} to {SPEC.SPEC_PATH}")
        S = SPEC.reload()
        c, res, block, inter = compute(copy.deepcopy(S), authoring=True)
    write_outputs(c, res, block, inter, Path(a.out) if a.out else OUT_DIR)
    sm = res["summary"]
    print(f"[structures] {sm['n_margins']} margins, min MS {sm['ms_min']:.3f} ({sm['ms_min_id']}), negative "
          f"{sm['n_negative']}, {sm['n_requirements']} requirement rows; mass delta vs concept allocation "
          f"{res['mass']['total_delta_base_kg']:+.3f} kg (base); interfaces {sum(i['ok'] for i in inter)}/{len(inter)}; "
          "outputs written")
    return 0


if __name__ == "__main__":
    sys.exit(main())
