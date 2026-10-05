# ucav250 (YK-250) — engineering architecture and module contract

This file is the contract every contributor (human or agent) follows. User-facing documents (README, reports,
assembly guide) are Turkish; code, identifiers and this file are English.

## 1. Principles

1. `spec.yaml` is the **single source of truth**. Every dimension, station, interface point, material, layup,
   process rule, mass item and requirement lives there (SI units, metres). Code never hard-codes a number that another
   module also needs; module-private detailing constants are allowed only if documented next to the code and
   reflected in the BOM/report.
2. **Interfaces before details.** Anything two modules share (frame stations, attach points, pin diameters, bolt
   patterns, panel split lines, bay boundaries, keep-out envelopes) is defined in `spec.yaml → layout` and read by
   both sides. A producer must not read another producer's geometry to position its own parts.
3. **Every part is a manufacturable part**: it names its material, process and governing thickness/layup, respects
   the process rules in `spec.yaml → processes`, and is either purchased (with vendor + datasheet mass) or made.
4. **Nothing broken, nothing floating, nothing colliding.** All meshes are closed 2-manifolds with outward normals
   and no self-intersections; every part is attached to something (parent/fasteners/bond) through a real load path;
   no two parts overlap in volume at rest or anywhere in any joint's range.

## 2. Folder layout and ownership

```
ucav250/
  spec.yaml                 single source of truth
  ARCHITECTURE.md           this contract
  README.md                 Turkish overview (owner-facing)
  requirements.txt, setup_env.sh
  core/                     framework (no aircraft knowledge)
    geom.py                 mesh kernel (closed meshes, extrude/revolve/loft/shell, booleans, mass props, checks)
    parts.py                Part / Joint / Fastener / Registry data model
    spec.py                 spec loader
    assemble.py             build_registry(): runs producer modules in order
  design/                   producers: register(reg, spec) -> None  (one owner each)
    oml.py                  outer mould line evaluators (fuselage, wing, tail, cowl) — shared math, no parts
    actuation.py            shared actuator/horn/linkage/hinge generators — shared math, no parts
    chassis.py              fuselage frames, keel/longerons, carry-through, attach fittings, bay structure
    wing.py                 wing panels: spar, ribs, skins, control surfaces + hinges + actuation, wing joints
    tail.py                 tail surfaces + boom/attachment, control surfaces + hinges + actuation
    shell.py                fuselage skins, removable panels, cowlings, fairings, hatches
    propulsion.py           engine model, engine mount, propeller, spinner, exhaust, cooling ducts/baffles
    fuel.py                 tanks, lines, vents, filler, pump/filter, tank mounts
    gear.py                 landing gear legs, wheels, brakes, steering, attachments (+ doors if retractable)
    systems.py              avionics trays, power, antennas, pitot, lights, parachute, harness conduits
    payload.py              EO/IR gimbal, mount, payload bay equipment
    hardware.py             LAST: fastener geometry for every Fastener record (bolts, nuts, inserts, camlocks)
  analysis/
    aero.py                 airfoil polars (NeuralFoil), lifting line, drag build-up
    sizing.py               sizing study + checks -> out/sizing.md (--check exits 1 on violations)
    structures.py           hand calculations -> out/structures.md
    mass.py                 mass properties from the registry, CG/inertia, budget cross-check
    checks.py               manifold, interference (static + swept), thickness, edge distance, attachment graph
  outputs/
    bom.py                  out/bom.csv
    drawings.py             out/ga_3view.svg + .pdf (dimensioned general arrangement)
    assembly_guide.py       out/montaj_kilavuzu.md
  blender/
    build.py                builds the scene from the registry, rigs, exploded/assembly states, previews
  data/                     research YAML, airfoil .dat files, datasheets extracts
  docs/                     Turkish reports (research, trade study, sizing, structures, ...)
  out/                      generated outputs (never edited by hand)
tests/test_ucav250_*.py     unit + integration tests (repo-level tests/ folder)
```

Run everything from the repo root: `python3 -m ucav250.analysis.sizing --check`, `python3 -m ucav250.blender.build`.

## 3. Frames and units

* Units: SI. Lengths in metres in code and spec (mm only in text and in keys ending `_mm`). Angles in degrees in
  the spec (keys ending `_deg`), radians in code.
* Aircraft frame = Blender world frame: **X = FS** (fuselage station, positive aft, 0 at the nose tip),
  **Y = BL** (butt line, positive starboard/right), **Z = WL** (water line, positive up; 0 = fuselage datum line
  defined in `spec.yaml → layout.datum`). Right-handed.
* Symmetric parts are modelled on the starboard side (Y > 0, suffix `-R`) and mirrored with
  `core.parts.mirror_part` (suffix `-L`). Centre-line parts have no suffix.
* The aircraft sits on its gear on the ground plane `z = layout.ground_z` (static, MTOW).

## 4. Registry contract (`core/parts.py`)

* `Part`: unique `id` (`YK250-<GROUP CODE>-<NNN>[-L|-R]`), `name` (EN), `name_tr` (TR), `group`, `material`,
  `process`, `thickness` or `layup`, `mesh_fn` (lazy), `purchased`/`vendor`/`mass_kg` for bought items, `joint`
  if it moves, `parent` (the part it is mounted on), `step` (assembly step), `explode` (offset), `fasteners`,
  `contacts` (intended touching neighbours), `outline` (mid-surface boundary polylines for edge-distance checks),
  `color` (display colour key).
* `Joint`: revolute/prismatic, origin + axis in the REST pose, range `[lo, hi]`, `rest`, Blender control property
  `prop` and `scale`, optional `parent` joint, optional coupled `expr`.
* `Fastener`: standard designation (`spec`), kind, diameter, length, head bearing point, insertion axis, joined part
  ids (head → nut order), nut/insert/nutplate designation, torque, assembly step.
* Consumables (fuel contents) are parts with `process: consumable`: mass from their volume x density, scaled by
  the loading case's fuel fraction; excluded from the BOM, interference and thickness checks.
* Part-number groups: CH chassis, SH shell, WG wing, TL tail, FC flight controls, PR propulsion, FU fuel, LG gear,
  SY systems, PL payload, HW hardware. Number ranges per module are allocated in `spec.yaml → layout.part_numbers`.

## 5. Geometry rules

* Use `core.geom` generators (extrude, revolve, loft, shell_from_grid, sweep_circle, box/cylinder/tube, booleans).
  Every returned mesh must pass `Mesh.check(self_intersect=True)["ok"]`.
* Model real features that matter for fit and function: holes for every fastener (clearance per ISO 273 medium,
  e.g. M5 → Ø5.5), flanges where panels fasten, hinge lugs and bores, pin clearances (ISO 286 fits), cut-outs for
  pass-throughs. Cosmetic micro-detail is not required.
* Boolean unions must OVERLAP (>= 0.1 mm), never merely touch: manifold3d keeps face-touching solids as separate
  shells, which the self-intersection check then reports. Extend joined features into each other.
* Mesh density: enough to represent the shape within ~0.5 mm (OML surfaces) and ~0.2 mm (fittings). Avoid needless
  millions of triangles; the whole aircraft should stay below ~3 M triangles.
* Thin parts: thickness from the layup/process (never below `processes[*].min_thickness`).
* Clearances (minimum gaps, from `spec.yaml → layout.clearances`): moving vs fixed parts in every pose, propeller
  tip vs structure, hot parts vs composites, tyre vs bay. Touching is allowed only between parts that list each other
  in `contacts` (bonded/fastened faces) and then volume overlap must still be below 1 mm^3.

## 6. Analysis and checks (what "done" means)

`analysis/checks.py` (and the integration tests) must report zero violations for:

1. every part mesh closed/manifold/outward, no self-intersections;
2. no volume interference > 1 mm^3 between any two parts at rest;
3. no interference for any joint swept over its full range (≥ 9 samples incl. both ends), including coupled joints
   (gear + doors sequence);
4. minimum clearances from `layout.clearances`;
5. wall/laminate thickness ≥ process minimum (declared and sampled by ray casting);
6. fastener edge distance ≥ 2.0·D in metal and ≥ 2.5·D in composite for every joined part; fastener pitch rules;
7. attachment graph: every part reaches the primary structure through parents/fasteners/bonds (nothing floats);
8. mass properties from geometry + datasheets agree with the mass budget (per group tolerance in spec), CG inside
   `stability.cg_range` for all loading cases, static margin within limits;
9. sizing checks (`analysis/sizing.py --check`) and structures margins of safety ≥ 0 (`analysis/structures.py`).

## 7. Blender contract (`blender/build.py`)

* One collection per group (`YK250_<Group>`), plus `YK250_Rig` (joint empties, control object).
* Object name = part id; mesh data name = part id; custom properties: `name_tr`, `material`, `process`,
  `thickness_mm`, `mass_kg`, `step`, `vendor`.
* Moving parts are parented to joint empties `J_<joint>` placed at the joint origin with local X = joint axis;
  rotation is driven by **simple-expression drivers** from the control object `YK250_Root` custom properties
  (`gear`, `flap_deg`, `aileron_deg`, `elevator_deg`, `rudder_deg` / ruddervator mix, `prop_rpm`, ...).
* Exploded view: `explode` property (0..1) drives `delta_location = explode * Part.explode`.
* Assembly steps: `step` property; a part is visible when `step >= Part.step`.
* Previews use the Workbench engine (Mesa EGL headless; `setup_env.sh` installs it). No photorealistic renders.

## 8. Outputs (`out/`)

`sizing.md`, `structures.md`, `mass.md`, `checks.md`, `bom.csv`, `ga_3view.svg` / `.pdf`, `montaj_kilavuzu.md`,
`yk250.blend`, `yk250.glb`, `previews/*.png`. All generated, reproducible from the spec.
