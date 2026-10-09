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
    layout_check.py         layout-phase checks of spec.layout / spec.assembly -> out/layout.md (--write; --check is read-only)
    structures.py           hand calculations (CS-LUAS / STANAG 4703 load cases, margins of safety) ->
                            out/structures.md/.json; owns spec.structures.sizing (--update-spec, --check)
    mass.py                 mass properties from the registry, CG/inertia, budget cross-check
    checks.py               manifold, interference (static + swept), thickness, edge distance, attachment graph
  outputs/
    bom.py                  out/bom.csv
    drawings.py             out/ga_3view.svg + .pdf (dimensioned general arrangement)
    assembly_guide.py       out/montaj_kilavuzu.md
  blender/
    build.py                builds the scene from the registry, rigs, exploded/assembly states, previews
  layout_build/             generator of spec.layout / spec.assembly (authored interface values; build --check/--write)
  data/                     research YAML, airfoil .dat files, datasheets extracts
  docs/                     Turkish reports (research, trade study, sizing, structures, ...)
  out/                      generated outputs (never edited by hand)
tests/test_ucav250_*.py     unit + integration tests (repo-level tests/ folder)
```

Run everything from the repo root: `python3 -m ucav250.analysis.sizing --check`, `python3 -m ucav250.blender.build`.
Structures workflow after a change: `layout_build --write` -> `structures --update-spec` -> `sizing --update-spec` (masses, closure; the wing moves only when |dx_c4| >= `sizing.WING_DEADBAND` 0.4 mm) -> `layout_build --write` (positions; last, because sizing stores the mechanism values to 6 significant digits) -> `layout_build --check`, `structures --check`, `sizing --check`, `layout_check --check` (all read-only).

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
* Fastened joints are made with `design/joints.py` (`bolt`, `bolt_through`, `pin`, `quarter_turn`): one call creates the
  `Fastener` with a standard length and grip, cuts the clearance hole in every clamped part (`Part.add_hole`; holes
  are subtracted lazily, so a later module may drill a part registered earlier), and the insert bore / tap-drill
  where used. `checks.py` measures edge distance and pierce on the geometry when a part has no `outline`.
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

## 9. `spec.yaml` schema (v1 written by the sizing phase, `layout`/`assembly` filled by later phases)

All lengths in metres, masses in kg, angles in degrees (`_deg`), powers in W. Every number that is not a pure design
choice carries a `source` (research file key or URL) or is produced by `analysis/sizing.py` (then sizing `--check`
verifies the spec value against the computed one within a stated tolerance).

| key | content |
|---|---|
| `meta` | `name`, `project`, `revision`, `date`, `description_tr`, `scope_tr` (civil EO/IR; no weapons, hardpoints, pylons or release mechanisms) |
| `requirements` | list of `{id, text_tr, metric, op, value, unit, source}`; `metric` is a key of `out/sizing.json` (or `mass.*`, `structures.*`) |
| `mission` | payload, altitudes, endurance/range targets, runway, climb, ceiling, `profile` segments for the fuel calculation |
| `engine` | model/vendor, displacement, max/continuous power, rpm, dry and installed masses (itemised), BSFC curve `[[power_fraction, g_per_kWh], ...]`, fuel, generator, envelope and mount pattern (all sourced) |
| `propeller` | type, blades, diameter, pitch, mass, position (pusher/tractor), efficiencies, static thrust, tip speed |
| `configuration` | chosen layout (`propulsion`, `tail`, `wing_position`, `gear`, `fuselage_style`, `transport_breakdown`) + `reasons_tr` and trade-study reference |
| `wing` | `span`, `area` (reference, trapezoid continued to the centre line), `aspect_ratio`, `taper`, `sweep_c4_deg`, `dihedral_deg`, `incidence_deg`, `washout_deg`, airfoils, `sections` (LiftingSurface format, starboard, root at the fuselage side to the tip), `mac`, `mac_le_x`, `mac_y`, `controls` (`aileron`/`flap`: `eta0`, `eta1` span coordinates, `xc_hinge`, `range_deg`) |
| `tail` | `type`; `surfaces: {name: {sections, mirror, area, span, controls}}`; `volume_h`, `volume_v`, arms |
| `fuselage` | `length`, `width_max`, `height_max`, `stations` (n × 6 or 7: x, w, h, zc, n_top, n_bot[, top_frac]) |
| `landing_gear` | type, positions, track, wheelbase, tyres, static load split, tip-back/turnover angles, propeller ground clearance, tail-strike angle |
| `aero` | drag build-up items, `cd0`, `e`, `k`, `clmax_clean/_to/_ld`, `ld_max`, polar references |
| `mass` | `mtow_kg`, `empty_kg`, `fuel_kg`, `payload_kg`, `budget: {group: {target_kg, tol_kg}}` (groups = part groups), `cases` (`name`, `fuel_fraction`, `payload`) |
| `stability` | `mac`, `mac_le_x`, `np_x`, `cg_design` [x, y, z], `cg_range_x` [fwd, aft], `static_margin_range` [min, max] |
| `performance` | reference copy of the sizing results (speeds, L/D, endurance, range, climb, ceiling, field lengths, payload) |
| `structures` | limit load factors (+3.8 / −1.5), FoS 1.5, fitting factor, design speeds (VA, VC, VD), gust velocities, gear sink rate, standards references; `sizing` (structures phase, written by `analysis/structures.py --update-spec`): sized dimensions, spar-cap / web ply zones, joint, fitting and member sizes, and the `mass` block that `analysis/sizing.py` reads in place of the concept-model wing, carry-through, keel/floor and spindle terms |
| `materials` | `{key: {name, kind (metal/composite/polymer/core/elastomer), density, E, Ftu, Fty, Fsu, Fbru, ply_t (plies), Tg_C/HDT_C, source}}` |
| `layups` | `{key: {plies: [[material, angle_deg, count]], core, core_t, adhesive_areal, use}}` |
| `processes` | `{key: {name, name_tr, min_thickness, tolerance (ISO 2768-m), draft_deg, min_bend_radius_t, edge_distance_D, notes}}` |
| `display` | `colors` per group, `drawing.dimensions`, `preview_states` |
| `layout` | datum, `ground_z`, `root_part`, stations/frames, interface points, keep-out envelopes, `part_numbers` ranges, `clearances` rules |
| `assembly` | `general` notes and `steps` (`step`, `title_tr`, `subassembly`, `text`, `tools`, `checks`) |

## 10. Writing a producer module (pattern every `design/<module>.py` follows)

```python
"""<Module> producer: what it builds, its interfaces (spec.layout keys it reads) and its manufacturing notes."""
from ..core.parts import Joint, Part, Registry, mirror_part, part_number
from . import joints as J, structgen as SG          # actuation as A, oml as O when needed

def register(reg: Registry, spec: dict) -> None:
    L = spec["layout"]                               # interfaces only — never another module's geometry
    n0 = L["part_numbers"]["<module>"][0]            # allocated number range
    pid = part_number("<group>", n0 + 1, side="R")   # YK250-<CODE>-NNN-R
    reg.add(Part(id=pid, name="...", name_tr="...", group="<group>", material="<spec.materials key>",
                 process="<spec.processes key>", layup="<spec.layups key>" or thickness=<m>,
                 mesh_fn=lambda: SG.rib(...), parent="<part it mounts on>", step=<assembly step>,
                 explode=(dx, dy, dz), contacts=("<bonded/fastened neighbours>",), side="R"))
    J.bolt_through(reg, f"{pid}-B1", 5, point, axis, [pid, "<mating part>"], nut="nutplate", step=<step>)
    reg.add(mirror_part(reg.parts[pid], pid[:-1] + "L", id_map={...}))   # port copy (holes/fasteners mirrored)
```

Rules: (1) read positions, sizes and part numbers from `spec.layout`; (2) every part has material + process +
thickness/layup, a parent and an assembly step; (3) joints via `design/joints.py`, hinges via `design/actuation.py`,
OML-fitted parts via `design/structgen.py`; (4) moving parts reference a `Joint` (radians/metres, rest pose = 0);
(5) `tests/test_ucav250_<module>.py` builds `build_registry(modules=[...dependencies, "<module>", "hardware"],
strict=False)` and asserts zero mesh / static / swept / clearance / thickness / fastener / attachment violations for
the module's parts: `python3 -m ucav250.analysis.checks --modules chassis,<module> --focus YK250-<CODE> --no-write`;
(6) look at the parts: `python3 -m ucav250.blender.build --modules chassis,<module> --previews --no-blend
--out /tmp/<module>` and Read the PNGs.
