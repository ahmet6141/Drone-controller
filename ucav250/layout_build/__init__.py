"""Layout-phase generator of ``spec.layout`` and ``spec.assembly`` (interface definition for the detail modules).

Modules: ``b_common`` (spec, sizing geometry anchors), ``b_stations`` (part numbering, frames), ``b_chassis``
(members, wing joint, fittings), ``b_loads`` (engine mount, turret elevator, parachute, fuel supports, trays, design
loads), ``b_shell`` (panels, fastening rules), ``b_mech`` (joints, sequences, keep-outs, clearances), ``b_systems``
(equipment, antennas, air data, lights, actuators, harness), ``b_assembly`` (assembly steps, transport, field assembly,
maintenance access) and ``build`` (CLI). Run from the repository root:

    python3 -m ucav250.layout_build.build --check | --write

The authored values live in these modules; ``spec.yaml`` is their generated, checked output
(``ucav250.analysis.layout_check``)."""
