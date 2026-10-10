"""Part registry: the single data model every ucav250 module writes into and every consumer reads from.

Producers (one module per subsystem, see ARCHITECTURE.md) implement ``register(reg: Registry, spec: dict) -> None``
and add :class:`Part`, :class:`Joint` and :class:`Fastener` records. Consumers (checks, mass properties, BOM,
drawings, Blender build, tests) only read the registry. Nothing outside a producer module creates geometry.
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Callable, Iterable

import numpy as np

from .geom import Mesh, rot_axis_angle, unit

# group -> part-number code. Order is also the Blender collection order.
GROUPS = {
    "chassis": "CH",      # primary structure: frames, keel/longerons, spar, carry-through, engine mount, gear mounts
    "shell": "SH",        # skins, fairings, cowlings, access panels, doors (non-moving)
    "wing": "WG",         # wing panels (if not split into chassis/shell by the wing module)
    "tail": "TL",         # tail surfaces
    "controls": "FC",     # control surfaces, hinges, horns, linkages, actuators
    "propulsion": "PR",   # engine, propeller, spinner, exhaust, cooling ducts, engine accessories
    "fuel": "FU",         # tanks, lines, vents, pumps, filters
    "gear": "LG",         # landing gear legs, wheels, brakes, steering, doors that move
    "systems": "SY",      # avionics, electrical, antennas, lights, parachute
    "payload": "PL",      # EO/IR gimbal and mission equipment
    "hardware": "HW",     # fasteners, inserts, camlocks, nutplates, pins
}
SIDES = ("C", "L", "R")


@dataclass
class Fastener:
    """One installed fastener. ``position`` is the head bearing point, ``axis`` the unit insertion direction (from
    head toward nut/insert). ``joins`` lists the part ids clamped together, in order from head to nut."""

    id: str
    spec: str                   # e.g. "ISO 4762 M6x20-12.9", "Camloc 4002-1W receptacle 2600", "ISO 8734 4m6x16"
    kind: str                   # bolt | screw | camloc | nutplate | insert | rivet | pin | clevis_pin
    d: float                    # nominal shank diameter (m)
    length: float               # shank length (m)
    position: np.ndarray
    axis: np.ndarray
    joins: tuple[str, ...]
    nut: str = ""               # e.g. "ISO 7040 M6 (nyloc)", "nutplate NAS1068-3", "potted insert M5"
    torque_nm: float | None = None
    step: int = 0
    notes: str = ""
    grip: float | None = None   # clamped stack thickness (m): nut/nutplate sits at position + grip * axis
    washer_head: bool = False   # ISO 7089 washer under the head
    washer_nut: bool = True     # ISO 7089 washer under the nut (ignored for nutplates/inserts)
    orient: np.ndarray | None = None  # long-axis direction of a nutplate / receptacle base in the land plane (None:
    #                                   hardware.py default); a producer aligns it with the land (frame cap, flange)

    def __post_init__(self):
        self.position = np.asarray(self.position, float)
        self.axis = unit(self.axis)
        if self.orient is not None:
            self.orient = unit(self.orient)


@dataclass
class Joint:
    """Kinematic joint. Defined in the REST configuration (all joints at ``rest``)."""

    name: str
    kind: str                   # "revolute" (value in rad) | "prismatic" (value in m)
    origin: np.ndarray
    axis: np.ndarray            # right-hand rule for positive values
    lo: float
    hi: float
    rest: float = 0.0
    prop: str = ""              # Blender control property (on the root control object) that drives this joint
    scale: float = 1.0          # joint value = scale * property value (e.g. deg -> rad with sign)
    parent: str | None = None   # parent joint (child axis moves with the parent)
    expr: str = ""              # optional Blender simple-expression for coupled motion (uses control properties)
    notes: str = ""

    def __post_init__(self):
        self.origin = np.asarray(self.origin, float)
        self.axis = unit(self.axis)
        if self.kind not in ("revolute", "prismatic"):
            raise ValueError(f"joint {self.name}: kind must be revolute|prismatic")
        if not self.lo <= self.rest <= self.hi:
            raise ValueError(f"joint {self.name}: rest {self.rest} outside [{self.lo}, {self.hi}]")

    def samples(self, n: int = 9) -> np.ndarray:
        return np.linspace(self.lo, self.hi, n)

    def apply(self, V: np.ndarray, value: float) -> np.ndarray:
        d = value - self.rest
        if self.kind == "revolute":
            R = rot_axis_angle(self.axis, d)
            return (V - self.origin) @ R.T + self.origin
        return V + d * self.axis


@dataclass
class Part:
    id: str                                  # unique part number, e.g. "YK250-CH-020" or "YK250-SH-110-R"
    name: str                                # short English name
    name_tr: str                             # Turkish name (BOM, assembly guide)
    group: str                               # key of GROUPS
    material: str                            # key of spec["materials"] ("purchased" allowed for bought items)
    process: str                             # key of spec["processes"]
    mesh_fn: Callable[[], Mesh] | None = None
    thickness: float | None = None           # governing wall / laminate thickness (m)
    layup: str | None = None                 # key of spec["layups"] for composite parts
    purchased: bool = False
    vendor: str = ""                         # manufacturer + model for purchased parts
    mass_kg: float | None = None             # datasheet mass; overrides geometry-derived mass
    side: str = "C"
    joint: str | None = None                 # moves with this joint (and its parents)
    parent: str | None = None                # part it is mounted on (assembly tree)
    step: int = 0                            # assembly step (see assembly.py / spec["assembly"])
    explode: tuple = (0.0, 0.0, 0.0)         # exploded-view offset (m)
    fasteners: list[Fastener] = field(default_factory=list)
    contacts: tuple[str, ...] = ()           # intended contacts (bonded/fastened faces); volume overlap must still be ~0
    outline: list = field(default_factory=list)   # mid-surface boundary polylines (k, 3) for edge-distance checks
    color: str = ""                          # display colour key (spec["display"]["colors"])
    notes: str = ""
    holes: list = field(default_factory=list)  # cutters subtracted from the mesh: (p0, p1, radius) cylinders or Mesh
    _base: Mesh | None = field(default=None, repr=False)
    _mesh: Mesh | None = field(default=None, repr=False)
    _n_holes: int = field(default=-1, repr=False)

    def __post_init__(self):
        if self.group not in GROUPS:
            raise ValueError(f"{self.id}: unknown group {self.group!r}")
        if self.side not in SIDES:
            raise ValueError(f"{self.id}: side must be one of {SIDES}")

    @property
    def base_mesh(self) -> Mesh:
        """Geometry from ``mesh_fn`` before the registered holes are cut."""
        if self._base is None:
            if self.mesh_fn is None:
                raise ValueError(f"{self.id}: no geometry")
            self._base = self.mesh_fn()
        return self._base

    @property
    def mesh(self) -> Mesh:
        """Final geometry: base mesh minus every entry of ``holes`` (re-evaluated when holes are added)."""
        if self._mesh is None or self._n_holes != len(self.holes):
            base = self.base_mesh
            if self.holes:
                from .geom import cylinder, difference
                cut = [h if isinstance(h, Mesh) else cylinder(float(h[2]), h[0], h[1], n=24) for h in self.holes]
                self._mesh = difference(base, cut)
            else:
                self._mesh = base
            self._n_holes = len(self.holes)
        return self._mesh

    def add_hole(self, p0, p1, radius: float) -> None:
        """Cylindrical hole (fastener clearance, pin bore, pass-through) from p0 to p1."""
        self.holes.append((np.asarray(p0, float), np.asarray(p1, float), float(radius)))


class Registry:
    def __init__(self, spec: dict):
        self.spec = spec
        self.parts: dict[str, Part] = {}
        self.joints: dict[str, Joint] = {}
        self.sequences: dict[str, list[dict[str, float]]] = {}   # named multi-joint motion sequences (sampled)
        self.log: list[str] = []

    # ------------------------------------------------------------------ producers
    def add(self, part: Part) -> Part:
        if part.id in self.parts:
            raise ValueError(f"duplicate part id {part.id}")
        self.parts[part.id] = part
        return part

    def add_joint(self, joint: Joint) -> Joint:
        if joint.name in self.joints:
            raise ValueError(f"duplicate joint {joint.name}")
        if joint.parent is not None and joint.parent not in self.joints:
            raise ValueError(f"joint {joint.name}: parent {joint.parent} must be added first")
        self.joints[joint.name] = joint
        return joint

    def add_sequence(self, name: str, states: list[dict[str, float]]) -> None:
        """Coupled motion (e.g. gear retraction with door opening/closing) as a list of joint-value dicts; the swept
        interference check evaluates every state. Single joints are swept automatically; add a sequence whenever two
        or more joints move together in service."""
        for st in states:
            for k in st:
                if k not in self.joints:
                    raise ValueError(f"sequence {name}: unknown joint {k}")
        self.sequences[name] = states

    def note(self, msg: str) -> None:
        self.log.append(msg)

    # ------------------------------------------------------------------ queries
    def by_group(self, group: str) -> list[Part]:
        return [p for p in self.parts.values() if p.group == group]

    def fasteners(self) -> list[Fastener]:
        return [f for p in self.parts.values() for f in p.fasteners]

    def joint_chain(self, name: str | None) -> list[Joint]:
        """Joints from the given one up to the root (child first)."""
        chain = []
        while name is not None:
            j = self.joints[name]
            chain.append(j)
            name = j.parent
        return chain

    def moving_parts(self, joint: str) -> list[Part]:
        """Parts that move when ``joint`` moves (directly or through a child joint)."""
        out = []
        for p in self.parts.values():
            if p.joint is not None and any(j.name == joint for j in self.joint_chain(p.joint)):
                out.append(p)
        return out

    def posed_vertices(self, part: Part, state: dict[str, float] | None = None) -> np.ndarray:
        """Vertices of ``part`` with joints set to ``state`` (missing joints at rest)."""
        V = part.mesh.V
        if part.joint is None or not state:
            return V
        for j in self.joint_chain(part.joint):              # child first, then parents
            V = j.apply(V, state.get(j.name, j.rest))
        return V

    def posed_mesh(self, part: Part, state: dict[str, float] | None = None) -> Mesh:
        return Mesh(self.posed_vertices(part, state), part.mesh.F)

    # ------------------------------------------------------------------ material helpers
    def density(self, part: Part) -> float:
        """Effective density (kg/m^3): layup areal mass / thickness for composites, else material density."""
        mats = self.spec.get("materials", {})
        if part.layup:
            lay = layup_props(self.spec, part.layup)
            return lay["areal_mass"] / lay["thickness"]
        m = mats.get(part.material)
        if m is None or "density" not in m:
            raise KeyError(f"{part.id}: material {part.material!r} has no density in spec.materials")
        return float(m["density"])

    def mass(self, part: Part) -> float:
        if part.mass_kg is not None:
            return float(part.mass_kg)
        return part.mesh.volume() * self.density(part)


def layup_props(spec: dict, key: str) -> dict:
    """Thickness (m) and areal mass (kg/m^2) of a layup in spec["layups"]:
    ``{plies: [[material, angle_deg, count], ...], inner_plies: [...], core: material|null, core_t: m}`` (``plies`` =
    outer face, ``inner_plies`` = inner face of a sandwich); ply materials must define ``ply_t`` (cured ply thickness,
    m) and ``density`` (kg/m^3)."""
    lay = spec["layups"][key]
    mats = spec["materials"]
    t = 0.0
    am = 0.0
    # both sandwich faces count: ``plies`` is the outer (tool-side) face, ``inner_plies`` the inner face
    for mat, _angle, count in list(lay.get("plies", []) or []) + list(lay.get("inner_plies", []) or []):
        if not isinstance(count, (int, float)):
            raise ValueError(f"layup {key}: ply count {count!r} is sized from the loads, not a number - give the part "
                             "an explicit thickness and a material instead of this layup")
        m = mats[mat]
        t += m["ply_t"] * count
        am += m["ply_t"] * m["density"] * count
    if lay.get("core"):
        c = mats[lay["core"]]
        t += lay["core_t"]
        am += lay["core_t"] * c["density"]
    if lay.get("adhesive_areal"):
        am += float(lay["adhesive_areal"])
    return {"thickness": t, "areal_mass": am}


def part_number(group: str, number: int, side: str = "C", prefix: str = "YK250") -> str:
    code = GROUPS[group]
    s = "" if side == "C" else f"-{side}"
    return f"{prefix}-{code}-{number:03d}{s}"


def mirror_part(p: Part, new_id: str, joint: str | None = None, id_map: dict[str, str] | None = None) -> Part:
    """Port copy of a starboard part (BL -> -BL): mesh, outlines, fasteners, holes and explode vector mirrored, side
    R -> L. Holes present at mirror time are copied; holes added to either side later stay on that side.
    ``joint`` is the (already mirrored) port joint name, if the part moves. ``id_map`` maps starboard part ids to
    port ids for ``contacts`` and fastener ``joins`` (ids not in the map are kept, e.g. centre-line parts)."""
    id_map = dict(id_map or {})
    id_map.setdefault(p.id, new_id)
    from .geom import Mesh as _Mesh  # noqa: F401

    def mfn(src=p):
        return src.base_mesh.mirrored_y()

    def mvec(v):
        v = np.asarray(v, float).copy()
        v[..., 1] *= -1
        return v

    fs = [Fastener(id=f.id.replace(p.id, new_id), spec=f.spec, kind=f.kind, d=f.d, length=f.length,
                   position=mvec(f.position), axis=mvec(f.axis),
                   joins=tuple(id_map.get(j, j) for j in f.joins), nut=f.nut, torque_nm=f.torque_nm,
                   step=f.step, notes=f.notes, grip=f.grip, washer_head=f.washer_head, washer_nut=f.washer_nut,
                   orient=None if f.orient is None else mvec(f.orient))
          for f in p.fasteners]

    def mhole(h):
        if isinstance(h, _Mesh):
            return h.mirrored_y()
        return (mvec(h[0]), mvec(h[1]), float(h[2]))
    return Part(id=new_id, name=p.name.replace("starboard", "port").replace("RH", "LH"),
                name_tr=p.name_tr.replace("sağ", "sol").replace("Sağ", "Sol"), group=p.group,
                material=p.material, process=p.process, mesh_fn=mfn, thickness=p.thickness, layup=p.layup,
                purchased=p.purchased, vendor=p.vendor, mass_kg=p.mass_kg, side="L" if p.side == "R" else p.side,
                joint=joint, parent=id_map.get(p.parent, p.parent) if p.parent else None, step=p.step,
                explode=tuple(mvec(p.explode)), fasteners=fs, contacts=tuple(id_map.get(c, c) for c in p.contacts),
                outline=[mvec(o) for o in p.outline], color=p.color, notes=p.notes,
                holes=[mhole(h) for h in p.holes])


def iter_pairs(parts: Iterable[Part]):
    ps = list(parts)
    for i in range(len(ps)):
        for j in range(i + 1, len(ps)):
            yield ps[i], ps[j]


def deg(x: float) -> float:
    return math.radians(x)
