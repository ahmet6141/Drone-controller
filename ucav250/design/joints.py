"""Joint helpers for producers: one call makes a real fastened joint.

A call creates the :class:`Fastener` record with a standard length and the correct ``grip``, cuts the clearance hole
through every clamped part (``Part.add_hole``, ISO 273 medium), cuts the insert bore / tap-drill hole in the receiving
part where there is one, and stores the fastener on its owner part. ``hardware.py`` later turns the record into
head, shank, washers and nut / nutplate / insert / receptacle geometry; ``analysis/checks.py`` verifies edge distance
(>= 2 D metal, >= 2.5 D composite), pitch and that the fastener really pierces each joined part.

Two ways to call:

* :func:`bolt` with an explicit ``stack`` ``[(part_id, thickness_m), ...]`` from the head side, the head bearing
  point ``position`` on the first part's outer face and the insertion ``axis``;
* :func:`bolt_through` with any point on the fastener centre line and the part ids: the stack thicknesses and the
  head point are measured on the parts' geometry (probe lines parallel to the axis), and a stack with gaps larger
  than ``max_gap`` raises (a bolted joint must clamp solid material; shim the joint in the geometry instead).

Lengths come from the ISO 4762 / ISO 4017 preferred length series. Nyloc nuts need >= 1 thread protruding; nutplates
need the bolt through the locking element; inserts and tapped holes use the available thread depth (at least 1.2 D
engagement, else ValueError).
"""
from __future__ import annotations

import math

import numpy as np

from ..core.geom import ray_hits, unit
from ..core.parts import Fastener, Registry
from . import fastener_catalog as C

STD_LENGTH_MM = (4, 5, 6, 8, 10, 12, 14, 16, 18, 20, 22, 25, 28, 30, 32, 35, 38, 40, 45, 50, 55, 60, 65, 70, 75,
                 80, 90, 100, 110, 120, 130, 140, 150)
PITCH = {2.5: 0.00045, 3: 0.0005, 4: 0.0007, 5: 0.0008, 6: 0.001, 8: 0.00125, 10: 0.0015, 12: 0.00175}
PIN_LENGTH_MM = (6, 8, 10, 12, 14, 16, 18, 20, 22, 24, 26, 28, 30, 32, 35, 40, 45, 50, 55, 60, 65, 70, 75, 80, 90,
                 100, 120)


def std_length(min_len: float, series=STD_LENGTH_MM) -> float:
    """Smallest preferred length (m) >= ``min_len``."""
    for L in series:
        if L * 1e-3 >= min_len - 1e-9:
            return L * 1e-3
    raise ValueError(f"no standard length >= {min_len * 1000:.1f} mm")


def std_length_floor(max_len: float, series=STD_LENGTH_MM) -> float:
    """Largest preferred length (m) <= ``max_len``."""
    best = None
    for L in series:
        if L * 1e-3 <= max_len + 1e-9:
            best = L * 1e-3
    if best is None:
        raise ValueError(f"no standard length <= {max_len * 1000:.1f} mm")
    return best


def _perp(a):
    ref = np.array([0.0, 0.0, 1.0]) if abs(a[2]) < 0.9 else np.array([1.0, 0.0, 0.0])
    e1 = unit(np.cross(a, ref))
    return e1, np.cross(a, e1)


def measure_stack(reg: Registry, part_ids, point, axis, probe_r: float, search: float = 0.3):
    """Material intervals of each part along the line ``point + t * axis`` (|t| <= search), measured on four probe
    lines at radius ``probe_r`` around it (so an existing hole does not hide the material). Returns
    ``[(part_id, t_enter, t_exit), ...]`` sorted along the axis."""
    a = unit(axis)
    c = np.asarray(point, float)
    e1, e2 = _perp(a)
    out = []
    for pid in part_ids:
        man = reg.parts[pid].mesh.to_manifold()
        ins, outs = [], []
        for d in (e1, -e1, e2, -e2):
            o = c + probe_r * d
            h = ray_hits(man, o - search * a, o + search * a) - search
            if len(h) >= 2:
                ins.append(h[0])
                outs.append(h[-1])
        if not ins:
            raise ValueError(f"fastener line does not pass through {pid}")
        out.append((pid, float(np.median(ins)), float(np.median(outs))))
    return sorted(out, key=lambda r: r[1])


def _spec(head: str, size, L: float, grade: str) -> str:
    return f"{head} M{size:g}x{L * 1000:.0f}-{grade}"


def bolt(reg: Registry, fid: str, size, position, axis, stack, *, owner: str | None = None, head: str = "ISO 4762",
         grade: str = "A2-70", nut: str = "ISO 7040", washer_head: bool = False, washer_nut: bool = True,
         insert_part: str | None = None, insert_depth: float | None = None, tapped_part: str | None = None,
         tapped_depth: float | None = None, step: int = 0, notes: str = "", torque_nm: float | None = None,
         hole_d: float | None = None) -> Fastener:
    """Bolted joint through ``stack`` [(part_id, thickness_m), ...] (head side first). ``nut`` selects the far side:

    * ``"ISO 7040"`` (nyloc nut, default) or ``"ISO 4032"``: length = grip + washers + nut + >= 1 thread protrusion;
    * ``"nutplate"``: floating anchor nutplate riveted to the far face of the last stack part;
    * ``"insert"``: potted insert in ``insert_part`` (sandwich panel) whose face is at the end of the stack;
    * ``"tapped"``: thread tapped in ``tapped_part`` (metal), starting at the end of the stack.
    """
    a = unit(axis)
    p = np.asarray(position, float)
    grip = float(sum(t for _pid, t in stack))
    d = float(size) * 1e-3
    pitch = PITCH.get(size, 0.15 * d)
    hd = hole_d if hole_d is not None else C.clearance(size)
    wh = C.ISO7089[size][2] if (washer_head and size in C.ISO7089) else 0.0
    nut_l = nut.lower()
    ids = [pid for pid, _t in stack]
    if "nutplate" in nut_l:
        if size not in C.NUTPLATE:
            raise ValueError(f"{fid}: no nutplate data for M{size}")
        H = C.NUTPLATE[size][2]
        L = std_length(wh + grip + H + pitch)
        nut_desc = f"nutplate (floating anchor) M{size:g}"
    elif "insert" in nut_l:
        if insert_part is None or size not in C.INSERT:
            raise ValueError(f"{fid}: insert joint needs insert_part and INSERT data for M{size}")
        fl, body, ln = C.INSERT[size]
        depth = min(ln, insert_depth) if insert_depth else ln
        L = std_length_floor(wh + grip + depth - 0.0005)
        if L - wh - grip < 1.2 * d:
            raise ValueError(f"{fid}: thread engagement {(L - wh - grip) * 1000:.1f} mm < 1.2 D in the insert")
        q = p + grip * a                      # flush potted insert: flange counterbore + body bore
        # a given insert_depth shorter than the catalogue length is a through-thickness insert of that length
        # (thin sandwich lands): bore and insert body end there (hardware.py reads "L <mm>" from the nut text)
        ln_i = depth if insert_depth else ln
        reg.parts[insert_part].add_hole(q - 0.0005 * a, q + (ln_i + 0.0005) * a, 0.5 * body)
        reg.parts[insert_part].add_hole(q - 0.0005 * a, q + 0.00105 * a, 0.5 * fl + 0.0001)
        ids = ids + [insert_part]
        nut_desc = f"potted insert M{size:g} (fl {fl * 1000:.0f} mm, L {ln_i * 1000:.1f} mm)"
    elif "tapped" in nut_l:
        if tapped_part is None or tapped_depth is None:
            raise ValueError(f"{fid}: tapped joint needs tapped_part and tapped_depth")
        L = std_length_floor(wh + grip + tapped_depth - 2 * pitch)
        if L - wh - grip < 1.2 * d:
            raise ValueError(f"{fid}: thread engagement {(L - wh - grip) * 1000:.1f} mm < 1.2 D in the tapped hole")
        q = p + grip * a
        # the thread is modelled at its nominal diameter (cosmetic thread): the shank (0.98 d, hardware.py) then
        # does not overlap the tapped part; the tap drill (d - pitch) is a drawing note
        reg.parts[tapped_part].add_hole(q - 0.0005 * a, q + (L - wh - grip + 2 * pitch) * a, 0.5 * d)
        ids = ids + [tapped_part]
        nut_desc = f"tapped M{size:g} in {tapped_part}"
    else:
        s_m = C.ISO7040.get(size, (1.7 * d, d))[1]
        wn = C.ISO7089[size][2] if (washer_nut and size in C.ISO7089) else 0.0
        L = std_length(wh + grip + wn + s_m + max(pitch, 0.001))
        nut_desc = f"{nut} M{size:g}" + (" + ISO 7089 washer" if washer_nut else "")
    for pid in [pid for pid, _t in stack]:
        reg.parts[pid].add_hole(p - 0.001 * a, p + (grip + 0.001) * a, 0.5 * hd)
    tq = torque_nm if torque_nm is not None else C.TORQUE.get(grade, C.TORQUE["A2-70"]).get(size)
    # Fastener convention (hardware.py): position = bearing face of the first part (a head washer sits outside it),
    # length = shank length measured from that face, grip = clamped part stack.
    f = Fastener(id=fid, spec=_spec(head, size, L, grade), kind="bolt", d=d, length=L - wh, position=p, axis=a,
                 joins=tuple(ids), nut=nut_desc, torque_nm=tq, step=step, notes=notes, grip=grip,
                 washer_head=washer_head, washer_nut=washer_nut and "ISO" in nut)
    reg.parts[owner or stack[0][0]].fasteners.append(f)
    return f


def bolt_through(reg: Registry, fid: str, size, point, axis, part_ids, *, max_gap: float = 0.0005,
                 **kw) -> Fastener:
    """Like :func:`bolt`, but the stack is measured on the geometry along the line through ``point`` (any point on
    the centre line) in direction ``axis`` (head side -> nut side). Raises if the parts are not clamped solid."""
    a = unit(axis)
    r = 0.5 * C.clearance(size) * 1.6
    iv = measure_stack(reg, part_ids, point, a, r)
    for (p0, _a0, e0), (p1, s1, _e1) in zip(iv, iv[1:]):
        if s1 - e0 > max_gap:
            raise ValueError(f"{fid}: gap {(s1 - e0) * 1000:.2f} mm between {p0} and {p1}")
    head_pt = np.asarray(point, float) + iv[0][1] * a
    stack = [(pid, e - s) for pid, s, e in iv]
    # small overlaps/gaps between faces are absorbed in the first/last thickness so the nut lands on the far face
    total = iv[-1][2] - iv[0][1]
    stack[-1] = (stack[-1][0], stack[-1][1] + (total - sum(t for _p, t in stack)))
    return bolt(reg, fid, size, head_pt, a, stack, **kw)


def pin(reg: Registry, fid: str, d: float, position, axis, stack, *, owner: str | None = None, fit_clearance: float = 0.00005,
        spec: str = "ISO 2341-B", retention: str = "ISO 1234 split pin + ISO 7089 washer", step: int = 0,
        notes: str = "") -> Fastener:
    """Clevis/hinge pin through ``stack`` [(part_id, thickness_m), ...] with bores of d + ``fit_clearance``
    (H8/f7-class running fit by default). Length leaves room for the washer and split pin."""
    a = unit(axis)
    p = np.asarray(position, float)
    grip = float(sum(t for _pid, t in stack))
    L = std_length(grip + 0.0025 + 0.35 * d, PIN_LENGTH_MM)
    for pid, _t in stack:
        reg.parts[pid].add_hole(p - 0.001 * a, p + (grip + 0.001) * a, 0.5 * d + 0.5 * fit_clearance)
    f = Fastener(id=fid, spec=f"{spec} {d * 1000:g}x{L * 1000:.0f}", kind="clevis_pin", d=d, length=L,
                 position=p, axis=a, joins=tuple(pid for pid, _t in stack), nut=retention, step=step, notes=notes,
                 grip=grip)
    reg.parts[owner or stack[0][0]].fasteners.append(f)
    return f


def quarter_turn(reg: Registry, fid: str, position, axis, panel: str, panel_t: float, structure: str,
                 structure_t: float, *, step: int = 0, notes: str = "") -> Fastener:
    """Quarter-turn panel fastener (Camloc 4002 class): stud with grommet in ``panel``, receptacle riveted to the far
    face of the ``structure`` flange. Holes: stud clearance through both."""
    Q = C.QUARTER_TURN
    a = unit(axis)
    p = np.asarray(position, float)
    grip = panel_t + structure_t
    L = grip + Q["rec_barrel"][1] - 0.001
    for pid in (panel, structure):
        reg.parts[pid].add_hole(p - 0.001 * a, p + (grip + 0.001) * a, 0.5 * Q["stud_d"] + 0.0004)
    f = Fastener(id=fid, spec="Camloc 4002 stud + 2600 receptacle (quarter-turn)", kind="camloc", d=Q["stud_d"],
                 length=L, position=p, axis=a, joins=(panel, structure), nut="Camloc 2600 receptacle (riveted)",
                 step=step, notes=notes, grip=grip)
    reg.parts[panel].fasteners.append(f)
    return f
