"""Producer (runs LAST): turns every :class:`Fastener` record already in the registry into a hardware part with real
geometry — head, shank, washers and nut, or a nutplate / potted insert / quarter-turn receptacle on the far side.

Geometry contract (see ARCHITECTURE.md): the producer that created a Fastener must have cut a clearance hole
(``fastener_catalog.clearance``) through every joined part along the fastener axis, and must give ``grip`` (clamped
stack thickness) so the nut/nutplate lands on the far face. Heads and nuts then only TOUCH the outer faces; the shank
passes through the holes; nothing overlaps.

Each fastener becomes one Part ``YK250-HW-<fastener id>`` in group "hardware" (material ``fastener_steel`` /
``fastener_stainless`` / ``fastener_ti`` when spec.materials defines them, else the spec material of the same alloy
family so that the mass has a density: ``steel_4130_n``, ``ss_304_annealed``, ``ti_6al_4v_annealed_sheet``), parented
to the first joined part, at the fastener's assembly step.
"""
from __future__ import annotations

import math

import numpy as np

from ..core.geom import Mesh, box, cylinder, difference, merge, revolve, union, unit
from ..core.parts import Fastener, Part, Registry
from . import fastener_catalog as C


def _prism_n(radius: float, p0, p1, n: int) -> Mesh:
    """n-sided prism (hex head/nut) between p0 and p1, circumradius ``radius``."""
    return cylinder(radius, p0, p1, n=n)


def _bolt_mesh(f: Fastener) -> Mesh:
    a = unit(f.axis)
    p = np.asarray(f.position, float)
    size = C.size_from_spec(f.spec) or max(3, int(round(f.d * 1000)))
    parts = []
    head_off = 0.0
    OV = 5e-5                                           # overlaps so unions fuse (touching solids stay separate)
    if f.washer_head and size in C.ISO7089:
        d1, d2, h = C.ISO7089[size]
        parts.append(revolve([(d1 / 2, 0), (d2 / 2, 0), (d2 / 2, h), (d1 / 2, h)], n=32, axis_origin=p - h * a,
                             axis=a))
        head_off = h - OV
    spec_u = f.spec.upper()
    if "7380" in spec_u and size in C.ISO7380:
        dk, k = C.ISO7380[size]
        prof = [(0.0, 0.0), (dk / 2, 0.0), (dk / 2 * 0.95, k * 0.35), (dk / 4, k), (0.0, k)]
        parts.append(revolve(prof, n=32, axis_origin=p - head_off * a, axis=-a))
    elif ("4017" in spec_u or "4014" in spec_u) and size in C.ISO4017:
        s, k = C.ISO4017[size]
        parts.append(_prism_n(s / math.sqrt(3), p - (head_off + k) * a, p - head_off * a, 6))
    else:                                               # ISO 4762 socket head (default)
        dk, k = C.ISO4762.get(size, (1.7 * f.d, f.d))
        parts.append(cylinder(dk / 2, p - (head_off + k) * a, p - head_off * a, n=32))
    # shank slightly undersized (0.98 d) so it never touches the clearance-hole wall numerically
    parts.append(cylinder(0.49 * f.d, p - head_off * a - 1e-4 * a, p + f.length * a, n=20))
    head_group = union(parts)
    parts = []
    nut = f.nut.lower()
    if f.grip is not None and nut and not any(k in nut for k in ("nutplate", "insert", "anchor", "receptacle",
                                                                     "tapped")):
        q = p + f.grip * a
        if f.washer_nut and size in C.ISO7089:
            d1, d2, h = C.ISO7089[size]
            parts.append(revolve([(d1 / 2, 0), (d2 / 2, 0), (d2 / 2, h), (d1 / 2, h)], n=32, axis_origin=q, axis=a))
            q = q + (h - OV) * a
        s, m = C.ISO7040.get(size, (1.7 * f.d, f.d))
        hexp = cylinder(s / math.sqrt(3), q, q + m * a, n=6)
        parts.append(difference(hexp, [cylinder(0.5 * f.d, q - 1e-3 * a, q + (m + 1e-3) * a, n=24)]))
    elif f.grip is not None and "nutplate" in nut and size in C.NUTPLATE:
        L, W, H, _pitch = C.NUTPLATE[size]
        q = p + f.grip * a
        ref = np.array([0, 0, 1.0]) if abs(a[2]) < 0.9 else np.array([1.0, 0, 0])
        e1 = unit(np.cross(a, ref))
        e2 = np.cross(a, e1)
        R = np.column_stack([e1, e2, a])
        base = box((L, W, 0.0008), center=q + 0.0004 * a, R=R)
        barrel = cylinder(W * 0.42, q + (0.0008 - 1e-4) * a, q + H * a, n=24)
        bore = cylinder(0.5 * f.d, q - 1e-3 * a, q + (H + 1e-3) * a, n=24)
        parts.append(difference(union([base, barrel]), [bore]))
    elif f.grip is not None and "insert" in nut and size in C.INSERT:
        fl, body, ln = C.INSERT[size]
        import re
        m_ln = re.search(r"L (\d+(?:\.\d+)?) mm", f.nut)      # shortened through-thickness insert (joints.bolt)
        if m_ln:
            ln = min(ln, float(m_ln.group(1)) * 1e-3)
        q = p + f.grip * a                              # panel face: flange flush there, body goes deeper (+axis)
        ins = union([cylinder(fl / 2, q, q + 0.001 * a, n=24), cylinder(body / 2, q + 0.0009 * a, q + ln * a, n=24)])
        parts.append(difference(ins, [cylinder(0.5 * f.d, q - 1e-3 * a, q + (ln + 1e-3) * a, n=24)]))
    if not parts:
        return head_group
    return merge([head_group, union(parts)])


def _quarter_turn_mesh(f: Fastener) -> Mesh:
    a = unit(f.axis)
    p = np.asarray(f.position, float)
    Q = C.QUARTER_TURN
    head = union([cylinder(Q["stud_head_d"] / 2, p - Q["stud_head_h"] * a, p, n=32),
                  cylinder(Q["stud_d"] / 2 * 0.98, p - 1e-4 * a, p + f.length * a, n=16)])
    parts = []
    if f.grip is not None:
        q = p + f.grip * a
        L, W, t = Q["rec_plate"]
        ref = np.array([0, 0, 1.0]) if abs(a[2]) < 0.9 else np.array([1.0, 0, 0])
        e1 = unit(np.cross(a, ref))
        R = np.column_stack([e1, np.cross(a, e1), a])
        bd, bh = Q["rec_barrel"]
        rec = union([box((L, W, t), center=q + 0.5 * t * a, R=R), cylinder(bd / 2, q + (t - 1e-4) * a, q + bh * a, n=24)])
        parts.append(difference(rec, [cylinder(Q["stud_d"] / 2 + 2e-4, q - 1e-3 * a, q + (bh + 1e-3) * a, n=24)]))
    return head if not parts else merge([head, union(parts)])


def fastener_mesh(f: Fastener) -> Mesh:
    if f.kind in ("camloc", "quarter_turn"):
        return _quarter_turn_mesh(f)
    if f.kind in ("pin", "clevis_pin", "rivet"):
        a = unit(f.axis)
        p = np.asarray(f.position, float)
        return cylinder(0.49 * f.d, p - 1e-4 * a, p + f.length * a, n=20)
    return _bolt_mesh(f)


# fastener material -> spec.materials key with a density (first present): dedicated fastener entries if the spec has
# them, else the spec material of the same alloy family (12.9 alloy steel ~ 4130 N density, A2/A4 ~ 304, Ti-6Al-4V)
_MAT_CANDIDATES = {"ti": ("fastener_ti", "ti_6al_4v_annealed_sheet"),
                   "stainless": ("fastener_stainless", "ss_304_annealed"),
                   "steel": ("fastener_steel", "steel_4130_n")}


def fastener_material(designation: str, mats: dict) -> str:
    u = designation.upper()
    fam = "ti" if "TI" in u else ("stainless" if ("A2-" in u or "A4-" in u) else "steel")
    cands = _MAT_CANDIDATES[fam]
    return next((m for m in cands if m in mats), cands[0])


def register(reg: Registry, spec: dict) -> None:
    mats = spec.get("materials", {}) or {}
    for f in list(reg.fasteners()):
        pid = f"YK250-HW-{f.id}"
        if pid in reg.parts:
            raise ValueError(f"duplicate fastener id {f.id}")
        owner = reg.parts[f.joins[0]] if f.joins and f.joins[0] in reg.parts else None
        reg.add(Part(id=pid, name=f.spec, name_tr=f.spec, group="hardware",
                     material=fastener_material(f.spec, mats), process="purchased",
                     mesh_fn=(lambda f=f: fastener_mesh(f)), thickness=None, purchased=True,
                     vendor=f.spec + (f" + {f.nut}" if f.nut else ""), side=owner.side if owner else "C",
                     joint=owner.joint if owner else None, parent=f.joins[0] if f.joins else None, step=f.step,
                     explode=tuple(np.asarray(owner.explode) if owner else (0.0, 0.0, 0.0)),
                     contacts=tuple(f.joins), notes=f.notes))
