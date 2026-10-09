"""Design-rule checks over the part registry (ARCHITECTURE.md §6).

Checks: mesh validity, static interference, swept interference (single joints, joint-pair extremes and registered
coupled sequences), minimum clearances, process thickness, fastener edge distance/pitch, attachment graph.
Each check returns a list of violation dicts ``{"check", "parts", "value", "limit", "detail"}``; an empty list
means pass. ``run_all`` writes ``out/checks.md`` (Turkish) and ``out/checks.json``.

CLI:  python3 -m ucav250.analysis.checks [--quick] [--only mesh,static,...]   (exit 1 on any violation)
"""
from __future__ import annotations

import argparse
import itertools
import json
import math
import sys
import time
from collections import defaultdict, deque

import numpy as np

from ..core import geom as G
from ..core.parts import Part, Registry

VOL_TOL = 1e-9          # 1 mm^3: numerical noise of coplanar contact faces
COMPOSITE_EDGE = 2.5    # edge distance factor (x D) in composites
METAL_EDGE = 2.0        # edge distance factor (x D) in metals and polymers


# =====================================================================================================================
# helpers
# =====================================================================================================================
class _ManCache:
    """Manifold objects and AABBs of rest-pose meshes, built once."""

    def __init__(self, reg: Registry):
        self.reg = reg
        self._man = {}
        self._box = {}

    def man(self, pid: str):
        if pid not in self._man:
            self._man[pid] = self.reg.parts[pid].mesh.to_manifold()
        return self._man[pid]

    def box(self, pid: str):
        if pid not in self._box:
            self._box[pid] = self.reg.parts[pid].mesh.bounds()
        return self._box[pid]


def _boxes_overlap(a, b, pad=0.0) -> bool:
    return bool(np.all(a[1] + pad >= b[0]) and np.all(b[1] + pad >= a[0]))


def _violation(check, parts, value, limit, detail=""):
    return {"check": check, "parts": list(parts), "value": value, "limit": limit, "detail": detail}


def _allowed_contact(a: Part, b: Part) -> bool:
    return b.id in a.contacts or a.id in b.contacts


def _material_kind(reg: Registry, part: Part) -> str:
    m = reg.spec.get("materials", {}).get(part.material, {})
    if part.layup:
        return "composite"
    return str(m.get("kind", "metal"))


# =====================================================================================================================
# 1. mesh validity
# =====================================================================================================================
def check_meshes(reg: Registry, self_intersect: bool = True) -> list[dict]:
    out = []
    for p in reg.parts.values():
        try:
            ck = p.mesh.check(self_intersect=self_intersect)
        except Exception as exc:  # geometry generator failed
            out.append(_violation("mesh", [p.id], str(exc), "valid closed mesh"))
            continue
        if not ck["ok"]:
            out.append(_violation("mesh", [p.id], {k: ck[k] for k in ck if k != "ok"}, "closed, outward, no "
                                  "degenerate faces, no self-intersections"))
    return out


# =====================================================================================================================
# 2. static interference
# =====================================================================================================================
def check_static(reg: Registry, cache: _ManCache | None = None, tol: float = VOL_TOL) -> list[dict]:
    cache = cache or _ManCache(reg)
    ids = [i for i, p in reg.parts.items() if p.process != "consumable"]
    boxes = {i: cache.box(i) for i in ids}
    out = []
    # sweep-and-prune on X
    order = sorted(ids, key=lambda i: boxes[i][0][0])
    active: list[str] = []
    for pid in order:
        lo = boxes[pid][0][0]
        active = [q for q in active if boxes[q][1][0] >= lo]
        for q in active:
            if not _boxes_overlap(boxes[pid], boxes[q]):
                continue
            v = cache.man(pid) ^ cache.man(q)
            vol = 0.0 if v.is_empty() else float(v.volume())
            if vol > tol:
                a, b = reg.parts[pid], reg.parts[q]
                why = "intended contact but volume overlap" if _allowed_contact(a, b) else "interference"
                out.append(_violation("static", sorted([pid, q]), round(vol * 1e9, 3), "<= 1 mm^3", why))
        active.append(pid)
    return out


# =====================================================================================================================
# 3. swept interference
# =====================================================================================================================
def _coupled(reg: Registry, name: str) -> bool:
    """A coupled joint (Joint.expr set: gear legs and doors, turret elevator and bay doors) moves only along its
    registered sequence; sweeping it alone or in independent extreme pairs poses states that never occur (e.g. a leg
    swinging through a closed door) - fix round 3, PK3-03."""
    j = reg.joints.get(name)
    return bool(j is not None and getattr(j, "expr", ""))


def _seq_states(reg: Registry, name: str) -> list[dict[str, float]]:
    """Every registered sequence state that sets joint ``name``."""
    return [st for seq in reg.sequences.values() for st in seq if name in st]


def _states(reg: Registry, n: int) -> list[tuple[str, dict[str, float]]]:
    """Single-joint sweeps of the independent joints, extreme combinations of independent joints whose moving parts are
    near each other, and the registered coupled sequences (coupled joints - those with an expr - are swept only along
    their sequences; a child of a coupled joint is swept at the parent's rest value)."""
    out = []
    for j in reg.joints.values():
        if _coupled(reg, j.name):
            continue
        if j.parent is not None:
            if _coupled(reg, j.parent):
                pvs = (reg.joints[j.parent].rest,)
            else:                     # child joints are swept together with their parent extremes as well
                pvs = (reg.joints[j.parent].lo, reg.joints[j.parent].hi)
            for pv in pvs:
                for v in j.samples(n):
                    out.append((f"{j.parent}={pv:.3f},{j.name}", {j.parent: pv, j.name: v}))
        for v in j.samples(n):
            out.append((j.name, {j.name: v}))
    # neighbouring independent joints at their extremes (e.g. aileron vs flap, ruddervator L vs R)
    movers = {name: [p.id for p in reg.moving_parts(name)] for name in reg.joints}
    names = [n_ for n_ in reg.joints if reg.joints[n_].parent is None and movers[n_] and not _coupled(reg, n_)]
    def swept_box(name):
        j = reg.joints[name]
        los, his = [], []
        for v in (j.lo, j.rest, j.hi):
            for pid in movers[name]:
                V = reg.posed_vertices(reg.parts[pid], {name: v})
                los.append(V.min(0))
                his.append(V.max(0))
        return np.min(los, 0), np.max(his, 0)
    sboxes = {n_: swept_box(n_) for n_ in names}
    for a, b in itertools.combinations(names, 2):
        if _boxes_overlap(sboxes[a], sboxes[b], pad=0.01):
            ja, jb = reg.joints[a], reg.joints[b]
            for va, vb in itertools.product((ja.lo, ja.hi), (jb.lo, jb.hi)):
                out.append((f"{a}+{b}", {a: va, b: vb}))
    for name, seq in reg.sequences.items():
        for k, st in enumerate(seq):
            out.append((f"seq:{name}[{k}]", st))
    return out


def check_swept(reg: Registry, n: int = 9, cache: _ManCache | None = None, tol: float = VOL_TOL) -> list[dict]:
    cache = cache or _ManCache(reg)
    out = []
    seen = set()
    for label, state in _states(reg, n):
        moving = {p.id for name in state for p in reg.moving_parts(name)}
        if not moving:
            continue
        posed = {pid: reg.posed_mesh(reg.parts[pid], state) for pid in moving}
        pboxes = {pid: m.bounds() for pid, m in posed.items()}
        pman = {pid: m.to_manifold() for pid, m in posed.items()}
        for pid in moving:
            for q in reg.parts:
                if q == pid or reg.parts[q].process == "consumable":
                    continue
                # the key holds the state values: a single-joint sweep has one label for all its samples (fix round
                # 3: only the first sample of every sweep was evaluated before)
                key = (label, tuple(sorted(state.items())), *sorted((pid, q)))
                if key in seen:
                    continue
                seen.add(key)
                qbox = pboxes[q] if q in moving else cache.box(q)
                if not _boxes_overlap(pboxes[pid], qbox):
                    continue
                if q in moving and reg.parts[q].joint == reg.parts[pid].joint:
                    continue                       # rigidly moving together: covered by the static check
                qman = pman[q] if q in moving else cache.man(q)
                v = pman[pid] ^ qman
                vol = 0.0 if v.is_empty() else float(v.volume())
                if vol > tol:
                    out.append(_violation("swept", sorted([pid, q]), round(vol * 1e9, 3), "<= 1 mm^3",
                                          f"state {label}: " + ", ".join(f"{k}={v_:.4g}" for k, v_ in state.items())))
    return _dedupe(out)


def _dedupe(viol: list[dict]) -> list[dict]:
    best = {}
    for v in viol:
        k = (v["check"], tuple(v["parts"]))
        if k not in best or v["value"] > best[k]["value"]:
            best[k] = v
    return list(best.values())


# =====================================================================================================================
# 4. clearances
# =====================================================================================================================
def _select(reg: Registry, sel) -> list[str]:
    """Selector: list of part ids, "group:<g>", "joint:<name>" (moving parts), "prefix:<id prefix>", "*"."""
    sels = sel if isinstance(sel, list) else [sel]
    out = []
    for s in sels:
        if s == "*":
            out += list(reg.parts)
        elif s.startswith("group:"):
            out += [p.id for p in reg.by_group(s[6:])]
        elif s.startswith("joint:"):
            out += [p.id for p in reg.moving_parts(s[6:])]
        elif s.startswith("prefix:"):
            out += [p for p in reg.parts if p.startswith(s[7:])]
        elif s in reg.parts:
            out.append(s)
    return sorted(set(out))


def check_clearances(reg: Registry, cache: _ManCache | None = None) -> list[dict]:
    """``spec.layout.clearances``: list of {name, a, b, min_mm, joints?: [...]} -> minimum gap between the two
    selections (rest pose, and over the listed joints' extremes if given; a coupled joint - Joint.expr - over the
    registered sequence states that set it instead, fix round 3 PK3-03). Pairs listed as contacts are skipped."""
    cache = cache or _ManCache(reg)
    rules = (reg.spec.get("layout", {}) or {}).get("clearances", []) or []
    out = []
    for r in rules:
        A, B = _select(reg, r["a"]), _select(reg, r["b"])
        need = float(r["min_mm"]) / 1000.0
        states = [{}]
        for jn in r.get("joints", []) or []:
            j = reg.joints.get(jn)
            if j is None:                          # rule names a joint no producer registered
                if A and B:                        # parts exist -> the swept part of the rule cannot be evaluated
                    out.append(_violation("clearance", [A[0], B[0]], None, f"joint {jn} registered",
                                          f"{r.get('name', '')}: joint not registered"))
                continue
            if _coupled(reg, jn):
                states += [st for st in _seq_states(reg, jn) if st not in states]
            else:
                states += [{jn: j.lo}, {jn: j.hi}]
        for st in states:
            moved = {p.id for name in st for p in reg.moving_parts(name)} if st else set()
            posed_b = {}
            for a in A:
                ma = reg.posed_mesh(reg.parts[a], st) if st else reg.parts[a].mesh
                ba = ma.bounds()
                mana = ma.to_manifold() if st else cache.man(a)
                for b in B:
                    if a == b or _allowed_contact(reg.parts[a], reg.parts[b]):
                        continue
                    if b in moved:             # B moves in this state too (sequence states pose several joints)
                        if b not in posed_b:
                            mb_ = reg.posed_mesh(reg.parts[b], st)
                            posed_b[b] = (mb_.bounds(), mb_.to_manifold())
                        bb, manb = posed_b[b]
                    else:
                        bb, manb = cache.box(b), cache.man(b)
                    if not _boxes_overlap(ba, bb, pad=need):
                        continue
                    gap = float(mana.min_gap(manb, need * 1.5 + 1e-4))
                    if gap < need - 1e-6:
                        out.append(_violation("clearance", [a, b], round(gap * 1000, 2), f">= {r['min_mm']} mm",
                                              f"{r.get('name', '')} {st or 'rest'}"))
    best = {}
    for v in out:                                  # keep the smallest gap per part pair
        k = tuple(sorted(v["parts"]))
        if k not in best or v["value"] is None or (best[k]["value"] is not None and v["value"] < best[k]["value"]):
            best[k] = v
    return list(best.values())


# =====================================================================================================================
# 5. thickness
# =====================================================================================================================
def check_thickness(reg: Registry, sample: bool = True, n: int = 300) -> list[dict]:
    procs = reg.spec.get("processes", {})
    out = []
    for p in reg.parts.values():
        if p.purchased or p.group == "hardware" or p.process == "consumable":
            continue
        proc = procs.get(p.process)
        if proc is None:
            out.append(_violation("thickness", [p.id], p.process, "known process", "process not in spec"))
            continue
        tmin = float(proc.get("min_thickness", 0.0))
        t = p.thickness
        if t is None and p.layup:
            from ..core.parts import layup_props
            t = layup_props(reg.spec, p.layup)["thickness"]
        if t is None:
            out.append(_violation("thickness", [p.id], None, f">= {tmin*1000:.2f} mm", "no declared thickness"))
            continue
        if t < tmin - 1e-9:
            out.append(_violation("thickness", [p.id], round(t * 1000, 3), f">= {tmin*1000:.2f} mm", "declared"))
        if sample and tmin > 0:
            w = G.wall_thickness_samples(p.mesh, n=n)
            w = w[np.isfinite(w)]
            if len(w):
                p05 = float(np.percentile(w, 5))
                if p05 < 0.9 * tmin:
                    out.append(_violation("thickness", [p.id], round(p05 * 1000, 3), f">= {0.9*tmin*1000:.2f} mm",
                                          "sampled 5th percentile wall"))
    return out


# =====================================================================================================================
# 6. fasteners: edge distance and pitch
# =====================================================================================================================
def _edge_distance(f, outline_list) -> float:
    """Shortest in-plane distance (perpendicular to the fastener axis) from the fastener centre line to the part's
    outline polylines, using outline points within 10 D of the fastener along the axis."""
    c, a = f.position, f.axis
    best = math.inf
    for poly in outline_list:
        P = np.asarray(poly, float)
        if len(P) < 2:
            continue
        rel = P - c
        ax = rel @ a
        near = np.abs(ax) < 10 * f.d + 0.05
        if not near.any():
            continue
        Q = rel - np.outer(ax, a)                  # projected onto the plane through c perpendicular to the axis
        for i in range(len(Q) - 1):
            if not (near[i] or near[i + 1]):
                continue
            p0, p1 = Q[i], Q[i + 1]
            d = p1 - p0
            L2 = float(d @ d)
            t = 0.0 if L2 < 1e-18 else float(np.clip(-(p0 @ d) / L2, 0.0, 1.0))
            best = min(best, float(np.linalg.norm(p0 + t * d)))
    return best


def _perp_basis(a):
    ref = np.array([0.0, 0.0, 1.0]) if abs(a[2]) < 0.9 else np.array([1.0, 0.0, 0.0])
    e1 = np.cross(a, ref)
    e1 /= np.linalg.norm(e1)
    return e1, np.cross(a, e1)


def _edge_distance_mesh(f, man, extent: float, n_dir: int = 36):
    """Edge distance measured on the geometry: radial rays from the fastener axis at the mid-plane of the part's
    material (found with four probe lines parallel to the axis just outside the hole). The first crossing is the hole
    wall (if the hole was cut), the next one the nearest edge/cut-out of the part. Returns (edge distance, pierced);
    pierced is False when the fastener does not pass through the part's material near its axis."""
    from ..core.geom import ray_hits
    from ..design.fastener_catalog import clearance, size_from_spec
    a = f.axis
    c = f.position
    size = size_from_spec(f.spec) or round(f.d * 1000)
    r_h = 0.5 * clearance(size) if f.kind not in ("pin", "clevis_pin") else 0.5 * f.d * 1.02
    e1, e2 = _perp_basis(a)
    lo = -0.02 - 2 * f.d
    hi = (f.grip if f.grip is not None else f.length) + 0.02 + 2 * f.d
    mids = []
    for d in (e1, -e1, e2, -e2):
        o = c + 1.6 * r_h * d
        h = ray_hits(man, o + lo * a, o + hi * a) + lo
        for i in range(0, len(h) - 1, 2):                      # (enter, exit) pairs along the probe line
            mids.append(0.5 * (h[i] + h[i + 1]))
    if not mids:
        return math.inf, False
    t_mid = float(np.median(mids))
    p0 = c + t_mid * a
    best = math.inf
    for th in np.linspace(0.0, 2 * math.pi, n_dir, endpoint=False):
        d = math.cos(th) * e1 + math.sin(th) * e2
        h = ray_hits(man, p0, p0 + extent * d)
        if not len(h):
            continue
        ed = h[1] if (h[0] < 1.6 * r_h and len(h) > 1) else (math.inf if h[0] < 1.6 * r_h else h[0])
        best = min(best, float(ed))
    return best, True


def check_fasteners(reg: Registry) -> list[dict]:
    """Edge distance (>= 2.0 D metal, >= 2.5 D composite) for every fastener in every joined part — from the part
    ``outline`` if given, otherwise measured on the mesh — that the fastener actually pierces each joined part, and
    the minimum pitch (3 D) between neighbouring fasteners in a part."""
    out = []
    by_part = defaultdict(list)
    mans = {}
    for f in reg.fasteners():
        for pid in f.joins:
            if pid not in reg.parts:
                out.append(_violation("fastener", [f.id], pid, "joined part exists", "unknown part in joins"))
                continue
            by_part[pid].append(f)
            p = reg.parts[pid]
            k = COMPOSITE_EDGE if _material_kind(reg, p) == "composite" else METAL_EDGE
            if p.outline:
                ed = _edge_distance(f, p.outline)
            else:
                if pid not in mans:
                    lo, hi = p.mesh.bounds()
                    mans[pid] = (p.mesh.to_manifold(), float(np.linalg.norm(hi - lo)) + 0.01)
                man, ext = mans[pid]
                ed, pierced = _edge_distance_mesh(f, man, ext)
                if not pierced:
                    out.append(_violation("fastener_miss", [f.id, pid], None, "fastener passes through the part",
                                          f.spec))
                    continue
            if ed < k * f.d - 1e-6:
                out.append(_violation("edge_distance", [f.id, pid], round(ed * 1000, 2),
                                      f">= {k:.1f} D = {k*f.d*1000:.1f} mm", f.spec))
    for pid, fs in by_part.items():
        if len(fs) < 2:
            continue
        P = np.array([f.position for f in fs])
        D = np.array([f.d for f in fs])
        for i in range(len(fs)):
            d = np.linalg.norm(P - P[i], axis=1)
            d[i] = np.inf
            j = int(np.argmin(d))
            need = 3.0 * max(D[i], D[j])          # minimum pitch 3 D (4-6 D preferred)
            if d[j] < need - 1e-6 and fs[i].id < fs[j].id:
                out.append(_violation("pitch", [fs[i].id, fs[j].id, pid], round(d[j] * 1000, 2),
                                      f">= 3 D = {need*1000:.1f} mm"))
    return out


# =====================================================================================================================
# 7. attachment graph
# =====================================================================================================================
def check_attachment(reg: Registry) -> list[dict]:
    root = (reg.spec.get("layout", {}) or {}).get("root_part")
    if not root or root not in reg.parts:
        return [_violation("attachment", [], root, "spec.layout.root_part exists")]
    adj = defaultdict(set)
    for p in reg.parts.values():
        if p.parent:
            adj[p.id].add(p.parent)
            adj[p.parent].add(p.id)
        for c in p.contacts:
            adj[p.id].add(c)
            adj[c].add(p.id)
    for f in reg.fasteners():
        ids = list(f.joins)
        for a, b in zip(ids, ids[1:]):
            adj[a].add(b)
            adj[b].add(a)
    seen = {root}
    dq = deque([root])
    while dq:
        x = dq.popleft()
        for y in adj[x]:
            if y not in seen:
                seen.add(y)
                dq.append(y)
    return [_violation("attachment", [p], "not connected", f"reachable from {root}") for p in reg.parts
            if p not in seen]


# =====================================================================================================================
# driver
# =====================================================================================================================
CHECKS = {
    "mesh": lambda reg, c, q: check_meshes(reg, self_intersect=not q),
    "static": lambda reg, c, q: check_static(reg, c),
    "swept": lambda reg, c, q: check_swept(reg, n=5 if q else 9, cache=c),
    "clearance": lambda reg, c, q: check_clearances(reg, c),
    "thickness": lambda reg, c, q: check_thickness(reg, sample=not q),
    "fastener": lambda reg, c, q: check_fasteners(reg),
    "attachment": lambda reg, c, q: check_attachment(reg),
}

TR = {"mesh": "Ağ geçerliliği", "static": "Durağan girişim", "swept": "Hareket taraması", "clearance": "Açıklıklar",
      "thickness": "Et kalınlığı", "fastener": "Bağlantı elemanları", "attachment": "Bağlantı grafiği"}


def run_all(reg: Registry, quick: bool = False, only: list[str] | None = None, write: bool = True,
            focus: set[str] | None = None) -> dict:
    """Run the checks. ``focus`` (part ids) keeps only violations that involve at least one focus part (a module
    author checks their own parts against everything built so far)."""
    cache = _ManCache(reg)
    res, times = {}, {}
    for name, fn in CHECKS.items():
        if only and name not in only:
            continue
        t0 = time.time()
        res[name] = fn(reg, cache, quick)
        if focus is not None:
            res[name] = [v for v in res[name] if any(pid in focus for pid in v["parts"]) or
                         any(isinstance(pid, str) and pid.startswith("YK250-HW-") and any(f in pid for f in focus)
                             for pid in v["parts"])]
        times[name] = round(time.time() - t0, 1)
    summary = {"parts": len(reg.parts), "joints": len(reg.joints), "fasteners": len(reg.fasteners()),
               "violations": {k: len(v) for k, v in res.items()}, "seconds": times, "details": res,
               "ok": all(len(v) == 0 for v in res.values())}
    if write:
        from ..core.spec import OUT_DIR
        OUT_DIR.mkdir(parents=True, exist_ok=True)
        (OUT_DIR / "checks.json").write_text(json.dumps(summary, indent=1, default=str), encoding="utf-8")
        (OUT_DIR / "checks.md").write_text(report_md(summary), encoding="utf-8")
    return summary


def report_md(S: dict) -> str:
    L = ["# YK-250 tasarım kuralı denetimleri", "",
         f"Parça: {S['parts']} · eklem: {S['joints']} · bağlantı elemanı: {S['fasteners']} · sonuç: "
         f"**{'TEMİZ' if S['ok'] else 'İHLAL VAR'}**", "", "| Denetim | İhlal | Süre (s) |", "|---|---:|---:|"]
    for k, n in S["violations"].items():
        L.append(f"| {TR.get(k, k)} | {n} | {S['seconds'].get(k, '')} |")
    for k, vs in S["details"].items():
        if not vs:
            continue
        L += ["", f"## {TR.get(k, k)}", "", "| Parçalar | Değer | Sınır | Ayrıntı |", "|---|---|---|---|"]
        for v in vs[:200]:
            L.append(f"| {', '.join(v['parts'])} | {v['value']} | {v['limit']} | {v['detail']} |")
        if len(vs) > 200:
            L.append(f"| … {len(vs) - 200} more | | | |")
    return "\n".join(L) + "\n"


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description="YK-250 design-rule checks")
    ap.add_argument("--quick", action="store_true", help="skip self-intersection and thickness sampling, 5 sweep "
                                                          "samples")
    ap.add_argument("--only", default="", help="comma list of: " + ",".join(CHECKS))
    ap.add_argument("--modules", default="", help="build only these producer modules (comma list, hardware is "
                                                  "appended), e.g. chassis,wing")
    ap.add_argument("--focus", default="", help="report only violations involving parts whose id starts with one of "
                                                "these comma-separated prefixes, e.g. YK250-WG,YK250-FC")
    ap.add_argument("--no-write", action="store_true", help="do not overwrite out/checks.md|json")
    a = ap.parse_args(argv)
    from ..core.assemble import build_registry
    mods = [m for m in a.modules.split(",") if m]
    if mods and "hardware" not in mods:
        mods.append("hardware")
    reg = build_registry(modules=mods or None, strict=not mods)
    focus = None
    if a.focus:
        pre = tuple(x for x in a.focus.split(",") if x)
        focus = {pid for pid in reg.parts if pid.startswith(pre)}
    S = run_all(reg, quick=a.quick, only=[x for x in a.only.split(",") if x] or None, write=not a.no_write,
                focus=focus)
    print(json.dumps({"ok": S["ok"], "violations": S["violations"], "seconds": S["seconds"]}))
    return 0 if S["ok"] else 1


if __name__ == "__main__":
    sys.exit(main())
