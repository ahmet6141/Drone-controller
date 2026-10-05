"""Dimensioned general-arrangement (GA) 3-view drawing -> out/ga_3view.svg and out/ga_3view.pdf (A3 landscape).

Two layouts, the one giving the larger standard scale is used (or ``spec.display.drawing.layout``):

* ``A`` (long fuselage, third-angle): side view (port side, nose left) bottom-left, plan view (from above, nose left,
  starboard up) above it, front view (from ahead) to the right of the side view;
* ``B`` (span > length): plan view (from above, nose up, starboard right) bottom-left, front view (from ahead,
  aligned on the span extents) above it, side view (port side, nose left) to the right of the front view, title
  block and data table below the side view.
 Outlines are exact silhouettes: every part mesh is
projected with manifold3d (``Manifold.project``) and the projections are unioned per view. Overall dimensions are
measured from the geometry; extra dimensions come from ``spec.display.drawing.dimensions``:
``[{view: plan|side|front, axis: h|v, a: [x, y], b: [x, y], offset: m, label: str}]`` in view coordinates (m).

CLI:  python3 -m ucav250.outputs.drawings
"""
from __future__ import annotations

import datetime as _dt
import sys

import manifold3d as m3
import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402
from matplotlib.patches import Polygon as MplPolygon  # noqa: E402

from ..core.geom import Mesh  # noqa: E402
from ..core.parts import Registry  # noqa: E402

# view -> (u, v, w) as linear maps of (X=FS, Y=BL, Z=WL); w points toward the viewer
VIEWS = {
    "plan": (np.array([1, 0, 0.]), np.array([0, 1, 0.]), np.array([0, 0, 1.])),
    "side": (np.array([1, 0, 0.]), np.array([0, 0, 1.]), np.array([0, -1, 0.])),
    "front": (np.array([0, -1, 0.]), np.array([0, 0, 1.]), np.array([-1, 0, 0.])),
    "plan_up": (np.array([0, 1, 0.]), np.array([-1, 0, 0.]), np.array([0, 0, 1.])),
}
SCALES = [5, 10, 15, 20, 25, 30, 40, 50]
A3 = (0.420, 0.297)
DEFAULT_GROUPS = ["shell", "wing", "tail", "controls", "propulsion", "gear", "payload", "systems"]


def _project(mesh: Mesh, view: str) -> m3.CrossSection:
    u, v, w = VIEWS[view]
    M = np.vstack([u, v, w])                       # rows
    V = mesh.V @ M.T
    F = mesh.F if np.linalg.det(M) > 0 else mesh.F[:, ::-1]
    man = Mesh(V, F).to_manifold()
    return man.project()


def silhouettes(reg: Registry, groups: list[str]) -> dict[str, dict]:
    """Per view: {"union": CrossSection, "parts": {pid: CrossSection}} for parts in ``groups``."""
    out = {}
    for view in VIEWS:
        parts = {}
        for p in reg.parts.values():
            if p.group not in groups or p.process == "consumable":
                continue
            try:
                cs = _project(p.mesh, view)
            except Exception:
                continue
            if cs.area() > 1e-8:
                parts[p.id] = cs
        union = m3.CrossSection.batch_boolean(list(parts.values()), m3.OpType.Add) if parts else m3.CrossSection()
        out[view] = {"union": union, "parts": parts}
    return out


def _polys(cs: m3.CrossSection) -> list[np.ndarray]:
    return [np.asarray(p, float) for p in cs.to_polygons() if len(p) >= 3]


def _bounds(cs: m3.CrossSection):
    b = cs.bounds()
    return np.array(b[:2]), np.array(b[2:])


def _dim(ax, p0, p1, offset, label, horizontal: bool, s: float):
    """Dimension line between p0 and p1 (paper metres), offset perpendicular, with extension lines."""
    p0, p1 = np.asarray(p0, float), np.asarray(p1, float)
    if horizontal:
        y = (max(p0[1], p1[1]) if offset > 0 else min(p0[1], p1[1])) + offset
        a, b = np.array([p0[0], y]), np.array([p1[0], y])
        ax.plot([p0[0], p0[0]], [p0[1], y + 0.002 * np.sign(offset)], lw=0.3, color="k")
        ax.plot([p1[0], p1[0]], [p1[1], y + 0.002 * np.sign(offset)], lw=0.3, color="k")
        rot = 0
        txt = ((a + b) / 2) + np.array([0, 0.0018 if offset > 0 else -0.0045])
    else:
        x = (max(p0[0], p1[0]) if offset > 0 else min(p0[0], p1[0])) + offset
        a, b = np.array([x, p0[1]]), np.array([x, p1[1]])
        ax.plot([p0[0], x + 0.002 * np.sign(offset)], [p0[1], p0[1]], lw=0.3, color="k")
        ax.plot([p1[0], x + 0.002 * np.sign(offset)], [p1[1], p1[1]], lw=0.3, color="k")
        rot = 90
        txt = ((a + b) / 2) + np.array([-0.0022 if offset < 0 else 0.0022, 0])
    ax.annotate("", xy=b, xytext=a, arrowprops=dict(arrowstyle="<|-|>", lw=0.4, mutation_scale=5, color="k"))
    ax.text(*txt, label, fontsize=5.5, ha="center", va="center", rotation=rot,
            bbox=dict(boxstyle="square,pad=0.08", fc="white", ec="none"))


def draw(reg: Registry, out_base=None) -> dict:
    from ..core.spec import OUT_DIR
    spec = reg.spec
    dcfg = ((spec.get("display", {}) or {}).get("drawing", {}) or {})
    groups = dcfg.get("groups", DEFAULT_GROUPS)
    sil = silhouettes(reg, groups)
    ext = {v: _bounds(sil[v]["union"]) for v in VIEWS}
    L = ext["side"][1][0] - ext["side"][0][0]
    H = ext["side"][1][1] - ext["side"][0][1]
    B = ext["plan"][1][1] - ext["plan"][0][1]
    usable_w, usable_h = A3[0] - 0.045, A3[1] - 0.075
    gap = 0.035

    def fits(layout, k):
        s_ = 1.0 / k
        if layout == "A":
            return (L + B) * s_ + gap + 0.015 <= usable_w and (B + H) * s_ + gap + 0.015 <= usable_h
        return (B + L) * s_ + gap + 0.015 <= usable_w and (L + H) * s_ + gap + 0.015 <= usable_h + 0.03

    choice = dcfg.get("layout")
    cands = [choice] if choice in ("A", "B") else ["A", "B"]
    best = None
    for lay in cands:
        k = next((k for k in SCALES if fits(lay, k)), SCALES[-1])
        if best is None or k < best[1]:
            best = (lay, k)
    layout, k = best
    s = 1.0 / k
    fig = plt.figure(figsize=(A3[0] / 0.0254, A3[1] / 0.0254))
    ax = fig.add_axes([0, 0, 1, 1])
    ax.set_xlim(0, A3[0])
    ax.set_ylim(0, A3[1])
    ax.set_aspect("equal")
    ax.axis("off")
    ax.add_patch(plt.Rectangle((0.01, 0.01), A3[0] - 0.02, A3[1] - 0.02, fill=False, lw=0.8))
    x0, y0 = 0.035, 0.07
    if layout == "A":
        views = ["side", "plan", "front"]
        origins = {
            "side": np.array([x0, y0]) - ext["side"][0] * s,
            "plan": np.array([x0, y0 + H * s + gap]) - ext["plan"][0] * s,
            "front": np.array([x0 + L * s + gap, y0]) - np.array([ext["front"][0][0], ext["side"][0][1]]) * s,
        }
    else:
        views = ["plan_up", "front", "side"]
        pu = ext["plan_up"]
        content_h = (pu[1][1] - pu[0][1] + H) * s + gap + 0.012
        y0 = 0.045 + max(0.0, (A3[1] - 0.035 - 0.045 - content_h) / 2)   # centre the views vertically
        y_front = y0 + (pu[1][1] - pu[0][1]) * s + gap
        origins = {
            "plan_up": np.array([x0, y0]) - pu[0] * s,
            "front": np.array([x0, y_front]) - np.array([ext["front"][0][0], ext["side"][0][1]]) * s,
            "side": np.array([x0 + B * s + gap, y_front]) - ext["side"][0] * s,
        }
    for view in views:
        o = origins[view]
        for pid, cs in sil[view]["parts"].items():
            for P in _polys(cs):
                ax.add_patch(MplPolygon(o + P * s, closed=True, fill=False, lw=0.18, ec="#555555"))
        for P in _polys(sil[view]["union"]):
            ax.add_patch(MplPolygon(o + P * s, closed=True, fill=False, lw=0.6, ec="k"))
    names = {"side": "YAN GÖRÜNÜŞ (iskele)", "plan": "ÜST GÖRÜNÜŞ", "plan_up": "ÜST GÖRÜNÜŞ (burun yukarı)",
             "front": "ÖN GÖRÜNÜŞ"}

    def vb(view):
        if view == "front":
            return np.array([ext["front"][0][0], ext["side"][0][1]]), ext["front"][1]
        return ext[view]
    for view in views:
        lo, hi = vb(view)
        ax.text(*(origins[view] + np.array([lo[0], hi[1]]) * s + np.array([0, 0.008])), names[view], fontsize=7,
                weight="bold")
    # overall dimensions (measured)
    fmt = lambda v: f"{v*1000:,.0f}".replace(",", " ")  # noqa: E731
    o = origins["side"]
    lo, hi = ext["side"]
    _dim(ax, o + np.array([lo[0], lo[1]]) * s, o + np.array([hi[0], lo[1]]) * s, -0.012, fmt(L) + " (toplam boy)",
         True, s)
    _dim(ax, o + np.array([hi[0], lo[1]]) * s, o + np.array([hi[0], hi[1]]) * s, 0.012, fmt(H), False, s)
    if layout == "A":
        o = origins["plan"]
        lo, hi = ext["plan"]
        _dim(ax, o + np.array([lo[0], lo[1]]) * s, o + np.array([lo[0], hi[1]]) * s, -0.012, fmt(B) + " (açıklık)",
             False, s)
    else:
        o = origins["plan_up"]
        lo, hi = ext["plan_up"]
        _dim(ax, o + np.array([lo[0], lo[1]]) * s, o + np.array([hi[0], lo[1]]) * s, -0.010, fmt(B) + " (açıklık)",
             True, s)
        _dim(ax, o + np.array([hi[0], lo[1]]) * s, o + np.array([hi[0], hi[1]]) * s, 0.010, fmt(L), False, s)
    for d in dcfg.get("dimensions", []) or []:
        if d["view"] not in origins:
            continue
        o = origins[d["view"]]
        a_ = o + np.asarray(d["a"], float) * s
        b_ = o + np.asarray(d["b"], float) * s
        _dim(ax, a_, b_, float(d.get("offset", 0.01)), d["label"], d.get("axis", "h") == "h", s)
    table = dcfg.get("data_table", []) or []
    if table and layout == "B":
        tx = origins["side"][0] + ext["side"][0][0] * s
        ty = y_front - 0.028
        ax.text(tx, ty, "TEKNİK VERİLER", fontsize=6.5, weight="bold")
        for i, (lab, val) in enumerate(table):
            ax.text(tx, ty - 0.0085 * (i + 1), str(lab), fontsize=5.5)
            ax.text(tx + 0.055, ty - 0.0085 * (i + 1), str(val), fontsize=5.5)
    # title block
    meta = spec.get("meta", {}) or {}
    tb = [(meta.get("name", "YK-250"), 9, "bold"), ("Genel yerleşim (GA) — 3 görünüş", 7, "normal"),
          (f"Ölçek 1:{k} (A3) · ölçüler mm · " + ("üçüncü açı izdüşümü" if layout == "A" else
                                                  "görünüşler hizalı, aynı ölçekte"), 6, "normal"),
          (f"Kaynak: ucav250/spec.yaml · {_dt.date.today().isoformat()}", 5.5, "normal")]
    bx, by = A3[0] - 0.14, 0.012
    ax.add_patch(plt.Rectangle((bx, by), 0.128, 0.045, fill=False, lw=0.6))
    for i, (t, fs, w) in enumerate(tb):
        ax.text(bx + 0.004, by + 0.036 - i * 0.0095, t, fontsize=fs, weight=w)
    out_base = out_base or (OUT_DIR / "ga_3view")
    OUT_DIR.mkdir(parents=True, exist_ok=True)
    fig.savefig(str(out_base) + ".svg")
    fig.savefig(str(out_base) + ".pdf")
    fig.savefig(str(out_base) + ".png", dpi=200)
    plt.close(fig)
    return {"scale": f"1:{k}", "layout": layout, "length_m": L, "span_m": B, "height_m": H, "files": [str(out_base) + e for e in
                                                                                  (".svg", ".pdf", ".png")]}


def main(argv=None) -> int:
    from ..core.assemble import build_registry
    print(draw(build_registry()))
    return 0


if __name__ == "__main__":
    sys.exit(main())
