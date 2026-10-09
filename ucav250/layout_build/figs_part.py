# =====================================================================================================================
# figures
# =====================================================================================================================
GROUP_COL = {"systems": "#2f7fbf", "payload": "#8e44ad", "propulsion": "#c0392b", "fuel": "#2e86c1", "controls": "#16a085",
             "gear": "#7f8c8d", "chassis": "#555555"}


def _oml_side(ctx: Ctx):
    xs = np.linspace(ctx.af.fus.x0 + 1e-4, ctx.af.fus.x1 - 1e-4, 300)
    return xs, np.array([ctx.z_top(x) for x in xs]), np.array([ctx.z_bot(x) for x in xs])


def _oml_top(ctx: Ctx):
    xs = np.linspace(ctx.af.fus.x0 + 1e-4, ctx.af.fus.x1 - 1e-4, 300)
    return xs, np.array([float(ctx.af.sec(x)[0][0]) for x in xs])


def _rect(ax, b, ix, iy, **kw):
    from matplotlib.patches import Rectangle
    b = np.asarray(b, float)
    lo, hi = np.minimum(b[0], b[1]), np.maximum(b[0], b[1])
    ax.add_patch(Rectangle((lo[ix], lo[iy]), hi[ix] - lo[ix], hi[iy] - lo[iy], **kw))


def _poly_obb(ax, o: OBB, ix, iy, **kw):
    from matplotlib.patches import Polygon
    from scipy.spatial import ConvexHull
    C = o.corners()[:, [ix, iy]]
    try:
        h = ConvexHull(C)
        ax.add_patch(Polygon(C[h.vertices], closed=True, **kw))
    except Exception:
        pass


def _draw_layout(ax, ctx: Ctx, view: str, objs: list, detail: bool = True):
    """view 'side' (x-z) or 'top' (x-y, starboard + port body)."""
    from matplotlib.patches import Circle
    L, S = ctx.L, ctx.S
    ix, iy = (0, 2) if view == "side" else (0, 1)
    if view == "side":
        xs, zt, zb = _oml_side(ctx)
        ax.fill_between(xs, zb, zt, color="#f2f2f2", zorder=0)
        ax.plot(xs, zt, "k-", lw=1.0)
        ax.plot(xs, zb, "k-", lw=1.0)
        ax.plot(xs, [float(ctx.af.sec(x)[3][0]) for x in xs], color="#bbbbbb", lw=0.6, ls="--")
    else:
        xs, hw = _oml_top(ctx)
        ax.fill_between(xs, -hw, hw, color="#f2f2f2", zorder=0)
        ax.plot(xs, hw, "k-", lw=1.0)
        ax.plot(xs, -hw, "k-", lw=1.0)
        W = S["wing"]["sections"]
        le = np.array([[s_["x_le"], s_["y"]] for s_ in W])
        te = np.array([[s_["x_le"] + s_["chord"], s_["y"]] for s_ in W])
        for sg in (1, -1):
            ax.plot(le[:, 0], sg * le[:, 1], "k-", lw=0.8)
            ax.plot(te[:, 0], sg * te[:, 1], "k-", lw=0.8)
            ax.plot([le[-1, 0], te[-1, 0]], [sg * le[-1, 1], sg * te[-1, 1]], "k-", lw=0.8)
        for name in ("fin", "stabilator", "stabilator_stub"):
            T = S["tail"]["surfaces"][name]["sections"]
            le_ = np.array([[s_["x_le"], s_["y"]] for s_ in T])
            te_ = np.array([[s_["x_le"] + s_["chord"], s_["y"]] for s_ in T])
            for sg in (1, -1):
                ax.plot(le_[:, 0], sg * le_[:, 1], color="#666666", lw=0.6)
                ax.plot(te_[:, 0], sg * te_[:, 1], color="#666666", lw=0.6)
                ax.plot([le_[0, 0], te_[0, 0]], [sg * le_[0, 1], sg * te_[0, 1]], color="#666666", lw=0.6)
                ax.plot([le_[-1, 0], te_[-1, 0]], [sg * le_[-1, 1], sg * te_[-1, 1]], color="#666666", lw=0.6)
    # stations
    for s_ in L["stations"]:
        x = float(s_["x"])
        if view == "side":
            ax.plot([x, x], [ctx.z_bot(x), ctx.z_top(x)], color="#1a5276", lw=1.4 if s_["type"] != "ring" else 0.9,
                    ls="-" if s_["type"] != "ring" else "--", zorder=3)
            ax.text(x, ctx.z_top(x) + 0.012, s_["id"], rotation=90, fontsize=6, ha="center", va="bottom",
                    color="#1a5276")
        else:
            yy = np.linspace(-0.4, 0.4, 41)
            hw_ = float(ctx.af.sec(x)[0][0])
            yy = yy[np.abs(yy) <= hw_]
            ax.plot([station_x(s_, y) for y in yy], yy, color="#1a5276", lw=1.2, zorder=3)
    # members
    for o in objs:
        if o.kind == "structure" and o.id.startswith("M-"):
            for p in o.prims:
                if isinstance(p, OBB):
                    _poly_obb(ax, p, ix, iy, fc="#a6acaf", ec="#5d6d7e", lw=0.4, alpha=0.55, zorder=2)
                elif isinstance(p, Capsule):
                    ax.plot([p.p0[ix], p.p1[ix]], [p.p0[iy], p.p1[iy]], color="#5d6d7e", lw=1.6, zorder=2)
        if o.kind == "structure" and o.id.startswith("F-"):
            for p in o.prims:
                if isinstance(p, OBB):
                    _poly_obb(ax, p, ix, iy, fc="#f5b041", ec="#9c640c", lw=0.5, zorder=5)
                elif isinstance(p, Cyl):
                    lo, hi = p.bounds()
                    _rect(ax, [lo, hi], ix, iy, fc="#f5b041", ec="#9c640c", lw=0.5, zorder=5)
    # fuel
    for o in objs:
        if o.kind == "fuel":
            P = o.prims[0].samples()
            if len(P):
                ax.scatter(P[:, ix], P[:, iy], s=1.0, c="#5dade2", alpha=0.25, lw=0, zorder=1)
    # zones
    Zp = L["zones_preliminary"]
    for k, col in (("payload_bay", "#d7bde2"), ("parachute_bay", "#f9e79f"), ("mission_computer", "#d6eaf8")):
        b = np.asarray(Zp[k]["box"], float)
        bb = b.copy()
        if view == "top" and Zp[k].get("symmetric"):
            bb[0][1] = -b[1][1]
        _rect(ax, bb, ix, iy, fc=col, ec="none", alpha=0.6, zorder=1)
    for o in objs:
        if o.kind in ("content",) and o.id.startswith(("EQ-", "ACT-")):
            col = GROUP_COL.get(o.group, "#2f7fbf")
            for p in o.prims:
                _poly_obb(ax, p, ix, iy, fc=col, ec="k", lw=0.3, alpha=0.75, zorder=4) if isinstance(p, OBB) else None
        if o.kind == "harness":
            for p in o.prims:
                ax.plot([p.p0[ix], p.p1[ix]], [p.p0[iy], p.p1[iy]], color="#e67e22", lw=1.3, zorder=6)
        if o.kind in ("engine", "exhaust"):
            for p in o.prims:
                _poly_obb(ax, p, ix, iy, fc="#f1948a" if o.kind == "engine" else "#e74c3c", ec="#922b21", lw=0.4,
                          alpha=0.45, zorder=3)
        if o.kind == "mount":
            for p in o.prims:
                ax.plot([p.p0[ix], p.p1[ix]], [p.p0[iy], p.p1[iy]], color="#1e8449", lw=1.2, zorder=6)
        if o.id == "DUCT-COOLING" and view == "side":
            for p in o.prims[:len(o.prims) // 3]:
                ax.plot([p.p0[ix], p.p1[ix]], [p.p0[iy], p.p1[iy]], color="#85c1e9", lw=6, alpha=0.5, zorder=2)
        if o.kind == "gear":
            for p in o.prims:
                lo, hi = p.bounds()
                _rect(ax, [lo, hi], ix, iy, fc="none", ec="#424949", lw=0.8, ls="-", zorder=5)
    # turret
    T = S["payload"]["turret"]
    r = 0.5 * float(T["growth_envelope"]["diameter"])
    xc = float(T["bay_center_x"])
    if view == "side":
        ax.add_patch(Circle((xc, float(T["ball_center_retracted_z"])), r, fc="#bb8fce", ec="#6c3483", lw=0.6,
                            alpha=0.6, zorder=5))
        ax.add_patch(Circle((xc, float(T["ball_center_extended_z"])), r, fc="none", ec="#6c3483", lw=0.6, ls="--",
                            zorder=5))
    else:
        ax.add_patch(Circle((xc, 0.0), r, fc="#bb8fce", ec="#6c3483", lw=0.6, alpha=0.6, zorder=5))
    # gear down (side): tyres at rest
    if view == "side":
        for jn, sd in (("main", "R"), ("nose", "")):
            g = gear_prims(ctx, jn, 0.0, sd or "R")
            c = g["tyre"].c
            ax.add_patch(Circle((c[0], c[2]), g["tyre"].R, fc="none", ec="#424949", lw=0.8, ls="--", zorder=5))
            ax.plot([g["leg"].p0[0], g["leg"].p1[0]], [g["leg"].p0[2], g["leg"].p1[2]], color="#424949", lw=1.2,
                    ls="--")
        ko = next(k for k in L["keep_outs"] if k["id"] == "KO-PROP")
        c, a, R = np.asarray(ko["centre"]), np.asarray(ko["axis"]), float(ko["radius"])
        e = np.array([-a[2], 0.0, a[0]])
        ax.plot([c[0] - R * e[0], c[0] + R * e[0]], [c[2] - R * e[2], c[2] + R * e[2]], color="#566573", lw=2.0,
                alpha=0.5)
        br = L["chassis"]["parachute"]["bridle"]
        cg = br["cg_used"]
        ax.plot([cg[0]], [cg[2]], marker="o", ms=7, mfc="w", mec="k", zorder=8)
        ax.text(cg[0] + 0.03, cg[2] + 0.03, "AM (MTOM)", fontsize=7)
    else:
        for pn in L["chassis"]["wing_joint"]["main_spar"]["pins"]:
            p = pn["position"]
            for sg in (1, -1):
                ax.plot([p[0]], [sg * p[1]], marker="o", ms=3.5, color="#922b21", zorder=8)
        for k in ("aileron_R", "flap_R"):
            j = next(j_ for j_ in L["mechanisms"]["joints"] if j_["name"] == k)
            o, a = np.asarray(j["origin"]), np.asarray(j["axis"])
            c = S["wing"]["controls"][k.split("_")[0]]
            b2 = 0.5 * float(S["wing"]["span"])
            y0, y1 = float(c["eta0"]) * b2, float(c["eta1"]) * b2
            t0, t1 = (y0 - o[1]) / a[1], (y1 - o[1]) / a[1]
            for sg in (1, -1):
                ax.plot([o[0] + t0 * a[0], o[0] + t1 * a[0]], [sg * (o[1] + t0 * a[1]), sg * (o[1] + t1 * a[1])],
                        color="#16a085", lw=1.0, ls="--")


def write_figures(ctx: Ctx, res: dict, fig_dir) -> dict:
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.lines import Line2D
    from matplotlib.patches import Patch
    fig_dir = Path(fig_dir)
    fig_dir.mkdir(parents=True, exist_ok=True)
    objs = layout_objects(ctx)
    out = {}
    leg = [Line2D([0], [0], color="#1a5276", lw=1.4, label="çerçeve / perde"),
           Patch(fc="#a6acaf", ec="#5d6d7e", label="şasi elemanı"), Patch(fc="#f5b041", label="bağlantı"),
           Patch(fc="#5dade2", alpha=0.5, label="yakıt hücresi"), Patch(fc="#2f7fbf", label="teçhizat"),
           Patch(fc="#bb8fce", label="taret (E180 zarfı)"), Line2D([0], [0], color="#e67e22", lw=1.3, label="kablo demeti"),
           Patch(fc="#f1948a", label="motor / egzoz zarfı"), Line2D([0], [0], color="#1e8449", lw=1.2, label="motor bağlantı kafesi"),
           Patch(fc="#d7bde2", label="faydalı yük bölmesi"), Patch(fc="#f9e79f", label="paraşüt bölmesi")]
    # ---- side
    fig, ax = plt.subplots(figsize=(17, 6.2))
    _draw_layout(ax, ctx, "side", objs)
    ax.set_aspect("equal")
    ax.set_xlim(-0.15, 4.75)
    ax.set_ylim(-0.48, 0.55)
    ax.set_xlabel("x (m, burundan geriye)")
    ax.set_ylabel("z (m)")
    ax.set_title("YK-250 HANÇER — yan kesit yerleşimi (y = 0 düzlemine izdüşüm): çerçeveler, şasi, bölmeler, teçhizat, "
                 "takım (açık: kesik çizgi, toplanmış: düz), taret", fontsize=10)
    ax.legend(handles=leg, loc="upper left", fontsize=7, ncol=4, frameon=True)
    ax.grid(alpha=0.25)
    fig.tight_layout()
    p = fig_dir / "yk250_layout_side.png"
    fig.savefig(p, dpi=125)
    plt.close(fig)
    out["side"] = p
    # ---- top
    fig, ax = plt.subplots(figsize=(15, 9.5))
    _draw_layout(ax, ctx, "top", objs)
    ax.set_aspect("equal")
    ax.set_xlim(-0.15, 4.75)
    ax.set_ylim(-1.0, 3.75)
    ax.set_xlabel("x (m)")
    ax.set_ylabel("y (m, sancak +)")
    ax.set_title("YK-250 HANÇER — üst görünüş yerleşimi: ok açılı kiriş çerçeveleri, ana pimler (kırmızı), "
                 "kumanda menteşe hatları (yeşil kesik), teçhizat, yakıt hücreleri", fontsize=10)
    ax.legend(handles=leg, loc="upper right", fontsize=7, ncol=2)
    ax.grid(alpha=0.25)
    fig.tight_layout()
    p = fig_dir / "yk250_layout_top.png"
    fig.savefig(p, dpi=115)
    plt.close(fig)
    out["top"] = p
    out["structure"] = _fig_structure(ctx, objs, fig_dir)
    out["shell"] = _fig_shell(ctx, fig_dir)
    out["sections"] = _fig_sections(ctx, objs, fig_dir)
    return {k: str(v) for k, v in out.items()}


def _arrow(ax, a, b, txt="", col="#c0392b"):
    ax.annotate("", xy=b, xytext=a, arrowprops=dict(arrowstyle="-|>", color=col, lw=1.6, mutation_scale=12), zorder=9)
    if txt:
        ax.text(0.5 * (a[0] + b[0]), 0.5 * (a[1] + b[1]), txt, fontsize=7, color=col, ha="center", va="bottom",
                zorder=10, bbox=dict(fc="w", ec="none", alpha=0.7, pad=0.5))


def _fig_structure(ctx: Ctx, objs: list, fig_dir: Path):
    import matplotlib.pyplot as plt
    L, S = ctx.L, ctx.S
    fig, (a1, a2) = plt.subplots(2, 1, figsize=(16, 12), gridspec_kw={"height_ratios": [1, 1.5]})
    for ax, view in ((a1, "side"), (a2, "top")):
        ix, iy = (0, 2) if view == "side" else (0, 1)
        if view == "side":
            xs, zt, zb = _oml_side(ctx)
            ax.plot(xs, zt, color="#aaaaaa", lw=0.8)
            ax.plot(xs, zb, color="#aaaaaa", lw=0.8)
        else:
            xs, hw = _oml_top(ctx)
            ax.plot(xs, hw, color="#aaaaaa", lw=0.8)
            ax.plot(xs, -hw, color="#aaaaaa", lw=0.8)
            W = S["wing"]["sections"]
            ax.plot([s_["x_le"] for s_ in W], [s_["y"] for s_ in W], color="#aaaaaa", lw=0.8)
            ax.plot([s_["x_le"] + s_["chord"] for s_ in W], [s_["y"] for s_ in W], color="#aaaaaa", lw=0.8)
        for s_ in L["stations"]:
            x = float(s_["x"])
            if view == "side":
                ax.plot([x, x], [ctx.z_bot(x), ctx.z_top(x)], color="#1a5276", lw=2.2 if s_["type"] != "ring" else 1.2)
                ax.text(x, ctx.z_top(x) + 0.01, s_["id"], rotation=90, fontsize=6, ha="center", va="bottom")
            else:
                yy = np.linspace(-0.4, 0.4, 41)
                yy = yy[np.abs(yy) <= float(ctx.af.sec(x)[0][0])]
                ax.plot([station_x(s_, y) for y in yy], yy, color="#1a5276", lw=2.0)
        for o in objs:
            if o.kind == "structure":
                for p in o.prims:
                    if isinstance(p, Capsule):
                        ax.plot([p.p0[ix], p.p1[ix]], [p.p0[iy], p.p1[iy]], color="#2c3e50", lw=2.5)
                    elif isinstance(p, OBB):
                        _poly_obb(ax, p, ix, iy, fc="#d5d8dc" if o.id.startswith("M-") else "#f5b041",
                                  ec="#2c3e50", lw=0.5, alpha=0.8)
                    else:
                        lo, hi = p.bounds()
                        _rect(ax, [lo, hi], ix, iy, fc="#f5b041", ec="#2c3e50", lw=0.5)
            if o.kind == "mount":
                for p in o.prims:
                    ax.plot([p.p0[ix], p.p1[ix]], [p.p0[iy], p.p1[iy]], color="#1e8449", lw=1.5)
        ax.set_aspect("equal")
        ax.grid(alpha=0.2)
    fit = {f["id"]: f for f in L["chassis"]["fittings"]}
    tr = np.asarray(fit["F-TRUNNION"]["pivot"])
    msx = float(next(s_ for s_ in L["stations"] if s_["id"] == "FS-MS")["x"])
    rsx = float(next(s_ for s_ in L["stations"] if s_["id"] == "FS-RS")["x"])
    gx = float(next(s_ for s_ in L["stations"] if s_["id"] == "FS-GEAR")["x"])
    # side arrows
    _arrow(a1, (tr[0], -0.33), (tr[0], tr[2]), "takım yükü → mafsal", "#7d3c98")
    _arrow(a1, (tr[0], tr[2]), (gx, -0.07), "→ takım kirişi → FS-GEAR", "#7d3c98")
    _arrow(a1, (4.15, 0.20), (3.75, 0.19), "itki / motor ataleti → kafes", "#1e8449")
    _arrow(a1, (3.70, 0.19), (3.665, 0.20), "", "#1e8449")
    _arrow(a1, (3.66, 0.26), (3.30, 0.17), "yangın perdesi → sırt kirişleri", "#1e8449")
    fr, fa = np.asarray(fit["F-RISER-FWD"]["point"]), np.asarray(fit["F-RISER-AFT"]["point"])
    _arrow(a1, (fr[0] + 0.25, 0.48), (fr[0], fr[2]), "paraşüt açılma yükü 13,1 kN", "#b9770e")
    _arrow(a1, (fr[0] + 0.25, 0.48), (fa[0], fa[2]), "", "#b9770e")
    _arrow(a1, (3.55, 0.48), (3.52, 0.30), "dikey / stabilatör yükü → kök bağlantıları", "#2874a6")
    _arrow(a1, (0.65, -0.33), (0.65, -0.125), "burun takımı → omurga duvarları", "#7d3c98")
    _arrow(a1, (1.22, -0.30), (1.22, -0.12), "taret ataleti → raylar", "#8e44ad")
    a1.set_xlim(-0.1, 4.6)
    a1.set_ylim(-0.42, 0.56)
    a1.set_title("Yapısal kavram — yan görünüş: birincil yük yolları (çerçeveler kalın mavi, uzun kirişler / omurga koyu, "
                 "bağlantılar turuncu)", fontsize=10)
    # top arrows
    pins = L["chassis"]["wing_joint"]["main_spar"]["pins"]
    p2 = np.asarray(pins[1]["position"])
    _arrow(a2, (p2[0] + 0.25, 1.4), (p2[0], p2[1]), "dış panel momenti → dil + 2 pim (Ø14)", "#c0392b")
    _arrow(a2, (p2[0], p2[1]), (msx + 0.02, 0.05), "çatal → kiriş başlıkları → orta kutu", "#c0392b")
    _arrow(a2, (msx + 0.02, 0.05), (msx - 0.5, 0.36), "kiriş çerçeveleri → kenar uzun kirişleri", "#c0392b")
    _arrow(a2, (tr[0] + 0.25, 0.75), (tr[0], tr[1]), "ana takım", "#7d3c98")
    _arrow(a2, (3.95, -0.6), (3.70, -0.12), "motor kafesi → 4 sert nokta", "#1e8449")
    a2.set_xlim(-0.1, 4.6)
    a2.set_ylim(-0.8, 1.8)
    a2.set_title("Yapısal kavram — üst görünüş: ok açılı orta kutu (kök parça YK250-CH-001), dış panel birleşimi "
                 "y = 0,70 m, takım / motor / kuyruk bağlantıları", fontsize=10)
    fig.tight_layout()
    p = fig_dir / "yk250_layout_structure.png"
    fig.savefig(p, dpi=115)
    plt.close(fig)
    return p


def _fig_shell(ctx: Ctx, fig_dir: Path):
    import matplotlib.pyplot as plt
    from matplotlib.patches import Rectangle
    L = ctx.L
    col = {"fixed": "#d5d8dc", "removable": "#82e0aa", "hinged": "#f8c471", "fairing": "#aed6f1"}
    fig, axs = plt.subplots(2, 1, figsize=(16, 10.5))
    for ax, surf, title in ((axs[0], ("body_upper", "body_full", "cowl_upper", "glove_upper"), "üst yüzey"),
                            (axs[1], ("body_lower", "body_full", "cowl_lower", "glove_lower", "body_side"), "alt yüzey")):
        xs, hw = _oml_top(ctx)
        ax.plot(xs, hw, "k-", lw=0.8)
        ax.plot(xs, -hw, "k-", lw=0.8)
        W = ctx.S["wing"]["sections"]
        ax.plot([s_["x_le"] for s_ in W if s_["y"] < 0.75], [s_["y"] for s_ in W if s_["y"] < 0.75], "k-", lw=0.6)
        ax.plot([s_["x_le"] + s_["chord"] for s_ in W if s_["y"] < 0.75], [s_["y"] for s_ in W if s_["y"] < 0.75],
                "k-", lw=0.6)
        for p in sorted(L["shell"]["panels"], key=lambda q: q["attach"] == "fixed", reverse=True):
            if p["surface"] not in surf:
                continue
            yy = [sorted(p["y"])] + ([sorted([-p["y"][1], -p["y"][0]])] if p.get("mirror") else [])
            for y0, y1 in yy:
                fc = col.get(p["attach"], "#d5d8dc")
                if p.get("layup") == "wing_skin_primary" and p["attach"] == "fixed":
                    fc = "#d2b4de"
                ax.add_patch(Rectangle((p["x"][0], y0), p["x"][1] - p["x"][0], y1 - y0, fc=fc, ec="#34495e", lw=0.7,
                                       alpha=0.85 if p["attach"] != "fixed" else 0.5,
                                       hatch="///" if p.get("rf_window") else None, zorder=3 if p["attach"] != "fixed"
                                       else 2))
                if p["attach"] != "fixed" or (y1 - y0) > 0.5:
                    ax.text(0.5 * (p["x"][0] + p["x"][1]), 0.5 * (y0 + y1), p["id"].replace("P-", ""), fontsize=5.5,
                            ha="center", va="center", rotation=90 if (p["x"][1] - p["x"][0]) < 0.12 else 0, zorder=6)
        ax.set_aspect("equal")
        ax.set_xlim(-0.05, 4.3)
        ax.set_ylim(-0.75, 0.75)
        ax.set_title(f"Kabuk paneli bölümlemesi — {title} (gri: sabit somun plakalı, yeşil: sökülebilir Camloc/vida, "
                     "turuncu: menteşeli, mavi: fileto, mor: yapıştırılmış birincil eldiven, tarama: RF penceresi)",
                     fontsize=9)
        ax.grid(alpha=0.2)
    fig.tight_layout()
    p = fig_dir / "yk250_layout_shell.png"
    fig.savefig(p, dpi=120)
    plt.close(fig)
    return p


def _fig_sections(ctx: Ctx, objs: list, fig_dir: Path):
    import matplotlib.pyplot as plt
    from matplotlib.patches import Rectangle
    L = ctx.L
    ids = ["FS0600", "FS1110", "FS1490", "FS-FUEL", "FS-MS", "FS-GEAR", "FS3480", "FS3670"]
    st = {s_["id"]: s_ for s_ in L["stations"]}
    fig, axs = plt.subplots(2, 4, figsize=(17, 8.5))
    for ax, sid in zip(axs.ravel(), ids):
        s_ = st[sid]
        x = float(s_["x"])
        ph = np.linspace(0, 2 * np.pi, 361)
        yy = np.linspace(-0.45, 0.45, 361)
        zt = np.array([ctx.z_top(x, y) for y in yy])
        zb = np.array([ctx.z_bot(x, y) for y in yy])
        hw = float(ctx.af.sec(x)[0][0])
        m = np.abs(yy) <= hw
        ax.fill_between(yy[m], zb[m], zt[m], color="#d6eaf8" if s_["type"] != "ring" else "#fdebd0", zorder=1)
        ax.plot(yy[m], zt[m], "k-", lw=1)
        ax.plot(yy[m], zb[m], "k-", lw=1)
        if s_["type"] == "ring":
            G = np.array(np.meshgrid(yy, np.linspace(-0.25, 0.4, 200))).reshape(2, -1).T
            P = np.column_stack([np.full(len(G), x), G])
            inner = ctx.af.inside(P, 0.006 + float(s_.get("ring_depth", 0.04)))
            ax.scatter(G[inner, 0], G[inner, 1], s=0.5, c="w", zorder=2)
        for c in s_.get("cutouts", []):
            for sg in ((1.0, -1.0) if c.get("mirror") else (1.0,)):
                y0, y1 = sorted([sg * c["y"][0], sg * c["y"][1]])
                ax.add_patch(Rectangle((y0, c["z"][0]), y1 - y0, c["z"][1] - c["z"][0], fc="w", ec="#c0392b", lw=0.8,
                                       zorder=3))
        for o in objs:
            if o.kind not in ("content", "harness", "fuel", "gear", "structure", "mount", "engine"):
                continue
            for p in o.prims:
                if isinstance(p, Capsule):
                    d = p.p1 - p.p0
                    if abs(d[0]) > 1e-9:
                        t = (x - p.p0[0]) / d[0]
                        if 0 <= t <= 1:
                            q = p.p0 + t * d
                            ax.add_patch(plt.Circle((q[1], q[2]), p.r, fc="#e67e22" if o.kind == "harness" else
                                                    "#5d6d7e", ec="k", lw=0.3, zorder=5))
                elif isinstance(p, OBB) and o.kind in ("content", "structure", "engine"):
                    lo, hi = p.bounds()
                    if lo[0] <= x <= hi[0]:
                        ax.add_patch(Rectangle((lo[1], lo[2]), hi[1] - lo[1], hi[2] - lo[2],
                                               fc=GROUP_COL.get(o.group, "#999999"), alpha=0.35, ec="k", lw=0.3,
                                               zorder=4))
        ax.set_aspect("equal")
        ax.set_xlim(-0.45, 0.45)
        ax.set_ylim(-0.25, 0.40)
        ax.set_title(f"{sid} (x = {x:.3f} m, {s_['type']}{', ' + s_['subtype'] if s_.get('subtype') else ''})",
                     fontsize=8)
        ax.grid(alpha=0.2)
    fig.suptitle("Çerçeve kesitleri: kesikler (kırmızı), kesişen teçhizat / şasi (kutular), kablo demetleri (turuncu), "
                 "halka çerçevelerin açıklığı (beyaz)", fontsize=10)
    fig.tight_layout()
    p = fig_dir / "yk250_layout_sections.png"
    fig.savefig(p, dpi=110)
    plt.close(fig)
    return p


