"""YELKOVAN YK-38 — Blender gövde kurucusu: ``build(scene=None)``.

``ucav.shapes`` ağlarını sözleşmedeki adlar, koleksiyonlar, pivotlar ve yerel eksenlerle sahneye kurar.
Bütün nesneler (doğrudan ya da ara nesneler üzerinden) tasarım ağırlık merkezindeki ``U_Root`` empty'sine
bağlıdır; ebeveyn ters matrisi dünya konumlarını korur (``U_Root`` hareket ettirilince uçak CG etrafında döner).

Nesneler (ebeveyn → koleksiyon; orijin; yerel eksen)
---------------------------------------------------
* ``U_Root`` (UCAV): ``params.U_ROOT_B`` = (−1,232, 0, 0,006), dünya eksenleri.
* Statik deriler (UCAV_Airframe; orijin dünya başlangıcı = burun ucu, FRL; dünya eksenleri): ``U_Fuselage``,
  ``U_WingCenter_L/R`` (kök/glove bloğu, gövde içinden y = 0'a), ``U_WingOuter_L/R``, ``U_Tip_L/R``, ``U_Stab``,
  ``U_Fin_L/R``, ``U_Fairing_Fillet_L/R`` (kanat üstü fileto), ``U_Fairing_Root_L/R`` (düz tabanlı kök kaportası),
  ``U_Fairing_Blister_L/R`` (ER-150 kabartması), ``U_Fairing_StabFillet`` (PA-CF), ``U_Bay_N/L/R`` (turuncu kuyu
  astarları). Kuyu açıklıkları gövde, kök bloğu, kaporta ve kabartmada EXACT boolean ile kesilmiş ve uygulanmıştır.
* Kumanda yüzeyleri (UCAV_Surfaces): ``U_Aileron_*``, ``U_FlapOut_*`` → ``U_WingOuter_*``; ``U_FlapIn_*`` →
  ``U_WingCenter_*``; ``U_Elevator_*`` → ``U_Stab``; ``U_Rudder_*`` → ``U_Fin_*``. Orijin menteşe hattı ortası
  (``HingeLine.mid_b``), yerel eksenler ``HingeLine.frame_b()``: yerel X = ``axis_positive_b`` → + dönüş iki yanda
  da firar kenarı AŞAĞI (dümende sancağa). Eksenler ``delta_rotation_euler``'dedir; ``rotation_euler.x`` sürülür.
* İtki (UCAV_Propulsion): ``U_Cowl``, ``U_ExhaustRing``, ``U_Intake``, ``U_Cowl_Louvers``, ``U_Exhaust_Muffler``,
  ``U_ScuffPad`` (dünya orijinli); ``U_Prop`` (orijin göbek ``PROP.hub_b``, yerel X = ``PROP.axis_aft_b``, pala 1
  yerel +Z) ve çocuğu ``U_Spinner``.
* Taret (UCAV_Payload): ``U_Turret_Mount`` (orijin (−0,215, 0, gövde altı), dünya eksenleri) → ``U_Turret_Pan``
  (orijin top merkezi, yerel Z = pan) → ``U_Turret_Tilt`` (orijin top merkezi, yerel Y = tilt; + dönüş aşağı) →
  ``U_Turret_Window``; ``U_Light_Turret_Ring`` → ``U_Turret_Mount``.
* Takım kapakları (UCAV_Gear): ``U_Door_N_1/2``, ``U_Door_L/R_1`` — orijin menteşe ortası, yerel X = menteşe,
  + dönüş açar (``door_open_deg`` özel özelliği); ``U_Door_L/R_2`` bacak kapağı — toplu (kapalı, deriyle aynı hizada)
  konumda modellenmiştir, orijin ana takım pivotu, yerel X = ``GearLeg.retract_axis_b``. Takım modülü için tarif:
  ``door.rotation_euler.x = −radians(retract_deg)`` (bacak açık pozu) yapıp ``U_GearPivot_<leg>``'e dünya korunarak
  bağlayın; pivot + ``retract_deg`` dönünce kapak tam yuvasına oturur.
* Ayrıntılar (UCAV_Details): ``U_Hatch``, ``U_Stripe_L/R``, ``U_Antenna_*`` → ``U_Fuselage``;
  ``U_Marking_Chevron`` (sol kanat altı yönelim şevronu) → ``U_WingOuter_L``; ``U_Pitot`` →
  ``U_WingCenter_L``; ``U_Light_Landing`` → ``U_WingCenter_R``; ``U_Light_Nav_L/R`` → ``U_Tip_L/R``;
  ``U_Light_Strobe_L/R`` → ``U_Fin_L/R``.

Kullanım::

    import bpy
    from ucav.blender import airframe
    objs = airframe.build()          # {ad: bpy.types.Object}
"""
from __future__ import annotations

import time

import bpy
import numpy as np

from .. import params as P
from .. import shapes as S
from . import util as U

ROOT = "U_Root"
SURFACE_HOST = {"Aileron": "U_WingOuter", "FlapOut": "U_WingOuter", "FlapIn": "U_WingCenter", "Elevator": "U_Stab",
                "Rudder": "U_Fin"}


def _host(name: str, side: str) -> str:
    h = SURFACE_HOST[name]
    return h if h == "U_Stab" else f"{h}_{side}"


def build(scene: bpy.types.Scene | None = None, *, cut_wells: bool = True, verbose: bool = False) -> dict:
    """Bütün gövde nesnelerini kurar ve ``{ad: nesne}`` döndürür. Tekrar çağrılabilir (aynı adlı nesneler
    yeniden oluşturulur). ``cut_wells`` False ise kuyu/cep booleanları atlanır (hızlı önizleme)."""
    scene = scene or bpy.context.scene
    t0 = time.time()
    cols = U.ensure_collections(scene)
    root = U.new_empty(ROOT, cols["UCAV"], P.U_ROOT_B, display="ARROWS", size=0.25)
    root["ucav_model"] = str(P.SPEC["meta"]["name"])
    objs: dict[str, bpy.types.Object] = {ROOT: root}
    parents: dict[str, str] = {}

    def log(msg: str) -> None:
        if verbose:
            print(f"[airframe {time.time() - t0:6.1f} s] {msg}")

    def add(md: S.MeshData, col: str, parent: str = ROOT, origin_b=(0.0, 0.0, 0.0), frame_b=None,
            name: str | None = None, **props) -> bpy.types.Object:
        ob = U.object_from_mesh(md, cols[col], name or md.name, origin_b, frame_b)
        for k, v in props.items():
            ob[k] = v
        objs[ob.name] = ob
        parents[ob.name] = parent
        return ob

    # ------------------------------------------------------------------ gövde ve itki kabukları
    add(S.fuselage(), "UCAV_Airframe")
    add(S.cowl(), "UCAV_Propulsion")
    add(S.exhaust_ring(), "UCAV_Propulsion")
    add(S.intake(), "UCAV_Propulsion", "U_Fuselage")
    add(S.cowl_louvers(), "UCAV_Propulsion", "U_Cowl")
    add(S.muffler_pipe(), "UCAV_Propulsion", "U_Cowl")
    add(S.scuff_pad(), "UCAV_Propulsion", "U_Cowl")
    log("gövde")

    # ------------------------------------------------------------------ kanat, kuyruk, kumanda yüzeyleri
    surfaces: dict[str, S.MeshData] = {}
    for sd in ("L", "R"):
        for name, md in S.wing_parts(sd).items():
            if name.startswith(("U_WingCenter", "U_WingOuter", "U_Tip")):
                add(md, "UCAV_Airframe")
            else:
                surfaces[name] = md
        for name, md in S.fin_parts(sd).items():
            if name.startswith("U_Fin"):
                add(md, "UCAV_Airframe")
            else:
                surfaces[name] = md
        add(S.fin_root_fairing(sd), "UCAV_Airframe", f"U_Fin_{sd}")
        add(S.wing_fillet(sd), "UCAV_Airframe", "U_Fuselage")
        add(S.root_fairing(sd), "UCAV_Airframe", f"U_WingCenter_{sd}")
        add(S.unit_blister(sd), "UCAV_Airframe", f"U_WingCenter_{sd}")
    for name, md in S.stab_parts().items():
        if name == "U_Stab":
            add(md, "UCAV_Airframe")
        else:
            surfaces[name] = md
    add(S.stab_fillet(), "UCAV_Airframe", "U_Stab")
    for h in P.hinge_lines():
        md = surfaces[h.obj_name]
        add(md, "UCAV_Surfaces", _host(h.name, h.side), np.asarray(h.mid_b), h.frame_b(),
            ucav_role="control_surface", hinge_pos_max_deg=h.pos_max_deg, hinge_neg_max_deg=h.neg_max_deg,
            hinge_positive_means=h.positive_means)
    log("kanat/kuyruk")

    # ------------------------------------------------------------------ pervane ve spinner
    pr = P.PROP
    fr_prop = S._frame_from_axes(pr.axis_aft_b, [0.0, 0.0, 1.0])
    add(S.prop(), "UCAV_Propulsion", ROOT, np.asarray(pr.hub_b), fr_prop, ucav_role="prop")
    add(S.spinner(), "UCAV_Propulsion", "U_Prop", np.asarray(pr.hub_b), fr_prop)

    # ------------------------------------------------------------------ taret
    t = P.TURRET
    mount_o = np.asarray(P.to_blender(t.s, 0.0, t.belly_z))
    ball_o = np.asarray(P.to_blender(*t.ball_center))
    I3 = np.eye(3)
    add(S.turret_mount(), "UCAV_Payload", ROOT, mount_o, I3)
    add(S.turret_ring(), "UCAV_Payload", "U_Turret_Mount", mount_o, I3)
    add(S.turret_pan(), "UCAV_Payload", "U_Turret_Mount", ball_o, I3, ucav_role="turret_pan")
    add(S.turret_ball(), "UCAV_Payload", "U_Turret_Pan", ball_o, I3, ucav_role="turret_tilt")
    add(S.turret_windows(), "UCAV_Payload", "U_Turret_Tilt", ball_o, I3)
    log("pervane/taret")

    # ------------------------------------------------------------------ takım: kuyular, kapaklar
    for leg in ("N", "L", "R"):
        add(S.gear_bay(leg), "UCAV_Airframe", ROOT if leg == "N" else f"U_WingCenter_{leg}")
    for d in P.gear_doors():
        g = S.gear_door(d.name)
        o_b = np.asarray(P.to_blender(*g.origin))
        add(g.mesh, "UCAV_Gear", ROOT, o_b, g.frame_b, ucav_role="gear_door",
            door_attach=g.attach, door_open_deg=float(g.open_deg), door_leg=d.leg,
            door_retract_deg=float(P.gear_leg(d.leg).retract_deg))
        hw = S.door_hardware(d.name)
        if hw is not None:                                 # menteşe bilekleri/kulakları: kapakla döner
            add(hw, "UCAV_Gear", d.name, o_b, g.frame_b, ucav_role="door_hardware")
    log("kuyu/kapak")

    # ------------------------------------------------------------------ paketleme zarfları (render/GLB dışı)
    for name, md in S.envelopes().items():
        ob = add(md, "UCAV_Envelopes", ROOT, ucav_role="envelope")
        ob.display_type = "WIRE"
        ob.hide_render = True
        ob.visible_shadow = False

    # ------------------------------------------------------------------ ayrıntılar
    add(S.hatch(), "UCAV_Details", "U_Fuselage")
    add(S.hatch_frame(), "UCAV_Details", "U_Fuselage")
    for sd in ("L", "R"):
        add(S.stripe(sd), "UCAV_Details", "U_Fuselage")
    add(S.chevron(), "UCAV_Details", "U_WingOuter_L")
    for a in P.antennas():
        md = S.blade_antenna(a) if a.kind == "blade" else S.gnss_puck(a) if a.kind == "puck" else S.dipole(a)
        add(md, "UCAV_Details", "U_Fuselage")
        if a.kind == "blade":
            add(S.antenna_doubler(a), "UCAV_Details", "U_Fuselage")
    add(S.pitot_tube(), "UCAV_Details", "U_WingCenter_L")
    add(S.landing_light(), "UCAV_Details", "U_WingCenter_R")
    for f in P.lights():
        if f.kind == "nav":
            add(S.nav_light(f), "UCAV_Details", f"U_Tip_{f.name[-1]}")
        elif f.kind == "strobe":
            add(S.strobe_light(f), "UCAV_Details", f"U_Fin_{f.name[-1]}")
    log("ayrıntılar")

    # ------------------------------------------------------------------ boolean: kuyular, taret yuvası, kapak cebi
    if cut_wells:
        cutters = bpy.data.collections.get("UCAV_Cutters") or bpy.data.collections.new("UCAV_Cutters")
        if cutters.name not in scene.collection.children.keys():
            scene.collection.children.link(cutters)
        jobs = [(md, ["U_Fuselage"]) for md in S.gear_cutters("N")]
        jobs += [(S.turret_cutter(), ["U_Fuselage"]), (S.hatch_cutter(), ["U_Fuselage"]),
                 (S.intake_cutter(), ["U_Fuselage"]), (S.cooling_exit_cutter(), ["U_Cowl"])]
        jobs += [(md, ["U_Cowl"]) for md in S.louver_cutters()]
        for sd in ("L", "R"):
            for md in S.gear_cutters(sd):           # önce dişli deri dudağı, sonra düz duvarlı kuyu hacmi
                jobs.append((md, ["U_Fuselage", f"U_WingCenter_{sd}", f"U_Fairing_Root_{sd}",
                                  f"U_Fairing_Blister_{sd}"]))
        for md, targets in jobs:
            cut = U.object_from_mesh(md, cutters)
            for tn in targets:
                U.boolean_difference_safe(objs[tn], cut)
            U.remove_object(cut.name)
        bpy.data.collections.remove(cutters)
        log("boolean")

    # ------------------------------------------------------------------ ebeveynler (dünya korunur)
    for name, par in parents.items():
        U.parent_keep_world(objs[name], objs[par])
    bpy.context.view_layer.update()

    # ------------------------------------------------------------------ şablon yazılar / servis işaretleri (F8)
    objs.update(build_stencils(cols["UCAV_Details"]))
    log(f"{len(objs)} nesne")
    return objs


def build_stencils(collection: bpy.types.Collection) -> dict:
    """``shapes.stencil_specs()`` işaretlerini kurar (``U_Stencil_*``; ev sahibine bağlı, kapalı ince katı). Ev sahibi
    yoksa ya da yüzeye oturmazsa o işaret atlanır. Dönüş: {ad: nesne}."""
    out: dict = {}
    bvhs: dict = {}
    for st in S.stencil_specs():
        host = bpy.data.objects.get(st.host)
        if host is None:
            continue
        if st.host not in bvhs:
            bvhs[st.host] = U._host_bvh_world(host)
        if st.text is not None:
            V2, F = U.text_mesh_2d(st.text, st.height)
        else:
            V2, F = st.poly
        ob = U.stencil_object(st.name, host, V2, F, P.to_blender(*st.center), st.u_b, st.n_b, st.mat, collection,
                              bvh=bvhs[st.host])
        if ob is None:
            print(f"[airframe] uyarı: {st.name} yüzeye oturmadı ({st.host})")
            continue
        ob["ucav_stencil"] = st.text or "işaret"
        out[ob.name] = ob
    return out


def catalog() -> list[dict]:
    """``build`` sonrası nesne listesi (ad, ebeveyn, koleksiyon, dünya orijini, yerel X/Y/Z) — test ve rapor için."""
    out = []
    for ob in sorted(bpy.data.objects, key=lambda o: o.name):
        if not ob.name.startswith("U_"):
            continue
        mw = ob.matrix_world
        out.append({"name": ob.name, "parent": ob.parent.name if ob.parent else None,
                    "collection": ob.users_collection[0].name if ob.users_collection else None,
                    "origin_b": tuple(round(v, 5) for v in mw.translation),
                    "x_b": tuple(round(v, 4) for v in mw.col[0][:3]), "y_b": tuple(round(v, 4) for v in mw.col[1][:3]),
                    "z_b": tuple(round(v, 4) for v in mw.col[2][:3]), "type": ob.type})
    return out
