"""YELKOVAN YK-38 — animasyonlar (bpy 4.5): ``build(scene=None)``, ``set_scene_range(name)``, ``clear()``.

İki klip kurulur; her biri ``U_Root`` dönüşümüne ve ``U_Root`` kontrol paneli özelliklerine (``rig.PROPS``)
anahtar kare koyar, hareketli parçaları ``rig`` sürücüleri taşır. Aksiyonlar adlandırılmış ve ``fake user``'lıdır;
``set_scene_range(ad)`` aksiyonları atar, kare aralığını, kamera işaretlerini (marker → kamera geçişi) ve sehpa
görünürlüğünü ayarlar.

``showcase`` (19 s, 24 fps, 456 kare)
-------------------------------------
* 0–3,4 s yer: motor çalıştırma (0 → 3000 dev/dk), kumanda kontrolü (kanatçık ±20, irtifa −22/+18, istikamet ±20 —
  burun tekeri birlikte döner), flap 20°, taret bakınır. Kamera A (yer, alçak ön-sol, yavaş yaklaşma).
* 3,4–7,3 s kalkış koşusu: 8600 dev/dk, düz kalkış (spec: koşu 25,5 m, V_LOF 13,2 m/s); teker dönüşü yer hızına
  bağlı, amortisörler kalkışta kendiliğinden uzar. Kamera B (pist kenarı araç kamerası, uçak onu geçer).
* 8,2–15,2 s takım toplama: ``gear`` 1 → 0 doğrusal 7 s (kapak 1 s + bacak 5 s + kapak 1 s, ER-150 8,4 V'ta 5 s).
  Kamera C (takip kamerası düzeneği: ``U_CamRig_Follow`` uçağın konum + yönünü izler, kamera alttan-arkadan).
* 10,6 s'den sonra tırmanarak sola yatışlı dönüş (25°), 15,4–16,6 s flap 0; taret yer kamerası D'yi izler
  (pan/tilt her kare hesaplanır). Kamera D (yerde sehpa; kameraman zum'u: kanat açıklığı kadrajın ~%45'i).

``mechanisms`` (10 s, 240 kare, kesintisiz döngü)
-------------------------------------------------
Uçak bakım sehpasında (``U_Stand_Mechanisms``, iki kriko, tekerler yerden ~6 cm yukarıda; amortisörler serbest).
Takım 0,6–4,2 s toplanır, 5,2–8,8 s açılır (gerçek sürenin yarısı); kanatçık, irtifa, flap (0 → 30 → 0), istikamet
(+ burun yönlendirmesi) sırayla süpürülür; taret 240° döner ve eğilir; pervane 2400 dev/dk (döngüde tam 400 tur).
Kamera ``U_Cam_Mechanisms`` ``U_CamRig_Orbit`` üzerinde yavaş sarkaç yörüngesinde (önden-sola, ±42°).

Not: pervane açısı ``rig`` sürücüsünde kare/fps·rpm/60·2π'dir; devir değişirken açının ∫rpm·dt olması için
``U_Prop["ucav_turns_offset"]`` her kare yazılır (aksiyon ``YK38_<klip>_Prop``).
"""
from __future__ import annotations

import math

import bpy
import numpy as np
from mathutils import Matrix, Vector

from .. import params as P
from . import rig as RIG
from . import util as U

FPS = 24
G0 = 9.80665
ROOT = "U_Root"
STUDIO = "UCAV_Studio"
CLIPS = {"showcase": 19.0, "mechanisms": 10.0}           # süre (s)
MARK = "YK38_"                                           # zaman çizelgesi işaret öneki

_PERF = P.SPEC.get("performance", {})


def _takeoff_numbers() -> tuple[float, float]:
    """(V_LOF m/s, kalkış koşusu m) — spec ``performance``: ``takeoff`` metnindeki V_LOF (yoksa 1,22·Vs flap 20),
    ``takeoff_run_m.asphalt``."""
    import re
    m = re.search(r"V_LOF\s*([0-9]+[.,]?[0-9]*)", str(_PERF.get("takeoff", "")))
    vs = _PERF.get("stall_speed_ms", {})
    v = float(m.group(1).replace(",", ".")) if m else 1.22 * float(vs.get("flaps_20_mtow", 10.8))
    run = _PERF.get("takeoff_run_m", 25.5)
    run = float(run.get("asphalt", 25.5)) if isinstance(run, dict) else float(run)
    return v, run


V_LOF, TAKEOFF_RUN = _takeoff_numbers()                  # 13,2 m/s, 25,5 m (düz kalkış, 20° flap)
STAND_LIFT = 0.07                                        # sehpada U_Root yükseltmesi (m). varsayım


def nframes(name: str) -> int:
    return int(round(CLIPS[name] * FPS))


def _t(n: int) -> np.ndarray:
    """Kare 1…n için zaman (s): kare 1 = 0 s."""
    return (np.arange(1, n + 1) - 1) / FPS


def _ease(t, keys):
    """Anahtarlar arası yumuşak (smoothstep) geçiş: ``keys`` = [(t, v), …]."""
    t = np.asarray(t, float)
    out = np.full_like(t, keys[0][1], dtype=float)
    for (t0, v0), (t1, v1) in zip(keys[:-1], keys[1:]):
        m = (t >= t0) & (t <= t1)
        u = np.clip((t[m] - t0) / max(t1 - t0, 1e-9), 0, 1)
        out[m] = v0 + (v1 - v0) * u * u * (3 - 2 * u)
    out[t > keys[-1][0]] = keys[-1][1]
    return out


def _lag(x: np.ndarray, tau: float, dt: float = 1.0 / FPS) -> np.ndarray:
    """Birinci derece gecikme süzgeci (kamera yumuşatma)."""
    y = np.array(x, float, copy=True)
    k = dt / (tau + dt)
    for i in range(1, len(y)):
        y[i] = y[i - 1] + k * (x[i] - y[i - 1])
    return y


# =====================================================================================================
# Planlar (saf numpy; bpy gerektirmez)
# =====================================================================================================
CONTROL_KEYS_SHOWCASE: dict[str, list[tuple[float, float]]] = {
    "gear": [(0.0, 1.0), (8.2, 1.0), (15.2, 0.0)],
    "gear_doors": [(0.0, 0.0)],
    "prop_rpm": [(0.0, 0.0), (0.25, 0.0), (1.0, 3600.0), (1.6, 3000.0), (3.4, 3000.0), (4.0, 8600.0), (11.0, 8600.0),
                 (12.4, 7400.0), (19.0, 7400.0)],
    "aileron_deg": [(0.0, 0.0), (0.5, 0.0), (0.95, 20.0), (1.45, -20.0), (1.9, 0.0), (5.0, 0.0), (5.6, 2.0), (6.3, -1.5),
                    (7.0, 0.0), (10.5, 0.0), (10.9, -12.0), (11.6, -1.0), (16.4, -1.0), (16.9, 6.0), (17.6, 0.0),
                    (19.0, 0.0)],
    "elevator_deg": [(0.0, 0.0), (1.2, 0.0), (1.65, -22.0), (2.1, 18.0), (2.5, 0.0), (6.4, 0.0), (7.1, -6.0), (8.0, -3.0),
                     (10.6, -3.0), (11.6, -6.0), (16.4, -6.0), (17.4, -2.0), (19.0, -2.0)],
    "rudder_deg": [(0.0, 0.0), (1.9, 0.0), (2.3, 20.0), (2.75, -20.0), (3.15, 0.0), (4.8, 0.0), (5.3, -3.0), (6.0, 2.5),
                   (6.7, 0.0), (10.5, 0.0), (10.9, -5.0), (11.8, -1.5), (16.4, -1.5), (17.0, 2.5), (17.6, 0.0),
                   (19.0, 0.0)],
    "flap_deg": [(0.0, 0.0), (2.5, 0.0), (3.2, 20.0), (15.4, 20.0), (16.6, 0.0)],
    "nav_lights": [(0.0, 1.0)],
    "strobe": [(0.0, 1.0)],
}
TURRET_SCRIPT_SHOWCASE = {"pan": [(0.0, 0.0), (0.7, -40.0), (1.9, 35.0), (2.9, 0.0), (9.0, 0.0)],
                          "tilt": [(0.0, 0.0), (0.7, -15.0), (1.9, -5.0), (2.9, -10.0), (9.0, -10.0)]}
TRACK_BLEND = (9.0, 10.2)                                # betik → hedef izleme geçişi (s)


def _euler_xyz(yaw, pitch_up, roll):
    """Uçak açıları → Blender XYZ euler (X: yatış + = sağ kanat aşağı, Y: −yunuslama, Z: baş + = sola)."""
    return np.column_stack([roll, -pitch_up, yaw])


def _rot_matrices(eul: np.ndarray) -> np.ndarray:
    out = np.empty((len(eul), 3, 3))
    for i, (a, b, c) in enumerate(eul):
        ca, sa, cb, sb, cc, sc = math.cos(a), math.sin(a), math.cos(b), math.sin(b), math.cos(c), math.sin(c)
        Rx = np.array([[1, 0, 0], [0, ca, -sa], [0, sa, ca]])
        Ry = np.array([[cb, 0, sb], [0, 1, 0], [-sb, 0, cb]])
        Rz = np.array([[cc, -sc, 0], [sc, cc, 0], [0, 0, 1]])
        out[i] = Rz @ Ry @ Rx
    return out


def _takeoff_accel() -> float:
    """Kalkış koşusu ivmesi (m/s²): itki 0,6 s'de oturur, ½ρv² sürükleme payı; V_LOF'a koşu = ``TAKEOFF_RUN``."""
    def run(a_max):
        v = x = t = 0.0
        dt = 1e-3
        while v < V_LOF and t < 30:
            a = a_max * min(1.0, t / 0.6) ** 2 * (3 - 2 * min(1.0, t / 0.6)) - 0.0035 * v * v
            v += a * dt
            x += v * dt
            t += dt
        return x, t
    lo, hi = 1.0, 12.0
    for _ in range(50):
        mid = 0.5 * (lo + hi)
        if run(mid)[0] > TAKEOFF_RUN:
            lo = mid
        else:
            hi = mid
    return 0.5 * (lo + hi)


def showcase_plan() -> dict:
    """Gösterim klibi: uçuş yolu (U_Root konum/euler), taret izleme açıları, kamera konumları, kontrol anahtarları."""
    n = nframes("showcase")
    t = _t(n)
    dt = 1.0 / FPS
    t_roll = 3.4
    a_max = _takeoff_accel()
    v = np.zeros(n)
    x = np.zeros(n)
    t_lof = None
    # hız profili (yer koşusu + tırmanış ivmesi)
    for i in range(1, n):
        tt = t[i - 1]
        if tt < t_roll:
            a = 0.0
        elif t_lof is None:
            u = min(1.0, (tt - t_roll) / 0.6)
            a = a_max * u * u * (3 - 2 * u) - 0.0035 * v[i - 1] ** 2
        else:
            a = 0.85 if v[i - 1] < 16.0 else (0.35 if v[i - 1] < 18.5 else 0.0)
        v[i] = v[i - 1] + a * dt
        if t_lof is None and v[i] >= V_LOF:
            t_lof = t[i]
    t_lof = t_lof or 7.3
    gamma = np.radians(_ease(t, [(t_lof, 0.0), (t_lof + 1.8, 7.0), (12.0, 7.0), (14.5, 2.5), (16.5, 0.5)]))
    alpha = np.radians(_ease(t, [(t_lof - 0.15, 0.0), (t_lof + 0.6, 2.2), (19.0, 2.2)]))
    bank = np.radians(_ease(t, [(10.6, 0.0), (11.9, -26.0), (16.4, -26.0), (17.6, -12.0)]))     # − = sol kanat aşağı
    yaw = np.zeros(n)
    pos = np.zeros((n, 3))
    for i in range(1, n):
        yaw[i] = yaw[i - 1] + G0 * math.tan(-bank[i - 1]) / max(v[i - 1], 1.0) * dt
        cg = math.cos(gamma[i - 1])
        pos[i] = pos[i - 1] + v[i - 1] * dt * np.array([cg * math.cos(yaw[i - 1]), cg * math.sin(yaw[i - 1]),
                                                       math.sin(gamma[i - 1])])
    pitch = gamma + alpha * (1.0 / np.cos(bank))
    root0 = np.asarray(P.U_ROOT_B, float)
    loc = root0 + pos
    eul = _euler_xyz(yaw, pitch, bank)
    Rm = _rot_matrices(eul)
    # ---------------------------------------------------------------- yer kamerası D ve taret izleme
    i_pass = int(round(16.9 * FPS))
    heading = np.array([math.cos(yaw[i_pass]), math.sin(yaw[i_pass]), 0.0])
    left = np.array([-heading[1], heading[0], 0.0])
    cam_d = loc[i_pass] * np.array([1, 1, 0]) + left * 13.0 + heading * 5.0
    cam_d[2] = P.GROUND_Z + 1.6
    ball = np.asarray(P.to_blender(*P.TURRET.ball_center), float) - root0
    pan_t = np.zeros(n)
    tilt_t = np.zeros(n)
    for i in range(n):
        d = Rm[i].T @ (cam_d - (loc[i] + Rm[i] @ ball))
        pan_t[i] = math.degrees(math.atan2(d[1], d[0]))
        tilt_t[i] = math.degrees(math.atan2(d[2], math.hypot(d[0], d[1])))
    pan_t = np.degrees(np.unwrap(np.radians(pan_t)))
    pan_s = _ease(t, TURRET_SCRIPT_SHOWCASE["pan"])
    tilt_s = _ease(t, TURRET_SCRIPT_SHOWCASE["tilt"])
    w = _ease(t, [(TRACK_BLEND[0], 0.0), (TRACK_BLEND[1], 1.0)])
    pan = np.clip(pan_s + w * (pan_t - pan_s), -180.0, 180.0)
    tilt = np.clip(tilt_s + w * (tilt_t - tilt_s), -90.0, 20.0)
    # D: kameraman zum'u — kanat açıklığı kadrajın ~%45'i (36 mm sensör)
    dist = np.linalg.norm(loc - cam_d, axis=1)
    span = 2 * P.WING_SEMI_SPAN
    lens_d = np.clip(_lag(36.0 * 0.45 * dist / span, 0.5), 28.0, 220.0)
    # ---------------------------------------------------------------- kameralar A (yer) ve B (pist kenarı araç)
    cam_a = np.column_stack([_ease(t, [(0.0, 2.45), (3.6, 1.85)]), _ease(t, [(0.0, 3.15), (3.6, 2.65)]),
                             _ease(t, [(0.0, -0.05), (3.6, -0.11)])])
    cam_b = np.column_stack([_lag(loc[:, 0] + 1.4, 0.55), np.full(n, 4.3), np.full(n, P.GROUND_Z + 0.50)])
    i_roll = int(round(t_roll * FPS))
    cam_b[:i_roll, 0] = cam_b[i_roll, 0]
    cam_b[:, 2] += _ease(t, [(7.0, 0.0), (8.6, 1.4)])                   # kalkışta hafif yükselir
    return {"n": n, "t": t, "v": v, "loc": loc, "euler": eul, "t_lof": t_lof, "x_lof": float(loc[int(t_lof * FPS), 0] - root0[0]),
            "a_max": a_max, "pan": pan, "tilt": tilt, "cam_a": cam_a, "cam_b": cam_b, "cam_d": cam_d, "lens_d": lens_d,
            "follow_offset": [(8.0, (-6.8, -3.2, -0.30)), (9.8, (-6.3, -3.4, -1.0)), (13.6, (-4.6, -3.9, -0.55))],
            "markers": [(1, "U_Cam_Showcase_A"), (int(3.4 * FPS) + 1, "U_Cam_Showcase_B"),
                        (int(8.6 * FPS) + 1, "U_Cam_Showcase_C"), (int(13.6 * FPS) + 1, "U_Cam_Showcase_D")],
            "controls": CONTROL_KEYS_SHOWCASE, "linear": {"gear"}}


CONTROL_KEYS_MECH: dict[str, list[tuple[float, float]]] = {
    "gear": [(0.0, 1.0), (0.6, 1.0), (4.2, 0.0), (5.2, 0.0), (8.8, 1.0), (10.0, 1.0)],
    "gear_doors": [(0.0, 0.0)],
    "prop_rpm": [(0.0, 2400.0), (10.0, 2400.0)],
    "aileron_deg": [(0.0, 0.0), (0.3, 0.0), (0.925, 20.0), (2.175, -20.0), (2.8, 0.0), (10.0, 0.0)],
    "elevator_deg": [(0.0, 0.0), (2.8, 0.0), (3.425, -25.0), (4.675, 20.0), (5.3, 0.0), (10.0, 0.0)],
    "flap_deg": [(0.0, 0.0), (5.3, 0.0), (6.4, 30.0), (6.8, 30.0), (7.6, 0.0), (10.0, 0.0)],
    "rudder_deg": [(0.0, 0.0), (7.6, 0.0), (8.2, 22.0), (9.4, -22.0), (10.0, 0.0)],
    "turret_pan_deg": [(0.0, 0.0), (2.5, 120.0), (7.5, -120.0), (10.0, 0.0)],
    "turret_tilt_deg": [(0.0, 0.0), (2.5, -45.0), (5.0, 15.0), (7.5, -80.0), (10.0, 0.0)],
    "nav_lights": [(0.0, 1.0)],
    "strobe": [(0.0, 1.0)],
}


def mechanisms_plan() -> dict:
    n = nframes("mechanisms")
    t = _t(n)
    root0 = np.asarray(P.U_ROOT_B, float)
    loc = np.tile(root0 + np.array([0.0, 0.0, STAND_LIFT]), (n, 1))
    eul = np.zeros((n, 3))
    ph = 2 * math.pi * t / CLIPS["mechanisms"]
    orbit_z = np.radians(32.0 + 42.0 * np.sin(ph))                      # sarkaç yörünge önden-sola (döngüde sürekli)
    orbit_h = 0.08 + 0.26 * np.cos(ph + 0.6)
    return {"n": n, "t": t, "loc": loc, "euler": eul, "orbit_z": orbit_z, "orbit_h": orbit_h,
            "center": root0 + np.array([0.15, 0.0, STAND_LIFT - 0.07]), "radius": 3.9,
            "markers": [(1, "U_Cam_Mechanisms")], "controls": CONTROL_KEYS_MECH, "linear": set()}


# =====================================================================================================
# Aksiyon yazımı
# =====================================================================================================
def _action(name: str) -> bpy.types.Action:
    old = bpy.data.actions.get(name)
    if old is not None:
        bpy.data.actions.remove(old)
    act = bpy.data.actions.new(name)
    act.use_fake_user = True
    return act


def _dense(act, path: str, index: int, values, start_frame: int = 1) -> bpy.types.FCurve:
    vals = np.asarray(values, float)
    fc = act.fcurves.new(path, index=max(index, 0))
    fr = np.arange(start_frame, start_frame + len(vals), dtype=float)
    fc.keyframe_points.add(len(vals))
    fc.keyframe_points.foreach_set("co", np.column_stack([fr, vals]).ravel())
    fc.keyframe_points.foreach_set("interpolation", np.full(len(vals), 1, dtype=np.int32))     # LINEAR
    fc.update()
    return fc


def _sparse(act, path: str, index: int, keys, linear: bool = False) -> bpy.types.FCurve:
    """Seyrek anahtarlar: düzenlenebilir; BEZIER + AUTO_CLAMPED (ya da LINEAR)."""
    fc = act.fcurves.new(path, index=index)
    fc.keyframe_points.add(len(keys))
    for kp, (tt, vv) in zip(fc.keyframe_points, keys):
        kp.co = (tt * FPS + 1.0, float(vv))
        kp.interpolation = "LINEAR" if linear else "BEZIER"
        kp.handle_left_type = kp.handle_right_type = "AUTO_CLAMPED"
    fc.update()
    return fc


def _assign(ob: bpy.types.Object, act: bpy.types.Action | None) -> None:
    if ob.animation_data is None:
        ob.animation_data_create()
    ob.animation_data.action = act


def _root_action(name: str, plan: dict) -> bpy.types.Action:
    act = _action(f"YK38_{name}_Root")
    for i in range(3):
        _dense(act, "location", i, plan["loc"][:, i])
        _dense(act, "rotation_euler", i, np.unwrap(plan["euler"][:, i]) if i == 2 else plan["euler"][:, i])
    for prop, keys in plan["controls"].items():
        _sparse(act, f'["{prop}"]', 0, keys, linear=prop in plan["linear"])
    if "pan" in plan:
        _dense(act, '["turret_pan_deg"]', 0, plan["pan"])
        _dense(act, '["turret_tilt_deg"]', 0, plan["tilt"])
    return act


def _prop_action(name: str, root_act: bpy.types.Action, n: int) -> bpy.types.Action:
    """``U_Prop["ucav_turns_offset"]``: açı(kare) = ∫rpm/60 dt olsun diye ofset = tur − kare/fps·rpm/60."""
    fc = next(f for f in root_act.fcurves if f.data_path == '["prop_rpm"]')
    frames = np.arange(1, n + 1, dtype=float)
    rpm = np.array([fc.evaluate(f) for f in frames])
    turns = np.concatenate([[0.0], np.cumsum(0.5 * (rpm[1:] + rpm[:-1]) / 60.0 / FPS)])
    offset = turns - frames / FPS * rpm / 60.0
    act = _action(f"YK38_{name}_Prop")
    _dense(act, '["ucav_turns_offset"]', 0, offset)
    return act


# =====================================================================================================
# Kameralar, düzenekler, sehpa
# =====================================================================================================
def _camera(name: str, col, lens: float, clip_start: float = 0.02) -> bpy.types.Object:
    U.remove_object(name)
    cam = bpy.data.cameras.get(name) or bpy.data.cameras.new(name)
    cam.lens = lens
    cam.clip_start = clip_start
    cam.clip_end = 2000.0
    cam.sensor_width = 36.0
    ob = bpy.data.objects.new(name, cam)
    col.objects.link(ob)
    return ob


def _empty(name: str, col, parent=None, loc=(0.0, 0.0, 0.0), size: float = 0.15, kind: str = "PLAIN_AXES"):
    U.remove_object(name)
    ob = bpy.data.objects.new(name, None)
    col.objects.link(ob)
    ob.empty_display_type = kind
    ob.empty_display_size = size
    if parent is not None:
        ob.parent = parent
        ob.matrix_parent_inverse = Matrix.Identity(4)
    ob.location = Vector(tuple(map(float, loc)))
    ob.hide_render = True
    return ob


def _track_to(ob, target) -> None:
    for c in list(ob.constraints):
        ob.constraints.remove(c)
    c = ob.constraints.new("TRACK_TO")
    c.target = target
    c.track_axis = "TRACK_NEGATIVE_Z"
    c.up_axis = "UP_Y"


def _stand_mesh(lift: float):
    """İki bakım krikosu (gövde altında s ≈ 0,80 ve 1,62): taban, kolon, vida, gövdeye oturan TPU yastık."""
    from .. import shapes as S
    from . import gear as GR
    parts = []
    for s in (0.80, 1.62):
        x = -s
        z_top = P.fuselage_section(s).z_bottom + lift
        zg = P.GROUND_Z
        parts.append(GR.cyl("base", (x, 0, zg), (x, 0, zg + 0.012), 0.11, "UM_Accent", 40, chamfer=0.004))
        for k in range(3):
            a = 2 * math.pi * k / 3 + 0.5
            c = np.array([x + 0.07 * math.cos(a), 0.07 * math.sin(a), zg + 0.020])
            R = np.column_stack([[math.cos(a), math.sin(a), 0], [-math.sin(a), math.cos(a), 0], [0, 0, 1]])
            parts.append(GR._xf(GR.box("rib", np.zeros(3), (0.07, 0.008, 0.010), None, "UM_Accent", r=0.004), R, c))
        parts.append(GR.cyl("col", (x, 0, zg + 0.010), (x, 0, z_top - 0.075), 0.019, "UM_Accent", 32, chamfer=0.002))
        parts.append(GR.cyl("band", (x, 0, z_top - 0.150), (x, 0, z_top - 0.120), 0.0195, "UM_Orange", 32))
        parts.append(GR.cyl("screw", (x, 0, z_top - 0.080), (x, 0, z_top - 0.020), 0.011, "UM_Steel", 24))
        parts.append(GR.cyl("cup", (x, 0, z_top - 0.024), (x, 0, z_top - 0.012), 0.032, "UM_Accent", 32, chamfer=0.002))
        # yastık: üst yüzü gövde altına uyar (y ±0,028, s ±0,035)
        ys = np.linspace(-0.028, 0.028, 15)
        ss = np.linspace(s - 0.035, s + 0.035, 9)
        mb = S.MeshBuilder("pad", "local")
        rings = []
        for sv in ss:
            zt = np.array([(P.fuselage_z_at(sv, yv, "bottom") or P.fuselage_section(sv).z_bottom) + lift - 0.0003
                           for yv in ys])
            ring = np.vstack([np.column_stack([np.full(len(ys), -sv), ys, np.full(len(ys), z_top - 0.013)]),
                              np.column_stack([np.full(len(ys), -sv), ys[::-1], zt[::-1]])])
            rings.append(mb.add(ring))
        mb.loft(rings, "UM_TPU")
        mb.cap(rings[0], "UM_TPU", start=True)
        mb.cap(rings[-1], "UM_TPU", start=False)
        parts.append(mb.build(40.0))
    return GR.merge("U_Stand_Mechanisms", parts, 35.0)


def _build_stand(col) -> bpy.types.Object:
    md = _stand_mesh(STAND_LIFT)
    U.remove_object(md.name)
    me = U.mesh_from_data(md, md.name)
    ob = bpy.data.objects.new(md.name, me)
    col.objects.link(ob)
    return ob


# =====================================================================================================
# Kurulum ve klip seçimi
# =====================================================================================================
def build(scene: bpy.types.Scene | None = None, *, select: str = "showcase") -> dict:
    """İki klibi (aksiyonlar, kameralar, düzenekler, sehpa) kurar ve ``select`` klibini etkinleştirir.
    Önkoşul: ``airframe.build()``, ``gear.build()``, ``rig.setup()``. Tekrar çağrılabilir."""
    scene = scene or bpy.context.scene
    cols = U.ensure_collections(scene)
    col = cols[STUDIO]
    root = bpy.data.objects[ROOT]
    prop = bpy.data.objects.get("U_Prop")
    scene.render.fps = FPS
    scene.render.fps_base = 1.0
    out: dict = {"actions": [], "cameras": []}
    # ---------------------------------------------------------------- gösterim
    sp = showcase_plan()
    ra = _root_action("showcase", sp)
    out["actions"].append(ra.name)
    if prop is not None:
        out["actions"].append(_prop_action("showcase", ra, sp["n"]).name)
    tgt = _empty("U_CamTarget_Showcase", col, root, (0.25, 0.0, -0.06), 0.1, "SPHERE")
    # A: yer kamerası, yavaş yaklaşma
    ca = _camera("U_Cam_Showcase_A", col, 36.0)
    act = _action("YK38_showcase_CamA")
    for i in range(3):
        _dense(act, "location", i, sp["cam_a"][:, i])
    _assign(ca, act)
    _track_to(ca, tgt)
    # B: pist kenarı araç kamerası
    cb = _camera("U_Cam_Showcase_B", col, 30.0)
    act = _action("YK38_showcase_CamB")
    for i in range(3):
        _dense(act, "location", i, sp["cam_b"][:, i])
    _assign(cb, act)
    _track_to(cb, tgt)
    # C: takip kamerası düzeneği (konum + yalnız baş açısı)
    rig_f = _empty("U_CamRig_Follow", col, None, P.U_ROOT_B, 0.3, "CUBE")
    for c in list(rig_f.constraints):
        rig_f.constraints.remove(c)
    cl = rig_f.constraints.new("COPY_LOCATION")
    cl.target = root
    cr = rig_f.constraints.new("COPY_ROTATION")
    cr.target = root
    cr.use_x = cr.use_y = False
    cr.use_z = True
    cc = _camera("U_Cam_Showcase_C", col, 40.0)
    cc.parent = rig_f
    cc.matrix_parent_inverse = Matrix.Identity(4)
    act = _action("YK38_showcase_CamC")
    for i in range(3):
        _sparse(act, "location", i, [(tt, off[i]) for tt, off in sp["follow_offset"]])
    _assign(cc, act)
    _track_to(cc, tgt)
    # D: yerde sehpa, uzun odak
    cd = _camera("U_Cam_Showcase_D", col, 55.0, 0.1)
    cd.location = Vector(tuple(sp["cam_d"]))
    _assign(cd, None)
    act = _action("YK38_showcase_CamD_Lens")
    _dense(act, "lens", -1, sp["lens_d"])
    if cd.data.animation_data is None:
        cd.data.animation_data_create()
    cd.data.animation_data.action = act
    _track_to(cd, tgt)
    out["cameras"] += [ca.name, cb.name, cc.name, cd.name]
    # ---------------------------------------------------------------- mekanizmalar
    mp = mechanisms_plan()
    ma = _root_action("mechanisms", mp)
    out["actions"].append(ma.name)
    if prop is not None:
        out["actions"].append(_prop_action("mechanisms", ma, mp["n"]).name)
    orbit = _empty("U_CamRig_Orbit", col, None, mp["center"], 0.3, "CIRCLE")
    act = _action("YK38_mechanisms_Orbit")
    _dense(act, "rotation_euler", 2, mp["orbit_z"])
    _assign(orbit, act)
    tgt_m = _empty("U_CamTarget_Mechanisms", col, None, mp["center"] + np.array([0.0, 0.0, -0.05]), 0.1, "SPHERE")
    cm = _camera("U_Cam_Mechanisms", col, 35.0)
    cm.parent = orbit
    cm.matrix_parent_inverse = Matrix.Identity(4)
    act = _action("YK38_mechanisms_Cam")
    _dense(act, "location", 0, np.full(mp["n"], mp["radius"]))
    _dense(act, "location", 2, mp["orbit_h"])
    _assign(cm, act)
    _track_to(cm, tgt_m)
    out["cameras"].append(cm.name)
    stand = _build_stand(col)
    out["stand"] = stand.name
    out["showcase"] = {"t_lof_s": round(sp["t_lof"], 3), "x_lof_m": round(sp["x_lof"], 2),
                       "a_takeoff_ms2": round(sp["a_max"], 3), "frames": sp["n"],
                       "extent_m": [np.round(sp["loc"].min(0), 1).tolist(), np.round(sp["loc"].max(0), 1).tolist()]}
    out["mechanisms"] = {"frames": mp["n"]}
    set_scene_range(select, scene)
    return out


def set_scene_range(name: str, scene: bpy.types.Scene | None = None) -> dict:
    """Klibi etkinleştirir: ``U_Root``/``U_Prop`` aksiyonları, kare aralığı (1…N), kamera işaretleri, etkin kamera,
    sehpa görünürlüğü. ``name`` ∈ {"showcase", "mechanisms"}."""
    if name not in CLIPS:
        raise KeyError(f"bilinmeyen klip: {name} ({', '.join(CLIPS)})")
    scene = scene or bpy.context.scene
    root = bpy.data.objects[ROOT]
    _assign(root, bpy.data.actions.get(f"YK38_{name}_Root"))
    prop = bpy.data.objects.get("U_Prop")
    if prop is not None:
        _assign(prop, bpy.data.actions.get(f"YK38_{name}_Prop"))
    n = nframes(name)
    scene.render.fps = FPS
    scene.render.fps_base = 1.0
    scene.frame_start, scene.frame_end = 1, n
    for m in list(scene.timeline_markers):
        if m.name.startswith(MARK):
            scene.timeline_markers.remove(m)
    plan_markers = showcase_plan()["markers"] if name == "showcase" else mechanisms_plan()["markers"]
    first = None
    for f, cam_name in plan_markers:
        cam = bpy.data.objects.get(cam_name)
        mk = scene.timeline_markers.new(f"{MARK}{cam_name[6:]}", frame=f)
        mk.camera = cam
        first = first or cam
    scene.camera = first
    for ob in bpy.data.objects:
        if ob.name.startswith("U_Stand_"):
            ob.hide_render = ob.hide_viewport = name != "mechanisms"
    scene.frame_set(1)
    RIG.refresh()
    return {"clip": name, "frames": n, "camera": first.name if first else None}


def clear(scene: bpy.types.Scene | None = None) -> None:
    """Animasyonu kaldırır: ``U_Root`` dinlenme konumuna, özellikler varsayılana, işaretler silinir."""
    scene = scene or bpy.context.scene
    root = bpy.data.objects[ROOT]
    _assign(root, None)
    prop = bpy.data.objects.get("U_Prop")
    if prop is not None:
        _assign(prop, None)
        prop["ucav_turns_offset"] = 0.0
    root.location = Vector(P.U_ROOT_B)
    root.rotation_euler = (0.0, 0.0, 0.0)
    RIG.ensure_props(root, reset=True)
    for m in list(scene.timeline_markers):
        if m.name.startswith(MARK):
            scene.timeline_markers.remove(m)
    for ob in bpy.data.objects:
        if ob.name.startswith("U_Stand_"):
            ob.hide_render = ob.hide_viewport = True
    RIG.refresh()
