"""YELKOVAN YK-38 — animasyonlar (bpy 4.5): ``build(scene=None)``, ``set_scene_range(name)``, ``clear()``.

İki klip kurulur; her biri ``U_Root`` dönüşümüne ve ``U_Root`` kontrol paneli özelliklerine (``rig.PROPS``)
anahtar kare koyar, hareketli parçaları ``rig`` sürücüleri taşır. Aksiyonlar adlandırılmış ve ``fake user``'lıdır;
``set_scene_range(ad)`` aksiyonları atar, kare aralığını, kamera işaretlerini (marker → kamera geçişi), olay
işaretlerini (kamerasız ``YK38_ev_*``; Blender kamera geçişinde yok sayar) ve sehpa görünürlüğünü ayarlar.

Anahtarlar düzenlenebilir: uçuş yolu, kameralar, lens ve taret açıları her kare yerine seyrek BEZIER (AUTO_CLAMPED)
anahtarlarla yazılır (RDP + doğrulama: konumda ≤ 5 mm, açıda ≤ 0,1°, taret ≤ 0,2°); planın kare kare hâli
``YK38_showcase_Root_pisirilmis`` (fake user) aksiyonunda durur. Pervane turu ``U_Prop["ucav_turns"]``'e ∫rpm·dt
olarak pişirilir (``U_Root["prop_auto"]`` = 0) — devir değişirken pervane geri dönmez. Tekerler yer koşusunda
``U_Root`` X'ine bağlıdır (``wheel_auto`` = 1, kaymasız); teker kesmede ``wheel_roll_m``'e geçilir, τ = 0,6 s ile
yavaşlar ve takım toplanmaya başlayınca fren (dönüş 0).

Her animasyon kamerasının kendi hedef boşluğu vardır (``U_CamTarget_<kamera>``), alan derinliği açıktır (odak =
hedef). Kameralar ``UCAV_Studio/UCAV_Cameras_Anim``, sehpa ``UCAV_Studio/UCAV_Stand`` koleksiyonundadır.

``showcase`` (21 s, 24 fps, 504 kare, pist)
------------------------------------------
* A (0–2,0 s): kuruluş planı, ön-sol 3/4, 40 mm yaklaşma + alçalma (uçak kadrajın %63'ünden %80'ine); motor
  çalışır (0 → 3600 → 3000 dev/dk), taret bakınır.
* A1 (2,0–2,95 s): sol kanatçık yakın planı (85 mm, f/11): kanatçık +20/−20 (firar kenarı ~25/42 piksel @ 720p).
* A1F (2,95–3,95 s): flap yakın planı (70 mm, arka-üst 3/4): iç + dış flap 0 → 20° (kalkış flabı). Kesmeyle.
* A2 (3,95–5,4 s): kuyruk yakın planı (80 mm): irtifa −25/+20, istikamet ±22 (burun tekeri birlikte döner).
* B (5,4–10,6 s): pist kenarı araç kamerası (öncü boşluk, hafif sarsıntı, kadrajın ~%60'ı): düz kalkış koşusu
  8600 dev/dk, 9,5 s'de 25,7 m'de teker keser (spec 25,5 m @ 13,2 m/s); amortisörler kendiliğinden uzar. Kamera
  teker kesmeden sonra uçağın tırmanışıyla (gecikmeli) yükselir: uçağın 0,1–0,25 m üstünde kalır, ufuk kadrajda
  (y 0,52–0,57), gölge tekerden ayrılırken kalkış okunur.
* C (10,6–17,6 s): takip düzeneği (``U_CamRig_Follow`` uçağın konum + baş açısını izler; kamera uçakla aynı
  yükseklikte → arkada zemin ve ufuk). Takım 12 s'de (≈ 2,8 m AGL, pozitif tırmanışta) gerçek sırayla toplanır
  (kapak 1 s + bacak 5 s + kapak 1 s, ER-150 8,4 V'ta 5 s); 12,6 s'den sonra 26° yatışlı tırmanan sol dönüş.
* D (17,6–21 s): uçakla aynı yükseklikte (tepe/direk) yer kamerası, kameraman zum'u: lens uçağın kadrajdaki
  izdüşüm genişliğini ~%60'ta tutar (35–300 mm, gecikmeli), öncü boşluk, el titremesi; en yakın geçiş ≈ 20 s'de
  (12 m).
  Taret 11–12,2 s'de betikten D kamerasını izlemeye geçer. Flap 17,4–18,6 s'de 0.

Uçuş evresi zamanları ilk 19 s'lik planın ``PRE`` (2 s) kaydırılmışıdır (``_T()``); kumanda kontrolü (A1–A2)
kendi zamanlarındadır.

``mechanisms`` (16 s, 384 kare, kesintisiz döngü, stüdyo)
--------------------------------------------------------
Uçak bakım sehpasında (``U_Stand_Mechanisms``, iki kriko, tekerler yerden ~6 cm yukarıda; amortisörler serbest),
pervane 240 dev/dk (marş motoruyla çevirme; döngüde tam 64 tur, palalar ve dönüş yönü okunur). İşaretlerle kesilen
yakın planlar:

* 0–0,6 s geniş 3/4 (yörünge kamerası ``U_Cam_Mechanisms``).
* 0,6–2,0 s kanatçık (+20/−20, sol kanatçık yakın planı, 85 mm); 2,0–4,0 s flap 0 → 30 → 0 (flap yakın planı).
* 4,0–5,8 s irtifa (−25/+20) ve istikamet (±22) (kuyruk yakın planı, 70 mm).
* 5,8–8,6 s istikamet → burun tekeri yönlendirme ±30°, sonra taret ±100° tarama + eğilme (burun yakın planı).
* 8,6–12,7 s takım toplama (sol ana takım yakın planı; dışarıdan, hafif önden — arka kriko kadrajın sağ üçte
  birinde, tekerin arkasında değil): kapak 0,54 s + bacak 2,52 s + kapak 0,54 s (gerçek sürenin yarısı, doğrusal
  anahtar).
* 12,4–12,7 s takım toplu ve kapaklar kapalı bekler (yakın planda okunur), 12,7–16,0 s takım açılır (kapak 0,50 s +
  bacak 2,31 s + kapak 0,50 s), geniş yörünge; son kare ilk kareyle aynı poz.
"""
from __future__ import annotations

import functools
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
ANIM_COL = "UCAV_Cameras_Anim"
STAND_COL = "UCAV_Stand"
CLIPS = {"showcase": 21.0, "mechanisms": 16.0}           # süre (s)
MARK = "YK38_"                                           # zaman çizelgesi işaret öneki
SENSOR = 36.0                                            # mm, yatay (16:9 çıktı)
ASPECT = 16.0 / 9.0

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
WHEEL_TAU = 0.6                                          # teker kesmeden sonra yavaşlama zaman sabiti (s). varsayım
MECH_RPM = 240.0                                         # sehpada pervane (marş motoru): 16 s'de tam 64 tur


def nframes(name: str) -> int:
    return int(round(CLIPS[name] * FPS))


def _fr(t: float) -> int:
    """Zaman (s) → kare (kare 1 = 0 s)."""
    return int(round(t * FPS)) + 1


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


def _smooth(x: np.ndarray, tau: float) -> np.ndarray:
    """Sıfır fazlı yumuşatma (ileri + geri gecikme): kameraman yaklaşan uçağı önceden sezer, zum geride kalmaz."""
    return _lag(_lag(x, tau)[::-1], tau)[::-1]


def _lag(x: np.ndarray, tau: float, dt: float = 1.0 / FPS) -> np.ndarray:
    """Birinci derece gecikme süzgeci (kamera yumuşatma)."""
    x = np.asarray(x, float)
    y = np.array(x, float, copy=True)
    k = dt / (tau + dt)
    for i in range(1, len(y)):
        y[i] = y[i - 1] + k * (x[i] - y[i - 1])
    return y


def _unit(v: np.ndarray) -> np.ndarray:
    v = np.asarray(v, float)
    n = np.linalg.norm(v, axis=-1, keepdims=True)
    return v / np.maximum(n, 1e-12)


# =====================================================================================================
# Kadraj geometrisi (saf numpy): uçağın uç noktaları, Track To kamera ekseni, izdüşüm
# =====================================================================================================
@functools.lru_cache(maxsize=1)
def _hull_local_cached() -> tuple:
    root0 = np.asarray(P.U_ROOT_B, float)
    pts = []
    try:
        y = P.WING_SEMI_SPAN
        for sg in (1.0, -1.0):
            st = P.wing_station(sg * y * 0.995)
            pts += [P.to_blender(st.le_s, sg * y, st.z_ref), P.to_blender(st.te_s, sg * y, st.z_ref)]
            st = P.wing_station(sg * 0.5 * y)
            pts += [P.to_blender(st.le_s, sg * 0.5 * y, st.z_ref), P.to_blender(st.te_s, sg * 0.5 * y, st.z_ref)]
            ss = P.stab_station(sg * P.STAB_HALF_SPAN)
            pts += [P.to_blender(ss.le_s, sg * P.STAB_HALF_SPAN, ss.z),
                    P.to_blender(ss.te_s, sg * P.STAB_HALF_SPAN, ss.z)]
            fs = P.fin_station(float(P.SPEC["tail"]["fin"]["height_m"]), "L" if sg > 0 else "R")
            pts += [P.to_blender(*fs.le), P.to_blender(fs.te_s, fs.le[1], fs.le[2])]
        for s in np.linspace(0.0, P.FUSELAGE_LENGTH, 9):
            fsec = P.fuselage_section(float(s))
            pts += [P.to_blender(s, 0.0, fsec.z_top), P.to_blender(s, 0.0, fsec.z_bottom),
                    P.to_blender(s, fsec.half_width, fsec.z_center), P.to_blender(s, -fsec.half_width, fsec.z_center)]
        hub = np.asarray(P.to_blender(*P.PROP.hub), float)
        r = float(P.PROP.radius)
        pts += [hub + (0, 0, r), hub - (0, 0, r), hub + (0, r, 0), hub - (0, r, 0), hub + (-0.06, 0, 0)]
        for leg in ("N", "L", "R"):
            g = P.gear_leg(leg)
            a = np.asarray(P.to_blender(*g.pivot), float)
            pts += [(a[0], a[1], P.GROUND_Z)]
    except Exception:                                     # geometri API'si değişirse kaba kutu
        pts = [(0.0, 0, 0), (-2.48, 0, 0.1), (-1.3, 1.9, 0.13), (-1.3, -1.9, 0.13), (-2.3, 0.6, 0.38),
               (-2.3, -0.6, 0.38), (-0.6, 0, P.GROUND_Z), (-1.3, 0.33, P.GROUND_Z)]
    return tuple(map(tuple, np.asarray(pts, float) - root0))


def _hull_local() -> np.ndarray:
    """Uçağın uç noktaları (``U_Root``'a göre, Blender takımı, dinlenme pozu): kanat/stabilize/dikey uçları, gövde
    üst/alt/yan çizgileri, pervane diski, tekerler. Kadraj hesapları için."""
    return np.asarray(_hull_local_cached(), float)


def _cam_axes(cam: np.ndarray, tgt: np.ndarray):
    """Track To (−Z hedefe, +Y dünya Z'sine yakın): ileri, sağ, yukarı birim vektörleri."""
    f = _unit(np.asarray(tgt, float) - np.asarray(cam, float))
    r = _unit(np.cross(f, [0.0, 0.0, 1.0]))
    u = np.cross(r, f)
    return f, r, u


def _project(cam: np.ndarray, tgt: np.ndarray, pts: np.ndarray):
    """Kamera (n,3), hedef (n,3), noktalar (n,k,3) → görüntü düzlemi teğetleri x, y (n,k) ve derinlik."""
    cam = np.atleast_2d(cam)
    tgt = np.atleast_2d(tgt)
    f, r, u = _cam_axes(cam, tgt)
    d = pts - cam[:, None, :]
    z = np.einsum("nkj,nj->nk", d, f)
    return np.einsum("nkj,nj->nk", d, r) / z, np.einsum("nkj,nj->nk", d, u) / z, z


def _frame_fraction(x: np.ndarray, y: np.ndarray, lens) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    """Kadraj kesirleri (0…1, ``world_to_camera_view`` gibi): xmin, xmax, ymin, ymax."""
    lens = np.asarray(lens, float).reshape(-1, 1)
    kx = lens / SENSOR
    ky = lens / (SENSOR / ASPECT)
    return (0.5 + x.min(1, keepdims=True) * kx).ravel(), (0.5 + x.max(1, keepdims=True) * kx).ravel(), \
        (0.5 + y.min(1, keepdims=True) * ky).ravel(), (0.5 + y.max(1, keepdims=True) * ky).ravel()


def _horizon_y(cam: np.ndarray, tgt: np.ndarray, lens) -> np.ndarray:
    """Uzak zemin çizgisinin (ufuk) kadrajdaki yüksekliği (0 = alt, 1 = üst)."""
    f, r, u = _cam_axes(np.atleast_2d(cam), np.atleast_2d(tgt))
    pitch = np.arcsin(np.clip(f[:, 2], -1, 1))
    return 0.5 + np.tan(-pitch) * np.asarray(lens, float) / (SENSOR / ASPECT)


def _world_points(loc: np.ndarray, Rm: np.ndarray, hull: np.ndarray) -> np.ndarray:
    return loc[:, None, :] + np.einsum("nij,kj->nki", Rm, hull)


# =====================================================================================================
# Planlar (saf numpy; bpy gerektirmez)
# =====================================================================================================
PRE = 2.0                                                # açılış planı uzatması (s): kuruluş planı 2 s
T_ROLL = PRE + 3.4                                       # kalkış koşusu başlangıcı (s)


def _T(t: float) -> float:
    """Uçuş evresi zamanı: 19 s'lik ilk plandaki ``t`` → bu klipte (açılış ``PRE`` kadar uzun)."""
    return PRE + t


CONTROL_KEYS_SHOWCASE: dict[str, list[tuple[float, float]]] = {
    # takım: pozitif tırmanışta (≈ 2,8 m AGL, teker kesmeden 2,5 s sonra) toplanır; bacaklar 13,05–17,95 s
    "gear": [(0.0, 1.0), (_T(10.0), 1.0), (_T(17.0), 0.0)],
    "gear_doors": [(0.0, 0.0)],
    "prop_rpm": [(0.0, 0.0), (0.45, 0.0), (1.25, 3600.0), (1.9, 3000.0), (T_ROLL, 3000.0), (_T(4.0), 8600.0),
                 (_T(11.0), 8600.0), (_T(12.4), 7400.0), (_T(19.0), 7400.0)],
    # kumanda kontrolü (yerde, tam sapma): kanatçık 2,0–2,95 s (A1), flap 2,95–3,95 s (A1F), irtifa ve istikamet
    # 3,95–5,4 s (A2; burun tekeri istikametle döner)
    "aileron_deg": [(0.0, 0.0), (2.05, 0.0), (2.3, 20.0), (2.62, -20.0), (2.88, 0.0), (_T(5.0), 0.0), (_T(5.6), 2.0),
                    (_T(6.3), -1.5), (_T(7.0), 0.0), (_T(10.5), 0.0), (_T(10.9), -12.0), (_T(11.6), -1.0),
                    (_T(16.4), -1.0), (_T(16.9), 6.0), (_T(17.6), 0.0), (_T(19.0), 0.0)],
    "elevator_deg": [(0.0, 0.0), (4.0, 0.0), (4.22, -25.0), (4.5, 20.0), (4.68, 0.0), (_T(6.4), 0.0), (_T(7.1), -6.0),
                     (_T(8.0), -3.0), (_T(10.6), -3.0), (_T(11.6), -6.0), (_T(16.4), -6.0), (_T(17.4), -2.0),
                     (_T(19.0), -2.0)],
    "rudder_deg": [(0.0, 0.0), (4.7, 0.0), (4.9, 22.0), (5.12, -22.0), (5.3, 0.0), (_T(4.8), 0.0), (_T(5.3), -3.0),
                   (_T(6.0), 2.5), (_T(6.7), 0.0), (_T(10.5), 0.0), (_T(10.9), -5.0), (_T(11.8), -1.5),
                   (_T(16.4), -1.5), (_T(17.0), 2.5), (_T(17.6), 0.0), (_T(19.0), 0.0)],
    "flap_deg": [(0.0, 0.0), (3.05, 0.0), (3.55, 20.0), (_T(15.4), 20.0), (_T(16.6), 0.0)],
    "nav_lights": [(0.0, 1.0)],
    "strobe": [(0.0, 1.0)],
    "status_led": [(0.0, 0.0)],
    "prop_auto": [(0.0, 0.0)],                            # açı = U_Prop["ucav_turns"] (pişirilmiş ∫rpm)
}
TURRET_SCRIPT_SHOWCASE = {"pan": [(0.0, 0.0), (0.35, 0.0), (0.85, -40.0), (1.35, 35.0), (1.9, 0.0), (_T(9.0), 0.0)],
                          "tilt": [(0.0, 0.0), (0.35, 0.0), (0.85, -15.0), (1.35, -5.0), (1.9, -10.0),
                                   (_T(9.0), -10.0)]}
TRACK_BLEND = (_T(9.0), _T(10.2))                        # betik → hedef izleme geçişi (s)
SHOT_TIMES = {"A": 0.0, "A1": 2.0, "A1F": 2.95, "A2": 3.95, "B": T_ROLL, "C": _T(8.6), "D": _T(15.6)}
EVENTS_SHOWCASE = {"ev_motor": 0.45, "ev_kumanda_kontrol": 2.0, "ev_kalkis": T_ROLL, "ev_takim_yukari": _T(10.0),
                   "ev_donus": _T(10.6), "ev_flap_0": _T(15.4)}
D_PASS_T = _T(17.6)                                      # D: en yakın geçiş anı (s)
D_SIDE, D_AHEAD, D_BELOW = 12.0, 6.0, 0.3                # D: geçiş noktasının solunda/önünde, uçağın altında (m)
D_FILL, D_LENS = 0.60, (35.0, 300.0)                     # D: izdüşüm genişliği / kadraj, lens sınırları
LEAD = {"B": 0.10, "D": 0.10}                            # öncü boşluk (kadraj genişliği oranı)
B_Y, B_FILL, B_LENS, B_AHEAD = 3.6, 0.60, (24.0, 70.0), 1.4   # B: pist kenarı uzaklığı, kadraj, zum, araç önde (m)


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


@functools.lru_cache(maxsize=None)
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


def _hinge_mid(name: str, side: str = "L") -> np.ndarray:
    return np.asarray(P.hinge_line(name, side).mid_b, float)


def _flight_path(n: int, t: np.ndarray, t_roll: float):
    """Hız, konum, euler ve dönüş matrisleri; teker kesme anı."""
    dt = 1.0 / FPS
    a_max = _takeoff_accel()
    v = np.zeros(n)
    t_lof, i_lof = None, None
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
            t_lof, i_lof = t[i], i
    if t_lof is None:
        t_lof, i_lof = t_roll + 3.9, int(round((t_roll + 3.9) * FPS))
    gamma = np.radians(_ease(t, [(t_lof, 0.0), (t_lof + 1.8, 7.0), (_T(12.0), 7.0), (_T(14.5), 2.5), (_T(16.5), 0.5)]))
    alpha = np.radians(_ease(t, [(t_lof - 0.15, 0.0), (t_lof + 0.6, 2.2), (_T(19.0), 2.2)]))
    bank = np.radians(_ease(t, [(_T(10.6), 0.0), (_T(11.9), -26.0), (_T(16.4), -26.0),     # − = sol kanat aşağı
                                (_T(17.6), -12.0)]))
    yaw = np.zeros(n)
    pos = np.zeros((n, 3))
    for i in range(1, n):
        yaw[i] = yaw[i - 1] + G0 * math.tan(-bank[i - 1]) / max(v[i - 1], 1.0) * dt
        cg = math.cos(gamma[i - 1])
        pos[i] = pos[i - 1] + v[i - 1] * dt * np.array([cg * math.cos(yaw[i - 1]), cg * math.sin(yaw[i - 1]),
                                                       math.sin(gamma[i - 1])])
    pitch = gamma + alpha * (1.0 / np.cos(bank))
    loc = np.asarray(P.U_ROOT_B, float) + pos
    eul = _euler_xyz(yaw, pitch, bank)
    return v, loc, eul, _rot_matrices(eul), float(t_lof), int(i_lof), a_max, yaw


def _lead_target(loc: np.ndarray, cam: np.ndarray, lens: np.ndarray, lead: float) -> np.ndarray:
    """Öncü boşluk: hedef = kök + v̂·LEAD·kadraj genişliği (uçak hareket yönünün gerisinde kalır)."""
    vel = np.gradient(loc, axis=0) * FPS
    sp = np.linalg.norm(vel, axis=1)
    vh = _unit(vel) * np.clip(sp / 1.5, 0.0, 1.0)[:, None]          # dururken öncü yok
    width = SENSOR / np.asarray(lens, float) * np.linalg.norm(loc - cam, axis=1)
    return loc + vh * (lead * width)[:, None]


def _visual_center(cam: np.ndarray, tgt: np.ndarray, loc: np.ndarray, W: np.ndarray) -> np.ndarray:
    """Hedefi, uçağın kadrajdaki kutu merkezi (kök yerine) öncü noktaya düşecek şekilde kaydırır: perspektifte yakın
    kanat/kuyruk kutuyu kökten uzaklaştırır; öncü boşluk görünen uçağa göre kalır."""
    cam = np.broadcast_to(np.asarray(cam, float), loc.shape)
    f, r, u = _cam_axes(cam, tgt)
    x, y, _ = _project(cam, tgt, W)
    d = loc - cam
    depth = np.einsum("nj,nj->n", d, f)
    xr, yr = np.einsum("nj,nj->n", d, r) / depth, np.einsum("nj,nj->n", d, u) / depth
    cx, cy = 0.5 * (x.max(1) + x.min(1)), 0.5 * (y.max(1) + y.min(1))
    return tgt + ((cx - xr)[:, None] * r + (cy - yr)[:, None] * u) * depth[:, None]


def showcase_plan(hull: np.ndarray | None = None) -> dict:
    """Gösterim klibi: uçuş yolu (U_Root konum/euler), taret izleme açıları, kameralar (konum, hedef, lens),
    kontrol anahtarları, kamera ve olay işaretleri."""
    n = nframes("showcase")
    t = _t(n)
    t_roll = SHOT_TIMES["B"]
    v, loc, eul, Rm, t_lof, i_lof, a_max, yaw = _flight_path(n, t, t_roll)
    root0 = np.asarray(P.U_ROOT_B, float)
    hull = _hull_local() if hull is None else np.asarray(hull, float)
    W = _world_points(loc, Rm, hull)
    # ---------------------------------------------------------------- D: uçak yüksekliğinde yer kamerası
    i_pass = min(n - 1, int(round(D_PASS_T * FPS)))
    hd = np.array([math.cos(yaw[i_pass]), math.sin(yaw[i_pass]), 0.0])
    left = np.array([-hd[1], hd[0], 0.0])
    cam_d = loc[i_pass] * np.array([1.0, 1.0, 0.0]) + left * D_SIDE + hd * D_AHEAD
    cam_d[2] = loc[i_pass, 2] - D_BELOW
    camD = np.tile(cam_d, (n, 1))
    x, _, _ = _project(camD, loc, W)
    lens0 = np.clip(D_FILL * SENSOR / (x.max(1) - x.min(1)), *D_LENS)
    tgt_d = _lead_target(loc, camD, lens0, LEAD["D"])
    x, _, _ = _project(camD, tgt_d, W)
    lens_d = np.clip(_smooth(D_FILL * SENSOR / (x.max(1) - x.min(1)), 0.2), *D_LENS)
    tgt_d = _visual_center(camD, _lead_target(loc, camD, lens_d, LEAD["D"]), loc, W)
    # ---------------------------------------------------------------- taret: betik → D kamerasını izleme
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
    # ---------------------------------------------------------------- A (kuruluş), A1, A1F, A2 (yakın planlar)
    # A: ön-sol 3/4, 2 s yaklaşma + alçalma (planform görünür → alçak kahraman açısı); motor çalışır, taret bakınır
    t_a = SHOT_TIMES["A1"] + 0.05
    cam_a = np.column_stack([_ease(t, [(0.0, 2.95), (t_a, 2.05)]), _ease(t, [(0.0, 3.60), (t_a, 2.85)]),
                             _ease(t, [(0.0, 0.62), (t_a, 0.07)])])
    tgt_a = root0 + np.array([-0.02, 0.45, -0.04])
    # A1: kanatçık, dış-arka-üstten (açıklık boyunca kısalır, firar kenarı hareketi bakışa dik)
    ail, flo, fli = _hinge_mid("Aileron"), _hinge_mid("FlapOut"), _hinge_mid("FlapIn")
    cam_a1 = ail + _unit(np.array([-1.0, 1.3, 0.3])) * 1.6
    tgt_a1 = ail + np.array([0.02, 0.03, -0.01])
    # A1F: iki flap, arka-üst 3/4 (gövde ve ana takım arka planda) — kanatçıktan kesmeyle (pan yok)
    tgt_af = 0.75 * flo + 0.25 * fli + np.array([-0.02, 0.0, 0.0])
    cam_af = tgt_af + _unit(np.array([-1.0, 0.8, 0.45])) * 2.2
    # A2: kuyruk 3/4 arka-sol-üst: irtifa ve istikamet aynı kadrajda
    ele = _hinge_mid("Elevator")
    stab_mid = np.array([ele[0], 0.0, ele[2]])
    tgt_a2 = stab_mid + np.array([-0.09, 0.27, 0.07])
    cam_a2 = tgt_a2 + _unit(np.array([-1.65, 1.05, 0.55])) * 2.03
    # ---------------------------------------------------------------- B: pist kenarı araç kamerası
    cam_b = np.column_stack([_lag(loc[:, 0] + B_AHEAD, 0.55), np.full(n, B_Y), np.full(n, P.GROUND_Z + 0.50)])
    i_roll = int(round(t_roll * FPS))
    cam_b[:i_roll, 0] = cam_b[i_roll, 0]
    # kalkışta uçağın tırmanışıyla (gecikmeli) hafif yükselir: kamera uçağın biraz üstünde kalır, ufuk kadrajda,
    # teker kesme yer çizgisinin üstünde okunur (eski sabit +1,4 m rampa uçağın 1,3 m üstüne çıkıp ufku kadrajın
    # tepesine itiyordu)
    climb = np.maximum(loc[:, 2] - loc[i_roll, 2], 0.0)
    cam_b[:, 2] += 0.8 * _lag(climb, 0.35) + _ease(t, [(_T(7.4), 0.0), (_T(8.6), 0.15)])
    x, _, _ = _project(cam_b, loc, W)                                     # kameraman zum'u: kadrajın ~%60'ı
    lens_b = np.clip(_smooth(B_FILL * SENSOR / (x.max(1) - x.min(1)), 0.3), *B_LENS)
    tgt_b = _visual_center(cam_b, _lead_target(loc, cam_b, lens_b, LEAD["B"]), loc, W)
    # ---------------------------------------------------------------- C: takip düzeneği (yerel ofsetler)
    follow = [(_T(8.0), (-6.8, -3.2, 0.25)), (_T(9.8), (-6.3, -3.4, -0.20)), (_T(13.6), (-4.6, -3.9, 0.15))]
    tgt_c_local = (0.25, 0.0, -0.06)
    cams = {
        "A": {"loc": cam_a, "target": tgt_a, "lens": 40.0, "fstop": 5.6},
        "A1": {"loc": cam_a1, "target": tgt_a1, "lens": 85.0, "fstop": 11.0},
        "A1F": {"loc": cam_af, "target": tgt_af, "lens": 70.0, "fstop": 8.0},
        "A2": {"loc": cam_a2, "target": tgt_a2, "lens": 80.0, "fstop": 5.6},
        "B": {"loc": cam_b, "target": tgt_b, "lens": lens_b, "fstop": 4.0, "noise": (0.12, 14.0)},
        "C": {"follow": follow, "target_local": tgt_c_local, "lens": 40.0, "fstop": 8.0},
        "D": {"loc": cam_d, "target": tgt_d, "lens": lens_d, "fstop": 5.6, "noise": (0.6, 40.0)},
    }
    markers = [(_fr(SHOT_TIMES[k]), f"U_Cam_Showcase_{k}") for k in cams]
    events = sorted([(_fr(tt), k) for k, tt in EVENTS_SHOWCASE.items()] + [(i_lof + 1, "ev_teker_kesme")])
    return {"n": n, "t": t, "v": v, "loc": loc, "euler": eul, "rot": Rm, "hull": hull, "t_lof": t_lof,
            "i_lof": i_lof, "x_lof": float(loc[i_lof, 0] - root0[0]), "a_max": a_max, "pan": pan, "tilt": tilt,
            "cams": cams, "cam_d": cam_d, "lens_d": lens_d, "i_pass": i_pass, "follow_offset": follow,
            "markers": markers, "events": events, "controls": CONTROL_KEYS_SHOWCASE, "linear": {"gear"},
            "constant": {"prop_auto", "status_led"}, "t_brake": CONTROL_KEYS_SHOWCASE["gear"][1][0]}


def framing_report(plan: dict | None = None) -> dict:
    """Plan tarafı kadraj ölçüleri (saf numpy, testler için): B ve D çekimlerinde uçağın izdüşüm genişliği (kadraj
    oranı), yatay aralığı, uzak zemin çizgisinin (ufuk) yüksekliği ve lens — kare başına. Uç noktalar
    ``_hull_local``."""
    sp = plan or showcase_plan()
    W = _world_points(sp["loc"], sp["rot"], sp["hull"])
    ends = {"B": SHOT_TIMES["C"], "D": CLIPS["showcase"] + 1.0 / FPS}
    out = {}
    for key in ("B", "D"):
        i0, i1 = _fr(SHOT_TIMES[key]) - 1, min(sp["n"], _fr(ends[key]) - 1)
        c = sp["cams"][key]
        cam = np.broadcast_to(np.asarray(c["loc"], float), sp["loc"].shape)[i0:i1]
        tgt = np.asarray(c["target"], float)[i0:i1]
        lens = np.broadcast_to(np.asarray(c["lens"], float), (sp["n"],))[i0:i1]
        x, y, _ = _project(cam, tgt, W[i0:i1])
        x0, x1, y0, y1 = _frame_fraction(x, y, lens)
        out[key] = {"frames": (i0 + 1, i1), "width": x1 - x0, "x": (x0, x1), "y": (y0, y1),
                    "horizon": _horizon_y(cam, tgt, lens), "lens": np.asarray(lens)}
    return out


# ----------------------------------------------------------------------------------------------- mekanizmalar
CONTROL_KEYS_MECH: dict[str, list[tuple[float, float]]] = {
    # takım: gerçek sürenin yarısı, DOĞRUSAL (kapak 0,54 s + bacak 2,52 s + kapak 0,54 s; sürücü bacağı zaten
    # yumuşatır); toplu hâl 0,3 s tutulur (kapaklar kapandıktan sonra kesme), açılış 3,3 s
    "gear": [(0.0, 1.0), (8.8, 1.0), (12.4, 0.0), (12.7, 0.0), (16.0, 1.0)],
    "gear_doors": [(0.0, 0.0)],
    "prop_rpm": [(0.0, MECH_RPM), (16.0, MECH_RPM)],
    "aileron_deg": [(0.0, 0.0), (0.75, 0.0), (1.1, 20.0), (1.55, -20.0), (1.9, 0.0), (16.0, 0.0)],
    "flap_deg": [(0.0, 0.0), (2.05, 0.0), (2.7, 30.0), (3.2, 30.0), (3.85, 0.0), (16.0, 0.0)],
    "elevator_deg": [(0.0, 0.0), (4.1, 0.0), (4.4, -25.0), (4.85, 20.0), (5.1, 0.0), (16.0, 0.0)],
    "rudder_deg": [(0.0, 0.0), (5.1, 0.0), (5.33, 22.0), (5.58, -22.0), (5.78, 0.0), (5.95, 0.0), (6.25, 22.0),
                   (6.7, -22.0), (6.95, 0.0), (16.0, 0.0)],
    "turret_pan_deg": [(0.0, 0.0), (6.85, 0.0), (7.45, 100.0), (8.15, -100.0), (8.55, 0.0), (16.0, 0.0)],
    "turret_tilt_deg": [(0.0, 0.0), (6.85, 0.0), (7.35, -35.0), (7.8, 12.0), (8.25, -55.0), (8.55, 0.0), (16.0, 0.0)],
    "nav_lights": [(0.0, 1.0)],
    "strobe": [(0.0, 1.0)],
    "status_led": [(0.0, 0.0)],
    "prop_auto": [(0.0, 0.0)],
    "wheel_auto": [(0.0, 0.0)],
    "wheel_roll_m": [(0.0, 0.0)],
}
SHOT_TIMES_MECH = {"": 0.0, "Aileron": 0.6, "Flap": 2.0, "Tail": 4.0, "Nose": 5.8, "Gear": 8.6, "_wide": 12.7}


def mechanisms_plan() -> dict:
    """Mekanizma döngüsü: sehpada sabit kök, sıralı kumanda anahtarları, yörünge kamerası ve yakın plan kameraları."""
    n = nframes("mechanisms")
    T = CLIPS["mechanisms"]
    t = _t(n + 1)                                                        # n + 1: döngü dikişi (kare N+1 = kare 1)
    root0 = np.asarray(P.U_ROOT_B, float)
    lift = np.array([0.0, 0.0, STAND_LIFT])
    root_m = root0 + lift
    loc = np.tile(root_m, (n + 1, 1))
    eul = np.zeros((n + 1, 3))
    # geniş yörünge: dikişin çevresinde (12,7 → 16 → 0,6 s) yavaş, sürekli pan; periyodik → kesintisiz döngü
    ph = 2 * math.pi * (t - 14.5) / T
    orbit_z = np.radians(34.0 + 14.0 * np.sin(ph))
    orbit_h = np.full(n + 1, 0.42)                                      # kamera hedefin 0,42 m üstünde
    center = root_m + np.array([0.0, 0.35, 0.03])                      # yakın (sol) kanada doğru: kadrajda ortalı
    # yakın planlar (sehpadaki uçak) — hedef, yön, uzaklık, lens, f-sayısı
    ail, fli, flo = (_hinge_mid(k) + lift for k in ("Aileron", "FlapIn", "FlapOut"))
    ele, rud = _hinge_mid("Elevator") + lift, _hinge_mid("Rudder") + lift
    ball = np.asarray(P.to_blender(*P.TURRET.ball_center), float) + lift
    gn, gl = P.gear_leg("N"), P.gear_leg("L")
    nose_ax = np.asarray(P.to_blender(*gn.axle_unloaded), float) + lift          # sehpada bacaklar yüksüz
    main_ext = np.asarray(P.to_blender(*gl.axle_unloaded), float) + lift
    main_ret = np.asarray(P.to_blender(*gl.axle_retracted), float) + lift

    def shot(tgt, direction, dist, lens, fstop, drift=(0.0, 0.0, 0.0)):
        d = _unit(np.asarray(direction, float))
        p0 = np.asarray(tgt, float) + d * dist
        return {"loc": p0, "loc_end": p0 + np.asarray(drift, float), "target": np.asarray(tgt, float),
                "lens": float(lens), "fstop": float(fstop)}

    cams = {
        "Aileron": shot(ail + np.array([0.01, 0.02, 0.0]), (-0.7, 1.0, 0.16), 1.45, 85.0, 8.0, (0.0, -0.04, 0.0)),
        "Flap": shot(0.5 * (fli + flo) + np.array([0.0, 0.08, -0.01]), (-0.7, 1.0, 0.45), 1.95, 85.0, 5.6,
                     (0.0, 0.05, 0.0)),
        "Tail": shot(0.5 * (ele + rud) + np.array([0.0, -0.02, -0.02]), (-1.0, 0.9, 0.48), 1.6, 70.0, 5.6,
                     (0.0, -0.04, 0.02)),
        "Nose": shot(0.5 * (ball + nose_ax) + np.array([0.0, 0.0, 0.01]), (1.0, 0.62, -0.03), 1.30, 85.0, 5.6,
                     (0.0, 0.03, 0.0)),
        # ana takım: dışarıdan, hafif önden (az ≈ 78°, el −6°) — arka kriko (s 1,62) tekerin tam arkasında kadraj
        # ortasında kalmasın (eski (1, 0,85) yönünde x = 0,50'deydi; şimdi ≈ 0,72, ön kriko kadraj dışında)
        "Gear": shot(0.5 * (main_ext + main_ret) + np.array([0.0, 0.02, -0.01]), (0.207, 0.973, -0.105), 1.0, 45.0, 5.6,
                     (0.0, 0.0, 0.01)),
    }
    segs = sorted((tt, k) for k, tt in SHOT_TIMES_MECH.items())
    for (t0, k), (t1, _) in zip(segs, segs[1:] + [(T, None)]):
        if k in cams:
            cams[k]["t"] = (t0, t1)
    markers = [(_fr(tt), "U_Cam_Mechanisms" + (f"_{k}" if k and not k.startswith("_") else ""))
               for tt, k in segs]
    return {"n": n, "t": t, "loc": loc, "euler": eul, "orbit_z": orbit_z, "orbit_h": orbit_h, "center": center,
            "radius": 4.2, "orbit_lens": 35.0, "orbit_fstop": 4.0, "cams": cams, "markers": markers,
            "events": [(_fr(8.8), "ev_takim_yukari"), (_fr(12.7), "ev_takim_asagi")],
            "controls": CONTROL_KEYS_MECH, "linear": {"gear", "prop_rpm"},
            "constant": {"prop_auto", "status_led", "wheel_auto", "wheel_roll_m"}, "prop_rpm_const": MECH_RPM}


# =====================================================================================================
# Aksiyon yazımı
# =====================================================================================================
_INTERP = {"CONSTANT": 0, "LINEAR": 1, "BEZIER": 2}


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
    fc.keyframe_points.foreach_set("interpolation", np.full(len(vals), _INTERP["LINEAR"], dtype=np.int32))
    fc.update()
    return fc


def _sparse(act, path: str, index: int, keys, linear: bool = False, constant: bool = False) -> bpy.types.FCurve:
    """Seyrek anahtarlar ``[(t s, değer), …]``: düzenlenebilir; BEZIER + AUTO_CLAMPED (ya da LINEAR/CONSTANT)."""
    fc = act.fcurves.new(path, index=index)
    fc.keyframe_points.add(len(keys))
    for kp, (tt, vv) in zip(fc.keyframe_points, keys):
        kp.co = (tt * FPS + 1.0, float(vv))
        kp.interpolation = "CONSTANT" if constant else ("LINEAR" if linear else "BEZIER")
        kp.handle_left_type = kp.handle_right_type = "AUTO_CLAMPED"
    fc.update()
    return fc


def _rdp(y: np.ndarray, tol: float) -> np.ndarray:
    """Ramer–Douglas–Peucker (düşey sapma): doğrusal parçalarla ``tol`` içinde kalan anahtar indisleri."""
    n = len(y)
    keep = np.zeros(n, bool)
    keep[0] = keep[-1] = True
    stack = [(0, n - 1)]
    while stack:
        a, b = stack.pop()
        if b - a < 2:
            continue
        xs = np.arange(a, b + 1)
        line = y[a] + (y[b] - y[a]) * (xs - a) / (b - a)
        err = np.abs(y[a:b + 1] - line)
        i = int(np.argmax(err))
        if err[i] > tol:
            k = a + i
            keep[k] = True
            stack += [(a, k), (k, b)]
    return np.flatnonzero(keep)


def _fit(act, path: str, index: int, values, tol: float, start_frame: int = 1) -> bpy.types.FCurve:
    """Kare kare değerlere seyrek BEZIER (AUTO_CLAMPED) eğri: RDP adayları, sonra değerlendirilen eğri her karede
    ``tol`` içinde kalana dek en kötü kareye anahtar eklenir. Düzenlenebilir ve planla aynı (± tol)."""
    vals = np.asarray(values, float)
    n = len(vals)
    fr = np.arange(start_frame, start_frame + n, dtype=float)
    span = float(vals.max() - vals.min())
    idx = _rdp(vals, max(25.0 * tol, 0.02 * span))                      # kaba adaylar; ayrıntıyı doğrulama ekler
    fc = act.fcurves.new(path, index=max(index, 0))
    kps = fc.keyframe_points
    kps.add(len(idx))
    kps.foreach_set("co", np.column_stack([fr[idx], vals[idx]]).ravel())
    for kp in kps:
        kp.interpolation = "BEZIER"
        kp.handle_left_type = kp.handle_right_type = "AUTO_CLAMPED"
    fc.update()
    have = set(int(i) for i in idx)
    for _ in range(4 * n):
        ev = np.fromiter((fc.evaluate(f) for f in fr), float, n)
        err = np.abs(ev - vals)
        i = int(np.argmax(err))
        if err[i] <= tol or i in have:
            break
        kp = kps.insert(fr[i], float(vals[i]), options={"FAST"})
        kp.interpolation = "BEZIER"
        kp.handle_left_type = kp.handle_right_type = "AUTO_CLAMPED"
        have.add(i)
        fc.update()
    return fc


def _drop_action(name: str) -> None:
    act = bpy.data.actions.get(name)
    if act is not None:
        bpy.data.actions.remove(act)


def _assign(ob: bpy.types.Object, act: bpy.types.Action | None) -> None:
    if ob.animation_data is None:
        ob.animation_data_create()
    ob.animation_data.action = act


TOL_LOC, TOL_ROT, TOL_TURRET, TOL_LENS = 0.005, math.radians(0.1), 0.2, 0.5


def _controls(act, plan: dict) -> None:
    for prop, keys in plan["controls"].items():
        _sparse(act, f'["{prop}"]', 0, keys, linear=prop in plan["linear"], constant=prop in plan["constant"])


def _root_action(name: str, plan: dict, sparse: bool = True) -> bpy.types.Action:
    """``U_Root`` aksiyonu: konum/euler (seyrek BEZIER ya da kare kare), kontrol anahtarları, taret açıları."""
    act = _action(f"YK38_{name}_Root" + ("" if sparse else "_pisirilmis"))
    for i in range(3):
        rot = np.unwrap(plan["euler"][:, i]) if i == 2 else plan["euler"][:, i]
        if sparse:
            _fit(act, "location", i, plan["loc"][:, i], TOL_LOC)
            _fit(act, "rotation_euler", i, rot, TOL_ROT)
        else:
            _dense(act, "location", i, plan["loc"][:, i])
            _dense(act, "rotation_euler", i, rot)
    _controls(act, plan)
    if "pan" in plan:
        for prop, vals in (("turret_pan_deg", plan["pan"]), ("turret_tilt_deg", plan["tilt"])):
            if sparse:
                _fit(act, f'["{prop}"]', 0, vals, TOL_TURRET)
            else:
                _dense(act, f'["{prop}"]', 0, vals)
    return act


def _wheel_keys(act: bpy.types.Action, plan: dict) -> dict:
    """Gösterim tekerleri: yer koşusunda ``wheel_auto`` = 1 (tekerler U_Root X'ine kaymasız bağlı); teker kesme
    karesinde ``wheel_roll_m`` o anki yola atlar ve ``wheel_auto`` = 0 (açı sürekli); τ = WHEEL_TAU ile yavaşlar,
    takım toplanmaya başlayınca (fren) sabit."""
    fx = act.fcurves.find("location", index=0)
    F = plan["i_lof"] + 1
    x_s = fx.evaluate(F)
    h = 0.05
    v_s = (fx.evaluate(F + h) - fx.evaluate(F - h)) / (2 * h) * FPS
    dist_s = x_s - P.U_ROOT_B[0]
    t_s = (F - 1) / FPS
    t_b = plan["t_brake"]
    dts = [0.0, 0.12, 0.3, 0.55, 0.85, 1.25, 1.75, t_b - t_s]
    roll = [(t_s + d, dist_s + v_s * WHEEL_TAU * (1 - math.exp(-d / WHEEL_TAU))) for d in dts if d <= t_b - t_s]
    fc = act.fcurves.new('["wheel_roll_m"]', index=0)
    fc.keyframe_points.add(1 + len(roll))
    k0 = fc.keyframe_points[0]
    k0.co = (1.0, 0.0)
    k0.interpolation = "CONSTANT"
    for kp, (tt, vv) in zip(list(fc.keyframe_points)[1:], roll):
        kp.co = (tt * FPS + 1.0, vv)
        kp.interpolation = "BEZIER"
        kp.handle_left_type = kp.handle_right_type = "AUTO_CLAMPED"
    fc.update()
    _sparse(act, '["wheel_auto"]', 0, [(0.0, 1.0), (t_s, 0.0)], constant=True)
    return {"frame_lof": F, "v_lof": round(v_s, 3), "roll_lof": round(dist_s, 4)}


def _prop_action(name: str, root_act: bpy.types.Action, n: int, rpm_const: float | None = None) -> bpy.types.Action:
    """``U_Prop["ucav_turns"]`` = ∫rpm/60 dt (kare başına 4 alt örnek, trapez), kare 1…n+1, LINEAR anahtar —
    ``prop_auto`` = 0 ile açı = tur·2π: devir değişirken pervane geri dönmez. Sabit devirde 2 anahtar yeter."""
    act = _action(f"YK38_{name}_Prop")
    if rpm_const is not None:
        turns = rpm_const / 60.0 * n / FPS
        fc = act.fcurves.new(f'["{RIG.PROP_TURNS}"]', index=0)
        fc.keyframe_points.add(2)
        fc.keyframe_points.foreach_set("co", [1.0, 0.0, float(n + 1), turns])
        fc.keyframe_points.foreach_set("interpolation", [_INTERP["LINEAR"]] * 2)
        fc.extrapolation = "LINEAR"
        fc.update()
        return act
    fc = next(f for f in root_act.fcurves if f.data_path == '["prop_rpm"]')
    frames = np.arange(1, n + 2, dtype=float)
    sub = 4
    turns = [0.0]
    for f in frames[:-1]:
        s = np.linspace(f, f + 1, sub + 1)
        r = np.maximum(np.array([fc.evaluate(x) for x in s]), 0.0)
        turns.append(turns[-1] + float(np.trapezoid(r, s) if hasattr(np, "trapezoid") else np.trapz(r, s)) / 60.0 / FPS)
    f2 = _dense(act, f'["{RIG.PROP_TURNS}"]', 0, np.asarray(turns))
    f2.extrapolation = "LINEAR"
    return act


# =====================================================================================================
# Kameralar, düzenekler, sehpa
# =====================================================================================================
def _camera(name: str, col, lens: float, clip_start: float = 0.02) -> bpy.types.Object:
    U.remove_object(name)
    cam = bpy.data.cameras.get(name) or bpy.data.cameras.new(name)
    if cam.animation_data is not None:
        cam.animation_data.action = None
    cam.lens = lens
    cam.clip_start = clip_start
    cam.clip_end = 3000.0
    cam.sensor_width = 36.0
    cam.sensor_fit = "AUTO"
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


def _dof(cam_ob, target, fstop: float) -> None:
    d = cam_ob.data.dof
    d.use_dof = True
    d.focus_object = target
    d.aperture_fstop = float(fstop)
    d.aperture_blades = 7
    d.aperture_rotation = math.radians(10.0)


def _noise(act, strength: float, scale: float, seed: float) -> None:
    """Hedef konumuna yumuşak gürültü (el/araç titremesi): her eksende farklı faz."""
    for k, fc in enumerate(f for f in act.fcurves if f.data_path == "location"):
        m = fc.modifiers.new("NOISE")
        m.strength = float(strength)
        m.scale = float(scale)
        m.phase = float(seed + 7.3 * k)
        m.offset = float(13.0 * k + seed)
        m.depth = 1
        m.blend_type = "REPLACE"                       # ortalanmış: (gürültü − 0,5)·güç


def _loc_action(name: str, ob, values: np.ndarray, tol: float = TOL_LOC) -> bpy.types.Action:
    act = _action(name)
    for i in range(3):
        _fit(act, "location", i, values[:, i], tol)
    _assign(ob, act)
    return act


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


_OLD_OBJECTS = ("U_CamTarget_Showcase", "U_CamTarget_Mechanisms")
_OLD_ACTIONS = ("YK38_mechanisms_Cam",)


def _collections(scene) -> tuple[bpy.types.Collection, bpy.types.Collection]:
    cols = U.ensure_collections(scene)
    anim = U.ensure_collection(ANIM_COL, cols[STUDIO], scene)
    stand = U.ensure_collection(STAND_COL, cols[STUDIO], scene)
    for c, tag in ((anim, "COLOR_04"), (stand, "COLOR_03")):
        if hasattr(c, "color_tag"):
            c.color_tag = tag
    return anim, stand


# =====================================================================================================
# Kurulum ve klip seçimi
# =====================================================================================================
def _build_showcase(col, root) -> dict:
    sp = showcase_plan()
    out = {"cameras": [], "actions": []}
    ra = _root_action("showcase", sp, sparse=True)
    wheel = _wheel_keys(ra, sp)
    _root_action("showcase", sp, sparse=False)                         # kare kare pişirilmiş (yeniden render)
    out["actions"] += [ra.name, ra.name + "_pisirilmis"]
    if bpy.data.objects.get("U_Prop") is not None:
        out["actions"].append(_prop_action("showcase", ra, sp["n"]).name)
    cams = sp["cams"]
    for key in cams:
        c = cams[key]
        name = f"U_Cam_Showcase_{key}"
        tname = f"U_CamTarget_Showcase_{key}"
        lens = c["lens"]
        cam = _camera(name, col, float(np.max(lens)) if np.ndim(lens) else float(lens), 0.1 if key == "D" else 0.02)
        if key == "C":
            rig_f = _empty("U_CamRig_Follow", col, None, P.U_ROOT_B, 0.3, "CUBE")
            for con in list(rig_f.constraints):
                rig_f.constraints.remove(con)
            cl = rig_f.constraints.new("COPY_LOCATION")
            cl.target = root
            cr = rig_f.constraints.new("COPY_ROTATION")
            cr.target = root
            cr.use_x = cr.use_y = False
            cr.use_z = True
            cam.parent = rig_f
            cam.matrix_parent_inverse = Matrix.Identity(4)
            act = _action("YK38_showcase_CamC")
            for i in range(3):
                _sparse(act, "location", i, [(tt, off[i]) for tt, off in c["follow"]])
            _assign(cam, act)
            tgt = _empty(tname, col, root, c["target_local"], 0.1, "SPHERE")
        else:
            loc = np.asarray(c["loc"], float)
            if loc.ndim == 1:                              # sabit kamera (eski sürümden kalan aksiyon silinir)
                cam.location = Vector(tuple(loc))
                _assign(cam, None)
                _drop_action(f"YK38_showcase_Cam{key}")
            else:
                _loc_action(f"YK38_showcase_Cam{key}", cam, loc)
            tg = np.asarray(c["target"], float)
            tgt = _empty(tname, col, None, tg if tg.ndim == 1 else tg[0], 0.1, "SPHERE")
            if tg.ndim == 2:
                ta = _loc_action(f"YK38_showcase_Target{key}", tgt, tg)
                if c.get("noise"):
                    _noise(ta, *c["noise"], seed=11.0 * (len(key) + ord(key[0])))
            else:
                _drop_action(f"YK38_showcase_Target{key}")
        if np.ndim(lens):
            act = _action(f"YK38_showcase_Cam{key}_Lens")
            _fit(act, "lens", -1, lens, TOL_LENS)
            if cam.data.animation_data is None:
                cam.data.animation_data_create()
            cam.data.animation_data.action = act
        else:
            _drop_action(f"YK38_showcase_Cam{key}_Lens")
        _track_to(cam, tgt)
        _dof(cam, tgt, c["fstop"])
        out["cameras"].append(cam.name)
    out["plan"] = sp
    out["wheel"] = wheel
    return out


def _build_mechanisms(col, stand_col, root) -> dict:
    mp = mechanisms_plan()
    out = {"cameras": [], "actions": []}
    n = mp["n"]
    act = _action("YK38_mechanisms_Root")
    for i in range(3):
        _sparse(act, "location", i, [(0.0, mp["loc"][0, i]), (CLIPS["mechanisms"], mp["loc"][0, i])], linear=True)
        _sparse(act, "rotation_euler", i, [(0.0, 0.0), (CLIPS["mechanisms"], 0.0)], linear=True)
    _controls(act, mp)
    out["actions"].append(act.name)
    if bpy.data.objects.get("U_Prop") is not None:
        out["actions"].append(_prop_action("mechanisms", act, n, mp["prop_rpm_const"]).name)
    # geniş yörünge
    orbit = _empty("U_CamRig_Orbit", col, None, mp["center"], 0.3, "CIRCLE")
    a = _action("YK38_mechanisms_Orbit")
    _fit(a, "rotation_euler", 2, mp["orbit_z"], math.radians(0.05))
    _assign(orbit, a)
    tgt_m = _empty("U_CamTarget_Mechanisms_Orbit", col, None, mp["center"], 0.1, "SPHERE")
    cm = _camera("U_Cam_Mechanisms", col, mp["orbit_lens"])
    cm.parent = orbit
    cm.matrix_parent_inverse = Matrix.Identity(4)
    cm.location = Vector((mp["radius"], 0.0, float(mp["orbit_h"][0])))
    _assign(cm, None)
    _track_to(cm, tgt_m)
    _dof(cm, tgt_m, mp["orbit_fstop"])
    out["cameras"].append(cm.name)
    # yakın planlar (yavaş kayma: kesitin başından sonuna doğrusal)
    for key, c in mp["cams"].items():
        cam = _camera(f"U_Cam_Mechanisms_{key}", col, c["lens"])
        a = _action(f"YK38_mechanisms_Cam{key}")
        t0, t1 = c.get("t", (0.0, CLIPS["mechanisms"]))
        for i in range(3):
            _sparse(a, "location", i, [(t0, c["loc"][i]), (t1, c["loc_end"][i])], linear=True)
        _assign(cam, a)
        tgt = _empty(f"U_CamTarget_Mechanisms_{key}", col, None, c["target"], 0.06, "SPHERE")
        _track_to(cam, tgt)
        _dof(cam, tgt, c["fstop"])
        out["cameras"].append(cam.name)
    stand = _build_stand(stand_col)
    out["stand"] = stand.name
    out["plan"] = mp
    return out


def build(scene: bpy.types.Scene | None = None, *, select: str = "showcase") -> dict:
    """İki klibi (aksiyonlar, kameralar, düzenekler, sehpa) kurar ve ``select`` klibini etkinleştirir.
    Önkoşul: ``airframe.build()``, ``gear.build()``, ``rig.setup()``. Tekrar çağrılabilir."""
    scene = scene or bpy.context.scene
    col, stand_col = _collections(scene)
    root = bpy.data.objects[ROOT]
    scene.render.fps = FPS
    scene.render.fps_base = 1.0
    for name in _OLD_OBJECTS:
        U.remove_object(name)
    for name in _OLD_ACTIONS:
        if bpy.data.actions.get(name) is not None:
            bpy.data.actions.remove(bpy.data.actions[name])
    sc = _build_showcase(col, root)
    me = _build_mechanisms(col, stand_col, root)
    sp = sc["plan"]
    out: dict = {"actions": sc["actions"] + me["actions"], "cameras": sc["cameras"] + me["cameras"],
                 "stand": me["stand"]}
    ra = bpy.data.actions[f"YK38_showcase_Root"]
    out["showcase"] = {"t_lof_s": round(sp["t_lof"], 3), "x_lof_m": round(sp["x_lof"], 2),
                       "a_takeoff_ms2": round(sp["a_max"], 3), "frames": sp["n"],
                       "root_keys": sum(len(f.keyframe_points) for f in ra.fcurves), "wheel": sc["wheel"],
                       "extent_m": [np.round(sp["loc"].min(0), 1).tolist(), np.round(sp["loc"].max(0), 1).tolist()]}
    out["mechanisms"] = {"frames": me["plan"]["n"]}
    out["normalized"] = len(RIG.normalize_parenting())     # rig'den sonra kurulan çıkartmalar (materials) da
    set_scene_range(select, scene)
    return out


def _plan_markers(name: str) -> tuple[list, list]:
    if name == "showcase":
        p = showcase_plan()
    else:
        p = mechanisms_plan()
    return p["markers"], p["events"]


def set_scene_range(name: str, scene: bpy.types.Scene | None = None) -> dict:
    """Klibi etkinleştirir: ``U_Root``/``U_Prop`` aksiyonları, kare aralığı (1…N), kamera ve olay işaretleri, etkin
    kamera, sehpa görünürlüğü. ``name`` ∈ {"showcase", "mechanisms"}."""
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
    markers, events = _plan_markers(name)
    first = None
    for f, cam_name in markers:
        cam = bpy.data.objects.get(cam_name)
        mk = scene.timeline_markers.new(f"{MARK}{cam_name[6:]}", frame=f)
        mk.camera = cam
        first = first or cam
    for f, ev in events:                                  # kamerasız: kamera geçişinde yok sayılır
        scene.timeline_markers.new(f"{MARK}{ev}", frame=int(f))
    scene.camera = first
    for ob in bpy.data.objects:
        if ob.name.startswith("U_Stand_"):
            ob.hide_render = ob.hide_viewport = name != "mechanisms"
    scene.frame_set(1)
    RIG.refresh()
    return {"clip": name, "frames": n, "camera": first.name if first else None}


def clear(scene: bpy.types.Scene | None = None) -> None:
    """Animasyonu kaldırır: ``U_Root`` dinlenme konumuna, özellikler varsayılana (``prop_auto`` = 1,
    ``wheel_auto`` = 1, ``ground_z`` = GROUND_Z), ``U_Prop["ucav_turns"]`` = 0, işaretler silinir."""
    scene = scene or bpy.context.scene
    root = bpy.data.objects[ROOT]
    _assign(root, None)
    prop = bpy.data.objects.get("U_Prop")
    if prop is not None:
        _assign(prop, None)
        prop[RIG.PROP_TURNS] = 0.0
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
