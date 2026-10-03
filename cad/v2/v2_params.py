"""DC7 v2 — bütünleşik (DJI tarzı) gövde: ortak ölçüler (mm, g). CadQuery gerektirmez.

Mimari: kalıplanabilir gövde kabukları (üst kabuk + taşıyıcı alt kabuk + burun), 4 özdeş kol–kanal
modülü (tek kalıp), arkadan takılan batarya (kuyruk kapağı gövdenin parçası), burun bölmesinde
gimbal kamera, gövde altında avuç ayağı. Koordinatlar v1 ile aynıdır (cad/params.py): x ileri,
y sol, z yukarı; z = 0 motor oturma düzlemi, pervane düzlemi z = 30.
"""
from __future__ import annotations

import math
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import params as P  # noqa: E402

# --- Gövde dış yüzeyi: x istasyonlarında "omuzlu" kesitler --------------------------------------
# Alt kısım geniş "şasi" (batarya, Pi, ESC; kanal yüksekliğinin altında), üst kısım dar "kanopi"
# (kanallar arası koridora sığar). Kesit = köşeleri yuvarlatılmış 8 köşeli çokgen → yoğun noktalar
# → kapalı spline → loft (pürüzsüz yüzey).
# (x, z0 alt, zs omuz başı, rise omuz yüksekliği, z1 üst, Wb alt genişlik, Wt üst genişlik,
#  Rb alt köşe, Rs omuz, Rt üst köşe yarıçapı)
STATIONS = (
    (-92.0, -38.0, 12.0, 4.0, 20.0, 80.0, 34.0, 10.0, 5.0, 10.0),     # kuyruk = batarya kapağı yüzü
    (-62.0, -40.0, 14.0, 6.0, 38.0, 88.0, 50.0, 10.0, 6.0, 18.0),
    (-30.0, -44.0, 14.0, 6.0, 46.0, 96.0, 62.0, 10.0, 6.0, 22.0),
    (0.0, -46.0, 14.0, 6.0, 48.0, 98.0, 66.0, 10.0, 6.0, 24.0),
    (35.0, -46.0, 13.0, 6.0, 46.0, 98.0, 62.0, 10.0, 6.0, 22.0),
    (64.0, -52.0, 11.0, 6.0, 38.0, 98.0, 50.0, 12.0, 6.0, 18.0),
    (82.0, -40.0, 8.5, 5.5, 28.0, 92.0, 43.0, 12.0, 5.5, 14.0),      # alt şasi öne doğru incelir (arka perde küçük)
    (100.0, -22.0, 6.0, 5.0, 18.0, 96.0, 36.0, 8.0, 5.0, 10.0),      # burun başlığı (z < −12 bölme için kesilir)
    (121.0, -20.0, 2.0, 4.5, 13.0, 89.0, 33.0, 6.0, 4.5, 7.5),
    (142.0, -18.0, -2.0, 4.0, 8.0, 82.0, 30.0, 5.0, 4.0, 5.0),
)
WALL = 2.0                     # kabuk et kalınlığı (PC/ABS enjeksiyon: 1,5–2,5 mm)
Z_SPLIT = -4.0                 # üst kabuk / alt kabuk ayrım düzlemi
NOSE_SPLIT_X = 80.0            # burun kapağı ayrımı (gimbal servisi, farklı renk/malzeme)
DRAFT_DEG = 1.5                # kalıptan çıkma açısı (dik duvarlar)
RIB_RATIO = 0.6                # kaburga kalınlığı ≤ 0,6 × et (çöküntü izi olmasın)
ARC_STEPS = 6                  # köşe yayı örnek sayısı

# --- Burun (gimbal) bölmesi -------------------------------------------------------------------
BAY = {"x0": 79.0, "x1": 160.0, "half_y": 60.0, "z0": -100.0, "z1": -12.0}   # yanaksız: gimbal başlığın altında açıkta
GIMBAL_MOUNT_Z = BAY["z1"]                       # sönümleyiciler bölme tavanına bağlanır
BAY_WALL = 1.6                                   # bölme tavanı ve arka perde (burun kapağının parçası, siyah)
GIMBAL_STACK_ABOVE = 36.8                        # kamera merkezi → üst plaka üstü (cad/gimbal_2axis)
# Kamera merkezi: gimbal üst parçasının arka dikey plakası arka perdeden (x = NOSE_SPLIT_X + BAY_WALL) ≥ 3 mm önde
GIMBAL_POS = (128.0, 0.0, GIMBAL_MOUNT_Z - P.DAMPER_H - GIMBAL_STACK_ABOVE)

# --- Batarya (6S1P 21700, hücreler x yönünde, 3 × 2 dizilim) --------------------------------
CELL_D, CELL_L = 21.7, 70.2
PACK_WALL = 1.2                                  # paket kabuğu (hücreler + BMS içte)
CAP_WALL = 1.5
PACK = (CELL_L + 2 * 4.0, 3 * CELL_D + 2 * PACK_WALL, 2 * CELL_D + 2 * PACK_WALL)   # x, y, z (≈ 78 × 68 × 47)
PACK_Z = (-34.0, -34.0 + PACK[2])                # batarya kanal yüksekliğinin altında
TAIL_CAP = 14.0                                  # batarya kapağı (gövde kuyruğunu oluşturur)
BATTERY_CLEAR = 0.6                              # batarya ↔ tünel boşluğu (yarıçapta)
GRILLE = {"cell": 10.0, "rib": 1.2, "t": 1.2}    # altıgen ızgara: hücre iç çapı ≤ 10 mm (parmak), kalıp için ≥ 1,2 mm kaburga
SPOKE = (2.0, 5.0)                               # ızgara altındaki 3 radyal kaburga (genişlik, yükseklik): halka desteği + kalıpta akış yolu
BATTERY_CAP_G = 18.0                             # kuyruk kapağı + kilit düğmeleri (CAD; kabuğun geri kalanı paket boyunca)
BATTERY_ELEC_G = 10.0                            # konnektör + BMS / yakıt göstergesi kartı + yaylar (tahmini; paketin ön ucunda)

# --- Kol–kanal modülü -------------------------------------------------------------------------
DUCT_IN_R = P.PROP_R + P.GUARD_CLEARANCE         # 94,9
DUCT_WALL = 1.6                                  # PA6-GF30 akış boyu ≈ 300 mm için alt sınır
DUCT_Z = (P.PROP_PLANE_Z - P.GUARD_BELOW, P.PROP_PLANE_Z + P.GUARD_ABOVE)   # 20 … 35
DUCT_FLARE = (4.0, 3.0)                          # çan ağzı (yükseklik, dışa açılma)
ARM_ROOT_R = 66.0                                # kolun gövde yüzeyinden çıktığı yarıçap (köşegen)
ARM_INSERT = 14.0                                # soket derinliği: kol kökü gövdeye bu kadar girer
ARM_PRISM = 18.0                                 # gövde yüzeyinden sonra da sabit kesit (yan duvar kolu 45° keser)
ARM_FIT = 0.3                                    # kol kökü ↔ soket boşluğu (her yüzde)
SLEEVE_WALL = 1.6                                # gövde içindeki soket kovanı eti
ARM_ROOT = (22.0, 34.0)                          # kök genişlik, yükseklik (U kesit) — titreşim: cad/v2/analysis_v2.py
ARM_TIP = (16.0, 17.0)                           # motor ucunda
ARM_WALL = 2.2
ARM_TOP_TIP_Z = 0.0                              # motor yuvası üstü (motor tabanı P.HUB_T'de)
MOTOR_POD = (20.0, 17.0)                         # motor yuvası yarıçap, yükseklik (alt yüzü kolla aynı hizada)
# Üst kabuk ↔ alt kabuk vida kuleleri (x, y; ±y simetrik): kol kökleri, batarya tüneli ve kartlardan uzakta.
# Kule tabandan ayrım düzlemine (alt kabuk, Ø2,8 geçiş), oradan omuz iç yüzeyine (üst kabuk, Ø2,2 pilot) uzanır.
BOSSES = ((0.0, 43.0), (72.0, 30.0), (-68.0, 40.0))
BOSS_D = 6.4

# --- Avuç ayağı -------------------------------------------------------------------------------
POD_TOP_R, POD_BOTTOM_R = 37.0, 33.0             # aşağı doğru hafif incelen
POD_X = 0.0

# --- CAD ağırlık merkezleri (cad/v2/build_v2.py; tests/test_v2.py ±3 mm ile doğrular), mm ----------
PART_CG = {"top_shell": (0.4, 0.0, 25.8), "bottom_tub": (2.4, 0.0, -31.6), "nose_cover": (107.5, 0.0, -4.7),
           "pod": (0.0, 0.0, -67.6)}
ARM_DUCT_CG = (-24.0, 12.0)                      # kol–kanal modülü: motor ekseninden kol yönünde (yerel x), z

# --- Malzemeler (g/cm³) -----------------------------------------------------------------------
DENSITY = {"PC/ABS": 1.15, "PA6-GF30": 1.36, "PA12 (MJF)": 1.01, "TPU": 1.21, "PC": 1.20, "POM": 1.41}
PROD_MATERIAL = {"shell": "PC/ABS", "arm": "PA6-GF30", "pod": "PC/ABS", "battery": "PC/ABS", "tpu": "TPU"}
E_MOLDED = {"PA6-GF30": 7.5e9, "PC/ABS": 2.4e9, "PA12 (MJF)": 1.7e9}      # Pa (kuru, oda sıcaklığı)


def section_at(x: float, inset: float = 0.0) -> tuple:
    """İstasyonlar arasında doğrusal ara değer; inset > 0 iç yüzey (et kalınlığı) için küçültür."""
    st = STATIONS
    if x <= st[0][0]:
        vals = st[0][1:]
    elif x >= st[-1][0]:
        vals = st[-1][1:]
    else:
        for a, b in zip(st, st[1:]):
            if a[0] <= x <= b[0]:
                t = (x - a[0]) / (b[0] - a[0])
                vals = tuple(a[i] + t * (b[i] - a[i]) for i in range(1, 10))
                break
    z0, zs, rise, z1, wb, wt, rb, rs, rt = vals
    if inset:
        z0, z1, wb, wt = z0 + inset, z1 - inset, wb - 2 * inset, wt - 2 * inset
        rb, rs, rt = max(rb - inset, 0.5), max(rs - inset * 0.5, 0.5), max(rt - inset, 0.5)
    return z0, zs, rise, z1, wb, wt, rb, rs, rt


# Kesit örneklemesi: her köşe yayı ARC_STEPS + 1 nokta, her kenar EDGE_STEPS kadar ara nokta (tüm kesitlerde
# aynı) → k. nokta her kesitte aynı özelliğe (ör. omuz yayının ortası) düşer, loft yüzeyi kıvrılmaz
# (parlak yüzeyde dalgalı yansıma olmaz). Kenar sırası: sağ yan, sağ omuz, sağ kanopi, üst, sol kanopi,
# sol omuz, sol yan, alt (alt kenarın orta noktası örnek noktasıdır → kesit oradan başlar).
EDGE_STEPS = (6, 2, 4, 7, 4, 2, 6, 9)
SECTION_POINTS = 8 * (ARC_STEPS + 1) + sum(EDGE_STEPS)


def _fillet_polygon(vertices: list[tuple[float, float]], radii: list[float],
                    edge_steps: tuple[int, ...] = EDGE_STEPS) -> list[tuple[float, float]]:
    """Kapalı çokgenin köşelerini yay ile yuvarlatır; sabit sayıda nokta döndürür (CCW).
    Sıra: 0. köşenin yayı, 0. kenarın ara noktaları, 1. köşenin yayı, … (kenar i: köşe i → i+1)."""
    n = len(vertices)
    arcs = []
    for i in range(n):
        a, b, c = vertices[i - 1], vertices[i], vertices[(i + 1) % n]
        u1 = (a[0] - b[0], a[1] - b[1])
        u2 = (c[0] - b[0], c[1] - b[1])
        l1, l2 = math.hypot(*u1), math.hypot(*u2)
        u1 = (u1[0] / l1, u1[1] / l1)
        u2 = (u2[0] / l2, u2[1] / l2)
        cos_t = max(-1.0, min(1.0, u1[0] * u2[0] + u1[1] * u2[1]))
        theta = math.acos(cos_t) / 2
        if radii[i] <= 0 or theta < 1e-3 or abs(theta - math.pi / 2) < 1e-3:
            raise ValueError(f"köşe {i}: yuvarlatılamaz (yarıçap {radii[i]}, açı {math.degrees(2 * theta):.1f}°)")
        d = min(radii[i] / math.tan(theta), 0.45 * l1, 0.45 * l2)
        r = d * math.tan(theta)
        t1 = (b[0] + u1[0] * d, b[1] + u1[1] * d)
        t2 = (b[0] + u2[0] * d, b[1] + u2[1] * d)
        bis = (u1[0] + u2[0], u1[1] + u2[1])
        bl = math.hypot(*bis)
        center = (b[0] + bis[0] / bl * r / math.sin(theta), b[1] + bis[1] / bl * r / math.sin(theta))
        a1 = math.atan2(t1[1] - center[1], t1[0] - center[0])
        a2 = math.atan2(t2[1] - center[1], t2[0] - center[0])
        da = (a2 - a1 + math.pi) % (2 * math.pi) - math.pi
        arcs.append([(center[0] + r * math.cos(a1 + da * k / ARC_STEPS),
                      center[1] + r * math.sin(a1 + da * k / ARC_STEPS)) for k in range(ARC_STEPS + 1)])
    pts: list[tuple[float, float]] = []
    for i in range(n):
        pts.extend(arcs[i])
        last, nxt = arcs[i][-1], arcs[(i + 1) % n][0]
        k = edge_steps[i]
        for j in range(1, k + 1):
            t = j / (k + 1)
            pts.append((last[0] + (nxt[0] - last[0]) * t, last[1] + (nxt[1] - last[1]) * t))
    return pts


def section_points(x: float, inset: float = 0.0) -> list[tuple[float, float]]:
    """Kesit noktaları (y, z): alt ortadan (0, z0) başlar, saat yönünün tersine, SECTION_POINTS nokta.
    Tüm kesitlerde aynı sayı ve özellik hizası; k. ve (n−k). noktalar ayna görüntüsüdür → loft bükülmez
    ve gövde y = 0'a göre simetrik kalır."""
    z0, zs, rise, z1, wb, wt, rb, rs, rt = section_at(x, inset)
    verts = [(wb / 2, z0), (wb / 2, zs), (wt / 2, zs + rise), (wt / 2, z1),
             (-wt / 2, z1), (-wt / 2, zs + rise), (-wb / 2, zs), (-wb / 2, z0)]
    radii = [rb, rs, rs, rt, rt, rs, rs, rb]
    pts = _fillet_polygon(verts, radii)
    mid = len(pts) - (EDGE_STEPS[-1] + 1) // 2           # alt kenarın orta ara noktası (y = 0)
    pts[mid] = (0.0, z0)                                  # sayısal sıfır
    return pts[mid:] + pts[:mid]


def half_width(x: float, z: float, inset: float = 0.0) -> float:
    """Gövde yüzeyinin z yüksekliğindeki yarı genişliği (sağ taraf); gövde dışındaysa −1."""
    pts = section_points(x, inset)
    best = -1.0
    for (y0, z0), (y1, z1) in zip(pts, pts[1:] + pts[:1]):
        if (z0 - z) * (z1 - z) <= 0 and z0 != z1:
            y = y0 + (y1 - y0) * (z - z0) / (z1 - z0)
            best = max(best, y)
    return best


def body_x_range() -> tuple[float, float]:
    return STATIONS[0][0], STATIONS[-1][0]
