"""DC7 parametrik CAD modelleri için ortak ölçüler (mm, g, derece).

Kaynaklar: config/hardware/tier-a-ekonomik.yaml (kütleler), docs/05 §3 (koruma ve avuç tutamağı
kuralları), docs/10 §3 (gimbal). "tahmini" ölçüler gerçek parçalar elde edilince kumpasla
ölçülüp güncellenmelidir; tüm parçalar bu dosyadaki değerlerden yeniden üretilir.

Koordinatlar: x ileri, y sol, z yukarı; orijin gövde merkezi, kol üst yüzeyi (motor oturma düzlemi).
"""
from __future__ import annotations

import math

# --- Gövde, motor, pervane --------------------------------------------------------------
MOTOR_DIAG = 336.0                       # GEPRC MOZ7 V2 motorlar arası çapraz mesafe
MOTOR_R = MOTOR_DIAG / 2                 # merkezden motora 168 mm
MOTOR_XY = MOTOR_R / math.sqrt(2)        # 118,8 mm → PX4 CA_ROTORn_PX/PY = ±0,119 m
ARM_T = 6.0                              # karbon kol kalınlığı
FRAME_BOTTOM_Z = -8.0                    # alt plakanın alt yüzeyi (tahmini)
FRAME_TOP_Z = 22.0                       # üst plakanın üst yüzeyi, yığın yüksekliği (tahmini)
PROP_D = 7.0 * 25.4                      # 177,8 mm
PROP_R = PROP_D / 2
HUB_T = 3.0                              # koruma bağlantı plakası (motor ile kol arasında)
PROP_ABOVE_MOTOR_BASE = 27.0             # motor tabanı → pervane düzlemi (2807, tahmini)
PROP_PLANE_Z = HUB_T + PROP_ABOVE_MOTOR_BASE
MOTOR_BELL_D = 34.0                      # 2807 çan çapı (tahmini)
MOTOR_HOLE_SPACING = 16.0                # kare M3 deseni; motora göre 16 veya 19 (doğrulayın)
M3, M2_5, M2 = 3.3, 2.8, 2.3             # vida geçiş delikleri
M2_PILOT = 1.8                           # M2 kendinden kılavuzlu vida için pilot delik

# --- Pervane koruması (docs/05 §3.2) ----------------------------------------------------
GUARD_CLEARANCE = 6.0                    # pervane ucu ↔ halka iç yüzeyi
GUARD_WALL = 1.2                         # PA-CF; üst kenarda dışa doğru boncuk (GUARD_LIP)
GUARD_LIP = 1.2
GUARD_ABOVE = 5.0                        # halka, pervane düzleminin bu kadar üstüne çıkar
GUARD_BELOW = 8.0                        # ağ, pervane düzleminin bu kadar altında (kanat ≥ 6 mm uzakta)
MESH_OPENING = 10.0                      # ağ gözü ≤ 10 mm → parmak geçmez
MESH_RIB = 1.0
MESH_T = 1.0
MESH_SECTOR_DEG = 360.0                  # 180 → yalnızca gövdeye bakan iç yarım ağ (hafif, daha az itki kaybı)
SPOKES = 4                               # ağ göbeği → halka takviye kolları
SPOKE_W = 2.0
SPOKE_T = 2.5                            # göbek ve kol kalınlığı
COLLAR_GAP = 3.0                         # motor çanı ↔ ağ göbeği boşluğu
COLLAR_W = 6.0
POSTS = 4                                # bağlantı plakası → ağ göbeği direkleri
POST_D = 5.0
LUG_W = 7.0                              # bağlantı plakası kolları (X biçimi)

# --- Avuç tutamağı (docs/05 §3.1) -------------------------------------------------------
GRIP_D = 70.0                            # Ø 65–75 mm
GRIP_WALL = 1.3                          # 3 çevre çizgisi (0,4 mm nozul)
GRIP_MIN_DROP = 120.0                    # taban, pervane düzleminin en az bu kadar altında
GRIP_BOTTOM_Z = PROP_PLANE_Z - 125.0     # −95 mm
SENSOR_RECESS = 25.0                     # kamera + ToF tabandan 25 mm içeride
BUMPER_H = 6.0                           # TPU tampon (köpük ped bunun içine yapışır)
BUMPER_WALL = 2.5                        # tampon duvarı → ağız yarıçapı = GRIP_D/2 − BUMPER_WALL
GROUND_FOOT_D = 150.0                    # yerden kalkış için sökülebilir geniş ayak (öneri)
STACK = 30.5                             # uçuş yığını M3 deseni (MOZ7: 30,5 / 25,5 / 20)
GRIP_WINDOW = 16.0                       # baklava hafifletme pencereleri (45° → desteksiz basılır)

# --- Kameralar ve sensörler -------------------------------------------------------------
CAM_BOARD = (25.0, 24.0, 1.0)            # Pi Camera Module 3 kartı (g × y × kalınlık)
CAM_HOLES = (21.0, 12.5)                 # M2 delik aralığı, Pi kamera deseni (kartla doğrulayın)
CAM_HOLE_OFFSET = 2.0                    # delik merkezi kartın üst kenarından (tahmini)
CAM_LENS = (8.5, 8.5, 6.5)               # mercek bloğu (tahmini)
CAM_MASS_G = 4.0
CAM_HFOV, CAM_VFOV = 66.0, 41.0          # Camera Module 3
WIDE_HFOV, WIDE_VFOV = 102.0, 67.0       # Camera Module 3 Wide
TOF_WINDOW = (9.0, 5.0)                  # VL53L8CX verici/alıcı penceresi (paylı)
FFC_SLOT = (18.0, 2.5)                   # 15 pinli kamera kablosu geçişi
FLOW_FOV = 42.0                          # MTF-01 akış görüş açısı
FLOW_X = -75.0                           # akış sensörü x konumu (tutamak görüşe girmesin)

# --- Gimbal (docs/10 §3) ----------------------------------------------------------------
GIMBAL_MOTOR_D = 27.0                    # 22xx fırçasız gimbal motoru
GIMBAL_MOTOR_H = 14.0
GIMBAL_MOTOR_MASS_G = 20.0
GIMBAL_MOTOR_HOLE_CIRCLE = 16.0          # 4 × M2, motora göre 12 / 16 / 19 mm
GIMBAL_PART_T = 3.0                      # baskı kol kalınlığı (≥ 3 mm, docs/10 §3.3)
GIMBAL_CLEARANCE = 4.0
CRADLE_SLOT = 5.0                        # kızak: kamera ± bu kadar kaydırılarak dengelenir
PITCH_RANGE = (-90.0, 30.0)
ROLL_RANGE = (-30.0, 30.0)
GIMBAL_POS = (115.0, 0.0, -56.0)         # pitch ekseni gövde koordinatında (cad/layout.py, gimbal_2axis.stack_fits)
PITCH_SOFT_MAX = 15.0                    # yazılım üst sınırı: üstünde ön korumalar kadraja girer
DAMPER_SPACING = (30.0, 30.0)            # 4 sönümleyici deseni
DAMPER_H = 8.0                           # sönümleyici yüksekliği
BOOM_X = (50.0, 125.0)                   # taşıyıcı kol başlangıcı (gövde alt plakası ön delikleri; uç x hesaplanır)

# --- Üst katlar -------------------------------------------------------------------------
PI5_BOARD = (85.0, 56.0)
PI5_HOLES = (58.0, 49.0)                 # M2.5; desen merkezi kart merkezinden 10 mm port olmayan uca
PI5_HOLE_OFFSET = 10.0
COMPANION_STACK_H = 32.0                 # Pi 5 + Active Cooler + AI HAT+ yüksekliği (tahmini)
TRAY = (120.0, 70.0, 2.0)                # companion tepsisi (PA-CF)
TRAY_Z = FRAME_TOP_Z + 6.0               # tepsi, yığın üstünde 6 mm ara parça ile
PI5_X = 15.0                             # Pi 5 kart merkezi; USB/Ethernet arkaya bakar
BATTERY_PLATE = (120.0, 70.0, 2.0)
BATTERY_PLATE_Z = TRAY_Z + TRAY[2] + COMPANION_STACK_H + 4.0
BATTERY_SIZE = (75.0, 46.0, 66.0)        # 6S1P 21700, 2×3 dizilim (tahmini dış ölçü)
STRAP_SLOT = (22.0, 3.0)                 # 20 mm batarya kayışı
STRAP_SPACING = 30.0                     # iki kayış arası (batarya merkezine göre ±15)
TRAY_POSTS = (100.0, 52.0)               # tepsi ↔ batarya plakası M3 ara parça deseni
GNSS_POS = (-75.0, 0.0, 140.0)           # arka direk (gövde üst plakası arka ucundan; Pi'den uzak)
OVERHEAD_TOF_POS = (50.0, 0.0)           # batarya plakası ön ucu (x, y)
LIGHTEN_HOLE = 11.0                      # plakalarda hafifletme deliği çapı
LIGHTEN_PITCH = 15.0

# --- Malzemeler (g/cm³, yaklaşık; ince duvarlı parçalar ≈ %100 dolu basılır) --------------
DENSITY = {"PETG": 1.27, "PA-CF": 1.10, "TPU": 1.21}


def guard_inner_r() -> float:
    return PROP_R + GUARD_CLEARANCE


def guard_outer_r() -> float:
    return guard_inner_r() + GUARD_WALL


def motor_positions() -> list[tuple[float, float]]:
    """Gerçek-X: ön sol, ön sağ, arka sağ, arka sol (x ileri, y sol)."""
    m = MOTOR_XY
    return [(m, m), (m, -m), (-m, -m), (-m, m)]
