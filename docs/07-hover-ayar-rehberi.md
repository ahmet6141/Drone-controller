# 07 — Profesyonel Hover Ayar Rehberi (PX4 v1.17)

"Profesyonel hover" = rüzgârda, GNSS'li ve GNSS'siz ortamda **titreşimsiz, kaymasız, tekrar
edilebilir** pozisyon tutma (hedefler: [01 §2.1](01-gereksinimler.md)). Parametre dosyaları:
[`config/px4/`](../config/px4/); tüm adlar v1.17 referansıyla doğrulanır. ArduPilot karşılığı §11.

## 1. Hover kalitesini belirleyen zincir

| Halka | Etkisi | Bu projedeki önlem |
|---|---|---|
| Mekanik | Titreşim → IMU gürültüsü, kestirim hatası | Balanslı pervane, sağlam gövde, kablo bağlama, FC yumuşak montaj |
| IMU | Sıcaklık kayması, gürültü | Endüstriyel IMU + ısıtıcı (ARK FPV: IIM-42653, 1 W), `SENS_EN_THERMAL=1` |
| Motor geri bildirimi | Notch filtresi frekansı | Bidirectional DShot → ESC RPM tabanlı dinamik notch |
| Kestirim (EKF2) | Konum/hız doğruluğu | RTK GNSS + akış + lidar (dış); VIO + akış (iç) |
| Kontrol | Tepki, salınım, rüzgâr reddi | Otomatik ayar + hız döngüsü I kazancı |
| İtki | Doğrusallık, kontrol payı | `THR_MDL_FAC` ölçümü, hover gazı ≈ 0.20 (geniş pay) |

## 2. Mekanik hazırlık
- Pervaneleri balanslayın; motor vidaları ve kol cıvataları torklu (vida sabitleyici).
- FC'yi titreşim sönümleyici ile monte edin; kablolar FC'ye gerilim aktarmamalı.
- GNSS/pusula **direkte**, güç kablolarından, ESC'den ve **Jetson'dan uzak** (USB3/HDMI gürültüsü
  GNSS sinyal/gürültü oranını düşürür).
- Barometreyi pervane akışından köpükle koruyun.
- Akış sensörü ve lidar aşağıya engelsiz bakmalı; avuç tutamağı görüş alanına girmemeli.

## 3. Sensör kurulumu ve kalibrasyon
1. İvmeölçer, jiroskop, pusula, RC, ESC kalibrasyonları (QGC).
2. **Pusula akım kompanzasyonu**: `CAL_MAG_COMP_TYP` ile akım tabanlı kompanzasyon (PX4
   "compass power compensation" prosedürü).
3. IMU sıcaklık kalibrasyonu (ısıtıcı açıkken bile önerilir).
4. Montaj ofsetlerini ağırlık merkezine göre girin: `EKF2_IMU_POS_*`, `EKF2_GPS_POS_*`,
   `EKF2_OF_POS_*`, `EKF2_RNG_POS_*`, `EKF2_EV_POS_*`.
5. Akış: `SENS_FLOW_ROT` (montaj yönü), `SENS_FLOW_MINHGT`/`SENS_FLOW_MAXHGT`; gerekirse
   ölçek kalibrasyonu (`SENS_FLOW_SCALE`, v1.16+).

## 4. Filtre ayarı (en kritik adım)
1. Bidirectional DShot: `DSHOT_BIDIR_EN=1`, `MOT_POLE_COUNT=14` (12N14P motorlar). STM32H7
   kartlarda yalnızca ilk 4 FMU çıkışı, aynı timer üzerinde.
2. RPM tabanlı dinamik notch: `IMU_GYRO_DNF_EN=1` (bit 0), `IMU_GYRO_DNF_HMC=3`,
   `IMU_GYRO_DNF_BW=15`, `IMU_GYRO_DNF_MIN=25`.
3. İlk uçuşlar varsayılan alçak geçiren filtrelerle: `IMU_GYRO_CUTOFF=40`, `IMU_DGYRO_CUTOFF=20`.
4. Hover + manevra kaydı → **Flight Review** aktüatör FFT ve titreşim grafikleri. Motor
   harmonikleri notch ile bastırılmışsa kesimler kademeli yükseltilir
   (`IMU_GYRO_CUTOFF` 60–80 Hz, `IMU_DGYRO_CUTOFF` 30 Hz) → daha az faz gecikmesi, daha sıkı hover.
5. İç döngü hızı: `IMU_GYRO_RATEMAX=800` (H7'de yeterli pay).

## 5. İç döngü (oran/tutum) — otomatik ayar
- `MC_AT_EN=1`, `MC_AT_APPLY=1` (kazançlar disarm sonrası uygulanır).
- Altitude modunda, sakin havada QGC'den başlatılır (~40 s, sistem tanılama).
- `MC_AIRMODE=0` ayar süresince. Batarya gerilim ölçekleme açık: `MC_BAT_SCALE_EN=1`.
- Doğrulama: Flight Review basamak yanıtı — aşım < %10, salınım yok.

## 6. İtki modeli ve hover itkisi
- `THR_MDL_FAC`: itki standı verisinden `tools/budget_calc.py` uydurması (F90 verisiyle ≈ 0.53;
  dosyada 0.5). Standda koruma ve ağ takılıyken ölçüp `config/hardware/*.yaml` →
  `throttle_points` güncellenir, değer yeniden hesaplanır.
- `MPC_THR_HOVER`: hesaplayıcı başlangıcı (Seviye B ≈ 0.20); `MPC_USE_HTE=1` uçuşta öğrenir.
  Hover gazının ≈ 0.20 olması rüzgâr ve manevra için geniş kontrol payı demektir.

## 7. Konum ve hız döngüsü
| Parametre | Değer | Amaç |
|---|---|---|
| `MPC_POS_MODE` | 4 | İvme tabanlı pilot girişi → yumuşak kamera hareketi |
| `MPC_XY_VEL_P_ACC` / `_I_ACC` / `_D_ACC` | 2.2 / 0.8 / 0.2 | Rüzgârda konum tutma (I ↑) |
| `MPC_Z_VEL_P_ACC` / `_I_ACC` | 4.0 / 2.0 | Dikey tutma |
| `MPC_JERK_AUTO` / `MPC_JERK_MAX` | 4 / 8 | Sarsıntısız otomatik hareket |
| `MPC_ALT_MODE` | 0 | Mutlak irtifa: altına el/kişi girince yükseklik sıçramaz |
| `MPC_TILTMAX_AIR` | 35° | Görüntü ve güvenlik için sınırlı eğim |

Ayar sırası: önce hız döngüsü (adım girişleri, salınım yoksa P'yi artır), sonra konum `MPC_XY_P`.
Rüzgârlı günde: konum hatası kalıcıysa `MPC_XY_VEL_I_ACC` artırılır.

## 8. EKF2 — ortama göre profil
| Ortam | Profil | Ana kaynaklar | Beklenen hover |
|---|---|---|---|
| Açık alan + RTK | `outdoor-gnss` | GNSS (RTK) yükseklik + konum, baro, akış, koşullu lidar | ±2–10 cm |
| Açık alan, standart GNSS | `outdoor-gnss` | GNSS, baro, akış (alçakta) | ±0,3–0,5 m |
| Kapalı alan | `indoor-vio` | cuVSLAM VIO (konum + yaw), akış, koşullu lidar | ±5–10 cm |

- Sağlık kontrolü: kayıtta inovasyon test oranları (`estimator_status`) < 0,5 tipik olmalı.
- VIO: 30–50 Hz, `EKF2_EV_DELAY` ölçülür (kamera pozlama → PX4 zaman damgası).
- GNSS: `EKF2_REQ_NSATS=8`, açık alanda `COM_ARM_WO_GPS=0` (GNSS'siz arm yok).

## 9. Avuca iniş için özel durumlar
- **El "zemin" sanılabilir**: koşullu lidar füzyonu (`EKF2_RNG_CTRL=1`, ≤ 5 m, ≤ 1 m/s) ve
  "terrain hold" altına giren eli yer olarak görür. Bu yüzden `MPC_ALT_MODE=0`, EKF yükseklik
  referansı GNSS/VIO/baro ve avuç mesafesi **özel modun kendi ToF ölçümüyle** kontrol edilir.
- PX4'ün yerleşik iniş hızları yeterince yavaş değil (`MPC_LAND_SPEED` alt sınırı 0,6 m/s);
  avuca iniş 0,20 → 0,08 m/s hız setpoint'leriyle özel modda yapılır.
- Temas sonrası zorlamalı disarm (`VEHICLE_CMD_COMPONENT_ARM_DISARM`, param2 = 21196);
  `COM_DISARM_LAND=0.5` yalnızca yedek.

## 10. Kabul uçuşları
| Test | Süre | Ölçüm (ulog) | Ölçüt |
|---|---|---|---|
| Sakin hover, RTK | 60 s × 10 | `vehicle_local_position` − setpoint | Yatay ≤ ±0,10 m, dikey ≤ ±0,05 m (2σ) |
| Sakin hover, standart GNSS | 60 s × 10 | aynı | ≤ ±0,5 m / ±0,2 m |
| GNSS'siz alçak hover (akış) | 5 dk | dış referans (zemin işaretleri) | drift ≤ 0,2 m/dk |
| Rüzgârda hover | 60 s × 5 | konum hatası, eğim, motor doygunluğu | 10 m/s'de tutma |
| Titreşim | her uçuş | Flight Review titreşim/FFT | "iyi" bölge |

## 11. ArduPilot 4.7.1 alternatifi (özet)
- Ayar: önce **QuikTune** (Lua, `RCx_OPTION=300`), sonra **AutoTune** (AltHold'dan).
- Notch: `INS_HNTCH_MODE=3` (ESC telemetrisi), `INS_HNTCH_OPTS=6`; analiz için
  `INS_LOG_BAT_MASK/OPT` + WebTools *Filter Review*.
- 4.7'de SI birimlerine geçiş: `ATC_INPUT_TC` 4.6'ya göre ~%33 düşük; `PSC_NE_*`, `PSC_D_*`,
  `LOIT_*_M(S)`, `LAND_SPD_MS` adları. Dosya: [`config/ardupilot/dc7-copter-4.7.param`](../config/ardupilot/dc7-copter-4.7.param).
- EKF3 kaynak setleri: GNSS / optik akış / Jetson VIO (`RC7_OPTION=90` ile geçiş).
