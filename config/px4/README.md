# PX4 v1.17 parametreleri

Tüm adlar, tipler, aralıklar ve enum değerleri resmi PX4 v1.17 parametre referansına göre
doğrulanır (`python3 tools/validate_params.py`; referans: `tools/data/px4_v1.17_params.json`).

## Yükleme sırası
1. QGroundControl → **Airframe**: *Generic Quadcopter* (`SYS_AUTOSTART=4001`) → yeniden başlat.
2. **Actuators** ekranı: çıkış protokolü **DShot600**, motor sırası ve yönleri (motor testi ile).
3. Parameters → Tools → *Load from file* — sırayla:
   1. `base/10-airframe-dc7.params` — geometri, DShot/RPM, filtreler, hover itkisi
   2. `base/20-position-control.params` — konum/hız kazançları, sınırlar, iniş
   3. `base/30-failsafe.params` — RC/link kaybı, batarya, geofence, RTL
   4. `base/40-companion-io.params` — uXRCE-DDS (TELEM2), MAVLink (TELEM1), DroneCAN akış
   5. **Bir** profil: `profiles/outdoor-gnss.params` **veya** `profiles/indoor-vio.params`
4. Kalibrasyonlar: ivmeölçer, pusula (+ akım tabanlı kompanzasyon `CAL_MAG_COMP_TYP`), ESC,
   RC; IMU sıcaklık kalibrasyonu önerilir.
5. Ölçülen montaj ofsetlerini girin: `EKF2_IMU_POS_*`, `EKF2_GPS_POS_*`, `EKF2_OF_POS_*`,
   `EKF2_RNG_POS_*`, `EKF2_EV_POS_*` (ağırlık merkezine göre, metre).

## Seviye farkları
| Konu | Seviye A | Seviye B (bu dosyalar) | Seviye C |
|---|---|---|---|
| FC | ARK FPV | ARK FPV | ARKV6X (PAB) — `EKF2_MULTI_IMU=3`, `SENS_IMU_MODE=0` ekleyin |
| Akış/lidar | MicoAir MTF-01 (UART, MAVLink) → `UAVCAN_SUB_FLOW/RNG` gerekmez; akış için bir MAVLink örneği yapılandırın | H-Flow (DroneCAN) | ARK Flow MR (DroneCAN) |
| GNSS | M10 (`EKF2_GPS_CTRL=7`) | F9P RTK | X20 RTK |
| İç mekân profili | VIO yok → `indoor-vio` yerine `EKF2_EV_CTRL=0`, `EKF2_HGT_REF=0` (baro) | `indoor-vio` | `indoor-vio` |
| Hover itkisi | `MPC_THR_HOVER` ≈ 0.18 | 0.20 | 0.19 |

## Sürüm notları (PX4 v1.18'e geçerken)
v1.18.0-rc1 (10.09.2026) kararlı değildir. v1.18'de değişecekler (kaynak: PX4 v1.18 sürüm notları):
- `DSHOT_BIDIR_EN`, `MOT_POLE_COUNT` → çıkış başına protokol ve `DSHOT_MOT_POL1..12`
- `COM_RC_OVERRIDE` / `COM_RC_STICK_OV` → `MAN_OVERRIDE_SPD`
- `EKF2_GPS_POS_*` / `EKF2_GPS_DELAY` → alıcı başına parametreler
- Yeni: `EKF2_SENS_EN`, Vision Target Estimator (`VTE_*`)

Geçişte `tools/data/` altına v1.18 referansı eklenip dosyalar yeniden doğrulanmalıdır.
