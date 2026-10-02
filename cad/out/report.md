# DC7 CAD raporu (otomatik — `python3 cad/build.py`)

## Parçalar

| Parça | Adet | Malzeme | Hacim (cm³) | Kütle (g) | PETG ile (g) | Baskı yönü |
|---|---:|---|---:|---:|---:|---|
| guard_ring | 4 | PA-CF | 17.97 | 19.8 | 22.8 | ağ yüzeyi tablada; destek yok |
| guard_mount | 4 | PA-CF | 4.15 | 4.6 | 5.3 | plaka tablada; direkler yukarı |
| grip_tube | 1 | PETG | 22.76 | 28.9 | 28.9 | ters: flanş tablada; pencereler ve basamak 45° → destek yok |
| grip_sensor_mount | 1 | PETG | 7.33 | 9.3 | 9.3 | disk tablada |
| grip_bumper | 1 | TPU | 4.05 | 4.9 | 5.1 | TPU 95A; ağız tablada |
| companion_tray | 1 | PA-CF | 14.67 | 16.1 | 18.6 | düz |
| battery_plate | 1 | PA-CF | 14.05 | 15.5 | 17.8 | düz |
| gimbal_cradle | 1 | PA-CF | 3.83 | 4.2 | 4.9 | arka plaka tablada; ara parçalar yukarı |
| gimbal_roll_arm | 1 | PA-CF | 5.81 | 6.4 | 7.4 | yan plaka tablada |
| gimbal_top | 1 | PA-CF | 8.29 | 9.1 | 10.5 | ters: üst plaka tablada |
| gimbal_boom | 1 | PA-CF | 8.36 | 9.2 | 10.6 | plaka tablada; kaburgalar yukarı |

## Bütçeyle karşılaştırma (config/hardware/tier-a-ekonomik.yaml)

| Bileşen | Bütçe (g) | Model (g) | Fark | Not |
|---|---:|---:|---:|---|
| Tam pervane koruması + alt ağ (PA12-CF, 3B baskı) | 110 | 97.6 | %-11 | 4 takım |
| Avuç iniş tutamağı (TPU + köpük, ≥120 mm) | 40 | 43.1 | %+8 | köpük ped hariç |
| Kendi tasarım 2 eksen fırçasız gimbal (2× motor, STorM32, IMU, baskı parçalar, sönümleyici) | 85 | 85.9 | %+1 | + 2 motor, STorM32, IMU, sönümleyiciler (docs/10 §3.2) |
| Üst plaka, bağlantılar, kablolama | 60 | 31.6 | %-47 | kalan pay kablolama ve bağlantılar için |

## Yerleşim

- Toplam kütle 1456 g; ağırlık merkezi (-0.0, +0.0, +39.2) mm
- Batarya konumu: x = -15.4 mm (ağırlık merkezini ortalar)
- Gimbal roll ekseni kamera merkezinden y = +21.4 mm (dönen grubun ağırlık merkezi)
- Devrilme açısı (motorlar kapalı): tutamakla 14.6°, Ø150 mm ayakla 29.2°

## Kontroller

| Kontrol | Sonuç | Değer |
|---|---|---|
| pervane ↔ halka boşluğu | ✅ | 6.0 mm ≥ 6 |
| ağ gözü | ✅ | 10.0 mm ≤ 10 |
| tutamak tabanı pervane düzleminin altında | ✅ | 125 mm ≥ 120 |
| tutamak çapı | ✅ | Ø 70 mm (65–75) |
| komşu korumalar arası boşluk | ✅ | 45.4 mm ≥ 20 |
| tepsi köşesi ↔ koruma halkası | ✅ | 6.3 mm ≥ 3 |
| batarya plakaya sığıyor | ✅ | batarya x = -15.4 mm |
| GNSS direği ↔ batarya | ✅ | 22.1 mm ≥ 5 |
| CG yatay ofset | ✅ | (-0.0, +0.0) mm ≤ 5 |
| akış sensörü görüşü temiz | ✅ | temiz |
| yukarı ToF görüşü temiz | ✅ | temiz |
| gimbal yazılım aralığında görüş temiz | ✅ | temiz -90° … +20°, yazılım sınırı ≤ +15° |
| guard_ring: tek katı | ✅ | 1 katı |
| guard_mount: tek katı | ✅ | 1 katı |
| grip_tube: tek katı | ✅ | 1 katı |
| grip_sensor_mount: tek katı | ✅ | 1 katı |
| grip_bumper: tek katı | ✅ | 1 katı |
| companion_tray: tek katı | ✅ | 1 katı |
| battery_plate: tek katı | ✅ | 1 katı |
| gimbal_cradle: tek katı | ✅ | 1 katı |
| gimbal_roll_arm: tek katı | ✅ | 1 katı |
| gimbal_top: tek katı | ✅ | 1 katı |
| gimbal_boom: tek katı | ✅ | 1 katı |
| gimbal pitch −90…+30° çakışma | ✅ | 0.000 mm³ |
| gimbal roll ±30° çakışma | ✅ | 0.000 mm³ |
| gimbal dikey yığın gövdeye sığıyor | ✅ | pay 0.2 mm |
| pitch dengesi kızak aralığında | ✅ | kayma (-0.26, +0.02) mm ≤ 5 |
| ağın kapattığı pervane alanı | ✅ | %20 → installation_factor itki standında ölçülmeli |
