# DC7 CAD raporu (otomatik — `python3 cad/build.py`)

## Parçalar

| Parça | Adet | Malzeme | Hacim (cm³) | Kütle (g) | PETG ile (g) | Baskı yönü |
|---|---:|---|---:|---:|---:|---|
| guard_ring | 4 | PA-CF | 18.93 | 20.8 | 24.0 | ağ yüzeyi tablada; destek yok |
| guard_mount | 4 | PA-CF | 4.22 | 4.6 | 5.4 | plaka tablada; direkler yukarı |
| grip_tube | 1 | PETG | 22.62 | 28.7 | 28.7 | ters: flanş tablada; pencereler ve basamak 45° → destek yok |
| grip_sensor_mount | 1 | PETG | 7.33 | 9.3 | 9.3 | disk tablada |
| grip_bumper | 1 | TPU | 3.91 | 4.7 | 5.0 | TPU 95A; ağız tablada |
| companion_tray | 1 | PA-CF | 14.67 | 16.1 | 18.6 | düz |
| battery_plate | 1 | PA-CF | 14.05 | 15.5 | 17.8 | düz |
| companion_shell | 1 | PA-CF | 12.29 | 13.5 | 15.6 | dik bant; yarıklar dikey → destek yok (isteğe bağlı) |
| gimbal_cradle | 1 | PA-CF | 3.79 | 4.2 | 4.8 | arka plaka tablada; ara parçalar yukarı |
| gimbal_roll_arm | 1 | PA-CF | 5.76 | 6.3 | 7.3 | yan plaka tablada |
| gimbal_top | 1 | PA-CF | 8.38 | 9.2 | 10.6 | ters: üst plaka tablada |
| gimbal_boom | 1 | PA-CF | 10.82 | 11.9 | 13.7 | plaka tablada; kaburgalar yukarı |

## Bütçeyle karşılaştırma (config/hardware/tier-a-ekonomik.yaml)

| Bileşen | Bütçe (g) | Model (g) | Fark | Not |
|---|---:|---:|---:|---|
| Tam pervane koruması + alt ağ (PA12-CF, 3B baskı) | 110 | 101.6 | %-8 | 4 takım |
| Avuç iniş tutamağı (TPU + köpük, ≥120 mm) | 40 | 42.7 | %+7 | köpük ped hariç |
| Kendi tasarım 2 eksen fırçasız gimbal (2× motor, STorM32, IMU, baskı parçalar, sönümleyici) | 85 | 88.6 | %+4 | + 2 motor, STorM32, IMU, sönümleyiciler (docs/10 §3.2) |
| Üst plaka, bağlantılar, kablolama | 60 | 45.1 | %-25 | kabuk dahil; kalan pay kablolama ve bağlantılar için |

## Yerleşim

- Toplam kütle 1456 g; ağırlık merkezi (-0.0, +0.0, +39.0) mm
- Batarya konumu: x = -15.4 mm (ağırlık merkezini ortalar)
- Gimbal roll ekseni kamera merkezinden y = +21.4 mm (dönen grubun ağırlık merkezi)
- Devrilme açısı (motorlar kapalı): tutamakla 14.6°, Ø150 mm ayakla 29.2°
- Atalet (CG'ye göre, SITL başlangıcı): Ixx 0.0095, Iyy 0.0108, Izz 0.0129 kg·m²

## Kontroller

| Kontrol | Sonuç | Değer |
|---|---|---|
| pervane ↔ halka boşluğu | ✅ | 6.0 mm ≥ 6 |
| kanat ↔ ağ / göbek düşey boşluğu | ✅ | 2.5 mm ≥ 2 (temsili kanat kesitleri) |
| ağ gözü | ✅ | 10.0 mm ≤ 10 |
| tutamak tabanı pervane düzleminin altında | ✅ | 125 mm ≥ 120 |
| tutamak çapı | ✅ | Ø 70 mm (65–75) |
| komşu korumalar arası boşluk | ✅ | 39.4 mm ≥ 20 |
| tepsi köşesi ↔ koruma halkası | ✅ | 3.3 mm ≥ 3 |
| batarya plakaya sığıyor | ✅ | batarya x = -15.4 mm |
| GNSS direği ↔ batarya | ✅ | 22.1 mm ≥ 5 |
| CG yatay ofset | ✅ | (-0.0, +0.0) mm ≤ 5 |
| akış sensörü görüşü temiz | ✅ | temiz |
| yukarı ToF görüşü temiz | ✅ | temiz |
| gimbal yazılım aralığında görüş temiz | ✅ | temiz -90° … +15°, yazılım sınırı ≤ +15° |
| guard_ring: tek katı | ✅ | 1 katı |
| guard_mount: tek katı | ✅ | 1 katı |
| grip_tube: tek katı | ✅ | 1 katı |
| grip_sensor_mount: tek katı | ✅ | 1 katı |
| grip_bumper: tek katı | ✅ | 1 katı |
| companion_tray: tek katı | ✅ | 1 katı |
| battery_plate: tek katı | ✅ | 1 katı |
| companion_shell: tek katı | ✅ | 1 katı |
| gimbal_cradle: tek katı | ✅ | 1 katı |
| gimbal_roll_arm: tek katı | ✅ | 1 katı |
| gimbal_top: tek katı | ✅ | 1 katı |
| gimbal_boom: tek katı | ✅ | 1 katı |
| gimbal pitch −90…+30° çakışma | ✅ | 0.000 mm³ |
| gimbal roll ±30° çakışma | ✅ | 0.000 mm³ |
| gimbal dikey yığın gövdeye sığıyor | ✅ | pay 0.2 mm |
| pitch dengesi kızak aralığında | ✅ | kayma (-0.28, +0.02) mm ≤ 5 |
| ağın kapattığı pervane alanı | ✅ | %20 → installation_factor itki standında ölçülmeli |
