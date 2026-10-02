# 02 — Donanım Analizi ve Seçim

Durum tarihi: **2 Ekim 2026**. Fiyatlar yaklaşık (USD) ve değişkendir; NVIDIA Temmuz 2026'da
Jetson fiyatlarını %101'e varan oranda artırdı. Seviyelerin tam bileşen listeleri ve kütle/güç
değerleri [`config/hardware/`](../config/hardware/) altındadır; bütçe tablosu
`tools/budget_calc.py` ile bu dosyalardan üretilir.

## 1. Özet: üç donanım seviyesi

| Bileşen | Seviye A — Geliştirme | **Seviye B — Önerilen Pro** | Seviye C — Üst Seviye |
|---|---|---|---|
| Gövde | GEPRC MOZ7 V2 (336 mm) + tam koruma/ağ + avuç tutamağı | ← aynı | ← aynı |
| Motor / pervane | EMAX ECO II 2807 1300KV / HQ 7×4×3 | T-Motor F90 2806.5 1300KV / HQ 7×4×3 | ← B |
| ESC | Tekko32 F4 50A (AM32) | Tekko32 F4 Metal 65A (AM32) | ARK 4IN1 50/75 A |
| Batarya | 6S1P Molicel P50B (5 Ah, 451 g) | ← aynı | ← aynı (dayanım: 6S2P) |
| Uçuş kontrolcüsü | ARK FPV (H743, endüstriyel IMU + ısıtıcı) | ARK FPV | ARKV6X (3 IMU, PAB yuvası) |
| Companion | Jetson Orin Nano 8GB Super + Auvidea JNX120X | **Jetson Orin NX 16GB** + JNX120X | Orin NX 16GB (MAXN SUPER) + JNX120X |
| Gimbal kamera | SIYI A8 mini | ← | ← |
| Aşağı kamera / ToF | IMX219-160° / VL53L1X | OV9281 global shutter / VL53L8CX 8×8 | ← B |
| Stereo (VIO + engel) | — | OAK-D Lite | OAK-D Pro W (150°) |
| Akış + lidar | MicoAir MTF-01 (UART) | Holybro H-Flow (DroneCAN) | ARK Flow MR (DroneCAN) |
| GNSS / pusula | Holybro M10 | H-RTK F9P Ultralight + RM3100 | ARK X20 RTK + ARK MAG |
| RC | TX15 + XR4 (ELRS 4.1) | TX16S MK3 + XR4 | ← B |
| Video/veri linki | WFB-ng (açık kaynak) | SIYI HM30 | Doodle Labs mini-OEM (mesh) |
| Uçuş yığını | PX4 v1.17 | PX4 v1.17 | PX4 v1.17 |

Üç seviye aynı gövde, aynı taşıyıcı kart ve aynı yazılım yığınını paylaşır; companion yükseltmesi
(Orin Nano → Orin NX) aynı yuvada modül değişimidir. **Önerilen başlangıç: Seviye A ile geliştirme, Seviye B ile ürün.**

## 2. Ağırlık, güç ve uçuş süresi bütçesi

`python3 tools/budget_calc.py --all --markdown` çıktısı (F90 itki verisi, log-log enterpolasyon,
koruma/gövde kaybı %10, kullanılabilir enerji %85, yük altında gerilim %90):

| Metrik | tier-a-gelistirme | tier-b-pro | tier-c-ust |
|---|---:|---:|---:|
| AUW (g) | 1499 | 1666 | 1639 |
| Batarya (g) | 451 | 451 | 451 |
| Maks itki (g) | 8500 | 8500 | 8500 |
| T/W (motor) | 5.67 | 5.10 | 5.18 |
| T/W (batarya sınırlı) | 2.68 | 2.39 | 2.42 |
| Hover itkisi/motor (g) | 375 | 416 | 410 |
| Hover verimi (g/W) | 5.69 | 5.44 | 5.48 |
| Hover gazı (MPC_THR_HOVER) | 0.18 | 0.20 | 0.19 |
| Hover gücü (W) | 293 | 352 | 352 |
| Hover akımı (A) | 13.6 | 16.3 | 16.3 |
| Hover süresi (dk) | 18.8 | 15.7 | 15.6 |
| Karma uçuş (dk) | 16.0 | 13.3 | 13.3 |
| Ek yük payı (g) | 329 | 147 | 167 |
| THR_MDL_FAC (tahmini) | 0.53 | 0.53 | 0.53 |
| Maliyet: hava aracı (USD, ≈) | 2,197 | 4,408 | 6,426 |
| Maliyet: yer ekipmanı (USD, ≈) | 240 | 200 | 1,505 |
| Limit kontrolleri | ✅ | ✅ | ✅ |

- **Korumanın bedeli**: Seviye B koruma ve ağ olmadan 1556 g / 18,2 dk → koruma ≈ 2,5 dk.
- **Dayanım seçeneği** (Seviye C, 6S2P P50B, 902 g): AUW 2090 g, hover **23,1 dk**, batarya
  sınırlı T/W 2,86. Ağırlık nedeniyle bu yapılandırmada avuca iniş yalnızca işaretli pedle önerilir.
- T/W (motor) ≈ 5 → bol kontrol otoritesi; asıl sınır bataryanın sürekli akımıdır (60 A).
- Maliyetler yaklaşık, vergi/kargo hariç; Jetson fiyatları Temmuz 2026 artışı sonrası.
- Doğrulama: Faz 2'de itki standı (koruma takılı) → profil eğrileri güncellenir.


## 3. Gövde (frame)

| Gövde | Dingil / kol | Kütle | Stack | Fiyat | Not |
|---|---|---|---|---|---|
| **GEPRC MOZ7 V2** | 336 mm / 6 mm | 270 g | 30,5 / 25,5 / 20 | ~$110 | 8 inç pervane alır → 7 inç etrafında koruma halkasına radyal yer kalır. **Tüm seviyelerde seçildi** |
| iFlight Chimera7 Pro V2 | 327 mm / 6 mm | 175 g (TPU hariç) | 21 mm yükseklik | €137 | Bazı satıcılarda tükendi; yeni Chimera7 O4 355 g |
| TBS Source One V5 7 DC | 320 mm / 6 mm | ~145 g | 30,5 / 20 | $45 | Deadcat, açık kaynak; stok sorunu |
| Foxeer MEGA 7 DC | 305 mm / 5,5 mm | 221 g | 30,5 (+ arka) | — | Deadcat |
| iFlight CineLR 7 (hazır referans) | 334 mm, H | 777 g kuru; 8 Ah Li-ion ile 1628 g | — | $740 | 800 g önerilen yük, ~25 dk hover iddiası |

- Hiçbir FPV 7 inç gövde Jetson için tasarlanmamış → **özel üst plaka** (karbon/PA12-CF) gerekli.
- **Gerçek-X (true-X)** seçildi: simetrik tam koruma, yönetilebilir ağırlık merkezi; gimbal ön
  burunda alçakta (A8 mini: 81° yatay FOV, −90°…+25° eğim) pervaneleri görmez. Deadcat temiz görüş
  sağlar ama asimetriktir ve korumalanması zordur.

## 4. İtki

### 4.1 Motorlar (6S, 7 inç)
| Motor | KV | Kütle | Azami | Fiyat | Not |
|---|---|---|---|---|---|
| **T-Motor F90 2806.5** | 1300 | 46,6 g | 2361 g @ 45 A (HQ 7×4×3) | $30 | Ölçülmüş itki verisi var; bazı satıcılarda üretimden kalktı |
| EMAX ECO II 2807 | 1300 | 47,6 g | 2640 g (6S) | ~$22 | Ekonomik seçim |
| EMAX ECO III 2807 | 1300 | 49,6 g | 1390 W, 55 A | — | F90 alternatifi |
| Axisflying C287 2807.5 | 1350 | 47 g | 2303 g, 48,6 A | $29 | F90 alternatifi |
| iFlight XING2 2809 | 1250 | 60,7 g | 1145 W, 50 A | $40 | 7,5 inç / ağır yük |
| T-Motor 2807 (2026) | 1250 | 52,6 g | 2178 g @ 44 A (7,5 inç) | — | Yeni liste |

### 4.2 Ölçülmüş itki verisi (bütçe hesabının temeli)
T-Motor F90 1300KV + HQProp 7×4×3, 24,2 V (güç = V×I, ESC girişi):

| Gaz | İtki | Akım | Güç | g/W |
|---|---|---|---|---|
| %30 | 353 g | 2,14 A | 52 W | **6,80** |
| %50 | 948 g | 8,83 A | 213 W | 4,45 |
| %100 | 2361 g | 45,1 A | 1059 W | 2,23 |

İki kanatlı GF 7040 %30'da 320 g / 43 W (**7,39 g/W**) ile hover'da daha verimlidir; HQ 7×4×3
daha fazla kontrol otoritesi verir. Ara noktalar log-log (güç yasası) enterpolasyonla hesaplanır.
Seviye A'da ECO II için de bu eğri **vekil** olarak kullanılmıştır; itki standında doğrulanacak.

### 4.3 ESC
| ESC | Sürekli / tepe | Hücre | Firmware | Telemetri | Kütle | Fiyat | Seviye |
|---|---|---|---|---|---|---|---|
| Holybro Tekko32 F4 50A | 50 / 60 A | 4–6S | AM32 | Bidirectional DShot, ESC telemetrisi, akım sensörü | 13,8 g | $63 | A |
| **Holybro Tekko32 F4 Metal 65A** | 65 / 75 A | 4–6S | AM32 | aynı | 15,8 g | $95 | **B** |
| ARK 4IN1 | 50 / 75 A | 3–8S | ARK32 (açık kaynak) | DShot300/600 + bidir, KISS seri telemetri | 14,5 g | $218,50 | C |
| T-Motor F55A Pro III (AM32) | 55 / 65 A | 3–8S | AM32 | telemetri | 17,8 g | $150 | — |

Firmware durumu: **BLHeli_32 Haziran 2024'te sona erdi**; **AM32** aktif (v2.20–2.21, Tem–Eyl 2026:
fren modları, DroneCAN parametre kaydı); Bluejay'in son kararlı sürümü v0.21.0 (Temmuz 2024).
→ AM32 seçildi; avuca iniş için "durunca fren" açık. DroneCAN ESC bu boyutta gereksiz:
bidirectional DShot notch filtresi için en düşük gecikmeli RPM geri bildirimini verir.


## 5. Batarya

| Hücre | Kapasite | Sürekli akım | Kütle | Not |
|---|---|---|---|---|
| Molicel P45B | 4,5 Ah | 45 A | ≤ 70 g | |
| **Molicel P50B** | 5,0 Ah | 60 A | 71 g | **Seçildi (6S1P ≈ 451 g)** |
| Molicel P60B (06/2025) | 6,0 Ah | "90 A" (basın bülteni) | — | Gelecek yükseltme adayı |
| Samsung 50S | 5,0 Ah | 25 A | ≤ 72 g | Akım sınırı düşük |
| Amprius SA112 | 6,5 Ah | 13 A | 72 g | 6S1P için akım yetersiz |

- 6S1P P50B: 108 Wh nominal, hover akımı ≈ 14–17 A (hesap) → hücrenin 60 A sınırına göre rahat.
- Yedek LiPo (ör. 6S 2000 mAh, 339 g) ≈ yarı uçuş süresi; yalnızca agresif test uçuşları için.
- Uçuş kontrolcüsünde batarya akımı **45–60 A** ile sınırlanır (dört motor tam gazda ≈ 180 A çekebilir).

## 6. Companion bilgisayar

### 6.1 Platformlar
| Platform | YZ (seyrek INT8) | RAM | Güç | Fiyat (1k adet) | Değerlendirme |
|---|---|---|---|---|---|
| Jetson Orin Nano 8GB (Super) | 67 TOPS | 8 GB | 7–25 W + MAXN SUPER | $399 (önce $299) | **Seviye A**. DLA yok, **donanım video kodlayıcı yok** (NVENC) |
| Jetson Orin NX 8GB | 117 TOPS | 8 GB | 10–40 W | $649 | — |
| **Jetson Orin NX 16GB** | 157 TOPS | 16 GB | 10–40 W + MAXN SUPER | $999 (önce $599) | **Seviye B/C**. Donanım H.265 kodlama; algı + VIO + küçük VLM birlikte sığar |
| Jetson T4000 / T5000 (Thor) | 1200 / 2070 FP4 TFLOPS | 64 / 128 GB | 40–130 W | $2999+ | 100×87 mm, güç → 7 inç için uygun değil |
| Jetson T2000 (duyuru 15.07.2026) | 400 FP4 TFLOPS | 16 GB | ~40 W | — | **Q1 2027**; ~50×87 mm, yeni taşıyıcı gerekir → gelecek yükseltme yolu |
| ModalAI VOXL 2 Mini | 15 TOPS | 8 GB | 0,5–8 W | $1350 | 11 g, PX4 + VIO dahili; Ubuntu 18.04, TFLite; YOLO çoğu zaman CPU'ya düşüyor |
| ModalAI VOXL 3 (QCS6490) | 12 TOPS | 8 GB | 0,1–6 W | — | 16 g, 2S–6S doğrudan; belgeli ama mağaza sayfası yok |
| RPi 5 + AI HAT+ 2 (Hailo-10H) | 40 TOPS INT4 | 8 GB (HAT) | ~2,5 W (HAT) | $200 (HAT) | CUDA yok → cuVSLAM yok; ayrı yazılım yığını |
| Luxonis OAK 4 D / S | 48 TOPS | 8 GB | 10–25 W | $849 / $749 | 674 / 325 g → çok ağır |

### 6.2 Taşıyıcı kartlar (Orin Nano / NX)
| Kart | Kütle | Giriş | Öne çıkan | Fiyat |
|---|---|---|---|---|
| **Auvidea JNX120X** (01/2026) | 47,2 g | 12–24 V (6S tam şarjda 25,2 V → 12 V BEC) | 5 portlu GbE switch (3 harici), 2× 4-şeritli kamera, 3 UART, CAN, **Pixhawk Autopilot Bus (PAB) yuvası** | €349 |
| ARK Jetson PAB V3 (06/2026) | 66 g | 5 V, ≥ 4 A (çok girişli) | FC (ör. ARKV6X) ile Ethernet switch; NDAA uyumlu | $800 |
| Connect Tech Hadron-DM | 56 g | 9–60 V (6S doğrudan) | −25…+85 °C | — |
| WeAct N006 (06/2026) | 57,8 g | 16–28 V (6S) | Yalnızca Orin NX; 0–40 °C | $110 |
| Holybro Pixhawk Jetson Baseboard | 85 g (+Jetson 110, +soğutucu 175 g) | 7–21 V | Pixhawk 6X ile birleşik | $396 |

Orin modülü ≈ 25–28 g, soğutucu ≈ 50–65 g (taşıyıcı tablolarından türetilmiştir).

### 6.3 Yazılım yığını (Ekim 2026)
| Bileşen | Sürüm | Not |
|---|---|---|
| **JetPack 7.2.1** (11.08.2026) | Ubuntu 24.04, CUDA 13.2.2, TensorRT 10.16.2, DeepStream 9.1 | 7.2 (02.06.2026) ile **Orin desteği geldi** — seçildi |
| JetPack 6.2.3 (12.08.2026) | Ubuntu 22.04, TensorRT 10.3 | NMS'siz YOLO26 INT8 motoru derlenemiyor → yalnızca yedek |
| ROS 2 **Jazzy** | LTS, ömür sonu 05/2029 | JetPack 7.2 ile eşleşir. **Not**: PX4 v1.17 belgesi resmi platform olarak Humble/Ubuntu 22.04'ü önerir; PX4 main (v1.18) belgesi Jazzy/24.04'ü önerir → Jazzy seçildi, sorun çıkarsa Humble Docker kapsayıcısı yedek |
| ROS 2 Lyrical (22.05.2026) | Ana platform Ubuntu 26.04 | Isaac ROS 5.0 ile geldi; Faz 5'te değerlendirilecek |
| **Isaac ROS 4.6** (18.08.2026) | Orin + JetPack 7.2 + Jazzy | Seçildi (cuVSLAM, NITROS) |
| Isaac ROS 5.0 (21.09.2026) | ROS 2 Lyrical | Sonraki yükseltme |
| Micro XRCE-DDS Agent | v2.4.3 | PX4 v1.17 belgesinin kurduğu sürüm (v3.x uyumsuz) |

> **DLA riski**: TensorRT belgeleri DLA desteğinin 10.7'de bittiğini belirtiyor; Orin'de
> JetPack 7.2 ile DLA'nın çalışıp çalışmadığı doğrulanmadı. Bütçe hesabında DLA TOPS'u sayılmaz.

## 7. Uçuş kontrol yığını ve hassas hover sensörleri

### 7.1 Yığın: PX4 v1.17 (birincil) — ArduPilot 4.7.1 (yedek)
| Konu | **PX4 v1.17.0** (13.05.2026) | ArduPilot Copter 4.7.1 (31.08.2026) |
|---|---|---|
| ROS 2 özel modlar | `px4-ros2-interface-lib`: ROS 2'den kaydedilen **gerçek uçuş modları**, failsafe entegrasyonu, yanıt vermezse yedek mod (hâlâ "deneysel") | Lua `register_custom_mode` + DDS `cmd_vel` (yalnızca guided tipi modlar, arm'lıyken) |
| Hover araçları | RPM notch, hover itki tahmincisi, autotune | Zengin notch seçenekleri, QuikTune → AutoTune, Filter/PID Review |
| SIYI A8 mini | Yerel sürücü yok → Jetson SIYI SDK ile sürer | Yerel sürücü (`MNT1_TYPE=8`) |
| Takip | Follow-Me yalnızca `FOLLOW_TARGET`, `FLW_TGT_HT` ≥ 8 m, görüntü yok → **özel mod şart** | Follow modu, görüntü yok |
| Lisans | **BSD-3** (kapalı ürün dostu) | GPLv3 (dağıtılan firmware kaynağı açılır; companion kodu kapalı kalabilir) |
| Not | v1.18.0-rc1 (10.09.2026) kararlı değil; v1.18'de bazı adlar değişiyor | 4.7'de birçok parametre SI birime geçti ve yeniden adlandırıldı |

Karar: **PX4 v1.17** birincil; FC'ler her iki yığını da çalıştırabilen kartlardan seçildi.

### 7.2 Uçuş kontrolcüleri
| FC | MCU | IMU | Baro | Yığın | Montaj | Kütle | Fiyat | Not |
|---|---|---|---|---|---|---|---|---|
| **ARK FPV** | H743 | IIM-42653 (endüstriyel) + 1 W ısıtıcı | BMP390 | PX4 (varsayılan), AP 4.7 | 30,5 | 7,5 g | $195 | 1 CAN, 9 PWM, 12 V/2 A BEC, 5,5–54 V → **A, B** |
| **ARKV6X** | H743 | 2× ICM-42688-P + IIM-42652, ısıtıcı | BMP390 | PX4, AP | PAB modül | 5 g | $400 | Taşıyıcı gerekir → **C** |
| Pixhawk 6C Mini | H743 + IO | ICM-42688-P + BMI088 (ısıtmalı, izoleli) | MS5611 | PX4, AP | 54×39 | 42,4 g | $131 | 2 CAN; A alternatifi |
| Pixhawk 6X Rev8 | H753 + IO | 3× ICM-45686 (ısıtmalı, izoleli) | ICP20100 + BMP388 | PX4, AP | modül | 31,3 g + 26,5 g taban | $269 / $389 | Ethernet (DDS) — C alternatifi |
| Matek H743-SLIM V4 | H743 | 2× ICM42688P | DPS368 | AP birinci sınıf; PX4 yalnızca derleme hedefi | 30,5 | 7 g | £85 | ArduPilot yolu |
| Holybro Kakute H7 v2 | H743 | BMI270 | BMP280 | PX4, AP | 30,5 | 8 g | $52 | PX4'te bidirectional DShot **yok** → elendi |
| ModalAI VOXL 2 Mini | QRB5165 | 2× ICM-42688-P | ICP-10100 | PX4 (üretici), AP | 30,5 | 11 g | $1350 | Jetson'ın yerini alan alternatif mimari |

### 7.3 Optik akış ve mesafe sensörleri
| Modül | Özellik | Arayüz | Kütle | Fiyat | Seviye |
|---|---|---|---|---|---|
| MicoAir MTF-01 | Akış (> 8 cm) + ToF 8 m, 2 cm ölü bölge, 100 Hz | UART (MAVLink/MSP) | 4,5 g | doğrulanmadı | A |
| **Holybro H-Flow** | PAA3905E1 + AFBR-S50LV85D + ICM-42688-P | DroneCAN | 15,2 g (çıplak 3,5 g) | $145 | **B** |
| **ARK Flow MR** | PAA3905 + IR LED, AFBR-S50LX85D (50 m, 2°) + IIM-42653 | DroneCAN | 5 g | $350 | C |
| Benewake TFmini-S | 0,1–12 m, 70 klux | UART/I2C | 5 g | $73 | — |

TF-Luna (0,2 m) ve TFmini-S (0,1 m) kör bölgeleri avuca inişin son santimleri için uygun değil →
son yaklaşma tutamaktaki kendi ToF sensörümüzle (VL53L8CX / VL53L1X) yapılır.

### 7.4 GNSS ve pusula
| Modül | Özellik | Kütle | Fiyat | Seviye |
|---|---|---|---|---|
| Holybro M10 | u-blox M10 + IST8310 | 32 g | $44 | A |
| ARK SAM GPS | SAM-M10Q + IIS2MDC | 11 g | $75 | — |
| **Holybro H-RTK F9P Ultralight** | ZED-F9P RTK | < 25 g | $279 | **B** |
| **ARK X20 RTK** | ZED-X20P, L1/L2/L5/E6, 25 Hz, RTK/PPP-RTK | 13 g (antenle 43,5 g) | $695–780 | C |
| Holybro H-RTK UM982 | Çift antenli yön (heading) | 40,3 g (antensiz) | $250 | ≥ 30 cm anten aralığı → 7 inçte sınırda |
| ARK G5 | Septentrio mosaic-G5 | 13 / 43,5 g | $745–835 | — |
| Pusula | Holybro DroneCAN RM3100 ($66) · ARK MAG RM3100 (3 g, $175) | | | B · C |

GNSS, pusula ve akış/lidar **DroneCAN** üzerinde (tek veri yolu yeterli); UART'lar Jetson
(uXRCE-DDS + MAVLink) ve RC alıcısı için ayrılır.


## 8. Kameralar ve mesafe sensörleri

### 8.1 Gimbal kamera
| Ürün | Özellik | Kütle | Güç | Fiyat | Not |
|---|---|---|---|---|---|
| **SIYI A8 mini** | 3 eksen; 1/1,7" 8 MP; 4K/2K/1080p **@25 fps**; 81° yatay FOV; 11–25,2 V | 95 g | 5 W ort. / 12 W tepe | $257 | Ethernet RTSP + SDK (UDP), ArduPilot sürücüsü, MAVLink gimbal v2; 6× **dijital** zoom (4K'da kapalı). Haziran 2023 öncesi partiler 25,2 V'u desteklemeyebilir → 12 V BEC |
| XF Z-1 Mini | 3 eksen; 4K30 akış, 1080p kayıt; 63° diyagonal | 69 g | 4,5 W | €395 | Daha hafif alternatif |
| Topotek GIP335 | Ağ gimbal | 60 g | — | — | ArduPilot 4.6+ |
| SIYI A2 mini | Tek eksen, 160° | ~85 g | — | $106 | Gimbal'sız bütçe seçeneği |

### 8.2 Aşağı kamera (avuç)
| Kamera | Özellik | Not |
|---|---|---|
| **Arducam OV9281** | **Global shutter**, 1280×800 @ 80 fps, mono, 130° D | Hareket bulanıklığı yok → **Seviye B/C** |
| IMX219-160° | Rolling shutter, renkli | JetPack stok sürücüsü → **Seviye A** |
| Arducam IMX708 geniş | 120° yatay | JetPack 6'da sürücü sorunları raporlanmış |

### 8.3 ToF (avuç mesafesi + baş üstü boşluk)
| Sensör | Bölge | Menzil | FOV | Not |
|---|---|---|---|---|
| VL53L1X | Tek | ~4 m | ~27° | Yukarı bakan boşluk sensörü; Seviye A aşağı |
| VL53L7CX | 8×8 | 3,5 m | 60°×60° | Geniş açı |
| **VL53L8CX** | 8×8 | 4 m | 63° D | Gün ışığında daha iyi (5 klux'ta 2,85 m) → **Seviye B/C aşağı** |
| VL53L9 (26.06.2026) | 54×42 | 5 cm–9 m | 54°×42° | 100 fps, MIPI; seri üretim 07/2026, kart erişimi doğrulanmadı → gelecek |

### 8.4 Stereo / derinlik (VIO + engel)
| Kamera | Derinlik FOV | Menzil | Kütle | Fiyat | Not |
|---|---|---|---|---|---|
| **OAK-D Lite** | 73° yatay | 0,4–8 m | 61 g | $269 | Global shutter stereo + IMU → **Seviye B** |
| **OAK-D Pro W** | 150° D | — | 91 g | $529 | Geniş açı + IR aydınlatma → **Seviye C** |
| Orbbec Gemini 336 | 90°×65° | 0,1–20 m | 99 g | $392 | IMU, IR geçiren filtre (dış mekân); B alternatifi |
| RealSense D435i | 87°×58° | 0,3–3 m | ~72 g | ~£434 | Cognex 22.09.2026'da satın alma anlaşması → **tedarik riski** |
| ZED X Mini | 110°×80° | 0,1–8 m | 150 g + GMSL2 kartı | $549 + $379 | Ağır |

## 9. Haberleşme

### 9.1 RC
- **ExpressLRS 4.1.0** (17.07.2026): Gemini/çift bant LR1121, otomatik anten çeşitliliği;
  MAVLink modu 3.5'ten beri var (RC linkinden yedek telemetri).
- Verici: RadioMaster TX16S MK3 ($200; dahili 2,4 GHz + 900 MHz Gemini) veya TX15 (~$140–182).
- Alıcı: RadioMaster XR4 (1,7 g, çift LR1121).
- Türkiye'de 900 MHz bandında **868 MHz (EU) düzenleme alanı** kullanılmalıdır (bkz. [08](08-guvenlik-ve-mevzuat.md)).

### 9.2 IP video + telemetri linki
| Link | Menzil | Gecikme | Hava ünitesi | Fiyat | Not |
|---|---|---|---|---|---|
| **SIYI HM30** | 20 km | 180–250 ms | 74 g | — | A8 mini ile aynı ekosistem; LAN girişi + UART telemetri. Jetson RTSP akışının taşınması **doğrulanacak** → Seviye B |
| Herelink 1.1 | 20 km | ~110 ms | 68 g | — | v1.1'de Ethernet |
| **Doodle Labs mini-OEM** | 80+ km (üretici) | — | 25 g | $1305 | Şeffaf IP radyo, 80 Mbps, mesh → Seviye C |
| WFB-ng (Jetson + RTL8812AU/EU) | ~20 km (yönlü antenle) | düşük | ~20 g | düşük | Açık kaynak, çift yönlü MAVLink + IP tüneli; **BTK güç sınırları** → Seviye A |
| OpenIPC | — | 80–100 ms @ 60 fps | — | — | Kendi kamerası; YZ katmanı zor |

## 10. Uzaktan tanımlama (Remote ID)
SHT-İHA Md. 23(7) gereği zorunlu. PX4'ün Open Drone ID desteğiyle uyumlu bir yayın modülü
(MAVLink veya DroneCAN) BOM'a eklenmiştir (≈ 10 g). İHATTYS'in ağ tabanlı tanımlama arayüzü
yayımlandığında entegrasyon güncellenecektir.

## 11. Tedarik ve fiyat riskleri
| Risk | Etki | Önlem |
|---|---|---|
| Jetson fiyat artışı (07/2026) | Seviye B/C maliyeti | Seviye A ile geliştirme, modül değişimi aynı yuvada |
| T-Motor F90 bazı satıcılarda üretimden kalktı | Motor tedariki | ECO III 2807 / Axisflying C287 eşdeğerleri; itki standında doğrulama |
| RealSense'in Cognex'e satışı | Yol haritası belirsiz | OAK-D / Orbbec tercih edildi |
| VOXL 3 mağazada yok | — | Yalnızca alternatif mimari olarak not edildi |
| Jetson T2000 Q1 2027 | — | Gelecek yükseltme; yeni taşıyıcı gerekir |

## 12. Kaynaklar (seçme)
- Jetson: https://developer.nvidia.com/embedded/jetpack/downloads · https://developer.nvidia.com/embedded/faq · https://www.cnx-software.com/2026/07/16/nvidia-jetson-t2000-and-t3000-modules-for-edge-ai-and-robotics-applications/
- Isaac ROS: https://nvidia-isaac-ros.github.io/releases/index.html · PX4 ROS 2: https://docs.px4.io/main/en/ros2/user_guide
- Taşıyıcılar: https://auvidea.eu/product/jnx120x-drone-carrier-board/ · https://arkelectron.com/product/ark-jetson-pab-v3/ · https://connecttech.com/ftp/pdf/NGX024_Hadron_DM.pdf · https://www.cnx-software.com/2026/06/30/weact-n006-a-compact-nvidia-jetson-orin-nx-carrier-board-designed-for-robots-and-uavs/
- Gövde/motor: https://rotorvillage.ca/geprc-moz7-v2-frame-kit/ · https://www.ligpower.com/product/f90-fpv-motor.html · https://shop.iflight.com/CineLR-7-6S-HD-Pro2265
- Batarya: https://akkuteile.de/en/lithium-ionen-battery/size-21700/molicel/molicel-inr21700-p50b-5000mah-60a-3-6-3-7v-li-ion-battery_100638_3507 · https://amprius.com/documents/Amprius_Product_Catalog.pdf
- Kameralar: https://res.siyi.biz/oss/other/2026/06/15/A8_mini_User_Manual_v1_10_563cde30.pdf · https://docs.arducam.com/Nvidia-Jetson-Camera/Jetvariety-Camera/OV9281/ · https://www.pololu.com/product/3419 · https://checkout.luxonis.com/products/oak-d-lite-1 · https://www.orbbec.com/products/stereo-vision-camera/gemini-336/
- Uçuş yığını: https://docs.px4.io/main/en/releases/1.17.html · https://docs.px4.io/main/en/ros2/px4_ros2_control_interface.html · https://github.com/ArduPilot/ardupilot/blob/master/ArduCopter/ReleaseNotes.txt · https://ardupilot.org/copter/docs/parameters-Copter-stable-V4.7.1.html
- FC/ESC/sensör: https://arkelectron.com/product/ark-fpv-flight-controller/ · https://arkelectron.com/product/arkv6x/ · https://holybro.com/products/tekko32-f4-metal-4in1-65a-esc-65a · https://arkelectron.com/product/ark-4in1-esc/ · https://github.com/am32-firmware/AM32/releases · https://holybro.com/products/h-flow · https://arkelectron.com/product/ark-flow-mr/ · https://arkelectron.com/product/ark-x20-rtk-gps/ · https://holybro.com/products/h-rtk-f9p-ultralight
- Linkler: https://github.com/ExpressLRS/ExpressLRS/releases · https://www.siyi.biz/en/product/image-digital-link/hm30/spec/ · https://docs.px4.io/main/en/companion_computer/video_streaming_wfb_ng_wifi
- RealSense/Cognex: https://www.sec.gov/Archives/edgar/data/0000851205/000085120526000071/exhibit991-pressrelease.htm
