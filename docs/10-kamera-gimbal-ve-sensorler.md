# 10 — Kameralar, Kendi Gimbalımız ve Uygun Fiyatlı Sensörler

Durum tarihi: **2 Ekim 2026**. Bu belge maliyeti düşürmek için yapılan değişiklikleri açıklar:
hazır gimbal kamera (SIYI A8 mini) yerine **Raspberry Pi kameraları + kendi tasarımımız gimbal**,
ve **kaliteli ama uygun fiyatlı** lazer, hareket (optik akış) ve mesafe sensörleri. Ayrıca 8×8
mesafe sensörünün nasıl çalıştığını ve drone'un boyutunu anlatır.

## 1. Kısa cevaplar

| Soru | Cevap |
|---|---|
| Drone kaç inç? | **7 inç** (pervane çapı 17,8 cm). Gövde GEPRC MOZ7 V2, motorlar arası çapraz 336 mm; korumalarla ≈ **43 × 43 cm**, yükseklik ≈ 18 cm; ağırlık ≈ **1,46 kg** (Seviye A) – 1,68 kg (B). Gerekçe: §6 |
| Hangi kameralar? | Gimbalda **Raspberry Pi Camera Module 3** (66°, 4 g, ≈ $29); aşağıda (avuç) **Camera Module 3 Wide** (102°, 4 g, ≈ $38,5). Jetson seviyelerinde IMX219 (Camera Module 2) |
| Gimbal? | **2 eksen fırçasız** (pitch + roll), yaw gövdeyle; STorM32 kontrolcü; ≈ 85 g, ≈ $95 (A8 mini: 95 g, $257) |
| Lazer / hareket / mesafe? | **MicoAir MTF-01** (akış + 8 m lazer, 4,5 g), **TFmini-S** (12 m lazer, 5 g), **VL53L8CX** (8×8 mesafe, 0,7 g, $24,95), **VL53L1X** (yukarı) |
| 8×8 sensör nasıl? | 64 bölgeli lazer mesafe ölçer: her bölge için ayrı mesafe → avucun konumu, mesafesi, eğimi (§5, çalışan örnek: `tools/tof8x8.py`) |
| Toplam etki | Seviye A (Raspberry Pi): **1456 g, ≈ 20 dk hover, hava aracı ≈ $1.490** (önceki A: 1499 g, 18,8 dk, $2.200) |

## 2. Raspberry Pi kameraları

### 2.1 Resmi kameralar (raspberrypi.com belgesinden)
| Kamera | Sensör | Çözünürlük | Yatay / dikey FOV | Deklanşör | Odak | Kütle | Fiyat |
|---|---|---|---|---|---|---|---|
| Camera Module 2 | IMX219 | 3280×2464 | 62,2° / 48,8° | Rolling | Ayarlanabilir | 3 g | ≈ $25 |
| **Camera Module 3** | IMX708 | 4608×2592 (1080p50, 720p120) | 66° / 41° | Rolling | Motorlu (PDAF) | 4 g | $25 liste, $29,25 (PiShop) |
| **Camera Module 3 Wide** | IMX708 | aynı | 102° / 67° | Rolling | Motorlu (PDAF) | 4 g | $38,50 (PiShop) |
| HQ Camera | IMX477 | 4056×3040 | lense bağlı | Rolling | Ayarlanabilir | 30,4 g + lens | — |
| Global Shutter | IMX296 | 1456×1088 @ 60 fps | lense bağlı | **Global** | Ayarlanabilir | 34 g (41 g adaptörle) + lens | $50 (lens hariç) |
| AI Camera | IMX500 (çip üstü YZ) | 4056×3040 (2028×1520 @ 30 fps) | 66° / 52,3° | Rolling | Ayarlanabilir | 6 g | $70 |

Üretim süresi taahhüdü: Camera Module 3 en az Ocak 2030, Global Shutter Ocak 2032, AI Camera Ocak 2028.

### 2.2 Seçim
| Görev | Seçim | Neden |
|---|---|---|
| Gimbal: takip + jest | **Camera Module 3** | 12 MP → dijital ROI/zoom; 66° ile 3 m'de 9 cm'lik avuç ≈ 44 piksel (A8 mini'nin 81°'sinde ≈ 34); PDAF; 4 g → küçük gimbal motorları yeter |
| Aşağı: avuç | **Camera Module 3 Wide** | 102° geniş alan; 720p120 modunda kısa pozlama ile hareket bulanıklığı azalır |
| Aşağı (alternatif) | Global Shutter | Bulanıklık hiç yok ama 34 g + C/CS lens → tutamak için ağır |
| Yapay zekâ kamerası | AI Camera (IMX500) | Ek hızlandırıcısız tespit mümkün; model esnekliği ve hız sınırlı → ileride değerlendirilir |

### 2.3 Platform uyumu (önemli)
- **Raspberry Pi 5**: iki adet 4 şeritli MIPI girişi → tam olarak gimbal + aşağı kamera. Tüm resmi
  kameralar yerel desteklidir (libcamera). 15 pinli kamera kablosu yerine "standart–mini" (22 pin) kablo.
- **Jetson Orin**: NVIDIA'nın Orin Nano geliştirme kiti belgesi yalnızca **Camera Module v2 (IMX219)**
  ve 15→22 pin dönüştürücü kabloyu anar → Jetson seviyelerinde gimbal kamerası **IMX219**; aşağı
  kamera Arducam OV9281 (global shutter, Jetson sürücülü). CM3/GS için Jetson sürücüsü üçüncü taraf.
- **Taşıyıcı kart**: Auvidea JNX120X'in kamera girişleri micro-coax'tır → Pi kameraları doğrudan
  takılamaz. Jetson seviyelerinde 22 pin FFC girişli **ARK Jetson PAB V3** seçildi (daha ucuz
  alternatif: Holybro Pixhawk Jetson Baseboard + Pixhawk 6X, ≈ $535 daha ucuz ama ≈ 43 g daha ağır).
- **Video kaydı**: Pi 5'te donanım H.264/HEVC **kodlayıcı yoktur** (yalnızca 4Kp60 HEVC çözücü);
  kayıt ve YZ katmanlı akış CPU ile yazılımla kodlanır → 720p–1080p30 önerilir. (Orin Nano'da da
  NVENC yok; Orin NX'te var.)
- **Gecikme kazancı**: A8 mini'nin RTSP akışı (≈ 65–150 ms) yerine CSI kamera (≈ 20–40 ms) →
  takip kontrolü daha atik.

## 3. Kendi gimbal tasarımımız (2 eksen)

### 3.1 Mimari
```
  gövde ── sönümleyiciler ── [ROLL motoru] ── kol ── [PITCH motoru] ── kamera plakası (CM3 + IMU)
                                                     (pitch −90° … +30°, roll ±30°)
  YAW: gimbalda yok → gövde yaw'ı hedefi ortalar (docs/06 §4, body_yaw_follow_gain)
```
2 eksen seçildi: daha hafif, daha basit, FFC kablosu için daha az dönme ekseni. Takip yazılımı
zaten gövde yaw'ını kullanıyor. 3. eksen gerekirse ileride eklenir.

### 3.2 Bileşenler (tahmini kütle ve fiyat)
| Bileşen | Seçenek | Kütle | Fiyat |
|---|---|---|---|
| Motor × 2 | 22xx sınıfı fırçasız gimbal motoru, tercihen **içi boş mil** (kablo geçişi) | ≈ 2 × 20 g | ≈ 2 × $20 |
| Kontrolcü | **STorM32-BGC** (açık kaynak, ucuz, 3 eksene kadar) · SimpleBGC32 Tiny (ticari, kararlı SDK) · SimpleFOC (tam özel, en çok iş) | ≈ 10 g | $25–40 · $100+ · $20–30 |
| Kamera IMU'su | Kontrolcünün IMU'su kamera plakasında | 2 g | — |
| Yapısal parçalar | 3B baskı PETG / PA12-CF kollar + kızaklı kamera plakası | 15–20 g | ≈ $10 |
| Titreşim izolasyonu | 4 kauçuk sönümleyici + taban plakası | ≈ 10 g | ≈ $5 |
| Kablo | 200–300 mm ince FFC (standart–mini) | — | ≈ $5 |
| **Toplam** | | **≈ 85 g** | **≈ $95** |

### 3.3 Tasarım kuralları
1. **Denge önce gelir**: kamera + plakanın ağırlık merkezi iki eksenin kesişiminde olmalı. Fırçasız
   gimbal motorları düşük torkludur; dengesiz yük titreşim, ısınma ve kayma yapar → kızaklı plaka ile ayar.
2. **Eksen sırası**: dış eksen roll (gövdeye bağlı), iç eksen pitch (kamerayı taşır). Pitch −90°
   (tam aşağı) ile +30° arası; mekanik durdurucular yazılım sınırlarının hemen dışında.
3. **Rijitlik**: 3B baskı kollar ≥ 3 mm, kaburgalı; motor montajı boşluksuz. Kol rezonansı motor
   frekanslarından (hover ≈ 150–165 Hz) uzak olmalı.
4. **Titreşim izolasyonu**: gimbal tabanı gövdeye 4 kauçuk sönümleyiciyle; sönümleyici sertliği
   gimbal kütlesine göre seçilir (yumuşak → sallanma, sert → titreşim geçişi).
5. **FFC kablo — en kritik pratik sorun**: kablo yay gibi davranır ve motora tork uygular. İnce,
   uzun FFC; her eksende gevşek "servis döngüsü"; eksene en yakın noktadan geçiş; içi boş mil
   kullanılıyorsa kablo milden geçer; sabitleme noktaları ve keskin kenar koruması.
6. **Güç**: STorM32 2S–4S ile beslenir → 12 V BEC. Motor gücü düşükten başlatılıp ısınma izlenir.
7. **Telsiz paraziti**: ArduPilot belgesine göre STorM32 v1.3x kartları **433/915 MHz** bantlarında
   parazit yapabilir → ELRS 868/900 MHz kullanılıyorsa menzil testi yapın; 2,4 GHz tercih edin veya
   kartı ekranlayın.

### 3.4 Kontrol ve entegrasyon
- Companion (Pi 5 / Jetson) ↔ STorM32 **UART** (seri protokol, 115200), 30–50 Hz açı/hız komutu;
  gimbal açı geri bildirimi hedef 3B kestirimine girer ([`behavior.yaml`](../config/mission/behavior.yaml) → `gimbal`).
- PX4'e bağlanmaz (`MNT_MODE_IN=-1`). ArduPilot alternatifi: `MNT1_TYPE=5` (STorM32 seri) veya 4
  (MAVLink, NT sürümü). Not: çoğu STorM32 kartı MAVLink Gimbal Manager v2'yi desteklemez.
- **Yazılım stabilizasyonu (EIS)**: kamera IMU'su / FC jiroskopu ile kırpmalı dijital stabilizasyon
  (%10 pay); kayıtlı video için Gyroflow.

### 3.5 En ucuz alternatif
Tek eksen servo (yalnızca tilt) + EIS: ≈ 25 g, ≈ $15. Takip için kabul edilebilir, video kalitesi
zayıf (servo titremesi). Prototipin ilk uçuşları için uygundur.

### 3.6 Test sırası
Tezgâhta denge → motor gücü ve PID → pervanesiz titreşim tablası → bağlı (tether) uçuş → serbest uçuş.

## 4. Uygun fiyatlı lazer, hareket ve mesafe sensörleri

Sürücü desteği, PX4 v1.17 ve ArduPilot 4.7.1 resmi parametre referanslarından doğrulanmıştır
(`tools/data/`).

| Görev | Sensör | Özellik | Arayüz | PX4 v1.17 | ArduPilot 4.7.1 | Kütle | Fiyat |
|---|---|---|---|---|---|---|---|
| **Hareket (optik akış) + yakın lazer** | **MicoAir MTF-01** | Akış 42° FOV, > 8 cm, 7 m/s @1 m; ToF 8 m (60 klux'ta 5 m), 2 cm ölü bölge, 100 Hz | UART (MAVLink / MSP) | MAVLink (`OPTICAL_FLOW_RAD` + `DISTANCE_SENSOR`) | `FLOW_TYPE`=5 (MAVLink) / 7 (MSP) | 4,5 g | ≈ $35 (doğrulanmadı) |
| **Lazer irtifa (uzun, güneşte iyi)** | **Benewake TFmini-S** | 0,1–12 m, 70 klux | UART / I2C | yerel sürücü (`SENS_TFMINI_CFG`) | `RNGFND1_TYPE`=20 | 5 g | ≈ $40–73 |
| Lazer irtifa (en ucuz) | Benewake TF-Luna | 0,2–8 m, ±6 cm, 250 Hz, ≤ 0,35 W, 2° FOV | UART / I2C | PX4 belgesinde yalnızca adaptörle → doğrulanacak | `RNGFND1_TYPE`=27 (seri) / 25 (I2C) | < 5 g | ≈ $20–25 |
| Akış (ayrı modül) | ThoneFlow-3901U (PMW3901) | > 8 cm | UART | yerel sürücü (`SENS_TFLOW_CFG`) | — | — | ≈ $20 (doğrulanmadı) |
| Akış + lidar (kaliteli) | Holybro H-Flow | PAA3905 + AFBR-S50 (30 m) + IMU | DroneCAN | ✔ | `FLOW_TYPE`=6 | 15 g | $145 |
| **8×8 mesafe (avuç)** | **VL53L8CX** (Pololu kartı) | 4 m, 65° D (45°×45°), 8×8 / 4×4 bölge, 5 klux'ta 2,8 m | I2C / SPI | companion okur | companion okur | 0,7 g | $24,95 |
| 8×8 geniş açı | VL53L7CX | 3,5 m, 90° D | I2C | — | — | ≈ 1 g | ≈ $25 |
| **Tek nokta (yukarı)** | **VL53L1X** | 4 m, 50 Hz | I2C | yerel sürücü (`SENS_EN_VL53L1X`) | `RNGFND1_TYPE`=16 | < 1 g | ≈ $15 |
| 360° engel (opsiyonel) | LDROBOT LD06 | 12 m, 4500 ölçüm/s, 30 klux, IPX4 | UART 230400 | yerel sürücü yok → companion | `PRX1_TYPE`=16 | — | ≈ $80 (doğrulanmadı) |

- **IMU ve barometre** ayrıca alınmaz: uçuş kontrolcüsünde (ARK FPV: endüstriyel IIM-42653 +
  ısıtıcı, BMP390).
- PX4 belgesi: optik akış için **aşağı bakan mesafe sensörü şart**; 20 m üstünde akış kaynaklı yavaş
  salınım olabilir.
- **Ultrasonik** (HC-SR05 vb.) önerilmez: pervane akışı ve gürültüsü ölçümü bozar.

**Önerilen setler**
| Seviye | Akış + irtifa | Avuç | Baş üstü | Engel |
|---|---|---|---|---|
| A — Ekonomik | MTF-01 (akış + 8 m lazer); güneşli/yüksek uçuşta + TFmini-S | VL53L8CX | VL53L1X | — |
| B — Pro | H-Flow (DroneCAN) | VL53L8CX | VL53L1X | OAK-D Lite derinliği |
| C — Üst | ARK Flow MR (DroneCAN) | VL53L8CX (VL53L9 gelince) | VL53L1X | OAK-D Pro W derinliği |

## 5. 8×8 mesafe sensörü nasıl çalışır?

### 5.1 Prensip
1. Sensördeki **VCSEL** (dikey boşluklu yüzey yayan lazer) 940 nm **görünmez kızılötesi** ışık
   darbeleri yayar; ışık 45°×45°'lik bir koniyi aydınlatır (VL53L8CX).
2. Alıcıdaki **SPAD** dizisi (tek foton çığ diyotları) geri dönen fotonları yakalar; dizi **8×8 = 64
   bölgeye** bölünmüştür.
3. Her bölge için fotonların dönüş süreleri bir **histogramda** toplanır; tepe noktası uçuş süresini
   verir: **mesafe = ışık hızı × süre / 2**.
4. Sonuç: aynı anda **64 ayrı mesafe** — 64 pikselli, çok düşük çözünürlüklü bir derinlik kamerası.
   Her bölge için ayrıca geçerlilik durumu, sinyal gücü ve ortam ışığı bilgisi gelir.

### 5.2 Geometri — bir bölge ne kadar alan görür?
VL53L8CX: 45° / 8 = **5,6° / bölge**.

| Mesafe | Toplam görüş alanı | Bir bölge | 9×10 cm avuç kaç bölge? |
|---|---|---|---|
| 1,0 m | 83 × 83 cm | ≈ 9,8 cm | ≈ 1 |
| 0,4 m | 33 × 33 cm | ≈ 3,9 cm | ≈ 2×2 |
| 0,15 m | 12 × 12 cm | ≈ 1,5 cm | ≈ 6×6 (görüşün çoğu) |
| ≤ 0,05 m | ≤ 4 × 4 cm | ≤ 0,5 cm | tamamı |

Bu yüzden: **uzakta** (> 0,5 m) avucu kamera bulur, 8×8 mesafeyi doğrular; **yakında**
(< 0,15 m) kamera aşırı yakın kalır, hizalamayı 8×8 bölge haritası üstlenir; **temasta** merkez
bölgeler tutamak içindeki gömme mesafesini (≈ 25–35 mm) okur.

### 5.3 Gerçek bir kare nasıl görünür?
`python3 tools/tof8x8.py --demo` çıktısı — avuç 0,40 m'de, zemin 1,10 m'de; `[` işaretli bölgeler
avuç kümesi:
```
        s0    s1    s2    s3    s4    s5    s6    s7
  s0    1233  1200  1180  1170  1170  1180  1200  1233
  s1    1200  1167  1146  1135  1135  1146  1167  1200
  s2    1180  1146  1124  1113  1113  1124  1146  1180
  s3    1170  1135  1113[  401[  401  1113  1135  1170
  s4    1170  1135  1113[  401[  401  1113  1135  1170
  s5    1180  1146  1124  1113  1113  1124  1146  1180
  s6    1200  1167  1146  1135  1135  1146  1167  1200
  s7    1233  1200  1180  1170  1170  1180  1200  1233
Avuç: 4 bölge, mesafe 0.400 m, sapma x=+0.000 m y=+0.000 m, eğim 0.0°, düzlük ±0.0 mm, temas: hayır
```
Köşelerin daha uzak okunması (1233 mm) ışının eğik gitmesindendir (radyal mesafe); kod bunu dik
mesafeye çevirir. Diğer senaryolar: `--palm-distance 0.15 --tilt 20` (eğik avuç → 20,0° bulunur),
`--palm-distance 0.03` (temas: EVET), `--no-palm` (zemin → avuç yok), `--offset-x 0.06` (yana kayma).

### 5.4 Avuca inişte kullanımı ([05](05-avuca-inis-tasarimi.md))
1. **Ayırma**: en yakın bölgeden başlayıp ≈ 6 cm kalınlıktaki bitişik bölgeler → avuç kümesi;
   küme zeminden en az 15 cm yakın olmalı (yoksa gördüğümüz zemindir).
2. **Merkez**: küme noktalarının ortalaması → drone'un yatay hizalama hatası.
3. **Düzlem uydurma**: z = a·x + b·y + c → eğim (avuç düz ve yatay mı?) ve düzlük artığı.
4. **Temas**: merkez 2×2 bölgenin en az 3'ü temas eşiğinin (35 mm) altında.
5. **Kamera ile birleştirme**: kamera "bu bir açık avuç" der, 8×8 "şu mesafede, şu konumda, düz"
   der; ikisi tutarlı değilse iniş başlamaz.

### 5.5 Sınırlamalar
- **Güneş ışığı** menzili kısaltır (VL53L8CX 5 klux'ta 2,8 m, VL53L5CX ≈ 1,7 m); doğrudan güneşte
  daha da kısalır — avuç menzili (0,1–1 m) için yeterli, irtifa için değil (irtifa: TFmini-S).
- Koyu ve çok parlak yüzeyler, eldivenler sinyali değiştirir → kamera ile doğrulama şart.
- Sensör bir **pencere/kapak camının** arkasına konursa iç yansıma oluşur → kalibrasyon gerekir.
- Birden fazla ToF sensörü aynı alana bakarsa birbirini etkileyebilir → zaman paylaşımı.
- Sürücünün **radyal mı dik mi** mesafe verdiği ST belgesinden doğrulanmalı; kod iki modu destekler.
- 8×8 modunda en fazla 15 Hz, 4×4 modunda 60 Hz → son santimlerde 4×4'e geçilebilir.

### 5.6 Bağlantı
- **Raspberry Pi 5**: VL53L8CX'i **SPI** üzerinden bağlayın (I2C adresi VL53L1X ile aynı, 0x29);
  LPn (etkinleştirme) ve INT (veri hazır) birer GPIO'ya. Pololu kartı 3,2–5,5 V ile beslenir.
- **Jetson**: 40 pinli başlıktaki SPI/I2C. Sürücü: ST'nin ULD C kütüphanesi → ROS 2 C++ düğümü
  (`dc7_palm_landing`), analiz mantığı `tools/tof8x8.py` ile aynıdır.

## 6. Drone kaç inç ve neden?

| Ölçü | Değer |
|---|---|
| Pervane | **7 inç** (17,8 cm), HQProp 7×4×3 |
| Gövde | GEPRC MOZ7 V2, motorlar arası çapraz **336 mm** (gerçek X) |
| Korumalarla izdüşüm | ≈ **43 × 43 cm** (uçtan uca çapraz ≈ 53 cm) |
| Yükseklik | ≈ 18 cm (avuç tutamağı pervane düzleminin ≥ 120 mm altında) |
| Kalkış ağırlığı | ≈ 1,46 kg (A) · 1,68 kg (B) · 1,66 kg (C) |
| Hover süresi (hesap) | ≈ 20 dk (A) · 15,6 dk (B) · 15,5 dk (C) |

| Boyut | Artı | Eksi |
|---|---|---|
| 5 inç | Küçük; pervane enerjisi daha düşük | ≈ 1,4 kg yükte disk yükü yüksek → verim düşük, uçuş süresi belirgin şekilde kısa; rüzgârda zayıf |
| **7 inç ✔** | ≈ 5,5–5,8 g/W hover verimi, 15–20 dk, rüzgâr dayanımı, koruma halkası için yer | Daha büyük pervane → tam koruma ve ağ zorunlu |
| 10 inç | Daha uzun uçuş | Büyük ve ağır; avuca iniş için uygun değil |

## 7. Kaynaklar
- Raspberry Pi kamera belgesi: https://www.raspberrypi.com/documentation/accessories/camera.html
- Camera Module 3: https://www.raspberrypi.com/products/camera-module-3/ · Global Shutter: https://www.raspberrypi.com/products/raspberry-pi-global-shutter-camera/ · AI Camera: https://www.raspberrypi.com/products/ai-camera/
- Raspberry Pi 5: https://www.raspberrypi.com/products/raspberry-pi-5/ · ürün özeti (HEVC çözücü, 0–70 °C) · fiyat artışı (02.02.2026): https://www.raspberrypi.com/news/more-memory-driven-price-rises/
- AI HAT+ 2: https://www.raspberrypi.com/products/ai-hat-plus-2/ · AI HAT+: https://www.raspberrypi.com/products/ai-hat/
- Jetson Orin Nano kamera bağlantısı: https://docs.nvidia.com/jetson/orin-nano-devkit/user-guide/latest/howto.html
- ARK Jetson PAB V3: https://arkelectron.com/product/ark-jetson-pab-v3/ · Auvidea JNX120X: https://auvidea.eu/product/jnx120x-drone-carrier-board/
- STorM32 (ArduPilot): https://ardupilot.org/copter/docs/common-storm32-gimbal.html
- VL53L8CX (Pololu): https://www.pololu.com/product/3419
- MTF-01: https://micoair.com/optical_range_sensor_mtf-01/ · TF-Luna: https://en.benewake.com/TFLuna/index.html · ArduPilot Benewake: https://ardupilot.org/copter/docs/common-benewake-tf02-lidar.html
- PX4 optik akış: https://docs.px4.io/v1.17/en/sensor/optical_flow.html · PX4 mesafe sensörleri: https://docs.px4.io/v1.17/en/sensor/rangefinders.html
- LD06 (ArduPilot): https://ardupilot.org/copter/docs/common-ld06.html
