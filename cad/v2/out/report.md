# DC7 v2 CAD raporu (otomatik — `python3 cad/v2/build_v2.py`)

## Parçalar

| Parça | Adet | Üretim malzemesi | Hacim (cm³) | Kütle (g) | Prototip (MJF) (g) | Ağırlık merkezi (mm) | Not |
|---|---:|---|---:|---:|---:|---|---|
| top_shell | 1 | PC/ABS | 48.6 | 55.9 | 49.1 | (+1, +0, +26) | kanopi; yarım kol soketleri ayrım düzlemine açık; 2K siyah vizör (ToF penceresi + bal peteği çıkış), logo; maçasız |
| bottom_tub | 1 | PC/ABS | 53.7 | 61.7 | 54.2 | (+2, +0, -31) | taşıyıcı: V uçlu kol soketleri, batarya rayları, vida kuleleri; havalandırma tabanda (bal peteği) → maçasız |
| nose_cover | 1 | PC/ABS | 16.7 | 19.2 | 16.9 | (+115, -1, +11) | burun üst yarısı: alın + yanak üstleri + ağız astarı (2K: dış gövde rengi, astar siyah) |
| nose_chin | 1 | PC/ABS | 19.7 | 22.7 | 19.9 | (+99, +0, -19) | burun alt yarısı + sönümleyici perdesi; gimbal iki yarının arasına oturur (ayrım = pitch ekseni) |
| arm_duct | 4 | PA6-GF30 | 56.4 | 76.6 | 56.9 | (-23, +0, +12) yerel | TEK kalıp × 4; U kesit kol + çan ağızlı kanal + bal peteği ızgara + 3 radyal kaburga + motor çanı eteği; göbekten yolluk |
| top_grille | 4 | PC | 9.4 | 11.2 | 9.4 | (-0, -0, +41) | çıkarılabilir üst ızgara: 6 ayak + 3 geçme tırnak; pervane değişiminde çıkar; tek kalıp × 4 |
| pod | 1 | PC/ABS | 20.0 | 23.0 | 20.2 | (-0, +0, -52) | avuç ayağı (kısa, Ø74); sensör tablası tabandan 22 mm, IR camlı |
| pod_tip | 1 | TPU | 3.3 | 4.0 | 4.0 | (-0, -0, -69) | avuca değen uç (TPU 95A; seri üretimde ayağın üstüne ikinci enjeksiyon) |
| sensor_window | 1 | PC | 3.7 | 4.5 | 3.8 | (+0, -0, -50) | IR geçirgen sensör camı (siyah IR mürekkep maskeli), sensörlere sıfır boşlukla |
| battery_shell | 1 | PC/ABS | 39.5 | 45.5 | 39.9 | (-55, -0, -10) | akıllı batarya kabuğu + kuyruk kapağı (gövde çizgisini tamamlar) |

Gövde toplamı: **588 g** (üretim) · 473 g (MJF PA12 prototip)

## Ön gimbal (cad/v2/gimbal_v2.py)

Kamera merkezi (129.4, 0, -9) mm: burnun önünde, orta hatta; pitch ekseni 10.0 mm arkada, kapsülün ağırlık merkezinde. Eksen sırası: dışta pitch (motor sağ yanakta, rulman sol yanakta), içte roll (kameranın arkasında, optik eksenle eş eksenli).

| Parça | Malzeme | Hacim (cm³) | Kütle (g) | Prototip (MJF) (g) | Not |
|---|---|---:|---:|---:|---|
| gimbal_bracket | PA6-GF30 | 10.9 | 14.9 | 11.0 | sönümlü U taşıyıcı: arka plaka 4 sönümleyiciyle perdeye; yan plakalar yanakların içinde |
| gimbal_frame | PA6-GF30 | 4.2 | 5.8 | 4.3 | pitch çerçevesi: roll motoru arka plakada; kollar pitch eksenine (+y motor, −y pim) |
| gimbal_cradle | PA6-GF30 | 2.3 | 3.1 | 2.3 | beşik: roll rotoruna bağlı; CM3 kartı ara parçalarla önünde |
| camera_housing | PC | 1.5 | 1.8 | 1.5 | kamera başlığı: öne daralan kapak (1 mm); mercek halkası alüminyum, koruyucu cam |

Satın alınan / temsili: kamera IMU 2.0 g, roll motoru 20.0 g, camera_bezel 0.3 g, camera_glass 0.1 g, pitch motoru 20.0 g, pim + rulman 2.8 g, sönümleyiciler 4.0 g, kontrolcü 10.0 g.
Gimbal toplamı (kamera hariç) **84.7 g**, ağırlık merkezi (100.8, +6.5, -4.3) mm.
Görüş temiz: -90° … +20° (mekanik -90…+30°, yazılım sınırı +15°).

## Varyant profiliyle karşılaştırma (config/hardware/variants/tier-a-entegre.yaml)

| Bileşen | Profil (g) | CAD (g) | Fark |
|---|---:|---:|---:|
| Üst kabuk (PC/ABS, 2 mm et) | 56 | 55.9 | %-0 |
| Alt kabuk / taşıyıcı (PC/ABS, kaburgalı) | 62 | 61.7 | %-0 |
| Burun kapağı (iki yarım: alın, yanaklar, siyah ağız astarı, sönümleyici perdesi) | 42 | 41.9 | %-0 |
| Kol–kanal modülü (PA6-GF30: kol + motor yuvası + çan ağızlı kanal + alt ızgara + motor çanı eteği) | 77 | 76.6 | %-0 |
| Avuç ayağı (PC/ABS + TPU uç + IR geçirgen sensör camı) | 32 | 31.5 | %-2 |
| Üst ızgara (PC, çıkarılabilir; pervane üstü parmak koruması) | 11 | 11.2 | %+2 |
| Akıllı batarya kabuğu + kilit + konnektör | 56 | 55.7 | %-1 |
| Kendi tasarım 2 eksen fırçasız gimbal (burun ağzında, ortada: dışta pitch, içte roll; sönümlü taşıyıcı) + kamera başlığı | 85 | 84.7 | %-0 |

## v1 (FPV gövde) ↔ v2 (bütünleşik)

| | v1 | v2 |
|---|---:|---:|
| Kalkış ağırlığı | 1456 g | 1594 g |
| Hover süresi | 20.1 dk | 16.5 dk |
| T/W (batarya sınırlı) | 2.77 | 2.39 |
| Hover gazı | 0.17 | 0.20 |

## Yerleşim (cad/v2/layout_v2.py)

- Toplam 1594 g; batarya paketi merkezi x = -36.8 mm; ağırlık merkezi (-0.0, +0.3, -0.8) mm
- Avuçta devrilme açısı 27.4°

## Kontroller

| Kontrol | Sonuç | Değer |
|---|---|---|
| iç bileşenler gövdenin içinde | ✅ | 11 bileşen |
| bileşenler çakışmıyor | ✅ | temiz |
| bileşen ↔ kanal boşluğu | ✅ | en yakın: GNSS + pusula 13.5 mm |
| gövde ↔ kanal boşluğu | ✅ | 3.8 mm ≥ 3 |
| batarya kuyruktan takılıyor (kapak = kuyruk) | ✅ | hücre merkezi x = -36.8 mm; kapaktan ön uca 94.3 mm (en az 92.2; BMS boşluğu +2.1 mm, en çok +8) |
| CG yatay ofset | ✅ | (-0.0, +0.3) mm |
| IMU ↔ CG (EKF2_IMU_POS) | ✅ | (+0, -0, +21) mm |
| GNSS/pusula ↔ gürültü kaynakları | ✅ | en yakın batarya konnektörü 74 mm ≥ 70 |
| avuç ayağı pervane düzleminin altında (tam kapalı pervane) | ✅ | 102 mm ≥ 100 (üst + alt ızgara + motor eteği; açık üstte 120) |
| gimbal sarkmıyor (burun alt çizgisinde, ayağın üstünde) | ✅ | en alçak -31 mm (−90…+30° pitch); burun altı -29, ayak -72 mm |
| kamera orta hatta, gövde orta yüksekliğinde | ✅ | y = 0; kamera z -9, ağırlık merkezi -1, burun ortası -2 mm |
| ayak sensörleri ağızdan kırpılmadan görür | ✅ | CM3 Wide 57° ≥ 51°, VL53L8CX 37° ≥ 32°, MTF-01 34° ≥ 21° |
| kol dikey eğilme frekansı bantların arasında | ✅ | 210–333 Hz (katılım 0–1, E %70–100); pencere 204–348 Hz |
| kol yanal eğilme frekansı 1× bandının üstünde | ✅ | 208 Hz ≥ 204 (p = 0,5, nemli; ızgara düzlem içinde rijit olduğundan gerçekte daha yüksek) |
| kanal kenarına düşme (kol kökü) | ✅ | F 156 N → σ 27 MPa, emniyet 3.3 ≥ 2 |
| top_shell: tek katı | ✅ | 1 katı |
| bottom_tub: tek katı | ✅ | 1 katı |
| nose_cover: tek katı | ✅ | 1 katı |
| nose_chin: tek katı | ✅ | 1 katı |
| arm_duct: tek katı | ✅ | 1 katı |
| top_grille: tek katı | ✅ | 1 katı |
| pod: tek katı | ✅ | 1 katı |
| pod_tip: tek katı | ✅ | 1 katı |
| sensor_window: tek katı | ✅ | 1 katı |
| battery_shell: tek katı | ✅ | 1 katı |
| gimbal_bracket: tek katı | ✅ | 1 katı |
| gimbal_frame: tek katı | ✅ | 1 katı |
| gimbal_cradle: tek katı | ✅ | 1 katı |
| camera_housing: tek katı | ✅ | 1 katı |
| gövde ↔ kanal halkası | ✅ | 2.7 mm ≥ 2 |
| kol kökleri soketlerden temassız geçer | ✅ | 0.00 mm³ |
| batarya tünele çakışmasız girer | ✅ | 0.00 mm³ |
| pervane ↔ kanal / ızgara | ✅ | 0.00 mm³ |
| gimbal taşıyıcısı ↔ burun (sönümleyici yolu) | ✅ | en yakın taşıyıcı ↔ nose_cover 1.5 mm ≥ 1 |
| gimbal kapsülü hareket aralığı ↔ burun, taşıyıcı, kanallar | ✅ | 0.00 mm³ (pitch −90/−45/0/+15/+30 × roll −30/0/+30) |
| gimbal kapsülü ↔ ağız boşluğu | ✅ | 2.0 mm ≥ 1,5 (−90°, 0°, +30° / roll 30°) |
| ağız payı (süpürme + 2 mm) ve parametreler CAD ile uyumlu | ✅ | süpürme r 22.1 / z -21.6…+20.9 mm; ağız r 24.5, üst 23.0 |
| gimbal dengesi: pitch ekseni kapsül ağırlık merkezinde, roll ekseni optik eksende | ✅ | pitch ekseni x -9.97 mm (parametre -10.0); roll kaçıklığı 0.02 mm |
| avuç ayağı en alçak nokta (avuca yalnızca o değer) | ✅ | ayak -72 mm; sonraki en alçak: bottom_tub -52.2 mm |
| üst ızgara ↔ pervane (gürültü payı) / somun | ✅ | kanat 5.4 mm ≥ 4, somun ve mil 3.8 mm ≥ 3 |
| motor çanı ↔ etek (dönen çan yandan kapalı) | ✅ | 3.0 mm ≥ 2 |
| ayak sensörleri ↔ Pi ve batarya | ✅ | 6.8 mm ≥ 1,5 |
| iç kartlar (Pi, FC, ESC, GNSS, gimbal kontrolcüsü) ↔ kabuklar, kovanlar, kollar | ✅ | 0.00 mm³ |
| kabuk et kalınlığı 1,5–2,5 mm | ✅ | gövde 2.0, burun 1.6 mm (iç duvarlar: ağız astarı 1.2, perde 1.2) |
| kol / kanal et kalınlığı ≥ 1,5 mm | ✅ | kol 2.2, kanal 1.6 mm |
| kaburga ≤ 0,6 × et | ✅ | 0.6 |
| kalıptan çıkma açısı ≥ 1° | ✅ | 1.5° (kanal iç duvarı) |
| ızgara hücresi ≤ 10 mm, kaburga ≥ 1,2 mm | ✅ | hücre 10.0 mm, kaburga 1.2 mm |

## Kol kesiti seçimi (cad/v2/analysis_v2.py)

Güvenli pencere 204–348 Hz (1× dönüş bandının üst payı … 3× kanat geçişinin alt payı).

| Kök yüksekliği | Dikey frekans aralığı | Sonuç |
|---:|---:|---|
| 24 mm | 124–197 Hz | bant payına giriyor |
| 28 mm | 154–244 Hz | bant payına giriyor |
| 35 mm (seçili) | 210–333 Hz | pencerede |
| 38 mm | 235–373 Hz | bant payına giriyor |
