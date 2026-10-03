# DC7 v2 CAD raporu (otomatik — `python3 cad/v2/build_v2.py`)

## Parçalar

| Parça | Adet | Üretim malzemesi | Hacim (cm³) | Kütle (g) | Prototip (MJF) (g) | Ağırlık merkezi (mm) | Not |
|---|---:|---|---:|---:|---:|---|---|
| top_shell | 1 | PC/ABS | 47.9 | 55.1 | 48.4 | (+0, -0, +26) | kanopi; yarım kol soketleri ayrım düzlemine açık; kanopi emiş yarıkları dikey (maçasız) |
| bottom_tub | 1 | PC/ABS | 52.4 | 60.2 | 52.9 | (+2, +0, -32) | taşıyıcı: V uçlu kol soketleri, batarya rayları, vida kuleleri; yan yarıklar için 2 kayar maça |
| nose_cover | 1 | PC/ABS | 22.1 | 25.4 | 22.3 | (+103, +0, -1) | saten-mat siyah; gimbal başlığı (tavan) + arka perde, yanaksız: gimbal altta açıkta |
| arm_duct | 4 | PA6-GF30 | 56.4 | 76.6 | 56.9 | (-23, -0, +12) yerel | TEK kalıp × 4; U kesit kol + çan ağızlı kanal + bal peteği ızgara + 3 radyal kaburga + motor çanı eteği; göbekten yolluk |
| top_grille | 4 | PC | 9.4 | 11.2 | 9.4 | (-0, -0, +41) | çıkarılabilir üst ızgara: 6 ayak + 3 geçme tırnak; pervane değişiminde çıkar; tek kalıp × 4 |
| pod | 1 | PC/ABS | 19.9 | 22.9 | 20.1 | (-0, +0, -52) | avuç ayağı (kısa, Ø74); sensör tablası tabandan 22 mm, IR camlı |
| pod_tip | 1 | TPU | 3.3 | 4.0 | 4.0 | (+0, -0, -69) | avuca değen uç (TPU 95A; seri üretimde ayağın üstüne ikinci enjeksiyon) |
| sensor_window | 1 | PC | 3.7 | 4.5 | 3.8 | (-0, +0, -50) | IR geçirgen sensör camı (siyah IR mürekkep maskeli), sensörlere sıfır boşlukla |
| battery_shell | 1 | PC/ABS | 37.4 | 43.1 | 37.8 | (-52, +0, -12) | akıllı batarya kabuğu + kuyruk kapağı (gövde çizgisini tamamlar) |

Gövde toplamı: **566 g** (üretim) · 455 g (MJF PA12 prototip)

## Varyant profiliyle karşılaştırma (config/hardware/variants/tier-a-entegre.yaml)

| Bileşen | Profil (g) | CAD (g) | Fark |
|---|---:|---:|---:|
| Üst kabuk (PC/ABS, 2 mm et) | 55 | 55.1 | %+0 |
| Alt kabuk / taşıyıcı (PC/ABS, kaburgalı) | 60 | 60.2 | %+0 |
| Burun kapağı (gimbal başlığı + arka perde) | 25 | 25.4 | %+2 |
| Kol–kanal modülü (PA6-GF30: kol + motor yuvası + çan ağızlı kanal + alt ızgara + motor çanı eteği) | 77 | 76.6 | %-0 |
| Avuç ayağı (PC/ABS + TPU uç + IR geçirgen sensör camı) | 31 | 31.4 | %+1 |
| Üst ızgara (PC, çıkarılabilir; pervane üstü parmak koruması) | 11 | 11.2 | %+2 |
| Akıllı batarya kabuğu + kilit + konnektör | 53 | 53.3 | %+0 |

## v1 (FPV gövde) ↔ v2 (bütünleşik)

| | v1 | v2 |
|---|---:|---:|
| Kalkış ağırlığı | 1456 g | 1564 g |
| Hover süresi | 20.1 dk | 16.9 dk |
| T/W (batarya sınırlı) | 2.77 | 2.44 |
| Hover gazı | 0.17 | 0.19 |

## Yerleşim (cad/v2/layout_v2.py)

- Toplam 1564 g; batarya paketi merkezi x = -34.3 mm; ağırlık merkezi (-0.0, +0.4, -1.9) mm
- Avuçta devrilme açısı 27.8°

## Kontroller

| Kontrol | Sonuç | Değer |
|---|---|---|
| iç bileşenler gövdenin içinde | ✅ | 10 bileşen |
| bileşenler çakışmıyor | ✅ | temiz |
| bileşen ↔ kanal boşluğu | ✅ | en yakın: GNSS + pusula 13.5 mm |
| gövde ↔ kanal boşluğu | ✅ | 3.6 mm ≥ 3 |
| batarya kuyruktan takılıyor (kapak = kuyruk) | ✅ | hücre merkezi x = -34.3 mm; kapaktan ön uca 96.8 mm (en az 92.2; BMS boşluğu +4.6 mm, en çok +8) |
| CG yatay ofset | ✅ | (-0.0, +0.4) mm |
| IMU ↔ CG (EKF2_IMU_POS) | ✅ | (+0, -0, +22) mm |
| GNSS/pusula ↔ gürültü kaynakları | ✅ | en yakın batarya konnektörü 77 mm ≥ 70 |
| avuç ayağı pervane düzleminin altında (tam kapalı pervane) | ✅ | 102 mm ≥ 100 (üst + alt ızgara + motor eteği; açık üstte 120) |
| gimbal avuç ayağından yukarıda | ✅ | gimbal alt ucu -62 mm, ayak tabanı -72 mm (≥ 10 mm pay) |
| ayak sensörleri ağızdan kırpılmadan görür | ✅ | CM3 Wide 57° ≥ 51°, VL53L8CX 37° ≥ 32°, MTF-01 34° ≥ 21° |
| kol dikey eğilme frekansı bantların arasında | ✅ | 210–333 Hz (katılım 0–1, E %70–100); pencere 204–348 Hz |
| kol yanal eğilme frekansı 1× bandının üstünde | ✅ | 208 Hz ≥ 204 (p = 0,5, nemli; ızgara düzlem içinde rijit olduğundan gerçekte daha yüksek) |
| kanal kenarına düşme (kol kökü) | ✅ | F 153 N → σ 27 MPa, emniyet 3.4 ≥ 2 |
| top_shell: tek katı | ✅ | 1 katı |
| bottom_tub: tek katı | ✅ | 1 katı |
| nose_cover: tek katı | ✅ | 1 katı |
| arm_duct: tek katı | ✅ | 1 katı |
| top_grille: tek katı | ✅ | 1 katı |
| pod: tek katı | ✅ | 1 katı |
| pod_tip: tek katı | ✅ | 1 katı |
| sensor_window: tek katı | ✅ | 1 katı |
| battery_shell: tek katı | ✅ | 1 katı |
| gövde ↔ kanal halkası | ✅ | 3.0 mm ≥ 2 |
| kol kökleri soketlerden temassız geçer | ✅ | 0.00 mm³ |
| batarya tünele çakışmasız girer | ✅ | 0.00 mm³ |
| pervane ↔ kanal / ızgara | ✅ | 0.00 mm³ |
| gimbal ↔ burun kapağı (nötr) | ✅ | 0.00 mm³ |
| gimbal + kamera başlığı hareket aralığı ↔ burun ve gimbal gövdesi | ✅ | 0.00 mm³ (pitch −90/−45/0/+30, roll ±30) |
| avuç ayağı en alçak nokta (avuca yalnızca o değer) | ✅ | ayak -72 mm; sonraki en alçak: gimbal_roll_arm -61.8 mm |
| üst ızgara ↔ pervane (gürültü payı) / somun | ✅ | kanat 5.4 mm ≥ 4, somun ve mil 3.8 mm ≥ 3 |
| motor çanı ↔ etek (dönen çan yandan kapalı) | ✅ | 3.0 mm ≥ 2 |
| ayak sensörleri ↔ Pi ve batarya | ✅ | 6.8 mm ≥ 1,5 |
| iç kartlar (Pi, FC, ESC, GNSS) ↔ kabuklar, kovanlar, kollar | ✅ | 0.00 mm³ |
| kabuk et kalınlığı 1,5–2,5 mm | ✅ | 2.0 mm |
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
