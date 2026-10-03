# DC7 v2 CAD raporu (otomatik — `python3 cad/v2/build_v2.py`)

## Parçalar

| Parça | Adet | Üretim malzemesi | Hacim (cm³) | Kütle (g) | Prototip (MJF) (g) | Ağırlık merkezi (mm) | Not |
|---|---:|---|---:|---:|---:|---|---|
| top_shell | 1 | PC/ABS | 48.1 | 55.3 | 48.6 | (+0, -0, +26) | kanopi; yarım kol soketleri ayrım düzlemine açık; kanopi emiş yarıkları dikey (maçasız) |
| bottom_tub | 1 | PC/ABS | 52.3 | 60.2 | 52.9 | (+2, +0, -32) | taşıyıcı: V uçlu kol soketleri, batarya rayları, vida kuleleri; yan yarıklar için 2 kayar maça |
| nose_cover | 1 | PC/ABS | 24.7 | 28.4 | 24.9 | (+108, -0, -5) | saten-mat siyah; gimbal başlığı (tavan) + arka perde, yanaksız: gimbal altta açıkta |
| arm_duct | 4 | PA6-GF30 | 53.3 | 72.5 | 53.8 | (-24, +0, +12) yerel | TEK kalıp × 4; U kesit kol + çan ağızlı kanal + bal peteği ızgara + 3 radyal kaburga; göbekten yolluk |
| pod | 1 | PC/ABS | 28.5 | 32.8 | 28.8 | (-0, +0, -65) | avuç ayağı; sensör tablası (tabandan 25 mm) |
| pod_tip | 1 | TPU | 3.0 | 3.6 | 3.6 | (-0, -0, -92) | avuca değen uç (TPU 95A; seri üretimde ayağın üstüne ikinci enjeksiyon) |
| battery_shell | 1 | PC/ABS | 37.2 | 42.8 | 37.6 | (-52, +0, -12) | akıllı batarya kabuğu + kuyruk kapağı (gövde çizgisini tamamlar) |

Gövde toplamı: **513 g** (üretim) · 412 g (MJF PA12 prototip)

## Varyant profiliyle karşılaştırma (config/hardware/variants/tier-a-entegre.yaml)

| Bileşen | Profil (g) | CAD (g) | Fark |
|---|---:|---:|---:|
| Üst kabuk (PC/ABS, 2 mm et) | 55 | 55.3 | %+0 |
| Alt kabuk / taşıyıcı (PC/ABS, kaburgalı) | 60 | 60.2 | %+0 |
| Burun kapağı (gimbal başlığı + arka perde) | 28 | 28.4 | %+1 |
| Kol–kanal modülü (PA6-GF30: kol + motor yuvası + çan ağızlı kanal + alt ızgara) | 72 | 72.5 | %+1 |
| Avuç ayağı (PC/ABS + TPU uç) | 36 | 36.4 | %+1 |
| Akıllı batarya kabuğu + kilit + konnektör | 52 | 53.0 | %+2 |

## v1 (FPV gövde) ↔ v2 (bütünleşik)

| | v1 | v2 |
|---|---:|---:|
| Kalkış ağırlığı | 1456 g | 1507 g |
| Hover süresi | 20.1 dk | 19.2 dk |
| T/W (batarya sınırlı) | 2.77 | 2.68 |
| Hover gazı | 0.17 | 0.18 |

## Yerleşim (cad/v2/layout_v2.py)

- Toplam 1507 g; batarya paketi merkezi x = -35.3 mm; ağırlık merkezi (-0.0, +0.4, -4.5) mm
- Avuçta devrilme açısı 20.0°

## Kontroller

| Kontrol | Sonuç | Değer |
|---|---|---|
| iç bileşenler gövdenin içinde | ✅ | 10 bileşen |
| bileşenler çakışmıyor | ✅ | temiz |
| bileşen ↔ kanal boşluğu | ✅ | en yakın: GNSS + pusula 13.5 mm |
| gövde ↔ kanal boşluğu | ✅ | 3.6 mm ≥ 3 |
| batarya kuyruktan takılıyor (kapak = kuyruk) | ✅ | hücre merkezi x = -35.3 mm; kapaktan ön uca 95.8 mm (en az 92.2; BMS boşluğu +3.6 mm, en çok +8) |
| CG yatay ofset | ✅ | (-0.0, +0.4) mm |
| IMU ↔ CG (EKF2_IMU_POS) | ✅ | (+0, -0, +24) mm |
| GNSS/pusula ↔ gürültü kaynakları | ✅ | en yakın batarya konnektörü 76 mm ≥ 70 |
| avuç ayağı pervane düzleminin altında | ✅ | 125 mm ≥ 120 |
| gimbal avuç ayağından yukarıda | ✅ | gimbal alt ucu -79 mm, ayak tabanı -95 mm |
| kol dikey eğilme frekansı bantların arasında | ✅ | 216–327 Hz (katılım 0–1, E %70–100); pencere 204–348 Hz |
| kol yanal eğilme frekansı 1× bandının üstünde | ✅ | 217 Hz ≥ 204 (p = 0,5, nemli; ızgara düzlem içinde rijit olduğundan gerçekte daha yüksek) |
| kanal kenarına düşme (kol kökü) | ✅ | F 148 N → σ 27 MPa, emniyet 3.3 ≥ 2 |
| top_shell: tek katı | ✅ | 1 katı |
| bottom_tub: tek katı | ✅ | 1 katı |
| nose_cover: tek katı | ✅ | 1 katı |
| arm_duct: tek katı | ✅ | 1 katı |
| pod: tek katı | ✅ | 1 katı |
| pod_tip: tek katı | ✅ | 1 katı |
| battery_shell: tek katı | ✅ | 1 katı |
| gövde ↔ kanal halkası | ✅ | 3.0 mm ≥ 2 |
| kol kökleri soketlerden temassız geçer | ✅ | 0.00 mm³ |
| batarya tünele çakışmasız girer | ✅ | 0.00 mm³ |
| pervane ↔ kanal / ızgara | ✅ | 0.00 mm³ |
| gimbal ↔ burun kapağı (nötr) | ✅ | 0.00 mm³ |
| gimbal + kamera başlığı hareket aralığı ↔ burun ve gimbal gövdesi | ✅ | 0.00 mm³ (pitch −90/−45/0/+30, roll ±30) |
| avuç ayağı en alçak nokta (avuca yalnızca o değer) | ✅ | ayak -95 mm; sonraki en alçak: gimbal_roll_arm -71.8 mm |
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
| 24 mm | 133–202 Hz | bant payına giriyor |
| 28 mm | 165–250 Hz | bant payına giriyor |
| 34 mm (seçili) | 216–327 Hz | pencerede |
| 37 mm | 243–368 Hz | bant payına giriyor |
