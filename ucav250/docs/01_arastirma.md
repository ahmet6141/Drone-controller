# YK-250 — Araştırma raporu (faz 1)

**Tarih:** 5 Ekim 2026 · **Durum:** araştırma fazı tamamlandı, boyutlandırma fazına devredildi.

> **Kapsam.** YK-250, MALE/UCAV görünümlü, yaklaşık 250 cc pistonlu motorla uçan **sivil gözetleme ve araştırma
> İHA'sıdır**. Faydalı yük yalnızca EO/IR algılayıcılar ve görev donanımıdır. Silah, mühimmat, askı noktası, pilon
> ve bırakma mekanizması yoktur; bu konular araştırılmadı ve hiçbir dosyada kayıtlı değildir. "UCAV görünümü"
> yalnız biçim dilidir.

Bu rapor altı araştırma dosyasının özetidir. Her sayının kaynağı ve etiketi YAML dosyalarındadır:

| Dosya | İçerik |
|---|---|
| [`engine.yaml`](../data/research/engine.yaml) | 11 aday motor, önerilen motor, kurulum verileri |
| [`comparables.yaml`](../data/research/comparables.yaml) | 19 benzer İHA, istatistik, önerilen aralıklar |
| [`standards.yaml`](../data/research/standards.yaml) | 106 izlenebilir gereksinim, tasarım yükleri, SHT-İHA özeti |
| [`aero.yaml`](../data/research/aero.yaml), [`aero_polars.py`](../data/research/aero_polars.py), [`airfoils/`](../data/airfoils/) | 17 profil, NeuralFoil polarları, pervane çalışma noktaları |
| [`components.yaml`](../data/research/components.yaml) | 7 alt sistem kategorisi, her birinde önerilen ürün |
| [`materials.yaml`](../data/research/materials.yaml) | kompozit, çekirdek, yapıştırıcı, metal değerleri; süreç, bağlantı ve tolerans kuralları |
| [`baseline.yaml`](../data/research/baseline.yaml) | **boyutlandırma fazının başlangıç girdileri** (bu sentezin çıktısı) |

Etiketler: *manufacturer* (üretici), *datasheet* (veri sayfası), *standard* (standart/yönetmelik veya istatistiksel
tasarım değeri veritabanı), *paper* (rapor, tez, sektör dergisi), *vendor* (satıcı), *estimate* (tahmin; her
tahminin gerekçesi dosyada yazılıdır). Rapordaki sayılar tahmin olarak işaretlenmemişse kaynaktan alınmıştır.

---

## Özet

| Konu | Başlangıç değeri | Etiket |
|---|---|---|
| Motor | Limbach L 275 EF, 274 cm³, iki zamanlı boxer, yakıt enjeksiyonlu; 18 kW / 7500 rpm, kuru 7,0 kg | üretici |
| Kurulu motor grubu | 10,96 kg (aralık 10,0–12,5 kg), ePropelled SG750 marş/jeneratör ile | tahmin |
| Azami kalkış kütlesi | **145 kg** hedef; **149,9 kg kesin üst sınır** (150 kg, SHT-İHA'da M3 sınıfıdır) | tahmin / standart |
| Kanat | açıklık 6,0 m, alan 3,0 m², AR 12, sivrilme 0,5, kanat yüklemesi 48,3 kg/m² | tahmin |
| Yakıt | 31,9 kg (0,22 × MTOW) = 43,1 L; ~45 L esnek yakıt torbası | tahmin |
| Havada kalış | deniz seviyesinde ~10,9 h, 3000 m'de ~9,8 h (%10 yedekle) | tahmin |
| Kanat profili | NASA NLF(1)-0416 (kök %16, uç %13'e inceltilmiş) | rapor |
| Kuyruk profili | NACA 0012 | rapor |
| Pervane | Mejzlik 32×18 2 palalı karbon, itici; boyutlandırma için statik itki 460 N | üretici / tahmin |
| Tasarım yükleri | +3,8 / −1,52; rüzgâr hamlesi (VC'de 15,24 m/s) belirleyici: ~+4,6 / −2,6; emniyet katsayısı 1,5 | standart / tahmin |
| Kompozit sistemi | Solvay MTM45-1 otoklav dışı prepreg (fırın + vakum torbası), NCAMP B-basis ETW değerleri | standart |
| EO/IR | Trillium HD59-LLVV, 1,55 kg; bölme Ø0,18 × 0,23 m büyüme payıyla | veri sayfası |
| Kurtarma | Galaxy GRS 4/240 roketli paraşüt, 5,9 kg (yalnız acil durum) | üretici |
| Mevzuat | SHT-İHA (2026) M2 sınıfı: SHGM uçuşa elverişlilik belgesi, yazılım sertifikası, uzaktan kimlik, geofence | standart |

Sentez sırasında dosyalar birbiriyle karşılaştırıldı ve **11 düzeltme** yapıldı (bkz. §9). En önemlileri:
150 kg hedefinin M3 sınıfına düşmesi, pervane statik itkisinin 512 N'dan 500 N'a inmesi, bekleme devrinde jeneratör
gücünün sanılandan düşük olması ve iniş takımı stroğunun yetersiz olması.

---

## 1. Motor seçimi

### 1.1 Adaylar

200–300 cc aralığında satın alınabilen İHA motorları tarandı ([`engine.yaml`](../data/research/engine.yaml)).

| Motor | Hacim | Azami güç | Kuru kütle | Yakıt sistemi | Değerlendirme |
|---|---|---|---|---|---|
| **Limbach L 275 EF** | 274 cm³ | **18,0 kW** / 7500 rpm | **7,0 kg** | enjeksiyon, ECU irtifa düzeltmeli | **Önerilen** |
| Sky Power SP-275 FI TS ROS | 275 cm³ | 15,8 kW / 6000 rpm | 9,25 kg | enjeksiyon, çift buji | Yedek; kurulum verisi yok |
| Sky Power SP-275 TS ROS | 275 cm³ | 16,0 kW | 7,86 kg | karbüratör | İrtifa düzeltmesi yok |
| 3W-275Xi B2 R TS | 273 cm³ | 19,1 kW | 7,03 kg | karbüratör, çift buji | Ucuz deneme motoru; marş/jeneratör yok |
| Limbach L 275 E | 274 cm³ | 15,0 kW / 7200 rpm | 7,2 kg | karbüratör, manyeto | Kısmi yükte yakıt tüketimi %15–40 kötü |
| Sky Power SP-210 FI TS ROS | 210 cm³ | 14 kW | 9,4 kg | enjeksiyon | Üretici 60–100 kg için konumlandırıyor |
| Sky Power SP-180 SRE (Wankel) | 180 cm³ | 22 kW | 9,5 kg | sıvı soğutmalı | Büyüme/hibrit seçeneği |
| UEL AR741 (Wankel) | 208 cm³ | 28,3 kW | 10,7 kg | – | Sivil temin doğrulanamadı |
| Hirth 4201 | 183 cm³ | 11,1 kW | 7,14 kg | enjeksiyon | Güç yetersiz; kısmi yük ve irtifa referansı |
| DLE-222 / DA-200 | 222 / 200 cm³ | 16 / 14 kW | ~5 kg | karbüratör | Hobi motoru; seri üretim İHA'ya uygun değil |

### 1.2 Neden Limbach L 275 EF

* 275 cc sınıfının belgelenmiş en güçlü İHA motoru: azami 18 kW, kalkışta ~17 kW (6950 rpm), sürekli ~16,2 kW
  (6000 rpm, tahmin). Kaynak: [Limbach ürün sayfası](https://www.limflug.de/en/products/engines-15kw-40kw.php)
  (5 Ekim 2026'da yeniden okundu ve doğrulandı), [veri sayfası](https://limflug.de/downloads/datasheets/L275-EF-datasheet-en.pdf).
* Enjeksiyonlu seçenekler içinde en hafifi (7,0 kg çıplak).
* ePropelled SG750 marş/jeneratör 0,44 kg'da 800 W verir (3 dakika 1 kW) ve aynı zamanda marş motorudur
  ([UST 048](https://www.uncrewed-systems.com/?p=9652), doğrulandı).
* ECU basınç ve sıcaklığa göre karışımı düzeltir.
* Kamuya açık en iyi belgeler: tam gaz güç/tork/yakıt eğrisi ve kurulum ölçülerini veren bir mühendislik
  dosyası ([UST 018](https://www.ust-media.com/ust-magazine/UST018/61/)).

**Yedek motor:** Sky Power SP-275 FI TS ROS — çift buji (ateşleme yedekliliği), 100–150 kg MTOW için
konumlandırılmış, 1000'den fazla üretilmiş; ancak 2,25 kg daha ağır, daha az güçlü ve ölçü/yakıt verisi açık değil.

### 1.3 Kurulu kütle (SG750 ile)

| Kalem | kg | Etiket |
|---|---|---|
| Motor (çıplak) | 7,00 | üretici |
| ECU, kablo demeti, bobinler | 0,80 | tahmin |
| Yakıt pompası, regülatör, filtre | 0,35 | tahmin |
| SG750 marş/jeneratör | 0,44 | rapor |
| Jeneratör güç elektroniği | 0,40 | tahmin |
| Kaplin ve adaptör | 0,15 | tahmin |
| Susturuculu egzoz | 1,00 | tahmin |
| Titreşim yalıtımlı motor yatağı ve cıvatalar | 0,70 | tahmin |
| Gaz servosu, CHT/EGT algılayıcıları | 0,12 | tahmin |
| **Toplam** | **10,96** (10,0–12,5) | tahmin |

SG750 piyasaya çıkmazsa Limbach'ın 28 V 45 A alternatörü (5,2 kg) ve 12 V marş seti (2,3 kg) kullanılır; grup
17,6 kg olur (+6,6 kg).

### 1.4 Yakıt tüketimi

Limbach yalnız tam gaz eğrisi yayımlıyor. Kısmi yük değerleri tahmindir (±%12).

| Güç oranı (18 kW'a göre) | Mil gücü | Özgül tüketim | Yakıt akışı | Etiket |
|---|---|---|---|---|
| 1,00 | 18,0 kW | 430 g/kWh | 7,74 kg/h (10,5 L/h) | tahmin (veri sayfası eğrisi) |
| 0,75 | 13,5 kW | 460 g/kWh | 6,21 kg/h | tahmin |
| 0,50 | 9,0 kW | 470 g/kWh | 4,23 kg/h | tahmin |
| 0,30 | 5,4 kW | 540 g/kWh | 2,92 kg/h | tahmin |
| 0,20 | 3,6 kW | 600 g/kWh | 2,16 kg/h | **dışdeğerleme** (bu sentezde eklendi) |

Bekleme uçuşu (145 kg, 30 m/s) jeneratör yüküyle birlikte ~4,4 kW mil gücü ister; bu, %24,6 güç oranıdır ve tablodaki
en düşük noktanın (%30) altındadır. Bu yüzden 0,20 noktası eklendi ve açıkça işaretlendi (düzeltme C-ENG-02).
Yakıt yoğunluğu her yerde 740 kg/m³ alınır (benzin + %2 sentetik yağ).

### 1.5 İrtifa, kurulum, soğutma, titreşim

* **İrtifa:** Hirth 4201 eğrisinden uydurulan P/P₀ = σ^1,23 kullanılır (tahmin): 3000 m'de 12,5 kW, 4500 m'de 10,3 kW.
* **Zarf:** 0,221 (boy) × 0,397 (genişlik, buji başlıkları dahil) × 0,295 m (yükseklik). Silindirler yataydır;
  MALE görünümlü bir gövdenin arka kısmı 0,397 m genişliği almalı ya da yanak kabarcıkları/hava girişleri gerekir.
* **Yatak:** karter arka yüzünde 4 × M8 dişli boğa. Delik konumları veri sayfasında okunamıyor; dosyadaki desen
  yalnızca yerleşim içindir (±3 mm), delmek için kullanılamaz.
* **Soğutma:** azami güçte ~0,42 kg/s hava (tahmin); ilk giriş alanı 0,031 m², çıkış 0,040 m². İtici düzende
  silindirlere pervane rüzgârı gelmez; yerde çalıştırma süresi CHT ile sınırlanmalı ya da yer fanı kullanılmalı.
  Sürekli CHT sınırı 240 °C (tahmin; Limbach yayımlamıyor).
* **Egzoz:** gaz sıcaklığı tasarım değeri 750 °C; kompozit parçalar borudan 25–50 mm uzak tutulmalı ya da ısı
  kalkanı kullanılmalı.
* **Titreşim:** iki silindir aynı anda ateşlenir. 5000 rpm'de 1. derece 83 Hz, 2 palalı pervane geçişi 167 Hz.
  Motor yatağının rijit cisim modları ≤ 20 Hz olmalı; EO/IR taret yalıtıcısının modları 80–120 Hz ve 160–240 Hz
  bantlarından uzak tutulmalı.

### 1.6 Motorla ilgili açık sorular

Limbach'tan istenecekler: STEP/DXF kurulum çizimi (yatak deseni, pervane göbeği), TBO, CHT/EGT sınırları,
rölanti devri, sürekli güç değeri, itici için ters dönüş, silindir yönü serbestliği, fiyat. Veri sayfasındaki
"kuru kütle yaklaşık 15 kg" ifadesi büyük olasılıkla alternatör ve marş setini içeriyor; doğrulanmalı.
Planlama TBO değeri 500 h (Hirth benzeri motorlardan, tahmin).

---

## 2. Benzer İHA'lar ve hedef aralıklar

### 2.1 Güç sınıfı (12–40 kW) karşılaştırması

19 sabit kanatlı pistonlu İHA incelendi; 16'sı istatistiğe girdi, bunların 10'u 12–40 kW sınıfındadır
([`comparables.yaml`](../data/research/comparables.yaml)).

| Uçak | MTOW (kg) | Güç (kW) | kg/kW | Açıklık (m) | Havada kalış (h) | Düzen |
|---|---|---|---|---|---|---|
| Shadow 200 RQ-7B | 170 | 28,3 | 6,0 | 4,27 | 7 | itici, ikiz kiriş |
| Shadow 600 | 264 | 38,8 | 6,8 | 6,83 | 12 | itici, ikiz kiriş |
| Pioneer RQ-2A | 205 | 19,4 | 10,6 | 5,15 | 5 | itici, ikiz kiriş |
| IAI Scout | 159 | 16,4 | 9,7 | 4,96 | 7,5 | itici, ikiz kiriş |
| RUAG Ranger | 285 | 31,5 | 9,0 | 5,71 | 9 | itici, kızaklı |
| Aerostar | 220 | 28,0 | 7,9 | 7,5 | 12 | itici, ikiz kiriş |
| BAE Phoenix | 175 | 19,0 | 9,2 | 5,5 | 4 | çekici |
| **Primoco One 150** | **150** | **18,4** | **8,2** | 4,85 | **10** | itici |
| Hermes 180 | 195 | 28,3 | 6,9 | 6,0 | 10 | – |
| TAI Gözcü | 85 | 28,0 | 3,0 | 3,75 | 2 | itici, delta + V |

Primoco One 150 en yakın örnektir: 150 kg, 1–30 kg faydalı yük, 10 h, 120 km/h seyir, 3300 m tavan
([üretici sayfası](https://uav-stol.com/?p=145), 5 Ekim 2026'da yeniden okundu).

### 2.2 İstatistik

* Güç yüklemesi: medyan 8,0 kg/kW (çeyrekler arası 6,8–9,2). Uydurma: MTOW = 25,1·P^0,615 → 18–20 kW'ta 148–158 kg.
* Boş kütle oranı medyanı 0,60; faydalı yük oranı 0,21; yakıt oranı 0,20 (çoğu kütle dengesinden tahmin).
* Uzun havada kalışlı tiplerde açıklık indeksi b/m^(1/3) ≈ 1,10; 145–150 kg'da 5,5–6,5 m eder.
* Düzen sayımı: düzeni bilinen 13 uçağın 11'i itici. 19 uçağın hiçbiri içeri katlanan iniş takımı kullanmıyor.
  Hepsi kompozit.
* Kanat alanı yalnız iki uçakta yayımlanmış (Pioneer, Ranger); kanat yüklemesi ve AR istatistiği ±%15–20 belirsiz.

### 2.3 Hedef aralıklar

| Büyüklük | İstatistiksel öneri | Başlangıç değeri (baseline) | Not |
|---|---|---|---|
| MTOW | 125–180 kg, nominal 150 | **145 kg**, üst sınır **149,9 kg** | 150 kg M3 sınıfıdır (C-COMP-01) |
| Açıklık | 5,5–6,8 m | 6,0 m | 6,8 m: taşınabilir kanat paneli sınırı |
| Kanat alanı | 2,5–3,6 m² | 3,0 m² | ortalama veter 0,50 m |
| Kanat yüklemesi | 45–60 kg/m² | 48,3 kg/m² | flapsız CLmax 1,4 ile tutunma ≤ 24 m/s için ≤ 50,4 |
| Açıklık oranı | 10–13 | 12 | |
| Yakıt oranı | 0,18–0,26 | 0,22 | 0,26 için 45 L torba yetmez |
| Faydalı yük | 20–35 kg | 20 kg tasarım değeri | EO/IR seti yalnız ~3,3 kg (§6) |
| Boş kütle oranı | 0,55–0,62 | 0,60 (87 kg) | |
| Gövde boyu | 3,0–3,8 m | 3,4 m | |
| Seyir hızı | 25–36 m/s | 30–36 m/s | |
| Servis tavanı | 3600–5500 m | 4500 m | |

Kütle dengesi (145 kg): boş 87,0 + yakıt 31,9 + faydalı yük 20,0 + **büyüme payı 6,1 kg** (%4,2); 149,9 kg sınırına
kadar 4,9 kg daha vardır.

---

## 3. Standartlar ve tasarım yükleri

### 3.1 Okunan belgeler

| Belge | Kapsam | Kullanım |
|---|---|---|
| [JARUS CS-LUAS Ed. 0.3 (2016)](http://jarus-rpas.org/wp-content/uploads/2023/06/jar_07_doc_CS_LUAS.pdf) | sabit kanatlı RPA ≤ 750 kg | **SHGM'ye önerilecek sertifikasyon tabanı** |
| [STANAG 4703 / AEP-83 (2014 taslağı)](https://assets.publishing.service.gov.uk/government/uploads/system/uploads/attachment_data/file/391827/20140916-STANAG-4703_AEP-83_A__1_.pdf) | hafif İHA ≤ 150 kg | çapraz kontrol; İHA'ya özgü katsayılar |
| [EASA CS-VLA Amdt 1](https://www.easa.europa.eu/en/downloads/66874/en) | insanlı ≤ 750 kg | formüllerin kaynağı (hamle, iniş, motor torku) |
| [FAR 23 (2017-08-29 hali)](https://www.ecfr.gov/on/2017-08-29/title-14/chapter-I/subchapter-C/part-23) | insanlı | jiroskopik yatak yükü, depo testleri |
| [EASA SC Light-UAS](https://www.easa.europa.eu/sites/default/files/dfu/special_condition_sc_light-uas_medium_risk_01.pdf) | amaç tabanlı | sayısal değer içermez |
| ASTM F3298-24, EUROCAE ED-325 | – | ücretli, okunmadı |

Kodların çoğu aynı kökten gelir; ayrıştıkları yerde en tutucu değer seçildi. Not: STANAG 4703 askeri kökenlidir;
yalnız yapı, motor, yakıt ve elektrik tasarım içeriği mühendislik referansı olarak kullanıldı.

### 3.2 Tasarım yükleri (limit değerler)

| Yük | Değer | Kaynak |
|---|---|---|
| Manevra yük katsayısı | +3,8 / −1,52 (VC'de), VD'de 0'a doğrusal | CS-LUAS.337, .333 |
| Flaplı manevra | +2,0 | CS-LUAS.345 |
| Dikey/yanal rüzgâr hamlesi | VC'de 15,24 m/s, VD ve VF'de 7,62 m/s (6096 m'ye kadar tam) | CS-LUAS.333(c) |
| Emniyet katsayısı | 1,5 | CS-LUAS.303 |
| Bağlantı elemanı (fitting) katsayısı | 1,15 | CS-LUAS.625 |
| Döküm katsayısı | 2,0 (radyografiyle 1,25) | CS-LUAS.621 |
| Dönen/pimli bağlantı yatak katsayısı | 2,0 | STANAG UL2.4 |
| Sık sökülen bağlantılar | 1,5 | STANAG UL2.4 |
| Menteşe yatağı toplam katsayısı | 6,67 | CS-LUAS.657 |
| Kumanda sistemi yükü | hesaplanan menteşe momentinin 1,25 katı | CS-LUAS.395 |
| Çırpınma (flutter) payı | 1,2 VD; kumandası kopmuş yüzey VD'ye kadar çırpınmasız | CS-LUAS.629 |
| Acil iniş (nihai) | ileri 9, yukarı 3, yan 1,5, aşağı 6 g | CS-VLA 561, FAR 23.561 |

**Önerilen tasarım hızları (tahmin, boyutlandırma kesinleştirecek):** VS (temiz) 23,5 m/s, VS (flaplı) 20,7 m/s,
VA = VC = 45 m/s EAS, VD = 57 m/s EAS, VF ≥ 37,3 m/s. VD, tahmini tam gaz düz uçuş hızından (55–60 m/s) küçük
olmamalıdır; daha düşük VC/VD (40/50) yalnız uçuş kontrol sistemi aşırı hızı (sıkışmış gaz kolu dahil)
engelliyorsa kullanılabilir.

**Rüzgâr hamlesi belirleyicidir.** 145 kg, 3,0 m², ortalama veter 0,50 m, CLα 5,10 /rad ile (CS-VLA 341 formülü):

| İrtifa | μ | Kg | VC'de n (45 m/s) | VD'de n (57 m/s) |
|---|---|---|---|---|
| Deniz seviyesi | 31,0 | 0,751 | +4,40 / −2,40 | +3,15 / −1,15 |
| 3000 m | 41,7 | 0,781 | +4,53 / −2,53 | +3,23 / −1,23 |
| 4500 m | 48,8 | 0,794 | +4,59 / −2,59 | +3,27 / −1,27 |

Kanat, manevra değeri 3,8 yerine yaklaşık +4,6 / −2,6 limit (nihai +6,9 / −3,9) yüke göre boyutlanacaktır.
Boyutlandırma her ağırlık ve irtifada hamle hesabı yapmalıdır. `analysis/structlib.vn_diagram` negatif katsayıyı
VD'ye kadar −1,52 tutuyor; bu tutucu tercih raporda belirtilmeli ya da kodlardaki doğrusal azalma uygulanmalıdır.

### 3.3 İniş yükleri

* Çökme hızı V = 0,51·(Mg/S)^0,25 → **2,38 m/s**; düşürme yüksekliği 0,287 m; iniş anında kanat kaldırması 2/3 W.
* Hedef: atalet yük katsayısı ≤ 4,0 (en fazla 4,47; bunun üstünde motor, depo, faydalı yük ve paraşüt bağlantıları
  manevra yerine iniş yüküyle boyutlanır, STANAG Annex B 1.2).
* Gereken etkin çökme (lastik + yay):

| Bacak tipi (verim) | n ≤ 4,47 için | n ≤ 4,0 için |
|---|---|---|
| GFRP yay bacak (0,5) | 0,183 m | 0,216 m |
| Sönümlü bacak (0,65) | 0,134 m | 0,157 m |
| Yağlı-havalı amortisör (0,8) | 0,106 m | 0,123 m |

[`components.yaml`](../data/research/components.yaml)'daki yay bacak 0,12–0,13 m çökmeyle n ≈ 6,1 verir; bu yüzden
strok artırılmalı ya da sönümleyici eklenmelidir (düzeltme C-CMP-03).

### 3.4 Motor yatağı

* Tork katsayısı **6** (tahmin): iki silindir aynı anda ateşlendiği için tek silindirli iki zamanlı gibi
  davranır (CS-VLA 361). Kodların harfi 3 (CS-VLA) veya 4 (CS-LUAS) der; aradaki kütle farkı küçüktür.
* Limit tork: sürekli güçte 25,8 N·m × 6 = **154,7 N·m** (uçuş durumu A yükleriyle birlikte); kalkışta 141,4 N·m.
* Yan yük 1,47 g; jiroskopik durum (sapma 2,5 rad/s, yunuslama 1,0 rad/s, n = 2,5); itici motor için ileri
  15 g nihai tutma (tahmin, CS-VLA 561(c)).

### 3.5 Kompozit katsayıları

* Tek yük yolu için A-basis, yedekli yapı için B-basis değer (CS-LUAS.613).
* Nem şartlandırılmış sıcak test (ETW) değerleriyle özel katsayı **1,2** → toplam **1,8**. Yalnız oda sıcaklığı
  kuru (RTD) veri varsa CS-LUAS AMC'si 2,0 × 1,2 = 2,4 → toplam **3,6** ister.
* Azami yapı sıcaklığı açık gri boyayla 70 °C (koyu boyada ~84 °C); ıslak Tg ≥ 98 °C olmalı. Dış boya açık renk
  (güneş soğurganlığı ≤ 0,6) seçilmelidir.

### 3.6 Yakıt sistemi kuralları

Pompa debisi kalkış tüketiminin 1,25 katı; %2 genleşme boşluğu; 300–600 göz/m süzgeç, çıkış alanının ≥ 5 katı;
2 s negatif g besleme; %1 eğimde yakıt kaçırmayan havalandırma; ateş duvarı açıklığı 13 mm; depo test basıncı
max(24 kPa, nihai ivmedeki basınç); 43 °C sıcak yakıt testi.

---

## 4. Türkiye mevzuatı

Esas belge **SHGM SHT-İHA (2026)**: 30 Temmuz 2026'da yürürlüğe girdi, 2016 talimatını kaldırdı; geçiş süresi
31 Temmuz 2027'de biter ([resmî PDF](https://web.shgm.gov.tr/documents/sivilhavacilik/files/mevzuat/sektorel/talimatlar/SHT-IHA(1).pdf),
5 Ekim 2026'da yeniden okundu).

| Konu | Hüküm | Madde |
|---|---|---|
| Sınıf | M2: 25 kg (dahil) – 150 kg; **M3: 150 kg (dahil) ve üstü** | 8(1) |
| Uçuşa elverişlilik | M2 ve M3 için SHGM'nin düzenleyeceği uçuşa elverişlilik belgesi; teknik gereklilikler henüz yayımlanmadı | 8(5) |
| Yazılım | Kayda tabi İHA'lar SHGM yazılım sertifikasyon testlerini geçmeli (uçuş kontrol ve yer istasyonu yazılımı) | 8(6), 23(4) |
| Uyum modülü | Yer istasyonunda İHATTYS komutlarını (zarf daraltma, rota değişikliği, iniş, uçuş sonlandırma) gecikmesiz uygulayan gömülü yazılım | 23(3) |
| Zorunlu işlevler | uzaktan kimlik, geofence, C2 kaybında önceden belirlenmiş alana otomatik dönüş veya uçuş sonlandırma, BTK uyumlu telsizler | 18(1), 23 |
| Tescil | 120 m yükseklik ve/veya 3000 m yatay uzaklığın ötesinde P2 lisansıyla uçulacak İHA'lar için zorunlu | 11(2) |
| Pilot | P2 lisansı, en az 3. sınıf sağlık sertifikası; P2 uçuşları bir işletici bünyesinde yapılır | 15, 19 |

**Tasarıma etkisi:**

* MTOM kesin olarak 150 kg'ın altında tutulur; spec.yaml'da üst sınır 149,9 kg'dır.
* SHGM ile erken bir toplantıda sertifikasyon tabanı olarak riske göre uyarlanmış CS-LUAS (STANAG 4703 atıflarıyla)
  önerilmelidir; talimat teknik kod belirtmiyor.
* Aviyonik uzaktan kimlik, İHATTYS beslemeli geofence, uyum yazılım modülü ve C2 kaybı kurtarma/sonlandırma
  işlevlerini taşımalıdır. Otopilot ve yer istasyonu yazılımının SHGM sertifikasyonu için zaman ve bütçe ayrılmalıdır.
* Gözetleme görevleri (120 m üstü, görüş hattı ötesi) tescil, P2 pilot ve İHATTYS uçuş bildirimi gerektirir.

Basında "150 kg ve üstü sicile girer" şeklinde özetler var; resmî metin (Madde 11(2)) tescili uçuş zarfına bağlar.
Resmî metin esas alındı.

---

## 5. Profil ve pervane

### 5.1 Kanat ve kuyruk profili

13 kanat ve 4 kuyruk profili NeuralFoil ("large" model) ile Re 0,8 / 1,5 / 2,5 milyonda, temiz (n_crit 9),
n_crit 5 ve geçiş 0,075c'de zorlanmış (böcek/yağmur/aşınma) durumlarında hesaplandı
([`aero.yaml`](../data/research/aero.yaml), [`aero_polars.py`](../data/research/aero_polars.py)).

**Seçim:**

| Yer | Profil | Gerekçe |
|---|---|---|
| Kanat kökü | NASA NLF(1)-0416 (%16) | kirli hücum kenarında cl_max düşmüyor ([NASA TP-1861](https://ntrs.nasa.gov/api/citations/19810015487/downloads/19810015487.pdf)); firar kenarından yumuşak tutunma kaybı; zorlanmış geçişte setin en iyi dayanım göstergesi (cl^1,5/cd = 117,6); cm ≈ −0,10; kökte ~106 mm kiriş derinliği |
| Kanat ucu | aynı profil %13'e inceltilmiş (`oml.resampled('nlf416', thickness_scale=0.8125)`) | aynı orta çizgi → sıfır kaldırma açısı ve cm sabit; 2–3° burulma (washout) |
| Kuyruk | NACA 0012 | kuyruk Re'sinde en yüksek cl_max (1,26–1,34) ve en yumuşak tutunma kaybı; kiriş, menteşe ve servo için derinlik |

Elenenler: NACA 23015 (çok ani tutunma kaybı, NACA Rapor 824), Wortmann FX 63-137 (cm −0,21), NLF(1)-1015
(pürüzlülükte büyük sürükleme cezası), 6-serisi profiller (sürükleme kovası cl ≈ 0,7'de bitiyor).

**Doğrulama ve düzeltme katsayıları:** NeuralFoil NLF(1)-0416 cl_max değerini rüzgâr tünelinden %6 yüksek
(1,80'e karşı 1,69) buluyor; LS(1)-0417'de zorlanmış geçiş sürüklemesini %11–17 düşük buluyor. Bu yüzden
boyutlandırmada:

* kesit cl_max × **0,94**,
* zorlanmış geçiş polarları × **1,15** (dayanım ve tırmanma bu polarlarla hesaplanır; temiz polar yalnız artı
  olasılıktır),
* hücum kenarının ilk %8–10'una aşınma bandı, yağmurda cm'de +0,01 kayma için yükseklik dümeni payı.

Re 1,5 milyonda NLF(1)-0416: zorlanmış geçişte cd(cl 0,9) = 0,0123 → düzeltilmiş 0,0141; düzeltilmiş cl_max 1,61.
Uçak CLmax başlangıç değerleri (tahmin): temiz, trimli 1,40; flaplı 1,80.

**Dikkat:** UIUC'den indirilen `naca4415.dat` ve `naca0010.dat` artık `design/oml.py`'deki analitik NACA
üretecinin önüne geçiyor (UIUC 4415 dosyası NACA denklemlerinden hücum kenarında sapıyor). Başlangıç tasarımı
bu profilleri kullanmıyor; ileride kullanılırsa dosyalar yeniden adlandırılmalı ya da silinmelidir.

### 5.2 Pervane

Mejzlik'in yayımladığı itki/tork tabloları Limbach L 275 EF'in tam gaz tork eğrisiyle eşleştirildi.

| | **Mejzlik 32×18 2B (önerilen)** | Mejzlik 31×12 3B (alternatif) |
|---|---|---|
| Çap / adım | 0,813 / 0,457 m | 0,787 / 0,305 m |
| Kütle | 0,334 kg (CW ve CCW var) | 0,412 kg |
| Statik itki (motor dengesinde) | **500 N** @ 5790 rpm, uç Mach 0,72 | 550 N @ 5960 rpm |
| Boyutlandırma için statik itki | **460 N** (Falcon ölçüm/tablo oranı 0,92) | – |
| 30 m/s seyirde | 3802 rpm, verim 0,786 | 4453 rpm, verim 0,746 |
| 35 m/s seyirde | 4174 rpm, verim 0,813 | 4945 rpm, verim 0,769 |
| 40 m/s seyirde | 4580 rpm, verim 0,806 | 5477 rpm, verim 0,753 |

Bu verimler yalıtılmış pervane içindir; gövde arkasındaki itici için 0,93–0,97 kurulum katsayısı uygulanır.
Kalkışta itki/ağırlık 460 / (145 × 9,81) = **0,32**.

**Düzeltme (C-AERO-01):** `aero.yaml` statik itkiyi 512 N / 6000 rpm veriyordu. Mejzlik 0161 sayfası yeniden
okundu: 5500 rpm statik satırı 451 N, 23,15 N·m. Yazılan yöntem (bu satırın n² ile ölçeklenmesi) motorla
5790 rpm'de 500 N dengesine gider; betiğin kübik eğri uydurması tablo dışına çıkarken torku %7 düşük verdiği için
512 N çıkmıştı. Elle yazılmış değerler düzeltildi; betikteki dışdeğerleme ayrıca düzeltilip yeniden çalıştırılmalıdır.

**Dikkat:** 32×18 ile 30 m/s deniz seviyesi beklemesi ~3800 rpm'dir; Limbach'ın 4000–6000 rpm olağan uçuş bandının
hemen altındadır ve jeneratör gücünü düşürür (§6.2). Bu sorun büyürse 31×12 3B daha uygun olabilir.

Pervane göbek deseni (Limbach) bilinmeden delikli pervane sipariş edilmemelidir. Yer açıklığı ≥ 0,18 m
(CS-VLA 925): kritik duruşta pervane göbeği yerden en az 0,59 m yüksekte olmalıdır.

---

## 6. Alt sistemler

### 6.1 Önerilen ürünler

| Kategori | Önerilen | Kütle | Güç / not | Etiket |
|---|---|---|---|---|
| Kumanda eyleyicileri | Volz DA 26 (28 V, RS-485) × 6: kanatçıklar, kuyruk yüzeyleri, burun teker yönlendirme, fren; Volz DA 30 × 2: flaplar | 3,28 kg kurulu | DA 26: 2,7 N·m anma / 5,0 N·m tepe, 0,27 kg ([veri sayfası](https://www.volz-servos.com/fileadmin/user_upload/Downloads/Datasheets/DA-26_Datasheet_uni.pdf), doğrulandı) | veri sayfası / tahmin |
| İniş takımı | sabit üç tekerli; GFRP yay ana bacak (kendi üretim), TOST 3,5 in diskli fren tekerleri, 200×50 lastik; DLR SAGITTA (150 kg İHA) örneği | ~9,6 kg | lastik hız sınırı 36 m/s; strok artırılmalı (§3.3) | tahmin |
| EO/IR taret | Trillium HD59-LLVV (EO zoom + LWIR 640×512) | 1,55 kg | 20 W ort. / 100 W tepe, 24 V, Ethernet video ([veri sayfası](https://www.trilliumeng.com/uploads/documents/Trillium-HD59-2026.pdf), doğrulandı); bölme Octopus E180 için (Ø0,18 × 0,23 m, 4,0 kg) | veri sayfası |
| Otopilot | Embention Veronte Autopilot 1x (ADS-B), DO-178C/DO-254, ayrık denetleyici ve FTS mantığı | 0,21 kg | prototip ve demir kuş için Cube Orange+ / ArduPilot | üretici |
| Hava verisi, GNSS | 28 V ısıtmalı pitot + PowerBox Pitot HASA; 2 × Calian HC977EXF | – | ısıtıcı 42 W | veri sayfası |
| Veri bağı | Silvus StreamCaster 4200 EP (2×2 MIMO, C2 + IP video); yedek C2 Microhard pDDL2450 | – | bant BTK tahsisine göre | veri sayfası |
| Transponder, uzaktan kimlik | uAvionix ping200X (Mode S ES); Dronetag Beacon | 0,05 / 0,016 kg | SHGM uzaktan kimlik biçimi doğrulanacak | veri sayfası |
| Elektrik | SG750 + iPS sınıfı kontrolcü → 48 V sınıfı bara; 14S2P LiFePO4 tampon batarya (231 Wh); VISIONAIRtronics 1000 W PDU (yedekli 28 V servo ve aviyonik hatları) | 4,53 kg | yedek: Limbach 28 V alternatör | tahmin / üretici |
| Yakıt | ATL 0,25 mm esnek torba ~45 L + ~0,3 L toplama deposu; Limbach EFI pompası, Andair gazkolatör, AN6 kuru kesme dolum bağlantısı, kapasitif seviye | 1,73 kg | | tahmin |
| Kurtarma ve güvenlik | Galaxy GRS 4/240 roketli paraşüt; bağımsız uçuş sonlandırma kanalı; 3 × AveoFlash ışık | 6,3 kg | 240 kg / 240 km/h, açılma şoku 13,1 kN, 43 m², iniş 6,9 m/s ([üretici](https://galaxysky.cz/grs-4-240-40m2-p30-en), doğrulandı) | üretici |

Paraşüt yalnız acil durum içindir; açılmadan önce motor durdurulmalı, kanopi itici pervane diskinin önünden
çıkmalıdır. 145 kg'da açılma şoku ~9,2 g'dir; özel bir sert bağlantı noktası gerekir.

### 6.2 Elektrik gücü payı (düzeltme C-ENG-01, C-CMP-01/02)

SG750'nin 800 W değeri 7500 rpm içindir ("800 W over 50 V AC at 7500 rpm", kaynak yeniden okundu). Kalıcı mıknatıslı
jeneratörde güç ve gerilim kabaca devirle orantılıdır (tahmin):

| Durum | Devir | Jeneratör çıkışı | Doğrultulmuş gerilim |
|---|---|---|---|
| Tam güç | 7500 rpm | 800 W | ~67 V |
| Seyir | 5200 rpm | ~550 W | ~47 V |
| Bekleme, 32×18 | 3800 rpm | **~405 W** | **~34 V** |
| Bekleme, 31×12 3B | 4450 rpm | ~475 W | ~40 V |

Yük: sürekli 303 W (bileşenler 273 W + görev bilgisayarı 30 W), HD59 tepesiyle 383 W, E180 büyüme taretiyle 443 W.
Bekleme devrinde pay incedir ve 14S LiFePO4 şarj gerilimi için kontrolcünün gerilim yükseltmesi gerekir.
Çözüm seçenekleri: ePropelled'den devir–güç eğrisini almak, beklemeyi ≥ 4400 rpm'de yapmak (31×12 3B veya daha düşük
adımlı pervane), pitot ısıtıcısını çevrimli çalıştırmak, gerekirse 28 V alternatör yedeğine geçmek.

### 6.3 Kütle toplamı ve yapı bütçesi

| Grup | kg |
|---|---|
| Kurulu motor grubu | 10,96 |
| Pervane + göbek adaptörü/spinner | 0,33 + 0,40 |
| Kaporta, soğutma kanalları, ateş duvarı | 1,50 (tahmin) |
| Yakıt sistemi | 1,73 |
| İniş takımı | 9,60 |
| Kumanda eyleyicileri | 3,28 |
| Aviyonik | 0,62 |
| Elektrik | 4,53 |
| Kablo demeti, konnektörler, koaksiyel | 2,50 (tahmin) |
| Kurtarma ve güvenlik | 6,30 |
| **Sistemler toplamı** | **41,75** |
| Boş kütle hedefi (0,60 × 145) | 87,0 |
| **Yapıya kalan bütçe** | **45,25 kg (%31 MTOW)** |

Faydalı yük seti: HD59 taret 1,55 + yalıtımlı bağlantı 0,20 + görev bilgisayarı/kayıt cihazı 1,0 (tahmin, ürün
seçilmedi) + tepsi/kablo 0,5 = **3,25 kg**. 20 kg tasarım faydalı yükünün kalan 16,75 kg'ı araştırma yükleri
için ayrılmıştır (örnek: RIEGL miniVUX-3UAV lazer tarayıcı 1,55 kg). Kullanılmayan pay, depo hacmi izin verirse
yakıta çevrilebilir.

---

## 7. Malzeme, süreç ve bağlantı kuralları

### 7.1 Kompozitler

Ana sistem **Solvay MTM45-1** otoklav dışı prepreg: fırında vakum torbasıyla 121 °C kür + 177 °C serbest son kür.
Aynı reçine, kür ve Tg ile üç kumaşın NCAMP B-basis verisi açıktır. ETW koşulu 93 °C (200 °F) nemli test sıcaklığıdır
(NCAMP raporu yeniden okunarak doğrulandı); 84 °C'lik koyu yüzey sıcaklığını bile kapsar.

| Malzeme (B-basis ETW) | Yoğunluk | E₁ | Çekme | Basma | Kesme* | Katman kalınlığı | Islak Tg |
|---|---|---|---|---|---|---|---|
| CFRP UD bant AS4-145 | 1542 kg/m³ | 127,6 GPa | 1557 MPa | 772 MPa | 25,6 MPa | 0,140 mm | 170,8 °C |
| CFRP 3K düz örgü AS4 193 g/m² | 1517 kg/m³ | 64,2 GPa | 714 MPa | 284 MPa | 19,2 MPa | 0,201 mm | 156,3 °C |
| GFRP 7781 E-cam | 1813 kg/m³ | 24,9 GPa | 236 MPa | 243 MPa | 14,6 MPa | 0,254 mm | 160 °C |

\* Kesme değerleri %0,2 kaymalı düzlem içi kesmedir (tutucu).

Delikli ve cıvatalı karbon yapı için **yarı izotrop laminat değerleri** kullanılır (katman değerleri değil):
açık delik çekme **312 MPa**, açık delik basma **175 MPa**, yatak (%2 kayma, e/D 3) **416 MPa**, E ≈ 43,4 GPa.
Toplam katsayı 1,5 × 1,2 = **1,8** bu değerlere uygulanır.

**Tek yük yolu:** CS-LUAS A-basis ister. NCAMP raporundaki A/B oranları (UD) 0,86–0,92'dir; A-basis değer üretilene
kadar tek yük yollu parçalarda (tek parça kiriş geçişi, kanat bağlantı kulağı) B değerleri **0,85** ile çarpılmalı ya
da bu parçalar yedekli yük yoluyla tasarlanmalıdır (tahmin).

Islak yatırma ve infüzyon değerleri tahmindir; birincil yapıda kullanmadan önce 3 partili kupon programı gerekir.
NCAMP paylaşımlı değerleri için üretim sürecinde eşdeğerlik testi yapılmalıdır.

**Çekirdek ve yapıştırıcılar:** kanat/kuyruk sandviçinde Nomex HRH-10-3.2-48 (film yapıştırıcıyla); Rohacell WF
130 °C ile sınırlı olduğundan WF-HT kullanılmalı ya da son kür çekirdek yapıştırmadan önce yapılmalıdır. EA 9394
(177 °C'ye kadar) dolgu ve motor bölmesi yapıştırmaları için; DP490 80 °C altı, Araldite 2015-1 60 °C altı.

### 7.2 Metaller (MIL-HDBK-5G, A-basis min(L, LT))

| Malzeme | Kullanım | Ftu | Fty | Fbru (e/D 2) |
|---|---|---|---|---|
| Al 7075-T651 levha | işlenmiş bağlantı parçaları | 531 MPa | 462 MPa | 1000 MPa |
| Al 6061-T6 sac | bükülmüş braketler | 290 MPa | 241 MPa | 607 MPa |
| 4130 normalize boru | motor yatağı, takım çerçeveleri | 655 MPa (kaynak yakını **552 MPa**) | 517 MPa | 1379 MPa |
| Ti-6Al-4V tavlanmış | CFRP'deki pim ve bağlantılar | 924 MPa | 869 MPa | 1875 MPa |
| 304 paslanmaz | ateş duvarı, egzoz kalkanı | 503 MPa | 179 MPa | – |

### 7.3 Süreç kuralları

* **Prepreg:** en ince laminat 0,6 mm (≥ 3 kumaş katı); kalıp açısı 2° (1–3°); iç köşe yarıçapı dişi kalıpta 3 mm,
  erkek kalıpta 1,5 mm, UD bantta 5 mm.
* **Dizilim:** simetrik ve dengeli; dış katlar ±45°; aynı yönde en fazla 4 ardışık kat (0,51 mm); her yönde
  (0/+45/−45/90) en az %10; kat bırakma eğimi yük yönünde 1:20, diğer yönlerde 1:10; bağlantı bölgelerinde ≥ %40 ±45°.
* **Sandviç:** çekirdek rampası ≤ 30°; ek parça veya bağlantı taşıyan her serbest kenar kapatılır; çıplak çekirdek
  üzerinden asla sıkılmaz. Gömme insertler ESA ECSS-E-HB-32-22A formülleriyle.
* **Yapıştırma:** Hart-Smith yöntemi; çift bindirmede bindirme ≥ 30 t; birincil yükte tek bindirmeden kaçınılır;
  yapıştırıcı kalınlığı 0,10–0,30 mm.
* **CNC:** yapısal alüminyumda en ince duvar 1,5 mm, iç köşe ≥ 3 mm; genel tolerans ISO 2768-mK, geçmelerde ±0,025 mm.
* **Sac:** AC 43.13-1B Tablo 4-6'nın üst değerleri (R/t: 5052-H32 1,5; 6061-T6 3,0; 2024-T3 6,0; 7075-T6 7,0);
  delik–büküm mesafesi 2,5 t + R.
* **4130 kaynağı:** TIG, AMS 6457 dolgu, en ince et 0,89 mm, kaynak yakını tasarım değeri 552 MPa.
* **Eklemeli imalat:** SLS PA12 ve FDM PA-CF yalnız yapısal olmayan/ikincil parçalarda, veri sayfası dayanımının
  ~0,34 katıyla; birincil yapı, kumanda yük yolu, motor yatağı, takım, yakıt sistemi ve egzoz bölgesinde yasak.
* **Geçmeler:** menteşe pimleri H7/g6; pimler ve merkezleme H7/h6; metalde burç H7/p6; CFRP'de burç sıkı geçme yerine
  0,05–0,15 mm boşlukla EA 9394 ile yapıştırılır.

### 7.4 Bağlantı elemanı kuralları

* Kenar mesafesi: metalde ≥ 2,0 D; kompozitte birincil bağlantıda ≥ 2,5 D (+1,3 mm delik konum toleransı), tam yatak
  dayanımı için 3,0 D; ikincil bağlantıda en az 2,0 D.
* Aralık: kompozitte sıralar arası ≥ 4 D, bağlantı başına genişlik ≥ 6 D. Hızlı kilit (çeyrek tur) aralığı
  kaportada 60–90 mm, kaplamalarda 100–150 mm.
* CFRP içinde yalnız titanyum, A286 veya paslanmaz bağlantı elemanı; alüminyum/kadmiyum/çinko kaplı parça karbonla
  temas etmez. Alüminyum–CFRP arasına bir kat E-cam yalıtım, astar ve sızdırmazlık maddesiyle ıslak montaj.
* Ön yük: ECSS yöntemi, akma dayanımının %65'i, sürtünme 0,12. Örnek sıkma torku (8.8): M5 5,1 N·m, M6 8,6 N·m,
  M8 20,7 N·m. Kompozit sıkıştırmada %50 kullanım, geniş pullar (ISO 7093), ortalama basınç ≤ 100 MPa.
  `design/fastener_catalog.py`'deki VDI 2230 değerleri ~%15–20 yüksektir; ayrıntılı tasarımda uzlaştırılmalı.
* Motor bölmesinde tamamen metal kilitli somun/somun plakası veya emniyet teli. CFRP kaplamalarda yapıştırmalı
  perçinsiz somun plakaları (Click Bond tipi). Kaplamalarda Skybolt SK2600, kaporta ve yapısal kapaklarda SK4002.

---

## 8. Boyutlandırma fazına devredilen başlangıç değerleri

Tamamı [`baseline.yaml`](../data/research/baseline.yaml)'dadır. Özet:

| Girdi | Değer |
|---|---|
| Tasarım noktası | MTOW 145 kg, S 3,0 m², b 6,0 m, AR 12, λ 0,5, kök veteri 0,667 m, uç 0,333 m, MAC 0,519 m |
| Sürükleme (ilk iterasyon) | CD0 0,035 (0,030–0,040), e 0,75 → bileşen bazlı sürükleme hesabıyla değiştirilecek |
| Motor modeli | 18 kW / 7500 rpm, sürekli 16,2 kW / 6000 rpm, özgül tüketim eğrisi [[0,20, 600], [0,30, 540], [0,50, 470], [0,75, 460], [1,00, 430]] g/kWh ±%12, irtifa σ^1,23 |
| Pervane | 32×18 2B itici, statik 460 N (boyutlandırma), seyir verimi 0,79–0,81 × 0,95 |
| Kütle | boş 87 kg, yakıt 31,9 kg, faydalı yük 20 kg, büyüme payı 6,1 kg, üst sınır 149,9 kg |
| Hızlar | VS 23,5 / 20,7 m/s, VC 45, VD 57, VF ≥ 37,3 m/s (EAS) |
| Yükler | +3,8 / −1,52, hamle 15,24 / 7,62 m/s, FoS 1,5, fitting 1,15, kompozit toplam 1,8, çökme hızı 2,38 m/s |
| Görev | ≥ 10 h havada kalış, 28–30 m/s bekleme, 30–36 m/s seyir, 4500 m tavan, ≤ 300 m pist |
| Konfigürasyon (önerilen başlangıç) | tek gövde, arkada itici motor, V/ters-V kuyruk (alternatif ikiz kiriş), sabit üç tekerli takım, burun/çene altında taret |

Kontrol hesapları (tahmin): 30 m/s deniz seviyesi beklemesinde sürükleme 101 N, toplam mil gücü 4,42 kW (%24,6),
yakıt akışı 2,53 kg/h → %10 yedekle ~10,9 h; 3000 m'de 2,80 kg/h → ~9,8 h. Tutunma hızı 23,5 m/s
(≤ 24 m/s hedefini karşılar). Kalkış itki/ağırlık 0,32.

---

## 9. Çapraz kontroller ve düzeltmeler

Altı dosya birbiriyle ve mühendislik mantığıyla karşılaştırıldı; kuşkulu değerler kaynağından yeniden okundu.
Her düzeltme ilgili dosyanın `corrections` listesinde yazılıdır.

| No | Dosya | Sorun | Düzeltme |
|---|---|---|---|
| C-COMP-01 | comparables | Nominal 150 kg ve üst 180 kg M3 sınıfında | `regulatory_cap` 149,9 kg eklendi; başlangıç 145 kg |
| C-COMP-02 | comparables | Yakıt hacmi 750 kg/m³ ile çevrilmiş | 740 kg/m³: 33 kg = 44,6 L |
| C-COMP-03 | comparables | Faydalı yük bandı 10–20 kg'lık gimbal varsayıyordu | Seçilen taret 1,55 kg; bant büyük ölçüde araştırma yükü payı |
| C-ENG-01 | engine | Jeneratör 5200 rpm varsayımıyla 550 W | Bekleme devri 3800 rpm → ~405 W, ~34 V |
| C-ENG-02 | engine | Bekleme gücü tablonun en düşük noktasının altında | Not eklendi; 0,20 dışdeğerleme noktası baseline'da |
| C-ENG-03 | engine | Pervane tahmini | aero.yaml pervanesine atıf (aynı boyut) |
| C-AERO-01 | aero | Statik itki 512 N, yazılan yöntemle tutarsız | 500 N @ 5790 rpm, boyutlandırma 460 N; 31×12 3B devri 5960 rpm |
| C-CMP-01 | components | 48 V bara gerilimi beklemede sağlanmıyor | Kontrolcü gerilim yükseltmeli (açık konu) |
| C-CMP-02 | components | Jeneratör payı 2,3–2,9 kat sanılmış | Beklemede 1,15–1,7 kat |
| C-CMP-03 | components | Takım yük katsayıları yer tepkisiydi; atalet katsayısı 5,3–6,2 | ≥ 0,18–0,22 m etkin çökme veya sönümleyici |
| C-CMP-04 | components | Flap menteşe momenti 30–33 m/s'ye göre | VF ≥ 37,3 m/s (CS-LUAS.345(b)): 15,1 N·m limit, DA 30 kol oranı ≥ 2,4 |

Değiştirilmeden doğrulananlar: Limbach güç/kütle/opsiyon kütleleri; SG750 verileri ve "will integrate" ifadesi;
SHT-İHA sınıf sınırları ve tescil maddesi; NCAMP MTM45-1 ETW sıcaklığı, Tg ve B-basis değerleri; Mejzlik 32×18
kütlesi ve tablosu; Volz DA 26, Trillium HD59, Galaxy GRS 4/240 ve Primoco One 150 verileri. Ayrıca motor yakıt
tablosu, irtifa düşümü, soğutma havası, benzer İHA uydurmaları ve tutunma tablosu, standartlardaki hamle/iniş örneği,
malzeme tork tablosu ve eklemeli imalat azaltma katsayıları, aero itki ihtiyacı yeniden hesaplanarak tutarlı bulundu.

---

## 10. Belirsizlikler ve açık konular

### 10.1 Belirsizlikler

* **Motor:** Limbach TBO, CHT/EGT sınırları, rölanti ve sürekli güç değeri yayımlamıyor; 16,2 kW sürekli güç ve
  500 h TBO tahmindir. Kısmi yük yakıt tüketimi ±%12; %30'un altı dışdeğerlemedir. İrtifa düşümü Limbach değil
  Hirth verisinden uyduruldu. SG750'nin üretim durumu ve devir–güç eğrisi bilinmiyor. "HP" birimi mekanik hp
  kabul edildi (PS olsaydı ~%1,4 fark).
* **Benzer İHA'lar:** kanat alanı yalnız iki uçakta yayımlanmış; kanat yüklemesi/AR istatistiği ±%15–20. Birçok
  üretici sayfası otomatik erişimi engelledi; bazı değerler arama motoru özetinden okundu. Yakıt kütleleri çoğunlukla
  kütle dengesinden tahmindir.
* **Standartlar:** STANAG 4703 değerleri 2014 taslağından; yürürlükteki 2016 baskısı ücretli ve karşılaştırılmadı.
  Tork katsayısı 6 kendi yorumumuzdur. Sıcaklık–soğurganlık eğrisi şekilden okundu (±3 °C). SHGM'nin M2 teknik
  gereklilikleri ve yazılım sertifikasyon usulü henüz yayımlanmadı.
* **Aerodinamik:** NeuralFoil yalnız iki rüzgâr tüneli verisiyle doğrulandı. %13 uç profili test edilmemiş türetilmiş
  bir kesittir. Zorlanmış geçiş pürüzlülük yüksekliğini modellemez; pürüzlülükte cl_max kaybı sonucu literatüre
  dayanır. Mejzlik uçuş verileri kendi simülasyonlarıdır; Mach 0,7 üstünde doğruluk düşer. Sürükleme kutupsalı
  (CD0 0,035) tahmindir ve ±%10 itki ihtiyacı değiştirir.
* **Alt sistemler:** menteşe momentleri, takım yükleri, yay bacak, batarya ve depo kütleleri ±%30–50 tarama
  tahminidir. iPS750 çıkış gerilimi yayımlanmamış. Fiyatların çoğu talep üzerinedir. Görev bilgisayarı seçilmedi
  (kütle ve güç tahmin).
* **Malzeme:** metal değerleri MIL-HDBK-5G'den (1994); güncel MMPDS ile karşılaştırılmadı. MTM45-1 reçine yoğunluğu
  yayımlanmamış (laminat yoğunluğu ±25 kg/m³). ISO tablo değerleri ikincil kaynaklardan. Islak yatırma/infüzyon
  değerleri tahmindir.

### 10.2 Açık konular (öncelik sırasıyla)

1. **Limbach:** STEP/DXF kurulum çizimi, TBO, sıcaklık sınırları, itici dönüş yönü, silindir yönü, fiyat.
2. **ePropelled:** SG750 durumu, 3500–7500 rpm arası güç ve gerilim eğrisi, kontrolcünün yükseltme aralığı ve kütlesi.
3. **SHGM:** M2 uçuşa elverişlilik belgesinin sertifikasyon tabanı (CS-LUAS + STANAG 4703 önerisi), yazılım
   sertifikasyonu, uyum modülü komut seti, uzaktan kimlik biçimi.
4. **Boyutlandırma:** VC/VD seçimi ve her ağırlık/irtifada hamle; bileşen bazlı sürükleme; bekleme devri, yakıt akışı
   ve elektrik payının yeniden hesabı; VD ≥ VH kontrolü.
5. **İniş takımı:** ≥ 0,18–0,22 m etkin çökme veya sönümlü bacak; zıplama ve burun tekeri titreşimi (shimmy).
6. **Mejzlik:** 32×18 2B ve 31×12 3B için L 275 EF üzerinde statik test raporu; `aero_polars.py` dışdeğerlemesinin
   düzeltilip yeniden çalıştırılması.
7. **Kompozit değerler:** üretim sürecinde NCAMP eşdeğerliği; tek yük yollu parçalar için A-basis değerler.
8. **Görev bilgisayarı ve araştırma yükü arayüzü** (kütle, güç, bağlantı).
9. **Teklifler:** Volz, Embention, Silvus, Trillium (ICD/STEP), Galaxy GRS, TOST, ATL.
10. **%13 uç profili** XFoil/MSES ile doğrulanmalı; NACA 4415/0010 kullanılacaksa `.dat` dosyaları yeniden
    adlandırılmalı.
11. **BTK** frekans tahsisleri, uzaktan kimlik ve transponder taşıma kuralları.
