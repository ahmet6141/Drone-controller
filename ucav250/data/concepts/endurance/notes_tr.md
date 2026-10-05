# YK-250 — Konsept “Dayanım” (MALE tipi) — tasarım notları

Kaynak: `ucav250/data/concepts/endurance/calc.py` (çalıştırma: `PYTHONPATH=. python3 ucav250/data/concepts/endurance/calc.py`).
Tüm sayılar `concept.yaml` dosyasından gelir. Her girdinin kaynağı ya da dayanağı da orada `inputs` altında kayıtlıdır.
Kapsam: sivil EO/IR gözetleme ve araştırma platformu. Silah, mühimmat, askı noktası, pilon ya da bırakma mekanizması yoktur.

## 1. Konfigürasyon ve gerekçe

| Karar | Seçim | Neden (sayısal) |
|---|---|---|
| İtki | Limbach L 275 EF, **itici**, Mejzlik 32x18 2B, 80 mm göbek ara parçası | Burun ve gövde altı EO/IR görüşü serbesttir. Pervane izi NLF kanat köküne ve tarete gelmez. Benzer 13 İHA'nın 11'i iticidir. 32x18 2B, 31x12 3B'ye göre +0,6 h dayanım kazandırır (tırmanma 4,35 ve 5,09 m/s). |
| Kanat | Omuz kanat, AR **15**, λ 0,45, NLF(1)-0416 (%16 → %13), 2,5° burulma, i = 4,50° | 16 noktalı AR × W/S taraması yapıldı. Dayanım W/S ile artar, çünkü süreç kuralındaki 0,6 mm asgari kaplama kanat kütlesini alana bağlar. Bekleme CL'si ise 1,2 VS payıyla sınırlıdır. En iyi nokta VS = 24 m/s köşesindedir; o yüklemede AR'yi 6,8 m açıklık sınırı belirler. Sonuçlar (dayanım, h): AR 12: 400 Pa 12,3, 433 Pa 13,1, 467 Pa 13,6, 500 Pa 13,8 (VS ✗); AR 13: 400 Pa 12,6, 433 Pa 13,4, 467 Pa 13,9, 500 Pa 14,2 (VS ✗); AR 14: 400 Pa 12,8 (açıklık ✗), 433 Pa 13,6, 467 Pa 14,1, 500 Pa 14,4 (VS ✗); AR 15: 400 Pa 12,9 (açıklık ✗), 433 Pa 13,8 (açıklık ✗), 467 Pa 14,4, 500 Pa 14,6 (VS ✗). |
| Kuyruk | **Düz V-kuyruk** (41°), alt kuyruk tamponu | Ters V'de panel uçları yer çizgisinin 0,26 m altına iner; iniş takımı 0,55 m uzamalıdır (dayanım 9,7 h). T-kuyruk: dikey kuyruk yatay kuyruk yükünü taşır, derin perdövites riski vardır (14,1 h). Y-kuyruk fazladan yüzey ve birleşim getirir (14,3 h). V-kuyruk 14,8 h verir. Purser–Campbell'e göre toplam alan aynıdır; kazanç birleşim sayısında, tek kalıpta ve yer açıklığındadır. |
| İniş takımı | **Sabit üç tekerlek**, GFRP yay + kaportalı TOST 200x50 | İçeri katlanır takım, sürüklemenin %90'ını kaldırmasına karşın +2,5 kg getirir ve dayanımı 0,73 h azaltır. Satın alınabilir ürün yoktur. Takım boşlukları, yakıt hücreleri ve faydalı yük bölmesiyle aynı AM bölgesine düşer. |
| Flap | Yalnız **kalkış flabı** (iç, 15°) | Kanat açısı i = 4,50° seçildi. Bu açıyla beklemede gövde yaklaşık 2,3° burun yukarı durur (tam yatay için 6,8° gerekirdi). İniş temasında ise ana tekerler 3° burun yukarı açıyla önce değer. Bu açıyla iniş flabı, teması burun tekerleğine kaydırır; bu yüzden iniş flabı kullanılmaz. Kuyruk, yüksek itki hattına karşı 30,4 m/s altında burnu kaldıramaz. Flap, kanadın yer açısında 27,0 m/s'de düz kalkmasını sağlar (yerde koşu 174 m; flapsız 233 m). |
| Yerleşim | Faydalı yük bölmesi AM'de, önünde ve arkasında birbirine bağlı iki yakıt hücresi | Yük değişimi ve yakıt tüketimi AM'yi oynatmaz (tüm yüklemelerde AM 2,232–2,235 m, SM 0,100–0,106). Trim sürüklemesi en azdır. İlk yerleşimde (yük önde) SM 0,10–0,42 arasında geziyordu. |
| Gövde boyu | Arka gövdeye 0,40 m uzatma (toplam 4,04 m) | Kuyruk kolu 1,36 m olur, V-kuyruk alanı küçülür. L/b = 0,60; benzer dayanım tipi İHA'larda 0,51–0,60. |

## 2. Ana sayılar (MTOM 145,0 kg, tavan 149,9 kg, Türkiye M2 sınıfı)

| | |
|---|---|
| Boş / yakıt / faydalı yük | 88,5 / 36,5 / 20,0 kg. Boş kütle oranı 0,610, %5 büyüme payı olmadan 0,581; benzerlerde 0,55–0,62. |
| Kanat | S 3,04 m², b 6,76 m, kök/uç veteri 0,621/0,280 m, OAK 0,472 m, W/S 47,6 kg/m² |
| Aerodinamik | CD0 0,0331, e 0,799, (L/D)maks 16,9. Kanat profil sürüklemesi tripli polarlardan alınıp 1,15 ile çarpıldı. CLα 5,69 1/rad; temiz trimli CLmax 1,35; perdövites başlangıcı η 0,42 (kökte başlar). |
| Hızlar | VS 23,8 m/s (temiz), kalkış flabıyla 22,6 m/s. 3000 m'de bekleme 28,5 m/s EAS / 33,1 m/s TAS. En uzun menzil hızı 34,2 m/s. Vmaks 57,1 m/s (deniz seviyesi) ve 54,3 m/s (3000 m). |
| Dayanım | **14,8 h**: 2×100 km intikal, 3000 m'de bekleme, %10 yedek. Bekleme yakıt akışı 2,38 kg/h, motor 4134 dev/dk'da. Yalnız temel algılayıcılarla, faydalı yük bölmesine yardımcı yakıt torbası konursa 23,4 h. Taban gereksinim olan 10 h için MTOW 130,6 kg yeterlidir. |
| Menzil | 1620 km (feribot, 3000 m, %10 yedek) |
| Tırmanma / tavan | 4,35 m/s deniz seviyesinde, 2,44 m/s 3000 m'de (hedef 4,9 m/s; pervane sınırlı). Servis tavanı 6912 m. |
| Pist | Kalkış yerde koşusu 174 m, 15 m'ye 302 m. İniş yerde koşusu 171 m (görev sonu); MTOM'da iptal inişinde 216 m. |
| Kararlılık | V_H 0,450, V_V 0,0240, dε/dα 0,25, Cnβ 0,057 1/rad |
| Yapısal yük | Limit n 5,03 (VC 45 m/s, 4500 m'de 15,24 m/s rüzgâr darbesi), nihai 7,55. Kök eğilme momenti 7807 N·m (nihai). |
| Duyarlılık | BSFC ±%12 → 16,8/13,2 h; CD0 +%10 → 14,2 h; boş kütle +%5 → 12,6 h |

## 3. Yapı konsepti: şasi + kabuk

**Şasi (birincil yapı, bütün tekil yükleri taşır).** Gövde tabanı boyunca iki CFRP şapka profilli omurga kirişi uzanır. Bunlara on adet sandviç perde bağlanır. Malzemeler: MTM45-1/AS4 OOA prepreg (fırında, vakum torbasında kürlenir; 121 °C + 177 °C serbest son kür) ve ROHACELL 51/71 WF-HT köpük. Ağır perdeler ve yük yolları şunlardır:
- **Kanat bağlantısı:** Kanat yarıları planör tipi kiriş dili/çatalıyla merkez hatta üst üste biner. Böylece kanadın eğilme momenti merkez hatta kendi içinde dengelenir; gövdeye yalnız kesme kuvveti, sürükleme ve burulma iner. Ana kiriş x ≈ 2,27 m'dedir ve faydalı yük bölmesinin tam üstüne düşer. Bu nedenle 2 ana Ti pim, bölmenin iki perdesini (x 2,03 ve 2,43 m) üst güvertede birleştiren talaşlı 7075-T651 eyer kirişine oturur. Arka sürükleme pimleri arka perdeye bağlanır. Yük, perdelerden omurga kirişlerine geçer. Faydalı yük bölmesi güvertenin altında kalır.
- **Ana iniş takımı:** GFRP yay (7781 cam + UD), iki 7075 kelepçeyle faydalı yük bölmesinin arka perdesine bağlanır (x ≈ 2,39 m). İniş yükü kelepçeden perdeye, oradan omurgaya geçer. Burun bacağı muylusu F3 perdesindedir (x 0,72 m).
- **Motor:** 4 × M8 + Limbach izolatörleri, TIG kaynaklı 4130 boru kafese bağlanır. Kafes, yangın duvarı perdesi (0,38 mm paslanmaz kaplı) üzerinden omurgaya bağlanır. Arkada 15 g ileri yönde tutma esastır.
- **V-kuyruk:** Kök bağlantı parçaları motor bölmesi önündeki perdeye bağlanır.
- **Paraşüt:** 13,1 kN açılma şoku sert noktadan F4 perdesine ve omurgaya iletilir.

**Kabuk (ikincil yapı, sökülebilir).** Gövde sandviç panelleri 0,4 mm + 0,4 mm CFRP PW ile 5 mm ROHACELL 51 WF-HT'den yapılır. Kapaklar şunlardır: burun aviyonik kapağı, paraşüt kapağı, kanat eyer kaplaması, alt faydalı yük kapağı (nadir pencereli), yakıt dolum paneli, motor kaportası (üst/alt). Kapaklar SK2600 çeyrek dönüşlü bağlantı elemanıyla 100–150 mm aralıkla, motor kaportası SK4002 ile 60–90 mm aralıkla tutturulur. Kanat ve V-kuyruk kaplamaları birincil yapıdır: dış yüz 0,6 mm, iç yüz 0,4 mm sandviç, dişi kalıpta tek parça alt/üst kabuk olarak üretilir. Hücum kenarının ilk %8–10'una erozyon bandı uygulanır (NLF kuralı). Kanat kirişi UD başlıklı ve ±45 gövdelidir. Başlık kesiti hasar toleransı birim uzama sınırıyla boyutlandırılmıştır (3000 µε, nihai yükte).

**Üretim süreçleri.** Kompozit parçalar dişi kalıpta OOA prepreg ile fırında üretilir. Metal parçalar ISO 2768-mK'ya göre CNC frezelenir (7075). Motor kafesi 4130 TIG kaynaklıdır (AMS 6457 dolgu). İniş yayı RTM ya da prepreg cam ile üretilir. Yapıştırmalarda EA 9394 kullanılır. CFRP'de yalnız Ti/A286 bağlantı elemanı kullanılır.

## 4. Taşıma ve bakım erişimi

- **Taşıma ayrımı:** 2 kanat yarısı (3,38 m), 2 V-kuyruk paneli (0,97 m), pervane, gövde (4,04 m, tek parça). Kanat sandığı yaklaşık 3,5 × 0,7 × 0,35 m, gövde sandığı yaklaşık 4,2 × 0,8 × 0,8 m. Kanat kökünde elektrik için tek konnektör vardır; yakıt kanatta olmadığından boşaltma gerekmez.
- **Bakım erişimi:**
  - Aviyonik ve batarya: burun üst kapağı.
  - Paraşüt: üst kapak, piroteknik emniyet pimiyle.
  - Faydalı yük: alt kapak; görev bilgisayarı aynı bölmede.
  - Yakıt torbaları: kanat sökülünce eyer açıklığından takılıp çıkarılır.
  - Dolum ve seviye sondası: yan panelden (kuru-kopma bağlantısı).
  - Motor: kaporta üst/alt yarılar; buji, CHT/EGT ve SG750'e yandan erişilir.
  - Servolar: kanatçık ve kuyruk dümeni servoları yüzey altındaki kapaklardadır.
  - Tekerlek kaportaları sökülebilir.

## 5. Riskler ve açık konular (dürüst değerlendirme)

1. Deniz seviyesinde bekleme devri, Limbach'ın 4000–6000 dev/dk düzenli uçuş bandının altında kalır (3000 m'de bandın içindedir). Limbach'tan düşük yük onayı alınmalı ya da en az 1500 m'de beklenmelidir.
2. Deniz seviyesinde tırmanma, 4,9 m/s hedefinin altındadır. Sınırlayan motor değil, 32x18 2B pervanedir. 31x12 3B seçeneği tırmanmayı iyileştirir ama dayanımdan götürür.
3. Bekleme gücü oranı, BSFC eğrisinin en düşük noktası olan 0,20'nin altındadır. BSFC doğrusal olarak dışa uzatıldı; düşük yük için dinamometre haritası gerekir. BSFC'deki ±%12 belirsizlik dayanımı yaklaşık ±1,8 h değiştirir.
4. AR 15 kanatta çırpınma, burulma rijitliği ve kanatçık ters etkisi henüz analiz edilmedi. CS-LUAS.629'a göre 1,2 VD'ye kadar temizlik gösterilmelidir.
5. Arka gövde 0,44 m'den 0,30 m kaporta dudağına 0,10 m içinde kapanır. Bu, itici pervanenin emişine ve soğutma çıkış akışına dayanır; CFD ile doğrulanmalıdır. 80 mm göbek ara parçası için Limbach onayı gerekir.
6. Kalkış düzdür (yer açısı, 15° kalkış flabı). Yüksek itki hattı nedeniyle kuyruk, burnu 30,4 m/s altında kaldıramaz. Otopilot kalkış kuralı, kanat uçağı kaldırana kadar burun tekerleğini yerde tutmalıdır. Kalkışta 15 m'ye mesafe 302 m'dir; 300 m pist sınırındadır.
7. Belgelenen tam motor zarfı kutusu (0,2214 × 0,3968 × 0,2946 m) alt köşelerinde OML'ye sığmıyor. Gerçekçi zarf sığıyor: silindir dilimi, 0,30 m genişlikte varsayılan emme kutusu ve karter/SG750. Emme gövdelerinin gerçek genişliği Limbach çiziminden alınmalıdır; gerekirse kaportaya yanak çıkıntısı eklenir.
8. İptal inişinde MTOM'da yerde koşu 216 m'dir; 200 m sınırının üstündedir. Kanat açısı yüksek olduğundan koşu sırasında kaldırma oluşur ve fren etkisi azalır. Önlem olarak inişte kanatçıklar yukarı alınabilir (kaldırma azaltma).
9. Jeneratör payı: 3000 m'de bekleme ×1,46, deniz seviyesinde ×1,26 (303 W yük). SG750 çıkış eğrisi ePropelled'den alınmalıdır.
10. Bu konsept çalışmasında rüzgâr darbesi yük katsayısıyla ön boyutlandırma yapıldı. Kanat bağlantıları, kuyruk bağlantısı ve motor kafesi için ayrıntılı mukavemet hesabı yapısal analiz fazında yapılacaktır.

## 6. Sonuç

Konsept uygulanabilir. Taban gereksinim olan 10 h'i yaklaşık %48 payla aşar; aynı görev için MTOW 131 kg yeterlidir. Kazancın kaynakları şunlardır: dar ve uzun kanat (b 6,76 m), AM'de sabitlenmiş yükleme (düşük trim sürüklemesi), kaportalı sabit takım ve az birleşimli V-kuyruk. Bedeli ise düz kalkış yöntemi, pervane sınırlı tırmanma ve sert arka gövde kapanışıdır. Bu üç konu CFD, motor üreticisi verisi ve uçuş testiyle doğrulanmalıdır.

Ek dosyalar: `sketch.png` (aynı ölçekte üç görünüş ve iç yerleşim), `constraint.png` (sizinglib kısıt diyagramı), `concept.yaml` (bütün girdiler, sonuçlar ve takaslar).
