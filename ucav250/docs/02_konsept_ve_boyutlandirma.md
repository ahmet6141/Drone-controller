# YK-250 HANÇER — Konsept ve boyutlandırma raporu (faz 2, rev. v1.6)

**Tarih:** 9 Ekim 2026 · **Durum:** proje liderinin faydalı yük – dayanım gereksinim kararı uygulandı; dördüncü (V3-01…V3-11) ve beşinci (V4-01…V4-08) bağımsız doğrulamanın bulguları işlendi. `--check` 61 gereksinimin 61'ini karşılanmış buluyor; spec'teki 1224 türetilmiş değerin hepsi tolerans içinde, kapanış yakınsıyor ve çıkış kodu 0'dır. `tests/test_ucav250_*.py` testlerinin hepsi geçiyor.

> **Kapsam.** YK-250 HANÇER sivil bir EO/IR gözetleme ve araştırma İHA'sıdır. Faydalı yük yalnızca EO/IR taret ve
> görev donanımıdır. Silah, mühimmat, dış yük bağlantı noktası, pilon ya da yük bırakma mekanizması yoktur ve tasarım
> bunlar için yer ayırmaz. UCAV görünümü yalnızca biçim dilidir.

![HANÇER — izometrik görünüş (takım ve taret açık)](fig/yk250_iso.png)

Bu raporun bütün sayıları tek kaynaktan gelir: [`spec.yaml`](../spec.yaml). Hesap [`analysis/sizing.py`](../analysis/sizing.py) içindedir. Çıktılar [`out/sizing.json`](../out/sizing.json), [`out/sizing.md`](../out/sizing.md) ve [`out/sizing_trades.json`](../out/sizing_trades.json) dosyalarına yazılır.

```
python3 -m ucav250.analysis.sizing --check        # yalnız doğrular: 61 gereksinim + spec'teki her türetilmiş değer + görev yükü kuralı + kapanışın yakınsaması (çıkış 1 = ihlal); dosya yazmaz
python3 -m ucav250.analysis.sizing --check --out /tmp/x   # aynı kontrol, çıktılar depo dışına
python3 -m ucav250.analysis.sizing                # çıktıları yazar: out/sizing.json, out/sizing.md, docs/fig/*.png
python3 -m ucav250.analysis.sizing --update-spec  # kapanış + görev yükü kuralı + değerlendirme; spec'teki türetilmiş değerleri ve LERX/eldiven kesit dosyalarını yeniden yazar
python3 -m ucav250.analysis.sizing --trades       # ödünleşimler + tasarım değişiklikleri merdiveni -> out/sizing_trades.json
python3 -m ucav250.analysis.sizing --render       # Workbench görüntüleri (docs/fig, bpy gerekir)
```

---

## Özet

| Büyüklük | Değer |
|---|---|
| Konfigürasyon | Köşeli (chine) taşıyıcı gövde + LERX + yüksek açıklık oranlı NLF kanat; dışa eğik ikiz dikey kuyruk (dümenli), sabit kök parçalı ve kısa milli tamamen hareketli yatay kuyruk (stabilatör), ventral kanatçık/kuyruk tamponu. Açık itici pervane; içeri katlanır üç tekerlekli takım (altı contalı kapak; iç kapaklar sıralı, eyleyicili); açıklık halkalı, geri çekilir EO/IR taret |
| MTOM / boş / yakıt / görev yükü | **149,9 kg** / 101,3 kg / 30,6 kg / 18,0 kg (azami faydalı yük 20,0 kg) |
| Kanat | açıklık **7,20 m**, alan 3,241 m² (referans yamuk), AR 16,0, λ 0,35, c/4 süpürme 8°, burulma 4°, OAK 0,485 m. Kesitler: dış kanatta NLF(1)-0416, LERX kökünde NACA 0016-34. Dış panel 2,90 m |
| Gövde | boy 4,00 m (pervane göbeğiyle 4,23 m), genişlik 0,80 m, yükseklik 0,46 m |
| Pervane | Mejzlik 31x12 3B, Ø 0,787 m, 5° aşağı itki hattı, statik uç Mach 0,712; kurulum katsayısı k_inst 0,93 tahmini (küt kaporta tabanı; test yok, duyarlılık §6.5) |
| Elektrik | SG750 marş/jeneratörü (DC = 800 W × rpm/7500 × 0,94); tampon batarya 12S2P Li-ion (Molicel P45B, 1,98 kg, 389 Wh); E180 tepe yükü bekleme boyunca jeneratör + batarya tepe destek payıyla (R-52 1,002; jeneratör açığı en çok 17 W) |
| Dayanım | **10,41 h** tasarım görevinde, 18,0 kg görev yüküyle (R-02 ≥ 10 h; görev yükü kuralı ≥ 10,25 h; bekleme 8,10 h). Azami 20 kg yükle **9,64 h** (R-02b ≥ 9,5 h). E180 büyüme görevi 10,15 h. Feribot menzili 1.157 km |
| Aerodinamik | CD0 0,0351 (temiz), 0,0370 (taret dışarıda); (L/D)maks 17,3 / 16,8 |
| Hızlar | VS 23,95 m/s (temiz, trimli, MTOM, en ön AM). Görev beklemesi 27,94 → 27,85 m/s EAS. Düz uçuş Vmaks 54,0 m/s. VNE 51,3 m/s ve VNO 45,0 m/s EAS; uçuş kontrol sistemi hızı 45 m/s'de sınırlar |
| Tırmanma / tavan | 4,21 m/s (DS), 2,38 m/s (3000 m); servis tavanı 7.100 m |
| Pist | kalkış koşusu 179 m (belirleyici: E180 taretli MTOM; yunuslama ataletiyle), görev sonu iniş koşusu 183 m, MTOM'da acil iniş 219,6 m |
| Kararlılık | statik marj %10,1–%14,6 OAK; Cnβ 0,0585 1/rad |
| Kumanda | stabilatör Volz DA 30 2,5:1, kanatçık ve dümen Volz DA 26 2:1, flap Volz DA 30 2:1; hepsi dört çubuk bağlantı, her sapmada menteşe momenti kontrolü (R-37, R-47, R-57…R-59) |
| Doğrulama | **61/61 gereksinim** karşılanıyor. `--check` 1224 türetilmiş değerin hepsini tolerans içinde buluyor, kapanış yakınsıyor, çıkış kodu 0 |

**Kısa sonuç.** v1.4'te dürüst modelle 20 kg faydalı yükle dayanım 9,90 h idi ve R-02 (10 h) karşılanmıyordu. Proje lideri gereksinimi yeniden tanımladı (aşağıdaki bölüm): 10 h artık *tasarım görevi faydalı yüküyle* istenir. Bu yük sizing.py'de bir kuralla türetilir: aynı görevde ≥ 10,25 h veren, 0,5 kg'a aşağı yuvarlanmış en büyük faydalı yük. v1.5'te kural 19,0 kg vermişti. v1.6'da takım kapağı kalemi v1.5'in kapak düzenine göre yeniden kuruldu (kalem +0,59 kg, boş kütle +0,62 kg; V4-03) ve kural yeni kapanışta **18,0 kg** verdi (10,41 h; 18,5 kg 10,22 h verirdi). Azami faydalı yük 20 kg olarak kalır ve aynı görevde 9,64 h verir (R-02b ≥ 9,5 h). Kütle bütçesi bu iki gereksinime göre 0,25 kg payla kapanır; hiçbir kalem silinmedi.

Beşinci doğrulamanın (V4-01…V4-08) bulguları da giderildi:

* R-60 artık kırpılmamış payla ölçülür ve dönüş yasasının tutarlılığına bağlıdır; bir stabilatör yetersizliği R-60'ı düşürür (V4-01).
* Faydalı yük – dayanım tablosu izin verilen en hafif yüklemeden (temel EO/IR seti) başlar. Yüksüz yükleme statik marjı sağlamaz; tabloda yer almaz, ayrıca raporlanır. Dolu depolu hafif yükleme yükleme durumlarına eklendi (V4-02).
* Takım kapağı kalemi kapak alanına, iç kapak eyleyicilerine ve bacak kapağı ayaklarına göre yeniden kuruldu; görev yükünün kural eşiğine payı raporlanır (V4-03, V4-04).
* Kalkış dönüşünde yunuslama ataleti modellendi (V4-05).
* Şekil ve görüntü kusurları (V4-06, V4-07) ile metin hataları (V4-08) düzeltildi.

v1.5'te dördüncü doğrulamanın bulguları (V3-01…V3-11) işlenmişti: bölme içerikleri, dönüş yasası (R-60), E180 görevinin elektrik yükü, R-52 işletme sınırı, k_inst ve düşük yük duyarlılıkları, kuyu/kapak görüntüleri (§3.2).

## Gereksinim kararı: faydalı yük – dayanım

**Neden bir karar gerekti.** Üç bağımsız doğrulama turu aynı sonucu verdi. Kullanıcının istediği bütün özellikler korunduğunda HANÇER, 149,9 kg'lık Türk M2 MTOM tavanının altında 20 kg faydalı yükle 10 saatlik tasarım görevini uçamıyor. Bu özellikler şunlardır: içeri katlanır iniş takımı, geri çekilir taret, dört kuyruk yüzeyi (iki eğik dikey ve iki stabilatör) ve köşeli geniş gövde. v1.4'ün dürüst sonucu 9,90 h idi.

Gereksinimi karşılamanın iki yolu vardı, ikisi de kullanıcının açık seçimlerini bozuyordu:

* sabit, kaportalı iniş takımı: 11,26 h;
* sabit (hep dışarıda) taret: 10,57 h.

Bu iki değer v1.4 ödünleşimlerinin 20 kg'daki sonuçlarıdır; v1.6 değerleri (görev yüküyle) §13'tedir. Yüksek bekleme irtifasını düşürmek görev tanımını değiştirirdi. MTOM'u artırmak M2 sınıfının dışına çıkarırdı. R-52'yi gevşetmek kullanıcı tarafından onaylanmamıştı.

**10 h / 20 kg bir kullanıcı gereksinimi değildi.** Bu çift araştırma fazında bir başlangıç hedefi olarak konmuştu. `baseline.yaml#mass_targets.payload_design_kg` 20 kg'ı "benzer araçların 20–35 kg yük bandının alt ucu" diye tanımlar. Bu bandın 10–20 kg'lık bir gimbal varsaydığı da not edilmiştir (C-COMP-03). Seçilen HD59 temel sensör seti ise yalnız 3,25 kg'dır (taret + bağlantı 1,75, görev bilgisayarı 1,0, tepsi/kablo 0,5). 20 kg'ın geri kalanı araştırma yükü için ayrılmış bir paydır.

**Karar (proje lideri, 9 Ekim 2026).** Kullanıcının seçtiği özellikler korunur. Gereksinimler şöyle yeniden tanımlanır:

| No | Yeni tanım | v1.6 sonucu |
|---|---|---|
| R-02 | Tasarım görevi dayanımı ≥ 10 h, **tasarım görevi faydalı yüküyle**. Bu yük, MTOM 149,9 kg'da aynı görevde ≥ 10,25 h veren (≥ 0,25 h sağlamlık payı), 0,5 kg'a aşağı yuvarlanmış en büyük faydalı yüktür. Değer `mission.payload_design_kg` içindedir, sizing.py türetir ve `--check` kuralla karşılaştırır | 18,0 kg ile 10,41 h ✔ |
| R-02b (yeni) | Azami faydalı yük 20 kg ile aynı görevde dayanım ≥ 9,5 h | 9,64 h ✔ |
| R-03 | Azami faydalı yük ≥ 20 kg: MTOM'daki yükleme durumlarından biri 20 kg taşır ve bütün kontroller bu yükle de yapılır | 20,0 kg ✔ |
| R-56 | Kütle bütçesi (grup tavanları + yedek) R-02 ve R-02b'yi tam karşılayan iki boş kütlenin küçüğünü aşmaz | +0,255 kg ✔ (belirleyen R-02b) |

**Sayılar.** Görev yükü kuralı kapanmış uçakta uygulanır (`performance.payload_design_rule`):

* 18,0 kg → 10,412 h ≥ 10,25 h;
* 18,5 kg → 10,218 h < 10,25 h.

Görev yükü bu yüzden 18,0 kg'dır: 3,25 kg temel sensör seti ve 14,75 kg araştırma yükü payı. Azami yük 20 kg için ek 2,0 kg aynı karın bölmesine konur. Proje liderinin ön tahmini "yaklaşık 18 kg" idi. v1.5'te kural 19,0 kg vermişti (10,272 h). Beşinci doğrulama, v1.5'in kapak düzeninin (altı kapak, sıralı iç kapaklar) kütle kaleminde yer almadığını gösterdi (V4-03). Kalem yeniden kurulunca boş kütle 0,62 kg arttı ve kural aynı yöntemle 18,0 kg verdi. Bu sayı türetilmiştir, elle seçilmedi.

**Kural eşiğine pay (V4-03, V4-04).** Görev yüküyle dayanım eşiğin 0,162 h üstündedir. Sabit MTOM'da bu, boş kütlenin **0,42 kg** artmasına denktir; faydalı yük olarak eşik ≈ 18,42 kg'dır. Boş kütle bundan fazla artarsa kural 17,5 kg'ı verir. Bu pay ile R-56 payı farklı şeylerdir (§14, §16).

R-02, 10 h'i 0,41 h payla karşılar. Bu pay, ölçülmemiş iki model girdisinin olumsuz durumlarını da kapsar: k_inst 0,90 olsa 10,22 h, düşük yükte Willans yakıt doğrusu kullanılsa 10,35 h (§6.5).

Görev yükü, yakıt ve bekleme süresi birlikte değişir. R-52'nin jeneratör açığı sınırı v1.5'te 18 W'tan 17 W'a indirilmişti (§12). v1.6'da 18,0 kg ile yeniden denetlendi: 17 W ile R-52 1,002'dir, 17,5 W ile 0,979 olurdu. 17 W korundu.

![Faydalı yük – dayanım](fig/yk250_payload_endurance.png)

| Faydalı yük | Yakıt | Kalkış kütlesi | Dayanım | Bekleme | Statik marj | Not |
|---|---|---|---|---|---|---|
| 0 kg (taret yok) | — | — | — | — | %5,2–%5,9 | **izin verilen yükleme değil** (R-09 ≥ %10); uçmak için taret bağlantısına 1,96 kg safra gerekir |
| 3,25 kg (temel sensör seti) | 35,95 kg | 140,5 kg | 12,93 h | 10,58 h | %10,6–%11,6 | izin verilen en hafif yükleme; yakıt hacmi sınırı |
| 5 kg | 35,95 kg | 142,3 kg | 12,86 h | 10,51 h | %10,5–%11,5 | yakıt hacmi sınırı |
| 7,5 kg | 35,95 kg | 144,8 kg | 12,76 h | 10,42 h | %10,4–%11,3 | yakıt hacmi sınırı |
| 10 kg | 35,95 kg | 147,3 kg | 12,63 h | 10,31 h | %10,3–%11,2 | yakıt hacmi sınırı |
| 12,5 kg | 35,95 kg | 149,8 kg | 12,51 h | 10,19 h | %10,2–%11,1 | yakıt hacmi sınırı |
| 12,61 kg | 35,95 kg | 149,9 kg | 12,50 h | 10,19 h | %10,2–%11,0 | kırılma: depolar dolu ve MTOM |
| 13 kg | 35,56 kg | 149,9 kg | 12,35 h | 10,04 h | %10,2–%11,0 | MTOM |
| 15 kg | 33,56 kg | 149,9 kg | 11,57 h | 9,26 h | %10,2–%10,9 | MTOM |
| 17,5 kg | 31,06 kg | 149,9 kg | 10,61 h | 8,29 h | %10,1–%10,8 | MTOM |
| **18,0 kg (görev yükü)** | 30,56 kg | 149,9 kg | **10,41 h** | 8,10 h | %10,1–%10,8 | R-02 |
| 18,5 kg | 30,06 kg | 149,9 kg | 10,22 h | 7,91 h | %10,1–%10,8 | kuralın bir adım fazlası (< 10,25 h) |
| **20 kg (azami)** | 28,56 kg | 149,9 kg | **9,64 h** | 7,33 h | %10,1–%10,7 | R-02b |

12,61 kg'ın altında yakıt hücreleri doludur (kullanılabilir 35,95 kg); yük azaldıkça yalnız kalkış kütlesi düşer. Tablonun her satırı izin verilen bir yüklemedir: statik marj aralığı (yakıtsızdan o satırın yakıtına) R-09'un (%10) ve R-10'un (%30) içindedir. En hafif satırın dolu depolu yüklemesi (`full_fuel_baseline_sensors_only`) yükleme durumları arasındadır; R-09, R-10 ve ağırlık merkezi zarfı onu da denetler (V4-02).

**İzin verilmeyen yüklemeler (V4-02).** v1.5 tablosunun "0 kg, 13,23 h" satırı uçulabilir bir yükleme değildi: taret olmadan ağırlık merkezi geri kayar ve statik marj bütün yakıt aralığında %5,2–%5,9'dur. Kanadın önündeki 1,75 kg'lık taret uçağı kararlı tutar. Taretsiz uçmak gerekirse taret bağlantısına (x = 1,22 m) en az 1,96 kg safra konmalıdır. Yalnız taretle (görev bilgisayarı ve tepsi yok) dolu depolarda statik marj %9,6'dır; bu yüklemede yakıt 10,8 kg ile sınırlıdır. İşletme kuralı: temel EO/IR seti her uçuşta takılıdır.

**Reddedilen seçenekler** (v1.4'te 20 kg yükle hesaplanan değerler; v1.6 ödünleşimleri §13'tedir):

| Seçenek | Dayanım (20 kg) | Neden reddedildi |
|---|---|---|
| Sabit, kaportalı iniş takımı | 11,26 h | kullanıcı içeri katlanır takımı açıkça seçti |
| Sabit (hep dışarıda) taret | 10,57 h | kullanıcı geri çekilir tareti açıkça seçti |
| Bekleme 1000 m'de | 9,38 h | görev tanımını değiştirir ve 10 h'e yetmez |
| MTOM'u artırmak | — | 149,9 kg M2 sınıfı tavanıdır (SHT-İHA) |
| R-52'nin v1.3 biçimi (yalnız takılı taretin tepe yükü) | 10,03 h | kullanıcı onaylamadı; yetenek kaybıdır |
| k_inst 0,95 kabul etmek | 10,02 h | dayanağı yoktur; test sonucudur, karar değildir |

**Bu bilinçli bir ödünleşimdir.** Kullanıcının seçtiği özellikler HANÇER'i benzersiz kılar, ama dayanıma bedeli vardır: içeri katlanır takım ve geri çekilir taret birlikte yaklaşık 2,2 h'e mal olur (§13). Karar bu özellikleri korur ve bedeli faydalı yükten öder: görev yükü 20 kg yerine 18,0 kg'dır. 20 kg yine taşınabilir; o zaman dayanım 9,64 h olur. Bu bir hesap düzeltmesi değil, gereksinim düzeyinde verilmiş bir karardır. Hiçbir kontrol, model ya da test gevşetilmedi.

## 1. Başlangıç: konvansiyonel referans kavramlar

Araştırma fazından sonra aynı motor, pervane, polar ve görev denklemleriyle üç konvansiyonel kavram çalışıldı ([`data/concepts/`](../data/concepts/)). Kullanıcı bunları "yine aynı tip" bularak reddetti; bu rapor onları yalnızca **ölçüt** olarak kullanır.

Hesap yöntemleri (pervane tablosu, BSFC, tripli polarlar, görev, kısıt diyagramı, yapı kütlesi) gözden geçirilmiş "Dayanım" çalışmasından alındı. `tests/test_ucav250_sizing.py`, itki modelinin o çalışmayla aynı denklemleri kullandığını sınar. Karşılaştırmada aynı pervane (32x18 2B), aynı elektrik yükü ve çalışmanın kendi kurulum katsayıları kullanılır.

| Kavram | Düzen | MTOM / boş / yakıt (kg) | CD0 | (L/D)maks | Dayanım | Neden reddedildi |
|---|---|---|---|---|---|---|
| Dayanım | omuz kanat AR 15, V-kuyruk, sabit kaportalı takım | 145 / 88,5 / 36,5 | 0,0331 | 16,9 | 14,77 h | klasik MALE görünümü |
| Kimlik | köşeli gövde, orta kanat, LERX tipi kök eldiveni | 145 / 90,5 / 34,5 | 0,0329 | 15,9 | 13,22 h | yine aynı tip |
| Üretim | çift kuyruk kirişi + H-kuyruk, üç parça kanat | 145 / 88,6 / 36,4 | 0,0375 | 14,7 | 13,19 h | yine aynı tip |

Ölçüt değerler `data/concepts/*/concept.yaml` dosyalarındandır. Bu kavramlar v1 yöntemiyle hesaplandı ve sonraki turlarda yeniden hesaplanmadı. Görev yakıtında hâlâ kapalı biçimli Breguet bağıntısını kullanırlar ve 20 kg yük taşırlar. HANÇER'e uygulanan düzeltmeler onlara da uygulansaydı dayanımları düşerdi. Bu düzeltmeler şunlardır: güce bağlı soğutma, kapanış taban sürüklemesi, birleşim sürüklemeleri, elektrik payları (v1.2); doğrudan yakıt akışı integrasyonu ve güç elektroniği verimi (v1.3); küt taban kurulum katsayısı, tam gazda kurulum kaybı ve integre alçalma (v1.4). Karşılaştırma bu yüzden farkın yönünü gösterir, büyüklüğünü göstermez.

## 2. Tasarım yönü çalışması ve kullanıcı seçimi

İkinci turda dört özgün yön aynı ön tahmin denklemleriyle karşılaştırıldı: [`data/concepts_v2/directions.yaml`](../data/concepts_v2/directions.yaml), [`compare.png`](../data/concepts_v2/compare.png). Her yönün ayrı bir sayfası vardır: [MANTA](../data/concepts_v2/manta/sheet.png), [OK](../data/concepts_v2/ok/sheet.png), [HALKA](../data/concepts_v2/halka/sheet.png), [HANÇER](../data/concepts_v2/hancer/sheet.png).

| Yön | Fikir | S (m²) | Açıklık (m) | AR | CD0 | (L/D)maks | Ön dayanım |
|---|---|---|---|---|---|---|---|
| MANTA | kanat-gövde birleşik (BWB) | 4,74 | 6,7 | 9,5 | 0,0255 | 15,8 | 17,0 h |
| OK | kanard-itici, ok kanat, kanat ucu dümenleri | 3,61 | 6,44 | 11,5 | 0,0285 | 15,9 | 15,3 h |
| HALKA | kutu / birleşik kanat | 4,77 | 5,5 | 6,3 | 0,029 | 13,1 | 13,9 h |
| **HANÇER** | köşeli taşıyıcı gövde + LERX + kanallı fan | 3,64 | 6,3 | 10,9 | 0,0312 | 14,8 | 14,3 h |

Ön tahminler kaba bir bileşen toplamı, sabit 33 kg yakıt ve 20 kg yükle yapılmıştı (±%20). **Kullanıcı D · HANÇER yönünü seçti** ve şu kararları verdi:

* Biçim dili korunacak: elmas benzeri kesitli, keskin kenar çizgili taşıyıcı gövde; kenar çizgileri LERX'e akar ve yüksek açıklık oranlı kanada kaynaşır; dışa eğik ikiz dikey kuyruk; tamamen hareketli yatay kuyruk.
* **Kanal (duct) reddedildi.** Pervane açık iticidir; dikey kuyruklar ve ventral kanatçık/kuyruk tamponu onu kısmen korur (§9).
* **İniş takımı içeri katlanır.** Kütlesi ve hacmi boyutlandırmada sayılır.
* **EO/IR taret gövde altındaki bölmeden indirilir.** Bölme, E180 büyüme zarfına göre boyutlanır.

Ayrıntılı boyutlandırmanın dayanımı ön tahminden belirgin biçimde düşüktür: 20 kg yükle 14,3 h yerine 9,64 h (görev yüküyle 10,41 h). Fark şu kaynaklardan gelir:

1. Boş kütle kalem kalem toplandı. Ön tahmin 92 kg kabul etmişti; sonuç 101,3 kg.
2. Bekleme taret dışarıda, 1,2 VS tabanında ve tripli polarlarla uçulur.
3. Görev 2 x 100 km geçiş ve %10 yedek içerir.
4. Doğrulamanın istediği sürükleme kalemleri ve gerçek kalkış fiziği eklendi.
5. Yakıt, her noktadaki yakıt akışının integraliyle bulunur ve jeneratör çıkışına dönüştürme kaybı uygulanır (v1.3).
6. E180 tepe yükü bütün bekleme boyunca karşılanır (R-52) ve küt taban nedeniyle itici kurulum katsayısı 0,93 tahmin edilir (v1.4).

## 3. Doğrulama bulguları ve düzeltmeler

### 3.1 Beşinci tur (V4-01…V4-08, v1.6)

Beşinci doğrulama, gereksinim kararının dürüst uygulandığını ve v1.5 sayılarının yeniden üretildiğini doğruladı; üç büyük ve beş küçük bulgu bildirdi. Her bulgu yeniden üretildi ve düzeltildi. Hiçbir kontrol ya da test gevşetilmedi; R-60 ve yük – dayanım tablosu sıkılaştırıldı.

| No | Önem | Bulgu | Yeniden üretim | Düzeltme | Sonuç (v1.6) |
|---|---|---|---|---|---|
| V4-01 | büyük | R-60 hiç başarısız olamıyor: aşağı kuvvet payı, gereken aşağı kuvvet azami değere kırpıldıktan sonra kaydediliyor | Doğru: `Flight.takeoff` hız sınırlı adımda F_t'yi azami değere eşitleyip payı ondan sonra yazıyordu; pay tanım gereği ≥ 0 idi | Gereken aşağı kuvvet kırpılmadan kaydedilir; pay negatif olabilir. R-60 ölçütü (`r60_metric`) ayrıca dönüş yasasının tutarlılığına bağlıdır: moment artığı ya da yerden kesilme sürekliliği 1e-6'yı aşarsa ya da hız sınırlı bir adım varsa ölçüt negatiftir. Yeni test, dönüşte azami aşağı kuvveti 0,9 ile çarpar ve `--check` değerlendirmesinin R-60'ı düşürdüğünü doğrular | R-60 +0,59 N ✔; zorlanmış durumda −17,6 N ✘ (test) |
| V4-02 | büyük | Yük – dayanım tablosunun 0 kg satırı (13,23 h) R-09'un dışında bir yükleme; R-52 tablosu E180 taşıyamayan yüklere E180 değeri veriyor | Doğru: tareti olmayan uçağın statik marjı %5,2–%5,9; yalnız taretle dolu depolarda %9,6 | Tablo ve şekil izin verilen en hafif yüklemeden (temel EO/IR seti, 3,25 kg) başlar; her satır statik marj aralığını taşır. Yüksüz yükleme "izin verilmez" diye ayrıca raporlanır (taret bağlantısında 1,96 kg safra gerekir); yalnız taretle yakıt sınırı 10,8 kg. `full_fuel_baseline_sensors_only` durumu dolu depolu oldu (`fuel_rule: capacity`, 35,95 kg); R-09, R-10 ve ağırlık merkezi zarfı onu da denetler. E180 sütunları yalnız E180 setini (5,5 kg) taşıyabilen yüklerde | 12 satırın hepsinde SM ≥ %10,1 ✔ |
| V4-03 | büyük | V3-10 kapak düzeni (altı kapak, sıralı iç kapaklar, bacak kapağı ayakları) kütle kalemine girmemiş; v1.5 görev yükünün boş kütle payı yalnız ~0,06 kg | Doğru: kalem kimlik çalışmasının beş kapak / 0,15 m² toplamıydı (1,60 kg); kapak eyleyicisi yoktu; v1.5 kapak alanı 0,192 m² | Kalem kapak düzeninden kuruldu (`gear_doors_mass`): 0,192 m² × 2,733 kg/m² (kimlik çalışması 0,41 kg / 0,15 m²) 0,525 kg + kuyu kapatmaları 0,36 + kesik takviyesi 0,50 + kilit/algılayıcı 0,25 + conta 0,101 (4,28 m birleşim × 23,5 g/m; ayırma çizgileri dahil) + iki iç kapak eyleyicisi (Volz DA 22, 2 × 0,132 kg veri sayfası) + bağlantıları 0,08 + dört bacak kapağı ayağı 0,08 = 2,160 kg (büyüme payıyla 2,268 kg; v1.5 1,68 kg). Conta birleşim sürüklemesi de 3,39 m'den 4,28 m'ye çıktı. `--update-spec` kuralı yeniden uyguladı. Takım tavanı 14,34, bağlantı elemanları 1,51, kuyruk 7,89 kg oldu | görev yükü 18,0 kg (10,41 h); eşiğe pay 0,162 h = 0,42 kg boş kütle; R-56 +0,255 kg ✔ |
| V4-04 | küçük | §16 madde 10, görev yükünün ne zaman değiştiğini yanlış anlatıyor | Doğru: R-56 payı 10 h'e göredir, kural eşiği 10,25 h'tir | İki pay ayrı yazıldı (§14, §16): R-02 18,0 kg ile +0,96 kg boş kütleye dayanır; R-56'yı belirleyen R-02b +0,25 kg; türetilen görev yükü +0,42 kg'dan sonra 17,5 kg'a iner | ✔ |
| V4-05 | küçük | Dönüş geçişi "I_yy bilinmiyor" diye ertelenmiş | Doğru: kütle kalemlerinden noktasal toplam 91,8 kg·m² | I_yy kütle kalemlerinden tahmin edilir (`distributed_pitch_inertia`: kaplama, kanat ve kuyruk yüzeyleri ıslak alanlarına yayılı; omurga, çerçeveler, kenar şeritleri ve kablo demeti çubuk; yakıt hücre dikdörtgenlerine yayılı). Dönüş yasası, yunuslama hızı komutunu 0,3 s'de rampalar ve ana teker noktasına göre (I_yy + m a²) θ̈ momentini ister; V_R bu momentin sağlandığı hızdır | I_yy 116,7–121,3 kg·m²; V_R 24,58 → 24,76 m/s, koşu 175,6 → 178,7 m (0,1 s rampayla 180,7 m) ✔ |
| V4-06 | küçük | Şekil kusurları: yük – dayanım taraması hacim kırılmasını atlıyor; AM zarfında MTOM işaretleri gizli; polar açıklaması eğrileri örtüyor; kısıt diyagramı dipnotunu çizgiler kesiyor | Doğru | Taramaya kırılma yükü (12,61 kg) ve 0,5 kg komşuları (12,5 / 13,0) eklendi; çakışan durumlar iç içe halkalar (ilki en büyük ve en altta); polar açıklaması panellerin altında; dipnot eksenlerin altında | ✔ (§5, §6) |
| V4-07 | küçük | İçeri katlanmış alt görüntü kapak düzenine uymuyor | Doğru: her kuyu çifti için ortadan bölünmüş tek dikdörtgen çiziliyordu | İç kapaklar (y ±0,19 m, ortadan bölünmüş) ve iki bacak kapağı ayrı çizilir; görüntü yeniden üretildi | ✔ (§10) |
| V4-08 | küçük | Küçük metin hataları | Doğru | Trim sürekliliği "≈1e-13 (< 1e-12)"; hücrelerin dolu olduğu yük sınırı v1.6'da 12,6 kg; Willans doğrusu "daha ihtiyatlı bir dışdeğerleme (sınır değil)"; sizing.md'de kural metni Türkçe | ✔ |

### 3.2 Dördüncü tur (V3-01…V3-11, v1.5)

Bu tablo v1.5'in kaydıdır; "Sonuç (v1.5)" sütunu o revizyonun değerleridir (V3-03 ve V3-07 satırlarındaki iki metin hatası V4-08 ile düzeltildi; v1.6 değerleri §3.1 ve sonraki bölümlerdedir).

Üçüncü doğrulama turunun sonunda `--check` 1 ile çıkıyordu (R-02 9,899 h, R-56 −0,361 kg) ve 89 testten 4'ü başarısızdı. Her bulgu yeniden üretildi ve düzeltildi. Bu turda hiçbir kontrol ya da test gevşetilmedi. R-02/R-03/R-56'nın yeni biçimi bir düzeltme değil, proje liderinin gereksinim kararıdır (önceki bölüm).

| No | Önem | Bulgu | Yeniden üretim | Düzeltme | Sonuç (v1.5) |
|---|---|---|---|---|---|
| V3-01 | kritik | R-02 ve R-56 karşılanmıyor; `--check` 1 ile çıkıyor; 4 test başarısız | Doğru: 9,899 h ve −0,361 kg | Gereksinim kararı: R-02 tasarım görevi yüküyle (kuralla türetilen 19,0 kg), yeni R-02b (20 kg ≥ 9,5 h), R-03 azami yük, R-56 iki gereksinime göre. Görev yükü `evaluate` içinde türetilir, `--update-spec` kapanışla birlikte yineler, `--check` karşılaştırır. Azami yük için yükleme durumları (MTOM, taret içeride/dışarıda, yakıtsız) eklendi; kalkış, AM zarfı ve kararlılık bu yükle de yapılır. R-52'nin jeneratör açığı sınırı uzayan bekleme için 17 W'a indirildi | R-02 10,27 h, R-02b 9,88 h, R-56 +0,600 kg ✔; `--check` çıkış 0; testlerin hepsi geçiyor |
| V3-02 | büyük | `avionics_power_bay` bildirdiği içerikleri taşıyamıyor; R-26 içeriği değil yalnız kutuyu denetliyor | Doğru: 0,26 × 0,14 × 0,06 m'lik kutuya 235 × 195 × 54,5 mm'lik PDU bile girmez; 21700 hücre boyu 70 mm'dir, kutu yüksekliği 60 mm | Burun bölmesi üç bölgeye ayrıldı: burun takımı kuyusunun üstünde güç anahtarlama bölmesi ve PDU güvertesi; kuyunun iki yanında yan bölmeler. Bileşenler gerçek zarflarıyla yerleştirildi: components.yaml boyutları, hücre veri sayfasından batarya paketi, birim seçilmemiş kalemler için ayırma kutuları. Her zarfa bağlayıcı payı eklendi. `bay_contents_check` her zarfı OML (≥ 10 mm), bölge duvarı (≥ 3 mm) ve komşu zarflar (≥ 10 mm) için denetler; sonuç R-26'ya girer. Kütle kalemleri paketlenmiş içeriklerin merkezlerindedir. v1.4 kutusunun bu kontrolle başarısız olduğunu bir test gösterir | 9 öğenin hepsi sığar (§12); R-26 ✔ |
| V3-03 | büyük | Kalkışta "dönüş sırasında stabilatör güç-açık trime döner" kuralı ana tekerlere göre moment dengesini bozuyor | Doğru: tekerlek tepkisi ağırlık merkezinin gerisinde olduğundan trim aşağı kuvvetiyle net moment burun aşağıdır; dönüş sürdürülemez | Dönüş yasası yeniden tanımlandı. Sabit yunuslama hızında, burun tekeri yerden kesik ve açısal ivme sıfırken stabilatör aşağı kuvveti, ana teker temas noktasına göre moment dengesini sağlayan değerdir. Bu değer V_R'deki tam burun yukarı değerden, ana teker tepkisinin sıfırlandığı anda güç-açık trim değerine iner. Momentler gerçek tutumda, yer çerçevesinde alınır. Her dönüş adımında gereken aşağı kuvvet azami değerle karşılaştırılır. Yeni R-60 eklendi | moment artığı ≈1e-13 N·m, yerden kesilmede trim sürekliliği farkı ≈1e-13 N (< 1e-12; V4-08 düzeltmesi); R-60 ✔ (§7) |
| V3-04 | küçük | Doc 02 §15 üç testin başarısız olduğunu söylüyor; dört test başarısızdı | Doğru (`test_fix_round2_checks` de R-56 yüzünden başarısızdı) | §15 yeniden yazıldı | testlerin hepsi geçiyor |
| V3-05 | küçük | Spec'te bayat ya da tutarsız metin | Doğru: `meta.description_tr` ("R-02 karşılanmıyor"), `configuration.reasons_tr`, R-50 ve `propeller.sources.k_inst` ("alt sınır"), aviyonik bölmesi içeriği, batarya yeri, E180 yükü kalemi ("20 kg tasarım yükü") | Metinler v1.5'e göre yeniden yazıldı; revizyon v1.5 | ✔ |
| V3-06 | küçük | k_inst 0,93 dayanaksız biçimde "alt sınır" diye sunuluyor | Doğru: aero.yaml'daki 0,93–0,97 aralığı mühendislik yargısıdır, test değildir; küt taban durumunu sınırlamaz | k_inst 0,93 her yerde **tahmin** diye anılır. Dayanım duyarlılığı k_inst 0,90 / 0,95 / 0,97 için hesaplanır (§6.5) | 10,08 / 10,39 / 10,51 h |
| V3-07 | küçük | Alçalma yakıtı, %5 güce dışdeğerlenmiş BSFC'ye dayanıyor | Doğru: alçalma %5,1–6,4, yedek bekleme %17,8 güçtedir; en düşük BSFC noktası %20'dedir | Sınır hesaplandı. Yakıt akışı güçle doğrusal varsayıldı (Willans doğrusu, en düşük iki noktadan); bu, alçak güçte yakıt akışını sıfıra götürmeyen daha ihtiyatlı bir dışdeğerlemedir, bir sınır değildir (%20 gücün altında veri yok; V4-08 düzeltmesi). Bu varsayım §16'da sınırlama olarak listelendi | alçalma 0,237 → 0,361 kg; dayanım 10,27 → 10,21 h (R-02 yine ✔) |
| V3-08 | küçük | E180 büyüme görevi HD59 sürekli elektrik yüküyle uçuluyor | Doğru | E180 görevinin motoru E180 sürekli yükünü (373 W) taşır; jeneratör mil çekişi ve alçalma devir tabanı buna göredir | E180 görevi 10,02 h; alçalma 3.720 rpm |
| V3-09 | küçük | R-52 batarya tepe desteği yalnız tasarım görevinin bekleme süresine göre boyutlanmış; işletme sınırı yazılmamış | Doğru: daha hafif yükte bekleme uzar ve pay biter (15 kg'da 148 Wh > 124,5 Wh) | İşletme sınırı yazıldı ve her yük için desteklenen bekleme süresi tabloya kondu (§12). Pay bitince güç yönetimi bekleme devrini, jeneratörün E180 tepesini tek başına taşıdığı devre (4.917 rpm) çıkarır ya da tepe yük süresi sınırlanır | açık sınırında en az 7,33 h; görev yüküyle bütün bekleme ✔ |
| V3-10 | küçük | Takım açık ve taret dışarıda görüntülerinde kuyu, kapak ve açıklık yok | Doğru | Kapak düzeni spec'e yazıldı (`landing_gear.doors`). Burun yuvası için iki istiridye kapak açık çizilir. Ana kuyularda iç kapaklar sıralıdır, takım kilitlenince kapanır; bacak kapakları bacakla döner ve bacak yarığı açık kalır. Taret dışarıda iken açıklık halkası ve topun çevresindeki açık halka görünür | §10, §11 görüntüleri |
| V3-11 | küçük | Şekil kusurları: üst üste binen etiketler, gizlenen işaret, yanıltıcı polar etiketi | Doğru: faydalı yük – dayanım şeklinde etiketler çakışıyordu; AM zarfında iki MTOM işareti üst üste düşüyordu; polar şeklindeki "bekleme CL 0,92 (MTOM)" görev beklemesi değildi ve "takım açık (kalkış/iniş)" poları flapsızdı | Faydalı yük – dayanım şekli yeniden çizildi (hacim sınırı tek bant, R-02 / kural / R-02b çizgileri). AM zarfında her durumun kendi simgesi ve boyutu var. Polarlarda görev beklemesinin CL aralığı bant olarak gösterilir ve takım açık polarının flapsız olduğu yazılır. Kısıt diyagramındaki etiket kaydırıldı | ✔ (§5, §6) |

### 3.3 Üçüncü tur (V2-01…V2-08, v1.4)

Bu tablo v1.4'ün kaydıdır; "Sonuç (v1.4)" sütunu o revizyonun değerleridir. V2-05'in dönüş kuralı v1.5'te V3-03 ile değişti.

| No | Önem | Bulgu | Yeniden üretim | Düzeltme | Sonuç (v1.4) |
|---|---|---|---|---|---|
| V2-01 | büyük | R-02 yalnız R-52 gevşetildiği için karşılanıyor; doc 02 iki yerde "gevşetilmedi" diyor | Doğru. v1.3 modelinde tasarım görevi E180 devir tabanıyla uçulursa 9,55 h (gevşetmenin katkısı +0,52 h) | Kullanıcı R-52 değişikliğini onaylamadı. R-52 v1.2 yeteneğine döndü: E180 tepe yükü bütün bekleme boyunca karşılanır. Bunu tasarım sağlar: LiFePO4 14S2P (2,43 kg, 231 Wh) yerine 12S2P Li-ion Molicel P45B (1,98 kg, 389 Wh). Kullanılabilir 311,2 Wh'den jeneratör kaybı/marş yedeği 186,7 Wh ayrılır; kalan 124,5 Wh tepe destek payıdır | R-52 1,029 ✔; R-02 9,90 h ✘ |
| V2-02 | büyük | Arka kapanış küt bir "kutu"; k_inst 0,95 kanıtsız | Doğru: x = 3,91 m'den sonra 59° eğim, taban 0,112 m² | Sivrilen kuyruk konisi denendi ve reddedildi (§13). Küt taban korundu; kurulum katsayısı aralığın alt değeri olan 0,93 alındı (v1.5'te "tahmin" olarak düzeltildi, V3-06) ve tam gaza da uygulandı | taban 0,112 m² (CD 0,0022); k_inst 0,93 |
| V2-03 | büyük | Dümen eyleyicisinin menteşe momenti kontrolü yok | Doğru: dümen VA'da tam sapmada 6,5 N·m; DA 26 doğrudan bağlantıyla 2,7 N·m | Bütün kumanda yüzeylerinde menteşe momenti ve dört çubuk bağlantı kontrolü; R-57…R-59 | R-57 1,235, R-58 1,83, R-59 6,7° ✔ |
| V2-04 | küçük | Tam gaz noktalarında kurulum katsayısı yok | Doğru | Tam gaz itkisi × [1 − (1 − k_inst) min(V/15 m/s, 1)] | tırmanma 4,21 m/s, tavan 7.105 m |
| V2-05 | küçük | Kalkış dizisi fiziksel değil (V_R düz kalkış hızının üstünde) | Doğru | Gerçek yerden kesilme; uçuş kontrol sisteminin stabilatör programı gereksinim olarak yazıldı (dönüş kuralı v1.5'te V3-03 ile değişti) | V_R 24,8, V_LOF 25,1 m/s |
| V2-06 | küçük | Stabilatörün "3:1 bellcrank" hareketi doğrusal oran gibi hesaplanmış | Doğru: simetrik 3:1 dört çubuk en çok asin(1/3) = 19,5° verir | Dört çubuk bağlantının tam kinematiği; stabilatör 2,5:1 | R-37 1,80, R-47 1,13 ✔ |
| V2-07 | küçük | Alçalma sabit kesirle hesaplanıyor | Doğru | Alçalma jeneratör devir tabanında integre edilir (v1.5'te BSFC dışdeğerlemesi sınırlandı, V3-07) | 20,0 dk, 0,236 kg |
| V2-08 | küçük | Bayat metin ve şekil ayrıntıları | Doğru | Metinler ve şekiller düzeltildi; içeri katlanmış durum alttan çizildi | ✔ |

### 3.4 İkinci tur (V1-01…V1-11, v1.3)

**Düzeltme notu (v1.4, V2-01):** v1.3 raporu bu turda "hiçbir gereksinim gevşetilmedi" diyordu; bu doğru değildi, çünkü R-52 gevşetilmişti. v1.4'te gevşetme geri alındı.

| No | Önem | Bulgu | Düzeltme | v1.6 durumu |
|---|---|---|---|---|
| V1-01 | kritik | Görev yakıtı, aşağı itkinin kaldırma bileşenini içeren CL/CD ile Breguet bağıntısından hesaplanıyor; yakıt ~%1 az çıkıyor | Kütle değişimi dW/dt = −ṁ(nokta) doğrudan zamanla integre edilir; itki T = D/cos ε; alçalma da integre edilir | ✔ (regresyon testleri) |
| V1-02 | kritik | SG750'nin AC anma gücü DC bara gücü sayılıyor | DC çıkış = 800 W × rpm/7500 × 0,94 | R-32 1,276, R-52 1,002 ✔ |
| V1-03 | büyük | Kütle bütçesi R-02'yi kapatan boş kütleden fazla | Bütçe tek yönlü bir tavandır (R-56); v1.5'ten beri R-02 ve R-02b'ye göre | R-56 +0,255 kg ✔ |
| V1-04 | büyük | Gömülü dikey kökü motor silindirleri ve mille çakışıyor | Kuyruk gövdeleri dış yüzeyde budanır; 20 mm'lik bağlantı bandı; R-53 | ✔ |
| V1-05 | büyük | Menteşe momentinde panel CN_maks yerine trim sabiti | Panel kesitinin CN_maks değeri (1,390); bağlantı dört çubuk kinematiğiyle | H_VA 17,86 N·m; R-37 1,79, R-47 1,12 ✔ |
| V1-06 | küçük | LERX–eldiven geçişinde görünür kırık | NACA 0016-34 → NLF(1)-0416 karışımı, aile değişmez | ✔ |
| V1-07 | küçük | Açıklık halkası V yüzeyinde değil | Halka gerçek V yüzeyini izler | ✔ |
| V1-08 | küçük | MTOM 3000 m noktası "bekleme başlangıcı" diye etiketli | Görev beklemesinin başlangıç/bitiş noktaları ayrıca raporlanır | ✔ |
| V1-09 | küçük | VNO/VNE ve hız koruması yok | CS-LUAS 1505; R-54 | ✔ |
| V1-10 | küçük | MTOM'da iniş için gereksinim yok | R-55 | 219,6 m ✔ |
| V1-11 | küçük | Kapanış iki değer arasında gidip geliyor | Yakınsama ölçütü, tek yönlü kuyruk kuralları | ✔ |

### 3.5 Birinci tur (F1–F19)

Birinci doğrulama ilk boyutlandırmayı "henüz hazır değil" olarak değerlendirmişti. On dokuz bulgunun hepsi v1.1/v1.2'de işlendi ve R-33…R-52 eklendi. "Sonuç" sütunu v1.6 tasarım noktasındaki değerdir.

| No | Bulgu (özet) | Düzeltme | Sonuç (v1.6) |
|---|---|---|---|
| F1 | Kalkış koşusu R-06'yı bozuyor; "düz kalkış" fiziksel değil | V_R moment dengesinden; yer koşusu zamanda integre; gerçek yerden kesilme ve dönüş yasası (V2-05, V3-03) | koşu 179 m ≤ 200 ✔; V_R 24,8, V_LOF 25,2 m/s; ana tekerde 113 N ✔ |
| F2 | Stabilatör mili panel AM'sinin önünde; menteşe momentleri eyleyicinin ~5 katı | Mil VLM AM bandının ön ucunda; DA 30 + 2,5:1 dört çubuk | R-37 1,79, R-47 1,12 ✔ |
| F3 | Stabilatör kökü gövdeye giriyor | Sabit kök parçası; R-38, R-39 | boşluk 8,0 mm, gövdeye pay 23,2 mm ✔ |
| F4 | Dikey kökü gövdeden havada | Kökler gövdeye kapanır, dış yüzeyde budanır (R-40, R-53) | ✔ |
| F5 | Stabilatör kuralı η_t'yi ve tam gaz itki momentini atlıyor | Yerel stabilatör CL'si; tam gazda yerden kesilme ve pas geçme (R-36) | 0,695 ≤ 0,72 ✔; stabilatör 0,624 m² |
| F6 | Pervane yer açıklığı yalnız statik tutumda | R-16 statik/yerden kesilme/teker koyma; R-33, R-34 | R-16 0,184 m ✔; radyal 32,9 mm, boyuna 29,4 mm ✔ |
| F7 | Kök eldiveninde kiriş derinliği yetersiz | Eldiven kalınlığı gerekli derinlikten (R-43, R-44) | ana 90,8 mm, arka 41,1 mm ✔ |
| F8 | LERX/kenar çizgisi geçişinde basamak | Kenar çizgisi kanat düzleminde tutulur (R-45) | 0,0 mm ✔ |
| F9 | Rüzgâr hamlesi yükü yanlış eğimle | Yapılandırma CLα'sı, kütle × irtifa matrisi | kanat tasarım n'si 5,47; en hafif durumda +7,02 g |
| F10 | Eğik dikeyler pervaneyi yandan korumuyor | Koruma eğik disk düzleminde ölçülür (R-41/42, R-48/49, R-51) | yan bölgeler açık; **yan koruma iddia edilmez** (§9) |
| F11 | Flap panel bağlantısının üzerinden geçiyor | Her dış panelde bir flap (R-46) | ✔ |
| F12 | Bu düzene özgü sürükleme kalemleri yok | Taban, taret bölmesi, birleşimler, kapaklar, soğutma, itici kurulumu | CD0 0,0352; dayanım 10,41 h görev yüküyle ✔ |
| F13 | Taret görüş testi yalnız gövdeye bakıyor | Işın izleme; engeller kanatlar, kuyruklar, ventral, pervane diski | R-25 −3° ✔ |
| F14 | Bayat sayılar | Rapor sayıları sizing.json'dan; TestDoc02 | ✔ |
| F15 | Spec şemadan sapıyor | Kuyruk kumanda blokları, aero anahtarları | ✔ |
| F16 | R-32 jeneratör payı yalnız 303 W'la | Sürekli yük E180 + araştırma payıyla (R-32); tepe yük R-52'de (batarya destekli) | R-32 1,276, R-52 1,002 ✔ |
| F17 | Kaba UIUC naca0010.dat | Analitik NACA 0010 | ✔ |
| F18 | `--check` ve testler depoya yazıyor | `--check` salt okunur | ✔ |
| F19 | Paraşüt bağlantı kaleminin adı | `parachute_attach_fitting` | ✔ |

## 4. Gereksinimler

Gereksinimler `spec.yaml → requirements` altındadır. Her birinin ölçütü `out/sizing.json → metrics` içindeki bir anahtardır ve kaynağı dosyada yazılıdır. R-33…R-46 birinci doğrulamadan, R-47…R-52 v1.2'den, R-53…R-56 ikinci, R-57…R-59 üçüncü, R-02b ve R-60 dördüncü turdan (v1.5) gelir. v1.6'da yeni gereksinim yoktur; R-60 sıkılaştırıldı (beşinci tur).

| No | Gereksinim | Hedef | Sonuç | |
|---|---|---|---|---|
| R-01 | Azami kalkış kütlesi 150 kg'ın altında (SHT-İHA M2 sınıfı, 149,9 kg tavan) | ≤ 149,9 kg | 149,9 kg | ✔ |
| R-02 | Tasarım görevi dayanımı ≥ 10 h, tasarım görevi faydalı yüküyle (mission.payload_design_kg: MTOM 149,9 kg'da aynı görevde ≥ 10,25 h veren, 0,5 kg'a aşağı yuvarlanmış en büyük faydalı yük; tırmanma, 2 x 100 km geçiş, 3000 m'de taret dışarıda bekleme, alçalma; %10 yedek ayrıca) | ≥ 10,00 h | 10,41 h | ✔ |
| R-02b | Azami faydalı yükle (20 kg) aynı tasarım görevinde dayanım ≥ 9,5 h (MTOM 149,9 kg; yakıt = MTOM − boş kütle − 20 kg; azami yükün kendi ağırlık merkezi) | ≥ 9,50 h | 9,64 h | ✔ |
| R-03 | Azami faydalı yük ≥ 20 kg: MTOM'daki yükleme durumlarından biri 20 kg faydalı yük taşır ve bütün kontroller (ağırlık merkezi zarfı, statik marj, kalkış, R-02b) bu yükle de yapılır | ≥ 20,0 kg | 20,0 kg | ✔ |
| R-04 | Servis tavanı (0,5 m/s) ≥ 4500 m, MTOM | ≥ 4.500 m | 7.100 m | ✔ |
| R-05 | Deniz seviyesi tırmanma hızı ≥ 4,0 m/s, MTOM, tam gaz (4,9 m/s karşılaştırma hedefi pervane sınırlı; bkz. pervane ödünleşimi) | ≥ 4,0 m/s | 4,2 m/s | ✔ |
| R-06 | Kalkış yer koşusu ≤ 200 m (MTOM, ISA deniz seviyesi; 300 m pist / 1,5) | ≤ 200,0 m | 178,7 m | ✔ |
| R-07 | İniş yer koşusu ≤ 200 m (görev sonu kütlesi, ISA deniz seviyesi) | ≤ 200,0 m | 182,6 m | ✔ |
| R-08 | Tutunma hızı ≤ 24 m/s (temiz, trimli, MTOM, deniz seviyesi) | ≤ 24,0 m/s | 23,9 m/s | ✔ |
| R-09 | Statik marj ≥ %10 OAK (tüm yükleme durumları, öndeki nötr nokta) | ≥ 0,100 | 0,101 | ✔ |
| R-10 | Statik marj ≤ %30 OAK (tüm yükleme durumları) | ≤ 0,300 | 0,146 | ✔ |
| R-11 | Yön kararlılığı Cnβ ≥ 0,057 1/rad (0,001 1/derece) | ≥ 0,057 1/rad | 0,059 1/rad | ✔ |
| R-12 | Geri devrilme açısı ≥ 15° (en arka zemin ağırlık merkezi) | ≥ 15,0 deg | 38,4 deg | ✔ |
| R-13 | Yana devrilme açısı ≤ 55° | ≤ 55,0 deg | 54,0 deg | ✔ |
| R-14 | Burun tekerleği yükü ≥ %8 (en arka ağırlık merkezi) | ≥ 0,080 | 0,161 | ✔ |
| R-15 | Burun tekerleği yükü ≤ %20 (en ön ağırlık merkezi) | ≤ 0,200 | 0,171 | ✔ |
| R-16 | Pervane yer açıklığı ≥ 0,18 m: statik tutum, kalkış (yerden kesilme) tutumu ve teker koyma tutumunun en kritiği (MTOM; kalkışta amortisör statik çökmede, teker koymada yüksüz) | ≥ 0,1800 m | 0,1836 m | ✔ |
| R-17 | Sönük ana lastik + dibe oturmuş amortisörde pozitif pervane açıklığı (≥ 0,02 m) | ≥ 0,0200 m | 0,0597 m | ✔ |
| R-18 | Kuyruk tamponu pervaneden önce yere değer (açı farkı ≥ 1°) | ≥ 1,000 deg | 4,128 deg | ✔ |
| R-19 | Kalkış ve flare açısında kuyruk tamponu yere değmez (pay ≥ 2°) | ≥ 2,0 deg | 2,3 deg | ✔ |
| R-20 | Teker koyma açısı ≥ 3° (ana tekerler önce değer) | ≥ 3,0 deg | 4,9 deg | ✔ |
| R-21 | Ana iniş takımı gövde içine toplanır (teker, bacak, mafsal; kanat kutusu ile çakışma yok) | = 1,000 | 1,000 | ✔ |
| R-22 | Burun iniş takımı omurga yuvasına toplanır | = 1,000 | 1,000 | ✔ |
| R-23 | Taret içeride kapakların üstünde kalır (gömülü) | ≥ 0,0000 m | 0,0265 m | ✔ |
| R-24 | E180 büyüme zarfı (0,18 x 0,23 m) taret bölmesine sığar | = 1,000 | 1,000 | ✔ |
| R-25 | Taret dışarıda: nadirden en az −5° yükselime kadar her azimutta engelsiz görüş | ≥ −5,0 deg | −3,0 deg | ✔ |
| R-26 | Yerleşim bölgeleri gövdeye sığar ve bildirilen içerikleri (aviyonik/güç bölmeleri: gerçek bileşen zarfları + bağlayıcı payı) kendi bölgelerine, gövdeye ve birbirlerine açıklıklarıyla sığar (hata sayısı 0) | = 0,000 | 0,000 | ✔ |
| R-27 | Yerleşim bölgeleri ve yakıt hücreleri çakışmaz (çakışma sayısı 0) | = 0,000 | 0,000 | ✔ |
| R-28 | Yakıt hacmi gereken hacmi karşılar (pay ≥ 0) | ≥ 0,000 | 0,176 | ✔ |
| R-29 | Taşıma: çıkarılabilir dış kanat paneli ≤ 3,4 m | ≤ 3,4 m | 2,9 m | ✔ |
| R-30 | Gövde + LERX orta kesiti ≤ 2,0 m (tek parça taşıma) | ≤ 2,0 m | 1,4 m | ✔ |
| R-31 | Statik pervane uç Mach sayısı ≤ 0,75 | ≤ 0,750 | 0,712 | ✔ |
| R-32 | Jeneratör çıkışı, beklemenin en düşük devrinde, en büyük sürekli elektrik yükünün ≥ 1,2 katı (E180 büyüme tareti ve araştırma yükü güç payı dahil) | ≥ 1,200 | 1,276 | ✔ |
| R-33 | Pervane uçları ile yapı arasında radyal açıklık ≥ 0,026 m (CS-VLA 925(c)(1)) | ≥ 0,0260 m | 0,0329 m | ✔ |
| R-34 | Pervane palaları ile sabit yapı (kaporta, kuyruk yüzeyleri) arasında boyuna açıklık ≥ 0,013 m (CS-VLA 925(c)(2)) | ≥ 0,0130 m | 0,0294 m | ✔ |
| R-35 | Kalkışta dönme yetkisi: stabilatörler burun tekerini ana tekerler hâlâ yüklüyken kaldırır (tekerlek arabası etkisi yok; bütün MTOM yükleme durumları) | > 0,0 N | 112,6 N | ✔ |
| R-36 | Stabilatör trim gereksinimi (yerel CL, η_t dahil; temiz/kalkış CLmax ve tam güçte yerden kesilme/pas geçme, en ön ağırlık merkezleri; yerden kesilmede takım açık) ≤ 0,8 × stabilatör CLmax | ≤ 0,720 | 0,695 | ✔ |
| R-37 | Stabilatör eyleyicisi: tepe tork × dört çubuk bağlantısının o sapmadaki tork oranı ≥ 1,25 × en büyük menteşe momenti, bütün sapma aralığında (CS-LUAS.395(a)(1)) | ≥ 1,000 | 1,792 | ✔ |
| R-38 | Stabilatör kök boşluğu ≤ 10 mm (sabit kök parçasına karşı, bütün sapma aralığında sabit) | ≤ 0,0100 m | 0,0080 m | ✔ |
| R-39 | Stabilatör kökü bütün sapma aralığında gövdeye girmez (gövde yarı genişliği ile pay ≥ 5 mm) | ≥ 0,0050 m | 0,0233 m | ✔ |
| R-40 | Dikey kuyruk ve ventral kanatçık kökleri bütün kök veteri boyunca gövdeye gömülü (boşluk ≤ 0) | ≤ 0,0000 m | −0,0150 m | ✔ |
| R-41 | Pervane koruması (üst-yan): eğik dikeylerin firar kenarı, itki hattı açısıyla eğik pervane disk düzlemini uç çemberinin ≥ 26 mm dışında keser | ≥ 0,0260 m | 0,0750 m | ✔ |
| R-42 | Pervane koruması (üst-yan): eğik dikeylerin firar kenarı disk düzlemini uç çemberinin ≤ 80 mm dışında keser (diski yakından çevreler) | ≤ 0,0800 m | 0,0750 m | ✔ |
| R-43 | Ana kiriş hattında OML derinliği (gövde yanından dış panel bağlantısına) ≥ birleşim kalınlığının %95'i | ≥ 0,950 | 0,977 | ✔ |
| R-44 | Arka kiriş hattında OML derinliği (gövde yanından dış panel bağlantısına) ≥ dış panel bağlantı kesitinin arka kiriş derinliğinin %95'i (arka kiriş kademesiz devam eder) | ≥ 0,950 | 1,002 | ✔ |
| R-45 | Kanat kökü ile köşe çizgisi arasında basamak ≤ 10 mm (LERX/köşe çizgisi geçişi) | ≤ 0,0100 m | 0,0000 m | ✔ |
| R-46 | Flap, sökülebilir dış panelin üzerinde kalır (iç ucu panel bağlantısının dışında) | ≥ 0,0000 m | 0,0301 m | ✔ |
| R-47 | Stabilatör eyleyicisi: anma (sürekli) torku × dört çubuk bağlantısının o sapmadaki tork oranı en büyük menteşe momentinin ≥ 1,1 katı, bütün sapma aralığında (mil, girdap kafesi AM bandının ön ucunda) | ≥ 1,100 | 1,120 | ✔ |
| R-48 | Pervane koruması (alt): ventral kanatçığın firar kenarı eğik disk düzlemini uç çemberinin ≥ 26 mm dışında keser; değiştirilebilir tampon kızağı diskin altındadır | ≥ 0,0260 m | 0,0522 m | ✔ |
| R-49 | Pervane koruması (alt): ventral kanatçığın firar kenarı disk düzlemini uç çemberinin ≤ 120 mm dışında keser (alt bölgeyi yakından korur) | ≤ 0,1200 m | 0,0522 m | ✔ |
| R-50 | Pervane düzlemi kaporta firar kenarının ≥ 0,10 D gerisinde (itici pervane gövde izinde; kaporta tabanı küt olduğundan kurulum katsayısı k_inst için 0,93 tahmini kullanılır, duyarlılık doc 02 §6.5) | ≥ 0,100 | 0,102 | ✔ |
| R-51 | Yanal koruma: uçak kanat ucu yere değene kadar yatarsa pervane ucu yerden ≥ 0,05 m yukarıda kalır (kanat ucu önce değer) | ≥ 0,0500 m | 0,3192 m | ✔ |
| R-52 | E180 büyüme taretinin tepe elektrik yükü (+ temel yük + araştırma yükü payı) tasarım görevinin beklemesi boyunca kesintisiz karşılanır (v1.2 yeteneği): jeneratörün DC çıkışı (güç elektroniği verimi dahil) ile tampon bataryanın tepe destek payı birlikte; tepe yükün bütün bekleme boyunca sürdüğü varsayılır (görev çevrimi kredisi yok); tepe destek payı = kullanılabilir batarya enerjisi − jeneratör kaybı/marş yedeği (400 W × 28 dk); E180 büyüme görevi de aynı kuralla kontrol edilir | ≥ 1,000 | 1,002 | ✔ |
| R-53 | Kuyruk kök yapıları (gövde dış yüzeyinde budanmış dikey, sabit stabilatör kök parçası ve ventral kökleri: kök kaburgası + bağlantı bandı) ve stabilatör milleri iç yerleşim bölgeleriyle ve birbirleriyle çakışmaz (çakışma sayısı 0) | = 0,000 | 0,000 | ✔ |
| R-54 | İşletme sınırları (CS-LUAS 1505): VNE = 0,9 VD, VNO = min(VC; 0,89 VNE); uçuş kontrol sisteminin zarf koruması hızı VNO'nun altında tutar (sınır hız ≤ VNO; tam güçte düz uçuş hızı VNE'yi aşar) | ≥ 0,000 m/s | 0,000 m/s | ✔ |
| R-55 | MTOM'da acil/geri dönüş inişi (flapsız, ISA deniz seviyesi) yer koşusu ≤ 300 m pist uzunluğu (anormal durum: 1,5 alan katsayısı uygulanmaz; normal iniş R-07 ve en büyük iniş kütlesi işletme sınırıdır) | ≤ 300,0 m | 219,6 m | ✔ |
| R-56 | Boş kütle bütçesi (mass.budget, grup tavanları) ve yedek payı, R-02'yi (tasarım görevi faydalı yükü, 10 h) ve R-02b'yi (20 kg, 9,5 h) tam karşılayan iki boş kütlenin küçüğünü aşmaz (pay ≥ 0); her grubun tahmini kendi tavanının altında kalır | ≥ 0,000 kg | 0,255 kg | ✔ |
| R-57 | Kanatçık, flap ve dümen eyleyicileri: anma torku × dört çubuk bağlantısının o sapmadaki tork oranı ≥ 1,1 × menteşe momenti, bütün sapma aralığında (kanatçık ve dümen VA'da tam sapma ve VD'de 1/3 sapma, flap VF'de; R-47 kuralı) | ≥ 1,100 | 1,235 | ✔ |
| R-58 | Kanatçık, flap ve dümen eyleyicileri: tepe tork × dört çubuk bağlantısının o sapmadaki tork oranı ≥ 1,25 × menteşe momenti, bütün sapma aralığında (CS-LUAS.395(a)(1)) | ≥ 1,000 | 1,830 | ✔ |
| R-59 | Bütün kumanda yüzeylerinde (kanatçık, flap, dümen, stabilatör) dört çubuk bağlantısı istenen sapma aralığına eyleyicinin hareket sınırı içinde ulaşır (pay ≥ 0°) | ≥ 0,000 deg | 6,704 deg | ✔ |
| R-60 | Kalkış dönüşü (uçuş kontrol sistemi dönüş yasası): yunuslama hızı komutu sıfırdan komut hızına rampalanır ve tutulur; burun tekeri yerden kesikken stabilatörün ana teker temas noktasına göre (I_yy + m a²) × açısal ivme momenti için gereken aşağı kuvveti her adımda azami aşağı kuvveti aşmaz (pay kırpılmadan ölçülür, ≥ 0; bütün MTOM yükleme durumları); yerden kesilmede ana teker tepkisi sıfırdır ve aşağı kuvvet havadaki değere (sabit hızda güç-açık trim) eşittir; moment artığı ya da bu süreklilik 1e-6'yı aşarsa ölçüt negatif olur | ≥ 0,0 N | 0,6 N | ✔ |

**v1.6'da değişenler (beşinci tur).**

* **R-60:** pay artık kırpılmadan ölçülür. Gereken aşağı kuvvet azami değeri aşarsa pay negatiftir ve R-60 başarısız olur. Moment artığı ya da yerden kesilme sürekliliği 1e-6'yı aşarsa veya hız sınırlı bir adım varsa ölçüt yine negatiftir (V4-01). Dönüş yasası yunuslama ataletini içerir (V4-05). En küçük pay 0,59 N'dur. Pay V_R'de tanım gereği küçüktür, çünkü V_R azami aşağı kuvvetin dönüşü başlatmaya yettiği ilk hızdır (0,02 m/s'lik ızgara).
* **R-35:** V_R'de ana teker yükü, dönüşün başlangıcındaki düşey ivmeyi (ana tekerin önündeki ağırlık merkezi yükselir) de içerir: 112,6 N.
* **R-02, R-56:** tanımları değişmedi; değerler yeni kapanıştandır (görev yükü 18,0 kg; R-56'yı R-02b belirler).

**v1.5'te değişen ve eklenen gereksinimler.**

* **R-02, R-02b, R-03, R-56:** proje liderinin gereksinim kararı (yukarıdaki bölüm). R-02'nin hedefi 10 h olarak kaldı; ölçüldüğü yük değişti. R-03 artık azami yükü (20 kg) denetler; bunun için MTOM'da 20 kg yüklü yükleme durumları eklendi.
* **R-26:** yalnız bölge kutularını değil, bölgelerde bildirilen içerikleri gerçek zarflarıyla da denetler (V3-02).
* **R-60 (yeni):** kalkış dönüş yasası (V3-03).
* **R-50:** metni k_inst'i tahmin olarak anar (V3-06); sınır değeri değişmedi.

**R-44 notu.** Arka kiriş derinliği, gövde yanından bağlantıya kadar, dış panel bağlantı kesitinin kendi arka kiriş derinliğinin %95'inden az olamaz. NLF(1)-0416'nın %72 veterdeki derinliği kalınlığın ancak 0,44'üdür; kanat yapısı bu yüzden gerçek derinlikle hesaplanır.

## 5. Boyutlandırma yöntemi

Tasarım, sabit MTOM ile bir **kapanış döngüsüdür** (`sizing.design_closure`). Her yinelemede aşağıdaki adımlar yapılır.

1. **Geometri** girdilerden üretilir.
   * Gövde, kontrol çizgilerinden (Hermite ogive, yumuşak geçişler) kurulan yoğun `oml.Fuselage` istasyonlarından oluşur. Üst fasetlerde n ≈ 1,06, alt fasetlerde n ≈ 1,15'tir; kenar çizgisi, sırt ve omurga keskindir.
   * LERX, kenar çizgisine teğet başlayıp dış kanat hücum kenarına teğet biten kübik bir Bezier'dir. Kesiti NACA 0016-34'ten NLF(1)-0416'ya 0,25'lik adımlarla karışır.
   * Kuyruk gövdeleri dış yüzeyde budanır; yapıları 20 mm'lik bağlantı bandındadır.
   * Stabilatör milinin yeri VLM AM bandından ve motor zarfından türetilir. Dikey ve ventral konumları pervane koruma kuralından gelir.
2. **Aerodinamik.**
   * Taşıyıcı çizgi, LERX dahil gerçek kanatta NeuralFoil kesit polarlarıyla çalışır. Kanat CLmax'ı süpürme için cos Λc/4 ile çarpılır. Kaldırma eğimi, aerodinamik merkez ve Trefftz verimi VLM ile bulunur.
   * Profil sürüklemesi, geçişi x/c 0,075'e zorlanmış polarlar × 1,15 kuralıyla hesaplanır.
   * Sürüklemenin geri kalanı OML ağının açıkta kalan ıslak alanları üzerinden toplanır. Ayrı kalemler: arka kapanış taban sürüklemesi, taret halkası açıklığı, kanat-gövde ve kuyruk-gövde birleşimleri, sabit stabilatör kök parçaları, contalı takım kapakları.
   * Soğutma sürüklemesi her noktada o noktanın hızı, irtifası ve yakıt akışıyla hesaplanır.
   * **İtici kurulumu.** Kısmi gaz itkisi × k_inst; k_inst = 0,93 bir tahmindir (küt taban; §6.5 duyarlılık). Tam gaz itkisi × [1 − 0,07 min(V/15, 1)].
3. **Kütle.** Kalemler tek tek ve kaynaklıdır. Kanat kirişi başlıkları, kütle × irtifa rüzgâr hamlesi matrisinin en büyük taşıma kuvvetinden ve gerçek eldiven derinliğinden boyutlanır. Bütün kalemlere %5 büyüme payı eklenir. Yakıt = MTOM − boş kütle − görev yükü. Aviyonik, batarya ve PDU kalemleri paketlenmiş içeriklerin kütle merkezlerindedir (V3-02). Takım kapağı kalemi kapak düzeninden kurulur: kapak alanı, birleşim uzunluğu (contalar), iç kapak eyleyicileri ve bacak kapağı ayakları (V4-03). Grup bütçeleri tek yönlü tavanlardır (R-56).
4. **Kararlılık.** İki nötr nokta hesaplanır: klasik yöntemle ve bütün konfigürasyonun VLM'siyle; öndeki kullanılır. Kanat, en arka yüklemede statik marj %10,1 olacak şekilde kaydırılır. Yükleme durumları azami yükle (20 kg) olanları ve dolu depolu temel sensör setini (V4-02) de içerir. Her kalkış yüklemesinin yunuslama ataleti I_yy kütle kalemlerinden hesaplanır (V4-05).
5. **Kurallar.**
   * Kanat alanı tutunma hızı kuralından gelir: VS = 23,95 m/s, trimli CLmax, MTOM, en ön AM (R-08 24 m/s).
   * Stabilatör, yerel trim CL'si (η_t dahil) 0,70'i geçmeyecek boyuttadır (R-36 0,72). Temiz ve kalkış CLmax'ında, tam gazda yerden kesilmede (1,1 VS_TO, takım açık) ve pas geçmede aranır.
   * Dikeyler Cnβ ≥ 0,0585 için boyutlanır (R-11 0,057).
   * Kuyruk kuralları tek yönlüdür. Kapanış, ardışık iki yinelemede yakıt 5 g ve x_c4 0,1 mm içinde aynı kaldığında yakınsamış sayılır.
   * **Görev faydalı yükü kuralı (v1.5).** `evaluate`, kapanmış uçakta MTOM'da 0,5 kg'lık yük ızgarasında aynı görevi uçar. ≥ 10,25 h veren en büyük yükü bulur, en çok azami yük kadar. `--update-spec` kapanış → değerlendirme → yük güncellemesi döngüsünü yük tekrarlanana kadar sürdürür. v1.6'da ilk geçiş 19,0 kg'dan 18,0 kg'a indi, ikinci geçişte tekrarlandı. Kural ayrıca eşiğe payı raporlar (dayanım farkı ve ona denk boş kütle). `--check`, spec değerini türetilen değerle karşılaştırır.
6. **Görev ve performans.**
   * Görev parça parça uçulur: ısınma, kalkış, 3000 m'ye tam gazda tırmanma, 100 km geçiş (en iyi menzil hızı, ≥ 30 m/s), bekleme (en düşük yakıt akışı; 26 m/s EAS, 1,2 VS ve jeneratör devir tabanları), 100 km dönüş, alçalma, iniş, %10 yedek.
   * Yakıt doğrudan integrasyonla bulunur: dW/dt = −ṁ. Tırmanma 500 m'lik enerji yüksekliği adımlarıyla, geçiş 25 km'lik, bekleme ve yedek 30 dk'lık adımlarla uçulur.
   * **Alçalma:** 500 m'lik adımlar. Motor, jeneratörün sürekli elektrik yükünü taşıdığı devirdedir (rölantiye inemez). Pervane tablosu o devirde itki ve tork verir. 2,5 m/s alçalma hızı için uçuş hızı T cos ε − D = −W sin γ dengesinden çözülür. Bu noktalar %5–6 güçtedir; BSFC eğrisi %20'nin altında dışdeğerlenir (V3-07, §6.5 ve §16).
   * **Bekleme devri tabanı (R-52):** jeneratörün DC çıkışı E180 tepe yükünün en çok 17 W altında kalır (4.747 rpm). E180 büyüme görevi de aynı kuralla ve kendi sürekli elektrik yüküyle (373 W, V3-08) uçulur.
   * **Azami yük görevi (R-02b):** aynı görev, MTOM'da 20 kg yük ve 28,6 kg yakıtla; trimli polarlar ve 1,2 VS tabanları bu yüklemenin ağırlık merkeziyle hesaplanır.

![Kısıt diyagramı](fig/yk250_constraint.png)

Kısıt diyagramında tasarım noktası, VS = 23,95 m/s'ye karşılık gelen kanat yüklemesindedir (kırmızı dikey çizgi). Tırmanma eğrisi R-05'in 4,0 m/s değeriyle, kesikli eğri 4,9 m/s temel hedefiyle çizildi. Yatay siyah çizgi, sabit hatveli pervanenin tırmanma hızında tam gazda emebildiği mil gücüdür. Motorun 18 kW çizgisi bu pervaneyle kullanılamaz.

## 6. Sonuçlar

![Genel yerleşim (3 görünüş)](fig/yk250_3view.png)

### 6.1 Geometri

| Kalem | Değer |
|---|---|
| Kanat | b 7,20 m · S 3,241 m² · AR 16,0 · λ 0,35 · c/4 süpürme 8° · dihedral 3° · i 3,5° · burulma 4° |
| Kanat kökü konumu | x_c4 kökü 2,513 m; OAK 0,485 m (ön kenar x = 2,604 m) |
| LERX ve eldiven | Tepe x = 1,80 m (kenar çizgisinde); eldiven/dış panel birleşimi y = 0,70 m. Kök kesiti NACA 0016-34; u = 0,17–0,80 aralığında NLF(1)-0416'ya karışır. Dış panel 2,90 m |
| Gövde | 4,00 m × 0,80 m × 0,46 m |
| Stabilatörler (çift, hareketli) | 0,624 m², NACA 0014 |
| Sabit kök parçaları (çift) | 0,175 m², NACA 0014 |
| Dikey kuyruklar (çift) | 0,794 m² panel, 22° dışa eğik, %30 dümen |
| Ventral kanatçık | 0,057 m², NACA 0010 |
| Pervane | Mejzlik 31x12 3B, Ø 0,787 m; düzlem kaporta firar kenarının 80 mm (0,102 D) gerisinde, göbek z = 0,22 m, 5° aşağı itki |

### 6.2 Kütle

| Grup | Kütle (kg) | Bütçe tavanı (kg) |
|---|---|---|
| Kanat (kirişler, kaburgalar, kaplamalar, panel bağlantıları; kumanda yüzeyleri hariç) | 16,40 | 16,40 |
| Sistemler (paraşüt, Li-ion tampon batarya, PDU, aviyonik, kablo demeti, taret kaldırma mekanizması) | 14,66 | 14,66 |
| İniş takımı (SAGITTA referanslı bacaklar, altı contalı kapak, iç kapak eyleyicileri, kilitler) | 14,34 | 14,34 |
| İtki (kurulu motor grubu, pervane, soğutma) | 14,30 | 14,31 |
| Şasi (omurga kirişleri, çerçeveler, kanat geçiş kutusu, stabilatör mil yatakları, bağlantılar) | 11,14 | 11,15 |
| Gövde kabuğu (sandviç kaplama, kenar çizgisi şeritleri) | 10,47 | 10,47 |
| Kumanda (eyleyiciler, dört çubuk bağlantılar, kanatçık/flap/dümen yüzeyleri) | 8,30 | 8,31 |
| Kuyruk (stabilatörler, sabit kök parçaları, dikeyler, ventral) | 7,88 | 7,89 |
| Yakıt sistemi (3 hücre) | 2,34 | 2,35 |
| Bağlantı elemanları | 1,51 | 1,51 |
| **Boş kütle (%5 büyüme payı dahil)** | **101,3 kg** (101,336) | 101,39 |

Faydalı yük: görev yükü 18,0 kg = temel sensör seti 3,25 kg (HD59 taret + bağlantı 1,75, görev bilgisayarı/kayıtçı 1,0, tepsi/kablo 0,5) + araştırma yükü payı 14,75 kg. Azami yük 20,0 kg için aynı karın bölmesine 2,0 kg daha konur; MTOM sabit olduğundan yakıt 2,0 kg azalır (28,6 kg).

Boş kütle v1.5'e göre 0,62 kg arttı (V4-03). `gear_doors_wells_locks_sensors` kalemi 1,68 kg'dan 2,27 kg'a çıktı: altı kapak 0,192 m² × 2,733 kg/m² = 0,525 kg; kuyu kapatmaları 0,36; kesik takviyesi 0,50; kilit/algılayıcılar 0,25; contalar 0,101 (4,28 m birleşim); iki iç kapak eyleyicisi (Volz DA 22, 2 × 0,132 kg); bağlantıları 0,08; dört bacak kapağı ayağı 0,08 kg; toplam 2,160 kg + %5 büyüme payı. Bağlantı elemanları (yapı + takımın %2,5'i) 0,015 kg, kuyruk (stabilatör kapanışta +%0,4) 0,016 kg arttı. Hiçbir kalem silinmedi. Kalemin ağırlık merkezi kapak parçalarının kütle ağırlıklı ortalamasıdır (x = 2,50 m).

### 6.3 Aerodinamik

![Trimli polarlar](fig/yk250_polars.png)

Polar eğrileri her yapılandırmanın trimli CLmax'ında biter. L/D ve dayanım parametresinin en büyük değerleri de bu sınırın içinde aranır. Takım açık poları flapsızdır; kalkış ve iniş flaplarının artımları alan performansı modellerinde ayrıca eklenir. Gölgeli bant tasarım görevinin bekleme CL aralığıdır (başlangıç → bitiş, taret dışarıda).

| Kalem | Değer |
|---|---|
| CLmax | kanat 1,333 (× cos 8°); trimli temiz 1,292 (en ön AM); 35° kalkış flabıyla 1,531 |
| CD0 | temiz 0,0352 · taret dışarıda 0,0371 · takım açık 0,0476 |
| (L/D)maks | 17,3 temiz, 16,8 taret dışarıda |
| Görev beklemesi başlangıcı (W = 1.416 N, 3000 m) | 32,43 m/s TAS (27,94 m/s EAS), CL 0,92, toplam güç 4,48 kW, 4.854 rpm, 2,56 kg/h, jeneratör 487 W |
| Görev beklemesi sonu (W = 1.222 N, 3000 m) | 32,33 m/s TAS (27,85 m/s EAS), CL 0,80, toplam güç 4,09 kW, 4.747 rpm, 2,39 kg/h, jeneratör 476 W |
| MTOM'da 3000 m bekleme noktası (görevde uçulmaz, karşılaştırma için) | 33,03 m/s TAS (28,46 m/s EAS), CL 0,92, L/D 16,1, mil gücü 4,27 kW, 4.946 rpm, η 0,711, BSFC 563 g/kWh, 2,65 kg/h, jeneratör 496 W |

Temiz sürükleme kalemleri (CD, S_ref'e göre):

| Kalem | CD |
|---|---|
| Gövde | 0,0076 |
| Stabilatörler | 0,0024 |
| Dikey kuyruklar | 0,0022 |
| Arka kapanış taban sürüklemesi | 0,0022 |
| Motor soğutma | 0,0019 |
| Arka gövde yukarı kıvrımı | 0,0012 |
| Sızıntı/çıkıntılar | 0,0010 |
| Kumanda yüzeyi boşlukları | 0,0008 |
| Anten/pito/ışıklar | 0,0008 |
| Kuyruk-gövde birleşimleri | 0,0006 |
| Stabilatör kök parçaları | 0,0004 |
| Contalı takım kapakları (4,28 m birleşim) | 0,0004 |
| Kanat-gövde birleşimleri | 0,0002 |
| Ventral | 0,0002 |
| Taret kapak boşlukları | 0,0001 |

Kanat profil sürüklemesi CL 0,7'de 0,0134 değerindedir (tripli × 1,15). Bekleme polarına taret dışarıdayken +0,0018 eklenir.

**Arka kapanış (F12, V2-02).** Motor silindirlerinin bittiği yerden pervane düzleminin 80 mm önündeki kaporta ağzına kadar yalnız 0,1 m vardır. Kaporta bu yüzden en dik yerinde 59° eğimle kapanır. Eğimin 15°'yi geçtiği yerden (x = 3,91 m) sonrası ayrılmış taban sayılır: 0,112 m², CD 0,0022 (Hoerner). Bu bir **ödünleşimdir** ve görüntülerde küt bir kaporta ucu olarak görünür. Gerçek bir sivrilen kuyruk konisi denendi; sonuç §13'tedir. Küt taban yüzünden itici kurulum katsayısı **tahmin olarak** 0,93 alınır. aero.yaml'daki 0,93–0,97 aralığı gövde arkasındaki itici için mühendislik yargısıdır, bir testten gelmez. Küt taban durumunu da sınırlamaz: 0,93 bir alt sınır değil, aralığın alt değerinden alınmış bir tahmindir (V3-06). Duyarlılık §6.5'tedir. Pervanenin iç bölgesinin ayrılmış izde çalışmasından gelen pala yük değişimi açık bir konudur (§16).

### 6.4 Kararlılık ve ağırlık merkezi

![Ağırlık merkezi zarfı](fig/yk250_cg_envelope.png)

* Nötr nokta x = 2,679 m'dir (VLM; klasik yöntemle 2,690 m). Köşeli ön gövde ve LERX kanadın önünde kaldırma üretir; ağırlık merkezi bu yüzden referans OAK'ın ön kenarına yakındır.
* Statik marj %10,1–%14,6 OAK'tır. En arka durum x = 2,630 m (MTOM, azami yük), en öndeki durum (E180 büyüme taretli MTOM) x = 2,608 m'dir. Azami yük ve görev yükü durumları neredeyse çakışır, çünkü yük bölmesi boş kütle ağırlık merkezine ortalanmıştır; şekilde iç içe halkalar olarak çizilirler (ilki en büyük ve en altta; V4-06). Dolu depolu temel sensör seti (140,5 kg, 35,95 kg yakıt) %10,6 statik marjla yükleme durumları arasındadır (V4-02).
* V_H 0,439 (kol 1,104 m), V_V 0,0274 (kol 1,096 m). Aşağı sapma dε/dα 0,39.
* Cnβ = 0,0585. Katkılar: dikeyler +0,0942, ventral +0,0029, gövde −0,0386 (DATCOM). Clβ = −0,113.

### 6.5 Performans

| Durum | Değer |
|---|---|
| Dayanım / bekleme / menzil (görev yüküyle) | 10,41 h / 8,10 h / 1.157 km |
| Azami yükle (20 kg, R-02b) | 9,64 h (bekleme 7,33 h, yakıt 28,6 kg) |
| E180 büyüme görevi (MTOM, tam yakıt, E180 tareti, E180 elektrik yükü) | 10,15 h (bekleme 7,85 h; aynı devir tabanı ve batarya destek kuralı) |
| 10 h görev için gereken MTOM (aynı uçak ölçeklenerek, görev yüküyle) | 147,3 kg |
| Tutunma (DS, MTOM) | 23,94 m/s |
| Tırmanma | 4,21 m/s (DS), 2,38 m/s (3000 m); 3000 m'ye 15,3 dk ve 1,57 kg yakıt |
| Alçalma | 20,0 dk, 0,237 kg; 3.521 rpm, 33,5–34,9 m/s TAS |
| Azami düz uçuş hızı | 54,0 m/s (DS). DS'de VNE'yi aşar; uçuş kontrol sistemi hızı 45 m/s EAS'de sınırlar (§6.6) |
| Tavan | servis 7.100 m, mutlak 8.343 m |
| Kalkış (DS, MTOM) | koşu 179 m, 15 m'ye 325 m (§7) |
| Kalkış (1500 m ISA) | koşu 276 m (gereksinim değil, bilgi) |
| İniş (DS, görev sonu, 123,0 kg) | koşu 183 m, teker koyma 24,9 m/s (flapsız) |
| İniş (DS, MTOM, acil/geri dönüş) | flapsız 219,6 m (teker koyma 27,5 m/s), R-55 ✔; 35° flapla 249,1 m. 1,5 katsayılı 200 m kuralı için en büyük iniş kütlesi 135,6 kg |
| Faydalı yük – dayanım | 3,25 kg 12,93 h · 10 kg 12,63 h · 12,61 kg 12,50 h · 15 kg 11,57 h · 18,0 kg 10,41 h · 20 kg 9,64 h (yüksüz yükleme izin verilmez; Gereksinim kararı bölümü) |

Duyarlılıklar (görev yüküyle dayanım, h; her satırda yalnız o girdi değişir, uçak aynıdır):

| Değişiklik | Dayanım |
|---|---|
| BSFC −%12 | 11,86 h |
| BSFC +%12 | 9,27 h |
| Bütün sürükleme +%10 | 9,86 h |
| Boş kütle +%5 | 8,46 h |
| Bekleme 1000 m'de | 9,83 h |
| Taret bütün görevde içeride | 10,54 h |
| Geçişsiz, yalnız bekleme | 10,36 h |
| **k_inst 0,90** (tahmin 0,93; küt taban daha kötü olursa) | 10,22 h |
| k_inst 0,95 | 10,54 h |
| k_inst 0,97 | 10,65 h |
| **Düşük yükte Willans yakıt doğrusu** (V3-07) | 10,35 h |

Dayanımın sürüklemeye eğimi, her uçuş evresinde +0,001 CD için −0,105 h'tir. k_inst 0,90 ve Willans doğrusu R-02'yi bozmaz: 10 h'in üstünde kalırlar. k_inst 0,90 dayanımı görev yükü kuralının eşiğinin (10,25 h) altına indirir (10,22 h), Willans doğrusu indirmez (10,35 h). Kurulum testi k_inst'i 0,93'ün altında bulursa kural daha küçük bir görev yükü verir (§16).

**Düşük yük BSFC'si (V3-07).** BSFC eğrisinin en düşük noktası %20 güçtedir ve bu nokta da araştırmada dışdeğerlenmiş bir tahmindir. Alçalma %5,1–6,4, yedek bekleme %17,8–17,9 güçte uçulur. Model BSFC doğrusunu bu güçlere dışdeğerler. Karşılaştırma için Willans doğrusu kullanıldı: yakıt akışı güçle doğrusal, en düşük iki noktadan geçer ve sıfır güçte pozitif bir yakıt akışı kalır. Bu, daha ihtiyatlı bir dışdeğerlemedir, bir sınır değildir: %20 gücün altında onu sınırlayan veri yoktur (V4-08). Willans doğrusuyla:

* alçalma yakıtı 0,237 kg'dan 0,362 kg'a çıkar;
* yedek bekleme yakıtı 2,053 kg'dan 2,070 kg'a çıkar;
* dayanım 10,41 h'ten 10,35 h'e iner.

Limbach'tan düşük yük haritası alınana kadar bu bir sınırlamadır (§16).

### 6.6 Yapısal yükler ve işletme sınırları

![V-n diyagramı](fig/yk250_vn.png)

VA = VC = 45 m/s EAS, VD = 57 m/s EAS. Rüzgâr hamlesi bütün kütle × irtifa matrisinde, yapılandırma kaldırma eğimiyle hesaplanır (F9):

| Kütle | İrtifa | n+ / n− |
|---|---|---|
| 106,1 kg | 0 m | +6,52 / −4,52 |
| 106,1 kg | 3.000 m | +6,86 / −4,86 |
| 106,1 kg | 4.500 m | +7,02 / −5,02 |
| 149,9 kg | 0 m | +5,19 / −3,19 |
| 149,9 kg | 3.000 m | +5,38 / −3,38 |
| 149,9 kg | 4.500 m | +5,47 / −3,47 |

Kanat, en büyük taşıma kuvvetine göre boyutlanır (n = 5,47). Nihai kök momenti 9.167 N·m, panel bağlantısında 5.496 N·m'dir; kök kiriş başlığı 306 mm²'dir.

**İşletme sınırları (R-54).** VNE = 0,9 VD = 51,3 m/s EAS ve VNO = 45,0 m/s EAS'dir. Tam güçte düz uçuş hızı DS'de 54,0 m/s EAS, 3000 m'de 44,1 m/s EAS'dir. Uçuş kontrol sisteminin zarf koruması bu yüzden zorunludur: hızı gaz ve yunuslama komutuyla 45 m/s EAS'de sınırlar.

## 7. Kalkış ve iniş (F1, F5, V2-04, V2-05, V3-03, V4-01, V4-05, V1-10)

Kalkış modeli dört adımdan oluşur (v1.5, yunuslama ataleti v1.6). Bütün momentler, ana teker yer temas noktası C'ye göre, uçağın o anki tutumunda ve yer çerçevesinde alınır. Burun yukarı dönüşte C'nin üstündeki noktalar geriye kayar. Kuvvetler şunlardır:

* ağırlık (AM'de);
* kanat-gövde kaldırması (AM'sinde, düşey);
* stabilatör aşağı kuvveti (stabilatör AM'sinde);
* itki (göbekte, gövde eksenine göre 5° aşağı; yer çerçevesinde ileri T cos(θ − ε), yukarı T sin(θ − ε));
* kalkış yapılandırmasının Cm0'ı;
* sürükleme ve boyuna atalet (AM yüksekliğinde);
* tekerlek tepkileri ve yuvarlanma sürtünmesi (yerde; C'ye göre momentsiz).

1. **Yer koşusu.** 35° flap ve 2,5° zemin tutumunda zaman adımıyla integre edilir. İtki tam gaz T(V)'dir (kurulum rampasıyla). Sürükleme zemin etkili indüklenmiş sürükleme, takım sürüklemesi ve tam güç soğutmasından oluşur. Sürtünme μN'dir.
2. **Yer koşusunda stabilatör programı (uçuş kontrol sistemi gereksinimi).** 23,5 m/s'den itibaren stabilatör, ana tekerleri yüklü tutan en küçük aşağı kuvveti verir; böylece uçak burun tekeri üzerinde "el arabası" gibi kalmaz. Aşağı kuvvet V_R'de en büyük değerine ulaşır (yerel CL 0,9, tam burun yukarı).
3. **Dönüş yasası (V3-03; yunuslama ataleti V4-05).** v1.4'teki "dönüş sırasında stabilatör güç-açık trime döner" kuralı fiziksel değildi; v1.5 yasası açısal ivmeyi yok sayıyordu. v1.6 yasası şöyledir:
   * Komut verilen yunuslama hızı, 1,1 VS_TO trim tutumuna (R-16'nın tutumu) 1 s'de varacak biçimde seçilir: 1,05°/s. Uçuş kontrol sistemi bu hızı sıfırdan **0,3 s'de** rampalar (sabit açısal ivme 3,50°/s²), sonra tutar (`mission.rotation_spin_up_s`, tasarım tercihi).
   * Burun tekeri yerden kesikken C'ye göre moment (I_yy + m a²) θ̈ olmalıdır; a, AM'nin C'ye yatay uzaklığıdır. m a² terimi, C'nin önündeki AM'nin dönüşle yükselmesinden gelir; bu düşey ivme ana teker tepkisine de girer: N = W − T sin(θ − ε) + F_t − L − m θ̈ a. Stabilatör aşağı kuvveti bu momenti veren değerdir.
   * **I_yy** kütle kalemlerinden tahmin edilir (`distributed_pitch_inertia`): kalemler kendi yerlerinde noktasal kütledir; gövde kaplaması, kanat ve kuyruk yapıları açıkta kalan ıslak alanlarına yayılır; omurga kirişleri, çerçeveler, kenar şeritleri ve kablo demeti gövde boyunca çubuktur; yakıt, hücre dikdörtgenlerine yayılır. MTOM kalkış yüklemelerinde I_yy 116,7–121,3 kg·m²'dir (noktasal toplam yalnız ~92 kg·m² verir).
   * V_R, azami aşağı kuvvetin (η q S_h CLt_maks) zemin tutumunda dönüş başlangıcının momentini verdiği ilk hızdır (burun tepkisi sıfır).
   * Kaldırma arttıkça gereken aşağı kuvvet azalır. N sıfıra indiği anda uçak yerden kesilir. Bu anda aşağı kuvvet, aynı açısal ivme ve aynı düşey ivmeyle havadaki değere eşittir; hız tutulduktan sonra bu güç-açık trim değeridir.
   * Gereken aşağı kuvvet her adımda azami değerle karşılaştırılır ve pay **kırpılmadan** kaydedilir (V4-01). Aşsaydı azami değer uygulanır, uçak onun verdiği açısal ivmeyle döner, adım sayılır ve pay negatif olurdu (R-60 başarısız). v1.6'da hiç böyle adım yoktur.
   * 1,1 VS_TO altında ana tekerler yüklü kalacak biçimde tutum sınırlanır; bu hızın altında yerden kesilme yoktur.
4. **Kontroller.** Bütün MTOM yüklemelerinde: V_R'de ana tekerler yüklüdür (R-35); dönüşün her adımında gereken aşağı kuvvet azami değerin altındadır (R-60, kırpılmamış pay). Moment dengesi artığı her adımda ≈1e-13 N·m'dir. Yerden kesilmede ana teker tepkisi 0'dır ve aşağı kuvvet havadaki değere eşittir (fark ≈1e-13 N, < 1e-12). R-60 ölçütü bu iki büyüklükten biri 1e-6'yı aşarsa ya da hız sınırlı bir adım varsa negatif olur. Yerden kesilme tutumu R-16 tutumunu (3,75°) aşmaz. Bir test, dönüşte azami aşağı kuvveti 0,9 ile çarpar; o durumda bütün yüklemelerde adımlar hız sınırlı olur, pay −17,6 N'a iner ve R-60 başarısız olur.

| Yükleme (takım açık) | I_yy | Koşu | V_R | V_LOF | θ_LOF | Ana tekerde yük (V_R) | Aşağı kuvvet V_R → yerden kesilme | Yerel stabilatör CL (V_LOF) | En küçük pay |
|---|---|---|---|---|---|---|---|---|---|
| MTOM, görev yükü, taret içeride | 116,8 kg·m² | 171 m | 24,1 m/s | 24,7 m/s | 2,8° | 189 N | 179 → 115 N | 0,55 | 0,59 N |
| MTOM, görev yükü, taret dışarıda | 116,9 kg·m² | 171 m | 24,1 m/s | 24,8 m/s | 2,8° | 186 N | 180 → 115 N | 0,55 | 1,63 N |
| MTOM, azami yük, taret içeride | 116,7 kg·m² | 171 m | 24,1 m/s | 24,8 m/s | 2,8° | 186 N | 180 → 116 N | 0,55 | 1,25 N |
| MTOM, azami yük, taret dışarıda | 116,8 kg·m² | 171 m | 24,1 m/s | 24,8 m/s | 2,8° | 184 N | 180 → 116 N | 0,55 | 2,29 N |
| MTOM, E180 büyüme tareti (en ön AM) | 121,3 kg·m² | 179 m | 24,8 m/s | 25,2 m/s | 2,6° | 113 N | 190 → 148 N | 0,68 | 0,79 N |

Belirleyici durumda V_LOF 25,2 m/s, yerden kesilme tutumu 2,6°'dir. Bu tutum zemin tutumundan (2,5°) büyük, 1,1 VS_TO tutumundan (3,55°) küçüktür; dönüş 0,30 s sürer ve rampa yerden kesilmeden önce tamamlanır. Stabilatör programı olmasaydı uçak zemin tutumunda 24,0 m/s'de yükselirdi; burun tekeri o hızda yüklü kaldığı için bu durum güvensiz olurdu. Program bunu önler. Statik itki 492 N'dur. 1500 m ISA pistte koşu 276 m'dir.

**Yunuslama ataletinin etkisi (V4-05).** Belirleyici durumda ataletsiz (v1.5) yasayla V_R 24,58 m/s ve koşu 175,6 m olurdu. Ataletle V_R 24,76 m/s, koşu 178,7 m'dir (+3,0 m). Rampa 0,1 s olsaydı koşu 180,7 m olurdu. Etki, doğrulamanın kaba tahmininden (+7…20 m) küçüktür: V_R civarında C'ye göre moment yalnız aşağı kuvvet kapasitesiyle değil, C'nin önündeki kanat kaldırmasıyla da büyür (yaklaşık 48 N·m/(m/s)); 8,7 N·m'lik dönüş başlangıcı momenti V_R'yi bu yüzden yalnız 0,18 m/s ötelemektedir. R-06 payı 21 m'dir.

**İniş (R-07, R-55).** Normal iniş görev sonu kütlesinde (123,3 kg) flapsız yapılır: koşu 183 m'dir. MTOM'da kalkıştan hemen sonra acil ya da geri dönüş inişinde koşu flapsız 219,7 m'dir; bu, 300 m'lik pist uzunluğunun altındadır. 35° flapla koşu 249,2 m'ye uzar; flabın yerdeki kaldırması fren normal kuvvetini azaltır. 1,5 katsayılı 200 m kuralı 135,6 kg'a kadar karşılanır; bu bir işletme sınırıdır.

## 8. Kanat kökü: LERX, kenar çizgisi ve kiriş derinliği (F7, F8, F11, V1-06)

* **Plan formu.** Sivrilme 0,35, eldiven/dış panel birleşimi y = 0,70 m, dış panel 2,90 m, c/4 süpürme 8°, burulma 4°, LERX tepesi x = 1,80 m, tutunma hızı kural hedefi 23,95 m/s (R-08 ≤ 24 m/s).
* **LERX–eldiven kesit geçişi (V1-06).** LERX kesiti NACA 0016-34'tür. LERX açıklığının u = 0,17–0,80 aralığında kesit 0,25'lik adımlarla NLF(1)-0416'ya karışır; komşu kesitler arasında aile değişmez.
* **Kenar çizgisi geçişi.** Kenar çizgisi kanat kökü firar kenarının 30 mm gerisine kadar kanat düzleminde tutulur. Kanat üst yüzeyi ile kenar çizgisi arasındaki basamak 0,0 mm'dir (R-45).
* **Kiriş derinliği.** Gövde yanından bağlantıya kadar en az derinlik ana kirişte 90,8 mm (gerekli 92,9 mm), arka kirişte 41,1 mm'dir (gerekli 41,0 mm).
* **Flap.** Her sökülebilir dış panelde bir flap vardır (η 0,203–0,58), iç ucu panel bağlantısının 30 mm dışındadır. Kanatçıklar η 0,60–0,95 aralığındadır.

## 9. Kuyruk ve kumanda yüzeyleri (F2, F3, F4, F5, F10, V1-04, V1-05, V2-03, V2-06)

![Arkadan görünüş](fig/yk250_rear.png)

* **Stabilatör boyutu.** Yerel trim CL'si en ön AM'lerde 0,70'i aşmaz; belirleyen durum takım açık ve tam gazla yerden kesilmedir (R-36 0,695). Sonuç 0,624 m², NACA 0014.
* **Aerodinamik merkez ve mil (F2).** Panel AM'si girdap kafesiyle bulunur; mil bandın ön ucundadır. Yüzey mil etrafında hiçbir durumda ıraksamaz.
* **Menteşe momentleri (V1-05).** VA'da tam sapmada panelin kesit CN_maks değeriyle (1,390) 17,86 N·m, VD'de 9,55 N·m, sürekli trimde 5,14 N·m.
* **Stabilatör bağlantısı (V2-06).** Bağlantı tam kinematiğiyle modellenir. Simetrik 2,5:1 dört çubuk kullanılır: servo kolu 15 mm, yüzey kolu 37,5 mm, itme çubuğu 100 mm. −20/+15° için servo −58°/+40° döner (Volz DA 30 ±85°). Tork oranı nötrde 2,50'dir ve uçlara doğru 4,18'e çıkar. CN_maks menteşe momenti her sapmaya uygulanır (ihtiyatlı). Paylar: tepe 1,79 (R-37, 1,25 katsayısıyla), anma 1,12 (R-47 ≥ 1,1), sürekli trim 3,89. Anma hızında yüzey hızı nötrde 60°/s, −20°'de 36°/s'dir. Bu hızın yeterliliği uçuş kontrol fazında doğrulanmalıdır.
* **Kanatçık, flap ve dümen (V2-03).** Menteşe momenti şeritlerle integre edilir: H = η q |Ch| ∫c_f² dy. Katsayı |Ch| = 0,6 |δ| + 0,09'dur (components.yaml tarama katsayıları: Ch_δ ≈ −0,6/rad artı Ch_α terimi; 20°'de 0,30). Flaplarda 30–40° arasında taramanın üst değeri 0,55 kullanılır. Durumlar: kanatçık ve dümen VA'da tam sapma ve VD'de 1/3 sapma; flap VF = maks(1,4 VS, 1,8 VSF) = 39,6 m/s'de bütün aralık (CS-LUAS.345(b)). Dümen hesabı bütün dikey panel açıklığını kullanır (ihtiyatlı). Her yüzey bir dört çubuk bağlantıyla sürülür; anma ve tepe payı her sapmada hesaplanır:

| Yüzey | Eyleyici | Bağlantı (nötr) | Durum | Menteşe momenti (N·m) | Servo açısı (aralık uçlarında; sınır) | Tork oranı | Anma payı (≥ 1,1) | Tepe payı (≥ 1,0) | Yüzey hızı (°/s) |
|---|---|---|---|---|---|---|---|---|---|
| Kanatçık (çift) | Volz DA 26 | 2,0:1 | 45,0 m/s | 3,26 | −43° / +43° (±50°) | 2,00–2,62 | 2,10 | 3,12 | 76–100 |
| Flap (çift) | Volz DA 30 | 2,0:1 | 39,6 m/s | 11,17 | −43° / +43° (±85°) | 2,00–2,62 | 1,51 | 2,41 | 57–75 |
| Dümen (çift) | Volz DA 26 (her dikeyde bir; uzatılmış hareket seçeneği) | 2,0:1 | 45,0 m/s | 6,52 | −57° / +58° (±85°) | 2,00–3,64 | 1,24 | 1,83 | 55–100 |
| Stabilatör (çift) | Volz DA 30 | 2,5:1 | 45,0 m/s | 17,86 | −58° / +40° (±85°) | 2,50–4,18 | 1,12 | 1,79 | 36–60 |

* **Mil düzeni.** İki kısa mil vardır. İç yatak motor bölmesinin halka çerçevesinde, krank/SG750 zarfının yanındadır; dış yatak sabit kök parçasının kaburgasındadır.
* **Stabilatör kökü (F3).** Hareketli kök sabit kök parçasının düz uç yüzüne karşı döner: boşluk 8,0 mm, gövdeye en yakın nokta 23,2 mm.
* **Dikey kuyruk ve ventral kökleri (F4, V1-04).** Kökler en az 21,2 mm (dikey) ve 15,0 mm (ventral) gömülüdür. Dış yüzeyin altındaki yapı 20 mm'lik banttadır (R-53).
* **Pervane koruması (F10) — dürüst durum.** Disk düzlemi itki hattıyla birlikte 5° eğiktir ve koruma bu düzlemde ölçülür:

| Yapı | Disk düzlemini kestiği açı (tepeden) | Uç çemberinin dışında |
|---|---|---|
| Stabilatör | 90° | 447 mm |
| Stabilatör | −90° | 447 mm |
| Eğik dikey | 36° | 75 mm |
| Eğik dikey | −36° | 75 mm |
| Ventral kanatçık (+ tampon kızağı altta) | 180° | 52 mm |

Dikeylerin firar kenarı diski üst-yan bölgelerde yakından çevreler (R-41/R-42). Ventral kanatçık ve altındaki tampon kızağı alt bölgeyi korur (R-18, R-48/R-49). **Göbek yüksekliğindeki yan bölgeler açıktır.** Uç çemberinin 0,3 m içinde hiçbir yapının bulunmadığı açısal pay %83'tür. Uçak kanat ucu yere değene kadar yatarsa (10,2°) pervane ucu yerden 0,319 m yukarıda kalır (R-51). Bu düzen **personel koruması sağlamaz**; yer operasyonunda pervane disk bölgesi yasak bölge olarak işaretlenmelidir.

## 10. İçeri katlanır iniş takımı (F6, V3-10, V4-03, V4-07)

![Alttan görünüş (takım ve taret açık): açık burun yuvası ve istiridye kapakları, ana takım bacak yarıkları ve bacak kapakları, kapanmış iç kapaklar, taret açıklık halkası](fig/yk250_belly.png)

**Kapak düzeni (`landing_gear.doors`, V3-10).** Takım açıkken görüntü şunları gösterir:

* **Burun yuvası:** omurga yarığı, iki yanındaki kenarlardan menteşeli iki istiridye kapakla kapanır (her biri 39 mm). Takım açıkken kapaklar 90° açıktır.
* **Ana kuyular:** her kuyuda iki kapak vardır.
  * İç kapak, kuyunun iç kenarından menteşelidir ve sıralı çalışır: yalnız takım hareket ederken açılır, yukarı ya da aşağı kilitten sonra yeniden kapanır. Açık kalsaydı gövdenin yaklaşık 0,17 m altına sarkardı; statik tutumda yerden 58 mm yukarıda kalırdı, sönük lastikte daha da az.
  * Bacak kapağı iki ayaklı bağlantıyla bacağa bağlıdır ve onunla döner. Görüntüde bacak kapağı, bacak gibi mafsal ekseni çevresinde geri çekme açısı kadar döndürülerek çizilir. Takım açıkken bacak yarığı açık kalır.
* Takım açık sürüklemesi açık kapak payını (`aero.drag_rules.door_Dq_open`) korur; iç kapaklar kapandığı için bu ihtiyatlıdır.

**Kapakların kütlesi ve sürüklemesi (V4-03).** v1.5'te kapak düzeni çizilmiş, ama kütle kalemi kimlik çalışmasının beş kapaklı toplamı (1,60 kg) olarak kalmıştı; sıralı iç kapakların eyleyicisi yoktu. Sıralı bir kapak, takım strokunun ortasında açılıp kapandığı için bacağa sabit bir bağlantıyla asılamaz. v1.6'da kalem kapak düzeninden kurulur (`sizing.gear_doors_mass`):

| Parça | Kütle (kg, büyüme payı hariç) | Dayanak |
|---|---|---|
| Altı kapak, 0,192 m² (iç 0,077 + bacak 0,079 + burun 0,036) | 0,525 | 2,733 kg/m² = kimlik çalışması 0,41 kg / 0,15 m² (sandviç, menteşe ve bağlantılar dahil) |
| Kuyu kapatmaları | 0,36 | kimlik çalışması |
| Kesik takviyesi | 0,50 | kimlik çalışması |
| Takım kilitleri ve algılayıcılar | 0,25 | kimlik çalışması |
| Çevre contaları, 4,28 m birleşim | 0,101 | 23,5 g/m (v1.2 tahmini 0,08 kg / 3,40 m) |
| İki iç kapak eyleyicisi | 0,264 | Volz DA 22-30-4128 (28 V), 0,132 kg veri sayfası, 1,8 N·m anma |
| İç kapak bağlantıları (krank, itme çubuğu, ölü nokta kilidi, kapalı anahtarı) | 0,08 | tahmin, 2 × 0,04 |
| Bacak kapağı ayakları | 0,08 | tahmin, 4 × 0,02 |
| **Toplam** | **2,160** (büyüme payıyla 2,268; v1.5 1,68) | |

Kapak menteşe çizgisi x yönündedir, kapak açıkken akış düzlemindedir; aerodinamik menteşe momenti küçüktür ve sayısal olarak denetlenmedi (§16). Birleşim uzunluğu artık kapaklar arası ayırma çizgilerini de içerir (iç/bacak kapağı ayrımı ve istiridye kapakların orta çizgisi): 3,39 m'den 4,28 m'ye; contalı kapak sürüklemesi buna göre arttı.

![Takım ve taret içeride (alttan görünüş; contalı kapakların ve taret açıklık halkasının birleşim çizgileri)](fig/yk250_turret_gear_retracted_belly.png)

İçeri katlanmış durumda burun takımının iki istiridye kapağı, ana takımın iç kapakları (y ±0,19 m, ortadan ayrık) ve iki bacak kapağı, taret açıklık halkası ve bölme kapakları birleşim çizgileri olarak görünür (V4-07: v1.5 görüntüsü her kuyu çifti için tek bir dikdörtgen çiziyordu). Gövde dış yüzeyi süreklidir; çizgiler yalnız görüntü içindir.

* **Pervane yer açıklığı (CS-VLA 925(a)).** Statik tutumda 0,208 m, yerden kesilme tutumunda (1,1 VS_TO trim tutumu, ihtiyatlı) 0,184 m, teker koyma tutumunda 0,204 m. Sönük lastik ve dibe oturmuş amortisörde 0,060 m kalır. Yer çizgisi z = −0,429 m'dir.
* **CS-VLA 925(c).** Pala uçlarıyla yapı arasındaki radyal açıklık en az 32,9 mm, boyuna açıklık en az 29,4 mm'dir.
* **Zemin tutumu.** 2,5° burun yukarı.
* **Konum.** Ana takım x = 3,001 m; iz 0,843 m, dingil açıklığı 2,321 m; geri devrilme 38,4°, yana devrilme 54,0°; burun yükü %16,1–%17,1.
* **Kapaklar.** Kapalıyken kaplamayla aynı yüzeydedir ve çevreleri contalıdır.

## 11. Geri çekilir EO/IR taret (F13, V1-07, V3-10)

* **Taret:** Trillium HD59-LLVV (1,55 kg + 0,20 kg bağlantı). Bölme, Octopus E180 büyüme zarfına göre boyutlanır.
* **İçeride:** HD59 topu kapak iç yüzünün 26,5 mm üstündedir.
* **Dışarıda:** strok 0,12 m. Kapaklar bölme duvarları boyunca içeri katlanır; top ve sap açıklık halkasından iner. Halka gerçek V yüzeyini izler. Görüntüde halkanın açıklığı ile topun deri düzlemindeki kesiti arasındaki açık halka koyu gösterilir (V3-10).
* **Görüş alanı.** Pencere merkezinden ve kenarından çıkan ışınlarla bulunur; engeller gövde, iki kanat, kuyruklar, ventral ve pervane diskidir. Azimuta göre engelsiz en yüksek yükselim (0° = ileri):

| Azimut | 0° | 10° | 20° | 30° | 40° | 50° | 60° | 70° | 80° | 90° | 100° | 110° | 120° | 130° | 140° | 150° | 160° | 170° |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| Üst sınır | −3° | 1° | 5° | 10° | 13° | 17° | 19° | 20° | 21° | 21° | 21° | 20° | 5° | 5° | 1° | −1° | −1° | −1° |

| Azimut | 180° | 190° | 200° | 210° | 220° | 230° | 240° | 250° | 260° | 270° | 280° | 290° | 300° | 310° | 320° | 330° | 340° | 350° |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| Üst sınır | −3° | −1° | −1° | −1° | 1° | 5° | 5° | 20° | 21° | 21° | 21° | 20° | 19° | 17° | 13° | 10° | 5° | 1° |

En kötü değer −3°'dir. Gereksinim (R-25), her azimutta nadirden en az −5°'ye kadar görüştür ve karşılanır.

## 12. Yerleşim, yakıt ve elektrik (F16, V1-02, V2-01, V3-02, V3-08, V3-09)

Yakıt: 49,6 L kullanılabilir (dolu depolar 35,95 kg), 42,1 L gerekli (görev yüküyle 30,6 kg yakıt); yakıt AM'si x = 2,64 m. Yerleşim bölgeleri `spec.yaml → layout.zones_preliminary` altındadır; hepsi gövdeye sığar ve çakışmaz (R-26, R-27).

**Aviyonik ve güç bölmeleri (V3-02).** v1.4'ün tek kutusu (0,26 × 0,14 × 0,06 m) bildirdiği içerikleri taşıyamıyordu. Yalnız PDU (235 × 195 × 54,5 mm) bile kutudan büyüktü; R-26 ise içerikleri değil yalnız kutuyu denetliyordu. v1.5'te burun bölgesi üç yerleşim bölgesine ayrıldı:

* burun takımı kuyusunun üstünde güç anahtarlama bölmesi (x 0,672–0,797 m) ve PDU güvertesi (x 0,797–1,085 m);
* kuyunun iki yanında bir yan bölme çifti (y 0,05–0,142 m, x 0,857–1,100 m). Sol yan bölmede batarya, sağ yan bölmede aviyonik vardır.

Bileşenler `layout.rules.bay_contents` içinde gerçek zarflarıyla yerleştirildi:

* veri sayfası kutuları components.yaml'dan okunur;
* batarya paketi hücre veri sayfasından kurulur: Molicel INR-21700-P45B, Ø 21,40 ± 0,15 mm, boy 70,00 ± 0,15 mm, en büyük değerler; 8 × 3 dik dizilim; kutu duvarı ve BMS/sigorta/bara payı tahmindir;
* birim seçilmemiş iki kalem (28–12 V DC-DC, ana kontaktör/sigorta) ayırma kutularıyla temsil edilir;
* her zarfa bir yüzünde bağlayıcı/kablo payı eklenir (tahmin).

`bay_contents_check` her zarfı denetler: OML'ye en az 10 mm, bölge duvarına en az 3 mm, komşu zarfa en az 10 mm. Sonuç R-26'nın parçasıdır.

| Öğe | Bölge | Zarf (mm) | OML'ye pay | Duvara pay | Komşuya en yakın |
|---|---|---|---|---|---|
| Ana kontaktör + sigorta (ayırma kutusu) | güç anahtarlama | 100 × 60 × 50 (+15) | 24,1 mm | 3,0 mm | 14,0 mm |
| 28–12 V DC-DC (ayırma kutusu) | güç anahtarlama | 60 × 60 × 25 (+15) | 59,5 mm | 5,0 mm | 14,0 mm |
| VISIONAIRtronics 1000 W PDU | PDU güvertesi | 235 × 195 × 54,5 (+40 bağlayıcı) | 18,2 mm | 3,0 mm | 11,4 mm |
| 12S2P Li-ion batarya (havalandırmalı kutu) | sol yan bölme | 184,4 × 76,6 × 96,2 (+20) | 17,9 mm | 3,2 mm | 11,4 mm |
| Veronte Autopilot 1x (GNSS/IMU içinde) | sağ yan bölme | 76 × 65 × 40 (+25) | 21,4 mm | 3,0 mm | 10,0 mm |
| Silvus SC4200EP (EP kutusu; modül boyutu yayımlanmamış) | sağ yan bölme | 101,6 × 66,8 × 38,4 (+20) | 40,4 mm | 3,4 mm | 11,6 mm |
| Microhard pDDL2450 (kutulu) | sağ yan bölme | 77 × 55 × 28 (+20) | 73,2 mm | 3,0 mm | 10,0 mm |
| uAvionix ping200X | sağ yan bölme | 47 × 54 × 13 (+20) | 97,6 mm | 15,0 mm | 10,0 mm |
| Dronetag Beacon | sağ yan bölme | 37 × 26 × 16 | 100,0 mm | 14,0 mm | 11,6 mm |

Yan bölmeler burun takımı yuvasından (omurga yarığı) en az 11 mm uzaktadır. PDU'nun altı kuyunun üstünden 9 mm yukarıdadır. Bölmeler taret bölmesine en az 20 mm uzaklıktadır. Batarya ön yakıt hücresinden 1,16 m uzaktadır (gereken > 1 m). Bataryanın sol bölmede olması boş kütlenin ağırlık merkezini 1,5 mm sola kaydırır (analysis/mass.py sınırı 5 mm). Aviyonik/güç kütle kalemleri paketlenmiş içeriklerin kütle merkezlerine taşındı; kanat konumu yalnız 0,2 mm değişti, statik marj değişmedi.

Bunun bir test karşılığı da vardır: v1.4 kutusu ve gerçek PDU ile kontrolün başarısız olması (`TestBayContents`).

**Elektrik (R-32, R-52).**

*Yükler.* Sürekli yük, temel 303 W (HD59 ortalaması 20 W dahil) ile 50 W araştırma yükü payının toplamıdır: 353 W. E180 ile sürekli yük 373 W'tır. Taret tepe yüküyle toplam HD59 ile 433 W, E180 ile 493 W'tır.

*Jeneratör.* DC çıkış = 800 W × rpm/7500 × 0,94 (güç elektroniği verimi tahmindir; iPS750 verisiyle değiştirilecek).

*E180 büyüme görevi (V3-08).* v1.4'te bu görev de tasarım görevinin HD59 yüküyle (353 W) uçuluyordu. v1.5'te motor E180 sürekli yükünü (373 W) taşır: jeneratör mil çekişi ve alçalma devir tabanı (3.720 rpm) buna göredir. E180 görevi 10,15 h'tir.

*R-52 (tepe yük batarya desteği).* E180 tepe yükü bütün bekleme boyunca jeneratör ve tampon bataryanın tepe destek payıyla karşılanır:

* **Tampon batarya:** 12S2P Molicel INR-21700-P45B Li-ion (components.yaml), 389 Wh, 1,98 kg. Kullanılabilir enerji (%80) 311,2 Wh'tir. Jeneratör kaybı ve marş yedeği 400 W × 28 dk = 186,7 Wh'tir. Kalan **124,5 Wh tepe destek payıdır**.
* **Bekleme devri tabanı:** jeneratör E180 tepesinin en çok 17 W altına düşer (4.747 rpm). v1.4'te bu açık 18 W idi. v1.5'te görev yükü 19,0 kg'a inince yakıt ve bekleme süresi arttı. 18 W ile bataryadan çekilecek enerji 128 Wh'e çıkıyordu ve R-52 0,973'te kalıyordu. Açık 0,5 W adımlarla tam görev modeliyle yeniden arandı: 18 W → 0,973; 17,5 W → 0,995; **17 W → 1,019**; 16 W → 1,069. v1.6'da görev yükü 18,0 kg'a inince yakıt ve bekleme yine arttı; arama 18,0 kg ile tekrarlandı: 17,5 W → 0,979; **17 W → 1,002**; 16 W → 1,052. 17 W korundu.
* **Ölçüt (R-52):** tepe yük bütün bekleme boyunca sürseydi bataryadan çekilecek enerji tasarım görevinde 124,3 Wh, E180 görevinde 98,0 Wh'tir. Pay 1,002'dir; görev yükü daha da azalır ve bekleme uzarsa açık sınırı yeniden aranmalıdır. Görev çevrimi kredisi alınmadı. Kısa tepeler (≤ 180 s) SG750'nin 1 kW/3 dk kısa süreli anma değeriyle de karşılanabilirdi; bu da kredi olarak kullanılmadı.
* Jeneratör tek başına, tasarım görevinin en düşük bekleme devrinde E180 tepesini 0,966, HD59 tepesini 1,099 payla karşılar. Sürekli yük payı (R-32): 1,276.

*R-52 işletme sınırı (V3-09).* Tepe destek payı, R-52'nin denetlediği iki görevin bekleme sürelerine göre boyutlanmıştır: görev yüküyle tasarım görevi (8,10 h) ve E180 görevi (7,85 h). Daha hafif bir yük daha çok yakıt ve daha uzun bekleme demektir. O zaman E180 tepe yükü bütün bekleme boyunca karşılanamaz. E180 sütunları yalnız E180 setini (E180 4,0 + görev bilgisayarı 1,0 + tepsi 0,5 = 5,5 kg) taşıyabilen yükler içindir (V4-02):

| Faydalı yük | Bekleme | Tepe yük bütün bekleme sürseydi çekilecek enerji | E180 tepesinin desteklendiği bekleme |
|---|---|---|---|
| 3,25 kg (temel set) | 10,58 h | — (E180 seti sığmaz) | — |
| 5 kg | 10,51 h | — (E180 seti sığmaz) | — |
| 7,5 kg | 10,42 h | 176,3 Wh (> 124,5 Wh) | 7,37 h |
| 10 kg | 10,31 h | 170,2 Wh (> 124,5 Wh) | 7,63 h |
| 12,5 kg | 10,19 h | 160,1 Wh (> 124,5 Wh) | 8,10 h |
| 15 kg | 9,26 h | 143,9 Wh (> 124,5 Wh) | 8,12 h |
| 17,5 kg | 8,29 h | 127,5 Wh (> 124,5 Wh) | 8,12 h |
| **18,0 kg (görev yükü)** | 8,10 h | 124,3 Wh | bütün bekleme |
| 20 kg (azami) | 7,33 h | 111,3 Wh | bütün bekleme |

İşletme kuralı şöyledir:

* Açık her an tam 17 W olsa bile pay en az **7,33 h** bekleme yeter (124,5 Wh / 17 W).
* Pay bitince güç yönetimi bekleme devrini, jeneratörün E180 tepesini tek başına taşıdığı 4.917 rpm'e çıkarır (dayanım bedeliyle) ya da tepe yük süresi sınırlanır.
* Batarya hiçbir durumda jeneratör kaybı/marş yedeğinin (186,7 Wh) altına inmez.

Bu sınır uçuş planlama ve güç yönetimi yazılımına taşınmalıdır (§16).

* **Güvenlik:** Li-ion'un ısıl kaçak payı LiFePO4'ten küçüktür. Bu yüzden hücre düzeyinde sigorta, havalandırmalı ayrı bir batarya kutusu ve yakıt hücrelerinden 1 m'den uzak bir bölme zorunludur (sol yan bölme, ön yakıt hücresine 1,16 m). 12S paketin en yüksek gerilimi 50,4 V'tur; PDU girişi 24–55 V'tur. Marş akımı paket için 2 × 45 A = 90 A'dir; SG750'nin marş akımı açık konudur.
* **Neden LiFePO4 değil:** aynı tepe destek payı LiFePO4 ile iki kat batarya kütlesi ister. E180 tepesini yalnız jeneratörle karşılamak (v1.2 çözümü) dayanımı düşürür (§14).

## 13. Ödünleşimler

Komut: `python3 -m ucav250.analysis.sizing --trades` ([`out/sizing_trades.json`](../out/sizing_trades.json)). Her satır tam bir tasarım kapanışıdır ve görev yüküyle (18,0 kg) uçulur. R-25 bu hafif değerlendirmelerde hesaplanmaz.

**Pervane:**

| Pervane | Dayanım | Tırmanma (DS) | Kalkış / iniş koşusu | Statik itki | Karşılanmayan |
|---|---|---|---|---|---|
| Mejzlik 31x12 3B (seçilen) | 10,41 h | 4,21 m/s | 179 / 183 m | 492 N | — |
| Mejzlik 32x18 2B | 8,67 h | 3,55 m/s | 207 / 182 m | 446 N | R-02, R-02b, R-05, R-06, R-21, R-26, R-33, R-50, R-56 |

Ara tasarım noktalarında taranan daha ince hatveli Mejzlik pervaneleri (30x13 3B, 30,5x13,5 2B, 32x10 2B) beklemede daha yüksek devirde dönerek devir tabanından kurtulur. Ancak hepsi statik uç Mach sınırını (R-31 ≤ 0,75; 0,83–0,89) bozar.

**İniş takımı ve taret** (kullanıcı kararlarının bedeli; MTOM sabit olduğundan fark yakıta gider):

| Seçenek | Dayanım (görev yüküyle) | Fark |
|---|---|---|
| İçeri katlanır takım, altı contalı kapak (seçilen) | 10,41 h | — |
| Sabit, kaportalı takım | 12,00 h | +1,58 h |
| Geri çekilir taret, açıklık halkası (seçilen) | 10,41 h | — |
| Sabit taret (hep dışarıda) | 11,07 h | +0,66 h |

Bu iki seçenek bir arada yaklaşık 2,2 h dayanım verirdi (v1.5'te 2,0 h; v1.6'da içeri katlanır takımın kapak kalemi büyüdü). Kullanıcı ikisini de açıkça seçtiği için uygulanmadılar. Bedel gereksinim kararıyla faydalı yükten ödendi.

**Kanat açıklığı ve kanat yüklemesi:**

| Açıklık | VS hedefi | S | AR | Kanat | Boş | (L/D)maks | Dayanım | Kalkış | Dış panel |
|---|---|---|---|---|---|---|---|---|---|
| **7,20 m** | 23,95 m/s | 3,241 m² | 16,0 | 16,40 kg | 101,34 kg | 17,3 | **10,41 h** | 179 m | 2,90 m |
| 6,80 m | 23,95 m/s | 3,234 m² | 14,3 | 15,77 kg | 100,87 kg | 16,5 | 10,37 h | 186 m | 2,70 m |
| 7,60 m | 23,95 m/s | 3,249 m² | 17,8 | 17,08 kg | 102,15 kg | 17,9 | 10,24 h | 173 m | 3,10 m |
| 7,20 m | 23,00 m/s | 3,505 m² | 14,8 | 17,25 kg | 103,22 kg | 17,0 | 9,59 h | 168 m | 2,90 m |
| 7,20 m | 24,50 m/s (R-08 ✘) | 3,098 m² | 16,7 | 15,95 kg | 100,42 kg | 17,4 | 10,77 h | 186 m | 2,90 m |

**Kuyruk kolu (gövde boyu):**

| Gövde uzatma | Gövde boyu | V_H | Boş | Dayanım | Cnβ |
|---|---|---|---|---|---|
| −0,10 m | 3,90 m | 0,431 | 101,40 kg | 10,35 h | 0,0593 |
| +0,10 m | 4,10 m | 0,444 | 101,36 kg | 10,42 h | 0,0589 |

**Arka kapanış (V2-02).** Sivrilen bir kuyruk konisiyle (kapanış 0,05 → 0,50 m, pervane 0,45 m geride, uç eğimi 12°) tam kapanış yapıldı. Sonuç: 15° ölçütüyle taban sıfır, CD0 −0,0020; ama gövde kabuğu +0,90 kg, şasi +0,16 kg (boş kütle +1,0 kg). Daha uzun takım gerektiği için R-21, R-26 ve R-33 bozuldu. Pervane mil uzatmasının kütlesi ve titreşimi hariç dayanım v1.3 modelinde 0,20 h düştü. 0,25–0,30 m'lik bir koni 15° ölçütüyle tabanı ancak 0,112 → 0,091–0,100 m²'ye indirir. **Reddedildi.** Küt taban ve 0,93 kurulum katsayısı tahmini korundu.

**Daha fazla aşağı itki (6–8°).** Stabilatör küçülür, ama dikeyler (eğik disk düzleminde koruma kuralı) büyür: dayanım −0,02…−0,04 h. **Daha küçük kalkış flabı (20–30°).** +0,005…+0,03 h; ama yerden kesilme tutumu artar ve ana takım toplanmaz (R-21, R-26); 25° altında R-06 da bozulur. İkisi de reddedildi (v1.4 değerlendirmesi).

## 14. R-02'nin geçmişi, tasarım değişiklikleri merdiveni ve kütle bütçesi

**Geçmiş.**

* v1.2 R-02'yi tasarım değişiklikleriyle karşılamıştı.
* v1.3'ün 10,06 h'i, kullanıcı tarafından onaylanmamış bir R-52 gevşetmesine dayanıyordu.
* v1.4 bu gevşetmeyi geri aldı ve k_inst'i 0,93 aldı; 20 kg yükle dayanım 9,90 h oldu ve R-02 karşılanmadı.
* v1.5'te proje lideri gereksinimi yeniden tanımladı (Gereksinim kararı bölümü): R-02 görev yüküyle (19,0 kg) 10,27 h, R-02b 20 kg ile 9,88 h.
* v1.6'da takım kapağı kalemi kapak düzenine göre yeniden kuruldu (+0,62 kg boş kütle, V4-03). Kural aynı yöntemle 18,0 kg verdi: R-02 10,41 h, R-02b 9,64 h.

R-52 gereksinimi ve k_inst tahmini değişmedi; yalnız R-52'nin jeneratör açığı sınırı (bir tasarım tercihi) v1.5'te uzayan bekleme için 18 W'tan 17 W'a indirildi ve v1.6'da 18,0 kg ile yeniden denetlendi (§12). Hiçbir gereksinim, kontrol ya da test gevşetilmedi.

**Tasarım değişiklikleri merdiveni.** Her satır, yalnız o değişiklik yapılıp tam tasarım kapanışı yeniden kurulduğunda görev yüküyle elde edilen dayanımdır (`out/sizing_trades.json → r02_ladder`). Etkiler tam toplanabilir değildir. Hiçbiri uygulanmadı; tablo her kararın değerini gösterir.

| Değişiklik | Dayanım | Fark |
|---|---|---|
| **v1.6 tasarımı (görev yükü 18,0 kg)** | **10,41 h** | — |
| k_inst 0,95 (tahmin 0,93 yerine; pervane/gövde kurulum testi bekliyor) | 10,54 h | +0,12 h |
| R-52 v1.3 biçimi (yalnız takılı taretin tepesi; kullanıcı onaylamadı, uygulanmadı) | 10,58 h | +0,17 h |
| v1.2 çözümü: LiFePO4 14S2P batarya, E180 tepesi yalnız jeneratörle (E180 devir tabanı) | 9,94 h | −0,47 h |
| k_inst 0,95 ve R-52 v1.3 birlikte (ikisi de uygulanmadı) | 10,75 h | +0,33 h |
| c/4 süpürme 8° → 6° (v1.2) | kapanmıyor: v1.6 kapanışında LERX hücum kenarı açıklık boyunca monoton olmuyor (v1.5'te 10,13 h) | — |
| burulma 4° → 3° (v1.2) | 10,33 h | −0,08 h |
| LERX tepesi x 1,80 → 1,85 m (v1.2) | 10,40 h | −0,01 h |
| kapaklar contalı değil (v1.1: uzatılmış takım sürüklemesinin %10'u) | 10,36 h | −0,05 h |
| eldiven/dış panel birleşimi y = 0,82 m (v1.1) | 10,15 h | −0,27 h |
| sivrilme 0,42 (v1.1) | 10,31 h | −0,10 h |

**Kütle bütçesi (V1-03, R-56).** Grup tavanları ayrıntılı tasarım için kesin sınırdır. Her tavan, o grubun %5 büyüme paylı tahmininin 10 g'a yukarı yuvarlanmış değeridir. v1.6'da üç tavan, kalemlerin gerçek artışı kadar yükseltildi: iniş takımı 13,75 → 14,34 kg (yeniden kurulan kapak kalemi, V4-03), bağlantı elemanları 1,50 → 1,51 kg (takımın büyüyen payı), kuyruk 7,87 → 7,89 kg (stabilatör kapanışta +%0,4). Hiçbir kalem silinmedi; tavan, kalem silerek değil, pay içinde kapanır. R-56 iki gereksinime göre denetlenir:

* R-02'yi (görev yükü 18,0 kg, 10 h) tam karşılayan boş kütle 102,398 kg'dır;
* R-02b'yi (20 kg, 9,5 h) tam karşılayan boş kütle 101,695 kg'dır.

Belirleyen artık R-02b'dir. Tavanların toplamı 101,39 kg ve yedek 0,05 kg ile pay **+0,255 kg**'dır (R-02'ye göre +0,958 kg). Bu pay hiçbir gruba dağıtılmadı; ayrıntılı tasarım için yönetim payıdır.

**Üç ayrı pay (V4-04).** Boş kütle artışına karşı üç ayrı eşik vardır ve hepsi aynı şeyi ölçmez:

* **Görev yükü kuralı (10,25 h):** görev yüküyle dayanım eşiğin 0,162 h üstündedir; bu **+0,42 kg** boş kütleye denktir. Daha büyük bir artışta kural 17,5 kg'ı verir (R-02 yine karşılanır).
* **R-02b (9,5 h, 20 kg):** **+0,25 kg** (R-56'yı belirleyen). Bunu aşan bir artış, bütçeyi yeniden dağıtmayı ya da kararı yeniden açmayı gerektirir.
* **R-02 (10 h, görev yüküyle):** **+0,96 kg**.

| Grup | Tahmin (kg) | Tavan (kg) | Tavanın tahmin üstündeki payı (kg) |
|---|---|---|---|
| Kanat | 16,400 | 16,40 | 0,000 |
| Sistemler | 14,657 | 14,66 | 0,003 |
| İniş takımı | 14,335 | 14,34 | 0,005 |
| İtki | 14,303 | 14,31 | 0,007 |
| Şasi | 11,143 | 11,15 | 0,007 |
| Gövde kabuğu | 10,467 | 10,47 | 0,003 |
| Kumanda | 8,304 | 8,31 | 0,006 |
| Kuyruk | 7,881 | 7,89 | 0,009 |
| Yakıt sistemi | 2,341 | 2,35 | 0,009 |
| Bağlantı elemanları | 1,506 | 1,51 | 0,004 |

Bilinen yönlü belirsizlikler kredi olarak kullanılmadı:

* arka taban sürüklemesi itici emişi olmadan sayıldı;
* stabilatör CN_maks değeri 3B azaltma almadı;
* E180 tepe yükü bütün bekleme boyunca varsayıldı;
* SG750'nin 1 kW/3 dk kısa süreli değeri kullanılmadı;
* takım açık sürüklemesinde açık kapak payı korundu.

Ayrıntılı tasarım fazının ilk ölçümleri şunlar olmalıdır:

* pervane/gövde kurulum testi (k_inst, iz uyarımı);
* rüzgâr tüneli/CFD (taban sürüklemesi, CLmax, birleşimler);
* Limbach düşük yük BSFC haritası;
* ePropelled iPS750 dönüştürücü verimi;
* Octopus E180 arayüz belgesi (tepe yükün süresi).

## 15. Spec ve doğrulama

`spec.yaml` v1.6 şu bölümleri içerir: `meta`, `requirements` (61), `mission`, `engine`, `propeller`, `configuration`, `wing`, `tail`, `fuselage`, `landing_gear`, `payload`, `aero`, `mass`, `stability`, `performance`, `structures`, `materials`, `adhesives`, `layups`, `processes`, `display`, `layout`, `assembly`. v1.6'da eklenen ya da değişen bölümler:

| Bölüm | v1.6 içeriği |
|---|---|
| `meta` | revizyon v1.6; açıklama beşinci tura göre genişletildi |
| `requirements` | R-60'ın metni ve kaynağı (kırpılmamış pay, tutarlılık kapısı, yunuslama ataleti) |
| `mission` | `payload_design_kg` 18,0 (türetilmiş), `rotation_spin_up_s` 0,3 (yeni, tasarım tercihi); R-52 açık sınırının kaynağı 18,0 kg aramasıyla |
| `mass.rules.gear_doors` | `gear_doors_locks_kg` (1,6 kg) yerine yapılandırılmış kapak kuralı: alan kütlesi, kuyu kapatmaları, kesik takviyesi, kilitler, conta/m, iç kapak eyleyicisi (components.yaml Volz DA 22 başvurusu), bağlantılar, bacak kapağı ayakları |
| `mass.items` | `gear_doors_wells_locks_sensors` 2,268 kg; boş kütle 101,336 kg |
| `mass.budget` | iniş takımı 14,34, bağlantı elemanları 1,51, kuyruk 7,89 kg tavanları; not yeniden yazıldı |
| `mass.cases` | `full_fuel_baseline_sensors_only` dolu depolu (`fuel_rule: capacity`) |
| `performance` | `payload_design_rule` (eşiğe pay), `payload_permitted_loadings` (yeni), `takeoff_pitch_inertia_effect` (yeni), yeni yük – dayanım tablosu |

`--check`, kapanışı spec girdilerinden yeniden kurar ve 1224 türetilmiş değeri spec ile karşılaştırır. Bunların içinde görev yükü kuralı (spec'teki 18,0 kg = türetilen değer), payload kalemleri, yükleme durumlarının yakıt kesirleri (dolu depolu durum dahil) ve kapak kalemi de vardır. Hepsi tolerans içindedir ve kapanış yakınsar. Ardından 61 gereksinim değerlendirilir; hepsi karşılanır ve **çıkış kodu 0'dır**.

`tests/test_ucav250_sizing.py` kontrolü geçici bir dizine çalıştırır ve izlenen dosyaların değişmediğini doğrular. v1.6'da `tests/test_ucav250_*.py` testlerinin hepsi geçiyor. Hiçbir test gevşetilmedi. v1.6'da değişen ve eklenen testler şunlardır:

* `test_fix_round4_checks`: yük – dayanım tablosu artık temel EO/IR setiyle başlar (önceki "ilk satır 0 kg" iddiası V4-02'nin kusurunu kodluyordu; yerine daha sıkı denetimler `test_fix_round5_checks`'te).
* `test_fix_round5_checks` (yeni): her MTOM kalkış yüklemesinde I_yy, rampa süresi, V_R ≥ ataletsiz V_R, ataletin koşuya etkisi ve 0,1 s rampayla R-06; R-60 ölçütünün kırpılmamış pay olması; yük – dayanım satırlarının hepsinin izin verilen yükleme olması (SM ≥ %10); E180 sütunlarının yalnız E180 setini taşıyan yüklerde dolu olması; yüksüz yüklemenin izin verilmemesi ve safra; yalnız taretle yakıt sınırı; dolu depolu durumun yakıtının hücre kapasitesine eşit olması; taramada kırılma yükü ve 0,5 kg komşuları; kapak kaleminin kapak düzeninden kurulması (alanlar, iki DA 22, ayaklar) ve birleşim uzunluğunun ayırma çizgilerini içermesi; görev yükünün eşiğe payı; Türkçe kural metni.
* `test_retracted_belly_door_seams` (yeni): içeri katlanmış alt görüntüde iç kapak ve iki bacak kapağı birleşim çizgileri (V4-07).
* `test_r60_fails_when_the_rotation_download_exceeds_the_maximum` (yeni): dönüşte azami aşağı kuvvet 0,9 ile çarpılınca R-60 başarısız olur; tutarlılık kapısı (moment artığı 1e-3) ölçütü negatif yapar (V4-01).
* `TestDoc02`: revizyon, görev yükü ve R-02b değeri bu raporda.

Görüntüler (`docs/fig/yk250_iso.png`, `yk250_rear.png`, `yk250_top.png`, `yk250_side.png`, `yk250_front.png`, `yk250_belly.png`, `yk250_turret_gear_retracted_belly.png`) ve 3 görünüş spec geometrisinden yeniden üretildi ve tek tek incelendi.

## 16. Açık konular

1. **Görev yükü 18,0 kg.** Kural her model güncellemesinde yeniden uygulanır; yük 0,5 kg adımlarla değişebilir (v1.5 19,0 kg → v1.6 18,0 kg, kapak kalemi). 20 kg ile dayanım 9,64 h'tir (R-02b).
2. **İtici kurulum katsayısı ve pervane iz uyarımı.** k_inst 0,93 bir tahmindir (V3-06). Küt kaporta tabanının ayrılmış izi diskin iç bölgesinde çalışır. Pervane/gövde kurulum testi k_inst'i, pala yük değişimini ve titreşimi (1P/2P/3P) Mejzlik ve Limbach ile birlikte ölçmelidir. k_inst 0,90 olsa dayanım 10,22 h olur (R-02 karşılanır, kural eşiğinin altında kalır).
3. **Düşük yükte BSFC (V3-07).** Bekleme %23–%25 güçtedir. Alçalma (%5–6) ve yedek bekleme (%18) ise en düşük BSFC noktasının (%20) altındadır. Willans doğrusu (daha ihtiyatlı bir dışdeğerleme, sınır değil) dayanımı 0,06 h düşürür. Limbach'tan düşük yük haritası istenmelidir.
4. **R-52 işletme sınırı (V3-09).** Tepe destek payı görev yüküyle tasarım görevinin ve E180 görevinin beklemelerine göre boyutlanmıştır. Daha hafif yükte pay bekleme bitmeden tükenir (§12). Güç yönetimi yazılımı payı izlemeli ve devir tabanını yükseltmelidir. E180 tepe yükünün gerçek süresi (arayüz belgesi) bu sınırı gevşetebilir.
5. **Bölme içerikleri (V3-02).** DC-DC ve kontaktör/sigorta ayırma kutularıdır; birim seçilmedi. Bağlayıcı payları ve batarya kutusu duvarları tahmindir. Silvus için EP kutusunun boyutu kullanıldı (modülün boyutu yayımlanmamış). Batarya havalandırma yolu ve PDU fan hava akışı yerleşim fazında tasarlanmalıdır.
6. **Dönüş yasası (V3-03, V4-05).** Yunuslama ataleti v1.6'da modellendi. I_yy kütle kalemlerinden bir tahmindir (yayılı kalemler ıslak alana ya da çubuğa yayılır; ekipmanın kendi ataleti sayılmadı); ayrıntılı tasarımın kütle modeli (analysis/mass.py) ile değiştirilmelidir. 0,3 s'lik hız rampası bir tasarım tercihidir. Dönüşten kaynaklanan yatay AM ivmesi (θ̈ h) ve merkezcil terimler boyuna integrasyonda ihmal edildi (küçük). Yasa uçuş kontrol yazılımı gereksinimlerine taşınmalı ve yer koşusu testleriyle doğrulanmalıdır.
7. **Arka kapanışın görünüşü.** Küt kaporta ucu bir ödünleşimdir (§6.3, §13).
8. **Li-ion tampon batarya.** Isıl kaçak önlemleri (hücre sigortası, havalandırma, yakıttan ayrım); marş akımı (90 A sürekli) SG750 ile doğrulanmalı; yaşlanmada kullanılabilir enerji (%80) izlenmeli.
9. **Güç elektroniği verimi.** 0,94 bir tahmindir. iPS750 verisi alınmalı; R-32, R-52 ve devir tabanı buna göre güncellenmelidir.
10. **MTOM payı yok (V4-04).** MTOM, M2 tavanı olan 149,9 kg'dadır. Her boş kütle artışı yakıttan düşülür. Üç ayrı pay vardır (§14): türetilen görev yükü (18,0 kg) boş kütlenin **+0,42 kg** artmasına dayanır (dayanım kural eşiğinin 0,162 h üstünde); daha büyük bir artışta kural 17,5 kg'ı verir. R-02b ve bununla R-56 **+0,25 kg**'a, R-02 (18,0 kg ile 10 h) **+0,96 kg**'a dayanır. v1.5 raporu "R-56 payı tükenirse kural daha küçük bir yük verir" diyordu; bu yanlıştı, çünkü kural eşiği (10,25 h) R-56'nın ölçtüğü 10 h'ten yüksektir.
11. **Çırpınma ve burulma rijitliği.** AR 16 kanat, tek eyleyicili hareketli stabilatörler ve eğik dikeyler analiz edilmedi.
12. **Kumanda hızları.** Stabilatör yüzey hızı −20°'de 36°/s'ye iner; dümen ve kanatçıkta da uçlara doğru düşer (§9). Menteşe momenti katsayıları tarama değerleridir (±%50); rüzgâr tüneli ya da CFD ile doğrulanmalıdır.
13. **İşletme sınırları.** Uçuş kontrol sisteminin hız koruması (45 m/s EAS) zorunludur.
14. **LERX aerodinamiği.** Hücum kenarı girdabının kaldırması sayılmadı; CLmax ve burun kalkması CFD/rüzgâr tüneliyle doğrulanmalıdır.
15. **Taşıyıcı gövdenin yön kararlılığı.** Gövde Cnβ için DATCOM kullanıldı (Raymer'den daha kötü).
16. **Pala modeli.** 925(c) boyuna açıklığı pala veteri ve hatve açısı tahminiyle hesaplandı; Mejzlik çizimiyle doğrulanmalıdır.
17. **Pervane koruması.** Yan bölgeler açıktır ve personel koruması yoktur.
18. **Kuyruk kökleri ve stabilatör milleri.** Kök bağlantı çerçeveleri, motor bölmesi ısısı ve titreşimi ayrıntı fazında tasarlanmalıdır.
19. **SG750 marş/jeneratörü.** Üretim durumu belirsizdir; Limbach'ın 28 V alternatörlü yedeği +6,6 kg getirir.
20. **İçeri katlanır takım ve kapaklar.** Satın alınabilir birim yoktur. Kapak kütlesi v1.6'da kapak düzeninden kuruldu (V4-03), ama kapak alan kütlesi kimlik çalışmasının tahminidir; iç kapak bağlantıları ve bacak kapağı ayakları tahmindir. İç kapak eyleyicisinin (Volz DA 22) menteşe momenti sayısal olarak denetlenmedi; kapak akış düzleminde olduğundan küçük beklenir. Bacak rijitliği, kilitler, acil indirme (iç kapakların serbest düşüşte açılması dahil), contalar ve sıralama ayrıntı fazında tasarlanmalıdır.
21. **Referans kavramlar.** `data/concepts/*/calc.py` hâlâ kapalı biçimli Breguet bağıntısını kullanır (§1).
22. **Yerleşim fazı.** `layout.zones_preliminary` kutuları ön yerleşimdir. `analysis/mass.py` bütçeyi hâlâ iki yönlü bir bant olarak okur.
23. **İzin verilen yüklemeler (V4-02).** Temel EO/IR seti her uçuşta takılı olmalıdır. Taretsiz uçuş taret bağlantısına en az 1,96 kg safra ister; yalnız taretle yakıt 10,8 kg ile sınırlıdır. Bu sınırlar uçuş planlama yazılımına ve uçuş el kitabına taşınmalıdır.
24. **R-52 payı ince.** Görev yüküyle R-52 1,002'dir. Görev yükü kuralı daha küçük bir yük verirse bekleme uzar ve jeneratör açığı sınırı yeniden aranmalıdır (§12).
