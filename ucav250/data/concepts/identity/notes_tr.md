# YK-250 — Konsept “Kimlik” (UCAV görünümlü MALE) — tasarım notları

Kaynak: `ucav250/data/concepts/identity/calc.py` (çalıştırma: `PYTHONPATH=. python3 ucav250/data/concepts/identity/calc.py`, yaklaşık 4 dk).
Tüm sayılar `concept.yaml` dosyasından gelir. Girdilerin kaynağı ya da dayanağı orada `inputs` altında kayıtlıdır. Tahmin olan girdiler `tag: estimate` ile işaretlidir.
Hesap, gözden geçirilmiş “Dayanım” çalışmasının `calc.py` dosyasını kütüphane olarak kullanır. İtki, polarlar, taşıyıcı çizgi, yapı, görev, performans ve kısıt denklemleri iki konseptte aynıdır. Bu dosya kimlik geometrisini, ek sürükleme kalemlerini, şasiyi, yer geometrisini ve takasları ekler.
Kapsam: sivil EO/IR gözetleme ve araştırma platformu. Silah, mühimmat, askı noktası, pilon ya da bırakma mekanizması yoktur. UCAV görünümü yalnız dış biçimdedir.

## 1. Konfigürasyon ve gerekçe

| Karar | Seçim | Neden (sayısal) |
|---|---|---|
| Gövde | **Köşeli (chined), fasetli kesit**. Üst fasetler n 1,35, alt fasetler n 1,75, düz çene n 2,4. Köşe çizgisi sivri burundan motor bölmesine kesintisiz uzanır. Motor üstünde sırt kamburu, kutu kesitli kaporta. | En güçlü UCAV işareti budur. Aynı hacimli düz eliptik gövdeye (n 2) göre bedeli 0,29 h'dir; köşe girdabı için gövde sürüklemesine %5 eklendi (tahmin). Gerçek düz fasetler (n 1) aynı hacim için ıslak alanı %7,7 büyütür (5,78 / 5,36 m²) ve 0,74 h götürür. Bu nedenle n 1,35 seçildi: köşede keskin kırım kalır, fasetler hafif dışbükeydir. |
| İtki | Limbach L 275 EF, **itici**, Mejzlik 32x18 2B (Ø0,813 m), 80 mm göbek ara parçası. Sırtta **S-kanallı soğutma girişi** (şevron dudak, sınır tabaka ayırıcı), çıkış pervane göbeği çevresinde halka. | Burun ve çene, köşeli burna ve EO/IR taretine kalır. S-kanal, burundan bakışta motoru gizler. Dinamik basınç geri kazanımı 0,88 kabul edildi (tahmin). Pitot kepçeye göre bedeli 0,07 h; gömülü NACA girişlerine göre 0,08 h daha iyidir. 32x18 2B, 31x12 3B'ye göre 0,48 h kazandırır (13,22 / 12,74 h). Bedeli tırmanmadır: 4,24 / 4,97 m/s. |
| Kanat | **Orta kanat** (köşe çizgisinde), AR 14, λ 0,42, hücum kenarı süpürmesi 12° (c/4 10,4°), NLF(1)-0416 %16 → %13, 3° burulma, 2,5° dihedral, i = 4,75°. **LERX tipi kök eldiveni** (0,30 m önde, 0,14 m açıklık). | AR × W/S taraması VS = 24 m/s köşesinde W/S 443 Pa verdi. Bu yüklemede 6,8 m açıklık sınırına AR 14 sığar. AR 14 için sonuçlar: 410 Pa 11,98 h (açıklık ✗), 443 Pa 12,54 h, 470 Pa 12,80 h (VS ✗). Kök eldiveni 0,03 h götürür. Süpürme bu yöntemde 0,28 h kazandırıyor görünür, iki nedenle. Birincisi: kanadın AM'si kök hücum kenarının 0,31 m arkasına kayar; kök, yakıt ve yük bloğu öne gelir, kuyruk kolu 1,21 m'den 1,27 m'ye uzar. İkincisi: baz yüklemenin burun yukarı momenti vardır (Cm0 +0,022). Buna karşılık açıklık verimi düşer (e_inv 0,970 → 0,917). Net kazanç yöntem doğruluğu sınırındadır; süpürme “bedava” sayıldı. |
| Kuyruk | **Y-kuyruk**: 40,3° açılı V paneller (0,99 m², NACA 0012, ruddervator) + **karın yüzgeci** (0,047 m²). Yüzgecin ucundaki kızak pervaneyi korur. | Kızak 9,4° burun yukarıda yere değer, pervane ancak 11,0°'de değerdi. Yüzgeç Cnβ'ye de katkı verir. Yüzgeçsiz V + kuyruk tamponuna göre bedeli 0,27 h'dir (0,52 kg ve D/q 0,0007 m²). Ters V'de panel uçları yere iner; takım 0,56 m uzamalıdır: CD0 +0,0016, dayanım −0,24 h. T ve klasik kuyruk modellenmedi. Görünüm hafif uçak olur; Dayanım çalışmasında V ile farkları 0,7 h içindeydi. |
| İniş takımı | **Sabit üç tekerlek**, GFRP yay + kaportalı TOST 200x50, kaportalı yönlendirilebilir burun bacağı | İçeri katlanır takım CD0'ı %9 düşürür (−0,0029), ama boş kütleyi 3,4 kg artırır: 2,97 kg parça + takım boşlukları için 0,24 m gövde uzatması. Sonuç: dayanım −0,97 h, 10 h görev için MTOW +3,5 kg. Satın alınabilir ürün yoktur. Yeni arıza türü getirir: takım açılmazsa uçak çene taretine ve karın yüzgecine oturur. Kaportasız sabit takım −1,20 h verir. |
| Flap | Yalnız **kalkış flabı** (iç, η 0,07–0,55, %25 veter, 15°) | ΔCLmax 0,165. Uçak kanadın yer açısında 26,3 m/s'de düz kalkar; yerde koşu 162 m. |
| Yerleşim | Faydalı yük bölmesi AM'de, kiriş tünelinin altında; önünde ve arkasında birbirine bağlı iki yakıt hücresi. Çene tareti x 0,55 m (batarya üstünde), aviyonik 0,84, PDU 1,12, paraşüt 0,99–1,37 m (köşe güvertesinde), motor arkada. | Yakıt AM'de tüketilir (yakıt AM'si x 2,430 m). Tasarım yükünde SM 0,100–0,118 kalır. E180 büyüme tareti (4,0 kg, çenede) AM'yi 31 mm öne alır, SM 0,18'e çıkar. |
| Gövde boyu | Göbek yüzü x 3,95 m (gövde 4,00 m, kök dahil 4,17 m, L/b 0,62) | Taramada x 3,75 / 3,95 / 4,15 m için 12,23 / 12,30 / 12,29 h bulundu; 4,15 m'de L/b 0,65 sınırı aşılıyor. |

## 2. Ana sayılar (MTOM 145,0 kg, tavan 149,9 kg)

| | |
|---|---|
| Boş / yakıt / faydalı yük | 90,5 / 34,5 / 20,0 kg. Boş kütle oranı 0,624; %5 büyüme payı olmadan 0,594; benzerlerde 0,55–0,62. |
| Kanat | S 3,21 m², b 6,70 m. Veter: merkez hatta 0,674 m, gövde yanında 0,645 m, uçta 0,283 m; OAK 0,505 m. W/S 45,2 kg/m². |
| Gövde / kuyruk | Gövde 4,00 × 0,50 × 0,52 m, ıslak alan 5,36 m². Kuyruk kolu 1,27 m, V_H 0,450, V_V 0,0271 (yüzgeç dahil). |
| Aerodinamik | CD0 0,0329, e 0,754, (L/D)maks 15,9. CLα 5,60 1/rad. Temiz trimli CLmax 1,295 (en ön AM'de), kalkış flabıyla 1,426. Perdövites başlangıcı η 0,38. Cm0 −0,070: kesitlerden −0,092, süpürme + burulmadan +0,022. |
| Hızlar | VS 23,6 m/s (temiz), kalkış flabıyla 22,5 m/s. 3000 m'de bekleme 28,4 m/s EAS / 32,9 m/s TAS. En uzun menzil hızı 35,9 m/s. Vmaks 56,1 m/s (deniz seviyesi), 53,3 m/s (3000 m). |
| Dayanım | **13,2 h**: 2×100 km intikal, 3000 m'de bekleme, %10 yedek. Bekleme yakıt akışı 2,48 kg/h, 4186 dev/dk. Yalnız temel algılayıcılarla ve faydalı yük bölmesinde 8,9 kg yardımcı yakıt torbasıyla 17,4 h. **10 h görev için MTOW 134,7 kg** (sizinglib.MassModel kapalı çevrim). |
| Menzil | 1448 km (feribot, 3000 m, %10 yedek) |
| Tırmanma / tavan | 4,24 m/s deniz seviyesinde, 2,32 m/s 3000 m'de (hedef 4,9 m/s; pervane sınırlı). Servis tavanı 6610 m. |
| Pist | Kalkış yerde koşusu 162 m, 15 m'ye 290 m. İniş yerde koşusu: görev sonunda 178 m, MTOM'da iptal inişinde 222 m. |
| Kararlılık | SM 0,100–0,180 (7 yükleme durumu). AM x 2,422–2,463 m, NN 2,513 m. Cnβ 0,057 1/rad: V paneller 0,101, yüzgeç 0,003, gövde −0,047. |
| Yer geometrisi | İz 0,86 m, dingil arası 1,81 m. Geriye devrilme açısı 15°, yana devrilme açısı 54,9°. Burun tekerleğine düşen yük %8–10. Pervane yer açıklığı yatayda 0,280 m, kalkışta 0,18 m. Uç Mach sayısı 0,716 (statik). |
| Yapısal yük | Limit n 5,13 (VC 45 m/s, 4500 m'de rüzgâr darbesi), nihai 7,70. Kök eğilme momenti 7855 N·m (nihai), kiriş başlığı 225 mm². |
| Duyarlılık | BSFC ±%12 → 15,0 / 11,8 h; CD0 +%10 → 12,7 h; boş kütle +%5 → 11,2 h. |

**Kimlik bedeli.** Her kimlik özelliği tek tek sade karşılığıyla değiştirildi. Her satır, aynı kurallarla yapılmış tam kapalı bir çözümdür. Δ değeri “kimlik özelliğinin etkisi”dir; eksi işareti bedeldir.

| Kimlik özelliği | Sade karşılığı | Δ dayanım (MTOM 145 kg) | Δ MTOW (10 h görev) |
|---|---|---|---|
| Köşeli fasetli gövde | Aynı hacimli eliptik gövde (n 2) | −0,29 h | +0,8 kg |
| LERX kök eldiveni | Eldivensiz kök | −0,03 h | +0,1 kg |
| 12° hücum kenarı süpürmesi | Süpürmesiz c/4 | +0,28 h | −0,7 kg |
| Karın yüzgeci (Y-kuyruk) | V-kuyruk + kuyruk tamponu | −0,27 h | +0,8 kg |
| Sırt S-kanal girişi | Pitot kepçe | −0,07 h | +0,2 kg |
| **Hepsi birlikte** | Sade referans | **−0,37 h (%2,7)** | **+1,2 kg** |

## 3. Yapı konsepti: şasi + kabuk

**Şasi (birincil yapı, bütün tekil yükleri taşır).** Gövde tabanında iki CFRP şapka profilli omurga kirişi (0,20 kg/m) vardır. Bunlara 11 sandviç perde bağlanır. Malzemeler: MTM45-1/AS4 OOA prepreg (121 °C + 177 °C serbest son kür) ve ROHACELL 51/71 WF-HT köpük. Yük yolları şunlardır:
- **Kanat:** Kanat yarılarının kiriş dili ve çatalı, gövde ortasındaki kiriş tünelinden geçer (x 2,14–2,49 m, z −0,075…+0,085 m). Eğilme momenti merkez hatta kendi içinde dengelenir; gövdeye yalnız kesme, sürükleme ve burulma iner. 2 ana Ti-6Al-4V pim, tünelin iki ucundaki perdelere (x ≈ 2,14 ve 2,49 m) bağlı talaşlı 7075-T651 bağlantılara oturur. Arka sürükleme pimleri arka tünel perdesine bağlanır. Yük, perdelerden omurga kirişlerine geçer. Tünelin altı faydalı yük bölmesi, üstü sırt güvertesidir (görev bilgisayarı, kayıt cihazı, telsizler).
- **İniş takımı:** GFRP yay (7781 cam + UD) x ≈ 2,61 m'dedir. İki 7075 kelepçeyle faydalı yük bölmesinin arka perdesine ve omurgaya bağlanır. Burun bacağı muylusu x 0,80 m perdesindedir; çene taret halkası x 0,55 m'de omurganın ön ucuna bağlanır.
- **Motor:** 4 × M8 ve Limbach izolatörleriyle TIG kaynaklı 4130 boru kafese bağlanır (AMS 6457 dolgu). Kafes, x ≈ 3,65 m'deki yangın duvarı perdesine bağlanır. Arkada 15 g ileri yönde tutma esastır.
- **Kuyruk:** V panellerin kök bağlantıları üst fasetler üzerinden aynı perdeye bağlanır. Karın yüzgecinin kökü (x 3,51–3,91 m) omurganın arka ucuna bağlanır. Kızağın ucunda değiştirilebilir cam/aramid aşınma pabucu vardır.
- **Paraşüt (GRS 4/240):** 13,1 kN açılma şoku, kızak tepsisinden x ≈ 1,37 m perdesine ve omurgaya iletilir. İkinci askı noktası tünel ön perdesine konur; uçak düz asılı iner (konsept önerisi).

**Kabuk (ikincil yapı, sökülebilir).** Gövde kabuğu 0,4 + 0,4 mm CFRP PW ile 5 mm ROHACELL 51 WF-HT sandviçtir. **Köşe çizgisi aynı zamanda kalıp ayırma hattıdır.** Üst ve alt kabuk iki dişi kalıpta üretilir. Kesitin en geniş yeri köşe olduğundan her iki kalıpta pozitif çıkış açısı kendiliğinden oluşur. Birleşim flanşı köşedeki kırımın içinde kalır ve UD karbon köşe bandıyla kapatılır (0,04 kg/m, tahmin). Bant, kırımı keskin ve rijit tutar. Kanat ve V-kuyruk kaplamaları birincil yapıdır: dış yüz 0,6 mm, iç yüz 0,4 mm sandviç, alt ve üst kabuk tek parça. Hücum kenarının ilk %8–10'una erozyon bandı uygulanır (NLF kuralı). Kanat kirişi UD başlıklı ve ±45 gövdelidir. Süpürme nedeniyle kiriş boyu ve başlık 1/cos ile büyütüldü.

**Üretim süreçleri.** Kompozit parçalar dişi kalıpta OOA prepreg ile fırında üretilir. Metal bağlantılar ISO 2768-mK'ya göre CNC frezelenir (7075-T651). S-kanal CFRP'den yapılır ve sırt kapağına yapıştırılır. Yapıştırmalarda EA 9394 kullanılır. CFRP'de yalnız Ti/A286 bağlantı elemanı kullanılır. Kapaklar SK2600 çeyrek dönüşlü bağlantı elemanıyla 100–150 mm aralıkla, kaporta ise SK4002 ile 60–90 mm aralıkla tutturulur.

## 4. Taşıma ve bakım erişimi

- **Taşıma ayrımı:** 2 kanat yarısı (3,35 m, kiriş dili dahil), 2 V panel (1,00 × 0,64 m), pervane. Gövde tek parçadır (4,00 m); karın yüzgeci ve burun bacağı üzerinde kalır. Ana takım yayı 2 kelepçeden sökülürse gövde sandığı yaklaşık 4,1 × 0,6 × 0,8 m olur (tahmin). Kanatta yakıt yoktur; kanat kökünde tek elektrik konnektörü vardır.
- **Bakım erişimi:**
  - Aviyonik ve batarya: burun üst kapağı.
  - EO/IR tareti: çene halkasından aşağı doğru sökülür; E180 büyüme zarfı (Ø0,18 × 0,23 m) yuvaya sığar.
  - Paraşüt: köşe güvertesindeki sırt kapağı, piroteknik emniyet pimiyle. PDU paraşüt kabının altındadır.
  - Faydalı yük: alt kapak (nadir pencereli). Sırt güvertesindeki ekipmana üst kapaktan erişilir.
  - Yakıt torbaları: iki hücrenin altındaki kapaklardan takılır ve çıkarılır (SK4002). Dolum ve seviye sondası yan paneldedir.
  - Motor: kaporta üst ve alt yarıları. Sırt kamburu kapağı S-kanal ve aşağı akışlı plenumla birlikte kalkar; bujiler, CHT/EGT ve SG750 açıkta kalır.
  - Servolar yüzey altı kapaklarındadır. Tekerlek kaportaları sökülebilir.

## 5. Riskler ve açık konular (dürüst değerlendirme)

1. Köşe girdabı sürüklemesi (gövdeye +%5), S-kanal geri kazanımı (0,88), süpürme düzeltmeleri ve taret izi tahmindir. Köşeli ön gövde rüzgâr tünelinde ya da CFD'de denenmelidir. Bakılacaklar: yanal kayma, köşe girdabının V paneller, yüzgeç ve itici pervaneyle etkileşimi.
2. Taşıyıcı çizgi süpürmeli yüklemeyi modellemez. c/4 süpürmesi 10,4° ile aero.py'nin 10° geçerlilik sınırındadır. Uç perdövites payı (burulma 3°, başlangıç η 0,38) ve süpürme kazancı VLM/CFD ile doğrulanmalıdır.
3. Çırpınma, süpürmeli kanatta eğilme–burulma bağlaşımı ve kanatçık ters etkisi analiz edilmedi. CS-LUAS.629'a göre 1,2 VD = 68,4 m/s'ye kadar temizlik gösterilmelidir.
4. Deniz seviyesinde bekleme devri 3625 dev/dk'dır; Limbach'ın 4000–6000 dev/dk bandının altındadır (3000 m'de 4186 dev/dk, bandın içinde). Tırmanma 4,24 m/s ile 4,9 m/s hedefinin altındadır (pervane sınırlı). 31x12 3B seçeneği 4,97 m/s verir, bedeli 0,48 h'dir. Kısıt diyagramında tasarım noktası tırmanma eğrisinin üstündedir. Bunun nedeni, diyagramın tam şaft gücünü kullanmasıdır: itici montaj kaybı, jeneratör yükü ve soğutma sürüklemesi orada yoktur.
5. Boş kütle oranı 0,624 ile benzerlerin üst sınırını (0,62) aşar. Büyüme payı olmadan 0,594'tür. Boş kütle %5 artarsa dayanım 11,2 h'e düşer.
6. Çene tareti AM'nin yaklaşık 1,9 m önündedir. Taret kütlesi AM'yi doğrudan oynatır: E180 ile SM 0,18'e çıkar, trimli CLmax 1,295'e iner. Araştırma yükü bölmede buna göre yerleştirilmelidir.
7. Arka gövde 0,48 m'den 0,32 m kaporta dudağına 0,10 m içinde kapanır (ortalama 39°). Bu kapanış itici pervanenin emişine ve halka soğutma çıkışına dayanır; CFD ile doğrulanmalıdır. 80 mm göbek ara parçası için Limbach onayı gerekir. İç payları küçüktür: plenum 2 mm, emme kutusu 5 mm, silindir dilimi 8 mm, paraşüt 8 mm. Limbach çizimleriyle doğrulanmalıdır.
8. İptal inişinde MTOM'da yerde koşu 222 m'dir; 200 m kuralını aşar. Önlem: kanatçıklar yukarı (kaldırma azaltma). Kalkış düzdür: kuyruk, yüksek itki hattına karşı burnu ancak 29,2 m/s'de kaldırabilir. Otopilot burun tekerleğini yerde tutmalıdır.
9. UCAV görünümü tescil, işaretleme ve ihracat değerlendirmesinde açıklama gerektirebilir. Uçakta askı noktası, pilon ya da bırakma mekanizması yoktur. Karın yüzgeci yalnız pervane koruması ve yön kararlılığı içindir.

## 6. Sonuç

Konsept uygulanabilir. 13,2 h dayanımla taban gereksinim olan 10 h'i %32 payla aşar. Aynı görev için MTOW 134,7 kg yeterlidir. Kimlik özelliklerinin doğrudan bedeli küçüktür: sade referansa göre −0,37 h ve 10 h görev için +1,2 kg. Dayanım konseptiyle fark daha büyüktür: −1,55 h ve 10 h görev için +4,0 kg. Bu farkın başlıca kaynakları şunlardır:
- İnce sivri burun ve çene tareti donanımı arkaya iter. Kanat AM'si x 2,47 m'ye kayar (Dayanım konseptinde 2,24 m). Kuyruk kolu 1,27 m'ye kısalır (1,36 m), V-kuyruk 0,99 m²'ye büyür (0,84 m²).
- Köşeli gövde daha geniştir: 0,50 m (0,44 m).
- En ön AM'de trimli CLmax daha düşüktür. Bu nedenle stall köşesi W/S 443 Pa'da kalır ve açıklık sınırına yalnız AR 14 sığar.

İçeri katlanır takım reddedildi: sürüklemeyi azaltır ama 3,4 kg'lık kütle cezası daha ağır basar. Açık konular CFD/rüzgâr tüneli, VLM, Limbach verisi ve uçuş testiyle kapatılmalıdır: köşeli ön gövde, S-kanal, süpürmeli kanadın perdövitesi ve çırpınma.

Ek dosyalar: `sketch.png` (aynı ölçekte üç görünüş, perspektif, iç yerleşim), `constraint.png` (sizinglib kısıt diyagramı), `concept.yaml` (bütün girdiler, sonuçlar, takaslar ve kimlik bedeli), `polars/` (bu konseptin tripli polar önbelleği).
