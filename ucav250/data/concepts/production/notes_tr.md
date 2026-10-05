# YK-250 — “Üretim” konsepti: tasarım notları

**Amaç:** en düşük üretim maliyeti, risk ve bakım yükü; UCAV şekil dili korunur. **Kapsam:** sivil EO/IR gözetleme ve araştırma platformu. Silah, mühimmat, askı noktası, pilon ya da bırakma mekanizması yoktur.
**Hesap:** `PYTHONPATH=. python3 ucav250/data/concepts/production/calc.py` (yaklaşık 11 dk); girdilerin kaynakları ve tüm sonuçlar `concept.yaml` dosyasındadır. Ortak denklemler (motor/BSFC, pervane tablosu, tripli polarlar × 1,15, taşıyıcı çizgi, görev, tavan, darbe, kısıt diyagramı) incelenmiş “Dayanım” zincirinden değiştirilmeden alınır; konseptler aynı denklemlerle karşılaştırılır.

## 1. Konfigürasyon ve gerekçe

| Karar | Seçim | Gerekçe (takas sonuçları) |
|---|---|---|
| İtki | L 275 EF **itici**, Mejzlik 32x18 2B, 80 mm göbek ara parçası; pervane iki kuyruk kirişinin arasında | Yağlı egzoz ve pervane diski EO/IR taretin arkasında kalır. Düzeni bilinen 13 benzer İHA'nın 11'i iticidir. Çekici düzen tahmini 12,89 h verir (itici 13,19 h). |
| Kuyruk | **Çift CFRP boru kuyruk kirişi + H-kuyruk**: pervane izinde sabit veterli yatay kuyruk (iki parçalı irtifa dümeni); 35° oklu, 12° dışa yatık iki özdeş dikey kuyruk | Aynı V_H ve Cnβ ile, kendi perdövites köşesinde boyutlanan tek gövde + V-kuyruk (en iyi gövde uzatması 0,6 m): 12,34 h, kalkış koşusu 227 m (flaplı: 212 m, 11,45 h). H-kuyrukta pervane izi dönüş hızını 19,9 m/s'ye indirir (V: 30,3 m/s): flapsız 163 m. En uzun parça 2,67 m (V: 3,48 m). Bedeli: 2 servo ve 1 kalıp fazla. |
| Kanat | Omuz kanat, **üç parça**: sabit veterli orta kanat (y = ±1,00 m, veter 0,581 m) ve doğrusal daralan dış paneller (λ 0,60, uç veteri 0,349 m, 3° burulma). NLF(1)-0416 her yerde %16. AR 12, S 3,04 m², b 6,04 m | Orta kanat: tek kaburga ailesi, silindirik kabuk; dış panel kabukları neredeyse konik (prepreg kesiksiz serilir). Burulmasız λ 0,5 / 0,6'da perdövites kanatçıkta başlar (η 0,68 / 0,59); λ 0,6 + 3° ile kökten başlar, kanatçıkta cl/cl_max ≤ 0,94. Orta kanat yarı açıklığı 0,56–1,20 m: dayanım farkı < 0,02 h; 1,00 m'de üç parça eşit boy (2,00 / 2,02 m). AR 10…14: 12,50/12,87/13,19/13,42/13,65 h; AR 13'te dış panel orta kanattan uzun (2,14 m). AR 10 kanatta yalnız 0,3 kg kazandırır. |
| W/S | 467,6 Pa (47,7 kg/m²): flapsız, trimli VS = 24 m/s köşesi | 433 Pa'da dayanım 12,69 h'e düşer. |
| Flap | Yok | Orta kanatta 15° flap (S_f/S 0,24; kuyruk kirişi eyerleri flabı böler) kalkış koşusunu 163 m'den 156 m'ye indirir, ama dayanımı 12,34 h'e düşürür ve 2 servo ekler. |
| Kuyruk kolu | Pervane düzleminden yatay kuyruk hücum kenarına 1,05 m (l_h 2,00 m) | Taranan aralık 0,45–1,25 m, dayanım 12,31–13,30 h. 1,25 m'de kuyruk kirişinin ilk modu 10,9 Hz'e düşer (ölçüt ≥ 12 Hz). 1,05 m'de kuyruk kirişi boyu (2,29 m) gövde boyunun (2,67 m) altında kalır. |
| İniş takımı | Sabit üç tekerlek: GFRP yay (bacakları kaportalı), TOST 200x50, yönlendirilebilir burun bacağı; **tekerlek kaportası yok** | Kaportalar 0,70 kg, kazanç yalnız 0,05 h (eşik 0,1 h): kısmi yükte BSFC güçle düştüğü için küçük sürükleme artışları ucuzdur. İçeri katlanan takım: 12,83 h. |
| Yapı | **Alüminyum şasi + cıvatalı kompozit kabuk** | CFRP şasi 2,1 kg hafif, 14,15 h. Alüminyum: A-tabanlı izin değerleri (en yüklü bağlantılarda kompozit nitelendirmesi yok), kalıpsız düz desen parçalar, görünür hasar, perçinle saha onarımı. Bedeli 0,96 h. |
| Yerleşim | Faydalı yük bölmesi AM'de; önünde ve arkasında eşit hacimli, aynı parça numaralı iki yakıt torbası | Beş yükleme durumunda AM 1,843–1,845 m, SM 0,100–0,104. |

## 2. Ana sayılar

| | |
|---|---|
| Kütle | MTOM 145,0 kg (SHGM M2 sınıfı 150 kg'ın altı; proje sınırı 149,9 kg). Boş kütle 88,6 kg: oranı 0,611, %5 büyüme payı hariç 0,582 (benzerlerden önerilen aralık 0,55–0,62). Yakıt 36,4 kg, faydalı yük 20,0 kg. |
| Kütle dağılımı (kg) | sabit donanım 32,4 · kanat 19,0 · şasi 9,9 · iniş takımı 9,9 · gövde kabuğu 6,2 · kuyruk kirişleri 3,6 · yatay kuyruk 1,9 · dikey kuyruklar 1,5 · büyüme payı 4,2 |
| Geometri | Toplam boy 4,11 m; gövde 2,67 × 0,46 m; yükseklik 1,12 m. Kuyruk kirişi aralığı 1,12 m; kuyruk kirişleri Ø80 × 1,5 mm × 2,29 m. Yatay kuyruk 0,352 m² (1,12 × 0,314 m), dikey kuyruklar 2 × 0,142 m² (yükseklik 0,48 m). |
| Aerodinamik | CD0 0,0375 · e 0,856 · (L/D)maks 14,7 · temiz trimli CLmax 1,33 · VS 23,9 m/s |
| Bekleme (3000 m) | 28,7 m/s EAS / 33,3 m/s TAS · yakıt 2,65 kg/h · 4306 dev/dk · BSFC 563 g/kWh |
| Görev | Dayanım **13,19 h**: 2 × 100 km intikal, 3000 m'de 10,94 h bekleme, uçuş süresinin %10'u kadar yedek. Feribot menzili 1453 km. 10 h görev 133,5 kg MTOM ile yapılabilir. |
| Duyarlılık | BSFC −%12 / +%12 → 15,0 / 11,8 h · CD0 +%10 → 12,6 h · boş kütle +%5 → 11,3 h |
| Hız ve tırmanma | Vmaks 54,5 m/s (DS) · en uzun menzil hızı 36,1 m/s (3000 m) · tırmanma 4,07 m/s (DS), 2,16 m/s (3000 m) · servis tavanı 6226 m |
| Pist (ISA) | Kalkış koşusu 163 m (1500 m irtifada 243 m), 15 m'ye 293 m · iniş koşusu 169 m |
| Kararlılık | SM 0,100–0,104 · V_H 0,450 · V_V 0,031 · Cnβ 0,057 1/rad |
| Yer geometrisi | İz 0,87 m · devrilme açısı 54,8° · burun yükü %10,6–10,7 · pervane–yer 234 mm · pervane–kuyruk kirişi 114 mm · statik uç Mach 0,72 |
| Yükler | Darbe limit yük katsayısı 4,90 (4500 m), nihai 7,35. Nihai eğilme momenti kanat kökünde 6956 N·m, dış panel bağlantısında 2774 N·m. |

## 3. Yapı: şasi + kabuk

**Şasi (birincil yapı; bütün tekil yükler).** Alüminyum sac ve ekstrüzyon: CAD düz deseninden su jeti/lazer kesim, abkant büküm, fikstürde perçin; kalıp yok.
- 2 alt (6061-T6 L 25×25×1,6) ve 2 üst (L 20×20×1,6) boyuna profil, x 0,50 m'den yangın perdesine (x 2,33 m).
- 3 ağır perde (6061-T6 1,0 mm, boşaltılmış): kanat ana kirişi (x 1,87 m), arka kiriş (x 2,07 m), yangın perdesi (CS-VLA 1191: 0,38 mm paslanmaz kaplı); 7 hafif Z-çerçeve (2024-T3 0,6 mm); kanat bölgesinde burulma kutusunu kapatan 2024-T3 0,6 mm yan levhalar; yakıt bölmelerinde 0,5 mm 6061 astar.
- İşlenmiş 7075-T651 sert noktalar (4 kanat bağlantısı, ana takım yay eyeri, burun takımı muylusu), paraşüt bağlantı noktası, 4130 borudan TIG kaynaklı motor kafesi (AMS 6457 tel).
- Kontrol (n_nihai 7,35): gövde eğilme momenti 1763 N·m, boyuna profillerde 35 MPa; izin 187 MPa (0,30 m çerçeve aralığında kolon burkulması). Şasiyi asgari et kalınlığı belirler.

**Yük yolları.**
- *Kanat:* orta kanadın UD CFRP başlıklı, ±45° gövdeli kirişi gövde üstünde kesintisizdir; eğilme orta kanatta kapanır. Orta kanat 4 cıvatayla ağır perdelerdeki 7075 bağlantılara oturur; gövdeye kesme ve burulma geçer.
- *Dış panel (y = ±1,00 m):* kiriş dili orta kanat kiriş kutusuna girer; üst ve alt başlık hizasındaki iki Ti-6Al-4V pim (16 mm, çift kesme) eğilme kuvvet çiftini, arka pim sürükleme ve burulmayı taşır. Bağlantı momenti kök momentinin %40'ı; 1,15 bağlantı ve 1,5 sık sökme faktörüyle MS 4,9.
- *Kuyruk:* kuyruk kirişleri y = ±0,56 m'de kanadın ana ve arka kirişi üzerindeki işlenmiş eyerlere kelepçelenir (her biri 2 cıvata + 1 konnektör). Kuyruk kirişi ucundaki bağlantı parçası yatay ve dikey kuyruğun kirişlerini ve tampon kızağını taşır. Kuyruk kirişinde nihai yükler 1217 N·m eğilme, 104 N·m burulma; et kalınlığını 1,5 mm taşıma/hasar asgarisi belirler.
- *İniş takımı:* GFRP yay → 7075 eyer (x 1,98 m, iki kanat perdesi arasında) → alt boyuna profiller → ağır perdeler; burun bacağı x 0,66 m'deki muyluda.
- *Motor:* 4 × M8 + elastomer izolatör → 4130 kafes → yangın perdesi → boyuna profiller (acil iniş 15 g ileri). *Paraşüt:* GRS 4/240 açılma şoku 13,1 kN, bağlantı noktasından çerçevelere ve boyuna profillere.

**Kabuk (ikincil; cıvatalı, yük taşımaz).** Burun konisi/radom, üst kapaklar (aviyonik, paraşüt, yakıt dolumu), yan paneller, alt faydalı yük kapağı, eyer kaplaması, üst/alt kaporta, kuyruk kirişi burunları. CFRP sandviç 0,4 mm PW / 5 mm ROHACELL 51 WF / 0,4 mm; radom ve anten pencereleri aynı reçineli 7781 cam prepreg. Kaplamalarda SK2600 çeyrek dönüş elemanı 100–150 mm, kaporta ve kapaklarda SK4002 60–90 mm aralıkla. Panel sökmek hiçbir yük yolunu değiştirmez.

**Kanat ve kuyruk: tek kompozit süreç hattı.** MTM45-1/AS4 OOA prepreg (UD + PW), vakum torbası, fırında 121 °C kür ve serbest 177 °C son kür; ROHACELL çekirdekli kabuklar; hücum kenarının ilk %8–10'unda erozyon bandı. Yatay kuyruk (simetrik profil ve plan) tek kalıptan iki kabuk verir; dikey kuyruklar özdeş; kumanda yüzeyleri ana yüzeyin kalıbında basılıp kesilir; kuyruk kirişleri satın alınan sarma karbon borulardır. Al–CFRP temasında 7781 cam izolasyon katı, CFRP'de yalnız Ti-6Al-4V/A286 bağlantı elemanı.

## 4. Taşıma, montaj, üretim göstergeleri

- **Parçalar:** gövde 2,67 m; orta kanat 2,00 × 0,58 m; 2 dış panel 2,02 m; fabrikada ayarlı kuyruk ünitesi (kuyruk kirişleri + yatay ve dikey kuyruklar) 2,29 × 1,32 m; pervane.
- **Saha montajı:** orta kanat 4 cıvata; dış panel başına dil + 2 pim + sürükleme pimi + 1 konnektör; kuyruk ünitesi 2 × 2 cıvata + 2 konnektör; pervane.
- **16 kalıp:** orta kanat 2, dış paneller 4, yatay kuyruk 1, dikey kuyruklar 2, kuyruk kirişi burnu 1, burun konisi/radom 1, gövde orta/geçiş bölümü üst/alt 2 (yan paneller ve kapaklar bunlardan kesilir), kaporta 2, eyer kaplaması 1. Şasi kalıpsız; satın alınan yapı parçası 2 karbon boru; 8 × Volz DA 26 + 1 kaporta flabı servosu.

## 5. Bakım erişimi ve LRU'lar

- Aviyonik tepsisi burun üst kapağından, batarya/PDU yan kapaktan; EO/IR taret burun altından 4 cıvata + 1 konnektörle sökülür.
- Paraşüt (GRS 4/240 SOFT, roketle açılır) üst kapaktan tek parça çıkar; roket 6 yılda bir değişir.
- Sistem bölmesi x 0,96–1,38 m (yaklaşık 0,42 m boş): jeneratör denetleyicisi, telsizler, büyüme payı.
- Faydalı yük alt kapaktan raylı paletle takılır; yakıt torbaları (bölme boyları 0,25 / 0,24 m) yan panellerden değişir.
- Motor hızlı değişim modülü (motor + kafes + egzoz + ECU + SG750 marş/jeneratör) 4 cıvata ve hızlı bağlantılarla sökülür. Kuyruk kökü olmadığından motor bölmesi üstten ve iki yandan açılır; pervaneyi yanlardan kuyruk kirişleri korur.
- Servolar kapak altında ve konnektörlü; tekerlek kaportası olmadığından lastik ve fren aletsiz kontrol edilir.

## 6. Riskler ve açık konular

1. **Yatay kuyruk pervane izinde:** devir başına 2 pala geçişi basınç yükü, akustik yorulma ve güçle değişen trim getirir. Titreşim taraması, yorulma testi ve otopilotta güç–trim telafisi gerekir. Dönüş hızı aktüatör disk tahminidir; iz katkısı olmadan kalkış koşusu 205 m (hedef 200 m). Taksi testiyle doğrulanmalı.
2. **Kuyruk kirişi ve kuyruk çırpınması:** kuyruk kirişinin ilk eğilme modu 12,5 Hz. FAA Report 45 kirişli kuyruklarda geçerli değil; CS-LUAS.629'a göre 1,2 VD = 68,4 m/s'ye kadar GVT ve analiz gerekir.
3. **Tırmanma 4,07 m/s (hedef 4,9 m/s):** sınırlayan 32x18 2B pervane. Aynı göbekte 31x12 3B: 4,81 m/s, dayanım −0,44 h.
4. **Deniz seviyesinde bekleme 3727 dev/dk:** Limbach'ın 4000–6000 dev/dk düzenli uçuş bandının altında (3000 m'de 4306 dev/dk). Limbach onayı ya da en az 1500 m'de bekleme gerekir.
5. **Alüminyum şasi:** 2,1 kg ve 0,96 h bedel; Al–CFRP galvanik korozyonu için süreç disiplini ve muayene planı.
6. **MTOM'da iptal inişi 214 m** (hedef 200 m, pist 300 m): flap yok; kanatçıklar yukarı alınarak kaldırma azaltılır ve bir işletme sınırı konur.
7. **Arka gövde kapanışı** (0,45 → 0,30 m kaporta dudağı) pervane emişine dayanır; CFD gerekir. 80 mm göbek ara parçası Limbach onayı ister.
8. **Görünüm:** çift kuyruk kirişli düzen tek gövdeli MALE/UCAV siluetinden uzaklaşır; şekil dili keskin kenarlı (chine) gövde, sırt kamburu, sivri kuyruk kirişi burunları ve oklu, dışa yatık dikey kuyruklarla korunur.
9. **Kütleler kavramsal tahmin** (%5 büyüme payı dahil); boş kütle oranı önerilen aralığın üst bölgesinde. Boş kütle +%5'te dayanım 11,3 h; 10 h yine karşılanır.

## 7. Sonuç

Konsept **uygulanabilir**: 145 kg MTOM ile 13,2 h dayanım (10 h gereksiniminin %32 üstü), 163 m kalkış ve 169 m iniş koşusu, 6226 m servis tavanı. Üretim kazançları: kalıpsız alüminyum şasi, 16 kalıp, tek kompozit süreç hattı, satın alınan kuyruk kirişleri, tek kaburga ailesi, en çok 2,7 m'lik parçalar. Bakım kazançları: yük taşımayan cıvatalı kabuk ve LRU yerleşimi. Bedelleri: “Dayanım” konseptine (AR 15, CFRP şasi; 14,77 h) göre 1,6 h daha az dayanım (alüminyum şasinin payı 0,96 h), pervane izindeki yatay kuyruğun yorulma ve çırpınma doğrulaması, MALE siluetinden uzak görünüm.

**Dosyalar:** `sketch.png` (aynı ölçekli üç görünüş ve iç yerleşim), `constraint.png` (kısıt diyagramı), `concept.yaml` (girdiler, sonuçlar, takaslar). Kısıt diyagramındaki genel kalkış eğrisi tasarım noktasında 10,5 W/N ister; pervanenin tam gazda tırmanmada emdiği güç 10,0 W/N'dir. Ayrıntılı kalkış hesabı (pervane tablosu ve iz destekli dönüş) 163 m verir.
