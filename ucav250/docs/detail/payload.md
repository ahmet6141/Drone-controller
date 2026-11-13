# YK-250 HANÇER — Faydalı yük (PAYLOAD) ayrıntılı tasarımı

Üretici modül: `ucav250/design/payload.py` (`register(reg, spec)`), test: `tests/test_ucav250_payload.py`.
Sözleşme: `ucav250/ARCHITECTURE.md`; tek doğruluk kaynağı: `ucav250/spec.yaml` (`layout.part_numbers.payload`
820–869, `payload.turret` (top çapı, yükseklik, gövde çapı, strok, top merkezleri, kapak kalınlığı / açıklığı,
açıklık halkası, E180 büyüme zarfı), `layout.chassis.turret_elevator` (ray hatları, vida ekseni, üst bağlantı,
strok), `structures.sizing.turret` (ray ucu başına vida sayısı), `layout.mechanisms` (eklemler `turret_elevator`,
`turret_door_R/L`, dizi `turret_extension`, `door_outlines`), `layout.stations` (FS1110 / FS1330 yüzleri,
C-TDOOR-SHAFT kesiği), `layout.chassis.members` (M-TURRETWALL / M-TURRETROOF), `layout.shell.panels` (P-TURRETRING),
`layout.systems.equipment` (EQ-TURRET, EQ-ELEVATOR, EQ-TDOORACT kutuları), `layout.keep_outs` (KO-TURRET-FOV),
`mass`, `materials`, `processes`, `assembly`) ve `ucav250/data/research/components.yaml` (Trillium HD59-LLVV, Volz
DA 22). Konumlar başka bir modülün geometrisinden okunmaz; tek istisna, layout kimliği olmayan şasi ray oturma
köşebentleridir (CH-125 / CH-126): kayıtta adlarındaki ray kimliğiyle (RAIL-FR / RAIL-AL) bulunurlar. Alt kaplama
iç yüzü, çerçeve başlıkları ve halka iç yüzü OML'den (`oml.fuselage_from_spec`) ve layout kalınlıklarından türetilir.

Kapsam yalnızca sivil EO/IR gözetleme taretidir (sınır, boru hattı, arama-kurtarma, orman yangını). Modülde silah,
mühimmat, dış yük askı noktası ya da bırakma düzeneği yoktur ve öngörülmemiştir.

## 1. Özet

| Büyüklük | Değer |
|---|---|
| Parça kimliği (820–849 kullanıldı) | 38 (19 satın alma, 19 imalat); 7 çift sağ/sol ayna (kapak, iki kızak, iki kapak rayı, pinyon, DA 22 + braket) |
| Bağlantı elemanı (`joints.py`, geometri `hardware.py`) | 82: 34 × ISO 4762 M3, 26 × ISO 4762 M4, 22 × ISO 7380 M3 (A2-70); gömme insert, yüzer somun plakası, nyloc, ünite dişleri |
| Hareketli eklem | `turret_elevator` (prizmatik, 0,128 m, rest = içeride), `turret_door_R/L` (prizmatik, 32° eğik, 0,12 m); bağlı dizi `turret_extension` (13 durum) |
| Tasarım kuralı denetimleri | `python3 -m ucav250.analysis.checks --modules chassis,shell,payload --focus YK250-PL --no-write`: PL parçalarının kendi aralarında ve şasi / kabuk parçalarıyla 0 ihlal; şasi / kabuk **bağlantı elemanlarıyla** 6 statik + 14 süpürme ihlali (§10, §11-1) |
| Kütle | HD59 + yalıtımlı bağlantı 1,736 kg (`payload.turret_mass_kg` 1,75); mekanizma 1,870 kg (kalem `turret_lift_mechanism_doors` 1,786 temel / 1,875 büyüme dahil) — §9 |
| Üçgen sayısı (PL parçaları) | ≈ 52 000 |
| Montaj adımı | 24 (`spec.assembly`: taret asansörü, kayar kapaklar ve HD59) |

## 2. Taret: Trillium HD59-LLVV ve E180 büyüme zarfı

- **HD59 (PL-820, satın alma).** components.yaml `trillium_hd59_llvv` veri sayfası: top Ø147 mm, yükseklik 202 mm,
  1,55 kg, 24 V, Ethernet video / kontrol, IP65, eğim −80…+35°. Model: top (Ø147, 96 dilimli küre; ekvator köşeleri
  halka açıklığının köşeleriyle çakışır), pan / çatal gövdesi (`payload.turret.stem_diameter` Ø90) ve üst bağlantı
  tabanı (Ø120 × 12 mm). Üretici bağlantı ICD'si yayımlanmamıştır: taban çapı ve 4 × M4 cıvata dairesi (r 42 mm)
  **tahmindir** (§11). Top merkezi içeride `ball_center_retracted` (z −0,0986), dışarıda `ball_center_extended`
  (z −0,2266); yüksekliği layout ile birebir.
- **E180 büyüme zarfı** (`payload.turret.growth_envelope` Ø180 × 230 mm, `sizing.turret_checks` tanımı: aynı top
  merkezinde Ø180 küre + üst uca kadar Ø180 silindir). `payload.e180_envelope()` ile kurulur. Testte zarf, dizinin her
  durumunda kapaklara, raylara, kızaklara, tahriklere, braketlere ve kablo parçalarına girmez; kapalı kapaklara
  uzaklık 5,1 mm (layout `growth_margin_to_door_m` 17,4 mm, kapak iç yüzü yerine kaplama + 4 mm referansıyla).
  Kapak raylarının iç uçları tam açılımdaki zarfın 0,5 mm dışında kalacak şekilde oyulmuştur. E180'e geçişte
  asansörle hareket eden HD59 takımı (adaptör, izolatörler, taşıyıcı), dudak contası ve 157 mm halka (SH-363 →
  0,19 m açıklıklı SH-364) değiştirilir.

## 3. Titreşim yalıtımlı bağlantı

- **Adaptör plakası (PL-821).** 7075-T651, 4 mm, 124 × 124 mm, Ø60 merkez deliği; HD59 tabanına 4 × ISO 4762 M4
  (tabandaki dişlere, vida sabitleyici), izolatörlere alttan 4 × M4.
- **Elastomer izolatörler (PL-843…846, satın alma, tahmin).** Ø20 × 16 mm silindirik, iki ucu M4 dişi, ~40 Shore A
  (HD59 kütlesine ayarlı), (±48, ±48) mm köşelerde. Adaptör + izolatörler 0,186 kg ≤ `payload.sources.turret_mass_kg`
  bağlantı payı 0,20 kg.

## 4. Asansör

- **Taşıyıcı (PL-822).** 7075-T651, 4 mm: Ø100 harness deliği çevresinde 10 mm halka, her izolatör vidasına Ø22 ped
  (M4 kenar mesafesi 8,8 mm), iki köşe tırnağına (4 mm, blok başına 2 × M3) ve vida somununa 20 mm genişlikte
  kollar, kablo tırnağı (oluk yarığına girer). Tırnak vidaları blok merkezine göre −8,5 / +6,5 mm'de (baş taşıyıcı plakadan uzak).
- **Profil raylar (PL-828 RAIL-FR, PL-829 RAIL-AL, satın alma).** MGN12 sınıfı, L 226 mm, layout hattı
  (z −0,131…0,095), şasi köşe oturmasına (CH-125 / CH-126) ray ucu başına `structures.sizing.turret` 4 × M3 ile,
  oturmanın arkasına perçinli yüzer somun plakalarına. Ray başına iki blok (PL-839…842, MGN12C sınıfı, 0,034 kg).
- **Bilyalı vida (PL-825), somun (PL-823), tahrik (PL-824), yataklar.** Ø8 × 2 vida ön-sol köşede (layout
  `screw_axis`), somun flanşı taşıyıcıya 4 × M3 + nyloc. Tahrik (PL-824, layout EQ-ELEVATOR parçası): BLDC Ø22 motor
  + sonsuz vida kademesi, yay frenli, mutlak kodlayıcı ve 2 sınır anahtarı (tahmin 0,20 kg), tavan altındaki 6061-T6
  yatak yuvasına (PL-827; 626 sınıfı açısal temaslı yatak çifti yuvası + dişli kutusu vida kulakları) 2 × M3 ile;
  yuva tavan sandviçindeki 4 gömme M4 inserte (`top_anchor`). Alt muylu braketi (PL-832, 6061-T6) FS1110 ağına
  2 × M4 gömme insertle; PTFE kaplı burç.
- **Sert durdurucular.** Elektriksel sınırların 0,5 mm ötesinde: içeride somunun PU tamponu dişli kutusu alt yüzünün
  0,5 mm altında; tam açılımda taşıyıcı plaka alt muylu göbeğinin 0,5 mm üstünde (braket ağı ve flanşı göbek
  üstünün 9 mm altında: somun flanşı nyloc'ları çarpmaz). Testte her iki boşluk geometriden ölçülür.
- **Eklem.** `turret_elevator`: prizmatik, eksen −z, 0…0,128 m, `expr 0.1280*clamp((turret-0.2)/0.8,0,1)`;
  hareket eden parçalar HD59, adaptör, izolatörler, taşıyıcı, bloklar, somun, HD59 arayüz kablosu.

## 5. Kayar kapaklar ve tahrikleri

- **Kapaklar (PL-826-R/L).** OOA prepreg 10 kat PW (2,0 mm). Dış yüz, kapalıyken halka iç yüzünün 1,0 mm üstünde,
  0,12 m kayma boyunca (eksen (0, 0,848, 0,530)) halka / alt kaplama / çerçeve başlıkları üzerinde 1,0 mm açıklıkla
  kalacak şekilde, bütün yol boyunca alınan zarftan kalıplanmıştır. Plan: x 1,1334–1,3066 (çerçeve yüzlerinden
  20 mm), y 0,0005–0,0944 (layout `outline_closed` 0,105 m yay uzunluğu); merkez hattında 1 mm aralık. Kapak kalınlığı
  layout `door_thickness` 4 mm zarfının (kapak + kızak) içindedir.
- **Kapak rayları (PL-837 ön / FS1110, PL-838 arka / FS1330).** 6061-T6, 6 × 6 mm çubuk, kapak tarafı yüzünde 2,6 × 3 mm
  kılavuz oluğu; çubuk, en alçak kayan noktası (oluktaki dil) bütün yol boyunca başlık / kaplama üstünde 0,5 mm kalacak
  en alçak hatta. Ayaklı kulaklar (12 mm, ISO 7380 M3 çubuğun üstünden geçer, baş 1,8 mm havşada, alt / üst kenar
  mesafesi 6,5 mm) çerçeve ağındaki gömme M3 insertlere; aralık ≤ 60 mm (9 g + emme yükünde sehim < 0,7 mm), duvar
  köşesi / asansör ray oturması altındaki geçiş bandında (|y| 0,075–0,13) ve DA 22 braket insertlerinin 16 mm
  yakınında kulak yoktur. İç uçlar E180 zarfı için oyuktur (§2).
- **Kızaklar (PL-834 ön, PL-836 arka + kremayer).** Kapağın üst yüzüne EA 9394 ile yapıştırılan yükseltici + 2 mm ped,
  PTFE kaplı 2,2 mm dil oluğa girer. Kızak hiçbir yerde raydan yüksek değildir: kapak takımı asansör ray oturmasının
  (CH-125 / CH-126, z −0,131) ve duvarın serbest alt kenarının altından geçer (en küçük boşluk ön kızak–CH-125 2,2 mm,
  arka kızak–duvar 2,0 mm). Arka kızak modül 0,5 kremayeri taşır (diş zarfı = hatve çizgisi − 0,2 mm, kapak kenarının
  dışına duvar altına uzanır).
- **Pinyon ve mil (PL-835).** 7075-T651 sert eloksal, modül 0,5, 78 diş (hatve Ø39), 2 mm gövde + 3,5 mm diş çemberi,
  merkezi C-TDOOR-SHAFT kesiğinin merkezinde; mil kesikten geçip DA 22 çıkışına kamalı. DA 22 1,8 N m nominalde diş
  kuvveti ≈ 92 N, 6 mm genişlikte eğilme ≈ 100 MPa (7075 için yeterli; yorulma doğrulaması açık konu).
- **Volz DA 22 (PL-830-R/L, EQ-TDOORACT).** components.yaml `volz_da22_28v`: 41,5 × 65,9 × 22 mm, 0,132 kg; layout
  kutusunda birebir. 6061-T6 braket (PL-831): 3 mm çerçeve flanşı (FS1330 arka yüzü ile ünite gövdesi arası,
  4 × M4 gömme insert; alt kenar kaplamaya paralel, arka başlığın 1 mm üstünde kırpılmış) + 2,5 mm yan flanş
  (2 × M3 ünite dişlerine) ve mil burcu. Erişim P-TDOORACC kapağından.
- **Eklemler ve dizi.** `turret_door_R/L` layout'tan (köken (1,22, 0, −0,208)); `turret_extension`: 0–0,2 kapaklar
  açılır, 0,2–1,0 asansör iner; kapama ters sırada. Testte 13 durum + ara noktalar: hareketli PL parçaları şasi,
  kabuk ve sabit PL parçalarıyla kesişmez; kapak–taret ≥ 16,9 mm (kural 5 mm), kapalıyken 21,6 mm
  (`door_clearance` 10 mm).

## 6. Sızdırmazlık

- **Dudak contası (PL-847, satın alma).** Kalıplanmış silikon, halka insertinin (SH-363) iç yüzüne RTV astarla
  yapıştırılmış, Ø160–172 mm; üst yüzü kapakların bütün yolu boyunca kapak dış yüzünün 0,3–0,5 mm altında (kapalıyken
  hafif temasla sızdırmaz kapanır, kayarken sürtmez).

## 7. Kablo servis döngüsü

- **HD59 arayüz kablosu (PL-833).** Ø8 mm, taret tepesindeki konnektörden taşıyıcı üstünden kablo tırnağına.
- **Kanal (PL-848).** SLS PA12, 40 × 12 mm C-kesit, 24 mm yarık; sağ bölme duvarına 4 × ISO 7380 M3 gömme insertle.
- **Döngü (PL-849).** Ekranlı yassı kablo 16 mm (24 V + Ethernet) — yuvarlanan U-dönüşü deforme olduğundan
  modellenmedi; sabit kol + 90° arka kabuk + duvardan geçen MIL-DTL-38999 seri III somunlu soket (duvarda Ø19 delik).

## 8. Malzeme ve üretim

| Parça | Malzeme / süreç | Kalınlık |
|---|---|---|
| adaptör, taşıyıcı, arka kızak + kremayer, pinyon | 7075-T651, CNC | 4 / 4 / 2,2 / 6 mm |
| yatak yuvası, alt muylu braketi, kapak rayları, ön kızak, DA 22 braketi | 6061-T6, CNC | 4 / 4 / 6 / 2,2 / 3 mm |
| kapaklar | CFRP PW MTM45/AS4, OOA vakum torbası | 2,0 mm |
| kanal | PA12 SLS | 1,5 mm |

Delikler ISO 273 orta; kenar mesafesi metal / polimerde ≥ 2 D, kompozitte ≥ 2,5 D (denetimle doğrulanır).

## 9. Kütle

| Kalem | Modelde (kg) |
|---|---|
| HD59 + adaptör + 4 izolatör (`payload.turret_mass_kg` 1,75) | 1,736 |
| profil raylar + bloklar + taşıyıcı | 0,294 + 0,136 + 0,138 |
| bilyalı vida + somun + tahrik + yatak yuvası + alt braket | 0,085 + 0,040 + 0,200 + 0,057 + 0,039 |
| kapaklar (2) | 0,114 |
| DA 22 (2) + braketler + pinyonlar + kızaklar + kapak rayları | 0,264 + 0,073 + 0,039 + 0,107 + 0,164 |
| conta, kanal, döngü, arayüz kablosu | 0,012 + 0,016 + 0,060 + 0,030 |
| **mekanizma toplamı** | **1,870** |
| **grup toplamı** | **3,606** |

Mekanizma, `mass.items.turret_lift_mechanism_doors` (sistem grubu) kaleminin büyüme dahil değeri 1,875 kg'ın 5 g
altındadır (temel 1,786 kg + `growth_allowance` %5; temelin 84 g üstü). Kalemin dökümüyle karşılaştırma: "kılavuz raylar / taşıyıcı 0,35" modelde
0,57 (MGN12 ray + blok yalnız 0,43); "kapak tahrikleri 0,304" modelde (DA 22 + pinyon + kremayerli kızak + kapak rayları
+ braketler) ≈ 0,64; buna karşılık "bölme astarı / çerçeve 0,40" ve "HD59 açıklık halkası 0,06" bu modülde yapılmaz
(bölme duvarları / tavanı şasinin, halka kabuğun parçasıdır). Büyüme payının %94'ü kullanılmıştır (§11).
Bağlantı elemanları (0,22 kg) donanım grubundadır.

**Bütçe grubu (IR-08):** taret HD59 + izolatörlü bağlantı `eo_ir_turret_hd59_mount` (faydalı yük), diğer bütün parçalar (asansör, kapaklar, tahrikler, conta, servis döngüsü) `turret_lift_mechanism_doors` (systems) kalemine yazılır (`Part.mass_item`); `analysis/mass.py` bu eşlemeyle toplar ve faydalı yüksüz durumlarda yalnız taret setini çıkarır.

## 10. Doğrulama

- Tasarım kuralı denetimleri (`chassis,shell,payload`, odak YK250-PL): mesh 0, açıklık 0 (taret–halka ≥ 5,0 mm,
  taret–duvar / çerçeve ≥ 6 mm, kapak–taret ≥ 5 mm), kalınlık 0, bağlantı elemanı 0, temas 0, bağlanma 0; statik 6 ve
  süpürme 14 ihlalin **tamamı** başka modüllerin bağlantı elemanlarıyladır (§11-1): SH-361-B1…B4 (alt kaplamanın FS1110
  başlığındaki yüzer somun plakaları, ön kapak rayı / kızağı içinde), SH-363-B5…B8 (halka yan alanındaki yüzer somun
  plakaları, kapak yolunda), CH-125 / CH-126-B3 / B4 (ray oturmasının M4 başları, MGN12 blok zarfında). PL
  parçalarının şasi / kabuk parçalarıyla ve kendi aralarında ihlali yoktur.
- `tests/test_ucav250_payload.py`: denetimler; tek kayıt; kimlik aralığı ve aynalar; layout sabit kimlikleri (EQ-TURRET,
  EQ-ELEVATOR, EQ-TDOORACT, raylar, kapaklar); parça sözleşmesi; silah ifadesi yokluğu; kapalı ağ ve ayna hacmi; taret
  zarfı ve layout kutuları; eklem / dizi layout uyumu (kapaklar tam açılmadan asansör oynamaz); **dizi taraması
  çarpışmasız** (13 durum + ara noktalar); **içeride gömme** (kapaklar halka açıklığını planda tamamen örter, OML'nin
  altına hiçbir PL parçası taşmaz, kapak–taret ≥ `door_clearance`); **dışarıda görüş alanı açık** (KO-TURRET-FOV −5°
  konisinin altına hiçbir PL / şasi parçası inmez, taret–halka ≥ 5 mm); E180 zarfı; sert durdurucular; bağlantı
  elemanları; kütle.
- Önizlemeler (`python3 -m ucav250.blender.build --modules chassis,shell,payload --previews --no-blend`) ve bölme yakın
  çekimleri (içeride, kapaklar yarı açık, dışarıda) incelendi.

## 11. Açık konular

1. **Başka modüllerden arayüz düzeltmesi gerekir (bu modülde çözülemez):**
   - *Kabuk (shell.py):* halka insertinin yan alan vidaları SH-363-B5…B8 (x 1,181 / 1,259, |y| 0,113) layout
     P-TURRETRING `fastening` metnindeki gibi **kaplama içinde gömme M4 insert** olmalı; yüzer somun plakası kaplamanın
     ~13 mm üstüne çıkıp kayar kapakların yoluna girer (layout `door_outlines` "band"). Alt orta kaplamanın FS1110
     başlığındaki SH-361-B1…B4 (x 1,125, |y| 0,147 / 0,171) yüzer somun plakaları da kapak bandında (x 1,123–1,317)
     ön kapak raylarının ve kızaklarının içinden geçer: bu satır için bant içinde gömme insert (ya da somun plakası
     bandın dışına) gerekir. Arka başlıktaki SH-361-B13… raylara değmez.
   - *Şasi (chassis.py):* ray oturmalarının CH-125 / CH-126-B3 / B4 cıvataları (ISO 4762 M4, ray ekseninden 16 mm,
     baş Ø7 × 4) MGN12 bloğunun zarfına (yarı genişlik 13,5 mm, blok altı 3 mm) 1 mm girer; blok bu z'lerden geçer.
     Cıvatalar ray ekseninden ≥ 18 mm'ye alınmalı ya da havşa başlı / ISO 7380 (k 2,2 mm) olmalı.
   Bu düzeltmeler gelene kadar `tests/test_ucav250_payload.py::test_design_rule_checks_clean` yalnız bu 20 ihlalle
   kalır; diğer bütün testler geçer.
2. **HD59 bağlantı ICD'si** yayımlanmamış: taban Ø120 ve M4 cıvata dairesi (r 42 mm) tahmindir; Trillium ICD/STEP
   ile adaptör delikleri doğrulanacak.
3. **Kütle:** mekanizma (1,870 kg), kalemin %5 büyüme payının %94'ünü kullanır. Kalem dökümü güncellenmeli: raylar /
   taşıyıcı 0,35 → 0,57, kapak tahrikleri 0,304 → ≈ 0,64; bölme astarı 0,40 ve halka 0,06 şasi / kabuk bütçesinde
   ise kalem ≈ 1,88 kg temel olmalıdır. MTOM payı (R-56 0,021 kg) dar olduğundan bu, kütle sahibinin kararıdır
   (spec değişikliği yapılmadı). Ayrıca parçalar `payload` grubunda, kalem `systems` grubundadır.
4. **Ray–şasi boşlukları:** ön kapak rayı ile CH-125 oturma köşebendi arası durağan 0,33 mm, rayların duvar uç
   direklerine 1,1 mm. Şasi çiziminde köşebendin alt ucunda 2 mm pah önerilir.
5. **Satın alma tahminleri:** elastomer izolatör sertliği (HD59 titreşim spektrumuna göre ayar), tahrik birimi
   (BLDC + sonsuz vida + fren + kodlayıcı, 0,20 kg), MGN12 sınıfı ray / blok, bilyalı vida ve somun boyutları üretici
   kataloğuyla doğrulanacak.
6. **Kapak tahriki:** pinyon diş yorulması (7075, sert eloksal) ve DA 22 çıkışına kama bağlantısı deneyle
   doğrulanacak; kapak konum algılayıcıları (layout kilitlemesi) modellenmedi.
7. **Servis döngüsü** U-dönüşü deforme olduğundan modellenmedi; 10 tam çevrim montaj denemesi (adım 24) ile kablo
   ömrü doğrulanacak.
8. **E180 takımı** (0,19 m halka SH-364, adaptör, conta) bu modülde kurulmaz; yalnız zarf uyumu doğrulanır.
9. Layout'un aynı aralıkta kimlik verdiği EQ-MC (PL-850 görev bilgisayarı) ve TR-PAYLOAD (PL-852 yük tepsisi) bu
   üretici kapsamı dışında bırakılmıştır (taret bölmesi dışı); ayrı iş olarak üretilmelidir.
