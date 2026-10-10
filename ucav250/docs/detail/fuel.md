# YK-250 HANÇER — Yakıt sistemi (FUEL) ayrıntılı tasarımı

Üretici modül: `ucav250/design/fuel.py` (`register(reg, spec)`), test: `tests/test_ucav250_fuel.py`.
Sözleşme: `ucav250/ARCHITECTURE.md`; tek doğruluk kaynağı: `ucav250/spec.yaml` (`layout.part_numbers.fuel` 570–619,
`layout.fuel_cells`, `layout.fuel_system`, `layout.fuel_lines` (yol, çap, delikler, valfler, uç bağlantıları),
`layout.chassis.fuel_supports` (bölme sınırları, OML iç payı, kaplama), `layout.chassis.members` (ön yakıt tabanı,
kuyu tavanı, orta kutu, kenar çizgisi kirişi, sırt kanalı), `layout.chassis.fittings` (F-RISER-AFT, F-TRUNNION),
`layout.chassis.trays` TR-AFTBAY, `layout.stations` kesikleri, `layout.systems.equipment` EQ-FUELPUMP / EQ-SHUTOFF,
`layout.shell.panels`, `engine.fuel`, `mass`, `materials`, `processes`, `assembly`) ve
`data/research/components.yaml` / `engine.yaml`. Modül başka bir modülün geometrisini okuyarak yerleşim yapmaz; bütün
arayüzler `spec.layout` üzerindendir (tek istisna: kabuk modülü kurulmuşsa gövde çıkış elemanlarının iç yüz pulu,
kaplamanın ölçülen iç yüzüne göre biçimlenir ve kaplamaya delik açılır — konum yine layout'tandır).

Kapsam yalnızca sivil EO/IR gözetleme platformunun yakıt sistemidir: üç esnek hücre, hatlar, valfler, pompa ünitesi,
ikmal bağlantısı ve yakıtın kendisi. Sistemde silah, mühimmat, dış yük taşıma ya da bırakma düzeneği yoktur ve
öngörülmemiştir; yakıt boşaltma yalnız yerde, hızlı boşaltma valflerinden ve kuru bağlantıdan (geri çekme) yapılır.

## 1. Özet

| Büyüklük | Değer |
|---|---|
| Parça kimliği (570–619) | 49: 46 donanım parçası (44 parça numarası; alt ara bağlantı hattı ve gömme çek valf sağ/sol) + 3 yakıt içeriği (`process: consumable`) |
| Bağlantı elemanı (`joints.py`, geometri `hardware.py`) | 40: 24 × ISO 4762 M3×12, 8 × ISO 7380 M3×6, 2 × ISO 4762 M3×6, 4 × ISO 4762 M4×14, 2 × ISO 4762 M4×12 (hepsi A2-70) |
| Hareketli eklem | yok (yakıt sisteminde `layout.mechanisms` eklemi tanımlı değildir; valf klapeleri satın alma iç parçasıdır) |
| Tasarım kuralı denetimleri | `python3 -m ucav250.analysis.checks --modules chassis,fuel --focus YK250-FU --no-write`: **0 ihlal** (mesh, statik, süpürme, açıklık, kalınlık, bağlantı elemanı, temas, bağlanma); kabuk ile birlikte (`chassis,shell,fuel`) de **0 ihlal** |
| Hücre iç hacmi (yakıt) | ön 26,27 L + eyer 9,07 L + arka 20,24 L = **55,58 L** |
| Tasarım yakıtı (`mass.fuel_kg`) | 30,289 kg = 40,93 L (yoğunluk 740 kg/m³), doluluk oranı 0,736; en ağır durum (`full_fuel_baseline_sensors_only`, × 1,184) 48,4 L — sığar |
| Yakıt AM (eşit doluluk) | x 2,6125 m (layout `mass.fuel_cg` 2,6155 m), y 0, z 0,012 m (yakıt bölme merkezinin altına çöker) |
| Yakıt grubu kütlesi (registry, içerik hariç) | **2,712 kg**; pompa ünitesi (0,35 kg, bütçesi itki kaleminde) hariç **2,362 kg** — bütçe 2,35 ± 0,07 kg (§12) |
| Hat uzunluğu | 14 hat parçası (15 kimlik), toplam ~3,53 m |
| Üçgen sayısı (FU parçaları) | 61 104 |
| Test | `tests/test_ucav250_fuel.py`: 18 test, hepsi geçer (~45 s) |

Önizlemeler (Blender Workbench: standart yedi görünüş `chassis,fuel`; yakıt odaklı izometrik / arka izometrik / üst /
yan / alt / ön, hücresiz görünüşler, şasi bağlamında yarı saydam görünüşler; kabuk ile alt arka bölgede gövde çıkış
elemanlarının yakın çekimi) incelendi: hücreler bölmelere oturur, hatlar çerçeve kesiklerinden geçer, yüzen ya da kopuk
parça, görünür girişim yoktur; boşaltma valfleri ve havalandırma çıkışı kaplama ile aynı yüzeydedir.

## 2. Sistem kavramı (`layout.fuel_system`, fix round 3 / PK3-01)

- **Hücreler.** Ön hücre (ön yakıt tabanı CH-029 üzerinde), eyer hücresi (orta kanat kutusunun üst kapakları üzerinde,
  FS-MS ile FS-RS arasında) ve arka hücre = **besleme hücresi** (ana takım kuyusu tavanı CH-030 üzerinde, FS-RS ile
  FS-GEAR arasında). Arka hücrenin ön alt kenarı orta kutunun altında arka kiriş çerçevesine kadar uzanır (dudak); alt
  ara bağlantıların girişleri bu dudakta, kuyu ön duvarının önündedir.
- **İkmal.** Sol yandaki P-REFUEL kapağının arkasındaki kuru bağlantıdan (FU-574) FL-REFUEL ile ön hücrenin alt
  bağlantısına. Ön hücre dolarken iki alt ara bağlantı (FL-XFER-LO-R/L, çek valf CV-FA ön → arka) arka hücreyi doldurur;
  eyer hücresi FL-FILL-S taşma hattından (çek valf CV-FS) dolar.
- **Aktarma.** Ön hücre arka hücreye yerçekimiyle tabanına kadar boşalır (CV-FA geri akışı keser); eyer hücresi taban
  boşaltma hattı FL-XFER-SA ile (çek valf CV-SA) arka hücrenin tortu bölgesine boşalır.
- **Besleme.** Arka hücredeki toplayıcı (FU-573, başlık deposu) içindeki ağırlıklı esnek emiş ucundan (flop pickup)
  FL-FEED-1 → FS-GEAR → gaskolatör + EFI pompa / regülatör / filtre (FU-617, FU-584; TR-AFTBAY) → FL-FEED-2 → kesme
  valfi (FU-576, perdenin önünde) → yangın perdesi geçiş bloğu (FU-577) → FL-FEED-3 ucu (motor tarafı hortum, itki
  modülü). Regülatör dönüşü FL-RETURN ile toplayıcıya.
- **Havalandırma.** Her hücre üstündeki şamandıralı / devrilme valflerinden (FV-F, FV-S, FV-S2, FV-A, FV-A2) üst
  havalandırma bağlantıları FL-VENT-1 / FL-VENT-2 ve ana hat FL-VENT; FL-VENT bütün hücre üstlerinin ≥ 20 mm üstüne
  çıkan bir **sifon kırıcı döngü** yapar ve P-AFT-LOWER'daki alev tutuculu gömme çıkışta (FU-611) biter.
- **Tortu / boşaltma.** Ön hücre: FL-DRAIN-F → P-CENTRE-LOWER'da hızlı boşaltma valfi (FU-612). Arka hücre / toplayıcı:
  FL-DRAIN-A → P-AFTHATCH'te FU-613. Gaskolatör: FL-DRAIN → P-AFTHATCH'te FU-614. Eyer hücresi arka hücreye boşalır.
  Günlük su kontrolü kapak açmadan yapılır.

## 3. Parça listesi (kütle: registry; sağ/sol ayrı satır)

| Kimlik | Parça | Malzeme / süreç | Kütle kg | Ebeveyn |
|---|---|---|---|---|
| FU-570 | ön yakıt hücresi (esnek) | satın alma (ATL, çizime göre) | 0,343 | CH-115 kaplama |
| FU-571 | eyer yakıt hücresi (esnek) | satın alma | 0,147 | CH-116 |
| FU-572 | arka / besleme hücresi (esnek) | satın alma | 0,286 | CH-117 |
| FU-573 | toplayıcı / başlık deposu | PA12, SLS, 2 mm | 0,068 | FU-572 |
| FU-574 | kuru bağlantı ikmal kaplini (uçak tarafı) | satın alma (OBP AN6) | 0,067 | FU-575 |
| FU-575 | ikmal kaplini braketi | 6061-T6 sac 2 mm, abkant | 0,044 | CH-029 |
| FU-576 | yakıt kesme valfi (EQ-SHUTOFF) | satın alma | 0,080 | FU-577 |
| FU-577 | yangın perdesi yakıt geçiş bloğu + valf rafı | AISI 304, CNC | 0,085 | CH-013 (FS3670) |
| FU-578 / 579 / 595 | kapasitif seviye algılayıcısı (ön / eyer / arka) | satın alma (Gill) | 3 × 0,060 | hücre |
| FU-580 … 594 | yakıt hatları (§5) | PTFE astarlı örgülü hortum / PA12 boru | 0,287 toplam | §5 |
| FU-584 | EFI pompa, regülatör, filtre (EQ-FUELPUMP) | satın alma (Limbach) | 0,350 | CH-121 tepsi |
| FU-596 / 597 / 598 | yakıt (ön / eyer / arka hücre) | `consumable` | 14,317 / 4,942 / 11,031 | hücre |
| FU-600 / 601 / 602 | geçiş contası takımı (FS-MS / FS-RS / FS-GEAR) | floro-silikon kalıp | 0,064 / 0,035 / 0,046 | çerçeve |
| FU-603, FU-605 | hat içi klapeli çek valf CV-FS, CV-SA | satın alma | 2 × 0,030 | hat |
| FU-604-R/L | gömme klapeli çek valf CV-FA | satın alma | 2 × 0,020 | FU-572 |
| FU-606 … 610 | şamandıralı / devrilme havalandırma valfi FV-F, FV-S, FV-S2, FV-A, FV-A2 | satın alma | 5 × 0,050 | hücre |
| FU-611 | alev tutuculu gömme havalandırma çıkışı | satın alma | 0,025 | FU-588 |
| FU-612 / 613 / 614 | hızlı boşaltma valfi (ön tortu / toplayıcı / gaskolatör) | satın alma | 3 × 0,020 | hat |
| FU-615 | arka bölme hat destek braketi | 6061-T6 sac 2 mm | 0,039 | CH-121 |
| FU-616 | yarık yastıklı burçlar (2) | floro-silikon kalıp | 0,004 | FU-615 |
| FU-617 | gaskolatör (Andair GAS375) | satın alma | 0,152 | FU-584 |

Her parçanın malzemesi, süreci (satın alma parçalarında tedarikçi tanımı), kalınlığı (imal edilen parçalarda), ebeveyni,
montaj adımı (18; yakıt içeriği 38), patlatma vektörü ve temas listesi vardır (test `test_part_contract`).

## 4. Yakıt hücreleri (FU-570 / 571 / 572)

**Zarf.** Hücre, `layout.chassis.fuel_supports` bölme bandının (ön / arka sınırlar ok açılı kiriş çerçeveleriyle
süpürülmüş, OML'den 25 mm içeride) kaplamanın içinde kalan kısmıdır: dış yüz kaplamaya / tabana 0,2 mm, üst yüz layout
`fuel_cells[*].z` üst sınırının 0,5 mm altında (sırt kanalı tabanı 1 mm yukarıda). Taban: ön hücrede ön yakıt tabanı
CH-029 + kaplama, arka hücrede kuyu tavanı CH-030 + kaplama, eyer hücresinde orta kutunun üst kapakları (layout z0).
Kesit halkaları sabit nokta düzeniyle (taban, sağ yan kenar çizgisi köşesinde bölünmüş, üst, sol yan) örneklenir ve
loft edilir; dış zarf, iç (yakıt) hacmi ve 0,25 mm duvar kendi kendini kesmeyecek halka sayısıyla seçilir.

**Kaçınma bölgeleri** (layout'tan): kenar çizgisi kirişi (J kesit, kaplama ve köşe payı ile), F-RISER-AFT paraşüt kayış
bağı somun yığını (sırt tabanının altında 12 mm), F-TRUNNION kuyu tavanı cıvatalarının sızdırmaz kubbe somun plakaları
(arka hücre tabanında 12 mm).

**Bağlantı flanşları** (hücreye vulkanize 6061-T6, 7,5 mm): ikmal girişi, iki alt ara bağlantı çıkışı (ön hücre) ve
girişi (arka hücre), ön tortu boşaltması. Her flanş tabanın kuru tarafından 3–4 adet M3 ile (flanşa helisel ek, vida
sabitleyici, deliğin çevresinde O-halka yüz contası) taban / tavana bağlanır; cıvata deseni hücre zarfı (kaçınma
bölgeleri çıkarılmış) içinde 2,5 D kenar payı ve 20 mm duvar şeridi dışında aranır; kuyu tavanında cıvata başları kuyu
ön duvarından ve ara bağlantı hattından uzak tutulur. Çıkış flanşlarında boşaltma için havşalı tortu çukuru (6 mm)
vardır; çıkışlar hücre tabanıyla aynı yüzdedir (kullanılamaz yakıt katmanı `layout_check` C02'de karşılaştırılır).
**Duvar portları:** hatların hücre uç duvarlarını geçtiği yerlerde (eyer doldurma, eyer boşaltma, havalandırma,
besleme, dönüş, toplayıcı boşaltma) çerçeve yüzüne kadar boşluğa sığan vulkanize port göbekleri. **Üst algılayıcı
göbeği** her hücrenin üstünde, yakıt bölmesi erişim kapağının (P-FUEL1/2/3) altında; kaplamaya göbek çevresinde delik.

**Kütle:** duvar alanı × 0,33 kg/m² (components.yaml atl_ultralight_bladder temeli) + flanş / göbek hacmi × 2713 kg/m³
(tahmin). Hücreler kaplamanın yapışkan bant (hook-and-loop) tırnaklarına ve iki tutma kayışına asılır (şasi).

## 5. Hatlar (`layout.fuel_lines`)

| Hat | Kimlik | Görev | Ø mm | Uzunluk m | Yapı | Valf / delik |
|---|---|---|---|---|---|---|
| FL-REFUEL | FU-580 | ikmal | 16 | 0,072 | PTFE hortum | ön taban deliği |
| FL-FILL-S | FU-581 | eyer doldurma / taşma | 12 | 0,060 | PTFE hortum | CV-FS |
| FL-XFER-LO | FU-582-R/L | alt ara bağlantı ön → arka | 10 | 2 × 0,398 | PTFE hortum | CV-FA; ön taban + kuyu tavanı delikleri; FS-MS / FS-RS alt kesikleri |
| FL-XFER-SA | FU-594 | eyer boşaltma → arka | 12 | 0,062 | PTFE hortum | CV-SA; FS-RS C-FUEL-SAD |
| FL-FEED-1 | FU-583 | toplayıcı → pompa | 12 | 0,100 | PTFE hortum | FS-GEAR C-FUEL-FEED |
| FL-FEED-2 | FU-585 | pompa → kesme valfi | 12 | 0,397 | PTFE hortum | destek braketi |
| FL-FEED-3 | FU-586 | kesme valfi → motor (yangın kılıfı) | 12 | 0,065 | PTFE hortum | perde bloğu; uçta nipel |
| FL-RETURN | FU-587 | regülatör → toplayıcı | 10 | 0,701 | PTFE hortum | perde bloğu, destek braketi, FS-GEAR C-FUEL-RET; başta nipel |
| FL-VENT-1 | FU-590 | üst havalandırma ön ↔ eyer | 10 | 0,113 | PA12 boru | FV-F, FV-S; FS-MS C-VENT-MS |
| FL-VENT-2 | FU-591 | üst havalandırma eyer ↔ arka | 10 | 0,123 | PA12 boru | FV-S2, FV-A; FS-RS C-VENT-RS |
| FL-VENT | FU-588 | ana havalandırma, sifon kırıcı döngü | 10 | 0,544 | PA12 boru | FV-A2; FS-GEAR C-FUEL-VENT; FU-611 |
| FL-DRAIN-F | FU-592 | ön tortu boşaltma | 8 | 0,135 | PA12 boru | ön taban deliği; FU-612 |
| FL-DRAIN-A | FU-593 | toplayıcı tortu boşaltma | 8 | 0,255 | PA12 boru | FS-GEAR C-FUEL-DRN; FU-613 |
| FL-DRAIN | FU-589 | gaskolatör boşaltma | 8 | 0,108 | PA12 boru | FU-614 |

Her hat layout yolunu izler; köşeler 2,5 D yarıçapla yuvarlatılır (kısa bacaklarda 90° hortum ucu; yarıçap hortum
yarıçapının 1,2 katından küçükse üretici hata verir). Hatlar çerçeveleri yalnız bildirilen kesiklerden (≥ 2 mm conta payı,
test `test_lines_cross_frames_inside_declared_cutouts`), tabanları yalnız bildirilen deliklerden geçer (< 0,5 mm,
`test_lines_pass_the_declared_penetrations`). Basınçlı hatlar (ikmal, doldurma, ara bağlantı, besleme, dönüş) kıvrık AN
uçlu PTFE astarlı paslanmaz örgülü hortum; havalandırma ve boşaltma hatları SAE J2260 sınıfı PA12 boru (uçları
sıvanmış). Kütle = duvar hacmi × 2400 kg/m³ (hortum) / 1010 kg/m³ (PA12) (tahmin). FL-FEED-3 sonu ve FL-RETURN başı
Ø8 mm nipeldir; motor tarafı hortumlar (itki modülü PR-516 / PR-517) bunlara geçer.

## 6. Valfler, toplayıcı, seviye algılayıcıları

- **Çek valfler.** CV-FS ve CV-SA hat sonunda, alıcı hücrenin içinde hat içi klapeli gövde (soket + akış deliği,
  FKM conta). CV-FA her iki yanda arka hücre alt flanşının üstünde gömme klapeli kafes (iki yan pencere; satıcı kitinin
  4 × M2,5 vidası flanşta — modellenmedi).
- **Şamandıralı / devrilme valfleri.** Hücrenin iç üst yüzünün altına, layout valf noktasında; havalandırma hattı
  valfe girer. Manevra ve ters uçuşta hücre üstünü kapatır.
- **Toplayıcı (FU-573).** SLS PA12, 2 mm duvar, ~0,3 L; besleme hücresinin tabanında arka duvara dayalı. Tabandan 1 mm
  yukarıda üç klapeli giriş, kapakta iki hava deliği; içinde FL-FEED-1'in ağırlıklı esnek emiş ucu, FL-DRAIN-A tortu
  ucu ve FL-RETURN ucu. Hücre bölmeye girmeden önce hücrenin arka port göbeğinden yerleştirilir.
- **Seviye algılayıcıları (FU-578 / 579 / 595).** Gill kapasitif, 60 g; üst duvar normaliyle hücre tabanının 4 mm
  üstüne kadar; boyları sırasıyla 170 / 81 / 183 mm (özel boy). Yakıt miktarı hesabında eğim düzeltmesi yapılır.

## 7. Ekipman ve bağlantı parçaları

- **EFI pompa ünitesi (FU-584, EQ-FUELPUMP) + gaskolatör (FU-617).** Ortak 3 mm taban plakası layout kutusu içinde
  (test `test_equipment_in_layout_boxes`), TR-AFTBAY tepsisine 4 × M4 (rondela + ISO 7040 kilitli somun tepsinin
  altında). Gaskolatör kâsesi plakada; boşaltma nipeli plaka ve tepsiden FL-DRAIN'e. Tepsiye gaskolatör nipeli,
  FL-DRAIN-A ve FL-VENT için geçiş delikleri açılır. Filtreye P-AFTHATCH'ten erişilir.
- **Yangın perdesi geçiş bloğu (FU-577) + kesme valfi (FU-576).** AISI 304 işlenmiş blok: perdenin ön yüzünde
  C-FW-FUEL kesiğini kapatan 1,5 mm flanş, besleme ve dönüş hortumlarını (yangın kılıfı) sandviç ve hava boşluğu
  boyunca paslanmaz kalkan açıklığına taşıyan iki göbek (ateşe dayanıklı geçiş, CS-VLA 1191) ve valf rafı. 4 × ISO 7380
  M3 sandviçin gömülü eklerine; valf rafa alttan 2 × M3. Kesme valfi perdenin motor tarafında değildir (CS-VLA 995);
  uçuş sonlandırma / motor durdurma mantığıyla komut alır.
- **İkmal kaplini (FU-574) + braketi (FU-575).** 6061-T6 2 mm U braket (iç büküm yarıçapı 3 t), dışa flanşlarla ön
  yakıt tabanının altına 4 × ISO 7380 M3 gömülü kör eklere (yakıt tarafı yüz levhası delinmez); kaplin gövdesi braket
  plakasından geçer, altta altıgen flanş, üstte kontra somun. Kaplin yüzü layout FL-REFUEL başlangıcında, P-REFUEL
  kapağının arkasında; ikmal topraklama saplaması brakettedir (CS-LUAS.867(d)).
- **Hat destek braketi (FU-615) + burçlar (FU-616).** FL-FEED-2 ve FL-RETURN'ün yaklaşık orta açıklığında 6061-T6 2 mm
  L braket; 2 × M4 tepsinin hafifletme delikleri arasından. Yarık floro-silikon burçlar hortumu yanal tutar, eksenel
  kaymaya izin verir. Desteksiz açıklık ≤ 0,30 m (test `test_aft_bay_hose_spans_supported`).
- **Geçiş contaları (FU-600 / 601 / 602).** Her bildirilen kesiği 0,2 mm boşlukla dolduran tapa ve iki web yüzünde
  1,5 mm flanş; tabanın, kuyu tavanının, tepsinin ve kiriş başlıklarının çerçeveye değdiği bantlarda flanş kırpılır;
  bölme kaplaması flanş çevresinde kırpılır. Hortumun çevresinde ikiye ayrılmış kalıp, yakıta dayanıklı sızdırmazlık
  macunuyla yapıştırılır (buhar sızdırmaz geçiş).
- **Gövde çıkış elemanları (FU-611 / 612 / 613 / 614).** Gövde dikey; **dış yüzü kavisli OML'ye göre işlenmiş** (alt arka
  gövdenin ~11° eğiminde bile yüzey ile aynı seviyede, ±0,1 mm; test `test_skin_fittings_flush_with_the_oml`), hat
  soketi, iç kaplama yüzünde kontra somun (boşaltma valflerinde altıgen) / flanş. Kabuk kuruluysa kaplamaya 24 köşeli
  delik (gövde + 0,05 mm) açılır ve kaplamanın kavisli iç yüzü ile düz somun arasını dolduran şekilli pul eklenir (pul
  kaplamanın 0,05 mm üstünde; kaplamanın altında pul kalmaz).

## 8. İmalat

| Parça | Süreç | Kural |
|---|---|---|
| Hücreler | ATL ultra hafif kaplı kumaş 0,25 mm, bölme çizimine göre; vulkanize 6061-T6 flanşlar / göbekler | flanş ≥ 7,5 mm (M3 kör diş ≥ 1,2 D), cıvata çevresinde ≥ 2 D + 0,5 mm |
| Toplayıcı | SLS PA12 | duvar 2 mm ≥ `processes.sls_pa12.min_thickness` |
| Braketler (FU-575, FU-615) | 6061-T6 sac 2 mm, lazer kesim + abkant | iç büküm yarıçapı 3 t, kenar payı ≥ 2 D |
| Perde bloğu (FU-577) | AISI 304 CNC frezeleme | flanş 1,5 mm = `cnc_milling_metal` minimumu, iç köşeler R 3 |
| Contalar, burçlar | floro-silikon (FVMQ) kalıp | yakıta dayanıklı, 60 Shore A |
| Hatlar | hazır hortum / boru takımı, boyuna kesilip uç takılır | büküm ≥ 2,5 D |

Genel toleranslar ISO 2768-mK; delikler ISO 273 orta; bütün bağlantı elemanları metrik A2-70 (`joints.py`, uzunluklar
standart diziden).

## 9. Montaj (`spec.assembly`)

**Adım 18 — Yakıt sistemi.** (1) Toplayıcı arka hücreye, çek valfler / şamandıralı valfler / algılayıcı göbekleri
hücrelere takılır. (2) Hücreler katlanarak yakıt bölmesi erişim kapaklarından (P-FUEL1/2/3) bölmelere yerleştirilir,
kaplama tırnaklarına ve kayışlara asılır. (3) Alt flanşlar tabanın kuru tarafından M3 ile (O-halka, vida
sabitleyici, tork) bağlanır. (4) Geçiş contaları hortumların çevresine sarılıp kesiklere yapıştırılır. (5) Hatlar
uçlardan bağlanır: ikmal braketi + kaplin, alt ara bağlantılar, doldurma / boşaltma, havalandırma döngüsü, besleme
(pompa ünitesi tepsiye, destek braketi, kesme valfi + perde bloğu), dönüş. (6) Boşaltma valfleri ve havalandırma
çıkışı kaplama deliklerine takılır (kontra somun / şekilli pul). (7) Seviye algılayıcıları üstten takılır. (8) Basınç
ve sızıntı testi (CS-LUAS.965(a)), hücrelerin perdeye ≥ 13 mm, bağlama (topraklama) sürekliliği kontrolü.

**Adım 38 — Fonksiyon testleri.** Yakıt yüklenir (FU-596/597/598 bu adıma bağlıdır), besleme ve aktarma, kesme
valfi, seviye göstergeleri ve boşaltmalar denenir; motor yer çalıştırması.

## 10. Arayüzler (yalnız `spec.layout`)

| Arayüz | Kaynak | Kullanım |
|---|---|---|
| Bölme sınırları, OML payı, kaplama kalınlığı | `chassis.fuel_supports` | hücre zarfı; hücre ebeveyni = bölme kaplaması (CH-115/116/117) |
| Taban / tavan / kutu / kenar kirişi / sırt | `chassis.members` M-FWDDECK, M-WELLROOF, M-CTBOX, M-CHINE, M-SPINE, M-WELLWALL-FWD | hücre tabanı, flanş cıvatalarının geçtiği parça, kaçınma bölgeleri |
| Bağlantı parçaları | `chassis.fittings` F-RISER-AFT, F-TRUNNION | kaçınma bölgeleri |
| Hat yolları, delikler, valfler | `fuel_lines` | hat geometrisi, flanş ve port konumları |
| Çerçeve kesikleri | `stations[*].cutouts` (FS-MS, FS-RS, FS-GEAR, FS3670) | contalar, perde bloğu |
| Ekipman kutuları | `systems.equipment` EQ-FUELPUMP, EQ-SHUTOFF | pompa ünitesi, kesme valfi |
| Tepsi | `chassis.trays` TR-AFTBAY, `rules.boxes.equipment_bay_aft` | pompa ve destek braketi bağlantısı |
| Kaplama panelleri | `shell.panels` P-REFUEL, P-CENTRE-LOWER, P-AFTHATCH, P-AFT-LOWER, P-FUEL1/2/3 | ikmal kapağı, gövde çıkış elemanları, algılayıcı erişimi |
| Motor tarafı hortumlar | `fuel_lines` FL-FEED-3 sonu / FL-RETURN başı | itki modülü hortumları nipellere geçer |

Bu modülün başka modüllerin parçalarına yaptığı değişiklikler yalnız deliklerdir (`Part.add_hole` / `holes`): bölme
kaplamalarında algılayıcı göbeği ve conta flanşı çevresi, TR-AFTBAY tepsisinde üç geçiş deliği, kabuk kuruluysa alt
kaplama panellerinde dört gövde çıkış deliği.

## 11. Bağlantı elemanları

| Yer | Adet | Eleman | Somun / dişi |
|---|---|---|---|
| Hücre alt flanşları (ikmal, 2 × ara bağlantı çıkışı, 2 × girişi, ön tortu) | 24 | ISO 4762 M3×12 | flanşta helisel ek (kör), O-halka yüz contası |
| İkmal braketi → ön yakıt tabanı | 4 | ISO 7380 M3×6 | gömülü kör ek (yakıt tarafı delinmez) |
| Perde bloğu → yangın perdesi | 4 | ISO 7380 M3×6 | sandviçte gömülü ek |
| Kesme valfi → raf | 2 | ISO 4762 M3×6 | valf tabanında diş |
| Pompa ünitesi → TR-AFTBAY | 4 | ISO 4762 M4×14 | rondela + ISO 7040 |
| Hat destek braketi → TR-AFTBAY | 2 | ISO 4762 M4×12 | rondela + ISO 7040 |

Delikler `joints.py` ile tembel açılır; kenar payı (≥ 2 D metal / 2,5 D kompozit), delme ve adım denetimleri 0 ihlal.

## 12. Kütle ve bütçe

| Kalem | kg |
|---|---|
| Hücreler (3) | 0,776 |
| Hatlar (15 kimlik) | 0,287 |
| Valfler (2 hat içi + 2 gömme çek, 5 şamandıralı, kesme valfi) | 0,430 |
| Toplayıcı | 0,068 |
| Seviye algılayıcıları (3) | 0,180 |
| İkmal kaplini + braketi | 0,111 |
| Perde bloğu | 0,085 |
| Gaskolatör | 0,152 |
| Geçiş contaları (3 takım) | 0,145 |
| Hat destek braketi + burçlar | 0,043 |
| Gövde çıkış elemanları (4) | 0,085 |
| **Yakıt bütçesine giren toplam** | **2,362** |
| EFI pompa / regülatör / filtre (FU-584; bütçesi itki kalemi `engine_group_installed`) | 0,350 |
| **Registry `fuel` grubu (içerik hariç)** | **2,712** |
| Bağlantı elemanları (40; `hardware` bütçesinde) | 0,073 |
| Yakıt içeriği (`consumable`, durum çarpanıyla ölçeklenir) | 30,289 |

Bütçe `mass.budget.fuel` = 2,35 ± 0,07 kg (sizing kalemi `fuel_system_3_cells`: components.yaml fuel_system 1,73 kg +
üç hücrenin ara bağlantısı 0,50 kg, büyüme payıyla). Bütçeye giren toplam **2,362 kg → +0,012 kg (+%0,5), tolerans
içinde.** EFI pompa ünitesinin parça numarası layout'ta yakıt aralığındadır (FU-584) ama kütlesi layout
`EQ-FUELPUMP.mass_item = engine_group_installed` ile itki bütçesinde sayılır (`engine.installed_items_kg`); test bu
kalemi ayrı karşılaştırır (0,35 kg = layout değeri). `analysis/mass.py` grupları `Part.group` ile topladığından, olduğu
gibi çalıştırıldığında yakıt grubunu 2,712 kg gösterir (üst sınırın 0,292 kg üstünde) ve itki grubunu aynı miktar eksik
görür — bu bir hesap ayrımıdır, fazla kütle değildir (açık konu §15).

Kütle temelleri: hücre duvarı 0,33 kg/m² (components.yaml), pompa 0,35 kg (engine.yaml / components.yaml Limbach EFI
kiti), gaskolatör 0,152 kg (üretici), kaplin 0,067 kg, algılayıcı 0,060 kg, şamandıralı valf 0,050 kg
(components.yaml); çek valfler, boşaltma valfleri, havalandırma çıkışı, kesme valfi, hortum / boru yoğunlukları ve
contalar **tahmindir** (parça notlarında belirtilir).

## 13. Layout değişiklikleri (yakıt ayrıntılı tasarımı sırasında; builder üzerinden)

`ucav250/layout_build/b_systems.py` içinde (sonra `python3 -m ucav250.layout_build.build --write`; `layout_check`,
`structures`, `sizing --check` çıkış kodu 0):

1. **Alt ara bağlantı yüksekliği** `Z_XLO` −0,070 → **−0,0735 m**: yatay bacak kuyu tavanının 1,8 mm içine giriyordu;
   şimdi tavanın 1,7 mm altında.
2. **`XLO_WELL_GAP` = 11 mm:** arka hücre girişinin yukarı bacağı ana takım kuyusu ön duvarının (M-WELLWALL-FWD, yakıt
   hattı deliği yok) önünde kalır; eski yolda yatay bacak duvarı kesiyordu.
3. **FL-XFER-SA parça numarası** FU-584 → **FU-594**: FU-584 EQ-FUELPUMP'a aitti (çakışma).

## 14. Doğrulama

```
python3 -m ucav250.analysis.checks --modules chassis,fuel --focus YK250-FU --no-write        # 0 ihlal
python3 -m ucav250.analysis.checks --modules chassis,shell,fuel --focus YK250-FU --no-write  # 0 ihlal
python3 -m unittest tests.test_ucav250_fuel                                                 # 18 test, OK
python3 -m ucav250.layout_build.build --check                                               # değişmedi
python3 -m ucav250.analysis.layout_check --check; python3 -m ucav250.analysis.structures --check
python3 -m ucav250.analysis.sizing --check                                                  # hepsi çıkış 0
python3 -m ucav250.blender.build --modules chassis,fuel --previews --no-blend --out <klasör> # önizlemeler
```

Test kapsamı: tasarım kuralı denetimleri (8 denetim, sıfır), kimlik aralığı ve sağ/sol eşleri, layout'un sabitlediği
kimlikler (hücreler, hatlar, pompa, kesme valfi), parça sözleşmesi, yasaklı ifade taraması, kapalı mesh ve ayna hacim
eşitliği, delikler ve çerçeve kesikleri, ekipman kutuları, sifon kırıcı döngü yüksekliği, desteksiz hortum açıklığı,
gövde çıkış elemanlarının OML ile aynı yüzeyde olması, yakıt içeriği (tasarım yakıtı, yoğunluk, en ağır durum sığar),
yakıt AM'si, satın alma verilerinin components.yaml ile eşliği, bağlantı elemanları, bütçe.

## 15. Açık konular

1. **Pompa kütlesinin bütçe grubu:** `analysis/mass.py` `Part.group` ile topluyor; FU-584 için layout `mass_item`
   (`engine_group_installed`) dikkate alınmalı ya da bütçe yeniden bölünmeli (entegrasyon, görev #130).
2. **Algılayıcı kablo yolu:** `layout.systems.harness` içinde yakıt seviye algılayıcıları için kol yok; kablolar
   şimdilik erişim kapaklarının altından çıkıyor (layout'a kol eklenmeli).
3. **Toplayıcı düşük seviye anahtarı** tipi seçilmedi (kapakta yer ayrıldı).
4. **Tedarikçi verileri:** kuru bağlantı kaplini boyutları (Ø32 × 40 mm zarf), çek valf, boşaltma valfi, havalandırma
   çıkışı ve kesme valfi kütleleri tahmindir; teklif / veri sayfası ile güncellenmeli.
5. **Hücre askıları:** kaplamadaki yapışkan bant tırnakları ve iki tutma kayışı modellenmedi (şasi kaplaması).
6. **CV-FA satıcı vidaları** (4 × M2,5, flanşta) ve ikmal topraklama saplaması modellenmedi.
7. **Basınç / sızıntı testi** (CS-LUAS.965(a)) ve hortum basınç sınıfları hesapla doğrulanmadı; hücre üreticisinin test
   raporu ile kapatılacak.
8. **Yakıt miktarı ölçümü:** üç algılayıcının eğim düzeltmeli kalibrasyon tablosu (hücre geometrisinden) hazırlanmalı.
