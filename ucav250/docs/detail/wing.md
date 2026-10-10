# YK-250 HANÇER — Dış kanat (WING) ayrıntılı tasarımı

Üretici modül: `ucav250/design/wing.py` (`register(reg, spec)`), test: `tests/test_ucav250_wing.py`.
Sözleşme: `ucav250/ARCHITECTURE.md`; tek doğruluk kaynağı: `ucav250/spec.yaml` (`wing`, `layout.part_numbers`,
`layout.chassis.wing_joint`, `layout.mechanisms.joints`, `layout.systems`, `layout.clearances`,
`structures.sizing.wing` / `wing_joint`, `layups`, `processes`, `materials`) ve `data/research/components.yaml`
(Volz eyleyici verileri). Modül başka bir modülün geometrisini okumaz; bütün arayüzler `spec.layout` üzerindendir.

Kapsam yalnızca sivil EO/IR gözetleme platformunun sökülebilir dış kanat panelleri, kanatçıkları ve flaplarıdır:
kanat üzerinde silah, mühimmat, dış yük taşıma ya da bırakma düzeneği yoktur ve öngörülmemiştir.

## 1. Özet

| Büyüklük | Değer |
|---|---|
| Parça kimliği | 149 (grup `wing` 85: YK250-WG-150 … -192; grup `controls` 64: YK250-FC-200 … -239) |
| Parça numarası | 75 (sağ panel +y'de modellenir, sol panel `mirror_part` ile aynalanır; WG-192 pito direği yalnız sol) |
| Bağlantı elemanı (`joints.py`, geometri `hardware.py`) | 108: 78 × M3, 16 × M4, 14 × ISO 2341-B menteşe pimi |
| Hareketli eklem | 12: `aileron_R/L`, `flap_R/L` (layout) + 8 bağlı eklem (`*_servo_*`, `*_rod_*`, `expr` ile) |
| Hareket dizileri | 6: `flap_R/L_linkage`, `aileron_R/L_linkage` (11'er tam dört-çubuk durumu), `wing_controls_R/L_extremes` |
| Tasarım kuralı denetimleri | `checks --modules chassis,wing --focus YK250-WG,YK250-FC`: **0 ihlal** (mesh, statik, süpürme, açıklık, kalınlık, bağlantı elemanı, temas, bağlanma) |
| Kanat grubu kütlesi (registry) | **12,30 kg** — bütçe 15,18 ± 0,46 kg (bkz. §9) |
| Kumanda grubu (kanat payı) | **4,51 kg** — bütçe kalemleri toplamı 4,56 kg |
| Kanat bağlantı elemanları (grup `hardware`) | 0,24 kg |

Önizlemeler (`python3 -m ucav250.blender.build --modules chassis,wing --previews --no-blend`): ön, yan, üst, alt, iki
izometrik ve patlatılmış izometrik görünüş incelendi; paneller birleşim düzleminde şasiye oturur, yüzen parça, kopuk
geometri ya da görünür girişim yoktur.

## 2. Yapı ve yük yolları

Panel y 0,70 … 3,60 m (açıklık boyunca 2,90 m; ok 8°, dihedral 3°, burulma 4°). Yapı iki kirişli bir kutu + D-burun
hücresidir:

* **Ana kiriş (WG-151, tek kür):** UD başlıklar 30 mm geniş, `structures.sizing.wing.main_cap.zones` kat sayılarıyla
  (y 0,70'te 36 kat ≈ 5,0 mm, y 2,0'dan dışarı 10 kat ≈ 1,4 mm), dış yüzleri OML'nin 1 mm + yapıştırma altında;
  ±45 PW gövde `main_web.zones` (6 → 3 kat). Kök bölmesinde 0,10 m'lik derinlik geçişi (VS2-04): başlıklar loft
  konumundan dil flanş çizgisine iner, ROHACELL 71 dolgular; birleşim dili 30 × 61 mm (UD flanşlar 10 mm, iç pimde
  3 mm'ye incelir; 25 katlı gövde; pim deliklerinde [±45/0/90] katı bloklar; uçta 3 × 30° pah). Eğilme → başlıklar →
  dil → iki ana pim (düşey kuvvet çifti) → CFRP çatal (YK250-CH-053) → orta kutu (YK250-CH-001).
* **Arka kiriş (WG-155):** PW C kesit (3 kat gövde, flanş altında 2 kat UD şerit, `rear_web` / `rear_cap`), gövde arka
  yüzü kumanda yüzeyi oluklarının hemen önünde düz bir plan çizgisinde (veter oranı 0,713 → 0,709). Menteşe braketlerinin
  altında 10 katlı gövde takviyeleri. Kökte 7075 bağlantı (WG-152): 8 mm kulak, Ø8 H8 delik; arka pim P-REAR (WG-160,
  Ti bilyalı kilit) şasideki yuva bağlantısına (YK250-CH-054) girer. Sürükleme momenti ana pimlerle arka pim arasında
  açıklık yönünde kuvvet çifti; burulma D-burun + kutu kapalı hücresinden kök kaburgaya, oradan pimlere.
* **Kaplamalar:** D-burun hücum kenarı kaplaması (WG-150, `wing_skin_primary` 0,6/5/0,4 mm), üst kutu kaplaması
  (WG-153, y 2,20'ye kadar `wing_box_skin_upper` 0,6/6/0,4 mm, 30 mm rampa ile birincil sandviçe döner) ve alt kutu
  kaplaması (WG-154). Başlıkların üstünde, burunda ve oluk dudaklarında çekirdek 1:3 rampayla 1 mm katı laminata iner.
  **Kök bölmesi takviyesi** (`wing_joint.transition`): iki kutu kaplamasının dış yüzüne kirişler arasında ilk 0,12 m
  boyunca +1 PW kat (son 20 mm kat bırakma); OML loft üzerinde kalır, iç yüz 0,2 mm içeri basamaklanır.
* **Kaburgalar:** kök kaburga (WG-161, birleşim düzlemine paralel, dil / kulak / CN-WING konnektör geçişleri ve uç
  bağlantı insertleri çevresinde 20 katlı katı alanlar), dokuz istasyonda burun (WG-164…172) ve kutu (WG-173…181)
  parçaları (y 0,83 / 1,13 / 1,40 / 1,70 / 2,01 / 2,26 / 2,55 / 2,88 / 3,20; aralık ≤ 0,35 m = `structures.rib_pitch_m`),
  uç kaburga (WG-182). `rib_panel` sandviç gövde, kaplamalara yapışan 15 mm × 1,6 mm T flanşlar, başlık / flanş
  çentikleri, hafifletme delikleri, kablo borusu geçişi. Servo bölmeleri y 0,83–1,13 (flap) ve 2,01–2,26 (kanatçık).
* **Firar kenarı:** kumanda yüzeyleri arasındaki sabit kapamalar (WG-183/184/185, köpük çekirdek + 3 kat PW), uç
  kaportası (WG-186, 4 kat 7781 CTP; LT-WING ışığı için cep).
* **Kumanda yüzeyleri:** kanatçık (FC-200, y 2,16–3,42) ve flap (FC-202, y 0,73–2,09), `structgen.control_surface_regions`
  ile yuvarlak burun + oluk (0,75 veter, orta kalınlık menteşe çizgisi, 3,5 mm oluk boşluğu), tam derinlikte ROHACELL
  51 WF çekirdek + 3 kat PW kaplama, uç yüzleri menteşe eksenine dik (3,5 mm uç boşluğu). Yüzey yükü → gömülü 7075
  menteşe dilleri (FC-230…236) → ISO 2341-B pimler → 7075 çatal braketler (FC-220…226) → 2 × M3 + somun plakası →
  arka kiriş gövdesi (10 katlı takviye) → kaburgalar / kaplamalar.
* **Tahrik:** Volz DA 26 (kanatçık, FC-204) ve DA 30 (flap, FC-206), alt kaplamadaki servo kapaklarının üzerinde
  7075 yataklarda (FC-214/215, kapağa yapıştırılmış; FC-216…219 bağlama kelepçeleri 2 × M3 ile); servo kolu
  (FC-208/209) → itme çubuğu (FC-210/211, 7075 boru 6 × 1,5) → boynuz (FC-212/213, 2 × M3 yüzey kaplamasından
  gömülü 7075 bloklara FC-238/239). Eyleyici tepkisi: yatak → kapak (WG-190/191, 4 × M3 somun plakalı) → kapak
  çerçevesi (WG-187/188, kaplama iç yüzüne yapışık 10 kat halka) → alt kaplama.
* **Sistem geçişleri:** CTP kablo borusu (WG-189, 14 × 1 mm) CN-WING fişinin arka kabuğundan (kök kaburgadan 40 mm)
  uç bölmesine; y 1,70'ten dışarı ana kiriş gövdesinin arka yüzüne yapışık, içeride flap servosunun üstünden geçer.
  Sol panelde AD-PITOT-WING sondası için 7075 pito direği (WG-192-L, %15 veterde alt kaplama, 2 × M3 insertli).
  Her kapalı bölmenin en alçak noktasında (iç kaburga flanşının hemen dışı; 3° dihedral suyu içeri akıtır) Ø4 tahliye
  deliği: panel başına 8; servo bölmeleri kapak boşluğundan boşalır.

## 3. Parça listesi

Kütleler registry'den (mesh hacmi × etkin yoğunluk; sandviçler `slab_mass` / `rib_mass` ile laminasyondan; satın alma
parçalar veri sayfası). Satırlar L + R çiftinin toplamıdır (WG-192 yalnız sol).

| No | Parça (TR) | Malzeme | Süreç | Kalınlık / laminasyon | Adım | Adet | Kütle (kg) |
|---|---|---|---|---|---:|---:|---:|
| WG-150 | hücum kenarı kaplaması (D-burun) | cfrp_pw_mtm45_as4 | prepreg_ooa_vacbag | wing_skin_primary, 1 mm katı burun/başlık | 35 | 2 | 2.128 |
| WG-151 | ana kiriş + birleşim dili | cfrp_ud_mtm45_as4 (+ PW, R71) | prepreg_ooa_vacbag | UD başlık zonları, gövde 6→3 kat | 35 | 2 | 2.566 |
| WG-152 | arka kiriş kök bağlantısı (kulak) | al_7075_t651_plate | cnc_milling_metal | 8 mm kulak, 3,8 mm flanş | 35 | 2 | 0.183 |
| WG-153 | üst kutu kaplaması | cfrp_pw_mtm45_as4 | prepreg_ooa_vacbag | wing_box_skin_upper → wing_skin_primary | 35 | 2 | 1.972 |
| WG-154 | alt kutu kaplaması (servo ağızları, tahliye delikleri) | cfrp_pw_mtm45_as4 | prepreg_ooa_vacbag | wing_skin_primary | 35 | 2 | 1.725 |
| WG-155 | arka kiriş (C kesit) + menteşe takviyeleri | cfrp_pw_mtm45_as4 | prepreg_ooa_vacbag | 0,6 mm gövde, 10 kat ped | 35 | 2 | 0.579 |
| WG-156/157 | dil burçları P-MAIN1/2 (4130, 16 H8 × Ø22 × 30) | steel_4130_n | cnc_milling_metal | 3 mm et | 35 | 4 | 0.168 |
| WG-158/159 | ana pimler P-MAIN1/2 (Ti-6Al-4V, Ø16 h8 × 62) | ti_6al_4v_annealed_sheet | cnc_milling_metal | — | 35 | 4 | 0.268 |
| WG-160 | arka pim P-REAR (Ti, bilyalı kilit, Ø8 f7) | ti_6al_4v_annealed_sheet | cnc_milling_metal | — | 35 | 2 | 0.014 |
| WG-161 | kök kaburga | cfrp_pw_mtm45_as4 | prepreg_ooa_vacbag | rib_panel + 20 kat katı alanlar | 35 | 2 | 0.291 |
| WG-162/163 | ana pim emniyet plakaları | al_7075_t651_plate | cnc_milling_metal | 2,5 mm | 35 | 4 | 0.064 |
| WG-164…172 | burun kaburgaları (9 istasyon) | cfrp_pw_mtm45_as4 | prepreg_ooa_vacbag | rib_panel, 1,6 mm flanş | 35 | 18 | 0.279 |
| WG-173…181 | kutu kaburgaları (9 istasyon) | cfrp_pw_mtm45_as4 | prepreg_ooa_vacbag | rib_panel, 1,6 mm flanş | 35 | 18 | 0.636 |
| WG-182 | uç kaburga | cfrp_pw_mtm45_as4 | prepreg_ooa_vacbag | rib_panel | 35 | 2 | 0.022 |
| WG-183/184/185 | sabit firar kenarı kapamaları (kök / orta / uç) | cfrp_pw_mtm45_as4 | prepreg_ooa_vacbag | R51 çekirdek + 0,6 mm | 35 | 6 | 0.106 |
| WG-186 | kanat ucu kaportası (CTP) | gfrp_7781_mtm45 | prepreg_ooa_vacbag | 1,0 mm | 35 | 2 | 0.140 |
| WG-187/188 | flap / kanatçık servo kapağı çerçevesi | cfrp_pw_mtm45_as4 | prepreg_ooa_vacbag | 2,0 mm (10 kat) | 35 | 4 | 0.183 |
| WG-189 | kablo demeti borusu (CTP 14 × 1) | gfrp_7781_mtm45 | prepreg_ooa_vacbag | 1,0 mm | 35 | 2 | 0.406 |
| WG-190/191 | kanatçık / flap servo kapağı + bağlantı kaportası | cfrp_pw_mtm45_as4 | prepreg_ooa_vacbag | 2,0 mm orta, kenar deri iç yüzüne kadar | 35 | 4 | 0.551 |
| WG-192-L | pito direği (AD-PITOT-WING) | al_7075_t651_plate | cnc_milling_metal | 2,0 mm taban | 35 | 1 | 0.020 |
| FC-200 | kanatçık | cfrp_pw_mtm45_as4 | prepreg_ooa_vacbag | R51 çekirdek + 0,6 mm | 35 | 2 | 0.545 |
| FC-202 | flap | cfrp_pw_mtm45_as4 | prepreg_ooa_vacbag | R51 çekirdek + 0,6 mm | 35 | 2 | 1.003 |
| FC-204 | kanatçık eyleyicisi Volz DA 26 | satın alma | — | — | 35 | 2 | 0.540 |
| FC-206 | flap eyleyicisi Volz DA 30 | satın alma | — | — | 35 | 2 | 1.260 |
| FC-208/209 | servo kolları | al_7075_t651_plate | cnc_milling_metal | 3 mm | 37 | 4 | 0.012 |
| FC-210/211 | itme çubukları (6 × 1,5) | al_7075_t651_plate | cnc_milling_metal | 1,5 mm et | 37 | 4 | 0.038 |
| FC-212/213 | kumanda boynuzları | al_7075_t651_plate | cnc_milling_metal | 3 mm | 37 | 4 | 0.038 |
| FC-214/215 | eyleyici yatakları | al_7075_t651_plate | cnc_milling_metal | 2 mm taban, 3 mm duvar | 35 | 4 | 0.732 |
| FC-216…219 | eyleyici bağlama kelepçeleri | al_7075_t651_plate | cnc_milling_metal | 2,5 mm | 35 | 8 | 0.066 |
| FC-220…222 | kanatçık menteşe braketleri (y 2,31 / 2,70 / 3,10) | al_7075_t651_plate | cnc_milling_metal | 3 mm kulak | 35 | 6 | 0.049 |
| FC-223…226 | flap menteşe braketleri (y 0,795 / 1,25 / 1,55 / 1,88) | al_7075_t651_plate | cnc_milling_metal | 3 mm kulak | 35 | 8 | 0.110 |
| FC-230…236 | menteşe dilleri (yüzeye gömülü) | al_7075_t651_plate | cnc_milling_metal | 3 mm | 35 | 14 | 0.054 |
| FC-238/239 | boynuz takviye blokları (gömülü) | al_7075_t651_plate | cnc_milling_metal | 8 mm | 35 | 4 | 0.063 |

Her parçada `material`, `process`, `thickness` veya `layup`, `parent`, `step`, `explode` ve `contacts` doludur
(`test_part_contract`); bildirilen temaslar gerçekten değer (`checks` temas denetimi); sol parçalar sağın birebir
aynasıdır, delikleri ve bağlantı elemanlarıyla (`test_meshes_closed_and_mirror_equal`).

## 4. İmalat

**OOA prepreg + vakum torbası, fırın kürü (`prepreg_ooa_vacbag`; en az 0,6 mm, kenar mesafesi ≥ 2,5 D, kat bırakma
1:20, ISO 2768-mK kesim ve delikler):**

* Kaplamalar dişi kalıplarda (OML takım yüzü): dış yüz katları, çekirdek (CNC ile 1:3 rampalı ROHACELL 51 WF),
  iç yüz katları; başlık / burun / oluk dudağı bölgelerinde çekirdek biter, 5 katlı katı laminat. Kök bölmesi takviye
  katı dış yüzün altına serilir, son 20 mm kademeli bırakılır. Servo ağızları ve tahliye delikleri kürden sonra CNC.
* Ana kiriş tek kürde eşleşik takımda: UD başlıklar zonlara göre kat bırakmalı, ±45 gövde, kök bölmesinde R71 dolgular
  ve dil; dil burç delikleri Ø22 boşaltılır, 4130 burçlar usta dil mastarında yapıştırılır. Arka kiriş C kesiti erkek
  kalıpta; menteşe pedleri aynı kürde.
* Kaburgalar düz sandviç levhadan su jeti / CNC kesim, T flanşlar 8 kat PW şerit olarak ikincil yapıştırma
  (EA 9394, 0,1–0,2 mm); kök kaburganın katı alanları 20 kat.
* Kumanda yüzeyleri: CNC ile işlenmiş tam derinlik R51 çekirdek, menteşe dilleri ve boynuz blokları çekirdeğe gömülür,
  3 kat PW kaplama dişi kalıpta sarılıp vakumla kürlenir; uç yüzleri ve oluk burnu kürden sonra kesilir.
* Kapak çerçeveleri ve kapaklar (2 mm katı laminat, kenar deri iç yüzüne kadar yükseltilmiş, bağlantı kabarcığı
  kapakla tek parça), CTP uç kaportası ve kablo borusu (mandrel üzerinde) aynı süreçle.

**CNC frezeleme (7075-T651 levha / 4130 / Ti-6Al-4V çubuk; ISO 2768-mK, oturmalar ISO 286):** kök bağlantısı, menteşe
braketleri (taban + çatal kulaklar tek parça; kulak yarıçapı 2 D + 0,5 mm), menteşe dilleri, servo kolları, boynuzlar,
yataklar ve kelepçeler, boynuz blokları, emniyet plakaları, pito direği; en küçük et 1,5 mm, iç köşe ≥ 3 mm.
Ana pimler Ø16 h8 taşlanmış, arka pim Ø8 f7; 4130 burçlar 16 H8 son raybalama (montaj adımı 5'teki hat raybalama ile).

## 5. Montaj

**Panel ön montajı (kanat sehpası, adım 35 öncesi alt montaj):** ana ve arka kiriş sehpaya; kaburgalar (kök, dokuz
istasyon, uç) kirişlere yapıştırılır; arka kiriş kök bağlantısı (WG-152) gövde flanşından yapıştırılır ve kök
kaburgaya 4 × M4 (geçme insertler) ile bağlanır; menteşe braketleri 2 × M3 ile arka kiriş gövdesine (somun plakaları
ön yüzde); kablo borusu ve servo kapağı çerçeveleri; alt kutu kaplaması, sonra üst kutu kaplaması ve D-burun kaplaması
yapıştırılır; sabit firar kenarı kapamaları, uç kaburga ve uç kaportası. Kumanda yüzeyleri menteşe pimleriyle
(ISO 2341-B + pul + kopilya) asılır; eyleyiciler yatak ve kelepçeleriyle kapaklara monte edilir, kapaklar 4 × M3 ile
kapatılır.

| Adım (spec.assembly) | Kanat işi | Başlıca bağlantılar |
|---:|---|---|
| 5 | (şasi) çatal burçları ve arka kiriş yuva bağlantısı — usta dil mastarı WG-151 arayüzünü tanımlar | — |
| 35 | Panel ana kiriş ekseni boyunca içeri sürülür (dil çatala, kulak yuvaya, CN-WING fişi yuvasına aynı anda); iki ana pim önden (P-JOINTACCESS), arka pim alttan (P-REARACCESS); emniyet plakaları; eyleyici kontrolü | emniyet plakası başına 2 × M4 çatal ön pedindeki insertlere; arka pim bilyalı kilit |
| 37 | Kumanda ayarı: nötr, aralık, yön; servo kolu / itme çubuğu / boynuz takılır | itme çubuğu uçları M3 ISO 7380 + ISO 7040; boynuz 2 × M3 gömülü bloğa |

Sökme sırası tersidir; servo bakımı için itme çubuğunun boynuz cıvatası sökülür, kapak eyleyici, yatak ve kolla
birlikte çıkar.

## 6. Arayüzler

| Modül | Arayüz (spec.layout) | Kanat tarafı |
|---|---|---|
| şasi | `chassis.wing_joint`: düzlem y 0,70 (1,5 mm conta), takma ekseni, pimler P-MAIN1/2 (konum, eksen, Ø, boy, baş), çatal yarığı, arka kulak / pim / yuva bağlantısı, `members.M-CTBOX.rear_spar_line` | kök kaburga iç yüzü y 0,7049; dil 30 × 61 mm, iki burç; pimler ve emniyet plakaları (CH-001 çatal ön pedine M4); WG-152 kulağı CH-054 yuvasında; kulak ön kenarı orta kutu arka kirişine paralel kırpılmış |
| kaplama (shell) | P-JOINTACCESS, P-REARACCESS kapakları, eldiven / birleşim panelleri | pimler bu kapaklardan takılır; kanat paneli kendi OML'sini kendisi kapatır |
| sistemler | `systems.actuators` ACT-AILERON / ACT-FLAP (parça no FC-204 / 206), `harness` CN-WING / H-WING, `air_data_lights` LT-WING, AD-PITOT-WING | eyleyiciler (veri sayfası zarfları), kök kaburgada Ø31 konnektör deliği, kablo borusu, uç kaportasında ışık cebi (ışık ve M5 vidaları sistemler modülünün), sol direkte sonda kelepçesi (sonda YK250-SY-789 sistemlerin) |
| mekanizmalar | `mechanisms.joints` aileron_R/L, flap_R/L (orijin, eksen, lo/hi, prop, scale) | eklemler birebir layout'tan; `KO-SWEEP-CONTROLS` ve `clearances` (kanatçık / flap ↔ kanat yapısı ≥ 3 mm) süpürme denetiminde |

## 7. Mekanizmalar ve kinematik

* Menteşe çizgileri layout eklemlerinden (0,75 veter, orta kalınlık; kanatçık ±20°, flap 0…40°, dinlenme 0 = nötr /
  flap kalkık). Her yüzeyin dört-çubuğu `actuation.linkage_kinematics` ile çözülür; servo ve itme çubuğu bağlı
  eklemlerdir (Blender ifadesi: kontrol özelliğinde 5. derece Horner polinomu, tam değerlerden sapma ≤ 2·10⁻³ rad);
  `*_linkage` dizileri 11 tam durum taşır, `test_linkages_close_in_every_sequence_state` her durumda çubuğun boynuz
  ucunun boynuz deliğine 0,05 mm içinde oturduğunu doğrular.
* Servo kursu ve iletim açısı (test `test_servo_travel_and_transmission`):

| Yüzey | Kol / boynuz (mm) | İtme çubuğu (mm) | Servo kursu | Veri sayfası sınırı | En küçük iletim açısı | Bölme payı (iç / dış) |
|---|---|---|---|---|---|---|
| Flap (DA 30) | 18 / 36 | 134,8 | 0 … 86,3° (kurs ortası ≈ flap 20°, bağlantı nötrü) | ±85° (varsayılan ±45°) | 68,8° | 9,3 / 9,3 mm |
| Kanatçık (DA 26) | 16,8 / 33,6 | 105,6 | −43,3 … +43,0° | ±50° standart | 68,6° | 11,9 / 11,8 mm |

* Süpürme: her eklem tam aralığında ≥ 9 örnekle ve kayıtlı dizilerle (iki yüzeyin uç durumları birlikte) denetlenir;
  yüzeyler, menteşe donanımı, boynuz / çubuk / kol ve kapak kabarcığı arasında girişim yoktur, yüzey ↔ sabit yapı
  ≥ 3 mm.
* Yapısal el kontrolleri (as-built geometri, `structures.py` ile aynı formüller; test
  `test_pushrod_column_and_rod_end_on_built_geometry`): sınır itme kuvveti = tepe tork × en büyük oran / boynuz kolu —
  flap 41,9 N·m / 36 mm = 1163 N, kanatçık 13,0 N·m / 33,6 mm = 386 N. İtme çubuğu kolonu (Euler, mafsallı uçlar,
  FoS 1,5): flap P_kr 2301 N → MS 0,32; kanatçık 3748 N → MS 5,47. Çubuk ucu M3'ün 3 mm 7075 boynuzda ezilmesi
  (3,33 toplam katsayı): flap MS 1,32, kanatçık MS 5,99. Kanatçık menteşe pimi Ø3 (bkz. §10): kulak ezilmesi
  9680 N / (6,67 × 241 N) → MS 5,0; pim çift kesme 4874 N / (1,725 × 241 N) → MS 10,7. Eyleyici bağlantısı
  (kapak → çerçeve 4 × M3): vida başına 1163/4 × 1,5 = 436 N sınır; 2 mm çerçeve alanında M3 ezilmesi
  (QI ezilme değeri 416 MPa, FoS 1,5 × kompozit 1,2) 2496 N → MS 2,2.

## 8. Bağlantı elemanları

| Bağlantı | Eleman | Somun / karşı parça | Adet (iki panel) |
|---|---|---|---:|
| Menteşe braketi → arka kiriş gövdesi | M3 | somun plakası (gövde ön yüzü) | 28 |
| Servo kapağı → çerçeve | M3 ISO 7380 | yüzer somun plakası | 16 |
| Kelepçe → eyleyici yatağı | M3 ISO 7380 | kılavuz dişli (yatak göbeği) | 16 |
| Boynuz → yüzey (gömülü blok) | M3 ISO 7380 | kılavuz dişli (7075 blok) | 8 |
| İtme çubuğu uçları | M3 ISO 7380 | ISO 7040 + pul | 8 |
| Pito direği → D-burun | M3 ISO 7380 | geçme insert | 2 |
| Kök bağlantısı → kök kaburga | M4 | geçme insert (kök kaburga) | 8 |
| Emniyet plakası → çatal ön pedi (CH-001) | M4 | geçme insert (şasi) | 8 |
| Menteşe pimi | ISO 2341-B Ø4 (flap) / Ø3 (kanatçık), A2-70 | ISO 1234 kopilya + ISO 7089 pul | 14 |
| **Toplam** | | | **108** |

Bütün elemanlar `joints.py` ile kurulur (yığın kalınlığı geometri üzerinde ölçülür, ISO 273 orta boşluk delikleri
parçalara açılır, boy standart boya yuvarlanır). Kenar mesafesi metalde ≥ 2 D, kompozitte ≥ 2,5 D ve adım kuralları
denetimde sıfır ihlal verir. Ana pimler ve arka pim parça olarak modellenmiştir (WG-158/159/160).

## 9. Kütle ve bütçe

| Grup / kalem | Bütçe (kg) | Ayrıntılı tasarım (kg) | Fark (kg) |
|---|---:|---:|---:|
| `wing` (wing_structure_pair) | 15.18 ± 0.46 | 12.30 | −2.88 |
| actuators_ailerons_2x_DA26 | 0.672 | 0.905 (eyleyici + yatak + kelepçe + kol + çubuk + boynuz + blok) | +0.233 |
| actuators_flaps_2x_DA30 | 1.638 | 1.844 | +0.206 |
| control_surfaces_ailerons_pair (+ menteşeler) | 0.910 | 0.613 | −0.297 |
| control_surfaces_flaps_pair (+ menteşeler) | 1.343 | 1.147 | −0.196 |
| **`controls` grubunun kanat payı** | **4.562** | **4.509** | **−0.053** |
| kanat bağlantı elemanları (`hardware` grubu) | — | 0.238 | |

`analysis/mass.py` bütçe denetimi iki yönlüdür (|m − hedef| ≤ tol): kanat grubu toleransın 2,42 kg **altındadır**
(aşım yoktur). Nedeni: `wing_structure_pair` kavram kalemi (dayanıklılık çalışması kanat modeli) orta kutudan geçen
kiriş başlıklarını ve dış panel birleşimlerini de kapsar; bunlar layout gereği şasi modülündedir (YK250-CH-001 orta
kutu UD başlıkları, CH-053 çatal + burçlar, CH-054 yuva bağlantısı). Kumanda tarafında yataklar (0,73 kg) eyleyici
kalemlerini aşar, yüzeyler kavram kaleminden hafiftir; toplam kanat payı bütçe içinde. Bütçe kalemlerinin ayrıntılı
geometriye göre gruplar arasında yeniden dağıtılması entegrasyon işidir (bkz. §11, açık konu 1).

## 10. Layout'tan / yapı hesabından sapmalar (gerekçeli)

* **Servo konumları:** layout ACT-FLAP / ACT-AILERON kutuları paketleme zarfıdır; modül servoyu kendi bölmesinde
  (kaburgalar arası, kapak ağzı ve alanları içinde, menteşe eksenine hizalı) iki uçta eşit payla yerleştirir.
  Kasa merkezi: flap Δx ≈ −0,035 m, Δy ≈ +0,09 m; kanatçık Δx ≈ +0,06 m (ileri sınır, alt kaplamadaki ana kiriş
  başlığı rampası + çerçeve kenarı + kapak alanıdır). Kütle merkezi etkisi registry geometrisinden hesaplanır.
* **Kol / boynuz boyları:** `wing.controls.*.linkage` oranı (2:1) ve nötr açıları korunarak flapta 1,2, kanatçıkta 1,4
  katı (18/36 ve 16,8/33,6 mm): boynuz ucu yüzeyin alt yüzünden kapak kabarcığına uzanır, çubuk alt kaplama
  çizgisinin altından geçer. Servo açıları ve oran eğrisi ölçekten bağımsızdır (spec ±43°, model ±43,2°); itme kuvveti
  aynı oranda azalır. İtme çubuğu boyu geometriden çıkar (135 / 106 mm; spec `pushrod_base_m` 100 / 160 mm) — kolon
  kontrolü §7'de as-built boyla tekrarlandı.
* **İtme çubuğu 6 × 1,5 mm** (yapı hesabı 6 × 1): CNC en küçük et 1,5 mm; kesit daha rijit.
* **Kanatçık menteşe pimi Ø3** (yapı hesabı C-HPIN Ø4): en dış menteşede (y 3,10) arka kiriş gövdesinin arkasında
  deri dudakları arasındaki serbest yükseklik 15,2 mm, Ø4 pimin kulak çapı (2 × 8,5 mm) 17 mm — sığmıyor. Ø3 kulak
  r 6,5 mm, kenar mesafesi e/D = 2,17; marjlar §7'de (MS ≥ 5).
* **Eyleyici bağlantısı:** yapı hesabı C-ACTMOUNT "servo çerçevesinde 4 × M4 gömülü burç" varsayar; ayrıntılı
  tasarımda eyleyici kapağa yapışık 7075 yatakta, kapak çerçeveye 4 × M3 somun plakalı (MS 2,2, §7).
* **Arka kiriş gövdesi:** veter oranı 0,709–0,713 (planform `rear_spar_frac` 0,72): oluk dairesi (burun yarıçapı +
  3,5 mm) gövdenin arkasında kalmalı; düz plan çizgisi (kalıp ve montaj kolaylığı).
* **Oluk ve uç boşlukları 3,5 mm** (layout hareketli yüzey açıklığı 3 mm + 0,5 mm tolerans payı).
* **Kaburga istasyonları** layout'ta tanımlı değildir; modül seçimi (aralık ≤ `structures.rib_pitch_m` 0,35 m;
  servo bölmesi sınırları kaburgadır). Menteşe braketleri kaburgalar arasındadır (cıvata somun plakaları gövdenin ön
  yüzünde, kaburga gövdesiyle çakışmaz); yük 10 katlı gövde pedinden C kesite yayılır.
* **`structgen.rib` yerine modül içi `rib_piece`:** sabit içe kaydırma kaplama kalınlık değişimini (rampa, katı
  bölgeler, kök takviyesi), başlık çentiklerini ve T flanşları temsil edemez; aynı `structgen` kesit yardımcıları
  (`section2d`, `largest`) üzerine kuruludur.
* Kök kaburga dış yüzü y 0,7117, uç kaburga dış yüzü y 3,5399 (LT-WING kutusunun 0,1 mm içi).

## 11. Açık konular

1. **Kütle bütçesi dağılımı:** kanat grubu −2,88 kg (toleransın 2,42 kg altı), şasi grubu fazla; orta kutu
   başlıklarının ve birleşim donanımının hangi grupta sayılacağı entegrasyonda `spec.mass` kalemlerinde düzeltilmeli.
2. **Yapı hesabı güncellemesi:** C-ACTMOUNT (kapak / çerçeve yük yolu), C-PUSHROD (as-built boylar 135 / 106 mm ve
   6 × 1,5 kesit), C-HINGE / C-HPIN kanatçık Ø3, menteşe braketi cıvataları ve gövde pedi `structures.py`'ye
   işlenmeli (el kontrolleri §7'de pozitif).
3. **Çırpınma (flutter):** kanatçık ve flap kütle dengesizdir; yüzey kütle dağılımı, menteşe rijitliği ve eyleyici
   boşluğu ile flutter analizi / yer titreşim testi (GVT) yapılmalı.
4. LT-WING ışık kablosu için uç kaburgada lastik geçmeli delik ve pito sondası kablosu / hortumunun direk ve D-burun
   içinden yolu modellenmedi (sistemler modülüyle birlikte).
5. Hafifletme seçenekleri: eyleyici yataklarının duvarlarında pencere (tahmini −0,1 kg/çift), kablo borusu 0,6 mm et
   (tahmini −0,16 kg/çift).
6. Birleşim contası (1,5 mm elastomer şerit, sarf malzemesi) ve kat kitabı / imalat çizimleri (zonlu UD kat bırakma
   sırası) çizim aşamasında.
7. Tam montaj denetimi (kaplama eldiven panelleri, sistemler, iniş takımı ile) entegrasyon aşamasındadır.

## 12. Doğrulama

```
python3 -m ucav250.analysis.checks --modules chassis,wing --focus YK250-WG,YK250-FC --no-write   # 0 ihlal
python3 -m unittest tests.test_ucav250_wing                                                      # 17 test
python3 -m ucav250.layout_build.build --check        # 0
python3 -m ucav250.analysis.layout_check --check     # 0
python3 -m ucav250.analysis.structures --check       # 0
python3 -m ucav250.analysis.sizing --check           # 0
python3 -m ucav250.blender.build --modules chassis,wing --previews --no-blend --out <dizin>
```
