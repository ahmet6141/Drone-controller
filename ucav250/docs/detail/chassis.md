# YK-250 HANÇER — Şasi (CHASSIS) ayrıntılı tasarımı

Üretici modül: `ucav250/design/chassis.py` (`register(reg, spec)`), test: `tests/test_ucav250_chassis.py`.
Sözleşme: `ucav250/ARCHITECTURE.md`; tek doğruluk kaynağı: `ucav250/spec.yaml` (`layout.stations`,
`layout.chassis.*`, `layout.part_numbers`, `structures.sizing`, `layups`, `processes`, `materials`). Modül başka bir
modülün geometrisini okumaz; tüm arayüzler `spec.layout` üzerinden tanımlıdır.

Kapsam yalnızca sivil EO/IR gözetleme platformunun birincil yapısıdır: şasi üzerinde silah, mühimmat, dış yük askı
noktası, pilon ya da bırakma mekanizması yoktur ve öngörülmemiştir.

## 1. Özet

| Büyüklük | Değer |
|---|---|
| Şasi parçası (grup `chassis`, YK250-CH-001 … -126) | 121 parça kimliği (85 parça numarası; L/R çiftleri ayna) |
| Şasinin ürettiği, `propulsion` grubunda numaralı parçalar | YK250-PR-088 ısı kalkanı, YK250-PR-094 mesafe parçaları (satın alma) |
| Bağlantı elemanı (`joints.py` ile, `hardware.py` geometrisi) | 219 (M3 14, M4 83, M5 44, M6 62, M8 8, kör perçin 8) |
| Hareketli eklem | 0 (şasi sabit yapıdır) |
| Kök parça | YK250-CH-001 orta kanat kutusu |
| Tasarım kuralı denetimleri | `checks --modules chassis,chassis --focus YK250-CH`: **0 ihlal** (mesh, statik girişim, kalınlık, kenar mesafesi, bağlanma) |
| Şasi grubu kütlesi (registry, `analysis/mass.py` yöntemi) | **20,75 kg** — bütçe tavanı 12,61 ± 0,37 kg (bkz. §8, açık konu) |

Önizlemeler (`python3 -m ucav250.blender.build --modules chassis --previews --no-blend`): ön, yan, üst, alt, iki izometrik
ve patlatılmış izometrik görünüş incelendi; yüzen parça, kopuk geometri ya da görünür girişim yoktur.

## 2. Yapı ve yük yolları

* **Orta kanat kutusu (CH-001, kök parça):** birleşimden birleşime sürekli UD ana ve arka kiriş başlıkları, sandviç
  kutu kapakları, çerçeve ayağı gövde takviyeleri, eldiven arka kiriş gövdesi ve yuva bağlantısı pedi. Kütlesi alt
  hacimlerden (UD başlıklar, kapak laminasyonu, PW gövdeler/takviyeler) ayrı ayrı hesaplanır (`ctbox_mass`). Kanat
  eğilmesi → başlıklar; kesme → kiriş gövdeleri → FS-MS / FS-RS ok (chevron) çerçeveleri → gövde.
* **Dış panel birleşimi:** CFRP çatal kulakları + yapıştırılmış 4130 burçlar (CH-053), arka kiriş yuva bağlantısı
  (CH-054, 2 × M5 Ti, arka kiriş gövde pedine dik), ok kırığı kaburgası (CH-055) ve iki 7075 kırık bağlantısı
  (CH-056/057, 5'er × M6 Ti). Gövde yanı / eldiven / birleşim kaburgaları (CH-050/051/052/058) eldiven kutusunu kapatır.
* **Çerçeveler (CH-002 … CH-013) ve motor bölmesi U halkası (CH-014/015):** sandviç gövde (layout kalınlığı) + deri
  hizasında katı T başlığı (CAP_T 1,6 mm, her iki yanda `flange_w`), deri bağlantı elemanlarının oturma alanıdır.
  FS-MS (8°) ve FS-RS (4,803°) ok çerçeveleri iki yarım + düz orta parça (|y| ≤ 1,5 mm) olarak kurulur. Her kesme ağzı,
  uzun kiriş geçişi ve üye kesişimi layout'tan; kenar çizgisi J kirişi çentikleri kirişin çerçeve kalınlığı (gövde) ve
  T başlığı genişliği (başlık) boyunca süpürülen kesitini 0,8 mm boşlukla içerir.
* **Uzun kirişler ve omurga:** kenar çizgisi J kirişleri (ön CH-020, arka CH-040; UD + PW, J 35 × 30 × 2,4) gövde yanı
  kaburgasına eklenir (ön ek 4 × M6 Ti, 18,5 mm adım; arka ek 2 × M4 Ti) ve her çerçevede 7075 kesme köşebendiyle
  (CH-042…048: çerçeveye 2 × M4 Ti, J gövdesine 1 × M3 Ti, J gövde normali boyunca) bağlanır; arka uç yangın perdesinde
  7075 uç bağlantısına (CH-041, 2 × M5 Ti) biter. Sırt şapka kirişleri (CH-032), yük bölmesi omurga kirişleri (CH-028),
  burun omurga duvarları (CH-021), talaşlı 7075 arka omurga (CH-033), ventral şerit (CH-039), kuyu orta duvarı (CH-036).
* **Güverteler / duvarlar / tabanlar:** aviyonik güverte (CH-022), taret bölmesi duvarları ve tavanı (CH-023/024),
  paraşüt bölmesi duvarları ve tabanı (CH-025/026), görev bölmesi tabanı + kesme ağzı kenar şapkaları (CH-027), ön yakıt
  tabanı (CH-029), ana takım kuyusu tavanı (CH-030), kuyu ön duvarı (CH-035), yük bölmesi arka duvarı (CH-034).
  Üye kenarları çerçeve T başlıklarına 0,3 mm yapıştırma boşluğuyla (MEM_GAP) yaklaşır.
* **Arayüz bağlantıları (7075-T651, talaşlı):** ana takım mafsalı (CH-070), yukarı kilit (CH-072), burun takımı mafsal
  blokları (CH-071), stabilatör düğümleri (CH-095), yangın perdesi köşe bağlantıları ve destek plakaları
  (CH-086/092/087/093), dikey / sabit kök / ventral kök bağlantıları (CH-096/098/099–101), paraşüt kayış U kulakları
  (CH-112/113) ve pul plakaları (CH-110/111), paraşüt kabı braketleri (CH-114/123), taret asansörü ray bağlantıları
  (CH-125/126).
* **Yangın perdesi yığını (FS3670):** CFRP sandviç perde (CH-013) + 7,6 mm hava boşluğu (her cıvatada AISI 304 ara
  borusu CH-090, 12 AISI 304 mesafe parçası PR-094) + 0,4 mm AISI 304 ısı kalkanı ve paslanmaz kenar köşebendi
  (PR-088, CS-VLA 1191 / standards FIRE-001). Kaynaklı 4130 boru motor yatağı (CH-085: 8 dikme 12,7 × 0,89, halka
  15,9 × 0,89, dört izolatör yuvası, dört 4 mm ayak plakası, her ayakta 2 × M8 12.9).
* **Yakıt bölmesi astarları (CH-115/116/117)** ve **ekipman tepsileri (CH-118…122).**

## 3. Parça listesi

Kütleler registry'den (mesh hacmi × etkin yoğunluk; CH-001 alt hacimlerden). L/R satırları çiftin toplamıdır.

| No | Parça (TR) | Malzeme | Süreç | Kalınlık / laminasyon | Adım | Adet | Kütle (kg) |
|---|---|---|---|---|---:|---:|---:|
| CH-001 | orta kanat kutusu (geçiş kutusu) | cfrp_ud_mtm45_as4 (+ PW) | prepreg_ooa_vacbag | UD başlıklar + ct_box_cover kapakları (zarf 6.8 mm) | 2 | 1 | 1.942 |
| CH-002 | burun konisi perdesi (FS0300) | cfrp_pw_mtm45_as4 | prepreg_ooa_vacbag | rib_panel | 6 | 1 | 0.024 |
| CH-003 | burun takımı ön perdesi (FS0600) | cfrp_pw_mtm45_as4 | prepreg_ooa_vacbag | rib_panel | 6 | 1 | 0.086 |
| CH-004 | taret bölmesi ön perdesi (FS1110) | cfrp_pw_mtm45_as4 | prepreg_ooa_vacbag | rib_panel | 6 | 1 | 0.198 |
| CH-005 | taret bölmesi arka perdesi (FS1330) | cfrp_pw_mtm45_as4 | prepreg_ooa_vacbag | rib_panel | 6 | 1 | 0.235 |
| CH-006 | paraşüt bölmesi ön perdesi (FS1490) | cfrp_pw_mtm45_as4 | prepreg_ooa_vacbag | rib_panel | 6 | 1 | 0.327 |
| CH-007 | paraşüt bölmesi arka perdesi / ön kayış bağlantısı (FS1810) | cfrp_pw_mtm45_as4 | prepreg_ooa_vacbag | rib_panel | 6 | 1 | 0.379 |
| CH-008 | ön yakıt bölmesi perdesi (FS-FUEL) | cfrp_pw_mtm45_as4 | prepreg_ooa_vacbag | rib_panel | 6 | 1 | 0.375 |
| CH-009 | ana kiriş çerçevesi (FS-MS, ok 8°) | cfrp_pw_mtm45_as4 | prepreg_ooa_vacbag | rib_panel | 3 | 1 | 0.267 |
| CH-010 | arka kiriş çerçevesi (FS-RS, ok 4,8°) | cfrp_pw_mtm45_as4 | prepreg_ooa_vacbag | rib_panel | 3 | 1 | 0.276 |
| CH-011 | arka ana takım çerçevesi / arka yakıt perdesi (FS-GEAR) | cfrp_pw_mtm45_as4 | prepreg_ooa_vacbag | rib_panel | 11 | 1 | 0.376 |
| CH-012 | kuyruk halka çerçevesi (FS3480) | cfrp_pw_mtm45_as4 | prepreg_ooa_vacbag | rib_panel | 11 | 1 | 0.126 |
| CH-013 | yangın perdesi (FS3670) | cfrp_pw_mtm45_as4 | prepreg_ooa_vacbag | rib_panel | 12 | 1 | 0.238 |
| CH-014 | motor bölmesi alt U halkası bacakları (FS3738) | al_2024_t3_sheet | sheet_metal_aluminium | 2.0 mm, r 12 mm | 13 | 1 | 0.115 |
| CH-015 | motor bölmesi alt U halkası alt parçası (7075 I kesit) | al_7075_t651_plate | cnc_milling_metal | 2.0 mm | 13 | 1 | 0.149 |
| CH-020 (L/R) | kenar çizgisi uzun kirişi, ön parça | cfrp_ud_mtm45_as4 | prepreg_ooa_vacbag | 2.4 mm | 9 | 2 | 0.836 |
| CH-021 (L/R) | burun omurga duvarı | cfrp_pw_mtm45_as4 | prepreg_ooa_vacbag | rib_panel | 7 | 2 | 0.272 |
| CH-022 | aviyonik güverte | cfrp_pw_mtm45_as4 | prepreg_ooa_vacbag | rib_panel | 7 | 1 | 0.245 |
| CH-023 (L/R) | taret bölmesi yan duvarı | cfrp_pw_mtm45_as4 | prepreg_ooa_vacbag | rib_panel | 8 | 2 | 0.150 |
| CH-024 | taret bölmesi tavanı | cfrp_pw_mtm45_as4 | prepreg_ooa_vacbag | rib_panel | 8 | 1 | 0.069 |
| CH-025 (L/R) | paraşüt bölmesi yan duvarı | cfrp_pw_mtm45_as4 | prepreg_ooa_vacbag | rib_panel | 8 | 2 | 0.294 |
| CH-026 | paraşüt bölmesi tabanı | cfrp_pw_mtm45_as4 | prepreg_ooa_vacbag | rib_panel | 8 | 1 | 0.156 |
| CH-027 | görev bölmesi tabanı (kesme ağzı kenar şapkaları) | cfrp_pw_mtm45_as4 | prepreg_ooa_vacbag | rib_panel | 8 | 1 | 0.301 |
| CH-028 (L/R) | yük bölmesi omurga kirişi | cfrp_pw_mtm45_as4 | prepreg_ooa_vacbag | rib_panel | 10 | 2 | 0.287 |
| CH-029 | ön yakıt bölmesi tabanı / yük bölmesi tavanı | cfrp_pw_mtm45_as4 | prepreg_ooa_vacbag | rib_panel | 10 | 1 | 0.380 |
| CH-030 | ana takım kuyusu tavanı | cfrp_pw_mtm45_as4 | prepreg_ooa_vacbag | fuel_floor_wellroof | 10 | 1 | 0.476 |
| CH-031 (L/R) | ana takım kirişi (dış kuyu duvarı, 8,3 mm insert ayağı) | cfrp_pw_mtm45_as4 | prepreg_ooa_vacbag | rib_panel | 10 | 2 | 0.040 |
| CH-032 (L/R) | sırt uzun kirişi (şapka 25 × 20) | cfrp_ud_mtm45_as4 | prepreg_ooa_vacbag | 2.0 mm | 11 | 2 | 0.278 |
| CH-033 | arka omurga kirişi | al_7075_t651_plate | cnc_milling_metal | 2.5 mm | 11 | 1 | 0.231 |
| CH-034 | faydalı yük bölmesi arka duvarı | cfrp_pw_mtm45_as4 | prepreg_ooa_vacbag | 1.6 mm | 10 | 1 | 0.133 |
| CH-035 | ana takım kuyusu ön duvarı | cfrp_pw_mtm45_as4 | prepreg_ooa_vacbag | 1.6 mm | 10 | 1 | 0.239 |
| CH-036 (L/R) | ana takım kuyusu orta duvarı | cfrp_pw_mtm45_as4 | prepreg_ooa_vacbag | 1.0 mm | 10 | 2 | 0.081 |
| CH-037 | sırt omurga kanalı (paraşüt kayış bağı) | cfrp_pw_mtm45_as4 | prepreg_ooa_vacbag | 0.8 mm (+16 kat ped) | 14 | 1 | 0.271 |
| CH-038 | burun takımı eşiği | cfrp_pw_mtm45_as4 | prepreg_ooa_vacbag | 2.4 mm | 7 | 1 | 0.003 |
| CH-039 | ventral omurga şeridi | cfrp_pw_mtm45_as4 | prepreg_ooa_vacbag | 1.6 mm | 11 | 1 | 0.056 |
| CH-040 (L/R) | kenar çizgisi uzun kirişi, arka parça | cfrp_ud_mtm45_as4 | prepreg_ooa_vacbag | 2.4 mm | 12 | 2 | 0.388 |
| CH-041 (L/R) | kenar çizgisi kirişi uç bağlantısı (yangın perdesi) | al_7075_t651_plate | cnc_milling_metal | 5.0 mm | 12 | 2 | 0.108 |
| CH-042…048 (L/R) | kenar çizgisi kesme köşebentleri (FS1110 … FS3480) | al_7075_t651_plate | cnc_milling_metal | 2.0 mm | 9 / 11 | 14 | 0.110 |
| CH-050 (L/R) | gövde yanı kaburgası | cfrp_pw_mtm45_as4 | prepreg_ooa_vacbag | rib_panel | 4 | 2 | 0.229 |
| CH-051 (L/R) | eldiven kaburgası, burun parçası | cfrp_pw_mtm45_as4 | prepreg_ooa_vacbag | rib_panel | 4 | 2 | 0.034 |
| CH-052 (L/R) | birleşim kaburgası (orta kesit) | cfrp_pw_mtm45_as4 | prepreg_ooa_vacbag | rib_panel | 4 | 2 | 0.077 |
| CH-053 (L/R) | dış panel birleşim çatalı (kulaklar + 4130 burçlar) | cfrp_pw_mtm45_as4 | prepreg_ooa_vacbag | 1.6 mm | 5 | 2 | 0.653 |
| CH-054 (L/R) | arka kiriş yuva bağlantısı | al_7075_t651_plate | cnc_milling_metal | 4.0 mm | 5 | 2 | 0.094 |
| CH-055 | orta kutu orta hat (ok kırığı) kaburgası | cfrp_pw_mtm45_as4 | prepreg_ooa_vacbag | rib_panel | 2 | 1 | 0.032 |
| CH-056 / 057 | orta kutu kırık bağlantısı, üst / alt ana başlık | al_7075_t651_plate | cnc_milling_metal | 2.0 mm | 2 | 2 | 0.092 |
| CH-058 (L/R) | eldiven kaburgası, kutu parçası | cfrp_pw_mtm45_as4 | prepreg_ooa_vacbag | rib_panel | 4 | 2 | 0.063 |
| CH-070 (L/R) | ana takım mafsal bağlantısı | al_7075_t651_plate | cnc_milling_metal | 6.5 mm flanş, 12 mm kulak gövdesi | 10 | 2 | 1.736 |
| CH-071 | burun takımı mafsal burç blokları (çift) | al_7075_t651_plate | cnc_milling_metal | 6.0 mm | 7 | 1 | 0.090 |
| CH-072 (L/R) | ana takım yukarı kilit kancası bağlantısı | al_7075_t651_plate | cnc_milling_metal | 5.0 mm | 10 | 2 | 0.094 |
| CH-085 | motor yatağı (kaynaklı 4130 boru) | steel_4130_n | tig_welding_4130 | 0.89 mm boru, 4 mm ayak | 12 | 1 | 1.022 |
| CH-086 (L/R) | yangın perdesi üst köşe bağlantısı (arka parça) | al_7075_t651_plate | cnc_milling_metal | 6.0 mm | 12 | 2 | 0.265 |
| CH-087 (L/R) | alt motor ayağı pedi | al_7075_t651_plate | cnc_milling_metal | 10.0 mm | 12 | 2 | 0.152 |
| CH-090 (L/R) | yangın perdesi ara boruları | ss_304_annealed | cnc_milling_metal | 2.0 mm | 12 | 2 | 0.124 |
| CH-092 (L/R) | yangın perdesi üst destek plakası | al_7075_t651_plate | cnc_milling_metal | 5.0 mm | 12 | 2 | 0.186 |
| CH-093 (L/R) | alt motor bağlantısı destek plakası | al_7075_t651_plate | cnc_milling_metal | 5.0 mm | 12 | 2 | 0.076 |
| CH-095 (L/R) | stabilatör düğüm bağlantısı | al_7075_t651_plate | cnc_milling_metal | 7.0 mm yanak | 13 | 2 | 0.989 |
| CH-096 (L/R) | dikey ön kiriş kök bağlantısı | al_7075_t651_plate | cnc_milling_metal | 6.0 mm | 11 | 2 | 0.212 |
| CH-098 (L/R) | sabit kök parçası ön kiriş bağlantısı | al_7075_t651_plate | cnc_milling_metal | 4.0 mm | 11 | 2 | 0.138 |
| CH-099/100/101 | ventral kök bağlantıları 1–3 | al_7075_t651_plate | cnc_milling_metal | 5.0 mm | 11 | 3 | 0.103 |
| CH-102 (L/R) | arka omurga / U halkası köşebendi | al_7075_t651_plate | cnc_milling_metal | 2.0 mm | 13 | 2 | 0.011 |
| CH-110 / 111 | kayış bağlantısı pul plakaları | al_7075_t651_plate | cnc_milling_metal | 3.0 mm | 14 | 2 | 0.032 |
| CH-112 / 113 | paraşüt ön / arka kayış U kulağı | al_7075_t651_plate | cnc_milling_metal | 5.0 mm | 14 | 2 | 0.117 |
| CH-114 / 123 (L/R) | paraşüt kabı kayış braketleri | al_7075_t651_plate | cnc_milling_metal | 2.5 mm | 14 | 4 | 0.078 |
| CH-115 | ön yakıt bölmesi astarı | cfrp_pw_mtm45_as4 | prepreg_ooa_vacbag | 0.6 mm (3 kat PW) | 14 | 1 | 0.621 |
| CH-116 | eyer yakıt bölmesi astarı | cfrp_pw_mtm45_as4 | prepreg_ooa_vacbag | 0.6 mm | 14 | 1 | 0.367 |
| CH-117 | arka yakıt bölmesi astarı | cfrp_pw_mtm45_as4 | prepreg_ooa_vacbag | 0.6 mm | 14 | 1 | 0.568 |
| CH-118 / 122 | aviyonik yan bölme tepsileri (TR-SIDEBAY-L / -R) | cfrp_pw_mtm45_as4 | prepreg_ooa_vacbag | 3.0 mm | 14 | 2 | 0.331 |
| CH-119 | ön bölme tepsisi (TR-FWDBAY) | cfrp_pw_mtm45_as4 | prepreg_ooa_vacbag | 3.0 mm | 14 | 1 | 0.158 |
| CH-120 | görev bölmesi sökülebilir tepsisi (TR-MISSION) | cfrp_pw_mtm45_as4 | prepreg_ooa_vacbag | 3.0 mm | 14 | 1 | 0.350 |
| CH-121 | arka teçhizat bölmesi tepsisi (TR-AFTBAY) | al_6061_t6_sheet | sheet_metal_aluminium | 2.0 mm, r 6 mm | 14 | 1 | 0.477 |
| CH-125 / 126 | taret asansörü ray bağlantıları (RAIL-FR / RAIL-AL) | al_7075_t651_plate | cnc_milling_metal | 3.0 mm | 14 | 2 | 0.656 |
| PR-088 | yangın perdesi ısı kalkanı (AISI 304 0,4 mm + kenar köşebendi) | ss_304_annealed | sheet_metal_steel | 0.4 mm | 12 | 1 | 0.580 |
| PR-094 | kalkan mesafe parçaları (12 × AISI 304, satın alma) | ss_304_annealed | purchased | — | 12 | 1 | 0.048 |

Her parçada `material`, `process`, `thickness` veya `layup`, `parent`, `step`, `explode` ve `contacts` doludur (test:
`test_part_contract`). Sağ (−R) parçalar +y'de modellenip kayıt sonunda aynalanır; ayna parçalar delikleri ve bağlantı
elemanlarıyla birebir simetriktir (`test_mirror_pairs`).

## 4. İmalat

**OOA prepreg + vakum torbası, fırın kürü (`prepreg_ooa_vacbag`, en az 0,6 mm, kenar mesafesi ≥ 2,5 D):**
çerçeveler, kaburgalar, güverteler ve duvarlar `layups` laminasyonlarıyla (çoğu `rib_panel` sandviç) erkek kalıpta
serilir; T başlıkları 8 kat PW katı kenar bandı olarak gövdeyle birlikte kürlenir, bağlantı elemanı çizgilerinde çekirdek
katı alana (16 kat, `FRAME_LAND` 3,2 mm) rampalanır (modelde sandviç tam kalınlıkta, aynı zarf). Kesim ve delikler CNC
ile, ISO 2768-mK; delikler ISO 273 orta seri. J kirişleri ve şapka kesitleri dişi/erkek kalıplı tek parça; kenar
çizgisi J'si ön ve arka iki parça (kalıp boyu ve gövde yanı kaburgasında ek). Yakıt astarları 3 kat PW (bkz. §9),
kürden sonra çerçevelere, tabana ve omurga kirişlerine yapıştırılır (0,2 mm yapıştırma çizgisi).

**CNC frezeleme (7075-T651 levha, ISO 2768-mK, oturma ve burç yuvaları ISO 286):** tüm bağlantılar tek bağlamada
işlenebilecek prizmatik formdadır; en küçük et kalınlığı ≥ 1,5 mm, iç köşe yarıçapı ≥ 3 mm (çizim notu). Ana takım
mafsalının iki kulağı 26 mm burç göbeği + 12 mm gövde (layout kulak hesabı t_m) olarak, üst plaka kulaklar arasında
alttan 5 mm'ye cep işlenir. Ok çerçeve yüzlerine dik cıvatalar için arka paraşüt kulağında cıvata eksenine dik punta
yüzeyi (spot face) işlenir. 4130 burçlar CH-053 kulaklarına yapıştırılır, 20 H7 / 16 H7 delikler son işlemle açılır.

**Sac (2024-T3 U halkası bacakları 2,0 mm, iç büküm yarıçapı 12 mm = 6 t; 6061-T6 arka tepsi 2,0 mm, r 6 mm = 3 t;
AISI 304 kalkan 0,4 mm):** lazer/su jeti kesim, abkant büküm, `processes.sheet_metal_aluminium` /
`sheet_metal_steel` kurallarına göre. Kalkan paslanmaz kenar köşebendine perçinlenir (kaporta oturma alanı).

**TIG kaynağı (4130 N boru):** motor yatağı kaynak fikstüründe; ayak plakaları ve izolatör yuvaları kaynaktan sonra
ISO 2768-m'ye işlenir, normalize/gerilim giderme ve korozyon koruması.

## 5. Montaj (spec.assembly adımları)

| Adım | Şasi işi | Başlıca bağlantılar |
|---:|---|---|
| 2 | Orta kanat kutusu tezgâha (kök parça), ok kırığı kaburgası, üst / alt kırık bağlantıları | 10 × M6 Ti (pul başta), 18,5 mm adım |
| 3 | FS-MS ve FS-RS ok çerçeveleri kutuya | çerçeve başına 2 × 4 × M6 Ti somun plakalı (astar + çerçeve + takviye, kiriş gövdesine dik) |
| 4 | Gövde yanı, eldiven ve birleşim kaburgaları | yapıştırma + kutu bağlantıları |
| 5 | CFRP çatallar (4130 burçlar), arka kiriş yuva bağlantısı | 2 × 2 × M5 Ti (arka kiriş gövdesine dik) |
| 6 | Ön gövde çerçeveleri FS0300 … FS-FUEL | yapıştırma, üye bağlantıları adım 7–10 |
| 7 | Burun omurga duvarları, aviyonik güverte, burun takımı eşiği ve mafsal blokları | 6 × M6 12.9 (omurga duvarından) |
| 8 | Taret, paraşüt ve görev bölmesi duvarları / tabanları | yapıştırma (0,3 mm), insertler |
| 9 | Kenar çizgisi ön J kirişleri, FS1110 … FS-FUEL kesme köşebentleri | ön ek 4 × M6 Ti; köşebent 2 × M4 Ti + 1 × M3 Ti |
| 10 | Yük bölmesi omurga kirişleri, ön yakıt tabanı, kuyu tavanı ve duvarları, ana takım mafsalı ve yukarı kilit | mafsal: 5 × M6 12.9 kirişteki geçme insertlere + 4 × M6 12.9 kuyu tavanına somun plakalı; kilit 4 × M5 somun plakalı |
| 11 | FS-GEAR, FS3480, sırt kirişleri, arka omurga, ventral şerit, dikey / sabit kök / ventral bağlantılar | dikey 2 × M6, sabit kök 2 × M5, ventral 2 × M4 kılavuz dişli |
| 12 | Yangın perdesi yığını (sandviç, ara boruları, mesafe parçaları, kalkan), köşe / alt ayak bağlantıları, motor yatağı, arka J kirişleri ve uç bağlantıları | motor 8 × M8 12.9 (NORD-LOCK çifti), düğüm 7 × M5, arka ek 2 × M4 Ti, uç bağlantısı 2 × M5 Ti, sırt eki 2 × M4 Ti |
| 13 | Stabilatör düğümleri, FS3738 alt U halkası ve arka omurga köşebentleri | U halkası 2 × 3 × M5, köşebent 2 × 2 × M5 |
| 14 | Sırt kanalı ve paraşüt kulakları, pul plakaları, kap braketleri (gömme kör perçin), taret ray bağlantıları, yakıt astarları, tepsiler | kulak başına 4 × M4 + 2 × M4; tepsiler M4 A2-70 insertlere / somun plakalarına |

Bütün cıvatalar `joints.py` ile kurulur: yığın kalınlığı geometri üzerinde ölçülür, boşluk deliği (ISO 273 orta) her
parçaya açılır, boy ISO standart boyuna yuvarlanır, sıkma momenti katalogdan; insertli bağlantılarda insert yuvası,
kılavuz dişlilerde anma çapında kozmetik diş açılır.

## 6. Arayüzler

| Modül | Arayüz (spec.layout) | Şasi tarafı |
|---|---|---|
| kanat | `chassis.wing_joint` (çatal, pim, yuva) | CH-053 kulak + 4130 burç, CH-054 yuva bağlantısı, CH-050 gövde yanı kaburgası (eldiven arka kenar bölmesi ~20 mm derin) |
| kaplama (shell) | çerçeve T başlıkları, J kirişi deri flanşları, yük kapağı ayağı (`KEEL_LAND_Y1`), sırt kanalı flanşları, kaporta ayağı (kalkan kenar köşebendi) | deri bağlantı elemanlarının oturma alanları; kaplama modülü kendi deliklerini açar |
| iniş takımı | F-TRUNNION, F-UPLOCK, F-NG-PIVOT, ACT-MLG-EMA zarfı | 20 H7 burç yuvaları, EMA zarfı kesilmiş, mafsal kulakları arasında bacak salınımı için boşluk |
| itki | KO-ENGINE, `engine_mount`, yangın perdesi kesme ağızları | CH-085 yatağı, izolatör yuvaları (Ø41), C-DUCT / C-FW-* geçişleri, PR-088 / PR-094 |
| kuyruk | F-FIN-*, F-STUB-*, F-VENTRAL-*, F-SPINDLE-NODE | dikey arka kiriş kulağı yuvası (x 3,680–3,688, y ≥ 0,140) boş bırakıldı; 61805 rulman yuvası |
| yakıt | `fuel_supports`, `fuel_lines` | astarlar, sızdırmaz boru / kablo geçişleri (`penetration_cuts`) |
| faydalı yük / görev | `turret_elevator.rails`, `trays` | ray oturma yüzeyleri (M3), tepsi insertleri |
| sistemler | `systems.harness`, `systems.actuators` | kablo demeti geçişleri, ön kapı aktüatörü için yan tepsi rahatlaması |
| paraşüt | F-RISER-*, kap kayışları | U kulakları, pul plakaları, gömme perçinli braketler (kap boşluğuna kafa taşmaz) |

## 7. Bağlantı elemanları

| Ölçü | Somun (ISO 7040 + pul) | Somun plakası | Geçme insert | Kılavuz dişli | Toplam |
|---|---:|---:|---:|---:|---:|
| M3 (Ti) | 14 | – | – | – | 14 |
| M4 | 48 | 8 | 22 | 5 | 83 |
| M5 | 36 | 8 | – | – | 44 |
| M6 | 28 | 24 | 10 | – | 62 |
| M8 12.9 | 8 | – | – | – | 8 |
| Kör perçin d 4 (gömme) | – | – | – | – | 8 |
| **Toplam** | | | | | **219** |

Kenar mesafesi her parçada metalde ≥ 2 D, kompozitte ≥ 2,5 D; komşu delik kenarı da kenar sayılır (adımlar buna göre
18,5 / 19 / 21 / 25 mm). Eğik yüzlerden geçen cıvatalar (ok çerçeveler, J gövdesi, arka kiriş gövdesi) yüze dik
yönlendirilmiştir. Bağlantı elemanlarının toplam kütlesi 1,50 kg'dır (grup `hardware`).

## 8. Kütle ve bütçe

Registry kütlesi (şasi grubu): **20,75 kg**; bütçe tavanı **12,61 ± 0,37 kg** → **+8,14 kg aşım** (`analysis/mass.py`
bu grup için tavan aşımı bildirir). Ayrıca şasinin ürettiği PR-088 + PR-094 (0,63 kg) itki grubundaki soğutma/yangın
perdesi kalemindedir; şasi bağlantı elemanları (1,50 kg) uçak geneli `hardware` tavanının (1,51 kg) neredeyse tamamıdır.

`spec.mass.items` şasi kalemlerine göre dağılım (eşleme bu belgenin; "frames" kalemine kapak çerçeveleri kalemi
eklenmiştir çünkü kapak ayakları çerçeve/güverte parçalarının içindedir):

| Bütçe kalemi | Bütçe (kg) | Ayrıntılı tasarım (kg) | Fark (kg) | Parçalar |
|---|---:|---:|---:|---|
| keel_beams_longerons | 1.533 | 2.661 | +1.128 | CH-020/040/041/042–048/032/028/021/033/039/036/038/102 |
| frames_bulkheads + hatch_frames_quick_access_fasteners | 3.571 | 3.482 | −0.089 | CH-002–015, 090, 092 |
| wing_carry_through_box_fittings | 2.218 | 3.217 | +0.999 | CH-001, 050–058 |
| main_gear_frame_trunnions_side_braces | 0.630 | 2.110 | +1.480 | CH-070, 072, 031, 035 |
| nose_gear_trunnion_fitting | 0.193 | 0.090 | −0.103 | CH-071 |
| engine_mount_4130 (+ yangın perdesi ayak bağlantıları) | 0.630 | 1.516 | +0.886 | CH-085, 086, 087, 093 |
| parachute_attach_fitting | 0.468 | 0.499 | +0.030 | CH-037, 110–114, 123 |
| turret_bay_frame_guides | 0.420 | 0.875 | +0.455 | CH-023, 024, 125, 126 |
| floors_trays_rails | 1.530 | 3.301 | +1.771 | CH-022, 025–027, 029, 030, 034, 118–122 |
| fuel_bay_liners_supports | 0.472 | 1.555 | +1.083 | CH-115–117 |
| stabilator_spindle_bearing_housings | 0.626 | 0.989 | +0.363 | CH-095 |
| fin_ventral_root_fittings | 0.315 | 0.453 | +0.138 | CH-096, 098–101 |
| **Toplam** | **12.606** | **20.747** | **+8.140** | |

Aşımın nedenleri: (1) layout'un verdiği zarflar ve laminasyonlarla modellenen bütün parçalar kavram kalemlerinden
ağırdır (çerçeveler hariç); (2) ana takım mafsalı (layout: 26 mm kulaklar, 20 mm üst plaka) — kulaklar 12 mm gövdeye,
üst plaka 5 mm'ye ceplenerek 2,23'ten 1,74 kg'a indirildi; (3) yakıt astarları tüm küveti kaplar ve prepreg süreç
alt sınırı nedeniyle 0,4 yerine 0,6 mm'dir; (4) tepsiler layout kalınlığında (CFRP 3 mm katı, 6061 2 mm);
(5) motor yatağı + ayak bağlantıları kavramda yalnız boru kafes olarak sayılmış. Düzeltme sistem düzeyinde bir karardır
(bkz. §10, açık konu 1).

## 9. Layout'tan sapmalar (gerekçeli)

* Yakıt astarları 3 kat PW 0,6 mm (layout 2 kat 0,4 mm < `prepreg_ooa_vacbag` en az 0,6 mm); FS-FUEL / FS-GEAR
  köşebent cıvatalarının somunları çevresinde astar 6 mm boşaltıldı (somun çerçeve gövdesine oturur).
* Kırık bağlantısı cıvata adımı 18,5 mm (layout 18), motor ayağı adımı 25 mm (layout 24), dikey ön kiriş tabanı
  43 mm (layout 42), düğüm B4/B6 1 mm dışa: kompozitte 2,5 D + komşu delik yarıçapı.
* Üst motor ayakları: dikey kulak yuvası (y ≥ 0,140) ile C-DUCT kenarı (y 0,090, M8 için 20 mm) arasında yatay 25 mm
  çift sığmadığından cıvatalar dikey çift (y 0,116, z 0,268 / 0,293); köşe tabanı ve ön plaka y < 0,1315'te z 0,250'ye
  uzatıldı; ayak plakası 33 × 58 mm, dikme bloğu iki pulun arasında. Köşe bağlantısı B5/B6 ek cıvataları adım
  çakışması nedeniyle kaldırıldı (ek, sırt eki ve düğüm cıvatalarıyla taşınır).
* Ana takım mafsalı: kirişe giden cıvatalar z −0,092 / −0,1125 (layout −0,085 / −0,115), B4 5 mm geriye ve arka flanş
  2 mm uzatıldı (insert yuvası Ø11 ile 2,5 D); kuyu tavanı cıvataları kulakların önüne/arkasına, sıralar 29 mm
  (kubbeli somun plakası 28 mm); yukarı kilit sıraları 26 mm (layout 20).
* Burun takımı mafsalı: B2/B5 (0,677, −0,1265), arka bloklar 8 mm uzun (omurga duvarının deri kenarına 2,5 D).
* Paraşüt kulakları: çerçeve cıvataları M4 (layout M5; kafalar taban şeritlerinden uzak), flanş ±23,5 mm ve z 0,1605'e
  kadar; arka kulak cıvataları FS-RS ok yüzüne dik + punta yüzeyi.
* Ventral kök bağlantıları kirişe 2 × M4 kılavuz dişli (layout 1 × M6 dikey: kulak yuvası 8 mm et bırakır).
* Kenar çizgisi arka eki 2 × M4 Ti (layout 4 × M6: eldiven arka kenar bölmesi gövde yanı kaburgasında ~20 mm derin).
* Kenar çizgisi kesme köşebentlerinin J tarafı M3 Ti (J gövdesi deri hizasında kırpılır); J yol kırıklarından ≥ 4,5 mm.
* Kenar çizgisi ön parçası FS-MS T başlığının 0,5 mm önünde, arka parça yangın perdesi uç bağlantısının önünde biter.
* Birleşim kaburgası yuva bağlantısının üzerine kadar uzatıldı (x 2,905; layout 2,8719).

## 10. Açık konular

1. **Kütle:** şasi 20,75 kg, tavan 12,61 kg (+8,14 kg). Sistem düzeyinde yeniden dağıtım (MTOM 149,9 kg sabit) veya
   aşağıdaki hafifletmeler gerekir: düğüm kolu ve yanağında cepler (~0,1–0,2 kg), ray bağlantılarında duvar tarafı cepleri
   (~0,2 kg), mafsal tavan uzantılarında punta cepleri (~0,1 kg), tepsilerde sandviç (layout değişikliği), astarların
   yalnız temas bölgelerine indirilmesi (layout değişikliği). Kavram kalemleri ayrıntılı geometriye göre güncellenmeli.
2. `hardware` tavanı (1,51 kg) yalnız şasi bağlantılarıyla dolmuştur; diğer modüllerin bağlantıları eklenince yeniden
   dağıtılmalıdır.
3. Ana takım: bacak salınımı, EMA ve kilit cıvatası tutucusu iniş takımı modülüyle birlikte kontrol edilmeli (mafsal
   kulakları arasında üst plaka cebi bacak yoluna açılmaz; kilit kulağı modellenmedi, takım tedarikçisi gereği).
4. Arka paraşüt kulağı somunları eyer yakıt bölmesi zarfına yakındır; yakıt modülü hücre geometrisiyle doğrulanmalı.
5. Üst motor ayağı dikey çifti için yapısal ayak/plaka hesabı (`structures` E-MOUNT kalemleri) yeni geometriyle
   güncellenmeli; tork ve NORD-LOCK pul zarfı tahmindir.
6. Ventral bağlantılardaki M4 kılavuz dişlerin (7,5 mm diş) ve kenar çizgisi arka ekinin (2 × M4) kesme taşıma payı
   yapı hesaplarına işlenmeli.
7. Astarların kesme ağızları ve somun boşaltmaları yakıt hücresi (YK250-FU-570…572) ile örtüşme açısından yakıt
   modülünde kontrol edilmeli.
8. Önizleme yalnız şasi + bağlantılarla yapıldı; kaplama, iniş takımı ve itki ile birlikte tam montaj denetimi
   entegrasyon aşamasındadır.

## 11. Çerçeve (framework) değişiklikleri — diğer modülleri de etkiler

* `analysis/checks.py`: kenar mesafesinde (a) geçme insertin alıcı parçasındaki yuva yarıçapı (`_insert_bore_radius`),
  (b) orta düzlem yalnız sıkıştırılan malzeme aralıklarından (somunun ötesindeki ikinci duvar — J kirişi, U kanal —
  ölçümü boşluğa kaydırmaz). Kurallar gevşetilmedi; yanlış yerde ölçülen değerler düzeltildi.
* `design/joints.py`: kılavuz dişli delik anma çapında (gövde parçayla girişmez); kısa geçme insert boyu ("L x mm").
* `design/hardware.py`: insert boyu nottan okunur; bağlantı elemanı malzemesi yoğunluğu olan spec malzemesine
  eşlenir (12.9 → `steel_4130_n`, A2/A4 → `ss_304_annealed`, Ti → `ti_6al_4v_annealed_sheet`; ayrı `fastener_*`
  kayıtları eklenirse onlar kullanılır) — bu olmadan kütle ve önizleme `KeyError` veriyordu.
* `core/parts.py`: `layup_props` sandviçin iç yüz katlarını (`inner_plies`) da sayar.
* `blender/build.py`: `display.preview_states` içindeki yalnız açıklama olan (sözlük olmayan) girdiler atlanır.
* `spec.yaml` ve `data/research/materials.yaml`: `processes.sheet_metal_steel` (AC 43.13-1B; AISI 304 kalkan ve kenar
  köşebendi). `layout_build --check` bayt-özdeş kaldı.

## 12. Doğrulama

```
python3 -m ucav250.analysis.checks --modules chassis,chassis --focus YK250-CH --no-write   # 0 ihlal
python3 -m unittest tests.test_ucav250_chassis                                               # 14 test
python3 -m ucav250.layout_build.build --check        # 0
python3 -m ucav250.analysis.layout_check --check     # 0
python3 -m ucav250.analysis.structures --check       # 0
python3 -m ucav250.analysis.sizing --check           # 0
python3 -m ucav250.blender.build --modules chassis --previews --no-blend --out <dizin>
```
