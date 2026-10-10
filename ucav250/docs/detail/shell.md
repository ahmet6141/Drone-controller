# YK-250 HANÇER — Gövde kaplamaları (SHELL) ayrıntılı tasarımı

Üretici modül: `ucav250/design/shell.py` (`register(reg, spec)`), test: `tests/test_ucav250_shell.py`.
Sözleşme: `ucav250/ARCHITECTURE.md`; tek doğruluk kaynağı: `ucav250/spec.yaml` (`layout.shell.panels`,
`layout.shell.rules`, `layout.shell.root_cut_lines`, `layout.shell.wing_root_fairing`, `layout.stations`,
`layout.chassis.members` / `fittings`, `layout.mechanisms.joints` / `door_outlines`, `layout.clearances`,
`layout.part_numbers`, `fuselage`, `wing.sections`, `payload.turret`, `layups`, `materials`, `processes`,
`assembly.steps`, `mass.budget`). Modül başka bir modülün geometrisini yerleştirmek için okumaz: bütün arayüzler
`spec.layout` üzerindendir. Bağlantı elemanı delikleri yalnızca **ispat** için şasi ve kendi kaplamalarına bakar (delik
gerçekten katı kara malzemesini sıkıyor mu, kenar mesafesi tutuyor mu, somun plakası arkadaki yapıya çarpıyor mu).

Kapsam yalnızca sivil EO/IR gözetleme platformunun gövde dış kabuğudur: kaplamalarda silah, mühimmat, dış yük taşıma ya
da bırakma düzeneği yoktur ve öngörülmemiştir.

## 1. Özet

| Büyüklük | Değer |
|---|---|
| Parça kimliği | 60 (grup `shell`, YK250-SH-350 … -424 ve 450 … 452; aralık `layout.part_numbers.shell` 350–499) |
| Parça numarası | 45 (sağ taraf +y'de modellenir, sol `mirror_part` ile aynalanır: 15 çift) |
| Layout panelleri | `layout.shell.panels` içindeki 43 panelin hepsi + kanat kökü birleşim kaplaması (SH-424, `wing_root_fairing`) + yakıt kapağı menteşe pimi (SH-386, satın alma) |
| Bağlantı elemanı (`joints.py`, geometri `hardware.py`) | **759**: 571 × ISO 7380 M4 + yüzer somun plakası, 54 × ISO 7380 M3 + somun plakası, 106 × Camloc 4002 (2600 yuva), 28 × kör perçin (3,2 / 4 mm) |
| Hareketli eklem | 2: `para_hatch` (prizmatik, `layout.mechanisms.joints`), `refuel_door` (döner, `P-REFUEL.hinge`, 0…100°) |
| Tasarım kuralı denetimleri | `checks --modules chassis,shell --focus YK250-SH --no-write`: **0 ihlal** (mesh, statik, süpürme, açıklık, kalınlık, bağlantı elemanı, temas, bağlanma) |
| Kanat ve kuyrukla birlikte | `checks --modules chassis,wing,tail,shell --focus YK250-SH`: **0 ihlal** |
| Kabuk grubu kütlesi (registry) | **10,561 kg** — bütçe 10,48 ± 0,31 kg (+0,081 kg, bant içinde; §9) |
| Kabuk bağlantı elemanları (grup `hardware`) | 3,34 kg (bütçe uyuşmazlığı, §11) |

Önizlemeler (`python3 -m ucav250.blender.build --modules chassis,shell --previews --no-blend`): ön, yan, üst ve
izometrik görünüşler incelendi. Kabuk kapalıdır; panel ayrım çizgileri çerçeve gövde çizgilerinde ve kenar çizgisindedir;
vida başları sıralar halinde görünür; açıklıklar yalnızca başka modüllerin parçalarının oturduğu yerlerdedir (dikey,
stabilatör kökü ve ventral kök kesik çizgileri, takım kapağı açıklıkları, dış kanat birleşim düzlemi). Yüzen parça ya da
görünür girişim yoktur.

## 2. Yapı (ne üretilir)

**Kaplama yapısı.** Her kaplama, birleşik dış kalıp çizgisinin (OML: gövde `oml.Fuselage` + kanat kökü birleşim kaplaması
= kanat kök profili, `wing.sections[0]`, y 0,30'a kadar içe doğru uzatılmış) içine, kesit düzleminde katman kalınlığı
kadar kaydırılmış bir kabuktur — şasi çerçevelerinin `structgen.fuselage_section2d` içe kaydırma kuralıyla aynı (kenar
çizgisinde gönyeli köşe). Gövde yapısal kaplamaları `layups.shell_secondary` (0,4/5/0,4 mm, 5,80 mm), RF pencereleri
`gfrp_7781_mtm45` (2 kat + 5 mm ROHACELL + 2 kat, 6,02 mm, karbon yok), LERX/eldiven `wing_skin_primary` (6,00 mm;
ana–arka başlık arası `wing_box_skin_upper` 7,00 mm, ilk LERX bölmesi `lerx_skin_upper_root` 8,00 mm),
P-COWL-UPS 0,8 mm 6061-T6 sac. Kaplamalar kenar çizgisi düzleminde (`z = zc(x)`) üst / alt diye bölünür; tam saran
paneller (burun konisi, ön gövde kaplaması, kaporta, kök şeritleri) iki yarıyı da kapsar.

**Şasi karalarına oturma.** Çerçeve T başlıkları, uzun kiriş flanşları (sırt kanalı, paraşüt duvarı, omurga kirişi,
arka omurga, ventral omurga, sırt uzun kirişi şapka flanşları) ve kenar çizgisi J flanşı layout'tan plan izleri olarak
alınır; kaplamanın iç yüzü bu izlerde 6,3 mm'ye kadar **kara dolgusuyla** kalınlaştırılır (6,5 mm kaplama çizgisinde
0,2 mm sıvı şim payı). Kenar çizgisi köşesinde ±4 mm dolgu yoktur ve iç yüz kenar çizgisi boyunca 1,2 mm boşaltılmıştır
(J'nin gönyeli köşesi istasyonlar arasında kaplama halkalarından farklı sarkar). Orta kanat kutusunun üstünde ve altında
kaplama, kutu kapaklarının tasarlandığı 1,0 mm katı laminata iner (kutu dış yüzü OML'nin 1 mm altında,
`M-CTBOX.oml_clearance_m`, 0,25 mm yapıştırma çizgisi); alt kapak ucunda gövde yan duvarı kapağın dışından geçer.

**Bindirmeli karalar (joggle).** Şasi karası üzerinde olmayan her sökülebilir panel kenarının altında, komşu sabit
kaplamaya entegre 1,6 mm katı laminat kara (paneli altında 28 mm, kendi kaplaması altında 20 mm; `layout.shell.rules`
"kara ≥ 25 mm"); şasi karalarından 1,5 mm geride durur. Panel ile kara arasında 0,2 mm conta çizgisi, panel aralığı
1,0 mm (paraşüt kapağı 0,2 mm, §7).

**Panel listesi** (§3): burun konisi (radom, GFRP), ön gövde kaplaması + ön bölme kapağı, üst/alt burun kaplaması +
aviyonik kapağı (GFRP RF penceresi) + iki yan bölme kapağı, üst/alt orta kaplama + taret açıklık halkası (2,0 mm katı
CFRP, HD59 açıklığı = bilye yarıçapı + radyal boşluk = 78,5 mm) + iki taret kapağı tahrik erişim kapağı, paraşüt kapağı
+ çevre ve alt kaplama, görev bölmesi üst/alt kaplaması + GNSS 2 penceresi + görev bölmesi kapağı + yırtılır kayış
örtüsü, üç çift yakıt bölmesi kapağı + üst/alt orta gövde kaplaması + faydalı yük kapağı + yakıt ikmal kapağı
(menteşeli, pim SH-386), alt/üst arka kaplama + arka teçhizat kapağı + iki stabilatör eyleyici kapağı + sırt soğutma
girişi dudağı + dikey kök örtüleri + ventral kök şeridi + stabilatör kökü fileto şeritleri, kaporta (üst orta parça,
iki üst yan alüminyum parça, iki alt yarım, iki sabit dikey kök kaplaması), LERX/eldiven üst/alt kaplamaları, kanat
birleşim erişim kapağı, arka pim erişim kapağı (bayonet), kanat kökü birleşim kaplaması.

## 3. Parça listesi

Kütle: registry (`reg.mass`: katman alan kütlesi / kalınlık × hacim; GFRP pencereleri sandviç alan kütlesi 2,10 kg/m²
× alan), iki taraf toplamı. BE: bağlantı elemanı sayısı (sahibi olan parça).

| No | Parça (TR) | Malzeme / katman | Adım | Adet | Kütle (kg) | BE |
|---|---|---|---|---|---|---|
| SH-350 | burun konisi (radom) | GFRP 7781 sandviç | 36 | 1 | 0,152 | 6 × M4 |
| SH-351 | ön gövde kaplaması | CFRP `shell_secondary` | 16 | 1 | 0,238 | 18 |
| SH-352 | alt burun kaplaması | CFRP | 16 | 1 | 0,284 | 16 |
| SH-353 | ön bölme kapağı | CFRP | 36 | 1 | 0,083 | 6 Camloc |
| SH-354 | aviyonik kapağı (GNSS 1 penceresi) | GFRP | 36 | 1 | 0,335 | 8 Camloc |
| SH-355 | üst burun kaplaması | CFRP | 16 | 1 | 0,190 | 8 |
| SH-357 / -358 | sol / sağ yan bölme kapağı | CFRP / GFRP | 36 | 1+1 | 0,072 / 0,103 | 9 + 9 Camloc |
| SH-360 | üst orta kaplama | CFRP | 16 | 1 | 0,420 | 58 |
| SH-361 | alt orta kaplama | CFRP | 16 | 1 | 0,324 | 42 |
| SH-363 | taret açıklık halkası | 2,0 mm katı CFRP | 24 | 1 | 0,158 | 8 |
| SH-365 | taret kapağı tahrik erişim kapağı | CFRP | 24 | 2 | 0,059 | 8 Camloc |
| SH-366 | paraşüt kapağı çevre kaplaması | CFRP | 16 | 1 | 0,212 | 28 |
| SH-367 | paraşüt kapağı | CFRP | 22 | 1 | 0,224 | — (mandal) |
| SH-368 | paraşüt bölmesi alt kaplaması | CFRP | 16 | 1 | 0,510 | 66 |
| SH-370 | görev bölmesi üst kaplaması | CFRP | 16 | 1 | 0,362 | 42 |
| SH-372 | GNSS 2 RF penceresi | GFRP | 21 | 1 | 0,015 | 3 |
| SH-373 | görev bölmesi kapağı | CFRP | 36 | 1 | 0,175 | 16 Camloc |
| SH-374 | görev bölmesi alt kaplaması | CFRP | 16 | 1 | 0,423 | 46 |
| SH-376 | yırtılır kayış örtüsü | GFRP | 22 | 1 | 0,203 | 4 × M3 naylon |
| SH-377 / 378 / 379 | ön / eyer / arka yakıt bölmesi kapağı | CFRP | 18 | 2+2+2 | 0,260 / 0,242 / 0,228 | 56 / 40 / 24 |
| SH-380 | üst orta gövde kaplaması | CFRP | 16 | 1 | 0,089 | 4 |
| SH-382 | faydalı yük kapağı | CFRP | 36 | 1 | 0,450 | 12 Camloc |
| SH-383 | alt orta gövde kaplaması | CFRP | 16 | 1 | 0,804 | 62 |
| SH-385 | yakıt ikmal kapağı | CFRP | 18 | 1 | 0,007 | menteşe |
| SH-386 | yakıt kapağı menteşe pimi Ø1,6 | A2 (ISO 2338) satın alma | 18 | 1 | 0,001 | — |
| SH-387 | arka teçhizat kapağı | CFRP | 36 | 1 | 0,137 | 10 Camloc |
| SH-388 | alt arka kaplama | CFRP | 16 | 1 | 0,596 | 44 |
| SH-389 | stabilatör eyleyici kapağı | CFRP | 36 | 2 | 0,094 | 10 Camloc |
| SH-390 | üst arka kaplama | CFRP | 16 | 1 | 0,439 | 30 |
| SH-391 | sırt soğutma girişi dudağı | CFRP (yapıştırma) | 16 | 1 | 0,042 | 6 perçin |
| SH-392 | dikey kök örtüsü | CFRP | 32 | 2 | 0,093 | 20 |
| SH-393 | stabilatör kökü fileto şeridi | CFRP | 31 | 2 | 0,030 | — (§11) |
| SH-394 | ventral kök şeridi | 5,8 mm katı CFRP | 33 | 1 | 0,103 | — (§11) |
| SH-395 | motor bölmesi üstü dikey kök kaplaması | CFRP | 32 | 2 | 0,057 | — (§11) |
| SH-420 | LERX/eldiven üst kaplaması | `wing_skin_primary` (+ kutu / LERX bölgeleri) | 17 | 2 | 0,733 | 12 perçin |
| SH-421 | LERX/eldiven alt kaplaması | `wing_skin_primary` | 17 | 2 | 0,528 | 8 perçin |
| SH-422 | kanat birleşim erişim kapağı | CFRP | 36 | 2 | 0,108 | 2 Camloc (§11) |
| SH-423 | arka pim erişim kapağı (bayonet) | ≈ 1,0 mm katı CFRP | 35 | 2 | 0,002 | — |
| SH-424 | kanat kökü birleşim kaplaması | `wing_skin_primary` | 17 | 2 | 0,420 | 2 perçin |
| SH-450 | üst kaporta orta parça | CFRP | 36 | 1 | 0,142 | 6 Camloc |
| SH-451 | alt kaporta yarımları | CFRP | 36 | 2 | 0,346 | 6 Camloc |
| SH-452 | üst kaporta yan parçası | 0,8 mm 6061-T6 sac | 36 | 2 | 0,071 | 4 Camloc |

## 4. İmalat

* **Kaplamalar ve kapaklar:** `prepreg_ooa_vacbag` — dişi kalıpta (OML takım yüzü) MTM45-1/AS4 PW prepreg, 0,4/5/0,4
  sandviç; her bağlantı sırasında, bindirme karasında ve menteşede çekirdek 1:3 rampayla 1,6 mm katı kenar bandına iner
  (`layout.shell.rules.sandwich_edges`). Bindirme karaları kaplamayla birlikte kürlenir (iç yüze 8 kat PW, kalıpta
  basamak). Kara dolguları ve 1,0 mm kutu üstü katı bölgeler aynı kürde; dolgu iç yüzü montajda sıvı şimle 0,2 mm'ye
  oturtulur. Kenarlar CNC ile kesilir (ISO 2768-mK), delikler şasi ana çerçeve referanslarından jig ile delinir.
* **RF pencereleri** (radom, aviyonik kapağı, sağ yan bölme kapağı, GNSS 2, kayış örtüsü): 7781 E-cam/MTM45 + ROHACELL;
  karbon yok.
* **Taret halkası, arka pim kapağı, ventral şerit:** katı CFRP (10 / 10 / 29 kat PW), CNC'de işlenir.
* **P-COWL-UPS:** 0,8 mm 6061-T6 sac, kesme + şekillendirme; kalınlık dik kesitte 0,8 mm (kaporta kapanışındaki dik
  yüzeyde kesit düzlemi kaydırması 3-B normal boyunca ölçülür).
* **Yakıt ikmal kapağı:** öne menteşeli; kapak burnunda Ø3,6 mm düğüm borusu, kaplamaya geçen Ø1,6 mm paslanmaz pim;
  kapağın uçları ve kaplamadaki açıklığın uçları menteşe eksenine diktir; menteşe önündeki oyuk (cove) R 7,2 mm (§7).
* **Arka pim erişim kapağı:** Ø30 mm bayonet kapak, alt eldiven kaplamasında Ø18 mm delik ve OML'ye paralel havşa.
  Kapak, arka pim P-REAR'ın alt ucunun (`layout.chassis.wing_joint.rear_spar.pin`: konum − boy/2, z −0,0296) 0,2 mm
  altında kalır: OML'ye paralel ≈ 1,0 mm katı CFRP (bayonet tırnakları havşa kenarında, modellenmedi). Arka kiriş
  başlığının arka kenarının 0,5 mm gerisinde düz kesilmiş D biçimi (başlık üstündeki 1 mm katı laminat sağlam kalır).
* **Stabilatör kökü fileto şeridi:** iç yüzde, kök bağlantısı cıvatalarının (`F-SPINDLE-NODE` B8–B11, M6, +y) somunları
  üzerinde Ø15 mm cepler; kaplama dışta 1,5 mm kalır.

## 5. Montaj (`spec.assembly`)

| Adım | Kabuk işi | Başlıca bağlantılar |
|---|---|---|
| 16 | Yapısal kaplamalar (burun, orta, merkez, arka; SH-351/352/355/360/361/366/368/370/374/380/383/388/390/391) | M4 ISO 7380 + yüzer somun plakası, 28 mm aralık, çerçeve başlıkları ve uzun kiriş flanşları |
| 17 | LERX/eldiven kaplamaları + kanat kökü birleşim kaplaması | EA 9394 yapıştırma, uç kaburgalarda kör perçin (peel stopper) |
| 18 | Yakıt bölmesi kapakları (conta), yakıt ikmal kapağı + menteşe pimi | M4 / M3 somun plakalı, 22–28 mm |
| 21 | GNSS 2 penceresi | M4 somun plakalı |
| 22 | Paraşüt kapağı (mandal + bağ ipi), kayış örtüsü | dil + pim; 4 × M3 naylon kesme vidası |
| 24 | Taret açıklık halkası, taret kapağı tahrik erişim kapakları | M4 / Camloc |
| 31–33 | Stabilatör kökü, dikey kök, ventral kök şeritleri | §11 |
| 35 | Arka pim erişim kapakları | bayonet |
| 36 | Kapaklar ve kaporta (alt yarımlar, üst yan parçalar, üst orta parça), burun konisi, birleşim erişim kapakları | Camloc 4002, M4 |

## 6. Arayüzler (hepsi `spec.layout`)

* **Panel ana hatları:** `layout.shell.panels[*].outline / x / y / z_band`; çerçeve gövde çizgileri
  `layout.stations[*].x_faces, sweep_deg, flange_w, inset`.
* **Şasi karaları:** istasyon T başlıkları; `layout.chassis.members[M-SPINE, M-PARAWALL, M-AFTKEEL, M-VENTRALKEEL].lands`,
  `M-KEEL.box`, `M-DORSAL.paths/section`, `M-CHINE.paths/section`, `M-CTBOX` kiriş çizgileri, başlık genişlikleri, `z`,
  `oml_clearance_m`, başlık seviyeleri; `M-SOB / M-GLOVERIB / M-JOINTRIB` kutuları (eldiven kaburga kuralı).
* **Kesikler:** `layout.mechanisms.door_outlines` (takım kapakları), `layout.shell.root_cut_lines` (dikey, stabilatör
  kökü, ventral; 3 mm boşluk), `payload.turret` (HD59 açıklığı), `F-TRUNNION.bolts` (iç yüzde Ø12 mm, ≈ 0,8 mm derin alın açma),
  `F-RISER-*` kutuları (kayış örtüsünde cep), `M-AFTKEEL.box`, `F-SPINDLE-NODE` kök cıvataları (stabilatör kökü
  şeridinde cep), `wing_joint.rear_spar.pin` (arka pim kapağı derinliği).
* **Mekanizmalar:** `layout.mechanisms.joints[para_hatch]`, `P-REFUEL.hinge` (eksen noktası, aralık).
* **Açıklıklar:** `layout.clearances` (paraşüt kapağı ≥ 10 mm; komşu kaplamalarla 0,2 mm kenar contası, bildirilmiş
  temas).

## 7. Mekanizmalar

* **Paraşüt kapağı (`para_hatch`, prizmatik, 0…0,15 m yukarı):** menteşesiz; FS1490 / FS1810 gövde yüzlerine dayanan
  dört konum dili ve YK250-SY-804 pim çekici mandal pimleri. Kapak kenarları komşu kaplamalardan (SH-360, -366, -370,
  -376) 0,2 mm, dik kesilmiş: kapak düz yukarı kalkarken temas etmez (süpürme denetimi 0 ihlal), kalkmış konumda
  ≥ 10 mm açıklık.
* **Yakıt ikmal kapağı (`refuel_door`, döner, 0…100°, `refuel_door_deg`):** eksen, kapağın ön OML kenarının kirişi
  2,0 mm içeri kaydırılmış doğrudur (karnın köşe eğriliği nedeniyle açıklık ortasında 4,3 mm derinlikte). 100° açıkken
  kapak dış yüzü menteşenin a·tan(θ/2) önünde OML seviyesini keser; oyuk yarıçapı bunu ve kapak kenarı köşelerini
  karşılar (R 7,2 mm). Kapak ve açıklık uçları eksene dik: y düzleminde kesilmiş bir uç, 13° eğik eksen etrafında
  dönerken yandaki kaplamaya girerdi. 0–100° süpürme 0 ihlal.

## 8. Bağlantı elemanları

**Sıra yerleşimi** (`Rows`): (1) çerçeve başlığı sıraları — gövde çizgisinin 15 mm önünde / arkasında, o başlıkta biten
her panel için (çerçeveyi geçen panelde yalnız arka flanşta); kesit eğrisi boyunca yoğun örnekleme, panel sahipliği
(plan bölgesi − kenar mesafesi; tam saran panellerde malzeme ışın denetimi), aralık; (2) uzun kiriş flanşı sıraları
(sırt kanalı, paraşüt duvarı, ventral omurga, omurga kirişi, sırt uzun kirişi şapka flanşları, kenar çizgisi J'si);
(3) sökülebilir panellerin ve kök örtülerinin ana hattının 2,5 D + 1,5 mm içindeki halka sıraları (bindirme karaları,
flanşlar); (4) özel sıralar (burun konisi 8 radyal M4 — 6'sı delinebildi, kayış örtüsü 4 × M3, yakıt kapakları iç
kenarı M3, görev bölmesi üst kaplamasının birleşim kaplaması karasındaki sırası, kaporta sıraları, eldiven perçinleri,
birleşim erişim kapağı). Simetrik çiftler birlikte ispatlanır ve birlikte delinir.

**İspat** (`Fix.prove`, `analysis.checks` ile aynı ışın sondaları): panel deliniyor ve yüzü OML noktasında; arkasındaki
ilk şasi / kabuk parçası 0,5 mm içinde (somun plakası ve perçinlerde ikinci / üçüncü parça da yığına girebilir); yığında
başka parça yok; her sıkılan parçada malzeme orta düzleminde kompozitte 2,5 D, metalde 2,0 D kenar mesafesi (+0,2 mm);
mevcut her deliğe 3 D + delik yarıçapı; baş ve somun plakası / Camloc yuvası eğri yüzeye en yüksek noktasından oturur
(plaka eğriliğin az olduğu yöne çevrilir, gerekirse diğer yöne); donanım başka parçaya ya da donanıma girmiyor. İspatı
geçmeyen aday delinmez ve sıra onun çevresinde kapanır (bu yapıda 238 aday). Kimlik: `<sahip parça>-B<n>` / `-C<n>`.

| Bağlantı | Eleman | Karşı parça | Adet |
|---|---|---|---|
| Yapısal kaplama → çerçeve / flanş | ISO 7380 M4 A2-70 | yüzer somun plakası M4 | 571 (yakıt kapakları dahil) |
| Yakıt kapağı iç kenarı, kayış örtüsü | ISO 7380 M3 | somun plakası M3 | 54 |
| Kapaklar, kaporta | Camloc 4002 | 2600 yuva (perçinli) | 106 |
| Eldiven / giriş / birleşim kaplaması uçları | kör perçin 3,2 / 4 mm (CherryMAX sınıfı) | — | 28 |

## 9. Kütle ve bütçe

Kabuk grubu (registry): **10,561 kg** ↔ bütçe `mass.budget.shell` 10,48 ± 0,31 kg → **+0,081 kg (+%0,8), bant
içinde.** Katkılar: yapısal gövde kaplamaları (13 parça) 4,89 kg, kapaklar / pencereler / sökülebilir paneller
2,95 kg, LERX/eldiven + birleşim kaplaması 1,68 kg, kaporta + kök şeritleri + giriş dudağı 0,88 kg, taret halkası
0,16 kg. Kütle, katman alan kütlesinin kalınlığa
bölünmüş eşdeğer yoğunluğuyla (`Registry.density`) hesaplanır: kara dolguları, bindirme karaları ve kutu üstü 1 mm katı
bölgeler sandviç eşdeğer yoğunluğundadır (gerçekte katı laminat; bu fark ayrıca hesaplanmadı), sandviç kenar bantları
da ayrı modellenmemiştir — gerçek kütle hesaplanandan biraz yüksek olabilir.

Kabuk bağlantı elemanları (grup `hardware`): **3,34 kg** — 759 eleman (ortalama ≈ 4,4 g; `hardware.py` geometrisi). Bu, layout'un
yapısal kaplamalar için istediği 25–32 mm aralıklı M4 somun plakalı vida kavramının doğrudan sonucudur; uçağın tamamı
için ayrılan `mass.budget.hardware` 1,51 kg ile çelişir (§11).

## 10. Layout'tan sapmalar (gerekçeli)

1. **Çerçeve başlığına düşen kapak kenarları gövde çizgisine taşındı** (P-FWDHATCH ön/arka FS0300/FS0600, P-AVHATCH
   0,60–1,11, P-SIDEBAY arka kenar FS1110, P-MBHATCH FS1810–FS-FUEL, P-FUEL1 ön FS-FUEL, P-FUEL3 arka FS-GEAR,
   P-AFTHATCH FS-GEAR–FS3480, P-STABACT FS3480–yangın perdesi ön yüzü): layout'un 0,3–6 mm kaymaları başlıkta bir
   tarafta 2,5 D bırakmaz; gövde çizgisinde panel, başlığı komşusuyla paylaşır (sıra gövdeden 15 mm, panel kenarına
   14,5 mm, başlık kenarına 13 mm).
2. **P-PAYHATCH:** ön kenar FS-FUEL gövde çizgisinde, arka kenar FS-RS'nin ok açılı arka başlığında, yanlar omurga
   kirişi başlıklarına (±0,2295) — Camloc sırası için kirişin iç yüzünden 24 mm.
3. **Paraşüt kapağı – komşular 0,2 mm** (layout: kayış örtüsü 1 mm geride): açıklık kuralı (≥ 10 mm) bildirilmiş temas
   dışındaki kabuk parçalarına uygulanır; kenar contalı, düz yukarı kalkan kapak için komşularla temas bildirildi.
4. **Bindirme karası 28 mm** (layout "25 mm", kural "≥ 25 mm"): Camloc sırasının panel kenarına ve kara kenarına 2,5 D'si
   için. Kaporta orta parçasının kenarları altında 32 mm.
5. **Kaporta bölümü X_COWL = 3,668** (yangın perdesi arka yüzünün 1,5 mm önü) ve **stabilatör kökü şeridinin kenarları
   yüzeye dik kesildi** (z bandı + y ≥ 0,236 sınırının OML'yi kestiği noktalardan normal boyunca): yatay / düşey düzlem
   kesimleri eğik yan yüzeyde tüy kenar bırakıyordu (kalınlık denetimi). Bant x 3,925'te biter (gövde y 0,236'nın
   içine daralır); oradan X_AFT'ye kadar üst yan ve alt kaporta kenar çizgisinde ayrılır.
6. **Yakıt kapakları iç kenarı M3** (layout M4): sırt kanalı flanşı (yapıldığı haliyle) panelin 17,5 mm altına uzanır;
   M4 için 2 × 10 mm gerekir. M3 22 mm aralıkla (≤ 8 D).
7. **Eldiven peel-stopper perçinleri 3,2 mm** (layout "M4 kör perçin, 150 mm"): uç kaburgaların T flanşları 20 mm;
   4 mm perçin 2 × 2,5 D = 20,4 mm ister.
8. **Kaplama "insert+screw" tipleri somun plakasıyla** (burun konisi, taret halkası, GNSS 2, kök örtüleri): karalar
   1,6 mm katı laminat; gömülü M4 insert 1,2 D diş boyu (4,8 mm) ister.
9. **Kayış örtüsü** 0,2 mm ileriye uzatıldı (madde 3) ve risers bağlantılarının üstünde ±32 mm cep.
10. **Taret halkası** ana hat dikdörtgeni (264 × 250 mm) korunur; sıralar FS1110 / FS1330 başlıklarında ve yan karada.
11. **Eldiven alt/üst ayrımı** kiriş çizgisi yerine her kesitin **orta eğri (camber) çizgisi**: kök kesitleri ters
    kamburlu, firar kenarı yakınında alt yüzey kiriş çizgisinin üstüne çıkar.

## 11. Açık konular

* **Donanım bütçesi:** kabuk bağlantı elemanları 3,34 kg ↔ uçak toplamı `mass.budget.hardware` 1,51 kg. Ya bütçe
  layout'un M4 / 25–32 mm kavramına göre güncellenmeli ya da kavram (ör. sabit kaplamalarda yapıştırma + seyrek vida)
  değiştirilmeli — sistem düzeyinde karar.
* **Kaporta ön kenarı – yangın perdesi:** layout'a göre kaporta ön sırası paslanmaz kenar köşebendine (YK250-PR-091,
  propulsion) M3 ile bağlanır. Propulsion kabuktan sonra kaydedildiği için bu sıra kabukta delinmez; propulsion ya da
  entegrasyon tarafından delinmeli (kaporta 1,5 mm önden başlar, 2,5 D hazır).
* **Kuyruk modülüne bağlanan şeritler:** SH-393 (stabilatör kökü), SH-395 (motor bölmesi üstü dikey kök kaplaması) ve
  SH-394 (ventral kök) — bunların karaları kuyruk modülünün kök kaburgası flanşlarıdır (layout notları); şasi
  karalarında 2,5 D'li delik yeri yoktur. Bağlantılar kuyruk modülüyle birlikte tasarlanmalı.
* **Kanat birleşim erişim kapağı:** 2 Camloc. Kapak ön kenarı eldiven burnunun eğri bölgesindedir; SOB / eldiven /
  birleşim kaburgası T flanşları (20 mm) Camloc için dar. Kaburga flanşlarının 26 mm'ye çıkarılması (şasi) ya da M4
  somun plakalı kenar önerilir.
* **Taret açıklık halkası:** layout 14 × M4 ister; ispatı geçen 8 vida (FS1110 / FS1330 başlıklarında 4, yan
  karada 4). Halka kenarının geri kalanı alt kaplama yuvasının dar şeridinde 2,5 D tutmaz; yan kara takviyesi
  (M-TURRETWALL.side_land) genişletilirse tamamlanır.
* **Üst orta gövde kaplaması (SH-380):** yakıt kapakları arasında kalan dar şeritler; yalnız 4 vida. Çoğu kenar yakıt
  kapaklarının ve birleşim kaplamasının karalarındadır.
* **Kanat kökü birleşim kaplaması:** layout'un drag modeline girmediği belirtiliyor (PK2-01/PK2-10, OML modülü).

## 12. Doğrulama

```
python3 -m ucav250.analysis.checks --modules chassis,shell --focus YK250-SH --no-write   # 0 ihlal
python3 -m ucav250.analysis.checks --modules chassis,wing,tail,shell --focus YK250-SH --no-write   # 0 ihlal
python3 -m unittest tests.test_ucav250_shell                                               # 12 test, OK
python3 -m ucav250.blender.build --modules chassis,shell --previews --no-blend --out <dizin>
python3 -m ucav250.layout_build.build --check
python3 -m ucav250.analysis.layout_check --check
python3 -m ucav250.analysis.structures --check
python3 -m ucav250.analysis.sizing --check
```

Modül `spec.layout`'u değiştirmez; layout / yapı / boyutlandırma denetimleri bu modülden etkilenmez.
