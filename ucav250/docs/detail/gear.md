# YK-250 HANÇER — İniş takımı (GEAR) ayrıntılı tasarımı

Üretici modül: `ucav250/design/gear.py` (`register(reg, spec)`), test: `tests/test_ucav250_gear.py`.
Sözleşme: `ucav250/ARCHITECTURE.md`; tek doğruluk kaynağı: `ucav250/spec.yaml` (`landing_gear`, `fuselage`,
`layout.part_numbers`, `layout.mechanisms` eklemleri / `gear_retraction` dizisi / `door_outlines` / `assembly_paths`,
`layout.chassis.fittings` F-TRUNNION / F-UPLOCK / F-NG-PIVOT, `layout.chassis.members` M-WELLKEEL / M-WELLWALL-FWD /
M-KEELWALL / M-GEARBEAM, `layout.stations` FS1110 / FS-GEAR, `layout.systems` ACT-MLG-EMA / ACT-NLG-EMA / ACT-STEER /
EQ-DOORACT / EQ-NDOORACT, `layout.shell.panels`, `layout.clearances`, `structures.sizing.gear`, `layups`, `processes`,
`materials`, `mass`) ve `data/research/components.yaml` (TOST tekerlek/lastik/kaliper, Volz DA 22 / DA 26 veri
sayfası değerleri, SAGITTA referans takımı). Modül başka bir modülün geometrisini okumaz; konumlar `spec.layout`
arayüzlerinden ve `oml.py` dış hat matematiğinden türetilir.

Kapsam yalnızca sivil EO/IR gözetleme platformunun içeri katlanan üçlü (tricycle) iniş takımıdır. Takımda silah,
mühimmat, dış yük taşıma ya da bırakma düzeneği yoktur ve öngörülmemiştir.

## 1. Özet

| Büyüklük | Değer |
|---|---|
| Parça kimliği | 66 (grup `gear` 65: YK250-LG-620 … -686 ve YK250-FC-676-R/L; grup `controls` 1: YK250-FC-628) |
| Bağlantı elemanı (`joints.py` + yerel pim yardımcısı, geometri `hardware.py`) | 50: 12 × M5 12.9, 6 × M4 12.9, 20 × M3 12.9, 8 × kör perçin Ø3,2 (CherryMAX), 4 × ISO 2341-B Ø5 pim |
| Hareketli eklem | 10 (layout'takiler, aynen): `main_gear_R/L`, `main_inner_door_R/L`, `main_trunnion_door_R/L`, `nose_gear`, `nose_steer` (`nose_gear` alt eklemi), `nose_door_R/L` |
| Hareket dizisi | `gear_retraction` (layout'taki 21 durum aynen; kontrol 0 = takım açık ve kilitli = dinlenme konumu) |
| Tasarım kuralı denetimleri | `checks --modules chassis,shell,gear --focus YK250-LG,YK250-FC-6`: sonuç §12'de |
| İniş takımı grubu kütlesi (registry) | **10,54 kg** — bütçe 14,57 ± 0,44 kg (**−4,03 kg**, bkz. §9) |
| Kumanda grubu (takım payı: yönlendirme DA 26) | 0,27 kg |
| Üçgen sayısı (takım parçaları) | ≈ 114 000 |

## 2. Yapı ve yük yolları

**Ana bacaklar (sağ modellenir, sol `mirror_part`).** Mafsal ekseni `landing_gear.main.trunnion` (x ekseni), aks
merkezi `axle_static`; bacak ekseni ikisini birleştiren 18° açılı (splay) doğrudur — 108° içe katlamada bacak
yatayda, tekerlek 18° eğik olarak kuyuya yerleşir (`stowed_wheel_center` testle doğrulanır). Bacak, SAGITTA
mimarisindeki teleskopik yay/yağ amortisörlü bacaktır:

* **Çatal + dış silindir (LG-620, 7075-T651 dövme, 5 eksen işleme).** Çatal kolları 13 mm (layout `arm_m`), F-TRUNNION
  kulaklarının iç yüzlerinden 1'er mm içeride; kol göbekleri r 25 mm. Kulak dışındaki kısımlar yalnız iç yarıda tepeye
  bağlanır, böylece çatal hiçbir konumda kanat-kökü kıvrımındaki alt kaplamaya değmez; tepe (crown) bacak yarığının
  genişliğindedir. Dış silindir OD 42 × 3,5 (`structures.sizing.gear.main_leg`), alt ucunda conta somunu. Silindir
  üzerinde tümleşik yukarı kilit makara kulağı (dinlenmede iç-alt yanda, toplanmışta üstte), bacak kapağı braket
  kulakları (dış-üst yanda) vardır.
* **Flanşlı saplama akslar (LG-636, 4130 N, d 20 × 30, Ø10 delikli).** Layout `pivot_scheme`: çatalın içinden dışa
  doğru kollardan geçirilip CH-074 20 H7 burçlarına girer (`assembly_paths main_stub_axle_*`); flanşları 3'er M3 12.9
  ile kola vidalanır. Ön saplama aks EMA çıkış kamasını (iç kanallı) alır.
* **Kayar boru + aks taşıyıcı (LG-621, 7075-T651).** Ø34,5 × 2,75 boru, silindir deliğinde 0,25 mm radyal çalışma
  boşluğu (conta burçları); statik basınçta 30 / 80 mm strok (`landing_gear.main`). Alt uçtaki 12 mm taşıyıcı plaka
  tekerleğin iç yanında, lastikten 7 mm uzakta; aks deliği 20,02 mm. Dönmeyi önleme: conta somunundaki iç kama
  (makas kolları bacak yarığı genişliğine sığmadığı için kullanılmadı).
* **Aks (LG-634, 4130 N, Ø20 × 4 et).** Konsol aks, başı taşıyıcının iç yüzünde, yayılma burcu ile tekerlek göbeği arası
  dolu; dış uçta kama ile tork plakası (LG-635, 7075) ve M20x1,5 taçlı somun.
* **Tekerlek + lastik (LG-622, TOST SB Max II 70-50-20 + 200x50 6 PR + iç lastik, 1,390 kg veri sayfası)**, fren diski
  (LG-637, tekerlek kütlesine dahil), BZT M kaliper (LG-623, 0,400 kg) tork plakasına 2 × M5 12.9 (kalipere diş).
* **Amortisör kartuşu (LG-625, zarf, 0,35 kg tahmin).** Endüstriyel hidrolik sönümleyici + helezon yay (SAGITTA).

Yük yolu: tekerlek → aks → taşıyıcı / kayar boru → amortisör + conta burçları → dış silindir → çatal → saplama akslar →
CH-074 burçları → F-TRUNNION → kiriş + kuyu tavanı (şasi).

**Ana takım toplama EMA'sı (LG-630, zarf, 0,27 kg tahmin).** Layout ACT-MLG-EMA: d 60 × 40 mm, F-TRUNNION ön
kulağının önünde, mafsal ekseninde; çıkış kaması ön saplama aksa girer. Gövdesi, kirişe giden B2 cıvatasının başını
geçmek için üst-dış yanda düzleştirilmiştir. Tepki flanşı (6 mm) kuyu ön duvarına (CH-035, 1,6 mm) 3 × M5 12.9 ile
(somunlar duvarın önündeki kablo kanalında) bağlanır.

**Yukarı kilitler.** Ana takım: F-UPLOCK yuvasının pim deliğinde dönen 4130 iki plakalı mandal (LG-642) ve bacaktaki
çift makara (LG-639, Ø14) — makaralar toplamanın sonunda mandal çentiklerine alttan girer. Burun takımı: aks
uçlarındaki makaralar (LG-647) omurga duvarına yapıştırılmış 4130 mandal plakalarına (LG-650-R/L) girer. Mandal
pençesi ve çözme solenoidi takım ünitesi kalemidir (§11).

**Burun bacağı.** Pivot `landing_gear.nose.pivot` (y ekseni, 90° arkaya katlanır), yönlendirme ekseni pivottan geçen
düşey (`nose_steer`, ±20°), çatal izi 30 mm. Çatal + dış silindir (LG-624, 7075; OD 36 × 3), kollar 9 mm iki katmanlı
(iç katman tepeye bağlanır; dış katman yalnız r 17,5 mm göbek — F-NG-PIVOT cıvata başlarının ve açık kapakların
10 mm kuralının içinde), flanşlı saplama akslar d 16 (LG-640, CH-073 burçlarına), amortisör kartuşu (LG-633, zarf),
yönlendirme borusu + arkaya izli çatal (LG-631), aks (LG-648) ve göbek ara parçaları (LG-649), TOST LR Max II +
200x50 tekerlek (LG-626, 0,895 kg), yönlendirme bileziği + kasnak (LG-644), sürtünmeli shimmy sönümleyici disk paketi
(LG-646), Volz DA 26 yönlendirme eyleyicisi (FC-628, layout ACT-STEER OBB'si; 90° dişli kafa + HTD kasnağı, kayış
LG-645). Toplama EMA'sı (LG-632) layout kutusunda, sağ omurga duvarı alanına 2 × M5 12.9.

**Kapaklar.** Hepsi `door_outlines` dış çizgisinden (1,0 mm panel aralığı) OML'ye oturan 4 mm CFRP sandviç (kütle:
alan × `shell_secondary` alan kütlesi), menteşe çizgisi layout eklem ekseni:

* *İç kapak (LG-670):* layout menteşe çizgisi (z −0,2036, yatay) gövdenin 15 mm'lik alt eğriliği nedeniyle yalnız
  x 2,914–2,950'de kapak laminatının içindedir; bu bölgede kuyu omurga gövdesine (CH-036) 2 × M4 ile bağlı üç kulaklı
  sabit menteşe braketi (LG-671), kapakta iki 7075 menteşe dili, Ø3 pim (LG-677). Kapağın menteşe kenarı, açılırken
  orta şeride süpürdüğü bölgede (döner kesit birleşimi) geri alınmıştır.
* *Bacak kapağı (LG-672):* bacakla döner (`main_gear_R`), takım toplanınca bacak yarığını kapatır; iki 7075 mesafe
  braketi (LG-680) bacak kulaklarına 2'şer kör perçinle, ayakları kapağa yapıştırılır.
* *Mafsal kapağı (LG-673):* kiriş alt kenarı menteşe çizgisinde; kiriş orada çentikli olduğu için menteşe uç blokları
  (LG-681) yarığın iki ucunda alt kaplamanın iç yüzüne yapıştırılır, Ø3 pim (LG-682), açma yayı pimde.
* *Burun istiridye kapakları (LG-674-R/L):* menteşe istasyonları yalnız bacak çatalının ve lastiğin menteşe çizgisine
  yaklaşmadığı yerlerde (x 0,700–0,746 ve 1,063–1,085): omurga duvarı iç yüzüne yapıştırılmış bloklar (LG-683),
  Ø3 pimler (LG-684).
* Kapak eyleyicileri: ana iç kapak DA 22'leri (FC-676-R/L) FS-GEAR arka yüzünde bağlantı plakasıyla (LG-685, gömme
  dişli burç, yakıt bölmesi tarafına delik yok), burun kapağı DA 22'si (LG-678) sol yan bölme tepsisinin oyuğundan
  geçerek iki kulakla tepsiye (2 × M4 + somun plakası).

## 3. Parça listesi (kütle bir taraf; R/L çiftlerinde ikisi de aynı)

| Kimlik | Ad | Malzeme / süreç | Kütle (kg) |
|---|---|---|---:|
| LG-620-R/L | ana bacak çatal + dış silindir | 7075-T651, CNC | 0,362 |
| LG-621-R/L | kayar boru + aks taşıyıcı | 7075-T651, CNC | 0,202 |
| LG-622-R/L | TOST SB Max II + 200x50 lastik + iç lastik | satın alma | 1,390 |
| LG-623-R/L | TOST BZT M kaliper | satın alma | 0,400 |
| LG-625-R/L | amortisör kartuşu (zarf) | satın alma (tahmin) | 0,350 |
| LG-630-R/L | toplama EMA'sı (zarf) | satın alma (tahmin) | 0,270 |
| LG-634-R/L | aks Ø20 + somun | 4130 N, CNC | ≈0,20 |
| LG-635-R/L | fren tork plakası | 7075-T651, CNC | 0,039 |
| LG-636-R/L | flanşlı saplama aks çifti | 4130 N, CNC | ≈0,20 |
| LG-637-R/L | fren diski (tekerleğe dahil) | satın alma | 0 |
| LG-639-R/L | yukarı kilit makara çifti | satın alma | 0,012 |
| LG-642-R/L | ana yukarı kilit mandalı | 4130 N, CNC | 0,063 |
| LG-670-R/L | ana iç kapak | CFRP sandviç | 0,074 |
| LG-671-R/L | iç kapak sabit menteşe braketi | 7075-T651, CNC | 0,010 |
| LG-672-R/L | bacak kapağı | CFRP sandviç | 0,005 |
| LG-673-R/L | mafsal kapağı | CFRP sandviç | 0,008 |
| LG-677 / -682-R/L | menteşe pimleri Ø3 | satın alma | 0,003 / 0,005 |
| LG-680-R/L | bacak kapağı mesafe braketleri | 7075-T651, CNC | 0,087 |
| LG-681-R/L | mafsal kapağı menteşe blokları | 7075-T651, CNC | 0,015 |
| LG-685-R/L | iç kapak eyleyici plakası | 7075-T651, CNC | 0,024 |
| FC-676-R/L | iç kapak eyleyicisi Volz DA 22 | satın alma | 0,132 |
| LG-624 | burun bacağı çatal + silindir | 7075-T651, CNC | 0,316 |
| LG-626 | TOST LR Max II + 200x50 + iç lastik | satın alma | 0,895 |
| LG-631 | yönlendirme borusu + çatal | 7075-T651, CNC | 0,349 |
| LG-632 | burun toplama EMA'sı (zarf) | satın alma (tahmin) | 0,270 |
| LG-633 | burun amortisör kartuşu (zarf) | satın alma (tahmin) | 0,250 |
| LG-640 | burun saplama aks çifti d 16 | 4130 N, CNC | 0,119 |
| LG-644 / -645 / -646 | yönlendirme bileziği / kayış / shimmy diskleri | 7075 / satın alma | 0,064 / 0,010 / 0,020 |
| LG-647 / -648 / -649 | yukarı kilit makaraları / aks / göbek araları | satın alma / 4130 / 7075 | 0,004 / 0,185 / 0,007 |
| LG-650-R/L | burun yukarı kilit mandal plakası | 4130 N, CNC | 0,022 |
| LG-674-R/L | burun istiridye kapağı | CFRP sandviç | 0,026 |
| LG-683-R/L / -684-R/L | burun kapağı menteşe blokları / pimleri | 7075 / satın alma | 0,006 / 0,025 |
| LG-678 | burun kapağı eyleyicisi Volz DA 22 | satın alma | 0,132 |
| FC-628 | yönlendirme eyleyicisi Volz DA 26 (+ dişli kafa) | satın alma | 0,270 |
| **Ayrıntı turu (GD-01 … GD-12)** | | | |
| LG-686-R/L | iç kapak tahrik krankı + Ø8 mil | 4130 N, CNC | 0,026 |
| LG-687-R/L / -688-R/L | iç kapak biyeli / kapak boynuzu | 7075-T651, CNC | 0,008 / 0,002 |
| LG-689-R/L | iç kapak mili yatak yuvası (2 × 688-2RS) | 7075-T651, CNC | 0,012 |
| LG-685-R/L | DA 22 kama plakası (eğik bağlantı) | 7075-T651, CNC | 0,033 |
| LG-691-R/L | mafsal kapağı mikro dişli motoru | satın alma (tahmin) | 0,045 |
| LG-692-R/L / -693-R/L / -694-R/L | mafsal kapağı krankı / biyeli / boynuzu | 4130 N / 7075 | 0,005 / 0,003 / 0,003 |
| LG-695-R/L | dişli motor braketi | 7075-T651, CNC | 0,009 |
| LG-641-R/L | ana aşağı kilit pimi Ø14 | 4130 N, CNC | 0,052 |
| LG-638-R/L | aşağı kilit solenoidi | satın alma (tahmin) | 0,060 |
| LG-643-R/L | dönme önleyici anahtar | 4130 N | 0,001 |
| LG-651-R/L | fren hattı döner rakoru | satın alma (tahmin) | 0,025 |
| LG-653 / LG-655 | burun sürükleme biyeli / EMA krankı | 4130 N, CNC | 0,069 / 0,057 |
| LG-656 | burun saplama aralık burcu (krikolu) | 7075 / 4130 | 0,013 |
| LG-679 / -696 / -697 | burun kapak tahrik krankı / biyeli / boynuzu | 4130 N / 7075 | 0,012 / 0,007 / 0,003 |
| LG-698-R/L / -699-R/L | eşleme boynuzları / biyelleri | 7075-T651 | 0,003 / 0,005 |
| LG-700 / -701 | eşleme kızağı / kılavuz plakası | 7075-T651 | 0,004 / ≈0,035 |
| FC-629 | fren birimi Volz DA 26 + ana silindir | satın alma (0,27 veri sayfası + 0,065 tahmin) | 0,335 |
| LG-658 | fren birimi askısı | 7075-T651, CNC | ≈0,040 |

## 4. İmalat

* 7075-T651 parçalar (çatallar, taşıyıcılar, braketler, plakalar): kaba dövme / levhadan 5 eksen CNC, ISO 2768-m,
  burç ve pim delikleri H7, sert eloksal; et kalınlığı ≥ 1,5 mm (`processes.cnc_milling_metal`).
* 4130 N (saplama akslar, akslar, mandallar): tornalama/frezeleme, Ø20 g6 yatak yüzeyleri, kadmiyumsuz çinko-nikel
  kaplama; saplama aksların ortası Ø10 delinir.
* Kapaklar: OOA prepreg sandviç, vakum torbası; kenar şeritleri 1,6 mm monolitik; menteşe dilleri ve blokları EA 9394
  ile yapıştırılır; kenarlar ISO 2768-mK (layout.shell.rules).
* Satın alma: TOST tekerlek/kaliper/lastik, Volz DA 22 / DA 26, EMA'lar ve amortisör kartuşları (takım ünitesi
  tedarikçisi), makaralar, ISO 2338 / 2341 pimler.

## 5. Montaj (`spec.assembly` adımları)

* **20** — (yalnız referans) fren eyleyicisi EQ-BRAKE_UNIT bu modülde yerleştirilmedi (§11).
* **28 Ana iniş takımı** — saplama akslar çatalın içinden burçlara (`main_stub_axle_fwd/aft_R`), flanş M3'leri
  (Loctite 243); EMA kuyu ön duvarına 3 × M5; tekerlek/fren, tork plakası, kaliper; bacak kapağı braketleri (perçin) ve
  bacak kapağı; yukarı kilit mandalı F-UPLOCK pimine. Kontrol: tekerlek–kuyu ≥ 12 mm (bütün hareket) checks ile.
* **29 Burun iniş takımı** — saplama akslar d 16, EMA sağ omurga duvarına, DA 26 bacağa (4 × M3), kayış.
* **30 Takım kapakları ve sıralama** — iç kapak braketleri ve pimleri, mafsal kapağı blokları, burun kapağı blokları
  ve pimleri, DA 22'ler; kapak contaları, tam toplama–açma dizisi.

## 6. Arayüzler

| Arayüz | Kaynak | Kullanım |
|---|---|---|
| Mafsal ekseni, aks, splay, katlama açısı | `landing_gear.main` | bacak ekseni, saplama akslar |
| F-TRUNNION kulak/burç x'leri, `pivot_scheme` | `layout.chassis.fittings` | çatal kolları, saplama aks boyu |
| F-UPLOCK noktası (+ 6 mm pim deliği) | `layout.chassis.fittings` | ana yukarı kilit mandalı |
| F-NG-PIVOT, M-KEELWALL | `layout.chassis` | burun çatalı, saplama aks boyu, EMA / blok bağlantısı |
| M-WELLKEEL, M-WELLWALL-FWD, M-GEARBEAM (çentik) | `layout.chassis.members` | menteşe braketi, EMA flanşı, kapak rölyefi |
| eklemler, dizi, `door_outlines` | `layout.mechanisms` | aynen kaydedilir |
| ACT-MLG-EMA, ACT-NLG-EMA, ACT-STEER, EQ-DOORACT, EQ-NDOORACT | `layout.systems` | EMA / DA 22 / DA 26 zarfları |
| P-CENTRE-LOWER, P-NOSE-LOWER | `layout.shell.panels` | menteşe oyukları (kaplamaya `add_hole`) |

Kabuk parçalarına açılan oyuklar: SH-383'te iç kapak menteşe bölgesinde r 4,2 mm ve mafsal kapağı menteşe çizgisi
boyunca r 4,2 mm; SH-352'de burun kapağı menteşe istasyonlarında r 4,2 mm (kabuk sahibinin kenar şeridine alması
gerekir, §11).

## 7. Mekanizmalar ve kinematik

* `main_gear_R/L`: 0 → 108° (eksen −x / +x), bacak + tekerlek + bacak kapağı birlikte; `nose_gear` 0 → 90° arkaya;
  `nose_steer` ±20° (yalnız takım açıkken, toplamadan önce ortalanır).
* `main_inner_door_R/L` 0 → 90° (gear_up 0–0,15 açılır, 0,85–1,0 kapanır), `main_trunnion_door_R/L` 125° açık →
  0 kapalı (0,85–1,0), `nose_door_R/L` 90° açık → 0 kapalı (0,85–1,0); layout eklemleri aynen kullanılır.
* Kapak menteşe kenarları, kapak açılırken çevredeki sabit kaplamanın (ve kiriş/omurga duvarının) süpürdüğü bölge
  çıkarılarak (kesit kesit döndürülmüş engel birleşimi, prizmatik kesici) şekillendirilmiştir.

### 7.1 Ayrıntı turu (GD-01 … GD-12): bağlantı mekanizmalarının modellenmesi

Her düzlemsel tahrik (krank – biyel – boynuz) tam düzlem kinematiğiyle çözülür (`gear.FourBar`): bütün eksenler
paraleldir, mekanizma eksene dik düzlemde çözülür ve iki zincirli döner eklemle konumlanır — krank eklemi (merkez C,
eksen u) ve onun alt eklemi olan biyel eklemi (merkez = dinlenme konumundaki krank pimi K). Eklem değerleri
`gear_retraction` dizisinin 21 örneğinde hesaplanır, diziye eklenir ve aynı değerler `gear_up`'ın parçalı doğrusal
`clamp` ifadesi (`expr`) olarak yazılır (örneklerde dizi = ifade; checks bağlı eklemleri yalnız dizi boyunca tarar).
Bütün yeni eklemler takım açıkken (dinlenme) 0'dır. Kilitli konumda krank ve biyel aynı doğru üzerindedir (katlanmış
ölü nokta = ölü nokta ötesi kilit): çıkış yükü krank eksenine moment vermez. Testler zincirin kapandığını (biyel ucu
pimi = çıkış boynuzu pimi, her durumda < 0,05 mm), ölü noktayı ve iletim açısını doğrular.

| Mekanizma (eklemler) | Seçilen düzen | Krank r / biyel l | Krank hareketi | min. iletim açısı | Pim |
|---|---|---|---:|---:|---|
| İç kapak (`main_idoor_crank/link_R/L`) | DA 22 FS-GEAR arka yüzünde, çıkışı menteşe hattına paralel (3,9° eğik kama plaka), mil C-DOORSHAFT'taki yataktan kuyuya; krank 686 + biyel 687 + kapak boynuzu 688 | 34,0 / 78,0 mm | 131° | 57° | Ø4 |
| Mafsal kapağı (`main_tdoor_crank/link_R/L`) | kendi mikro dişli motoru 691 (tahmin) F-TRUNNION flanşında, krank 692 + biyel 693 + boynuz 694 | 16,0 / 38,0 mm | 160° | 33° | Ø3 |
| Burun sürükleme (`nose_drag_crank/link`) | EMA çıkışı (sağ yarıda) → krank 655 + sürükleme biyeli 653 orta düzlemde → çatala tümleşik boynuz (r 31 mm, 116°) | 34,1 / 74,8 mm | 129° | 56° | Ø10 |
| Burun kapak tahriki (`nose_ddrive_crank/link`) | DA 22 sol yan bölmede, çıkışı menteşe eksenlerine paralel; krank 679 + biyel 696 sol omurga duvarındaki W-NDOOR-LINK penceresinden sol kapak boynuzu 697'ye | 16,9 / 82,6 mm | 141° | 41° | Ø4 |
| Burun kapak eşleme (`nose_dbar`, `nose_dlink_R/L`) | her kapakta boynuz 698 (r 28 mm, 160°), iki biyel 699 (50,9 mm) orta düzlemde kızak 700'e; kızak iki Ø5 kılavuz pimiyle kılavuz plakası 701'in yarığında kayar (43,6 mm) | — | — | 55° | Ø4 |
| Ana aşağı kilit (`main_downlock_R/L`, prizmatik) | yay itmeli, solenoid 638 çekmeli Ø14 4130 kilit pimi 641, arka mafsal kulağının alt uzantısında, mafsal ekseninin 28 mm altında, arka çatal kolundaki göbeğe 11,5 mm girer | — | 15 mm | — | — |

Çalışılan ve elenen düzenler (özet):

* **İç kapak (GD-05):** C-DOORLINK penceresinden itme çubuğu 90° kapak hareketini geçirmiyordu (en iyi pay −0,8 mm).
  DA 22'yi döndürüp çıkışını menteşe hattına paralel yapmak mekanizmayı tek düzleme indirir; krank merkezi
  (y 44, z −136 mm) ve boynuz pimi (y 41, z −180 mm) 2-B taramada (5 400 aday) sabit kesitlere ≥ 23 mm pay ve 57°
  iletim açısıyla seçildi. Düzlem sırası (u boyunca): boynuz x 3,0745–3,0775 (kapakta), biyel 3,078–3,081, krank
  3,0815–3,0845; bacak parçaları x ≥ 3,0745'e hiç ulaşmaz. Mil, FS-GEAR ön yüzündeki flanşlı yatak yuvası 689'dan
  (2 × 688-2RS, dudak keçe) geçer; C-DOORSHAFT 18 × 18 mm.
* **Mafsal kapağı (GD-07):** bacaktan tahrik (dört çubuk, makara-kam, Cenevre, dişli) tek tek çalışıldı: dört
  çubuk ya bacak kapağını keser ya da iletim açısı 24–34° ve kiriş köşesine girer; makara/kam izin verilen kapanma
  eğrisini (≈ 125° − 1,16 θ) ihlal eder. Kapak kendi motoruyla, bacak kilitlendikten sonra (0,85–1,0) kapanır —
  dizi ve eklem ifadesi artık tutarlıdır (§10). Gerekli moment G-TDOOR-LINK 0,257 N m (limit); kapalı konum ölü
  nokta ötesi olduğundan tutma momenti gerekmez.
* **Burun sürükleme (GD-04):** EMA kutusu sağ yarıya kaydırıldı (layout ACT-NLG-EMA x 0,636–0,696, y 0,006–0,041);
  krank, biyel ve boynuz orta düzlemde (y −10,5…+5,5 mm). Boynuz pimi r 31 mm'de: boynuz gözü r ≥ 20,5 mm'de kalır,
  saplama aksların içeriden takılma yolu (r ≤ 19 mm) serbesttir (r 25 mm'lik ilk çözüm bu yolu kapatıyordu; sabit
  EMA çıkışıyla yeniden tarandı, 60 000 aday, 2-B pay 8,6 mm). F-NG-PIVOT cıvata başlarına dokunulmadı.
* **Burun kapakları (GD-08):** layout'taki x eksenli orta dirsek iki kapağı ayna görüntüsü olarak hareket
  ettiremez. Tahrik sol kapağa, eşleme kızaklı simetrik bağlantıyla sağ kapağa: sağ kapak tam ayna (testte her
  durumda doğrulanır). Kızak ilk çözümde iki sabit pimde kayan uzun bir çubuktu (üst ucu −0,041'e çıkıp layout
  zarfını aşıyordu); pimler kızağa, yarık kılavuz plakasına alındı (plaka −0,170 … −0,095, eşiğin 2 mm üstünde).
* **Ana aşağı kilit (GD-03):** çift kesmeli çatal, süpürülen bacak hacminde ikinci kulak gerektirdiği için olanaksız;
  tek kesme (1 mm boşluk) ve Ø14: G-DOWNLOCK MS 2,52, G-DOWNLOCK-BEND MS 0,52 (`structures`). Pim gear_up 0,02–0,10
  arasında 15 mm geri çekilir, bacak 0,15'te hareket eder; açık konumda uç, kolun 3,5 mm arkasındadır.
* **Tork bağlantısı (GD-12):** kayar boru makaslı tork bağlantısı yerine anahtarla (643, 3 × 2,5 × 9 mm, keçe
  yuvasında) döndürülmez; 64 mm bacak yarığında OD 42 silindirin yanına makas (≈ 10 mm) + paylar sığmaz.
* **Burun saplama akslarının tutulması (GD-11):** iki flanş arasında krikolu aralık burcu 656 (iki yarım, sağ-sol
  dişli; emniyet teli) aksların içeri kaymasını önler; dönmeyi mevcut Ø3 pim tutar.
* **Fren (GD-09):** FC-629 (DA 26 + ana silindir) aviyonik güvertenin altına askı 658 ile asılır (4 × M4 güverte
  burçlarına); iki fren hattı H-MAIN boyunca ana takımlara, arka mafsal kulağı yüzündeki döner rakor 651'e
  (mafsal ekseninde), oradan delikli saplama aksından çatala ve bacak boyunca esnek hortumla kalipere (yol tarifi;
  bacak hortumu modellenmedi, §11).
* **Menteşeler (GD-06/07/08):** iç kapak menteşe hattı, kapak iç yüzü + 4 mm'den geçen en iyi (minimax) eğik doğru
  (sapma 1,33 mm); üç istasyon (x 2,8824–2,8954 / 2,9574–2,9704 / 3,0424–3,0554), her istasyonda 3 mm kulak çifti ve
  bir M4. Kabuk SH-383/SH-352 menteşe oyukları (r 4,2 mm, istasyon ± 1,5 mm) artık `shell.py` tarafından açılır
  (layout `door_outlines.*.hinge`); takım modülü kabuğa delik eklemez. Yapıştırılan menteşe blokları cıvatalıdır:
  mafsal kapağı blokları kiriş gövdesine 2 × M4 burç (çentik dışındaki flanşla), burun kapağı blokları ve yukarı kilit
  mandalları omurga duvarından M4 ile.
* **F-UPLOCK (GD-10):** mandal pim deliği layout `F-UPLOCK.pin` (z −0,080): göbek tabanına 10 mm (2 D), yüzlere
  10,5 mm; göbek cıvataları 15 mm'ye alındı (şasi).

## 8. Bağlantı elemanları

| Bağlantı | Eleman | Not |
|---|---|---|
| saplama aks flanşı → ana çatal kolu | 6 × M3 12.9 (diş kolda) | Loctite 243 |
| EMA flanşı → kuyu ön duvarı | 6 × M5 12.9 + ISO 7040 | baş tarafında pul |
| kaliper → tork plakası | 4 × M5 12.9 (diş kaliperde) | |
| bacak kapağı braketi → bacak kulağı | 8 × kör perçin Ø3,2 | başlar yarık genişliği içinde |
| iç kapak menteşe braketi → kuyu omurga gövdesi | 6 × M4 12.9 + ISO 7040 (istasyon başına 1) | somunlar kablo kanalında |
| yukarı kilit makara pimi, mandal pimi | 4 × ISO 2341-B Ø5 | |
| DA 26 → burun bacağı kulakları | 4 × M3 12.9 (diş kulakta) | |
| burun EMA flanşı → sağ omurga duvarı | 2 × M5 12.9 + ISO 7040 | somun sağ yan bölmede |
| DA 22 (ana) üst kulakları → kama plaka → FS-GEAR | 4 × M3 12.9, gömme dişli burç | yakıt bölmesi tarafına delik yok |
| iç kapak yatak yuvası → FS-GEAR ön yüzü | 4 × M3 12.9, gömme dişli burç | |
| DA 22 (burun) ön kulağı → yan bölme tepsisi | 2 × M4 12.9 + somun plakası | |
| tahrik pimleri (krank–biyel, biyel–boynuz) | ana: 8 × Ø4 + 4 × Ø3; burun: 2 × Ø10, 2 × Ø4 (tahrik), 3 × Ø4 + 2 × Ø5 kılavuz (eşleme) | ISO 2341-B + kopilya, düzlem aralığı 0,5 mm pul |
| mafsal kapağı menteşe blokları → kiriş gövdesi | 8 × M4 12.9, gömme dişli burç | GD-08 |
| dişli motor braketi → F-TRUNNION flanşı | 4 × M3 12.9 (diş flanşta) | |
| aşağı kilit solenoidi / fren döner rakoru → arka kulak yüzü | 4 + 4 × M3 12.9 (diş kulakta) | |
| burun kapak menteşe blokları → omurga duvarı | 6 × M4 12.9 + ISO 7040 | somun yan bölmede |
| burun yukarı kilit mandalı → omurga duvarı | 4 × M4 12.9 (diş mandal göbeğinde) | yan bölmeden |
| eşleme kılavuz plakası → omurga duvarları | 2 × M4 12.9 + ISO 7040 | |
| fren birimi askısı → aviyonik güvertesi / birim gövdesi | 4 × M4 12.9 burç / 4 × M3 A2-70 | |

Kenar mesafesi ≥ 2 D (metal) / 2,5 D (kompozit), adım ≥ 3 D: checks ile doğrulanır.

## 9. Kütle ve bütçe

İniş takımı grubu (registry): **10,54 kg**; bütçe `mass.budget.gear` 14,57 ± 0,44 kg → **4,03 kg eksik**. Bütçe
kalemleri SAGITTA'nın 4,0 / 3,5 kg bacak kütlelerinden ölçeklenmiştir (`main_gear_legs_wheels_brakes_emas_pair`
8,39 kg, `nose_gear_leg_wheel_steering` 3,67 kg) ve `gear_doors_wells_locks_sensors` 2,50 kg; modelde olmayanlar:
fren hatları ve kablo/sensörler, kilit pençeleri ve solenoidler (0,25 kg kural), kuyu kapatma ve kesik takviyeleri
(0,36 + 0,50 kg; şasi/kabuk tarafı), kapak contaları (0,10 kg), kapak tahrik bağlantıları (§11), EMA ve amortisör
kütleleri tahmindir. Fark bütçenin aşağı yönlü olduğunu gösterir; ağırlık merkezi etkisi `analysis/mass.py` ile
değerlendirilmelidir (toplanmış konum, burun takımı daha hafif).

## 10. Layout'tan sapmalar (gerekçeli)

* Ana EMA gövdesinde düzlük: F-TRUNNION B2 kiriş cıvatasının başı layout EMA zarfına girer.
* Burun saplama akslarının boyu bushing ucunda biter (layout 22 mm duvara girer); layout'taki 3 × M4 flanş vidası
  9 mm kola ve cıvata başları arasına sığmaz → pim + aralık burcu 656 ile tutma (GD-11).
* Burun kapak menteşe istasyonları yalnız iki bölgede — layout kuralları (kapak–bacak 10 mm, lastik–kapak 12 mm)
  sürekli bir menteşeyi engeller.
* Ayrıntı turunda layout'un kendisi düzeltildi (sapma değil): iç kapak menteşe hattı ve istasyonları, mafsal kapağı
  `expr`i = dizi (0,85–1,0), EQ-DOORACT / EQ-NDOORACT / ACT-NLG-EMA konumları ve mil eksenleri, C-DOORSHAFT,
  W-NDOOR-LINK, F-TRUNNION aşağı kilit uzantısı, F-UPLOCK pimi, `door_outlines.*.hinge`.

## 11. Açık konular

1. **G-DOOR-DRIVE:** iç kapak işi (|Cp| 1,0 üst sınırında 14,0 J) DA 22'nin anma torku × 180° işinden (5,7 J)
   büyüktür (`structures` G-DOOR-DRIVE, değişmedi). Yeni bağlantıda krank 131° döner (kapak/krank oranı 0,69);
   DA 22'nin bu açıyı verdiği ve tork açığı (daha güçlü eyleyici ya da açma yayı) tedarikçiyle kapatılmalı. Kapalı
   kapak ölü nokta ötesi kilitle tutulur (G-DOOR-LOCK eyleyiciden alınmaz).
2. **Mafsal kapağı dişli motoru 691:** zarf (Ø16 × 21 mm) ve kütle (0,045 kg) tahmindir; 0,3 N m çıkış, enkoder ve
   uç sınır anahtarları ile seçilmeli (Faulhaber / maxon sınıfı).
3. **Aşağı kilit solenoidi 638 ve fren döner rakoru 651:** katalog sınıfı tahmin zarfları/kütleleri; pim konum
   sensörü takım ünitesi kalemidir.
4. **Fren hattı:** H-MAIN boyunca boru ve bacak boyunca esnek hortum (döner rakordan kalipere) modellenmedi; FC-629
   bağlantı dişleri Volz kurulum resmine göre doğrulanmalı.
5. Yukarı kilit pençeleri ve çözme solenoidleri (takım ünitesi) zarfları modellenmedi.
6. Yönlendirme DA 26'nın milinin yatay olması nedeniyle 90° dişli kafa (tahmin) — DA 26 + kayış SAGITTA uygulaması.
7. Kütle bütçesi farkı (§9) ve EMA/amortisör tahminleri, tedarikçi seçimiyle güncellenmeli.
8. Ana bacak anahtar yuvası: kayar borunun tam basma konumunda (50 mm) yuva boyu borunun alt tapasına kadar
   uzatılmıştır; tam stroklu boru boyu amortisör tedarikçisiyle kesinleşmeli.

## 12. Doğrulama

* `python3 -m ucav250.analysis.checks --modules chassis,shell,gear --focus YK250-LG,YK250-FC-6,YK250-CH,YK250-SH --no-write`
* `python3 -m unittest tests.test_ucav250_gear`
* Önizleme: `python3 -m ucav250.blender.build --modules chassis,gear --previews --no-blend --out /tmp/...`
