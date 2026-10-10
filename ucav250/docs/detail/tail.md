# YK-250 HANÇER — Kuyruk (TAIL) ayrıntılı tasarımı

Üretici modül: `ucav250/design/tail.py` (`register(reg, spec)`), test: `tests/test_ucav250_tail.py`.
Sözleşme: `ucav250/ARCHITECTURE.md`; tek doğruluk kaynağı: `ucav250/spec.yaml` (`tail.surfaces`, `fuselage`,
`layout.part_numbers`, `layout.chassis.fittings` F-FIN-FRONT / F-FW-CORNER / F-STUB-FRONT / F-SPINDLE-NODE /
F-VENTRAL-1..3, `layout.stations` FS3480 / FS3670, `layout.mechanisms.joints`, `layout.systems` ACT-RUDDER /
EQ-STABACT, `layout.clearances`, `structures.sizing.tail`, `layups`, `processes`, `materials`) ve
`data/research/components.yaml` (Volz DA 26 / DA 30 veri sayfası değerleri). Modül başka bir modülün geometrisini
okumaz; bütün arayüzler `spec.layout` üzerindendir.

Kapsam yalnızca sivil EO/IR gözetleme platformunun kuyruk takımıdır: ikiz eğik dikey + dümenler, sabit kök parçaları,
tam hareketli stabilatörler, ventral kanatçık ve değiştirilebilir tampon kızağı. Kuyrukta silah, mühimmat, dış yük
taşıma ya da bırakma düzeneği yoktur ve öngörülmemiştir.

## 1. Özet

| Büyüklük | Değer |
|---|---|
| Parça kimliği | 126 (grup `tail` 76: YK250-TL-250 … -291 ve TL-301 mil; grup `controls` 50: YK250-FC-300 … -325) |
| Parça numarası | 67 (sağ taraf +y'de modellenir, sol taraf `mirror_part` ile aynalanır; ventral ve kızak merkez hattında) |
| Bağlantı elemanı (`joints.py`, geometri `hardware.py`) | 119: 14 × M6 12.9, 13 × M5 12.9, 18 × M5 Ti, 28 × M4, 40 × M3 Ti, 6 × ISO 2341-B Ø3 menteşe pimi |
| Hareketli eklem | 14: `rudder_R/L`, `stabilator_R/L` (layout) + 10 bağlı eklem (`rudder_arm/rod1/rod2_*`, `stab_arm/rod_*`, `expr` ile) |
| Hareket dizileri | 4: `rudder_R/L`, `stabilator_R/L` (25'er tam dört-çubuk durumu, iki uç ve nötr dahil) |
| Tasarım kuralı denetimleri | `checks --modules chassis,tail --focus YK250-TL,YK250-FC`: **0 ihlal** (mesh, statik, süpürme, açıklık, kalınlık, bağlantı elemanı, temas, bağlanma) |
| Kuyruk grubu kütlesi (registry) | **7,230 kg** — bütçe tavanı 7,35 ± 0,22 kg (−0,120 kg, bkz. §9) |
| Kumanda grubu (kuyruk payı) | **3,037 kg** — bütçe kalemleri toplamı 3,047 kg (−0,010 kg) |
| Kuyruk bağlantı elemanları (grup `hardware`) | 0,462 kg |
| Üçgen sayısı (kuyruk parçaları) | 207 490 |

Önizlemeler (`python3 -m ucav250.blender.build --modules chassis,tail --previews --no-blend`): ön, yan, üst, alt, iki
izometrik ve patlatılmış izometrik görünüş, ayrıca kuyruğa yakın arka 3/4, iç yan, alt 3/4 ve patlatılmış görünüşler
(dümen ±25°, stabilatör −20° / +15°) incelendi; parçalar şasi bağlantılarına oturur, yüzen ya da kopuk parça, görünür
girişim yoktur.

## 2. Yapı ve yük yolları

**Dikeyler (TL-250 … -270, -288 … -291; eğim 22°, açıklık 0,889 m, NACA 0012).** Kapalı sandviç kaplama
(`layups.tail_skin` 0,6 / 4 / 0,4 mm, gövde OML'sinin 1 mm üstünde kesilir, dolgu sızdırmazlığı), 0,241 c'de ön kiriş
(CFRP C kesit, başlıklar arkaya, 12 kat UD × 25 mm → uçta 4 kat, 1:20 kat bırakma; gövde 5 kat PW ±45) ve menteşe
çizgisinin 40 mm önünde ona paralel arka (menteşe) kirişi (başlıklar öne, 6 kat UD × 20 mm). Kaburgalar: gövde OML'sine
oturan kök kaburgası, iki kutu kaburgası (η 0,35 / 0,70), 3 mm 7075 servo kaburgası (DA 26 flanşı), dümen kökünün
altında arka kapama kaburgası ve uç kaburgası. Yük yolu: eğilme → kiriş başlıkları → iki işlenmiş 7075 kök bağlantısı
→ şasi çatalları. Ön bağlantı (TL-266) 7,8 mm kulağıyla F-FIN-FRONT çatalına (CH-096, FS3480) 2 × M6 12.9 çift kesme,
boynu sırt şapka kirişinin aralığından geçer, 5 mm flanşı ön kiriş gövdesine 2 × M5 Ti. Arka bağlantı (TL-267)
F-FW-CORNER yuvasına (CH-086, yangın perdesi) 2 × M5 12.9 (baş arka kulakta, diş perde tarafı gövdede), flanşı arka
kiriş gövdesine 2 × M5 Ti. İtme çubuğunun arka kirişten geçtiği yuvanın çevresinde 2 mm 7075 C takviye (TL-270).
GFRP uç kapağı (TL-258, 4 kat 7781, 2,4 GHz anten penceresi) kaplamanın kademeli (joggle) monolitik alanına oturur,
2 × M3 Ti somun plakalarına. İç yüzde servo kapağı (TL-268, 5 kat PW, gömme) 4 × M4 Ti somun plakalı 7075 şeritlere.

**Dümenler (FC-300 … -314, -325; η 0,144–0,844, %30 veter, ±25°).** Yuvarlak burunlu 0,6 mm CFRP kaplama (sabit
kaplamadaki oluğa 2 mm aralıkla oturur), menteşe çizgisinin 14,5 mm arkasında C kiriş (7 kat UD × 20 mm başlık),
ROHACELL 51 WF çekirdek. Üç menteşe (η 0,30 / 0,57 / 0,80): arka kiriş gövdesinde 7075 çatal (2 × M3 Ti somun plakası),
dümen kirişinde 7075 dil (2 × M3 Ti, somun çekirdek cebinde), Ø3 ISO 2341-B pim + pul + kopilya (H7/g6). Kol: iç yüzde
7075 uyumlu ped + 3 mm plaka, menteşe ekseninden 26,8 mm bilye; 2 × M3 Ti, 7075 karşı plakaya. Dümen uç boşlukları
6 mm. Kolun ve çubuğun bilye yuvasının ±25° süpürmesi için sabit kaplamanın oluk dudağında kol açıklığı (süpürülmüş
zarf + 2 mm, arka kiriş gövdesinin arkasında; kenar kapatma: çizim notu).

**Sabit kök parçaları (TL-252, -265, -269, -271 … -275; y 0,177–0,331).** Sandviç kaplama, ön kiriş (F-STUB-FRONT yuva
düzleminde, 0,141 c, başlıklar arkaya), bölünmüş kök ve uç kaburgaları (mil yuvası geçişi). Ana yük yolu: 7075 mil yuvası
(TL-269): 6 mm haç biçimli kök flanşı düğüm bağlantısının dış yanağına (CH-095, layout B8–B11) 4 × M6 12.9, Ø49 × 2
tüp, uçta dış 61805-ZZ rulmanının 37 H7 yuvası (düğüm göbeğiyle eş eksenli işlenir). Ön yük yolu: 7075 plaka bağlantı
(TL-265): 7,8 mm kulak CH-098 çatal yuvasında (uç yüzü yuva tabanına oturur) 2 × M5 12.9 çift kesme (başlar FS3480
erişim deliklerinden), boyun gövde yanından ve kök kaburgasından geçer, 5 mm flanş ön kiriş gövdesinin ön yüzüne
2 × M5 Ti. Yuva tüpü kontrolü: kök momenti 144 N m (yapı hesabı T-NODE-STUB, limit) × 1,5 × 1,15 → σ = 74 MPa,
7075 Ftu'ya göre MS ≈ 6.

**Stabilatörler (TL-256, -276 … -281, TL-301, FC-315 … -321; −20° / +15°).** Ti-6Al-4V mil (TL-301): Ø25 (rulman
yerleri taşlanmış), düğüm yanağı deliğinde boyun Ø21 (hareketli parça açıklığı ≥ 5 mm), dış rulmanda Ø28 bilezik,
delik 22/18, iç uçta 15 mm dolu (M6 uç cıvatası), son 20 mm'de kama (yapı hesabı). İç 61805-ZZ düğüm göbeğinde iki
DIN 472 segmanla, dış 61805-ZZ yuva ucunda segmanla; eksenel konum: iç bilezik – pul (Ti) – M6 uç cıvatası ve dış
bilezik. Panel: sandviç kaplama (kapalı uç), C kiriş (soketin arkasından uçta 0,40 c'ye, başlık 12 → 4 kat), kök /
soket / orta kaburgalar, 7075 kök soketi 29 × 2 × 100 mm (çapraz cıvata göbekleri), Ti tapa; stabilatör mile geçirilir ve
alt yüzden M5 12.9 çapraz cıvatayla (üst göbekte diş) emniyetlenir — sökülebilir. Kök boşluğu 8 mm (spec
`y_root_gap`), bütün hareket aralığında ≥ 6 mm (R-38).

**Ventral kanatçık ve tampon kızağı (TL-254, -260, -261, -282 … -286).** Sandviç kaplama (0,80 c'nin arkası çekirdek
dolgulu firar kenarı, 1,3 mm firar kenarı alanı), 3 mm orta düzlem gövdesi, kök kaburgası. Üç 7075 kök bağlantısı
F-VENTRAL-1..3 çatallarına (CH-099 … -101) M5 12.9 çift kesme; 1 ve 2 numaralı bağlantıların çatal yanakları orta düzlem
gövdesine 2 × M5 Ti, 3 numaralının dili kaplamanın monolitik kulak bölgesindeki yuvaya 2 × M5 Ti. Uçta 4130 U şerit
(TL-260) ve değiştirilebilir PA12 aşınma pabucu (TL-261, uç altında 12 mm), 2 × M4 ISO 7380; pabuç pervane diskinin
altındaki layout temas noktasını (`ventral.bumper.contact_point`) taşır (R-48).

**Tahrikler.** Dümen: Volz DA 26 (FC-303, 0,270 kg, veri sayfası) dikey kutusunda servo kaburgasının altında, kasa çıkış
milinin üstünde, flanş 4 × M3 Ti (helisel insert); 12 mm kol, 6 mm 7075 itme çubuğu (iki ucunda geçmeli bilye yuvası),
uzaysal dört-çubuk. Stabilatör: Volz DA 30 (FC-302, 0,630 kg) yangın perdesinin ön yüzünde (soğuk taraf) 7075 beşikte
(üst / alt plakalar 4 × M4 Ti ile kulak deseninden, taban 4 × M4 Ti gömülü insertlere; hafifletme pencereleri), 15 mm
kol, 8 mm itme çubuğu C-FW-PUSHROD'dan (yanmaz körük: sistemler), mil kamasına kelepçeli 37,5 mm kol (M4 12.9 sıkma
cıvatası), düzlemsel dört-çubuk.

## 3. Parça listesi (sağ + sol / merkez; kütle iki taraf toplamı)

| Parça | Ad | Malzeme | Süreç | Kalınlık / lamine | Adım | Kütle (kg) |
|---|---|---|---|---|---:|---:|
| TL-250-R/L | dikey kaplaması | CFRP PW | prepreg OoA | tail_skin | 32 | 1,361 |
| TL-251-R/L | dikey ön kiriş | CFRP UD | prepreg OoA | 1,0 mm | 32 | 0,187 |
| TL-252-R/L | sabit kök kaplaması | CFRP PW | prepreg OoA | tail_skin | 31 | 0,299 |
| TL-253-R/L | dikey arka (menteşe) kirişi | CFRP UD | prepreg OoA | 1,0 mm | 32 | 0,116 |
| TL-254 | ventral kanatçık kaplaması | CFRP PW | prepreg OoA | tail_skin | 33 | 0,168 |
| TL-255-R/L | dikey kök kaburgası | CFRP PW | prepreg OoA | rib_panel | 32 | 0,071 |
| TL-256-R/L | stabilatör kaplaması | CFRP PW | prepreg OoA | tail_skin | 34 | 2,077 |
| TL-257-R/L | dikey alt kutu kaburgası | CFRP PW | prepreg OoA | rib_panel | 32 | 0,027 |
| TL-258-R/L | dikey uç kapağı (GFRP, anten) | GFRP 7781 | prepreg OoA | 1,0 mm | 32 | 0,334 |
| TL-259-R/L | dikey üst kutu kaburgası | CFRP PW | prepreg OoA | rib_panel | 32 | 0,012 |
| TL-260 | tampon kızağı şeridi | 4130 N | CNC | 2,0 mm | 33 | 0,073 |
| TL-261 | tampon kızağı pabucu (değiştirilebilir) | PA12 | SLS | 4,0 mm | 33 | 0,026 |
| TL-262-R/L | dikey servo kaburgası | 7075-T651 | CNC | 3,0 mm | 32 | 0,121 |
| TL-263-R/L | dikey arka kapama kaburgası | CFRP PW | prepreg OoA | rib_panel | 32 | 0,011 |
| TL-264-R/L | dikey uç kaburgası | CFRP PW | prepreg OoA | rib_panel | 32 | 0,013 |
| TL-265-R/L | sabit kök ön kiriş kök bağlantısı | 7075-T651 | CNC | 5,0 mm | 31 | 0,148 |
| TL-266-R/L | dikey ön kiriş kök bağlantısı | 7075-T651 | CNC | 5,0 mm | 32 | 0,128 |
| TL-267-R/L | dikey arka kiriş kök bağlantısı | 7075-T651 | CNC | 5,0 mm | 32 | 0,127 |
| TL-268-R/L | dikey servo kapağı | CFRP PW | prepreg OoA | 1,0 mm | 32 | 0,054 |
| TL-269-R/L | sabit kök mil yuvası | 7075-T651 | CNC | 3,0 mm | 31 | 0,381 |
| TL-270-R/L | arka kiriş itme çubuğu yuvası takviyesi | 7075-T651 | CNC | 2,0 mm | 32 | 0,054 |
| TL-271/272-R/L | sabit kök kaburgası ön / arka | CFRP PW | prepreg OoA | rib_panel | 31 | 0,067 |
| TL-273/274-R/L | sabit kök uç kaburgası ön / arka | CFRP PW | prepreg OoA | rib_panel | 31 | 0,062 |
| TL-275-R/L | sabit kök ön kirişi | CFRP UD | prepreg OoA | 1,0 mm | 31 | 0,020 |
| TL-276-R/L | stabilatör kirişi | CFRP UD | prepreg OoA | 1,0 mm | 34 | 0,211 |
| TL-277/278/279-R/L | stabilatör kök / soket / orta kaburgası | CFRP PW | prepreg OoA | rib_panel | 34 | 0,134 |
| TL-280-R/L | stabilatör kök soketi | 7075-T651 | CNC | 2,0 mm | 34 | 0,131 |
| TL-281-R/L | stabilatör çapraz cıvata tapası | Ti-6Al-4V | CNC | 11 mm | 34 | 0,075 |
| TL-282 | ventral kök kaburgası | CFRP PW | prepreg OoA | rib_panel | 33 | 0,007 |
| TL-283 | ventral orta düzlem gövdesi | CFRP PW | prepreg OoA | 3,0 mm | 33 | 0,135 |
| TL-284/285/286 | ventral kök bağlantıları 1–3 | 7075-T651 | CNC | 3,0 mm | 33 | 0,162 |
| TL-288/289-R/L | servo kapağı somun plakası şeritleri | 7075-T651 | CNC | 1,5 mm | 32 | 0,019 |
| TL-290/291-R/L | uç kapağı somun plakası pedleri | 7075-T651 | CNC | 1,5 mm | 32 | 0,006 |
| TL-301-R/L | stabilatör mili | Ti-6Al-4V | CNC (torna + taşlama) | 1,5 mm | 31 | 0,414 |
| FC-300-R/L | dümen kaplaması | CFRP PW | prepreg OoA | 0,6 mm | 32 | 0,363 |
| FC-302-R/L | stabilatör eyleyicisi Volz DA 30 | satın alma | — | — | 31 | 1,260 |
| FC-303-R/L | dümen eyleyicisi Volz DA 26 | satın alma | — | — | 32 | 0,540 |
| FC-304-R/L | dümen kirişi | CFRP UD | prepreg OoA | 1,0 mm | 32 | 0,126 |
| FC-305-R/L | dümen çekirdeği | ROHACELL 51 WF | işlenmiş, ortak kür | 4,0 mm | 32 | 0,113 |
| FC-306/307/308-R/L | dümen menteşe dilleri | 7075-T651 | CNC | 3,0 mm | 32 | 0,042 |
| FC-309-R/L | dümen kolu | 7075-T651 | CNC | 3,0 mm | 32 | 0,017 |
| FC-310/311/312-R/L | dümen menteşe çatalları | 7075-T651 | CNC | 3,0 mm | 32 | 0,093 |
| FC-313-R/L | dümen servo kolu | 7075-T651 | CNC | 3,0 mm | 32 | 0,005 |
| FC-314-R/L | dümen itme çubuğu | 7075-T651 | CNC | 1,9 mm | 32 | 0,015 |
| FC-315/316-R/L | stabilatör iç / dış rulmanı 61805-ZZ | çelik | satın alma | — | 31 | 0,126 |
| FC-317/318/319-R/L | DIN 472 J37 segmanlar | yay çeliği | satın alma | — | 31 | 0,018 |
| FC-320-R/L | mil uç pulu | Ti-6Al-4V | CNC | 3,0 mm | 31 | 0,014 |
| FC-321-R/L | stabilatör mil kolu | 7075-T651 | CNC | 3,0 mm | 31 | 0,103 |
| FC-322-R/L | stabilatör eyleyici beşiği | 7075-T651 | CNC | 3,0 mm | 31 | 0,156 |
| FC-323-R/L | stabilatör servo kolu | 7075-T651 | CNC | 3,0 mm | 31 | 0,007 |
| FC-324-R/L | stabilatör itme çubuğu | 7075-T651 | CNC | 2,1 mm | 31 | 0,033 |
| FC-325-R/L | dümen kolu karşı plakası | 7075-T651 | CNC | 1,5 mm | 32 | 0,005 |

Her parça `name_tr`, malzeme, süreç, kalınlık ya da lamine, ana parça (`parent`), montaj adımı, patlatma vektörü ve
temas listesiyle kayıtlıdır. Satın alınan parçalar: DA 26 / DA 30 kütle ve boyutları `components.yaml` veri
sayfasından; rulman ve segman kütleleri zarf hacmi × çelik yoğunluğu (tahmin, etiketli).

## 4. İmalat

* **Kompozit (prepreg OoA, vakum torbası, `processes.prepreg_ooa_vacbag`: en az 0,6 mm, kenar mesafesi ≥ 2,5 D):**
  kaplamalar dişi kalıplarda (iç / dış yarım, ortak kür), çekirdek kenarları 1:3 rampayla monolitik alanlara iner
  (servo kapağı alanı, uç kapağı eteği, ventral kulak 3 ve kızak bölgesi); kirişler erkek kalıpta C kesit, UD başlık
  katları 1:20 bırakılır; kaburgalar düz `rib_panel` sandviç levhadan CNC kesim. Dümen: çekirdek OML'den işlenir,
  menteşe dili ve kol için çekirdek cepleri (somunlar kapatmadan önce yerleşir), kaplama tek parça sarılır.
  Kol açıklığı ve itme çubuğu yuvası kenarları reçine / pamuk dolgu ile kapatılır.
* **Metal (CNC, `processes.cnc_milling_metal`: en az 1,5 mm, ISO 2768-mK, kenar mesafesi ≥ 2 D):** kök bağlantıları,
  mil yuvası, beşik, menteşe çatal / dilleri, kol ve itme çubukları 7075-T651 levhadan; kök soketi ve mil yuvası
  rulman yeri düğüm göbeğiyle birlikte hat-delme fikstüründe 37 H7; Ti mil çubuktan torna + rulman yerleri taşlama,
  kama ucu frezede; 4130 kızak şeridi işlenir ve kaplanır (kadmiyumsuz çinko-nikel); PA12 pabuç SLS.
* Bütün delikler ISO 273 orta seri (çift kesme kulaklarında 5,3 / 6,4 mm sıkı delik), kompozit / metal arasında
  galvanik ayırma için cam elyaf kat ve Ti bağlantı elemanları; somun plakaları perçinle (çizim notu).

## 5. Montaj (`spec.assembly` adımları)

| Adım | İş | Bağlantılar (iki taraf) |
|---|---|---|
| 31 | Sabit kök parçaları: kulak (TL-265) önceden ön kirişe bağlı alt montaj yana sürülerek CH-098 yuvasına, mil yuvası flanşı düğüm yanağına; dış rulman yeri düğüm göbeğiyle hizalı. Mil kolu düğümün iki yanağı arasında tutulurken miller dışarıdan yuvadan, kol göbeğinden ve iki rulmandan geçirilir; segmanlar, pul + M6 uç cıvatası, kol sıkma cıvatası. DA 30 + beşik yangın perdesinin ön yüzüne; itme çubuğu körükten mil koluna | 4 × M5 12.9 kulak, 4 × M5 Ti flanş, 8 × M6 12.9 yanak, 2 × M6 uç, 2 × M4 12.9 sıkma, 8 × M4 Ti kulak, 8 × M4 Ti taban |
| 32 | Dikeyler: ön kulak F-FIN-FRONT'a, arka kulak F-FW-CORNER'a; dümenler menteşe pimleriyle; DA 26 servo kapağından; uç kapağı | 4 × M6 12.9, 4 × M5 12.9, 8 × M5 Ti flanş, 12 × M3 Ti çatal, 12 × M3 Ti dil, 4 × M3 Ti kol, 6 pim, 8 × M3 Ti DA 26, 8 × M4 Ti kapak, 4 × M3 Ti uç kapağı |
| 33 | Ventral kanatçık üç kök bağlantısıyla arka omurgaya; kızak şeridi ve pabucu | 3 × M5 12.9 kulak, 6 × M5 Ti çatal / ped, 2 × M4 A2-70 kızak |
| 34 | Stabilatörler mil uçlarına geçirilir, çapraz cıvata | 2 × M5 12.9 |

Denetimler (spec.assembly): kök boşluğu 8 mm, bağlantı oranı nötrde 2,5:1, −20° / +15° serbest, dikey eğimi 22°,
dümen kökü kaportaya ≥ 8 mm (±25°), kızak pervane diskinin altında.

## 6. Arayüzler

| Arayüz (spec.layout) | Kuyruk tarafı | Not |
|---|---|---|
| F-FIN-FRONT (CH-096, FS3480) | TL-266 kulak 7,8 mm, 2 × M6 12.9; FS3480'de 2 × Ø13 erişim deliği (baş tarafı) | delikler `Part.add_hole` ile (geç kesim) |
| F-FW-CORNER (CH-086) | TL-267 kulak x 3,680–3,688 yuvasında, 2 × M5 12.9 perde tarafı gövdeye dişli | chassis.md arayüz tablosu |
| F-STUB-FRONT (CH-098, FS3480) | TL-265 kulak (uç yüzü yuva tabanında), 2 × M5 12.9; FS3480'de 2 × Ø10,5 erişim deliği ve dış T başlığında boyun geçiş yarığı (8,6 × 52 mm) | yarık şasi parçasına geç kesimle açılır — bkz. açık konu 1 |
| F-SPINDLE-NODE (CH-095) | TL-269 flanşı dış yanağa (B8–B11, 4 × M6 12.9); iç 61805-ZZ göbekte, segman yivleri göbekte | yiv: şasi açık konusu |
| F-VENTRAL-1..3 (CH-099 … -101) | TL-284 … -286 dilleri, M5 12.9 çift kesme | |
| FS3670 (CH-013) | DA 30 beşiği ön yüzde 4 × M4 gömülü insert; C-FW-PUSHROD'dan itme çubuğu | körük: sistemler |
| mekanizmalar `rudder_R/L`, `stabilator_R/L` | aynı ad, eksen, aralık, `prop`, `scale`; dinlenme = nötr | |
| ACT-RUDDER / EQ-STABACT | FC-303 / FC-302 parça kimlikleri, kol / boynuz boyları layout `linkage` | konum sapmaları §10 |
| `clearances` | stabilatör – sabit kök ≥ 6 mm, stabilatör – gövde ≥ 5 mm, dümen kökü – kaporta ≥ 8 mm | denetimde sıfır ihlal (kaporta henüz yok) |
| kaplama (shell) | P-STUBROOT şeridinde TL-265 boynu için yarık (y ≈ 0,245–0,27, x 3,487–3,496, z 0,186–0,240); dikey kök fileto kesim çizgileri | shell modülü — açık konu 1 |

## 7. Mekanizmalar ve kinematik

* **Dümen (uzaysal dört-çubuk):** servo kolu 12 mm (nötrde içe normalin 10° önünde), boynuz bilyesi menteşe ekseninden
  26,8 mm (10° önde), çubuk boyu 79,3 mm. Nötr oran 1,98:1 (spec 2,0), ±25° dümen için servo −58,7° / +55,3° (DA 26
  uzatılmış seçenek ±85°), uçlarda oran 2,98 / 3,77. Bağlı eklemler `rudder_arm_*` (servo mili), `rudder_rod1_*` /
  `rudder_rod2_*` (çubuk, kol ucu bilyesi etrafında iki dönme); Blender ifadeleri 7. derece polinom (dizi tablosundan
  < 1·10⁻³ rad sapma).
* **Stabilatör (düzlemsel dört-çubuk, y = ±0,180):** kol 15 mm, boynuz 37,5 mm, çubuk 105,2 mm; nötr oran 2,50:1,
  −20° / +15° için servo −52,8° / +43,8° (DA 30 ±85°), uçlarda oran 3,18 / 4,10.
* Diziler (`rudder_R/L`, `stabilator_R/L`) 25 durumdur; her durumda çubuğun boynuz ucu boynuz bilyesinde kalır (test
  ≤ 0,05 mm). Süpürme denetimi bu durumları ve tek eklem örneklerini kullanır. Ölçülen en küçük boşluklar (±25°):
  dümen kolu – sabit kaplama 1,15 mm, itme çubuğu – kaplama / arka kiriş 1,41 mm (oluk dudağındaki kol açıklığı ve
  çubuk yuvası sayesinde), dümen burnu – oluk 0,87 mm (2 mm nominal oluk aralığı; menteşe ekseni kesit düzlemlerine
  eğik olduğundan uçta azalır), stabilatör – sabit kök 8,0 mm (R-38 ≥ 6 mm).

## 8. Bağlantı elemanları

| Bağlantı | Eleman | Karşı parça | Adet |
|---|---|---|---:|
| Dikey ön kulak → F-FIN-FRONT | M6 12.9 ISO 4762 | ISO 7040 + pul | 4 |
| Kök parçası flanşı → düğüm yanağı | M6 12.9 | ISO 7040 + pul | 8 |
| Mil uç cıvatası | M6 12.9 | milin dolu ucunda diş | 2 |
| Dikey arka kulak → F-FW-CORNER | M5 12.9 | perde tarafı gövdede diş | 4 |
| Kök parçası ön kulağı → F-STUB-FRONT | M5 12.9 | ISO 7040 + pul | 4 |
| Ventral kulakları → F-VENTRAL | M5 12.9 | ISO 7040 + pul | 3 |
| Stabilatör çapraz cıvatası | M5 12.9 | soket göbeğinde diş | 2 |
| Kök bağlantı flanşları → kiriş gövdeleri, ventral çatal / ped | M5 Ti-6Al-4V | ISO 7040 + pul | 18 |
| Mil kolu sıkma cıvatası | M4 12.9 | ISO 7040 | 2 |
| DA 30 kulakları / beşik tabanı | M4 Ti | ISO 7040 / gömülü insert (FS3670) | 16 |
| Servo kapağı | M4 Ti | yüzer somun plakası | 8 |
| Kızak pabucu | M4 A2-70 ISO 7380 | ISO 7040 | 2 |
| Menteşe çatalı / dil, dümen kolu | M3 Ti | somun plakası / çekirdek cebinde ISO 7040 | 28 |
| DA 26 flanşı, uç kapağı | M3 Ti | helisel insert / somun plakası | 12 |
| Menteşe pimi | ISO 2341-B Ø3 | ISO 1234 kopilya + pul | 6 |
| **Toplam** | | | **119** |

Kompozit yığınlarda vida ekseni yüzey normaline hizalıdır (düz baş ve somun plakası eğri yüzeye tam oturur). Kenar
mesafesi metalde ≥ 2 D, kompozitte ≥ 2,5 D, adım ≥ 3 D: denetimde sıfır ihlal.

## 9. Kütle ve bütçe

| Grup / kalem | Bütçe (kg) | Ayrıntılı tasarım (kg) | Fark (kg) |
|---|---:|---:|---:|
| fins_pair_fixed | 2,782 | 2,640 | −0,142 |
| stabilator_root_stubs_pair | 0,564 | 0,977 (mil yuvaları 0,381 + ön bağlantılar 0,148 dahil) | +0,413 |
| stabilators_pair (+ mil, soket) | 3,368 | 3,042 | −0,326 |
| ventral_fin_bumper_skid | 0,627 | 0,572 | −0,055 |
| **`tail` grubu** | **7,35 ± 0,22 (tavan)** | **7,230** | **−0,120** |
| actuators_rudders_2x_DA26 (+ çubuk, kol) | 0,630 | 0,560 | −0,070 |
| control_surfaces_rudders_pair (+ menteşeler, boynuz) | 0,863 | 0,758 | −0,105 |
| actuators_stabilators_2x_DA30 (+ beşik, mil kolu, rulmanlar) | 1,554 | 1,719 | +0,165 |
| **`controls` grubunun kuyruk payı** | **3,047** | **3,037** | **−0,010** |
| kuyruk bağlantı elemanları (`hardware` grubu) | — | 0,462 | |

Kuyruk grubu tavanın 0,12 kg altında, kumanda payı kalemlerin 0,01 kg altındadır. Kalem içi kaymalar: sabit kök
parçaları kavram kaleminden ağırdır (Ø49 yuva tüpü ve layout F-STUB-FRONT ön bağlantısı kavramda yoktu), stabilatör
panelleri hafiftir; stabilatör tahrik kalemine rulmanlar ve segmanlar (0,14 kg) ile beşik eklenmiştir. Hafifletme
adımları: kovan duvarı 3 → 2 mm, beşik pencereleri, mil kolu göbeği r 17,5 mm, kompozit yığınlarda Ti bağlantı
elemanları.

## 10. Layout'tan / yapı hesabından sapmalar (gerekçeli)

* **F-FIN-FRONT B1/B2:** layout'taki düşey M6 çifti (y 0,1492, z 0,2772 / 0,2592) yerine yatay çift y 0,1400 / 0,1584,
  z 0,2655: kulak boynu sırt şapka aralığından geçerken kulakta 2 D kenar mesafesi ancak bu düzende sağlanır.
* **F-FW-CORNER B3/B4:** M6 + somun yerine 2 × M5 12.9, baş arka kulakta, diş perde tarafındaki gövdede (yuva ile gövde
  arasında somun yeri yok); y / z kulak kenar mesafesine göre (0,1505 / 0,2745 ve 0,161 / 0,2925).
* **F-STUB-FRONT B3/B4:** M6 yerine M5 12.9, y 0,233 → 0,235: 8 mm yuva tabanı ile 6 mm kulak dış kenarı arasında
  20 mm var; M6 her iki tarafta 2 D (12 mm) istiyor, M5 tam 2 D (10 mm) sağlar. Kulak 7,8 mm, çift kesme.
* **Kök parçası ön kirişi 0,141 c:** layout noktası çerçeve düzleminde (0,112 c); kiriş gövdesi kulak plakasının
  arkasında yuva düzlemine alındı (bağlantı düz plaka, ofset yok).
* **F-VENTRAL-* B1:** M6 yerine M5 12.9 (8 mm dilde 2 D kenar mesafesi).
* **Mil duvarı 1,5 mm** (yapı hesabı 25 × 1,2): CNC en küçük et kalınlığı; daha güçlü ve 0,02 kg ağır.
* **Çapraz cıvata M5 12.9** (yapı hesabı metni M6): soket göbeği r 10 mm'de 2 D; T-CROSSBOLT yükü 198 N, M6 izin
  değeri 29,4 kN — M5 ile de MS > 40.
* **DA 26** kasası çıkış milinin üstünde (veri sayfası mil ofseti 17,7 mm), flanş servo kaburgasının altında; **DA 30**
  layout kutusundan 2,5 mm öne, 9,5 mm içe (kol itme çubuğu düzleminde).
* **Dümen nötr oranı 1,98** (spec 2,0): kol / boynuz açıları servo kapağı ve arka kiriş kapağı arasında; sizing
  `rated_margin_min` 1,29 → ≈ 1,27.
* **Kızak pabucu PA12 SLS:** spec.assembly UHMW-PE der, malzeme `spec.materials`'ta yok.

## 11. Açık konular

1. **Kaplama ve çerçeve yarığı (F-STUB-FRONT boynu):** CH-098 yuvası dışa açık olduğundan kulak gövde yanından girer;
   FS3480 dış T başlığında yarık bu modülce geç kesimle açılır (yarık çevresi takviyesi şasi çizim notu), P-STUBROOT
   kaplama şeridinde aynı yarık kaplama modülünce açılmalı. Kalıcı çözüm: `layout.stations.FS3480.cutouts` ve
   `layout.shell` içinde "C-STUBLUG" kesimi (layout_build).
2. **Yapı hesabı güncellemesi:** kök parçası ön kulağı (M5 çift kesme, ezilme), flanş cıvataları, Ø49 × 2 kovan
   (bu notta MS ≈ 6), F-FW-CORNER M5 dişli bağlantı, ventral M5 kulaklar, çapraz cıvata M5 `structures.py`'ye
   işlenmeli.
3. Düğüm göbeğinde DIN 472 segman yivleri ve kapak halkası (şasi CH-095); rulman C0 değerleri T-BRG-IN/OUT tedarik
   şartına göre seçilmeli; rulman / segman kütleleri tahmindir.
4. UHMW-PE `spec.materials`'a eklenirse pabuç malzemesi değiştirilir (geometri aynı).
5. Çırpınma: dümenler kütle dengesizdir, stabilatör mil ekseni 0,26 OAK'ta; flutter analizi ve yer titreşim testi.
6. DA 26 ve DA 30 konum sapmaları layout'a (`systems.actuators.ACT-RUDDER.obb`, `equipment.EQ-STABACT.box`) işlenmeli;
   kablo demeti (H-TAIL) kök kaburgası geçişleri ve uç kapağı anteni sistemler modülünde.
7. Egzoz yönünden HS-STUB folyo kalkanı (PR-522) itki modülünde; kök parçası alt yüzünün egzoz zarfına açıklığı tam
   montaj denetiminde doğrulanacak.

## 12. Doğrulama

```
python3 -m ucav250.analysis.checks --modules chassis,tail --focus YK250-TL,YK250-FC --no-write   # 0 ihlal
python3 -m unittest tests.test_ucav250_tail
python3 -m ucav250.layout_build.build --check        # 0
python3 -m ucav250.analysis.layout_check --check     # 0
python3 -m ucav250.analysis.structures --check       # 0
python3 -m ucav250.analysis.sizing --check           # 0
python3 -m ucav250.blender.build --modules chassis,tail --previews --no-blend --out <dizin>
```
