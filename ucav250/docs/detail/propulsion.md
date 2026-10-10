# YK-250 HANÇER — İtki sistemi (PROPULSION) ayrıntılı tasarımı

Üretici modül: `ucav250/design/propulsion.py` (`register(reg, spec)`), test: `tests/test_ucav250_propulsion.py`.
Sözleşme: `ucav250/ARCHITECTURE.md`; tek doğruluk kaynağı: `ucav250/spec.yaml` (`layout.part_numbers.propulsion`
500–569, `layout.chassis.engine_mount` (itki ekseni, bağlantı yüzü, cıvata deseni, takoz merkezleri, kaplar),
`layout.stations` FS3480 / FS3670 kesikleri, `layout.keep_outs` KO-ENGINE / KO-EXHAUST-R/L / KO-COOLING-DUCT / KO-PROP,
`layout.mechanisms.joints[prop_spin]`, `layout.systems.equipment` EQ-ECU / EQ-GENERATOR_PE, `layout.fuel_lines`,
`layout.heat_protection`, `layout.clearances`, `layout.shell.rules`, `engine`, `propeller`, `materials`, `processes`,
`assembly`) ve `data/research/components.yaml` (Volz DA 22, ePropelled SG750). Modül başka bir modülün geometrisini
okumaz; bütün arayüzler `spec.layout` üzerindendir.

Kapsam yalnızca sivil EO/IR gözetleme platformunun itki sistemidir: Limbach L 275 EF motoru, marş-jeneratörü, itici
pervanesi, egzozu, soğutma hava yolu ve ısı koruması. İtki sisteminde silah, mühimmat, dış yük taşıma ya da bırakma
düzeneği yoktur ve öngörülmemiştir.

## 1. Özet

| Büyüklük | Değer |
|---|---|
| Parça kimliği (500–569) | 27 (25 parça numarası; egzoz ve düğüm perdesi sağ/sol) + koşullu 4 (PR-520-R/L, PR-523-R/L, alt kaporta yarımları kayıtlı olduğunda) |
| Grup `propulsion` toplamı | 30 parça (şasi modülünün ürettiği yangın perdesi katmanı PR-088 / -091 / -094 dahil) |
| Bağlantı elemanı (`joints.py`, geometri `hardware.py`) | 65: 4 × M8×40 12.9, 6 × M6×55 12.9 (pervane), 10 × M6×16 12.9, 4 × M4×35 12.9, 4 × M4×12 12.9, 8 × M4×14 A2-70, 4 × ISO 7380 M4×8, 6 × M5×10 A2-70, 1 × ISO 7380 M5×14, 4 × M3×6 A2-70, 6 × Ø2,4 paslanmaz (Monel) perçin, 8 × Ø2,4 alüminyum perçin; koşullu +84 perçin (ısı koruması) |
| Hareketli eklem | 1: `prop_spin` (layout; 0 … 120°, üç pala simetrisi), PR-501 / -540 / -541 / -542 / -543 / -544 döner |
| Tasarım kuralı denetimleri | `python3 -m ucav250.analysis.checks --modules chassis,propulsion --focus YK250-PR --no-write`: **0 ihlal** (mesh, statik, süpürme, açıklık, kalınlık, bağlantı elemanı, temas, bağlanma); `chassis,tail,propulsion` ile de 0 ihlal |
| Isı koruması (vekil alt kaporta ile) | 0 ihlal (test `TestHeatProtection`) |
| İtki grubu kütlesi (registry) | **12,350 kg**; ısı koruması ile **13,097 kg** — bütçe tavanı 14,54 ± 0,44 kg (−1,44 kg, bkz. §10) |
| Üçgen sayısı (PR parçaları) | 104 802 |
| Test | `tests/test_ucav250_propulsion.py`: 24 test, hepsi geçer (~80 s) |

Önizlemeler (Blender Workbench, motor bölümü yakın çekimleri: izometrik, arka izometrik, yan, üst, alt, arka,
patlatılmış; ısı koruması dış/iç görünüş; S-kanalı ağzı) incelendi: parçalar şasi bağlantılarına oturur, yüzen ya da
kopuk parça, görünür girişim yoktur.

## 2. Motor ve motor yatağı

**Motor (PR-500, satın alma; zarf modeli KO-ENGINE içinde).** Motor çerçevesi: u pervane düzleminin önünde krank ekseni
boyunca, v = BL, w = motor üstü; itki ekseni layout'tan 5° aşağı eğimli. Modelde: karter (v ±58 mm, w −72 … +46 mm, 6 mm
döküm duvar, içi boş), karter arka yüzünde layout cıvata deseninde dört M8 göbeği (Ø33, kılavuz çekilmiş), simetrik iki
kanatçıklı silindir (eksen u = 245 mm; boxer silindirlerin gerçek eksenel kaçıklığı modellenmedi), cıvatalı ısı
yutuculu kafalar, buji + başlık (KO-ENGINE genişliği 396,8 mm), silindir alt yüzünde egzoz port göbekleri (port Ø19,
çevresinde boyun + 2 × M6 kulak), karter altında piston portlu emme manifoldları, ortak mil üzerinde gaz kelebekleri,
enjektörler + yakıt rayı, yarım küre tel elek hava filtreleri, arka çıkış (jeneratör kaplini) yuvası, plenum ve braket
göbekleri. Kütle: `engine.installed_items_kg.engine_bare` 7,0 kg − çıkış makarası.

**Çıkış makarası (PR-501, `prop_spin`).** Ø73 × 11 mm flanş (engine.yaml pervane flanşı tahmini 72 ± 5 mm), krank
burnundaki deliğe 0,1 mm radyal çalışma geçmeli göbek; ara parçanın 6 × M6 cıvatası için kılavuzlu.

**SG750 marş-jeneratörü (PR-502, 0,44 kg, components.yaml) ve adaptörü (PR-503, 7075, 10 mm).** Adaptör arka yuvaya
4 × M4 havşalı (kılavuzlu), SG750 göbeğinden 4 × M4 geçme cıvata adaptöre; adaptör yarıçapı 29,5 mm (motor halkasına
≥ 13 mm, dinamik pay 10 mm), SG750 kablo çıkışı +v tarafında.

**Titreşim takozları (PR-506 … -509, satın alma).** Konik elastomer Ø40 → Ø30 × 24,5 mm (layout zarfı Ø40 × 25), çelik
burç; şasi motor kaplarına (CH-085) oturur. Her biri M8×40 12.9 geçme cıvata: baş + pul kap tabanının altında, takozdan
geçer, karter göbeğinde 12,5 mm diş boyu (kılavuz derinliği 14,5 mm, ≥ 1,2 D). Kütle: hacim × 1200 kg/m³ (tahmin; spec'te
elastomer malzemesi yok).

## 3. Pervane grubu (`prop_spin`)

| Parça | Yapı |
|---|---|
| PR-541 göbek ara parçası 110 mm | 7075-T651 talaşlı: ön flanş Ø73 × 8 (6 × M6, PCD 48, makaraya), boru Ø70/65, arka flanş Ø88 × 10 (pervane cıvataları için 6 × M6 kılavuz, PCD 62), Ø22 ön delik, arka plaka ve pervane göbeği için Ø22 merkezleme pimi, ön flanş cıvatalarına altıgen anahtar erişim delikleri |
| PR-543 spinner arka plakası | 7075, 2 mm, pervane göbeğinin altında sıkışır |
| PR-540 Mejzlik 31x12 3B | tek parça karbon (0,412 kg, veri sayfası); pala planformu `propeller.blade_model`, kesit NACA 4410 benzeri (tahmin), kökler göbek diskinde |
| PR-544 baskı plakası + spinner dayanağı | 7075, 5 mm Ø88 plaka; entegre Ø16 × 2 dayanak borusu ve Ø21 uç göbeği (M5 için 2 D) |
| PR-542 spinner | CFRP PW 0,8 mm, Ø150 × 180 mm (spec), pala yuvaları (3 mm boşluk), uçta 4 mm göbek; ISO 7380 M5 merkez vidası dayanağa |

Pervane cıvataları: 6 × M6×55 12.9, yığın [baskı plakası, pervane, arka plaka] → ara parça arka flanşı (kılavuz), tork
9 N·m. Ara parça → makara: 6 × M6×16 12.9. Süpürme denetimi `prop_spin` 0 … 120° (9 örnek) temiz; statik parçalar
pervane diski zarfının dışında; layout açıklıkları (yapıya radyal ≥ 26 mm, boyuna ≥ 13 mm) sağlanır.

## 4. Egzoz (PR-504-R/L)

AISI 304 kaynaklı yapı (her taraf): 6 mm port flanşı (lazer kesim, 2 × M6×16 port göbeğine), Ø22 × 0,8 boyun, Ø38 × 94 mm
susturucu kabı (x 3,803 … 3,897, KO-EXHAUST kutusu 1 içinde), Ø20 × 0,8 çıkış borusu: kutu 1'de aşağı, 2 D yarıçaplı
90° dirsek, kutu 2 içinde arkaya, 1,5 D yarıçaplı son dirsekle layout çıkış noktasına (3,95; ±0,205; −0,005) layout
yönünde (dışa / 60° aşağı). Boru kaportayı x ≈ 3,877'de keser (çıkış levhası PR-520 yuvasından geçer) ve son ~70 mm
kaporta dışındadır. Ölçülen paylar: kompozit / kuyruk parçalarına ≥ 68 mm (kural 50 mm), dinamik zarf payı motorla
birlikte hareket eder.

## 5. Soğutma hava yolu

- **S-kanalı (PR-546, CFRP 4 kat PW 0,8 mm, bölünmüş erkek kalıp).** KO-COOLING-DUCT koridoru (üç 80 mm boru, 1 mm
  içeride), son 55 mm'de 176 × 69 mm (r 20) çıkış kesitine yassılaşır ve C-DUCT'tan geçer. **Ağız:** ilk koridor
  aralığında (x 3,30 … 3,40, layout yol noktaları) tavan açıktır; yan duvarlar OML'nin 7,9 mm altına kadar yükselir
  (P-INLET 5,8 mm + bindirme derinliği 0,3 mm + 1,6 mm iniş + 0,2 mm yapıştırma çizgisi, `layout.shell.rules`),
  12 mm içe dönük yapıştırma flanşları dudak inişine; x 3,40'ta (boğaz) tavan başlar, üstünde bölme duvarı.
  Arka uç paslanmaz ağız parçasının üzerine 35 mm geçer (4 × ISO 7380 M4 + somun plakası).
- **Geçiş kanalı (PR-548, AISI 304 0,5 mm, büküm + dikiş kaynağı).** C-DUCT içinden yanmaz silikon-cam contalı ağız;
  flanşı yangın perdesi kalkanına (PR-088) 6 × Ø2,4 Monel perçin; motor yatağı kafesinin üzerinden 5 mm payla yükselir.
- **Bağlantı körüğü (PR-550, satın alma).** Silikon kaplı cam kumaş 2 mm, iki kelepçe, takoz hareketi için 25 mm serbest
  boy; kütle hacim × 1500 kg/m³ (tahmin). Serbest kesit 0,0079 m² = kanal çıkışının %67'si (bkz. açık konular).
- **Plenum ve silindir perdeleri (PR-552, 5052-H32 0,8 mm, motora bağlı).** Tavan kaporta OML'sinin 15,8 mm içinde
  (5,8 mm kabuk + 10 mm dinamik pay), etekler kanatçık paketlerinden 1 mm (silikon sızdırmazlık şeridi), hava
  kanatçıklardan aşağı akar; 2 × M5 karter, 2 × M5 kafa göbeklerine.
- **Düğüm ısı perdeleri (PR-524-R/L, AISI 304 0,4 mm, 70 × 60 mm).** `layout.heat_protection.hardware[F-SPINDLE-NODE]`;
  silindir kafası ile stabilatör düğüm göbeği arasında, plenum ön eteğine 4'er Ø2,4 alüminyum perçin.

## 6. Motor yardımcıları ve ekipman

- Gaz eyleyicisi Volz DA 22 (PR-514; 41,5 × 65,9 × 22 mm, 0,132 kg, components.yaml) karter altında 5052 1,5 mm braket
  (PR-515) üzerinde: braket 2 × M5 karter göbeklerine, eyleyici 4 × M3; çıkış mili gaz kelebeği miline doğrudan.
- Yakıt hortumları (PR-516 besleme, PR-517 dönüş): -4 PTFE astarlı, yangın kılıflı, kıvrık uçlu; layout FL-FEED-3 ucundan
  / FL-RETURN başından ray / regülatör nipellerine (10 mm düz uç); takoz hareketini alır. Kütle 0,15 kg/m (tahmin).
- Motor kontrol birimi (PR-510, layout EQ-ECU kutusu ve kütlesi 0,8 kg, ateşleme bobinleri payı dahil) görev tepsisinde
  (CH-120) 4 × M4 + nyloc; jeneratör güç elektroniği (PR-512, EQ-GENERATOR_PE 0,4 kg) sol yan bölme tepsisinde (CH-118)
  4 × M4 + nyloc.
- Yakıt pompası / regülatör / filtre `fuel` grubundadır (FU-584, 0,35 kg); bu modülde yoktur.

## 7. Isı koruması (`layout.heat_protection`)

Isı koruması parçaları alt kaporta yarımlarına (YK250-SH-451-R/L, kabuk modülü) bağlanır; yalnız bu yarımlar
registry'de varsa kaydedilir (yoksa günlüğe not düşülür). Test, layout'tan türetilmiş bir vekil kaporta (5,8 mm
sandviç, kesik çevresinde 30 mm 1,6 mm katı kenar bandı, ek bölgeleri 0,5 mm boşlukla kesilmiş) ile doğrular.

- **HS-COWL-EXIT, egzoz çıkış levhası (PR-520-R/L, AISI 304 0,4 mm).** Kaporta kesiğinin (layout ek bölgeleri =
  egzoz güzergâh kutuları + 25 mm; kesiği kabuk açar) dışında, OML'nin 0,1 mm üstünde, 20 mm bindirme payı ile; bindirme
  sırası kesik kenarının 10 mm dışında Ø3,2 Monel kör perçin, ~25 mm adım (sağ 25 adet), kaportanın 1,6 mm katı kenar
  bandından geçer. Çıkış borusu yuvası boru çevresinde 10 mm radyal boşluk (motor takozlar üzerinde hareket eder;
  `clearance_values.engine_keep_out`).
- **HS-COWL-SHIELD, kaporta ısı kalkanı (PR-523-R/L, AISI 304 0,4 mm).** Zarfın 25 … 50 mm bandında, kaportanın en
  kalın laminatının 5 mm altında; kesik kenarının 5,5 mm içinde dönüş duvarı ve levhanın altına 14 mm flanş (Ø2,4 Monel
  kör perçin, ~35 mm adım, dışarıdan çakılır). Kalkan levhaya asılıdır: kaporta iç yüzü ve kesik kenarı, kenar takviyesi
  ne olursa olsun ≥ 5 mm hava boşluğu tutar.
- **HS-STUBROOT (PR-521), HS-STUB (PR-522): gerekli değil.** Ölçülen paylar: kök şeridi egzoz zarfına 73 mm, stabilatör
  kök parçası ≥ 50 mm (kural: kalkansız kompozite 50 mm) — kalkan kaydedilmez.

## 8. Üretim

| Süreç | Parçalar |
|---|---|
| Talaşlı imalat 7075-T651 (`cnc_milling_metal`) | PR-503 adaptör, PR-541 ara parça, PR-543 arka plaka, PR-544 baskı plakası |
| Paslanmaz sac / boru (`sheet_metal_steel`, AISI 304) | PR-504 egzoz (kıvrılmış kap, bükülmüş boru, TIG), PR-548 geçiş kanalı, PR-520 / -523 / -524 (hidroform / el ile şekillendirme, segmentli dikiş kaynağı) |
| Alüminyum sac (`sheet_metal_aluminium`, 5052-H32) | PR-515 braket, PR-552 plenum (büküm yarıçapı ≥ 1,5 t, perçinli) |
| CFRP ön-emdirilmiş, vakum torbası (`prepreg_ooa_vacbag`) | PR-542 spinner, PR-546 S-kanalı |
| Satın alma | PR-500 / -501 motor, PR-502 SG750, PR-506 … -509 takoz, PR-510 ECU, PR-512 güç elektroniği, PR-514 DA 22, PR-516 / -517 hortum, PR-540 pervane, PR-550 körük |

Bütün metal kalınlıkları süreç alt sınırlarının üstündedir (sac paslanmaz ≥ 0,38 mm). Çok parçalı yığınlarda ISO 273
ince seri delikler; kenar mesafeleri ≥ 2 D (metal) / 2,5 D (kompozit), adım ≥ 3 D (denetimle ölçülür; ince eğrisel
ısı koruması saclarında kenar mesafesi `Part.outline` ile gerçek kesik kenarına ölçülür, büküm çizgileri kenar sayılmaz).

## 9. Montaj (spec.assembly)

| Adım | İş |
|---|---|
| 20 | ECU görev tepsisine, jeneratör güç elektroniği sol yan bölme tepsisine (4 × M4 + nyloc) |
| 25 | Takozlar kaplara; motor (adaptör + SG750 önceden takılı) 4 × M8 12.9 ile, tork + emniyet teli; itki ekseni 5° ± 0,2° mastarla; dinamik zarf ile yapı ≥ 10 mm |
| 26 | Egzozlar (2 × M6 / taraf), geçiş kanalı (perçin), S-kanalı (4 × M4), körük + kelepçeler, plenum (4 × M5), düğüm perdeleri, DA 22 + braket, yakıt hortumları; perde geçişleri kapatılır |
| 27 | Ara parça (6 × M6), arka plaka, pervane, baskı plakası (6 × M6, 9 N·m), spinner + M5; pala izi ≤ 0,5 mm, balans |
| 36 | (koşullu) Isı koruması alt kaporta yarımına alt montaj olarak perçinlenir, kaporta kapatılır |

Patlatma vektörleri: motor grubu itki ekseni boyunca 0,30 m arkaya; yardımcılar aşağı/yana; pervane grubu arkaya.

## 10. Kütle ve bütçe

| Kalem (spec.mass.items, büyüme dahil) | Bütçe (kg) | Model (kg) |
|---|---|---|
| engine_group_installed (motor 7,0, ECU 0,8, pompa 0,35, SG750 0,44, PE 0,4, kaplin 0,15, egzoz 1,0, takoz + cıvata 0,7, servo + sensör 0,12) | 11,508 | 9,604 (pompa `fuel` grubunda; takoz cıvataları `hardware` grubunda; egzoz 0,399) |
| propeller | 0,433 | 0,412 |
| spinner_hub_adapter_spacer | 0,525 | 0,701 (ara parça 0,387 + arka plaka 0,096 + baskı plakası 0,123 + spinner 0,095) |
| cooling_baffles_firewall_cowl_flap | 1,804 | 1,450 + ısı koruması 0,748 (brüt) = 2,198 |
| dorsal_cooling_inlet_s_duct | 0,263 | 0,183 |
| **Grup toplamı** | **tavan 14,54** | **12,350 / ısı koruması ile 13,097** |

Grup tavanın 1,44 kg altındadır. Kalem bazında: spinner grubu +0,18 kg (ara parça 110 mm boyunda ve iki flanşlı) ve
soğutma/ısı koruması +0,39 kg (0,4 mm sac, layout'taki 0,1 mm folyonun yerine; brüt kütle) aşar; egzoz (0,40 kg'a
karşı 1,0 kg pay) ve motor yatağı kalemleri bunları karşılar.

## 11. Doğrulama

- `python3 -m ucav250.analysis.checks --modules chassis,propulsion --focus YK250-PR --no-write` → 0 ihlal.
- `tests/test_ucav250_propulsion.py` (24 test): denetimler temiz, tekrar kayıt yok sayılır, kimlik aralığı ve
  aynalama, layout'un sabitlediği kimlikler, parça sözleşmesi (malzeme, süreç, kalınlık, adım, patlatma, ebeveyn), silah
  ifadesi yok, kapalı ağlar, motor bağlantı cıvataları layout noktalarında, motor dinamik zarf içinde ve yapıya ≥ 10 mm,
  egzoz güzergâh kutularında ve layout çıkışında, kompozitlere ≥ 50 mm, pervane eklemi ve planform, statik parçalar disk
  dışında, S-kanalı koridorda / ağız OML altında / tavan açık / C-DUCT'tan geçer, soğutma kesit oranı, bağlantı
  elemanları, kütleler bütçeye karşı, satın alma verileri components.yaml'dan; `TestHeatProtection` (vekil kaporta):
  kayıt ve ebeveynler, denetimler temiz, perçinler sacları deler, hava boşluğu ≥ 5 mm, boru boşluğu ≥ 10 mm, kaporta
  ≥ 25 mm.
- Layout / yapı / boyutlandırma denetimleri (`layout_build.build --check`, `layout_check`, `structures --check`,
  `sizing --check`) değişmeden geçer; layout'a dokunulmadı.

## 12. Sapmalar ve kararlar

1. **Şasi düzeltmesi (bildirilir).** `chassis.build_engine_mount` içinde kap köşebentlerinin ucu kap duvarında kesildi
   (kap boşluğu Ø41, takoz yuvası): köşebentler takoz kaplarına 120–257 mm³ giriyordu. Şasi testleri geçer.
2. Silindirler simetrik modellendi (boxer eksenel kaçıklığı yok); egzoz portları layout güzergâhına göre silindir alt /
   dış yüzünde.
3. Takoz yüksekliği 24,5 mm (layout zarfı 25 mm eksi 0,5 mm oturma payı: taban kap zeminine hacim çakışması olmadan
   oturur).
4. Isı koruması 0,4 mm sac (layout folyosu 0,1 mm, `sheet_metal_steel` alt sınırı 0,38 mm); çıkış levhası gömme yerine
   dıştan bindirmeli (kaportada bindirme cebi gerekmez); kalkan kaporta üzerindeki mesafe parçaları yerine levhaya asılı
   dönüş duvarlı; boru çevresi boşluğu 5 mm halka yerine 10 mm (dinamik pay).
5. PR-521 / PR-522 kaydedilmedi (ölçülen paylar yeterli).
6. S-kanalının ağzı ilk koridor aralığında koridordan yukarı, P-INLET dudak inişinin altına kadar çıkar (hava girişinin
   bağlantısı; koridorun üstünde başka parça yok, denetlendi).

## 13. Açık konular

1. **Limbach montaj resmi:** egzoz / emme port konumları (layout egzozu alt-dış yanda, emmeyi altta öngörür), silindir
   kaçıklığı, karter bağlantı göbeği deseni, krank burnu flanşı (Ø72 ± 5 tahmin) doğrulanmalı.
2. **Takoz seçimi ve motor yatağı dinamiği:** takoz rijitlik verisi yayımlanmamış; egzoz çıkışında sallanma genliği
   10 mm boşlukla karşılaştırılmalı.
3. **SG750 bağlantı deseni ve kaplin** (adaptör tahmini), Mejzlik göbek deseni / ara parça onayı, dönüş yönü (CW/CCW),
   balans.
4. **Soğutma akış denetimi:** kanal çıkışı 0,0118 m², körük kesiti 0,0079 m² (%67, motor yatağı kafesinin üstünde
   daralma); giriş alanı 0,031 m² ilk tahmin — basınç kaybı ve silindir sıcaklığı hesabı yapılmalı.
5. **Kabuk arayüzü:** (a) SH-451 alt kaporta yarımları `heat_protection.inserts[HS-COWL-EXIT].regions` bölgelerini
   kesmelidir (geliştirilmekte olan kabuk modülü henüz kesmiyor: `chassis,shell,propulsion` çalıştırmasında egzoz /
   kaporta girişimi ve 25 mm kuralı ihlali bundan kaynaklanır; kesik uygulandığında egzoz–kaporta 29,4 mm, kalkan–kaporta
   ≥ 3,5 mm, perçin satırları temiz); (b) SH-451 dış yüzü x 3,96–3,97'de OML'nin ~0,8 mm dışına çıkıyor (iç bükey
   bölgede seyrek x örneklemesi; levha bindirmesiyle 63 mm³ çakışma); (c) kabuğun NACA ağzı x 3,205'te başlıyor,
   koridor 3,30'da — 3,205 … 3,30 rampası koridor dışında, layout'ta karara bağlanmalı; (d) S-kanalı flanşlarının dudak
   inişine yapışma teması, kabuk inişleri oluşunca beyan edilecek.
6. **Egzoz:** susturucu akustik / geri basınç performansı doğrulanmadı; kap ve boru port flanşından konsol (2 × M6);
   titreşim / yorulma ve gerekirse esnek bağlantı.
7. Ateşleme bobinlerinin konumu (ECU payında sayıldı), DA 22 bağlantısı ve kelebek mili kaplini tahmindir.
8. Spinner grubu +0,18 kg ve ısı koruması brüt kütlesi (0,748 kg; layout net 0,388 kg, diğer kalemler dahil) boyutlandırma
   kalemlerine yansıtılmalı.
9. Perde geçişleri: S-kanalı ağzı C-DUCT'ta yanmaz conta ile kapatılır; yakıt hatlarının ve kablo demetinin perde
   geçişleri yakıt / sistem modüllerindedir.
