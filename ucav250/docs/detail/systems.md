# YK-250 HANÇER — Sistemler (SYSTEMS) ayrıntılı tasarımı

Üretici modül: `ucav250/design/systems.py` (`register(reg, spec)`), test: `tests/test_ucav250_systems.py`.
Sözleşme: `ucav250/ARCHITECTURE.md`; tek doğruluk kaynağı: `ucav250/spec.yaml` (`layout.part_numbers.systems`
720–819, `layout.systems.equipment` / `antennas` / `air_data_lights` / `harness` (gövde yolları, delikler,
konnektörler), `layout.rules.boxes` (bölme kutuları), `layout.chassis.members` (aviyonik güverte, bölme duvarları ve
tabanları, kuyu omurga ağları, kaburgalar, sırt kanalı), `layout.chassis.trays`, `layout.chassis.fittings`
(F-RISER-FWD / F-RISER-AFT ve kayış makaraları), `layout.chassis.parachute`, `layout.stations` (yüzler, kesikler,
ok açısı), `layout.shell.panels` (RF pencereleri), `layout.mechanisms.joints` (para_hatch), `mass`, `materials`,
`processes`, `assembly`) ve `data/research/components.yaml`. Yerleşim başka bir modülün geometrisinden okunmaz;
bütün konumlar `spec.layout`'tandır. Kanat / kuyruk üzerindeki öğeler (HASA pitosu, kanat ucu ışıkları) yalnız
ilgili modül kayıttaysa (ebeveynleri varsa) kurulur; gövde dışı tek temas yüzeyleri (burun konisi iç yüzü, alt
kaplama) OML'den (`oml.fuselage_from_spec`) ve layout notlarındaki et kalınlığından (6,0 mm GFRP koni,
`layups.shell_secondary`) türetilir.

Kapsam yalnızca sivil EO/IR gözetleme platformunun aviyonik, elektrik gücü, kablo demeti, anten, hava verisi, ışık ve
kurtarma paraşütü sistemleridir. Sistemde silah, mühimmat, dış yük taşıma ya da bırakma düzeneği yoktur ve
öngörülmemiştir; paraşüt kapağı mandalı yalnız kurtarma paraşütünü açar.

## 1. Özet

| Büyüklük | Değer |
|---|---|
| Parça kimliği (720–819) | `chassis,shell,systems` kaydında 84 (60 satın alma, 24 imalat); kanat kayıttayken + 3 (HASA pitosu, 2 kanat ucu ışığı) = 87 |
| Bağlantı elemanı (`joints.py`, geometri `hardware.py`) | 75 (kanatla + 4 × M4): 30 × ISO 7380 M3, 36 × ISO 4762 M3, 9 × ISO 4762 M4 (A2-70); gömme insert, nyloc somun, ünite M3 dişleri |
| Hareketli eklem | yok (paraşüt kapağının `para_hatch` eklemi kabuk modülündedir; mandal ve konteyner bu eklemin her konumunda ≥ 10 mm uzakta kalır) |
| Tasarım kuralı denetimleri | `python3 -m ucav250.analysis.checks --modules chassis,shell,systems --focus YK250-SY --no-write`: **0 ihlal** (§10) |
| Sistem grubu kütlesi | 12,82 kg (`chassis,shell,systems`); kanatla ≈ 13,03 kg; tavan 14,79 kg (§9) |
| Üçgen sayısı (SY parçaları) | ≈ 130 000 |
| Test | `tests/test_ucav250_systems.py` (§10) |

## 2. Güç sistemi

- **Güç dağıtım birimi (PDU, YK250-SY-720).** VISIONAIRtronics 1000 W (components.yaml `vat_1000w_pdu`, 235 × 195 ×
  54,5 mm, 1,60 kg, IP67, MIL-DTL-38999). Aviyonik güverte (M-DECK-NOSE, CH-022) üstünde altı adet 6061-T6 torna
  burç (SY-721, Ø13 × 3,4 mm, güverte üstü ile PDU tabanı arası) üzerinde durur; altı ISO 4762 M3 cıvata güvertenin
  altından (yan bölme kapaklarından erişim) güvertedeki dökme burçlardan ve burçlardan geçip PDU tabanındaki üretici M3
  dişlerine girer. Cıvatalar omurga yarığının dışında (|y| = 0,085 m) kalır; orta çift, güverte altındaki fren birimi
  kutusunun (EQ-BRAKE_UNIT, askısı güverte altına dayanır) arka kenarından cıvata başı ≥ 3 mm uzakta (x 0,977 m). PDU iletimle soğur (taban – burç – güverte,
  ısı ileten ped); birimin termostatik fan çıkışları isteğe bağlı bir fan için kablolanır (ısıl deneme açık konusu).
- **Ana kontaktör + sigorta bloğu (SY-722) ve 28→12 V DC-DC (SY-724).** Güç anahtarlama bölmesinde, alttan cepli 6061-T6
  montaj plakası (SY-723: 1,6 mm ağ, Ø13 vida göbekleri, 6 mm kenar) üzerinde. Üniteler plakaya alttan, plaka yüzeyiyle
  aynı seviyedeki havşalarda oturan ISO 7380 M3 düğme başlı vidalarla bağlanır (tezgâhta alt montaj); plaka güverteye
  dört M3 cıvatayla, güvertedeki boydan boya dökme insertlere bağlanır. Plaka aynı zamanda ısı yayıcıdır.
- **Tampon batarya (SY-726).** 12S2P Molicel P45B Li-ion, hücre sigortalı havalandırmalı kutu (1,98 kg). TR-FWDBAY
  tepsisi (CH-119) üzerinde 6061-T6 beşik çerçeve (SY-725; 3,4 mm, 6 mm kenar, kayış altında iki çapraz kiriş, ön ve
  arka 6 mm dudak: 9 g ileri çarpma tutması) ve iki adet 304 paslanmaz bağlama kayışı (SY-727 / SY-729, 0,6 × 16 mm,
  iç büküm yarıçapı 0,5 t, kutudan 0,2 mm). Kayış ayakları beşik, tepsi (ve arka kayışta FTS askı köşebendi) üzerinden
  M3 cıvata + tepsi altında nyloc somunla bağlanır. Havalandırma çıkışı P-FWDHATCH'teki alev tutuculu ızgaraya.
- **Bağımsız uçuş sonlandırma birimi (FTS, SY-738).** Tepsinin altında iki şekillendirilmiş 6061-T6 köşebende (SY-739-R/L,
  1,6 mm, iç büküm 3 t) asılı; her köşebendin flanşı tepsi altına batarya cıvatalarıyla sıkışır, bacağı ünitenin yan
  dişlerine iki M3 ile bağlanır. Konnektörü (−x yüzü) bataryayı çıkarmadan erişilebilir.

## 3. Aviyonik

- **Sağ yan bölme rafı (SY-731).** Tek parça CNC 6061-T6: 1,6 mm taban (vida göbekli; Veronte otopilot ve Silvus
  SC4200EP), 4 mm iç dikey ağ, 2 mm raf (vida göbekli; Microhard pDDL2450 ve ping200X). Ünitelerin altında hafifletme
  pencereleri vardır (üniteler göbeklerine oturur). Raf TR-SIDEBAY-R (CH-122) tepsisine altı M3 cıvatayla, tepsi altında
  nyloc somunla bağlanır (yan bölme kapağı erişimi); üniteler rafa alttan, havşalı ISO 7380 M3 vidalarla bağlanır
  (Veronte "M3 mounting screws"). Rafın iç ağı tepsinin omurga duvarı flanş cıvatalarının başlarından 1,2 mm uzaktadır.
- **Uzaktan kimlik vericisi (Dronetag Beacon, SY-736).** FS0300'ün ön yüzüne iki M3 (dökme insert) ile bağlanan SLS PA12
  beşikte (SY-737, RF geçirgen), GFRP burun konisinin içinde, cırt bant + kablo bağı ile tutulur.
- **Mated konnektörler (SY-742…749, SY-733).** Her ünitenin konnektör yüzünde MIL-DTL-38999 seri III fiş + arka kılıf
  (koşum tarafı), layout `envelope_with_connector` payının içinde (pay − 2 mm). Ünite kırılım kabloları (fişten gövdeye)
  montajda düzenlenir.

## 4. Antenler ve hava verisi

| Öğe | Parça | Bağlantı |
|---|---|---|
| GNSS 1 (Calian HC977EXF) | SY-780 | FS1110 ön yüzünde 1,6 mm 6061 braket (SY-740; flanş 2 × M3 insert, kol antenin altında), P-AVHATCH GFRP penceresi altında |
| GNSS 2 | SY-781 | FS-FUEL ön yüzünde aynı tip braket (SY-741), P-GNSS2 penceresi altında (taban çizgisi 1,13 m) |
| Veri bağı anteni A (SWA 1001-202) | SY-782 | burun konisi iç yüzüne yapıştırılmış SLS PA12 kaide (SY-792, ayağı koni iç yüzüne uyar) |
| FTS anteni | SY-787 | aynı tip kaide (SY-793) |
| Transponder bıçak anteni (1090 MHz) | SY-786 | burun altı omurga V yüzeyinde (OML'ye uyan 3 mm taban), 4 × M4 kaplamaya dik, iç yüzde 4 kat PW + bakır ağ zemin düzlemi takviyesi (SY-785, yapıştırılmış), nyloc |
| Isıtmalı pito-statik sonda (Aeroprobe) | SY-788 | burun konisi ucundaki deliğe yapıştırılır (§11 açık konu) |
| HASA pitosu (DroneCAN) | SY-789 | kanat modülünün pito direği kelepçe halkasında (kanat kayıttaysa) |
| Kanat ucu seyir / çakar ışıkları (AveoFlashLP LSA) | SY-790-R/L | kanat ucu kaburgası yüzüne, 2 × M4 boydan boya insert (kanat kayıttaysa), uç kaportası cebinde; vidalar 38 mm aralıkla (tahmin) kaburga gövdesinin en kalın yerinde (IR-09: ön vida kaburganın ince hücum kenarı kesitinde 8,8 mm kenar mesafesindeydi, 2,5 D = 10 mm) |

Anten zarfları layout'taki noktalarda ve boyutlardadır (test); burun konisi içindeki antenler koni iç yüzünden ≥ 2 mm
içeride kalır (layout notu), önlerinde karbon yoktur.

## 5. Kablo demeti

- **Gövde hatları** (`layout.systems.harness`): H-MAIN (Ø20, SY-750), H-COAX (Ø16, SY-751), H-WING (Ø12, SY-752-R/L),
  H-TAIL (Ø16, SY-753-R/L), H-ENGINE (Ø20, SY-754), H-NOSE (Ø12, SY-755). Demetler layout yolları üzerinde, köşeleri
  4 D yarıçapla yuvarlatılmış yuvarlak kesitlerdir; çerçeve düzlemindeki yol köşeleri "düz geçiş" noktasına çevrilir
  (demet kesikten düz geçer, 14 mm sonra bükülür). Layout koridorları içinde kalan ayrıntı değişiklikleri:
  - H-MAIN / H-COAX, 30 mm genişliğindeki yanal kanalda (yük bölmesi arka duvarı ile kuyu ön duvarı arası) 20 mm
    yarıçapla merkez kanala döner (merkez çizgileri layout x'inin 4,6 / 6,2 mm arkasında) ve arka bölmedeki CN-TAIL
    braketine biter;
  - H-WING, ayrıldığı hattın yanında başlar (ortak kesiklerde yan yana), ana kiriş çerçevesinden düz geçer, ana hattın
    üstünden çaprazlar, orta kutu alt kapağından düz geçer ve CN-WING arka kılıfında biter;
  - H-TAIL ve H-ENGINE CN-TAIL'den başlar; H-TAIL FS3480'i halka açıklığının içinden geçer (halkanın dikey kök
    flanşları z 0,21 üstünde |y| 0,110'dan başlar);
  - H-NOSE FS0300 kesiğinden ön yüze 3 mm kalana kadar uzanır.
- **Geçiş rondelaları.** Her çerçevede bir takım (SY-756…765: FS0300, FS0600, FS1110, FS1330, FS1490, FS1810, FS-FUEL,
  FS-MS, FS-RS, FS-GEAR) ve yangın perdesinde yangına dayanıklı rondela (SY-770): kesiği 0,2 mm boşlukla dolduran tıkaç,
  iki yüzde 1,5 mm flanş (çerçeveye gelen elemanlarda kırpılır; FS-GEAR'da arka yüzde iç kapak eyleyicileri,
  ön yüzde C-DOORSHAFT çevresindeki yatak yuvası flanşları için kırpılır), demet delikleri. Eleman geçişleri: orta kutu alt kapağı
  (SY-766-R/L), gövde yanı kaburgası (SY-767-R/L, alt flanş için 1,5 mm kenar), eldiven kaburgası kutu parçası
  (SY-768-R/L, CH-058), ana kuyu ön duvarı (iki delikli sızdırmaz giriş, SY-769; arka flanş omurga ağlarının arasında).
- **Merkez kanal (SY-771).** Ana tekerlek kuyuları arasında, omurga ağlarına (M-WELLKEEL) yapıştırılan 4 kat PW (0,8 mm)
  CFRP U kanal: üst katta H-MAIN, alt katta H-COAX.
- **Arka bölme geçişi (SY-776) ve CN-TAIL (SY-774 / SY-775).** H-MAIN / H-COAX, TR-AFTBAY tepsisinin FS-GEAR
  flanşındaki boşluk çentiğinden ve tabandaki iki yuvadan (bu modül açar) tırmanır; yuvalarda bölünmüş naylon kenar
  koruma şeridi vardır. CN-TAIL: tepsi üstünde 1,6 mm 6061 braket (3 × M4, nyloc), beş geçiş konnektörü (H-MAIN, H-COAX
  önden; H-TAIL-R/L, H-ENGINE arkadan).
- **CN-WING (SY-772-R/L, SY-773-R/L).** Birleşim kaburgasının iç yüzünde 2 mm 6061 yüzer plaka (2 × M3 insert) ve
  MIL-DTL-38999 seri III dişi konnektör (19 pin + 2 koaks); gövde ±0,5 mm yüzer, kanat paneli sokma stroku sonunda kör
  takılır.

## 6. Kurtarma paraşütü

- **UAVOS 200 konteyneri (SY-800).** 0,300 × 0,300 × 0,275 m; ön ve arka üst kenarları R 14 mm yuvarlatılmış (kapağın
  konum dillerine 10 mm açıklık). M-PARAFLOOR üzerine yapıştırılmış dört 6061 ped (SY-808, 25 × 25 × 5,9 mm, neopren
  yüzlü) üzerinde durur; 20 mm kuşak kayışı (SY-807) dört kayış braketinin (CH-114-R/L x 1,56; CH-123-R/L x 1,74)
  köprüsü ile bölme duvarı arasından geçer.
- **Y-kayış.** İki kayış makarası (SY-803 / SY-805, 4130, Ø12 × Ø8,2 × 5 mm, layout `bridle_spool`) Ti-6Al-4V kelepçe
  pimleri (SY-810 / SY-811, Ø8, kulak yüzleriyle aynı hizada, iç segmanlı) üzerinde. Ön ayak (SY-801) dikişli halkasıyla
  ön makarada, katlanmış olarak taban şeritleri arasındaki yuvada; arka ayak (SY-802) arka makaradan sırt kanalı
  M-SPINE içinde (layout_check BRIDLE-AFT-LEG kapsülü) ön bağlantıya kadar istiflenmiştir.
- **Kapak mandalı (SY-804, SY-806).** Pim çekici mandal (30 × 30 × 22 mm, layout tahsis kutusu içinde) FS1810 ön yüzüne
  iki M4 insert ile bağlanan 2 mm 6061 braketin kolunda; kapağa her konumda ≥ 10 mm.

## 7. İmalat

- CNC frezeleme 6061-T6 (ISO 2768-mK): güç anahtarlama plakası, raf, batarya beşiği, PDU burçları, yüzer plakalar,
  pedler; min. et 1,5 mm.
- Abkant 6061-T6 sac (iç büküm yarıçapı 3 t): GNSS braketleri, FTS köşebentleri, CN-TAIL braketi, mandal braketi.
- 304 paslanmaz sac (0,5 t): batarya kayışları. SLS PA12: anten kaideleri, uzaktan kimlik beşiği. OOA prepreg:
  merkez kanal, zemin düzlemi takviyesi. 4130 / Ti-6Al-4V talaşlı: makaralar, pimler.
- Bağlantı elemanları metrik (ISO 4762 / ISO 7380, A2-70), ISO 273 orta boşluk delikleri, kenar mesafesi ≥ 2 D
  (metal / polimer), ≥ 2,5 D (kompozit); kompozit sandviçlerde boydan boya dökme insert, ince laminelerde (tepsiler)
  nyloc somun.

## 8. Montaj (spec.assembly)

| Adım | İş |
|---|---|
| 19 | demetler, rondela takımları, merkez kanal, tepsi geçiş şeritleri, CN-TAIL, CN-WING |
| 20 | güç anahtarlama plakası + kontaktör / DC-DC, PDU burçları + PDU, batarya beşiği + kayışlar, FTS köşebentleri + FTS, raf + aviyonik üniteler (tezgâhta alt montaj), fişler |
| 21 | GNSS braketleri + antenler, burun konisi kaideleri + antenler, uzaktan kimlik beşiği, transponder bıçak anteni + takviye, pitolar, ışıklar |
| 22 | konteyner pedleri, konteyner, kuşak kayışı, makaralar + pimler + Y-kayış ayakları, mandal braketi + mandal |

## 9. Kütle (registry) ve bütçe

`chassis,shell,systems` kaydında sistem grubu (yalnız bu modülün parçaları) 12,82 kg; kanat kayıttayken HASA pitosu
(0,048) ve iki kanat ucu ışığı (0,166) eklenir: ≈ 13,03 kg. Bütçe tavanı `mass.budget.systems` = 14,79 kg ✓.
Kalem eşlemesi (mass.items, temel / büyüme dahil):

| Kalem | Temel kg | Modelde |
|---|---|---|
| avionics | 0,62 | üniteler + antenler + pitolar (veri sayfası); yedek dipol ve FTS anteni bu modülde yer almaz (§11) |
| buffer_battery_12S2P_liion | 1,98 | SY-726 |
| pdu_dcdc_fuses | 2,10 | SY-720 / 722 / 724 |
| wiring_harness_connectors_coax | 2,70 | demetler + fişler + konnektörler + rondelalar = 2,70 (kablo %70 uzunluk × d², konnektör / rondela %30) |
| parachute_uavos_200 | 4,50 | konteyner + iki kayış ayağı |
| flight_termination_lights | 0,399 | FTS + 3 ışık (kuyruk ışığı kurulmadı, §11) |
| montaj donanımı | — | plakalar, raf, braketler, kayışlar, pedler, kanal, takviye: ≈ 0,55 kg (kalemlerin %5 büyüme payından) |

Uyarı: `turret_lift_mechanism_doors` kalemi (1,786 kg temel / 1,875 kg büyüme dahil) de sistem grubundadır ve yük
(payload) modülünce üretilecektir; bu modülün parçalarıyla birlikte grup ≈ 14,9 kg olur — tavanın ~0,1 kg üstü
(§11 açık konu: montaj donanımı hafifletmesi veya bütçe yeniden dağılımı).

## 10. Doğrulama

- Tasarım kuralı denetimleri (`chassis,shell,systems`, odak YK250-SY): mesh 0, statik 0, süpürme 0, açıklık 0, kalınlık
  0, bağlantı elemanı 0, temas 0, bağlanma 0.
- Tam uçak (chassis, wing, tail, shell, propulsion, fuel, gear, systems): §11'deki durum.
- Önizlemeler (`python3 -m ucav250.blender.build --modules chassis,systems --previews --no-blend`) incelendi.
- `tests/test_ucav250_systems.py`: denetimler, sözleşme, layout kutuları / noktaları / delikleri, kesik rondelaları,
  bağlantı elemanları, kütle.

## 11. Açık konular

1. **Kuyruk ucu öğeleri (kapandı, entegrasyon turu IR-06):** ANT-C2B (sol) / ANT-BKP (sağ) Southwest Antennas 1001-202 dipolleri artık dikey kuyruk açıklığı boyunca (22° eğik) uç kaburgasının üstünde, GFRP uç kapağının içinde durur: kapak eklemi üstünde 133 mm açıklık, 109 mm radom; taban %42 veterde, TNC geçiş jakı uç kaburgasındaki delikten geçer, altta kontra somun (jak kütlesi 0,012 kg tahmin). Polarizasyon 22° eğik (düşey yer antenine −0,3 dB). LT-TAIL (AveoFlash LSA, 100 × 45 × 30 mm) sağ uç kapağının uç yüzüne, %26–54 veterde (merkez %40), uç yüzünün 0,3 mm üstünde oturur; 2 × M4 (38 mm aralık, tahmin: veri sayfasında delik düzeni yok) kapaktan geçip kuyruk modülünün GFRP bağlantı bloğu TL-292'nin (%26–56 veter) gömme burçlarına girer (kanat ucu ışıklarıyla aynı M4; veri sayfası M5); iki vida da blok uçlarından ve kesit kalınlığından ≥ 2,5 D kenar mesafesi tutar. Koaks ve ışık beslemesi yeni kol H-FIN'de (SY-794, iki taraf, Ø8): H-TAIL ucundan sırt kirişi M-DORSAL'ın altından geçip kirişin şapka kesitinin iki gövdesi arasından (y 0,140 / 0,160; x 3,600, y 0,150; gövdelere bağlanır; layout'ta M-DORSAL geçişi olarak bildirilir), F-FW-CORNER sırt eki dilinin (x ≥ 3,620) önünde dikey köküne girer, kutu içinde ön kiriş gövdesi ile dümen eyleyicisi (DA 26, FC-303) arasında %27 veter çizgisinde (η 0,40–0,58) yükselir, servo kaburgasının üstünde %42 veter çizgisine geçer ve uç kaburgasının 6,5 mm altında biter (jak + 4 mm kontra somun + koaks fişi); geçtiği kuyruk kaburgalarına (kök, kutu, servo) kol + 1,5 mm delik açılır (kalıplı yarık rondela, çizim notu). layout_check C03, kuyrukta bildirilen kolları (in_tail) dikey gövdesinde de ölçer.

2. **Y-kayış ön ayak güzergâhı:** C-BRIDLE-F, şasi çerçevesinde yalnız başlık boşluğu olarak yapılmıştır; FS1810 web üstü
   ile kapak arasında 2,3 mm kalır. Ayakların konteyner ağzına serbest uzantısı modellenmedi; çentiğin ağda açılması
   (şasi) gerekir.
3. **Burun pitosu ekseni** (layout z −0,050) sarkık burun ucunun alt OML'sindedir (x 0,02'de OML z −0,0507…−0,0376):
   sonda konide yarı açık bir yuvaya oturur, pnömatik hatlar modellenmedi. Önerilen layout değişikliği: sonda ekseni burun
   merkez çizgisinde, GFRP / alüminyum uç göbeği.
4. **Mandal tahsisi:** layout `mount.tray` CH-006 (FS1490) yazar; mandal kapağın arka kenarında olduğundan FS1810'a
   (CH-007) bağlandı.
5. **TR-AFTBAY:** şasi tepsisinin FS-GEAR flanşı C-HARN-CL kesiğini örter ve taban demet geçişi içermez; bu modül
   tepside çentik + yuva açar (şasi çiziminde işlenmeli).
6. **Yakıt seviye probu kolları (kapandı, IR-07):** H-FUEL (SY-777, Ø6) H-COAX'tan görev bölmesinde ayrılır, H-COAX demetinin yüzeyinden radyal olarak ayrılır (demetler çakışmadan temas eder), sırt kanalı M-SPINE'ın tabanından contalı yarık rondela (SY-779; 0,8 mm tapa duvarı, 2 mm flanşlar sancak duvarında 0,3 mm içeride düz kesilmiş D biçimi) ile kanala girer ve sancak duvarı boyunca (y 0,0165, paraşüt kayışına 5,5 mm) ön ve eyer hücreleri üstünde x 2,775'e kadar uzanır; algılayıcıların uçuş kabloları kanal duvarındaki sızdırmaz bağlantılara gelir (yakıt bölmesinin içinde kol yok). Arka hücre algılayıcısı sol arka köşeye alındı (x 2,98, y −0,10); kablosu FS-GEAR'daki C-HARN-FUELA kesiğindeki sızdırmaz rondelaya (bağlantı arka yüzde; ön yüzde, yakıt bölmesi astarı tarafında rondela flanşı yok), oradan H-FUEL-A (SY-778, Ø6) arka ekipman bölmesinde CN-TAIL önündeki H-MAIN ayrımına gider.

7. **Isıl doğrulama:** PDU / DC-DC / kontaktör iletimle soğuyor; deneme ile fan gereksinimi belirlenecek.
8. **Kütle:** payload modülünün sistem grubuna ekleyeceği taret mekanizmasıyla grup tavanı ~0,1 kg aşabilir (§9).
9. Satın alma ünitelerinin montaj dişleri (taban M3) ve konnektör yerleri üretici çizimleriyle doğrulanacak; FTS bandı ve
   anteni, yedek C2 frekans planı (layout açık konuları) seçime bağlıdır.
