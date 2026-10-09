"""Layout builder: spec.assembly (general rules, step list, transport, field assembly, maintenance access)."""
from __future__ import annotations

GENERAL = [
    "Montaj sırası şasi -> sabit kabuk -> sistemler -> itki -> iniş takımı -> kuyruk ve kanat -> kabuk kapanışı -> "
    "ayar ve fonksiyon testleri şeklindedir. Her adımın parça listesi ve bağlantı elemanları, ayrıntılı tasarım "
    "modüllerinin Part.step / Fastener.step değerlerinden outputs/assembly_guide.py ile üretilir.",
    "Kök parça orta kanat kutusudur (layout.root_part = YK250-CH-001). Bütün çerçeveler, uzun kirişler ve "
    "bağlantılar ana montaj tezgâhında bu kutuya göre konumlanır; ölçü referansı spec.yaml çerçevesidir (X burundan "
    "geriye, Y sancak, Z yukarı; Z = 0 kenar çizgisi düzlemi).",
    "Genel toleranslar ISO 2768-mK; geçmeler ISO 286 (pim yuvaları H8/h8, yatak yuvaları H7). Kritik yuvalar "
    "(kanat çatalı pim delikleri, ana takım ve burun takımı mafsal yuvaları, stabilatör mil yatakları) tezgâh "
    "üzerinde, mastarla birlikte hat raybalama ile işlenir.",
    "Yapıştırma: EA 9394, yüzey hazırlığı (zımpara + çözücü silme), 0,2 mm yapıştırıcı kalınlığı, kür süresi "
    "boyunca tezgâh baskısı; her yapışma bölgesi tanık numunesiyle birlikte kürlenir ve tıklama testiyle denetlenir.",
    "Kompozit parçalarda bağlantı elemanı merkezi ile kenar arası >= 2,5 D, metal parçalarda >= 2,0 D; delikler "
    "ISO 273 orta sınıf boşlukla, matkap şablonundan delinir; sandviç panellerde bağlantı hattında çekirdek "
    "1:3 eğimle dolu laminata iner.",
    "Sıkma torkları kuru dişler içindir; vida sabitleyici (orta mukavemet) ve emniyet teli/kopilyası çizimde "
    "belirtilen yerlerde zorunludur. Her tork uygulaması tork işaretiyle (renkli çizgi) işaretlenir.",
    "Yakıt bölmeleri buhar sızdırmazdır; yakıt bölmesinden kablo geçmez. Li-ion batarya yakıt hücrelerinden "
    "en az 1 m uzaktadır. Egzoza 50 mm'den (ısı kalkanıyla 25 mm) yakın kompozit parça yoktur.",
    "Elektriksel bağlama: motor, yakıt sistemi, metal bağlantılar ve antenler bağlama hattına bağlanır (CS-VLA 857: "
    ">= 1,3 mm² bakır; geçiş direnci <= 2,5 mΩ). Yakıt ikmal noktasında topraklama ucu vardır.",
    "Kapsam meta.scope_tr ile tanımlıdır: yalnızca sivil EO/IR gözetleme ve araştırma. Dış yük taşıma bağlantısı "
    "yoktur; gövde altındaki yük bölmesi yalnızca araştırma sensörleri içindir.",
]


def step(n, title_tr, sub, text, tools, checks):
    return {"step": n, "title_tr": title_tr, "subassembly": sub, "text": text, "tools": tools, "checks": checks}


def steps() -> list:
    S = []
    S.append(step(1, "Ana montaj tezgâhının hazırlanması", "şasi tezgâhı",
                  "Ana tezgâh çerçeve istasyonlarına göre kurulur: her çerçeve için iki konum pimi ve tabla, orta "
                  "kanat kutusu için kök destekleri, kanat çatalı ve ana takım mafsal mastarları. Tezgâh lazer "
                  "izleyiciyle spec.yaml çerçevesine bağlanır (X burundan geriye, Z = 0 kenar çizgisi düzlemi).",
                  ["lazer izleyici", "tezgâh mastarları (çatal, mafsal, motor bağlantısı)", "tork anahtarı seti"],
                  ["tezgâh referans noktaları ±0,2 mm", "çerçeve istasyonları ±0,3 mm (layout.stations)",
                   "mastarların kalibrasyon etiketi geçerli"]))
    S.append(step(2, "Orta kanat kutusunun yerleştirilmesi (kök parça)", "orta kanat kutusu YK250-CH-001",
                  "Ana ve arka kiriş başlıkları kesintisiz, ortada kırık bağlantısıyla (7075-T651) birleştirilmiş "
                  "orta kanat kutusu tezgâhın kök desteklerine yerleştirilir ve kilitlenir. Kutunun ana kiriş hattı "
                  "ve arka kiriş hattı tezgâh mastarlarına göre doğrulanır.",
                  ["vinç askısı (kutu kaldırma noktalarından)", "lazer izleyici"],
                  ["ana kiriş hattı x (merkezde) = layout.stations FS-MS ±0,3 mm", "kutu simetrisi ±0,5 mm",
                   "kırık bağlantısı cıvataları torklu ve işaretli"]))
    S.append(step(3, "Ana ve arka kiriş çerçeveleri", "FS-MS, FS-RS",
                  "Ana kiriş ve arka kiriş çerçeveleri (kutunun üstünde yakıt bölmesi duvarı, altında yük bölmesi "
                  "dışındaki dikmeler) kiriş gövdelerine yapıştırılır ve M6 cıvatalarla bağlanır.",
                  ["yapıştırıcı tabancası (EA 9394)", "matkap şablonları", "tork anahtarı"],
                  ["yapışma tanık numunesi", "çerçeve dikliği ±0,5 mm", "yakıt bölmesi duvar yüzeyi sızdırmazlık için "
                                                                      "temiz"]))
    S.append(step(4, "Gövde yanı, eldiven ve birleşim kaburgaları", "eldiven yapısı (L/R)",
                  "Gövde yanı kaburgası (y = ±0,40 m), eldiven kaburgası (±0,55 m) ve birleşim kaburgası (±0,70 m) "
                  "kutunun kirişlerine yapıştırılır ve cıvatalanır. LERX burun kaburgası yoktur: pim, pim çekici ve "
                  "rayba koridorları (layout.mechanisms.assembly_paths) çatalın önünde serbesttir.",
                  ["kaburga konum mastarları", "yapıştırıcı"], ["kaburga istasyonları ±0,3 mm",
                                                                "birleşim kaburgası yüzü düzlemsellik 0,2 mm"]))
    S.append(step(5, "Kompozit kanat çatalı burçları ve arka kiriş yuva bağlantısı", "kanat birleşim bağlantıları (L/R)",
                  "Kompozit çatalın (eldiven ana kiriş kutusu, YK250-CH-053) pim bölgelerindeki dört 4130 burç "
                  "(16 H8 / dış çap 22) usta dil mastarı takılıyken yapıştırılır ve kürden sonra önden hat raybalanır; "
                  "rayba koridoru serbesttir (LERX kaburgası yok, eldiven kaplamaları henüz takılı değil). Arka kiriş "
                  "yuva bağlantısı (YK250-CH-054, iki 4 mm 7075 plaka, Ø8 H8) aynı mastara göre birleşim kaburgasına "
                  "2 x M4 ve arka kiriş gövdesine 2 x M5 Ti ile bağlanır.",
                  ["usta dil mastarı", "Ø16 H8 ve Ø8 H8 raybalar", "matkap kılavuzu"],
                  ["iki pim deliği arası 0,184 m ±0,05 mm (kiriş ekseni boyunca)",
                   "burç merkezi ile çatal ağzı arası >= 2,5 D (burç dış çapı)", "usta dil sıkışmasız kayar, boşluk "
                                                                                 "<= 0,1 mm"]))
    S.append(step(6, "Ön gövde çerçeveleri", "FS0300, FS0600, FS1110, FS1330, FS1490, FS1810, FS-FUEL",
                  "Burun ve orta gövde perde/çerçeveleri tezgâh pimlerine yerleştirilir. Kablo, pito ve yakıt geçiş "
                  "kesikleri layout.stations listesine göre açılmış olmalıdır.", ["tezgâh pimleri", "lazer izleyici"],
                  ["her çerçeve istasyonu ±0,3 mm", "kesiklerin konumu ve kenar payı"]))
    S.append(step(7, "Burun kutusu: omurga duvarları, aviyonik güverte, burun takımı mafsalı",
                  "burun kutusu", "İki omurga duvarı FS0600 ile FS1110 arasına, aviyonik güverte bunların üstüne "
                  "yapıştırılır. Burun takımı mafsal burç blokları (7075) her iki duvara bağlanır ve Ø16 H7 yuvalar "
                  "mafsal mastarıyla birlikte hat raybalanır. Yukarı kilit kancası güvertenin altına takılır.",
                  ["mafsal mastarı", "Ø16 H7 rayba", "yapıştırıcı"],
                  ["burun takımı mafsal ekseni = landing_gear.nose.pivot ±0,2 mm", "omurga yarığı iç genişliği "
                                                                                     "0,083 m ±0,3 mm"]))
    S.append(step(8, "Taret, paraşüt ve görev bölmeleri", "bölme duvarları ve tabanları",
                  "Taret bölmesi yan duvarları ve tavanı, paraşüt bölmesi duvarları ve tabanı, görev bölmesi tabanı "
                  "yerleştirilir. Taret bölmesi kapak rayları için duvar alt kenarındaki yarık bırakılır.",
                  ["yapıştırıcı", "kare mastar"], ["taret bölmesi iç ölçüsü 0,212 x 0,212 m", "paraşüt bölmesi iç ölçüsü "
                                                                                         ">= 0,300 x 0,312 m"]))
    S.append(step(9, "Kenar çizgisi uzun kirişleri", "YK250-CH-020 (L/R)",
                  "Kenar çizgisi uzun kirişleri FS0600'den ana kiriş çerçevesine ve arka kiriş çerçevesinden yangın "
                  "perdesinin ön yüzüne uzanır; kutu bölgesinde gövde yanı kaburgasına 2 x 4 M6 ile eklenir. Arka uçtaki "
                  "7075 uç bağlantısı adım 13'te perdeden geçen cıvatalarla stabilatör düğüm bağlantısına eklenir; "
                  "motor bölmesine karbon parça geçmez.", ["matkap şablonları", "tork anahtarı"],
                  ["kenar çizgisi hattı OML'ye göre 20 mm içeride ±0,5 mm", "ek cıvataları torklu"]))
    S.append(step(10, "Omurga kirişleri, güverteler ve ana takım yapısı",
                   "omurga kirişleri, ön yakıt güvertesi, kuyu tavanı, ana takım kirişleri, mafsal bağlantıları",
                   "Yük bölmesi kenarındaki iki omurga kirişi, ön yakıt bölmesi tabanı, kuyu tavanı ve iki ana takım "
                   "kirişi yerleştirilir. 7075 mafsal bağlantıları (YK250-CH-070) kirişe ve tavana bağlanır; Ø20 H7 "
                   "yatak yuvaları mafsal mastarıyla hat raybalanır.", ["ana takım mafsal mastarı", "Ø20 H7 rayba"],
                   ["mafsal ekseni = landing_gear.main.trunnion ±0,2 mm, x eksenine paralel 0,05°",
                    "iki taraf simetrisi ±0,3 mm"]))
    S.append(step(11, "Arka gövde: sırt uzun kirişleri, kuyruk ve ventral bağlantıları",
                   "FS-GEAR, FS3480, sırt uzun kirişleri, arka omurga kirişi, kuyruk bağlantıları",
                   "FS-GEAR ve FS3480 çerçeveleri, sırt uzun kirişleri (yangın perdesinin önünde biter), ventral "
                   "omurga şeridi ve arka omurga kirişi takılır. Dikey ön kiriş bağlantısı, sabit kök parçası ön "
                   "bağlantısı ve üç ventral bağlantısı matkap şablonlarından delinerek bağlanır.",
                   ["kuyruk bağlantı şablonları", "tork anahtarı"],
                   ["dikey kök bağlantı noktaları layout.chassis.fittings ±0,3 mm", "ventral bağlantıları aynı hatta"]))
    S.append(step(12, "Yangın perdesi ve motor bağlantı parçaları", "FS3670",
                   "Kompozit sandviç perde takılır; 12 paslanmaz ara parça üzerine 0,5 mm AISI 304 kalkan perçinlenir. "
                   "İki köşe bağlantısı (motor üst ayağı 2 x M8, dikey arka kiriş çatalı 2 x M6, sırt uzun kirişi eki) "
                   "ve iki alt motor ayağı (2 x M8) 7075 destek plakaları ve paslanmaz ara borularla perdeye bağlanır. "
                   "Kablo, yakıt ve itme çubuğu geçişlerine yanmaz rondela/körük takılır.",
                   ["perçin tabancası", "tork anahtarı"], ["paslanmaz kalkan >= 0,38 mm (FIRE-001), açık delik yok",
                                                           "motor bağlantı parçası konumları ±0,3 mm"]))
    S.append(step(13, "Stabilatör düğüm bağlantıları ve motor bölmesi alt U halkası", "F-SPINDLE-NODE, FS3738",
                   "İşlenmiş 7075 stabilatör düğüm bağlantıları (iç yatak yuvası işlenmiş halde) mil hizalama "
                   "fikstürüyle (sahte mil + FS3480 kök parçası ön bağlantısı deliklerine bağlanan şablon) "
                   "konumlanır ve 7 x M5 ile yangın perdesinden geçirilerek bağlanır; kenar çizgisi uzun kirişi uç "
                   "bağlantıları aynı cıvatalarla eklenir. 2024-T3 alt U halka takılır ve uçları düğümün dış yanaklarına "
                   "perçinlenir. Dış yatak yuvası kök parçası uç kaburgasında adım 31'de düğüm yuvasıyla eş eksenli "
                   "işlenir.",
                   ["mil hizalama fikstürü (iki taraf)", "tork anahtarı"],
                   ["düğüm yatak ekseni = tail.surfaces.stabilator.pivot ±0,2 mm, y eksenine paralel 0,05°",
                    "düğüm - motor dinamik zarfı >= 10 mm"]))
    S.append(step(14, "Paraşüt, taret asansörü, yakıt bölmesi ve teçhizat bağlantıları",
                   "kayış bağlantıları, asansör ray bağlantıları, yakıt bölmesi astarları, tepsiler",
                   "Sırt omurga kanalı (M-SPINE) FS1810 ile arka kiriş çerçevesi arasına, çerçeve kesiklerinden "
                   "geçirilerek sızdırmaz yapıştırılır ve vidalanır; paraşüt ön ve arka kayış bağlantıları kanalın iki "
                   "ucuna (taban 4 x M5, çerçeve 2 x M5), konteyner bağlama braketleri, taret asansör ray ve motor "
                   "bağlantıları, yakıt bölmesi astarları ve hücre askıları, aviyonik/arka teçhizat tepsileri takılır.",
                   ["tork anahtarı"], ["kayış bağlantıları 6 x M5 12.9 torklu", "astarlarda keskin kenar yok"]))
    S.append(step(15, "Şasi muayenesi ve tezgâhtan ayırma", "şasi",
                   "Şasi lazer izleyiciyle ölçülür, yapışmalar tıklama/ultrason testiyle denetlenir, şasi tartılır ve "
                   "elektriksel bağlama hattının sürekliliği ölçülür. Şasi kendi kendini taşır; tezgâh ayrılır, şasi "
                   "kaplama tezgâhına alınır.", ["lazer izleyici", "UT cihazı", "terazi", "mikro-ohmmetre"],
                   ["ölçü raporu ±0,5 mm", "şasi kütlesi <= mass.budget.chassis", "bağlama direnci <= 2,5 mΩ"]))
    S.append(step(16, "Sabit gövde kaplamaları", "yapısal kaplamalar",
                   "Yapısal kaplamalar (burun, orta, merkez, arka) çerçeve ve uzun kiriş flanşlarına M4 somun plakalı "
                   "vidalarla 25-35 mm aralıkla bağlanır; görev bölmesi üst kaplaması sırt kanalı flanşlarına da "
                   "vidalanır (kayış x yükünün kesme yolu). Delikler kaplama şablonlarından delinir. Yakıt bölmesi "
                   "çevresinde sızdırmazlık macunu kullanılır.", ["kaplama şablonları", "somun plakası perçin aleti"],
                   ["kenar payı >= 2,5 D", "kaplama basamağı <= 0,3 mm, aralık 1,0 ±0,3 mm"]))
    S.append(step(17, "LERX/eldiven kaplamalarının yapıştırılması", "LERX/eldiven (L/R)",
                   "Birincil sandviç LERX/eldiven üst ve alt kaplamaları kaburgalara, kiriş başlıklarına ve kenar "
                   "çizgisine yapıştırılır; birleşim erişim kapağı için çerçeveli kesik bırakılır.",
                   ["vakum torbası", "ısıtıcı battaniye", "yapıştırıcı"],
                   ["yapışma tanık numunesi", "hücum kenarı profil mastarı ±0,3 mm"]))
    S.append(step(18, "Yakıt sistemi", "yakıt hücreleri ve hatları",
                   "Üç esnek hücre (ön, eyer, arka) erişim kapaklarından yerleştirilir; ara bağlantı, havalandırma, "
                   "dolum (sol kuru bağlantı), besleme/dönüş hatları, pompa/filtre, kesme valfi (perdenin önünde) ve "
                   "seviye algılayıcıları bağlanır. Basınç ve sızıntı testi yapılır.",
                   ["basınç test seti", "tork anahtarı"],
                   ["test basıncı CS-LUAS.965(a)'ya göre, sızıntı yok", "hücreler perdeye >= 13 mm",
                    "bağlama sürekliliği"]))
    S.append(step(19, "Elektrik tesisatı", "kablo demetleri",
                   "Ana ve koaksiyel demetler layout.systems.harness yollarından, her çerçevede rondelalı geçişle "
                   "döşenir; ana tekerlek kuyuları arasındaki merkez kanalı kapatılır; kanat ve kuyruk konnektörleri "
                   "takılır.", ["kablo bağı aleti", "megger", "süreklilik test cihazı"],
                   ["süreklilik ve yalıtım direnci", "demet egzozdan >= 50 mm", "hareketli parçalara >= 10 mm"]))
    S.append(step(20, "Aviyonik ve güç dağıtımı", "aviyonik güverte ve yan bölmeler",
                   "PDU, kontaktör/sigorta bloğu ve DC-DC aviyonik güverteye; Li-ion tampon batarya havalandırmalı "
                   "kutusuyla ön bölmenin üst kısmına, bağımsız uçuş sonlandırma birimi onun altına; jeneratör güç "
                   "elektroniği ile fren eyleyicisi + ana silindir sol yan bölmeye; otopilot, veri bağları, transponder "
                   "sağ yan bölmeye, uzaktan kimlik vericisi burun konisi perdesinin ön yüzüne takılır; motor ECU'su "
                   "görev bölmesinin sökülebilir ekipman tepsisine bağlanır.",
                   ["tork anahtarı", "ESD bileklik"],
                   ["bağlayıcı payları serbest (layout.rules.bay_contents)", "batarya havalandırma çıkışı ön bölme kapağında",
                    "güç-açık testi: bara gerilimleri"]))
    S.append(step(21, "Antenler ve hava verisi", "antenler, pito",
                   "GNSS antenleri GFRP pencerelerin altına, veri bağı ve FTS antenleri burun konisine, transponder "
                   "bıçak anteni burun altına (bakır ağ zemin düzlemiyle), uzaktan kimlik vericisi burun konisinin içine "
                   "yerleşir; anten zarfları iç kaplama yüzeyinden en az 2 mm içeridedir. Burun pitosu ve kanat HASA sondası bağlanır, pito-statik hatlar sızdırmazlık testinden "
                   "geçer.", ["pito-statik test seti", "VSWR ölçer"],
                   ["VSWR <= 2,0", "pito-statik sızıntı yok", "anten önünde karbon yok (RF pencere)"]))
    S.append(step(22, "Paraşüt sistemi", "UAVOS 200 + Y-kayış",
                   "Paraşüt konteyneri bölmeye bağlanır; Y-kayışın ön ayağı FS1810, arka ayağı arka kiriş çerçevesi "
                   "bağlantısına takılır; arka ayak sırt kanalına yerleştirilip yırtılır örtüyle kapatılır. Menteşesiz "
                   "kapak dört köşe pimiyle oturtulur, pim çekici mandal, kapak bağlama ipi (1,5 m) ve tetik hattı "
                   "(FTS) bağlanır; emniyet pimleri takılı kalır.", ["kayış uzunluk mastarı"],
                   ["kayış ayak uzunlukları layout.chassis.parachute.bridle", "emniyet pimleri takılı (kırmızı flama)"]))
    S.append(step(23, "Görev bilgisayarı ve yük tepsisi rayları", "görev bölmesi, yük bölmesi",
                   "Görev bilgisayarı/kayıtçı ve motor ECU'su sökülebilir ekipman tepsisine bağlanır; tepsi alttan "
                   "P-MBHATCH açıklığından kaldırılıp taban kesiğinin kenar takviyelerine 8 x M4 ile vidalanır. Yük "
                   "tepsisi rayları ön güvertenin altına takılır.",
                   ["tork anahtarı"], ["ray paralelliği 0,2 mm"]))
    S.append(step(24, "Taret asansörü, kayar kapaklar ve HD59", "taret bölmesi",
                   "Asansör rayları, bilyalı vida ve motor takılır; kayar kapaklar raylarına, her kapağın DA 22 "
                   "eyleyicisi FS1330'un arka yüzüne, alt kaplamadaki P-TDOORACC kapağından (FS1330-FS1490 arası, kapak "
                   "süpürme bandının dışında) bağlanır; tahrik mili C-TDOOR-SHAFT kesiğinden geçer. Açıklık halkası "
                   "(HD59, 14 x M4 gömme dişli burç) takılır, "
                   "taret taşıyıcıya bağlanır. Kapak konum algılayıcıları ve asansör sınır anahtarları kilitlemesi "
                   "ayarlanır; asansör 10 tam çevrim çalıştırılır.",
                   ["asansör test kutusu"],
                   ["asansör yalnız iki kapak tam açık algılanınca hareket eder, kapaklar yalnız tam içeride kapanır",
                    "top ile halka arası >= 5 mm",
                    "içeride top ile kapak iç yüzü arası bütün top izdüşümünde >= 10 mm (payload.turret.bay.door_clearance)"]))
    S.append(step(25, "Motorun takılması", "L 275 EF + SG750",
                   "4130 kafes ve titreşim takozları perde bağlantı parçalarına bağlanır; motor 4 x M8 ile karter arka yüzündeki "
                   "göbeklere takılır. İtki ekseni 5° aşağı eğim mastarla doğrulanır.", ["motor askısı", "tork anahtarı",
                                                                                         "eğim mastarı"],
                   ["M8 civataları torklu + emniyet teli", "itki ekseni eğimi 5° ±0,2°", "motor dinamik zarfı ile "
                                                                                       "yapı arası >= 10 mm"]))
    S.append(step(26, "Egzoz, soğutma, motor yardımcıları", "itki sistemi",
                   "Egzoz/susturucular, soğutma kanalı ve silindir perdeleri, yakıt pompası/filtresi, gaz servosu ve "
                   "motor demeti bağlanır; motor demeti adım 20'de takılan ECU'ya (görev bölmesi) ve jeneratör güç "
                   "elektroniğine (sol yan bölme) bağlanır; perde geçişleri kapatılır.",
                   ["tork anahtarı", "sıcaklık etiketi"], ["egzoz-kompozit arası >= 50 mm (kalkanlı 25 mm)",
                                                           "yangın kılıfları takılı"]))
    S.append(step(27, "Pervane göbeği, pervane ve spinner", "pervane",
                   "110 mm göbek ara parçası, Mejzlik 31x12 3B pervane ve spinner takılır; cıvatalar torklanır, pala "
                   "izi ve balans ölçülür.", ["tork anahtarı", "pala izi mastarı", "balans cihazı"],
                   ["pala ucu izi <= 0,5 mm", "pervane düzlemi x = propeller.plane_x", "yapıya radyal >= 26 mm, "
                                                                                          "boyuna >= 13 mm"]))
    S.append(step(28, "Ana iniş takımı", "ana bacaklar (L/R)",
                   "Bacaklar mafsal yataklarına takılır, döner EMA'lar ve kilitler bağlanır; tekerlek/fren, hidrolik "
                   "fren hatları ve bacak kapakları takılır.", ["kriko seti", "tork anahtarı", "fren hava alma seti"],
                   ["aşağı/yukarı kilit çalışması", "fren testi", "tekerlek-kuyu arası >= 12 mm (tüm hareket)"]))
    S.append(step(29, "Burun iniş takımı", "burun bacağı",
                   "Burun bacağı mafsala, döner EMA sağ omurga duvarına, direksiyon eyleyicisi bacağa takılır.",
                   ["kriko seti", "tork anahtarı"], ["direksiyon ±20°, toplama öncesi merkezleme", "kilit çalışması"]))
    S.append(step(30, "Takım kapakları ve sıralama", "iç kapaklar, istiridye kapaklar",
                   "İç kapaklar menteşe millerine, DA 22 eyleyicileri FS-GEAR arka yüzüne bağlanır; burun istiridye "
                   "kapakları takılır. Kapak contaları ve kapak kapalı anahtarları ayarlanır; tam toplama-açma dizisi "
                   "çalıştırılır.", ["kriko seti", "ayar mastarları"],
                   ["dizi: kapak açık -> bacak hareketi -> kapak kapalı", "kapalı kapak yüzey basamağı <= 0,5 mm"]))
    S.append(step(31, "Stabilatör kök parçaları, milleri ve eyleyicileri", "stabilatör tahriki (L/R)",
                   "Sabit kök parçaları FS3480 bağlantısına ve düğüm bağlantısının dış yanağına (4 x M6) takılır; dış "
                   "yatak yuvası düğüm yuvasıyla eş eksenli işlenir (mil hizalama fikstürü); miller iki yatağa "
                   "yerleştirilir. "
                   "DA 30 eyleyicileri yangın perdesinin ön yüzüne bağlanır; itme çubukları yanmaz körüklerden "
                   "geçirilip mil kollarına takılır.", ["tork anahtarı", "açı ölçer"],
                   ["kök boşluğu 8 mm (R-38)", "bağlantı oranı 2,5:1 nötrde", "hareket sırasında sürtünme yok"]))
    S.append(step(32, "Dikey kuyruklar ve dümenler", "dikeyler (L/R)",
                   "Dikeyler ön ve arka kiriş kök bağlantılarına takılır; dümen eyleyicileri ve demetler bağlanır; "
                   "uç kapakları (GFRP, antenli) takılır.", ["tork anahtarı"],
                   ["dikey eğimi 22° ±0,2°", "dümen kökü kaportaya >= 8 mm (±25°)"]))
    S.append(step(33, "Ventral kanatçık ve tampon kızağı", "ventral",
                   "Ventral kanatçık arka omurga kirişine üç bağlantıyla, değiştirilebilir UHMW-PE kızak 4130 şerit "
                   "üzerine takılır.", ["tork anahtarı"], ["kızak pervane diskinin altında (R-48)"]))
    S.append(step(34, "Stabilatörlerin takılması", "stabilatörler (L/R)",
                   "Stabilatörler mil uçlarına geçirilir ve çapraz cıvatayla emniyetlenir.", ["tork anahtarı"],
                   ["-20° / +15° hareket serbest", "kök boşluğu sabit 8 mm"]))
    S.append(step(35, "Dış kanat panelleri (ilk takma)", "dış paneller (L/R)",
                   "Panel ana kiriş ekseni boyunca içeri sürülür; kompozit dil çatala, arka kiriş kulağı yuva "
                   "bağlantısına ve kör takılan kanat konnektörü yuvasına aynı anda girer. İki Ø16 ana pim önden "
                   "(P-JOINTACCESS), Ø8 arka pim alttan (P-REARACCESS, bilyalı kilit) takılır, emniyet plakaları "
                   "vidalanır; kanatçık ve flap eyleyicileri kontrol edilir.", ["kanat sehpası (2 adet)", "pim itici", "tork anahtarı"],
                   ["pimler tam oturmuş, emniyet plakaları takılı", "birleşim contası sürekli", "konnektör kilitli"]))
    S.append(step(36, "Kabuğun kapatılması", "kaportalar, kapaklar, radom",
                   "Kaporta parçaları (alt yarımlar, üst yan parçalar, üst orta parça; dikey ve kök parçası kök "
                   "şeritleri sabit), aviyonik kapağı, yan bölme kapakları, görev ve yük bölmesi kapakları, arka "
                   "teçhizat kapağı, stabilatör eyleyici kapak yarımları, birleşim erişim kapakları ve burun konisi "
                   "kapatılır.", ["Camloc anahtarı",
                                                                                            "tork anahtarı"],
                   ["her Camloc kilitli (işaret çizgisi)", "kapak basamağı <= 0,3 mm"]))
    S.append(step(37, "Kumanda yüzeylerinin ayarı", "kumanda",
                   "Kanatçık, flap, dümen ve stabilatör nötr konumları, hareket aralıkları ve yönleri ayarlanır; "
                   "boşluklar ölçülür.", ["açı ölçer", "kumanda yer istasyonu"],
                   ["aralıklar wing.controls / tail.*.controls.range_deg", "boşluk <= 0,5° (eyleyici) + 0,3° (bağlantı)",
                    "yön testi (her yüzey)"]))
    S.append(step(38, "Fonksiyon testleri", "bütün sistemler",
                   "Takım toplama/açma dizisi (5 çevrim), taret indirme/kaldırma (10 çevrim), paraşüt kapağı mandal "
                   "testi (piroteknik olmadan), ışıklar, veri bağları, uçuş sonlandırma uçtan uca, yakıt sistemi ve "
                   "motor yer çalıştırması (CHT sınırlı) yapılır.", ["yer istasyonu", "yangın söndürücü", "takozlar"],
                   ["bütün diziler hatasız", "CHT <= 240 °C yer çalıştırmasında"]))
    S.append(step(39, "Tartı ve denge", "uçak",
                   "Uçak üç teraziyle (burun ve iki ana teker) boş ve yükleme durumlarında tartılır; ağırlık merkezi "
                   "stability.cg_range_x ve R-09 ile karşılaştırılır.", ["3 platform terazi", "eğim ölçer"],
                   ["boş kütle <= mass.budget toplamı", "ağırlık merkezi zarf içinde", "yanal AM <= 5 mm"]))
    S.append(step(40, "Son muayene ve kabul", "uçak",
                   "Bütün tork işaretleri, emniyet telleri, kapak kilitleri ve belgeler denetlenir; uçuşa elverişlilik "
                   "dosyası tamamlanır.", ["kontrol listesi"], ["açık bulgu yok"]))
    S.append(step(41, "Taşıma için sökme", "taşıma",
                   "Pervane, stabilatörler ve dış kanat panelleri sökülür (assembly.transport); paneller kılıflarına, "
                   "orta gövde taşıma beşiğine konur.", ["kanat sehpası", "taşıma beşiği"],
                   ["pimler ve emniyet plakaları torbasında", "bütün açık konnektörler kapaklı"]))
    S.append(step(42, "Sahada montaj", "saha",
                   "Dış paneller, stabilatörler ve pervane takılır (assembly.field_assembly); uçuş öncesi denetimler "
                   "yapılır.", ["saha alet çantası", "tork anahtarı"],
                   ["pim ve emniyet plakaları", "kumanda yön testi", "pervane torku ve emniyet"]))
    return S


def transport() -> dict:
    """Transport units with sizes from the closed geometry (sizing.Airframe): centre body = body + glove + fins, stubs
    and ventral (fins stay on), gear retracted, no cradle; outer panel incl. the 0.25 m tongue."""
    import math as _m

    import numpy as _np

    from .b_common import P, PR, S as _S, YJ, af, r3

    def _c(v):
        return f"{v:.2f}".replace(".", ",")

    def ext(srf):
        sc = srf.span_coords()
        Q = _np.vstack([srf.loop_at(e, 41) for e in _np.linspace(sc[0], sc[-1], 15)])
        return Q.min(0), Q.max(0)
    lo_f, hi_f = ext(af.tail["fin"])
    lo_v, hi_v = ext(af.tail["ventral"])
    lo_s, hi_s = ext(af.tail["stabilator"])
    xs = _np.linspace(af.fus.x0 + 1e-4, af.fus.x1 - 1e-4, 200)
    z_low = min(min(float(af.z_bot(x)) for x in xs), float(lo_v[2]))
    L_c = max(float(af.fus.x1), float(hi_f[0]), float(hi_v[0]))
    W_c = max(2 * YJ, 2 * float(hi_f[1]))
    H_c = float(hi_f[2]) - z_low
    b2 = 0.5 * float(_S["wing"]["span"])
    root = af.wing.interpolate_section(YJ)
    from .b_chassis import D_S, Y_TONGUE_TIP
    tongue = round((YJ - Y_TONGUE_TIP) / float(D_S[1]), 3)   # CFRP tongue beyond the joint plane (fix round 1)
    # fix round 2 (PK2-12): outer-panel envelope from the loft (wing.split at the joint) in the panel frame (span along
    # the dihedral, chord along x, normal) plus the protrusions: tongue (inboard), wing pitot (ahead of / below the LE)
    dih = _m.radians(float(P["dihedral_deg"]))
    e_s, e_n = _np.array([0.0, _m.cos(dih), _m.sin(dih)]), _np.array([0.0, -_m.sin(dih), _m.cos(dih)])
    outer = af.wing.split([YJ])[-1]
    sc = outer.span_coords()
    Q = _np.vstack([outer.loop_at(e, 61) for e in _np.linspace(sc[0], sc[-1], 25)])
    from .b_systems import air_data_lights
    pit = next(a for a in air_data_lights() if a["id"] == "AD-PITOT-WING")
    rp = 0.5 * float(pit["diameter"])
    PP = _np.array([pit["p0"], pit["p1"]], float) * [1.0, -1.0 if pit["p0"][1] < 0 else 1.0, 1.0]
    PP = _np.vstack([PP + [0.0, 0.0, dz] for dz in (-rp, rp)] + [PP + [dx, 0.0, 0.0] for dx in (-rp, rp)])
    A = _np.vstack([Q, PP])
    u, sp, nn = A[:, 0], A @ e_s, A @ e_n
    L_o = float(sp.max() - sp.min()) + tongue
    C_o = float(u.max() - u.min())
    T_o = float(nn.max() - nn.min())
    c_loft, t_loft = float(Q[:, 0].max() - Q[:, 0].min()), float((Q @ e_n).max() - (Q @ e_n).min())
    R_p = 0.5 * float(PR["diameter"])
    return {
        "units": [
            {"unit": "centre body with LERX/glove, fins, stubs, ventral, gear, engine",
             "size_m": r3([L_c, W_c, H_c], 2),
             "mass_note": "empty mass minus outer panels, stabilators, propeller",
             "support": "transport cradle on the FS1810 and FS-GEAR lower frame lands (two padded saddles), gear "
                        "retracted (height without cradle) or on the gear with chocks; fin tips under padded covers"},
            {"unit": "outer wing panel (x2)", "size_m": r3([L_o, C_o, T_o], 3),
             "basis": f"loft of the outer panel (wing.split at y {YJ}) in the panel frame: span {L_o - tongue:.3f} m along "
                      f"the dihedral + tongue {tongue:.3f} m; chord-wise extent {c_loft:.3f} m (sweep and taper) and "
                      f"normal extent {t_loft:.3f} m (twist, dihedral frame) of the loft; with the wing pitot probe "
                      f"(AD-PITOT-WING, ahead of and below the leading edge) included {C_o:.3f} x {T_o:.3f} m; the "
                      "rear-spar lug (44 mm) lies inside the tongue length; control horns stay inside the coves "
                      "(fix round 2, PK2-12)",
             "case_inner_m": r3([L_o + 0.05, C_o + 0.05, T_o + 0.05], 2),
             "support": "padded rack / case (inner size = envelope + 25 mm foam each side, design choice), panel on its "
                        f"leading edge with the pitot removed or in a cut-out; tongue cap and rear-lug cap (length incl. "
                        f"the {tongue:.2f} m tongue)"},
            {"unit": "stabilator (x2)", "size_m": r3([float(hi_s[1] - lo_s[1]), float(hi_s[0] - lo_s[0]),
                                                      float(hi_s[2] - lo_s[2])], 2),
             "support": "foam box; root socket cap"},
            {"unit": "propeller (3 blades, one piece)", "size_m": r3([2 * R_p * _m.sin(_m.radians(60.0)),
                                                                     1.5 * R_p, 0.12], 2),
             "support": "propeller box (blade-tip triangle; depth 0.12 m estimate incl. the blade twist)"}],
        "requirements": "R-29 (outer panel <= 3.4 m) and R-30 (centre section <= 2.0 m) of the spec; van/trailer "
                        "transport",
        "text_tr": "Taşıma dört parçalıdır: kanat eldiveni, dikeyler, kök parçaları ve ventral ile birlikte orta gövde "
                   f"({_c(W_c)} m genişlik, {_c(L_c)} m boy, takım içeride {_c(H_c)} m yükseklik), iki dış kanat "
                   f"paneli ({_c(L_o - tongue)} m açıklık, dil dahil {_c(L_o)} m), iki stabilatör ve pervane."}


FIELD = [
    {"n": 1, "text_tr": "Orta gövdeyi düz zemine takozlarla yerleştir, kanat sehpalarını kur.", "people": 2,
     "minutes": 3},
    {"n": 2, "text_tr": "Birleşim erişim kapaklarını ve arka pim deliği kapağını aç; dil ve kulak kapaklarını çıkar, "
                        "çatal ağzını ve contayı denetle.", "people": 1, "minutes": 3},
    {"n": 3, "text_tr": "Dış paneli ana kiriş ekseni boyunca içeri sür (bir kişi uçta), dil, arka kiriş kulağı ve "
                        "konnektör aynı anda girer; panel oturunca iki Ø16 pimi önden, Ø8 arka pimi alttan tak, "
                        "emniyet plakalarını 2 x M4 ile vidala.", "people": 2, "minutes": 7},
    {"n": 4, "text_tr": "Konnektörün kilitlendiğini denetle, erişim kapaklarını kapat. Diğer kanat için tekrarla.",
     "people": 1, "minutes": 5},
    {"n": 5, "text_tr": "Stabilatörleri mil uçlarına geçir, çapraz cıvatayı tork ve emniyetle.", "people": 1,
     "minutes": 4},
    {"n": 6, "text_tr": "Pervaneyi tak, cıvataları torkla ve emniyetle; spinner'ı tak.", "people": 1, "minutes": 5},
    {"n": 7, "text_tr": "Uçuş öncesi denetim: kumanda yön testi, pim emniyetleri, paraşüt emniyet piminin çıkarılması "
                        "(son adım), yakıt drenajı.", "people": 2, "minutes": 8}]

MAINTENANCE = [
    {"item": "spark plugs, cylinders, CHT/EGT sensors, exhaust, baffles",
     "access": ["P-COWL-UP", "P-COWL-UPS", "P-COWL-LO"], "fasteners": "Camloc", "primary_structure_removed": False,
     "notes": "fix round 1 (VPK-05): the cowl is split around the fin, stub and ventral roots (fixed root strips), so "
              "no fixed surface is removed"},
    {"item": "engine (removal)", "access": ["P-COWL-UP", "P-COWL-UPS", "P-COWL-LO"],
     "fasteners": "Camloc + 4 x M8 mount bolts",
     "primary_structure_removed": False,
     "notes": "propeller and spinner off; the engine slides 0.25 m aft along the thrust axis off the mount face and is "
              "then lifted 0.35 m clear of the fin and stub roots (assembly_paths engine_removal + engine_lift, swept "
              "in layout_check C05; fix round 2, PK2-11)"},
    {"item": "fuel pump/filter + drain, inner-door actuators, tail connector, engine harness junction", "access": ["P-AFTHATCH"], "fasteners": "Camloc", "primary_structure_removed": False},
    {"item": "engine ECU, mission computer / recorder", "access": ["P-MBHATCH"], "fasteners": "Camloc",
     "primary_structure_removed": False,
     "notes": "both on the removable equipment tray TR-MISSION in the floor cut-out, lowered out through the hatch "
              "(fix round 1, VPK-03)"},
    {"item": "stabilator actuators (DA 30), pushrods and boots, fuel shut-off valve", "access": ["P-STABACT"],
     "fasteners": "Camloc", "primary_structure_removed": False},
    {"item": "stabilator spindle bearings", "access": ["P-COWL-UPS", "stabilator removal"], "fasteners": "Camloc + "
     "cross-bolt", "primary_structure_removed": False},
    {"item": "fuel cells (inspection, replacement)", "access": ["P-FUEL1", "P-FUEL2", "P-FUEL3"],
     "fasteners": "M4 nutplate screws + gasket", "primary_structure_removed": False},
    {"item": "PDU, contactor/fuses, DC-DC, nose harness", "access": ["P-AVHATCH"], "fasteners": "Camloc",
     "primary_structure_removed": False},
    {"item": "Li-ion buffer battery, independent FTS unit", "access": ["P-FWDHATCH"], "fasteners": "Camloc",
     "primary_structure_removed": False},
    {"item": "generator power electronics, brake actuator + master cylinder", "access": ["P-SIDEBAY-L"],
     "fasteners": "Camloc", "primary_structure_removed": False},
    {"item": "autopilot, datalinks, transponder", "access": ["P-SIDEBAY-R"], "fasteners": "Camloc",
     "primary_structure_removed": False},
    {"item": "nose antennas, Remote ID beacon, pitot lines", "access": ["P-NOSECONE"], "fasteners": "8 x M4",
     "primary_structure_removed": False},
    {"item": "EO/IR turret (HD59 / E180)", "access": ["P-TURRETRING"], "fasteners": "14 x M4 + 4 turret mount bolts",
     "primary_structure_removed": False, "notes": "elevator extended, ring off: 0.212 m square bay opening"},
    {"item": "turret elevator, sliding doors", "access": ["P-TURRETRING", "P-TDOORACC"], "fasteners": "M4 / Camloc",
     "primary_structure_removed": False,
     "notes": "elevator motor, ball screw, rails and limit switches through the bay opening (ring off, turret "
              "removed); door actuators through P-TDOORACC (L/R, aft of FS1330), racks and rails from inside the bay "
              "with the ring removed and the doors open"},
    {"item": "parachute (12-month repack, UAVOS service life)", "access": ["P-PARAHATCH"], "fasteners": "latch",
     "primary_structure_removed": False,
     "notes": "tethered lift-off hatch 376 x 376 mm, clear opening 313 x 312 mm for the 300 x 300 mm container"},
    {"item": "research payload", "access": ["P-PAYHATCH"], "fasteners": "Camloc", "primary_structure_removed": False},
    {"item": "main gear legs, EMAs, locks, brakes, centre harness channel", "access": ["main wells (gear down, inner "
                                                                                       "doors in maintenance mode)"],
     "fasteners": "-", "primary_structure_removed": False,
     "notes": "legs come out on their flanged stub axles, pushed inward out of the lug bushings from inside the yoke "
              "(assembly_paths main_stub_axle_*; fix round 2, PK2-04)"},
    {"item": "main-gear trunnion fittings F-TRUNNION, up-lock fittings F-UPLOCK (fasteners)",
     "access": ["main wells (gear down, trunnion door open)"],
     "fasteners": "5 x M6 12.9 per fitting into bonded inserts of the gear beam (heads in the well) + 4 x M6 12.9 into "
                  "sealed dome nutplates on the fuel side of the well roof; up-lock 4 x M5 into blind potted inserts",
     "primary_structure_removed": False,
     "notes": "fix round 2 (PK2-05): every bolt head is reached from the well; the dome nutplates on the fuel side of "
              "the well roof are installed and leak-tested in the chassis subassembly - replacing one needs the aft "
              "fuel cell removed through its fuel-bay panel (cell removal, then leak test); the up-lock inserts do "
              "not pierce the fuel-side facesheet"},
    {"item": "nose gear, steering", "access": ["keel slot (gear down)", "P-SIDEBAY-L", "P-SIDEBAY-R"],
     "fasteners": "Camloc", "primary_structure_removed": False,
     "notes": "nose leg on two flanged stub axles pushed outward into the pivot-block bushings from inside the yoke "
              "(assembly_paths nose_stub_axle_*); steering DA 26 on the aft face of the leg (gear down); nose-door "
              "DA 22s EQ-NDOORACT-L / -R through the side-bay panels"},
    {"item": "wing joint pins, keepers, wing connector", "access": ["P-JOINTACCESS", "P-REARACCESS"],
     "fasteners": "Camloc / bayonet cap", "primary_structure_removed": False},
    {"item": "aileron / flap actuators", "access": ["outer-panel lower servo hatches (wing module)"],
     "fasteners": "M4 inserts", "primary_structure_removed": False},
    {"item": "rudder actuators, fin-tip antennas, tail light", "access": ["fin servo hatches (inboard face)",
                                                                          "fin-tip caps"],
     "fasteners": "M4", "primary_structure_removed": False},
    {"item": "GNSS antennas", "access": ["P-AVHATCH", "P-GNSS2"], "fasteners": "Camloc / M4",
     "primary_structure_removed": False}]


_ITEM_TR = {
    "spark plugs, cylinders, CHT/EGT sensors, exhaust, baffles": "bujiler, silindirler, CHT/EGT algılayıcıları, egzoz, "
                                                                 "hava yönlendiricileri",
    "engine (removal)": "motor (sökme)",
    "fuel pump/filter + drain, inner-door actuators, tail connector, engine harness junction":
        "yakıt pompası/filtresi + boşaltma, iç kapak eyleyicileri, kuyruk konnektörü, motor kablo bağlantısı",
    "engine ECU, mission computer / recorder": "motor ECU, görev bilgisayarı / kayıt cihazı",
    "stabilator actuators (DA 30), pushrods and boots, fuel shut-off valve":
        "stabilatör eyleyicileri (DA 30), itme çubukları ve körükleri, yakıt kesme vanası",
    "stabilator spindle bearings": "stabilatör mili yatakları",
    "fuel cells (inspection, replacement)": "yakıt hücreleri (muayene, değiştirme)",
    "PDU, contactor/fuses, DC-DC, nose harness": "PDU, kontaktör/sigortalar, DC-DC, burun kablo demeti",
    "Li-ion buffer battery, independent FTS unit": "Li-ion tampon batarya, bağımsız FTS birimi",
    "generator power electronics, brake actuator + master cylinder":
        "jeneratör güç elektroniği, fren eyleyicisi + ana silindir",
    "autopilot, datalinks, transponder": "otopilot, veri bağları, transponder",
    "nose antennas, Remote ID beacon, pitot lines": "burun antenleri, uzaktan kimlik vericisi, pitot hatları",
    "EO/IR turret (HD59 / E180)": "EO/IR taret (HD59 / E180)",
    "turret elevator, sliding doors": "taret asansörü, kayar kapaklar",
    "parachute (12-month repack, UAVOS service life)": "paraşüt (12 ayda bir katlama, UAVOS servis ömrü)",
    "research payload": "araştırma faydalı yükü",
    "main gear legs, EMAs, locks, brakes, centre harness channel":
        "ana takım bacakları, EMA'lar, kilitler, frenler, orta kablo kanalı",
    "nose gear, steering": "burun takımı, yönlendirme",
    "main-gear trunnion fittings F-TRUNNION, up-lock fittings F-UPLOCK (fasteners)":
        "ana takım mafsal bağlantıları F-TRUNNION, yukarı kilit bağlantıları F-UPLOCK (bağlantı elemanları)",
    "wing joint pins, keepers, wing connector": "kanat birleşim pimleri, emniyetleri, kanat konnektörü",
    "aileron / flap actuators": "kanatçık / flap eyleyicileri",
    "rudder actuators, fin-tip antennas, tail light": "dümen eyleyicileri, dikey uç antenleri, kuyruk ışığı",
    "GNSS antennas": "GNSS antenleri",
}
_ACCESS_TR = {
    "stabilator removal": "stabilatör sökülerek",
    "main wells (gear down, inner doors in maintenance mode)": "ana takım kuyuları (takım açık, iç kapaklar bakım "
                                                               "modunda)",
    "keel slot (gear down)": "omurga yuvası (takım açık)",
    "outer-panel lower servo hatches (wing module)": "dış panel alt servo kapakları (kanat modülü)",
    "fin servo hatches (inboard face)": "dikey servo kapakları (iç yüz)",
    "fin-tip caps": "dikey uç kapakları",
}
_FAST_TR = {"Camloc + 4 x M8 mount bolts": "Camloc + 4 x M8 bağlantı cıvatası", "Camloc + cross-bolt": "Camloc + "
            "çapraz cıvata", "M4 nutplate screws + gasket": "M4 somun plakalı vida + conta",
            "14 x M4 + 4 turret mount bolts": "14 x M4 + 4 taret bağlantı cıvatası", "latch": "mandal",
            "Camloc / bayonet cap": "Camloc / süngü kapak",
            "M4 inserts": "M4 dişli burç"}
for _m in MAINTENANCE:
    _m["item_tr"] = _ITEM_TR[_m["item"]]
    _m["access_tr"] = [_ACCESS_TR.get(a, a) for a in _m["access"]]
    _m["fasteners_tr"] = _FAST_TR.get(_m["fasteners"], _m["fasteners"])
