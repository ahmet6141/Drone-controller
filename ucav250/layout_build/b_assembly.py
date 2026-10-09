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
    S.append(step(4, "Gövde yanı, eldiven, birleşim ve LERX burun kaburgaları", "eldiven yapısı (L/R)",
                  "Gövde yanı kaburgası (y = ±0,40 m), eldiven kaburgası (±0,55 m), birleşim kaburgası (±0,70 m) ve üç "
                  "LERX burun kaburgası kutunun kirişlerine yapıştırılır ve cıvatalanır.",
                  ["kaburga konum mastarları", "yapıştırıcı"], ["kaburga istasyonları ±0,3 mm",
                                                                "birleşim kaburgası yüzü düzlemsellik 0,2 mm"]))
    S.append(step(5, "Kanat çatalı ve sürükleme pimi burcu", "kanat birleşim bağlantıları (L/R)",
                  "7075-T651 çatal (YK250-CH-053) ana kirişe yapıştırılır ve 2 x 6 M6 cıvatayla bağlanır. Pim "
                  "delikleri (2 x Ø14 H8) ana kiriş mastarı (usta dil) takılıyken birlikte raybalanır. Arka kirişe "
                  "Ø10 H8 bronz burçlu sürükleme pimi bağlantısı (YK250-CH-054) aynı mastara göre bağlanır.",
                  ["usta dil mastarı", "Ø14 H8 ve Ø10 H8 raybalar", "matkap kılavuzu"],
                  ["iki pim deliği arası 0,2272 m ±0,05 mm (kiriş ekseni boyunca)",
                   "pim deliği ile çatal kenarı >= 2,0 D", "usta dil sıkışmasız kayar, boşluk <= 0,1 mm"]))
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
                  "Kenar çizgisi uzun kirişleri FS0600'den ana kiriş çerçevesine ve arka kiriş çerçevesinden FS3738'e "
                  "uzanır; kutu bölgesinde gövde yanı kaburgasına 2 x 4 M6 ile eklenir.", ["matkap şablonları",
                                                                                          "tork anahtarı"],
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
                   "FS-GEAR ve FS3480 çerçeveleri, sırt uzun kirişleri ve arka omurga kirişi takılır. Dikey ön/arka kiriş "
                   "bağlantıları, sabit kök parçası ön bağlantısı ve üç ventral bağlantısı matkap şablonlarından "
                   "delinerek bağlanır.", ["kuyruk bağlantı şablonları", "tork anahtarı"],
                   ["dikey kök bağlantı noktaları layout.chassis.fittings ±0,3 mm", "ventral bağlantıları aynı hatta"]))
    S.append(step(12, "Yangın perdesi ve motor bağlantı parçaları", "FS3670",
                   "Kompozit sandviç perde takılır; 12 paslanmaz ara parça üzerine 0,5 mm AISI 304 kalkan perçinlenir. "
                   "Motor bağlantısı için 4 bağlantı parçası (7075 destek plakaları, paslanmaz ara borular) perdeden "
                   "geçirilir. Kablo, yakıt ve itme çubuğu geçişlerine yanmaz rondela/körük takılır.",
                   ["perçin tabancası", "tork anahtarı"], ["paslanmaz kalkan >= 0,38 mm (FIRE-001), açık delik yok",
                                                           "motor bağlantı parçası konumları ±0,3 mm"]))
    S.append(step(13, "Motor bölmesi halka çerçevesi ve stabilatör mil iç yatakları", "FS3738",
                   "2024-T3 halka çerçeve yerleştirilir; stabilatör mil iç yatak yuvaları (7075) perçinlenir/cıvatalanır "
                   "ve dış yatak yuvasıyla eş eksenli olacak şekilde mil mastarıyla raybalanır.",
                   ["mil mastarı (iki taraf)", "rayba"], ["mil ekseni = tail.surfaces.stabilator.pivot ±0,2 mm, y "
                                                         "eksenine paralel 0,05°"]))
    S.append(step(14, "Paraşüt, taret asansörü, yakıt bölmesi ve teçhizat bağlantıları",
                   "kayış bağlantıları, asansör ray bağlantıları, yakıt bölmesi astarları, tepsiler",
                   "Paraşüt ön ve arka kayış bağlantıları (FS1810 ve arka kiriş çerçevesi üstü), konteyner bağlama "
                   "braketleri, taret asansör ray ve motor bağlantıları, yakıt bölmesi astarları ve hücre askıları, "
                   "aviyonik/arka teçhizat tepsileri takılır.", ["tork anahtarı"],
                   ["kayış bağlantıları 4 x M6 12.9 torklu", "astarlarda keskin kenar yok"]))
    S.append(step(15, "Şasi muayenesi ve tezgâhtan ayırma", "şasi",
                   "Şasi lazer izleyiciyle ölçülür, yapışmalar tıklama/ultrason testiyle denetlenir, şasi tartılır ve "
                   "elektriksel bağlama hattının sürekliliği ölçülür. Şasi kendi kendini taşır; tezgâh ayrılır, şasi "
                   "kaplama tezgâhına alınır.", ["lazer izleyici", "UT cihazı", "terazi", "mikro-ohmmetre"],
                   ["ölçü raporu ±0,5 mm", "şasi kütlesi <= mass.budget.chassis", "bağlama direnci <= 2,5 mΩ"]))
    S.append(step(16, "Sabit gövde kaplamaları", "yapısal kaplamalar",
                   "Yapısal kaplamalar (burun, orta, merkez, arka) çerçeve ve uzun kiriş flanşlarına M4 somun plakalı "
                   "vidalarla 25-35 mm aralıkla bağlanır; delikler kaplama şablonlarından delinir. Yakıt bölmesi "
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
                   "ve uzaktan kimlik sağ yan bölmeye; motor ECU'su görev bölmesi tabanına takılır.",
                   ["tork anahtarı", "ESD bileklik"],
                   ["bağlayıcı payları serbest (layout.rules.bay_contents)", "batarya havalandırma çıkışı ön bölme kapağında",
                    "güç-açık testi: bara gerilimleri"]))
    S.append(step(21, "Antenler ve hava verisi", "antenler, pito",
                   "GNSS antenleri GFRP pencerelerin altına, veri bağı ve FTS antenleri burun konisine, transponder "
                   "bıçak anteni burun altına (bakır ağ zemin düzlemiyle), uzaktan kimlik sağ GFRP kapağın arkasına "
                   "yerleşir. Burun pitosu ve kanat HASA sondası bağlanır, pito-statik hatlar sızdırmazlık testinden "
                   "geçer.", ["pito-statik test seti", "VSWR ölçer"],
                   ["VSWR <= 2,0", "pito-statik sızıntı yok", "anten önünde karbon yok (RF pencere)"]))
    S.append(step(22, "Paraşüt sistemi", "UAVOS 200 + Y-kayış",
                   "Paraşüt konteyneri bölmeye bağlanır; Y-kayışın ön ayağı FS1810, arka ayağı arka kiriş çerçevesi "
                   "bağlantısına takılır; arka ayak sırt kanalına yerleştirilip yırtılır örtüyle kapatılır. Kapak "
                   "mandalı ve tetik hattı (FTS) bağlanır; emniyet pimleri takılı kalır.", ["kayış uzunluk mastarı"],
                   ["kayış ayak uzunlukları layout.chassis.parachute.bridle", "emniyet pimleri takılı (kırmızı flama)"]))
    S.append(step(23, "Görev bilgisayarı ve yük tepsisi rayları", "görev bölmesi, yük bölmesi",
                   "Görev bilgisayarı/kayıtçı görev bölmesi tepsisine, yük tepsisi rayları ön güvertenin altına takılır.",
                   ["tork anahtarı"], ["ray paralelliği 0,2 mm"]))
    S.append(step(24, "Taret asansörü, kayar kapaklar ve HD59", "taret bölmesi",
                   "Asansör rayları, bilyalı vida ve motor takılır; kayar kapaklar raylarına ve kam kollarına bağlanır. "
                   "Açıklık halkası (HD59) takılır, taret taşıyıcıya bağlanır. Asansör 10 tam çevrim çalıştırılır.",
                   ["asansör test kutusu"],
                   ["kapaklar ilk 20 mm strokta tam açık", "top ile halka arası >= 5 mm", "içeride top kapak iç yüzünün "
                                                                                    ">= 26 mm üstünde"]))
    S.append(step(25, "Motorun takılması", "L 275 EF + SG750",
                   "4130 kafes ve titreşim takozları perde bağlantı parçalarına bağlanır; motor 4 x M8 ile karter arka yüzündeki "
                   "göbeklere takılır. İtki ekseni 5° aşağı eğim mastarla doğrulanır.", ["motor askısı", "tork anahtarı",
                                                                                         "eğim mastarı"],
                   ["M8 civataları torklu + emniyet teli", "itki ekseni eğimi 5° ±0,2°", "motor dinamik zarfı ile "
                                                                                       "yapı arası >= 10 mm"]))
    S.append(step(26, "Egzoz, soğutma, motor yardımcıları", "itki sistemi",
                   "Egzoz/susturucular, soğutma kanalı ve silindir perdeleri, ECU, jeneratör güç elektroniği, yakıt "
                   "pompası/filtresi, gaz servosu ve motor demeti bağlanır; perde geçişleri kapatılır.",
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
                   "Sabit kök parçaları FS3480 bağlantısına ve mil yuvasına takılır; miller iki yatağa yerleştirilir. "
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
                   "Panel ana kiriş ekseni boyunca içeri sürülür; dil çatala, sürükleme pimi burcuna aynı anda girer. "
                   "İki Ø14 ana pim önden takılır, emniyet plakaları vidalanır; kanat konnektörü bağlanır; kanatçık ve "
                   "flap eyleyicileri kontrol edilir.", ["kanat sehpası (2 adet)", "pim itici", "tork anahtarı"],
                   ["pimler tam oturmuş, emniyet plakaları takılı", "birleşim contası sürekli", "konnektör kilitli"]))
    S.append(step(36, "Kabuğun kapatılması", "kaportalar, kapaklar, radom",
                   "Üst ve alt kaporta, aviyonik kapağı, yan bölme kapakları, görev ve yük bölmesi kapakları, arka "
                   "teçhizat kapağı, birleşim erişim kapakları ve burun konisi kapatılır.", ["Camloc anahtarı",
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


TRANSPORT = {
    "units": [
        {"unit": "centre body with LERX/glove, fins, stubs, ventral, gear, engine", "size_m": [4.00, 1.40, 1.26],
         "mass_note": "empty mass minus outer panels, stabilators, propeller", "support": "transport cradle on the FS1810 and "
                      "FS-GEAR lower frame lands (two padded saddles), gear retracted or on the gear with chocks; fins "
                      "protected by tip caps covers"},
        {"unit": "outer wing panel (x2)", "size_m": [2.90, 0.58, 0.10], "support": "padded rack, panel on its leading "
                                                                                  "edge; tongue cap and drag-pin cap"},
        {"unit": "stabilator (x2)", "size_m": [0.80, 0.55, 0.08], "support": "foam box"},
        {"unit": "propeller", "size_m": [0.79, 0.15, 0.10], "support": "propeller box"}],
    "requirements": "R-29 (outer panel <= 3.4 m) and R-30 (centre section <= 2.0 m) of the spec; van/trailer "
                    "transport",
    "text_tr": "Taşıma dört parçalıdır: kanat eldiveni, dikeyler, kök parçaları ve ventral ile birlikte orta gövde "
               "(1,40 m genişlik), iki dış kanat paneli (2,90 m), iki stabilatör ve pervane."}

FIELD = [
    {"n": 1, "text_tr": "Orta gövdeyi düz zemine takozlarla yerleştir, kanat sehpalarını kur.", "people": 2,
     "minutes": 3},
    {"n": 2, "text_tr": "Birleşim erişim kapaklarını aç; dil kapağını ve sürükleme pimi kapağını çıkar, çatal ağzını "
                        "ve contayı denetle.", "people": 1, "minutes": 3},
    {"n": 3, "text_tr": "Dış paneli ana kiriş ekseni boyunca içeri sür (bir kişi uçta), dil ve sürükleme pimi aynı anda "
                        "girer; panel oturunca iki Ø14 pimi önden tak, emniyet plakalarını 2 x M4 ile vidala.",
     "people": 2, "minutes": 6},
    {"n": 4, "text_tr": "Kanat konnektörünü bağla ve kilitle, erişim kapağını Camloc ile kapat. Diğer kanat için "
                        "tekrarla.", "people": 1, "minutes": 6},
    {"n": 5, "text_tr": "Stabilatörleri mil uçlarına geçir, çapraz cıvatayı tork ve emniyetle.", "people": 1,
     "minutes": 4},
    {"n": 6, "text_tr": "Pervaneyi tak, cıvataları torkla ve emniyetle; spinner'ı tak.", "people": 1, "minutes": 5},
    {"n": 7, "text_tr": "Uçuş öncesi denetim: kumanda yön testi, pim emniyetleri, paraşüt emniyet piminin çıkarılması "
                        "(son adım), yakıt drenajı.", "people": 2, "minutes": 8}]

MAINTENANCE = [
    {"item": "spark plugs, cylinders, CHT/EGT sensors, exhaust, baffles", "access": ["P-COWL-UP", "P-COWL-LO"],
     "fasteners": "Camloc", "primary_structure_removed": False},
    {"item": "engine (removal)", "access": ["P-COWL-UP", "P-COWL-LO"], "fasteners": "Camloc + 4 x M8 mount bolts",
     "primary_structure_removed": False, "notes": "propeller and spinner off; engine lifts aft-up off the isolators"},
    {"item": "fuel pump/filter + drain, inner-door actuators, tail connector, engine harness junction", "access": ["P-AFTHATCH"], "fasteners": "Camloc", "primary_structure_removed": False},
    {"item": "engine ECU, mission computer / recorder", "access": ["P-MBHATCH"], "fasteners": "Camloc",
     "primary_structure_removed": False},
    {"item": "stabilator actuators (DA 30), pushrods and boots, fuel shut-off valve", "access": ["P-STABACT"],
     "fasteners": "Camloc", "primary_structure_removed": False},
    {"item": "stabilator spindle bearings", "access": ["P-COWL-UP", "stabilator removal"], "fasteners": "Camloc + "
     "cross-bolt", "primary_structure_removed": False},
    {"item": "fuel cells (inspection, replacement)", "access": ["P-FUEL1", "P-FUEL2", "P-FUEL3"],
     "fasteners": "M4 nutplate screws + gasket", "primary_structure_removed": False},
    {"item": "PDU, contactor/fuses, DC-DC, nose harness", "access": ["P-AVHATCH"], "fasteners": "Camloc",
     "primary_structure_removed": False},
    {"item": "Li-ion buffer battery, independent FTS unit", "access": ["P-FWDHATCH"], "fasteners": "Camloc",
     "primary_structure_removed": False},
    {"item": "generator power electronics, brake actuator + master cylinder", "access": ["P-SIDEBAY-L"],
     "fasteners": "Camloc", "primary_structure_removed": False},
    {"item": "autopilot, datalinks, transponder, Remote ID", "access": ["P-SIDEBAY-R"], "fasteners": "Camloc",
     "primary_structure_removed": False},
    {"item": "nose antennas, pitot lines", "access": ["P-NOSECONE"], "fasteners": "8 x M4",
     "primary_structure_removed": False},
    {"item": "EO/IR turret (HD59 / E180)", "access": ["P-TURRETRING"], "fasteners": "8 x M4 + 4 turret mount bolts",
     "primary_structure_removed": False, "notes": "elevator extended, ring off: 0.212 m square bay opening"},
    {"item": "turret elevator, sliding doors", "access": ["P-TURRETRING", "P-AVHATCH"], "fasteners": "M4 / Camloc",
     "primary_structure_removed": False},
    {"item": "parachute (12-month repack, UAVOS service life)", "access": ["P-PARAHATCH"], "fasteners": "latch",
     "primary_structure_removed": False},
    {"item": "research payload", "access": ["P-PAYHATCH"], "fasteners": "Camloc", "primary_structure_removed": False},
    {"item": "main gear legs, EMAs, locks, brakes, centre harness channel", "access": ["main wells (gear down, inner "
                                                                                       "doors in maintenance mode)"],
     "fasteners": "-", "primary_structure_removed": False},
    {"item": "nose gear, steering", "access": ["keel slot (gear down)", "P-SIDEBAY-L", "P-SIDEBAY-R"],
     "fasteners": "Camloc", "primary_structure_removed": False},
    {"item": "wing joint pins, keepers, wing connector", "access": ["P-JOINTACCESS"], "fasteners": "Camloc",
     "primary_structure_removed": False},
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
    "autopilot, datalinks, transponder, Remote ID": "otopilot, veri bağları, transponder, Remote ID",
    "nose antennas, pitot lines": "burun antenleri, pitot hatları",
    "EO/IR turret (HD59 / E180)": "EO/IR taret (HD59 / E180)",
    "turret elevator, sliding doors": "taret asansörü, kayar kapaklar",
    "parachute (12-month repack, UAVOS service life)": "paraşüt (12 ayda bir katlama, UAVOS servis ömrü)",
    "research payload": "araştırma faydalı yükü",
    "main gear legs, EMAs, locks, brakes, centre harness channel":
        "ana takım bacakları, EMA'lar, kilitler, frenler, orta kablo kanalı",
    "nose gear, steering": "burun takımı, yönlendirme",
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
            "8 x M4 + 4 turret mount bolts": "8 x M4 + 4 taret bağlantı cıvatası", "latch": "mandal",
            "M4 inserts": "M4 dişli burç"}
for _m in MAINTENANCE:
    _m["item_tr"] = _ITEM_TR[_m["item"]]
    _m["access_tr"] = [_ACCESS_TR.get(a, a) for a in _m["access"]]
    _m["fasteners_tr"] = _FAST_TR.get(_m["fasteners"], _m["fasteners"])
