# YK-250 HANÇER — Yerleşim ve yapı konsepti (faz 3, yerleşim evresi ve düzeltme turları 1–2)

**Tarih:** 9 Ekim 2026 · **Durum:** `spec.yaml → layout` ve `spec.yaml → assembly` bölümleri ayrıntılı tasarımın arayüz tanımıdır. Bağımsız doğrulamaların yerleşim bulguları düzeltildi: VPK-01 … VPK-14 (§14) ve PK2-01 … PK2-13 (§15). `python3 -m ucav250.analysis.layout_check --check` 88 kontrolün 88'ini geçiyor; `python3 -m ucav250.analysis.sizing --check` 61 gereksinimin 61'ini karşılanmış ve 1231 türetilmiş değerin hepsini tolerans içinde buluyor; `python3 -m ucav250.analysis.structures --check` bütün emniyet paylarını sıfır ya da pozitif buluyor ([04](04_yapi_hesaplari.md)).

> **Kapsam.** YK-250 HANÇER sivil bir EO/IR gözetleme ve araştırma İHA'sıdır. Gövdede dış yük taşıma bağlantısı,
> askı, yük bırakma düzeneği ya da benzeri hiçbir donanım yoktur ve yerleşim bunlar için yer ayırmaz. Karın altındaki
> faydalı yük bölmesi yalnızca araştırma sensörleri içindir. UCAV görünümü yalnızca biçim dilidir.

Bu belge, boyutlandırma evresinde (doc 02) kapanan HANÇER'in iç yerleşimini ve yapı konseptini açıklar: hangi çerçevenin nerede olduğu, birincil yüklerin hangi elemanlardan geçtiği, kabuğun nasıl bölündüğü ve bağlandığı, mekanizmaların nasıl hareket ettiği, sistemlerin nereye yerleştiği, uçağın nasıl üretilip monte edildiği ve bakımda nelere nereden erişildiği. Bütün sayılar tek kaynaktan gelir: [`spec.yaml`](../spec.yaml) (`layout`, `assembly`). Arayüz değerleri [`layout_build/`](../layout_build) modüllerinde yazılıdır ve `spec.yaml`'a üretilerek yazılır; [`analysis/layout_check.py`](../analysis/layout_check.py) onları denetler ve [`out/layout.md`](../out/layout.md) raporunu üretir. §4'teki istasyon tablosu, §6'daki panel tablosu ve §10'daki bakım matrisi `spec.yaml`'dan üretilmiştir; testler bu tablolardaki x değerlerini spec ile karşılaştırır, böylece tablolar spec'ten geri kalamaz.

```
python3 -m ucav250.layout_build.build --check    # spec.yaml, yerleşim üreticisinin çıktısıyla bayt bayt aynı mı (çıkış 1 = fark)
python3 -m ucav250.layout_build.build --write    # spec.layout / spec.assembly yeniden üretilir
python3 -m ucav250.analysis.sizing --update-spec # layout.mass_placement kütle kalemlerine uygulanır, tasarım yeniden kapanır
python3 -m ucav250.analysis.layout_check --check # 88 yerleşim kontrolü; salt okunur, hiçbir dosya yazmaz (çıkış 1 = hata)
python3 -m ucav250.analysis.layout_check --write # out/layout.md, out/layout.json, docs/fig/yk250_layout_*.png
```

Yerleşimde bir değişiklik yapıldığında sıra şudur: `build --write` → `structures --update-spec` → `sizing --update-spec` → `build --write` (boyutlandırma mekanizma değerlerini 6 anlamlı basamakla sakladığı için son yazan üreticidir) → `build --check` (değişmemeli), `structures --check`, `sizing --check` ve `layout_check --check`. Üretici yalnızca kendi sahip olduğu anahtarları yazar; boyutlandırmanın sahip olduğu `datum`, `rules`, `fuel_cells`, `zones_preliminary`, `firewall_x`, `ground_z` ve `bay_contents_check` olduğu gibi taşınır (`rules` yalnızca `b_systems.RULE_EDITS` üzerinden değişir). Raporlarda çalışma süresi yoktur: aynı spec her zaman bayt bayt aynı çıktıyı verir.

---

## 1. Neden bir arayüz tanımı

Ayrıntılı tasarım dokuz üretici modüle bölünür: şasi, kanat, kuyruk, kabuk, itki, yakıt, iniş takımı, sistemler, faydalı yük (ve en sonda bağlantı elemanları). ARCHITECTURE.md §1.2'ye göre bir modül başka bir modülün geometrisini okuyarak kendi parçasını konumlandıramaz. İki modülün paylaştığı her şey — çerçeve istasyonları, bağlantı noktaları, pim çapları, cıvata desenleri, panel ayırma çizgileri, bölme sınırları, yasak bölgeler, mafsal eksenleri, montaj yolları, kapak dış hatları, yakıt hatları, parça numaraları — burada, `spec.layout` içinde bir kez tanımlanır ve iki taraf da oradan okur. Örnekler:

* Kanat modülü dış panelin kompozit dilini (YK250-WG-151), şasi modülü kompozit çatalı (YK250-CH-053) üretir. İkisi de pimlerin yerini, eksenini ve çapını `layout.chassis.wing_joint.main_spar.pins` alanından okur.
* İniş takımı modülü ana bacağı, şasi modülü mafsal bağlantısını (F-TRUNNION) üretir; mafsal ekseni, burç yerleri ve cıvata deseni `layout.chassis.fittings` içindedir, toplama hareketi `layout.mechanisms.joints[main_gear_R]`, kapakların dış hatları `layout.mechanisms.door_outlines` içindedir.
* Kabuk modülü kapakları, sistemler modülü teçhizatı üretir; `layout_check` her teçhizatın aynı yüzdeki bir kapağın açık geçişinden çıkarılabildiğini denetler.

## 2. Genel yerleşim

![Yan kesit yerleşimi](fig/yk250_layout_side.png)

![Üst görünüş yerleşimi](fig/yk250_layout_top.png)

Burundan kuyruğa bölmeler (x burundan geriye, metre):

| Bölge | x (m) | İçerik |
|---|---|---|
| Burun konisi (radom) | 0–0,30 | GFRP; birincil veri bağı anteni A, FTS anteni, uzaktan kimlik vericisi (FS0300 ön yüzünde), pito-statik sonda kökü |
| Ön teçhizat bölmesi | 0,30–0,60 | üstte 12S2P Li-ion tampon batarya (havalandırmalı, hücre sigortalı kutu), altta bağımsız uçuş sonlandırma (FTS) birimi; transponder bıçak anteni karın altında |
| Burun kutusu | 0,60–1,11 | ortada burun takımı omurga yuvası; üstünde aviyonik güverte (PDU, kontaktör/sigorta, DC-DC); yanlarda iki yan bölme (sol: jeneratör güç elektroniği + fren birimi; sağ: otopilot, veri bağları, transponder) |
| Taret bölmesi | 1,11–1,33 | geri çekilir EO/IR taret (HD59, E180 büyüme zarfı), asansör, iki kayar kapak, açıklık halkası parçası |
| Paraşüt bölmesi | 1,49–1,81 | UAVOS 200 paraşüt kabı, bağlı sırt kapağı, Y-kayışın ön ayağı |
| Görev bölmesi | 1,81–2,21 | görev bilgisayarı / kayıtçı ve motor ECU'su sökülebilir ekipman tepsisinde (TR-MISSION, taban kesiği; karından P-MBHATCH ile); sırtta omurga kanalı (M-SPINE); altında kablo kanalları; LERX başlangıcı |
| Yakıt + orta kanat kutusu | 2,21–3,11 | üç ok açılı yakıt hücresi (ön, eyer, arka); ön hücrenin altında karın faydalı yük bölmesi; kutunun arkasında iki ana takım kuyusu |
| Arka teçhizat bölmesi | 3,11–3,67 | EFI yakıt pompası/filtresi, iç kapak eyleyicileri, kuyruk konnektörü; sırtta soğutma girişi ve S-kanalı; yangın perdesinin ön yüzünde stabilatör eyleyicileri ve yakıt kesme vanası |
| Motor bölmesi | 3,67–4,00 | Limbach L 275 EF + SG750, kaynaklı 4130 motor kafesi, egzoz, stabilatör düğüm bağlantıları ve milleri; bölünmüş kaportalar ve itici pervane |

Yerleşimin iki sert kuralı vardır: Li-ion tampon batarya her yakıt hücresinden en az 1 m uzaktadır (1,64 m) ve yakıt hücreleri yangın perdesinin ön yüzünden en az 13 mm uzaktadır (CS-LUAS.967(c); 0,61 m).

**Kütle yerleşimi.** Boyutlandırma evresinde 14 kütle kaleminin (motor grubu, soğutma, stabilatör ve burun eyleyicileri, kablo demeti, FTS/ışıklar, omurga ve uzun kirişler, çerçeveler, motor kafesi, paraşüt bağlantıları, tabanlar/tepsiler, kapak çerçeveleri, kuyruk kök bağlantıları) konumu varsayımdı. Yerleşim evresinde bu kalemlerin konumu, yerleştirilen nesnelerin ağırlık merkezinden hesaplanır (`layout.mass_placement`; her kalemin bileşenleri ve kütle payları yazılıdır) ve `sizing.mass_items` bunu uygular. Örneğin motor grubu kalemi 3,83 m'den 3,59 m'ye geldi: ECU görev bölmesinde, jeneratör güç elektroniği sol yan bölmede ve yakıt pompası arka bölmededir. Düzeltme turunda paraşüt bağlantıları kalemi sırt omurga kanalı (0,65), iki kayış bağlantısı (0,15 / 0,15) ve kap braketleri (0,05) olarak yerleştirildi. Spec kalemleriyle yerleşim kalemleri arasındaki boş uçak AM farkı 1 mm'nin altındadır (C06). Tasarım her değişiklikten sonra yeniden kapanır: kanat kökü x_c4 yerleşim evresinde 2,511 m, yapı evresinde 2,503 m, düzeltme turu 1 sonrasında 2,507 m'dir ve en arka yüklemede statik marj %10,1'dir (bkz. §12).

## 3. Parça numaralandırma ve kök parça

Kimlik biçimi `YK250-<KOD>-NNN[-L|-R]`'dir. KOD grup kodudur (CH şasi, SH kabuk, WG kanat, TL kuyruk, FC uçuş kumandaları, PR itki, FU yakıt, LG iniş takımı, SY sistemler, PL faydalı yük, HW bağlantı elemanları). NNN, parçayı üreten **modülün** numara aralığındandır; aralıklar modüller arasında çakışmaz, bu yüzden bir numara grup kodundan bağımsız olarak tekildir. `-R` +y'de modellenen sancak parçasıdır, `-L` onun aynalanmış iskele kopyasıdır; orta çizgi parçalarında sonek yoktur. Her yerleşim nesnesinin parça kimliği tekildir (taret asansörünün iki rayı RAIL-FR / RAIL-AL artık ayrı kimliklerdedir: YK250-PL-828 / 829; VPK-09).

| Modül | Aralık | Alt aralıklar |
|---|---|---|
| şasi | 1–149 | 1 orta kanat kutusu (kök parça); 2–19 çerçeveler; 20–49 uzun kirişler, omurga, güverteler, duvarlar, sırt kanalı; 50–69 eldiven kaburgaları ve birleşim bağlantıları; 70–84 takım bağlantıları; 85–94 motor bağlantısı ve yangın perdesi (köşe bağlantısı); 95–109 kuyruk bağlantıları ve stabilatör düğümleri; 110–149 taret, paraşüt, yakıt ve teçhizat destekleri |
| kanat | 150–249 | dış panel yapısı 150–199 (kompozit dil, arka kulak); kanatçık, flap, menteşe, eyleyici, bağlantı (FC) 200–249 |
| kuyruk | 250–349 | dikey, kök parçaları, ventral, stabilatörler 250–299; dümen, stabilatör tahriki (FC) 300–349 |
| kabuk | 350–499 | gövde kaplamaları ve kapaklar 350–419; LERX/eldiven ve birleşim kapakları 420–449; kaporta ve filetolar 450–499 |
| itki | 500–569 | motor, sönümleyiciler, egzoz; pervane, spinner, soğutma |
| yakıt | 570–619 | hücreler, hatlar, vanalar, havalandırma, dolum, algılayıcılar |
| iniş takımı | 620–719 | bacaklar, tekerler, frenler, eyleyiciler; kapaklar ve kilitler |
| sistemler | 720–819 | aviyonik, güç, demet; antenler, hava verisi, ışıklar, kurtarma |
| faydalı yük | 820–869 | taret, asansör, bölme kapakları; görev bilgisayarı, tepsi |
| bağlantı elemanları | 870–999 | hardware.py'nin ürettiği `YK250-HW-<bağlantı kimliği>` parçaları ve tek başına donanım |

**Kök parça YK250-CH-001**, orta kanat kutusudur (M-CTBOX). Montaj tezgâhında ilk konan parça budur; bütün çerçeveler, uzun kirişler ve bağlantılar ona göre konumlanır.

## 4. İstasyonlar (çerçeveler ve perdeler)

On üç istasyon vardır. İkisi (FS-MS ve FS-RS) kanat kirişleriyle aynı ok açısında çevron biçimlidir; çerçeve düzlemi x + |y|·tan(ok açısı) çizgisini izler. Bütün kompozit çerçeveler `layups.rib_panel` sandviçidir (iki kat PW CFRP / 6 mm ROHACELL 51 WF / iki kat; 6,8 mm); bağlantı hatlarında ve flanşlarda çekirdek 1:3 eğimle 1,6 mm'lik dolu laminata iner. Motor bölmesindeki FS3738 sıcak bölgededir ve metaldir; düzeltme turu 1'den beri yalnızca düğüm bağlantılarının altında biten bir alt U halkadır (VPK-01); düzeltme turu 2'den beri alt parçası, kuyruk tamponu çarpmasında arka omurganın tepkisini taşıyan talaşlı bir 7075 I kesittir (YK250-CH-015, VS2-03). Tablo `spec.yaml`'dan üretilmiştir (x = çerçevenin orta düzlemi, ok açılı çerçevelerde orta hatta):

| İstasyon | x (m) | Tür | Parça | Malzeme / katman | t (mm) | Görev | Geçiş kesikleri |
|---|---|---|---|---|---|---|---|
| FS0300 | 0,300 | perde | YK250-CH-002 | cfrp_pw_mtm45_as4 / rib_panel | 6,8 | burun konisi perdesi: radom flanşı (8 × M4 somun plakası), pito-statik ve FTS anten geçişleri; ön teçhizat bölmesini kapatır; uzaktan kimlik vericisi ön yüzünde (radom içinde) | C-PITOT, C-COAX-NC |
| FS0600 | 0,600 | perde | YK250-CH-003 | cfrp_pw_mtm45_as4 / rib_panel | 6,8 | burun takımı ön perdesi; omurga duvarlarının ve kenar uzun kirişlerinin ön ucu, burun takımı kuyusunun ön duvarı | C-HARN-FWD |
| FS1110 | 1,110 | perde | YK250-CH-004 | cfrp_pw_mtm45_as4 / rib_panel | 6,8 | taret bölmesi ön perdesi; burun kutusunun arka ucu; asansör rayı ön bağlantısı; taret halka parçasının ön oturma yüzeyi | C-HARN-1110 |
| FS1330 | 1,330 | perde | YK250-CH-005 | cfrp_pw_mtm45_as4 / rib_panel | 6,8 | taret bölmesi arka perdesi; asansör rayı arka bağlantısı; taret kapağı eyleyicileri arka yüzünde | C-HARN-1330, C-TDOOR-SHAFT |
| FS1490 | 1,490 | perde | YK250-CH-006 | cfrp_pw_mtm45_as4 / rib_panel | 6,8 | paraşüt bölmesi ön perdesi; paraşüt kapağının ön oturma yüzeyi (dışa dönük flanş), paraşüt tabanının ön ucu | C-HARN-1490 |
| FS1810 | 1,810 | bağlantı çerçevesi | YK250-CH-007 | cfrp_pw_mtm45_as4 / rib_panel | 6,8 | paraşüt bölmesi arka perdesi + LERX burun çerçevesi; sırt omurga kanalının (M-SPINE) ve ön kayış bağlantısının başlangıcı, görev bölmesinin ön duvarı | C-HARN-1810, C-BRIDLE-F |
| FS-FUEL | 2,1895 | perde | YK250-CH-008 | cfrp_pw_mtm45_as4 / rib_panel | 6,8 | ön yakıt bölmesi perdesi (buhar sızdırmaz; kanat demeti çentiği yok, VPK-13); omurga kirişlerinin ve yük bölmesi tavanının ön ucu | C-FUEL-FUEL, C-HARN-FUEL, C-SPINE-FUEL |
| FS-MS | 2,487 (ok açısı 8,0°) | bağlantı / kiriş çerçevesi | YK250-CH-009 | cfrp_pw_mtm45_as4 / rib_panel | 6,8 | ana kiriş çerçevesi: gövde içindeki kutunun ana kiriş gövdesi; eyer bölmesinin ön, ön bölmenin arka duvarı; kutunun altında yalnızca dış dikmeler (yük bölmesi açık); yakıt kapaklarının oturma flanşı | C-PAYLOAD-MS, C-FUEL-MS, C-SPINE-MS, C-HARN-MS |
| FS-RS | 2,800 (ok açısı 4,8°) | bağlantı / kiriş çerçevesi | YK250-CH-010 | cfrp_pw_mtm45_as4 / rib_panel | 6,8 | arka kiriş çerçevesi; eyer bölmesinin arka, arka bölmenin ön duvarı; arka kayış bağlantısı (sırt kanalının arka ucu); ana takım kirişlerinin ve kuyu tavanının ön ucu | C-PAYLOAD-RS, C-FUEL-RS, C-HARN-RS, C-BRIDLE-A |
| FS-GEAR | 3,0954 | perde | YK250-CH-011 | cfrp_pw_mtm45_as4 / rib_panel | 6,8 | arka ana takım çerçevesi ve arka yakıt perdesi (buhar sızdırmaz); iç kapak eyleyicileri arka yüzünde | C-FUEL-FEED, C-FUEL-RET, C-FUEL-VENT, C-HARN-CL, C-DOORLINK |
| FS3480 | 3,480 | halka | YK250-CH-012 | cfrp_pw_mtm45_as4 / rib_panel | 6,8 | kuyruk halka çerçevesi: dikey ön kiriş ve kök parçası ön bağlantıları; ortası açık (soğutma kanalı, teçhizat) | – (açık orta) |
| FS3670 | 3,670 | perde / yangın perdesi | YK250-CH-013 | CFRP sandviç (rib_panel) + hava boşluğu + 0,4 mm AISI 304 | 14,8 | yangın perdesi (CS-VLA 1191): iki köşe bağlantısı (motor üst ayakları, dikey arka kirişleri, sırt uzun kirişi ekleri), iki alt motor ayağı, arka yüzde stabilatör düğüm bağlantıları (kenar uzun kirişi ekleri), ventral ön bağlantısı; ön yüzde stabilatör eyleyicileri; hiçbir karbon eleman geçmez (VPK-06) | C-DUCT, C-FW-FUEL, C-FW-HARN, C-FW-PUSHROD |
| FS3738 | 3,7378 | alt U halka | YK250-CH-014 | al_2024_t3_sheet; alt parça al_7075_t651_plate (YK250-CH-015) | 2,0 | motor bölmesi alt U halkası (metal, sıcak bölge; y ±0,15 arası alt parçası talaşlı 7075 I 60 × 20 × 1,6 / 2,0 mm, arka omurganın tampon tepkisini taşır): ventral arka bağlantısı, arka omurga kirişi, alt kaporta ara oturma yüzeyi; üst uçları düğüm bağlantılarının dış yanak ayaklarına perçinli; mil yatağı taşımaz (VPK-01) | – (açık orta) |

![Çerçeve kesitleri](fig/yk250_layout_sections.png)

Her geçiş kesiği bir amaç taşır (kablo demeti, koaksiyel, yakıt hattı, itme çubuğu, kayış, sırt kanalı, soğutma kanalı, kapak tahrik mili) ve çerçeve gövdesinin içinde kalır. Yakıt bölmelerinden kablo geçmez: ana demet yakıt hücrelerinin altından, sızdırmaz rondelalarla geçer; kanat demeti H-WING artık FS-FUEL'den geçmez, ön yakıt güvertesinin altından orta kanat kutusunun alt kapağındaki sızdırmaz rondelaya girer (VPK-13). `layout_check`, her demetin, yakıt hattının, itme çubuğunun, kayışın ve soğutma kanalının bir çerçeve düzlemini kestiği her noktanın tanımlı bir kesiğin içinde olduğunu (halka çerçevelerde açık ortada olduğunu) ve hiçbir çerçevenin yakıt, taret, takım kuyusu, yük, paraşüt ya da teçhizat hacmini kesmediğini denetler.

**Yangın perdesi yığını** (önden arkaya): CFRP sandviç perde 6,8 mm / 12 paslanmaz ayak üzerinde 7,6 mm hava boşluğu / 0,4 mm AISI 304 paslanmaz sac (test gerektirmeyen yangın dayanımı için ≥ 0,38 mm; CS-VLA 1191). `layout.firewall_x` = 3,670 m paslanmaz sacın arka yüzüdür; yığın onun önündedir. Perdeden yalnızca yangına dayanıklı bağlantılar, rondelalar ve körükler geçer; **hiçbir karbon eleman geçmez**: kenar ve sırt uzun kirişleri perdenin ön yüzünde biter ve metal eklerle (düğüm bağlantısının ön dili, köşe bağlantısının ön ek levhası) devam eder (VPK-06). Alt motor ayaklarının çevresinde perde sandviçi ROHACELL 71 WF çekirdek parçası ve 22 katlı, r 82 mm dolu bantla güçlendirilmiştir (yapı, [04](04_yapi_hesaplari.md) §3.5); motor demeti kesiği C-FW-HARN bu bantların kenar çemberini kesmemesi için ayakların altındadır. Paslanmaz katmanın kütlesi (sac, kenar köşebendi, ayaklar, perçinler) `layout.chassis.engine_mount.firewall_stackup.mass` içinde aşağıdan yukarı hesaplanır ve motor soğutma / yangın koruma kalemine girer (0,705 kg).

## 5. Şasi + kabuk konsepti ve yük yolları

![Yapısal kavram ve yük yolları](fig/yk250_layout_structure.png)

**Şasi** (çerçeveler, kenar uzun kirişleri, omurga elemanları, güverteler, orta kanat kutusu ve bütün bağlantı parçaları) birincil yüklerin tamamını taşır ve tezgâhta kendi başına kapanan bir yapıdır. **Kabuk** (gövde kaplamaları, kapaklar, filetolar) şasiye bağlanır; yapısal kaplamalar yarı-monokok yapının kesme duvarlarıdır ama her biri değiştirilebilir (somun plakalı M4 vidalar). Sökülebilir kapaklar hiçbir birincil yük taşımaz: kesikler çerçeve ve uzun kiriş flanşlarına oturan çerçeveli açıklıklardır. Tek istisna LERX/eldiven kaplamalarıdır: eldiven kutusunun birincil kaplamalarıdır, şasiyle birlikte yapıştırılarak kapanır ve hiç sökülmez (birleşim erişim kesiği çerçevelidir ve kaplama yükünü çevresinden dolaştırır).

### 5.1 Uzun kirişler, omurga ve güverteler

| Eleman | Parça | Görev | Yük yolu |
|---|---|---|---|
| M-CHINE (L/R) | YK250-CH-020 | kenar çizgisi uzun kirişi, J kesit, UD başlık + PW gövde | kaplamalar (kesme) → uzun kiriş (eksenel) → çerçeveler / gövde yanı kaburgası; ön parça FS0600 → ana kiriş çerçevesi, arka parça arka kiriş çerçevesi → yangın perdesinin ön yüzü; arka ucu 2 × M5 Ti ile düğüm bağlantısının ön diline eklenir (karbon perdeden geçmez) |
| M-KEELWALL (L/R) | YK250-CH-021 | burun takımı omurga duvarları | burun takımı yükleri → mafsal burçları → duvarlar → FS0600 / FS1110 + aviyonik güverte → kenar kirişleri |
| M-DECK-NOSE | YK250-CH-022 | aviyonik güverte | teçhizat ataleti → güverte → duvarlar ve kenar kirişleri |
| M-TURRETWALL (L/R), M-TURRETROOF | YK250-CH-023/024 | taret bölmesi duvarları ve tavanı | karın kesiği kenar yükleri ve taret ataleti → duvarlar → FS1110 / FS1330; asansör tepkisi → tavan |
| M-PARAWALL (L/R), M-PARAFLOOR | YK250-CH-025/026 | paraşüt bölmesi duvarları ve tabanı; duvarların üst flanşı dışa dönüktür (50 mm: kapak ve çevre kaplaması oturma yüzeyleri) | kap ataleti → duvarlar/taban → FS1490 / FS1810 |
| M-MIDFLOOR | YK250-CH-027 | görev bölmesi tabanı; ortası (y ±0,125) sökülebilir TR-MISSION tepsisine açılan çerçeveli kesik, kesik kenarlarında iki yapıştırılmış şapka takviye | görev teçhizatı → tepsi / taban şeritleri → çerçeveler + kenar kirişleri; dış şeritler ön gövdenin alt başlığı |
| M-KEEL (L/R) | YK250-CH-028 | omurga kirişleri (yük bölmesi duvarları) | yük bölmesi kesiği çevresinde karın eğilmesi → FS-FUEL / kiriş çerçeveleri → kanat kutusu |
| M-FWDDECK | YK250-CH-029 | ön yakıt bölmesi tabanı / yük bölmesi tavanı; yük tepsisi rayları alt yüzde güverte takviyesidir; yakıt ikmal hattının sızdırmaz geçişi | yakıt ve yük ataleti → güverte → omurga kirişleri, çerçeveler |
| M-WELLROOF | YK250-CH-030 | ana takım kuyusu tavanı, arka yakıt hücresi tabanı (`layups.fuel_floor_wellroof`, 71 WF çekirdek) | takım ve yakıt yükleri → güverte → takım kirişleri, kuyu omurga gövdeleri, çerçeveler |
| M-WELLKEEL (L/R) | YK250-CH-036 | kuyu omurga gövdeleri: iki 1,0 mm PW gövde, orta hat kablo kanalının duvarları | kuyu tavanı → gövdeler → kuyu ön duvarı + FS-GEAR; iç kapak menteşe dirsekleri alt flanşlarda |
| M-GEARBEAM (L/R) | YK250-CH-031 | ana takım kirişi (kuyunun dış duvarı) | mafsal → takım kirişi → FS-RS + FS-GEAR + kuyu tavanı → kutu / kenar kirişleri |
| M-DORSAL (L/R) | YK250-CH-032 | sırt uzun kirişleri (FS-GEAR → yangın perdesinin ön yüzü) | dikey ve kök parçası yükleri → FS3480 / köşe bağlantısı → sırt kirişleri → FS-GEAR; arka ucu 3 × M5 Ti ile köşe bağlantısının ön ek levhasına eklenir |
| M-SPINE | YK250-CH-037 | sırt omurga kanalı (FS1810 → arka kiriş çerçevesi): PW U 44 × 24 mm, 60 mm flanşlar; yakıt bölmelerinden sızdırmaz oluk olarak geçer | kayış ayakları → F-RISER-FWD / F-RISER-AFT → kanal (eksenel) → vidalar → P-MB-UPPER (kesme) → kenar kirişleri; düşey bileşenler → FS1810 / arka kiriş çerçevesi |
| M-AFTKEEL | YK250-CH-033 | arka omurga kirişi (7075-T651 talaşlı U 24 × 40 × 2,5 mm, motor bölmesi; iki alt oturma flanşı 26 × 1,6 mm) | ventral ve tampon darbesi → kiriş → yangın perdesi + FS3738; alt kaporta yarımlarının ortak oturma yüzeyi |
| M-VENTRALKEEL | YK250-CH-039 | ventral omurga şeridi (FS3480 → yangın perdesi), CFRP şapka | ventral ön kökü (yan yükler) → şerit → FS3480 / perde; P-STABACT yarımlarının ortak oturma yüzeyi; perdeden metal ek levhayla arka omurgaya eklenir |
| M-PAYWALL-AFT, M-WELLWALL-FWD | YK250-CH-034/035 | yük bölmesi arka duvarı, kuyu ön duvarı (ikincil) | bölmeyi kapatır; aralarında enine kablo kanalı |

### 5.2 Orta kanat kutusu ve dış panel birleşimi

Orta kanat kutusu (M-CTBOX, YK250-CH-001) y = −0,70 … +0,70 m arasında tek parçadır. Ana ve arka kirişler referans yamuğun %25 ve %72 veter çizgilerindedir; gövde ortasında iki yana 8,0° (ana) ve 4,8° (arka) açılı bir çevron oluştururlar ve y = 0'da 7075-T651 bir kırılma bağlantısı vardır. UD kiriş başlıkları çataldan çatala kesintisizdir ve hiçbir bağlantı elemanı başlıklardan geçmez; gövdeler ±45° PW, gövde içindeki kutu kapakları `layups.ct_box_cover` sandviçidir (0,4/6/0,4 mm) ve y = 0'daki orta hat kaburgası kırık bağlantısını taşır. Simetrik eğilme kutunun içinde dengelenir; asimetrik yükler kiriş çerçeveleri üzerinden kenar uzun kirişlerine geçer. Eldiven yapısı kutuya gövde yanı kaburgası (y 0,40), eldiven kaburgası (y 0,55) ve birleşim kaburgasıyla (y 0,70) bağlanır; LERX burun kaburgaları kaldırıldı, böylece çatalın önündeki pim, pim çekici ve rayba koridorları boştur (VPK-02).

**Dış panel birleşimi y = 0,70 m** (her yan; planör uygulaması, pimli kompozit dil-çatal):

* **Ana kiriş: kompozit dil-çatal.** Dış panelin CFRP dili (YK250-WG-151; 30 × 61 mm, 0,292 m kavrama; UD flanşlar 30 × 10 mm, ±45 PW gövde 5 mm, pim deliklerinde [±45/0/90] bloklar ve yapıştırılmış 4130 burçlar 16 H8 × dış çap 22 × 30 mm) eldivenin ana kiriş kutusunun oluşturduğu kompozit çatala (YK250-CH-053; iki ±45 gövde 2 mm, pim bölgelerinde 10 mm'ye kalınlaşır, 4130 burçlar 16 H8 × dış çap 22 × 10 mm) girer. Eğilme, kiriş boyunca 184 mm aralıklı iki veter yönlü pimle düşey bir kuvvet çifti olarak aktarılır: P-MAIN1 (y 0,463) ve P-MAIN2 (y 0,645), Ø16 mm Ti-6Al-4V başlıklı pim (baş Ø24), 16 H8/h8 geçme, ön yüzde 2 × M4 emniyet plakası. Başlıktan metale yapıştırma ya da cıvatalı aktarım yoktur.
* **Arka kiriş: kulak ve yuva bağlantısı.** Dış panelin arka kiriş kök bağlantısındaki 7075 kulak (YK250-WG-152; 8 mm, 32 mm genişlik, Ø8 H8 delik, e 16 mm = 2 D) birleşim kaburgası / arka kiriş köşesindeki yuva bağlantısının (YK250-CH-054; iki 4 mm levha, 8,2 mm aralık) içine girer; düşey Ø8 Ti bilyalı kilit pimi (P-REAR) alttan, P-REARACCESS deliğinden takılır. Kulak veter yönü kuvveti ve düzlem içi momentin açıklık yönü kuvvet çiftini (ana pimlerle 0,29 m kol) taşır.
* **Takma yolu.** Dış panel ana kiriş ekseni boyunca içeri sürülür; dil çatala, arka kulak yuvaya ve kör takılan kanat konnektörü yuvasına aynı hareketle girer. Takma yolu, pim ve rayba süpürmeleri `layout.mechanisms.assembly_paths` içinde eksen, strok ve zarfla tanımlıdır ve `layout_check` bunları başka parçalara karşı süpürür.
* **Erişim.** Pimler alt eldivendeki birleşim erişim kapağından (P-JOINTACCESS, YK250-SH-422; Camloc) önden takılır. Kapak gövde yanı kaburgasından birleşim kaburgasına kadar uzanır; her iki pim, emniyetleri, pim çekici ve rayba koridorları açıktır. Arka pim için alt eldivende Ø18 bir delik ve gömülü süngü kapak (P-REARACCESS) vardır.

### 5.3 Bağlantı parçaları

Bütün metal bağlantı parçaları 7075-T651 levhadan CNC ile işlenir (motor bölmesi alt U halkası 2024-T3 sacdır, alt parçası talaşlı 7075; motor kafesi kaynaklı 4130 borudur). Her bağlantının cıvata deseni (nokta, eksen, çap) `layout.chassis.fittings[].bolts` içinde açıkça yazılıdır; `layout_check` kutunun her cıvatayı kenar mesafesi (metal ≥ 2 D, kompozit ≥ 2,5 D) ve aralık (≥ 3 D) kuralıyla taşıdığını denetler (VPK-07).

| Bağlantı | Parça | Konum / eksen | Bağlanma |
|---|---|---|---|
| F-TRUNNION (L/R) | YK250-CH-070 | ana takım mafsalı, eksen x; iki burç 20 H7, mafsalın ±49 mm'sinde | takım kirişinin yapıştırılmış ankrajlarına 5 × M6 12.9 (başlar kuyuda), kuyu tavanının yakıt tarafındaki sızdırmaz kubbe somun plakalarına 4 × M6 12.9; kulak kenar mesafesi 2 D'nin altında: kulak istisnası, Bruhn kesme-ezilmesiyle denetlenir |
| F-UPLOCK (L/R) | YK250-CH-072 | ana takım yukarı kilidi, kuyu tavanının altında | 4 × M5 (eksen z, 20 mm kare), dökme insertler; yaylı kanca takım biriminin parçasıdır |
| F-NG-PIVOT | YK250-CH-071 | burun takımı mafsalı, eksen y, burç 16 H7 | omurga duvarlarının **iç** yüzlerinde iki 7075 blok 64 × 46 × 6 mm, her biri flanşlı burç (dış çap 22) ve 4 × M6 |
| F-SPINDLE-NODE (L/R) | YK250-CH-095 | stabilatör düğümü: yangın perdesinin arka yüzünden 92 mm geriye konsol; iç yanakta iç yatak 61805-ZZ (25 × 37 × 7, yüksek sıcaklık gresi), dış yatak kök parçasının uç kaburgasında | taban flanşı 8 mm, 7 × M5 12.9 perde yığınından (önde 7075 destek levhası); dış yanak 7 mm, kök parçası arka kiriş kök bağlantısı 4 × M6 12.9 (76 × 62 mm dikdörtgen); kenar uzun kirişi ön dile 2 × M5 Ti |
| F-FW-CORNER (L/R) | YK250-CH-086 | yangın perdesi üst köşesi | tek parça: motor kafesi üst ayağı (taban levhası 10 mm, 2 × M8 12.9 perdeden), dikey arka kiriş kök kulağı (arka çatal kulağı, 2 × M6 12.9 çift kesme), sırt uzun kirişi ön ek levhası (perdenin ön yüzünde, M5 Ti) |
| F-EMOUNT-LO (L/R) | YK250-CH-087 | motor kafesi alt ayakları, yangın perdesinde | ayak pedi 56 × 32 mm, 2 × M8 12.9 perde yığınından (aralık 24 mm, kenar 16 mm); ön yüzde 5 mm 7075 destek levhası, paslanmaz ara borular |
| F-FIN-FRONT (L/R) | YK250-CH-096 | dikey ön kirişi, FS3480 arka yüzü | çatal 42 × 60 mm: dikey ön kiriş kök kulağı 2 × M6 12.9 çift kesme, çerçeveye 2 × M6 |
| F-STUB-FRONT (L/R) | YK250-CH-098 | kök parçası ön kirişi, FS3480 gövde yanında | çatal 40 × 53 mm: kök kulağı 2 × M6 12.9 çift kesme, çerçeveye 2 × M5; arka kiriş düğüm bağlantısında |
| F-VENTRAL-1/2/3 | YK250-CH-099…101 | arka omurga kirişi üzerinde | çatal 32 × 24 mm (1 × M6 omurgaya), ventral kök kulağı 1 × M6 12.9 çift kesme |
| F-FORK (L/R) | YK250-CH-053 | dış panel birleşimi (§5.2) | eldivenin ana kiriş kutusu: UD başlıklar, ±45 kulaklar, burçlar; metal bağlantı yok |
| F-REARSLOT (L/R) | YK250-CH-054 | arka kiriş yuva bağlantısı (§5.2) | 2 × M4 Ti birleşim kaburgasına, 2 × M5 Ti arka kiriş gövdesine |
| F-RISER-FWD / F-RISER-AFT | YK250-CH-112/113 | paraşüt kayış bağlantıları, sırt omurga kanalının iki ucunda (FS1810 ve arka kiriş çerçevesi) | U-kulak: 4 × M4 12.9 kanal tabanından (x bileşeni ve bağlantı momenti; somunlar 48 × 40 × 3 mm 7075 pul levhasında), 2 × M5 12.9 çerçeve bandına (düşey bileşen); Ø8 Ti-6Al-4V kilit pimi, 4130 kayış makarası, iki 6 mm kulak (boşluk 0,5 mm), e 15,5 mm |

### 5.4 Motor bağlantısı

Limbach L 275 EF ve SG750 marş/jeneratörü, krank ekseni itki ekseniyle çakışacak biçimde 5° aşağı itki açısıyla kurulur. Kaynaklı 4130 N boru kafes, yangın perdesindeki dört ayaktan (iki köşe bağlantısı F-FW-CORNER, iki alt ayak F-EMOUNT-LO) karter montaj yüzünün 30 mm önündeki kaynaklı bir halkaya gider; halka itki eksenine diktir. Halkadaki dört kaptaki elastomer sönümleyiciler motoru karterin dört arka göbeğinden taşır. Bağlantı 4 × M8 12.9 (ISO 4762 + NORD-LOCK, 12 mm diş kavraması, çiftler halinde emniyet teli) ile yapılır. Sönümleyici: Limbach'ın isteğe bağlı "Damper Shock Mount" ürünü; boyutları ve rijitliği yayımlanmadığı için paketleme zarfı tahminidir (konik Ø40 × 25 mm, elastomer koparsa motoru tutan emniyet pulu). **Cıvata deseni yalnızca paketleme içindir** (veri sayfasından ölçeklenmiş tahmin, ±3 mm); delme şablonu Limbach çiziminden ya da motorun kendisinden alınır. Motor, pervane, spinner, kaporta parçaları, egzoz, kablo fişleri, yakıt hızlı bağlantıları ve 4 × M8 sökülerek sönümleyicilerden geriye doğru çıkarılır (montaj yolu `engine_removal`, 0,15 m); kafes perdede kalır.

### 5.5 Taret asansörü

Taret (HD59, E180 büyüme zarfı) bilyalı vidalı bir asansör üzerindedir: iki minyatür profil ray (MGN9 sınıfı) çapraz iki bölme köşesinde, ray uçları FS1110 ve FS1330'a bağlı; bilyalı vida (Ø8 × 2 mm) ve yay frenli BLDC redüktörlü motor bölme tavanında. Strok 0,128 m'dir. İki kayar kapak V karnı boyunca bölme duvarlarının dışına kayar; her biri kendi Volz DA 22 eyleyicisiyle sürülür: eyleyici FS1330'un arka yüzündedir ve tahrik mili C-TDOOR-SHAFT kesiğinden bölmedeki kremayere gider. Kapakların ve raylarının dış hatları `layout.mechanisms.door_outlines` içindedir; kayma bantlarında açıklık halkası parçası dışında hiçbir sökülebilir kesik yoktur; halka parçasının 2,0 mm laminatı ve gömülü burçları kaplama kalınlığının içinde kalır, kapak bandına hiçbir şey taşmaz (VPK-08). Tahrik erişim kapağı (P-TDOORACC) FS1330 ile FS1490 arasında, bandın dışındadır. Açıklık halkası parçası (P-TURRETRING) 271 × 250 mm'lik gömülü bir dolu laminattır: FS1110 / FS1330 başlıklarına ve bölme duvarlarının dışa dönük flanşlarına 14 × M4 ile (≈ 67 mm aralık) oturur; bağlantı sıraları değiştirilebilir E180 halkasının 0,19 m açıklığından en az 17 mm uzaktadır. Kilitleme: asansör yalnızca iki kapağın açık algılayıcısı kapalıyken hareket eder, kapaklar yalnızca asansör üst sınır anahtarındayken kapanır. Yük yolu: taret ataleti → taşıyıcı → raylar → bölme duvarları → FS1110 / FS1330; tahrik yükü → bilyalı vida → tavan.

### 5.6 Paraşüt

UAVOS 200 paraşüt kabı (0,30 × 0,30 × 0,275 m, 4,5 kg) kanadın önündeki bölmededir; kabın ağzı sırt kapağının altındadır. **Kapak menteşesizdir** (VPK-11): V çatıyı izleyen tek parça sandviç kapak 376 × 376 mm'dir, FS1490 / FS1810 ve bölme duvarlarının dışa dönük flanşlarına (kap izdüşümünün dışında) dört konum diliyle oturur; FTS komutuyla pim çekici mandal (YK250-SY-804) dört köşe pimini geri çeker, açılan kanopi paketi kapağı düz yukarı kaldırır ve kapak 1,5 m'lik aramid bağla FS1810'a bağlı kalır. Açık geçiş 313 × 312 mm'dir; 300 × 300 mm'lik kap yukarı çıkarılır (montaj yolu `parachute_removal`). **Y-kayış**: ön ayak sırt omurga kanalının ön ucundaki U-kulağa (F-RISER-FWD, FS1810), arka ayak kanalın arka ucundaki U-kulağa (F-RISER-AFT, arka kiriş çerçevesi) gider; arka ayak kanalda yırtılır GFRP örtünün (P-SPINE) altında durur. Birleşme halkası MTOM ağırlık merkezinin 1,5 m üstündedir; uçak yaklaşık 3° burun aşağı asılır. **Açılma şoku 13,1 kN**'dur (Galaxy GRS 4/240'ın yayımlanmış değeri; UAVOS 200'ün 5 g × MTOM değerinden büyüktür) ve ihtiyatlı olarak her ayağın tek başına taşıdığı varsayılır; yalnız nihai durumdur ve 1,15 bağlantı katsayısıyla çarpılır. **Yük yolu** (S1-04): kanopi → kayış → birleşme halkası → iki ayak → U-kulaklar → x bileşenleri sırt omurga kanalına (karşıt bileşenler kanalda çubuk kuvveti, net bileşen vidalı görev bölmesi üst kaplamasından kesmeyle kenar uzun kirişlerine), z bileşenleri FS1810 / arka kiriş çerçevesinin dolu bantlarına → kenar uzun kirişleri, omurga ve kanat kutusu.

### 5.7 Yakıt bölmeleri, hatlar ve tepsiler

Üç yakıt hücresi (ATL esnek hücreler) ok açılı kiriş çerçevelerini izleyen bölmelerdedir: ön hücre FS-FUEL ile ana kiriş çerçevesi arasında, eyer hücre kutunun üstünde iki kiriş arasında, arka hücre arka kiriş çerçevesi ile FS-GEAR arasında. Her bölmede kaplamaya, çerçevelere ve güvertelere yapıştırılmış, keskin kenarsız 2 katlı CFRP bir astar vardır; hücre altı cırt bantlı askıdan ve iki tutma kayışından asılır. Her bölmenin üstünde contalı, somun plakalı bir yakıt kapağı vardır (P-FUEL1…3, L/R). Düzeltme turunda (VPK-04) kapakların ön ve arka kenarları ok açılı çerçevelere paralel yapıldı ve çerçeve başlıklarının T flanşlarına oturur (her kenar 25 mm'lik bant; komşu kapaklar bir başlığı 6 mm aralık ve gömülü dolgu şeridiyle paylaşır); iç kenarlar sırt omurga kanalının flanşına (y ≥ 0,055), dış kenarlar sabit orta kaplamanın bindirmeli yuvasına oturur. Yırtılır kayış örtüsü bu flanşların iç 25 mm'sine ayrı oturur. `layout_check`, ok açılı bölmelerin kullanılabilir hacmini (51,0 L) gereken hacimle (41,9 L) karşılaştırır.

**Yakıt hatları** (`layout.fuel_lines`, 9 hat): ikmal (Ø16, P-REFUEL'deki kuru bağlantıdan ön güvertedeki sızdırmaz geçişle ön hücreye; ana demetten 32 mm uzakta), iki aktarma (FS-MS ve FS-RS'deki sızdırmaz rakorlar), besleme, dönüş, havalandırma ve boşaltma; her biri çap, yol ve tanımlı çerçeve / güverte geçişleriyle yazılıdır ve `layout_check` bunları çakışma, OML ve kesik denetimlerine katar (VPK-09).

Teçhizat tepsileri: yan bölmelerde 3 mm CFRP tepsiler (otopilot tarafında 4 elastomer sönümleyici), ön bölmede batarya kutusu ve FTS birimi tepsisi, görev bölmesinde taban kesiğini kapatan sökülebilir TR-MISSION tepsisi (alttan 8 × M4), arka bölmede 2 mm 6061-T6 tepsi ve karın yük bölmesinde ön yakıt güvertesinin altındaki iki ray üzerinde araştırma yükü tepsisi (4 × M5 tutsak vida).

### 5.8 Tasarım yükleri ve ilk ön boyutlandırma

Tasarım yükleri `structures` bölümünden gelir: CS-LUAS'a uyarlanmış STANAG 4703, sınır yük katsayıları +3,8 / −1,52 ve rüzgâr hamlesi zarfı, emniyet katsayısı 1,5, bağlantı katsayısı 1,15, pimli bağlantılarda ezilme katsayısı 2,0, sık sökülen bağlantılarda ek 1,5; acil iniş nihai atalet yükleri 9 g ileri / 3 g yukarı / 1,5 g yana / 6 g aşağı. `layout_check` (C13), arayüz bağlantılarının ilk ön boyutlandırmasını kapalı biçimli formüllerle yapar. Güncel sonuçlar:

| Kalem | Emniyet payı |
|---|---|
| kanat birleşimi ana pimi, çift kesme | 3,48 |
| kanat birleşimi ana pimi, eğilme | 0,09 |
| kanat birleşimi CFRP dil: burç ezilme basıncı (dış çap 22 x 30 mm, açık delik bası sınırı) | 0,63 |
| kanat birleşimi CFRP çatal kulakları: burç ezilme basıncı (2 x dış çap 22 x 10 mm) | 0,07 |
| kayış bağlantısı çerçeve cıvataları 2 x M5 12.9 (tek kesme, şokun tamamı bu grupta) | 0,38 |
| kayış bağlantısı omurga tabanı cıvataları 4 x M4 12.9 (tek kesme, şokun tamamı bu grupta) | 0,71 |
| kayış kilit pimi Ø8 (Ti-6Al-4V, çift kesme) | 3,00 |
| kayış kilit pimi Ø8 eğilmesi (Melcon-Hoblit) | 0,30 |
| kayış U-kulak ezilmesi (7075, 2 kulak 6 mm, e/D 1,94) | 2,09 |
| motor bağlantı cıvatası M8 12.9 (tork + 3,8 g / 1,47 g yan / 6 g aşağı durumlarının en kötüsü) | 19,96 |

Bu ilk boyutlandırmanın yerini yapı evresinin el hesapları ([04](04_yapi_hesaplari.md), `out/structures.md`) alır: düzlem içi moment kuvvet çiftiyle birlikte ana pim eğilmesinde MS 0,208, çatal kulağı burcu ezilmesinde 0,195'dir ve bütün paylar sıfır ya da pozitiftir.

## 6. Kabuk: panel bölümlemesi ve bağlama

![Kabuk paneli bölümlemesi](fig/yk250_layout_shell.png)

**Bağlama kuralları** (`layout.shell.rules`):

* Yapısal (sabit, yarı kalıcı) kaplamalar: ISO 7380 M4 A2-70 vidalar, yüzer M4 somun plakalarına, 25–32 mm aralıkla (≥ 3 D, ≤ 8 D). Her kaplama değiştirilebilir.
* Erişim kapakları ve kaportalar: Camloc 4002 çeyrek tur (perçinli 2600 yuva), 75–100 mm (kaportada 70–90 mm) aralıkla.
* Filetolar ve küçük RF pencereleri: M4 vida, gömülü M4 dişli burçlara (insert), 60–100 mm.
* Yakıt kapakları: M4 + yüzer somun plakaları, 25–30 mm, yakıta dayanıklı floro-silikon conta (buhar sızdırmaz).
* LERX/eldiven kaplamaları: EA 9394 yapıştırma, 0,2 mm yapıştırıcı kalınlığı, panel uçlarında her 150 mm'de M4 kör perçin (soyulma durdurucu).
* Kenar mesafesi: kompozitte bağlantı elemanı merkezi ile panel ve yuva kenarı arası ≥ 2,5 D (M4 için 10 mm, Camloc için 12 mm), metalde ≥ 2,0 D; oturma yüzeyi (land) genişliği ≥ 25 mm.
* Sandviç kenarları: her bağlantı hattında, bindirmede ve menteşede çekirdek 1:3 eğimle 20 mm genişliğinde 1,6 mm'lik dolu laminata iner.
* Bindirmeler (joggle): sökülebilir paneller komşu sabit kaplamanın bindirmeli yuvasına ya da çerçeve/uzun kiriş flanşlarına gömülü oturur: bindirme derinliği = kenar bandı + 0,3 mm, eğim 1:10 (19 mm rampa), yuva 25 mm; panel aralığı 1,0 ± 0,3 mm, basamak ≤ 0,3 mm. İki sökülebilir panel arasındaki sabit kaplama şeridi ya 2 × 19 mm rampa + 20 mm bağlantı sırası kadar geniştir ya da iki panel ortak bir yapısal oturma yüzeyini (çerçeve başlığı, kanal flanşı) paylaşır.
* Contalar: kapaklar ve takım/taret kapakları çevre contalıdır; yakıt kapakları yakıta dayanıklı conta taşır.
* Katmanlar: gövde kaplamaları ve kapaklar `layups.shell_secondary` (0,4 / 5 / 0,4 mm), LERX/eldiven `layups.wing_skin_primary` (kirişler arası üst kaplama `layups.wing_box_skin_upper`), RF pencereleri `gfrp_7781_mtm45` (pencerede karbon yok), kaporta `shell_secondary` + egzoz çıkışlarında paslanmaz ısı kalkanı yamaları.

**Erişim kapakları kendi yüzeylerinde kalır ve kenarları yapıya oturur:** `layout_check`, her sökülebilir ve menteşeli gövde kapağının kenar çizgisinin (chine) en az 25 mm içinde ve eldiven kapağının hücum kenarının en az 40 mm gerisinde olduğunu, kenar bandının her noktasının listelenen bir oturma yüzeyinin (çerçeve başlığı, uzun kiriş ya da kanal flanşı, komşu sabit kaplamanın bindirme yuvası) üstünde olduğunu ve listelenen her oturma yüzeyinin gerçekten bir kenarı taşıdığını denetler (VPK-04, VPK-12). Sabit kuyruk yüzeylerinin (dikey, kök parçası, ventral) kök kesim çizgileri `layout.shell.root_cut_lines` içinde tanımlıdır ve hiçbir sökülebilir panelden geçmez (VPK-05): üst kaporta dikey kökleri çevresinde orta parça + iki yan parçaya, alt kaporta ve stabilatör eyleyici kapağı arka omurga / ventral şerit üzerinde buluşan sağ/sol yarımlara bölündü; kökler sabit şeritlerde (P-FINROOT-AFT, P-STUBROOT, P-VENTRALROOT) kalır.

Tablo `spec.yaml`'dan üretilmiştir (x ve y aralıkları panelin dış sınırlarıdır; (L/R) sancak tanımının iskeleye aynalandığını gösterir):

| Panel | Parça | Bölge | x (m) | y (m) | Bağlantı | Bağlama elemanı | Malzeme / katman |
|---|---|---|---|---|---|---|---|
| P-NOSECONE | YK250-SH-350 | burun konisi (radom) | 0,000–0,300 | -0,10…0,10 | sökülebilir | FS0300 flanşına 8 × M4 somun plakası | gfrp_7781_mtm45 / shell_secondary |
| P-FWDSKIN | YK250-SH-351 | ön gövde kaplaması | 0,300–0,600 | -0,17…0,17 | sabit | somun plakası + vida, 25–32 mm | cfrp_pw_mtm45_as4 / shell_secondary |
| P-FWDHATCH | YK250-SH-353 | ön bölme kapağı (tampon batarya, FTS) | 0,3061–0,5996 | -0,13…0,13 | sökülebilir | Camloc, 75–100 mm | cfrp_pw_mtm45_as4 / shell_secondary |
| P-NOSE-LOWER | YK250-SH-352 | alt burun kaplaması | 0,600–1,110 | -0,30…0,30 | sabit | somun plakası + vida, 25–32 mm | cfrp_pw_mtm45_as4 / shell_secondary |
| P-AVHATCH | YK250-SH-354 | aviyonik kapağı (GNSS 1 RF penceresi) | 0,603–1,107 | -0,135…0,135 | sökülebilir | Camloc, 75–100 mm | gfrp_7781_mtm45 / shell_secondary |
| P-NOSE-UPPER | YK250-SH-355 | üst burun kaplaması | 0,600–1,110 | -0,30…0,30 | sabit | somun plakası + vida, 25–32 mm | cfrp_pw_mtm45_as4 / shell_secondary |
| P-SIDEBAY-L | YK250-SH-357 | sol yan bölme kapağı (jeneratör güç elektroniği, fren birimi) | 0,830–1,107 | -0,20…-0,05 | sökülebilir | Camloc, 75–100 mm | cfrp_pw_mtm45_as4 / shell_secondary |
| P-SIDEBAY-R | YK250-SH-358 | sağ yan bölme kapağı | 0,830–1,107 | 0,05…0,20 | sökülebilir | Camloc, 75–100 mm | gfrp_7781_mtm45 / shell_secondary |
| P-MID-UPPER | YK250-SH-360 | üst orta kaplama | 1,110–1,490 | -0,37…0,37 | sabit | somun plakası + vida, 25–32 mm | cfrp_pw_mtm45_as4 / shell_secondary |
| P-MID-LOWER | YK250-SH-361 | alt orta kaplama | 1,110–1,490 | -0,37…0,37 | sabit | somun plakası + vida, 25–32 mm | cfrp_pw_mtm45_as4 / shell_secondary |
| P-TURRETRING | YK250-SH-363 | taret açıklık halkası (HD59) | 1,0816–1,3584 | -0,125…0,125 | sökülebilir | 14 × M4, gömülü burç (çerçeve başlıkları ve bölme duvarı flanşları), ≈ 67 mm | cfrp_pw_mtm45_as4 / 2,0 mm dolu laminat |
| P-TDOORACC (L/R) | YK250-SH-365 | taret kapağı tahrik erişim kapağı | 1,330–1,490 | 0,10…0,22 | sökülebilir | Camloc, 75–100 mm | cfrp_pw_mtm45_as4 / shell_secondary |
| P-PARA-SURR | YK250-SH-366 | paraşüt kapağı çevre kaplaması | 1,490–1,810 | -0,40…0,40 | sabit | somun plakası + vida, 25–32 mm | cfrp_pw_mtm45_as4 / shell_secondary |
| P-PARAHATCH | YK250-SH-367 | paraşüt kapağı (sırt, bağlı fırlatmalı) | 1,462–1,838 | -0,188…0,188 | sökülebilir | menteşesiz: 4 köşe pimi + pim çekici mandal, 1,5 m aramid bağ | cfrp_pw_mtm45_as4 / shell_secondary |
| P-PARA-LOWER | YK250-SH-368 | paraşüt bölmesi alt kaplaması | 1,490–1,810 | -0,40…0,40 | sabit | somun plakası + vida, 25–32 mm | cfrp_pw_mtm45_as4 / shell_secondary |
| P-MB-UPPER | YK250-SH-370 | görev bölmesi üst kaplaması | 1,810–2,1895 | -0,40…0,40 | sabit | somun plakası + vida, 25–32 mm | cfrp_pw_mtm45_as4 / shell_secondary |
| P-GNSS2 | YK250-SH-372 | GNSS 2 RF penceresi | 2,0495–2,1295 | 0,11…0,19 | sökülebilir | M4 + gömülü burç, 60–100 mm | gfrp_7781_mtm45 / shell_secondary |
| P-MBHATCH | YK250-SH-373 | görev bölmesi kapağı | 1,813–2,1895 | -0,155…0,155 | sökülebilir | Camloc, 75–100 mm | cfrp_pw_mtm45_as4 / shell_secondary |
| P-MB-LOWER | YK250-SH-374 | görev bölmesi alt kaplaması | 1,810–2,1895 | -0,40…0,40 | sabit | somun plakası + vida, 25–32 mm | cfrp_pw_mtm45_as4 / shell_secondary |
| P-SPINE | YK250-SH-376 | yırtılır kayış örtüsü (sırt kanalı üstü) | 1,813–2,816 | -0,045…0,045 | sökülebilir | cırt bant + 4 naylon M3 kesme vidası (yırtılır) | gfrp_7781_mtm45 / shell_secondary |
| P-FUEL1 (L/R) | YK250-SH-377 | ön yakıt bölmesi kapağı | 2,1925–2,5262 | 0,055…0,30 | sökülebilir | M4 + somun plakası + yakıta dayanıklı conta, 25–30 mm | cfrp_pw_mtm45_as4 / shell_secondary |
| P-FUEL2 (L/R) | YK250-SH-378 | orta yakıt bölmesi kapağı | 2,4977–2,8222 | 0,055…0,30 | sökülebilir | M4 + somun plakası + yakıta dayanıklı conta, 25–30 mm | cfrp_pw_mtm45_as4 / shell_secondary |
| P-FUEL3 (L/R) | YK250-SH-379 | arka yakıt bölmesi kapağı | 2,8077–3,0924 | 0,055…0,30 | sökülebilir | M4 + somun plakası + yakıta dayanıklı conta, 25–30 mm | cfrp_pw_mtm45_as4 / shell_secondary |
| P-CENTRE-UPPER | YK250-SH-380 | üst orta gövde kaplaması | 2,1895–3,0954 | -0,40…0,40 | sabit | somun plakası + vida, 25–32 mm | cfrp_pw_mtm45_as4 / shell_secondary |
| P-PAYHATCH | YK250-SH-382 | faydalı yük bölmesi kapağı (karın) | 2,1925–2,8406 | -0,222…0,222 | sökülebilir | Camloc, 75–100 mm | cfrp_pw_mtm45_as4 / shell_secondary |
| P-CENTRE-LOWER | YK250-SH-383 | alt orta gövde kaplaması | 2,1895–3,0954 | -0,40…0,40 | sabit | somun plakası + vida, 25–32 mm | cfrp_pw_mtm45_as4 / shell_secondary |
| P-REFUEL | YK250-SH-385 | yakıt ikmal kapağı (sol) | 2,300–2,380 | -0,35…-0,29 | menteşeli | gömülü düz menteşe + itmeli mandal | cfrp_pw_mtm45_as4 / shell_secondary |
| P-AFTHATCH | YK250-SH-387 | arka teçhizat kapağı (karın) | 3,0954–3,480 | -0,12…0,12 | sökülebilir | Camloc, 75–100 mm | cfrp_pw_mtm45_as4 / shell_secondary |
| P-STABACT (L/R) | YK250-SH-389 | alt kapak FS3480-yangın perdesi, sağ/sol yarımlar (stabilatör eyleyicileri) | 3,4806–3,6552 | 0,022…0,18 | sökülebilir | Camloc, 75–100 mm | cfrp_pw_mtm45_as4 / shell_secondary |
| P-VENTRALROOT | YK250-SH-394 | ventral kök şeridi | 3,4806–4,000 | -0,022…0,022 | fileto | M4 + gömülü burç, 60–100 mm | cfrp_pw_mtm45_as4 / shell_secondary |
| P-AFT-LOWER | YK250-SH-388 | alt arka kaplama | 3,0954–3,670 | -0,40…0,40 | sabit | somun plakası + vida, 25–32 mm | cfrp_pw_mtm45_as4 / shell_secondary |
| P-AFT-UPPER | YK250-SH-390 | üst arka kaplama | 3,0954–3,670 | -0,40…0,40 | sabit | somun plakası + vida, 25–32 mm | cfrp_pw_mtm45_as4 / shell_secondary |
| P-INLET | YK250-SH-391 | sırt soğutma girişi | 3,180–3,440 | -0,09…0,09 | sabit | EA 9394 yapıştırma, uçlarda kör perçin | cfrp_pw_mtm45_as4 / shell_secondary |
| P-FINROOT (L/R) | YK250-SH-392 | dikey kök örtüsü | 3,268–3,670 | 0,10…0,22 | fileto | M4 + gömülü burç, 60–100 mm | cfrp_pw_mtm45_as4 / shell_secondary |
| P-FINROOT-AFT (L/R) | YK250-SH-395 | motor bölmesi üstü dikey kök kaplaması (sabit) | 3,670–3,965 | 0,125…0,197 | fileto | M4 + gömülü burç, 60–100 mm | cfrp_pw_mtm45_as4 / shell_secondary |
| P-STUBROOT (L/R) | YK250-SH-393 | kök parçası fileto kaplaması | 3,415–3,962 | 0,236…0,30 | fileto | M4 + gömülü burç, 60–100 mm | cfrp_pw_mtm45_as4 / shell_secondary |
| P-COWL-UP | YK250-SH-450 | üst motor kaportası, orta parça | 3,670–4,000 | -0,236…0,236 | sökülebilir | Camloc, 70–90 mm | cfrp_pw_mtm45_as4 / shell_secondary |
| P-COWL-UPS (L/R) | YK250-SH-452 | üst motor kaportası, yan parça | 3,670–3,965 | 0,197…0,236 | sökülebilir | Camloc, 70–90 mm | cfrp_pw_mtm45_as4 / shell_secondary |
| P-COWL-LO (L/R) | YK250-SH-451 | alt motor kaportası, sağ/sol yarımlar | 3,670–4,000 | 0,022…0,236 | sökülebilir | Camloc, 70–90 mm | cfrp_pw_mtm45_as4 / shell_secondary |
| P-GLOVE-UP (L/R) | YK250-SH-420 | LERX/eldiven üst kaplaması | 1,800–3,0218 | 0,40…0,70 | sabit | EA 9394 yapıştırma, uçlarda kör perçin | cfrp_pw_mtm45_as4 / wing_skin_primary |
| P-GLOVE-LO (L/R) | YK250-SH-421 | LERX/eldiven alt kaplaması | 1,800–3,0218 | 0,40…0,70 | sabit | EA 9394 yapıştırma, uçlarda kör perçin | cfrp_pw_mtm45_as4 / wing_skin_primary |
| P-JOINTACCESS (L/R) | YK250-SH-422 | kanat birleşim erişim kapağı (alt eldiven) | 2,0605–2,5585 | 0,406…0,694 | sökülebilir | Camloc, 75–100 mm | cfrp_pw_mtm45_as4 / shell_secondary |
| P-REARACCESS (L/R) | YK250-SH-423 | arka pim erişim deliği (alt eldiven) | 2,861–2,891 | 0,657…0,687 | sökülebilir | gömülü süngü kapak (Ø30, bağlama elemanı yok) | cfrp_pw_mtm45_as4 / shell_secondary |

**RF pencereleri.** Gövde içindeki her anten GFRP bir panelin ya da dikey uç kapağının altındadır: GNSS 1 aviyonik kapağında, GNSS 2 görev bölmesinin üstündeki küçük pencerede (ikisi de gökyüzünü görür), birincil veri bağı anteni A, FTS anteni ve uzaktan kimlik vericisi (veri bağı radyosunun gölgesinden çıkarıldı) GFRP burun konisinde, veri bağı anteni B ve yedek C2 anteni dikey uç kapaklarında. Anten zarfları iç kaplama yüzeyinin en az 2 mm içindedir (OML − 5,8 mm kaplama − 2 mm). Transponder anteni karın altında, burun takımının önündedir (bakır ağ zemin düzlemli bıçak anten). `layout_check`, iç antenlerin RF geçirgen bir panelin altında olduğunu, pencerelerinin en az %50'sini karbon / metal yapıya ve teçhizata takılmadan gördüğünü (görüş hattı) ve taret açıkken bütün dış çıkıntıların taretin −5° görüş konisinin üstünde kaldığını denetler (VPK-10).

## 7. Mekanizmalar

Bütün mafsallar `core.parts.Joint` biçimindedir ve **dinlenme konumunda** tanımlıdır: takım açık, taret içeride, yüzeyler nötr, kapak kapalı. Üreticiler bu adları aynen kaydeder; `checks.py` ve `layout.clearances` bu adlara başvurur. İskele mafsalı, aynı kumanda değerinin aynı fiziksel hareketi vermesi için aynalanmış eksenle tanımlanır.

| Mafsal | Tür | Aralık | Kumanda özelliği / ifade | Hareket ettirdiği |
|---|---|---|---|---|
| aileron_R / _L | döner | −20° … +20° | aileron_deg | kanatçıklar YK250-FC-200-R/L |
| flap_R / _L | döner | 0° … 40° | flap_deg | flaplar YK250-FC-202-R/L |
| rudder_R / _L | döner | −25° … +25° | rudder_deg | dümenler YK250-FC-300-R/L (dikey açıklığının %14,4'ünden %84,4'üne) |
| stabilator_R / _L | döner (eksen y) | −20° … +15° | elevator_deg | stabilatörler + miller + kollar |
| main_gear_R / _L | döner (eksen x) | 0° … 108° | gear_up (0,15–0,85) | ana bacak, teker/fren, bacak kapağı |
| main_inner_door_R / _L | döner | 0° … 95° | gear_up: 0–0,15 açılır, 0,85–1 kapanır | ana takım iç kapakları |
| nose_gear | döner (eksen y) | 0° … 90° | gear_up (0,15–0,85) | burun bacağı, teker, yönlendirme eyleyicisi |
| nose_steer | döner (bacak ekseni) | −20° … +20° | steer_deg (yalnız takım açıkken) | burun tekeri + çatal |
| nose_door_R / _L | döner | 0° … 90° | takım kilitlenene kadar açık, kilitlenince kapanır | burun istiridye kapakları |
| turret_elevator | kayar (−z) | 0 … 128 mm | turret (0,2–1) | taret + taşıyıcı |
| turret_door_R / _L | kayar (V karın boyunca) | 0 … 120 mm | turret (0–0,2) | taret bölmesi kayar kapakları |
| para_hatch | kayar (+z, menteşesiz) | 0 … 0,15 m (modellenen kalkış, sonra bağla serbest) | para_hatch | paraşüt kapağı |
| prop_spin | döner (itki ekseni) | 0° … 120° | prop_deg | pervane + spinner (bir pala geçişi süpürme hacmini kapsar) |

**Diziler.** `gear_retraction` 21 örnekli durumdur: iç kapaklar takım hareketinden önce açılır (gear_up 0–0,15), bacaklar 0,15–0,85 arasında toplanır, iç kapaklar kilitlemeden sonra kapanır (0,85–1); burun yönlendirmesi toplamadan önce ortalanır. `turret_extension` 13 örnekli durumdur: kapaklar önce tam açılır (turret 0–0,2), asansör sonra iner (0,2–1); geri çekilmede tersi. `layout_check` bu dizileri adım adım süpürür: ana teker – kuyu yapısı en az 12 mm, bacaklar ≥ 10 mm, iç kapak – hareketli takım ≥ 10 mm, burun tekeri – omurga yuvası ≥ 12 mm; taret E180 zarfı – bölme duvarları ≥ 6 mm, raylar/vida ≥ 5 mm, kayar kapaklar ≥ 5 mm; HD59 topu – açıklık halkası 5 mm. Kumanda yüzeyleri bütün sapma aralığında egzoz zarflarına, duman konisine ve kanat eyleyicilerine; dümen kökü ±25°'de gövde / kaporta yüzeyine göre denetlenir.

**Kapak dış hatları** (`layout.mechanisms.door_outlines`): ana iç kapak, ana bacak kapağı, burun istiridye kapağı ve taret kayar kapağı (rayıyla) çokgen olarak tanımlıdır; takım ve kabuk modülleri aynı dış hattı okur.

**Montaj ve bakım yolları** (`layout.mechanisms.assembly_paths`, 15 tanım, her biri eksen, strok ve zarfla; sancak tanımları iskeleye aynalanır): dış panelin takılması (dil, arka kulak), iki ana pim ve çekiciler, iki rayba (adım 5, eldiven kaplamalarından önce), arka pim, motor, taret, batarya, paraşüt kabı, görev tepsisi ve ECU, stabilatörler, pervane. `layout_check` bu 23 süpürmenin hiçbir parçaya değmediğini denetler (VPK-02, VPK-09).

**Yasak bölgeler** (`layout.keep_outs`): motor dinamik zarfı (krank ekseni eğik, 10 mm sönümleyici payı), pervane diski (R + 26 mm radyal, pala yarı kalınlığı + 13 mm boyuna; CS-VLA 925(c)), silindir kafası sıcak bölgesi (25 mm içinde kompozit yok), iki egzoz zarfı (kompozite 50 mm, kalkanla 25 mm; 15°/0,30 m duman konisi ventral, stabilatör ve takıma değmez), taret görüş konisi (−5°), soğutma S-kanalı koridoru, paraşüt açılma hacmi, kanat dili takma yolu, batarya–yakıt ve yakıt–yangın perdesi ayrım kuralları; ayrıca ana ve burun takımı süpürme hacimleri, kumanda yüzeyi süpürme hacimleri, kablo demeti koridorları ve itme çubuğu koridorları. `layout.clearances` checks.py'nin LİSTE biçimindedir (`{name, a, b, min_mm, joints}`, 26 kural); ayrıntılı tasarımın gerçek parçaları bunlarla süpürülerek denetlenecektir.

## 8. Sistemler

* **Güç:** SG750 → jeneratör güç elektroniği (sol yan bölme) → 28 V PDU (VISIONAIRtronics 1000 W, aviyonik güverte) → kontaktör/sigorta bloğu ve 28-12 V DC-DC. 12S2P Li-ion tampon batarya ön bölmede, havalandırmalı ve hücre sigortalı kutuda; havalandırma çıkışı ön bölme kapağındaki alev tutucu ızgaradan. Ön bölme kapağı kenar çizgisinin daralmasını izleyen bir yamuktur; açık geçişi bataryanın 184 × 77 mm izdüşümünü geçirir (VPK-03).
* **Aviyonik:** Veronte 1x otopilot (GNSS/IMU, hava verisi), Silvus SC4200EP birincil veri bağı, Microhard pDDL2450 yedek C2 bağı, uAvionix ping200X transponder — sağ yan bölmede; Dronetag uzaktan kimlik vericisi burun konisinde (VPK-10). Bağımsız FTS birimi ön bölmede bataryanın altında.
* **Hava verisi ve ışıklar:** ısıtmalı burun pito-statik sondası, sol kanat altında ikinci (DroneCAN) pito; kanat uçlarında seyir/çakar ışıkları, dikey ucunda beyaz kuyruk ışığı.
* **Eyleyiciler:** kanatçık Volz DA 26, flap Volz DA 30 (dış panelde); dümen Volz DA 26 (dikey kutusunda); stabilatör Volz DA 30 (yangın perdesinin serin ön yüzünde, itme çubuğu yangına dayanıklı körükle perdeden geçer ve mil koluna gider); iç kapaklar Volz DA 22 (FS-GEAR arka yüzü); taret kapakları Volz DA 22 (FS1330 arka yüzü, tahrik mili kesikten); fren Volz DA 26 + ana silindir (sol yan bölme); ana ve burun takımı döner EMA'ları mafsal eksenlerinde.
* **Kablo demetleri:** H-MAIN (ana güç/veri, sol), H-COAX (koaksiyel/veri, sağ), H-WING (her yan; görev bölmesindeki gövde demetinden ön yakıt güvertesinin altından orta kanat kutusunun alt kapağına, oradan gövde yanı ve eldiven kaburgalarındaki rondelalardan birleşim kaburgasındaki kör takılan konnektöre; buhar sızdırmaz bölmeye girmez, VPK-13), H-TAIL, H-ENGINE (yangın perdesinden yangına dayanıklı rondelalarla), H-NOSE. Kurallar: demet koridoru = çap + 10 mm; her çerçevede rondela; güç ve RF koaksiyel arası ≥ 20 mm; egzoza ≥ 50 mm; yakıt bölmesinde demet yok.

## 9. Üretim yaklaşımı (parça ailelerine göre)

| Parça ailesi | Süreç | Kurallar |
|---|---|---|
| Sandviç çerçeveler, duvarlar, güverteler, kaburgalar | önceden emdirilmiş PW CFRP (MTM45-1/AS4), otoklav dışı vakum torbası kürü (`prepreg_ooa_vacbag`), ROHACELL 51 WF çekirdek; düz paneller düz kalıpta, CNC ile kesilir | asgari laminat kalınlığı süreç kuralına göre; bağlantı hatlarında 1:3 çekirdek eğimi ve 20 mm'lik dolu kenar bandı; ISO 2768-mK |
| Kenar ve sırt uzun kirişleri, kiriş başlıkları, sırt kanalı | UD MTM45-1 başlık + PW gövde, uzun dişi kalıpta | profil kalıbında ≥ 1° koniklik; uzun kirişler perdenin önünde metal eklerle biter |
| Orta kanat kutusu (kök parça) ve kompozit çatal | kiriş başlıkları ve gövdeleri ayrı kürlenir, kutu tezgâhta yapıştırılır ve cıvatalanır; kırılma bağlantısı 7075 | çatal burçları usta dil mastarıyla yapıştırılır ve önden hat raybalanır (H8), eldiven kaplamalarından önce |
| Kompozit dil (dış panel) | UD flanşlar + ±45 PW gövde, pim bölgelerinde [±45/0/90] bloklar, eş kalıpta | burçlar dil mastarında yapıştırılıp raybalanır; dil ucunda 3 × 30° pah |
| Gövde kaplamaları, kapaklar, kaportalar | `shell_secondary` sandviç (0,4 / 5 / 0,4 mm), erkek ana kalıptan alınan dişi kalıplarda vakum torbası | kapak açıklıkları çerçeveli; bindirmeler kalıpta; delikler matkap şablonundan |
| RF pencereleri, radom | E-cam 7781 / MTM45-1 (karbonsuz), aynı süreç | pencerede karbon ve metal yok |
| Metal bağlantı parçaları (düğüm, köşe, mafsal, kayış, motor, kuyruk), arka omurga | 7075-T651 levhadan 3/5 eksen CNC | kenar mesafesi ≥ 2,0 D (kulaklar Bruhn ile); bükme yok; yüzey anodik oksit (tip II) + astar |
| Motor bölmesi alt U halkası | 2024-T3 sac, şekillendirme; alt parçası talaşlı 7075 | bükme yarıçapı ≥ 6 t (`sheet_metal_aluminium`); sıcak bölgede kompozit yok |
| Motor kafesi | 4130 N boru, TIG kaynak, kaynak sonrası normalizasyon | kaynak fikstüründe itki ekseni ve montaj yüzü referanslı |
| Yangın perdesi kalkanı | 0,4 mm AISI 304, paslanmaz ayaklar ve kenar köşebendi | perçinli; perde geçişleri yangına dayanıklı |
| Yakıt bölmesi astarları | 2 kat CFRP, çerçevelere yapıştırılır | keskin kenar yok; hücre askı noktaları |

Genel kurallar `spec.assembly.general` içindedir: ISO 2768-mK genel toleranslar, ISO 286 geçmeler (pim yuvaları H8/h8, yatak yuvaları H7), EA 9394 yapıştırma (yüzey hazırlığı, 0,2 mm, tanık numunesi, tıklama testi), kompozitte ≥ 2,5 D / metalde ≥ 2,0 D kenar mesafesi, ISO 273 orta sınıf delikler, tork işaretleme, elektriksel bağlama (CS-VLA 857).

## 10. Montaj, taşıma ve bakım

**Montaj sırası:** şasi → sabit kabuk → sistemler → itki → iniş takımı → kuyruk ve kanat → kabuk kapanışı → ayar ve fonksiyon testleri. `spec.assembly.steps` 42 adımdır; her adımın Türkçe başlığı, alt montajı, metni, takımları ve kontrolleri vardır:

1. **Şasi (adım 1–15):** ana montaj tezgâhı; orta kanat kutusu (kök parça); FS-MS ve FS-RS; gövde yanı, eldiven ve birleşim kaburgaları (LERX burun kaburgası yok); kompozit çatal burçları ve arka kiriş yuva bağlantısı (usta dil mastarıyla yapıştırma ve önden hat raybalama; rayba koridoru serbest); ön gövde çerçeveleri; burun kutusu; taret, paraşüt ve görev bölmeleri; kenar uzun kirişleri (perdenin önünde biter); omurga kirişleri, güverteler ve ana takım yapısı; arka gövde; yangın perdesi, köşe bağlantıları ve alt motor ayakları; stabilatör düğüm bağlantıları (iç yatak yuvası işlenmiş halde, mil hizalama fikstürüyle — sahte mil + FS3480 kök parçası ön bağlantısı delikleri — konumlanır ve perdeden 7 × M5 ile bağlanır) ve alt U halka; sırt omurga kanalı, kayış bağlantıları, asansör, yakıt bölmesi ve teçhizat bağlantıları; şasi muayenesi ve tezgâhtan ayırma.
2. **Kabuk ve sistemler (adım 16–24):** sabit gövde kaplamaları (görev bölmesi üst kaplaması kanal flanşlarına da vidalanır); LERX/eldiven kaplamalarının yapıştırılması (çerçeveli birleşim erişim kesiği); yakıt sistemi; elektrik tesisatı; aviyonik ve güç dağıtımı; antenler ve hava verisi; paraşüt sistemi (menteşesiz kapak, mandal, bağ); görev tepsisi ve yük tepsisi rayları; taret asansörü, kayar kapaklar ve HD59.
3. **İtki ve takım (adım 25–30):** motor (4130 kafes, sönümleyiciler, 4 × M8; itki ekseni 5° ±0,2°); egzoz, soğutma, motor yardımcıları; pervane; ana ve burun takımı; takım kapakları ve sıralama.
4. **Kuyruk, kanat, kapanış ve testler (adım 31–40):** stabilatör kök parçaları (dış yatak yuvası düğüm yuvasıyla eş eksenli, aynı fikstürle işlenir), milleri ve eyleyicileri; dikeyler ve dümenler; ventral ve tampon kızağı; stabilatörler; dış kanat panelleri; kabuğun kapatılması (kaporta parçaları, kapaklar); kumanda ayarı; fonksiyon testleri; tartı ve denge; son muayene.
5. **Taşıma ve saha (adım 41–42).**

**Taşıma** dört parçalıdır: orta gövde (eldiven, dikeyler, kök parçaları ve ventral takılı; 4,23 × 1,40 × 1,29 m, takım içeride), iki dış kanat paneli (2,90 m açıklık, dil dahil 3,20 m; R-29 ≤ 3,4 m), iki stabilatör ve pervane. Orta kesit genişliği 1,40 m'dir (R-30 ≤ 2,0 m).

**Sahada montaj** iki kişiyle yaklaşık 35 dakikadır: gövdeyi takozlara koy; birleşim erişim kapaklarını ve arka pim deliği kapağını aç; dış paneli ana kiriş ekseni boyunca sür (dil, arka kulak ve konnektör aynı anda girer), iki Ø16 ana pimi önden, Ø8 arka pimi alttan tak ve emniyetle; konnektörü denetle, kapakları kapat; stabilatörleri mil uçlarına geçir; pervaneyi tak; uçuş öncesi denetim (paraşüt emniyet pimi en son çıkarılır).

**Bakım erişim matrisi.** 23 kalemin hiçbiri için birincil yapı sökülmez (tablo `spec.assembly.maintenance_access`'ten üretilmiştir). `layout_check` her teçhizat ve yakıt hücresi için kapağın aynı yüzde olduğunu, açık geçişinin (panel − 2 × 25 mm oturma) kalemin en küçük kesitinden büyük olduğunu ve söküm prizmasının güverte, taban ve başka teçhizattan boş olduğunu denetler (VPK-03):

| Kalem | Erişim | Bağlama |
|---|---|---|
| bujiler, silindirler, CHT/EGT algılayıcıları, egzoz, hava yönlendiricileri | P-COWL-UP, P-COWL-UPS, P-COWL-LO | Camloc |
| motor (sökme) | P-COWL-UP, P-COWL-UPS, P-COWL-LO | Camloc + 4 x M8 bağlantı cıvatası |
| yakıt pompası/filtresi + boşaltma, iç kapak eyleyicileri, kuyruk konnektörü, motor kablo bağlantısı | P-AFTHATCH | Camloc |
| motor ECU, görev bilgisayarı / kayıt cihazı | P-MBHATCH | Camloc |
| stabilatör eyleyicileri (DA 30), itme çubukları ve körükleri, yakıt kesme vanası | P-STABACT | Camloc |
| stabilatör mili yatakları | P-COWL-UPS, stabilatör sökülerek | Camloc + çapraz cıvata |
| yakıt hücreleri (muayene, değiştirme) | P-FUEL1, P-FUEL2, P-FUEL3 | M4 somun plakalı vida + conta |
| PDU, kontaktör/sigortalar, DC-DC, burun kablo demeti | P-AVHATCH | Camloc |
| Li-ion tampon batarya, bağımsız FTS birimi | P-FWDHATCH | Camloc |
| jeneratör güç elektroniği, fren eyleyicisi + ana silindir | P-SIDEBAY-L | Camloc |
| otopilot, veri bağları, transponder | P-SIDEBAY-R | Camloc |
| burun antenleri, uzaktan kimlik vericisi, pitot hatları | P-NOSECONE | 8 x M4 |
| EO/IR taret (HD59 / E180) | P-TURRETRING | 14 x M4 + 4 taret bağlantı cıvatası |
| taret asansörü, kayar kapaklar | P-TURRETRING, P-TDOORACC | M4 / Camloc |
| paraşüt (12 ayda bir katlama, UAVOS servis ömrü) | P-PARAHATCH | mandal |
| araştırma faydalı yükü | P-PAYHATCH | Camloc |
| ana takım bacakları, EMA'lar, kilitler, frenler, orta kablo kanalı | ana takım kuyuları (takım açık, iç kapaklar bakım modunda) | - |
| ana takım mafsal bağlantıları F-TRUNNION, yukarı kilit bağlantıları F-UPLOCK (bağlantı elemanları) | ana takım kuyuları (takım açık, mafsal kapağı açık) | bağlantı başına 5 x M6 12.9 takım kirişinin yapıştırılmış ankrajlarına (başlar kuyuda) + 4 x M6 12.9 kuyu tavanının yakıt tarafındaki sızdırmaz kubbe somun plakalarına; yukarı kilit 4 x M5 kör dökme insertlere |
| burun takımı, yönlendirme | omurga yuvası (takım açık), P-SIDEBAY-L, P-SIDEBAY-R | Camloc |
| kanat birleşim pimleri, emniyetleri, kanat konnektörü | P-JOINTACCESS, P-REARACCESS | Camloc / süngü kapak |
| kanatçık / flap eyleyicileri | dış panel alt servo kapakları (kanat modülü) | M4 dişli burç |
| dümen eyleyicileri, dikey uç antenleri, kuyruk ışığı | dikey servo kapakları (iç yüz), dikey uç kapakları | M4 |
| GNSS antenleri | P-AVHATCH, P-GNSS2 | Camloc / M4 |

## 11. Yerleşim kontrolleri

`layout_check` basitleştirilmiş geometri (kutular, kapsüller, silindirler, küreler; gövde ve kanat dış yüzeyi `sizing.Airframe`'den) üzerinde 13 grup altında 88 kontrol yapar: parça kimlikleri ve numaralandırma (C01); istasyonlar, kesikler, yakıt bölmeleri ve geçişler (C02); zarfların dış yüzeyin ve iç kaplama yüzeyinin içinde kalması (C03); içerik / yapı çakışmaları, **yapı – yapı çakışmaları** (yalnız bildirilen temaslar serbest) ve **bağlantı parçalarının cıvata desenleri** (C04); mekanizma süpürmeleri, dümen kökü, taret kapağı bantları ve **montaj / bakım yolları** (C05); kütle yerleşimi ve ağırlık merkezi (C06); taret görüş alanı, RF pencereleri ve **RF görüş hattı** (C07); yasak bölgeler ve **yangın perdesinden geçen kompozit eleman yokluğu** (C08); kabuk kuralları, **panel kenar oturma yüzeyleri**, **şerit genişlikleri** ve **sabit kuyruk yüzeyleri ile sökülebilir paneller** (C09); bakım erişimi (**açık geçiş, aynı yüz, söküm prizması**) (C10); montaj ve taşıma (C11); mekanizma tanımları, açıklık kuralları ve **montaj yolları / kapak dış hatları / panel kesiklerinin açık geometrisi** (C12); bağlantıların ön boyutlandırması (C13). Kalın yazılanlar düzeltme turunda eklendi. Sonuçlar [`out/layout.md`](../out/layout.md) dosyasındadır. Testler `tests/test_ucav250_layout.py` içindedir; bunlar her kontrol grubunun kasıtlı olarak bozulmuş bir spec kopyasında hatayı yakaladığını (yeni denetimler için de: bağlantı cıvatası kenar mesafesi, yapı – yapı çakışması, oturma yüzeyi olmayan panel kenarı, sökülebilir panelden geçen sabit yüzey, kalemden küçük kapak açıklığı), `--check`'in hiçbir dosya yazmadığını, yerleşim üreticisinin `spec.yaml`'ı bayt bayt yeniden ürettiğini ve bu belgedeki istasyon / panel tablolarının spec ile aynı olduğunu da doğrular.

## 12. Boyutlandırmaya etkileri

Yerleşim evresi boyutlandırmada şu değişiklikleri getirdi (hepsi `--update-spec` ile kapanıştan geçti; `sizing --check` 61/61):

* **Kütle yerleşimi:** 14 kütle kaleminin konumu yerleşimden hesaplanıyor; tampon batarya ön bölmede. Kanat yeniden konumlandı (x_c4 kökü 2,513 → 2,511 m), en arka yüklemede statik marj %10,1'de kaldı.
* **Taret kapakları:** kapaklar V karnı izleyen kayar kapaklardır; görüş alanını (R-25, −3°) korumak için strok 0,12 m'den 0,128 m'ye çıktı. Kapakların iki DA 22 eyleyicisi taret mekanizması kalemine eklendi (1,66 → 1,786 kg).
* **Stabilatör mili:** silindir ön yüzü itki ekseniyle 5° eğik modellendi (`engine_cylinder_front_x(z)`); mil yerleştirmesine 10 µm'lik bir yuvarlama koruması eklendi.
* **Sonuç (yerleşim evresi):** boş kütle 101,34 → 101,48 kg; görev yükü 18,0 kg; dayanım 10,41 → 10,35 h, R-02b 9,64 → 9,58 h.
* **Yapı fazı:** [doc 04](04_yapi_hesaplari.md) el hesaplarıyla yeniden boyutlandırılan üyelerin kütleleri kapanışa girdi; kanat x_c4 kökü 2,511 → 2,503 m, boş kütle 101,47 kg, R-56 payı +0,097 kg.
* **Düzeltme turu 1:** kompozit birleşim, düğüm bağlantıları, sırt kanalı, perde bantları, burun mafsalı ve paraşüt bağlantılarının aşağıdan yukarı kütleleri; burun bacağı zarfı 36 mm (S1-08); kanat konumu kapanışında 0,4 mm ölü bant. Kanat x_c4 kökü 2,503 → 2,507 m, boş kütle 101,469 kg, dayanım 10,36 h, R-02b 9,59 h; grup tavanları şasi 12,38, kanat 15,46, kuyruk 7,60 kg; R-56 payı +0,120 kg (doc 04 §7).
* **Düzeltme turu 2:** ilk eklemeler (kayış kilit pimleri, mafsal bağlantısı cıvata grubu, FS3738 alt parçası, kırık bağlantıları, ayak bantları, dört burun kapağı eyleyicisi, ısı koruması) boş kütleyi 102,46 kg'a çıkardı; bütçe yeniden kapatıldı: dil başlıkları pimler arasında incelir, iki burun kapağını tek DA 22 sürer (PK2-13), yangın perdesi ve ısı koruması kütlesi aşağıdan yukarı sayılır ve kaporta kaplaması soğutma kaleminde ikinci kez sayılmaz, küçük kesitler inceltildi (doc 04 §7). Yerleşimde aviyonik güvertesinin omurga yarığı üstüne 6 katlı dolu şerit (toplanmış burun tekerine 12 mm), ön yakıt perdesi FS-FUEL kapanışta öne geldiği için görev bilgisayarı FS1810'a, döndürülen ECU ve GNSS 2 penceresi FS-FUEL'e göre konumlandı (C04, C09). Kanat x_c4 kökü 2,507 → 2,487 m, boş kütle 101,630 kg, dayanım 10,32 h, R-02b 9,54 h; R-56 payı +0,013 kg.

## 13. Sınırlamalar ve açık konular

* Motor cıvata deseni ve sönümleyici boyutları tahminidir; Limbach kurulum çizimi ve sönümleyici verisi gelince güncellenecek. Egzoz çıkış noktası da Limbach çizimine bağlıdır.
* Burun tekeri zarfı, takım birimi seçilene kadar ana tekerle aynı alındı (ihtiyatlı). Ana ve burun takımı birimleri ve EMA'ları henüz seçilmedi; mafsal burç çapları (20 H7 / 16 H7) birime göre değişebilir.
* Motor bölmesi ısı korumasının (paslanmaz ek parça ve kalkanlar, düğüm perdeleri) yeterliliği ve stabilatör düğümünün sıcaklığı motor yer çalıştırmasında ölçülmelidir (doc 04 §9, T-NODE-TEMP).
* Bağlantıların ön boyutlandırması kapalı biçimlidir; kompozit birleşimin burç ezilmesi yarı izotrop delikli bası değeriyle ihtiyatlı denetlendi ve eleman testi gerektirir. Ayrıntılı tasarımda sonlu elemanlar ve test gerekir.
* Jeneratör güç elektroniğinin ısı atımı (yaklaşık 48 W) sol yan bölme kapağındaki ısı plakasıyla atılır; ısıl doğrulama açık konudur. ECU kablo uzunluğu Limbach ile teyit edilmelidir.
* `layout_check` basitleştirilmiş zarflarla çalışır; parça geometrisiyle çakışma, süpürme ve kenar mesafesi denetimi ayrıntılı tasarım modüllerinin `checks.py` çalıştırmasıyla yapılacaktır. Montaj yolu süpürmeleri düz eksenli prizmalardır; gerçek el ve takım erişimi maket üzerinde doğrulanmalıdır.
* Kütle yerleşimindeki bileşen ağırlıkları (çerçeve ağ alanı, kapak çevresi, demet uzunluğu × kesit) tahmindir; ayrıntılı tasarımın kayıt kütlesi (`analysis/mass.py`) bunların yerini alacak.
* Taret bölmesinin kayar kapaklarında pinyon, kremayer ve raylar için 0,04 kg bir tahmindir.
* Kütle payları dardır: R-56 +0,013 kg (doc 02 §14, doc 04 §7). Ayrıntılı tasarımda eklenecek her kütle bu paydan düşülür.

## 14. Düzeltme turu 1: yerleşim bulguları

Her bulgu önce yeniden üretildi, sonra düzeltildi; hiçbir kontrol, test ya da gereksinim gevşetilmedi. Yeni denetimler bu tür hataları bir daha geçirmeyecek biçimde `layout_check`'e eklendi.

| Bulgu | Düzeltme | Denetim |
|---|---|---|
| VPK-01 stabilatör düğümü kurulamıyor | FS3738'e cıvatalanan yatak yuvası yerine yangın perdesine 7 × M5 ile bağlı işlenmiş 7075 düğüm (iki yanak, iç yatak göbeği, kök parçası 4 × M6 dikdörtgen deseni); FS3738 düğümün altında biten alt U halka; kenar uzun kirişi perdenin önünde biter; kol iç yanakla dış yanak arasında, kök parçası bağlantısı kol süpürmesinin dışında; yapıda yeni düğüm satırları (doc 04 §3.3) | C04 yapı – yapı, C04 cıvata desenleri, I-NODE |
| VPK-02 ana pimlere erişilemiyor | LERX burun kaburgaları kaldırıldı; P-JOINTACCESS gövde yanı kaburgasına kadar uzatıldı; pimler yeni yerlerinde (y 0,463 / 0,645); pim, çekici ve rayba yolları tanımlandı; raybalama eldiven kaplamalarından önce | C05 montaj yolları (23 süpürme) |
| VPK-03 teçhizat kapaklardan çıkmıyor | görev bilgisayarı ve ECU taban kesiğindeki TR-MISSION tepsisinde; ön kapak yamuk ve büyük; paraşüt kapağı 376 × 376 mm, açık geçiş 313 × 312 mm | C10 açık geçiş / aynı yüz / söküm prizması |
| VPK-04 yakıt kapakları oturmuyor | kapak kenarları ok açılı çerçevelere paralel, çerçeve başlıklarında ortak oturma; iç kenar sırt kanalı flanşında; yırtılır örtü ayrı | C09 kenar oturma yüzeyi, şerit genişliği |
| VPK-05 kuyruk yüzeyleri kaportadan geçiyor | üst kaporta orta + iki yan parça, alt kaporta ve P-STABACT sağ/sol yarımlar; sabit kök şeritleri; M-VENTRALKEEL; dümen açıklığı η 0,144–0,844; kök kesim çizgileri spec'te | C09 sabit kuyruk yüzeyleri, C05 dümen kökü |
| VPK-06 perde köşesinde üç parça çakışıyor, karbon perdeden geçiyor | tek parça köşe bağlantısı F-FW-CORNER (motor üst ayağı + dikey arka kiriş çatalı + sırt kirişi eki); karbon uzun kirişler perdenin önünde biter | C04 yapı – yapı, C08 perdeden kompozit geçmez |
| VPK-07 bağlantı zarfları cıvatalarını taşımıyor | her bağlantıda açık cıvata koordinatları; zarflar cıvatalar + kenar mesafesinden (ör. alt motor ayağı 56 × 32 mm, burun mafsalı blokları iç yüzde) | C04 cıvata desenleri (76 delik) |
| VPK-08 taret kapakları erişim kesiğine park ediyor | P-TDOORACC bandın dışına (FS1330–FS1490); halka parçası 271 × 250 mm, 14 × M4; kapak ve ray dış hatları spec'te | C05 kapak bantları ve halka sıraları |
| VPK-09 eksik arayüzler | kapak dış hatları, yakıt hatları (9), montaj yolları (15), dümen açıklığı, tekil ray kimlikleri | C12 açık geometri, C01 tekil kimlik |
| VPK-10 RF gölgesi, antenler kaplamada | uzaktan kimlik burun konisine; GNSS 1, FTS, C2A, GNSS 2 iç kaplama yüzeyinin ≥ 2 mm içinde | C03 iç kaplama, C07 görüş hattı |
| VPK-11 V çatıda piyano menteşesi | menteşesiz, bağlı, düz kalkan kapak (para_hatch prizmatik) | C05 kapak kalkışı |
| VPK-12 montaj sırası ve oturma başvuruları | rayba adım 5'te kaburgalar koridoru kapatmadan; dış yatak adım 31'de düğüm yuvasıyla aynı fikstürle; oturma başvuruları geometrik | C09 oturma başvuruları |
| VPK-13 kanat demeti yakıt bölmesinde | H-WING ön yakıt güvertesinin altından kutu alt kapağına; FS-FUEL'de demet çentiği yok | C02 geçişler |
| VPK-14 eski tablolar, `--check` dosya yazıyor | bu belgedeki tablolar spec'ten üretildi ve testle karşılaştırılıyor; `--check` salt okunur, çıktılarda süre yok | test (dosya sağlamaları, tablo değerleri) |

## 15. Düzeltme turu 2: yerleşim bulguları

İkinci bağımsız doğrulamanın paketleme bulguları (PK2-01 … PK2-13) önce yeniden üretildi, sonra düzeltildi. Hiçbir kontrol, test ya da gereksinim gevşetilmedi; denetimler bulguyu bir daha geçirmeyecek biçimde sıkılaştırıldı ve her yeni denetimin kasıtlı olarak bozulmuş bir spec kopyasında hatayı yakaladığı testle gösterildi.

| Bulgu | Düzeltme | Denetim |
|---|---|---|
| PK2-01 antenler iç kaplama kuralını aşıyor; C03 yaklaşık pay kullanıyor | C03 artık dış yüzeye **gerçek mesafe** kullanır: gövdede parametrik yüzey örnekleri üzerinde KD ağacı + yerel iyileştirme, kanat ve kuyrukta kesit düzleminde 2-B mesafe; gerekli derinlik noktanın altındaki panelin serim kalınlığıdır (`layout.shell.panels[].layup`). FTS anteni x 0,17'ye ve aşağı, C2-A x 0,25'e ve aşağı, GNSS 1/2 alçaltıldı; hepsi iç kaplama yüzeyinin ≥ 2 mm içinde. Kanat kökü ile gövde arasındaki açık basamak, eldiven kök profilinin içe doğru uzatılmasıyla kapanan bir kök filetosu olarak tanımlandı (`layout.shell.wing_root_fairing`, y 0,30 – kök; üst planform alanı 0,15 m²) | C03 (antenler, teçhizat, elemanlar, bağlantılar, eyleyiciler, demet, yakıt hatları) |
| PK2-02 kesik köşeleri çerçeve kenarını kesiyor | C02 her kesiğin **dört köşesini** OML − iç pay − 20 mm kenar bandına karşı denetler; FS0300, FS0600, FS1110, FS1330, FS1490 ve FS1810 demet / koaks kesikleri yeniden boyutlandırıldı; sırt kanalı ve paraşüt ayağı geçişleri sökülebilir örtünün altındaki kenar çentikleridir (≤ 50 mm, U takviye); C-DUCT'un üst kenarı 0,311 m'ye indi ve perde kenarı sürekli kaldı (çıkış kesiti 0,0118 m²; spec'te kesit gereksinimi yok, soğutma doğrulaması açık konu) | C02 köşe ve çentik satırları |
| PK2-03 ana bacak ve bacak kapağı takım kirişine / mafsal bağlantısına çarpıyor; C05 bunları dışlıyor | takım kirişi kuyu dış duvarı olarak y 0,385'e alındı; alt kenarında bacak hizasında takviyeli bir çentik (x₀ ± 42 mm, üst z −0,134) var, F-TRUNNION'ın 7075 flanşı çentiği köprüler (G-BEAM-NOTCH); bacak kapağı kulaklar arasında 64 mm ve y 0,30'a kadar; y 0,30 – kiriş arası şeridi, kirişin alt kenarına menteşeli **mafsal kapağı** kapatır (takım kilitli değilken açık, iç kapak eyleyicisine ikinci krankla bağlı). C05 bacağı, tekerleği, bacak kapağını, mafsal kapağını, burun kapaklarını ve yönlendirme eyleyicisini **her** statik nesneye karşı süpürür; tek muafiyet mafsal kapağının menteşe ekseninin 15 mm'si içindeki noktalarının menteşe taşıyıcısı M-GEARBEAM'e karşı olmasıdır | C05 takım dizisi (21 durum) |
| PK2-04 mafsal pimleri takılamıyor | geçme pim yerine çatalın içinden dışa takılan flanşlı muylular: ana takım Ø20 × 30 mm, burun takımı Ø16 × 22 mm; montaj yolları `main_stub_axle_*`, `nose_stub_axle_*` | C05 montaj yolları, bakım matrisi |
| PK2-05 mafsal cıvatalarına erişilemiyor, tavan cıvataları yakıt bölmesinde bitiyor | kiriş cıvataları 5 × M6 12.9, başları kuyu tarafında, kirişin dolu bandındaki yapıştırılmış flanşlı ankrajlara; tavan cıvataları 4 × M6 12.9, yakıt tarafında sızdırmaz kubbe somun plakalarına (şasi alt montajında takılır ve sızdırmazlık testi yapılır); yukarı kilit kör dökme insertlere (yakıt yüzü delinmez); bakım matrisine "somun plakası değişimi arka hücrenin sökülmesini gerektirir" notuyla girdi | C10 bakım matrisi, structures G-FIT-* |
| PK2-06 kanatçık DA 26 kaplamalar arasına sığmıyor | eyleyici y 2,077–2,180'e (içe) ve itme çubuğu tabanı 0,16 m ile öne alındı; dört-çubuk denetimi de 0,16 m ile (`wing.controls.aileron.linkage`); C03 eyleyicileri kaplama + 2 mm montaj payıyla gerçek mesafede denetler | C03 eyleyiciler, sizing R-57…R-59 |
| PK2-07 eyleyici kütle kalemleri yerleşimdeki yerlerinde değil | kanatçık, flap, dümen eyleyicileri; ana ve burun takımı (TOST teker grubu, DA-26 sınıfı EMA tahmini, bacak); burun yönlendirme (toplanmış bacakta); takım kapakları ve sürücüleri; taret mekanizması; paraşüt; yakıt sistemi `layout.mass_placement`'a girdi. C06'ya yeni kural: `mass_item` etiketi taşıyan her yerleşim nesnesinin kalemi kütle yerleşiminde olmalıdır. Kapanış yeniden yapıldı (§12) | C06 |
| PK2-08 yakıt hücresi arayüzü yalnız kodda | `layout.chassis.fuel_supports`: ön/arka sınır (istasyon + ok açısı), OML'den 25 mm iç pay, 0,4 mm astar, taban/tavan, y sınırları; `FuelBand` bunları okur; üç astar ayrı parça numarası (CH-115/116/117), iki yan bölme tepsisi ayrı (CH-118 iskele, CH-122 sancak) | C02 yakıt satırları |
| PK2-09 stabilatör düğümü ve kök parçası sıcak bölgede, denetlenmiyor | C08 kompozit kabuk panellerine, açıktaki kuyruk yüzeylerine ve sıcak bölgedeki donanıma genişletildi (`layout.heat_protection`): P-COWL-UPS 0,8 mm 6061-T6 alüminyum; alt kaportanın egzoz çıkışı çevresi (egzoz zarfının 25 mm'si içi) 0,4 mm paslanmaz 304 ek parça (HS-COWL-EXIT, 20 mm perçinli bindirme); 25–50 mm arasında alt kaportanın, kök şeridinin alt kenarının ve kök parçası alt yüzünün iç yüzünde 5 mm ayaklı 0,1 mm paslanmaz folyo kalkanlar (HS-COWL-SHIELD, HS-STUBROOT, HS-STUB; kompozit 25 mm kalkanlı payı korur); düğüm iç yatağı 61805-ZZ + yüksek sıcaklık gresi, silindir kafaları ile düğüm arasında 0,4 mm paslanmaz perde (70 × 60 mm); düğümün 7075 sıcaklık dayanımı açık konu (doc 04 T-NODE-TEMP). Yeniden kapanışta her ek parça ve kalkanın alanı ve net kütlesi yerleşimden hesaplanır (`layout.heat_protection.mass`, 0,388 kg) ve motor soğutma / yangın koruma kalemine girer | C08 üç yeni satır; test: ısı koruması kütlesi kalemde |
| PK2-10 eleman ve bağlantı zarfları iç kaplama yüzeyine göre denetlenmiyor | C03: eleman ve bağlantılar ≥ yerel kaplama kalınlığı, teçhizat ≥ max(10 mm, kaplama + 2 mm); taret tavanı kenar şeritleri, burun mafsal bloklarının ön alt köşesi, kayış bağlantıları ve F-TRUNNION flanşları kırpıldı | C03 |
| PK2-11 motor sökme yolu kısa | itki ekseni boyunca 0,25 m geri çekme + 0,35 m yukarı kaldırma, ikisi de C05'te süpürülür; bakım matrisi güncel | C05 montaj yolları |
| PK2-12 dış panel taşıma zarfı küçük | taşıma boyutu loft sınır kutusundan panel ekseninde hesaplanır: açıklık 2,904 m + dil 0,295 m, veter yönü 0,728 m, normal 0,110 m (kanat pito sondası dahil); kasa iç ölçüsü zarf + her yanda 25 mm | C11 yeni satır |
| PK2-13 takım mekanizması arayüzü eksik | burun kapağı tahriki: iki istiridye kapağını tek DA 22 (EQ-NDOORACT, sol yan bölme tepsisinde) orta hattaki bir kol, iki bağlantı çubuğu ve menteşe pimi uzantılarındaki iki kolla sürer; kolun süpürme zarfı yerleşim nesnesidir (NDOOR-LINKAGE, YK250-LG-679); ana / burun EMA zarfları (ACT-MLG-EMA, ACT-NLG-EMA) ve yönlendirme eyleyicisi (ACT-STEER, burun bacağının arka yüzünde, toplanınca bacağın üstünde) OBB zarflarıyla; kapak eyleyicisi sayısı 3 + kol ve çubuklar 0,05 kg (`mass.rules.gear_doors`); tahrik torku doc 04 G-NDOOR-DRIVE (0,899) | C05 takım dizisi (kol zarfı burun tekerine karşı), C04, C10; testler: tek tahrik, kol zarfı 30 mm öne alınınca C05 hatası |

**Yeniden kapanış.** Bu turun düzeltmeleri ve yapı eklemeleri (doc 04 §6) kapanışta kanadı ve kütle yerleşimini taşıdı; yeni konumlarda üç denetim hatası çıktı ve düzeltildi: toplanmış burun tekeri aviyonik güvertesine 11,3 mm (< 12 mm) yaklaşıyordu (M-DECK-NOSE omurga yarığının üstünde 6 katlı dolu şerit, C05), öne gelen ön yakıt perdesiyle görev bilgisayarı ve ECU çakışıyordu (görev bilgisayarı FS1810'a, ECU döndürülerek FS-FUEL'e göre; C04), GNSS 2 penceresi ile ön yakıt kapağı arasındaki sabit şerit 57 mm'ye (< 58 mm) iniyordu (pencere FS-FUEL'e göre; C09). Burun tekeri – tam güverte durumu için kasıtlı olarak bozulmuş bir spec kopyasında hatanın yakalandığını gösteren yeni bir test vardır; diğer ikisi C04 ve C09'un mevcut testleriyle kapsanır. Kütle kapanışı doc 04 §7'dedir: boş kütle 101,630 kg, R-02b 9,54 h, R-56 +0,013 kg.
