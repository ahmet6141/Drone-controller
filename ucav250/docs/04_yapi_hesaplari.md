# YK-250 HANÇER — Yapı hesapları (faz 3, yapı evresi ve düzeltme turu 1)

**Tarih:** 9 Ekim 2026 · **Durum:** `python3 -m ucav250.analysis.structures --check` 244 satırın 227 emniyet payının (MS) hepsini sıfır ya da pozitif (en küçük 0,002, W-CAP-04), `spec.yaml → structures.sizing` bölümünü güncel ve 15 yerleşim arayüzünün 15'ini tutarlı buluyor. Boyut değişiklikleri `spec.yaml`'a yazıldı; `sizing --check` (61/61), `layout_build.build --check` (bayt bayt aynı) ve `layout_check --check` (77/77) yeşil.

> **Kapsam.** YK-250 HANÇER sivil bir EO/IR gözetleme ve araştırma İHA'sıdır. Hesaplarda dış yük taşıma bağlantısı,
> askı ya da yük bırakma düzeneği yoktur; "bağlantı parçası" (fitting) sözcüğü yalnızca yapı içi bağlantıları
> (kanat birleşimi, iniş takımı mafsalı, motor bağlantısı, paraşüt kayışı, kuyruk düğümleri) anlatır.

Bu belge HANÇER'in birincil yapısı için yapılan el hesaplarını anlatır: hangi yük durumlarının alındığı, her elemanın hangi yöntemle ve hangi izin verilen değerle denetlendiği, hangi elemanların yetmediği ve nasıl yeniden boyutlandırıldığı, bunun kütleye ve kütle bütçesine etkisi ve açık kalan konular. §5, bağımsız doğrulamanın yapı bulgularına (S1-01 … S1-10) karşı yapılan **düzeltme turu 1**'i özetler. Satır satır emniyet payı tablosu [`out/structures.md`](../out/structures.md) dosyasındadır (makinece okunur sürümü [`out/structures.json`](../out/structures.json)); bu belge yöntemi ve sonuçları özetler.

```
python3 -m ucav250.analysis.structures                # hesap; out/structures.md ve out/structures.json yazılır
python3 -m ucav250.analysis.structures --update-spec  # boyutlandırılan ölçüler spec.structures.sizing'e, katman dizilimleri spec.layups'a
python3 -m ucav250.analysis.structures --check        # MS >= 0, spec bloğu güncel, yerleşim arayüzleri tutarlı (çıkış 1 = hata); hiçbir dosya yazmaz
```

Bir değişiklikten sonra sıra şudur: `layout_build.build --write` → `structures --update-spec` → `sizing --update-spec` (kütle kalemleri, kapanış) → yeniden `layout_build.build --write` (kanat konumu ve kütle yerleşimi; `sizing` mekanizma değerlerini 6 anlamlı basamakla sakladığı için son yazan üretici olmalıdır) → `build --check`, `structures --check`, `sizing --check`, `layout_check --check`. Kanat konumu kapanışta değiştiğinde bu döngü sabit noktaya gelene kadar tekrarlanır. Düzeltme turunda yerleşim ile boyutlandırma arasında 0,1 mm'lik yuvarlamanın yol açtığı iki adımlı bir salınım görüldü; boyutlandırma kapanışı artık kanadı yalnızca |Δx_c4| ≥ 0,4 mm olduğunda taşır (`sizing.WING_DEADBAND`, kanat AM'sinin 0,4 mm içinde kalması statik payı ölçülebilir biçimde değiştirmez). Bu turda kanat kök çeyrek veter noktası 2,503 m'den 2,507 m'ye (4,2 mm geri) geldi (§6).

---

## 1. Yöntem

### 1.1 El hesapları ve tek kaynak

Her kontrol kapalı biçimli bir el hesabıdır ve [`analysis/structlib.py`](../analysis/structlib.py) içindeki yardımcılarla yapılır: klasik lamine teorisi (ABD matrisleri, ilk katman hasarı: en büyük gerilme ve Tsai-Wu), özel ortotrop levha basma ve uzun levha kesme burkulması (Kollár & Springer), sandviç kesme düzeltmesi, yüz buruşması (Zenkert, Q = 0,5), kesme kıvrılması (N = G_c d²/c), Johnson-Euler kolonu, boru burulması, Bredt kesme akışı, elastik cıvata grubu (düzlem içi ve çekme), pim eğilmesi (Melcon-Hoblit kolu), kulak (lug) dayanımı (Bruhn D1, gerçek e/D'de kesme-ezilme), doğrudan rijitlik yöntemiyle uzay kafes, enerji yöntemiyle iniş yük katsayısı ve düzeltme turunda eklenen basit mesnetli dikdörtgen levhanın merkez yamasındaki moment (Timoshenko, levhalar §19; yangın perdesi ayakları). Bu yardımcıların her biri [`tests/test_ucav250_structures.py`](../../tests/test_ucav250_structures.py) içinde kapalı biçimli bir değerle karşılaştırılarak test edilir.

Geometri, kütle ve malzeme tek kaynaktan gelir: kanat kesitleri ve kiriş çizgileri `wing`, çerçeveler, elemanlar, bağlantı parçaları, cıvata desenleri ve mekanizmalar `layout`, kütle kalemleri ve yükleme durumları `mass`, malzemeler `materials`, katman dizilimleri `layups`, katsayılar ve türetilmiş yükler `structures` (boyutlandırma evresinin hamle matrisi dahil). Yapı evresinin yazdığı ölçüler `structures.sizing` bölümündedir.

### 1.2 Emniyet payı ve katsayılar

**MS = izin verilen / (uygulanan × toplam katsayı) − 1.** Uygulanan değer yük durumunun LİMİT yüküdür (ya da limit gerilme / birim şekil değiştirme); izin verilen değer NİHAİ tasarım değeridir. Yalnız nihai durumlarda (acil iniş, paraşüt açılması, çarpma tutma, iniş yedek enerjisi, yakıt bölmesi tasarım basıncı) uygulanan değer zaten nihaidir ve emniyet katsayısı yoktur.

| Katsayı | Değer | Kaynak |
|---|---|---|
| Emniyet katsayısı | 1,5 | CS-LUAS.303, STANAG 4703 UL2.3 |
| Bağlantı parçası katsayısı | 1,15 | CS-LUAS.625 |
| Mafsallı / dönen bağlantı ezilmesi | 2,0 | STANAG 4703 UL2.4 |
| Sık sökülüp takılan bağlantı (dış kanat paneli pimleri ve burçları, stabilatör) | 1,5 | STANAG 4703 UL2.4 |
| Kompozit özel katsayısı (B-tabanı ETW değerleriyle) | 1,2 | AMC LUAS.619 |
| Tek yük yollu kompozit A-tabanı tahmini (dil UD flanşları dahil) | 0,85 × B-tabanı | CS-LUAS.613(b) |
| Kumanda menteşesi ezilmesi (toplam) | 6,67 | CS-LUAS.657 |
| İtme-çekme bağlantısı (toplam) | 3,33 | CS-LUAS.693 |

Birleştirme kuralı CS-LUAS.619'a göredir: toplam katsayı = 1,5 × uygulanabilir özel katsayıların **en büyüğü** (çarpılmazlar). Kompozit katsayısı 1,2 bu özel katsayılardan biridir (sıcak/nem etkisi ETW değerlerinin içindedir). Hasar toleransı birim şekil değiştirme sınırları (STANAG 4703 UL13.1.2, [`data/research/standards.yaml`](../data/research/standards.yaml)) nihai yükte uygulanır: 2 mm'den kalın lamine bası 3000 µε, sandviç kaplama ve ≤ 2 mm lamine bası 2600 µε, çekme 5000 µε, kesme 5200 µε.

İzin verilen değerler: MTM45-1/AS4 UD ve PW katları B-tabanı ETW (NCAMP), yarı izotrop delikli bası/ezilme değerleri ([`materials.yaml`](../data/research/materials.yaml) `design_values_for_code`), metal parçalar MMPDS (A-tabanı), cıvatalar ISO 898-1 / ISO 3506-1, ROHACELL çekirdekler üretici minimum değerleri. **UD bant için ezilme değeri yoktur** (düzeltme turunda `cfrp_ud_mtm45_as4`'ten kaldırıldı, S1-05): UD'ye bağlantı elemanı açılmaz, delikler her zaman ≥ %40 ±45 içeren bloklardadır. Burçlu kompozit deliklerin ezilmesi, delik kenar mesafesi %2 kaymalı ezilme değerinin koşulunu (e/D ≥ 3) sağlamadığı için **yarı izotrop delikli bası dayanımıyla (175,4 MPa)** ihtiyatlı biçimde denetlenir; bu bir yerine koymadır ve eleman testiyle doğrulanacaktır (§8).

---

## 2. Yük durumları

### 2.1 Uçuş zarfı ve kanat yükleri

| Büyüklük | Değer | Not |
|---|---|---|
| V_S (temiz, MTOM) / V_A / V_C / V_D (EAS) | 24,0 / 45,0 / 45,0 / 57,0 m/s | `structures.derived` |
| Manevra limit yük katsayıları | +3,8 / −1,52 | CS-LUAS.337 |
| Kanat tasarım yük katsayısı (MTOM'da) | +5,47 | hamle matrisinin en büyük n·m değeri (6 kütle × irtifa) |
| Negatif kanat durumu (MTOM'a eşdeğer) | −3,55 | en büyük \|n·m\| / MTOM |
| Teçhizat yük katsayısı (en hafif durum 106,3 kg) | +7,01 / −5,01 | hamle; bağlantı ve teçhizat tutma için |

Kanat yükü referans yamuk üzerinde Schrenk dağılımıdır; kanat yapısı, kumanda yüzeyleri ve kanatçık / flap eyleyicileri aynı yük katsayısıyla **atalet rahatlatması** olarak düşülür. Düzeltme turunda (S1-01) her kütle terimi fiziksel olarak bulunduğu yere konur: ana kiriş başlıkları kat sayısı programına göre (gövde içindeki y < 0,40 kutu başlıkları dış kanatta rahatlatma vermez), dış panel kiriş gövdeleri gövde kat bölgelerine ve vetere göre, arka kiriş gövde yanı kaburgasından uca, kaburgalar veterin karesiyle, birleşim donanımı (kompozit dil, başlık kalınlaşması, arka kulak) dilin ucundan başlık kalınlaşmasının sonuna kadar, kaplamalar ve kumanda yüzeyleri gerçek kanat vetere göre, eyleyiciler yerleşimdeki konumlarında nokta kütle olarak. Yarım kanat 10,0 kg'dır. Kök eğilme momenti (limit) **5293 N m**'dir (rahatlatmasız 6106 N m); birleşim düzleminde (y 0,70) **V 2562 N, M 3195 N m, T 104 N m**. Burulma zarfı üç durumun büyüğüdür: V_D'de kesit c_m0'ı (NLF(1)-0416, −0,102), V_A'da tam kanatçık (CS-LUAS.349(b): Δc_m = −0,01/derece) ve V_F'de 40° flap.

**Veter yönü durumları (S1-02).** Dış panel birleşimi düzlem içi (sürükleme) momentini de taşır. Üç durum tanımlıdır: PHAA (pozitif yüksek hücum açısı: kanat CL_maks'ta kesit açısı 11,9°, veter yönünde öne doğru C = L tan α = 541 N, birleşimde Mz = 675 N m), NHAA (negatif yüksek hücum açısı, kesit c_l,min: Mz 385 N m) ve VD-DRAG (V_D'de sürükleme ağır basan durum: Mz −16 N m). Mz, ana pimler ile arka pim arasındaki 0,294 m veter yönü kol üzerinden açıklık yönünde bir kuvvet çiftidir (F_y = Mz / Δx; ana pimler F_y/2'şer, arka pim F_y).

**Yuvarlanma (CS-LUAS.349(a)):** bir tarafta n_A'da %100, diğerinde 2/3 yük ve tam kanatçık burulması. Daha yüklü tarafın kök momenti 3678 N m'dir, tasarım durumunun (5293 N m) altındadır (W-ROLL bilgi satırı).

### 2.2 Kuyruk ve kumandalar

* **Stabilatör:** V_A'da tam sapma ve kesit CN_maks (panel başına 481 N limit, basınç merkezi y 0,684), V_C'de CS-VLA 425(d) hamlesi (261 N, en büyük hamle azaltma katsayısı 0,78), simetrik olmayan durum %100 / %72 (CS-LUAS.427(b)). Mil kesitinde moment 175 N m; iç yatak tepkisi 999 N, dış yatak tepkisi 1480 N (limit).
* **Dikeyler:** 15° yanal kayma × 1,5 aşma, V_A'da tam dümen ve V_C'de yanal hamle; her biri kesit CN_maks ile sınırlı (dikey başına 587 N, kökte 214 N m).
* **Kumanda yüzeyleri:** yüzey yükü CS-VLA Ek B ortalama yüklemesi ile eyleyicinin uygulayabildiği menteşe momentinin büyüğüdür; bu uçakta her yüzeyde eyleyici belirleyicidir (kanatçık 389 N, flap 836 N, dümen 326 N). Menteşe tepkileri sürekli kiriş katsayılarıyla; menteşe ekseni boyunca atalet 24 g / 12 g (CS-LUAS.393(b)); itme çubukları eyleyici durma torkuyla (CS-LUAS.395); yer rüzgârı (CS-VLA 415).
* **Kuyruk tamponu:** skidden 45° yukarı-geri 1,0 × MTOM ağırlığı (1470 N; tasarım kararı, burun tekerli uçak tamponu için kural yok).

### 2.3 Yer yükleri ve iniş takımı

Çökme hızı 2,35 m/s (CS-LUAS H.2(b)), serbest düşme yüksekliği 0,283 m, kullanılabilir yol 92 mm, verim 0,63, kanat taşıması 2/3 W → **n_j = 5,40**, atalet yük katsayısı 6,07; ana bacak başına düşey limit yük 3970 N. Ana takım durumları: düz iniş (dönme başlangıcı, geri yaylanma, en büyük düşey; AMC VLA 479(b)), kuyruk aşağı iniş (9,2°), tek teker inişi, yan yük (içe 0,5 W, dışa 0,33 W), frenli yuvarlanma (0,8 sürtünme). Burun takımı: üç noktalı düz iniş ve ek durumlar (arka 2,25/1,8, ön 3,2/0,9, yan 2,25/1,575 × statik). İniş yedek enerjisi 1,2 V_çökme'de n_j 7,0 (nihai). Takım işletme hızı tasarım kararı olarak 1,6 V_S = 38,3 m/s EAS. Toplanmış ana bacağın (4,20 kg) yukarı kilide yükü teçhizat yük katsayısı 7,01 ile hesaplanır (S1-06).

### 2.4 Motor bağlantısı

Motor grubu 10,10 kg (büyüme payı dahil), I_p 0,0227 kg m². Durumlar: MCP torku × 6 + n_A + itki; kalkış torku + 0,75 n_A + itki; en hafif durumun hamle yük katsayısı (+7,01) + MCP torku; negatif yük katsayısı; yan yük 1,47 g; jiroskopik (sapma 2,5, yunuslama 1,0 rad/s); iniş n_j + 0,67; çarpma 15 g ileri, acil iniş 6 g aşağı ve paraşüt açılması (nihai). Dört eşit sönümleyici üzerinde rijit cisim dağılımı; 4130 kaynaklı kafes doğrudan rijitlik yöntemiyle çözülür. Ayak tepkileri yangın perdesine düzlem içi ve **düzlem dışı** olarak verilir: alt ayakta en büyük düzlem dışı tepki 2207 N, üst ayakta 1785 N (nihai, hamle n 7,01 + MCP torku).

### 2.5 Paraşüt, yakıt, taret, teçhizat, taşıma

* **Paraşüt:** açılma şoku 13,1 kN tek kayış ayağında (yalnız nihai, × 1,15 bağlantı). Yönler: statik ayak yönü etrafında 30° koni (birleşme halkası AM'nin 1,5 m üstü) ve düşeyden 60° geriye kadar açılma silkmesi. Kayışın uçak eksenindeki (x) bileşeni en çok **11,36 kN**'dur (S1-04); en hafif kütlede yük katsayısı 12,6'dır.
* **Yakıt bölmeleri:** tasarım basıncı 21,0 kPa nihai = 1,5 × CS-LUAS.965(b) test basıncı 14 kPa; yakıt sütunu, acil iniş 9 g ileri ve 6 g aşağı ve paraşüt açılması karşılaştırılır.
* **Taret asansörü:** E180 büyüme tareti + taşıyıcı 4,35 kg; nihai yük katsayıları aşağı 12,6, ileri 10,9, yan 2,2. Kayar kapaklar V_D emmesinde (|Cp| = 1 üst sınır).
* **Teçhizat tutma:** nihai aşağı 12,6, yukarı 7,5, ileri 10,9, yan 2,2.
* **Taşıma:** kızakta orta gövde (81,5 kg); 3,0 g düşey, 1,5 g ileri-geri, 1,5 g yan limit (tasarım kararı).

### 2.6 Çerçevelere giren yoğun yükler (S1-06)

| Yük | Değer (limit, aksi belirtilmedikçe) | Kaynak durum |
|---|---|---|
| FS-GEAR çerçevesine takım kirişi tepkisi (yan başına) | 2171 N | düz iniş, takım kirişinin iki çerçeveye dağılımı |
| FS-RS çerçevesine takım kirişi tepkisi (yan başına) | 1799 N | aynı |
| FS-MS / FS-RS: gövde ataleti, kutu uçlarında | 5113 / 1852 N | kanat tasarım durumu, gövde kütlesi 129,9 kg |
| Yangın perdesi alt ayağı, düzlem dışı | 2207 N (nihai) | hamle n 7,01 + MCP torku |
| Ana yukarı kilit kancası | 434 N (nihai) | toplanmış bacak, n 7,01 |
| FS3738'de arka omurga desteği | 10,25 kN | kuyruk tamponu çarpması (yalnız bilgi satırı, §8) |

---

## 3. Elemanlar ve sonuçlar

227 emniyet payı ve 17 gereksinim / bilgi satırı (toplam 244) hesaplanır; negatif pay yoktur, en küçük MS 0,002'dir (kiriş başlıkları kat sayısı bölgelerinde boyutlandırıldığı için sıfıra yakındır). Gruplara göre belirleyici satırlar:

| Grup (satır) | Belirleyici kontrol | MS |
|---|---|---|
| Kanat (68) | ana kiriş başlığı bölgeleri: kaplama yüzü bası birim şekil değiştirmesi 2600 µε (W-CAP-04, y 0,70–0,80) | 0,002 |
| Dış panel birleşimi (20) | dış pimde dilin UD başlığı, bası birim şekil değiştirmesi 3000 µε (J-TONGUE-FLANGE-DT) | 0,139 |
| Orta kanat kutusu (11) | FS-MS gövdesi kesme kıvrılması | 0,229 |
| Kuyruk (30) | arka omurga kirişi, tampon yükü (T-AFTKEEL) | 0,123 |
| Kumanda yüzeyleri (21) | flap itme çubuğu kolonu | 0,709 |
| İniş takımı (20) | ana yukarı kilit bağlantısı, insert sökülmesi (G-UPLOCK) | 0,351 |
| Motor bağlantısı (8) | kafes çubuğu kolonu (Johnson-Euler) | 0,923 |
| Paraşüt (14) | bağlantı – çerçeve cıvataları 2 × M5 (P-FRAME-BOLTS) | 0,380 |
| Yakıt bölmeleri (12) | FS-FUEL perdesi çekirdek kesmesi | 0,198 |
| Taret (6) | kayar kapak tahriki | 8,01 |
| Taşıma (3) | kızak yastığı altında çekirdek ezilmesi | 0,591 |
| Gövde (10) | görev bölmesi taban şeridi burkulması (B-MIDFLOOR-BUCK) | 0,265 |
| Çerçeveler (14) | alt motor ayağında yangın perdesi çekirdek kesmesi (FW-FOOT-CORE) | 0,002 |
| Teçhizat (7) | faydalı yük tepsisi vidaları | 12,5 |

### 3.1 Kanat kesiti

Kesit modeli düzlem kesitler varsayar: ana ve arka kiriş UD başlıkları, başlıkların üstünde dolu kaplama (1 mm), kirişler arasındaki kutu kaplamalarının dış ve iç yüzleri; D-burnu ve firar kenarı kaplamaları sayılmaz (başlık birim şekil değiştirmesi için muhafazakâr). Gövde içinde (y < 0,40) kaplama yerine orta kutu kapakları alınır. Her başlık bölgesi için o bölgenin altı istasyonunda bütün ölçütleri sağlayan en küçük kat sayısı aranır: başlık bası/çekme birim şekil değiştirmesi (3000 / 5000 µε), A-tabanı tahmini 0,85 Fcu / 0,85 Ftu ile gerilme, kaplama yüzü bası birim şekil değiştirmesi (2600 µε), delikli bası (OHC) ve yüz buruşması. Belirleyici ölçüt kaplamanın 2600 µε sınırıdır. Başlıklar saf UD'dir: hiçbir ana kiriş başlığından bağlantı elemanı geçmez (S1-05).

**Eldiven bölgesi (y 0,40–0,70).** Kompozit dil, birleşim momentini pimlere kadar kendisi taşır: dilin momenti dış pimin dışında birleşim momenti + kesme × kol, pimler arasında iç pime doğru doğrusal olarak sıfıra iner. Eldiven kutusunun başlıkları bu yüzden toplam momentten dil momentinin çıkarılmışıyla boyutlandırılır (`structures.sizing.wing.main_cap.fork_zone`): y 0,55'te toplam 3592 N m'nin 1998 N m'si eldiven kutusundadır ve bölge 12 kata iner; iç pimin içinde (y 0,40–0,55) bütün moment kutudadır (33 kat). Eldiven ana kiriş gövdeleri aynı zamanda çatal kulaklarıdır (2 × 10 kat ±45; W-GLOVEWEB MS 7,74).

Kiriş gövdeleri (±45 PW) dış panelde ilk katman hasarı, 5200 µε kesme birim şekil değiştirmesi ve uzun levha kesme burkulmasıyla boyutlandırılır. Kutu kaplama panelleri bası + kesme etkileşimiyle (Rc + Rs² = 1, CLT D*, sandviç kesme düzeltmesi) ve kesme kıvrılmasıyla denetlenir (dış panel üst kutu kaplaması MS 0,034, orta kutu kapağı 0,035); kaburgalar Brazier ezilme yükünde sandviç kolon olarak.

### 3.2 Dış panel birleşimi (y 0,70): pimli kompozit dil-çatal

Düzeltme turunda yapıştırmalı metal dil-çatal bırakıldı (§5, S1-03) ve yerine planör uygulamasındaki pimli kompozit birleşim kondu (`structures.sizing.wing_joint`, `layout.chassis.wing_joint`):

* **Dil (YK250-WG-151, dış panel):** CFRP kiriş kökü 30 × 61 mm, 0,292 m kavrama; UD flanşlar 30 × 10 mm (dış panel ana kiriş başlıklarının devamı, kök kaburgasından 0,10 m boyunca 1:20 kalınlaşır), ±45 PW gövde 5 mm (25 kat), iki pim deliğinin çevresinde 30 mm genişlik × 50 mm boyunda dolu [±45/0/90] bloklar, yapıştırılmış 4130 burçlar 16 H8 × dış çap 22 × 30 mm.
* **Çatal (YK250-CH-053, orta kesit):** eldivenin ana kiriş kutusu; 40 mm UD başlıklar (çatal boyunca 52 mm'ye genişler), 30,4 mm aralıklı iki ±45 PW gövde (kulaklar, 2 mm, 10 kat), pim bölgelerinde 10 mm'ye kalınlaştırılmış [±45/0/90] bloklar, yapıştırılmış 4130 burçlar 16 H8 × dış çap 22 × 10 mm.
* **Ana pimler:** 2 × Ø16 Ti-6Al-4V, kiriş boyunca 184 mm aralıklı (P-MAIN1 y 0,463, P-MAIN2 y 0,645). Düşey kuvvet çifti R_iç 18,31 kN, R_dış 20,88 kN; burulma çifti F_T 354 N (limit, PHAA).
* **Arka kiriş:** dış panelin arka kiriş kök bağlantısındaki 7075 kulak (t 8 mm, genişlik 32 mm, e 16 mm = 2 D), birleşim kaburgasındaki yuva bağlantısının (YK250-CH-054) iki 4 mm levhası arasına girer ve düşey Ø8 Ti bilyalı kilit pimiyle tutulur. Kulak veter yönü kuvveti ve düzlem içi momentin açıklık yönü kuvvet çiftini taşır.

PHAA durumunda Mz 675 N m → F_y 2296 N; ana pimin bileşke yükü P_dış = 21,26 kN, arka pim 2,36 kN. Sonuçlar (MS): pim kesmesi 4,04, **pim eğilmesi 0,22** (Melcon-Hoblit kolu, plastik kazanç yok, × 1,5 × 1,5), pim–burç ezilmesi 5,92, dil burcu ezilmesi 0,82, **çatal kulağı burcu ezilmesi 0,21**, dil UD başlığı çekme / bası birim şekil değiştirmesi / A-tabanı gerilme 1,84 / **0,139** / 0,63, dil gövdesi kesmesi 0,16, çatal kulakları kesmesi 0,21; arka pim kesme / eğilme 10,4 / 3,27, arka kulak 13,7, kulak ezilmesi 8,04, yuva levhası 2,09, yuva bağlantısının kaburga ve gövde cıvataları ≥ 2,19.

### 3.3 Kuyruk: stabilatör düğüm bağlantısı (VPK-01 ile birlikte)

Yerleşim evresinin FS3738 halka çerçevesine cıvatalanan iç yatak yuvası kurulamıyordu (30 mm'lik halka bandı 25 mm'lik mil deliğini ve yuva cıvatalarını taşıyamaz). Yerine her yanda bir **işlenmiş 7075-T651 düğüm bağlantısı (F-SPINDLE-NODE, YK250-CH-095)** kondu: yangın perdesinin arka yüzünden 92 mm geriye konsol, 8 mm U biçimli taban flanşı 7 × M5 12.9 ile perde yığınından geçer (önde 7075 destek levhası); iç yanak 28 mm, iç yatak göbeği (61805-2RS, Ø37) burada; dış yanak 10 mm, kök parçası arka kiriş kök bağlantısını 4 × M6 12.9 ile (76 × 62 mm dikdörtgen, çekme + kesme) taşır ve kol süpürmesinin dışında kalır. FS3738 artık düğümün altında biten bir alt U halkadır.

* **Mil:** Ti-6Al-4V 25 × 1,5 mm (MS 0,79; kama kesiti 2,18). **Kök yuvası:** 7075 kovan 31 × 3 mm, 0,10 m (ezilme 79, boru 1,41, dış çap 16,4).
* **Düğüm:** kök parçası cıvataları 9,54, kök cıvataları ezilmesi 93, dış yanak burulması (b t³/3; kök momenti yanaktan tabana) 2,42, iç yanak 8,09, iç yatak göbeği halkası 0,53, taban cıvata grubu (düzlem içi moment + çekme) 3,94, perde ezilmesi 2,07, **perde sandviçi çekirdek kesmesi 0,22**.
* **Yataklar (tedarik gereksinimi):** iç 1,50 kN, dış 2,22 kN nihai statik yük sayısı (T-BRG-IN / T-BRG-OUT).

### 3.4 Paraşüt kayışı: sırt omurga kanalı (S1-04)

Kayışın x bileşeni çerçevelere düzlem dışı yük veriyordu ve onu taşıyan boyuna eleman yoktu. Yeni yük yolu: iki U-kulak bağlantısı (F-RISER-FWD / F-RISER-AFT) FS1810 ile arka kiriş çerçevesi arasındaki **sırt omurga kanalının (M-SPINE)** iki ucundadır. Kanal 4 kat PW (0,8 mm), U 44 × 24 mm, 60 mm flanşlı; iki ayağın karşıt x bileşenleri kanalda çubuk kuvvetidir (kayış üçgenini kapatır), net x bileşeni kanal flanşlarına 25 mm aralıkla vidalanan sabit görev bölmesi üst kaplamasıyla (P-MB-UPPER) kesme olarak kenar uzun kirişlerine geçer. x bileşeni 4 × M5 12.9 (eksen z) ile kanalın 16 katlı taban takviyesine, z bileşeni 2 × M5 12.9 (eksen x) ile çerçevenin 32 katlı (6,4 mm) dolu bandına girer. Kilit pimi Ø6 4130, kulaklar 2 × 6 mm, e 12 mm = 2 D.

Sonuçlar (MS): kilit pimi 0,475, kulak ezilmesi 1,75, kulak 5,45, **çerçeve cıvataları 0,38**, çerçeve bandı ezilmesi 0,70, çerçeve gövdesi 0,85, kanal cıvataları 1,51, kanal takviyesi ezilmesi 0,96, kanal eksenel 1,18, kolon 2,26, yerel 1,17, kaplama vidaları 5,78 / ezilme 4,86, kaplama kesmesi 4,69.

### 3.5 Çerçeveler ve yangın perdesi (S1-06)

* **FS-GEAR ve FS-RS:** takım kirişi tepkisinin girdiği yerde çerçeve gövdesi kesmesi, kesme kıvrılması ve bağlantı ezilmesi (FS-GEAR 1,71 / 1,67 / 4,13; FS-RS 2,27 / 2,23 / 5,19), takım yan yükünün kuyu tavanına girdiği 4 × M6 cıvatada bant ezilmesi 23,2.
* **FS-MS / FS-RS halkaları:** kutu dışında, gövde yanı bağlantısında gövde ataleti kesmesi (1,56 / 2,08; FS-MS'de yüz başına bir ±45 kat).
* **Yangın perdesi alt ayakları:** 2207 N düzlem dışı tepki 6,8 mm'lik sandviçte taşınamıyordu. Ayakların çevresine ROHACELL 71 WF çekirdek parçası ve 24 katlı (4,8 mm), r 66 mm dolu bant kondu; ayak destek levhası 56 × 32 × 5 mm. Merkez yamalı basit mesnetli levha momentiyle (Timoshenko) bant eğilmesi 0,40, yüzler 0,215, **çekirdek kesmesi 0,002**; ayak cıvatalarının bantta ezilmesi 1,48 (E-FOOT-BR). Üst ayaklar köşe bağlantısıyla (F-FW-CORNER) sırt uzun kirişi ekine bağlanır (FW-UPPER-SPLICE 2,88).
* **Ana yukarı kilit:** kanca bağlantısı kuyu tavanının dökme insertlerine 4 × M5; dökme yarıçapı 12 mm, kanca kaçıklığı 6 mm ile insert sökülmesi MS 0,35 (ilk tasarımda −0,18).
* **FS3738'de omurga desteği:** kuyruk tamponu çarpmasında arka omurga perde ucunda mafsallı ise FS3738 10,25 kN düşey tepki taşımalıdır; 2,0 mm 2024-T3 alt U halkası buna göre **boyutlandırılmadı** (FR-3738-KEEL bilgi satırı, açık konu §8).

### 3.6 Diğer elemanlar

* **Orta kutu:** gövdeler = FS-MS / FS-RS çerçeveleri, kutu-çerçeve bağlantıları, çevron kırığı (CT-KINK: başlık başına 17,0 kN veter yönü kuvvet; kırık bağlantısının ve orta hat kaburgasının detay tasarımına yük olarak verilir).
* **Kuyruk:** dikey kirişleri ve kök bağlantıları (köşe bağlantısının çatal kulağı), tail_skin kaplamaları, ventral kök kulakları ve arka omurga kirişi (7075 talaşlı U 24 × 40 × 2,5 mm, kaporta yarımları için alt oturma flanşlarıyla).
* **İniş takımı:** ana bacak (7075 boru 42 × 3,5 mm), mafsal kulağı (e/D 1,14 < 2: kulak istisnası, Bruhn kesme-ezilme gerçek e/D'de, MS 2,11; S1-09), mafsal muylusu, burun bacağı (dış çap 36 mm; takım zarfı genişliği 36 mm'ye çıkarıldı, S1-08), burun mafsalı (iç yüzdeki 7075 bloklar, 16 H7 flanşlı burç dış çapı 22, kesme-ezilme 11,7), EMA gereksinimleri, iç kapak menteşeleri.
* **Motor bağlantısı:** kafes çubukları (12,7 × 0,89 mm 4130), sönümleyici halkası, karter cıvataları, yangın perdesi ayakları, sönümleyici nihai kapasite gereksinimi (1,80 kN).
* **Gövde:** görev bölmesi tabanının ortası sökülebilir ekipman tepsisine (TR-MISSION) açılan çerçeveli bir kesiktir; iki dış şerit ön gövdenin alt başlığıdır ve kesik kenarlarındaki iki yapıştırılmış şapka takviyeyle (PW 4 kat, 20 × 15 mm) burkulmaya karşı desteklenir (B-MIDFLOOR 0,93, burkulma 0,265). Arka gövdede sırt ve kenar uzun kirişleri, sabit kaplamalar ve vida sıraları.
* **Yakıt bölmeleri:** FS-FUEL perdesi, ön hücre tabanı, kuyu tavanı, raylar ve kuyu omurga gövdeleri.

---

## 4. Yapı evresinde yetmeyen elemanlar ve yeniden boyutlandırma

İlk hesap turunda aşağıdaki elemanların payı negatifti. Her biri yeniden boyutlandırıldı ve yeni ölçü `spec.yaml`'a yazıldı (`structures.sizing`, gerekiyorsa `layups` ve `layout.chassis`).

### 4.1 Kanat kaplamaları: kesme kıvrılması (en önemli bulgu)

Boyutlandırma evresinin birincil kanat kaplaması `0/90, ±45, 0/90 / 5 mm ROHACELL 51 WF / ±45, 0/90` dizilimindeydi. Kanat başlıkları kaplama yüzü birim şekil değiştirme sınırına göre boyutlandırıldığında, 0/90 katları ağır basan yüzler limit yükte yaklaşık 83 N/mm bası taşır; 5 mm 51 WF çekirdeğin kesme kıvrılma yükü ise 85 N/mm'dir. Nihai yükte kaplama hem kıvrılır (MS −0,43) hem de kaburgalar arasında burkulur (MS −0,57).

Çözüm, planör kanatlarındaki bilinen düzendir: **kaplamanın açıklık yönü modülünü düşürmek ve eğilmeyi kiriş başlıklarına bırakmak.**

* `layups.wing_skin_primary`: dış yüz `±45, 0/90, ±45`, iç yüz `±45, ±45` (aynı kat sayısı, kalınlık ve kütle). Kaplama limit yükte yaklaşık 42 N/mm taşır, kıvrılma payı 0,12–0,30'dur ve burulma rijitliği artar.
* `layups.wing_box_skin_upper`: kirişler arasındaki üst kaplama y 2,20'ye kadar 6 mm çekirdek (0,6/6/0,4 mm). Dış yüz değişmez; iç yüz başlık kenarlarında 1 mm basamak yapar. Ek kütle 45 g (iki kanat).
* Kaplama bükülmeye daha az katıldığı için başlıklar kalınlaştı; bölgeler 0,10 m'ye inceltildi, kat bırakma eğimi 1:20. Arka kiriş başlıkları 2 kattır.

Ana kiriş başlıkları (UD MTM45-1/AS4, kat 0,1397 mm; gövde ve eldivende 40 mm, dış panelde 30 mm genişlik; düzeltme turu 1 sonrası):

| y (m) | 0–0,40 | 0,40–0,55 | 0,55–0,70 | 0,70–0,80 | 0,80–0,90 | 0,90–1,00 | 1,00–1,10 | 1,10–1,20 | 1,20–1,30 | 1,30–1,40 |
|---|---|---|---|---|---|---|---|---|---|---|
| kat | 57 | 33 | 12 | 35 | 34 | 32 | 30 | 27 | 25 | 23 |

| y (m) | 1,40–1,50 | 1,50–1,60 | 1,60–1,70 | 1,70–1,80 | 1,80–1,90 | 1,90–2,00 | 2,00–2,20 | 2,20–3,60 |
|---|---|---|---|---|---|---|---|---|
| kat | 21 | 19 | 17 | 15 | 13 | 11 | 10 | 10 (en az, 40 mm² ≈ boyutlandırma modelinin alt sınırı) |

Ana kiriş gövdesi (±45 PW): y 0,70–1,30 6 kat, 1,30–1,95 5 kat, 1,95–2,70 4 kat, 2,70–3,60 3 kat. Arka kiriş gövdesi 3 kat. Kesin değerler `structures.sizing.wing.main_cap.zones` ve `main_web.zones` alanlarındadır. 0,55–0,70 bölgesinin 12 kata inmesi §3.1'deki dil momentinden gelir.

### 4.2 Orta kanat kutusu

Yerleşim evresinin 2,0 mm yarı izotrop gövde içi kapakları 0,31 m genişlikte yaklaşık 7 MPa'da burkulur. Kapaklar `layups.ct_box_cover` sandviçiyle (±45, 0/90 / 6 mm 51 WF / ±45, ±45; 0,4/6/0,4 mm) değiştirildi; eldiven kaplamalarını gövde boyunca sürdürür. y = 0'da kırık bağlantılı bir orta hat kaburgası çevron kırığının başlık çiftini kiriş gövdelerine aktarır. FS-MS çerçeve gövdesine yüz başına bir ±45 kat eklendi.

### 4.3 Dış panel birleşimi: metal yapıştırmalıdan pimli kompozite

Yapı evresinde 7075 dil (I-kesit) ve 7075 çatal, kiriş başlıklarına EA 9394 ile yapıştırılmış, yapıştırma koparsa 2 × 6 M6 Ti cıvatanın limit yükü taşıdığı bir emniyetli yedek yolla tasarlanmıştı. Bağımsız doğrulama bunun üç noktada geçersiz olduğunu gösterdi (S1-03, S1-05): ortalama bindirme kesmesiyle hesaplanan yapıştırma Hart-Smith / Volkersen'e göre limit yükün altında kopar, tipik yapıştırıcı değerleri 1,8 değil 3,6 katsayı ister ve cıvataların ezildiği UD başlıklar ±45 içermediği için yarı izotrop ezilme değeri kullanılamaz. Ayrıca düzlem içi momentin yük yolu yoktu (S1-02). Bu yüzden birleşim §3.2'deki **pimli kompozit dil-çatala** çevrildi: başlıktan metale yapıştırma ya da cıvatalı aktarım kalmadı, pim delikleri ±45 ağırlıklı bloklardadır, düzlem içi moment arka kulakla bir kuvvet çiftine dönüşür. J-BOND-*, J-FS-*, J-DRAG-* ve eski dil / çatal satırları kaldırıldı; testler bunların geri gelmediğini de denetler.

### 4.4 Gövde ve şasi

| Eleman | Önce | Sonra | Neden |
|---|---|---|---|
| M-AFTKEEL arka omurga | 2024-T3 şekillendirilmiş U 24 × 40 × 1,6 mm | 7075-T651 talaşlı U 24 × 40 × 2,5 mm, iki alt oturma flanşı 26 × 1,6 mm | tampon yükünde MS −0,53; flanşlar alt kaporta yarımlarının oturma yüzeyi (VPK-05) |
| M-WELLROOF kuyu tavanı / arka hücre tabanı | rib_panel 0,4/6/0,4 (51 WF) | `fuel_floor_wellroof` 0,6/5,6/0,6 (71 WF), aynı 6,8 mm | yakıt basıncında yüz birim şekil değiştirmesi ve çekirdek kesmesi |
| M-WELLKEEL (yeni) | — | iki 1,0 mm PW gövde (y ±0,0105…0,0115), 0,12 m yükseklik | kuyu tavanı açıklığını yarıya indirir; aralarında orta hat kablo kanalı |
| M-MIDFLOOR görev bölmesi tabanı | rib_panel düz panel + orta hat şapka takviyesi | ortada TR-MISSION tepsi kesiği (y ±0,125), kesik kenarlarında iki yapıştırılmış şapka takviye | ekipmana alttan erişim (VPK-03); dış şeritler ön gövdenin alt başlığı (B-MIDFLOOR-BUCK 0,265) |
| M-FWDDECK ön hücre tabanı | — | yük tepsisi rayları (6061-T6 T 20 × 20 × 2,5 mm, y ±0,10) güverte takviyesi | güverte açıklığı yarıya iner |
| F-SPINDLE-NODE (düğüm) | FS3738 halkasına 6 × M5 ile bağlı yatak yuvası | yangın perdesine 7 × M5 ile bağlı işlenmiş 7075 düğüm, iki yanak | eski yuvanın bağlanacağı gövde yoktu (VPK-01) |
| Stabilatör mili / kök yuvası | Ti-6Al-4V 25 × 2,0 mm / kök yuvası ayrı denetlenmiyordu | Ti-6Al-4V 25 × 1,5 mm / 7075 kovan 31 × 3 mm, 0,10 m (T-SOCKET-*) | mil hafifletildi (MS 0,79), kök yuvası ayrı satırlarla denetlenir (boru MS 1,41) |
| M-SPINE sırt omurga kanalı | kaplamanın parçası (yırtılır örtü) | yapısal kanal 4 kat PW, U 44 × 24, 60 mm flanş | kayış x bileşeninin yük yolu (S1-04) |
| Yangın perdesi alt ayak bölgesi | düz sandviç | 71 WF çekirdek parçası + 24 katlı r 66 mm dolu bant | düzlem dışı ayak tepkisi (S1-06) |

---

## 5. Düzeltme turu 1: yapı bulguları

Her bulgu önce yeniden üretildi, sonra düzeltildi; hiçbir kontrol, test ya da gereksinim gevşetilmedi.

| Bulgu | Yeniden üretim | Düzeltme | Sonuç |
|---|---|---|---|
| S1-01 atalet rahatlatması yanlış yerde | doğrulandı: orta kutu başlıkları ve dil dış kanata yayılmıştı | `wing_inertia` her terimi yerinde dağıtır (§2.1); başlıklar yeni momentle yeniden boyutlandırıldı | kök M 5162 → 5293 N m, birleşim M 3092 → 3195 N m; bütün W-* payları ≥ 0 (en küçük 0,002) |
| S1-02 düzlem içi momentin yolu yok | doğrulandı | arka kirişte düşey pimli kulak + yuva bağlantısı; PHAA / NHAA / VD-DRAG durumları; pim bileşke yükü, dil iki eksenli gerilme, kulak ve arka pim satırları | J-* en küçük 0,139 |
| S1-03 yapıştırma yöntemi ve katsayısı geçersiz | doğrulandı | yapıştırmalı metal birleşim kaldırıldı, pimli kompozit birleşim (§4.3) | J-BOND satırları yok; yük yolu tümüyle pimli |
| S1-04 kayış x bileşeni çerçeve dışına | doğrulandı | sırt omurga kanalı M-SPINE + vidalı görev bölmesi üst kaplaması (§3.4) | P-* en küçük 0,38 |
| S1-05 UD başlıkta yarı izotrop ezilme | doğrulandı | başlıklardan bağlantı elemanı kaldırıldı; UD malzemeden ezilme değeri silindi; burçlu delikler ±45 bloklarda, OHC ile | I-UD-BEARING arayüz denetimi |
| S1-06 yoğun yük alan çerçeveler denetlenmemiş | doğrulandı | 14 satırlık "çerçeveler" grubu (FS-GEAR, FS-RS, FS-MS/RS halkaları, yangın perdesi ayakları, köşe eki), yukarı kilit, düğüm taban cıvataları | en küçük 0,002 (FW-FOOT-CORE); FS3738 omurga desteği açık konu |
| S1-07 açık konuda eski kırık kuvveti | doğrulandı | açık konu metni CT-KINK değerinden üretilir | 17,0 kN her yerde |
| S1-08 burun bacağı zarfı | doğrulandı | `landing_gear.nose.leg_frontal_width` 0,035 → 0,036 m (üretici `NOSE_LEG_WIDTH`) | sizing ve layout_check takım zarfları yeni genişlikte yeşil |
| S1-09 kulak kenar mesafeleri | doğrulandı | kayış U-kulağı e = 2 D (Ø6 pim, e 12 mm); mafsal ve burun kulakları kulak istisnası olarak Bruhn kesme-ezilmesiyle gerçek e/D'de | G-TRUN-BR 2,11, P-LUG-BR 1,75 |
| S1-10 `layout_check --check` dosya yazıyor | doğrulandı | `--check` salt okunur; çıktılarda çalışma süresi yok | testle denetlenir (dosya sağlamaları değişmez) |

---

## 6. Kütle ve bütçe

Boyutlandırılan elemanların aşağıdan yukarı kütlesi kavramsal kütle modelinin paylarıyla karşılaştırıldı (büyüme payı öncesi, kg):

| Grup | Aşağıdan yukarı | Model payı | Fark |
|---|---|---|---|
| A — kanat birincil yapısı (ana başlıklar 2,434, arka başlıklar 0,171, gövdeler 0,511 + 0,174, birleşim donanımı çifti 1,275: dil 0,596 / adet, arka kulak 0,042 / adet; üst kutu kaplaması çekirdeği 0,045) | 4,609 | 5,454 | −0,845 |
| B — orta kutu kapakları 0,686, gövde takviyeleri 0,034, kompozit çatallar 0,728, ana pimler (4) 0,261, arka pimler + yuva bağlantıları 0,111, kırık bağlantısı 0,10 | 1,920 | 1,600 | +0,320 |
| C — şasi eklemeleri (yakıt tabanları 0,165, M-WELLKEEL 0,093, taban takviyeleri 0,064, arka omurga 0,055, yangın perdesi ayak bantları 0,154, çekirdek parçası 0,017) | 0,548 | 0 | +0,548 |
| D — stabilatör milleri (0,152 / adet), düğüm bağlantıları (0,328 / adet: taban 0,086, iç yanak 0,053, dış yanak 0,189), yataklar, kök yuvaları + çapraz cıvata (0,084 / adet) | 1,208 | 1,100 | +0,108 |
| E — burun mafsalı bağlantısı 0,209, sırt omurga kanalı 0,258 + takviyeler 0,015, kayış bağlantıları 0,089, kilit pimleri 0,018, çerçeve bantları 0,026 | 0,616 | 0,650 | −0,034 |
| **Toplam** | | | **+0,097** (büyüme payıyla +0,102) |

Düğüm bağlantısının kütlesi zarf hacminin doluluk oranlarıyla (taban 0,55, dış yanak 0,70) ve iç yanak / göbek geometrisinden hesaplanan bir tahmindir. Bu değerler `structures.sizing.mass` bloğu olarak `spec.yaml`'a yazılır ve `analysis/sizing.py` bunları kavramsal modelin yerine okur (`wing_structure`, `mass_items`: orta kutu, çerçeve eklemeleri, burun mafsalı, paraşüt bağlantıları; `tail_structure`). Kapanış yeniden yapıldı: boş kütle 101,469 kg (MTOM 149,9 kg sabit), kanat kök x_c4 2,503 → 2,507 m, dayanım 10,36 h (R-02), azami faydalı yükle 9,59 h (R-02b); 61 gereksinimin 61'i karşılanır.

Grup tavanları yeni tahminlere (10 g'a yukarı yuvarlanmış) göre yeniden dağıtıldı: şasi 11,58 → 12,38 kg (12,370; kompozit çatallar, düğüm bağlantıları, sırt kanalı, perde bantları), kuyruk 7,91 → 7,60 kg (7,597; 25 × 1,5 mm mil ve 7075 kök yuvası kovanıyla stabilatör bağlantıları 0,384 → 0,236 kg / adet; yatak taşıyan düğüm bağlantıları şasi grubundadır), kanat 15,98 → 15,46 kg (15,453; metal dil yerine kompozit dil). Tavanların toplamı 101,56 → 101,53 kg'dır; R-56 payı (R-02b'nin tam karşılandığı boş kütle 101,700 kg − yedek 0,05 kg − tavanlar toplamı) **0,097 → 0,120 kg** olarak pozitif kalır. Pay küçüktür: ayrıntılı tasarımda her kütle artışı başka bir kalemden karşılanmalıdır.

---

## 7. Arayüzler

`structures --check` şunları da denetler: `spec.structures.sizing` yeniden üretilen blokla birebir aynı mı, `spec.layups` içindeki yapı evresi dizilimleri güncel mi ve yerleşimdeki elemanlar yapı ölçülerini taşıyor mu. 15 arayüz denetimi vardır: I-AFTKEEL (malzeme / kesit), I-WELLKEEL, I-WELLROOF (dizilim), I-CTBOX (kapak dizilimi), I-MIDFLOOR (iki kesik kenarı takviyesi ve tepsi kesiği), **I-NODE** (düğümün kök parçası cıvataları, perde cıvataları, iç yatak silindiri ve dış yatak istasyonu), **I-TONGUE** ve **I-FORK** (kompozit dil ve çatal, flanş / gövde / takviye kalınlıkları), **I-SPINE** (kanal kalınlığı = kat sayısı × kat kalınlığı), **I-RISER** (kilit pimi çapı ve kulak kalınlığı), **I-UD-BEARING** (UD malzemede ezilme değeri yok) ve dört katman dizilimi. Yerleşim tarafında `layout_check` düğümün, köşe bağlantısının ve bütün bağlantı parçalarının cıvata desenlerini (kenar ≥ 2 D metal / 2,5 D kompozit, aralık ≥ 3 D), yapı–yapı çakışmalarını ve pim / rayba yollarını denetler ([03](03_yerlesim_ve_yapi_konsepti.md) §11).

---

## 8. Sınırlamalar ve açık konular

* **Çırpınma, ıraksama, kanatçık tersinmesi analiz edilmedi.** Kanat ve kuyruğun rijitlik / kütle modeli henüz yok. İlk uçuştan önce yer titreşim testi (GVT) ve çırpınma analizi gerekir (CS-LUAS.629). ±45 ağırlıklı kaplamalar burulma rijitliğini artırır; stabilatörün kütle dengesi ve mil-yatak boşluğu ayrıca incelenmelidir.
* **FS3738'de arka omurga desteği (FR-3738-KEEL):** kuyruk tamponu çarpmasında omurga yangın perdesi ucunda mafsallı ise FS3738 10,25 kN (limit) düşey tepki taşımalıdır; 2,0 mm 2024-T3 alt U halkası buna göre boyutlandırılmadı. Talaşlı bir alt çerçeve, omurgaya ön gövdede bir moment eki ya da yük sınırlayıcı bir kızak detay tasarımda seçilecektir. Bu turda payı hesaplanmış bir eleman olarak değil, bilgi satırı ve açık konu olarak raporlanır.
* **Kompozit birleşim:** burçların dil ve kulak laminelerindeki ezilmesi yarı izotrop delikli bası dayanımıyla denetlendi (delik e/D 3'ün altında); tasarım değerleri dondurulmadan önce pimli CFRP dil / çatal eleman testi (statik, ETW, yorulma) gerekir.
* **Yorulma ve hasar toleransı** yalnızca STANAG UL13.1.2 birim şekil değiştirme sınırlarıyla temsil edildi. Metal parçaların (pimler, burçlar, arka kulak, düğüm ve takım bağlantıları, motor kafesi kaynakları) yorulma ömrü ve kompozit BVID/CVID kanıtı (CS-LUAS.572/573) test ister.
* **El hesabı varsayımları:** basit mesnetli paneller, düzlem kesitler, D-burnu ve firar kenarı kaplamaları eğilmede sayılmadı, üretici minimum çekirdek değerleri, çubuk parçalar için sac izin verilen değerleri (Ti-6Al-4V, 7075-T6). Kanat kutusu, birleşim, orta kutu, düğüm ve takım bağlantıları için sonlu eleman doğrulaması ve kupon / eleman testleri (sandviç kıvrılma ve buruşma, CFRP burç ezilmesi, dökme insertler) gerekir.
* **Tedarik gereksinimleri (MS'siz satırlar):** 61805-2RS yatakların statik yük sayısı (iç 1,50 kN, dış 2,22 kN nihai), taret asansörü bilyalı vida (0,62 kN) ve ray taşıyıcıları (0,27 kN), motor sönümleyicisi nihai kapasitesi (1,80 kN), ana / burun takımı EMA torku (14 / 25 N m), burun aşağı kilidi (190 N m limit). Katalog değerleri araştırma verisinde yoktur.
* **Ana iç takım kapakları:** DA 22 kapalı kapağı V_D emmesine karşı tutamaz (gereken nihai moment 10,3 N m); ölü nokta bağlantısı ya da kapak kilidi gerekir (G-DOOR-LOCK).
* **Çevron kırığı:** başlık başına 17,0 kN kırılma kuvveti (CT-KINK) kırık bağlantısının ve orta hat kaburgasının detay tasarımına yük olarak verildi.
* **Kütle tahminleri:** düğüm bağlantısı (doluluk oranları), kırık bağlantısı (0,10 kg) ve yatak kütleleri tahmindir.
* **Tasarım kararları:** taşıma yük katsayıları (3,0 / 1,5 / 1,5 g), kuyruk tamponu yükü (45°'de 1,0 × MTOM ağırlığı), takım işletme hızı 1,6 V_S. İşletmeci belirtimi ve test verisiyle değiştirilmelidir.
* **Kütle payı:** R-56 payı 0,120 kg'dır.
