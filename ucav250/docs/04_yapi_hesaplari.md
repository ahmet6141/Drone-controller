# YK-250 HANÇER — Yapı hesapları (faz 3, yapı evresi ve düzeltme turları 1–3)

**Tarih:** 10 Ekim 2026 · **Durum:** `python3 -m ucav250.analysis.structures --check` 323 satırın 300 emniyet payının (MS) hepsini sıfır ya da pozitif (en küçük 0,0002, W-CAP-24; kat sayıları tam sınırda boyutlandırılır), `spec.yaml → structures.sizing` bölümünü güncel ve 18 yerleşim arayüzünün 18'ini tutarlı buluyor; 23 satır MS'siz gereksinim satırıdır (biri, G-DOOR-DRIVE, açık bir tahrik sorununu gösterir, §6a / §9). Boyut değişiklikleri `spec.yaml`'a yazıldı; `sizing --check` (62/62), `layout_build.build --check` (bayt bayt aynı) ve `layout_check --check` (99/99) yeşil.

> **Kapsam.** YK-250 HANÇER sivil bir EO/IR gözetleme ve araştırma İHA'sıdır. Hesaplarda dış yük taşıma bağlantısı,
> askı ya da yük bırakma düzeneği yoktur; "bağlantı parçası" (fitting) sözcüğü yalnızca yapı içi bağlantıları
> (kanat birleşimi, iniş takımı mafsalı, motor bağlantısı, paraşüt kayışı, kuyruk düğümleri) anlatır.

Bu belge HANÇER'in birincil yapısı için yapılan el hesaplarını anlatır: hangi yük durumlarının alındığı, her elemanın hangi yöntemle ve hangi izin verilen değerle denetlendiği, hangi elemanların yetmediği ve nasıl yeniden boyutlandırıldığı, bunun kütleye ve kütle bütçesine etkisi ve açık kalan konular. §5, bağımsız doğrulamanın ilk yapı bulgularına (S1-01 … S1-10) karşı yapılan **düzeltme turu 1**'i, §6 ikinci doğrulamanın yapı bulgularına (VS2-01 … VS2-12) karşı yapılan **düzeltme turu 2**'yi özetler. Satır satır emniyet payı tablosu [`out/structures.md`](../out/structures.md) dosyasındadır (makinece okunur sürümü [`out/structures.json`](../out/structures.json)); bu belge yöntemi ve sonuçları özetler.

```
python3 -m ucav250.analysis.structures                # hesap; out/structures.md ve out/structures.json yazılır
python3 -m ucav250.analysis.structures --update-spec  # boyutlandırılan ölçüler spec.structures.sizing'e, katman dizilimleri spec.layups'a
python3 -m ucav250.analysis.structures --check        # MS >= 0, spec bloğu güncel, yerleşim arayüzleri tutarlı (çıkış 1 = hata); hiçbir dosya yazmaz
```

Bir değişiklikten sonra sıra şudur: `layout_build.build --write` → `structures --update-spec` → `sizing --update-spec` (kütle kalemleri, kapanış) → yeniden `layout_build.build --write` (kanat konumu ve kütle yerleşimi; `sizing` mekanizma değerlerini 6 anlamlı basamakla sakladığı için son yazan üretici olmalıdır) → `build --check`, `structures --check`, `sizing --check`, `layout_check --check`. Kanat konumu kapanışta değiştiğinde bu döngü sabit noktaya gelene kadar tekrarlanır. Düzeltme turu 1'de yerleşim ile boyutlandırma arasında 0,1 mm'lik yuvarlamanın yol açtığı iki adımlı bir salınım görüldü; boyutlandırma kapanışı artık kanadı yalnızca |Δx_c4| ≥ 0,4 mm olduğunda taşır (`sizing.WING_DEADBAND`, kanat AM'sinin 0,4 mm içinde kalması statik payı ölçülebilir biçimde değiştirmez). Düzeltme turu 1'de kanat kök çeyrek veter noktası 2,503 m'den 2,507 m'ye geldi; düzeltme turu 2'nin kütle değişiklikleri (kanat ve kuyruk hafifledi, motor bölmesinin ısı koruması ve yangın perdesi aşağıdan yukarı sayıldı; §7) ağırlık merkezini geri aldı ve kanat **2,487 m**'ye taşındı.

---

## 1. Yöntem

### 1.1 El hesapları ve tek kaynak

Her kontrol kapalı biçimli bir el hesabıdır ve [`analysis/structlib.py`](../analysis/structlib.py) içindeki yardımcılarla yapılır: klasik lamine teorisi (ABD matrisleri, ilk katman hasarı: en büyük gerilme ve Tsai-Wu), özel ortotrop levha basma ve uzun levha kesme burkulması (Kollár & Springer), sandviç kesme düzeltmesi, yüz buruşması (Zenkert, Q = 0,5), kesme kıvrılması (N = G_c d²/c), Johnson-Euler kolonu, boru burulması, Bredt kesme akışı, elastik cıvata grubu (düzlem içi, çekme ve düzeltme turu 2'de eklenen altı serbestlik dereceli grup), pim eğilmesi (Melcon-Hoblit kolu), kulak (lug) dayanımı (Bruhn D1, gerçek e/D'de kesme-ezilme), doğrudan rijitlik yöntemiyle uzay kafes, enerji yöntemiyle iniş yük katsayısı ve basit mesnetli dikdörtgen levhanın merkez yamasındaki moment (Timoshenko, levhalar §19; yangın perdesi ayakları). Bu yardımcıların her biri [`tests/test_ucav250_structures.py`](../../tests/test_ucav250_structures.py) içinde kapalı biçimli bir değerle karşılaştırılarak test edilir.

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

Birleştirme kuralı CS-LUAS.619'a göredir: toplam katsayı = 1,5 × uygulanabilir özel katsayıların **en büyüğü** (çarpılmazlar). Kompozit katsayısı 1,2 bu özel katsayılardan biridir (sıcak/nem etkisi ETW değerlerinin içindedir). Hasar toleransı birim şekil değiştirme sınırları (STANAG 4703 UL13.1.2, [`data/research/standards.yaml`](../data/research/standards.yaml)) nihai yükte uygulanır: 2 mm'den kalın lamine bası 3000 µε, sandviç kaplama ve ≤ 2 mm lamine bası 2600 µε, çekme 5000 µε, kesme 5200 µε. Bu sınırların bir uygulanabilirlik koşulu vardır: matrise duyarlı özelliklerin ETW kaybı oda sıcaklığı / kuru değerinin %50'sinin altında kalmalıdır. Düzeltme turu 2'de bu koşul ayrı bir satırla denetlenir (DT-COND, VS2-11): NCAMP MTM45-1/AS4 verisinde UD Xc, PW Xc / Yc ve PW S_0,2 için aynı yöndeki ETW ve RTD değerleri (B-tabanı varsa B-tabanı, yoksa ortalama) karşılaştırılır; en büyük kayıp **%47**'dir (PW S_0,2), koşul sağlanır ve UL13.1.2 sınırları kullanılır.

İzin verilen değerler: MTM45-1/AS4 UD ve PW katları B-tabanı ETW (NCAMP), yarı izotrop delikli bası/ezilme değerleri ([`materials.yaml`](../data/research/materials.yaml) `design_values_for_code`), metal parçalar MMPDS (A-tabanı), cıvatalar ISO 898-1 / ISO 3506-1, ROHACELL çekirdekler üretici minimum değerleri. **UD bant için ezilme değeri yoktur** (düzeltme turu 1'de `cfrp_ud_mtm45_as4`'ten kaldırıldı, S1-05): UD'ye bağlantı elemanı açılmaz, delikler her zaman ≥ %40 ±45 içeren bloklardadır. Burçlu kompozit deliklerin ezilmesi, delik kenar mesafesi %2 kaymalı ezilme değerinin koşulunu (e/D ≥ 3) sağlamadığı için **yarı izotrop delikli bası dayanımıyla (175,4 MPa)** ihtiyatlı biçimde denetlenir; bu bir yerine koymadır ve eleman testiyle doğrulanacaktır (§9). Kulak eksenel dayanımında (`lug_axial`, Bruhn D1.5) malzemenin çekme dayanımı Ftu kullanılır (düzeltme turu 2, VS2-09).

---

## 2. Yük durumları

### 2.1 Uçuş zarfı ve kanat yükleri

| Büyüklük | Değer | Not |
|---|---|---|
| V_S (temiz, MTOM) / V_A / V_C / V_D (EAS) | 23,9 / 45,0 / 45,0 / 57,0 m/s | `structures.derived` |
| Manevra limit yük katsayıları | +3,8 / −1,52 | CS-LUAS.337 |
| Kanat tasarım yük katsayısı (MTOM'da) | +5,46 | hamle matrisinin en büyük n·m değeri (6 kütle × irtifa) |
| Negatif kanat durumu (MTOM'a eşdeğer) | −3,54 | en büyük \|n·m\| / MTOM |
| Teçhizat yük katsayısı (en hafif durum 106,4 kg) | +6,99 / −4,99 | hamle; bağlantı ve teçhizat tutma için |

Kanat yükü referans yamuk üzerinde Schrenk dağılımıdır; kanat yapısı, kumanda yüzeyleri ve kanatçık / flap eyleyicileri aynı yük katsayısıyla **atalet rahatlatması** olarak düşülür. Düzeltme turu 1'de (S1-01) her kütle terimi fiziksel olarak bulunduğu yere konur: ana kiriş başlıkları kat sayısı programına göre (gövde içindeki y < 0,40 kutu başlıkları dış kanatta rahatlatma vermez), dış panel kiriş gövdeleri gövde kat bölgelerine ve vetere göre, arka kiriş gövde yanı kaburgasından uca, kaburgalar veterin karesiyle, birleşim donanımı (kompozit dil, başlık kalınlaşması, arka kulak) dilin ucundan başlık kalınlaşmasının sonuna kadar, kaplamalar ve kumanda yüzeyleri gerçek kanat vetere göre, eyleyiciler yerleşimdeki konumlarında nokta kütle olarak. Düzeltme turu 2'de (VS2-05) rahatlatma **en küçük inandırıcı kanat kütlesiyle** hesaplanır: kalemlerin büyüme payı olmayan taban kütleleri, boyutlandırma modelinin 1,05 katsayısı da çıkarılarak (daha ağır bir kanat yalnızca rahatlatmayı artırır). Rahatlatmaya giren yarım kanat 9,08 kg'dır. Kök eğilme momenti (limit) **5361 N m**'dir (rahatlatmasız 6098 N m); birleşim düzleminde (y 0,70) **V 2594 N, M 3234 N m, T 104 N m**. Burulma zarfı üç durumun büyüğüdür: V_D'de kesit c_m0'ı (NLF(1)-0416, −0,102), V_A'da tam kanatçık (CS-LUAS.349(b): Δc_m = −0,01/derece) ve V_F'de 40° flap.

**Veter yönü durumları (S1-02).** Dış panel birleşimi düzlem içi (sürükleme) momentini de taşır. Üç durum tanımlıdır: PHAA (pozitif yüksek hücum açısı: kanat CL_maks'ta veter yönünde öne doğru C = L tan α = 548 N, birleşimde Mz = 683 N m), NHAA (negatif yüksek hücum açısı, kesit c_l,min: Mz 390 N m) ve VD-DRAG (V_D'de sürükleme ağır basan durum: Mz −16 N m). Mz, ana pimler ile arka pim arasındaki 0,292 m veter yönü kol üzerinden açıklık yönünde bir kuvvet çiftidir (F_y = Mz / Δx; ana pimler F_y/2'şer, arka pim F_y).

**Yuvarlanma (CS-LUAS.349(a)):** bir tarafta n_A'da %100, diğerinde 2/3 yük ve tam kanatçık burulması. Daha yüklü tarafın kök momenti 3731 N m'dir, tasarım durumunun (5361 N m) altındadır (W-ROLL bilgi satırı).

### 2.2 Kuyruk ve kumandalar

* **Stabilatör:** V_A'da tam sapma ve kesit CN_maks (panel başına 471 N limit), V_C'de CS-VLA 425(d) hamlesi (256 N, en büyük hamle azaltma katsayısı 0,78), simetrik olmayan durum %100 / %72 (CS-LUAS.427(b)). Mil kesitinde moment 169 N m; iç yatak tepkisi 966 N, dış yatak tepkisi 1436 N (limit).
* **Dikeyler:** 15° yanal kayma × 1,5 aşma, V_A'da tam dümen ve V_C'de yanal hamle; her biri kesit CN_maks ile sınırlı (dikey başına 572 N, kökte 208 N m).
* **Kumanda yüzeyleri:** yüzey yükü CS-VLA Ek B ortalama yüklemesi ile eyleyicinin uygulayabildiği menteşe momentinin büyüğüdür; bu uçakta her yüzeyde eyleyici belirleyicidir. Menteşe tepkileri sürekli kiriş katsayılarıyla; menteşe ekseni boyunca atalet 24 g / 12 g (CS-LUAS.393(b)); itme çubukları eyleyici durma torkuyla (CS-LUAS.395); yer rüzgârı (CS-VLA 415).
* **Kuyruk tamponu:** skidden 45° yukarı-geri 1,0 × MTOM ağırlığı (1470 N; tasarım kararı, burun tekerli uçak tamponu için kural yok).

### 2.3 Yer yükleri ve iniş takımı

Çökme hızı 2,35 m/s (CS-LUAS H.2(b)), serbest düşme yüksekliği 0,283 m, kullanılabilir yol 92 mm, verim 0,63, kanat taşıması 2/3 W → **n_j = 5,40**, atalet yük katsayısı 6,07. Ana takım durumları: düz iniş (dönme başlangıcı, geri yaylanma, en büyük düşey; AMC VLA 479(b)), kuyruk aşağı iniş, tek teker inişi, yan yük (içe 0,5 W, dışa 0,33 W), frenli yuvarlanma (0,8 sürtünme). Burun takımı: üç noktalı düz iniş ve ek durumlar (arka 2,25/1,8, ön 3,2/0,9, yan 2,25/1,575 × statik). İniş yedek enerjisi 1,2 V_çökme'de n_j 7,0 (nihai). Takım işletme hızı tasarım kararı olarak 1,6 V_S = 38,3 m/s EAS. Toplanmış ana bacağın yukarı kilide yükü teçhizat yük katsayısıyla hesaplanır (S1-06). Düzeltme turu 2'de mafsal bağlantısına bacak momentlerinin üçü de (sürükleme çifti M_y ve yuvarlanma momenti M_x dahil) verilir (VS2-02).

**Yer taşıma (VS2-12).** Çekme yükü burun çatalı dingilinde 0,3 W'tır (W ≤ 30 000 lb için CS-23.509 / FAR 23.509 değeri, muhafazakâr tasarım kararı; CS-LUAS / CS-VLA çekme maddesinin metni araştırma dosyalarında yoktur), çekme demiri ekseninden ±30°'ye kadar. Uçak dışarıda bağlanmaz ve krikoyla kaldırılmaz: işletme kavramında uçmadığı zaman taşıma beşiğinde ya da barınaktadır (`structures.sizing.ground_handling`); bağlama noktaları açık konudur (§9).

### 2.4 Motor bağlantısı

Motor grubu 10,10 kg (büyüme payı dahil), I_p 0,0227 kg m². Durumlar: MCP torku × 6 + n_A + itki; kalkış torku + 0,75 n_A + itki; en hafif durumun hamle yük katsayısı + MCP torku; negatif yük katsayısı; yan yük 1,47 g; jiroskopik (sapma 2,5, yunuslama 1,0 rad/s); iniş n_j + 0,67; çarpma 15 g ileri, acil iniş 6 g aşağı ve paraşüt açılması (nihai). Dört eşit sönümleyici üzerinde rijit cisim dağılımı; 4130 kaynaklı kafes doğrudan rijitlik yöntemiyle çözülür. Ayak tepkileri yangın perdesine düzlem içi ve **düzlem dışı** olarak verilir: alt ayakta en büyük düzlem dışı tepki 2204 N, üst ayakta 1782 N (nihai, hamle + MCP torku).

### 2.5 Paraşüt, yakıt, taret, teçhizat, taşıma

* **Paraşüt:** açılma şoku 13,1 kN tek kayış ayağında (yalnız nihai, × 1,15 bağlantı). Yönler: statik ayak yönü etrafında 30° koni (birleşme halkası AM'nin 1,5 m üstü) ve düşeyden 60° geriye kadar açılma silkmesi. Kayışın uçak eksenindeki (x) bileşeni en çok **11,34 kN**'dur (S1-04); en hafif kütlede açılma yük katsayısı 12,6 olur.
* **Yakıt bölmeleri:** tasarım basıncı 21,0 kPa nihai = 1,5 × CS-LUAS.965(b) test basıncı 14 kPa; yakıt sütunu, acil iniş 9 g ileri ve 6 g aşağı ve paraşüt açılması karşılaştırılır.
* **Taret asansörü:** E180 büyüme tareti + taşıyıcı 4,35 kg; nihai yük katsayıları aşağı 12,6, ileri 10,9, yan 2,2. Kayar kapaklar V_D emmesinde (|Cp| = 1 üst sınır).
* **Teçhizat tutma:** nihai aşağı 12,6, yukarı 7,5, ileri 10,9, yan 2,2.
* **Taşıma:** kızakta orta gövde; 3,0 g düşey, 1,5 g ileri-geri, 1,5 g yan limit (tasarım kararı).

### 2.6 Çerçevelere giren yoğun yükler (S1-06)

| Yük | Değer (limit, aksi belirtilmedikçe) | Kaynak durum |
|---|---|---|
| FS-GEAR çerçevesine takım kirişi uç tepkisi (yan başına, sürükleme çifti dahil) | 3620 N | iniş durumlarının en büyüğü (VS2-02) |
| FS-RS çerçevesine takım kirişi uç tepkisi (yan başına, sürükleme çifti dahil) | 3755 N | aynı |
| Yangın perdesi alt ayağı, düzlem dışı | 2204 N (nihai) | hamle + MCP torku |
| Ana yukarı kilit kancası | 432 N (nihai) | toplanmış bacak, teçhizat yük katsayısı |
| FS3738'de arka omurga desteği | 10,20 kN | kuyruk tamponu çarpması (düzeltme turu 2'de boyutlandırıldı, VS2-03) |

---

## 3. Elemanlar ve sonuçlar

261 emniyet payı ve 19 gereksinim / bilgi / geometri satırı (toplam 280) hesaplanır; negatif pay yoktur; en küçük MS 0,0002 (W-CAP-07): kiriş başlıkları kat sayısı bölgelerinde en az kat sayısıyla boyutlandırıldığı için paylar sıfıra yakındır. Gruplara göre belirleyici satırlar:

| Grup (satır) | Belirleyici kontrol | MS |
|---|---|---|
| Kanat (69) | ana kiriş başlığı bölgeleri: kaplama yüzü bası birim şekil değiştirmesi 2600 µε (W-CAP-07, y 1,00–1,10) | 0,0002 |
| Dış panel birleşimi (27) | dil-başlık rampası: azalan başlık kolunda kaplama birim şekil değiştirmesi (J-TRANS-SKIN) | 0,014 |
| Orta kanat kutusu (15) | FS-MS gövdesi kesme kıvrılması | 0,210 |
| Kuyruk (32) | arka omurga kirişi, tampon yükü (T-AFTKEEL) | 0,128 |
| Kumanda yüzeyleri (21) | flap itme çubuğu kolonu | 0,709 |
| İniş takımı ve yer taşıma (33) | takım kirişi UD başlıkları, bacak çentiğinde düşey yük + sürükleme çifti (G-BEAM-CAP) | 0,027 |
| Motor bağlantısı (8) | kafes çubuğu kolonu (Johnson-Euler) | 0,924 |
| Paraşüt (17) | Ø8 Ti kilit pimi eğilmesi (P-SHACKLE-BEND) | 0,298 |
| Yakıt bölmeleri (12) | FS-FUEL perdesi çekirdek kesmesi | 0,198 |
| Taret (6) | kayar kapak tahriki | 8,01 |
| Taşıma (3) | kızak yastığı altında çekirdek ezilmesi | 0,596 |
| Gövde (10) | görev bölmesi taban şeridi burkulması (B-MIDFLOOR-BUCK) | 0,258 |
| Çerçeveler (20) | alt motor ayağında yangın perdesi çekirdek kesmesi (FW-FOOT-CORE) | 0,014 |
| Teçhizat (7) | faydalı yük tepsisi vidaları | 12,5 |

### 3.1 Kanat kesiti

Kesit modeli düzlem kesitler varsayar: ana ve arka kiriş UD başlıkları, başlıkların üstünde dolu kaplama (1 mm), kirişler arasındaki kutu kaplamalarının dış ve iç yüzleri; D-burnu ve firar kenarı kaplamaları sayılmaz (başlık birim şekil değiştirmesi için muhafazakâr). Gövde içinde (y < 0,40) kaplama yerine orta kutu kapakları alınır. Her başlık bölgesi için o bölgenin altı istasyonunda bütün ölçütleri sağlayan en küçük kat sayısı aranır: başlık bası/çekme birim şekil değiştirmesi (3000 / 5000 µε), A-tabanı tahmini 0,85 Fcu / 0,85 Ftu ile gerilme, kaplama yüzü bası birim şekil değiştirmesi (2600 µε), delikli bası (OHC) ve yüz buruşması. Belirleyici ölçüt kaplamanın 2600 µε sınırıdır. Başlıklar saf UD'dir: hiçbir ana kiriş başlığından bağlantı elemanı geçmez (S1-05).

**Eldiven bölgesi (y 0,40–0,70).** Kompozit dil, birleşim momentini pimlere kadar kendisi taşır: dilin momenti dış pimin dışında birleşim momenti + kesme × kol, pimler arasında iç pime doğru doğrusal olarak sıfıra iner. Eldiven kutusunun başlıkları bu yüzden toplam momentten dil momentinin çıkarılmışıyla boyutlandırılır (`structures.sizing.wing.main_cap.fork_zone`); iç pimin içinde (y 0,40–0,55) bütün moment kutudadır. Eldiven ana kiriş gövdeleri aynı zamanda çatal kulaklarıdır (2 × 10 kat ±45; W-GLOVEWEB MS 7,61).

Kiriş gövdeleri (±45 PW) dış panelde ilk katman hasarı, 5200 µε kesme birim şekil değiştirmesi ve uzun levha kesme burkulmasıyla boyutlandırılır. Kutu kaplama panelleri bası + kesme etkileşimiyle (Rc + Rs² = 1, CLT D*, sandviç kesme düzeltmesi) ve kesme kıvrılmasıyla denetlenir; kaburgalar Brazier ezilme yükünde sandviç kolon olarak.

### 3.2 Dış panel birleşimi (y 0,70): pimli kompozit dil-çatal

Düzeltme turu 1'de yapıştırmalı metal dil-çatal bırakıldı (§5, S1-03) ve yerine planör uygulamasındaki pimli kompozit birleşim kondu (`structures.sizing.wing_joint`, `layout.chassis.wing_joint`):

* **Dil (YK250-WG-151, dış panel):** CFRP kiriş kökü 30 × 61 mm, 0,292 m kavrama; UD flanşlar 30 × 10 mm kök kaburgasından dış pime kadar (dış panel ana kiriş başlıklarının devamı, kök kaburgasından 0,10 m boyunca 1:20 kalınlaşır), dış pimden iç pime doğru dil momentiyle birlikte doğrusal olarak **3 mm'ye incelir** (kat düşürmeler gövde tarafında, iç yüzde; eğim 1:26, 1:20 kuralından yatık; düzeltme turu 2); ±45 PW gövde 5 mm (25 kat), iki pim deliğinin çevresinde 30 mm genişlik × 50 mm boyunda dolu [±45/0/90] bloklar, yapıştırılmış 4130 burçlar 16 H8 × dış çap 22 × 30 mm.
* **Çatal (YK250-CH-053, orta kesit):** eldivenin ana kiriş kutusu; 40 mm UD başlıklar (çatal boyunca 52 mm'ye genişler), 30,4 mm aralıklı iki ±45 PW gövde (kulaklar, 2 mm, 10 kat), pim bölgelerinde 10 mm'ye kalınlaştırılmış [±45/0/90] bloklar, yapıştırılmış 4130 burçlar 16 H8 × dış çap 22 × 10 mm.
* **Ana pimler:** 2 × Ø16 Ti-6Al-4V, kiriş boyunca 184 mm aralıklı (P-MAIN1 y 0,463, P-MAIN2 y 0,645). Düşey kuvvet çifti R_iç 18,54 kN, R_dış 21,13 kN; burulma çifti F_T 357 N (limit, PHAA).
* **Arka kiriş:** dış panelin arka kiriş kök bağlantısındaki 7075 kulak (t 8 mm, genişlik 32 mm, e 16 mm = 2 D), birleşim kaburgasındaki yuva bağlantısının (YK250-CH-054) iki 4 mm levhası arasına girer ve düşey Ø8 Ti bilyalı kilit pimiyle tutulur. Kulak veter yönü kuvveti ve düzlem içi momentin açıklık yönü kuvvet çiftini taşır.
* **Derinlik geçişi (VS2-04):** dış panel ana kiriş başlıkları kök bölmesinde, 0,10 m'lik kalınlaşma bölgesi boyunca dil flanşı hattına doğrusal bir rampayla iner (`structures.sizing.wing_joint.transition`); rampanın iki ucundaki kırılma kuvvetleri dil gövdesine (ezilme), başlık-gövde ara yüzeyine (kesme) ve birleşim kaburgasının 20 katlı dolu bandına verilir; kök bölmesinin ilk 0,12 m'sinde kaplamanın dış yüzüne bir kat 0/90 takviye konur.

Sonuçlar (MS): pim kesmesi 3,98, **pim eğilmesi 0,208** (Melcon-Hoblit kolu, plastik kazanç yok, × 1,5 × 1,5), pim–burç ezilmesi 5,83, dil burcu ezilmesi 0,793, **çatal kulağı burcu ezilmesi 0,195**, dil UD başlığı kalınlık yönünde bası / bası birim şekil değiştirmesi / A-tabanı gerilme 1,81 / **0,125** / 0,607, incelen başlık boyunca birim şekil değiştirme 0,125, dil gövdesi kesmesi 0,147, çatal kulakları kesmesi 0,198; geçiş rampası: gövde ezilmesi 1,79, ara yüzey kesmesi 1,04, kaburga bandı 0,248, başlık 0,425, **kaplama 0,014**; arka pim kesme / eğilme 10,1 / 3,19, arka kulak 6,68, kulak ezilmesi 7,87, yuva levhası 2,07, yuva bağlantısının kaburga ve gövde cıvataları ≥ 2,13.

### 3.3 Kuyruk: stabilatör düğüm bağlantısı (VPK-01 ile birlikte)

Yerleşim evresinin FS3738 halka çerçevesine cıvatalanan iç yatak yuvası kurulamıyordu (30 mm'lik halka bandı 25 mm'lik mil deliğini ve yuva cıvatalarını taşıyamaz). Yerine her yanda bir **işlenmiş 7075-T651 düğüm bağlantısı (F-SPINDLE-NODE, YK250-CH-095)** kondu: yangın perdesinin arka yüzünden geriye konsol, 8 mm U biçimli taban flanşı 7 × M5 12.9 ile perde yığınından geçer (önde 7075 destek levhası); iç kol 3 mm gövdeli, iç yatak göbeği (61805-ZZ, Ø37) burada; dış yanak 7 mm, kök parçası arka kiriş kök bağlantısını 4 × M6 12.9 ile (çekme + kesme) taşır ve kol süpürmesinin dışında kalır. FS3738 artık düğümün altında biten bir alt U halkadır.

* **Mil:** Ti-6Al-4V 25 × 1,2 mm (MS 0,528); kama milin dış ucunda, yuva geçmesinin son 20 mm'sinde (kama kesiti burulma 1,31, kama kökünde eğilme + burulma 0,975). **Kök yuvası:** 7075 kovan 29 × 2 mm, 0,10 m (ezilme 81,6, boru 0,578, dış çap 15,8).
* **Düğüm:** kök parçası cıvataları 9,86, kök cıvataları ezilmesi 66,8, dış yanak burulması (b t³/3; kök momenti yanaktan tabana) 0,726, iç kol 3,70, iç yatak göbeği halkası 0,584, taban cıvata grubu (düzlem içi moment + çekme) 4,04, perde ezilmesi 2,14, **perde sandviçi çekirdek kesmesi 0,244**.
* **Yataklar (tedarik gereksinimi):** iç 1,45 kN, dış 2,15 kN nihai statik yük sayısı (T-BRG-IN / T-BRG-OUT).
* **Sıcaklık (PK2-09):** düğüm silindir kafaları bölgesindedir; T-NODE-TEMP satırı metal satırlarının gerektirdiği dayanım oranını raporlar (0,63); 7075'in düğüm sıcaklığındaki dayanımı açık konudur (§9).

### 3.4 Paraşüt kayışı: sırt omurga kanalı (S1-04, VS2-01)

Kayışın x bileşeni çerçevelere düzlem dışı yük veriyordu ve onu taşıyan boyuna eleman yoktu. Yük yolu: iki U-kulak bağlantısı (F-RISER-FWD / F-RISER-AFT) FS1810 ile arka kiriş çerçevesi arasındaki **sırt omurga kanalının (M-SPINE)** iki ucundadır. Kanal 4 kat PW (0,8 mm), U 44 × 24 mm, 60 mm flanşlı; iki ayağın karşıt x bileşenleri kanalda çubuk kuvvetidir (kayış üçgenini kapatır), net x bileşeni kanal flanşlarına 25 mm aralıkla vidalanan sabit görev bölmesi üst kaplamasıyla (P-MB-UPPER) kesme olarak kenar uzun kirişlerine geçer. x bileşeni ve kilit piminin taban üstündeki yüksekliğinden doğan **bağlantı momenti** 4 × M4 12.9 (eksen z) ile kanalın 16 katlı taban takviyesine, somunlar takviyenin altındaki 48 × 40 × 3 mm 7075 pul levhasında; z bileşeni 2 × M5 12.9 (eksen x) ile çerçevenin 32 katlı (6,4 mm) dolu bandına girer. Düzeltme turu 2'de (VS2-01) kilit pimi Ø8 Ti-6Al-4V'ye, kayış ucu 4130 makaraya (OD 12 × 5 mm, tanımlı ip sonlandırması) çevrildi ve çatal sıkı yapıldı (kulaklar 2 × 6 mm, boşluklar 0,5 mm); pim eğilmesi Melcon-Hoblit koluyla denetlenir.

Sonuçlar (MS): kilit pimi kesme 3,00, **eğilme 0,298**, makara ezilmesi 1,11, kulak ezilmesi 2,55, kulak 3,38, **çerçeve cıvataları 0,380**, çerçeve bandı ezilmesi 0,699, çerçeve gövdesi 0,852, taban cıvataları (kesme + moment çekmesi) 0,909, pul levhası çevresinde sökülme 0,782, kanal takviyesi ezilmesi 0,570, kanal eksenel 1,19, kolon 2,62, yerel 1,17, kaplama vidaları 5,78 / ezilme 4,87, kaplama kesmesi 4,41.

### 3.5 Çerçeveler ve yangın perdesi (S1-06, VS2-03, VS2-06)

* **FS-GEAR ve FS-RS:** takım kirişi uç tepkisinin (sürükleme çifti M_y / L dahil, VS2-02) girdiği yerde çerçeve gövdesi kesmesi, kesme kıvrılması ve bağlantı ezilmesi (FS-GEAR 0,297 / 0,278 / 2,07; FS-RS 0,251 / 0,232 / 1,96), takım yan yükünün kuyu tavanına girdiği 4 × M6 cıvatada bant ezilmesi 23,2.
* **FS-MS / FS-RS halkaları:** kutu dışında, gövde yanı bağlantısında gövde ataleti kesmesi (1,61 / 1,96; FS-MS'de yüz başına bir ±45 kat).
* **Yangın perdesi alt ayakları:** düzlem dışı ayak tepkisi 6,8 mm'lik sandviçte taşınamıyordu. Ayakların çevresine ROHACELL 71 WF çekirdek parçası ve **22 katlı (4,4 mm), r 82 mm** dolu bant kondu; ayak destek levhası 56 × 32 × 5 mm. Merkez yamalı basit mesnetli levha momentiyle (Timoshenko) bant eğilmesi 0,046, yüzler 0,478, **çekirdek kesmesi 0,014**; ayak cıvatalarının bantta ezilmesi 1,49 (E-FOOT-BR). Çekirdek kesmesi bant kenarında düzgün değildir (VS2-06): en yakın kenar desteği ayak yükünün daha büyük payını çeker; gözden geçirenin levha kesme dağılımından tepe / ortalama 1,23 alındı (kendi sonlu eleman hesabımız yok). Düzeltme turu 2'nin yeniden kapanışında ayrıca, kenar çemberini kesen perde kesiklerinin payı çevreden düşülür: motor demeti kesiği C-FW-HARN ilk yerinde (z 0,02–0,06) çemberin %5,7'sini kesiyor ve payı −0,008'e düşürüyordu; kesik ayakların altına (z −0,035…0,000) indirildi ve çemberden çıktı. Üst ayaklar köşe bağlantısıyla (F-FW-CORNER) sırt uzun kirişi ekine bağlanır (FW-UPPER-SPLICE 2,89).
* **Ana yukarı kilit:** kanca bağlantısı kuyu tavanının dökme insertlerine 4 × M5; insert sökülmesi MS 0,359 (ilk tasarımda −0,18).
* **FS3738'de omurga desteği (VS2-03):** kuyruk tamponu çarpmasında arka omurganın FS3738'deki 10,20 kN düşey tepkisini artık talaşlı 7075-T651 I kesitli bir alt parça (60 × 20 × 1,6 / 2,0 mm, halka bacakları arasında y ±0,15) taşır: eğilme 0,142, gövde kesmesi 3,40; arka omurga parçanın gövdesine 4 × M5 12.9 ile (kesme 1,36, ezilme 1,27), parçanın uçları halka bacaklarına 3'er × M5 12.9 ile (kesme 2,54, ezilme 2,41) bağlanır.

### 3.6 Diğer elemanlar

* **Orta kutu:** gövdeler = FS-MS / FS-RS çerçeveleri, kutu-çerçeve bağlantıları. **Çevron kırığı (VS2-03):** başlık başına 17,3 kN veter yönü kırılma kuvveti (CT-KINK) her başlıkta bir 7075 kırık bağlantısıyla (2,0 mm levha 40 × 100 mm, 30 mm kulak) orta hat kaburgasının 16 katlı dolu bandına 5 × M6 Ti ile verilir: cıvatalar 1,84, bant ezilmesi 0,286, kaburga bandı kesmesi 0,246, levha 0,238.
* **Kuyruk:** dikey kirişleri ve kök bağlantıları (köşe bağlantısının çatal kulağı), tail_skin kaplamaları, ventral kök kulakları ve arka omurga kirişi (7075 talaşlı U 24 × 40 × 2,5 mm, kaporta yarımları için alt oturma flanşlarıyla; T-AFTKEEL 0,128).
* **İniş takımı:** ana bacak (7075 boru 42 × 3,5 mm; 1,00), mafsal kulağı (e/D < 2: kulak istisnası, Bruhn kesme-ezilme gerçek e/D'de, 1,92; S1-09), mafsal muylusu (eğilme 1,95), **mafsal bağlantısı (VS2-02)**: 9 elastik cıvatalı altı serbestlik dereceli cıvata grubu (5 × M6 12.9 y ekseninde takım kirişinin yapıştırılmış flanşlı ankrajlarına, 4 × M6 12.9 z ekseninde kuyu tavanının kubbe somun plakalarına; 1,07, ezilme 0,868, sökülme 1,67 / 2,84); takım kirişi başlıkları 0,027, duvarı 0,251, çentik köprüsü 9,55, uç cıvataları 4,46; M_x kiriş duvarı ile kuyu tavanı arasında tanımlı bir yoldan (G-BEAM-TORSION 0,801, G-ROOF-TORSION 0,248). **Aşağı kilit (VS2-07):** Ø10 4130 kilit cıvatası sıkı çatalda çift kesmede, r 35 mm (kesme 3,49, eğilme 0,561). Burun bacağı (dış çap 36 mm; 3,29), burun mafsalı (iç yüzdeki 7075 bloklar, kesme-ezilme 11,6), çekme yükü altında burun bacağı 4,91 ve mafsal blokları 38,0 (VS2-12), EMA gereksinimleri, iç kapak menteşeleri. **Burun kapakları (PK2-13):** iki istiridye kapağı tek bir Volz DA 22 (EQ-NDOORACT) orta hattaki bir kol ve iki bağlantı çubuğuyla sürer; takım işletme hızında iki kapağın menteşe momentlerinin toplamı DA 22 anma torkuyla karşılaştırılır (G-NDOOR-DRIVE 0,899).
* **Motor bağlantısı:** kafes çubukları (12,7 × 0,89 mm 4130; kolon 0,924), sönümleyici halkası, karter cıvataları, yangın perdesi ayakları, sönümleyici nihai kapasite gereksinimi (1,80 kN).
* **Gövde:** görev bölmesi tabanının ortası sökülebilir ekipman tepsisine (TR-MISSION) açılan çerçeveli bir kesiktir; iki dış şerit ön gövdenin alt başlığıdır ve kesik kenarlarındaki iki yapıştırılmış şapka takviyeyle (PW 4 kat, 20 × 15 mm) burkulmaya karşı desteklenir (B-MIDFLOOR 0,910, burkulma 0,258). Arka gövdede sırt ve kenar uzun kirişleri, sabit kaplamalar ve vida sıraları.
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

Ana kiriş başlıkları (UD MTM45-1/AS4, kat 0,1397 mm; gövde ve eldivende 40 mm, dış panelde 30 mm genişlik; düzeltme turu 2 sonrası):

| y (m) | 0,00–0,40 | 0,40–0,55 | 0,55–0,70 | 0,70–0,80 | 0,80–0,90 | 0,90–1,00 | 1,00–1,10 | 1,10–1,20 | 1,20–1,30 |
|---|---|---|---|---|---|---|---|---|---|
| kat | 58 | 33 | 12 | 36 | 35 | 33 | 30 | 28 | 25 |

| y (m) | 1,30–1,40 | 1,40–1,50 | 1,50–1,60 | 1,60–1,70 | 1,70–1,80 | 1,80–1,90 | 1,90–2,00 | 2,00–3,60 |
|---|---|---|---|---|---|---|---|---|
| kat | 23 | 21 | 20 | 17 | 15 | 13 | 12 | 10 (en az, 40 mm² ≈ boyutlandırma modelinin alt sınırı) |

Ana kiriş gövdesi (±45 PW): y 0,70–1,30 6 kat, y 1,30–1,95 5 kat, y 1,95–2,70 4 kat, y 2,70–3,60 3 kat. Arka kiriş gövdesi 3 kat. Kesin değerler `structures.sizing.wing.main_cap.zones` ve `main_web.zones` alanlarındadır. 0,55–0,70 bölgesinin ince kalması §3.1'deki dil momentinden gelir.

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
| M-MIDFLOOR görev bölmesi tabanı | rib_panel düz panel + orta hat şapka takviyesi | ortada TR-MISSION tepsi kesiği (y ±0,125), kesik kenarlarında iki yapıştırılmış şapka takviye | ekipmana alttan erişim (VPK-03); dış şeritler ön gövdenin alt başlığı |
| M-FWDDECK ön hücre tabanı | — | yük tepsisi rayları (6061-T6 T 20 × 20 × 2,5 mm, y ±0,10) güverte takviyesi | güverte açıklığı yarıya iner |
| M-DECK-NOSE aviyonik güvertesi | 6,8 mm sandviç | omurga yarığının üstünde 6 katlı (1,2 mm) dolu şerit, iki yanda sandviç | toplanmış burun tekerine 12 mm açıklık (C05, düzeltme turu 2) |
| F-SPINDLE-NODE (düğüm) | FS3738 halkasına 6 × M5 ile bağlı yatak yuvası | yangın perdesine 7 × M5 ile bağlı işlenmiş 7075 düğüm, iki yanak | eski yuvanın bağlanacağı gövde yoktu (VPK-01) |
| Stabilatör mili / kök yuvası | Ti-6Al-4V 25 × 2,0 mm / kök yuvası ayrı denetlenmiyordu | Ti-6Al-4V 25 × 1,2 mm, kama dış uçta / 7075 kovan 29 × 2 mm, 0,10 m (T-SOCKET-*) | mil hafifletildi, kama kökü ayrı satırla (VS2-08) |
| M-SPINE sırt omurga kanalı | kaplamanın parçası (yırtılır örtü) | yapısal kanal 4 kat PW, U 44 × 24, 60 mm flanş | kayış x bileşeninin yük yolu (S1-04) |
| Yangın perdesi alt ayak bölgesi | düz sandviç | 71 WF çekirdek parçası + 22 katlı r 82 mm dolu bant | düzlem dışı ayak tepkisi (S1-06, VS2-06) |
| FS3738 alt parçası | 2,0 mm 2024-T3 U halka parçası (yer tutucu 0,10 kg) | talaşlı 7075 I 60 × 20 × 1,6 / 2,0 mm | omurga destek tepkisi (VS2-03) |
| Yangın perdesi paslanmaz katmanı | 0,5 mm AISI 304, 7,5 mm hava boşluğu, kütle model payında | 0,4 mm AISI 304 (≥ 0,38 mm, testsiz yangın dayanımı), 7,6 mm hava boşluğu, kütlesi aşağıdan yukarı | kütle kapanışı (düzeltme turu 2, §7) |

---

## 5. Düzeltme turu 1: yapı bulguları

Her bulgu önce yeniden üretildi, sonra düzeltildi; hiçbir kontrol, test ya da gereksinim gevşetilmedi. Sonuç sütunu o turun değerlerini verir; güncel değerler §3'tedir.

| Bulgu | Yeniden üretim | Düzeltme | Sonuç |
|---|---|---|---|
| S1-01 atalet rahatlatması yanlış yerde | doğrulandı: orta kutu başlıkları ve dil dış kanata yayılmıştı | `wing_inertia` her terimi yerinde dağıtır (§2.1); başlıklar yeni momentle yeniden boyutlandırıldı | kök M 5162 → 5293 N m, birleşim M 3092 → 3195 N m; bütün W-* payları ≥ 0 (en küçük 0,002) |
| S1-02 düzlem içi momentin yolu yok | doğrulandı | arka kirişte düşey pimli kulak + yuva bağlantısı; PHAA / NHAA / VD-DRAG durumları; pim bileşke yükü, dil iki eksenli gerilme, kulak ve arka pim satırları | J-* en küçük 0,139 |
| S1-03 yapıştırma yöntemi ve katsayısı geçersiz | doğrulandı | yapıştırmalı metal birleşim kaldırıldı, pimli kompozit birleşim (§4.3) | J-BOND satırları yok; yük yolu tümüyle pimli |
| S1-04 kayış x bileşeni çerçeve dışına | doğrulandı | sırt omurga kanalı M-SPINE + vidalı görev bölmesi üst kaplaması (§3.4) | P-* en küçük 0,38 |
| S1-05 UD başlıkta yarı izotrop ezilme | doğrulandı | başlıklardan bağlantı elemanı kaldırıldı; UD malzemeden ezilme değeri silindi; burçlu delikler ±45 bloklarda, OHC ile | I-UD-BEARING arayüz denetimi |
| S1-06 yoğun yük alan çerçeveler denetlenmemiş | doğrulandı | 14 satırlık "çerçeveler" grubu (FS-GEAR, FS-RS, FS-MS/RS halkaları, yangın perdesi ayakları, köşe eki), yukarı kilit, düğüm taban cıvataları | en küçük 0,002 (FW-FOOT-CORE); FS3738 omurga desteği açık konu (turu 2'de kapandı) |
| S1-07 açık konuda eski kırık kuvveti | doğrulandı | açık konu metni CT-KINK değerinden üretilir | CT-KINK değeri her yerde aynı |
| S1-08 burun bacağı zarfı | doğrulandı | `landing_gear.nose.leg_frontal_width` 0,035 → 0,036 m (üretici `NOSE_LEG_WIDTH`) | sizing ve layout_check takım zarfları yeni genişlikte yeşil |
| S1-09 kulak kenar mesafeleri | doğrulandı | kayış U-kulağı e ≈ 2 D; mafsal ve burun kulakları kulak istisnası olarak Bruhn kesme-ezilmesiyle gerçek e/D'de | G-TRUN-BR 2,11, P-LUG-BR 1,75 |
| S1-10 `layout_check --check` dosya yazıyor | doğrulandı | `--check` salt okunur; çıktılarda çalışma süresi yok | testle denetlenir (dosya sağlamaları değişmez) |

---

## 6. Düzeltme turu 2: yapı bulguları

İkinci bağımsız doğrulamanın yapı bulguları (VS2-01 … VS2-12) önce yeniden üretildi, sonra düzeltildi. Hiçbir kontrol, test ya da gereksinim gevşetilmedi; her düzeltme yeni bir emniyet payı satırı ya da sıkılaştırılmış bir hesap olarak `structures.py`'de durur ve her yeni denetimin kasıtlı olarak bozulmuş bir girdide hatayı yakaladığı testle gösterilir. Bu turun ek kütlesi ilk kapanışta kütle bütçesini aşırdı (boş kütle 102,46 kg; R-02b < 9,5 h); bütçe yeni yapı ve yerleşim kararlarıyla yeniden kapatıldı (§7).

| Bulgu | Yeniden üretim | Düzeltme | Sonuç (MS) |
|---|---|---|---|
| VS2-01 kayış kilit pimi yalnız kesmede denetlenmiş; bağlantı momenti taşınmıyor | doğrulandı: Ø6 4130 pim eğilmede MS −0,37 … −0,67; Fz × kolun momenti hiçbir satırda yok | Ti-6Al-4V Ø8 pim, 4130 kayış makarası (ip sonlandırması tanımlı), sıkı çatal (kulak 6 mm, boşluk 0,5 mm); P-SHACKLE-BEND (Melcon-Hoblit kolu, plastik kazanç yok); bağlantı momenti 4 × M4 12.9 taban cıvatasına çekme + 48 × 40 × 3 mm 7075 pul levhasıyla sökülme (P-SPINE-BOLTS, P-SPINE-PULL); layout_check C13 güncel bağlantı modelinde ve "ilk kesim" olarak işaretli | P-SHACKLE-BEND 0,298, P-SPINE-BOLTS 0,909, P-SPINE-PULL 0,782, P-SPOOL-BR 1,11 |
| VS2-02 mafsal bağlantısı cıvataları, takım kirişi ve çerçeve uçları bacak momentlerini görmüyor | doğrulandı: sürükleme çifti M_y ve M_x hesapta yok | F-TRUNNION 9 elastik cıvatalı (5 × M6 12.9 y ekseninde kiriş ankrajlarına, 4 × M6 12.9 z ekseninde kuyu tavanının sızdırmaz kubbe somun plakalarına) altı serbestlik dereceli cıvata grubu; kesme / çekme etkileşimi ve CFRP bandın sökülmesi; kiriş ve çerçeve uç tepkilerine M_y / L çifti; M_x kiriş duvarı ile kuyu tavanı arasında tanımlı bir yoldan (G-BEAM-TORSION, G-ROOF-TORSION; tavana her yüzde bir kat 0/90 takviye); cıvata tanımı kod ve yerleşimde aynı (M6 12.9) | G-FIT-BOLTS 1,07, G-FIT-BR 0,868, G-BEAM-CAP 0,027, G-BEAM-WEB 0,251, FR-GEAR-WEB 0,297, FR-GEAR-CRIMP 0,278, G-ROOF-TORSION 0,248 |
| VS2-03 kuyruk tamponu yük yolu kapanmıyor; kırık bağlantısı boyutlandırılmamış | doğrulandı: FS3738'in omurga tepkisi boyutlandırılmamış, kırık bağlantısı 0,10 kg yer tutucu | FS3738 U halkasının alt parçası talaşlı 7075-T651 I kesit (60 × 20 × 1,6 / 2,0 mm, y ±0,15 arası), arka omurga 4 × M5 12.9 ile gövdesine, uçları 3 × M5 12.9 ile halka bacaklarına; çevron kırığı için başlık başına bir 7075 bağlantı (2,0 mm levha 40 × 100 mm, 30 mm kulak, 5 × M6 Ti orta hat kaburgasının 16 katlı bandına); kütleler `structures.sizing.mass`'ta | T-AFTKEEL 0,128, FR-3738-SEG 0,142, FR-3738-SEG-SH 3,40, CT-KINK-BR 0,286, CT-KINK-RIB 0,246, CT-KINK-PLATE 0,238 |
| VS2-04 dil başlığından dış panel başlıklarına derinlik geçişi tanımsız | doğrulandı: başlık başına yaklaşık 16 mm basamak | geçiş kök bölmesinde 0,10 m üzerinde doğrusal rampa (`structures.sizing.wing_joint.transition`); iki uçtaki kırılma kuvvetleri dil gövdesinde ezilme, başlık-gövde ara yüzey kesmesi ve birleşim kaburgasının 20 katlı bandı olarak; azalan kol boyunca başlık ve kaplama birim şekil değiştirmesi; ilk 0,12 m'de dış yüze bir kat 0/90 kaplama takviyesi | J-TRANS-WEB 1,79, J-TRANS-ILSS 1,04, J-TRANS-RIB 0,248, J-TRANS-CAP 0,425, J-TRANS-SKIN 0,014 |
| VS2-05 atalet rahatlatması büyüme payını sayıyor | doğrulandı: rahatlatma büyüme paylı kalemlerden | rahatlatma en küçük inandırıcı kanat kütlesiyle: kalemlerin büyüme payı ve 1,05 model katsayısı olmayan taban kütleleri (§2.1); başlık bölgeleri yeniden boyutlandırıldı | en küçük W-CAP 0,0002 (W-CAP-07) |
| VS2-06 ayak bandı çekirdek kesmesi düzgün varsayılmış | doğrulandı | tepe / ortalama 1,23 (gözden geçirenin levha kesme dağılımı; kendi sonlu eleman hesabımız yok); bant yarıçapı 66 → 82 mm, 22 kat; yeniden kapanışta ayrıca kenar çemberini kesen perde kesiklerinin çevreden düşülmesi eklendi ve motor demeti kesiği C-FW-HARN ayakların altına indirildi (kesik çemberin %5,7'sini kesiyordu: MS −0,008) | FW-FOOT-CORE 0,014, FW-FOOT-LAND 0,046, FW-FOOT-FACE 0,478 |
| VS2-07 aşağı kilit pimi eğilmede denetlenmemiş | doğrulandı: Ø8 dalıcı pim kolu 4–6 mm'de MS −0,3 … −0,6 | kilit, 4130 Ø10 kilit cıvatası olarak sıkı çatalda çift kesmede (kulaklar 2 × 6 mm, boşluk ≤ 0,5 mm), 35 mm yarıçapta; takım tedarikçisine gereksinim | G-DOWNLOCK 3,49, G-DOWNLOCK-BEND 0,561 |
| VS2-08 mil kama kesiti yalnız burulmada | doğrulandı | kama milin dış ucunda, yuva geçmesinin son 20 mm'sinde (yuva momentinin yaklaşık %10'u); kama kökünde eğilme + burulma | T-SPINDLE-SPL 1,31, T-SPINDLE-SPL-MT 0,975 |
| VS2-09 `lug_axial`'e Fbru verilmiş | doğrulandı | iki çağrıda Ftu (Bruhn D1.5) | J-REAR-LUG 6,68, P-LUG 3,38 |
| VS2-10 eski / çelişkili metinler | doğrulandı | dil ve çatal metinleri DESIGN'dan (25 kat 5,0 mm; 10 kat 2,0 mm); stabilatör mili kaynak metni ve bölge metni düğüm bağlantısını anar; cıvatalar her yerde M6 12.9; C13 güncel bağlantı modelleriyle ve "ilk kesim çapraz denetimi" etiketiyle | metin denetimi |
| VS2-11 UL13.1.2 sınırlarının uygulanabilirlik koşulu denetlenmemiş | doğrulandı | DT-COND satırı: matrise duyarlı özelliklerin ETW kaybı (UD Xc, PW Xc / Yc, PW S_0,2; NCAMP aynı yön, B-tabanı varsa B, yoksa ortalama) | en büyük kayıp %47 < %50 → UL13.1.2 kullanılır (§1.2) |
| VS2-12 yer taşıma durumları yok | doğrulandı | çekme: burun çatalı dingilinde 0,3 W (CS-23.509 / FAR 23.509, tasarım kararı; CS-LUAS / CS-VLA metni araştırma dosyalarında yok); bağlama ve kriko noktası yok (taşıma beşiği; açık konu) | G-TOW-NLEG 4,91, G-TOW-PIVOT 38,0 |

Paketleme bulgularından yapıyı etkileyenler de bu turda hesaplandı: tek DA 22'nin iki burun kapağını sürmesi (G-NDOOR-DRIVE 0,899; PK2-13) ve stabilatör düğümünün sıcak bölgedeki dayanım oranı (T-NODE-TEMP; PK2-09). Kütle kapanışı için yapılan değişikliklerin yapı satırları: dil başlıklarının pimler arasında incelmesi (J-TONGUE-TAPER 0,125, kat düşürme eğimi J-TONGUE-TAPER-DROP 0,313), kırık bağlantısı levhası 2,5 → 2,0 mm (CT-KINK-PLATE 0,238), FS3738 alt parçası başlıkları 2,0 → 1,6 mm (FR-3738-SEG 0,142), düğüm iç kolu 4 → 3 mm (T-NODE-INBOARD 3,70) ve ayak bandı 24 → 22 kat, r 82 mm (FW-FOOT-LAND 0,046). Stabilatör mili duvarını 1,0 mm'ye inceltmek denendi ve geri alındı: kama dişlerinin altında 0,5 mm duvar kalıyordu.

---

## 6a. Düzeltme turu 3: yapı bulguları

Üçüncü bağımsız doğrulamanın yapı bulguları (VS3-01 … VS3-06) ve yapıyı ilgilendiren paketleme bulguları (PK3-06, PK3-07) önce yeniden üretildi, sonra düzeltildi. Hiçbir kontrol, test ya da gereksinim gevşetilmedi.

| Bulgu | Yeniden üretim | Düzeltme | Sonuç (MS) |
|---|---|---|---|
| VS3-01 eldiven ana başlıkları: hesap yerleşimde olmayan bir moment kolu kullanıyor; gövde yanı kaburgasında 9 mm başlık basamağı modellenmemiş | doğrulandı: yerleşim başlıkları ±35 mm'de (kutu yüksekliği), hesap eldiven loftunun altında; M-SOB'de süreksiz basamak | **tek geometri:** gövde içinde başlık yüzleri kutu kapağı seviyesinde, gövde yanı kaburgasından (y 0,40) iç pime (y 0,463) doğrusal **başlık rampası**, dışında eldiven loftu; başlık ağırlık merkezi = yüz − 1 mm dolu kaplama − t/2. Yerleşim (M-CTBOX `main_spar_caps_z / _t`, ply bölgelerinin her kırılma noktasında) ve kesit modeli (`wing_section`) aynı geometriyi kullanır; yerleşim bir başlığı birleşik dış yüzeye **gerçek mesafeyle** 1 mm kaplama bırakmak için içeri aldıysa kesit modeli de o (daha içteki) değeri kullanır (I-CAPZ: ≤ 0,5 mm, model başlığı yerleşimden dışarıda değil). Rampa uçlarındaki kırılma kuvvetleri F·sinθ: kaburga dolu bandı (12 kat, 50 mm), rampa altında konik ROHACELL 71 WF dolgu, kaburga yüzünde 1 kat ±45 takviye bandı; çatal dişleri tam derinlikte (8 kat), panel uzunluğuna göre sonlu levha kesme burkulması; ana başlık bölgeleri sıklaştırıldı (gövde 0,05 m, eldiven ve dış panel 0,05 m adımlı) | W-SOB-KINK-RIB 0,277, W-SOB-KINK-PRONG 9,73, W-SOB-KINK-ILSS 2,47, W-SOB-OFFSET 0,240, J-PRONG-WEB 0,260, J-PRONG-BUSH 0,195; W-CAP-* en küçük 0,000 (W-CAP-24, kat sayısı tam sınırda boyutlandırıldı) |
| VS3-02 esinti zarfı 4500 m'de duruyor, uçak 7114 m'ye çıkabiliyor | doğrulandı: esinti matrisi 0 / 3000 / 4500 m; MTOM'da servis tavanı 7114 m | **azami işletme irtifası 4500 m** (`structures.operating_limits.max_operating_altitude_m`) ve yeni gereksinim **R-61**: uçuş kontrol sisteminin irtifa zarfı koruması / coğrafi sınırı uçağı bu irtifanın altında tutar; sınır esinti matrisinin en üst irtifasını aşamaz (pay ≥ 0). Servis tavanı sınırın üstünde olduğundan koruma zorunlu bir işlevdir (R-54 hız korumasıyla aynı) | sizing R-61: pay 0 m (sınır = matrisin üstü) |
| VS3-03 orta hat kaburgası ve kırık bağlantıları yalnız metinde | doğrulandı | yerleşimde **M-CLRIB** (YK250-CH-055; rib_panel, 16 katlı dolu bantlar 100 × 30 mm, kapaklara 20 mm flanşlar, uç köşebentleri) ve **F-KINK-UP / -LO** (YK250-CH-056 / 057) nesneleri; kaburga kütlesi B grubunda (0,067 kg); kapakların kırılma kuvveti CT-KINK-COVER (2,90 kN) CT-KINK-BR / -RIB'e eklendi; kaburga gövdesi her yüzünde 1 kat takviye (CT-KINK-RIBWEB) | CT-KINK-BR 0,101, CT-KINK-RIB 0,067, CT-KINK-RIBWEB 0,434 |
| VS3-04 eldiven LERX üst kaplamaları hiç denetlenmemiş | doğrulandı: kendi panel modelimizle y 0,41'de burkulma MS −0,28 | LERX üst ve alt kaplamaları için burkulma, kıvrılma ve hasar toleransı satırları (W-SKINBUCK/-CRIMP/-DT-LERX-UP/-LO); LERX kesiti eldiven kutusu momentinde en dış lif sayılır; ilk LERX bölmesinde 7 mm çekirdek (`layups.lerx_skin_upper_root`, +0,008 kg) | W-SKINBUCK-LERX-UP 0,109, W-SKINCRIMP-LERX-UP 0,112, W-SKINDT-LERX-UP 0,016, W-SKINBUCK-LERX-LO 0,210 |
| VS3-05 çekirdek izin verilen değerleri oda sıcaklığı, yüzeyler ETW | doğrulandı: malzeme verisinde sıcaklık yok | veri uydurulmadı: açık konu olarak hesaplanır — çekirdeğin belirlediği satırların her birinin karşılayabildiği çekirdek özellik koruma oranı (≈ 1 / (1 + MS)) `open_items` içinde listelenir; Evonik WF sıcaklık eğrileri ya da azami sandviç sıcaklığı sınırı ayrıntılı tasarımdan önce gerekir (§9) | açık konu (sayısal liste `out/structures.md`) |
| VS3-06 iniş atalet hedefi (4,0) sessizce aşılıyor; iki noktalı inişin yunuslama momenti taşınmıyor | doğrulandı: n_j 5,40, n 6,07; θ̈ ≈ 26 rad/s² | kullanılmayan hedef kaldırıldı (`structures.landing.n_inertia_basis`: tasarım hesaplanan n_j / n ile yapılır, 4,0 hedefi 80 mm stroklu takımla karşılanmaz); düz iki noktalı inişte ana takım tepkisinin AM gerisinden yarattığı yunuslama ivmesi (I_yy 114 kg m², aerodinamik sönüm yok) motor bağlantısının iniş durumuna eklendi; G-LAND-ROT x 3,80'de n = 9,21 verir | E-* değişmedi (esinti + MCP durumu belirleyici) |
| PK3-06 taşıma kızağı yastıkları kapaklara basıyor | doğrulandı | bölünmüş yastıklar (iki × istasyon, sabit kaplamada, `layout.chassis.ground_handling.cradle_pads`) ile TR-PAD / TR-FRAME yeniden hesaplandı | TR-PAD 0,195, TR-FRAME 15,8 |
| PK3-07 mafsal kapağı tahrik yasası | doğrulandı | mafsal kapağı bacakla tahrik edilir (yarıklı bağlantı + açma yayı; G-TDOOR-LINK gerekli kapatma momenti 0,26 N m); DA 22 yalnız iç kapağı sürer. **G-DOOR-DRIVE** (yeni gereksinim satırı): \|Cp\| 1,0 basınç üst sınırında iç kapak 95° strok boyunca 4,95 N m nihai moment ister (8,2 J iş); DA 22 anma torku 1,8 N m ile 180°'de 5,7 J verir — hiçbir bağlantı oranı bunu kapatmaz | **açık konu** (§9): gövde altı basıncı ya da DA 26 sınıfı tahrik |

**Yeniden kapanış.** Bu turun yapı değişiklikleri kanat başlıklarını hafifletti (ince bölgeler: ana başlıklar 2,466 → 2,315 kg) ve orta kutuyu ağırlaştırdı (rampa bantları ve dolgu 0,041, orta hat kaburgası 0,067, tam derinlikte çatal dişleri). Grup tavanları yeni tahminlere göre (büyüme payı dahil, 10 g'a yukarı) yeniden dağıtıldı: şasi 12,46 → 12,61 kg, kanat 15,34 → 15,18 kg. Bu turun yerleşim ayrıntıları — uzun kiriş çentiklerinin köşebentleri, cıvataları ve U takviyeleri (14 çentik × ≈ 25 g ≈ 0,35 kg), kenar çizgisi ek cıvataları (8 × M6 Ti ≈ 0,05 kg), taret halkası yan oturma takviyeleri (≈ 0,04 kg), yarık sonu kenar şeridi (≈ 0,005 kg) — çerçeve, taret bölmesi ve bağlantı elemanı **kural paylarının** (`mass.rules.frames_kg`, `turret_bay_frame_kg`, donanım grubu) içinde sayıldı; bu paylar aşağıdan yukarı değildir ve ayrıntılı tasarımda doğrulanmalıdır (R-56 payı küçüktür, §7).

## 7. Kütle ve bütçe

Boyutlandırılan elemanların aşağıdan yukarı kütlesi kavramsal kütle modelinin paylarıyla karşılaştırıldı (büyüme payı öncesi, kg):

| Grup | Aşağıdan yukarı | Model payı | Fark |
|---|---|---|---|
| A — kanat birincil yapısı (ana başlıklar 2,315, arka başlıklar 0,171, gövdeler 0,511 + 0,174, birleşim donanımı çifti 1,111: incelen dil 0,513 / adet, arka kulak 0,042 / adet; üst kutu kaplaması çekirdeği 0,045, kök bölmesi kaplama takviyeleri 0,039, ilk LERX bölmesi 7 mm çekirdeği 0,008) | 4,372 | 5,450 | −1,077 |
| B — orta kutu kapakları 0,686, gövde takviyeleri 0,034, kompozit çatallar (tam derinlikte dişler) 0,759, ana pimler (4) 0,261, arka pimler + yuva bağlantıları 0,111, kırık bağlantıları (2) 0,153, başlık rampası bantları ve dolgu 0,041, orta hat kaburgası 0,067 | 2,112 | 1,600 | +0,512 |
| C — şasi eklemeleri (yakıt tabanları 0,165, M-WELLKEEL 0,093, taban takviyeleri 0,061, arka omurga 0,055, yangın perdesi ayak bantları 0,212, çekirdek parçası 0,017, FS3738 alt parçası 0,072, kuyu tavanı takviyeleri 0,018, aviyonik güvertesi şeridi 0,012) | 0,706 | 0 | +0,706 |
| D — stabilatör milleri (0,123 / adet), düğüm bağlantıları (0,258 / adet: taban 0,086, iç kol 0,040, dış yanak 0,132), yataklar, kök yuvaları + çapraz cıvata (0,057 / adet) | 0,956 | 1,100 | −0,144 |
| E — burun mafsalı bağlantısı 0,184, sırt omurga kanalı 0,253 + takviyeler 0,015, kayış bağlantıları 0,097, kilit pimleri + makaralar + pul levhaları 0,055, çerçeve bantları 0,026 | 0,630 | 0,650 | −0,020 |
| **Toplam** | | | **−0,024** (büyüme payıyla −0,025) |

Düğüm bağlantısının kütlesi zarf hacminin doluluk oranlarıyla (taban 0,55, dış yanak 0,70) ve iç kol / göbek geometrisinden hesaplanan bir tahmindir. Bu değerler `structures.sizing.mass` bloğu olarak `spec.yaml`'a yazılır ve `analysis/sizing.py` bunları kavramsal modelin yerine okur (`wing_structure`, `mass_items`: orta kutu, çerçeve eklemeleri, burun mafsalı, paraşüt bağlantıları; `tail_structure`).

**Düzeltme turu 2'nin kütle kapanışı.** Turun ilk eklemeleri (Ø8 Ti kilit pimleri ve makaralar, mafsal bağlantısı cıvata grubu, FS3738 alt parçası ve kırık bağlantıları, 85 mm ayak bandı, dört burun kapağı eyleyicisi, motor bölmesi ısı koruması) boş kütleyi 102,46 kg'a çıkardı ve R-02b karşılanmadı. Bütçe şu kararlarla yeniden kapatıldı (hepsi bir yapı satırıyla ya da yerleşim denetimiyle doğrulanır):

* **Dil başlıkları pimler arasında incelir** (10 → 3 mm, 1:20'den yatık; §3.2): dil momenti iç pimde sıfırdır.
* **Burun kapakları tek DA 22 ile** (PK2-13): iki kapak orta hattaki bir kol ve iki bağlantı çubuğuyla bağlıdır; kapak eyleyicisi sayısı 4 → 3, kol ve çubuklar 0,05 kg (`mass.rules.gear_doors.nose_door_drive`); G-NDOOR-DRIVE ve yerleşimde NDOOR-LINKAGE zarfı (C04 / C05).
* **Yangın perdesi ve ısı koruması aşağıdan yukarı:** motor soğutma / yangın koruma kalemi (`cooling_baffles_firewall_cowl_flap`) artık üç parçanın toplamıdır: yangın perdesi paslanmaz katmanı (0,705 kg; 0,4 mm sac, kenar köşebendi, ayaklar, perçinler; `layout.chassis.engine_mount.firewall_stackup.mass`), kompozit soğutma payı (temel 1 m² × 1 kg/m² eksi kabukta sayılan kaporta kaplaması 0,376 m² = 0,624 kg; `mass.rules.cooling_split`) ve ısı koruması (0,388 kg net: paslanmaz ek parça ve kalkanlar, düğüm perdeleri, metal kaporta parçaları; `layout.heat_protection.mass`). Kalem 1,500 → 1,718 kg. Önceki 1,5 kg'lık pay, kaportayı kabuk kaleminde ikinci kez sayıyordu.
* **Küçük kesitler:** kırık bağlantısı levhası 2,0 mm, FS3738 başlıkları 1,6 mm, düğüm iç kolu 3 mm, ayak bandı 22 kat / r 82 mm, aviyonik güvertesinin yarık şeridi 6 kat, perde kenar köşebendi 13 mm kollar, P-COWL-UPS 0,8 mm alüminyum.

Kapanış yeniden yapıldı: boş kütle **101,630 kg** (MTOM 149,9 kg sabit), kanat kök x_c4 2,507 → 2,487 m, dayanım 10,32 h (R-02), azami faydalı yükle 9,54 h (R-02b); 61 gereksinimin 61'i karşılanır.

Grup tavanları yeni tahminlere (büyüme payı dahil, 10 g'a yukarı yuvarlanmış) göre yeniden dağıtıldı: şasi 12,38 → 12,46 kg (12,459), kumandalar 8,31 → 8,29 kg (8,281), iniş takımı 14,34 → 14,57 kg (14,565), itki 14,31 → 14,54 kg (14,532), kuyruk 7,60 → 7,35 kg (7,341), kanat 15,46 → 15,34 kg (15,340). Tavanların toplamı 101,53 → 101,68 kg'dır; R-56 payı (R-02b'nin tam karşılandığı boş kütle 101,743 kg − yedek 0,05 kg − tavanlar toplamı) **+0,013 kg** olarak pozitif kalır. Pay çok küçüktür: ayrıntılı tasarımda her kütle artışı başka bir kalemden karşılanmalıdır.

---

## 8. Arayüzler

`structures --check` şunları da denetler: `spec.structures.sizing` yeniden üretilen blokla birebir aynı mı, `spec.layups` içindeki yapı evresi dizilimleri güncel mi ve yerleşimdeki elemanlar yapı ölçülerini taşıyor mu. 15 arayüz denetimi vardır: I-AFTKEEL (malzeme / kesit), I-WELLKEEL, I-WELLROOF (dizilim), I-CTBOX (kapak dizilimi), I-MIDFLOOR (iki kesik kenarı takviyesi ve tepsi kesiği), **I-NODE** (düğümün kök parçası cıvataları, perde cıvataları, iç yatak silindiri ve dış yatak istasyonu), **I-TONGUE** ve **I-FORK** (kompozit dil ve çatal, flanş / gövde / takviye kalınlıkları; dil metni iç pimdeki inceltilmiş kalınlığı da anmalıdır), **I-SPINE** (kanal kalınlığı = kat sayısı × kat kalınlığı), **I-RISER** (kilit pimi çapı ve kulak kalınlığı), **I-UD-BEARING** (UD malzemede ezilme değeri yok) ve dört katman dizilimi. Yerleşim tarafında `layout_check` düğümün, köşe bağlantısının ve bütün bağlantı parçalarının cıvata desenlerini (kenar ≥ 2 D metal / 2,5 D kompozit, aralık ≥ 3 D), yapı–yapı çakışmalarını ve pim / rayba yollarını denetler ([03](03_yerlesim_ve_yapi_konsepti.md) §11).

---

## 9. Sınırlamalar ve açık konular

* **Çırpınma, ıraksama, kanatçık tersinmesi analiz edilmedi.** Kanat ve kuyruğun rijitlik / kütle modeli henüz yok. İlk uçuştan önce yer titreşim testi (GVT) ve çırpınma analizi gerekir (CS-LUAS.629). ±45 ağırlıklı kaplamalar burulma rijitliğini artırır; stabilatörün kütle dengesi ve mil-yatak boşluğu ayrıca incelenmelidir.
* **Kompozit birleşim:** burçların dil ve kulak laminelerindeki ezilmesi yarı izotrop delikli bası dayanımıyla denetlendi (delik e/D 3'ün altında); dil başlığının iç kat düşürmeleri ve derinlik geçişi rampası el hesabıdır. Tasarım değerleri dondurulmadan önce pimli CFRP dil / çatal eleman testi (statik, ETW, yorulma) gerekir.
* **Yangın perdesi ayak bandı:** çekirdek kesmesinin tepe / ortalama oranı (1,23) gözden geçirenin levha kesme dağılımından alındı; kendi sonlu eleman hesabımız ve kupon testi yok. Pay küçüktür (FW-FOOT-CORE 0,014); perdeye yeni bir kesik ancak ayak bandı çemberinin dışında açılabilir (hesap kesikleri çevreden düşer).
* **Stabilatör düğümünün sıcaklığı:** 7075-T651 düğüm silindir kafaları bölgesindedir; düğüm sıcaklığı ölçülmedi, T-NODE-TEMP gereken dayanım oranını (0,63) raporlar. Motor yer çalıştırmasında sıcaklık ölçümü gerekir.
* **Yorulma ve hasar toleransı** yalnızca STANAG UL13.1.2 birim şekil değiştirme sınırlarıyla temsil edildi (uygulanabilirlik koşulu DT-COND ile sağlandı). Metal parçaların (pimler, burçlar, arka kulak, düğüm ve takım bağlantıları, motor kafesi kaynakları) yorulma ömrü ve kompozit BVID/CVID kanıtı (CS-LUAS.572/573) test ister.
* **El hesabı varsayımları:** basit mesnetli paneller, düzlem kesitler, D-burnu ve firar kenarı kaplamaları eğilmede sayılmadı, üretici minimum çekirdek değerleri, çubuk parçalar için sac izin verilen değerleri (Ti-6Al-4V, 7075-T6). Kanat kutusu, birleşim, orta kutu, düğüm ve takım bağlantıları için sonlu eleman doğrulaması ve kupon / eleman testleri (sandviç kıvrılma ve buruşma, CFRP burç ezilmesi, dökme insertler) gerekir.
* **Tedarik gereksinimleri (MS'siz satırlar):** 61805-ZZ yatakların statik yük sayısı (T-BRG-IN / T-BRG-OUT), taret asansörü bilyalı vida ve ray taşıyıcıları, motor sönümleyicisi nihai kapasitesi, ana / burun takımı EMA torku (14 / 26 N m), burun aşağı kilidi. Katalog değerleri araştırma verisinde yoktur.
* **Ana iç takım kapakları:** DA 22 kapalı kapağı V_D emmesine karşı tutamaz; ölü nokta bağlantısı ya da kapak kilidi gerekir (G-DOOR-LOCK). **İşletme torku da yetmez** (G-DOOR-DRIVE, düzeltme turu 3): \|Cp\| 1,0 basınç üst sınırında kapak 95° boyunca 4,95 N m nihai moment (8,2 J iş) ister, DA 22 180°'de 5,7 J verir; hiçbir bağlantı oranı kapatmaz. Ya V_LO'da gövde altı basıncı (HAD / uçuş testi; alt yüzey çoğunlukla basınç tarafıdır) üst sınırın yaklaşık üçte birinin altında gösterilmeli ya da iki iç kapağa DA 26 sınıfı tahrik verilmelidir (+0,28 kg; bugünkü bütçe payı bunu taşımaz). Mafsal kapakları bacakla tahrik edilir (eyleyicisiz).
* **Çekirdek sıcaklığı (VS3-05):** ROHACELL 51 / 71 WF izin verilen değerleri oda sıcaklığı üretici minimumlarıdır, yüzeyler ETW B-tabanıdır. Çekirdeğin belirlediği satırların karşılayabildiği özellik koruma oranları `out/structures.md` açık konularında listelenir (FW-FOOT-CORE ≥ 0,99, W-SKINBUCK-CT ≥ 0,98, W-SKINBUCK-OP-UP-IN ≥ 0,96, W-SKINBUCK-GLOVE-UP ≥ 0,93, W-SEC-CT-7 ≥ 0,91, LERX kaplamaları ≥ 0,90; dayanım satırları özellikle doğrusal, burkulma / buruşma satırları yaklaşık). Evonik sıcaklık eğrileri uygulanmalı ya da azami sandviç sıcaklığı sınırlanmalıdır.
* **Azami işletme irtifası (VS3-02):** kanat esinti zarfı 4500 m'ye kadar hesaplandı; daha yüksekte esinti yük katsayısı büyür ve başlık payları negatife düşer. Uçuş kontrol sisteminin irtifa koruması / coğrafi sınırı zorunlu bir işlevdir (R-61).
* **Taret bölmesi karın kesiği (PK3-09):** bölme duvarlarının alt kenarı kapı bandı boyunca serbesttir; kesik FS1110 / FS1330 ve yapıştırılmış yan oturma takviyeleriyle çerçevelenir. Gövde eğilme modeli alt kaplamayı başlık saymadığından (yalnız uzun kirişler ve tabanlar) hiçbir pay bu kesiğe bağlı değildir; kesik çevresindeki kesme akışı yeniden dağılımı ayrıntılı tasarımda (sonlu eleman) denetlenmelidir.
* **Düzeltme turu 3 yerleşim ayrıntılarının kütlesi:** uzun kiriş çentiği köşebentleri / takviyeleri, ek cıvataları, halka yan takviyeleri ve kenar şeridi (≈ 0,45 kg tahmin) kural paylarının içinde sayıldı; aşağıdan yukarı çerçeve kütlesi ayrıntılı tasarımda bunu doğrulamalıdır.
* **İniş:** 4,0 atalet hedefi 80 mm stroklu takımla karşılanmaz (n 6,07); yaklaşık 0,2 m etkin strok gerekir. Yoğun arka kütleler yunuslama ivmesiyle birlikte boyutlandırıldı (G-LAND-ROT).
* **Yer taşıma:** bağlama (rüzgâr) ve kriko noktaları tasarlanmadı; uçak açıkta park edilecekse eklenmelidir. Çekme yükü CS-23.509 değerinin tasarım kararı olarak alınmasıdır.
* **Kütle tahminleri:** düğüm bağlantısı (doluluk oranları), yatak kütleleri, burun kapağı kolu, yangın perdesi ayakları ve perçinleri, ısı kalkanı ayakları (0,30 kg/m²) ve kompozit soğutma payı (temel 1 kg/m²) tahmindir.
* **Tasarım kararları:** taşıma yük katsayıları (3,0 / 1,5 / 1,5 g), kuyruk tamponu yükü (45°'de 1,0 × MTOM ağırlığı), takım işletme hızı 1,6 V_S, çekme yükü 0,3 W. İşletmeci belirtimi ve test verisiyle değiştirilmelidir.
* **Kütle payı:** R-56 payı +0,021 kg'dır (düzeltme turu 3 sonrası; şasi ve kanat tavanları yeniden dağıtıldı, §6a).
