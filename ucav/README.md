# ucav/ — YELKOVAN YK-38

**YELKOVAN YK-38**, MALE görünümlü, benzinli ve itici motorlu bir **sivil gözetleme ve araştırma İHA'sıdır**.
Bu klasör uçağın tamamını kodla tanımlar. Tasarım sayıları tek bir YAML dosyasındadır. Aynı kaynaktan iki ürün
çıkar: animasyonlu, ayrıntılı bir **Blender modeli** (rig, iniş takımı, kumanda bağlantıları, render, animasyon) ve
gerçek et kalınlıklı, tablaya sığan **FDM baskı parçaları** (STL, Türkçe baskı raporu).

> **Kapsam.** YK-38 sivil bir EO/IR gözetleme ve araştırma platformudur. Tek faydalı yükü çene altındaki EO/IR
> kamera taretidir (SIYI ZT6 sınıfı). Silah, mühimmat, askı noktası, pilon ve bırakma mekanizması **yoktur**. Bunlar
> modele, spec'e ve bu depoya eklenmez. Testler (`tests/test_ucav_core.py`) spec'te bu tür anahtar olmadığını denetler.
> "UCAV görünümü" yalnız biçim dilidir (MALE oranları, chine çizgisi, U-kuyruk); işlev sivildir.

**Bağlantılar:** [spesifikasyon (`spec.yaml`)](spec.yaml) · [boyutlandırma raporu](out/sizing.md) ·
[baskı raporu (256 tabla)](out/print_report.md) · [baskı raporu (220 tabla)](out/print_220x220x250/print_report.md) ·
[sabit görüntüler](out/render/) · [gösterim animasyonu](out/anim/showcase.mp4) ·
[mekanizma döngüsü](out/anim/mechanisms.mp4) · [STL'ler](out/stl/) · [mevzuat özeti](../docs/08-guvenlik-ve-mevzuat.md).
Blender sahnesi `out/yk38.blend` ve `out/yk38.glb` git'e girmez; tek komutla yeniden üretilir (§4).

![YK-38 — pistte, takım açık (Blender Cycles)](out/render/yk38_hero.jpg)

| | |
|---|---|
| ![Arka-sağ 3/4](out/render/yk38_rear34.jpg) | ![Yan (ortografik)](out/render/yk38_side.jpg) |
| ![EO/IR taret](out/render/yk38_nose.jpg) | ![U-kuyruk ve itici pervane](out/render/yk38_tail.jpg) |
| ![Alttan, takım toplu](out/render/yk38_under.jpg) | ![Sol ana takım ve açık kuyu](out/render/yk38_gearbay.jpg) |
| ![Önden](out/render/yk38_front.jpg) | ![Üstten](out/render/yk38_top.jpg) |
| ![Taktik boya (yalnız render)](out/render/yk38_hero_taktik.jpg) | ![Stüdyo](out/render/yk38_hero_studyo.jpg) |

Diğer görüntüler: [taktik arka 3/4](out/render/yk38_rear34_taktik.jpg),
[taktik yan](out/render/yk38_side_taktik.jpg), [stüdyo yan](out/render/yk38_side_studyo.jpg).

## 1. Ana değerler

Bütün sayılar [`spec.yaml`](spec.yaml)'dandır. [`sizing.py`](sizing.py) bunları geometriden yeniden hesaplar
([`out/sizing.md`](out/sizing.md): **156/156 kontrol tolerans içinde**).

| Büyüklük | Değer |
|---|---|
| Sınıf | SHT-İHA **M1 (4–25 kg)**, sivil gözetleme/araştırma, VLOS, izinli saha |
| Açıklık / toplam boy / yükseklik | **3,80 m** / **2,475 m** (gövde 2,205 m, spinner dahil 2,310 m) / **0,668 m** (takım açık; strobe lensiyle 0,6725 m) |
| Kanat | S 0,976 m², AR 14,8, MAC 0,265 m, λ 0,52; SD7062 → SD7032, 4° dihedral, 3° washout, 36° kök glove, 70 mm raked uç |
| Kütle | MTOW **11,71 kg** (boş 10,07 kg; yakıt 1,40 L = 1,04 kg; görev modülü 0,60 kg); kanat yüklemesi 12,0 kg/m² |
| Motor ve pervane | DLE-20 (20 cc, 2 zamanlı benzinli, 1,86 kW) **ters** (silindir aşağıda), Xoar 16×8 **itici**, 25 mm uzatma mili; statik itki 5,57 kgf, T/W 0,48; itki ekseni 5° aşağı |
| Hızlar | Stall 12,0 m/s (temiz) / 10,8 m/s (20° flap); bekleme 15,6; seyir 20; Vne 28 m/s; (L/D)maks 18,4 @ 14,5 m/s |
| Havada kalış | 4,16 h (elektrik sınırı; yakıtla 3,6–4,25 h), menzil 233 km — **kâğıt değer**; operasyon VLOS'tur |
| Kalkış | Düz kalkış, 20° flap: V_LOF 13,2 m/s, koşu 25,5 m (asfalt) / 28 m (çim) |
| Kararlılık | Statik marj %10 MAC (η_h'ye göre %8,7–12,0); V_h 0,47, V_v 0,034; CG s = 1,232 m (aralık 1,219–1,245) |
| Kuyruk | Oklu U-kuyruk: stabilize 1,04 m, NACA 0010 → uçta %12, HK oku 30°; iki dikey 0,32 m, 12° dışa eğik; FK–pervane aralığı pala ucunda 0,128 m (0,32 D), pala süpürme hacmine en az 0,114 m (≥ 0,25 D) |
| İniş takımı | JP Hobby ER-150 ×3, elektrikli: ana bacaklar içe, burun bacağı geriye (12 mm çatal ofseti); iz 0,66 m, dingil açıklığı 0,785 m; pervane çarpma açısı 13,2°, devrilme 47,9° |
| Faydalı yük | **Yalnız EO/IR taret** (SIYI ZT6 sınıfı: 4K optik + 640×512 termal, 197 g); top Ø74 mm, zemine 156 mm |
| Renkler | Üst RAL 7035, alt RAL 9003, vurgu RAL 7016, ince çizgi RAL 5018; kuyu/kapak içleri açık gri astar (`UM_Liner`), kapak kenarları koyu conta; uyarı turuncusu RAL 2004 **yalnız** pala uçları ve dikeylerdeki pervane bandında; uyarı yazıları RAL 3020; füme PETG kapak; mat siyah pervane, saten antrasit spinner |

## 2. Konfigürasyon ve neden

* **İtici motor, çene altı taret.** Taretin görüşüne pervane ve egzoz girmez, gövdeye pervane rüzgârı gelmez, uçak
  MALE kimliğini kazanır. Bedeli yüksek itki hattıdır: uçak tam güçte yerde burnunu kaldıramaz. Bu yüzden kalkış
  **düz** yapılır (2,5° kanat açısı + 20° flap → CL_yer 1,10). İtki ekseni 5° aşağı eğiktir ve CG'nin 36 mm
  üstünden geçer (tam güçte 1,95 N·m burun aşağı moment).
* **Ters motor, kambursuz sırt.** DLE-20 ters bağlanır (silindir aşağıda; DLE her konumda çalışır). Kaporta tepesi
  silindir başını değil krank karterini örter: sırt çizgisi kanattan lüle halkasına kadar tek, gergin bir rampayla
  (≤ 8°) yükselir, "deve hörgücü" yoktur. Motor, susturucu, depo ve akü zarfları sahnede `U_Env_*` tel kafesleriyle
  durur (`UCAV_Envelopes`, render ve GLB dışı); kaporta iç yüzüne paylar: silindir 9,9 mm, buji başlığı 5,3 mm
  (≥ 5), susturucu hava boşluğu 17,1 mm (≥ 15,5), karbüratör–yangın perdesi 17,3 mm (≥ 15), spinner–lüle 5,5 mm.
* **Gömme NACA girişi, halka lüle.** Soğutma havası karın altındaki gömme NACA ağzından (≈ 22 cm², boğaz s = 2,02,
  7° rampa, 40 → 75 mm ıraksak planform; dudak karın eğrisini izler) girer; spinner çevresindeki halka lüleden
  (akış alanı 34,3 cm²) ve çene altındaki soğutma yarığından çıkar (çıkış/giriş ≈ 1,8). Sırt temizdir. Susturucu
  sol alt yanaktaki kabartıdadır; çıkışı 45° eğik kesikli, 0,5 mm ısı kalkanlıdır. Sağ yanakta 6 panjur yarığı.
* **Oklu U-kuyruk.** İki dikey, pervaneyi lir gibi çerçeveler. Stabilize firar kenarı pala ucundan 0,32 D,
  pala süpürme hacminden 0,114 m (≥ 0,25 D) uzaktadır. Stabilize kalınlığı uca doğru %10'dan %12'ye çıkar; Ø12
  kiriş boyunun tamamında profil içinde kalır. Ventral fin yoktur; pervane çarpma açısı 13,2°'dir.
* **Uzun, ince kanat.** AR 14,8, düz merkez (veter 0,29 m, y ≤ 1,00) ve konik dış panel. Seyir ve bekleme gücü düşüktür
  (238 W / 147 W). 3° washout ile perdövites kökte başlar. 36° glove, chine "ok çizgisini" kanada bağlar ve kök
  bloğuna ER-150 ünitelerini taşıyacak hacim verir. İç ve dış flap arasında 10 mm ek aralığı, hücum kenarlarında
  22 mm erozyon bandı ve kökte kano biçimli kaporta (arkada 12° rampa) vardır.
* **Gövde.** Chine çizgili, iki süperelips yarısından oluşan kesit, aşağı sarkık burun, yukarı kalkık kuyruk konisi.
  Burun modülü (s 0–0,40) ve kuyruk–motor modülü (s 1,55'te flanş) ayrılır; iki 0,70 L depo tam CG'dedir. Sırttaki
  füme aviyonik kapağı neredeyse gömmedir (en çok 1,6 mm kabarık); siyah iç bandı ve antrasit çerçevesi vardır.
* **Yapı ve baskı.** Dış paneller 2 × 1,54 m ve sökülebilirdir. Merkezde cıvatalı G10 dihedral köprüsü (2 × 3 × 40 mm)
  kanat kutusu çerçevelerine (PA-CF) oturur; ana takım yatakları, motor halkası ve longeron kırık soketleri basılı yük
  yolu parçalarıdır. Dört CF longeronun her biri üç düz parçadır (kırıklar halka sınırlarında). Sahada kurulum 7
  parçadır. Bütün parçalar 256 mm tablaya sığar (220 mm tabla da desteklenir).
* **İniş takımı.** Üç bacak da elektrikli ER-150'dir (8,4 V'ta 5 s). Ana ünite bacak yuvasının arkasında, 3 mm G10
  plakaya bağlıdır (CF borulara 5,7–52,8 mm pay). Burun bacağı s 0,53'te, sökülebilir burun modülünün arkasındadır
  (12 mm iz). Kapaklar deri paneli + nervürlü çerçeve, menteşe bilekleri, horn, conta dudağı ve 12 mm testere dişli
  kenardan oluşur; burun bacağı tapa kapağı (`U_Door_N_3`) takım topluyken çentiği deriyle aynı hizada kapatır. Ana
  tekerlerde fren, burunda servo yönlendirme vardır. Kapak sırası (kapak 1 s – bacak 5 s – kapak 1 s) ArduPilot Lua
  ile yürür.

Tasarım üç konsept ve bir hakem panelinden geldi; ardından tasarım, aerodinamik, baskı ve animasyon eleştirileriyle
iyileştirildi. Panelin kararları ve kaynakları `spec.yaml → meta.sources`'ta, paneldeki sayılardan sapmalar
`# varsayım` ya da açıklama satırlarıyla işaretlidir.

## 3. Dosyalar

| Dosya | İçerik |
|---|---|
| [`spec.yaml`](spec.yaml) | **Tek doğruluk kaynağı** (Türkçe açıklamalı): ölçüler, profil, kütle, performans, takım, malzemeler (RAL), boya şemaları, işaretler, baskı planı, rig aralıkları |
| [`params.py`](params.py) | Saf Python: spec'i okur; gövde kesitleri, kanat/kuyruk kesitleri, menteşe hatları, takım geometrisi, kuyular, kapaklar, motor zarfları, CG (`python3 ucav/params.py`) |
| [`airfoils.py`](airfoils.py), [`data/airfoils/`](data/airfoils/) | UIUC SD7062/SD7032 verisi, NACA 4 haneli üretici, profil işlemleri |
| [`sizing.py`](sizing.py) | Boyutlandırma ve geometri kontrolü → [`out/sizing.md`](out/sizing.md) (`--check`: tolerans dışı varsa çıkış 1) |
| [`shapes.py`](shapes.py) | Saf numpy: kapalı, dışa dönük ağlar (gövde, kanat, kuyruk, kumanda yüzeyleri, kuyular, kapaklar, taret, NACA girişi, susturucu, ayrıntılar, 29 şablon yazı) |
| [`blender/airframe.py`](blender/airframe.py) | Gövde sahnesi: adlar, koleksiyonlar, pivotlar, yerel eksenler, EXACT boolean kuyular, `U_Stencil_*` yazıları, `U_Env_*` zarfları |
| [`blender/gear.py`](blender/gear.py) | ER-150 üniteleri, oleo bacaklar, tork bağlantıları, frenli tekerler, burun yönlendirme, kapak donanımı, çakışma raporu |
| [`blender/rig.py`](blender/rig.py) | `U_Root` kontrol paneli, 57 basit ifade sürücüsü, kumanda bağlantıları (boynuz / servo kolu / itme çubuğu), pervane diski, pervane pişirme metin bloğu |
| [`blender/materials.py`](blender/materials.py) | 31 `UM_*` malzemesi (boya + panel çizgileri ve servis kapakları, astar, conta, PA-CF, füme PETG, ısı tonlu susturucu, mat siyah pervane, pervane diski, EO/IR camları, ışıklar), "taktik" boya şeması, CG ve kuyruk işaretleri |
| [`blender/studio.py`](blender/studio.py) | Cycles ayarları, "pist" (gökyüzü, pist, çim, yer yansıması dolgusu) ve "studyo" ortamları, 9 sabit kamera |
| [`blender/animation.py`](blender/animation.py) | `showcase` (21 s) ve `mechanisms` (16 s döngü) klipleri, kameralar, bakım sehpası |
| [`blender/render.py`](blender/render.py) | Sabit görüntü (JPG) ve animasyon (MP4/H.264, Blender'ın kendi FFMPEG'i) |
| [`blender/printprep.py`](blender/printprep.py) | Baskı segmentleri, kabuk/iç yapı, yük yolu parçaları, menteşe pimleri, ısı kuralı, STL, baskı raporu |
| [`blender/build.py`](blender/build.py) | **Tek komutla kurulum ve bütün çıktılar** (bu belgenin §4'ü) |
| `out/` | `render/*.jpg`, `anim/*.mp4`, `stl/*.stl`, `print_report.md/.json`, `sizing.md`; `yk38.blend` ve `yk38.glb` git'e girmez (yeniden üretilir) |

Sahnedeki koleksiyonlar: `UCAV` (uçak: `UCAV_Airframe`, `UCAV_Surfaces`, `UCAV_Gear`, `UCAV_Propulsion`,
`UCAV_Payload`, `UCAV_Details`; varlık olarak işaretli), `UCAV_Envelopes` (paketleme zarfları, render dışı),
`UCAV_Print` (baskı parçaları, görünüm katmanı dışında), `UCAV_Studio` (`UCAV_Env` zemin ve ışıklar,
`UCAV_Cameras_Stills`, `UCAV_Cameras_Anim`, `UCAV_Stand` bakım sehpası).

## 4. Kurulum ve çalıştırma

Gerekenler: Python 3.11, `pip install bpy==4.5.3 numpy pyyaml` (bpy modülü Blender 4.5 LTS'tir; render Cycles CPU
ile yapılır). Ya da Blender 4.5 uygulaması (aşağıya bakın).

```bash
python3 ucav/sizing.py --check                         # boyutlandırma: 156 kontrol → out/sizing.md (bpy gerekmez)
python3 ucav/blender/build.py --blend                  # ≈ 25 s → out/yk38.blend (sıkıştırılmış, rig hazır)
python3 ucav/blender/build.py --blend --glb --print --stills    # sahne + GLB + STL/rapor + 9 sabit görüntü
python3 ucav/blender/build.py --stills hero side --samples 32 --res 960x600     # hızlı deneme
python3 ucav/blender/build.py --stills hero rear34 side --livery taktik        # → yk38_hero_taktik.jpg …
python3 ucav/blender/build.py --stills hero side --env studyo                  # koyu stüdyo
python3 ucav/blender/build.py --anim mechanisms --res 960x540 --samples 16     # → out/anim/mechanisms.mp4
python3 ucav/blender/build.py --anim all                                       # tam kalite: 1280×720, 24 örnek
python3 ucav/blender/build.py --anim showcase --frames 1-120 --step 2          # hızlı önizleme (süre korunur)
python3 ucav/blender/build.py --print --bed 220                                # 220×220×250 tabla raporu
python3 ucav/blender/build.py --help                                           # bütün seçenekler
```

Depodaki çıktılar şu komutlarla üretildi (toplam ≈ 3,5 saat; 2,6 saati animasyon):

```bash
python3 ucav/blender/build.py --blend --glb --print --stills --res 1600x1000 --samples 128
python3 ucav/blender/build.py --print --bed 220 --no-stl
python3 ucav/blender/build.py --stills hero rear34 side --livery taktik --res 1600x1000 --samples 128
python3 ucav/blender/build.py --stills hero side --env studyo --res 1600x1000 --samples 128
python3 ucav/blender/build.py --anim all --res 960x540 --samples 16
```

Sıra her zaman aynıdır: `airframe → gear → rig → materials → studio → animation → çıktılar`. Çıktı bayrağı
verilmezse `--blend` varsayılır. Sonda çıktılar boyutlarıyla listelenir; bir sorun varsa (eksik sürücü, malzemesiz ağ,
manifold olmayan ya da tablaya sığmayan baskı parçası, ısı kuralı ihlali) çıkış kodu 1'dir.

**Blender uygulamasıyla.** `blender -b -P ucav/blender/build.py -- --blend --stills` aynı işi yapar. Blender'ın kendi
Python'unda PyYAML yoksa betik sistem Python'undaki `yaml` paketini geçici olarak ödünç alır; o da yoksa kurulum
komutunu yazdırır (`"<blender python>" -m pip install pyyaml`). **Scripting sekmesinde:** *Text → Open* ile
`ucav/blender/build.py`'yi açın ve *Run Script*'e basın. Sahne açık dosyaya kurulur, Blender kapanmaz. Argüman için
dosyanın başındaki `SCRIPTING_ARGS` satırını düzenleyin (ör. `"--blend --stills hero"`) ya da Blender'ı
`UCAV_BUILD_ARGS` ortam değişkeniyle başlatın.

**`out/yk38.blend`'i açınca.** Uçak pistte, takım açık, `showcase` klibi etkin (kare 1). Görünüm penceresi malzeme
önizlemesindedir. Zaman çizelgesinde oynatınca kameralar işaretlerle değişir (`YK38_ev_*` işaretleri olayları
gösterir, kamerasızdır). Sabit kameralar `U_Cam_hero … U_Cam_gearbay` adlıdır. Dosyada iki metin bloğu vardır:
`YK38_klip_sec.py` (`KLIP` satırına `showcase`, `mechanisms` ya da `yok` yazıp *Run Script*; `yok` animasyonu
kaldırır ve dinlenme pozuna döner — kendi animasyonunuz için) ve `YK38_pervane_pisir.py` (§5). İkisi de depoya
ihtiyaç duymaz. Baskı parçaları `UCAV_Print` koleksiyonundadır; görünüm katmanından çıkarılmıştır (Outliner'da onay
kutusuyla açılır) ve render'a girmez. `UCAV` koleksiyonu başka bir dosyaya eklenebilir (*Append*/*Link* + *Library
Override*): yalnız uçak gelir, stüdyo ve baskı parçaları gelmez; uçağı kendi zemininize koyunca `ground_z`'yi o
zeminin Z'sine eşitleyin.

## 5. Kontrol paneli (`U_Root`)

`U_Root` tasarım CG'sindedir (Blender X = −1,232, Z = 0,006). Bütün uçak ona bağlıdır; onu taşıyıp döndürmek uçağı
CG etrafında hareket ettirir. Özellikler: `U_Root` seçili → *Object Properties → Custom Properties* (ya da N paneli).
Bütün hareketli parçalar bu özelliklere Blender **basit ifade** sürücüleriyle bağlıdır. Python betiği izni
(*Auto Run*) gerekmez. Animasyon yalnızca bu özelliklere, `U_Root` dönüşümüne ve pişirilmiş pervane turuna
(`U_Prop["ucav_turns"]`) anahtar kare koyar. Sürülen parçaların konum/dönüş/ölçeği kilitlidir (G/R/S ile
menteşeden kaçmaz); `U_Root` önde çizilen, adı görünen büyük bir oktur.

| Özellik | Aralık | Anlam |
|---|---|---|
| `gear` | 0…1 | 0 = toplu, 1 = açık ve kilitli. Tek değer sırayı sürer: kapaklar 0–0,15'te açılır, bacaklar 0,15–0,85'te döner, kapaklar 0,85–1'de kapanır. Bacak tapa kapağı bacakla döner |
| `gear_doors` | 0…1 | Deri kapaklarını el ile açar (otomatik sırayla büyük olan geçerli) |
| `aileron_deg` | −25…25 | + = sağa yatış (sol firar kenarı aşağı). Diferansiyel 1,67:1: yukarı en çok 20°, aşağı en çok 12° |
| `flap_deg` | 0…35 | İç ve dış flaplar birlikte, + = aşağı; mekanik sınır 30° |
| `elevator_deg` | −25…25 | + = firar kenarı aşağı (burun aşağı); sınır −25 / +20 |
| `rudder_deg` | −25…25 | + = firar kenarı sancağa (burun sağa); sınır ±22. Takım açıkken burun tekeri de döner (±30°) |
| `prop_rpm` | 0…9000 | Pervane devri (dev/dk); arkadan bakınca saat yönü tersi. 900 dev/dk üstünde pervane diski (`U_PropDisc`) görünür |
| `prop_auto` | 0/1 | Pervane açısı kipi: 1 = kare · rpm / 1440 tur (sabit devirde tam, 24 fps); 0 = pişirilmiş `U_Prop["ucav_turns"]` (devir değişen animasyonda doğru; `YK38_pervane_pisir.py` yazar). Klipler 0 kullanır |
| `turret_pan_deg` | −180…180 | + = iskeleye (sola) bakar |
| `turret_tilt_deg` | −90…20 | + = yukarı, −90 = tam aşağı (nadir) |
| `nav_lights` | 0/1 | İskele kırmızı, sancak yeşil seyrüsefer ışıkları ve iniş ışığı |
| `strobe` | 0/1 | Dikey uçlarında beyaz çakarlar: 30 karede bir çift çakış |
| `status_led` | 0/1 | Taret durum LED halkası (yalnız bakım/test). Gerçek EO/IR taretler ışımaz: uçuşta, kliplerde ve GLB'de 0 |
| `ground_z` | m | Zemin yüksekliği (dünya Z): amortisörler ve tekerler bu düzleme oturur (varsayılan −0,2904). Uçağı kendi sahnenizde başka bir zemine koyunca o zeminin Z'sine eşitleyin |
| `wheel_auto` | 0/1 | 1 = tekerler `U_Root`'un baş yönündeki yer ilerlemesiyle kaymadan döner (her başta) |
| `wheel_roll_m` | m | Ek yuvarlanma yolu: açı = (`wheel_roll_m` + `wheel_auto` · ilerleme) / R. Klipler pişirir (kalkıştan sonra yavaşlama, takım toplanırken fren) |

**Pervane açısı.** Devri sabit tutan animasyonlarda `prop_auto` = 1 yeterlidir. `prop_rpm`'e farklı değerli anahtar
kareler koyarsanız *Text Editor*'da `YK38_pervane_pisir.py`'yi çalıştırın: açıyı gerçek integral olarak
(`U_Prop["ucav_turns"]` = ∫ rpm/60 dt) pişirir ve `prop_auto` = 0 yapar; yoksa açısal hız rpm + kare·d(rpm)/dt olur
ve pervane geri dönüyormuş gibi görünür.

Ek özellikler: ışık nesnelerinde `ucav_emission` (rig sürer; malzemeler yayımı bununla çarpar, bu yüzden iniş lambası
çakarla aynı malzemeyi paylaşsa da yanıp sönmez), taret halkasında `ucav_status_gate`, `U_PropDisc`'te `ucav_disc`
(devirle 0 → 0,35), işaretlerde `ucav_decal_mix` (yazı kontrastı), statik derilerde `ucav_panel` (panel çizgisi
ailesi). Aralıklar `spec.yaml → rig.ranges_deg`'dedir.

**Kumanda bağlantıları.** Kanatçık, dış flap, elevatör ve dümende (iki yan) servis kapağından çıkan servo kolu
(`U_ServoArm_*`), Ø1,6 itme çubuğu + çatallar (`U_Pushrod_*`) ve G10 boynuz (`U_Horn_*`) vardır. Paralelkenar
bağlantıdır: servo kolu yüzeyle aynı ifadeyle döner, çubuk dönmeden öteler, ucu boynuz deliğinde kalır. İç flap ayrı
servo kullanmaz; dış flaba Ø2 bağlayıcı telle bağlıdır (toplam 8 kumanda servosu).

**Eksen kuralı.** Kumanda yüzeylerinin orijini menteşe hattının ortasındadır. Yerel X ekseni
`params.HingeLine.axis_positive_b`'dir: `rotation_euler.x` iki yanda da + = firar kenarı aşağı, dümende + = firar
kenarı sancağa. Sözleşmedeki "yerel X dışa doğru" ifadesi sol yüzeylerde bu işaretle çelişir (sağ el kuralı). Bu
yüzden sol yüzeylerde yerel X içe bakar. Yerel çerçeveler `delta_rotation_euler`'dedir; `rotation_euler` dinlenmede
(0, 0, 0)'dır ve `UCAV` ağacındaki bütün ebeveyn ters matrisleri birimdir ("Clear Parent Inverse" hiçbir şeyi
oynatmaz). `U_GearPivot_*`'ta yerel X = `GearLeg.retract_axis_b` (+ = toplar). Pervanede yerel X = mil ekseni,
geriye doğru. Taret: pan yerel Z, tilt yerel Y (`turret_tilt_deg` → −Y).

## 6. Animasyonlar

| Klip | Süre | İçerik | Kameralar |
|---|---|---|---|
| `showcase` | 21 s, 504 kare, 24 fps, pist | 0–2 s kuruluş planı (vinç alçalması + yaklaşma), motor çalışır (0 → 3600 → 3000 dev/dk). Kumanda kontrolü yakın planlarda: kanatçık ±20, flap 0 → 20°, irtifa −25/+20 ve istikamet ±22 (burun tekeri birlikte). Düz kalkış: 8600 dev/dk, 9,5 s'de 25,7 m'de teker keser (spec 25,5 m @ 13,2 m/s); amortisörler uzar, tekerler yavaşlar. Takım 12 s'de ≈ 2,8 m AGL'de gerçek sırayla toplanır (kapak 1 s + bacak 5 s + kapak 1 s), 26° yatışlı tırmanan sol dönüş, flap 0. Taret yer kamerasını izler | A kuruluş, A1 kanatçık, A1F flap, A2 kuyruk, B pist kenarı araç, C takip düzeneği (uçakla aynı yükseklikte), D yer kamerası + zum (zaman çizelgesi işaretleri) |
| `mechanisms` | 16 s, 384 kare, kesintisiz döngü, stüdyo | Bakım sehpasında: kanatçık, flap 0 → 30 → 0, irtifa + istikamet, burun tekeri yönlendirme ±30°, taret ±100° tarama, takım topla/aç (kapak 0,54 s / bacak 2,52 s / kapak 0,54 s, gerçek sürenin yarısı), pervane 240 dev/dk (döngüde tam 64 tur). Son kare ilk kareyle aynı | Geniş yörünge + işaretlerle kesilen yakın planlar |

Aksiyonlar adlandırılmıştır (`YK38_<klip>_Root`, `_Prop`, kamera aksiyonları) ve *fake user*'lıdır; uçuş yolu ve
kameralar seyrek BEZIER anahtarlarıyla yazılır (planın kare kare hâli `YK38_showcase_Root_pisirilmis`'tedir).
`animation.set_scene_range(ad)` klibi etkinleştirir, `animation.clear()` dinlenme pozuna döner (`.blend` içinde:
`YK38_klip_sec.py`). GLB (`out/yk38.glb`) `mechanisms` döngüsünü kare kare pişirilmiş tek animasyon olarak taşır:
takım, kapaklar, yüzeyler, bağlantılar, taret ve pervane (64 tur). Boya renkleri GLB'de basit PBR'ye çevrilir;
pervane diski ve paketleme zarfları GLB'ye girmez, taret LED'i sönüktür.

## 7. Render

Cycles CPU, uyarlamalı örnekleme, OpenImageDenoise, AgX (Medium High Contrast). Pozlama pistte −0,4 EV, stüdyoda 0.
Pistte uçağın altına, kameraya ve yansımalara görünmez bir yer yansıması dolgusu (`S_UnderFill`) konur; uçağı izler
ve kalkıştan sonra söner. Animasyonda hareket bulanıklığı açıktır (obtüratör 0,5 kare; pervanede 64 alt adım) ve
900 dev/dk üstünde yarı saydam pervane diski (uçta turuncu halka) görünür.

| Görünüm | İçerik |
|---|---|
| `hero` | Ön-sol 3/4, alçak, 70 mm, f/5.6 alan derinliği, güneş 24° |
| `rear34` | Arka-sağ 3/4 |
| `side` | Yan, **ortografik**, kanat dihedrali (4°) yüksekliğinden: yakın kanat kenardan, uzak kanadın üst yüzü gövdenin üstünde görünür |
| `front`, `top` | Ortografiğe yakın (135 / 85 mm) |
| `under` | Alttan, takım toplu |
| `nose` | EO/IR taret yakın plan (pan 18°, tilt −12°), f/4 |
| `tail` | U-kuyruk ve itici pervane, f/5.6 |
| `gearbay` | Sol ana takım ve açık kuyu, 28 mm |

Boya şemaları: `standart` (spec renkleri) ve `taktik` (koyu düşük görünürlüklü gri: üst ≈ RAL 7015, alt ≈ RAL 7046,
mat boya, açık gri işaretler). **Taktik şema yalnız render içindir:** koyu üst boya güneşte LW-PLA'yı ısıtır
(Tg ≈ 55 °C; baskı raporundaki ısı/boya kuralı: üst yüzey güneş yansıtması ≥ 0,5).

Ölçülen süreler (4 çekirdekli CPU, makine boşken; depodaki çıktılar bu ayarlarla üretildi — varsayılan sabit görüntü 64 örnektir):

| İş | Ayar | Süre |
|---|---|---:|
| Sahne kurulumu (her komutun başında) | gövde 21 s (EXACT boolean kuyular dahil), takım 1,4 s, rig + malzemeler + stüdyo + animasyon 1,4 s | ≈ 25 s |
| `--print` | 256 tabla: 65 parça, 68 STL (yeniden okunup doğrulanır) + rapor | 90 s |
| `--print --bed 220 --no-stl` | 66 parça, yalnız rapor | 87 s |
| `--glb` / `--blend` | `mechanisms` pişirme (9,0 MB) / zstd sıkıştırma (22,2 MB) | 3 s / < 1 s |
| `--stills` (9 görünüm) | 1600×1000, 128 örnek, pist | **33 dk** — görünüm başına: under 80 s, front 184, hero 196, nose 202, rear34 207, side 235, top 235, tail 267, gearbay 370 s |
| `--stills hero rear34 side --livery taktik` | 1600×1000, 128 örnek | 178 + 197 + 233 s |
| `--stills hero side --env studyo` | 1600×1000, 128 örnek | 137 + 149 s |
| `--anim showcase` | 960×540, 16 örnek, 24 fps, hareket bulanıklığı, pervane diski | **85 dk** (504 kare, 10,1 s/kare; yakın planlar 16–22 s, havada 6–7 s) → `anim/showcase.mp4` 2,5 MB |
| `--anim mechanisms` | 960×540, 16 örnek, stüdyo | **73 dk** (384 kare, 11,4 s/kare) → `anim/mechanisms.mp4` 1,6 MB |
| Tam kalite animasyon (tahmin) | 1280×720, 24 örnek, hareket bulanıklığı | ≈ 2,7 × önizleme: showcase ≈ 3,8 h, mechanisms ≈ 3,3 h |

Daha yüksek kalite için: `--samples 192` (sabit görüntü), `--anim all` (1280×720, 24 örnek). Animasyon kareleri
`out/anim/<klip>_frames/`'e yazılır. Yarıda kalırsa aynı komut kaldığı yerden sürer (var olan kareler atlanır).
Kodlamadan sonra bu klasör silinir.

## 8. 3B baskı

```bash
python3 ucav/blender/build.py --print                 # 256×256×256 → out/stl/*.stl, out/print_report.md/.json
python3 ucav/blender/build.py --print --bed 220       # 220×220×250 → out/print_220x220x250/ (yalnız rapor)
python3 -m ucav.blender.printprep --only wing_panel fus_ring_1 --no-stl     # kısmi kurulum
```

Gövde sahnesi (dinlenme pozu, booleanlar uygulanmış), spec `print` segment planına göre tablaya sığan, kapalı
(manifold) katı parçalara bölünür. Her parça ikili STL (mm) olarak baskı yönünde ve tabla merkezinde yazılır. Sol yarı
üretilir; sağ eşler aynadır (dilimleyicide aynalayın).

1. **Segment kesimi:** kaynak katı ∩ bölge (EXACT boolean). Kanat dış paneli 7 × 210 mm + raked uç; 2 kök bloğu +
   glove; gövdede burun konisi (s 0–0,148), sökülebilir burun modülü (taret yuvası), 6 LW-PLA halka ve 3 LW-ASA kuyruk
   konisi halkası (ekler kuyu, eyer ve hava alığından kaydırılır); PA-CF kaporta, yanaklar ve lüle; stabilize/dikey
   ekleri %25 ok eksenine dik.
2. **Kabuk:** dış yüzey içe ötelenir. Et kalınlıkları: kanat 0,6 mm (D-kutu 1,2), gövde 0,8, kuyruk 0,5, LW-ASA 1,0,
   PA-CF kaporta 1,6 mm. Ek kenarlarında 0,4 × 0,3 mm pah (panel çizgisi; render'daki çizgiler aynı istasyonlardadır).
3. **Birleşimler:** tabla ucunda 1,2 mm kaburga (hafifletme delikleri, boru delikleri malzemeye göre paylı çevrel
   çokgen), karşı uçta 2,4 × 6 mm yapıştırma flanşı ve Ø3 CF hizalama pimleri; gövdede 1,6 × 8 mm çerçeve.
4. **İç yapı ve yük yolları:** CF boru ve longeron kovanları, kesme ağları, ±40° geodezik kafes (0,45 mm), servo
   yuvaları, Ø8/Ø6 kablo kanalları ve MPX cepleri. PA-CF kanat kutusu çerçeveleri (G10 köprü yuvaları), ana takım
   beşiği (ER-150, 2 × M3) ve burun takım yatağı, motor halkası (4 × M4, 60 mm kare), PETG longeron kırık soketleri.
5. **Kumanda yüzeyleri:** ayrı parçalar, basılı menteşe dilleri; pim Ø1,75 PETG filament, Ø2,1 yatak. Her yüzeyin
   tek bir pim takma yönü vardır (kanatçık uç kapağından, dış flap panel ekinden, iç flap kök bloğundan, elevatör
   stabilize ucundan dikeyler takılmadan, dümen dikey tepesinden); rapor yüzey başına pim boyunu verir.
6. **Aviyonik kapağı:** füme PETG levha ısıl biçimlendirilir (kalıp `hatch_buck_a/b`, 2° eğim, 8 mm kesim payı); PETG
   çerçeve (`hatch_frame_a/b`) düz basılır, 6 mıknatısla gövde basamağına kilitlenir.
7. **Denetim:** her parça kapalı ve tek parçadır (STL'ler tam float32 kenar eşleme, 0,1 µm kaynak ve Blender'a
   yeniden okumayla doğrulanır), tabla teması ≥ 3 cm² (gerekirse 0,4 mm düzleme ya da 1 mm kırılabilir ayak),
   tablaya bakan 45° sarkma varsa "tabla desteği" diye işaretlenir, et kalınlığı ışınla ölçülür, sığmayan parça
   otomatik bölünür. **Isı kuralı:** silindir, buji başlığı ve susturucu zarfına 150 mm içinde LW-PLA olmaz; o
   bölgedeki parçalar (stabilize 1, elevatör 1) kendiliğinden LW-ASA'ya geçer.

**Önce tolerans kuponunu basın** (`tolerance_coupon.stl`: Ø27/16/8/6/3/1,75 delikler parçalardaki payla aynı);
gerekirse `TUBE_CLEAR`'ı ayarlayın.

**Sonuç, 256 × 256 × 256 mm tabla:** **65 benzersiz STL, 104 basılı parça** (sağ eşler aynadır), **4,06 kg** filament, ≈ **292 h** baskı (kaba model), 27,8 MB STL; ayrıca 3 alet parçası (2 kapak kalıbı + tolerans kuponu: 0,29 kg, ≈ 9 h). Bütün parçalar kapalı, tek parça ve tablaya en az 20 mm payla sığar; ısı kuralı ihlali yoktur.

| Malzeme | Parça | Kütle | Süre |
|---|---:|---:|---:|
| LW-PLA (kanat, gövde halkaları 1–6 ve burun, kumanda yüzeyleri, stabilize/dikey dış parçaları) | 64 | 2,66 kg | 204 h |
| PA-CF (kaporta, yanaklar, lüle, NACA dudağı, kök kaportası, ER-150 kabartması, kanat kutusu çerçeveleri, takım yatakları, motor halkası) | 15 | 0,77 kg | 49 h |
| LW-ASA (kuyruk konisi halkaları 7–9; ısı bölgesinde stabilize 1 ve elevatör 1) | 7 | 0,35 kg | 25 h |
| PETG (burun flanşı, taret yakası, takım kapakları, aviyonik kapağı çerçevesi, longeron soketleri, panel tutma dili) | 17 | 0,27 kg | 15 h |
| TPU (çene sürtünme pabucu) | 1 | 0,003 kg | 0,5 h |

Gruplara göre: kanat 1,44 kg, gövde 0,91 kg, kumanda yüzeyleri 0,37 kg, yük yolları 0,36 kg, kuyruk 0,31 kg,
itki 0,28 kg, kaplamalar 0,25 kg, taret yakası 0,08 kg, takım kapakları 0,07 kg.

**220 × 220 × 250 mm tabla:** 66 benzersiz STL / 105 parça, 4,07 kg, ≈ 293 h (burun modülü ikiye bölünür). Kanat paneli 1 ve iki kök bloğu 2,5–2,9 mm dar payla sığar; rapor bunları "dar pay" diye işaretler
([`out/print_220x220x250/print_report.md`](out/print_220x220x250/print_report.md)).

Basılmayan parçalar (CF borular, G10 köprü/perde/flanş/plakalar, bağlantı elemanları, 8 kumanda servosu, ER-150,
DLE-20, pervane, akü, taret), yük yolu tablosu, montaj sırası, dilimleyici ayarları (parça grubuna göre nozul,
çizgi eni, katman, Arachne sınırları) ve ısı/boya kuralı [`out/print_report.md`](out/print_report.md)'dedir.
Kanat üstü filetosu, stabilize kök filetosu ve dikey kök mermisi sıfıra inen kama olduğundan basılmaz; epoksi +
mikrobalonla doldurulur.

## 9. Boyutlandırma

```bash
python3 ucav/sizing.py            # out/sizing.md
python3 ucav/sizing.py --check    # tolerans dışı kontrol varsa çıkış kodu 1
```

[`out/sizing.md`](out/sizing.md) planform, kuyruk hacimleri, nötr nokta (%10 statik marj), stall hızları, itki hattı
momenti, lüle akış alanı, yer geometrisi (tip-back 17,7°, devrilme 47,9°, pervane çarpma 13,2°), pervane–stabilize
aralıkları (pala süpürme hacmi dahil), takım kuyusu payları, kütle dökümü ve CF boruların profile sığmasını (stabilize
kirişi boyunca) spec referanslarıyla karşılaştırır.

## 10. Mevzuat

YK-38'in MTOW'u 11,71 kg'dır. Yeni SHT-İHA'da (30.07.2026) **M1 sınıfına (4–25 kg)** girer. Özet ve madde
numaraları: [`../docs/08-guvenlik-ve-mevzuat.md`](../docs/08-guvenlik-ve-mevzuat.md). Bu uçak için anlamı:

* Uçuşa elverişlilik için **emniyet ve uygunluk beyanı** (Md. 8(5)). Beyan dosyasına aşağıdaki test kanıtları girer.
* İHATTYS kaydı, **uzaktan tanımlama** (Remote ID), geofence, uyum yazılım modülü ve BTK'ya uygun telsiz. Mevcut
  araçlar için geçiş süresi 31.07.2027'dir.
* Yetki araca değil pilota bağlıdır: P0/P1 ile **VLOS**, ≤ 120 m, ≤ 3000 m, yeşil bölge ya da izinli saha. 4 saatlik
  havada kalış kâğıt değerdir; BVLOS ve otonom görev **P2 + SHGM yetkilendirmesi** ister.
* Benzinli motor: kill-switch (ateşleme kesici), yangın perdesi, yakıt sızdırmazlığı. Link kaybında takım iner ve uçak
  önceden belirlenmiş kurtarma alanına gider (Md. 18(1)).

Bu not mühendislik özetidir, hukuki görüş değildir. Uçuştan önce SHGM'nin güncel metni kontrol edilmelidir.

## 11. Testler ve sonraki adımlar

```bash
python3 -m unittest discover -s tests -q                 # bütün depo testleri
python3 -m unittest tests.test_ucav_core -v              # spec/params/profil/boyutlandırma/geometri (bpy gerekmez)
python3 -m unittest tests.test_ucav_blender -v           # sahne: nesneler, sürücüler, malzemeler, manifold, zemin teması,
                                                         # takım çevrimi, bağlantılar, pervane açısı ve diski, döngü
                                                         # dikişi, append, ölçüler %1, GLB, render, STL'ler ve rapor
```

Fiziksel doğrulama (prototipte, uçuştan önce):

1. **Kütle tartımı.** Basılı parçalar plan tahmininin %22 üstündedir (§12). İlk iş, parçaları basıp tartmak ve CG'yi
   ölçmektir; MTOW bütçesi buna göre güncellenir.
2. **Kum torbası testi, 6 g.** Bu ölçek, kanıtlanmış baskı RC pratiğinin çok üstündedir. Kanat, MTOW'un 6 katı yayılı
   yükle (açıklık boyunca eliptik dağılım) sınanır. Merkez G10 köprüsü, kanat kutusu çerçeveleri ve cıvatalı soket
   tek yük yoludur; ayrıca kırılma testi yapılır.
3. **Motor tezgâhı.** Ters DLE-20 + 16×8 itici: statik itki (hedef 5,57 kgf), yakıt tüketimi (havada kalış modeli
   belirsizdir), ters çalışmada buji kirlenmesi ve rölanti, titreşim spektrumu (baskı parçalar ve aviyonik), silindir
   kafası, kaporta, yanak ve **stabilize kökü / elevatör 1** sıcaklığı (termokupl; LW-ASA Tg ≈ 95 °C), egzozun
   pervane diskine etkisi.
4. **Titreşim ve çırpınma.** Kanat ve U-kuyruk için yer titreşim testi; kumanda yüzeyi ve itme çubuğu boşlukları.
5. **Takım ve yer testleri.** ER-150 sırası (kapak–bacak–kapak), fren, burun yönlendirme ve shimmy (12 mm iz), düz
   kalkış koşusu. Ön CG'de kuyruk payı yalnız 0,16 CL_h'dir: kalkış nominal ya da daha arka CG ile yapılır.
6. **Stall testi.** clmax değerleri %5–10 iyimser olabilir. Stall hızları (12,0 / 10,8 m/s) yükseklikte, flap 0/20/30 ile
   ölçülür; washout'un kanatçık etkinliğini koruduğu doğrulanır.

## 12. Bilinen sınırlar

* **Basılı kütle tahminin %22 üstünde:** kurulan plan 4,06 kg, spec/panel tahmini 3,33 kg (+0,36 kg'ı yeni yük yolu
  parçalarıdır). Fark (+0,73 kg), MTOW bütçesindeki kütle artış rezervinin (0,56 kg) tamamını tüketir ve ≈ 0,17 kg
  aşar: hafifletilmezse MTOW ≈ 11,9 kg olur (stall hızı ≈ %0,7 artar) ve büyüme payı kalmaz. Spec'teki kütle dökümü
  panel değerlerini korur; prototip tartımıyla güncellenmelidir.
* **Ana teker eni ≈ 20,5 mm** (spec 26 mm). Kuyu tavanı arka-iç köşede kanat üst derisiyle sınırlıdır; toplu tekerde
  tavana ≥ 2,5 mm pay bırakılır. `gear.tyre_fit_width()` eni tavana göre hesaplar; tavan yükseltilirse kendiliğinden
  genişler.
* **Isı:** elevatör 1'in iç ucu susturucu zarfına ≈ 16 mm, stabilize 1 ≈ 21 mm uzaktadır (arada PA-CF yanak duvarı ve
  15 mm hava boşluğu). İkisi de LW-ASA basılır; motor tezgâhında ölçülmeden uçulmamalıdır. Daha kalıcı çözüm:
  susturucu çıkışını ≥ 60 mm geriye almak ya da kalkanı stabilize köküne uzatmak.
* **Kafes et kalınlığı 0,45 mm**, köpüklü LW-PLA çizgi genişliğinin altındadır; dilimleyicide ince duvar algılama
  (Arachne) gerekir (raporun dilimleyici tablosu). 9 STL'de et örneklerinin %0,5–3,4'ü 0,4 mm'nin altındadır
  (menteşe dili çentikleri, panjur ve testere dişi kenarlarında < 0,1 mm şeritler; dilimleyici atar). `cowl_top`
  et p05 değeri 0,69 mm'dir (hedef 0,8).
* **Destek gereken parçalar:** `cowl_cheek_L` (susturucu kabartısı, ≈ 96 cm² tabla desteği), `fus_ring_1`
  (≈ 60 cm²) ve `fus_ring_2` (≈ 45 cm², burun kuyusu içi); raporda "tabla desteği" diye işaretlidir.
* **Kaplama kenarları** (kök kaportası, ER-150 kabartması) 0,9 mm'nin altına incelen yerlerde kırpılır; montajda
  epoksi + mikrobalonla deriye sıfırlanır. Dikey kök mermisi neredeyse tamamen dikey/stabilize içindedir, basılmaz.
* **Kapak menteşeleri** dış deri çizgisindedir: kaz boynu menteşe toplu tekere çarpardı. Burun tapa kapağının ön
  kenarında ≈ 4,8 mm aralık kalır. Susturucu çıkış borusu yanak yüzeyinin ≈ 8 mm içinden başlar (yalnız görsel).
* **Kuyruk uç boşlukları** (elevatör ve dümen uçları) 2,5–3,2 mm'dir. Menteşe ok açısı yüzünden ±25°'de çarpmama payı
  bu kadardır. Menteşe (oyuk) aralığı her yüzeyde 1 mm'dir.
* **Yan görünüşte iki ton zayıf:** RAL 7035 ile RAL 9003'ün parlaklık oranı yalnız 1,38'dir; belirgin karşı gölge için
  spec'te daha koyu bir üst renk gerekir (taktik şema bunu gösterir).
* **Pervane diski stilizedir:** gerçek kamerada pala izi çok soluktur (obtüratör içinde ≈ 3 tur); disk okunurluk için
  eklenmiştir ve yalnız render'dadır.
* **Sözleşme farkı:** kumanda yüzeylerinde yerel X, `HingeLine.axis_positive_b`'dir; sol yüzeylerde içe bakar (§5).
* Tam kalite animasyonlar (1280×720, 24 örnek) depoya eklenmedi; depodakiler düşük maliyetli önizlemelerdir (§7).
