# ucav/ — YELKOVAN YK-38

**YELKOVAN YK-38**, MALE görünümlü, benzinli ve itici motorlu bir **sivil gözetleme ve araştırma İHA'sıdır**.
Bu klasör uçağın tamamını kodla tanımlar. Tasarım sayıları tek bir YAML dosyasındadır. Aynı kaynaktan iki ürün
çıkar: animasyonlu, ayrıntılı bir **Blender modeli** (rig, iniş takımı, render, animasyon) ve gerçek et
kalınlıklı, tablaya sığan **FDM baskı parçaları** (STL, Türkçe baskı raporu).

> **Kapsam.** YK-38 sivil bir EO/IR gözetleme ve araştırma platformudur. Tek faydalı yükü çene altındaki EO/IR
> kamera taretidir (SIYI ZT6 sınıfı). Silah, mühimmat, askı noktası, pilon ve bırakma mekanizması **yoktur**. Bunlar
> modele, spec'e ve bu depoya eklenmez. Testler (`tests/test_ucav_core.py`) spec'te bu tür anahtar olmadığını denetler.

![YK-38 — pistte, takım açık (Blender Cycles)](out/render/yk38_hero.jpg)

| | |
|---|---|
| ![Arka-sağ 3/4](out/render/yk38_rear34.jpg) | ![Yan](out/render/yk38_side.jpg) |
| ![EO/IR taret](out/render/yk38_nose.jpg) | ![U-kuyruk ve itici pervane](out/render/yk38_tail.jpg) |
| ![Alttan, takım toplu](out/render/yk38_under.jpg) | ![Sol ana takım ve açık kuyu](out/render/yk38_gearbay.jpg) |

## 1. Ana değerler

Bütün sayılar [`spec.yaml`](spec.yaml)'dandır. [`sizing.py`](sizing.py) bunları geometriden yeniden hesaplar
([`out/sizing.md`](out/sizing.md): **150/150 kontrol tolerans içinde**).

| Büyüklük | Değer |
|---|---|
| Sınıf | SHT-İHA **M1 (4–25 kg)**, sivil gözetleme/araştırma, VLOS, izinli saha |
| Açıklık / toplam boy / yükseklik | **3,80 m** / **2,475 m** (gövde 2,205 m, spinner dahil 2,285 m) / **0,668 m** (takım açık) |
| Kanat | S 0,976 m², AR 14,8, MAC 0,265 m, λ 0,52; SD7062 → SD7032, 4° dihedral, 3° washout, 36° kök glove, 70 mm raked uç |
| Kütle | MTOW **11,71 kg** (boş 10,07 kg; yakıt 1,40 L = 1,04 kg; görev modülü 0,60 kg); kanat yüklemesi 12,0 kg/m² |
| Motor ve pervane | DLE-20 (20 cc, 2 zamanlı benzinli, 1,86 kW), Xoar 16×8 **itici**; statik itki 5,57 kgf, T/W 0,48; itki ekseni 5° aşağı |
| Hızlar | Stall 12,0 m/s (temiz) / 10,8 m/s (20° flap); bekleme 15,6; seyir 20; Vne 28 m/s; (L/D)maks 18,4 @ 14,5 m/s |
| Havada kalış | 4,16 h (elektrik sınırı; yakıtla 3,6–4,25 h), menzil 233 km — **kâğıt değer**; operasyon VLOS'tur |
| Kalkış | Düz kalkış, 20° flap: V_LOF 13,2 m/s, koşu 25,5 m (asfalt) / 28 m (çim) |
| Kararlılık | Statik marj %10 MAC (η_h'ye göre %8,9–12,2); V_h 0,47, V_v 0,034; CG s = 1,232 m (aralık 1,219–1,245) |
| Kuyruk | Oklu U-kuyruk: stabilize 1,04 m, NACA 0010, HK oku 30°; iki dikey 0,32 m, 12° dışa eğik; pala ucunda FK–pervane aralığı 0,103 m (0,25 D) |
| İniş takımı | JP Hobby ER-150 ×3, elektrikli: ana bacaklar içe, burun bacağı geriye; iz 0,66 m, dingil açıklığı 0,907 m; pervane çarpma açısı 13,5° |
| Faydalı yük | **Yalnız EO/IR taret** (SIYI ZT6 sınıfı: 4K optik + 640×512 termal, 197 g); top Ø74 mm, zemine 156 mm |
| Renkler | Üst RAL 7035, alt RAL 9003, vurgu RAL 7016, ince çizgi RAL 5018, kuyu/kapak içleri RAL 2005, füme PETG kapak |

## 2. Konfigürasyon ve neden

* **İtici motor, çene altı taret.** Taretin görüşüne pervane ve egzoz girmez, gövdeye pervane rüzgârı gelmez, uçak
  MALE kimliğini kazanır. Bedeli yüksek itki hattıdır: uçak tam güçte yerde burnunu kaldıramaz. Bu yüzden kalkış
  **düz** yapılır (2,5° kanat açısı + 20° flap → CL_yer 1,10). İtki ekseni 5° aşağı eğiktir ve CG'nin 38 mm
  üstünden geçer (tam güçte 2,1 N·m burun aşağı moment).
* **Oklu U-kuyruk.** İki dikey, pervaneyi lir gibi çerçeveler. Stabilize firar kenarı pala ucundan 0,25 D uzaktadır.
  Kuyruk bu aralık için 0,07 m öne alındı ve stabilize %8 büyütüldü (V_h 0,475). Ventral fin yoktur; pervane çarpma
  açısı 13,5°'dir.
* **Uzun, ince kanat.** AR 14,8, düz merkez (veter 0,29 m, y ≤ 1,00) ve konik dış panel. Seyir ve bekleme gücü düşüktür
  (238 W / 147 W). 3° washout ile perdövites kökte başlar. 36° glove, chine "ok çizgisini" kanada bağlar ve kök
  bloğuna ER-150 ünitelerini taşıyacak hacim verir.
* **Gövde.** Chine çizgili, iki süperelips yarısından oluşan kesit, aşağı sarkık burun, yukarı kalkık kuyruk konisi.
  Sırtta NACA tipi hava alığı (27 cm²), arkada spinner çevresinde halka lüle (35 cm²). Burun modülü (s 0–0,40) ve
  kuyruk–motor modülü (s 1,55'te flanş) ayrılır; iki 0,70 L depo tam CG'dedir.
* **Yapı ve baskı.** Dış paneller 2 × 1,54 m ve sökülebilirdir. Merkezde cıvatalı G10 dihedral köprüsü (2 × 3 × 45 mm)
  tek yük yoludur. Kök/glove blokları gövdeye kalıcı yapıştırılır. Sahada kurulum 7 parçadır. Bütün parçalar
  256 mm tablaya sığar (220 mm tabla da desteklenir).
* **İniş takımı.** Üç bacak da elektrikli ER-150'dir (8,4 V'ta 5 s). Ana tekerlerde fren, burunda servo yönlendirme
  vardır. Kapak sırası (kapak 1 s – bacak 5 s – kapak 1 s) ArduPilot Lua ile yürür.

Tasarım üç konsept ve bir hakem panelinden geldi. Panelin kararları ve kaynakları `spec.yaml → meta.sources`'ta,
paneldeki sayılardan sapmalar `# varsayım` ya da açıklama satırlarıyla işaretlidir.

## 3. Dosyalar

| Dosya | İçerik |
|---|---|
| [`spec.yaml`](spec.yaml) | **Tek doğruluk kaynağı** (Türkçe açıklamalı): ölçüler, profil, kütle, performans, takım, malzemeler (RAL), baskı planı, rig aralıkları |
| [`params.py`](params.py) | Saf Python: spec'i okur; gövde kesitleri, kanat/kuyruk kesitleri, menteşe hatları, takım geometrisi, kuyular, kapaklar, CG (`python3 ucav/params.py`) |
| [`airfoils.py`](airfoils.py), [`data/airfoils/`](data/airfoils/) | UIUC SD7062/SD7032 verisi, NACA 4 haneli üretici, profil işlemleri |
| [`sizing.py`](sizing.py) | Boyutlandırma ve geometri kontrolü → [`out/sizing.md`](out/sizing.md) (`--check`: tolerans dışı varsa çıkış 1) |
| [`shapes.py`](shapes.py) | Saf numpy: kapalı, dışa dönük ağlar (gövde, kanat, kuyruk, kumanda yüzeyleri, kuyular, kapaklar, taret, ayrıntılar) |
| [`blender/airframe.py`](blender/airframe.py) | Gövde sahnesi: adlar, koleksiyonlar, pivotlar, yerel eksenler, EXACT boolean kuyular |
| [`blender/gear.py`](blender/gear.py) | ER-150 üniteleri, oleo bacaklar, tork bağlantıları, frenli tekerler, burun yönlendirme, çakışma raporu |
| [`blender/rig.py`](blender/rig.py) | `U_Root` kontrol paneli ve 39 basit ifade sürücüsü |
| [`blender/materials.py`](blender/materials.py) | 24 `UM_*` malzemesi (boya, PA-CF, füme PETG, lüle ısı tonu, kayın pervane, EO/IR camları, ışıklar), "taktik" boya şeması, işaretler |
| [`blender/studio.py`](blender/studio.py) | Cycles ayarları, "pist" (gökyüzü, pist, çim) ve "studyo" ortamları, 9 sabit kamera |
| [`blender/animation.py`](blender/animation.py) | `showcase` ve `mechanisms` klipleri, kameralar, bakım sehpası |
| [`blender/render.py`](blender/render.py) | Sabit görüntü (JPG) ve animasyon (MP4/H.264, Blender'ın kendi FFMPEG'i) |
| [`blender/printprep.py`](blender/printprep.py) | Baskı segmentleri, kabuk/iç yapı, STL (mm), baskı raporu |
| [`blender/build.py`](blender/build.py) | **Tek komutla kurulum ve bütün çıktılar** (bu belgenin §4'ü) |
| `out/` | `render/*.jpg`, `anim/*.mp4`, `stl/*.stl`, `print_report.md/.json`, `sizing.md`; `yk38.blend` ve `yk38.glb` git'e girmez (yeniden üretilir) |

## 4. Kurulum ve çalıştırma

Gerekenler: Python 3.11, `pip install bpy==4.5.3 numpy pyyaml` (bpy modülü Blender 4.5 LTS'tir; render Cycles CPU
ile yapılır). Ya da Blender 4.5 uygulaması (aşağıya bakın).

```bash
python3 ucav/sizing.py --check                         # boyutlandırma: 150 kontrol → out/sizing.md (bpy gerekmez)
python3 ucav/blender/build.py --blend                  # ≈ 20 s → out/yk38.blend (sıkıştırılmış, rig hazır)
python3 ucav/blender/build.py --blend --glb --print --stills    # sahne + GLB + STL/rapor + 9 sabit görüntü
python3 ucav/blender/build.py --stills hero side --samples 32 --res 960x600     # hızlı deneme
python3 ucav/blender/build.py --stills hero rear34 --livery taktik             # → yk38_hero_taktik.jpg …
python3 ucav/blender/build.py --stills hero side --env studyo                  # koyu stüdyo
python3 ucav/blender/build.py --anim mechanisms --res 960x540 --samples 16     # → out/anim/mechanisms.mp4
python3 ucav/blender/build.py --anim all                                       # tam kalite: 1280×720, 24 örnek
python3 ucav/blender/build.py --anim showcase --frames 1-120 --step 2          # hızlı önizleme (süre korunur)
python3 ucav/blender/build.py --print --bed 220                                # 220×220×250 tabla raporu
python3 ucav/blender/build.py --help                                           # bütün seçenekler
```

Sıra her zaman aynıdır: `airframe → gear → rig → materials → studio → animation → çıktılar`. Çıktı bayrağı
verilmezse `--blend` varsayılır. Sonda çıktılar boyutlarıyla listelenir; bir sorun varsa (eksik sürücü, malzemesiz ağ,
manifold olmayan ya da tablaya sığmayan baskı parçası) çıkış kodu 1'dir.

**Blender uygulamasıyla.** `blender -b -P ucav/blender/build.py -- --blend --stills` aynı işi yapar. Blender'ın kendi
Python'unda PyYAML yoksa betik sistem Python'undaki `yaml` paketini geçici olarak ödünç alır; o da yoksa kurulum
komutunu yazdırır (`"<blender python>" -m pip install pyyaml`). **Scripting sekmesinde:** *Text → Open* ile
`ucav/blender/build.py`'yi açın ve *Run Script*'e basın. Sahne açık dosyaya kurulur, Blender kapanmaz. Argüman için
dosyanın başındaki `SCRIPTING_ARGS` satırını düzenleyin (ör. `"--blend --stills hero"`) ya da Blender'ı
`UCAV_BUILD_ARGS` ortam değişkeniyle başlatın.

**`out/yk38.blend`'i açınca.** Uçak pistte, takım açık, `showcase` klibi etkin (kare 1). Görünüm penceresi malzeme
önizlemesindedir. Zaman çizelgesinde oynatınca kameralar işaretlerle değişir. Sabit kameralar `U_Cam_hero …
U_Cam_gearbay` adlıdır. Dosyada bir metin bloğu (`YK38_klip_sec.py`) vardır: `KLIP` satırına `showcase`,
`mechanisms` ya da `yok` yazıp *Run Script* deyin. `yok` animasyonu kaldırır ve dinlenme pozuna döner; kendi
animasyonunuz için bunu kullanın. Bu betik depoya ihtiyaç duymaz. Baskı parçaları `UCAV_Print` koleksiyonundadır;
görünüm katmanından çıkarılmıştır (Outliner'da onay kutusuyla açılır) ve render'a girmez.

## 5. Kontrol paneli (`U_Root`)

`U_Root` tasarım CG'sindedir (Blender X = −1,232, Z = 0,006). Bütün uçak ona bağlıdır; onu taşıyıp döndürmek uçağı
CG etrafında hareket ettirir. Özellikler: `U_Root` seçili → *Object Properties → Custom Properties* (ya da N paneli).
Bütün hareketli parçalar bu özelliklere Blender **basit ifade** sürücüleriyle bağlıdır. Python betiği izni
(*Auto Run*) gerekmez. Animasyon yalnızca bu özelliklere ve `U_Root` dönüşümüne anahtar kare koyar.

| Özellik | Aralık | Anlam |
|---|---|---|
| `gear` | 0…1 | 0 = toplu, 1 = açık ve kilitli. Tek değer sırayı sürer: kapaklar 0–0,15'te açılır, bacaklar 0,15–0,85'te döner, kapaklar 0,85–1'de kapanır. Yerdeyken amortisörler çöker, havada uzar; tekerler `U_Root`'un X hareketiyle kaymadan döner |
| `gear_doors` | 0…1 | Deri kapaklarını el ile açar (otomatik sırayla büyük olan geçerli) |
| `aileron_deg` | −25…25 | + = sağa yatış (sol firar kenarı aşağı). Diferansiyel 1,67:1: yukarı en çok 20°, aşağı en çok 12° |
| `flap_deg` | 0…35 | İç ve dış flaplar birlikte, + = aşağı; mekanik sınır 30° |
| `elevator_deg` | −25…25 | + = firar kenarı aşağı (burun aşağı); sınır −25 / +20 |
| `rudder_deg` | −25…25 | + = firar kenarı sancağa (burun sağa); sınır ±22. Takım açıkken burun tekeri de döner (±30°) |
| `prop_rpm` | 0…9000 | Pervane açısı = kare/fps · rpm/60 · 2π (+ `U_Prop["ucav_turns_offset"]` · 2π); arkadan bakınca saat yönü tersi |
| `turret_pan_deg` | −180…180 | + = iskeleye (sola) bakar |
| `turret_tilt_deg` | −90…20 | + = yukarı, −90 = tam aşağı (nadir) |
| `nav_lights` | 0/1 | İskele kırmızı, sancak yeşil seyrüsefer ışıkları ve taret durum halkası |
| `strobe` | 0/1 | Dikey uçlarında beyaz çakarlar: 30 karede bir çift çakış |

Ek özellikler: `U_Prop["ucav_turns_offset"]` (animasyon, devir değişirken pervane açısını ∫rpm·dt'ye eşitler; el ile
kullanımda 0), ışık nesnelerinde `ucav_emission` (rig sürer; malzemeler yayımı bununla çarpar, bu yüzden iniş lambası
çakarla aynı malzemeyi paylaşsa da yanıp sönmez), işaretlerde `ucav_decal_mix` (yazı kontrastı). Aralıklar
`spec.yaml → rig.ranges_deg`'dedir.

**Eksen kuralı.** Kumanda yüzeylerinin orijini menteşe hattının ortasındadır. Yerel X ekseni
`params.HingeLine.axis_positive_b`'dir: `rotation_euler.x` iki yanda da + = firar kenarı aşağı, dümende + = firar
kenarı sancağa. Sözleşmedeki "yerel X dışa doğru" ifadesi sol yüzeylerde bu işaretle çelişir (sağ el kuralı). Bu
yüzden sol yüzeylerde yerel X içe bakar. Yerel çerçeveler `delta_rotation_euler`'dedir; `rotation_euler` dinlenmede
(0, 0, 0)'dır. `U_GearPivot_*`'ta yerel X = `GearLeg.retract_axis_b` (+ = toplar). Pervanede yerel X = mil ekseni,
geriye doğru. Taret: pan yerel Z, tilt yerel Y (`turret_tilt_deg` → −Y).

## 6. Animasyonlar

| Klip | Süre | İçerik | Kameralar |
|---|---|---|---|
| `showcase` | 19 s, 456 kare, 24 fps, pist | Yerde motor çalıştırma (0 → 3000 dev/dk); kumanda kontrolü (burun tekeri istikametle döner); flap 20°; taret bakınır. Düz kalkış: 25,7 m'de 7,5 s'de teker keser (spec 25,5 m @ 13,2 m/s). Takım 7 s'de gerçek sırayla toplanır, 26° yatışlı tırmanan sol dönüş, flap 0. Taret 10 s'den sonra yer kamerasını izler | A yer, B pist kenarı araç, C takip düzeneği, D sehpa + zum (zaman çizelgesi işaretleri) |
| `mechanisms` | 10 s, 240 kare, kesintisiz döngü, stüdyo | Bakım sehpasında: takım topla/aç, kanatçık, irtifa, flap 0 → 30 → 0, istikamet + burun yönlendirme, taret 240° tarama, pervane 2400 dev/dk (döngüde tam 400 tur) | Sarkaç yörüngeli kamera |

Aksiyonlar adlandırılmıştır (`YK38_<klip>_Root`, `_Prop`, kamera aksiyonları) ve *fake user*'lıdır.
`animation.set_scene_range(ad)` klibi etkinleştirir, `animation.clear()` dinlenme pozuna döner (`.blend` içinde:
`YK38_klip_sec.py`). GLB (`out/yk38.glb`) `mechanisms` döngüsünü kare kare pişirilmiş tek animasyon olarak taşır:
takım, kapaklar, yüzeyler, taret ve yavaşlatılmış pervane. Boya renkleri GLB'de basit PBR'ye çevrilir.

## 7. Render

Cycles CPU, uyarlamalı örnekleme, OpenImageDenoise, AgX (Medium High Contrast). Animasyonda hareket bulanıklığı açıktır
(obtüratör 0,5 kare; pervanede 64 alt adım, bu yüzden 8600 dev/dk'da doğru disk görünür).

| Görünüm | İçerik |
|---|---|
| `hero` | Ön-sol 3/4, alçak, 70 mm |
| `rear34` | Arka-sağ 3/4 |
| `side`, `front`, `top` | Ortografiğe yakın (135 / 85 mm) |
| `under` | Alttan, takım toplu |
| `nose` | EO/IR taret yakın plan (pan 18°, tilt −12°), f/4 |
| `tail` | U-kuyruk ve itici pervane, f/5.6 |
| `gearbay` | Sol ana takım ve açık kuyu, 28 mm |

Ölçülen süreler (4 çekirdekli CPU):

| İş | Ayar | Süre |
|---|---|---:|
| Sahne kurulumu (her komutun başında) | gövde 13,5 s (20 EXACT boolean dahil), takım 1,4 s, rig + malzemeler + stüdyo + animasyon 1 s | ≈ 17 s |
| `--print` | 256 tabla, 55 STL + rapor | 85 s |
| `--print --bed 220 --no-stl` | 57 parça, yalnız rapor | 82 s |
| `--glb` / `--blend` | `mechanisms` pişirme / zstd sıkıştırma | 1,2 s / < 1 s |
| `--stills` (9 görünüm) | 1600×1000, 96 örnek, pist | **24 dk** — görünüm başına: under 56 s, nose 138, side 148, front 150, hero 154, top 176, tail 202, gearbay 214, rear34 217 s |
| `--stills hero rear34 --livery taktik` | 1600×1000, 96 örnek | 146 + 169 s |
| `--stills hero side --env studyo` | 1600×1000, 96 örnek | 85 + 73 s |
| `--anim showcase` | 960×540, 16 örnek, 24 fps, hareket bulanıklığı | **44 dk** (456 kare, 5,8 s/kare) → `anim/showcase.mp4` 2,2 MB |
| `--anim mechanisms` | 960×540, 16 örnek, stüdyo | **23 dk** (240 kare, 5,6 s/kare) → `anim/mechanisms.mp4` 0,9 MB |
| Tam kalite animasyon (tahmin) | 1280×720, 24 örnek, hareket bulanıklığı | showcase ≈ 2–2,5 h, mechanisms ≈ 1 h |

Sabit görüntü süreleri, aynı makinede testler de çalışırken ölçüldü; boş bir makinede biraz daha kısadır.

Daha yüksek kalite için: `--samples 128` (sabit görüntü), `--anim all` (1280×720, 24 örnek). Animasyon kareleri
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
   glove; gövdede burun konisi, sökülebilir burun modülü (taret yuvası), 6 LW-PLA halka ve 3 LW-ASA kuyruk konisi
   halkası; stabilize/dikey ekleri %25 ok eksenine dik.
2. **Kabuk:** dış yüzey içe ötelenir. Et kalınlıkları: kanat 0,6 mm (D-kutu 1,2), gövde 0,8, kuyruk 0,5, LW-ASA 1,0,
   PA-CF kaporta 1,6 mm.
3. **Birleşimler:** tabla ucunda 1,2 mm kaburga (hafifletme delikleri, boru delikleri Ø + 0,4 mm), karşı uçta
   2,4 × 6 mm yapıştırma flanşı ve Ø3 CF hizalama pimleri; gövdede 1,6 × 8 mm çerçeve.
4. **İç yapı:** CF boru ve longeron kovanları, kesme ağları, ±45° geodezik kafes (0,45 mm), 36 × 24 mm servo yuvaları.
   Kumanda yüzeyleri ayrı parçadır (menteşe dilleri, Ø1,5 çelik pim yatağı). PETG burun flanşı 4 × M4 + 2 × Ø4 pimle
   bağlanır.
5. **Denetim:** her parça kapalı (sınır/manifold dışı kenar yok, hacim > 0), et kalınlığı ışınla ölçülür, en az çıkıntılı
   yön seçilir, sığmayan parça otomatik ikiye bölünür, STL'ler yeniden okunup doğrulanır.

**Sonuç, 256 × 256 × 256 mm tabla:** 55 benzersiz STL, **89 basılı parça** (sağ eşler aynadır), **3,62 kg** filament,
≈ **267 h** baskı (kaba model), 20,4 MB STL. Bütün parçalar kapalıdır (manifold) ve tablaya en az 20 mm payla sığar.

| Malzeme | Parça | Kütle | Süre |
|---|---:|---:|---:|
| LW-PLA (kanat, gövde, kumanda yüzeyleri) | 68 | 2,70 kg | 209 h |
| PA-CF (kaporta, lüle, kök kaportası) | 7 | 0,39 kg | 26 h |
| LW-ASA (kuyruk konisi, 1,0 mm) | 3 | 0,27 kg | 18 h |
| PETG (burun flanşı, taret yakası, kapaklar) | 8 | 0,15 kg | 9 h |
| Füme PETG (aviyonik kapağı, 2 parça) | 2 | 0,10 kg | 5 h |
| TPU (sürtünme pabucu) | 1 | 0,003 kg | 1 h |

Gruplara göre: kanat 1,41 kg, gövde 0,94 kg, kumanda yüzeyleri 0,35 kg, kuyruk 0,29 kg, itki 0,29 kg, kaplamalar
0,20 kg, taret yakası 0,08 kg, takım kapakları 0,05 kg. **220 × 220 × 250 mm tabla:** 57 benzersiz / 92 parça
(burun modülü ve stabilize 1 ikiye bölünür). Kanat paneli 1, iki kök bloğu ve kök kaportası 2,5–4,5 mm dar payla sığar;
rapor bunları "dar pay" diye işaretler ([`out/print_220x220x250/print_report.md`](out/print_220x220x250/print_report.md)).

Basılmayan parçalar (CF borular, G10 köprü/perde/flanş, bağlantı elemanları, servolar, ER-150, DLE-20, pervane, akü,
taret), montaj sırası ve yazıcı ayarları [`out/print_report.md`](out/print_report.md)'dedir. Kanat üstü ve stabilize
kök filetoları sıfıra inen kama olduğundan basılmaz; epoksi + mikrobalonla doldurulur.

## 9. Boyutlandırma

```bash
python3 ucav/sizing.py            # out/sizing.md
python3 ucav/sizing.py --check    # tolerans dışı kontrol varsa çıkış kodu 1
```

[`out/sizing.md`](out/sizing.md) planform, kuyruk hacimleri, nötr nokta (%10 statik marj), stall hızları, itki hattı
momenti, yer geometrisi (devrilme 17,7°, yana devrilme 46,9°, pervane çarpma 13,5°), takım kuyusu payları, kütle
dökümü ve CF boruların profile sığmasını spec referanslarıyla karşılaştırır.

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
python3 -m unittest tests.test_ucav_core -v              # spec/params/profil/boyutlandırma (bpy gerekmez)
python3 -m unittest tests.test_ucav_blender -v           # sahne: nesneler, sürücüler, malzemeler, manifold, zemin teması,
                                                         # takım çevrimi çakışması, ölçüler %1, GLB, render, baskı parçaları
```

Fiziksel doğrulama (prototipte, uçuştan önce):

1. **Kum torbası testi, 6 g.** Bu ölçek, kanıtlanmış baskı RC pratiğinin çok üstündedir. Kanat, MTOW'un 6 katı yayılı
   yükle (açıklık boyunca eliptik dağılım) sınanır. Merkez G10 köprüsü ve cıvatalı soket tek yük yoludur; ayrıca kırılma
   testi yapılır.
2. **Motor tezgâhı.** DLE-20 + 16×8 itici: statik itki (hedef 5,57 kgf), yakıt tüketimi (havada kalış modeli
   belirsizdir), titreşim spektrumu (baskı parçalar ve aviyonik), silindir kafası ve kaporta sıcaklığı (LW-PLA
   Tg ≈ 55 °C), egzozun pervane diskine etkisi.
3. **Titreşim ve çırpınma.** Kanat ve U-kuyruk için yer titreşim testi; kumanda yüzeyi boşlukları.
4. **Takım ve yer testleri.** ER-150 sırası (kapak–bacak–kapak), fren, burun yönlendirme, düz kalkış koşusu. Ön CG'de
   kuyruk payı yalnız 0,16 CL_h'dir: kalkış nominal ya da daha arka CG ile yapılır.
5. **Stall testi.** clmax değerleri %5–10 iyimser olabilir. Stall hızları (12,0 / 10,8 m/s) yükseklikte, flap 0/20/30 ile
   ölçülür; washout'un kanatçık etkinliğini koruduğu doğrulanır.

## 12. Bilinen sınırlar

* **Basılı kütle tahminin %9 üstünde:** 3,62 kg, spec/panel tahmini 3,33 kg. Parça sayısı panelde 125 segment olarak
  sayılmıştı; kurulan plan 89 basılı parçadır (burun s = 0,148'de bölünür, aviyonik kapağı iki parçadır).
* **Ana teker eni 22,5 mm** (spec 26 mm). Kuyu tavanı arka-iç köşede kanat üst derisiyle sınırlıdır.
  `gear.tyre_fit_width()` eni tavana göre hesaplar; tavan yükseltilirse (ör. kanat/fileto üstünde küçük bir kabartma)
  26 mm'ye kendiliğinden döner.
* **Kafes et kalınlığı 0,45 mm**, köpüklü LW-PLA çizgi genişliğinin (0,6 mm) altındadır. Dilimleyicide ince duvar
  algılama (Arachne) gerekir. 0,6 mm'ye çıkarmak ≈ 50 g ekler.
* **Kaplamaların yapışma kenarı** (hava alığı, kök kaportası, kaporta üstü) sıfıra incelir (alanın %1–7'si < 0,5 mm).
  Montajda epoksi macunla düzeltilir. `wing_tip` 0,15 mm voksel onarımla kapanır.
* **Stabilize kirişi** (CF 12/10, 0,56 m) NACA 0010 içine y ≈ 0,42'ye kadar sığar; dış ucu 0,7 mm kısadır. Dikey
  ekinin G10 bağlantısına girmeli ya da daha kısa/ince boru seçilmelidir.
* **Kuyruk uç boşlukları** (elevatör ve dümen uçları) 2,5–3,2 mm'dir. Menteşe ok açısı yüzünden ±25°'de çarpmama payı
  bu kadardır. Menteşe (oyuk) aralığı her yüzeyde 1 mm'dir.
* **Sözleşme farkı:** kumanda yüzeylerinde yerel X, `HingeLine.axis_positive_b`'dir; sol yüzeylerde içe bakar (§5).
* Burun kapaklarındaki bacak çentiği, takım topluyken küçük turuncu bir kuyu yüzeyi gösterir.
* Pervanenin 8600 dev/dk'daki disk görünümü fiziksel olarak soluktur (obtüratör içinde ≈ 3 tur).
* Tam kalite animasyonlar (1280×720, 24 örnek) depoya eklenmedi; depodakiler düşük maliyetli önizlemelerdir (§7).
