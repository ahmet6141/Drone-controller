# Drone-Controller — 7 inç YZ Destekli Profesyonel Drone

Açık avuç jestiyle **ele konan**, **hedef tespit edip takip eden** ve **profesyonel hover**
yeteneğine sahip, orta boy (7 inç, 6S) bir quadcopter'ın yazılım ve donanım tasarımı.

> **Durum: Faz 0 tamamlandı + yazılım öncesi hazırlık** — araştırma, mimari ve konfigürasyonlar;
> sensör alternatifleri ve uyumluluk matrisi, bütçe kesinti senaryoları, parametrik 3B modeller
> ([docs/11](docs/11-yazilim-oncesi-hazirlik.md)), seri üretime uygun **v2 bütünleşik gövde** ve
> Blender render hattı ([docs/12](docs/12-butunlesik-govde-ve-seri-uretim.md)). Uçuş yazılımı (ROS 2
> paketleri) Faz 1'de yazılacak.

## Öne çıkan kararlar

| Konu | Karar | Gerekçe / belge |
|---|---|---|
| Platform | 7 inç X, GEPRC MOZ7 V2, **tam pervane koruması + alt ağ + avuç tutamağı** | 1,5–1,7 kg'lık bir drone için koruma zorunlu — [05](docs/05-avuca-inis-tasarimi.md) |
| Uçuş yığını | **PX4 v1.17** (ArduPilot 4.7.1 yedek) | ROS 2'den kayıtlı gerçek uçuş modları, BSD-3 — [02 §7](docs/02-donanim-analizi.md) |
| Uçuş kontrolcüsü | ARK FPV (A), ARKV6X (B/C) | Endüstriyel IMU + ısıtıcı, her iki yığın |
| Companion | **Raspberry Pi 5 + AI HAT+ 2 (Hailo-10H)** (A) · Jetson Orin NX 16GB (B/C) — ROS 2 Jazzy | Pi kameraları yerel ve ucuz; Jetson: VIO + büyük modeller + VLM — [02 §6](docs/02-donanim-analizi.md) |
| Kameralar ve gimbal | **Raspberry Pi Camera Module 3** (gimbal) + **CM3 Wide** (avuç), **kendi tasarımımız 2 eksen gimbal** (STorM32) | Hazır gimbal kameraya (A8 mini, $257) göre ≈ $135 ucuz, CSI ile düşük gecikme — [10](docs/10-kamera-gimbal-ve-sensorler.md) |
| Sensörler | MTF-01 (akış + lazer), TFmini-S (lazer irtifa), **VL53L8CX 8×8**, VL53L1X | Uygun fiyatlı; PX4/ArduPilot sürücü desteği doğrulandı — [10 §4](docs/10-kamera-gimbal-ve-sensorler.md) |
| Tespit | **YOLO26** (geliştirme) / **RF-DETR** (ticari, Apache-2.0) | Orin NX: YOLO26s INT8 4,78 ms — [03](docs/03-yapay-zeka-modelleri.md) |
| Takip | BoT-SORT + OSNet ReID, SOT: LiteTrack/SUTrack | Örtülme sonrası yeniden yakalama — [06](docs/06-hedef-takip-tasarimi.md) |
| Jest / avuç | İki aşamalı el hattı (HaGRIDv2 + landmark) + aşağı kamera + 8×8 ToF (`tools/tof8x8.py`) | Uzak mesafede küçük el; ToF ile mesafe/eğim/temas — [03 §5](docs/03-yapay-zeka-modelleri.md), [10 §5](docs/10-kamera-gimbal-ve-sensorler.md) |
| Hover | RPM notch (bidirectional DShot), RTK/akış/VIO EKF2 profilleri, autotune | [07](docs/07-hover-ayar-rehberi.md) |
| Avuca iniş | Drone kişiye yaklaşmaz; ≥ 2 bağımsız temas ipucu → ≤ 150 ms zorlamalı disarm; önce **işaretli iniş pedi** | [05](docs/05-avuca-inis-tasarimi.md) |
| Mevzuat | Yeni **SHT-İHA (30.07.2026)**: M0 sınıfı, YZ destekli otonomi için P2 + kontrollü izin; Ar-Ge kontrollü sahada saklı | [08](docs/08-guvenlik-ve-mevzuat.md) |
| Sensör alternatifleri | 9 görev, **26 seçenek**; PX4/ArduPilot parametreleri resmi referansla test edilir; Pi üzerinden ROS 2 → DDS köprüsüyle FC sürücüsü olmayan sensörler de | [11 §2–4](docs/11-yazilim-oncesi-hazirlik.md) |
| Bütçe | Yetenek kaybı olmadan **−$275** (Pi 5 4GB, AI HAT+ 26 TOPS, kendi baskımız); Ar-Ge MVP **$1.185** | [11 §3](docs/11-yazilim-oncesi-hazirlik.md) |
| 3B model | Kendimize özel parçalar **parametrik CadQuery** modelleri: koruma, avuç tutamağı, gimbal, üst katlar; STL hazır, kütle/çakışma/görüş kontrolleri. Görseller **Blender/Cycles** ile (geometri CAD'de kalır) | [cad/](cad/README.md), [11 §5](docs/11-yazilim-oncesi-hazirlik.md), [12 §2](docs/12-butunlesik-govde-ve-seri-uretim.md) |
| Gövde v2 (ürün) | **Bütünleşik gövde**: 3 kalıplı kabuk + **tek kalıptan 4 kol–kanal modülü** + **üstten ve alttan ≤ 10 mm ızgarayla tam kapalı pervane** + burnun ortasında sarkmayan ön gimbal (dışta pitch, içte roll) + kısa avuç ayağı + kuyruktan takılan akıllı batarya; yerleşim ağırlık merkezine göre kodla çözülür, DFM kuralları kontrol edilir | Seri üretime uygun, parmak hiçbir yönden pervaneye ulaşamaz; kamera gövdenin orta hattında; avuçta devrilme 14,6° → 27,4°; 1589 g, 16,6 dk (v1: 1456 g, 20,1 dk) — [12](docs/12-butunlesik-govde-ve-seri-uretim.md) |

## Donanım seviyeleri (hesaplanmış) — drone: **7 inç**

| | **Seviye A — Ekonomik (Raspberry Pi)** | Seviye B — Pro (Jetson) | Seviye C — Üst Seviye |
|---|---:|---:|---:|
| Kalkış ağırlığı | 1456 g | 1683 g | 1659 g |
| Hover süresi | 20,1 dk | 15,6 dk | 15,5 dk (6S2P: 23 dk) |
| T/W (batarya sınırlı / motor) | 2,77 / 5,84 | 2,37 / 5,05 | 2,40 / 5,12 |
| Hava aracı maliyeti (≈) | **$1,5 bin** | $4,9 bin | $6,7 bin |

7 inç pervane (17,8 cm), 336 mm gövde; korumalarla ≈ 43 × 43 cm. Ayrıntı: [`config/hardware/`](config/hardware/) ·
`python3 tools/budget_calc.py --all` · Seviye A kesinti paketleri: `python3 tools/scenarios.py`

![DC7 yerleşimi (cad/build.py)](cad/out/preview_assembly.png)

![DC7 v2 bütünleşik gövde (cad/v2/build_v2.py + Blender)](cad/v2/out/render/hero.jpg)

## Dizin yapısı

```
docs/
  01-gereksinimler.md          Gereksinimler ve başarı ölçütleri (ID'li)
  02-donanim-analizi.md        Güncel donanım analizi, 3 seviye, bütçe, tedarik riskleri
  03-yapay-zeka-modelleri.md   YZ modelleri: tespit, takip, jest, derinlik, VIO, VLM + lisanslar
  04-sistem-mimarisi.md        Donanım/yazılım mimarisi, ROS 2 düğümleri, durum makinesi
  05-avuca-inis-tasarimi.md    Avuca iniş: FMEA, mekanik, algılama, kontrol, temas, test
  06-hedef-takip-tasarimi.md   Hedef seçimi, 3B kestirim, gimbal ve takip modları
  07-hover-ayar-rehberi.md     Profesyonel hover ayarı (PX4 v1.17), kabul uçuşları
  08-guvenlik-ve-mevzuat.md    SHT-İHA 2026 özeti, tasarım etkileri, güvenlik
  09-yol-haritasi.md           Faz planı ve gereksinim → test eşleşmesi
  10-kamera-gimbal-ve-sensorler.md  Pi kameraları, kendi gimbalımız, ucuz sensörler, 8×8 ToF, boyut
  11-yazilim-oncesi-hazirlik.md     Sensör alternatifleri, bütçe kesintileri, uyumluluk, 3B model, hazırlık listesi
  12-butunlesik-govde-ve-seri-uretim.md  CAD ↔ Blender, v2 bütünleşik gövde, CG yerleşimi, DFM, kalıp, EVT/DVT/PVT
config/
  hardware/   Seviye A/B/C bileşen, kütle, güç, itki eğrisi, batarya, fiyat; thrust/ itki standı şablonu;
              variants/ v2 bütünleşik gövde profili (gövde kütleleri CAD'den)
  sensors/    Sensör kataloğu: görev bazında alternatifler, PX4/ArduPilot sürücü eşlemesi, köprü konuları
  budget/     Bütçe kesinti senaryoları ve paketleri (Seviye A üzerine yamalar)
  px4/        PX4 v1.17 parametreleri: base/ (ortak) + profiles/ (dış mekân GNSS, iç mekân VIO)
  ardupilot/  ArduPilot Copter 4.7.1 yedek parametreleri
  ai/         Algı hattı: seviye bazında modeller, eşikler, lisans profili
  mission/    Davranış (avuca iniş, takip, gimbal) ve güvenlik sınırları
  companion/  Raspberry Pi 5 ve Jetson: ortam dosyaları, systemd, mavlink-router, kurulum rehberi
cad/
  params.py, layout.py Ölçüler; ağırlık merkezi, batarya konumu, görüş alanı kontrolleri (CadQuery'siz)
  prop_guard.py …      Koruma, avuç tutamağı, 2 eksen gimbal, üst katlar (CadQuery)
  build.py             STL/STEP/GLB, kütle–bütçe raporu, önizlemeler → out/
  render_blender.py    Blender/Cycles fotogerçekçi render (--model v1 | v2)
  v2/                  Bütünleşik gövde: yerleşim, titreşim, kabuklar, kol–kanal modülü, akıllı batarya
tools/
  budget_calc.py       AUW, T/W, hover gücü/süresi, MPC_THR_HOVER, THR_MDL_FAC, maliyet
  scenarios.py         Bütçe kesinti senaryoları: maliyet, kütle, hover ve limit etkisi
  sensor_matrix.py     Sensör kataloğu doğrulaması, uyumluluk matrisi ve genişliği
  thrust_stand.py      İtki standı CSV → itki eğrisi, THR_MDL_FAC, kurulum katsayısı
  validate_params.py   Parametre dosyalarını resmi PX4 v1.17 / ArduPilot 4.7.1 referansına göre doğrular
  tof8x8.py            8×8 ToF karesinden avuç tespiti: konum, mesafe, eğim, temas (demo + testler)
  data/                Resmi parametre referanslarından çıkarılmış veri (02.10.2026)
tests/
  test_configs.py      Hesaplayıcı + tüm konfigürasyonların tutarlılık testleri
  test_sensors.py      Sensör kataloğu: parametreler, enum'lar, seviye seçimleri, uyumluluk sayıları
  test_scenarios.py    Bütçe senaryoları: limitler, güvenlik bileşenleri korunuyor mu
  test_thrust_stand.py İtki standı aracı
  test_cad.py          Yerleşim kuralları (her zaman) + CAD parçaları (CadQuery kuruluysa)
  test_v2.py           v2 yerleşimi ve kol titreşimi (her zaman) + v2 CAD parçaları (CadQuery kuruluysa)
  test_tof8x8.py       8×8 ToF avuç analizi testleri
```

## Hızlı başlangıç

```bash
pip install pyyaml                         # tek bağımlılık
python3 tools/budget_calc.py --all         # üç seviyenin bütçesi
python3 tools/validate_params.py           # PX4 / ArduPilot parametre doğrulaması
python3 tools/tof8x8.py --demo             # 8×8 mesafe sensörü nasıl görür? (avuç 0,40 m'de)
python3 tools/sensor_matrix.py --markdown  # sensör alternatifleri ve PX4/ArduPilot uyumluluğu
python3 tools/scenarios.py --markdown      # bütçe kesinti senaryoları ve paketleri
python3 cad/layout.py                      # ağırlık merkezi, batarya konumu, görüş alanları
python3 cad/v2/layout_v2.py                # v2 bütünleşik gövde: yerleşim ve ağırlık merkezi
python3 cad/v2/analysis_v2.py              # v2 kol–kanal modülü titreşimi ve dayanımı
python3 -m unittest discover -s tests -v   # tüm tutarlılık testleri
# 3B modeller (isteğe bağlı): pip install cadquery matplotlib && python3 cad/build.py && python3 cad/v2/build_v2.py
# Render (isteğe bağlı, Python 3.11): pip install bpy==4.5.3 && python cad/render_blender.py --model v2
```

Testler şunları garanti eder: her parametre adı/tipi/aralığı resmi referansta geçerli;
PX4 ↔ `safety.yaml` sınırları (hız, geofence, batarya) aynı; `MPC_THR_HOVER` ve `THR_MDL_FAC`
bütçe hesabıyla uyumlu; YZ eşikleri davranış eşikleriyle aynı; avuca iniş güvenlik
değişmezleri (koruma şartı, drone'un kişiye yaklaşmaması, ≤ 150 ms temas onayı) korunuyor;
sensör kataloğundaki her sürücü parametresi geçerli ve seviye seçimleri donanım profilleriyle aynı;
bütçe paketleri limitleri ve güvenlik bileşenlerini koruyor; yerleşimde ağırlık merkezi ortada,
kamera/sensör görüşleri temiz ve gimbal yazılım sınırı görüş kontrolüyle uyumlu; v2'de bileşenler
gövdenin içinde ve çakışmasız, GNSS gürültü kaynaklarından uzak, kol frekansı motor ve kanat geçiş
bantlarının arasında.

## Önemli uyarılar

- **Güvenlik**: 7 inç pervaneler hover'da ≈ 76 m/s uç hızına ulaşır. Avuca iniş geliştirmesi
  fileli kapalı test hücresinde, KKD ile ve emniyet pilotu eşliğinde yapılmalıdır ([08 §3](docs/08-guvenlik-ve-mevzuat.md)).
- **Mevzuat**: YZ destekli otonom uçuş, ayrı talimat çıkana kadar P2 lisansı + kontrollü izin +
  geofence + tescil gerektirir; uçuş ve yer istasyonu yazılımları SHGM yazılım sertifikasyonuna
  tabidir ([08 §1](docs/08-guvenlik-ve-mevzuat.md)).
- **Lisans**: Ultralytics YOLO modelleri AGPL-3.0'dır; kapalı kaynak ticari ürün için Enterprise
  lisansı veya Apache yolu (RF-DETR + Roboflow trackers) gerekir ([03 §2](docs/03-yapay-zeka-modelleri.md)).
- "tahmini" / "doğrulanmadı" notlu değerler Faz 2 ölçümleriyle güncellenecektir.

## Sonraki adım
1. **Yazılım öncesi** ([11 §6](docs/11-yazilim-oncesi-hazirlik.md)): parçaları bas ve uydur, itki
   standı (`tools/thrust_stand.py`), sensör tezgâh testleri, gecikme ölçümü, el tipi veri toplama,
   port/EMI planı, mevzuat ve güvenlik ekipmanı.
2. **Faz 1**: ROS 2 çalışma alanı ve paket iskeleti, PX4 SITL + Gazebo'da DC7 modeli (kütle ve atalet
   `cad/layout.py`'den), `PalmLand` / `FollowTarget` özel modları, algı hattının kayıtlı videolarda
   (Pi 5 + Hailo) çalıştırılması ([09](docs/09-yol-haritasi.md)).
3. **v2 EVT (paralel)** ([12 §9](docs/12-butunlesik-govde-ve-seri-uretim.md)): MJF PA12 gövde seti, uydurma
   ve kablo demeti, IMU FFT ile kol frekansı, STEP'ten FEA (kanal ve ızgara modları), kalıp teklifleri.
