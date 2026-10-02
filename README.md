# Drone-Controller — 7 inç YZ Destekli Profesyonel Drone

Açık avuç jestiyle **ele konan**, **hedef tespit edip takip eden** ve **profesyonel hover**
yeteneğine sahip, orta boy (7 inç, 6S) bir quadcopter'ın yazılım ve donanım tasarımı.

> **Durum: Faz 0 tamamlandı** — araştırma, analiz, mimari ve konfigürasyonlar (2 Ekim 2026
> itibarıyla güncel cihaz, yazılım ve modeller). Uçuş yazılımı (ROS 2 paketleri) Faz 1'de yazılacak.

## Öne çıkan kararlar

| Konu | Karar | Gerekçe / belge |
|---|---|---|
| Platform | 7 inç X, GEPRC MOZ7 V2, **tam pervane koruması + alt ağ + avuç tutamağı** | 1,5–1,7 kg'lık bir drone için koruma zorunlu — [05](docs/05-avuca-inis-tasarimi.md) |
| Uçuş yığını | **PX4 v1.17** (ArduPilot 4.7.1 yedek) | ROS 2'den kayıtlı gerçek uçuş modları, BSD-3 — [02 §7](docs/02-donanim-analizi.md) |
| Uçuş kontrolcüsü | ARK FPV (A/B), ARKV6X (C) | Endüstriyel IMU + ısıtıcı, her iki yığın |
| Companion | **Jetson Orin NX 16GB** (B/C), Orin Nano 8GB Super (A) — JetPack 7.2.1, ROS 2 Jazzy, Isaac ROS 4.6 | 157 TOPS, donanım video kodlama, VIO + algı + küçük VLM birlikte — [02 §6](docs/02-donanim-analizi.md) |
| Tespit | **YOLO26** (geliştirme) / **RF-DETR** (ticari, Apache-2.0) | Orin NX: YOLO26s INT8 4,78 ms — [03](docs/03-yapay-zeka-modelleri.md) |
| Takip | BoT-SORT + OSNet ReID, SOT: LiteTrack/SUTrack | Örtülme sonrası yeniden yakalama — [06](docs/06-hedef-takip-tasarimi.md) |
| Jest / avuç | İki aşamalı el hattı (HaGRIDv2 + MediaPipe/RTMPose) + global shutter aşağı kamera + 8×8 ToF | Uzak mesafede küçük el, hareket bulanıklığı — [03 §5](docs/03-yapay-zeka-modelleri.md) |
| Hover | RPM notch (bidirectional DShot), RTK/akış/VIO EKF2 profilleri, autotune | [07](docs/07-hover-ayar-rehberi.md) |
| Avuca iniş | Drone kişiye yaklaşmaz; ≥ 2 bağımsız temas ipucu → ≤ 150 ms zorlamalı disarm; önce **işaretli iniş pedi** | [05](docs/05-avuca-inis-tasarimi.md) |
| Mevzuat | Yeni **SHT-İHA (30.07.2026)**: M0 sınıfı, YZ destekli otonomi için P2 + kontrollü izin; Ar-Ge kontrollü sahada saklı | [08](docs/08-guvenlik-ve-mevzuat.md) |

## Donanım seviyeleri (hesaplanmış)

| | Seviye A — Geliştirme | **Seviye B — Önerilen Pro** | Seviye C — Üst Seviye |
|---|---:|---:|---:|
| Kalkış ağırlığı | 1499 g | 1666 g | 1639 g |
| Hover süresi | 18,8 dk | 15,7 dk | 15,6 dk (6S2P: 23 dk) |
| T/W (batarya sınırlı / motor) | 2,68 / 5,67 | 2,39 / 5,10 | 2,42 / 5,18 |
| Hava aracı maliyeti (≈) | $2,2 bin | $4,4 bin | $6,4 bin |

Ayrıntı: [`config/hardware/`](config/hardware/) · `python3 tools/budget_calc.py --all`

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
config/
  hardware/   Seviye A/B/C bileşen, kütle, güç, itki eğrisi, batarya, fiyat
  px4/        PX4 v1.17 parametreleri: base/ (ortak) + profiles/ (dış mekân GNSS, iç mekân VIO)
  ardupilot/  ArduPilot Copter 4.7.1 yedek parametreleri
  ai/         Algı hattı: seviye bazında modeller, eşikler, lisans profili
  mission/    Davranış (avuca iniş, takip, gimbal) ve güvenlik sınırları
  companion/  Jetson: systemd servisleri, mavlink-router, netplan, kurulum rehberi
tools/
  budget_calc.py       AUW, T/W, hover gücü/süresi, MPC_THR_HOVER, THR_MDL_FAC, maliyet
  validate_params.py   Parametre dosyalarını resmi PX4 v1.17 / ArduPilot 4.7.1 referansına göre doğrular
  data/                Resmi parametre referanslarından çıkarılmış veri (02.10.2026)
tests/
  test_configs.py      Hesaplayıcı + tüm konfigürasyonların tutarlılık testleri
```

## Hızlı başlangıç

```bash
pip install pyyaml                         # tek bağımlılık
python3 tools/budget_calc.py --all         # üç seviyenin bütçesi
python3 tools/validate_params.py           # PX4 / ArduPilot parametre doğrulaması
python3 -m unittest discover -s tests -v   # tüm tutarlılık testleri
```

Testler şunları garanti eder: her parametre adı/tipi/aralığı resmi referansta geçerli;
PX4 ↔ `safety.yaml` sınırları (hız, geofence, batarya) aynı; `MPC_THR_HOVER` ve `THR_MDL_FAC`
bütçe hesabıyla uyumlu; YZ eşikleri davranış eşikleriyle aynı; avuca iniş güvenlik
değişmezleri (koruma şartı, drone'un kişiye yaklaşmaması, ≤ 150 ms temas onayı) korunuyor.

## Önemli uyarılar

- **Güvenlik**: 7 inç pervaneler hover'da ≈ 76 m/s uç hızına ulaşır. Avuca iniş geliştirmesi
  fileli kapalı test hücresinde, KKD ile ve emniyet pilotu eşliğinde yapılmalıdır ([08 §3](docs/08-guvenlik-ve-mevzuat.md)).
- **Mevzuat**: YZ destekli otonom uçuş, ayrı talimat çıkana kadar P2 lisansı + kontrollü izin +
  geofence + tescil gerektirir; uçuş ve yer istasyonu yazılımları SHGM yazılım sertifikasyonuna
  tabidir ([08 §1](docs/08-guvenlik-ve-mevzuat.md)).
- **Lisans**: Ultralytics YOLO modelleri AGPL-3.0'dır; kapalı kaynak ticari ürün için Enterprise
  lisansı veya Apache yolu (RF-DETR + Roboflow trackers) gerekir ([03 §2](docs/03-yapay-zeka-modelleri.md)).
- "tahmini" / "doğrulanmadı" notlu değerler Faz 2 ölçümleriyle güncellenecektir.

## Sonraki adım: Faz 1
ROS 2 çalışma alanı ve paket iskeleti, PX4 SITL + Gazebo'da `PalmLand` / `FollowTarget` özel
modları, algı hattının kayıtlı videolarda çalıştırılması ve jest veri setinin hazırlanması
([09](docs/09-yol-haritasi.md)).
