# cad/ — DC7 parametrik 3B modelleri

Gövde (GEPRC MOZ7 V2), motorlar ve elektronik hazır alınır; **kendimize özel parçaların** hepsi burada
kodla (CadQuery) tanımlanır. Ölçüyü değiştirip yeniden üretmek tek komuttur. Ayrıntılı yaklaşım:
[docs/11 §5](../docs/11-yazilim-oncesi-hazirlik.md).

| Dosya | İçerik |
|---|---|
| `params.py` | Tüm ölçüler (mm): gövde, pervane, koruma, tutamak, kamera, gimbal, üst katlar, malzeme yoğunlukları |
| `layout.py` | CadQuery gerektirmez: ağırlık merkezi, batarya konumu, devrilme açısı, kamera/sensör görüş alanı kontrolleri |
| `prop_guard.py` | Pervane koruması: halka + alt ağ (≤ 10 mm göz) ve motor bağlantı parçası (×4) |
| `palm_grip.py` | Avuç tutamağı: tüp, sensör tablası (CM3 Wide + VL53L8CX, tabandan 25 mm içeride), TPU tampon |
| `gimbal_2axis.py` | 2 eksen gimbal: beşik (kızaklı), roll kolu, üst parça, taşıyıcı kol; denge ve çakışma kontrolü |
| `top_deck.py` | Companion tepsisi (Pi 5 + AI HAT+) ve batarya plakası |
| `build.py` | Hepsini üretir: `out/stl/*.stl` (baskı), `out/step/*.step` (CAD), `out/report.md`, önizlemeler |

## Çalıştırma

```bash
python3 cad/layout.py                 # yalnızca PyYAML: ağırlık merkezi + görüş alanı + tasarım kuralları
pip install cadquery matplotlib       # Windows / macOS / Linux, Python 3.10–3.12
python3 cad/build.py                  # ≈ 30 s: STL + STEP + rapor + önizleme
python3 -m unittest tests.test_cad    # CadQuery varsa parça testleri de çalışır
```

STL dosyaları ve önizlemeler depoya eklenmiştir; STEP dosyaları (≈ 6 MB) `build.py` ile üretilir.

![Montaj önizlemesi](out/preview_assembly.png)

## Güncel sonuçlar (`out/report.md`)

| Parça grubu | Bütçe | Model | Not |
|---|---:|---:|---|
| Pervane koruması (4 takım) | 110 g | 97,6 g (PA-CF) | Ağ, pervane alanının %20'sini kapatır → itki kaybı itki standında ölçülür. `MESH_SECTOR_DEG = 180` (yalnızca gövdeye bakan yarı): 88 g, %13 |
| Avuç tutamağı | 40 g | 43,1 g | PETG tüp; PA-CF tüple ≈ 39 g |
| 2 eksen gimbal (motorlar dahil) | 85 g | 85,9 g | Pitch −90…+30° ve roll ±30° boyunca çakışma yok; pitch dengesizliği 0,26 mm |
| Üst plakalar | 60 g* | 31,6 g | *bütçe kablolama ve bağlantıları da kapsar |

Yerleşim: ağırlık merkezi yatayda merkezde (batarya x = −15,4 mm). Gimbal kamerasının görüşü
−90…+20° arası temiz → `behavior.yaml` yazılım sınırı +15°. Motorlar kapalıyken tutamak tabanında
devrilme açısı 14,6° → avuçta düz tutun, temastan sonra tutamağı kavrayın. Yerden kalkış için
Ø150 mm sökülebilir ayak (29°) önerilir.

## Baskı önerileri

| Parça | Malzeme | Ayar |
|---|---|---|
| Koruma halkası, bağlantı, gimbal, plakalar | PA-CF (veya PETG: ≈ %15 daha ağır) | 0,4 mm nozul, 3 çevre çizgisi, %40 dolgu (ince duvarlar zaten tam dolu) |
| Tutamak tüpü, sensör tablası | PETG / PA-CF | Tüp ters (flanş tablada), destek yok |
| Tampon | TPU 95A | Yavaş (20–30 mm/s) |

PA-CF için sertleştirilmiş nozul ve kurutulmuş filament gerekir. Delikler M3 3,3 mm, M2 2,3 mm,
M2 kendinden kılavuzlu vida pilot deliği 1,8 mm olarak modellenmiştir; yazıcınıza göre `params.py`
içinde ayarlayın.

## Ölç → güncelle → yeniden üret

`params.py` içinde "tahmini" yazan ölçüler gerçek parçalardan kumpasla ölçülmelidir: motor delik
deseni (16 / 19 mm), motor çanı çapı, pervane düzlemi yüksekliği, kamera kartı delikleri, gövde
alt plakası delikleri (gimbal taşıyıcı kolu), batarya ölçüleri. Sonra `python3 cad/build.py` ve
`python3 -m unittest discover -s tests` — kontrollerden biri bozulursa rapor ❌ gösterir.
