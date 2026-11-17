# YK-250 HANÇER — Devir Notu (yerel PC için)

**Dal:** `ccr-f52f9264-k52gnn` · **Son commit:** `add5cab` (R-02b = 6,0 h, sizing 62/62)
**Durum tarihi:** 17.11.2026

Bu belge, projeyi bu bulut oturumundan yerel bilgisayara taşıyıp devam etmek için yazıldı. Sohbet geçmişi repoya girmez; kararların ve açık işlerin özeti burada.

---

## 1. Yerel kurulum

```
git fetch origin
git checkout ccr-f52f9264-k52gnn
git pull
```

Python 3.11 kullan (bpy 4.5.3 için gerekli). Bağımlılıklar:

```
python3.11 -m pip install -r ucav250/requirements.txt
```

`numpy` 1.26.4'te kalmalı (bpy 4.5.3 `numpy<2` ister). `requirements.txt` dışında paket kurma.

Blender önizlemesi ve GLB için Linux'ta `libEGL` gerekebilir (`apt-get install libegl1`). Windows'ta bu gerekmez.

---

## 2. Doğrulama komutları

Çalışma dizini repo kökü olmalı.

| Ne | Komut | Beklenen |
|---|---|---|
| Spec ile tasarım tutarlı mı | `python3 -m ucav250.layout_build.build --check` | "spec.yaml is up to date" |
| Yerleşim kuralları (C01–C13) | `python3 -m ucav250.analysis.layout_check --check` | 99/99 |
| Yapı marjları | `python3 -m ucav250.analysis.structures --check` | 304 marj, negatif 0 |
| Kütle kaydı | `python3 -m ucav250.analysis.mass` | hatasız, kayıt defteri güncel |
| Boyutlandırma / gereksinimler | `python3 -m ucav250.analysis.sizing --check` | 62/62 gereksinim, 1347/1347 türetilmiş değer |
| Tam uçak denetimi | `python3 -m ucav250.analysis.checks --modules chassis,wing,tail,shell,propulsion,fuel,gear,systems,payload --no-write` | 0 ihlal (yaklaşık 7 dakika) |
| Birim testleri | `python3 -m unittest discover -s tests` | Son bilinen: sizing testleri 49/49 geçti |
| 3D model | `python3 -m ucav250.blender.build --modules chassis,wing,tail,shell,propulsion,fuel,gear,systems,payload --previews --no-blend --glb --out <klasör>` | ~2437 nesne, GLB ~108 MB, 7 önizleme |

**Dikkat:** Tam test paketi ve tam uçak denetimi, son boyutlandırma değişikliğinden (R-02b) sonra yeniden koşulmadı. Yerelde ilk iş olarak bunları çalıştır.

Bilinen not: Blender çıkışında "Not freed memory blocks" uyarısı çıkar. Bu bpy'nin çıkış sırasındaki bir uyarısı; model üretimini bozmaz.

---

## 3. Alınan kararlar

- **R-02b:** Azami 20 kg faydalı yükle dayanım **9,5 saatten 6,0 saate** indirildi (proje lideri kararı, 17.11.2026). Tasarım değişmedi; tasarım 6,05 saat veriyor. Spec'te `R-02b` ve `mission.endurance_max_payload_requirement_h` güncel.
- **Tasarım özellikleri:** D · HANÇER yönü; içeri katlanan takım, köşeli (chine) gövde, içeri çekilen EO/IR taret, açık itici pervane.
- **Kısıt:** Silah, cephane, hardpoint, pylon veya bırakma mekanizması yok. "Hardpoint" kelimesi kullanılmaz.
- **Gövde yapısı:** Kanat, kuyruk, kaporta, itki, yakıt, iniş takımı, sistemler ve faydalı yük modülleri ayrıntılı tasarlandı (`ucav250/design/*.py`).

---

## 4. Güncel sayılar

| Kalem | Değer |
|---|---|
| MTOM | 149,9 kg (M2 sınıfı kapağı) |
| Boş kütle (kayıt defteri + izinler) | 110,706 kg |
| Tasarım görevi dayanımı (9,0 kg yük) | 10,26 saat (R-02 ≥ 10 saat) |
| Azami yük dayanımı (20 kg) | 6,05 saat (R-02b ≥ 6,0 saat) |
| Statik marj minimumu | %10,14 MAC (R-09 ≥ %10; pay ~0,68 mm) |
| Kütle bütçesi (R-56) | +0,034 kg (çok dar) |
| Yapı marjı minimumu | 0,001 (W-CAP-03) |

---

## 5. Açık işler (öncelik sırasıyla)

1. **Kütle bütçesi:** R-56 payı +0,034 kg. Her yeni kütle artışı başka bir kalemden karşılanmalı. Adaylar: şasi hafifletmeleri (0,2–0,5 kg), ağırlık dengeleyici yerleşimleri (0,15–0,2 kg).
2. **Statik marj:** Minimum %10,14; yeni eklenen aft kütle ya da yakıt durumu değişirse kapanış yeniden yapılmalı (yapı, kütle, boyutlandırma, layout_build sırasıyla).
3. **Tam test ve tam uçak denetimi:** Son değişiklikten sonra yerelde koşulmalı (yukarıdaki tablo).
4. **Yerleşim-yapı arayüzü:** Şasi hafifletmesi yapılırsa `structures` satırları yeniden hesaplanmalı (CH-L2, CH-L3, CH-L4 önerileri). Kanat ve gövde kütle tahminleri değişebilir.
5. **Açık tasarım kalemleri:** Kablo demeti dalları ve sensörler (~0,2 kg); geared EMA seçimi (DA 26 yetersiz, DA 30 varsayıldı); egzoz susturucusu (0,6 kg tahmini, kayıt defterinde); fren hatları yönlendirmesi (modellenmedi).
6. **Satın alınan parça tahminleri:** Nutplate, Camloc alıcıları, redüktörlü motor, solenoid, DA 30 flanşı kütleleri zarf tahminidir; tedarikçi verisiyle değiştirilmeli.
7. **Malzeme verileri:** Ti-6Al-4V için levha verisi, işlenmiş çubuk yerine kullanılıyor. Kaplama ve sıcaklık araştırması (kaporta gölgelikleri, kalkan) yapılmalı.
8. **Yakıt seviyesine bağlı ağırlık merkezi:** Hesaplandı (`sizing.py` yakıt seviye tablosu). Son durum: ağırlık merkezi aralığı [2,5738; 2,6067] m.

---

## 6. Teslim belgeleri (henüz yapılmadı)

- GA çizimi (SVG/PDF)
- Malzeme listesi (BOM, CSV)
- Montaj kılavuzu
- Türkçe README ve nihai raporlar
- Son bağımsız inceleme (adversarial review)
- Bazı detay dokümanlarında (`docs/detail/chassis.md` vb.) eski sayılar var; 02 ve 04 güncel kabul edilmeli.

---

## 7. Referans dosyalar

- `ucav250/spec.yaml` — tek doğru kaynak (değerler buradan okunur, elle düzenlenmez; `--update-spec` ile yazılır)
- `ucav250/docs/02_konsept_ve_boyutlandirma.md` — gereksinimler, kararlar, sayılar
- `ucav250/docs/03_yerlesim_ve_yapi_konsepti.md` — yerleşim
- `ucav250/docs/04_yapi_hesaplari.md` — yapı hesapları
- `ucav250/docs/detail/*.md` — modül notları
- `ucav250/docs/reports/closure_round_agent_reports.md` — kapanış turu ajan raporları (ham)
- `ucav250/out/` — üretilen çıktılar (layout, mass, sizing, structures)

---

## 8. Çalışma kuralları (projeden)

- Gereksinimleri, kontrolleri ve testleri gevşetme; açık varsa bunu dürüstçe belgele.
- Kod İngilizce, belgeler ve kullanıcıya dönük metinler Türkçe.
- Git: yalnızca `ccr-f52f9264-k52gnn` dalına push; PR yalnızca istenirse.
- Commit mesajlarına model adı yazma.
