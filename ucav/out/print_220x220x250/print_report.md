# YELKOVAN YK-38 — 3B baskı raporu

Üretim: `ucav/blender/printprep.py` · tarih 2026-10-04 · tabla 220×220×250 mm (kenar payı 6 mm) · boolean: segment kesimi EXACT, kabuk/özellikler MANIFOLD (568 işlem).

## Özet

| | Bu model | Spec planı (kütle bütçesi) | Panel tahmini |
|---|---:|---:|---:|
| Benzersiz STL | 66 | — | — |
| Basılacak parça (ayna dahil) | 105 | 104 | 125 |
| Basılı kütle | 3,46 kg | 3,46 kg | 3,33 kg |
| Baskı süresi (kaba) | 257 h | 257 h | 316 h |
| STL toplamı | yazılmadı (--no-stl) | ≤ 40 MB hedef | |
| Manifold / kapalı | evet, hepsi | | |
| Tablaya sığma | hepsi (3 benzersiz parça dar payla, ≥ 2 mm) | | |

Kütle, filament etkin yoğunluğuyla (LW-PLA köpürmüş 0,65 g/cm³) parça hacminden hesaplanır; boya, yapıştırıcı ve basılmayan parçalar dahil değildir. Süre: hacimsel hız + katman başına 2,5 s + tabla başına 6 dk (kaba tahmin).

| Malzeme | Parça | Hacim (cm³) | Kütle (g) | Süre (h) |
|---|---:|---:|---:|---:|
| LW-ASA | 7 | 413 | 310 | 22,8 |
| LW-PLA | 65 | 3 542 | 2 302 | 180,7 |
| PA-CF | 16 | 575 | 662 | 42,4 |
| PETG | 17 | 170 | 187 | 11,2 |

## Kütle tablosu (R04: hedef ≤ 3,5 kg)

Basılı gövde kütlesi **3,46 kg** (hedefin altında; önceki rapor 4,06 kg). Kalın PA-CF/PETG parçalar (ortalama et > kabuk) dilimleyicideki gibi kabuk + %40 gyroid dolgu ile hesaplanır (PA-CF 1,6 mm, PETG 1,2 mm kabuk); dolu basılsalardı +56 g. Kanat, gövde ve kuyruk dayanımı: kiriş/longeron yükü bilezik ve kaburgalarla deriye geçer, ince deriler stringer ve kafesle bölünür; kök blokları, yük yolu çerçeveleri ve menteşe hatları inceltilmedi. Spec kütle bütçesinin "Basılı:" kalemleri (`mass.breakdown`) bu tablodaki grup kütleleridir; `sizing.py --check` toplamı ve her grubu bu raporla karşılaştırır (≤ 5 g).

| Grup | Önceki rapor (g) | Bu model (g) | Fark (g) | Ne değişti |
|---|---:|---:|---:|---|
| Kanat | 1 444 | 1 269 | -175 | kafes yalnız panel 1–4 (y < 1,19 m), 0,6 mm üye × 90 mm aralık; ana/arka kiriş kovanı yerine 16 mm bilezikler (≤ 55 mm aralık; boru yükü bileziklerden ve kaburgalardan deriye) |
| Kuyruk | 305 | 280 | -25 | dikey kiriş kesme ağı dikey normaline döndürüldü (orta düzlem zarı kalktı); kiriş kovanı → bilezik |
| Kumanda yüzeyleri | 365 | 309 | -56 | orta düzlem levhası yerine 0,5 mm dikey menteşe ağı; menteşe dili R 4,5 mm, çentik yalnız ön yarıda |
| Gövde | 911 | 772 | -139 | halka/modül derisi 0,8 → 0,7 mm + 4 iç stringer (0,8 × 5 mm); çerçeve 8 → 6 mm, flanş 2,4 → 2,0 mm; longeron kovanı → bilezik; keepout içindeki iç yapı artık gerçekten çıkarılıyor (halka 5); halka 9 arka flanşı motor halkası bindirmesinde yalnız deri; modülde PETG flanş cebi |
| Yük yolları | 356 | 312 | -43 | kanat kutusu çerçevesinde depo üstü hafifletme deliği; kalın PA-CF/PETG parçalar dilimleyici dolgusuyla (kabuk + %40 gyroid) hesaplanır; motor halkası NACA çerçevesinden, ana takım beşiği iç flap menteşe dilinden arındırıldı |
| İtki | 277 | 233 | -44 | kaporta print_solid kaynaktan (Ø92 arka açıklık, çene yarığı, motor zarfı boşlukları, panjur dudakları); lüle halkası kaportaya alın alına (geçme dili yok); pabuç PA-CF 2 mm, kaporta yüzünü izler |
| Kaplamalar | 247 | 192 | -55 | kök kaportası ve ER-150 kabartması 1,6 → 1,2 mm; tüy kenar ve iç ince kuşaklar kırpılır |
| Faydalı yük | 80 | 18 | -61 | taret yakası gövde dış yüzünde kesilir: fileto tabanı karın derisine bindirmeli yapışır, tüy kenar 0,8 mm'de kırpılır (gömülü üst kutu ve iç dudak basılmaz; modül derisiyle çakışıyordu) |
| Takım kapakları | 75 | 77 | 2 | burun kapağı çentik köşesi kıymığı kesildi (fark geometri turundaki kapak değişikliğinden) |
| **Toplam** | **4 059** | **3 461** | **-597** | |

## Parça tablosu

Ölçüler baskı yönündedir (X × Y tabla izi, Z yükseklik). **Yön**: tabladan yukarı bakan uçak ekseni; **Z açısı**: tablada izin döndürülmesi (köşegen yerleşim). Ayna (L/R) parçalar tek STL olarak verilir: sağ eş için dilimleyicide X ya da Y'de aynalayın.

### Kanat

| STL | Parça | Malz. | Et (mm) | Adet | Ölçü (mm) | Yön / Z açısı | Pay (mm) | Kütle (g) | Süre (h) | Destek | Not |
|---|---|---|---:|---:|---|---|---:|---:|---:|---|---|
| `wing_panel_1.stl` | Kanat dış paneli 1/7 (y 0.36–0.57) | LW-PLA | 0,6 | 2 | 215×212×210 | +y (sol/iskele) / 49°; tablada: y=0.360 | 2 | 84 | 5,9 | tabla desteği | sağ = ayna; dar pay |
| `wing_panel_2.stl` | Kanat dış paneli 2/7 (y 0.57–0.78) | LW-PLA | 0,6 | 2 | 162×164×210 | +y (sol/iskele) / 46°; tablada: y=0.570 | 28 | 79 | 5,7 | model üstü destek | sağ = ayna |
| `wing_panel_3.stl` | Kanat dış paneli 3/7 (y 0.78–0.99) | LW-PLA | 0,6 | 2 | 163×161×210 | +y (sol/iskele) / 45°; tablada: y=0.780 | 28 | 73 | 5,2 | yok | sağ = ayna |
| `wing_panel_4.stl` | Kanat dış paneli 4/7 (y 0.99–1.20) | LW-PLA | 0,6 | 2 | 202×204×210 | +y (sol/iskele) / 47°; tablada: y=0.990 | 8 | 74 | 5,3 | tabla desteği | sağ = ayna |
| `wing_panel_5.stl` | Kanat dış paneli 5/7 (y 1.20–1.41) | LW-PLA | 0,6 | 2 | 139×139×210 | +y (sol/iskele) / 45°; tablada: y=1.200 | 40 | 60 | 4,5 | tabla desteği | sağ = ayna |
| `wing_panel_6.stl` | Kanat dış paneli 6/7 (y 1.41–1.62) | LW-PLA | 0,6 | 2 | 122×123×210 | +y (sol/iskele) / 134°; tablada: y=1.410 | 49 | 48 | 3,6 | yok | sağ = ayna |
| `wing_panel_7.stl` | Kanat dış paneli 7/7 (y 1.62–1.83) | LW-PLA | 0,6 | 2 | 122×124×215 | +y (sol/iskele) / 41°; tablada: y=1.620 | 48 | 41 | 3,2 | tabla desteği | sağ = ayna |
| `wing_tip.stl` | Kanat uç kapağı (eğik uç, seyrüsefer LED yuvası) | LW-PLA | 0,6 | 2 | 114×116×72 | +y (sol/iskele) (6° eğik) / 49°; tablada: uç düzlemi | 52 | 14 | 1,1 | yok | sağ = ayna |
| `root_block_2.stl` | Kök bloğu 2 (y 0.231–0.36) | LW-PLA | 0,8 | 2 | 212×214×129 | −y (sağ/sancak) / 44°; tablada: y=0.360 (sökülebilir) | 3 | 53 | 3,8 | tabla desteği | sağ = ayna; dar pay |
| `root_block_1.stl` | Kök bloğu 1 (gövde yanı–y 0.231) | LW-PLA | 0,8 | 2 | 214×213×189 | −y (sağ/sancak) / 40°; tablada: y=0.231 | 3 | 83 | 5,8 | tabla desteği | sağ = ayna; gövdeye oturan kök yüzü gövde konturunu izler (0,3 mm boşluk); dar pay |
| `glove_strake.stl` | Glove strake (36° kök hücum kenarı) | LW-PLA | 0,8 | 2 | 124×124×123 | +x (burun) / 38°; tablada: strake ayrımı | 48 | 26 | 2,0 | model üstü destek | sağ = ayna |

### Kuyruk

| STL | Parça | Malz. | Et (mm) | Adet | Ölçü (mm) | Yön / Z açısı | Pay (mm) | Kütle (g) | Süre (h) | Destek | Not |
|---|---|---|---:|---:|---|---|---:|---:|---:|---|---|
| `stab_1.stl` | Stabilize yarısı 1/2 (%25 ok eksenine dik ek) | LW-ASA | 0,5 | 2 | 107×107×245 | −y (sağ/sancak) (29° eğik) / 107°; tablada: stabilize eki | 56 | 34 | 2,7 | tabla desteği | sağ = ayna; kök yüzü kuyruk konisi konturunu izler; borular koni içindeki eyerde birleşir |
| `stab_2a.stl` | Stabilize yarısı 2/2 (%25 ok eksenine dik ek) — parça A (tablaya sığması için bölündü) | LW-PLA | 0,5 | 2 | 64×63×172 | +y (sol/iskele) (29° eğik) / 106°; tablada: stabilize eki | 78 | 20 | 1,8 | yok | sağ = ayna; uç dikey içine 5 mm gömülür |
| `stab_2b.stl` | Stabilize yarısı 2/2 (%25 ok eksenine dik ek) — parça B (tablaya sığması için bölündü) | LW-PLA | 0,5 | 2 | 78×77×172 | +y (sol/iskele) (29° eğik) / 106°; tablada: stabilize eki | 71 | 14 | 1,4 | tabla desteği | sağ = ayna; uç dikey içine 5 mm gömülür |
| `fin_1.stl` | Dikey 1/2 (%25 ok eksenine dik ek) | LW-PLA | 0,5 | 2 | 153×153×220 | −z (aşağı) (29° eğik) / 3°; tablada: dikey eki | 33 | 47 | 3,6 | tabla desteği | sağ = ayna; iç yüzde stabilize ucu yuvası (0,3 mm boşluk) |
| `fin_2.stl` | Dikey 2/2 (%25 ok eksenine dik ek) | LW-PLA | 0,5 | 2 | 99×99×226 | +z (yukarı) (29° eğik) / 42°; tablada: dikey eki | 60 | 24 | 2,2 | tabla desteği | sağ = ayna |

### Kumanda yüzeyleri

| STL | Parça | Malz. | Et (mm) | Adet | Ölçü (mm) | Yön / Z açısı | Pay (mm) | Kütle (g) | Süre (h) | Destek | Not |
|---|---|---|---:|---:|---|---|---:|---:|---:|---|---|
| `aileron_1.stl` | Kanatçık 1/4 | LW-PLA | 0,5 | 2 | 64×64×182 | +y (sol/iskele) (6° eğik) / 57° | 78 | 12 | 1,3 | yok | sağ = ayna; menteşe ekseni Z'de; Ø1,75 filament pim, basılı dil çentikleri |
| `aileron_2.stl` | Kanatçık 2/4 | LW-PLA | 0,5 | 2 | 56×57×183 | +y (sol/iskele) (6° eğik) / 56°; tablada: yüzey eki | 82 | 11 | 1,2 | yok | sağ = ayna; menteşe ekseni Z'de; Ø1,75 filament pim, basılı dil çentikleri |
| `aileron_3.stl` | Kanatçık 3/4 | LW-PLA | 0,5 | 2 | 50×50×183 | +y (sol/iskele) (6° eğik) / 55°; tablada: yüzey eki | 85 | 10 | 1,2 | yok | sağ = ayna; menteşe ekseni Z'de; Ø1,75 filament pim, basılı dil çentikleri |
| `aileron_4.stl` | Kanatçık 4/4 | LW-PLA | 0,5 | 2 | 43×43×182 | +y (sol/iskele) (6° eğik) / 55°; tablada: yüzey eki | 88 | 9 | 1,1 | yok | sağ = ayna; menteşe ekseni Z'de; Ø1,75 filament pim, basılı dil çentikleri |
| `flapin.stl` | İç flap | LW-PLA | 0,5 | 2 | 68×68×173 | +y (sol/iskele) (4° eğik) / 143° | 76 | 14 | 1,4 | yok | sağ = ayna; menteşe ekseni Z'de; Ø1,75 filament pim, basılı dil çentikleri |
| `flapout_1.stl` | Dış flap 1/4 | LW-PLA | 0,5 | 2 | 69×68×178 | +y (sol/iskele) (4° eğik) / 53° | 75 | 14 | 1,4 | model üstü destek | sağ = ayna; menteşe ekseni Z'de; Ø1,75 filament pim, basılı dil çentikleri |
| `flapout_2.stl` | Dış flap 2/4 | LW-PLA | 0,5 | 2 | 70×70×179 | +y (sol/iskele) (4° eğik) / 143°; tablada: yüzey eki | 75 | 14 | 1,4 | yok | sağ = ayna; menteşe ekseni Z'de; Ø1,75 filament pim, basılı dil çentikleri |
| `flapout_3.stl` | Dış flap 3/4 | LW-PLA | 0,5 | 2 | 71×71×179 | +y (sol/iskele) (4° eğik) / 53°; tablada: yüzey eki | 74 | 15 | 1,5 | model üstü destek | sağ = ayna; menteşe ekseni Z'de; Ø1,75 filament pim, basılı dil çentikleri |
| `flapout_4.stl` | Dış flap 4/4 | LW-PLA | 0,5 | 2 | 72×71×178 | +y (sol/iskele) (4° eğik) / 53°; tablada: yüzey eki | 74 | 14 | 1,4 | tabla desteği | sağ = ayna; menteşe ekseni Z'de; Ø1,75 filament pim, basılı dil çentikleri |
| `elevator_1.stl` | Elevatör 1/2 | LW-ASA | 0,5 | 2 | 112×112×217 | +y (sol/iskele) / 45° | 54 | 13 | 1,4 | yok | sağ = ayna; menteşe ekseni Z'de; Ø1,75 filament pim, basılı dil çentikleri |
| `elevator_2a.stl` | Elevatör 2/2 — parça A (tablaya sığması için bölündü) | LW-PLA | 0,5 | 2 | 38×38×129 | +y (sol/iskele) (27° eğik) / 18°; tablada: yüzey eki | 91 | 6 | 0,8 | yok | sağ = ayna; menteşe ekseni Z'de; Ø1,75 filament pim, basılı dil çentikleri |
| `elevator_2b.stl` | Elevatör 2/2 — parça B (tablaya sığması için bölündü) | LW-PLA | 0,5 | 2 | 35×35×130 | +y (sol/iskele) (27° eğik) / 18°; tablada: yüzey eki | 92 | 5 | 0,7 | yok | sağ = ayna; menteşe ekseni Z'de; Ø1,75 filament pim, basılı dil çentikleri |
| `rudder_a.stl` | Dümen — parça A (tablaya sığması için bölündü) | LW-PLA | 0,5 | 2 | 90×90×151 | +z (yukarı) (12° eğik) / 45° | 65 | 10 | 1,1 | yok | sağ = ayna; menteşe ekseni Z'de; Ø1,75 filament pim, basılı dil çentikleri |
| `rudder_b.stl` | Dümen — parça B (tablaya sığması için bölündü) | LW-PLA | 0,5 | 2 | 52×52×162 | +z (yukarı) (24° eğik) / 43° | 84 | 8 | 1,0 | yok | sağ = ayna; menteşe ekseni Z'de; Ø1,75 filament pim, basılı dil çentikleri |

### Gövde

| STL | Parça | Malz. | Et (mm) | Adet | Ölçü (mm) | Yön / Z açısı | Pay (mm) | Kütle (g) | Süre (h) | Destek | Not |
|---|---|---|---:|---:|---|---|---:|---:|---:|---|---|
| `fus_nose_cone.stl` | Burun konisi (s 0–0.148) | LW-PLA | 0,7 | 1 | 135×128×148 | +x (burun) / 71°; tablada: s=0.148 | 43 | 20 | 1,7 | tabla desteği |  |
| `fus_nose_module_a.stl` | Burun görev modülü halkası (s 0.148–0.400, taret yuvası) — parça A (tablaya sığması için bölündü) | LW-PLA | 0,7 | 1 | 193×195×126 | +x (burun) / 11°; tablada: s=0.400 (sökülebilir) | 12 | 40 | 2,9 | yok |  |
| `fus_nose_module_b.stl` | Burun görev modülü halkası (s 0.148–0.400, taret yuvası) — parça B (tablaya sığması için bölündü) | LW-PLA | 0,7 | 1 | 174×177×126 | +x (burun) / 18°; tablada: tabla bölmesi | 21 | 40 | 2,9 | tabla desteği |  |
| `fus_ring_1.stl` | Gövde halkası 1 (s 0.400–0.592) | LW-PLA | 0,7 | 1 | 201×201×192 | −x (kuyruk) / 10°; tablada: s=0.400 | 9 | 82 | 5,8 | tabla desteği |  |
| `fus_ring_2.stl` | Gövde halkası 2 (s 0.592–0.783) | LW-PLA | 0,7 | 1 | 201×201×192 | −x (kuyruk) / 11°; tablada: s=0.592 | 10 | 85 | 6,0 | tabla desteği |  |
| `fus_ring_3.stl` | Gövde halkası 3 (s 0.783–0.975) | LW-PLA | 0,7 | 1 | 203×203×192 | −x (kuyruk) / 8°; tablada: s=0.783 | 9 | 75 | 5,3 | tabla desteği |  |
| `fus_ring_4.stl` | Gövde halkası 4 (s 0.975–1.167) | LW-PLA | 0,7 | 1 | 202×202×192 | −x (kuyruk) / 6°; tablada: s=0.975 | 9 | 70 | 5,0 | yok |  |
| `fus_ring_5.stl` | Gövde halkası 5 (s 1.167–1.379) | LW-PLA | 0,7 | 1 | 206×196×212 | −x (kuyruk) / 0°; tablada: s=1.167 | 7 | 71 | 5,2 | tabla desteği |  |
| `fus_ring_6.stl` | Gövde halkası 6 (s 1.379–1.550) | LW-PLA | 0,7 | 1 | 194×194×171 | −x (kuyruk) / 45°; tablada: s=1.379 | 13 | 57 | 4,2 | yok |  |
| `fus_ring_7.stl` | Kuyruk konisi halkası (s 1.550–1.715, LW-ASA) | LW-ASA | 1,0 | 1 | 184×184×165 | −x (kuyruk) / 45°; tablada: s=1.550 | 18 | 76 | 5,1 | yok |  |
| `fus_ring_8.stl` | Kuyruk konisi halkası (s 1.715–1.845, LW-ASA) | LW-ASA | 1,0 | 1 | 173×173×130 | −x (kuyruk) / 45°; tablada: s=1.715 | 24 | 58 | 3,9 | yok |  |
| `fus_ring_9.stl` | Kuyruk konisi halkası (s 1.845–2.042, LW-ASA) | LW-ASA | 1,0 | 1 | 171×171×198 | −x (kuyruk) / 45°; tablada: s=1.845 | 24 | 82 | 5,5 | model üstü destek | NACA dudak çerçevesi cebi + yangın perdesine kanal geçişi |
| `fus_nose_flange.stl` | Burun modülü PETG bağlantı flanşı (4×M4, 2 pim) | PETG | 3,0 | 1 | 189×191×3 | −x (kuyruk) / 11° | 15 | 17 | 0,8 | yok |  |

### Yük yolları

| STL | Parça | Malz. | Et (mm) | Adet | Ölçü (mm) | Yön / Z açısı | Pay (mm) | Kütle (g) | Süre (h) | Destek | Not |
|---|---|---|---:|---:|---|---|---:|---:|---:|---|---|
| `wing_frame_fwd.stl` | Kanat kutusu ön çerçevesi (PA-CF 3 mm; G10 köprüye yaslanır) | PA-CF | 1,6 | 1 | 191×189×13 | +x (burun) / 0° | 14 | 52 | 3,0 | yok | köprü plakasına epoksi + 2 × M4; 4 longeron soketi; depo zarfı çevresinde halka |
| `wing_frame_aft.stl` | Kanat kutusu arka çerçevesi (PA-CF 3 mm; G10 köprüye yaslanır) | PA-CF | 1,6 | 1 | 190×184×13 | −x (kuyruk) / 0° | 15 | 50 | 2,9 | yok | köprü plakasına epoksi + 2 × M4; 4 longeron soketi |
| `gear_mount_L.stl` | Ana takım yatağı (PA-CF; ER-150 beşiği + sokete eyerli kol) | PA-CF | 1,6 | 2 | 114×85×44 | +z (yukarı) (4° eğik) / 0° | 53 | 37 | 2,3 | tabla desteği | sağ = ayna; kök bloğu 2'nin açık iç ucundan kaydırılır; ünite ön duvara 2 × M3 ısıl gömme dişli |
| `gear_mount_N.stl` | Burun takım yatağı (PA-CF; chine longeronlarına eyer, akü kızağı tabanı) | PA-CF | 1,6 | 1 | 195×195×10 | +z (yukarı) / 45° | 12 | 53 | 3,1 | tabla desteği | halka 1'in ön çerçevesinden kaydırılır; G10 plaka 4 × M3 ısıl gömme dişli (M3×3) |
| `engine_ring.stl` | Motor halkası (PA-CF; yangın perdesi önü, 4 × M4 motor + 6 × M3 kaporta dişlisi) | PA-CF | 1,6 | 1 | 154×154×31 | +x (burun) / 45° | 33 | 64 | 3,8 | tabla desteği | halka 9'a 30 mm bindirerek yapıştırılır; 4 longeron ucu soketlere girer |
| `panel_lock_tab.stl` | Dış panel tutma dili (PETG; alttan M4 naylon cıvata, ısıl gömme dişli) | PETG | 1,6 | 2 | 34×34×8 | +z (yukarı) / 45° | 93 | 3 | 0,3 | yok | sağ = ayna; dış panel 1 kök kaburgasına yapıştırılır; kök bloğu 2 cebine girer |
| `longeron_joint_chine.stl` | Longeron kırık soketi — chine (PETG; s = 1.167) | PETG | 1,6 | 2 | 14×14×37 | −x (kuyruk) / 69° | 103 | 3 | 0,4 | yok | sağ = ayna; iki düz longeron parçası kırık açısıyla buluşur (halka 4–5 eki) |
| `longeron_joint_shoulder.stl` | Longeron kırık soketi — omuz (PETG; s = 1.167) | PETG | 1,6 | 2 | 14×14×37 | −x (kuyruk) / 163° | 103 | 4 | 0,4 | yok | sağ = ayna; iki düz longeron parçası kırık açısıyla buluşur (halka 4–5 eki) |

### İtki

| STL | Parça | Malz. | Et (mm) | Adet | Ölçü (mm) | Yön / Z açısı | Pay (mm) | Kütle (g) | Süre (h) | Destek | Not |
|---|---|---|---:|---:|---|---|---:|---:|---:|---|---|
| `cowl_top.stl` | Motor kaportası (PA-CF; yanak açıklıkları flanşlı, arka Ø92 açık) | PA-CF | 1,6 | 1 | 161×160×142 | −x (kuyruk) / 45°; tablada: yangın perdesi | 30 | 76 | 4,8 | tabla desteği |  |
| `cowl_cheek_L.stl` | Kaporta sol yanak (susturucu tarafı) | PA-CF | 1,6 | 1 | 142×142×130 | −x (kuyruk) / 33° | 39 | 81 | 5,0 | tabla desteği | eğik ekler deriye dik (deri normali 52°) |
| `cowl_cheek_R.stl` | Kaporta sağ yanak (panjurlu) | PA-CF | 1,6 | 1 | 90×89×116 | −x (kuyruk) / 131° | 65 | 41 | 2,8 | yok | eğik ekler deriye dik (deri normali 37°, -29°) |
| `exhaust_ring.stl` | Lüle halkası (PA-CF) | PA-CF | 1,6 | 1 | 112×112×15 | −x (kuyruk) / 3° | 54 | 25 | 1,5 | model üstü destek |  |
| `scuff_pad.stl` | Kaporta altı sürtünme pabucu (PA-CF; kuyruk çarpmasında ilk değen kenar) | PA-CF | 2,0 | 1 | 49×49×8 | +z (yukarı) / 45° | 85 | 4 | 0,3 | tabla desteği | kaporta dış yüzünü izleyen 2 mm katman (yapışma yüzü + 0,2 mm); aşınma yüzü tablada; yüksek sıcaklık epoksisiyle yapıştırılır, aşınınca sökülüp yenilenir |
| `intake.stl` | NACA hava alığı dudak çerçevesi (PA-CF, karın) | PA-CF | 1,6 | 1 | 70×70×10 | +x (burun) / 45° | 75 | 6 | 0,5 | yok | gövde halkası 9'daki cebe yapıştırılır; kanal yangın perdesindeki açıklıktan kaportaya geçer |

### Kaplamalar

| STL | Parça | Malz. | Et (mm) | Adet | Ölçü (mm) | Yön / Z açısı | Pay (mm) | Kütle (g) | Süre (h) | Destek | Not |
|---|---|---|---:|---:|---|---|---:|---:|---:|---|---|
| `root_fairing.stl` | Kök kaportası — kano (PA-CF) | PA-CF | 1,2 | 2 | 202×201×155 | +y (sol/iskele) (13° eğik) / 124° | 9 | 49 | 3,4 | tabla desteği | sağ = ayna; 1,6 mm kabuk, et < 3,6 mm olan kenar/dudaklar dolu; tüy kenar ≥ 0,8 mm'de kırpılır, kalan kama epoksi + mikrobalon ile sıfırlanır |
| `gear_blister.stl` | ER-150 ünite kabartması (PA-CF) | PA-CF | 1,2 | 2 | 78×77×141 | −y (sağ/sancak) / 136° | 71 | 19 | 1,6 | yok | sağ = ayna; kano kaportasına ve kanat altına yapıştırılır; ince kenarlar dolu, tüy kenar kırpılır |
| `hatch_frame_a.stl` | Aviyonik kapağı çerçevesi A (PETG; 3 mıknatıs cebi) | PETG | 1,6 | 1 | 176×176×13 | +z (yukarı) (2° eğik) / 45° | 22 | 26 | 1,3 | yok | levha altına yapıştırılır; düz alt yüz gövde basamağına oturur |
| `hatch_frame_b.stl` | Aviyonik kapağı çerçevesi B (PETG; 3 mıknatıs cebi) | PETG | 1,6 | 1 | 180×108×15 | +z (yukarı) (2° eğik) / 0° | 20 | 30 | 1,4 | yok | levha altına yapıştırılır; düz alt yüz gövde basamağına oturur |

### Faydalı yük

| STL | Parça | Malz. | Et (mm) | Adet | Ölçü (mm) | Yön / Z açısı | Pay (mm) | Kütle (g) | Süre (h) | Destek | Not |
|---|---|---|---:|---:|---|---|---:|---:|---:|---|---|
| `turret_collar.stl` | Taret yakası (24 faset + karın filetosu, PETG) | PETG | 1,6 | 1 | 105×100×31 | +z (yukarı) / 6° | 58 | 18 | 1,0 | tabla desteği | gövde dış yüzünde kesilir (montaj denetimi: modül derisiyle 2,4 cm³ çakışıyordu): fileto tabanı karın derisine bindirmeli yapışır; tüy kenar ≥ 0,8 mm'de kırpılır, kalan kama epoksi + mikrobalonla sıfırlanır (R04: sahnedeki gömülü üst kutu basılmaz) |

### Kalıp (alet)

| STL | Parça | Malz. | Et (mm) | Adet | Ölçü (mm) | Yön / Z açısı | Pay (mm) | Kütle (g) | Süre (h) | Destek | Not |
|---|---|---|---:|---:|---|---|---:|---:|---:|---|---|
| `hatch_buck_a.stl` | Aviyonik kapağı ısıl biçimlendirme kalıbı A (PLA; alet) | PLA | 1,6 | 1 | 191×126×31 | +z (yukarı) / 0° | 14 | 139 | 3,9 | yok | 1,0 mm füme PETG levha bu kalıp üstünde biçimlendirilir; uçak kütlesine dahil değil |
| `hatch_buck_b.stl` | Aviyonik kapağı ısıl biçimlendirme kalıbı B (PLA; alet) | PLA | 1,6 | 1 | 198×126×32 | +z (yukarı) / 0° | 11 | 148 | 4,2 | yok | 1,0 mm füme PETG levha bu kalıp üstünde biçimlendirilir; uçak kütlesine dahil değil |

### Test / alet

| STL | Parça | Malz. | Et (mm) | Adet | Ölçü (mm) | Yön / Z açısı | Pay (mm) | Kütle (g) | Süre (h) | Destek | Not |
|---|---|---|---:|---:|---|---|---:|---:|---:|---|---|
| `tolerance_coupon.stl` | Tolerans kuponu (LW-PLA; ilk baskı — delik payı denemesi) | LW-PLA | 0,6 | 1 | 96×95×20 | +z (yukarı) / 133° | 62 | 7 | 0,6 | yok | Ø27/16/8/6 boru, Ø3 pim, Ø1,75 menteşe pimi delikleri (parçalardaki payla aynı) |

### Takım kapakları

| STL | Parça | Malz. | Et (mm) | Adet | Ölçü (mm) | Yön / Z açısı | Pay (mm) | Kütle (g) | Süre (h) | Destek | Not |
|---|---|---|---:|---:|---|---|---:|---:|---:|---|---|
| `door_N_1.stl` | Burun takımı kapağı (menteşe dili + horn dahil) (PETG 1,2 mm) | PETG | 1,2 | 2 | 182×183×25 | +y (sol/iskele) / 45° | 19 | 12 | 0,7 | tabla desteği | sağ = ayna |
| `door_N_3.stl` | Burun takımı bacak tapa kapağı (bacağa bağlı) (PETG 1,2 mm) | PETG | 1,2 | 1 | 24×18×29 | −z (aşağı) / 0° | 98 | 2 | 0,3 | yok |  |
| `door_L_1.stl` | Ana takım kuyu kapağı (menteşe dili + horn dahil) (PETG 1,2 mm) | PETG | 1,2 | 2 | 94×92×8 | +z (yukarı) / 0° | 63 | 19 | 1,0 | yok | sağ = ayna |
| `door_L_2.stl` | Ana takım bacak kapağı (PETG 1,2 mm) | PETG | 1,2 | 2 | 134×134×23 | −y (sağ/sancak) / 149° | 43 | 6 | 0,5 | tabla desteği | sağ = ayna |

## Doğrulama

Her parça bmesh ile denetlenir: sınır kenar = 0, manifold olmayan kenar = 0, tel kenar = 0, yönü tutarsız kenar = 0, işaretli hacim > 0. Et ölçümü: yüzeyden içe ışın (alan ağırlıklı 1200 örnek); %5'lik değer nominal et ya da en ince iç yapıdır (kanat panellerinde 0,6 mm geodezik kafes, gövde halkalarında 0,7 mm deri); en küçük değer köşe/pah örneklerini ve kaplamaların sıfıra incelen yapışma kenarlarını içerir.

| STL | Kenar denetimi (yeniden okuma) | Hacim farkı | Blender içe aktarma |
|---|---|---:|---|

Yazılan tüm STL'lerde sıfır alanlı üçgen (float32 mm, |AB×AC| < 1e-09 mm²): 0. Float32 çözünürlüğünde doğrusal ince üçgen (uzun kenarına yükseklik < 10 nm): 0 kaldı; dışa aktarımda komşu üçgeni bölerek giderilen: 0 (yüzey ≤ 10 nm oynar, hacim ve kapalılık korunur). Kalanlar art arda doğrusal üçgenlerdir; dilimleyici için etkisizdir.

Manifold olmayan parça: 0. 

| Parça | Et %5 (mm) | Et medyan (mm) | Çıkıntı (cm²) |
|---|---:|---:|---:|
| wing_panel_1 | 0,60 | 0,80 | 21,6 |
| wing_panel_2 | 0,60 | 0,80 | 20,3 |
| wing_panel_3 | 0,60 | 0,80 | 5,9 |
| wing_panel_4 | 0,60 | 0,61 | 24,9 |
| wing_panel_5 | 0,60 | 0,80 | 14,6 |
| wing_panel_6 | 0,60 | 0,80 | 3,4 |
| wing_panel_7 | 0,60 | 0,80 | 7,2 |
| wing_tip | 0,60 | 0,60 | 5,5 |
| root_block_2 | 0,79 | 0,83 | 20,8 |
| root_block_1 | 0,78 | 0,80 | 82,5 |
| glove_strake | 0,80 | 1,20 | 34,9 |
| stab_1 | 0,50 | 0,50 | 26,2 |
| stab_2a | 0,50 | 0,50 | 6,0 |
| stab_2b | 0,49 | 0,50 | 16,1 |
| fin_1 | 0,49 | 0,50 | 72,3 |
| fin_2 | 0,50 | 0,50 | 16,7 |
| aileron_1 | 0,49 | 0,50 | 6,1 |
| aileron_2 | 0,50 | 0,50 | 5,3 |
| aileron_3 | 0,50 | 0,50 | 3,0 |
| aileron_4 | 0,50 | 0,50 | 2,7 |
| flapin | 0,49 | 0,50 | 7,7 |
| flapout_1 | 0,49 | 0,50 | 9,0 |
| flapout_2 | 0,50 | 0,50 | 7,6 |
| flapout_3 | 0,50 | 0,50 | 9,0 |
| flapout_4 | 0,50 | 0,50 | 8,5 |
| elevator_1 | 0,50 | 0,50 | 3,4 |
| elevator_2a | 0,50 | 0,50 | 4,6 |
| elevator_2b | 0,50 | 0,50 | 2,9 |
| rudder_a | 0,50 | 0,50 | 6,0 |
| rudder_b | 0,50 | 0,50 | 5,3 |
| fus_nose_cone | 0,70 | 0,70 | 5,6 |
| fus_nose_module_a | 0,70 | 0,70 | 0,3 |
| fus_nose_module_b | 0,69 | 0,70 | 53,5 |
| fus_ring_1 | 0,69 | 0,70 | 76,0 |
| fus_ring_2 | 0,69 | 0,70 | 49,4 |
| fus_ring_3 | 0,70 | 0,70 | 28,6 |
| fus_ring_4 | 0,70 | 0,70 | 1,9 |
| fus_ring_5 | 0,70 | 0,70 | 15,2 |
| fus_ring_6 | 0,70 | 0,70 | 2,8 |
| fus_ring_7 | 0,80 | 1,00 | 3,0 |
| fus_ring_8 | 0,80 | 1,00 | 2,0 |
| fus_ring_9 | 0,80 | 1,00 | 10,9 |
| wing_frame_fwd | 1,98 | 3,00 | 0,0 |
| wing_frame_aft | 1,98 | 3,00 | 0,0 |
| gear_mount_L | 2,20 | 2,20 | 21,7 |
| gear_mount_N | 1,95 | 4,30 | 10,8 |
| engine_ring | 2,17 | 3,00 | 6,7 |
| panel_lock_tab | 4,07 | 8,00 | 0,0 |
| longeron_joint_chine | 1,42 | 2,17 | 0,9 |
| longeron_joint_shoulder | 2,16 | 2,18 | 1,0 |
| fus_nose_flange | 3,00 | 3,00 | 0,0 |
| cowl_top | 1,50 | 1,60 | 79,2 |
| cowl_cheek_L | 1,59 | 1,60 | 113,1 |
| cowl_cheek_R | 1,60 | 1,60 | 16,3 |
| exhaust_ring | 1,58 | 1,60 | 22,4 |
| scuff_pad | 2,00 | 2,00 | 14,6 |
| intake | 2,07 | 2,50 | 0,0 |
| root_fairing | 1,17 | 1,20 | 35,9 |
| gear_blister | 1,18 | 1,20 | 1,3 |
| turret_collar | 1,51 | 1,60 | 26,8 |
| hatch_buck_a | 19,08 | 29,34 | 0,6 |
| hatch_frame_a | 2,41 | 7,80 | 1,3 |
| hatch_buck_b | 17,67 | 30,03 | 0,6 |
| hatch_frame_b | 2,32 | 7,80 | 0,6 |
| tolerance_coupon | 5,29 | 20,00 | 0,0 |
| door_N_1 | 0,79 | 1,39 | 7,7 |
| door_N_3 | 1,20 | 5,00 | 1,1 |
| door_L_1 | 0,80 | 1,80 | 0,4 |
| door_L_2 | 0,80 | 1,20 | 38,6 |

## Yük yolları (P4)

| Yük | Yol | Basılı parçalar |
|---|---|---|
| Kanat kaldırması (yarım kanat ≈ 11,7 kg × n) | dış panel derisi + Ø27 panel borusu → Ø30 merkez soket → G10 köprü (2 × 3 mm, soketin iki yanında 3,3 mm yuvalarda, boru başına 2 × M4) → kanat kutusu çerçeveleri (s 1,20 / 1,29) → 4 longeron + halka 5 | wing_panel_*, root_block_1/2, wing_frame_fwd, wing_frame_aft, fus_ring_5 |
| Kanat burulması / hücum açısı | Ø6 dolu CF açı pimi (y 0,334–0,434, %60 veter) + Ø8 arka kiriş → kök bloğu arka ağı → çerçeveler | wing_panel_1, root_block_2, root_block_1 |
| Dış panel eki (eksenel / sökülebilir) | PETG tutma dili (panel 1 kök kaburgası) → M4 naylon cıvata → kök bloğu 2'deki M4 ısıl gömme dişli | panel_lock_tab, wing_panel_1, root_block_2 |
| Ana takım (iniş darbesi, fren) | ER-150 ünitesi → G10 3 mm plaka → ana takım yatağı (PA-CF beşik, 2 × M3) → sokete eyerli kol → Ø30 soket → G10 köprü → çerçeveler / longeronlar | gear_mount_L (+ ayna), root_block_2, wing_frame_* |
| Burun takımı | burun ünitesi → G10 plaka (4 × M3 kısa dişli) → burun yatağı (PA-CF, iki chine longeronuna eyer) → longeronlar → halka 1 çerçevesi | gear_mount_N, fus_ring_1 |
| İtki + titreşim (DLE-20, 1,86 kW, tek silindir) | motor → 4 × M4 + kauçuk takoz → motor halkası (PA-CF, halka 9 içinde 30 mm bindirme, 4 longeron soketi) → longeronlar; G10 yangın perdesi halkaya yaslanır | engine_ring, fus_ring_9 |
| Kuyruk (stabilize + dikey yükleri) | Ø12 stabilize kirişi + Ø6 arka çubuk → koni içi G10 eyer → kuyruk konisi (LW-ASA) → s = 1,55 G10 flanşı (4 × M4) → longeronlar | stab_1/2, fin_1/2, fus_ring_7–9 |
| Longeron sürekliliği | düz CF parçalar s = 1,167 ve 1,55 kırıklarında PETG soket bloklarına 18 mm girer (epoksi) | longeron_joint_chine, longeron_joint_shoulder, fus_ring_4/5 |

## Isı kuralı (AERO-09, R05)

Kural 1: silindir (+ buji başlığı) ve susturucu zarflarının ve susturucu çıkış borusu/ısı kalkanının 150 mm yakınında LW-PLA yok. Kural 2: ısıl sınırı 120 °C'nin altındaki hiçbir filament (LW-ASA 95, PETG 80, LW-PLA 55 °C) bu kaynaklara 50 mm'den yakın değil (zarflar `params.engine_envelope`, çıkış `U_Exhaust_Muffler`). Isıl sınır amorf filamentlerde camsı geçiş (Tg), yarı kristal PA-CF'de ısıl eğilme sıcaklığıdır (HDT, 0,45 MPa; PA-CF'nin Tg'si ≈ 60 °C). Kural 3: hiçbir parça susturucu çıkış borusuna 5 mm'den yakın değil. Printprep malzemeyi kendiliğinden değiştirmez: ihlal varsa `build.py --print` ve `printprep` 1 ile çıkar. Isı bölgesindeki parçaların filamenti spec'te adıyla atanır (`print.zones[].parts`). Sonuç: **uygun**.

Kaynaklar ayrı adlandırılır: motor zarfı parçaları, susturucu çıkış borusu ve borunun çıktığı yerde sol yanak dış yüzüne (0,1–0,5 mm aralıkla) oturan 0,5 mm Al ısı kalkanı (kalkan yanağı borudan korur; yanak PA-CF, HDT ≈ 150 °C). Boru yanaktaki deliğinden 6 mm radyal boşlukla geçer; delikteki geçiş halkası boruyu ortalar ve sıcak gazı yanağa değdirmez (BOM: Isıya dayanıklı geçiş halkası (silikon/seramik keçe), iç Ø12,4 / dış Ø24,4 mm). Boru ve kalkan uzaklıkları kaynak yüzeyinin yoğun örneklerinden parça yüzeyine, zarf uzaklıkları parça köşelerinden ölçülür; 10 mm'nin altındakiler 0,1 mm çözünürlükle verilir; son sütun parçanın en yakın üç kaynağıdır.

Spec ataması: `stab_1` → LW-ASA (stab_root_heat; kaynağa 61 mm); `elevator_1` → LW-ASA (stab_root_heat; kaynağa 57 mm)

| Parça (250 mm içinde) | Malzeme | Isıl sınır (°C) | En yakın (mm) | Kaynak | Kaynaklara uzaklık (mm) |
|---|---|---:|---:|---|---|
| cowl_cheek_L | PA-CF | 150 | 0,1 | ısı kalkanı | ısı kalkanı 0,1; susturucu çıkış borusu 6,0; silindir 7,0 |
| cowl_top | PA-CF | 150 | 5,1 | silindir | silindir 5,1; buji başlığı 7,0; susturucu 35 |
| scuff_pad | PA-CF | 150 | 8,8 | buji başlığı | buji başlığı 8,8; silindir 19; susturucu 42 |
| cowl_cheek_R | PA-CF | 150 | 10 | silindir | silindir 10; buji başlığı 36; susturucu 65 |
| exhaust_ring | PA-CF | 150 | 20 | silindir | silindir 20; susturucu 28; susturucu çıkış borusu 45 |
| elevator_1 | LW-ASA | 95 | 57 | susturucu | susturucu 57; ısı kalkanı 75; silindir 78 |
| stab_1 | LW-ASA | 95 | 61 | susturucu | susturucu 61; silindir 77; susturucu çıkış borusu 90 |
| fus_ring_9 | LW-ASA | 95 | 74 | susturucu | susturucu 74; silindir 86; buji başlığı 104 |
| engine_ring | PA-CF | 150 | 74 | susturucu | susturucu 74; silindir 80; susturucu çıkış borusu 106 |
| intake | PA-CF | 150 | 92 | susturucu | susturucu 92; silindir 97; buji başlığı 115 |
| stab_2a | LW-PLA | 55 | 164 | ısı kalkanı | ısı kalkanı 164; susturucu 177; susturucu çıkış borusu 178 |
| elevator_2a | LW-PLA | 55 | 176 | ısı kalkanı | ısı kalkanı 176; susturucu çıkış borusu 179; susturucu 199 |

## Menteşe pimleri (P1)

Pim: Ø1,75 mm PETG filament (çelik tel yerine; esnek, paslanmaz, kesilip ısıyla ucu mantarlanabilir). Delik Ø2,1 mm çevrel çokgen (iç yarıçap 1,05 mm). Her yüzeyin tek bir düz takma yolu vardır; delik takma tarafında komşu parçadan dışarı uzatılmıştır.

| Yüzey | Dil | Pim boyu (mm) | Takma ucu | Yol | Tutma |
|---|---:|---:|---|---|---|
| Aileron | 8 | 860 | dış uç | uç kapağının dış yüzündeki Ø2,1 delikten (uç kapağı yapıştırıldıktan sonra) | iç uçta kör delik (dış panel / dış flap ucu); dış uçta delik CA + mikrobalonla tıkanır |
| FlapIn | 1 | 187 | dış uç | dış panel sökülüyken kök bloğu 2'nin y = 0,36 kaburgasındaki delikten | panel takılınca panel 1 kaburgası ve dış flap piminin ucu; iç uçta kör delik (kök bloğu 1) |
| FlapOut | 7 | 729 | iç uç | dış panel sökülüyken panel 1'in y = 0,36 kök kaburgasındaki delikten | panel takılınca kök bloğu 2 kaburgası ve iç flap piminin ucu; dış uçta kör delik (kanatçık ayrımı) |
| Elevator | 4 | 538 | dış uç | dikeyler takılmadan önce stabilize ucundaki delikten | dikey 1'in stabilize ucu yuvası (dikey takılınca elevatör pimi sökülemez); iç uçta kör delik |
| Rudder | 2 | 376 | üst uç | dikey 2'nin tepesindeki Ø2,1 delikten | alt uçta kör delik (dikey 1); üst delik CA ile tıkanır |

## Tolerans kuponu (P7)

**Önce kupon basın, gerekirse TUBE_CLEAR ayarlayın.** `tolerance_coupon.stl`: 20 mm LW-PLA blok, delikler segmentlerdeki gibi dik basılır ve parçalarla aynı payı taşır (çevrel çokgen, kenar ≈ 0,6 mm). Boru elle zorlanmadan geçmeli, 0,3 mm'den fazla boşluk kalmamalı; geçmiyorsa `HOLE_CLEAR` (LW-PLA boru 0,7 mm) artırılıp yeniden üretin.

| Delik | Nominal (mm) | Basılan (mm) |
|---|---:|---:|
| panel borusu Ø27 | 27 | 27,7 |
| uç borusu Ø16 | 16 | 16,7 |
| arka kiriş / longeron Ø8 | 8 | 8,7 |
| açı pimi Ø6 | 6 | 6,7 |
| hizalama pimi Ø3 | 3 | 3,4 |
| menteşe pimi Ø1,75 | 1,75 | 2,1 |

## Aviyonik kapağı — ısıl biçimlendirme (P10)

FDM PETG füme cam görünümü vermez (çok çizgili, buğulu); kapak 1,0 mm füme PETG levhadan basılı kalıp üstünde vakumla biçimlendirilir (RC kanopi yöntemi). Kalıp uçak kütlesine girmez.

1. Kalıp: hatch_buck_a + hatch_buck_b (PLA, 0,3 mm katman, 3 çevre, %15 dolgu) Ø3 CF pimlerle birleştirilip yapıştırılır; üst yüz 240 → 400 kum zımpara, dolgu astarı, 800 kum; kalıp ayırıcı (PVA / mum). Yanlar 2° eğimli, 8 mm kesim payı kalıba dahil.
2. Levha: 1,0 mm füme PETG 65 °C'de 2 saat kurutulur (kabarcık olmasın), alüminyum çerçeveye sıkıştırılır.
3. Isıtma: fırında / ısıtıcı altında 140–160 °C, levha 10–20 mm sarkana kadar (≈ 1–2 dk).
4. Biçimlendirme: kalıp vakum masasına (≥ 350 × 200 mm) konur, levha hızla indirilir, vakum (≥ 0,6 bar) açılır; soğuyunca (≈ 1 dk) çıkarılır.
5. Kesim: kalıp kenar çizgisinden makas / Dremel ile kesilir, kenar 400 kumla düzeltilir.
6. Çerçeve: hatch_frame_a/b (PETG, düz alt yüz tablada) cepleri ile 6 mıknatıs CA ile yapıştırılır (kutup gövde basamağındaki eşleriyle denenir); çerçeve levhanın iç yüzüne şeffaf epoksi / UV yapıştırıcı ile yapıştırılır (bölme s = 0,680).
7. Kontrol: kapak gövde basamağına oturur, dış yüz deriden en çok 1,6 mm yüksek (flush); mıknatıs eşleri 0,3 mm içinde eş eksenli (aynı HATCH_MAGNETS listesinden).

## Baskı kesim istasyonları = panel çizgileri (P13)

Segment eklerinde dış deri kenarına 0,4 × 0,3 mm pah: komşu iki parça birleşince V-oluk (panel çizgisi) oluşur. Render panel çizgileri bu istasyonlarda olmalı (JSON `print_cuts`).

- kanat y (m): 0,2313, 0,36, 0,57, 0,78, 0,99, 1,2, 1,41, 1,62, 1,83, 1,9; strake ayrımı s = 1,176
- gövde s (m): 0,148, 0,4, 0,5917, 0,7833, 0,975, 1,1667, 1,379, 1,55, 1,715, 1,845, 2,0425
- stabilize eki y = 0,26 m (ok ekseni boyunca), dikey eki h = 0,16 m
- kumanda yüzeyi ekleri y (m): Aileron 1,2825, 1,465, 1,6475; FlapOut 0,5437, 0,7225, 0,9012; Elevator 0,2825

## Alet ve test parçaları

Uçak toplamına girmez: `hatch_buck_a.stl`, `hatch_buck_b.stl`, `tolerance_coupon.stl` — 3 parça, ≈ 294 g, ≈ 8,7 h (kalıp %20 dolgu eşdeğeri).

## Basılmayan parçalar (BOM)

| Grup | Kalem | Özellik | Adet |
|---|---|---|---:|
| CF boru/çubuk | Merkez soket (G10 köprüde 4° açılı) | Ø30/27 mm × 400 mm | 2 |
| CF boru/çubuk | Panel borusu ve birleştirici (0,30 m sokete girer) | Ø27/25 mm × 1040 mm | 2 |
| CF boru/çubuk | Soket ağzında iç takviye | Ø25/22 mm × 350 mm | 2 |
| CF boru/çubuk | Uç borusu | Ø16/14 mm × 700 mm | 2 |
| CF boru/çubuk | Kademeli CF burç | Ø25/16 mm × 100 mm | 2 |
| CF boru/çubuk | Arka kiriş (yalnız panelde) | Ø8/6 mm × 1440 mm | 2 |
| CF boru/çubuk | Açı pimi, dolu CF | Ø6 mm dolu × 100 mm | 2 |
| CF boru/çubuk | Gövde longeronu — chine (düz parçalar, sol + sağ) | Ø8/6 mm; parça boyları 767 + 385 + 498 mm (uçlar soket bloklarına 18 mm girer) | 6 |
| CF boru/çubuk | Gövde longeronu — omuz (düz parçalar, sol + sağ) | Ø8/6 mm; parça boyları 767 + 384 + 497 mm (uçlar soket bloklarına 18 mm girer) | 6 |
| CF boru/çubuk | Stabilize kirişi (30° oklu) | Ø12/10 mm × 560 mm | 2 |
| CF boru/çubuk | Stabilize arka çubuğu | Ø6/4 mm × 500 mm | 2 |
| CF boru/çubuk | Dikey kirişi | Ø8/6 mm × 320 mm | 2 |
| CF boru/çubuk | Menteşe pimi — Ø1,75 mm PETG filament (P1; çelik tel yerine) | delik Ø2,1 mm çevrel; boylar (mm, yüzey başına): Aileron 860, FlapIn 187, FlapOut 729, Elevator 538, Rudder 376 | 10 |
| CF boru/çubuk | Segment hizalama pimi (CF çubuk) | Ø3 mm × 18 mm | 28 |
| CF boru/çubuk | İç flap bağlayıcı teli (P9: iç flap dış flaba bağlı, ayrı servo yok) | Ø2 mm yay teli, 60 mm, iki ucu 90° kıvrık + Ø3/2,1 pirinç boru soket (flap içine epoksi) | 2 |
| G10 / kontrplak | Dihedral köprüsü (P4: kök bloğu ve kanat kutusu çerçevelerindeki 3,3 mm yuvalara epoksi) | 2× 3×40×300 mm G10 (panelin 45 mm'si kök profiline ve depo tabanına sığmaz), boru başına 2×M4 | 1 |
| G10 / kontrplak | Yangın perdesi (s = 2,045) | G10 4 mm, gövde konturu; NACA kanal açıklığı | 1 |
| G10 / kontrplak | Ön gövde / kuyruk modülü flanşı (s = 1,55) | G10 2 mm, 4×M4 | 1 |
| G10 / kontrplak | ER-150 takım montaj plakası (ana yatağa 2 × M3, burun yatağına 4 × M3) | G10 3 mm | 3 |
| G10 / kontrplak | Kumanda hornu | G10 1,6 mm, yarığa yapıştırma | 8 |
| G10 / kontrplak | Kuyruk eyeri (stabilize boruları V birleşimi) | G10 2 mm + epoksi, koni içinde | 1 |
| Dolgu | Kanat üstü fileto (basılmaz: sıfıra inen kama) | epoksi + mikrobalon, kanat kökü–chine arası, ≈ 2 × 17 cm³; şablon: U_Fairing_Fillet_L/R | 2 |
| Dolgu | Stabilize kök filetosu (basılmaz: ≤ 3 mm kama) | epoksi + mikrobalon (ısı bölgesi; LW-PLA yok), ≈ 2 × 4 cm³ | 2 |
| Dolgu | Dikey kök mermisi (basılmaz: dikey + stabilizeye teğet, yandan kırpmalı tarif yok) | epoksi + mikrobalon ya da 0,5 mm ısıl biçimlendirilmiş levha; dışarıda ≈ 2 × 10 cm³; şablon: U_Fairing_FinRoot_L/R | 2 |
| Dolgu | Kaplama kenarları (kök kaportası, ER-150 kabartması, taret yakası: ≥ 0,8 mm'de kırpılmış) | epoksi + mikrobalon ile deriye sıfırlanır | 3 |
| Dolgu | Dümen servo kaportası (dikeyin iç yüzü; segment planında değil) | 30 × 10 × 5 mm kabarcık, ≈ 0,4 g: PETG (şablon U_Fairing_Servo_Rudder_L/R ağı, taban deriye zımparalanır) ya da 0,5 mm ısıl biçimlendirilmiş levha; dikey rengiyle boyanır, arka ağız açık | 2 |
| Bağlantı | Egzoz çıkış borusu geçiş halkası (sol yanak) | Isıya dayanıklı geçiş halkası (silikon/seramik keçe), iç Ø12,4 / dış Ø24,4 mm | 1 |
| Bağlantı | M4 cıvata + kelebek somun (burun modülü) | A2 paslanmaz | 4 |
| Bağlantı | M4 cıvata + somun (G10 köprü–soket–çerçeve, s = 1,55 flanşı) | 12.9; köprüde boru başına 2 adet | 12 |
| Bağlantı | Motor bağlantısı: M4 × 30 + titreşim takozu (kauçuk, Ø10 × 8, 40 Shore A) | motor halkasındaki 4 × M4 ısıl gömme dişliye, kare 60 mm | 4 |
| Bağlantı | M4 naylon cıvata (dış panel tutma dili; P9) | M4 × 16, alttan, kök bloğu 2'deki ısıl gömme dişliye | 2 |
| Bağlantı | Isıl gömme dişli M4 | Ø5,6 delik × 8,5: motor halkası 4 + tutma dili 2 | 6 |
| Bağlantı | Isıl gömme dişli M3 + M3 × 8 cıvata | Ø4 delik: ana yatak 2 × 2, burun yatağı 4 (kısa), kaporta/yanak 6 | 14 |
| Bağlantı | Hizalama pimi (burun modülü) | çelik Ø4×20 mm | 2 |
| Bağlantı | Neodim mıknatıs (aviyonik kapağı; çerçeve 6 + gövde basamağı 6, eş kutup) | Ø6×3 mm N52, cep Ø6,4×3,2 | 12 |
| Bağlantı | MPX konnektör | 6 pin: kanat paneli ↔ kök bloğu 2 (2), kuyruk modülü s = 1,55 (1); 8 pin: burun modülü (1) | 4 |
| Kapak | Füme PETG levha (aviyonik kapağı, ısıl biçimlendirme; P10) | 1 mm, ≥ 300 × 450 mm (1 yedek) | 2 |
| Servolar | 10 mm ince kanat servosu (KST X10 sınıfı) | kanatçık, dış flap (iç flap bağlı), elevatör, dümen — yan başına 4 | 8 |
| Servolar | İtme çubuğu Ø1,6 çelik + çatal (clevis) + kilit | her kumanda servosuna 1 | 8 |
| Servolar | Gaz, jikle, burun yönlendirme servosu | mini | 3 |
| Servolar | Kapak servoları + sıralayıcı + fren | mikro | 1 |
| İniş takımı | JP Hobby ER-150 15 mm ×3 (20 kg sınıfı, 'inside'; ünite 160 g, 26×102×32 mm, 12 kg·cm, 90°, 8,4 V'ta 5 s) |  | 1 |
| İniş takımı | Ana tekerlek | Ø82,5 mm, frenli | 2 |
| İniş takımı | Burun tekerleği | Ø70 mm | 1 |
| İtki | DLE Engines DLE-20 (20 cc, tek silindir, 2 zamanlı benzinli, yan egzoz, arka pompalı karbüratör) | susturucu + CDI dahil | 1 |
| İtki | Xoar PJA-P 16x8 itici (ters hatveli), kayın |  | 1 |
| İtki | Spinner | Al Ø64 mm | 1 |
| İtki | Yakıt deposu | 2 × 0,7 L | 2 |
| İtki | Isı kalkanı | 0,5 mm Al kalkan, seramik keçe, 15 mm hava boşluğu; 150 mm içinde LW-PLA yok | 1 |
| Faydalı yük / aviyonik | SIYI ZT6 sınıfı (4K optik + 640×512 termal, 3 eksen) |  | 1 |
| Faydalı yük / aviyonik | Ana akü 4S2P Molicel P45B | 130 Wh | 1 |
| Faydalı yük / aviyonik | Uçuş kontrolcüsü, 2× GNSS, telemetri, RC alıcı, LED'ler |  | 1 |

## Montaj sırası

0. **Tolerans kuponu**: `tolerance_coupon.stl` ilk basılır; CF borular, pimler ve Ø1,75 PETG menteşe pimi deliklerde
   denenir (bkz. Tolerans kuponu). Gerekirse `HOLE_CLEAR` ayarlanıp STL'ler yeniden üretilir.
1. **Kanat dış paneli** (her yan): segmentleri kökten uca dizin; her ekte alttaki segmentin kaburgası üstteki
   segmentin 2,4 mm flanşına oturur. Ø3 CF hizalama pimlerini hücum kenarı göbeklerine yapıştırın, 27/25 panel
   borusunu ve 8/6 arka kirişi kılavuz kovanlarından geçirerek kuru montajla hizayı kontrol edin; sonra ekleri
   ince CA / 5 dk epoksi ile sırayla yapıştırın. Uç borusu (16/14) ve kademeli burç 1,00–1,10'da panel borusuna
   girer. Servo kablolarını Ø8 kablo kanalından (ana kirişin arkasında; kaburga ve kafes delikleri hizalı) panel 1
   kök kaburgasındaki MPX 6 pin cebine çekin. PETG tutma dilini panel 1 kök kaburgasına (açı piminin arkası) yapıştırın.
2. **Dış flap, uç kapağı, kanatçık** (menteşe pimleri, P1):
   a. Dış flap segmentlerini çift kaburga yüzlerinden (pim delikleri hizalı) yapıştırın; flabı oyuğa yerleştirip
      Ø1,75 PETG pimi **iç uçtan**, panel 1'in y = 0,36 kök kaburgasındaki delikten sürün (dış uçta kör delikte durur).
   b. Uç kapağını seyrüsefer LED'i ve kablosuyla eğik uç düzlemine yapıştırın.
   c. Kanatçığı oyuğa yerleştirip pimi **dış uçtan**, uç kapağının dış yüzündeki Ø2,1 delikten sürün (iç uçta kör
      delikte durur); dış deliği CA + mikrobalonla tıkayın (sökmek için tıkaç delinir).
   G10 hornları yarıklarına epoksiyle yapıştırın; servoları yuvalarına takıp itme çubuğu + çatalla bağlayın.
3. **Kök blokları, iç flap, ana takım**: kök bloğu 1 ile glove strake'i pimlerle birleştirin, kök bloğu 2'yi
   flanşından yapıştırın; Ø30 soketi, iç takviyeyi ve açı pimi kovanını epoksiyle sabitleyin. İç flap: Ø3/2,1 pirinç
   bağlayıcı soketini flap içine epoksileyin; iç flabı oyuğa yerleştirip pimi **dış uçtan**, kök bloğu 2'nin
   y = 0,36 kaburgasındaki delikten sürün (panel sökülüyken). İç flabın servosu yoktur: Ø2 bağlayıcı tel panel
   takılırken dış flabın soketine girer (P9). Ana takım yatağını (gear_mount_L/R) kök bloğu 2'nin açık iç ucundan
   kaydırıp sokete eyerleyin; ER-150 ünitesini G10 plakasıyla yatağa 2 × M3 (ısıl gömme dişli) bağlayın.
4. **Gövde ve kanat kutusu** (P4): halkaları çerçeve (tabla tarafı) → flanş (üst) sırasıyla dizin. Longeronlar 3'er
   düz CF parçadır: chine ve omuz kovanlarından geçirip s = 1,167 ve 1,55 kırıklarında PETG soket bloklarına
   (longeron_joint_*) epoksiyle birleştirin. Kanat kutusu çerçevelerini (wing_frame_fwd/aft, PA-CF) halka 5'teki
   yuvalarına yerleştirin; G10 köprü plakalarını (2 × 3 × 40 × 300 mm) çerçeve ve kök bloğu yuvalarına epoksiyle
   oturtun, boru başına 2 × M4 ile plaka–soket–plaka sıkın. s = 1,55'e G10 flanşı yapıştırın. Burun takım yatağını
   (gear_mount_N) halka 1'in ön ucundan kaydırıp iki chine longeronuna eyerleyin; burun ünitesi 4 × M3.
5. **Kaplamalar**: kök kaportasını (kano) ve ER-150 kabartmasını yapıştırın; ≥ 0,8 mm'de kırpılmış kenarları ve
   kanat üstü filetoyu epoksi + mikrobalonla deriye sıfırlayın.
6. **Burun modülü**: burun konisi + modül halkasını yapıştırın; PETG flanşı modül halkasının flanşına yapıştırın;
   taret yakasını takın. Modül 2 × Ø4 pim + 4 × M4 kelebek somunla halka 1'e bağlanır (sökülebilir; MPX 8 pin).
7. **Kuyruk**: stabilize yarılarını 12/10 kiriş ve 6/4 arka çubukla koni içindeki G10 eyerde birleştirin; kök
   filetosunu epoksi + mikrobalonla doldurun. **Dikeyler takılmadan önce** elevatörü oyuğa yerleştirip pimi
   **dış uçtan**, stabilize ucundaki delikten sürün (dikey takılınca elevatör pimi sökülemez). Dikeyleri stabilize
   ucuna ve 8/6 dikey kirişine geçirin. Dümeni oyuğa yerleştirip pimi **üst uçtan**, dikey 2 tepesindeki Ø2,1
   delikten sürün; üst deliği CA ile tıkayın. Kuyruk servo kabloları Ø6 kanaldan (stabilize ağları → kök duvarı →
   koni) s = 1,55 MPX 6 pine.
8. **Motor bölümü**: motor halkasını (engine_ring) halka 9 içine 30 mm bindirerek yapıştırın (4 longeron ucu
   soketlere girer); G10 yangın perdesini halkaya yaslayın. DLE-20'yi 4 × M4 + kauçuk takozla halkadaki ısıl gömme
   dişlilere bağlayın, ısı kalkanını takın; PA-CF kaportayı 6 × M3 ile, yanakları flanşlarından vidalayın; lüle
   halkasını kaporta önüne, NACA dudak çerçevesini halka 9'daki cebe, PA-CF sürtünme pabucunu kaporta altına
   yüksek sıcaklık epoksisiyle yapıştırın (aşınınca sökülüp yenilenir).
9. **Kapaklar**: takım kapaklarını (PETG; menteşe dilleri ve horn baskıda dahil) menteşelerine takın, burun tapa
   kapağını (door_N_3) bacak dirseğine bağlayın. Aviyonik kapağını (ısıl biçimlendirilmiş levha + PETG çerçeve)
   mıknatıslarla oturtun.
10. **Saha montajı**: dış paneli boruya geçirip MPX'i takın, tutma dilini alttan M4 naylon cıvatayla kök bloğu 2'ye
   bağlayın. Ağırlık merkezini s = 1,232 m'de (aralık 1,219–1,245) doğrulayın.

## Yazıcı ayarları

Çizgi genişlikleri modeldeki etlerle (spec `print.walls_mm`) uyumludur: her et, tablodaki çizgi genişliğinin tam katıdır ya da Arachne aralığına tek çizgi olarak sığar.

| Grup | Parçalar | Malzeme | Nozul (mm) | Çizgi (mm) | Katman (mm) | Çevre | Dolgu | Arachne duvar genişliği |
|---|---|---|---:|---|---|---|---|---|
| Kanat panelleri, uç kapağı | wing_panel_*, wing_tip | LW-PLA | 0,4 | 0,6 (kafes 1 × 0,6 — yalnız panel 1–4; deri 0,6 = 1 × 0,6; D-kutu 1,2 = 2 × 0,6) | 0,20 (kafes)–0,25 | 1 (D-kutu 2) | %0 | min 0,50 / maks 0,70 |
| Kök blokları, strake, gövde halkaları | root_block_*, glove_strake, fus_* | LW-PLA | 0,6 | 0,7 (gövde derisi 0,7 = 1 çizgi + 0,8 × 5 mm stringer; kök bloğu 0,8; flanş 2,0 = 3 × 0,67) | 0,25–0,30 | 1 | %0 | min 0,55 / maks 0,95 |
| Kuyruk konisi ve ısı bölgesi | fus_ring_7–9, stab_1, elevator_1 (spec print.zones.stab_root_heat.parts) | LW-ASA | 0,4 | 0,5 (1,0 = 2 × 0,5; stabilize/elevatör derisi 0,5 = 1 çizgi) | 0,20–0,25 | 2 | %0 | min 0,40 / maks 0,60 |
| Kuyruk yüzeyleri, kumanda yüzeyleri | stab_*, fin_*, aileron_*, flap*, elevator_*, rudder_* | LW-PLA | 0,4 | 0,5 (deri 0,5 = 1 çizgi; menteşe ağı 0,5) | 0,20 | 1 | %0 | min 0,40 / maks 0,60 |
| PA-CF kaplama ve yük yolu parçaları | cowl_* (panjur dudakları sağ yanakta), exhaust_ring, scuff_pad, intake, root_fairing, gear_blister, wing_frame_*, gear_mount_*, engine_ring | PA-CF | 0,4 | 0,53 (1,6 = 3 × 0,53; kök kaportası 1,2 = 2 × 0,6; 3 mm çerçeve 6 çizgi) | 0,20 | 3 | %100 (≤ 3,2 mm), %40 gyroid (daha kalın; kütle modeli buna göre) | min 0,40 / maks 0,65 |
| PETG parçalar | door_*, hatch_frame_*, fus_nose_flange, turret_collar, panel_lock_tab, longeron_joint_* | PETG | 0,4 | 0,40 (1,2 = 3 × 0,4; 1,6 = 4 × 0,4) | 0,20 | 3–4 | %100 (ince), %40 gyroid (blok) | min 0,34 / maks 0,50 |
| Alet: kalıp + kupon | hatch_buck_*, tolerance_coupon | PLA / LW-PLA | 0,6 | 0,65 | 0,30 | 3 | %15 | — |

**LW-PLA** (colorFabb, köpüren): 230–245 °C, akış %55–60, 40–60 mm/s, fan %30–50, tabla 50–60 °C, geri çekme 0,5–1 mm ("geri çekmede sil" açık), seyir "çevreleri geçme", dikiş firar kenarında hizalı. İnce duvar algılama (Arachne / "Print Thin Walls") açık, boşluk dolgusu kapalı; kaburga 1,2 mm ve çerçeveler dolu basılır. Segmentler dik (açıklık / gövde ekseni Z'de) basılır: kaburga ya da çerçeve tablada, flanş üstte.
**LW-ASA**: 250–265 °C, akış %55–65, kapalı kabin, tabla 95–105 °C, fan %0–20.
**PA-CF**: sertleştirilmiş nozul, 270–290 °C, kurutulmuş filament (80 °C 6 h), kapalı kabin, fan %0–20.
**PETG**: 235–245 °C, fan %30–50, tabla 70–80 °C.
**PLA kalıp**: 0,6 mm nozul, 0,3 mm katman, 3 çevre, %15 dolgu; üst yüz zımpara + astar (ısıl biçimlendirme bölümü).

**Destek** (P6): parça tablosundaki "Destek" sütunu bağlayıcıdır — "tabla desteği": seçilen yönde 45°'den dik, altında model olmayan çıkıntı > 2 cm²; "model üstü destek": 5 mm'den yüksekten modele inen çıkıntı > 8 cm²; "yok". Ağaç (tree) destek, arayüz boşluğu 0,2 mm önerilir. Tabla teması 3 cm²'den ya da izin %3'ünden azsa STL'ye 1 mm kırılabilir baskı ayağı eklenmiştir (Not sütunu); baskıdan sonra kesilir. Kanat ve gövde segmentlerindeki çıkıntılar: menteşe dili kamaları, kuyu tavanları, kapak/kuyu açıklığı kenarları ve kapalı uç kapakları.

**Isı / boya kuralı** (P14): LW-PLA Tg ≈ 55 °C; güneşte koyu üst deri 65–75 °C'ye ısınır. Üst yüz boyasının güneş yansıtması ≥ 0,5 olmalı (açık gri ya da IR-yansıtıcı "cool" pigmentli gri). **"taktik" (koyu) livery yalnız render içindir**: gerçek uçakta koyu üst yüz kullanılacaksa üst deriler (kanat, gövde üstü, kuyruk) LW-ASA'ya (Tg 95 °C) geçirilmelidir. Isı kuralı bölgesindeki parçalar zaten LW-ASA / PA-CF.

**Montaj payları** (P7): boru deliği Ø + 0,7 mm (LW-PLA/LW-ASA; PETG/PA-CF + 0,6 mm), pim deliği Ø + 0,4 mm, M4 Ø4,5, M3 Ø3,4, ısıl gömme dişli M3 Ø4 / M4 Ø5,6 mm; bütün delikler çevrel çokgen (iç yarıçap = delik yarıçapı, kenar ≈ 0,6 mm). Kovan eti 0,8 mm, kesme ağı 0,8 mm, geodezik kafes 0,6 mm (±40°, 90 mm aralık, yalnız iç dört panelde; dik baskıda yatayla 50°), hizalama pimi Ø3 mm, menteşe pimi Ø1,75 mm, çerçeve 1,6 × 6 mm, flanş 2,0 × 6 mm.

## Kapsam

Sivil gözetleme/araştırma platformu. Faydalı yük yalnız EO/IR taret; silah, mühimmat, dış yük askısı ya da bırakma mekanizması yoktur ve eklenmez.
