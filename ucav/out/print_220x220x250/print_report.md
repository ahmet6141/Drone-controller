# YELKOVAN YK-38 — 3B baskı raporu

Üretim: `ucav/blender/printprep.py` · tarih 2026-10-04 · tabla 220×220×250 mm (kenar payı 6 mm) · boolean: segment kesimi EXACT, kabuk/özellikler MANIFOLD (392 işlem).

## Özet

| | Bu model | Spec tahmini |
|---|---:|---:|
| Benzersiz STL | 57 | — |
| Basılacak parça (ayna dahil) | 92 | 125 |
| Basılı kütle | 3,63 kg | 3,33 kg |
| Baskı süresi (kaba) | 268 h | 316 h |
| STL toplamı | yazılmadı (--no-stl) | ≤ 40 MB hedef |
| Manifold / kapalı | evet, hepsi | |
| Tablaya sığma | hepsi (4 benzersiz parça dar payla, ≥ 2 mm) | |

Kütle, filament etkin yoğunluğuyla (LW-PLA köpürmüş 0,65 g/cm³) parça hacminden hesaplanır; boya, yapıştırıcı ve basılmayan parçalar dahil değildir. Süre: hacimsel hız + katman başına 2.5 s + tabla başına 6 dk (kaba tahmin).

| Malzeme | Parça | Hacim (cm³) | Kütle (g) | Süre (h) |
|---|---:|---:|---:|---:|
| LW-ASA | 3 | 361 | 270 | 17,9 |
| LW-PLA | 71 | 4 171 | 2 711 | 209,8 |
| PA-CF | 7 | 328 | 394 | 25,7 |
| PETG | 8 | 120 | 152 | 9,0 |
| PETG-füme | 2 | 76 | 96 | 5,2 |
| TPU | 1 | 3 | 3 | 0,7 |

## Parça tablosu

Ölçüler baskı yönündedir (X × Y tabla izi, Z yükseklik). **Yön**: tabladan yukarı bakan uçak ekseni; **Z açısı**: tablada izin döndürülmesi (köşegen yerleşim). Ayna (L/R) parçalar tek STL olarak verilir: sağ eş için dilimleyicide X ya da Y'de aynalayın.

### Kanat

| STL | Parça | Malz. | Et (mm) | Adet | Ölçü (mm) | Yön / Z açısı | Pay (mm) | Kütle (g) | Süre (h) | Destek | Not |
|---|---|---|---:|---:|---|---|---:|---:|---:|---|---|
| `wing_panel_1.stl` | Kanat dış paneli 1/7 (y 0.36–0.57) | LW-PLA | 0,6 | 2 | 215×213×210 | +y (sol/iskele) / 49°; tablada: y=0.360 | 2 | 92 | 6,5 | kısa köprü (iç) | sağ = ayna; dar pay |
| `wing_panel_2.stl` | Kanat dış paneli 2/7 (y 0.57–0.78) | LW-PLA | 0,6 | 2 | 165×167×210 | +y (sol/iskele) / 136°; tablada: y=0.570 | 26 | 92 | 6,5 | kısa köprü (iç) | sağ = ayna |
| `wing_panel_3.stl` | Kanat dış paneli 3/7 (y 0.78–0.99) | LW-PLA | 0,6 | 2 | 166×166×210 | +y (sol/iskele) / 46°; tablada: y=0.780 | 27 | 87 | 6,2 | yok | sağ = ayna |
| `wing_panel_4.stl` | Kanat dış paneli 4/7 (y 0.99–1.20) | LW-PLA | 0,6 | 2 | 202×204×210 | +y (sol/iskele) / 47°; tablada: y=0.990 | 8 | 83 | 5,9 | kısa köprü (iç) | sağ = ayna |
| `wing_panel_5.stl` | Kanat dış paneli 5/7 (y 1.20–1.41) | LW-PLA | 0,6 | 2 | 141×142×210 | +y (sol/iskele) / 45°; tablada: y=1.200 | 39 | 72 | 5,2 | kısa köprü (iç) | sağ = ayna |
| `wing_panel_6.stl` | Kanat dış paneli 6/7 (y 1.41–1.62) | LW-PLA | 0,6 | 2 | 125×123×210 | +y (sol/iskele) / 44°; tablada: y=1.410 | 48 | 58 | 4,3 | kısa köprü (iç) | sağ = ayna |
| `wing_panel_7.stl` | Kanat dış paneli 7/7 (y 1.62–1.83) | LW-PLA | 0,6 | 2 | 122×124×215 | +y (sol/iskele) / 41°; tablada: y=1.620 | 48 | 45 | 3,5 | kısa köprü (iç) | sağ = ayna |
| `wing_tip.stl` | Kanat uç kapağı (eğik uç, seyrüsefer LED yuvası) | LW-PLA | 0,6 | 2 | 114×116×72 | +y (sol/iskele) (6° eğik) / 49°; tablada: uç düzlemi | 52 | 14 | 1,2 | kısa köprü (iç) | sağ = ayna |
| `root_block_2.stl` | Kök bloğu 2 (y 0.231–0.36) | LW-PLA | 0,8 | 2 | 215×212×129 | −y (sağ/sancak) / 134°; tablada: y=0.360 (sökülebilir) | 3 | 62 | 4,3 | kısa köprü (iç) | sağ = ayna; dar pay |
| `root_block_1.stl` | Kök bloğu 1 (gövde yanı–y 0.231) | LW-PLA | 0,8 | 2 | 214×213×189 | −y (sağ/sancak) / 40°; tablada: y=0.231 | 3 | 73 | 5,2 | destek önerilir | sağ = ayna; gövdeye oturan kök yüzü gövde konturunu izler (0,3 mm boşluk); dar pay |
| `glove_strake.stl` | Glove strake (36° kök hücum kenarı) | LW-PLA | 0,8 | 2 | 124×124×123 | +x (burun) / 38°; tablada: strake ayrımı | 48 | 26 | 2,0 | destek önerilir | sağ = ayna |

### Kuyruk

| STL | Parça | Malz. | Et (mm) | Adet | Ölçü (mm) | Yön / Z açısı | Pay (mm) | Kütle (g) | Süre (h) | Destek | Not |
|---|---|---|---:|---:|---|---|---:|---:|---:|---|---|
| `stab_1a.stl` | Stabilize yarısı 1/2 (%25 ok eksenine dik ek) — parça A (tablaya sığması için bölündü) | LW-PLA | 0,5 | 2 | 69×69×125 | −y (sağ/sancak) (29° eğik) / 17°; tablada: stabilize eki | 76 | 18 | 1,6 | kısa köprü (iç) | sağ = ayna; kök yüzü kuyruk konisi konturunu izler; borular koni içindeki eyerde birleşir |
| `stab_1b.stl` | Stabilize yarısı 1/2 (%25 ok eksenine dik ek) — parça B (tablaya sığması için bölündü) | LW-PLA | 0,5 | 2 | 108×108×137 | −y (sağ/sancak) (29° eğik) / 17°; tablada: stabilize eki | 56 | 15 | 1,4 | destek önerilir | sağ = ayna; kök yüzü kuyruk konisi konturunu izler; borular koni içindeki eyerde birleşir |
| `stab_2a.stl` | Stabilize yarısı 2/2 (%25 ok eksenine dik ek) — parça A (tablaya sığması için bölündü) | LW-PLA | 0,5 | 2 | 64×63×172 | +y (sol/iskele) (29° eğik) / 106°; tablada: stabilize eki | 78 | 19 | 1,7 | kısa köprü (iç) | sağ = ayna; uç dikey içine 5 mm gömülür |
| `stab_2b.stl` | Stabilize yarısı 2/2 (%25 ok eksenine dik ek) — parça B (tablaya sığması için bölündü) | LW-PLA | 0,5 | 2 | 77×78×172 | +y (sol/iskele) (29° eğik) / 16°; tablada: stabilize eki | 71 | 12 | 1,3 | kısa köprü (iç) | sağ = ayna; uç dikey içine 5 mm gömülür |
| `fin_1.stl` | Dikey 1/2 (%25 ok eksenine dik ek) | LW-PLA | 0,5 | 2 | 153×153×219 | −z (aşağı) (29° eğik) / 3°; tablada: dikey eki | 33 | 55 | 4,2 | destek önerilir | sağ = ayna; iç yüzde stabilize ucu yuvası (0,3 mm boşluk) |
| `fin_2.stl` | Dikey 2/2 (%25 ok eksenine dik ek) | LW-PLA | 0,5 | 2 | 99×99×226 | +z (yukarı) (29° eğik) / 42°; tablada: dikey eki | 60 | 30 | 2,5 | kısa köprü (iç) | sağ = ayna |

### Kumanda yüzeyleri

| STL | Parça | Malz. | Et (mm) | Adet | Ölçü (mm) | Yön / Z açısı | Pay (mm) | Kütle (g) | Süre (h) | Destek | Not |
|---|---|---|---:|---:|---|---|---:|---:|---:|---|---|
| `aileron_1.stl` | Kanatçık 1/4 | LW-PLA | 0,5 | 2 | 64×64×182 | +y (sol/iskele) (6° eğik) / 57° | 78 | 14 | 1,4 | kısa köprü (iç) | sağ = ayna; menteşe ekseni Z'de; Ø1,5 çelik pim, basılı dil çentikleri |
| `aileron_2.stl` | Kanatçık 2/4 | LW-PLA | 0,5 | 2 | 56×57×183 | +y (sol/iskele) (6° eğik) / 56°; tablada: yüzey eki | 82 | 13 | 1,4 | kısa köprü (iç) | sağ = ayna; menteşe ekseni Z'de; Ø1,5 çelik pim, basılı dil çentikleri |
| `aileron_3.stl` | Kanatçık 3/4 | LW-PLA | 0,5 | 2 | 50×50×183 | +y (sol/iskele) (6° eğik) / 55°; tablada: yüzey eki | 85 | 12 | 1,3 | yok | sağ = ayna; menteşe ekseni Z'de; Ø1,5 çelik pim, basılı dil çentikleri |
| `aileron_4.stl` | Kanatçık 4/4 | LW-PLA | 0,5 | 2 | 43×43×182 | +y (sol/iskele) (6° eğik) / 55°; tablada: yüzey eki | 88 | 11 | 1,2 | yok | sağ = ayna; menteşe ekseni Z'de; Ø1,5 çelik pim, basılı dil çentikleri |
| `flapin.stl` | İç flap | LW-PLA | 0,5 | 2 | 68×68×163 | +y (sol/iskele) (4° eğik) / 143° | 76 | 14 | 1,4 | kısa köprü (iç) | sağ = ayna; menteşe ekseni Z'de; Ø1,5 çelik pim, basılı dil çentikleri |
| `flapout_1.stl` | Dış flap 1/4 | LW-PLA | 0,5 | 2 | 69×68×174 | +y (sol/iskele) (4° eğik) / 53° | 75 | 15 | 1,5 | kısa köprü (iç) | sağ = ayna; menteşe ekseni Z'de; Ø1,5 çelik pim, basılı dil çentikleri |
| `flapout_2.stl` | Dış flap 2/4 | LW-PLA | 0,5 | 2 | 70×70×175 | +y (sol/iskele) (4° eğik) / 143°; tablada: yüzey eki | 75 | 16 | 1,5 | kısa köprü (iç) | sağ = ayna; menteşe ekseni Z'de; Ø1,5 çelik pim, basılı dil çentikleri |
| `flapout_3.stl` | Dış flap 3/4 | LW-PLA | 0,5 | 2 | 71×71×175 | +y (sol/iskele) (4° eğik) / 53°; tablada: yüzey eki | 74 | 16 | 1,5 | kısa köprü (iç) | sağ = ayna; menteşe ekseni Z'de; Ø1,5 çelik pim, basılı dil çentikleri |
| `flapout_4.stl` | Dış flap 4/4 | LW-PLA | 0,5 | 2 | 72×71×174 | +y (sol/iskele) (4° eğik) / 53°; tablada: yüzey eki | 74 | 16 | 1,5 | kısa köprü (iç) | sağ = ayna; menteşe ekseni Z'de; Ø1,5 çelik pim, basılı dil çentikleri |
| `elevator_1.stl` | Elevatör 1/2 | LW-PLA | 0,5 | 2 | 43×44×243 | +y (sol/iskele) (27° eğik) / 18° | 88 | 13 | 1,5 | kısa köprü (iç) | sağ = ayna; menteşe ekseni Z'de; Ø1,5 çelik pim, basılı dil çentikleri |
| `elevator_2a.stl` | Elevatör 2/2 — parça A (tablaya sığması için bölündü) | LW-PLA | 0,5 | 2 | 38×38×130 | +y (sol/iskele) (27° eğik) / 18°; tablada: yüzey eki | 91 | 7 | 0,8 | yok | sağ = ayna; menteşe ekseni Z'de; Ø1,5 çelik pim, basılı dil çentikleri |
| `elevator_2b.stl` | Elevatör 2/2 — parça B (tablaya sığması için bölündü) | LW-PLA | 0,5 | 2 | 34×35×130 | +y (sol/iskele) (27° eğik) / 18°; tablada: yüzey eki | 93 | 6 | 0,8 | yok | sağ = ayna; menteşe ekseni Z'de; Ø1,5 çelik pim, basılı dil çentikleri |
| `rudder_a.stl` | Dümen — parça A (tablaya sığması için bölündü) | LW-PLA | 0,5 | 2 | 62×63×162 | +z (yukarı) (24° eğik) / 43° | 79 | 12 | 1,2 | kısa köprü (iç) | sağ = ayna; menteşe ekseni Z'de; Ø1,5 çelik pim, basılı dil çentikleri |
| `rudder_b.stl` | Dümen — parça B (tablaya sığması için bölündü) | LW-PLA | 0,5 | 2 | 52×52×162 | +z (yukarı) (24° eğik) / 43° | 84 | 10 | 1,1 | yok | sağ = ayna; menteşe ekseni Z'de; Ø1,5 çelik pim, basılı dil çentikleri |

### Gövde

| STL | Parça | Malz. | Et (mm) | Adet | Ölçü (mm) | Yön / Z açısı | Pay (mm) | Kütle (g) | Süre (h) | Destek | Not |
|---|---|---|---:|---:|---|---|---:|---:|---:|---|---|
| `fus_nose_cone.stl` | Burun konisi (s 0–0.148) | LW-PLA | 0,8 | 1 | 135×128×148 | +x (burun) / 71°; tablada: s=0.148 | 43 | 23 | 1,9 | yok |  |
| `fus_nose_module_a.stl` | Burun görev modülü halkası (s 0.148–0.400, taret yuvası) — parça A (tablaya sığması için bölündü) | LW-PLA | 0,8 | 1 | 194×195×126 | +x (burun) / 11°; tablada: s=0.400 (sökülebilir) | 12 | 72 | 5,0 | kısa köprü (iç) |  |
| `fus_nose_module_b.stl` | Burun görev modülü halkası (s 0.148–0.400, taret yuvası) — parça B (tablaya sığması için bölündü) | LW-PLA | 0,8 | 1 | 174×177×126 | +x (burun) / 18°; tablada: tabla bölmesi | 21 | 44 | 3,2 | destek önerilir |  |
| `fus_ring_1.stl` | Gövde halkası 1 (s 0.400–0.592) | LW-PLA | 0,8 | 1 | 200×201×192 | −x (kuyruk) / 11°; tablada: s=0.400 | 10 | 101 | 7,0 | kısa köprü (iç) |  |
| `fus_ring_2.stl` | Gövde halkası 2 (s 0.592–0.783) | LW-PLA | 0,8 | 1 | 200×201×192 | −x (kuyruk) / 11°; tablada: s=0.592 | 10 | 87 | 6,1 | destek önerilir |  |
| `fus_ring_3.stl` | Gövde halkası 3 (s 0.783–0.975) | LW-PLA | 0,8 | 1 | 203×203×192 | −x (kuyruk) / 8°; tablada: s=0.783 | 9 | 84 | 5,9 | yok |  |
| `fus_ring_4.stl` | Gövde halkası 4 (s 0.975–1.167) | LW-PLA | 0,8 | 1 | 202×202×192 | −x (kuyruk) / 7°; tablada: s=0.975 | 9 | 86 | 6,0 | yok |  |
| `fus_ring_5.stl` | Gövde halkası 5 (s 1.167–1.379) | LW-PLA | 0,8 | 1 | 203×196×212 | −x (kuyruk) / 0°; tablada: s=1.167 | 8 | 88 | 6,2 | kısa köprü (iç) |  |
| `fus_ring_6.stl` | Gövde halkası 6 (s 1.379–1.550) | LW-PLA | 0,8 | 1 | 200×200×171 | −x (kuyruk) / 45°; tablada: s=1.379 | 10 | 70 | 5,0 | yok |  |
| `fus_ring_7.stl` | Kuyruk konisi halkası (s 1.550–1.760, LW-ASA) | LW-ASA | 1,0 | 1 | 206×206×210 | −x (kuyruk) / 45°; tablada: s=1.550 | 7 | 109 | 7,2 | kısa köprü (iç) |  |
| `fus_ring_8.stl` | Kuyruk konisi halkası (s 1.760–1.845, LW-ASA) | LW-ASA | 1,0 | 1 | 190×190×85 | −x (kuyruk) / 45°; tablada: s=1.760 | 15 | 50 | 3,3 | kısa köprü (iç) |  |
| `fus_ring_9.stl` | Kuyruk konisi halkası (s 1.845–2.058, LW-ASA) | LW-ASA | 1,0 | 1 | 200×200×212 | −x (kuyruk) / 45°; tablada: s=1.845 | 10 | 112 | 7,4 | kısa köprü (iç) |  |
| `fus_nose_flange.stl` | Burun modülü PETG bağlantı flanşı (4×M4, 2 pim) | PETG | 3,0 | 1 | 190×188×3 | −x (kuyruk) / 79° | 15 | 18 | 0,9 | kısa köprü (iç) |  |

### İtki

| STL | Parça | Malz. | Et (mm) | Adet | Ölçü (mm) | Yön / Z açısı | Pay (mm) | Kütle (g) | Süre (h) | Destek | Not |
|---|---|---|---:|---:|---|---|---:|---:|---:|---|---|
| `cowl_top.stl` | Motor kaportası (PA-CF; yanak açıklıkları flanşlı) | PA-CF | 1,6 | 1 | 170×170×128 | +x (burun) / 45°; tablada: yangın perdesi | 25 | 130 | 7,8 | destek önerilir |  |
| `cowl_cheek_L.stl` | Kaporta sol yanak (susturucu tarafı) | PA-CF | 1,6 | 1 | 102×101×80 | +z (yukarı) / 33° | 59 | 41 | 2,6 | kısa köprü (iç) |  |
| `cowl_cheek_R.stl` | Kaporta sağ yanak (panjurlu) | PA-CF | 1,6 | 1 | 101×102×80 | +z (yukarı) / 57° | 59 | 41 | 2,6 | kısa köprü (iç) |  |
| `exhaust_ring.stl` | Lüle halkası (PA-CF) | PA-CF | 1,6 | 1 | 121×121×17 | −x (kuyruk) / 45° | 50 | 32 | 1,9 | kısa köprü (iç) |  |
| `scuff_pad.stl` | Kaporta altı sürtünme pabucu (TPU) | TPU | 2,0 | 1 | 22×22×49 | −x (kuyruk) (9° eğik) / 45° | 99 | 3 | 0,7 | yok |  |
| `intake.stl` | Sırt hava alığı (PA-CF) | PA-CF | 1,6 | 1 | 76×76×138 | −x (kuyruk) / 45° | 72 | 48 | 3,2 | kısa köprü (iç) |  |

### Kaplamalar

| STL | Parça | Malz. | Et (mm) | Adet | Ölçü (mm) | Yön / Z açısı | Pay (mm) | Kütle (g) | Süre (h) | Destek | Not |
|---|---|---|---:|---:|---|---|---:|---:|---:|---|---|
| `root_fairing.stl` | Kök kaportası + ER-150 kabartması (PA-CF) | PA-CF | 1,6 | 2 | 211×208×224 | −x (kuyruk) / 50° | 4 | 52 | 3,7 | kısa köprü (iç) | sağ = ayna; dar pay |
| `hatch_a.stl` | Aviyonik kapağı (füme PETG, mıknatıslı) — parça A (tablaya sığması için bölündü) | PETG-füme | 1,6 | 1 | 172×172×116 | +y (sol/iskele) / 140° | 24 | 50 | 2,7 | yok |  |
| `hatch_b.stl` | Aviyonik kapağı (füme PETG, mıknatıslı) — parça B (tablaya sığması için bölündü) | PETG-füme | 1,6 | 1 | 164×163×116 | +y (sol/iskele) / 47° | 28 | 46 | 2,5 | yok |  |

### Faydalı yük

| STL | Parça | Malz. | Et (mm) | Adet | Ölçü (mm) | Yön / Z açısı | Pay (mm) | Kütle (g) | Süre (h) | Destek | Not |
|---|---|---|---:|---:|---|---|---:|---:|---:|---|---|
| `turret_collar.stl` | Taret yakası (8 faset, PETG) | PETG | 1,6 | 1 | 122×122×74 | +z (yukarı) / 23° | 49 | 80 | 3,8 | destek önerilir |  |

### Takım kapakları

| STL | Parça | Malz. | Et (mm) | Adet | Ölçü (mm) | Yön / Z açısı | Pay (mm) | Kütle (g) | Süre (h) | Destek | Not |
|---|---|---|---:|---:|---|---|---:|---:|---:|---|---|
| `door_N_1.stl` | Burun takımı kapağı (PETG 1,2 mm) | PETG | 1,2 | 2 | 188×191×25 | −y (sağ/sancak) / 45° | 14 | 9 | 0,6 | yok | sağ = ayna |
| `door_L_1.stl` | Ana takım kuyu kapağı (PETG 1,2 mm) | PETG | 1,2 | 2 | 94×89×1 | −z (aşağı) / 0° | 63 | 12 | 0,6 | yok | sağ = ayna |
| `door_L_2.stl` | Ana takım bacak kapağı (PETG 1,2 mm) | PETG | 1,2 | 2 | 46×45×162 | −z (aşağı) / 119° | 87 | 6 | 0,9 | yok | sağ = ayna |

## Doğrulama

Her parça bmesh ile denetlenir: sınır kenar = 0, manifold olmayan kenar = 0, tel kenar = 0, yönü tutarsız kenar = 0, işaretli hacim > 0. Et ölçümü: yüzeyden içe ışın (alan ağırlıklı 1200 örnek); %5'lik değer nominal et ya da en ince iç yapıdır (kanat panellerinde 0,45 mm geodezik kafes); en küçük değer köşe/pah örneklerini ve kaplamaların sıfıra incelen yapışma kenarlarını içerir.

| STL | Kenar denetimi (yeniden okuma) | Hacim farkı | Blender içe aktarma |
|---|---|---:|---|

Manifold olmayan parça: 0. 

| Parça | Et %5 (mm) | Et medyan (mm) | Çıkıntı (cm²) |
|---|---:|---:|---:|
| wing_panel_1 | 0,45 | 0,79 | 13,7 |
| wing_panel_2 | 0,45 | 0,79 | 16,9 |
| wing_panel_3 | 0,45 | 0,79 | 2,1 |
| wing_panel_4 | 0,45 | 0,79 | 20,6 |
| wing_panel_5 | 0,45 | 0,79 | 15,2 |
| wing_panel_6 | 0,45 | 0,79 | 5,7 |
| wing_panel_7 | 0,45 | 0,79 | 6,3 |
| wing_tip | 0,60 | 0,60 | 5,9 |
| root_block_2 | 0,79 | 0,80 | 13,5 |
| root_block_1 | 0,78 | 0,80 | 73,5 |
| glove_strake | 0,80 | 1,20 | 30,3 |
| stab_1a | 0,50 | 0,50 | 13,8 |
| stab_1b | 0,49 | 0,50 | 31,2 |
| stab_2a | 0,50 | 0,50 | 5,4 |
| stab_2b | 0,49 | 0,50 | 13,2 |
| fin_1 | 0,50 | 0,50 | 68,7 |
| fin_2 | 0,50 | 0,50 | 16,7 |
| aileron_1 | 0,49 | 0,50 | 4,7 |
| aileron_2 | 0,50 | 0,50 | 3,5 |
| aileron_3 | 0,50 | 0,50 | 2,2 |
| aileron_4 | 0,50 | 0,50 | 1,4 |
| flapin | 0,49 | 0,50 | 7,5 |
| flapout_1 | 0,49 | 0,50 | 7,7 |
| flapout_2 | 0,50 | 0,50 | 8,1 |
| flapout_3 | 0,50 | 0,50 | 8,4 |
| flapout_4 | 0,49 | 0,50 | 6,7 |
| elevator_1 | 0,50 | 0,50 | 6,8 |
| elevator_2a | 0,50 | 0,50 | 1,7 |
| elevator_2b | 0,50 | 0,50 | 1,5 |
| rudder_a | 0,50 | 0,50 | 12,7 |
| rudder_b | 0,50 | 0,50 | 2,3 |
| fus_nose_cone | 0,79 | 0,80 | 2,9 |
| fus_nose_module_a | 0,80 | 0,80 | 3,6 |
| fus_nose_module_b | 0,79 | 0,80 | 45,0 |
| fus_ring_1 | 0,79 | 0,80 | 8,9 |
| fus_ring_2 | 0,79 | 0,80 | 44,6 |
| fus_ring_3 | 0,79 | 0,80 | 0,6 |
| fus_ring_4 | 0,79 | 0,80 | 0,0 |
| fus_ring_5 | 0,79 | 0,80 | 7,6 |
| fus_ring_6 | 0,79 | 0,80 | 1,1 |
| fus_ring_7 | 0,79 | 1,00 | 5,7 |
| fus_ring_8 | 0,79 | 1,00 | 4,7 |
| fus_ring_9 | 0,79 | 1,00 | 6,2 |
| fus_nose_flange | 3,00 | 3,00 | 5,1 |
| cowl_top | 1,29 | 1,60 | 83,8 |
| cowl_cheek_L | 1,58 | 1,60 | 18,7 |
| cowl_cheek_R | 1,55 | 1,60 | 18,3 |
| exhaust_ring | 1,58 | 1,60 | 25,7 |
| scuff_pad | 1,73 | 2,53 | 0,2 |
| intake | 0,23 | 1,60 | 28,8 |
| root_fairing | 0,36 | 1,60 | 27,7 |
| turret_collar | 1,56 | 1,58 | 77,4 |
| hatch_a | 1,24 | 1,43 | 0,0 |
| hatch_b | 1,24 | 1,43 | 0,0 |
| door_N_1 | 1,18 | 1,20 | 0,4 |
| door_L_1 | 1,20 | 1,20 | 0,0 |
| door_L_2 | 0,81 | 1,15 | 0,0 |

### Uyarılar

- wing_tip: voksel onarım (0.15 mm), hacim 21.5 → 21.5 cm³, manifold evet

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
| CF boru/çubuk | Gövde longeronu | Ø8/6 mm × 1660 mm | 4 |
| CF boru/çubuk | Stabilize kirişi (30° oklu) | Ø12/10 mm × 560 mm | 2 |
| CF boru/çubuk | Stabilize arka çubuğu | Ø6/4 mm × 500 mm | 2 |
| CF boru/çubuk | Dikey kirişi | Ø8/6 mm × 320 mm | 2 |
| CF boru/çubuk | Menteşe pimi (çelik yay teli) | Ø1.5 mm, toplam ≈ 4.89 m (yüzey başına tek parça) | 10 |
| CF boru/çubuk | Segment hizalama pimi (CF çubuk) | Ø3 mm × 18 mm | 31 |
| G10 / kontrplak | Dihedral köprüsü (epoksi dolgulu yuvalar) | 2× 3×45×300 mm G10, boru başına 2×M4 | 1 |
| G10 / kontrplak | Yangın perdesi (s = 2,06) | G10 4 mm, gövde konturu | 1 |
| G10 / kontrplak | Ön gövde / kuyruk modülü flanşı (s = 1,55) | G10 2 mm, 4×M4 | 1 |
| G10 / kontrplak | ER-150 takım montaj plakası | G10 3 mm (kök bloğu) | 3 |
| G10 / kontrplak | Kumanda hornu | G10 1,6 mm, yarığa yapıştırma | 10 |
| G10 / kontrplak | Kuyruk eyeri (stabilize boruları V birleşimi) | G10 2 mm + epoksi, koni içinde | 1 |
| Dolgu | Kanat üstü fileto (basılmaz: sıfıra inen kama) | epoksi + mikrobalon, kanat kökü–chine arası, ≈ 2 × 17 cm³; şablon: U_Fairing_Fillet_L/R | 2 |
| Dolgu | Stabilize kök filetosu (basılmaz: 0,1–3 mm kama) | epoksi + mikrobalon (ısı bölgesi; LW-PLA yok), ≈ 2 × 4 cm³ | 2 |
| Bağlantı | M4 cıvata + kelebek somun (burun modülü) | A2 paslanmaz | 4 |
| Bağlantı | M4 cıvata + somun (G10 köprü, flanş, motor) | 12.9 | 12 |
| Bağlantı | M3 cıvata + ısıl gömme dişli (kaporta, yanaklar) | M3×8 | 14 |
| Bağlantı | Hizalama pimi (burun modülü) | çelik Ø4×20 mm | 2 |
| Bağlantı | Neodim mıknatıs (aviyonik kapağı) | Ø6×3 mm | 6 |
| Bağlantı | MPX 6 pin konnektör (kanat paneli), 8 pin (burun modülü) |  | 3 |
| Servolar | 10 mm ince kanat servosu (KST X10 sınıfı) | kanatçık, dış/iç flap, elevatör, dümen | 10 |
| Servolar | Gaz, jikle, burun yönlendirme servosu | mini | 3 |
| Servolar | Kapak servoları + sıralayıcı + fren | mikro | 1 |
| İniş takımı | JP Hobby ER-150 15 mm ×3 (20 kg sınıfı, 'inside'; ünite 160 g, 26×102×32 mm, 12 kg·cm, 90°, 8,4 V'ta 5 s) |  | 1 |
| İniş takımı | Ana tekerlek | Ø82.5 mm, frenli | 2 |
| İniş takımı | Burun tekerleği | Ø70 mm | 1 |
| İtki | DLE Engines DLE-20 (20 cc, tek silindir, 2 zamanlı benzinli, yan egzoz, arka pompalı karbüratör) | susturucu + CDI dahil | 1 |
| İtki | Xoar PJA-P 16x8 itici (ters hatveli), kayın |  | 1 |
| İtki | Spinner | Al Ø64 mm | 1 |
| İtki | Yakıt deposu | 2 × 0.7 L | 2 |
| İtki | Isı kalkanı | 0,5 mm Al kalkan, seramik keçe, 15 mm hava boşluğu; 150 mm içinde LW-PLA yok | 1 |
| Faydalı yük / aviyonik | SIYI ZT6 sınıfı (4K optik + 640×512 termal, 3 eksen) |  | 1 |
| Faydalı yük / aviyonik | Ana akü 4S2P Molicel P45B | 130 Wh | 1 |
| Faydalı yük / aviyonik | Uçuş kontrolcüsü, 2× GNSS, telemetri, RC alıcı, LED'ler |  | 1 |

## Montaj sırası

1. **Kanat dış paneli** (her yan): segmentleri kökten uca dizin; her ekte alttaki segmentin kaburgası üstteki
   segmentin 2,4 mm flanşına oturur. Ø3 CF hizalama pimlerini hücum kenarı göbeklerine yapıştırın, 27/25 panel
   borusunu ve 8/6 arka kirişi kılavuz kovanlarından geçirerek kuru montajla hizayı kontrol edin; sonra ekleri
   ince CA / 5 dk epoksi ile sırayla yapıştırın. Uç borusu (16/14) ve kademeli burç 1,00–1,10'da panel borusuna girer.
2. **Uç kapağı**: seyrüsefer LED'ini yuvasına koyup kabloyu kaburga deliklerinden geçirin; eğik uç düzlemine yapıştırın.
3. **Kumanda yüzeyleri**: segmentleri çift kaburga yüzlerinden yapıştırın (pim deliği hizalı). Yüzeyi oyuğa
   yerleştirip Ø1,5 çelik pimi dış uçtan dil ve kovanlardan geçirin; pim ucunu oyuk duvarında CA ile sabitleyin.
   G10 hornu yarığa epoksiyle yapıştırın; servo yuvasına servoyu takıp itme çubuğunu bağlayın.
4. **Kök blokları ve strake**: kök bloğu 1 ile glove strake'i pimlerle birleştirin, kök bloğu 2'yi flanşından
   yapıştırın. Ø30 soketi ve açı pimi kovanını epoksiyle sabitleyin; ER-150 ünitesini G10 plakasıyla kuyu
   üstüne bağlayın.
5. **Gövde**: halkaları çerçeve (tabla tarafı) → flanş (üst) sırasıyla dizin; 4 adet 8/6 CF longeronu chine ve omuz
   kovanlarından geçirerek hizalayın ve yapıştırın. s = 1,55'e G10 flanşı, s = 2,06'ya G10 yangın perdesini
   yapıştırın (kuyruk konisi LW-ASA). G10 dihedral köprüsünü kanat kutusu açıklığından geçirip kök bloklarına
   M4 ile bağlayın; kök kaportasını yapıştırın, kanat üstü filetoyu epoksi + mikrobalonla şekillendirin.
6. **Burun modülü**: burun konisi + modül halkasını yapıştırın; PETG flanşı modül halkasının flanşına yapıştırın;
   taret yakasını yuvasına takın. Modül 2×Ø4 pim + 4×M4 kelebek somunla halka 1'e bağlanır (sökülebilir).
7. **Kuyruk**: stabilize yarılarını 12/10 kiriş ve 6/4 arka çubukla koni içindeki G10 eyerde birleştirin; kök
   yüzleri koni konturuna 0,3 mm boşlukla oturur. Kök filetosunu epoksi + mikrobalonla doldurun. Dikeyleri stabilize ucuna
   (iç yüz yuvası) ve 8/6 dikey kirişine geçirin.
8. **Motor bölümü**: DLE-20'yi yangın perdesine bağlayın, ısı kalkanını takın; PA-CF kaportayı 6×M3 ile perdeye,
   yanakları M3 ısıl gömme dişlilerle flanşlarına vidalayın; lüle halkasını kaporta arkasına yapıştırın; TPU pabucu
   kaporta altına yapıştırın.
9. **Kapaklar**: takım kapaklarını (PETG) menteşelerine, aviyonik kapağını mıknatıslarına yerleştirin. Ağırlık
   merkezini s = 1,232 m'de (aralık 1,219–1,245) doğrulayın.

## Yazıcı ayarları

**LW-PLA tek duvar deriler** (kanat, gövde, kuyruk; colorFabb LW-PLA): nozul 0,4 mm, 230–245 °C (köpürme), akış
%55–60, çizgi genişliği 0,6 mm (köpürmüş), katman 0,25–0,30 mm, çevre sayısı 1 (0,6 mm deri tek çizgi;
1,2 mm D-kutu ve flanşlar 2 çizgi), dolgu %0, üst/alt katman 3 (kaburga 1,2 mm ve çerçeveler dolu basılır),
hız 40–60 mm/s, fan %30–50, tabla 50–60 °C, geri çekme 0,5–1 mm (LW-PLA sızar; "geri çekmede sil" açık),
"çevreleri geçme" seyir açık, dikiş firar kenarında hizalı. İnce duvar algılama (Arachne / "Print Thin Walls")
açık olmalı: 0,45 mm geodezik kafes tek ince çizgiyle basılır; boşluk dolgusu kapalı.
Segmentler dik (açıklık Z'de) basılır; kaburga tablada, 2,4 mm × 6 mm flanş üstte. 45°'den dik çıkıntı yalnız
menteşe dili kamalarında ve kapalı kuyruk uçlarında (kısa köprü) vardır; destek gerekmez.

**LW-ASA** (kuyruk konisi, 1,0 mm): 250–265 °C, akış %55–65, kapalı kabin, tabla 95–105 °C, fan %0–20.
**PA-CF** (kaporta, lüle, kök kaportası, alık; 1,6 mm): sertleştirilmiş nozul, 270–290 °C,
kurutulmuş filament (80 °C 6 h), 3 çevre, %100 dolgu (ince duvar), kapalı kabin. Kaplamaların gövdeye/kanada
yapışan kenarı sıfıra incelir (alanın %2–7'si < 0,5 mm); dilimleyici 0,4 mm altını basmaz → montajda epoksi
macunla kenar düzeltilir.
**PETG** (kapaklar, taret yakası, burun flanşı; 1,2–3 mm): 235–245 °C, %100 dolgu, katman 0,2 mm; aviyonik
kapağı füme PETG, parlak yüz yukarı.
**TPU 95A** (sürtünme pabucu): 220–230 °C, yavaş (20 mm/s), geri çekme kapalı.

Montaj payları: boru deliği Ø + 0,4 mm, kovan eti 0,8 mm, kesme ağı 0,8 mm,
geodezik kafes 0,45 mm (±45°, 70 mm aralık), hizalama pimi Ø3 mm, menteşe pimi Ø1,5 mm,
çerçeve 1,6 mm × 8 mm.

## Kapsam

Sivil gözetleme/araştırma platformu. Faydalı yük yalnız EO/IR taret; silah, mühimmat, dış yük askısı ya da bırakma mekanizması yoktur ve eklenmez.
