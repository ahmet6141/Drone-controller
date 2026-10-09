# YK-250 HANÇER — yapı hesapları: emniyet payı tablosu

Üreten: `python3 -m ucav250.analysis.structures` (el hesapları, `analysis/structlib.py`); yöntem ve açıklamalar: `docs/04_yapi_hesaplari.md`. Uygulanan değer LİMİT yük/gerilme/birim şekil değiştirme (yalnız nihai durumlarda nihai), izin verilen değer NİHAİ tasarım değeridir; **MS = izin verilen / (uygulanan × toplam katsayı) − 1**.

- Satır sayısı: 277 (258 emniyet payı, 19 gereksinim / bilgi satırı)
- En küçük MS: **0,001** (W-CAP-07); negatif MS: 0
- Arayüz kontrolleri (yerleşim ↔ yapı boyutları): 15/15 uygun

## Katsayılar

| Katsayı | Değer | Kaynak |
|---|---|---|
| Emniyet katsayısı (FoS) | 1,5 | CS-LUAS.303, STANAG 4703 UL2.3 |
| Bağlantı (fitting) katsayısı | 1,15 | CS-LUAS.625 |
| Mafsallı / dönen bağlantı ezilme katsayısı | 2 | STANAG 4703 UL2.4 |
| Sık sökülüp takılan bağlantı katsayısı | 1,5 | STANAG 4703 UL2.4 |
| Kompozit özel katsayısı (B-tabanı ETW ile) | 1,2 | AMC LUAS.619 |
| Tek yük yollu kompozit A-tabanı tahmini | 0,85 × B-tabanı | CS-LUAS.613(b) |
| Menteşe ezilmesi toplam | 6,67 | CS-LUAS.657 |
| İtme-çekme bağlantısı toplam | 3,33 | CS-LUAS.693 |
| Birleştirme kuralı | FoS × en büyük özel katsayı | CS-LUAS.619 |
| Hasar toleransı sınırları (nihai yükte) | 3000 µε bası (> 2 mm), 2600 µε bası (sandviç / ≤ 2 mm), 5000 µε çekme, 5200 µε kesme | STANAG 4703 UL13.1.2 |

## Emniyet payı tablosu

Yük durumu kodları (YD-xx) tablonun altında açıklanmıştır. Birimler: N, N m, MPa, N/mm (kesme akışı / doğrusal yük), µε; 'yük çarpanı' satırlarında uygulanan = 1 (katsayılı yük) ve izin verilen = kritik yük çarpanıdır.

### Kanat

| No | Eleman / kontrol | Yük durumu | Uygulanan | İzin verilen | Birim | Katsayılar | MS | Referans |
|---|---|---|---|---|---|---|---|---|
| DT-COND | STANAG UL13.1.2 hasar toleransı birim şekil değiştirme sınırlarının uygulanabilirliği: matrise duyarlı özelliklerin ETW kaybı RTD'nin % 50'sinin altında | YD-01 | 0,472 | 0,5 | fraction | - | gereksinim | largest: PW S_0.2; UD Xc 37 % (b_basis); PW Xc (warp) 31 % (mean); PW Yc (fill) 36 % (mean); PW S_0.2 47 % (b_basis); condition < 0,50 met -> UL13.1.2 strain limits used (fix round 2, VS2-11) |
| W-CAP-01 | ana kiriş başlığı y 0,00-0,40 m (58 kat x 40 mm, belirleyici y 0,00): kaplama yüzü bası birim şekil değiştirmesi (sandviç, hasar toleransı) | YD-02 | 1723,6 | 2600,0 | µε | FoS 1,5 | 0,006 | STANAG 4703 UL13.1.2 (2600 µε sandviç kaplamalar) |
| W-CAP-02 | ana kiriş başlığı y 0,40-0,55 m (33 kat x 40 mm, belirleyici y 0,40, eldiven kutusu momenti = toplam - dil momenti): kaplama yüzü bası birim şekil değiştirmesi (sandviç, hasar toleransı) | YD-02 | 1730,0 | 2600,0 | µε | FoS 1,5 | 0,002 | STANAG 4703 UL13.1.2 (2600 µε sandviç kaplamalar) |
| W-CAP-03 | ana kiriş başlığı y 0,55-0,70 m (12 kat x 40 mm, belirleyici y 0,55, eldiven kutusu momenti = toplam - dil momenti): kaplama yüzü bası birim şekil değiştirmesi (sandviç, hasar toleransı) | YD-02 | 1706,5 | 2600,0 | µε | FoS 1,5 | 0,016 | STANAG 4703 UL13.1.2 (2600 µε sandviç kaplamalar) |
| W-CAP-04 | ana kiriş başlığı y 0,70-0,80 m (36 kat x 30 mm, belirleyici y 0,70): kaplama yüzü bası birim şekil değiştirmesi (sandviç, hasar toleransı) | YD-02 | 1715,0 | 2600,0 | µε | FoS 1,5 | 0,011 | STANAG 4703 UL13.1.2 (2600 µε sandviç kaplamalar) |
| W-CAP-05 | ana kiriş başlığı y 0,80-0,90 m (35 kat x 30 mm, belirleyici y 0,80): kaplama yüzü bası birim şekil değiştirmesi (sandviç, hasar toleransı) | YD-02 | 1703,8 | 2600,0 | µε | FoS 1,5 | 0,017 | STANAG 4703 UL13.1.2 (2600 µε sandviç kaplamalar) |
| W-CAP-06 | ana kiriş başlığı y 0,90-1,00 m (33 kat x 30 mm, belirleyici y 0,90): kaplama yüzü bası birim şekil değiştirmesi (sandviç, hasar toleransı) | YD-02 | 1699,8 | 2600,0 | µε | FoS 1,5 | 0,020 | STANAG 4703 UL13.1.2 (2600 µε sandviç kaplamalar) |
| W-CAP-07 | ana kiriş başlığı y 1,00-1,10 m (30 kat x 30 mm, belirleyici y 1,00): kaplama yüzü bası birim şekil değiştirmesi (sandviç, hasar toleransı) | YD-02 | 1730,9 | 2600,0 | µε | FoS 1,5 | 0,001 | STANAG 4703 UL13.1.2 (2600 µε sandviç kaplamalar) |
| W-CAP-08 | ana kiriş başlığı y 1,10-1,20 m (28 kat x 30 mm, belirleyici y 1,10): kaplama yüzü bası birim şekil değiştirmesi (sandviç, hasar toleransı) | YD-02 | 1703,6 | 2600,0 | µε | FoS 1,5 | 0,017 | STANAG 4703 UL13.1.2 (2600 µε sandviç kaplamalar) |
| W-CAP-09 | ana kiriş başlığı y 1,20-1,30 m (25 kat x 30 mm, belirleyici y 1,20): kaplama yüzü bası birim şekil değiştirmesi (sandviç, hasar toleransı) | YD-02 | 1715,8 | 2600,0 | µε | FoS 1,5 | 0,010 | STANAG 4703 UL13.1.2 (2600 µε sandviç kaplamalar) |
| W-CAP-10 | ana kiriş başlığı y 1,30-1,40 m (23 kat x 30 mm, belirleyici y 1,30): kaplama yüzü bası birim şekil değiştirmesi (sandviç, hasar toleransı) | YD-02 | 1707,6 | 2600,0 | µε | FoS 1,5 | 0,015 | STANAG 4703 UL13.1.2 (2600 µε sandviç kaplamalar) |
| W-CAP-11 | ana kiriş başlığı y 1,40-1,50 m (21 kat x 30 mm, belirleyici y 1,40): kaplama yüzü bası birim şekil değiştirmesi (sandviç, hasar toleransı) | YD-02 | 1728,1 | 2600,0 | µε | FoS 1,5 | 0,003 | STANAG 4703 UL13.1.2 (2600 µε sandviç kaplamalar) |
| W-CAP-12 | ana kiriş başlığı y 1,50-1,60 m (20 kat x 30 mm, belirleyici y 1,50): kaplama yüzü bası birim şekil değiştirmesi (sandviç, hasar toleransı) | YD-02 | 1684,2 | 2600,0 | µε | FoS 1,5 | 0,029 | STANAG 4703 UL13.1.2 (2600 µε sandviç kaplamalar) |
| W-CAP-13 | ana kiriş başlığı y 1,60-1,70 m (17 kat x 30 mm, belirleyici y 1,60): kaplama yüzü bası birim şekil değiştirmesi (sandviç, hasar toleransı) | YD-02 | 1728,5 | 2600,0 | µε | FoS 1,5 | 0,003 | STANAG 4703 UL13.1.2 (2600 µε sandviç kaplamalar) |
| W-CAP-14 | ana kiriş başlığı y 1,70-1,80 m (15 kat x 30 mm, belirleyici y 1,70): kaplama yüzü bası birim şekil değiştirmesi (sandviç, hasar toleransı) | YD-02 | 1710,2 | 2600,0 | µε | FoS 1,5 | 0,014 | STANAG 4703 UL13.1.2 (2600 µε sandviç kaplamalar) |
| W-CAP-15 | ana kiriş başlığı y 1,80-1,90 m (13 kat x 30 mm, belirleyici y 1,80): kaplama yüzü bası birim şekil değiştirmesi (sandviç, hasar toleransı) | YD-02 | 1693,3 | 2600,0 | µε | FoS 1,5 | 0,024 | STANAG 4703 UL13.1.2 (2600 µε sandviç kaplamalar) |
| W-CAP-16 | ana kiriş başlığı y 1,90-2,00 m (12 kat x 30 mm, belirleyici y 1,90): kaplama yüzü bası birim şekil değiştirmesi (sandviç, hasar toleransı) | YD-02 | 1654,4 | 2600,0 | µε | FoS 1,5 | 0,048 | STANAG 4703 UL13.1.2 (2600 µε sandviç kaplamalar) |
| W-CAP-17 | ana kiriş başlığı y 2,00-2,10 m (10 kat x 30 mm, belirleyici y 2,00): kaplama yüzü bası birim şekil değiştirmesi (sandviç, hasar toleransı) | YD-02 | 1701,9 | 2600,0 | µε | FoS 1,5 | 0,018 | STANAG 4703 UL13.1.2 (2600 µε sandviç kaplamalar) |
| W-CAP-18 | ana kiriş başlığı y 2,10-2,20 m (10 kat x 30 mm, belirleyici y 2,10): kaplama yüzü bası birim şekil değiştirmesi (sandviç, hasar toleransı) | YD-02 | 1543,3 | 2600,0 | µε | FoS 1,5 | 0,123 | STANAG 4703 UL13.1.2 (2600 µε sandviç kaplamalar) |
| W-CAP-19 | ana kiriş başlığı y 2,20-3,60 m (10 kat x 30 mm, belirleyici y 2,20): kaplama yüzü bası birim şekil değiştirmesi (sandviç, hasar toleransı) | YD-02 | 1371,2 | 2600,0 | µε | FoS 1,5 | 0,264 | STANAG 4703 UL13.1.2 (2600 µε sandviç kaplamalar) |
| W-ROLL | yuvarlanma durumu: daha yüklü tarafın kök eğilme momenti (n_A'da %100) / tasarım durumu | YD-03 | 3724,3 | 5351,8 | N m | - | gereksinim | karşılanıyor: tasarım durumu kök momenti 5352 N m (n 5,46); asimetri yuvarlanma ataletiyle dengelenir (orta kutunun ortasındaki moment iki tarafın ortalamasıdır) |
| W-SEC-CT-1 | kanat kesiti y 0,00 m: başlık bası birim şekil değiştirmesi (hasar toleransı, > 2 mm) | YD-02 | 1501,0 | 3000,0 | µε | FoS 1,5 | 0,332 | STANAG 4703 UL13.1.2 (3000 µε nihai yükte) |
| W-SEC-CT-2 | kanat kesiti y 0,00 m: başlık bası gerilmesi (A-tabanı tahmini 0,85 Fcu) | YD-02 | 191,6 | 656,0 | MPa | FoS 1,5 x kompozit 1,2 | 0,902 | CS-LUAS.613(b), AMC LUAS.619; malzeme Fcu B-tabanı ETW |
| W-SEC-CT-3 | kanat kesiti y 0,00 m: başlık çekme birim şekil değiştirmesi (hasar toleransı) | YD-02 | 1501,0 | 5000,0 | µε | FoS 1,5 | 1,221 | STANAG 4703 UL13.1.2 |
| W-SEC-CT-4 | kanat kesiti y 0,00 m: başlık çekme gerilmesi (A-tabanı tahmini 0,85 Ftu) | YD-02 | 191,6 | 1323,2 | MPa | FoS 1,5 x kompozit 1,2 | 2,837 | CS-LUAS.613(b) |
| W-SEC-CT-5 | kanat kesiti y 0,00 m: kaplama yüzü bası birim şekil değiştirmesi (sandviç, hasar toleransı) | YD-02 | 1723,7 | 2600,0 | µε | FoS 1,5 | 0,006 | STANAG 4703 UL13.1.2 (2600 µε sandviç kaplamalar) |
| W-SEC-CT-6 | kanat kesiti y 0,00 m: kaplama dış yüzü delikli bası (OHC) gerilmesi | YD-02 | 76,05 | 175,4 | MPa | FoS 1,5 x kompozit 1,2 | 0,281 | materials.yaml design_values_for_code OHC (PW-QI ETW2 B-tabanı) |
| W-SEC-CT-7 | kanat kesiti y 0,00 m: kaplama dış yüzü buruşma | YD-02 | 76,05 | 151,5 | MPa | FoS 1,5 x kompozit 1,2 | 0,106 | Zenkert 1995: 0,5 (Ef Ec Gc)^(1/3), ROHACELL 51 WF minimum değerler |
| W-SEC-GLOVE-1 | kanat kesiti y 0,41 m: başlık bası birim şekil değiştirmesi (hasar toleransı, > 2 mm) | YD-02 | 1576,1 | 3000,0 | µε | FoS 1,5 | 0,269 | STANAG 4703 UL13.1.2 (3000 µε nihai yükte) |
| W-SEC-GLOVE-2 | kanat kesiti y 0,41 m: başlık bası gerilmesi (A-tabanı tahmini 0,85 Fcu) | YD-02 | 201,1 | 656,0 | MPa | FoS 1,5 x kompozit 1,2 | 0,812 | CS-LUAS.613(b), AMC LUAS.619; malzeme Fcu B-tabanı ETW |
| W-SEC-GLOVE-3 | kanat kesiti y 0,41 m: başlık çekme birim şekil değiştirmesi (hasar toleransı) | YD-02 | 1574,9 | 5000,0 | µε | FoS 1,5 | 1,117 | STANAG 4703 UL13.1.2 |
| W-SEC-GLOVE-4 | kanat kesiti y 0,41 m: başlık çekme gerilmesi (A-tabanı tahmini 0,85 Ftu) | YD-02 | 201,0 | 1323,2 | MPa | FoS 1,5 x kompozit 1,2 | 2,658 | CS-LUAS.613(b) |
| W-SEC-GLOVE-5 | kanat kesiti y 0,41 m: kaplama yüzü bası birim şekil değiştirmesi (sandviç, hasar toleransı) | YD-02 | 1696,9 | 2600,0 | µε | FoS 1,5 | 0,021 | STANAG 4703 UL13.1.2 (2600 µε sandviç kaplamalar) |
| W-SEC-GLOVE-6 | kanat kesiti y 0,41 m: kaplama dış yüzü delikli bası (OHC) gerilmesi | YD-02 | 59,09 | 175,4 | MPa | FoS 1,5 x kompozit 1,2 | 0,649 | materials.yaml design_values_for_code OHC (PW-QI ETW2 B-tabanı) |
| W-SEC-GLOVE-7 | kanat kesiti y 0,41 m: kaplama dış yüzü buruşma | YD-02 | 59,09 | 140,0 | MPa | FoS 1,5 x kompozit 1,2 | 0,316 | Zenkert 1995: 0,5 (Ef Ec Gc)^(1/3), ROHACELL 51 WF minimum değerler |
| W-SEC-OP-1 | kanat kesiti y 0,71 m: başlık bası birim şekil değiştirmesi (hasar toleransı, > 2 mm) | YD-02 | 1583,6 | 3000,0 | µε | FoS 1,5 | 0,263 | STANAG 4703 UL13.1.2 (3000 µε nihai yükte) |
| W-SEC-OP-2 | kanat kesiti y 0,71 m: başlık bası gerilmesi (A-tabanı tahmini 0,85 Fcu) | YD-02 | 202,1 | 656,0 | MPa | FoS 1,5 x kompozit 1,2 | 0,803 | CS-LUAS.613(b), AMC LUAS.619; malzeme Fcu B-tabanı ETW |
| W-SEC-OP-3 | kanat kesiti y 0,71 m: başlık çekme birim şekil değiştirmesi (hasar toleransı) | YD-02 | 1375,8 | 5000,0 | µε | FoS 1,5 | 1,423 | STANAG 4703 UL13.1.2 |
| W-SEC-OP-4 | kanat kesiti y 0,71 m: başlık çekme gerilmesi (A-tabanı tahmini 0,85 Ftu) | YD-02 | 175,6 | 1323,2 | MPa | FoS 1,5 x kompozit 1,2 | 3,187 | CS-LUAS.613(b) |
| W-SEC-OP-5 | kanat kesiti y 0,71 m: kaplama yüzü bası birim şekil değiştirmesi (sandviç, hasar toleransı) | YD-02 | 1710,9 | 2600,0 | µε | FoS 1,5 | 0,013 | STANAG 4703 UL13.1.2 (2600 µε sandviç kaplamalar) |
| W-SEC-OP-6 | kanat kesiti y 0,71 m: kaplama dış yüzü delikli bası (OHC) gerilmesi | YD-02 | 59,57 | 175,4 | MPa | FoS 1,5 x kompozit 1,2 | 0,636 | materials.yaml design_values_for_code OHC (PW-QI ETW2 B-tabanı) |
| W-SEC-OP-7 | kanat kesiti y 0,71 m: kaplama dış yüzü buruşma | YD-02 | 59,57 | 140,0 | MPa | FoS 1,5 x kompozit 1,2 | 0,305 | Zenkert 1995: 0,5 (Ef Ec Gc)^(1/3), ROHACELL 51 WF minimum değerler |
| W-WEB-01 | ana kiriş gövdesi y 0,70-0,85 m (6 kat ±45 PW, belirleyici y 0,70): gövde kesme burkulması (uzun levha, basit mesnet) | YD-04 | 30,69 | 63,43 | N/mm | FoS 1,5 x kompozit 1,2 | 0,148 | Kollár & Springer uzun levha kesme burkulması, CLT eğilme rijitliği |
| W-WEB-02 | ana kiriş gövdesi y 0,85-1,00 m (6 kat ±45 PW, belirleyici y 0,85): gövde kesme burkulması (uzun levha, basit mesnet) | YD-04 | 30,52 | 73,20 | N/mm | FoS 1,5 x kompozit 1,2 | 0,332 | Kollár & Springer uzun levha kesme burkulması, CLT eğilme rijitliği |
| W-WEB-03 | ana kiriş gövdesi y 1,00-1,15 m (6 kat ±45 PW, belirleyici y 1,00): gövde kesme burkulması (uzun levha, basit mesnet) | YD-04 | 29,64 | 78,54 | N/mm | FoS 1,5 x kompozit 1,2 | 0,472 | Kollár & Springer uzun levha kesme burkulması, CLT eğilme rijitliği |
| W-WEB-04 | ana kiriş gövdesi y 1,15-1,30 m (6 kat ±45 PW, belirleyici y 1,15): gövde kesme burkulması (uzun levha, basit mesnet) | YD-04 | 28,31 | 83,18 | N/mm | FoS 1,5 x kompozit 1,2 | 0,633 | Kollár & Springer uzun levha kesme burkulması, CLT eğilme rijitliği |
| W-WEB-05 | ana kiriş gövdesi y 1,30-1,45 m (5 kat ±45 PW, belirleyici y 1,30): gövde kesme burkulması (uzun levha, basit mesnet) | YD-04 | 26,51 | 50,28 | N/mm | FoS 1,5 x kompozit 1,2 | 0,054 | Kollár & Springer uzun levha kesme burkulması, CLT eğilme rijitliği |
| W-WEB-06 | ana kiriş gövdesi y 1,45-1,70 m (5 kat ±45 PW, belirleyici y 1,45): gövde kesme burkulması (uzun levha, basit mesnet) | YD-04 | 25,89 | 58,11 | N/mm | FoS 1,5 x kompozit 1,2 | 0,247 | Kollár & Springer uzun levha kesme burkulması, CLT eğilme rijitliği |
| W-WEB-07 | ana kiriş gövdesi y 1,70-1,95 m (5 kat ±45 PW, belirleyici y 1,70): gövde kesme burkulması (uzun levha, basit mesnet) | YD-04 | 22,87 | 64,31 | N/mm | FoS 1,5 x kompozit 1,2 | 0,562 | Kollár & Springer uzun levha kesme burkulması, CLT eğilme rijitliği |
| W-WEB-08 | ana kiriş gövdesi y 1,95-2,20 m (4 kat ±45 PW, belirleyici y 1,95): gövde kesme burkulması (uzun levha, basit mesnet) | YD-04 | 20,73 | 39,83 | N/mm | FoS 1,5 x kompozit 1,2 | 0,068 | Kollár & Springer uzun levha kesme burkulması, CLT eğilme rijitliği |
| W-WEB-09 | ana kiriş gövdesi y 2,20-2,45 m (4 kat ±45 PW, belirleyici y 2,20): gövde kesme burkulması (uzun levha, basit mesnet) | YD-04 | 18,53 | 48,09 | N/mm | FoS 1,5 x kompozit 1,2 | 0,442 | Kollár & Springer uzun levha kesme burkulması, CLT eğilme rijitliği |
| W-WEB-10 | ana kiriş gövdesi y 2,45-2,70 m (4 kat ±45 PW, belirleyici y 2,45): gövde kesme burkulması (uzun levha, basit mesnet) | YD-04 | 15,86 | 58,45 | N/mm | FoS 1,5 x kompozit 1,2 | 1,048 | Kollár & Springer uzun levha kesme burkulması, CLT eğilme rijitliği |
| W-WEB-11 | ana kiriş gövdesi y 2,70-2,95 m (3 kat ±45 PW, belirleyici y 2,70): gövde kesme burkulması (uzun levha, basit mesnet) | YD-04 | 13,38 | 33,97 | N/mm | FoS 1,5 x kompozit 1,2 | 0,411 | Kollár & Springer uzun levha kesme burkulması, CLT eğilme rijitliği |
| W-WEB-12 | ana kiriş gövdesi y 2,95-3,60 m (3 kat ±45 PW, belirleyici y 2,95): gövde kesme burkulması (uzun levha, basit mesnet) | YD-04 | 9,38 | 37,29 | N/mm | FoS 1,5 x kompozit 1,2 | 1,210 | Kollár & Springer uzun levha kesme burkulması, CLT eğilme rijitliği |
| W-RWEB | arka kiriş gövdesi dış panel (3 kat ±45, belirleyici y 0,71): gövde kesme burkulması (uzun levha, basit mesnet) | YD-04 | 9,01 | 33,19 | N/mm | FoS 1,5 x kompozit 1,2 | 1,046 | Kollár & Springer uzun levha kesme burkulması, CLT eğilme rijitliği |
| W-GLOVEWEB | iç pimin içinde eldiven ana kiriş gövdeleri (2 x 10 kat ±45 PW, çatal kulakları), kesme: gövde kesme birim şekil değiştirmesi (ince lamine, hasar toleransı) | YD-02 | 401,5 | 5200,0 | µε | FoS 1,5 | 7,634 | STANAG 4703 UL13.1.2 (5200 µε nihai yükte) |
| W-SKINBUCK-CT | orta kutu kapağı paneli 0,40 x 0,28 m (ct_box_cover 0,4/6/0,4 mm) burkulma, bası + kesme (belirleyici y 0,02) | YD-04 | 1 | 1,86 | yük çarpanı | FoS 1,5 x kompozit 1,2 | 0,035 | Rc + Rs^2 = 1; N_c 69 N/mm, N_s 70 N/mm (CLT D*, sandviç kesme düzeltmesi); Nx yüzeyin kesit birim şekil değiştirmesinden |
| W-SKINCRIMP-CT | orta kutu kapağı (ct_box_cover 0,4/6/0,4 mm): kesme kıvrılması (çekirdek kesme kararsızlığı) | YD-04 | 36,78 | 95,61 | N/mm | FoS 1,5 x kompozit 1,2 | 0,444 | Zenkert 1995: N = G_c d^2 / c (çekirdek minimum G); N = hypot(Nx, Nxy) |
| W-SKINBUCK-GLOVE-UP | eldiven üst kutu kaplaması paneli 0,15 x 0,26 m (wing_box_skin_upper 0,6/6/0,4 mm) burkulma, bası + kesme (belirleyici y 0,41) | YD-04 | 1 | 1,9 | yük çarpanı | FoS 1,5 x kompozit 1,2 | 0,055 | Rc + Rs^2 = 1; N_c 80 N/mm, N_s 91 N/mm (CLT D*, sandviç kesme düzeltmesi); Nx yüzeyin kesit birim şekil değiştirmesinden |
| W-SKINCRIMP-GLOVE-UP | eldiven üst kutu kaplaması (wing_box_skin_upper 0,6/6/0,4 mm): kesme kıvrılması (çekirdek kesme kararsızlığı) | YD-04 | 42,13 | 98,63 | N/mm | FoS 1,5 x kompozit 1,2 | 0,301 | Zenkert 1995: N = G_c d^2 / c (çekirdek minimum G); N = hypot(Nx, Nxy) |
| W-SKINBUCK-OP-UP-IN | dış panel üst kutu kaplaması y 0,70-2,20 paneli 0,35 x 0,25 m (wing_box_skin_upper 0,6/6/0,4 mm) burkulma, bası + kesme (belirleyici y 0,71) | YD-04 | 1 | 1,88 | yük çarpanı | FoS 1,5 x kompozit 1,2 | 0,043 | Rc + Rs^2 = 1; N_c 80 N/mm, N_s 81 N/mm (CLT D*, sandviç kesme düzeltmesi); Nx yüzeyin kesit birim şekil değiştirmesinden |
| W-SKINCRIMP-OP-UP-IN | dış panel üst kutu kaplaması y 0,70-2,20 (wing_box_skin_upper 0,6/6/0,4 mm): kesme kıvrılması (çekirdek kesme kararsızlığı) | YD-04 | 42,45 | 98,63 | N/mm | FoS 1,5 x kompozit 1,2 | 0,291 | Zenkert 1995: N = G_c d^2 / c (çekirdek minimum G); N = hypot(Nx, Nxy) |
| W-SKINBUCK-OP-UP-OUT | dış panel üst kutu kaplaması y 2,20-uç paneli 0,35 x 0,16 m (wing_skin_primary 0,6/5/0,4 mm) burkulma, bası + kesme (belirleyici y 2,20) | YD-04 | 1 | 2,19 | yük çarpanı | FoS 1,5 x kompozit 1,2 | 0,216 | Rc + Rs^2 = 1; N_c 75 N/mm, N_s 76 N/mm (CLT D*, sandviç kesme düzeltmesi); Nx yüzeyin kesit birim şekil değiştirmesinden |
| W-SKINCRIMP-OP-UP-OUT | dış panel üst kutu kaplaması y 2,20-uç (wing_skin_primary 0,6/5/0,4 mm): kesme kıvrılması (çekirdek kesme kararsızlığı) | YD-04 | 34,12 | 84,75 | N/mm | FoS 1,5 x kompozit 1,2 | 0,380 | Zenkert 1995: N = G_c d^2 / c (çekirdek minimum G); N = hypot(Nx, Nxy) |
| W-SKINBUCK-GLOVE-LO | eldiven alt kutu kaplaması (negatif durum) paneli 0,15 x 0,26 m (wing_skin_primary 0,6/5/0,4 mm) burkulma, bası + kesme (belirleyici y 0,41) | YD-04 | 1 | 2,41 | yük çarpanı | FoS 1,5 x kompozit 1,2 | 0,340 | Rc + Rs^2 = 1; N_c 66 N/mm, N_s 77 N/mm (CLT D*, sandviç kesme düzeltmesi); Nx yüzeyin kesit birim şekil değiştirmesinden |
| W-SKINCRIMP-GLOVE-LO | eldiven alt kutu kaplaması (negatif durum) (wing_skin_primary 0,6/5/0,4 mm): kesme kıvrılması (çekirdek kesme kararsızlığı) | YD-04 | 27,44 | 84,75 | N/mm | FoS 1,5 x kompozit 1,2 | 0,716 | Zenkert 1995: N = G_c d^2 / c (çekirdek minimum G); N = hypot(Nx, Nxy) |
| W-SKINBUCK-OP-LO | dış panel alt kutu kaplaması (negatif durum) paneli 0,35 x 0,25 m (wing_skin_primary 0,6/5/0,4 mm) burkulma, bası + kesme (belirleyici y 0,71) | YD-04 | 1 | 2,4 | yük çarpanı | FoS 1,5 x kompozit 1,2 | 0,336 | Rc + Rs^2 = 1; N_c 66 N/mm, N_s 67 N/mm (CLT D*, sandviç kesme düzeltmesi); Nx yüzeyin kesit birim şekil değiştirmesinden |
| W-SKINCRIMP-OP-LO | dış panel alt kutu kaplaması (negatif durum) (wing_skin_primary 0,6/5/0,4 mm): kesme kıvrılması (çekirdek kesme kararsızlığı) | YD-04 | 27,38 | 84,75 | N/mm | FoS 1,5 x kompozit 1,2 | 0,720 | Zenkert 1995: N = G_c d^2 / c (çekirdek minimum G); N = hypot(Nx, Nxy) |
| W-RIBCRUSH | kanat kaburgası gövdesi (rib_panel sandviç) ezilme yükü, kolon burkulması (y 0,91, aralık 0,35 m) | YD-02 | 1,81 | 81,85 | N/mm | FoS 1,5 x kompozit 1,2 | 24,105 | Niu ch. 9 / Brazier: w = s kappa^2 sum(E A |z|) / b; kesme düzeltmeli geniş kolon (Euler) |

### Kanat dış panel birleşimi (y 0,70)

| No | Eleman / kontrol | Yük durumu | Uygulanan | İzin verilen | Birim | Katsayılar | MS | Referans |
|---|---|---|---|---|---|---|---|---|
| J-PIN-SHEAR | ana pim Ø16 Ti-6Al-4V (dış pim; düşey çift, burulma çifti ve F_y/2 bileşkesi), çift kesme | YD-05 | 21496 | 241210 | N | FoS 1,5 x sık sökme 1,5 | 3,987 | Fsu Ti-6Al-4V tavlanmış (çubuk için sac değerleri, tahmin); STANAG UL2.4 sık sökme 1,5 |
| J-PIN-BEND | ana pim eğilmesi (Melcon-Hoblit kolu, plastik kazanç yok) | YD-05 | 339,4 | 923,9 | MPa | FoS 1,5 x sık sökme 1,5 | 0,210 | M = P/2 (t_prong 10/2 + gap 0,2 + t_tongue 30/4 mm); Ftu |
| J-PIN-BUSH | ana pimin 4130 kulak burçlarında ezilmesi (2 x 10 mm) | YD-05 | 21496 | 441280 | N | FoS 1,5 x ezilme 2 | 5,843 | MMPDS 4130 Fbru (bush backed by the laminate); ezilme katsayısı 2,0 |
| J-TONGUE-BUSH | dil göbeği: burç dış çapı 22 x 30 mm, [±45/0/90] blokta ezilme | YD-05 | 32,57 | 175,4 | MPa | FoS 1,5 x ezilme 2 | 0,795 | QI open-hole compression ETW B-tabanı as a conservative bearing limit (bore e/D below the 3 of the QI bearing value; element test required); ezilme katsayısı 2,0 |
| J-PRONG-BUSH | çatal kulak takviyeleri (10 mm): burç dış çapı 22 x 10 mm ezilmesi, iki kulak | YD-05 | 48,85 | 175,4 | MPa | FoS 1,5 x ezilme 2 | 0,197 | QI OHC ETW as the bearing limit (as J-TONGUE-BUSH) |
| J-TONGUE-FLANGE-TT | dil UD başlığı: göbek bloğu altında kalınlık yönünde bası (30 x 50 mm) | YD-05 | 14,33 | 90,67 | MPa | FoS 1,5 x sık sökme 1,5 | 1,812 | UD transverse compression F2cu (B-tabanı ETW); sık sökme 1,5 |
| J-TONGUE-FLANGE-DT | dış pimde dil UD başlığı 30 x 10 mm (M = R_iç d, + F_y/2 eksenel), bası birim şekil değiştirmesi | YD-05 | 1775,0 | 3000,0 | µε | FoS 1,5 | 0,127 | STANAG 4703 UL13.1.2 (3000 µε nihai yükte); boss block not credited |
| J-TONGUE-FLANGE | dış pimde dil UD başlığı, bası gerilmesi (tek yük yolu) | YD-05 | 226,5 | 656,0 | MPa | FoS 1,5 x kompozit 1,2 | 0,609 | A-basis estimate 0,85 Fcu (CS-LUAS.613(b)); composite factor 1,2 |
| J-TONGUE-WEB | pimler arasında dil ±45 gövdesi (25 kat), kesme R_iç: gövde kesme birim şekil değiştirmesi (ince lamine, hasar toleransı) | YD-05 | 3018,3 | 5200,0 | µε | FoS 1,5 | 0,149 | STANAG 4703 UL13.1.2 (5200 µε nihai yükte) |
| J-PRONG-WEB | pimler arasında çatal kulakları (eldiven ana kiriş gövdeleri, 2 x 10 kat ±45), kesme R_dış: gövde kesme birim şekil değiştirmesi (ince lamine, hasar toleransı) | YD-05 | 2890,5 | 5200,0 | µε | FoS 1,5 | 0,199 | STANAG 4703 UL13.1.2 (5200 µε nihai yükte) |
| J-TRANS-WEB | başlık rampası (başlık başına 14,3 mm basamak, 100 mm boyunca, 8,1°): dış uçta kırılma kuvveti 25 katlı dil gövdesine, 30 mm üzerinde gövde ezilmesi | YD-05 | 34,90 | 175,4 | MPa | FoS 1,5 x kompozit 1,2 | 1,792 | F_k = F_cap sin(theta); QI OHC ETW as the in-plane compression limit of the web - fix round 2, VS2-04 |
| J-TRANS-ILSS | başlık rampası: kırılma kuvvetinin başlık-gövde ara yüzey kesmesi (başlık genişliği x 30 mm) | YD-05 | 10,43 | 38,27 | MPa | FoS 1,5 x kompozit 1,2 | 1,038 | ILSS ETW B-tabanı (NCAMP PW) - fix round 2, VS2-04 |
| J-TRANS-RIB | başlık rampası: birleşim kaburgasında kırılma kuvveti 20 katlı dolu banda (4,0 x 30 mm) | YD-05 | 77,98 | 175,4 | MPa | FoS 1,5 x kompozit 1,2 | 0,250 | QI OHC ETW - fix round 2, VS2-04 |
| J-TRANS-CAP | başlık rampası: azalan moment kolu boyunca başlık birim şekil değiştirmesi (başlıklar dil hattına kaydırılmış, takviyeli alan; kaplamalar kesitte) | YD-05 | 1401,9 | 3000,0 | µε | FoS 1,5 | 0,427 | plane sections, ramp start / middle / end; damage-tolerance strain - fix round 2, VS2-04 |
| J-TRANS-SKIN | başlık rampası: azalan başlık kolu ile kaplama birim şekil değiştirmesi (kök bölmesi kaplama takviyesi dış yüze +1 kat 0/90, 120 mm) | YD-05 | 1707,2 | 2600,0 | µε | FoS 1,5 | 0,015 | plane sections; sandwich damage-tolerance strain - fix round 2, VS2-04 |
| J-REAR-PIN-SHEAR | arka pim Ø8 Ti-6Al-4V, çift kesme (C ve F_y) | YD-05 | 2403,0 | 60302 | N | FoS 1,5 x sık sökme 1,5 | 10,153 | Ti Fsu; sık sökme 1,5 |
| J-REAR-PIN-BEND | arka pim eğilmesi (Melcon-Hoblit) | YD-05 | 98,00 | 923,9 | MPa | FoS 1,5 x sık sökme 1,5 | 3,190 | Ftu |
| J-REAR-LUG | arka kiriş kulağı 7075 (t 8, w 32, e/D 2,0): eksenel F_y + enine C, eğik etkileşim | YD-05 | 1 | 7,69 | yük çarpanı | FoS 1,5 x sık sökme 1,5 (çarpan içinde) | 6,689 | structlib.lug_axial (Bruhn D1, net section / shear-bearing K_br F_tu D t - fix round 2, VS2-09: F_tu, not F_bru); transverse allowable 0,6 x axial (conservative Bruhn trend); (Ra^1.6 + Rtr^1.6) = 1 |
| J-REAR-BR | arka pim ezilmesi: kulak 8 mm ve yuva levhaları 2 x 4 mm (7075, e/D 2) | YD-05 | 2403,0 | 63983 | N | FoS 1,5 x ezilme 2 | 7,876 | MMPDS Fbru e/D 2; ezilme katsayısı 2,0 |
| J-REAR-PLATE | yuva levhası 4 mm, düşey burulma çifti altında eğilme (kulak yüzü teması, kol 16 mm) | YD-05 | 66,94 | 461,9 | MPa | FoS 1,5 x sık sökme 1,5 | 2,067 | elastic plate strip, Fty |
| J-REAR-BOLTS-RIB | yuva bağlantısı - birleşim kaburgası: 2 x M4 Ti, tek kesme (C, F_T) | YD-05 | 326,6 | 7537,8 | N | FoS 1,5 x bağlantı 1,15 | 12,378 | Ti Fsu gövde (shank) kesitinde |
| J-REAR-BR-RIB | yuva bağlantısı cıvataları M4: birleşim kaburgası bandında ezilme (3,2 mm) | YD-05 | 326,6 | 5341,9 | N | FoS 1,5 x kompozit 1,2 | 8,086 | QI ezilme ETW (pad >= 40 % +-45, e/D >= 3) |
| J-REAR-BOLTS-WEB | yuva bağlantısı - arka kiriş gövdesi: 2 x M5 Ti, tek kesme (F_y, F_T) | YD-05 | 1183,5 | 11778 | N | FoS 1,5 x bağlantı 1,15 | 4,769 | Ti Fsu gövde (shank) kesitinde |
| J-REAR-BR-WEB | yuva bağlantısı cıvataları M5: arka kiriş gövde takviyesinde ezilme (16 kat, 3,2 mm) | YD-05 | 1183,5 | 6677,3 | N | FoS 1,5 x kompozit 1,2 | 2,135 | QI ezilme ETW |
| J-REAR-LUGFIT | arka kulak bağlantısı - dış panel arka kirişi: 4 x M5 Ti, gövde takviyesinde ezilme (3,2 mm) | YD-05 | 607,3 | 6677,3 | N | FoS 1,5 x kompozit 1,2 | 5,108 | QI ezilme ETW; resultant of C, F_y, F_T shared equally |

### Orta kanat kutusu

| No | Eleman / kontrol | Yük durumu | Uygulanan | İzin verilen | Birim | Katsayılar | MS | Referans |
|---|---|---|---|---|---|---|---|---|
| CT-WEB-MS | orta kutu gövdesi = FS-MS çerçevesi (rib_panel + yüz başına 1 kat ±45), yüz kesmesi ilk katman hasarı | YD-06 | 46,60 | 186,0 | N/mm | FoS 1,5 x kompozit 1,2 | 1,218 | CLT, kat izin verilen değerleri B-tabanı ETW |
| CT-WEBDT-MS | orta kutu gövdesi FS-MS: yüz kesme birim şekil değiştirmesi (hasar toleransı) | YD-06 | 1866,1 | 5200,0 | µε | FoS 1,5 | 0,858 | STANAG UL13.1.2 5200 µε |
| CT-WEBCRIMP-MS | orta kutu gövdesi FS-MS: kesme kıvrılması | YD-06 | 46,60 | 101,7 | N/mm | FoS 1,5 x kompozit 1,2 | 0,213 | N = G_c d^2 / c |
| CT-WEB-RS | orta kutu gövdesi = FS-RS çerçevesi (rib_panel + yüz başına 0 kat ±45), yüz kesmesi ilk katman hasarı | YD-06 | 6,97 | 97,04 | N/mm | FoS 1,5 x kompozit 1,2 | 6,739 | CLT, kat izin verilen değerleri B-tabanı ETW |
| CT-WEBDT-RS | orta kutu gövdesi FS-RS: yüz kesme birim şekil değiştirmesi (hasar toleransı) | YD-06 | 535,9 | 5200,0 | µε | FoS 1,5 | 5,469 | STANAG UL13.1.2 5200 µε |
| CT-WEBCRIMP-RS | orta kutu gövdesi FS-RS: kesme kıvrılması | YD-06 | 6,97 | 95,61 | N/mm | FoS 1,5 x kompozit 1,2 | 6,625 | N = G_c d^2 / c |
| CT-ATT-BR-MS | kutu - FS-MS çerçevesi bağlantısı taraf başına 4 x M6 Ti: çerçeve kenar bandında ezilme (3,2 mm dolu lamine) | YD-07 | 727,7 | 8012,8 | N | FoS 1,5 x kompozit 1,2 | 5,117 | QI ezilme (%2 öteleme) ETW (materials.yaml design_values_for_code) |
| CT-ATT-SH-MS | kutu - FS-MS cıvataları M6 Ti, tek kesme | YD-07 | 727,7 | 16960 | N | FoS 1,5 x bağlantı 1,15 | 12,511 | Ti-6Al-4V Fsu |
| CT-ATT-BR-RS | kutu - FS-RS çerçevesi bağlantısı taraf başına 4 x M6 Ti: çerçeve kenar bandında ezilme (3,2 mm dolu lamine) | YD-07 | 341,4 | 8012,8 | N | FoS 1,5 x kompozit 1,2 | 12,038 | QI ezilme (%2 öteleme) ETW (materials.yaml design_values_for_code) |
| CT-ATT-SH-RS | kutu - FS-RS cıvataları M6 Ti, tek kesme | YD-07 | 341,4 | 16960 | N | FoS 1,5 x bağlantı 1,15 | 27,797 | Ti-6Al-4V Fsu |
| CT-KINK | y = 0 orta kırık bağlantısı (7075): ana başlık başına veter yönü kırılma kuvveti | YD-08 | 17281 | – | N | - | gereksinim | F = 2 F_cap sin(kink); kırık bağlantısıyla sandviç kapaklara ve FS-MS gövdesine aktarılır (detay tasarım) |
| CT-KINK-BOLTS | kırık bağlantısı kulağı - orta hat kaburgası: 5 x M6 Ti, tek kesme | YD-09 | 3456,2 | 16960 | N | FoS 1,5 x bağlantı 1,15 | 1,845 | Ti-6Al-4V Fsu - fix round 2, VS2-03 |
| CT-KINK-BR | kırık bağlantısı cıvataları: orta hat kaburgasının 16 katlı dolu bandında ezilme (3,2 mm) | YD-09 | 3456,2 | 8012,8 | N | FoS 1,5 x kompozit 1,2 | 0,288 | QI ezilme ETW |
| CT-KINK-RIB | orta hat kaburgası dolu bandı: kırılma kuvvetinin bağlantı boyunca (100 mm) düzlem içi kesmesi, ilk katman hasarı | YD-09 | 172,8 | 388,2 | N/mm | FoS 1,5 x kompozit 1,2 | 0,248 | CLT (50 % +-45 land) |
| CT-KINK-PLATE | kırık bağlantısı levhası 40 x 2 mm 7075: kırılma kuvveti çekme / bası | YD-09 | 172,8 | 461,9 | MPa | FoS 1,5 x bağlantı 1,15 | 0,550 | Fty |

### Kuyruk

| No | Eleman / kontrol | Yük durumu | Uygulanan | İzin verilen | Birim | Katsayılar | MS | Referans |
|---|---|---|---|---|---|---|---|---|
| T-SPINDLE | stabilatör mili 25 x 1,2 mm (ti_6al_4v_annealed_sheet), dış yatakta eğilme + burulma (von Mises) | YD-10 | 351,5 | 923,9 | MPa | FoS 1,5 x bağlantı 1,15 | 0,524 | M 169 N m, T 66,9 N m; MMPDS Ftu |
| T-SPINDLE-SPL | mil kama kesiti (duvardan 0,5 mm diş derinliği), yalnız burulma (kamada eğilme: T-SPINDLE-SPL-MT) | YD-10 | 115,4 | 599,8 | MPa | FoS 1,5 x sık sökme 1,5 | 1,311 | Fsu; sık sökme 1,5 (stabilatör taşıma için sökülür) |
| T-BRG-IN | mil iç yatağı 61805-ZZ (düğüm göbeği): gerekli statik yük sayısı C0 (nihai radyal yük) | YD-10 | 1452,3 | – | N | - | gereksinim | tedarik gereksinimi: C0 >= bu değer (katalog değeri araştırma verisinde yok) |
| T-BRG-OUT | mil dış yatağı 61805-ZZ: gerekli statik yük sayısı C0 (nihai radyal yük) | YD-10 | 2159,6 | – | N | - | gereksinim | tedarik gereksinimi: C0 >= bu değer |
| T-NODE-STUB-BOLTS | kök parçası arka kiriş kök bağlantısı - dış yanak: 4 x M6 12.9 (y ekseni), kök momentinden çekme + R_dış kesmesi, etkileşim (en yüklü cıvata) | YD-11 | 1 | 10,83 | yük çarpanı | FoS 1,5 x bağlantı 1,15 (çarpan içinde) | 9,835 | elastik cıvata grubu (tension about the pattern centroid, contact not credited); ISO 898-1 12.9: çekme Rm A_s, kesme 0,6 Rm A_s; R_s^2 + R_t^2 = 1 |
| T-NODE-STUB-BR | kök parçası cıvataları M6: 7 mm 7075 yanakta ezilme | YD-11 | 359,9 | 41989 | N | FoS 1,5 x bağlantı 1,15 | 66,628 | MMPDS Fbru (e/D >= 2) |
| T-NODE-CHEEK | düğüm dış yanağı 7 x 86 mm: kök parçası kök momentinden burulma (açık kesit b t^3/3) + R_dış düzlem içi eğilmesi, von Mises | YD-11 | 178,7 | 530,9 | MPa | FoS 1,5 x bağlantı 1,15 | 0,722 | arm of R_out to the base 60 mm; MMPDS Ftu |
| T-NODE-INBOARD | düğüm iç kolu (gövde 4 x 42 mm, tabandan 60 mm konsol) iç yatak yükü altında, eğilme | YD-11 | 49,23 | 530,9 | MPa | FoS 1,5 x bağlantı 1,15 | 5,251 | MMPDS Ftu |
| T-NODE-BOSS | iç yatak göbeği halkası (dış Ø42 / delik Ø37 mm, duvar 2,5 x 30 mm), yatak yükü altında halka eğilmesi | YD-11 | 194,8 | 530,9 | MPa | FoS 1,5 x bağlantı 1,15 | 0,580 | thin ring under diametral load M = P R / pi (Roark table 9,2 case 1, conservative: the arm web support not credited); MMPDS Ftu |
| T-NODE-BASE-BOLTS | düğüm taban flanşı - yangın perdesi katmanı: 7 x M5 12.9 (x ekseni), kesme + çekme etkileşimi (en yüklü cıvata) | YD-12 | 1 | 5,03 | yük çarpanı | FoS 1,5 x bağlantı 1,15 (çarpan içinde) | 4,027 | elastik cıvata grubu (structlib.bolt_group_inplane / bolt_group_tension); ISO 898-1 12.9 |
| T-NODE-FW-BR | düğüm taban cıvataları M5: yangın perdesi bandında ezilme (3,2 mm dolu lamine) | YD-12 | 1185,8 | 6677,3 | N | FoS 1,5 x kompozit 1,2 | 2,128 | QI ezilme ETW; stainless spacer tubes not credited |
| T-NODE-FW-CORE | düğüm tabanı altında yangın perdesi sandviçi: taban momentinin düzlem dışı cıvata çekmesi, destek levhası çevresinin çekme yarısında çekirdek kesmesi | YD-12 | 0,447 | 1 | MPa | FoS 1,5 x kompozit 1,2 | 0,242 | tension half-perimeter 198 mm, d 6,4 mm; ROHACELL 71 WF core insert (structures.sizing.firewall.core_insert) |
| T-CROSSBOLT | stabilatör tespit çapraz cıvatası M6 12.9 (çift kesme), menteşe ekseni ataleti 12 g | YD-13 | 205,7 | 29426 | N | FoS 1,5 x sık sökme 1,5 | 62,584 | ISO 898-1 12.9 |
| T-SOCKET-BR | stabilatör kök yuvası (7075 kovan) mil üzerinde ezilme, 0,10 m geçme | YD-10 | 4,04 | 999,7 | MPa | FoS 1,5 x ezilme 2 | 81,390 | doğrusal ezilme basıncı p = 6 M/(d L^2) + V/(d L); ezilme katsayısı 2,0 |
| T-SOCKET-TUBE | stabilatör kök yuvası kovanı 29 x 2,0 mm 7075, kökte eğilme | YD-10 | 149,9 | 530,9 | MPa | FoS 1,5 x sık sökme 1,5 | 0,574 | M 161 N m; MMPDS Ftu; sık sökme 1,5 |
| T-SOCKET-OD | kovanın stabilatör kök kaburgası / kiriş laminesi üzerinde ezilmesi (geçme uçları) | YD-10 | 3,49 | 175,4 | MPa | FoS 1,5 x ezilme 2 | 15,768 | p = 6 M/(D L^2) + V/(D L); QI OHC ETW as the bearing limit; ezilme katsayısı 2,0 |
| T-SPINDLE-SPL-MT | kama kökünde mil (mil ucundan 20 mm, 0,5 mm dişlerle incelmiş duvar): eğilme (yuva moment dağılımı) + burulma, von Mises | YD-10 | 208,0 | 923,9 | MPa | FoS 1,5 x sık sökme 1,5 | 0,974 | M_spline = 6 M_s / L^2 (l^2/2 - l^3/(3L)) = 16,7 N m (linear socket pressure, M_s 161 N m at the mouth); fix round 2, VS2-08 |
| T-SPLINE | 25 mm evolvent kama dişleri (0,9 d'de kesme, dişlerin yarısı etkin) | YD-10 | 1,68 | 599,8 | MPa | FoS 1,5 x sık sökme 1,5 | > 100 | basitleştirilmiş kama kesmesi; Fsu |
| T-STABSPAR | stabilatör panel kirişi başlığı (12 kat x 25 mm UD) yuva ucunda, bası birim şekil değiştirmesi | YD-10 | 370,7 | 3000,0 | µε | FoS 1,5 | 4,395 | derinlik 61 mm x/c 0,51; STANAG UL13.1.2 |
| T-STABSKIN | stabilatör kaplaması (tail_skin 0,6/4/0,4 mm) yuva ucunda: açıklık yönü basısında kesme kıvrılması | YD-10 | 19,31 | 70,93 | N/mm | FoS 1,5 x kompozit 1,2 | 1,041 | Zenkert 1995: N = G_c d^2 / c |
| T-FINSPAR | dikey ön kiriş başlığı (12 kat x 25 mm UD) kök bağlantısında, bası birim şekil değiştirmesi | YD-14 | 338,6 | 3000,0 | µε | FoS 1,5 | 4,907 | STANAG UL13.1.2 |
| T-FINSKIN | dikey kaplaması (tail_skin 0,6/4/0,4 mm) kökte: eğilme basısında kesme kıvrılması | YD-14 | 16,97 | 70,93 | N/mm | FoS 1,5 x kompozit 1,2 | 1,321 | Zenkert 1995: N = G_c d^2 / c |
| T-FINROOT-FRONT | dikey ön kiriş kök bağlantısı YK250-CH-096: 2 x M6 12.9 çift kesme (başlık cıvataları 45 mm aralık) | YD-14 | 2957,5 | 29426 | N | FoS 1,5 x bağlantı 1,15 | 4,768 | ISO 898-1 12.9 |
| T-FINROOT-BR-FRONT | dikey ön kiriş kök bağlantısı: 7075 köşebent kulak ezilmesi (t 5 mm) | YD-14 | 2957,5 | 29992 | N | FoS 1,5 x bağlantı 1,15 | 4,879 | MMPDS Fbru |
| T-FINFIT-FR-FRONT | dikey bağlantısı YK250-CH-096 - FS3480: 2 x M6, çerçeve kenar bandında ezilme (3,2 mm, en küçük cıvata) | YD-14 | 2957,5 | 8012,8 | N | FoS 1,5 x kompozit 1,2 | 0,505 | QI ezilme ETW |
| T-FINROOT-REAR | dikey arka kiriş kök bağlantısı YK250-CH-086: 2 x M6 12.9 çift kesme (başlık cıvataları 45 mm aralık) | YD-14 | 1980,6 | 29426 | N | FoS 1,5 x bağlantı 1,15 | 7,613 | ISO 898-1 12.9 |
| T-FINROOT-BR-REAR | dikey arka kiriş kök bağlantısı: 7075 köşebent kulak ezilmesi (t 5 mm) | YD-14 | 1980,6 | 29992 | N | FoS 1,5 x bağlantı 1,15 | 7,779 | MMPDS Fbru |
| T-FINFIT-FR-REAR | dikey bağlantısı YK250-CH-086 - FS3670: 2 x M5 + 2 x M8, çerçeve kenar bandında ezilme (3,2 mm, en küçük cıvata) | YD-14 | 990,3 | 6677,3 | N | FoS 1,5 x kompozit 1,2 | 2,746 | QI ezilme ETW |
| T-VENTRAL-BOLT | ventral kök bağlantıları: en yüklü kulak cıvatası M6 12.9 çift kesme | YD-15 | 3095,4 | 29426 | N | FoS 1,5 x bağlantı 1,15 | 4,511 | elastik cıvata grubu (omurga hattında 3 kulak) |
| T-VENTRAL-LUG | ventral kök kulağı ezilmesi (7075, t 6 mm) | YD-15 | 3095,4 | 35991 | N | FoS 1,5 x bağlantı 1,15 | 5,740 | MMPDS Fbru |
| T-AFTKEEL | arka omurga kirişi M-AFTKEEL (talaşlı 7075-T651 U 24 x 40 x 2,5) FS3738 gerisinde konsol, eğilme | YD-15 | 273,2 | 461,9 | MPa | FoS 1,5 | 0,127 | M = en arka ventral kulak yükü x FS3738'e kol; Fty (akma üstünde yerel flanş buruşması hesaba katılmadı) |
| T-NODE-TEMP | silindir kafası bölgesinde stabilatör düğümü F-SPINDLE-NODE (7075-T651): düğüm tasarım sıcaklığında metal satırlarının gerektirdiği dayanım oranı | YD-10 | 0,633 | – | F_tu(T) / F_tu(RT) | - | gereksinim | belirleyici T-NODE-BOSS (MS 0,58 at room temperature): the 7075 strength at the node temperature must stay >= this fraction of RT; MMPDS elevated-temperature curves are not in the research data and the node temperature is not known (61805-ZZ bearing, stainless baffle between heads and node, layout.heat_protection): open item - measure in the engine run (fix round 2, PK2-09) |

### Kumanda yüzeyleri

| No | Eleman / kontrol | Yük durumu | Uygulanan | İzin verilen | Birim | Katsayılar | MS | Referans |
|---|---|---|---|---|---|---|---|---|
| C-HINGE-AILERON | kanatçık menteşesi: Ø4 paslanmaz pim, 3 mm 7075 dirsek kulağında ezilme (6,67 toplam) | YD-16 | 241,2 | 12907 | N | 6,67 toplam | 7,022 | CS-LUAS.657 menteşe ezilme katsayısı 6,67; MMPDS Fbru |
| C-HPIN-AILERON | kanatçık menteşe pimi Ø4 (304), çift kesme | YD-16 | 241,2 | 8664,3 | N | FoS 1,5 x bağlantı 1,15 | 19,821 | MMPDS 304 Fsu |
| C-HLINE-AILERON | kanatçık menteşe ekseni dayanağı (eksenel ataleti tek menteşe taşır), dirsek kulağı ezilmesi | YD-17 | 53,51 | 12907 | N | 6,67 toplam | 35,160 | CS-LUAS.657 |
| C-RODEND-AILERON | kanatçık itme çubuğu ucu cıvatası M3 A2-70 boynuzda (t 3 mm 7075), ezilme (3,33 toplam) | YD-18 | 540,9 | 9680,4 | N | 3,33 toplam | 4,375 | CS-LUAS.693 itme-çekme bağlantı katsayısı 3,33 |
| C-PUSHROD-AILERON | kanatçık itme çubuğu 7075 boru 6 x 1 mm, L 160 mm, kolon | YD-18 | 540,9 | 1397,7 | N | FoS 1,5 | 0,723 | Johnson-Euler (Euler), mafsallı uçlar |
| C-ACTMOUNT-AILERON | kanatçık eyleyici bağlantısı: servo çerçevesinde 4 x M4 gömülü burç (2,0 mm dolu lamine), burç başına yüz ezilmesi (moment payı 1,5) | YD-18 | 202,8 | 7512,0 | N | FoS 1,5 x kompozit 1,2 | 19,575 | burç flanşı çapında (9 mm) QI ezilme |
| C-GGUST-AILERON | kanatçık yer rüzgârı menteşe momenti / eyleyici anma torku x en küçük bağlantı oranı (enerjili tutma) | YD-19 | 1,32 | 5,4 | N m | limit / anma (işlevsel) | 3,086 | CS-VLA 415; pay sağlanırsa rüzgâr kilidi gerekmez |
| C-HINGE-FLAP | flap menteşesi: Ø4 paslanmaz pim, 3 mm 7075 dirsek kulağında ezilme (6,67 toplam) | YD-20 | 306,7 | 12907 | N | 6,67 toplam | 5,310 | CS-LUAS.657 menteşe ezilme katsayısı 6,67; MMPDS Fbru |
| C-HPIN-FLAP | flap menteşe pimi Ø4 (304), çift kesme | YD-20 | 306,7 | 8664,3 | N | FoS 1,5 x bağlantı 1,15 | 15,379 | MMPDS 304 Fsu |
| C-HLINE-FLAP | flap menteşe ekseni dayanağı (eksenel ataleti tek menteşe taşır), dirsek kulağı ezilmesi | YD-17 | 79,00 | 12907 | N | 6,67 toplam | 23,494 | CS-LUAS.657 |
| C-RODEND-FLAP | flap itme çubuğu ucu cıvatası M3 A2-70 boynuzda (t 3 mm 7075), ezilme (3,33 toplam) | YD-21 | 1395,7 | 9680,4 | N | 3,33 toplam | 1,083 | CS-LUAS.693 itme-çekme bağlantı katsayısı 3,33 |
| C-PUSHROD-FLAP | flap itme çubuğu 7075 boru 6 x 1 mm, L 100 mm, kolon | YD-21 | 1395,7 | 3578,2 | N | FoS 1,5 | 0,709 | Johnson-Euler (Euler), mafsallı uçlar |
| C-ACTMOUNT-FLAP | flap eyleyici bağlantısı: servo çerçevesinde 4 x M4 gömülü burç (2,0 mm dolu lamine), burç başına yüz ezilmesi (moment payı 1,5) | YD-21 | 523,4 | 7512,0 | N | FoS 1,5 x kompozit 1,2 | 6,974 | burç flanşı çapında (9 mm) QI ezilme |
| C-GGUST-FLAP | flap yer rüzgârı menteşe momenti / eyleyici anma torku x en küçük bağlantı oranı (enerjili tutma) | YD-19 | 3,19 | 16,00 | N m | limit / anma (işlevsel) | 4,019 | CS-VLA 415; pay sağlanırsa rüzgâr kilidi gerekmez |
| C-HINGE-RUDDER | dümen menteşesi: Ø4 paslanmaz pim, 3 mm 7075 dirsek kulağında ezilme (6,67 toplam) | YD-22 | 205,9 | 12907 | N | 6,67 toplam | 8,398 | CS-LUAS.657 menteşe ezilme katsayısı 6,67; MMPDS Fbru |
| C-HPIN-RUDDER | dümen menteşe pimi Ø4 (304), çift kesme | YD-22 | 205,9 | 8664,3 | N | FoS 1,5 x bağlantı 1,15 | 23,395 | MMPDS 304 Fsu |
| C-HLINE-RUDDER | dümen menteşe ekseni dayanağı (eksenel ataleti tek menteşe taşır), dirsek kulağı ezilmesi | YD-23 | 102,3 | 12907 | N | 6,67 toplam | 17,918 | CS-LUAS.657 |
| C-RODEND-RUDDER | dümen itme çubuğu ucu cıvatası M3 A2-70 boynuzda (t 3 mm 7075), ezilme (3,33 toplam) | YD-24 | 758,9 | 9680,4 | N | 3,33 toplam | 2,831 | CS-LUAS.693 itme-çekme bağlantı katsayısı 3,33 |
| C-PUSHROD-RUDDER | dümen itme çubuğu 7075 boru 6 x 1 mm, L 80 mm, kolon | YD-24 | 758,9 | 4975,8 | N | FoS 1,5 | 3,371 | Johnson-Euler (Johnson), mafsallı uçlar |
| C-ACTMOUNT-RUDDER | dümen eyleyici bağlantısı: servo çerçevesinde 4 x M4 gömülü burç (2,0 mm dolu lamine), burç başına yüz ezilmesi (moment payı 1,5) | YD-24 | 284,6 | 7512,0 | N | FoS 1,5 x kompozit 1,2 | 13,664 | burç flanşı çapında (9 mm) QI ezilme |
| C-GGUST-RUDDER | dümen yer rüzgârı menteşe momenti / eyleyici anma torku x en küçük bağlantı oranı (enerjili tutma) | YD-19 | 2,44 | 5,4 | N m | limit / anma (işlevsel) | 1,214 | CS-VLA 415; pay sağlanırsa rüzgâr kilidi gerekmez |

### İniş takımı

| No | Eleman / kontrol | Yük durumu | Uygulanan | İzin verilen | Birim | Katsayılar | MS | Referans |
|---|---|---|---|---|---|---|---|---|
| G-MLEG | ana bacak dış silindiri 42 x 3,5 mm 7075, mafsal çatalında eğilme + eksenel (belirleyici: level_spring_back) | YD-25 | 179,0 | 537,8 | MPa | FoS 1,5 | 1,003 | elastik, Ftu (çubuk için 7075-T6 sac değerleri, tahmin) |
| G-TRUN-BR | mafsal yatak kulağı (7075, Ø20, t 12 mm, e/D 1,09 < 2: kulak istisnası), kesme-ezilme (belirleyici: level_spring_back) | YD-25 | 7776,1 | 68231 | N | FoS 1,5 x ezilme 2 | 1,925 | Bruhn D1.5 lug shear-bearing at the actual e/D: P = K_br F_tu D t, K_br 0,54 (structlib.lug_axial, conservative fit); ezilme katsayısı 2,0 (dönen bağlantı) - fix round 1, S1-09 |
| G-TRUN-NET | mafsal kulağı net kesiti (w 49 mm, D 20 mm) | YD-25 | 7776,1 | 169766 | N | FoS 1,5 x bağlantı 1,15 | 11,656 | Bruhn D1: K_t 0,92 F_tu (w - D) t |
| G-TRUN-PIN | mafsal muylusu Ø20 (4130), tek kesme | YD-25 | 7776,1 | 123465 | N | FoS 1,5 x bağlantı 1,15 | 8,204 | MMPDS 4130 Fsu |
| G-TRUN-AXLE-BEND | mafsal muylusu Ø20 eğilmesi: yatak yükü, çatal kolu yüzünden burç merkezine 13 mm kol (konsol), elastik | YD-25 | 128,7 | 655,0 | MPa | FoS 1,5 x bağlantı 1,15 | 1,950 | M = R_b arm, sigma = 32 M / (pi d^3); MMPDS 4130 Ftu (no plastic bending credit) - fix round 2, PK2-04 stub-axle scheme |
| G-TRUN-SO | mafsal kulağı kesme yırtılması (e 21,9 mm) | YD-25 | 7776,1 | 101318 | N | FoS 1,5 x bağlantı 1,15 | 6,553 | Bruhn D1: 2 t (e - D/2 cos 40) Fsu at the actual edge distance |
| G-DOWNLOCK | ana takım aşağı kilit cıvatası Ø10 (4130), toplama ekseninden r 35 mm, çift kesme (belirleyici: level_max_vertical) | YD-25 | 7961,8 | 61732 | N | FoS 1,5 x bağlantı 1,15 | 3,495 | mafsal ekseni etrafında moment / kilit yarıçapı |
| G-DOWNLOCK-BEND | ana takım aşağı kilit cıvatası Ø10 çatalda eğilme (Melcon-Hoblit: kulaklar 6 mm, kilit kulağı 10 mm, boşluklar 0,5 mm), elastik (belirleyici: level_max_vertical) | YD-25 | 243,3 | 655,0 | MPa | FoS 1,5 x bağlantı 1,15 | 0,561 | structlib.pin_bending_moment / pin_bending_stress, Ftu (no plastic factor) - fix round 2, VS2-07; requirement for the gear-unit supplier |
| G-FIT-BOLTS | mafsal bağlantısı cıvata grubu (5 x M6 12.9 y ekseni takım kirişine, 4 x M6 12.9 z ekseni kuyu tavanına): esnek cıvatalar üzerinde rijit bağlantı, mafsalda kuvvet + üç moment, kesme / çekme etkileşimi (belirleyici: level_spring_back, bolt B5) | YD-25 | 1 | 3,58 | yük çarpanı | FoS 1,5 x bağlantı 1,15 (çarpan içinde) | 1,075 | structlib.bolt_group_6dof (equal bolt stiffness, estimate); ISO 898-1 12.9; R_s^2 + R_t^2 = 1 - fix round 2, VS2-02 (was: |F| / 10 x 1,5 bearing only) |
| G-FIT-BR | mafsal bağlantısı cıvataları: en yüklü cıvatanın 16 katlı dolu bantta ezilmesi (t 3,2 mm) (belirleyici: level_spring_back, bolt B5) | YD-25 | 2382,8 | 8012,8 | N | FoS 1,5 x kompozit 1,2 | 0,868 | QI ezilme ETW (6-DOF bolt shear) |
| G-FIT-PULL-BEAM | takım kirişi ankrajları: ankraj flanşının (OD 14) 16 katlı banttan sıyrılması, en yüklü cıvata (belirleyici: side_inboard) | YD-25 | 1124,1 | 5403,5 | N | FoS 1,5 x kompozit 1,2 | 1,671 | punching shear pi D t ILSS ETW B-tabanı (estimate until test) |
| G-FIT-PULL-ROOF | kuyu tavanı kubbe somun plakaları: somun plakası tabanının (20 mm) 16 katlı tavan bandından sıyrılması, en yüklü cıvata (belirleyici: level_spring_back) | YD-25 | 1423,5 | 9828,4 | N | FoS 1,5 x kompozit 1,2 | 2,836 | punching shear (square base perimeter) x t x ILSS ETW B-tabanı (estimate until test) |
| G-BEAM-CAP | ana takım kirişi UD başlıkları (12 kat x 20 mm) bacak hizasında eğilme (çentik derinliği 67 mm), düşey yük + sürükleme çifti M_y, bası birim şekil değiştirmesi (belirleyici: level_spring_back) | YD-25 | 1944,8 | 3000,0 | µε | FoS 1,5 | 0,028 | kuyu duvarı ile FS-GEAR arasında basit mesnetli; the lower cap runs up round the notch (fix round 2, VS2-02 / PK2-03) |
| G-BEAM-WEB | ana takım kirişi sandviç duvarı (rib_panel) bağlantı yanında yüz kesmesi (tam derinlik 91 mm), düşey yük + sürükleme çifti, ilk katman hasarı (belirleyici: level_spring_back) | YD-25 | 43,21 | 97,04 | N/mm | FoS 1,5 x kompozit 1,2 | 0,248 | CLT |
| G-BEAM-NOTCH | bacak çentiğinde takım kirişi: kesme F-TRUNNION'ın çentiği köprüleyen 7075 flanşında (43 x 8 mm; çentikli kompozit duvar sayılmadı) (belirleyici: level_spring_back) | YD-25 | 16,32 | 296,5 | MPa | FoS 1,5 x bağlantı 1,15 | 9,534 | parabolic shear 1,5 V / (h t), Fsu - fix round 2, PK2-03 / VS2-02 |
| G-BEAM-END | takım kirişi uç bağlantıları: uç başına 3 x M5 Ti, sürükleme çifti M_y / L dahil uç tepkisinin kesmesi (belirleyici: level_spring_back) | YD-25 | 1253,1 | 11778 | N | FoS 1,5 x bağlantı 1,15 | 4,449 | Ti Fsu, single shear |
| G-BEAM-TORSION | bacak yuvarlanma momenti M_x ve sürükleme çifti F-TRUNNION cıvatalarından: kuyu tavanı ve alt kaplama kenarları arasında takım kirişi duvarı (rib_panel), cıvata kuvvetleri altında levha eğilmesi (6 serbestlik), yüz ilk katman hasarı (belirleyici: side_inboard) | YD-25 | 30,48 | 98,85 | N/mm | FoS 1,5 x kompozit 1,2 | 0,802 | simply supported strip, effective width = bolt-pattern length + 30 mm; face force = m / d; CLT - fix round 2, VS2-02 (defined M_x path: beam wall and well roof as plates, their edges on the roof / skin / keel-web diaphragms) |
| G-ROOF-TORSION | bacak yuvarlanma momenti M_x ve sürükleme çifti F-TRUNNION cıvatalarından: takım kirişi ile omurga perdesi arasında kuyu tavanı (yakıt bölmesi tabanı + bağlantı altında yüz başına 1 kat 0/90), cıvata kuvvetleri altında levha eğilmesi (6 serbestlik), yüz ilk katman hasarı (belirleyici: side_inboard) | YD-25 | 125,7 | 282,7 | N/mm | FoS 1,5 x kompozit 1,2 | 0,249 | simply supported strip, effective width = bolt-pattern length + 30 mm; face force = m / d; CLT - fix round 2, VS2-02 (defined M_x path: beam wall and well roof as plates, their edges on the roof / skin / keel-web diaphragms) |
| G-NLEG | burun bacağı 36 x 3,0 mm 7075, mafsal çatalında eğilme + eksenel (belirleyici: three_point_spin_up) | YD-26 | 83,46 | 537,8 | MPa | FoS 1,5 | 3,296 | elastik, Ftu |
| G-NPIVOT-BR | burun mafsal blokları (7075, 2 x 6 mm, flanşlı burç dış çapı 22, e/D 0,91): burcun bloklarda kesme-ezilmesi (belirleyici: three_point_level) | YD-26 | 1367,3 | 51489 | N | FoS 1,5 x ezilme 2 | 11,552 | Bruhn D1.5 lug at the actual e/D (K_br 0,37, F_tu); ezilme katsayısı 2,0 |
| G-NPIVOT-PIN | burun mafsal muyluları 2 x Ø16 (4130, dolu, flanşlı), her biri tek kesme | YD-26 | 1367,3 | 158035 | N | FoS 1,5 x bağlantı 1,15 | 66,002 | MMPDS 4130 Fsu; the pivot load shared by the two axles |
| G-NPIVOT-AXLE-BEND | burun mafsal muylusu Ø16 eğilmesi: mafsal yükünün yarısı, çatal kolu yüzünden burç merkezine 9 mm kol (konsol), elastik | YD-26 | 15,30 | 655,0 | MPa | FoS 1,5 x bağlantı 1,15 | 23,816 | M = (P/2) arm, sigma = 32 M / (pi d^3); MMPDS 4130 Ftu (no plastic bending credit) - fix round 2, PK2-04 stub-axle scheme |
| G-TOW-NLEG | çekme yükü altında burun bacağı (çatal dingili; belirleyici: +30 deg from the aft drag axis) | YD-27 | 60,61 | 537,8 | MPa | FoS 1,5 | 4,915 | elastik, Ftu - fix round 2, VS2-12 |
| G-TOW-PIVOT | çekme yükü altında burun mafsal blokları (burçların kesme-ezilmesi) | YD-27 | 441,0 | 51489 | N | FoS 1,5 x ezilme 2 | 37,918 | Bruhn D1.5 lug (as G-NPIVOT-BR); ezilme katsayısı 2,0 |
| G-NPIVOT-WALL | burun mafsal bloğu - omurga duvarı: taraf başına 4 x M6 Ti, CFRP bantta ezilme (t 3,2 mm) | YD-26 | 256,4 | 8012,8 | N | FoS 1,5 x kompozit 1,2 | 16,363 | en yüklü cıvata 1,5 x ortalama; QI ezilme ETW |
| G-NLOCK | burun aşağı kilidi (keel duvarlarına ölü nokta bağlantısı): mafsal ekseni etrafında limit moment | YD-26 | 192,0 | – | N m | - | gereksinim | belirleyici three_point_spin_up; kilit bağlantısı (takım ünitesi) bu moment x 1,5 x 1,15 için boyutlandırılır |
| G-KEELWALL | omurga duvarı (rib_panel) burun mafsalı ile FS0600 arasında kesme, yüz ilk katman hasarı | YD-26 | 2,71 | 97,04 | N/mm | FoS 1,5 x kompozit 1,2 | 18,902 | CLT |
| G-EMA-MAIN | ana takım toplama EMA'sı: mafsal ekseninde gerekli çıkış torku (n 2'de ağırlık momenti x 1,25; sürükleme eksene paralel) | YD-28 | 14,03 | – | N m | - | gereksinim | takım ünitesi EMA gereksinimi (CS-LUAS.395 benzeri) |
| G-EMA-NOSE | burun takımı toplama EMA'sı: mafsal ekseninde gerekli çıkış torku (1,25 x (n 2'de ağırlık momenti + teker/bacak sürüklemesi x bacak boyu, akışa karşı açılma)) | YD-28 | 25,71 | – | N m | - | gereksinim | takım ünitesi EMA gereksinimi |
| G-DOOR-HINGE | ana iç kapak menteşeleri (3 x Ø4 pim, 3 mm 7075 kulak), ezilme | YD-29 | 47,91 | 12907 | N | FoS 1,5 x ezilme 2 | 88,803 | ezilme katsayısı 2,0 (dönen bağlantı) |
| G-DOOR-LOCK | ana iç kapak: kapalı konumda tutma momenti (gereksinim: ölü nokta bağlantısı veya kapak kilidi; DA 22 anma torku 1,8 N m için bağlantı oranı >= değer / 1,8) | YD-29 | 10,35 | – | N m | - | gereksinim | nihai tutma momenti; kapalı kapağı tutmak için eyleyiciye güvenilmez |
| G-UPLOCK | ana yukarı kilit bağlantısı F-UPLOCK: kuyu tavanının dökme insertlerine 4 x M5 (dökme r 12 mm, çekirdek 5,6 mm), insert sökülmesi (en yüklü, kanca kaçıklığı) | YD-51 | 171,8 | 422,2 | N | FoS 1,5 x kompozit 1,2 | 0,366 | structlib.insert_pullout P = 2 pi b_p c tau_c (core minimum shear, no test correction) |

### Motor bağlantısı

| No | Eleman / kontrol | Yük durumu | Uygulanan | İzin verilen | Birim | Katsayılar | MS | Referans |
|---|---|---|---|---|---|---|---|---|
| E-STRUT_C | motor bağlantı çubuğu 12,7 x 0,89 mm 4130, kolon (Johnson-Euler, K 1) | YD-30 | 1 | 1,93 | yük çarpanı | katsayılı yükte çarpan | 0,928 | doğrudan rijitlik yöntemiyle uzay kafes (structlib.truss3d); MMPDS 4130 N, kaynak yakını Ftu 80 ksi (AC 43.13-1B); ISO 898-1 12.9; yükler zaten x FoS (limit durumlar) ve x 1,15 (cıvatalar) |
| E-STRUT_T | motor bağlantı çubuğu, kaynak yakını Ftu ile çekme | YD-30 | 1 | 2,1 | yük çarpanı | katsayılı yükte çarpan | 1,104 | doğrudan rijitlik yöntemiyle uzay kafes (structlib.truss3d); MMPDS 4130 N, kaynak yakını Ftu 80 ksi (AC 43.13-1B); ISO 898-1 12.9; yükler zaten x FoS (limit durumlar) ve x 1,15 (cıvatalar) |
| E-RING_C | sönümleyici halkası 15,9 x 0,89 mm 4130, kolon + yerel eğilme | YD-30 | 1 | 2,95 | yük çarpanı | katsayılı yükte çarpan | 1,945 | doğrudan rijitlik yöntemiyle uzay kafes (structlib.truss3d); MMPDS 4130 N, kaynak yakını Ftu 80 ksi (AC 43.13-1B); ISO 898-1 12.9; yükler zaten x FoS (limit durumlar) ve x 1,15 (cıvatalar) |
| E-RING_T | sönümleyici halkası, çekme (kaynak yakını Ftu) | YD-30 | 1 | 3,43 | yük çarpanı | katsayılı yükte çarpan | 2,432 | doğrudan rijitlik yöntemiyle uzay kafes (structlib.truss3d); MMPDS 4130 N, kaynak yakını Ftu 80 ksi (AC 43.13-1B); ISO 898-1 12.9; yükler zaten x FoS (limit durumlar) ve x 1,15 (cıvatalar) |
| E-BOLT | sönümleyicilerdeki karter cıvataları M8 12.9, kesme + çekme etkileşimi (x 1,15 bağlantı) | YD-30 | 1 | 15,38 | yük çarpanı | katsayılı yükte çarpan | 14,375 | doğrudan rijitlik yöntemiyle uzay kafes (structlib.truss3d); MMPDS 4130 N, kaynak yakını Ftu 80 ksi (AC 43.13-1B); ISO 898-1 12.9; yükler zaten x FoS (limit durumlar) ve x 1,15 (cıvatalar) |
| E-FOOT | yangın perdesi ayakları, her biri 2 x M8 12.9, kesme + çekme etkileşimi (x 1,15 bağlantı) | YD-30 | 1 | 6,45 | yük çarpanı | katsayılı yükte çarpan | 5,450 | doğrudan rijitlik yöntemiyle uzay kafes (structlib.truss3d); MMPDS 4130 N, kaynak yakını Ftu 80 ksi (AC 43.13-1B); ISO 898-1 12.9; yükler zaten x FoS (limit durumlar) ve x 1,15 (cıvatalar) |
| E-ISOLATOR | elastomer sönümleyici: gerekli nihai yük kapasitesi (sönümleyici başına en büyük bileşke; emniyet tamponu tutmalı) | YD-30 | 1796,2 | – | N | - | gereksinim | tedarik gereksinimi (Limbach sönümleyici verisi yayımlanmamış) |
| E-FOOT-BR | yangın perdesi ayakları: 2 x M8, yangın perdesi bandında ezilme (3,2 mm dolu lamine), düzlem içi ayak tepkisi | YD-31 | 3572,3 | 10684 | N | yalnız nihai x kompozit 1,2 | 1,492 | QI ezilme ETW; stainless spacer tubes not credited |

### Paraşüt

| No | Eleman / kontrol | Yük durumu | Uygulanan | İzin verilen | Birim | Katsayılar | MS | Referans |
|---|---|---|---|---|---|---|---|---|
| P-SHACKLE | kilit pimi Ø8 (Ti-6Al-4V), çift kesme | YD-32 | 13100 | 60302 | N | yalnız nihai x bağlantı 1,15 | 3,003 | Fsu of the pin material |
| P-SHACKLE-BEND | kilit pimi Ø8 eğilmesi (Melcon-Hoblit: kulaklar 6 mm, kayış makarası 5 mm, boşluklar 0,5 mm), elastik | YD-32 | 619,0 | 923,9 | MPa | yalnız nihai x bağlantı 1,15 | 0,298 | M = P/2 (t_o/2 + g + t_i/4) (structlib.pin_bending_moment), elastic stress against Ftu (no plastic bending factor credited) - fix round 2, VS2-01 |
| P-SPOOL-BR | kayış makarası (OD 12, 5 mm) pim üzerinde ezilme | YD-32 | 13100 | 55160 | N | yalnız nihai x ezilme 2 | 1,105 | Fbru; ezilme katsayısı 2,0 (rotating) |
| P-LUG-BR | U-kulak kulakları 2 x 6 mm 7075 ezilme (dönen kilit, e/D 1,94) | YD-32 | 13100 | 92976 | N | yalnız nihai x ezilme 2 | 2,549 | MMPDS Fbru at e/D 2 x (e/D)/2 below e/D 2 (linear reduction, conservative); ezilme katsayısı 2,0 |
| P-LUG | U-kulak (w 31, e 15,5 = 1,94 D, 2 x 6 mm) net kesit / kesme-ezilme | YD-32 | 13100 | 65938 | N | yalnız nihai x bağlantı 1,15 | 3,377 | structlib.lug_axial (Bruhn D1 eğilimi; shear-bearing K_br F_tu D t - fix round 2, VS2-09: F_tu, not F_bru) |
| P-FRAME-BOLTS | bağlantı - çerçeve: 2 x M5 12.9 (x ekseni), y/z bileşeninin tek kesmesi | YD-32 | 6550,0 | 10394 | N | yalnız nihai x bağlantı 1,15 | 0,380 | ISO 898-1 12.9, 0,6 Rm A_s |
| P-FRAME-BR | 2 çerçeve cıvatası altında çerçeve bandı ezilmesi (dolu lamine 32 kat, 6,4 mm) | YD-32 | 6550,0 | 13355 | N | yalnız nihai x kompozit 1,2 | 0,699 | QI ezilme ETW |
| P-FRAME | FS1810 / FS-RS çerçeve gövdesi (rib_panel): y/z bileşeninin çerçevenin iki yanına kesmesi, yüz ilk katman hasarı | YD-32 | 43,67 | 97,04 | N/mm | yalnız nihai x kompozit 1,2 | 0,852 | gövde derinliği 0,15 m kenar uzun kirişlerine kadar; CLT |
| P-SPINE-BOLTS | bağlantı - omurga tabanı: 4 x M4 12.9 (z ekseni): x bileşeni kesmesi + bağlantı momentinin çekmesi (pim çerçeve yüzünden 26 mm, tabandan 8 mm; topuk çerçeve yüzünde), etkileşim | YD-33 | 1 | 1,91 | yük çarpanı | yalnız nihai x bağlantı 1,15 (çarpan içinde) | 0,909 | ISO 898-1 12.9; R_s^2 + R_t^2 = 1 (fix round 2, VS2-01: Fz x arm now included) |
| P-SPINE-PULL | taban cıvatalarının omurga takviyesinden sıyrılması (16 kat, 3,2 mm): dört taban cıvatasının altındaki 48 x 40 mm pul levhası çevresinde zımbalama kesmesi | YD-33 | 10113 | 21623 | N | yalnız nihai x kompozit 1,2 | 0,782 | ILSS ETW B-tabanı (NCAMP PW) as the through-thickness shear estimate until a pull-through test exists - fix round 2, VS2-01 |
| P-SPINE-BR | omurga taban takviyesi (16 kat, 3,2 mm) 4 taban cıvatası altında ezilme | YD-34 | 2836,2 | 5341,9 | N | yalnız nihai x kompozit 1,2 | 0,570 | QI ezilme ETW (pad >= 40 % +-45, edge 16 mm = 3,2 D) |
| P-SPINE-AX | omurga kanalı M-SPINE (44 x 24 x 0,8 mm, flanşlar 60 mm): tam x bileşeninin eksenel gerilmesi | YD-34 | 66,89 | 175,4 | MPa | yalnız nihai x kompozit 1,2 | 1,185 | QI open-hole compression ETW (screw holes along the flanges) |
| P-SPINE-COL | çerçeveler arasında kolon olarak omurga kanalı (L 0,382 m, mafsallı, kaplama desteği sayılmadı) | YD-34 | 11345 | 48613 | N | yalnız nihai x kompozit 1,2 | 2,571 | Euler, QI modulus |
| P-SPINE-LOCAL | omurga kanalı duvarları (24 mm, iki kenar mesnetli) eksenel gerilme altında yerel burkulma | YD-34 | 66,89 | 174,2 | MPa | yalnız nihai x kompozit 1,2 | 1,170 | SS plate k = 4 (Bruhn C5), QI modulus |
| P-SPINE-SCREWS | omurga flanşları - P-MB-UPPER: 30 x M4 A2-70 (25 mm aralık, FS1810 - FS-FUEL), x bileşeninin kesmesi (yuvarlak baş 0,8 x sınıf) | YD-34 | 378,2 | 2950,1 | N | yalnız nihai x bağlantı 1,15 | 5,784 | ISO 3506-1 A2-70 |
| P-SPINE-SCREW-BR | omurga flanşı vidaları M4: kaplamanın 1,6 mm dolu kenar bandında ezilme | YD-34 | 378,2 | 2662,1 | N | yalnız nihai x kompozit 1,2 | 4,866 | QI ezilme ETW |
| P-SPINE-SKIN | görev bölmesi üst kaplaması P-MB-UPPER (shell_secondary): net x bileşeninin omurga flanşlarından kenar uzun kirişlerine kesme akışı, yüz ilk katman hasarı | YD-34 | 14,85 | 97,04 | N/mm | yalnız nihai x kompozit 1,2 | 4,445 | CLT |

### Yakıt bölmeleri

| No | Eleman / kontrol | Yük durumu | Uygulanan | İzin verilen | Birim | Katsayılar | MS | Referans |
|---|---|---|---|---|---|---|---|---|
| F-FACE-008 | FS-FUEL perdesi (rib_panel 0,4/6/0,4 mm), açıklık 0,212 m: yüz ilk katman hasarı | YD-35 | 18,43 | 98,85 | N/mm | nihai x kompozit 1,2 | 3,470 | sandviç şerit p b^2/8; CLT B-tabanı ETW |
| F-DT-008 | FS-FUEL perdesi: yüz birim şekil değiştirmesi (sandviç, hasar toleransı) | YD-35 | 1040,9 | 2600,0 | µε | nihai | 1,498 | STANAG UL13.1.2 |
| F-CORE-008 | FS-FUEL perdesi: çekirdek kesmesi (core_rohacell_51wf) | YD-35 | 0,348 | 0,5 | MPa | nihai x kompozit 1,2 | 0,198 | tau = (p b / 2) / d; ROHACELL minimum değerler |
| F-FACE-029 | ön yakıt hücresi tabanı M-FWDDECK yük rayları arasında (rib_panel 0,4/6/0,4 mm), açıklık 0,200 m: yüz ilk katman hasarı | YD-35 | 16,40 | 98,85 | N/mm | nihai x kompozit 1,2 | 4,022 | sandviç şerit p b^2/8; CLT B-tabanı ETW |
| F-DT-029 | ön yakıt hücresi tabanı M-FWDDECK yük rayları arasında: yüz birim şekil değiştirmesi (sandviç, hasar toleransı) | YD-35 | 926,4 | 2600,0 | µε | nihai | 1,806 | STANAG UL13.1.2 |
| F-CORE-029 | ön yakıt hücresi tabanı M-FWDDECK yük rayları arasında: çekirdek kesmesi (core_rohacell_51wf) | YD-35 | 0,328 | 0,5 | MPa | nihai x kompozit 1,2 | 0,270 | tau = (p b / 2) / d; ROHACELL minimum değerler |
| F-FACE-030 | arka hücre tabanı / kuyu tavanı M-WELLROOF (fuel_floor_wellroof 0,6/5,6/0,6 mm), açıklık 0,373 m: yüz ilk katman hasarı | YD-35 | 59,04 | 190,7 | N/mm | nihai x kompozit 1,2 | 1,691 | sandviç şerit p b^2/8; CLT B-tabanı ETW |
| F-DT-030 | arka hücre tabanı / kuyu tavanı M-WELLROOF: yüz birim şekil değiştirmesi (sandviç, hasar toleransı) | YD-35 | 1889,8 | 2600,0 | µε | nihai | 0,376 | STANAG UL13.1.2 |
| F-CORE-030 | arka hücre tabanı / kuyu tavanı M-WELLROOF: çekirdek kesmesi (core_rohacell_71wf) | YD-35 | 0,632 | 1 | MPa | nihai x kompozit 1,2 | 0,318 | tau = (p b / 2) / d; ROHACELL minimum değerler |
| F-PAYRAIL | faydalı yük rayı (6061-T6 T 20 x 20 x 2,5) ön güverte takviyesi olarak, eğilme | YD-35 | 113,6 | 289,6 | MPa | nihai x bağlantı 1,15 | 1,216 | basit mesnetli, açıklık 0,297 m; MMPDS 6061-T6 Ftu |
| F-WELLWEB | kuyu omurga gövdesi M-WELLKEEL (iki gövdeden her biri, dolu PW 1,0 mm, 120 mm yükseklik) kuyu tavanı tepkisiyle eğilme | YD-35 | 10,16 | 175,4 | MPa | nihai x kompozit 1,2 | 13,390 | kiriş, açıklık 0,223 m (kuyu ön duvarından FS-GEAR'e); OHC (menteşe dirseği delikleri) |
| F-WELLWEB-BUCK | kuyu omurga gövdesi: düzlem içi eğilmede levha burkulması (k 23,9, basit mesnet) | YD-35 | 10,16 | 65,06 | MPa | nihai x kompozit 1,2 | 4,337 | Bruhn C5 / Timoshenko: k_b = 23,9 saf eğilmede; QI modülü (materials.yaml) |

### Taret asansörü ve kapakları

| No | Eleman / kontrol | Yük durumu | Uygulanan | İzin verilen | Birim | Katsayılar | MS | Referans |
|---|---|---|---|---|---|---|---|---|
| TU-SCREW | bilyalı vida d 8 x 2 ve somun: gerekli statik eksenel yük sayısı (nihai) | YD-36 | 611,0 | – | N | - | gereksinim | tedarik gereksinimi (katalog değeri araştırma verisinde yok) |
| TU-RAIL | minyatür profil raylar (her birinde 2 taşıyıcı): taşıyıcı başına gerekli statik yük sayısı (ileri + yan, nihai) | YD-36 | 270,1 | – | N | - | gereksinim | tedarik gereksinimi |
| TU-ROOF | tavan yatak bloğu: 4 x M4 A2-70 geçme cıvata, 2 mm 7075 karşı plaka, çekme (asılı yük) | YD-36 | 132,8 | 6146,0 | N | yalnız nihai x bağlantı 1,15 | 39,234 | ISO 3506-1 A2-70 Rm A_s |
| TU-RAILSCR | ray ucu vidaları ray ucu başına 4 x M3 A2-70 (2 ray, 2 uç), kesme | YD-36 | 29,35 | 2112,6 | N | yalnız nihai x bağlantı 1,15 | 61,583 | 0,6 Rm A_s |
| TU-DOORDRIVE | kayar bölme kapağı tahriki: emme altında kapağı yürütme pinyon torku (ray sürtünmesi mu 0,3 tahmin, pinyon r 10 mm) / DA 22 anma torku | YD-37 | 0,133 | 1,8 | N m | FoS 1,5 (işlevsel) | 8,014 | components.yaml volz_da22_28v anma torku |
| TU-DOORRAIL | kayar kapak ray dudağı (6061-T6 ray, 2 ray x 0,12 m), emmede kapak tutma, 1 mm dudak kesmesi | YD-38 | 44,38 | 44678 | N | FoS 1,5 x bağlantı 1,15 | > 100 | 6061-T6 Fsu (ray dudağı, tahmini geometri) |

### Taşıma ve elleçleme

| No | Eleman / kontrol | Yük durumu | Uygulanan | İzin verilen | Birim | Katsayılar | MS | Referans |
|---|---|---|---|---|---|---|---|---|
| TR-PAD | kızak yastığı 200 x 50 mm, çerçeve bandı üzerindeki alt kaplamada: çekirdek ezilmesi (ROHACELL 51 WF), yastığın dolu kenar bandı dışına taştığı yerde | YD-39 | 0,14 | 0,4 | MPa | FoS 1,5 x kompozit 1,2 | 0,588 | ROHACELL 51 WF minimum Fcu |
| TR-FRAME | kızak üzerinde çerçeve gövdesi kenar basısı (rib_panel yüzleri) | YD-39 | 8,72 | 175,4 | MPa | FoS 1,5 x kompozit 1,2 | 10,178 | OHC (bağlantı delikli kenar bandı) |
| TR-PANEL-LE | dış panel hücum kenarı üzerinde yastıklı rafta (2 yastık 0,10 x 0,04 m): hücum kenarı dolu lamine ezilmesi | YD-40 | 0,0298 | 0,4 | MPa | FoS 1,5 x kompozit 1,2 | 6,453 | muhafazakâr: yastık basıncı bitişik sandviçin çekirdek ezilme dayanımıyla karşılaştırıldı |

### Gövde

| No | Eleman / kontrol | Yük durumu | Uygulanan | İzin verilen | Birim | Katsayılar | MS | Referans |
|---|---|---|---|---|---|---|---|---|
| B-DORSAL | sırt uzun kirişi (şapka 25 x 20 x 2,0, UD başlık + PW) FS-GEAR'de, birim şekil değiştirme (<= 2 mm) | YD-41 | 611,5 | 2600,0 | µε | FoS 1,5 | 1,834 | STANAG UL13.1.2; yalnız uzun kirişler (kaplamalar = kesme gövdeleri) |
| B-CHINE-AFT | kenar uzun kirişi (J 35 x 30 x 2,4) FS-GEAR'de, birim şekil değiştirme (> 2 mm) | YD-41 | 459,0 | 3000,0 | µε | FoS 1,5 | 3,357 | STANAG UL13.1.2 |
| B-DORSAL-COL | FS-GEAR ile FS3480 arasında sırt uzun kirişi kolonu (L 0,382 m, yalnız şapka, PW modülü, mafsallı) | YD-41 | 7042,4 | 42908 | N | FoS 1,5 x kompozit 1,2 | 2,385 | Euler; bağlı kaplama (25-32 mm somun plakaları) hesaba katılmadı |
| B-CHINE-FWD | FS-FUEL'de kenar uzun kirişi, birim şekil değiştirme (> 2 mm) | YD-42 | 494,8 | 3000,0 | µε | FoS 1,5 | 3,042 | STANAG UL13.1.2 |
| B-MIDFLOOR | alt başlık olarak görev bölmesi tabanı M-MIDFLOOR (tepsi kesiği dışında sabit dış şeritler net 0,470 m, rib_panel yüzleri), birim şekil değiştirme (sandviç) | YD-42 | 912,2 | 2600,0 | µε | FoS 1,5 | 0,900 | STANAG UL13.1.2 |
| B-MIDFLOOR-BUCK | görev bölmesi taban şeridi 0,38 x 0,235 m, kenar uzun kirişi ile kesik kenarı takviyesi arasında (2 yapıştırılmış şapka takviye) burkulma (bası) | YD-42 | 1 | 2,25 | yük çarpanı | FoS 1,5 x kompozit 1,2 | 0,250 | CLT D*, sandviç kesme düzeltmesi; SS edges |
| B-SKIN-SHEAR | sabit arka kaplamalar P-AFT-UPPER / -LOWER (shell_secondary 0,4/5/0,4): yüz kesmesi, ilk katman hasarı | YD-43 | 12,53 | 97,04 | N/mm | FoS 1,5 x kompozit 1,2 | 3,302 | CLT |
| B-SKIN-BUCK | sabit arka kaplama paneli kesme burkulması (çerçeve aralığı x uzun kirişler arası 0,25 m) | YD-43 | 1 | 4,87 | yük çarpanı | FoS 1,5 x kompozit 1,2 | 1,705 | Kollár & Springer uzun levha, sandviç düzeltmesi |
| B-SCREW-SH | kaplama vida sırası ISO 7380 M4 A2-70, 32 mm aralık: kesme (yuvarlak baş 0,8 x sınıf) | YD-43 | 401,0 | 2950,1 | N | FoS 1,5 x bağlantı 1,15 | 3,264 | ISO 3506-1 A2-70; materials.yaml screw_types |
| B-SCREW-BR | kaplama vida sırası M4: 1,6 mm dolu kenar bandında ezilme | YD-43 | 401,0 | 2662,1 | N | FoS 1,5 x kompozit 1,2 | 2,688 | QI ezilme ETW |

### frames

| No | Eleman / kontrol | Yük durumu | Uygulanan | İzin verilen | Birim | Katsayılar | MS | Referans |
|---|---|---|---|---|---|---|---|---|
| FR-GEAR-WEB | FS-GEAR gövdesi takım kirişi uç bağlantısında (h 87 mm): kiriş ucu tepkisinin kesmesi, yüz ilk katman hasarı | YD-44 | 41,63 | 97,04 | N/mm | FoS 1,5 x kompozit 1,2 | 0,295 | CLT (rib_panel faces) |
| FR-GEAR-CRIMP | FS-GEAR gövdesi kiriş ucunda: kesme kıvrılması | YD-44 | 41,63 | 95,61 | N/mm | FoS 1,5 x kompozit 1,2 | 0,276 | N = G_c d^2 / c |
| FR-GEAR-BR | takım kirişi uç bağlantısı - FS-GEAR: 3 x M5 Ti, çerçeve bandında ezilme (3,2 mm) | YD-44 | 1207,3 | 6677,3 | N | FoS 1,5 x kompozit 1,2 | 2,073 | QI ezilme ETW |
| FR-RS-WEB | FS-RS gövdesi takım kirişi uç bağlantısında (h 87 mm): kiriş ucu tepkisinin kesmesi, yüz ilk katman hasarı | YD-45 | 43,21 | 97,04 | N/mm | FoS 1,5 x kompozit 1,2 | 0,248 | CLT (rib_panel faces) |
| FR-RS-CRIMP | FS-RS gövdesi kiriş ucunda: kesme kıvrılması | YD-45 | 43,21 | 95,61 | N/mm | FoS 1,5 x kompozit 1,2 | 0,229 | N = G_c d^2 / c |
| FR-RS-BR | takım kirişi uç bağlantısı - FS-RS: 3 x M5 Ti, çerçeve bandında ezilme (3,2 mm) | YD-45 | 1253,1 | 6677,3 | N | FoS 1,5 x kompozit 1,2 | 1,960 | QI ezilme ETW |
| FR-ROOF-SIDE | kuyu tavanı M-WELLROOF: takım yan yükü 4 tavan cıvatasıyla M6 (z ekseni), 3,2 mm bantta ezilme | YD-46 | 183,8 | 8012,8 | N | FoS 1,5 x kompozit 1,2 | 23,226 | QI ezilme ETW (fuel_floor_wellroof 0,6/5,6/0,6 mm deck, solid land) |
| FR-MS-RING | FS-MS çerçeve gövdesi orta kutu dışında gövde yanı bağlantısında (rib_panel + yüz başına 1 kat ±45): gövde ataleti kesmesi, yüz ilk katman hasarı | YD-47 | 39,65 | 186,0 | N/mm | FoS 1,5 x kompozit 1,2 | 1,606 | CLT; FS-MS / FS-RS are full sandwich bulkheads above the box and 0,15 m wide posts beside the payload-bay cut-out below it, the body inertia enters at the box ends |
| FR-RS-RING | FS-RS çerçeve gövdesi orta kutu dışında gövde yanı bağlantısında (rib_panel + yüz başına 0 kat ±45): gövde ataleti kesmesi, yüz ilk katman hasarı | YD-48 | 17,99 | 97,04 | N/mm | FoS 1,5 x kompozit 1,2 | 1,997 | CLT; FS-MS / FS-RS are full sandwich bulkheads above the box and 0,15 m wide posts beside the payload-bay cut-out below it, the body inertia enters at the box ends |
| FW-FOOT-CORE | alt ayakta yangın perdesi sandviçi: bant kenarında en büyük çekirdek kesmesi (ROHACELL 71 WF parçası; tepe / ortalama 1,23) | YD-49 | 0,79 | 1 | MPa | yalnız nihai x kompozit 1,2 | 0,054 | tau_peak = k P / (2 pi r d), k from the plate-shear distribution (fix round 2, VS2-06); core minimum value |
| FW-FOOT-LAND | yangın perdesi dolu bandı (24 kat, 4,8 mm) destek levhasında eğilme | YD-49 | 119,2 | 175,4 | MPa | yalnız nihai x kompozit 1,2 | 0,226 | simply supported circular plate, central patch (Timoshenko sec. 19); QI OHC ETW |
| FW-FOOT-FACE | bant kenarında yangın perdesi sandviç yüzleri: kenar desteğine plaka eğilmesi | YD-49 | 95,01 | 175,4 | MPa | yalnız nihai x kompozit 1,2 | 0,538 | simply supported circular plate a = edge distance (Timoshenko sec. 19); QI OHC ETW |
| FW-UPPER-SPLICE | üst ayak x yükü sırt uzun kirişine: köşe bağlantısı eki 2 x M5 Ti, uzun kirişte ezilme (2,0 mm) | YD-50 | 887,9 | 4159,6 | N | yalnız nihai x kompozit 1,2 | 2,904 | QI ezilme ETW |
| FR-3738-KEEL | kuyruk tamponu çarpmasında FS3738'de arka omurga desteği: düşey tepki (omurga yangın perdesi ucunda mafsallı, FS3738'de destekli), talaşlı 7075 alt parça taşır (FR-3738-*) | YD-52 | 10208 | – | N | - | gereksinim | moment of the strike about the keel firewall end 617 N m / support spacing 0,0604 m (fix round 2, VS2-03: sized by FR-3738-SEG / -SEG-SH / -KEEL-* / -END-*) |
| FR-3738-SEG | FS3738 alt parçası (talaşlı 7075 I 60 x 20 x 2 / 2,5 mm, halka bacakları arası 0,30 m): omurga tepkisi altında eğilme (basit mesnetli, yük ortada) | YD-52 | 221,1 | 461,9 | MPa | FoS 1,5 | 0,393 | M = R L / 4; Fty - fix round 2, VS2-03 |
| FR-3738-SEG-SH | FS3738 alt parçası gövde kesmesi (halka bacaklarında) | YD-52 | 36,46 | 296,5 | MPa | FoS 1,5 | 4,421 | average web shear, Fsu |
| FR-3738-KEEL-BOLTS | FS3738 alt parçası: arka omurga - parça gövdesi 4 x M5 12.9, tek kesme | YD-52 | 2552,1 | 10394 | N | FoS 1,5 x bağlantı 1,15 | 1,361 | ISO 898-1 12.9, 0,6 Rm A_s |
| FR-3738-KEEL-BR | FS3738 alt parçası: omurga cıvatalarının 7075 gövdede ezilmesi (2,5 mm) | YD-52 | 2552,1 | 12497 | N | FoS 1,5 x bağlantı 1,15 | 1,839 | MMPDS Fbru (e/D 2) |
| FR-3738-END-BOLTS | FS3738 alt parçası: her halka bacağına ek 3 x M5 12.9, tek kesme | YD-52 | 1701,4 | 10394 | N | FoS 1,5 x bağlantı 1,15 | 2,542 | ISO 898-1 12.9, 0,6 Rm A_s |
| FR-3738-END-BR | FS3738 alt parçası: ek cıvatalarının 7075 gövdede ezilmesi (2,5 mm) | YD-52 | 1701,4 | 12497 | N | FoS 1,5 x bağlantı 1,15 | 3,258 | MMPDS Fbru (e/D 2) |

### Teçhizat tutma

| No | Eleman / kontrol | Yük durumu | Uygulanan | İzin verilen | Birim | Katsayılar | MS | Referans |
|---|---|---|---|---|---|---|---|---|
| EQ-BUFFER_BATTE | tampon batarya (12S2P Li-ion kutu): gerekli nihai tutma yükü (aşağı / ileri) | YD-53 | 253,9 | – | N | - | gereksinim | kütle 2,08 kg; tepsi / dirsek / kayış tasarım yükü x 1,15 bağlantı |
| EQ-PDU_DCDC_FUS | PDU / DC-DC / sigortalar: gerekli nihai tutma yükü (aşağı / ileri) | YD-53 | 269,3 | – | N | - | gereksinim | kütle 2,21 kg; tepsi / dirsek / kayış tasarım yükü x 1,15 bağlantı |
| EQ-AVIONICS | aviyonik (otopilot, veri bağları): gerekli nihai tutma yükü (aşağı / ileri) | YD-53 | 79,52 | – | N | - | gereksinim | kütle 0,65 kg; tepsi / dirsek / kayış tasarım yükü x 1,15 bağlantı |
| EQ-PARACHUTE_UA | paraşüt kabı: gerekli nihai tutma yükü (aşağı / ileri) | YD-53 | 577,1 | – | N | - | gereksinim | kütle 4,72 kg; tepsi / dirsek / kayış tasarım yükü x 1,15 bağlantı |
| EQ-FLIGHT_TERMI | UST + ışıklar: gerekli nihai tutma yükü (aşağı / ileri) | YD-53 | 51,18 | – | N | - | gereksinim | kütle 0,42 kg; tepsi / dirsek / kayış tasarım yükü x 1,15 bağlantı |
| EQ-PARA-BRK | paraşüt kabı dirsekleri 4 x (2 x M5 A2-70), kesme (bir dirsek çifti yarısını taşır) | YD-53 | 144,3 | 5964,0 | N | yalnız nihai x bağlantı 1,15 | 34,944 | 0,6 Rm A_s |
| EQ-PAYTRAY | faydalı yük tepsisi (en büyük yük 17,25 kg) 4 x M5 A2-70 tutsak vida, kesme | YD-54 | 384,9 | 5964,0 | N | yalnız nihai x bağlantı 1,15 | 12,473 | 0,6 Rm A_s |

## Yük durumları

| Kod | Yük durumu |
|---|---|
| YD-01 | malzeme yeterlilik verisi (NCAMP MTM45-1/AS4, materials.yaml) |
| YD-02 | simetrik hamle/manevra n = 5,46, MTOM (limit), negatif -3,55 (hamle matrisinin en büyük |n m| değeri MTOM'a göre); referans yamuk üzerinde Schrenk taşıma, kanat yapısı, kumanda yüzeyleri ve eyleyicilerin atalet rahatlatmasıyla |
| YD-03 | CS-LUAS.349(a): bir tarafta n_A 3,8, diğerinde 2/3, VA'da tam kanatçık burulması |
| YD-04 | simetrik hamle/manevra n = 5,46, MTOM (limit), negatif -3,55 (hamle matrisinin en büyük |n m| değeri MTOM'a göre); referans yamuk üzerinde Schrenk taşıma, kanat yapısı, kumanda yüzeyleri ve eyleyicilerin atalet rahatlatmasıyla; burulma: VD'de cm0, VA'da tam kanatçık (CS-LUAS.349(b)), VF'de 40° flap (zarf) |
| YD-05 | PHAA: n 5,46, kesit açısı 11,9° (kanat CLmaks); birleşim düzlemi y 0,70: V 2590 N, M 3230 N m, T 104 N m, C 547 N, Mz 682 N m (limit); pimler 184 mm aralıklı: R_iç 18517 N, R_dış 21107 N, burulma çifti 357 N, sürükleme çifti F_y 2340 N (dx 292 mm) |
| YD-06 | kanat tasarım durumu, orta kutu kesmesi V 3226 N, y 0,20 (limit) + burulma kesme akışı 3303 N/m |
| YD-07 | gövde ataleti m 129,6 kg x n 5,46 (limit), gövde AM x 2,576: FS-MS 5016 N, FS-RS 1926 N; gövde yanı kaburgasında kanat burulması 126 N m, taraf başına çerçeve çifti 403 N |
| YD-08 | kanat tasarım durumu (limit) |
| YD-09 | kanat tasarım durumu (limit): ana başlık başına ok kırılma kuvveti 17281 N (F = 2 F_başlık sin(kırık), y = 0'da F_başlık 62085 N) |
| YD-10 | VA'da tam sapmış stabilatör, CN_maks: panel başına N 472 N (limit; VC hamlesi 257 N), y 0,679; eyleyici durma torku 66,9 N m (DA 30 tepe x en büyük bağlantı oranı 4,18, CS-LUAS.395) |
| YD-11 | VA'da tam sapmış stabilatör, CN_maks: panel başına N 472 N (limit; VC hamlesi 257 N), y 0,679; eyleyici durma torku 66,9 N m (DA 30 tepe x en büyük bağlantı oranı 4,18, CS-LUAS.395); düğüm: iç yatak R_iç 968 N (y 0,145), kök parçası uç yatağı R_dış 1440 N (y 0,320), yanak yüzünde (y 0,2195) kök parçası kök momenti 145 N m |
| YD-12 | VA'da tam sapmış stabilatör, CN_maks: panel başına N 472 N (limit; VC hamlesi 257 N), y 0,679; eyleyici durma torku 66,9 N m (DA 30 tepe x en büyük bağlantı oranı 4,18, CS-LUAS.395); düğüm tabanı: x ekseni etrafında düzlem içi moment 225 N m, net düşey 472 N, y ekseni etrafında moment 32 N m (limit) |
| YD-13 | menteşe eksenine paralel atalet 12 x panel ağırlığı (CS-LUAS.393(b)) |
| YD-14 | dikey yan kuvveti 577 N (limit, tek dikey; 15° x 1,5 yanal kayma 563 N, tam dümen 577 N, yanal hamle 427 N, kesit CN_maks 1,47 ile sınırlı), açıkta kalan OAV'de, kök momenti 209 N m |
| YD-15 | kuyruk tamponu çarpması (tasarım kararı): skidden 45° yukarı-geri 1,0 x MTOM ağırlığı (CS-LUAS Ek H H.9 yönü; burun tekerli uçak tamponu için kuyruk tekeri kuralı yok) |
| YD-16 | kanatçık: yüzey yükü 386 N (limit) = max(CS-VLA Ek B w 789 Pa x 0,104 m2, durma menteşe momenti 13,0 N m / 0,4 c_f); 3 menteşe |
| YD-17 | menteşe eksenine paralel atalet 12 x yüzey ağırlığı (CS-LUAS.393(b)) |
| YD-18 | kanatçık: eyleyici tepe torku 5,0 N m x en büyük bağlantı oranı 2,60 = boynuzda 13,0 N m (CS-LUAS.395: sistem limit yükü eyleyici çıktısıdır) |
| YD-19 | yer rüzgârı V 18,1 m/s, K 0,75 (CS-VLA 415) |
| YD-20 | flap: yüzey yükü 836 N (limit) = max(CS-VLA Ek B w 539 Pa x 0,169 m2, durma menteşe momenti 41,9 N m / 0,4 c_f); 4 menteşe |
| YD-21 | flap: eyleyici tepe torku 16,0 N m x en büyük bağlantı oranı 2,62 = boynuzda 41,9 N m (CS-LUAS.395: sistem limit yükü eyleyici çıktısıdır) |
| YD-22 | dümen: yüzey yükü 329 N (limit) = max(CS-VLA Ek B w 851 Pa x 0,117 m2, durma menteşe momenti 18,2 N m / 0,4 c_f); 3 menteşe |
| YD-23 | menteşe eksenine paralel atalet 24 x yüzey ağırlığı (CS-LUAS.393(b)) |
| YD-24 | dümen: eyleyici tepe torku 5,0 N m x en büyük bağlantı oranı 3,64 = boynuzda 18,2 N m (CS-LUAS.395: sistem limit yükü eyleyici çıktısıdır) |
| YD-25 | CS-LUAS Ek H: V_çökme 2,35 m/s, d 92 mm, e_f 0,63, n_j 5,40, n 6,07; ana bacak başına P_v 3970 N (limit) |
| YD-26 | burun takımı: ön AM'de statik yük 253 N; üç noktalı düz iniş n_j payı (CS-LUAS H.4), ek durumlar arka 2,25/1,8, ön 3,2/0,9, yan 2,25/1,575 x statik (H.10, STANAG) |
| YD-27 | çekme (tasarım kararı): burun çatalı dingilinde 0,30 W = 441 N çekme yükü, geri / ileri ve iki yana 30° (W <= 30 000 lb için CS-23.509 / FAR 23,509 çekme yükü 0,3 W, muhafazakâr tasarım kararı olarak: CS-LUAS / CS-VLA çekme maddesi metni araştırma dosyalarında yok; çekme demiri burun çatalı dingilinde (layout.chassis.ground_handling)) |
| YD-28 | takım işletimi V_LO 38,3 m/s EAS'e kadar (tasarım kararı 1,6 VS, UKS ile sınırlı), n = 2,0 (CS-LUAS.345) |
| YD-29 | kapalı ana iç kapak, emme |Cp| 1,0 x q(VD) 1990 Pa (üst sınır), 0,0385 m2: 77 N, menteşe momenti 6,9 N m (limit) |
| YD-30 | hamle n 6,96 (en hafif durum) + MCP torku; bağlantı üzerindeki motor grubu 10,10 kg (büyüme payı dahil), AM x 3,871 z 0,183; I_p 0,0227 kg m2; MCP torku x 6 = 154,7 N m; statik itki 492 N |
| YD-31 | hamle n 6,96 (en hafif durum) + MCP torku (nihai); bağlantı üzerindeki motor grubu 10,10 kg (büyüme payı dahil), AM x 3,871 z 0,183; I_p 0,0227 kg m2; MCP torku x 6 = 154,7 N m; statik itki 492 N |
| YD-32 | paraşüt açılma şoku 13,1 kN tek kayış ayağında (yalnız nihai, CRASH-004); yönler: statik ayak yönü etrafında 30° koni (birleşme halkası AM'nin 1,5 m üstünde) ve düşeyden 60° geriye (silkme); belirleyici ön ayak, düşeyden 0°: Fx 0, Fy 0, Fz 13100 N |
| YD-33 | paraşüt açılma şoku 13,1 kN tek kayış ayağında (yalnız nihai, CRASH-004); yönler: statik ayak yönü etrafında 30° koni (birleşme halkası AM'nin 1,5 m üstünde) ve düşeyden 60° geriye (silkme); belirleyici arka ayak, düşeyden 37°: Fx -7876, Fy 0, Fz 10468 N |
| YD-34 | paraşüt açılma şoku 13,1 kN tek kayış ayağında (yalnız nihai, CRASH-004); yönler: statik ayak yönü etrafında 30° koni (birleşme halkası AM'nin 1,5 m üstünde) ve düşeyden 60° geriye (silkme); belirleyici ön ayak, düşeyden 60°: Fx 11345, Fy 0, Fz 6550 N |
| YD-35 | yakıt bölmesi tasarım basıncı 21,0 kPa nihai (1,5 x test pressure 14 kPa (CS-LUAS.965(b))) |
| YD-36 | E180 büyüme tareti + taşıyıcı 4,35 kg; nihai n aşağı 12,46, ileri 10,79, yan 2,21 |
| YD-37 | kapalı kapak emmesi |Cp| 1,0 x q(VD) 1990 Pa, 0,0223 m2 (üst sınır) |
| YD-38 | VD'de kapalı kapak emmesi (limit) |
| YD-39 | taşıma kızağı, orta gövde 82,3 kg (boş, dış paneller, stabilatörler ve pervane yok), AM x 2,55: kızak tepkileri 1 g'de 341 / 466 N; 3 g düşey (tasarım kararı) |
| YD-40 | dış panel 8,1 kg, 3 g düşey (tasarım kararı) |
| YD-41 | FS-GEAR'de arka gövde: arka kalemlerin n 6,96 (teçhizat n'i) ataleti + iki stabilatör paneli 472 N (aynı yön): M 1905 N m, V 2987 N (limit) |
| YD-42 | FS-FUEL'de ön gövde: ön kalemlerin ataleti, n 6,96 (teçhizat n'i, belirleyici işaret): M 1995 N m (limit) |
| YD-43 | arka gövde burulması 766 N m (iki dikey 577 N + %28 simetrik olmayan stabilatör) + yan kaplamalarda düşey kesme 2987 N (limit): q = 12533 N/m |
| YD-44 | CS-LUAS Ek H: n_j 5,40; ana bacak başına P_v 3970 N (limit), bütün ana takım durumları; kiriş ucu tepkisi 3622 N (düşey bacak yükü payı % 55 + sürükleme çifti M_y / L) |
| YD-45 | CS-LUAS Ek H: n_j 5,40; ana bacak başına P_v 3970 N (limit), bütün ana takım durumları; kiriş ucu tepkisi 3759 N (düşey bacak yükü payı % 45 + sürükleme çifti M_y / L) |
| YD-46 | CS-LUAS Ek H: n_j 5,40; ana bacak başına P_v 3970 N (limit), bütün ana takım durumları; yan yük 735 N |
| YD-47 | kanat tasarım durumu: FS-MS üzerinde gövde ataleti tepkisi 5016 N (limit, CT-ATT), taraf başına yarısı + kanat burulma çifti 403 N, gövde yanı kaburgasında 73 mm kutu derinliği |
| YD-48 | kanat tasarım durumu: FS-RS üzerinde gövde ataleti tepkisi 1926 N (limit, CT-ATT), taraf başına yarısı + kanat burulma çifti 403 N, gövde yanı kaburgasında 76 mm kutu derinliği |
| YD-49 | alt motor ayağı (y -0,11, z 0,12): en büyük düzlem dışı tepki 2197 N (nihai, 'hamle n 6,96 (en hafif durum) + MCP torku', E-FOOT), destek levhası 56 x 32 mm (c 24 mm), bant r 85 mm, kenar desteği 123 mm |
| YD-50 | üst ayak: en büyük x tepkisi 1776 N (nihai, 'hamle n 6,96 (en hafif durum) + MCP torku') |
| YD-51 | toplanmış ana bacak 4,20 kg, teçhizat yük katsayısı 6,96 (hamle matrisi): AM yarıçapı 0,159 m (0,7 x mafsal-dingil mesafesi), yukarı kilit yarıçapı 0,106 m: kanca yükü 429 N (limit) |
| YD-52 | kuyruk tamponu çarpması 45°'de 1,0 x MTOM ağırlığı (tasarım kararı, limit): omurga yangın perdesi ucunda mafsallı, FS3738'de destekli: tepki 10208 N (çarpma momenti 617 N m / 0,0604 m) |
| YD-53 | teçhizat tutma, nihai n: aşağı 12,46, yukarı 7,43, ileri 10,79, yan 2,21 |
| YD-54 | MTOM'da en büyük faydalı yük, nihai n 9,10 (uçuş, iniş, acil, paraşüt en büyüğü) |

## Boyutlandırılan ölçüler (spec.structures.sizing)

Ana kiriş başlıkları (UD MTM45-1/AS4, kat kalınlığı 0,1397 mm; gövde/eldiven genişliği 40,00 mm, dış panel 30,00 mm):

| y aralığı (m) | Kat sayısı | Kalınlık (mm) |
|---|---|---|
| 0 – 0,4 | 58 | 8,1 |
| 0,4 – 0,55 | 33 | 4,61 |
| 0,55 – 0,7 | 12 | 1,68 |
| 0,7 – 0,8 | 36 | 5,03 |
| 0,8 – 0,9 | 35 | 4,89 |
| 0,9 – 1 | 33 | 4,61 |
| 1 – 1,1 | 30 | 4,19 |
| 1,1 – 1,2 | 28 | 3,91 |
| 1,2 – 1,3 | 25 | 3,49 |
| 1,3 – 1,4 | 23 | 3,21 |
| 1,4 – 1,5 | 21 | 2,93 |
| 1,5 – 1,6 | 20 | 2,79 |
| 1,6 – 1,7 | 17 | 2,37 |
| 1,7 – 1,8 | 15 | 2,1 |
| 1,8 – 1,9 | 13 | 1,82 |
| 1,9 – 2 | 12 | 1,68 |
| 2 – 2,1 | 10 | 1,4 |
| 2,1 – 2,2 | 10 | 1,4 |
| 2,2 – 3,6 | 10 | 1,4 |

Ana kiriş gövdesi (±45 PW, dış panel):

| y aralığı (m) | Kat sayısı |
|---|---|
| 0,7 – 0,85 | 6 |
| 0,85 – 1 | 6 |
| 1 – 1,15 | 6 |
| 1,15 – 1,3 | 6 |
| 1,3 – 1,45 | 5 |
| 1,45 – 1,7 | 5 |
| 1,7 – 1,95 | 5 |
| 1,95 – 2,2 | 4 |
| 2,2 – 2,45 | 4 |
| 2,45 – 2,7 | 4 |
| 2,7 – 2,95 | 3 |
| 2,95 – 3,6 | 3 |

Yapı aşamasının sahiplendiği / revize ettiği lamine dizilimleri (spec.layups):

| Anahtar | Dış yüz | Çekirdek | İç yüz | Kullanım |
|---|---|---|---|---|
| wing_skin_primary | +-45,0/90,+-45 | core_rohacell_51wf 5 mm | +-45,+-45 | wing and LERX skins (primary sandwich, 0.6/5/0.4 mm); structures phase: +-45-dominated faces (one 0/90 ply per sandwich = 10 % 0 and 90 deg,… |
| wing_box_skin_upper | +-45,0/90,+-45 | core_rohacell_51wf 6 mm | +-45,+-45 | upper wing skin between the main- and rear-spar caps from the side-of-body rib to structures.sizing.wing.box_skin_upper_y_end_m (0.6/6/0.4 m… |
| ct_box_cover | +-45,0/90 | core_rohacell_51wf 6 mm | +-45,+-45 | carry-through box covers (upper and lower) between the spar frames FS-MS / FS-RS, y -0.40..0.40 (0.4/6/0.4 mm), bonded to the CT-box caps an… |
| fuel_floor_wellroof | +-45,0/90,0/90 | core_rohacell_71wf 5,6 mm | +-45,0/90,0/90 | aft fuel-cell floor = main-well roof M-WELLROOF (0.6/5.6/0.6 mm = 6.8 mm, the layout thickness): 3-ply faces and the denser ROHACELL 71 WF c… |

## Kütle: aşağıdan yukarı / kavramsal model payı (büyüme payı öncesi)

| Grup | Aşağıdan yukarı (kg) | Model payı (kg) | Fark (kg) |
|---|---|---|---|
| A kanat birincil yapısı (başlıklar, gövdeler, arka kiriş, dış birleşim, üst kutu kaplama çekirdeği) | 4,68 | 5,45 | -0,771 |
| B orta kutu + çatal + pimler + sürükleme pimi yuvaları | 1,98 | 1,6 | 0,384 |
| C şasi eklemeleri (kuyu tavanı dizilimi, M-WELLKEEL, orta taban takviyesi, arka omurga) | 0,773 | 0 | 0,773 |
| D stabilatör milleri, yatak yuvaları, kök yuvaları | 0,965 | 1,1 | -0,135 |
| **Toplam** | | | **0,231** (büyüme payıyla 0,242) |

Grup tavanları (mass.budget) ve sizing tahminleri (büyüme payı dahil):

| Grup | Tavan (kg) | Tahmin (kg) | Pay (kg) |
|---|---|---|---|
| chassis | 12,38 | 12,84 | -0,459 |
| controls | 8,31 | 8,29 | 0,0226 |
| fuel | 2,35 | 2,34 | 0,0085 |
| gear | 14,34 | 14,69 | -0,354 |
| hardware | 1,51 | 1,53 | -0,0201 |
| propulsion | 14,31 | 14,30 | 0,0069 |
| shell | 10,48 | 10,48 | 0,0045 |
| systems | 14,79 | 14,79 | 7,00e-04 |
| tail | 7,6 | 7,49 | 0,112 |
| wing | 15,46 | 15,71 | -0,248 |

## Arayüz kontrolleri

| No | Kontrol | Sonuç |
|---|---|---|
| I-AFTKEEL | layout M-AFTKEEL material / section = structures.sizing.body.aft_keel | uygun |
| I-WELLKEEL | layout M-WELLKEEL exists with the structures thickness | uygun |
| I-WELLROOF | layout M-WELLROOF layup = structures.sizing.fuel_bay.floor_layup | uygun |
| I-CTBOX | layout M-CTBOX section names the CT-box cover layup | uygun |
| I-MIDFLOOR | layout M-MIDFLOOR names the two bonded cut-out edge stiffeners and has the tray cut-out | uygun |
| I-NODE | layout F-SPINDLE-NODE: stub root bolts, firewall bolts, inboard bearing cylinder and outboard bearing station exist | uygun |
| I-TONGUE | layout tongue: CFRP, flange and web thickness = structures.sizing.wing_joint.tongue | uygun |
| I-FORK | layout fork: CFRP prongs and pads = structures.sizing.wing_joint.fork | uygun |
| I-SPINE | layout M-SPINE thickness = structures.sizing.parachute.spine_plies x ply | uygun |
| I-RISER | layout bridle U-lugs: shackle pin d and ear thickness = structures.sizing.parachute | uygun |
| I-UD-BEARING | materials.cfrp_ud_mtm45_as4 carries no bearing allowable (S1-05) | uygun |
| I-LAYUP-wing_skin_primary | spec.layups.wing_skin_primary = structures LAYUPS[wing_skin_primary] | uygun |
| I-LAYUP-wing_box_skin_upper | spec.layups.wing_box_skin_upper = structures LAYUPS[wing_box_skin_upper] | uygun |
| I-LAYUP-ct_box_cover | spec.layups.ct_box_cover = structures LAYUPS[ct_box_cover] | uygun |
| I-LAYUP-fuel_floor_wellroof | spec.layups.fuel_floor_wellroof = structures LAYUPS[fuel_floor_wellroof] | uygun |

## Açık konular

- çırpınma / ıraksama / kanatçık tersinmesi analiz edilmedi (kanat ve kuyruğun rijitlik / kütle modeli henüz yok): ilk uçuştan önce yer titreşim testi (GVT) ve çırpınma analizi (CS-LUAS.629); ±45 ağırlıklı kaplamalar burulma rijitliğini artırır
- metal parçaların (pimler, burçlar, arka kulak, düğüm ve takım bağlantıları, motor bağlantısı kaynakları) yorulması / hasar toleransı ve kompozit BVID/CVID kanıtı (CS-LUAS.572/573) testle; el hesapları yalnız STANAG UL13.1.2 birim şekil değiştirme sınırlarını kullanır
- kompozit dış panel birleşimi: burçların dil / kulak laminelerindeki ezilmesi, ihtiyatlı bir yerine koyma olarak QI delikli bası dayanımıyla kontrol edildi (delik e/D değeri 3'ün altında); tasarım değerleri dondurulmadan önce pimli CFRP dil / çatal eleman testi (statik, ETW, yorulma) gerekli
- tedarik gereksinimleri (MS'siz satırlar): yatak statik yük sayıları (61805-ZZ), taret asansörü bilyalı vida / ray yük sayıları, motor sönümleyici nihai kapasitesi, takım ünitesi EMA torkları ve burun aşağı kilidi - katalog değerleri araştırma verisinde yok
- ana iç takım kapakları: DA 22 kapalı kapağı VD emmesine karşı tutamaz; ölü nokta bağlantısı veya kapak kilidi gerekli (G-DOOR-LOCK)
- orta kırık bağlantısı (CT-KINK-*, başlık başına 17,3 kN, limit) ve FS3738 alt parçası (FR-3738-*, omurga tepkisi 10,2 kN, limit) el hesabı konsepti olarak boyutlandırıldı (düzeltme turu 2, VS2-03); detay çizimleri ve yük sınırlayıcı kızak seçeneği detay tasarımda kalır
- sıcak bölge: 7075 düğüm F-SPINDLE-NODE paslanmaz bir perdenin arkasında silindir kafası zarfına 10 mm mesafede; tasarım sıcaklığı bilinmiyor ve MMPDS yüksek sıcaklık eğrileri araştırma verisinde yok - T-NODE-TEMP marjların gerektirdiği dayanım oranını verir; düğüm sıcaklığı motor çalıştırmasında ölçülmeli
- yer işlemleri: burun çatalından 0,3 W ile çekme (CS-23.509 değeri; CS-LUAS / CS-VLA metni araştırma dosyalarında yok) kontrol edildi (G-TOW-*); açık havada bağlama işletme konseptiyle dışlandı, kriko noktası yok (beşik eyerleri) - işletmeci tarafından teyit edilmeli
- araştırma dosyalarında belirtimi olmayan tasarım kararları: taşıma yük katsayıları (3,0 / 1,5 / 1,5 g), kuyruk tamponu çarpma yükü (45°'de 1,0 x MTOM ağırlığı), takım işletme hızı 1,6 VS; işletmeci / test verisiyle değiştirilecek
- izin verilen değerler: çubuk parçalar (pimler, miller) için Ti-6Al-4V sac değerleri, takım bacağı boruları için 7075-T6 sac, çekirdek minimum değerleri, MTM45-1 ETW B-tabanı; sandviç kaplamaların (kıvrılma, buruşma) ve dökme insertlerin kupon / eleman testleri gerekli
