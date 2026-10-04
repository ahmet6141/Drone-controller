# YELKOVAN YK-38 — boyutlandırma ve geometri kontrolü

Bu rapor `python3 ucav/sizing.py` ile üretilir. Bütün değerler `ucav/params.py` geometrisinden yeniden hesaplanır ve `ucav/spec.yaml`'daki panel referanslarıyla karşılaştırılır. Elle düzenlemeyin.

**Sonuç: 163/163 kontrol tolerans içinde.**

Toleranslar (`spec.yaml → checks`): oranlar ve alanlar %1 (ayrıntı tasarımına bağlı olanlar %5), konumlar 3 mm, açıklıklar 4 mm, açılar 0,35°, %MAC değerleri 0,6 puan, hızlar 0,15 m/s, kütle 5 g. "Sınır" satırları bir eşiğe göre, "bilgi" satırları yalnız rapor içindir.

Eksenler: `s` burun ucundan geriye, `y` sol +, `z` FRL'den yukarı (m). Blender: X = −s.

## 1. Genel ölçüler

| Büyüklük | Hesap | Spec | Fark | Tolerans / sınır | Durum | Not |
|---|---:|---:|---:|---:|:---:|---|
| Açıklık | 3,800 m | 3,800 | 0,000 | ± 0,003 | ✓ |  |
| Toplam boy (dikey ucu firar kenarı) | 2,475 m | 2,475 | 0,000 | ± 0,003 | ✓ |  |
| Spinner dahil gövde boyu | 2,310 m | 2,310 | 0,000 | ± 0,003 | ✓ |  |
| Yükseklik (takım açık) | 0,668 m | 0,668 | 0,000 | ± 0,003 | ✓ |  |
| Yükseklik, strobe lensi dahil | 0,6726 m | 0,6725 | +0,0001 | ± 0,0030 | ✓ | dikey ucundaki strobe elipsoidinin tepesi (sahnede ölçülen en yüksek nokta) |
| Teker izi | 0,666 m | 0,666 | 0,000 | ± 0,003 | ✓ |  |
| Dingil açıklığı | 0,785 m | 0,785 | 0,000 | ± 0,003 | ✓ | statik temas noktaları arası (ana bacak 12° yatık; sıkışma aksı 1,7 mm öne alır) |

## 2. Kanat

| Büyüklük | Hesap | Spec | Fark | Tolerans / sınır | Durum | Not |
|---|---:|---:|---:|---:|:---:|---|
| Referans alan S | 0,9760 m² | 0,9760 | 0,0000 | ± 0,0098 | ✓ | trapez, gövde içi dahil |
| Açıklık oranı AR | 14,80 | 14,80 | 0,00 | ± 0,15 | ✓ |  |
| Sivrilme λ | 0,517 | 0,517 | 0,000 | ± 0,005 | ✓ |  |
| MAC | 0,2646 m | 0,2646 | 0,0000 | ± 0,0030 | ✓ |  |
| MAC y konumu | 0,866 m | 0,866 | 0,000 | ± 0,003 | ✓ |  |
| MAC hücum kenarı s | 1,1681 m | 1,1680 | +0,0001 | ± 0,0030 | ✓ |  |
| Kanat a.c. s (MAC %25) | 1,2343 m | 1,2340 | +0,0003 | ± 0,0030 | ✓ |  |
| Ana kiriş s (%28) | 1,2422 m | 1,2420 | +0,0002 | ± 0,0030 | ✓ |  |
| HK oku (dış panel) | 2,49 ° | 2,49 | 0,00 | ± 0,35 | ✓ |  |
| FK oku (dış panel) | -6,39 ° | -6,39 | 0,00 | ± 0,35 | ✓ |  |
| Uç veter düzlemi z | 0,0829 m | 0,0830 | -0,0001 | ± 0,0030 | ✓ |  |
| Glove alanı (iki yan) | 0,0137 m² | 0,0137 | 0,0000 | ± 0,0001 | ✓ |  |
| Gerçek planform alanı (glove + raked uç) | 1,0109 m² | — | — | — | bilgi | glove +0,0137, raked uç −0,0033 |
| Açıkta kalan referans alan (y ≥ 0,1025) | 0,9165 m² | — | — | — | bilgi |  |
| Raked uç veteri (y = 1,90) | 0,102 m | — | — | — | bilgi |  |

## 3. Kanat planform tablosu

| Büyüklük | Hesap | Spec | Fark | Tolerans / sınır | Durum | Not |
|---|---:|---:|---:|---:|:---:|---|
| y = 0: hücum kenarı s | 1,0375 m | 1,0375 | 0,0000 | ± 0,0030 | ✓ |  |
| y = 0: firar kenarı s | 1,4510 m | 1,4510 | 0,0000 | ± 0,0030 | ✓ |  |
| y = 0: referans veter | 0,2900 m | 0,2900 | 0,0000 | ± 0,0030 | ✓ |  |
| y = 0: veter düzlemi z (kiriş hattı) | -0,0500 m | -0,0500 | 0,0000 | ± 0,0030 | ✓ |  |
| y = 0: yerel açı (kök + burulma) | 2,500 ° | 2,500 | 0,000 | ± 0,350 | ✓ |  |
| y = 0,07: hücum kenarı s | 1,0375 m | 1,0375 | 0,0000 | ± 0,0030 | ✓ |  |
| y = 0,07: firar kenarı s | 1,4510 m | 1,4510 | 0,0000 | ± 0,0030 | ✓ |  |
| y = 0,07: referans veter | 0,2900 m | 0,2900 | 0,0000 | ± 0,0030 | ✓ |  |
| y = 0,07: veter düzlemi z (kiriş hattı) | -0,0451 m | -0,0451 | 0,0000 | ± 0,0030 | ✓ |  |
| y = 0,07: yerel açı (kök + burulma) | 2,500 ° | 2,500 | 0,000 | ± 0,350 | ✓ |  |
| y = 0,1025: hücum kenarı s | 1,0611 m | 1,0611 | 0,0000 | ± 0,0030 | ✓ |  |
| y = 0,1025: firar kenarı s | 1,4510 m | 1,4510 | 0,0000 | ± 0,0030 | ✓ |  |
| y = 0,1025: referans veter | 0,2900 m | 0,2900 | 0,0000 | ± 0,0030 | ✓ |  |
| y = 0,1025: veter düzlemi z (kiriş hattı) | -0,0428 m | -0,0428 | 0,0000 | ± 0,0030 | ✓ |  |
| y = 0,1025: yerel açı (kök + burulma) | 2,500 ° | 2,500 | 0,000 | ± 0,350 | ✓ |  |
| y = 0,24: hücum kenarı s | 1,1610 m | 1,1610 | 0,0000 | ± 0,0030 | ✓ |  |
| y = 0,24: firar kenarı s | 1,4510 m | 1,4510 | 0,0000 | ± 0,0030 | ✓ |  |
| y = 0,24: referans veter | 0,2900 m | 0,2900 | 0,0000 | ± 0,0030 | ✓ |  |
| y = 0,24: veter düzlemi z (kiriş hattı) | -0,0332 m | -0,0332 | 0,0000 | ± 0,0030 | ✓ |  |
| y = 0,24: yerel açı (kök + burulma) | 2,500 ° | 2,500 | 0,000 | ± 0,350 | ✓ |  |
| y = 0,36: hücum kenarı s | 1,1610 m | 1,1610 | 0,0000 | ± 0,0030 | ✓ |  |
| y = 0,36: firar kenarı s | 1,4510 m | 1,4510 | 0,0000 | ± 0,0030 | ✓ |  |
| y = 0,36: referans veter | 0,2900 m | 0,2900 | 0,0000 | ± 0,0030 | ✓ |  |
| y = 0,36: veter düzlemi z (kiriş hattı) | -0,0248 m | -0,0248 | 0,0000 | ± 0,0030 | ✓ |  |
| y = 0,36: yerel açı (kök + burulma) | 2,500 ° | 2,500 | 0,000 | ± 0,350 | ✓ |  |
| y = 1: hücum kenarı s | 1,1610 m | 1,1610 | 0,0000 | ± 0,0030 | ✓ |  |
| y = 1: firar kenarı s | 1,4510 m | 1,4510 | 0,0000 | ± 0,0030 | ✓ |  |
| y = 1: referans veter | 0,2900 m | 0,2900 | 0,0000 | ± 0,0030 | ✓ |  |
| y = 1: veter düzlemi z (kiriş hattı) | 0,0199 m | 0,0199 | 0,0000 | ± 0,0030 | ✓ |  |
| y = 1: yerel açı (kök + burulma) | 2,500 ° | 2,500 | 0,000 | ± 0,350 | ✓ |  |
| y = 1,83: hücum kenarı s | 1,1972 m | 1,1972 | 0,0000 | ± 0,0030 | ✓ |  |
| y = 1,83: firar kenarı s | 1,3580 m | 1,3580 | 0,0000 | ± 0,0030 | ✓ |  |
| y = 1,83: referans veter | 0,1609 m | 0,1609 | 0,0000 | ± 0,0030 | ✓ |  |
| y = 1,83: veter düzlemi z (kiriş hattı) | 0,0780 m | 0,0780 | 0,0000 | ± 0,0030 | ✓ |  |
| y = 1,83: yerel açı (kök + burulma) | -0,267 ° | -0,267 | 0,000 | ± 0,350 | ✓ |  |
| y = 1,9: hücum kenarı s | 1,2480 m | 1,2480 | 0,0000 | ± 0,0030 | ✓ |  |
| y = 1,9: firar kenarı s | 1,3502 m | 1,3502 | 0,0000 | ± 0,0030 | ✓ |  |
| y = 1,9: referans veter | 0,1500 m | 0,1500 | 0,0000 | ± 0,0030 | ✓ |  |
| y = 1,9: veter düzlemi z (kiriş hattı) | 0,0829 m | 0,0829 | 0,0000 | ± 0,0030 | ✓ |  |
| y = 1,9: yerel açı (kök + burulma) | -0,500 ° | -0,500 | 0,000 | ± 0,350 | ✓ |  |

## 4. Kumanda yüzeyleri

| Büyüklük | Hesap | Spec | Fark | Tolerans / sınır | Durum | Not |
|---|---:|---:|---:|---:|:---:|---|
| Kanatçık alanı (iki yan) | 0,0890 m² | 0,0890 | 0,0000 | ± 0,0009 | ✓ |  |
| Kanatçık veteri iç uç | 0,0768 m | 0,0770 | -0,0002 | ± 0,0030 | ✓ |  |
| Kanatçık veteri dış uç | 0,0450 m | 0,0450 | 0,0000 | ± 0,0030 | ✓ |  |
| Flap alanı (iki yan) | 0,1443 m² | 0,1440 | +0,0003 | ± 0,0072 | ✓ |  |
| Flaplı alan oranı | 0,528 | 0,528 | 0,000 | ± 0,005 | ✓ |  |
| İrtifa dümeni alanı (iki yan) | 0,0444 m² | 0,0440 | +0,0004 | ± 0,0022 | ✓ |  |
| İstikamet dümeni alanı (iki dümen) | 0,0386 m² | 0,0390 | -0,0004 | ± 0,0020 | ✓ |  |

## 5. Kuyruk

| Büyüklük | Hesap | Spec | Fark | Tolerans / sınır | Durum | Not |
|---|---:|---:|---:|---:|:---:|---|
| Yatay alan S_h | 0,1544 m² | 0,1544 | 0,0000 | ± 0,0015 | ✓ |  |
| Açıkta yatay alan S_h,açık | 0,1325 m² | 0,1335 | -0,0010 | ± 0,0067 | ✓ | gövde yarı genişliği stabilize kökünde 0,0629 m |
| Dikey alan (gerçek, iki) | 0,1216 m² | 0,1216 | 0,0000 | ± 0,0012 | ✓ |  |
| Dikey alan (izdüşüm) | 0,1189 m² | 0,1189 | 0,0000 | ± 0,0012 | ✓ |  |
| Toplam kuyruk alanı | 0,2760 m² | 0,2760 | 0,0000 | ± 0,0028 | ✓ |  |
| Kuyruk/kanat alan oranı | 0,283 | 0,283 | 0,000 | ± 0,003 | ✓ |  |
| Stabilize MAC | 0,1505 m | 0,1500 | +0,0005 | ± 0,0030 | ✓ |  |
| Dikey MAC | 0,1944 m | 0,1940 | +0,0004 | ± 0,0030 | ✓ |  |
| Stabilize AR | 7,00 | 7,00 | 0,00 | ± 0,07 | ✓ |  |
| Stabilize etkin AR (uç plakası) | 9,46 | 9,46 | 0,00 | ± 0,09 | ✓ |  |
| Stabilize a.c. s | 2,0278 m | 2,0280 | -0,0002 | ± 0,0030 | ✓ |  |
| Dikey a.c. s | 2,2829 m | 2,2830 | -0,0001 | ± 0,0030 | ✓ |  |
| Kuyruk kolu l_h (kanat a.c. → stabilize a.c.) | 0,7935 m | 0,7940 | -0,0005 | ± 0,0030 | ✓ |  |
| Dikey kolu l_v (CG → dikey a.c.) | 1,0509 m | 1,0510 | -0,0001 | ± 0,0030 | ✓ |  |
| Yatay hacim katsayısı V_h | 0,4745 | 0,4750 | -0,0005 | ± 0,0047 | ✓ |  |
| Dikey hacim katsayısı V_v | 0,0337 | 0,0337 | 0,0000 | ± 0,0003 | ✓ |  |
| Stabilize uç HK s | 2,1502 m | 2,1500 | +0,0002 | ± 0,0030 | ✓ |  |
| Stabilize uç FK s | 2,2692 m | 2,2690 | +0,0002 | ± 0,0030 | ✓ |  |
| Stabilize FK oku | 24,89 ° | 24,90 | -0,01 | ± 0,35 | ✓ |  |
| Dikey uç HK s | 2,3348 m | 2,3350 | -0,0002 | ± 0,0030 | ✓ |  |
| Dikey uç y | 0,5865 m | 0,5870 | -0,0005 | ± 0,0030 | ✓ |  |
| Dikey uç z | 0,3780 m | 0,3780 | 0,0000 | ± 0,0030 | ✓ |  |
| Dikey uçları arası | 1,173 m | 1,173 | 0,000 | ± 0,003 | ✓ |  |
| Pervane – stabilize FK aralığı (pala ucunda) | 0,1284 m | 0,1280 | +0,0004 | ± 0,0030 | ✓ | disk yarı genişliği stabilize düzleminde 0,1925 m, 5° eğim dahil |
| Pervane – stabilize FK aralığı (kökte) | 0,2177 m | 0,2180 | -0,0003 | ± 0,0030 | ✓ |  |
| Aralık / pervane çapı | 0,316 | 0,316 | 0,000 | ± 0,016 | ✓ |  |
| Pala süpürme hacmi – elevatör FK (en dar) | 0,1140 m | 0,1140 | 0,0000 | ± 0,0030 | ✓ | ≥ 0,25 D = 0,102 m şartı; pala ön yüzü düzlemin ≈ 10 mm önünde |

## 6. Kararlılık

| Büyüklük | Hesap | Spec | Fark | Tolerans / sınır | Durum | Not |
|---|---:|---:|---:|---:|:---:|---|
| Stabilize kaldırma eğimi a_h | 4,625 1/rad | 4,630 | -0,005 | ± 0,046 | ✓ | Helmbold, AR_etkin |
| Sapma gradyanı dε/dα | 0,253 | 0,252 | +0,001 | ± 0,003 | ✓ | DATCOM, l_H 0,794 m, h_H 0,115 m |
| Kuyruk terimi F_h | 0,422 | 0,422 | 0,000 | ± 0,004 | ✓ |  |
| Gövde Cmα_f | 0,691 1/rad | 0,690 | +0,001 | ± 0,007 | ✓ |  |
| Nötr nokta s | 1,2579 m | 1,2580 | -0,0001 | ± 0,0030 | ✓ |  |
| Nötr nokta (%MAC) | 33,9 % | 34,1 | -0,2 | ± 0,6 | ✓ |  |
| CG (%MAC) | 24,1 % | 24,1 | 0,0 | ± 0,6 | ✓ |  |
| Statik marj (η_h 0,90) | 9,8 %MAC | 10,0 | -0,2 | ± 0,6 | ✓ |  |
| Statik marj (η_h 0,85 / 1,00) | 8,7 %MAC | — | — | — | bilgi | η_h 1,00: 12,0 |
| Ön CG sınırı (SM %15) | 1,2182 m | 1,2190 | -0,0008 | ± 0,0030 | ✓ |  |
| Arka CG sınırı (SM %5) | 1,2447 m | 1,2450 | -0,0003 | ± 0,0030 | ✓ |  |

Yöntem (`spec.yaml → stability.method`): x_np = [a_wb·x_ac,w + F_h·x_ac,h + F_g·x_g − Cmα_f·c̄] / (a_wb + F_h + F_g). a_wb = 5,67/rad (kaldırma çizgisi, gövde etkisiyle); a_h Helmbold ile AR_etkin = AR_h·(1 + 1,9·0,6·h_dikey/b_h); dε/dα DATCOM (K_A·K_λ·K_H); F_h = η_h·a_h·S_h,açık/S·(1 − dε/dα); Cmα_f = K_f·w_f²·L_f/(c̄·S)·180/π, K_f = 0,0336/°; glove terimi F_g = 0,061 (x_g = 1,12). Duyarlılık: η_h 0,85 → SM %8,7, η_h 1,00 → SM %12,0.

## 7. Performans ve itki hattı

| Büyüklük | Hesap | Spec | Fark | Tolerans / sınır | Durum | Not |
|---|---:|---:|---:|---:|:---:|---|
| Kanat yüklemesi | 12,00 kg/m² | 12,00 | 0,00 | ± 0,12 | ✓ |  |
| Stall hızı, temiz (MTOW) | 12,00 m/s | 12,00 | 0,00 | ± 0,15 | ✓ |  |
| Stall hızı, 20° flap (MTOW) | 10,80 m/s | 10,80 | 0,00 | ± 0,15 | ✓ |  |
| Stall hızı, 20° flap (iniş kütlesi) | 10,41 m/s | 10,41 | 0,00 | ± 0,15 | ✓ |  |
| Stall hızı, 30° flap (MTOW) | 10,55 m/s | 10,55 | 0,00 | ± 0,15 | ✓ |  |
| İtki hattı z (CG istasyonunda) | 0,0418 m | 0,0420 | -0,0002 | ± 0,0030 | ✓ |  |
| İtki momenti (tam güç, burun aşağı) | 1,95 N·m | 1,95 | 0,00 | ± 0,10 | ✓ |  |
| İtki/ağırlık | 0,476 | 0,476 | 0,000 | ± 0,005 | ✓ |  |
| Lüle akış alanı (halka iç çapı – spinner) | 34,3 cm² | 34,3 | 0,0 | ± 0,3 | ✓ | soğutma havası çıkışı; NACA karın girişi ≈ 22 cm² → çıkış/giriş ≈ 1,8 (+ çene yarığı ≈ 6 cm²) |
| Lüle halka ön yüzü (dış – iç çap) | 32,0 cm² | 32,0 | 0,0 | ± 0,3 | ✓ | halka yüzü; akış alanı DEĞİL |

## 8. İniş takımı ve yer açıları

| Büyüklük | Hesap | Spec | Fark | Tolerans / sınır | Durum | Not |
|---|---:|---:|---:|---:|:---:|---|
| Ana aks s (statik) | 1,3265 m | 1,3265 | 0,0000 | ± 0,0030 | ✓ |  |
| CG yüksekliği (zeminden) | 0,2964 m | 0,2960 | +0,0004 | ± 0,0030 | ✓ |  |
| Tip-back açısı (nominal CG) | 17,69 ° | 17,70 | -0,01 | ± 0,35 | ✓ |  |
| Tip-back açısı (arka CG) | 15,38 ° | 15,40 | -0,02 | ± 0,35 | ✓ |  |
| Devrilme açısı | 47,71 ° | 47,70 | +0,01 | ± 0,35 | ✓ |  |
| Burun yükü | 12,0 % | 12,0 | 0,0 | ± 0,6 | ✓ |  |
| Statik yük, ana teker başına | 50,5 N | 50,5 | 0,0 | ± 0,5 | ✓ |  |
| Statik yük, burun | 13,8 N | 13,8 | 0,0 | ± 0,7 | ✓ |  |
| Pervane yer açıklığı | 218 mm | 218 | 0 | ± 4 | ✓ |  |
| Pervane çarpma açısı | 13,17 ° | 13,20 | -0,03 | ± 0,35 | ✓ | ≥ 13° şartı |
| Pervane açıklığı 6° burun yukarıda | 119 mm | 119 | 0 | ± 4 | ✓ |  |
| Kaporta/gövde temas açısı | 19,38 ° | 19,38 | 0,00 | ± 0,35 | ✓ | kritik nokta s = 2,178 (kaporta ağı, yanak kabartısı dahil; pabuçsuz) |
| Sürtünme pabucu temas açısı (ilk temas) | 19,26 ° | — | — | ≤ 19,33 ° | ✓ | PA-CF pabuç kaporta çene köşesinden önce değer (≥ 0,05° önde) |
| Stabilize ucu FK temas açısı | 20,66 ° | 20,66 | 0,00 | ± 0,35 | ✓ |  |
| Stabilize ucu kesiti (en kritik nokta) | 20,67 ° | — | — | — | bilgi |  |
| Dikey kökü FK temas açısı | 18,47 ° | — | — | — | bilgi | dikey kökü stabilize ucunun 0,12 m arkasına uzanır; pervane yine önce değer |
| Kanat ucu yer açıklığı | 363 mm | 363 | 0 | ± 4 | ✓ |  |
| Kanat ucuna yatış açısı | 13,04 ° | 13,10 | -0,06 | ± 0,35 | ✓ |  |
| Taret yer açıklığı | 156 mm | 156 | 0 | ± 4 | ✓ |  |
| Karın açıklığı (s 0,5–0,85) | 191 mm | 191 | 0 | ± 4 | ✓ |  |
| Katlanmış ana teker s | 1,3282 m | 1,3280 | +0,0002 | ± 0,0030 | ✓ |  |
| Katlanmış ana teker y | 0,1148 m | 0,1150 | -0,0002 | ± 0,0030 | ✓ |  |
| Katlanmış ana teker z | -0,0450 m | -0,0450 | 0,0000 | ± 0,0030 | ✓ |  |
| Katlanmış burun tekeri s | 0,726 m | 0,726 | 0,000 | ± 0,007 | ✓ | yüksüz bacak (uçuşta); statik boyla 0,620 |
| Ana bacak statik çökmesi (dikey) | 0,0080 m | 0,0080 | 0,0000 | ± 0,0030 | ✓ | zemin z'den türetilir |
| Burun bacağı statik çökmesi (dikey) | 0,0086 m | — | — | — | bilgi |  |
| Ana kuyu payı (en dar) | 3,9 mm | — | — | ≥ 2,0 mm | ✓ | ön 8,0, arka 4,5, iç 6,1, dış 3,9, ağız 5,0, tavan 11,0 |
| Burun kuyusu payı (en dar) | 5,0 mm | — | — | ≥ 2,0 mm | ✓ | ön 181,0, arka 5,0, yan 15,0, tavan 9,0, karın 7,6 |
| Ana pivot kanat altının altında | 6,1 mm | — | — | ≤ 8,9 mm | ✓ | ünite kabartması (unit_blister) bunu ve kabuk etini örtmeli |

Temas noktaları gerçek bacak geometrisinden: ana bacak pivot (1,2825; ±0,33; −0,042), 0,22 m, 12° geriye yatık, statik sıkışma bacak boyunca; burun bacağı pivot (0,42; 0; −0,056), 0,208 m. Zemin z = -0,2904. Açılar ana teker temas çizgisi etrafında burun yukarı dönüşle hesaplanır; pervane diskinin en alçak noktası 5° eğim nedeniyle göbekten geridedir.

## 9. Gövde

| Büyüklük | Hesap | Spec | Fark | Tolerans / sınır | Durum | Not |
|---|---:|---:|---:|---:|:---:|---|
| Azami genişlik | 0,2050 m | 0,2050 | 0,0000 | ± 0,0030 | ✓ |  |
| Azami yükseklik | 0,2000 m | 0,2000 | 0,0000 | ± 0,0030 | ✓ | s = 0,85'de (kabin); ters motorla kuyruk konisi 0,172'ye iner, kaporta kamburu yok |
| Islak alan | 1,214 m² | 1,210 | +0,004 | ± 0,060 | ✓ | çevre integrali |
| İncelik oranı L/((w+h)/2) | 10,89 | 10,90 | -0,01 | ± 0,55 | ✓ | eşdeğer çapla L/d_eş = 10,76 (A_max 0,0330 m², s = 0,85) |
| Karın kalkışı s 1,10→1,95 | 5,84 ° | 5,84 | 0,00 | ± 0,35 | ✓ |  |
| Karın kalkışı s 1,40→1,95 | 6,74 ° | 6,74 | 0,00 | ± 0,35 | ✓ |  |
| Burun sarkması | 0,0140 m | 0,0140 | 0,0000 | ± 0,0030 | ✓ |  |

## 10. Kütle

| Büyüklük | Hesap | Spec | Fark | Tolerans / sınır | Durum | Not |
|---|---:|---:|---:|---:|:---:|---|
| Kalemler toplamı | 11,710 kg | 11,710 | 0,000 | ± 0,005 | ✓ |  |
| Boş kütle (MTOW − yakıt − faydalı yük) | 10,067 kg | 10,067 | 0,000 | ± 0,005 | ✓ |  |
| Yakıt kütlesi (1,40 L × 0,745) | 1,043 kg | 1,043 | 0,000 | ± 0,005 | ✓ |  |
| Yapı kütlesi (basılı gövde → boya) | 5,635 kg | 5,635 | 0,000 | ± 0,005 | ✓ |  |
| Basılı kalemler toplamı ("Basılı:") | 3,457 kg | 3,457 | 0,000 | ± 0,005 | ✓ | 9 baskı grubu |
| Basılı gövde, baskı planı (out/print_report.json) | 3,457 kg | 3,457 | 0,000 | ± 0,005 | ✓ | printprep: parça hacmi × etkin yoğunluk; kalın PA-CF/PETG dilimleyici dolgusuyla |
| Baskı grubu sapması (spec ↔ print_report, en büyük) | 0,0 g | — | — | ≤ 5,0 g | ✓ | grup: Yük yolları |
| Kütle artış rezervi | 0,441 kg | 0,441 | 0,000 | ± 0,005 | ✓ |  |
| Rezerv / MTOW | 3,8 % | — | — | ≥ 3,0 % | ✓ | prototip tartımına kadar en az %3 |

## 11. Yapı ve baskı sığma

| Büyüklük | Hesap | Spec | Fark | Tolerans / sınır | Durum | Not |
|---|---:|---:|---:|---:|:---:|---|
| Merkez soket 30/27, y 0,40'ta kalınlık payı | 9,4 mm | — | — | ≥ 0,0 mm | ✓ | yerel kalınlık 40,6 mm, OD + 2 kabuk |
| Panel borusu 27/25, y 1,10'da kalınlık payı | 9,0 mm | — | — | ≥ 0,0 mm | ✓ | yerel kalınlık 37,2 mm |
| Uç borusu 16/14, y 1,70'te kalınlık payı | 0,5 mm | — | — | ≥ 0,0 mm | ✓ | yerel kalınlık 19,7 mm; ölçüt OD + 2 kabuk + 2 mm (panel: 19,2 ≤ 19,6) |
| Arka kiriş 8/6, y 1,82'de kalınlık payı | 1,2 mm | — | — | ≥ 0,0 mm | ✓ | yerel kalınlık 10,4 mm |
| Arka kiriş ↔ menteşe oyuğu (en dar) | 2,1 mm | — | — | ≥ 1,0 mm | ✓ | oyuk = menteşe − yuvarlak burun yarıçapı − 1 mm aralık |
| Stabilize kirişi 12/10, boru boyunca en dar kalınlık payı | 1,49 mm | — | — | ≥ 0,50 mm | ✓ | en dar y = 0,485; kesit kalınlığı − (OD + 2 kabuk), AERO-10 ≥ 0,5 mm |
| Stabilize kirişi deliği üstünde en ince kabuk | 0,95 mm | — | — | ≥ 0,80 mm | ✓ | delik OD + 0,4 mm; P8 ≥ 0,8 mm |
| Dikey kirişi 8/6, uçta kalınlık payı | 4,9 mm | — | — | ≥ 0,0 mm | ✓ |  |

## Notlar

- Panelden sapan ya da panelde olmayan değerler `spec.yaml`'da `# varsayım` ile işaretlidir. Bu rapordaki "bilgi" satırları tasarım kararı için girdidir, tolerans kontrolü yoktur.
- Arka kiriş %68 yerine %65'tedir: %72 menteşenin yuvarlak burun oyuğu %68'deki 8 mm boruyla 2,6 mm çakışıyordu.
- Dikeylerin kökü (0,24 m veter) stabilize ucunun 0,12 m arkasına uzanır; bu nokta stabilize ucundan önce yere değer (yukarıdaki "dikey kökü" satırı). Pervane çarpma açısı yine en küçük açıdır.
- Stabilize kalınlığı y 0,15 → 0,52 arasında %10'dan %12'ye (NACA 0012) çıkar: Ø12/10 kiriş boyunun tamamında profil içinde kalır (en dar pay ve delik üstü kabuk yukarıdaki iki satırda; AERO-10/P8).
