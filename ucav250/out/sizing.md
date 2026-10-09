# YK-250 HANÇER — boyutlandırma raporu (otomatik)

Kaynak: `ucav250/spec.yaml` (rev. v1.6, 2026-10-09); üreten: `python3 -m ucav250.analysis.sizing`. Bu dosya elle düzenlenmez.

> Kapsam: yalnızca sivil EO/IR gözetleme ve araştırma. Platform silah (weapon), mühimmat (munition), sert bağlantı noktası (hardpoint), pilon (pylon) veya yük bırakma (release) mekanizması içermez ve bunlar için tasarlanmamıştır; görünüm dili yalnızca biçimdir.

**Gereksinimler:** 61/61 karşılanıyor. 

## 1. Özet

| Büyüklük | Değer |
|---|---|
| MTOM / boş / yakıt / görev faydalı yükü | 149,9 / 101,6 / 30,3 / 18,0 kg |
| Azami faydalı yük (R-02b) | 20,0 kg: dayanım 9,54 h (yakıt 28,3 kg) |
| Kanat açıklığı / alan / AR | 7,20 m / 3,237 m² / 16,0 |
| Gövde boyu / genişlik / yükseklik | 4,00 / 0,80 / 0,46 m |
| Dayanım (tasarım görevi, görev yüküyle) | 10,32 h (bekleme 8,01 h) |
| Menzil (feribot, yedek dahil) | 1.148 km |
| Görev beklemesi başı / sonu (3000 m) | 32,44 m/s TAS (27,95 EAS), 4.851 rpm, 2,55 kg/h / 32,35 m/s TAS (27,87 EAS), 4.747 rpm, 2,39 kg/h |
| MTOM'da 3000 m bekleme noktası (karşılaştırma) | 33,0 m/s TAS (28,5 EAS), CL 0,92, L/D 16,2, 2,65 kg/h |
| CD0 temiz / bekleme (taret dışarıda) / takım açık | 0,0350 / 0,0369 / 0,0474 |
| (L/D)maks temiz / bekleme | 17,3 / 16,8 |
| CLmax kanat / trimli temiz | 1,331 / 1,293 |
| Tutunma hızı (temiz, MTOM, DS) | 23,95 m/s |
| Tırmanma DS / 3000 m | 4,22 / 2,39 m/s |
| Servis tavanı | 7.114 m |
| Kalkış / iniş koşusu (DS) | 178 / 183 m |
| Statik marj aralığı | %10,1 – %14,5 OAK |
| Cnβ | 0,0585 1/rad |

## 2. Gereksinim uyumu

| No | Gereksinim | Ölçüt | Hedef | Sonuç | Durum |
|---|---|---|---|---|---|
| R-01 | Azami kalkış kütlesi 150 kg'ın altında (SHT-İHA M2 sınıfı, 149,9 kg tavan) | `mtow_kg` | <= 149,900 kg | 149,900 | ✔ |
| R-02 | Tasarım görevi dayanımı ≥ 10 h, tasarım görevi faydalı yüküyle (mission.payload_design_kg: MTOM 149,9 kg'da aynı görevde ≥ 10,25 h veren, 0,5 kg'a aşağı yuvarlanmış en büyük faydalı yük; tırmanma, 2 x 100 km geçiş, 3000 m'de taret dışarıda bekleme, alçalma; %10 yedek ayrıca) | `endurance_h` | >= 10,000 h | 10,318 | ✔ |
| R-02b | Azami faydalı yükle (20 kg) aynı tasarım görevinde dayanım ≥ 9,5 h (MTOM 149,9 kg; yakıt = MTOM − boş kütle − 20 kg; azami yükün kendi ağırlık merkezi) | `endurance_max_payload_h` | >= 9,500 h | 9,543 | ✔ |
| R-03 | Azami faydalı yük ≥ 20 kg: MTOM'daki yükleme durumlarından biri 20 kg faydalı yük taşır ve bütün kontroller (ağırlık merkezi zarfı, statik marj, kalkış, R-02b) bu yükle de yapılır | `payload_max_checked_kg` | >= 20,000 kg | 20,000 | ✔ |
| R-04 | Servis tavanı (0,5 m/s) ≥ 4500 m, MTOM | `ceiling_service_m` | >= 4.500,000 m | 7.114,455 | ✔ |
| R-05 | Deniz seviyesi tırmanma hızı ≥ 4,0 m/s, MTOM, tam gaz (4,9 m/s karşılaştırma hedefi pervane sınırlı; bkz. pervane ödünleşimi) | `roc_sl_m_s` | >= 4,000 m/s | 4,218 | ✔ |
| R-06 | Kalkış yer koşusu ≤ 200 m (MTOM, ISA deniz seviyesi; 300 m pist / 1,5) | `takeoff_ground_roll_m` | <= 200,000 m | 178,168 | ✔ |
| R-07 | İniş yer koşusu ≤ 200 m (görev sonu kütlesi, ISA deniz seviyesi) | `landing_ground_roll_m` | <= 200,000 m | 182,878 | ✔ |
| R-08 | Tutunma hızı ≤ 24 m/s (temiz, trimli, MTOM, deniz seviyesi) | `vs_clean_sl_mtow_m_s` | <= 24,000 m/s | 23,947 | ✔ |
| R-09 | Statik marj ≥ %10 OAK (tüm yükleme durumları, öndeki nötr nokta) | `static_margin_min` | >= 0,100 - | 0,101 | ✔ |
| R-10 | Statik marj ≤ %30 OAK (tüm yükleme durumları) | `static_margin_max` | <= 0,300 - | 0,145 | ✔ |
| R-11 | Yön kararlılığı Cnβ ≥ 0,057 1/rad (0,001 1/derece) | `cn_beta_per_rad` | >= 0,057 1/rad | 0,059 | ✔ |
| R-12 | Geri devrilme açısı ≥ 15° (en arka zemin ağırlık merkezi) | `tipback_deg` | >= 15,000 deg | 38,633 | ✔ |
| R-13 | Yana devrilme açısı ≤ 55° | `turnover_deg` | <= 55,000 deg | 54,000 | ✔ |
| R-14 | Burun tekerleği yükü ≥ %8 (en arka ağırlık merkezi) | `nose_load_aft_cg` | >= 0,080 - | 0,163 | ✔ |
| R-15 | Burun tekerleği yükü ≤ %20 (en ön ağırlık merkezi) | `nose_load_fwd_cg` | <= 0,200 - | 0,172 | ✔ |
| R-16 | Pervane yer açıklığı ≥ 0,18 m: statik tutum, kalkış (yerden kesilme) tutumu ve teker koyma tutumunun en kritiği (MTOM; kalkışta amortisör statik çökmede, teker koymada yüksüz) | `prop_clear_min_925a_m` | >= 0,180 m | 0,184 | ✔ |
| R-17 | Sönük ana lastik + dibe oturmuş amortisörde pozitif pervane açıklığı (≥ 0,02 m) | `prop_clear_flat_tyre_m` | >= 0,020 m | 0,058 | ✔ |
| R-18 | Kuyruk tamponu pervaneden önce yere değer (açı farkı ≥ 1°) | `bumper_before_prop_margin_deg` | >= 1,000 deg | 3,898 | ✔ |
| R-19 | Kalkış ve flare açısında kuyruk tamponu yere değmez (pay ≥ 2°) | `bumper_rotation_margin_deg` | >= 2,000 deg | 2,300 | ✔ |
| R-20 | Teker koyma açısı ≥ 3° (ana tekerler önce değer) | `touchdown_attitude_deg` | >= 3,000 deg | 4,869 | ✔ |
| R-21 | Ana iniş takımı gövde içine toplanır (teker, bacak, mafsal; kanat kutusu ile çakışma yok) | `main_gear_stowed` | == 1,000 - | 1,000 | ✔ |
| R-22 | Burun iniş takımı omurga yuvasına toplanır | `nose_gear_stowed` | == 1,000 - | 1,000 | ✔ |
| R-23 | Taret içeride kapakların üstünde kalır (gömülü) | `turret_flush_margin_m` | >= 0,000 m | 0,034 | ✔ |
| R-24 | E180 büyüme zarfı (0,18 x 0,23 m) taret bölmesine sığar | `turret_growth_inside` | == 1,000 - | 1,000 | ✔ |
| R-25 | Taret dışarıda: nadirden en az −5° yükselime kadar her azimutta engelsiz görüş | `turret_fov_upper_min_deg` | >= -5,000 deg | -3,000 | ✔ |
| R-26 | Yerleşim bölgeleri gövdeye sığar ve bildirilen içerikleri (aviyonik/güç bölmeleri: gerçek bileşen zarfları + bağlayıcı payı) kendi bölgelerine, gövdeye ve birbirlerine açıklıklarıyla sığar (hata sayısı 0) | `packaging_zones_failed` | == 0,000 - | 0,000 | ✔ |
| R-27 | Yerleşim bölgeleri ve yakıt hücreleri çakışmaz (çakışma sayısı 0) | `zone_overlaps` | == 0,000 - | 0,000 | ✔ |
| R-28 | Yakıt hacmi gereken hacmi karşılar (pay ≥ 0) | `fuel_volume_margin` | >= 0,000 - | 0,184 | ✔ |
| R-29 | Taşıma: çıkarılabilir dış kanat paneli ≤ 3,4 m | `outer_panel_length_m` | <= 3,400 m | 2,900 | ✔ |
| R-30 | Gövde + LERX orta kesiti ≤ 2,0 m (tek parça taşıma) | `centre_section_width_m` | <= 2,000 m | 1,400 | ✔ |
| R-31 | Statik pervane uç Mach sayısı ≤ 0,75 | `prop_tip_mach_static` | <= 0,750 - | 0,712 | ✔ |
| R-32 | Jeneratör çıkışı, beklemenin en düşük devrinde, en büyük sürekli elektrik yükünün ≥ 1,2 katı (E180 büyüme tareti ve araştırma yükü güç payı dahil) | `generator_margin_loiter` | >= 1,200 - | 1,276 | ✔ |
| R-33 | Pervane uçları ile yapı arasında radyal açıklık ≥ 0,026 m (CS-VLA 925(c)(1)) | `prop_clear_radial_m` | >= 0,026 m | 0,031 | ✔ |
| R-34 | Pervane palaları ile sabit yapı (kaporta, kuyruk yüzeyleri) arasında boyuna açıklık ≥ 0,013 m (CS-VLA 925(c)(2)) | `prop_clear_longitudinal_m` | >= 0,013 m | 0,029 | ✔ |
| R-35 | Kalkışta dönme yetkisi: stabilatörler burun tekerini ana tekerler hâlâ yüklüyken kaldırır (tekerlek arabası etkisi yok; bütün MTOM yükleme durumları) | `takeoff_main_gear_load_at_rotation_N` | > 0,000 N | 111,688 | ✔ |
| R-36 | Stabilatör trim gereksinimi (yerel CL, η_t dahil; temiz/kalkış CLmax ve tam güçte yerden kesilme/pas geçme, en ön ağırlık merkezleri; yerden kesilmede takım açık) ≤ 0,8 × stabilatör CLmax | `stab_trim_cl_local_max` | <= 0,720 - | 0,693 | ✔ |
| R-37 | Stabilatör eyleyicisi: tepe tork × dört çubuk bağlantısının o sapmadaki tork oranı ≥ 1,25 × en büyük menteşe momenti, bütün sapma aralığında (CS-LUAS.395(a)(1)) | `stab_hinge_peak_margin` | >= 1,000 - | 1,885 | ✔ |
| R-38 | Stabilatör kök boşluğu ≤ 10 mm (sabit kök parçasına karşı, bütün sapma aralığında sabit) | `stab_root_gap_m` | <= 0,010 m | 0,008 | ✔ |
| R-39 | Stabilatör kökü bütün sapma aralığında gövdeye girmez (gövde yarı genişliği ile pay ≥ 5 mm) | `stab_root_body_clearance_m` | >= 0,005 m | 0,022 | ✔ |
| R-40 | Dikey kuyruk ve ventral kanatçık kökleri bütün kök veteri boyunca gövdeye gömülü (boşluk ≤ 0) | `tail_root_max_gap_m` | <= 0,000 m | -0,015 | ✔ |
| R-41 | Pervane koruması (üst-yan): eğik dikeylerin firar kenarı, itki hattı açısıyla eğik pervane disk düzlemini uç çemberinin ≥ 26 mm dışında keser | `prop_guard_fin_crossing_margin_m` | >= 0,026 m | 0,075 | ✔ |
| R-42 | Pervane koruması (üst-yan): eğik dikeylerin firar kenarı disk düzlemini uç çemberinin ≤ 80 mm dışında keser (diski yakından çevreler) | `prop_guard_fin_crossing_margin_m` | <= 0,080 m | 0,075 | ✔ |
| R-43 | Ana kiriş hattında OML derinliği (gövde yanından dış panel bağlantısına) ≥ birleşim kalınlığının %95'i | `spar_depth_main_ratio` | >= 0,950 - | 0,977 | ✔ |
| R-44 | Arka kiriş hattında OML derinliği (gövde yanından dış panel bağlantısına) ≥ dış panel bağlantı kesitinin arka kiriş derinliğinin %95'i (arka kiriş kademesiz devam eder) | `spar_depth_rear_ratio` | >= 0,950 - | 1,002 | ✔ |
| R-45 | Kanat kökü ile köşe çizgisi arasında basamak ≤ 10 mm (LERX/köşe çizgisi geçişi) | `wing_root_chine_step_m` | <= 0,010 m | 0,000 | ✔ |
| R-46 | Flap, sökülebilir dış panelin üzerinde kalır (iç ucu panel bağlantısının dışında) | `flap_inboard_end_outboard_of_joint_m` | >= 0,000 m | 0,030 | ✔ |
| R-47 | Stabilatör eyleyicisi: anma (sürekli) torku × dört çubuk bağlantısının o sapmadaki tork oranı en büyük menteşe momentinin ≥ 1,1 katı, bütün sapma aralığında (mil, girdap kafesi AM bandının ön ucunda) | `stab_hinge_rated_margin_max` | >= 1,100 - | 1,178 | ✔ |
| R-48 | Pervane koruması (alt): ventral kanatçığın firar kenarı eğik disk düzlemini uç çemberinin ≥ 26 mm dışında keser; değiştirilebilir tampon kızağı diskin altındadır | `prop_guard_ventral_margin_m` | >= 0,026 m | 0,050 | ✔ |
| R-49 | Pervane koruması (alt): ventral kanatçığın firar kenarı disk düzlemini uç çemberinin ≤ 120 mm dışında keser (alt bölgeyi yakından korur) | `prop_guard_ventral_margin_m` | <= 0,120 m | 0,050 | ✔ |
| R-50 | Pervane düzlemi kaporta firar kenarının ≥ 0,10 D gerisinde (itici pervane gövde izinde; kaporta tabanı küt olduğundan kurulum katsayısı k_inst için 0,93 tahmini kullanılır, duyarlılık doc 02 §6.5) | `prop_plane_behind_cowl_over_D` | >= 0,100 - | 0,102 | ✔ |
| R-51 | Yanal koruma: uçak kanat ucu yere değene kadar yatarsa pervane ucu yerden ≥ 0,05 m yukarıda kalır (kanat ucu önce değer) | `prop_clear_wingtip_on_ground_m` | >= 0,050 m | 0,320 | ✔ |
| R-52 | E180 büyüme taretinin tepe elektrik yükü (+ temel yük + araştırma yükü payı) tasarım görevinin beklemesi boyunca kesintisiz karşılanır (v1.2 yeteneği): jeneratörün DC çıkışı (güç elektroniği verimi dahil) ile tampon bataryanın tepe destek payı birlikte; tepe yükün bütün bekleme boyunca sürdüğü varsayılır (görev çevrimi kredisi yok); tepe destek payı = kullanılabilir batarya enerjisi − jeneratör kaybı/marş yedeği (400 W × 28 dk); E180 büyüme görevi de aynı kuralla kontrol edilir | `e180_peak_support_margin` | >= 1,000 - | 1,010 | ✔ |
| R-53 | Kuyruk kök yapıları (gövde dış yüzeyinde budanmış dikey, sabit stabilatör kök parçası ve ventral kökleri: kök kaburgası + bağlantı bandı) ve stabilatör milleri iç yerleşim bölgeleriyle ve birbirleriyle çakışmaz (çakışma sayısı 0) | `tail_root_interferences` | == 0,000 - | 0,000 | ✔ |
| R-54 | İşletme sınırları (CS-LUAS 1505): VNE = 0,9 VD, VNO = min(VC; 0,89 VNE); uçuş kontrol sisteminin zarf koruması hızı VNO'nun altında tutar (sınır hız ≤ VNO; tam güçte düz uçuş hızı VNE'yi aşar) | `fcs_speed_limit_margin_m_s` | >= 0,000 m/s | 0,000 | ✔ |
| R-55 | MTOM'da acil/geri dönüş inişi (flapsız, ISA deniz seviyesi) yer koşusu ≤ 300 m pist uzunluğu (anormal durum: 1,5 alan katsayısı uygulanmaz; normal iniş R-07 ve en büyük iniş kütlesi işletme sınırıdır) | `landing_ground_roll_mtow_m` | <= 300,000 m | 219,569 | ✔ |
| R-56 | Boş kütle bütçesi (mass.budget, grup tavanları) ve yedek payı, R-02'yi (tasarım görevi faydalı yükü, 10 h) ve R-02b'yi (20 kg, 9,5 h) tam karşılayan iki boş kütlenin küçüğünü aşmaz (pay ≥ 0); her grubun tahmini kendi tavanının altında kalır | `mass_budget_margin_kg` | >= 0,000 kg | 0,013 | ✔ |
| R-57 | Kanatçık, flap ve dümen eyleyicileri: anma torku × dört çubuk bağlantısının o sapmadaki tork oranı ≥ 1,1 × menteşe momenti, bütün sapma aralığında (kanatçık ve dümen VA'da tam sapma ve VD'de 1/3 sapma, flap VF'de; R-47 kuralı) | `control_actuator_rated_margin_min` | >= 1,100 - | 1,286 | ✔ |
| R-58 | Kanatçık, flap ve dümen eyleyicileri: tepe tork × dört çubuk bağlantısının o sapmadaki tork oranı ≥ 1,25 × menteşe momenti, bütün sapma aralığında (CS-LUAS.395(a)(1)) | `control_actuator_peak_margin_min` | >= 1,000 - | 1,905 | ✔ |
| R-59 | Bütün kumanda yüzeylerinde (kanatçık, flap, dümen, stabilatör) dört çubuk bağlantısı istenen sapma aralığına eyleyicinin hareket sınırı içinde ulaşır (pay ≥ 0°) | `control_linkage_travel_margin_deg` | >= 0,000 deg | 6,773 | ✔ |
| R-60 | Kalkış dönüşü (uçuş kontrol sistemi dönüş yasası): yunuslama hızı komutu sıfırdan komut hızına rampalanır ve tutulur; burun tekeri yerden kesikken stabilatörün ana teker temas noktasına göre (I_yy + m a²) × açısal ivme momenti için gereken aşağı kuvveti her adımda azami aşağı kuvveti aşmaz (pay kırpılmadan ölçülür, ≥ 0; bütün MTOM yükleme durumları); yerden kesilmede ana teker tepkisi sıfırdır ve aşağı kuvvet havadaki değere (sabit hızda güç-açık trim) eşittir; moment artığı ya da bu süreklilik 1e-6'yı aşarsa ölçüt negatif olur | `takeoff_rotation_download_margin_min_N` | >= 0,000 N | 0,564 | ✔ |

### Gereksinim kararı: görev faydalı yükü kuralı

Kural: 0,5 kg'lık ızgarada (aşağı yuvarlanmış), dayanımı ≥ 10,25 h (10,00 h + 0,25 h sağlamlık payı) olan en büyük faydalı yük; azami yükü aşmaz. Spec değeri 18,0 kg; kapalı tasarımdan türetilen 18,0 kg (10,318 h; bir adım fazlası 18,5 kg ile 10,124 h). Azami yük 20,0 kg ile dayanım 9,543 h (R-02b ≥ 9,5 h).

Kural eşiğine pay (V4-03): görev yüküyle dayanım eşiğin 0,068 h üstündedir. Bu, sabit MTOM'da boş kütlenin 0,175 kg artmasına denktir (her kg boş kütle bir kg yakıttır); faydalı yük olarak eşik ≈ 18,17 kg'dır (ızgara noktaları arasında doğrusal). Boş kütle bundan fazla artarsa kural bir alt adımı verir.

Kütle bütçesi (R-56): R-02'yi tam karşılayan boş kütle 102,449 kg, R-02b'yi tam karşılayan 101,743 kg; belirleyen R-02b; tavanlar 101,68 kg + yedek 0,05 kg → pay 0,013 kg (R-02'ye göre 0,719 kg). Bu paylar tavan toplamı + yedekten ölçülür.

Aynı başvuru kütlesinden (bugünkü boş kütle tahmini 101,630 kg) ölçülen üç boş kütle payı (V5-05): görev yükü kuralı eşiği +0,175 kg, R-02b +0,113 kg, R-02 +0,819 kg.

## 3. Kütle

| Grup | Kütle (kg) | Bütçe hedefi ± tolerans |
|---|---|---|
| wing | 15,34 | 15,34 ± 0,46 |
| systems | 14,79 | 14,79 ± 0,44 |
| gear | 14,57 | 14,57 ± 0,44 |
| propulsion | 14,53 | 14,54 ± 0,44 |
| chassis | 12,46 | 12,46 ± 0,37 |
| shell | 10,48 | 10,48 ± 0,31 |
| controls | 8,28 | 8,29 ± 0,25 |
| tail | 7,34 | 7,35 ± 0,22 |
| fuel | 2,34 | 2,35 ± 0,07 |
| hardware | 1,50 | 1,51 ± 0,05 |
| **boş (büyüme payı %5 dahil)** | **101,63** | |

Yükleme durumları (ağırlık merkezi, uçuş, takım içeride; taret durumu yükleme adında):

| Durum | Kütle (kg) | x_AM (m) | z_AM (m) | SM (% OAK) |
|---|---|---|---|---|
| mtow_design_payload_turret_retracted | 149,90 | 2,605 | 0,032 | 10,1 |
| mtow_design_payload_turret_extended | 149,90 | 2,605 | 0,031 | 10,1 |
| mtow_max_payload_turret_retracted | 149,90 | 2,606 | 0,030 | 10,1 |
| mtow_max_payload_turret_extended | 149,90 | 2,606 | 0,028 | 10,1 |
| full_fuel_baseline_sensors_only | 140,73 | 2,603 | 0,051 | 10,5 |
| zero_fuel_design_payload | 119,63 | 2,603 | 0,021 | 10,6 |
| zero_fuel_max_payload | 121,63 | 2,603 | 0,018 | 10,6 |
| reserve_fuel_design_payload | 123,26 | 2,603 | 0,024 | 10,6 |
| minimum_flying_turret_only | 106,41 | 2,605 | 0,046 | 10,1 |
| e180_growth_turret_full_fuel | 149,90 | 2,584 | 0,029 | 14,5 |

## 4. Aerodinamik

* Kaldırma eğimi (girdap kafesi, gövde planı hariç): 5,696 1/rad; taşıma hattı 6,153 1/rad. CL0 = 0,494.
* Açıklık verimi: taşıma hattı 0,965, Trefftz 0,979; polar uyumu e = 0,838 (profil sürüklemesinin CL ile değişimi ve trim dahil).
* Kritik kesit: η = 0,42 (LERX dışı), α_stall = 8,6°; Re kök/uç 1.95e+06 / 3.78e+05.

Sürükleme kalemleri (temiz, CD = D/q / S_ref):

| Kalem | CD |
|---|---|
| body | 0,00760 |
| aft_body_upsweep | 0,00118 |
| tail_stabilator | 0,00231 |
| tail_stabilator_stub | 0,00038 |
| tail_fin | 0,00219 |
| tail_ventral | 0,00016 |
| landing_gear | 0,00040 |
| eo_ir_turret | 0,00009 |
| engine_cooling | 0,00186 |
| aft_closure_base | 0,00216 |
| wing_body_junctions | 0,00024 |
| tail_body_junctions | 0,00057 |
| antennas_pitot_lights_vents | 0,00077 |
| control_surface_gaps | 0,00080 |
| leakage_protuberance | 0,00104 |
| kanat profil sürüklemesi (CL 0,7; geçiş zorlanmış ×1,15) | 0,01336 |

Bekleme durumunda taret dışarıda: +0,00166 CD.

## 5. Kararlılık

* Nötr nokta: klasik 2,665 m, girdap kafesi 2,654 m → kullanılan (öndeki) 2,654 m.
* Taşıyıcı gövde (Multhopp) Cmα = 0,817 1/rad (ön gövde 0,673, arka 0,144); kanat-gövde AM 2,573 m, Cm0_wb -0,123.
* Kuyruk hacmi V_H 0,437, V_V 0,0274; aşağı sapma dε/dα 0,385.
* Cnβ: eğik dikeyler 0,0943, ventral 0,0029, gövde -0,0387 → toplam 0,0585 1/rad. Clβ -0,1127 1/rad.

## 6. İniş takımı ve taret

* Zemin z = -0,430 m; dingil açıklığı 2,296 m; iz 0,836 m; geri devrilme 38,6°; yana devrilme 54,0°; burun yükü %16,3 – %17,2.
* Statik zemin tutumu 2,5° burun yukarı (burun tekeri teması ana tekerlerin 0,100 m altında, gövde ekseninde).
* Pervane yer açıklığı (CS-VLA 925(a), MTOM): statik 0,208 m, yerden kesilme tutumu 3,7° (amortisör statik) 0,184 m, teker koyma tutumu 4,9° (takım yüksüz, +0,042 m) 0,203 m → en az 0,184 m; sönük lastik + dibe oturmuş amortisör 0,058 m. Pervane temas açısı 13,1°, kuyruk tamponu temas açısı 9,2°; flare 6,9°.
* CS-VLA 925(c): radyal açıklık en az 0,031 m, boyuna açıklık en az 0,029 m (pala ekseni boyutu uç 0,009 m; parçalara göre: body 0,042, stabilator 0,100, stabilator_stub 0,098, fin 0,029, ventral 0,049). Pervane düzlemi kaporta firar kenarının 80 mm (0,102 D) gerisinde.
* Pervane koruması (itki hattı açısıyla eğik disk düzleminde): stabilatör 90° (tepeden), uç çemberinin 458 mm dışı; stabilatör -90° (tepeden), uç çemberinin 458 mm dışı; eğik dikey 36° (tepeden), uç çemberinin 75 mm dışı; eğik dikey -36° (tepeden), uç çemberinin 75 mm dışı; ventral kanatçık 180° (tepeden), uç çemberinin 50 mm dışı. Uç çemberinin 0,30 m içinde hiçbir yapının bulunmadığı açık bölge payı %83; disk açıktır, personel koruması iddia edilmez. Kanat ucu yere değene kadar yatışta (10,2°) pervane ucu yerden 0,320 m yukarıda kalır.
* Toplanmış takım: ana teker zarfı içeride evet, bacak evet, mafsal evet, merkez boşluğu 0,038 m, kanat kutusu açıklığı 0,034 m; burun takımı evet.
* Taret: içeride top kapak üstünde 0,034 m; E180 büyüme zarfı içeride evet; strok 0,128 m; görüş alanı üst sınırı en az -3,0° (ön ±60°: -3,0°).

## 7. Performans

| Durum | Değer |
|---|---|
| Kalkış (DS, MTOM, belirleyici yükleme: E180 taret, tam yakıt) | koşu 178 m, 15 m'ye 325 m; burun kaldırma V_R 24,7 m/s (ana tekerlerde 112 N), V_LOF 25,1 m/s, yerden kesilme tutumu 2,6° (zemin tutumu 2,5°, dönüş 0,30 s) (VS_TO 22,0 m/s, flap 35°) |
| — uçuş kontrol sistemi stabilatör programı | 23,5 m/s'den itibaren ana tekerleri yüklü tutan en küçük aşağı kuvvet, V_R'de tam burun yukarı (yerel CL 0,90); dönüşte sabit yunuslama hızı (1,04°/s), aşağı kuvvet ana teker temas noktasına göre moment dengesinden: 184 N'dan yerden kesilmede güç-açık trim değerine 143 N (trim 143 N); en büyük aşağı kuvvete en küçük pay 1,1 N, moment artığı en çok 8.7e-14 N·m; 24,2 m/s (1,1 VS_TO) altında yerden kesilme yok |
| — yunuslama ataleti (V4-05) | I_yy 114,2 kg·m² (kütle kalemlerinden; deri, kanat ve kuyruk yüzeylerine yayılı, uzun kalemler çubuk); yunuslama hızı komutu 0,3 s'de rampalanır (3,48°/s², V_R'de ana teker noktasına göre 8,2 N·m). V_R 24,58 → 24,74 m/s, koşu 175,2 → 178,2 m (+3,0 m); 0,1 s rampayla 179,7 m |
| — MTOM, görev yükü | koşu 171 m, V_R 24,1, V_LOF 24,7 m/s, θ_LOF 2,8°, x_AM 2,601 m, aşağı kuvvet payı 1,6 N |
| — MTOM, görev yükü, taret dışarıda | koşu 171 m, V_R 24,1, V_LOF 24,7 m/s, θ_LOF 2,8°, x_AM 2,601 m, aşağı kuvvet payı 0,8 N |
| — MTOM, azami yük | koşu 171 m, V_R 24,1, V_LOF 24,7 m/s, θ_LOF 2,8°, x_AM 2,602 m, aşağı kuvvet payı 0,6 N |
| — MTOM, azami yük, taret dışarıda | koşu 171 m, V_R 24,1, V_LOF 24,7 m/s, θ_LOF 2,8°, x_AM 2,602 m, aşağı kuvvet payı 1,5 N |
| — E180 taret, tam yakıt | koşu 178 m, V_R 24,7, V_LOF 25,1 m/s, θ_LOF 2,6°, x_AM 2,580 m, aşağı kuvvet payı 1,1 N |
| Kalkış (1500 m ISA) | koşu 275 m |
| İniş (görev sonu 123,3 kg) | koşu 183 m, V_TD 25,0 m/s |
| İniş (MTOM, acil dönüş, flapsız; R-55) | koşu 220 m, V_TD 27,5 m/s; 35° flapla 248 m |
| 0 m | VS 23,9 m/s, Vmaks 54,0 m/s, tırmanma 4,22 m/s @ 29,4 m/s |
| 3000 m | VS 27,8 m/s, Vmaks 51,3 m/s, tırmanma 2,39 m/s @ 32,0 m/s |
| Tavan | servis 7.114 m, mutlak 8.357 m |

Faydalı yük – dayanım (MTOM 149,9 kg; yakıt = MTOM − boş − yük, yakıt hacmiyle sınırlı). Tablo izin verilen en hafif yüklemeden, temel EO/IR setinden başlar (V4-02):

| Yük (kg) | Yakıt (kg) | Kalkış kütlesi (kg) | Dayanım (h) | Bekleme (h) | SM aralığı (% OAK) | E180 tepesi bütün bekleme sürseydi batarya (Wh) | E180 tepesinin desteklendiği bekleme (h) |
|---|---|---|---|---|---|---|---|
| 3,25 | 35,85 (hacim sınırı) | 140,7 | 12,91 | 10,56 | 10,5–11,4 | – (E180 seti sığmaz) | – |
| 5,00 | 35,85 (hacim sınırı) | 142,5 | 12,84 | 10,49 | 10,5–11,3 | – (E180 seti sığmaz) | – |
| 7,50 | 35,85 (hacim sınırı) | 145,0 | 12,74 | 10,39 | 10,4–11,2 | 175,9 > pay | 7,37 |
| 10,00 | 35,85 (hacim sınırı) | 147,5 | 12,61 | 10,29 | 10,3–11,0 | 169,7 > pay | 7,63 |
| 12,00 | 35,85 (hacim sınırı) | 149,5 | 12,51 | 10,20 | 10,2–10,9 | 161,9 > pay | 8,00 |
| 12,42 | 35,85 (hacim sınırı) | 149,9 | 12,49 | 10,18 | 10,2–10,9 | 159,9 > pay | 8,10 |
| 12,50 | 35,77 | 149,9 | 12,46 | 10,14 | 10,2–10,9 | 159,4 > pay | 8,10 |
| 15,00 | 33,27 | 149,9 | 11,48 | 9,17 | 10,2–10,8 | 142,9 > pay | 8,09 |
| 17,50 | 30,77 | 149,9 | 10,51 | 8,20 | 10,1–10,7 | 126,5 > pay | 8,08 |
| 18,00 | 30,27 | 149,9 | 10,32 | 8,01 | 10,1–10,6 | 123,3 | bütün bekleme |
| 18,50 | 29,77 | 149,9 | 10,12 | 7,82 | 10,1–10,6 | 120,0 | bütün bekleme |
| 20,00 | 28,27 | 149,9 | 9,54 | 7,24 | 10,1–10,6 | 110,3 | bütün bekleme |

Tepe destek payı 124,5 Wh. '> pay' satırlarında E180 tepesi bütün bekleme boyunca karşılanamaz (R-52 işletme sınırı, bkz. §8). E180 sütunları yalnız E180 setini (5,5 kg: E180 4,0 + görev bilgisayarı 1,0 + tepsi 0,5) taşıyabilen yükler içindir.

İzin verilmeyen yüklemeler (R-09, SM ≥ %10): faydalı yüksüz (taret de yok) SM %5,2–%5,9 aralığındadır; uçmak için taret bağlantısına (x = 1,220 m) 1,95 kg safra gerekir. Yalnız taretle (görev bilgisayarı ve tepsi yok) dolu depolarda SM %9,6 olur; bu yüklemede yakıt 7,3 kg ile sınırlıdır.

Duyarlılıklar (görev yüküyle dayanım, h): BSFC −%12 11,75, BSFC +%12 9,19, bütün sürükleme +%10 9,77, boş kütle +%5 8,36, bekleme 1000 m'de 9,75, geçişsiz 10,27, taret hep içeride 10,45, k_inst 0,90 10,13, k_inst 0,95 10,44, k_inst 0,97 10,56, düşük yükte Willans doğrusu 10,26

Düşük yük BSFC dışdeğerlemesi (V3-07): en düşük BSFC noktası %20 güçtedir; alçalma %5,1–%6,4, yedek bekleme %17,8–%17,9 güçtedir (dışdeğerleme). Alçalma yakıtı BSFC doğrusuyla 0,236 kg, Willans doğrusuyla (yakıt akışı güçle doğrusal; daha ihtiyatlı bir dışdeğerleme, sınır değil) 0,361 kg; yedek 2,030 / 2,048 kg; dayanım 10,318 / 10,257 h.

## 8. Kumanda yüzeyleri, kökler, kiriş derinliği, elektrik, yükler

* Stabilatör mili panel OAK'ının %25,9'inde (x = 3,738 m); girdap kafesi AM'si %29,7 (yalnız paneller) / %27,9 (kök parçalarıyla); AM bandı %25,9–%34,7 OAK → kol 0–36 mm (yüzey mil etrafında hiçbir durumda kararsız değil). Tam sapmada normal kuvvet katsayısı kesit polarından CN_maks 1,390 (NeuralFoil, Re 1.50e+06; trim kuralı sabiti 0,90 kullanılmaz); basınç merkezi en geride %34,8 OAK. Menteşe momenti VA'da 17,0 N·m, VD'de 9,1 N·m, sürekli trim 4,9 N·m; Volz DA 30, 2,5:1 dört çubuk bağlantısı (tam kinematik; tork oranı 2,50–4,18; −20/+15° için servo -58°/+40°) → bütün sapma aralığında tepe pay 1,88 (1,25 katsayısı dahil), anma torku / en büyük moment 1,18, sürekli pay 4,09; bağlantının ulaşabildiği en büyük sapma ±23,6°; anma hızında yüzey hızı nötrde 60°/s, −20°'de 36°/s.

| Yüzey | Eyleyici | Durum | Menteşe momenti (N·m) | Servo açısı (aralık uçları) | Tork oranı | Anma payı (≥ 1,1) | Tepe payı (≥ 1,0) | Yüzey hızı (°/s) |
|---|---|---|---|---|---|---|---|---|
| kanatçık | Volz DA 26 | 45,0 m/s EAS | 3,25 | -43° / 43° (sınır ±50°) | 2,00–2,60 | 2,12 | 3,15 | 77–100 |
| flap | Volz DA 30 | 39,6 m/s EAS | 11,14 | -43° / 43° (sınır ±85°) | 2,00–2,62 | 1,51 | 2,42 | 57–75 |
| dümen | Volz DA 26 (one per fin, extended-travel option) | 45,0 m/s EAS | 6,26 | -57° / 58° (sınır ±85°) | 2,00–3,64 | 1,29 | 1,91 | 55–100 |
| stabilatör | Volz DA 30 | 45,0 m/s EAS | 16,98 | -58° / 40° (sınır ±85°) | 2,50–4,18 | 1,18 | 1,88 | 36–60 |
* Stabilatör milleri: iki kısa mil; iç yatak motor bölmesi halka çerçevesinde krank/SG750 zarfının yanında (y = 0,110 m), dış yatak sabit kök parçasında; silindirlerin 10 mm önünde; kök parçasının mil istasyonundaki kalınlığı 58,2 mm (yatak yuvası + kaplama payı 10,2 mm).
* Stabilatör kökü: sabit kök parçasına boşluk 8,0 mm (bütün sapma aralığında sabit), gövdeye en az pay 22,4 mm (-20°…15°). Dikey kuyruk kökü bütün veter boyunca en az 20,8 mm, ventral kanatçık kökü en az 15,0 mm gömülü.
* Kiriş derinliği (gövde yanı → dış panel bağlantısı): ana kiriş en az 90,7 mm (gerekli 92,8 mm), arka kiriş en az 41,1 mm (gerekli 41,0 mm = bağlantı kesitinin kendi arka kiriş derinliği).
* Elektrik (R-32, R-52): jeneratörün DC çıkışı = 800 W × rpm/7500 × güç elektroniği verimi 0,94. E180 tepe yükü 493 W bütün bekleme boyunca jeneratör + bataryanın tepe destek payıyla karşılanır (R-52): bekleme devri alt sınırı 4.747 rpm (jeneratör en çok 17 W eksik kalır); tepe yük bütün bekleme sürseydi bataryadan çekilecek enerji tasarım görevinde 123,3 Wh, E180 görevinde 98,3 Wh; tepe destek payı 124,5 Wh (12S2P Molicel INR-21700-P45B Li-ion (cell-fused, vented box): kullanılabilir 311,2 Wh − jeneratör kaybı/marş yedeği 186,7 Wh) → pay 1,010. Jeneratör tek başına tasarım görevinin en düşük bekleme devrinde E180 tepesini 0,966, HD59 tepesini 1,099 payla karşılar. E180 görevinin dayanımı 10,07 h. Sürekli yük (en büyüğü E180 ile 373 W) iki görevin en düşük çıkışına göre pay 1,276 (R-32).
* E180 büyüme görevi (V3-08): jeneratör bu görevde E180 sürekli yükünü taşır (373 W; alçalma devir tabanı 3.720 rpm).
* R-52 işletme sınırı (V3-09): tepe destek payı, R-52'nin denetlediği bekleme sürelerine göre boyutlanmıştır (tasarım görevi 8,01 h). E180 tepesi devir tabanında sürekli çekilirse pay en az 7,33 h bekleme yeter (açık her an üst sınırda); daha uzun beklemede (daha hafif yük, daha çok yakıt) pay bitince güç yönetimi bekleme devrini jeneratörün tek başına E180 tepesini taşıdığı 4.917 rpm'e çıkarır ya da tepe yük süresi sınırlanır.
* Alçalma (V2-07): motor jeneratörün sürekli yükü taşıdığı devirde (3.521 rpm), 2,5 m/s alçalma hızı için çözülen 33,6–34,9 m/s TAS'ta; 20,0 dk, 0,236 kg yakıt (eski sabit kesir 0,998 ile 0,25 kg).
* İşletme sınırları (R-54): VNE 51,3 m/s EAS (0,9 VD), VNO 45,0 m/s EAS; uçuş kontrol sistemi hızı 45,0 m/s EAS ile sınırlar. Tam güçte düz uçuş hızı 54,0 m/s (DS) VNE'yi aştığından zarf koruması zorunlu bir işlevdir.
* Kuyruk kökleri (R-53): gövde dış yüzeyinde budanmış kök yapıları (kök bandı 20 mm, motor duvarının gerisinde 12 mm) ve stabilatör milleri iç bölgelerle çakışmıyor (0 çakışma); en küçük aralıklar: fin/stabilator_stub 41 mm, fin/ventral 403 mm, fin/stabilator_spindle 61 mm, stabilator_stub/ventral 339 mm, ventral/stabilator_spindle 300 mm.
* Rüzgâr hamlesi matrisi (yapılandırma CLα 6,212 1/rad): 106,4 kg / 0 m: +6,49 / -4,49; 106,4 kg / 3000 m: +6,84 / -4,84; 106,4 kg / 4500 m: +6,99 / -4,99; 149,9 kg / 0 m: +5,18 / -3,18; 149,9 kg / 3000 m: +5,37 / -3,37; 149,9 kg / 4500 m: +5,46 / -3,46. Kanat tasarım yük katsayısı 5,46 (en büyük n·m·g, MTOM); kök momenti (nihai) 9.145 N·m.

## 9. Yerleşim ve hacimler

| Bölge | Sığıyor |
|---|---|
| power_switching_bay | evet |
| avionics_power_deck | evet |
| avionics_side_bays | evet |
| parachute_bay | evet |
| mission_computer | evet |
| equipment_bay_aft | evet |
| forward_bay | evet |
| forward_bay_lower | evet |
| nose_cone_bay | evet |
| wing_carry_through | evet |
| payload_bay | evet |
| engine_cylinder_slab | evet |
| engine_intake_box | evet |
| engine_crankcase_sg750 | evet |
| bay_contents | evet |
| main_gear_wells | evet |
| nose_gear_well | evet |
| turret_bay | evet |
| stabilator_spindle | evet |
| overlaps | evet |

Bölme içerikleri (V3-02; gerçek zarflar + bağlayıcı payı; R-26'nın parçası):

| Öğe | Bölge | Boyut (mm) | OML'ye pay (mm) | Bölge duvarına pay (mm) | Komşuya en yakın (mm) |
|---|---|---|---|---|---|
| contactor_fuses | power_switching_bay | 100,0 × 60,0 × 50,0 | 20,5 | 3,0 | 14,0 |
| dcdc_28_12 | power_switching_bay | 60,0 × 60,0 × 25,0 | 55,4 | 5,0 | 14,0 |
| pdu | avionics_power_deck | 235,0 × 195,0 × 54,5 | 15,5 | 3,0 | 15,0 |
| buffer_battery | forward_bay | 184,4 × 76,6 × 96,2 | 18,9 | 3,2 | 15,4 |
| autopilot | avionics_side_bays | 76,0 × 65,0 × 40,0 | 21,4 | 3,0 | 10,0 |
| datalink_primary | avionics_side_bays | 101,6 × 66,8 × 38,4 | 40,4 | 3,4 | 11,6 |
| datalink_backup | avionics_side_bays | 77,0 × 55,0 × 28,0 | 73,2 | 3,0 | 10,0 |
| transponder | avionics_side_bays | 47,0 × 54,0 × 13,0 | 97,6 | 15,0 | 10,0 |
| remote_id | nose_cone_bay | 37,0 × 26,0 × 16,0 | 14,8 | 3,0 | 95,2 |
| fts_unit | forward_bay_lower | 80,0 × 60,0 × 30,0 | 21,4 | 3,0 | 15,4 |
| generator_pe | avionics_side_bays | 120,0 × 80,0 × 50,0 | 18,9 | 4,0 | 13,0 |
| brake_unit | avionics_side_bays | 102,8 × 54,0 × 26,0 | 91,3 | 9,5 | 13,0 |

Yakıt hacmi: gereken 41,7 L, kullanılabilir 49,4 L (tank verimi dahil); yakıt AM'si x = 2,615 m.

## 10. Ödünleşimler

| Pervane (her biri kendi kapanışıyla) | Dayanım (h) | Tırmanma DS (m/s) | Kalkış / iniş koşusu (m) | Statik itki (N) | Uç Mach | Karşılanmayan gereksinimler |
|---|---|---|---|---|---|---|
| Mejzlik 31x12 3B (tasarım) | 10,32 | 4,22 | 178 / 183 | 492 | 0,712 | – |
| Mejzlik 32x18 2B | 8,60 | 3,56 | 207 / 182 | 446 | 0,713 | R-02, R-02b, R-05, R-06, R-21, R-26, R-33, R-50, R-56 |

Dayanımın sürüklemeye duyarlılığı: her uçuş evresinde +0,001 CD → -0,104 h.

İniş takımı: içeri katlanır 10,32 h (14,57 kg) – sabit kaportalı 11,99 h (9,55 kg).
Taret: geri çekilir 10,32 h – sabit (sürekli dışarıda) 11,03 h.

| Açıklık (m) | VS hedefi (m/s) | S (m²) | AR | Kanat (kg) | Boş (kg) | Dayanım (h) | Kalkış (m) |
|---|---|---|---|---|---|---|---|
| 7,20 (tasarım) | 23,95 | 3,237 | 16,0 | 15,34 | 101,63 | 10,32 | 178 |
| 6,80 | 23,95 | 3,229 | 14,3 | 15,22 | 101,63 | 10,10 | 185 |
| 7,60 | 23,95 | 3,247 | 17,8 | 15,42 | 101,81 | 10,39 | 172 |
| 7,20 | 23,00 | 3,504 | 14,8 | 16,26 | 103,56 | 9,48 | 167 |
| 7,20 | 24,50 | 3,096 | 16,7 | 14,86 | 100,71 | 10,67 | 186 |

| Gövde uzatma (m) | Boy (m) | V_H | Boş (kg) | Dayanım (h) | Cnβ |
|---|---|---|---|---|---|
| -0,10 | 3,90 | 0,432 | 101,77 | 10,22 | 0,0588 |
| 0,10 | 4,10 | 0,441 | 101,62 | 10,35 | 0,0591 |

Tasarım değişiklikleri merdiveni (görev yüküyle; her değişiklik tek başına geri alınırsa, tam tasarım kapanışıyla):

| Durum | Dayanım (h) | Fark (h) |
|---|---|---|
| v1.6 tasarımı | 10,32 | – |
| k_inst 0,95 (tahmin 0,93 yerine; pervane/gövde kurulum testi bekliyor) | 10,44 | 0,12 |
| R-52 v1.3 biçimi (yalnız takılı taretin tepesi; kullanıcı onaylamadı, uygulanmadı) | 10,49 | 0,17 |
| v1.2 çözümü: LiFePO4 14S2P batarya, E180 tepesi yalnız jeneratörle (E180 devir tabanı) | 9,83 | -0,48 |
| k_inst 0,95 ve R-52 v1.3 birlikte (ikisi de uygulanmadı) | 10,65 | 0,33 |
| c/4 süpürme 8° → 6° (v1.2) | 10,17 | -0,14 |
| burulma 4° → 3° (v1.2) | 10,24 | -0,08 |
| LERX tepesi x 1,80 → 1,85 m (v1.2) | 10,30 | -0,02 |
| kapaklar contalı değil (v1.1: uzatılmış takım sürüklemesinin %10'u) | 10,27 | -0,05 |
| eldiven/dış panel birleşimi y = 0,82 m (v1.1) | 10,12 | -0,20 |
| sivrilme 0,42 (v1.1) | 10,27 | -0,05 |

## Şekiller

* `ucav250/docs/fig/yk250_constraint.png`
* `ucav250/docs/fig/yk250_polars.png`
* `ucav250/docs/fig/yk250_cg_envelope.png`
* `ucav250/docs/fig/yk250_payload_endurance.png`
* `ucav250/docs/fig/yk250_vn.png`
* `ucav250/docs/fig/yk250_3view.png`
