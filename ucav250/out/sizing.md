# YK-250 HANÇER — boyutlandırma raporu (otomatik)

Kaynak: `ucav250/spec.yaml` (rev. v1.6, 2026-10-09); üreten: `python3 -m ucav250.analysis.sizing`. Bu dosya elle düzenlenmez.

> Kapsam: yalnızca sivil EO/IR gözetleme ve araştırma. Platform silah (weapon), mühimmat (munition), sert bağlantı noktası (hardpoint), pilon (pylon) veya yük bırakma (release) mekanizması içermez ve bunlar için tasarlanmamıştır; görünüm dili yalnızca biçimdir.

**Gereksinimler:** 60/62 karşılanıyor. 

## 1. Özet

| Büyüklük | Değer |
|---|---|
| MTOM / boş / yakıt / görev faydalı yükü | 149,9 / 108,2 / 30,2 / 11,5 kg |
| Azami faydalı yük (R-02b) | 20,0 kg: dayanım 7,02 h (yakıt 21,7 kg) |
| Kanat açıklığı / alan / AR | 7,20 m / 3,234 m² / 16,0 |
| Gövde boyu / genişlik / yükseklik | 4,00 / 0,80 / 0,46 m |
| Dayanım (tasarım görevi, görev yüküyle) | 10,29 h (bekleme 7,98 h) |
| Menzil (feribot, yedek dahil) | 1.144 km |
| Görev beklemesi başı / sonu (3000 m) | 32,45 m/s TAS (27,95 EAS), 4.854 rpm, 2,56 kg/h / 32,33 m/s TAS (27,86 EAS), 4.747 rpm, 2,39 kg/h |
| MTOM'da 3000 m bekleme noktası (karşılaştırma) | 33,1 m/s TAS (28,5 EAS), CL 0,92, L/D 16,1, 2,65 kg/h |
| CD0 temiz / bekleme (taret dışarıda) / takım açık | 0,0351 / 0,0370 / 0,0476 |
| (L/D)maks temiz / bekleme | 17,3 / 16,8 |
| CLmax kanat / trimli temiz | 1,331 / 1,293 |
| Tutunma hızı (temiz, MTOM, DS) | 23,95 m/s |
| Tırmanma DS / 3000 m | 4,22 / 2,38 m/s |
| Servis tavanı | 7.107 m |
| Kalkış / iniş koşusu (DS) | 179 / 183 m |
| Statik marj aralığı | %10,0 – %14,9 OAK |
| Cnβ | 0,0586 1/rad |

## 2. Gereksinim uyumu

| No | Gereksinim | Ölçüt | Hedef | Sonuç | Durum |
|---|---|---|---|---|---|
| R-01 | Azami kalkış kütlesi 150 kg'ın altında (SHT-İHA M2 sınıfı, 149,9 kg tavan) | `mtow_kg` | <= 149,900 kg | 149,900 | ✔ |
| R-02 | Tasarım görevi dayanımı ≥ 10 h, tasarım görevi faydalı yüküyle (mission.payload_design_kg: MTOM 149,9 kg'da aynı görevde ≥ 10,25 h veren, 0,5 kg'a aşağı yuvarlanmış en büyük faydalı yük; tırmanma, 2 x 100 km geçiş, 3000 m'de taret dışarıda bekleme, alçalma; %10 yedek ayrıca) | `endurance_h` | >= 10,000 h | 10,292 | ✔ |
| R-02b | Azami faydalı yükle (20 kg) aynı tasarım görevinde dayanım ≥ 9,5 h (MTOM 149,9 kg; yakıt = MTOM − boş kütle − 20 kg; azami yükün kendi ağırlık merkezi) | `endurance_max_payload_h` | >= 9,500 h | 7,023 | ✘ |
| R-03 | Azami faydalı yük ≥ 20 kg: MTOM'daki yükleme durumlarından biri 20 kg faydalı yük taşır ve bütün kontroller (ağırlık merkezi zarfı, statik marj, kalkış, R-02b) bu yükle de yapılır | `payload_max_checked_kg` | >= 20,000 kg | 20,000 | ✔ |
| R-04 | Servis tavanı (0,5 m/s) ≥ 4500 m, MTOM | `ceiling_service_m` | >= 4.500,000 m | 7.106,800 | ✔ |
| R-05 | Deniz seviyesi tırmanma hızı ≥ 4,0 m/s, MTOM, tam gaz (4,9 m/s karşılaştırma hedefi pervane sınırlı; bkz. pervane ödünleşimi) | `roc_sl_m_s` | >= 4,000 m/s | 4,215 | ✔ |
| R-06 | Kalkış yer koşusu ≤ 200 m (MTOM, ISA deniz seviyesi; 300 m pist / 1,5) | `takeoff_ground_roll_m` | <= 200,000 m | 179,160 | ✔ |
| R-07 | İniş yer koşusu ≤ 200 m (görev sonu kütlesi, ISA deniz seviyesi) | `landing_ground_roll_m` | <= 200,000 m | 182,903 | ✔ |
| R-08 | Tutunma hızı ≤ 24 m/s (temiz, trimli, MTOM, deniz seviyesi) | `vs_clean_sl_mtow_m_s` | <= 24,000 m/s | 23,953 | ✔ |
| R-09 | Statik marj ≥ %10 OAK (tüm yükleme durumları, öndeki nötr nokta) | `static_margin_min` | >= 0,100 - | 0,100 | ✔ |
| R-10 | Statik marj ≤ %30 OAK (tüm yükleme durumları) | `static_margin_max` | <= 0,300 - | 0,149 | ✔ |
| R-11 | Yön kararlılığı Cnβ ≥ 0,057 1/rad (0,001 1/derece) | `cn_beta_per_rad` | >= 0,057 1/rad | 0,059 | ✔ |
| R-12 | Geri devrilme açısı ≥ 15° (en arka zemin ağırlık merkezi) | `tipback_deg` | >= 15,000 deg | 40,170 | ✔ |
| R-13 | Yana devrilme açısı ≤ 55° | `turnover_deg` | <= 55,000 deg | 52,518 | ✔ |
| R-14 | Burun tekerleği yükü ≥ %8 (en arka ağırlık merkezi) | `nose_load_aft_cg` | >= 0,080 - | 0,163 | ✔ |
| R-15 | Burun tekerleği yükü ≤ %20 (en ön ağırlık merkezi) | `nose_load_fwd_cg` | <= 0,200 - | 0,173 | ✔ |
| R-16 | Pervane yer açıklığı ≥ 0,18 m: statik tutum, kalkış (yerden kesilme) tutumu ve teker koyma tutumunun en kritiği (MTOM; kalkışta amortisör statik çökmede, teker koymada yüksüz) | `prop_clear_min_925a_m` | >= 0,180 m | 0,184 | ✔ |
| R-17 | Sönük ana lastik + dibe oturmuş amortisörde pozitif pervane açıklığı (≥ 0,02 m) | `prop_clear_flat_tyre_m` | >= 0,020 m | 0,059 | ✔ |
| R-18 | Kuyruk tamponu pervaneden önce yere değer (açı farkı ≥ 1°) | `bumper_before_prop_margin_deg` | >= 1,000 deg | 3,943 | ✔ |
| R-19 | Kalkış ve flare açısında kuyruk tamponu yere değmez (pay ≥ 2°) | `bumper_rotation_margin_deg` | >= 2,000 deg | 2,300 | ✔ |
| R-20 | Teker koyma açısı ≥ 3° (ana tekerler önce değer) | `touchdown_attitude_deg` | >= 3,000 deg | 4,872 | ✔ |
| R-21 | Ana iniş takımı gövde içine toplanır (teker, bacak, mafsal; kanat kutusu ile çakışma yok) | `main_gear_stowed` | == 1,000 - | 1,000 | ✔ |
| R-22 | Burun iniş takımı omurga yuvasına toplanır | `nose_gear_stowed` | == 1,000 - | 1,000 | ✔ |
| R-23 | Taret içeride kapakların üstünde kalır (gömülü) | `turret_flush_margin_m` | >= 0,000 m | 0,034 | ✔ |
| R-24 | E180 büyüme zarfı (0,18 x 0,23 m) taret bölmesine sığar | `turret_growth_inside` | == 1,000 - | 1,000 | ✔ |
| R-25 | Taret dışarıda: nadirden en az −5° yükselime kadar her azimutta engelsiz görüş | `turret_fov_upper_min_deg` | >= -5,000 deg | -3,000 | ✔ |
| R-26 | Yerleşim bölgeleri gövdeye sığar ve bildirilen içerikleri (aviyonik/güç bölmeleri: gerçek bileşen zarfları + bağlayıcı payı) kendi bölgelerine, gövdeye ve birbirlerine açıklıklarıyla sığar (hata sayısı 0) | `packaging_zones_failed` | == 0,000 - | 0,000 | ✔ |
| R-27 | Yerleşim bölgeleri ve yakıt hücreleri çakışmaz (çakışma sayısı 0) | `zone_overlaps` | == 0,000 - | 0,000 | ✔ |
| R-28 | Yakıt hacmi gereken hacmi karşılar (pay ≥ 0) | `fuel_volume_margin` | >= 0,000 - | 0,186 | ✔ |
| R-29 | Taşıma: çıkarılabilir dış kanat paneli ≤ 3,4 m | `outer_panel_length_m` | <= 3,400 m | 2,900 | ✔ |
| R-30 | Gövde + LERX orta kesiti ≤ 2,0 m (tek parça taşıma) | `centre_section_width_m` | <= 2,000 m | 1,400 | ✔ |
| R-31 | Statik pervane uç Mach sayısı ≤ 0,75 | `prop_tip_mach_static` | <= 0,750 - | 0,712 | ✔ |
| R-32 | Jeneratör çıkışı, beklemenin en düşük devrinde, en büyük sürekli elektrik yükünün ≥ 1,2 katı (E180 büyüme tareti ve araştırma yükü güç payı dahil) | `generator_margin_loiter` | >= 1,200 - | 1,276 | ✔ |
| R-33 | Pervane uçları ile yapı arasında radyal açıklık ≥ 0,026 m (CS-VLA 925(c)(1)) | `prop_clear_radial_m` | >= 0,026 m | 0,031 | ✔ |
| R-34 | Pervane palaları ile sabit yapı (kaporta, kuyruk yüzeyleri) arasında boyuna açıklık ≥ 0,013 m (CS-VLA 925(c)(2)) | `prop_clear_longitudinal_m` | >= 0,013 m | 0,029 | ✔ |
| R-35 | Kalkışta dönme yetkisi: stabilatörler burun tekerini ana tekerler hâlâ yüklüyken kaldırır (tekerlek arabası etkisi yok; bütün MTOM yükleme durumları) | `takeoff_main_gear_load_at_rotation_N` | > 0,000 N | 113,850 | ✔ |
| R-36 | Stabilatör trim gereksinimi (yerel CL, η_t dahil; temiz/kalkış CLmax ve tam güçte yerden kesilme/pas geçme, en ön ağırlık merkezleri; yerden kesilmede takım açık) ≤ 0,8 × stabilatör CLmax | `stab_trim_cl_local_max` | <= 0,720 - | 0,694 | ✔ |
| R-37 | Stabilatör eyleyicisi: tepe tork × dört çubuk bağlantısının o sapmadaki tork oranı ≥ 1,25 × en büyük menteşe momenti, bütün sapma aralığında (CS-LUAS.395(a)(1)) | `stab_hinge_peak_margin` | >= 1,000 - | 1,793 | ✔ |
| R-38 | Stabilatör kök boşluğu ≤ 10 mm (sabit kök parçasına karşı, bütün sapma aralığında sabit) | `stab_root_gap_m` | <= 0,010 m | 0,008 | ✔ |
| R-39 | Stabilatör kökü bütün sapma aralığında gövdeye girmez (gövde yarı genişliği ile pay ≥ 5 mm) | `stab_root_body_clearance_m` | >= 0,005 m | 0,023 | ✔ |
| R-40 | Dikey kuyruk ve ventral kanatçık kökleri bütün kök veteri boyunca gövdeye gömülü (boşluk ≤ 0) | `tail_root_max_gap_m` | <= 0,000 m | -0,015 | ✔ |
| R-41 | Pervane koruması (üst-yan): eğik dikeylerin firar kenarı, itki hattı açısıyla eğik pervane disk düzlemini uç çemberinin ≥ 26 mm dışında keser | `prop_guard_fin_crossing_margin_m` | >= 0,026 m | 0,075 | ✔ |
| R-42 | Pervane koruması (üst-yan): eğik dikeylerin firar kenarı disk düzlemini uç çemberinin ≤ 80 mm dışında keser (diski yakından çevreler) | `prop_guard_fin_crossing_margin_m` | <= 0,080 m | 0,075 | ✔ |
| R-43 | Ana kiriş hattında OML derinliği (gövde yanından dış panel bağlantısına) ≥ birleşim kalınlığının %95'i | `spar_depth_main_ratio` | >= 0,950 - | 0,977 | ✔ |
| R-44 | Arka kiriş hattında OML derinliği (gövde yanından dış panel bağlantısına) ≥ dış panel bağlantı kesitinin arka kiriş derinliğinin %95'i (arka kiriş kademesiz devam eder) | `spar_depth_rear_ratio` | >= 0,950 - | 1,002 | ✔ |
| R-45 | Kanat kökü ile köşe çizgisi arasında basamak ≤ 10 mm (LERX/köşe çizgisi geçişi) | `wing_root_chine_step_m` | <= 0,010 m | 0,000 | ✔ |
| R-46 | Flap, sökülebilir dış panelin üzerinde kalır (iç ucu panel bağlantısının dışında) | `flap_inboard_end_outboard_of_joint_m` | >= 0,000 m | 0,030 | ✔ |
| R-47 | Stabilatör eyleyicisi: anma (sürekli) torku × dört çubuk bağlantısının o sapmadaki tork oranı en büyük menteşe momentinin ≥ 1,1 katı, bütün sapma aralığında (mil, girdap kafesi AM bandının ön ucunda) | `stab_hinge_rated_margin_max` | >= 1,100 - | 1,121 | ✔ |
| R-48 | Pervane koruması (alt): ventral kanatçığın firar kenarı eğik disk düzlemini uç çemberinin ≥ 26 mm dışında keser; değiştirilebilir tampon kızağı diskin altındadır | `prop_guard_ventral_margin_m` | >= 0,026 m | 0,050 | ✔ |
| R-49 | Pervane koruması (alt): ventral kanatçığın firar kenarı disk düzlemini uç çemberinin ≤ 120 mm dışında keser (alt bölgeyi yakından korur) | `prop_guard_ventral_margin_m` | <= 0,120 m | 0,050 | ✔ |
| R-50 | Pervane düzlemi kaporta firar kenarının ≥ 0,10 D gerisinde (itici pervane gövde izinde; kaporta tabanı küt olduğundan kurulum katsayısı k_inst için 0,93 tahmini kullanılır, duyarlılık doc 02 §6.5) | `prop_plane_behind_cowl_over_D` | >= 0,100 - | 0,102 | ✔ |
| R-51 | Yanal koruma: uçak kanat ucu yere değene kadar yatarsa pervane ucu yerden ≥ 0,05 m yukarıda kalır (kanat ucu önce değer) | `prop_clear_wingtip_on_ground_m` | >= 0,050 m | 0,321 | ✔ |
| R-52 | E180 büyüme taretinin tepe elektrik yükü (+ temel yük + araştırma yükü payı) tasarım görevinin beklemesi boyunca kesintisiz karşılanır (v1.2 yeteneği): jeneratörün DC çıkışı (güç elektroniği verimi dahil) ile tampon bataryanın tepe destek payı birlikte; tepe yükün bütün bekleme boyunca sürdüğü varsayılır (görev çevrimi kredisi yok); tepe destek payı = kullanılabilir batarya enerjisi − jeneratör kaybı/marş yedeği (400 W × 28 dk); E180 büyüme görevi de aynı kuralla kontrol edilir | `e180_peak_support_margin` | >= 1,000 - | 1,018 | ✔ |
| R-53 | Kuyruk kök yapıları (gövde dış yüzeyinde budanmış dikey, sabit stabilatör kök parçası ve ventral kökleri: kök kaburgası + bağlantı bandı) ve stabilatör milleri iç yerleşim bölgeleriyle ve birbirleriyle çakışmaz (çakışma sayısı 0) | `tail_root_interferences` | == 0,000 - | 0,000 | ✔ |
| R-54 | İşletme sınırları (CS-LUAS 1505): VNE = 0,9 VD, VNO = min(VC; 0,89 VNE); uçuş kontrol sisteminin zarf koruması hızı VNO'nun altında tutar (sınır hız ≤ VNO; tam güçte düz uçuş hızı VNE'yi aşar) | `fcs_speed_limit_margin_m_s` | >= 0,000 m/s | 0,000 | ✔ |
| R-55 | MTOM'da acil/geri dönüş inişi (flapsız, ISA deniz seviyesi) yer koşusu ≤ 300 m pist uzunluğu (anormal durum: 1,5 alan katsayısı uygulanmaz; normal iniş R-07 ve en büyük iniş kütlesi işletme sınırıdır) | `landing_ground_roll_mtow_m` | <= 300,000 m | 219,541 | ✔ |
| R-56 | Boş kütle bütçesi (mass.budget, grup tavanları) ve yedek payı, R-02'yi (tasarım görevi faydalı yükü, 10 h) ve R-02b'yi (20 kg, 9,5 h) tam karşılayan iki boş kütlenin küçüğünü aşmaz (pay ≥ 0); her grubun tahmini kendi tavanının altında kalır | `mass_budget_margin_kg` | >= 0,000 kg | -6,524 | ✘ |
| R-57 | Kanatçık, flap ve dümen eyleyicileri: anma torku × dört çubuk bağlantısının o sapmadaki tork oranı ≥ 1,1 × menteşe momenti, bütün sapma aralığında (kanatçık ve dümen VA'da tam sapma ve VD'de 1/3 sapma, flap VF'de; R-47 kuralı) | `control_actuator_rated_margin_min` | >= 1,100 - | 1,286 | ✔ |
| R-58 | Kanatçık, flap ve dümen eyleyicileri: tepe tork × dört çubuk bağlantısının o sapmadaki tork oranı ≥ 1,25 × menteşe momenti, bütün sapma aralığında (CS-LUAS.395(a)(1)) | `control_actuator_peak_margin_min` | >= 1,000 - | 1,905 | ✔ |
| R-59 | Bütün kumanda yüzeylerinde (kanatçık, flap, dümen, stabilatör) dört çubuk bağlantısı istenen sapma aralığına eyleyicinin hareket sınırı içinde ulaşır (pay ≥ 0°) | `control_linkage_travel_margin_deg` | >= 0,000 deg | 6,773 | ✔ |
| R-60 | Kalkış dönüşü (uçuş kontrol sistemi dönüş yasası): yunuslama hızı komutu sıfırdan komut hızına rampalanır ve tutulur; burun tekeri yerden kesikken stabilatörün ana teker temas noktasına göre (I_yy + m a²) × açısal ivme momenti için gereken aşağı kuvveti her adımda azami aşağı kuvveti aşmaz (pay kırpılmadan ölçülür, ≥ 0; bütün MTOM yükleme durumları); yerden kesilmede ana teker tepkisi sıfırdır ve aşağı kuvvet havadaki değere (sabit hızda güç-açık trim) eşittir; moment artığı ya da bu süreklilik 1e-6'yı aşarsa ölçüt negatif olur | `takeoff_rotation_download_margin_min_N` | >= 0,000 N | 0,904 | ✔ |
| R-61 | Azami işletme irtifası (structures.operating_limits.max_operating_altitude_m, 4500 m): uçuş kontrol sisteminin irtifa zarfı koruması / coğrafi sınırı uçağı bu irtifanın altında tutar ve sınır, kanadın boyutlandırıldığı esinti matrisinin en üst irtifasını aşmaz (pay ≥ 0); MTOM'da servis tavanı daha yüksek olduğundan koruma zorunlu bir işlevdir | `fcs_altitude_limit_margin_m` | >= 0,000 m | 0,000 | ✔ |

### Gereksinim kararı: görev faydalı yükü kuralı

Kural: 0,5 kg'lık ızgarada (aşağı yuvarlanmış), dayanımı ≥ 10,25 h (10,00 h + 0,25 h sağlamlık payı) olan en büyük faydalı yük; azami yükü aşmaz. Spec değeri 11,5 kg; kapalı tasarımdan türetilen 11,5 kg (10,292 h; bir adım fazlası 12,0 kg ile 10,098 h). Azami yük 20,0 kg ile dayanım 7,023 h (R-02b ≥ 9,5 h).

Kural eşiğine pay (V4-03): görev yüküyle dayanım eşiğin 0,042 h üstündedir. Bu, sabit MTOM'da boş kütlenin 0,108 kg artmasına denktir (her kg boş kütle bir kg yakıttır); faydalı yük olarak eşik ≈ 11,61 kg'dır (ızgara noktaları arasında doğrusal). Boş kütle bundan fazla artarsa kural bir alt adımı verir.

Kütle bütçesi (R-56): R-02'yi tam karşılayan boş kütle 108,922 kg, R-02b'yi tam karşılayan 101,726 kg; belirleyen R-02b; tavanlar 108,20 kg + yedek 0,05 kg → pay -6,524 kg (R-02'ye göre 0,672 kg). Bu paylar tavan toplamı + yedekten ölçülür.

Aynı başvuru kütlesinden (bugünkü boş kütle tahmini 108,169 kg) ölçülen üç boş kütle payı (V5-05): görev yükü kuralı eşiği +0,108 kg, R-02b +-6,443 kg, R-02 +0,753 kg.

## 3. Kütle

| Grup | Kütle (kg) | Bütçe hedefi ± tolerans |
|---|---|---|
| chassis | 19,86 | 19,86 ± 0,60 |
| systems | 15,21 | 15,21 ± 0,46 |
| propulsion | 12,97 | 12,97 ± 0,39 |
| gear | 12,94 | 12,94 ± 0,39 |
| wing | 12,88 | 12,88 ± 0,39 |
| shell | 10,82 | 10,82 ± 0,32 |
| controls | 8,15 | 8,15 ± 0,24 |
| tail | 7,74 | 7,74 ± 0,23 |
| hardware | 5,25 | 5,26 ± 0,16 |
| fuel | 2,37 | 2,37 ± 0,07 |
| **boş (büyüme payı %5 dahil)** | **108,17** | |

Yükleme durumları (ağırlık merkezi, uçuş, takım içeride; taret durumu yükleme adında):

| Durum | Kütle (kg) | x_AM (m) | z_AM (m) | SM (% OAK) |
|---|---|---|---|---|
| mtow_design_payload_turret_retracted | 149,90 | 2,606 | 0,015 | 10,5 |
| mtow_design_payload_turret_extended | 149,90 | 2,606 | 0,014 | 10,5 |
| mtow_max_payload_turret_retracted | 149,90 | 2,607 | 0,007 | 10,3 |
| mtow_max_payload_turret_extended | 149,90 | 2,607 | 0,006 | 10,3 |
| full_fuel_baseline_sensors_only | 147,27 | 2,604 | 0,023 | 10,7 |
| zero_fuel_design_payload | 119,67 | 2,604 | 0,014 | 10,9 |
| zero_fuel_max_payload | 128,17 | 2,606 | 0,005 | 10,5 |
| reserve_fuel_design_payload | 123,30 | 2,604 | 0,016 | 10,8 |
| minimum_flying_turret_only | 112,94 | 2,608 | 0,028 | 10,0 |
| e180_growth_turret_full_fuel | 149,90 | 2,584 | 0,012 | 14,9 |

## 4. Aerodinamik

* Kaldırma eğimi (girdap kafesi, gövde planı hariç): 5,697 1/rad; taşıma hattı 6,154 1/rad. CL0 = 0,494.
* Açıklık verimi: taşıma hattı 0,965, Trefftz 0,979; polar uyumu e = 0,837 (profil sürüklemesinin CL ile değişimi ve trim dahil).
* Kritik kesit: η = 0,42 (LERX dışı), α_stall = 8,6°; Re kök/uç 1.95e+06 / 3.77e+05.

Sürükleme kalemleri (temiz, CD = D/q / S_ref):

| Kalem | CD |
|---|---|
| body | 0,00760 |
| aft_body_upsweep | 0,00118 |
| tail_stabilator | 0,00237 |
| tail_stabilator_stub | 0,00040 |
| tail_fin | 0,00219 |
| tail_ventral | 0,00017 |
| landing_gear | 0,00040 |
| eo_ir_turret | 0,00009 |
| engine_cooling | 0,00187 |
| aft_closure_base | 0,00216 |
| wing_body_junctions | 0,00024 |
| tail_body_junctions | 0,00058 |
| antennas_pitot_lights_vents | 0,00077 |
| control_surface_gaps | 0,00080 |
| leakage_protuberance | 0,00104 |
| kanat profil sürüklemesi (CL 0,7; geçiş zorlanmış ×1,15) | 0,01336 |

Bekleme durumunda taret dışarıda: +0,00166 CD.

## 5. Kararlılık

* Nötr nokta: klasik 2,667 m, girdap kafesi 2,656 m → kullanılan (öndeki) 2,656 m.
* Taşıyıcı gövde (Multhopp) Cmα = 0,818 1/rad (ön gövde 0,674, arka 0,144); kanat-gövde AM 2,572 m, Cm0_wb -0,123.
* Kuyruk hacmi V_H 0,450, V_V 0,0274; aşağı sapma dε/dα 0,384.
* Cnβ: eğik dikeyler 0,0944, ventral 0,0029, gövde -0,0387 → toplam 0,0586 1/rad. Clβ -0,1142 1/rad.

## 6. İniş takımı ve taret

* Zemin z = -0,431 m; dingil açıklığı 2,297 m; iz 0,836 m; geri devrilme 40,2°; yana devrilme 52,5°; burun yükü %16,3 – %17,3.
* Statik zemin tutumu 2,5° burun yukarı (burun tekeri teması ana tekerlerin 0,100 m altında, gövde ekseninde).
* Pervane yer açıklığı (CS-VLA 925(a), MTOM): statik 0,209 m, yerden kesilme tutumu 3,8° (amortisör statik) 0,184 m, teker koyma tutumu 4,9° (takım yüksüz, +0,042 m) 0,204 m → en az 0,184 m; sönük lastik + dibe oturmuş amortisör 0,059 m. Pervane temas açısı 13,1°, kuyruk tamponu temas açısı 9,2°; flare 6,9°.
* CS-VLA 925(c): radyal açıklık en az 0,031 m, boyuna açıklık en az 0,029 m (pala ekseni boyutu uç 0,009 m; parçalara göre: body 0,042, stabilator 0,096, stabilator_stub 0,094, fin 0,029, ventral 0,049). Pervane düzlemi kaporta firar kenarının 80 mm (0,102 D) gerisinde.
* Pervane koruması (itki hattı açısıyla eğik disk düzleminde): stabilatör 90° (tepeden), uç çemberinin 446 mm dışı; stabilatör -90° (tepeden), uç çemberinin 446 mm dışı; eğik dikey 36° (tepeden), uç çemberinin 75 mm dışı; eğik dikey -36° (tepeden), uç çemberinin 75 mm dışı; ventral kanatçık 180° (tepeden), uç çemberinin 50 mm dışı. Uç çemberinin 0,30 m içinde hiçbir yapının bulunmadığı açık bölge payı %83; disk açıktır, personel koruması iddia edilmez. Kanat ucu yere değene kadar yatışta (10,2°) pervane ucu yerden 0,321 m yukarıda kalır.
* Toplanmış takım: ana teker zarfı içeride evet, bacak evet, mafsal evet, merkez boşluğu 0,038 m, kanat kutusu açıklığı 0,034 m; burun takımı evet.
* Taret: içeride top kapak üstünde 0,034 m; E180 büyüme zarfı içeride evet; strok 0,128 m; görüş alanı üst sınırı en az -3,0° (ön ±60°: -3,0°).

## 7. Performans

| Durum | Değer |
|---|---|
| Kalkış (DS, MTOM, belirleyici yükleme: E180 taret, tam yakıt) | koşu 179 m, 15 m'ye 326 m; burun kaldırma V_R 24,8 m/s (ana tekerlerde 114 N), V_LOF 25,2 m/s, yerden kesilme tutumu 2,6° (zemin tutumu 2,5°, dönüş 0,30 s) (VS_TO 22,0 m/s, flap 35°) |
| — uçuş kontrol sistemi stabilatör programı | 23,5 m/s'den itibaren ana tekerleri yüklü tutan en küçük aşağı kuvvet, V_R'de tam burun yukarı (yerel CL 0,90); dönüşte sabit yunuslama hızı (1,09°/s), aşağı kuvvet ana teker temas noktasına göre moment dengesinden: 190 N'dan yerden kesilmede güç-açık trim değerine 149 N (trim 149 N); en büyük aşağı kuvvete en küçük pay 1,1 N, moment artığı en çok 7.1e-14 N·m; 24,2 m/s (1,1 VS_TO) altında yerden kesilme yok |
| — yunuslama ataleti (V4-05) | I_yy 112,8 kg·m² (kütle kalemlerinden; deri, kanat ve kuyruk yüzeylerine yayılı, uzun kalemler çubuk); yunuslama hızı komutu 0,3 s'de rampalanır (3,64°/s², V_R'de ana teker noktasına göre 8,5 N·m). V_R 24,62 → 24,78 m/s, koşu 175,6 → 179,2 m (+3,5 m); 0,1 s rampayla 180,7 m |
| — MTOM, görev yükü | koşu 172 m, V_R 24,1, V_LOF 24,8 m/s, θ_LOF 2,8°, x_AM 2,602 m, aşağı kuvvet payı 1,3 N |
| — MTOM, görev yükü, taret dışarıda | koşu 172 m, V_R 24,1, V_LOF 24,8 m/s, θ_LOF 2,8°, x_AM 2,602 m, aşağı kuvvet payı 2,3 N |
| — MTOM, azami yük | koşu 172 m, V_R 24,1, V_LOF 24,8 m/s, θ_LOF 2,8°, x_AM 2,603 m, aşağı kuvvet payı 0,9 N |
| — MTOM, azami yük, taret dışarıda | koşu 172 m, V_R 24,2, V_LOF 24,8 m/s, θ_LOF 2,8°, x_AM 2,603 m, aşağı kuvvet payı 1,9 N |
| — E180 taret, tam yakıt | koşu 179 m, V_R 24,8, V_LOF 25,2 m/s, θ_LOF 2,6°, x_AM 2,580 m, aşağı kuvvet payı 1,1 N |
| Kalkış (1500 m ISA) | koşu 276 m |
| İniş (görev sonu 123,3 kg) | koşu 183 m, V_TD 25,0 m/s |
| İniş (MTOM, acil dönüş, flapsız; R-55) | koşu 220 m, V_TD 27,5 m/s; 35° flapla 248 m |
| 0 m | VS 24,0 m/s, Vmaks 54,0 m/s, tırmanma 4,22 m/s @ 29,5 m/s |
| 3000 m | VS 27,8 m/s, Vmaks 51,2 m/s, tırmanma 2,38 m/s @ 32,0 m/s |
| Tavan | servis 7.107 m, mutlak 8.349 m |

Faydalı yük – dayanım (MTOM 149,9 kg; yakıt = MTOM − boş − yük, yakıt hacmiyle sınırlı). Tablo izin verilen en hafif yüklemeden, temel EO/IR setinden başlar (V4-02):

| Yük (kg) | Yakıt (kg) | Kalkış kütlesi (kg) | Dayanım (h) | Bekleme (h) | SM aralığı (% OAK) | E180 tepesi bütün bekleme sürseydi batarya (Wh) | E180 tepesinin desteklendiği bekleme (h) |
|---|---|---|---|---|---|---|---|
| 3,25 | 35,85 (hacim sınırı) | 147,3 | 12,60 | 10,28 | 10,7–11,3 | – (E180 seti sığmaz) | – |
| 5,00 | 35,85 (hacim sınırı) | 149,0 | 12,52 | 10,20 | 10,7–11,2 | – (E180 seti sığmaz) | – |
| 5,50 | 35,85 (hacim sınırı) | 149,5 | 12,49 | 10,18 | 10,7–11,1 | 160,9 > pay | 8,04 |
| 5,88 | 35,85 | 149,9 | 12,47 | 10,16 | 10,6–11,1 | 159,1 > pay | 8,13 |
| 6,00 | 35,73 | 149,9 | 12,43 | 10,12 | 10,6–11,1 | 158,3 > pay | 8,13 |
| 7,50 | 34,23 | 149,9 | 11,84 | 9,53 | 10,6–11,0 | 148,4 > pay | 8,13 |
| 10,00 | 31,73 | 149,9 | 10,87 | 8,56 | 10,5–10,9 | 132,1 > pay | 8,12 |
| 11,50 | 30,23 | 149,9 | 10,29 | 7,98 | 10,5–10,9 | 122,3 | bütün bekleme |
| 12,00 | 29,73 | 149,9 | 10,10 | 7,79 | 10,5–10,8 | 119,1 | bütün bekleme |
| 12,50 | 29,23 | 149,9 | 9,90 | 7,60 | 10,5–10,8 | 115,8 | bütün bekleme |
| 15,00 | 26,73 | 149,9 | 8,94 | 6,64 | 10,4–10,7 | 99,6 | bütün bekleme |
| 17,50 | 24,23 | 149,9 | 7,98 | 5,68 | 10,4–10,6 | 83,4 | bütün bekleme |
| 20,00 | 21,73 | 149,9 | 7,02 | 4,73 | 10,3–10,5 | 67,5 | bütün bekleme |

Tepe destek payı 124,5 Wh. '> pay' satırlarında E180 tepesi bütün bekleme boyunca karşılanamaz (R-52 işletme sınırı, bkz. §8). E180 sütunları yalnız E180 setini (5,5 kg: E180 4,0 + görev bilgisayarı 1,0 + tepsi 0,5) taşıyabilen yükler içindir.

İzin verilmeyen yüklemeler (R-09, SM ≥ %10): faydalı yüksüz (taret de yok) SM %5,4–%6,3 aralığındadır; uçmak için taret bağlantısına (x = 1,220 m) 1,83 kg safra gerekir. Yalnız taretle (görev bilgisayarı ve tepsi yok) dolu depolarda SM %9,8 olur; bu yüklemede yakıt 7,8 kg ile sınırlıdır.

Duyarlılıklar (görev yüküyle dayanım, h): BSFC −%12 11,73, BSFC +%12 9,17, bütün sürükleme +%10 9,75, boş kütle +%5 8,21, bekleme 1000 m'de 9,72, geçişsiz 10,24, taret hep içeride 10,42, k_inst 0,90 10,10, k_inst 0,95 10,41, k_inst 0,97 10,53, düşük yükte Willans doğrusu 10,23

Düşük yük BSFC dışdeğerlemesi (V3-07): en düşük BSFC noktası %20 güçtedir; alçalma %5,1–%6,4, yedek bekleme %17,8–%17,9 güçtedir (dışdeğerleme). Alçalma yakıtı BSFC doğrusuyla 0,236 kg, Willans doğrusuyla (yakıt akışı güçle doğrusal; daha ihtiyatlı bir dışdeğerleme, sınır değil) 0,361 kg; yedek 2,028 / 2,045 kg; dayanım 10,292 / 10,232 h.

## 8. Kumanda yüzeyleri, kökler, kiriş derinliği, elektrik, yükler

* Stabilatör mili panel OAK'ının %25,9'inde (x = 3,738 m); girdap kafesi AM'si %29,7 (yalnız paneller) / %27,9 (kök parçalarıyla); AM bandı %25,9–%34,7 OAK → kol 0–37 mm (yüzey mil etrafında hiçbir durumda kararsız değil). Tam sapmada normal kuvvet katsayısı kesit polarından CN_maks 1,390 (NeuralFoil, Re 1.50e+06; trim kuralı sabiti 0,90 kullanılmaz); basınç merkezi en geride %34,8 OAK. Menteşe momenti VA'da 17,8 N·m, VD'de 9,5 N·m, sürekli trim 5,1 N·m; Volz DA 30, 2,5:1 dört çubuk bağlantısı (tam kinematik; tork oranı 2,50–4,18; −20/+15° için servo -58°/+40°) → bütün sapma aralığında tepe pay 1,79 (1,25 katsayısı dahil), anma torku / en büyük moment 1,12, sürekli pay 3,90; bağlantının ulaşabildiği en büyük sapma ±23,6°; anma hızında yüzey hızı nötrde 60°/s, −20°'de 36°/s.

| Yüzey | Eyleyici | Durum | Menteşe momenti (N·m) | Servo açısı (aralık uçları) | Tork oranı | Anma payı (≥ 1,1) | Tepe payı (≥ 1,0) | Yüzey hızı (°/s) |
|---|---|---|---|---|---|---|---|---|
| kanatçık | Volz DA 26 | 45,0 m/s EAS | 3,25 | -43° / 43° (sınır ±50°) | 2,00–2,60 | 2,13 | 3,15 | 77–100 |
| flap | Volz DA 30 | 39,6 m/s EAS | 11,13 | -43° / 43° (sınır ±85°) | 2,00–2,62 | 1,51 | 2,42 | 57–75 |
| dümen | Volz DA 26 (one per fin, extended-travel option) | 45,0 m/s EAS | 6,26 | -57° / 58° (sınır ±85°) | 2,00–3,64 | 1,29 | 1,91 | 55–100 |
| stabilatör | Volz DA 30 | 45,0 m/s EAS | 17,85 | -58° / 40° (sınır ±85°) | 2,50–4,18 | 1,12 | 1,79 | 36–60 |
* Stabilatör milleri: iki kısa mil; iç yatak motor bölmesi halka çerçevesinde krank/SG750 zarfının yanında (y = 0,110 m), dış yatak sabit kök parçasında; silindirlerin 10 mm önünde; kök parçasının mil istasyonundaki kalınlığı 59,1 mm (yatak yuvası + kaplama payı 11,1 mm).
* Stabilatör kökü: sabit kök parçasına boşluk 8,0 mm (bütün sapma aralığında sabit), gövdeye en az pay 22,6 mm (-20°…15°). Dikey kuyruk kökü bütün veter boyunca en az 20,8 mm, ventral kanatçık kökü en az 15,0 mm gömülü.
* Kiriş derinliği (gövde yanı → dış panel bağlantısı): ana kiriş en az 90,6 mm (gerekli 92,7 mm), arka kiriş en az 41,1 mm (gerekli 41,0 mm = bağlantı kesitinin kendi arka kiriş derinliği).
* Elektrik (R-32, R-52): jeneratörün DC çıkışı = 800 W × rpm/7500 × güç elektroniği verimi 0,94. E180 tepe yükü 493 W bütün bekleme boyunca jeneratör + bataryanın tepe destek payıyla karşılanır (R-52): bekleme devri alt sınırı 4.747 rpm (jeneratör en çok 17 W eksik kalır); tepe yük bütün bekleme sürseydi bataryadan çekilecek enerji tasarım görevinde 122,3 Wh, E180 görevinde 97,1 Wh; tepe destek payı 124,5 Wh (12S2P Molicel INR-21700-P45B Li-ion (cell-fused, vented box): kullanılabilir 311,2 Wh − jeneratör kaybı/marş yedeği 186,7 Wh) → pay 1,018. Jeneratör tek başına tasarım görevinin en düşük bekleme devrinde E180 tepesini 0,966, HD59 tepesini 1,099 payla karşılar. E180 görevinin dayanımı 10,04 h. Sürekli yük (en büyüğü E180 ile 373 W) iki görevin en düşük çıkışına göre pay 1,276 (R-32).
* E180 büyüme görevi (V3-08): jeneratör bu görevde E180 sürekli yükünü taşır (373 W; alçalma devir tabanı 3.720 rpm).
* R-52 işletme sınırı (V3-09): tepe destek payı, R-52'nin denetlediği bekleme sürelerine göre boyutlanmıştır (tasarım görevi 7,98 h). E180 tepesi devir tabanında sürekli çekilirse pay en az 7,33 h bekleme yeter (açık her an üst sınırda); daha uzun beklemede (daha hafif yük, daha çok yakıt) pay bitince güç yönetimi bekleme devrini jeneratörün tek başına E180 tepesini taşıdığı 4.917 rpm'e çıkarır ya da tepe yük süresi sınırlanır.
* Alçalma (V2-07): motor jeneratörün sürekli yükü taşıdığı devirde (3.521 rpm), 2,5 m/s alçalma hızı için çözülen 33,6–34,9 m/s TAS'ta; 20,0 dk, 0,236 kg yakıt (eski sabit kesir 0,998 ile 0,25 kg).
* İşletme sınırları (R-54): VNE 51,3 m/s EAS (0,9 VD), VNO 45,0 m/s EAS; uçuş kontrol sistemi hızı 45,0 m/s EAS ile sınırlar. Tam güçte düz uçuş hızı 54,0 m/s (DS) VNE'yi aştığından zarf koruması zorunlu bir işlevdir.
* Azami işletme irtifası (R-61, düzeltme turu 3, VS3-02): 4.500 m; rüzgâr esintisi zarfı (ve kanat boyutlandırması) 4.500 m'ye kadar hesaplandı. MTOM'da servis tavanı 7.107 m bu sınırın üstünde olduğundan uçuş kontrol sisteminin irtifa zarfı koruması / coğrafi sınırı zorunlu bir işlevdir (daha yüksek irtifada esinti yük katsayısı büyür, kanat başlık payları negatife düşer).
* Kuyruk kökleri (R-53): gövde dış yüzeyinde budanmış kök yapıları (kök bandı 20 mm, motor duvarının gerisinde 12 mm) ve stabilatör milleri iç bölgelerle çakışmıyor (0 çakışma); en küçük aralıklar: fin/stabilator_stub 40 mm, fin/ventral 403 mm, fin/stabilator_spindle 61 mm, stabilator_stub/ventral 339 mm, ventral/stabilator_spindle 300 mm.
* Rüzgâr hamlesi matrisi (yapılandırma CLα 6,231 1/rad): 112,9 kg / 0 m: +6,25 / -4,25; 112,9 kg / 3000 m: +6,57 / -4,57; 112,9 kg / 4500 m: +6,71 / -4,71; 149,9 kg / 0 m: +5,18 / -3,18; 149,9 kg / 3000 m: +5,38 / -3,38; 149,9 kg / 4500 m: +5,47 / -3,47. Kanat tasarım yük katsayısı 5,47 (en büyük n·m·g, MTOM); kök momenti (nihai) 9.160 N·m.

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
