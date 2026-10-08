# YK-250 HANÇER — boyutlandırma raporu (otomatik)

Kaynak: `ucav250/spec.yaml` (rev. v1.1, 2026-10-06); üreten: `python3 -m ucav250.analysis.sizing`. Bu dosya elle düzenlenmez.

> Kapsam: yalnızca sivil EO/IR gözetleme ve araştırma. Platform silah (weapon), mühimmat (munition), sert bağlantı noktası (hardpoint), pilon (pylon) veya yük bırakma (release) mekanizması içermez ve bunlar için tasarlanmamıştır; görünüm dili yalnızca biçimdir.

**Gereksinimler:** 45/46 karşılanıyor. **Türetilmiş değer kontrolü:** 1195 değer, 0 sapma.

## 1. Özet

| Büyüklük | Değer |
|---|---|
| MTOM / boş / yakıt / faydalı yük | 149,9 / 101,8 / 28,1 / 20,0 kg |
| Kanat açıklığı / alan / AR | 7,20 m / 3,226 m² / 16,1 |
| Gövde boyu / genişlik / yükseklik | 4,02 / 0,80 / 0,46 m |
| Dayanım (tasarım görevi) | 9,59 h (bekleme 7,30 h) |
| Menzil (feribot, yedek dahil) | 1.065 km |
| Bekleme 3000 m | 33,3 m/s TAS (28,7 EAS), CL 0,91, L/D 15,4, 2,70 kg/h |
| CD0 temiz / bekleme (taret dışarıda) / takım açık | 0,0369 / 0,0394 / 0,0485 |
| (L/D)maks temiz / bekleme | 17,0 / 16,4 |
| CLmax kanat / trimli temiz | 1,365 / 1,302 |
| Tutunma hızı (temiz, MTOM, DS) | 23,91 m/s |
| Tırmanma DS / 3000 m | 4,62 / 2,66 m/s |
| Servis tavanı | 7.481 m |
| Kalkış / iniş koşusu (DS) | 191 / 190 m |
| Statik marj aralığı | %10,1 – %14,7 OAK |
| Cnβ | 0,0591 1/rad |

## 2. Gereksinim uyumu

| No | Gereksinim | Ölçüt | Hedef | Sonuç | Durum |
|---|---|---|---|---|---|
| R-01 | Azami kalkış kütlesi 150 kg'ın altında (SHT-İHA M2 sınıfı, 149,9 kg tavan) | `mtow_kg` | <= 149,900 kg | 149,900 | ✔ |
| R-02 | Tasarım görevi dayanımı ≥ 10 h (tırmanma, 2 x 100 km geçiş, 3000 m'de taret dışarıda bekleme, alçalma; %10 yedek ayrıca) | `endurance_h` | >= 10,000 h | 9,588 | ✘ |
| R-03 | Tasarım faydalı yükü ≥ 20 kg (EO/IR taret + görev bilgisayarı + araştırma yükü payı) | `payload_kg` | >= 20,000 kg | 20,000 | ✔ |
| R-04 | Servis tavanı (0,5 m/s) ≥ 4500 m, MTOM | `ceiling_service_m` | >= 4.500,000 m | 7.481,278 | ✔ |
| R-05 | Deniz seviyesi tırmanma hızı ≥ 4,0 m/s, MTOM, tam gaz (4,9 m/s karşılaştırma hedefi pervane sınırlı; bkz. pervane ödünleşimi) | `roc_sl_m_s` | >= 4,000 m/s | 4,623 | ✔ |
| R-06 | Kalkış yer koşusu ≤ 200 m (MTOM, ISA deniz seviyesi; 300 m pist / 1,5) | `takeoff_ground_roll_m` | <= 200,000 m | 190,975 | ✔ |
| R-07 | İniş yer koşusu ≤ 200 m (görev sonu kütlesi, ISA deniz seviyesi) | `landing_ground_roll_m` | <= 200,000 m | 190,329 | ✔ |
| R-08 | Tutunma hızı ≤ 24 m/s (temiz, trimli, MTOM, deniz seviyesi) | `vs_clean_sl_mtow_m_s` | <= 24,000 m/s | 23,906 | ✔ |
| R-09 | Statik marj ≥ %10 OAK (tüm yükleme durumları, öndeki nötr nokta) | `static_margin_min` | >= 0,100 - | 0,101 | ✔ |
| R-10 | Statik marj ≤ %30 OAK (tüm yükleme durumları) | `static_margin_max` | <= 0,300 - | 0,147 | ✔ |
| R-11 | Yön kararlılığı Cnβ ≥ 0,057 1/rad (0,001 1/derece) | `cn_beta_per_rad` | >= 0,057 1/rad | 0,059 | ✔ |
| R-12 | Geri devrilme açısı ≥ 15° (en arka zemin ağırlık merkezi) | `tipback_deg` | >= 15,000 deg | 37,151 | ✔ |
| R-13 | Yana devrilme açısı ≤ 55° | `turnover_deg` | <= 55,000 deg | 54,000 | ✔ |
| R-14 | Burun tekerleği yükü ≥ %8 (en arka ağırlık merkezi) | `nose_load_aft_cg` | >= 0,080 - | 0,155 | ✔ |
| R-15 | Burun tekerleği yükü ≤ %20 (en ön ağırlık merkezi) | `nose_load_fwd_cg` | <= 0,200 - | 0,164 | ✔ |
| R-16 | Pervane yer açıklığı ≥ 0,18 m: statik tutum, kalkış (yerden kesilme) tutumu ve teker koyma tutumunun en kritiği (MTOM; kalkışta amortisör statik çökmede, teker koymada yüksüz) | `prop_clear_min_925a_m` | >= 0,180 m | 0,185 | ✔ |
| R-17 | Sönük ana lastik + dibe oturmuş amortisörde pozitif pervane açıklığı (≥ 0,02 m) | `prop_clear_flat_tyre_m` | >= 0,020 m | 0,063 | ✔ |
| R-18 | Kuyruk tamponu pervaneden önce yere değer (açı farkı ≥ 1°) | `bumper_before_prop_margin_deg` | >= 1,000 deg | 4,724 | ✔ |
| R-19 | Kalkış ve flare açısında kuyruk tamponu yere değmez (pay ≥ 2°) | `bumper_rotation_margin_deg` | >= 2,000 deg | 2,299 | ✔ |
| R-20 | Teker koyma açısı ≥ 3° (ana tekerler önce değer) | `touchdown_attitude_deg` | >= 3,000 deg | 4,484 | ✔ |
| R-21 | Ana iniş takımı gövde içine toplanır (teker, bacak, mafsal; kanat kutusu ile çakışma yok) | `main_gear_stowed` | == 1,000 - | 1,000 | ✔ |
| R-22 | Burun iniş takımı omurga yuvasına toplanır | `nose_gear_stowed` | == 1,000 - | 1,000 | ✔ |
| R-23 | Taret içeride kapakların üstünde kalır (gömülü) | `turret_flush_margin_m` | >= 0,000 m | 0,026 | ✔ |
| R-24 | E180 büyüme zarfı (0,18 x 0,23 m) taret bölmesine sığar | `turret_growth_inside` | == 1,000 - | 1,000 | ✔ |
| R-25 | Taret dışarıda: nadirden en az −5° yükselime kadar her azimutta engelsiz görüş | `turret_fov_upper_min_deg` | >= -5,000 deg | -2,000 | ✔ |
| R-26 | Yerleşim bölgeleri gövdeye sığar (hata sayısı 0) | `packaging_zones_failed` | == 0,000 - | 0,000 | ✔ |
| R-27 | Yerleşim bölgeleri ve yakıt hücreleri çakışmaz (çakışma sayısı 0) | `zone_overlaps` | == 0,000 - | 0,000 | ✔ |
| R-28 | Yakıt hacmi gereken hacmi karşılar (pay ≥ 0) | `fuel_volume_margin` | >= 0,000 - | 0,290 | ✔ |
| R-29 | Taşıma: çıkarılabilir dış kanat paneli ≤ 3,4 m | `outer_panel_length_m` | <= 3,400 m | 2,780 | ✔ |
| R-30 | Gövde + LERX orta kesiti ≤ 2,0 m (tek parça taşıma) | `centre_section_width_m` | <= 2,000 m | 1,640 | ✔ |
| R-31 | Statik pervane uç Mach sayısı ≤ 0,75 | `prop_tip_mach_static` | <= 0,750 - | 0,712 | ✔ |
| R-32 | Jeneratör çıkışı beklemede en büyük sürekli elektrik yükünün ≥ 1,2 katı (E180 büyüme tareti ve araştırma yükü güç payı dahil; tepe yükler tampon bataryadan) | `generator_margin_loiter` | >= 1,200 - | 1,429 | ✔ |
| R-33 | Pervane uçları ile yapı arasında radyal açıklık ≥ 0,026 m (CS-VLA 925(c)(1)) | `prop_clear_radial_m` | >= 0,026 m | 0,123 | ✔ |
| R-34 | Pervane palaları ile sabit yapı (kaporta, kuyruk yüzeyleri) arasında boyuna açıklık ≥ 0,013 m (CS-VLA 925(c)(2)) | `prop_clear_longitudinal_m` | >= 0,013 m | 0,021 | ✔ |
| R-35 | Kalkışta dönme yetkisi: stabilatörler burun tekerini ana tekerler hâlâ yüklüyken kaldırır (tekerlek arabası etkisi yok; bütün MTOM yükleme durumları) | `takeoff_main_gear_load_at_rotation_N` | > 0,000 N | 103,119 | ✔ |
| R-36 | Stabilatör trim gereksinimi (yerel CL, η_t dahil; temiz/kalkış CLmax ve tam güçte yerden kesilme/pas geçme, en ön ağırlık merkezleri) ≤ 0,8 × stabilatör CLmax | `stab_trim_cl_local_max` | <= 0,720 - | 0,705 | ✔ |
| R-37 | Stabilatör eyleyicisi: tepe tork × bağlantı oranı ≥ 1,25 × en büyük menteşe momenti (CS-LUAS.395(a)(1)) | `stab_hinge_peak_margin` | >= 1,000 - | 1,771 | ✔ |
| R-38 | Stabilatör kök boşluğu ≤ 10 mm (sabit kök parçasına karşı, bütün sapma aralığında sabit) | `stab_root_gap_m` | <= 0,010 m | 0,008 | ✔ |
| R-39 | Stabilatör kökü bütün sapma aralığında gövdeye girmez (gövde yarı genişliği ile pay ≥ 5 mm) | `stab_root_body_clearance_m` | >= 0,005 m | 0,027 | ✔ |
| R-40 | Dikey kuyruk kökü bütün kök veteri boyunca gövdeye gömülü (boşluk ≤ 0) | `fin_root_max_gap_m` | <= 0,000 m | -0,021 | ✔ |
| R-41 | Pervane koruması: eğik dikeylerin firar kenarı pervane düzlemini disk ucundan ≥ 26 mm dışarıda keser | `prop_guard_fin_crossing_margin_m` | >= 0,026 m | 0,050 | ✔ |
| R-42 | Pervane koruması: eğik dikeylerin firar kenarı pervane düzlemini disk ucundan ≤ 80 mm dışarıda keser (diski yakından çevreler) | `prop_guard_fin_crossing_margin_m` | <= 0,080 m | 0,050 | ✔ |
| R-43 | Ana kiriş hattında OML derinliği (gövde yanından dış panel bağlantısına) ≥ birleşim kalınlığının %95'i | `spar_depth_main_ratio` | >= 0,950 - | 0,977 | ✔ |
| R-44 | Arka kiriş hattında OML derinliği (gövde yanından dış panel bağlantısına) ≥ dış panel bağlantı kesitinin arka kiriş derinliğinin %95'i (arka kiriş kademesiz devam eder) | `spar_depth_rear_ratio` | >= 0,950 - | 1,003 | ✔ |
| R-45 | Kanat kökü ile köşe çizgisi arasında basamak ≤ 10 mm (LERX/köşe çizgisi geçişi) | `wing_root_chine_step_m` | <= 0,010 m | 0,000 | ✔ |
| R-46 | Flap, sökülebilir dış panelin üzerinde kalır (iç ucu panel bağlantısının dışında) | `flap_inboard_end_outboard_of_joint_m` | >= 0,000 m | 0,030 | ✔ |

## 3. Kütle

| Grup | Kütle (kg) | Bütçe hedefi ± tolerans |
|---|---|---|
| wing | 17,27 | 17,27 ± 1,38 |
| systems | 15,07 | 15,07 ± 1,21 |
| propulsion | 14,30 | 14,30 ± 1,14 |
| gear | 13,66 | 13,66 ± 1,09 |
| chassis | 11,15 | 11,15 ± 0,89 |
| shell | 10,25 | 10,25 ± 0,82 |
| controls | 8,17 | 8,17 ± 0,65 |
| tail | 8,09 | 8,09 ± 0,65 |
| fuel | 2,34 | 2,34 ± 0,25 |
| hardware | 1,51 | 1,51 ± 0,25 |
| **boş (büyüme payı %5 dahil)** | **101,82** | |

Yükleme durumları (ağırlık merkezi, uçuş: takım ve taret içeride):

| Durum | Kütle (kg) | x_AM (m) | z_AM (m) | SM (% OAK) |
|---|---|---|---|---|
| mtow_design_payload_turret_retracted | 149,90 | 2,655 | 0,037 | 10,1 |
| mtow_design_payload_turret_extended | 149,90 | 2,655 | 0,036 | 10,1 |
| full_fuel_baseline_sensors_only | 133,15 | 2,654 | 0,058 | 10,4 |
| zero_fuel_design_payload | 121,82 | 2,638 | 0,028 | 13,7 |
| reserve_fuel_design_payload | 125,19 | 2,640 | 0,031 | 13,2 |
| minimum_flying_turret_only | 106,37 | 2,642 | 0,057 | 12,7 |
| e180_growth_turret_full_fuel | 149,90 | 2,633 | 0,034 | 14,7 |

## 4. Aerodinamik

* Kaldırma eğimi (girdap kafesi, gövde planı hariç): 5,815 1/rad; taşıma hattı 6,449 1/rad. CL0 = 0,529.
* Açıklık verimi: taşıma hattı 0,977, Trefftz 0,974; polar uyumu e = 0,839 (profil sürüklemesinin CL ile değişimi ve trim dahil).
* Kritik kesit: η = 0,38 (LERX dışı), α_stall = 8,1°; Re kök/uç 2.49e+06 / 4.29e+05.

Sürükleme kalemleri (temiz, CD = D/q / S_ref):

| Kalem | CD |
|---|---|
| body | 0,00741 |
| aft_body_upsweep | 0,00116 |
| tail_stabilator | 0,00240 |
| tail_stabilator_stub | 0,00040 |
| tail_fin | 0,00221 |
| tail_ventral | 0,00022 |
| landing_gear | 0,00122 |
| eo_ir_turret | 0,00009 |
| engine_cooling | 0,00188 |
| aft_closure_base | 0,00215 |
| wing_body_junctions | 0,00040 |
| antennas_pitot_lights_vents | 0,00077 |
| control_surface_gaps | 0,00080 |
| leakage_protuberance | 0,00106 |
| kanat profil sürüklemesi (CL 0,7; geçiş zorlanmış ×1,15) | 0,01375 |

Bekleme durumunda taret dışarıda: +0,00165 CD.

## 5. Kararlılık

* Nötr nokta: klasik 2,722 m, girdap kafesi 2,703 m → kullanılan (öndeki) 2,703 m.
* Taşıyıcı gövde (Multhopp) Cmα = 0,573 1/rad (ön gövde 0,451, arka 0,122); kanat-gövde AM 2,624 m, Cm0_wb -0,173.
* Kuyruk hacmi V_H 0,474, V_V 0,0276; aşağı sapma dε/dα 0,393.
* Cnβ: eğik dikeyler 0,0940, ventral 0,0043, gövde -0,0392 → toplam 0,0591 1/rad. Clβ -0,1164 1/rad.

## 6. İniş takımı ve taret

* Zemin z = -0,430 m; dingil açıklığı 2,332 m; iz 0,841 m; geri devrilme 37,2°; yana devrilme 54,0°; burun yükü %15,5 – %16,4.
* Statik zemin tutumu 2,5° burun yukarı (burun tekeri teması ana tekerlerin 0,102 m altında, gövde ekseninde).
* Pervane yer açıklığı (CS-VLA 925(a), MTOM): statik 0,210 m, yerden kesilme tutumu 3,8° (amortisör statik) 0,185 m, teker koyma tutumu 4,5° (takım yüksüz, +0,042 m) 0,214 m → en az 0,185 m; sönük lastik + dibe oturmuş amortisör 0,063 m. Pervane temas açısı 13,5°, kuyruk tamponu temas açısı 8,8°; flare 6,5°.
* CS-VLA 925(c): radyal açıklık en az 0,123 m, boyuna açıklık en az 0,021 m (pala ekseni boyutu uç 0,009 m; parçalara göre: body 0,022, stabilator 0,101, stabilator_stub 0,090, fin 0,021, ventral 0,151). Eğik dikeylerin firar kenarı pervane düzlemini 0,444 m yarıçapta keser (disk ucunun 50 mm dışı).
* Toplanmış takım: ana teker zarfı içeride evet, bacak evet, mafsal evet, merkez boşluğu 0,038 m, kanat kutusu açıklığı 0,035 m; burun takımı evet.
* Taret: içeride top kapak üstünde 0,026 m; E180 büyüme zarfı içeride evet; strok 0,120 m; görüş alanı üst sınırı en az -2,0° (ön ±60°: -2,0°).

## 7. Performans

| Durum | Değer |
|---|---|
| Kalkış (DS, MTOM, belirleyici yükleme: E180 taret, tam yakıt) | koşu 191 m, 15 m'ye 332 m; burun kaldırma V_R 25,3 m/s (ana tekerlerde 103 N), V_LOF 26,7 m/s (VS_TO 22,2 m/s, flap 35°) |
| — MTOM, tasarım yükü | koşu 179 m, V_R 24,6, V_LOF 26,1 m/s, x_AM 2,651 m |
| — MTOM, taret dışarıda | koşu 179 m, V_R 24,6, V_LOF 26,1 m/s, x_AM 2,651 m |
| — E180 taret, tam yakıt | koşu 191 m, V_R 25,3, V_LOF 26,7 m/s, x_AM 2,629 m |
| Kalkış (1500 m ISA) | koşu 279 m |
| İniş (görev sonu 125,2 kg) | koşu 190 m, V_TD 25,1 m/s |
| 0 m | VS 23,9 m/s, Vmaks 54,3 m/s, tırmanma 4,62 m/s @ 30,0 m/s |
| 3000 m | VS 27,8 m/s, Vmaks 52,0 m/s, tırmanma 2,66 m/s @ 31,9 m/s |
| Tavan | servis 7.481 m, mutlak 8.677 m |

Faydalı yük – dayanım:

| Yük (kg) | Yakıt (kg) | Dayanım (h) |
|---|---|---|
| 0,00 | 36,22 (hacim sınırı) | 13,98 |
| 3,25 | 36,22 (hacim sınırı) | 13,73 |
| 10,00 | 36,22 (hacim sınırı) | 13,14 |
| 15,00 | 33,08 | 11,65 |
| 20,00 | 28,08 | 9,59 |
| 25,00 | 23,08 | 7,58 |

Duyarlılıklar (dayanım, h): bsfc_minus_12pct 10,91, bsfc_plus_12pct 8,55, cd0_plus_10pct_all_drag 8,99, empty_plus_5pct 7,55, loiter_at_1000m 10,04, no_transit_loiter_only 9,56, turret_retracted_whole_mission 9,80

## 8. Kumanda yüzeyleri, kökler, kiriş derinliği, elektrik, yükler

* Stabilatör mili panel OAK'ının %20'sinde (x = 3,710 m); AM belirsizliği %22–%30 OAK → kol 9–43 mm. Menteşe momenti VA'da 14,5 N·m, VD'de 7,7 N·m, sürekli trim 6,4 N·m; Volz DA 30 × 2,0:1 bağlantı → tepe pay 1,77 (1,25 katsayısı dahil), sürekli pay 2,49; yüzey hareketi ±22,5°.
* Stabilatör kökü: sabit kök parçasına boşluk 8,0 mm (bütün sapma aralığında sabit), gövdeye en az pay 27,0 mm (-20°…15°). Dikey kuyruk kökü bütün veter boyunca en az 21,3 mm gömülü.
* Kiriş derinliği (gövde yanı → dış panel bağlantısı): ana kiriş en az 85,4 mm (gerekli 87,4 mm), arka kiriş en az 38,7 mm (gerekli 38,6 mm = bağlantı kesitinin kendi arka kiriş derinliği).
* Elektrik (R-32): jeneratör beklemede 533 W; sürekli yük HD59 ile 353 W, E180 ile 373 W (araştırma yükü payı dahil) → pay 1,43; E180 tepe 493 W jeneratörden karşılanır.
* Rüzgâr hamlesi matrisi (yapılandırma CLα 6,373 1/rad): 106,4 kg / 0 m: +6,62 / -4,62; 106,4 kg / 3000 m: +6,97 / -4,97; 106,4 kg / 4500 m: +7,13 / -5,13; 149,9 kg / 0 m: +5,27 / -3,27; 149,9 kg / 3000 m: +5,47 / -3,47; 149,9 kg / 4500 m: +5,56 / -3,56. Kanat tasarım yük katsayısı 5,56 (en büyük n·m·g, MTOM); kök momenti (nihai) 9.448 N·m.

## 9. Yerleşim ve hacimler

| Bölge | Sığıyor |
|---|---|
| avionics_power_bay | evet |
| parachute_bay | evet |
| mission_computer | evet |
| equipment_bay_aft | evet |
| wing_carry_through | evet |
| payload_bay | evet |
| engine_cylinder_slab | evet |
| engine_intake_box | evet |
| engine_crankcase_sg750 | evet |
| main_gear_wells | evet |
| nose_gear_well | evet |
| turret_bay | evet |
| overlaps | evet |

Yakıt hacmi: gereken 38,7 L, kullanılabilir 49,9 L (tank verimi dahil); yakıt AM'si x = 2,728 m.

## 10. Ödünleşimler

| Pervane (her biri kendi kapanışıyla) | Dayanım (h) | Tırmanma DS (m/s) | Kalkış / iniş koşusu (m) | Statik itki (N) | Uç Mach | Karşılanmayan gereksinimler |
|---|---|---|---|---|---|---|
| Mejzlik 31x12 3B (tasarım) | 9,59 | 4,62 | 191 / 190 | 493 | 0,712 | R-02 |
| Mejzlik 32x18 2B | 9,93 | 3,91 | 216 / 190 | 447 | 0,714 | R-02, R-05, R-06, R-21, R-26 |

Dayanımın sürüklemeye duyarlılığı: her uçuş evresinde +0,001 CD → -0,115 h.

İniş takımı: içeri katlanır 9,59 h (13,66 kg) – sabit kaportalı 11,09 h (9,55 kg).
Taret: geri çekilir 9,59 h – sabit (sürekli dışarıda) 10,26 h.

| Açıklık (m) | VS hedefi (m/s) | S (m²) | AR | Kanat (kg) | Boş (kg) | Dayanım (h) | Kalkış (m) |
|---|---|---|---|---|---|---|---|
| 7,20 (tasarım) | 23,9 | 3,226 | 16,1 | 17,27 | 101,82 | 9,59 | 191 |
| 6,80 | 23,9 | 3,231 | 14,3 | 16,65 | 101,27 | 9,56 | 197 |
| 7,60 | 23,9 | 3,227 | 17,9 | 17,94 | 102,46 | 9,53 | 185 |
| 7,20 | 23,0 | 3,477 | 14,9 | 18,09 | 103,37 | 9,07 | 178 |
| 7,20 | 24,5 | 3,074 | 16,9 | 16,79 | 101,10 | 9,73 | 199 |

| Gövde uzatma (m) | Boy (m) | V_H | Boş (kg) | Dayanım (h) | Cnβ |
|---|---|---|---|---|---|
| -0,10 | 3,92 | 0,469 | 101,80 | 9,57 | 0,0586 |
| 0,10 | 4,12 | 0,483 | 101,86 | 9,59 | 0,0577 |

## Şekiller

* `ucav250/docs/fig/yk250_constraint.png`
* `ucav250/docs/fig/yk250_polars.png`
* `ucav250/docs/fig/yk250_cg_envelope.png`
* `ucav250/docs/fig/yk250_payload_endurance.png`
* `ucav250/docs/fig/yk250_vn.png`
* `ucav250/docs/fig/yk250_3view.png`
