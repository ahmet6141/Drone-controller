# YK-250 kütle özellikleri

Boş kütle (faydalı yük ve yakıt hariç; kayıt defteri parçaları + kalemlendirilmiş paylar): **110,71 kg**, AG x = 2,626 m uçuş duruşunda (takım yukarıda, kapaklar kapalı); yerde (takım aşağıda, kapaklar açık) x = 2,622 m (kayıt defteri parçaları yalnız: 106,66 kg, 2440 parça).

Uçuş durumları kayıt defterini `gear_retraction` dizisinin son durumunda (takım yukarıda) ve taret açık durumlarda ayrıca `turret_extension` son durumunda konumlar; yakıt, yakıt modülünün seviye tablosundaki (üç hücrede ortak serbest yüzey) ağırlık merkezinde durur.

## Yükleme durumları

Her durum sizing.mass_cases ile aynı yüklemeyi taşır: kayıt defteri parçaları (yakıt içeriği durumun yakıt oranıyla), kalemlendirilmiş boş kütle payları ve durumun faydalı yük kalemleri (kayıt defterinde parçası olmayanlar — görev bilgisayarı, tepsi, araştırma yükü payı, E180 farkı — kendi spec konumlarında noktasal kütle).

| Durum | Kütle (kg) | Noktasal faydalı yük (kg) | AG x (m) | AG y (mm) | AG z (m) | %OAK (MAC) | Ixx | Iyy | Izz (kg·m²) |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| mtow_design_payload_turret_retracted | 149,90 | 7,26 | 2,6013 | -0,7 | 0,0275 | 4,9 | 68,4 | 127,1 | 188,0 |
| mtow_design_payload_turret_extended | 149,90 | 7,26 | 2,6013 | -0,7 | 0,0258 | 4,9 | 68,5 | 127,1 | 188,0 |
| mtow_max_payload_turret_retracted | 149,90 | 18,26 | 2,6025 | -0,7 | 0,0142 | 5,2 | 68,3 | 126,4 | 186,7 |
| mtow_max_payload_turret_extended | 149,90 | 18,26 | 2,6025 | -0,7 | 0,0124 | 5,2 | 68,4 | 126,5 | 186,7 |
| full_fuel_baseline_sensors_only | 149,81 | 1,51 | 2,6012 | -0,7 | 0,0359 | 4,9 | 68,5 | 127,4 | 188,7 |
| zero_fuel_design_payload | 119,71 | 7,26 | 2,6002 | -0,8 | 0,0299 | 4,7 | 67,4 | 124,6 | 184,5 |
| zero_fuel_max_payload | 130,71 | 18,26 | 2,6024 | -0,8 | 0,0165 | 5,2 | 67,7 | 124,8 | 184,5 |
| reserve_fuel_design_payload | 123,33 | 7,26 | 2,6018 | -0,8 | 0,0297 | 5,0 | 67,5 | 124,8 | 184,9 |
| minimum_flying_turret_only | 115,48 | 0,01 | 2,6061 | -0,9 | 0,0395 | 5,9 | 67,3 | 124,2 | 184,4 |
| e180_growth_turret_full_fuel | 149,90 | 7,26 | 2,5802 | -0,7 | 0,0243 | 0,6 | 68,6 | 131,4 | 192,3 |

## sizing.mass_cases ile karşılaştırma (öz denetim)

| Durum | Δx (mm) | Δz (mm) | Iyy kayıt defteri (kg·m²) |
|---|---:|---:|---:|
| mtow_design_payload_turret_retracted | 0,03 | -0,01 | 127,1 |
| mtow_design_payload_turret_extended | 0,03 | -0,01 | 127,1 |
| mtow_max_payload_turret_retracted | 0,03 | -0,01 | 126,4 |
| mtow_max_payload_turret_extended | 0,03 | -0,01 | 126,5 |
| full_fuel_baseline_sensors_only | 0,03 | -0,01 | 127,4 |
| zero_fuel_design_payload | 0,04 | -0,01 | 124,6 |
| zero_fuel_max_payload | 0,03 | -0,01 | 124,8 |
| reserve_fuel_design_payload | 0,03 | -0,01 | 124,8 |
| minimum_flying_turret_only | 0,04 | -0,01 | 124,2 |
| e180_growth_turret_full_fuel | 0,03 | -0,01 | 131,4 |

## Gruplar

| Grup | Kütle (kg) | AG x (m) |
|---|---:|---:|
| chassis | 19,82 | 2,509 |
| systems | 15,32 | 1,441 |
| gear | 14,49 | 2,375 |
| propulsion | 13,57 | 3,638 |
| wing | 12,91 | 2,770 |
| shell | 10,97 | 2,266 |
| controls | 8,16 | 3,040 |
| tail | 7,73 | 3,781 |
| hardware | 5,36 | 2,574 |
| fuel | 2,37 | 2,832 |
| payload | 1,74 | 1,220 |

## Kalemlendirilmiş boş kütle payları (geometride modellenmeyen; spec.mass.items registry_allowance)

| Kalem | Grup | Kütle (kg) | x (m) |
|---|---|---:|---:|
| paint_finish_body | shell | 0,611 | 2,300 |
| paint_finish_wing | wing | 0,587 | 2,772 |
| paint_finish_tail | tail | 0,308 | 3,759 |
| gear_door_perimeter_seals | gear | 0,101 | 2,517 |
| brake_lines_fluid | gear | 0,135 | 1,947 |
| exhaust_silencer_internals | propulsion | 0,601 | 3,861 |
| main_gear_retraction_emas_unit_delta | gear | 0,720 | 2,888 |
| nose_gear_retraction_ema_unit_delta | gear | 0,830 | 0,666 |
| fuel_panel_gaskets | shell | 0,153 | 2,640 |

## Kütle bütçesi karşılaştırması

| Grup | Hesap (kg) | Bütçe (kg) | Tolerans | Durum |
|---|---:|---:|---:|---|
| chassis | 19,82 | 19,82 | ±0,59 | ✓ |
| controls | 8,16 | 8,16 | ±0,24 | ✓ |
| fuel | 2,37 | 2,37 | ±0,07 | ✓ |
| gear | 14,49 | 14,49 | ±0,43 | ✓ |
| hardware | 5,36 | 5,37 | ±0,16 | ✓ |
| propulsion | 13,57 | 13,57 | ±0,41 | ✓ |
| shell | 10,97 | 10,98 | ±0,33 | ✓ |
| systems | 15,32 | 15,33 | ±0,46 | ✓ |
| tail | 7,73 | 7,74 | ±0,23 | ✓ |
| wing | 12,91 | 12,92 | ±0,39 | ✓ |

## Başka bütçe grubuna yazılan parçalar (layout / üretici mass_item)

| Parça | Üretici grubu | Bütçe grubu | Kütle kalemi | Kütle (kg) |
|---|---|---|---|---:|
| YK250-FU-584 | fuel | propulsion | engine_group_installed | 0,350 |
| YK250-PL-822 | payload | systems | turret_lift_mechanism_doors | 0,138 |
| YK250-PL-823 | payload | systems | turret_lift_mechanism_doors | 0,040 |
| YK250-PL-824 | payload | systems | turret_lift_mechanism_doors | 0,200 |
| YK250-PL-825 | payload | systems | turret_lift_mechanism_doors | 0,085 |
| YK250-PL-826-L | payload | systems | turret_lift_mechanism_doors | 0,057 |
| YK250-PL-826-R | payload | systems | turret_lift_mechanism_doors | 0,057 |
| YK250-PL-827 | payload | systems | turret_lift_mechanism_doors | 0,057 |
| YK250-PL-828 | payload | systems | turret_lift_mechanism_doors | 0,147 |
| YK250-PL-829 | payload | systems | turret_lift_mechanism_doors | 0,147 |
| YK250-PL-830-L | payload | systems | turret_lift_mechanism_doors | 0,132 |
| YK250-PL-830-R | payload | systems | turret_lift_mechanism_doors | 0,132 |
| YK250-PL-831-L | payload | systems | turret_lift_mechanism_doors | 0,037 |
| YK250-PL-831-R | payload | systems | turret_lift_mechanism_doors | 0,037 |
| YK250-PL-832 | payload | systems | turret_lift_mechanism_doors | 0,039 |
| YK250-PL-833 | payload | systems | turret_lift_mechanism_doors | 0,030 |
| YK250-PL-834-L | payload | systems | turret_lift_mechanism_doors | 0,018 |
| YK250-PL-834-R | payload | systems | turret_lift_mechanism_doors | 0,018 |
| YK250-PL-835-L | payload | systems | turret_lift_mechanism_doors | 0,020 |
| YK250-PL-835-R | payload | systems | turret_lift_mechanism_doors | 0,020 |
| YK250-PL-836-L | payload | systems | turret_lift_mechanism_doors | 0,036 |
| YK250-PL-836-R | payload | systems | turret_lift_mechanism_doors | 0,036 |
| YK250-PL-837-L | payload | systems | turret_lift_mechanism_doors | 0,044 |
| YK250-PL-837-R | payload | systems | turret_lift_mechanism_doors | 0,044 |
| YK250-PL-838-L | payload | systems | turret_lift_mechanism_doors | 0,038 |
| YK250-PL-838-R | payload | systems | turret_lift_mechanism_doors | 0,038 |
| YK250-PL-839 | payload | systems | turret_lift_mechanism_doors | 0,034 |
| YK250-PL-840 | payload | systems | turret_lift_mechanism_doors | 0,034 |
| YK250-PL-841 | payload | systems | turret_lift_mechanism_doors | 0,034 |
| YK250-PL-842 | payload | systems | turret_lift_mechanism_doors | 0,034 |
| YK250-PL-847 | payload | systems | turret_lift_mechanism_doors | 0,012 |
| YK250-PL-848 | payload | systems | turret_lift_mechanism_doors | 0,016 |
| YK250-PL-849 | payload | systems | turret_lift_mechanism_doors | 0,060 |
| YK250-SH-363 | shell | systems | turret_lift_mechanism_doors | 0,155 |

## En ağır 25 parça

| Parça | Kütle (kg) |
|---|---:|
| YK250-FU-596 | 15,778 |
| YK250-FU-598 | 11,927 |
| YK250-PR-500 | 6,533 |
| YK250-SY-800 | 4,412 |
| YK250-CH-001 | 2,551 |
| YK250-FU-597 | 2,489 |
| YK250-SY-726 | 1,980 |
| YK250-SY-720 | 1,600 |
| YK250-PL-820 | 1,550 |
| YK250-LG-622-R | 1,390 |
| YK250-LG-622-L | 1,390 |
| YK250-WG-151-R | 1,284 |
| YK250-WG-151-L | 1,284 |
| YK250-TL-256-R | 1,068 |
| YK250-TL-256-L | 1,068 |
| YK250-WG-150-R | 1,067 |
| YK250-WG-150-L | 1,067 |
| YK250-WG-153-R | 0,989 |
| YK250-WG-153-L | 0,989 |
| YK250-CH-085 | 0,956 |
| YK250-LG-626 | 0,895 |
| YK250-WG-154-R | 0,865 |
| YK250-WG-154-L | 0,865 |
| YK250-PR-510 | 0,800 |
| YK250-SH-383 | 0,775 |
