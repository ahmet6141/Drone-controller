# YK-250 kütle özellikleri

Boş kütle (faydalı yük ve yakıt hariç; kayıt defteri parçaları + kalemlendirilmiş paylar): **108,33 kg**, AG x = 2,633 m uçuş duruşunda (takım yukarıda, kapaklar kapalı); yerde (takım aşağıda, kapaklar açık) x = 2,629 m (kayıt defteri parçaları yalnız: 106,60 kg, 2436 parça).

Uçuş durumları kayıt defterini `gear_retraction` dizisinin son durumunda (takım yukarıda) ve taret açık durumlarda ayrıca `turret_extension` son durumunda konumlar; yakıt, yakıt modülünün seviye tablosundaki (üç hücrede ortak serbest yüzey) ağırlık merkezinde durur.

## Yükleme durumları

Her durum sizing.mass_cases ile aynı yüklemeyi taşır: kayıt defteri parçaları (yakıt içeriği durumun yakıt oranıyla), kalemlendirilmiş boş kütle payları ve durumun faydalı yük kalemleri (kayıt defterinde parçası olmayanlar — görev bilgisayarı, tepsi, araştırma yükü payı, E180 farkı — kendi spec konumlarında noktasal kütle).

| Durum | Kütle (kg) | Noktasal faydalı yük (kg) | AG x (m) | AG y (mm) | AG z (m) | %OAK (MAC) | Ixx | Iyy | Izz (kg·m²) |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| mtow_design_payload_turret_retracted | 150,07 | 9,76 | 2,6064 | -0,8 | 0,0258 | 5,8 | 68,3 | 123,0 | 183,8 |
| mtow_design_payload_turret_extended | 150,07 | 9,76 | 2,6064 | -0,8 | 0,0241 | 5,8 | 68,4 | 123,1 | 183,8 |
| mtow_max_payload_turret_retracted | 150,07 | 18,26 | 2,6072 | -0,8 | 0,0152 | 6,0 | 68,2 | 122,5 | 182,8 |
| mtow_max_payload_turret_extended | 150,07 | 18,26 | 2,6072 | -0,8 | 0,0135 | 6,0 | 68,3 | 122,6 | 182,8 |
| full_fuel_baseline_sensors_only | 147,43 | 1,51 | 2,6057 | -0,8 | 0,0369 | 5,7 | 68,3 | 123,2 | 184,4 |
| zero_fuel_design_payload | 119,83 | 9,76 | 2,6066 | -1,0 | 0,0278 | 5,9 | 67,3 | 120,5 | 180,2 |
| zero_fuel_max_payload | 128,33 | 18,26 | 2,6082 | -0,9 | 0,0174 | 6,2 | 67,5 | 120,7 | 180,2 |
| reserve_fuel_design_payload | 123,46 | 9,76 | 2,6080 | -1,0 | 0,0276 | 6,2 | 67,3 | 120,8 | 180,7 |
| minimum_flying_turret_only | 113,11 | 0,01 | 2,6121 | -1,0 | 0,0410 | 7,0 | 67,1 | 120,1 | 180,1 |
| e180_growth_turret_full_fuel | 150,07 | 9,76 | 2,5853 | -0,8 | 0,0226 | 1,5 | 68,5 | 127,4 | 188,0 |

## Gruplar

| Grup | Kütle (kg) | AG x (m) |
|---|---:|---:|
| chassis | 19,82 | 2,510 |
| systems | 15,32 | 1,441 |
| propulsion | 12,97 | 3,628 |
| gear | 12,94 | 2,459 |
| wing | 12,88 | 2,770 |
| shell | 10,82 | 2,260 |
| controls | 8,15 | 3,040 |
| tail | 7,74 | 3,782 |
| hardware | 5,34 | 2,573 |
| fuel | 2,37 | 2,832 |
| payload | 1,74 | 1,220 |

## Kalemlendirilmiş boş kütle payları (geometride modellenmeyen; spec.mass.items registry_allowance)

| Kalem | Grup | Kütle (kg) | x (m) |
|---|---|---:|---:|
| paint_finish_body | shell | 0,611 | 2,300 |
| paint_finish_wing | wing | 0,585 | 2,772 |
| paint_finish_tail | tail | 0,308 | 3,759 |
| gear_door_perimeter_seals | gear | 0,101 | 2,519 |
| brake_lines_fluid | gear | 0,135 | 1,949 |

## Kütle bütçesi karşılaştırması

| Grup | Hesap (kg) | Bütçe (kg) | Tolerans | Durum |
|---|---:|---:|---:|---|
| chassis | 19,82 | 19,86 | ±0,60 | ✓ |
| controls | 8,15 | 8,15 | ±0,24 | ✓ |
| fuel | 2,37 | 2,37 | ±0,07 | ✓ |
| gear | 12,94 | 12,94 | ±0,39 | ✓ |
| hardware | 5,34 | 5,26 | ±0,16 | ✓ |
| propulsion | 12,97 | 12,97 | ±0,39 | ✓ |
| shell | 10,82 | 10,82 | ±0,32 | ✓ |
| systems | 15,32 | 15,21 | ±0,46 | ✓ |
| tail | 7,74 | 7,74 | ±0,23 | ✓ |
| wing | 12,88 | 12,88 | ±0,39 | ✓ |

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
| YK250-FU-596 | 15,787 |
| YK250-FU-598 | 11,938 |
| YK250-PR-500 | 6,533 |
| YK250-SY-800 | 4,412 |
| YK250-CH-001 | 2,550 |
| YK250-FU-597 | 2,506 |
| YK250-SY-726 | 1,980 |
| YK250-SY-720 | 1,600 |
| YK250-PL-820 | 1,550 |
| YK250-LG-622-R | 1,390 |
| YK250-LG-622-L | 1,390 |
| YK250-WG-151-R | 1,285 |
| YK250-WG-151-L | 1,285 |
| YK250-TL-256-R | 1,069 |
| YK250-TL-256-L | 1,069 |
| YK250-WG-150-R | 1,063 |
| YK250-WG-150-L | 1,063 |
| YK250-WG-153-R | 0,985 |
| YK250-WG-153-L | 0,985 |
| YK250-CH-085 | 0,956 |
| YK250-LG-626 | 0,895 |
| YK250-WG-154-R | 0,862 |
| YK250-WG-154-L | 0,862 |
| YK250-PR-510 | 0,800 |
| YK250-SH-383 | 0,781 |

## Sorunlar

- zero_fuel_max_payload: CG x 2.6082 outside [2.58434, 2.60779]
- reserve_fuel_design_payload: CG x 2.6080 outside [2.58434, 2.60779]
- minimum_flying_turret_only: CG x 2.6121 outside [2.58434, 2.60779]
- mtow_design_payload_turret_retracted: 150.07 kg exceeds spec mtow 149.9
- mtow_design_payload_turret_extended: 150.07 kg exceeds spec mtow 149.9
- mtow_max_payload_turret_retracted: 150.07 kg exceeds spec mtow 149.9
- mtow_max_payload_turret_extended: 150.07 kg exceeds spec mtow 149.9
- e180_growth_turret_full_fuel: 150.07 kg exceeds spec mtow 149.9
- spec.mass.registry groups.chassis stale: 19.857 kg [2.5062, 0.0004, 0.0104] vs registry 19.822 kg [2.5099, 0.0004, 0.0104]
- spec.mass.registry groups.controls stale: 8.148 kg [3.0373, -0.004, 0.1488] vs registry 8.148 kg [3.0397, -0.004, 0.1541]
- spec.mass.registry groups.fuel stale: 2.366 kg [2.8324, 0.0105, 0.0168] vs registry 2.366 kg [2.8324, 0.0105, 0.0168]
- spec.mass.registry groups.gear stale: 12.704 kg [2.4288, -0.0009, -0.2476] vs registry 12.705 kg [2.464, -0.001, -0.1278]
- spec.mass.registry groups.hardware stale: 5.253 kg [2.5741, -0.0004, 0.0295] vs registry 5.339 kg [2.5725, -0.0004, 0.0332]
- spec.mass.registry groups.propulsion stale: 12.967 kg [3.6276, -0.0009, 0.1351] vs registry 12.967 kg [3.6276, -0.0009, 0.1351]
- spec.mass.registry groups.shell stale: 10.207 kg [2.2578, 0.0004, -0.0032] vs registry 10.206 kg [2.2578, 0.0004, -0.0032]
- spec.mass.registry groups.systems stale: 15.208 kg [1.4454, -0.0044, -0.0075] vs registry 15.323 kg [1.4413, -0.0036, -0.0076]
- spec.mass.registry groups.tail stale: 7.428 kg [3.7824, -0.0001, 0.3345] vs registry 7.428 kg [3.7824, -0.0001, 0.3345]
- spec.mass.registry groups.wing stale: 12.290 kg [2.7697, -0.0033, 0.0524] vs registry 12.290 kg [2.7697, -0.0033, 0.0524]
- spec.mass.registry fuel stale: 30.231 kg [2.6122, 0.0, 0.0115] vs registry 30.231 kg [2.6056, 0.0, 0.0096]
- spec.mass.registry fuel.levels stale
- spec.mass.registry payload_parts.eo_ir_turret_hd59_mount stale: 1.736 kg [1.22, 0.0, -0.065] vs registry 1.736 kg [1.22, -0.0, -0.065]
- spec.mass.registry refs.exhaust_pipes_silencers missing
- spec.mass.registry refs.main_gear_retraction_emas missing
- spec.mass.registry refs.nose_gear_retraction_ema missing
