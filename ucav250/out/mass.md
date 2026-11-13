# YK-250 kütle özellikleri

Boş kütle (faydalı yük ve yakıt hariç; kayıt defteri parçaları + kalemlendirilmiş paylar): **108,21 kg**, AG x = 2,629 m (kayıt defteri parçaları yalnız: 106,48 kg, 2468 parça).

## Yükleme durumları

Her durum sizing.mass_cases ile aynı yüklemeyi taşır: kayıt defteri parçaları (yakıt içeriği durumun yakıt oranıyla), kalemlendirilmiş boş kütle payları ve durumun faydalı yük kalemleri (kayıt defterinde parçası olmayanlar — görev bilgisayarı, tepsi, araştırma yükü payı, E180 farkı — kendi spec konumlarında noktasal kütle).

| Durum | Kütle (kg) | Noktasal faydalı yük (kg) | AG x (m) | AG y (mm) | AG z (m) | %OAK (MAC) | Ixx | Iyy | Izz (kg·m²) |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| mtow_design_payload_turret_retracted | 149,92 | 9,76 | 2,6049 | -0,9 | 0,0153 | 5,6 | 69,8 | 125,0 | 185,6 |
| mtow_design_payload_turret_extended | 149,92 | 9,76 | 2,6049 | -0,9 | 0,0153 | 5,6 | 69,8 | 125,0 | 185,6 |
| mtow_max_payload_turret_retracted | 149,92 | 18,26 | 2,6058 | -0,9 | 0,0074 | 5,8 | 69,6 | 124,5 | 184,6 |
| mtow_max_payload_turret_extended | 149,92 | 18,26 | 2,6058 | -0,9 | 0,0073 | 5,8 | 69,6 | 124,5 | 184,6 |
| full_fuel_baseline_sensors_only | 147,31 | 1,51 | 2,6038 | -0,9 | 0,0232 | 5,4 | 69,8 | 125,3 | 186,2 |
| zero_fuel_design_payload | 119,71 | 9,76 | 2,6031 | -1,1 | 0,0163 | 5,2 | 68,7 | 122,7 | 182,2 |
| zero_fuel_max_payload | 128,21 | 18,26 | 2,6048 | -1,0 | 0,0066 | 5,6 | 68,9 | 122,8 | 182,2 |
| reserve_fuel_design_payload | 123,34 | 9,76 | 2,6034 | -1,0 | 0,0161 | 5,3 | 68,8 | 122,9 | 182,6 |
| minimum_flying_turret_only | 112,98 | 0,01 | 2,6071 | -1,1 | 0,0282 | 6,1 | 68,6 | 122,3 | 182,1 |
| e180_growth_turret_full_fuel | 149,92 | 9,76 | 2,5837 | -0,9 | 0,0138 | 1,2 | 69,9 | 129,3 | 189,8 |

## Gruplar

| Grup | Kütle (kg) | AG x (m) |
|---|---:|---:|
| chassis | 19,67 | 2,512 |
| systems | 15,21 | 1,445 |
| propulsion | 12,97 | 3,628 |
| gear | 12,91 | 2,423 |
| wing | 12,87 | 2,769 |
| shell | 10,82 | 2,260 |
| controls | 8,15 | 3,037 |
| tail | 7,74 | 3,781 |
| hardware | 5,52 | 2,543 |
| fuel | 2,37 | 2,832 |
| payload | 1,74 | 1,220 |

## Kalemlendirilmiş boş kütle payları (geometride modellenmeyen; spec.mass.items registry_allowance)

| Kalem | Grup | Kütle (kg) | x (m) |
|---|---|---:|---:|
| paint_finish_body | shell | 0,611 | 2,300 |
| paint_finish_wing | wing | 0,585 | 2,771 |
| paint_finish_tail | tail | 0,308 | 3,759 |
| gear_door_perimeter_seals | gear | 0,099 | 2,516 |
| brake_lines_fluid | gear | 0,135 | 1,948 |

## Kütle bütçesi karşılaştırması

| Grup | Hesap (kg) | Bütçe (kg) | Tolerans | Durum |
|---|---:|---:|---:|---|
| chassis | 19,67 | 12,61 | ±0,37 | ✗ |
| controls | 8,15 | 8,29 | ±0,25 | ✓ |
| fuel | 2,37 | 2,35 | ±0,07 | ✓ |
| gear | 12,91 | 14,57 | ±0,44 | ✗ |
| hardware | 5,52 | 1,51 | ±0,05 | ✗ |
| propulsion | 12,97 | 14,54 | ±0,44 | ✗ |
| shell | 10,82 | 10,48 | ±0,31 | ✗ |
| systems | 15,21 | 14,79 | ±0,44 | ✓ |
| tail | 7,74 | 7,35 | ±0,22 | ✗ |
| wing | 12,87 | 15,18 | ±0,46 | ✗ |

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
| YK250-FU-596 | 14,281 |
| YK250-FU-598 | 10,995 |
| YK250-PR-500 | 6,533 |
| YK250-FU-597 | 4,928 |
| YK250-SY-800 | 4,412 |
| YK250-CH-001 | 2,544 |
| YK250-SY-726 | 1,980 |
| YK250-SY-720 | 1,600 |
| YK250-PL-820 | 1,550 |
| YK250-LG-622-R | 1,390 |
| YK250-LG-622-L | 1,390 |
| YK250-WG-151-R | 1,283 |
| YK250-WG-151-L | 1,283 |
| YK250-TL-256-L | 1,069 |
| YK250-TL-256-R | 1,069 |
| YK250-WG-150-R | 1,063 |
| YK250-WG-150-L | 1,063 |
| YK250-WG-153-R | 0,985 |
| YK250-WG-153-L | 0,985 |
| YK250-CH-085 | 0,956 |
| YK250-LG-626 | 0,895 |
| YK250-WG-154-R | 0,862 |
| YK250-WG-154-L | 0,862 |
| YK250-PR-510 | 0,800 |
| YK250-SH-383 | 0,780 |

## Sorunlar

- minimum_flying_turret_only: CG x 2.6071 outside [2.58336, 2.60677]
- budget chassis: 19.67 kg vs 12.61 ± 0.37
- budget gear: 12.91 kg vs 14.57 ± 0.44
- budget hardware: 5.52 kg vs 1.51 ± 0.05
- budget propulsion: 12.97 kg vs 14.54 ± 0.44
- budget shell: 10.82 kg vs 10.48 ± 0.31
- budget tail: 7.74 kg vs 7.35 ± 0.22
- budget wing: 12.87 kg vs 15.18 ± 0.46
- mtow_design_payload_turret_retracted: 149.92 kg exceeds spec mtow 149.9
- mtow_design_payload_turret_extended: 149.92 kg exceeds spec mtow 149.9
- mtow_max_payload_turret_retracted: 149.92 kg exceeds spec mtow 149.9
- mtow_max_payload_turret_extended: 149.92 kg exceeds spec mtow 149.9
- e180_growth_turret_full_fuel: 149.92 kg exceeds spec mtow 149.9
- spec.mass.registry groups.tail stale: 7.406 kg [3.7823, -0.0001, 0.3349] vs registry 7.429 kg [3.7824, -0.0001, 0.3345]
- spec.mass.registry fuel stale: 30.140 kg [2.6113, 0.0, 0.0114] vs registry 30.204 kg [2.6117, 0.0, 0.0115]
