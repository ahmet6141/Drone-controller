# YK-250 kütle özellikleri

Boş kütle (faydalı yük ve yakıt hariç; kayıt defteri parçaları + kalemlendirilmiş paylar): **108,20 kg**, AG x = 2,629 m (kayıt defteri parçaları yalnız: 106,46 kg, 2468 parça).

## Yükleme durumları

Her durum sizing.mass_cases ile aynı yüklemeyi taşır: kayıt defteri parçaları (yakıt içeriği durumun yakıt oranıyla), kalemlendirilmiş boş kütle payları ve durumun faydalı yük kalemleri (kayıt defterinde parçası olmayanlar — görev bilgisayarı, tepsi, araştırma yükü payı, E180 farkı — kendi spec konumlarında noktasal kütle).

| Durum | Kütle (kg) | Noktasal faydalı yük (kg) | AG x (m) | AG y (mm) | AG z (m) | %OAK (MAC) | Ixx | Iyy | Izz (kg·m²) |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| mtow_design_payload_turret_retracted | 149,84 | 9,76 | 2,6045 | -0,9 | 0,0152 | 5,6 | 69,8 | 125,0 | 185,5 |
| mtow_design_payload_turret_extended | 149,84 | 9,76 | 2,6045 | -0,9 | 0,0152 | 5,6 | 69,8 | 125,0 | 185,5 |
| mtow_max_payload_turret_retracted | 149,84 | 18,26 | 2,6055 | -0,9 | 0,0073 | 5,8 | 69,6 | 124,5 | 184,6 |
| mtow_max_payload_turret_extended | 149,84 | 18,26 | 2,6055 | -0,9 | 0,0073 | 5,8 | 69,6 | 124,5 | 184,6 |
| full_fuel_baseline_sensors_only | 147,29 | 1,51 | 2,6034 | -0,9 | 0,0231 | 5,4 | 69,8 | 125,2 | 186,1 |
| zero_fuel_design_payload | 119,70 | 9,76 | 2,6028 | -1,1 | 0,0162 | 5,2 | 68,7 | 122,6 | 182,2 |
| zero_fuel_max_payload | 128,20 | 18,26 | 2,6045 | -1,0 | 0,0066 | 5,6 | 68,8 | 122,8 | 182,2 |
| reserve_fuel_design_payload | 123,31 | 9,76 | 2,6030 | -1,0 | 0,0161 | 5,3 | 68,8 | 122,9 | 182,6 |
| minimum_flying_turret_only | 112,96 | 0,01 | 2,6068 | -1,1 | 0,0281 | 6,1 | 68,6 | 122,2 | 182,1 |
| e180_growth_turret_full_fuel | 149,84 | 9,76 | 2,5833 | -0,9 | 0,0138 | 1,2 | 69,8 | 129,3 | 189,8 |

## Gruplar

| Grup | Kütle (kg) | AG x (m) |
|---|---:|---:|
| chassis | 19,68 | 2,512 |
| systems | 15,21 | 1,445 |
| propulsion | 12,97 | 3,628 |
| gear | 12,91 | 2,422 |
| wing | 12,87 | 2,769 |
| shell | 10,82 | 2,260 |
| controls | 8,15 | 3,037 |
| tail | 7,71 | 3,781 |
| hardware | 5,52 | 2,543 |
| fuel | 2,37 | 2,832 |
| payload | 1,74 | 1,220 |

## Kalemlendirilmiş boş kütle payları (geometride modellenmeyen; spec.mass.items registry_allowance)

| Kalem | Grup | Kütle (kg) | x (m) |
|---|---|---:|---:|
| paint_finish_body | shell | 0,612 | 2,300 |
| paint_finish_wing | wing | 0,585 | 2,771 |
| paint_finish_tail | tail | 0,307 | 3,759 |
| gear_door_perimeter_seals | gear | 0,099 | 2,515 |
| brake_lines_fluid | gear | 0,135 | 1,948 |

## Kütle bütçesi karşılaştırması

| Grup | Hesap (kg) | Bütçe (kg) | Tolerans | Durum |
|---|---:|---:|---:|---|
| chassis | 19,68 | 12,61 | ±0,37 | ✗ |
| controls | 8,15 | 8,29 | ±0,25 | ✓ |
| fuel | 2,37 | 2,35 | ±0,07 | ✓ |
| gear | 12,91 | 14,57 | ±0,44 | ✗ |
| hardware | 5,52 | 1,51 | ±0,05 | ✗ |
| propulsion | 12,97 | 14,54 | ±0,44 | ✗ |
| shell | 10,82 | 10,48 | ±0,31 | ✗ |
| systems | 15,21 | 14,79 | ±0,44 | ✓ |
| tail | 7,71 | 7,35 | ±0,22 | ✗ |
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
| YK250-FU-596 | 14,254 |
| YK250-FU-598 | 10,966 |
| YK250-PR-500 | 6,533 |
| YK250-FU-597 | 4,920 |
| YK250-SY-800 | 4,412 |
| YK250-CH-001 | 2,544 |
| YK250-SY-726 | 1,980 |
| YK250-SY-720 | 1,600 |
| YK250-PL-820 | 1,550 |
| YK250-LG-622-R | 1,390 |
| YK250-LG-622-L | 1,390 |
| YK250-WG-151-R | 1,283 |
| YK250-WG-151-L | 1,283 |
| YK250-WG-150-R | 1,063 |
| YK250-WG-150-L | 1,063 |
| YK250-TL-256-R | 1,061 |
| YK250-TL-256-L | 1,061 |
| YK250-WG-153-R | 0,985 |
| YK250-WG-153-L | 0,985 |
| YK250-CH-085 | 0,956 |
| YK250-LG-626 | 0,895 |
| YK250-WG-154-R | 0,862 |
| YK250-WG-154-L | 0,862 |
| YK250-PR-510 | 0,800 |
| YK250-SH-383 | 0,781 |

## Sorunlar

- minimum_flying_turret_only: CG x 2.6068 outside [2.5835, 2.60668]
- e180_growth_turret_full_fuel: CG x 2.5833 outside [2.5835, 2.60668]
- budget chassis: 19.68 kg vs 12.61 ± 0.37
- budget gear: 12.91 kg vs 14.57 ± 0.44
- budget hardware: 5.52 kg vs 1.51 ± 0.05
- budget propulsion: 12.97 kg vs 14.54 ± 0.44
- budget shell: 10.82 kg vs 10.48 ± 0.31
- budget tail: 7.71 kg vs 7.35 ± 0.22
- budget wing: 12.87 kg vs 15.18 ± 0.46
- spec.mass.registry groups.chassis stale: 19.774 kg [2.5145, 0.0004, 0.0106] vs registry 19.676 kg [2.5119, 0.0004, 0.0111]
- spec.mass.registry groups.controls stale: 8.150 kg [3.0375, -0.004, 0.1488] vs registry 8.148 kg [3.0369, -0.004, 0.1488]
- spec.mass.registry groups.fuel stale: 2.366 kg [2.8324, 0.0105, 0.0168] vs registry 2.366 kg [2.8318, 0.0105, 0.0168]
- spec.mass.registry groups.gear stale: 12.703 kg [2.4277, -0.0009, -0.2471] vs registry 12.679 kg [2.4268, -0.0009, -0.2509]
- spec.mass.registry groups.tail stale: 7.332 kg [3.7819, -0.0001, 0.3361] vs registry 7.406 kg [3.7823, -0.0001, 0.3349]
- spec.mass.registry groups.wing stale: 12.297 kg [2.77, -0.0033, 0.0525] vs registry 12.285 kg [2.769, -0.0033, 0.0525]
- spec.mass.registry fuel stale: 30.289 kg [2.6125, 0.0, 0.0116] vs registry 30.140 kg [2.6113, 0.0, 0.0114]
