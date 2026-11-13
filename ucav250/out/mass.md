# YK-250 kütle özellikleri

Boş kütle (faydalı yük ve yakıt hariç; kayıt defteri parçaları + kalemlendirilmiş paylar): **106,52 kg**, AG x = 2,628 m (kayıt defteri parçaları yalnız: 106,52 kg, 2468 parça).

## Yükleme durumları

Her durum sizing.mass_cases ile aynı yüklemeyi taşır: kayıt defteri parçaları (yakıt içeriği durumun yakıt oranıyla), kalemlendirilmiş boş kütle payları ve durumun faydalı yük kalemleri (kayıt defterinde parçası olmayanlar — görev bilgisayarı, tepsi, araştırma yükü payı, E180 farkı — kendi spec konumlarında noktasal kütle).

| Durum | Kütle (kg) | Noktasal faydalı yük (kg) | AG x (m) | AG y (mm) | AG z (m) | %OAK (MAC) | Ixx | Iyy | Izz (kg·m²) |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| mtow_design_payload_turret_retracted | 154,81 | 16,26 | 2,6046 | -0,8 | 0,0088 | 5,4 | 70,0 | 124,4 | 185,0 |
| mtow_design_payload_turret_extended | 154,81 | 16,26 | 2,6046 | -0,8 | 0,0088 | 5,4 | 70,0 | 124,4 | 185,0 |
| mtow_max_payload_turret_retracted | 154,81 | 18,26 | 2,6048 | -0,8 | 0,0070 | 5,5 | 69,9 | 124,3 | 184,8 |
| mtow_max_payload_turret_extended | 154,81 | 18,26 | 2,6048 | -0,8 | 0,0070 | 5,5 | 69,9 | 124,3 | 184,8 |
| full_fuel_baseline_sensors_only | 145,62 | 1,51 | 2,6024 | -0,9 | 0,0228 | 5,0 | 69,9 | 124,5 | 185,6 |
| zero_fuel_design_payload | 124,52 | 16,26 | 2,6027 | -1,0 | 0,0081 | 5,0 | 68,9 | 122,1 | 181,7 |
| zero_fuel_max_payload | 126,52 | 18,26 | 2,6031 | -1,0 | 0,0059 | 5,1 | 68,9 | 122,1 | 181,7 |
| reserve_fuel_design_payload | 128,16 | 16,26 | 2,6029 | -1,0 | 0,0082 | 5,1 | 69,0 | 122,4 | 182,1 |
| minimum_flying_turret_only | 111,30 | 0,01 | 2,6052 | -1,2 | 0,0277 | 5,6 | 68,7 | 121,5 | 181,6 |
| e180_growth_turret_full_fuel | 154,81 | 16,26 | 2,5841 | -0,8 | 0,0074 | 1,2 | 70,1 | 128,8 | 189,3 |

## Gruplar

| Grup | Kütle (kg) | AG x (m) |
|---|---:|---:|
| chassis | 19,77 | 2,514 |
| systems | 15,21 | 1,445 |
| propulsion | 12,97 | 3,628 |
| gear | 12,70 | 2,428 |
| wing | 12,30 | 2,770 |
| shell | 10,21 | 2,258 |
| controls | 8,15 | 3,038 |
| tail | 7,33 | 3,782 |
| hardware | 5,52 | 2,543 |
| fuel | 2,37 | 2,832 |
| payload | 1,74 | 1,220 |

## Kütle bütçesi karşılaştırması

| Grup | Hesap (kg) | Bütçe (kg) | Tolerans | Durum |
|---|---:|---:|---:|---|
| chassis | 19,77 | 12,61 | ±0,37 | ✗ |
| controls | 8,15 | 8,29 | ±0,25 | ✓ |
| fuel | 2,37 | 2,35 | ±0,07 | ✓ |
| gear | 12,70 | 14,57 | ±0,44 | ✗ |
| hardware | 5,52 | 1,51 | ±0,05 | ✗ |
| propulsion | 12,97 | 14,54 | ±0,44 | ✗ |
| shell | 10,21 | 10,48 | ±0,31 | ✓ |
| systems | 15,21 | 14,79 | ±0,44 | ✓ |
| tail | 7,33 | 7,35 | ±0,22 | ✓ |
| wing | 12,30 | 15,18 | ±0,46 | ✗ |

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
| YK250-FU-596 | 14,317 |
| YK250-FU-598 | 11,031 |
| YK250-PR-500 | 6,533 |
| YK250-FU-597 | 4,942 |
| YK250-SY-800 | 4,411 |
| YK250-CH-001 | 2,545 |
| YK250-SY-726 | 1,980 |
| YK250-SY-720 | 1,600 |
| YK250-PL-820 | 1,550 |
| YK250-LG-622-R | 1,390 |
| YK250-LG-622-L | 1,390 |
| YK250-WG-151-R | 1,283 |
| YK250-WG-151-L | 1,283 |
| YK250-WG-150-R | 1,064 |
| YK250-WG-150-L | 1,064 |
| YK250-TL-256-R | 1,039 |
| YK250-TL-256-L | 1,039 |
| YK250-WG-153-R | 0,986 |
| YK250-WG-153-L | 0,986 |
| YK250-CH-085 | 0,956 |
| YK250-LG-626 | 0,895 |
| YK250-WG-154-R | 0,862 |
| YK250-WG-154-L | 0,862 |
| YK250-PR-510 | 0,800 |
| YK250-SH-383 | 0,778 |

## Sorunlar

- budget chassis: 19.77 kg vs 12.61 ± 0.37
- budget gear: 12.70 kg vs 14.57 ± 0.44
- budget hardware: 5.52 kg vs 1.51 ± 0.05
- budget propulsion: 12.97 kg vs 14.54 ± 0.44
- budget wing: 12.30 kg vs 15.18 ± 0.46
- mtow_design_payload_turret_retracted: 154.81 kg exceeds spec mtow 149.9
- mtow_design_payload_turret_extended: 154.81 kg exceeds spec mtow 149.9
- mtow_max_payload_turret_retracted: 154.81 kg exceeds spec mtow 149.9
- mtow_max_payload_turret_extended: 154.81 kg exceeds spec mtow 149.9
- e180_growth_turret_full_fuel: 154.81 kg exceeds spec mtow 149.9
- spec.mass.registry groups.chassis stale: 19.131 kg [2.5869, 0.0004, 0.0127] vs registry 19.774 kg [2.5144, 0.0004, 0.0106]
- spec.mass.registry groups.hardware stale: 5.499 kg [2.5502, -0.0004, 0.0227] vs registry 5.517 kg [2.5431, -0.0004, 0.0224]
