# YK-250 kütle özellikleri

Boş kütle (faydalı yük ve yakıt hariç): **107,44 kg**, AG x = 2,645 m.

## Yükleme durumları

| Durum | Kütle (kg) | AG x (m) | AG y (mm) | AG z (m) | %OAK (MAC) | Ixx | Iyy | Izz (kg·m²) |
|---|---:|---:|---:|---:|---:|---:|---:|---:|
| mtow_design_payload_turret_retracted | 139,46 | 2,6202 | -0,9 | 0,0254 | 8,6 | 70,1 | 122,2 | 183,5 |
| mtow_design_payload_turret_extended | 139,46 | 2,6202 | -0,9 | 0,0254 | 8,6 | 70,1 | 122,2 | 183,5 |
| mtow_max_payload_turret_retracted | 137,46 | 2,6203 | -0,9 | 0,0256 | 8,7 | 70,0 | 122,0 | 183,2 |
| mtow_max_payload_turret_extended | 137,46 | 2,6203 | -0,9 | 0,0256 | 8,7 | 70,0 | 122,0 | 183,2 |
| full_fuel_baseline_sensors_only | 145,03 | 2,6199 | -0,9 | 0,0248 | 8,6 | 70,3 | 122,6 | 184,1 |
| zero_fuel_design_payload | 109,17 | 2,6223 | -1,2 | 0,0292 | 9,1 | 69,0 | 119,8 | 180,1 |
| zero_fuel_max_payload | 109,17 | 2,6223 | -1,2 | 0,0292 | 9,1 | 69,0 | 119,8 | 180,1 |
| reserve_fuel_design_payload | 112,81 | 2,6220 | -1,1 | 0,0286 | 9,0 | 69,1 | 120,1 | 180,5 |
| minimum_flying_turret_only | 112,20 | 2,6220 | -1,1 | 0,0287 | 9,0 | 69,1 | 120,0 | 180,4 |
| e180_growth_turret_full_fuel | 139,46 | 2,6202 | -0,9 | 0,0254 | 8,6 | 70,1 | 122,2 | 183,5 |

## Gruplar

| Grup | Kütle (kg) | AG x (m) |
|---|---:|---:|
| chassis | 19,19 | 2,590 |
| systems | 15,21 | 1,445 |
| propulsion | 13,45 | 3,631 |
| gear | 12,70 | 2,428 |
| wing | 12,30 | 2,770 |
| shell | 10,21 | 2,258 |
| controls | 8,15 | 3,038 |
| tail | 7,33 | 3,782 |
| hardware | 6,53 | 2,525 |
| fuel | 2,37 | 2,832 |
| payload | 1,74 | 1,220 |

## Kütle bütçesi karşılaştırması

| Grup | Hesap (kg) | Bütçe (kg) | Tolerans | Durum |
|---|---:|---:|---:|---|
| chassis | 19,19 | 12,61 | ±0,37 | ✗ |
| controls | 8,15 | 8,29 | ±0,25 | ✓ |
| fuel | 2,37 | 2,35 | ±0,07 | ✓ |
| gear | 12,70 | 14,57 | ±0,44 | ✗ |
| hardware | 6,53 | 1,51 | ±0,05 | ✗ |
| propulsion | 13,45 | 14,54 | ±0,44 | ✗ |
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

- mtow_design_payload_turret_retracted: CG x 2.6202 outside [2.58402, 2.60535]
- mtow_design_payload_turret_extended: CG x 2.6202 outside [2.58402, 2.60535]
- mtow_max_payload_turret_retracted: CG x 2.6203 outside [2.58402, 2.60535]
- mtow_max_payload_turret_extended: CG x 2.6203 outside [2.58402, 2.60535]
- full_fuel_baseline_sensors_only: CG x 2.6199 outside [2.58402, 2.60535]
- zero_fuel_design_payload: CG x 2.6223 outside [2.58402, 2.60535]
- zero_fuel_max_payload: CG x 2.6223 outside [2.58402, 2.60535]
- reserve_fuel_design_payload: CG x 2.6220 outside [2.58402, 2.60535]
- minimum_flying_turret_only: CG x 2.6220 outside [2.58402, 2.60535]
- e180_growth_turret_full_fuel: CG x 2.6202 outside [2.58402, 2.60535]
- budget chassis: 19.19 kg vs 12.61 ± 0.37
- budget gear: 12.70 kg vs 14.57 ± 0.44
- budget hardware: 6.53 kg vs 1.51 ± 0.05
- budget propulsion: 13.45 kg vs 14.54 ± 0.44
- budget wing: 12.30 kg vs 15.18 ± 0.46
