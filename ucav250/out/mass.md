# YK-250 kütle özellikleri

Boş kütle (faydalı yük ve yakıt hariç; kayıt defteri parçaları + kalemlendirilmiş paylar): **108,16 kg**, AG x = 2,630 m (kayıt defteri parçaları yalnız: 106,42 kg, 2414 parça).

## Yükleme durumları

Her durum sizing.mass_cases ile aynı yüklemeyi taşır: kayıt defteri parçaları (yakıt içeriği durumun yakıt oranıyla), kalemlendirilmiş boş kütle payları ve durumun faydalı yük kalemleri (kayıt defterinde parçası olmayanlar — görev bilgisayarı, tepsi, araştırma yükü payı, E180 farkı — kendi spec konumlarında noktasal kütle).

| Durum | Kütle (kg) | Noktasal faydalı yük (kg) | AG x (m) | AG y (mm) | AG z (m) | %OAK (MAC) | Ixx | Iyy | Izz (kg·m²) |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| mtow_design_payload_turret_retracted | 149,90 | 9,76 | 2,6055 | -0,9 | 0,0157 | 5,6 | 69,9 | 125,1 | 185,8 |
| mtow_design_payload_turret_extended | 149,90 | 9,76 | 2,6055 | -0,9 | 0,0157 | 5,6 | 69,9 | 125,1 | 185,8 |
| mtow_max_payload_turret_retracted | 149,90 | 18,26 | 2,6065 | -0,9 | 0,0077 | 5,9 | 69,8 | 124,6 | 184,9 |
| mtow_max_payload_turret_extended | 149,90 | 18,26 | 2,6065 | -0,9 | 0,0077 | 5,9 | 69,8 | 124,6 | 184,9 |
| full_fuel_baseline_sensors_only | 147,26 | 1,51 | 2,6044 | -0,9 | 0,0236 | 5,4 | 69,9 | 125,4 | 186,4 |
| zero_fuel_design_payload | 119,66 | 9,76 | 2,6038 | -1,1 | 0,0167 | 5,3 | 68,8 | 122,7 | 182,5 |
| zero_fuel_max_payload | 128,16 | 18,26 | 2,6055 | -1,0 | 0,0071 | 5,7 | 69,0 | 122,9 | 182,5 |
| reserve_fuel_design_payload | 123,29 | 9,76 | 2,6040 | -1,0 | 0,0166 | 5,3 | 68,9 | 123,0 | 182,9 |
| minimum_flying_turret_only | 112,94 | 0,01 | 2,6078 | -1,1 | 0,0287 | 6,1 | 68,7 | 122,3 | 182,4 |
| e180_growth_turret_full_fuel | 149,90 | 9,76 | 2,5843 | -0,9 | 0,0142 | 1,3 | 70,0 | 129,4 | 190,1 |

## Gruplar

| Grup | Kütle (kg) | AG x (m) |
|---|---:|---:|
| chassis | 19,86 | 2,506 |
| systems | 15,21 | 1,445 |
| propulsion | 12,97 | 3,628 |
| gear | 12,94 | 2,424 |
| wing | 12,88 | 2,770 |
| shell | 10,82 | 2,260 |
| controls | 8,15 | 3,037 |
| tail | 7,74 | 3,782 |
| hardware | 5,25 | 2,573 |
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
| chassis | 19,86 | 19,86 | ±0,60 | ✓ |
| controls | 8,15 | 8,15 | ±0,24 | ✓ |
| fuel | 2,37 | 2,37 | ±0,07 | ✓ |
| gear | 12,94 | 12,94 | ±0,39 | ✓ |
| hardware | 5,25 | 5,25 | ±0,16 | ✓ |
| propulsion | 12,97 | 12,97 | ±0,39 | ✓ |
| shell | 10,82 | 10,82 | ±0,32 | ✓ |
| systems | 15,21 | 15,21 | ±0,46 | ✓ |
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
| YK250-FU-596 | 14,294 |
| YK250-FU-598 | 11,011 |
| YK250-PR-500 | 6,533 |
| YK250-FU-597 | 4,932 |
| YK250-SY-800 | 4,412 |
| YK250-CH-001 | 2,550 |
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
