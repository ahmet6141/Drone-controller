# YK-250 HANÇER — Yerleşim ve yapı arayüzü kontrolleri

Bu rapor `python3 -m ucav250.analysis.layout_check` tarafından `ucav250/spec.yaml` (`layout`, `assembly`) bölümlerinden üretilir; ayrıntılı tasarım modülleri (şasi, kanat, kuyruk, kabuk, itki, yakıt, takım, sistemler, faydalı yük) birbirlerinin geometrisini okumadan bu arayüzden çalışır. Açıklama ve gerekçeler: `docs/03_yerlesim_ve_yapi_konsepti.md`.

**Sonuç: 60/60 kontrol geçti** (130 yerleşim nesnesi, 178 yakın çift, süre 7 s).

## 1. Kontrol özeti

| No | Kontrol | Değer | Sınır | Sonuç |
|---|---|---|---|---|
| C01 | parça numarası aralıkları modüller arasında çakışmıyor | – | – | GEÇTİ |
| C01 | parça kimlikleri YK250-<KOD>-NNN kuralına ve modül aralığına uyuyor | 0 | 0 | GEÇTİ |
| C01 | parça kimlikleri yerleşim içinde tekil | 0 | 0 | GEÇTİ |
| C01 | kök parça (orta kanat kutusu) bir şasi elemanı | YK250-CH-001 | YK250-CH-... | GEÇTİ |
| C01 | çapraz başvurular çözülüyor (temas, iniş yüzeyleri, RF pencereleri, mafsallar, açıklık seçicileri, kütle kalemleri, erişim kapakları) | 0 | 0 | GEÇTİ |
| C02 | istasyonlar x boyunca sıralı, kimlikler tekil | – | – | GEÇTİ |
| C02 | istasyon türü, malzeme/süreç/katman anahtarları, kalınlık >= süreç alt sınırı, yüzler | 0 | 0 | GEÇTİ |
| C02 | geçiş kesikleri çerçeve gövdesi içinde | 0 | 0 | GEÇTİ |
| C02 | çerçeveler yakıt, taret, takım kuyusu, yük/paraşüt/teçhizat hacimlerini kesmiyor | 0 | 0 | GEÇTİ |
| C02 | yakıt bölmeleri ok açılı kiriş çerçevelerini izliyor (çerçeve gövdesine boşluk) | 6,6 | >= 0 mm | GEÇTİ |
| C02 | ok açılı bölmelerin kullanılabilir yakıt hacmi >= gerekli hacim | 51,1 | >= 42,07 L | GEÇTİ |
| C02 | kablo, itme çubuğu ve kayış geçişleri tanımlı kesiklerden | 0 | 0 | GEÇTİ |
| C03 | teçhizat zarfları dış yüzeyin (OML) içinde, 10 mm pay | 0 | >= 10 mm | GEÇTİ |
| C03 | şasi elemanları OML içinde (kaplamaya oturan yüzler hariç) | 0 | >= 0 (kaplamaya oturan yüzler hariç) | GEÇTİ |
| C03 | bağlantı parçaları OML / kuyruk içinde | 0 | >= 0 (kaplamaya oturan yüzler hariç) | GEÇTİ |
| C03 | kanat / dikey eyleyicileri kesit içinde, 3 mm pay | 0 | >= 3 mm | GEÇTİ |
| C03 | kablo demetleri OML içinde (yarıçap + 3 mm) | 0 | >= r + 3 mm | GEÇTİ |
| C03 | motor bağlantı kafesi kaporta içinde | 0 | 0 | GEÇTİ |
| C03 | taret büyüme zarfı (toplanmış) OML içinde | 0 | 0 | GEÇTİ |
| C04 | içerik / yapı çakışması yok | 0 | 0 | GEÇTİ |
| C05 | takım toplama dizisi (21 durum): ana takım lastiği – yapı / içerik | 12 | >= 12 mm | GEÇTİ |
| C05 | takım toplama dizisi (21 durum): ana takım bacağı – yapı / içerik | 30 | >= 10 mm | GEÇTİ |
| C05 | takım toplama dizisi (21 durum): ana takım iç kapağı – hareketli takım | 22,8 | >= 10 mm | GEÇTİ |
| C05 | takım toplama dizisi (21 durum): burun takımı lastiği – yapı / içerik | 13,5 | >= 12 mm | GEÇTİ |
| C05 | takım toplama dizisi (21 durum): burun takımı bacağı – yapı / içerik | 30 | >= 10 mm | GEÇTİ |
| C05 | taret E180 zarfı strok boyunca bölme duvarları / tavan / çerçevelere | 16,1 | >= 6 mm | GEÇTİ |
| C05 | taret E180 zarfı asansör raylarına / bilyalı vidaya | 30,3 | >= 5 mm | GEÇTİ |
| C05 | taret E180 zarfı kayar kapaklara (bağlı dizi) | 9 | >= 5 mm | GEÇTİ |
| C05 | HD59 topu ile açıklık halkası arası (radyal) | 5 | >= 5 mm | GEÇTİ |
| C05 | stabilatörler (tüm sapma) egzoz zarflarına | 125,5 | >= 50 mm | GEÇTİ |
| C05 | stabilatörler egzoz duman konisinin dışında | 190,6 | >= 0 | GEÇTİ |
| C05 | kanatçık / flap (tüm sapma) kanat eyleyicilerine | 66,5 | >= 3 mm | GEÇTİ |
| C05 | paraşüt kapağı (0-110°) dış antenlere, sondalara, ışıklara | 100 | >= 10 mm | GEÇTİ |
| C06 | kütle yerleşimi yerleşim nesnelerinden yeniden hesaplananla aynı | 0 | <= 2 mm | GEÇTİ |
| C06 | spec kütle kalemleri yerleşim konumlarında (sizing --update-spec uygulandı) | 0,1 | <= 2 mm | GEÇTİ |
| C06 | boş uçak AM'si: spec kalemleri ile yerleşim kalemleri arasındaki fark | [-0,02; 0; 0] | <= 1 mm | GEÇTİ |
| C07 | taret görüş alanı: bütün dış çıkıntılar -5° konisinin üstünde | -1,01 | >= -5,0° | GEÇTİ |
| C07 | RF pencereleri: iç antenlerin tümü GFRP panel / uç kapağı altında | 0 | 0 | GEÇTİ |
| C07 | GNSS antenleri üst yüzey pencerelerinin altında | 2 | – | GEÇTİ |
| C08 | Li-ion tampon batarya ile yakıt hücreleri arası | 1,651 | >= 1,0 m | GEÇTİ |
| C08 | yakıt hücreleri ile yangın perdesi ön yüzü arası (CS-LUAS.967(c)) | 0,5973 | >= 0,013 m | GEÇTİ |
| C08 | motor dinamik zarfı ile diğer nesneler arası | 10 | >= 10 mm | GEÇTİ |
| C08 | motor bağlantı kafesi ile SG750 arası | 17,7 | >= 10 mm | GEÇTİ |
| C08 | silindir/kafa sıcak bölgesi ile kompozit yapı arası | 34 | >= 25 mm | GEÇTİ |
| C08 | egzoz zarfı ile kalkansız kompozit yapı arası | 75,3 | >= 50 mm | GEÇTİ |
| C08 | egzoz zarfı ile kablo demetleri arası | 100 | >= 50 mm | GEÇTİ |
| C08 | ventral kanatçık ile egzoz zarfı / duman konisi | 137,3 | >= 50 mm (kutular) / koni dışında (duman) | GEÇTİ |
| C08 | pervane diski yasak bölgesi boş | 0 | 0 | GEÇTİ |
| C08 | paraşüt açılma hacmi boş | 0 | 0 | GEÇTİ |
| C09 | kabuk panelleri: kenar payı, aralık, menteşe, RF malzemesi | 0 | 0 | GEÇTİ |
| C09 | üst gövde burundan kaporta çıkışına kadar panellerle kaplı | 0 | 0 | GEÇTİ |
| C10 | bakım: her teçhizat ve yakıt hücresi sökülebilir/menteşeli bir kapağın altında | 0 | 0 | GEÇTİ |
| C10 | bakım erişim matrisi: hiçbir kalem için birincil yapı sökülmüyor | 22 | – | GEÇTİ |
| C11 | montaj adımları 1..N, Türkçe başlık/alt montaj/metin/takım/kontrol | 42 | – | GEÇTİ |
| C11 | montaj şasi tezgâhından ayara kadar bütün grupları kapsıyor | 0 | 0 | GEÇTİ |
| C11 | taşıma birimleri R-29 / R-30 ile uyumlu | [2,904; 1,4] | [3,4; 2] | GEÇTİ |
| C11 | sahada montaj ve bakım matrisi mevcut | 7 | – | GEÇTİ |
| C12 | mekanizma tanımları (alanlar, aralıklar, özellikler, ifadeler, L/R çiftleri, diziler) | 0 | 0 | GEÇTİ |
| C12 | layout.clearances checks.py LISTE biçiminde | 26 | – | GEÇTİ |
| C13 | bağlantıların ilk ön boyutlandırması: emniyet payları >= 0 | 0,07 | >= 0 | GEÇTİ |

## 2. Kütle yerleşimi ve ağırlık merkezi

Boş kütle 101,4 kg; spec kalemlerinden AM [2,656; -0,0014; 0,0523] m, yerleşim nesnelerinden AM [2,656; -0,0014; 0,0523] m. Aşağıdaki kalemlerin konumu boyutlandırma evresinde varsayımdı; yerleşim evresinde yerleştirilen nesnelerin ağırlık merkezinden hesaplanır (`layout.mass_placement`) ve `sizing.mass_items` bunu uygular. Boyutlandırma evresi konumları `position_sizing_phase` alanındadır.

| Kütle kalemi | Boyutlandırma evresi x (m) | Yerleşim x (m) | Δx (m) | Esas |
|---|---|---|---|---|
| engine_group_installed | 3,83 | 3,594 | -0,2365 | engine.installed_items_kg at the layout positions (engine on the inclined crank axis; ECU in the mission bay,  |
| cooling_baffles_firewall_cowl_flap | 3,73 | 3,756 | 0,0258 | split of the allowance (estimate): firewall shield 0.5 mm 304 sheet ~0.8 kg, baffles/plenum 0.6, exit lip 0.1 |
| dorsal_cooling_inlet_s_duct | 3,4 | 3,482 | 0,0815 | flush dorsal inlet (P-INLET) + S-duct to the firewall duct cut-out |
| actuators_stabilators_2x_DA30 | 3,737 | 3,619 | -0,1181 | DA 30 lying along y on the cool side of the firewall, pushrods through fireproof boots |
| actuators_nose_steering_brake_2x_DA26 | 1,825 | 0,8813 | -0,944 | steering actuator travels with the leg (flight CG = retracted); brake-by-wire master cylinder in the nose, bra |
| wiring_harness_connectors_coax | 1,9 | 2,317 | 0,417 | cables 70 % (estimate) by trunk length x diameter^2 (layout.systems.harness) + outer-panel harnesses; connecto |
| flight_termination_lights | 1,684 | 2,331 | 0,6473 | FTS 0.15 kg + 3 x AveoFlash 0.083 kg at their installed places |
| keel_beams_longerons | 2 | 2,256 | 0,2559 | length-weighted centroid of the longitudinal members (chine longerons, keel walls, keel beams, dorsal longeron |
| frames_bulkheads | 1,88 | 2,421 | 0,5413 | 13 stations, mass ~ net web area (section inside the 6 mm skin inset minus the declared cut-outs and, for the  |
| engine_mount_4130 | 3,67 | 3,716 | 0,046 | mean of the truss tube mid-points and the ring nodes (layout.chassis.engine_mount) |
| parachute_attach_fitting | 1,65 | 2,185 | 0,5352 | two bridle fittings (Y-bridle) + container restraint |
| floors_trays_rails | 1,4 | 1,78 | 0,3805 | area-weighted decks/floors + trays |
| hatch_frames_quick_access_fasteners | 1,5 | 2,325 | 0,825 | perimeter-weighted removable panels/hatches (frame lands, Camlocs, nutplates) |
| fin_ventral_root_fittings | 3,572 | 3,62 | 0,0477 | fin, stub and ventral root fittings at the frames |

## 3. Arayüz bağlantılarının ilk ön boyutlandırması

Yöntem: kapalı biçimli pim kesme / eğilme / ezilme ve cıvata grubu kesmesi, `spec.materials` izin verilen değerleri; yük katsayıları `structures` (FoS 1,5, bağlantı 1,15, pimli bağlantı ezilme 2,0, sık sökülen bağlantı 1,5) ve standards.yaml önerileri. Ayrıntılı tasarım SE/test ile değiştirir.

| Kalem | Uygulanan | İzin verilen | Emniyet payı | Esas |
|---|---|---|---|---|
| kanat birleşimi ana pimi, çift kesme | 142,8 | 599,8 | 3,2 | R = 29302 N (y = 0,7 m'de M_nihai 5498 N m, V_nihai 4479 N; pimler arası 227 mm) x 1,5 sık sökülen bağlantı katsayısı; MPa |
| kanat birleşimi ana pimi, eğilme | 864,7 | 923,9 | 0,07 | M = R/2 (t_çatal/2 + t_dil/4) = 233 N m; Ftu ile (plastik eğilme kazancı alınmadı); MPa |
| kanat birleşimi dil ezilmesi (7075) | 137,7 | 999,7 | 6,26 | x 2,0 ezilme katsayısı, dil kalınlığı 30,4 mm; MPa |
| kanat birleşimi çatal ezilmesi (7075, 2 kulak) | 348,8 | 999,7 | 1,87 | x 2,0 ezilme katsayısı, kulaklar 6 mm; MPa |
| kayış bağlantısı cıvataları 4 x M6 12.9 (tek kesme, şokun tamamını tek ayak taşır) | 187,4 | 732 | 2,91 | 13,1 kN (GRS 4/240 yayımlanmış açılma şoku, > UAVOS 200 5 g x MTOM) yalnız nihai (CRASH-004) x 1,15 bağlantı katsayısı; ISO 898-1 12.9 Rm 1220 MPa, gerilme alanında tau = 0,6 Rm; MPa |
| kayış kilit pimi Ø10 (4130, çift kesme) | 95,9 | 393 | 3,1 | MPa |
| kayış U-kulak ezilmesi (7075, 2 yanak 8 mm) | 188,3 | 999,7 | 4,31 | x 2,0 ezilme katsayısı; MPa |
| MTOM'da açılma yük katsayısı (bilgi) | 8,91 | – | – | 13,1 kN / (MTOM g); UAVOS 200 anma değeri 5 g |
| motor bağlantı cıvatası M8 12.9 (tork + 3,8 g / 1,47 g yan / 6 g aşağı durumlarının en kötüsü) | 34,9 | 732 | 19,96 | motor + SG750 + adaptör 7,59 kg, AM bağlantı yüzünün 90 mm önünde (tahmin), x 1,5 (uçuş durumları) x 1,15; gerilme alanında birleşik kesme/çekme, 0,6 Rm ile karşılaştırma; MPa; karter dişi sıyrılması ve sönümleyici kapasitesi açık konu (Limbach verisi) |
| 15 g ileri çarpma tutması (motor kafese basar, kafes basıda) | 1284 | – | – | N; kaynaklı kafes basıyla taşır (4130 boruların ayrıntılı boyutlandırması) |

## 4. İstasyonlar (çerçeveler / perdeler)

| Kimlik | x (m) | Tür | Malzeme / süreç / katman | t (mm) | Kesikler |
|---|---|---|---|---|---|
| FS0300 | 0,3 | bulkhead | cfrp_pw_mtm45_as4 / prepreg_ooa_vacbag / rib_panel | 6,8 | C-PITOT, C-COAX-NC |
| FS0600 | 0,6 | bulkhead | cfrp_pw_mtm45_as4 / prepreg_ooa_vacbag / rib_panel | 6,8 | C-HARN-FWD |
| FS1110 | 1,11 | bulkhead | cfrp_pw_mtm45_as4 / prepreg_ooa_vacbag / rib_panel | 6,8 | C-HARN-1110 |
| FS1330 | 1,33 | bulkhead | cfrp_pw_mtm45_as4 / prepreg_ooa_vacbag / rib_panel | 6,8 | C-HARN-1330 |
| FS1490 | 1,49 | bulkhead | cfrp_pw_mtm45_as4 / prepreg_ooa_vacbag / rib_panel | 6,8 | C-HARN-1490 |
| FS1810 | 1,81 | fitting frame | cfrp_pw_mtm45_as4 / prepreg_ooa_vacbag / rib_panel | 6,8 | C-HARN-1810, C-BRIDLE-F |
| FS-FUEL | 2,217 | bulkhead | cfrp_pw_mtm45_as4 / prepreg_ooa_vacbag / rib_panel | 6,8 | C-FUEL-FUEL, C-HARN-FUEL, C-HARN-WING, C-SPINE-FUEL |
| FS-MS | 2,514 | fitting frame / spar frame | cfrp_pw_mtm45_as4 / prepreg_ooa_vacbag / rib_panel | 6,8 | C-PAYLOAD-MS, C-FUEL-MS, C-SPINE-MS, C-HARN-MS |
| FS-RS | 2,828 | fitting frame / spar frame | cfrp_pw_mtm45_as4 / prepreg_ooa_vacbag / rib_panel | 6,8 | C-PAYLOAD-RS, C-FUEL-RS, C-HARN-RS, C-BRIDLE-A |
| FS-GEAR | 3,122 | bulkhead | cfrp_pw_mtm45_as4 / prepreg_ooa_vacbag / rib_panel | 6,8 | C-FUEL-FEED, C-HARN-CL, C-DOORLINK |
| FS3480 | 3,48 | ring | cfrp_pw_mtm45_as4 / prepreg_ooa_vacbag / rib_panel | 6,8 | – |
| FS3670 | 3,67 | bulkhead / firewall | cfrp_pw_mtm45_as4 / prepreg_ooa_vacbag / rib_panel | 14,8 | C-DUCT, C-FW-FUEL, C-FW-HARN, C-FW-PUSHROD |
| FS3738 | 3,738 | ring | al_2024_t3_sheet / sheet_metal_aluminium / – | 2 | – |

## 5. Şasi elemanları ve bağlantılar

| Kimlik | Parça | Ad | Malzeme | Yük yolu |
|---|---|---|---|---|
| M-CHINE | YK250-CH-020 (L/R) | kenar çizgisi uzun kirişi | cfrp_ud_mtm45_as4 | skins (shear) -> chine longeron (axial) -> frames / wing box side-of-body rib |
| M-KEELWALL | YK250-CH-021 (L/R) | burun takımı omurga duvarı | cfrp_pw_mtm45_as4 | nose-gear loads -> pivot bushings -> keel walls -> FS0600 / FS1110 + avionics deck -> chine longerons |
| M-DECK-NOSE | YK250-CH-022 | aviyonik güverte | cfrp_pw_mtm45_as4 | equipment inertia -> deck -> keel walls + chine longerons -> FS0600 / FS1110 |
| M-TURRETWALL | YK250-CH-023 (L/R) | taret bölmesi yan duvarı | cfrp_pw_mtm45_as4 | belly cut-out edge loads + turret inertia -> walls -> FS1110 / FS1330 |
| M-TURRETROOF | YK250-CH-024 | taret bölmesi tavanı | cfrp_pw_mtm45_as4 | turret elevator reaction -> roof -> walls / frames |
| M-PARAWALL | YK250-CH-025 (L/R) | paraşüt bölmesi yan duvarı | cfrp_pw_mtm45_as4 | container inertia -> walls/floor -> FS1490 / FS1810 |
| M-PARAFLOOR | YK250-CH-026 | paraşüt bölmesi tabanı | cfrp_pw_mtm45_as4 | container inertia -> floor -> FS1490 / FS1810 / walls |
| M-MIDFLOOR | YK250-CH-027 | görev bölmesi tabanı | cfrp_pw_mtm45_as4 | mission equipment inertia -> floor -> frames + lower skin |
| M-KEEL | YK250-CH-028 (L/R) | omurga kirişi (yük bölmesi duvarı) | cfrp_pw_mtm45_as4 | payload + belly loads -> keel beams -> FS-FUEL / spar frames -> wing box |
| M-FWDDECK | YK250-CH-029 | ön yakıt bölmesi tabanı / yük bölmesi tavanı | cfrp_pw_mtm45_as4 | fuel + payload inertia -> deck -> keel beams / FS-FUEL / main-spar frame |
| M-WELLROOF | YK250-CH-030 | ana takım kuyusu tavanı | cfrp_pw_mtm45_as4 | gear + fuel loads -> well roof -> gear beams / rear-spar frame / FS-GEAR |
| M-GEARBEAM | YK250-CH-031 (L/R) | ana takım kirişi | cfrp_pw_mtm45_as4 | trunnion -> gear beam -> rear-spar frame + FS-GEAR + well roof -> wing box / chine longerons |
| M-DORSAL | YK250-CH-032 (L/R) | sırt uzun kirişi | cfrp_ud_mtm45_as4 | fin/stub root fittings -> frames FS3480 / FS3670 -> dorsal longerons -> FS-GEAR |
| M-PAYWALL-AFT | YK250-CH-034 | faydalı yük bölmesi arka duvarı | cfrp_pw_mtm45_as4 | secondary (closes the bay; payload loads go to the keel beams and the deck) |
| M-WELLWALL-FWD | YK250-CH-035 | ana takım kuyusu ön duvarı | cfrp_pw_mtm45_as4 | secondary (well closure, harness duct wall) |
| M-AFTKEEL | YK250-CH-033 | arka omurga kirişi | al_2024_t3_sheet | ventral fin / bumper strike -> aft keel beam -> firewall + ring frame |
| M-CTBOX | YK250-CH-001 | orta kanat kutusu (geçiş kutusu) | cfrp_ud_mtm45_as4 | outer panel -> tongue + pins -> fork -> spar caps (bending couple) / webs (shear) -> box -> spar frames + side-of-body r |
| M-SOB | YK250-CH-050 (L/R) | gövde yanı kaburgası | cfrp_pw_mtm45_as4 | glove skins / fittings -> rib -> spars |
| M-GLOVERIB | YK250-CH-051 (L/R) | eldiven kaburgası | cfrp_pw_mtm45_as4 | glove skins / fittings -> rib -> spars |
| M-JOINTRIB | YK250-CH-052 (L/R) | birleşim kaburgası (orta kesit) | cfrp_pw_mtm45_as4 | glove skins / fittings -> rib -> spars |
| M-LERXRIB1 | YK250-CH-056 (L/R) | LERX burun kaburgası 1 | cfrp_pw_mtm45_as4 | LERX skins -> nose ribs -> main spar / side-of-body rib |
| M-LERXRIB2 | YK250-CH-057 (L/R) | LERX burun kaburgası 2 | cfrp_pw_mtm45_as4 | LERX skins -> nose ribs -> main spar / side-of-body rib |
| M-LERXRIB3 | YK250-CH-058 (L/R) | LERX burun kaburgası 3 | cfrp_pw_mtm45_as4 | LERX skins -> nose ribs -> main spar / side-of-body rib |
| F-TRUNNION | YK250-CH-070 (L/R) | ana takım mafsal bağlantısı | al_7075_t651_plate | bolted 6 x M6 12.9 to the gear beam (outboard) through a 7075 doubler and 4 x M6 to the well roof deck (top), nutplates/ |
| F-NG-PIVOT | YK250-CH-071 | burun takımı mafsal burçları | al_7075_t651_plate | two flanged bushings (16 H7) bonded into 7075 doubler blocks on the keel walls (3 mm proud of the outboard faces, the V  |
| F-SPINDLE-IN | YK250-CH-095 (L/R) | stabilatör mili iç yatak yuvası | al_7075_t651_plate | riveted/bolted (6 x M5) to the engine-bay ring frame FS3738 web |
| F-FIN-FRONT | YK250-CH-096 (L/R) | dikey ön kiriş bağlantısı | al_7075_t651_plate | angle fitting bolted 4 x M6 to the frame and the dorsal longeron; fin spar root lug (tail module) bolted to it with 2 x  |
| F-FIN-REAR | YK250-CH-097 (L/R) | dikey arka kiriş bağlantısı | al_7075_t651_plate | angle fitting bolted 4 x M6 to the frame and the dorsal longeron; fin spar root lug (tail module) bolted to it with 2 x  |
| F-STUB-FRONT | YK250-CH-098 (L/R) | sabit kök parçası ön bağlantısı | al_7075_t651_plate | fitting on FS3480 at the body side; stub front spar lug 2 x M6 (axis y); the stub rear attachment is the spindle housing |
| F-VENTRAL-1 | YK250-CH-099 | ventral kök bağlantısı 1 | al_7075_t651_plate | lug on the aft keel beam; ventral root lug 1 x M6 12.9 (double shear, axis y) |
| F-VENTRAL-2 | YK250-CH-100 | ventral kök bağlantısı 2 | al_7075_t651_plate | lug on the aft keel beam; ventral root lug 1 x M6 12.9 (double shear, axis y) |
| F-VENTRAL-3 | YK250-CH-101 | ventral kök bağlantısı 3 | al_7075_t651_plate | lug on the aft keel beam; ventral root lug 1 x M6 12.9 (double shear, axis y) |
| F-FORK | YK250-CH-053 (L/R) | dış panel birleşim çatalı (ana kiriş) | al_7075_t651_plate | layout.chassis.wing_joint.main_spar.fork (bonded + 2 x 6 M6 to the glove main-spar caps/web) |
| F-DRAGPIN | YK250-CH-054 (L/R) | sürükleme pimi burç bağlantısı (arka kiriş) | al_7075_t651_plate | layout.chassis.wing_joint.rear_spar.bushing_fitting (4 x M5 to the joint rib and rear-spar web) |
| F-RISER-FWD | YK250-CH-112 | paraşüt ön kayış bağlantısı | al_7075_t651_plate | U-lug fitting through-bolted 4 x M6 12.9 to FS1810 (7075 doublers both faces), shackle pin d 10 (steel), bridle forward  |
| F-RISER-AFT | YK250-CH-113 | paraşüt arka kayış bağlantısı | al_7075_t651_plate | U-lug fitting through-bolted 4 x M6 12.9 to the rear-spar frame top (7075 doublers), shackle pin d 10, bridle aft leg |
| F-EMOUNT-UP | YK250-CH-086 (L/R) | motor bağlantı parçası (yangın perdesi) | al_7075_t651_plate | mount foot bolted through the firewall stack with 2 x M8 12.9, 7075 backing plate 40 x 40 x 5 mm on the forward face, st |
| F-EMOUNT-LO | YK250-CH-087 (L/R) | motor bağlantı parçası (yangın perdesi) | al_7075_t651_plate | mount foot bolted through the firewall stack with 2 x M8 12.9, 7075 backing plate 40 x 40 x 5 mm on the forward face, st |

Dış panel birleşimi y = 0,7 m: dil-çatal, iki Ø14 mm Ti-6Al-4V pim ([2.579, 0.455, -0.003], [2.61, 0.68, 0.005]), arka kirişte Ø10 mm sürükleme pimi; takma yolu ana kiriş ekseni boyunca 0,25 m.

Motor bağlantısı: welded 4130 N tube bed mount: 4 feet on the firewall attach fittings (F-EMOUNT-UP/LO, R/L) -> a welded ring (4130 tube 15.9 x 0.89 mm) in a plane 30 mm forward of the crankcase mount face; 4 elastomer isolators in the ring cups carry the engine on its 4 rear crankcase bosses — 4 x M8 cıvata, sönümleyici: Limbach optional 'Damper Shock Mount' (engine.yaml installation.mount.isolators; dimensions and stiffness not published); yangın perdesi yığını CFRP sandwich bulkhead (layups.rib_panel) 6,8 mm, air gap on 12 stainless stand-offs (insulation blanket optional) 7,5 mm, AISI 304 stainless sheet, fireproof without test (>= 0.38 mm, standards.yaml FIRE-001) 0,5 mm.

Paraşüt: Y-kayış, ön ayak 1,63 m (F-RISER-FWD), arka ayak 1,363 m (F-RISER-AFT); açılma yükü 13,1 kN tek ayakta, nihai yalnız (CRASH-004).

## 6. Mekanizmalar

| Mafsal | Tür | Eksen | Alt / üst | Özellik / ifade |
|---|---|---|---|---|
| aileron_R | revolute | [0.08, 0.995, 0.056] | -0,3491 / 0,3491 | aileron_deg  |
| aileron_L | revolute | [-0.08, 0.995, -0.056] | -0,3491 / 0,3491 | aileron_deg  |
| flap_R | revolute | [0.08, 0.995, 0.062] | 0 / 0,6981 | flap_deg  |
| flap_L | revolute | [-0.08, 0.995, -0.062] | 0 / 0,6981 | flap_deg  |
| rudder_R | revolute | [0.448, 0.335, 0.829] | -0,4363 / 0,4363 | rudder_deg  |
| rudder_L | revolute | [0.448, -0.335, 0.829] | -0,4363 / 0,4363 | rudder_deg  |
| stabilator_R | revolute | [0.0, 1.0, 0.0] | -0,3491 / 0,2618 | elevator_deg  |
| stabilator_L | revolute | [0.0, 1.0, 0.0] | -0,3491 / 0,2618 | elevator_deg  |
| main_gear_R | revolute | [-1.0, 0.0, 0.0] | 0 / 1,885 | gear_up `1.88496*clamp((gear_up-0.15)/0.7,0,1)` |
| main_gear_L | revolute | [1.0, 0.0, 0.0] | 0 / 1,885 | gear_up `1.88496*clamp((gear_up-0.15)/0.7,0,1)` |
| main_inner_door_R | revolute | [-1.0, 0.0, 0.0] | 0 / 1,658 |  `1.65806*(clamp(gear_up/0.15,0,1)-clamp((gear_up-0.85)/0.15,0,1))` |
| main_inner_door_L | revolute | [1.0, 0.0, 0.0] | 0 / 1,658 |  `1.65806*(clamp(gear_up/0.15,0,1)-clamp((gear_up-0.85)/0.15,0,1))` |
| nose_gear | revolute | [0.0, -1.0, 0.0] | 0 / 1,571 |  `1.57080*clamp((gear_up-0.15)/0.7,0,1)` |
| nose_steer | revolute | [0.0, 0.0, -1.0] | -0,3491 / 0,3491 | steer_deg  |
| nose_door_R | revolute | [0.996, 0.0, -0.088] | 0 / 1,571 |  `1.57080*(1-clamp((gear_up-0.85)/0.15,0,1))` |
| nose_door_L | revolute | [-0.996, 0.0, 0.088] | 0 / 1,571 |  `1.57080*(1-clamp((gear_up-0.85)/0.15,0,1))` |
| turret_elevator | prismatic | [0.0, 0.0, -1.0] | 0 / 0,12 | turret `0.1200*clamp((turret-0.2)/0.8,0,1)` |
| turret_door_R | prismatic | [0.0, 0.899, 0.438] | 0 / 0,12 |  `0.12*clamp(turret/0.2,0,1)` |
| turret_door_L | prismatic | [0.0, -0.899, 0.438] | 0 / 0,12 |  `0.12*clamp(turret/0.2,0,1)` |
| para_hatch | revolute | [0.0, -1.0, 0.0] | 0 / 1,92 | para_hatch_deg  |
| prop_spin | revolute | [0.996, 0.0, 0.087] | 0 / 2,094 | prop_deg  |

Dizi `gear_retraction`: 21 örnek durum, denetim özelliği `gear_up`.

Dizi `turret_extension`: 13 örnek durum, denetim özelliği `turret`.

## 7. Kabuk

Panel sayısı (L/R ayrı): sökülebilir 21, sabit 19, menteşeli 2, fileto 4; RF pencereleri: P-NOSECONE, P-AVHATCH, P-SIDEBAY-R, P-GNSS2.

## 8. Bakım erişim matrisi

| Kalem | Erişim | Bağlantı elemanı | Birincil yapı sökülür mü |
|---|---|---|---|
| bujiler, silindirler, CHT/EGT algılayıcıları, egzoz, hava yönlendiricileri | P-COWL-UP, P-COWL-LO | Camloc | hayır |
| motor (sökme) | P-COWL-UP, P-COWL-LO | Camloc + 4 x M8 bağlantı cıvatası | hayır |
| yakıt pompası/filtresi + boşaltma, iç kapak eyleyicileri, kuyruk konnektörü, motor kablo bağlantısı | P-AFTHATCH | Camloc | hayır |
| motor ECU, görev bilgisayarı / kayıt cihazı | P-MBHATCH | Camloc | hayır |
| stabilatör eyleyicileri (DA 30), itme çubukları ve körükleri, yakıt kesme vanası | P-STABACT | Camloc | hayır |
| stabilatör mili yatakları | P-COWL-UP, stabilatör sökülerek | Camloc + çapraz cıvata | hayır |
| yakıt hücreleri (muayene, değiştirme) | P-FUEL1, P-FUEL2, P-FUEL3 | M4 somun plakalı vida + conta | hayır |
| PDU, kontaktör/sigortalar, DC-DC, burun kablo demeti | P-AVHATCH | Camloc | hayır |
| Li-ion tampon batarya, bağımsız FTS birimi | P-FWDHATCH | Camloc | hayır |
| jeneratör güç elektroniği, fren eyleyicisi + ana silindir | P-SIDEBAY-L | Camloc | hayır |
| otopilot, veri bağları, transponder, Remote ID | P-SIDEBAY-R | Camloc | hayır |
| burun antenleri, pitot hatları | P-NOSECONE | 8 x M4 | hayır |
| EO/IR taret (HD59 / E180) | P-TURRETRING | 8 x M4 + 4 taret bağlantı cıvatası | hayır |
| taret asansörü, kayar kapaklar | P-TURRETRING, P-AVHATCH | M4 / Camloc | hayır |
| paraşüt (12 ayda bir katlama, UAVOS servis ömrü) | P-PARAHATCH | mandal | hayır |
| araştırma faydalı yükü | P-PAYHATCH | Camloc | hayır |
| ana takım bacakları, EMA'lar, kilitler, frenler, orta kablo kanalı | ana takım kuyuları (takım açık, iç kapaklar bakım modunda) | - | hayır |
| burun takımı, yönlendirme | omurga yuvası (takım açık), P-SIDEBAY-L, P-SIDEBAY-R | Camloc | hayır |
| kanat birleşim pimleri, emniyetleri, kanat konnektörü | P-JOINTACCESS | Camloc | hayır |
| kanatçık / flap eyleyicileri | dış panel alt servo kapakları (kanat modülü) | M4 dişli burç | hayır |
| dümen eyleyicileri, dikey uç antenleri, kuyruk ışığı | dikey servo kapakları (iç yüz), dikey uç kapakları | M4 | hayır |
| GNSS antenleri | P-AVHATCH, P-GNSS2 | Camloc / M4 | hayır |

## 9. Montaj ve taşıma

42 montaj adımı (spec.assembly.steps): 1. Ana montaj tezgâhının hazırlanması; 2. Orta kanat kutusunun yerleştirilmesi (kök parça); 3. Ana ve arka kiriş çerçeveleri; 4. Gövde yanı, eldiven, birleşim ve LERX burun kaburgaları; 5. Kanat çatalı ve sürükleme pimi burcu; 6. Ön gövde çerçeveleri; 7. Burun kutusu: omurga duvarları, aviyonik güverte, burun takımı mafsalı; 8. Taret, paraşüt ve görev bölmeleri; 9. Kenar çizgisi uzun kirişleri; 10. Omurga kirişleri, güverteler ve ana takım yapısı; 11. Arka gövde: sırt uzun kirişleri, kuyruk ve ventral bağlantıları; 12. Yangın perdesi ve motor bağlantı parçaları; 13. Motor bölmesi halka çerçevesi ve stabilatör mil iç yatakları; 14. Paraşüt, taret asansörü, yakıt bölmesi ve teçhizat bağlantıları; 15. Şasi muayenesi ve tezgâhtan ayırma; 16. Sabit gövde kaplamaları; 17. LERX/eldiven kaplamalarının yapıştırılması; 18. Yakıt sistemi; 19. Elektrik tesisatı; 20. Aviyonik ve güç dağıtımı; 21. Antenler ve hava verisi; 22. Paraşüt sistemi; 23. Görev bilgisayarı ve yük tepsisi rayları; 24. Taret asansörü, kayar kapaklar ve HD59; 25. Motorun takılması; 26. Egzoz, soğutma, motor yardımcıları; 27. Pervane göbeği, pervane ve spinner; 28. Ana iniş takımı; 29. Burun iniş takımı; 30. Takım kapakları ve sıralama; 31. Stabilatör kök parçaları, milleri ve eyleyicileri; 32. Dikey kuyruklar ve dümenler; 33. Ventral kanatçık ve tampon kızağı; 34. Stabilatörlerin takılması; 35. Dış kanat panelleri (ilk takma); 36. Kabuğun kapatılması; 37. Kumanda yüzeylerinin ayarı; 38. Fonksiyon testleri; 39. Tartı ve denge; 40. Son muayene ve kabul; 41. Taşıma için sökme; 42. Sahada montaj.

Taşıma dört parçalıdır: kanat eldiveni, dikeyler, kök parçaları ve ventral ile birlikte orta gövde (1,40 m genişlik), iki dış kanat paneli (2,90 m), iki stabilatör ve pervane.

## 10. Şekiller

![side](../docs/fig/yk250_layout_side.png)
![top](../docs/fig/yk250_layout_top.png)
![structure](../docs/fig/yk250_layout_structure.png)
![shell](../docs/fig/yk250_layout_shell.png)
![sections](../docs/fig/yk250_layout_sections.png)
