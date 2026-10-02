# 08 — Güvenlik ve Mevzuat

> Bu belge mühendislik amaçlı bir özettir, hukuki görüş değildir. Uçuş öncesinde SHGM'nin güncel
> mevzuatı ve İHATTYS duyuruları kontrol edilmelidir. Durum tarihi: **2 Ekim 2026**.

## 1. Türkiye: Yeni SHT-İHA (30 Temmuz 2026)

SHGM, **30.07.2026**'da yeni *İnsansız Hava Aracı Sistemleri Talimatı (SHT-İHA)*'nı yayımladı;
talimat yayım tarihinde yürürlüğe girdi ve 22.02.2016 tarihli eski SHT-İHA'yı kaldırdı
(Md. 30–31). Uyum için **31.07.2027**'ye kadar geçiş süresi var (Geçici Md. 1). Aşağıdaki madde
numaraları resmi metinden doğrulanmıştır.

### 1.1 Temel kurallar

| Konu | Hüküm | Madde |
|---|---|---|
| Kapsam dışı | **Münhasıran kapalı alanlarda** yapılan operasyonlar | 2(2)(b) |
| Sınıflar | **M0: 500 g – 4 kg**, M1: 4–25 kg, M2: 25–150 kg, M3: ≥ 150 kg. 500 g altı ama "Basit İHA" (oyuncak; 10 m yarıçap / 20 m yükseklik zarfı) olmayanlar M0 hükümlerine tabi | 8 |
| Uçuşa elverişlilik | M0 ve M1 için **emniyet ve uygunluk beyanı** | 8(5) |
| Yetki modeli | Uçuş yetkisi aracın ağırlığına değil **pilot lisansına (P0/P1/P2)** ve bölgeye (yeşil/turuncu/kırmızı) bağlı | 6, 15 |
| P0 (amatör) | VLOS, gündüz, ≤ 120 m yükseklik, ≤ 3000 m mesafe; asgari yaş 10 | 15(2)(a), 17 |
| P1 (ticari) | VLOS, ≤ 120 m, ≤ 3000 m; uygun donanımla gece; asgari yaş 18 | 15(2)(b), 17 |
| P2 (ileri) | BVLOS, **otonom operasyonlar**, kalabalık ve insan üstü uçuş, kargo; 3. sınıf sağlık sertifikası; işletici bünyesinde | 15(2)(c), 15(6), 19 |
| Otonom operasyon | Yalnızca **P2** ve SHGM'den operasyonel yetkilendirme ile; operasyon el kitabında görev profili, acil durumlar ve insan müdahalesi koşulları belgelenir | 7(3), 7(4) |
| **YZ destekli otonom sistem** | Ayrı bir talimat çıkana kadar: **kontrollü izin + P2 + geofence + tescil** şartıyla. **Kontrollü sahada yürütülen Ar-Ge ve test uçuşları saklıdır** | 4(aa), 7(5) |
| Kayıt | Basit İHA hariç tüm kullanıcılar İHATTYS'e kayıt; P2 ile 120 m / 3000 m ötesi uçuşlarda araç tescili | 11 |
| Bölgeler | Yeşil (izinsiz), kırmızı (kapalı), turuncu (izin/tahsis); bunların dışı izne tabi | 12 |
| Bağlantı kaybı | Meskûn mahal üzerinden geçmeden **önceden belirlenmiş kurtarma alanına otomatik gidiş** veya uçuş sonlandırma | 18(1) |
| Uygulama bağlantısı | Mobil uygulama/sistem bağlantısı yoksa İHA çalışmamalı; seri numarası doğrulanmalı | 23(1)(a) |
| Geofence | Kayıt doğrulanana kadar **10 m yarıçap / 20 m yükseklik** varsayılan zarf; sonra lisansa göre genişler | 23(2) |
| **Uyum yazılım modülü** | Kontrol istasyonuna gömülü; İHATTYS'ten gelen zarf daraltma, rota değişikliği, iniş ve uçuş sonlandırma talimatlarını gecikmeksizin uygular | 23(3) |
| **Yazılım sertifikasyonu** | Basit İHA hariç (yerli üretim dahil) uçuş kontrol ve kontrol istasyonu yazılımları SHGM yazılım sertifikasyonuna sahip olmalı | 8(6), 23(4) |
| Telsiz | Haberleşme cihazları BTK düzenlemelerine uygun olmalı (5809 sayılı Kanun) | 23(5) |
| **Uzaktan tanımlama** | Kayda tabi tüm İHA'larda zorunlu | 23(7) |
| Mevcut araçlar | Kayıt, uzaktan tanımlama, geofence, uyum modülü, yazılım sertifikası ve İHATTYS entegrasyonu 31.07.2027'ye kadar | 27, Geçici 2 |
| İHATTYS öncesi | İHATTYS devreye alınıp duyurulana kadar kayıt/izin işlemlerinde eski hükümler uygulanır | Geçici 1(3) |

Kaynak: SHGM duyurusu ve talimat metni —
https://web.shgm.gov.tr/tr/genel-duyurular/7645-insansiz-hava-araci-sistemleri-talimati-sht-iha-yayimlanmistir ·
https://web.shgm.gov.tr/documents/sivilhavacilik/files/mevzuat/sektorel/talimatlar/SHT-IHA(1).pdf

### 1.2 Bu proje için anlamı

| Gereklilik | Etki | Tasarım yanıtı |
|---|---|---|
| Sınıf **M0** (1,5–1,8 kg) | Emniyet ve uygunluk beyanı | Test kanıtları (bu repo: testler, SITL ve uçuş kayıtları) beyan dosyasına girer |
| Hedef takip ve avuca iniş = **YZ destekli otonom sistem** | Açık alanda: P2 + kontrollü izin + geofence + tescil | Geliştirme **kapalı alanda (kapsam dışı)** fileli test hücresinde veya izinli kontrollü test sahasında (Ar-Ge istisnası) yapılır |
| Yazılım sertifikasyonu | PX4 tabanlı uçuş yazılımı ve yer istasyonu sertifikalanmalı; usul henüz SHGM tarafından belirlenecek | Sürüm etiketli derlemeler, değişiklik kaydı, otomatik testler, izlenebilir parametre dosyaları (`config/`) |
| Uyum yazılım modülü | SHGM komut setine göre entegrasyon zorunlu | Companion'da `ihattys_adapter` düğümü için yer ayrıldı: zarf daraltma → PX4 geofence, rota/iniş/sonlandırma → PX4 komutları |
| Uzaktan tanımlama | Zorunlu | PX4 Open Drone ID destekli bir Remote ID modülü (BOM'da); İHATTYS ağ tabanlı tanımlama arayüzü netleşince güncellenir |
| Bağlantı kaybı | Kurtarma alanına otomatik gidiş | PX4 RTL + güvenli noktalar (rally point), meskûn mahal dışına; `config/px4/failsafe.params` |
| Varsayılan geofence 10 m / 20 m | Kayıt doğrulanana kadar | Companion güvenlik izleyicisinde "kayıt doğrulanmadı" profili |
| BTK uyumu | Frekans ve çıkış gücü sınırları | ELRS'de 915 MHz (FCC) yerine **868 MHz (EU)** düzenleme alanı; 2,4 GHz link ve WFB-ng çıkış güçleri sınırlı tutulur. Bant ve güç sınırları BTK kısa mesafe telsiz (KMET) düzenlemesinden doğrulanmalıdır |

## 2. AB (EASA) karşılaştırması

- Açık kategori A1/A2/A3, azami 120 m. **C0**: < 250 g, 19 m/s; **C1**: < 900 g veya < 80 J
  çarpma enerjisi, doğrudan Remote ID ve coğrafi farkındalık; **C2**: < 4 kg, 3 m/s düşük hız modu.
- 250 g üstü sınıf etiketsiz kendi yapımı İHA'lar yalnızca **A3** (yerleşimden 150 m uzak) veya
  risk değerlendirmeli (SORA) **belirli kategoride** uçabilir.
- Bu drone C sınıfı etiketi taşımadığı için AB'de A3 veya belirli kategori geçerlidir.

## 3. Mühendislik güvenliği

### 3.1 Tehlike büyüklükleri
| Büyüklük | Değer |
|---|---|
| 7 inç pervane uç hızı | ≈ 76 m/s (hover), ≈ 183 m/s (tam gaz) |
| Aşağı akış hızı (1,25 kg) | ≈ 7 m/s rotorda, ≈ 14 m/s uzak izde |
| Kinetik enerji (1,6 kg) | 19 m/s: 289 J · 8 m/s (takip azamisi): 51 J · 3 m/s (kişi yakını): 7,2 J · 1 m/s: 0,8 J |
| Kanıt | Pervane çarpması, ABD'de hobi hava aracı yaralanmalarının en sık nedeni (2010–2017); yüz/göz kesileri vaka raporlarında |

### 3.2 Tasarım önlemleri
1. **Tam pervane koruması + alt ağ**; avuç tutamağı pervane düzleminin ≥ 120 mm altında
   ([05](05-avuca-inis-tasarimi.md)). Alternatif, daha güvenli ilk sürüm: kullanıcının elinde
   tuttuğu **işaretli iniş pedi** (20–25 cm) veya avuç içine işaret (AprilTag) dikili eldiven.
2. Kişilere yakın hız sınırları (≤ 1 m/s, 3 m içinde) → çarpma enerjisi < 1 J ([`safety.yaml`](../config/mission/safety.yaml)).
3. Avuca iniş modunda gaz, eğim ve alçalma hızı sınırları; dar geofence (ör. 10 m).
4. Companion'dan bağımsız **RC kill switch**; temas sonrası ESC frenli durma.
5. Arming öncesi kontroller: koruma takılı, sensörler sağlıklı, companion hazır, Remote ID aktif.
6. Bağlantı kaybında kurtarma alanına RTL; GNSS yoksa yerinde iniş.

### 3.3 Test güvenliği
- Avuca iniş geliştirmesi **kapalı, fileli test hücresinde** (hem güvenlik hem de mevzuat kapsamı
  açısından), emniyet pilotu ve kill switch ile.
- KKD: kesilmeye dayanıklı eldiven (EN 388 seviye F), koruyucu gözlük/yüz siperi.
- Test sırası ve kabul ölçütleri: [05 §7](05-avuca-inis-tasarimi.md) ve [09](09-yol-haritasi.md).

### 3.4 Uçuş öncesi kontrol listesi (özet)
- [ ] Pervaneler ve korumalar sağlam, vidalar torklu; tutamak sabit
- [ ] Batarya gerilimi/hücre dengesi; batarya sabitleme kayışı
- [ ] PX4 ön uçuş kontrolleri yeşil; EKF sağlıklı; GNSS/RTK durumu
- [ ] Companion servisleri çalışıyor (`behavior_manager`, DDS köprüsü, kayıt)
- [ ] RC kill switch ve mod anahtarı testi; failsafe parametreleri yüklü
- [ ] Remote ID yayını aktif; uçuş bölgesi İHATTYS/izin durumu kontrol edildi
- [ ] Hava durumu: rüzgâr ≤ 10 m/s (avuca iniş için ≤ 5 m/s)
