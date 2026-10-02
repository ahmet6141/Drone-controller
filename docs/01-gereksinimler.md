# 01 — Gereksinimler ve Başarı Ölçütleri

Bu belge, 7 inç sınıfı yapay zekâ destekli drone'un **ne yapması gerektiğini** tanımlar.
Donanım ve model seçimleri ([02](02-donanim-analizi.md), [03](03-yapay-zeka-modelleri.md)) bu
gereksinimlere göre yapılmıştır. Her gereksinimin bir kimliği (ör. `F-02`) vardır; test
planı ([09](09-yol-haritasi.md)) bu kimliklere referans verir.

## 1. Ürün tanımı

| Özellik | Hedef |
|---|---|
| Sınıf | Orta boy, 7 inç pervaneli quadcopter (X geometri), 6S |
| Kalkış ağırlığı (AUW) | 1,3–1,6 kg (seviyeye göre, bkz. [donanım profilleri](../config/hardware/)) |
| Uçuş süresi | ≥ 15 dk hover (Li-ion ile ≥ 18 dk hedef) |
| İtki/ağırlık | Batarya akım sınırı dahil ≥ 2,2; motor kapasitesi ≥ 3,5 |
| Ana yetenekler | Profesyonel hover, açık avuç jestiyle **ele konma**, YZ destekli **hedef takibi** |
| Çalışma ortamı | Açık alan (GNSS/RTK) + kapalı/GNSS'siz alan (optik akış + VIO) |

## 2. Fonksiyonel gereksinimler

### 2.1 Hover ve uçuş
| ID | Gereksinim | Ölçüt |
|---|---|---|
| F-01 | Sakin havada hassas pozisyon tutma | RTK veya VIO ile yatay ≤ ±0,10 m, dikey ≤ ±0,05 m (2σ, 60 s) |
| F-02 | Standart GNSS ile pozisyon tutma | Yatay ≤ ±0,5 m, dikey ≤ ±0,2 m |
| F-03 | GNSS'siz alçak irtifa hover (0,3–8 m) | Optik akış + lidar ile drift ≤ 0,2 m/dk |
| F-04 | Rüzgâr dayanımı | 10 m/s sürekli rüzgârda pozisyon tutma, 15 m/s'de güvenli iniş |
| F-05 | Yumuşak kalkış/iniş | İniş anında dikey hız ≤ 0,3 m/s |

### 2.2 Avuca iniş ("ele kon")
| ID | Gereksinim | Ölçüt |
|---|---|---|
| F-10 | Açık avuç jestini tanıma (ön kamera) | 1–5 m mesafede, ≥ %97 doğru pozitif, ≤ 1 yanlış alarm / saat |
| F-11 | Jest onayı | Jest ≥ 1,0 s kesintisiz görülmeden iniş başlamaz |
| F-12 | Avuç konumlandırma (aşağı kamera + ToF) | 0,15–1,0 m'de avuç merkezi hatası ≤ 3 cm |
| F-13 | Kontrollü alçalma | Uzakta ≤ 0,20 m/s, son 15 cm'de ≤ 0,08 m/s |
| F-14 | Temas algılama ve motor durdurma | Temastan itibaren ≤ 150 ms içinde motorlar durur |
| F-15 | Güvenli iptal | Avuç ≥ 0,3 s kaybolursa veya hata sınırı aşılırsa yukarı tırmanıp hover |
| F-16 | Kişiye yaklaşma kısıtı | Drone kişiye kendisi yaklaşmaz; kişi avucunu drone'un altına getirir |

### 2.3 Hedef tespiti ve takip
| ID | Gereksinim | Ölçüt |
|---|---|---|
| F-20 | İnsan/araç tespiti | 50 m'ye kadar insan, 120 m'ye kadar araç; ≥ 25 FPS |
| F-21 | Çoklu nesne takibi (MOT) | ID değişimi ≤ 1 / dk (kalabalık olmayan sahne) |
| F-22 | Hedef seçimi | Yer istasyonundan tıkla-takip et, jestle "beni takip et" |
| F-23 | Tek hedef takibi (SOT) | Keyfi nesne; 2 s'ye kadar örtülmede kaybetmeme |
| F-24 | Yeniden yakalama | 10 s içinde ReID ile aynı hedefi yeniden bulma |
| F-25 | Takip modları | Arkadan takip, yörünge (orbit), önden (lead), sabit (tripod/gimbal) |
| F-26 | Kadraj kalitesi | Hedef merkez hatası RMS ≤ görüntü genişliğinin %10'u |

### 2.4 Jest komutları
| Jest | Anlam | Not |
|---|---|---|
| Açık avuç (ön kamera) | Avuca iniş modunu kur | Onay süresi 1 s |
| Avuç (aşağı kamera) | İniş hedefi | Yalnızca avuca iniş kurulduysa |
| Yumruk | Dur / hover (takibi bırak) | Her durumda geçerli, en yüksek öncelik |
| Başparmak yukarı | Onay | Ör. takip hedefini onayla |
| Zafer (V) işareti | "Beni takip et" | Jesti yapan kişi hedef olur |

Jest kümesi bilerek küçük tutulmuştur: az sınıf → yüksek doğruluk ve düşük yanlış alarm.

### 2.5 Güvenlik ve failsafe
| ID | Gereksinim |
|---|---|
| S-01 | RC verici her zaman önceliklidir; kill switch ≤ 50 ms'de motorları durdurur |
| S-02 | Companion bilgisayar kaybında (ROS 2 / DDS bağlantısı) PX4 failsafe'i devreye girer (Hold → Land) |
| S-03 | RC + veri linki kaybında RTL; GNSS yoksa yerinde iniş |
| S-04 | Geofence: yarıçap ve irtifa sınırı (varsayılan 300 m / 120 m) |
| S-05 | Kişilere (avuca iniş dışında) ≥ 3 m yatay mesafe; kişiye 3 m içinde hız ≤ 1 m/s |
| S-06 | Batarya: %25'te uyarı + RTL, %15'te zorunlu iniş (Li-ion için gerilim tabanlı) |
| S-07 | Pervane koruması: avuca iniş özelliği yalnızca tam pervane koruması takılıyken etkin |

## 3. Fonksiyonel olmayan gereksinimler

| ID | Gereksinim | Hedef |
|---|---|---|
| N-01 | Algı gecikmesi (kamera → setpoint) | CSI yolu ≤ 60 ms, RTSP (gimbal) yolu ≤ 150 ms |
| N-02 | Algı hızı | Ana tespit ≥ 30 FPS, jest ≥ 15 FPS, avuç ≥ 30 FPS |
| N-03 | Companion güç tüketimi | Ortalama ≤ 25 W |
| N-04 | Kayıt | PX4 ulog + ROS 2 MCAP + 4K video; her uçuş zaman damgalı |
| N-05 | Yazılım güncellemesi | Docker imajları, sürüm etiketli, geri alınabilir |
| N-06 | Lisans | Ticari kullanım için model/veri seti lisansları uygun olmalı (bkz. [03](03-yapay-zeka-modelleri.md)) |
| N-07 | Mevzuat | SHGM İHA kayıt ve operasyon kurallarına uyum (bkz. [08](08-guvenlik-ve-mevzuat.md)) |

## 4. Kapsam dışı (ilk sürüm)
- Tam otonom engel kaçınma ile yüksek hızlı takip (yalnızca ileri yönde fren/durma yapılır).
- Avuçtan kalkış (Faz 4'te değerlendirilecek).
- Ses komutları (pervane gürültüsü nedeniyle drone üzerinde güvenilir değil; yer istasyonunda opsiyonel).
- Gece/termal operasyon.
