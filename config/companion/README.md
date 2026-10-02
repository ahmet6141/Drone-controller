# Companion kurulumu (Jetson ve Raspberry Pi 5)

Hedef yazılım yığını (Ekim 2026): **JetPack 7.2.1** (Ubuntu 24.04, CUDA 13.2, TensorRT 10.16),
**ROS 2 Jazzy**, **Isaac ROS 4.6**, **Micro XRCE-DDS Agent v2.4.x**, PX4 **v1.17** ile eşleşen
`px4_msgs`. Gerekçeler: [docs/02 §6.3](../../docs/02-donanim-analizi.md).

| Dosya | Hedef konum | Görev |
|---|---|---|
| `dc7.env` | `/etc/default/dc7` | **Jetson** (B/C): seviye, seri portlar, ROS alan kimliği, güç kipi |
| `dc7-rpi.env` | `/etc/default/dc7` | **Raspberry Pi 5** (A): seviye, UART'lar, ROS alan kimliği |
| `systemd/micro-xrce-dds-agent.service` | `/etc/systemd/system/` | PX4 ↔ ROS 2 köprüsü (FC TELEM2) |
| `systemd/dc7-power.service` | `/etc/systemd/system/` | `nvpmodel` + `jetson_clocks` (yalnızca Jetson) |
| `mavlink-router/main.conf` | `/etc/mavlink-router/main.conf` | FC TELEM1 → QGC (IP link) + yerel uygulamalar |
| `network/10-dc7-ethernet.yaml` | `/etc/netplan/` | Gimbal + link alt ağı (192.168.144.0/24) |

## Jetson (Seviye B/C) adımları

1. **JetPack 7.2.1** kurun (Orin NX: SDK Manager; Orin Nano geliştirme kiti: USB'den ISO —
   7.2 ile SD kart imajı kalktı). Kurulumdan sonra `cat /etc/nv_tegra_release` ile sürümü doğrulayın.
2. **Güç kipi**: `/etc/nvpmodel.conf` içinden MAXN_SUPER kip numarasını bulun, `dc7.env` →
   `DC7_NVPMODEL_MODE`. 30 dk tam yük ısıl testi yapmadan `jetson_clocks` açmayın.
3. **ROS 2 Jazzy + Isaac ROS 4.6** (apt depoları, NVIDIA Isaac ROS belgeleri). PX4 v1.17 belgesi
   resmi platform olarak Humble/Ubuntu 22.04'ü önerir (v1.18 belgesi Jazzy'yi); `px4_msgs` /
   `px4-ros2-interface-lib` Jazzy'de sorun çıkarırsa bu paketler Humble Docker kapsayıcısında çalıştırılır.
4. **Micro XRCE-DDS Agent v2.4.x** kaynaktan:
   ```bash
   git clone -b v2.4.3 https://github.com/eProsima/Micro-XRCE-DDS-Agent.git
   cd Micro-XRCE-DDS-Agent && mkdir build && cd build && cmake .. && make -j$(nproc) && sudo make install && sudo ldconfig
   ```
5. **ROS 2 çalışma alanı**: `px4_msgs` (PX4 v1.17 ile eşleşen dal) + `px4-ros2-interface-lib`
   (özel uçuş modları; PX4'te hâlâ "deneysel"). Mesaj sürümleri FC firmware'iyle aynı olmalı.
6. **UART**: FC TELEM2 → `DC7_DDS_SERIAL`, FC TELEM1 → `mavlink-router` `Device`. Bu UART'larda
   seri konsol kapalı olmalı; kullanıcı `dialout` grubunda olmalı. Hızlar PX4 `SER_TEL2_BAUD` /
   `SER_TEL1_BAUD` ile aynı (921600; testler kontrol eder).
7. **Ağ**: `sudo cp network/10-dc7-ethernet.yaml /etc/netplan/ && sudo netplan apply`;
   `ping 192.168.144.25` (gimbal). Video testi:
   `gst-launch-1.0 rtspsrc location=rtsp://192.168.144.25:8554/main.264 ! rtph264depay ! h264parse ! nvv4l2decoder ! fakesink`
8. **Servisler**:
   ```bash
   sudo cp dc7.env /etc/default/dc7
   sudo cp systemd/*.service /etc/systemd/system/
   sudo systemctl daemon-reload && sudo systemctl enable --now dc7-power micro-xrce-dds-agent
   ros2 topic list | grep /fmu/      # PX4 konuları görünmeli
   ```
9. **Kalibrasyon**: aşağı kamera iç parametreleri ve kamera–IMU dış parametreleri (Kalibr);
   sonuç `/opt/dc7/calib/`. ToF ve kamera montaj ofsetleri ölçülüp kaydedilir.

## Notlar
- Orin Nano'da donanım video kodlayıcı (NVENC) **yoktur**; YZ katmanlı video yeniden kodlaması
  CPU'da yapılır (720p önerilir) veya kutular metaveri olarak gönderilir.
- Model motorları (`.engine`) git'e girmez; hedef cihazda derlenir: `/opt/dc7/models/`.

## Raspberry Pi 5 (Seviye A) adımları

1. **Raspberry Pi OS (64-bit)** kurun ve güncelleyin. Pi kameraları (libcamera/PiSP) ve AI HAT+ 2
   (Hailo) bu işletim sisteminde yerel olarak desteklenir; ROS 2 **Jazzy** Docker kapsayıcısında
   çalışır (`/dev/ttyAMA*`, `/dev/i2c-*`, `/dev/spidev*`, `/dev/media*`, `/dev/video*`, Hailo aygıtı
   kapsayıcıya verilir). Alternatif: Ubuntu 24.04 + yerel ROS 2 (kamera için Raspberry Pi'nin
   libcamera çatalı gerekir).
2. **Kameralar**: Pi 5'in iki 22 pinli MIPI girişi → gimbal kamerası (Camera Module 3) ve aşağı
   kamera (Camera Module 3 Wide); 15 pin kameralar için "standart–mini" kablo. Kontrol:
   `rpicam-hello --list-cameras` (iki kamera görünmeli).
3. **AI HAT+ 2 (Hailo-10H)**: Raspberry Pi AI HAT+ 2 belgesindeki Hailo paketlerini kurun; modeller
   `.hef` biçiminde (`/opt/dc7/models`).
4. **UART'lar** (`/boot/firmware/config.txt`, aygıt adlarını overlay belgesinden doğrulayın):
   `dtparam=uart0=on` (FC TELEM2, DDS), `dtoverlay=uart2-pi5` (gimbal), `dtoverlay=uart3-pi5`
   (FC TELEM1, MAVLink). Seri konsolu kapatın (`raspi-config`). Değerler: `dc7-rpi.env`.
5. **I2C/SPI**: VL53L8CX ve VL53L1X'in varsayılan I2C adresi aynıdır (7 bit 0x29) → ya VL53L8CX'i
   **SPI** üzerinden bağlayın (hem daha hızlı), ya da açılışta LPn/XSHUT ile birinin adresini değiştirin.
6. **Güç**: 5 V / 5 A BEC; USB'den beslenen cihazlar (WFB-ng adaptörü) için `config.txt` içinde
   USB akım sınırını yükseltin (`usb_max_current_enable=1`, doğrulayın). Pi 5 0–70 °C'de çalışır;
   kapalı gövdede Active Cooler + hava akışı şart.
7. **Micro XRCE-DDS Agent v2.4.3** (Jetson ile aynı adım) + `micro-xrce-dds-agent.service`
   (`dc7-power.service` Pi'de kullanılmaz). `mavlink-router` → `Device = /dev/ttyAMA3`.
8. **Video**: Pi 5'te donanım H.264/HEVC kodlayıcı yoktur; YZ katmanlı akış 720p'de yazılımla kodlanır.
