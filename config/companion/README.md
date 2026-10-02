# Companion (Jetson) kurulumu

Hedef yazılım yığını (Ekim 2026): **JetPack 7.2.1** (Ubuntu 24.04, CUDA 13.2, TensorRT 10.16),
**ROS 2 Jazzy**, **Isaac ROS 4.6**, **Micro XRCE-DDS Agent v2.4.x**, PX4 **v1.17** ile eşleşen
`px4_msgs`. Gerekçeler: [docs/02 §6.3](../../docs/02-donanim-analizi.md).

| Dosya | Hedef konum | Görev |
|---|---|---|
| `dc7.env` | `/etc/default/dc7` | Seviye, seri port, ROS alan kimliği, güç kipi |
| `systemd/micro-xrce-dds-agent.service` | `/etc/systemd/system/` | PX4 ↔ ROS 2 köprüsü (FC TELEM2) |
| `systemd/dc7-power.service` | `/etc/systemd/system/` | `nvpmodel` + `jetson_clocks` |
| `mavlink-router/main.conf` | `/etc/mavlink-router/main.conf` | FC TELEM1 → QGC (IP link) + yerel uygulamalar |
| `network/10-dc7-ethernet.yaml` | `/etc/netplan/` | Gimbal + link alt ağı (192.168.144.0/24) |

## Adımlar

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
