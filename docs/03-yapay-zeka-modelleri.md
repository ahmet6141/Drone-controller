# 03 — Yapay Zekâ Modelleri: Analiz ve Seçim

Durum tarihi: **2 Ekim 2026**. Gecikme değerleri aksi belirtilmedikçe 640×640, batch 1,
TensorRT (Jetson) veya HailoRT (Raspberry Pi 5 + AI HAT+ 2) içindir. "tahmini" ibaresi mühendislik tahminidir; diğer değerler kaynaklıdır
(bkz. §11). Model ayarları: [`config/ai/perception.yaml`](../config/ai/perception.yaml).

## 1. Özet: seçilen model yığını

| İşlev | Seviye A — Raspberry Pi 5 + Hailo-10H | Seviye B — Orin NX 16GB | Seviye C — Orin NX 16GB MAXN SUPER |
|---|---|---|---|
| Nesne tespiti | **YOLO26n @640, Hailo HEF (8 bit)**, 15–30 Hz | **YOLO26s @960** (P2 başlığı, ince ayar) — Apache yolu: **RF-DETR-S** | YOLO26m @1280 veya RF-DETR-M, yüksek irtifada SAHI |
| Çoklu nesne takibi (MOT) | ByteTrack + OSNet x0_25 (Hailo) | **BoT-SORT + OSNet x0_25 ReID** | BoT-SORT / OccluBoost + OSNet x1_0 |
| Tek hedef takibi (SOT) | OpenCV VitTrack (CPU) | **LiteTrack-B4** veya SUTrack-T224 | SUTrack-T224 + ORTrack (İHA, örtülmeye dayanıklı) |
| Tıkla → kutu | MOT izi seçimi | NanoSAM | NanoSAM / EdgeTAM |
| El / jest | Kişi → el kırpma → el landmark (Hailo `hand_landmark_lite`) → MLP | **+ RTMPose-m hand (TRT)** | + RTMW tüm vücut |
| Avuç (aşağı kamera) | CM3 Wide (720p120) + el dedektörü + landmark + 8×8 ToF | aynı, OV9281 global shutter | aynı |
| Derinlik / engel | — (8×8 ToF yakın mesafe) | OAK-D Lite donanım derinliği; opsiyonel Light ESS | ESS + Depth Anything 3 Metric |
| VIO | Optik akış + lazer (VIO yok) | **cuVSLAM** stereo-ataletsel | cuVSLAM (geniş açılı stereo) |
| Görsel-dil modeli (VLM) | Yok (Hailo-10H LLM/VLM çalıştırabilir; hız doğrulanmadı) | **Qwen3-VL-4B** veya Gemma 4 E4B, asenkron ≤ 1 Hz | Qwen3-VL-4B / Gemma 4 E4B |

Hailo ölçümleri (Hailo model zoo, batch 1): Hailo-10H'de YOLO26n / YOLO26s **233 / 125 FPS** (x86
konak, PCIe Gen3 x4); Raspberry Pi 5 üzerinde (PCIe x1) Hailo-8 ile YOLO11n **104,9 FPS** (7,75 ms) →
Pi 5'te 30 FPS tespit için yeterli pay. Hailo-8'de `hand_landmark_lite` 3091 FPS, OSNet x1_0 215 FPS.

**Temel ilkeler**
- Kontrol döngüsünde yalnızca **deterministik, hızlı** modeller (tespit, takip, landmark) vardır.
  VLM yalnızca *hedef seçici* olarak asenkron çalışır; çıktısı bir kutudur ve operatör onayından
  sonra takipçiye verilir.
- Hat, model bağımsız bir çıkarım arayüzüyle kurulur (Jetson: TensorRT `.engine`, Pi 5: Hailo
  `.hef`); YOLO26 ↔ RF-DETR değişimi yapılandırma düzeyindedir.
- **JetPack 7.2.x** hedeflenir: JetPack 6'daki TensorRT 10.3 ile NMS'siz (uçtan uca) YOLO26 INT8
  motoru derlenemiyor; Ultralytics bu durumda NMS'li başlığa geri dönüyor.

## 2. Lisans stratejisi (ticari ürün için kritik)

| Durum | Bileşenler |
|---|---|
| ✅ **Ticari kullanıma uygun (Apache-2.0 / MIT)** | RF-DETR N/S/M/L (+Seg), D-FINE, DEIM v1, RT-DETRv4, LW-DETR, Roboflow `trackers`, OSNet (torchreid, MIT), MediaPipe, RTMPose/RTMDet, NanoSAM/NanoOWL, Depth Anything V2-**S**, Depth Anything 3 S/B/Metric-L, Qwen3-VL, Qwen3.5, Gemma 4, Moondream 2 |
| ⚠️ **AGPL-3.0 / ticari lisans gerekir** | Ultralytics YOLO26/YOLO11/YOLOE-26 ve izleyicileri, BoxMOT, YOLOv12, YOLOv13 (gömülü ürün ve eğitilmiş ağırlıklar için Ultralytics Enterprise lisansı gerekir) |
| ⚠️ Özel lisans | cuVSLAM (NVIDIA Community License, ikili), NVIDIA ESS, Cosmos-Reason2 (NVIDIA Open Model License), SAM 2/3 (SAM License) |
| ❌ **Ticari kullanım yasak** | DEIMv2, EdgeCrafter, WiLoR/HaMeR, FoundationStereo, Depth Anything V2 B/L, Qwen2.5-VL-3B (araştırma lisansı), Ultralytics hand-keypoints veri seti (CC BY-NC-SA), DOTA |

**Öneri**: Faz 1–4'te geliştirme hızı için **YOLO26** kullanılır. Kapalı kaynak bir ticari ürüne
geçilecekse ya Ultralytics Enterprise lisansı alınır ya da tespit **RF-DETR**'ye, takip
**Roboflow `trackers` + OSNet**'e taşınır. Hat bu geçişe hazır tasarlanmıştır.

## 3. (a) Nesne tespiti

| Model | Param | COCO AP50:95 | Jetson gecikmesi | Lisans | Çıkış |
|---|---|---|---|---|---|
| **YOLO26 n / s / m** | 2,4 / 9,5 / 20,4 M | 40,9 / 48,6 / 53,1 | Orin Nano Super: n FP16 4,57 ms, INT8 3,80; s FP16 7,17, INT8 5,25 · Orin NX 16GB: n 4,13 / 3,49; s 6,41 / 4,78 | AGPL / Enterprise | 14.01.2026 (Ultralytics 8.4.0) |
| YOLO11 n / s | 2,6 / 9,4 M | 39,5 / 47,0 | Orin Nano Super: s INT8 5,08 ms; 15 W modunda 145,8 FPS @ 11,1 W | AGPL | 2024 |
| **RF-DETR N / S / M / L** | 30,5–33,9 M | 48,4 / 53,0 / 54,7 / 56,5 | N: Orin NX 16GB 6,0 ms FP16 (JP 7.2); Base 560²: Orin Nano Super 14,8 ms FP16 | **Apache-2.0** | 2025-03; v1.11.1 (30.09.2026) |
| D-FINE N / S / M | 4 / 10 / 19 M | 42,8 / 48,5 / 52,3 | T4: 2,1 / 3,5 / 5,6 ms | Apache | ICLR 2025 |
| RT-DETRv4 S / M | — | 49,8 / 53,7 | T4: 3,7 / 5,9 ms | Apache | 2025-10 |
| YOLO27 | — | 42,3–60,4 | — | belirsiz | **Duyuruldu (13.09.2026), yayımlanmadı** |

**Havadan görüş notları**
- Küçük nesneler zor kalmaya devam ediyor: VisDrone'da nesnelerin %75'inden fazlası < 2000 px².
  → 960–1280 giriş, P2 (yüksek çözünürlüklü) başlık, yüksek irtifada SAHI döşeme (tiling).
- INT8 kalibrasyonu: hedef cihazda, uçuş irtifasında çekilmiş, hareket bulanıklığı içeren
  ≥ 500 temsilî kare ile; INT8 sonrası güven eşikleri yeniden ayarlanır
  (ör. YOLO26n Orin Nano Super: 0,480 → 0,449 mAP).
- Hazır havadan ağırlık yok; VisDrone/UAVDT ile ince ayar + kendi uçuş verimiz.

## 4. (b) Takip

### 4.1 Çoklu nesne takibi (MOT) — BoxMOT ortak tespitlerle MOT17 karşılaştırması
| İzleyici | HOTA / IDF1 | ReID | Kütüphane |
|---|---|---|---|
| OccluBoost | 71,10 / 85,28 | Evet | BoxMOT (AGPL) |
| **BoT-SORT** | 69,68 / 82,33 | Opsiyonel | BoxMOT, Ultralytics, Roboflow |
| BoostTrack | 69,25 / 83,20 | Evet | BoxMOT |
| Deep OC-SORT | 67,95 / 80,54 | Evet | BoxMOT, Ultralytics |
| **ByteTrack** | 67,68 / 79,16 | Hayır | hepsi |
| TrackTrack (CVPR 2025) | — | Opsiyonel | Ultralytics 8.4.63+ varsayılanı |

ReID: **OSNet x0_25** (0,2 M param, Market-1501 Rank-1 91,2; MIT) — Orin'de < 1 ms (tahmini);
OSNet x1_0 (Rank-1 94,2). Kapalı kaynak için: Roboflow `trackers` (Apache, ReID dalı yok) +
torchreid OSNet ile kendi ReID ilişkilendirmemiz.

### 4.2 Tek hedef takibi (SOT)
| İzleyici | LaSOT AUC | Kenar hızı | Not |
|---|---|---|---|
| HiT-S | 60,4 | Xavier NX ~34 FPS | ICCV 2023 |
| AsymTrack-T | 60,8 | AGX Xavier 84 FPS | AAAI 2025 |
| MaST-tiny | 63,8 | "Jetson Nano" 152 FPS (Orin Nano olduğu doğrulanmadı) | ECCV 2026 |
| **LiteTrack-B4** | 62,5–67 | **Orin NX 16GB > 100 FPS (ONNX)** | ICRA 2024 |
| SUTrack-T224 | 69,6 | AGX Xavier 34 FPS | AAAI 2025 |
| ORTrack | UAVDT kesinlik 83,4 | AGX Xavier 38 FPS | CVPR 2025, İHA + örtülme |
| OpenCV VitTrack | — | Orin Nano CPU ~19 ms | Yedek, bağımlılıksız |

## 5. (c) El ve jest tanıma

| Bileşen | Özellik | Lisans |
|---|---|---|
| MediaPipe Hand Landmarker | Avuç dedektörü + 21 nokta | Apache |
| MediaPipe Gesture Recognizer | Sınıflar: None, Closed_Fist, **Open_Palm**, Pointing_Up, Thumb_Down, **Thumb_Up**, **Victory**, ILoveYou; Model Maker ile özel sınıf. `mediapipe` 1.0.1 (14.08.2026) aarch64 tekerlekleri var | Apache |
| **HaGRIDv2** | 1 086 158 FullHD görüntü, 33 jest + no_gesture, 65 977 kişi, **0,5–4 m** mesafe, MediaPipe landmark'ları dahil | CC BY-SA 4.0 varyantı |
| RTMDet-nano hand / RTMPose-m hand | 320² dedektör, 256² poz | Apache |

> MediaPipe'ın Linux GPU yolu OpenGL ES 3.1/EGL ister; Jetson'da GPU delegesi **doğrulanmadı**.
> Plan: CPU (XNNPACK) yolu veya Apache lisanslı ONNX portu → TensorRT.

### 5.1 Uzak mesafede jest (ön kamera, 1–5 m)
3 m'de 1080p / 90° yatay görüşte 9 cm'lik avuç ≈ 29 piksel → tam kare avuç dedektörü kaçırır.
Bu yüzden **iki aşamalı kırpma**:
1. YOLO26(-pose) ile kişi (ve bilek noktaları),
2. üst gövde ROI'si üzerinde 320–416 px el dedektörü (HaGRIDv2 ile eğitilmiş),
3. el kırpmasında 21 nokta landmark,
4. normalize landmark'lar üzerinde küçük MLP sınıflandırıcı (`open_palm`, `fist`, `thumbs_up`, `peace`, `none`),
5. N/M kare onayı + histerezis ([`behavior.yaml`](../config/mission/behavior.yaml) → `gestures`).

### 5.2 Avuç konumu (aşağı kamera, 0,1–1 m)
- **Avuç merkezi** = landmark 0, 5, 9, 13, 17 ortalaması; **avuç normali** = (p5 − p0) × (p17 − p0).
- Mesafe: ToF (birincil) + kullanıcıya göre kalibre edilen avuç genişliği (MCP5–MCP17).
- Global shutter, kısa pozlama; bulanıklık/düşük ışık artırımıyla eğitim. Tek renkli (mono) kamera
  kullanılıyorsa eğitim verisi gri tonlamaya çevrilir.
- Özel veri ihtiyacı: aşağı kameradan 20–50 bin kare; farklı ten renkleri, eldiven, zemin, ışık.

### 5.3 Operatör kilidi
Uçuşta yüz doğrulama hata oranı ciddi artıyor (bir 2026 çalışmasında EER %0,32 → %19,3). Bu
yüzden kimlik kilidi yüz tanımaya değil; **ReID gömmesi + uzamsal tutarlılık + RC izin anahtarına**
dayanır.

## 6. (d) Derinlik / engel algılama

| Model | Hız | Lisans | Kullanım |
|---|---|---|---|
| Stereo kameranın **donanım derinliği** (OAK-D, Orbbec) | GPU yükü yok | — | Seviye B/C birincil |
| NVIDIA Light ESS / ESS | Orin Nano Super 82,2 / 34,8 FPS | NVIDIA | Seviye B/C opsiyonel |
| Depth Anything V2-S | ~44 FPS "Jetson Orin" | Apache | Tek kamera yedeği |
| Depth Anything 3 S/B, Metric-L | Orin NX ~23 ms (topluluk) | Apache (S/B/Metric-L) | Seviye C |
| FoundationStereo | Thor 0,65 FPS | Ticari değil | ❌ |

## 7. (e) Görsel-ataletsel odometri (VIO)

| Sistem | Girdi | Performans | Lisans |
|---|---|---|---|
| **cuVSLAM** (Isaac ROS 5.0: `isaac_ros_cuvslam`) | mono/stereo/RGB-D, 32 kameraya kadar, IMU | AGX Orin stereo-ataletsel 3,8 ms; EuRoC APE 0,054 m | NVIDIA Community (ikili) |
| OpenVINS | mono/stereo + IMU | CPU | GPL-3.0 |
| Basalt | stereo + IMU | CPU'da verimli | BSD-3 |
| ORB-SLAM3 | mono/stereo/RGB-D + IMU | CPU | GPL-3.0 |

cuVSLAM şartları: ≥ 30 Hz, kameralar arası ±100 µs senkron, donanım zaman damgası.
PX4'e besleme: uXRCE-DDS `/fmu/in/vehicle_visual_odometry`, **30–50 Hz**;
`EKF2_EV_CTRL`, `EKF2_EV_DELAY`, `EKF2_EV_POS_X/Y/Z` (bkz. [`config/px4`](../config/px4/)).

## 8. (f) Görsel-dil modelleri (VLM) — hedef seçici

| Model | Grounding (kutu) | Orin performansı | Lisans |
|---|---|---|---|
| **Qwen3-VL-2B / 4B** | Evet (0–1000 göreli koordinat); RefCOCO 84,8 / 88,2 | 2B, Orin Nano Super: 0,5–0,9 sorgu/s | Apache |
| **Cosmos-Reason2-2B** | Nokta + kutu | Orin Nano Super: 38–60 token/s, görüntü TTFT 75–114 ms | NVIDIA Open Model |
| **Gemma 4 E2B / E4B** (02.04.2026) | "nesne tespiti, işaretleme" | Nano: yalnızca E2B; NX: E2B/E4B | Apache |
| Qwen3.5-2B/4B (02.03.2026) | RefCOCO 84,8 (2B) | doğrulanmadı | Apache |
| Moondream 2 (0,5B) | Tespit + işaretleme | int4 816 MiB | Apache |

Kullanım: "Kırmızı montlu kişiyi takip et" → hover sırasında tek anahtar kare → kutu → MOT izi ile
eşleme → GCS'de onay → takip. Hiçbir zaman doğrudan setpoint üretmez.

## 9. Uçtan uca hat ve gecikme bütçesi (30 FPS = 33,3 ms/kare)

| Aşama | Orin NX 16GB (ms) | Raspberry Pi 5 + Hailo-10H (ms, tahmini) |
|---|---|---|
| Ön işleme (VPI / PiSP + CPU) | 1–2 | 2–4 |
| Tespit (YOLO26s FP16 / YOLO26n Hailo) | ~6 | ~5–10 |
| OSNet x0_25 (≤ 16 kırpma) | 1–2 | ~2–4 (Hailo) |
| MOT ilişkilendirme (CPU) | ~1 | ~1–2 |
| SOT (TRT / OpenCV VitTrack CPU) | 4–6 | ~15–20 (ayrı çekirdekte, 15 Hz) |
| El ROI hattı | 3–4 | 4–6 (Hailo) |
| Stereo DNN (opsiyonel; 10–15 Hz, kare başına amortize) | ~3 | — |
| cuVSLAM (ayrı iş parçacığı, kare başına GPU payı) | 2–3 | — (VIO yok) |
| **Toplam / kare** | **~22–27 → sığar** | **~15–25 hızlandırıcı + CPU paralel → 15–30 FPS** |

Pi 5'te tespit Hailo'da, SOT ve izleme CPU'da paralel çalışır; tespit 15 Hz'e indirilip SOT ile
dönüşümlü çalıştırılabilir. VLM sorguları (yalnızca Jetson) asenkron (0,3–1,9 s).

## 10. Eğitim ve veri planı

| Veri seti | Kullanım | Lisans |
|---|---|---|
| COCO / Objects365 (ön eğitim ağırlıkları) | Genel tespit | CC BY 4.0 (COCO açıklamaları) |
| VisDrone2019 (DET/MOT/SOT) | Havadan ince ayar, değerlendirme | Lisans belirtilmemiş → araştırma kabul edin |
| UAVDT, UAV123, LaSOT | Takip değerlendirme | doğrulanmadı |
| **HaGRIDv2** | Jest + el tespiti | CC BY-SA 4.0 varyantı |
| Market-1501, MOT17 | ReID / MOT değerlendirme | doğrulanmadı |
| **Kendi uçuş verimiz** | Ana ince ayar kaynağı (ticari açıdan temiz) | Proje mülkiyeti |

Hat: veri → etiketleme (SAM 3 ile çevrimdışı otomatik etiket + insan kontrolü) → eğitim (PC/bulut)
→ ONNX → **hedef cihazda** TensorRT (FP16 → INT8) → saha metrikleri → sürüm etiketi
(`models/<model>-<veri>-<tarih>.engine`).

## 11. Kaynaklar (seçme)

- YOLO26: https://docs.ultralytics.com/models/yolo26/ · Jetson ölçümleri: https://docs.ultralytics.com/guides/nvidia-jetson/
- Ultralytics lisansı: https://www.ultralytics.com/license · YOLO27 duyurusu: https://docs.ultralytics.com/models/yolo27
- RF-DETR: https://github.com/roboflow/rf-detr · D-FINE: https://github.com/Peterande/D-FINE · RT-DETRv4: https://github.com/RT-DETRs/RT-DETRv4
- DEIMv2 lisansı: https://github.com/Intellindust-AI-Lab/DEIMv2
- BoxMOT: https://github.com/mikel-brostrom/boxmot · Roboflow trackers: https://github.com/roboflow/trackers · Ultralytics takip: https://docs.ultralytics.com/modes/track/
- OSNet: https://github.com/KaiyangZhou/deep-person-reid · LiteTrack: https://arxiv.org/abs/2309.09249 · ORTrack: https://arxiv.org/abs/2504.09228 · SUTrack: https://github.com/chenxin-dlut/SUTrack
- MediaPipe: https://developers.google.com/edge/mediapipe/solutions/vision/gesture_recognizer · HaGRID: https://github.com/hukenovs/hagrid · RTMPose: https://github.com/open-mmlab/mmpose/tree/main/projects/rtmpose
- Isaac ROS sürümleri: https://nvidia-isaac-ros.github.io/releases/index.html · cuVSLAM: https://arxiv.org/abs/2506.04359
- PX4 harici konum kestirimi: https://docs.px4.io/main/en/ros/external_position_estimation.html
- Qwen3-VL: https://arxiv.org/abs/2511.21631 · Cosmos-Reason2: https://huggingface.co/nvidia/Cosmos-Reason2-2B · Gemma 4: https://ai.google.dev/gemma/docs/core/model_card_4
- Depth Anything 3: https://github.com/ByteDance-Seed/Depth-Anything-3 · ESS: https://nvidia-isaac-ros.github.io/repositories_and_packages/isaac_ros_dnn_stereo_depth/index.html
- Jest kontrollü drone (2026): https://arxiv.org/abs/2609.25511 · Avuca iniş (falconry, 2025): https://arxiv.org/abs/2507.17144
