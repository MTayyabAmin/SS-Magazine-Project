# ARK-5 Human Detector Robot

Autonomous robot jo teen tareeqon se human detect karta hai:

1. **Computer Vision (YOLOv8)** — laptop par; ESP32-CAM sirf video stream
2. **WiFi RSSI sensing** — cardboard wall ke peeche human
3. **Ultrasonic** — wall/obstacle 1m pehle stop

## Debug Scripts

Step-by-step sensor/link testing: [`debug/README.md`](debug/README.md)

```bash
python debug/debug_robot_sensors.py   # MPU + ultrasonic + RSSI
python debug/debug_cam_stream.py        # ESP32-CAM FPS & speed
python debug/debug_udp_link.py          # WiFi UDP latency
python debug/debug_all.py               # combined dashboard
```

## Quick Start

### 1. Wiring
Poori wiring guide: [`../wiring/WIRING_GUIDE.txt`](../wiring/WIRING_GUIDE.txt)

### 2. ESP32 Flash
```
esp32/robot_controller/   → Robot ESP32 DevKit (motors, MPU, ultrasonic)
esp32/esp32_cam_stream/   → ESP32-CAM OV3660 (MJPEG stream)
esp32/tests/step1_mpu6050_verify.ino → MPU test
```

Dono devices ka `config.h` mein router SSID/password daalo.

### 3. Laptop Setup
```bash
cd human_detector_robot
pip install -r requirements.txt
python scripts/download_yolov8.py
```

### 4. Config
`config/project.yaml` mein update karo:
- `camera.stream_url` → ESP32-CAM IP (e.g. `http://192.168.1.105:81/stream`)
- `robot.udp_host` → Robot ESP32 IP

### 5. Run
```bash
python main_controller.py                  # full autonomous
python main_controller.py --no-motors      # vision test only
python main_controller.py --calibrate-wifi # empty room WiFi baseline
```

## Human Model Training

**Full guide:** [`dataset/human/TRAINING_GUIDE.md`](dataset/human/TRAINING_GUIDE.md)

Quick flow (ARK-3 jaisa):
1. Photos → `dataset/human/raw/`
2. **Label Studio** → label `standing` / `fallen` → Export **YOLO**
3. `python scripts/prepare_labelstudio_export.py --zip export.zip`
4. `python scripts/make_colab_zip.py` → upload to Colab
5. `scripts/colab/train_human_yolov8.ipynb` on **T4 GPU**
6. `best.pt` → `models/` + update `project.yaml`

```bash
python scripts/train_human_yolo.py   # local train (no Colab)
```

## Project Structure

```
ARK_5/
├── wiring/WIRING_GUIDE.txt       ← hardware connections
├── esp32/
│   ├── robot_controller/       ← motors + IMU + ultrasonic + WiFi
│   ├── esp32_cam_stream/         ← OV3660 MJPEG stream
│   └── tests/                    ← step-by-step bring-up
└── human_detector_robot/         ← laptop brain (YOLOv8 + FSM)
    ├── config/project.yaml
    ├── main_controller.py
    ├── models/                   ← yolov8n.pt, best.pt
    ├── dataset/human/            ← training data
    ├── scripts/
    └── utils/
```

## YOLOv5 vs YOLOv8

Is project mein **YOLOv8** use ho raha hai (ultralytics):
- Faster, better accuracy, easier training API
- `yolov8n.pt` pretrained download ho chuka hai
- Custom classes: `standing`, `fallen`
