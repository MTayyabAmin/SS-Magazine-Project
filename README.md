# ARK-5 — Autonomous Human Detector Robot

## Active Project
**`human_detector_robot/`** — laptop brain (YOLOv8 + autonomous FSM)

## Hardware Wiring
**`wiring/WIRING_GUIDE.txt`** — complete pin diagram + network setup

## ESP32 Firmware
| Folder | Device | Purpose |
|--------|--------|---------|
| `esp32/robot_controller/` | ESP32 DevKit | Motors, MPU6050, ultrasonic, WiFi STA+AP |
| `esp32/esp32_cam_stream/` | ESP32-CAM OV3660 | MJPEG stream via router |
| `esp32/tests/` | ESP32 DevKit | Step-by-step bring-up tests |

## Legacy (removed)
Old trash-catcher folder deleted — use `human_detector_robot/` only.

## Start Here
1. Read `wiring/WIRING_GUIDE.txt`
2. Flash both ESP32 boards (update WiFi creds in config.h)
3. `cd human_detector_robot && python scripts/download_yolov8.py`
4. Edit `config/project.yaml` with ESP32 IPs
5. `python main_controller.py`
