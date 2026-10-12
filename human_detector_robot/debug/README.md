# Debug Scripts — Step-by-step hardware check

Har sensor / link alag test karo before full `main_controller.py`.

| Script | Kya check karta hai |
|--------|---------------------|
| `debug_robot_sensors.py` | MPU yaw, ultrasonic dist_cm, WiFi RSSI, wall_near |
| `debug_cam_stream.py` | ESP32-CAM FPS, read latency, frame size |
| `debug_udp_link.py` | Laptop ↔ ESP32 UDP RTT, packet loss, telemetry Hz |
| `debug_wifi_rssi.py` | Human sensing RSSI baseline vs drop |
| `debug_all.py` | Sab ek saath dashboard |

## Order (recommended)

```bash
cd human_detector_robot

# 1. Robot ESP32 flash ke baad — sensors
python debug/debug_robot_sensors.py

# 2. ESP32-CAM flash ke baad — video stream speed
python debug/debug_cam_stream.py --preview

# 3. UDP link quality
python debug/debug_udp_link.py

# 4. WiFi human sensing (empty room first)
python debug/debug_wifi_rssi.py --calibrate

# 5. Full combined
python debug/debug_all.py
```

## ESP32-side tests (Arduino Serial Monitor)

Flash from `esp32/tests/`:
- `step1_mpu6050_verify.ino` — MPU raw readings
- `step2_ultrasonic_verify.ino` — HC-SR04 distance

IP addresses `config/project.yaml` mein update karo pehle.
