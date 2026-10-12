#!/usr/bin/env python3
"""Project readiness check — run before first hardware test."""

from __future__ import annotations

import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
OK = "[OK]"
WARN = "[!!]"
FAIL = "[XX]"


def check(name: str, passed: bool, fix: str = "") -> bool:
    mark = OK if passed else FAIL
    print(f"  {mark} {name}")
    if not passed and fix:
        print(f"       -> {fix}")
    return passed


def main() -> int:
    print("=" * 55)
    print("  ARK-5 PROJECT READINESS CHECK")
    print("=" * 55)

    all_ok = True
    warns = 0

    # --- Folder structure ---
    print("\n[FOLDERS]")
    folders = [
        "config/project.yaml",
        "main_controller.py",
        "swarm_controller.py",
        "wiring/WIRING_GUIDE.txt",
        "esp32/human_detector_robot/human_detector_robot.ino",
        "esp32/camera_stream_2640/camera_stream_2640.ino",
        "esp32/camera_stream_3660/camera_stream_3660.ino",
        "esp32/tests/step3_full_hardware_verify/step3_full_hardware_verify.ino",
        "esp32/tests/step4_motor_verify/step4_motor_verify.ino",
        "dataset/human/data.yaml",
        "debug/debug_robot_sensors.py",
        "scripts/download_yolov8.py",
    ]
    for f in folders:
        p = ROOT.parent / f if f.startswith(("wiring", "esp32")) else ROOT / f
        if not check(f, p.exists(), "File missing -- project incomplete"):
            all_ok = False

    # --- YOLO model ---
    print("\n[MODELS]")
    yolo11n = ROOT / "models" / "yolo11n.pt"
    best = ROOT / "models" / "best.pt"
    if not yolo11n.is_file():
        print(f"  {WARN} models/yolo11n.pt missing")
        print("       -> python scripts/download_yolov8.py")
        warns += 1
    else:
        print(f"  {OK} models/yolo11n.pt ({yolo11n.stat().st_size // 1_000_000} MB)")

    if best.is_file():
        print(f"  {OK} models/best.pt (custom trained)")
    else:
        print(f"  {WARN} models/best.pt missing -- abhi pretrained yolo11n use hoga")
        print("       -> Label + train ke baad best.pt lagao")
        warns += 1

    # --- Dataset ---
    print("\n[DATASET]")
    train_imgs = list((ROOT / "dataset/human/images/train").glob("*.*"))
    train_imgs = [f for f in train_imgs if f.suffix.lower() in (".jpg", ".jpeg", ".png")]
    raw_imgs = list((ROOT / "dataset/human/raw").glob("*.*"))
    raw_imgs = [f for f in raw_imgs if f.suffix.lower() in (".jpg", ".jpeg", ".png")]

    if len(train_imgs) >= 10:
        print(f"  {OK} Train images: {len(train_imgs)}")
    elif len(raw_imgs) > 0:
        print(f"  {WARN} Raw photos: {len(raw_imgs)} -- abhi label karo")
        warns += 1
    else:
        print(f"  {WARN} No training photos yet")
        print("       -> Photos lo -> Label Studio -> export")
        warns += 1

    # --- Config placeholders ---
    print("\n[CONFIG -- user must edit]")
    cfg_text = (ROOT / "config/project.yaml").read_text(encoding="utf-8")
    esp_robot = (ROOT.parent / "esp32/human_detector_robot/config.h").read_text(encoding="utf-8")
    esp_cam2640 = (ROOT.parent / "esp32/camera_stream_2640/config.h").read_text(encoding="utf-8")
    _ = cfg_text

    checks = [
        ("robot config.h -- WiFi creds", "YourHomeWiFi" not in esp_robot,
         "esp32/human_detector_robot/config.h -> STA_SSID + STA_PASSWORD"),
        ("cam2640 config.h -- WiFi creds", "YourHomeWiFi" not in esp_cam2640,
         "esp32/camera_stream_2640/config.h -> WIFI_SSID + WIFI_PASSWORD"),
    ]
    for name, done, fix in checks:
        if done:
            print(f"  {OK} {name}")
        else:
            print(f"  {WARN} {name}")
            print(f"       -> {fix}")
            warns += 1

    print(f"  {WARN} project.yaml -- flash ke baad IPs verify karo (udp_host + stream_url)")
    warns += 1

    # --- Python deps ---
    print("\n[PYTHON DEPS]")
    for pkg in ("cv2", "yaml", "ultralytics"):
        try:
            __import__(pkg if pkg != "cv2" else "cv2")
            print(f"  {OK} {pkg}")
        except ImportError:
            print(f"  {FAIL} {pkg} not installed")
            print("       -> python -m pip install -r requirements.txt")
            all_ok = False

    # --- Summary ---
    print("\n" + "=" * 55)
    if all_ok and warns == 0:
        print("  READY -- hardware flash + debug scripts chalao")
    elif all_ok:
        print(f"  MOSTLY READY -- {warns} warning(s) upar dekho")
        print("  Code/folders OK. WiFi creds + model + dataset baqi hain.")
    else:
        print("  NOT READY -- failed checks fix karo pehle")
    print("=" * 55)
    return 0 if all_ok else 1


if __name__ == "__main__":
    sys.exit(main())
