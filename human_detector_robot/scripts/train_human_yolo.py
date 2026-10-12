#!/usr/bin/env python3
"""
Train custom YOLOv8 model for human standing/fallen detection.

Pehle dataset bharo:
  dataset/human/images/train/
  dataset/human/images/val/
  dataset/human/labels/train/
  dataset/human/labels/val/

Phir run karo:
  python scripts/train_human_yolo.py
"""

from pathlib import Path

def main():
    from ultralytics import YOLO

    root = Path(__file__).resolve().parent.parent
    data_yaml = root / "dataset" / "human" / "data.yaml"

    if not data_yaml.is_file():
        raise FileNotFoundError(f"data.yaml missing: {data_yaml}")

    # Start from pretrained yolo11n (YOLO11 — better than yolov8n)
    weights = root / "models" / "yolo11n.pt"
    model = YOLO(str(weights) if weights.is_file() else "yolo11n.pt")

    results = model.train(
        data=str(data_yaml),
        epochs=50,
        imgsz=640,
        batch=8,
        project=str(root / "runs"),
        name="human_detector",
        exist_ok=True,
    )

    best = root / "runs" / "human_detector" / "weights" / "best.pt"
    if best.is_file():
        dest = root / "models" / "best.pt"
        import shutil
        shutil.copy2(best, dest)
        print(f"\n[DONE] Best model copied to: {dest}")
        print("project.yaml update karo:")
        print("  yolo.weights: best.pt")
        print("  yolo.custom_model: true")

if __name__ == "__main__":
    main()
