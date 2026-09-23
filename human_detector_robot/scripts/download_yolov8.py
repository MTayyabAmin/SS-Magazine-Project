#!/usr/bin/env python3
"""Download YOLO11 pretrained weights to models/ folder.

YOLO11 (Sep 2024) — Ultralytics flagship, better than YOLOv8 in accuracy + efficiency.

Models:
  yolo11n.pt  — Nano  (2.6M params, 39.5 mAP) ← CPU real-time ke liye (DEFAULT)
  yolo11s.pt  — Small (9.4M params, 47.0 mAP) ← CPU + better accuracy
  yolo11x.pt  — XLarge(56.9M params, 54.7 mAP)← GPU ke liye (sabse accurate)
"""

from pathlib import Path


def main():
    from ultralytics import YOLO

    root = Path(__file__).resolve().parent.parent
    models_dir = root / "models"
    models_dir.mkdir(exist_ok=True)

    # Default: yolo11n.pt (CPU real-time ke liye best)
    # GPU ho to yolo11x.pt uncomment karo
    models_to_download = [
        "yolo11n.pt",   # CPU default
        # "yolo11s.pt", # Small — thori slow, zyada accurate
        # "yolo11x.pt", # XLarge — GPU chahiye, sabse accurate (54.7 mAP)
    ]

    for name in models_to_download:
        dest = models_dir / name
        if dest.is_file():
            print(f"  Already exists: {dest}")
            continue
        print(f"[DOWNLOAD] {name} ...")
        model = YOLO(name)
        src = Path(getattr(model, "ckpt_path", "") or "")
        if src.is_file():
            import shutil
            shutil.copy2(src, dest)
            print(f"  Saved → {dest}")
        else:
            print(f"  Cached by ultralytics at: {src}")

    print("\n[DONE] YOLO11 weights ready in models/")
    print("Custom training ke baad best.pt ko models/ mein copy karo")
    print("aur project.yaml mein: yolo.weights: best.pt, custom_model: true")
    print("\nGPU users: yolo11x.pt ke liye project.yaml mein weights: yolo11x.pt karo")


if __name__ == "__main__":
    main()
