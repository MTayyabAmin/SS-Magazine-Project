#!/usr/bin/env python3
"""
Label Studio YOLO export → train/val folders (80/20 split).

Usage:
  python scripts/prepare_labelstudio_export.py --zip ~/Downloads/ARK5_Human_yolo_export.zip

Label Studio export format expected:
  images/*.jpg
  labels/*.txt   (same basename as images)
  classes.txt    (optional)
"""

from __future__ import annotations

import argparse
import random
import shutil
import zipfile
from pathlib import Path


def find_image_label_pairs(src: Path) -> list[tuple[Path, Path]]:
    """Find image/label pairs in Label Studio YOLO export layout."""
    img_dirs = [src / "images", src]
    label_dirs = [src / "labels", src / "labels/train", src]

    images: dict[str, Path] = {}
    for d in img_dirs:
        if not d.is_dir():
            continue
        for ext in ("*.jpg", "*.jpeg", "*.png", "*.JPG", "*.PNG"):
            for p in d.glob(ext):
                images[p.stem] = p

    pairs: list[tuple[Path, Path]] = []
    for stem, img_path in images.items():
        label_path = None
        for ld in label_dirs:
            for ext in (".txt",):
                candidate = ld / f"{stem}{ext}"
                if candidate.is_file():
                    label_path = candidate
                    break
            if label_path:
                break
        if label_path:
            pairs.append((img_path, label_path))
        else:
            print(f"[WARN] No label for: {img_path.name} — skipped")

    return pairs


def main() -> None:
    parser = argparse.ArgumentParser(description="Prepare Label Studio export for YOLOv8 training")
    parser.add_argument("--zip", required=True, help="Label Studio YOLO export zip path")
    parser.add_argument("--val-ratio", type=float, default=0.2, help="Validation split (default 0.2)")
    parser.add_argument("--seed", type=int, default=42)
    args = parser.parse_args()

    root = Path(__file__).resolve().parent.parent
    dataset = root / "dataset" / "human"
    zip_path = Path(args.zip)

    if not zip_path.is_file():
        raise FileNotFoundError(f"Zip not found: {zip_path}")

    extract_dir = dataset / "_labelstudio_import"
    if extract_dir.exists():
        shutil.rmtree(extract_dir)
    extract_dir.mkdir(parents=True)

    print(f"[1/4] Extracting {zip_path.name}...")
    with zipfile.ZipFile(zip_path, "r") as z:
        z.extractall(extract_dir)

    pairs = find_image_label_pairs(extract_dir)
    if not pairs:
        # try one level deeper
        for sub in extract_dir.iterdir():
            if sub.is_dir():
                pairs = find_image_label_pairs(sub)
                if pairs:
                    break

    if not pairs:
        raise RuntimeError("No image/label pairs found. Label Studio se YOLO format export karo.")

    print(f"[2/4] Found {len(pairs)} labeled images")

    random.seed(args.seed)
    random.shuffle(pairs)
    n_val = max(1, int(len(pairs) * args.val_ratio))
    val_pairs = pairs[:n_val]
    train_pairs = pairs[n_val:]

    for sub in ("images/train", "images/val", "labels/train", "labels/val"):
        d = dataset / sub
        if d.exists():
            for f in d.iterdir():
                if f.is_file():
                    f.unlink()
        d.mkdir(parents=True, exist_ok=True)

    def copy_pairs(split_pairs: list[tuple[Path, Path]], split: str) -> None:
        for img, lbl in split_pairs:
            shutil.copy2(img, dataset / "images" / split / img.name)
            shutil.copy2(lbl, dataset / "labels" / split / lbl.name)

    print(f"[3/4] Split: train={len(train_pairs)}, val={len(val_pairs)}")
    copy_pairs(train_pairs, "train")
    copy_pairs(val_pairs, "val")

    shutil.rmtree(extract_dir, ignore_errors=True)

    print("[4/4] Done! Dataset ready at:", dataset)
    print("\nNext steps:")
    print("  1. Zip dataset/human/ for Colab (include data.yaml)")
    print("  2. Or local: python scripts/train_human_yolo.py")
    print("  3. See dataset/human/TRAINING_GUIDE.md for Colab steps")


if __name__ == "__main__":
    main()
