#!/usr/bin/env python3
"""Create human_dataset.zip for Google Colab upload."""

from __future__ import annotations

import zipfile
from pathlib import Path

def main():
    root = Path(__file__).resolve().parent.parent
    dataset = root / "dataset" / "human"
    out = root / "human_dataset.zip"

    required = [
        dataset / "data.yaml",
        dataset / "images" / "train",
        dataset / "images" / "val",
        dataset / "labels" / "train",
        dataset / "labels" / "val",
    ]
    for p in required:
        if not p.exists():
            raise FileNotFoundError(f"Missing: {p}\nPehle photos label karo — TRAINING_GUIDE.md dekho")

    train_imgs = list((dataset / "images" / "train").glob("*"))
    if len(train_imgs) < 5:
        raise RuntimeError(f"Only {len(train_imgs)} train images — pehle aur photos label karo")

    with zipfile.ZipFile(out, "w", zipfile.ZIP_DEFLATED) as zf:
        for folder in ["images/train", "images/val", "labels/train", "labels/val"]:
            for f in (dataset / folder).iterdir():
                if f.is_file():
                    zf.write(f, f"human_dataset/{folder}/{f.name}")
        zf.write(dataset / "data.yaml", "human_dataset/data.yaml")

    print(f"[OK] Created: {out}")
    print(f"     Train images: {len(train_imgs)}")
    print("     Upload this zip to Google Colab notebook")


if __name__ == "__main__":
    main()
