#!/usr/bin/env python3
"""Print current tentative map in terminal."""

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
from utils.vision import project_root

def main():
    path = project_root() / "maps" / "tentative_map.txt"
    if not path.is_file():
        print(f"No map yet: {path}")
        print("Run main_controller.py first.")
        return
    print(path.read_text(encoding="utf-8"))

if __name__ == "__main__":
    main()
