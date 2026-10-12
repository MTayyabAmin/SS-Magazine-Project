#!/usr/bin/env python3
"""
ESP32-CAM MJPEG stream debug — FPS, latency, frame stats.

Usage:
  python debug/debug_cam_stream.py
  python debug/debug_cam_stream.py --url http://192.168.1.105:81/stream
  python debug/debug_cam_stream.py --save-snapshot test_frame.jpg
"""

from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

import cv2

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
from utils.vision import load_config


def main() -> None:
    parser = argparse.ArgumentParser(description="ESP32-CAM stream speed debug")
    parser.add_argument("--url", default=None, help="MJPEG stream URL")
    parser.add_argument("--duration", type=float, default=30, help="Test duration seconds")
    parser.add_argument("--preview", action="store_true", help="Show live preview window")
    parser.add_argument("--save-snapshot", default=None, help="Save one frame to this path")
    args = parser.parse_args()

    cfg = load_config()
    url = args.url or cfg["camera"]["stream_url"]

    print("=" * 60)
    print("  ESP32-CAM STREAM DEBUG")
    print("=" * 60)
    print(f"  URL: {url}")
    print(f"  Duration: {args.duration}s\n")

    print("[INIT] Opening stream...")
    t0 = time.time()
    cap = cv2.VideoCapture(url)
    if not cap.isOpened():
        print("[FAIL] Stream open nahi hua!")
        print("  Check: ESP32-CAM powered? Same WiFi/router? URL sahi?")
        print("  Browser mein try karo:", url)
        sys.exit(1)

    open_ms = (time.time() - t0) * 1000
    print(f"[OK] Stream opened in {open_ms:.0f} ms\n")

    frame_times: list[float] = []
    read_times: list[float] = []
    widths: list[int] = []
    heights: list[int] = []
    failures = 0
    start = time.time()
    last_report = start
    frame_count = 0

    print(f"{'#':>6} | {'Read ms':>8} | {'FPS':>6} | {'Size':>12} | {'Status'}")
    print("-" * 55)

    try:
        while (time.time() - start) < args.duration:
            t_read = time.time()
            ok, frame = cap.read()
            read_ms = (time.time() - t_read) * 1000

            if not ok or frame is None:
                failures += 1
                print(f"{frame_count:6} | {'---':>8} | {'---':>6} | {'---':>12} | FAIL")
                time.sleep(0.05)
                continue

            frame_count += 1
            now = time.time()
            frame_times.append(now)
            read_times.append(read_ms)
            h, w = frame.shape[:2]
            widths.append(w)
            heights.append(h)

            if args.save_snapshot and frame_count == 1:
                cv2.imwrite(args.save_snapshot, frame)
                print(f"  [SNAP] Saved → {args.save_snapshot}")

            # Report every ~2 sec
            if now - last_report >= 2.0:
                window = [t for t in frame_times if now - t <= 2.0]
                fps = (len(window) - 1) / (window[-1] - window[0]) if len(window) > 1 else 0
                avg_read = sum(read_times[-30:]) / min(len(read_times), 30)
                status = "OK" if fps >= 5 else "SLOW"
                print(f"{frame_count:6} | {avg_read:8.1f} | {fps:6.1f} | {w}x{h:>5} | {status}")
                last_report = now

            if args.preview:
                cv2.imshow("CAM Debug", frame)
                if cv2.waitKey(1) & 0xFF == ord("q"):
                    break

    except KeyboardInterrupt:
        print("\n[STOP] User interrupt.")
    finally:
        cap.release()
        cv2.destroyAllWindows()
        elapsed = time.time() - start

        print("\n" + "=" * 60)
        print("  STREAM SUMMARY")
        print("=" * 60)
        print(f"  Total frames     : {frame_count}")
        print(f"  Failed reads     : {failures}")
        print(f"  Duration         : {elapsed:.1f}s")
        if frame_count > 1 and len(frame_times) > 1:
            overall_fps = (frame_count - 1) / (frame_times[-1] - frame_times[0])
            print(f"  Average FPS      : {overall_fps:.1f}")
        if read_times:
            print(f"  Read time avg    : {sum(read_times)/len(read_times):.1f} ms")
            print(f"  Read time max    : {max(read_times):.1f} ms")
        if widths:
            print(f"  Resolution       : {widths[-1]}x{heights[-1]}")
        print("\n  Good stream: FPS >= 10, failures near 0")
        print("  Slow stream: WiFi weak, 4700uF cap check, router door rakho")


if __name__ == "__main__":
    main()
