#!/usr/bin/env python3
"""
Full system debug — robot sensors + cam stream ek saath.

Usage:
  python debug/debug_all.py
"""

from __future__ import annotations

import argparse
import sys
import threading
import time
from pathlib import Path

import cv2

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
from utils.vision import load_config
from utils.udp_client import UdpMotorClient


def main() -> None:
    parser = argparse.ArgumentParser(description="Full system debug dashboard")
    parser.add_argument("--duration", type=float, default=0, help="Seconds (0=forever)")
    args = parser.parse_args()

    cfg = load_config()
    host = cfg["robot"]["udp_host"]
    url = cfg["camera"]["stream_url"]

    print("=" * 60)
    print("  FULL SYSTEM DEBUG")
    print("=" * 60)
    print(f"  Robot : {host}")
    print(f"  Camera: {url}\n")

    # --- Camera thread ---
    cam_state = {"fps": 0.0, "frames": 0, "failures": 0, "resolution": "---", "ok": False}
    stop = threading.Event()

    def cam_loop():
        cap = cv2.VideoCapture(url)
        if not cap.isOpened():
            cam_state["ok"] = False
            return
        cam_state["ok"] = True
        times: list[float] = []
        while not stop.is_set():
            t0 = time.time()
            ok, frame = cap.read()
            if ok and frame is not None:
                cam_state["frames"] += 1
                h, w = frame.shape[:2]
                cam_state["resolution"] = f"{w}x{h}"
                now = time.time()
                times.append(now)
                times = [t for t in times if now - t <= 2.0]
                if len(times) > 1:
                    cam_state["fps"] = (len(times) - 1) / (times[-1] - times[0])
            else:
                cam_state["failures"] += 1
            dt = time.time() - t0
            if dt < 0.02:
                time.sleep(0.02 - dt)
        cap.release()

    cam_thread = threading.Thread(target=cam_loop, daemon=True)
    cam_thread.start()
    time.sleep(1.5)

    client = UdpMotorClient(
        host=host,
        port=cfg["robot"]["udp_port"],
        telemetry_port=cfg["robot"]["telemetry_port"],
    )

    start = time.time()
    telem_count = 0

    print(f"{'TIME':>6} | {'CAM FPS':>8} | {'YAW°':>7} | {'DIST':>6} | {'RSSI':>5} | {'WALL':>5} | {'LINK'}")
    print("-" * 65)

    try:
        while not stop.is_set():
            client.send_stop()
            telem = client.poll_telemetry()
            elapsed = time.time() - start

            cam_fps = cam_state["fps"]
            cam_status = "OK" if cam_state["ok"] else "FAIL"

            if telem:
                telem_count += 1
                yaw = telem.get("yaw", 0)
                dist = telem.get("dist_cm", -1)
                rssi = telem.get("sense_rssi", 0)
                wall = "YES" if telem.get("wall_near") else "no"
                link = "OK" if client.send_ok else "ERR"
                print(
                    f"{elapsed:6.1f} | {cam_fps:8.1f} | {yaw:7.1f} | {dist:6} | {rssi:5} | {wall:>5} | {link}"
                )
            else:
                print(f"{elapsed:6.1f} | {cam_fps:8.1f} | {'---':>7} | {'---':>6} | {'---':>5} | {'---':>5} | {cam_status}")

            if args.duration > 0 and elapsed >= args.duration:
                break
            time.sleep(0.5)

    except KeyboardInterrupt:
        pass
    finally:
        stop.set()
        cam_thread.join(timeout=1)
        client.close()
        elapsed = time.time() - start
        print("\n" + "=" * 60)
        print(f"  Camera frames : {cam_state['frames']}  failures: {cam_state['failures']}")
        print(f"  Camera FPS    : {cam_state['fps']:.1f}  resolution: {cam_state['resolution']}")
        print(f"  Telemetry pkts: {telem_count}  ({telem_count/elapsed:.1f} Hz)" if elapsed else "")


if __name__ == "__main__":
    main()
