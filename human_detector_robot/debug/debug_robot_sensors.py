#!/usr/bin/env python3
"""
Robot ESP32 sensor debug — live UDP telemetry.

Dikhata hai: MPU yaw, ultrasonic distance, WiFi RSSI, wall_near flag.

Usage:
  python debug/debug_robot_sensors.py
  python debug/debug_robot_sensors.py --host 192.168.1.106
"""

from __future__ import annotations

import argparse
import json
import socket
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
from utils.vision import load_config
from utils.udp_client import UdpMotorClient


def main() -> None:
    parser = argparse.ArgumentParser(description="Robot ESP32 sensor telemetry debug")
    parser.add_argument("--host", default=None, help="Robot ESP32 IP (overrides project.yaml)")
    parser.add_argument("--duration", type=float, default=0, help="Seconds to run (0 = forever)")
    args = parser.parse_args()

    cfg = load_config()
    host = args.host or cfg["robot"]["udp_host"]
    port = cfg["robot"]["udp_port"]
    telem_port = cfg["robot"]["telemetry_port"]

    print("=" * 60)
    print("  ROBOT SENSOR DEBUG — MPU / Ultrasonic / WiFi RSSI")
    print("=" * 60)
    print(f"  Target: {host}:{port}  |  Telemetry port: {telem_port}")
    print("  Sending stop commands + reading telemetry. Ctrl+C to quit.\n")

    client = UdpMotorClient(host=host, port=port, telemetry_port=telem_port)
    start = time.time()
    last_seq = None
    lost = 0
    count = 0
    yaw_samples: list[float] = []

    print(f"{'TIME':>8} | {'YAW°':>7} | {'FRONT':>6} | {'LEFT':>5} | {'RIGHT':>5} | {'RSSI':>5} | {'WALL':>5}")
    print("-" * 70)

    try:
        while True:
            client.send_stop()
            telem = client.poll_telemetry()

            if telem:
                count += 1
                yaw = float(telem.get("yaw", 0))
                dist = int(telem.get("dist_front_cm", telem.get("dist_cm", -1)))
                dl = int(telem.get("dist_left_cm", -1))
                dr = int(telem.get("dist_right_cm", -1))
                rssi = int(telem.get("sense_rssi", 0))
                wall = bool(telem.get("wall_near", False))
                seq = int(telem.get("seq", 0))
                yaw_samples.append(yaw)
                elapsed = time.time() - start
                hz = count / elapsed if elapsed > 0 else 0
                wall_str = "YES" if wall else "no"

                print(
                    f"{elapsed:8.1f} | {yaw:7.1f} | {dist:6} | {dl:5} | {dr:5} | {rssi:5} | {wall_str:>5}"
                )

                if last_seq is not None and seq > last_seq + 1:
                    lost += seq - last_seq - 1
                last_seq = seq

                # MPU sanity hints
                if count == 30:
                    yaw_range = max(yaw_samples) - min(yaw_samples)
                    print(f"\n  [MPU CHECK] 30 samples — yaw range: {yaw_range:.1f}°")
                    if yaw_range < 0.5:
                        print("  → Board flat hai, yaw stable — MPU OK lag raha hai")
                        print("  → Ab board ghumao — yaw change hona chahiye\n")
                    print(f"{'TIME':>8} | {'YAW°':>7} | {'FRONT':>6} | {'LEFT':>5} | {'RIGHT':>5} | {'RSSI':>5} | {'WALL':>5}")
                    print("-" * 70)

            time.sleep(0.05)

            if args.duration > 0 and (time.time() - start) >= args.duration:
                break

    except KeyboardInterrupt:
        print("\n[STOP] User interrupt.")
    finally:
        elapsed = time.time() - start
        print("\n" + "=" * 60)
        print("  SUMMARY")
        print("=" * 60)
        print(f"  Packets received : {count}")
        print(f"  Duration         : {elapsed:.1f}s")
        print(f"  Telemetry rate   : {count/elapsed:.1f} Hz" if elapsed > 0 else "  N/A")
        print(f"  Seq gaps (lost)  : {lost}")
        if yaw_samples:
            print(f"  Yaw min/max      : {min(yaw_samples):.1f}° / {max(yaw_samples):.1f}°")
        print("\n  Expected: FRONT 20-400 when facing wall; LEFT when side clear")
        print("  Expected: RIGHT = -1 if only 2 sensors (FRONT+LEFT)")
        print("  Expected: yaw changes when you rotate the board")
        print("  Expected: wall_near YES when dist <= 100 cm")
        client.close()


if __name__ == "__main__":
    main()
