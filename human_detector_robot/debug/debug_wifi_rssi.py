#!/usr/bin/env python3
"""
WiFi human-sensing RSSI debug — baseline vs live comparison.

Usage:
  python debug/debug_wifi_rssi.py --calibrate     # empty room baseline
  python debug/debug_wifi_rssi.py                 # live monitor
"""

from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
from utils.vision import load_config
from utils.udp_client import UdpMotorClient
from utils.wifi_sensing import WifiHumanSensor


def main() -> None:
    parser = argparse.ArgumentParser(description="WiFi RSSI human sensing debug")
    parser.add_argument("--host", default=None)
    parser.add_argument("--calibrate", action="store_true", help="Calibrate empty-room baseline")
    parser.add_argument("--threshold", type=float, default=None, help="RSSI drop threshold dBm")
    args = parser.parse_args()

    cfg = load_config()
    host = args.host or cfg["robot"]["udp_host"]
    threshold = args.threshold or cfg.get("wifi_sensing", {}).get("rssi_drop_threshold_dbm", 8.0)

    sensor = WifiHumanSensor(rssi_drop_threshold_dbm=threshold)
    client = UdpMotorClient(
        host=host,
        port=cfg["robot"]["udp_port"],
        telemetry_port=cfg["robot"]["telemetry_port"],
    )

    print("=" * 60)
    print("  WiFi RSSI SENSING DEBUG")
    print("=" * 60)
    print(f"  Robot: {host}  |  Threshold: {threshold} dBm drop")
    if args.calibrate:
        print("  MODE: Calibrate — room khali rakho, human wall ke peeche na ho\n")
    else:
        print("  MODE: Live — human ko wall ke peeche rakho test ke liye\n")

    print(f"{'TIME':>8} | {'RSSI':>6} | {'BASE':>6} | {'DROP':>6} | {'ALERT'}")
    print("-" * 50)

    start = time.time()
    try:
        while True:
            client.send_stop()
            telem = client.poll_telemetry()
            if telem and "sense_rssi" in telem:
                rssi = float(telem["sense_rssi"])
                elapsed = time.time() - start

                if args.calibrate or sensor.baseline_rssi is None:
                    sensor.calibrate(rssi)
                    base_str = f"{sensor.baseline_rssi:.0f}" if sensor.baseline_rssi else "---"
                    print(f"{elapsed:8.1f} | {rssi:6.0f} | {base_str:>6} | {'---':>6} | calibrating...")
                else:
                    drop = sensor.baseline_rssi - rssi
                    alert = sensor.update(rssi) or ""
                    alert_str = alert if alert else "ok"
                    print(
                        f"{elapsed:8.1f} | {rssi:6.0f} | {sensor.baseline_rssi:6.0f} | {drop:6.1f} | {alert_str}"
                    )

            time.sleep(0.2)
    except KeyboardInterrupt:
        print("\n[STOP] Done.")
    finally:
        client.close()
        if sensor.baseline_rssi:
            print(f"\n  Baseline RSSI: {sensor.baseline_rssi:.1f} dBm")
            print(f"  Human detect if drop >= {threshold} dBm")


if __name__ == "__main__":
    main()
