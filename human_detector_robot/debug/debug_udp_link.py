#!/usr/bin/env python3
"""
UDP link debug — laptop ↔ robot ESP32 latency & packet stats.

Usage:
  python debug/debug_udp_link.py
  python debug/debug_udp_link.py --packets 100
"""

from __future__ import annotations

import argparse
import json
import socket
import statistics
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
from utils.vision import load_config


def main() -> None:
    parser = argparse.ArgumentParser(description="UDP latency debug")
    parser.add_argument("--host", default=None)
    parser.add_argument("--packets", type=int, default=100)
    parser.add_argument("--hz", type=float, default=20)
    args = parser.parse_args()

    cfg = load_config()
    host = args.host or cfg["robot"]["udp_host"]
    cmd_port = cfg["robot"]["udp_port"]
    telem_port = cfg["robot"]["telemetry_port"]
    interval = 1.0 / args.hz

    sock_send = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock_recv = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock_recv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock_recv.bind(("", telem_port))
    sock_recv.settimeout(0.15)

    print("=" * 60)
    print("  UDP LINK DEBUG — Laptop ↔ Robot ESP32")
    print("=" * 60)
    print(f"  Send → {host}:{cmd_port}")
    print(f"  Recv ← port {telem_port}")
    print(f"  Packets: {args.packets} @ {args.hz} Hz\n")

    rtts: list[float] = []
    gaps: list[float] = []
    telem_hz_samples: list[float] = []
    timeouts = 0
    last_send = 0.0
    last_telem_time = None
    seq = 0

    print(f"{'SEQ':>5} | {'RTT ms':>8} | {'Gap ms':>8} | {'Telem Hz':>9} | {'yaw':>7} | {'dist':>5}")
    print("-" * 60)

    for i in range(args.packets):
        now = time.time()
        if last_send > 0:
            gaps.append((now - last_send) * 1000)

        t_send = time.time()
        seq += 1
        payload = json.dumps({"v": 0.0, "omega": 0.0, "seq": seq, "t_send": t_send})
        sock_send.sendto(payload.encode(), (host, cmd_port))
        last_send = t_send

        try:
            data, _ = sock_recv.recvfrom(1024)
            t_recv = time.time()
            telem = json.loads(data.decode())
            rtt = (t_recv - t_send) * 1000
            rtts.append(rtt)

            if last_telem_time:
                dt = t_recv - last_telem_time
                if dt > 0:
                    telem_hz_samples.append(1.0 / dt)
            last_telem_time = t_recv

            hz_str = f"{telem_hz_samples[-1]:.1f}" if telem_hz_samples else "---"
            gap_str = f"{gaps[-1]:.1f}" if gaps else "---"
            yaw = telem.get("yaw", 0)
            dist = telem.get("dist_cm", -1)

            if i % 5 == 0 or i < 3:
                print(f"{seq:5} | {rtt:8.1f} | {gap_str:>8} | {hz_str:>9} | {yaw:7.1f} | {dist:5}")

        except socket.timeout:
            timeouts += 1
            print(f"{seq:5} | TIMEOUT")

        elapsed = time.time() - now
        if elapsed < interval:
            time.sleep(interval - elapsed)

    sock_send.close()
    sock_recv.close()

    print("\n" + "=" * 60)
    print("  UDP SUMMARY")
    print("=" * 60)
    if rtts:
        print(f"  RTT avg/min/max : {statistics.mean(rtts):.1f} / {min(rtts):.1f} / {max(rtts):.1f} ms")
        if len(rtts) > 1:
            print(f"  RTT std dev     : {statistics.stdev(rtts):.1f} ms")
    if gaps:
        print(f"  Send gap avg    : {statistics.mean(gaps):.1f} ms  (target {1000/args.hz:.0f} ms)")
    if telem_hz_samples:
        print(f"  Telemetry Hz    : {statistics.mean(telem_hz_samples):.1f} avg")
    print(f"  Timeouts        : {timeouts} / {args.packets}")
    print(f"  Success rate    : {100*(args.packets-timeouts)/args.packets:.0f}%")
    print("\n  Good link: RTT < 30ms, timeouts = 0, telem ~10 Hz")


if __name__ == "__main__":
    main()
