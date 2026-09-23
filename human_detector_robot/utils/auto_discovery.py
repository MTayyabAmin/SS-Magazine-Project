#!/usr/bin/env python3
"""
ARK-5 Smart Auto-Discovery Utility
Automatically locates ESP32-CAM streams and ESP32 robots across the local subnet.
Solves dynamic DHCP IP changes permanently.
"""

from __future__ import annotations

import concurrent.futures
import socket
import urllib.request
from typing import List, Optional


def check_stream_url(url: str, timeout: float = 0.6) -> bool:
    """Quickly tests if an HTTP stream URL is responding."""
    try:
        req = urllib.request.Request(url, headers={"User-Agent": "ARK5-Scanner"})
        with urllib.request.urlopen(req, timeout=timeout) as resp:
            return resp.status == 200
    except Exception:
        return False


def test_port(ip: str, port: int, timeout: float = 0.2) -> Optional[str]:
    """Test if a specific IP:port is open."""
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.settimeout(timeout)
    try:
        if s.connect_ex((ip, port)) == 0:
            return ip
    except Exception:
        pass
    finally:
        s.close()
    return None


def discover_cameras(subnets: Optional[List[str]] = None, port: int = 81) -> List[str]:
    """
    Scans candidate subnets for active ESP32-CAM streams on specified port.
    Returns a list of working stream URLs (e.g. ['http://192.168.137.74:81/stream']).
    """
    if not subnets:
        subnets = ["192.168.137", "10.147.103"]

    candidate_ips = []
    for subnet in subnets:
        for host in range(2, 255):
            candidate_ips.append(f"{subnet}.{host}")

    found_ips: List[str] = []
    with concurrent.futures.ThreadPoolExecutor(max_workers=100) as executor:
        results = executor.map(lambda ip: test_port(ip, port), candidate_ips)
        for ip in results:
            if ip:
                found_ips.append(ip)

    # Verify which open ports actually serve /stream
    valid_streams = []
    for ip in found_ips:
        url = f"http://{ip}:{port}/stream"
        if check_stream_url(url, timeout=0.8):
            valid_streams.append(url)

    return valid_streams


def resolve_camera_url(preferred_url: str, robot_id: int = 1) -> str:
    """
    Checks preferred_url. If accessible, returns it immediately.
    If unreachable, triggers fast auto-discovery across the network.
    """
    if check_stream_url(preferred_url, timeout=0.6):
        return preferred_url

    print(f"\n[AUTO-DISCOVERY] Configured stream '{preferred_url}' not responding.")
    print(f"[AUTO-DISCOVERY] Auto-scanning 2.4GHz network for active camera streams...")

    found = discover_cameras(subnets=["192.168.137", "10.147.103"])
    if found:
        # Match by robot_id (Robot 1 gets first, Robot 2 gets second if multiple found)
        idx = 0 if robot_id == 1 else min(1, len(found) - 1)
        selected = found[idx]
        print(f"[AUTO-DISCOVERY] Found active camera stream -> {selected}")
        return selected

    print(f"[AUTO-DISCOVERY] No new streams found. Falling back to: {preferred_url}")
    return preferred_url


if __name__ == "__main__":
    print("[SCAN] Scanning for active ESP32-CAM streams...")
    cams = discover_cameras()
    if cams:
        print(f"Discovered {len(cams)} camera(s):")
        for c in cams:
            print("  ->", c)
    else:
        print("No active camera streams found right now.")
