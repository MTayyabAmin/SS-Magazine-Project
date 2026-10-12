#!/usr/bin/env python3
"""
ARK-5 Smart Auto-Discovery Utility
Automatically locates ESP32-CAM streams and ESP32 robots across the local subnet.
Solves dynamic DHCP IP changes permanently.

Algorithm Overview:
  1. Generate candidate IPs across configured subnets (e.g. 192.168.137.2-254).
  2. Port-scan all candidates in parallel (thread pool, 100 workers).
  3. For each open port, verify it actually serves the MJPEG /stream endpoint
     with an HTTP GET.
  4. Return only validated stream URLs for the vision pipeline.
"""

from __future__ import annotations

import concurrent.futures
import socket
import urllib.request
from typing import List, Optional


def check_stream_url(url: str, timeout: float = 0.6) -> bool:
    """Quickly tests if an HTTP stream URL is responding.

    Args:
        url: Full URL to test (e.g. 'http://192.168.137.74:81/stream').
        timeout: Maximum seconds to wait for the HTTP response.

    Returns:
        True if the server replied HTTP 200 OK, False on any error
        (timeout, connection refused, non-200 status).

    Algorithm:
      1. Send HTTP GET with a custom 'ARK5-Scanner' User-Agent.
      2. Return resp.status == 200 if a response arrives.
      3. Catch-all except → False.
    """
    try:
        req = urllib.request.Request(url, headers={"User-Agent": "ARK5-Scanner"})
        with urllib.request.urlopen(req, timeout=timeout) as resp:
            return resp.status == 200
    except Exception:
        return False


def test_port(ip: str, port: int, timeout: float = 0.2) -> Optional[str]:
    """Test if a specific IP:port combination accepts a TCP connection.

    Args:
        ip: IP address string to probe (e.g. '192.168.137.50').
        port: TCP port number (e.g. 81 for ESP32-CAM).
        timeout: Seconds allowed for the connect attempt.

    Returns:
        The IP string if the port is open; None if closed/unreachable.

    Algorithm:
      1. Create a TCP socket with the given timeout.
      2. connect_ex() returns 0 on success (open port).
      3. Return the IP on success; otherwise None.
      4. Always close the socket in finally (no fd leaks).
    """
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

    Args:
        subnets: Subnet prefixes to scan (default ['192.168.137', '10.147.103']).
        port: TCP port to scan (default 81 — ESP32-CAM stream port).

    Returns:
        List of validated '/stream' URL strings that returned HTTP 200.

    Algorithm:
      1. Build the candidate IP list: for each subnet, enumerate .2 → .254.
      2. Parallel port scan with ThreadPoolExecutor(max_workers=100).
      3. Collect IPs whose port answered.
      4. Verify each open IP with check_stream_url on http://ip:port/stream.
      5. Return only the URLs that pass verification.
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

    Args:
        preferred_url: Configured camera URL from project.yaml.
        robot_id: Robot identifier (1 or 2) used to pick among multiple
                  discovered cameras (Robot 1 → first, Robot 2 → second).

    Returns:
        A reachable camera stream URL string.

    Algorithm:
      1. check_stream_url(preferred_url) → if OK, return it (fast path).
      2. Otherwise print a notice and scan the known subnets.
      3. If cameras found: index = 0 for robot 1, min(1, len-1) for robot 2.
      4. If none found: fall back to preferred_url (caller handles failure).
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
    # Standalone mode: full subnet scan, print every discovered stream.
    print("[SCAN] Scanning for active ESP32-CAM streams...")
    cams = discover_cameras()
    if cams:
        print(f"Discovered {len(cams)} camera(s):")
        for c in cams:
            print("  ->", c)
    else:
        print("No active camera streams found right now.")
