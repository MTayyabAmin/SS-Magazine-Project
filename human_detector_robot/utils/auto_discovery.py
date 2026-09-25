#!/usr/bin/env python3
"""
ARK-5 Smart Auto-Discovery Utility.

Automatically locates ESP32-CAM video streams and ESP32 robot controllers
across the local subnet using parallel port scanning and HTTP stream verification.

Problem Solved:
  ESP32 devices get dynamic DHCP IPs that change on reboot. This module
  eliminates the need to manually reconfigure IPs by scanning the subnet
  and verifying active camera streams in real time.

Algorithm Overview:
  1. Generate candidate IPs across configured subnets (e.g. 192.168.137.2-254).
  2. Port-scan all candidates in parallel using a thread pool (100 workers).
  3. For each open port, verify it actually serves an MJPEG /stream endpoint.
  4. Return validated stream URLs for use by the vision pipeline.
"""

from __future__ import annotations

import concurrent.futures
import socket
import urllib.request
from typing import List, Optional


def check_stream_url(url: str, timeout: float = 0.6) -> bool:
    """Quickly test if an HTTP stream URL is responding with HTTP 200.

    Args:
        url: Full URL to test (e.g. 'http://192.168.137.74:81/stream').
        timeout: Maximum seconds to wait for a response.

    Returns:
        True if the server responds with HTTP 200 OK, False otherwise.

    Algorithm:
      1. Send an HTTP GET request with a custom User-Agent header.
      2. If the response status is 200, return True.
      3. Any exception (timeout, connection refused, etc.) returns False.
    """
    try:
        req = urllib.request.Request(url, headers={"User-Agent": "ARK5-Scanner"})
        with urllib.request.urlopen(req, timeout=timeout) as resp:
            return resp.status == 200
    except Exception:
        return False


def test_port(ip: str, port: int, timeout: float = 0.2) -> Optional[str]:
    """Test if a specific IP:port combination is open (TCP connect check).

    Args:
        ip: IP address string to test (e.g. '192.168.137.50').
        port: TCP port number to check (e.g. 81 for ESP32-CAM).
        timeout: Maximum seconds to wait for the TCP connection attempt.

    Returns:
        The IP string if the port is open, None if closed/unreachable.

    Algorithm:
      1. Create a TCP socket with the specified timeout.
      2. Attempt connect_ex() which returns 0 on success.
      3. Return the IP if connection succeeded, None otherwise.
      4. Always close the socket in a finally block to prevent leaks.
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
    """Scan candidate subnets for active ESP32-CAM streams on the specified port.

    Args:
        subnets: List of subnet prefixes to scan (e.g. ['192.168.137']).
                 Defaults to ['192.168.137', '10.147.103'].
        port: TCP port to scan (default 81, standard ESP32-CAM stream port).

    Returns:
        List of validated stream URL strings (e.g. ['http://192.168.137.74:81/stream']).

    Algorithm:
      1. Generate all candidate IPs: for each subnet, enumerate .2 through .254.
      2. Port-scan all candidates in parallel using ThreadPoolExecutor (100 workers).
      3. Collect IPs that responded with an open port.
      4. For each open-Port IP, verify it serves /stream via HTTP GET.
      5. Return only URLs that returned HTTP 200 from the /stream endpoint.
    """
    if not subnets:
        subnets = ["192.168.137", "10.147.103"]

    # Build list of all candidate IPs across all subnets
    candidate_ips = []
    for subnet in subnets:
        for host in range(2, 255):
            candidate_ips.append(f"{subnet}.{host}")

    # Parallel port scan with 100 concurrent threads
    found_ips: List[str] = []
    with concurrent.futures.ThreadPoolExecutor(max_workers=100) as executor:
        results = executor.map(lambda ip: test_port(ip, port), candidate_ips)
        for ip in results:
            if ip:
                found_ips.append(ip)

    # Verify which open ports actually serve an MJPEG stream
    valid_streams = []
    for ip in found_ips:
        url = f"http://{ip}:{port}/stream"
        if check_stream_url(url, timeout=0.8):
            valid_streams.append(url)

    return valid_streams


def resolve_camera_url(preferred_url: str, robot_id: int = 1) -> str:
    """Resolve the camera URL: use preferred if reachable, otherwise auto-discover.

    Args:
        preferred_url: The configured camera URL from project.yaml.
        robot_id: Robot identifier (1 or 2) used to select among multiple found cameras.

    Returns:
        A valid, reachable camera stream URL string.

    Algorithm:
      1. Test if preferred_url is reachable via HTTP.
      2. If reachable, return it immediately (fast path).
      3. If unreachable, print a notice and trigger full subnet discovery.
      4. If cameras are found, select based on robot_id:
         - Robot 1 gets the first found camera.
         - Robot 2 gets the second (or last if only one found).
      5. If no cameras found, fall back to the preferred_url (caller handles failure).
    """
    if check_stream_url(preferred_url, timeout=0.6):
        return preferred_url

    print(f"\n[AUTO-DISCOVERY] Configured stream '{preferred_url}' not responding.")
    print(f"[AUTO-DISCOVERY] Auto-scanning 2.4GHz network for active camera streams...")

    found = discover_cameras(subnets=["192.168.137", "10.147.103"])
    if found:
        # Select camera based on robot_id for multi-robot setups
        idx = 0 if robot_id == 1 else min(1, len(found) - 1)
        selected = found[idx]
        print(f"[AUTO-DISCOVERY] Found active camera stream -> {selected}")
        return selected

    print(f"[AUTO-DISCOVERY] No new streams found. Falling back to: {preferred_url}")
    return preferred_url


if __name__ == "__main__":
    # Standalone execution: scan and print all discovered camera streams
    print("[SCAN] Scanning for active ESP32-CAM streams...")
    cams = discover_cameras()
    if cams:
        print(f"Discovered {len(cams)} camera(s):")
        for c in cams:
            print("  ->", c)
    else:
        print("No active camera streams found right now.")
