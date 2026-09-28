"""UDP client for robot ESP32 motor commands + telemetry."""

from __future__ import annotations

import json
import logging
import socket
import threading
import time
from dataclasses import dataclass, field

_log = logging.getLogger("ark5.udp")


@dataclass
class UdpMotorClient:
    host: str
    port: int
    telemetry_port: int = 4211
    _sock: socket.socket = field(init=False, repr=False)
    _telem_sock: socket.socket = field(init=False, repr=False)
    _seq: int = field(default=0, init=False)
    _last_telemetry: dict | None = field(default=None, init=False)
    last_telemetry_time: float = field(default=0.0, init=False)
    _lock: threading.Lock = field(init=False, repr=False)
    _running: bool = field(default=True, init=False)
    _last_payload: bytes | None = field(default=None, init=False)
    _heartbeat_thread: threading.Thread = field(init=False, repr=False)
    _last_send_error_log: float = field(default=0.0, init=False)
    send_ok: bool = field(default=True, init=False)

    def __post_init__(self) -> None:
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
        self._sock.settimeout(0.002)
        self._telem_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._telem_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._telem_sock.bind(("", self.telemetry_port))
        self._telem_sock.settimeout(0.005)
        self._lock = threading.Lock()
        self._heartbeat_thread = threading.Thread(target=self._heartbeat_loop, daemon=True)
        self._heartbeat_thread.start()

    def _log_send_failure(self, exc: Exception) -> None:
        self.send_ok = False
        now = time.time()
        if now - self._last_send_error_log > 2.0:
            _log.warning("UDP send fail: %s", exc)
            self._last_send_error_log = now

    def _heartbeat_loop(self) -> None:
        while self._running:
            time.sleep(0.05)
            with self._lock:
                payload = self._last_payload or b'{"v":0,"omega":0,"seq":0}'
                try:
                    self._sock.sendto(payload, (self.host, self.port))
                    # Broadcast ping across 192.168.137.255 if no telemetry yet
                    if self.last_telemetry_time == 0.0:
                        self._sock.sendto(payload, ("192.168.137.255", self.port))
                    self.send_ok = True
                except OSError as exc:
                    self._log_send_failure(exc)

    def send_command(self, v: float, omega: float, allow_creep: bool = False) -> None:
        self._seq += 1
        payload = {
            "v": round(v, 4),
            "omega": round(omega, 4),
            "seq": self._seq,
            "allow_creep": allow_creep,
        }
        data = json.dumps(payload).encode("utf-8")
        with self._lock:
            self._last_payload = data
        try:
            self._sock.sendto(data, (self.host, self.port))
            self.send_ok = True
        except OSError as exc:
            self._log_send_failure(exc)

    def send_stop(self) -> None:
        self.send_command(0.0, 0.0)

    def poll_telemetry(self) -> dict | None:
        """Checks both dedicated telemetry port (4211) and command socket reply."""
        # 1. Primary: dedicated telemetry port (4211/4221)
        try:
            data, addr = self._telem_sock.recvfrom(1024)
            self._last_telemetry = json.loads(data.decode("utf-8"))
            self.last_telemetry_time = time.time()
            if addr and addr[0] != self.host:
                _log.info("Auto-discovered Robot ESP32 at IP: %s (updated from %s)", addr[0], self.host)
                self.host = addr[0]
            return self._last_telemetry
        except (socket.timeout, BlockingIOError, json.JSONDecodeError):
            pass

        # 2. Fallback: reply on command socket
        try:
            data, addr = self._sock.recvfrom(1024)
            self._last_telemetry = json.loads(data.decode("utf-8"))
            self.last_telemetry_time = time.time()
            if addr and addr[0] != self.host:
                _log.info("Auto-discovered Robot ESP32 at IP: %s (updated from %s)", addr[0], self.host)
                self.host = addr[0]
            return self._last_telemetry
        except (socket.timeout, BlockingIOError, json.JSONDecodeError, OSError):
            pass

        return self._last_telemetry

    def seconds_since_telemetry(self) -> float:
        """Kitni der pehle AAKHRI baar fresh telemetry mila tha."""
        if self.last_telemetry_time == 0.0:
            return float("inf")
        return time.time() - self.last_telemetry_time

    def close(self) -> None:
        self._running = False
        if self._heartbeat_thread.is_alive():
            self._heartbeat_thread.join(timeout=0.1)
        self.send_stop()
        self._sock.close()
        self._telem_sock.close()


class RateLimiter:
    def __init__(self, rate_hz: float) -> None:
        self.period = 1.0 / rate_hz
        self._last = 0.0

    def ready(self) -> bool:
        now = time.time()
        if now - self._last >= self.period:
            self._last = now
            return True
        return False
