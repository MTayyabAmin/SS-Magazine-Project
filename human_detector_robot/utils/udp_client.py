"""UDP client for robot ESP32 motor commands + telemetry.

This module provides a thread-safe UDP link between the laptop and the
ESP32 robot:
  - Outbound: JSON motor commands (v, omega, seq, allow_creep) at a
    rate-limited cadence.
  - Inbound: JSON telemetry on a dedicated port (with a command-socket
    fallback), including auto-discovery of the robot's (possibly changed) IP.
  - A daemon heartbeat thread retransmits the LAST command every 50ms so the
    ESP32's command-timeout failsafe never trips while the laptop is healthy
    (the ESP32 stops motors if no packet arrives within CMD_TIMEOUT_MS).

Algorithm Overview:
  1. send_command() encodes the payload, stores it under a lock, sends once.
  2. _heartbeat_loop() re-sends that payload every 50ms until close(); if no
     telemetry has ever been seen it also broadcasts to 192.168.137.255 to
     discover the robot.
  3. poll_telemetry() tries the telemetry socket first, then the command
     socket; any sender with a different IP updates self.host (DHCP-proof).
"""

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
    """Thread-safe UDP motor-command sender + telemetry receiver.

    Attributes:
        host: Robot ESP32 IP (auto-updated when telemetry arrives from
            a different IP — handles DHCP changes transparently).
        port: Destination UDP port for motor commands (e.g. 4210).
        telemetry_port: Local port bound for telemetry (e.g. 4211).
        _sock: Command socket (broadcast-enabled, 2ms timeout).
        _telem_sock: Telemetry socket (SO_REUSEADDR, 5ms timeout, bound).
        _seq: Monotonic sequence number for outgoing command packets.
        _last_telemetry: Most recently parsed telemetry dict (or None).
        last_telemetry_time: Timestamp of last successful receive (0 = never).
        _lock: Protects _last_payload between caller and heartbeat thread.
        _running: Flag that keeps the heartbeat loop alive.
        _last_payload: Last command bytes — retransmitted by the heartbeat.
        _heartbeat_thread: Daemon thread running _heartbeat_loop().
        _last_send_error_log: Rate-limiter timestamp for send-error logs.
        send_ok: False while sends are failing (True otherwise).
    """

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
        """Create both UDP sockets and start the heartbeat thread.

        Algorithm:
          1. Command socket: AF_INET/SOCK_DGRAM with SO_BROADCAST=1 and a
             2ms timeout (send path must never block the control loop).
          2. Telemetry socket: SO_REUSEADDR, bound to ('', telemetry_port)
             with a 5ms timeout (poll_telemetry() must fail fast when quiet).
          3. Create the lock, then start _heartbeat_loop in a daemon thread.
        """
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
        """Mark sends as failed and log a warning at most once per 2 seconds.

        Args:
            exc: The OSError (or subclass) raised by sendto().

        Algorithm:
          1. send_ok = False.
          2. If ≥2s since the last error log → log and update timestamp.
             Rate-limiting avoids flooding the log on a dead link.
        """
        self.send_ok = False
        now = time.time()
        if now - self._last_send_error_log > 2.0:
            _log.warning("UDP send fail: %s", exc)
            self._last_send_error_log = now

    def _heartbeat_loop(self) -> None:
        """Background thread that retransmits the last command every 50ms.

        Algorithm:
          1. While _running: sleep 50ms.
          2. Under the lock, take _last_payload (or a default
             {"v":0,"omega":0,"seq":0} keepalive).
          3. sendto(host, port); if no telemetry has EVER arrived, also
             broadcast the same payload to 192.168.137.255 so a freshly
             DHCP'd robot hears us.
          4. Success → send_ok = True; OSError → rate-limited log.

        Purpose:
          The ESP32 stops its motors if commands stop arriving
          (CMD_TIMEOUT_MS failsafe), so this keeps the link alive even
          when the control loop hasn't issued a NEW command yet.
        """
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
        """Send a motor velocity command to the robot ESP32.

        Args:
            v: Linear velocity in m/s (positive=forward, negative=reverse).
            omega: Angular velocity in rad/s (positive=CCW/left).
            allow_creep: True to permit low-speed creep (ESP32 skips its
                wall-stop while creeping — used in CREEP/SCAN states).

        Algorithm:
          1. Increment the sequence number (packet ordering/dedup aid).
          2. Build JSON with velocities rounded to 4 decimals.
          3. Encode UTF-8; store as _last_payload under the lock so the
             heartbeat re-sends exactly this command.
          4. sendto() immediately; on OSError rate-limit the log.
        """
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
        """Send a zero-velocity command to halt the robot.

        Algorithm:
          Delegates to send_command(0.0, 0.0) — both v and omega are zeroed.
        """
        self.send_command(0.0, 0.0)

    def poll_telemetry(self) -> dict | None:
        """Checks both dedicated telemetry port (4211) and command socket reply.

        Returns:
            The newly parsed telemetry dict, or the last cached dict if no
            new packet arrived this tick, or None if nothing was ever received.

        Algorithm:
          1. PRIMARY: non-blocking recv on _telem_sock (5ms timeout).
             - Parse JSON, stamp last_telemetry_time.
             - If the source IP differs from self.host, log and UPDATE host
               (auto-discovery of the robot's new DHCP address).
             - Return the dict.
          2. FALLBACK: same logic on the command socket _sock (some firmware
             replies there instead).
          3. Both timed out / failed to parse → return the cached dict.
        """
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
        """Kitni der pehle AAKHRI baar fresh telemetry mila tha.

        Returns:
            Seconds since the last received telemetry packet; float('inf')
            if none was ever received (so staleness checks work from boot).

        Algorithm:
          1. last_telemetry_time == 0.0 means "never" → return infinity.
          2. Otherwise return now - last_telemetry_time.
        """
        if self.last_telemetry_time == 0.0:
            return float("inf")
        return time.time() - self.last_telemetry_time

    def close(self) -> None:
        """Shut the client down: stop heartbeat, halt robot, close sockets.

        Algorithm:
          1. _running = False → heartbeat loop exits on next tick.
          2. Join the thread with a 0.1s timeout (it's a daemon anyway).
          3. send_stop() so the robot doesn't coast on stale commands.
          4. Close both sockets.
        """
        self._running = False
        if self._heartbeat_thread.is_alive():
            self._heartbeat_thread.join(timeout=0.1)
        self.send_stop()
        self._sock.close()
        self._telem_sock.close()


class RateLimiter:
    """Time-based gate for capping how often the control loop sends commands.

    Attributes:
        period: Minimum seconds between two ready() == True results.
        _last: Timestamp of the last time ready() returned True.
    """

    def __init__(self, rate_hz: float) -> None:
        """Initialize the limiter.

        Args:
            rate_hz: Maximum call frequency in Hz (e.g. 20 → one pass per 50ms).
        """
        self.period = 1.0 / rate_hz
        self._last = 0.0

    def ready(self) -> bool:
        """Check whether the rate limit permits another command now.

        Returns:
            True if ≥ period has passed since the last True (and stamps the
            new time); False if the caller must wait.

        Algorithm:
          1. now = time.time().
          2. If now - _last >= period → _last = now; return True.
          3. Else return False (no side effects).
        """
        now = time.time()
        if now - self._last >= self.period:
            self._last = now
            return True
        return False
