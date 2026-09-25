"""UDP client for robot ESP32 motor commands and telemetry reception.

This module provides a thread-safe UDP communication layer between the
laptop controller and the ESP32 robot. It handles:
  - Sending motor velocity commands (v, omega) as JSON packets.
  - Receiving telemetry data from the robot on a dedicated port.
  - A background heartbeat thread that continuously sends the last command
    to prevent the robot from stopping if packets are lost.
  - Auto-discovery of the robot's IP from telemetry responses.

Algorithm Overview:
  1. Motor commands are JSON-encoded with linear velocity (v), angular
     velocity (omega), sequence number, and creep permission flag.
  2. A heartbeat thread sends the last command every 50ms to maintain
     the connection even when no new commands are issued.
  3. Telemetry is received on a separate UDP socket (port 4211) and
     also checked on the command socket as a fallback.
  4. If telemetry arrives from a different IP, the client auto-updates
     the robot host address (handles DHCP IP changes).
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
    """Thread-safe UDP client for sending motor commands and receiving telemetry.

    Attributes:
        host: IP address of the robot ESP32.
        port: UDP port for sending motor commands (e.g. 4210).
        telemetry_port: UDP port for receiving telemetry (e.g. 4211).
        _sock: UDP socket for sending commands.
        _telem_sock: UDP socket bound to telemetry_port for receiving telemetry.
        _seq: Monotonically increasing sequence number for command packets.
        _last_telemetry: Most recently received telemetry dictionary.
        last_telemetry_time: Timestamp of the last successful telemetry receive.
        _lock: Thread lock for safe access to shared _last_payload.
        _running: Flag to signal the heartbeat thread to stop.
        _last_payload: Last command payload bytes (repeated by heartbeat).
        _heartbeat_thread: Daemon thread sending periodic keepalive packets.
        _last_send_error_log: Timestamp of last logged send error (rate-limited).
        send_ok: Boolean indicating last send operation succeeded.
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
        """Initialize UDP sockets and start the heartbeat thread.

        Algorithm:
          1. Create a UDP socket for sending commands with broadcast enabled.
          2. Set a very short timeout (2ms) to prevent blocking on send.
          3. Create a separate UDP socket for telemetry, bind to telemetry_port.
          4. Set SO_REUSEADDR to allow quick rebinding after restart.
          5. Start the daemon heartbeat thread.
        """
        # Command sending socket with broadcast support
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
        self._sock.settimeout(0.002)

        # Telemetry receiving socket — bound to a dedicated port
        self._telem_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._telem_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._telem_sock.bind(("", self.telemetry_port))
        self._telem_sock.settimeout(0.005)

        self._lock = threading.Lock()
        self._heartbeat_thread = threading.Thread(target=self._heartbeat_loop, daemon=True)
        self._heartbeat_thread.start()

    def _log_send_failure(self, exc: Exception) -> None:
        """Log a send failure, rate-limited to once every 2 seconds.

        Args:
            exc: The exception that occurred during send.

        Algorithm:
          1. Set send_ok flag to False.
          2. Check if 2+ seconds have passed since last error log.
          3. If so, log the warning and update the timestamp.
        """
        self.send_ok = False
        now = time.time()
        if now - self._last_send_error_log > 2.0:
            _log.warning("UDP send fail: %s", exc)
            self._last_send_error_log = now

    def _heartbeat_loop(self) -> None:
        """Background thread that continuously retransmits the last command.

        Algorithm:
          1. Loop every 50ms while _running is True.
          2. Under thread lock, get the last payload (or a default zero-velocity command).
          3. Send the payload to the robot's host:port.
          4. If no telemetry has been received yet, also broadcast to 192.168.137.255
             to discover the robot's current IP.
          5. On send success, set send_ok=True.
          6. On OSError, rate-limit the error logging via _log_send_failure.

        Purpose:
          Ensures the robot doesn't stop due to missed packets. The ESP32
          stops motors if no command is received within a timeout window,
          so this heartbeat keeps the connection alive.
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
            omega: Angular velocity in rad/s (positive=left/CCW, negative=right/CW).
            allow_creep: If True, permits the robot to use low-speed creep mode.

        Algorithm:
          1. Increment the sequence number.
          2. Build a JSON payload with v, omega, seq, and allow_creep.
          3. Round velocities to 4 decimal places for compact packets.
          4. Encode as UTF-8 bytes.
          5. Store the payload under thread lock (for heartbeat retransmission).
          6. Send the packet immediately via UDP.
          7. On success, set send_ok=True; on failure, rate-limit error logging.
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
        """Send a zero-velocity stop command to halt the robot immediately.

        Algorithm:
          Calls send_command(0.0, 0.0) which sets both v and omega to zero.
        """
        self.send_command(0.0, 0.0)

    def poll_telemetry(self) -> dict | None:
        """Check for incoming telemetry from the robot on both ports.

        Returns:
            Dictionary of telemetry data (e.g. {'sonar_front': 120, 'yaw': 45.2}),
            or the last received telemetry if no new data is available,
            or None if no telemetry has ever been received.

        Algorithm:
          1. PRIMARY: Try to receive on the dedicated telemetry port (4211).
             - If data received, parse as JSON and store.
             - If the sender IP differs from current host, auto-update the host
               (handles DHCP IP changes transparently).
             - Return the parsed telemetry.
          2. FALLBACK: Try to receive on the command socket (reply channel).
             - Same auto-discovery logic as primary.
          3. If both ports have no data (timeout/BlockingIOError), return
             the last cached telemetry or None.
        """
        # 1. Primary: dedicated telemetry port (4211/4221)
        try:
            data, addr = self._telem_sock.recvfrom(1024)
            self._last_telemetry = json.loads(data.decode("utf-8"))
            self.last_telemetry_time = time.time()
            # Auto-discover robot IP from telemetry source
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
            # Auto-discover robot IP from telemetry source
            if addr and addr[0] != self.host:
                _log.info("Auto-discovered Robot ESP32 at IP: %s (updated from %s)", addr[0], self.host)
                self.host = addr[0]
            return self._last_telemetry
        except (socket.timeout, BlockingIOError, json.JSONDecodeError, OSError):
            pass

        return self._last_telemetry

    def seconds_since_telemetry(self) -> float:
        """Compute seconds elapsed since the last telemetry was received.

        Returns:
            Float seconds since last telemetry, or float('inf') if never received.

        Algorithm:
          1. If last_telemetry_time is 0.0 (never received), return infinity.
          2. Otherwise, return current_time - last_telemetry_time.
        """
        if self.last_telemetry_time == 0.0:
            return float("inf")
        return time.time() - self.last_telemetry_time

    def close(self) -> None:
        """Shut down the client: stop heartbeat, send stop command, close sockets.

        Algorithm:
          1. Set _running=False to signal heartbeat thread to exit.
          2. Join the heartbeat thread with a short timeout.
          3. Send a final stop command to halt the robot.
          4. Close both UDP sockets.
        """
        self._running = False
        if self._heartbeat_thread.is_alive():
            self._heartbeat_thread.join(timeout=0.1)
        self.send_stop()
        self._sock.close()
        self._telem_sock.close()


class RateLimiter:
    """Simple time-based rate limiter for controlling command send frequency.

    Attributes:
        period: Minimum seconds between consecutive ready() calls.
        _last: Timestamp of the last time ready() returned True.
    """

    def __init__(self, rate_hz: float) -> None:
        """Initialize the rate limiter.

        Args:
            rate_hz: Maximum frequency in Hz (e.g. 20.0 for 20 commands/second).
        """
        self.period = 1.0 / rate_hz
        self._last = 0.0

    def ready(self) -> bool:
        """Check if enough time has passed to allow the next command.

        Returns:
            True if the rate limit has been reached (caller should proceed).
            False if the caller should skip this cycle.

        Algorithm:
          1. Get current timestamp.
          2. If elapsed time since last ready >= period, update _last and return True.
          3. Otherwise return False (caller should wait).
        """
        now = time.time()
        if now - self._last >= self.period:
            self._last = now
            return True
        return False
