#!/usr/bin/env python3
"""
ARK-5 Sophisticated Terminal HUD & Live Telemetry Logger.

Provides a clean, tactical real-time dashboard in the terminal showing:
  - Motor actuation state (ON/OFF, Direction, Speeds)
  - Live sensor readings (Sonar Front/Left, MPU Yaw, WiFi RSSI)
  - Human vision detection status
  - Swarm coordination & anti-collision state

Uses ANSI escape codes for colorized output on both Linux and Windows.
Rendering is throttled to avoid flooding the terminal (max ~3 fps).
"""

from __future__ import annotations

import os
import sys
import time
from typing import Optional

# Ensure UTF-8 output on Windows to prevent encoding errors
if hasattr(sys.stdout, "reconfigure"):
    try:
        sys.stdout.reconfigure(encoding="utf-8")
    except Exception:
        pass

# Enable ANSI escape code processing on Windows (10+)
if os.name == "nt":
    os.system("")

# ANSI Color Codes for terminal formatting
RESET   = "\033[0m"     # Reset all formatting
BOLD    = "\033[1m"      # Bold/bright text
DIM     = "\033[2m"      # Dim/faded text
RED     = "\033[91m"     # Bright red
GREEN   = "\033[92m"     # Bright green
YELLOW  = "\033[93m"     # Bright yellow
BLUE    = "\033[94m"     # Bright blue
MAGENTA = "\033[95m"     # Bright magenta
CYAN    = "\033[96m"     # Bright cyan
WHITE   = "\033[97m"     # Bright white
BG_RED  = "\033[41m"     # Red background
BG_BLUE = "\033[44m"     # Blue background


def motor_status_str(v: float, omega: float, state_name: str, failsafe: bool) -> tuple[str, str]:
    """Generate a colorized motor status badge and description string.

    Args:
        v: Linear velocity in m/s (positive=forward, negative=reverse).
        omega: Angular velocity in rad/s (positive=left/CCW).
        state_name: Current FSM state name string.
        failsafe: True if the robot is in failsafe mode (no telemetry).

    Returns:
        Tuple of (status_badge, details):
          - status_badge: ANSI-colorized string showing motor state icon + label.
          - details: Human-readable description of speed/direction.

    Algorithm:
      1. If failsafe is True, show FAILSAFE message (red).
      2. If both v and omega are near zero (stopped):
         a. If in ALERT/PAUSE state → show PAUSED (magenta).
         b. If in WALL/OBSTACLE state → show STOPPED (red).
         c. Otherwise → show IDLE (dim).
      3. If motors are ON (v or omega non-zero):
         a. Forward + straight → GREEN FORWARD.
         b. Forward + turning → YELLOW ARC LEFT/RIGHT.
         c. Reversing → YELLOW REVERSING.
         d. Spinning in place (v≈0, omega large) → CYAN SPIN LEFT/RIGHT.
         e. Otherwise → GREEN generic MOTORS ON.
    """
    if failsafe:
        return f"{BOLD}{RED}[MOTORS OFF - FAILSAFE]{RESET}", "No telemetry received from robot"
    if abs(v) < 0.01 and abs(omega) < 0.01:
        if "ALERT" in state_name or "PAUSE" in state_name:
            return f"{BOLD}{MAGENTA}[MOTORS PAUSED]{RESET}", f"Human detected ({state_name})"
        if "WALL" in state_name or "OBSTACLE" in state_name:
            return f"{BOLD}{RED}[MOTORS STOPPED]{RESET}", "Obstacle in safety zone"
        return f"{DIM}[MOTORS IDLE]{RESET}", f"Holding position ({state_name})"

    # Motors are ON — classify the motion type
    if v > 0.03 and abs(omega) < 0.10:
        return (
            f"{BOLD}{GREEN}[* MOTORS ON - FORWARD]{RESET}",
            f"Speed: {v:+.2f} m/s | Straight cruising",
        )
    elif v > 0.03 and omega > 0.10:
        return (
            f"{BOLD}{YELLOW}[* MOTORS ON - ARC RIGHT]{RESET}",
            f"Speed: {v:+.2f} m/s | Turn: {omega:+.2f} rad/s",
        )
    elif v > 0.03 and omega < -0.10:
        return (
            f"{BOLD}{YELLOW}[* MOTORS ON - ARC LEFT]{RESET}",
            f"Speed: {v:+.2f} m/s | Turn: {omega:+.2f} rad/s",
        )
    elif v < -0.01:
        return (
            f"{BOLD}{YELLOW}[* MOTORS ON - REVERSING]{RESET}",
            f"Speed: {v:+.2f} m/s | Backing up",
        )
    elif omega > 0.15:
        return (
            f"{BOLD}{CYAN}[* MOTORS ON - SPIN RIGHT]{RESET}",
            f"Angular omega: {omega:+.2f} rad/s (In-place turn)",
        )
    elif omega < -0.15:
        return (
            f"{BOLD}{CYAN}[* MOTORS ON - SPIN LEFT]{RESET}",
            f"Angular omega: {omega:+.2f} rad/s (In-place turn)",
        )
    else:
        return (
            f"{BOLD}{GREEN}[* MOTORS ON]{RESET}",
            f"v={v:+.2f} m/s, w={omega:+.2f} rad/s",
        )


def sensor_badge(val: int, name: str, threshold_warn: int = 100) -> str:
    """Format an ultrasonic sensor reading with a color-coded status badge.

    Args:
        val: Ultrasonic distance reading in centimeters.
        name: Sensor name label (e.g. 'Front', 'Left').
        threshold_warn: Distance threshold (cm) for the APPROACHING warning zone.

    Returns:
        ANSI-colorized string showing distance + status label.

    Algorithm:
      1. If val is out of valid range (<0 or >400), show FAULT/TIMEOUT (red).
      2. If val <= 35cm, show OBSTACLE STOP (bold red) — critical proximity.
      3. If val <= threshold_warn (default 100cm), show APPROACHING (yellow).
      4. Otherwise, show PATH CLEAR (green).
    """
    if val < 0 or val > 400:
        return f"{RED}--- cm [FAULT/TIMEOUT]{RESET}"
    if val <= 35:
        return f"{BOLD}{RED}{val:3d} cm [OBSTACLE STOP]{RESET}"
    if val <= threshold_warn:
        return f"{BOLD}{YELLOW}{val:3d} cm [APPROACHING]{RESET}"
    return f"{GREEN}{val:3d} cm [PATH CLEAR]{RESET}"


class TerminalDashboard:
    """Manages periodic clean terminal rendering without flooding the screen.

    Attributes:
        interval: Minimum seconds between renders (controls HUD refresh rate).
        last_render_time: Timestamp of the last render call.
        last_state: Last FSM state name (forces re-render on state change).
    """

    def __init__(self, refresh_interval_s: float = 0.35):
        """Initialize the dashboard renderer.

        Args:
            refresh_interval_s: Minimum seconds between HUD refreshes.
                               Default 0.35s gives ~3 fps display update rate.
        """
        self.interval = refresh_interval_s
        self.last_render_time = 0.0
        self.last_state = ""

    def should_render(self, state_name: str) -> bool:
        """Determine if the HUD should re-render based on time or state change.

        Args:
            state_name: Current FSM state name.

        Returns:
            True if rendering should occur, False to skip (rate-limited).

        Algorithm:
          1. If the state name changed since last render, return True (immediate update).
          2. If the time interval has elapsed, return True and update timestamp.
          3. Otherwise return False to skip this render cycle.
        """
        now = time.time()
        if state_name != self.last_state or (now - self.last_render_time) >= self.interval:
            self.last_render_time = now
            self.last_state = state_name
            return True
        return False

    def render_single_robot(
        self,
        robot_id: int,
        name: str,
        state_name: str,
        v: float,
        omega: float,
        yaw: float,
        dist_front: int,
        dist_left: int,
        dist_right: int,
        dist_traveled_cm: float,
        sense_rssi: int,
        cv_alert: Optional[str],
        fps: float,
        stream_url: str,
        failsafe: bool = False,
        is_online: bool = True,
    ) -> None:
        """Print a sophisticated tactical dashboard for a single robot.

        Args:
            robot_id: Robot identifier number (1 or 2).
            name: Human-readable robot name (e.g. 'Alpha').
            state_name: Current FSM state name string.
            v: Linear velocity in m/s.
            omega: Angular velocity in rad/s.
            yaw: Current heading from MPU6050 in degrees.
            dist_front: Front ultrasonic distance in cm.
            dist_left: Left ultrasonic distance in cm.
            dist_right: Right ultrasonic distance in cm.
            dist_traveled_cm: Total distance traveled in centimeters.
            sense_rssi: WiFi RSSI sensing value (0 if inactive).
            cv_alert: Computer vision alert string or None.
            fps: Current camera processing frame rate.
            stream_url: Active camera stream URL.
            failsafe: True if robot is in failsafe mode.
            is_online: True if robot is connected.

        Algorithm:
          1. Check if render should occur (rate-limited by should_render).
          2. Generate motor status badge via motor_status_str().
          3. Generate sensor badges via sensor_badge() for front and left sonar.
          4. Determine yaw display, link status, and human detection status.
          5. Print a formatted box dashboard with sections:
             - Header with robot ID, name, and state.
             - Motors & Movement section with badge, details, and odometer.
             - Live Sensor Readings section with sonar and MPU data.
             - Rescue & Vision Detection section with human status.
        """
        if not self.should_render(state_name):
            return

        motor_badge, motor_desc = motor_status_str(v, omega, state_name, failsafe)
        front_badge = sensor_badge(dist_front, "Front", 100)
        left_badge  = sensor_badge(dist_left, "Left", 100)

        # Yaw display
        yaw_badge = f"{CYAN}{yaw:+6.1f} deg{RESET}"
        
        # Connection link status
        link_badge = f"{BOLD}{GREEN}[ONLINE]{RESET}" if is_online and not failsafe else f"{BOLD}{RED}[DISCONNECTED]{RESET}"
        
        # Human detection status with posture-specific styling
        if cv_alert:
            if "FALLEN" in cv_alert.upper():
                human_badge = f"{BOLD}{BG_RED}{WHITE} [!] HUMAN FALLEN DETECTED {RESET} -> {cv_alert}"
            else:
                human_badge = f"{BOLD}{YELLOW} [*] HUMAN STANDING DETECTED {RESET} -> {cv_alert}"
        else:
            human_badge = f"{GREEN}[v] CLEAR -- No human in view{RESET}"

        bar = "+-----------------------------------------------------------------------------+"
        
        # Print formatted dashboard sections
        print("\n" + bar)
        print(f"| {BOLD}{CYAN}ARK-5 ROBOT #{robot_id} ({name}) -- LIVE TACTICAL DASHBOARD{RESET}{' ' * (31 - len(name))} |")
        print(bar)
        print(f"| Link: {link_badge} | State: {BOLD}{WHITE}{state_name:<16}{RESET} | Stream: {fps:4.1f} FPS              |")
        print(bar)
        print(f"| {BOLD}1. MOTORS & MOVEMENT:{RESET}                                                       |")
        print(f"|    Actuation : {motor_badge:<58} |")
        print(f"|    Details   : {motor_desc:<58} |")
        print(f"|    Odometer  : {dist_traveled_cm:5.0f} cm traveled along mission path                 |")
        print(bar)
        print(f"| {BOLD}2. LIVE SENSOR READINGS:{RESET}                                                    |")
        print(f"|    * Sonar Front : {front_badge:<54} |")
        print(f"|    * Sonar Left  : {left_badge:<54} |")
        print(f"|    * MPU-6050 Yaw: {yaw_badge} (Heading stabilization active)            |")
        if sense_rssi != 0:
            print(f"|    * WiFi Sensing: {sense_rssi:4d} dBm [2.4GHz RF Body Sensing Active]             |")
        print(bar)
        print(f"| {BOLD}3. RESCUE & VISION DETECTION:{RESET}                                               |")
        print(f"|    Target Status : {human_badge:<50} |")
        print(bar)


    def render_swarm(
        self,
        r1,
        r2,
        inter_dist_m: float,
        collision_warning: bool,
    ) -> None:
        """Print side-by-side tactical telemetry for both swarm robots.

        Args:
            r1: Robot 1 controller object (must have .fsm, .last_v, .last_omega,
                .failsafe_active, .dist_front_cm, .dist_left_cm, .yaw,
                .tmap.total_distance_cm, .status_msg attributes).
            r2: Robot 2 controller object (same interface as r1).
            inter_dist_m: Distance between the two robots in meters.
            collision_warning: True if robots are too close (proximity alert).

        Algorithm:
          1. Create a combined state string from both robots' FSM states.
          2. Check if render should occur (rate-limited).
          3. Generate motor and sensor badges for both robots.
          4. Print a side-by-side formatted dashboard:
             - Header with swarm title.
             - Inter-robot spacing and collision status.
             - Robot 1 (Alpha, left bias) vs Robot 2 (Beta, right bias) columns.
             - Link, Motors, Front sonar, Left sonar, Yaw/Odometer, Human status.
        """
        combo_state = f"{r1.fsm.state.name}_{r2.fsm.state.name}"
        if not self.should_render(combo_state):
            return

        m1_badge, m1_desc = motor_status_str(r1.last_v, r1.last_omega, r1.fsm.state.name, r1.failsafe_active)
        m2_badge, m2_desc = motor_status_str(r2.last_v, r2.last_omega, r2.fsm.state.name, r2.failsafe_active)

        f1 = sensor_badge(r1.dist_front_cm, "F1", 100)
        f2 = sensor_badge(r2.dist_front_cm, "F2", 100)
        l1 = sensor_badge(r1.dist_left_cm, "L1", 100)
        l2 = sensor_badge(r2.dist_left_cm, "L2", 100)

        dist1 = r1.tmap.total_distance_cm if r1.tmap else 0.0
        dist2 = r2.tmap.total_distance_cm if r2.tmap else 0.0

        bar = "=" * 79

        print("\n" + bar)
        print(f"{BOLD}{CYAN}  ARK-5 SWARM CONTROLLER -- DUAL-ROBOT TACTICAL TELEMETRY HUD{RESET}")
        print(bar)

        # Inter-robot proximity warning
        if collision_warning:
            print(f"{BOLD}{BG_RED}{WHITE}  [!] SWARM PROXIMITY ALERT: {inter_dist_m:.2f}m apart -- ROBOT 2 YIELDING  {RESET}")
        else:
            print(f"  Swarm Distance: {inter_dist_m:.2f}m | Mutual Anti-Collision: {GREEN}[ACTIVE - SAFE]{RESET}")
        print("-" * 79)

        # Side-by-side robot comparison columns
        print(f"  {BOLD}ROBOT #1 (Alpha - Left Bias){RESET}          |  {BOLD}ROBOT #2 (Beta - Right Bias){RESET}")
        print(f"  Link   : {GREEN}ONLINE{RESET} ({r1.fsm.state.name:<13})    |  Link   : {GREEN}ONLINE{RESET} ({r2.fsm.state.name:<13})")
        print(f"  Motors : {m1_badge:<32} |  Motors : {m2_badge:<32}")
        print(f"  Front  : {f1:<32} |  Front  : {f2:<32}")
        print(f"  Left   : {l1:<32} |  Left   : {l2:<32}")
        print(f"  Yaw    : {r1.yaw:+5.1f} deg | Odo: {dist1:4.0f}cm       |  Yaw    : {r2.yaw:+5.1f} deg | Odo: {dist2:4.0f}cm")
        print(f"  Human  : {r1.status_msg:<27} |  Human  : {r2.status_msg:<27}")
        print(bar)
