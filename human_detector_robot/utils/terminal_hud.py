#!/usr/bin/env python3
"""
ARK-5 Sophisticated Terminal HUD & Live Telemetry Logger
Provides a clean, tactical real-time dashboard in the terminal showing:
 - Motor actuation state (ON/OFF, Direction, Speeds)
 - Live sensor readings (Sonar Front/Left, MPU Yaw, WiFi RSSI)
 - Human vision detection status
 - Swarm coordination & anti-collision state
"""

from __future__ import annotations

import os
import sys
import time
from typing import Optional

# Ensure UTF-8 output on Windows
if hasattr(sys.stdout, "reconfigure"):
    try:
        sys.stdout.reconfigure(encoding="utf-8")
    except Exception:
        pass

# Enable ANSI on Windows
if os.name == "nt":
    os.system("")

# ANSI Color Codes
RESET   = "\033[0m"
BOLD    = "\033[1m"
DIM     = "\033[2m"
RED     = "\033[91m"
GREEN   = "\033[92m"
YELLOW  = "\033[93m"
BLUE    = "\033[94m"
MAGENTA = "\033[95m"
CYAN    = "\033[96m"
WHITE   = "\033[97m"
BG_RED  = "\033[41m"
BG_BLUE = "\033[44m"


def motor_status_str(v: float, omega: float, state_name: str, failsafe: bool) -> tuple[str, str]:
    """Returns (status_badge, details) for motor state."""
    if failsafe:
        return f"{BOLD}{RED}[MOTORS OFF - FAILSAFE]{RESET}", "No telemetry received from robot"
    if abs(v) < 0.01 and abs(omega) < 0.01:
        if "ALERT" in state_name or "PAUSE" in state_name:
            return f"{BOLD}{MAGENTA}[MOTORS PAUSED]{RESET}", f"Human detected ({state_name})"
        if "WALL" in state_name or "OBSTACLE" in state_name:
            return f"{BOLD}{RED}[MOTORS STOPPED]{RESET}", "Obstacle in safety zone"
        return f"{DIM}[MOTORS IDLE]{RESET}", f"Holding position ({state_name})"

    # Motors are ON
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
    """Formats ultrasonic reading with colored status badge."""
    if val < 0 or val > 400:
        return f"{RED}--- cm [FAULT/TIMEOUT]{RESET}"
    if val <= 35:
        return f"{BOLD}{RED}{val:3d} cm [OBSTACLE STOP]{RESET}"
    if val <= threshold_warn:
        return f"{BOLD}{YELLOW}{val:3d} cm [APPROACHING]{RESET}"
    return f"{GREEN}{val:3d} cm [PATH CLEAR]{RESET}"


class TerminalDashboard:
    """Manages periodic clean terminal rendering without flooding the screen."""

    def __init__(self, refresh_interval_s: float = 0.35):
        self.interval = refresh_interval_s
        self.last_render_time = 0.0
        self.last_state = ""

    def should_render(self, state_name: str) -> bool:
        now = time.time()
        # Force render if state changed or interval elapsed
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
        """Prints a sophisticated tactical dashboard for a single robot."""
        if not self.should_render(state_name):
            return

        motor_badge, motor_desc = motor_status_str(v, omega, state_name, failsafe)
        front_badge = sensor_badge(dist_front, "Front", 100)
        left_badge  = sensor_badge(dist_left, "Left", 100)

        # Yaw status
        yaw_badge = f"{CYAN}{yaw:+6.1f} deg{RESET}"
        
        # Link status
        link_badge = f"{BOLD}{GREEN}[ONLINE]{RESET}" if is_online and not failsafe else f"{BOLD}{RED}[DISCONNECTED]{RESET}"
        
        # Human status
        if cv_alert:
            if "FALLEN" in cv_alert.upper():
                human_badge = f"{BOLD}{BG_RED}{WHITE} [!] HUMAN FALLEN DETECTED {RESET} -> {cv_alert}"
            else:
                human_badge = f"{BOLD}{YELLOW} [*] HUMAN STANDING DETECTED {RESET} -> {cv_alert}"
        else:
            human_badge = f"{GREEN}[v] CLEAR -- No human in view{RESET}"

        bar = "+-----------------------------------------------------------------------------+"
        
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
        """Prints side-by-side tactical telemetry for both swarm robots."""
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

        # Inter-robot spacing
        if collision_warning:
            print(f"{BOLD}{BG_RED}{WHITE}  [!] SWARM PROXIMITY ALERT: {inter_dist_m:.2f}m apart -- ROBOT 2 YIELDING  {RESET}")
        else:
            print(f"  Swarm Distance: {inter_dist_m:.2f}m | Mutual Anti-Collision: {GREEN}[ACTIVE - SAFE]{RESET}")
        print("-" * 79)

        # Robot 1 vs Robot 2
        print(f"  {BOLD}ROBOT #1 (Alpha - Left Bias){RESET}          |  {BOLD}ROBOT #2 (Beta - Right Bias){RESET}")
        print(f"  Link   : {GREEN}ONLINE{RESET} ({r1.fsm.state.name:<13})    |  Link   : {GREEN}ONLINE{RESET} ({r2.fsm.state.name:<13})")
        print(f"  Motors : {m1_badge:<32} |  Motors : {m2_badge:<32}")
        print(f"  Front  : {f1:<32} |  Front  : {f2:<32}")
        print(f"  Left   : {l1:<32} |  Left   : {l2:<32}")
        print(f"  Yaw    : {r1.yaw:+5.1f} deg | Odo: {dist1:4.0f}cm       |  Yaw    : {r2.yaw:+5.1f} deg | Odo: {dist2:4.0f}cm")
        print(f"  Human  : {r1.status_msg:<27} |  Human  : {r2.status_msg:<27}")
        print(bar)
