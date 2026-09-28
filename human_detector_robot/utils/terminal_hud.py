#!/usr/bin/env python3
"""
ARK-5 Sophisticated Terminal HUD & Live Telemetry Logger
Provides a clean, tactical real-time dashboard in the terminal showing:
 - Motor actuation state (ON/OFF, Direction, Speeds)
 - Live sensor readings (Sonar Front/Left/Right — FILTERED (TASK-01/02),
   MPU Yaw (jump-guarded), WiFi RSSI (hint-only, TASK-10))
 - Planner status (A* path + frontier goal — TASK-04/05/06, TASK-12)
 - Human vision detection status + unique-human count (TASK-09/12)
 - Swarm coordination & anti-collision state

Rendering notes:
 - ANSI escape codes colorize output; on Windows an empty `os.system("")`
   call enables VT processing, and stdout is reconfigured to UTF-8.
 - Rendering is throttled (refresh_interval_s) so the terminal is never
   flooded; a state change always forces an immediate redraw.
"""

from __future__ import annotations

import os
import sys
import time
from typing import Optional

# Ensure UTF-8 output on Windows (prevents UnicodeEncodeError on box chars)
if hasattr(sys.stdout, "reconfigure"):
    try:
        sys.stdout.reconfigure(encoding="utf-8")
    except Exception:
        pass

# Enable ANSI on Windows (empty command switches the console to VT mode)
if os.name == "nt":
    os.system("")

# ANSI Color Codes (SGR sequences)
RESET   = "\033[0m"   # reset all attributes
BOLD    = "\033[1m"   # bright/bold text
DIM     = "\033[2m"   # dim text
RED     = "\033[91m"
GREEN   = "\033[92m"
YELLOW  = "\033[93m"
BLUE    = "\033[94m"
MAGENTA = "\033[95m"
CYAN    = "\033[96m"
WHITE   = "\033[97m"
BG_RED  = "\033[41m"  # red background (critical banners)
BG_BLUE = "\033[44m"


def motor_status_str(v: float, omega: float, state_name: str, failsafe: bool) -> tuple[str, str]:
    """Returns (status_badge, details) for motor state.

    Args:
        v: Linear velocity in m/s (positive=forward, negative=reverse).
        omega: Angular velocity in rad/s (positive=CCW/left).
        state_name: Current FSM state name string.
        failsafe: True when telemetry is stale / motors force-stopped.

    Returns:
        (badge, details): ANSI-colored status label and a human-readable
        speed/direction description.

    Algorithm:
      1. failsafe → red "[MOTORS OFF - FAILSAFE]".
      2. |v|<0.01 and |omega|<0.01 (stopped): classify by state name —
         ALERT/PAUSE → magenta PAUSED; WALL/OBSTACLE → red STOPPED;
         else → dim IDLE.
      3. Moving: v>0.03 & |omega|<0.10 → green FORWARD;
         v>0.03 & omega>0.10 → yellow ARC RIGHT; v>0.03 & omega<-0.10 →
         yellow ARC LEFT; v<-0.01 → yellow REVERSING;
         omega>0.15 (no fwd) → cyan SPIN RIGHT; omega<-0.15 → cyan SPIN LEFT;
         else → green generic MOTORS ON.
    """
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
    """Formats ultrasonic reading with colored status badge.

    Args:
        val: Distance in cm (should already be FILTERED — TASK-01).
        name: Sensor label (kept for API compatibility; not printed here).
        threshold_warn: Distance (cm) at which the APPROACHING warning begins.

    Returns:
        ANSI-colored string like " 45 cm [APPROACHING]".

    Algorithm (threshold ladder):
      1. val < 0 or > 400 → red "[FAULT/TIMEOUT]" (invalid/timeout).
      2. val <= 35 → bold red "[OBSTACLE STOP]" (safety-critical).
      3. val <= threshold_warn → bold yellow "[APPROACHING]".
      4. otherwise → green "[PATH CLEAR]".
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
        interval: Minimum seconds between redraws (default 0.35 ≈ 3 fps).
        last_render_time: Timestamp of the previous redraw.
        last_state: State name used for the previous redraw — any state
            change forces an immediate redraw regardless of the interval.
    """

    def __init__(self, refresh_interval_s: float = 0.35):
        """Initialize the renderer.

        Args:
            refresh_interval_s: Minimum seconds between HUD refreshes.
        """
        self.interval = refresh_interval_s
        self.last_render_time = 0.0
        self.last_state = ""

    def should_render(self, state_name: str) -> bool:
        """Decide whether this tick may redraw the HUD.

        Args:
            state_name: Current FSM state name (or a combined key for swarm).

        Returns:
            True if a redraw is allowed now, False if rate-limited.

        Algorithm:
          1. Redraw when the state name changed (immediate feedback), OR
          2. when ≥ interval has elapsed since the last redraw.
          3. On True, stamp last_render_time and remember the state.
        """
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
        astar_path_len: int = 0,           # TASK-12: A* path cells count (0 = no active plan)
        human_count: int = 0,              # TASK-12: unique tracked humans (TASK-09)
        frontier_goal: Optional[tuple[float, float]] = None,  # TASK-12: next exploration target
    ) -> None:
        """Prints a sophisticated tactical dashboard for a single robot.

        Args:
            robot_id: Robot number (1 or 2).
            name: Display name (e.g. 'Robot_Alpha').
            state_name: Current FSM state name.
            v: Commanded linear velocity (m/s).
            omega: Commanded angular velocity (rad/s).
            yaw: FILTERED MPU heading (deg).
            dist_front / dist_left / dist_right: FILTERED sonar readings (cm).
            dist_traveled_cm: Odometer from TentativeMap.
            sense_rssi: WiFi RSSI (0 = inactive/not sent).
            cv_alert: Vision alert string or None.
            fps: Current camera processing frame rate.
            stream_url: Active camera stream URL (unused in printout,
                kept for API compatibility/future use).
            failsafe: True when telemetry is stale.
            is_online: True when telemetry is fresh.
            astar_path_len: TASK-12 — remaining A* waypoints (0 = no plan).
            human_count: TASK-12 — unique humans logged by HumanTracker.
            frontier_goal: TASK-12 — nearest frontier (x, y) in meters or None.

        Algorithm:
          1. Rate-limit via should_render(state_name); skip if not allowed.
          2. Build badges: motor (motor_status_str), sonar front/left/right
             (sensor_badge), yaw, link, human status.
          3. TASK-12: planner line — green "A* PATH ACTIVE (n cells)" when
             astar_path_len > 0, else dim "scan/rotate mode"; frontier goal
             rendered in cm or "none (area fully explored)".
          4. Print four boxed sections: Motors & Movement (badge, details,
             odometer), Live Sensors (3 sonars + yaw + optional WiFi),
             Planner (plan + next goal), Rescue & Vision (human badge +
             unique-human count).
        """
        if not self.should_render(state_name):
            return

        motor_badge, motor_desc = motor_status_str(v, omega, state_name, failsafe)
        front_badge = sensor_badge(dist_front, "Front", 100)
        left_badge  = sensor_badge(dist_left, "Left", 100)
        right_badge = sensor_badge(dist_right, "Right", 100)

        # Yaw status
        yaw_badge = f"{CYAN}{yaw:+6.1f} deg{RESET}"
        
        # Link status
        link_badge = f"{BOLD}{GREEN}[ONLINE]{RESET}" if is_online and not failsafe else f"{BOLD}{RED}[DISCONNECTED]{RESET}"
        
        # Human status (posture-aware coloring)
        if cv_alert:
            if "FALLEN" in cv_alert.upper():
                human_badge = f"{BOLD}{BG_RED}{WHITE} [!] HUMAN FALLEN DETECTED {RESET} -> {cv_alert}"
            else:
                human_badge = f"{BOLD}{YELLOW} [*] HUMAN STANDING DETECTED {RESET} -> {cv_alert}"
        else:
            human_badge = f"{GREEN}[v] CLEAR -- No human in view{RESET}"

        # TASK-12: planner status line
        if astar_path_len > 0:
            plan_badge = f"{GREEN}A* PATH ACTIVE ({astar_path_len} cells){RESET}"
        else:
            plan_badge = f"{DIM}No active A* plan (scan/rotate mode){RESET}"
        frontier_str = f"({frontier_goal[0]*100:+.0f}cm, {frontier_goal[1]*100:+.0f}cm)" if frontier_goal else "none (area fully explored)"

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
        print(f"| {BOLD}2. LIVE SENSOR READINGS (filtered -- TASK-01/02):{RESET}                          |")
        print(f"|    * Sonar Front : {front_badge:<54} |")
        print(f"|    * Sonar Left  : {left_badge:<54} |")
        print(f"|    * Sonar Right : {right_badge:<54} |")
        print(f"|    * MPU-6050 Yaw: {yaw_badge} (jump-guarded)                        |")
        if sense_rssi != 0:
            print(f"|    * WiFi Sensing: {sense_rssi:4d} dBm [hint-only -- TASK-10]                     |")
        print(bar)
        print(f"| {BOLD}3. PLANNER (A* + Frontiers -- TASK-04/05/06):{RESET}                              |")
        print(f"|    Plan      : {plan_badge:<58} |")
        print(f"|    Next goal : {frontier_str:<58} |")
        print(bar)
        print(f"| {BOLD}4. RESCUE & VISION DETECTION:{RESET}                                               |")
        print(f"|    Target Status : {human_badge:<50} |")
        print(f"|    Unique humans logged (TASK-09): {human_count:<3}                                    |")
        print(bar)


    def render_swarm(
        self,
        r1,
        r2,
        inter_dist_m: float,
        collision_warning: bool,
    ) -> None:
        """Prints side-by-side tactical telemetry for both swarm robots.

        Args:
            r1: Robot 1 context (RobotContext) — must expose .fsm, .last_v,
                .last_omega, .failsafe_active, .dist_front_cm, .dist_left_cm,
                .yaw, .status_msg, .tmap, .human_tracker.
            r2: Robot 2 context (same interface).
            inter_dist_m: Euclidean distance between robots (meters).
            collision_warning: True when inside the 0.8m safety bubble.

        Algorithm:
          1. Build a combined state key "STATE1_STATE2" and rate-limit.
          2. Motor badges for both robots (last_v/last_omega — TASK-12 bugfix:
             these fields must exist on the contexts or this raises).
          3. Sensor badges for front/left of each robot.
          4. Print: header, proximity line (red alert if collision_warning,
             else green "[ACTIVE - SAFE]"), then a two-column comparison of
             Link/State, Motors, Front, Left, Yaw+Odometer, Status, and
             unique-human counts (TASK-09) for each robot.
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

        # TASK-12: unique human counts (TASK-09) per robot
        h1 = r1.human_tracker.count if hasattr(r1, "human_tracker") else 0
        h2 = r2.human_tracker.count if hasattr(r2, "human_tracker") else 0

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
        print(f"  Unique humans (TASK-09): {h1:<3}            |  Unique humans (TASK-09): {h2:<3}")
        print(bar)
