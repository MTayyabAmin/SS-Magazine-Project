#!/usr/bin/env python3
"""
ARK-5 Human Detector Robot — Main Controller (Laptop Brain)

CV (YOLOv8) + WiFi sensing + Ultrasonic wall stop + MPU dead-reckoning
+ tentative ASCII map + L-turn corner scan algorithm.

Architecture
-----------
Single-robot control loop (run once per robot with --robot 1|2):

  1. Setup: parse args, load + validate project.yaml, resolve the target
     robot's UDP/stream/map paths, warm up YOLO, open the camera stream,
     and build the sensor filters, FSM, occupancy grid, watchdog,
     human tracker, WiFi sensor, and optional ASCII map.
  2. Loop (per frame):
     a. Read frame; run YOLO only every `infer_every_n` frames (reuse
        last detections in between to protect CPU/FPS).
     b. Poll UDP telemetry; clean sonar + yaw through TASK-01/02 filters
        so invalid -1s / spikes / yaw teleports never reach the FSM.
     c. Track telemetry staleness → motors-stop FAILSAFE (FIX).
     d. WiFi sense → HINT only; escalates to an alert only with CV
        confirmation or 3+ consecutive drops plus a wall ahead (TASK-10).
     e. Update occupancy grid + A* frontier plan → target heading for
        CRUISE (TASK-04/05/06); fall back to FSM scan if no path.
     f. FSM step → (v, omega); failsafe and StuckWatchdog may override
        (TASK-08). The chosen command plus its SOURCE (DEBUG / FSM /
        FAILSAFE / WATCHDOG) is then written to the [CMD] attribution log
        so a stray wheel movement can be traced after the fact. Finally the
        DEBUG_FORCE_FORWARD flag at the top of this file, when set, forces a
        straight-line drive and overrides everything above.
     g. Update pose/map/walls (dead-reckoning with failsafe awareness,
        TASK-03); register unique humans (TASK-09); autosave map.
     h. Send rate-limited motor command; render HUD (TASK-12) and
        optional OpenCV preview.
  3. Teardown: save final map + trajectory, close UDP/stream/windows.
"""

from __future__ import annotations

# ═════════════════════════════════════════════════════════════════════════════
# DEBUG FLAGS — deliberately kept at the TOP of this file, before any code.
# ═════════════════════════════════════════════════════════════════════════════
# DEBUG_FORCE_FORWARD = True  →  the robot ignores every condition evaluated
# downstream of it (FSM output, sonar / CV / WiFi decisions, the stale-
# telemetry FAILSAFE and the stuck watchdog) and simply keeps driving
# straight at DEBUG_FORWARD_V. Purpose: isolate drive / motor / IMU / encoder
# problems from the navigation logic while testing on the bench.
#
# When it is on:
#   * the outgoing command is forced to (DEBUG_FORWARD_V, 0.0) every tick,
#   * allow_creep is forced True, so the ESP32's 100cm wall-near stop
#     (US_STOP_CM) is bypassed as well,
#   * the [CMD] attribution log records the source as "DEBUG".
# It does NOT bypass the ESP32's <25cm hard stop (US_MIN_CM) — that one is
# firmware-side safety and is intentionally left alone.
#
# WARNING: set it back to False before any normal/autonomous run. It is a
# one-line edit on purpose — no CLI switch and no env var — so it cannot be
# enabled by accident or by a config typo.
DEBUG_FORCE_FORWARD: bool = True
DEBUG_FORWARD_V: float = 1      # m/s used while DEBUG_FORCE_FORWARD is on
# ═════════════════════════════════════════════════════════════════════════════

import argparse
import logging
import math
import sys
import time
from pathlib import Path

import cv2

cv2.setNumThreads(1)

sys.path.insert(0, str(Path(__file__).resolve().parent))

from utils.autonomous_fsm import AutonomousFSM
from utils.config_validation import validate_config
from utils.tentative_map import TentativeMap
from utils.udp_client import RateLimiter, UdpMotorClient, command_class
from utils.vision import (
    FpsCounter,
    detect_humans,
    draw_detections,
    draw_fps,
    load_config,
    open_stream,
    project_root,
    warmup_yolo,
)
from utils.terminal_hud import TerminalDashboard
from utils.wifi_sensing import WifiHumanSensor
from utils.sensor_filter import RobotSensorFilters
from utils.occupancy_grid import OccupancyGrid
from utils.astar import astar, path_to_waypoints, next_heading_deg
from utils.human_tracker import HumanTracker
from utils.watchdog import StuckWatchdog

HUMAN_MARK_COOLDOWN_S = 5.0

# FIX: pehle poore project mein sirf print() use ho raha tha — koi
# timestamp, koi log-level, koi file mein save nahi hota tha. Agar robot
# field mein crash ho jaye to pata lagana mushkil tha ke kab/kyun hua.
# Ab console + log file dono mein timestamped logs jaate hain.
def _setup_logging(root: Path) -> logging.Logger:
    """Configure timestamped logging to console + logs/ark5.log.

    FIX: pehle poore project mein sirf print() use ho raha tha — ab
    console + log file dono mein timestamped logs jaate hain.

    Args:
        root: Project root directory (logs/ is created underneath it).

    Returns:
        The "ark5.main" logger instance used throughout main().

    Algorithm:
      1. mkdir root/logs (exist_ok).
      2. basicConfig INFO with asctime/levelname/name format.
      3. Attach StreamHandler (console) + FileHandler (UTF-8 ark5.log).
      4. Return the named logger.
    """
    log_dir = root / "logs"
    log_dir.mkdir(exist_ok=True)
    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s [%(levelname)s] %(name)s: %(message)s",
        handlers=[
            logging.StreamHandler(),
            logging.FileHandler(log_dir / "ark5.log", encoding="utf-8"),
        ],
    )
    return logging.getLogger("ark5.main")


def _cv_alert_message(detections, thresholds: dict) -> tuple[str | None, str | None]:
    """Returns (alert_msg, map_source).

    FIX (thresholds): thresholds pehle yahan hardcoded the (0.4/0.5/0.55),
    jabke project.yaml mein alag `conf` value thi. Ab sab project.yaml ke
    `yolo.fallen_conf` / `standing_conf` / `person_conf` se aate hain.

    FIX (priority order — teammate ke suggestion par): agar frame mein
    EK SE ZYADA log dikhein (kuch khare, kuch gire huey), to rescue
    signal HAMESHA standing/alive logon ko pehle milna chahiye, fallen
    ko baad mein — chahe list mein fallen wala pehle number par kyun na
    ho. Isliye ab pehle SAARI detections mein standing dhoonda jata hai;
    sirf tab fallen check hota hai jab koi standing na mile.

    Args:
        detections: Vision detections (label/confidence/box objects).
        thresholds: Per-class confidence thresholds from project.yaml
            (keys: "fallen", "standing", "person").

    Returns:
        (alert_msg, map_source): alert message string + source class
        ("cv_standing"/"cv_fallen"/"cv") for the highest-priority
        detection meeting its threshold, or (None, None) if none qualify.

    Algorithm:
      1. Pass 1: any "standing" d with conf >= standing → alert + "cv_standing".
      2. Pass 2: any "fallen" d with conf >= fallen → alert + "cv_fallen".
      3. Pass 3: any "person" d with conf >= person → alert + "cv".
      4. Otherwise (None, None).
    """
    for d in detections:
        if d.label == "standing" and d.confidence >= thresholds["standing"]:
            return f"[CV] HUMAN STANDING detected (conf={d.confidence:.2f})", "cv_standing"
    for d in detections:
        if d.label == "fallen" and d.confidence >= thresholds["fallen"]:
            return f"[CV] HUMAN FALLEN detected (conf={d.confidence:.2f})", "cv_fallen"
    for d in detections:
        if d.label == "person" and d.confidence >= thresholds["person"]:
            return f"[CV] PERSON detected (conf={d.confidence:.2f})", "cv"
    return None, None


def main() -> None:
    """Run the single-robot control loop end-to-end (see module docstring).

    Args:
        None (reads argv via argparse):
          --robot {1,2}: target robot id (default: config active_robot).
          --config: path to project.yaml (default: discovered).
          --calibrate-wifi: reset the RSSI baseline instead of using it.
          --no-motors: vision + HUD only, no UDP motor commands.
          --no-map: disable the ASCII map/trajectory files.

    Returns:
        None.

    Algorithm:
      Setup (config → validate → robot resolution → YOLO/stream →
      clients/FSM/filters/grid/watchdog/tracker/WiFi/map), then the
      per-frame loop documented in the module header, then teardown in
      a finally block (final map save, UDP close, stream/Windows release).
      Ctrl+C exits cleanly via KeyboardInterrupt.
    """
    parser = argparse.ArgumentParser(description="ARK-5 Human Detector Robot (Multi-Robot Brain)")
    parser.add_argument("--robot", type=int, default=None, choices=[1, 2], help="Target Robot ID (1 or 2)")
    parser.add_argument("--config", default=None, help="Path to project.yaml")
    parser.add_argument("--calibrate-wifi", action="store_true", help="Calibrate WiFi baseline")
    parser.add_argument("--no-motors", action="store_true", help="Vision only, no motor commands")
    parser.add_argument("--no-map", action="store_true", help="Disable ASCII map file")
    args = parser.parse_args()

    config = load_config(Path(args.config) if args.config else None)
    root = project_root()
    log = _setup_logging(root)

    validate_config(config)

    # ── Resolve Active Robot (Robot 1 or Robot 2) ─────────────────────────
    robot_id = args.robot or config.get("active_robot", 1)
    robots_cfg = config.get("robots", {})
    target_cfg = robots_cfg.get(robot_id, {})

    udp_host = target_cfg.get("udp_host", config["robot"]["udp_host"])
    udp_port = target_cfg.get("udp_port", config["robot"]["udp_port"])
    telemetry_port = target_cfg.get("telemetry_port", config["robot"]["telemetry_port"])
    stream_url = target_cfg.get("stream_url", config["camera"]["stream_url"])
    map_file = target_cfg.get("map_file", f"maps/robot{robot_id}_map.txt")
    traj_file = target_cfg.get("trajectory_log", f"maps/robot{robot_id}_movement_log.txt")

    log.info("=" * 55)
    log.info("  ARK-5 Human Detector — Brain Starting for ROBOT #%d", robot_id)
    log.info("  ESP32 Host    : %s:%d (Telem port: %d)", udp_host, udp_port, telemetry_port)
    log.info("  Camera Stream : %s", stream_url)
    log.info("=" * 55)

    warmup_yolo(config)
    cap = open_stream(str(stream_url), robot_id=robot_id)
    log.info("Camera Stream OK (%s)", cap.stream_url)

    robot_cfg = config["robot"]
    ctrl_cfg = config.get("control", {})
    map_cfg = config.get("map", {})
    yolo_cfg = config.get("yolo", {})
    cv_thresholds = {
        "fallen": yolo_cfg.get("fallen_conf", 0.4),
        "standing": yolo_cfg.get("standing_conf", 0.5),
        "person": yolo_cfg.get("person_conf", 0.55),
    }
    infer_every_n = max(1, int(yolo_cfg.get("infer_every_n_frames", 1)))
    telemetry_timeout_s = float(robot_cfg.get("telemetry_timeout_s", 1.5))

    udp: UdpMotorClient | None = None
    limiter: RateLimiter | None = None
    if not args.no_motors:
        udp = UdpMotorClient(
            host=udp_host,
            port=udp_port,
            telemetry_port=telemetry_port,
        )
        limiter = RateLimiter(robot_cfg.get("command_rate_hz", 20))

    fsm = AutonomousFSM(
        v_cruise=ctrl_cfg.get("v_cruise", 0.22),
        v_creep=ctrl_cfg.get("v_creep", 0.06),
        omega_scan=ctrl_cfg.get("omega_scan", 0.35),
        wall_stop_cm=robot_cfg.get("wall_stop_cm", 100),
        scan_standoff_min_cm=ctrl_cfg.get("scan_standoff_min_cm", 30),
        scan_standoff_max_cm=ctrl_cfg.get("scan_standoff_max_cm", 40),
        open_path_cm=ctrl_cfg.get("open_path_cm", 120),
        scan_step_deg=ctrl_cfg.get("scan_step_deg", 15),
        scan_side_deg=ctrl_cfg.get("scan_side_deg", 90),
        alert_pause_s=ctrl_cfg.get("alert_pause_s", 2.0),
        alert_rearm_s=ctrl_cfg.get("alert_rearm_s", 3.0),
        align_tolerance_deg=ctrl_cfg.get("align_tolerance_deg", 4.0),
        creep_timeout_s=ctrl_cfg.get("creep_timeout_s", 3.0),
        center_gain=ctrl_cfg.get("center_gain", 0.0035),
        center_omega_max=ctrl_cfg.get("center_omega_max", 0.15),
        path_gain=ctrl_cfg.get("path_gain", 0.02),
        path_omega_max=ctrl_cfg.get("path_omega_max", 0.25),
    )

    # TASK-01/02: sonar median filter + yaw jump guard (D1)
    nav_cfg = config.get("navigation", {})
    filters = RobotSensorFilters(
        window=nav_cfg.get("sonar_filter_window", 5),
        max_jump_cm=nav_cfg.get("sonar_max_jump_cm", 80.0),
        max_yaw_deg_per_s=nav_cfg.get("yaw_max_deg_per_s", 90.0),
    )
    # TASK-04/05: occupancy grid the A* planner (TASK-06) plans on
    grid = OccupancyGrid(cell_size_m=nav_cfg.get("occupancy_cell_m", 0.05))
    # TASK-08: stuck / oscillation watchdog
    watchdog = StuckWatchdog(
        pose_stuck_timeout_s=nav_cfg.get("watchdog_pose_timeout_s", 4.0),
        state_timeout_s=nav_cfg.get("watchdog_state_timeout_s", 8.0),
    )
    # TASK-09: de-duplicated human/survivor tracker
    human_tracker = HumanTracker(merge_radius_m=nav_cfg.get("human_merge_radius_m", 0.8))
    astar_path_world: list[tuple[float, float]] = []
    astar_wp_index = 0
    next_replan_time = 0.0
    replan_interval_s = nav_cfg.get("replan_interval_s", 1.5)

    wifi_sensor = WifiHumanSensor(
        rssi_drop_threshold_dbm=config.get("wifi_sensing", {}).get("rssi_drop_threshold_dbm", 8.0)
    )
    if args.calibrate_wifi:
        wifi_sensor.reset()

    tmap: TentativeMap | None = None
    if not args.no_map:
        tmap = TentativeMap(
            cell_size_m=map_cfg.get("cell_size_m", 0.05),
            width=map_cfg.get("width", 80),
            height=map_cfg.get("height", 40),
            output_path=root / map_file,
            trajectory_path=root / traj_file,
            robot_id=robot_id,
        )
        log.info("2D ASCII Map file       : %s", tmap.output_path)
        log.info("Movement/Trajectory Log : %s", tmap.trajectory_path)

    fps_counter = FpsCounter()
    show = ctrl_cfg.get("show_preview", True)
    last_status = ""
    last_status_time = 0.0
    last_human_mark = 0.0
    last_map_save = 0.0
    last_v, last_omega = 0.0, 0.0
    # [CMD] motion-attribution log bookkeeping (see udp_client.command_class).
    last_cmd_source = "FSM"
    last_cmd_class = "STOP"
    last_cmd_log = 0.0
    frame_idx = 0
    last_detections: list = []
    failsafe_active = False
    dashboard = TerminalDashboard(refresh_interval_s=0.35)

    log.info("READY — Press Q to quit.")

    try:
        while True:
            now = time.time()
            ok, frame = cap.read()
            if not ok or frame is None:
                time.sleep(0.05)
                continue

            fps = fps_counter.tick()

            # FIX #2: har frame par YOLO na chalao — CPU par slow hota
            # hai aur FPS gir jata hai. Har `infer_every_n` frame par
            # naya detection chalao, beech ke frames purane result reuse
            # karein (screen par thoda bhi difference nahi lagta kyunki
            # 2-3 frames ka gap ~0.1s se kam hota hai).
            frame_idx += 1
            if frame_idx % infer_every_n == 0:
                last_detections = detect_humans(frame, config)
            detections = last_detections
            cv_alert, cv_source = _cv_alert_message(detections, cv_thresholds)

            telem = udp.poll_telemetry() if udp else None
            telemetry_stale = udp is not None and udp.seconds_since_telemetry() > telemetry_timeout_s

            raw_dist_cm = int(telem.get("dist_front_cm", telem.get("dist_cm", -1))) if telem else -1
            raw_dist_left = int(telem.get("dist_left_cm", -1)) if telem else -1
            raw_dist_right = int(telem.get("dist_right_cm", -1)) if telem else -1
            wall_near = bool(telem.get("wall_near", False)) if telem else False
            sense_rssi = int(telem.get("sense_rssi", 0)) if telem else 0
            raw_yaw = float(telem.get("yaw", 0.0)) if telem else 0.0

            # TASK-01/02: clean sonar + yaw BEFORE the FSM/mapping ever
            # sees them -- invalid -1s, spikes, and yaw teleports never
            # reach the rest of the system.
            dist_cm, dist_left, dist_right, yaw = filters.apply(raw_dist_cm, raw_dist_left, raw_dist_right, raw_yaw)

            if telemetry_stale and not failsafe_active:
                failsafe_active = True
                log.warning(
                    "FAILSAFE: %.1fs se robot se telemetry nahi aayi — motors STOP",
                    udp.seconds_since_telemetry(),
                )
            elif not telemetry_stale and failsafe_active:
                failsafe_active = False
                log.info("Telemetry link wapis aa gaya — failsafe hata diya")

            wifi_hint = None
            if sense_rssi != 0:
                if args.calibrate_wifi or wifi_sensor.baseline_rssi is None:
                    wifi_sensor.calibrate(float(sense_rssi))
                else:
                    wifi_hint = wifi_sensor.update(float(sense_rssi))

            # TASK-10: WiFi is a HINT, not a brake by itself. Only escalate
            # to an actual motor-stopping alert if CV also confirms, OR
            # there have been 3+ consecutive drops AND a wall is ahead.
            wall_ahead = 0 < dist_cm <= fsm.wall_stop_cm
            wifi_alert = None
            if wifi_hint and (cv_alert or (wifi_sensor.consecutive_drops >= 3 and wall_ahead)):
                wifi_alert = wifi_hint

            # TASK-04/05/06/07: update the occupancy grid from filtered
            # sonar, then (re)plan a frontier path with A* and hand the
            # FSM a heading to steer toward during CRUISE.
            target_heading_deg = None
            if fsm.state.name == "CRUISE" and tmap is not None:
                if dist_cm > 0:
                    grid.mark_ray(tmap.x, tmap.y, yaw, dist_cm)
                if dist_left > 0:
                    grid.mark_ray(tmap.x, tmap.y, yaw + 90.0, dist_left)
                if dist_right > 0:
                    grid.mark_ray(tmap.x, tmap.y, yaw - 90.0, dist_right)
                grid.mark_visited(tmap.x, tmap.y)

                if (not astar_path_world or astar_wp_index >= len(astar_path_world)) and now >= next_replan_time:
                    next_replan_time = now + replan_interval_s
                    start_cell = grid.world_to_cell(tmap.x, tmap.y)
                    goal_xy = grid.nearest_frontier(tmap.x, tmap.y)
                    astar_path_world, astar_wp_index = [], 0
                    if goal_xy is not None:
                        goal_cell = grid.world_to_cell(*goal_xy)
                        path_cells = astar(grid, start_cell, goal_cell)
                        if path_cells:  # TASK-06: if no path, fall back to FSM scan (target stays None)
                            astar_path_world = path_to_waypoints(grid, path_cells)

                if astar_wp_index < len(astar_path_world):
                    wp = astar_path_world[astar_wp_index]
                    if math.hypot(wp[0] - tmap.x, wp[1] - tmap.y) < 0.1:
                        astar_wp_index += 1
                    if astar_wp_index < len(astar_path_world):
                        target_heading_deg = next_heading_deg((tmap.x, tmap.y), astar_path_world[astar_wp_index])

            v, omega, status = fsm.step(
                now=time.time(),
                dist_cm=dist_cm,
                dist_left=dist_left,
                dist_right=dist_right,
                wall_near=wall_near,
                yaw_deg=yaw,
                cv_alert=cv_alert,
                wifi_alert=wifi_alert,
                target_heading_deg=target_heading_deg,
            )
            # FIX: telemetry stale ho to failsafe — motors ko force stop
            # karo, chahe FSM kuch bhi decide kare. Ye robot ko purane
            # data par blindly chalte rehne se bachata hai.
            if failsafe_active:
                v, omega = 0.0, 0.0

            # TASK-08: stuck / oscillation watchdog -- can override (v, omega)
            #
            # FIX (watchdog, 2026-09), do baatein:
            #   * failsafe ke waqt check hi NAHI chalta — warna telemetry
            #     drop ke beech watchdog apni recovery spin order kar sakta
            #     tha jab motors already stopped honi chahiye thi.
            #   * commanded_v = FSM output AFTER failsafe, BEFORE watchdog.
            #     Pose-stuck window sirf tab age hoti hai jab robot ko
            #     actually translate karne ko kaha gaya ho (neeche watchdog.py).
            wd_active = False
            if tmap is not None and not failsafe_active:
                wd_active, wd_v, wd_omega, wd_msg = watchdog.check(
                    now=now, x=tmap.x, y=tmap.y,
                    wall_present=(0 < dist_cm <= fsm.wall_stop_cm),
                    fsm=fsm,
                    commanded_v=v,
                )
                if wd_active:
                    v, omega = wd_v, wd_omega
                    status = wd_msg
                    astar_path_world, astar_wp_index = [], 0  # force a replan after recovery

            # DEBUG_FORCE_FORWARD (see top of file): ignore the FSM/failsafe/
            # watchdog result entirely and drive straight. Applied AFTER all
            # of them so this single switch really does override everything.
            if DEBUG_FORCE_FORWARD:
                v, omega = DEBUG_FORWARD_V, 0.0

            # Motion attribution (diagnostics): "kaun ne wheel chalayi?"
            # Har us log line ko likho jahan command ka SOURCE ya USKA
            # DIRECTION-CLASS badla ho, taake field mein adhuri movement ka
            # reason log se hi mil jaye (DEBUG vs FSM vs FAILSAFE vs WATCHDOG).
            if DEBUG_FORCE_FORWARD:
                cmd_source = "DEBUG"
            else:
                cmd_source = "FAILSAFE" if failsafe_active else ("WATCHDOG" if wd_active else "FSM")
            cmd_class = command_class(v, omega)
            if (
                (cmd_source != last_cmd_source or cmd_class != last_cmd_class)
                and now - last_cmd_log >= 0.3
            ):
                log.info(
                    "[CMD] %-8s %-12s v=%+.3f m/s  w=%+.3f rad/s  state=%s",
                    cmd_source, cmd_class, v, omega, fsm.state.name,
                )
                last_cmd_source, last_cmd_class = cmd_source, cmd_class
                last_cmd_log = now

            last_v, last_omega = v, omega

            # --- Tentative map & MPU Trajectory update ---
            if tmap is not None:
                tmap.update_pose(
                    yaw_deg=yaw,
                    v=v,
                    omega=omega,
                    dist_front_cm=dist_cm,
                    dist_left_cm=dist_left,
                    state_name=fsm.state.name,
                    failsafe_active=failsafe_active,  # TASK-03
                )
                if dist_cm > 0:
                    tmap.mark_wall_from_ultrasonic(dist_cm, yaw)
                    if fsm.state.name in ("SCAN_ROTATE", "CREEP_TO_SCAN", "WIFI_SCAN"):
                        fsm.note_sonar(dist_cm)

                # TASK-09: unique human tracker -- a pin is only created
                # (and only logged) if it's not the same person already
                # tracked nearby. Replaces the old TIME-only cooldown,
                # which let one person be logged 18+ times.
                human_event_cls = cv_source or (wifi_alert and "wifi")
                if human_event_cls:
                    offset_m = (dist_cm / 100.0) if dist_cm > 0 else 1.2
                    yaw_rad = math.radians(yaw)
                    hx = tmap.x + offset_m * math.cos(yaw_rad)
                    hy = tmap.y + offset_m * math.sin(yaw_rad)
                    conf = 0.5
                    for d in detections:
                        if d.label in ("standing", "fallen", "person"):
                            conf = d.confidence
                            break
                    track, is_new = human_tracker.observe(human_event_cls, conf, hx, hy, now=now)
                    if is_new:
                        tmap.mark_human(human_event_cls, note=cv_alert or wifi_alert or "",
                                         override_xy=(track.x, track.y))
                        path_m, path_t = tmap.save()
                        log.info("MAP & TRAJECTORY saved -> %s & %s", path_m.name, path_t.name)
                        last_map_save = now
                elif wifi_hint:
                    # TASK-10: unconfirmed wifi hint still drawn as '?' but
                    # never stops the robot and never inflates human_count.
                    offset_m = (dist_cm / 100.0) if dist_cm > 0 else 1.0
                    yaw_rad = math.radians(yaw)
                    hx = tmap.x + offset_m * math.cos(yaw_rad)
                    hy = tmap.y + offset_m * math.sin(yaw_rad)
                    tmap.mark_human("wifi", note=wifi_hint, override_xy=(hx, hy), is_hint=True)

                # Periodic autosave every 10s
                if now - last_map_save >= 10.0:
                    tmap.save()
                    last_map_save = now

            if status and status != last_status:
                log.info(status)
                last_status = status
                last_status_time = now

            if udp and limiter and limiter.ready():
                # DEBUG_FORCE_FORWARD also forces allow_creep so the ESP32's
                # 100cm wall stop cannot halt the straight-line drive.
                allow_creep = DEBUG_FORCE_FORWARD or fsm.state.name in (
                    "CREEP_TO_SCAN", "SCAN_ROTATE", "TURN_TO_PATH", "WIFI_SCAN"
                )
                udp.send_command(v, omega, allow_creep=allow_creep)

            # Render sophisticated real-time terminal HUD
            frontier_goal = grid.nearest_frontier(tmap.x, tmap.y) if tmap is not None else None
            dashboard.render_single_robot(
                robot_id=robot_id,
                name=target_cfg.get("name", f"Robot_{robot_id}"),
                state_name=fsm.state.name,
                v=v,
                omega=omega,
                yaw=yaw,
                dist_front=dist_cm,
                dist_left=dist_left,
                dist_right=dist_right,
                dist_traveled_cm=tmap.total_distance_cm if tmap else 0.0,
                sense_rssi=sense_rssi,
                cv_alert=cv_alert,
                fps=fps,
                stream_url=cap.stream_url,
                failsafe=failsafe_active,
                is_online=not telemetry_stale,
                astar_path_len=max(0, len(astar_path_world) - astar_wp_index),
                human_count=human_tracker.count,
                frontier_goal=frontier_goal,
            )

            if show:
                vis = draw_detections(frame, detections)
                vis = draw_fps(vis, fps)
                dist_traveled = tmap.total_distance_cm if tmap else 0.0
                info1 = f"Robot #{robot_id} | State: {fsm.state.name} | MPU Yaw: {yaw:+.1f} deg"
                info2 = f"Traveled: {dist_traveled:.0f}cm | Sonar: Front={dist_cm}cm, Left={dist_left}cm"
                cv2.putText(vis, info1, (10, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 2)
                cv2.putText(vis, info2, (10, 75), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 200), 2)
                cv2.imshow(f"ARK-5 Robot #{robot_id} — Laptop Brain", vis)
                if cv2.waitKey(1) & 0xFF in (ord("q"), ord("Q")):
                    break

    except KeyboardInterrupt:
        log.info("EXIT: Stopped by user (Ctrl+C).")
    finally:
        if tmap is not None:
            path_m, path_t = tmap.save()
            log.info("Final 2D Map -> %s", path_m)
            log.info("Final MPU/Sonar Trajectory Log -> %s", path_t)
        if udp:
            udp.close()
        cap.release()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
