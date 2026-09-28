#!/usr/bin/env python3
"""
ARK-5 Dual-Robot Swarm Controller (Laptop Brain)

Simultaneously controls two identical robots (Robot 1 & Robot 2):
  1. Independent Execution:
     - Each robot operates completely independently.
     - If one robot fails or drops a sensor, it reports "[ROBOT X: NOT WORKING / SENSOR FAULT]"
       while the other robot continues searching, cruising, and mapping normally.
  2. Multi-Direction Area Coverage:
     - Robot 1 explores LEFT sectors (Left-biased FSM).
     - Robot 2 explores RIGHT sectors (Right-biased FSM).
  3. Mutual Anti-Collision (Clash Prevention):
     - Continuous distance check between Robot 1 (X1, Y1) and Robot 2 (X2, Y2).
     - If distance < 80cm, Robot 2 yields and turns away to prevent any physical clash.
  4. Mapping & Trajectory Logging:
     - Individual maps & logs: maps/robot1_map.txt, maps/robot1_movement_log.txt
     - Combined swarm map: maps/swarm_combined_map.txt
  5. Live Dual-Camera Dashboard:
     - Side-by-side live video streams with YOLO11 detections and sensor health indicators.
"""

from __future__ import annotations

import argparse
import logging
import math
import sys
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

import cv2
import numpy as np

cv2.setNumThreads(1)
sys.path.insert(0, str(Path(__file__).resolve().parent))

from utils.autonomous_fsm import AutonomousFSM, RobotState
from utils.config_validation import validate_config
from utils.tentative_map import TentativeMap
from utils.udp_client import RateLimiter, UdpMotorClient
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
from utils.sensor_filter import RobotSensorFilters
from utils.wifi_sensing import WifiHumanSensor
from utils.human_tracker import HumanTracker
from utils.occupancy_grid import OccupancyGrid, OCCUPIED, FREE, VISITED
from utils.watchdog import StuckWatchdog

log = logging.getLogger("ark5.swarm")

SAFE_INTER_ROBOT_DISTANCE_M = 0.80  # 80 cm safety bubble between robots


@dataclass
class RobotContext:
    robot_id: int
    name: str
    udp_host: str
    udp_port: int
    telemetry_port: int
    stream_url: str
    map_file: str
    traj_file: str
    exploration_bias: str  # "left" or "right"

    # Runtime components
    cap: Any = None
    udp: UdpMotorClient | None = None
    limiter: RateLimiter | None = None
    fsm: AutonomousFSM | None = None
    tmap: TentativeMap | None = None
    fps_counter: FpsCounter = field(default_factory=FpsCounter)

    # TASK-01/02: per-sensor median filters + yaw jump guard
    filters: RobotSensorFilters = field(default_factory=RobotSensorFilters)
    # TASK-10: WiFi RSSI is a hint source, never a brake by itself
    wifi_sensor: WifiHumanSensor = field(default_factory=WifiHumanSensor)
    # TASK-09: de-duplicated human/survivor tracker
    human_tracker: HumanTracker = field(default_factory=HumanTracker)
    # TASK-04/05/TASK-11: this robot's own occupancy grid (merged later)
    grid: OccupancyGrid = field(default_factory=OccupancyGrid)
    # TASK-08: stuck / oscillation watchdog
    watchdog: StuckWatchdog = field(default_factory=StuckWatchdog)

    # Health & Telemetry State
    is_online: bool = False
    front_sonar_healthy: bool = False
    left_sonar_healthy: bool = False
    last_telemetry_time: float = 0.0
    last_front_sonar_time: float = 0.0
    last_left_sonar_time: float = 0.0
    last_frame_time: float = 0.0

    # Current Sensor Readings (TASK-01/02: these are the FILTERED values)
    yaw: float = 0.0
    dist_front_cm: int = -1
    dist_left_cm: int = -1
    dist_right_cm: int = -1
    wall_near: bool = False
    current_x: float = 0.0
    current_y: float = 0.0

    # Detections & Video
    last_frame: np.ndarray | None = None
    last_detections: list = field(default_factory=list)
    last_status: str = ""
    status_msg: str = "INITIALIZING"
    health_warning: str = ""
    failsafe_active: bool = False
    # NOTE (bug found while wiring TASK-12): render_swarm() in
    # terminal_hud.py reads r.last_v / r.last_omega, but nothing ever
    # set them -- this crashed the HUD with AttributeError. Declaring +
    # assigning them below fixes it.
    last_v: float = 0.0
    last_omega: float = 0.0
    # TASK-10: latest raw wifi hint (before brake-worthiness gating)
    _wifi_hint: str | None = None


def setup_logging(root: Path) -> logging.Logger:
    log_dir = root / "logs"
    log_dir.mkdir(exist_ok=True)
    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s [%(levelname)s] %(name)s: %(message)s",
        handlers=[
            logging.StreamHandler(),
            logging.FileHandler(log_dir / "ark5_swarm.log", encoding="utf-8"),
        ],
    )
    return logging.getLogger("ark5.swarm")


def render_combined_swarm_map(r1: RobotContext, r2: RobotContext, out_path: Path) -> None:
    """Combines paths, positions, and walls of both robots into a unified map."""
    width, height = 90, 45
    cell_size_m = 0.05
    cx, cy = width // 2, height - 4

    grid = [[" " for _ in range(width)] for _ in range(height)]

    def to_grid(x_m: float, y_m: float) -> tuple[int, int]:
        gx = int(round(x_m / cell_size_m)) + cx
        gy = cy - int(round(y_m / cell_size_m))
        return max(0, min(width - 1, gx)), max(0, min(height - 1, gy))

    # Mark Start positions
    gx_s, gy_s = to_grid(0.0, 0.0)
    grid[gy_s][gx_s] = "S"

    # Overlay Robot 1 cells
    if r1.tmap:
        for (gx, gy), ch in r1.tmap._cells.items():
            if 0 <= gx < width and 0 <= gy < height:
                grid[gy][gx] = "1" if ch == "." else ch

    # Overlay Robot 2 cells
    if r2.tmap:
        for (gx, gy), ch in r2.tmap._cells.items():
            if 0 <= gx < width and 0 <= gy < height:
                if grid[gy][gx] == "1":
                    grid[gy][gx] = "+"  # overlap path
                elif grid[gy][gx] == " ":
                    grid[gy][gx] = "2" if ch == "." else ch

    # Robot current positions
    if r1.is_online and r1.tmap:
        gx1, gy1 = to_grid(r1.current_x, r1.current_y)
        grid[gy1][gx1] = "A"  # Robot Alpha (1)
    if r2.is_online and r2.tmap:
        gx2, gy2 = to_grid(r2.current_x, r2.current_y)
        grid[gy2][gx2] = "B"  # Robot Beta (2)

    lines = [
        "==========================================================================================",
        "  ARK-5 COMBINED SWARM EXPLORATION MAP (ROBOT 1 & ROBOT 2)",
        "  Legend: S=Start  1=Robot 1 Path  2=Robot 2 Path  +=Shared Path  #=Wall  *=Human",
        "          A=Robot 1 Pos (@Alpha)  B=Robot 2 Pos (@Beta)",
        f"  Robot 1: Dist={r1.tmap.total_distance_cm if r1.tmap else 0:.0f}cm | Yaw={r1.yaw:+.0f}° | Health={r1.status_msg}",
        f"  Robot 2: Dist={r2.tmap.total_distance_cm if r2.tmap else 0:.0f}cm | Yaw={r2.yaw:+.0f}° | Health={r2.status_msg}",
        f"  Updated: {time.strftime('%Y-%m-%d %H:%M:%S')}",
        "==========================================================================================",
        "",
    ]
    lines.extend("".join(row).rstrip() for row in grid)
    out_path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def merge_occupancy_grids(r1: RobotContext, r2: RobotContext) -> OccupancyGrid:
    """TASK-11: merge both robots' occupancy grids (TASK-04/05) into one
    shared planner grid. OCCUPIED always wins (a wall seen by either
    robot is a wall); VISITED wins over plain FREE."""
    merged = OccupancyGrid(cell_size_m=r1.grid.cell_size_m,
                            width=max(r1.grid.width, r2.grid.width),
                            height=max(r1.grid.height, r2.grid.height))
    _PRECEDENCE = {OCCUPIED: 3, VISITED: 2, FREE: 1}
    for src in (r1.grid, r2.grid):
        for cell, state in src._cells.items():
            current = merged._cells.get(cell)
            if current is None or _PRECEDENCE.get(state, 0) > _PRECEDENCE.get(current, 0):
                merged._cells[cell] = state
    return merged


def save_merged_occupancy(r1: RobotContext, r2: RobotContext, out_path: Path) -> None:
    merged = merge_occupancy_grids(r1, r2)
    # Render centered between both robots' current positions.
    cx = (r1.current_x + r2.current_x) / 2.0
    cy = (r1.current_y + r2.current_y) / 2.0
    merged.save(out_path, robot_x_m=cx, robot_y_m=cy)


def create_placeholder_frame(robot: RobotContext, text: str, subtext: str = "") -> np.ndarray:
    """Creates a clean status frame when camera is not connected."""
    frame = np.zeros((480, 640, 3), dtype=np.uint8)
    cv2.rectangle(frame, (10, 10), (630, 470), (40, 40, 40), 2)
    cv2.putText(frame, f"ROBOT #{robot.robot_id} ({robot.name})", (30, 60),
                cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 200, 255), 2)
    cv2.putText(frame, text, (30, 200), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
    if subtext:
        cv2.putText(frame, subtext, (30, 240), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (200, 200, 200), 1)

    # Show last known telemetry
    info = f"Last Yaw: {robot.yaw:+.1f} deg | Sonar Front: {robot.dist_front_cm}cm, Left: {robot.dist_left_cm}cm"
    cv2.putText(frame, info, (30, 320), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (160, 160, 160), 1)
    cv2.putText(frame, f"Stream Target: {robot.stream_url}", (30, 350),
                cv2.FONT_HERSHEY_SIMPLEX, 0.45, (120, 120, 120), 1)
    return frame


def cv_alert_message(detections, thresholds: dict) -> tuple[str | None, str | None]:
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
    parser = argparse.ArgumentParser(description="ARK-5 Dual-Robot Swarm Controller")
    parser.add_argument("--config", default=None, help="Path to project.yaml")
    parser.add_argument("--no-motors", action="store_true", help="Vision only, no motor commands")
    parser.add_argument("--no-map", action="store_true", help="Disable map files")
    args = parser.parse_args()

    config = load_config(Path(args.config) if args.config else None)
    root = project_root()
    global log
    log = setup_logging(root)

    validate_config(config)

    log.info("=" * 65)
    log.info("  ARK-5 SWARM CONTROLLER — DUAL ROBOT INDEPENDENT BRAIN")
    log.info("  Robot 1 (Alpha): Left-Sector Exploration")
    log.info("  Robot 2 (Beta) : Right-Sector Exploration")
    log.info("=" * 65)

    warmup_yolo(config)

    robot_cfg = config["robot"]
    ctrl_cfg = config.get("control", {})
    map_cfg = config.get("map", {})
    yolo_cfg = config.get("yolo", {})
    robots_cfg = config.get("robots", {})

    cv_thresholds = {
        "fallen": yolo_cfg.get("fallen_conf", 0.4),
        "standing": yolo_cfg.get("standing_conf", 0.5),
        "person": yolo_cfg.get("person_conf", 0.55),
    }
    infer_every_n = max(1, int(yolo_cfg.get("infer_every_n_frames", 2)))
    telemetry_timeout_s = float(robot_cfg.get("telemetry_timeout_s", 2.0))

    # ── Initialize Robot 1 & Robot 2 Contexts ──────────────────────────────
    r1_cfg = robots_cfg.get(1, {})
    r2_cfg = robots_cfg.get(2, {})

    r1 = RobotContext(
        robot_id=1,
        name=r1_cfg.get("name", "Robot_Alpha"),
        udp_host=r1_cfg.get("udp_host", "10.147.103.106"),
        udp_port=r1_cfg.get("udp_port", 4210),
        telemetry_port=r1_cfg.get("telemetry_port", 4211),
        stream_url=r1_cfg.get("stream_url", config["camera"]["stream_url"]),
        map_file=r1_cfg.get("map_file", "maps/robot1_map.txt"),
        traj_file=r1_cfg.get("trajectory_log", "maps/robot1_movement_log.txt"),
        exploration_bias="left",  # Robot 1 explores LEFT
    )

    r2 = RobotContext(
        robot_id=2,
        name=r2_cfg.get("name", "Robot_Beta"),
        udp_host=r2_cfg.get("udp_host", "10.147.103.107"),
        udp_port=r2_cfg.get("udp_port", 4220),
        telemetry_port=r2_cfg.get("telemetry_port", 4221),
        stream_url=r2_cfg.get("stream_url", "http://10.147.103.129:81/stream"),
        map_file=r2_cfg.get("map_file", "maps/robot2_map.txt"),
        traj_file=r2_cfg.get("trajectory_log", "maps/robot2_movement_log.txt"),
        exploration_bias="right",  # Robot 2 explores RIGHT
    )

    robots = [r1, r2]

    # Initialize cameras and components
    for r in robots:
        log.info("[%s] Connecting Camera: %s", r.name, r.stream_url)
        try:
            r.cap = open_stream(str(r.stream_url), robot_id=r.robot_id)
            log.info("[%s] Camera stream connection initiated (%s)", r.name, r.cap.stream_url)
        except Exception as e:
            log.warning("[%s] Camera not available yet: %s", r.name, e)

        if not args.no_motors:
            r.udp = UdpMotorClient(host=r.udp_host, port=r.udp_port, telemetry_port=r.telemetry_port)
            r.limiter = RateLimiter(robot_cfg.get("command_rate_hz", 20))

        r.fsm = AutonomousFSM(
            v_cruise=ctrl_cfg.get("v_cruise", 0.15),
            v_creep=ctrl_cfg.get("v_creep", 0.06),
            omega_scan=ctrl_cfg.get("omega_scan", 0.35),
            wall_stop_cm=robot_cfg.get("wall_stop_cm", 100),
            scan_standoff_min_cm=ctrl_cfg.get("scan_standoff_min_cm", 30),
            scan_standoff_max_cm=ctrl_cfg.get("scan_standoff_max_cm", 40),
            open_path_cm=ctrl_cfg.get("open_path_cm", 120),
            scan_step_deg=ctrl_cfg.get("scan_step_deg", 15),
            scan_side_deg=ctrl_cfg.get("scan_side_deg", 90),
            alert_pause_s=ctrl_cfg.get("alert_pause_s", 2.0),
            exploration_bias=r.exploration_bias,
            align_tolerance_deg=ctrl_cfg.get("align_tolerance_deg", 4.0),
            creep_timeout_s=ctrl_cfg.get("creep_timeout_s", 3.0),
            center_gain=ctrl_cfg.get("center_gain", 0.0035),
            center_omega_max=ctrl_cfg.get("center_omega_max", 0.15),
            path_gain=ctrl_cfg.get("path_gain", 0.02),
            path_omega_max=ctrl_cfg.get("path_omega_max", 0.25),
        )

        if not args.no_map:
            r.tmap = TentativeMap(
                cell_size_m=map_cfg.get("cell_size_m", 0.05),
                width=map_cfg.get("width", 80),
                height=map_cfg.get("height", 40),
                output_path=root / r.map_file,
                trajectory_path=root / r.traj_file,
                robot_id=r.robot_id,
            )

    combined_map_path = root / "maps/swarm_combined_map.txt"
    combined_occupancy_path = root / "maps/swarm_combined_occupancy.txt"  # TASK-11
    last_swarm_map_save = 0.0
    frame_idx = 0
    dashboard = TerminalDashboard(refresh_interval_s=0.4)

    log.info("SWARM READY — Press 'Q' on the video window to stop.")

    try:
        while True:
            now = time.time()
            frame_idx += 1

            # ── 1. Telemetry & Sensor Health Monitoring ─────────────────────
            for r in robots:
                telem = r.udp.poll_telemetry() if r.udp else None
                if telem:
                    r.last_telemetry_time = now
                    r.is_online = True
                    raw_yaw = float(telem.get("yaw", r.yaw))
                    raw_f = int(telem.get("dist_front_cm", telem.get("dist_cm", -1)))
                    raw_l = int(telem.get("dist_left_cm", -1))
                    # TASK-11: read the REAL right sonar from telemetry --
                    # it was previously hardcoded to -1 below, meaning
                    # Robot 2 could never see or take a right opening.
                    raw_r = int(telem.get("dist_right_cm", -1))

                    # TASK-01/02: median-filter sonars + yaw-jump-guard
                    # BEFORE anything else (FSM, mapping, HUD) sees them.
                    f_cm, l_cm, r_cm, filt_yaw = r.filters.apply(raw_f, raw_l, raw_r, raw_yaw)
                    r.yaw = filt_yaw
                    r.dist_front_cm = f_cm
                    r.dist_left_cm = l_cm
                    r.dist_right_cm = r_cm
                    r.wall_near = bool(telem.get("wall_near", False))

                    if f_cm > 0:
                        r.last_front_sonar_time = now
                        r.front_sonar_healthy = True
                    if l_cm > 0:
                        r.last_left_sonar_time = now
                        r.left_sonar_healthy = True

                    # TASK-11: enable WiFi sensing like single-robot mode
                    # (previously WiFi was always disabled in swarm mode).
                    sense_rssi = telem.get("sense_rssi", 0)
                    if sense_rssi:
                        if r.wifi_sensor.baseline_rssi is None:
                            r.wifi_sensor.calibrate(float(sense_rssi))
                        else:
                            r._wifi_hint = r.wifi_sensor.update(float(sense_rssi))
                    else:
                        r._wifi_hint = None

                # Evaluate Health Status
                telem_age = now - r.last_telemetry_time
                if telem_age > telemetry_timeout_s:
                    if r.is_online:
                        r.is_online = False
                        log.warning("[%s] OFFLINE: Telemetry not reaching (%.1fs elapsed)", r.name, telem_age)
                    r.status_msg = "NOT WORKING / OFFLINE"
                    r.failsafe_active = True
                else:
                    r.failsafe_active = False
                    # Check individual sensors
                    warnings = []
                    if (now - r.last_front_sonar_time) > 3.0:
                        warnings.append("Front Sonar Missing")
                    if (now - r.last_left_sonar_time) > 3.0:
                        warnings.append("Left Sonar Missing")
                    r.health_warning = ", ".join(warnings)
                    r.status_msg = f"WARNING ({r.health_warning})" if warnings else "ACTIVE (All Sensors OK)"

            # ── 2. Mutual Anti-Collision (Clash Prevention) ────────────────
            inter_robot_dist = math.hypot(r1.current_x - r2.current_x, r1.current_y - r2.current_y)
            collision_warning = False
            r2_must_yield = False

            if r1.is_online and r2.is_online and inter_robot_dist < SAFE_INTER_ROBOT_DISTANCE_M:
                collision_warning = True
                r2_must_yield = True  # Robot 2 yields to Robot 1
                log.warning("[ANTI-COLLISION] Robots close (%.2fm)! Robot 2 yielding.", inter_robot_dist)

            # ── 3. Vision, FSM, Navigation & Mapping per Robot ──────────────
            rendered_frames = []

            for r in robots:
                # Read Camera
                ok, frame = (False, None)
                if r.cap:
                    try:
                        ok, frame = r.cap.read()
                    except Exception:
                        ok, frame = False, None

                if ok and frame is not None:
                    r.last_frame_time = now
                    r.last_frame = frame
                    fps = r.fps_counter.tick()

                    # YOLO Human Detection (every N frames)
                    if frame_idx % infer_every_n == 0:
                        r.last_detections = detect_humans(frame, config)
                    dets = r.last_detections
                    cv_alert, cv_source = cv_alert_message(dets, cv_thresholds)

                    # TASK-10: WiFi is a HINT, not a brake. Only escalate to
                    # an actual motor-stopping alert if CV also confirms, OR
                    # there have been 3+ consecutive drops AND a wall ahead.
                    wall_ahead = 0 < r.dist_front_cm <= r.fsm.wall_stop_cm
                    wifi_brake = None
                    if r._wifi_hint:
                        if cv_alert or (r.wifi_sensor.consecutive_drops >= 3 and wall_ahead):
                            wifi_brake = r._wifi_hint

                    # Autonomous FSM Decision
                    peer_close = (r.robot_id == 2 and r2_must_yield)
                    v, omega, status = r.fsm.step(
                        now=now,
                        dist_cm=r.dist_front_cm,
                        dist_left=r.dist_left_cm,
                        dist_right=r.dist_right_cm,  # TASK-11: real value, not hardcoded -1
                        wall_near=r.wall_near,
                        yaw_deg=r.yaw,
                        cv_alert=cv_alert,
                        wifi_alert=wifi_brake,
                        peer_too_close=peer_close,
                    )

                    # TASK-11: don't let a yielding robot wait forever --
                    # once the FSM reports a yield timeout, force it back
                    # to CRUISE so it replans its own route instead of
                    # freezing next to its sibling robot indefinitely.
                    if status and "Yield timeout" in status:
                        r.fsm.force_state(RobotState.CRUISE, now)
                        v, omega = r.fsm.v_cruise, 0.0

                    # TASK-08: stuck / oscillation watchdog
                    wd_active, wd_v, wd_omega, wd_msg = r.watchdog.check(
                        now=now, x=r.current_x, y=r.current_y,
                        wall_present=(0 < r.dist_front_cm <= r.fsm.wall_stop_cm),
                        fsm=r.fsm, exploration_bias=r.exploration_bias,
                    )
                    if wd_active:
                        v, omega = wd_v, wd_omega
                        status = wd_msg

                    # Failsafe: If robot offline, force 0 velocity
                    if r.failsafe_active:
                        v, omega = 0.0, 0.0

                    r.last_v, r.last_omega = v, omega  # bugfix: HUD needs these

                    # Send motor command via UDP
                    if r.udp and r.limiter and r.limiter.ready():
                        allow_creep = r.fsm.state.name in (
                            "CREEP_TO_SCAN", "SCAN_ROTATE", "TURN_TO_PATH", "WIFI_SCAN"
                        )
                        r.udp.send_command(v, omega, allow_creep=allow_creep)

                    # Update Tentative Map & Trajectory Log
                    if r.tmap is not None:
                        r.tmap.update_pose(
                            yaw_deg=r.yaw,
                            v=v,
                            omega=omega,
                            dist_front_cm=r.dist_front_cm,
                            dist_left_cm=r.dist_left_cm,
                            state_name=r.fsm.state.name,
                            failsafe_active=r.failsafe_active,  # TASK-03
                        )
                        r.current_x = r.tmap.x
                        r.current_y = r.tmap.y
                        r.grid.mark_visited(r.current_x, r.current_y)  # TASK-05

                        if r.dist_front_cm > 0:
                            r.tmap.mark_wall_from_ultrasonic(r.dist_front_cm, r.yaw)
                            r.grid.mark_ray(r.current_x, r.current_y, r.yaw, r.dist_front_cm)  # TASK-04
                            if r.fsm.state.name in ("SCAN_ROTATE", "CREEP_TO_SCAN"):
                                r.fsm.note_sonar(r.dist_front_cm)
                        if r.dist_left_cm > 0:
                            r.grid.mark_ray(r.current_x, r.current_y, r.yaw + 90.0, r.dist_left_cm)
                        if r.dist_right_cm > 0:
                            r.grid.mark_ray(r.current_x, r.current_y, r.yaw - 90.0, r.dist_right_cm)

                        # TASK-09: unique human tracker -- only draw a NEW
                        # pin (and only count it) if it isn't the same
                        # person already logged nearby.
                        human_event_cls = cv_source or (wifi_brake and "wifi")
                        if human_event_cls:
                            offset_m = (r.dist_front_cm / 100.0) if r.dist_front_cm > 0 else 1.2
                            yaw_rad = math.radians(r.yaw)
                            hx = r.current_x + offset_m * math.cos(yaw_rad)
                            hy = r.current_y + offset_m * math.sin(yaw_rad)
                            conf = 0.5
                            for d in dets:
                                if d.label in ("standing", "fallen", "person"):
                                    conf = d.confidence
                                    break
                            track, is_new = r.human_tracker.observe(human_event_cls, conf, hx, hy, now=now)
                            if is_new:
                                r.tmap.mark_human(human_event_cls, note=cv_alert or wifi_brake or "",
                                                   override_xy=(track.x, track.y))
                                r.tmap.save()
                        elif r._wifi_hint:
                            # TASK-10: an unconfirmed wifi hint still shows
                            # on the map as '?' but never stops the robot
                            # and never inflates human_count.
                            offset_m = (r.dist_front_cm / 100.0) if r.dist_front_cm > 0 else 1.0
                            yaw_rad = math.radians(r.yaw)
                            hx = r.current_x + offset_m * math.cos(yaw_rad)
                            hy = r.current_y + offset_m * math.sin(yaw_rad)
                            r.tmap.mark_human("wifi", note=r._wifi_hint, override_xy=(hx, hy), is_hint=True)

                    # Render UI on Frame
                    vis = draw_detections(frame, dets)
                    vis = draw_fps(vis, fps)

                    # On-screen dashboard banner
                    h_color = (0, 255, 0) if "ACTIVE" in r.status_msg else (0, 0, 255)
                    cv2.rectangle(vis, (0, 0), (640, 70), (20, 20, 20), -1)
                    cv2.putText(vis, f"ROBOT #{r.robot_id} ({r.name}) [{r.exploration_bias.upper()} BIAS]",
                                (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
                    cv2.putText(vis, f"Status: {r.status_msg}", (10, 48),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, h_color, 1)

                    dist_cm = r.tmap.total_distance_cm if r.tmap else 0.0
                    sub_info = f"Yaw:{r.yaw:+.0f}° | Moved:{dist_cm:.0f}cm | F:{r.dist_front_cm}cm, L:{r.dist_left_cm}cm | {r.fsm.state.name}"
                    cv2.putText(vis, sub_info, (10, 68), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (220, 220, 220), 1)

                    if r.robot_id == 2 and r2_must_yield:
                        cv2.putText(vis, "*** ANTI-COLLISION: YIELDING TO ROBOT 1 ***", (50, 240),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 140, 255), 2)

                    rendered_frames.append(vis)

                else:
                    # Camera Stream Failed / Offline Card
                    err_sub = "Connecting to camera..." if (now - r.last_frame_time < 5.0) else "Check WiFi / Power"
                    placeholder = create_placeholder_frame(r, f"STATUS: {r.status_msg}", err_sub)
                    rendered_frames.append(placeholder)

            # ── 4. Periodic Autosaves (Maps & Trajectories) ─────────────────
            if now - last_swarm_map_save >= 5.0:
                last_swarm_map_save = now
                for r in robots:
                    if r.tmap:
                        r.tmap.save()
                render_combined_swarm_map(r1, r2, combined_map_path)
                save_merged_occupancy(r1, r2, combined_occupancy_path)  # TASK-11

            # ── 5. Render Sophisticated Terminal Telemetry HUD ─────────────
            dashboard.render_swarm(r1, r2, inter_robot_dist, collision_warning)

            # ── 6. Show Dual-Robot Side-by-Side Window ─────────────────────
            if len(rendered_frames) == 2:
                dual_view = np.hstack([rendered_frames[0], rendered_frames[1]])
                # Central divider line
                cv2.line(dual_view, (640, 0), (640, 480), (80, 80, 80), 2)

                if collision_warning:
                    cv2.rectangle(dual_view, (400, 440), (880, 475), (0, 0, 180), -1)
                    cv2.putText(dual_view, f"SWARM PROXIMITY ALERT ({inter_robot_dist:.2f}m)!", (420, 465),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)

                cv2.imshow("ARK-5 Dual-Robot Swarm Dashboard (Alpha & Beta)", dual_view)
                key = cv2.waitKey(1) & 0xFF
                if key in (ord("q"), ord("Q")):
                    log.info("Quit command received.")
                    break

    except KeyboardInterrupt:
        log.info("Interrupted by user (Ctrl+C).")
    finally:
        log.info("Shutting down swarm controller...")
        for r in robots:
            if r.tmap:
                r.tmap.save()
            if r.udp:
                r.udp.send_command(0.0, 0.0)
                r.udp.close()
            if r.cap:
                try:
                    r.cap.release()
                except Exception:
                    pass
        render_combined_swarm_map(r1, r2, combined_map_path)
        save_merged_occupancy(r1, r2, combined_occupancy_path)  # TASK-11
        cv2.destroyAllWindows()
        log.info("Swarm controller exited cleanly.")


if __name__ == "__main__":
    main()
