#!/usr/bin/env python3
"""
ARK-5 Human Detector Robot — Main Controller (Laptop Brain)

CV (YOLOv8) + WiFi sensing + Ultrasonic wall stop + MPU dead-reckoning
+ tentative ASCII map + L-turn corner scan algorithm.
"""

from __future__ import annotations

import argparse
import logging
import sys
import time
from pathlib import Path

import cv2

cv2.setNumThreads(1)

sys.path.insert(0, str(Path(__file__).resolve().parent))

from utils.autonomous_fsm import AutonomousFSM
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
from utils.wifi_sensing import WifiHumanSensor

HUMAN_MARK_COOLDOWN_S = 5.0

# FIX: pehle poore project mein sirf print() use ho raha tha — koi
# timestamp, koi log-level, koi file mein save nahi hota tha. Agar robot
# field mein crash ho jaye to pata lagana mushkil tha ke kab/kyun hua.
# Ab console + log file dono mein timestamped logs jaate hain.
def _setup_logging(root: Path) -> logging.Logger:
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
    )

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

            dist_cm = int(telem.get("dist_front_cm", telem.get("dist_cm", -1))) if telem else -1
            dist_left = int(telem.get("dist_left_cm", -1)) if telem else -1
            dist_right = int(telem.get("dist_right_cm", -1)) if telem else -1
            wall_near = bool(telem.get("wall_near", False)) if telem else False
            sense_rssi = int(telem.get("sense_rssi", 0)) if telem else 0
            yaw = float(telem.get("yaw", 0.0)) if telem else 0.0

            if telemetry_stale and not failsafe_active:
                failsafe_active = True
                log.warning(
                    "FAILSAFE: %.1fs se robot se telemetry nahi aayi — motors STOP",
                    udp.seconds_since_telemetry(),
                )
            elif not telemetry_stale and failsafe_active:
                failsafe_active = False
                log.info("Telemetry link wapis aa gaya — failsafe hata diya")

            wifi_alert = None
            if sense_rssi != 0:
                if args.calibrate_wifi or wifi_sensor.baseline_rssi is None:
                    wifi_sensor.calibrate(float(sense_rssi))
                else:
                    wifi_alert = wifi_sensor.update(float(sense_rssi))

            v, omega, status = fsm.step(
                now=time.time(),
                dist_cm=dist_cm,
                dist_left=dist_left,
                dist_right=dist_right,
                wall_near=wall_near,
                yaw_deg=yaw,
                cv_alert=cv_alert,
                wifi_alert=wifi_alert,
            )
            # FIX: telemetry stale ho to failsafe — motors ko force stop
            # karo, chahe FSM kuch bhi decide kare. Ye robot ko purane
            # data par blindly chalte rehne se bachata hai.
            if failsafe_active:
                v, omega = 0.0, 0.0
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
                )
                if dist_cm > 0:
                    tmap.mark_wall_from_ultrasonic(dist_cm, yaw)
                    if fsm.state.name in ("SCAN_ROTATE", "CREEP_TO_SCAN", "WIFI_SCAN"):
                        fsm.note_sonar(dist_cm)

                human_event = cv_source or (wifi_alert and "wifi")
                if human_event and (now - last_human_mark) >= HUMAN_MARK_COOLDOWN_S:
                    last_human_mark = now
                    if cv_source:
                        tmap.mark_human(cv_source, note=cv_alert or "")
                    elif wifi_alert:
                        offset = dist_cm / 100.0 if dist_cm > 0 else 1.0
                        tmap.mark_human("wifi", note=wifi_alert, offset_m=offset)
                    path_m, path_t = tmap.save()
                    log.info("MAP & TRAJECTORY saved -> %s & %s", path_m.name, path_t.name)
                    last_map_save = now

                # Periodic autosave every 10s
                if now - last_map_save >= 10.0:
                    tmap.save()
                    last_map_save = now

            if status and status != last_status:
                log.info(status)
                last_status = status
                last_status_time = now

            if udp and limiter and limiter.ready():
                allow_creep = fsm.state.name in (
                    "CREEP_TO_SCAN", "SCAN_ROTATE", "TURN_TO_PATH", "WIFI_SCAN"
                )
                udp.send_command(v, omega, allow_creep=allow_creep)

            # Render sophisticated real-time terminal HUD
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
