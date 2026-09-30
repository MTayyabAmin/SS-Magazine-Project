"""ARK-5 navigation task tests -- TASK-01 .. TASK-12.

Run from the human_detector_robot/ folder:
    python tests/test_navigation_tasks.py

Pure Python, no hardware, no camera, no ESP32. Each test prints PASS/FAIL
and the script exits non-zero if anything fails (handy for the viva).
"""

from __future__ import annotations

import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))

from utils.astar import astar, next_heading_deg, path_to_waypoints
from utils.autonomous_fsm import AutonomousFSM, RobotState
from utils.human_tracker import HumanTracker
from utils.occupancy_grid import FREE, OCCUPIED, VISITED, OccupancyGrid
from utils.sensor_filter import SonarMedianFilter, YawJumpGuard
from utils.tentative_map import TentativeMap
from utils.watchdog import StuckWatchdog
from utils.wifi_sensing import WifiHumanSensor

_results: list[tuple[str, bool]] = []


def check(name: str, cond: bool) -> None:
    _results.append((name, bool(cond)))
    print(f"[{'PASS' if cond else 'FAIL'}] {name}")


# ---------------- D1: Pose & Sensor Clean ----------------
def test_task01_sonar_filter() -> None:
    f = SonarMedianFilter(window=5, max_jump_cm=80)
    out = [f.update(v) for v in [45, 46, -1, 44, 2, 47, 400, 45]]
    check("TASK-01 raw -1 never reported as a wall/opening", all(o > 0 for o in out))
    check("TASK-01 400cm spike rejected", max(out) < 60)
    check("TASK-01 2cm glitch never reported", 2 not in out)


def test_task02_yaw_guard() -> None:
    g = YawJumpGuard(max_deg_per_s=90.0)
    y1 = g.update(0.0, now=0.0)
    y2 = g.update(-173.0, now=1.0)  # the exact teleport seen in the log
    step = abs((y2 - y1 + 180) % 360 - 180)
    check("TASK-02 yaw 0 -> -173 in 1s capped to <= ~90deg", step <= 90.5)


def test_task03_honest_pose() -> None:
    tm = TentativeMap(output_path=Path("/tmp/_t_map.txt"), trajectory_path=Path("/tmp/_t_log.txt"))
    tm._last_t = time.time() - 0.1
    tm.update_pose(10.0, 0.0, 0.0, dist_front_cm=50, state_name="ALERT_PAUSE")
    x0, y0 = tm.x, tm.y
    time.sleep(0.05)
    tm.update_pose(10.0, 0.0, 0.0, dist_front_cm=50, state_name="ALERT_PAUSE")
    check("TASK-03 pose frozen when v=0 (ALERT_PAUSE)", (tm.x, tm.y) == (x0, y0))

    tm._last_dist_front_cm = 50
    tm._last_t = time.time() - 0.1
    tm.update_pose(10.0, 0.15, 0.0, dist_front_cm=65, state_name="CRUISE")  # dist grew while "moving forward"
    check("TASK-03 pose frozen when sonar disagrees with motion", tm.frozen_last_tick)

    tm._last_dist_front_cm = 65
    tm._last_t = time.time() - 0.1
    xb = tm.x
    tm.update_pose(0.0, 0.15, 0.0, dist_front_cm=48, state_name="CRUISE")
    check("TASK-03 pose advances when sonar confirms motion", tm.x > xb and not tm.frozen_last_tick)

    tm._last_dist_front_cm = 48
    tm._last_t = time.time() - 0.1
    xb = tm.x
    tm.update_pose(0.0, 0.15, 0.0, dist_front_cm=40, state_name="CRUISE", failsafe_active=True)
    check("TASK-03 pose frozen when failsafe active", tm.x == xb)


# ---------------- D2: Occupancy grid ----------------
def test_task04_05_grid() -> None:
    g = OccupancyGrid(cell_size_m=0.05, width=60, height=60)
    n0 = len(g._cells)
    g.mark_ray(0, 0, 0, 5)
    g.mark_ray(0, 0, 0, 350)
    check("TASK-04 readings outside 15-300cm ignored", len(g._cells) == n0)
    g.mark_ray(0.0, 0.0, 0.0, 100)
    check("TASK-04 wall marked OCCUPIED at sonar range", g.get(*g.world_to_cell(1.0, 0.0)) == OCCUPIED)
    check("TASK-04 cells along ray marked FREE", g.get(*g.world_to_cell(0.5, 0.0)) == FREE)
    check("TASK-05 frontiers detected next to free space", len(g.frontiers()) > 0)
    check("TASK-05 nearest frontier returned", g.nearest_frontier(0.0, 0.0) is not None)
    g.mark_visited(0.0, 0.0)
    check("TASK-05 robot cell marked VISITED", g.get(*g.world_to_cell(0.0, 0.0)) == VISITED)


# ---------------- D3: A* ----------------
def test_task06_astar() -> None:
    g = OccupancyGrid(cell_size_m=0.05, width=40, height=40)
    for i in range(-10, 11):
        if i != 0:
            g._set(20 + i, 15, OCCUPIED)  # wall with a gap at i == 0
    path = astar(g, (20, 25), (20, 5))
    check("TASK-06 A* finds path through the gap", path is not None and path[0] == (20, 25) and path[-1] == (20, 5))
    g2 = OccupancyGrid(cell_size_m=0.05, width=40, height=40)
    for cx in range(40):
        g2._set(cx, 15, OCCUPIED)
    check("TASK-06 no path -> returns None (FSM scan fallback)", astar(g2, (20, 25), (20, 5)) is None)
    wps = path_to_waypoints(g, path)
    check("TASK-06 waypoints produced", len(wps) == len(path))
    check("TASK-06 heading helper (0deg = +X)", abs(next_heading_deg((0, 0), (1, 0))) < 1e-6)


# ---------------- D4: FSM + watchdog ----------------
def test_task07_fsm() -> None:
    check("TASK-07 align tolerance is 4deg", AutonomousFSM().align_tolerance_deg == 4.0)

    _, omega, _ = AutonomousFSM().step(0.0, 200, 150, 50, False, 0.0, None, None)
    check("TASK-07 corridor centering steers away from closer wall", omega < 0)

    _, omega, _ = AutonomousFSM().step(0.0, 200, -1, -1, False, 0.0, None, None, target_heading_deg=30.0)
    check("TASK-07 A* heading blended into cruise omega", omega > 0)

    f = AutonomousFSM(open_path_cm=120)
    f._enter(RobotState.WIFI_SCAN, 0.0)
    _, _, msg = f.step(1.0, 35, 130, 250, False, 0.0, None, None)
    check("TASK-07 WIFI_SCAN checks BOTH sides (picks wider RIGHT)", msg is not None and "RIGHT" in msg)

    f = AutonomousFSM(creep_timeout_s=3.0)
    f._enter(RobotState.CREEP_TO_SCAN, 0.0)
    f.step(3.1, 55, -1, -1, False, 0.0, None, None)
    check("TASK-07 creep aborts after timeout", f.state == RobotState.WIFI_SCAN)

    f = AutonomousFSM(scan_side_deg=90, scan_step_deg=15)
    f._enter(RobotState.SCAN_ROTATE, 0.0)
    f._scan_center_yaw = f._scan_last_sample_yaw = 0.0
    f._scan_phase = "left"
    f.step(0.1, 999, -1, -1, False, 20.0, None, None)
    check("TASK-07 SCAN_ROTATE has no early-exit", f.state == RobotState.SCAN_ROTATE)

    # FIX (alert re-arm): ek sighting = EK hi pause. Pehle yeh block har tick
    # par chalta tha, is liye human frame mein rehne par ~72 pause/creep
    # cycles bante the (har cycle ke beech ek v_creep pulse = wheel twitch).
    f = AutonomousFSM(alert_pause_s=2.0, alert_rearm_s=3.0)
    _, _, msg = f.step(0.0, 300, -1, -1, False, 0.0, "[CV] PERSON", None)
    entered = f.state == RobotState.ALERT_PAUSE and msg is not None
    f.step(2.1, 300, -1, -1, False, 0.0, "[CV] PERSON", None)
    exited = f.state != RobotState.ALERT_PAUSE
    f.step(5.0, 300, -1, -1, False, 0.0, "[CV] PERSON", None)
    held = f.state != RobotState.ALERT_PAUSE
    f.step(5.1, 300, -1, -1, False, 0.0, None, None)
    f.step(8.2, 300, -1, -1, False, 0.0, None, None)
    f.step(8.3, 300, -1, -1, False, 0.0, "[CV] PERSON", None)
    rearmed = f.state == RobotState.ALERT_PAUSE
    check("TASK-07 alert pauses ONCE while human stays in frame",
          entered and exited and held)
    check("TASK-07 alert re-arms only after alert_rearm_s of clear", rearmed)


def test_task08_watchdog() -> None:
    # FIX (watchdog pose gate): the pose window only ages while the caller
    # is actually commanding translation, so every call below passes
    # commanded_v explicitly (it defaults to 0.0 = "deliberate hold").
    wd, fsm = StuckWatchdog(pose_stuck_timeout_s=4.0), AutonomousFSM()
    wd.check(0.0, 1.0, 1.0, True, fsm, commanded_v=0.15)
    active, _, omega, _ = wd.check(4.5, 1.001, 1.0, True, fsm, commanded_v=0.15)
    check("TASK-08 pose-stuck near wall forces a turn", active and omega != 0.0)

    wd2, fsm2 = StuckWatchdog(state_timeout_s=8.0, pose_stuck_timeout_s=999), AutonomousFSM()
    fsm2._enter(RobotState.CREEP_TO_SCAN, 0.0)
    active, *_ = wd2.check(8.5, 0.0, 0.0, False, fsm2)
    check("TASK-08 same state > 8s resets to CRUISE", active and fsm2.state == RobotState.CRUISE)

    wd3, fsm3 = StuckWatchdog(pose_stuck_timeout_s=4.0), AutonomousFSM()
    wd3.check(0.0, 0.0, 0.0, True, fsm3, commanded_v=0.15)
    active, *_ = wd3.check(4.5, 0.5, 0.0, True, fsm3, commanded_v=0.15)
    check("TASK-08 moving robot does not false-trigger", not active)

    # THE BUG: a deliberately held robot (ALERT_PAUSE / scan / failsafe)
    # must never be judged "stuck", no matter how long the pose freezes.
    wd4, fsm4 = StuckWatchdog(pose_stuck_timeout_s=4.0, state_timeout_s=999), AutonomousFSM()
    fsm4._enter(RobotState.ALERT_PAUSE, 0.0)
    wd4.check(0.0, 1.0, 1.0, True, fsm4, commanded_v=0.0)
    active, *_ = wd4.check(100.0, 1.0, 1.0, True, fsm4, commanded_v=0.0)
    check("TASK-08 idle robot (commanded_v=0) never pose-triggers", not active)

    # ...and the window must not carry over from the idle period: after the
    # hold ends, a fresh 4s of commanded motion is required.
    active, *_ = wd4.check(100.5, 1.0, 1.0, True, fsm4, commanded_v=0.15)
    check("TASK-08 window restarts when motion resumes", not active)

    src = (ROOT / "main_controller.py").read_text(encoding="utf-8")
    check("TASK-08 controller hands commanded_v to the watchdog", "commanded_v=v" in src)
    check("TASK-08 watchdog never runs while failsafe is active",
          "not failsafe_active" in src)


# ---------------- Debug harness safety ----------------
def test_debug_force_forward_flag() -> None:
    """DEBUG_FORCE_FORWARD must ship DISABLED and stay at the top of the file.

    The flag bypasses the failsafe and the wall stop, so committing it as
    True would make the next run drive blind — this guards against that.
    """
    src = (ROOT / "main_controller.py").read_text(encoding="utf-8")
    flag_def = "DEBUG_FORCE_FORWARD: bool = False"
    pos = src.find(flag_def)
    check("DEBUG-FLAG defined and ships as False", pos != -1)
    check("DEBUG-FLAG declared at the top (before any def)",
          pos != -1 and pos < src.find("def "))
    check("DEBUG-FLAG actually forces the outgoing command", "DEBUG_FORWARD_V, 0.0" in src)


# ---------------- D5: rescue + swarm ----------------
def test_task09_tracker() -> None:
    ht = HumanTracker(merge_radius_m=0.8)
    for i in range(18):
        ht.observe("cv_standing", 0.6, 1.0 + i * 0.01, 2.0, now=float(i))
    check("TASK-09 18 detections of one person -> 1 pin", ht.count == 1)
    _, is_new = ht.observe("cv_fallen", 0.5, 4.0, 2.0)
    check("TASK-09 person 3m away -> new pin", is_new and ht.count == 2)
    ht.observe("wifi", 0.4, 1.05, 2.0)
    check("TASK-09 wifi never downgrades standing", ht.all_tracks()[0].cls == "cv_standing")
    ht2 = HumanTracker()
    ht2.observe("wifi", 0.3, 0.0, 0.0)
    ht2.observe("cv_fallen", 0.7, 0.1, 0.0)
    check("TASK-09 cv confirmation upgrades wifi-only guess", ht2.all_tracks()[0].cls == "cv_fallen")
    t = ht.all_tracks()[0]
    check("TASK-09 track stores id/class/conf/x/y/timestamp",
          all(hasattr(t, a) for a in ("id", "cls", "conf", "x", "y", "timestamp")))


def test_task10_wifi_hint() -> None:
    w = WifiHumanSensor(rssi_drop_threshold_dbm=8.0)
    w.calibrate(-50.0)
    for _ in range(3):
        w.update(-65.0)
    check("TASK-10 3 consecutive drops counted", w.consecutive_drops == 3)
    w.update(-50.0)
    check("TASK-10 one normal sample resets the streak", w.consecutive_drops == 0)

    tm = TentativeMap(output_path=Path("/tmp/_t_map2.txt"), trajectory_path=Path("/tmp/_t_log2.txt"))
    tm.mark_human("wifi", note="hint", offset_m=1.0, is_hint=True)
    check("TASK-10 wifi hint does NOT inflate human_count", tm.human_count == 0)
    tm.mark_human("cv_standing", note="real", offset_m=1.0)
    check("TASK-10 confirmed human DOES count", tm.human_count == 1)


def test_task11_swarm() -> None:
    import swarm_controller as sc  # imports cleanly with the fixes
    r1 = sc.RobotContext(1, "A", "1.1.1.1", 1, 2, "u", "m", "t", "left")
    r2 = sc.RobotContext(2, "B", "1.1.1.2", 3, 4, "u", "m", "t", "right")
    check("TASK-11 RobotContext now tracks dist_right_cm", hasattr(r1, "dist_right_cm"))
    check("TASK-11 RobotContext carries per-robot WiFi sensor", hasattr(r1, "wifi_sensor"))
    r1.grid._set(10, 10, OCCUPIED)
    r2.grid._set(10, 10, FREE)
    r2.grid._set(30, 30, VISITED)
    m = sc.merge_occupancy_grids(r1, r2)
    check("TASK-11 merged grid: OCCUPIED from either robot wins", m.get(10, 10) == OCCUPIED)
    check("TASK-11 merged grid keeps other robot's VISITED", m.get(30, 30) == VISITED)
    src = (ROOT / "swarm_controller.py").read_text(encoding="utf-8")
    check("TASK-11 dist_right no longer hardcoded to -1", "dist_right=-1" not in src)
    check("TASK-11 wifi_alert no longer hardcoded to None", "wifi_alert=None" not in src)


def test_task12_hud() -> None:
    import inspect

    from utils.terminal_hud import TerminalDashboard

    params = inspect.signature(TerminalDashboard.render_single_robot).parameters
    check("TASK-12 HUD shows A* path length", "astar_path_len" in params)
    check("TASK-12 HUD shows unique human count", "human_count" in params)
    check("TASK-12 HUD shows frontier goal", "frontier_goal" in params)


def main() -> int:
    for fn in (
        test_task01_sonar_filter, test_task02_yaw_guard, test_task03_honest_pose,
        test_task04_05_grid, test_task06_astar, test_task07_fsm, test_task08_watchdog,
        test_task09_tracker, test_task10_wifi_hint, test_task11_swarm, test_task12_hud,
        test_debug_force_forward_flag,
    ):
        try:
            fn()
        except Exception as exc:  # a crash is a failure too
            check(f"{fn.__name__} crashed: {type(exc).__name__}: {exc}", False)
    passed = sum(ok for _, ok in _results)
    print(f"\n{passed}/{len(_results)} checks passed")
    return 0 if passed == len(_results) else 1


if __name__ == "__main__":
    sys.exit(main())
