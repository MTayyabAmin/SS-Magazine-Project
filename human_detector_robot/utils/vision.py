"""YOLO11 vision utilities — human standing/fallen detection on laptop."""

from __future__ import annotations

import logging
import threading
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import cv2
import numpy as np
import yaml

cv2.setNumThreads(1)

_yolo_model = None
_log = logging.getLogger("ark5.vision")


def project_root() -> Path:
    return Path(__file__).resolve().parent.parent


def load_config(path: Path | None = None) -> dict[str, Any]:
    if path is None:
        path = project_root() / "config" / "project.yaml"
    with open(path, encoding="utf-8") as f:
        return yaml.safe_load(f)


def _weights_path(config: dict[str, Any]) -> Path:
    yolo_cfg = config.get("yolo", {})
    weights = yolo_cfg.get("weights", "yolov8n.pt")
    custom = yolo_cfg.get("custom_model", False)
    root = project_root()

    if custom:
        p = root / "models" / weights
        if not weights.endswith(".pt"):
            p = p.with_suffix(".pt")
        return p

    p = root / "models" / weights
    if not p.suffix:
        p = p.with_suffix(".pt")
    return p


def get_yolo(config: dict[str, Any]):
    global _yolo_model
    if _yolo_model is not None:
        return _yolo_model

    from ultralytics import YOLO

    weights = _weights_path(config)
    if weights.is_file():
        _log.info("Loading YOLO weights: %s", weights)
        _yolo_model = YOLO(str(weights))
    else:
        # Auto-download pretrained (yolo11n.pt etc.)
        name = config.get("yolo", {}).get("weights", "yolo11n")
        _log.info("Downloading/loading YOLO: %s", name)
        _yolo_model = YOLO(name)
        weights.parent.mkdir(parents=True, exist_ok=True)
        import shutil
        src = Path(_yolo_model.ckpt_path or "")
        if src.is_file():
            shutil.copy2(src, weights)

    return _yolo_model


def warmup_yolo(config: dict[str, Any]) -> None:
    model = get_yolo(config)
    imgsz = config.get("yolo", {}).get("imgsz", 640)
    dummy = np.zeros((imgsz, imgsz, 3), dtype=np.uint8)
    model.predict(dummy, verbose=False)


try:
    from utils.auto_discovery import resolve_camera_url
except ImportError:
    try:
        from human_detector_robot.utils.auto_discovery import resolve_camera_url
    except ImportError:
        def resolve_camera_url(url: str, **kwargs: Any) -> str:
            return url


def _open_capture(stream_url: str) -> cv2.VideoCapture:
    if stream_url.isdigit():
        cap = cv2.VideoCapture(int(stream_url), cv2.CAP_DSHOW)
    else:
        cap = cv2.VideoCapture(stream_url)
    return cap


class ThreadedCameraStream:
    """Background thread jo hamesha latest frame kheenchta rehta hai.
    Auto-discovery support: agar stream URL unreachable ho, subnet scan karke
    ESP32-CAM ka naya IP auto-detect karta hai.
    """

    def __init__(self, stream_url: str, reconnect_delay_s: float = 2.0, robot_id: int = 1) -> None:
        if not stream_url.isdigit():
            stream_url = resolve_camera_url(stream_url, robot_id=robot_id)
        self.stream_url = stream_url
        self.reconnect_delay_s = reconnect_delay_s
        self.robot_id = robot_id
        self._reconnect_fails = 0

        self._cap = _open_capture(stream_url)
        if not self._cap.isOpened():
            # Second chance: try auto-discovery scan if first open failed
            if not stream_url.isdigit():
                stream_url = resolve_camera_url(stream_url, robot_id=robot_id)
                self.stream_url = stream_url
                self._cap = _open_capture(stream_url)

        for _ in range(3):
            self._cap.read()

        self._frame: np.ndarray | None = None
        self._lock = threading.Lock()
        self._running = True
        self._connected = self._cap.isOpened()
        self._thread = threading.Thread(target=self._loop, daemon=True)
        self._thread.start()

    def _loop(self) -> None:
        while self._running:
            ok, frame = self._cap.read()
            if not ok or frame is None:
                self._connected = False
                _log.warning("Stream read fail — reconnect try ho raha hai: %s", self.stream_url)
                self._reconnect()
                continue
            self._connected = True
            with self._lock:
                self._frame = frame

    def _reconnect(self) -> None:
        try:
            self._cap.release()
        except Exception:
            pass
        self._reconnect_fails += 1
        time.sleep(self.reconnect_delay_s)

        # Agar reconnect baar baar fail ho raha ho, to check karo naya IP to nahi mila
        if self._reconnect_fails >= 2 and not self.stream_url.isdigit():
            new_url = resolve_camera_url(self.stream_url, robot_id=self.robot_id)
            if new_url != self.stream_url:
                _log.info("Auto-discovery ne naya camera URL dhoond liya: %s", new_url)
                self.stream_url = new_url
                self._reconnect_fails = 0

        try:
            self._cap = _open_capture(self.stream_url)
            if self._cap.isOpened():
                self._reconnect_fails = 0
        except Exception as exc:
            _log.warning("Reconnect attempt failed: %s", exc)

    def read(self) -> tuple[bool, np.ndarray | None]:
        with self._lock:
            if self._frame is None:
                return False, None
            return True, self._frame.copy()

    @property
    def is_connected(self) -> bool:
        return self._connected

    def release(self) -> None:
        self._running = False
        self._thread.join(timeout=1.0)
        self._cap.release()


def open_stream(stream_url: str, robot_id: int = 1) -> ThreadedCameraStream:
    """Open ESP32-CAM MJPEG HTTP stream or local webcam index (threaded with auto-discovery)."""
    return ThreadedCameraStream(stream_url, robot_id=robot_id)


@dataclass
class HumanDetection:
    label: str          # standing | fallen | person
    confidence: float
    bbox: tuple[int, int, int, int]  # x1,y1,x2,y2
    aspect_ratio: float


def _classify_posture(class_name: str, aspect_ratio: float, cfg: dict) -> str:
    """Map YOLO class or bbox shape to standing/fallen."""
    name = class_name.lower()
    if name in ("fallen", "lying", "fall"):
        return "fallen"
    if name in ("standing", "stand", "upright"):
        return "standing"

    # Fallback: bbox aspect ratio (height/width)
    standing_min = cfg.get("posture", {}).get("standing_ratio_min", 1.2)
    fallen_max = cfg.get("posture", {}).get("fallen_ratio_max", 0.85)
    if aspect_ratio >= standing_min:
        return "standing"
    if aspect_ratio <= fallen_max:
        return "fallen"
    return "person"


def detect_humans(frame: np.ndarray, config: dict[str, Any]) -> list[HumanDetection]:
    model = get_yolo(config)
    yolo_cfg = config.get("yolo", {})
    conf = yolo_cfg.get("conf", 0.4)
    iou = yolo_cfg.get("iou", 0.45)
    imgsz = yolo_cfg.get("imgsz", 640)

    results = model.predict(frame, conf=conf, iou=iou, imgsz=imgsz, verbose=False)
    detections: list[HumanDetection] = []

    for r in results:
        if r.boxes is None:
            continue
        names = r.names
        for box in r.boxes:
            cls_id = int(box.cls[0])
            class_name = names.get(cls_id, str(cls_id))
            score = float(box.conf[0])
            x1, y1, x2, y2 = map(int, box.xyxy[0].tolist())
            w = max(x2 - x1, 1)
            h = max(y2 - y1, 1)
            aspect = h / w
            label = _classify_posture(class_name, aspect, config)
            detections.append(HumanDetection(label, score, (x1, y1, x2, y2), aspect))

    return detections


def draw_detections(frame: np.ndarray, dets: list[HumanDetection]) -> np.ndarray:
    out = frame.copy()
    colors = {"standing": (0, 255, 0), "fallen": (0, 0, 255), "person": (0, 255, 255)}
    for d in dets:
        x1, y1, x2, y2 = d.bbox
        color = colors.get(d.label, (255, 255, 255))
        cv2.rectangle(out, (x1, y1), (x2, y2), color, 2)
        text = f"{d.label} {d.confidence:.2f}"
        cv2.putText(out, text, (x1, max(y1 - 8, 12)), cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)
    return out


def draw_fps(frame: np.ndarray, fps: float) -> np.ndarray:
    cv2.putText(frame, f"FPS: {fps:.1f}", (10, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
    return frame


class FpsCounter:
    def __init__(self, window: int = 30) -> None:
        self._times: list[float] = []
        self._window = window

    def tick(self) -> float:
        now = time.time()
        self._times.append(now)
        if len(self._times) > self._window:
            self._times.pop(0)
        if len(self._times) < 2:
            return 0.0
        return (len(self._times) - 1) / (self._times[-1] - self._times[0])
