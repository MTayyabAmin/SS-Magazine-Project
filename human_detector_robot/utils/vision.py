"""YOLO11 vision utilities — human standing/fallen detection on laptop.

This module provides the complete computer vision pipeline for the ARK-5
robot:
  - YAML config loading (project.yaml) and project-root resolution.
  - YOLO model singleton (load once, reuse every frame) with auto-download
    of pretrained weights when no local .pt exists.
  - Threaded camera stream capture with auto-reconnect and subnet
    auto-discovery of ESP32-CAM IPs (DHCP-proof).
  - Human detection via YOLO predict + posture classification
    (standing/fallen/person) using class names or bbox aspect ratio.
  - Frame annotation helpers (boxes, labels, FPS) and an FPS counter.

Algorithm Overview:
  1. A background thread (ThreadedCameraStream) keeps the LATEST frame.
  2. The main loop grabs a copy and runs YOLO inference on it.
  3. Each detected box is mapped to a posture label and returned as a
     HumanDetection dataclass.
  4. Callers draw the detections and measure FPS for the HUD/preview.
"""

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

# Limit OpenCV to 1 thread to prevent CPU contention with YOLO inference
cv2.setNumThreads(1)

_yolo_model = None           # module-level singleton cache for the YOLO model
_log = logging.getLogger("ark5.vision")


def project_root() -> Path:
    """Return the project root directory (parent of this utils/ folder).

    Returns:
        Path to human_detector_robot/ — used to build relative paths for
        config/, models/, maps/, logs/.
    """
    return Path(__file__).resolve().parent.parent


def load_config(path: Path | None = None) -> dict[str, Any]:
    """Load the project YAML configuration file.

    Args:
        path: Explicit config path; None → <project_root>/config/project.yaml.

    Returns:
        Parsed YAML as a nested dictionary.

    Algorithm:
      1. Resolve the path (default location if None).
      2. Open with UTF-8 and parse via yaml.safe_load().
      3. Return the resulting dict.
    """
    if path is None:
        path = project_root() / "config" / "project.yaml"
    with open(path, encoding="utf-8") as f:
        return yaml.safe_load(f)


def _weights_path(config: dict[str, Any]) -> Path:
    """Resolve where the YOLO .pt weights file should live on disk.

    Args:
        config: Parsed project.yaml (reads yolo.weights, yolo.custom_model).

    Returns:
        Path under <root>/models/<weights> guaranteed to end with '.pt'.

    Algorithm:
      1. Read weights name (default 'yolov8n.pt') and custom_model flag.
      2. Build <root>/models/<weights>.
      3. Custom models: force a '.pt' suffix even if the config omitted it.
      4. Pretrained: append '.pt' only when the name has no suffix at all
         (names like 'yolo11n' become 'yolo11n.pt').
    """
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
    """Load (or return the cached) YOLO model singleton.

    Args:
        config: Parsed project.yaml.

    Returns:
        ultralytics.YOLO instance.

    Algorithm:
      1. If _yolo_model is already set → return it (load exactly once).
      2. Lazy-import ultralytics.YOLO (avoids import cost when unused).
      3. Resolve weights path via _weights_path().
      4. File exists → YOLO(path).
      5. Missing → instantiate YOLO(name) which auto-downloads the
         pretrained checkpoint, then copy it into models/ for next time
         (so the download happens only once per machine).
      6. Cache in the module global and return.
    """
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
    """Run one dummy inference so the FIRST real frame isn't slow.

    Args:
        config: Parsed project.yaml (reads yolo.imgsz, default 640).

    Algorithm:
      1. Load the model (get_yolo caches it).
      2. Build a black imgsz x imgsz x 3 uint8 image.
      3. model.predict(dummy) triggers weight upload / CUDA graph capture /
         kernel autotuning — costs a second once, saves it on every frame.
    """
    model = get_yolo(config)
    imgsz = config.get("yolo", {}).get("imgsz", 640)
    dummy = np.zeros((imgsz, imgsz, 3), dtype=np.uint8)
    model.predict(dummy, verbose=False)


# Import auto-discovery with graceful fallbacks so vision.py still works
# when run from different package layouts (utils.* or human_detector_robot.utils.*),
# and degrades to a pass-through if the module is missing entirely.
try:
    from utils.auto_discovery import resolve_camera_url
except ImportError:
    try:
        from human_detector_robot.utils.auto_discovery import resolve_camera_url
    except ImportError:
        def resolve_camera_url(url: str, **kwargs: Any) -> str:
            return url


def _open_capture(stream_url: str) -> cv2.VideoCapture:
    """Open an OpenCV VideoCapture from a URL or a device index string.

    Args:
        stream_url: All-digit string → local camera index (uses CAP_DSHOW
            on Windows for stability); otherwise treated as an HTTP URL.

    Returns:
        cv2.VideoCapture (open or not — caller checks isOpened()).

    Algorithm:
      1. If stream_url.isdigit() → VideoCapture(int(url), CAP_DSHOW).
      2. Else → VideoCapture(url) for MJPEG streams.
    """
    if stream_url.isdigit():
        cap = cv2.VideoCapture(int(stream_url), cv2.CAP_DSHOW)
    else:
        cap = cv2.VideoCapture(stream_url)
    return cap


class ThreadedCameraStream:
    """Background thread jo hamesha latest frame kheenchta rehta hai.
    Auto-discovery support: agar stream URL unreachable ho, subnet scan karke
    ESP32-CAM ka naya IP auto-detect karta hai.

    Callers get non-blocking read() copies while the daemon thread blocks
    on the camera instead of the control loop.

    Attributes:
        stream_url: Current stream URL (may be replaced by auto-discovery).
        reconnect_delay_s: Seconds to sleep between reconnect attempts.
        robot_id: Robot id used to pick the right camera when several are
            found during discovery (Robot 1 → first, Robot 2 → second).
        _cap: Active VideoCapture.
        _frame: Latest frame (or None before the first successful read).
        _lock: Guards _frame against the reader thread.
        _running: False requests the loop to exit.
        _connected: True while reads are succeeding.
        _thread: Daemon capture thread.
        _reconnect_fails: Consecutive failed reconnect attempts.
    """

    def __init__(self, stream_url: str, reconnect_delay_s: float = 2.0, robot_id: int = 1) -> None:
        """Resolve the URL, open the capture, flush it, start the thread.

        Args:
            stream_url: Camera URL or device-index string.
            reconnect_delay_s: Delay between reconnect attempts (seconds).
            robot_id: Robot id for multi-camera discovery selection.

        Algorithm:
          1. Non-numeric URLs go through resolve_camera_url() first (fast
             reachability check + subnet scan if unreachable).
          2. _open_capture(); if it fails to open and the URL is not a
             device index, run discovery once more and retry.
          3. Read and discard 3 frames (flush stale/buffered frames so the
             thread starts on fresh data).
          4. Initialize lock/flags and start the daemon _loop thread.
        """
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
        """Capture loop — always keeps the newest frame available.

        Algorithm:
          1. While _running: cap.read().
          2. Failure → _connected = False, log, call _reconnect(), continue.
          3. Success → _connected = True; store frame.copy()-worthy reference
             under the lock (readers copy on access).
        """
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
        """Recover from a dead stream: sleep, re-discover if needed, reopen.

        Algorithm:
          1. Release the current capture (ignore errors).
          2. Increment _reconnect_fails and sleep reconnect_delay_s.
          3. After ≥2 consecutive failures (and for URL streams), run
             auto-discovery; if a NEW url is found, adopt it and reset the
             failure counter (handles the ESP32 getting a new DHCP IP).
          4. Reopen; on success reset _reconnect_fails.
        """
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
        """Thread-safe read of the latest frame.

        Returns:
            (ok, frame): ok=True with a COPY of the newest frame, or
            (False, None) before the first frame arrives.

        Algorithm:
          1. Lock.
          2. No frame yet → (False, None).
          3. Else return (True, frame.copy()) — the copy prevents races
             with the capture thread overwriting the buffer mid-use.
        """
        with self._lock:
            if self._frame is None:
                return False, None
            return True, self._frame.copy()

    @property
    def is_connected(self) -> bool:
        """True when the last read attempt succeeded (stream healthy)."""
        return self._connected

    def release(self) -> None:
        """Stop the capture thread and free the VideoCapture.

        Algorithm:
          1. _running = False → loop exits on next iteration.
          2. join(timeout=1.0) so we don't hang forever.
          3. Release the capture.
        """
        self._running = False
        self._thread.join(timeout=1.0)
        self._cap.release()


def open_stream(stream_url: str, robot_id: int = 1) -> ThreadedCameraStream:
    """Open ESP32-CAM MJPEG HTTP stream or local webcam index (threaded with auto-discovery).

    Args:
        stream_url: Camera URL or device-index string.
        robot_id: Robot id for discovery-time camera selection.

    Returns:
        A started ThreadedCameraStream (its background thread is running).
    """
    return ThreadedCameraStream(stream_url, robot_id=robot_id)


@dataclass
class HumanDetection:
    """One YOLO human detection with posture classification.

    Attributes:
        label: 'standing' | 'fallen' | 'person' (posture classification).
        confidence: YOLO confidence score [0..1].
        bbox: Pixel bounding box (x1, y1, x2, y2).
        aspect_ratio: height/width of the box — the posture fallback signal.
    """
    label: str          # standing | fallen | person
    confidence: float
    bbox: tuple[int, int, int, int]  # x1,y1,x2,y2
    aspect_ratio: float


def _classify_posture(class_name: str, aspect_ratio: float, cfg: dict) -> str:
    """Map YOLO class or bbox shape to standing/fallen.

    Args:
        class_name: Raw YOLO class name (e.g. 'person', 'fallen').
        aspect_ratio: bbox height / width.
        cfg: project.yaml dict (reads posture.standing_ratio_min,
            posture.fallen_ratio_max).

    Returns:
        'fallen' | 'standing' | 'person'.

    Algorithm:
      1. Known fallen aliases (fallen/lying/fall) → 'fallen'.
      2. Known standing aliases (standing/stand/upright) → 'standing'.
      3. Aspect-ratio fallback (tall box = upright, wide box = lying):
         a. ratio >= standing_ratio_min (default 1.2) → 'standing'.
         b. ratio <= fallen_ratio_max (default 0.85) → 'fallen'.
         c. otherwise → 'person' (ambiguous — no posture claim).
    """
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
    """Run YOLO inference on a frame and return posture-labelled humans.

    Args:
        frame: BGR image (numpy array, as from OpenCV).
        config: project.yaml dict (reads yolo.conf/iou/imgsz).

    Returns:
        List of HumanDetection — one per detected box.

    Algorithm:
      1. Get the cached YOLO model.
      2. Read thresholds: conf (0.4), iou (0.45), imgsz (640).
      3. model.predict(frame, conf, iou, imgsz, verbose=False).
      4. For each result/box:
         a. class id → class name via r.names.
         b. extract confidence and xyxy pixels.
         c. compute w, h (min 1 to avoid /0), aspect = h/w.
         d. label = _classify_posture(...).
         e. append HumanDetection(label, score, bbox, aspect).
      5. Return the list (empty if nothing detected).
    """
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
    """Draw labeled bounding boxes for detections on a copy of the frame.

    Args:
        frame: Source BGR image.
        dets: HumanDetections to draw.

    Returns:
        Annotated COPY (the input frame is never modified).

    Algorithm:
      1. frame.copy().
      2. Color map: standing=green, fallen=red, person=yellow, default white.
      3. For each det: rectangle(x1,y1,x2,y2, color, 2) plus the text
         "label confidence" placed 8px above the box (min y=12).
      4. Return the annotated image.
    """
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
    """Overlay the current FPS value at the top-left of the frame (in place).

    Args:
        frame: BGR image to draw on (modified in place).
        fps: Frames-per-second value to display.

    Returns:
        The same frame with "FPS: n.n" text drawn at (10, 24).
    """
    cv2.putText(frame, f"FPS: {fps:.1f}", (10, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
    return frame


class FpsCounter:
    """Sliding-window FPS calculator (smooth, not single-frame noisy).

    Attributes:
        _times: Timestamps of the last `window` frames.
        _window: Max samples retained (default 30).
    """

    def __init__(self, window: int = 30) -> None:
        """Initialize the counter.

        Args:
            window: Number of frames in the averaging window.
        """
        self._times: list[float] = []
        self._window = window

    def tick(self) -> float:
        """Record a frame timestamp and compute the current FPS.

        Returns:
            (n-1) / (t_last - t_first) over the window; 0.0 with <2 samples.

        Algorithm:
          1. Append now.
          2. Trim oldest if the window is exceeded.
          3. Fewer than 2 samples → 0.0 (rate undefined).
          4. Else FPS = intervals / span = (len-1) / (last - first).
        """
        now = time.time()
        self._times.append(now)
        if len(self._times) > self._window:
            self._times.pop(0)
        if len(self._times) < 2:
            return 0.0
        return (len(self._times) - 1) / (self._times[-1] - self._times[0])
