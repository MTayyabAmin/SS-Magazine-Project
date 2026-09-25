"""YOLO11 vision utilities — human standing/fallen detection on laptop.

This module provides the complete computer vision pipeline for the ARK-5
human detection robot. It handles:
  - Loading YOLO11/YOLOv8 models (pretrained or custom).
  - Threaded camera stream capture with auto-reconnect and auto-discovery.
  - Running YOLO inference on frames to detect humans.
  - Classifying detected humans as 'standing' or 'fallen' based on
    bounding box aspect ratio or custom model class names.
  - Drawing detection overlays on frames for display/recording.

Algorithm Overview:
  1. Camera frames are captured in a background thread (ThreadedCameraStream).
  2. Main thread grabs the latest frame and runs YOLO prediction.
  3. YOLO returns bounding boxes for 'person' class (or custom classes).
  4. Each detection is classified as standing/fallen via aspect ratio heuristic.
  5. Results are drawn on the frame with colored boxes and labels.
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

_yolo_model = None
_log = logging.getLogger("ark5.vision")


def project_root() -> Path:
    """Return the project root directory (parent of this utils/ directory).

    Returns:
        Path object pointing to the human_detector_robot/ directory.
    """
    return Path(__file__).resolve().parent.parent


def load_config(path: Path | None = None) -> dict[str, Any]:
    """Load the project YAML configuration file.

    Args:
        path: Optional path to a custom config file.
              If None, loads config/project.yaml from project root.

    Returns:
        Parsed configuration as a nested dictionary.

    Algorithm:
      1. If no path given, default to project_root() / "config" / "project.yaml".
      2. Open and parse the YAML file using yaml.safe_load().
      3. Return the parsed dict.
    """
    if path is None:
        path = project_root() / "config" / "project.yaml"
    with open(path, encoding="utf-8") as f:
        return yaml.safe_load(f)


def _weights_path(config: dict[str, Any]) -> Path:
    """Resolve the YOLO model weights file path from configuration.

    Args:
        config: Project configuration dictionary.

    Returns:
        Absolute Path to the .pt weights file.

    Algorithm:
      1. Get the 'weights' filename from config['yolo']['weights'].
      2. Check if 'custom_model' flag is set in config.
      3. If custom: look in models/ directory, ensure .pt extension.
      4. If not custom: same location but may auto-download pretrained.
      5. Return the resolved Path.
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
    """Load or retrieve the cached YOLO model singleton.

    Args:
        config: Project configuration dictionary containing YOLO settings.

    Returns:
        YOLO model instance (ultralytics.YOLO).

    Algorithm:
      1. If _yolo_model is already loaded (global singleton), return it immediately.
      2. Import YOLO from ultralytics (lazy import to avoid slow startup).
      3. Resolve the weights file path from config.
      4. If weights file exists locally, load it.
      5. If not, download the pretrained model (e.g. yolo11n.pt).
      6. After download, copy weights to the models/ directory for caching.
      7. Store in global _yolo_model and return.
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
        # Auto-download pretrained model (yolo11n.pt etc.)
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
    """Run a warmup inference to initialize YOLO and CUDA/gpu memory.

    Args:
        config: Project configuration dictionary.

    Algorithm:
      1. Load the YOLO model (get_yolo handles caching).
      2. Create a black dummy image of the configured input size (default 640x640).
      3. Run predict() on the dummy image to trigger model compilation/warmup.
      4. First inference is typically slower due to CUDA graph capture; this
         ensures the actual detection loop starts at full speed.
    """
    model = get_yolo(config)
    imgsz = config.get("yolo", {}).get("imgsz", 640)
    dummy = np.zeros((imgsz, imgsz, 3), dtype=np.uint8)
    model.predict(dummy, verbose=False)


# Lazy import of auto_discovery with fallback for different package structures
try:
    from utils.auto_discovery import resolve_camera_url
except ImportError:
    try:
        from human_detector_robot.utils.auto_discovery import resolve_camera_url
    except ImportError:
        # Fallback: no-op if auto_discovery is unavailable
        def resolve_camera_url(url: str, **kwargs: Any) -> str:
            return url


def _open_capture(stream_url: str) -> cv2.VideoCapture:
    """Open an OpenCV VideoCapture from a URL or device index.

    Args:
        stream_url: Either a numeric string (local camera index) or
                    an HTTP URL (e.g. 'http://192.168.137.74:81/stream').

    Returns:
        OpenCV VideoCapture object (may or may not be successfully opened).

    Algorithm:
      1. If stream_url is all digits, treat as local camera index.
         Use CAP_DSHOW backend on Windows for better compatibility.
      2. Otherwise, treat as HTTP stream URL and open directly.
    """
    if stream_url.isdigit():
        cap = cv2.VideoCapture(int(stream_url), cv2.CAP_DSHOW)
    else:
        cap = cv2.VideoCapture(stream_url)
    return cap


class ThreadedCameraStream:
    """Background thread that continuously captures frames from a camera stream.

    Provides auto-discovery support: if the configured stream URL becomes
    unreachable, it scans the local subnet to find the ESP32-CAM's new IP.

    Attributes:
        stream_url: Active camera stream URL (may be updated by auto-discovery).
        reconnect_delay_s: Seconds to wait between reconnection attempts.
        robot_id: Robot identifier for camera selection in multi-robot setups.
        _cap: OpenCV VideoCapture instance.
        _frame: Latest captured frame (numpy array).
        _lock: Thread lock for safe frame access.
        _running: Flag to signal the capture thread to stop.
        _connected: Boolean indicating if the stream is currently connected.
        _thread: Background daemon thread running the capture loop.
        _reconnect_fails: Counter of consecutive reconnection failures.
    """

    def __init__(self, stream_url: str, reconnect_delay_s: float = 2.0, robot_id: int = 1) -> None:
        """Initialize and start the threaded camera stream.

        Args:
            stream_url: Camera stream URL or local device index string.
            reconnect_delay_s: Seconds to wait between reconnection attempts.
            robot_id: Robot ID for multi-robot camera selection.

        Algorithm:
          1. If stream_url is not a device index, run auto-discovery to resolve it.
          2. Open the VideoCapture.
          3. If first open fails, retry auto-discovery and open again.
          4. Read and discard 3 frames to flush the camera pipeline.
          5. Start the background capture thread.
        """
        if not stream_url.isdigit():
            stream_url = resolve_camera_url(stream_url, robot_id=robot_id)
        self.stream_url = stream_url
        self.reconnect_delay_s = reconnect_delay_s
        self.robot_id = robot_id
        self._reconnect_fails = 0

        self._cap = _open_capture(stream_url)
        if not self._cap.isOpened():
            # Second chance: try auto-discovery if initial open failed
            if not stream_url.isdigit():
                stream_url = resolve_camera_url(stream_url, robot_id=robot_id)
                self.stream_url = stream_url
                self._cap = _open_capture(stream_url)

        # Flush camera pipeline by reading and discarding 3 frames
        for _ in range(3):
            self._cap.read()

        self._frame: np.ndarray | None = None
        self._lock = threading.Lock()
        self._running = True
        self._connected = self._cap.isOpened()
        self._thread = threading.Thread(target=self._loop, daemon=True)
        self._thread.start()

    def _loop(self) -> None:
        """Background capture loop — continuously reads frames from the stream.

        Algorithm:
          1. While _running is True, read a frame from the VideoCapture.
          2. If read succeeds: store the frame under thread lock, set connected=True.
          3. If read fails: set connected=False, log warning, attempt reconnect.
          4. This loop runs as fast as the camera can deliver frames.
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
        """Attempt to reconnect to the camera stream after a failure.

        Algorithm:
          1. Release the current VideoCapture (ignore errors).
          2. Increment the consecutive failure counter.
          3. Wait for reconnect_delay_s seconds.
          4. If 2+ consecutive failures and URL is not a device index:
             a. Run auto-discovery to check if the camera got a new IP.
             b. If a new URL is found, update stream_url and reset fail counter.
          5. Try to open the (possibly updated) stream URL.
          6. If successful, reset the failure counter.
        """
        try:
            self._cap.release()
        except Exception:
            pass
        self._reconnect_fails += 1
        time.sleep(self.reconnect_delay_s)

        # After repeated failures, try auto-discovery for a new IP
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
        """Read the latest captured frame (thread-safe).

        Returns:
            Tuple of (success, frame):
              - success: True if a frame is available.
              - frame: Copy of the latest numpy frame, or None if unavailable.

        Algorithm:
          1. Acquire thread lock.
          2. If _frame is None, return (False, None).
          3. Otherwise, return (True, frame.copy()) — copy prevents race conditions
             with the background thread overwriting the frame.
        """
        with self._lock:
            if self._frame is None:
                return False, None
            return True, self._frame.copy()

    @property
    def is_connected(self) -> bool:
        """Check if the camera stream is currently connected and delivering frames.

        Returns:
            True if the last frame read was successful.
        """
        return self._connected

    def release(self) -> None:
        """Stop the capture thread and release the VideoCapture resource.

        Algorithm:
          1. Set _running=False to signal the loop thread to exit.
          2. Join the thread with a 1-second timeout.
          3. Release the OpenCV VideoCapture.
        """
        self._running = False
        self._thread.join(timeout=1.0)
        self._cap.release()


def open_stream(stream_url: str, robot_id: int = 1) -> ThreadedCameraStream:
    """Open an ESP32-CAM MJPEG HTTP stream or local webcam (threaded with auto-discovery).

    Args:
        stream_url: Camera stream URL or local device index string.
        robot_id: Robot ID for multi-robot camera selection.

    Returns:
        ThreadedCameraStream instance capturing frames in a background thread.
    """
    return ThreadedCameraStream(stream_url, robot_id=robot_id)


@dataclass
class HumanDetection:
    """Data class representing a single human detection from YOLO inference.

    Attributes:
        label: Classification label — 'standing', 'fallen', or 'person' (unclassified).
        confidence: YOLO detection confidence score (0.0 to 1.0).
        bbox: Bounding box as (x1, y1, x2, y2) pixel coordinates.
        aspect_ratio: Height/width ratio of the bounding box (used for posture classification).
    """
    label: str          # standing | fallen | person
    confidence: float
    bbox: tuple[int, int, int, int]  # x1,y1,x2,y2
    aspect_ratio: float


def _classify_posture(class_name: str, aspect_ratio: float, cfg: dict) -> str:
    """Classify a detected person as 'standing', 'fallen', or generic 'person'.

    Args:
        class_name: YOLO class name string (e.g. 'person', 'fallen', 'standing').
        aspect_ratio: Height/width ratio of the bounding box.
        cfg: Project configuration dictionary for posture thresholds.

    Returns:
        Classification string: 'standing', 'fallen', or 'person'.

    Algorithm:
      1. If class_name matches known fallen/lying labels → return 'fallen'.
      2. If class_name matches known standing labels → return 'standing'.
      3. Fallback to bbox aspect ratio heuristic:
         a. If aspect_ratio >= standing_ratio_min (default 1.2) → 'standing'
            (tall/narrow box = upright person).
         b. If aspect_ratio <= fallen_ratio_max (default 0.85) → 'fallen'
            (wide/short box = person lying down).
         c. Otherwise → 'person' (ambiguous, can't classify posture).
    """
    name = class_name.lower()
    if name in ("fallen", "lying", "fall"):
        return "fallen"
    if name in ("standing", "stand", "upright"):
        return "standing"

    # Fallback: bbox aspect ratio heuristic
    standing_min = cfg.get("posture", {}).get("standing_ratio_min", 1.2)
    fallen_max = cfg.get("posture", {}).get("fallen_ratio_max", 0.85)
    if aspect_ratio >= standing_min:
        return "standing"
    if aspect_ratio <= fallen_max:
        return "fallen"
    return "person"


def detect_humans(frame: np.ndarray, config: dict[str, Any]) -> list[HumanDetection]:
    """Run YOLO inference on a frame and return human detections with posture labels.

    Args:
        frame: Input image as a numpy BGR array (from OpenCV).
        config: Project configuration dictionary with YOLO settings.

    Returns:
        List of HumanDetection objects, one per detected human.

    Algorithm:
      1. Load the YOLO model (cached singleton via get_yolo).
      2. Extract YOLO config: confidence threshold, IoU threshold, image size.
      3. Run model.predict() on the frame with these parameters.
      4. For each detection in results:
         a. Extract class ID, class name, confidence score.
         b. Extract bounding box coordinates (x1, y1, x2, y2).
         c. Compute aspect ratio = height / width.
         d. Classify posture via _classify_posture().
         e. Append HumanDetection to results list.
      5. Return the list of all human detections.
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
    """Draw bounding boxes and labels for all detections on a copy of the frame.

    Args:
        frame: Input image as numpy BGR array.
        dets: List of HumanDetection objects to draw.

    Returns:
        New frame with detection overlays drawn (original is not modified).

    Algorithm:
      1. Copy the frame to avoid modifying the original.
      2. Define color mapping: standing=green, fallen=red, person=yellow.
      3. For each detection:
         a. Draw a colored rectangle around the bounding box.
         b. Draw the label text (class + confidence) above the box.
      4. Return the annotated frame.
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
    """Draw the current FPS counter on the frame.

    Args:
        frame: Input image as numpy BGR array.
        fps: Current frames per second value.

    Returns:
        Frame with FPS text overlay drawn.
    """
    cv2.putText(frame, f"FPS: {fps:.1f}", (10, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
    return frame


class FpsCounter:
    """Sliding-window FPS calculator for smooth frame rate measurement.

    Attributes:
        _times: List of timestamps for recent frames.
        _window: Number of frames in the sliding window for averaging.
    """

    def __init__(self, window: int = 30) -> None:
        """Initialize the FPS counter.

        Args:
            window: Number of recent frames to average over (default 30).
        """
        self._times: list[float] = []
        self._window = window

    def tick(self) -> float:
        """Record a frame timestamp and compute the current FPS.

        Returns:
            Current frames per second, or 0.0 if fewer than 2 frames recorded.

        Algorithm:
          1. Record the current timestamp.
          2. If the window is full, remove the oldest timestamp.
          3. If fewer than 2 timestamps, return 0.0 (can't compute rate).
          4. FPS = (number_of_intervals) / (time_span).
             E.g. with 30 frames: fps = 29 / (last_time - first_time).
        """
        now = time.time()
        self._times.append(now)
        if len(self._times) > self._window:
            self._times.pop(0)
        if len(self._times) < 2:
            return 0.0
        return (len(self._times) - 1) / (self._times[-1] - self._times[0])
