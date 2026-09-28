"""project.yaml ke zaroori settings validate karta hai.

FIX: Pehle agar `config/project.yaml` mein koi zaroori key missing hoti,
ya kisi key ki type galat hoti (jaise number ki jagah text), code beech
mein chal kar confusing `KeyError` ya `TypeError` ke saath crash ho
jata tha — is se pata nahi chalta tha ke ASAL mein masla kahan hai.

Ab `validate_config()` program shuru hote hi saari zaroori settings
check karta hai aur agar kuch missing/galat ho to EK clear error message
deta hai jisme exactly bataya hota hai kaunsi setting theek karni hai.
"""

from __future__ import annotations

from typing import Any


class ConfigError(Exception):
    """project.yaml mein koi zaroori setting missing ya galat hai."""


# (dotted key path, expected type, human-readable hint)
_REQUIRED_KEYS: list[tuple[str, type, str]] = [
    ("camera.stream_url", str, "ESP32-CAM ka MJPEG URL, e.g. http://192.168.1.105:81/stream"),
    ("robot.udp_host", str, "Robot ESP32 ka IP address"),
    ("robot.udp_port", int, "UDP port jahan motor commands jaate hain (e.g. 4210)"),
    ("robot.telemetry_port", int, "UDP port jahan se telemetry aati hai (e.g. 4211)"),
    ("yolo.weights", str, "YOLO model file ka naam, e.g. yolov8n.pt"),
    ("yolo.conf", (int, float), "Detection confidence threshold, e.g. 0.4"),
]


def _get_by_path(config: dict[str, Any], dotted_path: str):
    node: Any = config
    for part in dotted_path.split("."):
        if not isinstance(node, dict) or part not in node:
            return None, False
        node = node[part]
    return node, True


def validate_config(config: dict[str, Any]) -> None:
    """Raise ConfigError agar koi zaroori setting missing/galat type ki ho."""
    if not isinstance(config, dict):
        raise ConfigError("project.yaml khali ya galat format mein hai.")

    problems: list[str] = []
    for dotted_path, expected_type, hint in _REQUIRED_KEYS:
        value, found = _get_by_path(config, dotted_path)
        if not found or value is None:
            problems.append(f"  - '{dotted_path}' missing hai. ({hint})")
        elif not isinstance(value, expected_type):
            problems.append(
                f"  - '{dotted_path}' ki value galat type ki hai "
                f"(mila: {type(value).__name__}). ({hint})"
            )

    if problems:
        message = "project.yaml mein ye settings theek karo:\n" + "\n".join(problems)
        raise ConfigError(message)
