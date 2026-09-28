"""project.yaml ke zaroori settings validate karta hai.

FIX: Pehle agar `config/project.yaml` mein koi zaroori key missing hoti,
ya kisi key ki type galat hoti (jaise number ki jagah text), code beech
mein chal kar confusing `KeyError` ya `TypeError` ke saath crash ho
jata tha — is se pata nahi chalta tha ke ASAL mein masla kahan hai.

Ab `validate_config()` program shuru hote hi saari zaroori settings
check karta hai aur agar kuch missing/galat ho to EK clear error message
deta hai jisme exactly bataya hota hai kaunsi setting theek karni hai.

Algorithm Overview:
  1. A table of required keys: (dotted_path, expected_type, hint).
  2. Walk the nested config dict using dotted path notation ('robot.udp_port').
  3. Collect ALL missing/wrong-type problems (never fails on the first one).
  4. Raise a single ConfigError listing every problem with a fix hint.
"""

from __future__ import annotations

from typing import Any


class ConfigError(Exception):
    """project.yaml mein koi zaroori setting missing ya galat hai.

    The exception message contains a formatted list of all problems found,
    each with its dotted key path and a human-readable fix hint.
    """


# (dotted key path, expected type, human-readable hint)
# Each tuple is a REQUIRED config entry: it must exist at the dotted path
# and its value must be an instance of the given type (int passes for
# float checks too only if the tuple says so — note (int, float) covers both).
_REQUIRED_KEYS: list[tuple[str, type, str]] = [
    ("camera.stream_url", str, "ESP32-CAM ka MJPEG URL, e.g. http://192.168.1.105:81/stream"),
    ("robot.udp_host", str, "Robot ESP32 ka IP address"),
    ("robot.udp_port", int, "UDP port jahan motor commands jaate hain (e.g. 4210)"),
    ("robot.telemetry_port", int, "UDP port jahan se telemetry aati hai (e.g. 4211)"),
    ("yolo.weights", str, "YOLO model file ka naam, e.g. yolov8n.pt"),
    ("yolo.conf", (int, float), "Detection confidence threshold, e.g. 0.4"),
]


def _get_by_path(config: dict[str, Any], dotted_path: str):
    """Traverse a nested dict using a dotted key path (e.g. 'robot.udp_port').

    Args:
        config: Root configuration dictionary.
        dotted_path: Dot-separated key path (e.g. 'yolo.conf').

    Returns:
        Tuple (value, found):
          - value: value at the path, or None if not found.
          - found: True only if every segment of the path was traversable.

    Algorithm:
      1. Split dotted_path on '.'.
      2. For each segment: if current node isn't a dict or lacks the key →
         return (None, False).
      3. Descend into the value.
      4. Full traversal → return (final_value, True).
    """
    node: Any = config
    for part in dotted_path.split("."):
        if not isinstance(node, dict) or part not in node:
            return None, False
        node = node[part]
    return node, True


def validate_config(config: dict[str, Any]) -> None:
    """Raise ConfigError agar koi zaroori setting missing/galat type ki ho.

    Args:
        config: Parsed project.yaml as a nested dictionary.

    Raises:
        ConfigError: if config isn't a dict, OR any required key is missing
            or has the wrong type. The message lists EVERY problem found.

    Algorithm:
      1. Guard: config must be a dict (catches empty/malformed YAML).
      2. For each entry in _REQUIRED_KEYS:
         a. Fetch value via _get_by_path.
         b. Missing/None → append "missing" problem with hint.
         c. Wrong type → append "wrong type (got X)" problem with hint.
      3. If any problems were collected, raise ConfigError with a
         formatted bullet list of all of them.
    """
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
