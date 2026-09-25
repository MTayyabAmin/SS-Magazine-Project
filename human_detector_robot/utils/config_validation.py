"""project.yaml configuration validation utility.

This module validates that all required keys exist in the project configuration
YAML file and have the correct types before the application starts.

Problem Solved:
  Previously, missing or mistyped config keys would cause confusing KeyError
  or TypeError crashes deep in the code, making it hard to identify the root
  cause. Now validate_config() runs at startup and reports ALL problems at
  once with clear, actionable error messages.

Algorithm Overview:
  1. Define a list of required keys with expected types and human-readable hints.
  2. Walk the nested config dict using dotted path notation (e.g. 'robot.udp_port').
  3. Check each key exists and matches the expected type.
  4. Collect all problems and raise a single ConfigError with all issues listed.
"""

from __future__ import annotations

from typing import Any


class ConfigError(Exception):
    """Raised when a required config key is missing or has the wrong type.

    The exception message contains a formatted list of all problems found,
    with dotted key paths and hints for how to fix each one.
    """


# List of (dotted_key_path, expected_type, human_readable_hint) tuples.
# Each entry defines a required configuration key that must be present
# and must match the specified type for the application to function.
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
        config: The root configuration dictionary.
        dotted_path: Dot-separated key path (e.g. 'yolo.conf').

    Returns:
        Tuple of (value, found):
          - value: The value at the path, or None if not found.
          - found: True if the full path was traversed successfully.

    Algorithm:
      1. Split the dotted_path by '.' into individual key names.
      2. Walk the dict chain: for each key, check it exists in the current node.
      3. If any key is missing or a node is not a dict, return (None, False).
      4. If the full path is traversed, return (final_value, True).
    """
    node: Any = config
    for part in dotted_path.split("."):
        if not isinstance(node, dict) or part not in node:
            return None, False
        node = node[part]
    return node, True


def validate_config(config: dict[str, Any]) -> None:
    """Validate all required configuration keys are present and correctly typed.

    Args:
        config: The parsed project.yaml configuration dictionary.

    Raises:
        ConfigError: If any required key is missing or has the wrong type.
                     The exception message lists ALL problems found with
                     dotted key paths and fix hints.

    Algorithm:
      1. Verify config is actually a dict (catches empty/malformed YAML).
      2. Iterate over all _REQUIRED_KEYS entries.
      3. For each key, use _get_by_path to retrieve the value.
      4. If not found: record "key is missing" with its hint.
      5. If found but wrong type: record "key has wrong type (got X)" with hint.
      6. After checking all keys, if any problems exist, raise ConfigError
         with a formatted message listing all issues.
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
