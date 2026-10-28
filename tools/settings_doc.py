"""Read config/settings.yaml as documentation: the leaf keys in file order, their comments and their units.

PyYAML drops comments, so the comments are read from the text. Rules the file follows (tests enforce them):

* a value is described by the comment lines directly above it (no blank line between), or by a trailing ``# comment``;
* a section key (a key with nested keys) may have a comment block too; it describes the section, not a value;
* a mapping written on one line (``options: {...}``) is one value.
"""

from __future__ import annotations

import re
from pathlib import Path
from typing import Any, Mapping

from desktop_app.utils.settings import DEFAULT_SETTINGS_PATH, load_raw

# Mappings with free-form keys (UCI option names): one value, never recursed into.
OPAQUE_KEYS = frozenset({"options", "default_options"})

# Unit by key-name suffix, longest suffix first. The key of a value (or, for ``close_deg.P``, the key above it) carries it.
UNIT_SUFFIXES: tuple[tuple[str, str], ...] = (
    ("_percent_vdd", "% of Vdd"),
    ("_deg_s2", "deg/s^2"),
    ("_deg_s", "deg/s"),
    ("_deg", "deg"),
    ("_mm", "mm"),
    ("_ms", "ms"),
    ("_us", "us"),
    ("_hz", "Hz"),
    ("_ohm", "ohm"),
    ("_bytes", "bytes"),
    ("_bits", "bits"),
    ("_v", "V"),
    ("_s", "s"),
)

# Numeric values that have no unit: counts, GPIO numbers, channels, ratios, addresses. Listing one here is a decision;
# a new numeric key without a unit suffix and not listed here fails the test, so a unit is never forgotten.
DIMENSIONLESS: frozenset[str] = frozenset(
    {
        "geometry.squares_per_side",
        "drive.full_steps_per_rev",
        "drive.microstepping",
        "drive.gear_ratio",
        "pins.joint1.ena",
        "pins.joint1.dir",
        "pins.joint1.pul",
        "pins.joint2.ena",
        "pins.joint2.dir",
        "pins.joint2.pul",
        "pins.z_motor.pwm",
        "pins.z_motor.in1_raise",
        "pins.z_motor.in2_lower",
        "pins.i2c.sda",
        "pins.i2c.scl",
        "pins.servo_pwm",
        "pins.limit_top",
        "pins.limit_bottom",
        "electrical.ads1115.address",
        "electrical.as5600.outs_register",
        "electrical.as5600.joint1_alpha_adc_channel",
        "electrical.as5600.joint2_beta_adc_channel",
        "electrical.as5600.vdd_adc_channel",
        "electrical.as5600.r_fault_low",
        "electrical.as5600.r_fault_high",
        "electrical.z_axis.timeout_factor",
        "electrical.z_axis.ledc_channel",
        "electrical.servo.ledc_channel",
        "serial.baud",
        "serial.queue_depth",
        "link.max_in_flight",
        "link.reconnect_max_attempts",
        "engine.max_restarts",
        "engine.modes.beginner.max_depth",
        "engine.modes.amateur.max_depth",
        "engine.modes.player.max_depth",
        "engine.modes.master.max_depth",
        "engine.modes.grandmaster.max_depth",
        "logging.backup_count",
    }
)

_KEY_LINE = re.compile(r"^(?P<indent> *)(?P<key>[A-Za-z0-9_]+):(?:\s+(?P<value>.*))?$")


def leaf_paths(raw: Mapping[str, Any]) -> list[str]:
    """Dotted paths of every value of the parsed file, in file order (``assumed`` is not a setting)."""
    found: list[str] = []

    def walk(node: Mapping[str, Any], prefix: str) -> None:
        for key, value in node.items():
            dotted = f"{prefix}.{key}" if prefix else str(key)
            if not prefix and key == "assumed":
                continue
            if isinstance(value, Mapping) and key not in OPAQUE_KEYS:
                walk(value, dotted)
            else:
                found.append(dotted)

    walk(raw, "")
    return found


def value_at(raw: Mapping[str, Any], dotted: str) -> Any:
    node: Any = raw
    for part in dotted.split("."):
        node = node[part]
    return node


def _split_trailing_comment(value: str) -> tuple[str, str]:
    """``'100.0  # l: distance'`` -> (``'100.0'``, ``'l: distance'``); a '#' inside quotes is not a comment."""
    quote = ""
    for index, char in enumerate(value):
        if quote:
            if char == quote:
                quote = ""
        elif char in "\"'":
            quote = char
        elif char == "#" and (index == 0 or value[index - 1] in " \t"):
            return value[:index].strip(), value[index + 1 :].strip()
    return value.strip(), ""


def read_comments(text: str) -> dict[str, str]:
    """Dotted path of every value -> its comment ('' when it has none)."""
    comments: dict[str, str] = {}
    stack: list[tuple[int, str]] = []  # (indent, key) of the open sections
    pending: list[str] = []
    for line in text.splitlines():
        stripped = line.strip()
        if not stripped:
            pending = []
            continue
        if stripped.startswith("#"):
            pending.append(stripped.lstrip("#").strip())
            continue
        if stripped.startswith("-"):  # an item of a block list (the ``assumed`` list)
            pending = []
            continue
        match = _KEY_LINE.match(line)
        if not match:
            pending = []
            continue
        indent, key = len(match["indent"]), match["key"]
        value, trailing = _split_trailing_comment(match["value"] or "")
        while stack and stack[-1][0] >= indent:
            stack.pop()
        if not value:  # a section (or the ``assumed`` list): it only opens a scope
            stack.append((indent, key))
        else:
            dotted = ".".join([k for _, k in stack] + [key])
            comments[dotted] = " ".join([*pending, trailing]).strip()
        pending = []
    return comments


def unit_of(dotted: str) -> str:
    """The unit named by the key, or by the key above it; '' when the key names none."""
    parts = dotted.split(".")
    for name in (parts[-1], parts[-2] if len(parts) > 1 else ""):
        for suffix, unit in UNIT_SUFFIXES:
            if name.endswith(suffix):
                return unit
    return ""


def load_documentation(path: Path | None = None) -> tuple[dict[str, Any], dict[str, str]]:
    """(parsed data, comments by dotted path) of settings.yaml."""
    source = path if path is not None else DEFAULT_SETTINGS_PATH
    return load_raw(source), read_comments(source.read_text(encoding="utf-8"))
