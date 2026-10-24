"""Guards for the firmware project layout (phase 4a).

These tests read the firmware files as text. They do not compile anything; the C++ tests run with
``pio test -e native`` or ``firmware/tools/run_host_tests.sh``.

* platformio.ini: every version is pinned exactly, and the monitor baud equals ``kBaudRate``.
* config.h / pins.h agree with config/hardware.yaml and docs/SPEC.md section 4.2; every value the SPEC
  leaves open carries a ``TODO(confirm)``.
* The hardware-independent core includes nothing from Arduino or the ESP32 SDK.
* main.cpp holds no number of its own and puts the pin levels first in ``setup()`` (SPEC 10.2).
"""

from __future__ import annotations

import configparser
import re
from pathlib import Path

import pytest
import yaml

REPO_ROOT = Path(__file__).resolve().parents[1]
FIRMWARE = REPO_ROOT / "firmware"
INI = FIRMWARE / "platformio.ini"
CONFIG_H = FIRMWARE / "include" / "config.h"
PINS_H = FIRMWARE / "include" / "pins.h"
PROTOCOL_H = FIRMWARE / "include" / "protocol.h"
MAIN_CPP = FIRMWARE / "src" / "main.cpp"
CORE_DIR = FIRMWARE / "lib" / "chessbot_core" / "src"
SPEC = REPO_ROOT / "docs" / "SPEC.md"
HARDWARE_YAML = REPO_ROOT / "config" / "hardware.yaml"

CORE_FILES = sorted(p for p in CORE_DIR.iterdir() if p.suffix in {".h", ".cpp"})


def _code_only(text: str) -> str:
    """The text without // comments and without /* */ comments."""
    text = re.sub(r"/\*.*?\*/", "", text, flags=re.S)
    return "\n".join(line.split("//", 1)[0] for line in text.splitlines())


def _constants(path: Path) -> dict[str, str]:
    """``constexpr <type> kName = value;`` of a header, as text."""
    found: dict[str, str] = {}
    for match in re.finditer(r"constexpr\s+[\w:]+\s+(k\w+)(?:\[\])?\s*=\s*([^;]+);", _code_only(path.read_text(encoding="utf-8"))):
        found[match.group(1)] = match.group(2).strip()
    return found


def _number(text: str) -> float:
    return float(text.rstrip("fF").replace("'", ""))


@pytest.fixture(scope="module")
def ini() -> configparser.ConfigParser:
    parser = configparser.ConfigParser(interpolation=None, allow_no_value=True, comment_prefixes=(";", "#"))
    parser.read(INI, encoding="utf-8")
    return parser


@pytest.fixture(scope="module")
def config() -> dict[str, str]:
    return _constants(CONFIG_H)


@pytest.fixture(scope="module")
def pins() -> dict[str, str]:
    return _constants(PINS_H)


@pytest.fixture(scope="module")
def hardware() -> dict:
    return yaml.safe_load(HARDWARE_YAML.read_text(encoding="utf-8"))


# --------------------------------------------------------------------------- #
# platformio.ini
# --------------------------------------------------------------------------- #


def test_the_two_environments_exist(ini: configparser.ConfigParser) -> None:
    assert "env:esp32dev" in ini and "env:native" in ini
    assert ini["env:esp32dev"]["board"] == "esp32dev"
    assert ini["env:esp32dev"]["framework"] == "arduino"


def test_monitor_baud_equals_the_baud_in_config_h(ini: configparser.ConfigParser, config: dict[str, str]) -> None:
    assert int(ini["env:esp32dev"]["monitor_speed"]) == int(_number(config["kBaudRate"])) == 115200


def test_monitor_line_ending_is_lf(ini: configparser.ConfigParser) -> None:
    assert ini["env:esp32dev"]["monitor_eol"].strip() == "LF"


EXACT_PIN = re.compile(r"@\s*\d+\.\d+\.\d+(?:[-+.][0-9A-Za-z.]+)?\s*$")


def is_exactly_pinned(entry: str) -> bool:
    """``name@x.y.z`` (spaces around @ allowed): an exact version, no range operator."""
    return bool(EXACT_PIN.search(entry)) and not re.search(r"[\^~*<>=|]", entry.split("@", 1)[1])


def _pinned_entries(ini: configparser.ConfigParser) -> list[tuple[str, str]]:
    entries: list[tuple[str, str]] = []
    for section in ("env:esp32dev", "env:native"):
        for key in ("platform", "platform_packages", "lib_deps"):
            for raw in ini[section].get(key, "").splitlines():
                entry = raw.strip()
                if entry:
                    entries.append((f"{section} {key}", entry))
    return entries


def test_every_platform_package_and_library_is_pinned_exactly(ini: configparser.ConfigParser) -> None:
    entries = _pinned_entries(ini)
    kinds = {key for key, _ in entries}
    assert {"env:esp32dev platform", "env:esp32dev platform_packages", "env:native platform", "env:native lib_deps"} <= kinds
    for where, entry in entries:
        assert is_exactly_pinned(entry), f"{where}: {entry!r} is not pinned to an exact version (name@x.y.z)"


def test_the_pin_check_itself_rejects_unpinned_and_range_entries() -> None:
    for bad in ("espressif32", "espressif32@^6.9.0", "lib @ ~1.2.3", "lib@6", "lib@6.9", "lib@*", "lib@>=1.0.0", "lib@1.2.3 || 2.0.0"):
        assert not is_exactly_pinned(bad), bad
    for good in ("espressif32@6.9.0", "platformio/framework-arduinoespressif32 @ 3.20017.241212", "throwtheswitch/Unity@2.6.1"):
        assert is_exactly_pinned(good), good


def test_an_unpinned_ini_would_be_caught() -> None:
    unpinned = configparser.ConfigParser(interpolation=None)
    unpinned.read_string("[env:esp32dev]\nplatform = espressif32\n[env:native]\nlib_deps =\n    throwtheswitch/Unity\n")
    entries = _pinned_entries(unpinned)
    assert entries and not all(is_exactly_pinned(entry) for _, entry in entries)


# --------------------------------------------------------------------------- #
# config.h and pins.h against hardware.yaml and the SPEC
# --------------------------------------------------------------------------- #

TO_CONFIRM_CONSTANTS = [
    "kBaudRate",
    "kQueueDepth",
    "kHeartbeatTimeoutMs",
    "kZFullTravelMs",
    "kMaxAccelDegPerS2",
    "kMaxSpeedDegPerS",
    "kDirectionChangeDelayUs",
    "kStepLossThresholdDeg",
]


@pytest.mark.parametrize("name", TO_CONFIRM_CONSTANTS)
def test_every_open_value_is_marked_todo_confirm(name: str) -> None:
    lines = CONFIG_H.read_text(encoding="utf-8").splitlines()
    index = next(i for i, line in enumerate(lines) if re.search(rf"\b{name}\b\s*=", line))
    window = "\n".join(lines[max(0, index - 4) : index + 1])
    assert "TODO(confirm)" in window, f"{name} is a [TO CONFIRM] value without a TODO(confirm) comment"


def test_unset_values_are_zero_so_they_cannot_be_used_by_accident(config: dict[str, str]) -> None:
    for name in ("kZFullTravelMs", "kMaxAccelDegPerS2", "kMaxSpeedDegPerS", "kDirectionChangeDelayUs"):
        assert _number(config[name]) == 0, name


def test_config_h_agrees_with_hardware_yaml(config: dict[str, str], hardware: dict) -> None:
    electrical = hardware["electrical"]
    assert _number(config["kZMotorPwmHz"]) == electrical["z_axis"]["pwm_frequency_hz"]
    assert _number(config["kZTimeoutFactor"]) == electrical["z_axis"]["timeout_factor"]
    assert _number(config["kServoPwmHz"]) == electrical["servo"]["pwm_frequency_hz"]
    assert _number(config["kServoPulseMinUs"]) == electrical["servo"]["pulse_us_at_0_deg"]
    assert _number(config["kServoPulseMaxUs"]) == electrical["servo"]["pulse_us_at_180_deg"]
    assert _number(config["kSensorRMin"]) == electrical["as5600"]["r_fault_low"]
    assert _number(config["kSensorRMax"]) == electrical["as5600"]["r_fault_high"]
    limits = electrical["limit_switches"]
    assert limits["debounce_ms_min"] <= _number(config["kDebounceMs"]) <= limits["debounce_ms_max"]
    assert config["kTb6600EnaLevelEnabled"] == "true" and electrical["tb6600"]["ena_level_driver_enabled"] == "HIGH"
    assert config["kTb6600EnaLevelDisabled"] == "false" and electrical["tb6600"]["ena_level_driver_disabled"] == "LOW"


def test_pwm_channels_do_not_collide(config: dict[str, str]) -> None:
    """Arduino-ESP32 2.x: LEDC channel c uses timer (c // 2) % 4. The two PWM outputs need different timers."""
    z, servo = int(_number(config["kZMotorLedcChannel"])), int(_number(config["kServoLedcChannel"]))
    assert z != servo
    assert (z // 2) % 4 != (servo // 2) % 4


def test_pins_h_matches_hardware_yaml(pins: dict[str, str], hardware: dict) -> None:
    yaml_pins = hardware["pins"]
    expected = {
        "kTb6600Ena1": yaml_pins["joint1"]["ena"],
        "kTb6600Dir1": yaml_pins["joint1"]["dir"],
        "kTb6600Pul1": yaml_pins["joint1"]["pul"],
        "kTb6600Ena2": yaml_pins["joint2"]["ena"],
        "kTb6600Dir2": yaml_pins["joint2"]["dir"],
        "kTb6600Pul2": yaml_pins["joint2"]["pul"],
        "kL298Ena": yaml_pins["z_motor"]["pwm"],
        "kL298In1": yaml_pins["z_motor"]["in1_raise"],
        "kL298In2": yaml_pins["z_motor"]["in2_lower"],
        "kI2cSda": yaml_pins["i2c"]["sda"],
        "kI2cScl": yaml_pins["i2c"]["scl"],
        "kServoPwm": yaml_pins["servo_pwm"],
        "kLimitTop": yaml_pins["limit_top"],
        "kLimitBottom": yaml_pins["limit_bottom"],
        "kAds1115I2cAddress": hardware["electrical"]["ads1115"]["address"],
    }
    assert {name: int(_number(pins[name])) if not pins[name].startswith("0x") else int(pins[name], 16) for name in expected} == expected
    assert set(pins) == set(expected), "pins.h has a constant that hardware.yaml does not know about (or the reverse)"


def test_pins_h_matches_the_spec_pin_table(pins: dict[str, str]) -> None:
    """SPEC 4.2, parsed from the SPEC text: pin number by 'Connected to'."""
    text = SPEC.read_text(encoding="utf-8")
    section = re.search(r"### 4\.2 ESP32 pin map(.*?)^### ", text, re.S | re.M)
    assert section
    table: dict[str, list[int]] = {}
    for row in section.group(1).splitlines():
        cells = [c.strip() for c in row.strip().strip("|").split("|")]
        if len(cells) >= 2 and re.fullmatch(r"D\d+( / D\d+)?", cells[0]):
            table[cells[1]] = [int(n) for n in re.findall(r"\d+", cells[0])]
    by_constant = {
        "TB6600 #1 ENA-": "kTb6600Ena1",
        "TB6600 #1 DIR-": "kTb6600Dir1",
        "TB6600 #1 PUL-": "kTb6600Pul1",
        "TB6600 #2 ENA-": "kTb6600Ena2",
        "TB6600 #2 DIR-": "kTb6600Dir2",
        "TB6600 #2 PUL-": "kTb6600Pul2",
        "L298 ENA": "kL298Ena",
        "L298 IN1": "kL298In1",
        "L298 IN2": "kL298In2",
        "SG90 PWM": "kServoPwm",
        "Limit switch #1 (TOP)": "kLimitTop",
        "Limit switch #2 (BOTTOM)": "kLimitBottom",
    }
    assert set(by_constant) <= set(table), set(by_constant) - set(table)
    for connected_to, constant in by_constant.items():
        assert table[connected_to] == [int(pins[constant])], connected_to
    assert table["ADS1115 SDA / SCL"] == [int(pins["kI2cSda"]), int(pins["kI2cScl"])]


# --------------------------------------------------------------------------- #
# Layering and hard-coded values
# --------------------------------------------------------------------------- #

FORBIDDEN_IN_PORTABLE_CODE = re.compile(r"#\s*include\s*[<\"](?:Arduino|esp_|freertos|driver/|soc/|Wire|HardwareSerial|FastAccelStepper|ESP32Servo)", re.I)


@pytest.mark.parametrize("path", [*CORE_FILES, CONFIG_H, PINS_H, PROTOCOL_H], ids=lambda p: p.name)
def test_portable_code_includes_nothing_from_arduino_or_the_sdk(path: Path) -> None:
    assert not FORBIDDEN_IN_PORTABLE_CODE.search(path.read_text(encoding="utf-8")), f"{path.name} includes a platform header"
    assert not re.search(r"\bSerial\d?\s*\.", _code_only(path.read_text(encoding="utf-8"))), f"{path.name} uses the Serial object"


def test_the_core_is_not_empty() -> None:
    names = {p.name for p in CORE_FILES}
    for expected in ("line_framer.cpp", "command_parser.cpp", "response_format.cpp", "command_queue.cpp", "device_state.h", "command_processor.cpp"):
        assert expected in names


def _decimal_literals(text: str) -> list[str]:
    code = re.sub(r'"(?:\\.|[^"\\])*"', '""', _code_only(text))
    code = re.sub(r"'(?:\\.|[^'\\])+'", "''", code)
    return re.findall(r"(?<![\w.])\d+(?![\w.])", code)


def test_main_cpp_has_no_number_of_its_own() -> None:
    """Every value comes from config.h / pins.h. Only 0 (a comparison) and 1 are allowed."""
    literals = _decimal_literals(MAIN_CPP.read_text(encoding="utf-8"))
    assert set(literals) <= {"0"}, f"main.cpp hard-codes {sorted(set(literals) - {'0'})}"
    assert "0x" not in _code_only(MAIN_CPP.read_text(encoding="utf-8"))


def test_the_core_does_not_repeat_a_config_or_pin_value(config: dict[str, str], pins: dict[str, str]) -> None:
    """A literal equal to a pin number or to a config value of 3+ digits in the core is a copy."""
    values = {str(int(_number(v))) for v in pins.values() if re.fullmatch(r"\d+", v)}
    values |= {str(int(_number(v))) for v in config.values() if re.fullmatch(r"\d{3,}", v)}
    for path in CORE_FILES:
        copies = sorted(set(_decimal_literals(path.read_text(encoding="utf-8"))) & values)
        assert copies == [], f"{path.name} repeats a value of config.h / pins.h: {copies}"


def test_setup_puts_the_pin_levels_first_and_serial_after() -> None:
    """SPEC 10.2 Boot: the ENA pins are configured at the very start of setup()."""
    code = _code_only(MAIN_CPP.read_text(encoding="utf-8"))
    body = re.search(r"void\s+setup\s*\(\s*\)\s*\{(.*?)\n\}", code, re.S)
    assert body, "setup() not found"
    statements = [s.strip() for s in body.group(1).split(";") if s.strip()]
    assert statements[0] == "hold_boot_levels()"
    assert next(i for i, s in enumerate(statements) if "Serial." in s) > 0


def test_the_boot_function_drives_both_tb6600_ena_pins_and_the_three_z_pins() -> None:
    code = _code_only(MAIN_CPP.read_text(encoding="utf-8"))
    boot = re.search(r"void\s+hold_boot_levels\s*\(\s*\)\s*\{(.*?)\n\}", code, re.S)
    assert boot
    assert "write_drivers_enable(true)" in boot.group(1) and "drive_z_motor_pins_idle()" in boot.group(1)
    z = re.search(r"void\s+drive_z_motor_pins_idle\s*\(\s*\)\s*\{(.*?)\n\}", code, re.S)
    assert z
    for pin in ("kL298Ena", "kL298In1", "kL298In2"):
        assert pin in z.group(1)
    enable = re.search(r"void\s+write_drivers_enable\s*\([^)]*\)\s*\{(.*?)\n\}", code, re.S)
    assert enable and "kTb6600Ena1" in enable.group(1) and "kTb6600Ena2" in enable.group(1)


def test_stage_4a_touches_no_motion_pin() -> None:
    """No stepper, Z PWM, servo, I2C or limit-switch pin is used by main.cpp in stage 4a."""
    code = _code_only(MAIN_CPP.read_text(encoding="utf-8"))
    for name in ("kTb6600Dir1", "kTb6600Pul1", "kTb6600Dir2", "kTb6600Pul2", "kServoPwm", "kI2cSda", "kI2cScl", "kLimitTop", "kLimitBottom"):
        assert name not in code, name
    for call in ("ledcSetup", "ledcAttach", "ledcWrite", "Wire.", "attachInterrupt", "analogWrite", "tone("):
        assert call not in code, call
