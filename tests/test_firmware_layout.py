"""Guards for the firmware project layout (phase 4a).

These tests read the firmware files as text. They do not compile anything; the C++ tests run with
``pio test -e native`` or ``firmware/tools/run_host_tests.sh``.

* platformio.ini: every version is pinned exactly, and the monitor baud equals ``serial.baud`` of
  config/settings.yaml (and so ``kBaudRate``).
* config.h / pins.h are generated from config/settings.yaml (tests/test_generated_files.py checks they are
  current); here they are checked against the values and against docs/SPEC.md section 4.2. Every value that
  settings.yaml lists under ``assumed`` carries an ``ASSUMED`` marker in the header.
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
SETTINGS_YAML = REPO_ROOT / "config" / "settings.yaml"

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
def settings() -> dict:
    return yaml.safe_load(SETTINGS_YAML.read_text(encoding="utf-8"))


# --------------------------------------------------------------------------- #
# platformio.ini
# --------------------------------------------------------------------------- #


def test_the_two_environments_exist(ini: configparser.ConfigParser) -> None:
    assert "env:esp32dev" in ini and "env:native" in ini
    assert ini["env:esp32dev"]["board"] == "esp32dev"
    assert ini["env:esp32dev"]["framework"] == "arduino"


def test_monitor_baud_equals_the_baud_of_settings_yaml(
    ini: configparser.ConfigParser, config: dict[str, str], settings: dict
) -> None:
    """platformio.ini cannot read settings.yaml, so its monitor_speed is a copy that this test keeps honest."""
    assert int(ini["env:esp32dev"]["monitor_speed"]) == settings["serial"]["baud"] == int(_number(config["kBaudRate"]))


def test_the_baud_is_not_repeated_anywhere_else_in_platformio_ini(ini: configparser.ConfigParser) -> None:
    text = INI.read_text(encoding="utf-8")
    assert len(re.findall(r"\bmonitor_speed\b", text)) == 1 and "upload_speed" not in text


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
# config.h and pins.h against settings.yaml and the SPEC
# --------------------------------------------------------------------------- #

def _constant_of_each_key() -> dict[str, str]:
    from tools.gen_firmware_config import firmware_constants

    return firmware_constants()


def _assumed_keys() -> list[str]:
    raw = yaml.safe_load(SETTINGS_YAML.read_text(encoding="utf-8"))
    return [key for key in raw["assumed"] if key in _constant_of_each_key()]


def test_some_firmware_constants_are_assumed() -> None:
    assert {"serial.baud", "serial.queue_depth", "timeouts.heartbeat_timeout_ms"} <= set(_assumed_keys())


@pytest.mark.parametrize("key", _assumed_keys())
def test_every_assumed_value_is_marked_assumed_in_the_header(key: str) -> None:
    name = _constant_of_each_key()[key]
    header = PINS_H if name in _constants(PINS_H) else CONFIG_H
    lines = header.read_text(encoding="utf-8").splitlines()
    index = next(i for i, line in enumerate(lines) if re.search(rf"\b{name}\b\s*=", line))
    window = "\n".join(lines[max(0, index - 4) : index + 1])
    assert "ASSUMED" in window, f"{name} ({key}) is in `assumed` but its header comment does not say so"


def test_unset_values_are_zero_so_they_cannot_be_used_by_accident(config: dict[str, str], settings: dict) -> None:
    """A null in settings.yaml becomes 0 in the header; the stage that needs it must refuse to run until it is set."""
    by_key = _constant_of_each_key()
    nulls = [key for key in by_key if _lookup(settings, key) is None]
    for key in nulls:  # whatever is null in the committed file (nothing is, after stage 4b-1)
        assert _number(config[by_key[key]]) == 0, key


def test_a_null_in_settings_yaml_becomes_zero_in_config_h(tmp_path: Path) -> None:
    """The four values stage 4b-1 filled in are written as 0 again when they are set back to null."""
    from tools.gen_firmware_config import render_config_h

    data = yaml.safe_load(SETTINGS_YAML.read_text(encoding="utf-8"))
    data["electrical"]["z_axis"]["full_travel_ms"] = None
    data["motion"]["max_speed_deg_s"] = None
    data["motion"]["max_accel_deg_s2"] = None
    data["motion"]["direction_change_delay_us"] = None
    for name in data["motion"]["joint_limits"]:
        data["motion"]["joint_limits"][name] = None
    path = tmp_path / "settings.yaml"
    path.write_text(yaml.safe_dump(data, sort_keys=False), encoding="utf-8")
    text = render_config_h(path)
    values = {m.group(1): m.group(2) for m in re.finditer(r"constexpr \w+ (\w+) = ([^;]+);", text)}
    for name in ("kZFullTravelMs", "kMaxSpeedDegPerS", "kMaxAccelDegPerS2", "kDirectionChangeDelayUs", "kJointAlphaMinDeg", "kJointBetaMaxDeg"):
        assert _number(values[name]) == 0, name
    assert values["kJointLimitsConfigured"] == "false"


def _lookup(data: dict, dotted: str) -> object:
    node: object = data
    for part in dotted.split("."):
        node = node[part]  # type: ignore[index]
    return node


def test_config_h_agrees_with_settings_yaml(config: dict[str, str], settings: dict) -> None:
    electrical = settings["electrical"]
    assert _number(config["kZMotorPwmHz"]) == electrical["z_axis"]["pwm_frequency_hz"]
    assert _number(config["kZTimeoutFactor"]) == electrical["z_axis"]["timeout_factor"]
    assert _number(config["kServoPwmHz"]) == electrical["servo"]["pwm_frequency_hz"]
    assert _number(config["kServoPulseMinUs"]) == electrical["servo"]["pulse_at_0_deg_us"]
    assert _number(config["kServoPulseMaxUs"]) == electrical["servo"]["pulse_at_180_deg_us"]
    assert _number(config["kSensorRMin"]) == electrical["as5600"]["r_fault_low"]
    assert _number(config["kSensorRMax"]) == electrical["as5600"]["r_fault_high"]
    limits = electrical["limit_switches"]
    assert limits["debounce_min_ms"] <= _number(config["kDebounceMs"]) <= limits["debounce_max_ms"]
    assert config["kTb6600EnaLevelEnabled"] == "true" and electrical["tb6600"]["ena_level_driver_enabled"] == "HIGH"
    assert config["kTb6600EnaLevelDisabled"] == "false" and electrical["tb6600"]["ena_level_driver_disabled"] == "LOW"


def test_pwm_channels_do_not_collide(config: dict[str, str]) -> None:
    """Arduino-ESP32 2.x: LEDC channel c uses timer (c // 2) % 4. The two PWM outputs need different timers."""
    z, servo = int(_number(config["kZMotorLedcChannel"])), int(_number(config["kServoLedcChannel"]))
    assert z != servo
    assert (z // 2) % 4 != (servo // 2) % 4


def test_pins_h_matches_settings_yaml(pins: dict[str, str], settings: dict) -> None:
    yaml_pins = settings["pins"]
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
        "kAds1115I2cAddress": settings["electrical"]["ads1115"]["address"],
    }
    assert {name: int(_number(pins[name])) if not pins[name].startswith("0x") else int(pins[name], 16) for name in expected} == expected
    assert set(pins) == set(expected), "pins.h has a constant that settings.yaml does not know about (or the reverse)"


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
    """Every value comes from the generated config.h / pins.h. Only 0 (a comparison) and 1 are allowed."""
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


def test_stage_4b1_s1_touches_only_the_joint_stepper_pins() -> None:
    """Stage 4b-1 S1 drives the joint STEP/DIR pins (through FastAccelStepper) and nothing else of the motion hardware."""
    code = _code_only(MAIN_CPP.read_text(encoding="utf-8"))
    for name in ("kServoPwm", "kI2cSda", "kI2cScl", "kLimitTop", "kLimitBottom"):
        assert name not in code, name
    for call in ("ledcSetup", "ledcAttach", "ledcWrite", "Wire.", "attachInterrupt", "analogWrite", "tone("):
        assert call not in code, call
    # The joint driver ENA pins stay under the firmware's own drive_pin(): FastAccelStepper::setEnablePin() would
    # drive them at once and bypass the ENABLE/DISABLE/ESTOP rules.
    assert "setEnablePin" not in code
    assert "setAutoEnable(false)" in code


# --------------------------------------------------------------------------- #
# Include paths (phase 4a fixes)
# --------------------------------------------------------------------------- #

STUB = FIRMWARE / "tools" / "host_main" / "Arduino.h"
INCLUDE_PATH_SPELLINGS = {"include", "${platformio.include_dir}", "$PROJECT_INCLUDE_DIR"}


def _include_dirs(ini: configparser.ConfigParser, section: str) -> list[str]:
    """The ``-I`` directories of an environment's build_flags, normalised (``-Iinclude`` and ``-I include``)."""
    tokens = ini[section].get("build_flags", "").split()
    dirs: list[str] = []
    for index, token in enumerate(tokens):
        if token == "-I" and index + 1 < len(tokens):
            dirs.append(tokens[index + 1])
        elif token.startswith("-I"):
            dirs.append(token[2:])
    return [d.removeprefix("./").rstrip("/") for d in dirs]


@pytest.mark.parametrize("section", ["env:esp32dev", "env:native"])
def test_platformio_ini_puts_the_project_include_directory_on_the_include_path(ini: configparser.ConfigParser, section: str) -> None:
    dirs = _include_dirs(ini, section)
    assert set(dirs) & INCLUDE_PATH_SPELLINGS, f"{section} has no -I for the project include/ directory (build_flags -I: {dirs})"
    assert (FIRMWARE / "include").is_dir()


def test_both_environments_use_the_same_include_path(ini: configparser.ConfigParser) -> None:
    assert sorted(_include_dirs(ini, "env:esp32dev")) == sorted(_include_dirs(ini, "env:native"))


def test_the_include_path_check_itself_fails_without_the_flag() -> None:
    bare = configparser.ConfigParser(interpolation=None)
    bare.read_string("[env:esp32dev]\nbuild_flags =\n    -std=gnu++17\n    -Wall\n")
    assert not set(_include_dirs(bare, "env:esp32dev")) & INCLUDE_PATH_SPELLINGS
    spaced = configparser.ConfigParser(interpolation=None)
    spaced.read_string("[env:esp32dev]\nbuild_flags = -I include -I./other/\n")
    assert _include_dirs(spaced, "env:esp32dev") == ["include", "other"]


QUOTED_INCLUDE = re.compile(r'^\s*#\s*include\s*"([^"]+)"', re.M)


@pytest.mark.parametrize("path", [*CORE_FILES, MAIN_CPP], ids=lambda p: p.name)
def test_every_quoted_include_of_the_core_and_main_resolves_next_to_the_including_file(path: Path) -> None:
    """PlatformIO gives libraries no path to include/, and appends project -I flags after the SDK's, so a
    bare ``#include "config.h"`` fails to build or finds an SDK file. Each include must resolve relative
    to the file that contains it (the first place a quoted include is looked up)."""
    for name in QUOTED_INCLUDE.findall(_code_only_keep_includes(path.read_text(encoding="utf-8"))):
        found_next_to_file = (path.parent / name).is_file()
        # main.cpp (in src/) reaches the library through PlatformIO's library finder, which adds lib/chessbot_core/src
        # to the path of the project sources; the library's own header names are unique to the project.
        found_in_own_library = path == MAIN_CPP and (CORE_DIR / name).is_file()
        assert found_next_to_file or found_in_own_library, f"{path.name}: #include \"{name}\" is not found next to the file (it would depend on -I)"


def _code_only_keep_includes(text: str) -> str:
    return re.sub(r"/\*.*?\*/", "", text, flags=re.S)


def test_the_path_relative_include_check_would_catch_a_bare_include(tmp_path: Path) -> None:
    (tmp_path / "bad.h").write_text('#include "config.h"\n', encoding="utf-8")
    names = QUOTED_INCLUDE.findall((tmp_path / "bad.h").read_text(encoding="utf-8"))
    assert names == ["config.h"] and not (tmp_path / names[0]).is_file()


def test_the_bridge_header_reaches_the_project_headers() -> None:
    bridge = CORE_DIR / "project_headers.h"
    assert bridge.is_file()
    resolved = {(bridge.parent / name).resolve() for name in QUOTED_INCLUDE.findall(bridge.read_text(encoding="utf-8"))}
    assert resolved == {CONFIG_H.resolve(), PROTOCOL_H.resolve()}


def test_no_core_file_includes_a_project_header_other_than_through_the_bridge() -> None:
    for path in CORE_FILES:
        if path.name == "project_headers.h":
            continue
        for name in QUOTED_INCLUDE.findall(path.read_text(encoding="utf-8")):
            assert "include/" not in name and name not in {"config.h", "pins.h", "protocol.h"}, f"{path.name}: #include \"{name}\""


# --------------------------------------------------------------------------- #
# Names that Arduino.h defines
# --------------------------------------------------------------------------- #

# Required members of the stub's list (the names a collision is most likely with). The stub may define more.
REQUIRED_ARDUINO_NAMES = {
    "bit", "min", "max", "abs", "round", "sq", "constrain", "radians", "degrees", "word", "byte", "boolean",
    "HIGH", "LOW", "INPUT", "OUTPUT", "DEFAULT", "B0", "B1", "B2", "B3", "B4", "B5", "B6", "B7", "PI", "F",
    "SDA", "SCL", "SS", "TX", "RX", "A0", "T0",
}
# main.cpp is the one file allowed to use the Arduino API itself.
MAIN_MAY_USE = {"HIGH", "LOW", "OUTPUT"}


def _stub_names() -> tuple[set[str], set[str]]:
    """(object-like macros and global names, function-like macros) defined by the Arduino stub."""
    text = STUB.read_text(encoding="utf-8")
    objects = set(re.findall(r"^#define\s+(\w+)(?=\s|$)", text, re.M))
    functions = set(re.findall(r"^#define\s+(\w+)\(", text, re.M))
    objects |= set(re.findall(r"^static const \w+ (\w+) =", text, re.M))
    objects |= set(re.findall(r"^typedef [\w ]+ (\w+);", text, re.M))
    objects -= {"HOST_ARDUINO_STUB_H"}
    return objects, functions


def _strip_literals(code: str) -> str:
    code = re.sub(r'"(?:\\.|[^"\\])*"', '""', code)
    return re.sub(r"'(?:\\.|[^'\\])+'", "''", code)


def test_the_arduino_stub_defines_the_names_that_matter() -> None:
    objects, functions = _stub_names()
    assert REQUIRED_ARDUINO_NAMES <= objects | functions, sorted(REQUIRED_ARDUINO_NAMES - (objects | functions))
    assert {"bit", "min", "max", "abs", "round", "sq", "constrain", "radians", "degrees", "word", "F"} <= functions


def _collisions(code: str, allowed: set[str] = frozenset()) -> list[str]:  # type: ignore[assignment]
    objects, functions = _stub_names()
    code = _strip_literals(_code_only(code))
    found = {m for m in re.findall(r"\b[A-Za-z_]\w*\b", code) if m in objects and m not in allowed}
    found |= {m for m in functions if re.search(rf"\b{m}\s*\(", code) and m not in allowed}
    return sorted(found)


@pytest.mark.parametrize("path", [*CORE_FILES, CONFIG_H, PINS_H, PROTOCOL_H, MAIN_CPP], ids=lambda p: p.name)
def test_no_identifier_collides_with_an_arduino_macro_or_global(path: Path) -> None:
    allowed = MAIN_MAY_USE if path == MAIN_CPP else set()
    assert _collisions(path.read_text(encoding="utf-8"), allowed) == [], f"{path.name} uses a name Arduino.h defines"


def test_the_collision_scan_would_catch_the_bit_function_and_a_byte_parameter() -> None:
    assert _collisions("static uint8_t bit(protocol::AlarmCode alarm) { return 1; }") == ["bit"]
    assert _collisions("Result feed(uint8_t byte);") == ["byte"]
    assert _collisions("int x = max(a, b); const int PI = 3;") == ["PI", "max"]
    assert _collisions("X(MOVE, F, FLOAT, 1, 1)") == []  # F is only a macro when followed by "("
    assert _collisions('log("bit and byte"); // bit(3)') == []
    assert _collisions("int word(int a);") == ["word"]  # the real macro would rename it silently, so only a scan sees it
