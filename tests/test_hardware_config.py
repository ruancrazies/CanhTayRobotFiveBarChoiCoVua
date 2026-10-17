"""Tests for the typed hardware.yaml loader (SPEC section 4 constants)."""

from __future__ import annotations

import dataclasses
import logging
from pathlib import Path
from typing import Any, Callable

import pytest
import yaml

from desktop_app.utils.hardware_config import (
    DEFAULT_CONFIG_PATH,
    ConfigError,
    HardwareConfig,
    load_hardware_config,
)


@pytest.fixture
def cfg() -> HardwareConfig:
    return load_hardware_config()


def _write(tmp_path: Path, data: Any) -> Path:
    path = tmp_path / "hardware.yaml"
    path.write_text(yaml.safe_dump(data), encoding="utf-8")
    return path


def _set(data: dict[str, Any], dotted: str, value: Any) -> None:
    *parents, leaf = dotted.split(".")
    node = data
    for part in parents:
        node = node[part]
    node[leaf] = value


def _delete(data: dict[str, Any], dotted: str) -> None:
    *parents, leaf = dotted.split(".")
    node = data
    for part in parents:
        node = node[part]
    del node[leaf]


# --------------------------------------------------------------------------- #
# Section 4.1 geometry and motion
# --------------------------------------------------------------------------- #


def test_default_config_file_exists_and_loads(cfg: HardwareConfig) -> None:
    assert DEFAULT_CONFIG_PATH.is_file()
    assert isinstance(cfg, HardwareConfig)


def test_config_is_immutable(cfg: HardwareConfig) -> None:
    with pytest.raises(dataclasses.FrozenInstanceError):
        cfg.geometry.square_size_mm = 40.0  # type: ignore[misc]


def test_geometry_matches_spec_table(cfg: HardwareConfig) -> None:
    g = cfg.geometry
    assert g.motor_axis_distance_mm == 100.0  # l
    assert g.driving_link_mm == 200.0  # m
    assert g.driven_link_mm == 250.0  # n
    assert g.motor_to_board_edge_mm == 150.0  # k
    assert g.square_size_mm == 31.0  # s
    assert g.board_border_mm == 26.0
    assert g.squares_per_side == 8
    assert g.z_travel_mm == 70.0  # h


def test_drive_matches_spec_table(cfg: HardwareConfig) -> None:
    d = cfg.drive
    assert d.full_steps_per_rev == 200
    assert d.microstepping == 16
    assert d.gear_ratio == 1.0
    assert d.pulses_per_rev == 3200
    assert d.joint_resolution_deg == pytest.approx(0.1125)


def test_derived_values_follow_the_gear_ratio(hardware_yaml_data: dict[str, Any], tmp_path: Path) -> None:
    _set(hardware_yaml_data, "drive.gear_ratio", 2)
    drive = load_hardware_config(_write(tmp_path, hardware_yaml_data)).drive
    assert drive.gear_ratio == 2.0
    assert drive.pulses_per_rev == 6400
    assert drive.joint_resolution_deg == pytest.approx(0.05625)


def test_coordinate_frame(cfg: HardwareConfig) -> None:
    g = cfg.geometry
    assert g.motor1_xy_mm == (-50.0, 0.0)
    assert g.motor2_xy_mm == (50.0, 0.0)
    assert g.board_size_mm == 300.0
    assert g.board_center_xy_mm == (0.0, 300.0)
    assert g.first_square_center_xy_mm == (-108.5, 191.5)


# --------------------------------------------------------------------------- #
# Section 4.2 pin map
# --------------------------------------------------------------------------- #


def test_pin_map_matches_spec_table(cfg: HardwareConfig) -> None:
    assert cfg.pins.all_pins() == {
        "joint1.ena": 13,
        "joint1.dir": 14,
        "joint1.pul": 27,
        "joint2.ena": 26,
        "joint2.dir": 25,
        "joint2.pul": 33,
        "z_motor.pwm": 4,
        "z_motor.in1_raise": 18,
        "z_motor.in2_lower": 19,
        "i2c.sda": 21,
        "i2c.scl": 22,
        "servo_pwm": 32,
        "limit_top": 35,
        "limit_bottom": 34,
    }


# --------------------------------------------------------------------------- #
# Section 4.3 electrical conventions
# --------------------------------------------------------------------------- #


def test_tb6600_conventions(cfg: HardwareConfig) -> None:
    t = cfg.electrical.tb6600
    assert t.wiring == "common_anode"
    assert (t.microstep_dip.sw1_on, t.microstep_dip.sw2_on, t.microstep_dip.sw3_on) == (False, False, True)
    assert t.ena_level_driver_enabled == "HIGH"
    assert t.ena_level_driver_disabled == "LOW"
    assert t.low_active_enables_stepper is False
    assert t.auto_enable is False


def test_limit_switch_conventions(cfg: HardwareConfig) -> None:
    ls = cfg.electrical.limit_switches
    assert ls.contact == "NC"
    assert ls.com_connected_to == "3V3"
    assert ls.external_pulldown_ohm == 10_000
    assert (ls.idle_level, ls.actuated_level) == ("HIGH", "LOW")
    assert (ls.debounce_ms_min, ls.debounce_ms_max) == (5, 10)


def test_ads1115_and_as5600_conventions(cfg: HardwareConfig) -> None:
    adc = cfg.electrical.ads1115
    assert adc.address == 0x48
    assert adc.addr_pin_tied_to == "GND"
    assert adc.pga_full_scale_v == 4.096

    enc = cfg.electrical.as5600
    assert enc.output_mode == "analog"
    assert enc.outs_register == 0
    assert (enc.output_min_percent_vdd, enc.output_max_percent_vdd) == (0.0, 100.0)
    assert enc.joint2_beta_adc_channel == 0  # A0 = AS5600 #2 (beta)
    assert enc.joint1_alpha_adc_channel == 1  # A1 = AS5600 #1 (alpha)
    assert enc.vdd_adc_channel == 2  # A2 = Vdd
    assert (enc.r_fault_low, enc.r_fault_high) == (0.02, 0.98)
    assert enc.system_error_deg == 1.0
    assert enc.calibration_file == "data/calibration/encoder_calibration.json"


def test_z_axis_and_servo_conventions(cfg: HardwareConfig) -> None:
    z = cfg.electrical.z_axis
    assert z.driver == "L298"
    assert z.pwm_frequency_hz == 10_000
    assert z.timeout_factor == 1.5

    servo = cfg.electrical.servo
    assert servo.model == "SG90"
    assert servo.pwm_frequency_hz == 50
    assert (servo.pulse_us_at_0_deg, servo.pulse_us_at_180_deg) == (500, 2500)
    assert servo.separate_5v_supply is True
    assert servo.common_gnd_with_esp32 is True
    assert servo.separate_ledc_channel_and_timer is True


# --------------------------------------------------------------------------- #
# [TO CONFIRM] handling
# --------------------------------------------------------------------------- #


def test_gear_ratio_is_flagged_as_to_confirm(cfg: HardwareConfig) -> None:
    assert cfg.to_confirm == ("drive.gear_ratio",)


def test_loading_warns_about_each_to_confirm_value(caplog: pytest.LogCaptureFixture) -> None:
    with caplog.at_level(logging.WARNING, logger="desktop_app.utils.hardware_config"):
        load_hardware_config()
    warnings = [r.getMessage() for r in caplog.records if r.levelno == logging.WARNING]
    assert any("drive.gear_ratio" in message and "TO CONFIRM" in message for message in warnings)


def test_to_confirm_entry_must_point_at_a_real_value(hardware_yaml_data: dict[str, Any], tmp_path: Path) -> None:
    hardware_yaml_data["to_confirm"] = ["drive.no_such_value"]
    with pytest.raises(ConfigError, match="drive.no_such_value"):
        load_hardware_config(_write(tmp_path, hardware_yaml_data))


# --------------------------------------------------------------------------- #
# Validation failures
# --------------------------------------------------------------------------- #

Mutation = Callable[[dict[str, Any]], None]

BAD_CONFIGS: list[tuple[str, Mutation, str]] = [
    ("missing key", lambda d: _delete(d, "geometry.square_size_mm"), "missing key.*square_size_mm"),
    ("unknown key", lambda d: _set(d, "geometry.extra_value", 1), "unknown key.*extra_value"),
    ("missing section", lambda d: _delete(d, "pins"), "missing key.*pins"),
    ("string for a float", lambda d: _set(d, "geometry.driving_link_mm", "long"), "geometry.driving_link_mm"),
    ("bool for an int", lambda d: _set(d, "drive.microstepping", True), "drive.microstepping"),
    ("null value", lambda d: _set(d, "drive.gear_ratio", None), "drive.gear_ratio"),
    ("negative length", lambda d: _set(d, "geometry.driven_link_mm", -1.0), "driven_link_mm must be > 0"),
    ("zero gear ratio", lambda d: _set(d, "drive.gear_ratio", 0), "gear_ratio must be > 0"),
    ("duplicate GPIO", lambda d: _set(d, "pins.servo_pwm", 13), "GPIO 13 is assigned twice"),
    ("GPIO out of range", lambda d: _set(d, "pins.servo_pwm", 99), "outside 0..39"),
    ("output on input-only GPIO", lambda d: _set(d, "pins.servo_pwm", 36), "input-only"),
    ("wrong TB6600 wiring", lambda d: _set(d, "electrical.tb6600.wiring", "common_cathode"), "common_anode"),
    ("equal ENA levels", lambda d: _set(d, "electrical.tb6600.ena_level_driver_disabled", "HIGH"), "must differ"),
    ("normally-open limit switch", lambda d: _set(d, "electrical.limit_switches.contact", "NO"), "'NC'"),
    ("limit switch levels swapped", lambda d: _set(d, "electrical.limit_switches.actuated_level", "HIGH"), "HIGH when idle"),
    ("debounce range reversed", lambda d: _set(d, "electrical.limit_switches.debounce_ms_min", 20), "debounce_ms_min"),
    ("invalid ADS1115 address", lambda d: _set(d, "electrical.ads1115.address", 0x10), "not a valid ADS1115 address"),
    ("duplicate ADC channel", lambda d: _set(d, "electrical.as5600.vdd_adc_channel", 0), "must be distinct"),
    ("fault thresholds reversed", lambda d: _set(d, "electrical.as5600.r_fault_low", 0.99), "r_fault_low"),
    ("Z timeout shorter than travel", lambda d: _set(d, "electrical.z_axis.timeout_factor", 0.5), "timeout_factor"),
    ("servo pulses reversed", lambda d: _set(d, "electrical.servo.pulse_us_at_0_deg", 3000), "pulse_us_at_0_deg"),
]


@pytest.mark.parametrize(
    ("mutate", "pattern"),
    [pytest.param(mutate, pattern, id=name) for name, mutate, pattern in BAD_CONFIGS],
)
def test_invalid_config_is_rejected(
    hardware_yaml_data: dict[str, Any], tmp_path: Path, mutate: Mutation, pattern: str
) -> None:
    mutate(hardware_yaml_data)
    with pytest.raises(ConfigError, match=pattern):
        load_hardware_config(_write(tmp_path, hardware_yaml_data))


def test_missing_file_is_a_config_error(tmp_path: Path) -> None:
    with pytest.raises(ConfigError, match="cannot read"):
        load_hardware_config(tmp_path / "nope.yaml")


def test_malformed_yaml_is_a_config_error(tmp_path: Path) -> None:
    path = tmp_path / "hardware.yaml"
    path.write_text("geometry: [unclosed", encoding="utf-8")
    with pytest.raises(ConfigError, match="not valid YAML"):
        load_hardware_config(path)


def test_top_level_must_be_a_mapping(tmp_path: Path) -> None:
    with pytest.raises(ConfigError, match="expected a mapping"):
        load_hardware_config(_write(tmp_path, ["not", "a", "mapping"]))
