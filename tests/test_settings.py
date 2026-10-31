"""Tests for the typed settings.yaml loader (SPEC section 4: every changeable value, in one file)."""

from __future__ import annotations

import dataclasses
from pathlib import Path
from typing import Any, Callable

import pytest
import yaml

from desktop_app.utils.settings import (
    DEFAULT_SETTINGS_PATH,
    STOCKFISH_PATH_ENV,
    ConfigError,
    PathSource,
    Settings,
    get_settings,
    load_settings,
    to_seconds,
)


@pytest.fixture
def cfg() -> Settings:
    return load_settings(overlay=False)


def _write(tmp_path: Path, data: Any, name: str = "settings.yaml") -> Path:
    path = tmp_path / name
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


def test_default_config_file_exists_and_loads(cfg: Settings) -> None:
    assert DEFAULT_SETTINGS_PATH.is_file()
    assert isinstance(cfg, Settings)


def test_config_is_immutable(cfg: Settings) -> None:
    with pytest.raises(dataclasses.FrozenInstanceError):
        cfg.geometry.square_size_mm = 40.0  # type: ignore[misc]


def test_geometry_matches_spec_table(cfg: Settings) -> None:
    g = cfg.geometry
    assert g.motor_axis_distance_mm == 100.0  # l
    assert g.driving_link_mm == 200.0  # m
    assert g.driven_link_mm == 250.0  # n
    assert g.motor_to_board_edge_mm == 150.0  # k
    assert g.square_size_mm == 31.0  # s
    assert g.board_border_mm == 26.0
    assert g.squares_per_side == 8
    assert g.z_travel_mm == 70.0  # h


def test_drive_matches_spec_table(cfg: Settings) -> None:
    d = cfg.drive
    assert d.full_steps_per_rev == 200
    assert d.microstepping == 16
    assert d.gear_ratio == 1.0
    assert d.pulses_per_rev == 3200
    assert d.joint_resolution_deg == pytest.approx(0.1125)


def test_derived_values_follow_the_gear_ratio(settings_yaml_data: dict[str, Any], tmp_path: Path) -> None:
    _set(settings_yaml_data, "drive.gear_ratio", 2)
    drive = load_settings(_write(tmp_path, settings_yaml_data)).drive
    assert drive.gear_ratio == 2.0
    assert drive.pulses_per_rev == 6400
    assert drive.joint_resolution_deg == pytest.approx(0.05625)


def test_coordinate_frame(cfg: Settings) -> None:
    g = cfg.geometry
    assert g.motor1_xy_mm == (-50.0, 0.0)
    assert g.motor2_xy_mm == (50.0, 0.0)
    assert g.board_size_mm == 300.0
    assert g.board_center_xy_mm == (0.0, 300.0)
    assert g.first_square_center_xy_mm == (-108.5, 191.5)


# --------------------------------------------------------------------------- #
# Section 4.2 pin map
# --------------------------------------------------------------------------- #


def test_pin_map_matches_spec_table(cfg: Settings) -> None:
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


def test_tb6600_conventions(cfg: Settings) -> None:
    t = cfg.electrical.tb6600
    assert t.wiring == "common_anode"
    assert (t.microstep_dip.sw1_on, t.microstep_dip.sw2_on, t.microstep_dip.sw3_on) == (False, False, True)
    assert t.ena_level_driver_enabled == "HIGH"
    assert t.ena_level_driver_disabled == "LOW"
    assert t.low_active_enables_stepper is False
    assert t.auto_enable is False


def test_limit_switch_conventions(cfg: Settings) -> None:
    ls = cfg.electrical.limit_switches
    assert ls.contact == "NC"
    assert ls.com_connected_to == "3V3"
    assert ls.external_pulldown_ohm == 10_000
    assert (ls.idle_level, ls.actuated_level) == ("HIGH", "LOW")
    assert (ls.debounce_min_ms, ls.debounce_max_ms) == (5, 10)


def test_ads1115_and_as5600_conventions(cfg: Settings) -> None:
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


def test_z_axis_and_servo_conventions(cfg: Settings) -> None:
    z = cfg.electrical.z_axis
    assert z.driver == "L298"
    assert z.pwm_frequency_hz == 10_000
    assert z.timeout_factor == 1.5

    servo = cfg.electrical.servo
    assert servo.model == "SG90"
    assert servo.pwm_frequency_hz == 50
    assert (servo.pulse_at_0_deg_us, servo.pulse_at_180_deg_us) == (500, 2500)
    assert servo.separate_5v_supply is True
    assert servo.common_gnd_with_esp32 is True
    assert servo.separate_ledc_channel_and_timer is True


# --------------------------------------------------------------------------- #
# `assumed` handling
# --------------------------------------------------------------------------- #


def test_gear_ratio_is_confirmed_direct_drive(cfg: Settings) -> None:
    assert cfg.drive.gear_ratio == 1.0
    assert not cfg.is_assumed("drive.gear_ratio")


def test_assumed_is_a_tuple_of_real_dotted_keys(cfg: Settings) -> None:
    assert isinstance(cfg.assumed, tuple) and cfg.assumed
    assert len(set(cfg.assumed)) == len(cfg.assumed)
    for dotted in cfg.assumed:
        node: Any = cfg
        for part in dotted.split("."):
            node = node[part] if isinstance(node, dict) else getattr(node, part)


def test_assumed_under_lists_the_keys_of_a_section(cfg: Settings) -> None:
    assert cfg.assumed_under("grasp") == tuple(k for k in cfg.assumed if k.startswith("grasp."))
    assert cfg.assumed_under("geometry") == ()


def test_assumed_entry_must_point_at_a_real_value(settings_yaml_data: dict[str, Any], tmp_path: Path) -> None:
    settings_yaml_data["assumed"] = ["drive.no_such_value"]
    with pytest.raises(ConfigError, match="drive.no_such_value"):
        load_settings(_write(tmp_path, settings_yaml_data), overlay=False)


def test_removing_a_key_from_assumed_confirms_it(settings_yaml_data: dict[str, Any], tmp_path: Path) -> None:
    settings_yaml_data["assumed"].remove("serial.baud")
    assert not load_settings(_write(tmp_path, settings_yaml_data), overlay=False).is_assumed("serial.baud")


def test_a_null_value_means_not_decided(cfg: Settings) -> None:
    # Still null after stage 4b-1: the Commander timeouts nobody has chosen, the per-machine port, the simulator times.
    assert cfg.commander.start_tolerance_deg is None
    assert cfg.commander.done_timeout_ms is None
    assert cfg.serial.port is None
    assert cfg.simulator.home_joints_ms is None


def test_stage_4b1_assumed_values_are_set_and_stay_assumed(cfg: Settings) -> None:
    # The values the owner chose for the bench (conservative; none is measured yet), all listed in `assumed`.
    assert (cfg.motion.max_speed_deg_s, cfg.motion.max_accel_deg_s2, cfg.motion.direction_change_delay_us) == (30.0, 60.0, 20)
    limits = cfg.motion.joint_limits
    assert (limits.alpha_min_deg, limits.alpha_max_deg, limits.beta_min_deg, limits.beta_max_deg) == (60.0, 200.0, -20.0, 115.0)
    z = cfg.electrical.z_axis
    assert (z.full_travel_ms, z.timeout_factor, z.pwm_frequency_hz) == (5000, 1.5, 10000)
    assert (z.duty_start_percent, z.duty_peak_percent, z.duty_ramp_ms) == (40, 70, 500)
    assert (z.duty_eased_percent, z.ease_after_travel_percent) == (40, 80)
    assert cfg.electrical.limit_switches.debounce_ms == 8
    assert (cfg.commander.move_speed_deg_s, cfg.commander.acceleration_deg_s2) == (20.0, 40.0)
    assert (cfg.commander.ping_interval_ms, cfg.commander.grip_settle_ms) == (500, 400)
    assert cfg.timeouts.heartbeat_timeout_ms == 2000 and cfg.serial.queue_depth == 8
    assert cfg.motion.bench_move_without_homing is False  # bench only: off unless the owner turns it on
    for key in (
        "motion.max_speed_deg_s", "motion.max_accel_deg_s2", "motion.direction_change_delay_us",
        "motion.joint_limits.alpha_min_deg", "motion.joint_limits.beta_max_deg", "electrical.z_axis.full_travel_ms",
        "electrical.z_axis.duty_peak_percent", "electrical.limit_switches.debounce_ms", "commander.move_speed_deg_s",
        "commander.ping_interval_ms", "electrical.tb6600.joint1_dir_high_counts_up",
    ):
        assert cfg.is_assumed(key), key


def test_milliseconds_convert_to_seconds_without_changing_the_value(cfg: Settings) -> None:
    assert to_seconds(200) == 0.2
    assert to_seconds(cfg.timeouts.heartbeat_timeout_ms) == 2.0


# --------------------------------------------------------------------------- #
# The local overlay and STOCKFISH_PATH
# --------------------------------------------------------------------------- #


def test_the_overlay_overrides_an_existing_key_and_leaves_the_rest(tmp_path: Path) -> None:
    local = _write(tmp_path, {"serial": {"port": "COM7"}, "engine": {"path": "/x/stockfish"}}, "local.yaml")
    merged = load_settings(local_path=local, environ={})
    plain = load_settings(overlay=False)
    assert (merged.serial.port, merged.engine.path) == ("COM7", "/x/stockfish")
    assert merged.serial.baud == plain.serial.baud and merged.geometry == plain.geometry and merged.assumed == plain.assumed
    assert merged.path_source is PathSource.LOCAL_FILE


def test_the_overlay_can_change_a_number_with_the_same_type_rules(tmp_path: Path) -> None:
    assert load_settings(local_path=_write(tmp_path, {"link": {"read_poll_ms": 10}}, "local.yaml"), environ={}).link.read_poll_ms == 10
    with pytest.raises(ConfigError, match="read_poll_ms"):
        load_settings(local_path=_write(tmp_path, {"link": {"read_poll_ms": "fast"}}, "local.yaml"), environ={})


@pytest.mark.parametrize(
    ("overlay", "pattern"),
    [
        pytest.param({"geometry": {"nope": 1}}, "unknown key", id="unknown key"),
        pytest.param({"nope": 1}, "unknown key", id="unknown section"),
        pytest.param({"serial": {"port": 5}}, "serial.port", id="wrong type"),
        pytest.param({"assumed": ["serial.baud"]}, "assumed", id="assumed cannot be overlaid"),
        pytest.param({"drive": {"gear_ratio": 0}}, "gear_ratio", id="the cross-field checks still run"),
    ],
)
def test_an_invalid_overlay_is_rejected_and_names_the_file(tmp_path: Path, overlay: Any, pattern: str) -> None:
    local = _write(tmp_path, overlay, "local.yaml")
    with pytest.raises(ConfigError, match=pattern) as info:
        load_settings(local_path=local, environ={})
    assert "local.yaml" in str(info.value)


def test_the_environment_beats_the_overlay(tmp_path: Path) -> None:
    local = _write(tmp_path, {"engine": {"path": "/local/stockfish"}}, "local.yaml")
    loaded = load_settings(local_path=local, environ={STOCKFISH_PATH_ENV: "/env/stockfish"})
    assert (loaded.engine.path, loaded.path_source) == ("/env/stockfish", PathSource.ENVIRONMENT)


def test_a_uci_option_mapping_is_replaced_whole_not_merged(tmp_path: Path) -> None:
    local = _write(tmp_path, {"engine": {"modes": {"beginner": {"options": {"Skill Level": 3}}}}}, "local.yaml")
    assert load_settings(local_path=local, environ={}).engine.modes.beginner.options == {"Skill Level": 3}


def test_overlay_false_ignores_the_overlay_and_the_environment(tmp_path: Path) -> None:
    local = _write(tmp_path, {"serial": {"port": "COM7"}}, "local.yaml")
    assert load_settings(local_path=local, environ={STOCKFISH_PATH_ENV: "/env"}, overlay=False).serial.port is None


def test_get_settings_is_cached_and_follows_the_overlay_path(tmp_path: Path, monkeypatch: pytest.MonkeyPatch) -> None:
    from desktop_app.utils import settings as module

    assert get_settings() is get_settings()
    local = _write(tmp_path, {"serial": {"port": "COM9"}}, "local.yaml")
    monkeypatch.setattr(module, "DEFAULT_LOCAL_PATH", local)
    assert get_settings().serial.port == "COM9"


def test_the_overlay_example_is_a_valid_overlay() -> None:
    example = DEFAULT_SETTINGS_PATH.parent / "local.example.yaml"
    assert load_settings(local_path=example, environ={}).path_source is PathSource.LOCAL_FILE


def test_the_old_config_sources_are_gone() -> None:
    root = DEFAULT_SETTINGS_PATH.parents[1]
    for gone in (
        "config/hardware.yaml",
        "config/difficulty_levels.json",
        "config/local.example.json",
        "desktop_app/utils/hardware_config.py",
        "firmware/ASSUMPTIONS.md",
    ):
        assert not (root / gone).exists(), f"{gone} must not exist: settings.yaml is the only source"


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
    ("debounce range reversed", lambda d: _set(d, "electrical.limit_switches.debounce_min_ms", 20), "debounce_min_ms"),
    ("invalid ADS1115 address", lambda d: _set(d, "electrical.ads1115.address", 0x10), "not a valid ADS1115 address"),
    ("duplicate ADC channel", lambda d: _set(d, "electrical.as5600.vdd_adc_channel", 0), "must be distinct"),
    ("fault thresholds reversed", lambda d: _set(d, "electrical.as5600.r_fault_low", 0.99), "r_fault_low"),
    ("Z timeout shorter than travel", lambda d: _set(d, "electrical.z_axis.timeout_factor", 0.5), "timeout_factor"),
    ("servo pulses reversed", lambda d: _set(d, "electrical.servo.pulse_at_0_deg_us", 3000), "pulse_at_0_deg_us"),
]


@pytest.mark.parametrize(
    ("mutate", "pattern"),
    [pytest.param(mutate, pattern, id=name) for name, mutate, pattern in BAD_CONFIGS],
)
def test_invalid_config_is_rejected(
    settings_yaml_data: dict[str, Any], tmp_path: Path, mutate: Mutation, pattern: str
) -> None:
    mutate(settings_yaml_data)
    with pytest.raises(ConfigError, match=pattern):
        load_settings(_write(tmp_path, settings_yaml_data))


def test_missing_file_is_a_config_error(tmp_path: Path) -> None:
    with pytest.raises(ConfigError, match="cannot read"):
        load_settings(tmp_path / "nope.yaml")


def test_malformed_yaml_is_a_config_error(tmp_path: Path) -> None:
    path = tmp_path / "settings.yaml"
    path.write_text("geometry: [unclosed", encoding="utf-8")
    with pytest.raises(ConfigError, match="not valid YAML"):
        load_settings(path)


def test_top_level_must_be_a_mapping(tmp_path: Path) -> None:
    with pytest.raises(ConfigError, match="expected a mapping"):
        load_settings(_write(tmp_path, ["not", "a", "mapping"]))
