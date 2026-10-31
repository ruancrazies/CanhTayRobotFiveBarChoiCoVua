"""Typed loader for config/settings.yaml: the single source of every changeable value (SPEC rule 7).

``config/settings.yaml`` is turned into frozen, typed dataclasses. The loader refuses to start on anything
malformed: a missing key, an unknown key, a wrong type, or a physically impossible combination.

An optional, git-ignored ``config/local.yaml`` overlays per-machine values (the Stockfish path, the serial port). It
uses the same keys as settings.yaml and only overrides: it may not add a key, and it may not touch ``assumed``.
Precedence for the engine path, highest first: the ``STOCKFISH_PATH`` environment variable (blank counts as unset),
then ``engine.path`` of config/local.yaml, then ``engine.path`` of config/settings.yaml (:class:`PathSource`).

``Settings.assumed`` lists the dotted key paths that are not confirmed on the real hardware. The planner and the Robot
Commander read it to decide what is still a placeholder.

The firmware headers (firmware/include/pins.h, config.h) and docs/CONFIG_REFERENCE.md are generated from this file by
``python -m tools.gen_all``.
"""

from __future__ import annotations

import dataclasses
import logging
import math
import os
from collections.abc import Mapping as MappingABC
from dataclasses import dataclass, field
from enum import Enum
from pathlib import Path
from types import MappingProxyType
from typing import Any, Mapping, Optional, Union, get_args, get_origin, get_type_hints

import yaml

logger = logging.getLogger(__name__)

REPO_ROOT = Path(__file__).resolve().parents[2]
DEFAULT_SETTINGS_PATH = REPO_ROOT / "config" / "settings.yaml"
DEFAULT_LOCAL_PATH = REPO_ROOT / "config" / "local.yaml"  # optional, never tracked by git
STOCKFISH_PATH_ENV = "STOCKFISH_PATH"

# Facts about the ESP32 and the ADS1115 (chip limits, not settings).
_ESP32_INPUT_ONLY_GPIOS = frozenset(range(34, 40))
_ESP32_MAX_GPIO = 39
_ESP32_LEDC_CHANNELS = 16
_ESP32_LEDC_MAX_RESOLUTION_BITS = 20
_ESP32_LEDC_CHANNELS_PER_TIMER = 2  # Arduino-ESP32 2.x: channel c uses timer (c // 2) % 4
_ESP32_LEDC_TIMERS = 4
_ADS1115_ADDRESSES = frozenset({0x48, 0x49, 0x4A, 0x4B})
_ADS1115_CHANNELS = frozenset({0, 1, 2, 3})
_LEVELS = frozenset({"HIGH", "LOW"})
_PERCENT_MAX = 100
_UINT8_MAX = 255  # the firmware stores these in uint8_t / uint16_t / uint32_t constants
_UINT16_MAX = 65535
_UINT32_MAX = 4294967295
_MAX_SINGULARITY_MARGIN_DEG = 90.0  # the margin mu lives in [0, 90)
_RESERVE_PIECE_SYMBOLS = frozenset({"Q", "R", "B", "N"})  # the only pieces a pawn can promote to
_MS_PER_S = 1000.0  # unit conversion: settings.yaml holds milliseconds, the PC code works in seconds
_ENGINE_MODE_KEYS = ("beginner", "amateur", "player", "master", "grandmaster")
_UCI_OPTIONS_PATHS = ("engine.default_options",)  # plus every ``engine.modes.<mode>.options``


class ConfigError(ValueError):
    """Raised when settings.yaml (or the local overlay) is missing, malformed or inconsistent."""


class PathSource(str, Enum):
    """Where ``Settings.engine.path`` came from (highest precedence first)."""

    ENVIRONMENT = "environment"  # the STOCKFISH_PATH variable
    LOCAL_FILE = "local_file"  # engine.path in config/local.yaml
    SETTINGS_FILE = "settings_file"  # engine.path in config/settings.yaml


def to_seconds(milliseconds: float) -> float:
    """A duration of settings.yaml (milliseconds) in seconds."""
    return milliseconds / _MS_PER_S


def _positive(section: str, **values: Any) -> None:
    for name, value in values.items():
        if not value > 0:
            raise ConfigError(f"{section}.{name} must be > 0, got {value}")


def _optional_positive(section: str, **values: Any) -> None:
    for name, value in values.items():
        if value is not None and not value > 0:
            raise ConfigError(f"{section}.{name} must be > 0 or null, got {value}")


def _within(section: str, name: str, value: Any, low: int, high: int) -> None:
    if not low <= value <= high:
        raise ConfigError(f"{section}.{name} must be within {low}..{high}, got {value}")


# --------------------------------------------------------------------------- #
# Geometry and drive
# --------------------------------------------------------------------------- #


@dataclass(frozen=True)
class GeometryConfig:
    """Mechanical dimensions in millimetres (SPEC 4.1)."""

    motor_axis_distance_mm: float  # l
    driving_link_mm: float  # m
    driven_link_mm: float  # n
    motor_to_board_edge_mm: float  # k
    square_size_mm: float  # s
    board_border_mm: float
    squares_per_side: int
    z_travel_mm: float  # h

    def __post_init__(self) -> None:
        _positive(
            "geometry",
            motor_axis_distance_mm=self.motor_axis_distance_mm,
            driving_link_mm=self.driving_link_mm,
            driven_link_mm=self.driven_link_mm,
            motor_to_board_edge_mm=self.motor_to_board_edge_mm,
            square_size_mm=self.square_size_mm,
            squares_per_side=self.squares_per_side,
            z_travel_mm=self.z_travel_mm,
        )
        if self.board_border_mm < 0:
            raise ConfigError(f"geometry.board_border_mm must be >= 0, got {self.board_border_mm}")

    @property
    def motor1_xy_mm(self) -> tuple[float, float]:
        """Motor #1 axis (angle alpha) at (-l/2, 0)."""
        return (-self.motor_axis_distance_mm / 2.0, 0.0)

    @property
    def motor2_xy_mm(self) -> tuple[float, float]:
        """Motor #2 axis (angle beta) at (+l/2, 0)."""
        return (self.motor_axis_distance_mm / 2.0, 0.0)

    @property
    def board_size_mm(self) -> float:
        """Side of the square board including its border (300 mm)."""
        return self.squares_per_side * self.square_size_mm + 2.0 * self.board_border_mm

    @property
    def board_center_xy_mm(self) -> tuple[float, float]:
        """Board centre on the Y axis, (0, 300) for the reference build."""
        return (0.0, self.motor_to_board_edge_mm + self.board_size_mm / 2.0)

    @property
    def first_square_center_xy_mm(self) -> tuple[float, float]:
        """Centre of square A1 (file A, rank 1): (-108.5, 191.5) for the reference build."""
        x = -self.board_size_mm / 2.0 + self.board_border_mm + self.square_size_mm / 2.0
        y = self.motor_to_board_edge_mm + self.board_border_mm + self.square_size_mm / 2.0
        return (x, y)


@dataclass(frozen=True)
class DriveConfig:
    """Stepper drive chain (SPEC 4.1: pulses/rev = 200 * f * i)."""

    full_steps_per_rev: int
    microstepping: int  # f
    gear_ratio: float  # i

    def __post_init__(self) -> None:
        _positive(
            "drive",
            full_steps_per_rev=self.full_steps_per_rev,
            microstepping=self.microstepping,
            gear_ratio=self.gear_ratio,
        )

    @property
    def pulses_per_rev(self) -> float:
        """Pulses per revolution of the driving link: 3200 when i = 1."""
        return self.full_steps_per_rev * self.microstepping * self.gear_ratio

    @property
    def joint_resolution_deg(self) -> float:
        """Joint angle resolution: 0.1125 degrees when i = 1."""
        return 360.0 / self.pulses_per_rev


# --------------------------------------------------------------------------- #
# Pins
# --------------------------------------------------------------------------- #


@dataclass(frozen=True)
class JointPins:
    ena: int
    dir: int
    pul: int


@dataclass(frozen=True)
class ZMotorPins:
    pwm: int
    in1_raise: int
    in2_lower: int


@dataclass(frozen=True)
class I2cPins:
    sda: int
    scl: int


@dataclass(frozen=True)
class PinsConfig:
    """ESP32 GPIO assignment (SPEC 4.2)."""

    joint1: JointPins
    joint2: JointPins
    z_motor: ZMotorPins
    i2c: I2cPins
    servo_pwm: int
    limit_top: int
    limit_bottom: int

    def all_pins(self) -> dict[str, int]:
        """Every assigned GPIO keyed by a dotted role name."""
        return {
            "joint1.ena": self.joint1.ena,
            "joint1.dir": self.joint1.dir,
            "joint1.pul": self.joint1.pul,
            "joint2.ena": self.joint2.ena,
            "joint2.dir": self.joint2.dir,
            "joint2.pul": self.joint2.pul,
            "z_motor.pwm": self.z_motor.pwm,
            "z_motor.in1_raise": self.z_motor.in1_raise,
            "z_motor.in2_lower": self.z_motor.in2_lower,
            "i2c.sda": self.i2c.sda,
            "i2c.scl": self.i2c.scl,
            "servo_pwm": self.servo_pwm,
            "limit_top": self.limit_top,
            "limit_bottom": self.limit_bottom,
        }

    def __post_init__(self) -> None:
        pins = self.all_pins()
        for role, gpio in pins.items():
            if not 0 <= gpio <= _ESP32_MAX_GPIO:
                raise ConfigError(f"pins.{role}: GPIO {gpio} is outside 0..{_ESP32_MAX_GPIO}")
        by_gpio: dict[int, str] = {}
        for role, gpio in pins.items():
            if gpio in by_gpio:
                raise ConfigError(f"pins: GPIO {gpio} is assigned twice ({by_gpio[gpio]} and {role})")
            by_gpio[gpio] = role
        inputs = {"limit_top", "limit_bottom"}
        for role, gpio in pins.items():
            if role not in inputs and gpio in _ESP32_INPUT_ONLY_GPIOS:
                raise ConfigError(f"pins.{role}: GPIO {gpio} is input-only on the ESP32")


# --------------------------------------------------------------------------- #
# Electrical conventions (SPEC 4.3)
# --------------------------------------------------------------------------- #


@dataclass(frozen=True)
class Tb6600DipConfig:
    sw1_on: bool
    sw2_on: bool
    sw3_on: bool


@dataclass(frozen=True)
class Tb6600Config:
    wiring: str
    microstep_dip: Tb6600DipConfig
    ena_level_driver_enabled: str
    ena_level_driver_disabled: str
    low_active_enables_stepper: bool
    auto_enable: bool
    joint1_dir_high_counts_up: bool
    joint2_dir_high_counts_up: bool

    def __post_init__(self) -> None:
        if self.wiring != "common_anode":
            raise ConfigError(
                f"electrical.tb6600.wiring must be 'common_anode' (ENA polarity depends on it), got {self.wiring!r}"
            )
        for name in ("ena_level_driver_enabled", "ena_level_driver_disabled"):
            value = getattr(self, name)
            if value not in _LEVELS:
                raise ConfigError(f"electrical.tb6600.{name} must be HIGH or LOW, got {value!r}")
        if self.ena_level_driver_enabled == self.ena_level_driver_disabled:
            raise ConfigError("electrical.tb6600: enabled and disabled ENA levels must differ")


@dataclass(frozen=True)
class LimitSwitchConfig:
    contact: str
    com_connected_to: str
    external_pulldown_ohm: int
    idle_level: str
    actuated_level: str
    debounce_min_ms: int
    debounce_max_ms: int
    debounce_ms: int

    def __post_init__(self) -> None:
        if self.contact != "NC":
            raise ConfigError(
                f"electrical.limit_switches.contact must be 'NC' (fail-safe on a broken wire), got {self.contact!r}"
            )
        if self.idle_level != "HIGH" or self.actuated_level != "LOW":
            raise ConfigError(
                "electrical.limit_switches: NC switches must read HIGH when idle and LOW when actuated or broken"
            )
        _positive(
            "electrical.limit_switches",
            external_pulldown_ohm=self.external_pulldown_ohm,
            debounce_min_ms=self.debounce_min_ms,
            debounce_max_ms=self.debounce_max_ms,
        )
        if self.debounce_min_ms > self.debounce_max_ms:
            raise ConfigError("electrical.limit_switches: debounce_min_ms must be <= debounce_max_ms")
        if not self.debounce_min_ms <= self.debounce_ms <= self.debounce_max_ms:
            raise ConfigError(
                f"electrical.limit_switches.debounce_ms {self.debounce_ms} must be within "
                f"debounce_min_ms..debounce_max_ms ({self.debounce_min_ms}..{self.debounce_max_ms})"
            )
        _within("electrical.limit_switches", "debounce_ms", self.debounce_ms, 1, _UINT8_MAX)


@dataclass(frozen=True)
class Ads1115Config:
    address: int
    addr_pin_tied_to: str
    pga_full_scale_v: float

    def __post_init__(self) -> None:
        if self.address not in _ADS1115_ADDRESSES:
            raise ConfigError(f"electrical.ads1115.address {self.address:#x} is not a valid ADS1115 address")
        _positive("electrical.ads1115", pga_full_scale_v=self.pga_full_scale_v)


@dataclass(frozen=True)
class As5600Config:
    output_mode: str
    outs_register: int
    output_min_percent_vdd: float
    output_max_percent_vdd: float
    joint1_alpha_adc_channel: int
    joint2_beta_adc_channel: int
    vdd_adc_channel: int
    r_fault_low: float
    r_fault_high: float
    system_error_deg: float
    calibration_file: str

    def __post_init__(self) -> None:
        if self.output_mode != "analog":
            raise ConfigError(f"electrical.as5600.output_mode must be 'analog', got {self.output_mode!r}")
        if not self.output_min_percent_vdd < self.output_max_percent_vdd:
            raise ConfigError("electrical.as5600: output_min_percent_vdd must be < output_max_percent_vdd")
        channels = (self.joint1_alpha_adc_channel, self.joint2_beta_adc_channel, self.vdd_adc_channel)
        if any(ch not in _ADS1115_CHANNELS for ch in channels):
            raise ConfigError(f"electrical.as5600: ADC channels must be within 0..3, got {channels}")
        if len(set(channels)) != len(channels):
            raise ConfigError(f"electrical.as5600: ADC channels must be distinct, got {channels}")
        if not 0.0 <= self.r_fault_low < self.r_fault_high <= 1.0:
            raise ConfigError("electrical.as5600: need 0 <= r_fault_low < r_fault_high <= 1")
        _positive("electrical.as5600", system_error_deg=self.system_error_deg)


@dataclass(frozen=True)
class ZAxisConfig:
    driver: str
    pwm_frequency_hz: int
    timeout_factor: float
    full_travel_ms: Optional[int]
    pwm_resolution_bits: int
    ledc_channel: int
    duty_start_percent: int
    duty_peak_percent: int
    duty_ramp_ms: int
    duty_eased_percent: int
    ease_after_travel_percent: int
    motor_pins_boot_level: str

    def __post_init__(self) -> None:
        _positive("electrical.z_axis", pwm_frequency_hz=self.pwm_frequency_hz)
        for name in ("duty_start_percent", "duty_peak_percent", "duty_eased_percent"):
            _within("electrical.z_axis", name, getattr(self, name), 1, _PERCENT_MAX)
        if self.duty_start_percent > self.duty_peak_percent:
            raise ConfigError("electrical.z_axis: duty_start_percent must not exceed duty_peak_percent (the duty ramps UP)")
        _within("electrical.z_axis", "duty_ramp_ms", self.duty_ramp_ms, 0, _UINT16_MAX)
        _within("electrical.z_axis", "ease_after_travel_percent", self.ease_after_travel_percent, 1, _PERCENT_MAX)
        _optional_positive("electrical.z_axis", full_travel_ms=self.full_travel_ms)
        if self.full_travel_ms is not None:
            _within("electrical.z_axis", "full_travel_ms", self.full_travel_ms, 1, _UINT32_MAX)
        if self.timeout_factor < 1.0:
            raise ConfigError(
                f"electrical.z_axis.timeout_factor must be >= 1 (the full travel must fit), got {self.timeout_factor}"
            )
        _within("electrical.z_axis", "pwm_resolution_bits", self.pwm_resolution_bits, 1, _ESP32_LEDC_MAX_RESOLUTION_BITS)
        _within("electrical.z_axis", "ledc_channel", self.ledc_channel, 0, _ESP32_LEDC_CHANNELS - 1)
        if self.motor_pins_boot_level != "LOW":
            raise ConfigError(
                "electrical.z_axis.motor_pins_boot_level must be 'LOW' (Z must be off before a command arrives, "
                f"SPEC 10.2), got {self.motor_pins_boot_level!r}"
            )


@dataclass(frozen=True)
class ServoConfig:
    model: str
    pwm_frequency_hz: int
    pulse_at_0_deg_us: int
    pulse_at_180_deg_us: int
    pwm_resolution_bits: int
    ledc_channel: int
    separate_5v_supply: bool
    common_gnd_with_esp32: bool
    separate_ledc_channel_and_timer: bool

    def __post_init__(self) -> None:
        _positive("electrical.servo", pwm_frequency_hz=self.pwm_frequency_hz, pulse_at_0_deg_us=self.pulse_at_0_deg_us)
        if not self.pulse_at_0_deg_us < self.pulse_at_180_deg_us:
            raise ConfigError("electrical.servo: pulse_at_0_deg_us must be < pulse_at_180_deg_us")
        _within("electrical.servo", "pulse_at_180_deg_us", self.pulse_at_180_deg_us, 1, _UINT16_MAX)
        _within("electrical.servo", "pwm_resolution_bits", self.pwm_resolution_bits, 1, _ESP32_LEDC_MAX_RESOLUTION_BITS)
        _within("electrical.servo", "ledc_channel", self.ledc_channel, 0, _ESP32_LEDC_CHANNELS - 1)


def _ledc_timer(channel: int) -> int:
    return (channel // _ESP32_LEDC_CHANNELS_PER_TIMER) % _ESP32_LEDC_TIMERS


@dataclass(frozen=True)
class ElectricalConfig:
    tb6600: Tb6600Config
    limit_switches: LimitSwitchConfig
    ads1115: Ads1115Config
    as5600: As5600Config
    z_axis: ZAxisConfig
    servo: ServoConfig

    def __post_init__(self) -> None:
        z, servo = self.z_axis.ledc_channel, self.servo.ledc_channel
        if z == servo or _ledc_timer(z) == _ledc_timer(servo):
            raise ConfigError(
                f"electrical: the Z motor (LEDC channel {z}) and the servo (channel {servo}) must use different "
                "LEDC channels AND timers (SPEC 10.2)"
            )


# --------------------------------------------------------------------------- #
# Serial link, timeouts, motion
# --------------------------------------------------------------------------- #


@dataclass(frozen=True)
class SerialConfig:
    baud: int
    port: Optional[str]
    rx_buffer_bytes: int
    max_line_bytes: int
    max_response_bytes: int
    queue_depth: int

    def __post_init__(self) -> None:
        _positive("serial", baud=self.baud)
        _within("serial", "baud", self.baud, 1, _UINT32_MAX)
        if self.port is not None and not self.port.strip():
            raise ConfigError("serial.port must be a non-empty string or null")
        _within("serial", "rx_buffer_bytes", self.rx_buffer_bytes, 1, _UINT16_MAX)
        _within("serial", "max_line_bytes", self.max_line_bytes, 1, _UINT16_MAX)
        _within("serial", "max_response_bytes", self.max_response_bytes, 1, _UINT16_MAX)
        _within("serial", "queue_depth", self.queue_depth, 1, _UINT8_MAX)
        if self.rx_buffer_bytes < self.max_line_bytes:
            raise ConfigError("serial.rx_buffer_bytes must hold at least one full line (>= max_line_bytes)")


@dataclass(frozen=True)
class TimeoutsConfig:
    heartbeat_timeout_ms: int
    thread_join_ms: int

    def __post_init__(self) -> None:
        _positive("timeouts", heartbeat_timeout_ms=self.heartbeat_timeout_ms, thread_join_ms=self.thread_join_ms)
        _within("timeouts", "heartbeat_timeout_ms", self.heartbeat_timeout_ms, 1, _UINT32_MAX)


@dataclass(frozen=True)
class JointLimitsConfig:
    alpha_min_deg: Optional[float]
    alpha_max_deg: Optional[float]
    beta_min_deg: Optional[float]
    beta_max_deg: Optional[float]

    def __post_init__(self) -> None:
        values = (self.alpha_min_deg, self.alpha_max_deg, self.beta_min_deg, self.beta_max_deg)
        if all(v is None for v in values):
            return
        if any(v is None for v in values):
            raise ConfigError("motion.joint_limits: set all four limits or none of them")
        if not self.alpha_min_deg < self.alpha_max_deg or not self.beta_min_deg < self.beta_max_deg:  # type: ignore[operator]
            raise ConfigError("motion.joint_limits: each minimum must be below its maximum")

    @property
    def configured(self) -> bool:
        return self.alpha_min_deg is not None


@dataclass(frozen=True)
class MotionConfig:
    max_speed_deg_s: Optional[float]
    max_accel_deg_s2: Optional[float]
    direction_change_delay_us: Optional[int]
    step_loss_threshold_deg: float
    trajectory_step_mm: float
    singularity_margin_deg: float
    joint_limits: JointLimitsConfig
    bench_move_without_homing: bool

    def __post_init__(self) -> None:
        _optional_positive(
            "motion",
            max_speed_deg_s=self.max_speed_deg_s,
            max_accel_deg_s2=self.max_accel_deg_s2,
            direction_change_delay_us=self.direction_change_delay_us,
        )
        if self.direction_change_delay_us is not None:
            _within("motion", "direction_change_delay_us", self.direction_change_delay_us, 1, _UINT16_MAX)
        _positive("motion", step_loss_threshold_deg=self.step_loss_threshold_deg, trajectory_step_mm=self.trajectory_step_mm)
        if not 0.0 <= self.singularity_margin_deg < _MAX_SINGULARITY_MARGIN_DEG:
            raise ConfigError(f"motion.singularity_margin_deg must be in [0, 90), got {self.singularity_margin_deg}")


@dataclass(frozen=True)
class FirmwareConfig:
    version: str

    def __post_init__(self) -> None:
        if not self.version or any(ch.isspace() or ch == "=" for ch in self.version):
            raise ConfigError("firmware.version must be one token: no spaces and no '='")


# --------------------------------------------------------------------------- #
# PC side: Robot Commander, serial link, simulator
# --------------------------------------------------------------------------- #


@dataclass(frozen=True)
class CommanderConfig:
    move_speed_deg_s: Optional[float]
    acceleration_deg_s2: Optional[float]
    ping_interval_ms: Optional[float]
    grip_settle_ms: Optional[float]
    start_tolerance_deg: Optional[float]
    reply_timeout_ms: Optional[float]
    done_timeout_ms: Optional[float]
    send_timeout_ms: Optional[float]
    queue_retry_ms: Optional[float]
    queue_full_timeout_ms: Optional[float]
    clock_poll_ms: float

    def __post_init__(self) -> None:
        _optional_positive(
            "commander",
            move_speed_deg_s=self.move_speed_deg_s,
            acceleration_deg_s2=self.acceleration_deg_s2,
            ping_interval_ms=self.ping_interval_ms,
            start_tolerance_deg=self.start_tolerance_deg,
            reply_timeout_ms=self.reply_timeout_ms,
            done_timeout_ms=self.done_timeout_ms,
            send_timeout_ms=self.send_timeout_ms,
            queue_retry_ms=self.queue_retry_ms,
            queue_full_timeout_ms=self.queue_full_timeout_ms,
        )
        _positive("commander", clock_poll_ms=self.clock_poll_ms)
        if self.grip_settle_ms is not None and self.grip_settle_ms < 0:
            raise ConfigError(f"commander.grip_settle_ms must be >= 0 or null, got {self.grip_settle_ms}")

    def unset(self) -> tuple[str, ...]:
        """The keys the Robot Commander needs that have no value (null) yet."""
        needed = (
            "move_speed_deg_s",
            "ping_interval_ms",
            "grip_settle_ms",
            "start_tolerance_deg",
            "reply_timeout_ms",
            "done_timeout_ms",
            "send_timeout_ms",
            "queue_retry_ms",
            "queue_full_timeout_ms",
        )
        return tuple(f"commander.{name}" for name in needed if getattr(self, name) is None)


@dataclass(frozen=True)
class LinkConfig:
    max_in_flight: int
    read_poll_ms: float
    write_timeout_ms: Optional[float]
    reconnect_interval_ms: Optional[float]
    reconnect_max_attempts: Optional[int]
    raw_preview_bytes: int

    def __post_init__(self) -> None:
        _positive("link", max_in_flight=self.max_in_flight, read_poll_ms=self.read_poll_ms, raw_preview_bytes=self.raw_preview_bytes)
        _optional_positive("link", write_timeout_ms=self.write_timeout_ms, reconnect_interval_ms=self.reconnect_interval_ms)
        if self.reconnect_max_attempts is not None:
            if self.reconnect_max_attempts < 1:
                raise ConfigError("link.reconnect_max_attempts must be >= 1 or null")
            if self.reconnect_interval_ms is None:
                raise ConfigError("link.reconnect_max_attempts needs link.reconnect_interval_ms (without it the link does not reconnect)")


@dataclass(frozen=True)
class SimulatorConfig:
    firmware_version: str
    poll_ms: float
    home_joints_ms: Optional[float]
    initial_alpha_deg: Optional[float]
    initial_beta_deg: Optional[float]
    initial_z: str

    def __post_init__(self) -> None:
        if not self.firmware_version or any(ch.isspace() for ch in self.firmware_version):
            raise ConfigError("simulator.firmware_version must be one token")
        _positive("simulator", poll_ms=self.poll_ms)
        if self.home_joints_ms is not None and self.home_joints_ms < 0:
            raise ConfigError("simulator.home_joints_ms must be >= 0 or null")
        if (self.initial_alpha_deg is None) != (self.initial_beta_deg is None):
            raise ConfigError("simulator: set initial_alpha_deg and initial_beta_deg together or leave both null")
        if self.initial_z not in ("TOP", "BOTTOM"):
            raise ConfigError(f"simulator.initial_z must be TOP or BOTTOM, got {self.initial_z!r}")


# --------------------------------------------------------------------------- #
# Chess engine
# --------------------------------------------------------------------------- #

UciOptions = Mapping[str, Union[bool, int, str]]


@dataclass(frozen=True)
class DifficultyModeConfig:
    options: UciOptions
    think_time_ms: float
    max_depth: Optional[int]

    def __post_init__(self) -> None:
        _positive("engine.modes", think_time_ms=self.think_time_ms)
        if self.max_depth is not None and self.max_depth < 1:
            raise ConfigError("engine.modes: max_depth must be null or an integer >= 1")


@dataclass(frozen=True)
class ModesConfig:
    """The five difficulty modes (SPEC 9); the set is fixed, only their values change."""

    beginner: DifficultyModeConfig
    amateur: DifficultyModeConfig
    player: DifficultyModeConfig
    master: DifficultyModeConfig
    grandmaster: DifficultyModeConfig

    def as_dict(self) -> dict[str, DifficultyModeConfig]:
        return {key: getattr(self, key) for key in _ENGINE_MODE_KEYS}


@dataclass(frozen=True)
class EngineSettings:
    path: str
    answer_margin_ms: float
    max_restarts: int
    default_mode: str
    default_options: UciOptions
    modes: ModesConfig

    def __post_init__(self) -> None:
        if not self.path.strip():
            raise ConfigError("engine.path must be a non-empty string")
        object.__setattr__(self, "path", self.path.strip())  # surrounding blanks of a hand-edited value never count
        _positive("engine", answer_margin_ms=self.answer_margin_ms)
        if self.max_restarts < 0:
            raise ConfigError("engine.max_restarts must be an integer >= 0")
        if self.default_mode not in _ENGINE_MODE_KEYS:
            raise ConfigError(f"engine.default_mode must be one of {list(_ENGINE_MODE_KEYS)}, got {self.default_mode!r}")


# --------------------------------------------------------------------------- #
# Grasp angles, layout, paths, logging
# --------------------------------------------------------------------------- #


@dataclass(frozen=True)
class CloseAnglesConfig:
    P: float
    N: float
    B: float
    R: float
    Q: float
    K: float


@dataclass(frozen=True)
class GraspConfig:
    open_deg: float
    close_deg: CloseAnglesConfig

    def __post_init__(self) -> None:
        angles = {"open_deg": self.open_deg, **{f"close_deg.{f.name}": getattr(self.close_deg, f.name) for f in dataclasses.fields(self.close_deg)}}
        for name, value in angles.items():
            if value < 0:
                raise ConfigError(f"grasp.{name} must be a finite number >= 0, got {value!r}")


@dataclass(frozen=True)
class GridConfig:
    """A rectangular grid of slots, x-major: every y of the first x, then the next x, ..."""

    x_mm: tuple[float, ...]
    y_mm: tuple[float, ...]

    def __post_init__(self) -> None:
        if not self.x_mm or not self.y_mm:
            raise ConfigError("layout: a slot grid needs at least one x and one y")

    def slots(self) -> tuple[tuple[float, float], ...]:
        return tuple((x, y) for x in self.x_mm for y in self.y_mm)


@dataclass(frozen=True)
class ParkConfig:
    x_mm: float
    y_mm: float

    @property
    def xy_mm(self) -> tuple[float, float]:
        return (self.x_mm, self.y_mm)


@dataclass(frozen=True)
class LayoutConfig:
    capture_slots: GridConfig
    reserve_slots: GridConfig
    park: ParkConfig
    reserve_pieces: tuple[str, ...]

    def __post_init__(self) -> None:
        bad = [p for p in self.reserve_pieces if p not in _RESERVE_PIECE_SYMBOLS]
        if bad:
            raise ConfigError(f"layout.reserve_pieces: a reserve slot holds Q, R, B or N, got {bad}")
        wanted = len(self.reserve_slots.slots())
        if len(self.reserve_pieces) != wanted:
            raise ConfigError(f"layout.reserve_pieces must name the piece of each of the {wanted} reserve slots, got {len(self.reserve_pieces)}")


@dataclass(frozen=True)
class PathsConfig:
    games_dir: str
    fen_file: str
    pgn_file: str
    pending_file: str

    def __post_init__(self) -> None:
        for name in ("games_dir", "fen_file", "pgn_file", "pending_file"):
            if not getattr(self, name).strip():
                raise ConfigError(f"paths.{name} must be a non-empty string")
        for name in ("fen_file", "pgn_file", "pending_file"):
            if any(sep in getattr(self, name) for sep in ("/", "\\")):
                raise ConfigError(f"paths.{name} is a file name inside games_dir, not a path")


@dataclass(frozen=True)
class LoggingConfig:
    max_bytes: int
    backup_count: int

    def __post_init__(self) -> None:
        _positive("logging", max_bytes=self.max_bytes)
        if self.backup_count < 0:
            raise ConfigError("logging.backup_count must be >= 0")


# --------------------------------------------------------------------------- #
# Root
# --------------------------------------------------------------------------- #


@dataclass(frozen=True)
class Settings:
    """Everything in config/settings.yaml (plus the optional local overlay)."""

    assumed: tuple[str, ...]
    geometry: GeometryConfig
    drive: DriveConfig
    pins: PinsConfig
    electrical: ElectricalConfig
    serial: SerialConfig
    timeouts: TimeoutsConfig
    motion: MotionConfig
    firmware: FirmwareConfig
    commander: CommanderConfig
    link: LinkConfig
    simulator: SimulatorConfig
    engine: EngineSettings
    grasp: GraspConfig
    layout: LayoutConfig
    paths: PathsConfig
    logging: LoggingConfig
    # Where engine.path came from. Not a key of the file.
    path_source: PathSource = field(default=PathSource.SETTINGS_FILE, metadata={"yaml": False})

    def __post_init__(self) -> None:
        if len(set(self.assumed)) != len(self.assumed):
            raise ConfigError("assumed lists a key twice")
        for dotted in self.assumed:
            node: Any = self
            for part in dotted.split("."):
                if isinstance(node, MappingABC):
                    if part not in node:
                        raise ConfigError(f"assumed entry {dotted!r} does not match any settings key")
                    node = node[part]
                elif dataclasses.is_dataclass(node) and hasattr(node, part) and part != "assumed":
                    node = getattr(node, part)
                else:
                    raise ConfigError(f"assumed entry {dotted!r} does not match any settings key")
        if self.link.max_in_flight > self.serial.queue_depth:
            raise ConfigError(
                f"link.max_in_flight ({self.link.max_in_flight}) must not exceed serial.queue_depth ({self.serial.queue_depth})"
            )
        ping = self.commander.ping_interval_ms
        if ping is not None and not ping < self.timeouts.heartbeat_timeout_ms:
            raise ConfigError(
                f"commander.ping_interval_ms ({ping:g}) must stay below timeouts.heartbeat_timeout_ms "
                f"({self.timeouts.heartbeat_timeout_ms})"
            )

    def is_assumed(self, dotted: str) -> bool:
        """True when ``dotted`` itself, or the section it is in, is listed in ``assumed``."""
        return any(dotted == entry or dotted.startswith(entry + ".") for entry in self.assumed)

    def assumed_under(self, prefix: str) -> tuple[str, ...]:
        """The assumed keys of one section (``prefix`` itself or anything below it)."""
        return tuple(entry for entry in self.assumed if entry == prefix or entry.startswith(prefix + "."))

    def resolve_path(self, relative: str) -> Path:
        """``relative`` from settings, made absolute against the repository root."""
        return REPO_ROOT / relative

    @property
    def games_dir(self) -> Path:
        return self.resolve_path(self.paths.games_dir)


# --------------------------------------------------------------------------- #
# Typed building
# --------------------------------------------------------------------------- #


def _join(parent: str, name: str) -> str:
    return f"{parent}.{name}" if parent else name


def _is_number(value: Any) -> bool:
    return isinstance(value, (int, float)) and not isinstance(value, bool)


def _coerce(tp: Any, value: Any, path: str) -> Any:
    """Validate ``value`` against the dataclass field type ``tp``."""
    origin = get_origin(tp)
    if origin is Union:
        args = [a for a in get_args(tp) if a is not type(None)]
        if value is None and len(args) < len(get_args(tp)):
            return None
        if len(args) == 1:
            return _coerce(args[0], value, path)
        if get_args(tp) == (bool, int, str) or set(args) == {bool, int, str}:  # one UCI option value
            if isinstance(value, (bool, int, str)):
                return value
            raise ConfigError(f"{path}: expected a bool, int or string, got {type(value).__name__} ({value!r})")
        raise TypeError(f"unsupported config field type {tp!r} at {path}")
    if dataclasses.is_dataclass(tp):
        return _build(tp, value, path)
    if origin is tuple and len(get_args(tp)) == 2 and get_args(tp)[1] is Ellipsis:
        if not isinstance(value, (list, tuple)):
            raise ConfigError(f"{path}: expected a list, got {type(value).__name__} ({value!r})")
        return tuple(_coerce(get_args(tp)[0], item, f"{path}[{i}]") for i, item in enumerate(value))
    if origin in (MappingABC, dict):  # free-form UCI options: any names, typed values
        if not isinstance(value, MappingABC):
            raise ConfigError(f"{path}: expected a mapping, got {type(value).__name__} ({value!r})")
        item_type = get_args(tp)[1]
        return MappingProxyType({str(k): _coerce(item_type, v, _join(path, str(k))) for k, v in value.items()})
    if tp is bool:
        if isinstance(value, bool):
            return value
    elif tp is int:
        if isinstance(value, int) and not isinstance(value, bool):
            return value
    elif tp is float:
        if _is_number(value):
            number = float(value)
            if not math.isfinite(number):
                raise ConfigError(f"{path}: must be a finite number, got {value!r}")
            return number
    elif tp is str:
        if isinstance(value, str):
            return value
    else:
        raise TypeError(f"unsupported config field type {tp!r} at {path}")
    expected = getattr(tp, "__name__", str(tp))
    raise ConfigError(f"{path}: expected {expected}, got {type(value).__name__} ({value!r})")


def _build(cls: type, data: Any, path: str) -> Any:
    where = path or "<root>"
    if not isinstance(data, MappingABC):
        raise ConfigError(f"{where}: expected a mapping, got {type(data).__name__}")
    fields = [f for f in dataclasses.fields(cls) if f.metadata.get("yaml", True)]
    names = {f.name for f in fields}
    keys = {str(key) for key in data}
    if unknown := sorted(keys - names):
        raise ConfigError(f"{where}: unknown key(s) {unknown}")
    if missing := sorted(names - keys):
        raise ConfigError(f"{where}: missing key(s) {missing}")
    hints = get_type_hints(cls)
    kwargs = {f.name: _coerce(hints[f.name], data[f.name], _join(path, f.name)) for f in fields}
    return cls(**kwargs)


# --------------------------------------------------------------------------- #
# Loading and the local overlay
# --------------------------------------------------------------------------- #


def _read_yaml(path: Path, what: str) -> Any:
    try:
        text = path.read_text(encoding="utf-8")
    except OSError as exc:
        raise ConfigError(f"cannot read {what} {path}: {exc}") from exc
    try:
        return yaml.safe_load(text)
    except yaml.YAMLError as exc:
        raise ConfigError(f"{what} {path} is not valid YAML: {exc}") from exc


def load_raw(path: str | Path | None = None) -> dict[str, Any]:
    """The parsed settings.yaml as plain data (no overlay, no validation): for the generators and the tests."""
    source = Path(path) if path is not None else DEFAULT_SETTINGS_PATH
    raw = _read_yaml(source, "settings file")
    if not isinstance(raw, dict):
        raise ConfigError(f"settings file {source}: expected a mapping at the top level")
    return raw


def _is_opaque(dotted: str) -> bool:
    """UCI option sets are replaced as a whole by an overlay, never merged key by key."""
    return dotted in _UCI_OPTIONS_PATHS or (dotted.startswith("engine.modes.") and dotted.endswith(".options"))


def _merge(base: Mapping[str, Any], overlay: Any, path: str) -> dict[str, Any]:
    where = path or "<root>"
    if not isinstance(overlay, MappingABC):
        raise ConfigError(f"{where}: expected a mapping, got {type(overlay).__name__}")
    merged = dict(base)
    for key, value in overlay.items():
        key = str(key)
        dotted = _join(path, key)
        if key not in base:
            raise ConfigError(f"{where}: unknown key {key!r} (an overlay only overrides keys of settings.yaml)")
        if not path and key == "assumed":
            raise ConfigError("assumed cannot be set in the local overlay: it records what is not confirmed on the hardware")
        if isinstance(base[key], MappingABC) and not _is_opaque(dotted):
            merged[key] = _merge(base[key], value, dotted)
        else:
            merged[key] = value
    return merged


def load_settings(
    path: str | Path | None = None,
    *,
    local_path: str | Path | None = None,
    environ: Optional[Mapping[str, str]] = None,
    overlay: bool = True,
) -> Settings:
    """Load and validate settings.yaml (default: config/settings.yaml), then apply the overlay and the environment.

    ``local_path`` is the overlay file (default: config/local.yaml; a missing file overlays nothing, a broken one is an
    error). ``environ`` defaults to the real environment; both exist so tests and callers do not depend on the machine.
    ``overlay=False`` returns the committed file alone, ignoring config/local.yaml and STOCKFISH_PATH: the generators
    use it, so a generated header never depends on the machine it was generated on.
    """
    source = Path(path) if path is not None else DEFAULT_SETTINGS_PATH
    raw = load_raw(source)
    base = _build(Settings, raw, "")  # settings.yaml alone must be valid, whatever the overlay says

    if not overlay:
        return base
    overlay_path = Path(local_path) if local_path is not None else DEFAULT_LOCAL_PATH
    settings = base
    path_source = PathSource.SETTINGS_FILE
    if overlay_path.exists():
        overlay = _read_yaml(overlay_path, "local overlay")
        if overlay is not None:  # an empty file overrides nothing
            try:
                settings = _build(Settings, _merge(raw, overlay, ""), "")
            except ConfigError as exc:
                raise ConfigError(f"{overlay_path}: {exc}") from exc
            if isinstance(overlay, MappingABC) and "path" in (overlay.get("engine") or {}):
                path_source = PathSource.LOCAL_FILE
            logger.info("Applied the local overlay %s", overlay_path)

    env = os.environ if environ is None else environ
    from_env = env.get(STOCKFISH_PATH_ENV, "").strip()
    if from_env:
        settings = dataclasses.replace(settings, engine=dataclasses.replace(settings.engine, path=from_env))
        path_source = PathSource.ENVIRONMENT
    if path_source is not PathSource.SETTINGS_FILE:
        logger.info(
            "Engine path %r comes from %s (it overrides %r from %s)",
            settings.engine.path, path_source.value, base.engine.path, source.name,
        )
    settings = dataclasses.replace(settings, path_source=path_source)
    logger.info("Loaded settings from %s (%d assumed value(s))", source, len(settings.assumed))
    return settings


# --------------------------------------------------------------------------- #
# The default settings, cached (for library code that needs a default value)
# --------------------------------------------------------------------------- #

_cache: dict[tuple[Any, ...], Settings] = {}


def _stat_signature(path: Path) -> tuple[str, Optional[int], Optional[int]]:
    try:
        info = path.stat()
    except OSError:
        return (str(path), None, None)
    return (str(path), info.st_mtime_ns, info.st_size)


def get_settings() -> Settings:
    """The default settings (config/settings.yaml, the default overlay, STOCKFISH_PATH), loaded once.

    Library code that needs a default value reads it here instead of parsing the file on every call. The cache is
    keyed on both files' modification time and on STOCKFISH_PATH, so an edit or a monkeypatched overlay path loads
    again. Tests and tools that need a specific file call :func:`load_settings` instead.
    """
    key = (
        _stat_signature(DEFAULT_SETTINGS_PATH),
        _stat_signature(DEFAULT_LOCAL_PATH),
        os.environ.get(STOCKFISH_PATH_ENV, ""),
    )
    cached = _cache.get(key)
    if cached is None:
        _cache.clear()
        cached = _cache[key] = load_settings()
    return cached
