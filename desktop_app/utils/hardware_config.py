"""Typed loader for config/hardware.yaml.

The YAML file is the single PC-side source of truth for hardware constants
(docs/SPEC.md section 4, SPEC rule 7). This module turns it into frozen,
typed dataclasses and refuses to start on anything malformed: a missing key,
an unknown key, a wrong type, or a physically impossible combination.

Values that are still [TO CONFIRM] are listed in ``HardwareConfig.to_confirm``
and logged as warnings at load time, so a default is never used silently.
"""

from __future__ import annotations

import dataclasses
import logging
import math
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Mapping, get_args, get_origin, get_type_hints

import yaml

logger = logging.getLogger(__name__)

DEFAULT_CONFIG_PATH = Path(__file__).resolve().parents[2] / "config" / "hardware.yaml"

# GPIO34..39 on the ESP32 are input-only.
_ESP32_INPUT_ONLY_GPIOS = frozenset(range(34, 40))
_ESP32_MAX_GPIO = 39
_ADS1115_ADDRESSES = frozenset({0x48, 0x49, 0x4A, 0x4B})
_ADS1115_CHANNELS = frozenset({0, 1, 2, 3})
_LEVELS = frozenset({"HIGH", "LOW"})


class ConfigError(ValueError):
    """Raised when hardware.yaml is missing, malformed or inconsistent."""


def _positive(section: str, **values: float) -> None:
    for name, value in values.items():
        if not value > 0:
            raise ConfigError(f"{section}.{name} must be > 0, got {value}")


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
    gear_ratio: float  # i, [TO CONFIRM]

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
    debounce_ms_min: int
    debounce_ms_max: int

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
            debounce_ms_min=self.debounce_ms_min,
            debounce_ms_max=self.debounce_ms_max,
        )
        if self.debounce_ms_min > self.debounce_ms_max:
            raise ConfigError("electrical.limit_switches: debounce_ms_min must be <= debounce_ms_max")


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

    def __post_init__(self) -> None:
        _positive("electrical.z_axis", pwm_frequency_hz=self.pwm_frequency_hz)
        if self.timeout_factor < 1.0:
            raise ConfigError(
                f"electrical.z_axis.timeout_factor must be >= 1 (the full travel must fit), got {self.timeout_factor}"
            )


@dataclass(frozen=True)
class ServoConfig:
    model: str
    pwm_frequency_hz: int
    pulse_us_at_0_deg: int
    pulse_us_at_180_deg: int
    separate_5v_supply: bool
    common_gnd_with_esp32: bool
    separate_ledc_channel_and_timer: bool

    def __post_init__(self) -> None:
        _positive("electrical.servo", pwm_frequency_hz=self.pwm_frequency_hz, pulse_us_at_0_deg=self.pulse_us_at_0_deg)
        if not self.pulse_us_at_0_deg < self.pulse_us_at_180_deg:
            raise ConfigError("electrical.servo: pulse_us_at_0_deg must be < pulse_us_at_180_deg")


@dataclass(frozen=True)
class ElectricalConfig:
    tb6600: Tb6600Config
    limit_switches: LimitSwitchConfig
    ads1115: Ads1115Config
    as5600: As5600Config
    z_axis: ZAxisConfig
    servo: ServoConfig


# --------------------------------------------------------------------------- #
# Root
# --------------------------------------------------------------------------- #


@dataclass(frozen=True)
class HardwareConfig:
    """Everything in config/hardware.yaml."""

    geometry: GeometryConfig
    drive: DriveConfig
    pins: PinsConfig
    electrical: ElectricalConfig
    to_confirm: tuple[str, ...]

    def __post_init__(self) -> None:
        for dotted in self.to_confirm:
            node: Any = self
            for part in dotted.split("."):
                if not hasattr(node, part):
                    raise ConfigError(f"to_confirm entry {dotted!r} does not match any config value")
                node = getattr(node, part)


# --------------------------------------------------------------------------- #
# Loading
# --------------------------------------------------------------------------- #


def _join(parent: str, name: str) -> str:
    return f"{parent}.{name}" if parent else name


def _coerce(tp: Any, value: Any, path: str) -> Any:
    """Validate ``value`` against the dataclass field type ``tp``."""
    if dataclasses.is_dataclass(tp):
        return _build(tp, value, path)
    if tp is bool:
        if isinstance(value, bool):
            return value
    elif tp is int:
        if isinstance(value, int) and not isinstance(value, bool):
            return value
    elif tp is float:
        if isinstance(value, (int, float)) and not isinstance(value, bool):
            number = float(value)
            if not math.isfinite(number):
                raise ConfigError(f"{path}: must be a finite number, got {value!r}")
            return number
    elif tp is str:
        if isinstance(value, str):
            return value
    elif get_origin(tp) is tuple and get_args(tp) == (str, Ellipsis):
        if isinstance(value, (list, tuple)) and all(isinstance(item, str) for item in value):
            return tuple(value)
    else:
        raise TypeError(f"unsupported config field type {tp!r} at {path}")
    expected = getattr(tp, "__name__", str(tp))
    raise ConfigError(f"{path}: expected {expected}, got {type(value).__name__} ({value!r})")


def _build(cls: type, data: Any, path: str) -> Any:
    where = path or "<root>"
    if not isinstance(data, Mapping):
        raise ConfigError(f"{where}: expected a mapping, got {type(data).__name__}")
    fields = dataclasses.fields(cls)
    names = {f.name for f in fields}
    keys = {str(key) for key in data}
    if unknown := sorted(keys - names):
        raise ConfigError(f"{where}: unknown key(s) {unknown}")
    if missing := sorted(names - keys):
        raise ConfigError(f"{where}: missing key(s) {missing}")
    hints = get_type_hints(cls)
    kwargs = {f.name: _coerce(hints[f.name], data[f.name], _join(path, f.name)) for f in fields}
    return cls(**kwargs)


def load_hardware_config(path: str | Path | None = None) -> HardwareConfig:
    """Load and validate hardware.yaml (default: config/hardware.yaml in the repo)."""
    config_path = Path(path) if path is not None else DEFAULT_CONFIG_PATH
    try:
        text = config_path.read_text(encoding="utf-8")
    except OSError as exc:
        raise ConfigError(f"cannot read hardware config {config_path}: {exc}") from exc
    try:
        raw = yaml.safe_load(text)
    except yaml.YAMLError as exc:
        raise ConfigError(f"hardware config {config_path} is not valid YAML: {exc}") from exc

    config: HardwareConfig = _build(HardwareConfig, raw, "")
    for dotted in config.to_confirm:
        logger.warning(
            "hardware config value %s is a [TO CONFIRM] default from docs/SPEC.md; "
            "confirm it with the hardware owner before relying on values derived from it",
            dotted,
        )
    logger.info("Loaded hardware config from %s", config_path)
    return config
