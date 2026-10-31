"""ESP32 simulator: speaks the exact protocol of docs/SPEC.md section 5 over a Transport (phase 3).

It stands in for the firmware (phase 4) so the Robot Commander, the link and the FSM can be tested
end to end without hardware, and without any virtual COM port: it is plugged into the device end of
an in-process :func:`loopback_pair`.

Wire format: lines are decoded with ``protocol.decode_command`` and answered with the protocol's own
encoders, so the simulator and the PC code cannot disagree about syntax. Command semantics follow
SPEC 5.1, 5.2 and 10.2:

* both joints (angles, speed, steps issued, the AS5600 reading), Z as a position between TOP (0)
  and BOTTOM (1) so ``MID`` and the timing follow from one model, the gripper angle, the drivers;
* the motion queue: ``MOVE`` is queued and ``error:3`` is answered when it is full (the command is
  NOT queued); the queue drains in time at the commanded speed;
* ``HOME`` (Z first, then the joints) and the rule that every ``MOVE`` is ``error:4`` until homed;
* the heartbeat: after the first ``PING``, a gap longer than the threshold stops both joints, brakes
  Z, keeps the drivers enabled, makes homing mandatory and sends ``!alarm:HB_TIMEOUT``;
* injectable faults: ESTOP (a hardware button), LIMIT_Z, Z_TIMEOUT (Z stalls), SENSOR_FAULT,
  STEP_LOSS and a lost connection (the cable is cut; the heartbeat then times out by itself).

Every value is a setting of :class:`SimConfig`, and :meth:`SimConfig.from_settings` fills it from
``config/settings.yaml`` (queue depth, heartbeat threshold, Z timeout factor, steps per revolution, poll interval,
line limit, firmware version, start position). A value that is still null there (Z travel time, homing time, start
angles) must be passed explicitly: the simulator invents none of them.

Interpretations the SPEC does not spell out (reported in the phase 3 summary):

* ``F`` is in degrees per second (``ACCEL`` is in deg/s^2). The queue length counts the MOVE being
  executed. Every line is checked in the ONE order of SPEC 5.2, ``CHECK_ORDER`` of the declaration, which
  this module walks itself: unknown command (1), bad parameter or joint limits (2), ESTOP (5), drivers
  disabled (6, MOVE and a HOME that moves the joints), not homed or Z not at TOP (4), queue full (3).
* In ESTOP only POS, INFO, PING, ZSTOP, ESTOP, RESET and DISABLE are accepted; everything else, ENABLE
  included, is ``error:5`` until RESET. A repeated ESTOP is answered ``!alarm:ESTOP`` again.
* DISABLE (SPEC 5.1, "DISABLE rules") stops the joints where they are, clears the WHOLE motion queue (queued
  MOVEs are discarded, not drained), cancels the ``done`` of every pending WAIT (the same rule as ESTOP,
  RESET, the heartbeat and a new HOME: a ``done`` is only sent for motion that finished), clears both homed
  flags, and aborts a HOME that still has the joints to do, as ZSTOP aborts a homing: no ``done``. A Z-only
  HOME needs no driver and carries on; so does a running ZUP or ZDOWN. HOME never enables the drivers.
  LIMIT_Z is an emergency stop too (SPEC 10.2: "stop at once if triggered by ... LIMIT_Z"), and
  answers with ``!alarm:LIMIT_Z`` only.
* A Z command while Z is moving replaces the running one (its ``done`` is never sent).
* ``violations`` records misuse the firmware would not refuse but the Commander must never do:
  a Z move while the joints are moving, or ``GRIP`` while Z is not at BOTTOM.
"""

from __future__ import annotations

import logging
import threading
import time
from collections import deque
from dataclasses import dataclass
from typing import Any, Callable, Deque, Optional

from config.protocol_commands import ACCEPTED_IN_ESTOP, CHECK_ORDER, PROTOCOL_VERSION, AlarmCode, CommandName, ErrorCode, ZState
from desktop_app.hardware.protocol import (
    Command,
    CommandError,
    decode_command,
    encode_alarm,
    encode_data_line,
    encode_done,
    encode_error,
    encode_ok,
)
from desktop_app.hardware.serial_link import (
    ConnectionLost,
    LineFramer,
    LoopbackPair,
    Transport,
    loopback_pair,
)
from desktop_app.utils.settings import ConfigError, Settings, get_settings, to_seconds

logger = logging.getLogger(__name__)

JointLimits = tuple[tuple[float, float], tuple[float, float]]  # ((alpha_min, alpha_max), (beta_min, beta_max))


@dataclass(frozen=True)
class SimConfig:
    """Everything the simulated firmware needs. No field has a default: build it with :meth:`from_settings`."""

    queue_capacity: int  # depth of the motion queue (serial.queue_depth)
    z_travel_s: float  # time for the full Z travel TOP <-> BOTTOM (electrical.z_axis.full_travel_ms, measured at assembly)
    heartbeat_timeout_s: float  # PING gap that stops the arm (timeouts.heartbeat_timeout_ms)
    home_joints_s: float  # time the joint homing takes (simulator.home_joints_ms)
    initial_joint_deg: tuple[float, float]  # joint angles at power-up (simulator.initial_alpha_deg / initial_beta_deg)
    z_timeout_factor: float  # per-move Z timeout = factor x full travel (electrical.z_axis.timeout_factor)
    pulses_per_rev: float  # 200 * f * i (drive section)
    joint_limits_deg: Optional[JointLimits]  # None: no joint limit is modelled (motion.joint_limits)
    max_speed_deg_s: Optional[float]  # F above it is error:2; None: no ceiling is modelled (motion.max_speed_deg_s)
    max_accel_deg_s2: Optional[float]  # ACCEL S above it is error:2; None: no ceiling is modelled (motion.max_accel_deg_s2)
    bench_move_without_homing: bool  # BENCH ONLY: a MOVE needs no homed flag (motion.bench_move_without_homing)
    initial_z: ZState  # simulator.initial_z
    firmware_version: str  # simulator.firmware_version
    poll_s: float  # how often the serving thread wakes to run its timers (simulator.poll_ms)
    max_line_bytes: int  # serial.max_line_bytes

    def __post_init__(self) -> None:
        if self.queue_capacity < 1:
            raise ValueError("queue_capacity must be >= 1")
        for name in ("z_travel_s", "heartbeat_timeout_s", "poll_s", "pulses_per_rev"):
            if not getattr(self, name) > 0:
                raise ValueError(f"{name} must be > 0")
        if self.home_joints_s < 0:
            raise ValueError("home_joints_s must be >= 0")
        if self.z_timeout_factor < 1.0:
            raise ValueError("z_timeout_factor must be >= 1 (the full travel has to fit)")
        if self.initial_z not in (ZState.TOP, ZState.BOTTOM):
            raise ValueError("initial_z must be TOP or BOTTOM")
        if not self.firmware_version or any(ch.isspace() for ch in self.firmware_version):
            raise ValueError("firmware_version must be one token")

    @classmethod
    def from_settings(
        cls,
        config: Optional[Settings] = None,
        *,
        queue_capacity: Optional[int] = None,
        z_travel_s: Optional[float] = None,
        heartbeat_timeout_s: Optional[float] = None,
        home_joints_s: Optional[float] = None,
        initial_joint_deg: Optional[tuple[float, float]] = None,
        **override: object,
    ) -> "SimConfig":
        """A config filled from config/settings.yaml; an argument overrides the file.

        Raises:
            ConfigError: a value is neither passed nor set in settings.yaml (it is null there).
        """
        cfg = config if config is not None else get_settings()

        def pick(name: str, given: Optional[Any], from_file: Optional[Any], key: str) -> Any:
            value = given if given is not None else from_file
            if value is None:
                raise ConfigError(f"SimConfig needs {name}: pass it, or give {key} a value in config/settings.yaml")
            return value

        sim = cfg.simulator
        joints = initial_joint_deg
        if joints is None and sim.initial_alpha_deg is not None and sim.initial_beta_deg is not None:
            joints = (sim.initial_alpha_deg, sim.initial_beta_deg)
        limits = cfg.motion.joint_limits
        values: dict[str, Any] = dict(
            queue_capacity=pick("queue_capacity", queue_capacity, cfg.serial.queue_depth, "serial.queue_depth"),
            z_travel_s=pick(
                "z_travel_s",
                z_travel_s,
                to_seconds(cfg.electrical.z_axis.full_travel_ms) if cfg.electrical.z_axis.full_travel_ms is not None else None,
                "electrical.z_axis.full_travel_ms",
            ),
            heartbeat_timeout_s=pick(
                "heartbeat_timeout_s", heartbeat_timeout_s, to_seconds(cfg.timeouts.heartbeat_timeout_ms), "timeouts.heartbeat_timeout_ms"
            ),
            home_joints_s=pick(
                "home_joints_s",
                home_joints_s,
                to_seconds(sim.home_joints_ms) if sim.home_joints_ms is not None else None,
                "simulator.home_joints_ms",
            ),
            initial_joint_deg=pick("initial_joint_deg", joints, None, "simulator.initial_alpha_deg and initial_beta_deg"),
            z_timeout_factor=cfg.electrical.z_axis.timeout_factor,
            pulses_per_rev=cfg.drive.pulses_per_rev,
            joint_limits_deg=(
                ((limits.alpha_min_deg, limits.alpha_max_deg), (limits.beta_min_deg, limits.beta_max_deg))
                if limits.configured
                else None
            ),
            # A ceiling that is null in settings.yaml is not modelled (None); the firmware refuses every MOVE then.
            max_speed_deg_s=cfg.motion.max_speed_deg_s,
            max_accel_deg_s2=cfg.motion.max_accel_deg_s2,
            bench_move_without_homing=cfg.motion.bench_move_without_homing,
            initial_z=ZState(sim.initial_z),
            firmware_version=sim.firmware_version,
            poll_s=to_seconds(sim.poll_ms),
            max_line_bytes=cfg.serial.max_line_bytes,
        )
        values.update(override)
        return cls(**values)

    @property
    def z_timeout_s(self) -> float:
        return self.z_timeout_factor * self.z_travel_s


@dataclass(frozen=True)
class TraceEvent:
    """Something physical the arm did, with where the joints were: used to replay what was picked and dropped.

    ``kind`` is ``"z_bottom"`` (Z reached the bottom limit), ``"z_top"`` (Z reached the top limit) or
    ``"grip"`` (the servo was commanded; ``grip_deg`` is its angle).
    """

    kind: str
    joints_deg: tuple[float, float]
    grip_deg: float


@dataclass
class _Segment:
    """The MOVE being executed: a straight interpolation of both joints in time."""

    start: tuple[float, float]
    end: tuple[float, float]
    t0: float
    t1: float


@dataclass
class _ZMove:
    kind: str  # "ZUP", "ZDOWN" or "HOME" (the Z part of homing)
    target: float  # 0.0 = TOP, 1.0 = BOTTOM
    started: float
    deadline: float


@dataclass
class _HomeJob:
    joints: bool  # the joint phase is wanted
    joints_end: Optional[float] = None  # set when the joint phase has started


_Z_TOP, _Z_BOTTOM = 0.0, 1.0
# The clock is floating point, so "started + 0.5" and the clock value that means 0.5 s later can differ by
# one ulp. Time comparisons carry this much slack; it is float hygiene, not a physical tolerance.
_TIME_SLACK_S = 1e-9


def _reached(now: float, moment: float) -> bool:
    """True when ``now`` is at or after ``moment``."""
    return now + _TIME_SLACK_S >= moment

# The commands accepted while the ESTOP flag is set (SPEC 5.1). Taken from the declaration, so the simulator
# and the firmware (CB_ESTOP_ACCEPTED in protocol.h, cross-checked by the tests) cannot drift apart.
_ALWAYS_ALLOWED_IN_ESTOP = frozenset(ACCEPTED_IN_ESTOP)
_HOME, _ZUP, _ZDOWN = CommandName.HOME.value, CommandName.ZUP.value, CommandName.ZDOWN.value
_MOVE = CommandName.MOVE.value


class Esp32Simulator:
    """The simulated firmware. Serve it with :meth:`start` (a thread) or step it with :meth:`pump`."""

    def __init__(
        self,
        config: SimConfig,
        transport: Transport,
        *,
        cable: Optional[LoopbackPair] = None,
        clock: Callable[[], float] = time.monotonic,
    ) -> None:
        self.config = config
        self._transport = transport
        self._cable = cable
        self._clock = clock
        self._lock = threading.RLock()
        self._framer = LineFramer(config.max_line_bytes)
        self._thread: Optional[threading.Thread] = None
        self._stop = threading.Event()
        self._link_up = False

        self._joint_home = config.initial_joint_deg
        self._joints: tuple[float, float] = config.initial_joint_deg
        self._segment: Optional[_Segment] = None
        self._queue: Deque[tuple[float, float, float]] = deque()
        self._z_pos = _Z_TOP if config.initial_z == ZState.TOP else _Z_BOTTOM  # ==, not is: ZState is a str enum and a test may pass "TOP"
        self._z_move: Optional[_ZMove] = None
        self._z_stall = False
        self._home_job: Optional[_HomeJob] = None
        self._homed_z = False
        self._homed_joints = False
        self._waiters = 0  # WAIT commands that still owe a ``done``
        self._accel = 0.0
        self._grip_deg = 0.0
        self._drivers_enabled = True  # at boot the pins float, so the drivers are enabled (SPEC 10.2)
        self._estop = False
        self._latched: list[AlarmCode] = []
        self._sensor_offset = (0.0, 0.0)
        self._sensor_fault = False
        self._hb_armed = False
        self._hb_last = 0.0
        self.received: list[str] = []  # every non-blank line received, in order
        self.sent: list[str] = []  # every line sent (also those lost on a dead cable)
        self.violations: list[str] = []
        self.trace: list[TraceEvent] = []
        self._hooks: list[Callable[[str], None]] = []
        # One predicate per ErrorCode that is a gate after parsing; True = refuse with that code. The order comes
        # from CHECK_ORDER (SPEC 5.2), not from this table.
        self._gates: dict[ErrorCode, Callable[[Command, float], bool]] = {
            ErrorCode.BAD_PARAMETER: self._gate_out_of_limits,
            ErrorCode.IN_ESTOP: self._gate_estop,
            ErrorCode.DRIVERS_DISABLED: self._gate_drivers,
            ErrorCode.NOT_HOMED: self._gate_not_homed,
            ErrorCode.QUEUE_FULL: self._gate_queue_full,
        }

    # ------------------------------------------------------------------ #
    # Read-only view for tests
    # ------------------------------------------------------------------ #

    @property
    def homed(self) -> bool:
        return self._homed_z and self._homed_joints

    @property
    def z_state(self) -> ZState:
        with self._lock:
            return self._z_state(self._clock())

    @property
    def queue_length(self) -> int:
        with self._lock:
            return len(self._queue) + (1 if self._segment is not None else 0)

    @property
    def joints_moving(self) -> bool:
        return self.queue_length > 0

    @property
    def estop(self) -> bool:
        return self._estop

    @property
    def alarms(self) -> tuple[AlarmCode, ...]:
        """Alarms latched since the last RESET, in the order they were raised."""
        return tuple(self._latched)

    @property
    def joint_deg(self) -> tuple[float, float]:
        with self._lock:
            return self._joint_now(self._clock())

    @property
    def grip_deg(self) -> float:
        return self._grip_deg

    @property
    def drivers_enabled(self) -> bool:
        return self._drivers_enabled

    @property
    def link_up(self) -> bool:
        return self._link_up

    @property
    def heartbeat_armed(self) -> bool:
        return self._hb_armed

    def add_command_hook(self, hook: Callable[[str], None]) -> None:
        """Call ``hook(line)`` right after each received command has been handled.

        It runs in the serving thread, so a hook may call the ``inject_*`` methods to put a fault at an
        exact point of the command stream (no timing race). A hook must not block.
        """
        with self._lock:
            self._hooks.append(hook)

    # ------------------------------------------------------------------ #
    # Serving
    # ------------------------------------------------------------------ #

    def start(self) -> None:
        """Open the device end and serve in a background thread."""
        if self._thread is not None:
            raise RuntimeError("the simulator is already running")
        self.open_link()
        self._stop.clear()
        self._thread = threading.Thread(target=self._serve, name="esp32-simulator", daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._stop.set()
        self._transport.close()
        thread = self._thread
        if thread is not None and thread is not threading.current_thread():
            thread.join(timeout=to_seconds(get_settings().timeouts.thread_join_ms))
        self._thread = None
        self._link_up = False

    def __enter__(self) -> "Esp32Simulator":
        self.start()
        return self

    def __exit__(self, *_exc: object) -> None:
        self.stop()

    def open_link(self) -> None:
        """Open the device end of the cable. Fails quietly (link_up stays False) while it is unplugged."""
        try:
            self._transport.open()
        except ConnectionLost:
            self._link_up = False
            return
        self._framer.reset()
        self._link_up = True

    def _serve(self) -> None:
        while not self._stop.is_set():
            self.pump(read_timeout_s=self.config.poll_s)
            if not self._link_up:
                self._stop.wait(self.config.poll_s)  # nothing to block on while the cable is out

    def pump(self, *, read_timeout_s: float = 0.0) -> None:
        """Reopen the link if the cable is back, read what arrived, handle it, and run the timers.

        Time-dependent state is brought up to the clock's now on every call, with or without a link.
        """
        if not self._link_up and not self._stop.is_set():
            self.open_link()
        data = b""
        if self._link_up:
            try:
                data = self._transport.read(read_timeout_s)
            except ConnectionLost:
                self._link_up = False
                logger.info("simulator: the cable is gone")
        with self._lock:
            for item in self._framer.feed(data):
                if item.text is not None:
                    self._handle_line(item.text)
                else:
                    self._send(encode_error(ErrorCode.UNRECOGNIZED))
            self._tick(self._clock())

    # ------------------------------------------------------------------ #
    # Output
    # ------------------------------------------------------------------ #

    def _send(self, line: str) -> None:
        self.sent.append(line)
        if not self._link_up:
            logger.debug("simulator: dropped %r (no link)", line)
            return
        try:
            self._transport.write((line + "\n").encode("ascii"))
        except ConnectionLost:
            self._link_up = False

    def _error(self, code: ErrorCode) -> None:
        self._send(encode_error(code))

    # ------------------------------------------------------------------ #
    # State helpers
    # ------------------------------------------------------------------ #

    def _z_state(self, now: float) -> ZState:
        position = self._z_now(now)
        if position <= _Z_TOP:
            return ZState.TOP
        if position >= _Z_BOTTOM:
            return ZState.BOTTOM
        return ZState.MID

    def _z_now(self, now: float) -> float:
        """Z position now (0 = TOP, 1 = BOTTOM), following the running move at full travel speed."""
        move = self._z_move
        if move is None:
            return self._z_pos
        if self._z_stall:
            return self._z_pos
        step = (now - move.started) / self.config.z_travel_s
        start = self._z_pos
        return min(move.target, start + step) if move.target > start else max(move.target, start - step)

    def _joint_now(self, now: float) -> tuple[float, float]:
        seg = self._segment
        if seg is None:
            return self._joints
        if _reached(now, seg.t1) or seg.t1 <= seg.t0:
            return seg.end
        frac = (now - seg.t0) / (seg.t1 - seg.t0)
        return (seg.start[0] + (seg.end[0] - seg.start[0]) * frac, seg.start[1] + (seg.end[1] - seg.start[1]) * frac)

    def _steps(self, angle: float, home: float) -> int:
        return int(round(self.config.pulses_per_rev * (angle - home) / 360.0))

    def _latch(self, code: AlarmCode) -> None:
        if code not in self._latched:
            self._latched.append(code)

    # ------------------------------------------------------------------ #
    # Commands
    # ------------------------------------------------------------------ #

    def _handle_line(self, line: str) -> None:
        try:
            command = decode_command(line)
        except CommandError as exc:
            self.received.append(line.strip())
            logger.debug("simulator: %r -> error:%d (%s)", line, int(exc.code), exc)
            self._error(exc.code)
            return
        if command is None:
            return
        self.received.append(line.strip())
        now = self._clock()
        self._tick(now)  # bring time-dependent state up to date before looking at it
        for code in CHECK_ORDER:  # the one order of SPEC 5.2; the first check that fails decides the reply
            gate = self._gates.get(code)  # UNRECOGNIZED was decided by decode_command above
            if gate is not None and gate(command, now):
                self._error(code)
                return
        getattr(self, f"_cmd_{command.name.lower()}")(command, now)
        for hook in list(self._hooks):
            hook(line.strip())

    # ------------------------------------------------------------------ #
    # Gates (SPEC 5.2, "Order of checks"): True means "refuse with this code"
    # ------------------------------------------------------------------ #

    @staticmethod
    def _wants(command: Command) -> tuple[bool, bool]:
        """(Z part, joint part) of a HOME: no flag means both."""
        want_z = bool(command.params.get("Z")) or not command.params.get("AB")
        want_joints = bool(command.params.get("AB")) or not command.params.get("Z")
        return want_z, want_joints

    def _needs_joint_drivers(self, command: Command) -> bool:
        if command.name == _MOVE:
            return True
        return command.name == _HOME and self._wants(command)[1]

    def _gate_out_of_limits(self, command: Command, _now: float) -> bool:
        """Check 2 beyond the decoder (SPEC 5.2): the joint soft limits, the speed and the acceleration ceilings."""
        config = self.config
        if command.name == CommandName.ACCEL.value:
            return config.max_accel_deg_s2 is not None and float(command.params["S"]) > config.max_accel_deg_s2
        if command.name != _MOVE:
            return False
        if config.max_speed_deg_s is not None and float(command.params["F"]) > config.max_speed_deg_s:
            return True
        limits = config.joint_limits_deg
        if limits is None:
            return False
        a, b = float(command.params["A"]), float(command.params["B"])
        return not (limits[0][0] <= a <= limits[0][1] and limits[1][0] <= b <= limits[1][1])

    def _gate_estop(self, command: Command, _now: float) -> bool:
        return self._estop and command.name not in _ALWAYS_ALLOWED_IN_ESTOP

    def _gate_drivers(self, command: Command, _now: float) -> bool:
        return self._needs_joint_drivers(command) and not self._drivers_enabled

    def _gate_not_homed(self, command: Command, now: float) -> bool:
        if command.name == _MOVE:
            homing_missing = not self.homed and not self.config.bench_move_without_homing  # BENCH ONLY skips this part
            return homing_missing or self._z_state(now) is not ZState.TOP or self._z_move is not None
        if command.name == _HOME:
            want_z, want_joints = self._wants(command)
            if want_joints and not want_z and self._z_state(now) is not ZState.TOP:
                return True  # the joints may only be homed with Z at TOP
            return want_joints and self._sensor_fault
        return False

    def _gate_queue_full(self, command: Command, _now: float) -> bool:
        return command.name == _MOVE and self.queue_length >= self.config.queue_capacity  # the command is NOT queued

    def _ok(self) -> None:
        self._send(encode_ok())

    def _cmd_ping(self, _command: Command, now: float) -> None:
        self._hb_armed = True
        self._hb_last = now
        self._ok()

    def _cmd_info(self, _command: Command, _now: float) -> None:
        self._send(encode_data_line(CommandName.INFO.value, FW=self.config.firmware_version, PROTO=PROTOCOL_VERSION))
        self._ok()

    def _cmd_pos(self, _command: Command, now: float) -> None:
        a, b = self._joint_now(now)
        off_a, off_b = self._sensor_offset
        self._send(
            encode_data_line(
                CommandName.POS.value,
                A=a,
                B=b,
                SA=a + off_a,
                SB=b + off_b,
                NA=self._steps(a, self._joint_home[0]),
                NB=self._steps(b, self._joint_home[1]),
                Z=self._z_state(now),
            )
        )
        self._ok()

    def _cmd_accel(self, command: Command, _now: float) -> None:
        self._accel = float(command.params["S"])
        self._ok()

    def _cmd_grip(self, command: Command, now: float) -> None:
        if self._z_state(now) is not ZState.BOTTOM:
            self.violations.append(f"GRIP with Z {self._z_state(now).value}")
        self._grip_deg = float(command.params["S"])
        self.trace.append(TraceEvent("grip", self._joint_now(now), self._grip_deg))
        self._ok()

    def _cmd_enable(self, _command: Command, _now: float) -> None:
        self._drivers_enabled = True
        self._ok()

    def _cmd_disable(self, _command: Command, now: float) -> None:
        # SPEC 5.1, "DISABLE rules": stop the pulses and clear the whole queue (queued MOVEs are discarded, a pending
        # WAIT loses its done), then the drivers, then the homed flags.
        self._clear_motion(now)
        self._drivers_enabled = False
        self._homed_z = self._homed_joints = False  # homing is mandatory afterwards (SPEC 5.1)
        if self._home_job is not None and self._home_job.joints:
            self._freeze_z(now)  # a homing that still needs the joints cannot complete without the drivers
        self._ok()

    def _cmd_move(self, command: Command, now: float) -> None:
        a, b, speed = (float(command.params[key]) for key in ("A", "B", "F"))
        self._queue.append((a, b, speed))
        self._ok()
        self._tick(now)

    def _cmd_wait(self, _command: Command, now: float) -> None:
        self._ok()
        self._waiters += 1
        self._tick(now)

    def _cmd_zstop(self, _command: Command, now: float) -> None:
        self._freeze_z(now)
        self._ok()

    def _cmd_zup(self, _command: Command, now: float) -> None:
        self._start_z(_ZUP, _Z_TOP, now)

    def _cmd_zdown(self, _command: Command, now: float) -> None:
        self._start_z(_ZDOWN, _Z_BOTTOM, now)

    def _start_z(self, kind: str, target: float, now: float) -> None:
        if self.joints_moving:
            self.violations.append(f"{kind} while the joints are moving")
        if self._z_move is not None:
            self._freeze_z(now)  # the new command replaces the running one
        self._ok()
        self._z_move = _ZMove(kind, target, now, now + self.config.z_timeout_s)
        self._tick(now)

    def _freeze_z(self, now: float) -> None:
        self._z_pos = self._z_now(now)
        self._z_move = None
        if self._home_job is not None:
            self._home_job = None  # a stopped Z aborts homing: it never completes

    def _cmd_home(self, command: Command, now: float) -> None:
        want_z, want_joints = self._wants(command)  # the gates have run: drivers (6) and not-homed (4) are fine
        self._ok()
        self._clear_motion(now)
        self._freeze_z(now)  # a Z move already running is replaced (this also drops an older homing job)
        self._home_job = _HomeJob(joints=want_joints)
        if want_z:
            self._homed_z = False
            self._z_move = _ZMove(_HOME, _Z_TOP, now, now + self.config.z_timeout_s)
        else:  # joints only: Z is already at TOP, so the joint phase starts now
            self._home_job.joints_end = now + self.config.home_joints_s
        if want_joints:
            self._homed_joints = False
        self._tick(now)

    def _cmd_reset(self, _command: Command, now: float) -> None:
        self._clear_motion(now)
        self._freeze_z(now)
        self._estop = False
        self._latched.clear()
        self._sensor_fault = False
        self._z_stall = False
        self._sensor_offset = (0.0, 0.0)
        self._homed_z = self._homed_joints = False  # homing is mandatory afterwards
        self._ok()

    def _cmd_estop(self, _command: Command, now: float) -> None:
        self._emergency_stop(AlarmCode.ESTOP, now)

    # ------------------------------------------------------------------ #
    # Stopping
    # ------------------------------------------------------------------ #

    def _clear_motion(self, now: float) -> None:
        """Stop both joints where they are and forget the queue and every owed ``done`` of the joints.

        The one place that clears the queue (DISABLE, ESTOP, RESET, the heartbeat, a new HOME, a sensor fault):
        a WAIT that is pending is cancelled with it and never sends its ``done`` (SPEC 5.1, "Cancelled done").
        """
        self._joints = self._joint_now(now)
        self._segment = None
        self._queue.clear()
        self._waiters = 0

    def _emergency_stop(self, alarm: AlarmCode, now: float) -> None:
        self._clear_motion(now)
        self._freeze_z(now)
        self._estop = True  # drivers stay enabled (SPEC 10.2)
        self._homed_z = self._homed_joints = False
        self._latch(alarm)
        self._send(encode_alarm(alarm))

    # ------------------------------------------------------------------ #
    # Timers
    # ------------------------------------------------------------------ #

    def _tick(self, now: float) -> None:
        self._tick_heartbeat(now)
        self._tick_z(now)
        self._tick_home(now)
        self._tick_motion(now)
        if self._waiters and self._segment is None and not self._queue:
            owed, self._waiters = self._waiters, 0
            for _ in range(owed):
                self._send(encode_done())

    def _tick_heartbeat(self, now: float) -> None:
        if not self._hb_armed or now - self._hb_last <= self.config.heartbeat_timeout_s + _TIME_SLACK_S:
            return
        # PC went silent: controlled stop, brake Z, KEEP the drivers enabled, wait for re-homing.
        self._hb_armed = False
        self._clear_motion(now)
        self._freeze_z(now)
        self._homed_z = self._homed_joints = False
        self._latch(AlarmCode.HB_TIMEOUT)
        logger.info("simulator: heartbeat timeout")
        self._send(encode_alarm(AlarmCode.HB_TIMEOUT))

    def _tick_z(self, now: float) -> None:
        move = self._z_move
        if move is None:
            return
        if self._z_stall:
            if _reached(now, move.deadline):
                self._z_move = None  # Z stays where it stalled (_z_pos was frozen when the stall began)
                self._home_job = None
                self._latch(AlarmCode.Z_TIMEOUT)
                self._send(encode_alarm(AlarmCode.Z_TIMEOUT))
            return
        finished_at = move.started + abs(move.target - self._z_pos) * self.config.z_travel_s
        if not _reached(now, finished_at):
            return
        self._z_pos = move.target
        self._z_move = None
        if move.target == _Z_BOTTOM:
            self.trace.append(TraceEvent("z_bottom", self._joint_now(now), self._grip_deg))
        elif move.kind != _HOME:
            self.trace.append(TraceEvent("z_top", self._joint_now(now), self._grip_deg))
        if move.kind == _HOME:
            self._homed_z = True
            job = self._home_job
            if job is not None and job.joints:
                job.joints_end = finished_at + self.config.home_joints_s  # from when Z arrived, not from this tick
            else:
                self._home_job = None
                self._send(encode_done())
        else:
            self._send(encode_done())

    def _tick_home(self, now: float) -> None:
        job = self._home_job
        if job is None or job.joints_end is None or not _reached(now, job.joints_end):
            return
        self._homed_joints = True
        self._home_job = None
        self._send(encode_done())

    def _tick_motion(self, now: float) -> None:
        while True:
            seg = self._segment
            if seg is not None:
                if not _reached(now, seg.t1):
                    return
                self._joints = seg.end
                self._segment = None
                start_at = seg.t1  # chain back to back so the queue runs at the commanded speed
            elif self._queue:
                start_at = now
            else:
                return
            if not self._queue:
                return
            a, b, speed = self._queue.popleft()
            begin = self._joints
            duration = max(abs(a - begin[0]), abs(b - begin[1])) / speed
            self._segment = _Segment(begin, (a, b), start_at, start_at + duration)

    # ------------------------------------------------------------------ #
    # Fault injection (what the real world does to the firmware)
    # ------------------------------------------------------------------ #

    def inject_estop(self) -> None:
        """The hardware emergency button: the same as the ESTOP command, but nobody asked for it."""
        with self._lock:
            self._emergency_stop(AlarmCode.ESTOP, self._clock())

    def inject_limit_z(self) -> None:
        """Both Z switches active (or the top switch released without a command): stop at once."""
        with self._lock:
            self._emergency_stop(AlarmCode.LIMIT_Z, self._clock())

    def inject_z_stall(self) -> None:
        """Z stops making progress (stuck, no limit switch reached): ``Z_TIMEOUT`` fires after the timeout.

        Applies to the Z move running now, or to the next one.
        """
        with self._lock:
            now = self._clock()
            if self._z_move is not None:
                self._z_pos = self._z_now(now)
                self._z_move.started = now
            self._z_stall = True

    def inject_sensor_fault(self) -> None:
        """An AS5600 reading is out of range: stop the joints, homing is mandatory."""
        with self._lock:
            now = self._clock()
            self._clear_motion(now)
            self._homed_joints = False
            self._sensor_fault = True
            self._latch(AlarmCode.SENSOR_FAULT)
            self._send(encode_alarm(AlarmCode.SENSOR_FAULT))

    def inject_step_loss(self, offset_deg: float) -> None:
        """The AS5600 reading drifts ``offset_deg`` from the commanded angle: lost steps."""
        with self._lock:
            now = self._clock()
            self._clear_motion(now)
            self._homed_joints = False
            self._sensor_offset = (offset_deg, 0.0)
            self._latch(AlarmCode.STEP_LOSS)
            self._send(encode_alarm(AlarmCode.STEP_LOSS))

    def inject_lost_connection(self) -> None:
        """Pull the USB cable. Needs the loopback pair the simulator was built with."""
        if self._cable is None:
            raise RuntimeError("this simulator was built without a cable to cut")
        self._cable.cut()

    def restore_connection(self) -> None:
        """Plug the cable back in; the simulator reopens its end by itself."""
        if self._cable is None:
            raise RuntimeError("this simulator was built without a cable to restore")
        self._cable.restore()


def make_loopback_simulator(
    config: SimConfig, *, chunk_size: Optional[int] = None, clock: Callable[[], float] = time.monotonic
) -> tuple[Esp32Simulator, LoopbackPair]:
    """A simulator plugged into the device end of a new cable; give ``pair.pc`` to the SerialLink."""
    pair = loopback_pair(chunk_size)
    return Esp32Simulator(config, pair.device, cable=pair, clock=clock), pair
