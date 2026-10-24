"""Robot Commander: Move Planner chains to protocol commands, heartbeat, alarms (SPEC 5.3, 6.4, 7.2).

What it does
------------
* Translates a :class:`MoveChain` into commands (:func:`build_plan`): one ``MOVE`` per trajectory
  point, ``WAIT`` to drain the motion queue before every Z move and every ``GRIP``, ``ZDOWN`` /
  ``ZUP`` with their ``done``, ``GRIP`` with a settle time. The plan checks itself: MOVE only with Z
  at TOP, GRIP only with Z at BOTTOM, Z never moves while MOVEs are still queued.
* Sends the commands through the :class:`SerialLink`, waits for every ``ok`` and for every ``done``, and stops at
  the first problem (MOVEs one at a time, see below).
* Sends ``PING`` periodically in its own thread, bypassing the in-flight window so the heartbeat can
  never queue behind a saturated MOVE stream (SPEC 5.3, 10.2).
* Publishes every alarm on the Event Bus as an FSM ``ALARM`` request, BEFORE it releases the thread
  that was waiting on the chain, so when ``execute_chain`` raises the FSM is already in ERROR_RECOVER.

The placeholder guard (SPEC 7.2, "Rules clarified in phase 2")
--------------------------------------------------------------
Through a real serial port the Commander refuses to send a pick-and-place chain (or the park leg)
while ``MovePlanner.placeholders()`` is not empty: grasp angles, reserve slot contents and the
capture/reserve/park layout are [TO CONFIRM]. The refusal comes before any byte is sent.
The in-process loopback transport is not subject to the guard. A transport that does not say it is
simulated counts as real. The operator may explicitly enable the no-load bench mode
(:meth:`enable_bench_mode`): it logs a loud warning, allows single commands one at a time
(:meth:`bench_command`) and STILL refuses every chain, so a full pick-and-place never runs on
placeholder values.

Decisions the SPEC leaves open (reported in the phase 3 summary)
-----------------------------------------------------------------
* Any alarm, or a lost link, makes homing mandatory for the Commander (and a ``RESET`` goes before
  the next ``HOME``). The FSM agrees: every firmware alarm is in ``HOMING_MANDATORY_ALARMS`` (SPEC 5.3, 7.2).
* MOVEs are sent one at a time, each waited for, and a MOVE refused with ``error:3`` is sent again after a
  pause: ``ok`` means "queued", so the in-flight limit (SPEC 5.3) cannot bound the firmware queue, and with
  several MOVEs in flight a refused one could be followed by an accepted one (out of order).
* Every deadline and the PING schedule read an injectable ``clock`` (default ``time.monotonic``).
* A lost serial link is reported to the FSM as ``HB_TIMEOUT``: the ESP32 will have hit its own
  heartbeat timeout, and it is the same consequence (stopped, homing mandatory).
* An alarm code that is not declared triggers an emergency stop at once.
* ``HOME`` leaves the arm where it is (the AS5600 gives the absolute angle), and the planner assumes
  every chain starts at the park position. So before a chain the Commander checks ``POS`` against the
  chain's first point and refuses to start when the arm is somewhere else.
* ``F`` is sent in degrees per second.

No Qt and no gui imports here; only ``core.event_bus`` and ``core.fsm`` (for the request type) are
used, and only this module of the ``hardware`` package touches the core.
"""

from __future__ import annotations

import logging
import threading
import time
from collections import deque
from dataclasses import dataclass
from enum import Enum
from typing import Callable, Deque, Optional, Protocol, Union

from config.protocol_commands import COMMANDS, AlarmCode, CommandName, Completion, ErrorCode, ZState
from desktop_app.core.event_bus import EventBus
from desktop_app.core.fsm import FsmRequest, Trigger
from desktop_app.core.move_planner import GripStep, MoveChain, MoveStep, Step, ZDownStep, ZUpStep
from desktop_app.hardware.protocol import (
    Alarm,
    Command,
    DataLine,
    Done,
    EncodeError,
    ErrorResponse,
    Ok,
)
from desktop_app.hardware.serial_link import (
    BadLine,
    Connected,
    Disconnected,
    LinkDownError,
    LinkError,
    LinkMessage,
    LinkState,
    Reply,
    SerialLink,
    Unsolicited,
)

logger = logging.getLogger(__name__)

_JOIN_TIMEOUT_S = 3.0  # how long stop() waits for a worker thread
_CLOCK_POLL_S = 0.05  # waits wake this often to re-read the clock, so an injected clock can end them

_PING = CommandName.PING.value
_WAIT = CommandName.WAIT.value
_POS = CommandName.POS.value
_MOVE = CommandName.MOVE.value
_ZUP = CommandName.ZUP.value
_ZDOWN = CommandName.ZDOWN.value
_GRIP = CommandName.GRIP.value
_HOME = CommandName.HOME.value
_RESET = CommandName.RESET.value
_ESTOP = CommandName.ESTOP.value
_ACCEL = CommandName.ACCEL.value


# --------------------------------------------------------------------------- #
# Errors
# --------------------------------------------------------------------------- #


class CommanderError(RuntimeError):
    """Base class of every Robot Commander error."""


class GuardRefusedError(CommanderError):
    """A chain was refused because the planner still holds placeholders and the port is real."""

    def __init__(self, message: str, placeholders: tuple[str, ...]) -> None:
        super().__init__(message)
        self.placeholders = placeholders


class BenchModeError(CommanderError):
    """Bench mode is off (for a bench command) or on (for a chain, which it never runs)."""


class NotReadyError(CommanderError):
    """The Commander cannot start the operation: link down, homing required, arm not at the start point."""


class NotAtStartError(NotReadyError):
    """The arm is not where the chain starts (the park position)."""


class PlanError(CommanderError):
    """A chain cannot be turned into a safe command sequence. A bug in the caller or the planner."""


class RobotAlarmError(CommanderError):
    """An alarm (or a lost link) stopped the operation. The FSM has already been told."""

    def __init__(self, code: str, detail: str = "") -> None:
        super().__init__(f"alarm {code}" + (f": {detail}" if detail else ""))
        self.code = code
        self.detail = detail


class CommandFailedError(CommanderError):
    """The ESP32 answered a command with ``error:<code>``."""

    def __init__(self, command: Command, code: ErrorCode) -> None:
        super().__init__(f"{command.encode()} was refused with error:{int(code)} ({code.name})")
        self.command = command
        self.code = code


class ReplyTimeoutError(CommanderError):
    """The ESP32 did not answer in time."""


# --------------------------------------------------------------------------- #
# Settings
# --------------------------------------------------------------------------- #


@dataclass(frozen=True)
class CommanderSettings:
    """Every value here is a setting, not a constant. The [TO CONFIRM] ones have no default."""

    move_speed_deg_s: float  # F of every MOVE ([TO CONFIRM])
    ping_interval_s: float  # must be well below the firmware heartbeat threshold ([TO CONFIRM])
    reply_timeout_s: float  # how long an ``ok`` may take
    done_timeout_s: float  # how long a ``done`` may take (HOME, ZUP, ZDOWN, WAIT): above the Z timeout
    grip_settle_s: float  # the servo gives no feedback: wait this long after a GRIP ([TO CONFIRM])
    start_tolerance_deg: float  # how far the arm may be from the chain's first point (AS5600 is about +/-1 deg)
    send_timeout_s: float  # how long a send may wait for a free in-flight slot
    queue_retry_s: float  # pause before a MOVE refused with error:3 (queue full) is sent again
    queue_full_timeout_s: float  # give up (and stop the chain) when the queue stays full this long
    acceleration: Optional[float] = None  # ACCEL S= sent before each chain, if set ([TO CONFIRM])

    def __post_init__(self) -> None:
        for name in (
            "move_speed_deg_s",
            "ping_interval_s",
            "reply_timeout_s",
            "done_timeout_s",
            "start_tolerance_deg",
            "send_timeout_s",
            "queue_retry_s",
            "queue_full_timeout_s",
        ):
            value = getattr(self, name)
            if not value > 0:
                raise ValueError(f"{name} must be > 0, got {value!r}")
        if self.grip_settle_s < 0:
            raise ValueError("grip_settle_s must be >= 0")
        if self.acceleration is not None and not self.acceleration > 0:
            raise ValueError("acceleration must be > 0 or None")


class PlaceholderSource(Protocol):
    """What the guard needs from the Move Planner."""

    def placeholders(self) -> tuple[str, ...]: ...


# --------------------------------------------------------------------------- #
# The plan: a chain as protocol commands
# --------------------------------------------------------------------------- #


class PlanKind(str, Enum):
    MOVE = "move"  # streamed, acknowledged by ok only
    DRAIN = "drain"  # WAIT: ok, then done once the motion queue is empty
    Z = "z"  # ZUP / ZDOWN: ok, then done
    GRIP = "grip"  # ok, then the servo settle time


@dataclass(frozen=True)
class PlanItem:
    kind: PlanKind
    command: Command


def build_plan(chain: MoveChain, settings: CommanderSettings) -> tuple[PlanItem, ...]:
    """The commands of a whole chain: its transfers, then nothing else (the park leg is separate).

    Raises:
        PlanError: the steps break a rule of the sequence (SPEC 6.4).
    """
    return _plan_steps(chain.steps, settings)


def build_park_plan(chain: MoveChain, settings: CommanderSettings) -> tuple[PlanItem, ...]:
    """The commands of the return trip from the last drop point to the park position."""
    return _plan_steps((chain.park_leg,), settings)


def chain_start_joints(chain: MoveChain) -> tuple[float, float]:
    """The joint angles of the first point of the chain: where the planner assumes the arm is."""
    return _first_point(chain.steps[0] if chain.steps else chain.park_leg)


def park_start_joints(chain: MoveChain) -> tuple[float, float]:
    return _first_point(chain.park_leg)


def _first_point(step: Step) -> tuple[float, float]:
    if not isinstance(step, MoveStep):
        raise PlanError(f"a chain must start with a MoveStep, got {type(step).__name__}")
    alpha, beta = step.trajectory.joints_deg[0]
    return float(alpha), float(beta)


def _plan_steps(steps: tuple[Step, ...], settings: CommanderSettings) -> tuple[PlanItem, ...]:
    items: list[PlanItem] = []
    z_down = False
    moves_pending = False

    def drain() -> None:
        nonlocal moves_pending
        if moves_pending:
            items.append(PlanItem(PlanKind.DRAIN, Command(_WAIT, {})))
            moves_pending = False

    for step in steps:
        if isinstance(step, MoveStep):
            if z_down:
                raise PlanError("MOVE with Z not at TOP")
            # Point 0 is where the arm already is (the end of the previous leg, or the checked start).
            for alpha, beta in step.trajectory.joints_deg[1:]:
                command = Command(_MOVE, {"A": float(alpha), "B": float(beta), "F": settings.move_speed_deg_s})
                items.append(PlanItem(PlanKind.MOVE, command))
                moves_pending = True
        elif isinstance(step, ZDownStep):
            if z_down:
                raise PlanError("ZDOWN with Z already down")
            drain()  # Z must never move while the joints still do
            items.append(PlanItem(PlanKind.Z, Command(_ZDOWN, {})))
            z_down = True
        elif isinstance(step, ZUpStep):
            if not z_down:
                raise PlanError("ZUP with Z already at TOP")
            items.append(PlanItem(PlanKind.Z, Command(_ZUP, {})))
            z_down = False
        elif isinstance(step, GripStep):
            if not z_down:
                raise PlanError("GRIP with Z not at BOTTOM")
            items.append(PlanItem(PlanKind.GRIP, Command(_GRIP, {"S": float(step.angle_deg)})))
        else:
            raise PlanError(f"unknown step {step!r}")
    if z_down:
        raise PlanError("the chain ends with Z down")
    drain()
    return tuple(items)


# --------------------------------------------------------------------------- #
# Alarms to the FSM
# --------------------------------------------------------------------------- #


def publish_alarm(bus: EventBus, code: str) -> None:
    """Tell the FSM about an alarm: a request with the ALARM trigger (the FSM decides what follows)."""
    bus.publish(FsmRequest(Trigger.ALARM, {"alarm": code}))


# --------------------------------------------------------------------------- #
# The commander
# --------------------------------------------------------------------------- #


@dataclass(eq=False)
class _Request:
    """One command sent and not yet fully answered."""

    command: Command
    wants_done: bool
    acknowledged: bool = False
    finished: bool = False  # the done arrived (or it never needed one)
    error: Optional[ErrorCode] = None
    data: Optional[DataLine] = None

    @property
    def answered(self) -> bool:
        return self.acknowledged or self.error is not None


class RobotCommander:
    """Drives the robot through a :class:`SerialLink`. One operation at a time; alarms and PING any time."""

    def __init__(
        self,
        link: SerialLink,
        planner: PlaceholderSource,
        settings: CommanderSettings,
        bus: Optional[EventBus] = None,
        clock: Callable[[], float] = time.monotonic,
    ) -> None:
        self._clock = clock  # every deadline and the heartbeat schedule read it; tests inject a manual clock
        self._link = link
        self._planner = planner
        self._settings = settings
        self._bus = bus
        self._cond = threading.Condition()
        self._op_lock = threading.Lock()
        self._requests: dict[int, _Request] = {}
        self._done_fifo: Deque[_Request] = deque()
        self._epoch = 0  # bumped by every alarm: an operation that started earlier must stop
        self._last_alarm: Optional[RobotAlarmError] = None
        self._homed = False
        self._needs_reset = False
        self._bench = False
        self._comms_dead = False
        self._ping_thread: Optional[threading.Thread] = None
        self._ping_stop = threading.Event()
        self._started = False
        self.alarms_published: list[str] = []

    # ------------------------------------------------------------------ #
    # Read-only view
    # ------------------------------------------------------------------ #

    @property
    def homed(self) -> bool:
        return self._homed

    @property
    def needs_homing(self) -> bool:
        return not self._homed

    @property
    def bench_mode(self) -> bool:
        return self._bench

    @property
    def last_alarm(self) -> Optional[RobotAlarmError]:
        return self._last_alarm

    @property
    def settings(self) -> CommanderSettings:
        return self._settings

    # ------------------------------------------------------------------ #
    # Lifecycle
    # ------------------------------------------------------------------ #

    def start(self) -> None:
        """Listen to the link and start sending PING. The link itself is started by its owner."""
        if self._started:
            raise CommanderError("the commander is already started")
        self._started = True
        self._link.add_listener(self._on_message)
        self._ping_stop.clear()
        self._ping_thread = threading.Thread(target=self._ping_loop, name="commander-ping", daemon=True)
        self._ping_thread.start()

    def stop(self) -> None:
        self._ping_stop.set()
        thread = self._ping_thread
        if thread is not None and thread is not threading.current_thread():
            thread.join(timeout=_JOIN_TIMEOUT_S)
        self._ping_thread = None
        if self._started:
            self._link.remove_listener(self._on_message)
            self._started = False
        with self._cond:
            self._cond.notify_all()

    def __enter__(self) -> "RobotCommander":
        self.start()
        return self

    def __exit__(self, *_exc: object) -> None:
        self.stop()

    # ------------------------------------------------------------------ #
    # Guard and bench mode
    # ------------------------------------------------------------------ #

    def enable_bench_mode(self, *, confirm_no_load: bool) -> None:
        """Allow single commands on a real port while placeholders remain. Never runs a chain.

        ``confirm_no_load`` must be passed as True: the operator states that the arm carries no load,
        the links are disconnected or the arm is in a safe position, and no piece is on the board
        (SPEC 14, step 1).
        """
        if confirm_no_load is not True:
            raise BenchModeError("bench mode needs confirm_no_load=True (no load, no pieces, arm in a safe position)")
        self._bench = True
        logger.warning(
            "!!! BENCH MODE ENABLED: no-load bench commands are allowed through the real serial port. "
            "NO PICK-AND-PLACE CHAIN WILL BE RUN. Placeholders still open: %s !!!",
            ", ".join(self._planner.placeholders()) or "none",
        )

    def disable_bench_mode(self) -> None:
        if self._bench:
            logger.warning("bench mode disabled")
        self._bench = False

    def _guard_chain(self, what: str) -> None:
        """Refuse a pick-and-place chain on a real port while placeholders remain (loopback is exempt)."""
        if self._link.transport.is_simulated:
            return
        found = tuple(self._planner.placeholders())
        if not found:
            return
        if self._bench:
            logger.warning(
                "!!! BENCH MODE: %s refused. A full pick-and-place chain is NEVER run in bench mode "
                "(placeholders: %s) !!!", what, ", ".join(found),
            )
            raise BenchModeError(
                f"{what} refused: bench mode never runs a pick-and-place chain (placeholders: {', '.join(found)})"
            )
        logger.error("%s refused on a real serial port: still placeholders: %s", what, ", ".join(found))
        raise GuardRefusedError(
            f"{what} refused: the real serial port is not allowed to run a pick-and-place chain while "
            f"these are still placeholders: {', '.join(found)}",
            found,
        )

    # ------------------------------------------------------------------ #
    # Operations
    # ------------------------------------------------------------------ #

    def execute_chain(self, chain: MoveChain) -> None:
        """Run the pick-and-place chain of one engine move (FSM state ROBOT_EXEC).

        Raises:
            GuardRefusedError / BenchModeError: the placeholder guard (nothing was sent).
            NotReadyError: link down, homing required, or the arm is not at the start point.
            RobotAlarmError: an alarm or a lost link stopped the chain (the FSM was told).
            CommandFailedError / ReplyTimeoutError: the ESP32 refused or did not answer a command.
        """
        self._guard_chain("the pick-and-place chain")
        plan = build_plan(chain, self._settings)
        self._run(plan, chain_start_joints(chain), f"chain {chain.move_uci}")

    def park(self, chain: MoveChain) -> None:
        """Return the arm to the park position (FSM state VERIFY). Guarded like a chain."""
        self._guard_chain("the park leg")
        plan = build_park_plan(chain, self._settings)
        self._run(plan, park_start_joints(chain), f"park after {chain.move_uci}", accelerate=False)

    def home(self) -> None:
        """Homing (FSM state HOMING): RESET first when an alarm is latched, then HOME and its ``done``.

        Allowed through a real port even with placeholders: it moves nothing the placeholders describe.
        """
        with self._op_lock:
            epoch = self._begin()
            if self._needs_reset:
                self._finish(self._submit(Command(_RESET, {})), epoch)
                self._needs_reset = False
            self._finish(self._submit(Command(_HOME, {})), epoch)
            self._homed = True
            logger.info("homed")

    def position(self) -> DataLine:
        """The ``POS`` data line (A, B, SA, SB, NA, NB, Z)."""
        with self._op_lock:
            epoch = self._begin()
            data = self._finish(self._submit(Command(_POS, {})), epoch).data
            assert data is not None
            return data

    def emergency_stop(self) -> None:
        """ESTOP now, from any thread (the GUI button). Does not wait behind the in-flight window.

        If the command cannot be delivered the alarm is still published: the operator pressed the
        button, and the ESP32 stops by itself on its heartbeat timeout.
        """
        try:
            self._submit(Command(_ESTOP, {}), urgent=True)
        except CommanderError as exc:
            logger.error("ESTOP could not be delivered: %s", exc)
            self._raise_alarm(AlarmCode.ESTOP.value, "the ESTOP command could not be delivered")

    def bench_command(self, name: str, **params: Union[float, bool]) -> Optional[DataLine]:
        """One declared command, alone, in bench mode (or on the loopback). Returns the data line, if any.

        This is the building block of the phase 4 bring-up. It cannot be used to run a chain: each call
        is one command, with no sequencing.
        """
        if not (self._bench or self._link.transport.is_simulated):
            raise BenchModeError("bench commands need bench mode (enable_bench_mode) on a real serial port")
        try:
            command = Command(name, params)
            command.encode()  # validate against the declaration before anything is sent
        except EncodeError as exc:
            raise CommanderError(f"invalid bench command: {exc}") from exc
        if not self._link.transport.is_simulated:
            logger.warning("BENCH MODE command: %s", command.encode())
        with self._op_lock:
            epoch = self._begin(require_homed=False)
            request = self._finish(self._submit(command), epoch)
            return request.data

    # ------------------------------------------------------------------ #
    # Running a plan
    # ------------------------------------------------------------------ #

    def _begin(self, *, require_homed: bool = False) -> int:
        if self._link.state is not LinkState.CONNECTED:
            raise NotReadyError(f"the serial link is {self._link.state.value}")
        if require_homed and not self._homed:
            reason = f" after alarm {self._last_alarm.code}" if self._last_alarm else ""
            raise NotReadyError(f"homing is required{reason}")
        with self._cond:
            return self._epoch

    def _run(self, plan: tuple[PlanItem, ...], start: tuple[float, float], label: str, *, accelerate: bool = True) -> None:
        with self._op_lock:
            epoch = self._begin(require_homed=True)
            logger.info("%s: %d command(s)", label, len(plan))
            try:
                self._check_start(start, epoch)
                if accelerate and self._settings.acceleration is not None:
                    self._finish(self._submit(Command(_ACCEL, {"S": self._settings.acceleration})), epoch)
                self._execute(plan, epoch)
            except ReplyTimeoutError:
                logger.error("%s: no reply in time, emergency stop", label)
                self._best_effort_estop()
                raise
            except CommanderError as exc:
                logger.error("%s stopped: %s", label, exc)
                raise

    def _check_start(self, start: tuple[float, float], epoch: int) -> None:
        """Drain the queue, then compare POS with the chain's first point and Z with TOP."""
        self._finish(self._submit(Command(_WAIT, {})), epoch)
        data = self._finish(self._submit(Command(_POS, {})), epoch).data
        assert data is not None
        tolerance = self._settings.start_tolerance_deg
        for name, wanted in zip(("A", "B"), start):
            for field_name in (name, "S" + name):  # commanded and AS5600 angle
                actual = float(data.fields[field_name])
                if abs(actual - wanted) > tolerance:
                    raise NotAtStartError(
                        f"the arm is not at the start point: {field_name}={actual:.3f} but the chain starts at "
                        f"{wanted:.3f} (tolerance {tolerance:g} deg)"
                    )
        if data.fields["Z"] != ZState.TOP.value:
            raise NotAtStartError(f"Z is {data.fields['Z']}, not at the top")

    def _execute(self, plan: tuple[PlanItem, ...], epoch: int) -> None:
        for item in plan:
            if item.kind is PlanKind.MOVE:
                self._send_move(item.command, epoch)
                continue
            request = self._submit(item.command)
            self._wait_ok(request, epoch)
            if item.kind in (PlanKind.DRAIN, PlanKind.Z):
                self._wait_done(request, epoch)
            elif item.kind is PlanKind.GRIP:
                self._settle(self._settings.grip_settle_s, epoch)
            self._forget(request)

    def _send_move(self, command: Command, epoch: int) -> None:
        """Send one MOVE and wait for its ok. error:3 means "not queued": pause and send the same MOVE again.

        MOVEs are sent one at a time on purpose. The SPEC's in-flight limit (5.3) cannot bound the firmware
        queue, because ``ok`` means "queued" and arrives long before the move runs. With several MOVEs in
        flight a refused one could be followed by an accepted one, and the trajectory would run out of order.
        """
        give_up_at = self._clock() + self._settings.queue_full_timeout_s
        while True:
            request = self._submit(command)
            try:
                self._wait_ok(request, epoch)
            except CommandFailedError as exc:
                if exc.code is not ErrorCode.QUEUE_FULL:
                    raise
                if self._clock() >= give_up_at:
                    raise
                self._settle(self._settings.queue_retry_s, epoch)
                continue
            self._forget(request)
            return

    # ------------------------------------------------------------------ #
    # Sending and waiting
    # ------------------------------------------------------------------ #

    def _submit(self, command: Command, *, urgent: bool = False) -> _Request:
        request = _Request(command, wants_done=COMMANDS[command.name].completion is Completion.DONE)
        with self._cond:
            self._requests[id(command)] = request
        try:
            self._link.send(command, self._settings.send_timeout_s, urgent=urgent)
        except LinkError as exc:
            self._forget(request)
            if isinstance(exc, LinkDownError):
                raise NotReadyError(f"{command.name} was not sent: {exc}") from exc
            raise ReplyTimeoutError(f"{command.name} was not sent: {exc}") from exc
        return request

    def _forget(self, request: _Request) -> None:
        with self._cond:
            self._requests.pop(id(request.command), None)

    def _alarm_since(self, epoch: int) -> Optional[RobotAlarmError]:
        """A fresh copy of the latest alarm when one was raised after ``epoch`` (call with the lock held)."""
        if self._epoch == epoch:
            return None
        assert self._last_alarm is not None
        return RobotAlarmError(self._last_alarm.code, self._last_alarm.detail)

    def _check_epoch(self, epoch: int) -> None:
        with self._cond:
            if (alarm := self._alarm_since(epoch)) is not None:
                raise alarm

    def _wait_for(self, predicate: Callable[[], bool], timeout_s: float, epoch: int, what: str) -> None:
        deadline = self._clock() + timeout_s
        with self._cond:
            while True:
                if (alarm := self._alarm_since(epoch)) is not None:
                    raise alarm
                if predicate():
                    return
                remaining = deadline - self._clock()
                if remaining <= 0:
                    raise ReplyTimeoutError(f"no {what} within {timeout_s:g} s")
                self._cond.wait(min(remaining, _CLOCK_POLL_S))

    def _wait_ok(self, request: _Request, epoch: int) -> _Request:
        self._wait_for(lambda: request.answered, self._settings.reply_timeout_s, epoch, f"reply to {request.command.name}")
        if request.error is not None:
            self._forget(request)
            raise CommandFailedError(request.command, request.error)
        return request

    def _wait_done(self, request: _Request, epoch: int) -> None:
        self._wait_for(lambda: request.finished, self._settings.done_timeout_s, epoch, f"done after {request.command.name}")

    def _finish(self, request: _Request, epoch: int) -> _Request:
        """Wait for the ok and, when the command has one, for the done."""
        self._wait_ok(request, epoch)
        if request.wants_done:
            self._wait_done(request, epoch)
        self._forget(request)
        return request

    def _settle(self, seconds: float, epoch: int) -> None:
        deadline = self._clock() + seconds
        with self._cond:
            while (alarm := self._alarm_since(epoch)) is None:
                remaining = deadline - self._clock()
                if remaining <= 0:
                    return
                self._cond.wait(min(remaining, _CLOCK_POLL_S))
            raise alarm

    def _best_effort_estop(self) -> None:
        try:
            self._submit(Command(_ESTOP, {}), urgent=True)
        except CommanderError:
            logger.error("the emergency stop after a timeout could not be sent either")

    # ------------------------------------------------------------------ #
    # Heartbeat
    # ------------------------------------------------------------------ #

    def _ping_loop(self) -> None:
        next_at = self._clock()
        while not self._ping_stop.is_set():
            now = self._clock()
            if now >= next_at:
                next_at = now + self._settings.ping_interval_s  # from when this PING starts: the rate does not drift
                if self._link.state is LinkState.CONNECTED:
                    self._ping_once()
            self._ping_stop.wait(min(self._settings.ping_interval_s, _CLOCK_POLL_S))

    def _ping_once(self) -> None:
        try:
            request = self._submit(Command(_PING, {}), urgent=True)
        except CommanderError as exc:
            logger.debug("PING not sent: %s", exc)
            return
        deadline = self._clock() + self._settings.reply_timeout_s
        with self._cond:
            while not request.answered:
                remaining = deadline - self._clock()
                if remaining <= 0:
                    break
                self._cond.wait(min(remaining, _CLOCK_POLL_S))
            alive = request.answered  # even error:N proves the ESP32 is listening
        self._forget(request)
        if alive:
            self._comms_dead = False
        elif not self._comms_dead and self._link.state is LinkState.CONNECTED:
            self._comms_dead = True  # raise it once per episode
            self._raise_alarm(AlarmCode.HB_TIMEOUT.value, "PING was not answered")

    # ------------------------------------------------------------------ #
    # What the link tells us
    # ------------------------------------------------------------------ #

    def _on_message(self, message: LinkMessage) -> None:
        if isinstance(message, Reply):
            self._on_reply(message)
        elif isinstance(message, Unsolicited):
            if isinstance(message.response, Done):
                self._on_done()
            elif isinstance(message.response, Alarm):
                self._raise_alarm(message.response.code.value, "reported by the ESP32")
        elif isinstance(message, BadLine):
            if message.alarm_code is not None:
                logger.critical("undeclared alarm %r from the ESP32: emergency stop", message.alarm_code)
                self._best_effort_estop()
                self._abort_operations(RobotAlarmError(AlarmCode.ESTOP.value, f"undeclared alarm {message.alarm_code!r}"))
        elif isinstance(message, Disconnected):
            self._on_disconnected(message)
        elif isinstance(message, Connected) and message.reconnected:
            logger.warning("serial link is back: homing is required before the robot moves again")
            with self._cond:
                self._homed = False
                self._needs_reset = True

    def _on_reply(self, reply: Reply) -> None:
        with self._cond:
            request = self._requests.get(id(reply.command))
            if request is not None:
                response = reply.response
                if isinstance(response, DataLine):
                    request.data = response
                elif isinstance(response, ErrorResponse):
                    request.error = response.code
                elif isinstance(response, (Ok, Alarm)):  # an ESTOP is acknowledged by its alarm
                    request.acknowledged = True
                    if request.wants_done:
                        self._done_fifo.append(request)
                    else:
                        request.finished = True
                self._cond.notify_all()
        if isinstance(reply.response, Alarm):
            self._raise_alarm(reply.response.code.value, "reported by the ESP32")

    def _on_done(self) -> None:
        with self._cond:
            if not self._done_fifo:
                logger.warning("done arrived but no command is waiting for it")
                return
            self._done_fifo.popleft().finished = True
            self._cond.notify_all()

    def _on_disconnected(self, message: Disconnected) -> None:
        with self._cond:
            self._requests.clear()  # their outcome is unknown; the alarm below releases whoever waits
            self._done_fifo.clear()
        self._raise_alarm(AlarmCode.HB_TIMEOUT.value, f"serial link lost: {message.reason}")

    # ------------------------------------------------------------------ #
    # Alarms
    # ------------------------------------------------------------------ #

    def _raise_alarm(self, code: str, detail: str) -> None:
        """Publish the alarm to the FSM FIRST, then release whoever waits for the chain."""
        error = RobotAlarmError(code, detail)
        logger.error("ALARM %s (%s)", code, detail or "no detail")
        with self._cond:  # the state first, so a bus listener that looks at the Commander sees the alarm
            self._last_alarm = error
            self._homed = False
            self._needs_reset = True
            self._done_fifo.clear()
        self.alarms_published.append(code)
        if self._bus is not None:
            publish_alarm(self._bus, code)
        self._abort_operations(error)  # waiters are released only now: the FSM has already been told

    def _abort_operations(self, error: RobotAlarmError) -> None:
        with self._cond:
            self._last_alarm = error
            self._epoch += 1
            self._homed = False
            self._needs_reset = True
            self._cond.notify_all()
