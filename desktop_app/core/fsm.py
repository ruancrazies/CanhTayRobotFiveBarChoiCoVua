"""Operating-phase FSM (docs/SPEC.md sections 7.2 and 7.3).

The FSM only decides WHICH request is accepted and WHERE it leads. It does no work itself: the
caller (the console runner today, a controller later) performs the work of each state and then
reports the outcome with a trigger. It never changes GameState; it only reads it, to refuse a
trigger that does not fit the game (for example COMMIT_DONE before the move was committed).

Transitions are exactly the SPEC 7.2 table (including "Rules clarified in phase 2") plus the 7.3
rules (``PAUSED`` and ``IDLE``). Anything else is refused, and ``TRANSITIONS`` lists them all.

Failures (7.2):
  * ENGINE_THINK: ENGINE_FAILED (engine failed or timed out) -> ERROR_RECOVER.
  * PLAN: PLAN_FAILED (invalid point, or no free capture or reserve slot) -> ERROR_RECOVER.
  * ROBOT_EXEC / VERIFY: EXEC_FAILED / VERIFY_FAILED -> ERROR_RECOVER.
  * ALARM (ESTOP, HB_TIMEOUT, SENSOR_FAULT, camera loss, ...) from ANY running state ->
    ERROR_RECOVER. "Running" is every state except IDLE (program off), GAME_OVER (nothing in
    flight) and ERROR_RECOVER itself. An alarm that arrives while already in ERROR_RECOVER keeps the
    state but can make homing mandatory.
  * ERROR_RECOVER leaves only through RESYNC, to WAIT_HUMAN, PLAN, HOMING or GAME_OVER. pending_move
    is kept. The exit is chosen from the game, never from the camera (see ``_on_resync``).

Locks (7.2): the arm moves only in HOMING, ROBOT_EXEC and VERIFY (``arm_may_move``); ENGINE_THINK
and PLAN only compute (``compute_only``); move detection is locked in every state except WAIT_HUMAN
(``move_detector_enabled``).

Pause (7.3):
  * WAIT_HUMAN: immediate.
  * ENGINE_THINK / PLAN: deferred until the proposed move (or the chain) is ready, then PAUSED
    before the robot moves; RESUME continues with the step that would have followed.
  * ROBOT_EXEC / VERIFY: deferred until VERIFY has finished and committed the move.
YOLO off (7.3): immediate IDLE where nothing is in flight; deferred in CALIBRATE, HOMING,
COMMIT_HUMAN, ROBOT_EXEC and VERIFY (the chain finishes first). pending_move is kept. A fault
(EXEC_FAILED / VERIFY_FAILED) cancels a deferred pause or shutdown: the fault is handled first.
"""

from __future__ import annotations

import logging
import threading
from dataclasses import dataclass, field
from enum import Enum
from typing import Any, Mapping, Optional

import chess

from desktop_app.core.event_bus import EventBus, Notification, Request
from desktop_app.core.game_state import GameState

logger = logging.getLogger(__name__)


class State(str, Enum):
    """FSM states (SPEC 7.2). Names are English; the GUI maps them to Vietnamese."""

    IDLE = "IDLE"
    INIT = "INIT"
    CALIBRATE = "CALIBRATE"
    HOMING = "HOMING"
    SETUP_CHECK = "SETUP_CHECK"
    WAIT_HUMAN = "WAIT_HUMAN"
    COMMIT_HUMAN = "COMMIT_HUMAN"
    ENGINE_THINK = "ENGINE_THINK"
    PLAN = "PLAN"
    ROBOT_EXEC = "ROBOT_EXEC"
    VERIFY = "VERIFY"
    PAUSED = "PAUSED"
    ERROR_RECOVER = "ERROR_RECOVER"
    GAME_OVER = "GAME_OVER"


class Trigger(str, Enum):
    """Inputs of the FSM: user requests and the outcome of the work done in a state."""

    # user requests
    YOLO_ON = "yolo_on"
    YOLO_OFF = "yolo_off"
    START = "start"  # payload: human_color (chess.Color), pending="redo" (optional)
    HUMAN_MOVE = "human_move"  # stable detected move or manual confirm; payload: move
    PAUSE = "pause"
    RESUME = "resume"
    RESYNC = "resync"
    NEW_GAME = "new_game"
    RESTART = "restart"
    # outcomes reported by the work of a state
    INIT_DONE = "init_done"
    CALIBRATE_DONE = "calibrate_done"
    HOMING_DONE = "homing_done"
    COMMIT_DONE = "commit_done"
    ENGINE_MOVE_READY = "engine_move_ready"
    PLAN_READY = "plan_ready"
    EXEC_DONE = "exec_done"
    EXEC_FAILED = "exec_failed"  # payload: alarm (optional str)
    VERIFY_OK = "verify_ok"
    VERIFY_FAILED = "verify_failed"  # payload: alarm (optional str)
    ENGINE_FAILED = "engine_failed"  # engine error or timeout; payload: reason (optional str)
    PLAN_FAILED = "plan_failed"  # invalid point or no free slot; payload: reason (optional str)
    ALARM = "alarm"  # asynchronous alarm or camera loss; payload: alarm (required, one of ALARM_CODES)


# After these alarms homing is mandatory (SPEC 5.3 and 7.2).
HOMING_MANDATORY_ALARMS = frozenset({"ESTOP", "STEP_LOSS", "HB_TIMEOUT", "SENSOR_FAULT"})
# Alarm codes the FSM accepts with ALARM: the firmware alarms of SPEC 5.2 plus camera loss (msg.camera_lost).
ALARM_CODES = frozenset({"ESTOP", "HB_TIMEOUT", "SENSOR_FAULT", "STEP_LOSS", "LIMIT_Z", "Z_TIMEOUT", "CAMERA_LOST"})

S = State
T = Trigger

# Running = YOLO is on and the game is not over; ERROR_RECOVER is the destination, not a source.
RUNNING_STATES = frozenset(set(State) - {State.IDLE, State.GAME_OVER, State.ERROR_RECOVER})
# Errors before a game is being played: the arm position is unknown, so recovery re-homes.
PRE_GAME_STATES = frozenset({State.INIT, State.CALIBRATE, State.HOMING, State.SETUP_CHECK})
# SPEC 7.2: the arm moves only here (VERIFY = return to the park position).
ARM_MOVING_STATES = frozenset({State.HOMING, State.ROBOT_EXEC, State.VERIFY})
# SPEC 7.2: these states only compute.
COMPUTE_ONLY_STATES = frozenset({State.ENGINE_THINK, State.PLAN})
# SPEC 7.2 / rule 4: move detection is unlocked here and nowhere else.
DETECTING_STATES = frozenset({State.WAIT_HUMAN})

# Every state change the FSM can make: (state, trigger) -> possible next states (SPEC 7.2 and 7.3).
TRANSITIONS: Mapping[tuple[State, Trigger], frozenset[State]] = {
    (S.IDLE, T.YOLO_ON): frozenset({S.INIT}),
    (S.INIT, T.INIT_DONE): frozenset({S.CALIBRATE}),
    (S.CALIBRATE, T.CALIBRATE_DONE): frozenset({S.SETUP_CHECK, S.IDLE}),
    (S.HOMING, T.HOMING_DONE): frozenset({S.SETUP_CHECK, S.IDLE}),
    (S.SETUP_CHECK, T.START): frozenset({S.WAIT_HUMAN, S.ENGINE_THINK, S.PLAN}),
    (S.WAIT_HUMAN, T.HUMAN_MOVE): frozenset({S.COMMIT_HUMAN}),
    (S.WAIT_HUMAN, T.PAUSE): frozenset({S.PAUSED}),
    (S.COMMIT_HUMAN, T.COMMIT_DONE): frozenset({S.ENGINE_THINK, S.GAME_OVER, S.IDLE}),
    (S.ENGINE_THINK, T.ENGINE_MOVE_READY): frozenset({S.PLAN, S.PAUSED}),
    (S.ENGINE_THINK, T.ENGINE_FAILED): frozenset({S.ERROR_RECOVER}),
    (S.PLAN, T.PLAN_READY): frozenset({S.ROBOT_EXEC, S.PAUSED}),
    (S.PLAN, T.PLAN_FAILED): frozenset({S.ERROR_RECOVER}),
    (S.ROBOT_EXEC, T.EXEC_DONE): frozenset({S.VERIFY, S.IDLE}),
    (S.ROBOT_EXEC, T.EXEC_FAILED): frozenset({S.ERROR_RECOVER}),
    (S.VERIFY, T.VERIFY_OK): frozenset({S.WAIT_HUMAN, S.GAME_OVER, S.PAUSED, S.IDLE}),
    (S.VERIFY, T.VERIFY_FAILED): frozenset({S.ERROR_RECOVER}),
    (S.PAUSED, T.RESUME): frozenset({S.WAIT_HUMAN, S.PLAN, S.ROBOT_EXEC}),
    (S.ERROR_RECOVER, T.RESYNC): frozenset({S.HOMING, S.PLAN, S.WAIT_HUMAN, S.GAME_OVER}),
    (S.ERROR_RECOVER, T.ALARM): frozenset({S.ERROR_RECOVER}),  # stays; may make homing mandatory
    (S.GAME_OVER, T.NEW_GAME): frozenset({S.SETUP_CHECK}),
    (S.GAME_OVER, T.RESTART): frozenset({S.INIT}),
    # An alarm from ANY running state (SPEC 7.2: ERROR_RECOVER is "entered from any running state").
    **{(state, T.ALARM): frozenset({S.ERROR_RECOVER}) for state in RUNNING_STATES},
    # YOLO off, immediate: nothing is in flight in these states.
    **{(state, T.YOLO_OFF): frozenset({S.IDLE}) for state in (
        S.INIT, S.SETUP_CHECK, S.WAIT_HUMAN, S.ENGINE_THINK, S.PLAN, S.PAUSED, S.ERROR_RECOVER, S.GAME_OVER,
    )},
}

# Accepted without changing state (the request is remembered until it can take effect).
DEFERRABLE_PAUSE_STATES = frozenset({S.ENGINE_THINK, S.PLAN, S.ROBOT_EXEC, S.VERIFY})
DEFERRABLE_SHUTDOWN_STATES = frozenset({S.CALIBRATE, S.HOMING, S.COMMIT_HUMAN, S.ROBOT_EXEC, S.VERIFY})
_PAUSE_POINTS = frozenset({(S.ENGINE_THINK, S.PLAN), (S.PLAN, S.ROBOT_EXEC), (S.VERIFY, S.WAIT_HUMAN)})


# --------------------------------------------------------------------------- #
# Events
# --------------------------------------------------------------------------- #


@dataclass(frozen=True)
class StateChanged(Notification):
    previous: State
    current: State
    trigger: Trigger
    reason: str


@dataclass(frozen=True)
class RequestRejected(Notification):
    trigger: Trigger
    state: State
    reason: str


@dataclass(frozen=True)
class FsmRequest(Request):
    """GUI to core: ask the FSM to handle ``trigger``. The FSM may refuse (RequestRejected)."""

    trigger: Trigger
    payload: Mapping[str, Any] = field(default_factory=dict)


@dataclass(frozen=True)
class TransitionResult:
    accepted: bool
    previous: State
    state: State  # the state after handling
    reason: str
    deferred: bool = False  # accepted, but it takes effect later


@dataclass(frozen=True)
class _Decision:
    accepted: bool
    reason: str
    target: Optional[State] = None  # None: no state change
    deferred: bool = False
    pause_pending: Optional[bool] = None
    shutdown_pending: Optional[bool] = None
    resume_to: Optional[State] = None
    homing_required: Optional[bool] = None
    human_color: Optional[chess.Color] = None


def _refuse(reason: str) -> _Decision:
    return _Decision(False, reason)


# --------------------------------------------------------------------------- #
# FSM
# --------------------------------------------------------------------------- #


class Fsm:
    """Decides which request is accepted and where it leads. Starts in IDLE (YOLO off)."""

    def __init__(self, game_state: Optional[GameState] = None, bus: Optional[EventBus] = None) -> None:
        self._lock = threading.RLock()
        self._state = State.IDLE
        self._game_state = game_state
        self._bus = bus
        self._human_color: Optional[chess.Color] = None
        self._pause_pending = False
        self._shutdown_pending = False
        self._resume_state: Optional[State] = None
        self._homing_required = False
        self._subscription = bus.subscribe(FsmRequest, self._on_request) if bus is not None else None

    # ------------------------------------------------------------------ #
    # Read-only view
    # ------------------------------------------------------------------ #

    @property
    def state(self) -> State:
        return self._state

    @property
    def human_color(self) -> Optional[chess.Color]:
        return self._human_color

    @property
    def pause_pending(self) -> bool:
        return self._pause_pending

    @property
    def shutdown_pending(self) -> bool:
        return self._shutdown_pending

    @property
    def resume_state(self) -> Optional[State]:
        """Where RESUME will go while PAUSED."""
        return self._resume_state

    @property
    def homing_required(self) -> bool:
        return self._homing_required

    @property
    def move_detector_enabled(self) -> bool:
        """SPEC rule 4: the Move Detector runs only in WAIT_HUMAN."""
        return self._state in DETECTING_STATES

    @property
    def arm_may_move(self) -> bool:
        """SPEC 7.2: the arm moves only in HOMING, ROBOT_EXEC and VERIFY (return to park)."""
        return self._state in ARM_MOVING_STATES

    @property
    def compute_only(self) -> bool:
        """SPEC 7.2: ENGINE_THINK and PLAN only compute; nothing physical happens there."""
        return self._state in COMPUTE_ONLY_STATES

    def attach_game_state(self, game_state: GameState) -> None:
        """Use a different GameState (new or reloaded game). Refused while a game is being played."""
        with self._lock:
            if self._state not in (State.IDLE, State.INIT, State.SETUP_CHECK, State.GAME_OVER):
                raise RuntimeError(f"cannot replace the game in state {self._state.value}")
            self._game_state = game_state

    # ------------------------------------------------------------------ #
    # Requests
    # ------------------------------------------------------------------ #

    def can_accept(self, trigger: Trigger, **payload: Any) -> bool:
        """True when ``handle`` would accept the trigger now (nothing is changed)."""
        with self._lock:
            return self._decide(trigger, payload).accepted

    def handle(self, trigger: Trigger, **payload: Any) -> TransitionResult:
        """Handle one trigger. A refused trigger changes nothing."""
        with self._lock:
            previous = self._state
            decision = self._decide(trigger, payload)
            if not decision.accepted:
                result = TransitionResult(False, previous, previous, decision.reason)
                events: list[Notification] = [RequestRejected(trigger, previous, decision.reason)]
                logger.info("refused %s in %s: %s", trigger.value, previous.value, decision.reason)
            else:
                self._apply(decision)
                result = TransitionResult(True, previous, self._state, decision.reason, decision.deferred)
                events = (
                    [StateChanged(previous, self._state, trigger, decision.reason)] if self._state is not previous else []
                )
                logger.info("%s: %s -> %s (%s)", trigger.value, previous.value, self._state.value, decision.reason)
        if self._bus is not None:
            for event in events:
                self._bus.publish(event)
        return result

    def _on_request(self, request: FsmRequest) -> None:
        self.handle(request.trigger, **dict(request.payload))

    # ------------------------------------------------------------------ #
    # Decision (pure) and application
    # ------------------------------------------------------------------ #

    def _decide(self, trigger: Trigger, payload: Mapping[str, Any]) -> _Decision:
        state = self._state

        if trigger is T.YOLO_ON:
            return _Decision(True, "YOLO on", State.INIT) if state is S.IDLE else _refuse("YOLO is already on")

        if trigger is T.YOLO_OFF:
            if state is S.IDLE:
                return _refuse("YOLO is already off")
            if (state, T.YOLO_OFF) in TRANSITIONS:
                return _Decision(True, "YOLO off", S.IDLE)
            return _Decision(True, "YOLO off after the running work finishes", deferred=True, shutdown_pending=True)

        if trigger is T.PAUSE:
            if state is S.WAIT_HUMAN:
                return _Decision(True, "paused now", S.PAUSED, resume_to=S.WAIT_HUMAN)
            if state in DEFERRABLE_PAUSE_STATES:
                return _Decision(True, "pause after the current step", deferred=True, pause_pending=True)
            return _refuse(f"cannot pause in {state.value}")

        if trigger is T.RESUME:
            if state is S.PAUSED and self._resume_state is not None:
                return _Decision(True, "resumed", self._resume_state)
            if self._pause_pending:
                return _Decision(True, "pending pause cancelled", deferred=True, pause_pending=False)
            return _refuse(f"nothing to resume in {state.value}")

        handler = {
            T.INIT_DONE: self._on_init_done,
            T.CALIBRATE_DONE: self._on_calibrate_done,
            T.HOMING_DONE: self._on_homing_done,
            T.START: self._on_start,
            T.HUMAN_MOVE: self._on_human_move,
            T.COMMIT_DONE: self._on_commit_done,
            T.ENGINE_MOVE_READY: self._on_engine_move_ready,
            T.PLAN_READY: self._on_plan_ready,
            T.EXEC_DONE: self._on_exec_done,
            T.EXEC_FAILED: self._on_fault,
            T.VERIFY_OK: self._on_verify_ok,
            T.VERIFY_FAILED: self._on_fault,
            T.ENGINE_FAILED: self._on_failure,
            T.PLAN_FAILED: self._on_failure,
            T.ALARM: self._on_alarm,
            T.RESYNC: self._on_resync,
            T.NEW_GAME: self._on_new_game,
            T.RESTART: self._on_restart,
        }[trigger]
        return handler(trigger, payload)

    def _wrong_state(self, trigger: Trigger) -> _Decision:
        return _refuse(f"{trigger.value} is not accepted in {self._state.value}")

    def _route(self, natural: State, reason: str) -> _Decision:
        """Apply a deferred shutdown or pause to the natural next state."""
        state = self._state
        if self._shutdown_pending and state in DEFERRABLE_SHUTDOWN_STATES:
            return _Decision(True, "shutdown after the running work finished", S.IDLE)
        if self._pause_pending and (state, natural) in _PAUSE_POINTS:
            return _Decision(True, "paused before the next step", S.PAUSED, resume_to=natural)
        return _Decision(True, reason, natural)

    def _game(self) -> Optional[GameState]:
        return self._game_state

    def _on_init_done(self, trigger: Trigger, payload: Mapping[str, Any]) -> _Decision:
        if self._state is not S.INIT:
            return self._wrong_state(trigger)
        return _Decision(True, "initialised", S.CALIBRATE)

    def _on_calibrate_done(self, trigger: Trigger, payload: Mapping[str, Any]) -> _Decision:
        if self._state is not S.CALIBRATE:
            return self._wrong_state(trigger)
        return self._route(S.SETUP_CHECK, "calibration and homing done")

    def _on_homing_done(self, trigger: Trigger, payload: Mapping[str, Any]) -> _Decision:
        if self._state is not S.HOMING:
            return self._wrong_state(trigger)
        return self._route(S.SETUP_CHECK, "homing done")

    def _on_start(self, trigger: Trigger, payload: Mapping[str, Any]) -> _Decision:
        if self._state is not S.SETUP_CHECK:
            return self._wrong_state(trigger)
        game = self._game()
        if game is None:
            return _refuse("no game is attached")
        color = payload.get("human_color")
        if not isinstance(color, bool):
            return _refuse("START needs human_color (chess.WHITE or chess.BLACK)")
        pending = payload.get("pending")
        if pending not in (None, "redo"):
            return _refuse(f"unknown pending decision {pending!r}")
        if game.is_game_over:
            return _refuse("the game is already over")
        if game.pending_move is not None and pending != "redo":
            return _refuse("an unresolved pending move exists: redo it (pending='redo') or cancel it first")
        if pending == "redo":
            if game.pending_move is None:
                return _refuse("there is no pending move to redo")
            if game.turn == color:
                return _refuse("a pending move belongs to the engine, but it is the human's turn")
            return _Decision(True, "redo the pending move", S.PLAN, human_color=color)
        target = S.WAIT_HUMAN if game.turn == color else S.ENGINE_THINK
        return _Decision(True, "game started", target, human_color=color)

    def _on_human_move(self, trigger: Trigger, payload: Mapping[str, Any]) -> _Decision:
        if self._state is not S.WAIT_HUMAN:
            return self._wrong_state(trigger)
        game = self._game()
        if game is None or self._human_color is None:
            return _refuse("no game is attached or no side was chosen")
        move = payload.get("move")
        if not isinstance(move, chess.Move) or move not in game.legal_moves:
            return _refuse(f"{move!r} is not a legal move in the current position")
        if game.turn != self._human_color:
            return _refuse("it is not the human's turn")
        return _Decision(True, "human move accepted", S.COMMIT_HUMAN)

    def _on_commit_done(self, trigger: Trigger, payload: Mapping[str, Any]) -> _Decision:
        if self._state is not S.COMMIT_HUMAN:
            return self._wrong_state(trigger)
        game = self._game()
        if game is None or self._human_color is None:
            return _refuse("no game is attached or no side was chosen")
        if game.turn == self._human_color:
            return _refuse("the human move has not been committed to GameState yet")
        if game.is_game_over:
            return self._route(S.GAME_OVER, "the human move ended the game")
        return self._route(S.ENGINE_THINK, "human move committed")

    def _on_engine_move_ready(self, trigger: Trigger, payload: Mapping[str, Any]) -> _Decision:
        if self._state is not S.ENGINE_THINK:
            return self._wrong_state(trigger)
        game = self._game()
        if game is None or game.pending_move is None:
            return _refuse("the engine move has not been stored as pending_move yet")
        return self._route(S.PLAN, "engine move stored as pending_move")

    def _on_plan_ready(self, trigger: Trigger, payload: Mapping[str, Any]) -> _Decision:
        if self._state is not S.PLAN:
            return self._wrong_state(trigger)
        game = self._game()
        if game is None or game.pending_move is None:
            return _refuse("there is no pending move to execute")
        return self._route(S.ROBOT_EXEC, "command chain built")

    def _on_exec_done(self, trigger: Trigger, payload: Mapping[str, Any]) -> _Decision:
        if self._state is not S.ROBOT_EXEC:
            return self._wrong_state(trigger)
        return self._route(S.VERIFY, "robot finished the chain")

    def _on_fault(self, trigger: Trigger, payload: Mapping[str, Any]) -> _Decision:
        expected = S.ROBOT_EXEC if trigger is T.EXEC_FAILED else S.VERIFY
        if self._state is not expected:
            return self._wrong_state(trigger)
        alarm = payload.get("alarm")
        if alarm is not None and not isinstance(alarm, str):
            return _refuse("alarm must be a string code")
        return _Decision(
            True,
            f"fault: {alarm or trigger.value}",
            S.ERROR_RECOVER,
            homing_required=alarm in HOMING_MANDATORY_ALARMS,
        )

    def _on_failure(self, trigger: Trigger, payload: Mapping[str, Any]) -> _Decision:
        """ENGINE_FAILED in ENGINE_THINK, PLAN_FAILED in PLAN: nothing moved, nothing was committed."""
        expected = S.ENGINE_THINK if trigger is T.ENGINE_FAILED else S.PLAN
        if self._state is not expected:
            return self._wrong_state(trigger)
        reason = payload.get("reason")
        if reason is not None and not isinstance(reason, str):
            return _refuse("reason must be a string")
        return _Decision(True, f"{trigger.value}: {reason or 'no detail'}", S.ERROR_RECOVER, homing_required=False)

    def _on_alarm(self, trigger: Trigger, payload: Mapping[str, Any]) -> _Decision:
        alarm = payload.get("alarm")
        if not isinstance(alarm, str) or alarm not in ALARM_CODES:
            return _refuse(f"ALARM needs alarm to be one of {sorted(ALARM_CODES)}, got {alarm!r}")
        mandatory = alarm in HOMING_MANDATORY_ALARMS
        if self._state is S.ERROR_RECOVER:
            # Already in error: stay, but never forget that homing became mandatory.
            return _Decision(True, f"alarm {alarm} while recovering", S.ERROR_RECOVER,
                             homing_required=self._homing_required or mandatory)
        if self._state not in RUNNING_STATES:
            return _refuse(f"{self._state.value} is not a running state")
        return _Decision(True, f"alarm: {alarm}", S.ERROR_RECOVER,
                         homing_required=mandatory or self._state in PRE_GAME_STATES)

    def _on_verify_ok(self, trigger: Trigger, payload: Mapping[str, Any]) -> _Decision:
        if self._state is not S.VERIFY:
            return self._wrong_state(trigger)
        game = self._game()
        if game is None or self._human_color is None:
            return _refuse("no game is attached or no side was chosen")
        if game.pending_move is not None or (game.turn != self._human_color and not game.is_game_over):
            return _refuse("the engine move has not been committed to GameState yet")
        if game.is_game_over:
            return self._route(S.GAME_OVER, "the engine move ended the game")
        return self._route(S.WAIT_HUMAN, "position verified and engine move committed")

    def _on_resync(self, trigger: Trigger, payload: Mapping[str, Any]) -> _Decision:
        """Leave ERROR_RECOVER: to HOMING, GAME_OVER, PLAN or WAIT_HUMAN (SPEC 7.2).

        The exit comes from GameState (read-only), never from the camera. In this order:
          1. homing is mandatory (alarm, or the error hit before a game was running) -> HOMING;
          2. the game is over and nothing is pending -> GAME_OVER;
          3. an engine move is pending -> PLAN (pending_move is kept and redone);
          4. it is the human's turn -> WAIT_HUMAN;
          5. otherwise it is the engine's turn with no stored move (the engine failed, or the human
             move was just committed): the table has no exit to ENGINE_THINK, so go through HOMING
             and SETUP_CHECK, whose Start asks the engine again.
        """
        if self._state is not S.ERROR_RECOVER:
            return self._wrong_state(trigger)
        if self._homing_required:
            return _Decision(True, "homing is mandatory", S.HOMING)
        game = self._game()
        if game is None or self._human_color is None:
            return _refuse("no game is attached or no side was chosen")
        if game.is_game_over and game.pending_move is None:
            return _Decision(True, "the game is over", S.GAME_OVER)
        if game.pending_move is not None:
            return _Decision(True, "redo the kept pending move", S.PLAN)
        if game.turn == self._human_color:
            return _Decision(True, "back to the human's turn", S.WAIT_HUMAN)
        return _Decision(True, "no pending move: re-enter through HOMING and SETUP_CHECK", S.HOMING)

    def _on_new_game(self, trigger: Trigger, payload: Mapping[str, Any]) -> _Decision:
        if self._state is not S.GAME_OVER:
            return self._wrong_state(trigger)
        return _Decision(True, "new game", S.SETUP_CHECK)

    def _on_restart(self, trigger: Trigger, payload: Mapping[str, Any]) -> _Decision:
        if self._state is not S.GAME_OVER:
            return self._wrong_state(trigger)
        return _Decision(True, "restart from INIT", S.INIT)

    def _apply(self, decision: _Decision) -> None:
        if decision.human_color is not None:
            self._human_color = decision.human_color
        if decision.homing_required is not None:
            self._homing_required = decision.homing_required
        if decision.pause_pending is not None:
            self._pause_pending = decision.pause_pending
        if decision.shutdown_pending is not None:
            self._shutdown_pending = decision.shutdown_pending
        if decision.target is None:
            return
        target = decision.target
        if decision.resume_to is not None:
            self._resume_state = decision.resume_to
        elif target is not State.PAUSED:
            self._resume_state = None
        self._state = target
        # A deferred request only lives while the state it waits in is current.
        if target not in DEFERRABLE_PAUSE_STATES:
            self._pause_pending = False
        if target not in DEFERRABLE_SHUTDOWN_STATES:
            self._shutdown_pending = False
        if target not in (State.ERROR_RECOVER, State.HOMING):
            self._homing_required = False
