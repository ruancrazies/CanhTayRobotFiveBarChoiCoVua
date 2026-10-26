"""Robot Commander and the driver rules of phase 4a (owner decisions 1 and 2).

* HOME never enables the drivers, so ``home()`` sends RESET (when needed), ENABLE, HOME.
* The Commander's own DISABLE ends every pending wait for a done with a defined outcome, never a hang:
  ``OperationAbortedError``, no longer homed, and the done of a Z move that DISABLE does not cancel is ignored and
  blocks new operations until it has arrived.
* error:6 on a MOVE makes the Commander forget its homing.

The timings are TEST VALUES. A "hang" is detected by giving the done a long timeout (10 s) and requiring the
operation to end far sooner (``PROMPT_S``): ending by the timeout would pass for a hang in disguise.
"""

from __future__ import annotations

import threading
import time
from pathlib import Path
from typing import Any, Callable, Optional

import pytest

from config.protocol_commands import AlarmCode, CommandName, ErrorCode
from desktop_app.hardware.robot_commander import (
    CommandFailedError,
    CommanderError,
    NotReadyError,
    OperationAbortedError,
    RobotAlarmError,
)
from phase3_support import Stack, build_stack
from sim_support import wait_for
from test_robot_commander import plan_for

LONG_DONE_TIMEOUT_S = 10.0
PROMPT_S = 2.0  # an aborted wait must end within this, far below LONG_DONE_TIMEOUT_S

N = CommandName


def slow_stack(tmp_path: Path, *, sim: Optional[dict[str, Any]] = None, settings: Optional[dict[str, Any]] = None) -> Stack:
    settings_values: dict[str, Any] = {"done_timeout_s": LONG_DONE_TIMEOUT_S}
    settings_values.update(settings or {})
    return build_stack(tmp_path, sim_overrides=sim, settings_overrides=settings_values)


class Runner:
    """Run ``function`` in a thread and keep what it raised."""

    def __init__(self, function: Callable[[], Any]) -> None:
        self.error: Optional[BaseException] = None
        self.done = threading.Event()
        self.started_at = time.monotonic()
        self.finished_at: Optional[float] = None

        def body() -> None:
            try:
                function()
            except BaseException as exc:  # noqa: BLE001 - the test inspects it
                self.error = exc
            finally:
                self.finished_at = time.monotonic()
                self.done.set()

        self.thread = threading.Thread(target=body, daemon=True)
        self.thread.start()

    def finish(self, timeout_s: float = PROMPT_S) -> Optional[BaseException]:
        assert self.done.wait(timeout_s), "the operation is still waiting: a hang"
        return self.error


def received(stack: Stack, name: str) -> int:
    return sum(1 for line in stack.sim.received if line.split()[0] == name)


def sent_done_lines(stack: Stack) -> int:
    return stack.sim.sent.count("done")


# --------------------------------------------------------------------------- #
# HOME never enables the drivers: the Commander does
# --------------------------------------------------------------------------- #


def test_home_sends_enable_before_home(tmp_path: Path) -> None:
    stack = build_stack(tmp_path)
    try:
        stack.commander.home()
        assert [line for line in stack.sent_commands()][-2:] == [N.ENABLE.value, N.HOME.value]
        assert stack.sim.homed
    finally:
        stack.close()


def test_home_after_a_disable_works_because_it_enables_first(tmp_path: Path) -> None:
    stack = build_stack(tmp_path)
    try:
        stack.commander.home()
        stack.commander.disable_drivers()
        assert not stack.sim.drivers_enabled and not stack.commander.homed
        stack.commander.home()
        assert stack.sim.drivers_enabled and stack.sim.homed and stack.commander.homed
        names = stack.sent_commands()
        assert names[-3:] == [N.DISABLE.value, N.ENABLE.value, N.HOME.value]
    finally:
        stack.close()


def test_after_an_alarm_home_is_reset_then_enable_then_home(tmp_path: Path) -> None:
    stack = build_stack(tmp_path)
    try:
        stack.commander.home()
        stack.sim.inject_estop()
        assert wait_for(lambda: AlarmCode.ESTOP.value in stack.alarm_requests)
        stack.commander.home()
        assert stack.sent_commands()[-3:] == [N.RESET.value, N.ENABLE.value, N.HOME.value]
    finally:
        stack.close()


def test_a_bare_home_with_the_drivers_disabled_is_error_6(tmp_path: Path) -> None:
    """The firmware side of the rule, seen through the Commander: a HOME the PC sends without ENABLE is refused."""
    stack = build_stack(tmp_path)
    try:
        stack.commander.disable_drivers()
        with pytest.raises(CommandFailedError) as raised:
            stack.commander.bench_command(N.HOME.value)
        assert raised.value.code is ErrorCode.DRIVERS_DISABLED
        assert not stack.sim.drivers_enabled  # HOME did not enable them
        stack.commander.bench_command(N.HOME.value, Z=True)  # a Z-only HOME needs no joint driver
        assert not stack.sim.drivers_enabled
    finally:
        stack.close()


# --------------------------------------------------------------------------- #
# error:6 on a MOVE
# --------------------------------------------------------------------------- #


def test_error_6_on_a_move_stops_the_chain_without_a_retry_and_forgets_the_homing(tmp_path: Path) -> None:
    """The ESP32's drivers were disabled behind the Commander's back: the first MOVE says so, once."""
    stack = build_stack(tmp_path)
    try:
        stack.commander.home()
        assert stack.commander.homed
        with stack.sim._lock:  # white-box: somebody else disabled the drivers (the Commander was not told)
            stack.sim._drivers_enabled = False
        _planner, chain = plan_for("normal")
        with pytest.raises(CommandFailedError) as raised:
            stack.commander.execute_chain(chain)
        assert raised.value.code is ErrorCode.DRIVERS_DISABLED
        assert received(stack, N.MOVE.value) == 1, "error:6 is not retried like error:3"
        assert stack.commander.needs_homing
    finally:
        stack.close()


# --------------------------------------------------------------------------- #
# The Commander's own DISABLE ends every pending wait for a done
# --------------------------------------------------------------------------- #


def test_disable_ends_a_pending_home_wait(tmp_path: Path) -> None:
    stack = slow_stack(tmp_path, sim={"home_joints_s": 30.0})  # the joint phase would take 30 s
    try:
        runner = Runner(stack.commander.home)
        assert wait_for(lambda: received(stack, N.HOME.value) == 1)
        time.sleep(0.1)  # the HOME is acknowledged and the Commander waits for its done
        stack.commander.disable_drivers()
        error = runner.finish()
        assert isinstance(error, OperationAbortedError), error
        assert not isinstance(error, RobotAlarmError)
        assert not stack.commander.homed and stack.commander.needs_homing
        assert sent_done_lines(stack) == 0, "the firmware cancelled the done: nothing to wait for"
        assert stack.alarm_requests == [], "DISABLE is a command, not an alarm: the FSM is not told"
        assert (runner.finished_at or 0) - runner.started_at < PROMPT_S
    finally:
        stack.close()


def test_disable_ends_a_pending_wait(tmp_path: Path) -> None:
    stack = slow_stack(tmp_path)
    try:
        stack.commander.home()
        dones_before = sent_done_lines(stack)  # the HOME's
        stack.commander.bench_command(N.MOVE.value, A=170.0, B=60.0, F=1.0)  # tens of seconds at 1 deg/s
        runner = Runner(lambda: stack.commander.bench_command(N.WAIT.value))
        assert wait_for(lambda: received(stack, N.WAIT.value) == 1)
        time.sleep(0.1)
        stack.commander.disable_drivers()
        assert isinstance(runner.finish(), OperationAbortedError)
        assert stack.sim.queue_length == 0  # the queued MOVE was discarded, not drained
        time.sleep(0.1)
        assert sent_done_lines(stack) == dones_before  # no done: the WAIT's was cancelled
        assert not stack.commander.homed
    finally:
        stack.close()


def test_disable_ends_a_running_chain_and_nothing_more_is_sent(tmp_path: Path) -> None:
    stack = slow_stack(tmp_path, settings={"move_speed_deg_s": 20.0})
    try:
        stack.commander.home()
        _planner, chain = plan_for("normal")
        runner = Runner(lambda: stack.commander.execute_chain(chain))
        assert wait_for(lambda: received(stack, N.MOVE.value) >= 2)
        stack.commander.disable_drivers()
        assert isinstance(runner.finish(), OperationAbortedError)
        sent_then = len(stack.sim.received)
        time.sleep(0.2)
        late = [line.split()[0] for line in stack.sim.received[sent_then:] if line.split()[0] != N.PING.value]
        assert late == [], f"the aborted chain went on sending {late}"
        assert stack.sim.queue_length == 0 and not stack.commander.homed
    finally:
        stack.close()


def test_disable_ends_a_pending_z_wait_and_its_late_done_is_ignored(tmp_path: Path) -> None:
    """DISABLE does not stop Z: its done still arrives. It must not complete anything the Commander does next."""
    stack = slow_stack(tmp_path, sim={"z_travel_s": 0.6})
    try:
        stack.commander.home()
        runner = Runner(lambda: stack.commander.bench_command(N.ZDOWN.value))
        assert wait_for(lambda: received(stack, N.ZDOWN.value) == 1)
        time.sleep(0.1)
        stack.commander.disable_drivers()
        assert isinstance(runner.finish(), OperationAbortedError)
        # Z is still moving: a new operation is refused (not queued behind a done that would be mistaken for its own)
        with pytest.raises(NotReadyError, match="still finishing"):
            stack.commander.home()
        assert wait_for(lambda: stack.commander._stale_z_dones == 0, timeout_s=3.0), "the Z done never arrived"
        stack.commander.home()  # RESET is not needed; ENABLE then HOME; the late done did not complete it early
        assert stack.sim.homed and stack.commander.homed
    finally:
        stack.close()


def test_a_z_move_acknowledged_after_our_disable_is_handled_too(tmp_path: Path) -> None:
    """The ZDOWN was sent before DISABLE but its ok arrives after the Commander changed its state."""
    stack = slow_stack(tmp_path, sim={"z_travel_s": 0.4})
    try:
        stack.commander.home()
        runner = Runner(lambda: stack.commander.bench_command(N.ZDOWN.value))
        stack.commander.disable_drivers()  # may or may not be before the ok of ZDOWN: both orders must work
        error = runner.finish()
        assert error is None or isinstance(error, (OperationAbortedError, CommanderError)), error
        assert wait_for(lambda: stack.commander._stale_z_dones == 0 and stack.sim.z_state.value == "BOTTOM", timeout_s=3.0)
        assert not stack.commander._done_fifo, "a dead request must not wait for a later done"
    finally:
        stack.close()


def test_an_alarm_after_disable_clears_the_wait_for_the_z_done(tmp_path: Path) -> None:
    stack = slow_stack(tmp_path, sim={"z_travel_s": 5.0})
    try:
        stack.commander.home()
        runner = Runner(lambda: stack.commander.bench_command(N.ZDOWN.value))
        assert wait_for(lambda: received(stack, N.ZDOWN.value) == 1)
        time.sleep(0.1)
        stack.commander.disable_drivers()
        runner.finish()
        assert stack.commander._stale_z_dones == 1
        stack.sim.inject_estop()  # an alarm stops Z: the firmware owes no done any more
        assert wait_for(lambda: AlarmCode.ESTOP.value in stack.alarm_requests)
        assert stack.commander._stale_z_dones == 0
        stack.commander.home()  # RESET, ENABLE, HOME
        assert stack.sim.homed
    finally:
        stack.close()


def test_disable_that_cannot_be_delivered_still_forgets_the_homing_and_ends_the_wait(tmp_path: Path) -> None:
    stack = slow_stack(tmp_path, sim={"home_joints_s": 30.0}, settings={"reply_timeout_s": 0.3})
    try:
        runner = Runner(stack.commander.home)
        assert wait_for(lambda: received(stack, N.HOME.value) == 1)
        time.sleep(0.1)
        stack.pair.cut()  # the cable goes: the link reports the loss, which is an alarm and also ends the wait
        error = runner.finish()
        assert isinstance(error, (RobotAlarmError, OperationAbortedError, NotReadyError)), error
        with pytest.raises(CommanderError):
            stack.commander.disable_drivers()  # nothing can be sent now
        assert not stack.commander.homed
    finally:
        stack.pair.restore()
        stack.close()


def test_a_bench_disable_is_the_same_path(tmp_path: Path) -> None:
    """bench_command(DISABLE) must not queue behind the operation lock of the very operation it has to end."""
    stack = slow_stack(tmp_path, sim={"home_joints_s": 30.0})
    try:
        runner = Runner(stack.commander.home)
        assert wait_for(lambda: received(stack, N.HOME.value) == 1)
        time.sleep(0.1)
        begun = time.monotonic()
        stack.commander.bench_command(N.DISABLE.value)
        assert time.monotonic() - begun < PROMPT_S
        assert isinstance(runner.finish(), OperationAbortedError)
        assert not stack.sim.drivers_enabled
    finally:
        stack.close()
