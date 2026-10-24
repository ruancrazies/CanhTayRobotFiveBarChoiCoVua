"""Injected faults: ESTOP, LIMIT_Z, Z_TIMEOUT, SENSOR_FAULT, STEP_LOSS, lost connection, heartbeat timeout.

Every fault is put at an exact point of the command stream with the simulator's command hook (no timing race),
in the middle of a pick-and-place chain. Each must end with the FSM in ERROR_RECOVER, the engine move still
pending, and the Commander refusing to run anything until the arm is homed again.
"""

from __future__ import annotations

from pathlib import Path
from typing import Any, Callable

import pytest

from config.protocol_commands import AlarmCode, CommandName, ErrorCode
from desktop_app.core.fsm import HOMING_MANDATORY_ALARMS, State, Trigger
from desktop_app.hardware.robot_commander import CommandFailedError, NotAtStartError, NotReadyError, RobotAlarmError
from desktop_app.hardware.esp32_simulator import Esp32Simulator
from desktop_app.utils.hardware_config import load_hardware_config
from phase3_support import SCENARIOS, Stack, build_stack
from sim_support import wait_for

NTH_MOVE = 5


def fault_when(stack: Stack, command: str, nth: int, inject: Callable[[Esp32Simulator], None]) -> None:
    """Call ``inject(simulator)`` right after the simulator handled the ``nth`` (1-based) ``command``."""
    seen = {"count": 0}

    def hook(line: str) -> None:
        if line.split()[0] == command:
            seen["count"] += 1
            if seen["count"] == nth:
                inject(stack.sim)

    stack.sim.add_command_hook(hook)


def step_loss_offset() -> float:
    return load_hardware_config().electrical.as5600.system_error_deg * 3  # clearly beyond the sensor error


FAULTS: dict[str, tuple[str, int, Callable[[Esp32Simulator], None], str]] = {
    # name: (command that triggers it, which one, injection, alarm the FSM must receive)
    "estop_mid_move": (CommandName.MOVE.value, NTH_MOVE, lambda sim: sim.inject_estop(), "ESTOP"),
    "estop_while_z_down": (CommandName.ZDOWN.value, 1, lambda sim: sim.inject_estop(), "ESTOP"),
    "limit_z": (CommandName.ZDOWN.value, 1, lambda sim: sim.inject_limit_z(), "LIMIT_Z"),
    "z_timeout": (CommandName.ZDOWN.value, 1, lambda sim: sim.inject_z_stall(), "Z_TIMEOUT"),
    "sensor_fault": (CommandName.MOVE.value, NTH_MOVE, lambda sim: sim.inject_sensor_fault(), "SENSOR_FAULT"),
    "step_loss": (CommandName.MOVE.value, NTH_MOVE, lambda sim: sim.inject_step_loss(step_loss_offset()), "STEP_LOSS"),
}


def run_until_fault(stack: Stack) -> RobotAlarmError:
    stack.homing()
    stack.reach_plan(*SCENARIOS["normal"][1:])
    stack.plan()
    failure = stack.execute()
    assert isinstance(failure, RobotAlarmError), failure
    return failure


@pytest.mark.parametrize("name", sorted(FAULTS))
def test_a_fault_mid_chain_leads_to_error_recover(tmp_path: Path, name: str) -> None:
    command, nth, inject, alarm = FAULTS[name]
    stack = build_stack(tmp_path)
    try:
        fault_when(stack, command, nth, inject)
        failure = run_until_fault(stack)

        assert failure.code == alarm
        assert stack.alarm_requests == [alarm], "the alarm must reach the FSM exactly once"
        assert stack.fsm.state is State.ERROR_RECOVER
        assert stack.game.pending_move is not None, "the engine move is kept and redone after recovery"
        assert stack.fsm.homing_required  # every firmware alarm makes homing mandatory
        assert stack.commander.needs_homing  # the Commander never trusts the arm after an alarm
        assert stack.commander.last_alarm is not None and stack.commander.last_alarm.code == alarm
        assert stack.sim.violations == []
        # nothing is sent after the alarm: no further MOVE reaches the ESP32
        count = len(stack.sent_commands())
        with pytest.raises(NotReadyError):
            stack.commander.execute_chain(stack.chain)
        assert len(stack.sent_commands()) == count
    finally:
        stack.close()


def test_the_firmware_alarm_is_published_before_the_chain_raises(tmp_path: Path) -> None:
    """When execute_chain raises, the FSM is already in ERROR_RECOVER (not still in ROBOT_EXEC)."""
    stack = build_stack(tmp_path)
    try:
        fault_when(stack, CommandName.MOVE.value, NTH_MOVE, lambda sim: sim.inject_estop())
        stack.homing()
        stack.reach_plan(*SCENARIOS["normal"][1:])
        chain = stack.plan()
        with pytest.raises(RobotAlarmError):
            stack.commander.execute_chain(chain)
        assert stack.fsm.state is State.ERROR_RECOVER
    finally:
        stack.close()


def test_a_second_alarm_is_not_published_twice(tmp_path: Path) -> None:
    stack = build_stack(tmp_path)
    try:
        fault_when(stack, CommandName.MOVE.value, NTH_MOVE, lambda sim: sim.inject_estop())
        run_until_fault(stack)
        stack.pair.device.write(b"!alarm:ESTOP\n")  # the firmware repeats it
        assert wait_for(lambda: stack.commander.alarms_published.count("ESTOP") >= 1)
        assert stack.alarm_requests.count("ESTOP") <= 2
        assert stack.fsm.state is State.ERROR_RECOVER
    finally:
        stack.close()


# --------------------------------------------------------------------------- #
# Lost connection and heartbeat timeout
# --------------------------------------------------------------------------- #


def test_a_lost_connection_leads_to_error_recover_as_hb_timeout(tmp_path: Path) -> None:
    stack = build_stack(tmp_path)
    try:
        fault_when(stack, CommandName.MOVE.value, NTH_MOVE, lambda sim: sim.inject_lost_connection())
        failure = run_until_fault(stack)
        assert failure.code == AlarmCode.HB_TIMEOUT.value
        assert stack.alarm_requests == [AlarmCode.HB_TIMEOUT.value]
        assert stack.fsm.state is State.ERROR_RECOVER
        assert stack.fsm.homing_required  # HB_TIMEOUT makes homing mandatory
        assert stack.game.pending_move is not None
        assert stack.commander.needs_homing
    finally:
        stack.close()


def test_a_heartbeat_timeout_from_the_firmware_leads_to_error_recover(tmp_path: Path) -> None:
    """The PC pings far less often than the firmware tolerates: the ESP32 stops by itself and says so."""
    stack = build_stack(
        tmp_path,
        sim_overrides=dict(heartbeat_timeout_s=0.1),
        settings_overrides=dict(ping_interval_s=30.0),
    )
    try:
        stack.homing()
        stack.reach_plan(*SCENARIOS["normal"][1:])
        stack.plan()
        assert wait_for(lambda: stack.alarm_requests == ["HB_TIMEOUT"], 3.0)
        assert stack.fsm.state is State.ERROR_RECOVER
        assert stack.fsm.homing_required
        assert tuple(stack.sim.alarms) == (AlarmCode.HB_TIMEOUT,)
        assert stack.commander.needs_homing
        failure = stack.execute()
        assert isinstance(failure, NotReadyError)  # nothing is sent after the alarm
    finally:
        stack.close()


def test_unanswered_pings_are_reported_once_per_episode(tmp_path: Path) -> None:
    stack = build_stack(tmp_path, settings_overrides=dict(ping_interval_s=0.01, reply_timeout_s=0.05))
    try:
        stack.commander.home()
        stack.pair.cut()
        assert wait_for(lambda: bool(stack.alarm_requests), 3.0)
        import time

        time.sleep(0.2)
        assert stack.alarm_requests == [AlarmCode.HB_TIMEOUT.value]
    finally:
        stack.close()


# --------------------------------------------------------------------------- #
# Recovery
# --------------------------------------------------------------------------- #


def test_recovery_after_a_fault_before_any_motion(tmp_path: Path) -> None:
    """ESTOP while the arm is still parked: RESYNC, HOMING (RESET then HOME), PLAN again, the chain runs."""
    stack = build_stack(tmp_path)
    try:
        fault_when(stack, CommandName.WAIT.value, 1, lambda sim: sim.inject_estop())  # the start-point check
        run_until_fault(stack)
        assert stack.fsm.state is State.ERROR_RECOVER and stack.fsm.homing_required

        assert stack.fsm.handle(Trigger.RESYNC).accepted
        assert stack.fsm.state is State.HOMING
        stack.commander.home()
        names = [line.split()[0] for line in stack.sim.received if line.split()[0] != CommandName.PING.value]
        assert names.index(CommandName.RESET.value) < len(names) - 1
        assert names[-1] == CommandName.HOME.value and names[-2] == CommandName.RESET.value
        assert stack.fsm.handle(Trigger.HOMING_DONE).accepted
        stack.redo_pending()  # SETUP_CHECK, then START(pending="redo"): the engine move was kept

        stack.plan()
        assert stack.execute() is None
        assert stack.verify() == []
        assert stack.fsm.state is State.WAIT_HUMAN
        assert stack.sim.violations == []
    finally:
        stack.close()


def test_recovery_after_a_fault_mid_chain_is_refused_until_the_arm_is_back_at_the_start(tmp_path: Path) -> None:
    """HOME leaves the arm where it is, so a chain restarted mid-board must not run (a human must park the arm).

    The fault is put at the first ZDOWN, not at "the fifth MOVE": the WAIT before ZDOWN guarantees that the arm
    has finished its moves and stands over the first piece, whatever the resolution of the machine's clock.
    (With a 15.6 ms clock tick, as on Windows, MOVEs queued back to back make no progress before the fault.)
    """
    stack = build_stack(tmp_path)
    try:
        fault_when(stack, CommandName.ZDOWN.value, 1, lambda sim: sim.inject_estop())
        run_until_fault(stack)
        assert stack.fsm.handle(Trigger.RESYNC).accepted
        stack.commander.home()
        assert stack.fsm.handle(Trigger.HOMING_DONE).accepted
        stack.redo_pending()
        stack.plan()
        count_moves = sum(1 for line in stack.sim.received if line.split()[0] == CommandName.MOVE.value)
        failure = stack.execute()
        assert isinstance(failure, NotAtStartError)
        assert sum(1 for line in stack.sim.received if line.split()[0] == CommandName.MOVE.value) == count_moves
        assert stack.fsm.state is State.ERROR_RECOVER
    finally:
        stack.close()


# --------------------------------------------------------------------------- #
# Every firmware alarm makes homing mandatory (SPEC 5.3, 7.2)
# --------------------------------------------------------------------------- #


def test_the_fsm_demands_homing_after_every_declared_alarm() -> None:
    assert set(HOMING_MANDATORY_ALARMS) == {code.value for code in AlarmCode}


@pytest.mark.parametrize("alarm", ["LIMIT_Z", "Z_TIMEOUT"])
def test_after_a_z_alarm_the_loop_goes_through_homing_and_does_not_spin(tmp_path: Path, alarm: str) -> None:
    """LIMIT_Z and Z_TIMEOUT used to resync straight to PLAN, where the Commander refused: an endless loop."""
    command, nth, inject, _ = FAULTS[alarm.lower()]
    stack = build_stack(tmp_path)
    try:
        fault_when(stack, command, nth, inject)
        run_until_fault(stack)
        assert stack.fsm.homing_required and stack.commander.needs_homing

        assert stack.fsm.handle(Trigger.RESYNC).accepted
        assert stack.fsm.state is State.HOMING  # not PLAN
        stack.commander.home()  # RESET, then HOME
        assert stack.fsm.handle(Trigger.HOMING_DONE).accepted
        assert stack.sim.z_state.value == "TOP" and stack.sim.homed
        assert stack.commander.homed
    finally:
        stack.close()


def test_the_firmware_answers_error_4_after_limit_z_until_homed(tmp_path: Path) -> None:
    stack = build_stack(tmp_path)
    try:
        fault_when(stack, CommandName.ZDOWN.value, 1, lambda sim: sim.inject_limit_z())
        run_until_fault(stack)
        stack.commander.bench_command(CommandName.RESET.value)  # clears the alarm; homing is still needed
        with pytest.raises(CommandFailedError) as raised:
            stack.commander.bench_command(CommandName.MOVE.value, A=150.0, B=30.0, F=100.0)
        assert raised.value.code is ErrorCode.NOT_HOMED
    finally:
        stack.close()
