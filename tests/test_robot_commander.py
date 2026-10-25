"""Robot Commander: the plan, the placeholder guard, the heartbeat, replies and errors (phase 3).

The whole-stack tests (FSM to simulator) are in test_robot_commander_e2e.py and the injected faults
in test_robot_commander_faults.py.
"""

from __future__ import annotations

import logging
from pathlib import Path
from typing import Any, Optional

import chess
import pytest

from config.protocol_commands import COMMANDS, AlarmCode, CommandName, ErrorCode
from desktop_app.core.fsm import ALARM_CODES, State
from desktop_app.core.move_planner import GripStep, MoveChain, MoveStep, ZDownStep, ZUpStep, ZoneInventory
from desktop_app.hardware.robot_commander import (
    BenchModeError,
    CommandFailedError,
    CommanderError,
    CommanderSettings,
    GuardRefusedError,
    NotAtStartError,
    NotReadyError,
    PlanError,
    PlanKind,
    ReplyTimeoutError,
    RobotAlarmError,
    build_park_plan,
    build_plan,
    _plan_steps,
    chain_start_joints,
    park_start_joints,
    publish_alarm,
)
from desktop_app.core.event_bus import EventBus
from desktop_app.core.fsm import FsmRequest, Trigger
from phase2_support import E2E4, E7E5, make_planner
from phase3_support import RealLookingTransport, SCENARIOS, build_stack, commander_settings, park_joints
from sim_support import wait_for


def plan_for(scenario: str = "normal") -> tuple[Any, MoveChain]:
    planner = make_planner()
    fen, human, engine = SCENARIOS[scenario]
    board = chess.Board(fen)
    board.push(chess.Move.from_uci(human))
    return planner, planner.plan(board, chess.Move.from_uci(engine), ZoneInventory())


# --------------------------------------------------------------------------- #
# The plan
# --------------------------------------------------------------------------- #


@pytest.mark.parametrize("scenario", sorted(SCENARIOS))
def test_plan_has_one_move_per_trajectory_point_after_the_first(scenario: str) -> None:
    _planner, chain = plan_for(scenario)
    plan = build_plan(chain, commander_settings())
    expected = sum(len(s.trajectory.joints_deg) - 1 for s in chain.steps if isinstance(s, MoveStep))
    assert sum(item.kind is PlanKind.MOVE for item in plan) == expected


@pytest.mark.parametrize("scenario", sorted(SCENARIOS))
def test_plan_orders_wait_z_and_grip_safely(scenario: str) -> None:
    _planner, chain = plan_for(scenario)
    plan = build_plan(chain, commander_settings())
    names = [item.command.name for item in plan]
    z_down = False
    for index, name in enumerate(names):
        if name == CommandName.MOVE.value:
            assert not z_down, "MOVE with Z down"
        elif name == CommandName.ZDOWN.value:
            assert names[index - 1] == CommandName.WAIT.value, "ZDOWN must follow a WAIT"
            z_down = True
        elif name == CommandName.ZUP.value:
            assert z_down
            z_down = False
        elif name == CommandName.GRIP.value:
            assert z_down, "GRIP with Z up"
    assert not z_down
    # the chain ends with Z up (ZUP has its done), or with the queue drained: never with MOVEs still queued
    assert names[-1] in (CommandName.ZUP.value, CommandName.WAIT.value)


def test_plan_sends_the_configured_speed_and_the_planner_angles() -> None:
    _planner, chain = plan_for()
    settings = commander_settings(move_speed_deg_s=123.0)
    plan = build_plan(chain, settings)
    first_move = next(item for item in plan if item.kind is PlanKind.MOVE).command
    assert first_move.params["F"] == 123.0
    first_step = chain.steps[0]
    assert isinstance(first_step, MoveStep)
    alpha, beta = first_step.trajectory.joints_deg[1]
    assert (first_move.params["A"], first_move.params["B"]) == (float(alpha), float(beta))


def test_every_planned_command_is_valid_for_the_protocol() -> None:
    _planner, chain = plan_for("capture_promotion")
    for item in list(build_plan(chain, commander_settings())) + list(build_park_plan(chain, commander_settings())):
        assert item.command.name in COMMANDS
        item.command.encode()  # raises on an invalid parameter


def test_chain_starts_where_the_park_leg_ends() -> None:
    planner, chain = plan_for()
    alpha, beta = park_joints()
    start = chain_start_joints(chain)
    assert start == pytest.approx((alpha, beta), abs=1e-6)
    assert park_start_joints(chain) != start  # the park leg starts where the chain ended


def test_plan_rejects_unsafe_chains() -> None:
    _planner, chain = plan_for()
    move = chain.steps[0]
    assert isinstance(move, MoveStep)
    grip = next(s for s in chain.steps if isinstance(s, GripStep))
    settings = commander_settings()

    def chain_of(*steps: Any) -> tuple[Any, ...]:
        return tuple(steps)

    with pytest.raises(PlanError):
        _plan_steps(chain_of(move, ZDownStep(), move), settings)  # MOVE with Z down
    with pytest.raises(PlanError):
        _plan_steps(chain_of(move, ZDownStep(), ZDownStep()), settings)
    with pytest.raises(PlanError):
        _plan_steps(chain_of(move, ZUpStep()), settings)  # ZUP at the top
    with pytest.raises(PlanError):
        _plan_steps(chain_of(move, grip), settings)  # GRIP with Z up
    with pytest.raises(PlanError):
        _plan_steps(chain_of(move, ZDownStep()), settings)  # ends with Z down


def test_settings_reject_non_positive_values() -> None:
    with pytest.raises(ValueError):
        commander_settings(move_speed_deg_s=0.0)
    with pytest.raises(ValueError):
        commander_settings(queue_retry_s=0.0)
    with pytest.raises(ValueError):
        commander_settings(grip_settle_s=-1.0)
    with pytest.raises(ValueError):
        commander_settings(acceleration=-1.0)


# --------------------------------------------------------------------------- #
# The placeholder guard
# --------------------------------------------------------------------------- #


def real_port_stack(tmp_path: Path, **kwargs: Any) -> Any:
    return build_stack(tmp_path, transport_wrapper=RealLookingTransport, **kwargs)


def test_the_loopback_is_not_subject_to_the_guard(tmp_path: Path) -> None:
    stack = build_stack(tmp_path)
    try:
        assert stack.planner.placeholders()  # zones are placeholders in the test planner
        assert stack.link.transport.is_simulated
        stack.run_full_move("normal")
    finally:
        stack.close()


def test_a_real_port_refuses_a_chain_while_placeholders_remain(tmp_path: Path) -> None:
    stack = real_port_stack(tmp_path)
    try:
        assert not stack.link.transport.is_simulated
        stack.commander.home()
        _planner, chain = plan_for()
        before = len(stack.sent_commands())
        with pytest.raises(GuardRefusedError) as raised:
            stack.commander.execute_chain(chain)
        assert "zones" in raised.value.placeholders
        with pytest.raises(GuardRefusedError):
            stack.commander.park(chain)
        assert len(stack.sent_commands()) == before, "the guard must refuse BEFORE any command is sent"
    finally:
        stack.close()


def test_the_guard_names_every_open_placeholder(tmp_path: Path) -> None:
    stack = real_port_stack(tmp_path, planner=make_planner(capture_slots=None, park=False))
    try:
        _planner, chain = plan_for()
        with pytest.raises(GuardRefusedError) as raised:
            stack.commander.execute_chain(chain)
        text = str(raised.value)
        for name in stack.planner.placeholders():
            assert name in text
    finally:
        stack.close()


def test_a_real_port_without_placeholders_is_allowed(tmp_path: Path) -> None:
    class Settled:
        def placeholders(self) -> tuple[str, ...]:
            return ()

    stack = real_port_stack(tmp_path)
    try:
        stack.commander._planner = Settled()  # the guard only asks for placeholders()
        stack.run_full_move("normal")
    finally:
        stack.close()


def test_bench_mode_needs_an_explicit_confirmation(tmp_path: Path) -> None:
    stack = real_port_stack(tmp_path)
    try:
        with pytest.raises(BenchModeError):
            stack.commander.enable_bench_mode(confirm_no_load=False)
        assert not stack.commander.bench_mode
    finally:
        stack.close()


def test_bench_mode_logs_a_loud_warning(tmp_path: Path, caplog: pytest.LogCaptureFixture) -> None:
    stack = real_port_stack(tmp_path)
    try:
        with caplog.at_level(logging.WARNING):
            stack.commander.enable_bench_mode(confirm_no_load=True)
        text = "\n".join(r.getMessage() for r in caplog.records if r.levelno >= logging.WARNING)
        assert "BENCH MODE ENABLED" in text and "NO PICK-AND-PLACE" in text
    finally:
        stack.close()


def test_bench_mode_never_runs_a_chain_but_runs_single_commands(
    tmp_path: Path, caplog: pytest.LogCaptureFixture
) -> None:
    stack = real_port_stack(tmp_path)
    try:
        stack.commander.enable_bench_mode(confirm_no_load=True)
        _planner, chain = plan_for()
        before = len(stack.sent_commands())
        with caplog.at_level(logging.WARNING):
            with pytest.raises(BenchModeError):
                stack.commander.execute_chain(chain)
            with pytest.raises(BenchModeError):
                stack.commander.park(chain)
        assert len(stack.sent_commands()) == before
        assert any("NEVER run" in r.getMessage() for r in caplog.records)
        # single commands work
        stack.commander.bench_command(CommandName.ENABLE.value)
        data = stack.commander.bench_command(CommandName.INFO.value)
        assert data is not None
        stack.commander.disable_bench_mode()
        assert not stack.commander.bench_mode
    finally:
        stack.close()


def test_bench_commands_are_refused_on_a_real_port_without_bench_mode(tmp_path: Path) -> None:
    stack = real_port_stack(tmp_path)
    try:
        with pytest.raises(BenchModeError):
            stack.commander.bench_command(CommandName.ENABLE.value)
    finally:
        stack.close()


def test_bench_command_validates_before_sending(tmp_path: Path) -> None:
    stack = build_stack(tmp_path)
    try:
        before = len(stack.sent_commands())
        with pytest.raises(CommanderError, match="invalid bench command"):
            stack.commander.bench_command(CommandName.GRIP.value, S=999.0)  # out of range
        with pytest.raises(CommanderError, match="invalid bench command"):
            stack.commander.bench_command("NOPE")
        assert len(stack.sent_commands()) == before
    finally:
        stack.close()


def test_homing_is_allowed_on_a_real_port_with_placeholders(tmp_path: Path) -> None:
    stack = real_port_stack(tmp_path)
    try:
        stack.commander.home()
        assert stack.commander.homed and stack.sim.homed
    finally:
        stack.close()


def test_a_bench_disable_makes_homing_mandatory_for_the_commander_too(tmp_path: Path) -> None:
    """DISABLE clears the firmware's homed flag (SPEC 5.1); the Commander must not go on believing it is homed."""
    stack = build_stack(tmp_path)
    try:
        stack.commander.home()
        assert stack.commander.homed and stack.sim.homed
        stack.commander.bench_command(CommandName.DISABLE.value)
        assert not stack.sim.homed
        assert not stack.commander.homed and stack.commander.needs_homing
        _planner, chain = plan_for()
        before = len(stack.sent_commands())
        with pytest.raises(NotReadyError, match="homing is required"):
            stack.commander.execute_chain(chain)
        assert len(stack.sent_commands()) == before  # refused up front: nothing reached the ESP32
        stack.commander.bench_command(CommandName.ENABLE.value)
        assert stack.commander.needs_homing  # ENABLE does not give the homing back
        stack.commander.home()
        assert stack.commander.homed and stack.sim.homed
    finally:
        stack.close()


def test_a_bench_command_is_not_a_reason_to_forget_the_homing_unless_it_is_disable(tmp_path: Path) -> None:
    stack = build_stack(tmp_path)
    try:
        stack.commander.home()
        for name in (CommandName.ENABLE.value, CommandName.PING.value, CommandName.INFO.value):
            stack.commander.bench_command(name)
        assert stack.commander.homed and stack.sim.homed
    finally:
        stack.close()


def test_in_an_estop_a_bench_disable_is_accepted_and_a_bench_enable_is_refused_until_reset(tmp_path: Path) -> None:
    stack = build_stack(tmp_path)
    try:
        stack.commander.home()
        stack.sim.inject_estop()
        assert wait_for(lambda: AlarmCode.ESTOP.value in stack.alarm_requests)
        stack.commander.bench_command(CommandName.DISABLE.value)  # accepted in an ESTOP
        assert not stack.sim.drivers_enabled
        with pytest.raises(CommandFailedError) as raised:
            stack.commander.bench_command(CommandName.ENABLE.value)
        assert raised.value.code is ErrorCode.IN_ESTOP
        assert not stack.sim.drivers_enabled
        stack.commander.bench_command(CommandName.RESET.value)
        stack.commander.bench_command(CommandName.ENABLE.value)
        assert stack.sim.drivers_enabled
        assert stack.commander.needs_homing
    finally:
        stack.close()


def test_the_commander_homing_after_an_estop_still_works_with_the_new_estop_rules(tmp_path: Path) -> None:
    """RESET then HOME is how the Commander recovers; both are accepted (RESET) or only legal after it (HOME)."""
    stack = build_stack(tmp_path)
    try:
        stack.commander.home()
        stack.sim.inject_estop()
        assert wait_for(lambda: AlarmCode.ESTOP.value in stack.alarm_requests)
        assert not stack.commander.homed
        stack.commander.home()
        assert stack.commander.homed and stack.sim.homed and not stack.sim.estop
    finally:
        stack.close()


def test_a_repeated_emergency_stop_is_harmless_for_the_commander(tmp_path: Path) -> None:
    stack = build_stack(tmp_path)
    try:
        stack.commander.home()
        stack.commander.emergency_stop()
        assert wait_for(lambda: stack.sim.estop)
        stack.commander.emergency_stop()  # the firmware accepts it again and repeats the alarm
        # Wait for the repeated alarm itself: left to arrive during the homing below, it would (rightly) abort it.
        assert wait_for(lambda: stack.alarm_requests.count(AlarmCode.ESTOP.value) >= 2)
        assert stack.sim.estop and stack.sim.drivers_enabled
        stack.commander.home()  # RESET + HOME recovers from it as usual
        assert stack.commander.homed and not stack.sim.estop
    finally:
        stack.close()


# --------------------------------------------------------------------------- #
# Heartbeat
# --------------------------------------------------------------------------- #


def count_pings(stack: Any) -> int:
    return sum(1 for line in stack.sim.received if line.split()[0] == CommandName.PING.value)


def test_ping_is_sent_periodically_from_start(tmp_path: Path) -> None:
    stack = build_stack(tmp_path, settings_overrides=dict(ping_interval_s=0.01))
    try:
        assert wait_for(lambda: count_pings(stack) >= 5)
    finally:
        stack.close()


def test_ping_stops_when_the_commander_stops(tmp_path: Path) -> None:
    stack = build_stack(tmp_path, settings_overrides=dict(ping_interval_s=0.01))
    try:
        assert wait_for(lambda: count_pings(stack) >= 2)
        stack.commander.stop()
        settled = count_pings(stack)
        import time

        time.sleep(0.1)
        assert count_pings(stack) <= settled + 1
    finally:
        stack.close()


def test_ping_keeps_the_simulator_heartbeat_alive_during_a_chain(tmp_path: Path) -> None:
    stack = build_stack(
        tmp_path,
        "capture",
        sim_overrides=dict(heartbeat_timeout_s=0.15),
        settings_overrides=dict(ping_interval_s=0.02),
    )
    try:
        stack.run_full_move("capture")
        assert tuple(stack.sim.alarms) == ()
        assert stack.alarm_requests == []
    finally:
        stack.close()


def test_ping_bypasses_a_full_in_flight_window(tmp_path: Path) -> None:
    from desktop_app.hardware.serial_link import LinkSettings

    stack = build_stack(
        tmp_path,
        link_settings=LinkSettings(max_in_flight=1, read_poll_s=0.005),
        settings_overrides=dict(ping_interval_s=0.01),
    )
    try:
        stack.run_full_move("normal")  # MOVEs occupy the single slot all the time; PING still gets through
        assert count_pings(stack) >= 3
        assert stack.alarm_requests == []
    finally:
        stack.close()


def test_start_twice_is_an_error(tmp_path: Path) -> None:
    stack = build_stack(tmp_path)
    try:
        with pytest.raises(Exception):
            stack.commander.start()
    finally:
        stack.close()


# --------------------------------------------------------------------------- #
# Alarms to the bus
# --------------------------------------------------------------------------- #


def test_every_declared_alarm_is_an_fsm_alarm_code() -> None:
    assert {code.value for code in AlarmCode} <= set(ALARM_CODES)


@pytest.mark.parametrize("code", [c.value for c in AlarmCode])
def test_publish_alarm_sends_an_alarm_request_with_that_code(code: str) -> None:
    bus = EventBus()
    seen: list[FsmRequest] = []
    bus.subscribe(FsmRequest, seen.append)
    publish_alarm(bus, code)
    assert [(r.trigger, dict(r.payload)) for r in seen] == [(Trigger.ALARM, {"alarm": code})]


@pytest.mark.parametrize("code", [c.value for c in AlarmCode])
def test_an_unsolicited_alarm_is_published_once_and_latches(tmp_path: Path, code: str) -> None:
    stack = build_stack(tmp_path)
    try:
        stack.commander.home()
        stack.pair.device.write(f"!alarm:{code}\n".encode())
        assert wait_for(lambda: stack.alarm_requests == [code])
        assert stack.commander.needs_homing
        last = stack.commander.last_alarm
        assert last is not None and last.code == code
        assert stack.commander.alarms_published == [code]
    finally:
        stack.close()


def test_an_undeclared_alarm_triggers_an_emergency_stop(tmp_path: Path) -> None:
    stack = build_stack(tmp_path)
    try:
        stack.commander.home()
        stack.pair.device.write(b"!alarm:MYSTERY\n")
        assert wait_for(lambda: stack.sim.estop)
        assert wait_for(lambda: AlarmCode.ESTOP.value in stack.alarm_requests)
        assert "MYSTERY" not in stack.alarm_requests
    finally:
        stack.close()


def test_emergency_stop_sets_the_estop_flag_and_publishes(tmp_path: Path) -> None:
    stack = build_stack(tmp_path)
    try:
        stack.commander.home()
        stack.commander.emergency_stop()
        assert wait_for(lambda: stack.sim.estop)
        assert wait_for(lambda: stack.alarm_requests == ["ESTOP"])
        assert stack.commander.needs_homing
    finally:
        stack.close()


def test_emergency_stop_publishes_even_when_the_link_is_down(tmp_path: Path) -> None:
    stack = build_stack(tmp_path, start_commander=True)
    try:
        stack.link.stop()
        stack.commander.emergency_stop()
        assert stack.alarm_requests == ["ESTOP"]
    finally:
        stack.close()


# --------------------------------------------------------------------------- #
# Readiness, start position, replies
# --------------------------------------------------------------------------- #


def test_a_chain_needs_homing_first(tmp_path: Path) -> None:
    stack = build_stack(tmp_path)
    try:
        _planner, chain = plan_for()
        before = len(stack.sent_commands())
        with pytest.raises(NotReadyError):
            stack.commander.execute_chain(chain)
        assert [l for l in stack.sent_commands()[before:] if l.split()[0] == CommandName.MOVE.value] == []
    finally:
        stack.close()


def test_a_chain_is_refused_when_the_arm_is_not_at_its_first_point(tmp_path: Path) -> None:
    alpha, beta = park_joints()
    stack = build_stack(tmp_path, sim_overrides=dict(initial_joint_deg=(alpha + 10.0, beta)))
    try:
        stack.commander.home()
        _planner, chain = plan_for()
        with pytest.raises(NotAtStartError) as raised:
            stack.commander.execute_chain(chain)
        assert "start" in str(raised.value)
        assert not [l for l in stack.sim.received if l.split()[0] == CommandName.MOVE.value]
    finally:
        stack.close()


def test_start_tolerance_is_a_setting_not_a_loosened_check(tmp_path: Path) -> None:
    alpha, beta = park_joints()
    offset = 0.5
    stack = build_stack(
        tmp_path,
        sim_overrides=dict(initial_joint_deg=(alpha + offset, beta)),
        settings_overrides=dict(start_tolerance_deg=offset / 2),
    )
    try:
        stack.commander.home()
        _planner, chain = plan_for()
        with pytest.raises(NotAtStartError):
            stack.commander.execute_chain(chain)
    finally:
        stack.close()


def test_position_returns_the_pos_data_line(tmp_path: Path) -> None:
    stack = build_stack(tmp_path)
    try:
        data = stack.commander.position()
        assert float(data.fields["A"]) == pytest.approx(park_joints()[0], abs=1e-3)
        assert data.fields["Z"] == "TOP"
    finally:
        stack.close()


def test_move_before_homing_on_the_wire_is_error_4(tmp_path: Path) -> None:
    stack = build_stack(tmp_path)
    try:
        with pytest.raises(CommandFailedError) as raised:
            stack.commander.bench_command(CommandName.MOVE.value, A=150.0, B=30.0, F=100.0)
        assert raised.value.code is ErrorCode.NOT_HOMED
    finally:
        stack.close()


def test_a_reply_timeout_is_reported(tmp_path: Path) -> None:
    stack = build_stack(tmp_path, settings_overrides=dict(reply_timeout_s=0.1))
    try:
        stack.commander.home()
        stack.pair.cut()  # the ESP32 never hears the next command
        with pytest.raises(Exception) as raised:
            stack.commander.position()
        assert isinstance(raised.value, (ReplyTimeoutError, NotReadyError, RobotAlarmError))
    finally:
        stack.close()


def test_acceleration_is_sent_before_a_chain_when_configured(tmp_path: Path) -> None:
    stack = build_stack(tmp_path, settings_overrides=dict(acceleration=250.0))
    try:
        stack.run_full_move("normal")
        names = [line.split()[0] for line in stack.sim.received]
        assert CommandName.ACCEL.value in names
        assert names.index(CommandName.ACCEL.value) < names.index(CommandName.MOVE.value)
    finally:
        stack.close()


def test_no_acceleration_command_when_not_configured(tmp_path: Path) -> None:
    stack = build_stack(tmp_path)
    try:
        stack.run_full_move("normal")
        assert CommandName.ACCEL.value not in [line.split()[0] for line in stack.sim.received]
    finally:
        stack.close()


def test_the_simulator_saw_no_protocol_violation(tmp_path: Path) -> None:
    stack = build_stack(tmp_path, "capture_promotion")
    try:
        stack.run_full_move("capture_promotion")
        assert stack.sim.violations == []
    finally:
        stack.close()


# --------------------------------------------------------------------------- #
# The injected clock: deadlines and the heartbeat schedule follow it, not the wall clock
# --------------------------------------------------------------------------- #


def muted_stack(tmp_path: Path, clock: Any, **settings: Any) -> tuple[Any, Any]:
    from phase3_support import MutedTransport

    holder: dict[str, Any] = {}

    def wrap(inner: Any) -> Any:
        holder["transport"] = MutedTransport(inner)
        return holder["transport"]

    # the simulator runs on the wall clock: give it a heartbeat limit longer than any of these tests
    stack = build_stack(
        tmp_path, transport_wrapper=wrap, clock=clock, settings_overrides=settings, sim_overrides=dict(heartbeat_timeout_s=60.0)
    )
    return stack, holder["transport"]


def test_pings_follow_the_injected_clock(tmp_path: Path) -> None:
    from sim_support import ManualClock

    clock = ManualClock()
    stack, _transport = muted_stack(tmp_path, clock, ping_interval_s=10.0)
    try:
        assert wait_for(lambda: count_pings(stack) == 1)  # the first one goes out at start
        import time

        time.sleep(0.2)  # real time passes, the injected clock does not: no second PING
        assert count_pings(stack) == 1
        for expected in (2, 3):
            clock.advance(10.0)
            assert wait_for(lambda: count_pings(stack) == expected), expected
    finally:
        stack.close()


def test_a_reply_timeout_is_decided_by_the_injected_clock(tmp_path: Path) -> None:
    import threading
    import time

    from sim_support import ManualClock

    clock = ManualClock()
    stack, transport = muted_stack(tmp_path, clock, ping_interval_s=1000.0, reply_timeout_s=2.0)
    try:
        assert wait_for(lambda: count_pings(stack) == 1)
        transport.muted = True  # the link stays up, nobody answers
        outcome: dict[str, BaseException] = {}

        def ask() -> None:
            try:
                stack.commander.position()
            except BaseException as exc:  # noqa: BLE001 - the test reports whatever happened
                outcome["error"] = exc

        thread = threading.Thread(target=ask, daemon=True)
        thread.start()
        time.sleep(0.3)
        assert thread.is_alive(), "the timeout must not run on the wall clock"
        clock.advance(2.5)
        thread.join(timeout=2.0)
        assert not thread.is_alive()
        assert isinstance(outcome.get("error"), ReplyTimeoutError)
    finally:
        stack.close()


def test_an_unanswered_ping_raises_hb_timeout_once_when_the_injected_clock_says_so(tmp_path: Path) -> None:
    import time

    from sim_support import ManualClock

    clock = ManualClock()
    stack, transport = muted_stack(tmp_path, clock, ping_interval_s=10.0, reply_timeout_s=2.0)
    try:
        assert wait_for(lambda: count_pings(stack) == 1)
        transport.muted = True
        clock.advance(10.0)  # the next PING goes out and is never answered
        time.sleep(0.3)
        assert stack.alarm_requests == []  # the reply timeout has not elapsed on the injected clock
        clock.advance(2.5)
        assert wait_for(lambda: stack.alarm_requests == [AlarmCode.HB_TIMEOUT.value])
        clock.advance(30.0)
        time.sleep(0.3)
        assert stack.alarm_requests == [AlarmCode.HB_TIMEOUT.value]  # once per episode
    finally:
        stack.close()
