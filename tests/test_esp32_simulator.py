"""The ESP32 simulator: exact protocol, both joints, Z with timing, the motion queue, homing,
heartbeat and every injectable fault (SPEC 5 and 10). Deterministic: a manual clock, no threads,
except the few tests that say otherwise."""

from __future__ import annotations

import threading
import time

import pytest

from config.protocol_commands import COMMAND_ORDER, COMMANDS, PROTOCOL_VERSION, AlarmCode, ErrorCode
from desktop_app.hardware.esp32_simulator import Esp32Simulator, SimConfig
from desktop_app.hardware.serial_link import ConnectionLost
from desktop_app.utils.hardware_config import load_hardware_config
from sim_support import START_JOINTS, Bench, fast_sim_config, sim_config, threaded_simulator, wait_for

MOVE_10 = "MOVE A=160 B=40 F=20"  # 10 degrees on each joint at 20 deg/s = 0.5 s


# --------------------------------------------------------------------------- #
# Configuration
# --------------------------------------------------------------------------- #


def test_config_takes_derived_values_from_hardware_yaml() -> None:
    hardware = load_hardware_config()
    config = sim_config()
    assert config.pulses_per_rev == hardware.drive.pulses_per_rev
    assert config.z_timeout_factor == hardware.electrical.z_axis.timeout_factor
    assert config.z_timeout_s == pytest.approx(config.z_timeout_factor * config.z_travel_s)


def test_nothing_that_is_to_confirm_has_a_default() -> None:
    for missing in ("queue_capacity", "z_travel_s", "heartbeat_timeout_s", "home_joints_s", "initial_joint_deg"):
        kwargs = dict(queue_capacity=2, z_travel_s=1.0, heartbeat_timeout_s=1.0, home_joints_s=0.1, initial_joint_deg=(0.0, 0.0))
        del kwargs[missing]
        with pytest.raises(TypeError):
            SimConfig.from_hardware(**kwargs)


@pytest.mark.parametrize(
    "override",
    [
        {"queue_capacity": 0},
        {"z_travel_s": 0},
        {"heartbeat_timeout_s": 0},
        {"home_joints_s": -1},
        {"poll_s": 0},
        {"z_timeout_factor": 0.9},
        {"initial_z": "MID"},
        {"firmware_version": "two words"},
    ],
)
def test_config_is_validated(override: dict) -> None:
    with pytest.raises((ValueError, TypeError, AttributeError)):
        sim_config(**override)


# --------------------------------------------------------------------------- #
# Basic exchange
# --------------------------------------------------------------------------- #


def test_info_reports_the_declared_protocol_version() -> None:
    bench = Bench()
    assert bench.send("INFO") == [f"INFO FW=sim PROTO={PROTOCOL_VERSION}", "ok"]


def test_ping_is_acknowledged() -> None:
    assert Bench().send("PING") == ["ok"]


def test_pos_at_power_up() -> None:
    bench = Bench()
    assert bench.pos() == {"A": "150", "B": "30", "SA": "150", "SB": "30", "NA": "0", "NB": "0", "Z": "TOP"}


def test_blank_lines_get_no_answer() -> None:
    bench = Bench()
    assert bench.send("", "   ") == []
    assert bench.sim.received == []


def test_every_line_is_answered_in_order() -> None:
    bench = Bench()
    assert bench.send("PING", "INFO", "PING") == ["ok", f"INFO FW=sim PROTO={PROTOCOL_VERSION}", "ok", "ok"]


@pytest.mark.parametrize("line", ["FOO", "move A=1 B=2 F=3", "MOVEA=1", "ok", "\x01\x02"])
def test_error_1_for_an_unrecognized_command(line: str) -> None:
    assert Bench().send(line) == ["error:1"]


@pytest.mark.parametrize(
    "line",
    ["MOVE", "MOVE A=1 B=2", "MOVE A=1 B=2 F=0", "MOVE A=x B=2 F=3", "GRIP S=181", "GRIP S=-1", "ACCEL S=0", "PING NOW", "ZUP Z", "HOME Z=1"],
)
def test_error_2_for_a_missing_or_invalid_parameter(line: str) -> None:
    assert Bench().send(line) == ["error:2"]


def test_a_garbage_line_does_not_stop_the_next_one() -> None:
    bench = Bench()
    assert bench.send("\xff\xfe", "PING") == ["error:1", "ok"]  # non-ASCII bytes are an unrecognized command


def test_all_six_error_codes_can_be_produced() -> None:
    bench = Bench(queue_capacity=1)
    seen = {ErrorCode.UNRECOGNIZED: bench.send("FOO"), ErrorCode.BAD_PARAMETER: bench.send("GRIP"), ErrorCode.NOT_HOMED: bench.send(MOVE_10)}
    bench.home()
    assert bench.send(MOVE_10) == ["ok"]
    seen[ErrorCode.QUEUE_FULL] = bench.send(MOVE_10)
    bench.send("DISABLE")
    seen[ErrorCode.DRIVERS_DISABLED] = bench.send(MOVE_10)
    bench.send("ESTOP")
    seen[ErrorCode.IN_ESTOP] = bench.send("ZUP")
    assert {code: lines for code, lines in seen.items()} == {code: [f"error:{int(code)}"] for code in ErrorCode}


# --------------------------------------------------------------------------- #
# Homing
# --------------------------------------------------------------------------- #


def test_move_is_rejected_with_error_4_until_homed() -> None:
    bench = Bench()
    assert not bench.sim.homed
    assert bench.send(MOVE_10) == ["error:4"]
    assert bench.sim.queue_length == 0


def test_home_with_no_argument_raises_z_first_then_homes_the_joints() -> None:
    bench = Bench(initial_z="BOTTOM")
    assert bench.send("HOME") == ["ok"]
    assert bench.send(MOVE_10) == ["error:4"]  # still homing
    assert bench.advance(0.99) == []  # Z travel is 1 s
    assert bench.sim.z_state.value == "MID"
    assert bench.advance(0.01) == []  # Z reached TOP: now the joints (0.5 s)
    assert bench.sim.z_state.value == "TOP" and not bench.sim.homed
    assert bench.advance(0.49) == []
    assert bench.advance(0.01) == ["done"]
    assert bench.sim.homed
    assert bench.send(MOVE_10) == ["ok"]


def test_home_with_z_already_at_top_only_takes_the_joint_time() -> None:
    bench = Bench()
    assert bench.send("HOME") == ["ok"]
    assert bench.advance(0.49) == [] and bench.advance(0.01) == ["done"]


def test_home_z_only_does_not_home_the_joints() -> None:
    bench = Bench(initial_z="BOTTOM")
    assert bench.send("HOME Z") == ["ok"]
    assert bench.advance(1.0) == ["done"]
    assert bench.sim.z_state.value == "TOP" and not bench.sim.homed
    assert bench.send(MOVE_10) == ["error:4"]


def test_home_ab_only_needs_z_at_top() -> None:
    bench = Bench(initial_z="BOTTOM")
    assert bench.send("HOME AB") == ["error:4"]
    bench = Bench()
    assert bench.send("HOME AB") == ["ok"]
    assert bench.advance(0.5) == ["done"]


def test_home_z_and_ab_flags_are_the_same_as_no_flag() -> None:
    bench = Bench(initial_z="BOTTOM")
    assert bench.send("HOME Z AB") == ["ok"]
    assert bench.advance(1.5) == ["done"]
    assert bench.sim.homed


def test_move_needs_z_at_top() -> None:
    bench = Bench()
    bench.home()
    assert bench.send("ZDOWN") == ["ok"]
    assert bench.send(MOVE_10) == ["error:4"]  # Z is moving
    assert bench.advance(1.0) == ["done"]
    assert bench.send(MOVE_10) == ["error:4"]  # Z at BOTTOM
    assert bench.send("ZUP") == ["ok"] and bench.advance(1.0) == ["done"]
    assert bench.send(MOVE_10) == ["ok"]


def test_home_stops_the_queue_and_owed_waits() -> None:
    bench = Bench()
    bench.home()
    bench.send(MOVE_10, "WAIT")
    assert bench.send("HOME") == ["ok"]
    assert bench.sim.queue_length == 0
    assert bench.advance(5.0) == ["done"]  # one done: the HOME's. The WAIT was dropped


# --------------------------------------------------------------------------- #
# Joints and the motion queue
# --------------------------------------------------------------------------- #


def test_a_move_takes_the_largest_joint_travel_at_the_commanded_speed() -> None:
    bench = Bench()
    bench.home()
    assert bench.send("MOVE A=160 B=40 F=20", "WAIT") == ["ok", "ok"]
    assert bench.advance(0.49) == []
    assert bench.advance(0.01) == ["done"]
    assert bench.sim.joint_deg == (160.0, 40.0)


def test_both_joints_finish_together() -> None:
    bench = Bench()
    bench.home()
    bench.send("MOVE A=170 B=40 F=20")  # alpha travels 20, beta 10: the move takes 1 s for both
    bench.advance(0.5)
    a, b = bench.sim.joint_deg
    assert (a, b) == (pytest.approx(160.0), pytest.approx(35.0))  # halfway for both
    bench.advance(0.5)
    assert bench.sim.joint_deg == (170.0, 40.0)


def test_pos_follows_the_move_and_counts_steps_from_the_power_up_angle() -> None:
    bench = Bench()
    bench.home()
    bench.send("MOVE A=180 B=30 F=30")  # 1 s
    bench.advance(0.5)
    pos = bench.pos()
    assert float(pos["A"]) == pytest.approx(165.0) and float(pos["SA"]) == pytest.approx(165.0)
    ppr = bench.config.pulses_per_rev
    assert int(pos["NA"]) == round(ppr * 15.0 / 360.0) and pos["NB"] == "0"
    bench.advance(0.5)
    assert int(bench.pos()["NA"]) == round(ppr * 30.0 / 360.0)


def test_queued_moves_run_back_to_back_without_gaps() -> None:
    bench = Bench()
    bench.home()
    bench.send("MOVE A=160 B=40 F=20", "MOVE A=150 B=30 F=20", "WAIT")
    assert bench.advance(0.99) == []
    assert bench.advance(0.01) == ["done"]  # 0.5 + 0.5, exactly
    assert bench.sim.joint_deg == START_JOINTS


def test_a_zero_length_move_completes_at_once() -> None:
    bench = Bench()
    bench.home()
    assert bench.send("MOVE A=150 B=30 F=20", "WAIT") == ["ok", "ok", "done"]


def test_wait_on_an_empty_queue_is_done_at_once() -> None:
    bench = Bench()
    bench.home()
    assert bench.send("WAIT") == ["ok", "done"]


def test_two_waits_each_get_their_own_done() -> None:
    bench = Bench()
    bench.home()
    assert bench.send(MOVE_10, "WAIT", "WAIT") == ["ok", "ok", "ok"]
    assert bench.advance(0.5) == ["done", "done"]


def test_queue_full_is_error_3_and_the_command_is_not_queued() -> None:
    bench = Bench(queue_capacity=3)
    bench.home()
    out, back = "MOVE A=160 B=40 F=20", "MOVE A=150 B=30 F=20"  # 0.5 s each, never zero-length
    assert bench.send(out, back, out) == ["ok", "ok", "ok"]
    assert bench.sim.queue_length == 3  # counts the one being executed
    assert bench.send(back) == ["error:3"]
    assert bench.sim.queue_length == 3
    bench.advance(0.5)  # the first finishes: room again
    assert bench.sim.queue_length == 2
    assert bench.send(back) == ["ok"]
    assert bench.send(out) == ["error:3"]
    bench.send("WAIT")
    # Exactly the four moves that were accepted ran (out, back, out, back); the rejected ones never did.
    assert bench.advance(10.0) == ["done"]
    assert bench.sim.joint_deg == START_JOINTS


def test_move_checks_run_in_the_documented_order() -> None:
    limits = ((100.0, 200.0), (0.0, 90.0))
    bench = Bench(joint_limits_deg=limits, queue_capacity=1)
    assert bench.send("MOVE A=300 B=40 F=20") == ["error:2"]  # out of limits beats "not homed"
    assert bench.send("MOVE A=160 B=40 F=20") == ["error:4"]
    bench.home()
    assert bench.send("MOVE A=160 B=100 F=20") == ["error:2"]
    assert bench.send("MOVE A=99.9 B=40 F=20") == ["error:2"]
    assert bench.send("MOVE A=200 B=0 F=20") == ["ok"]  # the limits are inclusive
    assert bench.send("MOVE A=160 B=40 F=20") == ["error:3"]
    # SPEC 5.2 puts error:2 (a parameter check, joint limits included) BEFORE the ESTOP check: an out-of-range value is
    # error:2 even in an ESTOP, like any other bad parameter (owner decision 1, phase 4a driver rules).
    bench.send("ESTOP")
    assert bench.send("MOVE A=300 B=40 F=20") == ["error:2"]
    assert bench.send("MOVE A=160 B=40 F=20") == ["error:5"]  # a valid MOVE in an ESTOP: error:5


def test_without_limits_no_joint_limit_is_invented() -> None:
    bench = Bench()
    bench.home()
    assert bench.send("MOVE A=-1000 B=1000 F=1000") == ["ok"]  # no limits configured: none are invented


def test_accel_grip_enable_disable_are_acknowledged_and_remembered() -> None:
    bench = Bench()
    bench.home()
    assert bench.send("ACCEL S=120") == ["ok"]
    assert bench.send("DISABLE") == ["ok"] and not bench.sim.drivers_enabled
    assert bench.send("ENABLE") == ["ok"] and bench.sim.drivers_enabled
    bench.send("ZDOWN")
    bench.advance(1.0)
    assert bench.send("GRIP S=45.5") == ["ok"] and bench.sim.grip_deg == 45.5
    assert bench.sim.violations == []


def test_drivers_are_enabled_at_power_up() -> None:
    assert Bench().sim.drivers_enabled  # the pins float at boot, so the motors hold (SPEC 10.2)


# --------------------------------------------------------------------------- #
# Z axis
# --------------------------------------------------------------------------- #


def test_zdown_and_zup_take_the_travel_time_and_pass_through_mid() -> None:
    bench = Bench()
    assert bench.send("ZDOWN") == ["ok"]
    assert bench.advance(0.4) == [] and bench.pos()["Z"] == "MID"
    assert bench.advance(0.59) == [] and bench.pos()["Z"] == "MID"
    assert bench.advance(0.01) == ["done"] and bench.pos()["Z"] == "BOTTOM"
    assert bench.send("ZUP") == ["ok"]
    assert bench.advance(0.5) == [] and bench.pos()["Z"] == "MID"
    assert bench.advance(0.5) == ["done"] and bench.pos()["Z"] == "TOP"


def test_zup_at_top_and_zdown_at_bottom_are_done_at_once() -> None:
    bench = Bench()
    assert bench.send("ZUP") == ["ok", "done"]
    bench.send("ZDOWN")
    bench.advance(1.0)
    assert bench.send("ZDOWN") == ["ok", "done"]


def test_zstop_brakes_z_at_once_and_no_done_follows() -> None:
    bench = Bench()
    bench.send("ZDOWN")
    bench.advance(0.5)
    assert bench.send("ZSTOP") == ["ok"]
    assert bench.advance(5.0) == []
    assert bench.pos()["Z"] == "MID"
    # Half the travel remains, so ZDOWN now takes half the time.
    assert bench.send("ZDOWN") == ["ok"]
    assert bench.advance(0.49) == [] and bench.advance(0.01) == ["done"]


def test_a_new_z_command_replaces_the_running_one() -> None:
    bench = Bench()
    bench.send("ZDOWN")
    bench.advance(0.5)
    assert bench.send("ZUP") == ["ok"]
    assert bench.advance(0.5) == ["done"]  # one done only: the ZUP's
    assert bench.pos()["Z"] == "TOP"


def test_a_z_move_while_the_joints_move_is_recorded_as_a_violation() -> None:
    bench = Bench()
    bench.home()
    bench.send(MOVE_10)
    bench.send("ZDOWN")
    assert bench.sim.violations == ["ZDOWN while the joints are moving"]


def test_grip_with_z_not_at_bottom_is_recorded_as_a_violation() -> None:
    bench = Bench()
    assert bench.send("GRIP S=10") == ["ok"]
    assert bench.sim.violations == ["GRIP with Z TOP"]


def test_z_timeout_alarm_fires_after_the_factor_times_the_travel_and_not_before() -> None:
    bench = Bench()
    bench.sim.inject_z_stall()
    assert bench.send("ZDOWN") == ["ok"]
    timeout = bench.config.z_timeout_s
    assert timeout == pytest.approx(1.5)
    assert bench.advance(timeout - 0.01) == []
    assert bench.pos()["Z"] == "TOP"  # a stalled Z does not reach the limit
    assert bench.advance(0.011) == ["!alarm:Z_TIMEOUT"]
    assert bench.sim.alarms == (AlarmCode.Z_TIMEOUT,)
    assert bench.advance(10.0) == []  # once, and no done


def test_a_stall_that_starts_mid_travel_stops_z_where_it_was() -> None:
    bench = Bench()
    bench.send("ZDOWN")
    bench.advance(0.4)
    bench.sim.inject_z_stall()
    assert bench.advance(1.0) == []
    assert bench.advance(0.2) == ["!alarm:Z_TIMEOUT"]
    assert bench.pos()["Z"] == "MID"


def test_a_z_timeout_during_homing_aborts_it() -> None:
    bench = Bench(initial_z="BOTTOM")
    bench.sim.inject_z_stall()
    assert bench.send("HOME") == ["ok"]
    assert bench.advance(1.5) == ["!alarm:Z_TIMEOUT"]
    assert bench.advance(10.0) == [] and not bench.sim.homed


def test_reset_clears_a_z_stall_and_z_can_be_homed_again() -> None:
    bench = Bench(initial_z="BOTTOM")
    bench.sim.inject_z_stall()
    bench.send("HOME")
    bench.advance(1.5)
    assert bench.send("RESET") == ["ok"]
    assert bench.sim.alarms == ()
    bench.home()


# --------------------------------------------------------------------------- #
# ESTOP and LIMIT_Z
# --------------------------------------------------------------------------- #


def test_the_estop_command_is_acknowledged_by_its_alarm_alone() -> None:
    bench = Bench()
    assert bench.send("ESTOP") == ["!alarm:ESTOP"]
    assert bench.sim.estop and bench.sim.alarms == (AlarmCode.ESTOP,)


def test_estop_stops_both_joints_and_z_keeps_the_drivers_and_forces_homing() -> None:
    bench = Bench()
    bench.home()
    bench.send("MOVE A=170 B=40 F=20")
    bench.advance(0.5)
    at = bench.sim.joint_deg
    bench.send("ESTOP")
    assert bench.advance(10.0) == []
    assert bench.sim.joint_deg == at and bench.sim.queue_length == 0
    assert bench.sim.drivers_enabled
    assert not bench.sim.homed


def test_estop_during_a_z_move_brakes_z_in_the_middle() -> None:
    bench = Bench()
    bench.send("ZDOWN")
    bench.advance(0.3)
    bench.send("ESTOP")
    assert bench.advance(5.0) == []
    assert bench.pos()["Z"] == "MID"


def test_estop_drops_the_dones_that_were_owed() -> None:
    bench = Bench()
    bench.home()
    bench.send(MOVE_10, "WAIT")
    bench.send("ESTOP")
    assert bench.advance(5.0) == []


@pytest.mark.parametrize(
    "line",
    ["HOME", "MOVE A=160 B=40 F=20", "ZUP", "ZDOWN", "ACCEL S=1", "GRIP S=1", "ENABLE", "WAIT"],
)
def test_in_estop_everything_but_the_safe_commands_is_error_5(line: str) -> None:
    bench = Bench()
    bench.send("ESTOP")
    assert bench.send(line) == ["error:5"]


# The owner's ESTOP rules (SPEC 5.1 / 10.2), typed out here and NOT taken from the declaration the simulator uses.
ACCEPTED_IN_ESTOP = {"ZSTOP", "DISABLE", "POS", "INFO", "PING", "ESTOP", "RESET"}


def _line_for(name: str) -> str:
    """A valid line for ``name``: every required parameter given the value 1."""
    return " ".join([name, *(f"{p.name}=1" for p in COMMANDS[name].params if p.required)])


@pytest.mark.parametrize("name", list(COMMAND_ORDER))
def test_in_estop_a_command_is_gated_exactly_by_the_owner_set(name: str) -> None:
    """Every command of the protocol, one by one, in a fresh ESTOP: error:5 if and only if it is outside the set."""
    bench = Bench()
    bench.send("ESTOP")
    reply = bench.send(_line_for(name))
    if name in ACCEPTED_IN_ESTOP:
        assert "error:5" not in reply, f"{name} must still work in an ESTOP, got {reply}"
        assert reply, f"{name} gave no answer"
    else:
        assert reply == ["error:5"], f"{name} must be refused in an ESTOP, got {reply}"


def test_the_estop_set_and_its_complement_cover_every_command() -> None:
    assert ACCEPTED_IN_ESTOP < set(COMMAND_ORDER)
    assert set(COMMAND_ORDER) - ACCEPTED_IN_ESTOP == {"HOME", "MOVE", "ZUP", "ZDOWN", "ACCEL", "GRIP", "ENABLE", "WAIT"}


def test_in_estop_the_accepted_commands_answer_as_usual() -> None:
    bench = Bench()
    bench.send("ESTOP")
    assert bench.send("PING") == ["ok"]
    assert bench.send("INFO") == [f"INFO FW=sim PROTO={PROTOCOL_VERSION}", "ok"]
    assert bench.send("ZSTOP") == ["ok"]
    assert bench.send("DISABLE") == ["ok"]
    assert bench.pos()["Z"] == "TOP"
    assert bench.send("ESTOP") == ["!alarm:ESTOP"]
    assert bench.send("RESET") == ["ok"]


def test_parse_errors_come_before_the_estop_check() -> None:
    bench = Bench()
    bench.send("ESTOP")
    assert bench.send("FOO") == ["error:1"]
    assert bench.send("MOVE A=1") == ["error:2"]
    assert bench.send("ENABLE now") == ["error:2"]  # a bad parameter on a refused command is still error:2


# --------------------------------------------------------------------------- #
# ENABLE / DISABLE and the ESTOP (owner decisions 1 to 3)
# --------------------------------------------------------------------------- #


def test_disable_is_accepted_in_estop_and_frees_the_drivers() -> None:
    bench = Bench()
    bench.send("ESTOP")
    assert bench.sim.drivers_enabled
    assert bench.send("DISABLE") == ["ok"]
    assert not bench.sim.drivers_enabled
    assert bench.sim.estop  # DISABLE does not leave the ESTOP


def test_enable_is_refused_in_estop_and_accepted_again_after_reset() -> None:
    bench = Bench()
    bench.send("DISABLE")
    bench.send("ESTOP")
    assert bench.send("ENABLE") == ["error:5"]
    assert not bench.sim.drivers_enabled  # the refused ENABLE changed nothing
    assert bench.send("RESET") == ["ok"]
    assert not bench.sim.estop
    assert bench.send("ENABLE") == ["ok"]
    assert bench.sim.drivers_enabled


def test_reset_leaves_the_drivers_as_they_are() -> None:
    bench = Bench()
    bench.send("DISABLE")
    bench.send("ESTOP")
    bench.send("RESET")
    assert not bench.sim.drivers_enabled
    bench2 = Bench()
    bench2.send("ESTOP")
    bench2.send("RESET")
    assert bench2.sim.drivers_enabled


def test_disable_clears_the_homed_flag_and_enable_does_not_restore_it() -> None:
    bench = Bench()
    bench.home()
    assert bench.send(MOVE_10) == ["ok"]  # homed: a MOVE is accepted
    bench.advance(5.0)
    assert bench.send("DISABLE") == ["ok"]
    assert not bench.sim.homed
    assert bench.send(MOVE_10) == ["error:6"]  # the drivers are off: error:6 comes before error:4
    assert bench.send("ENABLE") == ["ok"]
    assert not bench.sim.homed
    assert bench.send(MOVE_10) == ["error:4"]  # ENABLE does not give the homing back
    bench.home()
    assert bench.send(MOVE_10) == ["ok"]  # only a new homing does


def test_disable_in_an_estop_leaves_homing_mandatory() -> None:
    bench = Bench()
    bench.home()
    bench.send("ESTOP")
    bench.send("DISABLE")
    bench.send("RESET")
    assert not bench.sim.homed
    assert bench.send("ENABLE") == ["ok"]
    assert bench.send(MOVE_10) == ["error:4"]


def test_disable_aborts_a_homing_that_still_needs_the_joints() -> None:
    """A homing running when the drivers go off must not set the homed flag when it ends, and sends no done."""
    # Z travel is 1.0 s and the joint phase 0.5 s (test values). From BOTTOM, 0.5 s is Z still rising and 1.2 s is
    # the joint phase running; from TOP the Z phase is instant, so 0.2 s is the joint phase too.
    for initial_z, advance_s in (("BOTTOM", 0.5), ("BOTTOM", 1.2), ("TOP", 0.2)):
        bench = Bench(initial_z=initial_z)
        assert bench.send("HOME") == ["ok"]
        assert bench.advance(advance_s) == []
        assert bench.send("DISABLE") == ["ok"]
        assert bench.advance(10.0) == [], f"an aborted homing must never send done ({initial_z}, {advance_s} s)"
        assert not bench.sim.homed
        assert bench.send(MOVE_10) == ["error:6"]  # drivers disabled (and not homed): error:6 first


def test_disable_does_not_stop_a_z_only_homing_because_z_needs_no_driver() -> None:
    bench = Bench(initial_z="BOTTOM")
    assert bench.send("HOME Z") == ["ok"]
    bench.advance(0.5)
    assert bench.send("DISABLE") == ["ok"]
    assert bench.advance(5.0) == ["done"]
    assert bench.pos()["Z"] == "TOP"
    assert not bench.sim.homed  # the joints were never homed, and DISABLE cleared the flag


def test_a_repeated_estop_is_accepted_and_harmless() -> None:
    bench = Bench()
    bench.home()
    bench.send("MOVE A=170 B=40 F=20")
    bench.advance(0.5)
    assert bench.send("ESTOP") == ["!alarm:ESTOP"]

    def snapshot() -> tuple:
        return (bench.sim.estop, bench.sim.alarms, bench.sim.homed, bench.sim.joint_deg, bench.sim.queue_length,
                bench.sim.drivers_enabled, bench.sim.z_state)

    first = snapshot()
    assert bench.send("ESTOP") == ["!alarm:ESTOP"]  # accepted, answered like the first one
    assert snapshot() == first
    assert bench.advance(10.0) == []
    assert bench.send("DISABLE") == ["ok"]
    after_disable = snapshot()
    assert bench.send("ESTOP") == ["!alarm:ESTOP"]
    assert snapshot() == after_disable  # a repeated ESTOP never re-enables the drivers
    assert not bench.sim.drivers_enabled
    assert bench.send("RESET") == ["ok"]  # one RESET is enough
    assert not bench.sim.estop and bench.sim.alarms == ()


def test_in_estop_the_safe_commands_still_work() -> None:
    bench = Bench()
    bench.send("ESTOP")
    assert bench.send("PING") == ["ok"]
    assert bench.send("ZSTOP") == ["ok"]
    assert bench.send("INFO")[-1] == "ok"
    assert bench.pos()["Z"] == "TOP"
    assert bench.send("ESTOP") == ["!alarm:ESTOP"]  # idempotent


def test_reset_clears_estop_and_makes_homing_mandatory() -> None:
    bench = Bench()
    bench.home()
    bench.send("ESTOP")
    assert bench.send("RESET") == ["ok"]
    assert not bench.sim.estop and bench.sim.alarms == ()
    assert bench.send(MOVE_10) == ["error:4"]
    bench.home()
    assert bench.send(MOVE_10) == ["ok"]


def test_a_hardware_estop_button_sends_an_unsolicited_alarm() -> None:
    bench = Bench()
    bench.home()
    bench.send(MOVE_10)
    bench.sim.inject_estop()
    assert bench.lines() == ["!alarm:ESTOP"]
    assert bench.send(MOVE_10) == ["error:5"]


def test_limit_z_is_an_emergency_stop_that_answers_with_its_own_alarm() -> None:
    bench = Bench()
    bench.home()
    bench.send(MOVE_10)
    bench.sim.inject_limit_z()
    assert bench.lines() == ["!alarm:LIMIT_Z"]
    assert bench.sim.estop and bench.sim.alarms == (AlarmCode.LIMIT_Z,)
    assert bench.send(MOVE_10) == ["error:5"]
    assert bench.sim.queue_length == 0 and not bench.sim.homed
    assert bench.send("RESET") == ["ok"]
    bench.home()


# --------------------------------------------------------------------------- #
# SENSOR_FAULT and STEP_LOSS
# --------------------------------------------------------------------------- #


def test_sensor_fault_stops_the_joints_and_forces_homing_until_reset() -> None:
    bench = Bench()
    bench.home()
    bench.send(MOVE_10, "WAIT")
    bench.sim.inject_sensor_fault()
    assert bench.lines() == ["!alarm:SENSOR_FAULT"]
    assert bench.sim.queue_length == 0 and not bench.sim.homed and not bench.sim.estop
    assert bench.send(MOVE_10) == ["error:4"]
    assert bench.send("HOME") == ["error:4"]  # the sensor is still faulty
    assert bench.send("RESET") == ["ok"]
    bench.home()
    assert bench.send(MOVE_10) == ["ok"]


def test_step_loss_shows_in_the_sensor_angle_and_forces_homing() -> None:
    bench = Bench()
    bench.home()
    bench.sim.inject_step_loss(3.5)
    assert bench.lines() == ["!alarm:STEP_LOSS"]
    pos = bench.pos()
    assert float(pos["SA"]) - float(pos["A"]) == pytest.approx(3.5)
    assert not bench.sim.homed
    bench.send("RESET")
    assert float(bench.pos()["SA"]) == pytest.approx(float(bench.pos()["A"]))


# --------------------------------------------------------------------------- #
# Heartbeat
# --------------------------------------------------------------------------- #


def test_the_heartbeat_is_not_armed_until_the_first_ping() -> None:
    bench = Bench()
    assert not bench.sim.heartbeat_armed
    assert bench.advance(60.0) == []
    bench.send("PING")
    assert bench.sim.heartbeat_armed


def test_regular_pings_keep_the_arm_alive() -> None:
    bench = Bench()
    bench.send("PING")
    for _ in range(20):
        assert bench.advance(1.9) == []
        assert bench.send("PING") == ["ok"]
    assert bench.sim.alarms == ()


def test_the_heartbeat_threshold_is_exclusive() -> None:
    bench = Bench()
    bench.send("PING")
    assert bench.advance(2.0) == []  # exactly the threshold: not yet
    assert bench.advance(0.001) == ["!alarm:HB_TIMEOUT"]


def test_heartbeat_timeout_stops_the_arm_but_keeps_the_drivers_enabled() -> None:
    bench = Bench()
    bench.home()
    bench.send("PING", "ZDOWN")
    bench.advance(0.4)
    bench.send("ZUP")
    bench.advance(0.4)
    bench.send("PING")
    bench.advance(0.3)  # Z is now at TOP again; start a joint move
    bench.send("ZUP")
    bench.send("MOVE A=170 B=40 F=5", "WAIT")  # 4 s: still running when the heartbeat fails
    assert bench.advance(2.001) == ["!alarm:HB_TIMEOUT"]
    at = bench.sim.joint_deg
    assert bench.advance(10.0) == [] and bench.sim.joint_deg == at
    assert bench.sim.queue_length == 0
    assert bench.sim.drivers_enabled  # releasing the motors could collapse the arm
    assert not bench.sim.homed and not bench.sim.estop
    assert bench.sim.alarms == (AlarmCode.HB_TIMEOUT,)


def test_heartbeat_timeout_brakes_z_mid_travel() -> None:
    bench = Bench(heartbeat_timeout_s=0.3)
    bench.send("PING", "ZDOWN")
    assert bench.advance(0.31) == ["!alarm:HB_TIMEOUT"]
    assert bench.pos()["Z"] == "MID"
    assert bench.advance(5.0) == []


def test_after_a_heartbeat_timeout_pings_resume_and_homing_is_required() -> None:
    bench = Bench()
    bench.home()
    bench.send("PING")
    assert bench.advance(2.5) == ["!alarm:HB_TIMEOUT"]
    assert bench.send("PING") == ["ok"] and bench.sim.heartbeat_armed
    assert bench.send(MOVE_10) == ["error:4"]
    bench.home()
    assert bench.send(MOVE_10) == ["ok"]


def test_the_heartbeat_alarm_is_sent_once() -> None:
    bench = Bench()
    bench.send("PING")
    assert bench.advance(3.0) == ["!alarm:HB_TIMEOUT"]
    assert bench.advance(30.0) == []


# --------------------------------------------------------------------------- #
# Lost connection
# --------------------------------------------------------------------------- #


def test_a_lost_connection_stops_the_arm_by_the_heartbeat_and_the_pc_never_hears_of_it() -> None:
    bench = Bench()
    bench.home()
    bench.send("PING")
    bench.sim.inject_lost_connection()
    with pytest.raises(ConnectionLost):
        bench.pair.pc.write(b"PING\n")
    bench.clock.advance(2.5)
    bench.sim.pump()
    assert not bench.sim.link_up
    assert bench.sim.alarms == (AlarmCode.HB_TIMEOUT,) and not bench.sim.homed
    assert bench.sim.sent[-1] == "!alarm:HB_TIMEOUT"  # sent into the void


def test_the_simulator_reopens_its_end_when_the_cable_is_back() -> None:
    bench = Bench()
    bench.sim.inject_lost_connection()
    bench.sim.pump()
    assert not bench.sim.link_up
    bench.sim.restore_connection()
    bench.sim.pump()
    assert bench.sim.link_up
    bench.pair.pc.open()
    assert bench.send("PING") == ["ok"]


def test_a_simulator_built_without_a_cable_cannot_cut_one() -> None:
    from desktop_app.hardware.serial_link import loopback_pair

    pair = loopback_pair()
    sim = Esp32Simulator(sim_config(), pair.device)
    with pytest.raises(RuntimeError, match="cable"):
        sim.inject_lost_connection()
    with pytest.raises(RuntimeError, match="cable"):
        sim.restore_connection()


# --------------------------------------------------------------------------- #
# Threaded serving on the real clock
# --------------------------------------------------------------------------- #


def read_lines(pair, count: int, timeout_s: float = 2.0) -> list[str]:
    out: list[str] = []
    buffer = b""
    deadline = time.monotonic() + timeout_s
    while len(out) < count and time.monotonic() < deadline:
        buffer += pair.pc.read(0.01)
        while b"\n" in buffer:
            line, buffer = buffer.split(b"\n", 1)
            out.append(line.decode())
    return out


def test_threaded_replies_arrive_in_protocol_order() -> None:
    sim, pair = threaded_simulator()
    try:
        pair.pc.open()
        pair.pc.write(b"PING\nHOME\n")
        assert read_lines(pair, 3) == ["ok", "ok", "done"]
        pair.pc.write(b"MOVE A=160 B=40 F=2000\nWAIT\n")
        assert read_lines(pair, 3) == ["ok", "ok", "done"]
        pair.pc.write(b"ZDOWN\n")
        assert read_lines(pair, 2) == ["ok", "done"]
        assert sim.violations == []
    finally:
        sim.stop()


def test_the_thread_fires_the_heartbeat_timeout_by_itself() -> None:
    sim, pair = threaded_simulator(heartbeat_timeout_s=0.05)
    try:
        pair.pc.open()
        pair.pc.write(b"PING\n")
        assert read_lines(pair, 1) == ["ok"]
        assert read_lines(pair, 1, timeout_s=1.0) == ["!alarm:HB_TIMEOUT"]
    finally:
        sim.stop()


def test_the_thread_survives_a_cable_cut_and_serves_again_after_the_cable_is_restored() -> None:
    sim, pair = threaded_simulator(heartbeat_timeout_s=0.05)
    try:
        pair.pc.open()
        pair.pc.write(b"PING\n")
        assert read_lines(pair, 1) == ["ok"]
        sim.inject_lost_connection()
        assert wait_for(lambda: not sim.link_up)
        assert wait_for(lambda: AlarmCode.HB_TIMEOUT in sim.alarms)
        sim.restore_connection()
        assert wait_for(lambda: sim.link_up)
        pair.pc.open()
        pair.pc.write(b"PING\n")
        assert read_lines(pair, 1) == ["ok"]
    finally:
        sim.stop()


def test_stop_is_prompt_and_idempotent_and_start_twice_is_refused() -> None:
    sim, _pair = threaded_simulator()
    with pytest.raises(RuntimeError, match="already running"):
        sim.start()
    started = time.monotonic()
    sim.stop()
    sim.stop()
    assert time.monotonic() - started < 2.0


def test_the_simulator_works_as_a_context_manager() -> None:
    from desktop_app.hardware.esp32_simulator import make_loopback_simulator

    sim, pair = make_loopback_simulator(fast_sim_config())
    with sim:
        pair.pc.open()
        pair.pc.write(b"PING\n")
        assert read_lines(pair, 1) == ["ok"]
    assert not sim.link_up


def test_injection_from_another_thread_is_safe_while_the_simulator_serves() -> None:
    sim, pair = threaded_simulator()
    try:
        pair.pc.open()
        pair.pc.write(b"HOME\n")
        assert read_lines(pair, 2) == ["ok", "done"]
        errors: list[BaseException] = []

        def hammer() -> None:
            try:
                for _ in range(50):
                    pair.pc.write(b"POS\n")
                    _ = sim.joint_deg, sim.queue_length, sim.z_state
            except BaseException as exc:  # noqa: BLE001 - the test reports whatever happened
                errors.append(exc)

        thread = threading.Thread(target=hammer)
        thread.start()
        sim.inject_sensor_fault()
        thread.join(timeout=5)
        assert errors == []
    finally:
        sim.stop()
