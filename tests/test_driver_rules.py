"""Phase 4a driver rules (owner decisions 1 to 3) in the PC simulator, and the declared order of checks.

* error:6 (drivers disabled) for MOVE and for a HOME that moves the joints; ``HOME Z`` is not refused; HOME never
  enables the drivers.
* the ONE order of checks (SPEC 5.2): 1 unknown, 2 bad parameter, 5 ESTOP, 6 drivers disabled, 4 not homed, 3 queue full.
  The expected replies below are typed out here, not computed from the declaration the simulator walks.
* DISABLE stops the joints, clears the WHOLE queue (discarded, not drained), cancels a pending WAIT's done (the same
  rule as ESTOP, RESET, the heartbeat and a new HOME), aborts the joint part of a running HOME without a done, and lets
  a Z-only HOME and a running ZUP / ZDOWN carry on.

Every rule here is also run against the code of ``main`` (before this change) by the report of the phase; the ones
that were already true then are said so in the commit message.
"""

from __future__ import annotations

from typing import Callable

import pytest

from config import protocol_commands as declaration
from config.protocol_commands import CHECK_ORDER, ErrorCode
from desktop_app.hardware import esp32_simulator
from sim_support import Bench

MOVE_10 = "MOVE A=160 B=40 F=20"  # from START_JOINTS (150, 30): 0.5 s
MOVE_FAR = "MOVE A=250 B=130 F=20"  # 5 s: still running when a test looks


# --------------------------------------------------------------------------- #
# The declaration
# --------------------------------------------------------------------------- #


def test_error_6_is_the_drivers_disabled_code() -> None:
    assert int(ErrorCode.DRIVERS_DISABLED) == 6
    assert {int(code) for code in ErrorCode} == {1, 2, 3, 4, 5, 6}


def test_the_declared_check_order_is_the_owner_decision() -> None:
    # typed out on purpose: parse (1, 2), ESTOP (5), drivers disabled (6), not homed (4), queue full (3)
    assert [int(code) for code in CHECK_ORDER] == [1, 2, 5, 6, 4, 3]
    assert sorted(int(code) for code in CHECK_ORDER) == sorted(int(code) for code in ErrorCode)  # every code, once


def test_the_simulator_walks_the_declared_order_not_a_copy_of_it(monkeypatch: pytest.MonkeyPatch) -> None:
    """Swap two entries of CHECK_ORDER: the simulator's replies must change (it holds no order of its own)."""
    bench = Bench()
    bench.send("DISABLE")
    assert bench.send(MOVE_10) == ["error:6"]  # drivers disabled AND not homed: 6 comes first
    swapped = (
        ErrorCode.UNRECOGNIZED, ErrorCode.BAD_PARAMETER, ErrorCode.IN_ESTOP,
        ErrorCode.NOT_HOMED, ErrorCode.DRIVERS_DISABLED, ErrorCode.QUEUE_FULL,
    )
    monkeypatch.setattr(esp32_simulator, "CHECK_ORDER", swapped)
    assert bench.send(MOVE_10) == ["error:4"]


# --------------------------------------------------------------------------- #
# Decision 1: error:6
# --------------------------------------------------------------------------- #


def test_move_with_the_drivers_disabled_is_error_6_homed_or_not() -> None:
    bench = Bench()
    bench.home()
    assert bench.send("DISABLE") == ["ok"]
    assert bench.send(MOVE_10) == ["error:6"]
    assert bench.sim.queue_length == 0
    assert bench.send("ENABLE") == ["ok"]
    assert bench.send(MOVE_10) == ["error:4"]  # enabled again, but DISABLE cleared the homing


@pytest.mark.parametrize("line", ["HOME", "HOME AB", "HOME Z AB", "HOME AB Z"])
def test_a_home_that_moves_the_joints_is_error_6_while_the_drivers_are_disabled(line: str) -> None:
    bench = Bench(initial_z="BOTTOM")
    bench.send("DISABLE")
    assert bench.send(line) == ["error:6"]
    assert bench.advance(10.0) == []  # nothing started: no done, Z did not move
    assert bench.pos()["Z"] == "BOTTOM"
    assert not bench.sim.homed and not bench.sim.drivers_enabled


def test_home_z_works_with_the_drivers_disabled_and_does_not_enable_them() -> None:
    bench = Bench(initial_z="BOTTOM")
    bench.send("DISABLE")
    assert bench.send("HOME Z") == ["ok"]
    assert bench.advance(1.0) == ["done"]
    assert bench.pos()["Z"] == "TOP"
    assert not bench.sim.drivers_enabled  # HOME never enables the drivers
    assert not bench.sim.homed  # the joints were not homed


def test_home_never_enables_the_drivers_the_pc_must_send_enable_first() -> None:
    bench = Bench()
    bench.send("DISABLE")
    assert bench.send("HOME") == ["error:6"]
    assert not bench.sim.drivers_enabled
    assert bench.send("ENABLE") == ["ok"]
    assert bench.send("HOME") == ["ok"]
    assert bench.advance(5.0) == ["done"]
    assert bench.sim.homed and bench.sim.drivers_enabled


def test_enable_and_the_other_commands_are_not_gated_by_the_drivers() -> None:
    bench = Bench()
    bench.send("DISABLE")
    for line, reply in (("PING", ["ok"]), ("ACCEL S=5", ["ok"]), ("GRIP S=10", ["ok"]), ("WAIT", ["ok", "done"]),
                        ("ZDOWN", ["ok"]), ("ZSTOP", ["ok"]), ("ZUP", ["ok", "done"])):  # Z stopped at once: still at TOP
        assert bench.send(line) == reply, line  # Z and the servo need no joint driver


# --------------------------------------------------------------------------- #
# Decision 1: the order of checks
# --------------------------------------------------------------------------- #


def _disabled(bench: Bench) -> None:
    bench.send("DISABLE")


def _estop(bench: Bench) -> None:
    bench.send("ESTOP")


def _estop_and_disabled(bench: Bench) -> None:
    bench.send("ESTOP", "DISABLE")


def _homed_queue_full(bench: Bench) -> None:
    bench.home()
    assert bench.send(MOVE_10) == ["ok"]  # capacity 1


def _z_not_top_queue_full(bench: Bench) -> None:
    """A MOVE is queued and Z has been sent down anyway (a Commander never does this): not at TOP AND queue full."""
    _homed_queue_full(bench)
    bench.send("ZDOWN")


def _homed(bench: Bench) -> None:
    bench.home()


ORDER_CASES: list[tuple[str, Callable[[Bench], None], str, str]] = [
    # (what the state is, how to get there, line, expected reply)
    ("1 beats 5, 6", _estop_and_disabled, "FOO", "error:1"),
    ("2 beats 5, 6", _estop_and_disabled, "MOVE A=1", "error:2"),
    ("2 beats 6 (missing F)", _disabled, "MOVE A=1 B=2", "error:2"),
    ("5 beats 6 (MOVE)", _estop_and_disabled, MOVE_10, "error:5"),
    ("5 beats 6 (HOME)", _estop_and_disabled, "HOME", "error:5"),
    ("6 beats 4 (MOVE, not homed)", _disabled, MOVE_10, "error:6"),
    ("6 beats 4 (HOME AB, Z not at TOP)", lambda b: (b.send("ZDOWN"), b.advance(1.0), b.send("DISABLE")), "HOME AB", "error:6"),
    ("4 for MOVE when enabled and not homed", lambda b: None, MOVE_10, "error:4"),
    ("4 beats 3 (Z not at TOP, queue full)", _z_not_top_queue_full, MOVE_10, "error:4"),
    ("3 when nothing before it fails", _homed_queue_full, MOVE_10, "error:3"),
    ("ok when nothing fails", _homed, MOVE_10, "ok"),
]


@pytest.mark.parametrize("name, prepare, line, expected", ORDER_CASES, ids=[case[0] for case in ORDER_CASES])
def test_the_checks_run_in_the_documented_order(name: str, prepare: Callable[[Bench], None], line: str, expected: str) -> None:
    bench = Bench(queue_capacity=1)
    prepare(bench)
    bench.lines()  # drop what the preparation produced
    assert bench.send(line)[0] == expected, name


# --------------------------------------------------------------------------- #
# Decision 3: DISABLE clears the whole queue; what a pending WAIT does
# --------------------------------------------------------------------------- #


def test_disable_discards_the_queue_it_does_not_drain_it() -> None:
    bench = Bench(queue_capacity=4)
    bench.home()
    out, back = "MOVE A=160 B=40 F=20", "MOVE A=150 B=30 F=20"  # 0.5 s each
    assert bench.send(out, back, out) == ["ok", "ok", "ok"]
    bench.advance(0.25)  # the first move is half way
    assert bench.sim.queue_length == 3
    halfway = bench.sim.joint_deg
    assert bench.send("DISABLE") == ["ok"]
    assert bench.sim.queue_length == 0
    assert bench.sim.joint_deg == pytest.approx(halfway)  # the pulses stopped where the joints were
    bench.advance(10.0)
    assert bench.sim.joint_deg == pytest.approx(halfway)  # nothing of the queue ran afterwards
    assert float(bench.pos()["A"]) == pytest.approx(halfway[0], abs=1e-3)  # POS agrees: the arm did not go on


def test_disable_with_the_arm_idle_changes_no_position() -> None:
    bench = Bench()
    bench.home()
    before = bench.sim.joint_deg
    assert bench.send("DISABLE") == ["ok"]
    assert bench.sim.joint_deg == before and bench.sim.queue_length == 0


def test_a_pending_wait_loses_its_done_when_disable_clears_the_queue() -> None:
    bench = Bench()
    bench.home()
    assert bench.send(MOVE_FAR, "WAIT") == ["ok", "ok"]
    assert bench.send("DISABLE") == ["ok"]
    assert bench.advance(30.0) == []  # the WAIT's done is cancelled, not sent
    assert bench.send("WAIT") == ["ok", "done"]  # a WAIT sent after the clear finds an empty queue


def test_two_pending_waits_both_lose_their_done() -> None:
    bench = Bench()
    bench.home()
    assert bench.send(MOVE_FAR, "WAIT", "WAIT") == ["ok", "ok", "ok"]
    assert bench.send("DISABLE") == ["ok"]
    assert bench.advance(30.0) == []


def _clear_by_estop(bench: Bench) -> list[str]:
    return bench.send("ESTOP")


def _clear_by_reset(bench: Bench) -> list[str]:
    return bench.send("RESET")


def _clear_by_disable(bench: Bench) -> list[str]:
    return bench.send("DISABLE")


def _clear_by_a_new_home(bench: Bench) -> list[str]:
    return bench.send("HOME")


def _clear_by_heartbeat(bench: Bench) -> list[str]:
    return bench.advance(bench.config.heartbeat_timeout_s + 0.5)


CLEARING_EVENTS: list[tuple[str, Callable[[Bench], list[str]], list[str], list[str]]] = [
    # (event, how, what the event itself sends, what arrives later)
    ("DISABLE", _clear_by_disable, ["ok"], []),
    ("ESTOP", _clear_by_estop, ["!alarm:ESTOP"], []),
    ("RESET", _clear_by_reset, ["ok"], []),
    ("a new HOME", _clear_by_a_new_home, ["ok"], ["done"]),  # the done that arrives is the HOME's, not the WAIT's
    ("the heartbeat", _clear_by_heartbeat, ["!alarm:HB_TIMEOUT"], []),
]


@pytest.mark.parametrize("event, clear, sent, later", CLEARING_EVENTS, ids=[c[0] for c in CLEARING_EVENTS])
def test_every_way_of_clearing_the_queue_cancels_the_pending_wait_the_same_way(
    event: str, clear: Callable[[Bench], list[str]], sent: list[str], later: list[str]
) -> None:
    """One rule for DISABLE, ESTOP, RESET, a heartbeat timeout and a new HOME (SPEC 5.1, "Cancelled done")."""
    bench = Bench()
    bench.home()
    if event == "the heartbeat":
        bench.send("PING")  # arms the heartbeat; the other cases must not have it armed
    assert bench.send(MOVE_FAR, "WAIT") == ["ok", "ok"]
    assert clear(bench) == sent, event
    assert bench.sim.queue_length == 0, event
    assert bench.advance(30.0 if event != "the heartbeat" else 0.0) == later, event


# --------------------------------------------------------------------------- #
# Decision 2: DISABLE during a running HOME
# --------------------------------------------------------------------------- #


@pytest.mark.parametrize(
    "initial_z, line, advance_s",
    [
        ("BOTTOM", "HOME", 0.5),  # Z phase of a full HOME
        ("BOTTOM", "HOME", 1.2),  # joint phase of a full HOME
        ("TOP", "HOME", 0.2),  # Z already at TOP: joint phase only
        ("TOP", "HOME AB", 0.2),
        ("BOTTOM", "HOME Z AB", 1.2),
    ],
)
def test_disable_aborts_the_joint_part_of_a_running_home_with_no_done(initial_z: str, line: str, advance_s: float) -> None:
    bench = Bench(initial_z=initial_z)
    assert bench.send(line) == ["ok"]
    assert bench.advance(advance_s) == []
    assert bench.send("DISABLE") == ["ok"]
    assert bench.advance(30.0) == [], "an aborted homing never sends done"
    assert not bench.sim.homed and not bench.sim.drivers_enabled
    assert bench.send(MOVE_10) == ["error:6"]
    assert bench.send("ENABLE") == ["ok"]
    assert bench.send(MOVE_10) == ["error:4"]  # the joints were never homed


def test_a_z_only_home_carries_on_through_disable_and_sends_its_done() -> None:
    bench = Bench(initial_z="BOTTOM")
    assert bench.send("HOME Z") == ["ok"]
    bench.advance(0.5)
    assert bench.send("DISABLE") == ["ok"]
    assert bench.advance(5.0) == ["done"]
    assert bench.pos()["Z"] == "TOP"


@pytest.mark.parametrize("line, final", [("ZUP", "TOP"), ("ZDOWN", "BOTTOM")])
def test_a_running_z_move_carries_on_through_disable_and_sends_its_done(line: str, final: str) -> None:
    bench = Bench(initial_z="BOTTOM" if line == "ZUP" else "TOP")
    assert bench.send(line) == ["ok"]
    bench.advance(0.5)
    assert bench.send("DISABLE") == ["ok"]
    assert bench.advance(5.0) == ["done"]
    assert bench.pos()["Z"] == final


def test_disable_twice_is_harmless() -> None:
    bench = Bench()
    bench.home()
    bench.send(MOVE_FAR, "WAIT")
    assert bench.send("DISABLE") == ["ok"]
    assert bench.send("DISABLE") == ["ok"]
    assert bench.advance(30.0) == []
