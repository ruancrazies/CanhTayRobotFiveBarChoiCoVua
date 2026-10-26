"""The wire-compat tool (firmware/tools/wire_compat/wire_compat.py) and what it enforces (SPEC 5.1, phase 4a ESTOP rules).

* The full run (C++ core vs Python simulator vs an independent model) needs a C++ compiler; without one that
  test is skipped and the skip reason says so (run firmware/tools/run_host_tests.sh elsewhere).
* The comparison itself must be able to FAIL: the tests below feed it transcripts of the OLD behaviour (ENABLE
  accepted in an ESTOP, DISABLE refused in one) and require a complaint.
"""

from __future__ import annotations

import importlib.util
import os
import shutil
import sys
from pathlib import Path
from types import ModuleType

import pytest

REPO_ROOT = Path(__file__).resolve().parents[1]
TOOL = REPO_ROOT / "firmware" / "tools" / "wire_compat" / "wire_compat.py"


def _load() -> ModuleType:
    spec = importlib.util.spec_from_file_location("wire_compat_tool", TOOL)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


wc = _load()
VERSION = "4a-test"


def _compiler() -> str | None:
    for candidate in (os.environ.get("CXX"), "g++", "clang++", "c++"):
        if candidate and shutil.which(candidate):
            return candidate
    return None


def test_the_owner_set_in_the_tool_is_the_declared_one() -> None:
    from config.protocol_commands import ACCEPTED_IN_ESTOP

    assert wc.OWNER_ESTOP_SET == frozenset(ACCEPTED_IN_ESTOP)  # typed out twice on purpose: either edit alone fails
    assert wc.OWNER_ESTOP_SET == {"POS", "INFO", "PING", "ZSTOP", "ESTOP", "RESET", "DISABLE"}


def test_the_model_follows_the_owner_decisions() -> None:
    steps = wc.model_transcript(["ESTOP", "ENABLE", "DISABLE", "ESTOP", "ENABLE", "RESET", "ENABLE"], VERSION)
    assert [s.replies for s in steps] == [
        ("!alarm:ESTOP",), ("error:5",), ("ok",), ("!alarm:ESTOP",), ("error:5",), ("ok",), ("ok",),
    ]
    assert [s.drivers for s in steps] == [True, True, False, False, False, False, True]
    assert [s.estop for s in steps] == [True, True, True, True, True, False, False]


def test_the_comparison_fails_on_the_old_behaviour() -> None:
    script = ["ESTOP", "ENABLE", "DISABLE"]
    model = wc.model_transcript(script, VERSION)
    old = [
        wc.Step("ESTOP", ("!alarm:ESTOP",), True, True, False),
        wc.Step("ENABLE", ("error:5",), True, True, False),
        wc.Step("DISABLE", ("error:5",), True, True, False),  # old: DISABLE refused in an ESTOP
    ]
    assert wc.compare_shared(script, model, model, model) is None
    assert wc.compare_shared(script, model, old, model) is not None
    assert wc.compare_shared(script, model, model, old) is not None
    old_enable = [*old[:1], wc.Step("ENABLE", ("ok",), True, True, False), model[2]]  # old: ENABLE accepted
    assert wc.compare_shared(script, model, old_enable, old_enable) is not None  # both wrong the same way


def test_the_gate_check_fails_on_the_old_behaviour() -> None:
    def last(reply: str) -> list:
        return [wc.Step("ESTOP", ("!alarm:ESTOP",), True, True, False), wc.Step("X", (reply,), True, True, False)]

    assert wc.check_gate("ENABLE", last("error:5"), last("error:5")) == []
    assert wc.check_gate("ENABLE", last("ok"), last("ok"))  # ENABLE accepted in an ESTOP
    assert wc.check_gate("DISABLE", last("ok"), last("ok")) == []
    assert wc.check_gate("DISABLE", last("error:5"), last("error:5"))  # DISABLE refused in an ESTOP
    assert wc.check_gate("ZSTOP", last("error:1"), last("ok")) == []  # a 4a stub in the firmware
    assert wc.check_gate("ZSTOP", last("ok"), last("ok"))  # the stub list is out of date: say so


def test_the_check_order_in_the_tool_is_the_declared_one() -> None:
    from config.protocol_commands import CHECK_ORDER

    assert wc.OWNER_CHECK_ORDER == tuple(int(code) for code in CHECK_ORDER)  # typed twice on purpose
    assert wc.OWNER_CHECK_ORDER == (1, 2, 5, 6, 4, 3)


def _oracle(line: str, drivers: bool, estop: bool, homed: bool, full: bool) -> str | None:
    """The fixed order written out a second time, as plain ifs, for MOVE and HOME lines. None = passes every gate."""
    is_move = line.startswith("MOVE")
    joints = is_move or line in ("HOME", "HOME AB", "HOME Z AB")
    if estop:
        return "error:5"
    if joints and not drivers:
        return "error:6"
    if is_move and not homed:
        return "error:4"
    if is_move and full:
        return "error:3"
    return None


def test_the_model_gate_follows_the_fixed_order_for_every_combination() -> None:
    for line in sorted(wc.GATED_LINES):
        for bits in range(16):
            drivers, estop, homed, full = (bool(bits & 1), bool(bits & 2), bool(bits & 4), bool(bits & 8))
            got = wc.model_gate_reply(line.split(), drivers, estop, homed, full)
            assert got == _oracle(line, drivers, estop, homed, full), (line, drivers, estop, homed, full)


def test_home_z_is_the_only_home_that_needs_no_joint_driver() -> None:
    assert [wc.needs_joint_drivers(line.split()) for line in ("HOME", "HOME Z", "HOME AB", "HOME Z AB", "MOVE A=1 B=2 F=3")] == [
        True, False, True, True, True,
    ]


def test_the_model_transcript_answers_move_with_error_6_then_4() -> None:
    steps = wc.model_transcript(["MOVE A=1 B=2 F=3", "DISABLE", "MOVE A=1 B=2 F=3", "ENABLE", "MOVE A=1 B=2 F=3", "ESTOP", "MOVE A=1 B=2 F=3"], VERSION)
    assert [s.replies for s in steps] == [
        ("error:4",), ("ok",), ("error:6",), ("ok",), ("error:4",), ("!alarm:ESTOP",), ("error:5",),
    ]


def _gate_steps(prelude: tuple[str, ...], line: str, reply: tuple[str, ...], drivers: bool) -> list:
    return [wc.Step(l, ("ok",), drivers, False, False) for l in prelude] + [wc.Step(line, reply, drivers, False, False)]


def test_the_driver_gate_check_passes_the_new_behaviour_and_fails_the_old_one() -> None:
    prelude = ("DISABLE",)
    new_fw = _gate_steps(prelude, "MOVE A=1 B=2 F=3", ("error:6",), False)
    assert wc.check_driver_gate("disabled", "MOVE A=1 B=2 F=3", new_fw, new_fw) == []
    old = _gate_steps(prelude, "MOVE A=1 B=2 F=3", ("error:4",), False)  # old: not-homed was all it said
    assert wc.check_driver_gate("disabled", "MOVE A=1 B=2 F=3", old, new_fw)
    assert wc.check_driver_gate("disabled", "MOVE A=1 B=2 F=3", new_fw, old)
    # HOME with the drivers disabled: new = error:6 on both; old = the firmware stub (error:1) and a simulator ok
    assert wc.check_driver_gate("disabled", "HOME", _gate_steps(prelude, "HOME", ("error:6",), False), _gate_steps(prelude, "HOME", ("error:6",), False)) == []
    assert wc.check_driver_gate("disabled", "HOME", _gate_steps(prelude, "HOME", ("error:1",), False), _gate_steps(prelude, "HOME", ("ok",), False))
    # HOME Z with the drivers disabled must NOT be refused: stub error:1 and ok are right, error:6 is wrong
    assert wc.check_driver_gate("disabled", "HOME Z", _gate_steps(prelude, "HOME Z", ("error:1",), False), _gate_steps(prelude, "HOME Z", ("ok", "done"), False)) == []
    assert wc.check_driver_gate("disabled", "HOME Z", _gate_steps(prelude, "HOME Z", ("error:6",), False), _gate_steps(prelude, "HOME Z", ("ok",), False))
    # HOME that enables the drivers by itself (the drivers flag flips back to True): caught
    enabled_by_home = _gate_steps(prelude, "HOME Z", ("error:1",), True)
    assert wc.check_driver_gate("disabled", "HOME Z", enabled_by_home, _gate_steps(prelude, "HOME Z", ("ok",), False))


def test_every_driver_gate_script_is_built_and_compared() -> None:
    scripts, _gate, driver_gate = wc.build_scripts(1)
    assert len(driver_gate) == len(wc.GATE_PRELUDES) * len(wc.GATE_LINES)
    for (prelude_name, line), index in driver_gate.items():
        assert scripts[index] == (*wc.GATE_PRELUDES[prelude_name], line)
    assert {"HOME Z", "HOME AB", "HOME Z AB", "HOME", "MOVE A=1 B=2 F=3"} <= set(wc.GATE_LINES)


@pytest.mark.skipif(_compiler() is None, reason="no C++ compiler (g++, clang++, c++ or $CXX): run firmware/tools/run_host_tests.sh elsewhere")
def test_the_cpp_core_the_simulator_and_the_model_agree() -> None:
    compiler = _compiler()
    assert compiler is not None
    report = wc.run(compiler, wc.DEFAULT_FIRMWARE_DIR, max_length=4)
    assert report.scripts > 11000
    assert report.mismatches == [] and report.gate_problems == []
