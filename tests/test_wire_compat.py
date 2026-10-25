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


@pytest.mark.skipif(_compiler() is None, reason="no C++ compiler (g++, clang++, c++ or $CXX): run firmware/tools/run_host_tests.sh elsewhere")
def test_the_cpp_core_the_simulator_and_the_model_agree() -> None:
    compiler = _compiler()
    assert compiler is not None
    report = wc.run(compiler, wc.DEFAULT_FIRMWARE_DIR, max_length=4)
    assert report.scripts > 7000
    assert report.mismatches == [] and report.gate_problems == []
