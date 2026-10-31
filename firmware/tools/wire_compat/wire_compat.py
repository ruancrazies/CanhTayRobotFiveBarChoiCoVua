#!/usr/bin/env python3
"""Wire compatibility of the C++ core and the Python ESP32 simulator (SPEC 5.1, 5.2, 10.2).

The firmware core (firmware/lib/chessbot_core) and the simulator (desktop_app/hardware/esp32_simulator.py)
must answer the same lines the same way. This tool runs the same scripts through both and through a third
witness, a tiny model of the owner's rules written out below. Comparing the two implementations only with
each other would pass when both are wrong in the same way (both used to refuse DISABLE in an ESTOP), so
every transcript is also compared with the model.

What it compares, for every line of every script: the reply lines, in order, and the state after the line
(drivers enabled, ESTOP flag, homed flag).

Scripts:
* every sequence of up to ``--max-length`` lines (default 4) drawn from INFO, PING, ENABLE, DISABLE, ESTOP,
  RESET, a well-formed MOVE, an unknown command, a bad parameter and a blank line, each on a fresh device (all of
  the ESTOP and ENABLE / DISABLE state machine, and the order of checks 5, 6, 4 for a MOVE);
* the BRINGUP.md session of stage 4a;
* the ESTOP gate for ALL 15 commands: a fresh ESTOP, then the command;
* the DRIVER GATES: MOVE and every form of HOME (and the bad lines, among them a MOVE or an ACCEL that is above a
  joint limit or a speed / acceleration ceiling: check 2, answered before ESTOP) after each of several preludes (fresh,
  DISABLE, ESTOP, ESTOP + DISABLE, DISABLE + ENABLE). The reply must be the one the model derives from the fixed
  order of checks (SPEC 5.2: 1, 2, 5, 6, 4, 3) on the firmware core AND on the simulator; and HOME never changes
  the drivers (it never enables them).

The model's order is typed out here (OWNER_CHECK_ORDER) and compared with the declared CHECK_ORDER by a test.

* the BENCH scripts (stage 4b-1): the firmware core and the simulator both with motion.bench_move_without_homing
  on, so a MOVE is accepted without HOME. They cover MOVE / WAIT / POS, the queue-full answer, the done of a WAIT and
  its cancellation by DISABLE and ESTOP, and the order of checks 2, 5, 6, 3. The motion is faked: the firmware side
  finishes the joints on the pseudo-line ``@finish``, the simulator side advances its clock by that line. Each
  script has a typed-out expected transcript (the third witness for these scripts); the SA, SB and Z fields of POS
  are not compared (no AS5600 and no limit switch exists in the firmware before stages 4b-4 and 4b-2).

Known differences, asserted explicitly instead of ignored:
* A HOME that passes every gate: the firmware answers error:1 (the stub, nothing queued; HOME is stage 4b-3), the
  simulator really runs it (``ok``). The driver-gate scripts require exactly that pair of replies, so a HOME that is
  refused by mistake, or one that sneaks past a gate, is a failure. Without the bench setting a MOVE is always
  ``error:4`` on both sides (the homed flag cannot be set before HOME exists).
* ZSTOP is accepted in an ESTOP but is a stub in the firmware (error:1) until stage 4b-2, while the simulator
  implements it. For it the gate check requires "not error:5" on both sides, and error:1 from the firmware.
  When a stage implements it, update STUBS_IN_THE_FIRMWARE below.
* The homed flag is never set on the firmware's wire (HOME is a stub), so "DISABLE clears the homed flag" is not
  visible there yet. It is unit tested in firmware/test (test_state, test_processor) and in
  tests/test_esp32_simulator.py.

Usage:  python3 firmware/tools/wire_compat/wire_compat.py [--cxx g++] [--max-length 4] [--firmware-dir DIR]
Exit status: 0 when everything agrees, 1 on any mismatch, 2 when the replay program cannot be built.
It builds the replay program (replay.cpp) with a plain C++ compiler; it never compiles for the ESP32.
"""

from __future__ import annotations

import argparse
import itertools
import logging
import os
import shutil
import subprocess
import sys
import tempfile
from dataclasses import dataclass, field
from pathlib import Path
from typing import Optional, Sequence

REPO_ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(REPO_ROOT))

from config.protocol_commands import COMMAND_ORDER, COMMANDS, PROTOCOL_VERSION  # noqa: E402
from desktop_app.hardware.esp32_simulator import SimConfig, make_loopback_simulator  # noqa: E402
from desktop_app.hardware.serial_link import ConnectionLost  # noqa: E402
from desktop_app.utils.settings import load_settings  # noqa: E402

logger = logging.getLogger("wire_compat")

DEFAULT_FIRMWARE_DIR = REPO_ROOT / "firmware"
HERE = Path(__file__).resolve().parent

# The owner's decisions, typed out here on purpose (this is the third witness): while the ESTOP flag is set
# only these are accepted; every other command, ENABLE included, is error:5.
OWNER_ESTOP_SET = frozenset({"ZSTOP", "DISABLE", "POS", "INFO", "PING", "ESTOP", "RESET"})
# Commands in OWNER_ESTOP_SET that the stage 4a firmware parses but does not implement (it answers error:1).
STUBS_IN_THE_FIRMWARE = frozenset({"ZSTOP"})

# The order of checks of SPEC 5.2, as error numbers (owner decision 1 of the phase 4a driver rules): unknown command (1),
# bad parameter (2), ESTOP (5), drivers disabled (6), not homed (4), queue full (3). Typed out here on purpose; the
# model below applies it by hand, and tests/test_wire_compat.py compares it with CHECK_ORDER.
OWNER_CHECK_ORDER = (1, 2, 5, 6, 4, 3)

# A MOVE that is inside the soft limits (settings.yaml motion.joint_limits: alpha 60..200, beta -20..115) and not
# above the speed ceiling (30 deg/s). 90 and 45 degrees are 800 and 400 whole steps (3200 steps per turn).
MOVE_LINE = "MOVE A=90 B=45 F=20"
# Lines of check 2 beyond the decoder (SPEC 5.2): well-formed, but outside what the robot allows.
OUT_OF_RANGE_LINES = ("MOVE A=1 B=45 F=20", "MOVE A=90 B=45 F=31", "MOVE A=90 B=116 F=20", "ACCEL S=61")
# Well-formed lines of the commands the model only gates (MOVE and every form of HOME).
GATED_LINES = frozenset({MOVE_LINE, "HOME", "HOME Z", "HOME AB", "HOME Z AB"})
# Lines that are valid for the parser and that the model answers only through the gates.
VALID_OTHER_LINES = GATED_LINES

# The lines the exhaustive scripts are built from. "FOO" is an unknown command, "PING x" a bad parameter.
ALPHABET = ("INFO", "PING", "ENABLE", "DISABLE", "ESTOP", "RESET", MOVE_LINE, "FOO", "PING x", "")

# The driver-gate scripts: a prelude, then one line. Every prelude is a list of lines of the shared subset.
GATE_PRELUDES: dict[str, tuple[str, ...]] = {
    "fresh": (),
    "disabled": ("DISABLE",),
    "estop": ("ESTOP",),
    "estop+disabled": ("ESTOP", "DISABLE"),
    "disabled, enabled again": ("DISABLE", "ENABLE"),
}
GATE_LINES = (*sorted(GATED_LINES), "MOVE A=1", "HOME x", *OUT_OF_RANGE_LINES)

BRINGUP_SESSION = (
    "INFO", "PING", "ENABLE", "DISABLE", "ENABLE", "ESTOP", MOVE_LINE, "ENABLE", "DISABLE", "ESTOP",
    "PING", "INFO", "RESET", "ENABLE", "FOO", "ping", "PING x", "GRIP S=181", "",
)


@dataclass(frozen=True)
class Step:
    """What one input line produced."""

    line: str
    replies: tuple[str, ...]
    drivers: bool
    estop: bool
    homed: bool


Transcript = list[Step]


# --------------------------------------------------------------------------- #
# The model (third witness)
# --------------------------------------------------------------------------- #


def needs_joint_drivers(words: Sequence[str]) -> bool:
    """MOVE, and HOME with no flag or with AB. ``HOME Z`` moves only Z and needs no joint driver."""
    return words[0] == "MOVE" or (words[0] == "HOME" and ("AB" in words[1:] or "Z" not in words[1:]))


def model_gate_reply(words: Sequence[str], drivers: bool, estop: bool, homed: bool, queue_full: bool) -> Optional[str]:
    """The reply of the checks 5, 6, 4, 3 (OWNER_CHECK_ORDER) for a well-formed MOVE / HOME line, or None when it passes.

    The checks 1 and 2 (the line itself) are decided before, by the caller. The first check that fails decides.
    """
    for number in OWNER_CHECK_ORDER[2:]:
        if number == 5 and estop and words[0] not in OWNER_ESTOP_SET:
            return "error:5"
        if number == 6 and needs_joint_drivers(words) and not drivers:
            return "error:6"
        if number == 4 and words[0] == "MOVE" and not homed:
            return "error:4"
        if number == 3 and words[0] == "MOVE" and queue_full:
            return "error:3"
    return None


def model_transcript(script: Sequence[str], firmware_version: str) -> Transcript:
    """The owner's rules for the lines of ALPHABET (and the stage 4a stubs outside it), applied step by step."""
    drivers, estop = True, False  # power-up: drivers enabled (SPEC 10.2), no ESTOP
    steps: Transcript = []
    for line in script:
        words = line.split()
        replies: tuple[str, ...]
        if not words:
            replies = ()
        elif words[0] == "INFO" and len(words) == 1:
            replies = (f"INFO FW={firmware_version} PROTO={PROTOCOL_VERSION}", "ok")
        elif words[0] == "PING" and len(words) == 1:
            replies = ("ok",)
        elif words[0] == "ENABLE" and len(words) == 1:
            if estop:
                replies = ("error:5",)
            else:
                drivers, replies = True, ("ok",)
        elif words[0] == "DISABLE" and len(words) == 1:
            drivers, replies = False, ("ok",)  # accepted in an ESTOP too
        elif words[0] == "ESTOP" and len(words) == 1:
            estop, replies = True, ("!alarm:ESTOP",)  # a repeated ESTOP is accepted and answered again
        elif words[0] == "RESET" and len(words) == 1:
            estop, replies = False, ("ok",)  # RESET leaves the drivers as they are
        elif words[0] not in COMMANDS:
            replies = ("error:1",)
        elif line in VALID_OTHER_LINES:
            # A well-formed MOVE / HOME: only the gates are modelled. The homed flag is never set in these scripts and
            # the queue is never full, so a MOVE is always refused by one of them; HOME may pass (a stub on the firmware).
            reply = model_gate_reply(words, drivers, estop, False, False)
            if reply is None:
                raise ValueError(f"the model has no answer for {line!r}: it passes every gate (a stage 4a stub on the firmware)")
            replies = (reply,)
        else:
            replies = ("error:2",)  # a known command with a parameter it does not take or an invalid one
        steps.append(Step(line, replies, drivers, estop, False))  # the homed flag is never set in these scripts
    return steps


# --------------------------------------------------------------------------- #
# The firmware core, through replay.cpp
# --------------------------------------------------------------------------- #


def build_replay(cxx: str, firmware_dir: Path, out_dir: Path) -> Path:
    core_src = firmware_dir / "lib" / "chessbot_core" / "src"
    stub = firmware_dir / "tools" / "host_main" / "Arduino.h"
    binary = out_dir / "replay"
    command = [
        cxx, "-std=gnu++17", "-Wall", "-Wextra", "-Wshadow", "-Werror",
        "-include", str(stub), f"-I{firmware_dir / 'include'}", f"-I{core_src}",
        str(HERE / "replay.cpp"), *sorted(str(p) for p in core_src.glob("*.cpp")), "-o", str(binary),
    ]
    result = subprocess.run(command, capture_output=True, text=True, check=False)
    if result.returncode != 0:
        raise RuntimeError(f"cannot build the replay program:\n{result.stderr[:2000]}")
    return binary


def run_firmware(binary: Path, scripts: Sequence[Sequence[str]], *, bench: bool = False) -> tuple[str, list[Transcript]]:
    """Run every script on a fresh device of the C++ core; returns the firmware version and one transcript each.

    ``bench``: motion.bench_move_without_homing is on. The pseudo-line ``@finish`` lets the (faked) joints arrive.
    """
    separator = "\n#NEW bench\n" if bench else "\n#NEW\n"
    text = ("#NEW bench\n" if bench else "") + separator.join("\n".join(script) for script in scripts) + "\n"
    result = subprocess.run([str(binary)], input=text, capture_output=True, text=True, check=True)
    lines = result.stdout.split("\n")
    assert lines[0].startswith("FW "), lines[:2]
    version = lines[0][3:]
    transcripts: list[Transcript] = [] if bench else [[]]
    pending_line: Optional[str] = None
    replies: list[str] = []
    for out in lines[1:]:
        if out.startswith("#NEW"):
            transcripts.append([])
        elif out.startswith("> "):
            pending_line, replies = out[2:], []
        elif out.startswith("< "):
            replies.append(out[2:])
        elif out.startswith("= "):
            fields = dict(token.split("=") for token in out[2:].split())
            assert pending_line is not None
            transcripts[-1].append(
                Step(pending_line, tuple(replies), fields["drivers"] == "1", fields["estop"] == "1", fields["homed"] == "1")
            )
    return version, transcripts


# --------------------------------------------------------------------------- #
# The simulator
# --------------------------------------------------------------------------- #


FINISH = "@finish"  # pseudo-line of the bench scripts: the joints have arrived (see the module docstring)
FINISH_ADVANCE_S = 1000.0  # how far the simulator clock jumps for it; far beyond any MOVE of the scripts


def run_simulator(scripts: Sequence[Sequence[str]], firmware_version: str, *, bench: bool = False) -> list[Transcript]:
    # The committed settings.yaml only (no local overlay, no environment), like the generated headers.
    settings = load_settings(overlay=False)
    # Test values for settings the shared subset never uses (nothing moves in these scripts).
    # The queue length is the firmware's (serial.queue_depth); the bench flag is the only difference of the two modes.
    config = SimConfig.from_settings(
        settings, z_travel_s=1.0, heartbeat_timeout_s=2.0, home_joints_s=0.5, initial_joint_deg=(0.0, 0.0),
        firmware_version=firmware_version, bench_move_without_homing=bench,
    )
    transcripts: list[Transcript] = []
    for script in scripts:
        now = [0.0]
        sim, pair = make_loopback_simulator(config, clock=lambda: now[0])
        pair.pc.open()
        sim.open_link()
        steps: Transcript = []
        for line in script:
            if line == FINISH:
                now[0] += FINISH_ADVANCE_S
            else:
                pair.pc.write((line + "\n").encode("ascii"))
            sim.pump()
            try:
                raw = pair.pc.read(0.0)
            except ConnectionLost:
                raw = b""
            replies = tuple(part for part in raw.decode("ascii").split("\n") if part)
            steps.append(Step(line, replies, sim.drivers_enabled, sim.estop, sim.homed))
        transcripts.append(steps)
    return transcripts


# --------------------------------------------------------------------------- #
# Scripts and comparison
# --------------------------------------------------------------------------- #


def line_for(name: str) -> str:
    """A valid line for ``name``: every required parameter given the value 1 (a MOVE: MOVE_LINE, 1 is outside the limits)."""
    if name == "MOVE":
        return MOVE_LINE
    return " ".join([name, *(f"{p.name}=1" for p in COMMANDS[name].params if p.required)])


def build_scripts(max_length: int) -> tuple[list[tuple[str, ...]], dict[str, int], dict[tuple[str, str], int]]:
    """All scripts, the index of the ESTOP gate script of each command, and the index of each driver-gate script."""
    scripts: list[tuple[str, ...]] = [BRINGUP_SESSION]
    for length in range(1, max_length + 1):
        scripts.extend(itertools.product(ALPHABET, repeat=length))
    gate: dict[str, int] = {}
    for name in COMMAND_ORDER:
        gate[name] = len(scripts)
        scripts.append(("ESTOP", line_for(name)))
    driver_gate: dict[tuple[str, str], int] = {}
    for prelude_name, prelude in GATE_PRELUDES.items():
        for line in GATE_LINES:
            driver_gate[(prelude_name, line)] = len(scripts)
            scripts.append((*prelude, line))
    return scripts, gate, driver_gate


def describe(steps: Transcript) -> str:
    return "\n".join(
        f"      {s.line!r:24} -> {list(s.replies)}  drivers={int(s.drivers)} estop={int(s.estop)} homed={int(s.homed)}" for s in steps
    )


def compare_shared(script: Sequence[str], model: Transcript, firmware: Transcript, simulator: Transcript) -> Optional[str]:
    """None when the three witnesses agree on this script, else a readable description."""
    if model == firmware == simulator:
        return None
    return (
        f"  script {list(script)}\n    model:\n{describe(model)}\n    firmware core:\n{describe(firmware)}\n"
        f"    simulator:\n{describe(simulator)}"
    )


def check_gate(name: str, firmware: Transcript, simulator: Transcript) -> list[str]:
    """The ESTOP gate for one command, from a fresh ESTOP. Returns problems (empty when fine)."""
    problems: list[str] = []
    expected_refused = name not in OWNER_ESTOP_SET
    for side, steps in (("firmware core", firmware), ("simulator", simulator)):
        replies = list(steps[-1].replies)
        refused = replies == ["error:5"]
        if refused != expected_refused:
            problems.append(f"{name} in an ESTOP on the {side}: answered {replies}, expected {'error:5' if expected_refused else 'anything but error:5'}")
        if "error:5" in replies and not expected_refused:
            problems.append(f"{name} in an ESTOP on the {side} contains error:5: {replies}")
    if name in STUBS_IN_THE_FIRMWARE and list(firmware[-1].replies) != ["error:1"]:
        problems.append(f"{name} is expected to be an error:1 stub in the stage 4a firmware, got {list(firmware[-1].replies)}: update STUBS_IN_THE_FIRMWARE")
    return problems


def check_driver_gate(prelude_name: str, line: str, firmware: Transcript, simulator: Transcript) -> list[str]:
    """One driver-gate script: the replies of both implementations against the model, and HOME never enables."""
    problems: list[str] = []
    prelude = list(GATE_PRELUDES[prelude_name])
    before = model_transcript(prelude, "x")[-1] if prelude else None
    drivers = before.drivers if before is not None else True
    estop = before.estop if before is not None else False
    words = line.split()
    if line not in GATED_LINES:  # a bad line: the parse checks (1, 2) come before every gate
        expected: Optional[str] = "error:2"
    else:
        expected = model_gate_reply(words, drivers, estop, False, False)
    where = f"{line!r} after {prelude_name}"
    fw, sim = list(firmware[-1].replies), list(simulator[-1].replies)
    if expected is not None:
        for side, replies in (("firmware core", fw), ("simulator", sim)):
            if replies != [expected]:
                problems.append(f"{where} on the {side}: answered {replies}, the model says {expected}")
    else:  # passed every gate: the 4a firmware stub says error:1, the simulator really runs it
        if fw != ["error:1"]:
            problems.append(f"{where} on the firmware core: answered {fw}, expected the error:1 stub (it passes every gate)")
        # ok, and for a HOME Z that starts with Z already at TOP also the done (the clock never advances here)
        if not sim or sim[0] != "ok" or any(reply != "done" for reply in sim[1:]):
            problems.append(f"{where} on the simulator: answered {sim}, expected ok (it passes every gate)")
    for side, steps in (("firmware core", firmware), ("simulator", simulator)):
        if steps[-1].drivers != drivers:
            problems.append(f"{where} on the {side}: the drivers went {drivers} -> {steps[-1].drivers}; HOME and MOVE never change them")
    return problems


# --------------------------------------------------------------------------- #
# The bench scripts (motion.bench_move_without_homing on, on both sides)
# --------------------------------------------------------------------------- #

BenchScript = tuple[str, tuple[str, ...], tuple[tuple[str, ...], ...]]  # name, lines, the expected replies of each line

POS_AT_ZERO = ("POS A=0 B=0 NA=0 NB=0", "ok")
POS_AT_MOVE_LINE = ("POS A=90 B=45 NA=800 NB=400", "ok")  # 90 and 45 degrees at 3200 / 360 steps per degree
BENCH_OK = ("ok",)
BENCH_OK_DONE = ("ok", "done")
BENCH_NOTHING: tuple[str, ...] = ()


def bench_scripts(queue_depth: int) -> list[BenchScript]:
    """The bench scripts with what the owner's rules say each line answers (typed out, not derived from either side)."""
    other_move = "MOVE A=100 B=45 F=20"
    flood = tuple(MOVE_LINE if index % 2 == 0 else other_move for index in range(queue_depth))
    return [
        ("a MOVE needs no HOME, WAIT answers done when the joints arrive, POS then reports the target",
         (MOVE_LINE, "WAIT", FINISH, "POS"), (BENCH_OK, BENCH_OK, ("done",), POS_AT_MOVE_LINE)),
        ("POS at power-up", ("POS",), (POS_AT_ZERO,)),
        ("a full queue answers error:3, and the queue drains when the joints arrive",
         (*flood, MOVE_LINE, FINISH, "WAIT"), (*([BENCH_OK] * queue_depth), ("error:3",), BENCH_NOTHING, BENCH_OK_DONE)),
        ("DISABLE cancels the done of a pending WAIT and stops the joints where they are",
         (MOVE_LINE, "WAIT", "DISABLE", FINISH, "WAIT", "POS"),
         (BENCH_OK, BENCH_OK, BENCH_OK, BENCH_NOTHING, BENCH_OK_DONE, POS_AT_ZERO)),
        ("ESTOP cancels the done of a pending WAIT and stops the joints",
         (MOVE_LINE, "WAIT", "ESTOP", FINISH, "RESET", "WAIT", "POS"),
         (BENCH_OK, BENCH_OK, ("!alarm:ESTOP",), BENCH_NOTHING, BENCH_OK, BENCH_OK_DONE, POS_AT_ZERO)),
        ("drivers disabled: a MOVE is error:6", ("DISABLE", MOVE_LINE), (BENCH_OK, ("error:6",))),
        ("ESTOP: a MOVE is error:5", ("ESTOP", MOVE_LINE), (("!alarm:ESTOP",), ("error:5",))),
        ("ESTOP and DISABLE: error:5 comes before error:6", ("ESTOP", "DISABLE", MOVE_LINE), (("!alarm:ESTOP",), BENCH_OK, ("error:5",))),
        ("check 2 (outside a joint limit) comes before error:5", ("ESTOP", "MOVE A=1 B=45 F=20"), (("!alarm:ESTOP",), ("error:2",))),
        ("check 2 (above the speed ceiling) comes before error:6", ("DISABLE", "MOVE A=90 B=45 F=31"), (BENCH_OK, ("error:2",))),
        ("check 2 (beta above its limit)", ("MOVE A=90 B=116 F=20",), (("error:2",),)),
        ("the limits and the ceilings are inclusive",
         ("MOVE A=60 B=-20 F=30", FINISH, "MOVE A=200 B=115 F=20", FINISH, "POS"),
         (BENCH_OK, BENCH_NOTHING, BENCH_OK, BENCH_NOTHING, ("POS A=200 B=115 NA=1778 NB=1022", "ok"))),
        ("ACCEL above the ceiling is error:2, at the ceiling ok",
         ("ACCEL S=61", "ACCEL S=60", MOVE_LINE, FINISH, "POS"),
         (("error:2",), BENCH_OK, BENCH_OK, BENCH_NOTHING, POS_AT_MOVE_LINE)),
        ("a second MOVE starts from where the first arrived",
         (MOVE_LINE, FINISH, "MOVE A=60 B=-20 F=20", FINISH, "POS"),
         (BENCH_OK, BENCH_NOTHING, BENCH_OK, BENCH_NOTHING, ("POS A=60 B=-20 NA=533 NB=-178", "ok"))),
    ]


def normalise_reply(reply: str) -> str:
    """POS without the fields the firmware cannot fill yet: SA, SB (the AS5600, stage 4b-4) and Z (the limit switches, 4b-2)."""
    if not reply.startswith("POS "):
        return reply
    return " ".join(token for token in reply.split() if not token.startswith(("SA=", "SB=", "Z=")))


def check_bench(script: BenchScript, firmware: Transcript, simulator: Transcript) -> list[str]:
    """Problems of one bench script: each side against the typed-out replies, and both sides' state against each other."""
    name, lines, expected = script
    problems: list[str] = []
    for side, steps in (("firmware core", firmware), ("simulator", simulator)):
        for index, (line, want) in enumerate(zip(lines, expected)):
            got = tuple(normalise_reply(reply) for reply in steps[index].replies)
            if got != want:
                problems.append(f"bench '{name}', line {index + 1} {line!r} on the {side}: answered {list(got)}, expected {list(want)}")
    for index, (fw, sim) in enumerate(zip(firmware, simulator)):
        if (fw.drivers, fw.estop, fw.homed) != (sim.drivers, sim.estop, sim.homed):
            problems.append(f"bench '{name}', line {index + 1} {fw.line!r}: state firmware drivers/estop/homed {(fw.drivers, fw.estop, fw.homed)} != simulator {(sim.drivers, sim.estop, sim.homed)}")
    return problems


@dataclass
class Report:
    scripts: int
    mismatches: list[str]
    gate_problems: list[str]
    bench_problems: list[str] = field(default_factory=list)
    bench_scripts: int = 0

    @property
    def ok(self) -> bool:
        return not self.mismatches and not self.gate_problems and not self.bench_problems


def run(cxx: str, firmware_dir: Path, max_length: int) -> Report:
    scripts, gate, driver_gate = build_scripts(max_length)
    with tempfile.TemporaryDirectory() as tmp:
        binary = build_replay(cxx, firmware_dir, Path(tmp))
        version, firmware = run_firmware(binary, scripts)
    simulator = run_simulator(scripts, version)
    assert len(firmware) == len(simulator) == len(scripts), (len(firmware), len(simulator), len(scripts))

    mismatches: list[str] = []
    for index, script in enumerate(scripts):
        found = compare_shared(script, model_transcript(script, version), firmware[index], simulator[index]) if index < gate[COMMAND_ORDER[0]] else None
        if found is not None:
            mismatches.append(found)
    gate_problems: list[str] = []
    for name, index in gate.items():
        gate_problems.extend(check_gate(name, firmware[index], simulator[index]))
        if name in ("INFO", "PING", "ESTOP", "RESET", "DISABLE"):  # implemented on both sides: exact replies too
            if firmware[index][-1].replies != simulator[index][-1].replies:
                gate_problems.append(f"{name} in an ESTOP: firmware core {list(firmware[index][-1].replies)} != simulator {list(simulator[index][-1].replies)}")
    for (prelude_name, line), index in driver_gate.items():
        gate_problems.extend(check_driver_gate(prelude_name, line, firmware[index], simulator[index]))

    bench = bench_scripts(load_settings(overlay=False).serial.queue_depth)
    with tempfile.TemporaryDirectory() as tmp:
        binary = build_replay(cxx, firmware_dir, Path(tmp))
        _, bench_firmware = run_firmware(binary, [lines for _, lines, _ in bench], bench=True)
    bench_simulator = run_simulator([lines for _, lines, _ in bench], version, bench=True)
    bench_problems: list[str] = []
    for script, fw_steps, sim_steps in zip(bench, bench_firmware, bench_simulator):
        bench_problems.extend(check_bench(script, fw_steps, sim_steps))
    return Report(len(scripts), mismatches, gate_problems, bench_problems, len(bench))


def main(argv: Optional[Sequence[str]] = None) -> int:
    parser = argparse.ArgumentParser(description="Compare the C++ firmware core with the Python ESP32 simulator on the wire.")
    parser.add_argument("--cxx", default=os.environ.get("CXX") or shutil.which("g++") or "c++")
    parser.add_argument("--max-length", type=int, default=4)
    parser.add_argument("--firmware-dir", type=Path, default=DEFAULT_FIRMWARE_DIR, help="a firmware/ directory (default: this repository's)")
    args = parser.parse_args(argv)
    logging.basicConfig(level=logging.ERROR)  # the settings loader may log about assumed values

    try:
        report = run(args.cxx, args.firmware_dir, args.max_length)
    except RuntimeError as exc:
        sys.stdout.write(f"wire_compat: {exc}\n")
        return 2
    for text in report.mismatches[:5]:
        sys.stdout.write("MISMATCH\n" + text + "\n")
    if len(report.mismatches) > 5:
        sys.stdout.write(f"... and {len(report.mismatches) - 5} more mismatching scripts\n")
    for problem in report.gate_problems:
        sys.stdout.write(f"GATE: {problem}\n")
    for problem in report.bench_problems:
        sys.stdout.write(f"BENCH: {problem}\n")
    if report.ok:
        sys.stdout.write(f"WIRE COMPAT OK: {report.scripts} scripts, C++ core == simulator == model; ESTOP gate checked for all {len(COMMAND_ORDER)} commands; driver gates checked ({len(GATE_PRELUDES)} preludes x {len(GATE_LINES)} lines); {report.bench_scripts} bench scripts (MOVE without HOME)\n")
        return 0
    sys.stdout.write(f"WIRE COMPAT FAILED: {len(report.mismatches)} mismatching scripts, {len(report.gate_problems)} gate problems, {len(report.bench_problems)} bench problems\n")
    return 1


if __name__ == "__main__":
    sys.exit(main())
