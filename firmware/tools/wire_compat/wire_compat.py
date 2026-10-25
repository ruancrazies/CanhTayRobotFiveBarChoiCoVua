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
  RESET, an unknown command, a bad parameter and a blank line, each on a fresh device (all of the ESTOP and
  ENABLE / DISABLE state machine);
* the BRINGUP.md session of stage 4a;
* the ESTOP gate for ALL 15 commands: a fresh ESTOP, then the command.

Known stage 4a differences, asserted explicitly instead of ignored:
* ZSTOP and POS are accepted in an ESTOP but are stubs in the firmware (error:1) until a later stage, while the
  simulator implements them. For them the gate check requires "not error:5" on both sides, and error:1 from
  the firmware. When a stage implements them, update STUBS_IN_THE_FIRMWARE below.
* The homed flag is never set in stage 4a (HOME is a stub), so "DISABLE clears the homed flag" is not visible
  on the firmware's wire yet. It is unit tested in firmware/test (test_state, test_processor) and in
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
from dataclasses import dataclass
from pathlib import Path
from typing import Optional, Sequence

REPO_ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(REPO_ROOT))

from config.protocol_commands import COMMAND_ORDER, COMMANDS, PROTOCOL_VERSION  # noqa: E402
from desktop_app.hardware.esp32_simulator import SimConfig, make_loopback_simulator  # noqa: E402
from desktop_app.hardware.serial_link import ConnectionLost  # noqa: E402
from desktop_app.utils.hardware_config import load_hardware_config  # noqa: E402

logger = logging.getLogger("wire_compat")

DEFAULT_FIRMWARE_DIR = REPO_ROOT / "firmware"
HERE = Path(__file__).resolve().parent

# The owner's decisions, typed out here on purpose (this is the third witness): while the ESTOP flag is set
# only these are accepted; every other command, ENABLE included, is error:5.
OWNER_ESTOP_SET = frozenset({"ZSTOP", "DISABLE", "POS", "INFO", "PING", "ESTOP", "RESET"})
# Commands in OWNER_ESTOP_SET that the stage 4a firmware parses but does not implement (it answers error:1).
STUBS_IN_THE_FIRMWARE = frozenset({"ZSTOP", "POS"})

# Well-formed lines of commands outside the model's own set, used in BRINGUP_SESSION in an ESTOP only.
VALID_OTHER_LINES = frozenset({"MOVE A=1 B=2 F=3"})

# The lines the exhaustive scripts are built from. "FOO" is an unknown command, "PING x" a bad parameter.
ALPHABET = ("INFO", "PING", "ENABLE", "DISABLE", "ESTOP", "RESET", "FOO", "PING x", "")

BRINGUP_SESSION = (
    "INFO", "PING", "ENABLE", "DISABLE", "ENABLE", "ESTOP", "MOVE A=1 B=2 F=3", "ENABLE", "DISABLE", "ESTOP",
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
            # A well-formed line of a command the model does not run: the ESTOP gate comes after parsing.
            if not estop:
                raise ValueError(f"the model has no answer for {line!r} outside an ESTOP (a stage 4a stub on the firmware)")
            replies = ("error:5",)
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


def run_firmware(binary: Path, scripts: Sequence[Sequence[str]]) -> tuple[str, list[Transcript]]:
    """Run every script on a fresh device of the C++ core; returns the firmware version and one transcript each."""
    text = "\n#NEW\n".join("\n".join(script) for script in scripts) + "\n"
    result = subprocess.run([str(binary)], input=text, capture_output=True, text=True, check=True)
    lines = result.stdout.split("\n")
    assert lines[0].startswith("FW "), lines[:2]
    version = lines[0][3:]
    transcripts: list[Transcript] = [[]]
    pending_line: Optional[str] = None
    replies: list[str] = []
    for out in lines[1:]:
        if out == "#NEW":
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


def run_simulator(scripts: Sequence[Sequence[str]], firmware_version: str) -> list[Transcript]:
    hardware = load_hardware_config()
    # Test values for settings the shared subset never uses (nothing moves in these scripts).
    config = SimConfig.from_hardware(
        queue_capacity=4, z_travel_s=1.0, heartbeat_timeout_s=2.0, home_joints_s=0.5,
        initial_joint_deg=(0.0, 0.0), hardware=hardware, firmware_version=firmware_version,
    )
    transcripts: list[Transcript] = []
    for script in scripts:
        sim, pair = make_loopback_simulator(config, clock=lambda: 0.0)
        pair.pc.open()
        sim.open_link()
        steps: Transcript = []
        for line in script:
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
    """A valid line for ``name``: every required parameter given the value 1."""
    return " ".join([name, *(f"{p.name}=1" for p in COMMANDS[name].params if p.required)])


def build_scripts(max_length: int) -> tuple[list[tuple[str, ...]], dict[str, int]]:
    """All scripts, and the index of the gate script of each command."""
    scripts: list[tuple[str, ...]] = [BRINGUP_SESSION]
    for length in range(1, max_length + 1):
        scripts.extend(itertools.product(ALPHABET, repeat=length))
    gate: dict[str, int] = {}
    for name in COMMAND_ORDER:
        gate[name] = len(scripts)
        scripts.append(("ESTOP", line_for(name)))
    return scripts, gate


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


@dataclass
class Report:
    scripts: int
    mismatches: list[str]
    gate_problems: list[str]

    @property
    def ok(self) -> bool:
        return not self.mismatches and not self.gate_problems


def run(cxx: str, firmware_dir: Path, max_length: int) -> Report:
    scripts, gate = build_scripts(max_length)
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
    return Report(len(scripts), mismatches, gate_problems)


def main(argv: Optional[Sequence[str]] = None) -> int:
    parser = argparse.ArgumentParser(description="Compare the C++ firmware core with the Python ESP32 simulator on the wire.")
    parser.add_argument("--cxx", default=os.environ.get("CXX") or shutil.which("g++") or "c++")
    parser.add_argument("--max-length", type=int, default=4)
    parser.add_argument("--firmware-dir", type=Path, default=DEFAULT_FIRMWARE_DIR, help="a firmware/ directory (default: this repository's)")
    args = parser.parse_args(argv)
    logging.basicConfig(level=logging.ERROR)  # the hardware config loader warns about [TO CONFIRM] values

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
        sys.stdout.write(f"ESTOP GATE: {problem}\n")
    if report.ok:
        sys.stdout.write(f"WIRE COMPAT OK: {report.scripts} scripts, C++ core == simulator == model; ESTOP gate checked for all {len(COMMAND_ORDER)} commands\n")
        return 0
    sys.stdout.write(f"WIRE COMPAT FAILED: {len(report.mismatches)} mismatching scripts, {len(report.gate_problems)} gate problems\n")
    return 1


if __name__ == "__main__":
    sys.exit(main())
