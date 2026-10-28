"""Shared helpers for the phase 3 tests: a manual clock, a bench that talks to the simulator, builders.

Every timing value below is a TEST VALUE. The real ones ([TO CONFIRM]: queue depth, Z travel time,
heartbeat threshold, homing time) are measured on the hardware in phase 4 and 5.
"""

from __future__ import annotations

import time
from typing import Any, Callable, Optional

from desktop_app.hardware.esp32_simulator import Esp32Simulator, SimConfig, make_loopback_simulator
from desktop_app.hardware.serial_link import ConnectionLost, LoopbackPair

START_JOINTS = (150.0, 30.0)  # TEST ONLY: where the simulated arm sits at power-up


class ManualClock:
    """A clock that only moves when a test says so, so every timing assertion is exact."""

    def __init__(self, start: float = 1000.0) -> None:
        self.now = start

    def __call__(self) -> float:
        return self.now

    def advance(self, seconds: float) -> None:
        self.now += seconds


def sim_config(**overrides: Any) -> SimConfig:
    values: dict[str, Any] = dict(
        queue_capacity=4,
        z_travel_s=1.0,
        heartbeat_timeout_s=2.0,
        home_joints_s=0.5,
        initial_joint_deg=START_JOINTS,
    )
    values.update(overrides)
    return SimConfig.from_settings(**values)


def fast_sim_config(**overrides: Any) -> SimConfig:
    """Timings small enough for real-time (threaded) tests."""
    values: dict[str, Any] = dict(
        queue_capacity=6, z_travel_s=0.02, heartbeat_timeout_s=0.5, home_joints_s=0.01, poll_s=0.001
    )
    values.update(overrides)
    return sim_config(**values)


class Bench:
    """A simulator on a manual clock; the test plays the PC by hand. No threads."""

    def __init__(self, **overrides: Any) -> None:
        self.clock = ManualClock()
        self.config = sim_config(**overrides)
        self.sim, self.pair = make_loopback_simulator(self.config, clock=self.clock)
        self.pair.pc.open()
        self.sim.open_link()

    def read(self) -> list[str]:
        try:
            data = self.pair.pc.read(0.0)
        except ConnectionLost:
            return []
        return data.decode().split()

    def lines(self) -> list[str]:
        """Whole response lines (data lines contain spaces, so split on newlines)."""
        try:
            data = self.pair.pc.read(0.0)
        except ConnectionLost:
            return []
        return [line for line in data.decode().split("\n") if line]

    def send(self, *commands: str) -> list[str]:
        for command in commands:
            self.pair.pc.write((command + "\n").encode())
        self.sim.pump()
        return self.lines()

    def advance(self, seconds: float) -> list[str]:
        self.clock.advance(seconds)
        self.sim.pump()
        return self.lines()

    def home(self) -> None:
        """Send HOME and let it finish; leaves the arm homed with Z at TOP."""
        assert self.send("HOME") == ["ok"]
        self.clock.advance(self.config.z_travel_s + self.config.home_joints_s)
        self.sim.pump()
        assert self.lines() == ["done"]
        assert self.sim.homed

    def pos(self) -> dict[str, str]:
        reply = self.send("POS")
        assert reply[-1] == "ok", reply
        return dict(token.split("=") for token in reply[0].split()[1:])


def wait_for(predicate: Callable[[], bool], timeout_s: float = 2.0) -> bool:
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        if predicate():
            return True
        time.sleep(0.001)
    return predicate()


def threaded_simulator(**overrides: Any) -> tuple[Esp32Simulator, LoopbackPair]:
    """A simulator serving in its own thread on the real clock (stop it with ``sim.stop()``)."""
    sim, pair = make_loopback_simulator(fast_sim_config(**overrides))
    sim.start()
    return sim, pair
