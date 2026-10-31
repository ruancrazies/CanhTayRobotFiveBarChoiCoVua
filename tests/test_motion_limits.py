"""Check 2 beyond the decoder (SPEC 5.2), the bench setting and the S1 settings, on the Python simulator (stage 4b-1).

The C++ core is held to the same rules by firmware/test/test_motion and test_processor, and by the wire-compat bench
scripts (tests/test_wire_compat.py). Every value used here comes from config/settings.yaml, so a change of a limit
there keeps the tests right.
"""

from __future__ import annotations

from typing import Any

import pytest

from desktop_app.hardware.esp32_simulator import SimConfig
from desktop_app.utils.settings import load_settings
from tests.sim_support import Bench

SETTINGS = load_settings(overlay=False)
MOTION = SETTINGS.motion
LIMITS = MOTION.joint_limits
MID_A = (LIMITS.alpha_min_deg + LIMITS.alpha_max_deg) / 2
MID_B = (LIMITS.beta_min_deg + LIMITS.beta_max_deg) / 2


def _bench(**overrides: Any) -> Bench:
    values: dict[str, Any] = dict(
        joint_limits_deg=((LIMITS.alpha_min_deg, LIMITS.alpha_max_deg), (LIMITS.beta_min_deg, LIMITS.beta_max_deg)),
        max_speed_deg_s=MOTION.max_speed_deg_s,
        max_accel_deg_s2=MOTION.max_accel_deg_s2,
        bench_move_without_homing=True,
        initial_joint_deg=(0.0, 0.0),
    )
    values.update(overrides)
    return Bench(**values)


def _send(bench: Bench, line: str) -> list[str]:
    bench.pair.pc.write((line + "\n").encode("ascii"))
    bench.sim.pump()
    return bench.lines()


def test_the_settings_carry_the_assumed_s1_values() -> None:
    assert (MOTION.max_speed_deg_s, MOTION.max_accel_deg_s2, MOTION.direction_change_delay_us) == (30.0, 60.0, 20)
    assert (LIMITS.alpha_min_deg, LIMITS.alpha_max_deg, LIMITS.beta_min_deg, LIMITS.beta_max_deg) == (60.0, 200.0, -20.0, 115.0)
    assert MOTION.bench_move_without_homing is False  # the bench setting is off by default
    for key in ("motion.max_speed_deg_s", "motion.max_accel_deg_s2", "motion.joint_limits.alpha_min_deg"):
        assert key in SETTINGS.assumed, key  # still to be confirmed by the owner


def test_the_simulator_takes_the_ceilings_and_the_bench_flag_from_settings() -> None:
    config = SimConfig.from_settings(SETTINGS, z_travel_s=1.0, home_joints_s=0.5, initial_joint_deg=(0.0, 0.0))
    assert config.max_speed_deg_s == MOTION.max_speed_deg_s
    assert config.max_accel_deg_s2 == MOTION.max_accel_deg_s2
    assert config.bench_move_without_homing is False


@pytest.mark.parametrize(
    "line",
    [
        f"MOVE A={LIMITS.alpha_min_deg - 1} B={MID_B} F=10",
        f"MOVE A={LIMITS.alpha_max_deg + 1} B={MID_B} F=10",
        f"MOVE A={MID_A} B={LIMITS.beta_min_deg - 1} F=10",
        f"MOVE A={MID_A} B={LIMITS.beta_max_deg + 1} F=10",
        f"MOVE A={MID_A} B={MID_B} F={MOTION.max_speed_deg_s + 1}",
        f"ACCEL S={MOTION.max_accel_deg_s2 + 1}",
    ],
)
def test_out_of_range_values_are_error_2_and_nothing_moves(line: str) -> None:
    bench = _bench()
    assert _send(bench, line) == ["error:2"]
    assert not bench.sim.joints_moving and bench.sim.queue_length == 0


def test_the_limits_and_the_ceilings_themselves_are_accepted() -> None:
    bench = _bench()
    assert _send(bench, f"MOVE A={LIMITS.alpha_min_deg} B={LIMITS.beta_min_deg} F={MOTION.max_speed_deg_s}") == ["ok"]
    assert _send(bench, f"ACCEL S={MOTION.max_accel_deg_s2}") == ["ok"]
    assert _send(bench, f"MOVE A={LIMITS.alpha_max_deg} B={LIMITS.beta_max_deg} F=1") == ["ok"]


def test_check_2_comes_before_the_estop_and_the_driver_checks() -> None:
    bench = _bench()
    out_of_range = f"MOVE A={LIMITS.alpha_min_deg - 1} B={MID_B} F=10"
    in_range = f"MOVE A={MID_A} B={MID_B} F=10"
    assert _send(bench, "DISABLE") == ["ok"]
    assert _send(bench, out_of_range) == ["error:2"]  # not error:6
    assert _send(bench, in_range) == ["error:6"]
    assert _send(bench, "ESTOP") == ["!alarm:ESTOP"]
    assert _send(bench, out_of_range) == ["error:2"]  # not error:5
    assert _send(bench, in_range) == ["error:5"]  # check 5 before check 6


def test_without_the_bench_setting_a_move_is_error_4_until_homed() -> None:
    bench = _bench(bench_move_without_homing=False)
    assert _send(bench, f"MOVE A={MID_A} B={MID_B} F=10") == ["error:4"]


def test_the_bench_setting_skips_only_the_homed_check() -> None:
    bench = _bench()
    assert _send(bench, f"MOVE A={MID_A} B={MID_B} F=10") == ["ok"]
    assert _send(bench, "DISABLE") == ["ok"]
    assert _send(bench, f"MOVE A={MID_A} B={MID_B} F=10") == ["error:6"]  # the other checks still apply


def test_a_ceiling_that_is_not_modelled_accepts_any_value() -> None:
    bench = _bench(max_speed_deg_s=None, max_accel_deg_s2=None, joint_limits_deg=None)
    assert _send(bench, "ACCEL S=1000") == ["ok"]
    assert _send(bench, "MOVE A=10 B=10 F=500") == ["ok"]
