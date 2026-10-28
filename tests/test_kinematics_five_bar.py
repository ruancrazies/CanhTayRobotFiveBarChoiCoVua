"""Tests for desktop_app/kinematics/five_bar.py (SPEC section 6.1 and 6.2)."""

from __future__ import annotations

import dataclasses
import math
from pathlib import Path
from typing import Any

import numpy as np
import pytest
import yaml

from desktop_app.kinematics.five_bar import (
    FiveBarKinematics,
    InvalidPoseError,
    KinematicsError,
    OutOfReachError,
    SingularityError,
)
from desktop_app.utils.settings import Settings, load_settings
from phase2_support import MIN_SINGULARITY_MARGIN_DEG

# SPEC 6.2 test vectors: square -> (x, y, alpha, beta). Tolerance +-0.1 degree.
SPEC_VECTORS = {
    "A1": (-108.5, 191.5, 184.3, 62.9),
    "H1": (108.5, 191.5, 117.1, -4.3),
    "A8": (-108.5, 408.5, 124.6, 96.5),
    "H8": (108.5, 408.5, 83.5, 55.4),
}


@pytest.fixture(scope="module")
def cfg() -> Settings:
    return load_settings(overlay=False)


@pytest.fixture(scope="module")
def model(cfg: Settings) -> FiveBarKinematics:
    return FiveBarKinematics.from_config(cfg)


@pytest.fixture(scope="module")
def centers(cfg: Settings) -> dict[str, tuple[float, float]]:
    """The 64 square centres, from the config geometry (not from the code under test)."""
    g = cfg.geometry
    x0, y0 = g.first_square_center_xy_mm
    return {
        f"{'ABCDEFGH'[f]}{r + 1}": (x0 + g.square_size_mm * f, y0 + g.square_size_mm * r)
        for f in range(g.squares_per_side)
        for r in range(g.squares_per_side)
    }


def _write_config(tmp_path: Path, data: dict[str, Any]) -> Path:
    path = tmp_path / "settings.yaml"
    path.write_text(yaml.safe_dump(data), encoding="utf-8")
    return path


def _cross(u: tuple[float, float], v: tuple[float, float]) -> float:
    return u[0] * v[1] - u[1] * v[0]


# --------------------------------------------------------------------------- #
# Model is built from the config
# --------------------------------------------------------------------------- #


def test_model_uses_config_values(model: FiveBarKinematics, cfg: Settings) -> None:
    assert model.l == cfg.geometry.motor_axis_distance_mm
    assert model.m == cfg.geometry.driving_link_mm
    assert model.n == cfg.geometry.driven_link_mm
    assert model.pulses_per_rev == cfg.drive.pulses_per_rev
    assert model.motor1_xy == cfg.geometry.motor1_xy_mm
    assert model.motor2_xy == cfg.geometry.motor2_xy_mm


def test_from_config_without_argument_loads_default_file(model: FiveBarKinematics) -> None:
    assert FiveBarKinematics.from_config() == model


def test_model_follows_the_config(tmp_path: Path, settings_yaml_data: dict[str, Any], model: FiveBarKinematics) -> None:
    # Change l and i (gear ratio) in a temporary config; the model must follow.
    settings_yaml_data["geometry"]["motor_axis_distance_mm"] = 120.0
    settings_yaml_data["drive"]["gear_ratio"] = 2.0
    changed = FiveBarKinematics.from_config(load_settings(_write_config(tmp_path, settings_yaml_data), overlay=False))

    assert changed.l == 120.0
    assert changed.m == model.m and changed.n == model.n
    assert changed.pulses_per_rev == 2.0 * model.pulses_per_rev
    assert changed.motor1_xy == (-60.0, 0.0) and changed.motor2_xy == (60.0, 0.0)

    # Different l gives different joint angles for the same point...
    point = (-108.5, 191.5)
    assert changed.ik(*point) != pytest.approx(model.ik(*point), abs=1.0)
    # ...and FK is consistent with the new l.
    alpha, beta = changed.ik(*point)
    assert changed.fk(alpha, beta) == pytest.approx(point, abs=1e-6)
    elbow_a, elbow_b = changed.elbows(alpha, beta)
    assert elbow_a[0] == pytest.approx(-60.0 + changed.m * math.cos(math.radians(alpha)))
    assert elbow_b[0] == pytest.approx(60.0 + changed.m * math.cos(math.radians(beta)))

    # Different i gives different step counts.
    assert changed.angle_to_steps(360.0) == 6400
    assert model.angle_to_steps(360.0) == 3200
    assert changed.angle_to_steps(0.05625) == 1
    assert changed.joint_resolution_deg == pytest.approx(0.05625)


@pytest.mark.parametrize("field", ["l", "m", "n", "pulses_per_rev"])
@pytest.mark.parametrize("bad", [0.0, -1.0, math.nan, math.inf])
def test_model_rejects_non_positive_or_non_finite_values(model: FiveBarKinematics, field: str, bad: float) -> None:
    with pytest.raises(KinematicsError):
        dataclasses.replace(model, **{field: bad})


# --------------------------------------------------------------------------- #
# SPEC 6.2 test vectors, IK and FK
# --------------------------------------------------------------------------- #


@pytest.mark.parametrize("square", SPEC_VECTORS)
def test_ik_matches_spec_vector(model: FiveBarKinematics, square: str) -> None:
    x, y, alpha, beta = SPEC_VECTORS[square]
    got_alpha, got_beta = model.ik(x, y)
    assert got_alpha == pytest.approx(alpha, abs=0.1)
    assert got_beta == pytest.approx(beta, abs=0.1)


@pytest.mark.parametrize("square", SPEC_VECTORS)
def test_fk_matches_spec_vector(model: FiveBarKinematics, square: str) -> None:
    x, y, alpha, beta = SPEC_VECTORS[square]
    # The table angles are rounded to 0.1 degree, so FK of the table values must
    # land where IK of that point puts them, within 0.1 degree...
    px, py = model.fk(alpha, beta)
    back_alpha, back_beta = model.ik(px, py)
    assert back_alpha == pytest.approx(alpha, abs=0.1)
    assert back_beta == pytest.approx(beta, abs=0.1)
    # ...which at the 200 mm / 250 mm links is within a millimetre of the square centre.
    assert (px, py) == pytest.approx((x, y), abs=1.0)


def test_fk_of_exact_ik_angles_returns_the_square(model: FiveBarKinematics) -> None:
    for x, y, _, _ in SPEC_VECTORS.values():
        assert model.fk(*model.ik(x, y)) == pytest.approx((x, y), abs=1e-6)


def test_round_trip_on_all_64_squares(model: FiveBarKinematics, centers: dict[str, tuple[float, float]]) -> None:
    assert len(centers) == 64
    for square, (x, y) in centers.items():
        px, py = model.fk(*model.ik(x, y))
        assert abs(px - x) < 1e-6 and abs(py - y) < 1e-6, square


def test_ik_is_elbows_out_on_all_64_squares(model: FiveBarKinematics, centers: dict[str, tuple[float, float]]) -> None:
    m1, m2 = model.motor1_xy, model.motor2_xy
    for square, (x, y) in centers.items():
        alpha, beta = model.ik(x, y)
        elbow_a, elbow_b = model.elbows(alpha, beta)
        # Elbow A is on the counter-clockwise (outer, left) side of the line motor1 -> P,
        # elbow B on the clockwise (outer, right) side of motor2 -> P.
        assert _cross((x - m1[0], y - m1[1]), (elbow_a[0] - m1[0], elbow_a[1] - m1[1])) > 0, square
        assert _cross((x - m2[0], y - m2[1]), (elbow_b[0] - m2[0], elbow_b[1] - m2[1])) < 0, square
        # and both driven links really have length n
        assert math.dist(elbow_a, (x, y)) == pytest.approx(model.n, abs=1e-9)
        assert math.dist(elbow_b, (x, y)) == pytest.approx(model.n, abs=1e-9)


# --------------------------------------------------------------------------- #
# Numbers over the 64 squares (SPEC 6.2). The SPEC values are rounded: angles
# to 0.1 degree, mu to the nearest degree ("about 26"), condition numbers to
# 2 decimals. Each check below uses exactly that rounding, nothing looser.
# --------------------------------------------------------------------------- #


def test_alpha_and_beta_spans(model: FiveBarKinematics, centers: dict[str, tuple[float, float]]) -> None:
    angles = np.array([model.ik(x, y) for x, y in centers.values()])
    assert angles[:, 0].min() == pytest.approx(83.5, abs=0.05)
    assert angles[:, 0].max() == pytest.approx(184.3, abs=0.05)
    assert angles[:, 1].min() == pytest.approx(-4.3, abs=0.05)
    assert angles[:, 1].max() == pytest.approx(96.5, abs=0.05)


def test_smallest_margin_is_about_26_at_a8_and_h8(model: FiveBarKinematics, centers: dict[str, tuple[float, float]]) -> None:
    margins = {square: model.margin_at_point(x, y) for square, (x, y) in centers.items()}
    smallest = min(margins.values())
    assert smallest == pytest.approx(26.0, abs=0.5)
    at_minimum = {square for square, mu in margins.items() if mu == pytest.approx(smallest, abs=1e-9)}
    assert at_minimum == {"A8", "H8"}
    assert all(mu >= MIN_SINGULARITY_MARGIN_DEG for mu in margins.values())


def test_margin_from_angles_equals_margin_from_point(model: FiveBarKinematics, centers: dict[str, tuple[float, float]]) -> None:
    for x, y in centers.values():
        alpha, beta = model.ik(x, y)
        assert model.singularity_margin(alpha, beta) == pytest.approx(model.margin_at_point(x, y), abs=1e-9)


def test_condition_number_range(model: FiveBarKinematics, centers: dict[str, tuple[float, float]]) -> None:
    conds = {square: model.condition_number_at_point(x, y) for square, (x, y) in centers.items()}
    # SPEC: between 1.03 and 2.79, i.e. min rounds to 1.03 and max rounds to 2.79.
    assert min(conds.values()) == pytest.approx(1.03, abs=0.005)
    assert max(conds.values()) == pytest.approx(2.79, abs=0.005)
    assert all(c >= 1.0 for c in conds.values())


def test_margin_formula_on_a_known_pose(model: FiveBarKinematics) -> None:
    # Square A8: APB, phiA, phiB computed independently from the vectors.
    x, y = -108.5, 408.5
    alpha, beta = model.ik(x, y)
    elbow_a, elbow_b = model.elbows(alpha, beta)

    def angle(u: tuple[float, float], v: tuple[float, float]) -> float:
        cos = (u[0] * v[0] + u[1] * v[1]) / (math.hypot(*u) * math.hypot(*v))
        return math.degrees(math.acos(cos))

    p = (x, y)
    apb = angle((elbow_a[0] - x, elbow_a[1] - y), (elbow_b[0] - x, elbow_b[1] - y))
    phi_a = angle((model.motor1_xy[0] - elbow_a[0], -elbow_a[1]), (x - elbow_a[0], y - elbow_a[1]))
    phi_b = angle((model.motor2_xy[0] - elbow_b[0], -elbow_b[1]), (x - elbow_b[0], y - elbow_b[1]))
    expected = min(apb, 180 - apb, phi_a, 180 - phi_a, phi_b, 180 - phi_b)
    assert model.margin_at_point(*p) == pytest.approx(expected, abs=1e-9)
    assert expected == pytest.approx(180.0 - phi_b)  # the binding term at A8


# --------------------------------------------------------------------------- #
# Jacobian
# --------------------------------------------------------------------------- #


def _finite_difference_jacobian(model: FiveBarKinematics, alpha: float, beta: float, h: float = 1e-5) -> np.ndarray:
    d_alpha = (np.array(model.fk(alpha + h, beta)) - np.array(model.fk(alpha - h, beta))) / (2 * h)
    d_beta = (np.array(model.fk(alpha, beta + h)) - np.array(model.fk(alpha, beta - h))) / (2 * h)
    return np.column_stack((d_alpha, d_beta))


def test_jacobian_matches_finite_differences_on_all_squares(
    model: FiveBarKinematics, centers: dict[str, tuple[float, float]]
) -> None:
    for square, (x, y) in centers.items():
        alpha, beta = model.ik(x, y)
        analytic = model.jacobian(alpha, beta)
        numeric = _finite_difference_jacobian(model, alpha, beta)
        assert analytic.shape == (2, 2)
        assert np.allclose(analytic, numeric, rtol=1e-6, atol=1e-6), square


def test_jacobian_matches_finite_differences_off_the_squares(model: FiveBarKinematics) -> None:
    for x, y in [(0.0, 300.0), (-200.0, 250.0), (180.0, 330.0), (0.0, 120.0), (60.0, 430.0)]:
        alpha, beta = model.ik(x, y)
        assert np.allclose(
            model.jacobian(alpha, beta), _finite_difference_jacobian(model, alpha, beta), rtol=1e-6, atol=1e-6
        ), (x, y)


def test_jacobian_predicts_a_small_move(model: FiveBarKinematics) -> None:
    alpha, beta = model.ik(10.0, 300.0)
    d_alpha, d_beta = 0.01, -0.02  # degrees
    predicted = np.array(model.fk(alpha, beta)) + model.jacobian(alpha, beta) @ np.array([d_alpha, d_beta])
    actual = np.array(model.fk(alpha + d_alpha, beta + d_beta))
    assert np.allclose(predicted, actual, atol=1e-5)


def test_condition_number_is_the_2_norm_condition_of_the_jacobian(model: FiveBarKinematics) -> None:
    alpha, beta = model.ik(-108.5, 408.5)
    jac = model.jacobian(alpha, beta)
    singular_values = np.linalg.svd(jac, compute_uv=False)
    assert model.condition_number(alpha, beta) == pytest.approx(singular_values[0] / singular_values[1])


def test_condition_number_blows_up_at_a_stretched_arm(model: FiveBarKinematics) -> None:
    # PD = m + n: the left driving and driven links are collinear (phiA = 180).
    angle = math.radians(60.0)
    x = model.motor1_xy[0] + (model.m + model.n) * math.cos(angle)
    y = (model.m + model.n) * math.sin(angle)
    assert model.margin_at_point(x, y) == pytest.approx(0.0, abs=1e-6)
    alpha, beta = model.ik(x, y)
    assert model.condition_number(alpha, beta) > 1e8


# --------------------------------------------------------------------------- #
# Mirror symmetry about the Y axis
# --------------------------------------------------------------------------- #


def test_mirror_symmetry_on_all_squares(model: FiveBarKinematics, centers: dict[str, tuple[float, float]]) -> None:
    for square, (x, y) in centers.items():
        alpha, beta = model.ik(x, y)
        m_alpha, m_beta = model.ik(-x, y)
        assert m_alpha == pytest.approx(180.0 - beta, abs=1e-9), square
        assert m_beta == pytest.approx(180.0 - alpha, abs=1e-9), square
        assert model.fk(180.0 - beta, 180.0 - alpha) == pytest.approx((-x, y), abs=1e-9)
        assert model.margin_at_point(-x, y) == pytest.approx(model.margin_at_point(x, y), abs=1e-9)
        assert model.condition_number_at_point(-x, y) == pytest.approx(model.condition_number_at_point(x, y), rel=1e-9)


def test_mirror_symmetry_of_the_jacobian(model: FiveBarKinematics) -> None:
    for x, y in [(-108.5, 191.5), (37.0, 320.0), (120.0, 400.0)]:
        alpha, beta = model.ik(x, y)
        jac = model.jacobian(alpha, beta)
        mirrored = model.jacobian(180.0 - beta, 180.0 - alpha)
        expected = np.array([[jac[0, 1], jac[0, 0]], [-jac[1, 1], -jac[1, 0]]])
        assert np.allclose(mirrored, expected, atol=1e-9), (x, y)


def test_points_on_the_y_axis_have_alpha_plus_beta_equal_180(model: FiveBarKinematics) -> None:
    for y in (150.0, 300.0, 420.0):
        alpha, beta = model.ik(0.0, y)
        assert alpha + beta == pytest.approx(180.0, abs=1e-9)


# --------------------------------------------------------------------------- #
# FK errors
# --------------------------------------------------------------------------- #


def test_fk_raises_when_the_elbows_coincide(model: FiveBarKinematics) -> None:
    # A == B needs sin(alpha) = sin(beta) and cos(alpha) - cos(beta) = l / m.
    alpha = math.degrees(math.acos(model.l / (2.0 * model.m)))
    beta = 180.0 - alpha
    elbow_a, elbow_b = model.elbows(alpha, beta)
    assert math.dist(elbow_a, elbow_b) < 1e-9  # numerically zero, e.g. 1e-14
    with pytest.raises(InvalidPoseError):
        model.fk(alpha, beta)


def test_fk_raises_when_ab_is_not_below_2n(model: FiveBarKinematics) -> None:
    # alpha = 180, beta = 0 stretches the elbows apart by l + 2m, which is 2n for the reference build.
    assert model.l + 2.0 * model.m == pytest.approx(2.0 * model.n)
    with pytest.raises(InvalidPoseError):
        model.fk(180.0, 0.0)
    shorter = dataclasses.replace(model, n=(model.l + 2.0 * model.m) / 2.0 - 10.0)
    with pytest.raises(InvalidPoseError):
        shorter.fk(180.0, 0.0)


def test_fk_accepts_ab_just_inside_the_limits(model: FiveBarKinematics) -> None:
    just_inside = dataclasses.replace(model, n=(model.l + 2.0 * model.m) / 2.0 + 1.0)
    x, y = just_inside.fk(180.0, 0.0)
    assert math.isfinite(x) and math.isfinite(y)


@pytest.mark.parametrize("bad", [math.nan, math.inf, -math.inf])
def test_fk_and_ik_reject_non_finite_input(model: FiveBarKinematics, bad: float) -> None:
    with pytest.raises(KinematicsError):
        model.fk(bad, 60.0)
    with pytest.raises(KinematicsError):
        model.fk(60.0, bad)
    with pytest.raises(KinematicsError):
        model.ik(bad, 300.0)
    with pytest.raises(KinematicsError):
        model.ik(0.0, bad)


def test_jacobian_requires_a_valid_pose(model: FiveBarKinematics) -> None:
    with pytest.raises(InvalidPoseError):
        model.jacobian(180.0, 0.0)


# --------------------------------------------------------------------------- #
# IK errors and the lower reach bound
# --------------------------------------------------------------------------- #


@pytest.mark.parametrize(
    "point",
    [(0.0, 460.0), (0.0, 1000.0), (400.0, 300.0), (-400.0, 300.0), (0.0, -500.0), (-50.0, 10.0), (60.0, 5.0)],
)
def test_ik_raises_out_of_reach(model: FiveBarKinematics, point: tuple[float, float]) -> None:
    with pytest.raises(OutOfReachError):
        model.ik(*point)
    assert not model.is_reachable(*point)


def test_ik_raises_out_of_reach_when_only_one_branch_is_out(model: FiveBarKinematics) -> None:
    # Reachable from motor 2 (PC <= m + n) but not from motor 1 (PD > m + n).
    x, y = 430.0, 100.0
    assert math.hypot(x - model.l / 2, y) <= model.m + model.n < math.hypot(x + model.l / 2, y)
    with pytest.raises(OutOfReachError):
        model.ik(x, y)


def test_ik_lower_bound_is_abs_m_minus_n_not_the_vacuous_m_minus_n(model: FiveBarKinematics) -> None:
    # m - n is negative, so the SPEC's literal bound "m - n <= PD" never rejects anything.
    assert model.m - model.n < 0
    x, y = -model.l / 2, 10.0  # 10 mm from motor 1: PD = 10 < |m - n| = 50
    pd = math.hypot(x + model.l / 2, y)
    assert model.m - model.n <= pd < abs(model.m - model.n)  # passes the literal bound, fails the real one
    with pytest.raises(OutOfReachError):
        model.ik(x, y)


def test_ik_accepts_the_exact_reach_limits(model: FiveBarKinematics) -> None:
    # PD = m + n (stretched) is reachable (mu = 0, so not usable, but IK itself is defined).
    angle = math.radians(60.0)
    x = model.motor1_xy[0] + (model.m + model.n) * math.cos(angle)
    y = (model.m + model.n) * math.sin(angle)
    alpha, beta = model.ik(x, y)
    assert math.isfinite(alpha) and math.isfinite(beta)
    # PD = |m - n| exactly (folded): along the Y axis PD = PC = hypot(l/2, y).
    y_fold = math.sqrt(abs(model.m - model.n) ** 2 - (model.l / 2) ** 2) if abs(model.m - model.n) > model.l / 2 else None
    if y_fold is not None:
        assert model.is_reachable(0.0, y_fold)


# --------------------------------------------------------------------------- #
# Margin enforcement
# --------------------------------------------------------------------------- #


def test_min_margin_of_settings_is_15_degrees() -> None:
    assert MIN_SINGULARITY_MARGIN_DEG == 15.0


def test_ik_checked_accepts_every_square(model: FiveBarKinematics, centers: dict[str, tuple[float, float]]) -> None:
    for x, y in centers.values():
        assert model.ik_checked(x, y) == model.ik(x, y)


def test_ik_checked_raises_on_low_margin(model: FiveBarKinematics) -> None:
    # Reachable, but close to the far edge of the workspace: mu is about 11 degrees.
    x, y = 0.0, 445.0
    assert model.is_reachable(x, y)
    assert model.margin_at_point(x, y) < MIN_SINGULARITY_MARGIN_DEG
    with pytest.raises(SingularityError):
        model.ik_checked(x, y)


def test_ik_checked_margin_threshold_can_be_overridden(model: FiveBarKinematics) -> None:
    x, y = 0.0, 440.0  # mu is about 20.6 degrees
    mu = model.margin_at_point(x, y)
    assert MIN_SINGULARITY_MARGIN_DEG < mu < 25.0
    assert model.ik_checked(x, y, min_margin_deg=MIN_SINGULARITY_MARGIN_DEG) == model.ik(x, y)
    with pytest.raises(SingularityError):
        model.ik_checked(x, y, min_margin_deg=25.0)
    assert model.ik_checked(0.0, 445.0, min_margin_deg=5.0) == model.ik(0.0, 445.0)


def test_ik_checked_still_raises_out_of_reach(model: FiveBarKinematics) -> None:
    with pytest.raises(OutOfReachError):
        model.ik_checked(0.0, 500.0)


def test_error_hierarchy() -> None:
    for error in (InvalidPoseError, OutOfReachError, SingularityError):
        assert issubclass(error, KinematicsError)
    assert issubclass(KinematicsError, ValueError)


# --------------------------------------------------------------------------- #
# Angle to steps (SPEC 6.1: N = 200 * f * i * (angle - angle0) / 360)
# --------------------------------------------------------------------------- #


def test_full_turn_is_3200_steps_when_i_is_1(model: FiveBarKinematics, cfg: Settings) -> None:
    assert cfg.drive.gear_ratio == 1.0
    assert model.angle_to_steps(360.0) == 3200
    assert model.angle_to_steps(-360.0) == -3200
    assert model.angle_to_steps(180.0) == 1600
    assert model.angle_to_steps(0.0) == 0


def test_one_step_is_0_1125_degree_when_i_is_1(model: FiveBarKinematics) -> None:
    assert model.joint_resolution_deg == pytest.approx(0.1125)
    assert model.angle_to_steps(0.1125) == 1
    assert model.angle_to_steps(-0.1125) == -1
    assert model.angle_to_steps(0.05) == 0  # less than half a step
    assert model.angle_to_steps(0.0625) == 1  # more than half a step
    assert model.steps_to_angle(1) == pytest.approx(0.1125)


def test_steps_are_relative_to_angle0(model: FiveBarKinematics) -> None:
    assert model.angle_to_steps(90.0, angle0_deg=90.0) == 0
    assert model.angle_to_steps(90.1125, angle0_deg=90.0) == 1
    assert model.angle_to_steps(0.0, angle0_deg=90.0) == -800
    assert model.steps_to_angle(-800, angle0_deg=90.0) == pytest.approx(0.0)


def test_step_round_trip_error_is_below_half_a_step(model: FiveBarKinematics) -> None:
    for angle in (-4.3, 62.9, 83.5, 117.1, 184.3, 12.3456):
        back = model.steps_to_angle(model.angle_to_steps(angle))
        assert abs(back - angle) <= model.joint_resolution_deg / 2 + 1e-9


def test_joints_to_steps(model: FiveBarKinematics) -> None:
    assert model.joints_to_steps(90.0, 45.0) == (800, 400)
    assert model.joints_to_steps(100.0, 50.0, alpha0_deg=10.0, beta0_deg=5.0) == (800, 400)


def test_angle_to_steps_rejects_non_finite(model: FiveBarKinematics) -> None:
    with pytest.raises(KinematicsError):
        model.angle_to_steps(math.nan)
    with pytest.raises(KinematicsError):
        model.angle_to_steps(10.0, angle0_deg=math.inf)
