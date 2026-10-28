"""Five-bar kinematics: FK, IK, Jacobian, singularity margin, angle-to-steps.

Implements docs/SPEC.md section 6.1 and 6.2 as pure NumPy and math: no Qt, no
serial, no imports from gui/, hardware/, core/ or vision/.

Conventions
-----------
* Lengths are in millimetres, angles in degrees (public API).
* Origin O is the midpoint of the two motor axes, +Y points toward the board.
  Motor #1 (angle alpha) is at (-l/2, 0), motor #2 (angle beta) at (+l/2, 0).
* The elbows-out configuration is the only one supported.
* Every constant (l, m, n and the drive chain f, i) and the singularity margin come from
  config/settings.yaml through :mod:`desktop_app.utils.settings`;
  nothing hardware-specific is hard-coded here.

Jacobian
--------
Let A and B be the elbows and P the gripper. The two distance constraints are
``|P - A|^2 = n^2`` and ``|P - B|^2 = n^2``. Differentiating both gives

    (P - A) . dP = (P - A) . dA,        dA = m * (-sin(alpha), cos(alpha)) * d(alpha)
    (P - B) . dP = (P - B) . dB,        dB = m * (-sin(beta),  cos(beta))  * d(beta)

that is ``Jx dP = Jq dq`` with ``Jx = [[(P-A)^T], [(P-B)^T]]`` and
``Jq = diag((P-A).tA, (P-B).tB)`` where ``tA`` and ``tB`` are the elbow velocity
vectors per radian. Hence ``J = dP/dq = Jx^-1 Jq``. ``Jx`` is singular when
angle APB is 0 or 180 degrees; ``Jq`` is singular when a driving and driven link
of the same branch are collinear (phi = 0 or 180 degrees).
"""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

from desktop_app.utils.settings import Settings, get_settings

# Numerical-zero threshold for the elbow distance AB (mm). An AB this small means
# both elbows coincide, which is not a valid pose (the perpendicular direction
# (yA - yB, xB - xA) / AB is undefined).
_AB_EPSILON_MM = 1e-9
# Relative threshold under which the Jacobian rows are treated as parallel.
_DET_REL_EPSILON = 1e-12


class KinematicsError(ValueError):
    """Base class of every error raised by the kinematics package."""


class InvalidPoseError(KinematicsError):
    """FK was asked for joint angles that do not give a valid pose."""


class OutOfReachError(KinematicsError):
    """IK was asked for a point the arm cannot reach."""


class SingularityError(KinematicsError):
    """A pose is too close to a singularity (mu below the required margin)."""


Point = tuple[float, float]


def _angle_between_deg(u: np.ndarray, v: np.ndarray) -> float:
    """Unsigned angle between two 2-D vectors in [0, 180] degrees (atan2 form, no acos clipping)."""
    cross = u[0] * v[1] - u[1] * v[0]
    dot = u[0] * v[0] + u[1] * v[1]
    return math.degrees(math.atan2(abs(cross), dot))


def _round_half_away(value: float) -> int:
    return int(math.copysign(math.floor(abs(value) + 0.5), value))


def _require_finite(**values: float) -> None:
    for name, value in values.items():
        if not math.isfinite(value):
            raise KinematicsError(f"{name} must be a finite number, got {value!r}")


@dataclass(frozen=True)
class FiveBarKinematics:
    """Kinematic model of the five-bar arm.

    Attributes:
        l: distance between the two motor axes (mm).
        m: driving link length (mm).
        n: driven link length (mm).
        pulses_per_rev: microsteps per revolution of the driving link, 200 * f * i.
    """

    l: float
    m: float
    n: float
    pulses_per_rev: float

    def __post_init__(self) -> None:
        for name in ("l", "m", "n", "pulses_per_rev"):
            value = getattr(self, name)
            if not (math.isfinite(value) and value > 0):
                raise KinematicsError(f"{name} must be a finite number > 0, got {value!r}")

    # ------------------------------------------------------------------ #
    # Construction
    # ------------------------------------------------------------------ #

    @classmethod
    def from_config(cls, config: Settings | None = None) -> "FiveBarKinematics":
        """Build the model from the settings (default: config/settings.yaml)."""
        cfg = config if config is not None else get_settings()
        return cls(
            l=cfg.geometry.motor_axis_distance_mm,
            m=cfg.geometry.driving_link_mm,
            n=cfg.geometry.driven_link_mm,
            pulses_per_rev=cfg.drive.pulses_per_rev,
        )

    # ------------------------------------------------------------------ #
    # Basic geometry
    # ------------------------------------------------------------------ #

    @property
    def motor1_xy(self) -> Point:
        return (-self.l / 2.0, 0.0)

    @property
    def motor2_xy(self) -> Point:
        return (self.l / 2.0, 0.0)

    @property
    def joint_resolution_deg(self) -> float:
        """Joint angle resolution: 0.1125 degrees when 200 * f * i = 3200."""
        return 360.0 / self.pulses_per_rev

    def _elbows(self, alpha_deg: float, beta_deg: float) -> tuple[np.ndarray, np.ndarray]:
        a = math.radians(alpha_deg)
        b = math.radians(beta_deg)
        elbow_a = np.array([-self.l / 2.0 + self.m * math.cos(a), self.m * math.sin(a)])
        elbow_b = np.array([self.l / 2.0 + self.m * math.cos(b), self.m * math.sin(b)])
        return elbow_a, elbow_b

    def elbows(self, alpha_deg: float, beta_deg: float) -> tuple[Point, Point]:
        """Elbow positions A and B for the given joint angles (SPEC 6.1)."""
        _require_finite(alpha_deg=alpha_deg, beta_deg=beta_deg)
        elbow_a, elbow_b = self._elbows(alpha_deg, beta_deg)
        return (float(elbow_a[0]), float(elbow_a[1])), (float(elbow_b[0]), float(elbow_b[1]))

    # ------------------------------------------------------------------ #
    # Forward kinematics
    # ------------------------------------------------------------------ #

    def _fk(self, alpha_deg: float, beta_deg: float) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
        _require_finite(alpha_deg=alpha_deg, beta_deg=beta_deg)
        elbow_a, elbow_b = self._elbows(alpha_deg, beta_deg)
        delta = elbow_b - elbow_a
        ab = math.hypot(delta[0], delta[1])
        # SPEC 6.1: require 0 < AB < 2n. An AB that is numerically zero (both
        # elbows coincide, e.g. 1e-14) is invalid as well.
        if not (_AB_EPSILON_MM < ab < 2.0 * self.n):
            raise InvalidPoseError(
                f"FK needs 0 < AB < 2n, got AB = {ab:.12g} mm for alpha = {alpha_deg}, beta = {beta_deg} "
                f"(2n = {2.0 * self.n:g} mm)"
            )
        w_squared = self.n**2 - ab**2 / 4.0
        if w_squared <= 0.0:
            raise InvalidPoseError(f"FK needs AB < 2n, got AB = {ab:.12g} mm (2n = {2.0 * self.n:g} mm)")
        w = math.sqrt(w_squared)
        # Perpendicular (yA - yB, xB - xA) / AB: elbows-out side of the line AB.
        perpendicular = np.array([-delta[1], delta[0]]) / ab
        gripper = (elbow_a + elbow_b) / 2.0 + w * perpendicular
        return elbow_a, elbow_b, gripper

    def fk(self, alpha_deg: float, beta_deg: float) -> Point:
        """Gripper position P (mm) for joint angles alpha, beta (degrees).

        Raises:
            InvalidPoseError: unless 0 < AB < 2n (AB numerically zero is invalid too).
        """
        _, _, gripper = self._fk(alpha_deg, beta_deg)
        return (float(gripper[0]), float(gripper[1]))

    # ------------------------------------------------------------------ #
    # Inverse kinematics
    # ------------------------------------------------------------------ #

    def ik(self, x: float, y: float) -> tuple[float, float]:
        """Joint angles (alpha, beta) in degrees for a gripper point, elbows-out only.

        Raises:
            OutOfReachError: when the point is out of reach of either branch.
        """
        _require_finite(x=x, y=y)
        pd = math.hypot(x + self.l / 2.0, y)
        pc = math.hypot(x - self.l / 2.0, y)
        # SPEC 6.1 writes the lower bound as m - n. Here m < n, so m - n is
        # negative and that bound is vacuous (it would admit points closer to a
        # motor axis than the folded arm can reach). The real lower bound is
        # |m - n|, which is what is used.
        lower = abs(self.m - self.n)
        upper = self.m + self.n
        for name, distance in (("PD", pd), ("PC", pc)):
            if not (lower <= distance <= upper) or distance <= 0.0:
                raise OutOfReachError(
                    f"point ({x:g}, {y:g}) is out of reach: {name} = {distance:.6g} mm "
                    f"is outside [{lower:g}, {upper:g}] mm"
                )
        alpha = math.atan2(y, x + self.l / 2.0) + math.acos(
            self._clip_unit((self.m**2 - self.n**2 + pd**2) / (2.0 * self.m * pd))
        )
        beta = math.atan2(y, x - self.l / 2.0) - math.acos(
            self._clip_unit((self.m**2 - self.n**2 + pc**2) / (2.0 * self.m * pc))
        )
        return (math.degrees(alpha), math.degrees(beta))

    @staticmethod
    def _clip_unit(value: float) -> float:
        # Boundary points land on +-1 up to rounding; clip only that rounding.
        return max(-1.0, min(1.0, value))

    def is_reachable(self, x: float, y: float) -> bool:
        """True when :meth:`ik` would succeed for the point."""
        try:
            self.ik(x, y)
        except KinematicsError:
            return False
        return True

    # ------------------------------------------------------------------ #
    # Jacobian and condition number
    # ------------------------------------------------------------------ #

    def jacobian(self, alpha_deg: float, beta_deg: float) -> np.ndarray:
        """Analytic Jacobian dP/dq, a 2x2 matrix in mm per degree.

        Row 0 is dx, row 1 is dy; column 0 is d(alpha), column 1 is d(beta).
        See the module docstring for the derivation.

        Raises:
            InvalidPoseError: the pose is not valid for FK.
            SingularityError: the rows of Jx are parallel (angle APB is 0 or 180).
        """
        elbow_a, elbow_b, gripper = self._fk(alpha_deg, beta_deg)
        a = math.radians(alpha_deg)
        b = math.radians(beta_deg)
        velocity_a = self.m * np.array([-math.sin(a), math.cos(a)])  # dA / d(alpha), mm per rad
        velocity_b = self.m * np.array([-math.sin(b), math.cos(b)])  # dB / d(beta), mm per rad
        pa = gripper - elbow_a
        pb = gripper - elbow_b
        jx = np.array([pa, pb])
        if abs(np.linalg.det(jx)) <= _DET_REL_EPSILON * self.n**2:
            raise SingularityError(
                f"Jacobian is singular: elbows and gripper are collinear (alpha = {alpha_deg}, beta = {beta_deg})"
            )
        jq = np.diag([float(pa @ velocity_a), float(pb @ velocity_b)])
        return np.linalg.solve(jx, jq) * (math.pi / 180.0)

    def condition_number(self, alpha_deg: float, beta_deg: float) -> float:
        """2-norm condition number of the Jacobian (unit-independent; inf at a singularity)."""
        return float(np.linalg.cond(self.jacobian(alpha_deg, beta_deg)))

    # ------------------------------------------------------------------ #
    # Singularity margin
    # ------------------------------------------------------------------ #

    @staticmethod
    def _margin(elbow_a: np.ndarray, elbow_b: np.ndarray, gripper: np.ndarray, motor1: Point, motor2: Point) -> float:
        apb = _angle_between_deg(elbow_a - gripper, elbow_b - gripper)
        phi_a = _angle_between_deg(np.asarray(motor1) - elbow_a, gripper - elbow_a)
        phi_b = _angle_between_deg(np.asarray(motor2) - elbow_b, gripper - elbow_b)
        return min(apb, 180.0 - apb, phi_a, 180.0 - phi_a, phi_b, 180.0 - phi_b)

    def singularity_margin(self, alpha_deg: float, beta_deg: float) -> float:
        """mu = min(APB, 180 - APB, phiA, 180 - phiA, phiB, 180 - phiB) in degrees (SPEC 6.2).

        phi is the angle between the driving and the driven link of the same branch.
        """
        elbow_a, elbow_b, gripper = self._fk(alpha_deg, beta_deg)
        return self._margin(elbow_a, elbow_b, gripper, self.motor1_xy, self.motor2_xy)

    def margin_at_point(self, x: float, y: float) -> float:
        """Singularity margin mu at a gripper point (via IK; raises OutOfReachError)."""
        alpha, beta = self.ik(x, y)
        elbow_a, elbow_b = self._elbows(alpha, beta)
        return self._margin(elbow_a, elbow_b, np.array([x, y], dtype=float), self.motor1_xy, self.motor2_xy)

    def condition_number_at_point(self, x: float, y: float) -> float:
        """Jacobian condition number at a gripper point (via IK)."""
        return self.condition_number(*self.ik(x, y))

    def ik_checked(self, x: float, y: float, min_margin_deg: float | None = None) -> tuple[float, float]:
        """IK that also enforces ``mu >= min_margin_deg`` (default: motion.singularity_margin_deg of the settings).

        Raises:
            OutOfReachError: the point is out of reach.
            SingularityError: the point is reachable but mu is below the margin.
        """
        if min_margin_deg is None:
            min_margin_deg = get_settings().motion.singularity_margin_deg
        alpha, beta = self.ik(x, y)
        elbow_a, elbow_b = self._elbows(alpha, beta)
        mu = self._margin(elbow_a, elbow_b, np.array([x, y], dtype=float), self.motor1_xy, self.motor2_xy)
        if mu < min_margin_deg:
            raise SingularityError(
                f"point ({x:g}, {y:g}) has singularity margin mu = {mu:.3f} deg, below the required {min_margin_deg:g} deg"
            )
        return (alpha, beta)

    # ------------------------------------------------------------------ #
    # Angle to steps (SPEC 6.1: N = 200 * f * i * (angle - angle0) / 360)
    # ------------------------------------------------------------------ #

    def angle_to_steps(self, angle_deg: float, angle0_deg: float = 0.0) -> int:
        """Microsteps for a joint angle measured from ``angle0_deg`` (rounded to the nearest step).

        ``angle0_deg`` is the joint angle of the step-zero reference (home). The
        SPEC does not fix it, so it is an explicit argument rather than a constant.
        """
        _require_finite(angle_deg=angle_deg, angle0_deg=angle0_deg)
        return _round_half_away(self.pulses_per_rev * (angle_deg - angle0_deg) / 360.0)

    def steps_to_angle(self, steps: float, angle0_deg: float = 0.0) -> float:
        """Inverse of :meth:`angle_to_steps` (degrees)."""
        return angle0_deg + steps * 360.0 / self.pulses_per_rev

    def joints_to_steps(
        self, alpha_deg: float, beta_deg: float, alpha0_deg: float = 0.0, beta0_deg: float = 0.0
    ) -> tuple[int, int]:
        """Step counts for both joints."""
        return (
            self.angle_to_steps(alpha_deg, alpha0_deg),
            self.angle_to_steps(beta_deg, beta0_deg),
        )
