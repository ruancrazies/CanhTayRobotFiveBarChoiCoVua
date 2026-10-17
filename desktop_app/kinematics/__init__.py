"""Five-bar kinematics, trajectories and board geometry (phase 1)."""

from desktop_app.kinematics.dimension import (
    BoardExtent,
    BoardGeometry,
    Workspace,
    ZoneNotDefinedError,
    ZoneValidationError,
    Zones,
)
from desktop_app.kinematics.five_bar import (
    MIN_SINGULARITY_MARGIN_DEG,
    FiveBarKinematics,
    InvalidPoseError,
    KinematicsError,
    OutOfReachError,
    SingularityError,
)
from desktop_app.kinematics.trajectory import (
    DEFAULT_MAX_SEGMENT_MM,
    Trajectory,
    TrajectoryError,
    straight_line,
    straight_line_points,
)

__all__ = [
    "DEFAULT_MAX_SEGMENT_MM",
    "MIN_SINGULARITY_MARGIN_DEG",
    "BoardExtent",
    "BoardGeometry",
    "FiveBarKinematics",
    "InvalidPoseError",
    "KinematicsError",
    "OutOfReachError",
    "SingularityError",
    "Trajectory",
    "TrajectoryError",
    "Workspace",
    "ZoneNotDefinedError",
    "ZoneValidationError",
    "Zones",
    "straight_line",
    "straight_line_points",
]
