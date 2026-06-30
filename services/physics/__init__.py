"""Physics engine abstraction for the navigation backend.

`NavigationEngine` used to embed two engines behind ``if self._is_guided()``
branches: a kinematic centerline-follow path and the MuJoCo physics path. This
package extracts that fork into a single :class:`PhysicsEngine` seam with two
implementations (:class:`KinematicEngine`, :class:`MuJoCoEngine`), chosen once at
construction by :func:`make_engine`. ``NavigationEngine`` keeps only the
engine-agnostic orchestration (planned-path progress, curvature, safety, risk).
"""

from services.physics.base import (
    MAX_WALL_DISTANCE,
    PhysicsEngine,
    PlannedPath,
    RawPose,
    quat_from_direction,
)
from services.physics.factory import make_engine
from services.physics.kinematic_engine import KinematicEngine
from services.physics.mujoco_engine import MuJoCoEngine
from services.physics.newton_engine import NewtonEngine

__all__ = [
    "MAX_WALL_DISTANCE",
    "PhysicsEngine",
    "PlannedPath",
    "RawPose",
    "quat_from_direction",
    "make_engine",
    "KinematicEngine",
    "MuJoCoEngine",
    "NewtonEngine",
]
