"""KinematicEngine: centerline-follow guidewire motion without physics.

Drives the guidewire tip along the planned centerline by an insertion-depth
parameter (arc length ``_s``): ``push`` advances/retracts it so the tip reliably
reaches the target. No contact is modeled (wall distance is the free-space
sentinel). This is the former ``NavigationEngine._guided_*`` code, lifted behind
the :class:`~services.physics.base.PhysicsEngine` seam.

Required for full-length VPP vessels (path ~1.1m) that the physical guidewire
(~0.08m, 0.2m insertion cap) cannot traverse.
"""

from __future__ import annotations

import numpy as np

from services.physics.base import (
    MAX_WALL_DISTANCE,
    PlannedPath,
    RawPose,
    quat_from_direction,
)


class KinematicEngine:
    """Kinematic centerline-follow engine implementing the PhysicsEngine seam."""

    def __init__(
        self,
        path: PlannedPath,
        advance_per_step: float = 0.01,
        wall_lean: float = 0.0025,
        wall_lean_gain: float = 1.5,
    ) -> None:
        """Initialize the engine.

        Args:
            path: Planned centerline geometry the tip rides along.
            advance_per_step: Arc length advanced per unit push (meters);
                ``push=1.0`` advances this much.
            wall_lean: Max offset (meters) toward the inner side of curves in the
                render, so the wire hugs the inner vessel wall at bends. 0
                disables (pure centerline).
            wall_lean_gain: Curvature-to-offset gain before the wall_lean cap.
        """
        self._path = path
        self._advance_per_step = float(advance_per_step)
        self._wall_lean = float(wall_lean)
        self._wall_lean_gain = float(wall_lean_gain)
        self._s = 0.0  # current insertion depth as arc length along the path (m)

    @property
    def control_timestep(self) -> float:
        # No MuJoCo env; assume a nominal ~30Hz control step for velocity.
        return 0.033

    def reset(self) -> RawPose:
        """Reset the insertion depth to the entry and return the initial pose."""
        self._s = 0.0
        return self._raw_pose()

    def step(self, push: float, rotate: float) -> RawPose:
        """Advance/retract the insertion depth along the path.

        ``rotate`` has no kinematic effect (the tip rides the centerline); it is
        accepted for interface parity with the physics engine.
        """
        self._s = float(
            np.clip(
                self._s + push * self._advance_per_step,
                0.0,
                self._path.total_len,
            )
        )
        return self._raw_pose()

    def _raw_pose(self) -> RawPose:
        """Synthesize a RawPose from the current insertion depth.

        The tip rides the planned path at arc length ``self._s``; deviation is
        zero by construction and ``arclen`` is reported so NavigationEngine
        derives an exact, continuous progress.
        """
        tip_dir = self._path.tangent_at_arclen(self._s)
        done = self._path.total_len > 0.0 and self._s >= 0.999 * self._path.total_len
        return RawPose(
            tip_position=[float(v) for v in self._path.point_at_arclen(self._s)],
            tip_direction=[float(v) for v in tip_dir],
            tip_quaternion=quat_from_direction(tip_dir),
            contact_force=0.0,
            wall_distance=MAX_WALL_DISTANCE,
            target_position=[float(v) for v in self._path.points[-1]],
            joint_positions=[],
            joint_velocities=[],
            reward=0.0,
            done=done,
            arclen=self._s,
        )

    def render_bodies(self) -> list[dict[str, list[float]]]:
        """Sample the full inserted guidewire (entry -> tip) along the centerline.

        Renders the entire inserted length so the wire visibly curves through
        every vessel bend (not just a short tip stub), leaning each point toward
        the inner side of curves so the shape reads as a real bending guidewire.
        """
        s_tip = self._s
        if s_tip <= 1e-6:
            pos = self._path.point_at_arclen(0.0)
            quat = quat_from_direction(self._path.tangent_at_arclen(0.0))
            return [{"pos": [float(v) for v in pos], "quat": quat}]

        spacing = 0.004  # ~4mm between render segments
        n = int(np.clip(int(s_tip / spacing) + 1, 2, 256))
        bodies: list[dict[str, list[float]]] = []
        for i in range(n + 1):
            s = s_tip * i / n
            pos = self._path.point_at_arclen(s)
            if self._wall_lean > 0.0:
                pos = pos + self._inner_wall_offset(s)
            quat = quat_from_direction(self._path.tangent_at_arclen(s))
            bodies.append({"pos": [float(v) for v in pos], "quat": quat})
        return bodies

    def _inner_wall_offset(self, s: float) -> np.ndarray:
        """Inner-wall lean offset at arc length ``s`` for this engine's config."""
        return self._path.inner_wall_offset(s, self._wall_lean, self._wall_lean_gain)

    def close(self) -> None:
        """No resources to release."""
        return None
