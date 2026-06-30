"""Engine factory: pick the physics backend once, at construction.

``NavigationEngine`` resolves the planned path and entry pose, then calls
:func:`make_engine` to get exactly one :class:`~services.physics.base.PhysicsEngine`.
This replaces the former runtime ``if self._is_guided()`` fork: the choice is now
made once, by data, and the orchestrator holds a single engine.
"""

from __future__ import annotations

import os
from typing import Sequence

from services.physics.base import PhysicsEngine, PlannedPath
from services.physics.kinematic_engine import KinematicEngine
from services.physics.mujoco_engine import MuJoCoEngine


def make_engine(
    *,
    guided: bool,
    path: PlannedPath | None,
    phantom: str,
    target: str,
    use_pixels: bool = False,
    image_size: int = 80,
    assets_dir: str | None = None,
    n_bodies: int = 80,
    n_substeps: int | None = None,
    insertion_max: float = 0.2,
    stiffness_scale: float = 1.0,
    prethread: bool = False,
    entry_point: Sequence[float] | None = None,
    entry_direction: Sequence[float] | None = None,
    advance_per_step: float = 0.01,
    wall_lean: float = 0.0025,
) -> PhysicsEngine:
    """Construct the appropriate physics backend.

    Kinematic centerline-follow is chosen only when ``guided`` is requested *and*
    a usable planned path exists (>= 2 points, positive length); otherwise the
    physical MuJoCo simulation is used. This mirrors the pre-refactor
    ``_is_guided()`` gate so ``guided=True`` without a path falls back to physics.
    """
    if os.environ.get("CATHSIM_PHYSICS_ENGINE", "").lower() in {"newton", "newton_demo"}:
        from services.physics.newton_engine import NewtonEngine

        return NewtonEngine(
            path=path,
            n_substeps=n_substeps,
            entry_point=entry_point,
            entry_direction=entry_direction,
        )

    if guided and path is not None and path.total_len > 0.0:
        return KinematicEngine(
            path=path,
            advance_per_step=advance_per_step,
            wall_lean=wall_lean,
        )

    return MuJoCoEngine(
        phantom=phantom,
        target=target,
        use_pixels=use_pixels,
        image_size=image_size,
        assets_dir=assets_dir,
        n_bodies=n_bodies,
        n_substeps=n_substeps,
        insertion_max=insertion_max,
        stiffness_scale=stiffness_scale,
        prethread=prethread,
        path=path,
        entry_point=entry_point,
        entry_direction=entry_direction,
    )
