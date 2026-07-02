"""ShapeIntent control abstraction: one command interface for Human and RL.

doc/09 (§一/§五) draws the boundary sharply: the *physics* engine stays pure
force-driven (D4), and everything above it -- a doctor at the keyboard, a click
on the vessel, or an RL policy -- issues the **same** high-level command, a
:class:`ShapeIntent`. A :class:`ShapeIntentController` resolves that intent
against the current physical state into a real ``(push, rotate)`` pair, exactly
the 2-DOF the guidewire actually has.

Crucially this is *not* a per-segment soft-constraint position field (that would
be a regression to the D3 graded soft-anchor kinematic teleport, destroying the
D4 force transmission / tip lag / real contact). The controller is a thin
generalisation of the already-validated :class:`~services.physics_autopilot.PhysicsAutopilot`
control law: it only changes *where the tip aims*, never how the physics runs.

``intent=None`` degenerates to the existing centerline-following autopilot,
byte-for-byte -- so wiring this in front of the autopilot is behaviour-preserving.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from services.physics_autopilot import AutopilotConfig, PhysicsAutopilot


@dataclass
class ShapeIntent:
    """A high-level steering command shared by Human UI and RL policy.

    All fields are world-space (MuJoCo meters). Exactly one of the two target
    forms is used per step; if both are set, ``target_direction`` wins.

    * ``target_direction`` -- desired tip heading (unit vector; unnormalized ok).
      Source: a gamepad stick, or an RL policy head.
    * ``target_waypoint`` -- a point the tip should reach. Source: a UI click
      projected onto the centerline, or an RL waypoint plan.
    * ``intensity`` -- insertion strength in [0, 1]; scales the aligned push.

    There is deliberately **no** field that sets segment positions directly --
    that is kinematic override and is forbidden (doc/09 §一, 结论1).
    """

    target_direction: np.ndarray | None = None
    target_waypoint: np.ndarray | None = None
    intensity: float = 1.0

    def __post_init__(self) -> None:
        if self.target_direction is not None:
            self.target_direction = np.asarray(self.target_direction, dtype=np.float64)
            if self.target_direction.shape != (3,):
                raise ValueError("target_direction must be a length-3 vector")
        if self.target_waypoint is not None:
            self.target_waypoint = np.asarray(self.target_waypoint, dtype=np.float64)
            if self.target_waypoint.shape != (3,):
                raise ValueError("target_waypoint must be a length-3 point")
        self.intensity = float(np.clip(self.intensity, 0.0, 1.0))


class ShapeIntentController:
    """Resolve a :class:`ShapeIntent` (or none) into real ``(push, rotate)``.

    This is the ``PhysicsAutopilot`` generalised to accept an external target,
    per doc/09 §五. Human, RL and centerline-autopilot all funnel through here;
    only *who produces the intent* differs -- the controller and the physics
    engine are identical across the three modes.

    Usage::

        ctl = ShapeIntentController(path_points)
        ctl.reset()
        push, rotate = ctl.compute(intent, tip_pos, tip_dir, contact_force)

    ``path_points`` is the same planned centerline the engine tracks; it is
    still needed for stall/progress bookkeeping and as the ``intent=None``
    fallback target.
    """

    def __init__(self, path_points, config: AutopilotConfig | None = None):
        self._autopilot = PhysicsAutopilot(path_points, config)

    @property
    def config(self) -> AutopilotConfig:
        return self._autopilot.config

    @property
    def deepest_arclen(self) -> float:
        return self._autopilot.deepest_arclen

    def reset(self) -> None:
        self._autopilot.reset()

    def compute(
        self,
        intent: ShapeIntent | None,
        tip_pos,
        tip_dir,
        contact_force: float = 0.0,
    ) -> tuple[float, float]:
        """Return real ``(push, rotate)`` in [-1, 1] for the given intent.

        ``intent=None`` -> centerline look-ahead autopilot (unchanged).
        Otherwise the intent's target redirects the aim while reusing the same
        look-ahead heading / J-tip hill-climb / force-gated push / stall law.
        """
        if intent is None:
            return self._autopilot.compute(tip_pos, tip_dir, contact_force)

        desired_override = None
        aim_override = None
        if intent.target_direction is not None:
            desired_override = intent.target_direction
        elif intent.target_waypoint is not None:
            aim_override = intent.target_waypoint

        return self._autopilot.compute(
            tip_pos,
            tip_dir,
            contact_force,
            aim_override=aim_override,
            desired_override=desired_override,
            push_scale=intent.intensity,
        )
