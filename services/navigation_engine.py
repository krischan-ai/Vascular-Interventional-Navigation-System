"""Navigation Engine: Bridge between FastAPI and CathSim MuJoCo environment.

This module provides a high-level interface for controlling the CathSim
guidewire simulation through the NavigationEngine class.
"""

from __future__ import annotations

import time
from dataclasses import dataclass, field
from typing import Any, Literal

import numpy as np


@dataclass
class NavigationState:
    """Normalized state representation from CathSim environment."""

    tip_position: list[float] = field(default_factory=lambda: [0.0, 0.0, 0.0])
    tip_direction: list[float] = field(default_factory=lambda: [0.0, 0.0, 1.0])
    velocity: float = 0.0
    contact_force: float = 0.0
    episode_length: int = 0
    target_position: list[float] = field(default_factory=lambda: [0.0, 0.0, 0.0])
    joint_positions: list[float] = field(default_factory=list)
    joint_velocities: list[float] = field(default_factory=list)
    reward: float = 0.0
    done: bool = False

    def as_dict(self) -> dict[str, Any]:
        """Convert state to dictionary for JSON serialization."""
        return {
            "tip_position": self.tip_position,
            "tip_direction": self.tip_direction,
            "velocity": self.velocity,
            "contact_force": self.contact_force,
            "episode_length": self.episode_length,
            "target_position": self.target_position,
            "joint_positions": self.joint_positions,
            "joint_velocities": self.joint_velocities,
            "reward": self.reward,
            "done": self.done,
        }


SafetyStatus = Literal["STANDBY", "SAFE_NAV", "DANGER_WARNING", "COLLISION_STOP"]


class NavigationEngine:
    """High-level interface for CathSim guidewire navigation.

    This class wraps the CathSim dm_control environment and provides:
    - Simplified step/reset interface
    - Normalized state extraction
    - Safety status monitoring

    Example:
        engine = NavigationEngine(phantom="low_tort", target="bca")
        state = engine.reset()
        state = engine.step(delta_push=0.5, delta_rotate=0.1)
    """

    VALID_PHANTOMS = ("low_tort", "phantom2", "phantom3", "phantom4")
    VALID_TARGETS = ("bca", "lcca")

    def __init__(
        self,
        phantom: str = "low_tort",
        target: str = "bca",
        use_pixels: bool = False,
        image_size: int = 80,
    ):
        """Initialize the navigation engine.

        Args:
            phantom: Phantom model name (low_tort, phantom2, phantom3, phantom4)
            target: Target site name (bca, lcca)
            use_pixels: Whether to include pixel observations
            image_size: Image size for pixel observations
        """
        self.phantom = phantom
        self.target = target
        self.use_pixels = use_pixels
        self.image_size = image_size

        self._env = None
        self._time_step = None
        self._episode_length = 0
        self._previous_tip_pos = None
        self._initialized = False

    def _ensure_initialized(self) -> None:
        """Lazy initialization of CathSim environment."""
        if self._initialized:
            return

        from cathsim.dm import make_dm_env

        self._env = make_dm_env(
            phantom=self.phantom,
            target=self.target,
            use_pixels=self.use_pixels,
            image_size=self.image_size,
            visualize_sites=False,
            visualize_target=False,
            sample_target=False,
        )
        self._initialized = True

    def reset(self) -> NavigationState:
        """Reset the environment and return initial state.

        Returns:
            NavigationState with initial positions and zeroed dynamics
        """
        self._ensure_initialized()

        self._time_step = self._env.reset()
        self._episode_length = 0
        self._previous_tip_pos = None

        return self._extract_state()

    def step(self, delta_push: float, delta_rotate: float) -> NavigationState:
        """Execute one simulation step.

        Args:
            delta_push: Push force coefficient [-1.0, 1.0], positive = forward
            delta_rotate: Rotation force coefficient [-1.0, 1.0], positive = clockwise

        Returns:
            NavigationState after the step
        """
        if not self._initialized or self._time_step is None:
            raise RuntimeError("Engine not initialized. Call reset() first.")

        delta_push = float(np.clip(delta_push, -1.0, 1.0))
        delta_rotate = float(np.clip(delta_rotate, -1.0, 1.0))

        action = np.array([delta_push, delta_rotate], dtype=np.float64)
        self._time_step = self._env.step(action)
        self._episode_length += 1

        return self._extract_state()

    def _extract_state(self) -> NavigationState:
        """Extract normalized state from current time_step."""
        obs = self._time_step.observation
        physics = self._env.physics
        task = self._env.task

        tip_pos = task.get_head_pos(physics).tolist()

        tip_direction = self._compute_tip_direction(physics)

        velocity = self._compute_velocity(tip_pos)

        contact_force = task.get_total_force(physics)

        target_pos = task.target_pos
        if isinstance(target_pos, np.ndarray):
            target_pos = target_pos.tolist()

        joint_pos = obs.get("joint_pos", np.array([])).tolist()
        joint_vel = obs.get("joint_vel", np.array([])).tolist()

        reward = self._time_step.reward if self._time_step.reward is not None else 0.0
        done = self._time_step.last()

        return NavigationState(
            tip_position=tip_pos,
            tip_direction=tip_direction,
            velocity=float(velocity),
            contact_force=float(contact_force),
            episode_length=self._episode_length,
            target_position=target_pos,
            joint_positions=joint_pos,
            joint_velocities=joint_vel,
            reward=float(reward),
            done=done,
        )

    def _compute_tip_direction(self, physics) -> list[float]:
        """Compute tip direction from the last two geom positions."""
        geom_xpos = physics.data.geom_xpos
        if geom_xpos.shape[0] < 2:
            return [0.0, 0.0, 1.0]

        tip_pos = geom_xpos[-1]
        prev_pos = geom_xpos[-2]
        direction = tip_pos - prev_pos
        norm = np.linalg.norm(direction)
        if norm > 1e-8:
            direction = direction / norm
        else:
            direction = np.array([0.0, 0.0, 1.0])
        return direction.tolist()

    def _compute_velocity(self, tip_pos: list[float]) -> float:
        """Compute tip velocity from position change."""
        if self._previous_tip_pos is None:
            self._previous_tip_pos = tip_pos
            return 0.0

        delta = np.array(tip_pos) - np.array(self._previous_tip_pos)
        velocity = np.linalg.norm(delta)
        self._previous_tip_pos = tip_pos

        control_timestep = getattr(self._env.task, "control_timestep", 0.02)
        velocity_per_second = velocity / control_timestep if control_timestep > 0 else 0.0

        return velocity_per_second

    def get_safety_status(self, state: NavigationState) -> SafetyStatus:
        """Determine safety status based on current state.

        Args:
            state: Current navigation state

        Returns:
            Safety status string
        """
        if self._episode_length == 0:
            return "STANDBY"

        if state.contact_force > 0.5:
            return "COLLISION_STOP"

        if state.contact_force > 0.1:
            return "DANGER_WARNING"

        return "SAFE_NAV"

    def close(self) -> None:
        """Clean up resources."""
        if self._env is not None:
            del self._env
            self._env = None
        self._initialized = False
        self._time_step = None

    def __del__(self):
        self.close()

    @property
    def is_initialized(self) -> bool:
        """Check if engine is initialized."""
        return self._initialized

    @property
    def episode_length(self) -> int:
        """Current episode length."""
        return self._episode_length
