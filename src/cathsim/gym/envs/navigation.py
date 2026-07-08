from __future__ import annotations

from typing import Any, Literal

import gymnasium as gym
import numpy as np
from gymnasium import spaces


ActionMode = Literal["shape_intent", "direct"]


class NavigationGymEnv(gym.Env):
    """Gymnasium environment over NavigationEngine + ShapeIntent.

    This is the T0 training surface: human HCI and RL both command the same
    NavigationEngine. In ``shape_intent`` mode the action is
    ``[target_direction.xyz, intensity]``; the controller resolves it into real
    push/rotate before the physics backend steps.
    """

    metadata = {"render_modes": [None], "render_fps": 60}

    def __init__(
        self,
        phantom: str = "aorta_tree",
        route_target: str | None = "endpoint_0",
        target: str = "root",
        action_mode: ActionMode = "shape_intent",
        max_episode_steps: int = 300,
        success_progress: float = 0.98,
        reward_weights: dict[str, float] | None = None,
        domain_randomization: bool = False,
        curriculum_stage: int = 0,
        **engine_kwargs: Any,
    ) -> None:
        super().__init__()
        if action_mode not in {"shape_intent", "direct"}:
            raise ValueError(f"Unsupported action_mode: {action_mode}")

        self.phantom = phantom
        self.route_target = route_target
        self.target = target
        self.action_mode: ActionMode = action_mode
        self.max_episode_steps = int(max_episode_steps)
        self.success_progress = float(success_progress)
        self.domain_randomization = bool(domain_randomization)
        self.curriculum_stage = int(curriculum_stage)
        self.engine_kwargs = dict(engine_kwargs)
        self.reward_weights = {
            "progress": 10.0,
            "deviation": 1.0,
            "contact": 0.02,
            "risk": 2.0,
            "success": 50.0,
            **(reward_weights or {}),
        }

        self.action_space = (
            spaces.Box(low=np.array([-1.0, -1.0, -1.0, 0.0], dtype=np.float32),
                       high=np.array([1.0, 1.0, 1.0, 1.0], dtype=np.float32),
                       dtype=np.float32)
            if action_mode == "shape_intent"
            else spaces.Box(low=-1.0, high=1.0, shape=(2,), dtype=np.float32)
        )
        self.observation_space = spaces.Dict({
            "tip_position": spaces.Box(-np.inf, np.inf, shape=(3,), dtype=np.float32),
            "tip_direction": spaces.Box(-1.0, 1.0, shape=(3,), dtype=np.float32),
            "progress": spaces.Box(0.0, 1.0, shape=(1,), dtype=np.float32),
            "deviation": spaces.Box(0.0, np.inf, shape=(1,), dtype=np.float32),
            "contact_force": spaces.Box(0.0, np.inf, shape=(1,), dtype=np.float32),
            "wall_distance": spaces.Box(0.0, np.inf, shape=(1,), dtype=np.float32),
            "curvature": spaces.Box(0.0, np.inf, shape=(1,), dtype=np.float32),
            "risk_score": spaces.Box(0.0, 1.0, shape=(1,), dtype=np.float32),
            "remaining_distance": spaces.Box(0.0, np.inf, shape=(1,), dtype=np.float32),
            "vessel_radius": spaces.Box(0.0, np.inf, shape=(1,), dtype=np.float32),
        })

        self.engine = self._make_engine()
        self._step_count = 0
        self._last_state = None
        self._prev_progress = 0.0

    def _make_engine(self):
        from services.navigation_engine import NavigationEngine

        return NavigationEngine(
            phantom=self.phantom,
            target=self.target,
            route_target=self.route_target,
            guided=False,
            **self.engine_kwargs,
        )

    def reset(self, *, seed: int | None = None, options: dict | None = None):
        super().reset(seed=seed)
        state = self.engine.reset()
        self._step_count = 0
        self._last_state = state
        self._prev_progress = float(state.path_progress)
        return self._state_to_obs(state), self._info(state)

    def step(self, action):
        if self._last_state is None:
            self.reset()

        action_arr = np.asarray(action, dtype=np.float32)
        if self.action_mode == "shape_intent":
            self.engine.set_shape_intent(
                {
                    "target_direction": action_arr[:3].astype(float).tolist(),
                    "intensity": float(action_arr[3]),
                },
                active=True,
            )
            state = self.engine.step(0.0, 0.0)
        else:
            state = self.engine.step(float(action_arr[0]), float(action_arr[1]))

        reward = self._compute_reward(state)
        self._step_count += 1
        terminated = bool(state.done or state.path_progress >= self.success_progress)
        truncated = self._step_count >= self.max_episode_steps
        self._last_state = state
        return self._state_to_obs(state), reward, terminated, truncated, self._info(state)

    def close(self) -> None:
        if getattr(self, "engine", None) is not None:
            self.engine.close()

    def _compute_reward(self, state) -> float:
        progress = float(state.path_progress)
        delta_progress = progress - self._prev_progress
        self._prev_progress = progress
        reward = self.reward_weights["progress"] * delta_progress
        reward -= self.reward_weights["deviation"] * float(state.path_deviation)
        reward -= self.reward_weights["contact"] * float(state.contact_force)
        reward -= self.reward_weights["risk"] * float(state.risk_score)
        if progress >= self.success_progress:
            reward += self.reward_weights["success"]
        return float(reward)

    def _state_to_obs(self, state) -> dict[str, np.ndarray]:
        radius = 0.0 if state.vessel_radius is None else float(state.vessel_radius)
        return {
            "tip_position": np.asarray(state.tip_position, dtype=np.float32),
            "tip_direction": np.asarray(state.tip_direction, dtype=np.float32),
            "progress": np.asarray([state.path_progress], dtype=np.float32),
            "deviation": np.asarray([state.path_deviation], dtype=np.float32),
            "contact_force": np.asarray([state.contact_force], dtype=np.float32),
            "wall_distance": np.asarray([state.wall_distance], dtype=np.float32),
            "curvature": np.asarray([state.curvature], dtype=np.float32),
            "risk_score": np.asarray([state.risk_score], dtype=np.float32),
            "remaining_distance": np.asarray([state.remaining_distance], dtype=np.float32),
            "vessel_radius": np.asarray([radius], dtype=np.float32),
        }

    def _info(self, state) -> dict[str, Any]:
        return {
            "progress": float(state.path_progress),
            "safety_status": state.safety_status,
            "fidelity_mode": state.fidelity_mode,
            "route_target": self.route_target,
            "curriculum_stage": self.curriculum_stage,
        }
