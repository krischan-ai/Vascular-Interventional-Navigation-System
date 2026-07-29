from __future__ import annotations

from typing import Any, Literal

import gymnasium as gym
import numpy as np
from gymnasium import spaces


ActionMode = Literal["shape_intent", "direct"]

ENVIRONMENT_VERSION = "navigation_env_v3"
OBSERVATION_VERSION = "navigation_observation_v1"
REWARD_VERSION = "navigation_reward_v2"
TERMINATION_VERSION = "navigation_termination_v1"
SAFETY_METRICS_VERSION = "navigation_safety_metrics_v2"
DOMAIN_RANDOMIZATION_VERSION = "navigation_domain_randomization_v1"

DEFAULT_REWARD_WEIGHTS = {
    "progress": 2.0,
    "alignment": 0.2,
    "deviation": 0.3,
    "contact": 0.5,
    "risk": 0.3,
    "smoothness": 0.05,
    "success": 10.0,
    "failure": 10.0,
    "timeout": 2.0,
    "regression": 1.0,
}

DEFAULT_DOMAIN_RANDOMIZATION_RANGES = {
    "bend": (16.0, 24.0),
    "tip_bend": (1.5, 2.5),
    "contact_ke": (2_400_000.0, 3_600_000.0),
    "push_speed": (0.045, 0.055),
    "rotate_speed": (2.7, 3.3),
}

DOMAIN_RANDOMIZATION_PARAMETERS = frozenset({
    "bend",
    "tip_bend",
    "stretch",
    "push_speed",
    "rotate_speed",
    "contact_ke",
    "free_len",
    "max_slack",
    "support_speed",
    "insertion_margin",
    "jtip_deg",
    "wall_thickness",
    "rod_length",
})


def _domain_randomization_ranges(
    ranges: dict[str, Any] | None,
) -> dict[str, tuple[float, float]]:
    source = DEFAULT_DOMAIN_RANDOMIZATION_RANGES if ranges is None else ranges
    normalized: dict[str, tuple[float, float]] = {}
    for name, bounds in source.items():
        if name not in DOMAIN_RANDOMIZATION_PARAMETERS:
            raise ValueError(f"Unsupported domain randomization parameter: {name}")
        if not isinstance(bounds, (list, tuple)) or len(bounds) != 2:
            raise ValueError(
                f"Domain randomization range for {name} must contain [low, high]"
            )
        low, high = (float(bounds[0]), float(bounds[1]))
        if not np.isfinite(low) or not np.isfinite(high) or low > high:
            raise ValueError(
                f"Invalid domain randomization range for {name}: {bounds}"
            )
        normalized[name] = (low, high)
    if not normalized:
        raise ValueError("At least one domain randomization range is required")
    return normalized


def navigation_protocol_metadata(
    *,
    phantom: str = "aorta_tree",
    route_target: str | None = "endpoint_0",
    action_mode: str = "shape_intent",
    max_episode_steps: int = 300,
    success_progress: float = 0.98,
    reward_weights: dict[str, float] | None = None,
    domain_randomization: bool = False,
    domain_randomization_ranges: dict[str, Any] | None = None,
    curriculum_stage: int = 0,
    observation_lookahead: float = 0.01,
    contact_force_scale: float = 1.0,
    curvature_scale: float = 100.0,
    radius_scale: float = 0.01,
    progress_reference: float = 0.01,
    requested_physics_engine: str = "auto",
    requested_physics_parameters: dict[str, Any] | None = None,
    effective_physics_engine: str | None = None,
    effective_physics_parameters: dict[str, Any] | None = None,
) -> dict[str, Any]:
    """JSON-safe contract used to compare training and frozen evaluations."""
    return {
        "environment_version": ENVIRONMENT_VERSION,
        "observation_version": OBSERVATION_VERSION,
        "reward_version": REWARD_VERSION,
        "termination_version": TERMINATION_VERSION,
        "safety_metrics_version": SAFETY_METRICS_VERSION,
        "domain_randomization_version": DOMAIN_RANDOMIZATION_VERSION,
        "task": {
            "phantom": phantom,
            "route_target": route_target,
            "action_mode": action_mode,
            "max_episode_steps": int(max_episode_steps),
            "success_progress": float(success_progress),
            "domain_randomization": bool(domain_randomization),
            "curriculum_stage": int(curriculum_stage),
        },
        "reward": {
            "weights": {
                **DEFAULT_REWARD_WEIGHTS,
                **dict(reward_weights or {}),
            },
            "progress_reference": float(progress_reference),
        },
        "observation_scales": {
            "lookahead_m": float(observation_lookahead),
            "contact_force_n": float(contact_force_scale),
            "curvature_m_inv": float(curvature_scale),
            "radius_m": float(radius_scale),
        },
        "domain_randomization": {
            "enabled": bool(domain_randomization),
            "ranges": {
                name: [float(bounds[0]), float(bounds[1])]
                for name, bounds in _domain_randomization_ranges(
                    domain_randomization_ranges
                ).items()
            }
            if domain_randomization
            else {},
        },
        "physics": {
            "requested_engine": requested_physics_engine,
            "effective_engine": effective_physics_engine,
            "requested_parameters": dict(requested_physics_parameters or {}),
            "effective_parameters": dict(effective_physics_parameters or {}),
        },
    }


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
        domain_randomization_ranges: dict[str, Any] | None = None,
        curriculum_stage: int = 0,
        observation_lookahead: float = 0.01,
        contact_force_scale: float = 1.0,
        curvature_scale: float = 100.0,
        radius_scale: float = 0.01,
        progress_reference: float = 0.01,
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
        self.domain_randomization_ranges = (
            _domain_randomization_ranges(domain_randomization_ranges)
            if self.domain_randomization
            else {}
        )
        self.curriculum_stage = int(curriculum_stage)
        self.observation_lookahead = max(float(observation_lookahead), 0.0)
        self.contact_force_scale = max(float(contact_force_scale), 1e-6)
        self.curvature_scale = max(float(curvature_scale), 1e-6)
        self.radius_scale = max(float(radius_scale), 1e-6)
        self.progress_reference = max(float(progress_reference), 1e-6)
        self.engine_kwargs = dict(engine_kwargs)
        self.reward_weights = {
            **DEFAULT_REWARD_WEIGHTS,
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
            "tip_position": spaces.Box(-2.0, 2.0, shape=(3,), dtype=np.float32),
            "tip_direction": spaces.Box(-1.0, 1.0, shape=(3,), dtype=np.float32),
            "target_direction_local": spaces.Box(-1.0, 1.0, shape=(3,), dtype=np.float32),
            "local_tangent": spaces.Box(-1.0, 1.0, shape=(3,), dtype=np.float32),
            "previous_action": spaces.Box(-1.0, 1.0, shape=self.action_space.shape, dtype=np.float32),
            "progress": spaces.Box(0.0, 1.0, shape=(1,), dtype=np.float32),
            "progress_velocity": spaces.Box(-1.0, 1.0, shape=(1,), dtype=np.float32),
            "contact_duration": spaces.Box(0.0, 1.0, shape=(1,), dtype=np.float32),
            "deviation": spaces.Box(0.0, 2.0, shape=(1,), dtype=np.float32),
            "contact_force": spaces.Box(0.0, 2.0, shape=(1,), dtype=np.float32),
            "wall_distance": spaces.Box(0.0, 2.0, shape=(1,), dtype=np.float32),
            "curvature": spaces.Box(0.0, 2.0, shape=(1,), dtype=np.float32),
            "risk_score": spaces.Box(0.0, 1.0, shape=(1,), dtype=np.float32),
            "remaining_distance": spaces.Box(0.0, 1.0, shape=(1,), dtype=np.float32),
            "vessel_radius": spaces.Box(0.0, 2.0, shape=(1,), dtype=np.float32),
        })

        self.engine = self._make_engine()
        self._step_count = 0
        self._last_state = None
        self._prev_progress = 0.0
        self._previous_action = np.zeros(self.action_space.shape, dtype=np.float32)
        self._last_reward_components: dict[str, float] = {}
        self._termination_reason = "running"
        self._progress_velocity = 0.0
        self._contact_steps = 0
        self._episode_randomization: dict[str, Any] = {
            "enabled": self.domain_randomization,
            "applied": False,
            "seed": None,
            "requested": {},
            "effective": {},
        }

    def protocol_metadata(self) -> dict[str, Any]:
        """Resolved task/reward/physics contract for reproducible evaluation."""
        backend = getattr(self.engine, "_engine", None)
        effective_parameters = getattr(backend, "protocol_parameters", None)
        if effective_parameters is None:
            effective_parameters = getattr(backend, "deform_params", {})
        if not isinstance(effective_parameters, dict):
            effective_parameters = {}
        return navigation_protocol_metadata(
            phantom=self.phantom,
            route_target=self.route_target,
            action_mode=self.action_mode,
            max_episode_steps=self.max_episode_steps,
            success_progress=self.success_progress,
            reward_weights=self.reward_weights,
            domain_randomization=self.domain_randomization,
            domain_randomization_ranges=self.domain_randomization_ranges,
            curriculum_stage=self.curriculum_stage,
            observation_lookahead=self.observation_lookahead,
            contact_force_scale=self.contact_force_scale,
            curvature_scale=self.curvature_scale,
            radius_scale=self.radius_scale,
            progress_reference=self.progress_reference,
            requested_physics_engine=str(
                self.engine_kwargs.get("physics_engine", "auto")
            ),
            requested_physics_parameters=dict(
                self.engine_kwargs.get("newton_params") or {}
            ),
            effective_physics_engine=(
                type(backend).__name__ if backend is not None else None
            ),
            effective_physics_parameters=effective_parameters,
        )

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
        self._apply_domain_randomization(seed)
        state = self.engine.reset()
        self._step_count = 0
        self._last_state = state
        self._prev_progress = float(state.path_progress)
        self._previous_action = np.zeros(self.action_space.shape, dtype=np.float32)
        self._last_reward_components = {}
        self._termination_reason = "running"
        self._progress_velocity = 0.0
        self._contact_steps = 0
        return self._state_to_obs(state), self._info(state)

    def _apply_domain_randomization(self, seed: int | None) -> None:
        if not self.domain_randomization:
            self._episode_randomization = {
                "enabled": False,
                "applied": False,
                "seed": seed,
                "requested": {},
                "effective": {},
            }
            return

        requested = {
            name: float(self.np_random.uniform(low, high))
            for name, (low, high) in self.domain_randomization_ranges.items()
        }
        effective = self.engine.set_engine_params(requested)
        if not isinstance(effective, dict):
            raise RuntimeError(
                "Domain randomization requires a physics backend with "
                "set_engine_params support"
            )
        self._episode_randomization = {
            "enabled": True,
            "applied": True,
            "seed": seed,
            "requested": requested,
            "effective": {
                name: effective.get(name)
                for name in requested
            },
        }

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

        reward = self._compute_reward(state, action_arr)
        self._step_count += 1
        if (
            int(getattr(state, "wall_contact_count", 0)) > 0
            or float(state.contact_force) > 0.0
        ):
            self._contact_steps += 1
        else:
            self._contact_steps = 0
        terminated, self._termination_reason = self._termination(state)
        truncated = self._step_count >= self.max_episode_steps
        if truncated and not terminated:
            self._termination_reason = "timeout"
            timeout_penalty = -self.reward_weights["timeout"]
            self._last_reward_components["timeout"] = timeout_penalty
            reward += timeout_penalty
        self._last_state = state
        return self._state_to_obs(state), reward, terminated, truncated, self._info(state)

    def close(self) -> None:
        if getattr(self, "engine", None) is not None:
            self.engine.close()

    def _compute_reward(self, state, action: np.ndarray) -> float:
        progress = self._finite_float(state.path_progress)
        previous_progress = self._prev_progress
        delta_progress = progress - previous_progress
        normalized_progress = float(np.clip(delta_progress / self.progress_reference, -1.0, 1.0))
        self._progress_velocity = float(np.clip(delta_progress, -1.0, 1.0))
        self._prev_progress = progress
        radius = max(self._finite_float(state.vessel_radius or 0.0), 1e-6)
        tangent = self._local_tangent(state)
        tip_direction = np.asarray(state.tip_direction, dtype=np.float64)
        tip_norm = float(np.linalg.norm(tip_direction))
        alignment = float(np.dot(tip_direction / tip_norm, tangent)) if tip_norm > 1e-9 else 0.0
        action_delta = np.asarray(action, dtype=np.float32) - self._previous_action
        self._previous_action = np.asarray(action, dtype=np.float32).copy()

        components = {
            "progress": self.reward_weights["progress"] * normalized_progress,
            "alignment": self.reward_weights["alignment"] * alignment * max(normalized_progress, 0.0),
            "regression": -self.reward_weights["regression"] * max(-normalized_progress, 0.0)
                          * float(np.clip((previous_progress - 0.8) / 0.2, 0.0, 1.0)),
            "deviation": -self.reward_weights["deviation"] * float(np.clip(self._finite_float(state.path_deviation) / radius, 0.0, 2.0)),
            "contact": -self.reward_weights["contact"] * float(np.clip(self._finite_float(state.contact_force), 0.0, 2.0)) ** 2,
            "risk": -self.reward_weights["risk"] * float(np.clip(self._finite_float(state.risk_score), 0.0, 1.0)),
            "smoothness": -self.reward_weights["smoothness"] * float(np.dot(action_delta, action_delta)),
            "terminal": self.reward_weights["success"] if progress >= self.success_progress else 0.0,
            "timeout": 0.0,
        }
        if state.done and progress < self.success_progress:
            components["terminal"] -= self.reward_weights["failure"]
        self._last_reward_components = components
        return float(sum(components.values()))

    def _local_tangent(self, state) -> np.ndarray:
        path = getattr(self.engine, "_path", None)
        if path is not None and getattr(path, "total_len", 0.0) > 0.0:
            tangent = np.asarray(
                path.tangent_at_arclen(float(state.path_progress) * path.total_len),
                dtype=np.float64,
            )
        else:
            tangent = np.asarray(state.tip_direction, dtype=np.float64)
        norm = float(np.linalg.norm(tangent))
        return tangent / norm if norm > 1e-9 else np.array([0.0, 0.0, 1.0])

    def _termination(self, state) -> tuple[bool, str]:
        if float(state.path_progress) >= self.success_progress:
            return True, "success"
        if state.safety_status == "COLLISION_STOP":
            return True, "collision_stop"
        if state.done:
            return True, "engine_done"
        return False, "running"

    def _state_to_obs(self, state) -> dict[str, np.ndarray]:
        radius = 0.0 if state.vessel_radius is None else float(state.vessel_radius)
        safe_radius = max(radius, self.radius_scale * 0.1, 1e-6)
        tangent = self._local_tangent(state)
        frame = self._local_frame(tangent)
        tip_position = self._finite_vector(state.tip_position)
        path = getattr(self.engine, "_path", None)
        if path is not None and getattr(path, "total_len", 0.0) > 0.0:
            s = float(np.clip(state.path_progress, 0.0, 1.0)) * path.total_len
            target = np.asarray(path.point_at_arclen(s + self.observation_lookahead), dtype=np.float64)
            total_len = max(float(path.total_len), 1e-6)
        else:
            target = self._finite_vector(state.target_position)
            if np.linalg.norm(target - tip_position) < 1e-9:
                target = tip_position + tangent * self.observation_lookahead
            total_len = max(float(state.remaining_distance) / max(1.0 - float(state.path_progress), 1e-6), 1e-6)
        target_delta = target - tip_position
        target_norm = float(np.linalg.norm(target_delta))
        target_direction = target_delta / target_norm if target_norm > 1e-9 else tangent
        tip_direction = self._unit_vector(state.tip_direction, tangent)

        return {
            "tip_position": np.clip(frame.T @ (tip_position - target) / safe_radius, -2.0, 2.0).astype(np.float32),
            "tip_direction": np.clip(frame.T @ tip_direction, -1.0, 1.0).astype(np.float32),
            "target_direction_local": np.clip(frame.T @ target_direction, -1.0, 1.0).astype(np.float32),
            "local_tangent": np.clip(frame.T @ tangent, -1.0, 1.0).astype(np.float32),
            "previous_action": np.clip(self._previous_action, -1.0, 1.0).astype(np.float32),
            "progress": self._scalar(state.path_progress, 0.0, 1.0),
            "progress_velocity": self._scalar(self._progress_velocity, -1.0, 1.0),
            "contact_duration": self._scalar(self._contact_steps / max(self.max_episode_steps, 1), 0.0, 1.0),
            "deviation": self._scalar(float(state.path_deviation) / safe_radius, 0.0, 2.0),
            "contact_force": self._scalar(float(state.contact_force) / self.contact_force_scale, 0.0, 2.0),
            "wall_distance": self._scalar(float(state.wall_distance) / safe_radius, 0.0, 2.0),
            "curvature": self._scalar(float(state.curvature) / self.curvature_scale, 0.0, 2.0),
            "risk_score": self._scalar(state.risk_score, 0.0, 1.0),
            "remaining_distance": self._scalar(float(state.remaining_distance) / total_len, 0.0, 1.0),
            "vessel_radius": self._scalar(radius / self.radius_scale, 0.0, 2.0),
        }

    @staticmethod
    def _unit_vector(value, fallback: np.ndarray) -> np.ndarray:
        vector = NavigationGymEnv._finite_vector(value)
        norm = float(np.linalg.norm(vector))
        return vector / norm if np.isfinite(norm) and norm > 1e-9 else fallback.copy()

    @staticmethod
    def _local_frame(tangent: np.ndarray) -> np.ndarray:
        reference = np.array([1.0, 0.0, 0.0]) if abs(float(tangent[0])) < 0.9 else np.array([0.0, 1.0, 0.0])
        lateral = np.cross(reference, tangent)
        lateral /= max(float(np.linalg.norm(lateral)), 1e-9)
        normal = np.cross(tangent, lateral)
        normal /= max(float(np.linalg.norm(normal)), 1e-9)
        return np.column_stack((lateral, normal, tangent))

    @staticmethod
    def _scalar(value: float, low: float, high: float) -> np.ndarray:
        finite = NavigationGymEnv._finite_float(value)
        return np.asarray([np.clip(finite, low, high)], dtype=np.float32)

    @staticmethod
    def _finite_float(value: float) -> float:
        converted = float(value)
        return converted if np.isfinite(converted) else 0.0

    @staticmethod
    def _finite_vector(value) -> np.ndarray:
        vector = np.asarray(value, dtype=np.float64).reshape(3)
        return np.nan_to_num(vector, nan=0.0, posinf=0.0, neginf=0.0)

    def _info(self, state) -> dict[str, Any]:
        contact_force = float(max(0.0, state.contact_force))
        contact_impulse = float(max(0.0, getattr(state, "contact_impulse", 0.0)))
        if contact_impulse <= 0.0 and contact_force > 0.0:
            backend = getattr(self.engine, "_engine", None)
            control_dt = float(getattr(backend, "control_timestep", 1.0))
            contact_impulse = contact_force * max(control_dt, 0.0)
        return {
            "progress": float(state.path_progress),
            "safety_status": state.safety_status,
            "fidelity_mode": state.fidelity_mode,
            "route_target": self.route_target,
            "curriculum_stage": self.curriculum_stage,
            "reward_components": dict(self._last_reward_components),
            "termination_reason": self._termination_reason,
            "protocol": {
                "environment_version": ENVIRONMENT_VERSION,
                "reward_version": REWARD_VERSION,
                "safety_metrics_version": SAFETY_METRICS_VERSION,
                "domain_randomization_version": DOMAIN_RANDOMIZATION_VERSION,
            },
            "domain_randomization": {
                **self._episode_randomization,
                "requested": dict(self._episode_randomization["requested"]),
                "effective": dict(self._episode_randomization["effective"]),
            },
            "costs": {
                "contact": contact_force,
                "contact_impulse": contact_impulse,
                "wall_contact_count": int(
                    max(0, getattr(state, "wall_contact_count", 0))
                ),
                "max_penetration_m": float(
                    max(0.0, getattr(state, "max_penetration", 0.0))
                ),
                "penetration": float(state.safety_status == "COLLISION_STOP"),
                "buckling": 0.0,
                "wrong_branch": 0.0,
                "shield_intervention": 0.0,
            },
        }
