import numpy as np
import pytest

from services.navigation_engine import NavigationState


class FakeNavigationEngine:
    def __init__(self, *_, **__):
        self.progress = 0.0
        self.intent_calls = []
        self.step_calls = []
        self.closed = False

    def reset(self):
        self.progress = 0.0
        return self._state()

    def step(self, push, rotate):
        self.step_calls.append((push, rotate))
        self.progress = min(1.0, self.progress + 0.2 + max(0.0, float(push)) * 0.1)
        return self._state()

    def set_shape_intent(self, intent, active=True):
        self.intent_calls.append((intent, active))
        return {"active": active, "mode": "direction"}

    def close(self):
        self.closed = True

    def _state(self):
        return NavigationState(
            tip_position=[0.0, 0.0, self.progress],
            tip_direction=[0.0, 0.0, 1.0],
            path_progress=self.progress,
            path_deviation=0.001,
            contact_force=0.0,
            wall_distance=0.003,
            curvature=0.0,
            remaining_distance=max(0.0, 1.0 - self.progress),
            vessel_radius=0.004,
            risk_score=0.1,
            safety_status="SAFE_NAV" if self.progress > 0.0 else "STANDBY",
            fidelity_mode="physics",
        )


def test_navigation_gym_shape_intent_step(monkeypatch):
    import services.navigation_engine as navigation_engine
    from cathsim.gym.envs.navigation import NavigationGymEnv

    monkeypatch.setattr(navigation_engine, "NavigationEngine", FakeNavigationEngine)
    env = NavigationGymEnv(action_mode="shape_intent", max_episode_steps=5)

    obs, info = env.reset()
    assert set(obs) >= {"tip_position", "progress", "remaining_distance", "vessel_radius"}
    assert obs["progress"].shape == (1,)
    assert obs["previous_action"].shape == (4,)
    assert obs["progress_velocity"][0] == 0.0
    assert info["fidelity_mode"] == "physics"

    obs, reward, terminated, truncated, info = env.step(np.array([0.0, 0.0, 1.0, 0.8]))

    assert env.engine.intent_calls[-1][0]["target_direction"] == [0.0, 0.0, 1.0]
    assert env.engine.intent_calls[-1][0]["intensity"] == 0.800000011920929
    assert env.engine.step_calls[-1] == (0.0, 0.0)
    assert obs["progress"][0] > 0.0
    assert obs["progress_velocity"][0] > 0.0
    assert np.allclose(obs["previous_action"], [0.0, 0.0, 1.0, 0.8])
    assert reward > 0.0
    assert terminated is False
    assert truncated is False
    assert set(info["reward_components"]) == {
        "progress", "alignment", "regression", "deviation", "contact", "risk", "smoothness",
        "terminal", "timeout"
    }
    assert info["termination_reason"] == "running"
    assert set(info["costs"]) == {
        "contact", "penetration", "buckling", "wrong_branch", "shield_intervention"
    }
    env.close()
    assert env.engine.closed is True


def test_navigation_gym_direct_action(monkeypatch):
    import services.navigation_engine as navigation_engine
    from cathsim.gym.envs.navigation import NavigationGymEnv

    monkeypatch.setattr(navigation_engine, "NavigationEngine", FakeNavigationEngine)
    env = NavigationGymEnv(action_mode="direct", max_episode_steps=1)
    env.reset()

    _, _, _, truncated, _ = env.step(np.array([0.5, -0.25]))

    assert env.engine.intent_calls == []
    assert env.engine.step_calls[-1] == (0.5, -0.25)
    assert truncated is True


def test_navigation_gym_registered(monkeypatch):
    import gymnasium as gym
    import services.navigation_engine as navigation_engine
    import cathsim.gym  # noqa: F401 - registers env ids

    monkeypatch.setattr(navigation_engine, "NavigationEngine", FakeNavigationEngine)
    env = gym.make("cathsim/NavigationGym-v0", action_mode="direct")
    obs, _ = env.reset()

    assert "risk_score" in obs
    env.close()


def test_navigation_gym_reports_timeout(monkeypatch):
    import services.navigation_engine as navigation_engine
    from cathsim.gym.envs.navigation import NavigationGymEnv

    monkeypatch.setattr(navigation_engine, "NavigationEngine", FakeNavigationEngine)
    env = NavigationGymEnv(action_mode="direct", max_episode_steps=1)
    env.reset()

    _, _, terminated, truncated, info = env.step(np.zeros(2, dtype=np.float32))

    assert terminated is False
    assert truncated is True
    assert info["termination_reason"] == "timeout"
    assert info["reward_components"]["timeout"] == -2.0


def test_navigation_gym_alignment_requires_progress_and_regression_is_penalized(monkeypatch):
    import services.navigation_engine as navigation_engine
    from cathsim.gym.envs.navigation import NavigationGymEnv

    monkeypatch.setattr(navigation_engine, "NavigationEngine", FakeNavigationEngine)
    env = NavigationGymEnv(action_mode="direct", max_episode_steps=5, progress_reference=0.1)
    env.reset()
    state = env.engine._state()
    env._prev_progress = state.path_progress
    env._compute_reward(state, np.zeros(2, dtype=np.float32))
    assert env._last_reward_components["alignment"] == 0.0

    env._prev_progress = 0.9
    state.path_progress = 0.8
    env._compute_reward(state, np.zeros(2, dtype=np.float32))
    assert env._last_reward_components["progress"] == pytest.approx(-2.0)
    assert env._last_reward_components["regression"] < 0.0
    env.close()


def test_navigation_gym_check_env_and_finite_rollout(monkeypatch):
    import services.navigation_engine as navigation_engine
    from gymnasium.utils.env_checker import check_env
    from cathsim.gym.envs.navigation import NavigationGymEnv

    monkeypatch.setattr(navigation_engine, "NavigationEngine", FakeNavigationEngine)
    env = NavigationGymEnv(action_mode="shape_intent", max_episode_steps=7)
    check_env(env, skip_render_check=True)
    obs, _ = env.reset(seed=7)

    for _ in range(100):
        obs, reward, terminated, truncated, info = env.step(env.action_space.sample())
        assert all(np.isfinite(value).all() for value in obs.values())
        assert np.isfinite(reward)
        assert all(np.isfinite(value) for value in info["reward_components"].values())
        if terminated or truncated:
            obs, _ = env.reset()

    env.close()


def test_navigation_gym_observations_have_finite_bounds_and_reset_history(monkeypatch):
    import services.navigation_engine as navigation_engine
    from cathsim.gym.envs.navigation import NavigationGymEnv

    monkeypatch.setattr(navigation_engine, "NavigationEngine", FakeNavigationEngine)
    env = NavigationGymEnv(action_mode="direct", max_episode_steps=5)
    obs, _ = env.reset()

    assert all(np.isfinite(space.low).all() and np.isfinite(space.high).all()
               for space in env.observation_space.spaces.values())
    assert env.observation_space.contains(obs)

    env.engine._state = lambda: NavigationState(
        tip_position=[np.inf, np.nan, -np.inf], tip_direction=[0.0, 0.0, 0.0],
        path_progress=np.nan, path_deviation=np.inf, contact_force=np.inf,
        wall_distance=np.inf, curvature=np.inf, remaining_distance=np.inf,
        vessel_radius=None, risk_score=np.nan,
    )
    obs, *_ = env.step(np.array([1.0, -1.0], dtype=np.float32))
    assert env.observation_space.contains(obs)
    assert all(np.isfinite(value).all() for value in obs.values())

    obs, _ = env.reset()
    assert np.all(obs["previous_action"] == 0.0)
    assert obs["progress_velocity"][0] == 0.0
    assert obs["contact_duration"][0] == 0.0
    env.close()
