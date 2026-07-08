import numpy as np

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
    assert info["fidelity_mode"] == "physics"

    obs, reward, terminated, truncated, info = env.step(np.array([0.0, 0.0, 1.0, 0.8]))

    assert env.engine.intent_calls[-1][0]["target_direction"] == [0.0, 0.0, 1.0]
    assert env.engine.intent_calls[-1][0]["intensity"] == 0.800000011920929
    assert env.engine.step_calls[-1] == (0.0, 0.0)
    assert obs["progress"][0] > 0.0
    assert reward > 0.0
    assert terminated is False
    assert truncated is False
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
    import cathsim.gym.envs  # noqa: F401 - registers env ids

    monkeypatch.setattr(navigation_engine, "NavigationEngine", FakeNavigationEngine)
    env = gym.make("cathsim/NavigationGym-v0", action_mode="direct")
    obs, _ = env.reset()

    assert "risk_score" in obs
    env.close()
