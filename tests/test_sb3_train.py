from types import SimpleNamespace

import gymnasium as gym
import numpy as np

from cathsim.rl import make_gym_env


class DummyEnv(gym.Env):
    observation_space = gym.spaces.Box(-1.0, 1.0, shape=(2,), dtype=np.float32)
    action_space = gym.spaces.Box(-1.0, 1.0, shape=(1,), dtype=np.float32)

    def reset(self, *, seed=None, options=None):
        super().reset(seed=seed)
        return np.zeros(2, dtype=np.float32), {}

    def step(self, action):
        return np.zeros(2, dtype=np.float32), 0.0, False, False, {}


def test_make_gym_env_uses_current_rl_entrypoint(monkeypatch):
    captured = {}

    def fake_make(env_id, **kwargs):
        captured.update(env_id=env_id, kwargs=kwargs)
        return DummyEnv()

    monkeypatch.setattr("cathsim.rl.env_utils.gym.make", fake_make)
    config = SimpleNamespace(
        task_kwargs={"phantom": "phantom3", "use_pixels": False},
        wrapper_kwargs={},
    )

    env = make_gym_env(config, monitor_wrapper=False)

    assert isinstance(env, DummyEnv)
    assert captured == {
        "env_id": "cathsim/CathSim-v0",
        "kwargs": config.task_kwargs,
    }
