import gymnasium as gym
import numpy as np
import pytest

import cathsim.gym.envs  # noqa: F401 - registers cathsim/CathSim-v0


@pytest.fixture(scope="module")
def env():
    instance = gym.make(
        "cathsim/CathSim-v0",
        phantom="phantom3",
        target="bca",
        use_pixels=False,
        sample_target=False,
        target_from_sites=False,
    )
    yield instance
    instance.close()


def test_gymnasium_reset_and_step_contract(env):
    observation, info = env.reset(seed=0)
    next_observation, reward, terminated, truncated, next_info = env.step(
        np.zeros(env.action_space.shape, dtype=env.action_space.dtype)
    )

    assert env.observation_space.contains(observation)
    assert env.observation_space.contains(next_observation)
    assert isinstance(info, dict)
    assert np.isscalar(reward)
    assert isinstance(terminated, bool)
    assert isinstance(truncated, bool)
    assert isinstance(next_info, dict)


def test_rgb_array_render(env):
    env.reset()
    image = env.unwrapped.render_frame(image_size=80)
    assert image.shape == (80, 80, 3)
    assert image.dtype == np.uint8
