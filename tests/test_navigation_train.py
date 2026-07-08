import sys
import types
from pathlib import Path


class DummyEnv:
    def __init__(self):
        self.closed = False

    def close(self):
        self.closed = True


class DummyMonitor:
    def __init__(self, env, filename=None):
        self.env = env
        self.filename = filename
        self.closed = False

    def close(self):
        self.closed = True
        self.env.close()


class DummyPPO:
    instances = []

    def __init__(self, policy, env, verbose=0, seed=None, tensorboard_log=None, **kwargs):
        self.policy = policy
        self.env = env
        self.verbose = verbose
        self.seed = seed
        self.tensorboard_log = tensorboard_log
        self.kwargs = kwargs
        self.learn_args = None
        self.saved_path = None
        DummyPPO.instances.append(self)

    def learn(self, **kwargs):
        self.learn_args = kwargs
        return self

    def save(self, path):
        self.saved_path = path
        Path(path).with_suffix(".zip").write_text("dummy", encoding="utf-8")


class DummyCheckpointCallback:
    def __init__(self, **kwargs):
        self.kwargs = kwargs


class DummyEvalCallback:
    def __init__(self, *args, **kwargs):
        self.args = args
        self.kwargs = kwargs


def install_fake_sb3(monkeypatch):
    sb3 = types.ModuleType("stable_baselines3")
    sb3.PPO = DummyPPO
    common = types.ModuleType("stable_baselines3.common")
    monitor = types.ModuleType("stable_baselines3.common.monitor")
    monitor.Monitor = DummyMonitor
    callbacks = types.ModuleType("stable_baselines3.common.callbacks")
    callbacks.CheckpointCallback = DummyCheckpointCallback
    callbacks.EvalCallback = DummyEvalCallback
    monkeypatch.setitem(sys.modules, "stable_baselines3", sb3)
    monkeypatch.setitem(sys.modules, "stable_baselines3.common", common)
    monkeypatch.setitem(sys.modules, "stable_baselines3.common.monitor", monitor)
    monkeypatch.setitem(sys.modules, "stable_baselines3.common.callbacks", callbacks)


def test_train_navigation_ppo_builds_pipeline(monkeypatch, tmp_path):
    import gymnasium as gym
    from cathsim.rl.navigation_train import NavigationTrainConfig, train_navigation_ppo

    DummyPPO.instances.clear()
    install_fake_sb3(monkeypatch)
    made = []

    def fake_make(env_id, **kwargs):
        made.append((env_id, kwargs))
        return DummyEnv()

    monkeypatch.setattr(gym, "make", fake_make)

    config = NavigationTrainConfig(
        output_dir=tmp_path,
        run_name="stage0_test",
        total_timesteps=20,
        eval_episodes=2,
        ppo_kwargs={"n_steps": 8},
    )
    model_path = train_navigation_ppo(config)

    assert made[0][0] == "cathsim/NavigationGym-v0"
    assert made[0][1]["phantom"] == "aorta_tree"
    assert made[0][1]["route_target"] == "endpoint_0"
    assert len(DummyPPO.instances) == 1
    model = DummyPPO.instances[0]
    assert model.policy == "MultiInputPolicy"
    assert model.tensorboard_log == str(config.log_dir)
    assert model.kwargs["n_steps"] == 8
    assert model.learn_args["total_timesteps"] == 20
    assert model.learn_args["tb_log_name"] == "stage0_test"
    assert model_path == config.model_dir / "final_model.zip"
    assert model_path.exists()


def test_make_navigation_env_uses_monitor(monkeypatch, tmp_path):
    import gymnasium as gym
    from cathsim.rl.navigation_train import NavigationTrainConfig, make_navigation_env

    install_fake_sb3(monkeypatch)
    monkeypatch.setattr(gym, "make", lambda *_, **__: DummyEnv())

    config = NavigationTrainConfig(output_dir=tmp_path)
    env = make_navigation_env(config, monitor=True)

    assert isinstance(env, DummyMonitor)
    assert env.filename == str(config.monitor_dir / "monitor.csv")
