import sys
import types
import json
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

    @classmethod
    def load(cls, path, env=None, tensorboard_log=None):
        model = cls("loaded", env, tensorboard_log=tensorboard_log)
        model.loaded_path = path
        return model


class DummySAC(DummyPPO):
    instances = []

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        DummySAC.instances.append(self)
        self.saved_replay_buffer = None
        self.loaded_replay_buffer = None

    def save_replay_buffer(self, path):
        self.saved_replay_buffer = path
        Path(path).write_text("replay", encoding="utf-8")

    def load_replay_buffer(self, path):
        self.loaded_replay_buffer = path


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
    sb3.SAC = DummySAC
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
    assert made[0][1]["physics_engine"] == "newton"
    assert made[0][1]["domain_randomization"] is False
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
    assert env.filename == str(config.monitor_dir / "train.csv")


def test_train_navigation_sac_saves_replay_and_metadata(monkeypatch, tmp_path):
    import gymnasium as gym
    from cathsim.rl.navigation_train import NavigationTrainConfig, train_navigation

    DummySAC.instances.clear()
    install_fake_sb3(monkeypatch)
    monkeypatch.setattr(gym, "make", lambda *_, **__: DummyEnv())
    config = NavigationTrainConfig(
        output_dir=tmp_path, run_name="sac_test", algorithm="sac",
        total_timesteps=10, progress_bar=False,
    )

    model_path = train_navigation(config)

    model = DummySAC.instances[-1]
    assert model_path.exists()
    assert model.kwargs["buffer_size"] == 100_000
    assert model.kwargs["learning_starts"] == 1_000
    assert model.saved_replay_buffer == str(config.model_dir / "final_replay_buffer.pkl")
    assert (config.run_dir / "run_config.json").exists()
    run_config = json.loads(
        (config.run_dir / "run_config.json").read_text(encoding="utf-8")
    )
    assert run_config["schema_version"] == "navigation_training_run_v2"
    assert run_config["protocol"]["reward_version"] == "navigation_reward_v2"
    assert run_config["protocol"]["physics"]["requested_engine"] == "newton"
    assert '"status": "completed"' in (config.run_dir / "run_status.json").read_text(encoding="utf-8")


def test_make_navigation_env_enables_domain_randomization(monkeypatch, tmp_path):
    import gymnasium as gym
    from cathsim.rl.navigation_train import NavigationTrainConfig, make_navigation_env

    install_fake_sb3(monkeypatch)
    made = []
    monkeypatch.setattr(
        gym,
        "make",
        lambda *args, **kwargs: made.append((args, kwargs)) or DummyEnv(),
    )
    config = NavigationTrainConfig(
        output_dir=tmp_path,
        domain_randomization=True,
        domain_randomization_ranges={"bend": (16.0, 24.0)},
    )

    make_navigation_env(config, monitor=False)

    assert made[0][1]["domain_randomization"] is True
    assert made[0][1]["domain_randomization_ranges"] == {
        "bend": (16.0, 24.0)
    }


def test_train_navigation_resume_keeps_timestep_counter(monkeypatch, tmp_path):
    import gymnasium as gym
    from cathsim.rl.navigation_train import NavigationTrainConfig, train_navigation

    DummyPPO.instances.clear()
    install_fake_sb3(monkeypatch)
    monkeypatch.setattr(gym, "make", lambda *_, **__: DummyEnv())
    resume = tmp_path / "checkpoint.zip"
    resume.write_text("dummy", encoding="utf-8")

    train_navigation(NavigationTrainConfig(
        output_dir=tmp_path, run_name="resume_test", total_timesteps=5,
        resume_model=resume, progress_bar=False,
    ))

    model = DummyPPO.instances[-1]
    assert model.loaded_path == str(resume)
    assert model.learn_args["reset_num_timesteps"] is False


def test_train_navigation_sweep_uses_independent_run_names_and_manifest(
    monkeypatch, tmp_path
):
    from cathsim.rl import navigation_train

    calls = []

    def fake_train(config):
        calls.append(config)
        return config.model_dir / "final_model.zip"

    monkeypatch.setattr(navigation_train, "train_navigation", fake_train)
    config = navigation_train.NavigationTrainConfig(
        output_dir=tmp_path,
        run_name="stage0_dr",
        domain_randomization=True,
        progress_bar=False,
    )

    manifest = navigation_train.train_navigation_sweep(
        config,
        [0, 1, 1, 2],
    )

    assert manifest["status"] == "completed"
    assert manifest["seeds"] == [0, 1, 2]
    assert [item.run_name for item in calls] == [
        "stage0_dr_seed0",
        "stage0_dr_seed1",
        "stage0_dr_seed2",
    ]
    assert all(item.domain_randomization for item in calls)
    saved = json.loads(
        (
            tmp_path
            / "stage0_dr_sweep"
            / "sweep_manifest.json"
        ).read_text(encoding="utf-8")
    )
    assert saved["completed_runs"] == 3
    assert saved["failed_runs"] == 0
