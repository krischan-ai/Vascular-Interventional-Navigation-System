from __future__ import annotations

import json
from dataclasses import asdict, dataclass
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Literal

import gymnasium as gym

import cathsim.gym  # noqa: F401 - registers cathsim/NavigationGym-v0


Algorithm = Literal["ppo", "sac"]


@dataclass(frozen=True)
class NavigationTrainConfig:
    """Reproducible PPO/SAC training configuration for NavigationGymEnv."""

    run_name: str = "stage0_endpoint_0"
    output_dir: Path = Path("results/navigation_rl")
    algorithm: Algorithm = "ppo"
    phantom: str = "aorta_tree"
    route_target: str = "endpoint_0"
    action_mode: str = "shape_intent"
    physics_engine: str = "newton"
    total_timesteps: int = 100_000
    seed: int = 0
    n_envs: int = 1
    eval_episodes: int = 5
    eval_freq: int = 10_000
    checkpoint_freq: int = 10_000
    tensorboard: bool = True
    progress_bar: bool = True
    policy: str = "MultiInputPolicy"
    resume_model: Path | None = None
    resume_replay_buffer: Path | None = None
    newton_rod_length: float | None = None
    newton_free_len: float | None = None
    newton_max_slack: float | None = None
    newton_insertion_margin: float | None = None
    algorithm_kwargs: dict[str, Any] | None = None
    ppo_kwargs: dict[str, Any] | None = None
    env_kwargs: dict[str, Any] | None = None

    @property
    def run_dir(self) -> Path:
        return self.output_dir / self.run_name

    @property
    def model_dir(self) -> Path:
        return self.run_dir / "models"

    @property
    def log_dir(self) -> Path:
        return self.run_dir / "tb"

    @property
    def monitor_dir(self) -> Path:
        return self.run_dir / "monitor"

    @property
    def eval_dir(self) -> Path:
        return self.run_dir / "eval"


def make_navigation_env(
    config: NavigationTrainConfig,
    monitor: bool = True,
    monitor_name: str = "train",
):
    """Create a NavigationGymEnv wrapped for SB3 training or evaluation."""
    from stable_baselines3.common.monitor import Monitor

    env_options = dict(config.env_kwargs or {})
    env_options.setdefault("physics_engine", config.physics_engine)
    newton_params = {
        key: value
        for key, value in {
            "rod_length": config.newton_rod_length,
            "free_len": config.newton_free_len,
            "max_slack": config.newton_max_slack,
            "insertion_margin": config.newton_insertion_margin,
        }.items()
        if value is not None
    }
    if newton_params:
        env_options["newton_params"] = {
            **newton_params,
            **dict(env_options.get("newton_params") or {}),
        }
    env = gym.make(
        "cathsim/NavigationGym-v0",
        phantom=config.phantom,
        route_target=config.route_target,
        action_mode=config.action_mode,
        **env_options,
    )
    if monitor:
        config.monitor_dir.mkdir(parents=True, exist_ok=True)
        env = Monitor(env, filename=str(config.monitor_dir / f"{monitor_name}.csv"))
    return env


def _json_config(config: NavigationTrainConfig) -> dict[str, Any]:
    data = asdict(config)
    for key in ("output_dir", "resume_model", "resume_replay_buffer"):
        if data[key] is not None:
            data[key] = str(data[key])
    return data


def _write_json(path: Path, payload: dict[str, Any]) -> None:
    path.write_text(json.dumps(payload, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")


def train_navigation(config: NavigationTrainConfig) -> Path:
    """Train PPO or SAC and return the final model zip path.

    Resuming keeps SB3's timestep counter. SAC runs also persist the replay
    buffer so an interrupted off-policy run can continue without cold-starting.
    """
    from stable_baselines3 import PPO, SAC
    from stable_baselines3.common.callbacks import CheckpointCallback, EvalCallback

    if config.algorithm not in {"ppo", "sac"}:
        raise ValueError(f"Unsupported algorithm: {config.algorithm}")
    if config.n_envs != 1:
        raise ValueError("Newton training currently supports n_envs=1 only")
    if config.total_timesteps <= 0:
        raise ValueError("total_timesteps must be positive")

    for path in (config.model_dir, config.log_dir, config.monitor_dir, config.eval_dir):
        path.mkdir(parents=True, exist_ok=True)
    _write_json(config.run_dir / "run_config.json", _json_config(config))
    started_at = datetime.now(timezone.utc).isoformat()
    _write_json(config.run_dir / "run_status.json", {"status": "running", "started_at": started_at})

    env = make_navigation_env(config, monitor=True, monitor_name="train")
    eval_env = make_navigation_env(config, monitor=True, monitor_name="eval")
    algorithm_class = PPO if config.algorithm == "ppo" else SAC
    replay_path = config.model_dir / "final_replay_buffer.pkl"

    checkpoint = CheckpointCallback(
        save_freq=max(1, config.checkpoint_freq),
        save_path=str(config.model_dir),
        name_prefix=f"{config.algorithm}_checkpoint",
        save_replay_buffer=config.algorithm == "sac",
        save_vecnormalize=True,
    )
    eval_callback = EvalCallback(
        eval_env,
        best_model_save_path=str(config.model_dir),
        log_path=str(config.eval_dir),
        eval_freq=max(1, config.eval_freq),
        n_eval_episodes=config.eval_episodes,
        deterministic=True,
    )

    try:
        if config.resume_model is not None:
            model = algorithm_class.load(
                str(config.resume_model),
                env=env,
                tensorboard_log=str(config.log_dir) if config.tensorboard else None,
            )
            if config.algorithm == "sac":
                candidate = config.resume_replay_buffer
                if candidate is not None and candidate.exists():
                    model.load_replay_buffer(str(candidate))
        else:
            algorithm_kwargs = dict(config.algorithm_kwargs or config.ppo_kwargs or {})
            if config.algorithm == "sac":
                algorithm_kwargs = {
                    "buffer_size": 100_000,
                    "learning_starts": 1_000,
                    **algorithm_kwargs,
                }
            model = algorithm_class(
                config.policy,
                env,
                verbose=1,
                seed=config.seed,
                tensorboard_log=str(config.log_dir) if config.tensorboard else None,
                **algorithm_kwargs,
            )

        model.learn(
            total_timesteps=config.total_timesteps,
            callback=[checkpoint, eval_callback],
            tb_log_name=config.run_name,
            progress_bar=config.progress_bar,
            reset_num_timesteps=config.resume_model is None,
        )
        final_path = config.model_dir / "final_model"
        model.save(str(final_path))
        if config.algorithm == "sac":
            model.save_replay_buffer(str(replay_path))
        result = final_path.with_suffix(".zip")
        _write_json(
            config.run_dir / "run_status.json",
            {"status": "completed", "started_at": started_at,
             "finished_at": datetime.now(timezone.utc).isoformat(), "model": str(result)},
        )
        return result
    except Exception as exc:
        _write_json(
            config.run_dir / "run_status.json",
            {"status": "failed", "started_at": started_at,
             "finished_at": datetime.now(timezone.utc).isoformat(), "error": repr(exc)},
        )
        raise
    finally:
        env.close()
        eval_env.close()


def train_navigation_ppo(config: NavigationTrainConfig) -> Path:
    """Backward-compatible PPO entrypoint."""
    if config.algorithm != "ppo":
        raise ValueError("train_navigation_ppo requires algorithm='ppo'")
    return train_navigation(config)


def main(argv: list[str] | None = None) -> int:
    import argparse

    parser = argparse.ArgumentParser(description="Train PPO/SAC on NavigationGymEnv.")
    parser.add_argument("--algorithm", choices=["ppo", "sac"], default="ppo")
    parser.add_argument("--run-name", default="stage0_endpoint_0")
    parser.add_argument("--output-dir", type=Path, default=Path("results/navigation_rl"))
    parser.add_argument("--phantom", default="aorta_tree")
    parser.add_argument("--route-target", default="endpoint_0")
    parser.add_argument("--action-mode", choices=["shape_intent", "direct"], default="shape_intent")
    parser.add_argument("--physics-engine", choices=["newton", "mujoco", "guided"], default="newton")
    parser.add_argument("--total-timesteps", type=int, default=100_000)
    parser.add_argument("--seed", type=int, default=0)
    parser.add_argument("--eval-episodes", type=int, default=5)
    parser.add_argument("--eval-freq", type=int, default=10_000)
    parser.add_argument("--checkpoint-freq", type=int, default=10_000)
    parser.add_argument("--resume-model", type=Path)
    parser.add_argument("--resume-replay-buffer", type=Path)
    parser.add_argument("--newton-rod-length", type=float)
    parser.add_argument("--newton-free-len", type=float)
    parser.add_argument("--newton-max-slack", type=float)
    parser.add_argument("--newton-insertion-margin", type=float)
    parser.add_argument("--no-tensorboard", action="store_true")
    parser.add_argument("--no-progress-bar", action="store_true")
    args = parser.parse_args(argv)
    values = vars(args)
    values["tensorboard"] = not values.pop("no_tensorboard")
    values["progress_bar"] = not values.pop("no_progress_bar")
    train_navigation(NavigationTrainConfig(**values))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
