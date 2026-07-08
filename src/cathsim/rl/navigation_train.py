from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Any

import gymnasium as gym


@dataclass(frozen=True)
class NavigationTrainConfig:
    """Configuration for T1 PPO training on NavigationGymEnv."""

    run_name: str = "stage0_endpoint_0"
    output_dir: Path = Path("results/navigation_rl")
    phantom: str = "aorta_tree"
    route_target: str = "endpoint_0"
    action_mode: str = "shape_intent"
    total_timesteps: int = 100_000
    seed: int = 0
    n_envs: int = 1
    eval_episodes: int = 5
    tensorboard: bool = True
    policy: str = "MultiInputPolicy"
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


def make_navigation_env(config: NavigationTrainConfig, monitor: bool = True):
    """Create a NavigationGymEnv wrapped for SB3 training."""
    from stable_baselines3.common.monitor import Monitor

    env = gym.make(
        "cathsim/NavigationGym-v0",
        phantom=config.phantom,
        route_target=config.route_target,
        action_mode=config.action_mode,
        **(config.env_kwargs or {}),
    )
    if monitor:
        config.monitor_dir.mkdir(parents=True, exist_ok=True)
        env = Monitor(env, filename=str(config.monitor_dir / "monitor.csv"))
    return env


def train_navigation_ppo(config: NavigationTrainConfig):
    """Run T1 PPO training and save ``model_dir/final_model.zip``.

    The environment is intentionally single-process by default. Newton sessions
    own GPU state, so parallelism should be introduced after Stage 0 converges.
    """
    from stable_baselines3 import PPO
    from stable_baselines3.common.callbacks import CheckpointCallback, EvalCallback

    for path in (config.model_dir, config.log_dir, config.monitor_dir, config.eval_dir):
        path.mkdir(parents=True, exist_ok=True)

    env = make_navigation_env(config, monitor=True)
    eval_env = make_navigation_env(config, monitor=True)

    checkpoint = CheckpointCallback(
        save_freq=max(1, config.total_timesteps // 10),
        save_path=str(config.model_dir),
        name_prefix="checkpoint",
    )
    eval_callback = EvalCallback(
        eval_env,
        best_model_save_path=str(config.model_dir),
        log_path=str(config.eval_dir),
        eval_freq=max(1, config.total_timesteps // 10),
        n_eval_episodes=config.eval_episodes,
        deterministic=True,
    )

    model = PPO(
        config.policy,
        env,
        verbose=1,
        seed=config.seed,
        tensorboard_log=str(config.log_dir) if config.tensorboard else None,
        **(config.ppo_kwargs or {}),
    )
    model.learn(
        total_timesteps=config.total_timesteps,
        callback=[checkpoint, eval_callback],
        tb_log_name=config.run_name,
        progress_bar=True,
    )
    final_path = config.model_dir / "final_model"
    model.save(str(final_path))
    env.close()
    eval_env.close()
    return final_path.with_suffix(".zip")


def main(argv: list[str] | None = None) -> int:
    import argparse

    parser = argparse.ArgumentParser(description="Train PPO on NavigationGymEnv.")
    parser.add_argument("--run-name", default="stage0_endpoint_0")
    parser.add_argument("--output-dir", type=Path, default=Path("results/navigation_rl"))
    parser.add_argument("--phantom", default="aorta_tree")
    parser.add_argument("--route-target", default="endpoint_0")
    parser.add_argument("--action-mode", choices=["shape_intent", "direct"], default="shape_intent")
    parser.add_argument("--total-timesteps", type=int, default=100_000)
    parser.add_argument("--seed", type=int, default=0)
    parser.add_argument("--eval-episodes", type=int, default=5)
    args = parser.parse_args(argv)

    train_navigation_ppo(NavigationTrainConfig(**vars(args)))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
