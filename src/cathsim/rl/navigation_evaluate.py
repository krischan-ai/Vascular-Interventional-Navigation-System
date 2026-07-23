from __future__ import annotations

import argparse
import json
from collections import Counter
from pathlib import Path
from typing import Any, Literal

import gymnasium as gym
import numpy as np

import cathsim.gym  # noqa: F401 - registers cathsim/NavigationGym-v0


def evaluate_navigation_model(model, env: gym.Env, episodes: int = 5, seed: int = 0) -> dict[str, Any]:
    """Evaluate a policy with navigation and safety metrics."""
    if episodes <= 0:
        raise ValueError("episodes must be positive")

    episode_results: list[dict[str, Any]] = []
    reasons: Counter[str] = Counter()
    for episode in range(episodes):
        obs, _ = env.reset(seed=seed + episode)
        terminated = truncated = False
        reward_total = 0.0
        max_progress = 0.0
        contact_integral = 0.0
        max_contact = 0.0
        steps = 0
        info: dict[str, Any] = {"progress": 0.0, "termination_reason": "running", "costs": {}}

        while not (terminated or truncated):
            action, _ = model.predict(obs, deterministic=True)
            obs, reward, terminated, truncated, info = env.step(action)
            if not np.isfinite(reward) or not all(np.isfinite(value).all() for value in obs.values()):
                raise RuntimeError("non-finite policy rollout detected")
            progress = float(info.get("progress", 0.0))
            contact = float(info.get("costs", {}).get("contact", 0.0))
            reward_total += float(reward)
            max_progress = max(max_progress, progress)
            contact_integral += max(0.0, contact)
            max_contact = max(max_contact, contact)
            steps += 1

        reason = str(info.get("termination_reason", "unknown"))
        reasons[reason] += 1
        episode_results.append({
            "episode": episode,
            "seed": seed + episode,
            "reward": reward_total,
            "steps": steps,
            "final_progress": float(info.get("progress", 0.0)),
            "max_progress": max_progress,
            "contact_integral": contact_integral,
            "max_contact": max_contact,
            "termination_reason": reason,
        })

    def stats(key: str) -> dict[str, float]:
        values = np.asarray([row[key] for row in episode_results], dtype=np.float64)
        return {"mean": float(values.mean()), "std": float(values.std()),
                "min": float(values.min()), "max": float(values.max())}

    return {
        "episodes": episodes,
        "success_rate": reasons["success"] / episodes,
        "termination_reasons": dict(reasons),
        "reward": stats("reward"),
        "steps": stats("steps"),
        "final_progress": stats("final_progress"),
        "max_progress": stats("max_progress"),
        "contact_integral": stats("contact_integral"),
        "max_contact": stats("max_contact"),
        "episode_results": episode_results,
    }


def load_model(path: Path, algorithm: Literal["ppo", "sac"]):
    from stable_baselines3 import PPO, SAC

    model_class = PPO if algorithm == "ppo" else SAC
    return model_class.load(str(path))


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="Evaluate a PPO/SAC NavigationGym policy.")
    parser.add_argument("model", type=Path)
    parser.add_argument("--algorithm", choices=["ppo", "sac"], default="sac")
    parser.add_argument("--episodes", type=int, default=5)
    parser.add_argument("--seed", type=int, default=1000)
    parser.add_argument("--phantom", default="aorta_tree")
    parser.add_argument("--route-target", default="endpoint_0")
    parser.add_argument("--physics-engine", choices=["newton", "mujoco", "guided"], default="newton")
    parser.add_argument("--max-episode-steps", type=int, default=300)
    parser.add_argument("--output", type=Path)
    args = parser.parse_args(argv)

    env = gym.make(
        "cathsim/NavigationGym-v0",
        phantom=args.phantom,
        route_target=args.route_target,
        action_mode="shape_intent",
        physics_engine=args.physics_engine,
        max_episode_steps=args.max_episode_steps,
    )
    try:
        result = evaluate_navigation_model(
            load_model(args.model, args.algorithm), env, episodes=args.episodes, seed=args.seed,
        )
    finally:
        env.close()
    result.update({"model": str(args.model), "algorithm": args.algorithm,
                   "phantom": args.phantom, "route_target": args.route_target,
                   "physics_engine": args.physics_engine})
    text = json.dumps(result, ensure_ascii=False, indent=2, sort_keys=True) + "\n"
    if args.output is not None:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(text, encoding="utf-8")
    print(text, end="")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
