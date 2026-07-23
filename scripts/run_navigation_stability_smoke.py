from __future__ import annotations

import argparse
import json
import os
from collections import Counter

import gymnasium as gym
import numpy as np

import cathsim.gym  # noqa: F401 - registers public environment IDs


def main() -> int:
    parser = argparse.ArgumentParser(description="Run finite-value checks on NavigationGymEnv.")
    parser.add_argument("--engine", choices=["newton", "mujoco"], default="newton")
    parser.add_argument("--steps", type=int, default=100)
    parser.add_argument("--seed", type=int, default=0)
    parser.add_argument("--phantom", default="aorta_tree")
    parser.add_argument("--route-target", default="endpoint_0")
    args = parser.parse_args()

    os.environ["CATHSIM_PHYSICS_ENGINE"] = args.engine
    env = gym.make(
        "cathsim/NavigationGym-v0",
        phantom=args.phantom,
        route_target=args.route_target,
        action_mode="shape_intent",
        max_episode_steps=max(args.steps, 20),
    )
    obs, _ = env.reset(seed=args.seed)
    rng = np.random.default_rng(args.seed)
    reasons: Counter[str] = Counter()
    reward_total = 0.0

    try:
        for _ in range(args.steps):
            action = rng.uniform(env.action_space.low, env.action_space.high).astype(np.float32)
            obs, reward, terminated, truncated, info = env.step(action)
            if not all(np.isfinite(value).all() for value in obs.values()):
                raise RuntimeError("non-finite observation detected")
            if not np.isfinite(reward):
                raise RuntimeError("non-finite reward detected")
            if not all(np.isfinite(value) for value in info["reward_components"].values()):
                raise RuntimeError("non-finite reward component detected")
            reward_total += float(reward)
            if terminated or truncated:
                reason = info["termination_reason"]
                if truncated and reason == "running":
                    reason = "time_limit"
                reasons[reason] += 1
                obs, _ = env.reset()
    finally:
        env.close()

    print(json.dumps({
        "status": "ok",
        "engine": args.engine,
        "steps": args.steps,
        "reward_total": reward_total,
        "termination_reasons": dict(reasons),
    }, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
