from __future__ import annotations

import argparse
import hashlib
import json
from collections import Counter
from pathlib import Path
from typing import Any, Literal

import gymnasium as gym
import numpy as np

import cathsim.gym  # noqa: F401 - registers cathsim/NavigationGym-v0

EVALUATION_SCHEMA_VERSION = "navigation_evaluation_v3"
EVALUATION_PROTOCOL_VERSION = "navigation_frozen_eval_v2"


def _environment_protocol(env: gym.Env) -> dict[str, Any]:
    candidate = env
    visited: set[int] = set()
    while candidate is not None and id(candidate) not in visited:
        visited.add(id(candidate))
        metadata = getattr(candidate, "protocol_metadata", None)
        if callable(metadata):
            return metadata()
        unwrapped = getattr(candidate, "unwrapped", None)
        if unwrapped is not None and unwrapped is not candidate:
            candidate = unwrapped
            continue
        candidate = getattr(candidate, "env", None)

    from cathsim.gym.envs.navigation import navigation_protocol_metadata

    return navigation_protocol_metadata()


def _file_artifact(path: Path) -> dict[str, Any]:
    artifact = {
        "path": str(path),
        "sha256": None,
        "size_bytes": None,
    }
    if not path.is_file():
        return artifact
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    artifact["sha256"] = digest.hexdigest()
    artifact["size_bytes"] = path.stat().st_size
    return artifact


def evaluate_navigation_model(model, env: gym.Env, episodes: int = 5, seed: int = 0) -> dict[str, Any]:
    """Evaluate a policy with navigation and safety metrics."""
    if episodes <= 0:
        raise ValueError("episodes must be positive")

    protocol = _environment_protocol(env)
    episode_results: list[dict[str, Any]] = []
    reasons: Counter[str] = Counter()
    for episode in range(episodes):
        obs, reset_info = env.reset(seed=seed + episode)
        terminated = truncated = False
        reward_total = 0.0
        max_progress = 0.0
        contact_integral = 0.0
        max_contact = 0.0
        wall_contact_steps = 0
        wall_contact_pair_samples = 0
        max_penetration_m = 0.0
        steps = 0
        info: dict[str, Any] = {"progress": 0.0, "termination_reason": "running", "costs": {}}

        while not (terminated or truncated):
            action, _ = model.predict(obs, deterministic=True)
            obs, reward, terminated, truncated, info = env.step(action)
            if not np.isfinite(reward) or not all(np.isfinite(value).all() for value in obs.values()):
                raise RuntimeError("non-finite policy rollout detected")
            progress = float(info.get("progress", 0.0))
            costs = info.get("costs", {})
            contact = float(costs.get("contact", 0.0))
            contact_impulse = float(costs.get("contact_impulse", contact))
            wall_contacts = int(costs.get("wall_contact_count", contact > 0.0))
            penetration = float(costs.get("max_penetration_m", 0.0))
            reward_total += float(reward)
            max_progress = max(max_progress, progress)
            contact_integral += max(0.0, contact_impulse)
            max_contact = max(max_contact, contact)
            wall_contact_steps += int(wall_contacts > 0)
            wall_contact_pair_samples += max(0, wall_contacts)
            max_penetration_m = max(max_penetration_m, penetration)
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
            "wall_contact_steps": wall_contact_steps,
            "wall_contact_pair_samples": wall_contact_pair_samples,
            "max_penetration_m": max_penetration_m,
            "termination_reason": reason,
            "domain_randomization": reset_info.get(
                "domain_randomization",
                {
                    "enabled": False,
                    "applied": False,
                    "seed": seed + episode,
                    "requested": {},
                    "effective": {},
                },
            ),
        })

    def stats(key: str) -> dict[str, float]:
        values = np.asarray([row[key] for row in episode_results], dtype=np.float64)
        return {"mean": float(values.mean()), "std": float(values.std()),
                "min": float(values.min()), "max": float(values.max())}

    return {
        "schema_version": EVALUATION_SCHEMA_VERSION,
        "evaluation_protocol_version": EVALUATION_PROTOCOL_VERSION,
        "protocol": protocol,
        "rollout": {
            "deterministic": True,
            "base_seed": int(seed),
            "episode_seeds": [int(seed + episode) for episode in range(episodes)],
        },
        "episodes": episodes,
        "success_rate": reasons["success"] / episodes,
        "termination_reasons": dict(reasons),
        "reward": stats("reward"),
        "steps": stats("steps"),
        "final_progress": stats("final_progress"),
        "max_progress": stats("max_progress"),
        "contact_integral": stats("contact_integral"),
        "max_contact": stats("max_contact"),
        "wall_contact_steps": stats("wall_contact_steps"),
        "wall_contact_pair_samples": stats("wall_contact_pair_samples"),
        "max_penetration_m": stats("max_penetration_m"),
        "metric_units": {
            "contact_integral": "N s",
            "max_contact": "N (solver force; penetration proxy fallback)",
            "max_penetration_m": "m",
        },
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
    parser.add_argument("--newton-rod-length", type=float)
    parser.add_argument("--newton-free-len", type=float)
    parser.add_argument("--newton-max-slack", type=float)
    parser.add_argument("--newton-insertion-margin", type=float)
    parser.add_argument("--domain-randomization", action="store_true")
    parser.add_argument("--output", type=Path)
    args = parser.parse_args(argv)

    newton_params = {
        key: value
        for key, value in {
            "rod_length": args.newton_rod_length,
            "free_len": args.newton_free_len,
            "max_slack": args.newton_max_slack,
            "insertion_margin": args.newton_insertion_margin,
        }.items()
        if value is not None
    }

    env = gym.make(
        "cathsim/NavigationGym-v0",
        phantom=args.phantom,
        route_target=args.route_target,
        action_mode="shape_intent",
        physics_engine=args.physics_engine,
        max_episode_steps=args.max_episode_steps,
        newton_params=newton_params,
        domain_randomization=args.domain_randomization,
    )
    try:
        result = evaluate_navigation_model(
            load_model(args.model, args.algorithm), env, episodes=args.episodes, seed=args.seed,
        )
    finally:
        env.close()
    result.update({
        "model": str(args.model),
        "model_artifact": _file_artifact(args.model),
        "algorithm": args.algorithm,
        "phantom": args.phantom,
        "route_target": args.route_target,
        "physics_engine": args.physics_engine,
        "newton_params": newton_params,
        "domain_randomization": args.domain_randomization,
    })
    text = json.dumps(result, ensure_ascii=False, indent=2, sort_keys=True) + "\n"
    if args.output is not None:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(text, encoding="utf-8")
    print(text, end="")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
