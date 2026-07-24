from __future__ import annotations

import argparse
import json
from typing import Any

import numpy as np


def _positive_float(value: str) -> float:
    parsed = float(value)
    if parsed <= 0.0:
        raise argparse.ArgumentTypeError("value must be positive")
    return parsed


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(
        description="Run a scripted NavigationGym reachability check without training."
    )
    parser.add_argument("--phantom", default="aorta_tree")
    parser.add_argument("--route-target", default="endpoint_0")
    parser.add_argument("--physics-engine", choices=["newton", "mujoco", "guided"], default="newton")
    parser.add_argument("--action-mode", choices=["direct", "shape_intent"], default="direct")
    parser.add_argument("--steps", type=int, default=320)
    parser.add_argument("--success-progress", type=float, default=0.98)
    parser.add_argument("--newton-rod-length", type=_positive_float, default=0.012)
    parser.add_argument("--newton-free-len", type=_positive_float, default=0.006)
    parser.add_argument("--newton-max-slack", type=_positive_float, default=0.006)
    parser.add_argument("--newton-insertion-margin", type=float, default=0.0)
    args = parser.parse_args(argv)

    from cathsim.gym.envs.navigation import NavigationGymEnv

    newton_params: dict[str, float] = {
        "rod_length": args.newton_rod_length,
        "free_len": args.newton_free_len,
        "max_slack": args.newton_max_slack,
        "insertion_margin": args.newton_insertion_margin,
    }
    env = NavigationGymEnv(
        phantom=args.phantom,
        route_target=args.route_target,
        action_mode=args.action_mode,
        max_episode_steps=args.steps,
        success_progress=args.success_progress,
        physics_engine=args.physics_engine,
        newton_params=newton_params,
    )
    try:
        _, info = env.reset()
        max_progress = float(info["progress"])
        min_remaining = float(getattr(env._last_state, "remaining_distance", 0.0))
        final_info: dict[str, Any] = dict(info)
        terminated = False
        truncated = False
        for step in range(1, args.steps + 1):
            if args.action_mode == "direct":
                action = np.asarray([1.0, 0.0], dtype=np.float32)
            else:
                action = np.asarray([0.0, 0.0, 1.0, 1.0], dtype=np.float32)
            _, _, terminated, truncated, info = env.step(action)
            state = env._last_state
            max_progress = max(max_progress, float(info["progress"]))
            min_remaining = min(min_remaining, float(state.remaining_distance))
            final_info = dict(info)
            if terminated or truncated:
                break
        state = env._last_state
        backend = getattr(env.engine, "_engine", None)
        diagnostics = backend.diagnostics() if hasattr(backend, "diagnostics") else {}
        result = {
            "ok": bool(max_progress >= args.success_progress),
            "phantom": args.phantom,
            "route_target": args.route_target,
            "physics_engine": args.physics_engine,
            "action_mode": args.action_mode,
            "steps": int(env._step_count),
            "terminated": bool(terminated),
            "truncated": bool(truncated),
            "termination_reason": final_info.get("termination_reason"),
            "success_progress": args.success_progress,
            "final_progress": float(final_info["progress"]),
            "max_progress": max_progress,
            "final_remaining_m": float(state.remaining_distance),
            "min_remaining_m": min_remaining,
            "contact_force": float(state.contact_force),
            "safety_status": state.safety_status,
            "newton_params": newton_params,
            "diagnostics": diagnostics,
        }
        print(json.dumps(result, ensure_ascii=False, indent=2))
        return 0 if result["ok"] else 1
    finally:
        env.close()


if __name__ == "__main__":
    raise SystemExit(main())
