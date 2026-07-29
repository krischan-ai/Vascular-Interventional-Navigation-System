from __future__ import annotations

import argparse
import json
import os
from pathlib import Path
from typing import Any

import gymnasium as gym

import cathsim.gym  # noqa: F401 - registers cathsim/NavigationGym-v0
from cathsim.rl.navigation_evaluate import (
    EVALUATION_PROTOCOL_VERSION,
    _file_artifact,
    evaluate_navigation_model,
    load_model,
)
from scripts.run_newton_contact_calibration import (
    DEFAULT_PENETRATIONS_MM,
    _configure_reproducible_probe,
    _run_case,
    _validate,
)


MATRIX_SCHEMA_VERSION = "navigation_evaluation_matrix_v1"
MATRIX_PROTOCOL_VERSION = "navigation_frozen_matrix_v1"
_CONTACT_ENV_KEYS = (
    "CATHSIM_NEWTON_DRIVE",
    "CATHSIM_NEWTON_SETTLE_STEPS",
    "CATHSIM_NEWTON_JTIP_DEG",
    "CATHSIM_NEWTON_JTIP_BODIES",
    "CATHSIM_NEWTON_CONTACT_KE",
    "CATHSIM_NEWTON_ROD_RADIUS",
)


def _csv_strings(value: str) -> list[str]:
    values = [item.strip() for item in value.split(",") if item.strip()]
    if not values:
        raise argparse.ArgumentTypeError("at least one route is required")
    return values


def _csv_floats(value: str) -> list[float]:
    try:
        values = [float(item.strip()) for item in value.split(",") if item.strip()]
    except ValueError as exc:
        raise argparse.ArgumentTypeError(
            "values must be comma-separated numbers"
        ) from exc
    if not values:
        raise argparse.ArgumentTypeError("at least one value is required")
    return values


def _evaluate_route(
    model,
    *,
    phantom: str,
    route_target: str,
    episodes: int,
    seed: int,
    physics_engine: str,
    max_episode_steps: int,
    newton_params: dict[str, float],
    domain_randomization: bool = False,
) -> dict[str, Any]:
    env = gym.make(
        "cathsim/NavigationGym-v0",
        phantom=phantom,
        route_target=route_target,
        action_mode="shape_intent",
        physics_engine=physics_engine,
        max_episode_steps=max_episode_steps,
        newton_params=newton_params,
        domain_randomization=domain_randomization,
    )
    try:
        return evaluate_navigation_model(
            model,
            env,
            episodes=episodes,
            seed=seed,
        )
    finally:
        env.close()


def _run_contact_probe(
    *,
    penetrations_mm: list[float],
    lumen_radius_mm: float,
    rod_radius_mm: float,
    contact_ke: float,
) -> dict[str, Any]:
    previous_env = {key: os.environ.get(key) for key in _CONTACT_ENV_KEYS}
    try:
        _configure_reproducible_probe(contact_ke, rod_radius_mm / 1000.0)
        cases = [
            _run_case(
                penetration_mm / 1000.0,
                lumen_radius_m=lumen_radius_mm / 1000.0,
                rod_radius_m=rod_radius_mm / 1000.0,
                contact_ke=contact_ke,
            )
            for penetration_mm in penetrations_mm
        ]
    finally:
        for key, previous in previous_env.items():
            if previous is None:
                os.environ.pop(key, None)
            else:
                os.environ[key] = previous

    checks = _validate(cases, rod_radius_mm=rod_radius_mm)
    warning_names = (
        "solver_force_is_monotonic",
        "deep_penetration_manifold_is_continuous",
    )
    has_warnings = not all(checks[name] for name in warning_names)
    status = "failed"
    if checks["passed"]:
        status = "passed_with_warnings" if has_warnings else "passed"
    return {
        "status": status,
        "probe": "newton_vbd_controlled_wall_contact_v1",
        "parameters": {
            "penetrations_mm": penetrations_mm,
            "lumen_radius_mm": lumen_radius_mm,
            "rod_radius_mm": rod_radius_mm,
            "contact_ke_n_per_m": contact_ke,
        },
        "cases": cases,
        "checks": checks,
    }


def run_frozen_matrix(
    model,
    *,
    model_path: Path,
    algorithm: str,
    phantom: str,
    routes: list[str],
    baseline_route: str,
    episodes: int,
    seed: int,
    randomized_episodes: int,
    physics_engine: str,
    max_episode_steps: int,
    newton_params: dict[str, float],
    penetrations_mm: list[float],
    lumen_radius_mm: float,
    rod_radius_mm: float,
    contact_ke: float,
    baseline_success_threshold: float,
) -> dict[str, Any]:
    if baseline_route not in routes:
        raise ValueError("baseline route must be included in routes")
    if episodes <= 0 or randomized_episodes < 0:
        raise ValueError("episodes must be positive and randomized episodes non-negative")

    route_results = {
        route: _evaluate_route(
            model,
            phantom=phantom,
            route_target=route,
            episodes=episodes,
            seed=seed,
            physics_engine=physics_engine,
            max_episode_steps=max_episode_steps,
            newton_params=newton_params,
        )
        for route in routes
    }
    randomized_results: dict[str, Any] = {}
    if randomized_episodes:
        randomized_results[baseline_route] = _evaluate_route(
            model,
            phantom=phantom,
            route_target=baseline_route,
            episodes=randomized_episodes,
            seed=seed,
            physics_engine=physics_engine,
            max_episode_steps=max_episode_steps,
            newton_params=newton_params,
            domain_randomization=True,
        )

    contact_calibration = _run_contact_probe(
        penetrations_mm=penetrations_mm,
        lumen_radius_mm=lumen_radius_mm,
        rod_radius_mm=rod_radius_mm,
        contact_ke=contact_ke,
    )
    artifact = _file_artifact(model_path)
    gates = {
        "model_artifact_present": artifact["sha256"] is not None,
        "baseline_success_rate": (
            float(route_results[baseline_route]["success_rate"])
            >= baseline_success_threshold
        ),
        "contact_calibration": bool(
            contact_calibration["checks"].get("passed", False)
        ),
    }
    characterization = {
        route: {
            "success_rate": float(result["success_rate"]),
            "final_progress_mean": float(result["final_progress"]["mean"]),
            "wall_contact_steps_mean": float(
                result["wall_contact_steps"]["mean"]
            ),
            "max_penetration_m": float(result["max_penetration_m"]["max"]),
        }
        for route, result in route_results.items()
        if route != baseline_route
    }
    return {
        "schema_version": MATRIX_SCHEMA_VERSION,
        "matrix_protocol_version": MATRIX_PROTOCOL_VERSION,
        "evaluation_protocol_version": EVALUATION_PROTOCOL_VERSION,
        "status": "passed" if all(gates.values()) else "failed",
        "model": {
            **artifact,
            "algorithm": algorithm,
        },
        "configuration": {
            "phantom": phantom,
            "routes": routes,
            "baseline_route": baseline_route,
            "episodes_per_route": episodes,
            "base_seed": seed,
            "randomized_episodes": randomized_episodes,
            "physics_engine": physics_engine,
            "max_episode_steps": max_episode_steps,
            "newton_params": newton_params,
            "baseline_success_threshold": baseline_success_threshold,
        },
        "acceptance_gates": gates,
        "route_results": route_results,
        "randomized_route_results": randomized_results,
        "contact_calibration": contact_calibration,
        "nonbaseline_characterization": characterization,
        "limitations": [
            "Only the baseline route is a release gate; other routes characterize transfer.",
            "Repeated evaluation seeds are not independent when domain randomization is disabled.",
            "Controlled VBD force is an engineering signal, not a clinically calibrated wall force.",
        ],
    }


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(
        description=(
            "Evaluate a navigation policy on frozen routes and the controlled "
            "Newton wall-contact probe."
        )
    )
    parser.add_argument("model", type=Path)
    parser.add_argument("--algorithm", choices=["ppo", "sac"], default="ppo")
    parser.add_argument(
        "--routes",
        type=_csv_strings,
        default=["endpoint_0", "endpoint_18"],
    )
    parser.add_argument("--baseline-route", default="endpoint_0")
    parser.add_argument("--episodes", type=int, default=5)
    parser.add_argument("--seed", type=int, default=1000)
    parser.add_argument("--randomized-episodes", type=int, default=0)
    parser.add_argument("--phantom", default="aorta_tree")
    parser.add_argument(
        "--physics-engine",
        choices=["newton", "mujoco", "guided"],
        default="newton",
    )
    parser.add_argument("--max-episode-steps", type=int, default=300)
    parser.add_argument("--newton-rod-length", type=float)
    parser.add_argument("--newton-free-len", type=float)
    parser.add_argument("--newton-max-slack", type=float)
    parser.add_argument("--newton-insertion-margin", type=float)
    parser.add_argument(
        "--penetrations-mm",
        type=_csv_floats,
        default=list(DEFAULT_PENETRATIONS_MM),
    )
    parser.add_argument("--lumen-radius-mm", type=float, default=3.0)
    parser.add_argument("--rod-radius-mm", type=float, default=0.27)
    parser.add_argument("--contact-ke", type=float, default=3_000_000.0)
    parser.add_argument("--baseline-success-threshold", type=float, default=1.0)
    parser.add_argument("--output", type=Path)
    args = parser.parse_args(argv)

    if not 0.0 <= args.baseline_success_threshold <= 1.0:
        parser.error("baseline-success-threshold must be in [0, 1]")
    if args.lumen_radius_mm <= args.rod_radius_mm or args.rod_radius_mm <= 0.0:
        parser.error("lumen radius must exceed a positive rod radius")
    if args.contact_ke <= 0.0:
        parser.error("contact-ke must be positive")

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
    result = run_frozen_matrix(
        load_model(args.model, args.algorithm),
        model_path=args.model,
        algorithm=args.algorithm,
        phantom=args.phantom,
        routes=args.routes,
        baseline_route=args.baseline_route,
        episodes=args.episodes,
        seed=args.seed,
        randomized_episodes=args.randomized_episodes,
        physics_engine=args.physics_engine,
        max_episode_steps=args.max_episode_steps,
        newton_params=newton_params,
        penetrations_mm=args.penetrations_mm,
        lumen_radius_mm=args.lumen_radius_mm,
        rod_radius_mm=args.rod_radius_mm,
        contact_ke=args.contact_ke,
        baseline_success_threshold=args.baseline_success_threshold,
    )
    text = json.dumps(result, ensure_ascii=False, indent=2, sort_keys=True) + "\n"
    if args.output is not None:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(text, encoding="utf-8")
    print(text, end="")
    return 0 if result["status"] == "passed" else 1


if __name__ == "__main__":
    raise SystemExit(main())
