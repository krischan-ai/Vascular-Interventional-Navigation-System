"""Verify a CathSim source tree or generated standalone server bundle."""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path


REQUIRED_PATHS = (
    "services/main.py",
    "services/navigation_engine.py",
    "services/physics/base.py",
    "services/physics/newton_engine.py",
    "src/cathsim/dm",
    "src/cathsim/gym/envs/navigation.py",
    "src/cathsim/rl/navigation_train.py",
    "src/cathsim/rl/navigation_evaluate.py",
    "reinforcement_learning/configs",
    "reinforcement_learning/scripts/train_navigation.py",
    "reinforcement_learning/requirements-backend.txt",
    "reinforcement_learning/requirements-train.txt",
)

FORBIDDEN_BUNDLE_PATHS = ("godot_client", "docs", "doc")


def verify(root: Path, *, standalone: bool) -> dict[str, object]:
    root = root.resolve()
    missing = [rel for rel in REQUIRED_PATHS if not (root / rel).exists()]
    if missing:
        raise RuntimeError("Missing required server paths: " + ", ".join(missing))

    if standalone:
        forbidden = [rel for rel in FORBIDDEN_BUNDLE_PATHS if (root / rel).exists()]
        if forbidden:
            raise RuntimeError("Client/document paths leaked into bundle: " + ", ".join(forbidden))

    sys.path.insert(0, str(root))
    sys.path.insert(0, str(root / "src"))

    from services.main import health
    from cathsim.gym.envs.navigation import NavigationGymEnv
    from cathsim.rl.navigation_train import NavigationTrainConfig

    health_result = health()
    health_data = (
        health_result.model_dump()
        if hasattr(health_result, "model_dump")
        else health_result.dict()
    )
    if health_data.get("status") != "ok":
        raise RuntimeError(f"Health check failed: {health_data}")

    config = NavigationTrainConfig(total_timesteps=8, tensorboard=False)
    result = {
        "status": "ok",
        "root": str(root),
        "standalone": standalone,
        "health": health_data,
        "gym_env": NavigationGymEnv.__name__,
        "rl_run_name": config.run_name,
    }
    build_manifest = root / "SERVER_BUILD.json"
    if standalone and not build_manifest.is_file():
        raise RuntimeError("Standalone bundle is missing SERVER_BUILD.json")
    if build_manifest.is_file():
        build_data = json.loads(build_manifest.read_text(encoding="utf-8"))
        result["build"] = {
            key: build_data[key]
            for key in ("name", "built_at", "file_count", "total_bytes")
            if key in build_data
        }
    return result


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--root", type=Path, default=Path(__file__).resolve().parents[1])
    parser.add_argument(
        "--standalone",
        action="store_true",
        help="Also reject frontend/document directories and require SERVER_BUILD.json.",
    )
    args = parser.parse_args()
    print(json.dumps(verify(args.root, standalone=args.standalone), ensure_ascii=False, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
