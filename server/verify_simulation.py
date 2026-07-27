"""Verify the API/simulation-only image and its excluded RL frameworks."""

from __future__ import annotations

import importlib.util
import json


EXCLUDED_MODULES = (
    "torch",
    "stable_baselines3",
    "tensorboard",
    "gymnasium",
)


def main() -> int:
    unexpected = [
        module_name
        for module_name in EXCLUDED_MODULES
        if importlib.util.find_spec(module_name) is not None
    ]
    if unexpected:
        raise RuntimeError(
            "RL frameworks unexpectedly present: " + ", ".join(unexpected)
        )

    import newton
    import warp
    from cathsim.dm import make_dm_env
    from services.main import health

    health_result = health()
    health_data = (
        health_result.model_dump()
        if hasattr(health_result, "model_dump")
        else health_result.dict()
    )
    if health_data.get("status") != "ok":
        raise RuntimeError(f"Health check failed: {health_data}")

    print(
        json.dumps(
            {
                "status": "ok",
                "health": health_data,
                "newton": newton.__version__,
                "warp": warp.__version__,
                "mujoco_factory": make_dm_env.__name__,
                "excluded_modules": list(EXCLUDED_MODULES),
            },
            ensure_ascii=False,
            indent=2,
        )
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
