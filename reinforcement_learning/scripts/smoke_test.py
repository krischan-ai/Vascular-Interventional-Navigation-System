from __future__ import annotations
import importlib
for name in ("torch", "gymnasium", "stable_baselines3", "tensorboard", "cathsim"):
    module = importlib.import_module(name)
    print(f"{name}: {getattr(module, '__version__', 'ok')}")
from cathsim.gym import envs as _registered_envs  # noqa: F401
from cathsim.rl.navigation_train import NavigationTrainConfig
config = NavigationTrainConfig(total_timesteps=8, tensorboard=False)
print(f"config: {config.run_name} -> {config.run_dir}")
print("RL_IMPORT_SMOKE_OK")
