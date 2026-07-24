from cathsim.gym import envs as _registered_envs  # noqa: F401
from cathsim.rl.navigation_train import main

if __name__ == "__main__":
    raise SystemExit(main())
