from cathsim.rl.config_manager import Config
from cathsim.rl.env_utils import make_gym_env


def train(*args, **kwargs):
    """Lazy wrapper for the legacy dm_control training entrypoint."""
    from cathsim.rl.train import train as _train

    return _train(*args, **kwargs)


def train_navigation(*args, **kwargs):
    """Lazy wrapper for the NavigationGym PPO/SAC training entrypoint."""
    from cathsim.rl.navigation_train import train_navigation as _train_navigation

    return _train_navigation(*args, **kwargs)
