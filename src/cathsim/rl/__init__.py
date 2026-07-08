from cathsim.rl.config_manager import Config
from cathsim.rl.env_utils import make_gym_env
from cathsim.rl.navigation_train import NavigationTrainConfig, train_navigation_ppo


def train(*args, **kwargs):
    """Lazy wrapper for the legacy dm_control training entrypoint."""
    from cathsim.rl.train import train as _train

    return _train(*args, **kwargs)
