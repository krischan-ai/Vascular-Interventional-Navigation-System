from cathsim.gym.envs.cathsim import CathSim
from cathsim.gym.envs.navigation import NavigationGymEnv
from gymnasium.envs.registration import register


register(
    id="cathsim/CathSim-v0",
    entry_point="cathsim.gym.envs:CathSim",
    max_episode_steps=300,
    nondeterministic=True,
)

register(
    id="cathsim/NavigationGym-v0",
    entry_point="cathsim.gym.envs:NavigationGymEnv",
    max_episode_steps=300,
    nondeterministic=True,
)
