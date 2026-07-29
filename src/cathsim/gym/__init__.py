"""Gymnasium environments exposed by CathSim.

Importing :mod:`cathsim.gym` registers public IDs without initializing a
physics or display backend.
"""

from gymnasium.envs.registration import register, registry


def _register(id_: str, entry_point: str) -> None:
    if id_ not in registry:
        register(
            id=id_,
            entry_point=entry_point,
            nondeterministic=True,
        )


_register("cathsim/CathSim-v0", "cathsim.gym.envs:CathSim")
_register("cathsim/NavigationGym-v0", "cathsim.gym.envs:NavigationGymEnv")
