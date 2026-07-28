import numpy as np

from cathsim.rl.data import Trajectory, TrajectoriesDataset


def test_trajectory_dataset_round_trip(tmp_path):
    for trajectory_index in range(2):
        trajectory = Trajectory()
        for step in range(3):
            trajectory.add_transition(
                head_pos=np.array([trajectory_index, step, step + 1], dtype=float)
            )
        trajectory.to_array().save(tmp_path / f"trajectory_{trajectory_index}.pkl")

    dataset = TrajectoriesDataset(tmp_path, lazy_load=False)
    (start, goal), path = dataset[0]

    assert len(dataset) == 2
    assert start.shape == goal.shape == (3,)
    assert path.shape == (300, 3)
