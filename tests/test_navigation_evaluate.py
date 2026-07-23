import numpy as np


class FakeModel:
    def predict(self, obs, deterministic=True):
        assert deterministic is True
        return np.zeros(2, dtype=np.float32), None


class FakeEnv:
    def reset(self, seed=None):
        self.step_count = 0
        return {"x": np.zeros(1, dtype=np.float32)}, {}

    def step(self, action):
        self.step_count += 1
        done = self.step_count == 3
        progress = self.step_count / 3
        return (
            {"x": np.asarray([progress], dtype=np.float32)},
            1.0,
            done,
            False,
            {"progress": progress, "termination_reason": "success" if done else "running",
             "costs": {"contact": 0.5}},
        )


def test_evaluate_navigation_model_reports_success_and_costs():
    from cathsim.rl.navigation_evaluate import evaluate_navigation_model

    result = evaluate_navigation_model(FakeModel(), FakeEnv(), episodes=2, seed=10)

    assert result["success_rate"] == 1.0
    assert result["termination_reasons"] == {"success": 2}
    assert result["reward"]["mean"] == 3.0
    assert result["steps"]["mean"] == 3.0
    assert result["final_progress"]["mean"] == 1.0
    assert result["contact_integral"]["mean"] == 1.5
