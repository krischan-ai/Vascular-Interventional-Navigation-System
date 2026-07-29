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


def test_evaluate_cli_passes_newton_params(monkeypatch, tmp_path, capsys):
    import gymnasium as gym
    from cathsim.rl import navigation_evaluate

    made = []

    class ClosableFakeEnv(FakeEnv):
        def close(self):
            self.closed = True

    def fake_make(*args, **kwargs):
        made.append((args, kwargs))
        return ClosableFakeEnv()

    monkeypatch.setattr(gym, "make", fake_make)
    monkeypatch.setattr(navigation_evaluate, "load_model", lambda *_: FakeModel())

    output = tmp_path / "eval.json"
    status = navigation_evaluate.main([
        "model.zip",
        "--algorithm", "sac",
        "--episodes", "1",
        "--newton-rod-length", "0.012",
        "--newton-free-len", "0.006",
        "--newton-max-slack", "0.006",
        "--newton-insertion-margin", "0.0",
        "--output", str(output),
    ])

    assert status == 0
    assert made[0][1]["newton_params"] == {
        "rod_length": 0.012,
        "free_len": 0.006,
        "max_slack": 0.006,
        "insertion_margin": 0.0,
    }
    assert output.exists()
    assert '"success_rate": 1.0' in output.read_text(encoding="utf-8")
    assert '"success_rate": 1.0' in capsys.readouterr().out
