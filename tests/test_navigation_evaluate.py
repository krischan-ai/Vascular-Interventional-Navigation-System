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
    assert result["schema_version"] == "navigation_evaluation_v3"
    assert result["evaluation_protocol_version"] == "navigation_frozen_eval_v2"
    assert result["rollout"]["episode_seeds"] == [10, 11]
    assert result["protocol"]["reward_version"] == "navigation_reward_v2"
    assert result["episode_results"][0]["domain_randomization"]["enabled"] is False


class RandomizedFakeEnv(FakeEnv):
    def reset(self, seed=None):
        obs, _ = super().reset(seed=seed)
        return obs, {
            "domain_randomization": {
                "enabled": True,
                "applied": True,
                "seed": seed,
                "requested": {"bend": float(seed)},
                "effective": {"bend": float(seed)},
            },
        }


def test_evaluate_navigation_model_records_per_episode_randomization():
    from cathsim.rl.navigation_evaluate import evaluate_navigation_model

    result = evaluate_navigation_model(
        FakeModel(), RandomizedFakeEnv(), episodes=2, seed=10
    )

    samples = [
        row["domain_randomization"]["requested"]["bend"]
        for row in result["episode_results"]
    ]
    assert samples == [10.0, 11.0]


class ContactMetricEnv(FakeEnv):
    def step(self, action):
        obs, reward, terminated, truncated, info = super().step(action)
        info["costs"] = {
            "contact": 10.0,
            "contact_impulse": 0.25,
            "wall_contact_count": 3,
            "max_penetration_m": 0.0002,
        }
        return obs, reward, terminated, truncated, info


def test_evaluate_navigation_model_uses_impulse_and_contact_events():
    from cathsim.rl.navigation_evaluate import evaluate_navigation_model

    result = evaluate_navigation_model(FakeModel(), ContactMetricEnv(), episodes=1, seed=10)

    assert result["contact_integral"]["mean"] == 0.75
    assert result["max_contact"]["mean"] == 10.0
    assert result["wall_contact_steps"]["mean"] == 3.0
    assert result["wall_contact_pair_samples"]["mean"] == 9.0
    assert result["max_penetration_m"]["mean"] == 0.0002
    assert result["metric_units"]["contact_integral"] == "N s"


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
    output_text = output.read_text(encoding="utf-8")
    assert '"success_rate": 1.0' in output_text
    assert '"sha256": null' in output_text
    assert '"success_rate": 1.0' in capsys.readouterr().out
