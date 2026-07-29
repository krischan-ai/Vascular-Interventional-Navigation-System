from pathlib import Path


def _evaluation(success_rate: float, progress: float = 0.5) -> dict:
    return {
        "success_rate": success_rate,
        "final_progress": {"mean": progress},
        "wall_contact_steps": {"mean": 0.0},
        "max_penetration_m": {"max": 0.0},
    }


def _arguments(model_path: Path) -> dict:
    return {
        "model_path": model_path,
        "algorithm": "ppo",
        "phantom": "aorta_tree",
        "routes": ["endpoint_0", "endpoint_18"],
        "baseline_route": "endpoint_0",
        "episodes": 2,
        "seed": 1000,
        "randomized_episodes": 2,
        "physics_engine": "newton",
        "max_episode_steps": 300,
        "newton_params": {"rod_length": 0.012},
        "penetrations_mm": [-0.5, 0.0, 0.1, 0.3],
        "lumen_radius_mm": 3.0,
        "rod_radius_mm": 0.27,
        "contact_ke": 3_000_000.0,
        "baseline_success_threshold": 1.0,
    }


def test_frozen_matrix_gates_baseline_and_characterizes_transfer(
    monkeypatch, tmp_path
):
    from scripts import run_navigation_frozen_matrix as matrix

    model_path = tmp_path / "model.zip"
    model_path.write_bytes(b"model")
    calls = []

    def fake_evaluate(*_, route_target, domain_randomization=False, **__):
        calls.append((route_target, domain_randomization))
        if domain_randomization:
            return _evaluation(0.5, 0.9)
        if route_target == "endpoint_0":
            return _evaluation(1.0, 0.98)
        return _evaluation(0.0, 0.4)

    monkeypatch.setattr(matrix, "_evaluate_route", fake_evaluate)
    monkeypatch.setattr(
        matrix,
        "_run_contact_probe",
        lambda **_: {
            "status": "passed_with_warnings",
            "checks": {"passed": True},
        },
    )

    result = matrix.run_frozen_matrix(object(), **_arguments(model_path))

    assert result["status"] == "passed"
    assert result["schema_version"] == "navigation_evaluation_matrix_v1"
    assert result["acceptance_gates"] == {
        "model_artifact_present": True,
        "baseline_success_rate": True,
        "contact_calibration": True,
    }
    assert result["nonbaseline_characterization"]["endpoint_18"][
        "success_rate"
    ] == 0.0
    assert result["randomized_route_results"]["endpoint_0"][
        "success_rate"
    ] == 0.5
    assert calls == [
        ("endpoint_0", False),
        ("endpoint_18", False),
        ("endpoint_0", True),
    ]


def test_frozen_matrix_fails_when_contact_probe_fails(monkeypatch, tmp_path):
    from scripts import run_navigation_frozen_matrix as matrix

    model_path = tmp_path / "model.zip"
    model_path.write_bytes(b"model")
    monkeypatch.setattr(
        matrix,
        "_evaluate_route",
        lambda *_, **__: _evaluation(1.0, 0.98),
    )
    monkeypatch.setattr(
        matrix,
        "_run_contact_probe",
        lambda **_: {"status": "failed", "checks": {"passed": False}},
    )

    result = matrix.run_frozen_matrix(object(), **_arguments(model_path))

    assert result["status"] == "failed"
    assert result["acceptance_gates"]["contact_calibration"] is False
