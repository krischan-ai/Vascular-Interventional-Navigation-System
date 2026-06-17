"""Unit tests for the RiskAssessor multi-metric risk model."""

import pytest

from services.navigation_engine import NavigationState
from services.risk_assessor import RiskAssessor


@pytest.fixture
def assessor() -> RiskAssessor:
    return RiskAssessor()


class TestRiskAssessorBands:
    """Per-metric classification and overall status."""

    def test_standby_when_episode_not_started(self, assessor):
        result = assessor.assess(NavigationState())
        assert result["safety_status"] == "STANDBY"
        assert result["risk_score"] == 0.0
        assert result["metrics"] == {}

    def test_all_safe(self, assessor):
        state = NavigationState(
            episode_length=5,
            wall_distance=0.05,
            curvature=10.0,
            velocity=0.003,
            path_deviation=0.0005,
        )
        result = assessor.assess(state)
        assert result["risk_level"] == "SAFE"
        assert result["safety_status"] == "SAFE_NAV"
        assert result["risk_score"] == 0.0
        for metric in result["metrics"].values():
            assert metric["level"] == "SAFE"

    def test_all_critical(self, assessor):
        state = NavigationState(
            episode_length=5,
            wall_distance=0.0002,
            curvature=250.0,
            velocity=0.02,
            path_deviation=0.005,
        )
        result = assessor.assess(state)
        assert result["risk_level"] == "CRITICAL"
        assert result["safety_status"] == "COLLISION_STOP"
        assert result["risk_score"] == pytest.approx(1.0)

    def test_worst_metric_drives_overall_level(self, assessor):
        # Everything safe except wall_distance, which is critical.
        state = NavigationState(
            episode_length=3,
            wall_distance=0.0002,
            curvature=10.0,
            velocity=0.003,
            path_deviation=0.0005,
        )
        result = assessor.assess(state)
        assert result["risk_level"] == "CRITICAL"
        assert result["metrics"]["wall_distance"]["level"] == "CRITICAL"
        assert result["metrics"]["curvature"]["level"] == "SAFE"
        # Weighted score is below 1 since only one metric contributes.
        assert 0.0 < result["risk_score"] < 1.0

    def test_warning_band_wall_distance(self, assessor):
        state = NavigationState(episode_length=2, wall_distance=0.0007)
        result = assessor.assess(state)
        assert result["metrics"]["wall_distance"]["level"] == "WARNING"

    def test_metric_risk_is_clamped(self, assessor):
        # Values beyond critical should clamp risk at 1.0, not overshoot.
        state = NavigationState(
            episode_length=1,
            wall_distance=-1.0,  # absurd, past critical
            curvature=10_000.0,
            velocity=10.0,
            path_deviation=10.0,
        )
        result = assessor.assess(state)
        for metric in result["metrics"].values():
            assert 0.0 <= metric["risk"] <= 1.0
        assert result["risk_score"] == pytest.approx(1.0)


class TestRiskAssessorConfig:
    """Custom weights and thresholds."""

    def test_custom_weights_normalized(self):
        assessor = RiskAssessor(weights={"wall_distance": 1.0})
        state = NavigationState(episode_length=1, wall_distance=0.0003)
        result = assessor.assess(state)
        # Only wall_distance weighted -> score equals its risk (=1.0 at critical).
        assert result["risk_score"] == pytest.approx(1.0)

    def test_weighted_average(self):
        assessor = RiskAssessor(
            weights={"wall_distance": 0.5, "curvature": 0.5},
        )
        # wall_distance critical (risk 1.0), curvature safe (risk 0.0).
        state = NavigationState(
            episode_length=1,
            wall_distance=0.0003,
            curvature=10.0,
            velocity=0.003,
            path_deviation=0.0005,
        )
        result = assessor.assess(state)
        assert result["risk_score"] == pytest.approx(0.5)
