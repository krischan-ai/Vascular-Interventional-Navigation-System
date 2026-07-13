from types import SimpleNamespace

from tools.audit_large_curvature import ScenarioMetrics, build_report, update_metrics, verdict


def _state(**kwargs):
    defaults = {
        "path_progress": 0.0,
        "path_deviation": 0.0,
        "wall_distance": 0.05,
        "contact_force": 0.0,
        "risk_score": 0.0,
        "safety_status": "SAFE_NAV",
        "flow_guidance": {"workflow": {"phase": "MICRO_ADVANCE"}},
    }
    defaults.update(kwargs)
    return SimpleNamespace(**defaults)


def test_update_metrics_accumulates_state_and_diagnostics():
    metrics = ScenarioMetrics(route_target="endpoints_3")

    update_metrics(
        metrics,
        _state(
            path_progress=0.2,
            path_deviation=0.001,
            wall_distance=0.002,
            contact_force=0.5,
            risk_score=0.2,
            safety_status="DANGER_WARNING",
            flow_guidance={"workflow": {"phase": "WALL_SLIDE"}},
        ),
        {
            "slack_m": 0.006,
            "max_breach_m": 0.0001,
            "feed_budget_m": 0.008,
        },
    )
    update_metrics(
        metrics,
        _state(
            path_progress=0.35,
            path_deviation=0.003,
            wall_distance=0.0002,
            contact_force=3.0,
            risk_score=0.8,
            safety_status="COLLISION_STOP",
            flow_guidance={"workflow": {"phase": "STRATEGY_SWITCH"}},
        ),
        {
            "slack_m": 0.012,
            "max_breach_m": 0.0004,
            "feed_budget_m": 0.0,
        },
    )

    assert metrics.final_progress == 0.35
    assert metrics.max_progress == 0.35
    assert metrics.max_path_deviation_m == 0.003
    assert metrics.min_wall_distance_m == 0.0002
    assert metrics.max_contact_force == 3.0
    assert metrics.max_risk_score == 0.8
    assert metrics.warning_count == 1
    assert metrics.stop_count == 1
    assert metrics.max_slack_m == 0.012
    assert metrics.max_breach_m == 0.0004
    assert metrics.min_feed_budget_m == 0.0
    assert metrics.final_phase == "STRATEGY_SWITCH"


def test_verdict_fails_on_low_progress_stop_and_breach():
    metrics = ScenarioMetrics(
        route_target="endpoints_3",
        max_progress=0.1,
        stop_count=1,
        max_breach_m=0.0004,
    )

    result = verdict(metrics, min_progress=0.25)

    assert result["passed"] is False
    assert result["failed_reasons"] == [
        "insufficient_progress",
        "collision_stop",
        "breach_over_0.3mm",
    ]


def test_build_report_counts_passed_and_failed_scenarios():
    args = SimpleNamespace(
        phantom="aorta_tree",
        target="root",
        physics_engine="newton_demo",
        autopilot=False,
        steps=10,
        push=0.5,
        rotate=0.0,
        base_push=0.35,
        lookahead=0.025,
        min_progress=0.25,
    )
    passing = ScenarioMetrics(route_target="endpoints_1", max_progress=0.4)
    failing = ScenarioMetrics(route_target="endpoints_2", max_progress=0.1)

    report = build_report(args, [passing, failing])

    assert report["kind"] == "large_curvature_audit"
    assert report["summary"] == {
        "scenario_count": 2,
        "passed_count": 1,
        "failed_count": 1,
    }
    assert report["scenarios"][0]["verdict"]["passed"] is True
    assert report["scenarios"][1]["verdict"]["failed_reasons"] == ["insufficient_progress"]
