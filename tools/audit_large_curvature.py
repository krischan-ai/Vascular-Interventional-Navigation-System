from __future__ import annotations

import argparse
import json
import os
import sys
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

_PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(_PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(_PROJECT_ROOT))


@dataclass
class ScenarioMetrics:
    route_target: str | None
    steps: int = 0
    final_progress: float = 0.0
    max_progress: float = 0.0
    max_path_deviation_m: float = 0.0
    min_wall_distance_m: float | None = None
    max_contact_force: float = 0.0
    max_risk_score: float = 0.0
    stop_count: int = 0
    warning_count: int = 0
    max_slack_m: float | None = None
    max_breach_m: float | None = None
    min_feed_budget_m: float | None = None
    final_safety_status: str = "STANDBY"
    final_phase: str | None = None
    phase_counts: dict[str, int] = field(default_factory=dict)
    final_tip_shape: str | None = None
    final_support_state: str | None = None
    final_training_score: int | None = None
    min_training_score: int | None = None
    min_component_scores: dict[str, int] = field(default_factory=dict)
    deduction_counts: dict[str, int] = field(default_factory=dict)
    strategy_state_counts: dict[str, int] = field(default_factory=dict)
    strategy_action_counts: dict[str, int] = field(default_factory=dict)
    hard_push_warning_count: int = 0
    max_hard_push_score: float = 0.0
    wall_slide_state_counts: dict[str, int] = field(default_factory=dict)
    max_normal_poking_score: float | None = None
    max_tangential_slide_score: float | None = None
    flow_guidance_missing_count: int = 0
    elapsed_s: float = 0.0
    error: str | None = None
    diagnostics: dict[str, Any] = field(default_factory=dict)

    def as_dict(self) -> dict[str, Any]:
        return {
            "route_target": self.route_target,
            "steps": self.steps,
            "final_progress": self.final_progress,
            "max_progress": self.max_progress,
            "max_path_deviation_m": self.max_path_deviation_m,
            "min_wall_distance_m": self.min_wall_distance_m,
            "max_contact_force": self.max_contact_force,
            "max_risk_score": self.max_risk_score,
            "stop_count": self.stop_count,
            "warning_count": self.warning_count,
            "max_slack_m": self.max_slack_m,
            "max_breach_m": self.max_breach_m,
            "min_feed_budget_m": self.min_feed_budget_m,
            "final_safety_status": self.final_safety_status,
            "final_phase": self.final_phase,
            "phase_counts": self.phase_counts,
            "final_tip_shape": self.final_tip_shape,
            "final_support_state": self.final_support_state,
            "final_training_score": self.final_training_score,
            "min_training_score": self.min_training_score,
            "min_component_scores": self.min_component_scores,
            "deduction_counts": self.deduction_counts,
            "strategy_state_counts": self.strategy_state_counts,
            "strategy_action_counts": self.strategy_action_counts,
            "hard_push_warning_count": self.hard_push_warning_count,
            "max_hard_push_score": self.max_hard_push_score,
            "wall_slide_state_counts": self.wall_slide_state_counts,
            "max_normal_poking_score": self.max_normal_poking_score,
            "max_tangential_slide_score": self.max_tangential_slide_score,
            "flow_guidance_missing_count": self.flow_guidance_missing_count,
            "elapsed_s": self.elapsed_s,
            "error": self.error,
            "diagnostics": self.diagnostics,
        }


def update_metrics(metrics: ScenarioMetrics, state, diagnostics: dict[str, Any] | None = None) -> None:
    """Accumulate one simulated state into a scenario metrics object."""
    metrics.final_progress = float(state.path_progress)
    metrics.max_progress = max(metrics.max_progress, float(state.path_progress))
    metrics.max_path_deviation_m = max(metrics.max_path_deviation_m, float(state.path_deviation))
    wall_distance = float(state.wall_distance)
    metrics.min_wall_distance_m = (
        wall_distance
        if metrics.min_wall_distance_m is None
        else min(metrics.min_wall_distance_m, wall_distance)
    )
    metrics.max_contact_force = max(metrics.max_contact_force, float(state.contact_force))
    metrics.max_risk_score = max(metrics.max_risk_score, float(state.risk_score))
    metrics.final_safety_status = str(state.safety_status)
    if state.safety_status == "COLLISION_STOP":
        metrics.stop_count += 1
    elif state.safety_status == "DANGER_WARNING":
        metrics.warning_count += 1
    flow = getattr(state, "flow_guidance", {}) or {}
    if not isinstance(flow, dict) or not flow:
        metrics.flow_guidance_missing_count += 1
        flow = {}
    workflow = flow.get("workflow", {}) if isinstance(flow, dict) else {}
    if isinstance(workflow, dict):
        phase = workflow.get("phase")
        metrics.final_phase = phase
        if phase:
            _bump(metrics.phase_counts, str(phase))

    tip_shape = flow.get("tip_shape", {}) if isinstance(flow, dict) else {}
    if isinstance(tip_shape, dict):
        shape = tip_shape.get("shape_type") or tip_shape.get("tip_shape")
        if shape is not None:
            metrics.final_tip_shape = str(shape)

    support = flow.get("support", {}) if isinstance(flow, dict) else {}
    if isinstance(support, dict):
        support_state = support.get("support_state")
        if support_state is not None:
            metrics.final_support_state = str(support_state)

    micro = flow.get("micro_advance", {}) if isinstance(flow, dict) else {}
    if isinstance(micro, dict):
        hard_push_score = _float_or_none(micro.get("hard_push_score"))
        if hard_push_score is not None:
            metrics.max_hard_push_score = max(metrics.max_hard_push_score, hard_push_score)
        if micro.get("hard_push_state") in {"warning", "unsafe", "excessive"}:
            metrics.hard_push_warning_count += 1

    wall = flow.get("wall_slide", {}) if isinstance(flow, dict) else {}
    if isinstance(wall, dict):
        wall_state = wall.get("wall_slide_state")
        if wall_state is not None:
            _bump(metrics.wall_slide_state_counts, str(wall_state))
        _max_optional(metrics, "max_normal_poking_score", wall.get("normal_poking_score"))
        _max_optional(metrics, "max_tangential_slide_score", wall.get("tangential_slide_score"))

    strategy = flow.get("strategy_switch", {}) if isinstance(flow, dict) else {}
    if isinstance(strategy, dict):
        strategy_state = strategy.get("strategy_switch_state")
        if strategy_state is not None:
            _bump(metrics.strategy_state_counts, str(strategy_state))
        for action in strategy.get("recommended_actions", []) or []:
            _bump(metrics.strategy_action_counts, str(action))

    training = flow.get("training_score", {}) if isinstance(flow, dict) else {}
    if isinstance(training, dict):
        overall = _int_or_none(training.get("overall"))
        if overall is not None:
            metrics.final_training_score = overall
            metrics.min_training_score = (
                overall if metrics.min_training_score is None else min(metrics.min_training_score, overall)
            )
        components = training.get("components", {})
        if isinstance(components, dict):
            for key, value in components.items():
                component_score = _int_or_none(value)
                if component_score is None:
                    continue
                current = metrics.min_component_scores.get(str(key))
                metrics.min_component_scores[str(key)] = (
                    component_score if current is None else min(current, component_score)
                )
        for reason in training.get("deductions", []) or []:
            _bump(metrics.deduction_counts, str(reason))

    if diagnostics:
        metrics.diagnostics = dict(diagnostics)
        _max_optional(metrics, "max_slack_m", diagnostics.get("slack_m"))
        _max_optional(metrics, "max_breach_m", diagnostics.get("max_breach_m"))
        _min_optional(metrics, "min_feed_budget_m", diagnostics.get("feed_budget_m"))


def verdict(
    metrics: ScenarioMetrics,
    *,
    min_progress: float,
    max_stop_count: int = 0,
    min_training_score: int | None = None,
    require_flow_guidance: bool = False,
) -> dict[str, Any]:
    """Return a simple pass/fail verdict for a large-curvature scenario."""
    failed_reasons: list[str] = []
    if metrics.error:
        failed_reasons.append("scenario_error")
    if metrics.max_progress < min_progress:
        failed_reasons.append("insufficient_progress")
    if metrics.stop_count > max_stop_count:
        failed_reasons.append("collision_stop")
    if metrics.max_breach_m is not None and metrics.max_breach_m > 0.0003:
        failed_reasons.append("breach_over_0.3mm")
    if require_flow_guidance and metrics.flow_guidance_missing_count > 0:
        failed_reasons.append("flow_guidance_missing")
    if (
        min_training_score is not None
        and metrics.min_training_score is not None
        and metrics.min_training_score < min_training_score
    ):
        failed_reasons.append("training_score_below_threshold")
    return {
        "passed": not failed_reasons,
        "failed_reasons": failed_reasons,
        "min_progress": min_progress,
        "max_stop_count": max_stop_count,
        "min_training_score": min_training_score,
        "require_flow_guidance": require_flow_guidance,
    }


def run_scenario(args: argparse.Namespace, route_target: str | None) -> ScenarioMetrics:
    from services.navigation_engine import NavigationEngine

    metrics = ScenarioMetrics(route_target=route_target)
    t0 = time.perf_counter()
    engine = None
    try:
        engine = NavigationEngine(
            phantom=args.phantom,
            target=args.target,
            guided=False,
            physics_engine=args.physics_engine,
            route_target=route_target,
            n_bodies=args.n_bodies,
            n_substeps=args.n_substeps,
            insertion_max=args.insertion_max,
            prethread=args.prethread,
        )
        if route_target is not None and route_target not in engine.available_routes:
            raise ValueError(
                f"unknown route target {route_target!r}; "
                f"available={sorted(engine.available_routes)[:20]}"
            )
        state = engine.reset()
        backend = getattr(engine, "_engine", None)
        update_metrics(metrics, state, _diagnostics(backend))

        autopilot = None
        if args.autopilot:
            from services.physics_autopilot import AutopilotConfig, PhysicsAutopilot

            autopilot = PhysicsAutopilot(
                engine.planned_path,
                config=AutopilotConfig(base_push=args.base_push, lookahead_m=args.lookahead),
            )

        for _ in range(args.steps):
            if autopilot is not None:
                push, rotate = autopilot.compute(
                    state.tip_position,
                    state.tip_direction,
                    state.contact_force,
                )
            else:
                push, rotate = args.push, args.rotate
            state = engine.step(push, rotate)
            metrics.steps += 1
            update_metrics(metrics, state, _diagnostics(backend))

    except Exception as exc:  # noqa: BLE001 - report scenario availability/failure
        metrics.error = f"{type(exc).__name__}: {exc}"
    finally:
        metrics.elapsed_s = time.perf_counter() - t0
        if engine is not None:
            engine.close()
    return metrics


def build_report(args: argparse.Namespace, scenario_metrics: list[ScenarioMetrics]) -> dict[str, Any]:
    scenarios = []
    for metrics in scenario_metrics:
        data = metrics.as_dict()
        data["verdict"] = verdict(
            metrics,
            min_progress=args.min_progress,
            min_training_score=args.min_training_score,
            require_flow_guidance=args.require_flow_guidance,
        )
        scenarios.append(data)
    return {
        "schema_version": 2,
        "kind": "large_curvature_audit",
        "phantom": args.phantom,
        "target": args.target,
        "physics_engine": args.physics_engine,
        "autopilot": args.autopilot,
        "controls": {
            "steps": args.steps,
            "push": args.push,
            "rotate": args.rotate,
            "base_push": args.base_push,
            "lookahead_m": args.lookahead,
        },
        "thresholds": {
            "min_progress": args.min_progress,
            "max_stop_count": 0,
            "max_breach_m": 0.0003,
            "min_training_score": args.min_training_score,
            "require_flow_guidance": args.require_flow_guidance,
        },
        "scenarios": scenarios,
        "summary": {
            "scenario_count": len(scenarios),
            "passed_count": sum(1 for item in scenarios if item["verdict"]["passed"]),
            "failed_count": sum(1 for item in scenarios if not item["verdict"]["passed"]),
        },
    }


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Run large-curvature Newton scenario audits.")
    parser.add_argument("--phantom", default="aorta_tree")
    parser.add_argument("--target", default="root")
    parser.add_argument("--physics-engine", default="newton_demo")
    parser.add_argument("--routes", nargs="*", default=["endpoint_18", "endpoint_21", "endpoint_15"])
    parser.add_argument("--steps", type=int, default=240)
    parser.add_argument("--push", type=float, default=0.7)
    parser.add_argument("--rotate", type=float, default=0.0)
    parser.add_argument("--autopilot", action="store_true")
    parser.add_argument("--base-push", type=float, default=0.35)
    parser.add_argument("--lookahead", type=float, default=0.025)
    parser.add_argument("--n-bodies", type=int, default=80)
    parser.add_argument("--n-substeps", type=int, default=None)
    parser.add_argument("--insertion-max", type=float, default=0.2)
    parser.add_argument("--prethread", action="store_true")
    parser.add_argument("--min-progress", type=float, default=0.25)
    parser.add_argument("--min-training-score", type=int, default=None)
    parser.add_argument("--require-flow-guidance", action="store_true")
    parser.add_argument("--output", type=Path, default=Path("data") / "large_curvature_audit.json")
    return parser.parse_args()


def main() -> int:
    os.environ.setdefault("MUJOCO_GL", "glfw")
    args = parse_args()
    metrics = [run_scenario(args, route) for route in args.routes]
    report = build_report(args, metrics)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(
        json.dumps(report, ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
    )
    print(f"Wrote large-curvature audit: {args.output}")
    print(json.dumps(report["summary"], ensure_ascii=False, indent=2))
    return 0 if report["summary"]["failed_count"] == 0 else 1


def _diagnostics(backend) -> dict[str, Any]:
    if backend is not None and hasattr(backend, "diagnostics"):
        return backend.diagnostics()
    return {}


def _max_optional(metrics: ScenarioMetrics, attr: str, value: Any) -> None:
    try:
        val = float(value)
    except (TypeError, ValueError):
        return
    current = getattr(metrics, attr)
    setattr(metrics, attr, val if current is None else max(current, val))


def _min_optional(metrics: ScenarioMetrics, attr: str, value: Any) -> None:
    try:
        val = float(value)
    except (TypeError, ValueError):
        return
    current = getattr(metrics, attr)
    setattr(metrics, attr, val if current is None else min(current, val))


def _bump(counts: dict[str, int], key: str) -> None:
    counts[key] = counts.get(key, 0) + 1


def _float_or_none(value: Any) -> float | None:
    try:
        return float(value)
    except (TypeError, ValueError):
        return None


def _int_or_none(value: Any) -> int | None:
    try:
        return int(value)
    except (TypeError, ValueError):
        return None


if __name__ == "__main__":
    raise SystemExit(main())
