"""Risk assessment for guidewire navigation.

Implements the multi-metric weighted risk model from doc/03-API与通信协议.md §3.3
and doc/01-总体技术方案.md §3.3. The assessor is stateless: it reads a
``NavigationState`` and returns a normalized risk score plus per-metric levels.

Units follow ``NavigationState`` (MuJoCo): distances in meters, curvature in
m^-1, velocity in m/s. Default thresholds are the spec's millimeter values
converted to these units (documented inline).
"""

from __future__ import annotations

from typing import TYPE_CHECKING, Literal

if TYPE_CHECKING:
    from services.navigation_engine import NavigationState

RiskLevel = Literal["SAFE", "WARNING", "CRITICAL"]
SafetyStatus = Literal["STANDBY", "SAFE_NAV", "DANGER_WARNING", "COLLISION_STOP"]

# Map an internal risk level to the protocol safety status.
_LEVEL_TO_STATUS: dict[RiskLevel, SafetyStatus] = {
    "SAFE": "SAFE_NAV",
    "WARNING": "DANGER_WARNING",
    "CRITICAL": "COLLISION_STOP",
}


class RiskAssessor:
    """Weighted multi-metric risk assessor.

    Metrics and default weights (doc §3.3):

    | Metric        | Weight | Direction      |
    |---------------|--------|----------------|
    | wall_distance | 0.4    | lower = worse  |
    | curvature     | 0.3    | higher = worse |
    | velocity      | 0.2    | higher = worse |
    | deviation     | 0.1    | higher = worse |

    Thresholds are (safe, warning, critical) in state units. Defaults convert the
    spec's mm values: wall 1.0/0.5/0.3 mm, curvature 0.10/0.15/0.20 mm^-1
    (= 100/150/200 m^-1), velocity 5/8/10 mm/s, deviation 1/2/3 mm.
    """

    DEFAULT_WEIGHTS = {
        "wall_distance": 0.4,
        "curvature": 0.3,
        "velocity": 0.2,
        "deviation": 0.1,
    }

    DEFAULT_THRESHOLDS = {
        # lower is worse
        "wall_distance": {"safe": 0.001, "warning": 0.0005, "critical": 0.0003},
        # higher is worse
        "curvature": {"safe": 100.0, "warning": 150.0, "critical": 200.0},
        "velocity": {"safe": 0.005, "warning": 0.008, "critical": 0.010},
        "deviation": {"safe": 0.001, "warning": 0.002, "critical": 0.003},
    }

    # Metrics where a lower value indicates higher risk.
    _LOWER_IS_WORSE = frozenset({"wall_distance"})

    def __init__(
        self,
        weights: dict[str, float] | None = None,
        thresholds: dict[str, dict[str, float]] | None = None,
    ) -> None:
        self.weights = dict(weights or self.DEFAULT_WEIGHTS)
        self.thresholds = {
            name: dict(values)
            for name, values in (thresholds or self.DEFAULT_THRESHOLDS).items()
        }

    def assess(self, state: NavigationState) -> dict:
        """Assess the risk of a navigation state.

        Returns a dict with:
            - ``risk_score``: weighted overall risk in [0, 1]
            - ``risk_level``: worst per-metric level (SAFE/WARNING/CRITICAL)
            - ``safety_status``: protocol safety status mapped from risk_level
            - ``metrics``: per-metric {value, risk, level}
        """
        if state.episode_length == 0:
            return {
                "risk_score": 0.0,
                "risk_level": "SAFE",
                "safety_status": "STANDBY",
                "metrics": {},
            }

        metric_values = {
            "wall_distance": state.wall_distance,
            "curvature": state.curvature,
            "velocity": state.velocity,
            "deviation": state.path_deviation,
        }

        metrics: dict[str, dict] = {}
        weighted_sum = 0.0
        total_weight = 0.0
        worst_rank = 0
        rank_to_level: list[RiskLevel] = ["SAFE", "WARNING", "CRITICAL"]

        for name, value in metric_values.items():
            thr = self.thresholds[name]
            risk = self._metric_risk(name, value, thr)
            level = self._metric_level(name, value, thr)

            weight = self.weights.get(name, 0.0)
            weighted_sum += weight * risk
            total_weight += weight
            worst_rank = max(worst_rank, rank_to_level.index(level))

            metrics[name] = {
                "value": float(value),
                "risk": round(float(risk), 4),
                "level": level,
            }

        risk_score = weighted_sum / total_weight if total_weight > 0 else 0.0
        risk_level = rank_to_level[worst_rank]

        return {
            "risk_score": round(float(risk_score), 4),
            "risk_level": risk_level,
            "safety_status": _LEVEL_TO_STATUS[risk_level],
            "metrics": metrics,
        }

    def _metric_risk(self, name: str, value: float, thr: dict[str, float]) -> float:
        """Normalize a metric to a [0, 1] risk using safe/critical thresholds."""
        safe = thr["safe"]
        critical = thr["critical"]

        if name in self._LOWER_IS_WORSE:
            span = safe - critical
            if span <= 0:
                return 0.0
            return _clip01((safe - value) / span)

        span = critical - safe
        if span <= 0:
            return 0.0
        return _clip01((value - safe) / span)

    def _metric_level(self, name: str, value: float, thr: dict[str, float]) -> RiskLevel:
        """Classify a metric into SAFE / WARNING / CRITICAL bands."""
        safe = thr["safe"]
        warning = thr["warning"]

        if name in self._LOWER_IS_WORSE:
            if value >= safe:
                return "SAFE"
            if value >= warning:
                return "WARNING"
            return "CRITICAL"

        if value <= safe:
            return "SAFE"
        if value <= warning:
            return "WARNING"
        return "CRITICAL"


def _clip01(x: float) -> float:
    """Clamp a value to the [0, 1] range."""
    if x < 0.0:
        return 0.0
    if x > 1.0:
        return 1.0
    return float(x)
