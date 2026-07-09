from __future__ import annotations

import heapq
import json
import math
import time
import uuid
from dataclasses import dataclass, field
from pathlib import Path
from typing import Sequence

import numpy as np
from scipy import interpolate

from services.graph_loader import Graph, GraphLoader, Node3D


@dataclass(frozen=True)
class PathResult:
    path_id: str
    waypoints: list[Node3D]
    length_mm: float
    node_count: int
    compute_time_ms: float
    smooth_waypoints: list[Node3D] | None = None
    smooth_length_mm: float | None = None
    max_curvature: float | None = None
    radii: list[float] | None = None
    smooth_radii: list[float] | None = None

    def as_dict(self) -> dict:
        result = {
            "path_id": self.path_id,
            "waypoints": [list(point) for point in self.waypoints],
            "length_mm": self.length_mm,
            "node_count": self.node_count,
            "compute_time_ms": self.compute_time_ms,
        }
        if self.smooth_waypoints is not None:
            result["smooth_waypoints"] = [list(p) for p in self.smooth_waypoints]
            result["smooth_length_mm"] = self.smooth_length_mm
            result["max_curvature"] = self.max_curvature
        if self.radii is not None:
            result["radii"] = [float(r) for r in self.radii]
        if self.smooth_radii is not None:
            result["smooth_radii"] = [float(r) for r in self.smooth_radii]
        return result


def euclidean(a: Sequence[float], b: Sequence[float]) -> float:
    return math.sqrt(
        (float(a[0]) - float(b[0])) ** 2
        + (float(a[1]) - float(b[1])) ** 2
        + (float(a[2]) - float(b[2])) ** 2
    )


class PathPlanner:
    def __init__(self, graph_path: str | Path | None = None) -> None:
        self.loader = GraphLoader(graph_path) if graph_path is not None else GraphLoader()
        self._radii: dict[Node3D, float] = {}
        if graph_path is not None:
            self._load_radii_for_graph(Path(graph_path))

    @property
    def graph(self) -> Graph:
        return self.loader.graph

    @property
    def nodes(self) -> list[Node3D]:
        return self.loader.nodes

    def load(self, graph_path: str | Path) -> None:
        self.loader.load(graph_path)
        self._load_radii_for_graph(Path(graph_path))

    def find_nearest_node(self, position: Sequence[float]) -> Node3D:
        if not self.nodes:
            raise ValueError("PathPlanner graph is not loaded")
        return min(self.nodes, key=lambda node: euclidean(node, position))

    def plan(
        self,
        start: Sequence[float],
        end: Sequence[float],
        algorithm: str = "astar",
        smooth: bool = False,
        smooth_factor: float = 0.0,
        num_points: int | None = None,
    ) -> PathResult:
        """Plan a path from start to end.

        Args:
            start: Start position [x, y, z]
            end: End position [x, y, z]
            algorithm: Planning algorithm ("astar")
            smooth: Whether to apply B-spline smoothing
            smooth_factor: Smoothing factor for B-spline (0.0 = interpolating)
            num_points: Number of points in smoothed path (default: 2x original)

        Returns:
            PathResult with waypoints and optionally smooth_waypoints
        """
        if algorithm != "astar":
            raise ValueError(f"Unsupported planning algorithm: {algorithm}")
        if not self.graph:
            raise ValueError("PathPlanner graph is not loaded")

        start_time = time.perf_counter()
        start_node = self.find_nearest_node(start)
        end_node = self.find_nearest_node(end)
        waypoints = self._astar(start_node, end_node)

        smooth_waypoints = None
        smooth_length_mm = None
        max_curvature = None
        radii = self._radii_for_waypoints(waypoints)
        smooth_radii = None

        if smooth and len(waypoints) >= 4:
            smooth_result = self.smooth_path(
                waypoints,
                smoothing_factor=smooth_factor,
                num_points=num_points,
                radii=radii,
            )
            smooth_waypoints = smooth_result["waypoints"]
            smooth_length_mm = smooth_result["length_mm"]
            max_curvature = smooth_result["max_curvature"]
            smooth_radii = smooth_result.get("radii")

        elapsed_ms = (time.perf_counter() - start_time) * 1000.0

        return PathResult(
            path_id=str(uuid.uuid4()),
            waypoints=waypoints,
            length_mm=sum(
                euclidean(waypoints[index], waypoints[index + 1])
                for index in range(len(waypoints) - 1)
            ),
            node_count=len(waypoints),
            compute_time_ms=elapsed_ms,
            smooth_waypoints=smooth_waypoints,
            smooth_length_mm=smooth_length_mm,
            max_curvature=max_curvature,
            radii=radii,
            smooth_radii=smooth_radii,
        )

    def smooth_path(
        self,
        waypoints: list[Node3D],
        smoothing_factor: float = 0.0,
        num_points: int | None = None,
        degree: int = 3,
        radii: Sequence[float] | None = None,
    ) -> dict:
        """Apply B-spline smoothing to a path.

        Args:
            waypoints: List of 3D waypoints
            smoothing_factor: Controls smoothness (0.0 = interpolating spline)
            num_points: Number of output points (default: 2x input)
            degree: B-spline degree (1-5, default 3 = cubic)

        Returns:
            Dictionary with smoothed waypoints, length, and curvature
        """
        if len(waypoints) < 4:
            raise ValueError("Need at least 4 waypoints for B-spline smoothing")

        points = np.array([[p[0], p[1], p[2]] for p in waypoints])
        n_points = len(points)

        if num_points is None:
            num_points = n_points * 2

        cumulative_dist = np.zeros(n_points)
        for i in range(1, n_points):
            cumulative_dist[i] = cumulative_dist[i - 1] + np.linalg.norm(
                points[i] - points[i - 1]
            )

        if cumulative_dist[-1] < 1e-10:
            result = {
                "waypoints": [tuple(p) for p in points],
                "length_mm": 0.0,
                "max_curvature": 0.0,
            }
            if radii is not None and len(radii) == len(waypoints):
                result["radii"] = [float(r) for r in radii]
            return result

        u = cumulative_dist / cumulative_dist[-1]

        k = min(degree, n_points - 1)

        if smoothing_factor == 0.0:
            tck, _ = interpolate.splprep(
                [points[:, 0], points[:, 1], points[:, 2]],
                u=u,
                k=k,
                s=0,
            )
        else:
            s = smoothing_factor * n_points
            tck, _ = interpolate.splprep(
                [points[:, 0], points[:, 1], points[:, 2]],
                u=u,
                k=k,
                s=s,
            )

        u_new = np.linspace(0, 1, num_points)
        smooth_points = np.array(interpolate.splev(u_new, tck)).T

        smooth_waypoints = [tuple(p) for p in smooth_points]

        length_mm = sum(
            np.linalg.norm(smooth_points[i + 1] - smooth_points[i])
            for i in range(len(smooth_points) - 1)
        )

        max_curvature = self._compute_max_curvature(tck, u_new)

        result = {
            "waypoints": smooth_waypoints,
            "length_mm": float(length_mm),
            "max_curvature": float(max_curvature),
        }
        if radii is not None and len(radii) == len(waypoints):
            result["radii"] = [
                float(r) for r in np.interp(u_new, u, np.asarray(radii, dtype=np.float64))
            ]
        return result

    def _compute_max_curvature(self, tck, u_values: np.ndarray) -> float:
        """Compute maximum curvature along a B-spline curve."""
        d1 = np.array(interpolate.splev(u_values, tck, der=1)).T
        d2 = np.array(interpolate.splev(u_values, tck, der=2)).T

        curvatures = []
        for i in range(len(u_values)):
            v1 = d1[i]
            v2 = d2[i]
            cross = np.cross(v1, v2)
            norm_v1 = np.linalg.norm(v1)
            if norm_v1 > 1e-10:
                kappa = np.linalg.norm(cross) / (norm_v1 ** 3)
                curvatures.append(kappa)

        return max(curvatures) if curvatures else 0.0

    def _load_radii_for_graph(self, graph_path: Path) -> None:
        """Load optional per-node lumen radii next to ``graph.json``.

        VPP stores graph coordinates and node radii in millimeters. The planner
        keeps those native units; callers that convert waypoints to meters must
        convert radii at the same boundary.
        """
        self._radii = {}
        radii_path = graph_path.parent / "node_radii.json"
        if not radii_path.is_file():
            return
        try:
            raw = json.loads(radii_path.read_text(encoding="utf-8"))
            radii = raw.get("radii", {}) if isinstance(raw, dict) else {}
            if not isinstance(radii, dict):
                return
            self._radii = {
                self.loader.node_for_key(key): float(value)
                for key, value in radii.items()
                if key in self.loader._key_to_node
            }
        except Exception:
            self._radii = {}

    def _radii_for_waypoints(self, waypoints: list[Node3D]) -> list[float] | None:
        if not self._radii:
            return None
        values = [self._radii.get(point) for point in waypoints]
        if any(value is None for value in values):
            return None
        return [float(value) for value in values]

    def _astar(self, start: Node3D, goal: Node3D) -> list[Node3D]:
        frontier: list[tuple[float, int, Node3D]] = []
        counter = 0
        heapq.heappush(frontier, (0.0, counter, start))

        came_from: dict[Node3D, Node3D | None] = {start: None}
        cost_so_far: dict[Node3D, float] = {start: 0.0}

        while frontier:
            _, _, current = heapq.heappop(frontier)
            if current == goal:
                return self._reconstruct_path(came_from, goal)

            for next_node, edge_weight in self.graph[current]:
                new_cost = cost_so_far[current] + edge_weight
                if next_node not in cost_so_far or new_cost < cost_so_far[next_node]:
                    cost_so_far[next_node] = new_cost
                    priority = new_cost + euclidean(next_node, goal)
                    counter += 1
                    heapq.heappush(frontier, (priority, counter, next_node))
                    came_from[next_node] = current

        raise ValueError("No path found")

    @staticmethod
    def _reconstruct_path(
        came_from: dict[Node3D, Node3D | None],
        goal: Node3D,
    ) -> list[Node3D]:
        path = [goal]
        current = goal
        while came_from[current] is not None:
            current = came_from[current]
            path.append(current)
        path.reverse()
        return path
