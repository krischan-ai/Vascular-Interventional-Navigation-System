from __future__ import annotations

import heapq
import math
import time
import uuid
from dataclasses import dataclass
from pathlib import Path
from typing import Sequence

from services.graph_loader import Graph, GraphLoader, Node3D


@dataclass(frozen=True)
class PathResult:
    path_id: str
    waypoints: list[Node3D]
    length_mm: float
    node_count: int
    compute_time_ms: float

    def as_dict(self) -> dict:
        return {
            "path_id": self.path_id,
            "waypoints": [list(point) for point in self.waypoints],
            "length_mm": self.length_mm,
            "node_count": self.node_count,
            "compute_time_ms": self.compute_time_ms,
        }


def euclidean(a: Sequence[float], b: Sequence[float]) -> float:
    return math.sqrt(
        (float(a[0]) - float(b[0])) ** 2
        + (float(a[1]) - float(b[1])) ** 2
        + (float(a[2]) - float(b[2])) ** 2
    )


class PathPlanner:
    def __init__(self, graph_path: str | Path | None = None) -> None:
        self.loader = GraphLoader(graph_path) if graph_path is not None else GraphLoader()

    @property
    def graph(self) -> Graph:
        return self.loader.graph

    @property
    def nodes(self) -> list[Node3D]:
        return self.loader.nodes

    def load(self, graph_path: str | Path) -> None:
        self.loader.load(graph_path)

    def find_nearest_node(self, position: Sequence[float]) -> Node3D:
        if not self.nodes:
            raise ValueError("PathPlanner graph is not loaded")
        return min(self.nodes, key=lambda node: euclidean(node, position))

    def plan(
        self,
        start: Sequence[float],
        end: Sequence[float],
        algorithm: str = "astar",
    ) -> PathResult:
        if algorithm != "astar":
            raise ValueError(f"Unsupported planning algorithm: {algorithm}")
        if not self.graph:
            raise ValueError("PathPlanner graph is not loaded")

        start_time = time.perf_counter()
        start_node = self.find_nearest_node(start)
        end_node = self.find_nearest_node(end)
        waypoints = self._astar(start_node, end_node)
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
        )

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
