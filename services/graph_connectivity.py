from __future__ import annotations

import json
import math
from collections import deque
from dataclasses import dataclass
from pathlib import Path
from typing import Any

from services.graph_loader import GraphLoader, Node3D


RawGraph = dict[str, list[list[Any]]]


@dataclass(frozen=True)
class VirtualEdge:
    source_key: str
    target_key: str
    weight: float


def connected_components(raw_graph: RawGraph) -> list[list[str]]:
    """Return undirected connected components of an adjacency-map graph."""
    neighbors: dict[str, set[str]] = {key: set() for key in raw_graph}
    for source, edges in raw_graph.items():
        for edge in edges:
            if not edge:
                continue
            target = str(edge[0])
            if target not in neighbors:
                continue
            neighbors[source].add(target)
            neighbors[target].add(source)

    seen: set[str] = set()
    components: list[list[str]] = []
    for key in raw_graph:
        if key in seen:
            continue
        queue = deque([key])
        seen.add(key)
        component: list[str] = []
        while queue:
            current = queue.popleft()
            component.append(current)
            for nxt in neighbors[current]:
                if nxt not in seen:
                    seen.add(nxt)
                    queue.append(nxt)
        components.append(component)

    components.sort(key=len, reverse=True)
    return components


def virtual_edges_to_connect(raw_graph: RawGraph) -> list[VirtualEdge]:
    """Build nearest-neighbor bidirectional virtual edges between components."""
    components = connected_components(raw_graph)
    if len(components) <= 1:
        return []

    coords = {key: GraphLoader.parse_node(key) for key in raw_graph}
    connected = set(components[0])
    remaining = [set(component) for component in components[1:]]
    virtual_edges: list[VirtualEdge] = []

    while remaining:
        best: tuple[float, str, str, int] | None = None
        for index, component in enumerate(remaining):
            for source in connected:
                source_pt = coords[source]
                for target in component:
                    distance = _distance(source_pt, coords[target])
                    if best is None or distance < best[0]:
                        best = (distance, source, target, index)

        if best is None:
            break

        distance, source, target, index = best
        virtual_edges.append(VirtualEdge(source, target, distance))
        connected.update(remaining.pop(index))

    return virtual_edges


def connect_components(raw_graph: RawGraph, precision: int = 6) -> tuple[RawGraph, list[VirtualEdge]]:
    """Return a graph with bidirectional virtual edges added between components."""
    repaired: RawGraph = {
        key: [list(edge) for edge in edges]
        for key, edges in raw_graph.items()
    }
    virtual_edges = virtual_edges_to_connect(repaired)
    for edge in virtual_edges:
        weight = round(edge.weight, precision)
        _add_edge_once(repaired, edge.source_key, edge.target_key, weight)
        _add_edge_once(repaired, edge.target_key, edge.source_key, weight)
    return repaired, virtual_edges


def repair_graph_file(path: str | Path) -> list[VirtualEdge]:
    """Connect all components in ``path`` in-place and return added virtual edges."""
    graph_path = Path(path)
    raw = json.loads(graph_path.read_text(encoding="utf-8"))
    repaired, virtual_edges = connect_components(raw)
    graph_path.write_text(
        json.dumps(repaired, ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
    )
    return virtual_edges


def _add_edge_once(raw_graph: RawGraph, source: str, target: str, weight: float) -> None:
    edges = raw_graph[source]
    for edge in edges:
        if edge and edge[0] == target:
            return
    edges.append([target, weight])


def _distance(a: Node3D, b: Node3D) -> float:
    return math.sqrt(
        (a[0] - b[0]) ** 2
        + (a[1] - b[1]) ** 2
        + (a[2] - b[2]) ** 2
    )
