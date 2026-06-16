import json

import pytest

from services.graph_loader import GraphLoader
from services.path_planner import PathPlanner


def write_graph(path):
    graph = {
        "0.000000,0.000000,0.000000": [["1.000000,0.000000,0.000000", 1.0]],
        "1.000000,0.000000,0.000000": [
            ["0.000000,0.000000,0.000000", 1.0],
            ["2.000000,0.000000,0.000000", 1.0],
        ],
        "2.000000,0.000000,0.000000": [["1.000000,0.000000,0.000000", 1.0]],
    }
    path.write_text(json.dumps(graph), encoding="utf-8")


def write_longer_graph(path):
    """Create a graph with enough nodes for B-spline smoothing."""
    graph = {}
    for i in range(10):
        key = f"{float(i):.6f},0.000000,{float(i * 0.5):.6f}"
        neighbors = []
        if i > 0:
            prev_key = f"{float(i - 1):.6f},0.000000,{float((i - 1) * 0.5):.6f}"
            neighbors.append([prev_key, 1.118])
        if i < 9:
            next_key = f"{float(i + 1):.6f},0.000000,{float((i + 1) * 0.5):.6f}"
            neighbors.append([next_key, 1.118])
        graph[key] = neighbors
    path.write_text(json.dumps(graph), encoding="utf-8")


def test_graph_loader_parses_vpp_adjacency(tmp_path):
    graph_path = tmp_path / "graph.json"
    write_graph(graph_path)

    loader = GraphLoader(graph_path)

    assert loader.stats.node_count == 3
    assert loader.stats.edge_count == 4
    assert loader.node_for_key("1.000000,0.000000,0.000000") == (1.0, 0.0, 0.0)


def test_path_planner_finds_astar_path(tmp_path):
    graph_path = tmp_path / "graph.json"
    write_graph(graph_path)

    planner = PathPlanner(graph_path)
    result = planner.plan((0.1, 0.0, 0.0), (1.9, 0.0, 0.0))

    assert result.node_count == 3
    assert result.length_mm == 2.0
    assert result.waypoints[0] == (0.0, 0.0, 0.0)
    assert result.waypoints[-1] == (2.0, 0.0, 0.0)


class TestBSplineSmoothing:
    """Tests for B-spline path smoothing."""

    def test_smooth_path_basic(self, tmp_path):
        """Test basic B-spline smoothing."""
        graph_path = tmp_path / "graph.json"
        write_longer_graph(graph_path)

        planner = PathPlanner(graph_path)
        result = planner.plan(
            (0.0, 0.0, 0.0),
            (9.0, 0.0, 4.5),
            smooth=True,
        )

        assert result.smooth_waypoints is not None
        assert len(result.smooth_waypoints) > len(result.waypoints)
        assert result.smooth_length_mm is not None
        assert result.max_curvature is not None
        assert result.max_curvature >= 0

    def test_smooth_path_preserves_endpoints(self, tmp_path):
        """Test that smoothing preserves start and end points approximately."""
        graph_path = tmp_path / "graph.json"
        write_longer_graph(graph_path)

        planner = PathPlanner(graph_path)
        result = planner.plan(
            (0.0, 0.0, 0.0),
            (9.0, 0.0, 4.5),
            smooth=True,
        )

        first_smooth = result.smooth_waypoints[0]
        last_smooth = result.smooth_waypoints[-1]
        first_orig = result.waypoints[0]
        last_orig = result.waypoints[-1]

        assert abs(first_smooth[0] - first_orig[0]) < 0.1
        assert abs(first_smooth[1] - first_orig[1]) < 0.1
        assert abs(first_smooth[2] - first_orig[2]) < 0.1

        assert abs(last_smooth[0] - last_orig[0]) < 0.1
        assert abs(last_smooth[1] - last_orig[1]) < 0.1
        assert abs(last_smooth[2] - last_orig[2]) < 0.1

    def test_smooth_path_without_flag(self, tmp_path):
        """Test that smooth=False returns no smooth data."""
        graph_path = tmp_path / "graph.json"
        write_longer_graph(graph_path)

        planner = PathPlanner(graph_path)
        result = planner.plan(
            (0.0, 0.0, 0.0),
            (9.0, 0.0, 4.5),
            smooth=False,
        )

        assert result.smooth_waypoints is None
        assert result.smooth_length_mm is None
        assert result.max_curvature is None

    def test_smooth_path_direct_method(self, tmp_path):
        """Test smooth_path method directly."""
        graph_path = tmp_path / "graph.json"
        write_longer_graph(graph_path)

        planner = PathPlanner(graph_path)

        waypoints = [
            (0.0, 0.0, 0.0),
            (1.0, 0.5, 0.5),
            (2.0, 1.0, 1.0),
            (3.0, 0.5, 1.5),
            (4.0, 0.0, 2.0),
        ]

        result = planner.smooth_path(waypoints, num_points=20)

        assert len(result["waypoints"]) == 20
        assert result["length_mm"] > 0
        assert result["max_curvature"] >= 0

    def test_smooth_path_too_few_points(self, tmp_path):
        """Test that smoothing fails with too few waypoints."""
        graph_path = tmp_path / "graph.json"
        write_longer_graph(graph_path)

        planner = PathPlanner(graph_path)

        waypoints = [(0.0, 0.0, 0.0), (1.0, 0.0, 0.0), (2.0, 0.0, 0.0)]

        with pytest.raises(ValueError, match="at least 4 waypoints"):
            planner.smooth_path(waypoints)

    def test_path_result_as_dict_with_smooth(self, tmp_path):
        """Test PathResult.as_dict includes smooth data when present."""
        graph_path = tmp_path / "graph.json"
        write_longer_graph(graph_path)

        planner = PathPlanner(graph_path)
        result = planner.plan(
            (0.0, 0.0, 0.0),
            (9.0, 0.0, 4.5),
            smooth=True,
        )

        result_dict = result.as_dict()

        assert "smooth_waypoints" in result_dict
        assert "smooth_length_mm" in result_dict
        assert "max_curvature" in result_dict
        assert len(result_dict["smooth_waypoints"]) > 0
