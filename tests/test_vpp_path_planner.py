import json

import pytest

from services.graph_loader import GraphLoader
from services.path_planner import PathPlanner
from tools.build_vpp_routes import build_routes


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


def write_node_radii(path, count):
    radii = {}
    for i in range(count):
        key = f"{float(i):.6f},0.000000,{float(i * 0.5):.6f}"
        radii[key] = 1.0 + i * 0.1
    path.write_text(
        json.dumps({"n_nodes": count, "method": "test", "radii": radii}),
        encoding="utf-8",
    )


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


def test_graph_connectivity_repair_links_components(tmp_path):
    from services.graph_connectivity import connect_components, connected_components

    raw = {
        "0.000000,0.000000,0.000000": [["1.000000,0.000000,0.000000", 1.0]],
        "1.000000,0.000000,0.000000": [["0.000000,0.000000,0.000000", 1.0]],
        "5.000000,0.000000,0.000000": [["6.000000,0.000000,0.000000", 1.0]],
        "6.000000,0.000000,0.000000": [["5.000000,0.000000,0.000000", 1.0]],
    }

    assert len(connected_components(raw)) == 2
    repaired, virtual_edges = connect_components(raw)

    assert len(virtual_edges) == 1
    assert len(connected_components(repaired)) == 1

    graph_path = tmp_path / "graph.json"
    graph_path.write_text(json.dumps(repaired), encoding="utf-8")
    result = PathPlanner(graph_path).plan(
        (0.0, 0.0, 0.0),
        (6.0, 0.0, 0.0),
    )
    assert result.waypoints[0] == (0.0, 0.0, 0.0)
    assert result.waypoints[-1] == (6.0, 0.0, 0.0)


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

    def test_plan_carries_node_radii(self, tmp_path):
        graph_path = tmp_path / "graph.json"
        write_longer_graph(graph_path)
        write_node_radii(tmp_path / "node_radii.json", 10)

        result = PathPlanner(graph_path).plan(
            (0.0, 0.0, 0.0),
            (9.0, 0.0, 4.5),
            smooth=False,
        )

        assert result.radii is not None
        assert len(result.radii) == len(result.waypoints)
        assert result.radii[0] == pytest.approx(1.0)
        assert result.radii[-1] == pytest.approx(1.9)
        assert result.as_dict()["radii"] == pytest.approx(result.radii)

    def test_smooth_plan_interpolates_node_radii(self, tmp_path):
        graph_path = tmp_path / "graph.json"
        write_longer_graph(graph_path)
        write_node_radii(tmp_path / "node_radii.json", 10)

        result = PathPlanner(graph_path).plan(
            (0.0, 0.0, 0.0),
            (9.0, 0.0, 4.5),
            smooth=True,
            num_points=25,
        )

        assert result.smooth_waypoints is not None
        assert result.smooth_radii is not None
        assert len(result.smooth_radii) == len(result.smooth_waypoints)
        assert result.smooth_radii[0] == pytest.approx(result.radii[0])
        assert result.smooth_radii[-1] == pytest.approx(result.radii[-1])
        assert len(result.as_dict()["smooth_radii"]) == len(result.smooth_waypoints)


def test_build_vpp_routes_cache(tmp_path):
    case_dir = tmp_path / "case_x"
    graph_dir = case_dir / "graph"
    derived_dir = case_dir / "derived"
    graph_dir.mkdir(parents=True)
    derived_dir.mkdir(parents=True)
    write_longer_graph(graph_dir / "graph.json")
    write_node_radii(graph_dir / "node_radii.json", 10)
    targets = {
        "case_id": "case_x",
        "coordinate_system": "LPS",
        "unit": "mm",
        "endpoints": {
            "Endpoints-24": {"id": "24", "position_lps": [0.0, 0.0, 0.0]},
            "Endpoints-1": {"id": "1", "position_lps": [9.0, 0.0, 4.5]},
        },
    }
    (derived_dir / "targets.json").write_text(json.dumps(targets), encoding="utf-8")

    out = build_routes(case_dir, smooth=True, smooth_factor=0.5)
    data = json.loads(out.read_text(encoding="utf-8"))
    route = data["routes"]["endpoints_1"]

    assert data["phantom"] == "case_x_vpp"
    assert data["unit"] == "m"
    assert len(data["routes"]) == 2
    assert len(route["waypoints"]) == len(route["radius_m"])
    assert route["length_m"] > 0.0
    assert route["target"] == pytest.approx([0.009, 0.0, 0.0045], abs=0.001)
