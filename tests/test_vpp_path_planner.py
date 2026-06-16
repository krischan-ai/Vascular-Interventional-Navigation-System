import json

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
