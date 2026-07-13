import json

import pytest

from tools.build_route_quality_report import build_route_quality_report


def _write_case(case_dir, graph, radii, targets):
    graph_dir = case_dir / "graph"
    derived_dir = case_dir / "derived"
    graph_dir.mkdir(parents=True)
    derived_dir.mkdir(parents=True)
    (graph_dir / "graph.json").write_text(json.dumps(graph), encoding="utf-8")
    if radii is not None:
        (graph_dir / "node_radii.json").write_text(
            json.dumps({"radii": radii}),
            encoding="utf-8",
        )
    (derived_dir / "targets.json").write_text(json.dumps(targets), encoding="utf-8")


def _line_graph(count=6):
    graph = {}
    radii = {}
    for index in range(count):
        key = f"{float(index):.6f},0.000000,0.000000"
        edges = []
        if index > 0:
            edges.append([f"{float(index - 1):.6f},0.000000,0.000000", 1.0])
        if index < count - 1:
            edges.append([f"{float(index + 1):.6f},0.000000,0.000000", 1.0])
        graph[key] = edges
        radii[key] = 1.0 + index * 0.1
    return graph, radii


def _targets(end_x=5.0):
    return {
        "endpoints": {
            "Endpoints-24": {"position_lps": [0.0, 0.0, 0.0]},
            "Endpoints-1": {"position_lps": [end_x, 0.0, 0.0]},
        },
    }


def test_build_route_quality_report_for_reachable_case(tmp_path):
    case_dir = tmp_path / "case_x"
    graph, radii = _line_graph()
    _write_case(case_dir, graph, radii, _targets())

    out = build_route_quality_report(case_dir, smooth=False)
    data = json.loads(out.read_text(encoding="utf-8"))
    route = data["routes"][1]

    assert data["schema_version"] == 1
    assert data["case_id"] == "case_x"
    assert data["graph"]["component_count"] == 1
    assert data["radii"]["missing_rate"] == pytest.approx(0.0)
    assert data["targets"]["reachable_count"] == 2
    assert route["route_id"] == "endpoints_1"
    assert route["reachable"] is True
    assert route["length_mm"] == pytest.approx(5.0)
    assert route["min_radius_mm"] == pytest.approx(1.0)
    assert route["max_radius_mm"] == pytest.approx(1.5)
    assert data["warnings"] == []


def test_build_route_quality_report_flags_unreachable_target(tmp_path):
    case_dir = tmp_path / "case_split"
    graph = {
        "0.000000,0.000000,0.000000": [["1.000000,0.000000,0.000000", 1.0]],
        "1.000000,0.000000,0.000000": [["0.000000,0.000000,0.000000", 1.0]],
        "9.000000,0.000000,0.000000": [],
    }
    _write_case(case_dir, graph, None, _targets(end_x=9.0))

    out = build_route_quality_report(case_dir, smooth=False)
    data = json.loads(out.read_text(encoding="utf-8"))

    assert data["graph"]["component_count"] == 2
    assert data["radii"]["exists"] is False
    assert data["targets"]["unreachable_targets"] == ["endpoints_1"]
    assert data["routes"][1]["reachable"] is False
    assert "No path found" in data["routes"][1]["error"]
    assert any("connected components" in item for item in data["warnings"])
    assert any("unreachable targets" in item for item in data["warnings"])
