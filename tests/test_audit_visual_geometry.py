import numpy as np
import trimesh

from tools.audit_visual_geometry import audit_mesh


def test_audit_mesh_reports_boundary_edges_for_open_triangle():
    mesh = trimesh.Trimesh(
        vertices=np.array(
            [
                [0.0, 0.0, 0.0],
                [1.0, 0.0, 0.0],
                [0.0, 1.0, 0.0],
            ]
        ),
        faces=np.array([[0, 1, 2]]),
        process=False,
    )

    report = audit_mesh(mesh)

    assert report["vertices"] == 3
    assert report["faces"] == 1
    assert report["watertight"] is False
    assert report["boundary_edges"] == 3
    assert report["nonmanifold_edges"] == 0


def test_audit_mesh_reports_closed_smooth_primitive():
    mesh = trimesh.creation.icosphere(subdivisions=1, radius=1.0)

    report = audit_mesh(mesh)

    assert report["watertight"] is True
    assert report["boundary_edges"] == 0
    assert report["nonmanifold_edges"] == 0
    assert report["adjacent_edge_count"] > 0
    assert 0.0 <= report["sharp_edge_ratio"] <= 1.0
