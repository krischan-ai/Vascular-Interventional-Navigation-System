import numpy as np
import trimesh

from tools import export_godot_assets


def _triangle_mesh() -> trimesh.Trimesh:
    return trimesh.Trimesh(
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


def test_quality_defaults_and_output_names():
    parser = export_godot_assets.build_arg_parser()

    visual = parser.parse_args([])
    preview = parser.parse_args(["--quality", "preview"])
    native = parser.parse_args(["--quality", "visual_native"])

    assert visual.quality == "visual_high"
    assert native.quality == "visual_native"
    assert export_godot_assets.default_max_faces(native.quality) == 1_000_000_000
    assert export_godot_assets.default_max_faces(visual.quality) == 300000
    assert export_godot_assets.default_max_faces(preview.quality) == 120000
    assert export_godot_assets.output_stem("blood_vessels", "visual_native") == "blood_vessels_visual_native"
    assert export_godot_assets.output_stem("blood_vessels", "visual_high") == "blood_vessels_visual_high"
    assert export_godot_assets.output_stem("segment_part", "preview") == "segment_part_preview"


def test_no_decimate_skips_decimation(monkeypatch):
    calls = []

    def fake_decimate(mesh, max_faces):
        calls.append((mesh, max_faces))
        return mesh

    monkeypatch.setattr(export_godot_assets, "decimate", fake_decimate)

    mesh = export_godot_assets.prepare_visual_mesh(
        _triangle_mesh(),
        max_faces=1,
        no_decimate=True,
    )

    assert len(mesh.faces) == 1
    assert calls == []


def test_prepare_visual_mesh_decimates_when_enabled(monkeypatch):
    calls = []

    def fake_decimate(mesh, max_faces):
        calls.append(max_faces)
        return mesh

    monkeypatch.setattr(export_godot_assets, "decimate", fake_decimate)

    export_godot_assets.prepare_visual_mesh(
        _triangle_mesh(),
        max_faces=42,
        no_decimate=False,
    )

    assert calls == [42]
