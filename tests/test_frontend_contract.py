from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]


def test_hud_uses_backend_dashboard_fields():
    hud = (ROOT / "godot_client/scripts/hud_controller.gd").read_text(encoding="utf-8")
    main = (ROOT / "godot_client/scripts/main_controller.gd").read_text(encoding="utf-8")
    path = (ROOT / "godot_client/scripts/path_renderer.gd").read_text(encoding="utf-8")
    ws = (ROOT / "godot_client/scripts/websocket_client.gd").read_text(encoding="utf-8")

    for field in ("remaining_distance", "vessel_radius", "eta_seconds", "latency_ms"):
        assert field in hud
        assert field in main

    assert "last_latency_ms" in ws
    assert "物理仿真 PHYSICS · 手动 MANUAL" not in main
    assert '"手动"' in main
    assert '"自动"' in main
    assert "_remaining_from_waypoints" in main
    assert "_camera_user_controlled" in main
    assert "func _guidewire_front_pose_from_batch" in main
    assert "bodies[bodies.size() - 1]" in main
    assert "func _set_orbit_focus" in main
    assert "func _setup_risk_volume" not in main
    assert "func _make_risk_volume_material" not in main
    assert "func _update_risk_volume" not in main
    assert "_risk_volume" not in main
    assert "nogo_lo" not in main
    assert "MOCK" not in main
    assert "no_go_zones" not in main
    assert "_focus_entry" not in main
    assert "_orbit_pivot = _entry_world" not in main
    assert "var b0: Dictionary = bodies[0]" not in main
    assert "func _free_look" not in main
    assert "_smooth_wall_mm" in hud
    assert "TODO backend" not in hud
    assert '"23.6"' not in hud
    assert '"02:35"' not in hud
    assert '"18 ms"' not in hud
    assert "blood_vessels_visual_high.glb" in main
    assert "func clear_forced_ranges" in path
    assert "var _risk_ranges" not in path
    assert "_curvature_hint" in path
