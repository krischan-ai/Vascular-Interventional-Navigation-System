from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]


def test_hud_uses_backend_dashboard_fields():
    hud = (ROOT / "godot_client/scripts/hud_controller.gd").read_text(encoding="utf-8")
    main = (ROOT / "godot_client/scripts/main_controller.gd").read_text(encoding="utf-8")
    path = (ROOT / "godot_client/scripts/path_renderer.gd").read_text(encoding="utf-8")
    guidewire = (ROOT / "godot_client/scripts/guidewire_renderer.gd").read_text(encoding="utf-8")
    entry = (ROOT / "godot_client/scripts/entry_marker.gd").read_text(encoding="utf-8")
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
    assert "blood_vessels_visual_native.glb" in main
    assert "%s_visual_native.glb" in main
    assert '@export var phantom: String = "case_001_vpp"' in main
    assert '@export var target: String = "endpoints_1"' in main
    assert "func _is_debug_low_poly_phantom" in main
    assert "debug low-poly phantom" in main
    assert "enum OrbitPreset { CLINICAL, TREE }" in main
    assert "Clinical Orbit" in main
    assert "Tree Overview" in main
    assert "func _apply_clinical_orbit" in main
    assert "_set_orbit_preset(OrbitPreset.CLINICAL, true)" in main
    assert "_set_orbit_preset(OrbitPreset.TREE, true)" in main
    assert "back_wall_color" in main
    assert "focus_alpha_far" in main
    assert "focus_emission_far" in main
    assert "func _update_vessel_focus_tip" in main
    assert "_update_vessel_focus_tip(tip_world, true)" in main
    assert "rim_alpha = 0.46" in main
    assert "core_alpha = 0.014" in main
    assert "relief_light_dir" in main
    assert "relief_strength" in main
    assert "relief_shadow" in main
    assert "vessel loaded from %s" in main
    assert "func clear_forced_ranges" in path
    assert "var _risk_ranges" not in path
    assert "_curvature_hint" in path
    assert "path_radius_main: float = 0.00055" in path
    assert "mix(0.72, 0.10, behind)" in path
    assert "func set_visual_state" in path
    assert "guidewire_mechanics" in path
    assert "set_visual_state(\"stale\", 0.0, true)" in path
    assert "safety_color_stale" in path
    assert "tip_radius_overview: float = 0.0011" in guidewire
    assert "func set_landmarks_visible" in entry
    assert "_landmarks_visible: bool = false" in entry
