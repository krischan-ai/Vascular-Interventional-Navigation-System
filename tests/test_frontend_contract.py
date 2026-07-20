from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]


def test_hud_uses_backend_dashboard_fields():
    hud = (ROOT / "godot_client/scripts/hud_controller.gd").read_text(encoding="utf-8")
    main = (ROOT / "godot_client/scripts/main_controller.gd").read_text(encoding="utf-8")
    path = (ROOT / "godot_client/scripts/path_renderer.gd").read_text(encoding="utf-8")
    guidewire = (ROOT / "godot_client/scripts/guidewire_renderer.gd").read_text(encoding="utf-8")
    entry = (ROOT / "godot_client/scripts/entry_marker.gd").read_text(encoding="utf-8")
    rig = (ROOT / "godot_client/scripts/camera_rig.gd").read_text(encoding="utf-8")
    navcam = (ROOT / "godot_client/scripts/navigation_camera_controller.gd").read_text(encoding="utf-8")
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
    assert "func set_device_state" in hud
    assert '_conn["guidewire"]' in hud
    assert '_conn["support"]' in hud
    assert '_conn["buckling"]' in hud
    assert '_conn["procedure"]' not in hud
    assert '_conn["access"]' not in hud
    assert '_conn["wall_slide"]' in hud
    assert 'connp.add_kv("\u5bfc\u4e1d"' in hud
    assert 'connp.add_kv("\u652f\u6491"' in hud
    assert 'connp.add_kv("\u5c48\u66f2"' in hud
    assert 'connp.add_kv("\u8d34\u58c1"' in hud
    assert 'func _cn_tip_shape' in hud
    assert 'func _cn_wall_slide' in hud
    assert '_tool("调试", func(): _show_device_debug())' in hud
    assert 'func _show_device_debug' in hud
    assert 'AcceptDialog.new()' in hud
    assert '器械与术式调试数据' in hud
    assert '_debug_line(lines, "guidewire_summary"' in hud
    assert '_debug_line(lines, "needle_entry_label"' in hud
    assert '_debug_line(lines, "intravascular_length_mm"' in hud
    assert '_debug_line(lines, "external_tail_length_mm"' in hud
    assert '_debug_line(lines, "render_scope"' in hud
    assert 'func _debug_line(lines: Array[String], key: String, value: Variant, label: String = "")' in hud
    assert '"%s (%s)" % [label, key]' in hud
    assert '"临床总长"' in hud
    assert '"血管内渲染长度"' in hud
    assert '"体外剩余长度"' in hud
    assert '"3D渲染范围"' in hud
    assert '"进针位置"' in hud
    assert '"当前支撑器械"' in hud
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
    assert "navigation_camera_controller.gd" in rig
    assert "func set_navigation_route" in rig
    assert "func clear_navigation_route" in rig
    assert "var pose := _nav_camera.navigation_pose" in rig
    assert "_rig.set_navigation_route(path_wps)" in main
    assert "_rig.clear_navigation_route()" in main
    assert "_set_camera_mode(CamMode.FOLLOW)" in main
    assert "func navigation_pose" in navcam
    assert "func _closest_arclength" in navcam
    assert "func _transport_up" in navcam
    assert "back_wall_color" in main
    assert "focus_alpha_far" in main
    assert "focus_emission_far" in main
    assert "func _update_vessel_focus_tip" in main
    assert "_update_vessel_focus_tip(tip_world, true)" in main
    assert "func _update_vessel_route_visibility" in main
    assert "route_points[96]" in main
    assert "route_visibility(world_pos)" in main
    assert "path_info.get(\"vessel_radius\", null)" in main
    assert "_hud.set_device_state" in main
    assert 'batch.get("guidewire", {})' in main
    assert 'batch.get("support", {})' in main
    assert 'batch.get("risk", {})' in main
    assert 'batch.get("procedure", {})' in main
    assert "_update_vessel_route_visibility(path_wps, true, route_radius)" in main
    assert "_update_vessel_route_visibility([], false)" in main
    assert "max(tip_prox * (1.0 - route_focus_enabled), route_prox * route_focus_enabled)" in main
    assert "corridor_radius = maxf(route_vessel_radius, radius_m * 1.35)" in main
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
    assert "proximal_tail_spacing" not in guidewire
    assert "func _extend_points_to_clinical_length" not in guidewire
    assert "clinical_total_length_mm" not in guidewire
    assert "func _trim_points_to_clinical_length" not in guidewire
    assert "vertex_color_use_as_albedo" in guidewire
    assert "SHADING_MODE_UNSHADED" in guidewire
    assert "func _body_color" in guidewire
    assert "func _segment_color" in guidewire
    assert 'body.get("material_segment"' in guidewire
    assert 'body.get("support_state"' in guidewire
    assert '"inside_support_tube"' in guidewire
    assert '.lerp(Color(0.05, 0.95, 1.0), 0.38)' in guidewire
    assert "func set_landmarks_visible" in entry
    assert "_landmarks_visible: bool = false" in entry
