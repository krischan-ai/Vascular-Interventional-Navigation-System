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
    assert "var metrics := batch.duplicate(true)" in main

    assert "last_latency_ms" in ws
    assert "@export var n_substeps: int = 8" in ws
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
    assert '@export var physics_engine: String = "newton_demo"' in main
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


def test_frontend_emergency_control_and_reconnect_contract():
    ws = (ROOT / "godot_client/scripts/websocket_client.gd").read_text(encoding="utf-8")
    main = (ROOT / "godot_client/scripts/main_controller.gd").read_text(encoding="utf-8")
    hud = (ROOT / "godot_client/scripts/hud_controller.gd").read_text(encoding="utf-8")
    inputs = (ROOT / "godot_client/scripts/input_handler.gd").read_text(encoding="utf-8")
    dial = (ROOT / "godot_client/scripts/ui/intent_dial.gd").read_text(encoding="utf-8")

    # The frontend requests a backend latch; it never substitutes a one-shot zero
    # control frame or displays success before the acknowledgement.
    assert 'func send_emergency_stop(reason: String = "operator") -> bool' in ws
    assert 'func send_resume() -> bool' in ws
    assert '"emergency_stop_confirmed"' in ws
    assert '"resume_confirmed"' in ws
    assert "func controls_blocked() -> bool" in ws
    emergency_handler = main.split("func _on_emergency_stop() -> void:", 1)[1].split(
        "\nfunc ", 1
    )[0]
    assert "send_control(0.0, 0.0)" not in emergency_handler
    assert "func confirm_emergency_stop" in hud
    assert "急停已由后端锁存" in hud

    # Keyboard, pointer navigation, automatic ticks and mouse buttons share the
    # same emergency/session gate.
    assert "var _controls_blocked: bool = true" in main
    assert "if _controls_blocked or not _autopilot_active" in main
    assert "if _controls_blocked or _path_waypoints.is_empty()" in main
    assert "func set_control_enabled(enabled: bool)" in inputs
    assert "if not _control_enabled" in inputs

    # A closed WebSocket gets a fresh peer, a recreated session, and old-session
    # frames are filtered by their server session id.
    assert "func _connect_socket(is_retry: bool)" in ws
    assert "_socket = WebSocketPeer.new()" in ws
    assert "func _message_matches_session" in ws
    assert '"reconnecting"' in ws
    assert '"session_failed"' in ws

    # Presets scale both button and keyboard commands; up/down directions differ.
    assert "push += push_scale" in inputs
    assert "push -= push_scale" in inputs
    assert "rotate += rotate_scale" in inputs
    assert "rotate -= rotate_scale" in inputs
    assert "_send_push.bind(1.0)" in hud
    assert "_send_push.bind(-1.0)" in hud
    assert "_send_rotate.bind(1.0)" in hud
    assert "_send_rotate.bind(-1.0)" in hud
    assert "func _set_speed_preset" in hud
    assert "func _apply_angle_step" in hud
    assert "func set_input(push: float, rotate: float)" in dial
    assert "control_profile_changed.connect(_input.set_control_profile)" in main

    # All four protection switches now round-trip through a backend config ack.
    assert "func send_control_config(config: Dictionary)" in ws
    assert "protection_changed" in hud
    assert "control_config_received" in ws
    for key in (
        "jtip_assist_enabled",
        "torque_limit_enabled",
        "withdrawal_protection_enabled",
        "auto_stop_push_enabled",
    ):
        assert key in hud


def test_endoscope_pane_uses_live_vessel_world_and_real_controls():
    main = (ROOT / "godot_client/scripts/main_controller.gd").read_text(encoding="utf-8")
    fallback = (ROOT / "godot_client/scripts/ui/endoscope_fallback.gd").read_text(
        encoding="utf-8"
    )
    overlay = (ROOT / "godot_client/scripts/ui/endoscope_overlay.gd").read_text(
        encoding="utf-8"
    )
    icons = (ROOT / "godot_client/scripts/ui/pane_tool_icon.gd").read_text(
        encoding="utf-8"
    )
    material_factory = (
        ROOT / "godot_client/scripts/rendering/endoscope_material_factory.gd"
    ).read_text(encoding="utf-8")
    lighting_rig = (
        ROOT / "godot_client/scripts/rendering/endoscope_lighting_rig.gd"
    ).read_text(encoding="utf-8")
    camera_filter = (
        ROOT / "godot_client/scripts/rendering/endoscope_camera_filter.gd"
    ).read_text(encoding="utf-8")
    wall_shader = (
        ROOT / "godot_client/scripts/rendering/endoscope_wall.gdshader"
    ).read_text(encoding="utf-8")

    # The scope owns an isolated 3D world and renders the real vessel copy from
    # the guidewire's live front pose instead of a decorative 2D tunnel.
    assert "_scope_viewport.own_world_3d = true" in main
    assert "_scope_viewport.add_child(_scope_vessel)" in main
    assert "_rig.endoscope_cam.global_transform" in main
    assert "interpolate_with" in main
    assert "func _scope_frame_ready() -> bool" in main
    assert "_connection_ready" in main
    assert "_scope_has_tip" in main
    assert '"● LIVE" if ready else "○ WAIT"' in main
    assert "00:12:36" not in main

    # Illumination and wall appearance are part of the render path, while the
    # bottom controls change real frontend state or produce an output file.
    assert "scope_render_profile" in main
    assert "scope_render_quality" in main
    assert "EndoscopeMaterialFactoryScript.create_material" in main
    assert "_scope_vessel_mat" in main
    assert "func _on_scope_record_toggled" in main
    assert "func _on_scope_brightness_changed" in main
    assert "func _capture_scope_frame" in main
    assert "--scope-validation-capture" in main
    assert "--scope-render-profile=" in main
    assert "--scope-render-quality=" in main
    assert "--scope-validation-output=" in main
    assert "--scope-validation-motion" in main
    assert "real motion controls complete" in main
    assert "func _save_scope_frame" in main
    assert "func _toggle_scope_fullscreen" in main

    assert "等待腔镜数据" in fallback
    assert "range(28)" not in fallback
    assert "draw_colored_polygon" not in fallback
    assert "class_name EndoscopeOverlay" in overlay
    assert "class_name EndoscopeMaterialFactory" in material_factory
    for feature in (
        "roughness_texture",
        "normal_texture",
        "uv1_world_triplanar",
        "generate_mipmaps",
    ):
        assert feature in material_factory
    assert "render_mode unshaded" in wall_shader
    assert "cull_front" in wall_shader
    assert "inward_world = -normalize(world_normal)" in wall_shader
    assert "sample_triplanar" in wall_shader
    assert "clearcoat_strength" in wall_shader
    assert "wet_highlight" in wall_shader
    assert "class_name EndoscopeLightingRig" in lighting_rig
    assert "ScopeKeyLight" in lighting_rig
    assert "ScopeFillLight" in lighting_rig
    assert "_smoothed_radius" in lighting_rig
    assert "_smoothed_radius * 14.0" in lighting_rig
    assert "_scope_radius_target" in main
    assert "_scope_environment.fog_density" in main
    assert "class_name EndoscopeCameraFilter" in camera_filter
    assert "_critical_damped_position" in camera_filter
    assert "_max_roll_degrees_per_second" in camera_filter
    assert "Basis.looking_at" in camera_filter
    assert "_scope_viewport.msaa_3d = Viewport.MSAA_2X" in main
    assert "_scope_viewport.msaa_3d = Viewport.MSAA_4X" in main
    assert "_scope_viewport.use_taa = false" in main
    for kind in ("refresh", "record", "brightness", "capture", "fullscreen"):
        assert f'"{kind}"' in icons
