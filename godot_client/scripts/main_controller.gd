extends Node3D
## Root controller for the CathSim VPP client.
##
## Builds the 3D scene in code (environment, light, camera, vessel mesh,
## guidewire renderer, HUD) and wires the WebSocket client and input handler
## together. Kept code-driven so the .tscn stays minimal and robust.

# Navigation configuration (single source of truth, pushed to the WebSocket
# client). For VPP navigation set phantom to "<case>_vpp" and provide start/end
# endpoint positions in LPS millimeters; the backend plans the route, spawns the
# guidewire at the vessel entry, and streams the path for visualization.
# For a built-in phantom that ships its own entry + centerline (e.g.
# segment_part), set phantom/target and leave start/end empty: the backend reads
# the entry landmark and centerline.json from the phantom assets.
@export var phantom: String = "case_001_vpp"
@export var target: String = "endpoints_1"
@export var case_id: String = "case_001"
## LPS millimeters; leave empty for non-VPP (low_tort / segment_part) sessions.
@export var start_position: Array = [0.173, -268.24, 291.25]
@export var end_position: Array = [-975.65, -217.22, 250.32]
@export var physics_engine: String = "newton_demo"

# Surgical-view wall fade (meters). In the follow ("手术视图") view the vessel wall
# is fully opaque within `surgical_fade_near` of the guidewire tip and fully
# transparent beyond `surgical_fade_far`, so only the local lumen segment shows
# instead of many overlapping translucent walls糊成一片 (doc/08 §9.3 P1 可视性修复).
# The overview stays the whole-tree translucent 演示视图.
@export var surgical_fade_near: float = 0.03
@export var surgical_fade_far: float = 0.11
# Minimum route corridor size. FOLLOW expands this with backend path.vessel_radius
# when available, so the planned vessel wall remains visible beyond the guidewire.
@export var route_vessel_radius: float = 0.024
@export var route_vessel_feather: float = 0.018

# Switchable phantom models, cycled at runtime with the M key. The exported
# phantom above selects which entry is active on launch (matched by phantom
# name); each entry is the single source of truth for that model's navigation
# config, so switching just reapplies one row and re-handshakes the session.
#   - 项目初始模型: the original CathSim low-tort aorta near the origin.
#   - 主动脉干: the aorta_trunk sealed-lumen phantom (ships its own entry +
#     centerline; the centerline is the VMTK route, centered in the lumen).
#   - 全身体膜: the segment_part cavity phantom (ships its own entry + centerline).
#   - 局部血管空腔: the VPP real-vessel case; backend plans the route from the
#     start/end endpoints (LPS millimeters) documented in godot_client/README.md.
const MODELS: Array = [
	{
		"name": "主动脉树 Aorta-Tree (Newton)",
		"phantom": "aorta_tree",
		"target": "root",
		"case_id": "case_001",
		"start": [],
		"end": [],
		"physics_engine": "newton_demo",
	},
	{
		"name": "项目初始模型 Low-Tort (MuJoCo)",
		"phantom": "low_tort",
		"target": "bca",
		"case_id": "case_001",
		"start": [],
		"end": [],
		"physics_engine": "mujoco",
	},
	{
		"name": "主动脉干 Aorta-Trunk",
		"phantom": "aorta_trunk",
		"target": "root",
		"case_id": "case_001",
		"start": [],
		"end": [],
		"physics_engine": "mujoco",
	},
	{
		"name": "主动脉树 Aorta-Tree (MuJoCo)",
		"phantom": "aorta_tree",
		"target": "root",
		"case_id": "case_001",
		"start": [],
		"end": [],
		"physics_engine": "mujoco",
	},
	{
		"name": "全身体膜 Segment-Part",
		"phantom": "segment_part",
		"target": "root",
		"case_id": "case_001",
		"start": [],
		"end": [],
		"physics_engine": "auto",
	},
	{
		"name": "局部血管空腔 VPP",
		"phantom": "case_001_vpp",
		"target": "endpoints_1",
		"case_id": "case_001",
		"start": [0.173, -268.24, 291.25],
		"end": [-975.65, -217.22, 250.32],
		"physics_engine": "newton_demo",
	},
	{
		"name": "VPP MuJoCo Compare",
		"phantom": "case_001_vpp",
		"target": "endpoints_1",
		"case_id": "case_001",
		"start": [0.173, -268.24, 291.25],
		"end": [-975.65, -217.22, 250.32],
		"physics_engine": "mujoco",
	},
	{
		"name": "VPP Guided Compare",
		"phantom": "case_001_vpp",
		"target": "endpoints_1",
		"case_id": "case_001",
		"start": [0.173, -268.24, 291.25],
		"end": [-975.65, -217.22, 250.32],
		"physics_engine": "guided",
	},
]

enum CamMode { OVERVIEW, FOLLOW, ENDOSCOPE }
enum OrbitPreset { CLINICAL, TREE }
enum ViewToolMode { SELECT, ORBIT, PAN }
const CAM_MODE_NAMES := {
	CamMode.OVERVIEW: "外部 Orbit",
	CamMode.FOLLOW: "跟随 Follow",
	CamMode.ENDOSCOPE: "内窥镜 Endoscope",
}
const _SCOPE_LAYER_MASK := 1 << 17
const EndoscopeFallbackScript := preload("res://scripts/ui/endoscope_fallback.gd")
const EndoscopeOverlayScript := preload("res://scripts/ui/endoscope_overlay.gd")
const EndoscopeMaterialFactoryScript := preload(
	"res://scripts/rendering/endoscope_material_factory.gd")
const EndoscopeLightingRigScript := preload(
	"res://scripts/rendering/endoscope_lighting_rig.gd")
const EndoscopeCameraFilterScript := preload(
	"res://scripts/rendering/endoscope_camera_filter.gd")
# UI contract labels: "手动", "自动".

const ORBIT_PRESET_NAMES := {
	OrbitPreset.CLINICAL: "Clinical Orbit",
	OrbitPreset.TREE: "Tree Overview",
}

var _ws  # WebSocketClient node
var _input  # InputHandler node
var _hud  # HUD CanvasLayer
var _deform  # DeformPanel CanvasLayer (live guidewire deformation sliders)
var _guidewire  # GuidewireRenderer node
var _path  # PathRenderer node
var _entry_marker  # EntryMarker node (vessel entry + target highlights)
var _rig  # CameraRig node (follow + endoscope cameras)
var _camera: Camera3D  # overview camera
# 3D pane: the whole 3D world renders into this SubViewport (own World3D) shown as a
# windowed pane, instead of filling the root window. The root window then only holds
# the dashboard UI + a dark backdrop, matching the设计图 multi-pane layout.
var _world: SubViewport
var _pane_3d_container: SubViewportContainer  # the 3D pane rect (for click coord translation)
# Pane swap (DSA 实时影像 ⇄ 3D 血管导航): when true the 3D pane takes the big left
# region and the DSA pane the small right-top one. Toggled by the 互换 buttons on
# both panes or the X key; overlays inside each pane are anchored so they adapt.
var _dsa_pane: PanelContainer
var _scope_pane: PanelContainer
var _scope_viewport: SubViewport
var _scope_camera: Camera3D
var _scope_fallback: Control
var _scope_view_container: SubViewportContainer
var _scope_overlay: Control
var _scope_live_label: Label
var _scope_time_label: Label
var _scope_rec_button: Button
var _scope_capture_button: Button
var _scope_brightness_slider: HSlider
var _scope_headlight: OmniLight3D
var _scope_lighting
var _scope_camera_filter
var _scope_environment: Environment
var _scope_radius_target: float = 0.003
var _scope_target_fov: float = 79.0
var _scope_has_tip: bool = false
var _scope_pose_initialized: bool = false
var _scope_recording: bool = false
var _scope_record_started_msec: int = 0
var _scope_record_elapsed_msec: int = 0
var _scope_fullscreen: bool = false
@export var scope_camera_smooth: float = 18.0
@export_enum("baseline", "enhanced") var scope_render_profile: String = "enhanced"
@export_enum("performance", "balanced", "high") var scope_render_quality: String = "balanced"
const _SCOPE_PERF_SAMPLE_COUNT := 300
var _scope_frame_times_ms: Array[float] = []
var _scope_perf_reported: bool = false
var _scope_validation_capture_pending: bool = false
var _scope_validation_capture_frames: int = 0
var _scope_validation_output: String = ""
var _scope_validation_motion: bool = false
var _scope_validation_motion_stage: int = 0
var _scope_validation_motion_accum: float = 0.0
var _scope_validation_motion_steps: int = 0
var _panes_swapped: bool = false
# Overview orbit camera (参考图视角): the external camera is a pivot-orbit rig —
# spherical (yaw/pitch/dist) around a pan-able pivot. The selected view tool makes
# left-drag orbit or pan; middle-drag always pans, the wheel or 放大/缩小 tools zoom,
# and 复位 reframes the vessel AABB. Only OVERVIEW is orbit-driven.
var _orbit_pivot := Vector3.ZERO
var _orbit_yaw: float = 0.0
var _orbit_pitch: float = 0.0
var _orbit_dist: float = 2.0
var _orbit_min: float = 0.05
var _orbit_max: float = 20.0
var _orbit_preset: int = OrbitPreset.CLINICAL
var _last_aabb := AABB()  # last framed vessel AABB, for the 复位 tool
const _ORBIT_SPEED := 0.008     # rad per drag px
const _PAN_SPEED := 0.0012      # pivot m per drag px per m of distance
const _WHEEL_STEP := 0.88       # dist multiplier per wheel notch
const _VESSEL_ROUTE_SHADER_SAMPLES := 96
# 3D-pane pointer gesture state: a left press inside the pane is a pending click;
# once cumulative travel exceeds the threshold it becomes an orbit drag instead of
# a click-to-navigate on release.
var _press_in_pane: bool = false
var _press_travel: float = 0.0
const _CLICK_TRAVEL_MAX := 6.0  # px
# Right tool strip mode: 选择 (drag inert), 旋转 (default) or 平移. All three retain
# the existing short-click path navigation; only a drag beyond the threshold uses
# the selected camera operation.
var _view_tool_mode: int = ViewToolMode.ORBIT
# Direction cube (§11): wireframe cube in its own SubViewport, counter-rotated
# against the active 3D camera every frame so it always shows world orientation.
var _cube_root: Node3D
# Route display: the route line itself carries a continuous curvature hint
# gradient (computed in path_renderer). Real spatial risk regions are not
# rendered here; they must arrive as source-backed backend risk_regions.
# Route entry marker: route start is recorded for markers/navigation, but free
# orbit stays framed on the vessel as a whole so dragging is not locked to the
# green entry sphere.
var _entry_world := Vector3.ZERO
var _entry_known: bool = false
var _camera_user_controlled: bool = false
# 跟随 (follow) toolbar toggle + the last known guidewire-front world position.
# Exiting follow switches to an orbit camera centered on the actual guidewire
# front, not the route target marker.
var _follow_btn: Button
var _tip_world_last := Vector3.ZERO
var _tip_world_known: bool = false
var _cam_mode: int = CamMode.OVERVIEW
var _vessel_meshes: Array = []  # MeshInstance3D nodes of the vessel
var _vessel_mat_overlay: ShaderMaterial  # cyan fresnel-glow whole-tree (overview/demo)
var _vessel_mat_interior: StandardMaterial3D  # opaque inner wall (endoscope)
var _scope_vessel_mat: Material  # profile-driven warm inner wall for private scope world
var _vessel_mat_surgical: ShaderMaterial  # cyan fresnel + tip-proximity fade (follow/surgical)
var _env: Environment  # world environment; endoscope toggles its depth fog
var _vessel: Node3D  # current vessel scene root (freed/rebuilt on model switch)
var _scope_vessel: Node3D  # same GLB on a private render layer for the endoscope pane
var _model_index: int = 0  # index into MODELS of the active phantom
# Multi-branch navigation: ordered target endpoint ids the backend offers for the
# active phantom (aorta_tree), cycled with the B key. Empty for single-route models.
var _branch_targets: Array = []
var _branch_index: int = 0
# Auto-switch to the close-up follow view once the first tip pose streams in, so
# the guidewire is visible immediately instead of a dot in the overview.
var _auto_followed: bool = false
# Click-to-navigate (doc/09 ShapeIntent). The planned route's waypoints in the
# backend meter frame, captured from state_batch. A left click picks the nearest
# route waypoint to the click ray and drives the tip there via the autopilot; ESC
# or any manual key hands control back. Tick the backend at ~20 Hz while engaged.
var _path_waypoints: Array = []
var _autopilot_active: bool = false
var _autopilot_accum: float = 0.0
const _AUTOPILOT_TICK: float = 0.05
# "waypoint k/n" label for the engaged click goal, shown in the HUD nav line
# alongside live progress so the (slow) autopilot reads as responsive.
var _autopilot_wp_text: String = ""
# Last clicked navigation goal (backend meter frame), so the 恢复导航 button can
# re-engage the autopilot toward it after a 人工接管 / 急停.
var _last_click_target: Array = []
var _connection_ready: bool = false
var _controls_blocked: bool = true

# On-screen diagnostics.
var _session_id: String = "none"
var _msg_count: int = 0
var _last_msg: String = "—"
var _logged_first_batch: bool = false


func _update_debug() -> void:
	_hud.set_debug("session: %s\nstate msgs: %d\nlast: %s" % [
		_session_id, _msg_count, _last_msg,
	])


func _ready() -> void:
	print("[Main] _ready: building scene")
	_apply_scope_launch_overrides()
	_scope_validation_capture_pending = (
		"--scope-validation-capture" in OS.get_cmdline_user_args())
	_scope_validation_motion = (
		"--scope-validation-motion" in OS.get_cmdline_user_args())
	_scope_validation_capture_pending = (
		_scope_validation_capture_pending or _scope_validation_motion)
	# Pick the launch model from the exported phantom name and pull its full
	# config from MODELS so the table stays the single source of truth.
	_model_index = _resolve_model_index(phantom)
	_apply_model_config(MODELS[_model_index])
	_setup_panes()
	_setup_environment()
	_setup_camera_and_light()
	_setup_hud()
	_load_model_scene()
	_setup_network_and_input()
	_hud.set_model(str(MODELS[_model_index].name))
	print("[Main] camera pos=%s current=%s" % [_camera.position, _camera.current])


func _apply_scope_launch_overrides() -> void:
	for argument in OS.get_cmdline_user_args():
		if argument.begins_with("--scope-render-profile="):
			var profile_value := argument.trim_prefix("--scope-render-profile=")
			if profile_value in ["baseline", "enhanced"]:
				scope_render_profile = profile_value
		elif argument.begins_with("--scope-render-quality="):
			var quality_value := argument.trim_prefix("--scope-render-quality=")
			if quality_value in ["performance", "balanced", "high"]:
				scope_render_quality = quality_value
		elif argument.begins_with("--scope-validation-output="):
			_scope_validation_output = argument.trim_prefix(
				"--scope-validation-output=")
	print("[Scope] render profile=%s quality=%s" % [
		scope_render_profile, scope_render_quality])


func _resolve_model_index(phantom_name: String) -> int:
	var first_match := -1
	for i in MODELS.size():
		if str(MODELS[i].phantom) == phantom_name:
			if first_match < 0:
				first_match = i
			if str(MODELS[i].get("physics_engine", "auto")) == physics_engine:
				return i
	return first_match if first_match >= 0 else 0


func _apply_model_config(cfg: Dictionary) -> void:
	phantom = str(cfg.phantom)
	target = str(cfg.target)
	case_id = str(cfg.case_id)
	start_position = (cfg.start as Array).duplicate()
	end_position = (cfg.end as Array).duplicate()
	physics_engine = str(cfg.get("physics_engine", "auto"))


# Build the vessel mesh plus the renderers that share its coordinate frame
# (guidewire / path / entry marker / camera rig). Reusable on model switch:
# tears down the previous model's scene first, then frames the camera. Network,
# HUD, environment and light persist across switches and are set up once.
func _load_model_scene() -> void:
	_teardown_model_scene()
	var vessel := _setup_vessel()
	_vessel = vessel
	# Parent the guidewire and planned-path renderers under the vessel scene root
	# so all three share the same coordinate space (the glTF/trimesh axis-
	# conversion transform applies equally to the mesh and the streamed
	# guidewire/path positions). Falls back to the 3D pane's SubViewport when the GLB
	# is missing, so the wire still renders inside the pane (not the root window).
	var frame: Node = vessel if vessel != null else _world
	_setup_guidewire(frame)
	_setup_path(frame)
	_setup_entry_marker(frame)
	_setup_rig(frame)
	# Reset view state so the new model starts in overview (translucent wall, no
	# fog) and auto-follows again once its first tip pose streams in.
	_auto_followed = false
	_cam_mode = CamMode.OVERVIEW
	_orbit_preset = OrbitPreset.CLINICAL
	_set_vessel_view_material(CamMode.OVERVIEW)
	if _env:
		_env.fog_enabled = false
	if vessel != null:
		var aabb := _scene_aabb(vessel)
		print("[Main] vessel AABB pos=%s size=%s" % [aabb.position, aabb.size])
		_frame_camera(aabb)
		_apply_clinical_orbit(true)
	else:
		# No vessel mesh: still place the camera somewhere sane so the HUD and
		# guidewire tip are visible (zero AABB -> orbit defaults around origin).
		_frame_camera(AABB())
	_camera.make_current()


func _teardown_model_scene() -> void:
	# Free the per-model renderers first (they may be children of the vessel),
	# then the vessel itself. Guarded so an initial build (nothing yet) is a no-op.
	for node in [_guidewire, _path, _entry_marker, _rig, _scope_vessel]:
		if node != null and is_instance_valid(node):
			node.queue_free()
	_guidewire = null
	_path = null
	_entry_marker = null
	_rig = null
	_scope_vessel = null
	_scope_has_tip = false
	_scope_pose_initialized = false
	_update_scope_state()
	if _vessel != null and is_instance_valid(_vessel):
		_vessel.queue_free()
	_vessel = null
	_vessel_meshes = []
	_entry_known = false  # the next route re-focuses the camera on its entry
	_camera_user_controlled = false
	_tip_world_known = false


# Build the windowed pane layout (设计图 局部窗口视图): a dark backdrop, a left DSA
# main-view placeholder, and a right-side 3D navigation pane backed by a SubViewport
# with its own World3D. The 3D scene (environment/light/cameras/vessel/renderers) is
# parented under `_world`, so it renders ONLY inside the pane, not the whole window.
# The panes sit in the central gap between the dashboard's top bar / left panel /
# right cards / bottom bar, so the HUD needs no rearrangement.
func _setup_panes() -> void:
	var layer := CanvasLayer.new()
	layer.layer = 0  # below the HUD (layer 1) so the dashboard chrome overlays the panes
	add_child(layer)

	var rootc := Control.new()
	rootc.set_anchors_and_offsets_preset(Control.PRESET_FULL_RECT)
	rootc.mouse_filter = Control.MOUSE_FILTER_IGNORE
	layer.add_child(rootc)

	# Dark medical backdrop (UiStyle.BG); the root window no longer renders any 3D.
	var bg := ColorRect.new()
	bg.set_anchors_and_offsets_preset(Control.PRESET_FULL_RECT)
	bg.color = UiStyle.BG
	bg.mouse_filter = Control.MOUSE_FILTER_IGNORE
	rootc.add_child(bg)

	_build_dsa_pane(rootc)
	_build_3d_pane(rootc)
	_build_scope_pane(rootc)


# DSA main view (doc/11 §5-§8, left 59%). Real X-ray fluoroscopy is a separate track,
# so the "image" is a grayscale noise placeholder; on top of it sit the medical
# overlays required by §6 (DSAOverlay), the info float, tool strip and legend box.
func _build_dsa_pane(rootc: Control) -> void:
	var dsa := PanelContainer.new()
	dsa.add_theme_stylebox_override("panel", UiStyle.panel_box(0.95, 8))
	UiStyle.place(dsa, UiStyle.dsa_rect())
	rootc.add_child(dsa)
	_dsa_pane = dsa

	# Grayscale "X-ray" placeholder texture (§5: 灰度、低饱和、医学透视感).
	var img := TextureRect.new()
	var noise_tex := NoiseTexture2D.new()
	var noise := FastNoiseLite.new()
	noise.frequency = 0.008
	noise.fractal_octaves = 4
	noise_tex.noise = noise
	noise_tex.width = 512
	noise_tex.height = 512
	img.texture = noise_tex
	img.expand_mode = TextureRect.EXPAND_IGNORE_SIZE
	img.stretch_mode = TextureRect.STRETCH_SCALE
	img.modulate = Color(0.42, 0.46, 0.5, 0.35)  # dim gray, low contrast
	img.set_anchors_and_offsets_preset(Control.PRESET_FULL_RECT)
	img.mouse_filter = Control.MOUSE_FILTER_IGNORE
	dsa.add_child(img)

	# Medical navigation overlay (§6): catheter / path / target / risk / locator.
	var overlay := DSAOverlay.new()
	overlay.set_anchors_and_offsets_preset(Control.PRESET_FULL_RECT)
	dsa.add_child(overlay)

	# UI overlays (mouse-ignoring).
	var ov := Control.new()
	ov.set_anchors_and_offsets_preset(Control.PRESET_FULL_RECT)
	ov.mouse_filter = Control.MOUSE_FILTER_IGNORE
	dsa.add_child(ov)

	_corner_label(ov, "①  DSA 实时影像", UiStyle.GREEN, Vector2(14, 10), 16)

	# 影像参数浮窗 (§5: 底 #101A26 75%, 圆角 6, 文字 #B8C7D6 14px).
	var info := PanelContainer.new()
	info.add_theme_stylebox_override("panel", UiStyle.card_box(0.75, 6))
	info.position = Vector2(14, 42)
	info.add_child(UiStyle.label("LAO: 12°\nCRA: 2°\nZoom: 100%", UiStyle.TEXT_MID, 14))
	ov.add_child(info)

	# 左侧悬浮工具栏 (§7: 48x48). Starts right below the info float so the 5-button
	# strip also fits the small right-top region when the panes are swapped.
	var tools := VBoxContainer.new()
	tools.add_theme_constant_override("separation", 8)
	tools.position = Vector2(14, 122)
	for t in ["视图", "增强", "标记", "测量", "截图"]:
		tools.add_child(_pane_tool(t, Vector2(48, 48)))
	ov.add_child(tools)


	# 图例盒 (§8: 面板底 + 边框, 右下角, 文字 #D8E6F3 14px + 彩色标记).
	var legend_panel := PanelContainer.new()
	legend_panel.add_theme_stylebox_override("panel", UiStyle.card_box(0.8, 8))
	legend_panel.anchor_left = 1.0
	legend_panel.anchor_top = 1.0
	legend_panel.anchor_right = 1.0
	legend_panel.anchor_bottom = 1.0
	legend_panel.offset_left = -220
	legend_panel.offset_top = -242
	legend_panel.offset_right = -14
	legend_panel.offset_bottom = -88
	legend_panel.grow_horizontal = Control.GROW_DIRECTION_BEGIN
	legend_panel.grow_vertical = Control.GROW_DIRECTION_BEGIN
	var legend := VBoxContainer.new()
	legend.add_theme_constant_override("separation", 6)
	for item in [["当前导管位置", UiStyle.GREEN], ["规划路径", UiStyle.YELLOW],
			["目标点", UiStyle.RED], ["高风险区域", UiStyle.RED], ["中风险区域", UiStyle.YELLOW]]:
		var lr := HBoxContainer.new()
		lr.add_theme_constant_override("separation", 8)
		var mark := ColorRect.new()
		mark.custom_minimum_size = Vector2(14, 4)
		mark.color = item[1]
		var mc := CenterContainer.new()
		mc.add_child(mark)
		lr.add_child(mc)
		lr.add_child(UiStyle.label(item[0], UiStyle.TEXT, 14))
		legend.add_child(lr)
	legend_panel.add_child(legend)
	ov.add_child(legend_panel)
	var strip := PanelContainer.new()
	strip.add_theme_stylebox_override("panel", UiStyle.card_box(0.82, 8))
	strip.anchor_left = 0.0
	strip.anchor_top = 1.0
	strip.anchor_right = 1.0
	strip.anchor_bottom = 1.0
	strip.offset_left = 10
	strip.offset_top = -82
	strip.offset_right = -10
	strip.offset_bottom = -10
	strip.grow_vertical = Control.GROW_DIRECTION_BEGIN
	var strip_row := HBoxContainer.new()
	strip_row.add_theme_constant_override("separation", 10)
	strip.add_child(strip_row)
	for item in [["配备导丝", "直径: 0.25mm\n长度: 215cm", UiStyle.GREEN], ["远端材质", "亲水涂层", UiStyle.BLUE], ["软头形状", "J-tip\n45°", UiStyle.YELLOW], ["分段长度", "支撑段: 185cm\n自由段: 30cm", UiStyle.TEXT_MID]]:
		strip_row.add_child(_dsa_device_block(item[0], item[1], item[2]))
	ov.add_child(strip)


# Endoscope pane (middle top): a private World3D renders a second copy of the real
# vessel GLB from the streamed guidewire-front pose. Keeping it independent from
# the navigation pane prevents its warm fog/light/material from leaking into the
# external 3D view and removes the old transparent procedural-tunnel fallback.
func _build_scope_pane(rootc: Control) -> void:
	var scope := PanelContainer.new()
	scope.add_theme_stylebox_override("panel", UiStyle.panel_box(0.95, 8))
	UiStyle.place(scope, UiStyle.scope_rect())
	rootc.add_child(scope)
	_scope_pane = scope

	var view_box := Control.new()
	view_box.clip_contents = true
	view_box.mouse_filter = Control.MOUSE_FILTER_IGNORE
	view_box.set_anchors_and_offsets_preset(Control.PRESET_FULL_RECT)
	view_box.offset_left = 10
	view_box.offset_top = 44
	view_box.offset_right = -10
	view_box.offset_bottom = -52
	scope.add_child(view_box)

	_scope_fallback = EndoscopeFallbackScript.new()
	_scope_fallback.set_anchors_and_offsets_preset(Control.PRESET_FULL_RECT)
	view_box.add_child(_scope_fallback)

	var vpc := SubViewportContainer.new()
	vpc.stretch = true
	vpc.mouse_filter = Control.MOUSE_FILTER_IGNORE
	vpc.set_anchors_and_offsets_preset(Control.PRESET_FULL_RECT)
	view_box.add_child(vpc)
	_scope_view_container = vpc

	_scope_viewport = SubViewport.new()
	_scope_viewport.own_world_3d = true
	_scope_viewport.transparent_bg = false
	_scope_viewport.render_target_clear_mode = SubViewport.CLEAR_MODE_ALWAYS
	_scope_viewport.render_target_update_mode = SubViewport.UPDATE_ALWAYS
	vpc.add_child(_scope_viewport)
	_setup_scope_environment()

	_scope_camera = Camera3D.new()
	_scope_camera.near = 0.0005
	_scope_camera.far = 2.0
	_scope_camera.fov = 88.0
	_scope_camera.cull_mask = _SCOPE_LAYER_MASK
	_scope_viewport.add_child(_scope_camera)
	_scope_camera.make_current()
	_scope_camera_filter = EndoscopeCameraFilterScript.new()
	_scope_camera_filter.configure(scope_render_quality)
	_scope_lighting = EndoscopeLightingRigScript.new()
	_scope_lighting.configure(
		_SCOPE_LAYER_MASK, scope_render_profile, scope_render_quality)
	_scope_camera.add_child(_scope_lighting)
	# Compatibility handle used by older UI code/tests; brightness is now applied
	# through the full key/fill rig so their energy ratio stays stable.
	_scope_headlight = _scope_lighting.key_light
	_apply_scope_render_quality()

	_scope_overlay = EndoscopeOverlayScript.new()
	_scope_overlay.set_anchors_and_offsets_preset(Control.PRESET_FULL_RECT)
	_scope_overlay.visible = false
	view_box.add_child(_scope_overlay)

	var ov := Control.new()
	ov.set_anchors_and_offsets_preset(Control.PRESET_FULL_RECT)
	ov.mouse_filter = Control.MOUSE_FILTER_IGNORE
	scope.add_child(ov)
	var badge := PanelContainer.new()
	badge.position = Vector2(12, 7)
	badge.custom_minimum_size = Vector2(28, 28)
	badge.add_theme_stylebox_override("panel",
		UiStyle.bordered_box(Color(0.035, 0.12, 0.20, 0.9), UiStyle.BLUE, 6, 1))
	var badge_label := UiStyle.label("2", UiStyle.BLUE, 16)
	badge_label.horizontal_alignment = HORIZONTAL_ALIGNMENT_CENTER
	badge_label.vertical_alignment = VERTICAL_ALIGNMENT_CENTER
	badge.add_child(badge_label)
	ov.add_child(badge)
	_corner_label(ov, "腔镜实时视图", UiStyle.TEXT, Vector2(48, 10), 16)
	_scope_live_label = UiStyle.label("○ WAIT", UiStyle.TEXT2, 12)
	_scope_live_label.anchor_left = 1.0
	_scope_live_label.anchor_right = 1.0
	_scope_live_label.offset_left = -92
	_scope_live_label.offset_top = 12
	_scope_live_label.offset_right = -14
	_scope_live_label.horizontal_alignment = HORIZONTAL_ALIGNMENT_RIGHT
	ov.add_child(_scope_live_label)

	var controls := HBoxContainer.new()
	controls.add_theme_constant_override("separation", 7)
	controls.anchor_top = 1.0
	controls.anchor_bottom = 1.0
	controls.offset_left = 12
	controls.offset_top = -42
	controls.offset_right = -12
	controls.offset_bottom = -7
	controls.grow_vertical = Control.GROW_DIRECTION_BEGIN
	ov.add_child(controls)
	var reset_btn := _scope_icon_button("refresh", "重置腔镜姿态")
	reset_btn.pressed.connect(_reset_scope_view)
	controls.add_child(reset_btn)
	_scope_rec_button = _scope_icon_button("record", "开始/停止本地录制计时", UiStyle.RED)
	_scope_rec_button.toggle_mode = true
	_scope_rec_button.toggled.connect(_on_scope_record_toggled)
	controls.add_child(_scope_rec_button)
	_scope_time_label = UiStyle.label("00:00:00", UiStyle.TEXT_MID, 13)
	_scope_time_label.custom_minimum_size = Vector2(66, 28)
	_scope_time_label.vertical_alignment = VERTICAL_ALIGNMENT_CENTER
	controls.add_child(_scope_time_label)
	controls.add_child(PaneToolIcon.new("brightness", UiStyle.TEXT_MID))
	var slider := HSlider.new()
	slider.min_value = 0.55
	slider.max_value = 1.35
	slider.step = 0.01
	slider.value = 0.92
	slider.custom_minimum_size = Vector2(92, 28)
	slider.size_flags_horizontal = Control.SIZE_EXPAND_FILL
	slider.tooltip_text = "腔镜亮度"
	slider.value_changed.connect(_on_scope_brightness_changed)
	controls.add_child(slider)
	_scope_brightness_slider = slider
	_scope_capture_button = _scope_icon_button("capture", "保存当前腔镜帧")
	_scope_capture_button.pressed.connect(_capture_scope_frame)
	controls.add_child(_scope_capture_button)
	var fullscreen_btn := _scope_icon_button("fullscreen", "扩大/恢复腔镜窗格")
	fullscreen_btn.pressed.connect(_toggle_scope_fullscreen)
	controls.add_child(fullscreen_btn)
	_update_scope_state()


func _setup_scope_environment() -> void:
	if _scope_viewport == null:
		return
	var world_env := WorldEnvironment.new()
	var env := Environment.new()
	env.background_mode = Environment.BG_COLOR
	env.background_color = Color(0.055, 0.006, 0.004)
	env.ambient_light_source = Environment.AMBIENT_SOURCE_COLOR
	env.ambient_light_color = Color(0.52, 0.10, 0.045)
	env.ambient_light_energy = 0.64
	env.tonemap_mode = Environment.TONE_MAPPER_ACES
	env.glow_enabled = true
	env.glow_intensity = 0.34
	env.glow_bloom = 0.06
	env.glow_hdr_threshold = 1.15
	env.fog_enabled = true
	env.fog_mode = Environment.FOG_MODE_EXPONENTIAL
	env.fog_light_color = Color(0.055, 0.006, 0.004)
	# World units are metres. A high density makes a 5–10 cm lumen collapse into
	# one flat red field; this light haze preserves branch depth without hiding it.
	env.fog_density = 1.0
	_scope_environment = env
	world_env.environment = env
	_scope_viewport.add_child(world_env)


func _apply_scope_render_quality() -> void:
	if _scope_viewport == null:
		return
	# TAA is intentionally disabled for a fast-moving near-wall camera; quality
	# presets use private-viewport MSAA and never alter the main navigation view.
	_scope_viewport.use_taa = false
	if scope_render_profile == "baseline":
		_scope_viewport.msaa_3d = Viewport.MSAA_DISABLED
		_scope_target_fov = 85.0
		if _scope_environment != null:
			_scope_environment.glow_intensity = 0.34
			_scope_environment.glow_bloom = 0.06
			_scope_environment.tonemap_exposure = 1.0
			_scope_environment.ambient_light_color = Color(0.52, 0.10, 0.045)
		return
	if _scope_environment != null:
		_scope_environment.ambient_light_color = Color(0.58, 0.15, 0.065)
	match scope_render_quality:
		"performance":
			_scope_viewport.msaa_3d = Viewport.MSAA_DISABLED
			_scope_target_fov = 78.0
			if _scope_environment != null:
				_scope_environment.glow_intensity = 0.18
				_scope_environment.glow_bloom = 0.035
				_scope_environment.tonemap_exposure = 0.58
		"high":
			_scope_viewport.msaa_3d = Viewport.MSAA_4X
			_scope_target_fov = 80.0
			if _scope_environment != null:
				_scope_environment.glow_intensity = 0.28
				_scope_environment.glow_bloom = 0.05
				_scope_environment.tonemap_exposure = 0.68
		_:
			_scope_viewport.msaa_3d = Viewport.MSAA_2X
			_scope_target_fov = 79.0
			if _scope_environment != null:
				_scope_environment.glow_intensity = 0.24
				_scope_environment.glow_bloom = 0.04
				_scope_environment.tonemap_exposure = 0.62


func _scope_frame_ready() -> bool:
	return _connection_ready and _scope_has_tip \
		and _scope_vessel != null and is_instance_valid(_scope_vessel)


func _update_scope_state() -> void:
	var ready := _scope_frame_ready()
	if _scope_view_container != null:
		_scope_view_container.visible = ready
	if _scope_overlay != null:
		_scope_overlay.visible = ready
	if _scope_fallback != null:
		_scope_fallback.visible = not ready
	if _scope_live_label != null:
		_scope_live_label.text = "● LIVE" if ready else "○ WAIT"
		_scope_live_label.add_theme_color_override(
			"font_color", UiStyle.GREEN if ready else UiStyle.TEXT2)
	for button in [_scope_rec_button, _scope_capture_button]:
		if button != null:
			button.disabled = not ready
	if _scope_brightness_slider != null:
		_scope_brightness_slider.editable = ready
		_scope_brightness_slider.mouse_filter = (
			Control.MOUSE_FILTER_STOP if ready else Control.MOUSE_FILTER_IGNORE)
	if ready:
		return
	_scope_pose_initialized = false
	if _scope_camera_filter != null:
		_scope_camera_filter.reset()
	if _scope_recording:
		_scope_recording = false
		_scope_record_elapsed_msec = 0
		if _scope_rec_button != null:
			_scope_rec_button.set_pressed_no_signal(false)
		_update_scope_timer()
	if _scope_fallback == null:
		return
	if not _connection_ready:
		_scope_fallback.call("set_status", "等待仿真连接",
			"会话就绪后显示真实血管腔内画面", UiStyle.TEXT2)
	elif _scope_vessel == null or not is_instance_valid(_scope_vessel):
		_scope_fallback.call("set_status", "血管模型不可用",
			"请检查当前模型 GLB 的 Godot 导入状态", UiStyle.YELLOW)
	else:
		_scope_fallback.call("set_status", "等待导丝尖端位姿",
			"收到首个 state_batch 后自动切换为 LIVE", UiStyle.BLUE)


func _reset_scope_view() -> void:
	if not _scope_frame_ready():
		return
	_scope_pose_initialized = false
	if _scope_camera_filter != null:
		_scope_camera_filter.reset()
	print("[Scope] camera smoothing reset at current guidewire pose")


func _on_scope_record_toggled(active: bool) -> void:
	if active and not _scope_frame_ready():
		_scope_rec_button.set_pressed_no_signal(false)
		return
	if active:
		_scope_recording = true
		_scope_record_elapsed_msec = 0
		_scope_record_started_msec = Time.get_ticks_msec()
		_scope_time_label.add_theme_color_override("font_color", UiStyle.RED)
	else:
		if _scope_recording:
			_scope_record_elapsed_msec += Time.get_ticks_msec() - _scope_record_started_msec
		_scope_recording = false
		_scope_time_label.add_theme_color_override("font_color", UiStyle.TEXT_MID)
	_update_scope_timer()


func _update_scope_timer() -> void:
	if _scope_time_label == null:
		return
	var elapsed := _scope_record_elapsed_msec
	if _scope_recording:
		elapsed += Time.get_ticks_msec() - _scope_record_started_msec
	var total_seconds: int = maxi(0, int(elapsed / 1000))
	var hours: int = total_seconds / 3600
	var minutes: int = (total_seconds % 3600) / 60
	var seconds: int = total_seconds % 60
	_scope_time_label.text = "%02d:%02d:%02d" % [hours, minutes, seconds]


func _on_scope_brightness_changed(value: float) -> void:
	var gain := clampf(value, 0.55, 1.35)
	if _scope_vessel_mat != null:
		EndoscopeMaterialFactoryScript.apply_brightness(_scope_vessel_mat, gain)
	if _scope_lighting != null and is_instance_valid(_scope_lighting):
		_scope_lighting.set_brightness(gain)


func _capture_scope_frame() -> void:
	if not _scope_frame_ready() or _scope_viewport == null:
		return
	var stamp := Time.get_datetime_string_from_system().replace(":", "-")
	_save_scope_frame("user://endoscope_capture_%s.png" % stamp)


func _save_scope_frame(path: String) -> bool:
	var image := _scope_viewport.get_texture().get_image()
	if image == null or image.is_empty():
		push_warning("[Scope] capture skipped: SubViewport returned no image")
		return false
	if path.contains("scope-validation") or path.contains("scope_validation"):
		var albedo_format := -1
		var albedo_texture: Texture2D
		if _scope_vessel_mat is StandardMaterial3D:
			albedo_texture = (
				_scope_vessel_mat as StandardMaterial3D).albedo_texture
		elif _scope_vessel_mat is ShaderMaterial:
			albedo_texture = (
				_scope_vessel_mat as ShaderMaterial).get_shader_parameter(
					"albedo_texture") as Texture2D
		if albedo_texture != null:
			var albedo_image := albedo_texture.get_image()
			if albedo_image != null and not albedo_image.is_empty():
				albedo_format = albedo_image.get_format()
				print("[ScopeValidation] albedo_samples=%s,%s" % [
					albedo_image.get_pixel(0, 0),
					albedo_image.get_pixel(
						int(albedo_image.get_width() / 2),
						int(albedo_image.get_height() / 2)),
				])
		print("[ScopeValidation] image_format=%d albedo_format=%d" % [
			image.get_format(), albedo_format])
	if image.get_format() not in [Image.FORMAT_RGB8, Image.FORMAT_RGBA8]:
		image.convert(Image.FORMAT_RGBA8)
	var err := image.save_png(path)
	if err == OK:
		print("[Scope] frame saved: %s" % ProjectSettings.globalize_path(path))
		return true
	else:
		push_warning("[Scope] failed to save frame (%d): %s" % [err, path])
		return false


func _toggle_scope_fullscreen() -> void:
	if _scope_pane == null:
		return
	_scope_fullscreen = not _scope_fullscreen
	_scope_pane.z_index = 40 if _scope_fullscreen else 0
	if _scope_fullscreen:
		UiStyle.place(_scope_pane, Rect2(8, 112, 1252, 688))
	else:
		UiStyle.place(_scope_pane, UiStyle.scope_rect())

# 3D navigation assistant pane (VPP §2.3, right-top). A framed SubViewport with its own
# World3D, plus reference overlays: title, a left tool strip and a direction-cube stub.
func _build_3d_pane(rootc: Control) -> void:
	_pane_3d_container = SubViewportContainer.new()
	_pane_3d_container.stretch = true
	# Clicks are translated manually in _on_navigate_click, so the container itself
	# ignores the mouse and the global input_handler keeps working unchanged.
	_pane_3d_container.mouse_filter = Control.MOUSE_FILTER_IGNORE
	UiStyle.place(_pane_3d_container, UiStyle.pane3d_rect())
	rootc.add_child(_pane_3d_container)

	_world = SubViewport.new()
	_world.own_world_3d = true  # isolate 3D to this pane (no full-window bleed)
	_world.transparent_bg = false
	_world.render_target_update_mode = SubViewport.UPDATE_ALWAYS
	_pane_3d_container.add_child(_world)

	# A framed border + overlays on top of the viewport.
	var frame := Panel.new()
	frame.add_theme_stylebox_override("panel", _hollow_border())
	frame.set_anchors_and_offsets_preset(Control.PRESET_FULL_RECT)
	frame.mouse_filter = Control.MOUSE_FILTER_IGNORE
	_pane_3d_container.add_child(frame)

	_corner_label(frame, "③  3D 解剖导航", UiStyle.TEXT, Vector2(12, 8), 16)

	# 右侧竖向图标工具栏 (参考图): 选择/旋转/平移 are an exclusive group gating
	# what a left drag does; 放大/缩小 step the orbit zoom; 复位 reframes the vessel.
	var tools := VBoxContainer.new()
	tools.add_theme_constant_override("separation", 2)
	tools.anchor_left = 1.0
	tools.anchor_right = 1.0
	tools.offset_left = -44
	tools.offset_top = 88
	tools.offset_right = -14
	tools.grow_horizontal = Control.GROW_DIRECTION_BEGIN
	frame.add_child(tools)

	var mode_group := ButtonGroup.new()
	var select_btn := _icon_tool("select", "选择：单击路径点导航")
	select_btn.toggle_mode = true
	select_btn.button_group = mode_group
	select_btn.toggled.connect(func(on: bool) -> void:
		if on:
			_view_tool_mode = ViewToolMode.SELECT)
	var orbit_btn := _icon_tool("orbit", "旋转：左键拖拽环绕视角")
	orbit_btn.toggle_mode = true
	orbit_btn.button_group = mode_group
	orbit_btn.button_pressed = true
	orbit_btn.toggled.connect(func(on: bool) -> void:
		if on:
			_view_tool_mode = ViewToolMode.ORBIT)
	var pan_btn := _icon_tool("pan", "平移：左键拖拽移动视图中心")
	pan_btn.toggle_mode = true
	pan_btn.button_group = mode_group
	pan_btn.toggled.connect(func(on: bool) -> void:
		if on:
			_view_tool_mode = ViewToolMode.PAN)
	_follow_btn = _icon_tool("follow", "跟随：视角自动跟随导丝（点击其他区域退出）")
	_follow_btn.toggle_mode = true
	_follow_btn.toggled.connect(_on_follow_toggled)
	var zin_btn := _icon_tool("zoom_in", "放大 (滚轮)")
	zin_btn.pressed.connect(func() -> void: _zoom_by(2.0))
	var zout_btn := _icon_tool("zoom_out", "缩小 (滚轮)")
	zout_btn.pressed.connect(func() -> void: _zoom_by(-2.0))
	var reset_btn := _icon_tool("frame", "复位视角")
	reset_btn.pressed.connect(_reset_view)
	for b in [select_btn, orbit_btn, pan_btn, _follow_btn, zin_btn, zout_btn, reset_btn]:
		tools.add_child(b)

	# 方向立方体 (§11 / 参考图右上): a real wireframe cube in its own transparent
	# SubViewport; _sync_direction_cube counter-rotates it against the active 3D
	# camera each frame so its faces always show the world (patient) orientation.
	var cube_vpc := SubViewportContainer.new()
	cube_vpc.stretch = true
	cube_vpc.mouse_filter = Control.MOUSE_FILTER_IGNORE
	cube_vpc.anchor_left = 1.0
	cube_vpc.anchor_right = 1.0
	cube_vpc.offset_left = -82
	cube_vpc.offset_top = 12
	cube_vpc.offset_right = -12
	cube_vpc.offset_bottom = 82
	frame.add_child(cube_vpc)

	var cube_vp := SubViewport.new()
	cube_vp.own_world_3d = true
	cube_vp.transparent_bg = true
	cube_vp.render_target_update_mode = SubViewport.UPDATE_ALWAYS
	cube_vpc.add_child(cube_vp)

	_cube_root = Node3D.new()
	cube_vp.add_child(_cube_root)
	_cube_root.add_child(_wireframe_cube())
	# Anatomical face labels. Axis mapping assumes the LPS→glTF export convention
	# (tools/export_godot_assets.py): +X=L/-X=R, +Y=S/-Y=I, +Z=A/-Z=P. If实机
	# anatomy reads flipped, adjust the axis signs here only.
	for face in [["L", Vector3.RIGHT], ["R", Vector3.LEFT], ["S", Vector3.UP],
			["I", Vector3.DOWN], ["A", Vector3.BACK], ["P", Vector3.FORWARD]]:
		var lbl := Label3D.new()
		lbl.text = face[0]
		lbl.font_size = 96
		lbl.pixel_size = 0.004
		lbl.modulate = Color(UiStyle.BLUE.r, UiStyle.BLUE.g, UiStyle.BLUE.b, 0.95)
		lbl.billboard = BaseMaterial3D.BILLBOARD_ENABLED
		lbl.no_depth_test = true
		lbl.position = (face[1] as Vector3) * 0.72
		_cube_root.add_child(lbl)

	var cube_cam := Camera3D.new()
	cube_cam.projection = Camera3D.PROJECTION_ORTHOGONAL
	cube_cam.size = 2.3
	cube_cam.position = Vector3(0, 0, 3)
	cube_vp.add_child(cube_cam)



# A compact 30x30 icon tool button for the 3D pane's right strip. Seven controls
# fit between the direction cube and pane bottom while retaining the established
# hover #1A2A3A and selected/pressed #2F8CFF states.
func _icon_tool(kind: String, tip: String) -> Button:
	var b := Button.new()
	b.custom_minimum_size = Vector2(30, 30)
	b.tooltip_text = tip
	b.add_theme_stylebox_override("normal", UiStyle.card_box(0.8, 6))
	b. add_theme_stylebox_override("hover", UiStyle.flat(Color(0.102, 0.165, 0.227), 6))
	b.add_theme_stylebox_override("pressed", UiStyle.flat(UiStyle.BLUE, 6))
	b.add_theme_stylebox_override("focus", StyleBoxEmpty.new())
	var ic: Control = preload("res://scripts/ui/pane_tool_icon.gd").new(kind, UiStyle.TEXT)
	ic.set_anchors_and_offsets_preset(Control.PRESET_FULL_RECT)
	ic.offset_left = 4
	ic.offset_top = 4
	ic.offset_right = -4
	ic.offset_bottom = -4
	b.add_child(ic)
	return b


# Unit wireframe cube (12 edges) in the pane's accent blue, unshaded.
func _wireframe_cube() -> MeshInstance3D:
	var corners: Array = []
	for x in [-0.5, 0.5]:
		for y in [-0.5, 0.5]:
			for z in [-0.5, 0.5]:
				corners.append(Vector3(x, y, z))
	var mat := StandardMaterial3D.new()
	mat.shading_mode = BaseMaterial3D.SHADING_MODE_UNSHADED
	mat.albedo_color = Color(UiStyle.BLUE.r, UiStyle.BLUE.g, UiStyle.BLUE.b, 0.85)
	mat.transparency = BaseMaterial3D.TRANSPARENCY_ALPHA
	var im := ImmediateMesh.new()
	im.surface_begin(Mesh.PRIMITIVE_LINES, mat)
	for i in corners.size():
		for j in range(i + 1, corners.size()):
			var a: Vector3 = corners[i]
			var b: Vector3 = corners[j]
			# An edge joins corners differing in exactly one axis.
			var diff := int(a.x != b.x) + int(a.y != b.y) + int(a.z != b.z)
			if diff == 1:
				im.surface_add_vertex(a)
				im.surface_add_vertex(b)
	im.surface_end()
	var mi := MeshInstance3D.new()
	mi.mesh = im
	return mi


# Counter-rotate the direction cube against the active 3D camera so the cube's
# world-axis faces always match what the operator sees in the pane.
func _sync_direction_cube() -> void:
	if _cube_root == null or not is_instance_valid(_cube_root) or _world == null:
		return
	var cam := _world.get_camera_3d()
	if cam == null:
		return
	_cube_root.transform.basis = cam.global_transform.basis.inverse()


# ── Route risk + entry marker bookkeeping ─────────────────────────────────────
# Called whenever route waypoints stream in (first batch / reset / branch switch).
# The route line's curvature hint gradient is computed inside path_renderer from
# the polyline itself. The orbit camera is not re-pivoted here: left-drag should
# stay a free vessel-level orbit, not a rotation around the green entry marker.
func _apply_route_features() -> void:
	if _path == null or not is_instance_valid(_path) or not _path.is_inside_tree():
		return
	var pts: Array = []  # Vector3, path-local frame
	for wp in _path_waypoints:
		if typeof(wp) == TYPE_ARRAY and (wp as Array).size() >= 3:
			pts.append(Vector3(float(wp[0]), float(wp[1]), float(wp[2])))
	if pts.size() < 8:
		return

	# No fake no-go/risk overlays: leave the route unforced unless the backend
	# provides real source-backed spatial risk data.
	if _path.has_method("clear_forced_ranges"):
		_path.clear_forced_ranges()
	else:
		_path.set_risk_ranges([])

	_entry_world = _path.global_transform * (pts[0] as Vector3)
	_entry_known = true


# 跟随 toolbar toggle: on -> chase-follow the wire; off -> tip-centered orbit.
func _on_follow_toggled(on: bool) -> void:
	if on:
		_set_camera_mode(CamMode.FOLLOW)
	elif _cam_mode == CamMode.FOLLOW:
		_exit_follow_to_free()


# Leave 跟随 into an overview orbit around the guidewire front. The pivot must be
# the rendered wire front (bodies[-1] / tip.position), not the route target sphere.
func _exit_follow_to_free() -> void:
	_orbit_preset = OrbitPreset.CLINICAL
	_set_camera_mode(CamMode.OVERVIEW)
	_camera_user_controlled = true
	if _tip_world_known:
		_set_orbit_focus(_tip_world_last, true)
	_update_orbit_camera()


# Add a right-anchored 互换 (swap) button to a pane overlay at `top` px from the
# pane's top edge. Both panes get one so the swap is always reachable from the
# pane the operator is looking at; X key does the same via the input handler.
func _add_swap_button(parent: Control, top: float) -> void:
	var b := _pane_tool("互换", Vector2(56, 32))
	b.anchor_left = 1.0
	b.anchor_right = 1.0
	b.offset_left = -70
	b.offset_top = top
	b.offset_right = -14
	b.offset_bottom = top + 32
	b.grow_horizontal = Control.GROW_DIRECTION_BEGIN
	b.grow_vertical = Control.GROW_DIRECTION_END
	b.tooltip_text = "互换 DSA 实时影像与 3D 血管导航窗口 (X)"
	b.pressed.connect(_swap_panes)
	parent.add_child(b)


# Swap the DSA 实时影像 and 3D 血管导航 panes between the big left region and the
# small right-top region (doc/11 §2 layout). Both panes keep their full subtree
# (overlays, tool strips, legend, viewport); only their placement rects change —
# anchored overlays adapt and the SubViewportContainer (stretch=true) resizes the
# 3D render target automatically. Click-to-navigate keeps working because it
# translates clicks via the container's live global rect.
func _swap_panes() -> void:
	_panes_swapped = false
	_apply_pane_layout()
	print("[Main] fixed reference panes: DSA / scope / 3D / safety")


func _apply_pane_layout() -> void:
	if _dsa_pane != null:
		UiStyle.place(_dsa_pane, UiStyle.dsa_rect())
	if _scope_pane != null:
		if _scope_fullscreen:
			UiStyle.place(_scope_pane, Rect2(8, 112, 1252, 688))
		else:
			UiStyle.place(_scope_pane, UiStyle.scope_rect())
	if _pane_3d_container != null:
		UiStyle.place(_pane_3d_container, UiStyle.pane3d_rect())



func _dsa_device_block(title: String, detail: String, color: Color) -> PanelContainer:
	var panel := PanelContainer.new()
	panel.add_theme_stylebox_override("panel", UiStyle.flat(Color(0, 0, 0, 0), 6))
	panel.size_flags_horizontal = Control.SIZE_EXPAND_FILL
	var row := HBoxContainer.new()
	row.add_theme_constant_override("separation", 8)
	panel.add_child(row)
	var dial := Control.new()
	dial.custom_minimum_size = Vector2(48, 48)
	dial.draw.connect(func() -> void:
		var c := dial.size * 0.5
		dial.draw_circle(c, 22.0, Color(0.047, 0.118, 0.176, 0.85))
		dial.draw_arc(c, 20.0, -PI * 0.8, PI * 0.55, 32, color, 2.0, true)
		dial.draw_circle(c + Vector2(8, -6), 3.0, color)
	)
	row.add_child(dial)
	var vb := VBoxContainer.new()
	vb.add_theme_constant_override("separation", 1)
	vb.add_child(UiStyle.label(title, UiStyle.TEXT, 12))
	vb.add_child(UiStyle.label(detail, UiStyle.TEXT_MID, 11))
	row.add_child(vb)
	return panel
func _corner_label(parent: Control, text: String, color: Color, pos: Vector2,
		size := 12) -> void:
	var l := UiStyle.label(text, color, size)
	l.position = pos
	parent.add_child(l)


# A placeholder tool button for the pane tool strips (§7/§10: hover #1A2A3A,
# 选中 #2F8CFF; visual only for now).
func _pane_tool(text: String, btn_size: Vector2) -> Button:
	var b := Button.new()
	b.text = text
	b.add_theme_font_override("font", UiStyle.font())
	b.add_theme_font_size_override("font_size", 12)
	b.add_theme_color_override("font_color", UiStyle.TEXT)
	b.custom_minimum_size = btn_size
	b.add_theme_stylebox_override("normal", UiStyle.card_box(0.8, 8))
	b.add_theme_stylebox_override("hover", UiStyle.flat(Color(0.102, 0.165, 0.227), 8))
	b.add_theme_stylebox_override("pressed", UiStyle.flat(UiStyle.BLUE, 8))
	return b


func _scope_icon_button(kind: String, tooltip: String,
		icon_color: Color = UiStyle.TEXT_MID) -> Button:
	var b := Button.new()
	b.custom_minimum_size = Vector2(32, 28)
	b.tooltip_text = tooltip
	b.focus_mode = Control.FOCUS_NONE
	b.add_theme_stylebox_override("normal", UiStyle.card_box(0.78, 6))
	b.add_theme_stylebox_override("hover", UiStyle.flat(Color(0.10, 0.17, 0.24), 6))
	b.add_theme_stylebox_override("pressed", UiStyle.flat(Color(0.07, 0.22, 0.39), 6))
	b.add_theme_stylebox_override("disabled", UiStyle.flat(Color(0.05, 0.07, 0.09, 0.55), 6))
	var icon := PaneToolIcon.new(kind, icon_color)
	icon.set_anchors_and_offsets_preset(Control.PRESET_FULL_RECT)
	icon.offset_left = 5
	icon.offset_top = 3
	icon.offset_right = -5
	icon.offset_bottom = -3
	b.add_child(icon)
	return b


# A transparent stylebox with just a rounded border, for framing the 3D viewport.
func _hollow_border() -> StyleBoxFlat:
	var sb := UiStyle.flat(Color(0, 0, 0, 0), 8)
	sb.border_color = UiStyle.BORDER
	sb.set_border_width_all(1)
	return sb


func _setup_environment() -> void:
	var world_env := WorldEnvironment.new()
	var env := Environment.new()
	env.background_mode = Environment.BG_COLOR
	env.background_color = UiStyle.PANE3D_BG  # 050B12 (doc/11 §9 深黑蓝)
	env.ambient_light_source = Environment.AMBIENT_SOURCE_COLOR
	env.ambient_light_color = Color(0.22, 0.36, 0.55)
	env.ambient_light_energy = 0.45
	# Bloom/glow: the "通电发光" look of the reference view. Only fragments whose
	# EMISSION exceeds 1.0 spill light (hdr_threshold), so the fresnel rims (glow
	# uniform > 1), the white route line and the marker spheres halo while the flat
	# UI/backdrop stays crisp.
	env.glow_enabled = true
	env.glow_intensity = 0.78
	env.glow_bloom = 0.10
	env.glow_hdr_threshold = 0.95
	env.tonemap_mode = Environment.TONE_MAPPER_ACES
	# Depth fog for the endoscope tunnel cue: near wall keeps its red, the far
	# lumen fades toward the (near-black) fog color, giving depth WITHOUT any
	# light that could clip a mm-close wall to white. High density because the
	# lumen is only centimeters across. Enabled only in endoscope view (see
	# _set_camera_mode) so it does not darken the overview/follow scene.
	env.fog_enabled = false
	env.fog_mode = Environment.FOG_MODE_EXPONENTIAL
	env.fog_light_color = Color(0.06, 0.01, 0.01)
	env.fog_density = 45.0
	_env = env
	world_env.environment = env
	_world.add_child(world_env)


func _setup_camera_and_light() -> void:
	var light := DirectionalLight3D.new()
	light.rotation_degrees = Vector3(-50, -30, 0)
	light.light_energy = 1.1
	_world.add_child(light)

	_camera = Camera3D.new()
	_camera.near = 0.001
	_camera.far = 50.0
	_camera.fov = 42.0
	_camera.cull_mask = 0xFFFFF & ~_SCOPE_LAYER_MASK
	_world.add_child(_camera)
	# Activate only after the camera is in the scene tree, otherwise it may not
	# become the active viewport camera.
	_camera.make_current()


func _resolve_vessel_glb() -> String:
	var candidates := _vessel_glb_candidates()
	for glb in candidates:
		if ResourceLoader.exists(glb):
			return glb
	return str(candidates[0]) if not candidates.is_empty() else ""


func _vessel_glb_candidates() -> Array:
	# Prefer the phantom-named GLB (e.g. segment_part.glb); all VPP cases share the
	# blood_vessels export. Do NOT silently substitute a different anatomy: the
	# guidewire/path stream in this phantom's frame, so drawing another vessel
	# leaves the wire floating far from it (e.g. segment_part is ~0.8 m off-origin).
	# When the named GLB is missing (not yet imported in the Godot editor),
	# _setup_vessel warns and renders no vessel rather than the wrong one.
	var high: String
	var native: String
	var fallback: String
	if phantom.ends_with("_vpp"):
		native = "res://assets/models/blood_vessels_visual_native.glb"
		high = "res://assets/models/blood_vessels_visual_high.glb"
		fallback = "res://assets/models/blood_vessels.glb"
	else:
		native = "res://assets/models/%s_visual_native.glb" % phantom
		high = "res://assets/models/%s_visual_high.glb" % phantom
		fallback = "res://assets/models/%s.glb" % phantom
	return [native, high, fallback]


func _setup_vessel() -> Node3D:
	var candidates := _vessel_glb_candidates()
	var packed: PackedScene = null
	var glb := ""
	for candidate in candidates:
		var path := str(candidate)
		if not ResourceLoader.exists(path):
			continue
		var scene: PackedScene = load(path)
		if scene != null:
			packed = scene
			glb = path
			break
		push_warning("Failed to load vessel GLB (import may have failed): %s" % path)
	if packed == null:
		push_warning("Vessel GLB not found or not imported. Tried: %s. Run tools/export_godot_assets.py and open Godot once to import." % str(candidates))
		return null
	if candidates.size() > 1 and glb == str(candidates[candidates.size() - 1]):
		push_warning("Native/high-quality vessel GLB not imported; falling back to %s." % glb)
	if _is_debug_low_poly_phantom(phantom):
		push_warning("%s is a debug low-poly phantom; use case_001_vpp or segment_part for surface relief QA." % phantom)
	var vessel: Node3D = packed.instantiate()
	_world.add_child(vessel)
	var mesh_count := vessel.find_children("*", "MeshInstance3D", true, false).size()
	print("[Main] vessel loaded from %s, MeshInstance3D count=%d" % [glb, mesh_count])
	_apply_vessel_material(vessel)
	_scope_vessel = packed.instantiate()
	if _scope_viewport != null:
		_scope_viewport.add_child(_scope_vessel)
	else:
		_world.add_child(_scope_vessel)
	_prepare_scope_vessel(_scope_vessel)
	_update_scope_state()
	return vessel


func _prepare_scope_vessel(node: Node) -> void:
	if node == null:
		return
	node.name = "EndoscopeVesselLayer"
	for mi in node.find_children("*", "MeshInstance3D", true, false):
		mi.layers = _SCOPE_LAYER_MASK
		mi.material_override = _scope_vessel_mat
		mi.cast_shadow = GeometryInstance3D.SHADOW_CASTING_SETTING_OFF


func _is_debug_low_poly_phantom(name: String) -> bool:
	return name in ["low_tort", "aorta_trunk", "aorta_tree"]


func _apply_vessel_material(node: Node) -> void:
	# Overview/demo material: cyan-blue fresnel glass (参考图 三维导航视图 look,
	# superseding the doc/11 §9 暗红 palette by user decision). The fresnel fixes
	# the α-stacking haze at the root: the lumen CENTRE is near-transparent (so
	# overlapping walls no longer accumulate) while the silhouette EDGE is a thin
	# blooming rim — the tube boundary defines the structure instead of alpha
	# stacking. Unshaded so it cannot clip to white from any camera angle.
	_vessel_mat_overlay = _make_fresnel_material(false)
	_vessel_mat_overlay.set_shader_parameter("rim_color", Color(0.96, 0.36, 0.16))
	_vessel_mat_overlay.set_shader_parameter("core_color", Color(0.20, 0.055, 0.025))
	_vessel_mat_overlay.set_shader_parameter("back_wall_color", Color(0.64, 0.20, 0.10))
	_vessel_mat_overlay.set_shader_parameter("core_alpha", 0.12)
	_vessel_mat_overlay.set_shader_parameter("rim_alpha", 1.0)
	_vessel_mat_overlay.set_shader_parameter("glow", 3.8)
	_vessel_mat_overlay.set_shader_parameter("relief_light_dir", Vector3(-0.35, 0.72, 0.59))
	_vessel_mat_overlay.set_shader_parameter("relief_strength", 0.42)
	_vessel_mat_overlay.set_shader_parameter("relief_shadow", 0.34)

	# Interior material (endoscope): opaque so the lumen wall is visible from
	# inside instead of see-through. Double-sided so Godot flips back-face
	# normals for correct shading. A flat dark-red emission keeps every wall an even
	# tone (it cannot clip to white like a close light does); the endoscope's depth
	# fog supplies the near/far gradient so the tunnel does not read as one solid
	# field.
	_vessel_mat_interior = StandardMaterial3D.new()
	_vessel_mat_interior.albedo_color = Color(0.55, 0.22, 0.20)
	_vessel_mat_interior.cull_mode = BaseMaterial3D.CULL_DISABLED
	_vessel_mat_interior.roughness = 0.85
	_vessel_mat_interior.metallic = 0.0
	_vessel_mat_interior.emission_enabled = true
	_vessel_mat_interior.emission = Color(0.36, 0.13, 0.12)
	_vessel_mat_interior.emission_energy_multiplier = 0.3

	# The private scope material is profile-driven and never displaces the real
	# vessel mesh. Baseline preserves the previous look; enhanced adds independent
	# macro colour, mesoscopic roughness and micro-normal triplanar fields.
	_scope_vessel_mat = EndoscopeMaterialFactoryScript.create_material(
		scope_render_profile, scope_render_quality)
	var initial_brightness := 0.92
	if _scope_brightness_slider != null:
		initial_brightness = float(_scope_brightness_slider.value)
	EndoscopeMaterialFactoryScript.apply_brightness(
		_scope_vessel_mat, initial_brightness)

	# Surgical (follow) material: the same Fresnel glow, additionally faded by
	# distance from the guidewire tip. Fragments within `fade_near` of the tip keep
	# full glow; past `fade_far` they vanish, so the follow view shows only the local
	# glowing lumen segment and the distant tree no longer piles up. `tip_world_pos`
	# is refreshed each frame in _feed_rig.
	_vessel_mat_surgical = _make_fresnel_material(true)
	_vessel_mat_surgical.set_shader_parameter("rim_color", Color(0.98, 0.39, 0.17))
	_vessel_mat_surgical.set_shader_parameter("core_color", Color(0.22, 0.060, 0.030))
	_vessel_mat_surgical.set_shader_parameter("back_wall_color", Color(0.68, 0.22, 0.11))
	_vessel_mat_surgical.set_shader_parameter("core_alpha", 0.070)
	_vessel_mat_surgical.set_shader_parameter("rim_alpha", 0.82)
	_vessel_mat_surgical.set_shader_parameter("glow", 3.0)
	_vessel_mat_surgical.set_shader_parameter("fade_near", surgical_fade_near)
	_vessel_mat_surgical.set_shader_parameter("fade_far", surgical_fade_far)
	_vessel_mat_surgical.set_shader_parameter("route_corridor_radius", route_vessel_radius)
	_vessel_mat_surgical.set_shader_parameter("route_corridor_feather", route_vessel_feather)
	_vessel_mat_surgical.set_shader_parameter("relief_light_dir", Vector3(-0.35, 0.72, 0.59))
	_vessel_mat_surgical.set_shader_parameter("relief_strength", 0.34)
	_vessel_mat_surgical.set_shader_parameter("relief_shadow", 0.26)
	_update_vessel_focus_tip(Vector3.ZERO, false)
	_update_vessel_route_visibility([], false)

	_vessel_meshes = node.find_children("*", "MeshInstance3D", true, false)
	_set_vessel_view_material(CamMode.OVERVIEW)


# Build the 青蓝菲涅尔发光 vessel shader (参考图 glass-tree look).
#
# Shared behaviour:
#   - FRESNEL edge: near-transparent lumen centre (core_alpha ≈ 0, so overlapping
#     walls never stack into fog) with a thin cyan rim whose EMISSION exceeds 1.0
#     (glow uniform) so the environment bloom halos it.
#   - CAMERA-PROXIMITY fade: fragments within cam_fade_near..far of the camera
#     dissolve, so a branch sweeping the lens (orbit zoom-in, follow view) can
#     never shroud the view — the structural fix for the 红雾/笼罩 problem.
#
# Overview (with_fade=false): alpha is fresnel-driven — silhouette rims define the
# tree viewed from outside.
#
# Follow/surgical (with_fade=true): almost every wall is at a grazing angle where
# fresnel saturates, so alpha is CAPPED (0.6) and multiplied by a tip-proximity
# fade: only the local lumen segment near the wire stays visible.
func _make_fresnel_material(with_fade: bool) -> ShaderMaterial:
	var fade_decl := ""
	var alpha_line: String
	if with_fade:
		fade_decl = "uniform float fade_near = 0.03;\n" \
			+ "uniform float fade_far = 0.11;\n"
		alpha_line = "	float tip_prox = 1.0 - smoothstep(fade_near, fade_far, distance(tip_world_pos, world_pos));\n" \
			+ "	float route_prox = route_visibility(world_pos);\n" \
			+ "	float keep = max(tip_prox * (1.0 - route_focus_enabled), route_prox * route_focus_enabled);\n" \
			+ "	ALPHA = clamp(core_alpha + rim_alpha * rim, 0.0, 0.52) * keep * cam_fade;\n"
	else:
		alpha_line = "	ALPHA = clamp(core_alpha + rim_alpha * rim, 0.0, 0.68) * focus_alpha * cam_fade;\n"
	var shader := Shader.new()
	shader.code = "shader_type spatial;\n" \
		+ "render_mode cull_disabled, unshaded, depth_draw_never, blend_mix;\n" \
		+ "uniform vec3 rim_color : source_color = vec3(0.18, 0.67, 1.0);\n" \
		+ "uniform vec3 core_color : source_color = vec3(0.005, 0.030, 0.075);\n" \
		+ "uniform vec3 back_wall_color : source_color = vec3(0.030, 0.180, 0.360);\n" \
		+ "uniform float rim_power = 2.65;\n" \
		+ "uniform float core_alpha = 0.014;\n" \
		+ "uniform float rim_alpha = 0.46;\n" \
		+ "uniform float glow = 2.9;\n" \
		+ "uniform vec3 relief_light_dir = vec3(-0.35, 0.72, 0.59);\n" \
		+ "uniform float relief_strength = 0.42;\n" \
		+ "uniform float relief_shadow = 0.34;\n" \
		+ "uniform float cam_fade_near = 0.012;\n" \
		+ "uniform float cam_fade_far = 0.045;\n" \
		+ "uniform vec3 tip_world_pos;\n" \
		+ "uniform float focus_enabled = 0.0;\n" \
		+ "uniform float focus_near = 0.035;\n" \
		+ "uniform float focus_far = 0.22;\n" \
		+ "uniform float focus_alpha_far = 0.30;\n" \
		+ "uniform float focus_emission_far = 0.24;\n" \
		+ "uniform float route_focus_enabled = 0.0;\n" \
		+ "uniform int route_point_count = 0;\n" \
		+ "uniform vec3 route_points[96];\n" \
		+ "uniform float route_corridor_radius = 0.024;\n" \
		+ "uniform float route_corridor_feather = 0.018;\n" \
		+ fade_decl \
		+ "varying vec3 world_pos;\n" \
		+ "void vertex() { world_pos = (MODEL_MATRIX * vec4(VERTEX, 1.0)).xyz; }\n" \
		+ "float dist_to_segment(vec3 p, vec3 a, vec3 b) {\n" \
		+ "	vec3 ab = b - a;\n" \
		+ "	float d = dot(ab, ab);\n" \
		+ "	if (d < 0.0000000001) { return distance(p, a); }\n" \
		+ "	float t = clamp(dot(p - a, ab) / d, 0.0, 1.0);\n" \
		+ "	return distance(p, a + ab * t);\n" \
		+ "}\n" \
		+ "float route_visibility(vec3 p) {\n" \
		+ "	if (route_point_count < 2) { return 0.0; }\n" \
		+ "	float best = 1000000.0;\n" \
		+ "	for (int i = 0; i < 95; i++) {\n" \
		+ "		if (i >= route_point_count - 1) { break; }\n" \
		+ "		best = min(best, dist_to_segment(p, route_points[i], route_points[i + 1]));\n" \
		+ "	}\n" \
		+ "	return 1.0 - smoothstep(route_corridor_radius, route_corridor_radius + route_corridor_feather, best);\n" \
		+ "}\n" \
		+ "void fragment() {\n" \
		+ "	float f = 1.0 - clamp(dot(normalize(NORMAL), normalize(VIEW)), 0.0, 1.0);\n" \
		+ "	float rim = pow(f, rim_power);\n" \
		+ "	float back = pow(1.0 - rim, 1.7) * 0.38;\n" \
		+ "	float focus = mix(1.0, 1.0 - smoothstep(focus_near, focus_far, distance(tip_world_pos, world_pos)), focus_enabled);\n" \
		+ "	float focus_alpha = mix(focus_alpha_far, 1.0, focus);\n" \
		+ "	float focus_emission = mix(focus_emission_far, 1.0, focus);\n" \
		+ "	vec3 n = normalize(NORMAL);\n" \
		+ "	float lit = clamp(dot(n, normalize(relief_light_dir)) * 0.5 + 0.5, 0.0, 1.0);\n" \
		+ "	float relief = mix(1.0 - relief_shadow, 1.0 + relief_strength, lit);\n" \
		+ "	vec3 vessel_color = mix(mix(core_color, back_wall_color, back), rim_color, rim);\n" \
		+ "	ALBEDO = vessel_color * relief;\n" \
		+ "	EMISSION = rim_color * rim * glow * focus_emission * mix(0.72, 1.0, lit);\n" \
		+ "	float cam_fade = smoothstep(cam_fade_near, cam_fade_far, distance(CAMERA_POSITION_WORLD, world_pos));\n" \
		+ alpha_line \
		+ "}\n"
	var mat := ShaderMaterial.new()
	mat.shader = shader
	return mat


# Select the vessel wall material for the active camera mode: whole-tree translucent
# in overview (演示视图), tip-proximity fade in follow (手术视图), opaque inner wall
# in endoscope.
func _update_vessel_focus_tip(tip_world: Vector3, enabled: bool) -> void:
	var focus_far := 0.22
	var focus_near := 0.035
	var radius := _last_aabb.size.length() * 0.5
	if radius > 0.0:
		focus_near = clampf(radius * 0.045, 0.018, 0.06)
		focus_far = clampf(radius * 0.42, 0.10, 0.42)
	for mat in [_vessel_mat_overlay, _vessel_mat_surgical]:
		if mat == null:
			continue
		mat.set_shader_parameter("tip_world_pos", tip_world)
		mat.set_shader_parameter("focus_enabled", 1.0 if enabled else 0.0)
		mat.set_shader_parameter("focus_near", focus_near)
		mat.set_shader_parameter("focus_far", focus_far)


func _update_vessel_route_visibility(waypoints: Array, enabled: bool, vessel_radius: float = -1.0) -> void:
	var route_points: Array = []
	route_points.resize(_VESSEL_ROUTE_SHADER_SAMPLES)
	for i in range(_VESSEL_ROUTE_SHADER_SAMPLES):
		route_points[i] = Vector3.ZERO

	var route_count := 0
	var radius_m := vessel_radius
	if radius_m > 1.0:
		radius_m *= 0.001
	var corridor_radius := route_vessel_radius
	if radius_m > 0.0:
		corridor_radius = maxf(route_vessel_radius, radius_m * 1.35)
	var corridor_feather := maxf(route_vessel_feather, corridor_radius * 0.35)
	if enabled and waypoints.size() >= 2:
		route_count = min(_VESSEL_ROUTE_SHADER_SAMPLES, waypoints.size())
		var route_xform := Transform3D.IDENTITY
		if _path != null and is_instance_valid(_path):
			route_xform = _path.global_transform
		elif _vessel != null and is_instance_valid(_vessel):
			route_xform = _vessel.global_transform
		for i in range(route_count):
			var src_idx := int(round(float(i) * float(waypoints.size() - 1) / float(route_count - 1)))
			route_points[i] = route_xform * _vec3_from_backend_point(waypoints[src_idx])

	for mat in [_vessel_mat_overlay, _vessel_mat_surgical]:
		if mat == null:
			continue
		mat.set_shader_parameter("route_focus_enabled", 1.0 if route_count >= 2 else 0.0)
		mat.set_shader_parameter("route_point_count", route_count)
		mat.set_shader_parameter("route_points", route_points)
		mat.set_shader_parameter("route_corridor_radius", corridor_radius)
		mat.set_shader_parameter("route_corridor_feather", corridor_feather)


func _set_vessel_view_material(mode: int) -> void:
	var mat: Material
	match mode:
		CamMode.ENDOSCOPE:
			mat = _vessel_mat_interior
		CamMode.FOLLOW:
			mat = _vessel_mat_surgical
		_:
			mat = _vessel_mat_overlay
	if mat == null:
		return
	for child in _vessel_meshes:
		child.material_override = mat


func _setup_guidewire(parent: Node) -> void:
	_guidewire = preload("res://scripts/guidewire_renderer.gd").new()
	parent.add_child(_guidewire)


func _setup_path(parent: Node) -> void:
	_path = preload("res://scripts/path_renderer.gd").new()
	parent.add_child(_path)


func _setup_entry_marker(parent: Node) -> void:
	# Parented under the vessel frame (same as the guidewire/path) so the entry
	# and target marker coordinates need no conversion.
	_entry_marker = preload("res://scripts/entry_marker.gd").new()
	parent.add_child(_entry_marker)


func _setup_rig(parent: Node) -> void:
	# Parent under the vessel frame so tip coordinates need no conversion.
	_rig = preload("res://scripts/camera_rig.gd").new()
	parent.add_child(_rig)
	_rig.follow_cam.cull_mask = 0xFFFFF & ~_SCOPE_LAYER_MASK
	_rig.endoscope_cam.cull_mask = (0xFFFFF & ~(1 << 1)) & ~_SCOPE_LAYER_MASK
	if not _path_waypoints.is_empty():
		_rig.set_navigation_route(_path_waypoints)
	_seed_rig_pose_from_config()



func _seed_rig_pose_from_config() -> void:
	if _rig == null or not is_instance_valid(_rig):
		return
	var start := _config_point_to_scene(start_position)
	var finish := _config_point_to_scene(end_position)
	var dir := finish - start
	if dir.length() < 1e-6:
		dir = Vector3(0.0, 1.0, 0.0)
	_rig.update_tip(start, dir.normalized(), Quaternion.IDENTITY)


func _config_point_to_scene(point: Array) -> Vector3:
	if point.size() < 3:
		return Vector3.ZERO
	var p := Vector3(float(point[0]), float(point[1]), float(point[2]))
	var largest := maxf(absf(p.x), maxf(absf(p.y), absf(p.z)))
	return p * 0.001 if largest > 10.0 else p
func _setup_hud() -> void:
	_hud = preload("res://scripts/hud_controller.gd").new()
	add_child(_hud)


func _setup_network_and_input() -> void:
	_ws = preload("res://scripts/websocket_client.gd").new()
	# Push the navigation configuration before the node enters the tree, so the
	# first session_start (sent in _process) carries the right phantom/route.
	_ws.phantom = phantom
	_ws.target = target
	_ws.case_id = case_id
	_ws.start_position = start_position
	_ws.end_position = end_position
	_ws.physics_engine = physics_engine
	add_child(_ws)
	_input = preload("res://scripts/input_handler.gd").new()
	add_child(_input)

	_ws.connected.connect(_on_connected)
	_ws.disconnected.connect(_on_disconnected)
	_ws.connection_state_changed.connect(_on_connection_state_changed)
	_ws.error_received.connect(_on_server_error)
	_ws.session_started.connect(_on_session_started)
	_ws.routes_received.connect(_on_routes)
	_ws.batch_received.connect(_on_batch)
	_ws.state_received.connect(_on_state)
	_ws.emergency_stop_confirmed.connect(_on_emergency_stop_confirmed)
	_ws.resume_confirmed.connect(_on_resume_confirmed)
	_ws.control_rejected.connect(_on_control_rejected)
	_ws.control_config_received.connect(_on_control_config_received)

	# Interactive deformation panel: live-tune the Newton force-drive params. It's a
	# developer tool (not part of the VPP §2 operator dashboard), so it starts hidden
	# and is toggled by the 形变 G tool button / HUD deform_toggle signal.
	_deform = preload("res://scripts/deform_panel.gd").new()
	add_child(_deform)
	_deform.visible = false
	_deform.param_changed.connect(_ws.send_engine_params)
	_ws.engine_params_received.connect(_deform.sync_effective)

	_input.control.connect(_on_keyboard_control)
	_input.input_state.connect(_hud.update_input)
	_input.reset_requested.connect(_on_reset_button)
	_input.view_cycle.connect(_cycle_camera_mode)
	_input.model_cycle.connect(_cycle_model)
	_input.branch_cycle.connect(_cycle_branch)
	_input.pane_swap.connect(_swap_panes)
	_input.pointer_down.connect(_on_pointer_down)
	_input.pointer_drag.connect(_on_pointer_drag)
	_input.pointer_up.connect(_on_pointer_up)
	_input.pan_drag.connect(_on_pan_drag)
	_input.wheel_zoom.connect(_on_wheel_zoom)
	_input.autopilot_off.connect(_disengage_autopilot)
	_ws.shape_intent_received.connect(_on_shape_intent)

	# Dashboard control buttons (设计图 仪表盘): mouse equivalents of the safety
	# actions and view/model/branch/reset shortcuts, so the platform is operable
	# without the keyboard.
	_hud.emergency_stop.connect(_on_emergency_stop)
	_hud.manual_takeover.connect(_disengage_autopilot)
	_hud.resume_nav.connect(_on_resume_nav)
	_hud.motion_command.connect(_on_motion)
	_hud.control_profile_changed.connect(_input.set_control_profile)
	_hud.protection_changed.connect(_on_protection_changed)
	_hud.view_cycle_requested.connect(_cycle_camera_mode)
	_hud.model_cycle_requested.connect(_cycle_model)
	_hud.branch_cycle_requested.connect(_cycle_branch)
	_hud.reset_requested.connect(_on_reset_button)
	_hud.deform_toggle.connect(_on_deform_toggle)


func _on_connected() -> void:
	print("[Main] WebSocket connected")
	_last_msg = "connected"
	_update_scope_state()
	_update_debug()


func _on_disconnected() -> void:
	print("[Main] WebSocket disconnected")
	_connection_ready = false
	_controls_blocked = true
	_input.set_control_enabled(false)
	_last_msg = "DISCONNECTED"
	_update_scope_state()
	_update_debug()


func _on_connection_state_changed(state: String, details: Dictionary) -> void:
	_connection_ready = state == "ready"
	_controls_blocked = not _connection_ready or _ws.controls_blocked()
	_input.set_control_enabled(not _controls_blocked)
	_hud.set_connection_state(state, details)
	_update_scope_state()


func _on_server_error(err: Dictionary) -> void:
	push_warning("[Main] server error: %s" % str(err))
	var code := str(err.get("code", "?"))
	var message := str(err.get("message", ""))
	_last_msg = "error %s: %s" % [code, message]
	_update_debug()


func _on_session_started(sid: String, state: Dictionary, metadata: Dictionary) -> void:
	print("[Main] session started: %s" % sid)
	_session_id = sid.substr(0, 8) if sid.length() >= 8 else sid
	_last_msg = "session_started"
	_hud.apply_control_state(metadata.get("control_state", {}) as Dictionary)
	_controls_blocked = not _connection_ready or _ws.controls_blocked()
	_input.set_control_enabled(not _controls_blocked)
	_update_debug()
	if not state.is_empty():
		_on_state(state)


# Capture the selectable branch targets for the active phantom (aorta_tree ships
# many; single-route models send an empty dict). Sorted for a stable B-key order.
func _on_routes(routes: Dictionary) -> void:
	_branch_targets = routes.keys()
	_branch_targets.sort()
	_branch_index = 0
	if _branch_targets.is_empty():
		_hud.set_model(str(MODELS[_model_index].name))
		return
	_hud.set_model("%s  ·  分支 B 切换 (%d)" % [
		str(MODELS[_model_index].name), _branch_targets.size()])
	print("[Main] %d branch targets available (press B to cycle)" % _branch_targets.size())


# Cycle to the next target branch (B key) and ask the backend to re-route there.
func _cycle_branch() -> void:
	if _controls_blocked or _branch_targets.size() < 2:
		return
	_branch_index = (_branch_index + 1) % _branch_targets.size()
	var target := str(_branch_targets[_branch_index])
	_ws.send_select_route(target)
	_path_waypoints = []  # the new branch re-streams its route
	if _rig != null and is_instance_valid(_rig):
		_rig.clear_navigation_route()
	_update_vessel_route_visibility([], false)
	_camera_user_controlled = false
	_disengage_autopilot()
	_auto_followed = false  # re-drop into follow view on the new branch's first pose
	_hud.set_model("%s  ·  分支 %d/%d %s" % [
		str(MODELS[_model_index].name), _branch_index + 1,
		_branch_targets.size(), target])
	print("[Main] select branch -> %s" % target)


func _on_batch(batch: Dictionary) -> void:
	_sync_control_state(batch.get("control_state", {}) as Dictionary)
	if not _logged_first_batch:
		_logged_first_batch = true
		var path: Dictionary = batch.get("path", {})
		var diag: Dictionary = batch.get("diagnostics", {}) as Dictionary
		print("[Main] first state_batch engine=%s mode=%s waypoints=%d target=%s diagnostics=%s" % [
			str(batch.get("engine", "")),
			str(batch.get("fidelity_mode", "")),
			(path.get("waypoints", []) as Array).size(),
			str(batch.get("target", [])),
			str(diag),
		])
	# Capture the route waypoints (backend meter frame) for click-to-navigate;
	# they arrive only on the first batch / after a reset or route switch.
	var path_info: Dictionary = batch.get("path", {})
	var path_wps: Array = path_info.get("waypoints", [])
	var radius_value: Variant = path_info.get("vessel_radius", null)
	if (typeof(radius_value) == TYPE_FLOAT or typeof(radius_value) == TYPE_INT) \
			and float(radius_value) > 0.0005:
		_scope_radius_target = clampf(float(radius_value), 0.0015, 0.008)
	if not path_wps.is_empty():
		_path_waypoints = path_wps
		if _rig != null and is_instance_valid(_rig):
			_rig.set_navigation_route(path_wps)
		var route_radius := -1.0
		if typeof(radius_value) == TYPE_FLOAT or typeof(radius_value) == TYPE_INT:
			route_radius = float(radius_value)
		_update_vessel_route_visibility(path_wps, true, route_radius)
		_apply_route_features()
	_guidewire.update_from_batch(batch)
	_path.update_from_batch(batch)
	_entry_marker.update_from_batch(batch)
	_sync_entry_marker_visibility()
	_feed_rig(_guidewire_front_pose_from_batch(batch))
	var safety: Dictionary = batch.get("safety", {})
	var diagnostics: Dictionary = batch.get("diagnostics", {}) as Dictionary
	_hud.set_backend(
		str(batch.get("engine", "")),
		str(batch.get("fidelity_mode", "")),
		diagnostics
	)
	var guidewire_state: Dictionary = batch.get("guidewire", {}) as Dictionary
	if guidewire_state.is_empty() and diagnostics.has("guidewire"):
		guidewire_state = diagnostics.get("guidewire", {}) as Dictionary
	var support_state: Dictionary = batch.get("support", {}) as Dictionary
	if support_state.is_empty() and diagnostics.has("support"):
		support_state = diagnostics.get("support", {}) as Dictionary
	_hud.set_device_state(
		guidewire_state,
		support_state,
		batch.get("risk", {}) as Dictionary,
		batch.get("procedure", {}) as Dictionary
	)
	var status := str(safety.get("status", "STANDBY"))
	# Fixed control-mode display (VPP §2.1/§2.4): SAFE HOLD on a collision stop,
	# SUPERVISED AUTO while click-autopilot drives, otherwise MANUAL.
	var mode := "手动"
	if _ws.controls_blocked():
		mode = "急停 STOP"
	elif status == "COLLISION_STOP":
		mode = "安全保持"
	elif _autopilot_active:
		mode = "自动"
	_hud.set_control_mode(mode)
	# v3 canonical fields live at data.<field> for both state message types.
	# Keep render-only path/safety blocks separate and never infer missing force or
	# risk values in the UI layer.
	var metrics := batch.duplicate(true)
	if _ws.last_latency_ms >= 0.0:
		metrics["latency_ms"] = _ws.last_latency_ms
	_hud.update_metrics(metrics)
	_hud.update_safety_contract(safety)
	# While click autopilot is engaged, keep the nav line live with progress so the
	# slow tip advance is legible as motion, not a hang.
	if _autopilot_active:
		var prog := float(batch.get("path", {}).get("progress", 0.0)) * 100.0
		_hud.set_nav("自动 Auto → %s · %.0f%%" % [_autopilot_wp_text, prog], true)
	_msg_count += 1
	_last_msg = "state_batch"
	_update_debug()


func _on_state(state: Dictionary) -> void:
	_sync_control_state(state.get("control_state", {}) as Dictionary)
	_guidewire.update_from_state(state)
	var safety: Dictionary = state.get("safety", {}) as Dictionary
	_hud.set_control_mode("急停 STOP" if _ws.controls_blocked() else "手动")
	if _ws.last_latency_ms >= 0.0:
		state["latency_ms"] = _ws.last_latency_ms
	_hud.update_metrics(state)
	_hud.update_safety_contract(safety)
	_msg_count += 1
	_last_msg = "state_update"
	_update_debug()


func _sync_control_state(control_state: Dictionary) -> void:
	if control_state.is_empty():
		return
	_hud.apply_control_state(control_state)
	if bool(control_state.get("emergency_stop_latched", false)):
		_controls_blocked = true
		_input.set_control_enabled(false)


func _fidelity_label(mode: String) -> String:
	match mode:
		"guided":
			return "运动学演示 GUIDED"
		"rl":
			return "策略推理 RL"
		_:
			return "物理仿真 PHYSICS"


func _remaining_from_waypoints(progress: float) -> float:
	if _path_waypoints.size() < 2:
		return 0.0
	var total := 0.0
	var prev := _vec3_from_backend_point(_path_waypoints[0])
	for i in range(1, _path_waypoints.size()):
		var cur := _vec3_from_backend_point(_path_waypoints[i])
		total += prev.distance_to(cur)
		prev = cur
	return maxf(0.0, (1.0 - clampf(progress, 0.0, 1.0)) * total)


func _vec3_from_backend_point(p) -> Vector3:
	if typeof(p) == TYPE_VECTOR3:
		return p
	if typeof(p) == TYPE_ARRAY and (p as Array).size() >= 3:
		var a := p as Array
		return Vector3(float(a[0]), float(a[1]), float(a[2]))
	return Vector3.ZERO


# Per-frame: keep the direction cube counter-rotated to the active camera; while
# click autopilot is engaged, tick the backend at ~20 Hz with a neutral control
# frame (the server overrides it with the ShapeIntentController output, so the
# tip keeps advancing toward the clicked waypoint without keyboard input).
func _process(delta: float) -> void:
	_sync_direction_cube()
	_sync_scope_endoscope_camera(delta)
	_update_scope_timer()
	_sample_scope_performance(delta)
	_tick_scope_validation_capture(delta)
	if _controls_blocked or not _autopilot_active:
		return
	_autopilot_accum += delta
	if _autopilot_accum < _AUTOPILOT_TICK:
		return
	_autopilot_accum = 0.0
	_ws.send_control(0.0, 0.0)


func _tick_scope_validation_capture(delta: float) -> void:
	if not _scope_validation_capture_pending or not _scope_frame_ready():
		return
	if _scope_validation_motion_stage == 1:
		_scope_validation_motion_accum += delta
		if _scope_validation_motion_accum < 0.05:
			return
		_scope_validation_motion_accum = 0.0
		if _ws != null and is_instance_valid(_ws) and not _controls_blocked:
			_ws.send_control(0.28, 0.18)
			_scope_validation_motion_steps += 1
		if _scope_validation_motion_steps < 60:
			return
		_ws.send_control(0.0, 0.0)
		_scope_validation_motion_stage = 2
		_scope_validation_capture_frames = 0
		print("[ScopeValidation] real motion controls complete: %d" %
			_scope_validation_motion_steps)
		return
	_scope_validation_capture_frames += 1
	if _scope_validation_capture_frames < 180:
		return
	var suffix := ""
	if _scope_validation_motion:
		suffix = "before" if _scope_validation_motion_stage == 0 else "after"
	var path := _scope_validation_capture_path(suffix)
	if not _save_scope_frame(path):
		return
	if _scope_validation_motion and _scope_validation_motion_stage == 0:
		_scope_validation_motion_stage = 1
		_scope_validation_capture_frames = 0
		print("[ScopeValidation] before-motion capture complete")
	else:
		_scope_validation_capture_pending = false
		print("[ScopeValidation] capture complete")


func _scope_validation_capture_path(suffix: String) -> String:
	var path := _scope_validation_output
	if path.is_empty():
		path = "user://scope_validation_%s_%s.png" % [
			scope_render_profile, scope_render_quality]
	if suffix.is_empty():
		return path
	return "%s_%s.%s" % [
		path.get_basename(), suffix, path.get_extension()]


func _sample_scope_performance(delta: float) -> void:
	if _scope_perf_reported or not _scope_frame_ready():
		return
	_scope_frame_times_ms.append(delta * 1000.0)
	if _scope_frame_times_ms.size() < _SCOPE_PERF_SAMPLE_COUNT:
		return
	var sorted_samples := _scope_frame_times_ms.duplicate()
	sorted_samples.sort()
	var total_ms := 0.0
	for sample in _scope_frame_times_ms:
		total_ms += sample
	var average_ms := total_ms / float(_scope_frame_times_ms.size())
	var p95_index := clampi(
		int(ceil(float(sorted_samples.size()) * 0.95)) - 1,
		0,
		sorted_samples.size() - 1)
	var average_fps := 1000.0 / maxf(average_ms, 0.001)
	print("[ScopePerf] profile=%s quality=%s frames=%d avg_fps=%.1f avg_ms=%.2f p95_ms=%.2f" % [
		scope_render_profile,
		scope_render_quality,
		_scope_frame_times_ms.size(),
		average_fps,
		average_ms,
		sorted_samples[p95_index],
	])
	_scope_perf_reported = true



func _sync_scope_endoscope_camera(delta: float) -> void:
	if _scope_viewport == null or _scope_camera == null:
		return
	if _rig == null or not is_instance_valid(_rig) or _rig.endoscope_cam == null:
		return
	var target_transform: Transform3D = _rig.endoscope_cam.global_transform
	if not _scope_pose_initialized:
		_scope_camera.global_transform = target_transform
		_scope_pose_initialized = true
		if _scope_camera_filter != null:
			_scope_camera_filter.reset()
	else:
		if scope_render_profile == "baseline":
			var blend := clampf(scope_camera_smooth * delta, 0.0, 1.0)
			_scope_camera.global_transform = _scope_camera.global_transform.interpolate_with(
				target_transform, blend)
		elif _scope_camera_filter != null:
			_scope_camera.global_transform = _scope_camera_filter.update(
				target_transform, delta)
	_scope_camera.fov = (
		_rig.endoscope_cam.fov
		if scope_render_profile == "baseline"
		else _scope_target_fov)
	_scope_camera.near = _rig.endoscope_cam.near
	if _scope_lighting != null and is_instance_valid(_scope_lighting):
		_scope_lighting.update_radius(_scope_radius_target, delta)
		var radius_m: float = _scope_lighting.get_smoothed_radius()
		if scope_render_profile == "baseline":
			_scope_camera.far = minf(2.0, _rig.endoscope_cam.far)
			if _scope_environment != null:
				_scope_environment.fog_density = 1.0
		else:
			_scope_camera.far = minf(
				clampf(radius_m * 45.0, 0.30, 0.90),
				_rig.endoscope_cam.far)
			if _scope_environment != null:
				var radius_t := clampf(
					inverse_lerp(0.0015, 0.006, radius_m), 0.0, 1.0)
				_scope_environment.fog_density = lerpf(0.35, 0.85, radius_t)
# Left click -> pick the route waypoint whose ON-SCREEN position is nearest the
# click, and drive the tip there via the backend autopilot. Robust to coordinate
# frames: the waypoints are in the backend meter frame and rendered under the path
# node, so we project them to screen via that node's world transform + the camera
# and send back the exact backend waypoint (no inverse transform needed).
#
# NB: we score by 2D screen distance, NOT perpendicular distance to the pick ray.
# Ray-perpendicular distance is depth-biased — a point near the camera origin has a
# tiny perpendicular distance for any ray, so in the close-up follow view (camera
# sitting on the tip ≈ waypoint 0) every click degenerated to selecting waypoint 0.
func _on_navigate_click(screen_pos: Vector2) -> void:
	if _controls_blocked or _path_waypoints.is_empty() or _path == null or not is_instance_valid(_path):
		return
	# Clicks that land on a UI button (pane tools / 互换 / dashboard) are button
	# presses, not navigation intents — the input handler emits before GUI
	# delivery, so filter them here via the hovered control.
	var hovered := get_viewport().gui_get_hovered_control()
	if hovered is BaseButton:
		return
	# The 3D now renders inside the pane's SubViewport, so query THAT camera (not the
	# root viewport's). Clicks arrive in root-window coords: ignore clicks outside the
	# 3D pane and translate the rest into pane-local space so they match the camera's
	# unproject_position (which is in SubViewport pixel space).
	if _world == null or _pane_3d_container == null:
		return
	var cam := _world.get_camera_3d()
	if cam == null:
		return
	var pane_rect := _pane_3d_container.get_global_rect()
	if not pane_rect.has_point(screen_pos):
		return
	var local_pos := screen_pos - pane_rect.position
	var xform: Transform3D = _path.global_transform

	var best_index := -1
	var best_screen_dist := INF
	for i in _path_waypoints.size():
		var wp: Variant = _path_waypoints[i]
		if typeof(wp) != TYPE_ARRAY or (wp as Array).size() < 3:
			continue
		var world: Vector3 = xform * Vector3(float(wp[0]), float(wp[1]), float(wp[2]))
		if cam.is_position_behind(world):
			continue
		var screen_dist := cam.unproject_position(world).distance_to(local_pos)
		if screen_dist < best_screen_dist:
			best_screen_dist = screen_dist
			best_index = i

	if best_index < 0:
		return
	var target: Array = _path_waypoints[best_index]
	_last_click_target = target.duplicate()
	_autopilot_active = true
	_autopilot_accum = _AUTOPILOT_TICK  # tick immediately next frame
	_autopilot_wp_text = "waypoint %d/%d" % [best_index + 1, _path_waypoints.size()]
	_ws.send_shape_intent(true, target)
	print("[Main] click autopilot -> %s %s" % [_autopilot_wp_text, str(target)])
	# Visual + HUD feedback so the slow autopilot reads as engaged: a cyan goal
	# marker at the clicked waypoint and a nav status line (doc/09 §9.5 boundary 3).
	if _entry_marker != null and is_instance_valid(_entry_marker):
		_entry_marker.set_goal(Vector3(float(target[0]), float(target[1]), float(target[2])))
	_hud.set_nav("自动 Auto → %s" % _autopilot_wp_text, true)


# Cancel click autopilot and hand control back to manual push/rotate.
func _disengage_autopilot() -> void:
	if not _autopilot_active:
		return
	_autopilot_active = false
	_autopilot_accum = 0.0
	_autopilot_wp_text = ""
	if _ws != null and is_instance_valid(_ws):
		_ws.send_shape_intent(false)
	print("[Main] click autopilot disengaged")
	if _entry_marker != null and is_instance_valid(_entry_marker):
		_entry_marker.clear_goal()
	_hud.set_nav("手动 Manual", false)


# Any manual W/S/A/D tick cancels click autopilot so the operator regains control.
func _on_manual_control(_push: float, _rotate: float) -> void:
	_disengage_autopilot()


func _on_keyboard_control(push: float, rotate: float) -> void:
	if _controls_blocked:
		return
	_on_manual_control(push, rotate)
	_ws.send_control(push, rotate)


# 急停 button: gate every local control source immediately, then wait for the
# backend latch confirmation before the HUD says the stop succeeded.
func _on_emergency_stop() -> void:
	_controls_blocked = true
	_input.set_control_enabled(false)
	_disengage_autopilot()
	if _ws != null and is_instance_valid(_ws):
		_ws.send_emergency_stop("operator_hud")
	print("[Main] emergency stop requested")


# 恢复 only releases the backend latch. Automatic navigation is intentionally not
# re-engaged; the operator must issue a fresh click/command after confirmation.
func _on_resume_nav() -> void:
	if _ws != null and is_instance_valid(_ws):
		_ws.send_resume()
	print("[Main] resume requested")


func _on_emergency_stop_confirmed(control_state: Dictionary) -> void:
	_controls_blocked = true
	_input.set_control_enabled(false)
	_hud.confirm_emergency_stop(control_state)
	print("[Main] emergency stop confirmed by backend")


func _on_resume_confirmed(control_state: Dictionary) -> void:
	_controls_blocked = not _connection_ready or bool(
		control_state.get("emergency_stop_latched", false)
	)
	_input.set_control_enabled(not _controls_blocked)
	_hud.confirm_resume(control_state)
	print("[Main] resume confirmed by backend")


func _on_control_rejected(rejection: Dictionary) -> void:
	var control_state: Dictionary = rejection.get("control_state", {}) as Dictionary
	_sync_control_state(control_state)
	_hud.add_log_line("控制被后端拒绝：%s" % str(rejection.get("reason", "unknown")))


func _on_control_config_received(control_state: Dictionary) -> void:
	_hud.apply_control_state(control_state)


func _on_protection_changed(name: String, enabled: bool) -> void:
	if _ws != null and is_instance_valid(_ws):
		var config := {}
		config[name] = enabled
		_ws.send_control_config(config)


# 重置 button: cancel autopilot and reset the episode.
func _on_reset_button() -> void:
	if _controls_blocked:
		return
	_disengage_autopilot()
	if _ws != null and is_instance_valid(_ws):
		_ws.send_reset()
	print("[Main] reset (button)")


# 形变 G button: show/hide the developer deformation-tuning panel.
func _on_deform_toggle() -> void:
	if _deform != null and is_instance_valid(_deform):
		_deform.visible = not _deform.visible


# 运动控制 buttons (推进/旋转): grab manual control and send one control tick.
func _on_motion(push: float, rotate: float) -> void:
	if _controls_blocked:
		return
	_disengage_autopilot()
	if _ws != null and is_instance_valid(_ws):
		_ws.send_control(push, rotate)


func _on_shape_intent(result: Dictionary) -> void:
	print("[Main] shape_intent ack: %s" % str(result))


func _feed_rig(tip: Dictionary) -> void:
	if tip.is_empty():
		return
	var pos := _to_vec3(tip.get("position", []))
	var dir := _to_vec3(tip.get("direction", []))
	var quat := _to_quat(tip.get("quaternion", []))
	_rig.update_tip(pos, dir, quat)
	_scope_has_tip = true
	_update_scope_state()
	# Refresh the surgical-view fade origin: the tip in world space (the path node
	# shares the vessel frame, so its transform maps the frame-local tip to world,
	# matching how click-nav projects waypoints).
	if _path != null and is_instance_valid(_path):
		var tip_world: Vector3 = _path.global_transform * pos
		_update_vessel_focus_tip(tip_world, true)
		# §2.6 bottom status bar: tip world coordinate. Also the pivot the free
		# orbit view drops onto when 跟随 is exited.
		_hud.set_coord(tip_world)
		_tip_world_last = tip_world
		_tip_world_known = true
	# Keep the middle-bottom 3D pane in external navigation by default; the
	# endoscope reference pane is visible above it.
	if not _auto_followed:
		_auto_followed = true


# Camera orbit/follow uses the same front point the renderer draws as the
# guidewire tip. Bodies are streamed root->front; prefer bodies[-1] so Newton
# debug frames cannot accidentally follow the proximal/root body or route target.
func _guidewire_front_pose_from_batch(batch: Dictionary) -> Dictionary:
	var bodies: Array = batch.get("bodies", [])
	if bodies.size() >= 2:
		var prev: Dictionary = bodies[bodies.size() - 2]
		var front: Dictionary = bodies[bodies.size() - 1]
		if prev.has("pos") and front.has("pos"):
			var p0 := _to_vec3(prev["pos"])
			var p1 := _to_vec3(front["pos"])
			var dir := p1 - p0
			if dir.length() > 1e-6:
				return {
					"position": front["pos"],
					"direction": [dir.x, dir.y, dir.z],
					"quaternion": front.get("quat", []),
				}
	return batch.get("tip", {})


func _cycle_camera_mode() -> void:
	if _cam_mode == CamMode.OVERVIEW:
		if _orbit_preset == OrbitPreset.CLINICAL:
			_set_orbit_preset(OrbitPreset.TREE, true)
		else:
			_set_camera_mode(CamMode.FOLLOW)
	elif _cam_mode == CamMode.FOLLOW:
		_set_camera_mode(CamMode.ENDOSCOPE)
	else:
		_set_orbit_preset(OrbitPreset.CLINICAL, true)


# Cycle to the next phantom model (M key): reapply its config, rebuild the
# vessel scene, and re-handshake the WebSocket session with the new phantom.
func _cycle_model() -> void:
	if _controls_blocked:
		return
	_model_index = (_model_index + 1) % MODELS.size()
	var cfg: Dictionary = MODELS[_model_index]
	_apply_model_config(cfg)
	print("[Main] switching model -> %s (phantom=%s, physics_engine=%s)" % [
		cfg.name, phantom, physics_engine])
	_branch_targets = []
	_branch_index = 0
	_logged_first_batch = false
	_path_waypoints = []
	if _rig != null and is_instance_valid(_rig):
		_rig.clear_navigation_route()
	_update_vessel_route_visibility([], false)
	_disengage_autopilot()
	_load_model_scene()
	_ws.restart_session(phantom, target, case_id, start_position, end_position, physics_engine)
	_session_id = "none"
	_msg_count = 0
	_last_msg = "model switch"
	_hud.set_model(str(cfg.name))
	_hud.set_view_mode(_view_mode_label())
	_hud.mark_navigation_stale("等待新模型数据")
	_update_debug()


func _set_camera_mode(mode: int) -> void:
	_cam_mode = mode
	match mode:
		CamMode.OVERVIEW:
			_camera.make_current()
		CamMode.FOLLOW:
			_rig.follow_cam.make_current()
		CamMode.ENDOSCOPE:
			_rig.endoscope_cam.make_current()
	# Whole-tree translucent in overview (演示视图), tip-proximity fade in follow
	# (手术视图), opaque inner wall in endoscope.
	_set_vessel_view_material(mode)
	# Depth fog on only in endoscope so the lumen reads with depth (near wall red,
	# far lumen fades dark) without darkening the overview/follow scene.
	if _env:
		_env.fog_enabled = (mode == CamMode.ENDOSCOPE)
	# Thin guidewire in the close-up views; thicker in the wide overview so it
	# stays visible at low magnification. The route line likewise: visible white
	# line outside, hair-thin on-centerline guide in the endoscope.
	_guidewire.set_close_up(mode != CamMode.OVERVIEW)
	if _path != null and is_instance_valid(_path):
		_path.set_endoscope(mode == CamMode.ENDOSCOPE)
	# Keep the 跟随 toolbar toggle in sync (the C key also cycles into FOLLOW).
	if _follow_btn != null and is_instance_valid(_follow_btn):
		_follow_btn.set_pressed_no_signal(mode == CamMode.FOLLOW)
	_sync_entry_marker_visibility()
	_hud.set_view_mode(_view_mode_label())
	print("[Main] camera mode -> %s" % _view_mode_label())


func _set_orbit_preset(preset: int, reset_angle: bool) -> void:
	_orbit_preset = preset
	if _cam_mode != CamMode.OVERVIEW:
		_set_camera_mode(CamMode.OVERVIEW)
	if preset == OrbitPreset.TREE:
		_frame_camera(_last_aabb)
	else:
		_apply_clinical_orbit(reset_angle)
	_sync_entry_marker_visibility()
	_hud.set_view_mode(_view_mode_label())
	print("[Main] orbit preset -> %s" % _view_mode_label())


func _apply_clinical_orbit(reset_angle: bool) -> void:
	var radius := _last_aabb.size.length() * 0.5
	if _last_aabb.size == Vector3.ZERO:
		_orbit_pivot = _tip_world_last if _tip_world_known else Vector3.ZERO
		_orbit_min = 0.025
		_orbit_max = 4.0
		_orbit_dist = 0.18
		if reset_angle:
			_orbit_yaw = deg_to_rad(28.0)
			_orbit_pitch = deg_to_rad(34.0)
		_update_orbit_camera()
		return
	_orbit_pivot = _tip_world_last if _tip_world_known else (_last_aabb.position + _last_aabb.size * 0.5)
	_orbit_min = maxf(0.02, radius * 0.08)
	_orbit_max = maxf(0.5, radius * 4.0)
	_orbit_dist = clampf(radius * 0.38, _orbit_min, _orbit_max)
	if reset_angle:
		_orbit_yaw = deg_to_rad(28.0)
		_orbit_pitch = deg_to_rad(34.0)
	_update_orbit_camera()


func _view_mode_label() -> String:
	if _cam_mode == CamMode.OVERVIEW:
		return str(ORBIT_PRESET_NAMES.get(_orbit_preset, "Clinical Orbit"))
	return str(CAM_MODE_NAMES.get(_cam_mode, "?"))


func _sync_entry_marker_visibility() -> void:
	if _entry_marker == null or not is_instance_valid(_entry_marker):
		return
	if _entry_marker.has_method("set_landmarks_visible"):
		_entry_marker.set_landmarks_visible(_cam_mode == CamMode.OVERVIEW and _orbit_preset == OrbitPreset.TREE)


func _to_vec3(arr: Variant) -> Vector3:
	if typeof(arr) == TYPE_ARRAY and arr.size() >= 3:
		return Vector3(float(arr[0]), float(arr[1]), float(arr[2]))
	return Vector3.ZERO


# tip.quaternion follows the protocol order [x, y, z, w].
func _to_quat(arr: Variant) -> Quaternion:
	if typeof(arr) == TYPE_ARRAY and arr.size() >= 4:
		return Quaternion(float(arr[0]), float(arr[1]), float(arr[2]), float(arr[3]))
	return Quaternion.IDENTITY


func _scene_aabb(node: Node) -> AABB:
	var result := AABB()
	var initialized := false
	for mi in node.find_children("*", "MeshInstance3D", true, false):
		if not mi.visible:
			continue
		var aabb: AABB = mi.global_transform * mi.get_aabb()
		if not initialized:
			result = aabb
			initialized = true
		else:
			result = result.merge(aabb)
	return result


# Frame the vessel AABB by (re)initializing the orbit rig: pivot at the AABB
# centre, distance for ~70-80% occupancy (doc/11 §9 "不要让模型占满窗口"), yaw/pitch
# giving the 3/4 anatomical angle. Also the 复位 tool's target state.
func _frame_camera(aabb: AABB) -> void:
	_last_aabb = aabb
	if aabb.size == Vector3.ZERO:
		_orbit_pivot = Vector3.ZERO
		_orbit_yaw = 0.0
		_orbit_pitch = 0.0
		_orbit_dist = 2.0
		_orbit_min = 0.05
		_orbit_max = 20.0
		_update_orbit_camera()
		return
	var center := aabb.position + aabb.size * 0.5
	var radius := aabb.size.length() * 0.5
	var distance := radius / tan(deg_to_rad(_camera.fov * 0.5)) * 1.22
	var offset := Vector3(radius * 1.15, aabb.size.y * 0.58, distance)
	_orbit_pivot = center
	_orbit_dist = offset.length()
	_orbit_yaw = atan2(offset.x, offset.z)
	_orbit_pitch = asin(clampf(offset.y / _orbit_dist, -1.0, 1.0))
	_orbit_min = radius * 0.15
	_orbit_max = radius * 8.0
	_update_orbit_camera()


# Apply the spherical orbit state to the overview camera: yaw about world up,
# pitch (elevation, +up) about the local right, at _orbit_dist from the pivot.
func _update_orbit_camera() -> void:
	if _camera == null or not is_instance_valid(_camera):
		return
	var rot := Basis(Vector3.UP, _orbit_yaw) * Basis(Vector3.RIGHT, -_orbit_pitch)
	_camera.position = _orbit_pivot + rot * Vector3(0.0, 0.0, _orbit_dist)
	_camera.look_at(_orbit_pivot, Vector3.UP)


func _set_orbit_focus(pivot: Vector3, keep_camera_offset: bool) -> void:
	if _camera == null or not is_instance_valid(_camera):
		return
	_orbit_pivot = pivot
	if not keep_camera_offset:
		return
	var offset := _camera.global_position - pivot
	var radius := _last_aabb.size.length() * 0.5
	var fallback_dist := clampf(radius * 0.35 if radius > 0.0 else 0.18, _orbit_min, _orbit_max)
	if offset.length() < 1e-5:
		offset = Vector3(0.0, fallback_dist * 0.25, fallback_dist)
	_orbit_dist = clampf(offset.length(), _orbit_min, _orbit_max)
	_orbit_yaw = atan2(offset.x, offset.z)
	_orbit_pitch = asin(clampf(offset.y / _orbit_dist, -1.0, 1.0))


func _zoom_by(steps: float) -> void:
	if _cam_mode != CamMode.OVERVIEW:
		return
	_orbit_dist = clampf(_orbit_dist * pow(_WHEEL_STEP, steps), _orbit_min, _orbit_max)
	_update_orbit_camera()


# 复位 tool: back to the clinical close orbit around the guidewire tip.
func _reset_view() -> void:
	_camera_user_controlled = false
	_set_orbit_preset(OrbitPreset.CLINICAL, true)


# ── 3D-pane pointer gestures (from input_handler's raw pointer stream) ────────
# A left press inside the pane starts a pending gesture; small total travel on
# release = click-to-navigate, larger travel = the selected 旋转/平移 operation.
# Presses on UI buttons or outside the pane are ignored entirely.
func _on_pointer_down(pos: Vector2) -> void:
	_press_in_pane = false
	_press_travel = 0.0
	if get_viewport().gui_get_hovered_control() is BaseButton:
		return
	# 跟随 mode: clicking any non-button area exits back to the free overview
	# camera and keeps the same press alive, so a drag immediately free-looks.
	if _cam_mode == CamMode.FOLLOW:
		_exit_follow_to_free()
	if _pane_3d_container == null or not _pane_3d_container.get_global_rect().has_point(pos):
		return
	_press_in_pane = true


func _on_pointer_drag(_pos: Vector2, relative: Vector2) -> void:
	if not _press_in_pane:
		return
	_press_travel += relative.length()
	# Below the click threshold nothing moves yet, so a jittery click never nudges
	# the camera; past it the gesture is committed to the selected view tool.
	if _press_travel <= _CLICK_TRAVEL_MAX:
		return
	if _cam_mode != CamMode.OVERVIEW:
		return
	match _view_tool_mode:
		ViewToolMode.ORBIT:
			_camera_user_controlled = true
			# Rotate around the current pivot. Reset/follow/presets may explicitly
			# choose the tip again, but a user-panned focus must survive this drag.
			_orbit_yaw -= relative.x * _ORBIT_SPEED
			_orbit_pitch = clampf(_orbit_pitch + relative.y * _ORBIT_SPEED, -1.45, 1.45)
			_update_orbit_camera()
		ViewToolMode.PAN:
			_pan_orbit(relative)


func _on_pointer_up(pos: Vector2) -> void:
	if _press_in_pane and _press_travel <= _CLICK_TRAVEL_MAX:
		_on_navigate_click(pos)
	_press_in_pane = false


func _on_pan_drag(pos: Vector2, relative: Vector2) -> void:
	if _cam_mode != CamMode.OVERVIEW or _pane_3d_container == null:
		return
	if not _pane_3d_container.get_global_rect().has_point(pos):
		return
	_pan_orbit(relative)


func _pan_orbit(relative: Vector2) -> void:
	if _camera == null or not is_instance_valid(_camera):
		return
	_camera_user_controlled = true
	var b := _camera.global_transform.basis
	_orbit_pivot += (-b.x * relative.x + b.y * relative.y) * _orbit_dist * _PAN_SPEED
	_update_orbit_camera()


func _on_wheel_zoom(steps: int, pos: Vector2) -> void:
	if _pane_3d_container == null or not _pane_3d_container.get_global_rect().has_point(pos):
		return
	_camera_user_controlled = true
	_zoom_by(float(steps))
