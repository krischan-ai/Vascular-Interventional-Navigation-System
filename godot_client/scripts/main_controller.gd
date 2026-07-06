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
@export var phantom: String = "aorta_tree"
@export var target: String = "root"
@export var case_id: String = "case_001"
## LPS millimeters; leave empty for non-VPP (low_tort / segment_part) sessions.
@export var start_position: Array = []
@export var end_position: Array = []

# Surgical-view wall fade (meters). In the follow ("手术视图") view the vessel wall
# is fully opaque within `surgical_fade_near` of the guidewire tip and fully
# transparent beyond `surgical_fade_far`, so only the local lumen segment shows
# instead of many overlapping translucent walls糊成一片 (doc/08 §9.3 P1 可视性修复).
# The overview stays the whole-tree translucent 演示视图.
@export var surgical_fade_near: float = 0.03
@export var surgical_fade_far: float = 0.11

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
		"name": "项目初始模型 Low-Tort",
		"phantom": "low_tort",
		"target": "bca",
		"case_id": "case_001",
		"start": [],
		"end": [],
	},
	{
		"name": "主动脉干 Aorta-Trunk",
		"phantom": "aorta_trunk",
		"target": "root",
		"case_id": "case_001",
		"start": [],
		"end": [],
	},
	{
		"name": "主动脉树 Aorta-Tree",
		"phantom": "aorta_tree",
		"target": "root",
		"case_id": "case_001",
		"start": [],
		"end": [],
	},
	{
		"name": "全身体膜 Segment-Part",
		"phantom": "segment_part",
		"target": "root",
		"case_id": "case_001",
		"start": [],
		"end": [],
	},
	{
		"name": "局部血管空腔 VPP",
		"phantom": "case_001_vpp",
		"target": "endpoints_1",
		"case_id": "case_001",
		"start": [0.173, -268.24, 291.25],
		"end": [-975.65, -217.22, 250.32],
	},
]

enum CamMode { OVERVIEW, FOLLOW, ENDOSCOPE }
const CAM_MODE_NAMES := {
	CamMode.OVERVIEW: "概览 Overview",
	CamMode.FOLLOW: "跟随 Follow",
	CamMode.ENDOSCOPE: "内窥镜 Endoscope",
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
var _cam_mode: int = CamMode.OVERVIEW
var _vessel_meshes: Array = []  # MeshInstance3D nodes of the vessel
var _vessel_mat_overlay: ShaderMaterial  # cyan fresnel-glow whole-tree (overview/demo)
var _vessel_mat_interior: StandardMaterial3D  # opaque inner wall (endoscope)
var _vessel_mat_surgical: ShaderMaterial  # cyan fresnel + tip-proximity fade (follow/surgical)
var _env: Environment  # world environment; endoscope toggles its depth fog
var _vessel: Node3D  # current vessel scene root (freed/rebuilt on model switch)
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


func _resolve_model_index(phantom_name: String) -> int:
	for i in MODELS.size():
		if str(MODELS[i].phantom) == phantom_name:
			return i
	return 0


func _apply_model_config(cfg: Dictionary) -> void:
	phantom = str(cfg.phantom)
	target = str(cfg.target)
	case_id = str(cfg.case_id)
	start_position = (cfg.start as Array).duplicate()
	end_position = (cfg.end as Array).duplicate()


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
	_set_vessel_view_material(CamMode.OVERVIEW)
	if _env:
		_env.fog_enabled = false
	if vessel != null:
		var aabb := _scene_aabb(vessel)
		print("[Main] vessel AABB pos=%s size=%s" % [aabb.position, aabb.size])
		_frame_camera(aabb)
	else:
		# No vessel mesh: still place the camera somewhere sane so the HUD and
		# guidewire tip are visible.
		_camera.position = Vector3(0, 0, 1.5)
		_camera.look_at(Vector3.ZERO, Vector3.UP)
	_camera.make_current()


func _teardown_model_scene() -> void:
	# Free the per-model renderers first (they may be children of the vessel),
	# then the vessel itself. Guarded so an initial build (nothing yet) is a no-op.
	for node in [_guidewire, _path, _entry_marker, _rig]:
		if node != null and is_instance_valid(node):
			node.queue_free()
	_guidewire = null
	_path = null
	_entry_marker = null
	_rig = null
	if _vessel != null and is_instance_valid(_vessel):
		_vessel.queue_free()
	_vessel = null
	_vessel_meshes = []


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


# DSA main view (doc/11 §5-§8, left 59%). Real X-ray fluoroscopy is a separate track,
# so the "image" is a grayscale noise placeholder; on top of it sit the medical
# overlays required by §6 (DSAOverlay), the info float, tool strip and legend box.
func _build_dsa_pane(rootc: Control) -> void:
	var dsa := PanelContainer.new()
	dsa.add_theme_stylebox_override("panel", UiStyle.panel_box(0.95, 8))
	UiStyle.place(dsa, UiStyle.dsa_rect())
	rootc.add_child(dsa)

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

	_corner_label(ov, "● DSA 实时影像", UiStyle.GREEN, Vector2(14, 10), 16)

	# 影像参数浮窗 (§5: 底 #101A26 75%, 圆角 6, 文字 #B8C7D6 14px).
	var info := PanelContainer.new()
	info.add_theme_stylebox_override("panel", UiStyle.card_box(0.75, 6))
	info.position = Vector2(14, 42)
	info.add_child(UiStyle.label("LAO: 12°\nCRA: 2°\nZoom: 100%", UiStyle.TEXT_MID, 14))
	ov.add_child(info)

	# 左侧悬浮工具栏 (§7: 48x48).
	var tools := VBoxContainer.new()
	tools.add_theme_constant_override("separation", 8)
	tools.position = Vector2(14, 150)
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
	legend_panel.offset_top = -168
	legend_panel.offset_right = -14
	legend_panel.offset_bottom = -14
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

	_corner_label(frame, "3D 血管导航", UiStyle.TEXT, Vector2(12, 8), 16)

	# 左侧竖向工具栏 (§10: 36x36, 选择/旋转/平移/放大/缩小).
	var tools := VBoxContainer.new()
	tools.add_theme_constant_override("separation", 6)
	tools.position = Vector2(10, 42)
	for t in ["选择", "旋转", "平移", "放大", "缩小"]:
		tools.add_child(_pane_tool(t, Vector2(36, 36)))
	frame.add_child(tools)

	# 方向立方体 (§11: 70x70, #2F8CFF 70%, 线框风, S/I/L/R/A).
	var cube := Panel.new()
	var cube_sb := UiStyle.flat(Color(UiStyle.BLUE.r, UiStyle.BLUE.g, UiStyle.BLUE.b, 0.10), 6)
	cube_sb.border_color = Color(UiStyle.BLUE.r, UiStyle.BLUE.g, UiStyle.BLUE.b, 0.7)
	cube_sb.set_border_width_all(1)
	cube.add_theme_stylebox_override("panel", cube_sb)
	cube.anchor_left = 1.0
	cube.anchor_right = 1.0
	cube.offset_left = -82
	cube.offset_top = 12
	cube.offset_right = -12
	cube.offset_bottom = 82
	var cube_lbl := UiStyle.label("S\nR A L\nI", Color(UiStyle.BLUE.r, UiStyle.BLUE.g, UiStyle.BLUE.b, 0.9), 12)
	cube_lbl.set_anchors_and_offsets_preset(Control.PRESET_FULL_RECT)
	cube_lbl.horizontal_alignment = HORIZONTAL_ALIGNMENT_CENTER
	cube_lbl.vertical_alignment = VERTICAL_ALIGNMENT_CENTER
	cube.add_child(cube_lbl)
	frame.add_child(cube)


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
	env.ambient_light_color = Color(0.4, 0.4, 0.45)
	env.ambient_light_energy = 0.6
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
	_world.add_child(_camera)
	# Activate only after the camera is in the scene tree, otherwise it may not
	# become the active viewport camera.
	_camera.make_current()


func _resolve_vessel_glb() -> String:
	# Prefer the phantom-named GLB (e.g. segment_part.glb); all VPP cases share the
	# blood_vessels export. Do NOT silently substitute a different anatomy: the
	# guidewire/path stream in this phantom's frame, so drawing another vessel
	# leaves the wire floating far from it (e.g. segment_part is ~0.8 m off-origin).
	# When the named GLB is missing (not yet imported in the Godot editor),
	# _setup_vessel warns and renders no vessel rather than the wrong one.
	if phantom.ends_with("_vpp"):
		return "res://assets/models/blood_vessels.glb"
	return "res://assets/models/%s.glb" % phantom


func _setup_vessel() -> Node3D:
	var glb := _resolve_vessel_glb()
	if not ResourceLoader.exists(glb):
		push_warning("Vessel GLB not found at %s. Run tools/export_godot_assets.py." % glb)
		return null
	var packed: PackedScene = load(glb)
	if packed == null:
		push_warning("Failed to load vessel GLB (import may have failed): %s" % glb)
		return null
	var vessel: Node3D = packed.instantiate()
	_world.add_child(vessel)
	var mesh_count := vessel.find_children("*", "MeshInstance3D", true, false).size()
	print("[Main] vessel loaded, MeshInstance3D count=%d" % mesh_count)
	_apply_vessel_material(vessel)
	return vessel


func _apply_vessel_material(node: Node) -> void:
	# Overview/demo material: a Fresnel edge shader in the doc/11 §9 palette (棕红/暗红/
	# 半透明粉 anatomical vessel, replacing the earlier cyan glow). The fresnel still
	# fixes the α-stacking haze at the root: the lumen CENTRE is near-transparent (so
	# overlapping walls no longer accumulate) while the silhouette EDGE is a defined
	# pink-red rim — the tube boundary defines the structure instead of alpha stacking.
	# Unshaded so it cannot clip to white from any camera angle.
	_vessel_mat_overlay = _make_fresnel_material(false)

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

	# Surgical (follow) material: the same Fresnel glow, additionally faded by
	# distance from the guidewire tip. Fragments within `fade_near` of the tip keep
	# full glow; past `fade_far` they vanish, so the follow view shows only the local
	# glowing lumen segment and the distant tree no longer piles up. `tip_world_pos`
	# is refreshed each frame in _feed_rig.
	_vessel_mat_surgical = _make_fresnel_material(true)
	_vessel_mat_surgical.set_shader_parameter("fade_near", surgical_fade_near)
	_vessel_mat_surgical.set_shader_parameter("fade_far", surgical_fade_far)
	_vessel_mat_surgical.set_shader_parameter("tip_world_pos", Vector3.ZERO)

	_vessel_meshes = node.find_children("*", "MeshInstance3D", true, false)
	_set_vessel_view_material(CamMode.OVERVIEW)


# Build the青蓝菲涅尔发光 vessel shader.
#
# Overview/demo (with_fade=false): FRESNEL drives alpha — the silhouette edge is
# opaque cyan and the lumen centre near-transparent, so the vessel tree reads as a
# glowing glass structure viewed from OUTSIDE (设计图右上角). No red-haze α-stacking
# because the centres don't accumulate.
#
# Follow/surgical (with_fade=true): the camera sits INSIDE / on the wire, so almost
# every wall is seen at a grazing angle where fresnel saturates → the whole view
# would go solid cyan. So here alpha is a CAPPED constant translucency multiplied by
# a tip-proximity fade (near the tip stays visible, distant walls vanish), and
# fresnel is used only as an emission edge accent — never to drive opacity. This
# keeps the local lumen see-through to the guidewire instead of a solid blob.
func _make_fresnel_material(with_fade: bool) -> ShaderMaterial:
	var fade_decl := ""
	var alpha_line: String
	if with_fade:
		fade_decl = "uniform vec3 tip_world_pos;\n" \
			+ "uniform float fade_near = 0.03;\n" \
			+ "uniform float fade_far = 0.11;\n" \
			+ "uniform float base_alpha = 0.30;\n"
		alpha_line = "	float prox = 1.0 - smoothstep(fade_near, fade_far, distance(tip_world_pos, world_pos));\n" \
			+ "	ALPHA = (base_alpha + f * 0.3) * prox;\n"
	else:
		alpha_line = "	ALPHA = clamp(core_alpha + f * 0.9, 0.0, 1.0);\n"
	var shader := Shader.new()
	shader.code = "shader_type spatial;\n" \
		+ "render_mode cull_disabled, unshaded, depth_draw_never, blend_mix;\n" \
		+ "uniform vec3 rim_color : source_color = vec3(0.85, 0.42, 0.40);\n" \
		+ "uniform vec3 core_color : source_color = vec3(0.38, 0.11, 0.11);\n" \
		+ "uniform float rim_power = 2.2;\n" \
		+ "uniform float core_alpha = 0.14;\n" \
		+ "uniform float glow = 0.9;\n" \
		+ fade_decl \
		+ "varying vec3 world_pos;\n" \
		+ "void vertex() { world_pos = (MODEL_MATRIX * vec4(VERTEX, 1.0)).xyz; }\n" \
		+ "void fragment() {\n" \
		+ "	float f = pow(1.0 - clamp(dot(normalize(NORMAL), normalize(VIEW)), 0.0, 1.0), rim_power);\n" \
		+ "	ALBEDO = mix(core_color, rim_color, f);\n" \
		+ "	EMISSION = rim_color * f * glow;\n" \
		+ alpha_line \
		+ "}\n"
	var mat := ShaderMaterial.new()
	mat.shader = shader
	return mat


# Select the vessel wall material for the active camera mode: whole-tree translucent
# in overview (演示视图), tip-proximity fade in follow (手术视图), opaque inner wall
# in endoscope.
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
	add_child(_ws)
	_input = preload("res://scripts/input_handler.gd").new()
	add_child(_input)

	_ws.connected.connect(_on_connected)
	_ws.disconnected.connect(_on_disconnected)
	_ws.error_received.connect(_on_server_error)
	_ws.session_started.connect(_on_session_started)
	_ws.routes_received.connect(_on_routes)
	_ws.batch_received.connect(_on_batch)
	_ws.state_received.connect(_on_state)

	# Interactive deformation panel: live-tune the Newton force-drive params. It's a
	# developer tool (not part of the VPP §2 operator dashboard), so it starts hidden
	# and is toggled by the 形变 G tool button / HUD deform_toggle signal.
	_deform = preload("res://scripts/deform_panel.gd").new()
	add_child(_deform)
	_deform.visible = false
	_deform.param_changed.connect(_ws.send_engine_params)
	_ws.engine_params_received.connect(_deform.sync_effective)

	_input.control.connect(_ws.send_control)
	# Grabbing manual control (any W/S/A/D tick) cancels click autopilot.
	_input.control.connect(_on_manual_control)
	_input.input_state.connect(_hud.update_input)
	_input.reset_requested.connect(_ws.send_reset)
	_input.view_cycle.connect(_cycle_camera_mode)
	_input.model_cycle.connect(_cycle_model)
	_input.branch_cycle.connect(_cycle_branch)
	_input.navigate_click.connect(_on_navigate_click)
	_input.autopilot_off.connect(_disengage_autopilot)
	_ws.shape_intent_received.connect(_on_shape_intent)

	# Dashboard control buttons (设计图 仪表盘): mouse equivalents of the safety
	# actions and view/model/branch/reset shortcuts, so the platform is operable
	# without the keyboard.
	_hud.emergency_stop.connect(_on_emergency_stop)
	_hud.manual_takeover.connect(_disengage_autopilot)
	_hud.resume_nav.connect(_on_resume_nav)
	_hud.motion_command.connect(_on_motion)
	_hud.view_cycle_requested.connect(_cycle_camera_mode)
	_hud.model_cycle_requested.connect(_cycle_model)
	_hud.branch_cycle_requested.connect(_cycle_branch)
	_hud.reset_requested.connect(_on_reset_button)
	_hud.deform_toggle.connect(_on_deform_toggle)


func _on_connected() -> void:
	print("[Main] WebSocket connected")
	_hud.set_connection(true)
	_last_msg = "connected"
	_update_debug()


func _on_disconnected() -> void:
	print("[Main] WebSocket disconnected")
	_hud.set_connection(false)
	_last_msg = "DISCONNECTED"
	_update_debug()


func _on_server_error(err: Dictionary) -> void:
	push_warning("[Main] server error: %s" % str(err))
	var code := str(err.get("code", "?"))
	var message := str(err.get("message", ""))
	_last_msg = "error %s: %s" % [code, message]
	_update_debug()


func _on_session_started(sid: String, state: Dictionary) -> void:
	print("[Main] session started: %s" % sid)
	_session_id = sid.substr(0, 8) if sid.length() >= 8 else sid
	_last_msg = "session_started"
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
	if _branch_targets.size() < 2:
		return
	_branch_index = (_branch_index + 1) % _branch_targets.size()
	var target := str(_branch_targets[_branch_index])
	_ws.send_select_route(target)
	_path_waypoints = []  # the new branch re-streams its route
	_disengage_autopilot()
	_auto_followed = false  # re-drop into follow view on the new branch's first pose
	_hud.set_model("%s  ·  分支 %d/%d %s" % [
		str(MODELS[_model_index].name), _branch_index + 1,
		_branch_targets.size(), target])
	print("[Main] select branch -> %s" % target)


func _on_batch(batch: Dictionary) -> void:
	if not _logged_first_batch:
		_logged_first_batch = true
		var path: Dictionary = batch.get("path", {})
		print("[Main] first state_batch waypoints=%d target=%s" % [
			(path.get("waypoints", []) as Array).size(),
			str(batch.get("target", [])),
		])
	# Capture the route waypoints (backend meter frame) for click-to-navigate;
	# they arrive only on the first batch / after a reset or route switch.
	var path_wps: Array = batch.get("path", {}).get("waypoints", [])
	if not path_wps.is_empty():
		_path_waypoints = path_wps
	_guidewire.update_from_batch(batch)
	_path.update_from_batch(batch)
	_entry_marker.update_from_batch(batch)
	_feed_rig(_camera_pose_from_batch(batch))
	var safety: Dictionary = batch.get("safety", {})
	var episode: Dictionary = batch.get("episode", {})
	var status := str(safety.get("status", "STANDBY"))
	_hud.update_safety(status)
	# Fixed control-mode display (VPP §2.1/§2.4): SAFE HOLD on a collision stop,
	# SUPERVISED AUTO while click-autopilot drives, otherwise MANUAL.
	var mode := "手动 MANUAL"
	if status == "COLLISION_STOP":
		mode = "安全保持 SAFE HOLD"
	elif _autopilot_active:
		mode = "自动 SUPERVISED AUTO"
	_hud.set_control_mode(mode)
	_hud.update_metrics({
		"episode_length": episode.get("length", 0),
		"velocity": safety.get("speed", 0.0),
		"wall_distance": safety.get("wall_distance", 0.0),
		"curvature": safety.get("curvature", 0.0),
		"path_progress": batch.get("path", {}).get("progress", 0.0),
		"risk_score": safety.get("risk_score", 0.0),
	})
	# While click autopilot is engaged, keep the nav line live with progress so the
	# slow tip advance is legible as motion, not a hang.
	if _autopilot_active:
		var prog := float(batch.get("path", {}).get("progress", 0.0)) * 100.0
		_hud.set_nav("自动 Auto → %s · %.0f%%" % [_autopilot_wp_text, prog], true)
	_msg_count += 1
	_last_msg = "state_batch"
	_update_debug()


func _on_state(state: Dictionary) -> void:
	_guidewire.update_from_state(state)
	_hud.update_safety(str(state.get("safety_status", "STANDBY")))
	_hud.update_metrics(state)
	_msg_count += 1
	_last_msg = "state_update"
	_update_debug()


# While click autopilot is engaged, tick the backend at ~20 Hz with a neutral
# control frame; the server overrides it with the ShapeIntentController output,
# so the tip keeps advancing toward the clicked waypoint without keyboard input.
func _process(delta: float) -> void:
	if not _autopilot_active:
		return
	_autopilot_accum += delta
	if _autopilot_accum < _AUTOPILOT_TICK:
		return
	_autopilot_accum = 0.0
	_ws.send_control(0.0, 0.0)


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
	if _path_waypoints.is_empty() or _path == null or not is_instance_valid(_path):
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


# 急停 button: halt immediately — drop autopilot and send a neutral control frame so
# the backend stops advancing the tip.
func _on_emergency_stop() -> void:
	_disengage_autopilot()
	if _ws != null and is_instance_valid(_ws):
		_ws.send_control(0.0, 0.0)
	_hud.set_nav("急停 STOP", false)
	print("[Main] EMERGENCY STOP")


# 恢复导航 button: re-engage the autopilot toward the last clicked goal (if any),
# after a 人工接管 / 急停 handed control back.
func _on_resume_nav() -> void:
	if _last_click_target.size() < 3 or _autopilot_active:
		return
	_autopilot_active = true
	_autopilot_accum = _AUTOPILOT_TICK
	_ws.send_shape_intent(true, _last_click_target)
	if _entry_marker != null and is_instance_valid(_entry_marker):
		_entry_marker.set_goal(Vector3(
			float(_last_click_target[0]), float(_last_click_target[1]), float(_last_click_target[2])))
	_hud.set_nav("自动 Auto → %s" % _autopilot_wp_text, true)
	print("[Main] resume nav -> %s" % str(_last_click_target))


# 重置 button: cancel autopilot and reset the episode.
func _on_reset_button() -> void:
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
	# Refresh the surgical-view fade origin: the tip in world space (the path node
	# shares the vessel frame, so its transform maps the frame-local tip to world,
	# matching how click-nav projects waypoints).
	if _vessel_mat_surgical != null and _path != null and is_instance_valid(_path):
		var tip_world: Vector3 = _path.global_transform * pos
		_vessel_mat_surgical.set_shader_parameter("tip_world_pos", tip_world)
		# §2.6 bottom status bar: tip world coordinate.
		_hud.set_coord(tip_world)
	# Keep the 3D assistant pane in the external OVERVIEW angle (VPP §2.3). An inside
	# follow/endoscope camera sees the lumen wall at grazing angles where the 半透 walls
	# stack into a solid cyan blob; OVERVIEW shows the whole vessel tree cleanly (the
	# fresnel glow reads as a glass tree). The operator can still press C for follow.
	if not _auto_followed:
		_auto_followed = true
		if _cam_mode != CamMode.OVERVIEW:
			_set_camera_mode(CamMode.OVERVIEW)


func _camera_pose_from_batch(batch: Dictionary) -> Dictionary:
	# Newton demo currently drives the proximal/root body; the distal red tip can
	# lag or buckle. Follow the active root end for debugging while keeping other
	# engines on the semantic tip pose.
	if str(batch.get("engine", "")) == "NewtonEngine":
		var bodies: Array = batch.get("bodies", [])
		if bodies.size() >= 2:
			var b0: Dictionary = bodies[0]
			var b1: Dictionary = bodies[1]
			if b0.has("pos") and b1.has("pos"):
				var p0 := _to_vec3(b0["pos"])
				var p1 := _to_vec3(b1["pos"])
				var dir := p1 - p0
				if dir.length() > 1e-6:
					return {
						"position": b0["pos"],
						"direction": [dir.x, dir.y, dir.z],
						"quaternion": b0.get("quat", []),
					}
	return batch.get("tip", {})


func _cycle_camera_mode() -> void:
	_set_camera_mode((_cam_mode + 1) % CamMode.size())


# Cycle to the next phantom model (M key): reapply its config, rebuild the
# vessel scene, and re-handshake the WebSocket session with the new phantom.
func _cycle_model() -> void:
	_model_index = (_model_index + 1) % MODELS.size()
	var cfg: Dictionary = MODELS[_model_index]
	_apply_model_config(cfg)
	print("[Main] switching model -> %s (phantom=%s)" % [cfg.name, phantom])
	_branch_targets = []
	_branch_index = 0
	_logged_first_batch = false
	_path_waypoints = []
	_disengage_autopilot()
	_load_model_scene()
	_ws.restart_session(phantom, target, case_id, start_position, end_position)
	_session_id = "none"
	_msg_count = 0
	_last_msg = "model switch"
	_hud.set_model(str(cfg.name))
	_hud.set_view_mode(CAM_MODE_NAMES.get(CamMode.OVERVIEW, "?"))
	_hud.update_safety("STANDBY")
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
	# stays visible at low magnification.
	_guidewire.set_close_up(mode != CamMode.OVERVIEW)
	_hud.set_view_mode(CAM_MODE_NAMES.get(mode, "?"))
	print("[Main] camera mode -> %s" % CAM_MODE_NAMES.get(mode, "?"))


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
		var aabb: AABB = mi.global_transform * mi.get_aabb()
		if not initialized:
			result = aabb
			initialized = true
		else:
			result = result.merge(aabb)
	return result


func _frame_camera(aabb: AABB) -> void:
	if aabb.size == Vector3.ZERO:
		_camera.position = Vector3(0, 0, 2)
		_camera.look_at(Vector3.ZERO, Vector3.UP)
		return
	var center := aabb.position + aabb.size * 0.5
	var radius := aabb.size.length() * 0.5
	# Pull back further than the exact fit so the model keeps ~70-80% area occupancy
	# with UI space around it (doc/11 §9 "不要让模型占满窗口"), and offset in x/y for a
	# 3/4 anatomical angle instead of a flat front-on shot.
	var distance := radius / tan(deg_to_rad(_camera.fov * 0.5)) * 1.35
	_camera.position = center + Vector3(radius * 0.7, aabb.size.y * 0.5, distance)
	_camera.look_at(center, Vector3.UP)
