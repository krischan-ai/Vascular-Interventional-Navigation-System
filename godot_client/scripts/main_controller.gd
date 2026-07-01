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
var _guidewire  # GuidewireRenderer node
var _path  # PathRenderer node
var _entry_marker  # EntryMarker node (vessel entry + target highlights)
var _rig  # CameraRig node (follow + endoscope cameras)
var _camera: Camera3D  # overview camera
var _cam_mode: int = CamMode.OVERVIEW
var _vessel_meshes: Array = []  # MeshInstance3D nodes of the vessel
var _vessel_mat_overlay: StandardMaterial3D  # translucent (overview/follow)
var _vessel_mat_interior: StandardMaterial3D  # opaque inner wall (endoscope)
var _env: Environment  # world environment; endoscope toggles its depth fog
var _vessel: Node3D  # current vessel scene root (freed/rebuilt on model switch)
var _model_index: int = 0  # index into MODELS of the active phantom
# Auto-switch to the close-up follow view once the first tip pose streams in, so
# the guidewire is visible immediately instead of a dot in the overview.
var _auto_followed: bool = false

# On-screen diagnostics.
var _session_id: String = "none"
var _msg_count: int = 0
var _last_msg: String = "—"


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
	# guidewire/path positions). Falls back to this node when the GLB is missing.
	var frame: Node = vessel if vessel != null else self
	_setup_guidewire(frame)
	_setup_path(frame)
	_setup_entry_marker(frame)
	_setup_rig(frame)
	# Reset view state so the new model starts in overview (translucent wall, no
	# fog) and auto-follows again once its first tip pose streams in.
	_auto_followed = false
	_cam_mode = CamMode.OVERVIEW
	_set_vessel_interior(false)
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


func _setup_environment() -> void:
	var world_env := WorldEnvironment.new()
	var env := Environment.new()
	env.background_mode = Environment.BG_COLOR
	env.background_color = Color(0.05, 0.06, 0.09)
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
	add_child(world_env)


func _setup_camera_and_light() -> void:
	var light := DirectionalLight3D.new()
	light.rotation_degrees = Vector3(-50, -30, 0)
	light.light_energy = 1.1
	add_child(light)

	_camera = Camera3D.new()
	_camera.near = 0.001
	_camera.far = 50.0
	add_child(_camera)
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
	add_child(vessel)
	var mesh_count := vessel.find_children("*", "MeshInstance3D", true, false).size()
	print("[Main] vessel loaded, MeshInstance3D count=%d" % mesh_count)
	_apply_vessel_material(vessel)
	return vessel


func _apply_vessel_material(node: Node) -> void:
	# Overlay material (overview/follow): translucent so the guidewire shows
	# through from outside.
	_vessel_mat_overlay = StandardMaterial3D.new()
	_vessel_mat_overlay.albedo_color = Color(0.8, 0.3, 0.3, 0.28)
	_vessel_mat_overlay.transparency = BaseMaterial3D.TRANSPARENCY_ALPHA
	_vessel_mat_overlay.cull_mode = BaseMaterial3D.CULL_DISABLED

	# Interior material (endoscope): opaque so the lumen wall is visible from
	# inside instead of see-through. Double-sided so Godot flips back-face
	# normals for correct shading. A flat emission keeps every wall an even red
	# (it cannot clip to white like a close light does); the endoscope's depth
	# fog supplies the near/far gradient so the tunnel does not read as one solid
	# field.
	_vessel_mat_interior = StandardMaterial3D.new()
	_vessel_mat_interior.albedo_color = Color(0.7, 0.28, 0.26)
	_vessel_mat_interior.cull_mode = BaseMaterial3D.CULL_DISABLED
	_vessel_mat_interior.roughness = 0.85
	_vessel_mat_interior.metallic = 0.0
	_vessel_mat_interior.emission_enabled = true
	_vessel_mat_interior.emission = Color(0.45, 0.16, 0.15)
	_vessel_mat_interior.emission_energy_multiplier = 0.3

	_vessel_meshes = node.find_children("*", "MeshInstance3D", true, false)
	_set_vessel_interior(false)


func _set_vessel_interior(interior: bool) -> void:
	# Swap the vessel material so the lumen is opaque in endoscope view but
	# translucent (guidewire visible) in overview/follow.
	var mat: StandardMaterial3D = _vessel_mat_interior if interior else _vessel_mat_overlay
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
	_ws.batch_received.connect(_on_batch)
	_ws.state_received.connect(_on_state)
	_input.control.connect(_ws.send_control)
	_input.input_state.connect(_hud.update_input)
	_input.reset_requested.connect(_ws.send_reset)
	_input.view_cycle.connect(_cycle_camera_mode)
	_input.model_cycle.connect(_cycle_model)


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


func _on_batch(batch: Dictionary) -> void:
	_guidewire.update_from_batch(batch)
	_path.update_from_batch(batch)
	_entry_marker.update_from_batch(batch)
	_feed_rig(batch.get("tip", {}))
	var safety: Dictionary = batch.get("safety", {})
	var episode: Dictionary = batch.get("episode", {})
	_hud.update_safety(str(safety.get("status", "STANDBY")))
	_hud.update_metrics({
		"episode_length": episode.get("length", 0),
		"velocity": safety.get("speed", 0.0),
		"wall_distance": safety.get("wall_distance", 0.0),
		"curvature": safety.get("curvature", 0.0),
		"path_progress": batch.get("path", {}).get("progress", 0.0),
		"risk_score": safety.get("risk_score", 0.0),
	})
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


func _feed_rig(tip: Dictionary) -> void:
	if tip.is_empty():
		return
	var pos := _to_vec3(tip.get("position", []))
	var dir := _to_vec3(tip.get("direction", []))
	var quat := _to_quat(tip.get("quaternion", []))
	_rig.update_tip(pos, dir, quat)
	# First real pose: drop the operator into the close-up follow view.
	if not _auto_followed:
		_auto_followed = true
		_set_camera_mode(CamMode.FOLLOW)


func _cycle_camera_mode() -> void:
	_set_camera_mode((_cam_mode + 1) % CamMode.size())


# Cycle to the next phantom model (M key): reapply its config, rebuild the
# vessel scene, and re-handshake the WebSocket session with the new phantom.
func _cycle_model() -> void:
	_model_index = (_model_index + 1) % MODELS.size()
	var cfg: Dictionary = MODELS[_model_index]
	_apply_model_config(cfg)
	print("[Main] switching model -> %s (phantom=%s)" % [cfg.name, phantom])
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
	# Opaque inner wall only in endoscope; translucent otherwise so the
	# guidewire stays visible from outside.
	_set_vessel_interior(mode == CamMode.ENDOSCOPE)
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
	var distance := radius / tan(deg_to_rad(_camera.fov * 0.5))
	_camera.position = center + Vector3(0.0, aabb.size.y, distance)
	_camera.look_at(center, Vector3.UP)
