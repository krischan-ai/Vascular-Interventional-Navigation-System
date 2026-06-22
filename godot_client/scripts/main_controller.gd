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
@export var phantom: String = "case_001_vpp"
@export var target: String = "endpoints_1"
@export var case_id: String = "case_001"
## LPS millimeters; leave empty for non-VPP (low_tort) sessions.
@export var start_position: Array = [0.173, -268.24, 291.25]
@export var end_position: Array = [-975.65, -217.22, 250.32]

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
	_setup_environment()
	_setup_camera_and_light()
	var vessel := _setup_vessel()
	# Parent the guidewire and planned-path renderers under the vessel scene root
	# so all three share the same coordinate space (the glTF/trimesh axis-
	# conversion transform applies equally to the mesh and the streamed
	# guidewire/path positions). Falls back to this node when the GLB is missing.
	var frame: Node = vessel if vessel != null else self
	_setup_guidewire(frame)
	_setup_path(frame)
	_setup_entry_marker(frame)
	_setup_rig(frame)
	_setup_hud()
	_setup_network_and_input()
	if vessel != null:
		var aabb := _scene_aabb(vessel)
		print("[Main] vessel AABB pos=%s size=%s" % [aabb.position, aabb.size])
		_frame_camera(aabb)
	else:
		# No vessel mesh: still place the camera somewhere sane so the HUD and
		# guidewire tip are visible.
		_camera.position = Vector3(0, 0, 1.5)
		_camera.look_at(Vector3.ZERO, Vector3.UP)
	print("[Main] camera pos=%s current=%s" % [_camera.position, _camera.current])


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
	# Prefer a phantom-named GLB (e.g. case_001_vpp.glb); fall back to the VPP
	# vessel export for any *_vpp phantom, else the low_tort phantom mesh.
	var direct := "res://assets/models/%s.glb" % phantom
	if ResourceLoader.exists(direct):
		return direct
	if phantom.ends_with("_vpp"):
		return "res://assets/models/blood_vessels.glb"
	return "res://assets/models/low_tort.glb"


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
