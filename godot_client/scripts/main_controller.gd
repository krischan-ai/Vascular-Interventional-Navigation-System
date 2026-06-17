extends Node3D
## Root controller for the CathSim VPP client.
##
## Builds the 3D scene in code (environment, light, camera, vessel mesh,
## guidewire renderer, HUD) and wires the WebSocket client and input handler
## together. Kept code-driven so the .tscn stays minimal and robust.

# Render the mesh of the phantom actually being simulated so the vessel and the
# streamed guidewire share one coordinate frame. The client defaults to the
# low_tort phantom (native scale, at origin), exported via
# `python tools/export_godot_assets.py --phantom low_tort`.
const VESSEL_GLB := "res://assets/models/low_tort.glb"

var _ws  # WebSocketClient node
var _input  # InputHandler node
var _hud  # HUD CanvasLayer
var _guidewire  # GuidewireRenderer node
var _camera: Camera3D

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
	# Parent the guidewire under the vessel scene root so both share the same
	# coordinate space (the glTF/trimesh axis-conversion transform applies
	# equally to the mesh and the streamed guidewire positions). Falls back to
	# this node when the vessel GLB is missing.
	_setup_guidewire(vessel if vessel != null else self)
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


func _setup_vessel() -> Node3D:
	if not ResourceLoader.exists(VESSEL_GLB):
		push_warning("Vessel GLB not found at %s. Run tools/export_godot_assets.py." % VESSEL_GLB)
		return null
	var packed: PackedScene = load(VESSEL_GLB)
	if packed == null:
		push_warning("Failed to load vessel GLB (import may have failed): %s" % VESSEL_GLB)
		return null
	var vessel: Node3D = packed.instantiate()
	add_child(vessel)
	var mesh_count := vessel.find_children("*", "MeshInstance3D", true, false).size()
	print("[Main] vessel loaded, MeshInstance3D count=%d" % mesh_count)
	_apply_vessel_material(vessel)
	return vessel


func _apply_vessel_material(node: Node) -> void:
	# Make the vessel translucent so the guidewire is visible inside.
	var mat := StandardMaterial3D.new()
	mat.albedo_color = Color(0.8, 0.3, 0.3, 0.28)
	mat.transparency = BaseMaterial3D.TRANSPARENCY_ALPHA
	mat.cull_mode = BaseMaterial3D.CULL_DISABLED
	for child in node.find_children("*", "MeshInstance3D", true, false):
		child.material_override = mat


func _setup_guidewire(parent: Node) -> void:
	_guidewire = preload("res://scripts/guidewire_renderer.gd").new()
	parent.add_child(_guidewire)


func _setup_hud() -> void:
	_hud = preload("res://scripts/hud_controller.gd").new()
	add_child(_hud)


func _setup_network_and_input() -> void:
	_ws = preload("res://scripts/websocket_client.gd").new()
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
