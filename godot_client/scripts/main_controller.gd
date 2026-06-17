extends Node3D
## Root controller for the CathSim VPP client.
##
## Builds the 3D scene in code (environment, light, camera, vessel mesh,
## guidewire renderer, HUD) and wires the WebSocket client and input handler
## together. Kept code-driven so the .tscn stays minimal and robust.

const VESSEL_GLB := "res://assets/models/blood_vessels.glb"

var _ws  # WebSocketClient node
var _input  # InputHandler node
var _hud  # HUD CanvasLayer
var _guidewire  # GuidewireRenderer node
var _camera: Camera3D


func _ready() -> void:
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
		_frame_camera(_scene_aabb(vessel))


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
	_camera.current = true
	add_child(_camera)


func _setup_vessel() -> Node3D:
	if not ResourceLoader.exists(VESSEL_GLB):
		push_warning("Vessel GLB not found at %s. Run tools/export_godot_assets.py." % VESSEL_GLB)
		return null
	var packed: PackedScene = load(VESSEL_GLB)
	var vessel: Node3D = packed.instantiate()
	add_child(vessel)
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

	_ws.connected.connect(func(): _hud.set_connection(true))
	_ws.disconnected.connect(func(): _hud.set_connection(false))
	_ws.batch_received.connect(_on_batch)
	_ws.state_received.connect(_on_state)
	_input.control.connect(_ws.send_control)
	_input.reset_requested.connect(_ws.send_reset)


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


func _on_state(state: Dictionary) -> void:
	_guidewire.update_from_state(state)
	_hud.update_safety(str(state.get("safety_status", "STANDBY")))
	_hud.update_metrics(state)


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
