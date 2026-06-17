extends Node3D
## Renders the guidewire: a tip sphere plus a tube/line through the per-segment
## body positions streamed in state_batch messages.
##
## All positions arrive in the MuJoCo/guidewire meter frame, which is the same
## frame as the vessel GLB exported by tools/export_godot_assets.py, so no
## coordinate conversion is required.

@export var tip_radius: float = 0.004      ## meters
@export var wire_color: Color = Color(0.9, 0.9, 0.95)
@export var tip_color: Color = Color(1.0, 0.35, 0.2)

var _tip: MeshInstance3D
var _wire: MeshInstance3D
var _wire_mesh: ImmediateMesh
var _wire_material: StandardMaterial3D


func _ready() -> void:
	_tip = MeshInstance3D.new()
	var sphere := SphereMesh.new()
	sphere.radius = tip_radius
	sphere.height = tip_radius * 2.0
	_tip.mesh = sphere
	var tip_mat := StandardMaterial3D.new()
	tip_mat.albedo_color = tip_color
	tip_mat.emission_enabled = true
	tip_mat.emission = tip_color
	tip_mat.emission_energy_multiplier = 0.6
	_tip.material_override = tip_mat
	add_child(_tip)

	_wire_mesh = ImmediateMesh.new()
	_wire = MeshInstance3D.new()
	_wire.mesh = _wire_mesh
	_wire_material = StandardMaterial3D.new()
	_wire_material.albedo_color = wire_color
	_wire_material.shading_mode = BaseMaterial3D.SHADING_MODE_UNSHADED
	_wire_material.vertex_color_use_as_albedo = false
	_wire.material_override = _wire_material
	add_child(_wire)


func update_from_batch(batch: Dictionary) -> void:
	var tip_data: Dictionary = batch.get("tip", {})
	if tip_data.has("position"):
		_tip.position = _to_vec3(tip_data["position"])

	var bodies: Array = batch.get("bodies", [])
	_draw_wire(bodies)


func update_from_state(state: Dictionary) -> void:
	if state.has("tip_position"):
		_tip.position = _to_vec3(state["tip_position"])


func _draw_wire(bodies: Array) -> void:
	_wire_mesh.clear_surfaces()
	if bodies.size() < 2:
		return
	_wire_mesh.surface_begin(Mesh.PRIMITIVE_LINE_STRIP, _wire_material)
	for body in bodies:
		if typeof(body) == TYPE_DICTIONARY and body.has("pos"):
			_wire_mesh.surface_add_vertex(_to_vec3(body["pos"]))
	_wire_mesh.surface_end()


func _to_vec3(arr: Variant) -> Vector3:
	if typeof(arr) == TYPE_ARRAY and arr.size() >= 3:
		return Vector3(float(arr[0]), float(arr[1]), float(arr[2]))
	return Vector3.ZERO
