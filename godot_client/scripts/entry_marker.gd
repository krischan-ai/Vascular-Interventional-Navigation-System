extends Node3D
## Highlights the guidewire entry (vascular access) and the navigation target.
##
## In an interventional procedure the guidewire is introduced into the vessel at
## a defined access point and advanced along the centerline toward the target.
## This node draws that access point as a bright green emissive marker with a
## short arrow along the introduction direction, and the target as a red marker,
## so the operator can immediately see where the guidewire should enter and where
## it is headed.
##
## entry / target arrive in the MuJoCo/guidewire meter frame (the same frame as
## the vessel GLB and the guidewire), so this node is parented under the vessel
## frame and needs no coordinate conversion. The backend sends `entry` only on
## the first batch / after a reset (it is constant for a session); the marker is
## cached, so an empty `entry` on later frames keeps the existing drawing.

@export var entry_color: Color = Color(0.15, 1.0, 0.4)   ## green = access / go
@export var target_color: Color = Color(1.0, 0.2, 0.2)   ## red = target
@export var entry_radius: float = 0.006                  ## meters
@export var target_radius: float = 0.006                 ## meters
@export var arrow_length: float = 0.02                   ## meters

var _entry: MeshInstance3D
var _target: MeshInstance3D
var _arrow: MeshInstance3D
var _arrow_mesh: ImmediateMesh
var _arrow_material: StandardMaterial3D
var _entry_set: bool = false


func _ready() -> void:
	_entry = _make_sphere(entry_radius, entry_color)
	_entry.visible = false
	add_child(_entry)

	_target = _make_sphere(target_radius, target_color)
	_target.visible = false
	add_child(_target)

	_arrow_mesh = ImmediateMesh.new()
	_arrow = MeshInstance3D.new()
	_arrow.mesh = _arrow_mesh
	_arrow_material = StandardMaterial3D.new()
	_arrow_material.albedo_color = entry_color
	_arrow_material.shading_mode = BaseMaterial3D.SHADING_MODE_UNSHADED
	_arrow.material_override = _arrow_material
	_arrow.visible = false
	add_child(_arrow)


func update_from_batch(batch: Dictionary) -> void:
	var entry: Dictionary = batch.get("entry", {})
	if not entry.is_empty():
		_set_entry(_to_vec3(entry.get("position", [])), _to_vec3(entry.get("direction", [])))

	var target: Variant = batch.get("target", [])
	if typeof(target) == TYPE_ARRAY and target.size() >= 3:
		_target.position = _to_vec3(target)
		_target.visible = true


func _set_entry(pos: Vector3, dir: Vector3) -> void:
	# Ignore an all-zero position (no entry configured / not yet resolved).
	if pos == Vector3.ZERO:
		return
	_entry_set = true
	_entry.position = pos
	_entry.visible = true
	_draw_arrow(pos, dir)


func _draw_arrow(pos: Vector3, dir: Vector3) -> void:
	_arrow_mesh.clear_surfaces()
	if dir.length() < 1e-6:
		_arrow.visible = false
		return
	var d := dir.normalized()
	var tip := pos + d * arrow_length
	# A back-swept ortho axis for the two arrowhead barbs.
	var ortho := d.cross(Vector3.UP)
	if ortho.length() < 1e-4:
		ortho = d.cross(Vector3.RIGHT)
	ortho = ortho.normalized() * (arrow_length * 0.25)
	var back := tip - d * (arrow_length * 0.3)

	_arrow_mesh.surface_begin(Mesh.PRIMITIVE_LINES, _arrow_material)
	_arrow_mesh.surface_add_vertex(pos)
	_arrow_mesh.surface_add_vertex(tip)
	_arrow_mesh.surface_add_vertex(tip)
	_arrow_mesh.surface_add_vertex(back + ortho)
	_arrow_mesh.surface_add_vertex(tip)
	_arrow_mesh.surface_add_vertex(back - ortho)
	_arrow_mesh.surface_end()
	_arrow.visible = true


func _make_sphere(radius: float, color: Color) -> MeshInstance3D:
	var mi := MeshInstance3D.new()
	var sphere := SphereMesh.new()
	sphere.radius = radius
	sphere.height = radius * 2.0
	mi.mesh = sphere
	var mat := StandardMaterial3D.new()
	mat.albedo_color = color
	mat.emission_enabled = true
	mat.emission = color
	mat.emission_energy_multiplier = 0.8
	mi.material_override = mat
	return mi


func _to_vec3(arr: Variant) -> Vector3:
	if typeof(arr) == TYPE_ARRAY and arr.size() >= 3:
		return Vector3(float(arr[0]), float(arr[1]), float(arr[2]))
	return Vector3.ZERO
