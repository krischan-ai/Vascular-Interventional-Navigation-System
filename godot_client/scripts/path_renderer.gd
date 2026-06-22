extends Node3D
## Renders the planned navigation path (state_batch.path.waypoints) as a line
## strip, so the operator can see the intended centerline route through the
## vessel.
##
## Waypoints arrive in the MuJoCo/guidewire meter frame (the same frame as the
## vessel GLB and the guidewire), so no coordinate conversion is needed. The
## path is constant for a session, so it is only (re)drawn when the waypoint
## count changes (e.g. after a reset or a new route).

@export var path_color: Color = Color(0.2, 0.8, 1.0)

var _mesh: ImmediateMesh
var _mi: MeshInstance3D
var _material: StandardMaterial3D
var _drawn_count: int = -1


func _ready() -> void:
	_mesh = ImmediateMesh.new()
	_mi = MeshInstance3D.new()
	_mi.mesh = _mesh
	_material = StandardMaterial3D.new()
	_material.albedo_color = path_color
	_material.shading_mode = BaseMaterial3D.SHADING_MODE_UNSHADED
	_mi.material_override = _material
	add_child(_mi)


func update_from_batch(batch: Dictionary) -> void:
	var path: Dictionary = batch.get("path", {})
	var waypoints: Array = path.get("waypoints", [])
	# The backend sends the (constant) path only on the first batch / after reset
	# and omits it afterwards; keep the existing drawing when waypoints is empty.
	if waypoints.is_empty():
		return
	# Redraw only when the route changes to avoid rebuilding the mesh every frame.
	if waypoints.size() == _drawn_count:
		return
	_drawn_count = waypoints.size()
	_draw(waypoints)


func _draw(waypoints: Array) -> void:
	_mesh.clear_surfaces()
	if waypoints.size() < 2:
		return
	_mesh.surface_begin(Mesh.PRIMITIVE_LINE_STRIP, _material)
	for point in waypoints:
		if typeof(point) == TYPE_ARRAY and point.size() >= 3:
			_mesh.surface_add_vertex(Vector3(
				float(point[0]), float(point[1]), float(point[2])
			))
	_mesh.surface_end()
