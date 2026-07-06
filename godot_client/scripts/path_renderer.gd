extends Node3D
## Renders the planned navigation path (state_batch.path.waypoints) as a thin
## tube, so the operator can see the intended centerline route through the vessel.
##
## A tube (not a 1px line strip) is used so the path stays visible in the
## first-person endoscope view: there the camera sits on the centerline and looks
## along it, so the near path runs almost parallel to the view. A zero-width line
## projects to nothing when seen end-on, leaving only the distant, cross-view part
## visible; a tube keeps a visible cross-section all the way from the camera out.
##
## Waypoints arrive in the MuJoCo/guidewire meter frame (the same frame as the
## vessel GLB and the guidewire), so no coordinate conversion is needed. The
## path is constant for a session, so it is only (re)drawn when the waypoint
## count changes (e.g. after a reset or a new route).

# 规划路径黄色 (doc/11 §9: 导管路径颜色 黄色; matches the DSA overlay's planned path).
@export var path_color: Color = Color(1.0, 0.831, 0.278)
# Hair-thin guide line. The endoscope camera sits on the centerline (guided mode),
# i.e. inside this tube, so the nearest ring wraps the view; its angular size is
# ~atan(radius / near_plane) with near_plane = 0.0005 m. The radius must be a
# small fraction of the near plane so that ring shrinks to a dot instead of a
# hexagon blocking the curve behind it, leaving a thin line receding down the
# lumen. More sides so the tiny cross-section reads round, not faceted.
@export var path_radius: float = 0.00004  ## meters
@export var path_sides: int = 8           ## tube cross-section segments
# Hide the tube right at the camera so the near cross-section (a faceted ring the
# endoscope camera sits inside) disappears, leaving the line from a few mm out.
# Distance fade is by camera distance, so the overview/follow cameras (always far
# from the path) keep it fully visible; only the on-centerline endoscope fades.
@export var near_hide_distance: float = 0.0015  ## m: invisible nearer than this
@export var near_show_distance: float = 0.004   ## m: fully visible beyond this

var _mesh: ImmediateMesh
var _mi: MeshInstance3D
var _material: StandardMaterial3D
var _drawn_count: int = -1
var _drawn_signature: String = ""


func _ready() -> void:
	_mesh = ImmediateMesh.new()
	_mi = MeshInstance3D.new()
	_mi.mesh = _mesh
	_material = StandardMaterial3D.new()
	_material.albedo_color = path_color
	_material.shading_mode = BaseMaterial3D.SHADING_MODE_UNSHADED
	# Double-sided so the guide stays visible when the endoscope camera is inside
	# or alongside the thin tube.
	_material.cull_mode = BaseMaterial3D.CULL_DISABLED
	# Fade out the tube within near_hide_distance of the camera (per-pixel) so the
	# near ring the endoscope sits inside vanishes, leaving the receding line.
	_material.distance_fade_mode = BaseMaterial3D.DISTANCE_FADE_PIXEL_ALPHA
	_material.distance_fade_min_distance = near_hide_distance
	_material.distance_fade_max_distance = near_show_distance
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
	# A branch switch can produce the same waypoint count, so include endpoints.
	var signature := _path_signature(waypoints)
	if waypoints.size() == _drawn_count and signature == _drawn_signature:
		return
	_drawn_count = waypoints.size()
	_drawn_signature = signature
	print("[Path] redraw waypoints=%d signature=%s" % [_drawn_count, _drawn_signature])
	_draw(waypoints)


func _draw(waypoints: Array) -> void:
	var points := PackedVector3Array()
	for point in waypoints:
		if typeof(point) == TYPE_ARRAY and point.size() >= 3:
			points.append(Vector3(float(point[0]), float(point[1]), float(point[2])))
	_build_tube(points, path_radius)


func _path_signature(waypoints: Array) -> String:
	if waypoints.is_empty():
		return ""
	return "%d:%s:%s" % [
		waypoints.size(),
		str(waypoints[0]),
		str(waypoints[waypoints.size() - 1]),
	]


func _build_tube(points: PackedVector3Array, radius: float) -> void:
	_mesh.clear_surfaces()
	var count := points.size()
	if count < 2:
		return

	var sides: int = max(3, path_sides)
	# Ring frames via parallel transport to avoid twisting along the curve.
	var rings: Array = []
	var prev_n := Vector3.ZERO
	for i in count:
		var tangent := _tangent_at(points, i)
		var n: Vector3
		if i == 0:
			n = _arbitrary_perp(tangent)
		else:
			n = prev_n - tangent * prev_n.dot(tangent)
			if n.length() < 1e-6:
				n = _arbitrary_perp(tangent)
			n = n.normalized()
		prev_n = n
		var b := tangent.cross(n).normalized()
		var ring := PackedVector3Array()
		for k in sides:
			var ang := TAU * float(k) / float(sides)
			var dir := (n * cos(ang) + b * sin(ang)).normalized()
			ring.append(points[i] + dir * radius)
		rings.append(ring)

	_mesh.surface_begin(Mesh.PRIMITIVE_TRIANGLES, _material)
	for i in count - 1:
		var r0: PackedVector3Array = rings[i]
		var r1: PackedVector3Array = rings[i + 1]
		for k in sides:
			var k2 := (k + 1) % sides
			_mesh.surface_add_vertex(r0[k])
			_mesh.surface_add_vertex(r1[k])
			_mesh.surface_add_vertex(r0[k2])
			_mesh.surface_add_vertex(r0[k2])
			_mesh.surface_add_vertex(r1[k])
			_mesh.surface_add_vertex(r1[k2])
	_mesh.surface_end()


func _tangent_at(points: PackedVector3Array, i: int) -> Vector3:
	var a: Vector3
	var b: Vector3
	if i == 0:
		a = points[0]; b = points[1]
	elif i == points.size() - 1:
		a = points[i - 1]; b = points[i]
	else:
		a = points[i - 1]; b = points[i + 1]
	var t := b - a
	if t.length() < 1e-9:
		return Vector3(0, 0, 1)
	return t.normalized()


func _arbitrary_perp(t: Vector3) -> Vector3:
	var ref := Vector3.UP
	if absf(t.dot(ref)) > 0.99:
		ref = Vector3.RIGHT
	return (ref - t * ref.dot(t)).normalized()
