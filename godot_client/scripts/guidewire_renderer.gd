extends Node3D
## Renders the guidewire: a tip sphere plus a smooth tube through the per-segment
## body positions streamed in state_batch messages.
##
## The wire is built as a real tube (extruded ring along the polyline with
## parallel-transport frames) so it reads as a smooth, lit, bending guidewire
## instead of a thin jagged line. In guided mode the backend streams the full
## inserted length (entry -> tip, leaning on the inner wall at bends), so the
## whole wire visibly curves through every vessel turn.
##
## All positions arrive in the MuJoCo/guidewire meter frame, the same frame as
## the vessel GLB exported by tools/export_godot_assets.py, so no coordinate
## conversion is required.

# Wire/tip radii differ by camera: thicker in the wide overview so the wire is
# visible at low magnification, thin in the close-up follow view so it reads as a
# real (sub-mm) guidewire. The endoscope culls the wire entirely, so its value is
# irrelevant there. set_close_up() (called by the main controller on camera
# switch) swaps the active radii below.
@export var wire_radius_overview: float = 0.0014  ## meters
@export var wire_radius_closeup: float = 0.0006   ## meters
@export var tip_radius_overview: float = 0.0018   ## meters
@export var tip_radius_closeup: float = 0.0009    ## meters
@export var wire_sides: int = 8            ## tube cross-section segments
@export var wire_color: Color = Color(0.85, 0.86, 0.9)
@export var tip_color: Color = Color(1.0, 0.35, 0.2)
@export var root_color: Color = Color(0.2, 0.75, 1.0)

# Active radii (default to overview); the tube rebuilds every batch so wire_radius
# takes effect next frame, while the tip sphere is resized immediately.
var wire_radius: float = 0.0014
var tip_radius: float = 0.0018

## Render layer for the guidewire (tip + tube). The first-person endoscope camera
## culls this layer (see camera_rig.gd) so it never renders the wire it is
## embedded in; all other cameras keep it visible.
const GUIDEWIRE_RENDER_LAYER := 1 << 1  ## render layer 2

var _tip: MeshInstance3D
var _root: MeshInstance3D
var _wire: MeshInstance3D
var _wire_mesh: ImmediateMesh
var _wire_material: StandardMaterial3D
# Hidden until the first real frame arrives. A freshly created renderer (initial
# load or model switch) has its tip at the origin, which is far outside the
# off-origin VPP / segment_part vessels; showing it only once a position streams
# in avoids a stray sphere appearing outside the vessel during a switch.
var _has_data: bool = false


func _ready() -> void:
	wire_radius = wire_radius_overview
	tip_radius = tip_radius_overview
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
	_tip.layers = GUIDEWIRE_RENDER_LAYER
	add_child(_tip)

	_root = MeshInstance3D.new()
	var root_sphere := SphereMesh.new()
	root_sphere.radius = tip_radius * 0.8
	root_sphere.height = tip_radius * 1.6
	_root.mesh = root_sphere
	var root_mat := StandardMaterial3D.new()
	root_mat.albedo_color = root_color
	root_mat.emission_enabled = true
	root_mat.emission = root_color
	root_mat.emission_energy_multiplier = 0.55
	_root.material_override = root_mat
	_root.layers = GUIDEWIRE_RENDER_LAYER
	add_child(_root)

	_wire_mesh = ImmediateMesh.new()
	_wire = MeshInstance3D.new()
	_wire.mesh = _wire_mesh
	# A lit, slightly metallic material so the tube's curvature is visible
	# through shading (a smooth bending wire, not a flat scribble).
	_wire_material = StandardMaterial3D.new()
	_wire_material.albedo_color = wire_color
	_wire_material.metallic = 0.6
	_wire_material.roughness = 0.35
	_wire_material.cull_mode = BaseMaterial3D.CULL_DISABLED
	# Faint self-emission so the wire stays a crisp bright line against the cyan
	# fresnel-glow vessel and dark background (设计图: 亮白导丝), instead of dimming
	# into the wall when scene lighting is low.
	_wire_material.emission_enabled = true
	_wire_material.emission = wire_color
	_wire_material.emission_energy_multiplier = 0.25
	_wire.material_override = _wire_material
	_wire.layers = GUIDEWIRE_RENDER_LAYER
	add_child(_wire)

	# Stay hidden until the first streamed position (see _has_data).
	visible = false


func update_from_batch(batch: Dictionary) -> void:
	var tip_data: Dictionary = batch.get("tip", {})
	if tip_data.has("position"):
		_tip.position = _to_vec3(tip_data["position"])
		_mark_data()

	var bodies: Array = batch.get("bodies", [])
	var points := PackedVector3Array()
	for body in bodies:
		if typeof(body) == TYPE_DICTIONARY and body.has("pos"):
			points.append(_to_vec3(body["pos"]))
	if points.size() > 0:
		_root.position = points[0]
	_build_tube(points, wire_radius)


func update_from_state(state: Dictionary) -> void:
	if state.has("tip_position"):
		_tip.position = _to_vec3(state["tip_position"])
		_mark_data()


## Reveal the guidewire once the first real position has been applied, so its
## origin-default pose never flashes outside an off-origin vessel.
func _mark_data() -> void:
	if not _has_data:
		_has_data = true
		visible = true


## Swap to thin close-up radii (follow view) or back to the thicker overview
## radii. The tube picks up wire_radius on the next batch; resize the tip now.
func set_close_up(close: bool) -> void:
	wire_radius = wire_radius_closeup if close else wire_radius_overview
	tip_radius = tip_radius_closeup if close else tip_radius_overview
	var sphere := _tip.mesh as SphereMesh
	if sphere:
		sphere.radius = tip_radius
		sphere.height = tip_radius * 2.0
	var root_sphere := _root.mesh as SphereMesh
	if root_sphere:
		root_sphere.radius = tip_radius * 0.8
		root_sphere.height = tip_radius * 1.6


func _build_tube(points: PackedVector3Array, radius: float) -> void:
	_wire_mesh.clear_surfaces()
	var count := points.size()
	if count < 2:
		return

	var sides: int = max(3, wire_sides)
	# Build ring frames with parallel transport to avoid twisting.
	var rings: Array = []
	var ring_normals: Array = []
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
		var normals := PackedVector3Array()
		for k in sides:
			var ang := TAU * float(k) / float(sides)
			var dir := (n * cos(ang) + b * sin(ang)).normalized()
			ring.append(points[i] + dir * radius)
			normals.append(dir)
		rings.append(ring)
		ring_normals.append(normals)

	_wire_mesh.surface_begin(Mesh.PRIMITIVE_TRIANGLES, _wire_material)
	for i in count - 1:
		var r0: PackedVector3Array = rings[i]
		var r1: PackedVector3Array = rings[i + 1]
		var n0: PackedVector3Array = ring_normals[i]
		var n1: PackedVector3Array = ring_normals[i + 1]
		for k in sides:
			var k2 := (k + 1) % sides
			# Two triangles per quad (a=r0[k], b=r0[k2], c=r1[k], d=r1[k2]).
			_emit(r0[k], n0[k]);  _emit(r1[k], n1[k]);  _emit(r0[k2], n0[k2])
			_emit(r0[k2], n0[k2]); _emit(r1[k], n1[k]); _emit(r1[k2], n1[k2])
	_wire_mesh.surface_end()


func _emit(v: Vector3, n: Vector3) -> void:
	_wire_mesh.surface_set_normal(n)
	_wire_mesh.surface_add_vertex(v)


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


func _to_vec3(arr: Variant) -> Vector3:
	if typeof(arr) == TYPE_ARRAY and arr.size() >= 3:
		return Vector3(float(arr[0]), float(arr[1]), float(arr[2]))
	return Vector3.ZERO
