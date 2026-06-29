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

# Active radii (default to overview); the tube rebuilds every batch so wire_radius
# takes effect next frame, while the tip sphere is resized immediately.
var wire_radius: float = 0.0014
var tip_radius: float = 0.0018

## Render layer for the guidewire (tip + tube). The first-person endoscope camera
## culls this layer (see camera_rig.gd) so it never renders the wire it is
## embedded in; all other cameras keep it visible.
const GUIDEWIRE_RENDER_LAYER := 1 << 1  ## render layer 2

var _tip: MeshInstance3D
var _wire: MeshInstance3D
var _wire_mesh: ImmediateMesh
var _wire_material: StandardMaterial3D
# Hidden until the first real frame arrives. A freshly created renderer (initial
# load or model switch) has its tip at the origin, which is far outside the
# off-origin VPP / segment_part vessels; showing it only once a position streams
# in avoids a stray sphere appearing outside the vessel during a switch.
var _has_data: bool = false

# --- Frame interpolation (Phase B real-time link) -------------------------------
# The backend streams the latest physics frame at ~25Hz, but physics itself is
# low frequency (~3Hz). To render at a smooth 60fps we interpolate body positions
# between two *distinct* physics frames (identified by the batch ``seq``): on a
# new seq we snapshot the currently displayed pose as the source and lerp toward
# the new target over the measured inter-frame interval. Duplicate-seq frames are
# ignored so the wire keeps gliding instead of stalling between physics steps.
var _source_pts := PackedVector3Array()  # displayed pose when the target arrived
var _target_pts := PackedVector3Array()  # latest distinct physics frame
var _display_pts := PackedVector3Array() # currently displayed (interpolated)
var _source_tip := Vector3.ZERO
var _target_tip := Vector3.ZERO
var _display_tip := Vector3.ZERO
var _last_seq: int = -999          # last applied physics frame number
var _lerp_t: float = 0.0           # seconds since the current target arrived
var _lerp_dur: float = 0.2         # estimated interval between physics frames (s)
var _last_frame_time: float = 0.0  # wall time the last distinct frame arrived
const _LERP_DUR_MIN := 0.05
const _LERP_DUR_MAX := 1.0


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
	_wire.material_override = _wire_material
	_wire.layers = GUIDEWIRE_RENDER_LAYER
	add_child(_wire)

	# Stay hidden until the first streamed position (see _has_data).
	visible = false


func update_from_batch(batch: Dictionary) -> void:
	# Parse the new frame's tip + body centers.
	var tip_data: Dictionary = batch.get("tip", {})
	var new_tip := _target_tip
	if tip_data.has("position"):
		new_tip = _to_vec3(tip_data["position"])

	var bodies: Array = batch.get("bodies", [])
	var new_pts := PackedVector3Array()
	for body in bodies:
		if typeof(body) == TYPE_DICTIONARY and body.has("pos"):
			new_pts.append(_to_vec3(body["pos"]))

	var seq: int = int(batch.get("seq", -1))
	# Duplicate physics frame (sender re-pushed the same seq): ignore so the
	# in-flight interpolation continues smoothly instead of resetting.
	if seq != -1 and seq == _last_seq:
		return

	# A new distinct frame: re-estimate the physics interval from arrival timing
	# so the lerp duration tracks the real (variable) physics rate.
	var now := Time.get_ticks_msec() / 1000.0
	if _last_seq != -999:
		var dt := now - _last_frame_time
		if dt > 0.0:
			_lerp_dur = clampf(dt, _LERP_DUR_MIN, _LERP_DUR_MAX)
	_last_frame_time = now
	_last_seq = seq

	# Source = currently displayed pose (so motion is continuous), unless the body
	# count changed (kinematic wire grows) or there is no seq -- then snap.
	var can_lerp := seq != -1 and _display_pts.size() == new_pts.size() and new_pts.size() > 0
	if can_lerp:
		_source_pts = _display_pts.duplicate()
		_source_tip = _display_tip
	else:
		_source_pts = new_pts.duplicate()
		_source_tip = new_tip
		_display_pts = new_pts.duplicate()
		_display_tip = new_tip
	_target_pts = new_pts
	_target_tip = new_tip
	_lerp_t = 0.0

	if new_pts.size() > 0:
		_mark_data()
	elif tip_data.has("position"):
		# No body data yet (e.g. pre-init frame): at least place the tip sphere.
		_tip.position = new_tip
		_mark_data()


func _process(delta: float) -> void:
	if not _has_data or _target_pts.size() < 2:
		return

	_lerp_t += delta
	var alpha := 1.0
	if _lerp_dur > 0.0:
		alpha = clampf(_lerp_t / _lerp_dur, 0.0, 1.0)

	if _source_pts.size() == _target_pts.size():
		_display_pts.resize(_target_pts.size())
		for i in _target_pts.size():
			_display_pts[i] = _source_pts[i].lerp(_target_pts[i], alpha)
	else:
		_display_pts = _target_pts.duplicate()
	_display_tip = _source_tip.lerp(_target_tip, alpha)

	_tip.position = _display_tip
	_build_tube(_display_pts, wire_radius)


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
