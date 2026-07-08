extends Node3D
## Renders the planned navigation path (state_batch.path.waypoints) as one thin
## tube whose color runs along its length (涓€鏉″棰滆壊鐨勭嚎):
##   - Per-waypoint curvature hint = smoothed discrete curvature (real geometry) mapped
##     through a white-cyan -> warm gradient. This is a route readability
##     cue only, not a no-go/risk volume.
##   - The traversed portion behind the wire (path.progress) dims and fades via a
##     shader `progress` uniform, so the route never visually competes with the
##     guidewire that has already covered it. No waypoint dot spheres.
##
## A tube (not a 1px line strip) is used so the path stays visible in the
## first-person endoscope view: there the camera sits on the centerline and looks
## along it, so the near path runs almost parallel to the view. A zero-width line
## projects to nothing when seen end-on; a tube keeps a visible cross-section.
##
## Waypoints arrive in the MuJoCo/guidewire meter frame (the same frame as the
## vessel GLB and the guidewire), so no coordinate conversion is needed. The
## geometry is constant for a session and only rebuilt when the route changes;
## per-batch progress updates are a single uniform write.

# Curvature hint gradient endpoints. Keep the high end warm but not saturated
# red, so it cannot be mistaken for a real no-go/risk region.
@export var color_safe: Color = Color(0.85, 0.95, 1.0)    # 骞崇洿娈?鐧介潚
@export var color_warn: Color = Color(1.0, 0.83, 0.28)    # medium curvature: amber
@export var color_high_curvature: Color = Color(1.0, 0.58, 0.34)  # high curvature: warm hint
# Curvature (1/m) that maps to the warm endpoint; smaller = the ramp saturates earlier.
@export var curvature_red: float = 120.0
# Radius by view. Overview/follow see the route from outside, so the line must be
# visibly thick and bloom-lit. The endoscope camera sits ON the centerline (guided
# mode), i.e. inside this tube, so there the radius must be a small fraction of the
# near plane (0.0005 m) so the surrounding ring shrinks to a dot instead of a
# hexagon blocking the curve behind it 鈥?set_endoscope() swaps them.
@export var path_radius_main: float = 0.0008       ## meters (overview/follow)
@export var path_radius_endoscope: float = 0.00004 ## meters (on-centerline view)
@export var path_sides: int = 8            ## tube cross-section segments
@export var emission_energy: float = 1.8   ## >1 so the line blooms
@export var traversed_dim: float = 0.25    ## brightness of the宸茶蛋杩?portion
# Hide the tube right at the camera so the near cross-section (a faceted ring the
# endoscope camera sits inside) disappears, leaving the line from a few mm out.
# Distance fade is by camera distance, so the overview/follow cameras (always far
# from the path) keep it fully visible; only the on-centerline endoscope fades.
@export var near_hide_distance: float = 0.0015  ## m: invisible nearer than this
@export var near_show_distance: float = 0.004   ## m: fully visible beyond this

var _mesh: ImmediateMesh
var _mi: MeshInstance3D
var _material: ShaderMaterial
var _points := PackedVector3Array()  # last drawn route, for re-builds
var _curvature_hint := PackedFloat32Array()  # per-waypoint 0..1 route curvature hint
var _forced_ranges: Array = []               # legacy compatibility; forced ranges stay disabled
var _path_radius: float = 0.0008     # active radius (set_endoscope swaps)
var _endoscope: bool = false
var _drawn_count: int = -1
var _drawn_signature: String = ""


func _ready() -> void:
	_path_radius = path_radius_main
	_material = _make_path_material()
	_mesh = ImmediateMesh.new()
	_mi = MeshInstance3D.new()
	_mi.mesh = _mesh
	_mi.material_override = _material
	add_child(_mi)


# Vertex-colored emissive line shader. COLOR carries the curvature hint, UV.x the
# arc-length fraction so the `progress` uniform can dim the traversed stretch
# per-pixel. depth_test_disabled + render_priority keep the route visible through
# the vessel walls (閫忚鍥剧敾娉? so the camera can stay outside the tree; the
# view-distance fade hides the near ring the endoscope camera sits inside.
func _make_path_material() -> ShaderMaterial:
	var shader := Shader.new()
	shader.code = "shader_type spatial;\n" \
		+ "render_mode unshaded, cull_disabled, depth_test_disabled, blend_mix;\n" \
		+ "uniform float progress = 0.0;\n" \
		+ "uniform float emission_energy = 1.8;\n" \
		+ "uniform float traversed_dim = 0.25;\n" \
		+ "uniform float near_hide = 0.0015;\n" \
		+ "uniform float near_show = 0.004;\n" \
		+ "void fragment() {\n" \
		+ "	float behind = 1.0 - smoothstep(progress - 0.01, progress + 0.005, UV.x);\n" \
		+ "	float dim = mix(1.0, traversed_dim, behind);\n" \
		+ "	ALBEDO = COLOR.rgb * dim;\n" \
		+ "	EMISSION = COLOR.rgb * emission_energy * dim * dim;\n" \
		+ "	float cam_d = length(VERTEX);\n" \
		+ "	ALPHA = mix(1.0, 0.12, behind) * smoothstep(near_hide, near_show, cam_d);\n" \
		+ "}\n"
	var mat := ShaderMaterial.new()
	mat.shader = shader
	mat.render_priority = 10
	mat.set_shader_parameter("emission_energy", emission_energy)
	mat.set_shader_parameter("traversed_dim", traversed_dim)
	mat.set_shader_parameter("near_hide", near_hide_distance)
	mat.set_shader_parameter("near_show", near_show_distance)
	return mat


## Route progress 0..1 from the backend; the stretch behind it dims/fades so the
## already-covered path stops competing with the guidewire. Uniform-only: no
## geometry rebuild.
func set_progress(p: float) -> void:
	_material.set_shader_parameter("progress", clampf(p, 0.0, 1.0))


## Legacy compatibility entry point. Forced route ranges are intentionally ignored:
## real spatial risks must arrive through backend `risk_regions` and the dedicated
## risk renderer, not as path-index color overrides.
func set_risk_ranges(ranges: Array) -> void:
	if not ranges.is_empty():
		push_warning("Ignoring forced path ranges; use source-backed risk_regions for spatial risk.")
	clear_forced_ranges()


func clear_forced_ranges() -> void:
	_forced_ranges = []
	if _points.size() >= 2:
		_curvature_hint = _compute_curvature_hint(_points)
		_build_tube(_points, _path_radius)


## Endoscope on/off: swap between the visible colored line (outside views) and the
## hair-thin on-centerline guide, rebuilding the tube from the cached route.
func set_endoscope(thin: bool) -> void:
	if thin == _endoscope:
		return
	_endoscope = thin
	_path_radius = path_radius_endoscope if thin else path_radius_main
	if _points.size() >= 2:
		_build_tube(_points, _path_radius)


func update_from_batch(batch: Dictionary) -> void:
	var path: Dictionary = batch.get("path", {})
	if path.has("progress"):
		set_progress(float(path.get("progress", 0.0)))
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
	_points = points
	_curvature_hint = _compute_curvature_hint(points)
	_build_tube(points, _path_radius)


func _path_signature(waypoints: Array) -> String:
	if waypoints.is_empty():
		return ""
	return "%d:%s:%s" % [
		waypoints.size(),
		str(waypoints[0]),
		str(waypoints[waypoints.size() - 1]),
	]


# Per-waypoint curvature hint 0..1: discrete curvature (turning angle over mean
# local spacing), smoothed over +/-2 neighbors, scaled by curvature_red.
func _compute_curvature_hint(points: PackedVector3Array) -> PackedFloat32Array:
	var n := points.size()
	var out := PackedFloat32Array()
	out.resize(n)
	if n < 3:
		return out
	var kappa := PackedFloat32Array()
	kappa.resize(n)
	for i in range(1, n - 1):
		var a := points[i] - points[i - 1]
		var b := points[i + 1] - points[i]
		if a.length() < 1e-9 or b.length() < 1e-9:
			kappa[i] = 0.0
			continue
		var s := (a.length() + b.length()) * 0.5
		kappa[i] = a.angle_to(b) / s
	kappa[0] = kappa[1]
	kappa[n - 1] = kappa[n - 2]
	for i in n:
		var acc := 0.0
		var cnt := 0
		for j in range(max(0, i - 2), min(n, i + 3)):
			acc += kappa[j]
			cnt += 1
		out[i] = clampf((acc / float(cnt)) / curvature_red, 0.0, 1.0)
	for r in _forced_ranges:
		var lo: int = clampi(int(r.lo), 0, n - 1)
		var hi: int = clampi(int(r.hi), 0, n - 1)
		for i in range(lo, hi + 1):
			out[i] = 1.0
	return out


# White-cyan -> yellow -> warm-orange heat ramp for a curvature hint value.
func _gradient(d: float) -> Color:
	if d < 0.5:
		return color_safe.lerp(color_warn, d * 2.0)
	return color_warn.lerp(color_high_curvature, (d - 0.5) * 2.0)


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
		rings.append(_ring_points(points[i], n, b, sides, radius))

	# Per-ring vertex attributes: danger color + arc-length fraction (UV.x).
	var colors: Array = []
	var fracs := PackedFloat32Array()
	fracs.resize(count)
	for i in count:
		var d: float = _curvature_hint[i] if i < _curvature_hint.size() else 0.0
		colors.append(_gradient(d))
		fracs[i] = float(i) / float(count - 1)

	_mesh.surface_begin(Mesh.PRIMITIVE_TRIANGLES, _material)
	for i in count - 1:
		var r0: PackedVector3Array = rings[i]
		var r1: PackedVector3Array = rings[i + 1]
		var c0: Color = colors[i]
		var c1: Color = colors[i + 1]
		var u0 := fracs[i]
		var u1 := fracs[i + 1]
		for k in sides:
			var k2 := (k + 1) % sides
			_emit(r0[k], c0, u0)
			_emit(r1[k], c1, u1)
			_emit(r0[k2], c0, u0)
			_emit(r0[k2], c0, u0)
			_emit(r1[k], c1, u1)
			_emit(r1[k2], c1, u1)
	_mesh.surface_end()


func _emit(v: Vector3, c: Color, u: float) -> void:
	_mesh.surface_set_color(c)
	_mesh.surface_set_uv(Vector2(u, 0.0))
	_mesh.surface_add_vertex(v)


func _ring_points(center: Vector3, n: Vector3, b: Vector3, sides: int,
		radius: float) -> PackedVector3Array:
	var ring := PackedVector3Array()
	for k in sides:
		var ang := TAU * float(k) / float(sides)
		var dir := (n * cos(ang) + b * sin(ang)).normalized()
		ring.append(center + dir * radius)
	return ring


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
