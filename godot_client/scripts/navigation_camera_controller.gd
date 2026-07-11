extends RefCounted
## Route-driven follow camera pose helper.
##
## The camera rig owns the actual Camera3D nodes; this helper only samples the
## planned route and builds a stable local navigation frame. When no route is
## available, the rig falls back to the streamed guidewire direction.

const MIN_SEG_LEN := 1e-7

var _points := PackedVector3Array()
var _arc := PackedFloat32Array()
var _total_len: float = 0.0
var _has_route: bool = false
var _frame_up := Vector3.UP
var _frame_init := false


func set_route_points(waypoints: Array) -> void:
	_points = PackedVector3Array()
	for point in waypoints:
		if typeof(point) == TYPE_ARRAY and point.size() >= 3:
			_points.append(Vector3(float(point[0]), float(point[1]), float(point[2])))
		elif typeof(point) == TYPE_VECTOR3:
			_points.append(point)
	_rebuild_arclength()
	_frame_init = false


func clear_route() -> void:
	_points = PackedVector3Array()
	_arc = PackedFloat32Array()
	_total_len = 0.0
	_has_route = false
	_frame_init = false


func has_route() -> bool:
	return _has_route


func navigation_pose(
		tip: Vector3,
		fallback_dir: Vector3,
		follow_back: float,
		follow_height: float,
		lookahead_focus: float,
		lookahead_target: float) -> Dictionary:
	if not _has_route:
		return {}

	var s_tip := _closest_arclength(tip)
	var s_focus: float = minf(_total_len, s_tip + lookahead_focus)
	var s_target: float = minf(_total_len, s_tip + lookahead_target)
	var focus := _sample(s_focus)
	var target := _sample(s_target)
	var tangent := _smooth_tangent(s_focus, maxf(lookahead_focus * 0.5, 0.006))
	if tangent.length() < MIN_SEG_LEN:
		tangent = fallback_dir.normalized()
	if tangent.length() < MIN_SEG_LEN:
		tangent = Vector3.FORWARD

	var up := _transport_up(tangent)
	var camera_pos := focus - tangent * follow_back + up * follow_height
	return {
		"position": camera_pos,
		"look": target,
		"up": up,
		"tangent": tangent,
		"s_tip": s_tip,
	}


func _rebuild_arclength() -> void:
	_arc = PackedFloat32Array()
	_arc.resize(_points.size())
	_total_len = 0.0
	if _points.size() < 2:
		_has_route = false
		return
	_arc[0] = 0.0
	for i in range(1, _points.size()):
		_total_len += _points[i - 1].distance_to(_points[i])
		_arc[i] = _total_len
	_has_route = _total_len > MIN_SEG_LEN


func _closest_arclength(p: Vector3) -> float:
	var best_s := 0.0
	var best_d2 := INF
	for i in range(_points.size() - 1):
		var a := _points[i]
		var b := _points[i + 1]
		var ab := b - a
		var len2 := ab.length_squared()
		if len2 < MIN_SEG_LEN * MIN_SEG_LEN:
			continue
		var t := clampf((p - a).dot(ab) / len2, 0.0, 1.0)
		var q := a + ab * t
		var d2 := p.distance_squared_to(q)
		if d2 < best_d2:
			best_d2 = d2
			best_s = lerpf(_arc[i], _arc[i + 1], t)
	return best_s


func _sample(s: float) -> Vector3:
	if not _has_route:
		return Vector3.ZERO
	var ss := clampf(s, 0.0, _total_len)
	for i in range(_arc.size() - 1):
		var a_s := _arc[i]
		var b_s := _arc[i + 1]
		if ss <= b_s:
			var span := b_s - a_s
			var t := 0.0 if span < MIN_SEG_LEN else (ss - a_s) / span
			return _points[i].lerp(_points[i + 1], t)
	return _points[_points.size() - 1]


func _smooth_tangent(s: float, window: float) -> Vector3:
	var a := _sample(maxf(0.0, s - window))
	var b := _sample(minf(_total_len, s + window))
	var t := b - a
	if t.length() < MIN_SEG_LEN:
		return Vector3.ZERO
	return t.normalized()


func _transport_up(tangent: Vector3) -> Vector3:
	var t := tangent.normalized()
	var up := _frame_up if _frame_init else Vector3.UP
	up = up - t * up.dot(t)
	if up.length() < MIN_SEG_LEN:
		up = Vector3.RIGHT - t * Vector3.RIGHT.dot(t)
	if up.length() < MIN_SEG_LEN:
		up = Vector3.FORWARD - t * Vector3.FORWARD.dot(t)
	_frame_up = up.normalized()
	_frame_init = true
	return _frame_up
