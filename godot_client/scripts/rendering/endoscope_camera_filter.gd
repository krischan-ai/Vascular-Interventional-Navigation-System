class_name EndoscopeCameraFilter
extends RefCounted
## Stable private-scope pose filter.
##
## Position uses a critically damped second-order response with retained
## velocity. Rotation has an independent exponential response. Before slerp, the
## desired up vector is rate-limited around the optical axis so real guidewire
## roll remains visible without allowing a one-frame roll jump.

const QUALITY_PERFORMANCE := "performance"
const QUALITY_BALANCED := "balanced"
const QUALITY_HIGH := "high"

var _current := Transform3D.IDENTITY
var _position_velocity := Vector3.ZERO
var _position_smooth_time := 0.065
var _rotation_response := 14.0
var _max_roll_degrees_per_second := 140.0
var _initialized := false


func configure(quality: String) -> void:
	match quality:
		QUALITY_PERFORMANCE:
			_position_smooth_time = 0.045
			_rotation_response = 16.0
			_max_roll_degrees_per_second = 180.0
		QUALITY_HIGH:
			_position_smooth_time = 0.080
			_rotation_response = 12.0
			_max_roll_degrees_per_second = 120.0
		_:
			_position_smooth_time = 0.065
			_rotation_response = 14.0
			_max_roll_degrees_per_second = 140.0


func reset() -> void:
	_initialized = false
	_position_velocity = Vector3.ZERO


func update(target: Transform3D, delta: float) -> Transform3D:
	if not _initialized or delta <= 0.0:
		_current = target.orthonormalized()
		_position_velocity = Vector3.ZERO
		_initialized = true
		return _current
	var frame_delta := minf(delta, 0.1)
	var smoothed_origin := _critical_damped_position(
		_current.origin, target.origin, frame_delta)
	var limited_target_basis := _roll_limited_basis(
		target.basis.orthonormalized(), frame_delta)
	var current_quat := _current.basis.get_rotation_quaternion().normalized()
	var target_quat := limited_target_basis.get_rotation_quaternion().normalized()
	var rotation_blend := 1.0 - exp(-_rotation_response * frame_delta)
	var smoothed_quat := current_quat.slerp(target_quat, rotation_blend).normalized()
	_current = Transform3D(Basis(smoothed_quat).orthonormalized(), smoothed_origin)
	return _current


func _critical_damped_position(
		current: Vector3, target: Vector3, delta: float) -> Vector3:
	var smooth_time := maxf(_position_smooth_time, 0.001)
	var omega := 2.0 / smooth_time
	var x := omega * delta
	var decay := 1.0 / (1.0 + x + 0.48 * x * x + 0.235 * x * x * x)
	var change := current - target
	var temporary := (_position_velocity + change * omega) * delta
	_position_velocity = (_position_velocity - temporary * omega) * decay
	return target + (change + temporary) * decay


func _roll_limited_basis(target_basis: Basis, delta: float) -> Basis:
	var forward := (-target_basis.z).normalized()
	var current_up := _project_up(_current.basis.y, forward)
	var target_up := _project_up(target_basis.y, forward)
	if current_up.length_squared() < 1e-8:
		current_up = target_up
	if target_up.length_squared() < 1e-8:
		target_up = current_up
	var roll_delta := atan2(
		forward.dot(current_up.cross(target_up)),
		clampf(current_up.dot(target_up), -1.0, 1.0))
	var max_step := deg_to_rad(_max_roll_degrees_per_second) * delta
	var limited_up := current_up.rotated(
		forward, clampf(roll_delta, -max_step, max_step)).normalized()
	return Basis.looking_at(forward, limited_up).orthonormalized()


func _project_up(up: Vector3, forward: Vector3) -> Vector3:
	var projected := up - forward * up.dot(forward)
	return projected.normalized() if projected.length_squared() > 1e-8 else Vector3.ZERO
