extends Node3D
## Guidewire-tracking camera rig.
##
## Parented under the vessel GLB root (the same frame as the guidewire and path
## renderers) so the streamed tip position/direction/quaternion are used directly
## as frame-local coordinates -- no world-transform conversion, and the glTF axis
## conversion applies equally to vessel, guidewire and cameras.
##
## Owns two close-range cameras; the overview camera stays owned by the main
## controller. The controller decides which camera is current.
##   - FOLLOW    : third-person close-up trailing the planned-route tangent.
##   - ENDOSCOPE : first-person view from the tip looking down the guidewire.

## Third-person follow tuning (meters). Chase configuration: the camera trails the
## tip ALONG the travel direction and looks ahead of it, so when the wire turns a
## corner the whole view swings with it (跟随模式下转弯自动转视角). Walls close to
## the lens are dissolved by the vessel shader's camera-proximity fade, so sitting
## near/inside the lumen no longer fogs the view.
@export var follow_back: float = 0.045      ## trailing distance along travel
@export var follow_height: float = 0.015    ## height above the tip
@export var follow_focus_lookahead: float = 0.012  ## route point used as chase focus
@export var follow_lookahead: float = 0.03  ## look-at point ahead of the tip
@export var follow_smooth: float = 12.0     ## position/look lerp speed (1/s)
@export var follow_fov: float = 55.0
## First-person endoscope tuning.
@export var endoscope_back: float = 0.002  ## pull back from the tip to avoid clipping
@export var endoscope_fov: float = 85.0

var follow_cam: Camera3D
var endoscope_cam: Camera3D

# Latest tip pose (frame-local). Direction defaults to the guidewire feed axis
# (+y, see src/cathsim/dm/env.py GUIDEWIRE_FEED_AXIS) until real data arrives.
var _tip := Vector3.ZERO
var _dir := Vector3(0.0, 1.0, 0.0)
var _quat := Quaternion.IDENTITY
var _has_tip := false

# Smoothed follow-cam state.
var _follow_pos := Vector3.ZERO
var _follow_look := Vector3.ZERO
var _follow_init := false
var _nav_camera = preload("res://scripts/navigation_camera_controller.gd").new()


func _ready() -> void:
	follow_cam = Camera3D.new()
	follow_cam.near = 0.001
	follow_cam.far = 50.0
	follow_cam.fov = follow_fov
	add_child(follow_cam)

	endoscope_cam = Camera3D.new()
	endoscope_cam.near = 0.0005
	endoscope_cam.far = 50.0
	endoscope_cam.fov = endoscope_fov
	# Cull the guidewire render layer so the first-person view never renders the
	# wire/tip it is embedded in (it sits inside the tube at the tip) -- otherwise
	# the camera just sees the inside of its own guidewire instead of the lumen.
	# Keep all other layers (vessel lumen, path, markers). Matches
	# guidewire_renderer.gd GUIDEWIRE_RENDER_LAYER (render layer 2).
	endoscope_cam.cull_mask = 0xFFFFF & ~(1 << 1)
	add_child(endoscope_cam)


## Feed the latest tip pose. Degenerate direction/quaternion are ignored so the
## last good orientation is kept (e.g. during STANDBY when direction is zero).
func update_tip(pos: Vector3, dir: Vector3, quat: Quaternion) -> void:
	_tip = pos
	if dir.length() > 1e-6:
		_dir = dir.normalized()
	if quat.length_squared() > 1e-6:
		_quat = quat.normalized()
	_has_tip = true


func set_navigation_route(waypoints: Array) -> void:
	_nav_camera.set_route_points(waypoints)
	_follow_init = false


func clear_navigation_route() -> void:
	_nav_camera.clear_route()
	_follow_init = false


func _process(delta: float) -> void:
	if not _has_tip:
		return
	_update_follow(delta)
	_update_endoscope()


func _update_follow(delta: float) -> void:
	# Chase pose: use the planned route tangent first, so the view follows the
	# vessel lane through bends instead of trusting a transient guidewire body
	# direction. If no backend route is available, fall back to the streamed tip
	# direction.
	var up := _safe_up(_dir, Vector3.UP)
	var desired_pos := _tip - _dir * follow_back + up * follow_height
	var desired_look := _tip + _dir * follow_lookahead
	var pose := _nav_camera.navigation_pose(
		_tip, _dir, follow_back, follow_height, follow_focus_lookahead, follow_lookahead)
	if not pose.is_empty():
		up = pose["up"]
		desired_pos = pose["position"]
		desired_look = pose["look"]
	if not _follow_init:
		_follow_pos = desired_pos
		_follow_look = desired_look
		_follow_init = true
	else:
		var t := clampf(follow_smooth * delta, 0.0, 1.0)
		_follow_pos = _follow_pos.lerp(desired_pos, t)
		_follow_look = _follow_look.lerp(desired_look, t)
	follow_cam.position = _follow_pos
	if (_follow_look - _follow_pos).length() > 1e-5:
		follow_cam.look_at(_follow_look, _safe_up(_follow_look - _follow_pos, up))


func _update_endoscope() -> void:
	endoscope_cam.position = _tip - _dir * endoscope_back
	var target := _tip + _dir
	# Roll the up vector with the tip quaternion for a natural in-vessel feel.
	var up := (_quat * Vector3.UP)
	endoscope_cam.look_at(target, _safe_up(_dir, up))


# Returns an up vector guaranteed non-parallel to forward (look_at requires it).
func _safe_up(forward: Vector3, preferred: Vector3) -> Vector3:
	var f := forward.normalized()
	var u := preferred.normalized() if preferred.length() > 1e-6 else Vector3.UP
	if absf(f.dot(u)) > 0.999:
		u = Vector3.RIGHT if absf(f.dot(Vector3.RIGHT)) < 0.999 else Vector3.FORWARD
	return u
