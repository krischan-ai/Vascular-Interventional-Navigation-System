class_name CathSimXRBootstrap
extends Node3D
## Sprint 0 OpenXR bootstrap. It reports capability only; every business
## control remains neutral until the M2 safety gate exists.

signal status_changed(status: Dictionary)

const INTERFACE_NAME := "OpenXR"
const CONTROL_BLOCK_REASON := "sprint0_zero_control"

var _xr_interface: XRInterface
var _runtime_state := "unavailable"
var _last_status: Dictionary = {}
var _left_grip: XRController3D
var _right_grip: XRController3D
var _left_aim: XRController3D
var _right_aim: XRController3D


func _ready() -> void:
	_xr_interface = XRServer.find_interface(INTERFACE_NAME)
	if _xr_interface == null:
		_runtime_state = "interface_missing"
		push_warning("[XRBootstrap] OpenXR interface missing; zero-control fallback active")
	elif _xr_interface.is_initialized():
		_runtime_state = "initialized"
		DisplayServer.window_set_vsync_mode(DisplayServer.VSYNC_DISABLED)
		get_viewport().use_xr = true
		_create_tracking_rig()
		print("[XRBootstrap] OpenXR initialized; zero-control gate remains locked")
	else:
		_runtime_state = "runtime_unavailable"
		push_warning("[XRBootstrap] OpenXR runtime unavailable; zero-control fallback active")
	_publish_status(true)
	print("[XRBootstrap] status=%s" % JSON.stringify(status_snapshot()))


func _process(_delta: float) -> void:
	_publish_status(false)


func neutral_control_frame() -> Dictionary:
	return {
		"control_source": "xr_sprint0",
		"input_sequence": 0,
		"client_timestamp_ms": int(Time.get_unix_time_from_system() * 1000.0),
		"deadman_active": false,
		"push": 0.0,
		"rotate": 0.0,
		"controls_blocked": true,
		"control_block_reason": CONTROL_BLOCK_REASON,
	}


func status_snapshot() -> Dictionary:
	return {
		"xr_interface": INTERFACE_NAME,
		"xr_runtime_state": _runtime_state,
		"xr_session_focused": false,
		"xr_actions_active": false,
		"left_grip_tracking": _has_tracking(_left_grip),
		"right_grip_tracking": _has_tracking(_right_grip),
		"left_aim_tracking": _has_tracking(_left_aim),
		"right_aim_tracking": _has_tracking(_right_aim),
		"reference_space_type": "unresolved",
		"reference_space_valid": false,
		"controls_blocked": true,
		"control_block_reason": CONTROL_BLOCK_REASON,
		"control": neutral_control_frame(),
	}


func _create_tracking_rig() -> void:
	var origin := XROrigin3D.new()
	origin.name = "XROrigin3D"
	add_child(origin)
	var camera := XRCamera3D.new()
	camera.name = "XRCamera3D"
	origin.add_child(camera)
	_left_grip = _add_controller(origin, "LeftGrip", &"left_hand", &"grip")
	_left_aim = _add_controller(origin, "LeftAim", &"left_hand", &"aim")
	_right_grip = _add_controller(origin, "RightGrip", &"right_hand", &"grip")
	_right_aim = _add_controller(origin, "RightAim", &"right_hand", &"aim")


func _add_controller(parent: Node, node_name: String, tracker_name: StringName,
		pose_name: StringName) -> XRController3D:
	var controller := XRController3D.new()
	controller.name = node_name
	controller.tracker = tracker_name
	controller.pose = pose_name
	controller.show_when_tracked = true
	parent.add_child(controller)
	return controller


func _has_tracking(controller: XRController3D) -> bool:
	return controller != null and controller.get_has_tracking_data()


func _publish_status(force: bool) -> void:
	var next_status := status_snapshot()
	if force or next_status != _last_status:
		_last_status = next_status
		status_changed.emit(next_status.duplicate(true))
