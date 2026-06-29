extends Node
## Translates keyboard input into navigation control commands.
##
## W/S drive delta_push (forward/back), A/D drive delta_rotate (left/right),
## R requests an episode reset. Uses direct physical-key polling so it works
## regardless of the project's InputMap. Control commands are throttled to
## ~20 Hz to stay under the backend's 30 Hz rate limit.

## control: emitted every throttled tick (including zeros). The backend's physics
##          worker steps autonomously and reads the *latest* input, so we must keep
##          sending the current keys -- including the zero on key release, or the
##          worker would keep advancing on the last non-zero push forever.
## input_state: emitted every throttled tick (including zeros) for live HUD
##              feedback, independent of the backend connection.
signal control(delta_push: float, delta_rotate: float)
signal input_state(delta_push: float, delta_rotate: float)
signal reset_requested
## Emitted on each C keypress to cycle the camera view (overview/follow/endoscope).
signal view_cycle
## Emitted on each M keypress to cycle the phantom model
## (初始模型 low_tort / 全身体膜 segment_part / 局部血管空腔 VPP).
signal model_cycle

@export var send_interval: float = 0.05  ## seconds (~20 Hz)

var _accum: float = 0.0


func _input(event: InputEvent) -> void:
	if event is InputEventKey and event.pressed and not event.echo:
		if event.physical_keycode == KEY_R:
			reset_requested.emit()
		elif event.physical_keycode == KEY_C:
			view_cycle.emit()
		elif event.physical_keycode == KEY_M:
			model_cycle.emit()


func _process(delta: float) -> void:
	_accum += delta
	if _accum < send_interval:
		return
	_accum = 0.0

	var push := 0.0
	if Input.is_physical_key_pressed(KEY_W):
		push += 1.0
	if Input.is_physical_key_pressed(KEY_S):
		push -= 1.0

	var rotate := 0.0
	if Input.is_physical_key_pressed(KEY_D):
		rotate += 1.0
	if Input.is_physical_key_pressed(KEY_A):
		rotate -= 1.0

	input_state.emit(push, rotate)
	# Always send (including zeros): the worker reads the latest input, so a
	# release must be communicated or the guidewire keeps advancing.
	control.emit(push, rotate)
