extends Node
## Translates keyboard input into navigation control commands.
##
## W/S drive delta_push (forward/back), A/D drive delta_rotate (left/right),
## R requests an episode reset. Uses direct physical-key polling so it works
## regardless of the project's InputMap. Control commands are throttled to
## ~20 Hz to stay under the backend's 30 Hz rate limit.

## control: only emitted when there is input, so the backend does not step the
##          simulation on idle frames.
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
## Emitted on each B keypress to cycle the navigation target branch
## (multi-branch phantoms like aorta_tree; ignored when only one route exists).
signal branch_cycle
## Emitted on each X keypress to swap the DSA 实时影像 and 3D 血管导航 panes
## between the big left region and the small right-top region.
signal pane_swap
## Raw pointer stream (root-window coords). The main controller decides what a
## gesture means per pane: a short left press-release is click-to-navigate, a
## left drag orbits the 3D camera, a middle drag pans it, the wheel zooms.
signal pointer_down(screen_pos: Vector2)
signal pointer_drag(screen_pos: Vector2, relative: Vector2)
signal pointer_up(screen_pos: Vector2)
signal pan_drag(screen_pos: Vector2, relative: Vector2)
signal wheel_zoom(steps: int, screen_pos: Vector2)
## Emitted on ESC to disengage click autopilot and return to manual control.
signal autopilot_off

@export var send_interval: float = 0.05  ## seconds (~20 Hz)

var _accum: float = 0.0
var _left_held: bool = false
var _mid_held: bool = false


func _input(event: InputEvent) -> void:
	if event is InputEventKey and event.pressed and not event.echo:
		if event.physical_keycode == KEY_R:
			reset_requested.emit()
		elif event.physical_keycode == KEY_C:
			view_cycle.emit()
		elif event.physical_keycode == KEY_M:
			model_cycle.emit()
		elif event.physical_keycode == KEY_B:
			branch_cycle.emit()
		elif event.physical_keycode == KEY_X:
			pane_swap.emit()
		elif event.physical_keycode == KEY_ESCAPE:
			autopilot_off.emit()
	elif event is InputEventMouseButton:
		match event.button_index:
			MOUSE_BUTTON_LEFT:
				_left_held = event.pressed
				if event.pressed:
					pointer_down.emit(event.position)
				else:
					pointer_up.emit(event.position)
			MOUSE_BUTTON_MIDDLE:
				_mid_held = event.pressed
			MOUSE_BUTTON_WHEEL_UP:
				if event.pressed:
					wheel_zoom.emit(1, event.position)
			MOUSE_BUTTON_WHEEL_DOWN:
				if event.pressed:
					wheel_zoom.emit(-1, event.position)
	elif event is InputEventMouseMotion:
		if _left_held:
			pointer_drag.emit(event.position, event.relative)
		elif _mid_held:
			pan_drag.emit(event.position, event.relative)


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
	if push != 0.0 or rotate != 0.0:
		control.emit(push, rotate)
