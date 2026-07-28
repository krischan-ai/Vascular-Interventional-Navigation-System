class_name EndoscopeOverlay
extends Control
## Transparent optical overlay for the real endoscope SubViewport.
## Draws only lens-edge falloff and subtle corner glints; anatomy always comes from
## the vessel GLB rendered below this control.

@export var vignette_strength: float = 0.62:
	set(value):
		vignette_strength = clampf(value, 0.0, 1.0)
		queue_redraw()


func _init() -> void:
	mouse_filter = Control.MOUSE_FILTER_IGNORE


func _draw() -> void:
	if size.x <= 1.0 or size.y <= 1.0:
		return
	var steps := 18
	var max_inset := minf(size.x, size.y) * 0.18
	for i in range(steps):
		var t := float(i) / float(steps - 1)
		var inset := t * max_inset
		var alpha := pow(1.0 - t, 2.2) * 0.10 * vignette_strength
		draw_rect(
			Rect2(Vector2(inset, inset), size - Vector2(inset, inset) * 2.0),
			Color(0.12, 0.008, 0.004, alpha), false, 3.0, true)
	# Fine warm rim and two optical highlights keep the pane legible as a lens.
	draw_rect(Rect2(Vector2(1, 1), size - Vector2(2, 2)),
		Color(0.96, 0.32, 0.14, 0.24), false, 1.0, true)
	draw_line(Vector2(size.x * 0.18, 2), Vector2(size.x * 0.38, 2),
		Color(1.0, 0.72, 0.42, 0.18), 2.0, true)
	draw_line(Vector2(size.x - 2, size.y * 0.60), Vector2(size.x - 2, size.y * 0.78),
		Color(1.0, 0.52, 0.28, 0.12), 2.0, true)
	# Virtual scope sheath nose. It is an optical-housing cue anchored to the lens,
	# not fake anatomy; the moving vessel surface below remains the live content.
	var hood := PackedVector2Array([
		Vector2(size.x * 0.5 - 15.0, size.y + 2.0),
		Vector2(size.x * 0.5 + 15.0, size.y + 2.0),
		Vector2(size.x * 0.5 + 5.0, size.y - 42.0),
		Vector2(size.x * 0.5 - 4.0, size.y - 42.0),
	])
	draw_colored_polygon(hood, Color(0.93, 0.88, 0.78, 0.72))
	draw_line(hood[3], hood[2], Color(1.0, 0.96, 0.88, 0.65), 1.2, true)
