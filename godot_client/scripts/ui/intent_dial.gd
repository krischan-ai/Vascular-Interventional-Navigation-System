class_name IntentDial
extends Control

var strength := 0.7:
	set(v):
		strength = clampf(v, 0.0, 1.0)
		queue_redraw()

func _init() -> void:
	custom_minimum_size = Vector2(152, 132)
	mouse_filter = Control.MOUSE_FILTER_IGNORE

func _draw() -> void:
	var c := size * 0.5
	var r: float = minf(size.x, size.y) * 0.38
	draw_arc(c, r, -PI * 0.85, PI * 0.85, 72, Color(0.55, 0.68, 0.78, 0.95), 3.0, true)
	draw_arc(c, r * 0.72, -PI * 0.5, PI * (strength - 0.5), 48, Color(0.184, 0.549, 1.0, 0.85), 7.0, true)
	var arrow := PackedVector2Array([c + Vector2(0, -28), c + Vector2(14, 6), c + Vector2(4, 2), c + Vector2(0, 24), c + Vector2(-4, 2), c + Vector2(-14, 6)])
	draw_colored_polygon(arrow, Color(0.184, 0.549, 1.0, 0.95))
	draw_string(UiStyle.font(), Vector2(c.x - 34, size.y - 8), "强度：0.7", HORIZONTAL_ALIGNMENT_CENTER, 68, 13, Color(0.722, 0.780, 0.839))