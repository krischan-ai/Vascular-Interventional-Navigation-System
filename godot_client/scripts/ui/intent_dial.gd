class_name IntentDial
extends Control

var strength := 0.0:
	set(v):
		strength = clampf(v, 0.0, 1.0)
		queue_redraw()
var _direction_angle := 0.0


func set_input(push: float, rotate: float) -> void:
	strength = clampf(Vector2(rotate, push).length(), 0.0, 1.0)
	if absf(push) > 0.0001 or absf(rotate) > 0.0001:
		# The neutral arrow points upward. Positive rotate turns it clockwise;
		# negative push points it backward.
		_direction_angle = atan2(rotate, push)
	queue_redraw()

func _init() -> void:
	custom_minimum_size = Vector2(152, 132)
	mouse_filter = Control.MOUSE_FILTER_IGNORE

func _draw() -> void:
	var c := size * 0.5
	var r: float = minf(size.x, size.y) * 0.38
	draw_arc(c, r, -PI * 0.85, PI * 0.85, 72, Color(0.55, 0.68, 0.78, 0.95), 3.0, true)
	draw_arc(c, r * 0.72, -PI * 0.5, PI * (strength - 0.5), 48, Color(0.184, 0.549, 1.0, 0.85), 7.0, true)
	var arrow := PackedVector2Array()
	for point in [Vector2(0, -28), Vector2(14, 6), Vector2(4, 2), Vector2(0, 24), Vector2(-4, 2), Vector2(-14, 6)]:
		arrow.append(c + point.rotated(_direction_angle))
	draw_colored_polygon(arrow, Color(0.184, 0.549, 1.0, 0.95))
	draw_string(UiStyle.font(), Vector2(c.x - 34, size.y - 8), "强度：%.2f" % strength, HORIZONTAL_ALIGNMENT_CENTER, 68, 13, Color(0.722, 0.780, 0.839))
