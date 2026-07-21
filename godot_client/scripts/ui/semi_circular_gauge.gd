class_name SemiCircularGauge
extends Control

var value := 0.66:
	set(v):
		value = clampf(v, 0.0, 1.0)
		queue_redraw()
var accent := Color(0.306, 0.902, 0.420):
	set(v):
		accent = v
		queue_redraw()
var label := "156 mm":
	set(v):
		label = v
		queue_redraw()
var sublabel := "剩余距离":
	set(v):
		sublabel = v
		queue_redraw()

func _init() -> void:
	custom_minimum_size = Vector2(118, 96)
	mouse_filter = Control.MOUSE_FILTER_IGNORE

func _draw() -> void:
	var c := Vector2(size.x * 0.5, size.y * 0.76)
	var radius: float = minf(size.x * 0.42, size.y * 0.68)
	draw_arc(c, radius, PI, TAU, 48, Color(0.149, 0.224, 0.302, 0.75), 8.0, true)
	draw_arc(c, radius, PI, PI + PI * value, 48, accent, 8.0, true)
	draw_string(UiStyle.font(), c + Vector2(-34, -16), label, HORIZONTAL_ALIGNMENT_CENTER, 68, 16, Color(0.847, 0.902, 0.953))
	draw_string(UiStyle.font(), c + Vector2(-30, 0), sublabel, HORIZONTAL_ALIGNMENT_CENTER, 60, 11, Color(0.498, 0.561, 0.639))