class_name EndoscopeFallback
extends Control

var wall_color := Color(0.82, 0.30, 0.16):
	set(v):
		wall_color = v
		queue_redraw()
var shadow_color := Color(0.11, 0.030, 0.018):
	set(v):
		shadow_color = v
		queue_redraw()


func _init() -> void:
	mouse_filter = Control.MOUSE_FILTER_IGNORE


func _draw() -> void:
	var rect := Rect2(Vector2.ZERO, size)
	draw_rect(rect, shadow_color)
	var center := rect.size * 0.5 + Vector2(0, 4)
	var outer := maxf(rect.size.x, rect.size.y) * 0.52
	var inner := maxf(18.0, minf(rect.size.x, rect.size.y) * 0.13)
	for i in range(28):
		var t := float(i) / 27.0
		var radius := lerpf(outer, inner, t)
		var alpha := lerpf(0.95, 0.18, t)
		var col := wall_color.lerp(Color(0.27, 0.075, 0.035), t)
		col.a = alpha
		draw_arc(center, radius, 0.0, TAU, 96, col, maxf(3.0, radius * 0.08), true)
		center += Vector2(0, -1.8)
	for side_value in [-1.0, 1.0]:
		var side := float(side_value)
		var pts := PackedVector2Array()
		for i in range(18):
			var t := float(i) / 17.0
			var y: float = lerpf(rect.size.y * 0.12, rect.size.y * 0.88, t)
			var x: float = rect.size.x * 0.5 + side * lerpf(rect.size.x * 0.33, rect.size.x * 0.08, t)
			pts.append(Vector2(x + sin(t * 14.0) * 5.0, y))
		for i in range(pts.size() - 1):
			draw_line(pts[i], pts[i + 1], Color(1.0, 0.55, 0.30, 0.32), 2.0, true)
	var catheter := PackedVector2Array([
		Vector2(rect.size.x * 0.49, rect.size.y * 0.98),
		Vector2(rect.size.x * 0.54, rect.size.y * 0.98),
		Vector2(rect.size.x * 0.515, rect.size.y * 0.53),
	])
	draw_colored_polygon(catheter, Color(0.88, 0.92, 0.88, 0.82))
