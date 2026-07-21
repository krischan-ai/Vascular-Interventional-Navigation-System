class_name MiniTrendChart
extends Control

var line_color := Color(0.306, 0.902, 0.420):
	set(v):
		line_color = v
		queue_redraw()
var threshold_color := Color(1.0, 0.302, 0.310):
	set(v):
		threshold_color = v
		queue_redraw()
var values := PackedFloat32Array([0.08, 0.12, 0.10, 0.18, 0.22, 0.16, 0.30, 0.42, 0.36, 0.64, 0.48, 0.26, 0.34, 0.46, 0.60]):
	set(v):
		values = v
		queue_redraw()

func _init() -> void:
	custom_minimum_size = Vector2(160, 104)
	mouse_filter = Control.MOUSE_FILTER_IGNORE

func _draw() -> void:
	var r := Rect2(Vector2(8, 10), size - Vector2(16, 24))
	for i in range(1, 4):
		var y := r.position.y + r.size.y * float(i) / 4.0
		draw_line(Vector2(r.position.x, y), Vector2(r.end.x, y), Color(0.149, 0.224, 0.302, 0.35), 1.0)
	for i in range(1, 5):
		var x := r.position.x + r.size.x * float(i) / 5.0
		draw_line(Vector2(x, r.position.y), Vector2(x, r.end.y), Color(0.149, 0.224, 0.302, 0.30), 1.0)
	var ty := r.position.y + r.size.y * 0.34
	for x in range(int(r.position.x), int(r.end.x), 8):
		draw_line(Vector2(x, ty), Vector2(x + 4, ty), threshold_color, 1.2)
	if values.size() < 2:
		return
	var pts := PackedVector2Array()
	for i in values.size():
		var x := r.position.x + r.size.x * float(i) / float(values.size() - 1)
		var y := r.end.y - clampf(values[i], 0.0, 1.0) * r.size.y
		pts.append(Vector2(x, y))
	for i in range(pts.size() - 1):
		draw_line(pts[i], pts[i + 1], line_color, 2.0, true)