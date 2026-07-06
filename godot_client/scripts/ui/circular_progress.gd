class_name CircularProgress
extends Control
## Circular progress ring (doc/11 §4 路径进度: "显示圆环进度，不要只显示文字").
## Drawn with _draw(): a dark full ring + a colored value arc starting at 12 o'clock.

var value := 0.0:  # 0..100
	set(v):
		value = clampf(v, 0.0, 100.0)
		queue_redraw()
var ring_color := Color(0.306, 0.902, 0.420):  # 4EE66B
	set(c):
		ring_color = c
		queue_redraw()

const TRACK := Color(0.149, 0.224, 0.302)  # 26394D


func _init() -> void:
	custom_minimum_size = Vector2(40, 40)
	mouse_filter = Control.MOUSE_FILTER_IGNORE


func _draw() -> void:
	var c := size * 0.5
	var r: float = min(size.x, size.y) * 0.5 - 3.0
	draw_arc(c, r, 0.0, TAU, 40, TRACK, 4.0, true)
	if value > 0.5:
		var start := -PI * 0.5  # 12 o'clock
		draw_arc(c, r, start, start + TAU * value / 100.0, 40, ring_color, 4.0, true)
