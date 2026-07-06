class_name StatusIcon
extends Control
## Line-drawn status icons for the top StatusCards (doc/11 §4: 线性图标, 2px 描边,
## 圆角线帽, 32-42px; §26 禁止 emoji). Each `kind` is a small _draw() sketch:
##   robot   机器人头     compass  罗盘/导航环   path    路径节点连线
##   radius  圆环+半径线  curve    弯曲曲线      wall    圆环+内部偏移点
##   warning 三角警告

var kind := "robot"
var color := Color(0.847, 0.902, 0.953):
	set(c):
		color = c
		queue_redraw()

const W := 2.0  # stroke width


func _init(icon_kind := "robot", icon_color := Color(0.847, 0.902, 0.953)) -> void:
	kind = icon_kind
	color = icon_color
	custom_minimum_size = Vector2(36, 36)
	mouse_filter = Control.MOUSE_FILTER_IGNORE


func _draw() -> void:
	var c := size * 0.5
	var r: float = min(size.x, size.y) * 0.5 - 3.0
	match kind:
		"robot":
			# Rounded head + eyes + antenna.
			var head := Rect2(c - Vector2(r * 0.8, r * 0.55), Vector2(r * 1.6, r * 1.2))
			_rounded_rect(head, 4.0)
			draw_circle(c + Vector2(-r * 0.35, 0), 2.2, color)
			draw_circle(c + Vector2(r * 0.35, 0), 2.2, color)
			draw_line(c - Vector2(0, r * 0.55), c - Vector2(0, r * 0.95), color, W, true)
			draw_circle(c - Vector2(0, r * 0.95), 2.0, color)
		"compass":
			draw_arc(c, r, 0.0, TAU, 32, color, W, true)
			# Needle: two opposing strokes.
			var tipn := c + Vector2(r * 0.55, -r * 0.55)
			var tail := c - Vector2(r * 0.35, -r * 0.35)
			draw_line(tail, tipn, color, W, true)
			draw_circle(c, 2.0, color)
		"path":
			# Three nodes joined by a polyline.
			var p1 := c + Vector2(-r * 0.8, r * 0.7)
			var p2 := c + Vector2(0.0, -r * 0.1)
			var p3 := c + Vector2(r * 0.8, -r * 0.75)
			draw_line(p1, p2, color, W, true)
			draw_line(p2, p3, color, W, true)
			for p in [p1, p2, p3]:
				draw_circle(p, 3.0, color)
		"radius":
			draw_arc(c, r, 0.0, TAU, 32, color, W, true)
			draw_line(c, c + Vector2(r, 0).rotated(-PI / 4.0), color, W, true)
			draw_circle(c, 2.0, color)
		"curve":
			# An S-ish bend from two arcs.
			draw_arc(c + Vector2(-r * 0.4, r * 0.45), r * 0.62, -PI * 0.55, PI * 0.1, 20, color, W, true)
			draw_arc(c + Vector2(r * 0.4, -r * 0.45), r * 0.62, PI * 0.45, PI * 1.1, 20, color, W, true)
		"wall":
			draw_arc(c, r, 0.0, TAU, 32, color, W, true)
			var p := c + Vector2(r * 0.35, r * 0.2)
			draw_circle(p, 3.0, color)
			draw_line(p, c + Vector2(r, 0).rotated(0.35), color, W * 0.75, true)
		"warning":
			var top := c + Vector2(0, -r)
			var bl := c + Vector2(-r * 0.95, r * 0.75)
			var br := c + Vector2(r * 0.95, r * 0.75)
			draw_line(top, bl, color, W, true)
			draw_line(bl, br, color, W, true)
			draw_line(br, top, color, W, true)
			draw_line(c + Vector2(0, -r * 0.35), c + Vector2(0, r * 0.15), color, W, true)
			draw_circle(c + Vector2(0, r * 0.42), 1.6, color)
		_:
			draw_arc(c, r, 0.0, TAU, 32, color, W, true)


func _rounded_rect(rect: Rect2, _radius: float) -> void:
	# Stroke-only rounded rect approximated with 4 lines (corners visually soft at 2px).
	var tl := rect.position
	var br := rect.end
	draw_line(Vector2(tl.x + 3, tl.y), Vector2(br.x - 3, tl.y), color, W, true)
	draw_line(Vector2(br.x, tl.y + 3), Vector2(br.x, br.y - 3), color, W, true)
	draw_line(Vector2(br.x - 3, br.y), Vector2(tl.x + 3, br.y), color, W, true)
	draw_line(Vector2(tl.x, br.y - 3), Vector2(tl.x, tl.y + 3), color, W, true)
