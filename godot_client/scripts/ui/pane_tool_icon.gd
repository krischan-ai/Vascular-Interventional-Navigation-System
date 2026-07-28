class_name PaneToolIcon
extends Control
## Line-drawn icons for the 3D pane's right-side tool strip (参考图右侧操作栏; doc/11
## §26 禁止 emoji). Same 2px-stroke idiom as StatusIcon. Kinds:
##   select    鼠标箭头 (单击路径点导航)
##   orbit     环绕旋转 (圆 + 双箭头)
##   follow    视角跟随 (准星 + 追尾箭头)
##   zoom_in   放大镜 +
##   zoom_out  放大镜 −
##   frame     复位取景 (四角标 + 中心点)
##   refresh   重置腔镜姿态
##   record    录制状态圆点
##   brightness 亮度太阳
##   capture   截取当前帧
##   fullscreen 扩大/恢复腔镜窗格

var kind := "select"
var color := Color(0.847, 0.902, 0.953):
	set(c):
		color = c
		queue_redraw()

const W := 2.0  # stroke width


func _init(icon_kind := "select", icon_color := Color(0.847, 0.902, 0.953)) -> void:
	kind = icon_kind
	color = icon_color
	custom_minimum_size = Vector2(22, 22)
	mouse_filter = Control.MOUSE_FILTER_IGNORE


func _draw() -> void:
	var c := size * 0.5
	var r: float = min(size.x, size.y) * 0.5 - 2.0
	match kind:
		"select":
			# Classic pointer arrow, stroke only.
			var tip := c + Vector2(-r * 0.55, -r * 0.9)
			var pts := PackedVector2Array([
				tip,
				tip + Vector2(0, r * 1.55),
				tip + Vector2(r * 0.42, r * 1.18),
				tip + Vector2(r * 0.72, r * 1.7),
				tip + Vector2(r * 0.95, r * 1.55),
				tip + Vector2(r * 0.66, r * 1.05),
				tip + Vector2(r * 1.2, r * 1.05),
				tip,
			])
			draw_polyline(pts, color, W, true)
		"orbit":
			# Sphere with an equator ellipse + two orbit arrowheads.
			draw_arc(c, r * 0.55, 0.0, TAU, 24, color, W, true)
			draw_arc(c, r, -PI * 0.15, PI * 0.75, 20, color, W, true)
			draw_arc(c, r, PI * 0.85, PI * 1.75, 20, color, W, true)
			_arrow_head(c + Vector2(r, 0).rotated(PI * 0.75), PI * 0.75 + PI * 0.5)
			_arrow_head(c + Vector2(r, 0).rotated(PI * 1.75), PI * 1.75 + PI * 0.5)
		"follow":
			# Chase-cam: a reticle dot being trailed by two chevrons (视角跟随).
			var cc := c + Vector2(r * 0.3, 0)
			draw_arc(cc, r * 0.5, 0.0, TAU, 24, color, W, true)
			draw_circle(cc, 2.2, color)
			for j in 2:
				var bx := c.x - r * 0.95 + float(j) * r * 0.4
				draw_line(Vector2(bx, c.y - r * 0.32), Vector2(bx + r * 0.28, c.y), color, W, true)
				draw_line(Vector2(bx + r * 0.28, c.y), Vector2(bx, c.y + r * 0.32), color, W, true)
		"zoom_in":
			_magnifier(c, r)
			var g := c + Vector2(-r * 0.18, -r * 0.18)  # glass centre
			draw_line(g + Vector2(-r * 0.32, 0), g + Vector2(r * 0.32, 0), color, W, true)
			draw_line(g + Vector2(0, -r * 0.32), g + Vector2(0, r * 0.32), color, W, true)
		"zoom_out":
			_magnifier(c, r)
			var g := c + Vector2(-r * 0.18, -r * 0.18)
			draw_line(g + Vector2(-r * 0.32, 0), g + Vector2(r * 0.32, 0), color, W, true)
		"frame":
			# Four corner brackets + centre dot (取景复位).
			var half := r * 0.95
			var arm := half * 0.55
			for sx in [-1.0, 1.0]:
				for sy in [-1.0, 1.0]:
					var corner := c + Vector2(half * sx, half * sy)
					draw_line(corner, corner + Vector2(-arm * sx, 0), color, W, true)
					draw_line(corner, corner + Vector2(0, -arm * sy), color, W, true)
			draw_circle(c, 2.0, color)
		"refresh":
			draw_arc(c, r * 0.72, -PI * 0.15, PI * 1.55, 26, color, W, true)
			_arrow_head(c + Vector2(r * 0.72, 0).rotated(-PI * 0.15), PI * 0.35)
		"record":
			draw_arc(c, r * 0.76, 0.0, TAU, 28, color, W, true)
			draw_circle(c, r * 0.39, color)
		"brightness":
			draw_circle(c, r * 0.33, color)
			for i in 8:
				var a := TAU * float(i) / 8.0
				var d := Vector2.RIGHT.rotated(a)
				draw_line(c + d * r * 0.56, c + d * r * 0.92, color, W, true)
		"capture":
			var body := Rect2(c - Vector2(r * 0.86, r * 0.56), Vector2(r * 1.72, r * 1.12))
			draw_rect(body, color, false, W, true)
			draw_rect(Rect2(body.position + Vector2(r * 0.22, -r * 0.18),
				Vector2(r * 0.52, r * 0.24)), color, false, W, true)
			draw_arc(c, r * 0.34, 0.0, TAU, 24, color, W, true)
		"fullscreen":
			var half := r * 0.92
			var arm := half * 0.55
			for sx in [-1.0, 1.0]:
				for sy in [-1.0, 1.0]:
					var corner := c + Vector2(half * sx, half * sy)
					draw_line(corner, corner + Vector2(-arm * sx, 0), color, W, true)
					draw_line(corner, corner + Vector2(0, -arm * sy), color, W, true)
		_:
			draw_arc(c, r, 0.0, TAU, 24, color, W, true)


func _magnifier(c: Vector2, r: float) -> void:
	var g := c + Vector2(-r * 0.18, -r * 0.18)
	draw_arc(g, r * 0.62, 0.0, TAU, 24, color, W, true)
	var dir := Vector2(1, 1).normalized()
	draw_line(g + dir * r * 0.62, g + dir * r * 1.15, color, W, true)


func _arrow_head(at: Vector2, heading: float) -> void:
	var d := Vector2.RIGHT.rotated(heading)
	var n := d.orthogonal()
	draw_line(at, at - d * 4.5 + n * 3.0, color, W, true)
	draw_line(at, at - d * 4.5 - n * 3.0, color, W, true)
