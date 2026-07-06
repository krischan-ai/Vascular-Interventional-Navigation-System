class_name DSAOverlay
extends Control
## Medical navigation overlay for the DSA pane (doc/11 §6): catheter curve, planned
## path, target point, risk halos and a locator frame, drawn with _draw() over the
## (placeholder) fluoroscopy image. Geometry is defined in normalized [0..1] pane
## coordinates and scaled to the current size, so it survives resizes.
##
## The curves are MOCK DATA shaped after the reference design; when the backend later
## streams a 2D-projected catheter/route (DSA track), replace the constants with
## set_catheter()/set_path() calls — the drawing code already works off the members.

const GREEN := Color(0.306, 0.902, 0.420)   # 4EE66B
const YELLOW := Color(1.0, 0.831, 0.278)    # FFD447
const RED := Color(1.0, 0.302, 0.310)       # FF4D4F
const RED_HOT := Color(1.0, 0.180, 0.180)   # FF2E2E

# Catheter already-travelled curve (bottom-left entering, snaking up). Normalized.
var catheter := PackedVector2Array([
	Vector2(0.18, 0.92), Vector2(0.22, 0.82), Vector2(0.30, 0.74), Vector2(0.36, 0.66),
	Vector2(0.38, 0.56), Vector2(0.42, 0.48), Vector2(0.50, 0.44),
])
# Planned path continuing from the catheter tip to the target. Normalized.
var planned := PackedVector2Array([
	Vector2(0.50, 0.44), Vector2(0.56, 0.38), Vector2(0.58, 0.30),
	Vector2(0.62, 0.24), Vector2(0.66, 0.18),
])
var target := Vector2(0.66, 0.18)
var mid_risk := Vector2(0.44, 0.47)   # 黄色光圈 (near the current tip)
var high_risk := Vector2(0.58, 0.29)  # 红色热区 (on the路径弯折处)


func _ready() -> void:
	mouse_filter = Control.MOUSE_FILTER_IGNORE
	resized.connect(queue_redraw)


func _draw() -> void:
	var s := size

	# Risk halos first (under the lines). 25%~40% translucent per spec.
	_halo(high_risk * s, s.x * 0.045, RED_HOT, 0.32)
	_halo(mid_risk * s, s.x * 0.038, YELLOW, 0.28)

	# Planned path: yellow dashed, 3px.
	_dashed_polyline(_scaled(planned), YELLOW, 3.0, 10.0, 7.0)

	# Catheter: green solid, 3px, endpoint ring.
	var cat := _scaled(catheter)
	if cat.size() >= 2:
		draw_polyline(cat, GREEN, 3.0, true)
		var tip := cat[cat.size() - 1]
		draw_arc(tip, 7.0, 0.0, TAU, 24, GREEN, 2.0, true)
		# Locator corner brackets around the tip (定位框).
		_corner_frame(tip, 26.0, GREEN, 2.0)

	# Target: red ring + center dot, ~20px.
	var t := target * s
	draw_arc(t, 10.0, 0.0, TAU, 24, RED, 2.5, true)
	draw_circle(t, 3.0, RED)


func _scaled(pts: PackedVector2Array) -> PackedVector2Array:
	var out := PackedVector2Array()
	for p in pts:
		out.append(p * size)
	return out


func _halo(center: Vector2, radius: float, color: Color, alpha: float) -> void:
	var c := color
	c.a = alpha
	draw_circle(center, radius, c)
	c.a = alpha * 0.5
	draw_circle(center, radius * 1.45, c)


func _dashed_polyline(pts: PackedVector2Array, color: Color, width: float,
		dash: float, gap: float) -> void:
	for i in pts.size() - 1:
		var a := pts[i]
		var b := pts[i + 1]
		var seg := b - a
		var seg_len := seg.length()
		if seg_len < 0.001:
			continue
		var dir := seg / seg_len
		var d := 0.0
		while d < seg_len:
			var e: float = min(d + dash, seg_len)
			draw_line(a + dir * d, a + dir * e, color, width, true)
			d = e + gap


func _corner_frame(center: Vector2, half: float, color: Color, width: float) -> void:
	var arm := half * 0.45
	for sx in [-1.0, 1.0]:
		for sy in [-1.0, 1.0]:
			var corner := center + Vector2(half * sx, half * sy)
			draw_line(corner, corner + Vector2(-arm * sx, 0), color, width, true)
			draw_line(corner, corner + Vector2(0, -arm * sy), color, width, true)
