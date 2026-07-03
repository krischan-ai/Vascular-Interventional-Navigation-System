extends CanvasLayer
## Dashboard-style HUD (设计图 doc/10 仪表盘): a top status/command bar, a left tool +
## diagnostics panel, right-side metric cards, and a bottom progress bar. All chrome
## is built in code and shares one dark, rounded Theme. The 3D viewport stays clickable:
## the root Control ignores mouse events, so only the panels/buttons capture clicks and
## the open centre still forwards left-clicks to the navigate handler.
##
## Control buttons emit signals the main controller wires to real actions (急停/接管/恢复
## and 视角/模型/分支/重置), duplicating the keyboard shortcuts for mouse-only operation.

signal emergency_stop
signal manual_takeover
signal resume_nav
signal view_cycle_requested
signal model_cycle_requested
signal branch_cycle_requested
signal reset_requested

const STATUS_COLORS := {
	"STANDBY": Color(0.55, 0.55, 0.6),
	"SAFE_NAV": Color(0.2, 0.85, 0.35),
	"DANGER_WARNING": Color(0.95, 0.8, 0.2),
	"COLLISION_STOP": Color(0.95, 0.25, 0.2),
}
const STATUS_TEXT := {
	"STANDBY": "待机 STANDBY",
	"SAFE_NAV": "安全导航 SAFE",
	"DANGER_WARNING": "预警 WARNING",
	"COLLISION_STOP": "制动 STOP",
}

# Palette (shared with the cyan fresnel vessel so UI + 3D read as one product).
const C_BG := Color(0.07, 0.09, 0.13, 0.92)
const C_PANEL := Color(0.10, 0.13, 0.18, 0.94)
const C_CARD := Color(0.12, 0.16, 0.22, 0.96)
const C_ACCENT := Color(0.35, 0.85, 1.0)
const C_TEXT := Color(0.86, 0.9, 0.95)
const C_DIM := Color(0.55, 0.62, 0.7)
const C_STOP := Color(0.9, 0.25, 0.22)
const C_TAKE := Color(0.95, 0.72, 0.2)

var _light: ColorRect
var _status_label: Label
var _connection_label: Label
var _risk_pill: Label
var _progress_bar: ProgressBar
var _clock_label: Label
var _bottom_status: Label
var _cards := {}  # metric key -> value Label
var _view_label: Label
var _model_label: Label
var _nav_label: Label
var _input_label: Label
var _debug_label: Label
var _theme: Theme


func _ready() -> void:
	_theme = _build_theme()
	var root := Control.new()
	root.set_anchors_and_offsets_preset(Control.PRESET_FULL_RECT)
	root.mouse_filter = Control.MOUSE_FILTER_IGNORE
	root.theme = _theme
	add_child(root)

	_build_top_bar(root)
	_build_left_panel(root)
	_build_right_cards(root)
	_build_bottom_bar(root)


func _process(_delta: float) -> void:
	if _clock_label:
		_clock_label.text = Time.get_time_string_from_system()


# ── Theme ────────────────────────────────────────────────────────────────────
func _build_theme() -> Theme:
	var t := Theme.new()
	t.default_font_size = 14
	t.set_stylebox("panel", "PanelContainer", _panel_style(C_PANEL))
	# ProgressBar: dark track, cyan fill.
	t.set_stylebox("background", "ProgressBar", _flat(Color(0.06, 0.08, 0.11), 6))
	t.set_stylebox("fill", "ProgressBar", _flat(C_ACCENT, 6))
	return t


func _flat(color: Color, radius: int) -> StyleBoxFlat:
	var sb := StyleBoxFlat.new()
	sb.bg_color = color
	sb.corner_radius_top_left = radius
	sb.corner_radius_top_right = radius
	sb.corner_radius_bottom_left = radius
	sb.corner_radius_bottom_right = radius
	return sb


func _panel_style(color: Color) -> StyleBoxFlat:
	var sb := _flat(color, 8)
	sb.border_color = Color(1, 1, 1, 0.06)
	sb.set_border_width_all(1)
	sb.content_margin_left = 12
	sb.content_margin_right = 12
	sb.content_margin_top = 10
	sb.content_margin_bottom = 10
	return sb


# ── Top command bar ──────────────────────────────────────────────────────────
func _build_top_bar(root: Control) -> void:
	var bar := PanelContainer.new()
	bar.set_anchors_and_offsets_preset(Control.PRESET_TOP_WIDE)
	bar.offset_left = 8
	bar.offset_right = -8
	bar.offset_top = 8
	root.add_child(bar)

	var row := HBoxContainer.new()
	row.add_theme_constant_override("separation", 16)
	bar.add_child(row)

	var title := Label.new()
	title.text = "CathSim 介入导航"
	title.add_theme_font_size_override("font_size", 19)
	title.add_theme_color_override("font_color", C_TEXT)
	row.add_child(title)

	# Safety status: light + text.
	var status_box := HBoxContainer.new()
	status_box.add_theme_constant_override("separation", 8)
	row.add_child(status_box)
	_light = ColorRect.new()
	_light.custom_minimum_size = Vector2(20, 20)
	_light.color = STATUS_COLORS["STANDBY"]
	status_box.add_child(_light)
	_status_label = Label.new()
	_status_label.text = STATUS_TEXT["STANDBY"]
	_status_label.add_theme_font_size_override("font_size", 16)
	status_box.add_child(_status_label)

	_connection_label = _mk_label("● 未连接", Color(0.9, 0.4, 0.4), 14)
	row.add_child(_connection_label)

	# Risk-level pill.
	_risk_pill = Label.new()
	_risk_pill.text = "风险 正常"
	_risk_pill.add_theme_font_size_override("font_size", 14)
	_risk_pill.add_theme_color_override("font_color", Color(0.05, 0.07, 0.1))
	_risk_pill.add_theme_stylebox_override("normal", _pill_style(Color(0.3, 0.8, 0.4)))
	row.add_child(_risk_pill)

	var spacer := Control.new()
	spacer.size_flags_horizontal = Control.SIZE_EXPAND_FILL
	row.add_child(spacer)

	# Command buttons (right-aligned): resume / takeover / e-stop.
	row.add_child(_accent_button("恢复导航", C_ACCENT, resume_nav))
	row.add_child(_accent_button("人工接管", C_TAKE, manual_takeover))
	row.add_child(_accent_button("急停 STOP", C_STOP, emergency_stop))


# ── Left tool + diagnostics panel ────────────────────────────────────────────
func _build_left_panel(root: Control) -> void:
	var panel := PanelContainer.new()
	panel.set_anchors_and_offsets_preset(Control.PRESET_TOP_LEFT)
	panel.offset_left = 8
	panel.offset_top = 64
	panel.custom_minimum_size = Vector2(240, 0)
	root.add_child(panel)

	var vbox := VBoxContainer.new()
	vbox.add_theme_constant_override("separation", 6)
	panel.add_child(vbox)

	vbox.add_child(_mk_label("工具 Tools", C_DIM, 13))
	var tools := GridContainer.new()
	tools.columns = 2
	tools.add_theme_constant_override("h_separation", 6)
	tools.add_theme_constant_override("v_separation", 6)
	tools.add_child(_tool_button("视角 C", view_cycle_requested))
	tools.add_child(_tool_button("模型 M", model_cycle_requested))
	tools.add_child(_tool_button("分支 B", branch_cycle_requested))
	tools.add_child(_tool_button("重置 R", reset_requested))
	vbox.add_child(tools)

	vbox.add_child(HSeparator.new())

	_view_label = _mk_label("视角 View   概览 Overview", Color(0.85, 0.8, 1.0), 14)
	vbox.add_child(_view_label)
	_model_label = _mk_label("模型 Model  —", Color(0.8, 0.95, 0.85), 14)
	vbox.add_child(_model_label)
	_nav_label = _mk_label("导航 Nav    手动 Manual", C_ACCENT, 14)
	vbox.add_child(_nav_label)
	_input_label = _mk_label("Input  push=+0.0  rot=+0.0", Color(0.6, 0.85, 1.0), 14)
	vbox.add_child(_input_label)

	vbox.add_child(HSeparator.new())

	var legend := VBoxContainer.new()
	legend.add_theme_constant_override("separation", 2)
	legend.add_child(_mk_label("● 入口 Entry", Color(0.15, 1.0, 0.4), 12))
	legend.add_child(_mk_label("● 目标 Target", Color(1.0, 0.2, 0.2), 12))
	legend.add_child(_mk_label("● 路径 Path", Color(0.2, 0.8, 1.0), 12))
	legend.add_child(_mk_label("● 点击 Goal", Color(0.2, 1.0, 0.9), 12))
	vbox.add_child(legend)

	vbox.add_child(HSeparator.new())

	_debug_label = _mk_label("session: none\nmsgs: 0\nlast: —", Color(0.8, 0.75, 0.5), 12)
	_debug_label.autowrap_mode = TextServer.AUTOWRAP_WORD_SMART
	_debug_label.custom_minimum_size = Vector2(216, 0)
	vbox.add_child(_debug_label)

	var hint := _mk_label("W/S 推进·后退  A/D 旋转  左键 导航  ESC 脱离", C_DIM, 11)
	hint.autowrap_mode = TextServer.AUTOWRAP_WORD_SMART
	hint.custom_minimum_size = Vector2(216, 0)
	vbox.add_child(hint)


# ── Right metric cards ───────────────────────────────────────────────────────
func _build_right_cards(root: Control) -> void:
	var panel := PanelContainer.new()
	panel.set_anchors_and_offsets_preset(Control.PRESET_TOP_RIGHT)
	panel.offset_left = -212
	panel.offset_right = -8
	panel.offset_top = 64
	root.add_child(panel)

	var grid := GridContainer.new()
	grid.columns = 2
	grid.add_theme_constant_override("h_separation", 8)
	grid.add_theme_constant_override("v_separation", 8)
	panel.add_child(grid)

	grid.add_child(_metric_card("wall", "壁距 Wall", "mm"))
	grid.add_child(_metric_card("curv", "曲率 Curv", "1/m"))
	grid.add_child(_metric_card("speed", "速度 Speed", "m/s"))
	grid.add_child(_metric_card("progress", "进度 Progress", "%"))
	grid.add_child(_metric_card("risk", "风险 Risk", ""))
	grid.add_child(_metric_card("episode", "步数 Step", ""))


func _metric_card(key: String, title: String, unit: String) -> Control:
	var card := PanelContainer.new()
	card.add_theme_stylebox_override("panel", _card_style())
	card.custom_minimum_size = Vector2(92, 0)
	var vb := VBoxContainer.new()
	vb.add_theme_constant_override("separation", 1)
	card.add_child(vb)
	vb.add_child(_mk_label(title, C_DIM, 11))
	var value := Label.new()
	value.text = "—"
	value.add_theme_font_size_override("font_size", 20)
	value.add_theme_color_override("font_color", C_ACCENT)
	vb.add_child(value)
	if unit != "":
		vb.add_child(_mk_label(unit, C_DIM, 10))
	_cards[key] = value
	return card


func _card_style() -> StyleBoxFlat:
	var sb := _flat(C_CARD, 6)
	sb.border_color = Color(1, 1, 1, 0.05)
	sb.set_border_width_all(1)
	sb.content_margin_left = 10
	sb.content_margin_right = 10
	sb.content_margin_top = 8
	sb.content_margin_bottom = 8
	return sb


# ── Bottom progress bar ──────────────────────────────────────────────────────
func _build_bottom_bar(root: Control) -> void:
	var bar := PanelContainer.new()
	bar.set_anchors_and_offsets_preset(Control.PRESET_BOTTOM_WIDE)
	bar.offset_left = 8
	bar.offset_right = -8
	bar.offset_bottom = -8
	root.add_child(bar)

	var row := HBoxContainer.new()
	row.add_theme_constant_override("separation", 14)
	bar.add_child(row)

	_bottom_status = _mk_label("待机 STANDBY", C_DIM, 13)
	row.add_child(_bottom_status)

	row.add_child(_mk_label("进度", C_DIM, 13))
	_progress_bar = ProgressBar.new()
	_progress_bar.min_value = 0.0
	_progress_bar.max_value = 100.0
	_progress_bar.value = 0.0
	_progress_bar.show_percentage = true
	_progress_bar.custom_minimum_size = Vector2(0, 18)
	_progress_bar.size_flags_horizontal = Control.SIZE_EXPAND_FILL
	row.add_child(_progress_bar)

	_clock_label = _mk_label("--:--:--", C_TEXT, 13)
	row.add_child(_clock_label)


# ── Small builders ───────────────────────────────────────────────────────────
func _mk_label(text: String, color: Color, size: int) -> Label:
	var l := Label.new()
	l.text = text
	l.add_theme_font_size_override("font_size", size)
	l.add_theme_color_override("font_color", color)
	return l


func _pill_style(color: Color) -> StyleBoxFlat:
	var sb := _flat(color, 10)
	sb.content_margin_left = 10
	sb.content_margin_right = 10
	sb.content_margin_top = 3
	sb.content_margin_bottom = 3
	return sb


func _accent_button(text: String, color: Color, sig: Signal) -> Button:
	var b := Button.new()
	b.text = text
	b.add_theme_font_size_override("font_size", 14)
	b.add_theme_color_override("font_color", Color(0.05, 0.07, 0.1))
	b.add_theme_color_override("font_hover_color", Color(0.02, 0.03, 0.05))
	var normal := _pill_style(color)
	normal.content_margin_left = 14
	normal.content_margin_right = 14
	normal.content_margin_top = 6
	normal.content_margin_bottom = 6
	var hover: StyleBoxFlat = normal.duplicate()
	hover.bg_color = color.lightened(0.12)
	var pressed: StyleBoxFlat = normal.duplicate()
	pressed.bg_color = color.darkened(0.15)
	b.add_theme_stylebox_override("normal", normal)
	b.add_theme_stylebox_override("hover", hover)
	b.add_theme_stylebox_override("pressed", pressed)
	b.pressed.connect(func(): sig.emit())
	return b


func _tool_button(text: String, sig: Signal) -> Button:
	var b := Button.new()
	b.text = text
	b.add_theme_font_size_override("font_size", 13)
	b.add_theme_color_override("font_color", C_TEXT)
	b.custom_minimum_size = Vector2(104, 30)
	var normal := _flat(Color(0.16, 0.2, 0.27), 6)
	var hover := _flat(Color(0.2, 0.26, 0.34), 6)
	var pressed := _flat(C_ACCENT.darkened(0.2), 6)
	b.add_theme_stylebox_override("normal", normal)
	b.add_theme_stylebox_override("hover", hover)
	b.add_theme_stylebox_override("pressed", pressed)
	b.pressed.connect(func(): sig.emit())
	return b


# ── Public API (unchanged signatures, consumed by main_controller) ───────────
func set_connection(connected: bool) -> void:
	if connected:
		_connection_label.text = "● 已连接 Connected"
		_connection_label.add_theme_color_override("font_color", Color(0.4, 0.85, 0.45))
	else:
		_connection_label.text = "● 未连接 Disconnected"
		_connection_label.add_theme_color_override("font_color", Color(0.9, 0.4, 0.4))


func update_safety(status: String) -> void:
	_status_label.text = STATUS_TEXT.get(status, status)
	_light.color = STATUS_COLORS.get(status, STATUS_COLORS["STANDBY"])
	if _bottom_status:
		_bottom_status.text = STATUS_TEXT.get(status, status)
		_bottom_status.add_theme_color_override("font_color", STATUS_COLORS.get(status, C_DIM))


func update_metrics(metrics: Dictionary) -> void:
	var wall_mm := float(metrics.get("wall_distance", 0.0)) * 1000.0
	var progress := float(metrics.get("path_progress", 0.0)) * 100.0
	var risk := float(metrics.get("risk_score", 0.0))
	_set_card("wall", "%.1f" % wall_mm)
	_set_card("curv", "%.2f" % float(metrics.get("curvature", 0.0)))
	_set_card("speed", "%.4f" % float(metrics.get("velocity", 0.0)))
	_set_card("progress", "%.0f" % progress)
	_set_card("risk", "%.2f" % risk)
	_set_card("episode", "%d" % int(metrics.get("episode_length", 0)))
	if _progress_bar:
		_progress_bar.value = progress
	_update_risk_pill(risk)


func _set_card(key: String, text: String) -> void:
	if _cards.has(key):
		_cards[key].text = text


func _update_risk_pill(risk: float) -> void:
	var color: Color
	var text: String
	if risk < 0.34:
		color = Color(0.3, 0.8, 0.4); text = "风险 正常 Safe"
	elif risk < 0.67:
		color = Color(0.95, 0.75, 0.2); text = "风险 预警 Warning"
	else:
		color = Color(0.92, 0.3, 0.25); text = "风险 危险 Danger"
	_risk_pill.text = text
	_risk_pill.add_theme_stylebox_override("normal", _pill_style(color))


func update_input(push: float, rotate: float) -> void:
	_input_label.text = "Input  push=%+0.1f  rot=%+0.1f" % [push, rotate]


func set_view_mode(mode_name: String) -> void:
	_view_label.text = "视角 View   %s" % mode_name


func set_model(model_name: String) -> void:
	_model_label.text = "模型 Model  %s" % model_name


func set_nav(text: String, active: bool = true) -> void:
	_nav_label.text = "导航 Nav    %s" % text
	var color := C_ACCENT if active else C_DIM
	_nav_label.add_theme_color_override("font_color", color)


func set_debug(text: String) -> void:
	_debug_label.text = text
