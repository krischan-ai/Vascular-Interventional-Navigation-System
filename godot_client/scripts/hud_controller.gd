extends CanvasLayer
## Heads-up display: connection + safety status light, live navigation metrics,
## a real-time keyboard-input readout, and control hints.

const STATUS_COLORS := {
	"STANDBY": Color(0.55, 0.55, 0.6),
	"SAFE_NAV": Color(0.2, 0.85, 0.35),
	"DANGER_WARNING": Color(0.95, 0.8, 0.2),
	"COLLISION_STOP": Color(0.95, 0.25, 0.2),
}

var _light: ColorRect
var _status_label: Label
var _connection_label: Label
var _metrics_label: Label
var _input_label: Label
var _debug_label: Label


func _ready() -> void:
	var panel := PanelContainer.new()
	panel.position = Vector2(16, 16)
	panel.custom_minimum_size = Vector2(360, 0)
	add_child(panel)

	var margin := MarginContainer.new()
	margin.add_theme_constant_override("margin_left", 14)
	margin.add_theme_constant_override("margin_right", 14)
	margin.add_theme_constant_override("margin_top", 12)
	margin.add_theme_constant_override("margin_bottom", 12)
	panel.add_child(margin)

	var vbox := VBoxContainer.new()
	vbox.add_theme_constant_override("separation", 8)
	margin.add_child(vbox)

	var title := Label.new()
	title.text = "CathSim 导航"
	title.add_theme_font_size_override("font_size", 20)
	vbox.add_child(title)

	# Status light + label row.
	var status_row := HBoxContainer.new()
	status_row.add_theme_constant_override("separation", 10)
	vbox.add_child(status_row)

	_light = ColorRect.new()
	_light.custom_minimum_size = Vector2(28, 28)
	_light.color = STATUS_COLORS["STANDBY"]
	status_row.add_child(_light)

	_status_label = Label.new()
	_status_label.text = "STANDBY"
	_status_label.add_theme_font_size_override("font_size", 18)
	status_row.add_child(_status_label)

	_connection_label = Label.new()
	_connection_label.text = "● Disconnected"
	_connection_label.add_theme_color_override("font_color", Color(0.9, 0.4, 0.4))
	vbox.add_child(_connection_label)

	vbox.add_child(_separator())

	_metrics_label = Label.new()
	_metrics_label.add_theme_font_size_override("font_size", 15)
	_metrics_label.text = _format_metrics({})
	vbox.add_child(_metrics_label)

	vbox.add_child(_separator())

	_input_label = Label.new()
	_input_label.add_theme_font_size_override("font_size", 15)
	_input_label.add_theme_color_override("font_color", Color(0.6, 0.85, 1.0))
	_input_label.text = "Input  push=+0.0  rot=+0.0"
	vbox.add_child(_input_label)

	var hint := Label.new()
	hint.text = "W/S 推进·后退   A/D 旋转   R 重置"
	hint.add_theme_font_size_override("font_size", 13)
	hint.add_theme_color_override("font_color", Color(0.65, 0.65, 0.7))
	vbox.add_child(hint)

	vbox.add_child(_separator())

	_debug_label = Label.new()
	_debug_label.add_theme_font_size_override("font_size", 13)
	_debug_label.add_theme_color_override("font_color", Color(0.8, 0.75, 0.5))
	_debug_label.autowrap_mode = TextServer.AUTOWRAP_WORD_SMART
	_debug_label.custom_minimum_size = Vector2(330, 0)
	_debug_label.text = "session: none\nmsgs: 0\nlast: —"
	vbox.add_child(_debug_label)


func _separator() -> HSeparator:
	return HSeparator.new()


func set_connection(connected: bool) -> void:
	if connected:
		_connection_label.text = "● Connected"
		_connection_label.add_theme_color_override("font_color", Color(0.4, 0.85, 0.45))
	else:
		_connection_label.text = "● Disconnected"
		_connection_label.add_theme_color_override("font_color", Color(0.9, 0.4, 0.4))


func update_safety(status: String) -> void:
	_status_label.text = status
	_light.color = STATUS_COLORS.get(status, STATUS_COLORS["STANDBY"])


func update_metrics(metrics: Dictionary) -> void:
	_metrics_label.text = _format_metrics(metrics)


func update_input(push: float, rotate: float) -> void:
	_input_label.text = "Input  push=%+0.1f  rot=%+0.1f" % [push, rotate]


func set_debug(text: String) -> void:
	_debug_label.text = text


func _format_metrics(metrics: Dictionary) -> String:
	var lines := [
		"步数 Episode : %d" % int(metrics.get("episode_length", 0)),
		"速度 Speed   : %.4f m/s" % float(metrics.get("velocity", 0.0)),
		"壁距 Wall    : %.4f m" % float(metrics.get("wall_distance", 0.0)),
		"曲率 Curv    : %.2f 1/m" % float(metrics.get("curvature", 0.0)),
		"进度 Progress: %.1f %%" % (float(metrics.get("path_progress", 0.0)) * 100.0),
		"风险 Risk    : %.2f" % float(metrics.get("risk_score", 0.0)),
	]
	return "\n".join(lines)
