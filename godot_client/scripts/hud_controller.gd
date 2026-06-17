extends CanvasLayer
## Minimal heads-up display: a safety-status light plus readouts for episode
## length, velocity, wall distance, curvature, path progress and risk score.

const STATUS_COLORS := {
	"STANDBY": Color(0.5, 0.5, 0.5),
	"SAFE_NAV": Color(0.2, 0.8, 0.3),
	"DANGER_WARNING": Color(0.95, 0.8, 0.2),
	"COLLISION_STOP": Color(0.9, 0.2, 0.2),
}

var _light: ColorRect
var _status_label: Label
var _metrics_label: Label
var _connection_label: Label


func _ready() -> void:
	var panel := PanelContainer.new()
	panel.position = Vector2(16, 16)
	panel.custom_minimum_size = Vector2(300, 0)
	add_child(panel)

	var vbox := VBoxContainer.new()
	vbox.add_theme_constant_override("separation", 6)
	panel.add_child(vbox)

	var header := HBoxContainer.new()
	vbox.add_child(header)

	_light = ColorRect.new()
	_light.custom_minimum_size = Vector2(24, 24)
	_light.color = STATUS_COLORS["STANDBY"]
	header.add_child(_light)

	_status_label = Label.new()
	_status_label.text = "STANDBY"
	header.add_child(_status_label)

	_connection_label = Label.new()
	_connection_label.text = "Disconnected"
	vbox.add_child(_connection_label)

	_metrics_label = Label.new()
	_metrics_label.text = "—"
	vbox.add_child(_metrics_label)


func set_connection(connected: bool) -> void:
	_connection_label.text = "Connected" if connected else "Disconnected"


func update_safety(status: String) -> void:
	_status_label.text = status
	_light.color = STATUS_COLORS.get(status, STATUS_COLORS["STANDBY"])


func update_metrics(metrics: Dictionary) -> void:
	var lines := [
		"Episode : %d" % int(metrics.get("episode_length", 0)),
		"Speed   : %.4f m/s" % float(metrics.get("velocity", 0.0)),
		"Wall    : %.4f m" % float(metrics.get("wall_distance", 0.0)),
		"Curv    : %.2f 1/m" % float(metrics.get("curvature", 0.0)),
		"Progress: %.1f %%" % (float(metrics.get("path_progress", 0.0)) * 100.0),
		"Risk    : %.2f" % float(metrics.get("risk_score", 0.0)),
	]
	_metrics_label.text = "\n".join(lines)
