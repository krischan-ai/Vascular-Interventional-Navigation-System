class_name EndoscopeFallback
extends Control
## Honest empty/loading state for the endoscope pane.
##
## This intentionally does not draw a simulated tunnel. A procedural vessel image
## behind a green LIVE badge was indistinguishable from real telemetry. The pane now
## becomes LIVE only after the connection is ready, a vessel GLB is loaded and a real
## guidewire-front pose has arrived.

var _status_dot: Label
var _title_label: Label
var _detail_label: Label


func _init() -> void:
	mouse_filter = Control.MOUSE_FILTER_IGNORE


func _ready() -> void:
	var bg := ColorRect.new()
	bg.color = Color(0.028, 0.018, 0.020, 1.0)
	bg.set_anchors_and_offsets_preset(Control.PRESET_FULL_RECT)
	bg.mouse_filter = Control.MOUSE_FILTER_IGNORE
	add_child(bg)

	var center := CenterContainer.new()
	center.set_anchors_and_offsets_preset(Control.PRESET_FULL_RECT)
	center.mouse_filter = Control.MOUSE_FILTER_IGNORE
	add_child(center)

	var column := VBoxContainer.new()
	column.alignment = BoxContainer.ALIGNMENT_CENTER
	column.add_theme_constant_override("separation", 7)
	center.add_child(column)

	_status_dot = UiStyle.label("○", UiStyle.TEXT2, 30)
	_status_dot.horizontal_alignment = HORIZONTAL_ALIGNMENT_CENTER
	column.add_child(_status_dot)

	_title_label = UiStyle.label("等待腔镜数据", UiStyle.TEXT_MID, 15)
	_title_label.horizontal_alignment = HORIZONTAL_ALIGNMENT_CENTER
	column.add_child(_title_label)

	_detail_label = UiStyle.label("连接仿真并接收导丝尖端位姿后显示", UiStyle.TEXT2, 12)
	_detail_label.horizontal_alignment = HORIZONTAL_ALIGNMENT_CENTER
	column.add_child(_detail_label)


func set_status(title: String, detail: String, color: Color = UiStyle.TEXT2) -> void:
	if _title_label == null:
		return
	_title_label.text = title
	_detail_label.text = detail
	_status_dot.add_theme_color_override("font_color", color)
	_title_label.add_theme_color_override("font_color", color.lightened(0.18))
