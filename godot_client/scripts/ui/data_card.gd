class_name DataCard
extends PanelContainer
## Navigation-and-safety data card (VPP §2.5): 标题小字 / 主数值大字 + 单位 / 底部进度条.
## Used in the right-side 2x4 data grid. Construct with
## DataCard.new(title, value, unit, color) and update via set_value / set_bar / set_color.

var _value: Label
var _unit: Label
var _hint: Label
var _bar: ProgressBar
var _accent: Color


func _init(title: String, value: String, unit: String, color: Color, bar_pct := 60.0,
		hint := "") -> void:
	_accent = color
	add_theme_stylebox_override("panel", UiStyle.card_box(0.9, 8))
	size_flags_horizontal = Control.SIZE_EXPAND_FILL
	size_flags_vertical = Control.SIZE_EXPAND_FILL

	var vb := VBoxContainer.new()
	vb.add_theme_constant_override("separation", 1)
	add_child(vb)

	vb.add_child(UiStyle.label(title, UiStyle.TEXT_MID, 12))

	var valrow := HBoxContainer.new()
	valrow.add_theme_constant_override("separation", 4)
	_value = UiStyle.label(value, color, 22)
	valrow.add_child(_value)
	_unit = UiStyle.label(unit, UiStyle.TEXT2, 11)
	_unit.size_flags_vertical = Control.SIZE_SHRINK_END
	valrow.add_child(_unit)
	vb.add_child(valrow)

	# Optional hint line (doc/11 §13 卡8: "请保持谨慎操作").
	if hint != "":
		_hint = UiStyle.label(hint, UiStyle.TEXT2, 10)
		vb.add_child(_hint)

	var spacer := Control.new()
	spacer.size_flags_vertical = Control.SIZE_EXPAND_FILL
	vb.add_child(spacer)

	_bar = ProgressBar.new()
	_bar.show_percentage = false
	_bar.min_value = 0.0
	_bar.max_value = 100.0
	_bar.value = bar_pct
	_bar.custom_minimum_size = Vector2(0, 4)
	var styles := UiStyle.bar_style(color)
	_bar.add_theme_stylebox_override("background", styles[0])
	_bar.add_theme_stylebox_override("fill", styles[1])
	vb.add_child(_bar)


func set_value(v: String) -> void:
	_value.text = v


func set_unit(u: String) -> void:
	_unit.text = u


func set_bar(pct: float) -> void:
	if _bar:
		_bar.value = pct


func set_color(c: Color) -> void:
	_accent = c
	_value.add_theme_color_override("font_color", c)
	if _bar:
		_bar.add_theme_stylebox_override("fill", UiStyle.bar_style(c)[1])


func set_hint(text: String, color := Color(0.498, 0.561, 0.639)) -> void:
	if _hint:
		_hint.text = text
		_hint.add_theme_color_override("font_color", color)
