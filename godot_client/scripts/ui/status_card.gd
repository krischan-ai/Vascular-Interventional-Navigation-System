class_name StatusCard
extends PanelContainer
## Top status-bar card (doc/11 §4): 左侧线性图标 + 右侧(上标题/下主数值+单位).
## Pass any Control as `icon` (a StatusIcon or a CircularProgress); it is vertically
## centred on the left. Update live via set_value / set_color / set_icon_color.

var _icon: Control
var _value: Label
var _unit: Label


func _init(title: String, value: String, unit: String, color: Color,
		icon: Control = null) -> void:
	add_theme_stylebox_override("panel", UiStyle.card_box(0.9, 8))
	size_flags_horizontal = Control.SIZE_EXPAND_FILL
	custom_minimum_size = Vector2(150, 0)

	var hb := HBoxContainer.new()
	hb.add_theme_constant_override("separation", 8)
	add_child(hb)

	if icon != null:
		_icon = icon
		var cc := CenterContainer.new()
		cc.add_child(icon)
		hb.add_child(cc)

	var vb := VBoxContainer.new()
	vb.add_theme_constant_override("separation", 2)
	vb.size_flags_horizontal = Control.SIZE_EXPAND_FILL
	vb.alignment = BoxContainer.ALIGNMENT_CENTER
	hb.add_child(vb)

	vb.add_child(UiStyle.label(title, UiStyle.TEXT_MID, 14))

	var valrow := HBoxContainer.new()
	valrow.add_theme_constant_override("separation", 4)
	_value = UiStyle.label(value, color, 26)
	valrow.add_child(_value)
	_unit = UiStyle.label(unit, UiStyle.TEXT2, 14)
	_unit.size_flags_vertical = Control.SIZE_SHRINK_END
	valrow.add_child(_unit)
	vb.add_child(valrow)


func set_value(v: String) -> void:
	_value.text = v


func set_unit(u: String) -> void:
	_unit.text = u


func set_color(c: Color) -> void:
	_value.add_theme_color_override("font_color", c)
	set_icon_color(c)


func set_icon_color(c: Color) -> void:
	if _icon is StatusIcon:
		(_icon as StatusIcon).color = c
	elif _icon is CircularProgress:
		(_icon as CircularProgress).ring_color = c


## For the 路径进度 card: forward the percent to an attached CircularProgress.
func set_ring(pct: float) -> void:
	if _icon is CircularProgress:
		(_icon as CircularProgress).value = pct
