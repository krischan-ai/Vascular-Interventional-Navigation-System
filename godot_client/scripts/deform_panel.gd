extends CanvasLayer
## Interactive guidewire deformation panel (D4 real-force Newton backend).
##
## Right-side sliders that live-tune the backend's force-drive parameters over the
## `engine_params` WebSocket message (services/websocket_handler.py). Live params
## (bend / tip_bend / push_speed / rotate_speed) map to joint_target_ke and drive
## gains and apply instantly; jtip_deg rebuilds the rod rest shape, so it is sent
## only when the slider drag ends (not every tick) to avoid rebuild spam.

signal param_changed(params: Dictionary)  ## {name: value} to send to the backend

# {name, min, max, def, rebuild, label}. rebuild=true params reshape geometry.
const SPECS := [
	{"name": "bend", "min": 1.0, "max": 80.0, "def": 20.0, "rebuild": false, "label": "杆身刚度 Bend"},
	{"name": "tip_bend", "min": 0.5, "max": 20.0, "def": 2.0, "rebuild": false, "label": "软头刚度 Tip"},
	{"name": "jtip_deg", "min": 0.0, "max": 60.0, "def": 35.0, "rebuild": true, "label": "J尖角度 J-tip°"},
	{"name": "push_speed", "min": 0.01, "max": 0.15, "def": 0.05, "rebuild": false, "label": "推进速度 Push"},
	{"name": "rotate_speed", "min": 0.5, "max": 8.0, "def": 3.0, "rebuild": false, "label": "扭转速度 Rotate"},
]

var _value_labels := {}
var _sliders := {}


func _ready() -> void:
	# Anchored to the BOTTOM-RIGHT so it clears the dashboard top bar and the
	# top-right metric cards (it previously sat top-right and overlapped both).
	# Grows up-and-left from the corner, sitting just above the bottom progress bar.
	var panel := PanelContainer.new()
	panel.anchor_left = 1.0
	panel.anchor_right = 1.0
	panel.anchor_top = 1.0
	panel.anchor_bottom = 1.0
	panel.offset_left = -320.0
	panel.offset_right = -16.0
	panel.offset_bottom = -52.0
	panel.grow_horizontal = Control.GROW_DIRECTION_BEGIN
	panel.grow_vertical = Control.GROW_DIRECTION_BEGIN
	add_child(panel)

	var margin := MarginContainer.new()
	for side in ["left", "right", "top", "bottom"]:
		margin.add_theme_constant_override("margin_%s" % side, 12)
	panel.add_child(margin)

	var vbox := VBoxContainer.new()
	vbox.add_theme_constant_override("separation", 6)
	margin.add_child(vbox)

	var title := Label.new()
	title.text = "形变参数 Deform (Newton)"
	title.add_theme_font_size_override("font_size", 16)
	vbox.add_child(title)

	for spec in SPECS:
		_add_slider(vbox, spec)

	var hint := Label.new()
	hint.text = "拖动实时改变导丝形变 · J尖角度松手后重建"
	hint.add_theme_font_size_override("font_size", 11)
	hint.add_theme_color_override("font_color", Color(0.65, 0.65, 0.7))
	hint.autowrap_mode = TextServer.AUTOWRAP_WORD_SMART
	hint.custom_minimum_size = Vector2(280, 0)
	vbox.add_child(hint)


func _add_slider(parent: VBoxContainer, spec: Dictionary) -> void:
	var pname := str(spec.name)
	var is_rebuild := bool(spec.rebuild)

	var head := HBoxContainer.new()
	parent.add_child(head)
	var name_label := Label.new()
	name_label.text = str(spec.label)
	name_label.add_theme_font_size_override("font_size", 13)
	name_label.size_flags_horizontal = Control.SIZE_EXPAND_FILL
	head.add_child(name_label)
	var val_label := Label.new()
	val_label.add_theme_font_size_override("font_size", 13)
	val_label.add_theme_color_override("font_color", Color(0.6, 0.85, 1.0))
	head.add_child(val_label)
	_value_labels[pname] = val_label

	var slider := HSlider.new()
	slider.min_value = float(spec.min)
	slider.max_value = float(spec.max)
	slider.step = (float(spec.max) - float(spec.min)) / 200.0
	slider.value = float(spec.def)
	slider.custom_minimum_size = Vector2(280, 0)
	parent.add_child(slider)
	_sliders[pname] = slider
	_update_value_label(pname, float(spec.def))

	# Signal args arrive first, then the bound extras.
	slider.value_changed.connect(_on_slider_changed.bind(pname, is_rebuild))
	if is_rebuild:
		slider.drag_ended.connect(_on_drag_ended.bind(pname, slider))


func _on_slider_changed(value: float, pname: String, is_rebuild: bool) -> void:
	_update_value_label(pname, value)
	# Live params stream on every change; rebuild params wait for drag release.
	if not is_rebuild:
		param_changed.emit({pname: value})


func _on_drag_ended(value_changed: bool, pname: String, slider: HSlider) -> void:
	if value_changed:
		param_changed.emit({pname: slider.value})


func _update_value_label(pname: String, value: float) -> void:
	var fmt := "%.3f" if value < 1.0 else "%.1f"
	_value_labels[pname].text = fmt % value


## Sync sliders to the backend's echoed effective state (reflects any clamping).
func sync_effective(effective: Dictionary) -> void:
	for pname in effective.keys():
		if _sliders.has(pname):
			var slider: HSlider = _sliders[pname]
			slider.set_value_no_signal(float(effective[pname]))
			_update_value_label(pname, float(effective[pname]))
