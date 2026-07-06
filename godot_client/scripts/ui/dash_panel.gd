class_name DashPanel
extends PanelContainer
## Titled bottom-dashboard panel (VPP §2.6): a blue accent bar + 标题, then a content
## VBox the caller fills. Used for the five bottom panels (系统状态 / 机器人连接 /
## 运动控制 / 系统日志 / 告警信息). Access the body via `.content`.

var content: VBoxContainer


func _init(title: String, stretch := 1.0) -> void:
	add_theme_stylebox_override("panel", UiStyle.panel_box(0.92, 10))
	size_flags_horizontal = Control.SIZE_EXPAND_FILL
	size_flags_stretch_ratio = stretch

	var vb := VBoxContainer.new()
	vb.add_theme_constant_override("separation", 6)
	add_child(vb)

	var head := HBoxContainer.new()
	head.add_theme_constant_override("separation", 8)
	var accent := ColorRect.new()
	accent.custom_minimum_size = Vector2(3, 15)
	accent.color = UiStyle.BLUE
	var cc := CenterContainer.new()
	cc.add_child(accent)
	head.add_child(cc)
	head.add_child(UiStyle.label(title, UiStyle.TEXT, 16))
	vb.add_child(head)

	content = VBoxContainer.new()
	content.add_theme_constant_override("separation", 4)
	content.size_flags_vertical = Control.SIZE_EXPAND_FILL
	vb.add_child(content)


## Convenience: a "标题 …… 值" row for 系统状态 / 机器人连接 panels.
## (Default value color = UiStyle.TEXT #D8E6F3, written as a literal so it stays a valid
## constant default-argument expression.)
func add_kv(key: String, value: String, value_color := Color(0.847, 0.902, 0.953)) -> Label:
	var row := HBoxContainer.new()
	var k := UiStyle.label(key, UiStyle.TEXT2, 12)
	k.size_flags_horizontal = Control.SIZE_EXPAND_FILL
	row.add_child(k)
	var v := UiStyle.label(value, value_color, 12)
	row.add_child(v)
	content.add_child(row)
	return v


## Convenience: a "时间戳 事件" log line for the 系统日志 panel (§18: 时间 #8FA6B8,
## 内容 #B8C7D6, 14px). Keeps at most `max_rows` lines, newest last.
func add_log(time: String, text: String, color := Color(0.722, 0.780, 0.839),
		max_rows := 4) -> void:
	var row := HBoxContainer.new()
	row.add_theme_constant_override("separation", 8)
	row.add_child(UiStyle.label(time, Color(0.561, 0.651, 0.722), 13))
	var t := UiStyle.label(text, color, 13)
	t.size_flags_horizontal = Control.SIZE_EXPAND_FILL
	row.add_child(t)
	content.add_child(row)
	_trim(max_rows)


## An alert card for the 告警信息 panel (§19): colored translucent bg + border.
## level: "warning" (黄) / "success" (绿) / "danger" (红).
func add_alert(time: String, text: String, level := "warning", max_rows := 3) -> void:
	var bg: Color
	var accent: Color
	match level:
		"success":
			bg = UiStyle.GREEN_BG; accent = UiStyle.GREEN
		"danger":
			bg = UiStyle.RED_BG; accent = UiStyle.RED
		_:
			bg = UiStyle.YELLOW_BG; accent = UiStyle.YELLOW
	bg.a = 0.8
	var card := PanelContainer.new()
	card.add_theme_stylebox_override("panel", UiStyle.bordered_box(bg, accent, 6))
	var row := HBoxContainer.new()
	row.add_theme_constant_override("separation", 8)
	var dot := ColorRect.new()
	dot.custom_minimum_size = Vector2(8, 8)
	dot.color = accent
	var cc := CenterContainer.new()
	cc.add_child(dot)
	row.add_child(cc)
	row.add_child(UiStyle.label(time, Color(0.561, 0.651, 0.722), 12))
	var t := UiStyle.label(text, accent, 12)
	t.size_flags_horizontal = Control.SIZE_EXPAND_FILL
	row.add_child(t)
	card.add_child(row)
	content.add_child(card)
	_trim(max_rows)


## Drop oldest rows beyond `max_rows` so the panel never overflows its region.
func _trim(max_rows: int) -> void:
	while content.get_child_count() > max_rows:
		var oldest := content.get_child(0)
		content.remove_child(oldest)
		oldest.queue_free()
