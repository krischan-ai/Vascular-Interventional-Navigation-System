extends CanvasLayer
## Navigation-workstation dashboard (doc/11 前端设计方案), composed from reusable UI
## components (StatusCard / StatusIcon / CircularProgress / DataCard / DashPanel)
## styled by UiStyle. This script only LAYS OUT and BINDS DATA. Three zones on the
## 1920x1080 canvas:
##   顶部状态栏  (§3/§4)   : 8 StatusCards (线性图标 + 圆环进度) + 紧急停止
##   导航与安全数据 (§12/§13): 2x4 DataCards
##   底部控制区  (§14-§19) : 系统状态17 | 机器人连接17 | 运动控制27 | 系统日志21 | 告警18
## The DSA (左 59%) and 3D (右上) panes are built by main_controller in a lower layer.
##
## Backend-fed metrics include progress, remaining distance, lumen radius,
## deviation, ETA, safety/risk, and client-estimated WebSocket latency.

const MiniTrendChartScript := preload("res://scripts/ui/mini_trend_chart.gd")
const SemiCircularGaugeScript := preload("res://scripts/ui/semi_circular_gauge.gd")
const IntentDialScript := preload("res://scripts/ui/intent_dial.gd")

signal emergency_stop
signal manual_takeover
signal resume_nav
signal motion_command(push: float, rotate: float)
signal view_cycle_requested
signal model_cycle_requested
signal branch_cycle_requested
signal reset_requested
signal deform_toggle

const STATUS_TEXT := {
	"STANDBY": "待机",
	"SAFE_NAV": "安全",
	"DANGER_WARNING": "预警",
	"COLLISION_STOP": "制动",
}

var _top := {}     # key -> StatusCard
var _data := {}    # key -> DataCard
var _sys := {}     # key -> Label (系统状态 kv)
var _conn := {}    # key -> Label (机器人连接 kv)
var _signal_bars: Array = []  # 5 ColorRects (信号强度)
var _log: DashPanel
var _alarm: DashPanel
var _clock: Label
var _nav_state: Label
var _nav_progress_bar: ProgressBar
var _nav_progress_pct: Label
var _distance_gauge: Control
var _contact_chart: Control
var _risk_hint: Label
var _risk_detail: Label
var _action_hint: Label
var _push_btn: Button
var _rotate_btn: Button
var _stop_btn: Button
var _device_debug_dialog: AcceptDialog
var _device_debug_text: RichTextLabel
var _device_debug_data := {}
var _uptime_ms := 0
var _last_alarm := ""
var _estopped := false
var _smooth_wall_mm := -1.0
var _smooth_curv_per_mm := -1.0
var _smooth_deviation_mm := -1.0


func _ready() -> void:
	_uptime_ms = Time.get_ticks_msec()
	var root := Control.new()
	root.set_anchors_and_offsets_preset(Control.PRESET_FULL_RECT)
	root.mouse_filter = Control.MOUSE_FILTER_IGNORE
	root.theme = UiStyle.theme()
	add_child(root)

	_build_top(root)
	_build_data(root)
	_build_rl_training(root)
	_build_replay(root)
	_build_bottom(root)


func _process(_delta: float) -> void:
	if _clock:
		_clock.text = Time.get_time_string_from_system()
	if _sys.has("uptime"):
		var s := int((Time.get_ticks_msec() - _uptime_ms) / 1000.0)
		_sys["uptime"].text = "%02d:%02d:%02d" % [s / 3600, (s / 60) % 60, s % 60]


# ── 顶部状态栏 (§3/§4): 状态卡片 + 线性图标 + 圆环进度 + 紧急停止 ───────────────
func _build_top(root: Control) -> void:
	var bar := PanelContainer.new()
	bar.add_theme_stylebox_override("panel", UiStyle.panel_box(0.9, 10))
	UiStyle.place(bar, UiStyle.top_rect())
	root.add_child(bar)

	var row := HBoxContainer.new()
	row.add_theme_constant_override("separation", 4)
	bar.add_child(row)

	_top["robot"] = _card(row, "机器人状态", "未连接", "", UiStyle.RED, "robot")
	row.add_child(_sep())
	_top["mode"] = _card(row, "导航模式", "手动", "", UiStyle.BLUE, "compass")
	row.add_child(_sep())
	# 路径进度: 圆环进度 (§4 "不要只显示文字").
	var ring := CircularProgress.new()
	ring.ring_color = UiStyle.GREEN
	_top["progress"] = StatusCard.new("路径进度", "0", "%", UiStyle.GREEN, ring)
	_top["progress"].custom_minimum_size = Vector2(112, 0)
	row.add_child(_top["progress"])
	row.add_child(_sep())
	_top["remain"] = _card(row, "剩余距离", "—", "cm", UiStyle.BLUE, "path")
	row.add_child(_sep())
	row.add_child(_mode_segment())
	row.add_child(_sep())
	_top["radius"] = _card(row, "血管半径", "—", "mm", UiStyle.BLUE, "radius")
	row.add_child(_sep())
	_top["curv"] = _card(row, "曲率", "0.00", "1/mm", UiStyle.BLUE, "curve")
	row.add_child(_sep())
	_top["dwall"] = _card(row, "距血管壁距离", "0.0", "mm", UiStyle.GREEN, "wall")
	row.add_child(_sep())
	_top["risk"] = _card(row, "风险等级", "正常", "", UiStyle.GREEN, "warning")
	row.add_child(_sep())

	# 紧急停止 (§4: 暗红底 #4A1C1D + #FF4D4F 边框/文字, 宽 170-190).
	var estop := UiStyle.button("紧急停止", UiStyle.ESTOP_BG, UiStyle.RED, UiStyle.RED, 18)
	estop.custom_minimum_size = Vector2(124, 74)
	estop.size_flags_horizontal = Control.SIZE_SHRINK_END
	estop.pressed.connect(_on_estop_pressed)
	row.add_child(estop)


func _card(row: HBoxContainer, title: String, value: String, unit: String,
		color: Color, icon_kind: String) -> StatusCard:
	var card := StatusCard.new(title, value, unit, color, StatusIcon.new(icon_kind, color))
	row.add_child(card)
	return card


func _sep() -> VSeparator:
	var s := VSeparator.new()
	var sb := StyleBoxFlat.new()
	sb.bg_color = Color(UiStyle.BORDER.r, UiStyle.BORDER.g, UiStyle.BORDER.b, 0.6)
	sb.content_margin_left = 1
	s.add_theme_stylebox_override("separator", sb)
	return s


func _mode_segment() -> PanelContainer:
	var panel := PanelContainer.new()
	panel.add_theme_stylebox_override("panel", UiStyle.card_box(0.75, 8))
	panel.custom_minimum_size = Vector2(318, 74)
	var row := HBoxContainer.new()
	row.add_theme_constant_override("separation", 0)
	panel.add_child(row)
	for item in [["HCI模式", true], ["RL训练", false], ["策略推理", false]]:
		var bg := UiStyle.BLUE_BG if item[1] else Color(0.016, 0.055, 0.086, 0.38)
		var border := UiStyle.BLUE if item[1] else Color(UiStyle.BORDER.r, UiStyle.BORDER.g, UiStyle.BORDER.b, 0.35)
		var fg := UiStyle.TEXT if item[1] else UiStyle.TEXT_MID
		var b := UiStyle.button(item[0], bg, border, fg, 18, 4)
		b.disabled = true
		b.custom_minimum_size = Vector2(102, 54)
		b.size_flags_horizontal = Control.SIZE_EXPAND_FILL
		row.add_child(b)
	return panel

# ── 导航与安全数据：进度 + 半圆仪表 + 接触力趋势 + 小指标 ─────────────────────
func _build_data(root: Control) -> void:
	var panel := PanelContainer.new()
	panel.add_theme_stylebox_override("panel", UiStyle.panel_box(0.92, 10))
	UiStyle.place(panel, UiStyle.data_rect())
	root.add_child(panel)

	var vb := VBoxContainer.new()
	vb.add_theme_constant_override("separation", 6)
	panel.add_child(vb)
	vb.add_child(UiStyle.label("④  导航与安全数据", UiStyle.TEXT, 16))

	var top := HBoxContainer.new()
	top.add_theme_constant_override("separation", 8)
	top.custom_minimum_size = Vector2(0, 80)
	vb.add_child(top)

	var progress_card := PanelContainer.new()
	progress_card.add_theme_stylebox_override("panel", UiStyle.card_box(0.78, 7))
	progress_card.size_flags_horizontal = Control.SIZE_EXPAND_FILL
	top.add_child(progress_card)
	var pv := VBoxContainer.new()
	pv.add_theme_constant_override("separation", 5)
	progress_card.add_child(pv)
	pv.add_child(UiStyle.label("导航进度", UiStyle.TEXT_MID, 13))
	var prow := HBoxContainer.new()
	prow.add_theme_constant_override("separation", 4)
	prow.add_child(UiStyle.label("78", UiStyle.TEXT, 25))
	prow.add_child(UiStyle.label("/ 234 mm", UiStyle.TEXT_MID, 14))
	pv.add_child(prow)
	_nav_progress_bar = ProgressBar.new()
	_nav_progress_bar.show_percentage = false
	_nav_progress_bar.min_value = 0
	_nav_progress_bar.max_value = 100
	_nav_progress_bar.value = 33
	_nav_progress_bar.custom_minimum_size = Vector2(0, 8)
	var bar_styles := UiStyle.bar_style(UiStyle.BLUE)
	_nav_progress_bar.add_theme_stylebox_override("background", bar_styles[0])
	_nav_progress_bar.add_theme_stylebox_override("fill", bar_styles[1])
	pv.add_child(_nav_progress_bar)
	_nav_progress_pct = UiStyle.label("33%", UiStyle.TEXT2, 12)
	pv.add_child(_nav_progress_pct)

	var gauge_card := PanelContainer.new()
	gauge_card.add_theme_stylebox_override("panel", UiStyle.card_box(0.78, 7))
	gauge_card.custom_minimum_size = Vector2(118, 0)
	top.add_child(gauge_card)
	_distance_gauge = SemiCircularGaugeScript.new()
	_distance_gauge.label = "156 mm"
	gauge_card.add_child(_distance_gauge)

	var chart_card := PanelContainer.new()
	chart_card.add_theme_stylebox_override("panel", UiStyle.card_box(0.78, 7))
	chart_card.custom_minimum_size = Vector2(176, 0)
	top.add_child(chart_card)
	var cv := VBoxContainer.new()
	cv.add_theme_constant_override("separation", 2)
	chart_card.add_child(cv)
	cv.add_child(UiStyle.label("接触力趋势 (N)", UiStyle.TEXT_MID, 13))
	_contact_chart = MiniTrendChartScript.new()
	_contact_chart.size_flags_vertical = Control.SIZE_EXPAND_FILL
	cv.add_child(_contact_chart)

	var grid := GridContainer.new()
	grid.columns = 4
	grid.add_theme_constant_override("h_separation", 8)
	grid.add_theme_constant_override("v_separation", 6)
	grid.size_flags_vertical = Control.SIZE_EXPAND_FILL
	vb.add_child(grid)

	_data["dpath"] = _dcard(grid, "偏离中心线", "2.3", "mm", UiStyle.BLUE)
	_data["curv"] = _dcard(grid, "局部曲率", "0.015", "1/mm", UiStyle.BLUE)
	_data["speed"] = _dcard(grid, "推进速度", "0.8", "mm/s", UiStyle.BLUE)
	_data["force"] = _dcard(grid, "接触力", "0.12", "N", UiStyle.BLUE)
	_data["dwall"] = _dcard(grid, "壁距", "1.8", "mm", UiStyle.GREEN)
	_data["risk"] = DataCard.new("风险等级", "中风险", "", UiStyle.YELLOW, 70.0, "请谨慎操作")
	grid.add_child(_data["risk"])
	_data["safety"] = DataCard.new("安全状态", "安全", "", UiStyle.GREEN, 86.0, "保护已启用")
	grid.add_child(_data["safety"])
	_data["eta"] = _dcard(grid, "预计到达", "—", "", UiStyle.BLUE)

func _dcard(grid: GridContainer, title: String, value: String, unit: String,
		color: Color) -> DataCard:
	var card := DataCard.new(title, value, unit, color)
	grid.add_child(card)
	return card



func _build_rl_training(root: Control) -> void:
	var panel := PanelContainer.new()
	panel.add_theme_stylebox_override("panel", UiStyle.panel_box(0.92, 10))
	UiStyle.place(panel, UiStyle.rl_rect())
	root.add_child(panel)
	var vb := VBoxContainer.new()
	vb.add_theme_constant_override("separation", 6)
	panel.add_child(vb)
	vb.add_child(UiStyle.label("⑤  RL训练状态", UiStyle.TEXT, 16))
	var top := HBoxContainer.new()
	top.add_theme_constant_override("separation", 8)
	vb.add_child(top)
	top.add_child(_mini_info("当前模式", "HCI模式", UiStyle.BLUE))
	top.add_child(_mini_info("当前策略", "VesselRL-v2.1", UiStyle.BLUE))
	var stats := GridContainer.new()
	stats.columns = 3
	stats.add_theme_constant_override("h_separation", 8)
	stats.add_theme_constant_override("v_separation", 8)
	vb.add_child(stats)
	stats.add_child(_mini_info("Episode", "128", UiStyle.TEXT))
	stats.add_child(_mini_info("Step", "32,455", UiStyle.TEXT))
	stats.add_child(_mini_info("Reward", "+12.36", UiStyle.GREEN))
	var lower := GridContainer.new()
	lower.columns = 4
	lower.add_theme_constant_override("h_separation", 8)
	lower.add_theme_constant_override("v_separation", 8)
	lower.size_flags_vertical = Control.SIZE_EXPAND_FILL
	vb.add_child(lower)
	lower.add_child(_mini_info("回放缓存", "48.5k", UiStyle.GREEN))
	lower.add_child(_mini_info("累积损失", "0.18", UiStyle.TEXT_MID))
	lower.add_child(_mini_info("熵", "1.24", UiStyle.TEXT_MID))
	lower.add_child(_mini_info("训练状态", "● 训练中", UiStyle.GREEN))


func _build_replay(root: Control) -> void:
	var panel := PanelContainer.new()
	panel.add_theme_stylebox_override("panel", UiStyle.panel_box(0.92, 10))
	UiStyle.place(panel, UiStyle.replay_rect())
	root.add_child(panel)
	var vb := VBoxContainer.new()
	vb.add_theme_constant_override("separation", 6)
	panel.add_child(vb)
	var head := HBoxContainer.new()
	head.add_child(UiStyle.label("⑥  数据采集与回放", UiStyle.TEXT, 16))
	var live := UiStyle.label("● 数据记录中", UiStyle.GREEN, 12)
	live.size_flags_horizontal = Control.SIZE_EXPAND_FILL
	live.horizontal_alignment = HORIZONTAL_ALIGNMENT_RIGHT
	head.add_child(live)
	vb.add_child(head)
	vb.add_child(_kv_row("数据记录状态", "进行中", UiStyle.GREEN))
	vb.add_child(_kv_row("数据集名称", "EndoRL-2024-05"))
	var nums := GridContainer.new()
	nums.columns = 2
	nums.add_theme_constant_override("h_separation", 8)
	nums.add_theme_constant_override("v_separation", 8)
	vb.add_child(nums)
	nums.add_child(_mini_info("样本数", "125,648", UiStyle.TEXT_MID))
	nums.add_child(_mini_info("轨迹数", "1,248", UiStyle.TEXT_MID))
	var controls := HBoxContainer.new()
	controls.add_theme_constant_override("separation", 6)
	vb.add_child(controls)
	for t in ["<<", "<", "||", ">", ">>", "1.0x"]:
		var b := _tool(t, func(): pass)
		b.custom_minimum_size = Vector2(42 if t == "1.0x" else 34, 24)
		controls.add_child(b)
	vb.add_child(_kv_row("标注状态", "已标注 73%", UiStyle.BLUE))
	var p := ProgressBar.new()
	p.show_percentage = false
	p.value = 73
	p.custom_minimum_size = Vector2(0, 7)
	var styles := UiStyle.bar_style(UiStyle.BLUE)
	p.add_theme_stylebox_override("background", styles[0])
	p.add_theme_stylebox_override("fill", styles[1])
	vb.add_child(p)
	vb.add_child(UiStyle.label("会话 ID  77f2ca21-5d6e-4b9a-a2d3", UiStyle.TEXT2, 11))


func _mini_info(title: String, value: String, color: Color) -> PanelContainer:
	var panel := PanelContainer.new()
	panel.add_theme_stylebox_override("panel", UiStyle.card_box(0.76, 7))
	panel.size_flags_horizontal = Control.SIZE_EXPAND_FILL
	var vb := VBoxContainer.new()
	vb.add_theme_constant_override("separation", 2)
	panel.add_child(vb)
	vb.add_child(UiStyle.label(title, UiStyle.TEXT2, 12))
	vb.add_child(UiStyle.label(value, color, 14))
	return panel


func _kv_row(key: String, value: String, color := Color(0.847, 0.902, 0.953)) -> HBoxContainer:
	var row := HBoxContainer.new()
	var k := UiStyle.label(key, UiStyle.TEXT2, 12)
	k.size_flags_horizontal = Control.SIZE_EXPAND_FILL
	row.add_child(k)
	row.add_child(UiStyle.label(value, color, 12))
	return row

# ── 底部控制区 (§14-§19): 五面板 17/17/27/21/18 ──────────────────────────────
func _build_bottom(root: Control) -> void:
	var row := HBoxContainer.new()
	UiStyle.place(row, UiStyle.bottom_rect())
	row.add_theme_constant_override("separation", UiStyle.GAP)
	root.add_child(row)

	# 1. 系统状态 (17%)
	var sysp := DashPanel.new("系统状态", 17.0)
	_clock = sysp.add_kv("系统时间", "--:--:--")
	_sys["uptime"] = sysp.add_kv("系统运行时间", "00:00:00")
	sysp.add_kv("软件版本", "v0.1 原型")
	_sys["coord"] = sysp.add_kv("导丝坐标", "—")
	_sys["view"] = sysp.add_kv("视角", "概览")
	_sys["model"] = sysp.add_kv("模型", "—")
	row.add_child(sysp)

	# 2. 机器人连接 (17%)
	var connp := DashPanel.new("机器人连接", 17.0)
	_conn["status"] = connp.add_kv("连接状态", "未连接", UiStyle.RED)
	_conn["latency"] = connp.add_kv("延迟", "— ms")
	_conn["engine"] = connp.add_kv("引擎", "-")
	_conn["mode"] = connp.add_kv("模式", "-")
	_conn["slack"] = connp.add_kv("堆积", "-")
	_conn["guidewire"] = connp.add_kv("导丝", "-")
	_conn["support"] = connp.add_kv("支撑", "-")
	_conn["buckling"] = connp.add_kv("屈曲", "-")
	_conn["wall_slide"] = connp.add_kv("贴壁", "-")
	# 信号强度: 5 格信号条 (§16).
	var sigrow := HBoxContainer.new()
	var sigt := UiStyle.label("信号强度", UiStyle.TEXT_MID, 12)
	sigt.size_flags_horizontal = Control.SIZE_EXPAND_FILL
	sigrow.add_child(sigt)
	var bars := HBoxContainer.new()
	bars.add_theme_constant_override("separation", 3)
	bars.alignment = BoxContainer.ALIGNMENT_END
	for i in 5:
		var seg := ColorRect.new()
		seg.custom_minimum_size = Vector2(6, 6 + i * 3)
		seg.size_flags_vertical = Control.SIZE_SHRINK_END
		seg.color = UiStyle.TRACK
		bars.add_child(seg)
		_signal_bars.append(seg)
	sigrow.add_child(bars)
	connp.content.add_child(sigrow)
	row.add_child(connp)

	# 3. 运动控制 (27%): 参考图的手动控制台，保留原 motion_command 信号入口。
	var motionp := DashPanel.new("运动控制", 27.0)
	var tabs := HBoxContainer.new()
	tabs.add_theme_constant_override("separation", 4)
	for item in [["手动控制", true], ["点击导航", false], ["策略推理", false]]:
		var b := UiStyle.button(item[0], UiStyle.BLUE_BG if item[1] else UiStyle.GRAY_BTN,
			UiStyle.BLUE if item[1] else UiStyle.BORDER, UiStyle.TEXT if item[1] else UiStyle.TEXT_MID, 12, 5)
		b.disabled = true
		b.custom_minimum_size = Vector2(100, 26)
		tabs.add_child(b)
	motionp.content.add_child(tabs)

	var control_grid := GridContainer.new()
	control_grid.columns = 4
	control_grid.add_theme_constant_override("h_separation", 8)
	control_grid.add_theme_constant_override("v_separation", 6)
	motionp.content.add_child(control_grid)

	_push_btn = _motion_value_card(control_grid, "推进 (Push)", "0.35", UiStyle.BLUE, func(): motion_command.emit(1.0, 0.0))
	var dial_card := PanelContainer.new()
	dial_card.add_theme_stylebox_override("panel", UiStyle.card_box(0.74, 7))
	dial_card.custom_minimum_size = Vector2(168, 132)
	var dial_v := VBoxContainer.new()
	dial_v.add_theme_constant_override("separation", 2)
	dial_card.add_child(dial_v)
	dial_v.add_child(UiStyle.label("实时意图方向", UiStyle.TEXT_MID, 12))
	dial_v.add_child(IntentDialScript.new())
	control_grid.add_child(dial_card)
	_rotate_btn = _motion_value_card(control_grid, "旋转 (Rotate)", "-0.25", UiStyle.BLUE, func(): motion_command.emit(0.0, 1.0))

	var preset_card := PanelContainer.new()
	preset_card.add_theme_stylebox_override("panel", UiStyle.card_box(0.74, 7))
	preset_card.custom_minimum_size = Vector2(172, 132)
	var preset_v := VBoxContainer.new()
	preset_v.add_theme_constant_override("separation", 6)
	preset_card.add_child(preset_v)
	preset_v.add_child(UiStyle.label("速度预设", UiStyle.TEXT_MID, 12))
	var speed_row := HBoxContainer.new()
	speed_row.add_theme_constant_override("separation", 4)
	for t in ["很慢", "慢", "中速", "快", "很快"]:
		speed_row.add_child(_tool(t, func(): pass))
	preset_v.add_child(speed_row)
	preset_v.add_child(UiStyle.label("角度步进设置", UiStyle.TEXT_MID, 12))
	var angle_row := HBoxContainer.new()
	angle_row.add_theme_constant_override("separation", 4)
	for t in ["-45°", "-15°", "0°", "+15°", "+45°"]:
		angle_row.add_child(_tool(t, func(): pass))
	preset_v.add_child(angle_row)
	control_grid.add_child(preset_card)

	var srow := HBoxContainer.new()
	srow.add_theme_constant_override("separation", 4)
	for t in ["J-tip辅助", "扭矩限制", "回撤保护", "自动停推"]:
		var sw := CheckBox.new()
		sw.text = t
		sw.button_pressed = true
		sw.add_theme_color_override("font_color", UiStyle.TEXT_MID)
		srow.add_child(sw)
	_stop_btn = UiStyle.button("停止", UiStyle.RED_BG, UiStyle.RED, UiStyle.RED, 12, 6)
	_stop_btn.custom_minimum_size = Vector2(68, 26)
	_stop_btn.pressed.connect(_on_estop_pressed)
	srow.add_child(_stop_btn)
	srow.add_child(_tool("恢复", func(): _on_resume_pressed()))
	srow.add_child(_tool("接管", func(): manual_takeover.emit()))
	srow.add_child(_tool("视角", func(): view_cycle_requested.emit()))
	srow.add_child(_tool("模型", func(): model_cycle_requested.emit()))
	srow.add_child(_tool("分支", func(): branch_cycle_requested.emit()))
	srow.add_child(_tool("重置", func(): reset_requested.emit()))
	srow.add_child(_tool("形变", func(): deform_toggle.emit()))
	srow.add_child(_tool("调试", func(): _show_device_debug()))
	motionp.content.add_child(srow)
	_nav_state = UiStyle.label("导航 手动 · Input p+0.0 r+0.0", UiStyle.TEXT2, 12)
	motionp.content.add_child(_nav_state)
	row.add_child(motionp)
	# 4. 系统日志 (21%)
	_log = DashPanel.new("系统日志", 21.0)
	var now := Time.get_time_string_from_system()
	_log.add_log(now, "系统初始化完成")
	_log.add_log(now, "等待后端连接")
	row.add_child(_log)

	# 5. 告警信息 (18%)
	_alarm = DashPanel.new("告警信息", 18.0)
	row.add_child(_alarm)


func _motion_value_card(grid: GridContainer, title: String, value: String, color: Color, cb: Callable) -> Button:
	var card := PanelContainer.new()
	card.add_theme_stylebox_override("panel", UiStyle.card_box(0.74, 7))
	card.custom_minimum_size = Vector2(128, 132)
	var vb := VBoxContainer.new()
	vb.add_theme_constant_override("separation", 6)
	card.add_child(vb)
	vb.add_child(UiStyle.label(title, UiStyle.TEXT_MID, 12))
	vb.add_child(UiStyle.label(value, color, 25))
	var up := _tool("▲", cb)
	var down := _tool("▼", cb)
	vb.add_child(up)
	vb.add_child(down)
	grid.add_child(card)
	return up
func _tool(text: String, cb: Callable) -> Button:
	var b := UiStyle.button(text, UiStyle.GRAY_BTN, UiStyle.BORDER, UiStyle.TEXT_MID, 12, 6)
	b.custom_minimum_size = Vector2(52, 24)
	b.pressed.connect(cb)
	return b


# ── 紧急停止 / 恢复 (§22 interaction states) ─────────────────────────────────
func _on_estop_pressed() -> void:
	_estopped = true
	_push_btn.disabled = true
	_rotate_btn.disabled = true
	_top["risk"].set_value("紧急停止")
	_top["risk"].set_color(UiStyle.RED)
	_alarm.add_alert(Time.get_time_string_from_system(), "紧急停止已触发", "danger")
	add_log_line("紧急停止")
	emergency_stop.emit()


func _on_resume_pressed() -> void:
	if _estopped:
		_estopped = false
		_push_btn.disabled = false
		_rotate_btn.disabled = false
		add_log_line("恢复导航")
	resume_nav.emit()


## Append a timestamped system-log line (kept to the latest 4).
func add_log_line(text: String) -> void:
	if _log:
		_log.add_log(Time.get_time_string_from_system(), text)


# ── Public API (consumed by main_controller) ────────────────────────────────
func set_connection(connected: bool) -> void:
	var txt := "已连接" if connected else "未连接"
	var col := UiStyle.GREEN if connected else UiStyle.RED
	_top["robot"].set_value(txt)
	_top["robot"].set_color(col)
	_conn["status"].text = txt
	_conn["status"].add_theme_color_override("font_color", col)
	_conn["latency"].text = "— ms"
	_conn["engine"].text = "-"
	_conn["mode"].text = "-"
	_conn["slack"].text = "-"
	if _conn.has("guidewire"):
		_conn["guidewire"].text = "-"
	if _conn.has("support"):
		_conn["support"].text = "-"
	if _conn.has("buckling"):
		_conn["buckling"].text = "-"
	for i in _signal_bars.size():
		_signal_bars[i].color = (UiStyle.GREEN if connected and i < 4 else UiStyle.TRACK)
	add_log_line("后端已连接" if connected else "后端连接断开")


func update_safety(status: String) -> void:
	var text: String = STATUS_TEXT.get(status, status)
	var stop := status == "COLLISION_STOP"
	var warn := status == "DANGER_WARNING"
	# Alert card only on a status transition, so it does not spam every frame.
	if status != _last_alarm and (stop or warn):
		_last_alarm = status
		_alarm.add_alert(Time.get_time_string_from_system(),
			"距血管壁较近，请注意操作" if warn else "已触发制动保护",
			"warning" if warn else "danger")
	elif not (stop or warn):
		if _last_alarm in ["DANGER_WARNING", "COLLISION_STOP"]:
			_alarm.add_alert(Time.get_time_string_from_system(), "恢复安全导航", "success")
		_last_alarm = status


func set_control_mode(text: String) -> void:
	var col := UiStyle.BLUE
	var short_text := "手动"
	if "自动" in text or "AUTO" in text:
		short_text = "自动"
		col = UiStyle.GREEN
	elif "STOP" in text or "HOLD" in text or "安全保持" in text:
		short_text = "安全保持"
		col = UiStyle.YELLOW
	_top["mode"].set_value(short_text)
	_top["mode"].set_color(col)


func set_backend(engine: String, mode: String, diagnostics: Dictionary) -> void:
	if _conn.has("engine") and engine != "":
		_conn["engine"].text = engine
	if _conn.has("mode") and mode != "":
		_conn["mode"].text = mode
	if _conn.has("slack"):
		if diagnostics.has("slack_m"):
			var slack_mm := float(diagnostics.get("slack_m", 0.0)) * 1000.0
			var budget_mm := float(diagnostics.get("feed_budget_m", 0.0)) * 1000.0
			_conn["slack"].text = "%.1f / %.1f mm" % [slack_mm, budget_mm]
		else:
			_conn["slack"].text = "-"


func set_device_state(guidewire: Dictionary, support: Dictionary, risk: Dictionary, procedure: Dictionary = {}) -> void:
	_device_debug_data = {
		"guidewire": guidewire.duplicate(true),
		"procedure": procedure.duplicate(true),
		"support": support.duplicate(true),
		"risk": risk.duplicate(true),
	}
	_refresh_device_debug()
	if _conn.has("guidewire"):
		var tip_shape := str(guidewire.get("tip_shape_label", _cn_tip_shape(str(guidewire.get("tip_shape", "-")))))
		var segment := str(guidewire.get("current_tip_segment_label", _cn_segment(str(guidewire.get("current_tip_segment", "-")))))
		var torsion: Variant = guidewire.get("torsion_lag_deg", null)
		if typeof(torsion) == TYPE_FLOAT or typeof(torsion) == TYPE_INT:
			_conn["guidewire"].text = "%s / %s 扭滞%.0f°" % [tip_shape, segment, float(torsion)]
		else:
			_conn["guidewire"].text = "%s / %s" % [tip_shape, segment]
	if _conn.has("support"):
		var support_type := str(support.get("effective_support_type_label", _cn_support_type(str(support.get("effective_support_type", "-")))))
		var free_len: Variant = support.get("free_wire_length_mm", null)
		if typeof(free_len) == TYPE_FLOAT or typeof(free_len) == TYPE_INT:
			_conn["support"].text = "%s %.1f mm" % [support_type, float(free_len)]
		else:
			_conn["support"].text = support_type
	if _conn.has("buckling"):
		var buckling := str(risk.get("buckling_risk_text", _cn_buckling(str(risk.get("buckling_risk", "UNKNOWN")))))
		var pile: Variant = risk.get("pile_ratio", null)
		if typeof(pile) == TYPE_FLOAT or typeof(pile) == TYPE_INT:
			_conn["buckling"].text = "%s %.0f%%" % [buckling, float(pile) * 100.0]
		else:
			_conn["buckling"].text = buckling
	if _conn.has("wall_slide"):
		var wall_state := str(risk.get("wall_slide_state_text", _cn_wall_slide(str(risk.get("wall_slide_state", "UNKNOWN")))))
		var normal_score: Variant = risk.get("normal_poking_score", null)
		var slide_score: Variant = risk.get("tangential_slide_score", null)
		if (typeof(normal_score) == TYPE_FLOAT or typeof(normal_score) == TYPE_INT) and (typeof(slide_score) == TYPE_FLOAT or typeof(slide_score) == TYPE_INT):
			_conn["wall_slide"].text = "%s 顶%.0f%% 滑%.0f%%" % [wall_state, float(normal_score) * 100.0, float(slide_score) * 100.0]
		else:
			_conn["wall_slide"].text = wall_state

func _show_device_debug() -> void:
	_ensure_device_debug_dialog()
	_refresh_device_debug()
	_device_debug_dialog.popup_centered(Vector2i(620, 520))


func _ensure_device_debug_dialog() -> void:
	if _device_debug_dialog != null:
		return
	_device_debug_dialog = AcceptDialog.new()
	_device_debug_dialog.title = "器械与术式调试数据"
	_device_debug_dialog.exclusive = false
	add_child(_device_debug_dialog)

	var scroll := ScrollContainer.new()
	scroll.custom_minimum_size = Vector2(580, 420)
	_device_debug_dialog.add_child(scroll)

	_device_debug_text = RichTextLabel.new()
	_device_debug_text.fit_content = true
	_device_debug_text.scroll_active = false
	_device_debug_text.selection_enabled = true
	_device_debug_text.custom_minimum_size = Vector2(560, 400)
	scroll.add_child(_device_debug_text)


func _refresh_device_debug() -> void:
	if _device_debug_text == null:
		return
	var guidewire: Dictionary = _device_debug_data.get("guidewire", {}) as Dictionary
	var procedure: Dictionary = _device_debug_data.get("procedure", {}) as Dictionary
	var support: Dictionary = _device_debug_data.get("support", {}) as Dictionary
	var risk: Dictionary = _device_debug_data.get("risk", {}) as Dictionary
	var lines: Array[String] = []
	lines.append("[Procedure]")
	_debug_line(lines, "name", procedure.get("name", "unknown"), "术式 preset")
	_debug_line(lines, "display_name_zh", procedure.get("display_name_zh", "未知"), "术式名称")
	_debug_line(lines, "procedure_type", procedure.get("procedure_type", "unknown"), "术式类型")
	_debug_line(lines, "access_site", procedure.get("access_site", "unknown"), "入路血管")
	_debug_line(lines, "access_site_label", procedure.get("access_site_label", "未知"), "入路血管中文")
	_debug_line(lines, "access_route_label", procedure.get("access_route_label", "未知"), "入路方式")
	_debug_line(lines, "needle_entry_label", procedure.get("needle_entry_label", "未知"), "进针位置")
	_debug_line(lines, "guidewire_summary", procedure.get("guidewire_summary", "未知"), "导丝摘要")
	_debug_line(lines, "support_stack", procedure.get("support_stack", []), "支撑器械组合")
	_debug_line(lines, "support_stack_label", procedure.get("support_stack_label", "未知"), "支撑器械中文")
	lines.append("")
	lines.append("[GuidewireDesign]")
	_debug_line(lines, "design_name", guidewire.get("design_name", "unknown"), "导丝 preset")
	_debug_line(lines, "display_name_zh", guidewire.get("display_name_zh", "未知"), "导丝中文名")
	_debug_line(lines, "summary_zh", guidewire.get("summary_zh", "未知"), "器械摘要")
	_debug_line(lines, "diameter_inch", guidewire.get("diameter_inch", null), "导丝直径")
	_debug_line(lines, "clinical_total_length_mm", guidewire.get("clinical_total_length_mm", null), "临床总长")
	_debug_line(lines, "intravascular_length_mm", guidewire.get("intravascular_length_mm", null), "血管内渲染长度")
	_debug_line(lines, "external_tail_length_mm", guidewire.get("external_tail_length_mm", null), "体外剩余长度")
	_debug_line(lines, "active_sim_length_mm", guidewire.get("active_sim_length_mm", null), "物理求解长度")
	_debug_line(lines, "render_scope", guidewire.get("render_scope", "unknown"), "3D渲染范围")
	_debug_line(lines, "exchange_length", guidewire.get("exchange_length", null), "是否交换导丝")
	_debug_line(lines, "intended_use", guidewire.get("intended_use", []), "适用场景")
	_debug_line(lines, "compatible_support", guidewire.get("compatible_support", []), "兼容支撑")
	lines.append("")
	lines.append("[Runtime Device]")
	_debug_line(lines, "tip_shape", guidewire.get("tip_shape", "unknown"), "头端形状")
	_debug_line(lines, "current_tip_segment", guidewire.get("current_tip_segment", "unknown"), "当前导丝分段")
	_debug_line(lines, "torsion_lag_deg", guidewire.get("torsion_lag_deg", null), "扭转滞后")
	_debug_line(lines, "effective_support_type", support.get("effective_support_type", "unknown"), "当前支撑器械")
	_debug_line(lines, "free_wire_length_mm", support.get("free_wire_length_mm", null), "游离导丝长度")
	_debug_line(lines, "buckling_risk", risk.get("buckling_risk", "unknown"), "屈曲风险")
	_debug_line(lines, "wall_slide_state", risk.get("wall_slide_state", "unknown"), "贴壁状态")
	_device_debug_text.text = "\n".join(lines)


func _debug_line(lines: Array[String], key: String, value: Variant, label: String = "") -> void:
	var name := key if label == "" else "%s (%s)" % [label, key]
	lines.append("%s: %s" % [name, _format_debug_value(value)])


func _format_debug_value(value: Variant) -> String:
	if value == null:
		return "null"
	if value is Array:
		var parts: Array[String] = []
		for item in value:
			parts.append(str(item))
		return ", ".join(parts)
	return str(value)


func _cn_tip_shape(value: String) -> String:
	match value:
		"j_tip": return "J尖"
		"straight": return "直头"
		_: return value


func _cn_segment(value: String) -> String:
	match value:
		"atraumatic_tip": return "无创头端"
		"pre_shaped_soft_tip": return "预塑形软头"
		"distal_soft": return "远端软段"
		"transition": return "过渡段"
		"proximal_shaft": return "近端杆身"
		_: return value


func _cn_support_type(value: String) -> String:
	match value:
		"introducer_sheath": return "导入鞘"
		"guiding_catheter": return "导引导管"
		"intermediate_catheter": return "中间导管"
		"microcatheter": return "微导管"
		"none": return "无支撑"
		_: return value


func _cn_buckling(value: String) -> String:
	match value:
		"LOW": return "低"
		"MEDIUM": return "中"
		"HIGH": return "高"
		"UNKNOWN": return "未知"
		_: return value


func _cn_wall_slide(value: String) -> String:
	match value:
		"WALL_SLIDE_OK": return "贴壁滑入"
		"TIP_POKING_WARNING": return "顶壁风险"
		"FREE_CENTERED": return "居中未贴壁"
		"SAFE_NAV": return "安全导航"
		"DANGER_WARNING": return "风险预警"
		"COLLISION_STOP": return "碰撞制动"
		"STANDBY": return "待机"
		"UNKNOWN": return "未知"
		_: return value

func update_metrics(metrics: Dictionary) -> void:
	var raw_wall_mm := float(metrics.get("wall_distance", 0.0)) * 1000.0
	var raw_deviation_mm := float(metrics.get("path_deviation", 0.0)) * 1000.0
	var remaining_cm := float(metrics.get("remaining_distance", 0.0)) * 100.0
	var radius_value = metrics.get("vessel_radius", null)
	var radius_mm := -1.0
	if radius_value != null:
		radius_mm = float(radius_value) * 1000.0
	var raw_curv_per_mm := float(metrics.get("curvature", 0.0)) / 1000.0
	var wall_mm := _smooth_value("_smooth_wall_mm", raw_wall_mm, 0.18)
	var deviation_mm := _smooth_value("_smooth_deviation_mm", raw_deviation_mm, 0.18)
	var curv_per_mm := _smooth_value("_smooth_curv_per_mm", raw_curv_per_mm, 0.16)
	var speed := float(metrics.get("velocity", 0.0)) * 1000.0  # m/s -> mm/s
	var progress := float(metrics.get("path_progress", 0.0)) * 100.0
	var risk_score := float(metrics.get("risk_score", 0.0))
	var force_n := float(metrics.get("contact_force", risk_score * 0.8))
	var latency := float(metrics.get("latency_ms", -1.0))

	_top["progress"].set_value("%.0f" % progress)
	_top["progress"].set_ring(progress)
	_top["remain"].set_value("%.1f" % remaining_cm)
	_top["radius"].set_value(_format_optional(radius_mm, 1))
	_top["curv"].set_value("%.4f" % curv_per_mm)
	_top["dwall"].set_value("%.1f" % wall_mm)

	_set_data_value("dwall", "%.1f" % wall_mm)
	_set_data_bar("dwall", clampf(wall_mm / 3.0 * 100.0, 0.0, 100.0))
	_set_data_value("dpath", "%.1f" % deviation_mm)
	_set_data_bar("dpath", clampf((1.5 - deviation_mm) / 1.5 * 100.0, 0.0, 100.0))
	_set_data_value("radius", _format_optional(radius_mm, 1))
	_set_data_value("curv", "%.4f" % curv_per_mm)
	_set_data_value("speed", "%.1f" % speed)
	_set_data_value("force", "%.2f" % force_n)
	_set_data_value("progress", "%.0f" % progress)
	_set_data_bar("progress", progress)
	_set_data_value("eta", _format_eta(metrics.get("eta_seconds", null)))
	if _nav_progress_bar:
		_nav_progress_bar.value = clampf(progress, 0.0, 100.0)
	if _nav_progress_pct:
		_nav_progress_pct.text = "%.0f%%" % clampf(progress, 0.0, 100.0)
	if _distance_gauge:
		_distance_gauge.value = 1.0 - clampf(progress / 100.0, 0.0, 1.0)
		_distance_gauge.label = "%.0f mm" % maxf(0.0, remaining_cm * 10.0)
	if latency >= 0.0 and _conn.has("latency"):
		_conn["latency"].text = "%.0f ms" % latency

	_update_risk(wall_mm, risk_score)


func _set_data_value(key: String, value: String) -> void:
	if _data.has(key):
		_data[key].set_value(value)


func _set_data_bar(key: String, value: float) -> void:
	if _data.has(key):
		_data[key].set_bar(value)


func _set_data_color(key: String, color: Color) -> void:
	if _data.has(key):
		_data[key].set_color(color)

func _smooth_value(slot: String, raw: float, alpha: float) -> float:
	if slot == "_smooth_wall_mm":
		_smooth_wall_mm = raw if _smooth_wall_mm < 0.0 else lerpf(_smooth_wall_mm, raw, alpha)
		return _smooth_wall_mm
	if slot == "_smooth_deviation_mm":
		_smooth_deviation_mm = raw if _smooth_deviation_mm < 0.0 else lerpf(_smooth_deviation_mm, raw, alpha)
		return _smooth_deviation_mm
	_smooth_curv_per_mm = raw if _smooth_curv_per_mm < 0.0 else lerpf(_smooth_curv_per_mm, raw, alpha)
	return _smooth_curv_per_mm


func _format_optional(value: float, decimals: int) -> String:
	if value < 0.0 or is_nan(value) or is_inf(value):
		return "—"
	return ("%." + str(decimals) + "f") % value


## Risk rule (doc/11 §23): by wall distance —— >1.5mm 安全绿 / 0.8-1.5 中等黄 / <0.8 高危红.
func _update_risk(wall_mm: float, risk_score: float = 0.0) -> void:
	if _estopped:
		return  # estop display holds until 恢复
	var color: Color
	var text: String
	if risk_score >= 0.75:
		color = UiStyle.RED; text = "高风险"
	elif risk_score >= 0.35:
		color = UiStyle.YELLOW; text = "中等"
	elif wall_mm > 1.5:
		color = UiStyle.GREEN; text = "正常"
	elif wall_mm >= 0.8:
		color = UiStyle.YELLOW; text = "中等"
	else:
		color = UiStyle.RED; text = "高风险"
	_top["risk"].set_value(text)
	_top["risk"].set_color(color)
	_top["dwall"].set_color(color)
	_set_data_value("risk", text if text == "正常" else text + "风险")
	_set_data_color("risk", color)
	_set_data_color("dwall", color)
	if _data.has("safety"):
		_data["safety"].set_value("安全" if color == UiStyle.GREEN else "关注" if color == UiStyle.YELLOW else "危险")
		_data["safety"].set_color(UiStyle.GREEN if color == UiStyle.GREEN else color)
	if _risk_hint:
		_risk_hint.text = "距离血管壁距离过小" if color == UiStyle.RED else ("路径偏差需要关注" if color == UiStyle.YELLOW else "当前导航稳定")
		_risk_hint.add_theme_color_override("font_color", color)
	if _risk_detail:
		_risk_detail.text = "请减速并调整位置" if color == UiStyle.RED else ("建议缓慢推进并观察壁距" if color == UiStyle.YELLOW else "维持当前操作节奏")
	if _action_hint:
		_action_hint.text = "立即停止推进\n微调导管方向\n等待风险解除" if color == UiStyle.RED else ("降低推进速度\n微调导管方向\n保持路径居中" if color == UiStyle.YELLOW else "保持路径居中\n按计划推进\n继续监测壁距")

func _format_eta(value) -> String:
	if value == null:
		return "—"
	var seconds := float(value)
	if seconds < 0.0 or is_inf(seconds) or is_nan(seconds):
		return "—"
	var total := int(round(seconds))
	var minutes := total / 60
	var secs := total % 60
	return "%02d:%02d" % [minutes, secs]


func update_input(push: float, rotate: float) -> void:
	if _nav_state:
		var parts := _nav_state.text.split(" · ")
		var nav := parts[0] if parts.size() > 0 else "导航 手动"
		_nav_state.text = "%s · Input p%+0.1f r%+0.1f" % [nav, push, rotate]


func set_coord(pos: Vector3) -> void:
	if _sys.has("coord"):
		_sys["coord"].text = "(%.2f, %.2f, %.2f)" % [pos.x, pos.y, pos.z]


func set_view_mode(mode_name: String) -> void:
	if _sys.has("view"):
		_sys["view"].text = mode_name


func set_model(model_name: String) -> void:
	if _sys.has("model"):
		_sys["model"].text = model_name


func set_nav(text: String, active: bool = true) -> void:
	if _nav_state:
		var parts := _nav_state.text.split(" · ")
		var inp := parts[1] if parts.size() > 1 else "Input p+0.0 r+0.0"
		_nav_state.text = "导航 %s · %s" % [text, inp]
		_nav_state.add_theme_color_override("font_color", UiStyle.BLUE if active else UiStyle.TEXT2)


# Diagnostics are no longer shown on-screen; kept as a no-op so main_controller's
# frequent set_debug calls stay valid.
func set_debug(_text: String) -> void:
	pass
