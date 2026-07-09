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
var _push_btn: Button
var _rotate_btn: Button
var _stop_btn: Button
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
	row.add_theme_constant_override("separation", 6)
	bar.add_child(row)

	_top["robot"] = _card(row, "机器人状态", "未连接", "", UiStyle.RED, "robot")
	row.add_child(_sep())
	_top["mode"] = _card(row, "导航模式", "手动", "", UiStyle.BLUE, "compass")
	row.add_child(_sep())
	# 路径进度: 圆环进度 (§4 "不要只显示文字").
	var ring := CircularProgress.new()
	ring.ring_color = UiStyle.GREEN
	_top["progress"] = StatusCard.new("路径进度", "0", "%", UiStyle.GREEN, ring)
	row.add_child(_top["progress"])
	row.add_child(_sep())
	_top["remain"] = _card(row, "剩余距离", "—", "cm", UiStyle.BLUE, "path")
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
	estop.custom_minimum_size = Vector2(150, 74)
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


# ── 导航与安全数据 (§12/§13): 2x4 DataCards ──────────────────────────────────
func _build_data(root: Control) -> void:
	var panel := PanelContainer.new()
	panel.add_theme_stylebox_override("panel", UiStyle.panel_box(0.92, 10))
	UiStyle.place(panel, UiStyle.data_rect())
	root.add_child(panel)

	var vb := VBoxContainer.new()
	vb.add_theme_constant_override("separation", 8)
	panel.add_child(vb)
	vb.add_child(UiStyle.label("导航与安全数据", UiStyle.TEXT, 16))

	var grid := GridContainer.new()
	grid.columns = 4
	grid.add_theme_constant_override("h_separation", 10)
	grid.add_theme_constant_override("v_separation", 10)
	grid.size_flags_vertical = Control.SIZE_EXPAND_FILL
	vb.add_child(grid)

	_data["dwall"] = _dcard(grid, "距血管壁距离", "1.6", "mm", UiStyle.GREEN)
	_data["dpath"] = _dcard(grid, "路径偏差", "—", "mm", UiStyle.GREEN)
	_data["radius"] = _dcard(grid, "血管半径", "—", "mm", UiStyle.BLUE)
	_data["curv"] = _dcard(grid, "曲率", "0.42", "1/mm", UiStyle.BLUE)
	_data["speed"] = _dcard(grid, "导管速度", "3.2", "mm/s", UiStyle.BLUE)
	_data["progress"] = _dcard(grid, "路径进度", "72", "%", UiStyle.BLUE)
	_data["eta"] = _dcard(grid, "预计到达目标", "—", "", UiStyle.BLUE)
	# 卡8 风险状态 with 提示行 (§13).
	var risk := DataCard.new("风险状态", "正常", "", UiStyle.GREEN, 30.0, "请保持谨慎操作")
	grid.add_child(risk)
	_data["risk"] = risk


func _dcard(grid: GridContainer, title: String, value: String, unit: String,
		color: Color) -> DataCard:
	var card := DataCard.new(title, value, unit, color)
	grid.add_child(card)
	return card


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
	_conn["engine"] = connp.add_kv("Engine", "-")
	_conn["mode"] = connp.add_kv("Mode", "-")
	_conn["slack"] = connp.add_kv("Slack", "-")
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

	# 3. 运动控制 (27%): 大按钮 暗底+彩色描边 (§17), 急停后禁用推进/旋转 (§22).
	var motionp := DashPanel.new("运动控制", 27.0)
	var mrow := HBoxContainer.new()
	mrow.add_theme_constant_override("separation", 10)
	_push_btn = UiStyle.button("推进", UiStyle.BLUE_BG, UiStyle.BLUE, UiStyle.TEXT, 16)
	_push_btn.custom_minimum_size = Vector2(130, 62)
	_push_btn.pressed.connect(func(): motion_command.emit(1.0, 0.0))
	mrow.add_child(_push_btn)
	_rotate_btn = UiStyle.button("旋转", UiStyle.GRAY_BTN, UiStyle.GRAY_BORDER, UiStyle.TEXT, 16)
	_rotate_btn.custom_minimum_size = Vector2(130, 62)
	_rotate_btn.pressed.connect(func(): motion_command.emit(0.0, 1.0))
	mrow.add_child(_rotate_btn)
	_stop_btn = UiStyle.button("停止", UiStyle.RED_BG, UiStyle.RED, UiStyle.RED, 16)
	_stop_btn.custom_minimum_size = Vector2(130, 62)
	_stop_btn.pressed.connect(_on_estop_pressed)
	mrow.add_child(_stop_btn)
	motionp.content.add_child(mrow)
	# Secondary tools + live nav/input readout.
	var srow := HBoxContainer.new()
	srow.add_theme_constant_override("separation", 6)
	srow.add_child(_tool("恢复", func(): _on_resume_pressed()))
	srow.add_child(_tool("接管", func(): manual_takeover.emit()))
	srow.add_child(_tool("视角", func(): view_cycle_requested.emit()))
	srow.add_child(_tool("模型", func(): model_cycle_requested.emit()))
	srow.add_child(_tool("分支", func(): branch_cycle_requested.emit()))
	srow.add_child(_tool("重置", func(): reset_requested.emit()))
	srow.add_child(_tool("形变", func(): deform_toggle.emit()))
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
	var latency := float(metrics.get("latency_ms", -1.0))

	_top["progress"].set_value("%.0f" % progress)
	_top["progress"].set_ring(progress)
	_top["remain"].set_value("%.1f" % remaining_cm)
	_top["radius"].set_value(_format_optional(radius_mm, 1))
	_top["curv"].set_value("%.4f" % curv_per_mm)
	_top["dwall"].set_value("%.1f" % wall_mm)

	_data["dwall"].set_value("%.1f" % wall_mm)
	_data["dwall"].set_bar(clampf(wall_mm / 3.0 * 100.0, 0.0, 100.0))
	_data["dpath"].set_value("%.1f" % deviation_mm)
	_data["dpath"].set_bar(clampf((1.5 - deviation_mm) / 1.5 * 100.0, 0.0, 100.0))
	_data["radius"].set_value(_format_optional(radius_mm, 1))
	_data["curv"].set_value("%.4f" % curv_per_mm)
	_data["speed"].set_value("%.1f" % speed)
	_data["progress"].set_value("%.0f" % progress)
	_data["progress"].set_bar(progress)
	_data["eta"].set_value(_format_eta(metrics.get("eta_seconds", null)))
	if latency >= 0.0 and _conn.has("latency"):
		_conn["latency"].text = "%.0f ms" % latency

	_update_risk(wall_mm, risk_score)


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
	_data["risk"].set_value(text if text == "正常" else text + "风险")
	_data["risk"].set_color(color)
	_data["dwall"].set_color(color)


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
