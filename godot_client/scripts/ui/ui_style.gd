class_name UiStyle
extends RefCounted
## Single source of truth for the navigation dashboard's dark medical-industrial style
## and its 1920x1080 layout (doc/11 前端设计方案). The project stretches on a 1920x1080
## logical canvas (project.godot: stretch/mode=canvas_items, aspect=keep), so absolute
## pixel rects here are stable regardless of the real window size. Every UI script pulls
## colors, style boxes and region rects from here so the look stays consistent and DRY.

# ── Palette (doc/11 §21 颜色参数总表) ─────────────────────────────────────────
const BG := Color(0.027, 0.063, 0.098)       # 071019 主背景
const BG2 := Color(0.043, 0.067, 0.102)      # 0B111A 深色背景
const PANEL := Color(0.063, 0.102, 0.149)    # 101A26 面板
const PANEL2 := Color(0.055, 0.094, 0.141)   # 0E1824 次级面板
const BORDER := Color(0.145, 0.220, 0.290)   # 25384A 边框
const TRACK := Color(0.149, 0.224, 0.302)    # 26394D 分隔线/进度条背景
const BLUE := Color(0.184, 0.549, 1.0)       # 2F8CFF 强调蓝
const GREEN := Color(0.306, 0.902, 0.420)    # 4EE66B 安全绿
const YELLOW := Color(1.0, 0.831, 0.278)     # FFD447 警告黄
const RED := Color(1.0, 0.302, 0.310)        # FF4D4F 危险红
const TEXT := Color(0.847, 0.902, 0.953)     # D8E6F3 主文字
const TEXT_MID := Color(0.722, 0.780, 0.839) # B8C7D6 次级文字
const TEXT2 := Color(0.498, 0.561, 0.639)    # 7F8FA3 弱文字
const RED_BG := Color(0.165, 0.067, 0.078)   # 2A1114 红色背景
const ESTOP_BG := Color(0.290, 0.110, 0.114) # 4A1C1D 紧急停止背景
const YELLOW_BG := Color(0.227, 0.180, 0.0)  # 3A2E00 黄色背景
const GREEN_BG := Color(0.055, 0.165, 0.094) # 0E2A18 绿色背景
const BLUE_BG := Color(0.071, 0.227, 0.439)  # 123A70 蓝色按钮背景
const GRAY_BTN := Color(0.102, 0.141, 0.188) # 1A2430 旋转按钮背景
const GRAY_BORDER := Color(0.290, 0.365, 0.439) # 4A5D70
const PANE3D_BG := Color(0.020, 0.043, 0.071)   # 050B12 3D 场景背景
const TEXT_DISABLED := Color(0.353, 0.400, 0.451) # 5A6673

const MARGIN := 12
const GAP := 12

# ── Layout regions (1920x1080 logical canvas, doc/11 §2) ─────────────────────
# Top 100px, middle 742px, bottom 190px; DSA 59% | right 41%.
# Right column: 3D 420px (56.6%, spec 53-58%) over 导航与安全数据 310px.
static func top_rect() -> Rect2: return Rect2(12, 12, 1896, 100)
static func dsa_rect() -> Rect2: return Rect2(12, 124, 1112, 742)
static func pane3d_rect() -> Rect2: return Rect2(1136, 124, 772, 420)
static func data_rect() -> Rect2: return Rect2(1136, 556, 772, 310)
static func bottom_rect() -> Rect2: return Rect2(12, 878, 1896, 190)


## Pin a Control to an absolute logical-pixel rect (anchors at 0).
static func place(c: Control, r: Rect2) -> void:
	c.anchor_left = 0.0
	c.anchor_top = 0.0
	c.anchor_right = 0.0
	c.anchor_bottom = 0.0
	c.offset_left = r.position.x
	c.offset_top = r.position.y
	c.offset_right = r.position.x + r.size.x
	c.offset_bottom = r.position.y + r.size.y


# ── Font (doc/11 §20: Noto Sans SC / Microsoft YaHei / 思源黑体) ──────────────
static var _font: SystemFont


static func font() -> SystemFont:
	if _font == null:
		_font = SystemFont.new()
		_font.font_names = PackedStringArray(["Microsoft YaHei", "Noto Sans SC", "思源黑体 CN"])
	return _font


## A shared Theme carrying the default font + panel styles; set on UI layer roots.
static func theme() -> Theme:
	var t := Theme.new()
	t.default_font = font()
	t.default_font_size = 14
	t.set_stylebox("panel", "PanelContainer", panel_box())
	t.set_stylebox("background", "ProgressBar", flat(TRACK, 3))
	t.set_stylebox("fill", "ProgressBar", flat(BLUE, 3))
	return t


# ── Style boxes ──────────────────────────────────────────────────────────────
static func flat(color: Color, radius := 8) -> StyleBoxFlat:
	var sb := StyleBoxFlat.new()
	sb.bg_color = color
	sb.set_corner_radius_all(radius)
	return sb


static func panel_box(alpha := 0.92, radius := 10) -> StyleBoxFlat:
	var c := PANEL
	c.a = alpha
	var sb := flat(c, radius)
	sb.border_color = BORDER
	sb.set_border_width_all(1)
	sb.content_margin_left = 12
	sb.content_margin_right = 12
	sb.content_margin_top = 10
	sb.content_margin_bottom = 10
	return sb


static func card_box(alpha := 0.9, radius := 8) -> StyleBoxFlat:
	var c := PANEL
	c.a = alpha
	var sb := flat(c, radius)
	sb.border_color = BORDER
	sb.set_border_width_all(1)
	sb.content_margin_left = 12
	sb.content_margin_right = 12
	sb.content_margin_top = 8
	sb.content_margin_bottom = 8
	return sb


## Filled box with a colored border (buttons / alert cards per doc/11 §17/§19).
static func bordered_box(bg: Color, border: Color, radius := 8, border_w := 1) -> StyleBoxFlat:
	var sb := flat(bg, radius)
	sb.border_color = border
	sb.set_border_width_all(border_w)
	sb.content_margin_left = 10
	sb.content_margin_right = 10
	sb.content_margin_top = 6
	sb.content_margin_bottom = 6
	return sb


static func bar_style(fill: Color) -> Array:
	# [background, fill] styleboxes for a thin ProgressBar (§13: track #26394D).
	return [flat(TRACK, 3), flat(fill, 3)]


# ── Labels / buttons ─────────────────────────────────────────────────────────
static func label(text: String, color: Color, size: int) -> Label:
	var l := Label.new()
	l.text = text
	l.add_theme_font_override("font", font())
	l.add_theme_font_size_override("font_size", size)
	l.add_theme_color_override("font_color", color)
	return l


## Bordered button (doc/11 §17/§22: dark bg + accent border; hover +10%, pressed -10%,
## disabled 40% alpha + #5A6673 text).
static func button(text: String, bg: Color, border: Color, fg: Color, size := 14,
		radius := 8) -> Button:
	var b := Button.new()
	b.text = text
	b.add_theme_font_override("font", font())
	b.add_theme_font_size_override("font_size", size)
	b.add_theme_color_override("font_color", fg)
	b.add_theme_color_override("font_hover_color", fg)
	b.add_theme_color_override("font_pressed_color", fg)
	b.add_theme_color_override("font_disabled_color", TEXT_DISABLED)
	b.add_theme_stylebox_override("normal", bordered_box(bg, border, radius))
	b.add_theme_stylebox_override("hover", bordered_box(bg.lightened(0.10), border, radius))
	b.add_theme_stylebox_override("pressed", bordered_box(bg.darkened(0.10), border, radius))
	var dis := bordered_box(Color(bg.r, bg.g, bg.b, 0.4), Color(border.r, border.g, border.b, 0.4), radius)
	b.add_theme_stylebox_override("disabled", dis)
	return b
