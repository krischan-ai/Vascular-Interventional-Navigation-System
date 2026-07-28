class_name EndoscopeLightingRig
extends Node3D
## Camera-local near-axis illumination for the private endoscope world.
##
## The key light stays close to the optical axis.  A weaker light offset by
## 3.5 mm reveals the real mesh normals without inventing shadows or geometry.

const PROFILE_BASELINE := "baseline"
const QUALITY_PERFORMANCE := "performance"
const QUALITY_BALANCED := "balanced"
const QUALITY_HIGH := "high"
const DEFAULT_RADIUS_M := 0.003

var key_light: OmniLight3D
var fill_light: OmniLight3D
var _profile := "enhanced"
var _quality := QUALITY_BALANCED
var _brightness := 0.92
var _smoothed_radius := DEFAULT_RADIUS_M


func configure(layer_mask: int, profile: String, quality: String) -> void:
	_profile = profile
	_quality = quality
	key_light = _make_light(
		"ScopeKeyLight", Color(1.0, 0.55, 0.36), layer_mask)
	key_light.position = Vector3(0.0, -0.001, -0.002)
	add_child(key_light)
	fill_light = _make_light(
		"ScopeFillLight", Color(1.0, 0.38, 0.23), layer_mask)
	fill_light.position = Vector3(0.0035, -0.0025, -0.001)
	add_child(fill_light)
	_apply_light_state()


func set_brightness(value: float) -> void:
	_brightness = clampf(value, 0.55, 1.35)
	_apply_light_state()


func update_radius(radius_m: float, delta: float) -> void:
	var target := radius_m if radius_m > 0.0005 else DEFAULT_RADIUS_M
	target = clampf(target, 0.0015, 0.008)
	var blend := 1.0 - exp(-8.0 * maxf(delta, 0.0))
	_smoothed_radius = lerpf(_smoothed_radius, target, blend)
	_apply_light_state()


func get_smoothed_radius() -> float:
	return _smoothed_radius


func _apply_light_state() -> void:
	if key_light == null or fill_light == null:
		return
	if _profile == PROFILE_BASELINE:
		key_light.light_energy = 3.2 * _brightness
		key_light.omni_range = 0.095
		key_light.omni_attenuation = 1.35
		fill_light.light_energy = 0.0
		fill_light.visible = false
		return
	var radius_t := clampf(
		inverse_lerp(0.0015, 0.006, _smoothed_radius), 0.0, 1.0)
	key_light.omni_range = clampf(_smoothed_radius * 14.0, 0.055, 0.12)
	key_light.omni_attenuation = lerpf(1.18, 1.38, radius_t)
	var key_base := lerpf(3.80, 4.40, radius_t)
	var fill_ratio := 0.22
	match _quality:
		QUALITY_PERFORMANCE:
			fill_ratio = 0.16
		QUALITY_HIGH:
			fill_ratio = 0.27
	key_light.light_energy = key_base * _brightness
	fill_light.light_energy = key_base * fill_ratio * _brightness
	fill_light.omni_range = key_light.omni_range * 0.90
	fill_light.omni_attenuation = 1.55
	fill_light.visible = true


func _make_light(name_value: String, color: Color, layer_mask: int) -> OmniLight3D:
	var light := OmniLight3D.new()
	light.name = name_value
	light.light_color = color
	light.light_cull_mask = layer_mask
	light.shadow_enabled = false
	return light
