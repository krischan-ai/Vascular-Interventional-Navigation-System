class_name EndoscopeMaterialFactory
extends RefCounted
## Builds the private endoscope wall material without changing vessel geometry.
##
## The source VPP mesh has normals but no UV channel, so every texture uses UV1
## world triplanar projection.  The enhanced profile separates macro colour,
## mesoscopic roughness and micro-normal scales; none of them displaces vertices.

const PROFILE_BASELINE := "baseline"
const PROFILE_ENHANCED := "enhanced"
const QUALITY_PERFORMANCE := "performance"
const QUALITY_BALANCED := "balanced"
const QUALITY_HIGH := "high"

const BASE_ALBEDO := Color(1.0, 0.82, 0.70)
const BASE_EMISSION := Color(0.20, 0.035, 0.018)
const EndoscopeWallShader := preload(
	"res://scripts/rendering/endoscope_wall.gdshader")


static func create_material(profile: String, quality: String) -> Material:
	if profile != PROFILE_BASELINE:
		return _create_enhanced_material(quality)
	var material := StandardMaterial3D.new()
	material.albedo_color = BASE_ALBEDO
	material.cull_mode = BaseMaterial3D.CULL_DISABLED
	material.metallic = 0.0
	material.emission_enabled = true
	material.emission = BASE_EMISSION
	material.texture_filter = BaseMaterial3D.TEXTURE_FILTER_LINEAR_WITH_MIPMAPS_ANISOTROPIC
	_apply_baseline(material)
	return material


static func apply_brightness(material: Material, value: float) -> void:
	if material == null:
		return
	var gain := clampf(value, 0.55, 1.35)
	if material is StandardMaterial3D:
		var standard := material as StandardMaterial3D
		standard.albedo_color = Color(
			BASE_ALBEDO.r * gain,
			BASE_ALBEDO.g * gain,
			BASE_ALBEDO.b * gain,
			1.0)
		var emission_base := float(
			standard.get_meta("scope_emission_base", 0.16))
		standard.emission_energy_multiplier = emission_base * gain
	elif material is ShaderMaterial:
		var shader_material := material as ShaderMaterial
		shader_material.set_shader_parameter("albedo_gain", gain)
		shader_material.set_shader_parameter("emission_gain", 0.025 * gain)


static func _apply_baseline(material: StandardMaterial3D) -> void:
	material.roughness = 0.72
	material.emission_energy_multiplier = 0.16
	material.set_meta("scope_emission_base", 0.16)
	var ramp := _gradient(
		PackedFloat32Array([0.0, 0.34, 0.66, 1.0]),
		PackedColorArray([
			Color(0.34, 0.055, 0.025),
			Color(0.66, 0.16, 0.075),
			Color(0.90, 0.32, 0.16),
			Color(0.48, 0.075, 0.035),
		]))
	material.albedo_texture = _noise_texture(512, 9.2, 5, ramp)
	_apply_triplanar(material, 70.0)


static func _create_enhanced_material(quality: String) -> ShaderMaterial:
	var texture_size := 512
	var normal_strength := 0.18
	var clearcoat_strength := 0.18
	var clearcoat_roughness := 0.28
	match quality:
		QUALITY_PERFORMANCE:
			texture_size = 256
			normal_strength = 0.14
			clearcoat_strength = 0.12
			clearcoat_roughness = 0.32
		QUALITY_HIGH:
			texture_size = 768
			normal_strength = 0.24
			clearcoat_strength = 0.23
			clearcoat_roughness = 0.24

	# Fractal octaves add restrained mesoscopic colour variation to a slow macro
	# field. The ramp deliberately avoids lesion-like high-contrast spots.
	var color_ramp := _gradient(
		PackedFloat32Array([0.0, 0.30, 0.58, 0.82, 1.0]),
		PackedColorArray([
			Color(0.38, 0.055, 0.022),
			Color(0.62, 0.135, 0.052),
			Color(0.84, 0.270, 0.110),
			Color(0.98, 0.410, 0.190),
			Color(0.54, 0.090, 0.034),
		]))
	var albedo_texture := _noise_texture(
		texture_size, 9.2, 5, color_ramp)

	# A separate lower-contrast field modulates roughness from approximately
	# 0.42–0.72 after multiplication by the material's 0.72 base value.
	var roughness_ramp := _gradient(
		PackedFloat32Array([0.0, 0.42, 0.72, 1.0]),
		PackedColorArray([
			Color(0.58, 0.58, 0.58),
			Color(0.72, 0.72, 0.72),
			Color(0.88, 0.88, 0.88),
			Color(1.0, 1.0, 1.0),
		]))
	var roughness_texture := _noise_texture(
		texture_size, 8.0, 4, roughness_ramp)

	# NoiseTexture2D generates a normal map from a high-frequency height field.
	# Mipmaps and a deliberately small normal scale limit shimmer in motion.
	var normal_texture := _noise_texture(
		texture_size, 28.0, 3, null, true, normal_strength)
	var material := ShaderMaterial.new()
	material.shader = EndoscopeWallShader
	material.set_shader_parameter("albedo_texture", albedo_texture)
	material.set_shader_parameter("roughness_texture", roughness_texture)
	material.set_shader_parameter("micro_normal_texture", normal_texture)
	material.set_shader_parameter("triplanar_scale", 70.0)
	material.set_shader_parameter("triplanar_sharpness", 2.5)
	material.set_shader_parameter("normal_strength", normal_strength)
	material.set_shader_parameter("clearcoat_strength", clearcoat_strength)
	material.set_shader_parameter("clearcoat_roughness", clearcoat_roughness)
	material.set_shader_parameter("albedo_gain", 0.92)
	material.set_shader_parameter("emission_gain", 0.025 * 0.92)
	return material


static func _apply_triplanar(material: StandardMaterial3D, scale: float) -> void:
	material.uv1_triplanar = true
	material.uv1_world_triplanar = true
	material.uv1_triplanar_sharpness = 2.5
	material.uv1_scale = Vector3.ONE * scale


static func _noise_texture(
		size: int,
		cycles: float,
		octaves: int,
		ramp: Gradient = null,
		as_normal: bool = false,
		bump_strength: float = 0.25) -> NoiseTexture2D:
	var noise := FastNoiseLite.new()
	noise.frequency = cycles / float(size)
	noise.fractal_octaves = octaves
	noise.fractal_gain = 0.48
	noise.fractal_lacunarity = 2.0
	var texture := NoiseTexture2D.new()
	texture.width = size
	texture.height = size
	texture.generate_mipmaps = true
	texture.seamless = true
	if ramp != null:
		texture.color_ramp = ramp
	if as_normal:
		texture.as_normal_map = true
		texture.bump_strength = bump_strength
	# Assign noise last. NoiseTexture2D starts generation asynchronously when the
	# source is attached; setting the colour ramp afterwards can leave the first
	# generated image in single-channel R8 form (the historical pure-red scope).
	texture.noise = noise
	return texture


static func _gradient(offsets: PackedFloat32Array, colors: PackedColorArray) -> Gradient:
	var ramp := Gradient.new()
	ramp.offsets = offsets
	ramp.colors = colors
	return ramp
