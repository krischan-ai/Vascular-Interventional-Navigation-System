shader_type spatial;
render_mode unshaded, cull_front, depth_draw_opaque;

uniform sampler2D albedo_texture : source_color, filter_linear_mipmap_anisotropic, repeat_enable;
uniform sampler2D roughness_texture : filter_linear_mipmap_anisotropic, repeat_enable;
uniform sampler2D micro_normal_texture : hint_normal, filter_linear_mipmap_anisotropic, repeat_enable;
uniform float triplanar_scale = 70.0;
uniform float triplanar_sharpness = 2.5;
uniform float albedo_gain = 0.92;
uniform float emission_gain = 0.025;
uniform vec3 emission_color : source_color = vec3(0.18, 0.052, 0.022);
uniform float normal_strength = 0.18;
uniform float roughness_min = 0.42;
uniform float roughness_max = 0.72;
uniform float clearcoat_strength = 0.18;
uniform float clearcoat_roughness = 0.28;

varying vec3 world_position;
varying vec3 world_normal;


void vertex() {
	world_position = (MODEL_MATRIX * vec4(VERTEX, 1.0)).xyz;
	world_normal = normalize(MODEL_NORMAL_MATRIX * NORMAL);
}


vec3 triplanar_weights(vec3 normal_value) {
	vec3 weights = pow(abs(normal_value), vec3(triplanar_sharpness));
	return weights / max(weights.x + weights.y + weights.z, 0.0001);
}


vec4 sample_triplanar(sampler2D texture_value, vec3 position_value, vec3 normal_value) {
	vec3 weights = triplanar_weights(normal_value);
	vec4 x_sample = texture(texture_value, position_value.yz);
	vec4 y_sample = texture(texture_value, position_value.xz);
	vec4 z_sample = texture(texture_value, position_value.xy);
	return x_sample * weights.x + y_sample * weights.y + z_sample * weights.z;
}


void fragment() {
	// The GLB surface normals point away from the vessel. The camera sees the
	// back faces from inside, so use an explicit inward normal for lighting.
	vec3 inward_world = -normalize(world_normal);
	vec3 inward_view = normalize((VIEW_MATRIX * vec4(inward_world, 0.0)).xyz);
	vec3 sample_position = world_position * triplanar_scale;
	vec4 wall_color = sample_triplanar(
		albedo_texture, sample_position, inward_world);
	float roughness_field = sample_triplanar(
		roughness_texture, sample_position, inward_world).r;
	vec2 micro_xy = sample_triplanar(
		micro_normal_texture, sample_position, inward_world).rg * 2.0 - 1.0;

	vec3 reference_up_world = abs(inward_world.y) < 0.95
		? vec3(0.0, 1.0, 0.0)
		: vec3(1.0, 0.0, 0.0);
	vec3 tangent_world = normalize(cross(reference_up_world, inward_world));
	vec3 bitangent_world = normalize(cross(inward_world, tangent_world));
	vec3 surface_normal = normalize(
		inward_world
		+ tangent_world * micro_xy.x * normal_strength
		+ bitangent_world * micro_xy.y * normal_strength);

	vec3 view_direction = normalize(CAMERA_POSITION_WORLD - world_position);
	vec3 camera_right = normalize(INV_VIEW_MATRIX[0].xyz);
	vec3 camera_up = normalize(INV_VIEW_MATRIX[1].xyz);
	vec3 fill_direction = normalize(
		view_direction + camera_right * 0.18 + camera_up * 0.10);
	float key_term = pow(max(dot(surface_normal, view_direction), 0.0), 0.72);
	float fill_term = pow(max(dot(surface_normal, fill_direction), 0.0), 0.85);
	float roughness_value = mix(
		roughness_min, roughness_max, roughness_field);
	vec3 half_vector = normalize(view_direction + fill_direction);
	float wet_specular = pow(
		max(dot(surface_normal, half_vector), 0.0),
		mix(16.0, 48.0, 1.0 - roughness_value));

	float illumination = 0.27 + key_term * 0.82 + fill_term * 0.22;
	vec3 diffuse_color = wall_color.rgb * vec3(1.0, 0.96, 0.90)
		* illumination * albedo_gain;
	vec3 wet_highlight = vec3(1.0, 0.50, 0.30) * wet_specular
		* clearcoat_strength * 0.65;
	ALBEDO = diffuse_color + wet_highlight
		+ emission_color * emission_gain;
}
