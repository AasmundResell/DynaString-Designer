#version 330 core
out vec4 frag_color;

in vec3 v_normal;
in vec3 v_frag_pos;
in vec3 v_color;

uniform vec3 light_pos;
uniform vec3 view_pos;
uniform vec3 light_color;
uniform vec4 object_color;
uniform vec3 ambient_color;
uniform float specular_strength;

void main() {

    // Calculate lighting
    vec3 ambient = ambient_color * light_color;

    // Diffuse lighting
    vec3 norm = normalize(v_normal);

    vec3 light_dir = normalize(light_pos - v_frag_pos);
    float diff = max(dot(norm, light_dir), 0.0);
    vec3 diffuse = diff * light_color;

    // Specular lighting
    vec3 view_dir = normalize(view_pos - v_frag_pos);
    vec3 reflect_dir = reflect(-light_dir, norm);
    float spec = pow(max(dot(view_dir, reflect_dir), 0.0), 32);
    vec3 specular = specular_strength * spec * light_color;

    // Set base color and alpha first
    vec3 base_color;
    float alpha = object_color.a;

    base_color = object_color.rgb;
    alpha *= min(0.8, (1.0 - abs(dot(norm, view_dir))));

    // Final color
    vec3 result = (ambient + diffuse + specular) * base_color;
    frag_color = vec4(result, alpha);
}
