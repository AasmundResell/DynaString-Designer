#version 330 core
out vec4 frag_color;

in vec3 frag_pos;
in vec3 vertex_color;
in vec3 normal;

uniform vec3 light_pos;
uniform vec3 view_pos;
uniform vec3 light_color;

void main() {

    // ambient
    float ambient_strength = 0.6;
    vec3 ambient = ambient_strength * light_color;

    // diffuse
    vec3 norm = normalize(normal);
    vec3 light_dir = normalize(light_pos - frag_pos);
    float diff = max(dot(norm, light_dir), 0.0);
    vec3 diffuse = diff * light_color;

    // specular
    float specular_strength = 0.6;
    vec3 view_dir = normalize(view_pos - frag_pos);
    vec3 reflect_dir = reflect(-light_dir, norm);
    float spec = pow(max(dot(view_dir, reflect_dir), 0.0), 32);
    vec3 specular = specular_strength * spec * light_color;

    vec3 result = (ambient + diffuse + specular) * vertex_color;
    frag_color = vec4(result, 1.0);
}