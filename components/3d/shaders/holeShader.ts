export const HoleVertexShader = `
attribute vec3 color;
varying vec3 v_normal;
varying vec3 v_frag_pos;
varying vec3 v_color;

void main() {
    v_color = color;
    v_frag_pos = vec3(modelViewMatrix * vec4(position, 1.0));
    v_normal = normalMatrix * normal;
    
    gl_Position = projectionMatrix * modelViewMatrix * vec4(position, 1.0);
}
`;

export const HoleFragmentShader = `
precision mediump float;

varying vec3 v_normal;
varying vec3 v_frag_pos;
varying vec3 v_color;

uniform vec3 light_dir;
uniform vec3 view_pos;
uniform vec3 light_color;
uniform vec3 ambient_color;
uniform float specular_strength;
uniform vec3 object_color;
uniform float alpha;

void main() {
    vec3 norm = normalize(v_normal);
    vec3 L = normalize(-light_dir);

    float diff = max(dot(norm, L), 0.0);
    vec3 diffuse = diff * light_color;

    vec3 view_dir = normalize(view_pos - v_frag_pos);
    vec3 half_vec = normalize(L + view_dir);
    float spec = pow(max(dot(norm, half_vec), 0.0), 32.0);
    vec3 specular = specular_strength * spec * light_color;

    vec3 ambient = ambient_color * light_color;
    vec3 result = (ambient + diffuse + specular) * object_color;
    gl_FragColor = vec4(result, alpha);
}
`;
