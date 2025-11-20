
export const PipeVertexShader = `
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

export const PipeFragmentShader = `
precision mediump float;

varying vec3 v_normal;
varying vec3 v_frag_pos;
varying vec3 v_color;

uniform vec3 light_dir;
uniform vec3 view_pos;
uniform vec3 light_color;
uniform vec3 ambient_color;
uniform float specular_strength;

void main() {
    vec3 norm = normalize(v_normal);

    // For a headlight defined by camera.front (rays from camera into scene)
    // the direction FROM surface TOWARDS the light is -light_dir
    vec3 L = normalize(-light_dir);

    float diff = max(dot(norm, L), 0.0);
    vec3 diffuse = diff * light_color;

    // Specular (Blinn-Phong)
    vec3 view_dir = normalize(view_pos - v_frag_pos);
    vec3 half_vec = normalize(L + view_dir);
    float spec = pow(max(dot(norm, half_vec), 0.0), 32.0);
    vec3 specular = specular_strength * spec * light_color;

    vec3 ambient = ambient_color * light_color;
    vec3 result = (ambient + diffuse + specular) * v_color;
    gl_FragColor = vec4(result, 1.0);
}
`;
