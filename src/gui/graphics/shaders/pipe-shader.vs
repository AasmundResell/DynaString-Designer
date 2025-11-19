#version 330 core
layout(location = 0) in vec3 a_pos;
layout(location = 1) in vec3 a_normal;
layout(location = 2) in vec3 a_color;

out vec3 v_normal;
out vec3 v_frag_pos;
out vec3 v_color;

uniform mat4 model;
uniform mat4 view;
uniform mat4 projection;

void main() {

    v_color = a_color;
    v_frag_pos = a_pos;
    v_normal = a_normal;

    gl_Position = projection * view * model * vec4(v_frag_pos, 1.0);
}