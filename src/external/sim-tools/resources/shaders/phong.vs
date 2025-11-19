#version 330 core
layout(location = 0) in vec3 a_pos;
layout(location = 1) in vec3 a_color;
layout(location = 2) in vec3 a_normal;

out vec3 frag_pos;
out vec3 vertex_color;
out vec3 normal;

uniform mat4 view;
uniform mat4 projection;
uniform mat4 model;
uniform mat3 normal_matrix;

void main() {

    vertex_color = a_color;
    frag_pos = a_pos;
    normal = normal_matrix * a_normal;

    gl_Position = projection * view * model * vec4(frag_pos, 1.0);
}