#version 330 core
layout(location = 0) in vec3 a_pos;
layout(location = 1) in vec3 a_normal;

layout(location = 2) in vec4 model_col0; // Instance data: Model matrix column 0
layout(location = 3) in vec4 model_col1; // Instance data: Model matrix column 1
layout(location = 4) in vec4 model_col2; // Instance data: Model matrix column 2
layout(location = 5) in vec4 model_col3; // Instance data: Model matrix column 3

layout(location = 6) in vec3 color; // Instance data: color

out vec3 frag_pos;
out vec3 vertex_color;
out vec3 normal;

// uniform vec3 color;
uniform mat4 view;
uniform mat4 projection;
// uniform mat4 model; // REMOVE

void main() {
    mat4 model = mat4(model_col0, model_col1, model_col2, model_col3);

    // model = mat4(1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0);

    // Must transform normal to account for non uniform scaling
    mat3 normal_matrix = transpose(inverse(mat3(model)));
    normal = normalize(normal_matrix * a_normal);

    vertex_color = color; // color
    frag_pos = a_pos;

    gl_Position = projection * view * model * vec4(frag_pos, 1.0);
}