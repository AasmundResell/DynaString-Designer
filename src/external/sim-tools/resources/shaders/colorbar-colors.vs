#version 330 core
layout (location = 0) in vec2 a_pos;
layout (location = 1) in vec3 a_color;

out vec3 vertex_color;
uniform mat4 projection;

void main()
{    
    vertex_color = a_color;

    vec2 ndc_pos = 2.0 * a_pos - 1.0; 
    gl_Position = projection * vec4(a_pos, 0.0, 1.0);
}