#version 330 core

out vec4 frag_color;
vec3 line_color = vec3(0.0, 0.0, 0.0); // black color

void main()
{
    frag_color = vec4(line_color, 1.0f);
}