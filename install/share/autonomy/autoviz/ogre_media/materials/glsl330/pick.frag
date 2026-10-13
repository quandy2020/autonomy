#version 330 core
in vec3 vPickColor;
out vec4 fragColor;
void main() { fragColor = vec4(vPickColor, 1.0); }
