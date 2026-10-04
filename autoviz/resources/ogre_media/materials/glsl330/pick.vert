#version 330 core
layout(location = 0) in vec3 aPos;
layout(location = 1) in vec3 aPickColor;
uniform mat4 uMvp;
uniform float uPointSize;
out vec3 vPickColor;
void main() {
  gl_Position = uMvp * vec4(aPos, 1.0);
  gl_PointSize = uPointSize;
  vPickColor = aPickColor;
}
