#version 120
attribute vec4 vertex;
attribute vec3 normal;
attribute vec4 colour;
attribute vec4 uv0;
attribute vec4 uv1;
uniform mat4 worldviewproj_matrix;
varying vec3 vWorldPos;
varying vec3 vNormal;
varying vec2 vUv;
varying vec4 vTint;
varying vec2 vMaterial;
void main() {
  vWorldPos = vertex.xyz;
  vNormal = normal;
  vUv = uv0.xy;
  vTint = colour;
  vMaterial = uv1.xy;
  gl_Position = worldviewproj_matrix * vec4(vertex.xyz, 1.0);
}
