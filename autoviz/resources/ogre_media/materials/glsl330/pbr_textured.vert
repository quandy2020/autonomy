#version 330 core
layout(location = 0) in vec3 aPos;
layout(location = 1) in vec3 aNormal;
layout(location = 2) in vec2 aUv;
layout(location = 3) in vec4 aTint;
layout(location = 4) in vec2 aMaterial;
uniform mat4 uMvp;
out vec3 vWorldPos;
out vec3 vNormal;
out vec2 vUv;
out vec4 vTint;
out vec2 vMaterial;
void main() {
  vWorldPos = aPos;
  vNormal = aNormal;
  vUv = aUv;
  vTint = aTint;
  vMaterial = aMaterial;
  gl_Position = uMvp * vec4(aPos, 1.0);
}
