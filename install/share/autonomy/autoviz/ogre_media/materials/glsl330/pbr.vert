#version 330 core
layout(location = 0) in vec3 aPos;
layout(location = 1) in vec3 aNormal;
layout(location = 2) in vec4 aAlbedo;
layout(location = 3) in vec2 aMaterial;
uniform mat4 uMvp;
out vec3 vWorldPos;
out vec3 vNormal;
out vec4 vAlbedo;
out vec2 vMaterial;
void main() {
  vWorldPos = aPos;
  vNormal = aNormal;
  vAlbedo = aAlbedo;
  vMaterial = aMaterial;
  gl_Position = uMvp * vec4(aPos, 1.0);
}
