#version 330 core
in vec3 vWorldPos;
in vec3 vNormal;
in vec4 vAlbedo;
in vec2 vMaterial;
uniform vec3 uLightDir;
uniform vec3 uCameraPos;
uniform vec3 uAmbient;
out vec4 fragColor;

float DistributionGGX(vec3 N, vec3 H, float roughness) {
  float a = roughness * roughness;
  float a2 = a * a;
  float NdotH = max(dot(N, H), 0.0);
  float denom = NdotH * NdotH * (a2 - 1.0) + 1.0;
  return a2 / max(3.14159265 * denom * denom, 1e-4);
}

float GeometrySchlickGGX(float NdotV, float roughness) {
  float r = roughness + 1.0;
  float k = (r * r) / 8.0;
  return NdotV / max(NdotV * (1.0 - k) + k, 1e-4);
}

float GeometrySmith(vec3 N, vec3 V, vec3 L, float roughness) {
  float NdotV = max(dot(N, V), 0.0);
  float NdotL = max(dot(N, L), 0.0);
  return GeometrySchlickGGX(NdotV, roughness) *
         GeometrySchlickGGX(NdotL, roughness);
}

vec3 FresnelSchlick(float cosTheta, vec3 F0) {
  return F0 + (1.0 - F0) * pow(1.0 - cosTheta, 5.0);
}

void main() {
  vec3 N = normalize(vNormal);
  vec3 V = normalize(uCameraPos - vWorldPos);
  vec3 L = normalize(-uLightDir);
  vec3 H = normalize(V + L);
  vec3 albedo = vAlbedo.rgb;
  float alpha = vAlbedo.a;
  float metallic = vMaterial.x;
  float roughness = vMaterial.y;
  vec3 F0 = mix(vec3(0.04), albedo, metallic);
  float NDF = DistributionGGX(N, H, roughness);
  float G = GeometrySmith(N, V, L, roughness);
  vec3 F = FresnelSchlick(max(dot(H, V), 0.0), F0);
  vec3 specular = (NDF * G * F) /
                  max(4.0 * max(dot(N, V), 0.0) * max(dot(N, L), 0.0), 1e-4);
  vec3 kD = (vec3(1.0) - F) * (1.0 - metallic);
  vec3 diffuse = kD * albedo / 3.14159265;
  vec3 color = (diffuse + specular) * max(dot(N, L), 0.0) + uAmbient * albedo;
  fragColor = vec4(color, alpha);
}
