#version 120
varying vec3 vWorldPos;
varying vec3 vNormal;
varying vec4 vAlbedo;
varying vec2 vMaterial;
uniform vec3 uLightDir;
uniform vec3 uCameraPos;
uniform vec3 uAmbient;

float DistributionGGX(vec3 N, vec3 H, float roughness) {
  float a = roughness * roughness;
  float a2 = a * a;
  float NdotH = max(dot(N, H), 0.0);
  float NdotH2 = NdotH * NdotH;
  float denom = NdotH2 * (a2 - 1.0) + 1.0;
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
  return F0 + (1.0 - F0) * pow(clamp(1.0 - cosTheta, 0.0, 1.0), 5.0);
}

void main() {
  vec3 N = normalize(vNormal);
  vec3 V = normalize(uCameraPos - vWorldPos);
  vec3 L = normalize(-uLightDir);
  vec3 H = normalize(V + L);
  vec3 albedo = vAlbedo.rgb;
  float alpha = vAlbedo.a;
  float metallic = vMaterial.x;
  float roughness = clamp(vMaterial.y, 0.04, 1.0);
  vec3 F0 = mix(vec3(0.04), albedo, metallic);

  float NDF = DistributionGGX(N, H, roughness);
  float G = GeometrySmith(N, V, L, roughness);
  vec3 F = FresnelSchlick(max(dot(H, V), 0.0), F0);
  vec3 numerator = NDF * G * F;
  float denom = 4.0 * max(dot(N, V), 0.0) * max(dot(N, L), 0.0) + 1e-4;
  vec3 specular = numerator / denom;

  vec3 kS = F;
  vec3 kD = (vec3(1.0) - kS) * (1.0 - metallic);
  float NdotL = max(dot(N, L), 0.0);
  vec3 diffuse = kD * albedo / 3.14159265;
  vec3 radiance = vec3(1.0) * NdotL;
  vec3 color = (diffuse + specular) * radiance + uAmbient * albedo;
  gl_FragColor = vec4(color, alpha);
}
