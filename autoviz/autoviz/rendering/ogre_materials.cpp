/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#include "autoviz/rendering/ogre_materials.hpp"

#ifdef AUTOVIZ_USE_OGRE

#include <Ogre.h>

#include <cmath>

namespace autoviz {
namespace rendering {
namespace {

constexpr char kPointSpriteTex[] = "AvizPointDiscTex";
constexpr char kPointSpriteMat[] = "AvizPointSprite";
constexpr char kPbrMat[] = "AvizPBR";
constexpr char kPbrVert[] = "AvizPBRVert";
constexpr char kPbrFrag[] = "AvizPBRFrag";

constexpr char kPbrVertSource[] = R"GLSL(
#version 330 core
in vec4 position;
in vec3 normal;
in vec4 diffuse;
in vec2 texcoord;
uniform mat4 worldviewproj_matrix;
out vec3 vWorldPos;
out vec3 vNormal;
out vec4 vAlbedo;
out vec2 vMaterial;
void main() {
  vWorldPos = position.xyz;
  vNormal = normal;
  vAlbedo = diffuse;
  vMaterial = texcoord;
  gl_Position = worldviewproj_matrix * vec4(position.xyz, 1.0);
}
)GLSL";

constexpr char kPbrFragSource[] = R"GLSL(
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
  return F0 + (1.0 - F0) * pow(1.0 - cosTheta, 5.0);
}

void main() {
  vec3 N = normalize(vNormal);
  vec3 V = normalize(uCameraPos - vWorldPos);
  vec3 L = normalize(-uLightDir);
  vec3 H = normalize(V + L);
  vec3 albedo = vAlbedo.rgb;
  float alpha = vAlbedo.a;
  float metallic = vMaterial.x();
  float roughness = vMaterial.y();
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
  fragColor = vec4(color, alpha);
}
)GLSL";

void CreatePointDiscTexture() {
  if (Ogre::TextureManager::getSingleton().resourceExists(kPointSpriteTex)) {
    return;
  }
  constexpr int kSize = 64;
  std::vector<Ogre::uint8> pixels(static_cast<std::size_t>(kSize * kSize * 4));
  const float center = (kSize - 1) * 0.5f;
  const float radius = center - 1.f;
  for (int y = 0; y < kSize; ++y) {
    for (int x = 0; x < kSize; ++x) {
      const float dx = static_cast<float>(x) - center;
      const float dy = static_cast<float>(y) - center;
      const float dist = std::sqrt(dx * dx + dy * dy);
      float alpha = 1.f;
      if (dist > radius) {
        alpha = 0.f;
      } else if (dist > radius - 1.5f) {
        alpha = (radius - dist) / 1.5f;
      }
      const std::size_t idx =
          static_cast<std::size_t>((y * kSize + x) * 4);
      pixels[idx + 0] = 255;
      pixels[idx + 1] = 255;
      pixels[idx + 2] = 255;
      pixels[idx + 3] = static_cast<Ogre::uint8>(alpha * 255.f);
    }
  }
  Ogre::TexturePtr texture =
      Ogre::TextureManager::getSingleton().createManual(
          kPointSpriteTex,
          Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME,
          Ogre::TEX_TYPE_2D, kSize, kSize, 0, Ogre::PF_R8G8B8A8,
          Ogre::TU_STATIC);
  Ogre::PixelBox box(static_cast<Ogre::uint32>(kSize),
                     static_cast<Ogre::uint32>(kSize), 1,
                     Ogre::PF_R8G8B8A8, pixels.data());
  texture->getBuffer()->blitFromMemory(box);
}

void CreatePointSpriteMaterial() {
  if (Ogre::MaterialManager::getSingleton().resourceExists(kPointSpriteMat)) {
    return;
  }
  CreatePointDiscTexture();
  Ogre::MaterialPtr material =
      Ogre::MaterialManager::getSingleton().create(
          kPointSpriteMat,
          Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME);
  Ogre::Pass* pass = material->getTechnique(0)->getPass(0);
  pass->createTextureUnitState(kPointSpriteTex);
  pass->setSceneBlending(Ogre::SBT_TRANSPARENT_ALPHA);
  pass->setDepthWriteEnabled(false);
  pass->setLightingEnabled(false);
  pass->setVertexColourTracking(Ogre::TVC_DIFFUSE);
}

void CreateGpuProgram(const Ogre::String& name, Ogre::GpuProgramType type,
                      const char* source) {
  if (Ogre::GpuProgramManager::getSingleton().resourceExists(name)) {
    return;
  }
  Ogre::GpuProgramPtr program =
      Ogre::GpuProgramManager::getSingleton().createProgram(
          name, Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME,
          "glsl", type);
  program->setSource(source);
  program->load();
}

constexpr char kPbrTexturedMat[] = "AvizPBRTextured";
constexpr char kPbrTexturedVert[] = "AvizPBRTexturedVert";
constexpr char kPbrTexturedFrag[] = "AvizPBRTexturedFrag";

constexpr char kPbrTexturedVertSource[] = R"GLSL(
#version 330 core
in vec4 position;
in vec3 normal;
in vec4 diffuse;
in vec2 texcoord0;
in vec2 texcoord1;
uniform mat4 worldviewproj_matrix;
out vec3 vWorldPos;
out vec3 vNormal;
out vec2 vUv;
out vec4 vTint;
out vec2 vMaterial;
void main() {
  vWorldPos = position.xyz;
  vNormal = normal;
  vUv = texcoord0;
  vTint = diffuse;
  vMaterial = texcoord1;
  gl_Position = worldviewproj_matrix * vec4(position.xyz, 1.0);
}
)GLSL";

constexpr char kPbrTexturedFragSource[] = R"GLSL(
#version 330 core
in vec3 vWorldPos;
in vec3 vNormal;
in vec2 vUv;
in vec4 vTint;
in vec2 vMaterial;
uniform sampler2D uAlbedoMap;
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
  vec4 sampled = texture(uAlbedoMap, vUv);
  vec3 albedo = sampled.rgb * vTint.rgb;
  float alpha = sampled.a * vTint.a;
  float metallic = vMaterial.x();
  float roughness = vMaterial.y();
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
)GLSL";

void CreatePbrTexturedMaterial() {
  if (Ogre::MaterialManager::getSingleton().resourceExists(kPbrTexturedMat)) {
    return;
  }
  CreateGpuProgram(kPbrTexturedVert, Ogre::GPT_VERTEX_PROGRAM,
                   kPbrTexturedVertSource);
  CreateGpuProgram(kPbrTexturedFrag, Ogre::GPT_FRAGMENT_PROGRAM,
                   kPbrTexturedFragSource);
  Ogre::MaterialPtr material =
      Ogre::MaterialManager::getSingleton().create(
          kPbrTexturedMat,
          Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME);
  Ogre::Pass* pass = material->getTechnique(0)->getPass(0);
  pass->setLightingEnabled(false);
  pass->setSceneBlending(Ogre::SBT_TRANSPARENT_ALPHA);
  pass->setVertexProgram(kPbrTexturedVert);
  pass->setFragmentProgram(kPbrTexturedFrag);
  pass->getVertexProgramParameters()->setNamedAutoConstant(
      "worldviewproj_matrix",
      Ogre::GpuProgramParameters::ACT_WORLDVIEWPROJ_MATRIX);
  Ogre::GpuProgramParametersSharedPtr fp_params =
      pass->getFragmentProgramParameters();
  fp_params->setNamedConstant("uAmbient", Ogre::Vector3(0.18f, 0.18f, 0.2f));
  fp_params->setNamedAutoConstant(
      "uCameraPos", Ogre::GpuProgramParameters::ACT_CAMERA_POSITION);
}

void CreatePbrMaterial() {
  if (Ogre::MaterialManager::getSingleton().resourceExists(kPbrMat)) {
    return;
  }
  CreateGpuProgram(kPbrVert, Ogre::GPT_VERTEX_PROGRAM, kPbrVertSource);
  CreateGpuProgram(kPbrFrag, Ogre::GPT_FRAGMENT_PROGRAM, kPbrFragSource);

  Ogre::MaterialPtr material =
      Ogre::MaterialManager::getSingleton().create(
          kPbrMat, Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME);
  Ogre::Pass* pass = material->getTechnique(0)->getPass(0);
  pass->setLightingEnabled(false);
  pass->setVertexProgram(kPbrVert);
  pass->setFragmentProgram(kPbrFrag);

  Ogre::GpuProgramParametersSharedPtr vp_params =
      pass->getVertexProgramParameters();
  vp_params->setNamedAutoConstant(
      "worldviewproj_matrix", Ogre::GpuProgramParameters::ACT_WORLDVIEWPROJ_MATRIX);

  Ogre::GpuProgramParametersSharedPtr fp_params =
      pass->getFragmentProgramParameters();
  fp_params->setNamedConstant("uAmbient",
                             Ogre::Vector3(0.18f, 0.18f, 0.2f));
  fp_params->setNamedAutoConstant(
      "uCameraPos", Ogre::GpuProgramParameters::ACT_CAMERA_POSITION);
}

}  // namespace

void EnsureOgreMaterials(Ogre::SceneManager* /*scene*/) {
  CreatePointSpriteMaterial();
  CreatePbrMaterial();
  CreatePbrTexturedMaterial();
}

}  // namespace rendering
}  // namespace autoviz

#endif
