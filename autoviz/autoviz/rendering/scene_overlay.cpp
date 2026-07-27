/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#include "autoviz/rendering/scene_overlay.hpp"

#include <QOpenGLExtraFunctions>
#include <QOpenGLFunctions>

#include "autoviz/display/obj_mesh.hpp"

#include "autoviz/common/pick_handle.hpp"
#include "autoviz/common/pick_registry.hpp"
#include "autoviz/common/selection_handler.hpp"

#include <cmath>

namespace autoviz {
namespace rendering {
namespace {

constexpr char kVertexShader[] = R"(#version 330 core
layout(location = 0) in vec3 aPos;
layout(location = 1) in vec4 aColor;
uniform mat4 uMvp;
out vec4 vColor;
void main() {
  gl_Position = uMvp * vec4(aPos, 1.0);
  vColor = aColor;
}
)";

constexpr char kFragmentShader[] = R"(#version 330 core
in vec4 vColor;
out vec4 fragColor;
void main() { fragColor = vColor; }
)";

constexpr char kPointVertexShader[] = R"(#version 330 core
layout(location = 0) in vec3 aPos;
layout(location = 1) in vec4 aColor;
uniform mat4 uMvp;
uniform float uPointSize;
out vec4 vColor;
void main() {
  gl_Position = uMvp * vec4(aPos, 1.0);
  gl_PointSize = uPointSize;
  vColor = aColor;
}
)";

constexpr char kTexturedVertexShader[] = R"(#version 330 core
layout(location = 0) in vec3 aPos;
layout(location = 1) in vec2 aUv;
layout(location = 2) in vec4 aColor;
uniform mat4 uMvp;
out vec2 vUv;
out vec4 vColor;
void main() {
  gl_Position = uMvp * vec4(aPos, 1.0);
  vUv = aUv;
  vColor = aColor;
}
)";

constexpr char kTexturedFragmentShader[] = R"(#version 330 core
in vec2 vUv;
in vec4 vColor;
uniform sampler2D uTexture;
out vec4 fragColor;
void main() {
  fragColor = texture(uTexture, vUv) * vColor;
}
)";

constexpr char kPbrVertexShader[] = R"(#version 330 core
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
)";

constexpr char kPbrFragmentShader[] = R"(#version 330 core
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
)";

constexpr char kPbrTexturedVertexShader[] = R"(#version 330 core
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
)";

constexpr char kPbrTexturedFragmentShader[] = R"(#version 330 core
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
)";

unsigned CompileShader(unsigned type, const char* source) {
  QOpenGLFunctions* gl = QOpenGLContext::currentContext()->functions();
  const unsigned shader = gl->glCreateShader(type);
  gl->glShaderSource(shader, 1, &source, nullptr);
  gl->glCompileShader(shader);
  return shader;
}

unsigned LinkProgram(const char* vs, const char* fs) {
  QOpenGLFunctions* gl = QOpenGLContext::currentContext()->functions();
  const unsigned v_shader = CompileShader(0x8B31, vs);
  const unsigned f_shader = CompileShader(0x8B30, fs);
  const unsigned program = gl->glCreateProgram();
  gl->glAttachShader(program, v_shader);
  gl->glAttachShader(program, f_shader);
  gl->glLinkProgram(program);
  gl->glDeleteShader(v_shader);
  gl->glDeleteShader(f_shader);
  return program;
}

QVector4D ToVec4(const QColor& color) {
  return QVector4D(color.redF(), color.greenF(), color.blueF(), color.alphaF());
}

void UploadVertices(unsigned vao, unsigned vbo,
                    const std::vector<SceneOverlay::ColoredVertex>& data,
                    bool* dirty_flag) {
  if (!(*dirty_flag) || data.empty()) {
    return;
  }
  QOpenGLExtraFunctions* gl =
      QOpenGLContext::currentContext()->extraFunctions();
  gl->glBindVertexArray(vao);
  gl->glBindBuffer(0x8892, vbo);
  gl->glBufferData(0x8892,
                   static_cast<int>(data.size() * sizeof(SceneOverlay::ColoredVertex)),
                   data.data(), 0x88E0);
  gl->glEnableVertexAttribArray(0);
  gl->glVertexAttribPointer(0, 3, 0x1406, 0, sizeof(SceneOverlay::ColoredVertex),
                            reinterpret_cast<void*>(0));
  gl->glEnableVertexAttribArray(1);
  gl->glVertexAttribPointer(1, 4, 0x1406, 0, sizeof(SceneOverlay::ColoredVertex),
                            reinterpret_cast<void*>(sizeof(QVector3D)));
  gl->glBindVertexArray(0);
  *dirty_flag = false;
}

constexpr char kPickVertexShader[] = R"(#version 330 core
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
)";

constexpr char kPickFragmentShader[] = R"(#version 330 core
in vec3 vPickColor;
out vec4 fragColor;
void main() { fragColor = vec4(vPickColor, 1.0); }
)";

constexpr char kPickFlatVertexShader[] = R"(#version 330 core
layout(location = 0) in vec3 aPos;
layout(location = 1) in vec3 aPickColor;
uniform mat4 uMvp;
out vec3 vPickColor;
void main() {
  gl_Position = uMvp * vec4(aPos, 1.0);
  vPickColor = aPickColor;
}
)";

QVector3D HandleToVec3(uint32_t handle) {
  const common::PickColor color = common::handleToPickColor(handle);
  return QVector3D(color.r / 255.f, color.g / 255.f, color.b / 255.f);
}

struct GpuPickVertex {
  QVector3D position;
  QVector3D pick_color;
};

}  // namespace

void SceneOverlay::clear() {
  line_vertices_.clear();
  point_vertices_.clear();
  triangle_vertices_.clear();
  pick_samples_.clear();
  pick_point_vertices_.clear();
  textured_batches_.clear();
  billboard_requests_.clear();
  view_facing_textured_requests_.clear();
  pbr_vertices_.clear();
  pbr_textured_batches_.clear();
  pick_display_name_.clear();
  pick_display_type_.clear();
  has_pick_display_name_ = false;
  has_pick_display_type_ = false;
  line_dirty_ = true;
  point_dirty_ = true;
  triangle_dirty_ = true;
  textured_dirty_ = true;
  pbr_dirty_ = true;
  pick_point_dirty_ = true;
}

void SceneOverlay::setPickSource(const std::string* display_name,
                                 const std::string* display_type) {
  if (display_name != nullptr) {
    pick_display_name_ = *display_name;
    has_pick_display_name_ = true;
  } else {
    pick_display_name_.clear();
    has_pick_display_name_ = false;
  }
  if (display_type != nullptr) {
    pick_display_type_ = *display_type;
    has_pick_display_type_ = true;
  } else {
    pick_display_type_.clear();
    has_pick_display_type_ = false;
  }
}

void SceneOverlay::recordPick(const QVector3D& position) {
  PickSample sample;
  sample.position = position;
  if (has_pick_display_name_) {
    sample.display_name = pick_display_name_;
  }
  if (has_pick_display_type_) {
    sample.display_type = pick_display_type_;
  }
  pick_samples_.push_back(std::move(sample));
}

void SceneOverlay::addLine(const QVector3D& a, const QVector3D& b,
                           const QColor& color) {
  const QVector4D c = ToVec4(color);
  line_vertices_.push_back({a, c});
  line_vertices_.push_back({b, c});
  recordPick(a);
  recordPick(b);
  line_dirty_ = true;
}

void SceneOverlay::addEllipsoidWireframe(const QVector3D& center, float radius_x,
                                         float radius_y, float radius_z,
                                         const QColor& color) {
  constexpr int kSegments = 24;
  const QVector4D c = ToVec4(color);
  auto add_circle = [&](auto point_fn) {
    for (int i = 0; i < kSegments; ++i) {
      const float t0 = static_cast<float>(i) / static_cast<float>(kSegments) *
                       2.f * static_cast<float>(M_PI);
      const float t1 = static_cast<float>(i + 1) / static_cast<float>(kSegments) *
                       2.f * static_cast<float>(M_PI);
      const QVector3D a = point_fn(t0);
      const QVector3D b = point_fn(t1);
      line_vertices_.push_back({a, c});
      line_vertices_.push_back({b, c});
    }
    line_dirty_ = true;
  };
  add_circle([&](float t) {
    return center + QVector3D(radius_x * qCos(t), radius_y * qSin(t), 0.f);
  });
  add_circle([&](float t) {
    return center + QVector3D(radius_x * qCos(t), 0.f, radius_z * qSin(t));
  });
  add_circle([&](float t) {
    return center + QVector3D(0.f, radius_y * qCos(t), radius_z * qSin(t));
  });
}

void SceneOverlay::addPoint(const QVector3D& p, const QColor& color) {
  addPickPoint(p, color, -1);
}

void SceneOverlay::addPickPoint(
    const QVector3D& p, const QColor& color, int point_index,
    const std::shared_ptr<common::SelectionHandler>& handler) {
  const QVector4D c = ToVec4(color);
  point_vertices_.push_back({p, c});
  recordPick(p);
  if (pick_registry_ != nullptr && has_pick_display_name_) {
    common::PickRecord record;
    record.display_name = pick_display_name_;
    if (has_pick_display_type_) {
      record.display_type = pick_display_type_;
    }
    record.position = p;
    record.point_index = point_index;
    const uint32_t handle = pick_registry_->registerPick(record);
    std::shared_ptr<common::SelectionHandler> bound = handler;
    if (bound == nullptr && handler_manager_ != nullptr) {
      bound = common::CreateSelectionHandler<common::DisplayPointSelectionHandler>();
      bound->setDisplayInfo(record.display_name, record.display_type);
      static_cast<common::DisplayPointSelectionHandler*>(bound.get())
          ->setPointIndex(point_index);
    }
    if (handler_manager_ != nullptr && bound != nullptr) {
      handler_manager_->registerHandler(handle, bound);
    }
    pick_point_vertices_.push_back({p, handle});
    pick_point_dirty_ = true;
  }
  point_dirty_ = true;
}

void SceneOverlay::addPickMarker(
    const QVector3D& p, int point_index,
    const std::shared_ptr<common::SelectionHandler>& handler) {
  const common::PickHandle handle = registerPickEntry(p, point_index, handler);
  if (handle != common::kInvalidPickHandle) {
    pick_point_vertices_.push_back({p, handle});
    pick_point_dirty_ = true;
  }
}

common::PickHandle SceneOverlay::registerPickEntry(
    const QVector3D& position, int point_index,
    const std::shared_ptr<common::SelectionHandler>& handler) {
  recordPick(position);
  if (pick_registry_ == nullptr || !has_pick_display_name_) {
    return common::kInvalidPickHandle;
  }
  common::PickRecord record;
  record.display_name = pick_display_name_;
  if (has_pick_display_type_) {
    record.display_type = pick_display_type_;
  }
  record.position = position;
  record.point_index = point_index;
  const common::PickHandle handle = pick_registry_->registerPick(record);
  std::shared_ptr<common::SelectionHandler> bound = handler;
  if (bound == nullptr && handler_manager_ != nullptr) {
    bound = common::CreateSelectionHandler<common::DisplayPointSelectionHandler>();
    bound->setDisplayInfo(record.display_name, record.display_type);
    static_cast<common::DisplayPointSelectionHandler*>(bound.get())
        ->setPointIndex(point_index);
  }
  if (bound != nullptr) {
    bound->setHandle(handle);
    if (handler_manager_ != nullptr) {
      handler_manager_->registerHandler(handle, bound);
    }
  }
  return handle;
}

void SceneOverlay::addPoints(const std::vector<QVector3D>& points,
                             const QColor& color) {
  for (std::size_t i = 0; i < points.size(); ++i) {
    addPickPoint(points[i], color, static_cast<int>(i));
  }
}

void SceneOverlay::addBoxWireframe(const QVector3D& center,
                                   const QVector3D& half_extents,
                                   const QMatrix4x4& transform,
                                   const QColor& color) {
  const QVector3D corners[8] = {
      center + QVector3D(-half_extents.x(), -half_extents.y(), -half_extents.z()),
      center + QVector3D(half_extents.x(), -half_extents.y(), -half_extents.z()),
      center + QVector3D(half_extents.x(), half_extents.y(), -half_extents.z()),
      center + QVector3D(-half_extents.x(), half_extents.y(), -half_extents.z()),
      center + QVector3D(-half_extents.x(), -half_extents.y(), half_extents.z()),
      center + QVector3D(half_extents.x(), -half_extents.y(), half_extents.z()),
      center + QVector3D(half_extents.x(), half_extents.y(), half_extents.z()),
      center + QVector3D(-half_extents.x(), half_extents.y(), half_extents.z()),
  };
  const int edges[12][2] = {{0, 1}, {1, 2}, {2, 3}, {3, 0}, {4, 5}, {5, 6},
                            {6, 7}, {7, 4}, {0, 4}, {1, 5}, {2, 6}, {3, 7}};
  for (const auto& edge : edges) {
    addLine(transform.map(corners[edge[0]]), transform.map(corners[edge[1]]),
            color);
  }
}

void SceneOverlay::addBoxSolid(const QVector3D& center,
                               const QVector3D& half_extents,
                               const QMatrix4x4& transform,
                               const QColor& color) {
  const QVector3D corners[8] = {
      center + QVector3D(-half_extents.x(), -half_extents.y(), -half_extents.z()),
      center + QVector3D(half_extents.x(), -half_extents.y(), -half_extents.z()),
      center + QVector3D(half_extents.x(), half_extents.y(), -half_extents.z()),
      center + QVector3D(-half_extents.x(), half_extents.y(), -half_extents.z()),
      center + QVector3D(-half_extents.x(), -half_extents.y(), half_extents.z()),
      center + QVector3D(half_extents.x(), -half_extents.y(), half_extents.z()),
      center + QVector3D(half_extents.x(), half_extents.y(), half_extents.z()),
      center + QVector3D(-half_extents.x(), half_extents.y(), half_extents.z()),
  };
  const int faces[12][3] = {
      {0, 1, 2}, {0, 2, 3}, {4, 6, 5}, {4, 7, 6}, {0, 4, 5}, {0, 5, 1},
      {2, 6, 7}, {2, 7, 3}, {0, 3, 7}, {0, 7, 4}, {1, 5, 6}, {1, 6, 2},
  };
  const QVector4D c = ToVec4(color);
  for (const auto& face : faces) {
    const QVector3D v0 = transform.map(corners[face[0]]);
    const QVector3D v1 = transform.map(corners[face[1]]);
    const QVector3D v2 = transform.map(corners[face[2]]);
    triangle_vertices_.push_back({v0, c});
    triangle_vertices_.push_back({v1, c});
    triangle_vertices_.push_back({v2, c});
    recordPick(v0);
    recordPick(v1);
    recordPick(v2);
  }
  triangle_dirty_ = true;
}

void SceneOverlay::addTriangleMeshWireframe(const display::ObjMesh& mesh,
                                            const QMatrix4x4& transform,
                                            const QColor& color) {
  for (const auto& triangle : mesh.triangles) {
    if (triangle[0] < 0 ||
        triangle[0] >= static_cast<int>(mesh.vertices.size()) ||
        triangle[1] < 0 ||
        triangle[1] >= static_cast<int>(mesh.vertices.size()) ||
        triangle[2] < 0 ||
        triangle[2] >= static_cast<int>(mesh.vertices.size())) {
      continue;
    }
    const QVector3D a = transform.map(mesh.vertices[static_cast<std::size_t>(triangle[0])]);
    const QVector3D b = transform.map(mesh.vertices[static_cast<std::size_t>(triangle[1])]);
    const QVector3D c = transform.map(mesh.vertices[static_cast<std::size_t>(triangle[2])]);
    addLine(a, b, color);
    addLine(b, c, color);
    addLine(c, a, color);
  }
}

void SceneOverlay::addTriangleMeshSolid(const display::ObjMesh& mesh,
                                        const QMatrix4x4& transform,
                                        const QColor& color) {
  const QVector4D c_color = ToVec4(color);
  for (const auto& triangle : mesh.triangles) {
    if (triangle[0] < 0 ||
        triangle[0] >= static_cast<int>(mesh.vertices.size()) ||
        triangle[1] < 0 ||
        triangle[1] >= static_cast<int>(mesh.vertices.size()) ||
        triangle[2] < 0 ||
        triangle[2] >= static_cast<int>(mesh.vertices.size())) {
      continue;
    }
    const QVector3D a = transform.map(mesh.vertices[static_cast<std::size_t>(triangle[0])]);
    const QVector3D b = transform.map(mesh.vertices[static_cast<std::size_t>(triangle[1])]);
    const QVector3D c = transform.map(mesh.vertices[static_cast<std::size_t>(triangle[2])]);
    triangle_vertices_.push_back({a, c_color});
    triangle_vertices_.push_back({b, c_color});
    triangle_vertices_.push_back({c, c_color});
    recordPick(a);
    recordPick(b);
    recordPick(c);
  }
  triangle_dirty_ = true;
}

void SceneOverlay::addTriangleMeshSolidPbr(const display::ObjMesh& mesh,
                                            const QMatrix4x4& transform,
                                            const QColor& color,
                                            float metallic, float roughness) {
  const QVector4D albedo = ToVec4(color);
  for (const auto& triangle : mesh.triangles) {
    if (triangle[0] < 0 ||
        triangle[0] >= static_cast<int>(mesh.vertices.size()) ||
        triangle[1] < 0 ||
        triangle[1] >= static_cast<int>(mesh.vertices.size()) ||
        triangle[2] < 0 ||
        triangle[2] >= static_cast<int>(mesh.vertices.size())) {
      continue;
    }
    const QVector3D a = transform.map(mesh.vertices[static_cast<std::size_t>(triangle[0])]);
    const QVector3D b = transform.map(mesh.vertices[static_cast<std::size_t>(triangle[1])]);
    const QVector3D c = transform.map(mesh.vertices[static_cast<std::size_t>(triangle[2])]);
    QVector3D normal = QVector3D::crossProduct(b - a, c - a);
    if (normal.lengthSquared() < 1e-10f) {
      normal = QVector3D(0.f, 1.f, 0.f);
    } else {
      normal.normalize();
    }
    pbr_vertices_.push_back({a, normal, albedo, metallic, roughness});
    pbr_vertices_.push_back({b, normal, albedo, metallic, roughness});
    pbr_vertices_.push_back({c, normal, albedo, metallic, roughness});
    recordPick(a);
    recordPick(b);
    recordPick(c);
  }
  pbr_dirty_ = true;
}

SceneOverlay::PbrTexturedBatch* SceneOverlay::findOrCreatePbrTexturedBatch(
    const QImage& image) {
  if (image.isNull()) {
    return nullptr;
  }
  const QImage rgba = image.convertToFormat(QImage::Format_RGBA8888);
  for (auto& batch : pbr_textured_batches_) {
    if (batch.image.cacheKey() == rgba.cacheKey()) {
      return &batch;
    }
  }
  pbr_textured_batches_.push_back({rgba, {}});
  return &pbr_textured_batches_.back();
}

void SceneOverlay::addTriangleMeshTexturedPbr(const display::ObjMesh& mesh,
                                              const QMatrix4x4& transform,
                                              const QImage& image,
                                              const QColor& tint, float metallic,
                                              float roughness) {
  PbrTexturedBatch* batch = findOrCreatePbrTexturedBatch(image);
  if (batch == nullptr) {
    return;
  }
  display::ObjMesh mesh_with_uv = mesh;
  if (mesh_with_uv.texcoords.size() != mesh_with_uv.vertices.size()) {
    display::ensureMeshTexcoords(&mesh_with_uv);
  }
  const QVector4D tint_vec = ToVec4(tint);
  for (const auto& triangle : mesh_with_uv.triangles) {
    if (triangle[0] < 0 ||
        triangle[0] >= static_cast<int>(mesh_with_uv.vertices.size()) ||
        triangle[1] < 0 ||
        triangle[1] >= static_cast<int>(mesh_with_uv.vertices.size()) ||
        triangle[2] < 0 ||
        triangle[2] >= static_cast<int>(mesh_with_uv.vertices.size())) {
      continue;
    }
    const QVector3D corners[3] = {
        transform.map(mesh_with_uv.vertices[static_cast<std::size_t>(triangle[0])]),
        transform.map(mesh_with_uv.vertices[static_cast<std::size_t>(triangle[1])]),
        transform.map(mesh_with_uv.vertices[static_cast<std::size_t>(triangle[2])])};
    QVector3D normal = QVector3D::crossProduct(corners[1] - corners[0], corners[2] - corners[0]);
    if (normal.lengthSquared() < 1e-10f) {
      normal = QVector3D(0.f, 1.f, 0.f);
    } else {
      normal.normalize();
    }
    for (int i = 0; i < 3; ++i) {
      const int vi = triangle[i];
      const QVector2D uv =
          mesh_with_uv.texcoords.size() == mesh_with_uv.vertices.size()
              ? mesh_with_uv.texcoords[static_cast<std::size_t>(vi)]
              : QVector2D(0.5f, 0.5f);
      batch->vertices.push_back(
          {corners[i], normal, uv, tint_vec, metallic, roughness});
      recordPick(corners[i]);
    }
  }
  pbr_dirty_ = true;
}

void SceneOverlay::addBoxSolidPbr(const QVector3D& center,
                                  const QVector3D& half_extents,
                                  const QMatrix4x4& transform,
                                  const QColor& color, float metallic,
                                  float roughness) {
  const QVector3D corners[8] = {
      center + QVector3D(-half_extents.x(), -half_extents.y(), -half_extents.z()),
      center + QVector3D(half_extents.x(), -half_extents.y(), -half_extents.z()),
      center + QVector3D(half_extents.x(), half_extents.y(), -half_extents.z()),
      center + QVector3D(-half_extents.x(), half_extents.y(), -half_extents.z()),
      center + QVector3D(-half_extents.x(), -half_extents.y(), half_extents.z()),
      center + QVector3D(half_extents.x(), -half_extents.y(), half_extents.z()),
      center + QVector3D(half_extents.x(), half_extents.y(), half_extents.z()),
      center + QVector3D(-half_extents.x(), half_extents.y(), half_extents.z()),
  };
  const int faces[12][3] = {
      {0, 1, 2}, {0, 2, 3}, {4, 6, 5}, {4, 7, 6}, {0, 4, 5}, {0, 5, 1},
      {2, 6, 7}, {2, 7, 3}, {0, 3, 7}, {0, 7, 4}, {1, 5, 6}, {1, 6, 2},
  };
  const QVector4D albedo = ToVec4(color);
  for (const auto& face : faces) {
    const QVector3D v0 = transform.map(corners[face[0]]);
    const QVector3D v1 = transform.map(corners[face[1]]);
    const QVector3D v2 = transform.map(corners[face[2]]);
    QVector3D normal = QVector3D::crossProduct(v1 - v0, v2 - v0);
    if (normal.lengthSquared() < 1e-10f) {
      normal = QVector3D(0.f, 1.f, 0.f);
    } else {
      normal.normalize();
    }
    pbr_vertices_.push_back({v0, normal, albedo, metallic, roughness});
    pbr_vertices_.push_back({v1, normal, albedo, metallic, roughness});
    pbr_vertices_.push_back({v2, normal, albedo, metallic, roughness});
  }
  pbr_dirty_ = true;
}

void SceneOverlay::addTexturedQuad(const QVector3D& top_left,
                                   const QVector3D& top_right,
                                   const QVector3D& bottom_right,
                                   const QVector3D& bottom_left,
                                   const QImage& image) {
  if (image.isNull()) {
    return;
  }
  TexturedBatch batch;
  batch.image = image.convertToFormat(QImage::Format_RGBA8888);
  batch.vertices = {
      {top_left, QVector2D(0.f, 0.f)},
      {top_right, QVector2D(1.f, 0.f)},
      {bottom_right, QVector2D(1.f, 1.f)},
      {top_left, QVector2D(0.f, 0.f)},
      {bottom_right, QVector2D(1.f, 1.f)},
      {bottom_left, QVector2D(0.f, 1.f)},
  };
  textured_batches_.push_back(std::move(batch));
  textured_dirty_ = true;
}

void SceneOverlay::addViewFacingQuad(const QVector3D& center, float half_extent,
                                     const QColor& color) {
  if (half_extent <= 0.f) {
    return;
  }
  billboard_requests_.push_back({center, half_extent, ToVec4(color)});
}

void SceneOverlay::addViewFacingTexturedQuad(const QVector3D& center,
                                             float half_width, float half_height,
                                             const QImage& image) {
  if (half_width <= 0.f || half_height <= 0.f || image.isNull()) {
    return;
  }
  ViewFacingTexturedRequest request;
  request.center = center;
  request.half_width = half_width;
  request.half_height = half_height;
  request.image = image.convertToFormat(QImage::Format_RGBA8888);
  view_facing_textured_requests_.push_back(std::move(request));
  textured_dirty_ = true;
}

std::vector<SceneOverlay::TexturedBatch> SceneOverlay::expandedTexturedBatches(
    const QMatrix4x4& view) const {
  std::vector<TexturedBatch> batches = textured_batches_;
  if (!view_facing_textured_requests_.empty()) {
    const QMatrix4x4 inv_view = view.inverted();
    const QVector3D camera_right = inv_view.column(0).toVector3D().normalized();
    const QVector3D camera_up = inv_view.column(1).toVector3D().normalized();

    for (const auto& request : view_facing_textured_requests_) {
      if (request.image.isNull()) {
        continue;
      }
      TexturedBatch batch;
      batch.image = request.image;
      const QVector3D c = request.center;
      const float hw = request.half_width;
      const float hh = request.half_height;
      const QVector3D top_left = c - camera_right * hw + camera_up * hh;
      const QVector3D top_right = c + camera_right * hw + camera_up * hh;
      const QVector3D bottom_right = c + camera_right * hw - camera_up * hh;
      const QVector3D bottom_left = c - camera_right * hw - camera_up * hh;
      batch.vertices = {
          {top_left, QVector2D(0.f, 0.f)},
          {top_right, QVector2D(1.f, 0.f)},
          {bottom_right, QVector2D(1.f, 1.f)},
          {top_left, QVector2D(0.f, 0.f)},
          {bottom_right, QVector2D(1.f, 1.f)},
          {bottom_left, QVector2D(0.f, 1.f)},
      };
      batches.push_back(std::move(batch));
    }
  }
  appendPointSpriteBatch(view, &batches);
  return batches;
}

std::vector<SceneOverlay::ColoredVertex> SceneOverlay::expandedFlatTriangles(
    const QMatrix4x4& view) const {
  std::vector<ColoredVertex> triangles = triangle_vertices_;
  appendViewFacingBillboards(view, &triangles);
  return triangles;
}

std::vector<SceneOverlay::ColoredVertex> SceneOverlay::expandedTriangles(
    const QMatrix4x4& view) const {
  return expandedFlatTriangles(view);
}

QImage SceneOverlay::pointDiscImage() {
  static QImage disc;
  if (!disc.isNull()) {
    return disc;
  }
  constexpr int kSize = 64;
  disc = QImage(kSize, kSize, QImage::Format_RGBA8888);
  disc.fill(Qt::transparent);
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
      disc.setPixelColor(
          x, y, QColor(255, 255, 255, static_cast<int>(alpha * 255.f)));
    }
  }
  return disc;
}

void SceneOverlay::appendPointSpriteBatch(
    const QMatrix4x4& view, std::vector<TexturedBatch>* batches) const {
  if (batches == nullptr || point_vertices_.empty() || point_size_ <= 1.f) {
    return;
  }
  const QMatrix4x4 inv_view = view.inverted();
  const QVector3D camera_right = inv_view.column(0).toVector3D().normalized();
  const QVector3D camera_up = inv_view.column(1).toVector3D().normalized();
  const float half = std::max(0.004f, point_size_ * 0.012f);
  TexturedBatch batch;
  batch.image = pointDiscImage();
  batch.vertices.reserve(point_vertices_.size() * 6);
  for (const auto& point : point_vertices_) {
    const QVector3D c = point.position;
    const QVector3D tl = c - camera_right * half + camera_up * half;
    const QVector3D tr = c + camera_right * half + camera_up * half;
    const QVector3D br = c + camera_right * half - camera_up * half;
    const QVector3D bl = c - camera_right * half - camera_up * half;
    batch.vertices.push_back({tl, QVector2D(0.f, 0.f), point.color});
    batch.vertices.push_back({tr, QVector2D(1.f, 0.f), point.color});
    batch.vertices.push_back({br, QVector2D(1.f, 1.f), point.color});
    batch.vertices.push_back({tl, QVector2D(0.f, 0.f), point.color});
    batch.vertices.push_back({br, QVector2D(1.f, 1.f), point.color});
    batch.vertices.push_back({bl, QVector2D(0.f, 1.f), point.color});
  }
  if (!batch.vertices.empty()) {
    batches->push_back(std::move(batch));
  }
}

void SceneOverlay::appendViewFacingBillboards(
    const QMatrix4x4& view, std::vector<ColoredVertex>* triangles) const {
  if (triangles == nullptr || billboard_requests_.empty()) {
    return;
  }
  const QMatrix4x4 inv_view = view.inverted();
  const QVector3D camera_right = inv_view.column(0).toVector3D().normalized();
  const QVector3D camera_up = inv_view.column(1).toVector3D().normalized();

  for (const auto& billboard : billboard_requests_) {
    const QVector3D c = billboard.center;
    const float s = billboard.half_extent;
    const QVector3D p0 = c - camera_right * s - camera_up * s;
    const QVector3D p1 = c + camera_right * s - camera_up * s;
    const QVector3D p2 = c + camera_right * s + camera_up * s;
    const QVector3D p3 = c - camera_right * s + camera_up * s;
    triangles->push_back({p0, billboard.color});
    triangles->push_back({p1, billboard.color});
    triangles->push_back({p2, billboard.color});
    triangles->push_back({p0, billboard.color});
    triangles->push_back({p2, billboard.color});
    triangles->push_back({p3, billboard.color});
  }
}

void SceneOverlay::initialize() {
  if (initialized_) {
    return;
  }
  line_program_ = LinkProgram(kVertexShader, kFragmentShader);
  point_program_ = LinkProgram(kPointVertexShader, kFragmentShader);
  triangle_program_ = LinkProgram(kVertexShader, kFragmentShader);
  textured_program_ = LinkProgram(kTexturedVertexShader, kTexturedFragmentShader);
  pbr_program_ = LinkProgram(kPbrVertexShader, kPbrFragmentShader);
  pbr_textured_program_ =
      LinkProgram(kPbrTexturedVertexShader, kPbrTexturedFragmentShader);
  pick_program_ = LinkProgram(kPickFlatVertexShader, kPickFragmentShader);
  QOpenGLExtraFunctions* gl =
      QOpenGLContext::currentContext()->extraFunctions();
  gl->glGenVertexArrays(1, reinterpret_cast<unsigned*>(&line_vao_));
  gl->glGenBuffers(1, reinterpret_cast<unsigned*>(&line_vbo_));
  gl->glGenVertexArrays(1, reinterpret_cast<unsigned*>(&point_vao_));
  gl->glGenBuffers(1, reinterpret_cast<unsigned*>(&point_vbo_));
  gl->glGenVertexArrays(1, reinterpret_cast<unsigned*>(&triangle_vao_));
  gl->glGenBuffers(1, reinterpret_cast<unsigned*>(&triangle_vbo_));
  gl->glGenVertexArrays(1, reinterpret_cast<unsigned*>(&textured_vao_));
  gl->glGenBuffers(1, reinterpret_cast<unsigned*>(&textured_vbo_));
  gl->glGenVertexArrays(1, reinterpret_cast<unsigned*>(&pbr_vao_));
  gl->glGenBuffers(1, reinterpret_cast<unsigned*>(&pbr_vbo_));
  gl->glGenVertexArrays(1, reinterpret_cast<unsigned*>(&pbr_textured_vao_));
  gl->glGenBuffers(1, reinterpret_cast<unsigned*>(&pbr_textured_vbo_));
  gl->glGenVertexArrays(1, reinterpret_cast<unsigned*>(&pick_vao_));
  gl->glGenBuffers(1, reinterpret_cast<unsigned*>(&pick_vbo_));
  initialized_ = true;
}

void SceneOverlay::uploadIfNeeded() {
  UploadVertices(line_vao_, line_vbo_, line_vertices_, &line_dirty_);
  UploadVertices(point_vao_, point_vbo_, point_vertices_, &point_dirty_);
  UploadVertices(triangle_vao_, triangle_vbo_, triangle_vertices_,
                 &triangle_dirty_);
}

void SceneOverlay::renderTexturedBatches(const QMatrix4x4& view,
                                         const QMatrix4x4& mvp) {
  const std::vector<TexturedBatch> batches = expandedTexturedBatches(view);
  if (batches.empty() || textured_program_ == 0) {
    return;
  }
  QOpenGLFunctions* gl = QOpenGLContext::currentContext()->functions();
  QOpenGLExtraFunctions* gl_extra =
      QOpenGLContext::currentContext()->extraFunctions();
  gl->glEnable(0x0B71);  // GL_DEPTH_TEST
  gl->glDepthMask(0x0001);
  gl->glUseProgram(textured_program_);
  gl_extra->glUniformMatrix4fv(
      gl->glGetUniformLocation(textured_program_, "uMvp"), 1, 0, mvp.constData());

  for (const auto& batch : batches) {
    if (batch.vertices.empty() || batch.image.isNull()) {
      continue;
    }
    gl_extra->glBindVertexArray(textured_vao_);
    gl_extra->glBindBuffer(0x8892, textured_vbo_);
    gl_extra->glBufferData(
        0x8892,
        static_cast<int>(batch.vertices.size() * sizeof(TexturedVertex)),
        batch.vertices.data(), 0x88E0);
    gl_extra->glEnableVertexAttribArray(0);
    gl_extra->glVertexAttribPointer(0, 3, 0x1406, 0, sizeof(TexturedVertex),
                                    reinterpret_cast<void*>(0));
    gl_extra->glEnableVertexAttribArray(1);
    gl_extra->glVertexAttribPointer(1, 2, 0x1406, 0, sizeof(TexturedVertex),
                                    reinterpret_cast<void*>(sizeof(QVector3D)));
    gl_extra->glEnableVertexAttribArray(2);
    gl_extra->glVertexAttribPointer(
        2, 4, 0x1406, 0, sizeof(TexturedVertex),
        reinterpret_cast<void*>(sizeof(QVector3D) + sizeof(QVector2D)));

    unsigned texture = 0;
    gl_extra->glGenTextures(1, &texture);
    gl_extra->glBindTexture(0x0DE1, texture);
    gl_extra->glTexParameteri(0x0DE1, 0x2801, 0x2601);
    gl_extra->glTexParameteri(0x0DE1, 0x2800, 0x2601);
    gl_extra->glTexParameteri(0x0DE1, 0x2802, 0x812F);
    gl_extra->glTexParameteri(0x0DE1, 0x2803, 0x812F);
    gl_extra->glTexImage2D(0x0DE1, 0, 0x1908, batch.image.width(),
                           batch.image.height(), 0, 0x1908, 0x1401,
                           batch.image.constBits());
    gl_extra->glUniform1i(gl->glGetUniformLocation(textured_program_, "uTexture"),
                          0);
    gl->glDrawArrays(0x0004, 0, static_cast<int>(batch.vertices.size()));
    gl_extra->glDeleteTextures(1, &texture);
  }
  gl_extra->glBindVertexArray(0);
}

void SceneOverlay::renderPbrMesh(const QMatrix4x4& mvp, const QMatrix4x4& view) {
  if (pbr_vertices_.empty() || pbr_program_ == 0) {
    return;
  }
  struct GpuPbrVertex {
    QVector3D position;
    QVector3D normal;
    QVector4D albedo;
    QVector2D material;
  };
  std::vector<GpuPbrVertex> gpu_vertices;
  gpu_vertices.reserve(pbr_vertices_.size());
  for (const PbrVertex& vertex : pbr_vertices_) {
    gpu_vertices.push_back({vertex.position, vertex.normal, vertex.albedo,
                            QVector2D(vertex.metallic, vertex.roughness)});
  }

  QOpenGLFunctions* gl = QOpenGLContext::currentContext()->functions();
  QOpenGLExtraFunctions* gl_extra =
      QOpenGLContext::currentContext()->extraFunctions();
  gl->glEnable(0x0B71);
  gl->glDepthMask(0x0001);
  gl->glUseProgram(pbr_program_);
  gl_extra->glUniformMatrix4fv(
      gl->glGetUniformLocation(pbr_program_, "uMvp"), 1, 0, mvp.constData());
  const QMatrix4x4 inv_view = view.inverted();
  const QVector3D camera_pos = inv_view.column(3).toVector3D();
  gl_extra->glUniform3f(gl->glGetUniformLocation(pbr_program_, "uCameraPos"),
                        camera_pos.x(), camera_pos.y(), camera_pos.z());
  gl_extra->glUniform3f(gl->glGetUniformLocation(pbr_program_, "uLightDir"),
                        0.3f, -0.85f, -0.45f);
  gl_extra->glUniform3f(gl->glGetUniformLocation(pbr_program_, "uAmbient"),
                        0.18f, 0.18f, 0.2f);

  gl_extra->glBindVertexArray(pbr_vao_);
  gl_extra->glBindBuffer(0x8892, pbr_vbo_);
  gl_extra->glBufferData(
      0x8892, static_cast<int>(gpu_vertices.size() * sizeof(GpuPbrVertex)),
      gpu_vertices.data(), 0x88E0);
  gl_extra->glEnableVertexAttribArray(0);
  gl_extra->glVertexAttribPointer(0, 3, 0x1406, 0, sizeof(GpuPbrVertex),
                                  reinterpret_cast<void*>(0));
  gl_extra->glEnableVertexAttribArray(1);
  gl_extra->glVertexAttribPointer(1, 3, 0x1406, 0, sizeof(GpuPbrVertex),
                                  reinterpret_cast<void*>(sizeof(QVector3D)));
  gl_extra->glEnableVertexAttribArray(2);
  gl_extra->glVertexAttribPointer(
      2, 4, 0x1406, 0, sizeof(GpuPbrVertex),
      reinterpret_cast<void*>(sizeof(QVector3D) * 2));
  gl_extra->glEnableVertexAttribArray(3);
  gl_extra->glVertexAttribPointer(
      3, 2, 0x1406, 0, sizeof(GpuPbrVertex),
      reinterpret_cast<void*>(sizeof(QVector3D) * 2 + sizeof(QVector4D)));
  gl->glDrawArrays(0x0004, 0, static_cast<int>(gpu_vertices.size()));
  gl_extra->glBindVertexArray(0);
}

void SceneOverlay::renderPbrTexturedMeshes(const QMatrix4x4& mvp,
                                           const QMatrix4x4& view) {
  if (pbr_textured_batches_.empty() || pbr_textured_program_ == 0) {
    return;
  }
  struct GpuVertex {
    QVector3D position;
    QVector3D normal;
    QVector2D uv;
    QVector4D tint;
    QVector2D material;
  };

  QOpenGLFunctions* gl = QOpenGLContext::currentContext()->functions();
  QOpenGLExtraFunctions* gl_extra =
      QOpenGLContext::currentContext()->extraFunctions();
  gl->glEnable(0x0B71);
  gl->glDepthMask(0x0001);
  gl->glUseProgram(pbr_textured_program_);
  gl_extra->glUniformMatrix4fv(
      gl->glGetUniformLocation(pbr_textured_program_, "uMvp"), 1, 0, mvp.constData());
  const QMatrix4x4 inv_view = view.inverted();
  const QVector3D camera_pos = inv_view.column(3).toVector3D();
  gl_extra->glUniform3f(gl->glGetUniformLocation(pbr_textured_program_, "uCameraPos"),
                        camera_pos.x(), camera_pos.y(), camera_pos.z());
  gl_extra->glUniform3f(gl->glGetUniformLocation(pbr_textured_program_, "uLightDir"),
                        0.3f, -0.85f, -0.45f);
  gl_extra->glUniform3f(gl->glGetUniformLocation(pbr_textured_program_, "uAmbient"),
                        0.18f, 0.18f, 0.2f);

  for (const PbrTexturedBatch& batch : pbr_textured_batches_) {
    if (batch.vertices.empty() || batch.image.isNull()) {
      continue;
    }
    std::vector<GpuVertex> gpu_vertices;
    gpu_vertices.reserve(batch.vertices.size());
    for (const PbrTexturedVertex& vertex : batch.vertices) {
      gpu_vertices.push_back({vertex.position, vertex.normal, vertex.uv, vertex.tint,
                            QVector2D(vertex.metallic, vertex.roughness)});
    }
    gl_extra->glBindVertexArray(pbr_textured_vao_);
    gl_extra->glBindBuffer(0x8892, pbr_textured_vbo_);
    gl_extra->glBufferData(
        0x8892, static_cast<int>(gpu_vertices.size() * sizeof(GpuVertex)),
        gpu_vertices.data(), 0x88E0);
    gl_extra->glEnableVertexAttribArray(0);
    gl_extra->glVertexAttribPointer(0, 3, 0x1406, 0, sizeof(GpuVertex),
                                    reinterpret_cast<void*>(0));
    gl_extra->glEnableVertexAttribArray(1);
    gl_extra->glVertexAttribPointer(1, 3, 0x1406, 0, sizeof(GpuVertex),
                                    reinterpret_cast<void*>(sizeof(QVector3D)));
    gl_extra->glEnableVertexAttribArray(2);
    gl_extra->glVertexAttribPointer(
        2, 2, 0x1406, 0, sizeof(GpuVertex),
        reinterpret_cast<void*>(sizeof(QVector3D) * 2));
    gl_extra->glEnableVertexAttribArray(3);
    gl_extra->glVertexAttribPointer(
        3, 4, 0x1406, 0, sizeof(GpuVertex),
        reinterpret_cast<void*>(sizeof(QVector3D) * 2 + sizeof(QVector2D)));
    gl_extra->glEnableVertexAttribArray(4);
    gl_extra->glVertexAttribPointer(
        4, 2, 0x1406, 0, sizeof(GpuVertex),
        reinterpret_cast<void*>(sizeof(QVector3D) * 2 + sizeof(QVector2D) +
                                sizeof(QVector4D)));

    unsigned texture = 0;
    gl_extra->glGenTextures(1, &texture);
    gl_extra->glBindTexture(0x0DE1, texture);
    gl_extra->glTexParameteri(0x0DE1, 0x2801, 0x2601);
    gl_extra->glTexParameteri(0x0DE1, 0x2800, 0x2601);
    gl_extra->glTexParameteri(0x0DE1, 0x2802, 0x812F);
    gl_extra->glTexParameteri(0x0DE1, 0x2803, 0x812F);
    const QImage rgba = batch.image.convertToFormat(QImage::Format_RGBA8888);
    gl_extra->glTexImage2D(0x0DE1, 0, 0x1908, rgba.width(), rgba.height(), 0,
                           0x1908, 0x1401, rgba.constBits());
    gl_extra->glUniform1i(
        gl->glGetUniformLocation(pbr_textured_program_, "uAlbedoMap"), 0);
    gl->glDrawArrays(0x0004, 0, static_cast<int>(gpu_vertices.size()));
    gl_extra->glDeleteTextures(1, &texture);
  }
  gl_extra->glBindVertexArray(0);
}

void SceneOverlay::render(const QMatrix4x4& view, const QMatrix4x4& projection) {
  if (!initialized_ || empty()) {
    return;
  }

  std::vector<ColoredVertex> render_triangles = expandedFlatTriangles(view);

  QOpenGLFunctions* gl = QOpenGLContext::currentContext()->functions();
  QOpenGLExtraFunctions* gl_extra =
      QOpenGLContext::currentContext()->extraFunctions();
  const QMatrix4x4 mvp = projection * view;

  gl->glEnable(0x0BE2);  // GL_BLEND
  gl->glBlendFunc(0x0302, 0x0303);

  if (!render_triangles.empty()) {
    gl->glEnable(0x0B71);  // GL_DEPTH_TEST
    gl->glDepthMask(0x0001);
    gl->glUseProgram(triangle_program_);
    gl_extra->glUniformMatrix4fv(
        gl->glGetUniformLocation(triangle_program_, "uMvp"), 1, 0,
        mvp.constData());
    gl_extra->glBindVertexArray(triangle_vao_);
    gl_extra->glBindBuffer(0x8892, triangle_vbo_);
    gl_extra->glBufferData(
        0x8892,
        static_cast<int>(render_triangles.size() * sizeof(ColoredVertex)),
        render_triangles.data(), 0x88E0);
    gl_extra->glEnableVertexAttribArray(0);
    gl_extra->glVertexAttribPointer(0, 3, 0x1406, 0, sizeof(ColoredVertex),
                                    reinterpret_cast<void*>(0));
    gl_extra->glEnableVertexAttribArray(1);
    gl_extra->glVertexAttribPointer(1, 4, 0x1406, 0, sizeof(ColoredVertex),
                                    reinterpret_cast<void*>(sizeof(QVector3D)));
    gl->glDrawArrays(0x0004, 0, static_cast<int>(render_triangles.size()));
    gl_extra->glBindVertexArray(0);
  }

  uploadIfNeeded();

  renderTexturedBatches(view, mvp);
  renderPbrMesh(mvp, view);
  renderPbrTexturedMeshes(mvp, view);

  if (!line_vertices_.empty()) {
    gl->glUseProgram(line_program_);
    gl_extra->glUniformMatrix4fv(gl->glGetUniformLocation(line_program_, "uMvp"), 1,
                                 0, mvp.constData());
    gl_extra->glBindVertexArray(line_vao_);
    gl->glDrawArrays(0x0001, 0, static_cast<int>(line_vertices_.size()));
    gl_extra->glBindVertexArray(0);
  }

  if (!point_vertices_.empty() && point_size_ <= 1.f) {
    gl->glEnable(0x8642);  // GL_PROGRAM_POINT_SIZE
    gl->glUseProgram(point_program_);
    gl_extra->glUniformMatrix4fv(gl->glGetUniformLocation(point_program_, "uMvp"), 1,
                                 0, mvp.constData());
    gl_extra->glUniform1f(gl->glGetUniformLocation(point_program_, "uPointSize"),
                          point_size_);
    gl_extra->glBindVertexArray(point_vao_);
    gl->glDrawArrays(0x0000, 0, static_cast<int>(point_vertices_.size()));
    gl_extra->glBindVertexArray(0);
  }
}

void SceneOverlay::render(const CameraState& camera, float aspect_ratio) {
  render(camera.viewMatrix(), camera.projectionMatrix(aspect_ratio));
}

void SceneOverlay::appendPickPointSpriteBatch(
    const QMatrix4x4& view, std::vector<PickVertex>* triangles) const {
  if (triangles == nullptr || pick_point_vertices_.empty()) {
    return;
  }
  const QMatrix4x4 inv_view = view.inverted();
  const QVector3D camera_right = inv_view.column(0).toVector3D().normalized();
  const QVector3D camera_up = inv_view.column(1).toVector3D().normalized();
  const float half = std::max(0.004f, point_size_ * 0.012f);
  triangles->reserve(triangles->size() + pick_point_vertices_.size() * 6);
  for (const PickVertex& point : pick_point_vertices_) {
    const QVector3D c = point.position;
    const QVector3D tl = c - camera_right * half + camera_up * half;
    const QVector3D tr = c + camera_right * half + camera_up * half;
    const QVector3D br = c + camera_right * half - camera_up * half;
    const QVector3D bl = c - camera_right * half - camera_up * half;
    triangles->push_back({tl, point.handle});
    triangles->push_back({tr, point.handle});
    triangles->push_back({br, point.handle});
    triangles->push_back({tl, point.handle});
    triangles->push_back({br, point.handle});
    triangles->push_back({bl, point.handle});
  }
}

void SceneOverlay::renderPickPass(const QMatrix4x4& view,
                                  const QMatrix4x4& projection) {
  if (!initialized_ || pick_program_ == 0 || pick_point_vertices_.empty()) {
    return;
  }
  QOpenGLFunctions* gl = QOpenGLContext::currentContext()->functions();
  QOpenGLExtraFunctions* gl_extra =
      QOpenGLContext::currentContext()->extraFunctions();
  const QMatrix4x4 mvp = projection * view;

  std::vector<PickVertex> billboard_tris;
  appendPickPointSpriteBatch(view, &billboard_tris);
  std::vector<GpuPickVertex> gpu_vertices;
  gpu_vertices.reserve(billboard_tris.size());
  for (const PickVertex& vertex : billboard_tris) {
    gpu_vertices.push_back({vertex.position, HandleToVec3(vertex.handle)});
  }
  if (gpu_vertices.empty()) {
    return;
  }
  gl->glEnable(0x0B71);
  gl->glDepthMask(0x0001);
  gl->glUseProgram(pick_program_);
  gl_extra->glUniformMatrix4fv(
      gl->glGetUniformLocation(pick_program_, "uMvp"), 1, 0, mvp.constData());
  gl_extra->glBindVertexArray(pick_vao_);
  gl_extra->glBindBuffer(0x8892, pick_vbo_);
  gl_extra->glBufferData(
      0x8892, static_cast<int>(gpu_vertices.size() * sizeof(GpuPickVertex)),
      gpu_vertices.data(), 0x88E0);
  gl_extra->glEnableVertexAttribArray(0);
  gl_extra->glVertexAttribPointer(0, 3, 0x1406, 0, sizeof(GpuPickVertex),
                                  reinterpret_cast<void*>(0));
  gl_extra->glEnableVertexAttribArray(1);
  gl_extra->glVertexAttribPointer(1, 3, 0x1406, 0, sizeof(GpuPickVertex),
                                  reinterpret_cast<void*>(sizeof(QVector3D)));
  gl->glDrawArrays(0x0004, 0, static_cast<int>(gpu_vertices.size()));
  gl_extra->glBindVertexArray(0);
}

void SceneOverlay::shutdown() {
  if (!initialized_) {
    return;
  }
  QOpenGLExtraFunctions* gl =
      QOpenGLContext::currentContext()->extraFunctions();
  QOpenGLFunctions* gl_base = QOpenGLContext::currentContext()->functions();
  if (line_vao_ != 0) {
    gl->glDeleteVertexArrays(1, reinterpret_cast<unsigned*>(&line_vao_));
    line_vao_ = 0;
  }
  if (line_vbo_ != 0) {
    gl->glDeleteBuffers(1, reinterpret_cast<unsigned*>(&line_vbo_));
    line_vbo_ = 0;
  }
  if (point_vao_ != 0) {
    gl->glDeleteVertexArrays(1, reinterpret_cast<unsigned*>(&point_vao_));
    point_vao_ = 0;
  }
  if (point_vbo_ != 0) {
    gl->glDeleteBuffers(1, reinterpret_cast<unsigned*>(&point_vbo_));
    point_vbo_ = 0;
  }
  if (triangle_vao_ != 0) {
    gl->glDeleteVertexArrays(1, reinterpret_cast<unsigned*>(&triangle_vao_));
    triangle_vao_ = 0;
  }
  if (triangle_vbo_ != 0) {
    gl->glDeleteBuffers(1, reinterpret_cast<unsigned*>(&triangle_vbo_));
    triangle_vbo_ = 0;
  }
  if (line_program_ != 0) {
    gl_base->glDeleteProgram(line_program_);
    line_program_ = 0;
  }
  if (point_program_ != 0) {
    gl_base->glDeleteProgram(point_program_);
    point_program_ = 0;
  }
  if (triangle_program_ != 0) {
    gl_base->glDeleteProgram(triangle_program_);
    triangle_program_ = 0;
  }
  if (textured_vao_ != 0) {
    gl->glDeleteVertexArrays(1, reinterpret_cast<unsigned*>(&textured_vao_));
    textured_vao_ = 0;
  }
  if (textured_vbo_ != 0) {
    gl->glDeleteBuffers(1, reinterpret_cast<unsigned*>(&textured_vbo_));
    textured_vbo_ = 0;
  }
  if (textured_program_ != 0) {
    gl_base->glDeleteProgram(textured_program_);
    textured_program_ = 0;
  }
  if (pbr_program_ != 0) {
    gl_base->glDeleteProgram(pbr_program_);
    pbr_program_ = 0;
  }
  if (pbr_vao_ != 0) {
    gl->glDeleteVertexArrays(1, reinterpret_cast<unsigned*>(&pbr_vao_));
    pbr_vao_ = 0;
  }
  if (pbr_vbo_ != 0) {
    gl->glDeleteBuffers(1, reinterpret_cast<unsigned*>(&pbr_vbo_));
    pbr_vbo_ = 0;
  }
  if (pbr_textured_program_ != 0) {
    gl_base->glDeleteProgram(pbr_textured_program_);
    pbr_textured_program_ = 0;
  }
  if (pbr_textured_vao_ != 0) {
    gl->glDeleteVertexArrays(1, reinterpret_cast<unsigned*>(&pbr_textured_vao_));
    pbr_textured_vao_ = 0;
  }
  if (pbr_textured_vbo_ != 0) {
    gl->glDeleteBuffers(1, reinterpret_cast<unsigned*>(&pbr_textured_vbo_));
    pbr_textured_vbo_ = 0;
  }
  if (pick_program_ != 0) {
    gl_base->glDeleteProgram(pick_program_);
    pick_program_ = 0;
  }
  if (pick_vao_ != 0) {
    gl->glDeleteVertexArrays(1, reinterpret_cast<unsigned*>(&pick_vao_));
    pick_vao_ = 0;
  }
  if (pick_vbo_ != 0) {
    gl->glDeleteBuffers(1, reinterpret_cast<unsigned*>(&pick_vbo_));
    pick_vbo_ = 0;
  }
  initialized_ = false;
}

}  // namespace rendering
}  // namespace autoviz
