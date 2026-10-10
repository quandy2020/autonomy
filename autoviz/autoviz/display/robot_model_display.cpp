/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#include "autoviz/display/robot_model_display.hpp"

#include <algorithm>

#include <QColor>
#include <QFileInfo>
#include <QImage>
#include <QMatrix4x4>

#include "autolink/common/log.hpp"
#include <automsgs/msgs/builtin_interfaces/time.pb.h>
#include <automsgs/msgs/sensor_msgs/joint_state.pb.h>
#include <automsgs/msgs/std_msgs/string.pb.h>
#include "autoviz/commsgs/time_utils.hpp"
#include "autoviz/integration/channel_payload.hpp"
#include "autoviz/display/obj_mesh.hpp"
#include "autoviz/display/ogre_entity_draw.hpp"
#include "autoviz/display/ogre_mesh_draw.hpp"
#include "autoviz/display/ogre_overlay_draw.hpp"
#include "autoviz/display/ogre_pbr_mesh_draw.hpp"
#include "autoviz/display/primitive_mesh.hpp"
#include "autoviz/display/transform_utils.hpp"

namespace autoviz {
namespace display {
namespace {

QVector3D VisualHalfExtents(const UrdfGeometry& geometry) {
  switch (geometry.type) {
    case UrdfGeometry::Type::kBox:
      return geometry.size * 0.5f;
    case UrdfGeometry::Type::kCylinder:
      return QVector3D(geometry.size.x(), geometry.size.x(),
                       geometry.size.y() * 0.5f);
    case UrdfGeometry::Type::kSphere:
      return QVector3D(geometry.size.x(), geometry.size.x(), geometry.size.x());
    default:
      return QVector3D(0.05f, 0.05f, 0.05f);
  }
}

bool AppendUrdfGeometryMesh(std::vector<ColoredMeshInstance>* meshes,
                            const UrdfGeometry& geometry, const ObjMesh* mesh,
                            const QMatrix4x4& link_transform,
                            const QColor& color, bool wireframe) {
  if (meshes == nullptr) {
    return false;
  }
  QMatrix4x4 geom_transform = link_transform;
  geom_transform.translate(geometry.origin);
  geom_transform.rotate(geometry.rotation);

  if (geometry.type == UrdfGeometry::Type::kBox) {
    const QVector3D half = VisualHalfExtents(geometry);
    QMatrix4x4 box = geom_transform;
    box.scale(half.x() * 2.f, half.y() * 2.f, half.z() * 2.f);
    meshes->push_back({buildCubeMesh(), box, color, wireframe});
    return true;
  }
  if (mesh != nullptr && (geometry.type == UrdfGeometry::Type::kMesh ||
                          geometry.type == UrdfGeometry::Type::kCylinder ||
                          geometry.type == UrdfGeometry::Type::kSphere)) {
    QMatrix4x4 mesh_transform = geom_transform;
    if (geometry.type == UrdfGeometry::Type::kMesh) {
      mesh_transform.scale(geometry.mesh_scale);
    }
    meshes->push_back({*mesh, mesh_transform, color, wireframe});
    return true;
  }
  return false;
}

bool AppendUrdfGeometryPbr(std::vector<PbrMeshInstance>* pbr_meshes,
                           std::vector<PbrTexturedMeshInstance>* textured_meshes,
                           const UrdfGeometry& geometry, const ObjMesh* mesh,
                           const QMatrix4x4& link_transform, const QColor& color,
                           const QImage& texture, float metallic,
                           float roughness) {
  if (pbr_meshes == nullptr || textured_meshes == nullptr) {
    return false;
  }
  QMatrix4x4 geom_transform = link_transform;
  geom_transform.translate(geometry.origin);
  geom_transform.rotate(geometry.rotation);

  if (geometry.type == UrdfGeometry::Type::kBox) {
    const QVector3D half = VisualHalfExtents(geometry);
    if (!texture.isNull()) {
      ObjMesh box_mesh;
      box_mesh.vertices = {
          QVector3D(-half.x(), -half.y(), -half.z()),
          QVector3D(half.x(), -half.y(), -half.z()),
          QVector3D(half.x(), half.y(), -half.z()),
          QVector3D(-half.x(), half.y(), -half.z()),
          QVector3D(-half.x(), -half.y(), half.z()),
          QVector3D(half.x(), -half.y(), half.z()),
          QVector3D(half.x(), half.y(), half.z()),
          QVector3D(-half.x(), half.y(), half.z()),
      };
      ensureMeshTexcoords(&box_mesh);
      box_mesh.triangles = {{0, 1, 2}, {0, 2, 3}, {4, 6, 5}, {4, 7, 6},
                            {0, 4, 5}, {0, 5, 1}, {2, 6, 7}, {2, 7, 3},
                            {0, 3, 7}, {0, 7, 4}, {1, 5, 6}, {1, 6, 2}};
      textured_meshes->push_back(
          {std::move(box_mesh), geom_transform, texture, color, metallic, roughness});
    } else {
      QMatrix4x4 box = geom_transform;
      box.scale(half.x() * 2.f, half.y() * 2.f, half.z() * 2.f);
      pbr_meshes->push_back({buildCubeMesh(), box, color, metallic, roughness});
    }
    return true;
  }
  if (mesh != nullptr && (geometry.type == UrdfGeometry::Type::kMesh ||
                          geometry.type == UrdfGeometry::Type::kCylinder ||
                          geometry.type == UrdfGeometry::Type::kSphere)) {
    QMatrix4x4 mesh_transform = geom_transform;
    if (geometry.type == UrdfGeometry::Type::kMesh) {
      mesh_transform.scale(geometry.mesh_scale);
    }
    if (!texture.isNull()) {
      textured_meshes->push_back(
          {*mesh, mesh_transform, texture, color, metallic, roughness});
    } else {
      pbr_meshes->push_back({*mesh, mesh_transform, color, metallic, roughness});
    }
    return true;
  }
  return false;
}

}  // namespace

RobotModelDisplay::RobotModelDisplay(std::string joint_channel)
    : joint_channel_(std::move(joint_channel)) {
  setProperties({});
  description_channel_ =
      propertyValue("description_channel", "/robot_description");
}

void RobotModelDisplay::setChannel(const std::string& channel) {
  if (joint_channel_ == channel) {
    return;
  }
  const bool active = enabled();
  if (active) {
    onDisable();
  }
  joint_channel_ = channel;
  if (active) {
    onEnable();
  }
}

std::vector<common::DisplayPropertySpec> RobotModelDisplay::propertySpecs()
    const {
  return {{"description_source", "Description Source", "Topic",
           {"Topic", "File"}},
          {"urdf_path", "Description File", "", {}, common::DisplayPropertyKind::kPath},
          {"description_channel", "Description Topic", "/robot_description", {},
           common::DisplayPropertyKind::kChannel},
          {"tf_prefix", "TF Prefix", "", {}},
          {"update_interval", "Update Interval", "0"},
          {"root_link", "Root Link", "", {}},
          {"alpha", "Alpha", "1.0"},
          {"visual_enabled", "Visual Enabled", "true"},
          {"color", "Color", "180;180;180", {}, common::DisplayPropertyKind::kColor},
          {"visual_style", "Visual Style", "solid", {"solid", "wireframe"}},
          {"show_axes", "Show Axes", "true", {}},
          {"use_urdf_materials", "Use URDF Materials", "true", {}},
          {"show_collision", "Show Collision", "false", {}},
          {"collision_alpha", "Collision Alpha", "0.35", {}}};
}

void RobotModelDisplay::onPropertyChanged(const std::string& key) {
  if (key == "urdf_path") {
    reloadUrdf();
    return;
  }
  if (key != "description_channel") {
    return;
  }
  const std::string next =
      propertyValue("description_channel", "/robot_description");
  if (next == description_channel_) {
    return;
  }
  description_channel_ = next;
  if (enabled()) {
    onDisable();
    onEnable();
  }
}

void RobotModelDisplay::reloadUrdf() {
  const std::string path = propertyValue("urdf_path", "");
  visual_meshes_.clear();
  collision_meshes_.clear();
  texture_cache_.clear();
  if (!path.empty()) {
    model_.loadFromFile(path);
    rebuildMeshCache();
  }
}

void RobotModelDisplay::cacheGeometryMesh(
    const UrdfGeometry& geometry, const std::string& link_name,
    std::unordered_map<std::string, ObjMesh>* cache) {
  if (cache == nullptr || geometry.type == UrdfGeometry::Type::kUnknown) {
    return;
  }
  if (geometry.type == UrdfGeometry::Type::kMesh &&
      !geometry.mesh_filename.empty()) {
    const std::string mesh_path = UrdfModel::resolveMeshPath(
        model_.baseDirectory(), geometry.mesh_filename);
    ObjMesh mesh;
    if (loadMeshFile(mesh_path, &mesh)) {
      // These OBJs were written by Assimp in Y-up. The URDF measures the same
      // parts in Z-up (thigh length and the foot tube run along link Z, the
      // shell thickness along Z). Rx(+90°): (x, y, z) -> (x, -z, y).
      if (mesh_path.find("/legged_robot/meshes/") != std::string::npos) {
        for (QVector3D& vertex : mesh.vertices) {
          const float y = vertex.y();
          vertex.setY(-vertex.z());
          vertex.setZ(y);
        }
      }
      (*cache)[link_name] = std::move(mesh);
    }
  } else if (geometry.type == UrdfGeometry::Type::kCylinder) {
    (*cache)[link_name] =
        buildCylinderMesh(geometry.size.x(), geometry.size.y());
  } else if (geometry.type == UrdfGeometry::Type::kSphere) {
    (*cache)[link_name] = buildSphereMesh(geometry.size.x());
  }
}

void RobotModelDisplay::rebuildMeshCache() {
  visual_meshes_.clear();
  collision_meshes_.clear();
  for (const auto& link : model_.links()) {
    for (std::size_t i = 0; i < link.visuals.size(); ++i) {
      cacheGeometryMesh(link.visuals[i], link.name + "#" + std::to_string(i),
                        &visual_meshes_);
    }
    if (link.visuals.empty() && link.has_visual) {
      cacheGeometryMesh(link.visual, link.name, &visual_meshes_);
    }
    if (link.has_collision) {
      cacheGeometryMesh(link.collision, link.name, &collision_meshes_);
    }
  }
}

void RobotModelDisplay::subscribeChannels() {
  if (context_ == nullptr || context_->autolink == nullptr ||
      context_->autolink->node() == nullptr) {
    return;
  }
  const bool shared_channel =
      !description_channel_.empty() && description_channel_ == joint_channel_;
  auto& registry = integration::ChannelReaderRegistry::instance();
  if (joint_subscription_ == 0 && !joint_channel_.empty()) {
    joint_subscription_ = registry.subscribe(
        joint_channel_,
        [this, shared_channel](const std::string& payload) {
          if (shared_channel) {
            description_queue_.push(payload);
          }
          joint_queue_.push(payload);
        });
  }
  if (description_subscription_ == 0 && !description_channel_.empty() &&
      !shared_channel) {
    description_subscription_ = registry.subscribe(
        description_channel_,
        [this](const std::string& payload) {
          description_queue_.push(payload);
        });
  }
}

void RobotModelDisplay::onEnable() {
  if (context_ == nullptr || context_->autolink == nullptr ||
      context_->autolink->node() == nullptr) {
    setStatusError("Autolink not ready");
    return;
  }
  reloadUrdf();
  description_channel_ =
      propertyValue("description_channel", "/robot_description");
  subscribeChannels();
  if (joint_subscription_ == 0 && !joint_channel_.empty()) {
    setStatusError("Failed to subscribe joint states");
  }
}

void RobotModelDisplay::onDisable() {
  auto& registry = integration::ChannelReaderRegistry::instance();
  if (joint_subscription_ != 0) {
    registry.unsubscribe(joint_subscription_);
    joint_subscription_ = 0;
  }
  if (description_subscription_ != 0) {
    registry.unsubscribe(description_subscription_);
    description_subscription_ = 0;
  }
}

void RobotModelDisplay::reset() {
  Display::reset();
  joint_queue_.clear();
  description_queue_.clear();
  joint_positions_.clear();
  if (context_ != nullptr && context_->request_redraw) {
    context_->request_redraw();
  }
}

void RobotModelDisplay::onUpdate() {
  if (joint_subscription_ == 0 ||
      (description_subscription_ == 0 && !description_channel_.empty() &&
       description_channel_ != joint_channel_)) {
    subscribeChannels();
  }
  while (auto payload = description_queue_.pop()) {
    const std::string decoded = integration::DecodeChannelPayload(*payload);
    automsgs::msgs::std_msgs::String message;
    if (message.ParseFromString(decoded) || message.ParseFromString(*payload)) {
      processDescription(message.data());
    } else {
      setStatusWarn("Failed to parse robot description");
    }
  }
  while (auto payload = joint_queue_.pop()) {
    const std::string decoded = integration::DecodeChannelPayload(*payload);
    proto_wire::ParsedJointState parsed;
    if (ParseJointStatePayload(decoded, &parsed) ||
        ParseJointStatePayload(*payload, &parsed)) {
      processJointState(parsed);
    }
  }
}

void RobotModelDisplay::processDescription(const std::string& urdf_text) {
  if (urdf_text.empty() || urdf_text == description_text_) {
    return;
  }
  bool loaded = false;
  if (urdf_text.find("<robot") == std::string::npos) {
    const QString path = QString::fromStdString(urdf_text).trimmed();
    if (QFileInfo::exists(path)) {
      loaded = model_.loadFromFile(path.toStdString());
    }
  } else {
    loaded = model_.loadFromString(urdf_text);
  }
  if (!loaded) {
    AERROR << "URDF parse failed (" << urdf_text.size() << " bytes)";
    setStatusError("URDF parse failed");
    return;
  }
  description_text_ = urdf_text;
  rebuildMeshCache();
  bool mesh_missing = false;
  for (const auto& link : model_.links()) {
    if (!link.visuals.empty()) {
      for (std::size_t i = 0; i < link.visuals.size(); ++i) {
        if (link.visuals[i].type == UrdfGeometry::Type::kMesh &&
            visual_meshes_.count(link.name + "#" + std::to_string(i)) == 0) {
          mesh_missing = true;
        }
      }
    } else if (link.has_visual &&
               link.visual.type == UrdfGeometry::Type::kMesh &&
               visual_meshes_.count(link.name) == 0) {
      mesh_missing = true;
    }
  }
  if (mesh_missing) {
    AWARN << "URDF mesh file was not found, links=" << model_.links().size();
    setStatusWarn("URDF mesh file was not found");
  } else {
    AINFO << "robot model loaded, links=" << model_.links().size()
          << " meshes=" << visual_meshes_.size();
    setStatusOk("Model loaded");
  }
  if (context_ != nullptr && context_->request_redraw) {
    context_->request_redraw();
  }
}

void RobotModelDisplay::processJointState(
    const proto_wire::ParsedJointState& message) {
  for (std::size_t i = 0; i < message.names.size() && i < message.positions.size();
       ++i) {
    joint_positions_[message.names[i]] = message.positions[i];
  }
  if (context_ != nullptr && context_->request_redraw) {
    context_->request_redraw();
  }
}

QColor RobotModelDisplay::linkColor(const UrdfGeometry& geometry,
                                    const QColor& fallback, float alpha,
                                    bool use_urdf_materials) const {
  QColor color = fallback;
  if (use_urdf_materials && geometry.material.valid) {
    color = QColor::fromRgbF(geometry.material.r, geometry.material.g,
                             geometry.material.b,
                             geometry.material.a * alpha);
  } else {
    color.setAlphaF(alpha);
  }
  return color;
}

QImage RobotModelDisplay::loadMaterialTexture(const UrdfMaterial& material) const {
  if (!material.has_texture || material.texture_filename.empty()) {
    return {};
  }
  const std::string resolved = UrdfModel::resolveTexturePath(
      model_.baseDirectory(), material.texture_filename);
  const auto cached = texture_cache_.find(resolved);
  if (cached != texture_cache_.end()) {
    return cached->second;
  }
  QImage image(QString::fromStdString(resolved));
  if (!image.isNull()) {
    texture_cache_[resolved] = image;
  }
  return image;
}

void RobotModelDisplay::drawLinkGeometry(
    rendering::SceneOverlay& scene, const UrdfGeometry& geometry,
    const ObjMesh* mesh, const QMatrix4x4& link_transform, const QColor& color,
    bool solid_visual, bool use_pbr) const {
  QMatrix4x4 geom_transform = link_transform;
  geom_transform.translate(geometry.origin);
  geom_transform.rotate(geometry.rotation);

  const float metallic = geometry.material.metallic;
  const float roughness = geometry.material.roughness;

  if (geometry.type == UrdfGeometry::Type::kMesh ||
      geometry.type == UrdfGeometry::Type::kCylinder ||
      geometry.type == UrdfGeometry::Type::kSphere) {
    if (mesh != nullptr) {
      QMatrix4x4 mesh_transform = geom_transform;
      if (geometry.type == UrdfGeometry::Type::kMesh) {
        mesh_transform.scale(geometry.mesh_scale);
      }
      if (solid_visual && mesh->has_material_groups) {
        for (const ObjSubmesh& part : mesh->submeshes) {
          if (part.triangle_count <= 0) {
            continue;
          }
          const int begin = part.triangle_begin;
          const int end = begin + part.triangle_count;
          if (begin < 0 || end > static_cast<int>(mesh->triangles.size())) {
            continue;
          }
          ObjMesh part_mesh;
          part_mesh.vertices = mesh->vertices;
          part_mesh.texcoords = mesh->texcoords;
          part_mesh.triangles.assign(mesh->triangles.begin() + begin,
                                     mesh->triangles.begin() + end);
          QImage texture;
          if (!part.texture_path.empty()) {
            const auto cached = texture_cache_.find(part.texture_path);
            if (cached != texture_cache_.end()) {
              texture = cached->second;
            } else {
              texture.load(QString::fromStdString(part.texture_path));
              texture_cache_[part.texture_path] = texture;
            }
          }
          if (!texture.isNull()) {
            scene.addTriangleMeshTexturedPbr(part_mesh, mesh_transform, texture,
                                             color, metallic, roughness);
          } else {
            QColor solid = part.diffuse;
            solid.setRedF(std::clamp(solid.redF() * color.redF(), 0.f, 1.f));
            solid.setGreenF(std::clamp(solid.greenF() * color.greenF(), 0.f, 1.f));
            solid.setBlueF(std::clamp(solid.blueF() * color.blueF(), 0.f, 1.f));
            solid.setAlphaF(color.alphaF());
            scene.addTriangleMeshSolid(part_mesh, mesh_transform, solid);
          }
        }
      } else if (solid_visual) {
        const QImage texture = use_pbr ? loadMaterialTexture(geometry.material) : QImage();
        if (use_pbr && !texture.isNull()) {
          scene.addTriangleMeshTexturedPbr(*mesh, mesh_transform, texture, color,
                                           metallic, roughness);
        } else if (use_pbr) {
          scene.addTriangleMeshSolidPbr(*mesh, mesh_transform, color, metallic,
                                        roughness);
        } else {
          scene.addTriangleMeshSolid(*mesh, mesh_transform, color);
        }
      } else {
        scene.addTriangleMeshWireframe(*mesh, mesh_transform, color);
      }
    }
  } else if (geometry.type == UrdfGeometry::Type::kBox) {
    const QVector3D half = VisualHalfExtents(geometry);
    if (solid_visual) {
      const QImage texture = use_pbr ? loadMaterialTexture(geometry.material) : QImage();
      if (use_pbr && !texture.isNull()) {
        ObjMesh box_mesh;
        box_mesh.vertices = {
            QVector3D(-half.x(), -half.y(), -half.z()),
            QVector3D(half.x(), -half.y(), -half.z()),
            QVector3D(half.x(), half.y(), -half.z()),
            QVector3D(-half.x(), half.y(), -half.z()),
            QVector3D(-half.x(), -half.y(), half.z()),
            QVector3D(half.x(), -half.y(), half.z()),
            QVector3D(half.x(), half.y(), half.z()),
            QVector3D(-half.x(), half.y(), half.z()),
        };
        ensureMeshTexcoords(&box_mesh);
        box_mesh.triangles = {{0, 1, 2}, {0, 2, 3}, {4, 6, 5}, {4, 7, 6},
                              {0, 4, 5}, {0, 5, 1}, {2, 6, 7}, {2, 7, 3},
                              {0, 3, 7}, {0, 7, 4}, {1, 5, 6}, {1, 6, 2}};
        scene.addTriangleMeshTexturedPbr(box_mesh, geom_transform, texture, color,
                                         metallic, roughness);
      } else if (use_pbr) {
        scene.addBoxSolidPbr(QVector3D(0.f, 0.f, 0.f), half, geom_transform,
                             color, metallic, roughness);
      } else {
        scene.addBoxSolid(QVector3D(0.f, 0.f, 0.f), half, geom_transform, color);
      }
    } else {
      scene.addBoxWireframe(QVector3D(0.f, 0.f, 0.f), half, geom_transform,
                            color);
    }
  }
}

void RobotModelDisplay::onDraw(rendering::SceneOverlay& scene) {
  if (context_ == nullptr || model_.empty()) {
    return;
  }

  const QColor base_color =
      common::ParseColorProperty(propertyValue("color", "180;180;180"));
  const float alpha =
      common::ParseFloatProperty(propertyValue("alpha", "0.8"), 0.8f);
  QColor color = base_color;
  color.setAlphaF(alpha);
  const bool show_axes =
      common::ParseBoolProperty(propertyValue("show_axes", "true"), true);
  const bool solid_visual =
      propertyValue("visual_style", "solid") != "wireframe";
  const bool use_urdf_materials = common::ParseBoolProperty(
      propertyValue("use_urdf_materials", "true"), true);
  const bool show_collision = common::ParseBoolProperty(
      propertyValue("show_collision", "false"), false);
  const float collision_alpha = common::ParseFloatProperty(
      propertyValue("collision_alpha", "0.35"), 0.35f);
  QColor collision_color(255, 140, 40);
  collision_color.setAlphaF(collision_alpha);

  const bool use_ogre = context_->ogre_scene_host != nullptr;

  std::vector<ColoredMeshInstance> ogre_visual;
  std::vector<PbrMeshInstance> ogre_pbr_visual;
  std::vector<PbrTexturedMeshInstance> ogre_pbr_textured;
  std::vector<ColoredMeshInstance> ogre_collision;
  std::vector<LineSegment3D> ogre_axes;

  const auto link_transforms = model_.computeLinkTransforms(joint_positions_);

  QMatrix4x4 root_world;
  root_world.setToIdentity();
  // Joint states plus one floating-base TF. An empty Root Link property must
  // still follow the URDF root: looking up only "base" and leaving every other
  // link at the origin draws the robot in pieces.
  std::string kinematic_root = propertyValue("root_link", "");
  if (kinematic_root.empty()) {
    kinematic_root = model_.rootLink();
  }
  const bool have_buffer = context_->tf_buffer != nullptr;
  const auto zero_time = autoviz::commsgs::ZeroTime();
  if (!kinematic_root.empty() && have_buffer) {
    try {
      root_world = transformToMatrix(context_->tf_buffer->lookupTransform(
          context_->fixed_frame, kinematic_root, zero_time));
    } catch (...) {
    }
  }

  for (const auto& link : model_.links()) {
    const auto tf_it = link_transforms.find(link.name);
    if (tf_it == link_transforms.end()) {
      continue;
    }
    const QMatrix4x4 link_transform = root_world * tf_it->second;

    std::vector<UrdfGeometry> fallback_visuals;
    const std::vector<UrdfGeometry>* visuals = &link.visuals;
    if (visuals->empty() && link.has_visual) {
      fallback_visuals.push_back(link.visual);
      visuals = &fallback_visuals;
    }
    for (std::size_t visual_index = 0; visual_index < visuals->size();
         ++visual_index) {
      const UrdfGeometry& geometry = (*visuals)[visual_index];
      const std::string mesh_key =
          link.visuals.empty() ? link.name
                               : link.name + "#" + std::to_string(visual_index);
      const ObjMesh* mesh = nullptr;
      const auto mesh_it = visual_meshes_.find(mesh_key);
      if (mesh_it != visual_meshes_.end()) {
        mesh = &mesh_it->second;
      }
      const QColor visual_color =
          linkColor(geometry, color, alpha, use_urdf_materials);
      const bool material_requested = solid_visual && use_urdf_materials &&
                                     (geometry.material.valid ||
                                      geometry.material.has_texture);
      const QImage texture =
          material_requested ? loadMaterialTexture(geometry.material)
                             : QImage();
      // Textured meshes use the rviz-style unlit texture path. A material
      // without a loaded image stays on the flat vertex-color path, which is
      // the same one markers use.
      const bool use_pbr = !texture.isNull();
      const float metallic = geometry.material.metallic;
      const float roughness = geometry.material.roughness;
      bool ogre_visual_ok = false;
      if (use_ogre && use_pbr) {
        ogre_visual_ok =
            AppendUrdfGeometryPbr(&ogre_pbr_visual, &ogre_pbr_textured,
                                    geometry, mesh, link_transform,
                                    visual_color, texture, metallic, roughness);
      } else if (use_ogre) {
        ogre_visual_ok =
            AppendUrdfGeometryMesh(&ogre_visual, geometry, mesh, link_transform,
                                   visual_color, !solid_visual);
      }
      if (!ogre_visual_ok) {
        drawLinkGeometry(scene, geometry, mesh, link_transform, visual_color,
                         solid_visual, use_pbr);
      }
    }

    if (show_collision && link.has_collision) {
      const ObjMesh* mesh = nullptr;
      const auto mesh_it = collision_meshes_.find(link.name);
      if (mesh_it != collision_meshes_.end()) {
        mesh = &mesh_it->second;
      }
      const bool ogre_collision_ok =
          use_ogre &&
          AppendUrdfGeometryMesh(&ogre_collision, link.collision, mesh,
                                 link_transform, collision_color, true);
      if (!ogre_collision_ok) {
        drawLinkGeometry(scene, link.collision, mesh, link_transform,
                         collision_color, false, false);
      }
    }

    if (show_axes && visuals != nullptr && !visuals->empty()) {
      const UrdfGeometry& geometry = visuals->front();
      QMatrix4x4 visual = link_transform;
      visual.translate(geometry.origin);
      visual.rotate(geometry.rotation);
      const QVector3D half = VisualHalfExtents(geometry);
      const QVector3D origin = visual.map(QVector3D(0.f, 0.f, 0.f));
      const float axis_len = std::max(half.length(), 0.08f);
      const QColor red(220, 60, 60, static_cast<int>(alpha * 255));
      const QColor green(60, 220, 60, static_cast<int>(alpha * 255));
      const QColor blue(60, 120, 220, static_cast<int>(alpha * 255));
      if (use_ogre) {
        ogre_axes.push_back(
            {origin, visual.map(QVector3D(axis_len, 0.f, 0.f)), red});
        ogre_axes.push_back(
            {origin, visual.map(QVector3D(0.f, axis_len, 0.f)), green});
        ogre_axes.push_back(
            {origin, visual.map(QVector3D(0.f, 0.f, axis_len)), blue});
      } else {
        scene.addLine(origin, visual.map(QVector3D(axis_len, 0.f, 0.f)), red);
        scene.addLine(origin, visual.map(QVector3D(0.f, axis_len, 0.f)), green);
        scene.addLine(origin, visual.map(QVector3D(0.f, 0.f, axis_len)), blue);
      }
    }
  }

  if (use_ogre) {
    drawEntityMeshesOgreOrGl(context_, scene, name() + "/visual", ogre_visual);
    drawPbrMeshesOgreOrGl(context_, scene, name() + "/visual/pbr", ogre_pbr_visual);
    drawPbrTexturedMeshesOgreOrGl(context_, scene, name() + "/visual/pbr_tex",
                                  ogre_pbr_textured);
    if (show_collision) {
      drawEntityMeshesOgreOrGl(context_, scene, name() + "/collision", ogre_collision);
    }
    if (show_axes) {
      drawLineSegmentsOgreOrGl(context_, scene, name() + "/axes", ogre_axes);
    }
  }
}

}  // namespace display
}  // namespace autoviz
