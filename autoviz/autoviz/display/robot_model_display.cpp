/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#include "autoviz/display/robot_model_display.hpp"

#include <QColor>
#include <QFileInfo>
#include <QImage>
#include <QMatrix4x4>

#include "autolink/message/raw_message.hpp"
#include <automsgs/msgs/builtin_interfaces/time.pb.h>
#include <automsgs/msgs/sensor_msgs/joint_state.pb.h>
#include <automsgs/msgs/std_msgs/string.pb.h>
#include "autoviz/commsgs/time_utils.hpp"
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
  if (key == "urdf_path" || key == "description_channel") {
    description_channel_ = propertyValue("description_channel", "/robot_description");
    reloadUrdf();
    if (enabled()) {
      onDisable();
      onEnable();
    }
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
    if (link.has_visual) {
      cacheGeometryMesh(link.visual, link.name, &visual_meshes_);
    }
    if (link.has_collision) {
      cacheGeometryMesh(link.collision, link.name, &collision_meshes_);
    }
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

  joint_reader_ =
      context_->autolink->node()->CreateReader<autolink::message::RawMessage>(
          joint_channel_,
          [this](const std::shared_ptr<autolink::message::RawMessage>& msg) {
            if (msg != nullptr) {
              joint_queue_.push(msg->message);
            }
          });

  if (!description_channel_.empty()) {
    description_reader_ =
        context_->autolink->node()
            ->CreateReader<autolink::message::RawMessage>(
                description_channel_,
                [this](
                    const std::shared_ptr<autolink::message::RawMessage>& msg) {
                  if (msg != nullptr) {
                    description_queue_.push(msg->message);
                  }
                });
  }
}

void RobotModelDisplay::onDisable() {
  joint_reader_.reset();
  description_reader_.reset();
}

void RobotModelDisplay::onUpdate() {
  while (auto payload = description_queue_.pop()) {
    automsgs::msgs::std_msgs::String message;
    if (message.ParseFromString(*payload)) {
      processDescription(message.data());
    }
  }
  while (auto payload = joint_queue_.pop()) {
    proto_wire::ParsedJointState parsed;
    if (ParseJointStatePayload(*payload, &parsed)) {
      processJointState(parsed);
    }
  }
}

void RobotModelDisplay::processDescription(const std::string& urdf_text) {
  if (model_.loadFromString(urdf_text)) {
    if (context_ != nullptr && context_->request_redraw) {
      context_->request_redraw();
    }
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
      if (solid_visual) {
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

#ifdef AUTOVIZ_USE_OGRE
  const bool use_ogre = context_->ogre_scene_host != nullptr;
#else
  const bool use_ogre = false;
#endif

  std::vector<ColoredMeshInstance> ogre_visual;
  std::vector<PbrMeshInstance> ogre_pbr_visual;
  std::vector<PbrTexturedMeshInstance> ogre_pbr_textured;
  std::vector<ColoredMeshInstance> ogre_collision;
  std::vector<LineSegment3D> ogre_axes;

  const auto link_transforms = model_.computeLinkTransforms(joint_positions_);

  QMatrix4x4 root_world;
  root_world.setToIdentity();
  const std::string root_link = propertyValue("root_link", model_.rootLink());
  if (!root_link.empty() && context_->tf_buffer != nullptr) {
    const auto zero_time = autoviz::commsgs::ZeroTime();
    try {
      const auto tf = context_->tf_buffer->lookupTransform(
          context_->fixed_frame, root_link, zero_time);
      root_world = transformToMatrix(tf);
    } catch (...) {
    }
  }

  for (const auto& link : model_.links()) {
    const auto tf_it = link_transforms.find(link.name);
    if (tf_it == link_transforms.end()) {
      continue;
    }

    QMatrix4x4 link_transform = root_world * tf_it->second;

    if (link.has_visual) {
      const ObjMesh* mesh = nullptr;
      const auto mesh_it = visual_meshes_.find(link.name);
      if (mesh_it != visual_meshes_.end()) {
        mesh = &mesh_it->second;
      }
      const QColor visual_color =
          linkColor(link.visual, color, alpha, use_urdf_materials);
      const bool use_pbr = solid_visual && use_urdf_materials &&
                           (link.visual.material.valid ||
                            link.visual.material.has_texture);
      const QImage texture =
          use_pbr ? loadMaterialTexture(link.visual.material) : QImage();
      const float metallic = link.visual.material.metallic;
      const float roughness = link.visual.material.roughness;
      bool ogre_visual_ok = false;
      if (use_ogre && use_pbr) {
        ogre_visual_ok =
            AppendUrdfGeometryPbr(&ogre_pbr_visual, &ogre_pbr_textured,
                                    link.visual, mesh, link_transform,
                                    visual_color, texture, metallic, roughness);
      } else if (use_ogre) {
        ogre_visual_ok =
            AppendUrdfGeometryMesh(&ogre_visual, link.visual, mesh, link_transform,
                                   visual_color, !solid_visual);
      }
      if (!ogre_visual_ok) {
        drawLinkGeometry(scene, link.visual, mesh, link_transform, visual_color,
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

    if (show_axes && link.has_visual) {
      QMatrix4x4 visual = link_transform;
      visual.translate(link.visual.origin);
      visual.rotate(link.visual.rotation);
      const QVector3D half = VisualHalfExtents(link.visual);
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
