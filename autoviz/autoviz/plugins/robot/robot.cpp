/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *
 * Adapted from rviz_default_plugins robot::Robot
 * (Willow Garage / Bosch / Open Source Robotics Foundation).
 *****************************************************************************/

#include "autoviz/plugins/robot/robot.hpp"

#include <algorithm>
#include <sstream>

#include <QColor>
#include <QFileInfo>
#include <QImage>
#include <QMatrix4x4>

#include "autolink/common/log.hpp"
#include <automsgs/msgs/builtin_interfaces/time.pb.h>
#include "autoviz/commsgs/time_utils.hpp"
#include "autoviz/common/display_context.hpp"
#include "autoviz/common/display_property.hpp"
#include "autoviz/transform/buffer.hpp"
#include "autoviz/plugins/displays/common/obj_mesh.hpp"
#include "autoviz/plugins/displays/common/ogre_entity_draw.hpp"
#include "autoviz/plugins/displays/common/ogre_mesh_draw.hpp"
#include "autoviz/plugins/displays/common/ogre_overlay_draw.hpp"
#include "autoviz/plugins/displays/common/ogre_pbr_mesh_draw.hpp"
#include "autoviz/plugins/displays/common/primitive_mesh.hpp"
#include "autoviz/plugins/displays/common/transform_utils.hpp"
#include "autoviz/rendering/ogre_scene_host.hpp"
#include "autoviz/rendering/scene_overlay.hpp"

namespace autoviz {
namespace display {
namespace robot {
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
    static const ObjMesh kUnitCube = buildCubeMesh();
    const QVector3D half = VisualHalfExtents(geometry);
    QMatrix4x4 box = geom_transform;
    box.scale(half.x() * 2.f, half.y() * 2.f, half.z() * 2.f);
    meshes->push_back({&kUnitCube, box, color, wireframe});
    return true;
  }
  if (mesh != nullptr && (geometry.type == UrdfGeometry::Type::kMesh ||
                          geometry.type == UrdfGeometry::Type::kCylinder ||
                          geometry.type == UrdfGeometry::Type::kSphere)) {
    QMatrix4x4 mesh_transform = geom_transform;
    if (geometry.type == UrdfGeometry::Type::kMesh) {
      mesh_transform.scale(geometry.mesh_scale);
    }
    meshes->push_back({mesh, mesh_transform, color, wireframe});
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

Robot::~Robot() { clear(); }

void Robot::clear() {
  links_.clear();
  joints_.clear();
  model_ = UrdfModel{};
  visual_meshes_.clear();
  collision_meshes_.clear();
  texture_cache_.clear();
  joint_positions_.clear();
  geometry_errors_.clear();
}

void Robot::rebuildLinkJointMaps() {
  links_.clear();
  joints_.clear();
  for (const auto& link : model_.links()) {
    links_.emplace(link.name, std::make_unique<RobotLink>(this, link.name));
  }
  for (const auto& joint : model_.joints()) {
    joints_.emplace(joint.name, std::make_unique<RobotJoint>(this, joint));
  }
}

RobotLink* Robot::getLink(const std::string& name) {
  const auto it = links_.find(name);
  return it == links_.end() ? nullptr : it->second.get();
}

RobotJoint* Robot::getJoint(const std::string& name) {
  const auto it = joints_.find(name);
  return it == joints_.end() ? nullptr : it->second.get();
}

void Robot::setVisualVisible(bool visible) { visual_visible_ = visible; }

void Robot::setCollisionVisible(bool visible) { collision_visible_ = visible; }

void Robot::setAlpha(float alpha) {
  alpha_ = alpha;
  for (auto& [name, link] : links_) {
    (void)name;
    if (link != nullptr) {
      link->setRobotAlpha(alpha);
    }
  }
}

bool Robot::loadFromFile(const std::string& filepath) {
  visual_meshes_.clear();
  collision_meshes_.clear();
  texture_cache_.clear();
  geometry_errors_.clear();
  links_.clear();
  joints_.clear();
  if (filepath.empty() || !model_.loadFromFile(filepath)) {
    return false;
  }
  rebuildLinkJointMaps();
  rebuildMeshCache();
  return true;
}

bool Robot::loadFromString(const std::string& urdf_text) {
  if (urdf_text.empty()) {
    return false;
  }
  geometry_errors_.clear();
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
    return false;
  }
  rebuildLinkJointMaps();
  rebuildMeshCache();
  std::ostringstream ss;
  auto report_missing = [&](const UrdfGeometry& geometry,
                            const std::string& link_name) {
    if (geometry.type != UrdfGeometry::Type::kMesh ||
        geometry.mesh_filename.empty()) {
      return;
    }
    const std::string resolved = UrdfModel::resolveMeshPath(
        model_.baseDirectory(), geometry.mesh_filename);
    ss << "\n• for link '" << link_name << "': Could not load mesh resource '"
       << geometry.mesh_filename << "'";
    if (!resolved.empty() && resolved != geometry.mesh_filename) {
      ss << " (resolved: " << resolved << ")";
    }
  };
  for (const auto& link : model_.links()) {
    if (!link.visuals.empty()) {
      for (std::size_t i = 0; i < link.visuals.size(); ++i) {
        if (link.visuals[i].type == UrdfGeometry::Type::kMesh &&
            visual_meshes_.count(link.name + "#" + std::to_string(i)) == 0) {
          report_missing(link.visuals[i], link.name);
        }
      }
    } else if (link.has_visual &&
               link.visual.type == UrdfGeometry::Type::kMesh &&
               visual_meshes_.count(link.name) == 0) {
      report_missing(link.visual, link.name);
    }
  }
  geometry_errors_ = ss.str();
  return true;
}

void Robot::setJointPositions(
    const std::unordered_map<std::string, double>& positions) {
  for (const auto& [name, value] : positions) {
    joint_positions_[name] = value;
    if (RobotJoint* joint = getJoint(name)) {
      joint->setPosition(value);
    }
  }
}

void Robot::update(const LinkUpdater& updater) {
  for (auto& [name, link] : links_) {
    if (link == nullptr) {
      continue;
    }
    QVector3D visual_position;
    QQuaternion visual_orientation;
    QVector3D collision_position;
    QQuaternion collision_orientation;
    if (updater.getLinkTransforms(name, &visual_position, &visual_orientation,
                                  &collision_position,
                                  &collision_orientation)) {
      link->setTransforms(visual_position, visual_orientation,
                          collision_position, collision_orientation);
    } else {
      link->clearTransform();
    }
  }
}

void Robot::cacheGeometryMesh(
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
    // Exact URDF radius (no inflate): 1.02× pushed the ball under z=0 and
    // Z-fought the ground grid (white flicker). Denser tessellation ≈ RViz.
    (*cache)[link_name] = buildSphereMesh(geometry.size.x(), 32, 24);
  }
}


void Robot::rebuildMeshCache() {
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


QColor Robot::linkColor(const UrdfGeometry& geometry,
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


QImage Robot::loadMaterialTexture(const UrdfMaterial& material) const {
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


void Robot::drawLinkGeometry(
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
          QImage texture = part.texture;
          if (texture.isNull() && !part.texture_path.empty()) {
            const auto cached = texture_cache_.find(part.texture_path);
            if (cached != texture_cache_.end()) {
              texture = cached->second;
            } else {
              texture.load(QString::fromStdString(part.texture_path));
              texture_cache_[part.texture_path] = texture;
            }
          }
          if (!texture.isNull()) {
            // Mesh albedo already carries the look (ANYmal DAE); keep tint white.
            QColor tint(255, 255, 255, color.alpha());
            scene.addTriangleMeshTexturedPbr(part_mesh, mesh_transform, texture,
                                             tint, metallic, roughness);
          } else {
            QColor solid = part.diffuse;
            solid.setAlphaF(color.alphaF() * solid.alphaF());
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


void Robot::draw(rendering::SceneOverlay& scene, common::DisplayContext* context,
                 const common::DisplayPropertyMap& properties,
                 const std::string& display_name) {
  if (!visible_ || context == nullptr || model_.empty()) {
    return;
  }
  auto Prop = [&properties](const char* key, const char* fallback) -> std::string {
    const auto it = properties.find(key);
    return (it == properties.end() || it->second.empty()) ? fallback : it->second;
  };

  const QColor base_color =
      common::ParseColorProperty(Prop("color", "180;180;180"));
  const float alpha = (alpha_ > 0.f && alpha_ < 1.f)
      ? alpha_
      : common::ParseFloatProperty(Prop("alpha", "1.0"), 1.0f);
  QColor color = base_color;
  color.setAlphaF(alpha);
  const bool show_axes =
      common::ParseBoolProperty(Prop("show_axes", "true"), true);
  const bool solid_visual =
      Prop("visual_style", "solid") != "wireframe";
  const bool use_urdf_materials = common::ParseBoolProperty(
      Prop("use_urdf_materials", "true"), true);
  const bool show_collision = collision_visible_ || common::ParseBoolProperty(
      Prop("show_collision", "false"), false) ||
      common::ParseBoolProperty(Prop("collision_enabled", "false"), false);
  if (!visual_visible_) {
    // Still allow collision-only draws.
  }
  const float collision_alpha = common::ParseFloatProperty(
      Prop("collision_alpha", "0.35"), 0.35f);
  QColor collision_color(255, 140, 40);
  collision_color.setAlphaF(collision_alpha);

  const bool use_ogre = context->ogre_scene_host != nullptr;

  std::vector<ColoredMeshInstance> ogre_visual_meshes;
  std::vector<ColoredMeshInstance> ogre_visual_prims;
  std::vector<PbrMeshInstance> ogre_pbr_visual;
  std::vector<PbrTexturedMeshInstance> ogre_pbr_textured;
  std::vector<rendering::OgreTexturedLinkInstance> textured_links;
  auto cacheImageAt = [this](const std::string& path) -> const QImage* {
    if (path.empty()) {
      return nullptr;
    }
    const auto cached = texture_cache_.find(path);
    if (cached != texture_cache_.end()) {
      return &cached->second;
    }
    QImage image(QString::fromStdString(path));
    if (image.isNull()) {
      return nullptr;
    }
    return &texture_cache_.emplace(path, std::move(image)).first->second;
  };
  auto textureFor = [this, &cacheImageAt](const UrdfMaterial& material) -> const QImage* {
    if (!material.has_texture || material.texture_filename.empty()) {
      return nullptr;
    }
    const std::string resolved = UrdfModel::resolveTexturePath(
        model_.baseDirectory(), material.texture_filename);
    return cacheImageAt(resolved);
  };
  // Pointers into the cache are held for this frame. Reserve first so an
  // insert cannot rehash and invalidate them.
  texture_cache_.reserve(64);
  std::vector<ColoredMeshInstance> ogre_collision;
  std::vector<LineSegment3D> ogre_axes;

  const auto link_transforms = model_.computeLinkTransforms(joint_positions_);

  QMatrix4x4 root_world;
  root_world.setToIdentity();
  // Joint states plus one floating-base TF. An empty Root Link property must
  // still follow the URDF root: looking up only "base" and leaving every other
  // link at the origin draws the robot in pieces.
  std::string kinematic_root = Prop("root_link", "");
  if (kinematic_root.empty()) {
    kinematic_root = model_.rootLink();
  }
  const bool have_buffer = context->tf_buffer != nullptr;
  const auto zero_time = autoviz::commsgs::ZeroTime();
  if (!kinematic_root.empty() && have_buffer) {
    try {
      root_world = transformToMatrix(context->tf_buffer->lookupTransform(
          context->fixed_frame, kinematic_root, zero_time));
    } catch (...) {
    }
  }

  for (const auto& link : model_.links()) {
    RobotLink* robot_link = getLink(link.name);
    if (robot_link != nullptr && !robot_link->getEnabled()) {
      continue;
    }

    QMatrix4x4 link_transform;
    if (robot_link != nullptr && robot_link->hasTransform()) {
      // Per-link TF from LinkUpdater (RViz path).
      link_transform = robot_link->visualWorldMatrix();
    } else {
      const auto tf_it = link_transforms.find(link.name);
      if (tf_it == link_transforms.end()) {
        continue;
      }
      // JointState FK relative to root, then root → fixed_frame.
      link_transform = root_world * tf_it->second;
    }

    std::vector<UrdfGeometry> fallback_visuals;
    const std::vector<UrdfGeometry>* visuals = &link.visuals;
    if (visuals->empty() && link.has_visual) {
      fallback_visuals.push_back(link.visual);
      visuals = &fallback_visuals;
    }
    for (std::size_t visual_index = 0;
         visual_visible_ && visual_index < visuals->size();
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
      const QImage* texture =
          material_requested ? textureFor(geometry.material) : nullptr;
      // Textured meshes stay on the GPU. Rebuilding them every frame reuploads
      // every 2048² albedo and every vertex, which stalls the viewport.
      const bool use_pbr = texture != nullptr && !texture->isNull();
      const float metallic = geometry.material.metallic;
      const float roughness = geometry.material.roughness;
      bool ogre_visual_ok = false;
      if (use_ogre && use_pbr && mesh != nullptr &&
          (geometry.type == UrdfGeometry::Type::kMesh ||
           geometry.type == UrdfGeometry::Type::kCylinder ||
           geometry.type == UrdfGeometry::Type::kSphere)) {
        QMatrix4x4 geom_transform = link_transform;
        geom_transform.translate(geometry.origin);
        geom_transform.rotate(geometry.rotation);
        if (geometry.type == UrdfGeometry::Type::kMesh) {
          geom_transform.scale(geometry.mesh_scale);
        }
        textured_links.push_back(
            {mesh, texture, geom_transform, visual_color, metallic, roughness});
        ogre_visual_ok = true;
      } else if (use_ogre && solid_visual && mesh != nullptr &&
                 mesh->has_material_groups &&
                 geometry.type == UrdfGeometry::Type::kMesh &&
                 mesh->parts.size() == mesh->submeshes.size()) {
        // COLLADA / OBJ materials (ANYmal embedded DAE albedos).
        QMatrix4x4 geom_transform = link_transform;
        geom_transform.translate(geometry.origin);
        geom_transform.rotate(geometry.rotation);
        geom_transform.scale(geometry.mesh_scale);
        const QColor mesh_tint(255, 255, 255, visual_color.alpha());
        for (std::size_t part_i = 0; part_i < mesh->submeshes.size(); ++part_i) {
          const ObjSubmesh& part = mesh->submeshes[part_i];
          const ObjMesh* part_mesh = &mesh->parts[part_i];
          if (part_mesh->triangles.empty()) {
            continue;
          }
          const QImage* part_tex = nullptr;
          if (!part.texture.isNull()) {
            part_tex = &part.texture;
          } else if (!part.texture_path.empty()) {
            part_tex = cacheImageAt(part.texture_path);
          }
          if (part_tex != nullptr && !part_tex->isNull()) {
            textured_links.push_back(
                {part_mesh, part_tex, geom_transform, mesh_tint, metallic,
                 roughness});
            ogre_visual_ok = true;
          } else {
            QColor solid = part.diffuse;
            solid.setAlphaF(visual_color.alphaF() * solid.alphaF());
            ogre_visual_meshes.push_back(
                {part_mesh, geom_transform, solid, !solid_visual});
            ogre_visual_ok = true;
          }
        }
      } else if (use_ogre && use_pbr) {
        ogre_visual_ok =
            AppendUrdfGeometryPbr(&ogre_pbr_visual, &ogre_pbr_textured,
                                    geometry, mesh, link_transform,
                                    visual_color, *texture, metallic, roughness);
      } else if (use_ogre) {
        std::vector<ColoredMeshInstance>* bucket =
            (geometry.type == UrdfGeometry::Type::kMesh) ? &ogre_visual_meshes
                                                        : &ogre_visual_prims;
        ogre_visual_ok =
            AppendUrdfGeometryMesh(bucket, geometry, mesh, link_transform,
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
      // Sphere axes pierce the ball surface and read as RGB sparkle/flicker at
      // the contact point (ballbot). Skip axes for pure sphere visuals.
      if (geometry.type == UrdfGeometry::Type::kSphere && visuals->size() == 1) {
        continue;
      }
      QMatrix4x4 visual = link_transform;
      visual.translate(geometry.origin);
      visual.rotate(geometry.rotation);
      const QVector3D half = VisualHalfExtents(geometry);
      const QVector3D origin = visual.map(QVector3D(0.f, 0.f, 0.f));
      // Scale with the link visual; floor used to be 8 cm and dwarfed small robots.
      const float axis_len = std::clamp(half.length() * 0.55f, 0.02f, 0.12f);
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
    // File meshes first, then primitives so the URDF blue ball wins depth
    // against coplanar base.obj socket geometry (RViz-like visibility).
    drawEntityMeshesOgreOrGl(context, scene, display_name + "/visual/mesh",
                             ogre_visual_meshes);
    drawEntityMeshesOgreOrGl(context, scene, display_name + "/visual/prim",
                             ogre_visual_prims);
    drawPbrMeshesOgreOrGl(context, scene, display_name + "/visual/pbr", ogre_pbr_visual);
    drawPbrTexturedMeshesOgreOrGl(context, scene, display_name + "/visual/pbr_tex",
                                  ogre_pbr_textured);
    if (context->ogre_scene_host != nullptr) {
      context->ogre_scene_host->setDisplayTexturedLinks(display_name + "/visual/links",
                                                         textured_links);
    }
    if (show_collision) {
      drawEntityMeshesOgreOrGl(context, scene, display_name + "/collision", ogre_collision);
    }
    if (show_axes) {
      drawLineSegmentsOgreOrGl(context, scene, display_name + "/axes", ogre_axes);
    }
  }
}


}  // namespace robot
}  // namespace display
}  // namespace autoviz
