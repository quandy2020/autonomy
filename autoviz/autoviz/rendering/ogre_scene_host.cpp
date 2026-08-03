/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#include "autoviz/rendering/ogre_scene_host.hpp"

#ifdef AUTOVIZ_USE_OGRE

#include <algorithm>
#include <unordered_map>

#include <QQuaternion>
#include <QVector2D>

#include <Ogre.h>

#include "autoviz/display/arrow_mesh_utils.hpp"
#include "autoviz/display/obj_mesh.hpp"
#include "autoviz/rendering/apply_visibility_bits.hpp"
#include "autoviz/rendering/ogre_custom_parameter_indices.hpp"
#include "autoviz/rendering/ogre_material_manager.hpp"
#include "autoviz/rendering/ogre_mesh_loader.hpp"
#include "autoviz/rendering/objects/ogre_billboard_line.hpp"
#include "autoviz/rendering/objects/ogre_arrow.hpp"
#include "autoviz/rendering/objects/ogre_line.hpp"
#include "autoviz/rendering/objects/ogre_covariance_visual.hpp"
#include "autoviz/rendering/objects/ogre_movable_text.hpp"
#include "autoviz/rendering/objects/ogre_point_cloud.hpp"
#include "autoviz/rendering/objects/ogre_screw_visual.hpp"
#include "autoviz/rendering/objects/ogre_wrench_visual.hpp"
#include "autoviz/rendering/point_cloud_style_utils.hpp"

namespace autoviz {
namespace rendering {
namespace {

Ogre::ColourValue ToOgrePickColor(common::PickHandle handle) {
  const common::PickColor color = common::handleToPickColor(handle);
  return Ogre::ColourValue(color.r / 255.f, color.g / 255.f, color.b / 255.f,
                             1.f);
}

Ogre::ColourValue ToOgreColor(const QColor& color) {
  return Ogre::ColourValue(color.redF(), color.greenF(), color.blueF(),
                           color.alphaF());
}

Ogre::Quaternion ToOgreQuaternion(const QQuaternion& q) {
  return Ogre::Quaternion(q.scalar(), q.x(), q.y(), q.z());
}

void ApplyPickToSection(Ogre::ManualObject* object, common::PickHandle handle) {
  if (object == nullptr || handle == common::kInvalidPickHandle ||
      object->getNumSections() == 0) {
    return;
  }
  Ogre::Renderable* section =
      object->getSection(object->getNumSections() - 1);
  if (section == nullptr) {
    return;
  }
  const Ogre::ColourValue pick = ToOgrePickColor(handle);
  section->setCustomParameter(
      AUTOVIZ_OGRE_PICK_COLOR_PARAMETER,
      Ogre::Vector4(pick.r, pick.g, pick.b, pick.a));
  section->getUserObjectBindings().setUserAny(
      "pick_handle", Ogre::Any(static_cast<Ogre::uint32>(1)));
}

void ApplyPickToEntity(Ogre::Entity* entity, common::PickHandle handle) {
  if (entity == nullptr || handle == common::kInvalidPickHandle) {
    return;
  }
  const Ogre::ColourValue pick = ToOgrePickColor(handle);
  const Ogre::Vector4 pick_vec(pick.r, pick.g, pick.b, pick.a);
  for (unsigned int i = 0; i < entity->getNumSubEntities(); ++i) {
    Ogre::SubEntity* sub = entity->getSubEntity(i);
    if (sub == nullptr) {
      continue;
    }
    sub->setCustomParameter(AUTOVIZ_OGRE_PICK_COLOR_PARAMETER, pick_vec);
    sub->getUserObjectBindings().setUserAny(
        "pick_handle", Ogre::Any(static_cast<Ogre::uint32>(1)));
  }
}

void ApplyMatrixToNode(Ogre::SceneNode* node, const QMatrix4x4& matrix) {
  if (node == nullptr) {
    return;
  }
  const Ogre::Vector3 position(matrix(0, 3), matrix(1, 3), matrix(2, 3));
  const QVector3D col0(matrix(0, 0), matrix(1, 0), matrix(2, 0));
  const QVector3D col1(matrix(0, 1), matrix(1, 1), matrix(2, 1));
  const QVector3D col2(matrix(0, 2), matrix(1, 2), matrix(2, 2));
  const float sx = col0.length();
  const float sy = col1.length();
  const float sz = col2.length();
  const Ogre::Vector3 scale(
      sx > 1e-6f ? sx : 1.f, sy > 1e-6f ? sy : 1.f, sz > 1e-6f ? sz : 1.f);

  float rot[3][3] = {
      {sx > 1e-6f ? matrix(0, 0) / sx : 1.f,
       sy > 1e-6f ? matrix(0, 1) / sy : 0.f,
       sz > 1e-6f ? matrix(0, 2) / sz : 0.f},
      {sx > 1e-6f ? matrix(1, 0) / sx : 0.f,
       sy > 1e-6f ? matrix(1, 1) / sy : 1.f,
       sz > 1e-6f ? matrix(1, 2) / sz : 0.f},
      {sx > 1e-6f ? matrix(2, 0) / sx : 0.f,
       sy > 1e-6f ? matrix(2, 1) / sy : 0.f,
       sz > 1e-6f ? matrix(2, 2) / sz : 1.f},
  };
  Ogre::Matrix3 rot_mat;
  for (int row = 0; row < 3; ++row) {
    for (int col = 0; col < 3; ++col) {
      rot_mat[row][col] = rot[row][col];
    }
  }
  const Ogre::Quaternion orientation(rot_mat);
  node->setPosition(position);
  node->setOrientation(orientation);
  node->setScale(scale);
}

}  // namespace

OgreSceneHost::OgreSceneHost(Ogre::SceneManager* scene_manager)
    : scene_manager_(scene_manager) {
  OgreMaterialManager::ensureDefaultMaterials();
}

OgreSceneHost::~OgreSceneHost() { clear(); }

Ogre::SceneNode* OgreSceneHost::displayNode(const std::string& display_name) {
  if (scene_manager_ == nullptr) {
    return nullptr;
  }
  DisplayEntry& entry = displays_[display_name];
  if (entry.node == nullptr) {
    entry.node = scene_manager_->getRootSceneNode()->createChildSceneNode(
        "AvizDisplay/" + display_name);
  }
  return entry.node;
}

void OgreSceneHost::applyEntryVisibility(DisplayEntry& entry) {
  if (entry.node != nullptr) {
    applyVisibilityBits(entry.visibility_bits, entry.node);
  }
}

void OgreSceneHost::setDisplayVisibilityBits(const std::string& display_name,
                                               uint32_t bits) {
  const auto it = displays_.find(display_name);
  if (it == displays_.end()) {
    return;
  }
  it->second.visibility_bits = bits;
  applyEntryVisibility(it->second);
}

void OgreSceneHost::destroyLines(DisplayEntry& entry) {
  if (scene_manager_ == nullptr || entry.lines == nullptr) {
    return;
  }
  if (entry.node != nullptr) {
    entry.node->detachObject(entry.lines);
  }
  scene_manager_->destroyManualObject(entry.lines);
  entry.lines = nullptr;
}

void OgreSceneHost::destroyMeshes(DisplayEntry& entry) {
  if (scene_manager_ == nullptr || entry.meshes == nullptr) {
    return;
  }
  if (entry.node != nullptr) {
    entry.node->detachObject(entry.meshes);
  }
  scene_manager_->destroyManualObject(entry.meshes);
  entry.meshes = nullptr;
}

void OgreSceneHost::destroyEntities(DisplayEntry& entry) {
  if (scene_manager_ == nullptr) {
    entry.entities.clear();
    return;
  }
  for (DisplayEntry::EntitySlot& slot : entry.entities) {
    if (slot.entity != nullptr) {
      if (slot.node != nullptr) {
        slot.node->detachObject(slot.entity);
      }
      scene_manager_->destroyEntity(slot.entity);
      slot.entity = nullptr;
    }
    if (!slot.material_name.empty() &&
        Ogre::MaterialManager::getSingleton().resourceExists(slot.material_name)) {
      Ogre::MaterialManager::getSingleton().remove(slot.material_name);
    }
    slot.material_name.clear();
    if (slot.node != nullptr) {
      scene_manager_->destroySceneNode(slot.node);
      slot.node = nullptr;
    }
  }
  entry.entities.clear();
}

void OgreSceneHost::destroyPbr(DisplayEntry& entry) {
  if (scene_manager_ == nullptr) {
    entry.pbr_texture_names.clear();
    entry.pbr_material_names.clear();
    entry.pbr_textured_objects.clear();
    return;
  }
  if (entry.pbr_meshes != nullptr) {
    if (entry.node != nullptr) {
      entry.node->detachObject(entry.pbr_meshes);
    }
    scene_manager_->destroyManualObject(entry.pbr_meshes);
    entry.pbr_meshes = nullptr;
  }
  for (Ogre::ManualObject* object : entry.pbr_textured_objects) {
    if (object != nullptr) {
      if (entry.node != nullptr) {
        entry.node->detachObject(object);
      }
      scene_manager_->destroyManualObject(object);
    }
  }
  entry.pbr_textured_objects.clear();
  for (const std::string& name : entry.pbr_texture_names) {
    if (Ogre::TextureManager::getSingleton().resourceExists(name)) {
      Ogre::TextureManager::getSingleton().remove(name);
    }
  }
  entry.pbr_texture_names.clear();
  for (const std::string& name : entry.pbr_material_names) {
    if (Ogre::MaterialManager::getSingleton().resourceExists(name)) {
      Ogre::MaterialManager::getSingleton().remove(name);
    }
  }
  entry.pbr_material_names.clear();
}

void OgreSceneHost::destroyLabels(DisplayEntry& entry) {
  if (scene_manager_ == nullptr) {
    entry.labels.clear();
    return;
  }
  for (DisplayEntry::LabelSlot& slot : entry.labels) {
    if (slot.text != nullptr && slot.node != nullptr) {
      slot.node->detachObject(slot.text.get());
    }
    slot.text.reset();
    if (slot.node != nullptr) {
      scene_manager_->destroySceneNode(slot.node);
      slot.node = nullptr;
    }
  }
  entry.labels.clear();
}

void OgreSceneHost::destroyToolLine(DisplayEntry& entry) {
  entry.tool_line.reset();
}

void OgreSceneHost::destroyToolPoseArrow(DisplayEntry& entry) {
  entry.tool_pose_arrow.reset();
}

void OgreSceneHost::destroyBillboardLine(DisplayEntry& entry) {
  entry.billboard_line.reset();
}

void OgreSceneHost::destroyWrench(DisplayEntry& entry) {
  entry.wrench.reset();
}

void OgreSceneHost::destroyScrew(DisplayEntry& entry) {
  entry.screw.reset();
}

void OgreSceneHost::destroyCovariance(DisplayEntry& entry) {
  entry.covariance.reset();
}

void OgreSceneHost::uploadLines(
    Ogre::ManualObject* object,
    const std::vector<OgreColoredLineSegment>& segments) {
  if (object == nullptr || segments.empty()) {
    return;
  }
  object->clear();
  object->begin("Autoviz/LineNoLighting", Ogre::RenderOperation::OT_LINE_LIST,
                "AvizOgre");
  for (const OgreColoredLineSegment& segment : segments) {
    object->position(segment.a.x(), segment.a.y(), segment.a.z());
    const Ogre::ColourValue color = ToOgreColor(segment.color);
    object->colour(color.r, color.g, color.b, color.a);
    object->position(segment.b.x(), segment.b.y(), segment.b.z());
    object->colour(color.r, color.g, color.b, color.a);
  }
  object->end();
}

void OgreSceneHost::uploadMeshes(
    Ogre::ManualObject* object,
    const std::vector<OgreColoredMeshInstance>& meshes) {
  if (object == nullptr) {
    return;
  }
  object->clear();

  for (const OgreColoredMeshInstance& instance : meshes) {
    if (instance.wireframe) {
      object->begin("Autoviz/LineNoLighting", Ogre::RenderOperation::OT_LINE_LIST,
                "AvizOgre");
      const Ogre::ColourValue color = ToOgreColor(instance.color);
      for (const auto& triangle : instance.mesh.triangles) {
        if (triangle[0] < 0 ||
            triangle[0] >= static_cast<int>(instance.mesh.vertices.size()) ||
            triangle[1] < 0 ||
            triangle[1] >= static_cast<int>(instance.mesh.vertices.size()) ||
            triangle[2] < 0 ||
            triangle[2] >= static_cast<int>(instance.mesh.vertices.size())) {
          continue;
        }
        const QVector3D a = instance.transform.map(
            instance.mesh.vertices[static_cast<std::size_t>(triangle[0])]);
        const QVector3D b = instance.transform.map(
            instance.mesh.vertices[static_cast<std::size_t>(triangle[1])]);
        const QVector3D c = instance.transform.map(
            instance.mesh.vertices[static_cast<std::size_t>(triangle[2])]);
        const auto add_edge = [&](const QVector3D& p1, const QVector3D& p2) {
          object->position(p1.x(), p1.y(), p1.z());
          object->colour(color.r, color.g, color.b, color.a);
          object->position(p2.x(), p2.y(), p2.z());
          object->colour(color.r, color.g, color.b, color.a);
        };
        add_edge(a, b);
        add_edge(b, c);
        add_edge(c, a);
      }
      object->end();
      ApplyPickToSection(object, instance.pick_handle);
      continue;
    }

    object->begin("Autoviz/FlatNoLighting", Ogre::RenderOperation::OT_TRIANGLE_LIST);
    const Ogre::ColourValue color = ToOgreColor(instance.color);
    for (const auto& triangle : instance.mesh.triangles) {
      if (triangle[0] < 0 ||
          triangle[0] >= static_cast<int>(instance.mesh.vertices.size()) ||
          triangle[1] < 0 ||
          triangle[1] >= static_cast<int>(instance.mesh.vertices.size()) ||
          triangle[2] < 0 ||
          triangle[2] >= static_cast<int>(instance.mesh.vertices.size())) {
        continue;
      }
      for (int index : triangle) {
        const QVector3D vertex =
            instance.transform.map(instance.mesh.vertices[static_cast<std::size_t>(index)]);
        object->position(vertex.x(), vertex.y(), vertex.z());
        object->colour(color.r, color.g, color.b, color.a);
      }
    }
    object->end();
    ApplyPickToSection(object, instance.pick_handle);
  }
}

void OgreSceneHost::uploadPbrMeshes(
    Ogre::ManualObject* object,
    const std::vector<OgrePbrMeshInstance>& meshes) {
  if (object == nullptr || meshes.empty()) {
    return;
  }
  object->clear();
  object->begin("AvizPBR", Ogre::RenderOperation::OT_TRIANGLE_LIST);
  for (const OgrePbrMeshInstance& instance : meshes) {
    const Ogre::ColourValue albedo = ToOgreColor(instance.color);
    for (const auto& triangle : instance.mesh.triangles) {
      if (triangle[0] < 0 ||
          triangle[0] >= static_cast<int>(instance.mesh.vertices.size()) ||
          triangle[1] < 0 ||
          triangle[1] >= static_cast<int>(instance.mesh.vertices.size()) ||
          triangle[2] < 0 ||
          triangle[2] >= static_cast<int>(instance.mesh.vertices.size())) {
        continue;
      }
      const QVector3D corners[3] = {
          instance.transform.map(
              instance.mesh.vertices[static_cast<std::size_t>(triangle[0])]),
          instance.transform.map(
              instance.mesh.vertices[static_cast<std::size_t>(triangle[1])]),
          instance.transform.map(
              instance.mesh.vertices[static_cast<std::size_t>(triangle[2])])};
      QVector3D normal =
          QVector3D::crossProduct(corners[1] - corners[0], corners[2] - corners[0]);
      if (normal.lengthSquared() < 1e-10f) {
        normal = QVector3D(0.f, 1.f, 0.f);
      } else {
        normal.normalize();
      }
      for (const QVector3D& vertex : corners) {
        object->position(vertex.x(), vertex.y(), vertex.z());
        object->normal(normal.x(), normal.y(), normal.z());
        object->colour(albedo.r, albedo.g, albedo.b, albedo.a);
        object->textureCoord(instance.metallic, instance.roughness);
      }
    }
  }
  object->end();
}

void OgreSceneHost::uploadPbrTexturedMeshes(
    DisplayEntry& entry, const std::string& display_name,
    const std::vector<OgrePbrTexturedMeshInstance>& meshes) {
  if (scene_manager_ == nullptr || entry.node == nullptr || meshes.empty() ||
      !Ogre::MaterialManager::getSingleton().resourceExists("AvizPBRTextured")) {
    return;
  }

  struct BatchKey {
    qint64 cache_key = 0;
    float metallic = 0.f;
    float roughness = 0.f;
    bool operator==(const BatchKey& other) const {
      return cache_key == other.cache_key && metallic == other.metallic &&
             roughness == other.roughness;
    }
  };
  struct BatchKeyHash {
    std::size_t operator()(const BatchKey& key) const {
      return static_cast<std::size_t>(key.cache_key) ^
             (static_cast<std::size_t>(key.metallic * 1000.f) << 1) ^
             (static_cast<std::size_t>(key.roughness * 1000.f) << 2);
    }
  };

  std::unordered_map<BatchKey, std::vector<const OgrePbrTexturedMeshInstance*>,
                     BatchKeyHash>
      batches;
  for (const OgrePbrTexturedMeshInstance& mesh : meshes) {
    if (mesh.texture.isNull()) {
      continue;
    }
    const QImage rgba = mesh.texture.format() == QImage::Format_RGBA8888
                            ? mesh.texture
                            : mesh.texture.convertToFormat(QImage::Format_RGBA8888);
    batches[{rgba.cacheKey(), mesh.metallic, mesh.roughness}].push_back(&mesh);
  }

  int index = 0;
  for (const auto& [key, group] : batches) {
    if (group.empty()) {
      continue;
    }
    const QImage& rgba =
        group.front()->texture.format() == QImage::Format_RGBA8888
            ? group.front()->texture
            : group.front()->texture.convertToFormat(QImage::Format_RGBA8888);
    const Ogre::String base_name =
        display_name + "/PbrTex" + Ogre::StringConverter::toString(index++);
    const Ogre::String tex_name = base_name + "Tex";
    Ogre::TexturePtr texture =
        Ogre::TextureManager::getSingleton().createManual(
            tex_name, Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME,
            Ogre::TEX_TYPE_2D, rgba.width(), rgba.height(), 0,
            Ogre::PF_R8G8B8A8, Ogre::TU_DYNAMIC);
    Ogre::PixelBox pixel_box(
        static_cast<Ogre::uint32>(rgba.width()),
        static_cast<Ogre::uint32>(rgba.height()), 1, Ogre::PF_R8G8B8A8,
        const_cast<uchar*>(rgba.constBits()));
    texture->getBuffer()->blitFromMemory(pixel_box);

    const Ogre::String mat_name = base_name + "Mat";
    Ogre::MaterialPtr material =
        Ogre::MaterialManager::getSingleton()
            .getByName("AvizPBRTextured")
            ->clone(mat_name);
    material->getTechnique(0)->getPass(0)->createTextureUnitState(tex_name);

    Ogre::ManualObject* object = scene_manager_->createManualObject(base_name + "MO");
    object->begin(mat_name, Ogre::RenderOperation::OT_TRIANGLE_LIST);
    for (const OgrePbrTexturedMeshInstance* instance : group) {
      display::ObjMesh mesh_with_uv = instance->mesh;
      if (mesh_with_uv.texcoords.size() != mesh_with_uv.vertices.size()) {
        display::ensureMeshTexcoords(&mesh_with_uv);
      }
      const Ogre::ColourValue tint = ToOgreColor(instance->tint);
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
            instance->transform.map(
                mesh_with_uv.vertices[static_cast<std::size_t>(triangle[0])]),
            instance->transform.map(
                mesh_with_uv.vertices[static_cast<std::size_t>(triangle[1])]),
            instance->transform.map(
                mesh_with_uv.vertices[static_cast<std::size_t>(triangle[2])])};
        QVector3D normal = QVector3D::crossProduct(corners[1] - corners[0],
                                                   corners[2] - corners[0]);
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
          object->position(corners[i].x(), corners[i].y(), corners[i].z());
          object->normal(normal.x(), normal.y(), normal.z());
          object->textureCoord(0, uv.x(), uv.y());
          object->colour(tint.r, tint.g, tint.b, tint.a);
          object->textureCoord(1, instance->metallic, instance->roughness);
        }
      }
    }
    object->end();
    entry.node->attachObject(object);
    entry.pbr_textured_objects.push_back(object);
    entry.pbr_texture_names.push_back(tex_name);
    entry.pbr_material_names.push_back(mat_name);
  }
}

void OgreSceneHost::setDisplayPoints(const std::string& display_name,
                                     float point_size, PointCloudStyle style,
                                     const std::vector<QVector3D>& positions,
                                     const std::vector<QColor>& colors) {
  if (scene_manager_ == nullptr || positions.empty()) {
    return;
  }
  displayNode(display_name);
  DisplayEntry& entry = displays_[display_name];
  if (entry.cloud == nullptr) {
    entry.cloud = std::make_unique<OgrePointCloud>();
    entry.cloud->setName(display_name);
    entry.node->attachObject(entry.cloud.get());
  }

  const float world_size = std::max(0.002f, point_size * 0.006f);
  entry.cloud->clearAndRemoveAllPoints();
  entry.cloud->setRenderMode(toOgrePointCloudRenderMode(style));
  entry.cloud->setDimensions(world_size, world_size, world_size);

  std::vector<OgrePointCloud::Point> ogre_points;
  ogre_points.reserve(positions.size());
  for (std::size_t i = 0; i < positions.size(); ++i) {
    OgrePointCloud::Point p;
    *p.mutable_position() = Ogre::Vector3(positions[i].x(), positions[i].y(), positions[i].z());
    const QColor& c = i < colors.size() ? colors[i] : QColor(200, 200, 200);
    p.setColor(c.redF(), c.greenF(), c.blueF(), c.alphaF());
    ogre_points.push_back(p);
  }
  entry.cloud->addPoints(ogre_points.begin(), ogre_points.end());
  applyEntryVisibility(entry);
}

void OgreSceneHost::setDisplayBillboardStrip(const std::string& display_name,
                                             const std::vector<QVector3D>& points,
                                             const QColor& color, float line_width) {
  if (scene_manager_ == nullptr) {
    return;
  }
  displayNode(display_name);
  DisplayEntry& entry = displays_[display_name];
  destroyLines(entry);
  if (points.size() < 2 || line_width <= 0.f) {
    destroyBillboardLine(entry);
    return;
  }
  if (entry.billboard_line == nullptr) {
    entry.billboard_line =
        std::make_unique<OgreBillboardLine>(scene_manager_, entry.node);
  }
  std::vector<Ogre::Vector3> ogre_points;
  ogre_points.reserve(points.size());
  for (const QVector3D& point : points) {
    ogre_points.emplace_back(point.x(), point.y(), point.z());
  }
  const Ogre::ColourValue ogre_color(color.redF(), color.greenF(), color.blueF(),
                                     color.alphaF());
  entry.billboard_line->setPolyline(ogre_points, ogre_color, line_width);
  applyEntryVisibility(entry);
}

void OgreSceneHost::setCloudPickHandle(const std::string& display_name,
                                       common::PickHandle handle) {
  if (handle == common::kInvalidPickHandle) {
    return;
  }
  cloud_pick_handles_[display_name] = handle;
  const auto it = displays_.find(display_name);
  if (it != displays_.end() && it->second.cloud != nullptr) {
    it->second.cloud->setPickColor(ToOgrePickColor(handle));
  }
}

bool OgreSceneHost::isCloudPickHandle(common::PickHandle handle) const {
  for (const auto& pair : cloud_pick_handles_) {
    if (pair.second == handle) {
      return true;
    }
  }
  return false;
}

const std::string* OgreSceneHost::displayForCloudPickHandle(
    common::PickHandle handle) const {
  for (const auto& pair : cloud_pick_handles_) {
    if (pair.second == handle) {
      return &pair.first;
    }
  }
  return nullptr;
}

void OgreSceneHost::setColorByIndexForAll(bool enabled) {
  for (auto& pair : displays_) {
    if (pair.second.cloud != nullptr) {
      pair.second.cloud->setColorByIndex(enabled);
    }
  }
}

void OgreSceneHost::setDisplayLines(
    const std::string& display_name,
    const std::vector<OgreColoredLineSegment>& segments) {
  if (scene_manager_ == nullptr) {
    return;
  }
  displayNode(display_name);
  DisplayEntry& entry = displays_[display_name];
  destroyToolLine(entry);
  destroyToolPoseArrow(entry);
  destroyBillboardLine(entry);
  if (segments.empty()) {
    destroyLines(entry);
    return;
  }
  if (entry.lines == nullptr) {
    entry.lines = scene_manager_->createManualObject(display_name + "/Lines");
    entry.node->attachObject(entry.lines);
  }
  uploadLines(entry.lines, segments);
  entry.node->needUpdate();
  applyEntryVisibility(entry);
}

void OgreSceneHost::setDisplayArrow(const std::string& display_name,
                                    const QVector3D& start, const QVector3D& end,
                                    const QColor& color, float head_fraction) {
  std::vector<display::ColoredMeshInstance> meshes;
  display::appendSolidArrowMeshes(&meshes, start, end, color, head_fraction);
  std::vector<OgreColoredMeshInstance> ogre_meshes;
  ogre_meshes.reserve(meshes.size());
  for (const auto& mesh : meshes) {
    ogre_meshes.push_back(
        {mesh.mesh, mesh.transform(), mesh.color, mesh.wireframe});
  }
  setDisplayMeshes(display_name, ogre_meshes);
}

void OgreSceneHost::setDisplayMeshes(
    const std::string& display_name,
    const std::vector<OgreColoredMeshInstance>& meshes) {
  if (scene_manager_ == nullptr) {
    return;
  }
  displayNode(display_name);
  DisplayEntry& entry = displays_[display_name];
  destroyToolPoseArrow(entry);
  destroyEntities(entry);
  if (meshes.empty()) {
    destroyMeshes(entry);
    return;
  }
  if (entry.meshes == nullptr) {
    entry.meshes = scene_manager_->createManualObject(display_name + "/Meshes");
    entry.node->attachObject(entry.meshes);
  }
  uploadMeshes(entry.meshes, meshes);
  entry.node->needUpdate();
  applyEntryVisibility(entry);
}

void OgreSceneHost::setDisplayEntities(
    const std::string& display_name,
    const std::vector<OgreEntityInstance>& entities) {
  if (scene_manager_ == nullptr) {
    return;
  }
  OgreMeshLoader::ensurePrimitiveMeshes();
  displayNode(display_name);
  DisplayEntry& entry = displays_[display_name];
  destroyMeshes(entry);
  destroyEntities(entry);
  if (entities.empty()) {
    return;
  }

  int index = 0;
  for (const OgreEntityInstance& instance : entities) {
    if (instance.mesh_name.empty()) {
      continue;
    }
    const std::string entity_name =
        display_name + "/Entity/" + std::to_string(index++);
    DisplayEntry::EntitySlot slot;
    slot.node = entry.node->createChildSceneNode(entity_name + "Node");
    ApplyMatrixToNode(slot.node, instance.transform());
    slot.entity = scene_manager_->createEntity(
        entity_name, instance.mesh_name, "rviz_rendering");
    slot.material_name = entity_name + "Mat";
    Ogre::MaterialPtr material =
        OgreMaterialManager::createMaterialWithLighting(slot.material_name);
    const Ogre::ColourValue color = ToOgreColor(instance.color);
    material->getTechnique(0)->setAmbient(color * 0.5f);
    material->getTechnique(0)->setDiffuse(color);
    OgreMaterialManager::enableAlphaBlending(material, color.a);
    slot.entity->setMaterialName(slot.material_name);
    slot.node->attachObject(slot.entity);
    ApplyPickToEntity(slot.entity, instance.pick_handle);
    entry.entities.push_back(std::move(slot));
  }
  applyEntryVisibility(entry);
}

void OgreSceneHost::setDisplayPbrMeshes(
    const std::string& display_name,
    const std::vector<OgrePbrMeshInstance>& meshes) {
  if (scene_manager_ == nullptr) {
    return;
  }
  displayNode(display_name);
  DisplayEntry& entry = displays_[display_name];
  if (meshes.empty()) {
    destroyPbr(entry);
    return;
  }
  destroyPbr(entry);
  entry.pbr_meshes =
      scene_manager_->createManualObject(display_name + "/PbrMeshes");
  uploadPbrMeshes(entry.pbr_meshes, meshes);
  entry.node->attachObject(entry.pbr_meshes);
  entry.node->needUpdate();
  applyEntryVisibility(entry);
}

void OgreSceneHost::setDisplayPbrTexturedMeshes(
    const std::string& display_name,
    const std::vector<OgrePbrTexturedMeshInstance>& meshes) {
  if (scene_manager_ == nullptr) {
    return;
  }
  displayNode(display_name);
  DisplayEntry& entry = displays_[display_name];
  destroyPbr(entry);
  if (meshes.empty()) {
    return;
  }
  uploadPbrTexturedMeshes(entry, display_name, meshes);
  entry.node->needUpdate();
  applyEntryVisibility(entry);
}

void OgreSceneHost::setDisplayLabels(const std::string& display_name,
                                     const std::vector<OgreTextLabel>& labels) {
  if (scene_manager_ == nullptr) {
    return;
  }
  displayNode(display_name);
  DisplayEntry& entry = displays_[display_name];
  destroyLabels(entry);
  if (labels.empty()) {
    return;
  }

  int index = 0;
  for (const OgreTextLabel& label : labels) {
    DisplayEntry::LabelSlot slot;
    slot.node = entry.node->createChildSceneNode(
        display_name + "/Label/" + std::to_string(index++));
    slot.node->setPosition(label.position.x(), label.position.y(),
                           label.position.z());

    const Ogre::ColourValue color(label.color.redF(), label.color.greenF(),
                                  label.color.blueF(), label.color.alphaF());
    slot.text = std::make_unique<OgreMovableText>(
        label.text, "Liberation Sans", label.char_height, color);
    slot.text->setTextAlignment(OgreMovableText::H_CENTER,
                                OgreMovableText::V_CENTER);
    if (label.space_width > 0.f) {
      slot.text->setSpaceWidth(label.space_width);
    }
    slot.node->attachObject(slot.text.get());
    entry.labels.push_back(std::move(slot));
  }
  applyEntryVisibility(entry);
}

void OgreSceneHost::setDisplayWrench(const std::string& display_name,
                                      const QVector3D& origin,
                                      const QVector3D& force,
                                      const QVector3D& torque,
                                      const QColor& force_color,
                                      const QColor& torque_color,
                                      float force_scale, float torque_scale,
                                      float width) {
  if (scene_manager_ == nullptr) {
    return;
  }
  displayNode(display_name);
  DisplayEntry& entry = displays_[display_name];
  destroyLines(entry);
  destroyMeshes(entry);
  destroyBillboardLine(entry);
  destroyScrew(entry);
  destroyCovariance(entry);
  if (entry.wrench == nullptr) {
    entry.wrench =
        std::make_unique<OgreWrenchVisual>(scene_manager_, entry.node);
  }
  entry.wrench->setFramePosition(
      Ogre::Vector3(origin.x(), origin.y(), origin.z()));
  entry.wrench->setFrameOrientation(Ogre::Quaternion::IDENTITY);
  entry.wrench->setWrench(
      Ogre::Vector3(force.x(), force.y(), force.z()),
      Ogre::Vector3(torque.x(), torque.y(), torque.z()));
  entry.wrench->setForceColor(force_color.redF(), force_color.greenF(),
                              force_color.blueF(), force_color.alphaF());
  entry.wrench->setTorqueColor(torque_color.redF(), torque_color.greenF(),
                               torque_color.blueF(), torque_color.alphaF());
  entry.wrench->setForceScale(force_scale);
  entry.wrench->setTorqueScale(torque_scale);
  entry.wrench->setWidth(width);
  entry.wrench->setVisible(true);
  applyEntryVisibility(entry);
}

void OgreSceneHost::setDisplayScrew(const std::string& display_name,
                                    const QVector3D& origin,
                                    const QVector3D& linear,
                                    const QVector3D& angular,
                                    const QColor& linear_color,
                                    const QColor& angular_color,
                                    float linear_scale, float angular_scale,
                                    float width, bool hide_small_values) {
  if (scene_manager_ == nullptr) {
    return;
  }
  displayNode(display_name);
  DisplayEntry& entry = displays_[display_name];
  destroyLines(entry);
  destroyMeshes(entry);
  destroyBillboardLine(entry);
  destroyWrench(entry);
  destroyCovariance(entry);
  if (entry.screw == nullptr) {
    entry.screw = std::make_unique<OgreScrewVisual>(scene_manager_, entry.node);
  }
  entry.screw->setFramePosition(
      Ogre::Vector3(origin.x(), origin.y(), origin.z()));
  entry.screw->setFrameOrientation(Ogre::Quaternion::IDENTITY);
  entry.screw->setScrew(
      Ogre::Vector3(linear.x(), linear.y(), linear.z()),
      Ogre::Vector3(angular.x(), angular.y(), angular.z()));
  entry.screw->setLinearColor(linear_color.redF(), linear_color.greenF(),
                               linear_color.blueF(), linear_color.alphaF());
  entry.screw->setAngularColor(angular_color.redF(), angular_color.greenF(),
                               angular_color.blueF(), angular_color.alphaF());
  entry.screw->setLinearScale(linear_scale);
  entry.screw->setAngularScale(angular_scale);
  entry.screw->setWidth(width);
  entry.screw->setHideSmallValues(hide_small_values);
  entry.screw->setVisible(true);
  applyEntryVisibility(entry);
}

void OgreSceneHost::setDisplayCovariance(
    const std::string& display_name, const QVector3D& position,
    const QQuaternion& pose_orientation, const QQuaternion& frame_orientation,
    const std::array<double, 36>& covariance, const QColor& position_color,
    float position_scale, float orientation_scale, float orientation_offset,
    bool visible) {
  if (scene_manager_ == nullptr) {
    return;
  }
  displayNode(display_name);
  DisplayEntry& entry = displays_[display_name];
  destroyLines(entry);
  destroyMeshes(entry);
  destroyBillboardLine(entry);
  destroyWrench(entry);
  destroyScrew(entry);
  if (!visible) {
    destroyCovariance(entry);
    return;
  }
  if (entry.covariance == nullptr) {
    entry.covariance = std::make_unique<OgreCovarianceVisual>(
        scene_manager_, entry.node, false, true, position_scale,
        orientation_scale, orientation_offset);
  }
  CovarianceUserData user_data;
  user_data.visible = true;
  user_data.position_visible = true;
  user_data.position_color = ToOgreColor(position_color);
  user_data.position_scale = position_scale;
  user_data.orientation_visible = true;
  user_data.orientation_frame = Frame::Local;
  user_data.orientation_color_style = ColorStyle::RGB;
  user_data.orientation_color = ToOgreColor(position_color);
  user_data.orientation_offset = orientation_offset;
  user_data.orientation_scale = orientation_scale;
  entry.covariance->updateUserData(user_data);
  entry.covariance->setPosition(
      Ogre::Vector3(position.x(), position.y(), position.z()));
  entry.covariance->setOrientation(ToOgreQuaternion(frame_orientation));
  entry.covariance->setCovariance(ToOgreQuaternion(pose_orientation),
                                  covariance);
  entry.covariance->setVisible(true);
  applyEntryVisibility(entry);
}

void OgreSceneHost::setToolBillboardStrip(const std::string& tool_id,
                                          const std::vector<QVector3D>& points,
                                          const QColor& color, float line_width) {
  const std::string name = "AvizTool/" + tool_id;
  setDisplayBillboardStrip(name, points, color, line_width);
  setDisplayVisibilityBits(name, 0xFFFFFFFFu);
}

void OgreSceneHost::setToolLineSegment(const std::string& tool_id,
                                       const QVector3D& start,
                                       const QVector3D& end,
                                       const QColor& color) {
  const std::string line_name = "AvizTool/" + tool_id + "/line";
  const std::string strip_name = "AvizTool/" + tool_id + "/strip";
  if ((start - end).lengthSquared() < 1e-10f) {
    removeDisplay(line_name);
    removeDisplay(strip_name);
    return;
  }
  setDisplayLines(line_name, {{start, end, color}});
  setDisplayVisibilityBits(line_name, 0xFFFFFFFFu);
  setDisplayBillboardStrip(strip_name, {start, end}, color, 0.02f);
  setDisplayVisibilityBits(strip_name, 0xFFFFFFFFu);
}

void OgreSceneHost::setToolArrow(const std::string& tool_id,
                                 const QVector3D& start, const QVector3D& end,
                                 const QColor& color, float head_fraction) {
  const std::string name = "AvizTool/" + tool_id + "/arrow";
  if ((start - end).lengthSquared() < 1e-8f) {
    removeDisplay(name);
    return;
  }
  setDisplayArrow(name, start, end, color, head_fraction);
  setDisplayVisibilityBits(name, 0xFFFFFFFFu);
}

void OgreSceneHost::setToolPoseArrow(const std::string& tool_id,
                                     const QVector3D& position, float yaw,
                                     const QColor& color, bool visible) {
  if (scene_manager_ == nullptr) {
    return;
  }
  const std::string name = "AvizTool/" + tool_id + "/pose_arrow";
  if (!visible) {
    removeDisplay(name);
    return;
  }
  // Same ManualObject mesh path as PoseDisplay / drawArrowOgreOrGl (not OgreArrow Entity).
  constexpr float kShaftLength = 2.0f;
  constexpr float kHeadLength = 0.5f;
  const float reach = kShaftLength + kHeadLength;
  const QVector3D dir(std::cos(yaw), std::sin(yaw), 0.f);
  const QVector3D tip = position + dir * reach;
  std::vector<display::ColoredMeshInstance> meshes;
  display::appendSolidArrowMeshes(&meshes, position, tip, color,
                                  kHeadLength / reach, 0.2f, 0.35f);
  if (meshes.empty()) {
    removeDisplay(name);
    return;
  }
  std::vector<OgreColoredMeshInstance> ogre_meshes;
  ogre_meshes.reserve(meshes.size());
  for (const auto& mesh : meshes) {
    ogre_meshes.push_back(
        {mesh.mesh, mesh.transform(), mesh.color, mesh.wireframe});
  }
  setDisplayMeshes(name, ogre_meshes);
  setDisplayVisibilityBits(name, 0xFFFFFFFFu);
}

void OgreSceneHost::setToolPoints(const std::string& tool_id, float point_size,
                                  rendering::PointCloudStyle style,
                                  const std::vector<QVector3D>& positions,
                                  const std::vector<QColor>& colors) {
  const std::string name = "AvizTool/" + tool_id + "/points";
  setDisplayPoints(name, point_size, style, positions, colors);
  setDisplayVisibilityBits(name, 0xFFFFFFFFu);
}

void OgreSceneHost::clearToolOverlay(const std::string& tool_id) {
  removeDisplaysWithPrefix("AvizTool/" + tool_id);
}

void OgreSceneHost::removeDisplaysWithPrefix(const std::string& prefix) {
  std::vector<std::string> to_remove;
  for (const auto& pair : displays_) {
    if (pair.first == prefix ||
        (pair.first.size() > prefix.size() &&
         pair.first.compare(0, prefix.size(), prefix) == 0 &&
         pair.first[prefix.size()] == '/')) {
      to_remove.push_back(pair.first);
    }
  }
  for (const std::string& name : to_remove) {
    removeDisplay(name);
  }
}

void OgreSceneHost::removeDisplay(const std::string& display_name) {
  const auto it = displays_.find(display_name);
  if (it == displays_.end() || scene_manager_ == nullptr) {
    return;
  }
  DisplayEntry& entry = it->second;
  if (entry.cloud != nullptr && entry.node != nullptr) {
    entry.node->detachObject(entry.cloud.get());
  }
  destroyLines(entry);
  destroyMeshes(entry);
  destroyEntities(entry);
  destroyPbr(entry);
  destroyLabels(entry);
  destroyToolLine(entry);
  destroyToolPoseArrow(entry);
  destroyBillboardLine(entry);
  destroyWrench(entry);
  destroyScrew(entry);
  destroyCovariance(entry);
  if (entry.node != nullptr) {
    scene_manager_->destroySceneNode(entry.node);
  }
  cloud_pick_handles_.erase(display_name);
  displays_.erase(it);
}

void OgreSceneHost::clear() {
  if (scene_manager_ == nullptr) {
    displays_.clear();
    cloud_pick_handles_.clear();
    return;
  }
  for (auto& pair : displays_) {
    DisplayEntry& entry = pair.second;
    if (entry.cloud != nullptr && entry.node != nullptr) {
      entry.node->detachObject(entry.cloud.get());
    }
    destroyLines(entry);
    destroyMeshes(entry);
    destroyEntities(entry);
    destroyPbr(entry);
    destroyLabels(entry);
    destroyToolLine(entry);
    destroyBillboardLine(entry);
    destroyWrench(entry);
    destroyScrew(entry);
    destroyCovariance(entry);
    if (entry.node != nullptr) {
      scene_manager_->destroySceneNode(entry.node);
    }
  }
  displays_.clear();
  cloud_pick_handles_.clear();
}

}  // namespace rendering
}  // namespace autoviz

#endif
