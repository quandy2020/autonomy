/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#pragma once

#ifdef AUTOVIZ_USE_OGRE

#include <array>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include <cstdint>

#include <QColor>
#include <QImage>
#include <QMatrix4x4>
#include <QQuaternion>
#include <QVector3D>

#include "autoviz/common/pick_handle.hpp"
#include "autoviz/display/obj_mesh.hpp"
#include "autoviz/rendering/render_settings.hpp"

namespace Ogre {
class Entity;
class ManualObject;
class SceneManager;
class SceneNode;
}  // namespace Ogre

namespace autoviz {
namespace rendering {

class OgreMovableText;
class OgreLine;
class OgreArrow;
class OgreBillboardLine;
class OgreWrenchVisual;
class OgreScrewVisual;
class OgreCovarianceVisual;

class OgrePointCloud;

struct OgreColoredLineSegment {
  QVector3D a;
  QVector3D b;
  QColor color;
};

struct OgreColoredMeshInstance {
  display::ObjMesh mesh;
  QMatrix4x4 transform;
  QColor color;
  bool wireframe = false;
  common::PickHandle pick_handle = common::kInvalidPickHandle;
};

/** Entity path: mesh registered in MeshManager + per-instance material. */
struct OgreEntityInstance {
  std::string mesh_name;
  QMatrix4x4 transform;
  QColor color;
  common::PickHandle pick_handle = common::kInvalidPickHandle;
};

struct OgrePbrMeshInstance {
  display::ObjMesh mesh;
  QMatrix4x4 transform;
  QColor color;
  float metallic = 0.08f;
  float roughness = 0.52f;
};

struct OgrePbrTexturedMeshInstance {
  display::ObjMesh mesh;
  QMatrix4x4 transform;
  QImage texture;
  QColor tint;
  float metallic = 0.08f;
  float roughness = 0.52f;
};

struct OgreTextLabel {
  std::string text;
  QVector3D position;
  QColor color;
  float char_height = 0.2f;
  float space_width = 0.f;
};

/** Per-display Ogre scene attachment for persistent MovableObjects. */
class OgreSceneHost {
 public:
  explicit OgreSceneHost(Ogre::SceneManager* scene_manager);
  ~OgreSceneHost();

  Ogre::SceneManager* sceneManager() const { return scene_manager_; }

  /** Upload colored points (rviz PointCloud shaders). */
  void setDisplayPoints(const std::string& display_name, float point_size,
                        PointCloudStyle style,
                        const std::vector<QVector3D>& positions,
                        const std::vector<QColor>& colors);

  /** Camera-facing polyline strip (rviz BillboardLine). */
  void setDisplayBillboardStrip(const std::string& display_name,
                                const std::vector<QVector3D>& points,
                                const QColor& color, float line_width);

  /** Cloud-level pick handle for Ogre Pick pass 0 (rviz SelectionHandler handle). */
  void setCloudPickHandle(const std::string& display_name,
                          common::PickHandle handle);
  bool isCloudPickHandle(common::PickHandle handle) const;
  const std::string* displayForCloudPickHandle(common::PickHandle handle) const;
  void setColorByIndexForAll(bool enabled);

  void setDisplayLines(const std::string& display_name,
                       const std::vector<OgreColoredLineSegment>& segments);
  void setDisplayArrow(const std::string& display_name, const QVector3D& start,
                       const QVector3D& end, const QColor& color,
                       float head_fraction = 0.2f);
  void setDisplayMeshes(const std::string& display_name,
                        const std::vector<OgreColoredMeshInstance>& meshes);
  /** rviz Shape-style Entity instances (MeshManager + SceneNode). */
  void setDisplayEntities(const std::string& display_name,
                          const std::vector<OgreEntityInstance>& entities);
  void setDisplayPbrMeshes(const std::string& display_name,
                           const std::vector<OgrePbrMeshInstance>& meshes);
  void setDisplayPbrTexturedMeshes(
      const std::string& display_name,
      const std::vector<OgrePbrTexturedMeshInstance>& meshes);
  void setDisplayLabels(const std::string& display_name,
                        const std::vector<OgreTextLabel>& labels);
  /** rviz WrenchVisual — force arrow + torque circle. */
  void setDisplayWrench(const std::string& display_name, const QVector3D& origin,
                        const QVector3D& force, const QVector3D& torque,
                        const QColor& force_color, const QColor& torque_color,
                        float force_scale, float torque_scale, float width);
  /** rviz ScrewVisual — linear/angular screw. */
  void setDisplayScrew(const std::string& display_name, const QVector3D& origin,
                       const QVector3D& linear, const QVector3D& angular,
                       const QColor& linear_color, const QColor& angular_color,
                       float linear_scale, float angular_scale, float width,
                       bool hide_small_values);
  /** rviz CovarianceVisual — position/orientation ellipsoids. */
  void setDisplayCovariance(
      const std::string& display_name, const QVector3D& position,
      const QQuaternion& pose_orientation, const QQuaternion& frame_orientation,
      const std::array<double, 36>& covariance, const QColor& position_color,
      float position_scale, float orientation_scale, float orientation_offset,
      bool visible);
  void setDisplayVisibilityBits(const std::string& display_name, uint32_t bits);

  /** Tool overlays (Measure, etc.): always visible, outside Display draw cycle. */
  void setToolBillboardStrip(const std::string& tool_id,
                             const std::vector<QVector3D>& points,
                             const QColor& color, float line_width);
  /** RViz MeasureTool-style wire segment (persistent OgreLine). */
  void setToolLineSegment(const std::string& tool_id, const QVector3D& start,
                          const QVector3D& end, const QColor& color);
  void setToolArrow(const std::string& tool_id, const QVector3D& start,
                    const QVector3D& end, const QColor& color,
                    float head_fraction = 0.2f);
  /** RViz PoseTool-style fixed arrow at position + yaw (REP-103 XY ground). */
  void setToolPoseArrow(const std::string& tool_id, const QVector3D& position,
                        float yaw, const QColor& color, bool visible);
  void setToolPoints(const std::string& tool_id, float point_size,
                     rendering::PointCloudStyle style,
                     const std::vector<QVector3D>& positions,
                     const std::vector<QColor>& colors);
  void clearToolOverlay(const std::string& tool_id);

  void removeDisplay(const std::string& display_name);
  /** Remove all entries whose key equals prefix or starts with prefix + '/'. */
  void removeDisplaysWithPrefix(const std::string& prefix);
  void clear();

 private:
  struct DisplayEntry {
    Ogre::SceneNode* node = nullptr;
    std::unique_ptr<OgrePointCloud> cloud;
    Ogre::ManualObject* lines = nullptr;
    Ogre::ManualObject* meshes = nullptr;
    Ogre::ManualObject* pbr_meshes = nullptr;
    std::vector<Ogre::ManualObject*> pbr_textured_objects;
    std::vector<std::string> pbr_texture_names;
    std::vector<std::string> pbr_material_names;
    struct LabelSlot {
      Ogre::SceneNode* node = nullptr;
      std::unique_ptr<OgreMovableText> text;
    };
    std::vector<LabelSlot> labels;
    std::unique_ptr<OgreLine> tool_line;
    std::unique_ptr<OgreArrow> tool_pose_arrow;
    std::unique_ptr<OgreBillboardLine> billboard_line;
    std::unique_ptr<OgreWrenchVisual> wrench;
    std::unique_ptr<OgreScrewVisual> screw;
    std::unique_ptr<OgreCovarianceVisual> covariance;
    struct EntitySlot {
      Ogre::SceneNode* node = nullptr;
      Ogre::Entity* entity = nullptr;
      std::string material_name;
    };
    std::vector<EntitySlot> entities;
    uint32_t visibility_bits = 0xFFFFFFFFu;
  };

  Ogre::SceneNode* displayNode(const std::string& display_name);
  void applyEntryVisibility(DisplayEntry& entry);
  void destroyLines(DisplayEntry& entry);
  void destroyMeshes(DisplayEntry& entry);
  void destroyPbr(DisplayEntry& entry);
  void destroyLabels(DisplayEntry& entry);
  void destroyToolLine(DisplayEntry& entry);
  void destroyToolPoseArrow(DisplayEntry& entry);
  void destroyBillboardLine(DisplayEntry& entry);
  void destroyWrench(DisplayEntry& entry);
  void destroyScrew(DisplayEntry& entry);
  void destroyCovariance(DisplayEntry& entry);
  void destroyEntities(DisplayEntry& entry);
  void uploadLines(Ogre::ManualObject* object,
                   const std::vector<OgreColoredLineSegment>& segments);
  void uploadMeshes(Ogre::ManualObject* object,
                    const std::vector<OgreColoredMeshInstance>& meshes);
  void uploadPbrMeshes(Ogre::ManualObject* object,
                       const std::vector<OgrePbrMeshInstance>& meshes);
  void uploadPbrTexturedMeshes(
      DisplayEntry& entry, const std::string& display_name,
      const std::vector<OgrePbrTexturedMeshInstance>& meshes);

  Ogre::SceneManager* scene_manager_ = nullptr;
  std::unordered_map<std::string, DisplayEntry> displays_;
  std::unordered_map<std::string, common::PickHandle> cloud_pick_handles_;
};

}  // namespace rendering
}  // namespace autoviz

#endif
