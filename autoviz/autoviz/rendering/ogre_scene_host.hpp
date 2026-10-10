/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

/**
 * @file ogre_scene_host.hpp
 * @brief Per-display persistent Ogre MovableObject attachments.
 *
 * Displays upload points, lines, meshes, labels, wrench/screw/covariance
 * visuals, and tool overlays into named entries. Unlike @ref SceneOverlay
 * (rebuilt each frame on the CPU path), this host keeps GPU objects across
 * frames and only updates when setters are called.
 *
 * @see OgreRenderBackend
 * @see OgrePointCloud
 * @see OgreBillboardLine
 * @see SceneOverlay
 */

#pragma once

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

/**
 * @struct OgreColoredLineSegment
 * @brief Colored line segment for @ref OgreSceneHost::setDisplayLines().
 */
struct OgreColoredLineSegment {
  QVector3D a; /**< Start point. */
  QVector3D b; /**< End point. */
  QColor color; /**< Segment color. */
};

/**
 * @struct OgreColoredMeshInstance
 * @brief Transformed ObjMesh instance (solid or wireframe ManualObject path).
 */
struct OgreColoredMeshInstance {
  display::ObjMesh mesh; /**< Triangle mesh. */
  QMatrix4x4 transform;  /**< World transform. */
  QColor color;          /**< Tint color. */
  bool wireframe = false; /**< Draw edges instead of filled triangles. */
  common::PickHandle pick_handle = common::kInvalidPickHandle; /**< Optional pick. */
};

/**
 * @struct OgreEntityInstance
 * @brief Entity path: mesh registered in MeshManager + per-instance material.
 */
struct OgreEntityInstance {
  std::string mesh_name; /**< MeshManager resource name. */
  QMatrix4x4 transform;  /**< World transform. */
  QColor color;          /**< Instance color. */
  common::PickHandle pick_handle = common::kInvalidPickHandle; /**< Optional pick. */
};

/**
 * @struct OgrePbrMeshInstance
 * @brief Untextured PBR ObjMesh instance.
 */
struct OgrePbrMeshInstance {
  display::ObjMesh mesh; /**< Triangle mesh. */
  QMatrix4x4 transform;  /**< World transform. */
  QColor color;          /**< Albedo tint. */
  float metallic = 0.08f; /**< Metallic factor. */
  float roughness = 0.52f; /**< Roughness factor. */
};

/**
 * @struct OgrePbrTexturedMeshInstance
 * @brief Textured PBR ObjMesh instance.
 */
struct OgrePbrTexturedMeshInstance {
  display::ObjMesh mesh; /**< Triangle mesh. */
  QMatrix4x4 transform;  /**< World transform. */
  QImage texture;        /**< Albedo texture. */
  QColor tint;           /**< Color tint. */
  float metallic = 0.08f; /**< Metallic factor. */
  float roughness = 0.52f; /**< Roughness factor. */
};

/**
 * @struct OgreTexturedLinkInstance
 * @brief One rigid textured link. Mesh and texture stay on the GPU; only
 * @c transform is applied on later frames.
 */
struct OgreTexturedLinkInstance {
  const display::ObjMesh* mesh = nullptr; /**< Stable mesh, not copied. */
  const QImage* texture = nullptr;        /**< Stable albedo image. */
  QMatrix4x4 transform;                   /**< World transform, including scale. */
  QColor tint;                            /**< Vertex-color tint. */
  float metallic = 0.08f;                 /**< Unused by the unlit material. */
  float roughness = 0.52f;                /**< Unused by the unlit material. */
};

/**
 * @struct OgreTextLabel
 * @brief 3D text label parameters for MovableText slots.
 */
struct OgreTextLabel {
  std::string text;          /**< Label contents. */
  QVector3D position;        /**< World position. */
  QColor color;              /**< Text color. */
  float char_height = 0.2f;  /**< Character height in meters. */
  float space_width = 0.f;   /**< Optional space width override (0 = default). */
};

/**
 * @class OgreSceneHost
 * @brief Per-display Ogre scene attachment for persistent MovableObjects.
 *
 * ## Keys
 *
 * Display geometry is keyed by @p display_name. Tool overlays use @p tool_id
 * and are intended to remain outside the Display draw cycle.
 *
 * ## Visibility
 *
 * @ref setDisplayVisibilityBits() applies Ogre visibility masks via
 * @ref applyVisibilityBits().
 */
class OgreSceneHost {
 public:
  /**
   * @brief Creates an empty host bound to @p scene_manager.
   * @param scene_manager Non-null Ogre scene manager.
   */
  explicit OgreSceneHost(Ogre::SceneManager* scene_manager);

  /** @brief Destroys all display entries and owned Ogre objects. */
  ~OgreSceneHost();

  /**
   * @brief Returns the bound scene manager.
   * @return Non-owning scene manager pointer.
   */
  Ogre::SceneManager* sceneManager() const { return scene_manager_; }

  /**
   * @brief Uploads colored points (rviz PointCloud shaders).
   *
   * @param display_name Entry key.
   * @param point_size Point / billboard size.
   * @param style Draw style (@ref PointCloudStyle).
   * @param positions Point positions.
   * @param colors Per-point colors (size should match positions).
   */
  void setDisplayPoints(const std::string& display_name, float point_size,
                        PointCloudStyle style,
                        const std::vector<QVector3D>& positions,
                        const std::vector<QColor>& colors);

  /**
   * @brief Camera-facing polyline strip (rviz BillboardLine).
   *
   * @param display_name Entry key.
   * @param points Polyline vertices.
   * @param color Strip color.
   * @param line_width Width in meters.
   */
  void setDisplayBillboardStrip(const std::string& display_name,
                                const std::vector<QVector3D>& points,
                                const QColor& color, float line_width);

  /**
   * @brief Cloud-level pick handle for Ogre Pick pass 0.
   *
   * @param display_name Entry key.
   * @param handle Handle assigned to the whole cloud (rviz SelectionHandler).
   */
  void setCloudPickHandle(const std::string& display_name,
                          common::PickHandle handle);

  /**
   * @brief Whether @p handle is registered as a cloud-level pick handle.
   * @param handle Candidate handle.
   * @return @c true if mapped to a display name.
   */
  bool isCloudPickHandle(common::PickHandle handle) const;

  /**
   * @brief Looks up the display name for a cloud pick handle.
   * @param handle Cloud-level handle.
   * @return Pointer to stored name, or @c nullptr if unknown.
   */
  const std::string* displayForCloudPickHandle(common::PickHandle handle) const;

  /**
   * @brief Enables color-by-index mode on all point clouds.
   * @param enabled When @c true, points encode index in color for Pick1.
   */
  void setColorByIndexForAll(bool enabled);

  /**
   * @brief Replaces line ManualObject geometry for a display.
   */
  void setDisplayLines(const std::string& display_name,
                       const std::vector<OgreColoredLineSegment>& segments);

  /**
   * @brief Places a single arrow from @p start to @p end.
   */
  void setDisplayArrow(const std::string& display_name, const QVector3D& start,
                       const QVector3D& end, const QColor& color,
                       float head_fraction = 0.2f);

  /**
   * @brief Uploads colored / wireframe ObjMesh instances via ManualObject.
   */
  void setDisplayMeshes(const std::string& display_name,
                        const std::vector<OgreColoredMeshInstance>& meshes);

  /**
   * @brief rviz Shape-style Entity instances (MeshManager + SceneNode).
   */
  void setDisplayEntities(const std::string& display_name,
                          const std::vector<OgreEntityInstance>& entities);

  /**
   * @brief Uploads untextured PBR meshes.
   */
  void setDisplayPbrMeshes(const std::string& display_name,
                           const std::vector<OgrePbrMeshInstance>& meshes);

  /**
   * @brief Uploads textured PBR meshes.
   */
  void setDisplayPbrTexturedMeshes(
      const std::string& display_name,
      const std::vector<OgrePbrTexturedMeshInstance>& meshes);

  /**
   * @brief Draws textured links. Geometry is uploaded once per mesh; later
   * calls with the same meshes only move the scene nodes.
   */
  void setDisplayTexturedLinks(
      const std::string& display_name,
      const std::vector<OgreTexturedLinkInstance>& links);

  /**
   * @brief Updates text label slots for a display.
   */
  void setDisplayLabels(const std::string& display_name,
                        const std::vector<OgreTextLabel>& labels);

  /**
   * @brief rviz WrenchVisual — force arrow + torque circle.
   */
  void setDisplayWrench(const std::string& display_name, const QVector3D& origin,
                        const QVector3D& force, const QVector3D& torque,
                        const QColor& force_color, const QColor& torque_color,
                        float force_scale, float torque_scale, float width);

  /**
   * @brief rviz ScrewVisual — linear/angular screw.
   */
  void setDisplayScrew(const std::string& display_name, const QVector3D& origin,
                       const QVector3D& linear, const QVector3D& angular,
                       const QColor& linear_color, const QColor& angular_color,
                       float linear_scale, float angular_scale, float width,
                       bool hide_small_values);

  /**
   * @brief rviz CovarianceVisual — position/orientation ellipsoids.
   */
  void setDisplayCovariance(
      const std::string& display_name, const QVector3D& position,
      const QQuaternion& pose_orientation, const QQuaternion& frame_orientation,
      const std::array<double, 36>& covariance, const QColor& position_color,
      float position_scale, float orientation_scale, float orientation_offset,
      bool visible);

  /**
   * @brief Sets Ogre visibility bits for all movables under a display entry.
   * @param display_name Entry key.
   * @param bits Visibility mask.
   * @see applyVisibilityBits()
   */
  void setDisplayVisibilityBits(const std::string& display_name, uint32_t bits);

  /**
   * @brief Tool overlays (Measure, etc.): always visible, outside Display cycle.
   */
  void setToolBillboardStrip(const std::string& tool_id,
                             const std::vector<QVector3D>& points,
                             const QColor& color, float line_width);

  /**
   * @brief RViz MeasureTool-style wire segment (persistent @ref OgreLine).
   */
  void setToolLineSegment(const std::string& tool_id, const QVector3D& start,
                          const QVector3D& end, const QColor& color);

  /**
   * @brief Tool arrow overlay.
   */
  void setToolArrow(const std::string& tool_id, const QVector3D& start,
                    const QVector3D& end, const QColor& color,
                    float head_fraction = 0.2f);

  /**
   * @brief RViz PoseTool-style fixed arrow at position + yaw (REP-103 XY ground).
   */
  void setToolPoseArrow(const std::string& tool_id, const QVector3D& position,
                        float yaw, const QColor& color, bool visible);

  /**
   * @brief Tool point cloud overlay.
   */
  void setToolPoints(const std::string& tool_id, float point_size,
                     rendering::PointCloudStyle style,
                     const std::vector<QVector3D>& positions,
                     const std::vector<QColor>& colors);

  /**
   * @brief Removes all geometry for a tool overlay key.
   * @param tool_id Tool entry key.
   */
  void clearToolOverlay(const std::string& tool_id);

  /**
   * @brief Removes a single display entry and destroys its Ogre objects.
   * @param display_name Entry key.
   */
  void removeDisplay(const std::string& display_name);

  /**
   * @brief Remove all entries whose key equals prefix or starts with prefix + '/'.
   * @param prefix Display name prefix.
   */
  void removeDisplaysWithPrefix(const std::string& prefix);

  /**
   * @brief Clears every display and tool entry.
   */
  void clear();

 private:
  /**
   * @struct DisplayEntry
   * @brief Owned Ogre objects for one display or tool key.
   */
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
    struct TexturedLinkSlot {
      Ogre::SceneNode* node = nullptr;
      Ogre::ManualObject* object = nullptr;
      const display::ObjMesh* mesh = nullptr;
      qint64 texture_key = 0;
    };
    std::vector<TexturedLinkSlot> textured_links;
    std::vector<std::string> textured_link_texture_names;
    std::vector<std::string> textured_link_material_names;
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
  void destroyTexturedLinks(DisplayEntry& entry);
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

