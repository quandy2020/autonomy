/******************************************************************************
 * Copyright 2008, Willow Garage, Inc. · Copyright 2017, Bosch Software Innovations GmbH.
 * Adapted for Autoviz (BSD-3-Clause).
 *****************************************************************************/

#pragma once

#ifdef AUTOVIZ_USE_OGRE

#include <cstdint>
#include <deque>
#include <memory>
#include <string>
#include <vector>

#include <OgreAxisAlignedBox.h>
#include <OgreColourValue.h>
#include <OgreHardwareBufferManager.h>
#include <OgreMaterial.h>
#include <OgreMovableObject.h>
#include <OgreRoot.h>
#include <OgreSharedPtr.h>
#include <OgreSimpleRenderable.h>
#include <OgreString.h>
#include <OgreVector.h>

#include "autoviz/rendering/objects/ogre_point_cloud_renderable.hpp"

namespace Ogre {
class Camera;
class ManualObject;
class Matrix4;
class RenderQueue;
class RenderSystem;
class SceneManager;
class SceneNode;
}  // namespace Ogre

namespace autoviz {
namespace rendering {

/** rviz_rendering::PointCloud — persistent GPU point cloud using ogre_media shaders. */
class OgrePointCloud : public Ogre::MovableObject {
 public:
  enum RenderMode {
    kPoints,
    kSquares,
    kFlatSquares,
    kSpheres,
    kTiles,
    kBoxes,
  };

  struct Point {
    void setColor(float r, float g, float b, float a = 1.0f) {
      color = Ogre::ColourValue(r, g, b, a);
    }
    Ogre::Vector3 position;
    Ogre::ColourValue color;
  };

  OgrePointCloud();
  ~OgrePointCloud() override;

  void clear();
  void clearAndRemoveAllPoints();
  void addPoints(std::vector<Point>::iterator start,
                 std::vector<Point>::iterator end);
  void popPoints(uint32_t num_points);
  std::vector<Point> getPoints();

  void setRenderMode(RenderMode mode);
  void setDimensions(float width, float height, float depth);
  void setAutoSize(bool auto_size);
  void setCommonDirection(const Ogre::Vector3& vec);
  void setCommonUpVector(const Ogre::Vector3& vec);
  void setAlpha(float alpha, bool per_point_alpha = false);
  void setColor(const Ogre::ColourValue& color);
  void setPickColor(const Ogre::ColourValue& color);
  void setColorByIndex(bool set);
  void setHighlightColor(float r, float g, float b);

  const Ogre::String& getMovableType() const override;
  const Ogre::AxisAlignedBox& getBoundingBox() const override;
  float getBoundingRadius() const override;
  void getWorldTransforms(Ogre::Matrix4* xform) const;
  uint16_t getNumWorldTransforms() const;
  void _updateRenderQueue(Ogre::RenderQueue* queue) override;
  void _notifyCurrentCamera(Ogre::Camera* camera) override;
  void _notifyAttached(Ogre::Node* parent, bool is_tag_point = false) override;
  void visitRenderables(Ogre::Renderable::Visitor* visitor,
                        bool debug_renderables) override;
  void setName(const std::string& name);
  OgrePointCloudRenderableQueue getRenderables();
  uint32_t getVerticesPerPoint();

 private:
  struct RenderableInternals {
    bool bufferIsFull() const { return current_vertex_count >= buffer_size; }
    bool noBufferOverflowOccurred() const {
      return current_vertex_count == buffer_size;
    }
    OgrePointCloudRenderablePtr rend;
    float* float_buffer = nullptr;
    uint32_t buffer_size = 0;
    Ogre::AxisAlignedBox aabb;
    uint32_t current_vertex_count = 0;
  };

  float* getVertices();
  Ogre::MaterialPtr getMaterialForRenderMode(RenderMode render_mode);
  bool changingGeometrySupportIsNecessary(const Ogre::MaterialPtr material);
  OgrePointCloudRenderablePtr createRenderable(
      int num_points, Ogre::RenderOperation::OperationType operation_type);
  void regenerateAll();
  size_t removePointsFromRenderables(uint32_t number_of_points,
                                     uint32_t vertices_per_point);
  void resetBoundingBoxForCurrentPoints();
  RenderableInternals createNewRenderable(uint32_t number_of_points_to_be_added);
  Ogre::RenderOperation::OperationType getRenderOperationType() const;
  void finishRenderable(RenderableInternals internals,
                        uint32_t vertex_count_of_renderable);
  uint32_t getColorForPoint(uint32_t current_point,
                            std::vector<Point>::iterator point) const;
  RenderableInternals addPointToHardwareBuffer(
      RenderableInternals internals, std::vector<Point>::iterator point,
      uint32_t current_point);

  Ogre::AxisAlignedBox bounding_box_;
  std::vector<Point> points_;
  uint32_t point_count_ = 0;
  RenderMode render_mode_ = kSpheres;
  Ogre::Vector4 point_extensions_;
  Ogre::Vector3 common_direction_;
  Ogre::Vector3 common_up_vector_;
  Ogre::MaterialPtr point_material_;
  Ogre::MaterialPtr square_material_;
  Ogre::MaterialPtr flat_square_material_;
  Ogre::MaterialPtr sphere_material_;
  Ogre::MaterialPtr tile_material_;
  Ogre::MaterialPtr box_material_;
  Ogre::MaterialPtr current_material_;
  float alpha_ = 1.f;
  bool color_by_index_ = false;
  OgrePointCloudRenderableQueue renderables_;
  bool current_mode_supports_geometry_shader_ = false;
  Ogre::ColourValue pick_color_;
  static Ogre::String sm_type_;
};

}  // namespace rendering
}  // namespace autoviz

#endif
