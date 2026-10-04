/******************************************************************************
 * Copyright 2008, Willow Garage, Inc. · Copyright 2017, Bosch Software Innovations GmbH.
 * Adapted for Autoviz (BSD-3-Clause).
 *****************************************************************************/

/**
 * @file ogre_point_cloud.hpp
 * @brief Persistent GPU point cloud using rviz ogre_media shaders.
 *
 * rviz_rendering::PointCloud — supports Points/Squares/Spheres/Tiles/Boxes
 * render modes, per-point color, pick colour, and color-by-index for the
 * Pick1 scheme. Uploaded via @ref OgreSceneHost::setDisplayPoints().
 *
 * @see OgrePointCloudRenderable
 * @see PointCloudStyle
 * @see OgreSceneHost
 */

#pragma once

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

/**
 * @class OgrePointCloud
 * @brief rviz_rendering::PointCloud — GPU point cloud MovableObject.
 *
 * ## Data flow
 *
 * Call @ref clear() / @ref addPoints() / @ref popPoints() to mutate CPU
 * points, then the cloud regenerates @ref OgrePointCloudRenderable chunks as
 * needed. Attach to a scene node via the usual Ogre MovableObject API (done
 * by @ref OgreSceneHost).
 */
class OgrePointCloud : public Ogre::MovableObject {
 public:
  /**
   * @enum RenderMode
   * @brief Draw style (maps from @ref PointCloudStyle).
   */
  enum RenderMode {
    kPoints,      /**< GL points. */
    kSquares,     /**< Camera-facing squares. */
    kFlatSquares, /**< Flat squares. */
    kSpheres,     /**< Sphere impostors. */
    kTiles,       /**< Tiles with up vector. */
    kBoxes,       /**< Box geometry. */
  };

  /**
   * @struct Point
   * @brief CPU-side point sample (position + colour).
   */
  struct Point {
    /**
     * @brief Sets RGBA colour components.
     * @param r Red.
     * @param g Green.
     * @param b Blue.
     * @param a Alpha (default 1).
     */
    void setColor(float r, float g, float b, float a = 1.0f) {
      color = Ogre::ColourValue(r, g, b, a);
    }
    Ogre::Vector3 position;   /**< World/parent position. */
    Ogre::ColourValue color;  /**< Point colour. */
  };

  /** @brief Constructs an empty cloud with default sphere mode. */
  OgrePointCloud();

  /** @brief Clears renderables and detaches from the scene. */
  ~OgrePointCloud() override;

  /**
   * @brief Clears points but may keep renderable pool capacity.
   */
  void clear();

  /**
   * @brief Clears points and destroys all renderable chunks.
   */
  void clearAndRemoveAllPoints();

  /**
   * @brief Appends a range of points and updates GPU buffers.
   * @param start Inclusive iterator start.
   * @param end Exclusive iterator end.
   */
  void addPoints(std::vector<Point>::iterator start,
                 std::vector<Point>::iterator end);

  /**
   * @brief Removes @p num_points from the front of the CPU buffer.
   * @param num_points Count to pop.
   */
  void popPoints(uint32_t num_points);

  /**
   * @brief Returns a copy of the current CPU points.
   * @return Point vector.
   */
  std::vector<Point> getPoints();

  /**
   * @brief Sets the render mode and regenerates materials/geometry as needed.
   * @param mode New @ref RenderMode.
   */
  void setRenderMode(RenderMode mode);

  /**
   * @brief Sets point dimensions (width/height/depth custom parameters).
   * @param width Size X.
   * @param height Size Y.
   * @param depth Size Z.
   */
  void setDimensions(float width, float height, float depth);

  /**
   * @brief Enables distance-based auto sizing (shader custom param).
   * @param auto_size Enable flag.
   */
  void setAutoSize(bool auto_size);

  /**
   * @brief Sets the common facing direction (tiles / flat squares).
   * @param vec Direction vector.
   */
  void setCommonDirection(const Ogre::Vector3& vec);

  /**
   * @brief Sets the common up vector (tiles).
   * @param vec Up vector.
   */
  void setCommonUpVector(const Ogre::Vector3& vec);

  /**
   * @brief Sets global alpha and optionally enables per-point alpha.
   * @param alpha Opacity.
   * @param per_point_alpha When @c true, use alpha from each point colour.
   */
  void setAlpha(float alpha, bool per_point_alpha = false);

  /**
   * @brief Sets a uniform colour override.
   * @param color Colour value.
   */
  void setColor(const Ogre::ColourValue& color);

  /**
   * @brief Sets the GPU pick-pass colour (custom parameter).
   * @param color Encoded pick colour.
   */
  void setPickColor(const Ogre::ColourValue& color);

  /**
   * @brief Enables color-by-index mode for Pick1 point identification.
   * @param set Enable flag.
   */
  void setColorByIndex(bool set);

  /**
   * @brief Sets selection highlight tint.
   * @param r Red.
   * @param g Green.
   * @param b Blue.
   */
  void setHighlightColor(float r, float g, float b);

  /** @name Ogre::MovableObject overrides */
  ///@{
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
  ///@}

  /**
   * @brief Sets the MovableObject name.
   * @param name Object name.
   */
  void setName(const std::string& name);

  /**
   * @brief Returns the current renderable queue (shared ptrs).
   * @return Copy of the renderable deque.
   */
  OgrePointCloudRenderableQueue getRenderables();

  /**
   * @brief Vertices emitted per logical point for the current mode.
   * @return Vertex count per point (1 for points, more for expanded modes).
   */
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

