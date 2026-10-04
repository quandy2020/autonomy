/******************************************************************************
 * Copyright 2008, Willow Garage, Inc. · Copyright 2017, Bosch Software Innovations GmbH.
 * Adapted for Autoviz (BSD-3-Clause).
 *****************************************************************************/

/**
 * @file ogre_point_cloud_renderable.hpp
 * @brief Ogre SimpleRenderable chunk for @ref OgrePointCloud hardware buffers.
 *
 * Each renderable owns a vertex buffer slice of the parent cloud. Created and
 * pooled by @ref OgrePointCloud when regenerating GPU geometry.
 *
 * @see OgrePointCloud
 * @see AUTOVIZ_OGRE_SIZE_PARAMETER
 */

#pragma once

#include <deque>
#include <memory>

#include <OgreSimpleRenderable.h>

namespace Ogre {
class Camera;
class Matrix4;
}  // namespace Ogre

namespace autoviz {
namespace rendering {

class OgrePointCloud;

/**
 * @class OgrePointCloudRenderable
 * @brief One GPU buffer chunk of an @ref OgrePointCloud.
 *
 * Implements Ogre renderable queries (bounding radius, view depth, lights,
 * world transforms) by delegating to the parent cloud's scene node.
 */
class OgrePointCloudRenderable : public Ogre::SimpleRenderable {
 public:
  /**
   * @brief Allocates a vertex buffer for @p num_points.
   *
   * @param parent Owning point cloud (non-owning).
   * @param num_points Capacity in points.
   * @param use_tex_coords Whether the vertex format includes UVs.
   * @param operation_type @c OT_POINT_LIST or geometry-shader expanded ops.
   */
  OgrePointCloudRenderable(OgrePointCloud* parent, int num_points, bool use_tex_coords,
                           Ogre::RenderOperation::OperationType operation_type);

  /** @brief Releases the hardware vertex buffer. */
  ~OgrePointCloudRenderable() override;

  /**
   * @brief Returns the mutable render operation (for buffer fills).
   * @return Pointer to @c mRenderOp.
   */
  Ogre::RenderOperation* getRenderOperation() { return &mRenderOp; }
  using Ogre::SimpleRenderable::getRenderOperation;

  /**
   * @brief Returns the shared vertex buffer pointer.
   * @return Hardware vertex buffer.
   */
  Ogre::HardwareVertexBufferSharedPtr getBuffer();

  /**
   * @brief Bounding radius for LOD / culling.
   * @return Radius in world units.
   */
  Ogre::Real getBoundingRadius() const override;

  /**
   * @brief Squared distance from camera for transparent sorting.
   * @param cam Active camera.
   * @return Squared view depth.
   */
  Ogre::Real getSquaredViewDepth(const Ogre::Camera* cam) const override;

  /**
   * @brief Notifies the renderable of the current camera (custom params).
   * @param camera Active camera.
   */
  void _notifyCurrentCamera(Ogre::Camera* camera) override;

  /**
   * @brief Number of world transforms (usually 1).
   * @return Transform count.
   */
  uint16_t getNumWorldTransforms() const override;

  /**
   * @brief Fills world transform matrix from the parent node.
   * @param[out] xform Destination matrix.
   */
  void getWorldTransforms(Ogre::Matrix4* xform) const override;

  /**
   * @brief Returns the light list from the parent movable.
   * @return Const light list reference.
   */
  const Ogre::LightList& getLights() const override;

 private:
  void initializeRenderOperation(Ogre::RenderOperation::OperationType operation_type);
  void specifyBufferContent(bool use_tex_coords);
  void createAndBindBuffer(int num_points);

  OgrePointCloud* parent_ = nullptr; /**< Non-owning parent cloud. */
};

/** @brief Shared pointer alias for renderable chunks. */
using OgrePointCloudRenderablePtr = std::shared_ptr<OgrePointCloudRenderable>;

/** @brief Queue / pool of renderable chunks. */
using OgrePointCloudRenderableQueue = std::deque<OgrePointCloudRenderablePtr>;

}  // namespace rendering
}  // namespace autoviz

