/******************************************************************************
 * Copyright 2008, Willow Garage, Inc. · Copyright 2017, Bosch Software Innovations GmbH.
 * Adapted for Autoviz (BSD-3-Clause).
 *****************************************************************************/

#pragma once

#ifdef AUTOVIZ_USE_OGRE

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

class OgrePointCloudRenderable : public Ogre::SimpleRenderable {
 public:
  OgrePointCloudRenderable(OgrePointCloud* parent, int num_points, bool use_tex_coords,
                           Ogre::RenderOperation::OperationType operation_type);
  ~OgrePointCloudRenderable() override;

  Ogre::RenderOperation* getRenderOperation() { return &mRenderOp; }
  using Ogre::SimpleRenderable::getRenderOperation;

  Ogre::HardwareVertexBufferSharedPtr getBuffer();
  Ogre::Real getBoundingRadius() const override;
  Ogre::Real getSquaredViewDepth(const Ogre::Camera* cam) const override;
  void _notifyCurrentCamera(Ogre::Camera* camera) override;
  uint16_t getNumWorldTransforms() const override;
  void getWorldTransforms(Ogre::Matrix4* xform) const override;
  const Ogre::LightList& getLights() const override;

 private:
  void initializeRenderOperation(Ogre::RenderOperation::OperationType operation_type);
  void specifyBufferContent(bool use_tex_coords);
  void createAndBindBuffer(int num_points);

  OgrePointCloud* parent_ = nullptr;
};

using OgrePointCloudRenderablePtr = std::shared_ptr<OgrePointCloudRenderable>;
using OgrePointCloudRenderableQueue = std::deque<OgrePointCloudRenderablePtr>;

}  // namespace rendering
}  // namespace autoviz

#endif
