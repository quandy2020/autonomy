/******************************************************************************
 * Copyright 2008, Willow Garage, Inc. · Copyright 2017, Bosch Software Innovations GmbH.
 * Adapted for Autoviz (BSD-3-Clause).
 *****************************************************************************/

#include "autoviz/rendering/objects/ogre_point_cloud_renderable.hpp"

#ifdef AUTOVIZ_USE_OGRE

#include <algorithm>

#include <OgreCamera.h>

#include "autoviz/rendering/objects/ogre_point_cloud.hpp"

namespace autoviz {
namespace rendering {

OgrePointCloudRenderable::OgrePointCloudRenderable(
    OgrePointCloud* parent, int num_points, bool use_tex_coords,
    Ogre::RenderOperation::OperationType operation_type)
    : parent_(parent) {
  initializeRenderOperation(operation_type);
  specifyBufferContent(use_tex_coords);
  createAndBindBuffer(num_points);
}

OgrePointCloudRenderable::~OgrePointCloudRenderable() {
  delete mRenderOp.vertexData;
  delete mRenderOp.indexData;
}

Ogre::HardwareVertexBufferSharedPtr OgrePointCloudRenderable::getBuffer() {
  return mRenderOp.vertexData->vertexBufferBinding->getBuffer(0);
}

void OgrePointCloudRenderable::_notifyCurrentCamera(Ogre::Camera* camera) {
  Ogre::SimpleRenderable::_notifyCurrentCamera(camera);
}

Ogre::Real OgrePointCloudRenderable::getBoundingRadius() const {
  return Ogre::Math::Sqrt(std::max(mBox.getMaximum().squaredLength(),
                                   mBox.getMinimum().squaredLength()));
}

Ogre::Real OgrePointCloudRenderable::getSquaredViewDepth(
    const Ogre::Camera* cam) const {
  const Ogre::Vector3 vMid =
      ((mBox.getMaximum() - mBox.getMinimum()) * 0.5f) + mBox.getMinimum();
  const Ogre::Vector3 vDist = cam->getDerivedPosition() - vMid;
  return vDist.squaredLength();
}

void OgrePointCloudRenderable::getWorldTransforms(Ogre::Matrix4* xform) const {
  parent_->getWorldTransforms(xform);
}

uint16_t OgrePointCloudRenderable::getNumWorldTransforms() const { return 1; }

const Ogre::LightList& OgrePointCloudRenderable::getLights() const {
  return parent_->queryLights();
}

void OgrePointCloudRenderable::initializeRenderOperation(
    Ogre::RenderOperation::OperationType operation_type) {
  mRenderOp.operationType = operation_type;
  mRenderOp.useIndexes = false;
  mRenderOp.vertexData = new Ogre::VertexData;
  mRenderOp.vertexData->vertexStart = 0;
  mRenderOp.vertexData->vertexCount = 0;
}

void OgrePointCloudRenderable::specifyBufferContent(bool use_tex_coords) {
  Ogre::VertexDeclaration* declaration = mRenderOp.vertexData->vertexDeclaration;
  size_t offset = 0;
  declaration->addElement(0, offset, Ogre::VET_FLOAT3, Ogre::VES_POSITION);
  offset += Ogre::VertexElement::getTypeSize(Ogre::VET_FLOAT3);
  if (use_tex_coords) {
    declaration->addElement(0, offset, Ogre::VET_FLOAT3,
                            Ogre::VES_TEXTURE_COORDINATES, 0);
    offset += Ogre::VertexElement::getTypeSize(Ogre::VET_FLOAT3);
  }
  declaration->addElement(0, offset, Ogre::VET_COLOUR, Ogre::VES_DIFFUSE);
}

void OgrePointCloudRenderable::createAndBindBuffer(int num_points) {
  Ogre::HardwareVertexBufferSharedPtr vertex_buffer =
      Ogre::HardwareBufferManager::getSingleton().createVertexBuffer(
          mRenderOp.vertexData->vertexDeclaration->getVertexSize(0), num_points,
          Ogre::HardwareBuffer::HBU_DYNAMIC);
  mRenderOp.vertexData->vertexBufferBinding->setBinding(0, vertex_buffer);
}

}  // namespace rendering
}  // namespace autoviz

#endif
