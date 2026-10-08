/******************************************************************************
 * Copyright 2008, Willow Garage, Inc.
 * Copyright 2017, Bosch Software Innovations GmbH.
 * Adapted for Autoviz (BSD-3-Clause).
 *****************************************************************************/

#include "autoviz/rendering/objects/ogre_point_cloud.hpp"

#include <algorithm>
#include <sstream>

#include <OgreCamera.h>
#include <OgreMaterialManager.h>
#include <OgreSceneNode.h>
#include <OgreTechnique.h>

#include "autoviz/rendering/ogre_custom_parameter_indices.hpp"

namespace autoviz {
namespace rendering {
namespace {

constexpr float kUnitAlphaThreshold = 0.999f;
constexpr uint32_t kVertexBufferCapacity = 36 * 1024 * 10;

float g_point_vertices[3] = {0.0f, 0.0f, 0.0f};

float g_billboard_vertices[6 * 3] = {
    -0.5f, 0.5f, 0.0f, -0.5f, -0.5f, 0.0f, 0.5f, 0.5f, 0.0f,
    0.5f, 0.5f, 0.0f, -0.5f, -0.5f, 0.0f, 0.5f, -0.5f, 0.0f,
};

float g_billboard_sphere_vertices[3 * 3] = {
    0.0f, 1.0f, 0.0f, -0.866025404f, -0.5f, 0.0f, 0.866025404f, -0.5f, 0.0f,
};

float g_box_vertices[6 * 6 * 3] = {
    -0.5f, 0.5f, -0.5f, -0.5f, -0.5f, -0.5f, 0.5f, 0.5f, -0.5f,
    0.5f, 0.5f, -0.5f, -0.5f, -0.5f, -0.5f, 0.5f, -0.5f, -0.5f,
    -0.5f, 0.5f, 0.5f, 0.5f, 0.5f, 0.5f, -0.5f, -0.5f, 0.5f,
    0.5f, 0.5f, 0.5f, 0.5f, -0.5f, 0.5f, -0.5f, -0.5f, 0.5f,
    0.5f, 0.5f, 0.5f, 0.5f, 0.5f, -0.5f, 0.5f, -0.5f, 0.5f,
    0.5f, 0.5f, -0.5f, 0.5f, -0.5f, -0.5f, 0.5f, -0.5f, 0.5f,
    -0.5f, 0.5f, 0.5f, -0.5f, -0.5f, 0.5f, -0.5f, 0.5f, -0.5f,
    -0.5f, 0.5f, -0.5f, -0.5f, -0.5f, 0.5f, -0.5f, -0.5f, -0.5f,
    -0.5f, 0.5f, -0.5f, 0.5f, 0.5f, -0.5f, -0.5f, 0.5f, 0.5f,
    0.5f, 0.5f, -0.5f, 0.5f, 0.5f, 0.5f, -0.5f, 0.5f, 0.5f,
    -0.5f, -0.5f, -0.5f, -0.5f, -0.5f, 0.5f, 0.5f, -0.5f, -0.5f,
    0.5f, -0.5f, -0.5f, -0.5f, -0.5f, 0.5f, 0.5f, -0.5f, 0.5f,
};

void setAlphaBlending(const Ogre::MaterialPtr& mat) {
  if (mat && mat->getBestTechnique()) {
    mat->getBestTechnique()->setSceneBlending(Ogre::SBT_TRANSPARENT_ALPHA);
    mat->getBestTechnique()->setDepthWriteEnabled(false);
  }
}

void setReplace(const Ogre::MaterialPtr& mat) {
  if (mat && mat->getBestTechnique()) {
    mat->getBestTechnique()->setSceneBlending(Ogre::SBT_REPLACE);
    mat->getBestTechnique()->setDepthWriteEnabled(true);
  }
}

void removeMaterial(Ogre::MaterialPtr& material) {
  if (!material) {
    return;
  }
  Ogre::MaterialManager::getSingleton().remove(material->getHandle());
  material.reset();
}

}  // namespace

Ogre::String OgrePointCloud::sm_type_ = "AvizPointCloud";

uint32_t OgrePointCloud::getVerticesPerPoint() {
  if (current_mode_supports_geometry_shader_) {
    return 1;
  }
  switch (render_mode_) {
    case kPoints:
      return 1;
    case kSquares:
    case kFlatSquares:
    case kTiles:
      return 6;
    case kSpheres:
      return 3;
    case kBoxes:
      return 36;
  }
  return 1;
}

float* OgrePointCloud::getVertices() {
  if (current_mode_supports_geometry_shader_) {
    return g_point_vertices;
  }
  switch (render_mode_) {
    case kPoints:
      return g_point_vertices;
    case kSquares:
    case kFlatSquares:
    case kTiles:
      return g_billboard_vertices;
    case kSpheres:
      return g_billboard_sphere_vertices;
    case kBoxes:
      return g_box_vertices;
  }
  return g_point_vertices;
}

Ogre::MaterialPtr OgrePointCloud::getMaterialForRenderMode(RenderMode mode) {
  switch (mode) {
    case kPoints:
      return point_material_;
    case kSquares:
      return square_material_;
    case kFlatSquares:
      return flat_square_material_;
    case kSpheres:
      return sphere_material_;
    case kTiles:
      return tile_material_;
    case kBoxes:
      return box_material_;
  }
  return point_material_;
}

OgrePointCloud::OgrePointCloud()
    : common_direction_(Ogre::Vector3::NEGATIVE_UNIT_Z),
      common_up_vector_(Ogre::Vector3::UNIT_Y) {
  std::stringstream ss;
  static int count = 0;
  ss << "AvizPointCloudMaterial" << count++;

  auto clone_or_create = [&](const char* base, const std::string& suffix) {
    Ogre::MaterialPtr src =
        Ogre::MaterialManager::getSingleton().getByName(base);
    if (src) {
      return src->clone(ss.str() + suffix);
    }
    return Ogre::MaterialManager::getSingleton().create(
        ss.str() + suffix,
        Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME);
  };

  point_material_ = clone_or_create("aviz/PointCloudPoint", "Point");
  square_material_ = clone_or_create("aviz/PointCloudSquare", "Square");
  flat_square_material_ =
      clone_or_create("aviz/PointCloudFlatSquare", "FlatSquare");
  sphere_material_ = clone_or_create("aviz/PointCloudSphere", "Sphere");
  tile_material_ = clone_or_create("aviz/PointCloudTile", "Tiles");
  box_material_ = clone_or_create("aviz/PointCloudBox", "Box");

  point_material_->load();
  square_material_->load();
  flat_square_material_->load();
  sphere_material_->load();
  tile_material_->load();
  box_material_->load();

  setAlpha(1.0f);
  setRenderMode(kSpheres);
  setDimensions(0.01f, 0.01f, 0.01f);
  clear();
}

OgrePointCloud::~OgrePointCloud() {
  clear();
  if (point_material_) point_material_->unload();
  if (square_material_) square_material_->unload();
  if (flat_square_material_) flat_square_material_->unload();
  if (sphere_material_) sphere_material_->unload();
  if (tile_material_) tile_material_->unload();
  if (box_material_) box_material_->unload();
  removeMaterial(point_material_);
  removeMaterial(square_material_);
  removeMaterial(flat_square_material_);
  removeMaterial(sphere_material_);
  removeMaterial(tile_material_);
  removeMaterial(box_material_);
}

const Ogre::AxisAlignedBox& OgrePointCloud::getBoundingBox() const {
  return bounding_box_;
}

float OgrePointCloud::getBoundingRadius() const {
  return bounding_box_.isNull()
             ? 0.0f
             : Ogre::Math::Sqrt(std::max(bounding_box_.getMaximum().squaredLength(),
                                         bounding_box_.getMinimum().squaredLength()));
}

void OgrePointCloud::getWorldTransforms(Ogre::Matrix4* xform) const {
  *xform = _getParentNodeFullTransform();
}

uint16_t OgrePointCloud::getNumWorldTransforms() const { return 1; }

const Ogre::String& OgrePointCloud::getMovableType() const { return sm_type_; }

void OgrePointCloud::setName(const std::string& name) { mName = name; }

void OgrePointCloud::clear() {
  point_count_ = 0;
  bounding_box_.setNull();
  if (getParentSceneNode()) {
    for (const auto& renderable : renderables_) {
      getParentSceneNode()->detachObject(renderable.get());
    }
    getParentSceneNode()->needUpdate();
  }
  renderables_.clear();
}

void OgrePointCloud::clearAndRemoveAllPoints() {
  clear();
  points_.clear();
}

void OgrePointCloud::regenerateAll() {
  if (point_count_ == 0) {
    return;
  }
  std::vector<Point> points;
  points.swap(points_);
  clear();
  addPoints(points.begin(), points.end());
}

void OgrePointCloud::setColorByIndex(bool set) {
  color_by_index_ = set;
  regenerateAll();
}

void OgrePointCloud::setHighlightColor(float r, float g, float b) {
  const Ogre::Vector4 highlight(r, g, b, 0.0f);
  for (auto& renderable : renderables_) {
    renderable->setCustomParameter(AUTOVIZ_OGRE_HIGHLIGHT_PARAMETER, highlight);
  }
}

bool OgrePointCloud::changingGeometrySupportIsNecessary(
    const Ogre::MaterialPtr material) {
  bool geom_support_changed = false;
  Ogre::Technique* best = material ? material->getBestTechnique() : nullptr;
  if (best) {
    if (best->getName() == "gp") {
      if (!current_mode_supports_geometry_shader_) {
        geom_support_changed = true;
      }
      current_mode_supports_geometry_shader_ = true;
    } else {
      if (current_mode_supports_geometry_shader_) {
        geom_support_changed = true;
      }
      current_mode_supports_geometry_shader_ = false;
    }
  } else {
    geom_support_changed = true;
    current_mode_supports_geometry_shader_ = false;
  }
  return geom_support_changed;
}

void OgrePointCloud::setRenderMode(RenderMode mode) {
  render_mode_ = mode;
  current_material_ = getMaterialForRenderMode(mode);
  if (current_material_) {
    current_material_->load();
  }
  if (changingGeometrySupportIsNecessary(current_material_)) {
    renderables_.clear();
  }
  for (auto& renderable : renderables_) {
    renderable->setMaterial(current_material_);
  }
  regenerateAll();
}

void OgrePointCloud::setDimensions(float width, float height, float depth) {
  point_extensions_ = Ogre::Vector4(width, height, depth, 0.0f);
  for (auto& renderable : renderables_) {
    renderable->setCustomParameter(AUTOVIZ_OGRE_SIZE_PARAMETER, point_extensions_);
  }
}

void OgrePointCloud::setAutoSize(bool auto_size) {
  for (auto& renderable : renderables_) {
    renderable->setCustomParameter(AUTOVIZ_OGRE_AUTO_SIZE_PARAMETER,
                                   Ogre::Vector4(auto_size ? 1.f : 0.f));
  }
}

void OgrePointCloud::setCommonDirection(const Ogre::Vector3& vec) {
  common_direction_ = vec;
  for (auto& renderable : renderables_) {
    renderable->setCustomParameter(AUTOVIZ_OGRE_NORMAL_PARAMETER,
                                   Ogre::Vector4(vec));
  }
}

void OgrePointCloud::setCommonUpVector(const Ogre::Vector3& vec) {
  common_up_vector_ = vec;
  for (auto& renderable : renderables_) {
    renderable->setCustomParameter(AUTOVIZ_OGRE_UP_PARAMETER, Ogre::Vector4(vec));
  }
}

void OgrePointCloud::setAlpha(float alpha, bool per_point_alpha) {
  alpha_ = alpha;
  if (alpha < kUnitAlphaThreshold || per_point_alpha) {
    setAlphaBlending(point_material_);
    setAlphaBlending(square_material_);
    setAlphaBlending(flat_square_material_);
    setAlphaBlending(sphere_material_);
    setAlphaBlending(tile_material_);
    setAlphaBlending(box_material_);
  } else {
    setReplace(point_material_);
    setReplace(square_material_);
    setReplace(flat_square_material_);
    setReplace(sphere_material_);
    setReplace(tile_material_);
    setReplace(box_material_);
  }
  const Ogre::Vector4 alpha4(alpha_, alpha_, alpha_, alpha_);
  for (auto& renderable : renderables_) {
    renderable->setCustomParameter(AUTOVIZ_OGRE_ALPHA_PARAMETER, alpha4);
  }
}

void OgrePointCloud::setColor(const Ogre::ColourValue& color) {
  for (auto& point : points_) {
    point.setColor(color.r, color.g, color.b, color.a);
  }
  regenerateAll();
}

void OgrePointCloud::setPickColor(const Ogre::ColourValue& color) {
  pick_color_ = color;
  const Ogre::Vector4 pick_col(pick_color_.r, pick_color_.g, pick_color_.b,
                               pick_color_.a);
  for (auto& renderable : renderables_) {
    renderable->setCustomParameter(AUTOVIZ_OGRE_PICK_COLOR_PARAMETER, pick_col);
  }
}

Ogre::RenderOperation::OperationType OgrePointCloud::getRenderOperationType() const {
  if (current_mode_supports_geometry_shader_ || render_mode_ == kPoints) {
    return Ogre::RenderOperation::OT_POINT_LIST;
  }
  return Ogre::RenderOperation::OT_TRIANGLE_LIST;
}

OgrePointCloud::RenderableInternals OgrePointCloud::createNewRenderable(
    uint32_t number_of_points_to_be_added) {
  RenderableInternals internals;
  internals.buffer_size = std::min<uint32_t>(
      kVertexBufferCapacity,
      number_of_points_to_be_added * getVerticesPerPoint());
  internals.rend =
      createRenderable(static_cast<int>(internals.buffer_size),
                       getRenderOperationType());
  internals.float_buffer = reinterpret_cast<float*>(
      internals.rend->getBuffer()->lock(Ogre::HardwareBuffer::HBL_NO_OVERWRITE));
  internals.aabb.setNull();
  return internals;
}

void OgrePointCloud::finishRenderable(RenderableInternals internals,
                                      uint32_t vertex_count_of_renderable) {
  Ogre::RenderOperation* op = internals.rend->getRenderOperation();
  op->vertexData->vertexCount =
      vertex_count_of_renderable - op->vertexData->vertexStart;
  internals.rend->setBoundingBox(internals.aabb);
  bounding_box_.merge(internals.aabb);
  internals.rend->getBuffer()->unlock();
}

uint32_t OgrePointCloud::getColorForPoint(
    uint32_t current_point, std::vector<Point>::iterator point) const {
  uint32_t color = 0;
  auto* root = Ogre::Root::getSingletonPtr();
  if (root == nullptr) {
    return color;
  }
  if (color_by_index_) {
    color = (current_point + point_count_ + 1);
    Ogre::ColourValue c;
    c.a = 1.0f;
    c.r = ((color >> 16) & 0xff) / 255.0f;
    c.g = ((color >> 8) & 0xff) / 255.0f;
    c.b = (color & 0xff) / 255.0f;
    root->convertColourValue(c, &color);
  } else {
    root->convertColourValue(point->color, &color);
  }
  return color;
}

OgrePointCloud::RenderableInternals OgrePointCloud::addPointToHardwareBuffer(
    RenderableInternals internals, std::vector<Point>::iterator point,
    uint32_t current_point) {
  const uint32_t color = getColorForPoint(current_point, point);
  float* vertices = getVertices();
  float* float_buffer = internals.float_buffer;
  const float x = point->position.x;
  const float y = point->position.y;
  const float z = point->position.z;
  for (uint32_t j = 0; j < getVerticesPerPoint();
       ++j, ++internals.current_vertex_count) {
    *float_buffer++ = x;
    *float_buffer++ = y;
    *float_buffer++ = z;
    if (!current_mode_supports_geometry_shader_) {
      *float_buffer++ = vertices[(j * 3)];
      *float_buffer++ = vertices[(j * 3) + 1];
      *float_buffer++ = vertices[(j * 3) + 2];
    }
    *reinterpret_cast<uint32_t*>(float_buffer) = color;
    ++float_buffer;
  }
  internals.float_buffer = float_buffer;
  return internals;
}

void OgrePointCloud::addPoints(std::vector<Point>::iterator start,
                               std::vector<Point>::iterator end) {
  if (end - start <= 0) {
    return;
  }
  const auto num_points = static_cast<uint32_t>(std::distance(start, end));
  points_.insert(points_.cend(), start, end);

  RenderableInternals internals = createNewRenderable(num_points);
  for (auto current_point = start; current_point < end; ++current_point) {
    if (internals.bufferIsFull()) {
      finishRenderable(internals, internals.current_vertex_count);
      internals = createNewRenderable(
          static_cast<uint32_t>(end - current_point));
    }
    internals.aabb.merge(current_point->position);
    internals = addPointToHardwareBuffer(
        internals, current_point,
        static_cast<uint32_t>(current_point - start));
  }
  finishRenderable(internals, internals.current_vertex_count);
  point_count_ += num_points;
  if (getParentSceneNode()) {
    getParentSceneNode()->needUpdate();
  }
}

void OgrePointCloud::popPoints(uint32_t num_points) {
  if (num_points > point_count_) {
    num_points = point_count_;
  }
  points_.erase(points_.begin(), points_.begin() + num_points);
  point_count_ -= num_points;
  removePointsFromRenderables(num_points, getVerticesPerPoint());
  resetBoundingBoxForCurrentPoints();
  if (getParentSceneNode()) {
    getParentSceneNode()->needUpdate();
  }
}

std::vector<OgrePointCloud::Point> OgrePointCloud::getPoints() { return points_; }

size_t OgrePointCloud::removePointsFromRenderables(uint32_t number_of_points,
                                                   uint32_t vertices_per_point) {
  size_t popped_count = 0;
  while (popped_count < number_of_points * vertices_per_point &&
         !renderables_.empty()) {
    OgrePointCloudRenderablePtr rend = renderables_.front();
    Ogre::RenderOperation* op = rend->getRenderOperation();
    const size_t popped_in_renderable = std::min(
        static_cast<size_t>(number_of_points * vertices_per_point - popped_count),
        static_cast<size_t>(op->vertexData->vertexCount));
    op->vertexData->vertexStart += popped_in_renderable;
    op->vertexData->vertexCount -= popped_in_renderable;
    popped_count += popped_in_renderable;
    if (op->vertexData->vertexCount == 0) {
      renderables_.pop_front();
    }
  }
  return popped_count;
}

void OgrePointCloud::resetBoundingBoxForCurrentPoints() {
  bounding_box_.setNull();
  for (uint32_t i = 0; i < point_count_; ++i) {
    bounding_box_.merge(points_[i].position);
  }
}

void OgrePointCloud::_notifyCurrentCamera(Ogre::Camera* camera) {
  Ogre::MovableObject::_notifyCurrentCamera(camera);
}

void OgrePointCloud::_updateRenderQueue(Ogre::RenderQueue* queue) {
  for (auto& renderable : renderables_) {
    queue->addRenderable(renderable.get());
  }
}

void OgrePointCloud::_notifyAttached(Ogre::Node* parent, bool is_tag_point) {
  Ogre::MovableObject::_notifyAttached(parent, is_tag_point);
}

void OgrePointCloud::visitRenderables(Ogre::Renderable::Visitor* /*visitor*/,
                                      bool /*debug_renderables*/) {}

OgrePointCloudRenderablePtr OgrePointCloud::createRenderable(
    int num_points, Ogre::RenderOperation::OperationType operation_type) {
  OgrePointCloudRenderablePtr rend(new OgrePointCloudRenderable(
      this, num_points, !current_mode_supports_geometry_shader_,
      operation_type));
  rend->setMaterial(current_material_);
  const Ogre::Vector4 alpha(alpha_, 0.0f, 0.0f, 0.0f);
  const Ogre::Vector4 highlight(0.0f, 0.0f, 0.0f, 0.0f);
  const Ogre::Vector4 pick_col(pick_color_.r, pick_color_.g, pick_color_.b,
                               pick_color_.a);
  rend->setCustomParameter(AUTOVIZ_OGRE_SIZE_PARAMETER, point_extensions_);
  rend->setCustomParameter(AUTOVIZ_OGRE_ALPHA_PARAMETER, alpha);
  rend->setCustomParameter(AUTOVIZ_OGRE_HIGHLIGHT_PARAMETER, highlight);
  rend->setCustomParameter(AUTOVIZ_OGRE_PICK_COLOR_PARAMETER, pick_col);
  rend->setCustomParameter(AUTOVIZ_OGRE_NORMAL_PARAMETER,
                           Ogre::Vector4(common_direction_));
  rend->setCustomParameter(AUTOVIZ_OGRE_UP_PARAMETER,
                           Ogre::Vector4(common_up_vector_));
  if (getParentSceneNode()) {
    getParentSceneNode()->attachObject(rend.get());
  }
  renderables_.push_back(rend);
  return rend;
}

OgrePointCloudRenderableQueue OgrePointCloud::getRenderables() {
  return renderables_;
}

}  // namespace rendering
}  // namespace autoviz

