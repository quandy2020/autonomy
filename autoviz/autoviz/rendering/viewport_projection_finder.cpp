/******************************************************************************
 * Copyright 2012, Willow Garage, Inc. · Copyright 2017–2018, Open Source Robotics Foundation, Inc.
 * Adapted for Autoviz (BSD-3-Clause).
 *****************************************************************************/

#include "autoviz/rendering/viewport_projection_finder.hpp"

#include <OgreCamera.h>
#include <OgrePlane.h>
#include <OgreRay.h>
#include <OgreVector3.h>

#ifdef AUTOVIZ_USE_OGRE
#include <OgreViewport.h>
#endif

namespace autoviz {
namespace rendering {
namespace {

QVector3D toQVector3(const Ogre::Vector3& v) {
  return QVector3D(v.x(), v.y(), v.z());
}

bool intersectPlane(const QVector3D& origin, const QVector3D& direction,
                    const QVector3D& plane_normal, float plane_offset,
                    QVector3D* hit) {
  const float denom = QVector3D::dotProduct(direction, plane_normal);
  if (std::abs(denom) < 1e-8f) {
    return false;
  }
  const float t = -(QVector3D::dotProduct(origin, plane_normal) + plane_offset) / denom;
  if (t < 0.f) {
    return false;
  }
  if (hit != nullptr) {
    *hit = origin + direction * t;
  }
  return true;
}

}  // namespace

std::pair<bool, QVector3D> ViewportProjectionFinder::projectOnGroundPlane(
    int pixel_x, int pixel_y, int viewport_width, int viewport_height,
    const QMatrix4x4& view, const QMatrix4x4& projection) {
  if (viewport_width <= 0 || viewport_height <= 0) {
    return {false, {}};
  }
  const QMatrix4x4 inverse = (projection * view).inverted();
  const float ndc_x =
      (2.f * static_cast<float>(pixel_x) / static_cast<float>(viewport_width)) - 1.f;
  const float ndc_y =
      1.f - (2.f * static_cast<float>(pixel_y) / static_cast<float>(viewport_height));
  const QVector3D far_point = inverse.map(QVector3D(ndc_x, ndc_y, 1.f));
  const QVector3D origin = inverse.map(QVector3D(0.f, 0.f, 0.f));
  const QVector3D direction = (far_point - origin).normalized();
  QVector3D hit;
  if (!intersectPlane(origin, direction, QVector3D(0.f, 0.f, 1.f), 0.f, &hit)) {
    return {false, {}};
  }
  return {true, hit};
}

#ifdef AUTOVIZ_USE_OGRE

std::pair<bool, QVector3D> ViewportProjectionFinder::projectOgreViewportOnGroundPlane(
    Ogre::Viewport* viewport, int pixel_x, int pixel_y) {
  Ogre::Plane plane(Ogre::Vector3::UNIT_Z, 0.f);
  return projectOgreViewportOnPlane(viewport, pixel_x, pixel_y, plane);
}

std::pair<bool, QVector3D> ViewportProjectionFinder::projectOgreViewportOnPlane(
    Ogre::Viewport* viewport, int pixel_x, int pixel_y, Ogre::Plane& plane) {
  if (viewport == nullptr) {
    return {false, {}};
  }
  const int width = static_cast<int>(viewport->getActualWidth());
  const int height = static_cast<int>(viewport->getActualHeight());
  if (width <= 0 || height <= 0) {
    return {false, {}};
  }
  const Ogre::Ray mouse_ray = viewport->getCamera()->getCameraToViewportRay(
      static_cast<float>(pixel_x) / static_cast<float>(width),
      static_cast<float>(pixel_y) / static_cast<float>(height));
  const auto intersection = mouse_ray.intersects(plane);
  if (!intersection.first) {
    return {false, {}};
  }
  return {true, toQVector3(mouse_ray.getPoint(intersection.second))};
}

#endif

}  // namespace rendering
}  // namespace autoviz
