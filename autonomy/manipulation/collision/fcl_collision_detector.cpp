/*
 * Copyright 2026 The Openbot Authors
 *
 * FCL-backed world collision for box/sphere scene objects.
 */

#include "autonomy/manipulation/collision/fcl_collision_detector.hpp"

#include <cmath>
#include <memory>
#include <string>
#include <vector>

#include <fcl/fcl.h>

#include "autolink/plugin_manager/plugin_manager.hpp"
#include "autonomy/common/logging.hpp"

namespace autonomy {
namespace manipulation {
namespace collision {
namespace {

struct Vec3 {
  double x = 0.0;
  double y = 0.0;
  double z = 0.0;
};

/** Same planar EE proxy as AABB detector for consistent bring-up. */
Vec3 EstimateEe(const core::JointState& state, double link_length) {
  double x = 0.0;
  double y = 0.0;
  double yaw = 0.0;
  for (double q : state.positions) {
    yaw += q;
    x += link_length * std::cos(yaw);
    y += link_length * std::sin(yaw);
  }
  return {x, y, 0.0};
}

fcl::CollisionObjectd MakeObject(const scene::CollisionObject& obj) {
  std::shared_ptr<fcl::CollisionGeometryd> geom;
  if (obj.type == scene::ShapeType::kSphere) {
    geom = std::make_shared<fcl::Sphered>(std::max(1e-4, obj.size_x));
  } else if (obj.type == scene::ShapeType::kCylinder) {
    geom = std::make_shared<fcl::Cylinderd>(std::max(1e-4, obj.size_x),
                                            std::max(1e-4, obj.size_z));
  } else {
    // box and mesh (AABB proxy)
    geom = std::make_shared<fcl::Boxd>(std::max(1e-4, obj.size_x),
                                      std::max(1e-4, obj.size_y),
                                      std::max(1e-4, obj.size_z));
  }
  fcl::Transform3d tf = fcl::Transform3d::Identity();
  tf.translation() = fcl::Vector3d(obj.x, obj.y, obj.z);
  return fcl::CollisionObjectd(geom, tf);
}

}  // namespace

bool FclCollisionDetector::Init(const std::string& id) {
  id_ = id.empty() ? "fcl" : id;
  AINFO << "FclCollisionDetector ready id=" << id_;
  return true;
}

CollisionResult FclCollisionDetector::CheckRobotWorld(
    const core::JointState& state,
    const scene::PlanningScene& scene) const {
  CollisionResult result;
  const Vec3 ee = EstimateEe(state, link_length_);
  auto sphere = std::make_shared<fcl::Sphered>(ee_radius_);
  fcl::Transform3d ee_tf = fcl::Transform3d::Identity();
  ee_tf.translation() = fcl::Vector3d(ee.x, ee.y, ee.z);
  fcl::CollisionObjectd ee_obj(sphere, ee_tf);

  for (const auto& obj : scene.GetCollisionObjects()) {
    fcl::CollisionObjectd world = MakeObject(obj);
    fcl::CollisionRequestd req;
    fcl::CollisionResultd res;
    fcl::collide(&ee_obj, &world, req, res);
    if (res.isCollision()) {
      result.collision = true;
      result.contact_body_a = "ee";
      result.contact_body_b = obj.id;
      return result;
    }
  }
  return result;
}

CollisionResult FclCollisionDetector::CheckRobotSelf(
    const core::JointState& /*state*/) const {
  return {};
}

std::shared_ptr<CollisionDetector> CreateFclCollisionDetector() {
  return std::make_shared<FclCollisionDetector>();
}

AUTOLINK_PLUGIN_MANAGER_REGISTER_PLUGIN(FclCollisionDetector, CollisionDetector);

}  // namespace collision
}  // namespace manipulation
}  // namespace autonomy
