/*
 * Copyright 2026 The Openbot Authors
 */

#include "autonomy/manipulation/collision/aabb_collision_detector.hpp"

#include <cmath>
#include <unordered_map>
#include <unordered_set>

#include "autolink/plugin_manager/plugin_manager.hpp"

namespace autonomy {
namespace manipulation {
namespace collision {
namespace {

struct Vec3 {
  double x = 0.0;
  double y = 0.0;
  double z = 0.0;
};

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

bool SphereAabbOverlap(const Vec3& c, double r, const scene::CollisionObject& o) {
  double hx = 0.5 * o.size_x;
  double hy = 0.5 * o.size_y;
  double hz = 0.5 * o.size_z;
  if (o.type == scene::ShapeType::kSphere) {
    const double dx = c.x - o.x;
    const double dy = c.y - o.y;
    const double dz = c.z - o.z;
    const double rr = r + o.size_x;
    return dx * dx + dy * dy + dz * dz <= rr * rr;
  }
  if (o.type == scene::ShapeType::kCylinder) {
    // Upright cylinder along Z: radius=size_x, height=size_z.
    hx = o.size_x;
    hy = o.size_x;
    hz = 0.5 * o.size_z;
  }
  // box / mesh AABB / cylinder AABB
  const double qx = std::max(o.x - hx, std::min(c.x, o.x + hx));
  const double qy = std::max(o.y - hy, std::min(c.y, o.y + hy));
  const double qz = std::max(o.z - hz, std::min(c.z, o.z + hz));
  const double dx = c.x - qx;
  const double dy = c.y - qy;
  const double dz = c.z - qz;
  return dx * dx + dy * dy + dz * dz <= r * r;
}

}  // namespace

bool AabbCollisionDetector::Init(const std::string& id) {
  id_ = id.empty() ? "aabb" : id;
  return true;
}

CollisionResult AabbCollisionDetector::CheckRobotWorld(
    const core::JointState& state,
    const scene::PlanningScene& scene) const {
  CollisionResult result;
  const Vec3 ee = EstimateEe(state, link_length_);
  for (const auto& obj : scene.GetCollisionObjects()) {
    if (SphereAabbOverlap(ee, ee_radius_, obj)) {
      result.collision = true;
      result.contact_body_a = "ee";
      result.contact_body_b = obj.id;
      return result;
    }
  }
  return result;
}

CollisionResult AabbCollisionDetector::CheckRobotSelf(
    const core::JointState& state) const {
  CollisionResult result;
  if (!link_tree_) {
    return result;
  }
  std::unordered_map<std::string, core::Transform> poses;
  if (!link_tree_->Compute(state, &poses) || poses.size() < 2) {
    return result;
  }

  std::unordered_set<std::string> adjacent;
  for (const auto& j : link_tree_->Joints()) {
    adjacent.insert(j.parent + "|" + j.child);
    adjacent.insert(j.child + "|" + j.parent);
  }

  std::vector<std::string> names;
  names.reserve(poses.size());
  for (const auto& kv : poses) {
    names.push_back(kv.first);
  }
  for (std::size_t i = 0; i < names.size(); ++i) {
    for (std::size_t j = i + 1; j < names.size(); ++j) {
      if (adjacent.count(names[i] + "|" + names[j])) {
        continue;
      }
      const auto& a = poses.at(names[i]);
      const auto& b = poses.at(names[j]);
      const double dx = a.x - b.x;
      const double dy = a.y - b.y;
      const double dz = a.z - b.z;
      const double rr = 2.0 * link_radius_;
      if (dx * dx + dy * dy + dz * dz <= rr * rr) {
        result.collision = true;
        result.contact_body_a = names[i];
        result.contact_body_b = names[j];
        return result;
      }
    }
  }
  return result;
}

AUTOLINK_PLUGIN_MANAGER_REGISTER_PLUGIN(AabbCollisionDetector, CollisionDetector);

}  // namespace collision
}  // namespace manipulation
}  // namespace autonomy
