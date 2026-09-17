/*
 * Copyright 2026 The Openbot Authors
 */

#include "autonomy/manipulation/motion/collision/aabb_collision_detector.hpp"

#include <cmath>
#include <unordered_map>
#include <unordered_set>

#include "autolink/plugin_manager/plugin_manager.hpp"
#include "autonomy/manipulation/motion/scene/collision_object_helpers.hpp"

#include <automsgs/msgs/shape_msgs/solid_primitive.pb.h>

namespace autonomy {
namespace manipulation {
namespace collision {
namespace {

struct Vec3 {
  double x = 0.0;
  double y = 0.0;
  double z = 0.0;
};

Vec3 EstimateEndEffectorPosition(const automsgs::msgs::sensor_msgs::JointState& state, double link_length) {
  double x = 0.0;
  double y = 0.0;
  double yaw = 0.0;
  for (double q : state.position()) {
    yaw += q;
    x += link_length * std::cos(yaw);
    y += link_length * std::sin(yaw);
  }
  return {x, y, 0.0};
}

bool SphereAabbOverlap(const Vec3& c, double r, const automsgs::msgs::moveit_msgs::CollisionObject& o) {
  using SP = automsgs::msgs::shape_msgs::SolidPrimitive;
  const automsgs::msgs::geometry_msgs::Pose pose = scene::GetObjectPose(o);
  const double ox = pose.position().x();
  const double oy = pose.position().y();
  const double oz = pose.position().z();
  double sx = 0.0;
  double sy = 0.0;
  double sz = 0.0;
  scene::GetPrimitiveSizes(o, &sx, &sy, &sz);
  double hx = 0.5 * sx;
  double hy = 0.5 * sy;
  double hz = 0.5 * sz;
  if (scene::GetPrimitiveType(o) == SP::SPHERE) {
    const double dx = c.x - ox;
    const double dy = c.y - oy;
    const double dz = c.z - oz;
    const double rr = r + sx;
    return dx * dx + dy * dy + dz * dz <= rr * rr;
  }
  if (scene::GetPrimitiveType(o) == SP::CYLINDER) {
    // Upright cylinder along Z: radius=sx, height=sz.
    hx = sx;
    hy = sx;
    hz = 0.5 * sz;
  }
  // box / mesh AABB / cylinder AABB
  const double qx = std::max(ox - hx, std::min(c.x, ox + hx));
  const double qy = std::max(oy - hy, std::min(c.y, oy + hy));
  const double qz = std::max(oz - hz, std::min(c.z, oz + hz));
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

::autonomy::manipulation::proto::CollisionResult AabbCollisionDetector::CheckRobotWorld(
    const automsgs::msgs::sensor_msgs::JointState& state,
    const scene::PlanningScene& scene) const {
  ::autonomy::manipulation::proto::CollisionResult result;
  const Vec3 ee = EstimateEndEffectorPosition(state, link_length_);
  for (const auto& obj : scene.GetCollisionObjects()) {
    if (SphereAabbOverlap(ee, end_effector_radius_, obj)) {
      result.set_collision(true);
      result.set_contact_body_a("ee");
      result.set_contact_body_b(obj.id());
      return result;
    }
  }
  return result;
}

::autonomy::manipulation::proto::CollisionResult AabbCollisionDetector::CheckRobotSelf(
    const automsgs::msgs::sensor_msgs::JointState& state,
    const scene::PlanningScene* scene) const {
  ::autonomy::manipulation::proto::CollisionResult result;
  if (!link_tree_) {
    return result;
  }
  std::unordered_map<std::string, automsgs::msgs::geometry_msgs::Pose> poses;
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
  const double rr = 2.0 * (link_radius_ + contact_padding_m_);
  for (std::size_t i = 0; i < names.size(); ++i) {
    for (std::size_t j = i + 1; j < names.size(); ++j) {
      if (adjacent.count(names[i] + "|" + names[j])) {
        continue;
      }
      if (scene && scene->IsCollisionAllowed(names[i], names[j])) {
        continue;
      }
      const auto& a = poses.at(names[i]);
      const auto& b = poses.at(names[j]);
      const double dx = a.x - b.x;
      const double dy = a.y - b.y;
      const double dz = a.z - b.z;
      if (dx * dx + dy * dy + dz * dz <= rr * rr) {
        result.set_collision(true);
        result.set_contact_body_a(names[i]);
        result.set_contact_body_b(names[j]);
        return result;
      }
    }
  }
  return result;
}

::autonomy::manipulation::proto::DistanceResult AabbCollisionDetector::DistanceRobotWorld(
    const automsgs::msgs::sensor_msgs::JointState& state,
    const scene::PlanningScene& scene) const {
  ::autonomy::manipulation::proto::DistanceResult best;
  best.set_distance(1e9);
  const double r = end_effector_radius_ + contact_padding_m_;
  auto dist_to_obj = [&](const Vec3& c, const automsgs::msgs::moveit_msgs::CollisionObject& o) {
    using SP = automsgs::msgs::shape_msgs::SolidPrimitive;
    const automsgs::msgs::geometry_msgs::Pose pose = scene::GetObjectPose(o);
    const double ox = pose.position().x();
    const double oy = pose.position().y();
    const double oz = pose.position().z();
    double sx = 0.0;
    double sy = 0.0;
    double sz = 0.0;
    scene::GetPrimitiveSizes(o, &sx, &sy, &sz);
    if (scene::GetPrimitiveType(o) == SP::SPHERE) {
      const double dx = c.x - ox;
      const double dy = c.y - oy;
      const double dz = c.z - oz;
      return std::sqrt(dx * dx + dy * dy + dz * dz) - sx - r;
    }
    const double hx = 0.5 * std::max(sx, 1e-4);
    const double hy = 0.5 * std::max(sy, 1e-4);
    const double hz = 0.5 * std::max(sz, 1e-4);
    const double qx = std::max(ox - hx, std::min(c.x, ox + hx));
    const double qy = std::max(oy - hy, std::min(c.y, oy + hy));
    const double qz = std::max(oz - hz, std::min(c.z, oz + hz));
    const double dx = c.x - qx;
    const double dy = c.y - qy;
    const double dz = c.z - qz;
    return std::sqrt(dx * dx + dy * dy + dz * dz) - r;
  };

  if (link_tree_) {
    std::unordered_map<std::string, automsgs::msgs::geometry_msgs::Pose> poses;
    if (link_tree_->Compute(state, &poses)) {
      for (const auto& kv : poses) {
        const Vec3 c{kv.second.x, kv.second.y, kv.second.z};
        for (const auto& obj : scene.GetCollisionObjects()) {
          if (scene.IsCollisionAllowed(kv.first, obj.id())) {
            continue;
          }
          const double d = dist_to_obj(c, obj);
          if (d < best.distance()) {
            best.set_distance(d);
            best.set_nearest_body_a(kv.first);
            best.set_nearest_body_b(obj.id());
            best.set_nearest_point_x(c.x);
            best.set_nearest_point_y(c.y);
            best.set_nearest_point_z(c.z);
          }
        }
      }
    }
  } else {
    const Vec3 ee = EstimateEndEffectorPosition(state, link_length_);
    for (const auto& obj : scene.GetCollisionObjects()) {
      const double d = dist_to_obj(ee, obj);
      if (d < best.distance()) {
        best.set_distance(d);
        best.set_nearest_body_a("ee");
        best.set_nearest_body_b(obj.id());
        best.set_nearest_point_x(ee.x);
        best.set_nearest_point_y(ee.y);
        best.set_nearest_point_z(ee.z);
      }
    }
  }
  best.set_collision(best.distance() <= 0.0);
  return best;
}

AUTOLINK_PLUGIN_MANAGER_REGISTER_PLUGIN(AabbCollisionDetector,
                                        common::CollisionInterface);

}  // namespace collision
}  // namespace manipulation
}  // namespace autonomy
