/*
 * Copyright 2026 The Openbot Authors
 */

#include "autonomy/manipulation/scene/simple_planning_scene.hpp"

#include <cmath>
#include <mutex>
#include <unordered_map>

#include "autonomy/manipulation/collision/collision_detector.hpp"

namespace autonomy {
namespace manipulation {
namespace scene {
namespace {

struct Vec3 {
  double x = 0.0;
  double y = 0.0;
  double z = 0.0;
};

Vec3 EstimateEe(const core::JointState& state, double link_length = 0.3) {
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

/** Temporary scene view that overlays attached world poses into GetCollisionObjects. */
class OverlayScene : public PlanningScene {
 public:
  OverlayScene(const PlanningScene* base, std::vector<CollisionObject> objects)
      : base_(base), objects_(std::move(objects)) {}

  void SetCurrentState(const core::JointState& state) override {
    (void)state;
  }
  core::JointState GetCurrentState() const override {
    return base_->GetCurrentState();
  }
  void AddCollisionObject(const CollisionObject&) override {}
  void RemoveCollisionObject(const std::string&) override {}
  void AttachObject(const AttachedCollisionObject&) override {}
  void DetachObject(const std::string&) override {}
  std::vector<AttachedCollisionObject> GetAttachedObjects() const override {
    return {};
  }
  bool IsStateValid(const core::JointState& state) const override {
    return !CheckCollision(state);
  }
  bool CheckCollision(const core::JointState&) const override { return false; }
  bool IsPathValid(const core::RobotTrajectory&) const override { return true; }
  std::vector<CollisionObject> GetCollisionObjects() const override {
    return objects_;
  }
  void SetAllowedCollision(const std::string&, const std::string&,
                           bool) override {}
  bool IsCollisionAllowed(const std::string& a,
                          const std::string& b) const override {
    return base_->IsCollisionAllowed(a, b);
  }
  void SetCollisionDetector(
      std::shared_ptr<collision::CollisionDetector>) override {}
  void SetLinkTree(std::shared_ptr<const core::LinkFkTree>) override {}
  void SetOccupiedPoints(std::vector<OccupiedPoint>, double) override {}
  void ClearOccupiedPoints() override {}
  void ClearWorldObjects() override {}
  void ClearAttachedObjects() override {}
  std::vector<OccupiedPoint> OccupiedPoints() const override {
    return base_->OccupiedPoints();
  }
  double OccupancyResolution() const override {
    return base_->OccupancyResolution();
  }

 private:
  const PlanningScene* base_;
  std::vector<CollisionObject> objects_;
};

}  // namespace

void SimplePlanningScene::SetCurrentState(const core::JointState& state) {
  std::unique_lock lock(mutex_);
  current_ = state;
}

core::JointState SimplePlanningScene::GetCurrentState() const {
  std::shared_lock lock(mutex_);
  return current_;
}

void SimplePlanningScene::AddCollisionObject(const CollisionObject& object) {
  CollisionObject copy = object;
  if (!copy.mesh_vertices.empty()) {
    UpdateMeshAabb(&copy);
  }
  std::unique_lock lock(mutex_);
  objects_[copy.id] = std::move(copy);
  attached_.erase(copy.id);
}

void SimplePlanningScene::RemoveCollisionObject(const std::string& id) {
  std::unique_lock lock(mutex_);
  objects_.erase(id);
}

void SimplePlanningScene::AttachObject(const AttachedCollisionObject& attached) {
  std::unique_lock lock(mutex_);
  objects_.erase(attached.object.id);
  attached_[attached.object.id] = attached;
  for (const auto& touch : attached.touch_links) {
    acm_[attached.link_name].insert(touch);
    acm_[touch].insert(attached.link_name);
    acm_[attached.object.id].insert(touch);
    acm_[touch].insert(attached.object.id);
  }
  acm_[attached.link_name].insert(attached.object.id);
  acm_[attached.object.id].insert(attached.link_name);
}

void SimplePlanningScene::DetachObject(const std::string& object_id) {
  std::unique_lock lock(mutex_);
  const auto it = attached_.find(object_id);
  if (it == attached_.end()) {
    return;
  }
  CollisionObject world = it->second.object;
  // Leave at last known link-relative pose; caller may transform.
  objects_[object_id] = std::move(world);
  attached_.erase(it);
}

std::vector<AttachedCollisionObject> SimplePlanningScene::GetAttachedObjects()
    const {
  std::shared_lock lock(mutex_);
  std::vector<AttachedCollisionObject> out;
  out.reserve(attached_.size());
  for (const auto& kv : attached_) {
    out.push_back(kv.second);
  }
  return out;
}

std::vector<CollisionObject> SimplePlanningScene::WorldPlusAttached(
    const core::JointState& state) const {
  std::vector<CollisionObject> objects;
  objects.reserve(objects_.size() + attached_.size());
  for (const auto& kv : objects_) {
    objects.push_back(kv.second);
  }
  std::unordered_map<std::string, core::Transform> poses;
  const bool have_fk = link_tree_ && link_tree_->Compute(state, &poses);
  for (const auto& kv : attached_) {
    CollisionObject world = kv.second.object;
    if (have_fk) {
      const auto pit = poses.find(kv.second.link_name);
      if (pit != poses.end()) {
        world = TransformAttached(kv.second.object, pit->second);
      }
    } else {
      const Vec3 ee = EstimateEe(state);
      world.x += ee.x;
      world.y += ee.y;
      world.z += ee.z;
    }
    world.id = kv.second.object.id;
    objects.push_back(std::move(world));
  }
  return objects;
}

bool SimplePlanningScene::CheckOccupancy(const core::JointState& state) const {
  std::vector<OccupiedPoint> occupied;
  double resolution = 0.05;
  {
    std::shared_lock lock(mutex_);
    if (occupied_.empty()) {
      return false;
    }
    occupied = occupied_;
    resolution = occupancy_resolution_;
  }
  const Vec3 ee = EstimateEe(state);
  const double r = std::max(0.02, resolution * 0.75);
  const double rr = r * r;
  for (const auto& p : occupied) {
    const double dx = ee.x - p.x;
    const double dy = ee.y - p.y;
    const double dz = ee.z - p.z;
    if (dx * dx + dy * dy + dz * dz <= rr) {
      return true;
    }
  }
  return false;
}

bool SimplePlanningScene::IsStateValid(const core::JointState& state) const {
  return !CheckCollision(state);
}

PlanningScene::CollisionInfo SimplePlanningScene::CheckCollisionDetailed(
    const core::JointState& state) const {
  CollisionInfo info;
  if (CheckOccupancy(state)) {
    info.collision = true;
    info.contact_body_a = "robot";
    info.contact_body_b = "occupancy";
    return info;
  }
  std::shared_ptr<collision::CollisionDetector> detector;
  std::vector<CollisionObject> objects;
  {
    std::shared_lock lock(mutex_);
    detector = detector_;
    objects = WorldPlusAttached(state);
  }
  if (!detector) {
    return info;
  }
  OverlayScene overlay(this, std::move(objects));
  const auto world = detector->CheckRobotWorld(state, overlay);
  if (world.collision &&
      !IsCollisionAllowed(world.contact_body_a, world.contact_body_b)) {
    info.collision = true;
    info.contact_body_a = world.contact_body_a;
    info.contact_body_b = world.contact_body_b;
    return info;
  }
  const auto self = detector->CheckRobotSelf(state);
  if (self.collision &&
      !IsCollisionAllowed(self.contact_body_a, self.contact_body_b)) {
    info.collision = true;
    info.contact_body_a = self.contact_body_a;
    info.contact_body_b = self.contact_body_b;
  }
  return info;
}

bool SimplePlanningScene::CheckCollision(const core::JointState& state) const {
  return CheckCollisionDetailed(state).collision;
}

bool SimplePlanningScene::IsPathValid(
    const core::RobotTrajectory& trajectory) const {
  for (const auto& wp : trajectory.waypoints) {
    if (CheckCollision(wp)) {
      return false;
    }
  }
  return true;
}

std::vector<CollisionObject> SimplePlanningScene::GetCollisionObjects() const {
  std::shared_lock lock(mutex_);
  std::vector<CollisionObject> out;
  out.reserve(objects_.size());
  for (const auto& kv : objects_) {
    out.push_back(kv.second);
  }
  return out;
}

void SimplePlanningScene::SetAllowedCollision(const std::string& a,
                                              const std::string& b,
                                              bool allowed) {
  std::unique_lock lock(mutex_);
  if (allowed) {
    acm_[a].insert(b);
    acm_[b].insert(a);
  } else {
    if (acm_.count(a)) {
      acm_[a].erase(b);
    }
    if (acm_.count(b)) {
      acm_[b].erase(a);
    }
  }
}

bool SimplePlanningScene::IsCollisionAllowed(const std::string& a,
                                             const std::string& b) const {
  std::shared_lock lock(mutex_);
  const auto it = acm_.find(a);
  if (it == acm_.end()) {
    return false;
  }
  return it->second.count(b) > 0;
}

void SimplePlanningScene::SetCollisionDetector(
    std::shared_ptr<collision::CollisionDetector> detector) {
  std::unique_lock lock(mutex_);
  detector_ = std::move(detector);
}

void SimplePlanningScene::SetLinkTree(
    std::shared_ptr<const core::LinkFkTree> tree) {
  std::unique_lock lock(mutex_);
  link_tree_ = std::move(tree);
}

void SimplePlanningScene::SetOccupiedPoints(std::vector<OccupiedPoint> points,
                                            double resolution) {
  std::unique_lock lock(mutex_);
  occupied_ = std::move(points);
  occupancy_resolution_ = resolution > 1e-6 ? resolution : 0.05;
}

void SimplePlanningScene::ClearOccupiedPoints() {
  std::unique_lock lock(mutex_);
  occupied_.clear();
}

void SimplePlanningScene::ClearWorldObjects() {
  std::unique_lock lock(mutex_);
  objects_.clear();
}

void SimplePlanningScene::ClearAttachedObjects() {
  std::unique_lock lock(mutex_);
  attached_.clear();
}

std::vector<OccupiedPoint> SimplePlanningScene::OccupiedPoints() const {
  std::shared_lock lock(mutex_);
  return occupied_;
}

double SimplePlanningScene::OccupancyResolution() const {
  std::shared_lock lock(mutex_);
  return occupancy_resolution_;
}

}  // namespace scene
}  // namespace manipulation
}  // namespace autonomy
