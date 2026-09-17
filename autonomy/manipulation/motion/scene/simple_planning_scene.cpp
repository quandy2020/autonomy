/*
 * Copyright 2026 The Openbot Authors
 */

#include "autonomy/manipulation/motion/scene/simple_planning_scene.hpp"

#include <algorithm>
#include <cmath>
#include <mutex>
#include <unordered_map>

#include "autonomy/manipulation/common/collision_interface.hpp"
#include "autonomy/manipulation/model/joint_state_utilities.hpp"

namespace autonomy {
namespace manipulation {
namespace scene {
namespace {

struct Vec3 {
  double x = 0.0;
  double y = 0.0;
  double z = 0.0;
};

Vec3 EstimateEndEffectorPosition(const automsgs::msgs::sensor_msgs::JointState& state, double link_length = 0.3) {
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

/** Temporary scene view that overlays attached world poses into GetCollisionObjects. */
class OverlayScene : public PlanningScene {
 public:
  OverlayScene(const PlanningScene* base, std::vector<automsgs::msgs::moveit_msgs::CollisionObject> objects)
      : base_(base), objects_(std::move(objects)) {}

  void SetCurrentState(const automsgs::msgs::sensor_msgs::JointState& state) override {
    (void)state;
  }
  automsgs::msgs::sensor_msgs::JointState GetCurrentState() const override {
    return base_->GetCurrentState();
  }
  void AddCollisionObject(const automsgs::msgs::moveit_msgs::CollisionObject&) override {}
  void RemoveCollisionObject(const std::string&) override {}
  void AttachObject(const automsgs::msgs::moveit_msgs::AttachedCollisionObject&) override {}
  void DetachObject(const std::string&) override {}
  std::vector<automsgs::msgs::moveit_msgs::AttachedCollisionObject> GetAttachedObjects() const override {
    return {};
  }
  bool IsStateValid(const automsgs::msgs::sensor_msgs::JointState& state) const override {
    return !CheckCollision(state);
  }
  bool CheckCollision(const automsgs::msgs::sensor_msgs::JointState&) const override { return false; }
  bool IsPathValid(const automsgs::msgs::trajectory_msgs::JointTrajectory&) const override { return true; }
  std::vector<automsgs::msgs::moveit_msgs::CollisionObject> GetCollisionObjects() const override {
    return objects_;
  }
  void SetAllowedCollision(const std::string&, const std::string&,
                           bool) override {}
  bool IsCollisionAllowed(const std::string& a,
                          const std::string& b) const override {
    return base_->IsCollisionAllowed(a, b);
  }
  automsgs::msgs::moveit_msgs::AllowedCollisionMatrix GetAllowedCollisionMatrix() const override {
    return base_->GetAllowedCollisionMatrix();
  }
  void SetAllowedCollisionMatrix(const automsgs::msgs::moveit_msgs::AllowedCollisionMatrix&) override {}
  void SetCollisionDetector(
      std::shared_ptr<common::CollisionInterface>) override {}
  void SetLinkTree(std::shared_ptr<const model::LinkForwardKinematicsTree>) override {}
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
  std::vector<automsgs::msgs::moveit_msgs::CollisionObject> objects_;
};

}  // namespace

void SimplePlanningScene::SetCurrentState(const automsgs::msgs::sensor_msgs::JointState& state) {
  std::unique_lock lock(mutex_);
  current_ = state;
}

automsgs::msgs::sensor_msgs::JointState SimplePlanningScene::GetCurrentState() const {
  std::shared_lock lock(mutex_);
  return current_;
}

void SimplePlanningScene::AddCollisionObject(const automsgs::msgs::moveit_msgs::CollisionObject& object) {
  automsgs::msgs::moveit_msgs::CollisionObject copy = object;
  if (HasMesh(copy)) {
    UpdateMeshAabb(&copy);
  }
  const std::string id = copy.id();
  std::unique_lock lock(mutex_);
  objects_[id] = std::move(copy);
  attached_.erase(id);
}

void SimplePlanningScene::RemoveCollisionObject(const std::string& id) {
  std::unique_lock lock(mutex_);
  objects_.erase(id);
}

void SimplePlanningScene::AttachObject(const automsgs::msgs::moveit_msgs::AttachedCollisionObject& attached) {
  std::unique_lock lock(mutex_);
  const std::string& oid = attached.object().id();
  objects_.erase(oid);
  attached_[oid] = attached;
  for (const auto& touch : attached.touch_links()) {
    acm_[attached.link_name()].insert(touch);
    acm_[touch].insert(attached.link_name());
    acm_[oid].insert(touch);
    acm_[touch].insert(oid);
  }
  acm_[attached.link_name()].insert(oid);
  acm_[oid].insert(attached.link_name());
}

void SimplePlanningScene::DetachObject(const std::string& object_id) {
  std::unique_lock lock(mutex_);
  const auto it = attached_.find(object_id);
  if (it == attached_.end()) {
    return;
  }
  automsgs::msgs::moveit_msgs::CollisionObject world = it->second.object();
  // Leave at last known link-relative pose; caller may transform.
  objects_[object_id] = std::move(world);
  attached_.erase(it);
}

std::vector<automsgs::msgs::moveit_msgs::AttachedCollisionObject> SimplePlanningScene::GetAttachedObjects()
    const {
  std::shared_lock lock(mutex_);
  std::vector<automsgs::msgs::moveit_msgs::AttachedCollisionObject> out;
  out.reserve(attached_.size());
  for (const auto& kv : attached_) {
    out.push_back(kv.second);
  }
  return out;
}

std::vector<automsgs::msgs::moveit_msgs::CollisionObject> SimplePlanningScene::GetWorldAndAttachedObjects(
    const automsgs::msgs::sensor_msgs::JointState& state) const {
  std::vector<automsgs::msgs::moveit_msgs::CollisionObject> objects;
  objects.reserve(objects_.size() + attached_.size());
  for (const auto& kv : objects_) {
    objects.push_back(kv.second);
  }
  std::unordered_map<std::string, automsgs::msgs::geometry_msgs::Pose> poses;
  const bool have_fk = link_tree_ && link_tree_->Compute(state, &poses);
  for (const auto& kv : attached_) {
    automsgs::msgs::moveit_msgs::CollisionObject world = kv.second.object();
    if (have_fk) {
      const auto pit = poses.find(kv.second.link_name());
      if (pit != poses.end()) {
        world = TransformAttached(kv.second.object(), pit->second);
      }
    } else {
      const Vec3 ee = EstimateEndEffectorPosition(state);
      TranslateObject(&world, ee.x, ee.y, ee.z);
    }
    world.set_id(kv.second.object().id());
    objects.push_back(std::move(world));
  }
  return objects;
}

bool SimplePlanningScene::CheckOccupancy(const automsgs::msgs::sensor_msgs::JointState& state) const {
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
  const Vec3 ee = EstimateEndEffectorPosition(state);
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

bool SimplePlanningScene::IsStateValid(const automsgs::msgs::sensor_msgs::JointState& state) const {
  return !CheckCollision(state);
}

PlanningScene::CollisionInfo SimplePlanningScene::CheckCollisionDetailed(
    const automsgs::msgs::sensor_msgs::JointState& state) const {
  CollisionInfo info;
  if (CheckOccupancy(state)) {
    info.set_collision(true);
    info.set_contact_body_a("robot");
    info.set_contact_body_b("occupancy");
    return info;
  }
  std::shared_ptr<common::CollisionInterface> detector;
  std::vector<automsgs::msgs::moveit_msgs::CollisionObject> objects;
  {
    std::shared_lock lock(mutex_);
    detector = detector_;
    objects = GetWorldAndAttachedObjects(state);
  }
  if (!detector) {
    return info;
  }
  OverlayScene overlay(this, std::move(objects));
  const auto world = detector->CheckRobotWorld(state, overlay);
  if (world.collision() &&
      !IsCollisionAllowed(world.contact_body_a(), world.contact_body_b())) {
    info.set_collision(true);
    info.set_contact_body_a(world.contact_body_a());
    info.set_contact_body_b(world.contact_body_b());
    return info;
  }
  const auto self = detector->CheckRobotSelf(state, this);
  if (self.collision() &&
      !IsCollisionAllowed(self.contact_body_a(), self.contact_body_b())) {
    info.set_collision(true);
    info.set_contact_body_a(self.contact_body_a());
    info.set_contact_body_b(self.contact_body_b());
  }
  return info;
}

bool SimplePlanningScene::CheckCollision(const automsgs::msgs::sensor_msgs::JointState& state) const {
  return CheckCollisionDetailed(state).collision();
}

PlanningScene::DistanceInfo SimplePlanningScene::DistanceRobotWorld(
    const automsgs::msgs::sensor_msgs::JointState& state) const {
  DistanceInfo info;
  std::shared_ptr<common::CollisionInterface> detector;
  std::vector<automsgs::msgs::moveit_msgs::CollisionObject> objects;
  {
    std::shared_lock lock(mutex_);
    detector = detector_;
    objects = GetWorldAndAttachedObjects(state);
  }
  if (!detector) {
    info.set_collision(CheckCollision(state));
    info.set_distance(info.collision() ? 0.0 : 1e3);
    return info;
  }
  OverlayScene overlay(this, std::move(objects));
  const auto d = detector->DistanceRobotWorld(state, overlay);
  info.set_distance(d.distance());
  info.set_collision(d.collision());
  info.set_nearest_body_a(d.nearest_body_a());
  info.set_nearest_body_b(d.nearest_body_b());
  info.set_nearest_point_x(d.nearest_point_x());
  info.set_nearest_point_y(d.nearest_point_y());
  info.set_nearest_point_z(d.nearest_point_z());
  return info;
}

bool SimplePlanningScene::IsPathValid(
    const automsgs::msgs::trajectory_msgs::JointTrajectory& trajectory) const {
  for (int i = 0; i < trajectory.points_size(); ++i) {
    if (!IsStateValid(MakeJointStateFromPoint(trajectory, i))) {
      return false;
    }
  }
  return true;
}

bool SimplePlanningScene::IsPathValidDense(
    const automsgs::msgs::trajectory_msgs::JointTrajectory& trajectory, int segments_per_edge) const {
  if (!IsPathValid(trajectory)) {
    return false;
  }
  const int segs = std::max(1, segments_per_edge);
  for (int i = 0; i + 1 < trajectory.points_size(); ++i) {
    const automsgs::msgs::sensor_msgs::JointState a = MakeJointStateFromPoint(trajectory, i);
    const automsgs::msgs::sensor_msgs::JointState b = MakeJointStateFromPoint(trajectory, i + 1);
    const int dof = std::min(a.position_size(), b.position_size());
    for (int s = 1; s < segs; ++s) {
      const double t = static_cast<double>(s) / static_cast<double>(segs);
      automsgs::msgs::sensor_msgs::JointState mid = a;
      for (int j = 0; j < dof; ++j) {
        mid.set_position(j, a.position(j) + t * (b.position(j) - a.position(j)));
      }
      if (!IsStateValid(mid)) {
        return false;
      }
    }
  }
  return true;
}

std::shared_ptr<const model::LinkForwardKinematicsTree> SimplePlanningScene::GetLinkTree()
    const {
  std::shared_lock lock(mutex_);
  return link_tree_;
}

std::vector<automsgs::msgs::moveit_msgs::CollisionObject> SimplePlanningScene::GetCollisionObjects() const {
  std::shared_lock lock(mutex_);
  std::vector<automsgs::msgs::moveit_msgs::CollisionObject> out;
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

automsgs::msgs::moveit_msgs::AllowedCollisionMatrix SimplePlanningScene::GetAllowedCollisionMatrix() const {
  std::shared_lock lock(mutex_);
  automsgs::msgs::moveit_msgs::AllowedCollisionMatrix matrix;
  std::vector<std::string> names;
  names.reserve(acm_.size());
  for (const auto& kv : acm_) {
    names.push_back(kv.first);
  }
  std::sort(names.begin(), names.end());
  for (const auto& n : names) {
    matrix.add_entry_names(n);
  }
  const int n = static_cast<int>(names.size());
  for (int i = 0; i < n; ++i) {
    auto* row = matrix.add_entry_values();
    for (int j = 0; j < n; ++j) {
      const auto it = acm_.find(names[static_cast<std::size_t>(i)]);
      const bool enabled =
          it != acm_.end() &&
          it->second.count(names[static_cast<std::size_t>(j)]) > 0;
      row->add_enabled(enabled);
    }
  }
  return matrix;
}

void SimplePlanningScene::SetAllowedCollisionMatrix(
    const automsgs::msgs::moveit_msgs::AllowedCollisionMatrix& matrix) {
  std::unique_lock lock(mutex_);
  acm_.clear();
  const int n = matrix.entry_names_size();
  for (int i = 0; i < n; ++i) {
    if (i >= matrix.entry_values_size()) {
      break;
    }
    const auto& row = matrix.entry_values(i);
    for (int j = 0; j < n && j < row.enabled_size(); ++j) {
      if (!row.enabled(j)) {
        continue;
      }
      const std::string& a = matrix.entry_names(i);
      const std::string& b = matrix.entry_names(j);
      acm_[a].insert(b);
      acm_[b].insert(a);
    }
  }
  for (int i = 0; i < matrix.default_entry_names_size(); ++i) {
    if (i >= matrix.default_entry_values_size()) {
      break;
    }
    if (!matrix.default_entry_values(i)) {
      continue;
    }
    // default_entry: allow against all known names.
    const std::string& a = matrix.default_entry_names(i);
    for (int j = 0; j < n; ++j) {
      const std::string& b = matrix.entry_names(j);
      acm_[a].insert(b);
      acm_[b].insert(a);
    }
  }
}

void SimplePlanningScene::SetCollisionDetector(
    std::shared_ptr<common::CollisionInterface> detector) {
  std::unique_lock lock(mutex_);
  detector_ = std::move(detector);
}

void SimplePlanningScene::SetLinkTree(
    std::shared_ptr<const model::LinkForwardKinematicsTree> tree) {
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
