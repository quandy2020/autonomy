/*
 * Copyright 2026 The Openbot Authors
 */

#include "autonomy/manipulation/dispatch/capability/capability.hpp"

#include <fstream>
#include <string>

#include "autolink/plugin_manager/plugin_manager.hpp"
#include "autonomy/manipulation/planner/constraint_samplers/constraint_samplers.hpp"
#include "autonomy/manipulation/motion/scene/geometry_io.hpp"
#include "autonomy/manipulation/manipulation_server.hpp"

namespace autonomy {
namespace manipulation {
namespace server {

bool PlanCapability::Init(ManipulationServer* server) {
  server_ = server;
  return server_ != nullptr;
}

planning::MotionPlanResponse PlanCapability::Plan(
    const planning::MotionPlanRequest& req) {
  return server_->Plan(req);
}

bool ExecuteCapability::Init(ManipulationServer* server) {
  server_ = server;
  return server_ != nullptr;
}

ErrorCode ExecuteCapability::Execute(const core::RobotTrajectory& trajectory) {
  return server_->ExecuteTrajectory(trajectory);
}

bool PlanAndExecuteCapability::Init(ManipulationServer* server) {
  server_ = server;
  return server_ != nullptr;
}

planning::MotionPlanResponse PlanAndExecuteCapability::Run(
    const planning::MotionPlanRequest& req) {
  auto response = server_->Plan(req);
  if (!response.success) {
    return response;
  }
  const ErrorCode code = server_->ExecuteTrajectory(response.trajectory);
  if (code != ErrorCode::kSuccess) {
    response.success = false;
    response.error_code = code;
    response.error = ErrorCodeName(code);
  }
  return response;
}

bool CartesianCapability::Init(ManipulationServer* server) {
  server_ = server;
  return server_ != nullptr;
}

planning::MotionPlanResponse CartesianCapability::Plan(
    const planning::MotionPlanRequest& req) {
  planning::MotionPlanRequest r = req;
  r.planner_id = "cartesian";
  return server_->Plan(r);
}

bool FkIkCapability::Init(ManipulationServer* server) {
  server_ = server;
  return server_ != nullptr;
}

bool FkIkCapability::ComputeFk(const core::JointState& joints,
                               kinematics::Pose* pose) const {
  return server_ && server_->kinematics() &&
         server_->kinematics()->GetPositionFK(joints, pose);
}

ErrorCode FkIkCapability::ComputeIk(const kinematics::Pose& pose,
                                    const core::JointState& seed,
                                    core::JointState* solution) const {
  if (!server_ || !server_->kinematics()) {
    return ErrorCode::kNoIkSolution;
  }
  return server_->kinematics()->GetPositionIK(pose, seed, {}, solution);
}

bool GetPlanningSceneCapability::Init(ManipulationServer* server) {
  server_ = server;
  return server_ != nullptr;
}

std::shared_ptr<scene::PlanningScene> GetPlanningSceneCapability::Scene()
    const {
  if (!server_ || !server_->scene_monitor()) {
    return nullptr;
  }
  return server_->scene_monitor()->Scene();
}

bool ApplyPlanningSceneCapability::Init(ManipulationServer* server) {
  server_ = server;
  return server_ != nullptr;
}

bool ApplyPlanningSceneCapability::Apply(const scene::SceneDiff& diff) {
  return server_ && server_->scene_monitor() &&
         server_->scene_monitor()->ApplySceneDiff(diff);
}

bool StateValidationCapability::Init(ManipulationServer* server) {
  server_ = server;
  return server_ != nullptr;
}

StateValidationResult StateValidationCapability::CheckState(
    const core::JointState& state,
    const planning::MotionPlanRequest& constraints) const {
  StateValidationResult out;
  if (!server_ || !server_->scene()) {
    out.error_code = ErrorCode::kInvalidPlanningScene;
    out.error = "no planning scene";
    return out;
  }
  if (!server_->scene()->IsStateValid(state)) {
    const auto info = server_->scene()->CheckCollisionDetailed(state);
    out.error_code = ErrorCode::kStartStateInCollision;
    out.error = "state in collision";
    out.contact_body_a = info.contact_body_a;
    out.contact_body_b = info.contact_body_b;
    return out;
  }
  const planning::MotionPlanRequest& req = constraints;
  if (!constraint_samplers::SatisfiesJointConstraints(req, state)) {
    out.error_code = ErrorCode::kStartStateViolatesPathConstraints;
    out.error = "state violates joint constraints";
    return out;
  }
  auto* kin = server_->kinematics();
  if (kin &&
      (!req.position_constraints.empty() ||
       !req.orientation_constraints.empty())) {
    kinematics::Pose tip;
    if (kin->GetPositionFK(state, &tip)) {
      for (const auto& c : req.position_constraints) {
        if (!constraint_samplers::SatisfiesPositionConstraint(c, tip)) {
          out.error_code = ErrorCode::kStartStateViolatesPathConstraints;
          out.error = "state violates position constraint";
          return out;
        }
      }
      for (const auto& c : req.orientation_constraints) {
        if (!constraint_samplers::SatisfiesOrientationConstraint(c, tip)) {
          out.error_code = ErrorCode::kStartStateViolatesPathConstraints;
          out.error = "state violates orientation constraint";
          return out;
        }
      }
    }
  }
  out.valid = true;
  out.error_code = ErrorCode::kSuccess;
  return out;
}

std::vector<StateValidationResult> StateValidationCapability::CheckStates(
    const std::vector<core::JointState>& states,
    const planning::MotionPlanRequest& constraints) const {
  std::vector<StateValidationResult> results;
  results.reserve(states.size());
  for (const auto& s : states) {
    results.push_back(CheckState(s, constraints));
  }
  return results;
}

bool QueryPlannersCapability::Init(ManipulationServer* server) {
  server_ = server;
  return server_ != nullptr;
}

std::vector<PlannerInterfaceInfo> QueryPlannersCapability::ListPlanners()
    const {
  struct Entry {
    const char* id;
    const char* desc;
  };
  static const Entry kIds[] = {
      {"joint_interpolation", "Linear joint-space interpolation"},
      {"cartesian", "Cartesian linear path via stepwise IK"},
      {"rrt_connect", "In-tree joint-space RRT-Connect"},
      {"ompl", "OMPL via OmplInterface + ompl_planning.conf"},
      {"ompl_interface", "OmplInterfacePlanner (configs + Context)"},
      {"pilz_ptp", "Pilz PTP cosine joint motion"},
      {"pilz_lin", "Pilz LIN Cartesian straight line"},
      {"pilz_circ", "Pilz CIRC circular arc"},
      {"pilz_sequence", "Pilz Sequence (PTP/LIN/CIRC + blend)"},
      {"chomp", "CHOMP covariant gradient optimizer"},
      {"stomp", "STOMP correlated-noise optimizer"},
      {"hybrid", "OMPL/RRT seed + CHOMP polish"},
  };
  std::vector<PlannerInterfaceInfo> out;
  for (const auto& e : kIds) {
    PlannerInterfaceInfo info;
    info.name = e.id;
    info.description = e.desc;
    out.push_back(info);
  }
  return out;
}

std::unordered_map<std::string, std::string>
QueryPlannersCapability::GetPlannerParams(const std::string& planner_id) const {
  std::unordered_map<std::string, std::string> params;
  if (!server_) {
    return params;
  }
  const auto& opt = server_->options();
  params["planner_id"] =
      planner_id.empty() ? opt.planner_id() : planner_id;
  params["planning_time_ms"] = std::to_string(opt.planning_time_ms());
  params["max_velocity"] = std::to_string(opt.max_velocity());
  params["max_acceleration"] = std::to_string(opt.max_acceleration());
  params["interpolation_steps"] = std::to_string(opt.interpolation_steps());
  params["max_attempts"] = std::to_string(opt.max_planning_attempts());
  return params;
}

bool QueryPlannersCapability::SetPlannerParams(
    const std::string& planner_id,
    const std::unordered_map<std::string, std::string>& params) {
  if (!server_) {
    return false;
  }
  return server_->SetPlannerParams(planner_id, params);
}

bool ClearOctomapCapability::Init(ManipulationServer* server) {
  server_ = server;
  return server_ != nullptr;
}

bool ClearOctomapCapability::Clear() {
  return server_ && server_->scene_monitor() &&
         server_->scene_monitor()->ClearOctomap();
}

bool ClearSceneCapability::Init(ManipulationServer* server) {
  server_ = server;
  return server_ != nullptr;
}

bool ClearSceneCapability::Clear(bool clear_attached) {
  return server_ && server_->scene_monitor() &&
         server_->scene_monitor()->ClearScene(clear_attached);
}

bool ValidateTrajectoryCapability::Init(ManipulationServer* server) {
  server_ = server;
  return server_ != nullptr;
}

ErrorCode ValidateTrajectoryCapability::Validate(
    const core::RobotTrajectory& trajectory,
    const planning::MotionPlanRequest& constraints) const {
  if (!server_ || !server_->scene()) {
    return ErrorCode::kInvalidPlanningScene;
  }
  if (!server_->scene()->IsPathValid(trajectory)) {
    return ErrorCode::kInvalidMotionPlan;
  }
  planning::MotionPlanRequest req = constraints;
  req.kinematics = server_->SharedKinematics();
  if (!constraint_samplers::SatisfiesPathConstraints(req, trajectory)) {
    return ErrorCode::kInvalidMotionPlan;
  }
  return ErrorCode::kSuccess;
}

AUTOLINK_PLUGIN_MANAGER_REGISTER_PLUGIN(PlanCapability, Capability);
AUTOLINK_PLUGIN_MANAGER_REGISTER_PLUGIN(ExecuteCapability, Capability);
AUTOLINK_PLUGIN_MANAGER_REGISTER_PLUGIN(PlanAndExecuteCapability, Capability);
AUTOLINK_PLUGIN_MANAGER_REGISTER_PLUGIN(CartesianCapability, Capability);
AUTOLINK_PLUGIN_MANAGER_REGISTER_PLUGIN(FkIkCapability, Capability);
AUTOLINK_PLUGIN_MANAGER_REGISTER_PLUGIN(GetPlanningSceneCapability, Capability);
AUTOLINK_PLUGIN_MANAGER_REGISTER_PLUGIN(ApplyPlanningSceneCapability,
                                        Capability);
AUTOLINK_PLUGIN_MANAGER_REGISTER_PLUGIN(StateValidationCapability, Capability);
AUTOLINK_PLUGIN_MANAGER_REGISTER_PLUGIN(QueryPlannersCapability, Capability);
AUTOLINK_PLUGIN_MANAGER_REGISTER_PLUGIN(ClearOctomapCapability, Capability);
AUTOLINK_PLUGIN_MANAGER_REGISTER_PLUGIN(ClearSceneCapability, Capability);
AUTOLINK_PLUGIN_MANAGER_REGISTER_PLUGIN(ValidateTrajectoryCapability,
                                        Capability);

bool GetUrdfCapability::Init(ManipulationServer* server) {
  server_ = server;
  return server_ != nullptr;
}

std::string GetUrdfCapability::UrdfPath() const {
  return server_ ? server_->UrdfPath() : std::string();
}

std::string GetUrdfCapability::UrdfXml() const {
  const std::string path = UrdfPath();
  if (path.empty()) {
    return {};
  }
  std::ifstream in(path);
  if (!in) {
    return {};
  }
  return std::string((std::istreambuf_iterator<char>(in)),
                     std::istreambuf_iterator<char>());
}

bool SaveLoadGeometryCapability::Init(ManipulationServer* server) {
  server_ = server;
  return server_ != nullptr;
}

bool SaveLoadGeometryCapability::Save(const std::string& path) const {
  return server_ && server_->scene() &&
         scene::SaveGeometryToFile(*server_->scene(), path);
}

bool SaveLoadGeometryCapability::Load(const std::string& path) {
  return server_ && server_->scene() &&
         scene::LoadGeometryFromFile(server_->scene(), path);
}

bool GetDynamicsCapability::Init(ManipulationServer* server) {
  server_ = server;
  return server_ != nullptr;
}

bool GetDynamicsCapability::HasPinocchio() const {
  return dynamics::HasPinocchioDynamics();
}

bool GetDynamicsCapability::GetGravityTorques(
    const std::vector<double>& positions,
    std::vector<double>* torques) const {
  if (!server_ || !server_->dynamics_solver()) {
    return false;
  }
  return server_->dynamics_solver()->GetGravityTorques(positions, torques);
}

bool GetDynamicsCapability::GetCoriolisTorques(
    const std::vector<double>& positions,
    const std::vector<double>& velocities,
    std::vector<double>* torques) const {
  if (!server_ || !server_->dynamics_solver()) {
    return false;
  }
  return server_->dynamics_solver()->GetCoriolisTorques(positions, velocities,
                                                        torques);
}

AUTOLINK_PLUGIN_MANAGER_REGISTER_PLUGIN(GetUrdfCapability, Capability);
AUTOLINK_PLUGIN_MANAGER_REGISTER_PLUGIN(SaveLoadGeometryCapability, Capability);
AUTOLINK_PLUGIN_MANAGER_REGISTER_PLUGIN(GetDynamicsCapability, Capability);

}  // namespace server
}  // namespace manipulation
}  // namespace autonomy
